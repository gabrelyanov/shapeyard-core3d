#pragma once

#include "RetainedFeatureReplay.hxx"
#include "RetainedRecipeSnapshot.hxx"
#include <cstring>
#include <cstdint>
#include <memory>
#include <map>
#include <string>
#include <vector>

#include <TDF_Label.hxx>
#include <TDocStd_Document.hxx>

class OcctDocument;

namespace core3d::retained_feature {

enum class MutationKind : std::uint8_t { EditSource = 1, EditFeature = 2, AppendFeature = 3, RemoveFeature = 4 };
enum class OwnerOutcome : std::uint8_t { Rejected, Cancelled, Unchanged, Committed, RecoveryRequired };

struct MutationRequest final {
    MutationKind kind = MutationKind::EditFeature;
    retained_recipe::RecipeLocator target;
    Key codec;
    std::vector<std::uint8_t> typedParameters;
};

struct CompleteFence final {
    retained_recipe::RevisionFence revision;
    retained_recipe::Digest graph{}, output{}, identities{}, metadata{}, history{};
    bool valid() const noexcept {
        return retained_recipe::Valid(revision) && retained_recipe::Nonzero(graph)
            && retained_recipe::Nonzero(output) && retained_recipe::Nonzero(identities)
            && retained_recipe::Nonzero(metadata) && retained_recipe::Nonzero(history);
    }
    bool operator==(const CompleteFence& value) const noexcept {
        return graph == value.graph && output == value.output && identities == value.identities
            && metadata == value.metadata && history == value.history
            && revision.documentGeneration == value.revision.documentGeneration
            && revision.modelRevision == value.revision.modelRevision
            && revision.dependencies == value.revision.dependencies;
    }
};

inline bool InstallCapturedRevision(double effectiveMetersPerUnit,
                                    retained_recipe::RevisionFence revision,
                                    CompleteFence& fence) noexcept {
    std::vector<std::uint8_t> unitBytes;
    std::uint64_t unitBits = 0;
    std::memcpy(&unitBits, &effectiveMetersPerUnit, sizeof(unitBits));
    for (unsigned index = 0; index < 8; ++index)
        unitBytes.push_back(std::uint8_t(unitBits >> (index * 8)));
    if (!composite_recipe::Hash(unitBytes, revision.ownerPlacement)) return false;
    fence.revision = std::move(revision);
    return true;
}

struct PreparedChange final {
    std::uint64_t ownerNonce = 0, session = 0;
    MutationRequest request;
    composite_recipe::Definition graph;
    std::vector<ReplayValue> sourceShapes;
    ReplayResult replay;
    CompleteFence before, expected;
};

struct OwnerReceipt final {
    OwnerOutcome outcome = OwnerOutcome::Rejected;
    std::string reason;
    std::uint64_t session = 0;
    int measuredHistoryDelta = 0;
    bool settled = true;
};

// OCAF access remains behind the document-owned port. Builders and proofs only
// receive detached values through ReplayDetached and therefore cannot mutate a
// document while a candidate is being prepared.
class DocumentPort {
public:
    virtual ~DocumentPort() = default;
    virtual bool capture(CompleteFence&, composite_recipe::Definition&,
                         std::vector<SourceValue>&) noexcept = 0;
    virtual bool stageAndReadBack(const PreparedChange&) noexcept = 0;
    virtual bool commitOneCommand(int& measuredHistoryDelta) noexcept = 0;
    virtual bool abortAndProve(const CompleteFence&) noexcept = 0;
    virtual bool reconcile(const CompleteFence&, const CompleteFence&,
                           OwnerOutcome&) noexcept = 0;
};

class Owner final {
public:
    Owner(DocumentPort& port, std::uint64_t nonce,
          const RegistryView& registry, const retained_source::RegistryView& sources) noexcept
        : port_(port), nonce_(nonce), registry_(registry), sources_(sources) {}

    std::shared_ptr<const PreparedChange> prepare(const MutationRequest& request,
                                                  ReplayBudget budget,
                                                  OwnerReceipt& receipt) noexcept {
        receipt = {};
        try {
            if (nonce_ == 0 || activeSession_ || recoverySession_) {
                receipt.reason = "owner-busy-or-unbound"; return {};
            }
            CompleteFence before, after; composite_recipe::Definition current;
            std::vector<SourceValue> sourceValues;
            if (!port_.capture(before, current, sourceValues) || !before.valid()
                || current.schemaVersion != 3) {
                receipt.reason = "capture-incomplete"; return {};
            }
            const ReplayResult fixed = ReplayDetached(
                current, registry_, sources_, sourceValues, budget);
            if (!fixed.replayed() || fixed.output.geometry != before.output) {
                receipt.reason = fixed.replayed() ? "stored-output-not-fixed" : fixed.reason;
                return {};
            }
            auto prepared = std::make_shared<PreparedChange>();
            prepared->ownerNonce = nonce_; prepared->session = ++nextSession_;
            prepared->request = request; prepared->graph = current; prepared->before = before;
            bool changed = false;
            for (auto& node : prepared->graph.nodes) {
                auto* feature = std::get_if<composite_recipe::FeatureNode>(&node.value);
                if (!feature || feature->node != request.target.node) continue;
                if (request.kind != MutationKind::EditFeature
                    || feature->kind != request.codec.kind
                    || feature->codecVersion != request.codec.codecVersion) {
                    receipt.reason = "typed-request-mismatch"; return {};
                }
                changed = feature->parameters != request.typedParameters;
                feature->parameters = request.typedParameters;
            }
            if (!changed) {
                receipt = {OwnerOutcome::Unchanged, "no-op", 0, 0, true}; return {};
            }
            prepared->replay = ReplayDetached(prepared->graph, registry_, sources_, sourceValues, budget);
            if (!prepared->replay.replayed()) {
                receipt.reason = prepared->replay.reason; return {};
            }
            if (!port_.capture(after, current, sourceValues) || !(after == before)) {
                receipt.reason = "prepare-mutated-or-stale"; return {};
            }
            prepared->expected = before;
            prepared->expected.graph = retained_recipe::Digest{};
            std::vector<std::uint8_t> graphBytes;
            if (!composite_recipe::EncodeV3(prepared->graph, registry_, sources_, graphBytes)
                || !composite_recipe::Hash(graphBytes, prepared->expected.graph)) {
                receipt.reason = "candidate-readback"; return {};
            }
            prepared->expected.output = prepared->replay.output.geometry;
            activeSession_ = prepared->session;
            receipt = {OwnerOutcome::Unchanged, "prepared", activeSession_, 0, true};
            return prepared;
        } catch (...) { receipt.reason = "prepare-exception"; return {}; }
    }

    OwnerReceipt apply(const std::shared_ptr<const PreparedChange>& prepared) noexcept {
        if (!prepared || prepared->ownerNonce != nonce_ || prepared->session != activeSession_)
            return {OwnerOutcome::Rejected, "foreign-prepared-change", activeSession_, 0, true};
        try {
            CompleteFence fence; composite_recipe::Definition graph; std::vector<SourceValue> sources;
            if (!port_.capture(fence, graph, sources) || !(fence == prepared->before))
                return retire(OwnerOutcome::Rejected, "stale-final-fence", 0);
            if (!port_.stageAndReadBack(*prepared)) {
                if (port_.abortAndProve(prepared->before)) return retire(OwnerOutcome::Rejected, "stage-aborted", 0);
                recoverySession_ = activeSession_; activeSession_ = 0;
                return {OwnerOutcome::RecoveryRequired, "stage-outcome-uncertain", recoverySession_, 0, false};
            }
            int delta = 0;
            if (port_.commitOneCommand(delta) && delta == 1)
                return retire(OwnerOutcome::Committed, "committed", delta);
            recoverySession_ = activeSession_; activeSession_ = 0;
            return {OwnerOutcome::RecoveryRequired, "commit-outcome-uncertain", recoverySession_, delta, false};
        } catch (...) {
            if (port_.abortAndProve(prepared->before)) return retire(OwnerOutcome::Rejected, "apply-aborted", 0);
            recoverySession_ = activeSession_; activeSession_ = 0;
            return {OwnerOutcome::RecoveryRequired, "apply-outcome-uncertain", recoverySession_, 0, false};
        }
    }

    OwnerReceipt reconcile(const CompleteFence& before, const CompleteFence& candidate) noexcept {
        if (!recoverySession_) return {OwnerOutcome::Rejected, "no-recovery", 0, 0, true};
        OwnerOutcome outcome = OwnerOutcome::RecoveryRequired;
        if (!port_.reconcile(before, candidate, outcome))
            return {OwnerOutcome::RecoveryRequired, "reconcile-unsettled", recoverySession_, 0, false};
        const auto session = recoverySession_; recoverySession_ = 0;
        return {outcome, outcome == OwnerOutcome::Committed ? "reconciled-candidate" : "reconciled-prior",
                session, outcome == OwnerOutcome::Committed ? 1 : 0, true};
    }

    OwnerReceipt cancel() noexcept {
        if (!activeSession_) return {OwnerOutcome::Rejected, "no-active-session", 0, 0, true};
        return retire(OwnerOutcome::Cancelled, "cancelled", 0);
    }

    bool blocksOtherWork() const noexcept { return activeSession_ != 0 || recoverySession_ != 0; }

private:
    OwnerReceipt retire(OwnerOutcome outcome, const char* reason, int delta) noexcept {
        const auto session = activeSession_; activeSession_ = 0;
        return {outcome, reason, session, delta, true};
    }
    DocumentPort& port_;
    std::uint64_t nonce_ = 0, nextSession_ = 0, activeSession_ = 0, recoverySession_ = 0;
    const RegistryView& registry_;
    const retained_source::RegistryView& sources_;
};

// Document-owned bridge. It is inert for ordinary P1 documents because the
// production table installs no complete v3 execution/editor route.
class OcafOwnerService final {
public:
    explicit OcafOwnerService(OcctDocument& owner,
                              const RegistryView& registry = EffectiveRegistry(),
                              const retained_source::RegistryView& sources
                                  = retained_source::ProductionRegistry()) noexcept;
    ~OcafOwnerService();
    bool boundTo(const Handle(TDocStd_Document)& document) const noexcept;
    bool blocksOtherWork() const noexcept;
    void retireForDocumentReplacement() noexcept;
    std::shared_ptr<const PreparedChange> prepare(
        const TDF_Label& carrier, const MutationRequest& request,
        ReplayBudget budget, OwnerReceipt& receipt) noexcept;
    OwnerReceipt apply(const std::shared_ptr<const PreparedChange>& prepared) noexcept;
    OwnerReceipt cancel() noexcept;
    OwnerReceipt reconcile(const CompleteFence& before,
                           const CompleteFence& candidate) noexcept;
#if DEBUG
    bool installSyntheticFixture(double metersPerUnit, TDF_Label& carrier) noexcept;
    std::map<std::string, bool> debugLifecycle(double metersPerUnit) noexcept;
#endif
private:
    struct State;
    std::unique_ptr<State> state_;
};

} // namespace core3d::retained_feature
