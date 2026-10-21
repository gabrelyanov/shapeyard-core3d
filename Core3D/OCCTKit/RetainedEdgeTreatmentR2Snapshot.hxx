#pragma once

#include "RetainedEdgeTreatmentR2Values.hxx"
#include "RetainedEdgeTreatmentSnapshot.hxx"
#include "RetainedFaceSelector.hxx"
#include "RetainedEdgeTreatmentBuild.hxx" // SourceRebindRoles handoff
#include <AIS_Shape.hxx>
#include <TDF_Label.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_TShape.hxx>
#include <TopLoc_Location.hxx>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>

class OcctDocument;
namespace core3d { class Core3DViewer; class RawR2BevelPreviewAdapter; }

namespace core3d::retained_face_selector {
// NP6 operation-bound proof sharing: the sealed exact-input certificate the
// source/stage proof issuer mints alongside its ephemeral context. It binds
// the exact canonical source/prefix digest, source/root/owner identities,
// schema and unit bits, the document-time nonce and the exact proved
// detached source/base identity. Shape identity alone is not immutability:
// transport also requires the private producer and one-use operation below.
// It is a mathematical proof value, not document authority: private, nonpersistent, never encoded
// and never staged on the document; no OCAF handle crosses the detached
// boundary. The commit owner validates its native lease before consumption;
// any changed input or staged binding refuses instead of reissuing.
class SourceStageCertificate final {
    friend class SourceStageProofIssuer;
    SourceStageCertificate() = default;
    // No caller can manufacture a seal, copy its mutable OCCT handles, or
    // obtain a source/base alias from this mathematical proof value.
    retained_edge_treatment::r2::BooleanBaseBinding binding;
    std::vector<std::uint8_t> canonicalPrefixBytes, invariantBytes;
    retained_edge_treatment::SourceStageContext issuedContext;
    std::uint64_t nonce = 0;
    bool initialIssuance = false;
    retained_edge_treatment::r2::Digest prefixDigest{};
    retained_edge_treatment::r2::UUID document, entity, definition, sourceFeature, sourceNode;
    std::uint32_t sourceSchema = 0;
    std::uint64_t unitBits = 0;
    Handle(TopoDS_TShape) sourceBaseShape, savedBaseShape;
    TopLoc_Location sourceBaseLocation, savedBaseLocation;
    TopAbs_Orientation sourceBaseOrientation = TopAbs_EXTERNAL;
    TopAbs_Orientation savedBaseOrientation = TopAbs_EXTERNAL;
public:
    ~SourceStageCertificate() = default;
    SourceStageCertificate(const SourceStageCertificate&) = delete;
    SourceStageCertificate& operator=(const SourceStageCertificate&) = delete;
};
} // namespace core3d::retained_face_selector

namespace core3d::retained_edge_treatment::r2 {
struct SelectorAppend { et::Kind kind = et::Kind::ConstantFillet; double amountMM = 0; };
struct RawAppend { et::Kind kind = et::Kind::ConstantFillet; double amountMM = 0; };
using Edit = std::variant<et::SetAmount, et::Remove, SelectorAppend, RawAppend, RebuildBooleanInput,
                          RebuildAnalyticTool, SetBooleanOperation, SetInputPlacement,
                          et::SetSelectorIntent>;
// R2 replay accounting shares the common per-operation topology counter with
// B1 (same frozen limits, validation-first arithmetic, sticky refusal, DEBUG
// trace) and keeps the R2-only envelope/identity dimensions untouched.
struct ReplayBudget : core3d::retained_topology_budget::Counter {
    std::size_t envelopeBytes = 0, featureIdentities = 0;
};
struct LegacyStepSelectorBinding { std::uint64_t oldStepID = 0; fs::SelectorIntent intent; };

class Snapshot final {
    friend class ::OcctDocument; friend class core3d::Core3DViewer;
    TDF_Label ownerLabel_, sourceLabel_;
    rr::OwnerSnapshot owner_;
    BaseRecipe source_;
    Definition definition_;
    std::vector<std::uint8_t> sourceBytes_, definitionBytes_;
    // The proven pre-Boolean source base. An admitted prefix source edit
    // rebuilds the complete prefix from this shape, never from the treated
    // current shape.
    TopoDS_Shape sourceBase_;
    TopoDS_Shape base_, current_;
    ReplayBudget chargedBudget_;
    std::uint64_t nonce_ = 0, presentationRevision_ = 0;
public:
    const rr::OwnerSnapshot& owner() const noexcept { return owner_; }
    const BaseRecipe& source() const noexcept { return source_; }
    const Definition& definition() const noexcept { return definition_; }
    const std::vector<std::uint8_t>& canonicalBytes() const noexcept { return definitionBytes_; }
    bool current() const noexcept { return owner_.status == rr::OwnerStatus::CurrentEditable && !current_.IsNull(); }
    double dimensionMetersPerUnit() const noexcept {
        return std::visit([](const auto& base) { return base.metersPerLocalUnit; }, definition_.base);
    }
};

class MigrationCapture final {
    friend class ::OcctDocument; friend class core3d::Core3DViewer;
    TDF_Label ownerLabel_, sourceLabel_;
    rr::OwnerSnapshot owner_;
    retained_boolean::Recipe original_;
    std::vector<std::uint8_t> originalBytes_;
    TopoDS_Shape originalBase_, originalCurrent_;
    ReplayBudget chargedBudget_;
    std::uint64_t nonce_ = 0, presentationRevision_ = 0;
public:
    const rr::OwnerSnapshot& owner() const noexcept { return owner_; }
    const retained_boolean::Recipe& source() const noexcept { return original_; }
    const std::vector<std::uint8_t>& canonicalOriginalBytes() const noexcept { return originalBytes_; }
};
class MigrationReview final {
    friend class core3d::Core3DViewer;
    std::shared_ptr<const MigrationCapture> original_;
    Digest requestDigest_{};
    MigrationM3 request_;
    ReplayBudget chargedBudget_;
    // proofs_[index] is the resolved membership proof for
    // request_.selectors[index], resolved on the replay stage immediately
    // before that legacy step; appendProof_ belongs to the optional append and
    // is resolved on the post-existing-treatment stage.
    std::vector<std::shared_ptr<const fs::FaceMembershipProof>> proofs_;
    std::shared_ptr<const fs::FaceMembershipProof> appendProof_;
    // Reviewed stage values (copied shapes only, no document authority): the
    // true untreated post-Boolean base — never originalCurrent_ when the
    // recipe carries a legacy tail — and the post-existing-treatment stage.
    TopoDS_Shape postBooleanBase_, postTreatmentStage_;
public:
    const Digest& requestDigest() const noexcept { return requestDigest_; }
    const std::vector<std::shared_ptr<const fs::FaceMembershipProof>>& proofs() const noexcept { return proofs_; }
    const std::shared_ptr<const fs::FaceMembershipProof>& appendProof() const noexcept { return appendProof_; }
};
// A1 first enrollment: the read-only capture of an A1-composite RetainedBoolean
// owner that holds no R2 record yet, and the reviewed first append (selector
// proof resolved against the exact untreated carrier shape). The composite
// carrier is the enrollment authority and is never restaged by this lane.
class EnrollmentCapture final {
    friend class ::OcctDocument; friend class core3d::Core3DViewer;
    TDF_Label ownerLabel_, sourceLabel_;
    rr::OwnerSnapshot owner_;
    composite_recipe::Definition original_;
    std::vector<std::uint8_t> originalBytes_;
    TopoDS_Shape originalCurrent_;
    ReplayBudget chargedBudget_;
    std::uint64_t nonce_ = 0, presentationRevision_ = 0;
public:
    const rr::OwnerSnapshot& owner() const noexcept { return owner_; }
    const composite_recipe::Definition& source() const noexcept { return original_; }
    const std::vector<std::uint8_t>& canonicalOriginalBytes() const noexcept { return originalBytes_; }
};
class EnrollmentReview final {
    friend class core3d::Core3DViewer;
    std::shared_ptr<const EnrollmentCapture> original_;
    Digest requestDigest_{};
    SelectorAppendIntent request_;
    std::shared_ptr<const fs::FaceMembershipProof> proof_;
    ReplayBudget chargedBudget_;
};
class SelectorTargetCapture final {
    friend class core3d::Core3DViewer;
    std::shared_ptr<const Snapshot> original_;
    std::shared_ptr<const fs::FaceMembershipProof> proof_;
    ReplayBudget chargedBudget_;
public:
    const std::shared_ptr<const fs::FaceMembershipProof>& selectorProof() const noexcept { return proof_; }
};

// Native-only authority for a raw R2 append. The selected edges are measured
// on the captured current R2 stage and retained as values; document authority,
// owner presentation and the complete operation budget remain on this object.
// The bridge may inspect the immutable values, but only Core3DViewer can bind
// the exact Object-mode handoff or consume the capture.
class RawTargetCapture final {
    friend class core3d::Core3DViewer;
    friend class core3d::RawR2BevelPreviewAdapter;
    std::shared_ptr<const Snapshot> original_;
    std::vector<et::Anchor> anchors_;
    ReplayBudget chargedBudget_;
    Handle(AIS_Shape) presentation_;
    std::string entityIdentifier_, publicationSourceIdentifier_;
    std::uint64_t documentGeneration_ = 0, modelRevision_ = 0;
    std::uint64_t capturePresentationRevision_ = 0;
    mutable std::atomic_bool objectModeBound_{false};
    mutable std::atomic_bool consumed_{false};
public:
    const std::shared_ptr<const Snapshot>& snapshot() const noexcept { return original_; }
    const std::vector<et::Anchor>& anchors() const noexcept { return anchors_; }
};

class Work;
class DetachedInput;
class DetachedResult;
class RawPreviewCandidate;
// Native-only command ownership. The session keeps the consumed capture and
// its fences; public anchor observations can neither construct nor copy it.
class RawPreviewSession final {
    friend class core3d::Core3DViewer;
    friend class core3d::RawR2BevelPreviewAdapter;
    friend class RawPreviewCandidate;
    std::shared_ptr<const RawTargetCapture> targets_;
    std::string entityIdentifier_, publicationSourceIdentifier_;
    std::uint64_t documentGeneration_ = 0, modelRevision_ = 0;
    std::uint64_t presentationRevision_ = 0, nextGeneration_ = 1;
    std::uint32_t width_ = 0, height_ = 0;
    std::shared_ptr<RawPreviewCandidate> active_, pending_;
    mutable std::mutex mutex_;
public:
    std::shared_ptr<const Snapshot> snapshot() const noexcept {
        return targets_ ? targets_->snapshot() : nullptr;
    }
};

// Each candidate owns a distinct Work. Its sticky capture/prepare/build debt
// is never reset when the candidate is cancelled, superseded, or observed.
class RawPreviewCandidate final {
    friend class core3d::Core3DViewer;
    friend class core3d::RawR2BevelPreviewAdapter;
public:
    enum class State { Prepared, Pending, Building, Ready, Superseded, Cancelled, Refused };
private:
    std::weak_ptr<RawPreviewSession> session_;
    std::shared_ptr<Work> work_;
    std::shared_ptr<const DetachedInput> input_;
    std::shared_ptr<const DetachedResult> result_;
    ReplayBudget observedBudget_;
    std::uint64_t generation_ = 0;
    et::Kind kind_ = et::Kind::ConstantFillet;
    double amountMM_ = 0;
    std::atomic<State> state_{State::Prepared};
    bool workerInFlight_ = false; // guarded by the owning session mutex
public:
    std::uint64_t generation() const noexcept { return generation_; }
    State state() const noexcept { return state_.load(); }
    et::Kind kind() const noexcept { return kind_; }
    double amountMM() const noexcept { return amountMM_; }
    std::shared_ptr<const DetachedResult> result() const noexcept {
        const auto session=session_.lock();if(!session)return {};
        std::lock_guard<std::mutex> lock(session->mutex_);return result_;
    }
};

// Private source-edit lease: one producer, one detached result and one
// synchronous command consumption. Document time may legitimately be zero;
// this native identity, not a nonzero timestamp, prevents operation reuse.
// No document or OCAF object crosses the detached boundary.
class SourceStageOperation final {
    friend class core3d::Core3DViewer;
    enum class State { Prepared, Building, Ready, Committing, Staging, Spent };
    std::atomic<State> state_{State::Prepared};
    std::shared_ptr<std::atomic_bool> cancelled_;
    explicit SourceStageOperation(std::shared_ptr<std::atomic_bool> cancelled)
        : cancelled_(std::move(cancelled)) {}
    bool advance(State from, State to) noexcept {
        return cancelled_ && !cancelled_->load()
            && state_.compare_exchange_strong(from, to);
    }
public:
    bool beginBuild() noexcept { return advance(State::Prepared, State::Building); }
    bool finishBuild() noexcept { return advance(State::Building, State::Ready); }
    bool beginCommit(const std::shared_ptr<std::atomic_bool>& cancelled) noexcept {
        return cancelled == cancelled_ && advance(State::Ready, State::Committing);
    }
    bool beginStage() noexcept { return advance(State::Committing, State::Staging); }
    bool activeForReadback() const noexcept {
        return cancelled_ && !cancelled_->load() && state_.load() == State::Staging;
    }
    void expire() noexcept { state_.store(State::Spent); }
};

class DetachedInput final {
    friend class core3d::Core3DViewer;
    std::shared_ptr<SourceStageOperation> sourceStageOperation_;
    std::uint64_t nonce_ = 0;
    Definition candidate_;
    BaseRecipe source_;
    // The admitted edit and the proven pre-Boolean source base travel across
    // the detached boundary as values/shapes only; no document authority.
    std::optional<Edit> edit_;
    TopoDS_Shape sourceBase_;
    TopoDS_Shape base_, originalCurrent_;
    // Old-stage selector roles captured on the main-thread authority side for
    // an admitted Boolean-input edit; the detached build rebinds moved
    // witnesses through the unique role correspondence before suffix replay.
    std::shared_ptr<const et::SourceRebindRoles> rebindRoles_;
    ReplayBudget chargedBudget_;
    std::shared_ptr<std::atomic_bool> cancelled_;
};
class DetachedResult final {
    friend class ::OcctDocument; friend class core3d::Core3DViewer;
    DetachedResult() = default;
    DetachedResult(const DetachedResult&) = delete;
    DetachedResult& operator=(const DetachedResult&) = delete;
    std::shared_ptr<SourceStageOperation> sourceStageOperation_;
    std::uint64_t nonce_ = 0;
    Definition definition_;
    BaseRecipe source_;
    std::optional<Edit> edit_;
    std::vector<std::uint8_t> definitionBytes_, prefixBytes_;
    // Set only when an admitted source edit rebuilt the prefix: the restaged
    // pre-Boolean source base and a marker the staging path requires before
    // it may touch the retained source carrier.
    TopoDS_Shape editedSourceBase_;
    bool sourceChanged_ = false;
    TopoDS_Shape base_, result_;
    ReplayBudget budget_;
    // NP6: the sealed operation-bound source/stage proof, issued once by the
    // detached build and handed to the matching commit stage for bound
    // exact-input consumption. Private and nonpersistent: never encoded,
    // never staged on the document, and no OCAF handle crosses the detached
    // boundary. The commit owner validates the certificate's native lease
    // (canonical bytes/digest, identities, schema/unit bits, per-feature
    // intent binding, proved-shape ownership and the operation nonce) before
    // reuse; any mismatch refuses instead of reissuing.
    std::shared_ptr<const et::SourceStageContext> boundSourceContext_;
    std::shared_ptr<const core3d::retained_face_selector::SourceStageCertificate>
        boundSourceCertificate_;
    // The proven edited source base is commit-stage authority under the
    // existing friend boundary; it is no longer a public accessor.
    const TopoDS_Shape& editedSourceBase() const noexcept { return editedSourceBase_; }
public:
    const Definition& definition() const noexcept { return definition_; }
    const TopoDS_Shape& result() const noexcept { return result_; }
    const std::vector<std::uint8_t>& canonicalPrefixBytes() const noexcept { return prefixBytes_; }
    bool sourceChanged() const noexcept { return sourceChanged_; }
};
class Work final {
    friend class core3d::Core3DViewer;
public: enum class State { Fresh, Prepared, Building, Ready, Committing, Settled };
private:
    std::shared_ptr<const Snapshot> snapshot_;
    std::shared_ptr<const MigrationCapture> migration_;
    std::shared_ptr<const MigrationReview> review_;
    std::shared_ptr<const EnrollmentCapture> enrollment_;
    std::shared_ptr<const RawTargetCapture> rawTargets_;
    std::variant<Edit, MigrationM3> mutation_;
    Definition candidate_;
    BaseRecipe source_;
    TopoDS_Shape sourceBase_;
    std::shared_ptr<const et::SourceRebindRoles> rebindRoles_;
    std::vector<std::uint8_t> prefixBytes_;
    TopoDS_Shape base_, originalCurrent_;
    ReplayBudget chargedBudget_;
    std::uint64_t nonce_ = 0;
    TDF_Label label_; Handle(AIS_Shape) presentation_;
    std::shared_ptr<std::atomic_bool> cancelled_ = std::make_shared<std::atomic_bool>(false);
    State state_ = State::Fresh;
};
struct Record {
    TDF_Label label, owner;
    std::shared_ptr<const Definition> definition;
    std::vector<std::uint8_t> bytes;
    // The exact retained source-carrier bytes staged by the same transaction;
    // empty only when the carrier was not touched.
    std::vector<std::uint8_t> sourceBytes;
    TopoDS_Shape base, current;
};
} // namespace core3d::retained_edge_treatment::r2
