#import <Foundation/Foundation.h>

#include "GeneralLoftEdit.hxx"
#include "GeneralLoftOwner.hxx"
#include "OcctDocument.h"

#include <algorithm>
#include <map>
#include <set>

namespace core3d::general_loft::owner {
namespace {
constexpr std::uint32_t FenceWidth = 64, FenceHeight = 64;

SceneFence SceneOf(const native_opening::Fence& fence) noexcept {
    return {fence.documentGeneration(), fence.modelRevision(), fence.metersPerUnit()};
}

bool SameScene(const SceneFence& expected, const native_opening::Fence& actual) noexcept {
    return expected.documentGeneration == actual.documentGeneration()
        && expected.modelRevision == actual.modelRevision()
        && expected.metersPerUnit == actual.metersPerUnit();
}

bool UUIDFromText(const std::string& text, UUID& output) noexcept {
    output = {};
    if (!profile::IsIdentifier(text)) return false;
    std::size_t index = 0; unsigned nibble = 0;
    for (char value : text) {
        if (value == '-') continue;
        unsigned digit = value >= '0' && value <= '9' ? unsigned(value - '0')
            : value >= 'A' && value <= 'F' ? unsigned(value - 'A' + 10)
            : value >= 'a' && value <= 'f' ? unsigned(value - 'a' + 10) : 16;
        if (digit > 15) return false;
        if ((nibble++ & 1U) == 0) output[index] = std::uint8_t(digit << 4);
        else output[index++] |= std::uint8_t(digit);
    }
    return index == output.size() && bounded_curve::Nonzero(output);
}

bool Issue(UUID& output) noexcept {
    return UUIDFromText(OcctDocument::NewProfileIdentifier(), output);
}

Receipt Refusal(const SceneFence& scene, Outcome outcome, const char* reason) noexcept {
    Receipt receipt; receipt.scene = scene; receipt.outcome = outcome; receipt.reason = reason;
    return receipt;
}

bool AssignCurveDigest(bounded_curve::RetainedState& state) noexcept {
    bounded_curve::Value value{state.authority.feature, state.definition};
    std::vector<std::uint8_t> bytes;
    return bounded_curve::Encode(value, bytes)
        && bounded_curve::Hash(bytes, bounded_curve::MaximumDefinitionBytes,
                               state.authority.recipeDigest);
}

bool IssueDefinition(const retained_recipe::OwnerKey& owner,
                     const CreateRequest& request, Definition& output) noexcept {
    output = {};
    try {
        if (request.stations.size() < MinimumStations
            || request.stations.size() > MaximumStations) return false;
        const std::size_t count = request.stations.front().junctions.size();
        if (count < MinimumSegments || count > MaximumSegments) return false;
        output.owner = owner; output.dimensionMetersPerUnit = request.expectedScene.metersPerUnit;
        output.orderAxis = request.orderAxis;
        if (!Issue(output.feature)) return false;
        std::vector<UUID> junctionCorrespondence(count), segmentCorrespondence(count);
        for (UUID& identifier : junctionCorrespondence) if (!Issue(identifier)) return false;
        for (UUID& identifier : segmentCorrespondence) if (!Issue(identifier)) return false;
        for (std::size_t stationIndex = 0; stationIndex < request.stations.size(); ++stationIndex) {
            const CreationStation& input = request.stations[stationIndex];
            if (input.junctions.size() != count) return false;
            Station station; station.frame = input.frame;
            station.orderParameter = input.orderParameter;
            station.twistFromPreviousRadians = input.twistFromPreviousRadians;
            if (!Issue(station.identifier) || !Issue(station.frame.identifier)) return false;
            station.frame.revision = 1;
            for (std::size_t index = 0; index < count; ++index) {
                Junction junction; junction.correspondence = junctionCorrespondence[index];
                junction.local = input.junctions[index];
                if (!Issue(junction.identifier)) return false;
                station.junctions.push_back(junction);
            }
            for (std::size_t index = 0; index < count; ++index) {
                Segment segment; segment.correspondence = segmentCorrespondence[index];
                segment.startJunction = station.junctions[index].identifier;
                segment.endJunction = station.junctions[(index + 1) % count].identifier;
                if (!Issue(segment.identifier)) return false;
                auto& state = segment.curveState;
                state.authority.owner = owner; state.authority.feature = segment.identifier;
                state.authority.definitionRevision = 1;
                state.authority.frame = station.frame.identifier;
                state.authority.frameRevision = station.frame.revision;
                state.definition.domain = bounded_curve::Domain::Sketch2D;
                state.definition.frame = station.frame; state.definition.degree = 1;
                bounded_curve::ControlPoint a, b;
                if (!Issue(a.identifier) || !Issue(b.identifier)) return false;
                a.local = {{input.junctions[index][0], input.junctions[index][1], 0}};
                const auto& next = input.junctions[(index + 1) % count];
                b.local = {{next[0], next[1], 0}};
                state.definition.controlPoints = {a, b};
                state.definition.knots = {{0, 2}, {1, 2}};
                state.nextLocalID = 3;
                if (!AssignCurveDigest(state) || !bounded_curve::ValidState(state)) return false;
                station.segments.push_back(std::move(segment));
            }
            output.stations.push_back(std::move(station));
        }
        return persistence::AssignCanonicalDigest(output)
            && persistence::Canonical(output);
    } catch (...) { output = {}; return false; }
}
} // namespace

struct OcafOwner::State final {
    struct Session final {
        std::shared_ptr<const Opening> opening;
        OcctIssuedLabelIdentity issued;
        OcctGeneralLoftCapture exact;
    };
    struct PreparedValue final {
        std::shared_ptr<const Prepared> prepared;
        std::uint64_t session = 0;
    };
    OcctDocument* document = nullptr;
    std::shared_ptr<native_opening::Context> context;
    std::uint64_t nextSession = 0, nextPreparation = 0;
    std::map<std::uint64_t, Session> sessions;
    std::map<std::uint64_t, PreparedValue> preparations;
    bool bound() const noexcept {
        return document && context && !document->Document().IsNull()
            && context->openingFence().document() == document->Document()
            && context->openingFence().data() == document->Document()->GetData();
    }
};

OcafOwner::OcafOwner(OcctDocument& document,
                     std::shared_ptr<native_opening::Context> context) noexcept
    : state_(new State()) { state_->document = &document; state_->context = std::move(context); }
OcafOwner::~OcafOwner() = default;

std::shared_ptr<const Opening> OcafOwner::beginCreation(
    const CreateRequest& request, Receipt& receipt) noexcept {
    const SceneFence scene = state_ && state_->context
        ? SceneOf(state_->context->openingFence()) : SceneFence{};
    receipt = Refusal(scene, Outcome::refused, "creation-refused");
    const char *gate = "owner-bound-scene";
    const auto refused = [&]() -> std::shared_ptr<const Opening> {
#if DEBUG
        NSLog(@"R4_C3N refused=%s", gate);
#endif
        return {};
    };
    if (![NSThread isMainThread] || !state_ || !state_->bound()
        || !SameScene(request.expectedScene, state_->context->openingFence())
        || state_->nextSession == UINT64_MAX) return refused();
    try {
        gate = "owner-reserve-identities";
        std::vector<OcctIssuedLabelIdentity> issued;
        if (!state_->document->ReserveExactLabelIdentities(1, {}, issued) || issued.size() != 1)
            return refused();
        gate = "owner-identity-parse";
        retained_recipe::OwnerKey owner;
        if (!UUIDFromText(state_->document->DocumentIdentifier(), owner.document)
            || !UUIDFromText(issued.front().EntityIdentifier(), owner.entity)
            || !UUIDFromText(issued.front().DefinitionIdentifier(), owner.definition)) return refused();
        gate = "owner-issue-definition";
        Definition definition;
        if (!IssueDefinition(owner, request, definition)) return refused();
        std::atomic_bool cancelled{false};
        gate = "owner-detached-build-proof";
        // Same single build/proof sequence as BuildAndProveDetached; diagnostics
        // observe its results and never contribute to admission or receipts.
        const KernelBuild built = BuildDetached(definition, cancelled);
        const AdmittedSolid admitted = ProveDetached(definition, built, cancelled);
#if DEBUG
        NSLog(@"R4_C3N kernel=%u proof=%u stations=%zu metersPerUnit=%.17g",
              unsigned(built.status), unsigned(admitted.proof.status),
              definition.stations.size(), definition.dimensionMetersPerUnit);
#endif
        if (!admitted.admitted()) return refused();
        gate = "owner-opening-publication";
        auto opening = std::make_shared<Opening>();
        opening->scene = request.expectedScene; opening->definition = definition;
        opening->session = ++state_->nextSession; opening->creating = true;
        opening->requestedName = request.requestedName;
        state_->sessions.emplace(opening->session,
            State::Session{opening, issued.front(), {}});
        receipt.outcome = Outcome::captured; receipt.reason = "creation-captured";
        receipt.definition = definition; receipt.session = opening->session;
        return opening;
    } catch (...) { receipt.reason = "creation-exception"; return refused(); }
}

std::shared_ptr<const Opening> OcafOwner::capture(
    const std::string& entityIdentifier, const SceneFence& scene) noexcept {
    if (![NSThread isMainThread] || !state_ || !state_->bound()
        || !SameScene(scene, state_->context->openingFence())
        || state_->nextSession == UINT64_MAX) return {};
    try {
        OcctGeneralLoftCapture exact;
        if (!state_->document->CaptureGeneralLoftExact(
                entityIdentifier, *state_->context, exact)) return {};
        auto opening = std::make_shared<Opening>();
        opening->scene = scene; opening->definition = exact.record.value->definition;
        opening->session = ++state_->nextSession;
        state_->sessions.emplace(opening->session,
            State::Session{opening, {}, std::move(exact)});
        return opening;
    } catch (...) { return {}; }
}

std::shared_ptr<const Prepared> OcafOwner::prepare(
    const std::shared_ptr<const Opening>& opening, const Candidate& candidate,
    Receipt& receipt) noexcept {
    receipt = Refusal(opening ? opening->scene : SceneFence{}, Outcome::refused,
                      "prepare-refused");
    if (![NSThread isMainThread] || !state_ || !state_->bound() || !opening
        || state_->nextPreparation == UINT64_MAX) return {};
    try {
        const auto found = state_->sessions.find(opening->session);
        if (found == state_->sessions.end() || found->second.opening != opening
            || !SameScene(opening->scene, state_->context->openingFence())) return {};
        Definition next = candidate.descriptive;
        // Swift-projected owner/revision/digest are fences only. The native
        // opening is the mutation authority and computes the replacement hash.
        next.owner = opening->definition.owner;
        next.feature = opening->definition.feature;
        next.definitionRevision = opening->definition.definitionRevision;
        next.recipeDigest = opening->definition.recipeDigest;
        if (opening->creating && SameDefinitionValue(next, opening->definition)) {
            std::atomic_bool cancelled{false};
            AdmittedSolid admitted = BuildAndProveDetached(next, cancelled);
            if (!admitted.admitted()) return {};
            auto prepared = std::make_shared<Prepared>();
            prepared->opening = *opening; prepared->candidate = next;
            prepared->admitted = std::move(admitted);
            prepared->preparation = ++state_->nextPreparation;
            state_->preparations.emplace(prepared->preparation,
                State::PreparedValue{prepared, opening->session});
            receipt.outcome = Outcome::prepared; receipt.reason = "creation-prepared";
            receipt.definition = next; receipt.session = opening->session;
            receipt.preparation = prepared->preparation;
            return prepared;
        }
        StationEditProposal proposal;
        proposal.expectedOwner = opening->definition.owner;
        proposal.expectedFeature = opening->definition.feature;
        proposal.expectedDefinitionRevision = opening->definition.definitionRevision;
        proposal.expectedRecipeDigest = opening->definition.recipeDigest;
        proposal.station = candidate.editedStation;
        const auto station = std::find_if(next.stations.begin(), next.stations.end(),
            [&](const Station& value) { return value.identifier == candidate.editedStation; });
        if (station == next.stations.end()) return {};
        proposal.replacement = *station;
        Definition digestCandidate = opening->definition;
        const auto original = std::find_if(digestCandidate.stations.begin(),
            digestCandidate.stations.end(), [&](const Station& value) {
                return value.identifier == candidate.editedStation;
            });
        if (original == digestCandidate.stations.end()) return {};
        *original = *station; ++digestCandidate.definitionRevision;
        if (!persistence::AssignCanonicalDigest(digestCandidate)) return {};
        proposal.replacementRecipeDigest = digestCandidate.recipeDigest;
        const PreparedEdit edit = PrepareStationEdit(opening->definition, proposal);
        if (edit.refusal == EditRefusal::NoChange) {
            receipt.outcome = Outcome::unchanged; receipt.reason = "unchanged"; return {};
        }
        if (edit.refusal != EditRefusal::None
            || !persistence::Canonical(edit.candidate)) return {};
        if (!opening->creating
            && state_->document->HasUnsupportedGeneralLoftDependent(found->second.exact)) {
            receipt.outcome = Outcome::unsupportedDependent;
            receipt.reason = "unsupported-descendant"; return {};
        }
        std::atomic_bool cancelled{false};
        AdmittedSolid admitted = BuildAndProveDetached(edit.candidate, cancelled);
        if (!admitted.admitted()) return {};
        auto prepared = std::make_shared<Prepared>();
        prepared->opening = *opening; prepared->candidate = edit.candidate;
        prepared->admitted = std::move(admitted);
        prepared->preparation = ++state_->nextPreparation;
        state_->preparations.emplace(prepared->preparation,
            State::PreparedValue{prepared, opening->session});
        receipt.outcome = Outcome::prepared; receipt.reason = "prepared";
        receipt.definition = prepared->candidate; receipt.session = opening->session;
        receipt.preparation = prepared->preparation;
        return prepared;
    } catch (...) { receipt.reason = "prepare-exception"; return {}; }
}

Receipt OcafOwner::apply(const std::shared_ptr<const Prepared>& prepared) noexcept {
    const SceneFence scene = prepared ? prepared->opening.scene : SceneFence{};
    if (![NSThread isMainThread] || !state_ || !state_->bound() || !prepared)
        return Refusal(scene, Outcome::refused, "apply-refused");
    try {
        const auto p = state_->preparations.find(prepared->preparation);
        const auto s = p == state_->preparations.end() ? state_->sessions.end()
            : state_->sessions.find(p->second.session);
        if (p == state_->preparations.end() || p->second.prepared != prepared
            || s == state_->sessions.end()
            || !SameScene(scene, state_->context->openingFence()))
            return Refusal(scene, Outcome::staleOwner, "preparation-not-current");
        if (!prepared->opening.creating) {
            OcctGeneralLoftCapture current;
            if (!state_->document->ReadGeneralLoftExact(
                    prepared->opening.definition.owner, current)
                || !s->second.exact.IsEqual(current))
                return Refusal(scene, Outcome::staleDefinition, "opening-changed");
        }
        const int undoBefore = state_->document->Document()->GetAvailableUndos();
        auto lease = state_->context->beginCommandLease(
            state_->context->openingFence(), FenceWidth, FenceHeight);
        if (!lease) return Refusal(scene, Outcome::busy, "command-refused");
        OcctGeneralLoftCapture staged;
        const bool ok = prepared->opening.creating
            ? state_->document->StageGeneralLoftCreate(*lease, s->second.issued,
                prepared->candidate, prepared->admitted,
                prepared->opening.requestedName, staged)
            : state_->document->StageGeneralLoftReplacement(*lease, s->second.exact,
                prepared->candidate, prepared->admitted, staged);
        if (!ok) {
            const bool aborted = lease->abort();
            return Refusal(scene, aborted ? Outcome::refused : Outcome::recoveryRequired,
                           aborted ? "staging-refused" : "abort-unknown");
        }
        if (!lease->commit()) return Refusal(scene, Outcome::outcomeUnknown, "close-unknown");
        OcctGeneralLoftCapture read;
        const int historyDelta = state_->document->Document()->GetAvailableUndos() - undoBefore;
        if (historyDelta != 1 || !state_->document->ReadGeneralLoftExact(
                prepared->candidate.owner, read) || !staged.IsEqual(read))
            return Refusal(scene, Outcome::outcomeUnknown, "post-close-unreadable");
        Receipt receipt; receipt.outcome = Outcome::committed; receipt.reason = "committed";
        receipt.scene = scene; receipt.definition = prepared->candidate;
        receipt.session = prepared->opening.session; receipt.preparation = prepared->preparation;
        receipt.historyDelta = historyDelta;
        state_->preparations.erase(p); state_->sessions.erase(s); return receipt;
    } catch (...) { return Refusal(scene, Outcome::recoveryRequired, "apply-exception"); }
}

Receipt OcafOwner::cancel(std::uint64_t session) noexcept {
    const auto found = state_ ? state_->sessions.find(session) : decltype(state_->sessions)::iterator{};
    if (![NSThread isMainThread] || !state_ || found == state_->sessions.end())
        return Refusal({}, Outcome::refused, "cancel-refused");
    const SceneFence scene = found->second.opening->scene;
    for (auto it = state_->preparations.begin(); it != state_->preparations.end();)
        if (it->second.session == session) it = state_->preparations.erase(it); else ++it;
    state_->sessions.erase(found);
    return Refusal(scene, Outcome::cancelled, "cancelled");
}
} // namespace core3d::general_loft::owner
