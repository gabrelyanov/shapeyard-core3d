#pragma once
#include "BoundedCurveEditProbe.hxx"
#include <map>
#include <string>

namespace core3d::bounded_curve::debug {
inline bool CanonicalizeState(RetainedState& state,
                              std::vector<std::uint8_t>* definitionBytes = nullptr,
                              std::vector<std::uint8_t>* ownerBytes = nullptr) {
    Value value{state.authority.feature, state.definition};
    std::vector<std::uint8_t> definition;
    Digest digest{};
    if (!Encode(value, definition)
        || !core3d::bounded_curve::Hash(definition, MaximumDefinitionBytes, digest)) return false;
    state.authority.recipeDigest = digest;
    OwnerState owner;
    owner.owner = state.authority.owner; owner.feature = state.authority.feature;
    owner.definitionRevision = state.authority.definitionRevision;
    owner.nextLocalID = state.nextLocalID;
    owner.canonicalDefinitionDigest = digest; owner.tombstones = state.tombstones;
    std::vector<std::uint8_t> authority;
    if (!EncodeOwnerState(owner, authority)) return false;
    if (definitionBytes) *definitionBytes = std::move(definition);
    if (ownerBytes) *ownerBytes = std::move(authority);
    return ValidState(state);
}

inline bool ColdRoundTrip(const RetainedState& state, RetainedState& output) {
    output = {};
    std::vector<std::uint8_t> definitionBytes, ownerBytes;
    RetainedState canonical = state;
    if (!CanonicalizeState(canonical, &definitionBytes, &ownerBytes)) return false;
    Value value; OwnerState owner;
    if (!Decode(definitionBytes, value) || !DecodeOwnerState(ownerBytes, owner)) return false;
    return FromPersistedValue({std::move(value), std::move(owner)}, output);
}

inline RetainedState FixtureForDomain(Domain domain) {
    RetainedState state = Fixture();
    state.definition.domain = domain;
    if (domain == Domain::Path3D) state.definition.controlPoints[1].local[2] = 1;
    CanonicalizeState(state);
    return state;
}

inline bool EditAndDurability(Domain domain, EditKind kind, double coordinate,
                              std::map<std::string, bool>& checks,
                              const std::string& prefix) {
    RetainedState original = FixtureForDomain(domain);
    Owner owner(original);
    EditProposal proposal = Move(original, kind, ID(11), {{2, coordinate,
        domain == Domain::Path3D ? 1.0 : 0.0}}, 50);
    RetainedState candidate = PrepareEdit(original, proposal).candidate;
    if (!CanonicalizeState(candidate)) return false;
    proposal.replacementRecipeDigest = candidate.authority.recipeDigest;
    const auto detached = [](const RetainedState& state) { return ValidState(state); };
    const bool committed = CommitEdit(owner, proposal, detached) == CommitResult::Committed;
    const RetainedState edited = owner.state;
    const bool history = committed && owner.undo() && SameState(owner.state, original)
        && owner.redo() && SameState(owner.state, edited);
    RetainedState reopened;
    const bool cold = history && ColdRoundTrip(owner.state, reopened)
        && SameState(owner.state, reopened);
    Owner later(reopened);
    EditProposal second = Move(reopened, EditKind::MovePole, ID(10),
        {{1, 0, 0}}, 51);
    RetainedState secondCandidate = PrepareEdit(reopened, second).candidate;
    if (!CanonicalizeState(secondCandidate)) return false;
    second.replacementRecipeDigest = secondCandidate.authority.recipeDigest;
    const bool reedited = cold
        && CommitEdit(later, second, detached) == CommitResult::Committed;
    checks[prefix + "-commit"] = committed;
    checks[prefix + "-undo-redo"] = history;
    checks[prefix + "-cold-open"] = cold;
    checks[prefix + "-later-edit"] = reedited;
    checks[prefix + "-identity"] = reedited
        && later.state.authority.owner == original.authority.owner
        && later.state.authority.feature == original.authority.feature
        && later.state.definition.frame.identifier == original.definition.frame.identifier
        && later.state.definition.controlPoints[1].identifier == ID(11);
    return committed && history && cold && reedited;
}

inline std::map<std::string, bool> PersistenceProbe(unsigned scenario) {
    std::map<std::string, bool> checks;
    if (scenario == 0) {
        EditAndDurability(Domain::Sketch2D, EditKind::MovePole, 4, checks, "mm-pole");
    } else if (scenario == 1) {
        EditAndDurability(Domain::Path3D, EditKind::MoveOutgoingHandle, 5, checks, "metre-handle");
    } else if (scenario == 2) {
        const std::vector<std::uint8_t> lineArcFixture{
            'S','Y','L','V',1,1,1,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
        };
        const auto before = lineArcFixture;
        Value curve;
        checks["legacy-refused-as-sycv"] = !Decode(lineArcFixture, curve);
        checks["legacy-bytes-exact"] = lineArcFixture == before;
        RetainedState state = FixtureForDomain(Domain::Sketch2D); RetainedState reopened;
        checks["curve-driver-roundtrip"] = ColdRoundTrip(state, reopened) && SameState(state, reopened);
        checks["legacy-still-exact-after-roundtrip"] = lineArcFixture == before;
    } else if (scenario == 3) {
        RetainedState state = FixtureForDomain(Domain::Sketch2D);
        state.tombstones.push_back(ID(42)); ++state.nextLocalID; CanonicalizeState(state);
        RetainedState reopened;
        const bool cold = ColdRoundTrip(state, reopened);
        Owner owner(reopened); EditProposal deleted = Move(reopened, EditKind::MovePole,
            ID(42), {{1, 1, 0}}, 61);
        const RetainedState before = owner.state;
        checks["tombstone-cold-open"] = cold && reopened.tombstones == state.tombstones;
        checks["tombstone-refuses"] = cold && CommitEdit(owner, deleted,
            [](const RetainedState&) { return true; }) == CommitResult::Refused;
        checks["tombstone-no-history"] = SameState(owner.state, before) && owner.begins == 0;
    } else if (scenario == 4) {
        RetainedState state = FixtureForDomain(Domain::Sketch2D); Owner owner(state);
        EditProposal human = Move(state, EditKind::MovePole, ID(11), {{2, 6, 0}}, 70);
        RetainedState candidate = PrepareEdit(state, human).candidate; CanonicalizeState(candidate);
        human.replacementRecipeDigest = candidate.authority.recipeDigest;
        const bool committed = CommitEdit(owner, human,
            [](const RetainedState& value) { return ValidState(value); }) == CommitResult::Committed;
        RetainedState reopened; const bool cold = committed && ColdRoundTrip(owner.state, reopened);
        Owner later(reopened); const RetainedState before = later.state; later.unrelatedLaterStep();
        checks["human-edit-current"] = committed && owner.state.definition.controlPoints[1].local[1] == 6;
        checks["unrelated-preserves-current"] = cold && SameState(later.state, before)
            && later.unrelated == 1;
    } else if (scenario == 5) {
        RetainedState state = FixtureForDomain(Domain::Sketch2D); Owner owner(state);
        EditProposal targeted = Move(state, EditKind::MovePole, ID(12), {{7, 0, 0}}, 80);
        RetainedState candidate = PrepareEdit(state, targeted).candidate; CanonicalizeState(candidate);
        targeted.replacementRecipeDigest = candidate.authority.recipeDigest;
        const bool committed = CommitEdit(owner, targeted,
            [](const RetainedState& value) { return ValidState(value); }) == CommitResult::Committed;
        EditProposal stale = Move(state, EditKind::MovePole, ID(12), {{8, 0, 0}}, 81);
        const RetainedState before = owner.state;
        const auto refusal = CommitEdit(owner, stale,
            [](const RetainedState& value) { return ValidState(value); });
        checks["objects-targets-uuid"] = committed
            && owner.state.definition.controlPoints[2].identifier == ID(12)
            && owner.state.definition.controlPoints[2].local[0] == 7;
        checks["stale-revision-reported"] = refusal == CommitResult::Refused;
        checks["stale-zero-mutation"] = SameState(owner.state, before);
    } else checks["invalid-scenario"] = false;
    return checks;
}
} // namespace core3d::bounded_curve::debug

inline std::map<std::string, bool> Core3DDebugBoundedCurvePersistenceProbe(unsigned scenario) {
    return core3d::bounded_curve::debug::PersistenceProbe(scenario);
}
