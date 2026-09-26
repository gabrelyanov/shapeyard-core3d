#pragma once

// C1b retained-edit admission. This layer never decodes bytes, invents IDs,
// opens an AI command, or owns document lifetime. The serial integration binds
// Owner to the G0 document owner and persists RetainedState through C1a's
// canonical attribute/driver.
#include "BoundedCurveCodec.hxx"
#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <set>
#include <vector>

namespace core3d::bounded_curve {
using Digest = retained_recipe::Digest;

enum class EditKind : std::uint8_t {
    MovePole = 1,
    MoveIncomingHandle = 2,
    MoveOutgoingHandle = 3,
    SetWeight = 4
};

enum class EditRefusal : std::uint8_t {
    None = 0,
    InvalidState,
    StaleOwner,
    StaleDefinition,
    StaleFrame,
    StaleRecipe,
    MissingControlPoint,
    ControlPointDeleted,
    InvalidReplacement,
    NoChange
};

enum class CommitResult : std::uint8_t {
    NoChange = 0,
    Committed,
    Refused,
    Busy,
    OutcomeUnknown,
    RecoveryRequired
};

enum class CloseResult : std::uint8_t { Committed, StillOpen, ClosedUnknown };

struct Authority {
    retained_recipe::OwnerKey owner;
    UUID feature{};
    std::uint64_t definitionRevision = 0;
    UUID frame{};
    std::uint64_t frameRevision = 0;
    Digest recipeDigest{};
};

struct RetainedState {
    Authority authority;
    Definition definition;
    // Audit-only monotonic issuance fence. UUID remains the durable identity.
    std::uint64_t nextLocalID = 0;
    // Canonical lexicographic order. IDs are never removed from this ledger.
    std::vector<UUID> tombstones;
};

struct EditProposal {
    EditKind kind = EditKind::MovePole;
    Authority expected;
    UUID controlPoint{};
    // Handle gestures are resolved by the detached native adapter to the
    // replacement pole of this exact UUID. No sampled vertex is accepted.
    std::array<double, 3> replacementLocal{};
    double replacementWeight = 1;
    // Digest of the canonical rebuilt owner recipe, supplied only after C1a
    // encoding and detached native rebuild succeed.
    Digest replacementRecipeDigest{};
};

struct PreparedEdit {
    EditRefusal refusal = EditRefusal::InvalidState;
    RetainedState before;
    RetainedState candidate;
};

inline bool SameFrame(const Frame& a, const Frame& b) noexcept {
    return a.identifier == b.identifier && a.revision == b.revision
        && a.origin == b.origin && a.xAxis == b.xAxis && a.yAxis == b.yAxis
        && a.zAxis == b.zAxis && a.handedness == b.handedness;
}

inline bool SameDefinition(const Definition& a, const Definition& b) noexcept {
    if (a.schema != b.schema || a.domain != b.domain || !SameFrame(a.frame, b.frame)
        || a.degree != b.degree || a.controlPoints.size() != b.controlPoints.size()
        || a.knots.size() != b.knots.size() || a.weights != b.weights) return false;
    for (std::size_t i = 0; i < a.controlPoints.size(); ++i)
        if (a.controlPoints[i].identifier != b.controlPoints[i].identifier
            || a.controlPoints[i].local != b.controlPoints[i].local) return false;
    for (std::size_t i = 0; i < a.knots.size(); ++i)
        if (a.knots[i].value != b.knots[i].value
            || a.knots[i].multiplicity != b.knots[i].multiplicity) return false;
    return true;
}

inline bool SameAuthority(const Authority& a, const Authority& b) noexcept {
    return a.owner == b.owner && a.feature == b.feature
        && a.definitionRevision == b.definitionRevision && a.frame == b.frame
        && a.frameRevision == b.frameRevision && a.recipeDigest == b.recipeDigest;
}

inline bool SameState(const RetainedState& a, const RetainedState& b) noexcept {
    return SameAuthority(a.authority, b.authority)
        && SameDefinition(a.definition, b.definition)
        && a.nextLocalID == b.nextLocalID
        && a.tombstones == b.tombstones;
}

inline bool ValidState(const RetainedState& state) noexcept {
    if (!retained_recipe::Valid(state.authority.owner)
        || !Nonzero(state.authority.feature)
        || state.authority.definitionRevision == 0
        || !Nonzero(state.authority.frame)
        || state.authority.frameRevision == 0
        || !retained_recipe::Nonzero(state.authority.recipeDigest)
        || Validate(state.definition) != Refusal::None
        || state.definition.frame.identifier != state.authority.frame
        || state.definition.frame.revision != state.authority.frameRevision
        || state.nextLocalID == 0) return false;
    std::set<UUID> live;
    for (const ControlPoint& point : state.definition.controlPoints)
        if (!live.insert(point.identifier).second) return false;
    UUID previous{};
    bool first = true;
    for (const UUID& id : state.tombstones) {
        if (!Nonzero(id) || live.count(id) || (!first && !(previous < id))) return false;
        previous = id;
        first = false;
    }
    if (live.size() + state.tombstones.size() >= state.nextLocalID) return false;
    return true;
}

inline bool ToPersistedValue(const RetainedState& state,
                             PersistedValue& output) noexcept {
    output = {};
    if (!ValidState(state)) return false;
    output.value.feature = state.authority.feature;
    output.value.definition = state.definition;
    output.ownerState.owner = state.authority.owner;
    output.ownerState.feature = state.authority.feature;
    output.ownerState.definitionRevision = state.authority.definitionRevision;
    output.ownerState.nextLocalID = state.nextLocalID;
    output.ownerState.canonicalDefinitionDigest = state.authority.recipeDigest;
    output.ownerState.tombstones = state.tombstones;
    return ValidatePersisted(output);
}

inline bool FromPersistedValue(const PersistedValue& persisted,
                               RetainedState& output) noexcept {
    output = {};
    if (!ValidatePersisted(persisted)) return false;
    output.authority.owner = persisted.ownerState.owner;
    output.authority.feature = persisted.value.feature;
    output.authority.definitionRevision = persisted.ownerState.definitionRevision;
    output.authority.frame = persisted.value.definition.frame.identifier;
    output.authority.frameRevision = persisted.value.definition.frame.revision;
    output.authority.recipeDigest = persisted.ownerState.canonicalDefinitionDigest;
    output.definition = persisted.value.definition;
    output.nextLocalID = persisted.ownerState.nextLocalID;
    output.tombstones = persisted.ownerState.tombstones;
    return ValidState(output);
}

inline Authority CurrentAuthority(const RetainedState& state) noexcept {
    return state.authority;
}

inline PreparedEdit PrepareEdit(const RetainedState& current,
                                const EditProposal& proposal) noexcept {
    PreparedEdit result;
    result.before = current;
    result.candidate = current;
    if (!ValidState(current)) { result.refusal = EditRefusal::InvalidState; return result; }
    if (!(proposal.expected.owner == current.authority.owner)
        || proposal.expected.feature != current.authority.feature) {
        result.refusal = EditRefusal::StaleOwner; return result;
    }
    if (proposal.expected.definitionRevision != current.authority.definitionRevision) {
        result.refusal = EditRefusal::StaleDefinition; return result;
    }
    if (proposal.expected.frame != current.authority.frame
        || proposal.expected.frameRevision != current.authority.frameRevision) {
        result.refusal = EditRefusal::StaleFrame; return result;
    }
    if (proposal.expected.recipeDigest != current.authority.recipeDigest) {
        result.refusal = EditRefusal::StaleRecipe; return result;
    }
    const auto found = std::find_if(result.candidate.definition.controlPoints.begin(),
        result.candidate.definition.controlPoints.end(), [&](const ControlPoint& point) {
            return point.identifier == proposal.controlPoint;
        });
    if (found == result.candidate.definition.controlPoints.end()) {
        result.refusal = std::binary_search(current.tombstones.begin(), current.tombstones.end(),
            proposal.controlPoint) ? EditRefusal::ControlPointDeleted
                                   : EditRefusal::MissingControlPoint;
        return result;
    }
    if (!retained_recipe::Nonzero(proposal.replacementRecipeDigest)
        || proposal.replacementRecipeDigest == current.authority.recipeDigest
        || current.authority.definitionRevision == std::numeric_limits<std::uint64_t>::max()) {
        result.refusal = EditRefusal::InvalidReplacement; return result;
    }
    const std::size_t index = std::size_t(found - result.candidate.definition.controlPoints.begin());
    bool changed = false;
    if (proposal.kind == EditKind::SetWeight) {
        if (result.candidate.definition.weights.size()
                != result.candidate.definition.controlPoints.size()
            || !Finite(proposal.replacementWeight)
            || proposal.replacementWeight < MinimumWeight
            || proposal.replacementWeight > MaximumWeight) {
            result.refusal = EditRefusal::InvalidReplacement; return result;
        }
        changed = result.candidate.definition.weights[index] != proposal.replacementWeight;
        result.candidate.definition.weights[index] = proposal.replacementWeight;
    } else if (proposal.kind == EditKind::MovePole
               || proposal.kind == EditKind::MoveIncomingHandle
               || proposal.kind == EditKind::MoveOutgoingHandle) {
        for (double scalar : proposal.replacementLocal)
            if (!Finite(scalar) || std::abs(scalar) > CoordinateLimit) {
                result.refusal = EditRefusal::InvalidReplacement; return result;
            }
        if (result.candidate.definition.domain == Domain::Sketch2D
            && proposal.replacementLocal[2] != 0) {
            result.refusal = EditRefusal::InvalidReplacement; return result;
        }
        changed = found->local != proposal.replacementLocal;
        result.candidate.definition.controlPoints[index].local = proposal.replacementLocal;
    } else {
        result.refusal = EditRefusal::InvalidReplacement; return result;
    }
    if (!changed) { result.refusal = EditRefusal::NoChange; return result; }
    ++result.candidate.authority.definitionRevision;
    result.candidate.authority.recipeDigest = proposal.replacementRecipeDigest;
    if (Validate(result.candidate.definition) != Refusal::None
        || !SameFrame(result.before.definition.frame, result.candidate.definition.frame)
        || result.before.tombstones != result.candidate.tombstones) {
        result.candidate = current;
        result.refusal = EditRefusal::InvalidReplacement;
        return result;
    }
    for (std::size_t i = 0; i < current.definition.controlPoints.size(); ++i)
        if (current.definition.controlPoints[i].identifier
            != result.candidate.definition.controlPoints[i].identifier) {
            result.candidate = current;
            result.refusal = EditRefusal::InvalidReplacement;
            return result;
        }
    result.refusal = EditRefusal::None;
    return result;
}

// Owner is the main-thread G0 document owner. Required methods are deliberately
// narrow: closed/read/begin/owns/stage/replace/verify/commit/abort. `replace`
// publishes a detached native result bound to candidate canonical bytes.
template <class Owner, class DetachedBuild>
CommitResult CommitEdit(Owner& owner, const EditProposal& proposal,
                        DetachedBuild&& buildDetached) noexcept {
    try {
        RetainedState original;
        if (!owner.closed()) return CommitResult::Busy;
        if (!owner.read(original)) return CommitResult::Refused;
        const PreparedEdit prepared = PrepareEdit(original, proposal);
        if (prepared.refusal == EditRefusal::NoChange) return CommitResult::NoChange;
        if (prepared.refusal != EditRefusal::None
            || !buildDetached(prepared.candidate)) return CommitResult::Refused;
        RetainedState fenced;
        if (!owner.closed() || !owner.read(fenced) || !SameState(fenced, original))
            return CommitResult::Refused;
        const std::uint64_t token = owner.begin();
        if (token == 0) return CommitResult::Busy;
        auto abort = [&]() noexcept {
            if (!owner.owns(token) || !owner.abort(token) || !owner.closed())
                return CommitResult::RecoveryRequired;
            RetainedState restored;
            return owner.read(restored) && SameState(restored, original)
                ? CommitResult::Refused : CommitResult::RecoveryRequired;
        };
        RetainedState inside;
        if (!owner.owns(token) || !owner.read(inside) || !SameState(inside, original)
            || !owner.stage(token, prepared.candidate)
            || !owner.replace(token, prepared.candidate)
            || !owner.verify(token, prepared.candidate)) return abort();
        RetainedState staged;
        if (!owner.read(staged) || !SameState(staged, prepared.candidate)) return abort();
        const CloseResult close = owner.commit(token);
        if (close == CloseResult::StillOpen) return abort();
        if (close == CloseResult::ClosedUnknown || !owner.closed())
            return CommitResult::OutcomeUnknown;
        RetainedState committed;
        if (!owner.read(committed) || !SameState(committed, prepared.candidate))
            return CommitResult::OutcomeUnknown;
        return CommitResult::Committed;
    } catch (...) {
        // The concrete G0 owner must close an owned command in its noexcept
        // operation guards. Escaping exceptions never imply a successful edit.
        return CommitResult::RecoveryRequired;
    }
}
} // namespace core3d::bounded_curve
