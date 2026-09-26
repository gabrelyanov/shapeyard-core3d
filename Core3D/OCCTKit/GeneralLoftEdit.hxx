#pragma once

// C3 station replacement gate. The Objects editor submits one complete station
// under an exact retained-owner fence. It cannot insert, delete, rotate, reverse
// or sort correspondence entries in the ruled-v1 release.
#include "GeneralLoftDefinition.hxx"
#include <algorithm>
#include <limits>

namespace core3d::general_loft {
enum class EditRefusal : std::uint8_t {
    None = 0, InvalidState, StaleOwner, StaleDefinition, StaleRecipe,
    MissingStation, IdentityChanged, FrameChangedImplicitly,
    InvalidReplacement, NoChange
};

struct StationEditProposal {
    retained_recipe::OwnerKey expectedOwner;
    UUID expectedFeature{};
    std::uint64_t expectedDefinitionRevision = 0;
    Digest expectedRecipeDigest{};
    UUID station{};
    Station replacement;
    Digest replacementRecipeDigest{};
};

struct PreparedEdit {
    EditRefusal refusal = EditRefusal::InvalidState;
    Definition before;
    Definition candidate;
};

inline bool SameCurveValue(const bounded_curve::Definition& a,
                           const bounded_curve::Definition& b) noexcept {
    return bounded_curve::SameDefinition(a, b);
}
inline bool SameStationValue(const Station& a, const Station& b) noexcept {
    if (a.identifier != b.identifier || a.orderParameter != b.orderParameter
        || a.twistFromPreviousRadians != b.twistFromPreviousRadians
        || !SameFrameBits(a.frame, b.frame) || a.junctions.size() != b.junctions.size()
        || a.segments.size() != b.segments.size()) return false;
    for (std::size_t i = 0; i < a.junctions.size(); ++i)
        if (a.junctions[i].identifier != b.junctions[i].identifier
            || a.junctions[i].correspondence != b.junctions[i].correspondence
            || a.junctions[i].local != b.junctions[i].local) return false;
    for (std::size_t i = 0; i < a.segments.size(); ++i) {
        const Segment& x = a.segments[i]; const Segment& y = b.segments[i];
        if (x.identifier != y.identifier || x.correspondence != y.correspondence
            || x.startJunction != y.startJunction || x.endJunction != y.endJunction
            || !bounded_curve::SameAuthority(x.curveState.authority, y.curveState.authority)
            || x.curveState.tombstones != y.curveState.tombstones
            || !SameCurveValue(x.curveState.definition, y.curveState.definition)) return false;
    }
    return true;
}
inline bool SameDefinitionValue(const Definition& a, const Definition& b) noexcept {
    if (a.schema != b.schema || !(a.owner == b.owner) || a.feature != b.feature
        || a.definitionRevision != b.definitionRevision || a.recipeDigest != b.recipeDigest
        || a.dimensionMetersPerUnit != b.dimensionMetersPerUnit
        || a.interpolation != b.interpolation || a.holes != b.holes || a.caps != b.caps
        || a.orderAxis != b.orderAxis || a.stations.size() != b.stations.size()) return false;
    for (std::size_t i = 0; i < a.stations.size(); ++i)
        if (!SameStationValue(a.stations[i], b.stations[i])) return false;
    return true;
}

inline PreparedEdit PrepareStationEdit(const Definition& current,
                                       const StationEditProposal& proposal) noexcept {
    PreparedEdit result; result.before = current; result.candidate = current;
    try {
        if (Validate(current) != Admission::Accepted) return result;
        if (!(proposal.expectedOwner == current.owner)
            || proposal.expectedFeature != current.feature) {
            result.refusal = EditRefusal::StaleOwner; return result;
        }
        if (proposal.expectedDefinitionRevision != current.definitionRevision) {
            result.refusal = EditRefusal::StaleDefinition; return result;
        }
        if (proposal.expectedRecipeDigest != current.recipeDigest) {
            result.refusal = EditRefusal::StaleRecipe; return result;
        }
        const auto found = std::find_if(result.candidate.stations.begin(),
            result.candidate.stations.end(), [&](const Station& station) {
                return station.identifier == proposal.station;
            });
        if (found == result.candidate.stations.end()) {
            result.refusal = EditRefusal::MissingStation; return result;
        }
        const std::size_t index = std::size_t(found - result.candidate.stations.begin());
        if (proposal.replacement.identifier != proposal.station
            || !retained_recipe::Nonzero(proposal.replacementRecipeDigest)
            || proposal.replacementRecipeDigest == current.recipeDigest
            || current.definitionRevision == std::numeric_limits<std::uint64_t>::max()) {
            result.refusal = EditRefusal::InvalidReplacement; return result;
        }
        if (SameStationValue(*found, proposal.replacement)) {
            result.refusal = EditRefusal::NoChange; return result;
        }
        const bool frameChanged = !SameFrameBits(found->frame, proposal.replacement.frame);
        if (frameChanged && bounded_curve::ValidateFrameReplacement(
                found->frame, proposal.replacement.frame) != bounded_curve::Refusal::None) {
            result.refusal = EditRefusal::FrameChangedImplicitly; return result;
        }
        result.candidate.stations[index] = proposal.replacement;
        ++result.candidate.definitionRevision;
        result.candidate.recipeDigest = proposal.replacementRecipeDigest;
        if (!PreservesIdentityAndCorrespondence(current, result.candidate)) {
            result.candidate = current; result.refusal = EditRefusal::IdentityChanged; return result;
        }
        if (Validate(result.candidate) != Admission::Accepted) {
            result.candidate = current; result.refusal = EditRefusal::InvalidReplacement; return result;
        }
        result.refusal = EditRefusal::None; return result;
    } catch (...) {
        result.candidate = current; result.refusal = EditRefusal::InvalidReplacement; return result;
    }
}

// Owner follows the C1b read/build/re-read/one-command protocol. Concrete
// document integration must stage SYCR/2 + GLRF/1 and replay every descendant.
template<class Owner, class DetachedBuild>
bounded_curve::CommitResult CommitStationEdit(Owner& owner,
    const StationEditProposal& proposal, DetachedBuild&& buildDetached) noexcept {
    try {
        Definition original;
        if (!owner.closed()) return bounded_curve::CommitResult::Busy;
        if (!owner.read(original)) return bounded_curve::CommitResult::Refused;
        const PreparedEdit prepared = PrepareStationEdit(original, proposal);
        if (prepared.refusal == EditRefusal::NoChange) return bounded_curve::CommitResult::NoChange;
        if (prepared.refusal != EditRefusal::None || !buildDetached(prepared.candidate))
            return bounded_curve::CommitResult::Refused;
        Definition fenced;
        if (!owner.closed() || !owner.read(fenced) || !SameDefinitionValue(fenced, original))
            return bounded_curve::CommitResult::Refused;
        const std::uint64_t token = owner.begin();
        if (token == 0) return bounded_curve::CommitResult::Busy;
        auto abort = [&]() noexcept {
            if (!owner.owns(token) || !owner.abort(token) || !owner.closed())
                return bounded_curve::CommitResult::RecoveryRequired;
            Definition restored;
            return owner.read(restored) && SameDefinitionValue(restored, original)
                ? bounded_curve::CommitResult::Refused
                : bounded_curve::CommitResult::RecoveryRequired;
        };
        if (!owner.owns(token) || !owner.stage(token, prepared.candidate)
            || !owner.replace(token, prepared.candidate)
            || !owner.replayDependents(token, prepared.candidate)
            || !owner.verify(token, prepared.candidate)) return abort();
        Definition staged;
        if (!owner.read(staged) || !SameDefinitionValue(staged, prepared.candidate)) return abort();
        const auto close = owner.commit(token);
        if (close == bounded_curve::CloseResult::StillOpen) return abort();
        if (close == bounded_curve::CloseResult::ClosedUnknown || !owner.closed())
            return bounded_curve::CommitResult::OutcomeUnknown;
        Definition committed;
        return owner.read(committed) && SameDefinitionValue(committed, prepared.candidate)
            ? bounded_curve::CommitResult::Committed
            : bounded_curve::CommitResult::OutcomeUnknown;
    } catch (...) {
        return bounded_curve::CommitResult::RecoveryRequired;
    }
}
} // namespace core3d::general_loft
