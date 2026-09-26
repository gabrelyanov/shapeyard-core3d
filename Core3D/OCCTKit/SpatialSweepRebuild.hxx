#pragma once

// Detached fixed-point and retained replay policy. The real G0 adapter must
// supply BinTools VERSION_4 read/write and the actual OCAF transaction. These
// functions make the one-roundtrip rule and atomic refusal boundary explicit;
// they do not synthesize missing C1 codec bytes or open a document command.
#include "SpatialSweepSolid.hxx"
#include <cstdint>
#include <vector>

namespace core3d::spatial_sweep {
using GeometryBytes = std::vector<std::uint8_t>;
using NormalizeGeometry = bool (*)(const GeometryBytes&, GeometryBytes&) noexcept;

enum class FixedPointRefusal : std::uint8_t {
    None = 0,
    EmptyGeometry,
    FirstReadWrite,
    SecondReadWrite,
    NonFixedPoint,
    IndependentBuildMismatch,
};

struct FixedPointResult {
    FixedPointRefusal refusal = FixedPointRefusal::EmptyGeometry;
    GeometryBytes canonicalGeometry;
    bool admitted() const noexcept { return refusal == FixedPointRefusal::None; }
};

inline FixedPointResult PrepareFixedPoint(const GeometryBytes& firstDetachedBuild,
                                          const GeometryBytes& secondDetachedBuild,
                                          NormalizeGeometry normalize) noexcept {
    FixedPointResult result;
    try {
        if (firstDetachedBuild.empty() || secondDetachedBuild.empty() || normalize == nullptr)
            return result;
        GeometryBytes firstCanonical, secondCanonical, repeated;
        if (!normalize(firstDetachedBuild, firstCanonical)
            || !normalize(secondDetachedBuild, secondCanonical)) {
            result.refusal = FixedPointRefusal::FirstReadWrite; return result;
        }
        if (firstCanonical != secondCanonical) {
            result.refusal = FixedPointRefusal::IndependentBuildMismatch; return result;
        }
        if (!normalize(firstCanonical, repeated)) {
            result.refusal = FixedPointRefusal::SecondReadWrite; return result;
        }
        if (firstCanonical != repeated) {
            result.refusal = FixedPointRefusal::NonFixedPoint; return result;
        }
        result.refusal = FixedPointRefusal::None;
        result.canonicalGeometry = std::move(firstCanonical);
        return result;
    } catch (...) { return result; }
}

struct RetainedIdentityFence {
    UUID owner{};
    UUID definition{};
    UUID sourceNode{};
    UUID curveFeature{};
    UUID sweepFeature{};
    UUID section{};
    std::uint64_t ownerRevision = 0;
    std::uint64_t curveRevision = 0;
    Digest sourceDigest{};
    Digest featureDigest{};
};

inline bool SameFence(const RetainedIdentityFence& a,
                      const RetainedIdentityFence& b) noexcept {
    return a.owner == b.owner && a.definition == b.definition
        && a.sourceNode == b.sourceNode && a.curveFeature == b.curveFeature
        && a.sweepFeature == b.sweepFeature && a.section == b.section
        && a.ownerRevision == b.ownerRevision && a.curveRevision == b.curveRevision
        && a.sourceDigest == b.sourceDigest && a.featureDigest == b.featureDigest;
}

enum class ReplayRefusal : std::uint8_t {
    None = 0,
    Stale,
    UnsupportedDependent,
    MissingCanonicalRecipe,
    DetachedCandidateRejected,
    StageFailed,
    VerifyFailed,
    OutcomeUnknown,
};

struct ReplayPreparation {
    RetainedIdentityFence captured;
    RetainedIdentityFence reread;
    bool canonicalC1AndCompositeBytesAvailable = false;
    bool allDescendantsSupported = false;
    bool detachedCandidateAdmitted = false;
    bool stageWouldSucceed = false;
    bool bindingVerificationWouldSucceed = false;
    bool closeOutcomeKnown = true;
};

struct ReplayDecision {
    ReplayRefusal refusal = ReplayRefusal::Stale;
    bool mayOpenOneOwnedCommand = false;
    bool oldStateMustRemainExact = true;
    bool admitted() const noexcept { return refusal == ReplayRefusal::None; }
};

inline ReplayDecision PrepareRetainedReplay(const ReplayPreparation& input) noexcept {
    ReplayDecision result;
    if (!SameFence(input.captured, input.reread)) return result;
    if (!input.canonicalC1AndCompositeBytesAvailable) {
        result.refusal = ReplayRefusal::MissingCanonicalRecipe; return result;
    }
    if (!input.allDescendantsSupported) {
        result.refusal = ReplayRefusal::UnsupportedDependent; return result;
    }
    if (!input.detachedCandidateAdmitted) {
        result.refusal = ReplayRefusal::DetachedCandidateRejected; return result;
    }
    result.mayOpenOneOwnedCommand = true;
    if (!input.stageWouldSucceed) {
        result.refusal = ReplayRefusal::StageFailed; return result;
    }
    if (!input.bindingVerificationWouldSucceed) {
        result.refusal = ReplayRefusal::VerifyFailed; return result;
    }
    if (!input.closeOutcomeKnown) {
        result.refusal = ReplayRefusal::OutcomeUnknown;
        result.oldStateMustRemainExact = false;
        return result;
    }
    result.refusal = ReplayRefusal::None;
    result.oldStateMustRemainExact = false;
    return result;
}
} // namespace core3d::spatial_sweep
