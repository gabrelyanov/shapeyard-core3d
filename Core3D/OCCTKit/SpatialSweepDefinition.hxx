#pragma once

// C2-K0 persisted feature value. The C1 SYCV source remains owned by C1 and
// is carried separately by the composite source node; this value stores only
// the exact binding and complete C1 owner-state receipt needed to replay it.
#include "BoundedCurveEdit.hxx"
#include "RetainedSourceRegistry.hxx"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace core3d::spatial_sweep {
using bounded_curve::Digest;
using bounded_curve::UUID;

inline constexpr std::uint32_t Schema = 1;
inline constexpr std::size_t MaximumPayloadBytes = 16 * 1024;
inline constexpr std::size_t MaximumCurveDefinitionBytes = 16 * 1024;
inline constexpr std::size_t MaximumCurveRecordsPerDocument = 256;
inline constexpr std::size_t MaximumCurveDocumentBytes = 1024 * 1024;
inline constexpr std::uint32_t CompositeEnvelopeSchema = 2;
// C2-K0 reservation alias. The D65/D67 frozen source allocation is
// authoritative; the historical draft raw kind 5 is never emitted.
inline constexpr std::uint8_t BoundedCurvePathSourceKind =
    retained_source::BoundedCurvePathKind;
inline constexpr std::uint32_t SpatialCircleSweepFeatureKind = 2;
inline constexpr std::uint32_t SpatialCircleSweepFeatureCodec = 1;
inline constexpr double Pi = 3.141592653589793238462643383279502884;

enum class SectionKind : std::uint8_t { SolidCircle = 1 };
enum class TransportKind : std::uint8_t { BishopV1 = 1 };
enum class ParameterizationKind : std::uint8_t { NormalizedArcLengthV1 = 1 };
enum class RadiusLawKind : std::uint8_t { Constant = 1, LinearArcLength = 2 };
enum class TwistLawKind : std::uint8_t { LinearArcLength = 1, CloseFrame = 2 };
enum class ClosureKind : std::uint8_t { OpenFlatCaps = 1, ClosedNoCaps = 2 };

enum class Refusal : std::uint8_t {
    None = 0,
    BadSchema,
    BadBinding,
    BadCurveOwnerState,
    BadUnit,
    BadSection,
    BadOrientation,
    BadTransport,
    BadRadiusLaw,
    BadTwistLaw,
    BadClosure,
    BadWitness,
    BadProfile,
    NonCanonicalEncoding,
    PayloadTooLarge,
    RuleNotInstalled,
};

struct CurveOwnerState {
    std::uint64_t definitionRevision = 0;
    std::uint64_t nextLocalID = 0;
    Digest canonicalDefinitionDigest{};
    std::vector<UUID> tombstones;
};

struct PathBinding {
    UUID inputNode{};
    UUID curveFeature{};
    Digest sourceRecipeDigest{};
    CurveOwnerState ownerState;
};

struct Orientation {
    std::array<double, 3> authoredSeed{{0, 0, 1}};
    double phaseRadians = 0;
};

struct RadiusLaw {
    RadiusLawKind kind = RadiusLawKind::Constant;
    double startRadius = 0;
    double endRadius = 0;
};

struct TwistLaw {
    TwistLawKind kind = TwistLawKind::LinearArcLength;
    double totalRadians = 0;
    std::int32_t windingTurns = 0;
};

struct TransportWitness {
    bool present = false;
    double unwrappedHolonomyReference = 0;
    std::int32_t lift = 0;
    std::uint32_t solverVersion = 1;
    std::uint32_t proofVersion = 1;
};

struct BuildProfile {
    // c2-circle-1. Changing any component requires a new installed profile.
    std::uint32_t algorithm = 1;
    std::uint32_t tolerance = 1;
    std::uint32_t proof = 1;
    std::uint32_t serializer = 1;
};

struct Definition {
    std::uint32_t schema = Schema;
    PathBinding path;
    double dimensionMetersPerUnit = 0;
    SectionKind section = SectionKind::SolidCircle;
    UUID sectionIdentifier{};
    Orientation orientation;
    TransportKind transport = TransportKind::BishopV1;
    ParameterizationKind parameterization = ParameterizationKind::NormalizedArcLengthV1;
    RadiusLaw radius;
    TwistLaw twist;
    ClosureKind closure = ClosureKind::OpenFlatCaps;
    TransportWitness witness;
    BuildProfile profile;
};

inline bool Finite(double value) noexcept { return std::isfinite(value); }

inline bool CanonicalOwnerState(const CurveOwnerState& value) noexcept {
    if (value.definitionRevision == 0 || value.nextLocalID == 0
        || !retained_recipe::Nonzero(value.canonicalDefinitionDigest)
        || value.tombstones.size() > bounded_curve::MaximumControlPoints) return false;
    UUID previous{};
    bool first = true;
    for (const UUID& identifier : value.tombstones) {
        if (!retained_recipe::Nonzero(identifier)
            || (!first && !(previous < identifier))) return false;
        previous = identifier;
        first = false;
    }
    return true;
}

inline Refusal Validate(const Definition& value) noexcept {
    if (value.schema != Schema) return Refusal::BadSchema;
    if (!retained_recipe::Nonzero(value.path.inputNode)
        || !retained_recipe::Nonzero(value.path.curveFeature)
        || !retained_recipe::Nonzero(value.path.sourceRecipeDigest)) return Refusal::BadBinding;
    if (!CanonicalOwnerState(value.path.ownerState)) return Refusal::BadCurveOwnerState;
    if (!Finite(value.dimensionMetersPerUnit) || value.dimensionMetersPerUnit <= 0)
        return Refusal::BadUnit;
    if (value.section != SectionKind::SolidCircle
        || !retained_recipe::Nonzero(value.sectionIdentifier)) return Refusal::BadSection;
    double seed2 = 0;
    for (double component : value.orientation.authoredSeed) {
        if (!Finite(component)) return Refusal::BadOrientation;
        seed2 += component * component;
    }
    if (!Finite(seed2) || seed2 < 0.25 || seed2 > 4
        || !Finite(value.orientation.phaseRadians)
        || value.orientation.phaseRadians < -Pi || value.orientation.phaseRadians > Pi)
        return Refusal::BadOrientation;
    if (value.transport != TransportKind::BishopV1
        || value.parameterization != ParameterizationKind::NormalizedArcLengthV1)
        return Refusal::BadTransport;
    if ((value.radius.kind != RadiusLawKind::Constant
            && value.radius.kind != RadiusLawKind::LinearArcLength)
        || !Finite(value.radius.startRadius) || value.radius.startRadius <= 0
        || !Finite(value.radius.endRadius) || value.radius.endRadius <= 0
        || (value.radius.kind == RadiusLawKind::Constant
            && value.radius.startRadius != value.radius.endRadius)) return Refusal::BadRadiusLaw;
    if (value.closure != ClosureKind::OpenFlatCaps
        && value.closure != ClosureKind::ClosedNoCaps) return Refusal::BadClosure;
    if (value.closure == ClosureKind::OpenFlatCaps) {
        if (value.twist.kind != TwistLawKind::LinearArcLength
            || !Finite(value.twist.totalRadians)
            || value.twist.windingTurns != 0 || value.witness.present)
            return Refusal::BadTwistLaw;
    } else {
        if (value.radius.kind != RadiusLawKind::Constant
            || value.twist.kind != TwistLawKind::CloseFrame
            || value.twist.totalRadians != 0 || value.twist.windingTurns < -2
            || value.twist.windingTurns > 2) return Refusal::BadTwistLaw;
        if (!value.witness.present || !Finite(value.witness.unwrappedHolonomyReference)
            || std::abs(value.witness.unwrappedHolonomyReference) > 8 * Pi
            || value.witness.solverVersion != 1 || value.witness.proofVersion != 1)
            return Refusal::BadWitness;
    }
    if (value.profile.algorithm != 1 || value.profile.tolerance != 1
        || value.profile.proof != 1 || value.profile.serializer != 1)
        return Refusal::BadProfile;
    return Refusal::None;
}

inline bool SameDefinition(const Definition& a, const Definition& b) noexcept {
    return a.schema == b.schema && a.path.inputNode == b.path.inputNode
        && a.path.curveFeature == b.path.curveFeature
        && a.path.sourceRecipeDigest == b.path.sourceRecipeDigest
        && a.path.ownerState.definitionRevision == b.path.ownerState.definitionRevision
        && a.path.ownerState.nextLocalID == b.path.ownerState.nextLocalID
        && a.path.ownerState.canonicalDefinitionDigest == b.path.ownerState.canonicalDefinitionDigest
        && a.path.ownerState.tombstones == b.path.ownerState.tombstones
        && a.dimensionMetersPerUnit == b.dimensionMetersPerUnit
        && a.section == b.section && a.sectionIdentifier == b.sectionIdentifier
        && a.orientation.authoredSeed == b.orientation.authoredSeed
        && a.orientation.phaseRadians == b.orientation.phaseRadians
        && a.transport == b.transport && a.parameterization == b.parameterization
        && a.radius.kind == b.radius.kind && a.radius.startRadius == b.radius.startRadius
        && a.radius.endRadius == b.radius.endRadius && a.twist.kind == b.twist.kind
        && a.twist.totalRadians == b.twist.totalRadians
        && a.twist.windingTurns == b.twist.windingTurns && a.closure == b.closure
        && a.witness.present == b.witness.present
        && a.witness.unwrappedHolonomyReference == b.witness.unwrappedHolonomyReference
        && a.witness.lift == b.witness.lift
        && a.witness.solverVersion == b.witness.solverVersion
        && a.witness.proofVersion == b.witness.proofVersion
        && a.profile.algorithm == b.profile.algorithm
        && a.profile.tolerance == b.profile.tolerance
        && a.profile.proof == b.profile.proof
        && a.profile.serializer == b.profile.serializer;
}
} // namespace core3d::spatial_sweep
