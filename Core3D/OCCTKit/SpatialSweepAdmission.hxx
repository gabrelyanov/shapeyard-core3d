#pragma once
#include "SpatialSweepLaws.hxx"
#include "SpatialSweepProof.hxx"
#include "SpatialSweepTransport.hxx"

namespace core3d::spatial_sweep {
enum class AdmissionRefusal : std::uint8_t {
    None = 0,
    MalformedRecipe,
    WrongCurveDomain,
    CertificationBudgetExceeded,
    RegularityUnproved,
    ZeroTangent,
    Kink,
    LengthOutOfBounds,
    ExtentOutOfBounds,
    ParallelSeed,
    LawOutOfBounds,
    TightCurvature,
    NonlocalContact,
    NonlocalClearanceUnproved,
    OpenSeam,
    HolonomyAmbiguous,
    FrameUnproved,
};

struct PreparedAdmission {
    AdmissionRefusal refusal = AdmissionRefusal::MalformedRecipe;
    double certifiedLengthMM = 0;
    double radiusStartMM = 0;
    double radiusEndMM = 0;
    double radiusEnvelopeMM = 0;
    double correctedTotalSpin = 0;
    double checkedHolonomyLift = 0;
    std::int32_t checkedHolonomyInteger = 0;
    bool admitted() const noexcept { return refusal == AdmissionRefusal::None; }
};

inline PreparedAdmission Admit(const Definition& definition,
                               const bounded_curve::Definition& curve,
                               const CurveCertificate& proof,
                               const Vector3& startTangentInRetainedFrame,
                               bool creation) noexcept {
    PreparedAdmission result;
    if (Validate(definition) != Refusal::None
        || bounded_curve::Validate(curve) != bounded_curve::Refusal::None) return result;
    if (curve.domain != bounded_curve::Domain::Path3D) {
        result.refusal = AdmissionRefusal::WrongCurveDomain; return result;
    }
    if (!WithinBudget(proof.work)) {
        result.refusal = AdmissionRefusal::CertificationBudgetExceeded; return result;
    }
    if (!StructurallyValid(proof)) {
        result.refusal = AdmissionRefusal::RegularityUnproved; return result;
    }
    if (proof.speedPerNormalizedParameter.lower <= 64 * PositionalEpsilonMM) {
        result.refusal = AdmissionRefusal::ZeroTangent; return result;
    }
    if (!proof.everyJoinExactG1G2) {
        result.refusal = AdmissionRefusal::Kink; return result;
    }
    if (proof.lengthMM.lower < 1 || proof.lengthMM.upper > 1e6) {
        result.refusal = AdmissionRefusal::LengthOutOfBounds; return result;
    }
    if (proof.maximumAbsCoordinateMM.upper > 1e6) {
        result.refusal = AdmissionRefusal::ExtentOutOfBounds; return result;
    }
    Frame seeded{};
    if (!SeedFrame(startTangentInRetainedFrame, definition.orientation.authoredSeed, seeded)) {
        result.refusal = AdmissionRefusal::ParallelSeed; return result;
    }
    const double mmPerNativeUnit = definition.dimensionMetersPerUnit * 1000;
    result.radiusStartMM = definition.radius.startRadius * mmPerNativeUnit;
    result.radiusEndMM = definition.radius.endRadius * mmPerNativeUnit;
    if (!Finite(result.radiusStartMM) || !Finite(result.radiusEndMM)
        || result.radiusStartMM < 0.1 || result.radiusStartMM > 100000
        || result.radiusEndMM < 0.1 || result.radiusEndMM > 100000) {
        result.refusal = AdmissionRefusal::LawOutOfBounds; return result;
    }
    const double minimumRadius = std::min(result.radiusStartMM, result.radiusEndMM);
    const double maximumRadius = std::max(result.radiusStartMM, result.radiusEndMM);
    const double certifiedLength = proof.lengthMM.lower;
    if (maximumRadius / minimumRadius > 4
        || std::abs(result.radiusEndMM - result.radiusStartMM) / certifiedLength > 0.25) {
        result.refusal = AdmissionRefusal::LawOutOfBounds; return result;
    }
    result.certifiedLengthMM = certifiedLength;
    result.radiusEnvelopeMM = maximumRadius + 4 * PositionalEpsilonMM;
    if (!(proof.curvaturePerMM.upper * result.radiusEnvelopeMM < 0.25)) {
        result.refusal = AdmissionRefusal::TightCurvature; return result;
    }
    if (!proof.localBandInjective || !proof.fullPairDomainVisited) {
        result.refusal = AdmissionRefusal::NonlocalClearanceUnproved; return result;
    }
    const double requiredDistance = 2 * result.radiusEnvelopeMM + 16 * PositionalEpsilonMM;
    if (!(proof.nonlocalCentrelineDistanceMM.lower > requiredDistance)) {
        result.refusal = proof.nonlocalCentrelineDistanceMM.upper <= 2 * result.radiusEnvelopeMM
            ? AdmissionRefusal::NonlocalContact
            : AdmissionRefusal::NonlocalClearanceUnproved;
        return result;
    }
    if (definition.closure == ClosureKind::OpenFlatCaps) {
        result.correctedTotalSpin = definition.twist.totalRadians;
        if (std::abs(result.correctedTotalSpin) > 4 * Pi
            || std::abs(result.correctedTotalSpin) / certifiedLength
                > Pi / (4 * result.radiusEnvelopeMM)) {
            result.refusal = AdmissionRefusal::LawOutOfBounds; return result;
        }
    } else {
        if (!proof.endpointsExactlyEqual || !proof.seamExactG1G2
            || proof.lengthMM.lower <= 8 * result.radiusEnvelopeMM) {
            result.refusal = AdmissionRefusal::OpenSeam; return result;
        }
        if (proof.holonomyRadians.upper - proof.holonomyRadians.lower
            > 2 * FrameTolerance(result.radiusEnvelopeMM)) {
            result.refusal = AdmissionRefusal::FrameUnproved; return result;
        }
        const double wrapped = (proof.holonomyRadians.lower + proof.holonomyRadians.upper) / 2;
        if (!SelectHolonomyLift(wrapped,
                definition.witness.unwrappedHolonomyReference, creation,
                FrameTolerance(result.radiusEnvelopeMM), result.checkedHolonomyLift,
                result.checkedHolonomyInteger)) {
            result.refusal = AdmissionRefusal::HolonomyAmbiguous; return result;
        }
        result.correctedTotalSpin = -result.checkedHolonomyLift
            + 2 * Pi * double(definition.twist.windingTurns);
        if (std::abs(result.correctedTotalSpin) > 4 * Pi
            || std::abs(result.correctedTotalSpin) / certifiedLength
                > Pi / (4 * result.radiusEnvelopeMM)) {
            result.refusal = AdmissionRefusal::LawOutOfBounds; return result;
        }
    }
    result.refusal = AdmissionRefusal::None;
    return result;
}
} // namespace core3d::spatial_sweep
