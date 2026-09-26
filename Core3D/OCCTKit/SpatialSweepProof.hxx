#pragma once
#include "SpatialSweepDefinition.hxx"
#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace core3d::spatial_sweep {
inline constexpr double PositionalEpsilonMM = 0.001;
inline constexpr std::size_t MaximumProofLeaves = 4096;
inline constexpr std::uint32_t MaximumBisectionDepth = 20;
inline constexpr std::size_t MaximumCurvePairCells = 262144;
inline constexpr std::size_t MaximumSurfaceCertificateCells = 262144;
inline constexpr std::uint32_t MaximumExactArithmeticBits = 4096;

struct Interval {
    double lower = 0;
    double upper = 0;
};

inline bool Valid(const Interval& value) noexcept {
    return Finite(value.lower) && Finite(value.upper) && value.lower <= value.upper;
}

struct WorkReceipt {
    std::size_t proofLeaves = 0;
    std::uint32_t maximumDepth = 0;
    std::size_t curvePairCells = 0;
    std::size_t surfaceCells = 0;
    std::uint32_t exactArithmeticBits = 0;
};

enum class CurveProofProvenance : std::uint8_t {
    None = 0,
    HomogeneousBernsteinExactV1 = 1,
};

inline bool WithinBudget(const WorkReceipt& value, bool includeSurface = false) noexcept {
    return value.proofLeaves <= MaximumProofLeaves
        && value.maximumDepth <= MaximumBisectionDepth
        && value.curvePairCells <= MaximumCurvePairCells
        && (!includeSurface || value.surfaceCells <= MaximumSurfaceCertificateCells)
        && value.exactArithmeticBits <= MaximumExactArithmeticBits;
}

// Output of C1 rational-span extraction and outward-rounded certification.
// No sampled observation may set one of these fields to true. The K1 adapter
// must produce this receipt from homogeneous Bernstein spans and exact join
// arithmetic before calling Admit below.
struct CurveCertificate {
    Interval lengthMM;
    Interval speedPerNormalizedParameter;
    Interval curvaturePerMM;
    Interval maximumAbsCoordinateMM;
    Interval nonlocalCentrelineDistanceMM;
    Interval holonomyRadians;
    bool homogeneousBernsteinBounds = false;
    bool everyJoinExactG1G2 = false;
    bool localBandInjective = false;
    bool fullPairDomainVisited = false;
    bool endpointsExactlyEqual = false;
    bool seamExactG1G2 = false;
    WorkReceipt work;
    CurveProofProvenance provenance = CurveProofProvenance::None;
};

inline bool StructurallyValid(const CurveCertificate& value) noexcept {
    return Valid(value.lengthMM) && Valid(value.speedPerNormalizedParameter)
        && Valid(value.curvaturePerMM) && Valid(value.maximumAbsCoordinateMM)
        && Valid(value.nonlocalCentrelineDistanceMM) && Valid(value.holonomyRadians)
        && value.homogeneousBernsteinBounds
        && value.provenance == CurveProofProvenance::HomogeneousBernsteinExactV1
        && WithinBudget(value.work);
}

struct PairCell {
    Interval intrinsicSeparationMM;
    Interval centrelineDistanceMM;
};

enum class PairDecision : std::uint8_t {
    LocalBand = 1,
    CertifiedSeparated = 2,
    MustSplit = 3,
    Contact = 4,
};

inline PairDecision ClassifyPairCell(const PairCell& cell, double radiusEnvelopeMM) noexcept {
    if (!Valid(cell.intrinsicSeparationMM) || !Valid(cell.centrelineDistanceMM)
        || !Finite(radiusEnvelopeMM) || radiusEnvelopeMM <= 0) return PairDecision::Contact;
    const double localBand = 4 * radiusEnvelopeMM;
    const double requiredDistance = 2 * radiusEnvelopeMM + 16 * PositionalEpsilonMM;
    if (cell.intrinsicSeparationMM.upper <= localBand) return PairDecision::LocalBand;
    if (cell.centrelineDistanceMM.lower > requiredDistance)
        return PairDecision::CertifiedSeparated;
    if (cell.centrelineDistanceMM.upper <= 2 * radiusEnvelopeMM)
        return PairDecision::Contact;
    return PairDecision::MustSplit; // includes every cell straddling 4R.
}

inline double FrameTolerance(double radiusEnvelopeMM) noexcept {
    return std::min(1e-8, PositionalEpsilonMM / (32 * radiusEnvelopeMM));
}

// K2 consumes proof receipts produced from the returned rational surfaces. A
// receipt is usable only when every generated cell is independently mapped
// back to the intended spine interval and circular quarter; OCCT's scalar
// fitting error is recorded, but is never treated as that proof.
struct SurfaceCellCertificate {
    Interval normalEquationDerivative;
    Interval radialDeviationMM;
    Interval orientedMapJacobian;
    bool intendedSpineBracketIsUnique = false;
    bool otherSpineIntervalsDischarged = false;
};

struct SurfaceCertificate {
    std::vector<SurfaceCellCertificate> cells;
    std::array<bool, 4> orderedQuarterCoverage{{false, false, false, false}};
    bool sharedMeridiansMatch = false;
    bool pathBoundaryCoveredOnce = false;
    bool openCapsPlanarAndOriented = false;
    bool closedHasNoCaps = false;
    double fittingErrorMM = 0;
    double transportErrorMM = 0;
    double sewingErrorMM = 0;
    double serializationErrorMM = 0;
    WorkReceipt work;
    enum class Provenance : std::uint8_t {
        None = 0,
        ReturnedRationalSurfaceIntervalsV1 = 1,
        SyntheticFixture = 255,
    } provenance = Provenance::None;
};

struct IndependentMeasureCertificate {
    Interval sourceLengthMM;
    Interval expectedVolumeMM3;
    Interval measuredVolumeMM3;
    std::vector<Interval> stationRadiusMM;
    bool sourcePolynomialIntegratedIndependently = false;
    bool everyLocalSectionAccountedFor = false;
    bool markedMeridianMatches = false;
};

enum class SurfaceProofRefusal : std::uint8_t {
    None = 0,
    Budget,
    FittingError,
    SurfaceMap,
    QuarterCoverage,
    Boundary,
    IndependentMeasure,
};

inline SurfaceProofRefusal ValidateSurfaceProof(const SurfaceCertificate& surface,
                                                const IndependentMeasureCertificate& measure,
                                                ClosureKind closure) noexcept {
    if (surface.provenance != SurfaceCertificate::Provenance::ReturnedRationalSurfaceIntervalsV1
        || !WithinBudget(surface.work, true) || surface.cells.empty()
        || surface.cells.size() != surface.work.surfaceCells)
        return SurfaceProofRefusal::Budget;
    const double spent = surface.fittingErrorMM + surface.transportErrorMM
        + surface.sewingErrorMM + surface.serializationErrorMM;
    if (!Finite(spent) || spent < 0 || spent > PositionalEpsilonMM)
        return SurfaceProofRefusal::FittingError;
    for (const auto& cell : surface.cells) {
        if (!Valid(cell.normalEquationDerivative) || !Valid(cell.radialDeviationMM)
            || !Valid(cell.orientedMapJacobian)
            || cell.normalEquationDerivative.upper >= -0.75
            || cell.radialDeviationMM.lower < -PositionalEpsilonMM
            || cell.radialDeviationMM.upper > PositionalEpsilonMM
            || cell.orientedMapJacobian.lower <= 0
            || !cell.intendedSpineBracketIsUnique || !cell.otherSpineIntervalsDischarged)
            return SurfaceProofRefusal::SurfaceMap;
    }
    if (!std::all_of(surface.orderedQuarterCoverage.begin(),
                     surface.orderedQuarterCoverage.end(), [](bool value) { return value; })
        || !surface.sharedMeridiansMatch || !surface.pathBoundaryCoveredOnce)
        return SurfaceProofRefusal::QuarterCoverage;
    if ((closure == ClosureKind::OpenFlatCaps && !surface.openCapsPlanarAndOriented)
        || (closure == ClosureKind::ClosedNoCaps && !surface.closedHasNoCaps))
        return SurfaceProofRefusal::Boundary;
    if (!Valid(measure.sourceLengthMM) || !Valid(measure.expectedVolumeMM3)
        || !Valid(measure.measuredVolumeMM3)
        || measure.expectedVolumeMM3.upper < measure.measuredVolumeMM3.lower
        || measure.measuredVolumeMM3.upper < measure.expectedVolumeMM3.lower
        || !measure.sourcePolynomialIntegratedIndependently
        || !measure.everyLocalSectionAccountedFor || !measure.markedMeridianMatches)
        return SurfaceProofRefusal::IndependentMeasure;
    for (const auto& radius : measure.stationRadiusMM)
        if (!Valid(radius) || radius.lower <= 0) return SurfaceProofRefusal::IndependentMeasure;
    return SurfaceProofRefusal::None;
}

inline double IdealCircularSweepVolume(double lengthMM, double startRadiusMM,
                                       double endRadiusMM) noexcept {
    if (!Finite(lengthMM) || !Finite(startRadiusMM) || !Finite(endRadiusMM)
        || lengthMM <= 0 || startRadiusMM <= 0 || endRadiusMM <= 0)
        return std::numeric_limits<double>::quiet_NaN();
    return Pi * lengthMM * (startRadiusMM * startRadiusMM
        + startRadiusMM * endRadiusMM + endRadiusMM * endRadiusMM) / 3;
}
} // namespace core3d::spatial_sweep
