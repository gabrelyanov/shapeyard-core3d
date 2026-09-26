#pragma once

// K2 detached-result gate. The concrete adapter is required to use the
// GeomFill_CurveAndTrihedron/EvolvedSection/Sweep route with four known
// rational circular quarters. This header deliberately has no corrected-
// Frenet MakePipeShell fallback and mutates no OCAF label.
#include "SpatialSweepAdmission.hxx"
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace core3d::spatial_sweep {
inline constexpr std::size_t MaximumSpatialSweepFaces = 1026;
inline constexpr std::size_t MaximumSpatialSweepEdges = 4096;

struct TopologyCertificate {
    std::size_t solidCount = 0;
    std::size_t shellCount = 0;
    std::size_t faceCount = 0;
    std::size_t edgeCount = 0;
    bool connectedFaceGraph = false;
    bool everyShellEdgeHasTwoOrientedUses = false;
    bool noExtraSubshapes = false;
    bool forwardOriented = false;
    bool closedShell = false;
    bool brepCheckValid = false;
    bool selfInterferenceCheckedSerially = false;
    bool selfInterferenceFree = false;
    bool sewingHasNoFreeOrMultipleEdges = false;
};

struct DetachedKernelReceipt {
    SurfaceCertificate surface;
    IndependentMeasureCertificate independentMeasure;
    TopologyCertificate topology;
    Digest geometryBinaryDigest{};
    std::uint32_t serializerFormatVersion = 0;
    bool usedBishopTrihedron = false;
    bool usedFourKnownQuarterSections = false;
    bool withKpartDisabled = false;
    bool forceApproxC1Disabled = false;
    bool deterministicSpanOrder = false;
};

enum class SolidRefusal : std::uint8_t {
    None = 0,
    Admission,
    KernelRoute,
    SurfaceProof,
    OpenShell,
    ExtraSolid,
    TopologyBudget,
    InvalidSolid,
    SelfInterference,
    MissingGeometryDigest,
};

struct DetachedSolidResult {
    SolidRefusal refusal = SolidRefusal::Admission;
    Digest geometryBinaryDigest{};
    std::uint32_t serializerFormatVersion = 0;
    bool admitted() const noexcept { return refusal == SolidRefusal::None; }
};

inline DetachedSolidResult ValidateDetachedSolid(const Definition& definition,
                                                 const PreparedAdmission& admission,
                                                 const DetachedKernelReceipt& candidate) noexcept {
    DetachedSolidResult result;
    if (!admission.admitted() || Validate(definition) != Refusal::None) return result;
    if (!candidate.usedBishopTrihedron || !candidate.usedFourKnownQuarterSections
        || !candidate.withKpartDisabled || !candidate.forceApproxC1Disabled
        || !candidate.deterministicSpanOrder) {
        result.refusal = SolidRefusal::KernelRoute; return result;
    }
    if (ValidateSurfaceProof(candidate.surface, candidate.independentMeasure,
                             definition.closure) != SurfaceProofRefusal::None) {
        result.refusal = SolidRefusal::SurfaceProof; return result;
    }
    const auto& topology = candidate.topology;
    if (!topology.sewingHasNoFreeOrMultipleEdges || !topology.closedShell) {
        result.refusal = SolidRefusal::OpenShell; return result;
    }
    if (topology.solidCount != 1 || topology.shellCount != 1) {
        result.refusal = SolidRefusal::ExtraSolid; return result;
    }
    if (topology.faceCount == 0 || topology.faceCount > MaximumSpatialSweepFaces
        || topology.edgeCount == 0 || topology.edgeCount > MaximumSpatialSweepEdges) {
        result.refusal = SolidRefusal::TopologyBudget; return result;
    }
    if (!topology.connectedFaceGraph || !topology.everyShellEdgeHasTwoOrientedUses
        || !topology.noExtraSubshapes || !topology.forwardOriented
        || !topology.brepCheckValid) {
        result.refusal = SolidRefusal::InvalidSolid; return result;
    }
    if (!topology.selfInterferenceCheckedSerially || !topology.selfInterferenceFree) {
        result.refusal = SolidRefusal::SelfInterference; return result;
    }
    if (!retained_recipe::Nonzero(candidate.geometryBinaryDigest)
        || candidate.serializerFormatVersion == 0) {
        result.refusal = SolidRefusal::MissingGeometryDigest; return result;
    }
    result.refusal = SolidRefusal::None;
    result.geometryBinaryDigest = candidate.geometryBinaryDigest;
    result.serializerFormatVersion = candidate.serializerFormatVersion;
    return result;
}
} // namespace core3d::spatial_sweep
