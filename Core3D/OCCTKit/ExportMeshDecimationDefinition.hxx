#pragma once
// F2 contract seed. This describes a source-bound export derivative only;
// it is not a BRep modifier, a tessellation preset, or a vertex-edit API.
#include "PublishDerivativeDefinition.hxx"
#include <cstdint>
#include <string>
#include <vector>

namespace core3d::export_decimation {

inline constexpr char kSchemaVersion[] = "shapeyard.export-mesh-decimation.v1";
inline constexpr char kAlgorithmID[] = "qem-edge-collapse.fixed-order.v1";
inline constexpr char kRemapSchemaVersion[] = "shapeyard.decimation-provenance-remap.v1";
inline constexpr std::uint32_t kMinimumClosedMeshTriangles = 4;
inline constexpr std::uint32_t kMaximumTriangleTarget = 4U * 1024U * 1024U;
inline constexpr std::uint32_t kMaximumMeasurementSamples = 4U * 1024U * 1024U;
inline constexpr std::uint32_t kMaximumSilhouetteResolution = 4096;

enum class InputClass : std::uint8_t { PositionOnlyUntexturedStatic = 1 };
enum class DerivativeOutcome : std::uint8_t {
    Current, RejectedTargetBelowClosedMeshMinimum, RejectedUnreachableTriangleBudget,
    RejectedQualityLimit, RejectedInputNotClosedTwoManifold,
    RejectedAttributesOutOfScope, RejectedMalformedInput, Cancelled, RecoveryRequired
};

// All values are persisted as canonical settings bytes and included in
// settingsSHA256.  No execution-time default may change them.
struct QualitySettings {
    double maximumSymmetricSurfaceDistanceMM = 0;
    double maximumSilhouettePixelDeviation = 0;
    double hardEdgeAngleDegrees = 0;
    double contactClearanceMM = 0;
    std::uint32_t measurementSamplesPerMesh = 0;
    std::uint32_t silhouetteResolution = 0;
    std::string silhouetteCameraSetSHA256;
};

struct Settings {
    std::string schemaVersion = kSchemaVersion;
    std::string algorithmID = kAlgorithmID;
    InputClass inputClass = InputClass::PositionOnlyUntexturedStatic;
    std::uint32_t targetTriangleCount = 0; // Maximum emitted triangle count.
    QualitySettings quality;
    std::string settingsSHA256; // SHA-256 of canonical complete Settings bytes.
};

// This replaces, rather than reuses, SourceFaceProvenanceRecord's ordered
// face ranges. Contribution tokens are SHA-256 content witnesses made from
// the source artifact/face evidence and are never topology ordinals.
struct RemappedTriangleProvenance {
    std::string outputTriangleContentSHA256;
    std::vector<std::string> sortedSourceContributionSHA256;
};

struct RemappedProvenanceReceipt {
    std::string schemaVersion = kRemapSchemaVersion;
    std::string sourceArtifactSHA256;
    std::string sourceGeometrySHA256;
    std::string sourceFaceProvenanceSHA256;
    std::string outputGeometrySHA256;
    std::vector<RemappedTriangleProvenance> outputTrianglesInEmittedOrder;
    std::string receiptSHA256;
};

struct SourceReceipt {
    publish::PublishSourceBinding source;
    std::string sourceRevisionSHA256;
    std::string sourceArtifactSHA256; // F1's completed, independently parsed GLB.
    std::string sourceGeometrySHA256;
    std::string sourceFaceProvenanceSHA256;
};

struct MeasurementReceipt {
    std::string methodVersion; // F1 emitted-artifact parser/measurement version.
    std::string sourceArtifactSHA256;
    std::string outputArtifactSHA256;
    double sourceToOutputMaxMM = 0;
    double outputToSourceMaxMM = 0;
    double silhouettePixelDeviation = 0;
    std::uint64_t measuredBoundaryEdges = 0;
    std::uint64_t measuredNonManifoldEdges = 0;
    std::uint64_t measuredInconsistentWindingEdges = 0;
    std::uint64_t measuredDegenerateTriangles = 0;
    std::uint64_t measuredSelfContacts = 0;
    std::uint64_t measuredHardEdgeFailures = 0;
    std::string evidenceSHA256;
};

struct DerivativeDefinition {
    std::string schemaVersion = kSchemaVersion;
    SourceReceipt source;
    Settings settings;
    std::string algorithmBuildSHA256;
    std::string outputArtifactSHA256;
    std::string outputGeometrySHA256;
    std::uint32_t emittedTriangleCount = 0;
    RemappedProvenanceReceipt remappedProvenance;
    MeasurementReceipt independentMeasurement;
    DerivativeOutcome outcome = DerivativeOutcome::RejectedMalformedInput;
};

inline bool IsSettingsStructurallyValid(const Settings& value) noexcept {
    const QualitySettings& quality = value.quality;
    return value.schemaVersion == kSchemaVersion && value.algorithmID == kAlgorithmID
        && value.inputClass == InputClass::PositionOnlyUntexturedStatic
        && value.targetTriangleCount >= kMinimumClosedMeshTriangles
        && value.targetTriangleCount <= kMaximumTriangleTarget
        && quality.maximumSymmetricSurfaceDistanceMM > 0
        && quality.maximumSilhouettePixelDeviation >= 0
        && quality.hardEdgeAngleDegrees > 0 && quality.hardEdgeAngleDegrees < 180
        && quality.contactClearanceMM >= 0
        && quality.measurementSamplesPerMesh > 0
        && quality.measurementSamplesPerMesh <= kMaximumMeasurementSamples
        && quality.silhouetteResolution > 0
        && quality.silhouetteResolution <= kMaximumSilhouetteResolution
        && quality.silhouetteCameraSetSHA256.size() == 64
        && value.settingsSHA256.size() == 64;
}

inline bool IsCurrentDerivative(const DerivativeDefinition& value,
                                const std::string& recapturedSourceRevisionSHA256,
                                const std::string& recapturedSettingsSHA256) noexcept {
    return value.outcome == DerivativeOutcome::Current
        && value.source.sourceRevisionSHA256 == recapturedSourceRevisionSHA256
        && value.settings.settingsSHA256 == recapturedSettingsSHA256
        && value.outputArtifactSHA256.size() == 64
        && value.remappedProvenance.sourceArtifactSHA256 == value.source.sourceArtifactSHA256
        && value.remappedProvenance.outputGeometrySHA256 == value.outputGeometrySHA256;
}

} // namespace core3d::export_decimation
