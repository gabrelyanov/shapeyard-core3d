#pragma once

#include "AttachmentRigMetadata.hxx"
#include "ExportMeshDecimationDefinition.hxx"

#include <algorithm>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

// F3 retained LOD-set seam. Every level is an F2 export derivative of the
// same retained source. This value never replaces or mutates the BRep recipe.
namespace core3d::retained_lod {

inline constexpr char kSchemaVersion[] = "shapeyard.retained-lod-set.v1";
inline constexpr char kManifestSchemaVersion[] = "shapeyard.lod-artifact-manifest.v1";
inline constexpr std::size_t kMaximumLevels = 8; // Internal allocation bound, not a destination rule.

enum class LevelState : std::uint8_t { Unbuilt, Current, Stale, Failed };
enum class Refusal : std::uint8_t {
    None, InvalidDefinition, StaleSource, PartialSet, BudgetExceeded,
    NonMonotoneTriangleCounts, AttachmentParityFailure, ArtifactCollision
};

struct AttachmentEvidence {
    std::string attachmentID;
    std::string canonicalMetreFrameSHA256;
};

struct Level {
    std::string levelID; // Native-issued and stable across setting edits/rebuilds.
    std::string displayName;
    std::uint32_t triangleBudget = 0;
    export_decimation::QualitySettings quality;
    std::string settingsSHA256;
    std::string artifactName;
    std::string artifactSHA256;
    std::uint32_t measuredTriangles = 0;
    std::vector<AttachmentEvidence> attachments;
    export_decimation::DerivativeDefinition derivative;
    LevelState state = LevelState::Unbuilt;
    std::string failure;
};

struct Definition {
    std::string schemaVersion = kSchemaVersion;
    std::string lodSetID;
    publish::PublishSourceBinding source;
    std::string boundSourceRevisionSHA256;
    std::string observedSourceRevisionSHA256;
    std::string attachmentMetadataSHA256;
    std::vector<AttachmentEvidence> expectedAttachmentsInCanonicalOrder;
    std::vector<Level> levels;
};

struct ArtifactMember {
    std::string levelID;
    std::string artifactName;
    std::string artifactSHA256;
    std::uint32_t triangles = 0;
    std::uint32_t triangleBudget = 0;
    std::string qualitySettingsSHA256;
    std::vector<AttachmentEvidence> attachments;
};

struct Manifest {
    std::string schemaVersion = kManifestSchemaVersion;
    std::string lodSetID;
    std::string sourceRevisionSHA256;
    std::string attachmentMetadataSHA256;
    std::vector<ArtifactMember> levels;
};

inline bool IsSHA256(const std::string& value) noexcept { return value.size() == 64; }

inline bool HasCanonicalAttachmentParity(const Definition& value, const Level& level) noexcept {
    if (level.attachments.size() != value.expectedAttachmentsInCanonicalOrder.size()) return false;
    for (std::size_t index = 0; index < level.attachments.size(); ++index) {
        const AttachmentEvidence& expected = value.expectedAttachmentsInCanonicalOrder[index];
        if (level.attachments[index].attachmentID != expected.attachmentID
            || level.attachments[index].canonicalMetreFrameSHA256 != expected.canonicalMetreFrameSHA256
            || !IsSHA256(level.attachments[index].canonicalMetreFrameSHA256)) return false;
    }
    return true;
}

inline bool HasSafeGLBName(const std::string& value) noexcept {
    return value.size() > 4 && value.substr(value.size() - 4) == ".glb"
        && value.find('/') == std::string::npos && value.find('\\') == std::string::npos
        && value != ".glb";
}

inline void ObserveSourceRevision(Definition& value, const std::string& revisionSHA256) noexcept {
    value.observedSourceRevisionSHA256 = revisionSHA256;
    if (revisionSHA256 == value.boundSourceRevisionSHA256) return;
    for (Level& level : value.levels) {
        level.state = LevelState::Stale;
        level.failure = "sourceRevisionChanged";
    }
}

inline bool ValidateForExport(const Definition& value, Refusal& refusal) noexcept {
    refusal = Refusal::InvalidDefinition;
    if (value.schemaVersion != kSchemaVersion || value.lodSetID.empty()
        || !publish::IsStrictlySortedOwners(value.source)
        || !IsSHA256(value.boundSourceRevisionSHA256)
        || value.boundSourceRevisionSHA256 != value.observedSourceRevisionSHA256
        || !IsSHA256(value.attachmentMetadataSHA256)
        || value.expectedAttachmentsInCanonicalOrder.empty()
        || value.expectedAttachmentsInCanonicalOrder.size() > attachment_rig::kMaxAttachments
        || value.levels.size() < 2 || value.levels.size() > kMaximumLevels) {
        if (value.boundSourceRevisionSHA256 != value.observedSourceRevisionSHA256) refusal = Refusal::StaleSource;
        return false;
    }
    for (std::size_t index = 0; index < value.expectedAttachmentsInCanonicalOrder.size(); ++index) {
        const AttachmentEvidence& expected = value.expectedAttachmentsInCanonicalOrder[index];
        if (expected.attachmentID.empty() || !IsSHA256(expected.canonicalMetreFrameSHA256)
            || (index != 0 && value.expectedAttachmentsInCanonicalOrder[index - 1].attachmentID
                >= expected.attachmentID)) return false;
    }
    std::set<std::string> ids, names, hashes;
    std::uint32_t previousTriangles = 0, previousBudget = 0;
    for (std::size_t index = 0; index < value.levels.size(); ++index) {
        const Level& level = value.levels[index];
        if (level.levelID.empty() || !ids.insert(level.levelID).second || level.displayName.empty()
            || level.triangleBudget < export_decimation::kMinimumClosedMeshTriangles
            || level.triangleBudget > export_decimation::kMaximumTriangleTarget
            || !IsSHA256(level.settingsSHA256) || !HasSafeGLBName(level.artifactName)
            || !names.insert(level.artifactName).second || !IsSHA256(level.artifactSHA256)
            || !hashes.insert(level.artifactSHA256).second) return false;
        if (level.state != LevelState::Current
            || !export_decimation::IsCurrentDerivative(level.derivative,
                                                        value.boundSourceRevisionSHA256,
                                                        level.settingsSHA256)
            || level.derivative.settings.targetTriangleCount != level.triangleBudget
            || level.derivative.outputArtifactSHA256 != level.artifactSHA256
            || level.derivative.emittedTriangleCount != level.measuredTriangles) {
            refusal = Refusal::PartialSet; return false;
        }
        if (level.measuredTriangles > level.triangleBudget || level.measuredTriangles == 0) {
            refusal = Refusal::BudgetExceeded; return false;
        }
        if (index != 0 && (level.triangleBudget >= previousBudget
                           || level.measuredTriangles >= previousTriangles)) {
            refusal = Refusal::NonMonotoneTriangleCounts; return false;
        }
        if (!HasCanonicalAttachmentParity(value, level)) {
            refusal = Refusal::AttachmentParityFailure; return false;
        }
        previousTriangles = level.measuredTriangles;
        previousBudget = level.triangleBudget;
    }
    refusal = Refusal::None;
    return true;
}

inline bool BuildManifest(const Definition& value, Manifest& manifest, Refusal& refusal) noexcept {
    if (!ValidateForExport(value, refusal)) return false;
    Manifest candidate;
    candidate.lodSetID = value.lodSetID;
    candidate.sourceRevisionSHA256 = value.boundSourceRevisionSHA256;
    candidate.attachmentMetadataSHA256 = value.attachmentMetadataSHA256;
    for (const Level& level : value.levels) {
        candidate.levels.push_back({level.levelID, level.artifactName, level.artifactSHA256,
                                    level.measuredTriangles, level.triangleBudget,
                                    level.settingsSHA256, level.attachments});
    }
    manifest = std::move(candidate);
    return true;
}

// The caller builds every candidate level off-document. Nothing is installed
// unless the complete candidate set passes. Stable IDs come from the existing
// definition, never from the F2 executor or an artifact filename.
inline bool InstallAtomicRebuild(Definition& value, std::vector<Level> candidateLevels,
                                 Refusal& refusal) noexcept {
    if (candidateLevels.size() != value.levels.size()) { refusal = Refusal::PartialSet; return false; }
    for (std::size_t index = 0; index < candidateLevels.size(); ++index)
        if (candidateLevels[index].levelID != value.levels[index].levelID) {
            refusal = Refusal::InvalidDefinition; return false;
        }
    Definition candidate = value;
    candidate.levels = std::move(candidateLevels);
    if (!ValidateForExport(candidate, refusal)) return false;
    value = std::move(candidate);
    return true;
}

} // namespace core3d::retained_lod
