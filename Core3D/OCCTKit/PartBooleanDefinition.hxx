#pragma once

// Codec-2 values are persistence-only.  They describe the one P1 shell/profile
// composition and grant no builder, proof, editor, or native admission right.
#include "RetainedRecipeIdentity.hxx"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace core3d::part_boolean {
using retained_recipe::Digest;
using retained_recipe::UUID;

inline constexpr std::uint32_t FeatureKind = 1;
inline constexpr std::uint32_t LegacyCodecVersion = 1; // opaque transport remains unchanged
inline constexpr std::uint32_t ShellCodecVersion = 2;
inline constexpr std::uint32_t PayloadVersion = 2;
inline constexpr std::uint32_t AnalyticPayloadVersion = 1;
inline constexpr char AnalyticOwner[] = "part_boolean::analytic_owner_v1";
inline constexpr std::size_t MaximumPayloadBytes = 16 * 1024;
inline constexpr std::size_t MaximumMaterials = 64;
inline constexpr std::size_t MaximumRegions = 128;
inline constexpr std::size_t MaximumSelectors = 128;
inline constexpr std::size_t MaximumGroupsPerInput = 16;
inline constexpr std::size_t MaximumAliases = 8;
inline constexpr std::size_t MaximumTextBytes = 128;

enum class Operation : std::uint8_t { Union = 1, Subtract = 2, Intersect = 3 };
enum class InputFamily : std::uint8_t {
    ShellProfile = 1,
    AnalyticRectangularPrism = 2,
};
enum class MaterialPolicy : std::uint8_t { RejectConflictingMerges = 1 };
enum class MaterialKind : std::uint8_t { ResolvedScalars = 1, OwnedResource = 2 };
enum class RegionKind : std::uint8_t {
    OuterWall = 1, InnerWall = 2, Floor = 3, Rim = 4, ToolBoundary = 5,
};
enum class SelectorPolicy : std::uint8_t { UniqueFace = 1, WholeRegion = 2 };

struct CodecVersions final {
    std::uint32_t serializer = 1;
    std::uint32_t build = 1;
    std::uint32_t proof = 1;
    std::uint32_t selector = 1;
    std::uint32_t material = 1;
};

struct SignatureCommitments final {
    Digest geometry{}, recipe{}, placement{}, material{}, groups{};
};

struct MaterialValue final {
    UUID identifier{};
    MaterialKind kind = MaterialKind::ResolvedScalars;
    std::array<double, 4> baseColorSRGB{{0, 0, 0, 1}};
    double metallic = 0;
    double roughness = 1;
    UUID resource{};
    Digest resourceDigest{};
};

struct InputBinding final {
    UUID rootNode{};
    UUID originalSourceFeature{};
    InputFamily family = InputFamily::ShellProfile;
    SignatureCommitments commitments;
    MaterialValue originalMaterial;
    std::string originalName;
    std::vector<std::string> originalGroups;
    bool originallyVisible = true;
};

struct RegionBinding final {
    UUID region{};
    std::uint8_t sourceInput = 0;
    RegionKind kind = RegionKind::OuterWall;
    std::uint16_t materialIndex = 0;
};

struct SelectorBinding final {
    UUID selector{};
    UUID region{};
    SelectorPolicy policy = SelectorPolicy::WholeRegion;
    std::uint16_t expectedCardinality = 1;
};

struct ShellAdoptionAlias final {
    UUID alias{};
    UUID originalSourceFeature{};
    std::uint8_t sourceInput = 0;
    std::uint32_t shellStepIndex = 0;
};

struct Definition final {
    Operation operation = Operation::Union;
    std::array<InputBinding, 2> inputs;
    MaterialPolicy materialPolicy = MaterialPolicy::RejectConflictingMerges;
    CodecVersions versions;
    std::vector<MaterialValue> materials;
    std::vector<RegionBinding> regions;
    std::vector<SelectorBinding> selectors;
    std::vector<ShellAdoptionAlias> shellAliases;
    // Persisted values may be decoded before a native rule exists.  Admission
    // is deliberately not a wire-controlled switch and must remain false.
    bool nativeAdmissionEnabled = false;
};

// SYPB/1 is a strict native meaning inside the reserved codec-1 byte field.
// It is never inferred from an arbitrary old codec-1 payload.
struct AnalyticPrismInput final {
    UUID rootNode{};
    UUID originalSourceFeature{};
    SignatureCommitments commitments;
    std::array<double, 3> dimensions{{0, 0, 0}};
    std::array<double, 3> translation{{0, 0, 0}};
    std::array<double, 4> rotationXYZW{{0, 0, 0, 1}};
    double metersPerUnit = 0;
    MaterialValue originalMaterial;
    std::string originalName;
    std::vector<std::string> originalGroups;
    bool originallyVisible = true;
};

struct AnalyticDefinition final {
    Operation operation = Operation::Union;
    std::array<AnalyticPrismInput, 2> inputs;
    MaterialPolicy materialPolicy = MaterialPolicy::RejectConflictingMerges;
    CodecVersions versions;
    bool nativeAdmissionEnabled = false;
};

inline bool Zero(const UUID& value) noexcept {
    return !retained_recipe::Nonzero(value);
}
inline bool Zero(const Digest& value) noexcept {
    return !retained_recipe::Nonzero(value);
}
inline bool ValidText(const std::string& value) noexcept {
    return value.size() <= MaximumTextBytes
        && std::none_of(value.begin(), value.end(), [](unsigned char byte) {
            return byte == 0 || byte < 0x20 || byte == 0x7f;
        });
}
inline bool Valid(const SignatureCommitments& value) noexcept {
    return retained_recipe::Nonzero(value.geometry)
        && retained_recipe::Nonzero(value.recipe)
        && retained_recipe::Nonzero(value.placement)
        && retained_recipe::Nonzero(value.material)
        && retained_recipe::Nonzero(value.groups);
}
inline bool Valid(const MaterialValue& value) noexcept {
    if (!retained_recipe::Nonzero(value.identifier)) return false;
    for (double scalar : value.baseColorSRGB)
        if (!std::isfinite(scalar) || scalar < 0 || scalar > 1) return false;
    if (!std::isfinite(value.metallic) || value.metallic < 0 || value.metallic > 1
        || !std::isfinite(value.roughness) || value.roughness < 0 || value.roughness > 1) return false;
    if (value.kind == MaterialKind::ResolvedScalars)
        return Zero(value.resource) && Zero(value.resourceDigest);
    if (value.kind == MaterialKind::OwnedResource)
        return retained_recipe::Nonzero(value.resource)
            && retained_recipe::Nonzero(value.resourceDigest);
    return false;
}
inline bool Valid(const InputBinding& value) noexcept {
    if (!retained_recipe::Nonzero(value.rootNode)
        || !retained_recipe::Nonzero(value.originalSourceFeature)
        || !Valid(value.commitments) || !Valid(value.originalMaterial)
        || !ValidText(value.originalName)
        || value.originalGroups.size() > MaximumGroupsPerInput) return false;
    std::set<std::string> groups;
    for (const std::string& group : value.originalGroups)
        if (group.empty() || !ValidText(group) || !groups.insert(group).second) return false;
    return value.family == InputFamily::ShellProfile
        || value.family == InputFamily::AnalyticRectangularPrism;
}
inline bool Valid(const Definition& value) noexcept {
    try {
        if (value.nativeAdmissionEnabled
            || (value.operation != Operation::Union && value.operation != Operation::Subtract
                && value.operation != Operation::Intersect)
            || value.materialPolicy != MaterialPolicy::RejectConflictingMerges
            || value.versions.serializer != 1 || value.versions.build != 1
            || value.versions.proof != 1 || value.versions.selector != 1
            || value.versions.material != 1
            || !Valid(value.inputs[0]) || !Valid(value.inputs[1])
            || value.inputs[0].rootNode == value.inputs[1].rootNode
            || value.inputs[0].originalSourceFeature == value.inputs[1].originalSourceFeature
            || value.materials.empty() || value.materials.size() > MaximumMaterials
            || value.regions.empty() || value.regions.size() > MaximumRegions
            || value.selectors.size() > MaximumSelectors
            || value.shellAliases.size() != 1 || value.shellAliases.size() > MaximumAliases) return false;
        const bool leftShell = value.inputs[0].family == InputFamily::ShellProfile;
        const bool rightShell = value.inputs[1].family == InputFamily::ShellProfile;
        if (leftShell == rightShell) return false;
        std::set<UUID> materialIDs;
        for (const MaterialValue& material : value.materials)
            if (!Valid(material) || !materialIDs.insert(material.identifier).second) return false;
        std::set<UUID> regions;
        for (const RegionBinding& region : value.regions)
            if (!retained_recipe::Nonzero(region.region) || !regions.insert(region.region).second
                || region.sourceInput > 1 || region.materialIndex >= value.materials.size()
                || (region.kind != RegionKind::OuterWall && region.kind != RegionKind::InnerWall
                    && region.kind != RegionKind::Floor && region.kind != RegionKind::Rim
                    && region.kind != RegionKind::ToolBoundary)) return false;
        std::set<UUID> selectors;
        for (const SelectorBinding& selector : value.selectors)
            if (!retained_recipe::Nonzero(selector.selector)
                || !selectors.insert(selector.selector).second || !regions.count(selector.region)
                || selector.expectedCardinality == 0
                || (selector.policy != SelectorPolicy::UniqueFace
                    && selector.policy != SelectorPolicy::WholeRegion)
                || (selector.policy == SelectorPolicy::UniqueFace
                    && selector.expectedCardinality != 1)) return false;
        const ShellAdoptionAlias& alias = value.shellAliases.front();
        const std::uint8_t shellIndex = leftShell ? 0 : 1;
        return retained_recipe::Nonzero(alias.alias)
            && alias.sourceInput == shellIndex && alias.shellStepIndex == 0
            && alias.originalSourceFeature == value.inputs[shellIndex].originalSourceFeature;
    } catch (...) { return false; }
}
inline bool Valid(const AnalyticPrismInput& value) noexcept {
    if (!retained_recipe::Nonzero(value.rootNode)
        || !retained_recipe::Nonzero(value.originalSourceFeature)
        || !Valid(value.commitments) || !Valid(value.originalMaterial)
        || value.originalMaterial.kind != MaterialKind::ResolvedScalars
        || !ValidText(value.originalName)
        || value.originalGroups.size() > MaximumGroupsPerInput
        || !std::isfinite(value.metersPerUnit) || value.metersPerUnit <= 0) return false;
    for (double dimension : value.dimensions)
        if (!std::isfinite(dimension) || dimension <= 0) return false;
    for (double coordinate : value.translation)
        if (!std::isfinite(coordinate)) return false;
    double norm = 0;
    for (double coordinate : value.rotationXYZW) {
        if (!std::isfinite(coordinate)) return false;
        norm += coordinate * coordinate;
    }
    if (std::abs(norm - 1.0) > 1e-12) return false;
    std::set<std::string> groups;
    for (const auto& group : value.originalGroups)
        if (group.empty() || !ValidText(group) || !groups.insert(group).second) return false;
    return true;
}
inline bool Valid(const AnalyticDefinition& value) noexcept {
    return !value.nativeAdmissionEnabled
        && value.materialPolicy == MaterialPolicy::RejectConflictingMerges
        && (value.operation == Operation::Union || value.operation == Operation::Subtract
            || value.operation == Operation::Intersect)
        && value.versions.serializer == 1 && value.versions.build == 1
        && value.versions.proof == 1 && value.versions.selector == 1
        && value.versions.material == 1 && Valid(value.inputs[0]) && Valid(value.inputs[1])
        && value.inputs[0].rootNode != value.inputs[1].rootNode
        && value.inputs[0].originalSourceFeature != value.inputs[1].originalSourceFeature;
}
} // namespace core3d::part_boolean
