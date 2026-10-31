#pragma once

// Codec-2 values are persistence-only.  They describe the one P1 shell/profile
// composition and grant no builder, proof, editor, or native admission right.
#include "RetainedRecipeIdentity.hxx"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace core3d::part_boolean {
using retained_recipe::Digest;
using retained_recipe::UUID;

inline constexpr std::uint32_t FeatureKind = 1;
inline constexpr std::uint32_t LegacyCodecVersion = 1; // opaque transport remains unchanged
inline constexpr std::uint32_t ShellCodecVersion = 2;
inline constexpr std::uint32_t RecipeCodecVersion = 3;
inline constexpr std::uint32_t OriginalsCodecVersion = 4;
// SYPB/5 is readable retained-program operand state. It does not grant
// execution authority and does not alter any codec-1 through codec-4 bytes.
inline constexpr std::uint32_t ProgramOperandCodecVersion = 5;
inline constexpr std::uint32_t PayloadVersion = 2;
inline constexpr std::uint32_t AnalyticPayloadVersion = 1;
inline constexpr std::uint32_t RecipePayloadVersion = 3;
inline constexpr std::uint32_t OriginalsPayloadVersion = 4;
inline constexpr char AnalyticOwner[] = "part_boolean::analytic_owner_v1";
inline constexpr std::size_t MaximumPayloadBytes = 16 * 1024;
inline constexpr std::size_t MaximumMaterials = 64;
inline constexpr std::size_t MaximumRegions = 128;
inline constexpr std::size_t MaximumSelectors = 128;
inline constexpr std::size_t MaximumGroupsPerInput = 16;
inline constexpr std::size_t MaximumAliases = 8;
inline constexpr std::size_t MaximumTextBytes = 128;
inline constexpr std::size_t MaximumUTF16CodeUnits = 128;
inline constexpr std::size_t MaximumMetadataRows = 128;
inline constexpr std::size_t MaximumRecipeBytes = 4096;

enum class Operation : std::uint8_t { Union = 1, Subtract = 2, Intersect = 3 };
enum class InputFamily : std::uint8_t {
    ShellProfile = 1,
    AnalyticRectangularPrism = 2,
    FrozenSolid = 3,
};
enum class RecipeInputRole : std::uint8_t { Left = 1, Right = 2 };
enum class RecipeInputFamily : std::uint8_t {
    Profile = 1,
    Enclosure = 2,
    RectangularLoft = 3,
    // Match composite_recipe::RecipeKind::OrdinarySource on the shared wire.
    // Existing values 1-3 retain their historical meanings.
    OrdinarySource = 0x12,
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

// SYPB/3 is a persistence-only copy of the two complete native source
// recipes. It intentionally has no builder/proof/editor flag and is kept
// distinct from the frozen SYPB/1 and SYPB/2 input families.
struct RecipeInput final {
    RecipeInputRole role = RecipeInputRole::Left;
    RecipeInputFamily family = RecipeInputFamily::Profile;
    bool originallyVisible = true;
    UUID sourceNode{};
    std::uint64_t sourceLocalID = 0;
    retained_recipe::SourceIdentity original;
    std::uint32_t shapeSlot = 0;
    std::uint32_t recipeSchema = 0;
    double sourceMetersPerUnit = 0;
    double carrierMetersPerUnit = 0;
    std::array<double, 16> inputToCarrier{{
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
    }};
    SignatureCommitments commitments;
    MaterialValue originalMaterial;
    std::string originalName;
    std::vector<std::string> originalGroups;
    std::vector<double> recipeScalars;
};

struct RecipeDefinition final {
    Operation operation = Operation::Union;
    MaterialPolicy materialPolicy = MaterialPolicy::RejectConflictingMerges;
    CodecVersions versions;
    retained_recipe::OwnerKey owner;
    UUID booleanNode{};
    UUID feature{};
    std::array<RecipeInput, 2> inputs;
    bool nativeAdmissionEnabled = false;
};

// SYPB/4 is an immutable-originals persistence record. These types are kept
// separate from AnalyticDefinition so an effective edit cannot rewrite an
// original projection merely because both contain prism scalars.
enum class OriginalsAvailability : std::uint8_t {
    LegacyNotRecorded = 0,
    Complete = 1,
};

inline constexpr OriginalsAvailability AvailabilityForCodec(
    std::uint32_t codecVersion) noexcept {
    return codecVersion == OriginalsCodecVersion
        ? OriginalsAvailability::Complete
        : OriginalsAvailability::LegacyNotRecorded;
}

struct UTF16Name final {
    bool attributePresent = false;
    std::vector<std::uint16_t> codeUnits;
};

struct OccurrenceState final {
    std::uint8_t encoding = 1;
    std::uint8_t presenceMask = 0;
    // translation XYZ, quaternion XYZW, scale. Absent slots are zero.
    std::array<double, 8> values{{0, 0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 16> derivedMatrix{{
        1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
    }};
};

struct ScalarMaterialRow final {
    std::uint16_t ordinal = 0;
    std::string labelEntry;
    std::int32_t faceCulling = 0;
    std::int32_t alphaMode = 0;
    double alphaCutoff = 0;
    bool pbrDefined = false;
    bool commonDefined = false;
    // PBR: base RGBA, emissive RGB, metallic, roughness, refraction index.
    std::array<double, 10> pbr{{}};
    // Common: ambient/diffuse/specular/emissive RGB, shininess, transparency.
    std::array<double, 14> common{{}};
};

struct AppearanceState final {
    bool legacyMaterialPresent = false;
    std::int32_t legacyMaterial = 0;
    bool legacyColorPresent = false;
    std::int32_t legacyColor = 0;
    bool localPBRMarkerPresent = false;
    std::optional<std::uint16_t> material;
};

struct GroupRow final {
    std::uint16_t ordinal = 0;
    std::string identifier;
    UTF16Name name;
    bool originPresent = false;
    std::array<double, 3> origin{{0, 0, 0}};
    std::vector<retained_recipe::OwnerKey> members;
};

struct LayerRow final {
    std::uint16_t ordinal = 0;
    std::string labelEntry;
    UTF16Name name;
    bool invisibleAttributePresent = false;
};

struct CompleteMetadata final {
    UTF16Name name;
    OccurrenceState occurrence;
    AppearanceState appearance;
    std::vector<std::uint16_t> groups;
    bool invisibleAttributePresent = false;
    bool layerLinkPresent = false;
    std::vector<std::uint16_t> layers;
};

struct OriginalAnalyticProjection final {
    std::array<double, 3> dimensions{{0, 0, 0}};
    std::array<double, 3> translation{{0, 0, 0}};
    std::array<double, 4> rotationXYZW{{0, 0, 0, 1}};
};

struct EffectiveAnalyticGeometry final {
    std::array<double, 3> dimensions{{0, 0, 0}};
    std::array<double, 3> translation{{0, 0, 0}};
    std::array<double, 4> rotationXYZW{{0, 0, 0, 1}};
};

// The archived solid remains immutable. Ordinary edits own only this affine
// source-local-to-carrier placement, so replay never synthesizes dimensions.
struct EffectiveFrozenSolidGeometry final {
    std::array<double, 16> sourceToCarrier{{
        1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
    }};
};

struct OriginalInput final {
    RecipeInputRole role = RecipeInputRole::Left;
    UUID sourceNode{};
    std::uint64_t sourceLocalID = 0;
    std::uint32_t shapeSlot = 0;
    retained_recipe::SourceIdentity original;
    RecipeInputFamily recipeKind = RecipeInputFamily::Profile;
    std::uint32_t recipeSchema = 0;
    std::vector<std::uint8_t> recipeBytes;
    double sourceMetersPerUnit = 0;
    double carrierMetersPerUnit = 0;
    std::array<double, 16> sourceRootToCarrier{{
        1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
    }};
    OriginalAnalyticProjection projection;
    SignatureCommitments commitments;
    CompleteMetadata metadata;
};

struct OriginalStateBlock final {
    std::uint16_t schema = 1;
    Operation creationOperation = Operation::Union;
    std::array<OriginalInput, 2> inputs;
    std::vector<ScalarMaterialRow> materials;
    std::vector<GroupRow> groups;
    std::vector<LayerRow> layers;
};

struct EffectiveInput final {
    RecipeInputRole role = RecipeInputRole::Left;
    InputFamily family = InputFamily::AnalyticRectangularPrism;
    UUID sourceNode{};
    UUID originalSourceFeature{};
    SignatureCommitments commitments;
    EffectiveAnalyticGeometry geometry;
    EffectiveFrozenSolidGeometry frozenGeometry;
    double sourceMetersPerUnit = 0;
    CompleteMetadata metadata;
};

struct EffectiveStateBlock final {
    std::uint16_t metadataSchema = 1;
    std::array<EffectiveInput, 2> inputs;
    std::vector<ScalarMaterialRow> materials;
    std::vector<GroupRow> groups;
    std::vector<LayerRow> layers;
};

struct OriginalsDefinition final {
    Operation operation = Operation::Union;
    MaterialPolicy materialPolicy = MaterialPolicy::RejectConflictingMerges;
    CodecVersions versions;
    retained_recipe::OwnerKey owner;
    UUID booleanNode{};
    UUID feature{};
    double carrierMetersPerUnit = 0;
    OriginalStateBlock originals;
    EffectiveStateBlock effective;
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

inline bool ValidRecipePlacement(const RecipeInput& value) noexcept {
    for (double scalar : value.inputToCarrier)
        if (!std::isfinite(scalar)) return false;
    if (!std::isfinite(value.sourceMetersPerUnit) || value.sourceMetersPerUnit <= 0
        || !std::isfinite(value.carrierMetersPerUnit) || value.carrierMetersPerUnit <= 0
        || value.inputToCarrier[12] != 0 || value.inputToCarrier[13] != 0
        || value.inputToCarrier[14] != 0 || value.inputToCarrier[15] != 1) return false;
    const auto dot = [&](unsigned a, unsigned b) {
        return value.inputToCarrier[a] * value.inputToCarrier[b]
            + value.inputToCarrier[4 + a] * value.inputToCarrier[4 + b]
            + value.inputToCarrier[8 + a] * value.inputToCarrier[8 + b];
    };
    const double scale2 = dot(0, 0);
    if (!std::isfinite(scale2) || scale2 <= 0) return false;
    const double tolerance = std::max(1.0, scale2) * 1e-12;
    if (std::abs(dot(1, 1) - scale2) > tolerance
        || std::abs(dot(2, 2) - scale2) > tolerance
        || std::abs(dot(0, 1)) > tolerance
        || std::abs(dot(0, 2)) > tolerance
        || std::abs(dot(1, 2)) > tolerance) return false;
    const double determinant =
        value.inputToCarrier[0] * (value.inputToCarrier[5] * value.inputToCarrier[10]
            - value.inputToCarrier[6] * value.inputToCarrier[9])
      - value.inputToCarrier[1] * (value.inputToCarrier[4] * value.inputToCarrier[10]
            - value.inputToCarrier[6] * value.inputToCarrier[8])
      + value.inputToCarrier[2] * (value.inputToCarrier[4] * value.inputToCarrier[9]
            - value.inputToCarrier[5] * value.inputToCarrier[8]);
    return std::isfinite(determinant) && determinant > 0;
}

inline bool Valid(const RecipeInput& value) noexcept {
    if ((value.role != RecipeInputRole::Left && value.role != RecipeInputRole::Right)
        || (value.family != RecipeInputFamily::Profile
            && value.family != RecipeInputFamily::Enclosure
            && value.family != RecipeInputFamily::RectangularLoft)
        || !retained_recipe::Nonzero(value.sourceNode) || value.sourceLocalID == 0
        || !retained_recipe::Valid(value.original) || value.recipeSchema == 0
        || !ValidRecipePlacement(value) || !Valid(value.commitments)
        || !Valid(value.originalMaterial) || !ValidText(value.originalName)
        || value.originalGroups.size() > MaximumGroupsPerInput
        || value.recipeScalars.empty()) return false;
    std::set<std::string> groups;
    for (const auto& group : value.originalGroups)
        if (group.empty() || !ValidText(group) || !groups.insert(group).second) return false;
    for (double scalar : value.recipeScalars)
        if (!std::isfinite(scalar)) return false;
    return true;
}

inline bool Valid(const RecipeDefinition& value) noexcept {
    return !value.nativeAdmissionEnabled && retained_recipe::Valid(value.owner)
        && retained_recipe::Nonzero(value.booleanNode)
        && retained_recipe::Nonzero(value.feature)
        && value.materialPolicy == MaterialPolicy::RejectConflictingMerges
        && (value.operation == Operation::Union || value.operation == Operation::Subtract
            || value.operation == Operation::Intersect)
        && value.versions.serializer == 1 && value.versions.build == 1
        && value.versions.proof == 1 && value.versions.selector == 1
        && value.versions.material == 1 && Valid(value.inputs[0]) && Valid(value.inputs[1])
        && value.inputs[0].role == RecipeInputRole::Left
        && value.inputs[1].role == RecipeInputRole::Right
        && value.inputs[0].sourceNode != value.inputs[1].sourceNode
        && value.inputs[0].sourceLocalID != value.inputs[1].sourceLocalID
        && !(value.inputs[0].original == value.inputs[1].original)
        && value.inputs[0].shapeSlot != value.inputs[1].shapeSlot
        && value.inputs[0].shapeSlot <= 1 && value.inputs[1].shapeSlot <= 1;
}

inline bool Valid(const UTF16Name& value) noexcept {
    if (!value.attributePresent) return value.codeUnits.empty();
    if (value.codeUnits.size() > MaximumUTF16CodeUnits) return false;
    for (std::size_t index = 0; index < value.codeUnits.size(); ++index) {
        const std::uint16_t unit = value.codeUnits[index];
        if (unit == 0 || unit < 0x20 || unit == 0x7f) return false;
        if (unit >= 0xd800 && unit <= 0xdbff) {
            if (++index >= value.codeUnits.size()
                || value.codeUnits[index] < 0xdc00
                || value.codeUnits[index] > 0xdfff) return false;
        } else if (unit >= 0xdc00 && unit <= 0xdfff) return false;
    }
    return true;
}

inline bool SameDoubleBits(double left, double right) noexcept {
    return std::memcmp(&left, &right, sizeof(double)) == 0;
}

inline bool ValidFiniteScalars(const double* values, std::size_t count) noexcept {
    for (std::size_t index = 0; index < count; ++index)
        if (!std::isfinite(values[index])) return false;
    return true;
}

inline bool Valid(const OccurrenceState& value) noexcept {
    if (value.encoding != 1 || !ValidFiniteScalars(value.values.data(), value.values.size())
        || !ValidFiniteScalars(value.derivedMatrix.data(), value.derivedMatrix.size())) return false;
    for (std::size_t index = 0; index < value.values.size(); ++index)
        if (!(value.presenceMask & (1U << index)) && !SameDoubleBits(value.values[index], 0.0))
            return false;
    const double tx = value.presenceMask & 1 ? value.values[0] : 0;
    const double ty = value.presenceMask & 2 ? value.values[1] : 0;
    const double tz = value.presenceMask & 4 ? value.values[2] : 0;
    const double qx = value.presenceMask & 8 ? value.values[3] : 0;
    const double qy = value.presenceMask & 16 ? value.values[4] : 0;
    const double qz = value.presenceMask & 32 ? value.values[5] : 0;
    const double qw = value.presenceMask & 64 ? value.values[6] : 1;
    const double scale = value.presenceMask & 128 ? value.values[7] : 1;
    const double norm = qx*qx + qy*qy + qz*qz + qw*qw;
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1e-12
        || !std::isfinite(scale) || scale == 0) return false;
    const std::array<double, 16> expected{{
        scale * (1 - 2*(qy*qy + qz*qz)), scale * 2*(qx*qy - qz*qw),
        scale * 2*(qx*qz + qy*qw), tx,
        scale * 2*(qx*qy + qz*qw), scale * (1 - 2*(qx*qx + qz*qz)),
        scale * 2*(qy*qz - qx*qw), ty,
        scale * 2*(qx*qz - qy*qw), scale * 2*(qy*qz + qx*qw),
        scale * (1 - 2*(qx*qx + qy*qy)), tz,
        0, 0, 0, 1,
    }};
    for (std::size_t index = 0; index < expected.size(); ++index)
        if (std::abs(value.derivedMatrix[index] - expected[index]) > 1e-12) return false;
    return true;
}

inline bool ValidArchiveText(const std::string& value, bool allowEmpty = false) noexcept {
    if ((!allowEmpty && value.empty()) || value.size() > MaximumTextBytes) return false;
    for (unsigned char byte : value)
        if (byte == 0 || byte < 0x20 || byte == 0x7f || byte >= 0x80) return false;
    return true;
}

inline bool Valid(const ScalarMaterialRow& value, std::uint16_t ordinal) noexcept {
    return value.ordinal == ordinal && ValidArchiveText(value.labelEntry)
        && std::isfinite(value.alphaCutoff)
        && ValidFiniteScalars(value.pbr.data(), value.pbr.size())
        && ValidFiniteScalars(value.common.data(), value.common.size());
}

inline bool Valid(const GroupRow& value, std::uint16_t ordinal) noexcept {
    if (value.ordinal != ordinal || !ValidArchiveText(value.identifier)
        || !Valid(value.name) || value.members.empty()
        || value.members.size() > MaximumMetadataRows
        || !ValidFiniteScalars(value.origin.data(), value.origin.size())) return false;
    if (!value.originPresent)
        for (double coordinate : value.origin) if (!SameDoubleBits(coordinate, 0.0)) return false;
    for (std::size_t index = 0; index < value.members.size(); ++index) {
        if (!retained_recipe::Valid(value.members[index])) return false;
        for (std::size_t prior = 0; prior < index; ++prior)
            if (value.members[index] == value.members[prior]) return false;
    }
    return true;
}

inline bool Valid(const LayerRow& value, std::uint16_t ordinal) noexcept {
    return value.ordinal == ordinal && ValidArchiveText(value.labelEntry)
        && Valid(value.name);
}

inline bool ValidReferences(const std::vector<std::uint16_t>& references,
                            std::size_t count) noexcept {
    if (references.size() > MaximumMetadataRows) return false;
    std::set<std::uint16_t> seen;
    for (std::uint16_t reference : references)
        if (reference >= count || !seen.insert(reference).second) return false;
    return true;
}

inline bool Valid(const CompleteMetadata& value, std::size_t materials,
                  std::size_t groups, std::size_t layers) noexcept {
    if (!Valid(value.name) || !Valid(value.occurrence)
        || (value.appearance.material && *value.appearance.material >= materials)
        || (!value.appearance.legacyMaterialPresent && value.appearance.legacyMaterial != 0)
        || (!value.appearance.legacyColorPresent && value.appearance.legacyColor != 0)
        || !ValidReferences(value.groups, groups)
        || !ValidReferences(value.layers, layers)) return false;
    return value.layerLinkPresent || value.layers.empty();
}

inline bool ValidTables(const std::vector<ScalarMaterialRow>& materials,
                        const std::vector<GroupRow>& groups,
                        const std::vector<LayerRow>& layers) noexcept {
    if (materials.size() > MaximumMetadataRows || groups.size() > MaximumMetadataRows
        || layers.size() > MaximumMetadataRows) return false;
    std::set<std::string> materialProvenance, groupIDs, layerProvenance;
    for (std::size_t index = 0; index < materials.size(); ++index)
        if (!Valid(materials[index], std::uint16_t(index))
            || !materialProvenance.insert(materials[index].labelEntry).second) return false;
    for (std::size_t index = 0; index < groups.size(); ++index)
        if (!Valid(groups[index], std::uint16_t(index))
            || !groupIDs.insert(groups[index].identifier).second) return false;
    for (std::size_t index = 0; index < layers.size(); ++index)
        if (!Valid(layers[index], std::uint16_t(index))
            || !layerProvenance.insert(layers[index].labelEntry).second) return false;
    return true;
}

inline bool ValidProjection(const OriginalAnalyticProjection& value) noexcept {
    if (!ValidFiniteScalars(value.dimensions.data(), value.dimensions.size())
        || !ValidFiniteScalars(value.translation.data(), value.translation.size())
        || !ValidFiniteScalars(value.rotationXYZW.data(), value.rotationXYZW.size())) return false;
    for (double dimension : value.dimensions) if (dimension <= 0) return false;
    double norm = 0; for (double q : value.rotationXYZW) norm += q*q;
    return std::abs(norm - 1.0) <= 1e-12;
}

inline bool ValidGeometry(const EffectiveAnalyticGeometry& value) noexcept {
    OriginalAnalyticProjection projection;
    projection.dimensions = value.dimensions;
    projection.translation = value.translation;
    projection.rotationXYZW = value.rotationXYZW;
    return ValidProjection(projection);
}

inline bool ValidGeometry(const EffectiveFrozenSolidGeometry& value) noexcept {
    if (!ValidFiniteScalars(value.sourceToCarrier.data(), value.sourceToCarrier.size())
        || value.sourceToCarrier[12] != 0 || value.sourceToCarrier[13] != 0
        || value.sourceToCarrier[14] != 0 || value.sourceToCarrier[15] != 1)
        return false;
    const auto& m = value.sourceToCarrier;
    const double determinant =
        m[0] * (m[5] * m[10] - m[6] * m[9])
      - m[1] * (m[4] * m[10] - m[6] * m[8])
      + m[2] * (m[4] * m[9] - m[5] * m[8]);
    return std::isfinite(determinant) && determinant > 0;
}

inline bool Valid(const OriginalsDefinition& value) noexcept {
    try {
        if (!retained_recipe::Valid(value.owner)
            || !retained_recipe::Nonzero(value.booleanNode)
            || !retained_recipe::Nonzero(value.feature)
            || !std::isfinite(value.carrierMetersPerUnit) || value.carrierMetersPerUnit <= 0
            || value.materialPolicy != MaterialPolicy::RejectConflictingMerges
            || (value.operation != Operation::Union && value.operation != Operation::Subtract
                && value.operation != Operation::Intersect)
            || value.versions.serializer != 1 || value.versions.build != 1
            || value.versions.proof != 1 || value.versions.selector != 1
            || value.versions.material != 1 || value.originals.schema != 1
            || value.effective.metadataSchema != 1
            || (value.originals.creationOperation != Operation::Union
                && value.originals.creationOperation != Operation::Subtract
                && value.originals.creationOperation != Operation::Intersect)
            || !ValidTables(value.originals.materials, value.originals.groups, value.originals.layers)
            || !ValidTables(value.effective.materials, value.effective.groups, value.effective.layers))
            return false;
        std::set<UUID> nodes, features;
        std::set<std::uint64_t> localIDs;
        std::set<std::uint32_t> slots;
        for (std::size_t index = 0; index < 2; ++index) {
            const OriginalInput& original = value.originals.inputs[index];
            const EffectiveInput& effective = value.effective.inputs[index];
            const RecipeInputRole role = index == 0 ? RecipeInputRole::Left : RecipeInputRole::Right;
            const bool analytic = effective.family
                == InputFamily::AnalyticRectangularPrism;
            const bool frozen = effective.family == InputFamily::FrozenSolid;
            if (original.role != role || effective.role != role
                || (!analytic && !frozen)
                || !retained_recipe::Nonzero(original.sourceNode)
                || original.sourceNode != effective.sourceNode
                || !nodes.insert(original.sourceNode).second
                || original.sourceLocalID == 0 || !localIDs.insert(original.sourceLocalID).second
                || original.shapeSlot > 1 || !slots.insert(original.shapeSlot).second
                || !retained_recipe::Valid(original.original)
                || original.original.document != value.owner.document
                || !features.insert(original.original.sourceFeature).second
                || effective.originalSourceFeature != original.original.sourceFeature
                || (analytic && original.recipeKind != RecipeInputFamily::Profile
                    && original.recipeKind != RecipeInputFamily::Enclosure
                    && original.recipeKind != RecipeInputFamily::RectangularLoft)
                || (frozen && original.recipeKind
                    != RecipeInputFamily::OrdinarySource)
                || original.recipeSchema == 0 || original.recipeBytes.empty()
                || original.recipeBytes.size() > MaximumRecipeBytes
                || (frozen && original.recipeSchema != 1)
                || !std::isfinite(original.sourceMetersPerUnit)
                || original.sourceMetersPerUnit <= 0
                || !SameDoubleBits(original.carrierMetersPerUnit, value.carrierMetersPerUnit)
                || !ValidFiniteScalars(original.sourceRootToCarrier.data(), 16)
                || (analytic && !ValidProjection(original.projection))
                || !Valid(original.commitments)
                || (analytic && !ValidGeometry(effective.geometry))
                || (frozen && !ValidGeometry(effective.frozenGeometry))
                || !Valid(effective.commitments)
                || !std::isfinite(effective.sourceMetersPerUnit)
                || effective.sourceMetersPerUnit <= 0
                || !Valid(original.metadata, value.originals.materials.size(),
                    value.originals.groups.size(), value.originals.layers.size())
                || !Valid(effective.metadata, value.effective.materials.size(),
                    value.effective.groups.size(), value.effective.layers.size())) return false;
        }
        return slots.count(0) == 1 && slots.count(1) == 1;
    } catch (...) { return false; }
}
} // namespace core3d::part_boolean
