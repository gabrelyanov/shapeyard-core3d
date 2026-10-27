#pragma once
// Persistence-only SYPB/5 record. It owns no document/shape authority.
#include "PartBooleanPersistence.hxx"
#include "CompositeRecipeDefinition.hxx"
#include "RetainedBooleanProgram.hxx"

namespace core3d::part_boolean::program_operand {
inline constexpr std::uint8_t PayloadVersion = 5;
enum class Refusal : std::uint8_t;
enum class CutSurfacePolicy : std::uint8_t { TargetDefault = 1, ToolRegion = 2 };

struct DependencyDefinition final {
    retained_recipe::RecipeLocator locator;
    retained_recipe::Digest geometry{}, recipe{}, placement{}, material{}, groups{};
};
struct FenceDefinition final {
    std::uint64_t documentGeneration = 0, modelRevision = 0;
    double effectiveMetersPerUnit = 0;
    retained_recipe::Digest ownerShape{}, ownerRecipe{}, ownerPlacement{}, ownerMaterial{};
    std::vector<DependencyDefinition> dependencies;
    retained_recipe::Digest base{}, result{}, units{}, selection{}, issuance{}, history{};
};
struct AppearanceDefinition final {
    part_boolean::CompleteMetadata original, effective;
    std::vector<part_boolean::ScalarMaterialRow> originalMaterials, effectiveMaterials;
    std::vector<part_boolean::GroupRow> originalGroups, effectiveGroups;
    std::vector<part_boolean::LayerRow> originalLayers, effectiveLayers;
    std::vector<part_boolean::RegionBinding> regions;
    std::vector<part_boolean::SelectorBinding> selectors;
    part_boolean::MaterialPolicy materialPolicy =
        part_boolean::MaterialPolicy::RejectConflictingMerges;
    part_boolean::CodecVersions versions;
    bool originallyVisible = true, originallyConsumed = false;
};
struct InputDefinition final {
    part_boolean::RecipeInputRole role = part_boolean::RecipeInputRole::Left;
    part_boolean::RecipeInputFamily baseFamily = part_boolean::RecipeInputFamily::Profile;
    retained_recipe::UUID sourceNode{};
    std::uint64_t sourceLocalID = 0;
    std::uint32_t shapeSlot = 0;
    retained_recipe::SourceIdentity original;
    retained_recipe::UUID derivedFeature{}, outputFeature{};
    std::uint64_t nextLocalID = 0;
    std::uint32_t originalSchema = 0;
    std::uint8_t originalMinor = 0;
    std::vector<std::uint8_t> originalBytes;
    retained_recipe::Digest originalDigest{}, effectiveDigest{};
    retained_recipe::Digest originalBase{}, originalResult{}, effectiveBase{}, effectiveResult{};
    composite_recipe::InputPlacement placement;
    double ownerMetersPerUnit = 0, documentMetersPerUnit = 0;
    part_boolean::SignatureCommitments originalCommitments, effectiveCommitments;
    AppearanceDefinition appearance;
    FenceDefinition fence;
};
struct Definition final {
    part_boolean::Operation operation = part_boolean::Operation::Union;
    CutSurfacePolicy cutSurfacePolicy = CutSurfacePolicy::TargetDefault;
    part_boolean::MaterialPolicy materialPolicy =
        part_boolean::MaterialPolicy::RejectConflictingMerges;
    part_boolean::CodecVersions versions;
    retained_recipe::OwnerKey owner;
    retained_recipe::UUID booleanNode{}, feature{};
    part_boolean::SignatureCommitments ownerOutputCommitments;
    std::array<InputDefinition, 2> inputs;
};

namespace codec_detail {
inline void Owner(part_boolean::Writer& w, const retained_recipe::OwnerKey& v) {
    w.raw(v.document); w.raw(v.entity); w.raw(v.definition);
}
inline bool Owner(part_boolean::Reader& r, retained_recipe::OwnerKey& v) {
    return r.raw(v.document) && r.raw(v.entity) && r.raw(v.definition);
}
inline void Identity(part_boolean::Writer& w, const retained_recipe::SourceIdentity& v) {
    w.raw(v.document); w.raw(v.entity); w.raw(v.definition); w.raw(v.sourceFeature);
}
inline bool Identity(part_boolean::Reader& r, retained_recipe::SourceIdentity& v) {
    return r.raw(v.document) && r.raw(v.entity) && r.raw(v.definition)
        && r.raw(v.sourceFeature);
}
inline void Placement(part_boolean::Writer& w, const composite_recipe::InputPlacement& v) {
    for (double x : v.matrix) w.scalar(x);
    w.scalar(v.sourceMetersPerUnit); w.scalar(v.carrierMetersPerUnit);
}
inline bool Placement(part_boolean::Reader& r, composite_recipe::InputPlacement& v) {
    for (double& x : v.matrix) if (!r.scalar(x)) return false;
    return r.scalar(v.sourceMetersPerUnit) && r.scalar(v.carrierMetersPerUnit);
}
inline void Dependency(part_boolean::Writer& w, const DependencyDefinition& v) {
    Owner(w, v.locator.owner); w.raw(v.locator.node); w.raw(v.locator.sourceFeature);
    w.raw(v.geometry); w.raw(v.recipe); w.raw(v.placement); w.raw(v.material); w.raw(v.groups);
}
inline bool Dependency(part_boolean::Reader& r, DependencyDefinition& v) {
    return Owner(r, v.locator.owner) && r.raw(v.locator.node)
        && r.raw(v.locator.sourceFeature) && r.raw(v.geometry) && r.raw(v.recipe)
        && r.raw(v.placement) && r.raw(v.material) && r.raw(v.groups);
}
inline void Fence(part_boolean::Writer& w, const FenceDefinition& v) {
    w.integer(v.documentGeneration, 8); w.integer(v.modelRevision, 8);
    w.scalar(v.effectiveMetersPerUnit); w.raw(v.ownerShape); w.raw(v.ownerRecipe);
    w.raw(v.ownerPlacement); w.raw(v.ownerMaterial); w.integer(v.dependencies.size(), 2);
    for (const auto& d : v.dependencies) Dependency(w, d);
    w.raw(v.base); w.raw(v.result); w.raw(v.units); w.raw(v.selection);
    w.raw(v.issuance); w.raw(v.history);
}
inline bool Fence(part_boolean::Reader& r, FenceDefinition& v) {
    std::uint64_t count = 0;
    if (!r.integer(8, v.documentGeneration) || !r.integer(8, v.modelRevision)
        || !r.scalar(v.effectiveMetersPerUnit) || !r.raw(v.ownerShape)
        || !r.raw(v.ownerRecipe) || !r.raw(v.ownerPlacement) || !r.raw(v.ownerMaterial)
        || !r.integer(2, count) || count == 0
        || count > composite_recipe::MaximumSourceNodes) return false;
    v.dependencies.resize(std::size_t(count));
    for (auto& d : v.dependencies) if (!Dependency(r, d)) return false;
    return r.raw(v.base) && r.raw(v.result) && r.raw(v.units) && r.raw(v.selection)
        && r.raw(v.issuance) && r.raw(v.history);
}
inline void Regions(part_boolean::Writer& w,
                    const std::vector<part_boolean::RegionBinding>& values) {
    w.integer(values.size(), 2);
    for (const auto& v : values) { w.raw(v.region); w.integer(v.sourceInput, 1);
        w.integer(std::uint8_t(v.kind), 1); w.integer(v.materialIndex, 2); }
}
inline bool Regions(part_boolean::Reader& r,
                    std::vector<part_boolean::RegionBinding>& values) {
    std::uint64_t count = 0;
    if (!r.integer(2, count) || count > part_boolean::MaximumRegions) return false;
    values.resize(std::size_t(count));
    for (auto& v : values) { std::uint64_t source = 0, kind = 0, material = 0;
        if (!r.raw(v.region) || !r.integer(1, source) || !r.integer(1, kind)
            || !r.integer(2, material)) return false;
        v.sourceInput = std::uint8_t(source); v.kind = part_boolean::RegionKind(kind);
        v.materialIndex = std::uint16_t(material); }
    return true;
}
inline void Selectors(part_boolean::Writer& w,
                      const std::vector<part_boolean::SelectorBinding>& values) {
    w.integer(values.size(), 2);
    for (const auto& v : values) { w.raw(v.selector); w.raw(v.region);
        w.integer(std::uint8_t(v.policy), 1); w.integer(0, 1);
        w.integer(v.expectedCardinality, 2); }
}
inline bool Selectors(part_boolean::Reader& r,
                      std::vector<part_boolean::SelectorBinding>& values) {
    std::uint64_t count = 0;
    if (!r.integer(2, count) || count > part_boolean::MaximumSelectors) return false;
    values.resize(std::size_t(count));
    for (auto& v : values) { std::uint64_t policy = 0, reserved = 0, cardinality = 0;
        if (!r.raw(v.selector) || !r.raw(v.region) || !r.integer(1, policy)
            || !r.integer(1, reserved) || reserved || !r.integer(2, cardinality)) return false;
        v.policy = part_boolean::SelectorPolicy(policy);
        v.expectedCardinality = std::uint16_t(cardinality); }
    return true;
}
inline void Appearance(part_boolean::Writer& w, const AppearanceDefinition& v) {
    std::vector<std::uint8_t> bytes;
    if (!part_boolean::WriteMetadata(v.original, bytes)) w.valid = false;
    else part_boolean::WriteSized(w, bytes);
    if (!part_boolean::WriteMetadata(v.effective, bytes)) w.valid = false;
    else part_boolean::WriteSized(w, bytes);
    part_boolean::WriteTables(w, v.originalMaterials, v.originalGroups, v.originalLayers);
    part_boolean::WriteTables(w, v.effectiveMaterials, v.effectiveGroups, v.effectiveLayers);
    Regions(w, v.regions); Selectors(w, v.selectors);
    w.integer(std::uint8_t(v.materialPolicy), 1);
    w.integer(v.originallyVisible ? 1 : 0, 1);
    w.integer(v.originallyConsumed ? 1 : 0, 1); w.integer(0, 1);
    for (std::uint32_t x : {v.versions.serializer, v.versions.build, v.versions.proof,
         v.versions.selector, v.versions.material}) w.integer(x, 4);
}
inline bool Appearance(part_boolean::Reader& r, AppearanceDefinition& v) {
    std::vector<std::uint8_t> bytes;
    std::uint64_t policy = 0, visible = 0, consumed = 0, reserved = 0;
    if (!part_boolean::ReadSized(r, bytes) || !part_boolean::ReadMetadata(bytes, v.original)
        || !part_boolean::ReadSized(r, bytes) || !part_boolean::ReadMetadata(bytes, v.effective)
        || !part_boolean::ReadTables(r, v.originalMaterials, v.originalGroups, v.originalLayers)
        || !part_boolean::ReadTables(r, v.effectiveMaterials, v.effectiveGroups, v.effectiveLayers)
        || !Regions(r, v.regions) || !Selectors(r, v.selectors)
        || !r.integer(1, policy) || !r.integer(1, visible) || !r.integer(1, consumed)
        || !r.integer(1, reserved) || visible > 1 || consumed > 1 || reserved) return false;
    v.materialPolicy = part_boolean::MaterialPolicy(policy);
    v.originallyVisible = visible != 0; v.originallyConsumed = consumed != 0;
    std::uint32_t* versions[] = {&v.versions.serializer, &v.versions.build,
        &v.versions.proof, &v.versions.selector, &v.versions.material};
    for (auto* version : versions) { std::uint64_t x = 0;
        if (!r.integer(4, x) || x > UINT32_MAX) return false; *version = std::uint32_t(x); }
    return true;
}
inline void Input(part_boolean::Writer& w, const InputDefinition& v) {
    w.integer(std::uint8_t(v.role), 1); w.integer(std::uint8_t(v.baseFamily), 1);
    w.integer(v.originalMinor, 1); w.integer(0, 1); w.raw(v.sourceNode);
    w.integer(v.sourceLocalID, 8); w.integer(v.shapeSlot, 4); Identity(w, v.original);
    w.raw(v.derivedFeature); w.raw(v.outputFeature); w.integer(v.nextLocalID, 8);
    w.integer(v.originalSchema, 4); part_boolean::WriteSized(w, v.originalBytes);
    w.raw(v.originalDigest); w.raw(v.effectiveDigest); w.raw(v.originalBase);
    w.raw(v.originalResult); w.raw(v.effectiveBase); w.raw(v.effectiveResult);
    Placement(w, v.placement); w.scalar(v.ownerMetersPerUnit);
    w.scalar(v.documentMetersPerUnit); part_boolean::WriteCommitments(w, v.originalCommitments);
    part_boolean::WriteCommitments(w, v.effectiveCommitments);
    Appearance(w, v.appearance); Fence(w, v.fence);
}
inline bool Input(part_boolean::Reader& r, InputDefinition& v) {
    std::uint64_t role = 0, family = 0, minor = 0, reserved = 0, slot = 0, schema = 0;
    if (!r.integer(1, role) || !r.integer(1, family) || !r.integer(1, minor)
        || !r.integer(1, reserved) || reserved || !r.raw(v.sourceNode)
        || !r.integer(8, v.sourceLocalID) || !r.integer(4, slot) || slot > UINT32_MAX
        || !Identity(r, v.original) || !r.raw(v.derivedFeature) || !r.raw(v.outputFeature)
        || !r.integer(8, v.nextLocalID) || !r.integer(4, schema) || schema > UINT32_MAX
        || !part_boolean::ReadSized(r, v.originalBytes, part_boolean::MaximumRecipeBytes)
        || !r.raw(v.originalDigest) || !r.raw(v.effectiveDigest)
        || !r.raw(v.originalBase) || !r.raw(v.originalResult)
        || !r.raw(v.effectiveBase) || !r.raw(v.effectiveResult)
        || !Placement(r, v.placement) || !r.scalar(v.ownerMetersPerUnit)
        || !r.scalar(v.documentMetersPerUnit)
        || !part_boolean::ReadCommitments(r, v.originalCommitments)
        || !part_boolean::ReadCommitments(r, v.effectiveCommitments)
        || !Appearance(r, v.appearance) || !Fence(r, v.fence)) return false;
    v.role = part_boolean::RecipeInputRole(role);
    v.baseFamily = part_boolean::RecipeInputFamily(family);
    v.originalMinor = std::uint8_t(minor); v.shapeSlot = std::uint32_t(slot);
    v.originalSchema = std::uint32_t(schema); return true;
}
inline bool SameDouble(double a, double b) noexcept {
    return std::memcmp(&a, &b, sizeof(double)) == 0;
}
inline bool SamePlacement(const composite_recipe::InputPlacement& a,
                          const composite_recipe::InputPlacement& b) noexcept {
    for (std::size_t i = 0; i < a.matrix.size(); ++i)
        if (!SameDouble(a.matrix[i], b.matrix[i])) return false;
    return SameDouble(a.sourceMetersPerUnit, b.sourceMetersPerUnit)
        && SameDouble(a.carrierMetersPerUnit, b.carrierMetersPerUnit);
}
} // namespace codec_detail

inline bool Valid(const Definition& value) noexcept {
    try {
        if ((value.operation != part_boolean::Operation::Union
                && value.operation != part_boolean::Operation::Subtract
                && value.operation != part_boolean::Operation::Intersect)
            || (value.cutSurfacePolicy != CutSurfacePolicy::TargetDefault
                && value.cutSurfacePolicy != CutSurfacePolicy::ToolRegion)
            || value.materialPolicy != part_boolean::MaterialPolicy::RejectConflictingMerges
            || value.versions.serializer != 1 || value.versions.build != 1
            || value.versions.proof != 1 || value.versions.selector != 1
            || value.versions.material != 1 || !retained_recipe::Valid(value.owner)
            || !retained_recipe::Nonzero(value.booleanNode)
            || !retained_recipe::Nonzero(value.feature)
            || !part_boolean::Valid(value.ownerOutputCommitments)) return false;
        std::set<retained_recipe::UUID> nodes;
        std::set<std::uint64_t> locals;
        std::set<std::uint32_t> slots;
        for (std::size_t index = 0; index < 2; ++index) {
            const auto& input = value.inputs[index];
            retained_boolean::Program program; retained_recipe::Digest digest{};
            if (input.role != (index == 0 ? part_boolean::RecipeInputRole::Left
                                           : part_boolean::RecipeInputRole::Right)
                || (input.baseFamily != part_boolean::RecipeInputFamily::Profile
                    && input.baseFamily != part_boolean::RecipeInputFamily::Enclosure
                    && input.baseFamily != part_boolean::RecipeInputFamily::RectangularLoft)
                || !retained_recipe::Nonzero(input.sourceNode)
                || !nodes.insert(input.sourceNode).second || input.sourceLocalID == 0
                || !locals.insert(input.sourceLocalID).second || input.shapeSlot > 1
                || !slots.insert(input.shapeSlot).second || !retained_recipe::Valid(input.original)
                || input.original.document != value.owner.document
                || !retained_recipe::Nonzero(input.derivedFeature)
                || !retained_recipe::Nonzero(input.outputFeature)
                || input.nextLocalID <= input.sourceLocalID || input.originalBytes.empty()
                || input.originalBytes.size() > part_boolean::MaximumRecipeBytes
                || !retained_boolean::Decode(input.originalBytes, program)
                || program.source.schema != input.originalSchema
                || program.codecMinor != input.originalMinor
                || program.source.document != input.original.document
                || program.source.entity != input.original.entity
                || program.source.definition != input.original.definition
                || program.source.sourceFeature != input.original.sourceFeature
                || program.source.derivedFeature != input.derivedFeature
                || !retained_solid::Hash(input.originalBytes, digest)
                || digest != input.originalDigest
                || input.originalCommitments.recipe != input.originalDigest
                || input.effectiveCommitments.recipe != input.effectiveDigest
                || !composite_recipe::ValidPlacement(input.placement)
                || !codec_detail::SameDouble(input.ownerMetersPerUnit,
                                              input.placement.carrierMetersPerUnit)
                || !codec_detail::SameDouble(input.documentMetersPerUnit,
                                              input.placement.carrierMetersPerUnit)
                || !part_boolean::Valid(input.originalCommitments)
                || !part_boolean::Valid(input.effectiveCommitments)
                || input.appearance.materialPolicy
                    != part_boolean::MaterialPolicy::RejectConflictingMerges
                || !part_boolean::ValidTables(input.appearance.originalMaterials,
                    input.appearance.originalGroups, input.appearance.originalLayers)
                || !part_boolean::ValidTables(input.appearance.effectiveMaterials,
                    input.appearance.effectiveGroups, input.appearance.effectiveLayers)
                || !part_boolean::Valid(input.appearance.original,
                    input.appearance.originalMaterials.size(), input.appearance.originalGroups.size(),
                    input.appearance.originalLayers.size())
                || !part_boolean::Valid(input.appearance.effective,
                    input.appearance.effectiveMaterials.size(), input.appearance.effectiveGroups.size(),
                    input.appearance.effectiveLayers.size())
                || input.appearance.regions.empty() || input.appearance.selectors.empty()
                || input.fence.documentGeneration == 0 || input.fence.modelRevision == 0
                || input.fence.dependencies.size() != 2
                || input.fence.base != input.effectiveBase
                || input.fence.result != input.effectiveResult) return false;
        }
        return slots.count(0) == 1 && slots.count(1) == 1;
    } catch (...) { return false; }
}

inline bool EncodeProgramOperands(const Definition& value,
                                  std::vector<std::uint8_t>& output,
                                  Refusal* refusal = nullptr) noexcept {
    output.clear(); if (refusal) *refusal = static_cast<Refusal>(2);
    try {
        if (!Valid(value)) return false;
        part_boolean::Writer writer;
        writer.raw(reinterpret_cast<const std::uint8_t*>("SYPB"), 4);
        writer.integer(PayloadVersion, 1); writer.integer(0, 3);
        writer.integer(std::uint8_t(value.operation), 1);
        writer.integer(std::uint8_t(value.cutSurfacePolicy), 1);
        writer.integer(std::uint8_t(value.materialPolicy), 1); writer.integer(0, 1);
        for (std::uint32_t x : {value.versions.serializer, value.versions.build,
             value.versions.proof, value.versions.selector, value.versions.material})
            writer.integer(x, 4);
        codec_detail::Owner(writer, value.owner); writer.raw(value.booleanNode);
        writer.raw(value.feature); part_boolean::WriteCommitments(writer,
            value.ownerOutputCommitments); writer.integer(2, 1); writer.integer(0, 3);
        for (const auto& input : value.inputs) codec_detail::Input(writer, input);
        if (!writer.valid || writer.bytes.size() > part_boolean::MaximumPayloadBytes - 32) {
            if (refusal) *refusal = static_cast<Refusal>(3); return false;
        }
        retained_recipe::Digest digest{};
        if (!retained_solid::Hash(writer.bytes, digest)) return false;
        writer.raw(digest); if (!writer.valid) return false;
        output = std::move(writer.bytes); if (refusal) *refusal = static_cast<Refusal>(0);
        return true;
    } catch (...) { output.clear(); return false; }
}

inline bool DecodeProgramOperands(const std::vector<std::uint8_t>& bytes,
                                  Definition& output) noexcept {
    output = Definition{};
    try {
        if (bytes.size() < 8 + 4 + 20 + 48 + 32 + 32
            || bytes.size() > part_boolean::MaximumPayloadBytes
            || std::memcmp(bytes.data(), "SYPB\5\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        retained_recipe::Digest expected{}, actual{};
        if (!retained_solid::Hash(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin()); if (expected != actual) return false;
        part_boolean::Reader reader(bytes, bytes.size() - 32);
        std::array<std::uint8_t, 8> prefix{};
        std::uint64_t operation = 0, cut = 0, material = 0, reserved = 0, count = 0;
        Definition value;
        if (!reader.raw(prefix) || !reader.integer(1, operation) || !reader.integer(1, cut)
            || !reader.integer(1, material) || !reader.integer(1, reserved) || reserved) return false;
        value.operation = part_boolean::Operation(operation);
        value.cutSurfacePolicy = CutSurfacePolicy(cut);
        value.materialPolicy = part_boolean::MaterialPolicy(material);
        std::uint32_t* versions[] = {&value.versions.serializer, &value.versions.build,
            &value.versions.proof, &value.versions.selector, &value.versions.material};
        for (auto* version : versions) { std::uint64_t x = 0;
            if (!reader.integer(4, x) || x > UINT32_MAX) return false;
            *version = std::uint32_t(x); }
        if (!codec_detail::Owner(reader, value.owner) || !reader.raw(value.booleanNode)
            || !reader.raw(value.feature)
            || !part_boolean::ReadCommitments(reader, value.ownerOutputCommitments)
            || !reader.integer(1, count) || count != 2
            || !reader.integer(3, reserved) || reserved) return false;
        for (auto& input : value.inputs) if (!codec_detail::Input(reader, input)) return false;
        if (!reader.complete() || !Valid(value)) return false;
        output = std::move(value); return true;
    } catch (...) { output = Definition{}; return false; }
}
} // namespace core3d::part_boolean::program_operand
