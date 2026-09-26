#pragma once
#include "CompositeRecipeCodec.hxx"
#include "RetainedSolidAttribute.hxx"
#include <string>

namespace core3d::retained_recipe {
enum class OwnerStatus : std::uint8_t {
    CurrentEditable = 1,
    AbsentLegacy = 2,
    ReadableNoncurrent = 3,
    UnsupportedVersion = 4,
    Malformed = 5,
    Ambiguous = 6,
};

struct DependencyRead {
    RecipeLocator locator;
    Digest geometry{}, recipe{}, placement{}, material{}, groups{};
    bool operator==(const DependencyRead& value) const noexcept {
        return locator == value.locator && geometry == value.geometry && recipe == value.recipe
            && placement == value.placement && material == value.material && groups == value.groups;
    }
};

struct RevisionFence {
    std::uint64_t documentGeneration = 0, modelRevision = 0;
    double effectiveMetersPerUnit = 0;
    Digest ownerShape{}, ownerRecipe{}, ownerPlacement{}, ownerMaterial{};
    std::vector<DependencyRead> dependencies;
};

struct SourceSnapshot {
    RecipeLocator locator;
    SourceIdentity original;
    composite_recipe::SourceRecipe recipe;
    composite_recipe::InputPlacement inputToCarrier;
    composite_recipe::Commitments commitments;
};

struct FeatureSnapshot {
    RecipeLocator locator;
    std::uint32_t kind = 0, codecVersion = 0;
    std::vector<UUID> orderedInputs;
    std::vector<std::uint8_t> parameters;
};

struct OwnerSnapshot {
    OwnerStatus status = OwnerStatus::Malformed;
    std::string reason;
    OwnerKey owner;
    UUID outputNode{};
    RevisionFence fence;
    std::vector<SourceSnapshot> sources;
    std::vector<FeatureSnapshot> features;
};

// Process-local receipt minted only after the document owner has decoded the
// canonical SYPB/1 feature, rebuilt both inputs/result, proved correspondence,
// and captured the complete native read set. It is intentionally absent from
// every persistence codec.
struct NativeCurrentnessFacts final {
    OwnerKey owner;
    UUID outputNode{}, feature{};
    Digest graph{}, outputShape{}, readSet{};
    std::uint64_t ownerNonce = 0;
    bool builderInstalled = false;
    bool proofInstalled = false;
    bool ownerInstalled = false;
    bool persistenceInstalled = false;
};

struct GraphNodeFact final {
    UUID node{}, feature{};
    std::uint32_t kind = 0, codecVersion = 0;
    std::vector<UUID> orderedInputs;
    Digest payload{}, output{}, familyProof{};
};

struct GraphCurrentnessFacts final {
    OwnerKey owner;
    UUID outputNode{};
    Digest graph{}, outputShape{}, readSet{}, sourceMetadata{}, units{}, dependencyFence{};
    std::vector<GraphNodeFact> orderedNodes;
    std::uint64_t ownerNonce = 0;
    bool persistenceInstalled = false;
};

inline bool Valid(const GraphCurrentnessFacts& value,
                  const composite_recipe::Definition& definition,
                  const retained_feature::RegistryView& registry) noexcept {
    try {
        if (definition.schemaVersion != 3 || value.ownerNonce == 0
            || !value.persistenceInstalled || !(value.owner == definition.owner)
            || value.outputNode != definition.outputNode || !Nonzero(value.graph)
            || !Nonzero(value.outputShape) || !Nonzero(value.readSet)
            || !Nonzero(value.sourceMetadata) || !Nonzero(value.units)
            || !Nonzero(value.dependencyFence)) return false;
        std::vector<std::uint8_t> bytes; Digest graph;
        if (!composite_recipe::EncodeV3(definition, bytes)
            || !composite_recipe::Hash(bytes, graph) || graph != value.graph) return false;
        std::size_t fact = 0;
        for (const auto& node : definition.nodes) {
            const auto* feature = std::get_if<composite_recipe::FeatureNode>(&node.value);
            if (!feature) continue;
            if (fact >= value.orderedNodes.size()) return false;
            const auto* entry = registry.find({feature->kind, feature->codecVersion});
            const auto& observed = value.orderedNodes[fact++]; Digest payload;
            if (!entry || !entry->execution.installed()
                || !composite_recipe::Hash(feature->parameters, payload)
                || observed.node != feature->node || observed.feature != feature->feature
                || observed.kind != feature->kind || observed.codecVersion != feature->codecVersion
                || observed.orderedInputs != feature->inputs || observed.payload != payload
                || !Nonzero(observed.output) || !Nonzero(observed.familyProof)) return false;
        }
        return fact == value.orderedNodes.size()
            && !value.orderedNodes.empty() && value.orderedNodes.back().node == definition.outputNode
            && value.orderedNodes.back().output == value.outputShape;
    } catch (...) { return false; }
}

inline bool Valid(const NativeCurrentnessFacts& value,
                  const composite_recipe::Definition& definition) noexcept {
    if (!value.builderInstalled || !value.proofInstalled || !value.ownerInstalled
        || !value.persistenceInstalled || value.ownerNonce == 0
        || !(value.owner == definition.owner) || value.outputNode != definition.outputNode
        || !Nonzero(value.feature) || !Nonzero(value.graph)
        || !Nonzero(value.outputShape) || !Nonzero(value.readSet)) return false;
    const auto* feature = definition.nodes.empty() ? nullptr
        : std::get_if<composite_recipe::FeatureNode>(&definition.nodes.back().value);
    return feature && feature->feature == value.feature;
}

inline bool Valid(const RevisionFence& fence) noexcept {
    if (fence.documentGeneration == 0 || fence.modelRevision == 0
        || !std::isfinite(fence.effectiveMetersPerUnit) || fence.effectiveMetersPerUnit <= 0
        || !Nonzero(fence.ownerShape) || !Nonzero(fence.ownerRecipe)
        || !Nonzero(fence.ownerPlacement) || !Nonzero(fence.ownerMaterial)
        || fence.dependencies.empty() || fence.dependencies.size() > composite_recipe::MaximumNodes) return false;
    std::set<UUID> nodes;
    for (const auto& read : fence.dependencies)
        if (!Valid(read.locator) || !nodes.insert(read.locator.node).second
            || !Nonzero(read.geometry) || !Nonzero(read.recipe) || !Nonzero(read.placement)
            || !Nonzero(read.material) || !Nonzero(read.groups)) return false;
    return true;
}

inline OwnerSnapshot Snapshot(const composite_recipe::Definition& definition,
                              const RevisionFence& fence,
                              const NativeCurrentnessFacts* facts = nullptr,
                              const GraphCurrentnessFacts* graphFacts = nullptr,
                              const retained_feature::RegistryView* graphRegistry = nullptr) noexcept {
    OwnerSnapshot result;
    try {
        result.owner = definition.owner; result.outputNode = definition.outputNode; result.fence = fence;
        if (!composite_recipe::Valid(definition)) { result.reason = "malformed-composite"; return result; }
        if (!Valid(fence)) { result.status = OwnerStatus::ReadableNoncurrent; result.reason = "stale-or-incomplete-fence"; return result; }
        if (!(definition.owner == fence.dependencies.front().locator.owner)) {
            result.status = OwnerStatus::Ambiguous; result.reason = "foreign-fence-owner"; return result;
        }
        std::set<UUID> dependencyNodes;
        for (const auto& read : fence.dependencies) {
            if(!(read.locator.owner==definition.owner)){
                result.status=OwnerStatus::Ambiguous;result.reason="foreign-dependency-owner";return result;
            }
            dependencyNodes.insert(read.locator.node);
        }
        for (const auto& node : definition.nodes) if (const auto* source = std::get_if<composite_recipe::SourceNode>(&node.value)) {
            RecipeLocator locator{definition.owner, source->node, source->original.sourceFeature};
            const auto read=std::find_if(fence.dependencies.begin(),fence.dependencies.end(),
                [&](const DependencyRead& value){return value.locator==locator;});
            if (!dependencyNodes.count(source->node)||read==fence.dependencies.end()
                ||read->geometry!=source->commitments.geometry||read->recipe!=source->commitments.recipe
                ||read->placement!=source->commitments.placement||read->material!=source->commitments.material
                ||read->groups!=source->commitments.groups) {
                result.status = OwnerStatus::ReadableNoncurrent; result.reason = "missing-source-read"; return result;
            }
            result.sources.push_back({locator, source->original, source->recipe,
                source->inputToCarrier, source->commitments});
        }
        for (const auto& node : definition.nodes) if (const auto* feature = std::get_if<composite_recipe::FeatureNode>(&node.value))
            result.features.push_back({{definition.owner, feature->node, feature->feature},
                feature->kind, feature->codecVersion, feature->inputs, feature->parameters});
        if (definition.schemaVersion == 3) {
            const auto& registry = graphRegistry ? *graphRegistry : retained_feature::ProductionRegistry();
            if (graphFacts && Valid(*graphFacts, definition, registry)) {
                result.status = OwnerStatus::CurrentEditable; result.reason = "native-v3-graph-current";
            } else {
                result.status = OwnerStatus::UnsupportedVersion;
                result.reason = "v3-native-currentness-not-installed";
            }
            return result;
        }
        const auto* feature = definition.nodes.empty() ? nullptr
            : std::get_if<composite_recipe::FeatureNode>(&definition.nodes.back().value);
        part_boolean::AnalyticDefinition analytic;
        bool exactAnalytic = definition.schemaVersion == 1 && result.sources.size() == 2
            && definition.nodes.size() == 3 && feature
            && feature->kind == composite_recipe::PartBooleanFeatureKind
            && feature->codecVersion == composite_recipe::PartBooleanFeatureCodec
            && feature->inputs.size() == 2 && part_boolean::DecodeAnalytic(feature->parameters, analytic);
        for (std::size_t index = 0; exactAnalytic && index < 2; ++index) {
            const auto& source = result.sources[index];
            const auto& input = analytic.inputs[index];
            exactAnalytic = feature->inputs[index] == source.locator.node
                && input.rootNode == source.locator.node
                && input.originalSourceFeature == source.locator.sourceFeature
                && input.commitments.geometry == source.commitments.geometry
                && input.commitments.recipe == source.commitments.recipe
                && input.commitments.placement == source.commitments.placement
                && input.commitments.material == source.commitments.material
                && input.commitments.groups == source.commitments.groups;
        }
        part_boolean::Definition shell;
        bool exactShell = definition.schemaVersion == 2 && result.sources.size() == 2
            && definition.nodes.size() == 3 && feature
            && feature->kind == composite_recipe::PartBooleanFeatureKind
            && feature->codecVersion == composite_recipe::PartBooleanShellFeatureCodec
            && feature->inputs.size() == 2
            && part_boolean::Decode(feature->parameters, shell)
            && shell.inputs[0].family == part_boolean::InputFamily::ShellProfile
            && shell.inputs[1].family
                == part_boolean::InputFamily::AnalyticRectangularPrism;
        for (std::size_t index = 0; exactShell && index < 2; ++index) {
            const auto& source = result.sources[index];
            const auto& input = shell.inputs[index];
            exactShell = feature->inputs[index] == source.locator.node
                && input.rootNode == source.locator.node
                && input.originalSourceFeature == source.locator.sourceFeature
                && input.commitments.geometry == source.commitments.geometry
                && input.commitments.recipe == source.commitments.recipe
                && input.commitments.placement == source.commitments.placement
                && input.commitments.material == source.commitments.material
                && input.commitments.groups == source.commitments.groups
                && source.recipe.kind == composite_recipe::RecipeKind::Profile
                && ((index == 0 && source.recipe.schema == 5)
                    || (index == 1 && source.recipe.schema >= 1
                        && source.recipe.schema <= 4));
        }
        if (exactAnalytic && facts && Valid(*facts, definition)) {
            result.status = OwnerStatus::CurrentEditable;
            result.reason = "native-analytic-current";
            return result;
        }
        if (exactShell && facts && Valid(*facts, definition)) {
            result.status = OwnerStatus::CurrentEditable;
            result.reason = "native-shell-current";
            return result;
        }
        // C2's builder/proof packages are installed before this serial G0
        // transition. Recognize only the exact two-node curve->sweep graph;
        // every downstream feature remains readable but non-editable until
        // its own source-family proof is installed.
        bool spatial = definition.schemaVersion == 2
            && definition.nodes.size() == 2 && result.sources.size() == 1;
        if (spatial) {
            const auto* source = std::get_if<composite_recipe::SourceNode>(
                &definition.nodes.front().value);
            const auto* feature = std::get_if<composite_recipe::FeatureNode>(
                &definition.nodes.back().value);
            spatial = source && feature
                && source->recipe.kind == composite_recipe::RecipeKind::BoundedCurvePath
                && feature->kind == composite_recipe::SpatialCircleSweepFeatureKind
                && feature->codecVersion == composite_recipe::SpatialCircleSweepFeatureCodec
                && feature->inputs.size() == 1 && feature->inputs.front() == source->node
                && definition.outputNode == feature->node;
        }
        if (spatial) {
            result.status = OwnerStatus::CurrentEditable;
            result.reason = "spatial-circle-sweep-current"; return result;
        }
        // Structural decode never grants editability without the nonserializable
        // owner receipt. Malformed present records remain distinct from absence.
        result.status = OwnerStatus::UnsupportedVersion;
        result.reason = (exactAnalytic || exactShell)
            ? "native-currentness-not-installed" : "unsupported-dependent-feature";
        return result;
    } catch (...) { return {}; }
}

inline OwnerSnapshot AbsentLegacy(const OwnerKey& owner) noexcept;

inline OwnerSnapshot LegacySnapshot(const OwnerKey& owner, const UUID& node,
                                    const SourceIdentity& source,
                                    composite_recipe::SourceRecipe recipe,
                                    composite_recipe::InputPlacement placement,
                                    composite_recipe::Commitments commitments,
                                    const RevisionFence& fence,
                                    bool current) noexcept {
    OwnerSnapshot result;
    try {
        result.owner = owner; result.outputNode = node; result.fence = fence;
        if (!Valid(owner) || !Valid(source) || !Nonzero(node)
            || source.document != owner.document || !composite_recipe::ValidRecipe(recipe)
            || !composite_recipe::ValidPlacement(placement) || !composite_recipe::Valid(commitments)) {
            result.reason = "malformed-legacy-recipe"; return result;
        }
        result.sources.push_back({{owner,node,source.sourceFeature},source,std::move(recipe),placement,commitments});
        if (!current || !Valid(fence)) {
            result.status = OwnerStatus::ReadableNoncurrent; result.reason = "legacy-recipe-not-current";
        } else {
            result.status = OwnerStatus::CurrentEditable; result.reason = "legacy-current";
        }
        return result;
    } catch (...) { return {}; }
}

inline OwnerSnapshot LegacyProfileSnapshot(const OwnerKey& owner, const UUID& node,
                                           const SourceIdentity& source,
                                           const profile::Record& record,
                                           const composite_recipe::InputPlacement& placement,
                                           const composite_recipe::Commitments& commitments,
                                           const RevisionFence& fence) noexcept {
    composite_recipe::SourceRecipe recipe; recipe.kind = composite_recipe::RecipeKind::Profile;
    recipe.schema = record.label.IsNull() ? 0 : std::uint32_t(profile::SchemaFor(record.parameters));
    if (record.label.IsNull() || !composite_recipe::EncodeScalarRecipe(recipe.kind, recipe.schema,
            record.values, recipe.bytes)) return AbsentLegacy(owner);
    return LegacySnapshot(owner,node,source,std::move(recipe),placement,commitments,fence,
        record.IsCurrent(TDocStd_Document::Get(record.label),record.label.Father()));
}

inline OwnerSnapshot LegacyEnclosureSnapshot(const OwnerKey& owner, const UUID& node,
                                             const SourceIdentity& source,
                                             const enclosure::Record& record,
                                             const composite_recipe::InputPlacement& placement,
                                             const composite_recipe::Commitments& commitments,
                                             const RevisionFence& fence) noexcept {
    composite_recipe::SourceRecipe recipe; recipe.kind = composite_recipe::RecipeKind::Enclosure;
    recipe.schema = record.label.IsNull() ? 0 : std::uint32_t(record.parameters.definition.constructionFrame
        ? enclosure::FramedSchemaVersion : enclosure::SchemaVersion);
    if (record.label.IsNull() || !composite_recipe::EncodeScalarRecipe(recipe.kind,recipe.schema,
            record.values,recipe.bytes)) return AbsentLegacy(owner);
    return LegacySnapshot(owner,node,source,std::move(recipe),placement,commitments,fence,
        record.IsCurrent(TDocStd_Document::Get(record.label),record.label.Father()));
}

inline OwnerSnapshot LegacyLoftSnapshot(const OwnerKey& owner, const UUID& node,
                                        const SourceIdentity& source,
                                        const loft_persistence::Record& record,
                                        const composite_recipe::InputPlacement& placement,
                                        const composite_recipe::Commitments& commitments,
                                        const RevisionFence& fence) noexcept {
    composite_recipe::SourceRecipe recipe; recipe.kind = composite_recipe::RecipeKind::RectangularLoft;
    recipe.schema = loft_persistence::Schema;
    if (record.label.IsNull() || !composite_recipe::EncodeScalarRecipe(recipe.kind,recipe.schema,
            record.values,recipe.bytes)) return AbsentLegacy(owner);
    return LegacySnapshot(owner,node,source,std::move(recipe),placement,commitments,fence,
        record.IsCurrent(TDocStd_Document::Get(record.label),record.label.Father()));
}

inline OwnerSnapshot LegacyRetainedSnapshot(const OwnerKey& owner, const UUID& node,
                                            const SourceIdentity& source,
                                            const retained_solid::Record& record,
                                            const composite_recipe::InputPlacement& placement,
                                            composite_recipe::Commitments commitments,
                                            const RevisionFence& fence) noexcept {
    if (record.label.IsNull() || !record.value) return AbsentLegacy(owner);
    composite_recipe::SourceRecipe recipe{composite_recipe::RecipeKind::RetainedBoolean,1,record.value->bytes};
    composite_recipe::Hash(recipe.bytes,commitments.recipe);
    double unit=0;const auto document=TDocStd_Document::Get(record.label);
    const bool current=!document.IsNull()&&XCAFDoc_DocumentTool::GetLengthUnit(document,unit)
        && retained_solid::Bits(unit)==retained_solid::Bits(placement.sourceMetersPerUnit)
        && !record.current.IsNull()&&record.current.IsEqual(XCAFDoc_ShapeTool::GetShape(record.owner));
    return LegacySnapshot(owner,node,source,std::move(recipe),placement,commitments,fence,current);
}

inline OwnerSnapshot AbsentLegacy(const OwnerKey& owner) noexcept {
    OwnerSnapshot result; result.owner = owner; result.status = OwnerStatus::AbsentLegacy;
    result.reason = "no-recipe-metadata"; return result;
}
} // namespace core3d::retained_recipe
