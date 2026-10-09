#pragma once

#include "CompositeRecipeAttribute.hxx"
#include "PartBooleanPersistence.hxx"
#include "../UI/RetainedA1AnalyticInputAdapter.hxx"
#include "RetainedEdgeTreatmentR2Build.hxx"
#include "RetainedEdgeTreatmentR2Snapshot.hxx"

#include <atomic>
#include <cstring>
#include <memory>

namespace core3d::authored_boolean {

enum class Status : std::uint8_t {
    Prepared = 0,
    Unchanged,
    Malformed,
    UnsupportedSource,
    IdentityChanged,
    FieldChanged,
    InvalidDefinition,
    BuildFailed,
    Budget,
    Cancelled,
};

enum class Kind : std::uint8_t { AnalyticInput = 0, Operation, Placement };

struct Candidate final {
    std::shared_ptr<const composite_recipe::Payload> composite;
    retained_edge_treatment::r2::Definition treatment;
    std::vector<std::uint8_t> treatmentBytes;
    TopoDS_Shape untreated;
    TopoDS_Shape treated;
    retained_edge_treatment::ReplayBudget budget;
};

namespace detail {

inline bool SameDouble(double left, double right) noexcept {
    return std::memcmp(&left, &right, sizeof(double)) == 0;
}

template <typename Vector>
inline bool SameVector(const Vector& left, const Vector& right) noexcept {
    if (left.size() != right.size()) return false;
    for (std::size_t index = 0; index < left.size(); ++index)
        if (!SameDouble(left[index], right[index])) return false;
    return true;
}

inline bool SameStableInput(const part_boolean::AnalyticPrismInput& left,
                            const part_boolean::AnalyticPrismInput& right) noexcept {
    return left.rootNode == right.rootNode
        && left.originalSourceFeature == right.originalSourceFeature
        && left.commitments.geometry == right.commitments.geometry
        && left.commitments.recipe == right.commitments.recipe
        && left.commitments.placement == right.commitments.placement
        && left.commitments.material == right.commitments.material
        && left.commitments.groups == right.commitments.groups
        && SameDouble(left.metersPerUnit, right.metersPerUnit)
        && left.originalMaterial.identifier == right.originalMaterial.identifier
        && left.originalMaterial.kind == right.originalMaterial.kind
        && left.originalMaterial.baseColorSRGB == right.originalMaterial.baseColorSRGB
        && SameDouble(left.originalMaterial.metallic, right.originalMaterial.metallic)
        && SameDouble(left.originalMaterial.roughness, right.originalMaterial.roughness)
        && left.originalMaterial.resource == right.originalMaterial.resource
        && left.originalMaterial.resourceDigest == right.originalMaterial.resourceDigest
        && left.originalName == right.originalName
        && left.originalGroups == right.originalGroups
        && left.originallyVisible == right.originallyVisible;
}

inline bool Extract(const retained_edge_treatment::r2::Snapshot& snapshot,
                    composite_recipe::Definition& graph,
                    part_boolean::AnalyticDefinition& analytic,
                    const composite_recipe::FeatureNode*& feature) noexcept {
    graph = {}; analytic = {}; feature = nullptr;
    try {
        namespace r2 = retained_edge_treatment::r2;
        const auto* retained = std::get_if<r2::RetainedBooleanBase>(&snapshot.source());
        const auto* composite = retained
            ? std::get_if<r2::CompositeBooleanBase>(&retained->source) : nullptr;
        const auto* binding = std::get_if<r2::BooleanBaseBinding>(&snapshot.definition().base);
        const auto* links = binding
            ? std::get_if<r2::CompositePrefixBinding>(&binding->prefix) : nullptr;
        if (!composite || !binding || !links
            || binding->format != r2::PrefixFormat::A1Composite
            || links->links.size() != 1
            || !composite_recipe::Decode(composite->canonicalPrefixBytes, graph)
            || graph.nodes.size() != 3 || graph.owner != snapshot.definition().owner)
            return false;
        const auto* left = std::get_if<composite_recipe::SourceNode>(&graph.nodes[0].value);
        const auto* right = std::get_if<composite_recipe::SourceNode>(&graph.nodes[1].value);
        feature = std::get_if<composite_recipe::FeatureNode>(&graph.nodes[2].value);
        if (!left || !right || !feature
            || feature->kind != composite_recipe::PartBooleanFeatureKind
            || feature->codecVersion != composite_recipe::PartBooleanFeatureCodec
            || feature->inputs != std::vector<retained_recipe::UUID>{left->node, right->node}
            || feature->node != graph.outputNode
            || !part_boolean::DecodeAnalytic(feature->parameters, analytic)
            || analytic.inputs[0].rootNode != left->node
            || analytic.inputs[1].rootNode != right->node
            || analytic.inputs[0].originalSourceFeature != left->original.sourceFeature
            || analytic.inputs[1].originalSourceFeature != right->original.sourceFeature)
            return false;
        return true;
    } catch (...) { graph = {}; analytic = {}; feature = nullptr; return false; }
}

inline retained_recipe::DependencyRead Read(
    const retained_recipe::OwnerKey& owner,
    const composite_recipe::SourceNode& source) noexcept {
    return {{owner, source.node, source.original.sourceFeature},
        source.commitments.geometry, source.commitments.recipe,
        source.commitments.placement, source.commitments.material,
        source.commitments.groups};
}

} // namespace detail

inline Status Describe(const retained_edge_treatment::r2::Snapshot& snapshot,
                       part_boolean::AnalyticDefinition& output) noexcept {
    composite_recipe::Definition graph;
    const composite_recipe::FeatureNode* feature = nullptr;
    return detail::Extract(snapshot, graph, output, feature)
        ? Status::Prepared : Status::UnsupportedSource;
}

inline Status Prepare(const retained_edge_treatment::r2::Snapshot& snapshot,
                      Kind capability,
                      std::size_t inputIndex,
                      const part_boolean::AnalyticDefinition& requested,
                      const TopoDS_Shape& originalUntreated,
                      const std::atomic_bool& cancelled,
                      Candidate& output) noexcept {
    output = {};
    try {
        if (inputIndex > 1 || !part_boolean::Valid(requested)) return Status::Malformed;
        composite_recipe::Definition graph;
        part_boolean::AnalyticDefinition original;
        const composite_recipe::FeatureNode* oldFeature = nullptr;
        if (!detail::Extract(snapshot, graph, original, oldFeature))
            return Status::UnsupportedSource;
        if (requested.materialPolicy != original.materialPolicy
            || requested.versions.serializer != original.versions.serializer
            || requested.versions.build != original.versions.build
            || requested.versions.proof != original.versions.proof
            || requested.versions.selector != original.versions.selector
            || requested.versions.material != original.versions.material
            || requested.nativeAdmissionEnabled != original.nativeAdmissionEnabled)
            return Status::IdentityChanged;
        for (std::size_t index = 0; index < 2; ++index)
            if (!detail::SameStableInput(original.inputs[index], requested.inputs[index]))
                return Status::IdentityChanged;

        const auto dimensionsChanged = [&](std::size_t index) {
            return !detail::SameVector(original.inputs[index].dimensions,
                                       requested.inputs[index].dimensions);
        };
        const auto placementChanged = [&](std::size_t index) {
            return !detail::SameVector(original.inputs[index].translation,
                                       requested.inputs[index].translation)
                || !detail::SameVector(original.inputs[index].rotationXYZW,
                                        requested.inputs[index].rotationXYZW);
        };
        const bool operationChanged = requested.operation != original.operation;
        if (capability == Kind::AnalyticInput) {
            if (operationChanged || !dimensionsChanged(inputIndex)
                || dimensionsChanged(1 - inputIndex)
                || placementChanged(0) || placementChanged(1)) return Status::FieldChanged;
        } else if (capability == Kind::Operation) {
            if (!operationChanged || dimensionsChanged(0) || dimensionsChanged(1)
                || placementChanged(0) || placementChanged(1)) return Status::FieldChanged;
        } else if (capability == Kind::Placement) {
            if (operationChanged || dimensionsChanged(0) || dimensionsChanged(1)
                || !placementChanged(inputIndex) || placementChanged(1 - inputIndex))
                return Status::FieldChanged;
        } else return Status::Malformed;

        std::vector<std::uint8_t> oldBytes, requestedBytes;
        if (!part_boolean::EncodeAnalytic(original, oldBytes)
            || !part_boolean::EncodeAnalytic(requested, requestedBytes))
            return Status::InvalidDefinition;
        if (oldBytes == requestedBytes) return Status::Unchanged;

        const auto* left = std::get_if<composite_recipe::SourceNode>(&graph.nodes[0].value);
        const auto* right = std::get_if<composite_recipe::SourceNode>(&graph.nodes[1].value);
        if (!left || !right) return Status::UnsupportedSource;
        retained_part_boolean::OperandReadSet reads{
            detail::Read(graph.owner, *left), detail::Read(graph.owner, *right)};
        const double unit = snapshot.dimensionMetersPerUnit();
        const auto replay = a1_analytic_input_adapter::ReplayCounted(
            requested, unit, reads, reads, output.budget, cancelled);
        if (replay.status == a1_analytic_input_adapter::ReplayStatus::Budget)
            return Status::Budget;
        if (replay.status == a1_analytic_input_adapter::ReplayStatus::Cancelled)
            return Status::Cancelled;
        if (!replay.proven()) return Status::BuildFailed;

        auto payload = std::make_shared<composite_recipe::Payload>();
        payload->definition = graph;
        payload->sourceShapes.assign(replay.production.sources.begin(),
                                     replay.production.sources.end());
        part_boolean::AnalyticDefinition staged = requested;
        for (std::size_t index = 0; index < 2; ++index) {
            auto* source = std::get_if<composite_recipe::SourceNode>(
                &payload->definition.nodes[index].value);
            const std::vector<std::uint8_t> sourceBytes(
                replay.production.sourceBytes[index].begin(),
                replay.production.sourceBytes[index].end());
            if (!source || !composite_recipe::Hash(
                    sourceBytes, source->commitments.geometry))
                return Status::BuildFailed;
            staged.inputs[index].commitments.geometry = source->commitments.geometry;
        }
        auto* feature = std::get_if<composite_recipe::FeatureNode>(
            &payload->definition.nodes[2].value);
        if (!feature || !part_boolean::EncodeAnalytic(staged, feature->parameters))
            return Status::InvalidDefinition;
        if (!composite_recipe::Encode(payload->definition, payload->bytes))
            return Status::InvalidDefinition;

        output.untreated = replay.production.candidate.solid;
        output.treatment = snapshot.definition();
        retained_edge_treatment::Refusal refusal;
        retained_edge_treatment::Definition shadow;
        const auto* binding = std::get_if<retained_edge_treatment::r2::BooleanBaseBinding>(
            &output.treatment.base);
        if (!binding || originalUntreated.IsNull()) return Status::UnsupportedSource;
        shadow.schema = output.treatment.schema;
        shadow.owner = output.treatment.owner;
        shadow.base.family = retained_edge_treatment::SourceFamily::Profile;
        shadow.base.source = binding->source;
        shadow.base.sourceNode = binding->sourceNode;
        shadow.base.sourceSchema = binding->sourceSchema;
        shadow.base.sourceRecipeDigest = binding->sourceRecipeDigest;
        shadow.base.metersPerLocalUnit = binding->metersPerLocalUnit;
        shadow.issuance = output.treatment.issuance;
        shadow.outputNode = output.treatment.outputNode;
        shadow.steps = output.treatment.steps;
        std::vector<std::uint8_t> shadowBytes;
        auto roles = std::make_shared<retained_edge_treatment::SourceRebindRoles>();
        if (!retained_edge_treatment::Encode(shadow, shadowBytes, refusal)
            || !retained_edge_treatment::CaptureSourceRebindRoles(
                originalUntreated, shadow, shadowBytes, output.budget, refusal, *roles)
            || !retained_edge_treatment::r2::ApplySourceRebindR2(
                *roles, output.untreated, output.treatment, output.treated,
                output.budget, refusal))
            return output.budget.exhausted ? Status::Budget : Status::BuildFailed;
        auto& stagedBinding = std::get<retained_edge_treatment::r2::BooleanBaseBinding>(
            output.treatment.base);
        if (!composite_recipe::Hash(payload->bytes, stagedBinding.sourceRecipeDigest)
            || !retained_edge_treatment::r2::Encode(
                output.treatment, output.treatmentBytes, refusal))
            return Status::BuildFailed;
        output.composite = std::move(payload);
        return Status::Prepared;
    } catch (...) { output = {}; return Status::BuildFailed; }
}

} // namespace core3d::authored_boolean
