#pragma once

#include <Core3D/FacePushPullBuild.hxx>
#include <Core3D/RetainedFeatureReplay.hxx>
#include <BRep_Builder.hxx>
#include <cstdio>

namespace core3d::face_push_pull {
namespace registry_detail {
inline bool buildObserved(const composite_recipe::FeatureNode& feature,
                           const std::vector<retained_feature::ReplayValue>& inputs,
                           retained_topology_budget::Counter& topology,
                           const std::atomic_bool& cancelled,
                           retained_feature::ReplayValue& output) noexcept {
    output = {};
    try {
        if (feature.kind != FeatureKind || feature.codecVersion != CodecVersion
            || feature.inputs.size() != 1 || inputs.size() != 1 || !inputs[0].valid()
            || inputs[0].shape != retained_feature::ShapeKind::Solid || cancelled.load()
            || inputs[0].detachedShape.size() > composite_recipe::MaximumDocumentAggregateBytes) return false;
        Definition definition; retained_recipe::Digest inputHash{};
        if (!Decode(feature.parameters, definition)
            || !composite_recipe::Hash(inputs[0].detachedShape, inputHash) || inputHash != inputs[0].geometry) return false;
        const std::string value(inputs[0].detachedShape.begin(), inputs[0].detachedShape.end());
        std::istringstream stream(value); stream.imbue(std::locale::classic());
        TopoDS_Shape source; BRep_Builder builder; BRepTools::Read(source, stream, builder);
        if (stream.fail() || source.IsNull()) return false;
        stream >> std::ws; if (!stream.eof()) return false;
        BuildResult built = BuildDeterministically(source, definition, cancelled, topology);
        if (!built.built() || retained_topology_budget::ChargeTraversal(built.solid, topology,
                cancelled, detail::Site::C15ReplaySteps) != retained_topology_budget::WalkStatus::Completed) return false;
        retained_feature::ReplayValue result;
        if (!detail::exactShapeBytes(built.solid, result.detachedShape)
            || !composite_recipe::Hash(result.detachedShape, result.geometry)) return false;
        // Bind the family proof to payload, actual input geometry and output.
        std::vector<std::uint8_t> proof = feature.parameters;
        proof.insert(proof.end(), inputHash.begin(), inputHash.end());
        proof.insert(proof.end(), result.geometry.begin(), result.geometry.end());
        if (!composite_recipe::Hash(proof, result.familyProof) || cancelled.load()) return false;
        result.shape = retained_feature::ShapeKind::Solid;
        if (!result.valid()) return false;
        output = std::move(result); return true;
    } catch (...) { output = {}; return false; }
}
inline bool build(const composite_recipe::FeatureNode& feature,
                   const std::vector<retained_feature::ReplayValue>& inputs,
                   retained_feature::ReplayBudget& budget, retained_feature::ReplayValue& output) noexcept {
    output = {};
    if (budget.exhausted || inputs.size() != 1
        || !budget.consume(1, feature.inputs.size(), feature.parameters.size())
        || !budget.consume(0, 0, inputs[0].detachedShape.size())) return false;
    retained_topology_budget::Counter topology;
    const std::atomic_bool cancelled{false}; retained_feature::ReplayValue result;
    const bool built = buildObserved(feature, inputs, topology, cancelled, result);
    if (!budget.consume(0, 0, topology.topologyVisits)) return false;
    if (!built || !budget.consume(0, 0, result.detachedShape.size())) return false;
    output = std::move(result); return true;
}
inline bool prove(const composite_recipe::FeatureNode& feature,
                   const std::vector<retained_feature::ReplayValue>& inputs,
                   const retained_feature::ReplayValue& output,
                   retained_feature::ReplayBudget& budget) noexcept {
    retained_feature::ReplayValue rebuilt;
    return output.valid() && output.shape == retained_feature::ShapeKind::Solid
        && build(feature, inputs, budget, rebuilt) && rebuilt.detachedShape == output.detachedShape
        && rebuilt.geometry == output.geometry && rebuilt.familyProof == output.familyProof;
}
} // namespace registry_detail

// P1 exposes a detached descriptor only. F3 owns production codec-table
// installation; P2 owns call-bound execution, Apply and editor/dependency
// routes. No caller can enable those routes by passing booleans here.
inline retained_feature::Entry RegistryEntry() noexcept {
    retained_feature::CodecDescriptor codec;
    codec.key = {FeatureKind, CodecVersion}; codec.graphMajor = 3;
    codec.maximumPayloadBytes = MaximumPayloadBytes;
    codec.orderedInputs[0] = retained_feature::ShapeKind::Solid;
    codec.inputCount = 1; codec.output = retained_feature::ShapeKind::Solid;
    codec.canonicalPayload = CanonicalPayload;
    retained_feature::ExecutionDescriptor execution;
    execution.buildDetached = registry_detail::build;
    execution.proveFamily = registry_detail::prove;
    execution.verifyFixedPoint = registry_detail::prove;
    std::fprintf(stderr, "T3_P1_PUSHPULL kind=%u registry=ok\n", unsigned(FeatureKind));
    return {codec, execution};
}
} // namespace core3d::face_push_pull
