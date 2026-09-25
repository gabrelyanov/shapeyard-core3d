#pragma once

// Stacked on G0 SYCR/3. This descriptor is intentionally not inserted into
// ProductionRegistry by this source-bound draft. The serial integration owner
// installs it only with the OCAF transaction, Objects editor, and dependent
// replay routes; until then execution.installed() remains false.
#include "RetainedFeatureRegistry.hxx"
#include "RetainedFeatureReplay.hxx"
#include "VariableRadiusFilletBuild.hxx"
#include <BRep_Builder.hxx>
#include <sstream>

namespace core3d::variable_radius_fillet {

namespace detail {
inline bool readShape(const std::vector<std::uint8_t>& bytes, TopoDS_Shape& shape) {
    shape.Nullify();
    if (bytes.empty()) return false;
    const std::string value(bytes.begin(), bytes.end()); std::istringstream stream(value);
    stream.imbue(std::locale::classic()); BRep_Builder builder;
    BRepTools::Read(shape, stream, builder);
    return stream.good() && !shape.IsNull();
}

inline bool buildDetached(const composite_recipe::FeatureNode& feature,
                          const std::vector<retained_feature::ReplayValue>& inputs,
                          retained_feature::ReplayBudget& budget,
                          retained_feature::ReplayValue& output) noexcept {
    output = {};
    try {
        if (inputs.size() != 1 || inputs[0].shape != retained_feature::ShapeKind::Solid
            || !budget.consume(1, feature.inputs.size() + 2, feature.parameters.size())) return false;
        Definition definition; TopoDS_Shape source;
        if (!Decode(feature.parameters, definition) || !readShape(inputs[0].detachedShape, source)) return false;
        const std::atomic_bool cancelled{false};
        BuildResult built = BuildDeterministically(source, definition, cancelled);
        if (!built.built() || !exactShapeBytes(built.solid, output.detachedShape)
            || !composite_recipe::Hash(output.detachedShape, output.geometry)) return false;
        std::vector<std::uint8_t> proof;
        proof.reserve(feature.parameters.size() + output.detachedShape.size());
        proof.insert(proof.end(), feature.parameters.begin(), feature.parameters.end());
        proof.insert(proof.end(), output.geometry.begin(), output.geometry.end());
        if (!composite_recipe::Hash(proof, output.familyProof)) return false;
        output.shape = retained_feature::ShapeKind::Solid; return output.valid();
    } catch (...) { output = {}; return false; }
}

inline bool proveFamily(const composite_recipe::FeatureNode& feature,
                        const std::vector<retained_feature::ReplayValue>& inputs,
                        const retained_feature::ReplayValue& output,
                        retained_feature::ReplayBudget& budget) noexcept {
    retained_feature::ReplayValue rebuilt;
    if (!buildDetached(feature, inputs, budget, rebuilt)) return false;
    return rebuilt.detachedShape == output.detachedShape
        && rebuilt.geometry == output.geometry && rebuilt.familyProof == output.familyProof;
}

inline bool verifyFixedPoint(const composite_recipe::FeatureNode& feature,
                             const std::vector<retained_feature::ReplayValue>& inputs,
                             const retained_feature::ReplayValue& output,
                             retained_feature::ReplayBudget& budget) noexcept {
    return proveFamily(feature, inputs, output, budget);
}
} // namespace detail

inline retained_feature::Entry RegistryEntry(bool editorRouteInstalled,
                                             bool dependencyRouteInstalled) noexcept {
    retained_feature::CodecDescriptor codec;
    codec.key = {FeatureKind, CodecVersion}; codec.graphMajor = 3;
    codec.maximumPayloadBytes = MaximumPayloadBytes;
    codec.orderedInputs[0] = retained_feature::ShapeKind::Solid;
    codec.inputCount = 1; codec.output = retained_feature::ShapeKind::Solid;
    codec.canonicalPayload = CanonicalPayload;
    retained_feature::ExecutionDescriptor execution;
    execution.buildDetached = detail::buildDetached;
    execution.proveFamily = detail::proveFamily;
    execution.verifyFixedPoint = detail::verifyFixedPoint;
    execution.editorRouteInstalled = editorRouteInstalled;
    execution.dependencyRouteInstalled = dependencyRouteInstalled;
    return {codec, execution};
}

inline retained_feature::Entry DraftRegistryEntry() noexcept {
    return RegistryEntry(false, false);
}

} // namespace core3d::variable_radius_fillet
