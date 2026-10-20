#pragma once

// Stacked on the G0 SYCR/3 overlay. B4-S2 (D785) installs the descriptor in
// ProductionRegistry as a readable, canonical-only codec with an empty
// execution descriptor; public executable admission stays disabled, so
// DraftRegistryEntry().execution.installed() is false. The S2 native owner
// replays exclusively through the closed, call-bound owner execution
// registry at the bottom of this header (real owner callbacks, validation
// stubs that can never build, per-operation construction, owner-context
// pointer identity enforced by ReplayOwnerDetached). The Objects editor
// route remains S4 work.
#include "RetainedFeatureRegistry.hxx"
#include "RetainedFeatureReplay.hxx"
#include "RetainedFeatureOwner.hxx"
#include "DraftFacesBuild.hxx"
#include <BRep_Builder.hxx>
#include <array>
#include <sstream>

namespace core3d::draft_faces {
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
        if (!Decode(feature.parameters, definition)
            || !readShape(inputs[0].detachedShape, source)) return false;
        const std::atomic_bool cancelled{false};
        BuildResult built = BuildDeterministically(source, definition, cancelled);
        if (!built.built() || !exactShapeBytes(built.solid, output.detachedShape)
            || !composite_recipe::Hash(output.detachedShape, output.geometry)) return false;
        std::vector<std::uint8_t> proof;
        proof.reserve(feature.parameters.size() + output.geometry.size());
        proof.insert(proof.end(), feature.parameters.begin(), feature.parameters.end());
        proof.insert(proof.end(), output.geometry.begin(), output.geometry.end());
        if (!composite_recipe::Hash(proof, output.familyProof)) return false;
        output.shape = retained_feature::ShapeKind::Solid; return output.valid();
    } catch (...) { output = {}; return false; }
}

inline bool buildDetachedObserved(const composite_recipe::FeatureNode& feature,
                          const std::vector<retained_feature::ReplayValue>& inputs,
                          retained_topology_budget::Counter& topologyBudget,
                          const std::atomic_bool& cancelled,
                          retained_feature::ReplayValue& output,
                          Refusal *refusalOut = nullptr,
                          BuildEvidence *evidenceOut = nullptr) noexcept {
    output = {};
    Refusal refusal = Refusal::MalformedPayload;
    try {
        if (cancelled.load()) refusal = Refusal::Cancelled;
        else if (inputs.size() != 1 || inputs[0].shape != retained_feature::ShapeKind::Solid)
            refusal = Refusal::InvalidCandidate;
        else {
            Definition definition; TopoDS_Shape source;
            if (!Decode(feature.parameters, definition)) refusal = Refusal::MalformedPayload;
            else if (!readShape(inputs[0].detachedShape, source)) refusal = Refusal::InvalidCandidate;
            else {
                BuildResult built = BuildDeterministically(source, definition,
                                                            cancelled, topologyBudget);
                refusal = built.refusal;
                if (evidenceOut) *evidenceOut = built.evidence;
                if (built.built() && exactShapeBytes(built.solid, output.detachedShape)
                    && composite_recipe::Hash(output.detachedShape, output.geometry)) {
                    std::vector<std::uint8_t> proof;
                    proof.reserve(feature.parameters.size() + output.geometry.size());
                    proof.insert(proof.end(), feature.parameters.begin(), feature.parameters.end());
                    proof.insert(proof.end(), output.geometry.begin(), output.geometry.end());
                    if (composite_recipe::Hash(proof, output.familyProof)) {
                        output.shape = retained_feature::ShapeKind::Solid;
                        if (output.valid()) refusal = Refusal::None;
                    }
                }
                if (built.built() && refusal != Refusal::None) refusal = Refusal::InvalidCandidate;
            }
        }
    } catch (...) { output = {}; refusal = Refusal::BuildFailure; }
    if (cancelled.load()) { output = {}; refusal = Refusal::Cancelled; }
    if (refusalOut) *refusalOut = refusal;
    return refusal == Refusal::None && output.valid();
}

inline bool proveFamily(const composite_recipe::FeatureNode& feature,
                        const std::vector<retained_feature::ReplayValue>& inputs,
                        const retained_feature::ReplayValue& output,
                        retained_feature::ReplayBudget& budget) noexcept {
    retained_feature::ReplayValue rebuilt;
    return buildDetached(feature, inputs, budget, rebuilt)
        && rebuilt.detachedShape == output.detachedShape
        && rebuilt.geometry == output.geometry
        && rebuilt.familyProof == output.familyProof;
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

namespace owner_detail {
// Fail-closed validation stubs: the validation entry's execution descriptor is
// present with both routes installed so installed() holds, but it can never
// build. Real builds arrive only through the call-bound owner table below,
// whose callbacks receive the exact OwnerExecutionContext of the replay.
inline bool contextRequiredBuild(const composite_recipe::FeatureNode&,
                                 const std::vector<retained_feature::ReplayValue>&,
                                 retained_feature::ReplayBudget&,
                                 retained_feature::ReplayValue&) noexcept {
    return false;
}
inline bool contextRequiredProof(const composite_recipe::FeatureNode&,
                                 const std::vector<retained_feature::ReplayValue>&,
                                 const retained_feature::ReplayValue&,
                                 retained_feature::ReplayBudget&) noexcept {
    return false;
}
inline bool ownerBuild(const retained_feature::OwnerExecutionContext& context,
                       const composite_recipe::FeatureNode& feature,
                       const std::vector<retained_feature::ReplayValue>& inputs,
                       retained_feature::ReplayBudget& budget,
                       retained_feature::ReplayValue& output) noexcept {
    return context.graph != nullptr && detail::buildDetached(feature, inputs, budget, output);
}
inline bool ownerProve(const retained_feature::OwnerExecutionContext& context,
                       const composite_recipe::FeatureNode& feature,
                       const std::vector<retained_feature::ReplayValue>& inputs,
                       const retained_feature::ReplayValue& output,
                       retained_feature::ReplayBudget& budget) noexcept {
    return context.graph != nullptr && detail::proveFamily(feature, inputs, output, budget);
}
} // namespace owner_detail

//! Closed, call-bound B4 owner execution registry. Constructed per operation
//! as a local; it contains no captured graph, settings, mutable process state
//! or production override. The execution entry's validation pointer is wired
//! to this object's own validation entry in the constructor and the object is
//! immovable afterwards, so ReplayOwnerDetached's pointer-identity rule always
//! binds this exact pair. ProductionRegistry() never contains it.
class OwnerRegistry final {
public:
    OwnerRegistry() noexcept {
        validation_[0] = RegistryEntry(true, true);
        validation_[0].execution.buildDetached = owner_detail::contextRequiredBuild;
        validation_[0].execution.proveFamily = owner_detail::contextRequiredProof;
        validation_[0].execution.verifyFixedPoint = owner_detail::contextRequiredProof;
        execution_[0].key = {FeatureKind, CodecVersion};
        execution_[0].execution.validation = &validation_[0].execution;
        execution_[0].execution.buildDetached = owner_detail::ownerBuild;
        execution_[0].execution.proveFamily = owner_detail::ownerProve;
        execution_[0].execution.verifyFixedPoint = owner_detail::ownerProve;
    }
    OwnerRegistry(const OwnerRegistry&) = delete;
    OwnerRegistry& operator=(const OwnerRegistry&) = delete;
    retained_feature::RegistryView validationView() const noexcept {
        return retained_feature::RegistryView(validation_.data(), validation_.size());
    }
    retained_feature::OwnerExecutionRegistryView executionView() const noexcept {
        return retained_feature::OwnerExecutionRegistryView(execution_.data(), execution_.size());
    }
private:
    std::array<retained_feature::Entry, 1> validation_{};
    std::array<retained_feature::OwnerExecutionEntry, 1> execution_{};
};
} // namespace core3d::draft_faces
