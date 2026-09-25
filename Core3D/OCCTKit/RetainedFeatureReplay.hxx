#pragma once

#include "CompositeRecipeCodec.hxx"
#include "RetainedFeatureRegistry.hxx"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace core3d::retained_feature {

struct ReplayBudget final {
    std::size_t workRemaining = composite_recipe::MaximumNodes;
    std::size_t identitiesRemaining = composite_recipe::MaximumNodes * 4;
    std::size_t topologyRemaining = composite_recipe::MaximumDocumentAggregateBytes;
    std::size_t depthRemaining = composite_recipe::MaximumDepth;
    bool exhausted = false;
    bool consume(std::size_t work, std::size_t identities,
                 std::size_t topology, std::size_t depth = 0) noexcept {
        if (work > workRemaining || identities > identitiesRemaining
            || topology > topologyRemaining || depth > depthRemaining) {
            exhausted = true; return false;
        }
        workRemaining -= work; identitiesRemaining -= identities;
        topologyRemaining -= topology; depthRemaining -= depth;
        return true;
    }
};

struct ReplayValue final {
    ShapeKind shape = ShapeKind::Solid;
    std::vector<std::uint8_t> detachedShape;
    retained_recipe::Digest geometry{}, familyProof{};
    bool valid() const noexcept {
        return !detachedShape.empty() && retained_recipe::Nonzero(geometry)
            && retained_recipe::Nonzero(familyProof);
    }
};

struct SourceValue final {
    retained_recipe::UUID node{};
    ReplayValue value;
};

struct NodeObservation final {
    retained_recipe::UUID node{}, feature{};
    Key key;
    std::vector<retained_recipe::UUID> inputs;
    retained_recipe::Digest payload{}, output{}, proof{};
};

enum class ReplayRefusal : std::uint8_t {
    None = 0, MalformedGraph, UnknownCodec, ExecutionNotInstalled,
    MissingSource, ShapeMismatch, BudgetExceeded, BuildFailed,
    FamilyProofFailed, FixedPointFailed, NonCanonicalOutput,
};

struct ReplayResult final {
    ReplayRefusal refusal = ReplayRefusal::MalformedGraph;
    std::string reason;
    ReplayValue output;
    std::vector<NodeObservation> observations;
    bool replayed() const noexcept { return refusal == ReplayRefusal::None; }
};

inline ReplayResult ReplayDetached(const composite_recipe::Definition& definition,
                                   const RegistryView& registry,
                                   const retained_source::RegistryView& sources,
                                   const std::vector<SourceValue>& sourceValues,
                                   ReplayBudget& budget) noexcept {
    ReplayResult result;
    try {
        const auto validity = composite_recipe::ValidateV3(definition, registry, sources);
        if (!validity.valid()) { result.reason = validity.reason; return result; }
        std::map<retained_recipe::UUID, ReplayValue> values;
        for (const SourceValue& source : sourceValues)
            if (!retained_recipe::Nonzero(source.node) || !source.value.valid()
                || !values.emplace(source.node, source.value).second) {
                result.refusal = ReplayRefusal::MissingSource; result.reason = "invalid-source-values"; return result;
            }
        for (const auto& node : definition.nodes) {
            if (!budget.consume(1, 1, 0)) {
                result.refusal = ReplayRefusal::BudgetExceeded; result.reason = "replay-budget"; return result;
            }
            if (const auto* source = std::get_if<composite_recipe::SourceNode>(&node.value)) {
                const auto found = values.find(source->node);
                if (found == values.end()) {
                    result.refusal = ReplayRefusal::MissingSource; result.reason = "missing-source"; return result;
                }
                const auto* sourceCodec = sources.find({std::uint8_t(source->recipe.kind), source->recipe.schema});
                if (!sourceCodec || found->second.shape != ShapeKind(std::uint8_t(sourceCodec->shape))) {
                    result.refusal = ReplayRefusal::ShapeMismatch; result.reason = "source-shape-kind"; return result;
                }
                continue;
            }
            const auto& feature = std::get<composite_recipe::FeatureNode>(node.value);
            const auto* entry = registry.find({feature.kind, feature.codecVersion});
            if (!entry) { result.refusal = ReplayRefusal::UnknownCodec; result.reason = "unknown-feature-codec"; return result; }
            if (!entry->execution.installed()) {
                result.refusal = ReplayRefusal::ExecutionNotInstalled; result.reason = "feature-execution-not-installed"; return result;
            }
            std::vector<ReplayValue> inputs; inputs.reserve(feature.inputs.size());
            for (std::size_t index = 0; index < feature.inputs.size(); ++index) {
                const auto found = values.find(feature.inputs[index]);
                if (found == values.end()) {
                    result.refusal = ReplayRefusal::MissingSource; result.reason = "missing-feature-input"; return result;
                }
                if (found->second.shape != entry->codec.orderedInputs[index]) {
                    result.refusal = ReplayRefusal::ShapeMismatch; result.reason = "feature-input-shape"; return result;
                }
                inputs.push_back(found->second);
            }
            ReplayValue output; output.shape = entry->codec.output;
            if (!entry->execution.buildDetached(feature, inputs, budget, output) || !output.valid()) {
                result.refusal = budget.exhausted ? ReplayRefusal::BudgetExceeded : ReplayRefusal::BuildFailed;
                result.reason = budget.exhausted ? "replay-budget" : "detached-build"; return result;
            }
            if (!entry->execution.proveFamily(feature, inputs, output, budget)) {
                result.refusal = budget.exhausted ? ReplayRefusal::BudgetExceeded : ReplayRefusal::FamilyProofFailed;
                result.reason = budget.exhausted ? "replay-budget" : "family-proof"; return result;
            }
            if (!entry->execution.verifyFixedPoint(feature, inputs, output, budget)) {
                result.refusal = budget.exhausted ? ReplayRefusal::BudgetExceeded : ReplayRefusal::FixedPointFailed;
                result.reason = budget.exhausted ? "replay-budget" : "fixed-point"; return result;
            }
            retained_recipe::Digest payload{};
            if (!composite_recipe::Hash(feature.parameters, payload)) {
                result.refusal = ReplayRefusal::NonCanonicalOutput; result.reason = "payload-commitment"; return result;
            }
            result.observations.push_back({feature.node, feature.feature,
                {feature.kind, feature.codecVersion}, feature.inputs,
                payload, output.geometry, output.familyProof});
            values.emplace(feature.node, std::move(output));
        }
        const auto found = values.find(definition.outputNode);
        if (found == values.end()) { result.reason = "missing-output"; return result; }
        result.output = found->second; result.refusal = ReplayRefusal::None;
        result.reason = "detached-replay-proven"; return result;
    } catch (...) { result = {}; result.reason = "replay-exception"; return result; }
}

} // namespace core3d::retained_feature
