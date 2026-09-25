#pragma once

#include "RetainedFeatureReplay.hxx"

namespace core3d::retained_feature {

#if DEBUG
struct RegistryProbe final {
    static constexpr std::uint32_t SyntheticKind = 0xF0000001;
    static constexpr std::uint32_t SyntheticCodec = 1;

    static bool canonical(const composite_recipe::FeatureNode& feature,
                          const std::vector<composite_recipe::Node>& prior) noexcept {
        if (feature.inputs.size() != 1 || feature.parameters.size() != 1
            || feature.parameters[0] == 0) return false;
        return std::find_if(prior.begin(), prior.end(), [&](const composite_recipe::Node& node) {
            return composite_recipe::NodeID(node) == feature.inputs[0];
        }) != prior.end();
    }
    static bool build(const composite_recipe::FeatureNode& feature,
                      const std::vector<ReplayValue>& inputs,
                      ReplayBudget& budget, ReplayValue& output) noexcept {
        if (inputs.size() != 1 || !inputs[0].valid() || !budget.consume(1, 1, inputs[0].detachedShape.size())) return false;
        // The synthetic descriptor deliberately changes retained parameters
        // while leaving its real OCAF BRep fixed. This isolates transaction,
        // currentness and persistence behavior without admitting geometry.
        output.shape = ShapeKind::Solid; output.detachedShape = inputs[0].detachedShape;
        return composite_recipe::Hash(output.detachedShape, output.geometry)
            && composite_recipe::Hash(feature.parameters, output.familyProof);
    }
    static bool prove(const composite_recipe::FeatureNode& feature,
                      const std::vector<ReplayValue>& inputs,
                      const ReplayValue& output, ReplayBudget& budget) noexcept {
        return budget.consume(1, 0, 0) && inputs.size() == 1 && output.valid()
            && output.detachedShape == inputs[0].detachedShape
            && feature.parameters.size() == 1 && feature.parameters[0] != 0;
    }
    static bool fixed(const composite_recipe::FeatureNode& feature,
                      const std::vector<ReplayValue>& inputs,
                      const ReplayValue& output, ReplayBudget& budget) noexcept {
        ReplayValue rebuilt;
        return build(feature, inputs, budget, rebuilt)
            && rebuilt.detachedShape == output.detachedShape
            && rebuilt.geometry == output.geometry && rebuilt.familyProof == output.familyProof;
    }
    static const RegistryView& registry() noexcept {
        static const std::array<Entry, 1> entries{{
            {{{SyntheticKind, SyntheticCodec}, 3, 1,
              {ShapeKind::Solid, ShapeKind::Solid, ShapeKind::Solid, ShapeKind::Solid},
              1, ShapeKind::Solid, canonical},
             {build, prove, fixed, true, true}},
        }};
        static const RegistryView value(entries.data(), entries.size());
        return value;
    }
};
#endif

} // namespace core3d::retained_feature
