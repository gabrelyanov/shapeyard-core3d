#pragma once
#include "RetainedRecipeAdmission.hxx"
#include <map>
#include <string>

namespace core3d::retained_recipe {
#if DEBUG
struct Probe {
    static UUID ID(std::uint8_t seed) {
        UUID value{}; for (std::size_t index = 0; index < value.size(); ++index) value[index] = std::uint8_t(seed + index);
        return value;
    }
    static Digest HashValue(std::uint8_t seed) {
        Digest value{}; for (std::size_t index = 0; index < value.size(); ++index) value[index] = std::uint8_t(seed + index);
        return value;
    }
    static composite_recipe::InputPlacement Placement(double metersPerUnit) {
        composite_recipe::InputPlacement value;
        value.sourceMetersPerUnit = metersPerUnit; value.carrierMetersPerUnit = metersPerUnit;
        return value;
    }
    static composite_recipe::Commitments Commit(std::uint8_t seed,
                                                 const std::vector<std::uint8_t>& recipe) {
        composite_recipe::Commitments value;
        value.geometry = HashValue(seed); value.placement = HashValue(seed + 1);
        value.material = HashValue(seed + 2); value.groups = HashValue(seed + 3);
        composite_recipe::Hash(recipe, value.recipe); return value;
    }
    static std::map<std::string, bool> Inspect(const composite_recipe::Definition& definition) {
        std::vector<std::uint8_t> bytes; composite_recipe::Definition decoded;
        const bool encoded = composite_recipe::Encode(definition, bytes);
        const bool roundTrip = encoded && composite_recipe::Decode(bytes, decoded);
        return {{"bounded", !encoded || bytes.size() <= composite_recipe::MaximumEnvelopeBytes},
                {"structural", composite_recipe::Valid(definition)},
                {"roundTrip", roundTrip && decoded.owner == definition.owner
                    && decoded.outputNode == definition.outputNode
                    && decoded.nodes.size() == definition.nodes.size()}};
    }
};
#endif
} // namespace core3d::retained_recipe
