#pragma once
#include "RetainedRecipeIdentity.hxx"
#include <array>
#include <cmath>
#include <cstdint>
#include <variant>
#include <vector>

namespace core3d::composite_recipe {
using retained_recipe::Digest;
using retained_recipe::IssuanceState;
using retained_recipe::OwnerKey;
using retained_recipe::SourceIdentity;
using retained_recipe::UUID;

inline constexpr std::size_t MaximumNodes = 64;
inline constexpr std::size_t MaximumSourceNodes = 16;
inline constexpr std::size_t MaximumDepth = 16;
inline constexpr std::size_t MaximumEnvelopeBytes = 64 * 1024;
inline constexpr std::size_t MaximumDocumentAggregateBytes = 8 * 1024 * 1024;
inline constexpr std::size_t MaximumFeaturePayloadBytes = 16 * 1024;
inline constexpr int MinimumRecordTag = 13;

enum class RecipeKind : std::uint8_t {
    Profile = 1,
    Enclosure = 2,
    RectangularLoft = 3,
    RetainedBoolean = 4,
    // Reserved wire value only. P1 deliberately does not install a codec,
    // source builder, editor, or family proof for it.
    ReservedPlanarSplineProfile = 0x10,
};
enum class NodeKind : std::uint8_t { Source = 1, Feature = 2 };
enum class SourceShapeKind : std::uint8_t { Unknown = 0, Solid = 1, Face = 2, Wire = 3 };

struct InputPlacement {
    // Row-major authored-source-to-carrier affine transform. Authored recipe
    // numbers remain unchanged; both unit values are explicit.
    std::array<double, 16> matrix{{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}};
    double sourceMetersPerUnit = 0;
    double carrierMetersPerUnit = 0;
};

struct Commitments {
    Digest geometry{}, recipe{}, placement{}, material{}, groups{};
};

struct SourceRecipe {
    RecipeKind kind = RecipeKind::Profile;
    std::uint32_t schema = 0;
    std::vector<std::uint8_t> bytes;
};

struct SourceNode {
    UUID node{};
    std::uint64_t localID = 0;
    SourceIdentity original;
    SourceRecipe recipe;
    InputPlacement inputToCarrier;
    Commitments commitments;
    std::uint32_t shapeSlot = 0;
};

// SYCR/1 and the first P1 SYCR/2 profile retain only forward solids.  Later
// registry owners may add recipe kinds with a different explicit shape kind;
// callers must use this function instead of assuming every future source is a
// solid.  Unknown recipe tags never reach this point because ValidRecipe
// rejects them before a binary shape is accepted.
inline SourceShapeKind ExpectedSourceShapeKind(const SourceRecipe& recipe) noexcept {
    switch (recipe.kind) {
    case RecipeKind::Profile:
    case RecipeKind::Enclosure:
    case RecipeKind::RectangularLoft:
    case RecipeKind::RetainedBoolean:
        return SourceShapeKind::Solid;
    default:
        return SourceShapeKind::Unknown;
    }
}

struct FeatureNode {
    UUID node{}, feature{};
    std::uint64_t localID = 0;
    std::uint32_t kind = 0, codecVersion = 0;
    std::vector<UUID> inputs;
    std::vector<std::uint8_t> parameters;
};

using NodeValue = std::variant<SourceNode, FeatureNode>;
struct Node { NodeValue value; };

struct Definition {
    std::uint32_t schemaVersion = 1;
    OwnerKey owner;
    UUID outputNode{};
    IssuanceState issuance;
    std::vector<Node> nodes;
};

inline Definition MakeV3Definition() noexcept {
    Definition result;
    result.schemaVersion = 3;
    return result;
}

inline NodeKind Kind(const Node& node) noexcept {
    return std::holds_alternative<SourceNode>(node.value) ? NodeKind::Source : NodeKind::Feature;
}
inline const UUID& NodeID(const Node& node) noexcept {
    if (const auto* source = std::get_if<SourceNode>(&node.value)) return source->node;
    return std::get<FeatureNode>(node.value).node;
}
inline std::uint64_t LocalID(const Node& node) noexcept {
    if (const auto* source = std::get_if<SourceNode>(&node.value)) return source->localID;
    return std::get<FeatureNode>(node.value).localID;
}

inline bool ValidPlacement(const InputPlacement& value) noexcept {
    for (double scalar : value.matrix) if (!std::isfinite(scalar)) return false;
    if (!std::isfinite(value.sourceMetersPerUnit) || value.sourceMetersPerUnit <= 0
        || !std::isfinite(value.carrierMetersPerUnit) || value.carrierMetersPerUnit <= 0
        || value.matrix[12] != 0 || value.matrix[13] != 0
        || value.matrix[14] != 0 || value.matrix[15] != 1) return false;
    const auto dot = [&](unsigned a, unsigned b) {
        return value.matrix[a] * value.matrix[b]
            + value.matrix[4 + a] * value.matrix[4 + b]
            + value.matrix[8 + a] * value.matrix[8 + b];
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
        value.matrix[0] * (value.matrix[5] * value.matrix[10] - value.matrix[6] * value.matrix[9])
      - value.matrix[1] * (value.matrix[4] * value.matrix[10] - value.matrix[6] * value.matrix[8])
      + value.matrix[2] * (value.matrix[4] * value.matrix[9] - value.matrix[5] * value.matrix[8]);
    return std::isfinite(determinant) && determinant > 0;
}

inline bool Valid(const Commitments& value) noexcept {
    return retained_recipe::Nonzero(value.geometry)
        && retained_recipe::Nonzero(value.recipe)
        && retained_recipe::Nonzero(value.placement)
        && retained_recipe::Nonzero(value.material)
        && retained_recipe::Nonzero(value.groups);
}
} // namespace core3d::composite_recipe
