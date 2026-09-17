#pragma once

// External implementation candidate, not included by production.
// This transforms already captured recipe VALUES only. It does not inspect a
// retained solid, prove correspondence, bind an owner, or authorize a command.
#include "RetainedSolidAttribute.hxx"
#include <array>
#include <cstdint>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

namespace core3d::saved_cut_source_values {
enum class Component : std::uint8_t { U, V };
struct CoordinateChange {
    // Ordinals are bound to the captured recipe; they are not persistent IDs.
    std::uint32_t ordinal = 0;
    Component component = Component::U;
    double value = 0;
};
struct PolygonPatch {
    std::optional<double> depth;
    std::vector<CoordinateChange> coordinates;
};
struct EnclosurePatch {
    // Width, depth, height, wall, floor, corner radius, in recipe units.
    std::array<std::optional<double>, 6> dimensions;
};
// The authored centre stays fixed; source edits replay every bore verbatim.
struct CirclePatch {
    std::optional<double> outerRadius;
    std::optional<double> innerRadius;
    std::optional<double> depth;
};
using Patch = std::variant<PolygonPatch, EnclosurePatch, CirclePatch>;
struct Result {
    retained_solid::Envelope envelope;
    bool changed = false;
};

inline bool PositiveSourceFrame(const std::optional<profile::ConstructionFrame>& frame) noexcept {
    return !frame || (frame->IsValid() && frame->values[7] > 0);
}

inline std::optional<Result> Apply(const retained_solid::Envelope& original,
    const Patch& patch) noexcept {
    try {
        if (!retained_solid::Valid(original)) return {};
        // Bounded by the existing source schema before any candidate allocation.
        std::array<bool, profile::MaximumScalars> touched{};
        auto candidate = original;
        bool changed = false;
        const auto assign = [&](std::size_t offset, double value) -> bool {
            if (!std::isfinite(value) || offset >= candidate.sourceValues.size()
                || offset >= touched.size() || touched[offset]) return false;
            touched[offset] = true;
            // Same numeric value is a no-op, including +0 submitted for a saved
            // -0. Preserve the existing bits instead of canonicalizing them.
            if (candidate.sourceValues[offset] != value) {
                candidate.sourceValues[offset] = value;
                changed = true;
            }
            return true;
        };
        if (const auto* polygon = std::get_if<PolygonPatch>(&patch)) {
            profile::Parameters decoded;
            if (original.sourceFamily != 1 || original.sourceSchema > 2
                || !profile::Decode(original.sourceValues, decoded)
                || !PositiveSourceFrame(decoded.constructionFrame)) return {};
            const auto& definition = decoded.definition;
            if (definition.revolve || definition.circle || definition.curves
                || !definition.holes.empty()
                || polygon->coordinates.size() > definition.points.size() * 2) return {};
            if (polygon->depth && !assign(1, *polygon->depth)) return {};
            for (const auto& edit : polygon->coordinates) {
                if (edit.ordinal >= definition.points.size()) return {};
                std::size_t component;
                switch (edit.component) {
                    case Component::U: component = 0; break;
                    case Component::V: component = 1; break;
                    default: return {};
                }
                if (!assign(7 + std::size_t(edit.ordinal) * 2 + component, edit.value)) return {};
            }
        } else if (const auto* enclosurePatch = std::get_if<EnclosurePatch>(&patch)) {
            enclosure::Parameters decoded;
            if (original.sourceFamily != 2
                || !enclosure::Decode(int(original.sourceSchema), original.sourceValues, decoded)
                || !PositiveSourceFrame(decoded.definition.constructionFrame)) return {};
            for (std::size_t field = 0; field < enclosurePatch->dimensions.size(); ++field)
                if (enclosurePatch->dimensions[field]
                    && !assign(2 + field, *enclosurePatch->dimensions[field])) return {};
        } else if (const auto* circlePatch = std::get_if<CirclePatch>(&patch)) {
            profile::Parameters decoded;
            if (original.sourceFamily != 1 || original.sourceSchema > 2
                || !profile::Decode(original.sourceValues, decoded)
                || !PositiveSourceFrame(decoded.constructionFrame)) return {};
            const auto& definition = decoded.definition;
            if (!definition.circle || definition.revolve || definition.curves
                || !definition.points.empty() || !definition.holes.empty()) return {};
            if (circlePatch->outerRadius && !assign(9, *circlePatch->outerRadius)) return {};
            if (circlePatch->innerRadius && !assign(10, *circlePatch->innerRadius)) return {};
            if (circlePatch->depth && !assign(1, *circlePatch->depth)) return {};
        } else return {};

        // Validate the complete multi-field result once. An intermediate width
        // or radius may be invalid even when the submitted combination is valid.
        // Decode/validation never replaces the original raw representation.
        if (!retained_solid::Valid(candidate)) return {};
        for (std::size_t i = 0; i < original.sourceValues.size(); ++i)
            if (!touched[i] && retained_solid::Bits(original.sourceValues[i])
                != retained_solid::Bits(candidate.sourceValues[i])) return {};
        // IDs, schema, units, source frame and tool remain copied from original.
        // Geometry, physical conversion and correspondence are separate gates.
        return Result{std::move(candidate), changed};
    } catch (...) { return {}; }
}
} // namespace core3d::saved_cut_source_values
