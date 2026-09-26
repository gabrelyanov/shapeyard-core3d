#pragma once

#include "CompositeRecipeDefinition.hxx"
#include <cstddef>
#include <cstdint>
#include <set>

namespace core3d::retained_source {

inline constexpr std::uint8_t ReservedPlanarSplineProfileKind = 0x10;
inline constexpr std::uint8_t BoundedCurvePathKind =
    std::uint8_t(composite_recipe::RecipeKind::BoundedCurvePath);
static_assert(BoundedCurvePathKind == 0x11,
              "D65/D67 frozen bounded-curve source allocation changed");

struct Key final {
    std::uint8_t kind = 0;
    std::uint32_t schema = 0;
    bool operator<(const Key& value) const noexcept {
        return kind < value.kind || (kind == value.kind && schema < value.schema);
    }
    bool operator==(const Key& value) const noexcept {
        return kind == value.kind && schema == value.schema;
    }
};

using CanonicalSource = bool (*)(const composite_recipe::SourceRecipe&) noexcept;
struct Descriptor final {
    Key key;
    composite_recipe::SourceShapeKind shape = composite_recipe::SourceShapeKind::Unknown;
    std::size_t maximumPayloadBytes = 0;
    CanonicalSource canonical = nullptr;
    bool sourceBuilderInstalled = false;
    bool familyProofInstalled = false;
    bool editorRouteInstalled = false;
};

class RegistryView final {
public:
    constexpr RegistryView(const Descriptor* entries, std::size_t count) noexcept
        : entries_(entries), count_(count) {}
    const Descriptor* find(Key key) const noexcept {
        for (std::size_t index = 0; index < count_; ++index)
            if (entries_[index].key == key) return entries_ + index;
        return nullptr;
    }
    bool valid() const noexcept {
        try {
            if (count_ && !entries_) return false;
            std::set<Key> seen;
            for (std::size_t index = 0; index < count_; ++index) {
                const auto& value = entries_[index];
                if (value.key.kind == 0 || value.key.schema == 0
                    || value.shape == composite_recipe::SourceShapeKind::Unknown
                    || value.maximumPayloadBytes == 0
                    || value.maximumPayloadBytes > composite_recipe::MaximumEnvelopeBytes
                    || !value.canonical || !seen.insert(value.key).second) return false;
            }
            return true;
        } catch (...) { return false; }
    }
    std::size_t size() const noexcept { return count_; }
private:
    const Descriptor* entries_ = nullptr;
    std::size_t count_ = 0;
};

const RegistryView& ProductionRegistry() noexcept;

inline bool ExpectedShape(const composite_recipe::SourceRecipe& recipe,
                          const RegistryView& registry,
                          composite_recipe::SourceShapeKind& output) noexcept {
    output = composite_recipe::SourceShapeKind::Unknown;
    if (!registry.valid()) return false;
    const auto* descriptor = registry.find({std::uint8_t(recipe.kind), recipe.schema});
    if (!descriptor) return false;
    output = descriptor->shape;
    return output != composite_recipe::SourceShapeKind::Unknown;
}

} // namespace core3d::retained_source
