#pragma once

#include "CompositeRecipeDefinition.hxx"
#include <array>
#include <cstddef>
#include <cstdint>
#include <set>

namespace core3d::retained_feature {

// D67 reserves these persisted kind values without installing descriptors.
inline constexpr std::uint32_t ReservedD67FeatureKind1001 = 0x00001001;
inline constexpr std::uint32_t ReservedD67FeatureKind3001 = 0x00003001;
inline constexpr std::uint32_t ExistingChamferAdapterKind = 0x00002001;
inline constexpr std::uint32_t ExistingConstantFilletAdapterKind = 0x00002002;
inline constexpr std::uint32_t VariableRadiusFilletKind = 0x00002003;
inline constexpr std::uint32_t DraftFacesKind = 0x00002004;
// D65 central allocation; reserved/disabled until the C4 integration.
inline constexpr std::uint32_t SplineProfileRevolveKind = 0x00003004;

struct Key final {
    std::uint32_t kind = 0;
    std::uint32_t codecVersion = 0;
    bool operator<(const Key& value) const noexcept {
        return kind < value.kind || (kind == value.kind && codecVersion < value.codecVersion);
    }
    bool operator==(const Key& value) const noexcept {
        return kind == value.kind && codecVersion == value.codecVersion;
    }
};

enum class ShapeKind : std::uint8_t { Solid = 1, Face = 2, Wire = 3 };
struct ReplayBudget;
struct ReplayValue;

using CanonicalPayload = bool (*)(const composite_recipe::FeatureNode&,
                                  const std::vector<composite_recipe::Node>&) noexcept;
using BuildDetached = bool (*)(const composite_recipe::FeatureNode&,
                               const std::vector<ReplayValue>&,
                               ReplayBudget&, ReplayValue&) noexcept;
using ProveFamily = bool (*)(const composite_recipe::FeatureNode&,
                             const std::vector<ReplayValue>&,
                             const ReplayValue&, ReplayBudget&) noexcept;
using VerifyFixedPoint = bool (*)(const composite_recipe::FeatureNode&,
                                  const std::vector<ReplayValue>&,
                                  const ReplayValue&, ReplayBudget&) noexcept;

struct CodecDescriptor final {
    Key key;
    std::uint32_t graphMajor = 0;
    std::size_t maximumPayloadBytes = 0;
    std::array<ShapeKind, 4> orderedInputs{};
    std::uint8_t inputCount = 0;
    ShapeKind output = ShapeKind::Solid;
    CanonicalPayload canonicalPayload = nullptr;
};

struct ExecutionDescriptor final {
    BuildDetached buildDetached = nullptr;
    ProveFamily proveFamily = nullptr;
    VerifyFixedPoint verifyFixedPoint = nullptr;
    bool editorRouteInstalled = false;
    bool dependencyRouteInstalled = false;
    bool installed() const noexcept {
        return buildDetached && proveFamily && verifyFixedPoint
            && editorRouteInstalled && dependencyRouteInstalled;
    }
};

struct Entry final { CodecDescriptor codec; ExecutionDescriptor execution; };

class RegistryView final {
public:
    constexpr RegistryView(const Entry* entries, std::size_t entryCount,
                           const Key* tombstones = nullptr,
                           std::size_t tombstoneCount = 0) noexcept
        : entries_(entries), entryCount_(entryCount),
          tombstones_(tombstones), tombstoneCount_(tombstoneCount) {}

    const Entry* find(Key key) const noexcept {
        for (std::size_t index = 0; index < entryCount_; ++index)
            if (entries_[index].codec.key == key) return entries_ + index;
        return nullptr;
    }
    bool tombstoned(Key key) const noexcept {
        for (std::size_t index = 0; index < tombstoneCount_; ++index)
            if (tombstones_[index] == key) return true;
        return false;
    }
    bool valid() const noexcept {
        try {
            if ((entryCount_ && !entries_) || (tombstoneCount_ && !tombstones_)) return false;
            std::set<Key> seen;
            for (std::size_t index = 0; index < tombstoneCount_; ++index)
                if (tombstones_[index].kind == 0 || tombstones_[index].codecVersion == 0
                    || !seen.insert(tombstones_[index]).second) return false;
            for (std::size_t index = 0; index < entryCount_; ++index) {
                const auto& value = entries_[index].codec;
                if (value.key.kind == 0 || value.key.codecVersion == 0
                    || value.graphMajor != 3 || value.maximumPayloadBytes == 0
                    || value.maximumPayloadBytes > composite_recipe::MaximumFeaturePayloadBytes
                    || value.inputCount == 0 || value.inputCount > value.orderedInputs.size()
                    || !value.canonicalPayload || tombstoned(value.key)
                    || !seen.insert(value.key).second) return false;
            }
            return true;
        } catch (...) { return false; }
    }
    std::size_t size() const noexcept { return entryCount_; }

private:
    const Entry* entries_ = nullptr;
    std::size_t entryCount_ = 0;
    const Key* tombstones_ = nullptr;
    std::size_t tombstoneCount_ = 0;
};

// Defined by CompositeRecipeCodec.hxx after the immutable payload codecs are
// visible. It is a closed native table, never a provider or document value.
const RegistryView& ProductionRegistry() noexcept;

#if DEBUG
// Process-local test seam. The binary driver observes this only while an
// internal fixture scope is alive; ordinary production reads always use the
// closed production table and therefore refuse synthetic feature keys.
inline const RegistryView*& DebugRegistryOverride() noexcept {
    static thread_local const RegistryView* value = nullptr;
    return value;
}

class DebugRegistryScope final {
public:
    explicit DebugRegistryScope(const RegistryView& registry) noexcept
        : previous_(DebugRegistryOverride()) { DebugRegistryOverride() = &registry; }
    ~DebugRegistryScope() { DebugRegistryOverride() = previous_; }
    DebugRegistryScope(const DebugRegistryScope&) = delete;
    DebugRegistryScope& operator=(const DebugRegistryScope&) = delete;
private:
    const RegistryView* previous_ = nullptr;
};
#endif

inline const RegistryView& EffectiveRegistry() noexcept {
#if DEBUG
    if (DebugRegistryOverride()) return *DebugRegistryOverride();
#endif
    return ProductionRegistry();
}

} // namespace core3d::retained_feature
