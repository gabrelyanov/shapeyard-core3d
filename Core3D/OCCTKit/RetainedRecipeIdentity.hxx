#pragma once
// Persisted logical identity values only. These values are evidence and
// locators; they never carry a native lease, document handle, or mutation right.
#include "RetainedSolidEnvelope.hxx"
#include <algorithm>
#include <cstdint>
#include <vector>

namespace core3d::retained_recipe {
using UUID = retained_solid::UUID;
using Digest = retained_solid::Digest;

inline bool Nonzero(const UUID& value) noexcept { return retained_solid::Nonzero(value); }
inline bool Nonzero(const Digest& value) noexcept {
    return std::any_of(value.begin(), value.end(), [](std::uint8_t byte) { return byte != 0; });
}

struct OwnerKey {
    UUID document{}, entity{}, definition{};
    bool operator==(const OwnerKey& other) const noexcept {
        return document == other.document && entity == other.entity && definition == other.definition;
    }
};

struct SourceIdentity {
    UUID document{}, entity{}, definition{}, sourceFeature{};
    bool operator==(const SourceIdentity& other) const noexcept {
        return document == other.document && entity == other.entity
            && definition == other.definition && sourceFeature == other.sourceFeature;
    }
};

struct RecipeLocator {
    OwnerKey owner;
    UUID node{}, sourceFeature{};
    bool operator==(const RecipeLocator& other) const noexcept {
        return owner == other.owner && node == other.node && sourceFeature == other.sourceFeature;
    }
};

struct IssuanceState {
    // Local IDs are monotonically issued evidence. UUIDs remain the durable
    // identity; a removed local ID is never made available for reuse.
    std::uint64_t nextLocalID = 1;
    std::vector<std::uint64_t> retiredLocalIDs;
};

inline bool Valid(const OwnerKey& key) noexcept {
    return Nonzero(key.document) && Nonzero(key.entity) && Nonzero(key.definition);
}
inline bool Valid(const SourceIdentity& key) noexcept {
    return Nonzero(key.document) && Nonzero(key.entity)
        && Nonzero(key.definition) && Nonzero(key.sourceFeature);
}
inline bool Valid(const RecipeLocator& locator) noexcept {
    return Valid(locator.owner) && Nonzero(locator.node) && Nonzero(locator.sourceFeature);
}
inline bool Valid(const IssuanceState& state) noexcept {
    if (state.nextLocalID == 0) return false;
    std::uint64_t previous = 0;
    for (std::uint64_t value : state.retiredLocalIDs) {
        if (value == 0 || value >= state.nextLocalID || value <= previous) return false;
        previous = value;
    }
    return true;
}
} // namespace core3d::retained_recipe
