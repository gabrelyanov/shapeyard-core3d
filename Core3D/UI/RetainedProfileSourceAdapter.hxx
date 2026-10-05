#pragma once

// Pure value translation for complete retained-profile submissions. Document,
// selection, command and history authority remain in Core3DViewer and the
// ordinary/R2 owners. Shell values are capture provenance in B2c: callers must
// carry the exact ordered list and units; authoring them belongs to B2d.
#include "../OCCTKit/ProfilePersistence.hxx"
#include <cstring>

namespace core3d::retained_profile_source_adapter {

inline std::uint64_t Bits(double value) noexcept {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

inline bool SameBits(const std::vector<double>& left,
                     const std::vector<double>& right) noexcept {
    if (left.size() != right.size()) return false;
    for (std::size_t i = 0; i < left.size(); ++i)
        if (Bits(left[i]) != Bits(right[i])) return false;
    return true;
}

inline bool SameFrameBits(const profile::ConstructionFrame& left,
                          const profile::ConstructionFrame& right) noexcept {
    for (std::size_t i = 0; i < left.values.size(); ++i)
        if (Bits(left.values[i]) != Bits(right.values[i])) return false;
    return true;
}

inline bool SameFrozenShellBits(const profile::Parameters& captured,
                                const profile::Parameters& requested) noexcept {
    if (Bits(captured.metersPerUnit) != Bits(requested.metersPerUnit)
        || captured.shells.size() != requested.shells.size()) return false;
    for (std::size_t i = 0; i < captured.shells.size(); ++i) {
        const auto& left = captured.shells[i];
        const auto& right = requested.shells[i];
        if (Bits(left.thickness) != Bits(right.thickness)
            || Bits(left.metersPerLocalUnit) != Bits(right.metersPerLocalUnit)
            || !SameFrameBits(left.frame, right.frame)
            || left.openings != right.openings) return false;
    }
    return true;
}

inline bool TranslateComplete(const profile::Parameters& captured,
                              const profile::Parameters& requested,
                              profile::Parameters& translated) noexcept {
    translated = {};
    try {
        if (!SameFrozenShellBits(captured, requested)) return false;
        std::vector<double> canonical;
        if (!profile::Encode(requested, canonical)) return false;
        translated = requested;
        return true;
    } catch (...) {
        translated = {};
        return false;
    }
}

template <class DetachedGeometry>
inline void PopulateDetached(const profile::Parameters& parameters,
                             DetachedGeometry& geometry) {
    static_cast<ProfileDefinition&>(geometry) = parameters.definition;
    geometry.constructionFrame = parameters.constructionFrame;
    geometry.shells = parameters.shells;
    geometry.shellMetersPerUnit = parameters.metersPerUnit;
}

} // namespace core3d::retained_profile_source_adapter
