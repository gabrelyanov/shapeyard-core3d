#pragma once

// Exact persisted construction authority; detached geometry derives a
// gp_Trsf from these values without rewriting the saved representation.
#include <gp_Trsf.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Vec.hxx>
#include <array>
#include <algorithm>
#include <cmath>

namespace core3d::profile {
struct ConstructionFrame {
    // Translation in document units, proper rotation quaternion, signed scale.
    std::array<double, 8> values{0, 0, 0, 0, 0, 0, 1, 1};
    bool IsEqual(const ConstructionFrame& other) const noexcept {
        return values == other.values;
    }
    bool operator==(const ConstructionFrame& other) const noexcept { return IsEqual(other); }
    bool operator!=(const ConstructionFrame& other) const noexcept { return !IsEqual(other); }
    bool IsValid() const noexcept {
        for (double value : values) if (!std::isfinite(value)) return false;
        for (std::size_t axis = 0; axis < 3; ++axis)
            if (std::abs(values[axis]) > 1e6) return false;
        if (std::abs(values[7]) < 1e-6 || std::abs(values[7]) > 1e6) return false;
        const double norm = std::hypot(std::hypot(values[3], values[4]),
                                      std::hypot(values[5], values[6]));
        return std::isfinite(norm) && std::abs(norm - 1.0) <= 1e-12;
    }
    bool Transform(gp_Trsf& result) const noexcept {
        result = gp_Trsf();
        if (!IsValid()) return false;
        try {
            gp_Trsf transform;
            transform.SetRotationPart(gp_Quaternion(values[3], values[4], values[5], values[6]));
            transform.SetScaleFactor(values[7]);
            transform.SetTranslationPart(gp_Vec(values[0], values[1], values[2]));
            for (int row = 1; row <= 3; ++row)
                for (int column = 1; column <= 4; ++column)
                    if (!std::isfinite(transform.Value(row, column))) return false;
            result = transform;
            return true;
        } catch (...) { return false; }
    }
    double AbsoluteVolumeScale() const noexcept {
        if (!IsValid()) return 0;
        const double scale = std::abs(values[7]);
        return scale * scale * scale;
    }
    // Only capture of a NEW frame canonicalizes quaternion sign. Existing
    // record values are never normalized or canonicalized during reads.
    static bool Capture(const gp_Trsf& transform, ConstructionFrame& result) noexcept {
        result = {};
        try {
            const auto rotation = transform.GetRotation();
            const auto translation = transform.TranslationPart();
            ConstructionFrame frame;
            frame.values = {translation.X(), translation.Y(), translation.Z(),
                            rotation.X(), rotation.Y(), rotation.Z(), rotation.W(),
                            transform.ScaleFactor()};
            bool negate = false;
            for (const int index : {6, 3, 4, 5}) {
                if (frame.values[index] == 0) continue;
                negate = frame.values[index] < 0; break;
            }
            if (negate) for (int index = 3; index <= 6; ++index) frame.values[index] = -frame.values[index];
            gp_Trsf reconstructed;
            if (!frame.Transform(reconstructed)) return false;
            for (int row = 1; row <= 3; ++row) {
                for (int column = 1; column <= 4; ++column) {
                    const double expected = transform.Value(row, column);
                    const double actual = reconstructed.Value(row, column);
                    if (!std::isfinite(expected) || std::abs(actual - expected)
                        > 1e-12 * std::max({1.0, std::abs(expected), std::abs(actual)})) return false;
                }
            }
            result = frame; return true;
        } catch (...) { return false; }
    }
};
} // namespace core3d::profile
