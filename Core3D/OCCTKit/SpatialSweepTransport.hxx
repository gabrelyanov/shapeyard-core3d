#pragma once
#include "SpatialSweepProof.hxx"
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace core3d::spatial_sweep {
using Vector3 = std::array<double, 3>;

inline Vector3 Add(const Vector3& a, const Vector3& b) noexcept {
    return {{a[0] + b[0], a[1] + b[1], a[2] + b[2]}};
}
inline Vector3 Subtract(const Vector3& a, const Vector3& b) noexcept {
    return {{a[0] - b[0], a[1] - b[1], a[2] - b[2]}};
}
inline Vector3 Scale(const Vector3& a, double value) noexcept {
    return {{a[0] * value, a[1] * value, a[2] * value}};
}
inline double Dot(const Vector3& a, const Vector3& b) noexcept {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline Vector3 Cross(const Vector3& a, const Vector3& b) noexcept {
    return {{a[1] * b[2] - a[2] * b[1],
             a[2] * b[0] - a[0] * b[2],
             a[0] * b[1] - a[1] * b[0]}};
}
inline double Norm(const Vector3& value) noexcept { return std::sqrt(Dot(value, value)); }
inline bool Normalize(const Vector3& value, Vector3& output) noexcept {
    const double norm = Norm(value);
    if (!Finite(norm) || norm <= 0) { output = {}; return false; }
    output = Scale(value, 1 / norm);
    return true;
}

struct Frame {
    Vector3 tangent{{1, 0, 0}};
    Vector3 e1{{0, 1, 0}};
    Vector3 e2{{0, 0, 1}};
};

inline bool ValidFrame(const Frame& value, double tolerance = 1e-10) noexcept {
    const Vector3 cross = Cross(value.e1, value.e2);
    return std::abs(Norm(value.tangent) - 1) <= tolerance
        && std::abs(Norm(value.e1) - 1) <= tolerance
        && std::abs(Norm(value.e2) - 1) <= tolerance
        && std::abs(Dot(value.tangent, value.e1)) <= tolerance
        && std::abs(Dot(value.tangent, value.e2)) <= tolerance
        && std::abs(Dot(value.e1, value.e2)) <= tolerance
        && Dot(cross, value.tangent) >= 1 - tolerance;
}

inline bool SeedFrame(const Vector3& tangentValue, const Vector3& authoredSeed,
                      Frame& output) noexcept {
    Vector3 tangent{};
    if (!Normalize(tangentValue, tangent)) return false;
    const double seedNorm = Norm(authoredSeed);
    const Vector3 projected = Subtract(authoredSeed, Scale(tangent, Dot(authoredSeed, tangent)));
    if (!Finite(seedNorm) || seedNorm < 0.5 || seedNorm > 2
        || Norm(projected) < 0.25 * seedNorm || !Normalize(projected, output.e1)) return false;
    output.tangent = tangent;
    output.e2 = Cross(tangent, output.e1);
    return ValidFrame(output);
}

// Minimal SO(3) rotation carrying one tangent to the next. Applying it to the
// complete transverse frame is the deterministic discrete realization of
// Bishop parallel transport; there is no curvature-normal sign selection.
inline bool BishopStep(const Frame& before, const Vector3& nextTangentValue,
                       Frame& output) noexcept {
    if (!ValidFrame(before)) return false;
    Vector3 nextTangent{};
    if (!Normalize(nextTangentValue, nextTangent)) return false;
    const Vector3 axis = Cross(before.tangent, nextTangent);
    const double sine2 = Dot(axis, axis);
    const double cosine = Dot(before.tangent, nextTangent);
    if (!Finite(sine2) || !Finite(cosine) || cosine <= -1 + 1e-14) return false;
    const auto rotate = [&](const Vector3& vector) {
        if (sine2 <= 1e-28) return vector;
        return Add(Add(vector, Cross(axis, vector)),
                   Scale(Cross(axis, Cross(axis, vector)), (1 - cosine) / sine2));
    };
    output.tangent = nextTangent;
    Vector3 first = rotate(before.e1);
    first = Subtract(first, Scale(nextTangent, Dot(first, nextTangent)));
    if (!Normalize(first, output.e1)) return false;
    output.e2 = Cross(nextTangent, output.e1);
    return ValidFrame(output, 5e-10);
}

inline bool ApplySpin(const Frame& bishop, double radians, Frame& output) noexcept {
    if (!ValidFrame(bishop) || !Finite(radians)) return false;
    const double cosine = std::cos(radians), sine = std::sin(radians);
    output.tangent = bishop.tangent;
    output.e1 = Add(Scale(bishop.e1, cosine), Scale(bishop.e2, sine));
    output.e2 = Add(Scale(bishop.e1, -sine), Scale(bishop.e2, cosine));
    return ValidFrame(output, 5e-10);
}

struct TransportStation {
    double normalizedArcLength = 0;
    Frame bishop;
};

struct PreparedTransport {
    std::vector<TransportStation> stations;
    double phase = 0;
    double totalSpin = 0;
};

inline bool Valid(const PreparedTransport& value) noexcept {
    if (value.stations.size() < 2 || value.stations.front().normalizedArcLength != 0
        || value.stations.back().normalizedArcLength != 1
        || !Finite(value.phase) || !Finite(value.totalSpin)) return false;
    for (std::size_t index = 0; index < value.stations.size(); ++index)
        if (!ValidFrame(value.stations[index].bishop)
            || (index != 0 && !(value.stations[index - 1].normalizedArcLength
                < value.stations[index].normalizedArcLength))) return false;
    return true;
}

// Query order cannot mutate this immutable table. Interpolation uses a fresh
// minimal rotation from the lower certified station and then authored spin.
inline bool Evaluate(const PreparedTransport& value, double q,
                     const Vector3& exactTangent, Frame& output) noexcept {
    output = {};
    if (!Valid(value) || !Finite(q) || q < 0 || q > 1) return false;
    const auto upper = std::upper_bound(value.stations.begin(), value.stations.end(), q,
        [](double target, const TransportStation& station) {
            return target < station.normalizedArcLength;
        });
    const TransportStation& base = upper == value.stations.begin()
        ? value.stations.front() : *(upper - 1);
    Frame bishop{};
    if (!BishopStep(base.bishop, exactTangent, bishop)) return false;
    return ApplySpin(bishop, value.phase + value.totalSpin * q, output);
}

inline bool Holonomy(const Frame& start, const Frame& finish, double& radians) noexcept {
    radians = 0;
    if (!ValidFrame(start) || !ValidFrame(finish)
        || Dot(start.tangent, finish.tangent) < 1 - 1e-10) return false;
    radians = std::atan2(Dot(start.tangent, Cross(start.e1, finish.e1)),
                         Dot(start.e1, finish.e1));
    return Finite(radians);
}

inline bool SelectHolonomyLift(double wrapped, double reference, bool creation,
                               double frameTolerance, double& lifted,
                               std::int32_t& lift) noexcept {
    lifted = 0; lift = 0;
    if (!Finite(wrapped) || !Finite(reference) || !Finite(frameTolerance)
        || frameTolerance <= 0) return false;
    const double target = creation ? 0 : reference;
    const double turns = (target - wrapped) / (2 * Pi);
    const double nearest = std::round(turns);
    if (nearest < double(std::numeric_limits<std::int32_t>::min())
        || nearest > double(std::numeric_limits<std::int32_t>::max())
        || std::abs(std::abs(turns - std::floor(turns)) - 0.5)
            <= 16 * frameTolerance / (2 * Pi)) return false;
    lift = std::int32_t(nearest);
    lifted = wrapped + 2 * Pi * double(lift);
    return Finite(lifted) && std::abs(lifted) <= 8 * Pi;
}
} // namespace core3d::spatial_sweep
