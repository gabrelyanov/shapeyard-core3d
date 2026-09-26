#pragma once

// C1 slice 1: pure persisted-value contract. This header deliberately has no
// OCAF, OCCT geometry, UI, or planner dependency. A caller must validate the
// complete value before allocating, building, or publishing anything.
#include "RetainedRecipeIdentity.hxx"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <vector>

namespace core3d::bounded_curve {
using UUID = retained_recipe::UUID;

inline constexpr std::size_t MaximumControlPoints = 128;
inline constexpr std::uint8_t MinimumDegree = 1;
inline constexpr std::uint8_t MaximumDegree = 7;
inline constexpr std::size_t MaximumDistinctKnotEntries = 128;
inline constexpr std::size_t MaximumExpandedKnotEntries = MaximumControlPoints + MaximumDegree + 1; // 136
inline constexpr double CoordinateLimit = 1'000'000.0;
inline constexpr double MinimumWeight = 1e-6;
inline constexpr double MaximumWeight = 1e6;
inline constexpr std::uint32_t Schema = 1;

enum class Domain : std::uint8_t { Sketch2D = 1, Path3D = 2 };
enum class Handedness : std::uint8_t { Right = 1 };
enum class Refusal : std::uint8_t {
    None = 0, BadSchema, BadDomain, BadFrame, FrameFlip, BadDegree,
    BadControlPointCount, DuplicateControlPointID, BadCoordinate,
    BadKnotCount, BadKnotOrder, BadMultiplicity, BadEndMultiplicity,
    BadWeightCount, BadWeight, NonCanonicalEncoding
};

struct Frame {
    UUID identifier{};
    std::uint64_t revision = 0;
    std::array<double, 3> origin{};
    std::array<double, 3> xAxis{{1, 0, 0}};
    std::array<double, 3> yAxis{{0, 1, 0}};
    std::array<double, 3> zAxis{{0, 0, 1}};
    Handedness handedness = Handedness::Right;
};

struct ControlPoint {
    // Durable issued UUID, never an array index, knot ordinal, or TopoDS key.
    UUID identifier{};
    std::array<double, 3> local{};
};

struct Knot {
    double value = 0;
    std::uint8_t multiplicity = 0;
};

struct Definition {
    std::uint32_t schema = Schema;
    Domain domain = Domain::Sketch2D;
    Frame frame;
    std::uint8_t degree = 0;
    std::vector<ControlPoint> controlPoints;
    // Strictly increasing unique values; multiplicity expands only at build.
    std::vector<Knot> knots;
    // Empty means non-rational (all implicit 1). Otherwise exactly one/pole.
    std::vector<double> weights;
};

inline bool Finite(double value) noexcept { return std::isfinite(value); }
inline bool Nonzero(const UUID& value) noexcept { return retained_recipe::Nonzero(value); }
inline double Dot(const std::array<double, 3>& a, const std::array<double, 3>& b) noexcept {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline std::array<double, 3> Cross(const std::array<double, 3>& a,
                                   const std::array<double, 3>& b) noexcept {
    return {{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
             a[0] * b[1] - a[1] * b[0]}};
}
inline bool SameVector(const std::array<double, 3>& a, const std::array<double, 3>& b) noexcept {
    return a == b; // persisted flip comparison is exact; no silent reorientation.
}

inline Refusal ValidateFrame(const Frame& frame) noexcept {
    if (!Nonzero(frame.identifier) || frame.revision == 0 || frame.handedness != Handedness::Right) return Refusal::BadFrame;
    for (double value : frame.origin) if (!Finite(value) || std::abs(value) > CoordinateLimit) return Refusal::BadFrame;
    for (const auto* axis : {&frame.xAxis, &frame.yAxis, &frame.zAxis})
        for (double value : *axis) if (!Finite(value) || std::abs(value) > 1) return Refusal::BadFrame;
    constexpr double tolerance = 1e-12;
    if (std::abs(Dot(frame.xAxis, frame.xAxis) - 1) > tolerance
        || std::abs(Dot(frame.yAxis, frame.yAxis) - 1) > tolerance
        || std::abs(Dot(frame.zAxis, frame.zAxis) - 1) > tolerance
        || std::abs(Dot(frame.xAxis, frame.yAxis)) > tolerance
        || std::abs(Dot(frame.xAxis, frame.zAxis)) > tolerance
        || std::abs(Dot(frame.yAxis, frame.zAxis)) > tolerance) return Refusal::BadFrame;
    const auto cross = Cross(frame.xAxis, frame.yAxis);
    if (Dot(cross, frame.zAxis) < 1 - tolerance) return Refusal::BadFrame;
    return Refusal::None;
}

inline Refusal Validate(const Definition& value) noexcept {
    if (value.schema != Schema) return Refusal::BadSchema;
    if (value.domain != Domain::Sketch2D && value.domain != Domain::Path3D) return Refusal::BadDomain;
    if (const auto frame = ValidateFrame(value.frame); frame != Refusal::None) return frame;
    if (value.degree < MinimumDegree || value.degree > MaximumDegree) return Refusal::BadDegree;
    if (value.controlPoints.size() < std::size_t(value.degree) + 1
        || value.controlPoints.size() > MaximumControlPoints) return Refusal::BadControlPointCount;
    std::set<UUID> identifiers;
    for (const ControlPoint& point : value.controlPoints) {
        if (!Nonzero(point.identifier) || !identifiers.insert(point.identifier).second) return Refusal::DuplicateControlPointID;
        for (double scalar : point.local)
            if (!Finite(scalar) || std::abs(scalar) > CoordinateLimit) return Refusal::BadCoordinate;
        if (value.domain == Domain::Sketch2D && point.local[2] != 0) return Refusal::BadCoordinate;
    }
    if (value.knots.size() < 2 || value.knots.size() > MaximumDistinctKnotEntries) return Refusal::BadKnotCount;
    std::size_t expanded = 0;
    for (std::size_t index = 0; index < value.knots.size(); ++index) {
        const Knot& knot = value.knots[index];
        if (!Finite(knot.value) || std::abs(knot.value) > CoordinateLimit) return Refusal::BadKnotOrder;
        if (index != 0 && !(value.knots[index - 1].value < knot.value)) return Refusal::BadKnotOrder;
        if (knot.multiplicity == 0 || knot.multiplicity > value.degree + 1) return Refusal::BadMultiplicity;
        expanded += knot.multiplicity;
    }
    if (expanded > MaximumExpandedKnotEntries
        || expanded != value.controlPoints.size() + value.degree + 1) return Refusal::BadKnotCount;
    if (value.knots.front().multiplicity != value.degree + 1
        || value.knots.back().multiplicity != value.degree + 1) return Refusal::BadEndMultiplicity;
    for (std::size_t index = 1; index + 1 < value.knots.size(); ++index)
        if (value.knots[index].multiplicity > value.degree) return Refusal::BadMultiplicity;
    if (!value.weights.empty() && value.weights.size() != value.controlPoints.size()) return Refusal::BadWeightCount;
    for (double weight : value.weights)
        if (!Finite(weight) || weight < MinimumWeight || weight > MaximumWeight) return Refusal::BadWeight;
    return Refusal::None;
}

// A frame may be changed only by an explicit operation which proves the exact
// current axes and increments the persistent revision. This catches the common
// tangent/normal sign flip instead of accepting an equivalent-looking frame.
inline Refusal ValidateFrameReplacement(const Frame& before, const Frame& after) noexcept {
    if (ValidateFrame(before) != Refusal::None || ValidateFrame(after) != Refusal::None
        || before.identifier != after.identifier || after.revision != before.revision + 1) return Refusal::FrameFlip;
    return Refusal::None;
}
} // namespace core3d::bounded_curve
