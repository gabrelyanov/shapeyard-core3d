#pragma once

// C3 ruled general-loft value gate. This is a distinct composite feature and
// deliberately does not extend RectangularLoftDefinition/RectangularLoftSolid.
// Each contour edge owns the complete current C1/C1b curve state. Native UUIDs
// and authored sequence are correspondence; OCCT edge order is never authority.
#include "BoundedCurveEdit.hxx"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <vector>

namespace core3d::general_loft {
using bounded_curve::Digest;
using bounded_curve::UUID;

inline constexpr std::uint32_t Schema = 1;
inline constexpr std::size_t MinimumStations = 2;
inline constexpr std::size_t MaximumStations = 8;
inline constexpr std::size_t MinimumSegments = 3;
inline constexpr std::size_t MaximumSegments = 16;
inline constexpr double MaximumCoordinate = 1'000'000.0;
inline constexpr double MaximumAdjacentTwistRadians = 2.6179938779914944; // 150 degrees.

enum class Interpolation : std::uint8_t { Ruled = 1 };
enum class HolePolicy : std::uint8_t { Reject = 1 };
enum class CapPolicy : std::uint8_t { FlatBothEnds = 1 };
enum class Admission : std::uint8_t {
    Accepted = 0,
    InvalidSchema,
    InvalidOwner,
    InvalidPolicy,
    InvalidUnits,
    InvalidStationCount,
    InvalidSegmentCount,
    InvalidIdentifier,
    InvalidFrame,
    InvalidStationOrder,
    InvalidClosure,
    InvalidOrientation,
    InvalidCurve,
    InvalidCorrespondence,
    InvalidTwist
};

struct Junction {
    // identifier is station-local durable identity; correspondence is the same
    // native-issued UUID at this logical corner in every station.
    UUID identifier{};
    UUID correspondence{};
    std::array<double, 2> local{};
};

struct Segment {
    // identifier is also the embedded C1 feature identity. correspondence is
    // repeated at the same array position in every station and is never sorted.
    UUID identifier{};
    UUID correspondence{};
    UUID startJunction{};
    UUID endJunction{};
    bounded_curve::RetainedState curveState;
};

struct Station {
    UUID identifier{};
    // Physical ordering is projection of frame.origin on Definition::orderAxis.
    // The persisted value is an exact witness exposed in the native editor.
    double orderParameter = 0;
    double twistFromPreviousRadians = 0;
    bounded_curve::Frame frame;
    std::vector<Junction> junctions;
    std::vector<Segment> segments;
};

struct Definition {
    std::uint32_t schema = Schema;
    retained_recipe::OwnerKey owner;
    UUID feature{};
    std::uint64_t definitionRevision = 1;
    Digest recipeDigest{};
    double dimensionMetersPerUnit = 0;
    Interpolation interpolation = Interpolation::Ruled;
    HolePolicy holes = HolePolicy::Reject;
    CapPolicy caps = CapPolicy::FlatBothEnds;
    std::array<double, 3> orderAxis{{0, 0, 1}};
    std::vector<Station> stations;
};

inline double Dot(const std::array<double, 3>& a,
                  const std::array<double, 3>& b) noexcept {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline std::array<double, 3> Cross(const std::array<double, 3>& a,
                                   const std::array<double, 3>& b) noexcept {
    return {{a[1] * b[2] - a[2] * b[1],
             a[2] * b[0] - a[0] * b[2],
             a[0] * b[1] - a[1] * b[0]}};
}
inline bool SameFrameBits(const bounded_curve::Frame& a,
                          const bounded_curve::Frame& b) noexcept {
    return a.identifier == b.identifier && a.revision == b.revision
        && a.origin == b.origin && a.xAxis == b.xAxis && a.yAxis == b.yAxis
        && a.zAxis == b.zAxis && a.handedness == b.handedness;
}
inline bool FiniteCoordinate(double value) noexcept {
    return std::isfinite(value) && std::abs(value) <= MaximumCoordinate;
}
inline bool UnitVector(const std::array<double, 3>& value) noexcept {
    for (double scalar : value) if (!FiniteCoordinate(scalar)) return false;
    return std::abs(Dot(value, value) - 1) <= 1e-12;
}
inline bool SameScalar(double a, double b) noexcept {
    return std::abs(a - b) <= 1e-10 * std::max({1.0, std::abs(a), std::abs(b)});
}

inline Admission Validate(const Definition& value) noexcept {
    try {
        if (value.schema != Schema) return Admission::InvalidSchema;
        if (!retained_recipe::Valid(value.owner)
            || !bounded_curve::Nonzero(value.feature)
            || value.definitionRevision == 0
            || !retained_recipe::Nonzero(value.recipeDigest)) return Admission::InvalidOwner;
        if (value.interpolation != Interpolation::Ruled
            || value.holes != HolePolicy::Reject
            || value.caps != CapPolicy::FlatBothEnds) return Admission::InvalidPolicy;
        if (!std::isfinite(value.dimensionMetersPerUnit)
            || value.dimensionMetersPerUnit <= 0
            || value.dimensionMetersPerUnit > 1'000'000) return Admission::InvalidUnits;
        if (!UnitVector(value.orderAxis)) return Admission::InvalidStationOrder;
        if (value.stations.size() < MinimumStations
            || value.stations.size() > MaximumStations) return Admission::InvalidStationCount;

        std::vector<UUID> junctionCorrespondence, segmentCorrespondence;
        std::set<UUID> stationIDs, stationLocalIDs, curveFeatureIDs;
        std::array<double, 3> previousProjectedX{};
        double previousOrder = 0;
        for (std::size_t stationIndex = 0; stationIndex < value.stations.size(); ++stationIndex) {
            const Station& station = value.stations[stationIndex];
            if (!bounded_curve::Nonzero(station.identifier)
                || !stationIDs.insert(station.identifier).second) return Admission::InvalidIdentifier;
            if (bounded_curve::ValidateFrame(station.frame) != bounded_curve::Refusal::None)
                return Admission::InvalidFrame;
            if (station.junctions.size() < MinimumSegments
                || station.junctions.size() > MaximumSegments
                || station.segments.size() != station.junctions.size())
                return Admission::InvalidSegmentCount;

            const double projectedOrder = Dot(station.frame.origin, value.orderAxis);
            if (!FiniteCoordinate(station.orderParameter)
                || !SameScalar(station.orderParameter, projectedOrder)
                || (stationIndex != 0 && !(station.orderParameter > previousOrder)))
                return Admission::InvalidStationOrder;
            previousOrder = station.orderParameter;

            std::array<double, 3> projectedX = station.frame.xAxis;
            const double axial = Dot(projectedX, value.orderAxis);
            for (std::size_t component = 0; component < 3; ++component)
                projectedX[component] -= axial * value.orderAxis[component];
            const double projectedNorm = std::sqrt(Dot(projectedX, projectedX));
            if (!std::isfinite(projectedNorm) || projectedNorm <= 1e-6)
                return Admission::InvalidTwist;
            for (double& component : projectedX) component /= projectedNorm;
            if (stationIndex == 0) {
                if (station.twistFromPreviousRadians != 0) return Admission::InvalidTwist;
            } else {
                const double measured = std::atan2(
                    Dot(value.orderAxis, Cross(previousProjectedX, projectedX)),
                    Dot(previousProjectedX, projectedX));
                if (!std::isfinite(station.twistFromPreviousRadians)
                    || std::abs(station.twistFromPreviousRadians) > MaximumAdjacentTwistRadians
                    || !SameScalar(measured, station.twistFromPreviousRadians))
                    return Admission::InvalidTwist;
            }
            previousProjectedX = projectedX;

            std::set<UUID> localJunctions, localCorrespondence;
            double twiceArea = 0;
            std::vector<UUID> thisJunctionCorrespondence, thisSegmentCorrespondence;
            for (std::size_t index = 0; index < station.junctions.size(); ++index) {
                const Junction& junction = station.junctions[index];
                const Junction& next = station.junctions[(index + 1) % station.junctions.size()];
                if (!bounded_curve::Nonzero(junction.identifier)
                    || !bounded_curve::Nonzero(junction.correspondence)
                    || !localJunctions.insert(junction.identifier).second
                    || !localCorrespondence.insert(junction.correspondence).second
                    || !stationLocalIDs.insert(junction.identifier).second)
                    return Admission::InvalidIdentifier;
                if (!FiniteCoordinate(junction.local[0]) || !FiniteCoordinate(junction.local[1]))
                    return Admission::InvalidClosure;
                twiceArea += junction.local[0] * next.local[1]
                    - next.local[0] * junction.local[1];
                thisJunctionCorrespondence.push_back(junction.correspondence);
            }
            if (!std::isfinite(twiceArea) || twiceArea <= 1e-12)
                return Admission::InvalidOrientation;

            std::set<UUID> localSegments, localSegmentCorrespondence;
            for (std::size_t index = 0; index < station.segments.size(); ++index) {
                const Segment& segment = station.segments[index];
                const Junction& start = station.junctions[index];
                const Junction& end = station.junctions[(index + 1) % station.junctions.size()];
                if (!bounded_curve::Nonzero(segment.identifier)
                    || !bounded_curve::Nonzero(segment.correspondence)
                    || !localSegments.insert(segment.identifier).second
                    || !localSegmentCorrespondence.insert(segment.correspondence).second
                    || !stationLocalIDs.insert(segment.identifier).second
                    || !curveFeatureIDs.insert(segment.curveState.authority.feature).second
                    || segment.identifier != segment.curveState.authority.feature)
                    return Admission::InvalidIdentifier;
                if (segment.startJunction != start.identifier
                    || segment.endJunction != end.identifier) return Admission::InvalidClosure;
                if (!bounded_curve::ValidState(segment.curveState)
                    || !(segment.curveState.authority.owner == value.owner)
                    || segment.curveState.definition.domain != bounded_curve::Domain::Sketch2D
                    || !SameFrameBits(segment.curveState.definition.frame, station.frame))
                    return Admission::InvalidCurve;
                const auto& points = segment.curveState.definition.controlPoints;
                if (points.empty()
                    || points.front().local != std::array<double, 3>{{start.local[0], start.local[1], 0}}
                    || points.back().local != std::array<double, 3>{{end.local[0], end.local[1], 0}})
                    return Admission::InvalidClosure;
                thisSegmentCorrespondence.push_back(segment.correspondence);
            }
            if (stationIndex == 0) {
                junctionCorrespondence = std::move(thisJunctionCorrespondence);
                segmentCorrespondence = std::move(thisSegmentCorrespondence);
            } else if (thisJunctionCorrespondence != junctionCorrespondence
                       || thisSegmentCorrespondence != segmentCorrespondence) {
                // Exact authored sequence: never rotate/reverse/sort profiles.
                return Admission::InvalidCorrespondence;
            }
        }
        return Admission::Accepted;
    } catch (...) {
        return Admission::InvalidCurve;
    }
}

inline bool PreservesIdentityAndCorrespondence(const Definition& before,
                                               const Definition& after) noexcept {
    if (!(before.owner == after.owner) || before.feature != after.feature
        || before.stations.size() != after.stations.size()) return false;
    for (std::size_t s = 0; s < before.stations.size(); ++s) {
        const Station& a = before.stations[s]; const Station& b = after.stations[s];
        if (a.identifier != b.identifier || a.junctions.size() != b.junctions.size()
            || a.segments.size() != b.segments.size()) return false;
        for (std::size_t i = 0; i < a.junctions.size(); ++i)
            if (a.junctions[i].identifier != b.junctions[i].identifier
                || a.junctions[i].correspondence != b.junctions[i].correspondence) return false;
        for (std::size_t i = 0; i < a.segments.size(); ++i)
            if (a.segments[i].identifier != b.segments[i].identifier
                || a.segments[i].correspondence != b.segments[i].correspondence) return false;
    }
    return true;
}
} // namespace core3d::general_loft
