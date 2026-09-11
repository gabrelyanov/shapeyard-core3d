#pragma once

// Structural inspection alone is not complete solid admission:
// exact intersection, containment, clearance and native face checks must follow.
#include <gp_Pnt2d.hxx>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <vector>

namespace core3d {

// IDs are scoped to one persistent feature. Independent copies have a new
// feature ID and can retain local element IDs for later selected-element edits.
using ProfileCurveID = std::uint32_t;
struct ProfileCurveVertex {
    ProfileCurveID identifier = 0;
    gp_Pnt2d point;
};
enum class ProfileCurveKind { Line = 0, CircularArc = 1 };
struct ProfileCurveSegment {
    ProfileCurveID identifier = 0;
    ProfileCurveID startVertex = 0, endVertex = 0;
    ProfileCurveKind kind = ProfileCurveKind::Line;
    gp_Pnt2d center;
    double radius = 0, startDegrees = 0, sweepDegrees = 0;
};
struct ProfileCurveLoop {
    ProfileCurveID identifier = 0;
    std::vector<ProfileCurveVertex> vertices;
    std::vector<ProfileCurveSegment> segments;
};
struct ProfileCurveSection {
    ProfileCurveLoop outer;
    std::vector<ProfileCurveLoop> inner;
};
struct ProfileCurveLoopInspection {
    // Relative to a caller-supplied shared origin, to avoid cancellation when
    // a small section is authored far from the work-plane origin.
    double signedArea = 0;
    double signedFirstMomentX = 0;
    std::array<double, 4> bounds{}; // minX,maxX,minY,maxY in authored coordinates.
};

inline bool InspectProfileCurveLoopStructure(
    const ProfileCurveLoop& loop, const gp_Pnt2d& integrationOrigin,
    std::set<ProfileCurveID>& sectionIdentifiers,
    std::size_t& inspectedVertices, std::size_t& inspectedSegments,
    ProfileCurveLoopInspection& output) noexcept {
    output = {};
    try {
        constexpr std::size_t maximumElements = 512;
        constexpr double limit = 1e6, minimumLength = 1e-3;
        constexpr double endpointTolerance = 1e-7;
        const double pi = std::acos(-1.0), degreesToRadians = pi / 180.0;
        const auto finitePoint = [](const gp_Pnt2d& p) {
            return std::isfinite(p.X()) && std::isfinite(p.Y())
                && std::abs(p.X()) <= limit && std::abs(p.Y()) <= limit;
        };
        const auto claim = [&](ProfileCurveID identifier) {
            return identifier != 0 && sectionIdentifiers.insert(identifier).second;
        };
        if (!claim(loop.identifier) || !finitePoint(integrationOrigin)
            || loop.vertices.size() < 2 || loop.vertices.size() > maximumElements
            || loop.segments.size() != loop.vertices.size()
            || inspectedVertices > maximumElements - loop.vertices.size()
            || inspectedSegments > maximumElements - loop.segments.size()) return false;
        inspectedVertices += loop.vertices.size();
        inspectedSegments += loop.segments.size();
        for (const auto& vertex : loop.vertices)
            if (!claim(vertex.identifier) || !finitePoint(vertex.point)) return false;
        for (const auto& segment : loop.segments)
            if (!claim(segment.identifier)) return false;
        ProfileCurveLoopInspection result;
        result.bounds = {limit, -limit, limit, -limit};
        const auto includePoint = [&](const gp_Pnt2d& point) {
            result.bounds[0] = std::min(result.bounds[0], point.X());
            result.bounds[1] = std::max(result.bounds[1], point.X());
            result.bounds[2] = std::min(result.bounds[2], point.Y());
            result.bounds[3] = std::max(result.bounds[3], point.Y());
        };
        for (std::size_t i = 0; i < loop.segments.size(); ++i) {
            const auto& segment = loop.segments[i];
            const auto& a = loop.vertices[i];
            const auto& b = loop.vertices[(i + 1) % loop.vertices.size()];
            // A vertex owns one shared endpoint. Ordered edges may not refer
            // to detached or foreign vertices, or silently reorder the loop.
            if (segment.startVertex != a.identifier || segment.endVertex != b.identifier
                || a.point.Distance(b.point) < minimumLength) return false;
            includePoint(a.point); includePoint(b.point);
            const double ax = a.point.X() - integrationOrigin.X();
            const double ay = a.point.Y() - integrationOrigin.Y();
            const double bx = b.point.X() - integrationOrigin.X();
            const double by = b.point.Y() - integrationOrigin.Y();
            if (segment.kind == ProfileCurveKind::Line) {
                // Closed typed union: unused fields cannot hide contradictory data.
                if (segment.center.X() != 0 || segment.center.Y() != 0
                    || segment.radius != 0 || segment.startDegrees != 0
                    || segment.sweepDegrees != 0) return false;
                result.signedArea += .5 * (ax * by - bx * ay);
                result.signedFirstMomentX += (by - ay) * (ax * ax + ax * bx + bx * bx) / 6.0;
            } else if (segment.kind == ProfileCurveKind::CircularArc) {
                if (!finitePoint(segment.center) || !std::isfinite(segment.radius)
                    || segment.radius < minimumLength || segment.radius > limit
                    || !std::isfinite(segment.startDegrees) || std::abs(segment.startDegrees) > 360
                    || !std::isfinite(segment.sweepDegrees)
                    || std::abs(segment.sweepDegrees) < 1e-6
                    || std::abs(segment.sweepDegrees) >= 360) return false;
                // Complete circles use at least two explicit arcs, retaining
                // distinct shared vertices instead of an ambiguous closed edge.
                const double first = segment.startDegrees * degreesToRadians;
                const double sweep = segment.sweepDegrees * degreesToRadians;
                const double last = first + sweep, radius = segment.radius;
                if (radius * std::abs(sweep) < minimumLength) return false;
                const auto pointAt = [&](double theta) {
                    return gp_Pnt2d(segment.center.X() + radius * std::cos(theta),
                                    segment.center.Y() + radius * std::sin(theta));
                };
                if (pointAt(first).Distance(a.point) > endpointTolerance
                    || pointAt(last).Distance(b.point) > endpointTolerance) return false;
                // Include every cardinal extremum actually traversed by the arc.
                const double low = std::min(first, last), high = std::max(first, last);
                for (int quadrant = -8; quadrant <= 8; ++quadrant) {
                    const double theta = quadrant * pi / 2.0;
                    if (theta >= low && theta <= high) {
                        const auto point = pointAt(theta);
                        if (!finitePoint(point)) return false;
                        includePoint(point);
                    }
                }
                const double cx = segment.center.X() - integrationOrigin.X();
                const double cy = segment.center.Y() - integrationOrigin.Y();
                result.signedArea += .5 * (radius * cx * (std::sin(last) - std::sin(first))
                    - radius * cy * (std::cos(last) - std::cos(first)) + radius * radius * sweep);
                const auto momentPrimitive = [&](double theta) {
                    const double sine = std::sin(theta);
                    return .5 * radius * (cx * cx * sine
                        + cx * radius * (theta + std::sin(2 * theta) / 2.0)
                        + radius * radius * (sine - sine * sine * sine / 3.0));
                };
                result.signedFirstMomentX += momentPrimitive(last) - momentPrimitive(first);
            } else return false;
        }
        if (!std::isfinite(result.signedArea) || std::abs(result.signedArea) < 1e-6
            || !std::isfinite(result.signedFirstMomentX)) return false;
        output = result;
        return true;
    } catch (...) { output = {}; return false; }
}

} // namespace core3d
