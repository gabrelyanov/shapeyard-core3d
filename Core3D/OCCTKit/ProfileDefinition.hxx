#pragma once

// Shared construction inputs and validation for touch-authored profiles,
// persistent native features and future typed modeling commands. Validation
// has no viewport, document mutation or UI ownership dependency.
#include "ProfileCurveAdmission.hxx"
#include <gp_Pnt2d.hxx>
#include <gp_Pnt.hxx>
#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace core3d {
struct ProfileCircularSection {
    gp_Pnt2d center;
    double outerRadius = 0;
    double innerRadius = 0; // Zero is a disk; positive is a concentric hole.
};
struct ProfileCircularHole {
    gp_Pnt2d center;
    double radius = 0;
};

struct ProfileDefinition {
    std::optional<ProfileCurveSection> curves; // Explicit authored line/arc loops.
    std::vector<gp_Pnt2d> points;
    std::optional<ProfileCircularSection> circle;
    std::vector<ProfileCircularHole> holes;
    int plane = 0; // 0 XY -> +Z, 1 XZ -> +Y, 2 YZ -> +X.
    double depth = 10; // Extrusion in document length units; revolution in degrees.
    bool revolve = false;
};

inline double ProfileCross(const gp_Pnt2d& a, const gp_Pnt2d& b, const gp_Pnt2d& c) {
    return (b.X() - a.X()) * (c.Y() - a.Y()) - (b.Y() - a.Y()) * (c.X() - a.X());
}
inline bool ProfilePointOnSegment(const gp_Pnt2d& p, const gp_Pnt2d& a, const gp_Pnt2d& b) {
    constexpr double tolerance = 1e-7;
    return std::abs(ProfileCross(a, b, p)) <= tolerance * a.Distance(b)
        && p.X() >= std::min(a.X(), b.X()) - tolerance
        && p.X() <= std::max(a.X(), b.X()) + tolerance
        && p.Y() >= std::min(a.Y(), b.Y()) - tolerance
        && p.Y() <= std::max(a.Y(), b.Y()) + tolerance;
}
inline bool ProfileSegmentsMeet(const gp_Pnt2d& a, const gp_Pnt2d& b,
                         const gp_Pnt2d& c, const gp_Pnt2d& d) {
    const double abC = ProfileCross(a, b, c), abD = ProfileCross(a, b, d);
    const double cdA = ProfileCross(c, d, a), cdB = ProfileCross(c, d, b);
    const bool proper = ((abC > 0 && abD < 0) || (abC < 0 && abD > 0))
        && ((cdA > 0 && cdB < 0) || (cdA < 0 && cdB > 0));
    return proper || ProfilePointOnSegment(c, a, b) || ProfilePointOnSegment(d, a, b)
        || ProfilePointOnSegment(a, c, d) || ProfilePointOnSegment(b, c, d);
}
inline bool ValidateProfileOutline(const std::vector<gp_Pnt2d>& points, int plane,
                            double depth, double& signedArea) {
    constexpr double coordinateLimit = 1e6, minimumEdge = 1e-3;
    signedArea = 0;
    if (plane < 0 || plane > 2 || points.size() < 3 || points.size() > 64
        || !std::isfinite(depth) || depth < minimumEdge || depth > coordinateLimit) { return false; }
    for (const auto& point : points) {
        if (!std::isfinite(point.X()) || !std::isfinite(point.Y())
            || std::abs(point.X()) > coordinateLimit || std::abs(point.Y()) > coordinateLimit) { return false; }
    }
    for (std::size_t i = 0; i < points.size(); ++i) {
        const auto& a = points[i];
        const auto& b = points[(i + 1) % points.size()];
        const auto& c = points[(i + 2) % points.size()];
        for (std::size_t j = i + 1; j < points.size(); ++j) {
            if (a.Distance(points[j]) < minimumEdge) { return false; }
        }
        if (std::abs(ProfileCross(a, b, c)) <= 1e-7 * (a.Distance(b) + b.Distance(c))) { return false; }
        // Shift the shoelace origin to reduce cancellation for a small outline
        // authored far from the work-plane origin.
        signedArea += ProfileCross(points.front(), a, b) * 0.5;
        for (std::size_t j = i + 1; j < points.size(); ++j) {
            if (j == (i + 1) % points.size() || (j + 1) % points.size() == i) { continue; }
            if (ProfileSegmentsMeet(a, b, points[j], points[(j + 1) % points.size()])) { return false; }
        }
    }
    return std::isfinite(signedArea) && std::abs(signedArea) >= 1e-6;
}
inline bool ProfileExpectedVolume(const std::vector<gp_Pnt2d>& points, int plane,
                           double parameter, bool revolve, double& signedArea, double& volume) {
    if (!ValidateProfileOutline(points, plane, revolve ? 1.0 : parameter, signedArea)) { return false; }
    if (!revolve) { volume = std::abs(signedArea) * parameter; return std::isfinite(volume) && volume > 0; }
    if (!std::isfinite(parameter) || parameter < 0.001 || parameter > 360) { return false; }
    // One-sided contours avoid sweeping through the axis and self-overlapping.
    // Axis-touching edges are valid, including degenerate pole edges in the solid.
    for (const auto& point : points) { if (point.X() < 0) { return false; } }
    double firstMoment = 0;
    const auto& origin = points.front();
    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
        const auto& a = points[i]; const auto& b = points[i + 1];
        const double area = ProfileCross(origin, a, b) * 0.5;
        firstMoment += area * (origin.X() + a.X() + b.X()) / 3.0;
    }
    volume = std::abs(firstMoment) * (parameter * std::acos(-1.0) / 180.0);
    return std::isfinite(volume) && volume > 1e-8;
}

inline bool ProfileHolesArea(const std::vector<gp_Pnt2d>& points,
                      const std::vector<ProfileCircularHole>& holes, double& area) {
    constexpr double clearance = 1e-3, limit = 1e6;
    area = 0;
    if (holes.size() > 16 || points.size() < 3) { return false; }
    for (std::size_t i = 0; i < holes.size(); ++i) {
        const auto& hole = holes[i];
        if (!std::isfinite(hole.center.X()) || !std::isfinite(hole.center.Y())
            || !std::isfinite(hole.radius) || hole.radius < clearance
            || std::abs(hole.center.X()) + hole.radius > limit
            || std::abs(hole.center.Y()) + hole.radius > limit) { return false; }
        bool inside = false;
        for (std::size_t edge = 0; edge < points.size(); ++edge) {
            const auto& a = points[edge]; const auto& b = points[(edge + 1) % points.size()];
            const double dx = b.X() - a.X(), dy = b.Y() - a.Y();
            const double lengthSquared = dx * dx + dy * dy;
            if (!std::isfinite(lengthSquared) || lengthSquared <= 0) { return false; }
            const double t = std::max(0.0, std::min(1.0,
                ((hole.center.X() - a.X()) * dx + (hole.center.Y() - a.Y()) * dy) / lengthSquared));
            const double distance = std::hypot(hole.center.X() - a.X() - t * dx,
                                               hole.center.Y() - a.Y() - t * dy);
            if (!std::isfinite(distance) || distance < hole.radius + clearance) { return false; }
            // Half-open ray crossings handle concave polygons and shared vertices.
            if ((a.Y() > hole.center.Y()) != (b.Y() > hole.center.Y())) {
                const double crossingX = a.X() + (hole.center.Y() - a.Y()) * dx / dy;
                if (hole.center.X() < crossingX) { inside = !inside; }
            }
        }
        if (!inside) { return false; }
        for (std::size_t other = 0; other < i; ++other) {
            if (hole.center.Distance(holes[other].center) < hole.radius + holes[other].radius + clearance) {
                return false;
            }
        }
        area += std::acos(-1.0) * hole.radius * hole.radius;
    }
    return std::isfinite(area);
}

inline bool ProfileDefinitionExpectedVolume(const std::vector<gp_Pnt2d>& points,
    const std::optional<ProfileCircularSection>& circle, const std::vector<ProfileCircularHole>& holes,
    int plane, double parameter, bool revolve, double& signedArea, double& volume) {
    if (!holes.empty() && (circle || revolve)) { return false; }
    if (!circle) {
        if (!ProfileExpectedVolume(points, plane, parameter, revolve, signedArea, volume)) { return false; }
        if (holes.empty()) { return true; }
        double holeArea = 0;
        if (!ProfileHolesArea(points, holes, holeArea)) { return false; }
        const double remaining = std::abs(signedArea) - holeArea;
        volume = remaining * parameter;
        // Keep signedArea as the original outer winding for wire construction.
        return remaining >= 1e-6 && std::isfinite(volume) && volume > 1e-8;
    }
    // A typed circular section cannot also carry a polygon outline.
    if (!points.empty() || plane < 0 || plane > 2) { return false; }
    const auto& c = *circle;
    constexpr double minimum = 1e-3, limit = 1e6;
    for (const double value : {c.center.X(), c.center.Y(), c.outerRadius, c.innerRadius, parameter}) {
        if (!std::isfinite(value)) { return false; }
    }
    if (parameter < minimum || parameter > (revolve ? 360.0 : limit) || c.outerRadius < minimum
        || c.innerRadius < 0 || (c.innerRadius > 0 && c.innerRadius < minimum)
        || c.outerRadius - c.innerRadius < minimum
        || std::abs(c.center.X()) + c.outerRadius > limit
        || std::abs(c.center.Y()) + c.outerRadius > limit) { return false; }
    signedArea = std::acos(-1.0) * (c.outerRadius - c.innerRadius) * (c.outerRadius + c.innerRadius);
    if (revolve) {
        // Keep the whole cross-section strictly outside the revolution axis.
        // Axis-crossing spindle/horn surfaces are not admitted as valid tubes.
        if (c.center.X() - c.outerRadius < minimum) { return false; }
        volume = signedArea * c.center.X() * (parameter * std::acos(-1.0) / 180.0);
    } else {
        volume = signedArea * parameter;
    }
    return std::isfinite(volume) && volume > 0;
}

// One typed entry point preserves all existing polygon/circular behavior.
// Curves cannot carry a simultaneous legacy outline or hole payload.
inline bool ProfileDefinitionExpectedVolume(const ProfileDefinition& definition,
    double& signedArea, double& volume) {
    signedArea=0;volume=0;
    if (!definition.curves)
        return ProfileDefinitionExpectedVolume(definition.points,definition.circle,definition.holes,
            definition.plane,definition.depth,definition.revolve,signedArea,volume);
    if (!definition.points.empty() || definition.circle || !definition.holes.empty()
        || definition.plane<0 || definition.plane>2 || !std::isfinite(definition.depth)
        || definition.depth<1e-3 || definition.depth>(definition.revolve?360.0:1e6)) return false;
    ProfileCurveSectionInspection inspection;
    if (!InspectProfileCurveSection(*definition.curves,inspection)) return false;
    double expected=inspection.area*definition.depth;
    if (definition.revolve) {
        // Authored U is radial in each existing work-plane mapping. A section
        // crossing the axis would sweep overlapping material and is refused.
        if (inspection.outerBounds[0]<0) return false;
        for (const auto& bounds:inspection.innerBounds)
            if (bounds[0]<0) return false;
        expected=inspection.firstMomentX*(definition.depth*std::acos(-1.0)/180.0);
    }
    if (!std::isfinite(expected) || expected<=1e-8) return false;
    signedArea=inspection.area;volume=expected;return true;
}

inline gp_Pnt ProfilePointInPlane(const gp_Pnt2d& p, int plane) {
    switch (plane) {
        case 0: return gp_Pnt(p.X(), p.Y(), 0);
        case 1: return gp_Pnt(p.X(), 0, p.Y());
        default: return gp_Pnt(0, p.X(), p.Y());
    }
}

} // namespace core3d
