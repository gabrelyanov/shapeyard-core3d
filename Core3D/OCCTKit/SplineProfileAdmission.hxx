#pragma once

// T-C C4: structural admission and authored (non-OCCT) measurement for
// profile sections containing spline segments. Line/arc terms reuse the exact
// closed forms of ProfileCurveStructure.hxx; spline terms use Gauss-Legendre
// quadrature over knot spans, which is exact for the polynomial integrands
// (16 points per span integrate degree <= 31 >= 3*SplineMaximumDegree - 1).
// Legacy line/arc-only sections never route through this header.
#include "SplineCurveDefinition.hxx"
#include <functional>

namespace core3d {

struct SplineProfileSectionMoments {
    double area = 0;     // Outer minus inner, positive.
    double momentU = 0;  // Signed (winding-normalized, role-weighted) ∬u dA.
    double momentV = 0;  // Signed ∬v dA.
    std::array<double, 4> outerBounds{};
    std::vector<std::array<double, 4>> innerBounds;
};

namespace spline_profile_admission {
constexpr double clearance = 1e-3;
constexpr double coordinateLimit = 1e6;
constexpr double minimumLength = 1e-3;
constexpr double endpointTolerance = 1e-7;

inline void GaussLegendre(int n, std::vector<double>& nodes, std::vector<double>& weights) {
    nodes.assign(std::size_t(n), 0);
    weights.assign(std::size_t(n), 0);
    const double pi = std::acos(-1.0);
    for (int i = 0; i < (n + 1) / 2; ++i) {
        double z = std::cos(pi * (i + 0.75) / (n + 0.5)), previous = 0, p1 = 1, p2 = 0, pp = 0;
        do {
            p1 = 1.0; p2 = 0.0;
            for (int j = 0; j < n; ++j) {
                const double p3 = p2; p2 = p1;
                p1 = ((2.0 * j + 1.0) * z * p2 - j * p3) / (j + 1.0);
            }
            pp = n * (z * p1 - p2) / (z * z - 1.0);
            previous = z; z = previous - p1 / pp;
        } while (std::abs(z - previous) > 1e-14);
        nodes[std::size_t(i)] = -z; nodes[std::size_t(n - 1 - i)] = z;
        weights[std::size_t(i)] = 2.0 / ((1.0 - z * z) * pp * pp);
        weights[std::size_t(n - 1 - i)] = weights[std::size_t(i)];
    }
}

struct LoopMoments {
    double signedArea = 0, signedMomentU = 0, signedMomentV = 0;
    std::array<double, 4> bounds{coordinateLimit, -coordinateLimit, coordinateLimit,
                                 -coordinateLimit};
};

inline void IncludePoint(std::array<double, 4>& bounds, const gp_Pnt2d& p) {
    bounds[0] = std::min(bounds[0], p.X()); bounds[1] = std::max(bounds[1], p.X());
    bounds[2] = std::min(bounds[2], p.Y()); bounds[3] = std::max(bounds[3], p.Y());
}

// Exact closed forms, mirroring InspectProfileCurveLoopStructure for area/momentU
// and adding the symmetric momentV term.
inline void AccumulateLine(const gp_Pnt2d& a, const gp_Pnt2d& b, LoopMoments& m) {
    m.signedArea += 0.5 * (a.X() * b.Y() - b.X() * a.Y());
    m.signedMomentU += (b.Y() - a.Y()) * (a.X() * a.X() + a.X() * b.X() + b.X() * b.X()) / 6.0;
    m.signedMomentV -= (b.X() - a.X()) * (a.Y() * a.Y() + a.Y() * b.Y() + b.Y() * b.Y()) / 6.0;
}

inline void AccumulateArc(const ProfileCurveSegment& segment, LoopMoments& m) {
    const double degreesToRadians = std::acos(-1.0) / 180.0;
    const double first = segment.startDegrees * degreesToRadians;
    const double sweep = segment.sweepDegrees * degreesToRadians;
    const double last = first + sweep, radius = segment.radius;
    const double cx = segment.center.X(), cy = segment.center.Y();
    const double dSin = std::sin(last) - std::sin(first);
    const double dCos = std::cos(last) - std::cos(first);
    const double dSin2 = std::sin(2 * last) - std::sin(2 * first);
    m.signedArea += 0.5 * (radius * cx * dSin - radius * cy * dCos + radius * radius * sweep);
    m.signedMomentU += 0.5 * radius * (cx * cx * dSin
        + cx * radius * (sweep + dSin2 / 2.0)
        + radius * radius * (dSin - (std::pow(std::sin(last), 3) - std::pow(std::sin(first), 3)) / 3.0));
    m.signedMomentV += 0.5 * radius * (-cy * cy * dCos
        + cy * radius * (sweep - dSin2 / 2.0)
        - radius * radius * (dCos - (std::pow(std::cos(last), 3) - std::pow(std::cos(first), 3)) / 3.0));
}

inline bool AccumulateSpline(const SplineCurveSegment& segment,
    const std::vector<double>& nodes, const std::vector<double>& weights, LoopMoments& m) {
    for (std::size_t span = 0; span + 1 < segment.knots.size(); ++span) {
        const double a = segment.knots[span], b = segment.knots[span + 1];
        const double half = 0.5 * (b - a), mid = 0.5 * (a + b);
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            gp_Pnt2d point, derivative;
            if (!SplineEvaluate(segment, mid + half * nodes[i], point, derivative)) return false;
            const double w = weights[i] * half;
            // Same Green's-theorem forms as AccumulateLine/AccumulateArc so
            // mixed loops sum consistently per edge.
            m.signedArea += w * 0.5 * (point.X() * derivative.Y() - point.Y() * derivative.X());
            m.signedMomentU += w * 0.5 * point.X() * point.X() * derivative.Y();
            m.signedMomentV -= w * 0.5 * point.Y() * point.Y() * derivative.X();
        }
    }
    return true;
}
} // namespace spline_profile_admission

// Structural admission + authored boundary integration for one loop that may
// mix line, circular-arc and spline segments. Spline payloads are looked up by
// the owning segment identifier; a kind-Spline segment without a payload (or
// vice versa) is malformed.
inline bool InspectSplineProfileLoop(const ProfileCurveLoop& loop,
    const SplineProfileSpec& spec, std::set<ProfileCurveID>& sectionIdentifiers,
    std::set<ProfileCurveID>& claimedSplines, std::size_t& inspectedElements,
    spline_profile_admission::LoopMoments& output) noexcept {
    output = {};
    try {
        using namespace spline_profile_admission;
        constexpr std::size_t maximumElements = 512;
        if (loop.identifier == 0 || !sectionIdentifiers.insert(loop.identifier).second
            || loop.vertices.size() < 2 || loop.vertices.size() > maximumElements
            || loop.segments.size() != loop.vertices.size()
            || inspectedElements > 2 * maximumElements - loop.vertices.size() - loop.segments.size())
            return false;
        inspectedElements += loop.vertices.size() + loop.segments.size();
        std::vector<double> nodes, weights;
        GaussLegendre(16, nodes, weights);
        const auto finitePoint = [](const gp_Pnt2d& p) {
            return std::isfinite(p.X()) && std::isfinite(p.Y())
                && std::abs(p.X()) <= coordinateLimit && std::abs(p.Y()) <= coordinateLimit;
        };
        LoopMoments result;
        for (std::size_t i = 0; i < loop.segments.size(); ++i) {
            const auto& segment = loop.segments[i];
            const auto& a = loop.vertices[i];
            const auto& b = loop.vertices[(i + 1) % loop.vertices.size()];
            if (segment.identifier == 0 || !sectionIdentifiers.insert(segment.identifier).second
                || a.identifier == 0 || !sectionIdentifiers.insert(a.identifier).second
                || segment.startVertex != a.identifier || segment.endVertex != b.identifier
                || !finitePoint(a.point) || a.point.Distance(b.point) < minimumLength) return false;
            IncludePoint(result.bounds, a.point);
            IncludePoint(result.bounds, b.point);
            if (segment.kind == ProfileCurveKind::Line) {
                if (segment.center.X() != 0 || segment.center.Y() != 0 || segment.radius != 0
                    || segment.startDegrees != 0 || segment.sweepDegrees != 0) return false;
                AccumulateLine(a.point, b.point, result);
            } else if (segment.kind == ProfileCurveKind::CircularArc) {
                // Endpoint agreement and arc ranges are re-proven by the exact
                // accumulation inputs; keep the same bounds as the legacy path.
                const double degreesToRadians = std::acos(-1.0) / 180.0;
                const double first = segment.startDegrees * degreesToRadians;
                const double sweep = segment.sweepDegrees * degreesToRadians;
                const double last = first + sweep;
                if (!finitePoint(segment.center) || !std::isfinite(segment.radius)
                    || segment.radius < minimumLength || segment.radius > coordinateLimit
                    || !std::isfinite(segment.startDegrees) || std::abs(segment.startDegrees) > 360
                    || !std::isfinite(segment.sweepDegrees) || std::abs(sweep) < 1e-6 * 180 / std::acos(-1.0)
                    || std::abs(segment.sweepDegrees) >= 360
                    || segment.radius * std::abs(sweep) < minimumLength) return false;
                const auto pointAt = [&](double theta) {
                    return gp_Pnt2d(segment.center.X() + segment.radius * std::cos(theta),
                                    segment.center.Y() + segment.radius * std::sin(theta));
                };
                if (pointAt(first).Distance(a.point) > endpointTolerance
                    || pointAt(last).Distance(b.point) > endpointTolerance) return false;
                AccumulateArc(segment, result);
            } else if (segment.kind == ProfileCurveKind::Spline) {
                if (segment.center.X() != 0 || segment.center.Y() != 0 || segment.radius != 0
                    || segment.startDegrees != 0 || segment.sweepDegrees != 0) return false;
                const SplineCurveSegment* spline = nullptr;
                for (const auto& candidate : spec.segments)
                    if (candidate.identifier == segment.identifier) { spline = &candidate; break; }
                if (!spline || !claimedSplines.insert(segment.identifier).second
                    || !ValidSplineCurveSegment(*spline, a.point, b.point, sectionIdentifiers)
                    || spec.segments.size() > SplineMaximumSegmentsPerSection
                    || !AccumulateSpline(*spline, nodes, weights, result)) return false;
                for (const auto& pole : spline->poles) IncludePoint(result.bounds, pole.point);
            } else return false;
        }
        if (!std::isfinite(result.signedArea) || std::abs(result.signedArea) < 1e-6
            || !std::isfinite(result.signedMomentU) || !std::isfinite(result.signedMomentV))
            return false;
        output = result;
        return true;
    } catch (...) { output = {}; return false; }
}

// Section-level structural admission and authored measurements. Geometric
// clearance, containment, closure and kernel validity remain the face
// builder's proof (SplineProfileFace.hxx), exactly as the legacy split between
// InspectProfileCurveSection and BuildProfileCurveFace.
inline bool InspectSplineProfileSection(const ProfileCurveSection& section,
    const SplineProfileSpec& spec, SplineProfileSectionMoments& output,
    const std::function<bool()>& cancelled = {}) noexcept {
    output = {};
    try {
        if (section.inner.size() > 16 || section.outer.vertices.empty()
            || spec.segments.empty() || spec.segments.size() > SplineMaximumSegmentsPerSection)
            return false;
        std::set<ProfileCurveID> identifiers, claimedSplines;
        std::size_t elements = 0;
        SplineProfileSectionMoments measured;
        double area = 0, momentU = 0, momentV = 0;
        std::size_t loopIndex = 0;
        const auto admitLoop = [&](const ProfileCurveLoop& loop) {
            spline_profile_admission::LoopMoments loopMoments;
            if (!InspectSplineProfileLoop(loop, spec, identifiers, claimedSplines, elements,
                    loopMoments)) return false;
            // Winding normalization: outer contributes positive, inner negative.
            const double orientation = loopMoments.signedArea > 0 ? 1.0 : -1.0;
            const double role = loopIndex == 0 ? 1.0 : -1.0;
            area += role * orientation * loopMoments.signedArea;
            momentU += role * orientation * loopMoments.signedMomentU;
            momentV += role * orientation * loopMoments.signedMomentV;
            if (loopIndex == 0) measured.outerBounds = loopMoments.bounds;
            else measured.innerBounds.push_back(loopMoments.bounds);
            ++loopIndex;
            return true;
        };
        if (!admitLoop(section.outer)) return false;
        for (const auto& loop : section.inner) {
            if (cancelled && cancelled()) return false;
            if (!admitLoop(loop)) return false;
        }
        if (claimedSplines.size() != spec.segments.size()) return false;
        if (!std::isfinite(area) || area < 1e-6
            || !std::isfinite(momentU) || !std::isfinite(momentV)) return false;
        measured.area = area;
        measured.momentU = momentU;
        measured.momentV = momentV;
        output = measured;
        return true;
    } catch (...) { output = {}; return false; }
}

// Explicit-axis revolve admission: exact one-sidedness from B-spline convex
// hulls (signed axis distance is linear in the poles), the tangency rule for
// axis contacts, the inner-loop policy, and the Pappus expected volume from
// authored moments. Returns the axis-side sign (+1/-1) for the face builder.
inline bool AdmitSplineProfileRevolve(const ProfileCurveSection& section,
    const SplineProfileSpec& spec, const SplineProfileSectionMoments& moments,
    double angleDegrees, double& expectedVolume) noexcept {
    expectedVolume = 0;
    try {
        using namespace spline_profile_admission;
        if (!spec.revolveAxis || !std::isfinite(angleDegrees)
            || angleDegrees < 0.001 || angleDegrees > 360) return false;
        const auto& axis = *spec.revolveAxis;
        if (!std::isfinite(axis.origin.X()) || !std::isfinite(axis.origin.Y())
            || std::abs(axis.origin.X()) > coordinateLimit
            || std::abs(axis.origin.Y()) > coordinateLimit) return false;
        const double length = std::hypot(axis.direction.X(), axis.direction.Y());
        if (!std::isfinite(length) || std::abs(length - 1.0) > 1e-9) return false;
        if (!section.inner.empty() && spec.innerLoopPolicy != SplineInnerLoopPolicy::Void)
            return false;
        double side = 0;
        const auto admitPoint = [&](const gp_Pnt2d& p, bool splineInteriorPole,
                                    bool splineEndpointPole,
                                    const SplineCurveSegment* spline, bool atStart) {
            const double s = SplineAxisSignedDistance(axis, p);
            if (s > clearance) {
                if (side < 0) return false; // Axis crossing.
                side = 1; return true;
            }
            if (s < -clearance) {
                if (side > 0) return false;
                side = -1; return true;
            }
            // Contact with the axis.
            if (splineInteriorPole) return false; // Interior pole touch: ambiguous bulge.
            if (splineEndpointPole && spline) {
                // Endpoint contact is admitted only with a nonzero tangent
                // perpendicular to the axis (no ambiguous tangency).
                gp_Pnt2d point, tangent;
                const double t = atStart ? spline->knots.front() : spline->knots.back();
                if (!SplineEvaluate(*spline, t, point, tangent)) return false;
                const double tangentLength = std::hypot(tangent.X(), tangent.Y());
                if (tangentLength < minimumLength) return false;
                if (std::abs(tangent.X() * axis.direction.X()
                        + tangent.Y() * axis.direction.Y()) > 1e-6 * tangentLength) return false;
            }
            return true;
        };
        std::size_t loopIndex = 0;
        const auto admitLoop = [&](const ProfileCurveLoop& loop, bool inner) {
            for (std::size_t i = 0; i < loop.segments.size(); ++i) {
                const auto& segment = loop.segments[i];
                const gp_Pnt2d& a = loop.vertices[i].point;
                const gp_Pnt2d& b = loop.vertices[(i + 1) % loop.vertices.size()].point;
                if (segment.kind == ProfileCurveKind::Spline) {
                    const SplineCurveSegment* spline = nullptr;
                    for (const auto& candidate : spec.segments)
                        if (candidate.identifier == segment.identifier) { spline = &candidate; break; }
                    if (!spline) return false;
                    for (std::size_t pole = 0; pole < spline->poles.size(); ++pole) {
                        const bool endpoint = pole == 0 || pole + 1 == spline->poles.size();
                        if (!admitPoint(spline->poles[pole].point, !endpoint, endpoint,
                                spline, pole == 0)) return false;
                    }
                } else {
                    if (!admitPoint(a, false, false, nullptr, false)
                        || !admitPoint(b, false, false, nullptr, false)) return false;
                    if (segment.kind == ProfileCurveKind::CircularArc) {
                        // Arc extrema along the axis normal must keep the same side.
                        const gp_Pnt2d normal(-axis.direction.Y(), axis.direction.X());
                        for (double sign : {-1.0, 1.0}) {
                            const gp_Pnt2d extreme(segment.center.X() + sign * segment.radius * normal.X(),
                                                   segment.center.Y() + sign * segment.radius * normal.Y());
                            const double degreesToRadians = std::acos(-1.0) / 180.0;
                            const double first = segment.startDegrees * degreesToRadians;
                            const double last = first + segment.sweepDegrees * degreesToRadians;
                            const double theta = std::atan2(extreme.Y() - segment.center.Y(),
                                                            extreme.X() - segment.center.X());
                            const double tau = 2 * std::acos(-1.0);
                            double delta = std::fmod(theta - first, tau);
                            if (delta < 0) delta += tau;
                            const bool traversed = segment.sweepDegrees >= 0
                                ? delta <= segment.sweepDegrees * degreesToRadians + 1e-12
                                : std::fmod(first - theta + tau, tau) <=
                                    -segment.sweepDegrees * degreesToRadians + 1e-12;
                            (void)last;
                            if (traversed) {
                                const double s = SplineAxisSignedDistance(axis, extreme);
                                if (s > clearance && side < 0) return false;
                                if (s < -clearance && side > 0) return false;
                            }
                        }
                    }
                }
            }
            if (inner) {
                // Void policy: inner loops stay strictly off the axis.
                for (const auto& vertex : loop.vertices)
                    if (std::abs(SplineAxisSignedDistance(axis, vertex.point)) < clearance)
                        return false;
                ++loopIndex;
            }
            return true;
        };
        if (!admitLoop(section.outer, false)) return false;
        loopIndex = 0;
        for (const auto& loop : section.inner)
            if (!admitLoop(loop, true)) return false;
        if (side == 0) return false; // Section fully on the axis is degenerate.
        // Pappus: V = angle * sum(role * sbar * area) with signed axis distance
        // of each winding-normalized centroid.
        const double normalDotMoment =
            axis.direction.X() * moments.momentV - axis.direction.Y() * moments.momentU;
        const double originTerm =
            (axis.direction.X() * axis.origin.Y() - axis.direction.Y() * axis.origin.X())
            * moments.area;
        const double signedMoment = normalDotMoment - originTerm;
        expectedVolume = signedMoment * side * (angleDegrees * std::acos(-1.0) / 180.0);
        return std::isfinite(expectedVolume) && expectedVolume > 1e-8;
    } catch (...) { expectedVolume = 0; return false; }
}

// Single typed entry for the native worker, mirroring
// ProfileDefinitionExpectedVolume for spline-carrying sections.
inline bool SplineProfileExpectedVolume(const ProfileCurveSection& section,
    const SplineProfileSpec& spec, int plane, double depth, bool revolve,
    const std::function<bool()>& cancelled, double& area, double& volume) noexcept {
    area = 0; volume = 0;
    try {
        if (plane < 0 || plane > 2 || !std::isfinite(depth)
            || depth < 1e-3 || depth > (revolve ? 360.0 : 1e6)) return false;
        SplineProfileSectionMoments moments;
        if (!InspectSplineProfileSection(section, spec, moments, cancelled)) return false;
        area = moments.area;
        if (revolve) {
            if (!AdmitSplineProfileRevolve(section, spec, moments, depth, volume)) return false;
        } else {
            // Extrusion keeps the legacy inner-loop behavior; the axis and
            // policy fields must be absent/zero by the typed-union rule.
            if (spec.revolveAxis || spec.innerLoopPolicy != SplineInnerLoopPolicy::Refuse)
                return false;
            volume = moments.area * depth;
        }
        return std::isfinite(volume) && volume > 1e-8;
    } catch (...) { area = 0; volume = 0; return false; }
}

} // namespace core3d
