#pragma once

// T-C C4: bounded 2D sketch spline segments for profile sections, with the
// explicit revolve axis and inner-loop policy as editable source values.
// Field semantics mirror the C1 canonical SYCV/1 curve definition (persistent
// control-point identity, degree, distinct knots with multiplicities, optional
// positive rational weights), specialized to the authored profile UV plane so
// a later integration can lift these values to SYCV/1 records unchanged.
// This header has no document, presentation or viewport dependency.
#include "ProfileCurveStructure.hxx"
#include "RetainedFeatureRegistry.hxx"
#include <optional>

namespace core3d {

// The persisted identity is allocated by the frozen G0 registry. Keeping the
// key here makes every future descriptor/owner route consume that allocation;
// defining the numeric kind again in a C4 file is forbidden. This geometry
// slice does not install a production descriptor by itself.
inline constexpr retained_feature::Key SplineProfileRevolveRegistryKey{
    retained_feature::SplineProfileRevolveKind, 1};

inline constexpr std::size_t SplineMaximumPoles = 32;
inline constexpr int SplineMaximumDegree = 7;
inline constexpr std::size_t SplineMaximumSegmentsPerSection = 16;
inline constexpr double SplineEndpointTolerance = 1e-7;

// One spline replaces one ordered ProfileCurveSegment of kind
// ProfileCurveKind::Spline; the owning segment keeps the shared start/end
// vertices so loop topology and element identity stay uniform.
struct SplineCurveSegment {
    ProfileCurveID identifier = 0; // == owning ProfileCurveSegment identifier.
    int degree = 0;
    std::vector<ProfileCurveVertex> poles; // Persistent pole IDs + authored UV.
    std::vector<double> knots;             // Strictly increasing distinct knots.
    std::vector<int> multiplicities;       // One per distinct knot, clamped ends.
    std::vector<double> weights;           // Empty = non-rational; else one per pole, all > 0.
    bool operator==(const SplineCurveSegment& other) const {
        if (identifier != other.identifier || degree != other.degree
            || poles.size() != other.poles.size() || knots != other.knots
            || multiplicities != other.multiplicities || weights != other.weights) return false;
        for (std::size_t i = 0; i < poles.size(); ++i)
            if (poles[i].identifier != other.poles[i].identifier
                || poles[i].point.X() != other.poles[i].point.X()
                || poles[i].point.Y() != other.poles[i].point.Y()) return false;
        return true;
    }
};

// The revolve axis is an editable authored source value, never inferred from
// topology. The legacy implicit axis is origin (0,0), direction (0,1) in UV.
struct SplineRevolveAxis {
    gp_Pnt2d origin;
    gp_Pnt2d direction; // Unit length within 1e-9.
};

enum class SplineInnerLoopPolicy : int {
    Refuse = 0, // Revolve refuses any inner loop (legacy polygon behavior).
    Void = 1,   // Inner loops revolve to void shells; strict axis clearance.
};

struct SplineProfileSpec {
    std::vector<SplineCurveSegment> segments;
    std::optional<SplineRevolveAxis> revolveAxis; // Required iff the profile revolves.
    SplineInnerLoopPolicy innerLoopPolicy = SplineInnerLoopPolicy::Refuse;
};

// Bounded structural validation of one spline segment and its endpoint
// agreement with the owning shared vertices. The caller owns claiming the
// segment identifier; this function claims the pole identifiers into the same
// section set. Geometric clearance, containment and kernel checks are the face
// builder's proof, not this function's.
inline bool ValidSplineCurveSegment(const SplineCurveSegment& segment,
    const gp_Pnt2d& start, const gp_Pnt2d& end,
    std::set<ProfileCurveID>& sectionIdentifiers) noexcept {
    try {
        constexpr double coordinateLimit = 1e6;
        if (segment.identifier == 0
            || segment.degree < 1 || segment.degree > SplineMaximumDegree
            || segment.poles.size() < 2 || segment.poles.size() > SplineMaximumPoles
            || segment.poles.size() < std::size_t(segment.degree + 1)) return false;
        const std::size_t flatKnots = segment.poles.size() + std::size_t(segment.degree) + 1;
        if (segment.knots.size() < 2 || segment.knots.size() > flatKnots
            || segment.multiplicities.size() != segment.knots.size()) return false;
        std::size_t multiplicitySum = 0;
        double previous = 0;
        for (std::size_t i = 0; i < segment.knots.size(); ++i) {
            const double knot = segment.knots[i];
            const int multiplicity = segment.multiplicities[i];
            if (!std::isfinite(knot) || std::abs(knot) > coordinateLimit
                || (i != 0 && knot <= previous)) return false;
            previous = knot;
            // Clamped open knot vector: end multiplicities fix the endpoints.
            const int maximum = (i == 0 || i + 1 == segment.knots.size())
                ? segment.degree + 1 : segment.degree;
            if (multiplicity < 1 || multiplicity > maximum) return false;
            multiplicitySum += std::size_t(multiplicity);
        }
        if (multiplicitySum != flatKnots
            || segment.multiplicities.front() != segment.degree + 1
            || segment.multiplicities.back() != segment.degree + 1
            || segment.knots.back() - segment.knots.front() < 1e-9) return false;
        if (!segment.weights.empty() && segment.weights.size() != segment.poles.size()) return false;
        for (std::size_t i = 0; i < segment.poles.size(); ++i) {
            const auto& pole = segment.poles[i];
            if (pole.identifier == 0 || !sectionIdentifiers.insert(pole.identifier).second
                || !std::isfinite(pole.point.X()) || !std::isfinite(pole.point.Y())
                || std::abs(pole.point.X()) > coordinateLimit
                || std::abs(pole.point.Y()) > coordinateLimit) return false;
            if (!segment.weights.empty()
                && (!std::isfinite(segment.weights[i]) || segment.weights[i] <= 0
                    || segment.weights[i] > 1e6)) return false;
        }
        // Clamped ends interpolate the first/last pole; those must be the
        // owning segment's shared vertices, so wire closure stays exact.
        if (segment.poles.front().point.Distance(start) > SplineEndpointTolerance
            || segment.poles.back().point.Distance(end) > SplineEndpointTolerance) return false;
        return true;
    } catch (...) { return false; }
}

// De Boor evaluation on the clamped flat knot vector. t in [knots.front(),
// knots.back()]. Returns the curve point and first derivative.
inline bool SplineEvaluate(const SplineCurveSegment& segment, double t,
    gp_Pnt2d& point, gp_Pnt2d& derivative) noexcept {
    try {
        const int degree = segment.degree;
        const std::size_t poleCount = segment.poles.size();
        std::vector<double> flat;
        flat.reserve(poleCount + std::size_t(degree) + 1);
        for (std::size_t i = 0; i < segment.knots.size(); ++i)
            for (int m = 0; m < segment.multiplicities[i]; ++m) flat.push_back(segment.knots[i]);
        if (flat.size() != poleCount + std::size_t(degree) + 1) return false;
        const double low = flat.front(), high = flat.back();
        if (t < low || t > high) return false;
        std::size_t span = 0;
        if (t >= high) span = poleCount - 1;
        else {
            // Last index with flat[span] <= t < flat[span+1].
            std::size_t lo = std::size_t(degree), hi = poleCount;
            while (lo + 1 < hi) {
                const std::size_t mid = (lo + hi) / 2;
                if (flat[mid] <= t) lo = mid; else hi = mid;
            }
            span = lo;
        }
        const auto weight = [&](std::size_t i) {
            return segment.weights.empty() ? 1.0 : segment.weights[i];
        };
        // Rational De Boor in homogeneous (w*x, w*y, w) space.
        std::vector<std::array<double, 3>> d(std::size_t(degree) + 1);
        for (int j = 0; j <= degree; ++j) {
            const std::size_t index = span - std::size_t(degree) + std::size_t(j);
            if (index >= poleCount) return false;
            const double w = weight(index);
            d[std::size_t(j)] = {segment.poles[index].point.X() * w,
                                 segment.poles[index].point.Y() * w, w};
        }
        for (int r = 1; r <= degree; ++r)
            for (int j = degree; j >= r; --j) {
                const std::size_t i = span - std::size_t(degree) + std::size_t(j);
                const double denominator = flat[i + std::size_t(degree) + 1 - std::size_t(r)] - flat[i];
                const double alpha = denominator > 0 ? (t - flat[i]) / denominator : 0.0;
                auto& a = d[std::size_t(j - 1)];
                auto& b = d[std::size_t(j)];
                b = {a[0] + alpha * (b[0] - a[0]), a[1] + alpha * (b[1] - a[1]),
                     a[2] + alpha * (b[2] - a[2])};
            }
        const auto& h = d[std::size_t(degree)];
        if (h[2] <= 0 || !std::isfinite(h[2])) return false;
        point = gp_Pnt2d(h[0] / h[2], h[1] / h[2]);
        // Derivative of degree-(p-1) control polygon, homogeneous quotient.
        if (degree >= 1) {
            std::vector<std::array<double, 3>> e;
            e.resize(std::size_t(degree));
            for (int j = 0; j < degree; ++j) {
                const std::size_t i0 = span - std::size_t(degree) + std::size_t(j);
                const std::size_t i1 = i0 + 1;
                if (i1 >= poleCount) return false;
                const double denominator = flat[i1 + std::size_t(degree)] - flat[i1];
                if (denominator <= 0) return false;
                const double factor = degree / denominator;
                const double w0 = weight(i0), w1 = weight(i1);
                e[std::size_t(j)] = {
                    factor * (segment.poles[i1].point.X() * w1 - segment.poles[i0].point.X() * w0),
                    factor * (segment.poles[i1].point.Y() * w1 - segment.poles[i0].point.Y() * w0),
                    factor * (w1 - w0)};
            }
            for (int r = 1; r < degree; ++r)
                for (int j = degree - 1; j >= r; --j) {
                    const std::size_t i = span - std::size_t(degree) + std::size_t(j) + 1;
                    const double denominator =
                        flat[i + std::size_t(degree) - std::size_t(r)] - flat[i];
                    const double alpha = denominator > 0 ? (t - flat[i]) / denominator : 0.0;
                    auto& a = e[std::size_t(j - 1)];
                    auto& b = e[std::size_t(j)];
                    b = {a[0] + alpha * (b[0] - a[0]), a[1] + alpha * (b[1] - a[1]),
                         a[2] + alpha * (b[2] - a[2])};
                }
            const auto& g = e[std::size_t(degree - 1)];
            derivative = gp_Pnt2d((g[0] * h[2] - h[0] * g[2]) / (h[2] * h[2]),
                                  (g[1] * h[2] - h[1] * g[2]) / (h[2] * h[2]));
        } else {
            derivative = gp_Pnt2d(0, 0);
        }
        return std::isfinite(point.X()) && std::isfinite(point.Y())
            && std::isfinite(derivative.X()) && std::isfinite(derivative.Y());
    } catch (...) { return false; }
}

// Signed distance from a point to the explicit axis line; linear in the point,
// so B-spline convex-hull signs bound the whole curve exactly.
inline double SplineAxisSignedDistance(const SplineRevolveAxis& axis, const gp_Pnt2d& p) {
    return axis.direction.X() * (p.Y() - axis.origin.Y())
        - axis.direction.Y() * (p.X() - axis.origin.X());
}

} // namespace core3d
