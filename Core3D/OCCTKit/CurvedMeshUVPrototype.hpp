#pragma once

// Curved-mesh UV unwrap candidate kernel — PROTOTYPE ONLY, not production.
//
// Status 2026-09-16: explicitly unqualified for production. Any production
// call-site integration requires frontier review, source-authorized candidate
// admission, native OCCT/OCAF integration (material/UV payload preservation,
// history), UI controls, authored metadata/persistence, retained stool GLB
// checks and guarded iPhone/iPad workflow evidence. Roadmap: B01-curved-uv-gap,
// NAT-uv-seams-curved-unwrap; see docs/B01_CURVED_UV_FEASIBILITY_2026-09-16.md
// (curved-chart fragmentation is the demonstrated finishing gap this kernel
// targets) and docs/NATIVE_CAPABILITY_SOURCE_AUDIT_2026-09-16.md.
//
// Admitted surfaces, bounded on purpose (never guessed from the mesh, never
// silently flattened):
//   * Cylindrical side bands with caller-supplied center/axis/radius and a
//     deterministic seam angle. Exact isometric development (arc length x
//     height) with deliberate per-corner seam duplication.
//   * Toroidal bands are NOT admitted in this increment. A torus has no
//     isometric development (unavoidable areal distortion), so it needs a
//     separate bounded increment with its own explicit distortion budget.
//   * Vertex link: every shared geometric vertex must have exactly one
//     connected incident fan (or exactly two boundary rays on an open rim),
//     validated by reusing EditableMeshTopology.hpp's stdlib analysis.
//     Disconnected bowtie fans reject as NonManifoldVertex.
//
// The kernel never alters input geometry, face order, source IDs or any
// existing material/UV payload: it reads triangles and returns per-triangle-
// corner UVs only. It is not a tessellation, serialization, renderer or
// viewer change, and introduces no library dependency (C++ standard library
// only, reusing the CoherentMeshUVAtlas types and overlap test).

#include "CoherentMeshUVAtlas.hpp"

// Vertex-link admission reuses the existing stdlib topology analysis from
// EditableMeshTopology.hpp (exact-position vertex identity, fan connectivity
// and boundary-ray rules) instead of adding a second topology framework.
#include "EditableMeshTopology.hpp"

namespace shapeyard::uv::prototype {

// Caller-supplied cylindrical band description. No surface class is inferred
// from the mesh; vertices are validated against this exact surface.
struct CylinderBand {
    Point center{0.0, 0.0, 0.0};  // any point on the axis, model units
    Point axis{0.0, 0.0, 1.0};    // direction; normalized internally
    double radius = 0.0;          // model units, must be > 0
    double seamAngle = 0.0;       // radians; cut direction in the deterministic frame
    // Radial fit admission: |distance(vertex, axis) - radius| must be within
    // fitToleranceModelUnits + 1e-9 for every vertex. Absolute model units,
    // never scaled by radius: the future integration derives it from
    // BRep_Tool::Tolerance on the source face's edges/vertices.
    double fitToleranceModelUnits = 1.e-6;
    // Maximum admitted angular span of one triangle around the axis, strictly
    // below pi/2: the no-re-centering seam rule below keeps every shared edge
    // exactly continuous or exactly one developed turn apart only while two
    // corners of one triangle stay strictly within pi of each other, i.e.
    // maxTriangleArc < pi/2. Also bounds chord-vs-arc deviation.
    double maxTriangleArc = 1.5;  // radians, below the strict pi/2 bound
};

enum class CylinderStatus {
    Ok,
    InvalidSettings,      // Settings failed valid()
    InvalidSurface,       // bad center/axis/radius/seam/tolerance parameters
    EmptyInput,
    TooManyTriangles,     // above the retained 4096 admission cap
    NonFiniteCoordinate,  // NaN/inf or |coordinate| > 1e6
    DegenerateTriangle,   // zero-length edge or collapsed area
    NonManifoldEdge,      // edge shared by more than two triangles
    InconsistentWinding,  // shared edge with the same orientation in both uses
    NonManifoldVertex,    // shared geometric vertex has disconnected fans or
                          // a boundary vertex without exactly two boundary rays
    OffSurfaceVertex,     // vertex outside the cylindrical fit tolerance
    ExcessiveAngularSpan, // one triangle spans more than maxTriangleArc
    InternalOverlap,      // developed triangles overlap (or check budget out)
    PackingFailure,       // band cannot be packed into the unit square+ gutters
    DistortionBeyondModel // measured distortion exceeds the analytic model
};

inline const char* cylinderStatusName(CylinderStatus status) noexcept {
    switch (status) {
        case CylinderStatus::Ok: return "Ok";
        case CylinderStatus::InvalidSettings: return "InvalidSettings";
        case CylinderStatus::InvalidSurface: return "InvalidSurface";
        case CylinderStatus::EmptyInput: return "EmptyInput";
        case CylinderStatus::TooManyTriangles: return "TooManyTriangles";
        case CylinderStatus::NonFiniteCoordinate: return "NonFiniteCoordinate";
        case CylinderStatus::DegenerateTriangle: return "DegenerateTriangle";
        case CylinderStatus::NonManifoldEdge: return "NonManifoldEdge";
        case CylinderStatus::InconsistentWinding: return "InconsistentWinding";
        case CylinderStatus::NonManifoldVertex: return "NonManifoldVertex";
        case CylinderStatus::OffSurfaceVertex: return "OffSurfaceVertex";
        case CylinderStatus::ExcessiveAngularSpan: return "ExcessiveAngularSpan";
        case CylinderStatus::InternalOverlap: return "InternalOverlap";
        case CylinderStatus::PackingFailure: return "PackingFailure";
        case CylinderStatus::DistortionBeyondModel: return "DistortionBeyondModel";
    }
    return "Unknown";
}

struct CylinderResult {
    CylinderStatus status = CylinderStatus::Ok;
    int detailIndex = -1;  // triangle/vertex index relevant to a rejection
    Atlas atlas;           // populated only when status == Ok (single chart)
    // Numeric evidence, measured from independent triangle geometry:
    double maxLinearStretch = 1.0;  // max UV edge length / 3D chord length (= chordRelativeMax)
    double maxAreaError = 0.0;      // max |uvArea / (s^2 * area3d) - 1|
    // Stretch reported two ways per edge. chordRelative is developed/3D-chord,
    // tessellation-dependent: the chord under-measures a circumferential arc
    // of angle D by kappa(D) = (D/2)/sin(D/2). metricRelative divides that
    // chord factor out of the circumferential component — developed /
    // hypot(radius*dAngle, dz) from the analytic arc angle — and is 1 for
    // this isometric development regardless of tessellation.
    double chordRelativeMin = 1.0, chordRelativeMax = 1.0;
    double metricRelativeMin = 1.0, metricRelativeMax = 1.0;
    bool ok() const noexcept { return status == CylinderStatus::Ok; }
};

namespace curved_detail {
using namespace shapeyard::uv::detail;

constexpr double kTwoPi = 6.2831853071795865;

// Deterministic orthonormal frame perpendicular to the axis: pick the world
// axis least aligned with it, then cross products. No mesh-derived guessing.
inline void frameFor(const Point& axis, Point& u, Point& v) noexcept {
    const double ax = std::abs(axis[0]), ay = std::abs(axis[1]), az = std::abs(axis[2]);
    Point world = (ax <= ay && ax <= az) ? Point{1.0, 0.0, 0.0}
                : (ay <= az ? Point{0.0, 1.0, 0.0} : Point{0.0, 0.0, 1.0});
    u = cross(axis, world);
    u = divide(u, magnitude(u));
    v = cross(axis, u);
}
}  // namespace curved_detail

// Unwrap a triangulated cylindrical side band onto a single chart packed into
// the unit UV square with the Settings gutter. The u axis is arc length
// (radius * angle from the seam), the v axis is height along the axis.
// Failure returns a status and an empty atlas; the input is never modified.
inline CylinderResult unwrapCylindricalBand(const std::vector<Triangle>& input,
                                            const CylinderBand& surface,
                                            const Settings& settings) noexcept {
    CylinderResult result;
    auto fail = [&](CylinderStatus status, int index) {
        result.status = status;
        result.detailIndex = index;
        result.atlas = {};
        result.maxLinearStretch = 1.0;
        result.maxAreaError = 0.0;
        result.chordRelativeMin = 1.0;
        result.chordRelativeMax = 1.0;
        result.metricRelativeMin = 1.0;
        result.metricRelativeMax = 1.0;
        return result;
    };
    if (!settings.valid()) return fail(CylinderStatus::InvalidSettings, -1);
    if (input.empty()) return fail(CylinderStatus::EmptyInput, -1);
    if (input.size() > 4096) return fail(CylinderStatus::TooManyTriangles, -1);
    try {
        using namespace curved_detail;
        for (double c : surface.center) if (!std::isfinite(c)) return fail(CylinderStatus::InvalidSurface, -1);
        for (double c : surface.axis) if (!std::isfinite(c)) return fail(CylinderStatus::InvalidSurface, -1);
        const double axisLength = magnitude(surface.axis);
        if (axisLength <= 1.e-12) return fail(CylinderStatus::InvalidSurface, -1);
        if (!std::isfinite(surface.radius) || surface.radius <= 1.e-9 || surface.radius > 1.e6)
            return fail(CylinderStatus::InvalidSurface, -1);
        if (!std::isfinite(surface.seamAngle)) return fail(CylinderStatus::InvalidSurface, -1);
        if (!std::isfinite(surface.fitToleranceModelUnits)
            || surface.fitToleranceModelUnits <= 0.0)
            return fail(CylinderStatus::InvalidSurface, -1);
        // Strictly below pi/2: at or above it the no-re-centering placement
        // can tear a shared edge (per-endpoint turn offsets), see the comment
        // at the unwrap below.
        if (!std::isfinite(surface.maxTriangleArc) || surface.maxTriangleArc <= 0.0
            || surface.maxTriangleArc >= 1.5707963267948966)  // pi/2
            return fail(CylinderStatus::InvalidSurface, -1);

        const Point axis = divide(surface.axis, axisLength);
        Point frameU, frameV;
        frameFor(axis, frameU, frameV);
        const double seamCos = std::cos(surface.seamAngle), seamSin = std::sin(surface.seamAngle);
        // Deterministic seam direction in the plane perpendicular to the axis.
        const Point seamDir{frameU[0] * seamCos + frameV[0] * seamSin,
                            frameU[1] * seamCos + frameV[1] * seamSin,
                            frameU[2] * seamCos + frameV[2] * seamSin};
        const Point seamPerp = cross(axis, seamDir);
        const double fitLimit = surface.fitToleranceModelUnits + 1.e-9;

        const int count = static_cast<int>(input.size());
        std::vector<double> areas(count);
        // Per-vertex developed coordinates before seam branch placement:
        // canonical angle in [0, 2*pi) from the seam, plus axis height.
        std::vector<std::array<double, 3>> angles(count);
        std::map<Edge, std::vector<EdgeUse>> edges;
        for (int i = 0; i < count; ++i) {
            const auto& p = input[i].points;
            for (const auto& v : p)
                for (double c : v)
                    if (!std::isfinite(c) || std::abs(c) > 1.e6)
                        return fail(CylinderStatus::NonFiniteCoordinate, i);
            const auto e = sub(p[1], p[0]), f = sub(p[2], p[0]), n = cross(e, f);
            const double length = magnitude(e), area2 = magnitude(n);
            if (!std::isfinite(area2) || length <= 1.e-12 || area2 <= length * length * 1.e-12)
                return fail(CylinderStatus::DegenerateTriangle, i);
            areas[i] = area2 * 0.5;
            for (int k = 0; k < 3; ++k) {
                const Point rel = sub(p[k], surface.center);
                const double height = dot(rel, axis);
                const Point axial{axis[0] * height, axis[1] * height, axis[2] * height};
                const Point radial = sub(rel, axial);
                const double radialLength = magnitude(radial);
                if (std::abs(radialLength - surface.radius) > fitLimit)
                    return fail(CylinderStatus::OffSurfaceVertex, i);
                double theta = std::atan2(dot(radial, seamPerp), dot(radial, seamDir));
                if (theta < 0.0) theta += kTwoPi;
                angles[i][k] = theta;
                Point a = p[k], b = p[(k + 1) % 3];
                if (a == b) return fail(CylinderStatus::DegenerateTriangle, i);
                const bool forward = a < b;
                if (!forward) std::swap(a, b);
                auto& uses = edges[{a, b}];
                uses.push_back({i, forward});
                if (uses.size() > 2) return fail(CylinderStatus::NonManifoldEdge, i);
            }
        }
        for (const auto& entry : edges) {
            const auto& uses = entry.second;
            if (uses.size() == 2 && uses[0].forward == uses[1].forward)
                return fail(CylinderStatus::InconsistentWinding, uses[0].triangle);
        }

        // Seam-aware contiguous unwrap. Each triangle's corners are unwrapped
        // relative to corner 0 with NO re-centering shift: a mean-based
        // whole-turn shift can pick different branches for the two triangles
        // of one seam-crossed quad and put a spurious one-turn discontinuity
        // on their shared diagonal. Without re-centering, every shared edge
        // is either exactly continuous or exactly one developed turn apart —
        // a guarantee that holds only because admission requires
        // maxTriangleArc < pi/2, keeping any two corners of one triangle
        // strictly within pi of each other. Triangles the seam cut passes
        // through keep corners on both sides of the cut: their shared-edge
        // partners receive the duplicated corner values exactly one turn
        // apart. The chart may extend up to one maxTriangleArc beyond the
        // canonical turn at a seam, which the bounds below absorb.
        std::vector<std::array<UV, 3>> raw(count);
        double lowU = INFINITY, highU = -INFINITY, lowV = INFINITY, highV = -INFINITY;
        for (int i = 0; i < count; ++i) {
            const auto& p = input[i].points;
            std::array<double, 3> u;
            for (int k = 0; k < 3; ++k) {
                const double d = std::remainder(angles[i][k] - angles[i][0], kTwoPi);
                if (std::abs(d) > surface.maxTriangleArc)
                    return fail(CylinderStatus::ExcessiveAngularSpan, i);
                u[k] = angles[i][0] + d;
            }
            for (int k = 0; k < 3; ++k) {
                const double arc = u[k] * surface.radius;
                const double height = dot(sub(p[k], surface.center), axis);
                raw[i][k] = {arc, height};
                lowU = std::min(lowU, arc); highU = std::max(highU, arc);
                lowV = std::min(lowV, height); highV = std::max(highV, height);
            }
            const UV& a = raw[i][0]; const UV& b = raw[i][1]; const UV& c = raw[i][2];
            const double uvArea2 = std::abs((b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1]));
            if (!std::isfinite(uvArea2) || uvArea2 <= 1.e-14 * areas[i] * areas[i] * 4.0)
                return fail(CylinderStatus::DegenerateTriangle, i);
        }

        // Vertex-link admission: reuse EditableMeshTopology.hpp's stdlib
        // analysis on a copy (input immutability is preserved). Every other
        // cause it could report (non-finite coordinates, degenerate
        // triangles, third edge uses, inconsistent winding, size) has already
        // been excluded above with its dedicated status, so a NonManifold
        // result here is a vertex-fan violation: a disconnected/bowtie fan at
        // a shared geometric vertex or a boundary vertex without exactly two
        // boundary rays. Its stricter degenerate floor is the only reachable
        // residual and still means the triangle is degenerate. Rejection
        // clears all output, keeping failure atomic.
        const std::atomic_bool linkNotCancelled{false};
        std::vector<std::array<Point, 3>> linkInput;
        linkInput.reserve(input.size());
        for (const auto& triangle : input) linkInput.push_back(triangle.points);
        core3d::meshedit::Topology linkTopology;
        const auto linkResult = core3d::meshedit::Analyze(linkInput, linkTopology,
                                                          linkNotCancelled);
        if (linkResult == core3d::meshedit::TopologyResult::NonManifold)
            return fail(CylinderStatus::NonManifoldVertex, -1);
        if (linkResult != core3d::meshedit::TopologyResult::Ready)
            return fail(CylinderStatus::DegenerateTriangle, -1);

        const double width = highU - lowU, heightExtent = highV - lowV;
        if (!std::isfinite(width) || !std::isfinite(heightExtent)
            || width <= 0.0 || heightExtent <= 1.e-12 * surface.radius)
            return fail(CylinderStatus::PackingFailure, -1);

        // Exact non-overlap verification on normalized developed coordinates,
        // reusing the production atlas's bounded sweep + convex SAT interior
        // test. Overlap or budget exhaustion rejects; nothing partial is
        // published.
        const double extent = std::max(width, heightExtent);
        {
            struct Bounds { int triangle; double xmin, xmax, ymin, ymax; };
            std::vector<Bounds> bounds(count);
            for (int i = 0; i < count; ++i) {
                Bounds b{i, INFINITY, -INFINITY, INFINITY, -INFINITY};
                for (const auto& uv : raw[i]) {
                    const double x = (uv[0] - lowU) / extent, y = (uv[1] - lowV) / extent;
                    b.xmin = std::min(b.xmin, x); b.xmax = std::max(b.xmax, x);
                    b.ymin = std::min(b.ymin, y); b.ymax = std::max(b.ymax, y);
                }
                bounds[i] = b;
            }
            std::sort(bounds.begin(), bounds.end(), [](const Bounds& a, const Bounds& b) {
                return a.xmin != b.xmin ? a.xmin < b.xmin : a.triangle < b.triangle;
            });
            std::size_t comparisons = 0;
            for (std::size_t i = 0; i < bounds.size(); ++i)
                for (std::size_t j = i + 1; j < bounds.size(); ++j) {
                    if (++comparisons > 1000000) return fail(CylinderStatus::InternalOverlap, -1);
                    const auto& a = bounds[i]; const auto& b = bounds[j];
                    if (b.xmin >= a.xmax) break;
                    if (b.ymin >= a.ymax || a.ymin >= b.ymax) continue;
                    std::array<UV, 3> x, y;
                    for (int k = 0; k < 3; ++k) {
                        x[k] = {(raw[a.triangle][k][0] - lowU) / extent,
                                (raw[a.triangle][k][1] - lowV) / extent};
                        y[k] = {(raw[b.triangle][k][0] - lowU) / extent,
                                (raw[b.triangle][k][1] - lowV) / extent};
                    }
                    if (interiorsOverlap(x, y)) return fail(CylinderStatus::InternalOverlap, a.triangle);
                }
        }

        // Single-chart pack into the unit square with the requested gutter on
        // every side, centered, uniform scale (the development is isometric,
        // so one scale serves both axes).
        const double gutter = double(settings.gutterPixels) / settings.resolution;
        const double inner = 1.0 - 2.0 * gutter;
        const double scale = std::min(inner / width, inner / heightExtent);
        if (!std::isfinite(scale) || scale <= 0.0)
            return fail(CylinderStatus::PackingFailure, -1);
        const double originU = (1.0 - width * scale) * 0.5;
        const double originV = (1.0 - heightExtent * scale) * 0.5;

        Atlas atlas;
        atlas.corners.resize(count);
        atlas.chartForTriangle.assign(count, 0);
        double occupancy = 0.0, maxStretch = 1.0, maxAreaError = 0.0;
        double chordMin = INFINITY, chordMax = 0.0, metricMin = INFINITY, metricMax = 0.0;
        for (int i = 0; i < count; ++i) {
            const auto& p = input[i].points;
            double uvArea2 = 0.0;
            for (int k = 0; k < 3; ++k) {
                atlas.corners[i][k] = {originU + (raw[i][k][0] - lowU) * scale,
                                       originV + (raw[i][k][1] - lowV) * scale};
                for (int d = 0; d < 2; ++d)
                    if (!std::isfinite(atlas.corners[i][k][d]))
                        return fail(CylinderStatus::PackingFailure, i);
            }
            // Per-edge distortion measured against the independent 3D chord
            // geometry, validated against the exact analytic chord/arc model.
            for (int k = 0; k < 3; ++k) {
                const int next = (k + 1) % 3;
                const double chord = magnitude(sub(p[k], p[next]));
                const auto& a = atlas.corners[i][k]; const auto& b = atlas.corners[i][next];
                const double developed = std::hypot(a[0] - b[0], a[1] - b[1]) / scale;
                if (!std::isfinite(developed) || developed <= 1.e-14)
                    return fail(CylinderStatus::DistortionBeyondModel, i);
                const double stretch = developed / chord;
                const double dAngle = std::abs(std::remainder(angles[i][next] - angles[i][k], kTwoPi));
                const double arc = surface.radius * dAngle;
                const double halfChord = 2.0 * surface.radius * std::sin(dAngle * 0.5);
                const double dz = std::abs(dot(sub(p[k], p[next]), axis));
                const double expected = std::hypot(arc, dz) / std::hypot(halfChord, dz);
                if (std::abs(stretch - expected) > expected * 1.e-9 + 1.e-12)
                    return fail(CylinderStatus::DistortionBeyondModel, i);
                // Metric-relative stretch: divide out the per-edge chord
                // factor kappa(D) = (D/2)/sin(D/2) of the circumferential
                // component via the analytic arc angle (developed /
                // hypot(radius*dAngle, dz)); axial-only edges have dAngle = 0
                // and measure exactly 1.
                const double metricStretch = developed / std::hypot(arc, dz);
                chordMin = std::min(chordMin, stretch);
                chordMax = std::max(chordMax, stretch);
                metricMin = std::min(metricMin, metricStretch);
                metricMax = std::max(metricMax, metricStretch);
                maxStretch = std::max(maxStretch, stretch);
            }
            const UV& a = atlas.corners[i][0]; const UV& b = atlas.corners[i][1];
            const UV& c = atlas.corners[i][2];
            uvArea2 = std::abs((b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1]));
            occupancy += uvArea2 * 0.5;
            maxAreaError = std::max(maxAreaError,
                std::abs(uvArea2 * 0.5 / (scale * scale * areas[i]) - 1.0));
        }
        if (!std::isfinite(occupancy) || occupancy <= 0.0 || occupancy > 1.0 + 1.e-10)
            return fail(CylinderStatus::PackingFailure, -1);
        atlas.occupancy = occupancy;
        atlas.unitsPerMillimeter = scale;
        atlas.chartCount = 1;

        result.status = CylinderStatus::Ok;
        result.detailIndex = -1;
        result.atlas = std::move(atlas);
        result.maxLinearStretch = maxStretch;
        result.maxAreaError = maxAreaError;
        result.chordRelativeMin = chordMin;
        result.chordRelativeMax = chordMax;
        result.metricRelativeMin = metricMin;
        result.metricRelativeMax = metricMax;
        return result;
    } catch (...) {
        return fail(CylinderStatus::PackingFailure, -1);
    }
}

}  // namespace shapeyard::uv::prototype
