#pragma once

// Toroidal UV unwrap candidate kernel — PROTOTYPE ONLY, not production.
//
// Status 2026-09-16: explicitly unqualified for production, same admission
// class as CurvedMeshUVPrototype.hpp. Any production call-site integration
// requires frontier review, source-authorized candidate admission, native
// OCCT/OCAF integration (material/UV payload preservation, history), UI
// controls, authored metadata/persistence, retained stool GLB checks and
// guarded iPhone/iPad workflow evidence. Roadmap: B01-curved-uv-gap,
// NAT-uv-seams-curved-unwrap; see docs/B01_CURVED_UV_FEASIBILITY_2026-09-16.md
// and the cylindrical correction review that scoped this increment
// (root-curved-uv-correction-review/review.md, 2026-09-16). The cylinder
// candidate and the production atlas are untouched by this file.
//
// Admitted surface, bounded on purpose (never guessed from the mesh, never
// silently flattened): a regular ring torus only, fully caller-supplied as
// center/axis/major radius R/minor radius r with R > r > 0, both periodic
// seam angles and explicit finite distortion budgets. Self-intersecting
// (spindle, r >= R) and degenerate (r <= 0) tori reject at admission. Every
// vertex is validated against this exact surface; any rejection returns an
// empty atlas. Triangle order, source IDs, input bytes and any existing
// material/UV payload are never altered: the kernel reads triangles and
// returns per-triangle-corner UVs only. No welding: vertex identity stays
// exact-position via the reused EditableMeshTopology.hpp analysis.
//
// A torus has no isometric rectangular development. With toroidal angle u
// (around the axis) and poloidal angle v (around the tube), the first
// fundamental form is
//     ds^2 = (R + r*cos(v))^2 du^2 + r^2 dv^2.
// This kernel develops the parameter rectangle (x, y) = (R*u, r*v). Relative
// to the true surface the u direction is scaled by lambda(v) =
// R / (R + r*cos(v)) and the v direction is isometric; the area scale is
// lambda(v). lambda ranges over [R/(R+r), R/(R-r)]: unavoidable expansion
// (> 1) on the inner side near v = pi and unavoidable contraction (< 1) on
// the outer side near v = 0. The caller's DistortionBudgets bound the
// measured per-edge metric-relative stretch (developed length with the
// per-edge chord factor kappa(D) = (D/2)/sin(D/2) divided out via the
// analytic arc angles) on both sides and the per-triangle area error; the
// tessellation-dependent developed/chord value is reported alongside as
// evidence but never graded. Distortion is measured from the developed
// output against the independent 3D chord geometry and cross-checked against
// this analytic metric model per edge; exceeding a caller budget rejects the
// whole atlas atomically (DistortionBeyondBudget). Budgets are caller-owned:
// validated once at admission, never adjusted after measuring output.

// Reuses the curved-kernel deterministic frame, the production atlas types
// (Point/UV/Triangle/Settings/Atlas), the bounded non-overlap helper and the
// stdlib vertex-link topology admission. No second framework is added.
#include "CurvedMeshUVPrototype.hpp"

namespace shapeyard::uv::prototype {

// Caller-supplied ring-torus band description. No surface class is inferred
// from the mesh; vertices are validated against this exact surface.
struct TorusBand {
    Point center{0.0, 0.0, 0.0};  // torus center, model units
    Point axis{0.0, 0.0, 1.0};    // symmetry axis direction; normalized internally
    double majorRadius = 0.0;     // R: center to tube-center circle, model units
    double minorRadius = 0.0;     // r: tube radius; admitted only when 0 < r < R
    double seamAngleU = 0.0;      // radians; toroidal cut direction in the deterministic frame
    double seamAngleV = 0.0;      // radians; poloidal cut around the tube (0 = outer equator)
    // Surface fit admission: |distance(vertex, tube-center circle) - r| must
    // be within fitToleranceModelUnits + 1e-9 for every vertex. Absolute
    // model units, never scaled by minorRadius: the future integration
    // derives it from BRep_Tool::Tolerance on the source face's
    // edges/vertices.
    double fitToleranceModelUnits = 1.e-6;
    // Maximum admitted angular span of one triangle in each periodic
    // direction, strictly below pi/2: the no-re-centering seam rule below
    // keeps every shared edge exactly continuous or exactly one developed
    // turn apart per direction only while two corners of one triangle stay
    // strictly within pi of each other, i.e. maxTriangleArc < pi/2. Also
    // bounds chord-vs-arc deviation.
    double maxTriangleArcU = 1.5;  // radians, below the strict pi/2 bound
    double maxTriangleArcV = 1.5;  // radians, below the strict pi/2 bound
};

// Explicit caller distortion budgets for the measured output. The defaults
// are deliberately invalid: the caller must state finite budgets. The kernel
// validates them once and never adjusts them after measuring output.
struct DistortionBudgets {
    double minLinearStretch = 0.0;  // admitted lower bound on metric-relative stretch (contraction side), > 0
    double maxLinearStretch = 0.0;  // admitted upper bound on metric-relative stretch (expansion side), >= min
    double maxAreaError = -1.0;     // admitted |uvArea / (scale^2 * area3d) - 1|, >= 0
};

enum class TorusStatus {
    Ok,
    InvalidSettings,       // Settings failed valid()
    InvalidSurface,        // bad center/axis/radii/seam/tolerance/arc parameters, or not a ring torus
    InvalidBudget,         // non-finite or inconsistent distortion budgets
    EmptyInput,
    TooManyTriangles,      // above the retained 4096 admission cap
    NonFiniteCoordinate,   // NaN/inf or |coordinate| > 1e6
    DegenerateTriangle,    // zero-length edge or collapsed area (3D or developed)
    NonManifoldEdge,       // edge shared by more than two triangles
    InconsistentWinding,   // shared edge with the same orientation in both uses
    NonManifoldVertex,     // shared geometric vertex has disconnected fans or
                           // a boundary vertex without exactly two boundary rays
    OffSurfaceVertex,      // vertex outside the toroidal fit tolerance or on the axis
    ExcessiveAngularSpan,  // one triangle spans more than maxTriangleArcU/V
    InternalOverlap,       // developed triangles overlap (or check budget out)
    PackingFailure,        // band cannot be packed into the unit square + gutters
    DistortionBeyondModel, // measured distortion contradicts the analytic metric model
    DistortionBeyondBudget // measured distortion exceeds the caller budgets
};

inline const char* torusStatusName(TorusStatus status) noexcept {
    switch (status) {
        case TorusStatus::Ok: return "Ok";
        case TorusStatus::InvalidSettings: return "InvalidSettings";
        case TorusStatus::InvalidSurface: return "InvalidSurface";
        case TorusStatus::InvalidBudget: return "InvalidBudget";
        case TorusStatus::EmptyInput: return "EmptyInput";
        case TorusStatus::TooManyTriangles: return "TooManyTriangles";
        case TorusStatus::NonFiniteCoordinate: return "NonFiniteCoordinate";
        case TorusStatus::DegenerateTriangle: return "DegenerateTriangle";
        case TorusStatus::NonManifoldEdge: return "NonManifoldEdge";
        case TorusStatus::InconsistentWinding: return "InconsistentWinding";
        case TorusStatus::NonManifoldVertex: return "NonManifoldVertex";
        case TorusStatus::OffSurfaceVertex: return "OffSurfaceVertex";
        case TorusStatus::ExcessiveAngularSpan: return "ExcessiveAngularSpan";
        case TorusStatus::InternalOverlap: return "InternalOverlap";
        case TorusStatus::PackingFailure: return "PackingFailure";
        case TorusStatus::DistortionBeyondModel: return "DistortionBeyondModel";
        case TorusStatus::DistortionBeyondBudget: return "DistortionBeyondBudget";
    }
    return "Unknown";
}

struct TorusResult {
    TorusStatus status = TorusStatus::Ok;
    int detailIndex = -1;  // triangle/vertex index relevant to a rejection
    Atlas atlas;           // populated only when status == Ok (single chart)
    // Numeric evidence, measured from independent triangle geometry:
    double minLinearStretch = 1.0;  // min UV edge length / 3D chord length (contraction side)
    double maxLinearStretch = 1.0;  // max UV edge length / 3D chord length (expansion side)
    double maxAreaError = 0.0;      // max |uvArea / (s^2 * area3d) - 1|
    // Stretch reported two ways per edge. chordRelative is developed/3D-chord
    // (mirrors min/maxLinearStretch), tessellation-dependent: the chord
    // under-measures an arc of angle D by kappa(D) = (D/2)/sin(D/2).
    // metricRelative divides that chord factor out per direction via the
    // analytic arc angles — developed / hypot((R + r*cos(vMid))*du, r*dv) —
    // and sits inside [R/(R+r), R/(R-r)] regardless of tessellation. Caller
    // budgets grade metricRelative, never the chord-relative value.
    double chordRelativeMin = 1.0, chordRelativeMax = 1.0;
    double metricRelativeMin = 1.0, metricRelativeMax = 1.0;
    bool ok() const noexcept { return status == TorusStatus::Ok; }
};

// Unwrap a triangulated ring-torus band onto a single rectangular chart
// packed into the unit UV square with the Settings gutter. The u axis is the
// developed toroidal arc R*u from the u seam, the v axis is the developed
// poloidal arc r*v from the v seam. Failure returns a status and an empty
// atlas; the input and the budgets are never modified.
inline TorusResult unwrapToroidalBand(const std::vector<Triangle>& input,
                                      const TorusBand& surface,
                                      const DistortionBudgets& budgets,
                                      const Settings& settings) noexcept {
    TorusResult result;
    auto fail = [&](TorusStatus status, int index) {
        result.status = status;
        result.detailIndex = index;
        result.atlas = {};
        result.minLinearStretch = 1.0;
        result.maxLinearStretch = 1.0;
        result.maxAreaError = 0.0;
        result.chordRelativeMin = 1.0;
        result.chordRelativeMax = 1.0;
        result.metricRelativeMin = 1.0;
        result.metricRelativeMax = 1.0;
        return result;
    };
    if (!settings.valid()) return fail(TorusStatus::InvalidSettings, -1);
    if (input.empty()) return fail(TorusStatus::EmptyInput, -1);
    if (input.size() > 4096) return fail(TorusStatus::TooManyTriangles, -1);
    try {
        using namespace curved_detail;
        for (double c : surface.center) if (!std::isfinite(c)) return fail(TorusStatus::InvalidSurface, -1);
        for (double c : surface.axis) if (!std::isfinite(c)) return fail(TorusStatus::InvalidSurface, -1);
        const double axisLength = magnitude(surface.axis);
        if (axisLength <= 1.e-12) return fail(TorusStatus::InvalidSurface, -1);
        // Regular ring torus only: 0 < r < R, both bounded like coordinates.
        if (!std::isfinite(surface.majorRadius) || surface.majorRadius <= 1.e-9
            || surface.majorRadius > 1.e6)
            return fail(TorusStatus::InvalidSurface, -1);
        if (!std::isfinite(surface.minorRadius) || surface.minorRadius <= 1.e-9
            || surface.minorRadius >= surface.majorRadius)
            return fail(TorusStatus::InvalidSurface, -1);
        if (!std::isfinite(surface.seamAngleU) || !std::isfinite(surface.seamAngleV))
            return fail(TorusStatus::InvalidSurface, -1);
        if (!std::isfinite(surface.fitToleranceModelUnits)
            || surface.fitToleranceModelUnits <= 0.0)
            return fail(TorusStatus::InvalidSurface, -1);
        // Strictly below pi/2: at or above it the no-re-centering placement
        // can tear a shared edge (per-endpoint turn offsets), see the comment
        // at the unwrap below.
        if (!std::isfinite(surface.maxTriangleArcU) || surface.maxTriangleArcU <= 0.0
            || surface.maxTriangleArcU >= 1.5707963267948966)  // pi/2
            return fail(TorusStatus::InvalidSurface, -1);
        if (!std::isfinite(surface.maxTriangleArcV) || surface.maxTriangleArcV <= 0.0
            || surface.maxTriangleArcV >= 1.5707963267948966)  // pi/2
            return fail(TorusStatus::InvalidSurface, -1);
        // Explicit finite budgets, validated once here and never adjusted.
        if (!std::isfinite(budgets.minLinearStretch) || budgets.minLinearStretch <= 0.0
            || !std::isfinite(budgets.maxLinearStretch)
            || budgets.maxLinearStretch < budgets.minLinearStretch
            || !std::isfinite(budgets.maxAreaError) || budgets.maxAreaError < 0.0)
            return fail(TorusStatus::InvalidBudget, -1);

        const double bigR = surface.majorRadius, smallR = surface.minorRadius;
        const Point axis = divide(surface.axis, axisLength);
        Point frameU, frameV;
        frameFor(axis, frameU, frameV);
        const double fitLimit = surface.fitToleranceModelUnits + 1.e-9;
        auto wrapTurn = [&](double angle) {
            angle = std::fmod(angle, kTwoPi);
            if (angle < 0.0) angle += kTwoPi;
            return angle;
        };

        const int count = static_cast<int>(input.size());
        std::vector<double> areas(count);
        // Per-vertex canonical angles in [0, 2*pi) from each seam, before
        // seam branch placement.
        std::vector<std::array<double, 3>> anglesU(count), anglesV(count);
        std::map<Edge, std::vector<EdgeUse>> edges;
        for (int i = 0; i < count; ++i) {
            const auto& p = input[i].points;
            for (const auto& v : p)
                for (double c : v)
                    if (!std::isfinite(c) || std::abs(c) > 1.e6)
                        return fail(TorusStatus::NonFiniteCoordinate, i);
            const auto e = sub(p[1], p[0]), f = sub(p[2], p[0]), n = cross(e, f);
            const double length = magnitude(e), area2 = magnitude(n);
            if (!std::isfinite(area2) || length <= 1.e-12 || area2 <= length * length * 1.e-12)
                return fail(TorusStatus::DegenerateTriangle, i);
            areas[i] = area2 * 0.5;
            for (int k = 0; k < 3; ++k) {
                const Point rel = sub(p[k], surface.center);
                const double height = dot(rel, axis);
                const Point axial{axis[0] * height, axis[1] * height, axis[2] * height};
                const Point radial = sub(rel, axial);
                const double rho = magnitude(radial);
                // On-axis vertices have no defined toroidal angle and cannot
                // satisfy the ring-torus fit anyway.
                if (rho <= 1.e-12 * bigR) return fail(TorusStatus::OffSurfaceVertex, i);
                // Distance to the tube-center circle; must equal r within fit.
                const double tube = std::hypot(rho - bigR, height);
                if (std::abs(tube - smallR) > fitLimit)
                    return fail(TorusStatus::OffSurfaceVertex, i);
                anglesU[i][k] = wrapTurn(std::atan2(dot(radial, frameV), dot(radial, frameU))
                                         - surface.seamAngleU);
                // Poloidal angle: radial tube component rho - R vs axial lift.
                anglesV[i][k] = wrapTurn(std::atan2(height, rho - bigR) - surface.seamAngleV);
                Point a = p[k], b = p[(k + 1) % 3];
                if (a == b) return fail(TorusStatus::DegenerateTriangle, i);
                const bool forward = a < b;
                if (!forward) std::swap(a, b);
                auto& uses = edges[{a, b}];
                uses.push_back({i, forward});
                if (uses.size() > 2) return fail(TorusStatus::NonManifoldEdge, i);
            }
        }
        for (const auto& entry : edges) {
            const auto& uses = entry.second;
            if (uses.size() == 2 && uses[0].forward == uses[1].forward)
                return fail(TorusStatus::InconsistentWinding, uses[0].triangle);
        }

        // Seam-aware contiguous unwrap in BOTH periodic directions. Each
        // triangle's corners are unwrapped relative to corner 0 with NO
        // re-centering shift: a mean-based whole-turn shift (as the
        // cylindrical kernel used before the 2026-09-17 correction) can pick different branches for the two
        // triangles of one seam-crossed quad and tear their shared diagonal
        // by a full turn. Without re-centering, every shared edge is either
        // exactly continuous or exactly one developed turn apart per
        // direction; the chart may extend up to one maxTriangleArc beyond
        // the canonical turn at a seam, which the bounds below absorb. This
        // no-tear guarantee depends on the strict maxTriangleArcU/V < pi/2
        // admission bound: two corners of one triangle then stay strictly
        // within pi of each other, so nearest-representative placement
        // cannot diverge per endpoint on a shared edge.
        std::vector<std::array<UV, 3>> raw(count);
        std::vector<std::array<double, 3>> finalU(count), finalV(count);
        double lowU = INFINITY, highU = -INFINITY, lowV = INFINITY, highV = -INFINITY;
        for (int i = 0; i < count; ++i) {
            std::array<double, 3> u, v;
            for (int k = 0; k < 3; ++k) {
                const double du = std::remainder(anglesU[i][k] - anglesU[i][0], kTwoPi);
                const double dv = std::remainder(anglesV[i][k] - anglesV[i][0], kTwoPi);
                if (std::abs(du) > surface.maxTriangleArcU
                    || std::abs(dv) > surface.maxTriangleArcV)
                    return fail(TorusStatus::ExcessiveAngularSpan, i);
                u[k] = anglesU[i][0] + du;
                v[k] = anglesV[i][0] + dv;
            }
            for (int k = 0; k < 3; ++k) {
                finalU[i][k] = u[k];
                finalV[i][k] = v[k];
                const double arc = finalU[i][k] * bigR;
                const double lift = finalV[i][k] * smallR;
                raw[i][k] = {arc, lift};
                lowU = std::min(lowU, arc); highU = std::max(highU, arc);
                lowV = std::min(lowV, lift); highV = std::max(highV, lift);
            }
            const UV& a = raw[i][0]; const UV& b = raw[i][1]; const UV& c = raw[i][2];
            const double uvArea2 = std::abs((b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1]));
            if (!std::isfinite(uvArea2) || uvArea2 <= 1.e-14 * areas[i] * areas[i] * 4.0)
                return fail(TorusStatus::DegenerateTriangle, i);
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
            return fail(TorusStatus::NonManifoldVertex, -1);
        if (linkResult != core3d::meshedit::TopologyResult::Ready)
            return fail(TorusStatus::DegenerateTriangle, -1);

        const double width = highU - lowU, heightExtent = highV - lowV;
        if (!std::isfinite(width) || !std::isfinite(heightExtent)
            || width <= 1.e-12 * bigR || heightExtent <= 1.e-12 * smallR)
            return fail(TorusStatus::PackingFailure, -1);

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
                    if (++comparisons > 1000000) return fail(TorusStatus::InternalOverlap, -1);
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
                    if (interiorsOverlap(x, y)) return fail(TorusStatus::InternalOverlap, a.triangle);
                }
        }

        // Single-chart pack into the unit square with the requested gutter on
        // every side, centered, uniform scale (distortion evidence is
        // scale-invariant, so one scale serves both axes).
        const double gutter = double(settings.gutterPixels) / settings.resolution;
        const double inner = 1.0 - 2.0 * gutter;
        const double scale = std::min(inner / width, inner / heightExtent);
        if (!std::isfinite(scale) || scale <= 0.0)
            return fail(TorusStatus::PackingFailure, -1);
        const double originU = (1.0 - width * scale) * 0.5;
        const double originV = (1.0 - heightExtent * scale) * 0.5;

        // Closed-form torus point at absolute angles, used only to cross-check
        // the measured distortion against the analytic metric model.
        auto modelPoint = [&](double absU, double absV) {
            const double cu = std::cos(absU), su = std::sin(absU);
            const double ring = bigR + smallR * std::cos(absV);
            const double lift = smallR * std::sin(absV);
            return Point{surface.center[0] + ring * (frameU[0] * cu + frameV[0] * su) + lift * axis[0],
                         surface.center[1] + ring * (frameU[1] * cu + frameV[1] * su) + lift * axis[1],
                         surface.center[2] + ring * (frameU[2] * cu + frameV[2] * su) + lift * axis[2]};
        };

        Atlas atlas;
        atlas.corners.resize(count);
        atlas.chartForTriangle.assign(count, 0);
        double occupancy = 0.0, minStretch = INFINITY, maxStretch = 0.0, maxAreaError = 0.0;
        double minMetric = INFINITY, maxMetric = 0.0;
        for (int i = 0; i < count; ++i) {
            const auto& p = input[i].points;
            double uvArea2 = 0.0;
            for (int k = 0; k < 3; ++k) {
                atlas.corners[i][k] = {originU + (raw[i][k][0] - lowU) * scale,
                                       originV + (raw[i][k][1] - lowV) * scale};
                for (int d = 0; d < 2; ++d)
                    if (!std::isfinite(atlas.corners[i][k][d]))
                        return fail(TorusStatus::PackingFailure, i);
            }
            // Per-edge distortion measured against the independent 3D chord
            // geometry, cross-checked against the analytic metric model
            // ds^2 = (R + r*cos(v))^2 du^2 + r^2 dv^2 evaluated at the
            // extracted angles, then graded against the caller budgets.
            for (int k = 0; k < 3; ++k) {
                const int next = (k + 1) % 3;
                const double chord = magnitude(sub(p[k], p[next]));
                const auto& a = atlas.corners[i][k]; const auto& b = atlas.corners[i][next];
                const double developed = std::hypot(a[0] - b[0], a[1] - b[1]) / scale;
                if (!std::isfinite(developed) || developed <= 1.e-14)
                    return fail(TorusStatus::DistortionBeyondModel, i);
                const double stretch = developed / chord;
                const double du = finalU[i][next] - finalU[i][k];
                const double dv = finalV[i][next] - finalV[i][k];
                const double modelLength = std::hypot(bigR * du, smallR * dv);
                const double modelChord = magnitude(sub(
                    modelPoint(surface.seamAngleU + finalU[i][k],
                               surface.seamAngleV + finalV[i][k]),
                    modelPoint(surface.seamAngleU + finalU[i][next],
                               surface.seamAngleV + finalV[i][next])));
                if (!std::isfinite(modelChord) || modelChord <= 1.e-14)
                    return fail(TorusStatus::DistortionBeyondModel, i);
                const double expected = modelLength / modelChord;
                // Endpoints may sit fitLimit off the exact surface, moving the
                // measured chord by at most 2*fitLimit; allow that plus rounding.
                const double tolerance = expected * (1.e-9 + 4.0 * fitLimit / chord) + 1.e-12;
                if (std::abs(stretch - expected) > tolerance)
                    return fail(TorusStatus::DistortionBeyondModel, i);
                // Metric-relative stretch: divide out the per-edge chord
                // factor kappa(D) = (D/2)/sin(D/2) per direction via the
                // analytic arc angles; combined per edge this is developed /
                // hypot((R + r*cos(vMid))*du, r*dv), which sits inside the
                // metric band [R/(R+r), R/(R-r)] regardless of tessellation.
                // Caller budgets grade this quantity, not the
                // tessellation-dependent chord-relative value.
                const double vMid = 0.5 * (finalV[i][k] + finalV[i][next]);
                const double metricLength = std::hypot(
                    (bigR + smallR * std::cos(vMid)) * du, smallR * dv);
                const double metricStretch = developed / metricLength;
                if (metricStretch < budgets.minLinearStretch
                    || metricStretch > budgets.maxLinearStretch)
                    return fail(TorusStatus::DistortionBeyondBudget, i);
                minStretch = std::min(minStretch, stretch);
                maxStretch = std::max(maxStretch, stretch);
                minMetric = std::min(minMetric, metricStretch);
                maxMetric = std::max(maxMetric, metricStretch);
            }
            const UV& a = atlas.corners[i][0]; const UV& b = atlas.corners[i][1];
            const UV& c = atlas.corners[i][2];
            uvArea2 = std::abs((b[0]-a[0])*(c[1]-a[1]) - (c[0]-a[0])*(b[1]-a[1]));
            occupancy += uvArea2 * 0.5;
            const double areaError = std::abs(uvArea2 * 0.5 / (scale * scale * areas[i]) - 1.0);
            if (!std::isfinite(areaError) || areaError > budgets.maxAreaError)
                return fail(TorusStatus::DistortionBeyondBudget, i);
            maxAreaError = std::max(maxAreaError, areaError);
        }
        if (!std::isfinite(occupancy) || occupancy <= 0.0 || occupancy > 1.0 + 1.e-10)
            return fail(TorusStatus::PackingFailure, -1);
        atlas.occupancy = occupancy;
        atlas.unitsPerMillimeter = scale;
        atlas.chartCount = 1;

        result.status = TorusStatus::Ok;
        result.detailIndex = -1;
        result.atlas = std::move(atlas);
        result.minLinearStretch = minStretch;
        result.maxLinearStretch = maxStretch;
        result.maxAreaError = maxAreaError;
        result.chordRelativeMin = minStretch;
        result.chordRelativeMax = maxStretch;
        result.metricRelativeMin = minMetric;
        result.metricRelativeMax = maxMetric;
        return result;
    } catch (...) {
        return fail(TorusStatus::PackingFailure, -1);
    }
}

}  // namespace shapeyard::uv::prototype
