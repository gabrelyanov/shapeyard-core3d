// Standalone tests for the ToroidalMeshUVPrototype candidate kernel.
// Ordinary C++ test program, no framework, stdlib only. Compile and run:
//   /usr/bin/clang++ -std=c++17 -Wall -Wextra -Werror -O2 \
//       ToroidalMeshUVPrototypeTests.cpp -o ToroidalMeshUVPrototypeTests
// Expected metrics are derived independently from the generated analytic
// geometry (test-side angle inversion and closed-form chord/area math, never
// from kernel internals), covering contraction as well as expansion.
// Analytic tolerances are stated next to each assertion before grading; this
// is candidate-kernel evidence, not a benchmark pass (no B01 claim).

#include "../Core3D/OCCTKit/ToroidalMeshUVPrototype.hpp"

#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <set>

using namespace shapeyard::uv;
using namespace shapeyard::uv::prototype;
using shapeyard::uv::detail::sub;
using shapeyard::uv::detail::cross;
using shapeyard::uv::detail::magnitude;

namespace {

int failures = 0;
int checks = 0;

void check(bool condition, const char* name) {
    ++checks;
    if (!condition) {
        ++failures;
        std::printf("FAIL: %s\n", name);
    }
}

bool nearly(double a, double b, double relTol, double absTol = 0.0) {
    return std::abs(a - b) <= relTol * std::max(std::abs(a), std::abs(b)) + absTol;
}

constexpr double kTwoPi = 6.2831853071795865;

// Closed ring-torus band: su segments around the axis, sv segments around the
// tube, outward winding (counterclockwise toroidal then poloidal order gives
// (b-a)x(c-a) along the outward normal r(R+r cos v)(cos v * radial + sin v *
// axis)). Both directions are periodic. Deterministic vertex order.
std::vector<Triangle> makeTorusBand(int su, int sv, double bigR, double smallR) {
    auto vertex = [&](int s, int j) {
        const double u = kTwoPi * s / su, v = kTwoPi * j / sv;
        const double ring = bigR + smallR * std::cos(v);
        return Point{ring * std::cos(u), ring * std::sin(u), smallR * std::sin(v)};
    };
    std::vector<Triangle> mesh;
    for (int s = 0; s < su; ++s) {
        const int s1 = (s + 1) % su;
        for (int j = 0; j < sv; ++j) {
            const int j1 = (j + 1) % sv;
            const Point a = vertex(s, j), b = vertex(s1, j);
            const Point c = vertex(s1, j1), d = vertex(s, j1);
            // Outward-facing diagonal split, consistent across the band.
            mesh.push_back(Triangle{{a, b, c}});
            mesh.push_back(Triangle{{a, c, d}});
        }
    }
    return mesh;
}

// Open toroidal patch: periodic in u, but v spans only [vLow, vHigh], leaving
// two open poloidal rims. Boundary vertices get exactly two boundary rays.
std::vector<Triangle> makeTorusOpen(int su, int rows, double bigR, double smallR,
                                    double vLow, double vHigh) {
    auto vertex = [&](int s, int j) {
        const double u = kTwoPi * s / su, v = vLow + (vHigh - vLow) * j / rows;
        const double ring = bigR + smallR * std::cos(v);
        return Point{ring * std::cos(u), ring * std::sin(u), smallR * std::sin(v)};
    };
    std::vector<Triangle> mesh;
    for (int s = 0; s < su; ++s) {
        const int s1 = (s + 1) % su;
        for (int j = 0; j < rows; ++j) {
            const Point a = vertex(s, j), b = vertex(s1, j);
            const Point c = vertex(s1, j + 1), d = vertex(s, j + 1);
            mesh.push_back(Triangle{{a, b, c}});
            mesh.push_back(Triangle{{a, c, d}});
        }
    }
    return mesh;
}

// Rotation about a unit axis (Rodrigues), then translation.
Point transform(const Point& p, const Point& axis, double angle, const Point& offset) {
    const double c = std::cos(angle), s = std::sin(angle);
    const double d = axis[0] * p[0] + axis[1] * p[1] + axis[2] * p[2];
    Point cross{axis[1] * p[2] - axis[2] * p[1],
                axis[2] * p[0] - axis[0] * p[2],
                axis[0] * p[1] - axis[1] * p[0]};
    return {p[0] * c + cross[0] * s + axis[0] * d * (1 - c) + offset[0],
            p[1] * c + cross[1] * s + axis[1] * d * (1 - c) + offset[1],
            p[2] * c + cross[2] * s + axis[2] * d * (1 - c) + offset[2]};
}

Settings defaultSettings() {
    Settings s;
    s.resolution = 1024;
    s.gutterPixels = 8;
    return s;
}

TorusBand defaultBand(double bigR, double smallR) {
    TorusBand band;
    band.center = {0.0, 0.0, 0.0};
    band.axis = {0.0, 0.0, 1.0};
    band.majorRadius = bigR;
    band.minorRadius = smallR;
    band.seamAngleU = 0.0;
    band.seamAngleV = 0.0;
    return band;
}

// Stated before grading: for R=30, r=10 the metric lambda(v) = R/(R+r cos v)
// ranges over [0.75, 1.5]; discretization (chord undermeasures arc) lifts the
// expansion side slightly above 1.5. These budgets admit that with margin.
DistortionBudgets generousBudgets() {
    DistortionBudgets b;
    b.minLinearStretch = 0.5;
    b.maxLinearStretch = 2.0;
    b.maxAreaError = 1.0;
    return b;
}

void expectRejection(const char* name, const std::vector<Triangle>& input,
                     const TorusBand& band, const DistortionBudgets& budgets,
                     const Settings& settings, TorusStatus expected) {
    const TorusResult r = unwrapToroidalBand(input, band, budgets, settings);
    bool ok = r.status == expected;
    // Failed-input atomicity: no partial atlas or misleading numbers.
    ok = ok && !r.ok() && r.atlas.corners.empty() && r.atlas.chartForTriangle.empty()
        && r.atlas.chartCount == 0 && r.atlas.occupancy == 0.0
        && r.atlas.unitsPerMillimeter == 0.0
        && r.minLinearStretch == 1.0 && r.maxLinearStretch == 1.0 && r.maxAreaError == 0.0
        && r.chordRelativeMin == 1.0 && r.chordRelativeMax == 1.0
        && r.metricRelativeMin == 1.0 && r.metricRelativeMax == 1.0;
    check(ok, name);
    if (!ok)
        std::printf("  detail: expected %s, got %s (index %d)\n",
                    torusStatusName(expected), torusStatusName(r.status), r.detailIndex);
}

// Independent per-edge/per-triangle distortion evidence, derived test-side
// from the generated geometry: angles are recovered by the test's own
// inversion (never by calling the kernel), developed lengths come from the
// analytic rectangle (R*u, r*v), chords and 3D areas from the raw points.
struct IndependentEvidence {
    double minStretch = 0.0, maxStretch = 0.0, maxAreaError = 0.0;
};
IndependentEvidence measureIndependently(const std::vector<Triangle>& mesh,
                                         double bigR, double smallR) {
    IndependentEvidence ev;
    ev.minStretch = std::numeric_limits<double>::infinity();
    for (const auto& t : mesh) {
        std::array<double, 3> u, v;
        for (int k = 0; k < 3; ++k) {
            const Point& p = t.points[k];
            const double rho = std::hypot(p[0], p[1]);
            u[k] = std::atan2(p[1], p[0]);
            v[k] = std::atan2(p[2], rho - bigR);
        }
        std::array<double, 3> du, dv;
        for (int k = 0; k < 3; ++k) {
            du[k] = std::remainder(u[(k + 1) % 3] - u[k], kTwoPi);
            dv[k] = std::remainder(v[(k + 1) % 3] - v[k], kTwoPi);
        }
        for (int k = 0; k < 3; ++k) {
            const int n = (k + 1) % 3;
            double chord = 0.0;
            for (int d = 0; d < 3; ++d) chord += (t.points[n][d] - t.points[k][d])
                                               * (t.points[n][d] - t.points[k][d]);
            chord = std::sqrt(chord);
            const double developed = std::hypot(bigR * du[k], smallR * dv[k]);
            const double s = developed / chord;
            ev.minStretch = std::min(ev.minStretch, s);
            ev.maxStretch = std::max(ev.maxStretch, s);
        }
        const double devArea = 0.5 * bigR * smallR
            * std::abs(du[0] * dv[1] - du[1] * dv[0]);
        const auto e0 = sub(t.points[1], t.points[0]);
        const auto e1 = sub(t.points[2], t.points[0]);
        const double area3d = 0.5 * magnitude(cross(e0, e1));
        ev.maxAreaError = std::max(ev.maxAreaError, std::abs(devArea / area3d - 1.0));
    }
    return ev;
}

// Shared-edge continuity/duplication census from independent shared-edge
// geometry: both uses of every shared edge must map to identical UV endpoints
// (continuous), or differ by exactly one developed turn in u (turnX) or v
// (turnY) when the seam passes between them. Anything else is torn.
void censusSharedEdges(const char* label, const std::vector<Triangle>& mesh,
                       const TorusResult& r, double bigR, double smallR,
                       int expectDupU, int expectDupV, bool assertCounts) {
    std::map<std::pair<Point, Point>, std::vector<std::array<UV, 2>>> edgeUV;
    for (std::size_t i = 0; i < mesh.size(); ++i)
        for (int k = 0; k < 3; ++k) {
            Point a = mesh[i].points[k], b = mesh[i].points[(k + 1) % 3];
            UV ua = r.atlas.corners[i][k], ub = r.atlas.corners[i][(k + 1) % 3];
            if (b < a) { std::swap(a, b); std::swap(ua, ub); }
            edgeUV[{a, b}].push_back({ua, ub});
        }
    int continuous = 0, duplicatedU = 0, duplicatedV = 0, broken = 0;
    const double turnX = kTwoPi * bigR * r.atlas.unitsPerMillimeter;
    const double turnY = kTwoPi * smallR * r.atlas.unitsPerMillimeter;
    for (const auto& entry : edgeUV) {
        if (entry.second.size() != 2) continue;  // boundary edges: none in a closed band
        const auto& x = entry.second[0]; const auto& y = entry.second[1];
        // Endpoints are stored in the same geometric order in both uses.
        const double dx = std::abs(x[0][0] - y[0][0]) + std::abs(x[1][0] - y[1][0]);
        const double dy = std::abs(x[0][1] - y[0][1]) + std::abs(x[1][1] - y[1][1]);
        if (dx <= 1.e-9 * turnX && dy <= 1.e-9 * turnY) ++continuous;
        else if (nearly(dx, 2.0 * turnX, 1.e-9, 1.e-12) && dy <= 1.e-9 * turnY) ++duplicatedU;
        else if (nearly(dy, 2.0 * turnY, 1.e-9, 1.e-12) && dx <= 1.e-9 * turnX) ++duplicatedV;
        else ++broken;
    }
    std::printf("%s: shared edges continuous=%d seamU=%d seamV=%d broken=%d\n",
                label, continuous, duplicatedU, duplicatedV, broken);
    check(broken == 0, "seams: no torn shared edges");
    if (assertCounts) {
        check(duplicatedU == expectDupU, "seams: exactly the u-seam column is duplicated");
        check(duplicatedV == expectDupV, "seams: exactly the v-seam row is duplicated");
    }
}

void testAnalyticTorus() {
    const int su = 64, sv = 32;
    const double bigR = 30.0, smallR = 10.0;
    const auto mesh = makeTorusBand(su, sv, bigR, smallR);
    const DistortionBudgets budgets = generousBudgets();
    const TorusResult r = unwrapToroidalBand(mesh, defaultBand(bigR, smallR),
                                             budgets, defaultSettings());
    check(r.ok(), "torus: status Ok");
    if (!r.ok()) {
        std::printf("  detail: status %s index %d\n", torusStatusName(r.status), r.detailIndex);
        return;
    }
    check(r.atlas.chartCount == 1, "torus: single chart");
    check(r.atlas.corners.size() == mesh.size(), "torus: per-triangle corners");
    bool chartsZero = true;
    for (int c : r.atlas.chartForTriangle) chartsZero = chartsZero && c == 0;
    check(chartsZero, "torus: chartForTriangle all 0");

    // Independent analytic expectations, derived from the generated geometry
    // before grading. Tolerance rel 1e-9 covers packed-UV rounding only.
    const IndependentEvidence ev = measureIndependently(mesh, bigR, smallR);
    std::printf("torus: minStretch=%.12f maxStretch=%.12f maxAreaError=%.6f occupancy=%.6f\n",
                r.minLinearStretch, r.maxLinearStretch, r.maxAreaError, r.atlas.occupancy);
    std::printf("torus: independent min=%.12f max=%.12f area=%.6f\n",
                ev.minStretch, ev.maxStretch, ev.maxAreaError);
    check(nearly(r.minLinearStretch, ev.minStretch, 1.e-9, 1.e-12),
          "torus: min stretch matches independent derivation (rel 1e-9)");
    check(nearly(r.maxLinearStretch, ev.maxStretch, 1.e-9, 1.e-12),
          "torus: max stretch matches independent derivation (rel 1e-9)");
    check(nearly(r.maxAreaError, ev.maxAreaError, 1.e-9, 1.e-12),
          "torus: area error matches independent derivation (rel 1e-9)");
    // T3: stretch reported both ways. chordRelative mirrors the developed /
    // 3D-chord evidence; metricRelative divides out the per-edge chord factor
    // and must sit at the tessellation-independent analytic band endpoints
    // [R/(R+r), R/(R-r)] = [0.75, 1.5] within rel 1e-9.
    std::printf("torus: chordRelative=[%.12f, %.12f] metricRelative=[%.12f, %.12f]\n",
                r.chordRelativeMin, r.chordRelativeMax,
                r.metricRelativeMin, r.metricRelativeMax);
    check(r.chordRelativeMin == r.minLinearStretch && r.chordRelativeMax == r.maxLinearStretch,
          "torus: chordRelative mirrors min/maxLinearStretch");
    check(nearly(r.metricRelativeMin, bigR / (bigR + smallR), 1.e-9, 1.e-12),
          "torus: metric-relative min within 1e-9 of R/(R+r) = 0.75");
    check(nearly(r.metricRelativeMax, bigR / (bigR - smallR), 1.e-9, 1.e-12),
          "torus: metric-relative max within 1e-9 of R/(R-r) = 1.5");
    // The metric forces contraction on the outer side and expansion on the
    // inner side: both directions must be present in the measured evidence.
    check(r.minLinearStretch < 1.0 && r.maxLinearStretch > 1.0,
          "torus: both contraction and expansion present");
    // Global metric band: developed/chord >= D/L >= 1/lambda_max = R/(R+r),
    // and the pure-u inner-equator edges attain at least R/(R-r).
    check(r.minLinearStretch >= bigR / (bigR + smallR) * (1.0 - 1.e-12),
          "torus: contraction within the analytic lower band R/(R+r)");
    check(r.maxLinearStretch >= bigR / (bigR - smallR),
          "torus: expansion reaches the analytic inner-equator factor R/(R-r)");
    check(r.atlas.occupancy > 0.0 && r.atlas.occupancy <= 1.0 + 1.e-10,
          "torus: occupancy in (0, 1]");

    // Bounded output with the gutter honored on every side.
    const double gutter = 8.0 / 1024.0;
    bool bounded = true;
    for (const auto& tri : r.atlas.corners)
        for (const auto& uv : tri)
            for (int d = 0; d < 2; ++d)
                bounded = bounded && std::isfinite(uv[d])
                        && uv[d] >= gutter - 1.e-12 && uv[d] <= 1.0 - gutter + 1.e-12;
    check(bounded, "torus: UVs finite and inside gutter bounds");

    // Both seams at angle 0 pass through mesh vertex columns: exactly sv
    // u-seam edges and su v-seam edges are duplicated by one developed turn.
    censusSharedEdges("torus", mesh, r, bigR, smallR, sv, su, true);
}

void testCoarseMeshMetricBudget() {
    // T3: budgets grade the metric-relative stretch, so a coarse 16-division
    // torus still passes the analytic band budget [R/(R+r), R/(R-r)] even
    // though its chord-relative max is lifted to ~1.5097 by kappa(2*pi/16)
    // ~= 1.0065. The budgets are padded by rel 1e-9 for packed-UV rounding
    // only; the chord-relative max exceeds even the padded band, which
    // demonstrates the grading is metric-relative.
    const double bigR = 30.0, smallR = 10.0;
    const auto mesh = makeTorusBand(16, 16, bigR, smallR);
    DistortionBudgets budgets;
    budgets.minLinearStretch = bigR / (bigR + smallR) * (1.0 - 1.e-9);
    budgets.maxLinearStretch = bigR / (bigR - smallR) * (1.0 + 1.e-9);
    budgets.maxAreaError = 1.0;
    const TorusResult r = unwrapToroidalBand(mesh, defaultBand(bigR, smallR),
                                             budgets, defaultSettings());
    check(r.ok(), "coarse: 16-division torus admitted by the analytic band budget");
    if (!r.ok()) {
        std::printf("  detail: status %s index %d\n", torusStatusName(r.status), r.detailIndex);
        return;
    }
    std::printf("coarse: chordRelative=[%.12f, %.12f] metricRelative=[%.12f, %.12f]\n",
                r.chordRelativeMin, r.chordRelativeMax,
                r.metricRelativeMin, r.metricRelativeMax);
    check(nearly(r.metricRelativeMin, bigR / (bigR + smallR), 1.e-9, 1.e-12),
          "coarse: metric-relative min within 1e-9 of R/(R+r)");
    check(nearly(r.metricRelativeMax, bigR / (bigR - smallR), 1.e-9, 1.e-12),
          "coarse: metric-relative max within 1e-9 of R/(R-r)");
    check(r.chordRelativeMax > bigR / (bigR - smallR) * (1.0 + 1.e-9),
          "coarse: chord-relative max exceeds the band (kappa lift), so grading is metric-relative");
}

void testTearProbeRegression() {
    // T1 regression (root-toroidal-review torus_tear_probe.cpp): a 2-triangle
    // manifold patch, every vertex exactly on the declared R=30/r=10 torus,
    // with maxTriangleArcU = 1.5 (< pi/2, admitted) and maxTriangleArcV = 2.6
    // (>= pi/2). Before the strict pi/2 admission bound this returned Ok with
    // per-endpoint turn offsets differing on the shared edge — a torn atlas
    // published as Ok. It must now be REJECTED at admission.
    const double bigR = 30.0, smallR = 10.0;
    auto p = [&](double u, double v) {
        const double ring = bigR + smallR * std::cos(v);
        return Point{ring * std::cos(u), ring * std::sin(u), smallR * std::sin(v)};
    };
    const std::vector<Triangle> mesh{
        Triangle{{p(0.00, 0.0), p(0.10, 2.5), p(0.20, -2.5)}},
        Triangle{{p(0.20, -2.5), p(0.10, 2.5), p(0.30, 3.14)}}};
    TorusBand band = defaultBand(bigR, smallR);
    band.maxTriangleArcU = 1.5;
    band.maxTriangleArcV = 2.6;
    DistortionBudgets budgets;
    budgets.minLinearStretch = 0.01;
    budgets.maxLinearStretch = 100.0;
    budgets.maxAreaError = 100.0;
    expectRejection("tear probe: arcV >= pi/2 rejected, not a torn Ok atlas",
                    mesh, band, budgets, defaultSettings(), TorusStatus::InvalidSurface);
}

void testSeamWrapControls() {
    const int su = 64, sv = 32;
    const double bigR = 30.0, smallR = 10.0;
    const auto mesh = makeTorusBand(su, sv, bigR, smallR);
    const DistortionBudgets budgets = generousBudgets();

    // Control A: u seam off-vertex, cutting through one quad column. The cut
    // still produces exactly one duplicated shared edge per poloidal row; the
    // aligned v seam duplicates exactly one per toroidal column.
    {
        TorusBand band = defaultBand(bigR, smallR);
        band.seamAngleU = 0.37;  // inside the 4th toroidal quad column
        const TorusResult r = unwrapToroidalBand(mesh, band, budgets, defaultSettings());
        check(r.ok(), "seamU off-axis: status Ok");
        if (!r.ok()) {
            std::printf("  detail: status %s index %d\n", torusStatusName(r.status), r.detailIndex);
        } else {
            check(r.atlas.occupancy > 0.0 && r.atlas.occupancy <= 1.0 + 1.e-10,
                  "seamU off-axis: occupancy in (0, 1]");
            censusSharedEdges("seamU off-axis", mesh, r, bigR, smallR, sv, su, true);
        }
    }
    // Control B: both seams off-vertex. Both cuts run through quad interiors;
    // each cut still duplicates exactly one shared edge per crossing row or
    // column and every other shared edge stays continuous.
    {
        TorusBand band = defaultBand(bigR, smallR);
        band.seamAngleU = 0.37;
        band.seamAngleV = 0.11;  // inside the 1st poloidal quad row
        const TorusResult r = unwrapToroidalBand(mesh, band, budgets, defaultSettings());
        check(r.ok(), "seams both off-axis: status Ok");
        if (!r.ok()) {
            std::printf("  detail: status %s index %d\n", torusStatusName(r.status), r.detailIndex);
        } else {
            const double gutter = 8.0 / 1024.0;
            bool bounded = true;
            for (const auto& tri : r.atlas.corners)
                for (const auto& uv : tri)
                    for (int d = 0; d < 2; ++d)
                        bounded = bounded && std::isfinite(uv[d])
                                && uv[d] >= gutter - 1.e-12 && uv[d] <= 1.0 - gutter + 1.e-12;
            check(bounded, "seams both off-axis: UVs finite and inside gutter bounds");
            censusSharedEdges("seams both off-axis", mesh, r, bigR, smallR, sv, su, true);
        }
    }
}

void testRotatedFrame() {
    const int su = 64, sv = 32;
    const double bigR = 30.0, smallR = 10.0;
    const auto base = makeTorusBand(su, sv, bigR, smallR);
    Point axis{1.0, 2.0, 3.0};
    const double len = std::sqrt(14.0);
    for (double& c : axis) c /= len;
    const Point offset{7.5, -3.25, 11.0};
    std::vector<Triangle> rotated = base;
    for (auto& t : rotated) for (auto& p : t.points) p = transform(p, axis, 0.7, offset);

    TorusBand band;
    band.center = offset;  // the original torus center, transformed
    // Directions rotate without translation; the band axis is the rotated
    // original +z axis, not the rotation axis.
    band.axis = transform(Point{0.0, 0.0, 1.0}, axis, 0.7, Point{0.0, 0.0, 0.0});
    band.majorRadius = bigR;
    band.minorRadius = smallR;
    const TorusResult r = unwrapToroidalBand(rotated, band, generousBudgets(),
                                             defaultSettings());
    check(r.ok(), "rotated: status Ok");
    if (!r.ok()) {
        std::printf("  detail: status %s index %d\n", torusStatusName(r.status), r.detailIndex);
        return;
    }
    // Distortion evidence is a rigid-motion invariant: it must match the
    // independent base-frame derivation within rounding (rel 1e-9).
    const IndependentEvidence ev = measureIndependently(base, bigR, smallR);
    std::printf("rotated: minStretch=%.12f maxStretch=%.12f maxAreaError=%.6f\n",
                r.minLinearStretch, r.maxLinearStretch, r.maxAreaError);
    check(nearly(r.minLinearStretch, ev.minStretch, 1.e-9, 1.e-12),
          "rotated: min stretch frame-invariant (rel 1e-9)");
    check(nearly(r.maxLinearStretch, ev.maxStretch, 1.e-9, 1.e-12),
          "rotated: max stretch frame-invariant (rel 1e-9)");
    check(nearly(r.maxAreaError, ev.maxAreaError, 1.e-9, 1.e-12),
          "rotated: area error frame-invariant (rel 1e-9)");
}

void testInputImmutabilityAndRepeatability() {
    const auto mesh = makeTorusBand(48, 24, 25.0, 8.0);
    const std::vector<Triangle> snapshot = mesh;
    const DistortionBudgets budgets = generousBudgets();
    const TorusResult first = unwrapToroidalBand(mesh, defaultBand(25.0, 8.0),
                                                 budgets, defaultSettings());
    check(first.ok(), "repeat: first run Ok");
    check(mesh.size() == snapshot.size()
          && std::memcmp(mesh.data(), snapshot.data(),
                         mesh.size() * sizeof(Triangle)) == 0,
          "immutability: input byte-identical after run");
    const TorusResult second = unwrapToroidalBand(mesh, defaultBand(25.0, 8.0),
                                                  budgets, defaultSettings());
    check(second.ok(), "repeat: second run Ok");
    bool identical = first.atlas.corners.size() == second.atlas.corners.size()
        && std::memcmp(first.atlas.corners.data(), second.atlas.corners.data(),
                       first.atlas.corners.size() * sizeof(std::array<UV, 3>)) == 0
        && first.atlas.chartForTriangle == second.atlas.chartForTriangle
        && first.atlas.chartCount == second.atlas.chartCount
        && std::memcmp(&first.atlas.occupancy, &second.atlas.occupancy, sizeof(double)) == 0
        && std::memcmp(&first.atlas.unitsPerMillimeter, &second.atlas.unitsPerMillimeter,
                       sizeof(double)) == 0
        && std::memcmp(&first.minLinearStretch, &second.minLinearStretch, sizeof(double)) == 0
        && std::memcmp(&first.maxLinearStretch, &second.maxLinearStretch, sizeof(double)) == 0
        && std::memcmp(&first.maxAreaError, &second.maxAreaError, sizeof(double)) == 0;
    check(identical, "repeatability: two runs byte-identical");
}

void testRejections() {
    const auto mesh = makeTorusBand(64, 32, 30.0, 10.0);
    const TorusBand band = defaultBand(30.0, 10.0);
    const DistortionBudgets budgets = generousBudgets();
    const Settings settings = defaultSettings();

    expectRejection("reject: empty input", {}, band, budgets, settings,
                    TorusStatus::EmptyInput);

    expectRejection("reject: 4224 triangles (> 4096 cap)",
                    makeTorusBand(64, 33, 30.0, 10.0), band, budgets, settings,
                    TorusStatus::TooManyTriangles);

    {
        auto bad = mesh;
        bad[5].points[1][0] = std::numeric_limits<double>::quiet_NaN();
        expectRejection("reject: NaN coordinate", bad, band, budgets, settings,
                        TorusStatus::NonFiniteCoordinate);
    }
    {
        auto bad = mesh;
        bad[3].points[2] = bad[3].points[1];  // collapsed vertex
        expectRejection("reject: degenerate triangle", bad, band, budgets, settings,
                        TorusStatus::DegenerateTriangle);
    }
    {
        // Third use of the same edges; built on a smaller band so the 4096
        // cap does not mask the edge check.
        auto bad = makeTorusBand(32, 16, 30.0, 10.0);
        bad.push_back(bad[0]);
        expectRejection("reject: nonmanifold edge", bad, band, budgets, settings,
                        TorusStatus::NonManifoldEdge);
    }
    {
        auto bad = mesh;
        std::swap(bad[10].points[1], bad[10].points[2]);  // flipped winding
        expectRejection("reject: inconsistent winding", bad, band, budgets, settings,
                        TorusStatus::InconsistentWinding);
    }
    {
        auto bad = mesh;
        const Point orig = bad[7].points[0];
        const Point moved{orig[0] * 1.01, orig[1] * 1.01, orig[2] * 1.01};  // 1% bump
        for (auto& t : bad) for (auto& q : t.points)
            if (q == orig) q = moved;
        expectRejection("reject: off-surface vertex", bad, band, budgets, settings,
                        TorusStatus::OffSurfaceVertex);
    }
    {
        Settings bad = settings;
        bad.resolution = 100;
        expectRejection("reject: invalid settings", mesh, band, budgets, bad,
                        TorusStatus::InvalidSettings);
    }
    {
        TorusBand bad = band;
        bad.minorRadius = 0.0;
        expectRejection("reject: zero minor radius", mesh, bad, budgets, settings,
                        TorusStatus::InvalidSurface);
    }
    {
        TorusBand bad = band;
        bad.minorRadius = band.majorRadius;  // spindle torus, not a ring torus
        expectRejection("reject: minor radius equals major (not ring)", mesh, bad,
                        budgets, settings, TorusStatus::InvalidSurface);
    }
    {
        TorusBand bad = band;
        bad.minorRadius = band.majorRadius * 1.5;  // self-intersecting spindle
        expectRejection("reject: minor radius above major (not ring)", mesh, bad,
                        budgets, settings, TorusStatus::InvalidSurface);
    }
    {
        TorusBand bad = band;
        bad.axis = {0.0, 0.0, 0.0};
        expectRejection("reject: zero axis", mesh, bad, budgets, settings,
                        TorusStatus::InvalidSurface);
    }
    {
        TorusBand bad = band;
        bad.seamAngleU = std::numeric_limits<double>::infinity();
        expectRejection("reject: nonfinite u seam", mesh, bad, budgets, settings,
                        TorusStatus::InvalidSurface);
    }
    {
        TorusBand bad = band;
        bad.seamAngleV = std::numeric_limits<double>::quiet_NaN();
        expectRejection("reject: nonfinite v seam", mesh, bad, budgets, settings,
                        TorusStatus::InvalidSurface);
    }
    {
        TorusBand bad = band;
        bad.maxTriangleArcU = 3.1415926535897932;  // must stay below pi/2
        expectRejection("reject: u arc bound at pi", mesh, bad, budgets, settings,
                        TorusStatus::InvalidSurface);
    }
    {
        TorusBand bad = band;
        bad.maxTriangleArcU = 1.5707963267948966;  // exactly pi/2: strict bound
        expectRejection("reject: u arc bound at pi/2 (strict)", mesh, bad, budgets,
                        settings, TorusStatus::InvalidSurface);
    }
    {
        TorusBand bad = band;
        bad.maxTriangleArcV = 1.5707963267948966;  // exactly pi/2: strict bound
        expectRejection("reject: v arc bound at pi/2 (strict)", mesh, bad, budgets,
                        settings, TorusStatus::InvalidSurface);
    }
    {
        TorusBand bad = band;
        bad.fitToleranceModelUnits = 0.0;  // must be finite and > 0
        expectRejection("reject: zero fit tolerance", mesh, bad, budgets, settings,
                        TorusStatus::InvalidSurface);
    }
    {
        TorusBand bad = band;
        bad.fitToleranceModelUnits = std::numeric_limits<double>::quiet_NaN();
        expectRejection("reject: nonfinite fit tolerance", mesh, bad, budgets, settings,
                        TorusStatus::InvalidSurface);
    }
    {
        TorusBand bad = band;
        bad.maxTriangleArcV = 0.0;
        expectRejection("reject: zero v arc bound", mesh, bad, budgets, settings,
                        TorusStatus::InvalidSurface);
    }
    {
        // 3 toroidal segments: one triangle spans 2*pi/3 > pi/2 in u.
        expectRejection("reject: excessive toroidal span",
                        makeTorusBand(3, 32, 30.0, 10.0), band, budgets, settings,
                        TorusStatus::ExcessiveAngularSpan);
    }
    {
        // 3 poloidal segments: one triangle spans 2*pi/3 > pi/2 in v.
        expectRejection("reject: excessive poloidal span",
                        makeTorusBand(64, 3, 30.0, 10.0), band, budgets, settings,
                        TorusStatus::ExcessiveAngularSpan);
    }
    {
        // A flat outer-equator ring (all v = 0) is not a toroidal band: zero
        // poloidal extent collapses every developed triangle.
        std::vector<Triangle> ring;
        const int n = 16;
        for (int i = 0; i < n; ++i) {
            const double a0 = kTwoPi * i / n, a1 = kTwoPi * (i + 1) / n;
            const double radius = 40.0;  // R + r at v = 0
            ring.push_back(Triangle{{Point{radius * std::cos(a0), radius * std::sin(a0), 0.0},
                                     Point{radius * std::cos(a1), radius * std::sin(a1), 0.0},
                                     Point{radius * std::cos((a0 + a1) / 2),
                                           radius * std::sin((a0 + a1) / 2), 0.0}}});
        }
        expectRejection("reject: flat equator ring is not a band", ring, band,
                        budgets, settings, TorusStatus::DegenerateTriangle);
    }
}

void testBudgetAdmissionAndRejection() {
    const auto mesh = makeTorusBand(64, 32, 30.0, 10.0);
    const TorusBand band = defaultBand(30.0, 10.0);
    const Settings settings = defaultSettings();

    // Invalid budget admission: non-finite, non-positive or inconsistent.
    {
        DistortionBudgets bad = generousBudgets();
        bad.maxLinearStretch = std::numeric_limits<double>::quiet_NaN();
        expectRejection("budget: reject NaN max stretch", mesh, band, bad, settings,
                        TorusStatus::InvalidBudget);
    }
    {
        DistortionBudgets bad = generousBudgets();
        bad.minLinearStretch = 0.0;
        expectRejection("budget: reject zero min stretch", mesh, band, bad, settings,
                        TorusStatus::InvalidBudget);
    }
    {
        DistortionBudgets bad = generousBudgets();
        bad.minLinearStretch = 1.2;  // above maxLinearStretch after swap below
        bad.maxLinearStretch = 1.1;
        expectRejection("budget: reject min above max", mesh, band, bad, settings,
                        TorusStatus::InvalidBudget);
    }
    {
        DistortionBudgets bad = generousBudgets();
        bad.maxAreaError = -0.1;
        expectRejection("budget: reject negative area error", mesh, band, bad, settings,
                        TorusStatus::InvalidBudget);
    }

    // Measured distortion exceeds each caller budget in turn. The metric band
    // for R=30, r=10 is [0.75, 1.5]: expansion reaches ~1.501 (chord effect),
    // contraction reaches ~0.751, area error reaches ~0.5. All rejections are
    // atomic with empty output (asserted inside expectRejection).
    {
        DistortionBudgets tight = generousBudgets();
        tight.maxLinearStretch = 1.4;  // below the unavoidable inner-side expansion
        expectRejection("budget: reject expansion above caller max", mesh, band, tight,
                        settings, TorusStatus::DistortionBeyondBudget);
    }
    {
        DistortionBudgets tight = generousBudgets();
        tight.minLinearStretch = 0.9;  // above the unavoidable outer-side contraction
        expectRejection("budget: reject contraction below caller min", mesh, band, tight,
                        settings, TorusStatus::DistortionBeyondBudget);
    }
    {
        DistortionBudgets tight = generousBudgets();
        tight.maxAreaError = 0.1;  // below the unavoidable areal distortion
        expectRejection("budget: reject area error above caller max", mesh, band, tight,
                        settings, TorusStatus::DistortionBeyondBudget);
    }

    // Caller budgets are never adjusted after measuring output.
    DistortionBudgets budgets = generousBudgets();
    const DistortionBudgets before = budgets;
    const TorusResult r = unwrapToroidalBand(mesh, band, budgets, settings);
    check(r.ok(), "budget: generous budgets admit the analytic band");
    check(std::memcmp(&budgets, &before, sizeof(DistortionBudgets)) == 0,
          "budget: caller budgets unchanged after the run");
}

void testManifoldVertexLink() {
    const TorusBand band = defaultBand(30.0, 10.0);
    const DistortionBudgets budgets = generousBudgets();
    const Settings settings = defaultSettings();
    auto p = [](double u, double v) {
        const double ring = 30.0 + 10.0 * std::cos(v);
        return Point{ring * std::cos(u), ring * std::sin(u), 10.0 * std::sin(v)};
    };

    // Bowtie reproduction on the torus: every vertex sits on the declared
    // surface and no developed interiors overlap, but the shared vertex
    // p(0,0) has two disconnected single-triangle fans.
    const std::vector<Triangle> bowtie{
        Triangle{{p(0, 0), p(0.05, 0), p(0, 0.05)}},
        Triangle{{p(0, 0), p(kTwoPi - 0.05, 0), p(0, kTwoPi - 0.05)}}};
    expectRejection("vertexLink: reject bowtie on torus", bowtie, band, budgets,
                    settings, TorusStatus::NonManifoldVertex);

    // Same violation class, three disconnected fans at one geometric vertex.
    const std::vector<Triangle> triple{
        Triangle{{p(0, 0), p(0.05, 0), p(0, 0.05)}},
        Triangle{{p(0, 0), p(kTwoPi - 0.05, 0), p(0, kTwoPi - 0.05)}},
        Triangle{{p(0, 0), p(0.1, 0.02), p(0.02, 0.1)}}};
    expectRejection("vertexLink: reject three disconnected fans", triple, band,
                    budgets, settings, TorusStatus::NonManifoldVertex);

    // Transformed-frame control: the same bowtie in a rotated/translated
    // frame must reject identically.
    Point axis{1.0, 2.0, 3.0};
    const double len = std::sqrt(14.0);
    for (double& c : axis) c /= len;
    const Point offset{7.5, -3.25, 11.0};
    auto rotate = [&](std::vector<Triangle> mesh) {
        for (auto& t : mesh) for (auto& q : t.points) q = transform(q, axis, 0.7, offset);
        return mesh;
    };
    TorusBand rotatedBand = band;
    rotatedBand.center = offset;
    rotatedBand.axis = transform(Point{0.0, 0.0, 1.0}, axis, 0.7, Point{0.0, 0.0, 0.0});
    expectRejection("vertexLink: reject rotated-frame bowtie", rotate(bowtie),
                    rotatedBand, budgets, settings, TorusStatus::NonManifoldVertex);

    // Normal connected fan control: two edge-adjacent triangles (one quad on
    // the same torus) stay admitted with a single chart.
    const std::vector<Triangle> quad{
        Triangle{{p(0, 0), p(0.05, 0), p(0.05, 0.05)}},
        Triangle{{p(0, 0), p(0.05, 0.05), p(0, 0.05)}}};
    const TorusResult connected = unwrapToroidalBand(quad, band, budgets, settings);
    check(connected.ok(), "vertexLink: connected fans Ok");
    if (!connected.ok())
        std::printf("  detail: status %s index %d\n",
                    torusStatusName(connected.status), connected.detailIndex);
    check(connected.ok() && connected.atlas.chartCount == 1,
          "vertexLink: connected fans single chart");
    const TorusResult rotatedConnected = unwrapToroidalBand(rotate(quad), rotatedBand,
                                                            budgets, settings);
    check(rotatedConnected.ok(), "vertexLink: rotated connected fans Ok");

    // Open-patch boundary control: a half-tube band (v spans 0..pi) leaves two
    // open poloidal rims; every boundary vertex has exactly two boundary rays
    // and stays admitted.
    const std::vector<Triangle> open = makeTorusOpen(16, 8, 30.0, 10.0, 0.0,
                                                     3.1415926535897932);
    const TorusResult openResult = unwrapToroidalBand(open, band, budgets, settings);
    check(openResult.ok(), "vertexLink: open half-tube patch Ok");
    if (!openResult.ok())
        std::printf("  detail: status %s index %d\n",
                    torusStatusName(openResult.status), openResult.detailIndex);
}

}  // namespace

int main() {
    testAnalyticTorus();
    testCoarseMeshMetricBudget();
    testTearProbeRegression();
    testSeamWrapControls();
    testRotatedFrame();
    testInputImmutabilityAndRepeatability();
    testRejections();
    testBudgetAdmissionAndRejection();
    testManifoldVertexLink();
    std::printf("checks=%d failures=%d\n", checks, failures);
    if (failures == 0) std::printf("ALL TESTS PASSED\n");
    return failures == 0 ? 0 : 1;
}
