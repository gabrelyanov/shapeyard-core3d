// Standalone tests for the CurvedMeshUVPrototype candidate kernel.
// Ordinary C++ test program, no framework, stdlib only. Compile and run:
//   /usr/bin/clang++ -std=c++17 -Wall -Wextra -Werror -O2 \
//       CurvedMeshUVPrototypeTests.cpp -o CurvedMeshUVPrototypeTests
// Analytic tolerances are stated next to each assertion before grading; this
// is candidate-kernel evidence, not a benchmark pass (no B01 claim).

#include "../Core3D/OCCTKit/CurvedMeshUVPrototype.hpp"

#include <cstdio>
#include <cstring>
#include <map>
#include <set>

using namespace shapeyard::uv;
using namespace shapeyard::uv::prototype;

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

// Closed cylindrical side band: segments around, rows along +z, outward
// winding (counterclockwise seen from outside). Deterministic vertex order.
std::vector<Triangle> makeCylinderBand(int segments, int rows, double radius,
                                       double zLow, double zHigh) {
    auto vertex = [&](int s, int r) {
        const double theta = kTwoPi * s / segments;
        const double z = zLow + (zHigh - zLow) * r / rows;
        return Point{radius * std::cos(theta), radius * std::sin(theta), z};
    };
    std::vector<Triangle> mesh;
    for (int s = 0; s < segments; ++s) {
        const int s1 = (s + 1) % segments;
        for (int r = 0; r < rows; ++r) {
            const Point a = vertex(s, r), b = vertex(s1, r);
            const Point c = vertex(s1, r + 1), d = vertex(s, r + 1);
            // Outward-facing diagonal split, consistent across the band.
            mesh.push_back(Triangle{{a, c, b}});
            mesh.push_back(Triangle{{a, d, c}});
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

CylinderBand defaultBand(double radius) {
    CylinderBand band;
    band.center = {0.0, 0.0, 0.0};
    band.axis = {0.0, 0.0, 1.0};
    band.radius = radius;
    band.seamAngle = 0.0;
    return band;
}

void expectRejection(const char* name, const std::vector<Triangle>& input,
                     const CylinderBand& band, const Settings& settings,
                     CylinderStatus expected) {
    const CylinderResult r = unwrapCylindricalBand(input, band, settings);
    bool ok = r.status == expected;
    // Failed-input atomicity: no partial atlas or misleading numbers.
    ok = ok && !r.ok() && r.atlas.corners.empty() && r.atlas.chartForTriangle.empty()
        && r.atlas.chartCount == 0 && r.atlas.occupancy == 0.0
        && r.atlas.unitsPerMillimeter == 0.0
        && r.maxLinearStretch == 1.0 && r.maxAreaError == 0.0
        && r.chordRelativeMin == 1.0 && r.chordRelativeMax == 1.0
        && r.metricRelativeMin == 1.0 && r.metricRelativeMax == 1.0;
    check(ok, name);
    if (!ok)
        std::printf("  detail: expected %s, got %s (index %d)\n",
                    cylinderStatusName(expected), cylinderStatusName(r.status), r.detailIndex);
}

// Shared-edge continuity/duplication census from independent shared-edge
// geometry: both uses of every shared edge must map to identical UV endpoints
// (continuous), or differ by exactly one developed turn (2*pi*radius*scale)
// in u when the seam passes between them. A one-turn-duplicated edge that is
// not a constant-u column edge is a spurious interior diagonal seam (the
// defect the removed mean-based re-centering produced). Anything else is
// torn.
struct SeamCensus {
    int continuous = 0, column = 0, diagonal = 0, broken = 0;
};
SeamCensus censusSharedEdges(const char* label, const std::vector<Triangle>& mesh,
                             const CylinderResult& r, double radius) {
    std::map<std::pair<Point, Point>, std::vector<std::array<UV, 2>>> edgeUV;
    for (std::size_t i = 0; i < mesh.size(); ++i)
        for (int k = 0; k < 3; ++k) {
            Point a = mesh[i].points[k], b = mesh[i].points[(k + 1) % 3];
            UV ua = r.atlas.corners[i][k], ub = r.atlas.corners[i][(k + 1) % 3];
            if (b < a) { std::swap(a, b); std::swap(ua, ub); }
            edgeUV[{a, b}].push_back({ua, ub});
        }
    SeamCensus census;
    const double turn = kTwoPi * radius * r.atlas.unitsPerMillimeter;
    for (const auto& entry : edgeUV) {
        if (entry.second.size() != 2) continue;  // boundary edges: none in a closed band
        const auto& x = entry.second[0]; const auto& y = entry.second[1];
        // Endpoints are stored in the same geometric order in both uses.
        const double du = std::abs(x[0][0] - y[0][0]) + std::abs(x[1][0] - y[1][0]);
        const double dv = std::abs(x[0][1] - y[0][1]) + std::abs(x[1][1] - y[1][1]);
        if (du <= 1.e-9 * turn && dv <= 1.e-9) { ++census.continuous; continue; }
        if (!nearly(du, 2.0 * turn, 1.e-9, 1.e-12) || dv > 1.e-9) { ++census.broken; continue; }
        // One-turn duplicated: a seam-column edge has constant u in the
        // chart; anything with extent in both chart directions is an interior
        // diagonal seam.
        const double uExtent = std::abs(x[0][0] - x[1][0]);
        const double vExtent = std::abs(x[0][1] - x[1][1]);
        if (uExtent <= 1.e-9 * turn && vExtent > 1.e-9) ++census.column;
        else ++census.diagonal;
    }
    std::printf("%s: shared edges continuous=%d seamColumn=%d interiorDiagonal=%d broken=%d\n",
                label, census.continuous, census.column, census.diagonal, census.broken);
    return census;
}

void testAnalyticCylinder() {
    const int segments = 64, rows = 4;
    const double radius = 30.0, zLow = -20.0, zHigh = 20.0;
    const auto mesh = makeCylinderBand(segments, rows, radius, zLow, zHigh);
    const CylinderResult r = unwrapCylindricalBand(mesh, defaultBand(radius), defaultSettings());
    check(r.ok(), "cylinder: status Ok");
    if (!r.ok()) {
        std::printf("  detail: status %s index %d\n", cylinderStatusName(r.status), r.detailIndex);
        return;
    }
    check(r.atlas.chartCount == 1, "cylinder: single chart");
    check(r.atlas.corners.size() == mesh.size(), "cylinder: per-triangle corners");
    bool chartsZero = true;
    for (int c : r.atlas.chartForTriangle) chartsZero = chartsZero && c == 0;
    check(chartsZero, "cylinder: chartForTriangle all 0");

    // Analytic tolerances, stated before grading. The development is
    // isometric on the surface; the 3D chords undermeasure arcs, so the exact
    // circumferential stretch is delta / (2 sin(delta/2)) with delta = 2*pi/64.
    const double delta = kTwoPi / segments;
    const double expectedStretch = delta / (2.0 * std::sin(delta / 2.0));
    std::printf("cylinder: maxLinearStretch=%.12f expected=%.12f maxAreaError=%.3e occupancy=%.6f\n",
                r.maxLinearStretch, expectedStretch, r.maxAreaError, r.atlas.occupancy);
    check(std::abs(r.maxLinearStretch - expectedStretch) <= expectedStretch * 1.e-9 + 1.e-12,
          "cylinder: stretch matches analytic chord/arc model (rel 1e-9)");
    // First-order area error bound is delta^2/24; admit delta^2/16 (1.5x margin).
    check(r.maxAreaError <= delta * delta / 16.0, "cylinder: area error within delta^2/16");
    check(r.atlas.occupancy > 0.0 && r.atlas.occupancy <= 1.0 + 1.e-10,
          "cylinder: occupancy in (0, 1]");

    // Bounded output with the gutter honored on every side.
    const double gutter = 8.0 / 1024.0;
    bool bounded = true;
    for (const auto& tri : r.atlas.corners)
        for (const auto& uv : tri)
            for (int d = 0; d < 2; ++d)
                bounded = bounded && std::isfinite(uv[d])
                        && uv[d] >= gutter - 1.e-12 && uv[d] <= 1.0 - gutter + 1.e-12;
    check(bounded, "cylinder: UVs finite and inside gutter bounds");

    // T3: stretch reported both ways; the metric-relative value divides out
    // the circumferential chord factor and is exactly 1 for this isometric
    // development, regardless of tessellation.
    std::printf("cylinder: chordRelative=[%.12f, %.12f] metricRelative=[%.12f, %.12f]\n",
                r.chordRelativeMin, r.chordRelativeMax,
                r.metricRelativeMin, r.metricRelativeMax);
    check(r.chordRelativeMax == r.maxLinearStretch,
          "cylinder: chordRelativeMax mirrors maxLinearStretch");
    check(std::abs(r.metricRelativeMin - 1.0) <= 1.e-9
          && std::abs(r.metricRelativeMax - 1.0) <= 1.e-9,
          "cylinder: metric-relative stretch is 1 (isometric) within 1e-9");

    // Seam behavior from independent shared-edge geometry: every edge shared
    // by two triangles must map both uses to identical UV endpoints
    // (continuous), except exactly the `rows` seam-column edges, which must
    // differ by exactly one developed turn (2*pi*radius*scale) in u.
    const SeamCensus census = censusSharedEdges("cylinder", mesh, r, radius);
    check(census.broken == 0, "cylinder: no torn shared edges");
    check(census.diagonal == 0, "cylinder: no interior diagonal seams");
    check(census.column == rows, "cylinder: exactly the seam column is duplicated");
}

void testRotatedFrame() {
    const int segments = 64, rows = 4;
    const double radius = 30.0;
    const auto base = makeCylinderBand(segments, rows, radius, -20.0, 20.0);
    Point axis{1.0, 2.0, 3.0};
    const double len = std::sqrt(14.0);
    for (double& c : axis) c /= len;
    const Point offset{7.5, -3.25, 11.0};
    std::vector<Triangle> rotated = base;
    for (auto& t : rotated) for (auto& p : t.points) p = transform(p, axis, 0.7, offset);

    CylinderBand band;
    band.center = offset;  // the original origin, transformed
    // Directions rotate without translation; the band axis is the rotated
    // original +z axis, not the rotation axis.
    band.axis = transform(Point{0.0, 0.0, 1.0}, axis, 0.7, Point{0.0, 0.0, 0.0});
    band.radius = radius;
    band.seamAngle = 0.0;
    const CylinderResult r = unwrapCylindricalBand(rotated, band, defaultSettings());
    check(r.ok(), "rotated: status Ok");
    if (!r.ok()) {
        std::printf("  detail: status %s index %d\n", cylinderStatusName(r.status), r.detailIndex);
        return;
    }
    const double delta = kTwoPi / segments;
    const double expectedStretch = delta / (2.0 * std::sin(delta / 2.0));
    std::printf("rotated: maxLinearStretch=%.12f maxAreaError=%.3e occupancy=%.6f\n",
                r.maxLinearStretch, r.maxAreaError, r.atlas.occupancy);
    check(std::abs(r.maxLinearStretch - expectedStretch) <= expectedStretch * 1.e-9 + 1.e-12,
          "rotated: stretch matches analytic model (rel 1e-9)");
    check(r.maxAreaError <= delta * delta / 16.0, "rotated: area error within delta^2/16");
}

void testOffAxisSeamAngle() {
    // Seam cut not aligned to any mesh vertex: the cut passes through a quad
    // column. Use a MID-COLUMN seam (fraction 0.5 into the column): the old
    // mean-based re-centering disagreed on the two triangles of each
    // seam-crossed quad for seam fractions in (1/3, 2/3) — the review's
    // cyl_fix_probe shows seamAngle 0.37 lands at fraction 0.769 and never
    // exercises the bug — putting a spurious one-turn duplication on their
    // shared diagonal. After its removal the census must show exactly one
    // duplicated column edge per row, no interior diagonal seams and zero
    // torn edges.
    const int segments = 64, rows = 4;
    const double radius = 30.0;
    const auto mesh = makeCylinderBand(segments, rows, radius, -20.0, 20.0);
    CylinderBand band = defaultBand(radius);
    band.seamAngle = kTwoPi / (2.0 * segments);  // mid-column (fraction 0.5)
    const CylinderResult r = unwrapCylindricalBand(mesh, band, defaultSettings());
    check(r.ok(), "offAxisSeam: status Ok");
    if (!r.ok()) {
        std::printf("  detail: status %s index %d\n", cylinderStatusName(r.status), r.detailIndex);
        return;
    }
    std::printf("offAxisSeam: maxLinearStretch=%.12f occupancy=%.6f\n",
                r.maxLinearStretch, r.atlas.occupancy);
    check(r.atlas.occupancy > 0.0 && r.atlas.occupancy <= 1.0 + 1.e-10,
          "offAxisSeam: occupancy in (0, 1]");
    const SeamCensus census = censusSharedEdges("offAxisSeam", mesh, r, radius);
    check(census.broken == 0, "offAxisSeam: no torn shared edges");
    check(census.diagonal == 0, "offAxisSeam: no interior diagonal seams");
    check(census.column == rows, "offAxisSeam: exactly the seam column is duplicated");
}

void testInputImmutabilityAndRepeatability() {
    const auto mesh = makeCylinderBand(48, 3, 25.0, 0.0, 50.0);
    const std::vector<Triangle> snapshot = mesh;
    const CylinderResult first = unwrapCylindricalBand(mesh, defaultBand(25.0), defaultSettings());
    check(first.ok(), "repeat: first run Ok");
    check(mesh.size() == snapshot.size()
          && std::memcmp(mesh.data(), snapshot.data(),
                         mesh.size() * sizeof(Triangle)) == 0,
          "immutability: input byte-identical after run");
    const CylinderResult second = unwrapCylindricalBand(mesh, defaultBand(25.0), defaultSettings());
    check(second.ok(), "repeat: second run Ok");
    bool identical = first.atlas.corners.size() == second.atlas.corners.size()
        && std::memcmp(first.atlas.corners.data(), second.atlas.corners.data(),
                       first.atlas.corners.size() * sizeof(std::array<UV, 3>)) == 0
        && first.atlas.chartForTriangle == second.atlas.chartForTriangle
        && first.atlas.chartCount == second.atlas.chartCount
        && std::memcmp(&first.atlas.occupancy, &second.atlas.occupancy, sizeof(double)) == 0
        && std::memcmp(&first.atlas.unitsPerMillimeter, &second.atlas.unitsPerMillimeter,
                       sizeof(double)) == 0
        && std::memcmp(&first.maxLinearStretch, &second.maxLinearStretch, sizeof(double)) == 0
        && std::memcmp(&first.maxAreaError, &second.maxAreaError, sizeof(double)) == 0;
    check(identical, "repeatability: two runs byte-identical");
}

void testRejections() {
    const auto mesh = makeCylinderBand(64, 4, 30.0, -20.0, 20.0);
    const CylinderBand band = defaultBand(30.0);
    const Settings settings = defaultSettings();

    expectRejection("reject: empty input", {}, band, settings, CylinderStatus::EmptyInput);

    expectRejection("reject: 4097 triangles", makeCylinderBand(64, 33, 30.0, -20.0, 20.0),
                    band, settings, CylinderStatus::TooManyTriangles);

    {
        auto bad = mesh;
        bad[5].points[1][0] = std::numeric_limits<double>::quiet_NaN();
        expectRejection("reject: NaN coordinate", bad, band, settings,
                        CylinderStatus::NonFiniteCoordinate);
    }
    {
        auto bad = mesh;
        bad[3].points[2] = bad[3].points[1];  // collapsed vertex
        expectRejection("reject: degenerate triangle", bad, band, settings,
                        CylinderStatus::DegenerateTriangle);
    }
    {
        auto bad = mesh;
        bad.push_back(mesh[0]);  // third use of the same edges
        expectRejection("reject: nonmanifold edge", bad, band, settings,
                        CylinderStatus::NonManifoldEdge);
    }
    {
        auto bad = mesh;
        std::swap(bad[10].points[1], bad[10].points[2]);  // flipped winding
        expectRejection("reject: inconsistent winding", bad, band, settings,
                        CylinderStatus::InconsistentWinding);
    }
    {
        auto bad = mesh;
        const Point orig = bad[7].points[0];
        const Point moved{orig[0] * 1.01, orig[1] * 1.01, orig[2]};  // 1% radial bump
        for (auto& t : bad) for (auto& q : t.points)
            if (q == orig) q = moved;
        expectRejection("reject: off-surface vertex", bad, band, settings,
                        CylinderStatus::OffSurfaceVertex);
    }
    {
        Settings bad = settings;
        bad.resolution = 100;
        expectRejection("reject: invalid settings", mesh, band, bad,
                        CylinderStatus::InvalidSettings);
    }
    {
        CylinderBand bad = band;
        bad.radius = 0.0;
        expectRejection("reject: zero radius", mesh, bad, settings,
                        CylinderStatus::InvalidSurface);
    }
    {
        CylinderBand bad = band;
        bad.axis = {0.0, 0.0, 0.0};
        expectRejection("reject: zero axis", mesh, bad, settings,
                        CylinderStatus::InvalidSurface);
    }
    {
        CylinderBand bad = band;
        bad.seamAngle = std::numeric_limits<double>::infinity();
        expectRejection("reject: nonfinite seam angle", mesh, bad, settings,
                        CylinderStatus::InvalidSurface);
    }
    {
        CylinderBand bad = band;
        bad.maxTriangleArc = 1.5707963267948966;  // exactly pi/2: strict bound
        expectRejection("reject: arc bound at pi/2 (strict)", mesh, bad, settings,
                        CylinderStatus::InvalidSurface);
    }
    {
        CylinderBand bad = band;
        bad.maxTriangleArc = 2.6;  // above pi/2 (the torus tear-probe value)
        expectRejection("reject: arc bound above pi/2", mesh, bad, settings,
                        CylinderStatus::InvalidSurface);
    }
    {
        CylinderBand bad = band;
        bad.fitToleranceModelUnits = 0.0;  // must be finite and > 0
        expectRejection("reject: zero fit tolerance", mesh, bad, settings,
                        CylinderStatus::InvalidSurface);
    }
    {
        CylinderBand bad = band;
        bad.fitToleranceModelUnits = std::numeric_limits<double>::quiet_NaN();
        expectRejection("reject: nonfinite fit tolerance", mesh, bad, settings,
                        CylinderStatus::InvalidSurface);
    }
    {
        // 3 segments: one triangle spans 2*pi/3, above the < pi/2 admission
        // bound and the default maxTriangleArc.
        expectRejection("reject: excessive angular span",
                        makeCylinderBand(3, 1, 30.0, 0.0, 10.0), band, settings,
                        CylinderStatus::ExcessiveAngularSpan);
    }
    {
        // A flat annulus ring is not a side band: zero height extent.
        std::vector<Triangle> ring;
        const int n = 16;
        for (int i = 0; i < n; ++i) {
            const double a0 = kTwoPi * i / n, a1 = kTwoPi * (i + 1) / n;
            ring.push_back(Triangle{{Point{30.0 * std::cos(a0), 30.0 * std::sin(a0), 5.0},
                                     Point{30.0 * std::cos(a1), 30.0 * std::sin(a1), 5.0},
                                     Point{30.0 * std::cos((a0 + a1) / 2), 30.0 * std::sin((a0 + a1) / 2), 5.0}}});
        }
        // All corners share one height, so each developed UV triangle is
        // degenerate; the kernel rejects before packing. Either way a flat
        // ring is not an admitted side band.
        expectRejection("reject: flat ring is not a band", ring, band, settings,
                        CylinderStatus::DegenerateTriangle);
    }
}

void testManifoldVertexLink() {
    const CylinderBand band = defaultBand(30.0);
    const Settings settings = defaultSettings();
    auto p = [](double a, double z) { return Point{30 * std::cos(a), 30 * std::sin(a), z}; };

    // Exact coordinator bowtie reproduction (root-curved-uv-review/bowtie-
    // probe.cpp): every vertex sits on the declared cylinder and no UV
    // interiors overlap, but the shared vertex p(0,0) has two disconnected
    // single-triangle fans. Rejects with no partial UV output.
    const std::vector<Triangle> bowtie{Triangle{{p(0, 0), p(.1, 0), p(0, 1)}},
                                       Triangle{{p(0, 0), p(-.1, 0), p(0, -1)}}};
    expectRejection("vertexLink: reject exact bowtie probe mesh", bowtie, band, settings,
                    CylinderStatus::NonManifoldVertex);

    // Same violation class, three disconnected fans at one geometric vertex.
    const std::vector<Triangle> triple{Triangle{{p(0, 0), p(.1, 0), p(0, 1)}},
                                       Triangle{{p(0, 0), p(-.1, 0), p(0, -1)}},
                                       Triangle{{p(0, 0), p(.2, 0), p(.1, -1)}}};
    expectRejection("vertexLink: reject three disconnected fans", triple, band, settings,
                    CylinderStatus::NonManifoldVertex);

    // Transformed-frame controls: the same meshes in a rotated/translated
    // frame must reject/admit identically.
    Point axis{1.0, 2.0, 3.0};
    const double len = std::sqrt(14.0);
    for (double& c : axis) c /= len;
    const Point offset{7.5, -3.25, 11.0};
    auto rotate = [&](std::vector<Triangle> mesh) {
        for (auto& t : mesh) for (auto& q : t.points) q = transform(q, axis, 0.7, offset);
        return mesh;
    };
    CylinderBand rotatedBand = band;
    rotatedBand.center = offset;
    rotatedBand.axis = transform(Point{0.0, 0.0, 1.0}, axis, 0.7, Point{0.0, 0.0, 0.0});
    expectRejection("vertexLink: reject rotated-frame bowtie", rotate(bowtie), rotatedBand,
                    settings, CylinderStatus::NonManifoldVertex);

    // Normal connected fan control: two edge-adjacent triangles (one quad
    // column on the same cylinder) stay admitted with a single chart.
    const std::vector<Triangle> quad{Triangle{{p(0, 0), p(.1, 0), p(0, 1)}},
                                     Triangle{{p(.1, 0), p(.1, 1), p(0, 1)}}};
    const CylinderResult connected = unwrapCylindricalBand(quad, band, settings);
    check(connected.ok(), "vertexLink: connected fans Ok");
    check(connected.ok() && connected.atlas.chartCount == 1,
          "vertexLink: connected fans single chart");
    const CylinderResult rotatedConnected = unwrapCylindricalBand(rotate(quad), rotatedBand,
                                                                  settings);
    check(rotatedConnected.ok(), "vertexLink: rotated connected fans Ok");

    // Open-manifold boundary control: one open band row gives every boundary
    // vertex a connected fan with exactly two boundary rays; still admitted.
    const std::vector<Triangle> openRow = makeCylinderBand(8, 1, 30.0, -1.0, 1.0);
    const CylinderResult open = unwrapCylindricalBand(openRow, band, settings);
    check(open.ok(), "vertexLink: open band two-ray boundary Ok");
    if (!open.ok())
        std::printf("  detail: status %s index %d\n",
                    cylinderStatusName(open.status), open.detailIndex);
}

}  // namespace

int main() {
    testAnalyticCylinder();
    testRotatedFrame();
    testOffAxisSeamAngle();
    testInputImmutabilityAndRepeatability();
    testRejections();
    testManifoldVertexLink();
    std::printf("checks=%d failures=%d\n", checks, failures);
    if (failures == 0) std::printf("ALL TESTS PASSED\n");
    return failures == 0 ? 0 : 1;
}
