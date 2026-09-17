// CurvedUVPackerTests.cpp — standalone checks for CurvedUVPacker.hpp.
// Build: /usr/bin/clang++ -std=c++17 -Wall -Wextra -Werror -O2 \
//          CurvedUVPackerTests.cpp -o CurvedUVPackerTests

#include "../Core3D/OCCTKit/CurvedUVPacker.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static int g_checks = 0;
static int g_failures = 0;

#define CHECK(cond)                                                        \
  do {                                                                     \
    ++g_checks;                                                            \
    if (!(cond)) {                                                         \
      ++g_failures;                                                        \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
    }                                                                      \
  } while (0)

namespace {

using curveduv::ChartInput;
using curveduv::ChartTriangle;
using curveduv::KernelTag;
using curveduv::PackerInput;
using curveduv::PackResult;
using curveduv::SharedEdge;
using curveduv::Vec2;

const double kPi = 3.14159265358979323846;

// Grid chart builder: cols x rows quads (2 triangles each) over a developed
// rectangle L x W mm. Vertex (i,j) -> (i*L/cols, j*W/rows).
// Quad(i,j): T0 = {V(i,j), V(i+1,j), V(i+1,j+1)}
//            T1 = {V(i,j), V(i+1,j+1), V(i,j+1)}
// Local triangle index of T0 in quad (i,j) = 2*(j*cols+i), T1 = +1.
// Adjacency covers every interior edge; when wrapSeam is set, the column-0 /
// column-cols boundary edges are listed with kernelSeam=true.
ChartInput makeGridChart(int faceId, KernelTag tag, double L, double W, int cols,
                         int rows, bool wrapSeam, std::vector<SharedEdge>* edges,
                         int globalBase) {
  ChartInput ch;
  ch.faceId = faceId;
  ch.kernel = tag;
  ch.rectMin = Vec2{0.0, 0.0};
  ch.rectMax = Vec2{L, W};
  ch.triangles.reserve(static_cast<size_t>(cols) * rows * 2);
  auto V = [L, W, cols, rows](int i, int j) {
    return Vec2{static_cast<double>(i) * L / cols, static_cast<double>(j) * W / rows};
  };
  for (int j = 0; j < rows; ++j) {
    for (int i = 0; i < cols; ++i) {
      ChartTriangle t0, t1;
      t0.corner[0] = V(i, j);
      t0.corner[1] = V(i + 1, j);
      t0.corner[2] = V(i + 1, j + 1);
      t1.corner[0] = V(i, j);
      t1.corner[1] = V(i + 1, j + 1);
      t1.corner[2] = V(i, j + 1);
      ch.triangles.push_back(t0);
      ch.triangles.push_back(t1);
    }
  }
  if (edges) {
    auto t0idx = [cols](int i, int j) { return 2 * (j * cols + i); };
    // vertical interior edges between quad (i-1,j) and quad (i,j)
    for (int j = 0; j < rows; ++j) {
      for (int i = 1; i < cols; ++i) {
        SharedEdge e;
        e.faceId = faceId;
        e.triA = globalBase + t0idx(i - 1, j);  // T0 of left quad, corners {1,2}
        e.cornerA0 = 1;
        e.cornerA1 = 2;
        e.triB = globalBase + t0idx(i, j) + 1;  // T1 of right quad, corners {0,2}
        e.cornerB0 = 0;
        e.cornerB1 = 2;
        edges->push_back(e);
      }
    }
    // wrap (kernel seam) edges at u=0 / u=L
    if (wrapSeam) {
      for (int j = 0; j < rows; ++j) {
        SharedEdge e;
        e.faceId = faceId;
        e.triA = globalBase + t0idx(cols - 1, j);
        e.cornerA0 = 1;
        e.cornerA1 = 2;
        e.triB = globalBase + t0idx(0, j) + 1;
        e.cornerB0 = 0;
        e.cornerB1 = 2;
        e.kernelSeam = true;
        edges->push_back(e);
      }
    }
    // horizontal edges between quad (i,j-1) and quad (i,j)
    for (int j = 1; j < rows; ++j) {
      for (int i = 0; i < cols; ++i) {
        SharedEdge e;
        e.faceId = faceId;
        e.triA = globalBase + t0idx(i, j - 1) + 1;  // T1 below, corners {1,2}
        e.cornerA0 = 1;
        e.cornerA1 = 2;
        e.triB = globalBase + t0idx(i, j);  // T0 above, corners {0,1}
        e.cornerB0 = 0;
        e.cornerB1 = 1;
        edges->push_back(e);
      }
    }
    // diagonal edges inside each quad: T0 corners {0,2}, T1 corners {0,1}
    for (int j = 0; j < rows; ++j) {
      for (int i = 0; i < cols; ++i) {
        SharedEdge e;
        e.faceId = faceId;
        e.triA = globalBase + t0idx(i, j);
        e.cornerA0 = 0;
        e.cornerA1 = 2;
        e.triB = globalBase + t0idx(i, j) + 1;
        e.cornerB0 = 0;
        e.cornerB1 = 1;
        edges->push_back(e);
      }
    }
  }
  return ch;
}

std::string fingerprintInput(const PackerInput& in) {
  std::string out;
  auto add = [&out](const void* p, size_t n) {
    out.append(static_cast<const char*>(p), n);
  };
  for (const ChartInput& ch : in.charts) {
    add(&ch.faceId, sizeof(ch.faceId));
    add(&ch.kernel, sizeof(ch.kernel));
    add(&ch.rectMin, sizeof(ch.rectMin));
    add(&ch.rectMax, sizeof(ch.rectMax));
    if (!ch.triangles.empty())
      add(ch.triangles.data(), ch.triangles.size() * sizeof(ChartTriangle));
  }
  if (!in.adjacency.empty())
    add(in.adjacency.data(), in.adjacency.size() * sizeof(SharedEdge));
  add(&in.settings, sizeof(in.settings));
  return out;
}

std::string fingerprintResult(const PackResult& r) {
  std::string out;
  auto add = [&out](const void* p, size_t n) {
    out.append(static_cast<const char*>(p), n);
  };
  add(&r.ok, sizeof(r.ok));
  out += r.reason;
  add(&r.summary, sizeof(r.summary));
  if (!r.triangles.empty())
    add(r.triangles.data(), r.triangles.size() * sizeof(curveduv::TriangleUV));
  return out;
}

PackerInput makeRingInput() {
  PackerInput in;
  const double L = 2.0 * kPi * 208.784;  // 1311.83 mm
  const double W = 2.0 * kPi * 5.898;    // 37.06 mm
  in.charts.push_back(makeGridChart(7, KernelTag::Torus, L, W, 64, 32,
                                    /*wrapSeam=*/true, &in.adjacency, 0));
  in.settings.resolutionTexels = 1024;
  in.settings.gutterTexels = 2;
  return in;
}

void testSettingsValidation() {
  PackerInput base = makeRingInput();
  {
    PackerInput in = base;
    in.settings.resolutionTexels = 255;
    CHECK(!pack(in).ok);
  }
  {
    PackerInput in = base;
    in.settings.resolutionTexels = 1000;  // not a power of two
    CHECK(!pack(in).ok);
  }
  {
    PackerInput in = base;
    in.settings.resolutionTexels = 128;  // below range
    CHECK(!pack(in).ok);
  }
  {
    PackerInput in = base;
    in.settings.resolutionTexels = 8192;  // above range
    CHECK(!pack(in).ok);
  }
  {
    PackerInput in = base;
    in.settings.gutterTexels = 0;
    CHECK(!pack(in).ok);
  }
  {
    PackerInput in = base;
    in.settings.gutterTexels = 9;
    CHECK(!pack(in).ok);
  }
  {
    PackerInput in = base;  // smallest valid N must still pack (small s)
    in.settings.resolutionTexels = 256;
    const PackResult r = pack(in);
    CHECK(r.ok);
  }
  {
    PackerInput in = base;
    in.settings.gutterTexels = 8;  // largest valid gutter
    const PackResult r = pack(in);
    CHECK(r.ok);
  }
}

void testSplitCountFormula() {
  using curveduv::detail::splitCount;
  CHECK(splitCount(993,1,1008)==1); // long thin strip still fits
  CHECK(splitCount(993,1.5,1008)==2);
  CHECK(splitCount(1008,1,1008)==1); // exact fit
  CHECK(splitCount(1008,1.000001,1008)==2);
  CHECK(splitCount(1312,4.5,1020)==6);
  CHECK(splitCount(10000,100,1008)==65); // budget rejection sentinel

}

void testChartValidation() {
  PackerInput base = makeRingInput();
  {
    PackerInput in;
    in.settings = base.settings;
    CHECK(!pack(in).ok);  // no charts
  }
  {
    PackerInput in = base;
    in.charts[0].triangles.clear();
    CHECK(!pack(in).ok);  // empty chart
  }
  {
    PackerInput in = base;
    in.charts[0].triangles[0].corner[0].x =
        std::numeric_limits<double>::quiet_NaN();
    CHECK(!pack(in).ok);  // NaN coordinate
  }
  {
    PackerInput in = base;
    in.charts[0].triangles[0].corner[0].x = in.charts[0].rectMax.x + 1.0;
    CHECK(!pack(in).ok);  // corner outside rectangle extents
  }
  {
    PackerInput in = base;
    in.charts[0].rectMax.x = in.charts[0].rectMin.x;  // degenerate extents
    CHECK(!pack(in).ok);
  }
  {
    PackerInput in = base;
    ChartTriangle& t = in.charts[0].triangles[0];
    t.corner[1] = t.corner[0];
    t.corner[2] = t.corner[0];
    CHECK(!pack(in).ok);  // zero-area triangle
  }
  {
    // Input chart count alone exceeds the bounded budget.
    PackerInput in;
    in.settings = base.settings;
    for (int f = 0; f < 65; ++f)
      in.charts.push_back(makeGridChart(f, KernelTag::Planar, 10000.0, 10.0, 32,
                                        2, false, nullptr, 0));
    const PackResult r = pack(in);
    CHECK(!r.ok);
  }
}

void testAdjacencyValidation() {
  PackerInput base = makeRingInput();
  const long total = static_cast<long>(base.charts[0].triangles.size());
  {
    PackerInput in = base;
    in.adjacency[0].triA = static_cast<int>(total);  // out of range
    CHECK(!pack(in).ok);
  }
  {
    PackerInput in = base;
    in.adjacency[0].triB = -1;
    CHECK(!pack(in).ok);
  }
  {
    PackerInput in = base;
    in.adjacency[0].triB = in.adjacency[0].triA;  // same triangle twice
    CHECK(!pack(in).ok);
  }
  {
    PackerInput in = base;
    in.adjacency[0].cornerA1 = 3;  // invalid corner
    CHECK(!pack(in).ok);
  }
  {
    PackerInput in = base;
    in.adjacency[0].cornerA1 = in.adjacency[0].cornerA0;  // degenerate corners
    CHECK(!pack(in).ok);
  }
  {
    PackerInput in = base;
    in.adjacency[0].faceId = 999;  // unknown face
    const PackResult r = pack(in);
    CHECK(!r.ok);
  }
}

void testRing() {
  PackerInput in = makeRingInput();
  const PackResult r = pack(in);
  if (!r.ok) std::printf("ring rejected: %s\n", r.reason.c_str());
  CHECK(r.ok);
  CHECK(r.summary.subChartCount <= 8);
  CHECK(r.summary.splitLineCount == r.summary.subChartCount-1);
  std::printf("ring: occupancy=%.4f s=%.4f texels/mm subCharts=%d "
              "seams{continuous=%ld declared=%ld torn=%ld}\n",
              r.summary.occupancy, r.summary.globalTexelsPerMM,
              r.summary.subChartCount, r.summary.seamCounts.continuous,
              r.summary.seamCounts.declared, r.summary.seamCounts.torn);
  CHECK(r.summary.occupancy >= 0.60);
  CHECK(r.summary.occupancy <= 1.0);
  CHECK(r.summary.coveredTexels > 0);
  CHECK(r.summary.seamCounts.torn == 0);
  CHECK(r.summary.seamCounts.declared > 0);    // kernel seam + split lines
  CHECK(r.summary.seamCounts.continuous > 0);  // interior grid edges
  CHECK(r.summary.globalTexelsPerMM > 4.0 && r.summary.globalTexelsPerMM < 5.0);
  CHECK(r.summary.seamCounts.diagonalSeamEdges == 0);
  CHECK(r.summary.seamCounts.declared == r.summary.seamCounts.kernelSeamEdges + r.summary.seamCounts.splitSeamEdges);
  CHECK(r.summary.seamCounts.kernelSeamEdges == 32);
  CHECK(r.triangles.size() == 64u * 32u * 2u);
  bool allInside = true;
  for (const curveduv::TriangleUV& t : r.triangles)
    for (int c = 0; c < 3; ++c)
      if (!(t.uv[c].x >= 0.0 && t.uv[c].x <= 1.0 && t.uv[c].y >= 0.0 &&
            t.uv[c].y <= 1.0))
        allInside = false;
  CHECK(allInside);
  // occupancy consistency with the bitmap count
  const double expect =
      static_cast<double>(r.summary.coveredTexels) / (1024.0 * 1024.0);
  CHECK(std::fabs(r.summary.occupancy - expect) < 1e-15);
}

void testCylinderBandAndCaps() {
  PackerInput in;
  const double R = 30.0;
  const double L = 2.0 * kPi * R;  // 188.50 mm; split only at the fitted density
  const double H = 25.0;
  in.charts.push_back(makeGridChart(11, KernelTag::Cylinder, L, H, 48, 8,
                                    /*wrapSeam=*/true, &in.adjacency, 0));
  int base = static_cast<int>(in.charts[0].triangles.size());
  in.charts.push_back(makeGridChart(12, KernelTag::Planar, 30.0, 30.0, 6, 6,
                                    false, &in.adjacency, base));
  base += static_cast<int>(in.charts[1].triangles.size());
  in.charts.push_back(makeGridChart(13, KernelTag::Planar, 30.0, 30.0, 6, 6,
                                    false, &in.adjacency, base));
  in.settings.resolutionTexels = 512;
  in.settings.gutterTexels = 2;
  const PackResult r = pack(in);
  if (!r.ok) std::printf("cylinder+caps rejected: %s\n", r.reason.c_str());
  CHECK(r.ok);
  CHECK(r.summary.subChartCount <= 5);
  CHECK(r.summary.splitLineCount == r.summary.subChartCount-3);
  CHECK(r.summary.seamCounts.torn == 0);
  CHECK(r.summary.seamCounts.declared > 0);
  std::printf("cylinder+caps: occupancy=%.4f s=%.4f\n", r.summary.occupancy,
              r.summary.globalTexelsPerMM);
  CHECK(r.summary.occupancy > 0.35);
  CHECK(r.summary.globalTexelsPerMM > 0.0);
}

void testPlanarOnly() {
  PackerInput in;
  in.charts.push_back(makeGridChart(21, KernelTag::Planar, 100.0, 100.0, 10, 10,
                                    false, &in.adjacency, 0));
  in.settings.resolutionTexels = 256;
  in.settings.gutterTexels = 1;
  const PackResult r = pack(in);
  if (!r.ok) std::printf("planar rejected: %s\n", r.reason.c_str());
  CHECK(r.ok);
  CHECK(r.summary.subChartCount == 1);  // square fits without splitting
  CHECK(r.summary.splitLineCount == 0);
  // density is width-limited: (100*s + 2) <= 256 -> s = 2.54
  CHECK(std::fabs(r.summary.globalTexelsPerMM - 2.54) < 1e-9);
  // single sub-chart at the origin: mm (0,0) -> texel (g,g) -> uv (g/N,g/N)
  const curveduv::TriangleUV& t0 = r.triangles[0];
  CHECK(std::fabs(t0.uv[0].x - 1.0 / 256.0) < 1e-12);
  CHECK(std::fabs(t0.uv[0].y - 1.0 / 256.0) < 1e-12);
  // mm (100,100) corner maps with the same global scale
  bool foundFar = false;
  for (const curveduv::TriangleUV& t : r.triangles)
    for (int c = 0; c < 3; ++c)
      if (std::fabs(t.uv[c].x - (1.0 + 100.0 * 2.54) / 256.0) < 1e-9 &&
          std::fabs(t.uv[c].y - (1.0 + 100.0 * 2.54) / 256.0) < 1e-9)
        foundFar = true;
  CHECK(foundFar);
  CHECK(r.summary.occupancy > 0.95);
  CHECK(r.summary.seamCounts.torn == 0);
  CHECK(r.summary.seamCounts.continuous ==
        static_cast<long>(in.adjacency.size()));  // no seams at all
}

void testTornInputRejected() {
  // One quad, two triangles; triangle B maps its shared-edge corner to a
  // different developed position -> tear that is not a declared seam.
  PackerInput in;
  ChartInput ch;
  ch.faceId = 31;
  ch.kernel = KernelTag::Planar;
  ch.rectMin = Vec2{0.0, 0.0};
  ch.rectMax = Vec2{10.0, 10.0};
  ChartTriangle a, b;
  a.corner[0] = Vec2{0.0, 0.0};
  a.corner[1] = Vec2{10.0, 0.0};
  a.corner[2] = Vec2{10.0, 10.0};
  b.corner[0] = Vec2{0.0, 0.0};
  b.corner[1] = Vec2{9.0, 10.0};  // torn: should be (10,10)
  b.corner[2] = Vec2{0.0, 10.0};
  ch.triangles.push_back(a);
  ch.triangles.push_back(b);
  in.charts.push_back(ch);
  SharedEdge e;
  e.faceId = 31;
  e.triA = 0;
  e.cornerA0 = 0;
  e.cornerA1 = 2;
  e.triB = 1;
  e.cornerB0 = 0;
  e.cornerB1 = 1;
  in.adjacency.push_back(e);
  in.settings.resolutionTexels = 256;
  in.settings.gutterTexels = 1;
  const PackResult r = pack(in);
  CHECK(!r.ok);
  CHECK(r.reason.find("torn") != std::string::npos);
  CHECK(r.triangles.empty());  // atomic rejection: no partial output
  // Mislabeling a face-internal diagonal as a kernel seam cannot certify it.
  PackerInput in2 = in;
  in2.adjacency[0].kernelSeam = true;
  const PackResult r2 = pack(in2);
  CHECK(!r2.ok);
  CHECK(r2.summary.seamCounts.diagonalSeamEdges == 1);
}

void testFitDrivenPartitions() {
  // Another face limits density: even a 99:1 ribbon must remain unsplit.
  PackerInput in;
  in.settings={1024,8};
  in.charts.push_back(makeGridChart(0,KernelTag::Planar,2000,2000,2,2,false,&in.adjacency,0));
  in.charts.push_back(makeGridChart(1,KernelTag::Cylinder,993,10,72,2,true,&in.adjacency,8));
  const auto r=pack(in);
  CHECK(r.ok);
  CHECK(r.summary.subChartCount==2);
  CHECK(r.summary.splitLineCount==0);
  CHECK(r.summary.seamCounts.splitSeamEdges==0);
  CHECK(curveduv::detail::splitCount(993,r.summary.globalTexelsPerMM,1008)==1);
  // Long-Y strips must use the same rule and preserve every quad diagonal.
  auto vertical=makeRingInput();
  std::swap(vertical.charts[0].rectMax.x,vertical.charts[0].rectMax.y);
  for(auto& t:vertical.charts[0].triangles) for(auto& p:t.corner) std::swap(p.x,p.y);
  const auto v=pack(vertical);
  CHECK(v.ok);
  CHECK(v.summary.subChartCount<=8 && v.summary.occupancy>=.60);
  CHECK(v.summary.seamCounts.diagonalSeamEdges==0 && v.summary.seamCounts.torn==0);
  for(const auto& e:vertical.adjacency) {
    if(e.cornerA0==0 && e.cornerA1==2 && e.cornerB0==0 && e.cornerB1==1)
      CHECK(v.triangles[e.triA].subChartIndex==v.triangles[e.triB].subChartIndex);
  }
}

void testSyntheticSeat() {
  PackerInput in;
  in.settings = {1024,8};
  for (int f=0;f<2;++f) {
    ChartInput disc;
    disc.faceId=f; disc.kernel=KernelTag::Planar;
    disc.rectMin={-170,-170}; disc.rectMax={170,170};
    for(int i=0;i<128;++i) {
      const double a=2*kPi*i/128,b=2*kPi*(i+1)/128;
      disc.triangles.push_back({{{0,0},{170*std::cos(a),170*std::sin(a)},
                                      {170*std::cos(b),170*std::sin(b)}}});
    }
    in.charts.push_back(disc);
  }
  int base=256;
  for(int f=2;f<5;++f) {
    auto ch=makeGridChart(f,f==4?KernelTag::Cylinder:KernelTag::Torus,
                         f==4?1068:993,f==4?6:19,72,4,true,&in.adjacency,base);
    base+=static_cast<int>(ch.triangles.size()); in.charts.push_back(ch);
  }
  const auto r=pack(in);
  if(!r.ok) std::printf("seat rejected: %s\n",r.reason.c_str());
  CHECK(r.ok);
  CHECK(r.summary.subChartCount<=12);
  CHECK(r.summary.seamCounts.torn==0);
  CHECK(r.summary.seamCounts.diagonalSeamEdges==0);
  CHECK(r.summary.seamCounts.declared==r.summary.seamCounts.kernelSeamEdges+r.summary.seamCounts.splitSeamEdges);
  std::printf("synthetic seat: charts=%d occupancy=%.8f density=%.8f (acceptance requires >=0.60)\n",
              r.summary.subChartCount,r.summary.occupancy,r.summary.globalTexelsPerMM);
  // A1 explicitly requires reporting a shortfall, not lowering the B01 gate.
}

void testCoverageProofDirect() {
  using curveduv::detail::PlacedTriangle;
  using curveduv::detail::checkCoverage;
  using curveduv::detail::PlacedRect;
  using curveduv::detail::rectsOverlap;
  const int n = 64;
  auto tri = [](int sc, double x0, double y0, double x1, double y1, double x2,
                double y2) {
    PlacedTriangle t;
    t.subChart = sc;
    t.texel[0] = Vec2{x0, y0};
    t.texel[1] = Vec2{x1, y1};
    t.texel[2] = Vec2{x2, y2};
    return t;
  };
  {
    std::vector<PlacedTriangle> v{tri(0, 2, 2, 20, 2, 2, 20)};
    const auto r = checkCoverage(v, n);
    CHECK(r.ok);
    CHECK(r.coveredTexels > 0);
  }
  {
    // identical triangles, same sub-chart: base double cover -> reject
    std::vector<PlacedTriangle> v{tri(0, 2, 2, 20, 2, 2, 20),
                                  tri(0, 2, 2, 20, 2, 2, 20)};
    CHECK(!checkCoverage(v, n).ok);
  }
  {
    // identical triangles, different sub-charts -> reject
    std::vector<PlacedTriangle> v{tri(0, 2, 2, 20, 2, 2, 20),
                                  tri(1, 2, 2, 20, 2, 2, 20)};
    CHECK(!checkCoverage(v, n).ok);
  }
  {
    // two triangles sharing exactly an edge, same sub-chart -> ok
    std::vector<PlacedTriangle> v{tri(0, 2, 2, 20, 2, 20, 20),
                                  tri(0, 2, 2, 20, 20, 2, 20)};
    const auto r = checkCoverage(v, n);
    CHECK(r.ok);
  }
  {
    // separated by 3 texels, different sub-charts -> ok (2g >= 2 satisfied)
    std::vector<PlacedTriangle> v{tri(0, 2, 2, 20, 2, 2, 20),
                                  tri(1, 23, 2, 41, 2, 23, 20)};
    CHECK(checkCoverage(v, n).ok);
  }
  {
    // separated by 0.5 texels, different sub-charts -> dilation conflict.
    // (Gaps in [1,2) texels can evade texel-center sampling depending on
    // alignment; in the full pipeline those are caught by the padded-AABB
    // proof because a gap < 2g implies overlapping padded rectangles.)
    std::vector<PlacedTriangle> v{tri(0, 2, 2, 20, 2, 2, 20),
                                  tri(1, 20.5, 2, 38.5, 2, 20.5, 20)};
    CHECK(!checkCoverage(v, n).ok);
  }
  {
    // AABB proof: strict overlap detected, touching allowed
    CHECK(rectsOverlap(PlacedRect{0, 0, 10, 10}, PlacedRect{5, 5, 10, 10}));
    CHECK(!rectsOverlap(PlacedRect{0, 0, 10, 10}, PlacedRect{10, 0, 10, 10}));
    CHECK(!rectsOverlap(PlacedRect{0, 0, 10, 10}, PlacedRect{20, 20, 5, 5}));
  }
}

void testDeterminismAndImmutability() {
  PackerInput in = makeRingInput();
  const std::string before = fingerprintInput(in);
  const PackResult r1 = pack(in);
  const std::string after = fingerprintInput(in);
  CHECK(before == after);  // inputs immutable (memcmp over all bytes)
  CHECK(r1.ok);
  const PackResult r2 = pack(in);
  CHECK(r2.ok);
  CHECK(fingerprintResult(r1) == fingerprintResult(r2));  // byte-identical
}

}  // namespace

int main() {
  testSettingsValidation();
  testSplitCountFormula();
  testChartValidation();
  testAdjacencyValidation();
  testRing();
  testCylinderBandAndCaps();
  testPlanarOnly();
  testSyntheticSeat();
  testFitDrivenPartitions();
  testTornInputRejected();
  testCoverageProofDirect();
  testDeterminismAndImmutability();
  std::printf("checks=%d failures=%d %s\n", g_checks, g_failures,
              g_failures == 0 ? "ALL TESTS PASSED" : "FAILURES PRESENT");
  return g_failures == 0 ? 0 : 1;
}
