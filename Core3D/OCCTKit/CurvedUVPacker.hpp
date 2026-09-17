#ifndef CURVED_UV_PACKER_HPP
#define CURVED_UV_PACKER_HPP

// CurvedUVPacker.hpp — standalone prototype of the shared curved-UV chart packer.
// Design source: root-curved-uv-packer-integration-design-20260916.md, decisions
// D2 (strip splitting), D3 (one global texel density + shelf packing + non-overlap
// proof), D4 (seam census). stdlib-only C++17; no OCCT dependency. Plain structs
// in, plain structs out; thin gp_* adapters come later in the N3 package.
//
// Semantics notes (kept explicit so the proof object is reviewable):
//  - Fit-driven splitting (A1): at each candidate density s, use
//    k = max(1, ceil(s*L/(N-2g))). Connected triangles across edges that
//    cannot be a transverse cut stay together, so a split never cuts a quad
//    diagonal. Assign each such group by its developed centroid to k equal
//    segments. Empty segments are dropped; actual triangle bounds must fit.
//  - One global density is bisected within each interval of constant k.
//    Feasibility can jump when k changes, so a failed interval must not hide
//    a higher-density fit. Deterministic height-sorted shelf packing is used.
//  - Non-overlap proof (both required):
//    (a) pairwise padded-AABB test over all placed sub-chart rectangles
//        (touching allowed, strict overlap rejected);
//    (b) coverage bitmap at N x N. Every triangle is rasterized with 1-texel
//        dilation; base coverage uses a half-open (top-left) rule so texel
//        centers on a shared mesh edge are owned by exactly one triangle.
//        Rejections: a texel base-covered twice (true area overlap), or a
//        texel covered by the 1-texel dilation of two DIFFERENT sub-charts
//        (cross-chart proximity below 2 texels). Dilation uses threshold
//        1.0 - 1e-6 texel so exact gutter contact (separation == 2g) is not a
//        false positive while any real cross-chart encroachment is caught.
//        Within one sub-chart the mesh tiles the developed rectangle, so
//        same-sub-chart dilation hits along shared edges are expected and are
//        covered by the base-coverage rule instead.
//  - Seam census: caller-provided adjacency (shared edge -> two triangle
//    corner pairs, global triangle indices in chart order). An edge is
//    continuous if both UV segments coincide within 1e-6 texel (either
//    orientation); else declared if flagged kernelSeam or if the two triangles
//    are in different sub-charts (split line); else torn -> atomic rejection
//    with a reason, no partial output.
//  - Determinism: all iteration orders are fixed (input order, then the
//    explicit packing sort); no hashing, no clock, no platform-dependent
//    behavior beyond IEEE double arithmetic. Inputs are const and never
//    mutated.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace curveduv {

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

enum class KernelTag : int { Planar = 0, Cylinder = 1, Torus = 2, Fallback = 3 };

struct ChartTriangle {
  Vec2 corner[3];
};

struct ChartInput {
  int faceId = 0;
  KernelTag kernel = KernelTag::Planar;
  std::vector<ChartTriangle> triangles;  // developed corner coordinates, mm
  Vec2 rectMin;                          // developed rectangle extents, mm
  Vec2 rectMax;
};

struct SharedEdge {
  int faceId = 0;      // owning face; both triangles must belong to it
  int triA = 0;        // global triangle index (charts flattened in order)
  int cornerA0 = 0;
  int cornerA1 = 0;
  int triB = 0;
  int cornerB0 = 0;
  int cornerB1 = 0;
  bool kernelSeam = false;  // caller-declared kernel seam (one per closed direction)
};

struct PackSettings {
  int resolutionTexels = 1024;  // N, power of two in [256, 4096]
  int gutterTexels = 2;         // g in [1, 8]
};

struct PackerInput {
  std::vector<ChartInput> charts;
  std::vector<SharedEdge> adjacency;
  PackSettings settings;
};

struct TriangleUV {
  int faceId = 0;
  int chartIndex = 0;
  int triangleIndex = 0;  // local index within its chart
  int subChartIndex = 0;  // actual fitted partition; never recompute from aspect
  Vec2 uv[3];             // [0,1]^2
};

struct SeamCounts {
  long continuous = 0;
  long declared = 0;
  long torn = 0;
  long kernelSeamEdges = 0;
  long splitSeamEdges = 0;
  long diagonalSeamEdges = 0;
};

struct PackSummary {
  int subChartCount = 0;
  int splitLineCount = 0;  // sum over charts of (nonempty fitted segments - 1)
  double occupancy = 0.0;  // covered texels / N^2, post gutter
  long coveredTexels = 0;
  double globalTexelsPerMM = 0.0;
  SeamCounts seamCounts;
};

struct PackResult {
  bool ok = false;
  std::string reason;  // empty on success; rejection cause otherwise
  std::vector<TriangleUV> triangles;
  PackSummary summary;
};

namespace detail {

inline Vec2 vsub(const Vec2& a, const Vec2& b) { return Vec2{a.x - b.x, a.y - b.y}; }
inline double vcross(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }

// Sentinel 65 means the bounded atlas budget cannot admit this candidate.
inline int splitCount(double L, double s, double availableTexels) {
  const double k = std::ceil(s * L / availableTexels);
  return k > 64 || !std::isfinite(k) ? 65 : std::max(1, static_cast<int>(k));
}

struct PlacedRect {
  double x = 0.0, y = 0.0, w = 0.0, h = 0.0;  // padded, texels
};

// Strict overlap; touching edges do not count.
inline bool rectsOverlap(const PlacedRect& a, const PlacedRect& b) {
  return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
}

// Half-open point-in-triangle (top-left rule) in texel space. Boundary texel
// centers are owned by exactly one of the two triangles sharing an edge.
inline bool baseContains(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& p) {
  Vec2 v[3] = {a, b, c};
  if (vcross(vsub(b, a), vsub(c, a)) < 0.0) std::swap(v[1], v[2]);  // force CCW
  for (int i = 0; i < 3; ++i) {
    const Vec2 d = vsub(v[(i + 1) % 3], v[i]);
    const double e = vcross(d, vsub(p, v[i]));
    const double band = 1e-9 * (d.x * d.x + d.y * d.y + 1.0);
    if (e > band) continue;
    if (e < -band) return false;
    const bool owns = (d.y < 0.0) || (d.y == 0.0 && d.x < 0.0);  // top-left
    if (!owns) return false;
  }
  return true;
}

inline double pointSegDist2(const Vec2& p, const Vec2& a, const Vec2& b) {
  const Vec2 d = vsub(b, a);
  const Vec2 q = vsub(p, a);
  const double l2 = d.x * d.x + d.y * d.y;
  double t = (l2 > 0.0) ? (q.x * d.x + q.y * d.y) / l2 : 0.0;
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;
  const double dx = q.x - t * d.x;
  const double dy = q.y - t * d.y;
  return dx * dx + dy * dy;
}

// Squared distance from p to the triangle (0 inside, edge band inclusive).
inline double triDist2(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& p) {
  Vec2 v[3] = {a, b, c};
  if (vcross(vsub(b, a), vsub(c, a)) < 0.0) std::swap(v[1], v[2]);
  bool inside = true;
  for (int i = 0; i < 3; ++i) {
    const Vec2 d = vsub(v[(i + 1) % 3], v[i]);
    const double e = vcross(d, vsub(p, v[i]));
    const double band = 1e-9 * (d.x * d.x + d.y * d.y + 1.0);
    if (e < -band) inside = false;
  }
  if (inside) return 0.0;
  double best = pointSegDist2(p, v[0], v[1]);
  best = std::min(best, pointSegDist2(p, v[1], v[2]));
  best = std::min(best, pointSegDist2(p, v[2], v[0]));
  return best;
}

struct PlacedTriangle {
  int subChart = 0;  // 0..63
  Vec2 texel[3];     // texel-space coordinates (not normalized)
};

struct CoverageResult {
  bool ok = false;
  std::string reason;
  long coveredTexels = 0;
};

// Coverage-bitmap proof at n x n. See file header for the exact semantics.
inline CoverageResult checkCoverage(const std::vector<PlacedTriangle>& tris, int n) {
  static const uint8_t kEmpty = 0xFF;
  const double dil2 = (1.0 - 1e-6) * (1.0 - 1e-6);
  std::vector<uint8_t> base(static_cast<size_t>(n) * static_cast<size_t>(n), kEmpty);
  std::vector<uint8_t> dil(static_cast<size_t>(n) * static_cast<size_t>(n), kEmpty);
  long covered = 0;
  for (size_t ti = 0; ti < tris.size(); ++ti) {
    const PlacedTriangle& t = tris[ti];
    if (t.subChart < 0 || t.subChart > 254) {
      return CoverageResult{false, "coverage: sub-chart id out of range", covered};
    }
    const uint8_t sc = static_cast<uint8_t>(t.subChart);
    double minx = t.texel[0].x, maxx = t.texel[0].x;
    double miny = t.texel[0].y, maxy = t.texel[0].y;
    for (int k = 1; k < 3; ++k) {
      minx = std::min(minx, t.texel[k].x);
      maxx = std::max(maxx, t.texel[k].x);
      miny = std::min(miny, t.texel[k].y);
      maxy = std::max(maxy, t.texel[k].y);
    }
    // texel centers are at (i + 0.5, j + 0.5); expand the bbox by 1 texel.
    int i0 = static_cast<int>(std::ceil(minx - 1.5));
    int i1 = static_cast<int>(std::floor(maxx + 0.5));
    int j0 = static_cast<int>(std::ceil(miny - 1.5));
    int j1 = static_cast<int>(std::floor(maxy + 0.5));
    if (i0 < 0) i0 = 0;
    if (j0 < 0) j0 = 0;
    if (i1 > n - 1) i1 = n - 1;
    if (j1 > n - 1) j1 = n - 1;
    for (int j = j0; j <= j1; ++j) {
      for (int i = i0; i <= i1; ++i) {
        const Vec2 p{static_cast<double>(i) + 0.5, static_cast<double>(j) + 0.5};
        const bool inBase = baseContains(t.texel[0], t.texel[1], t.texel[2], p);
        const bool inDil =
            inBase || triDist2(t.texel[0], t.texel[1], t.texel[2], p) < dil2;
        if (!inBase && !inDil) continue;
        const size_t idx = static_cast<size_t>(j) * static_cast<size_t>(n) +
                           static_cast<size_t>(i);
        if (inBase) {
          if (base[idx] != kEmpty) {
            return CoverageResult{false,
                                  "coverage: texel covered twice (area overlap) at (" +
                                      std::to_string(i) + "," + std::to_string(j) + ")",
                                  covered};
          }
          base[idx] = sc;
          ++covered;
        }
        if (inDil) {
          if (base[idx] != kEmpty && base[idx] != sc) {
            return CoverageResult{false,
                                  "coverage: dilated cross-sub-chart hit at (" +
                                      std::to_string(i) + "," + std::to_string(j) + ")",
                                  covered};
          }
          if (dil[idx] != kEmpty && dil[idx] != sc) {
            return CoverageResult{false,
                                  "coverage: dilated cross-sub-chart hit at (" +
                                      std::to_string(i) + "," + std::to_string(j) + ")",
                                  covered};
          }
          dil[idx] = sc;
        }
      }
    }
  }
  return CoverageResult{true, std::string(), covered};
}

}  // namespace detail

inline PackResult pack(const PackerInput& input) {
  PackResult out;
  const int n = input.settings.resolutionTexels;
  const int g = input.settings.gutterTexels;

  // --- settings validation -------------------------------------------------
  if (n < 256 || n > 4096 || (n & (n - 1)) != 0) {
    out.reason = "invalid-settings: resolutionTexels must be a power of two in [256,4096]";
    return out;
  }
  if (g < 1 || g > 8) {
    out.reason = "invalid-settings: gutterTexels must be in [1,8]";
    return out;
  }
  if (input.charts.empty()) {
    out.reason = "invalid-input: no charts";
    return out;
  }

  // --- chart validation ----------------------------------------------------
  long totalTriangles = 0;
  long totalSplitLines = 0;
  std::vector<long> chartOffset(input.charts.size(), 0);
  for (size_t ci = 0; ci < input.charts.size(); ++ci) {
    const ChartInput& ch = input.charts[ci];
    chartOffset[ci] = totalTriangles;
    if (ch.triangles.empty()) {
      out.reason = "invalid-input: chart faceId=" + std::to_string(ch.faceId) + " has no triangles";
      return out;
    }
    const double w = ch.rectMax.x - ch.rectMin.x;
    const double h = ch.rectMax.y - ch.rectMin.y;
    if (!std::isfinite(ch.rectMin.x) || !std::isfinite(ch.rectMin.y) ||
        !std::isfinite(ch.rectMax.x) || !std::isfinite(ch.rectMax.y) || w <= 0.0 ||
        h <= 0.0) {
      out.reason = "invalid-input: chart faceId=" + std::to_string(ch.faceId) +
                   " has degenerate or non-finite rectangle extents";
      return out;
    }
    for (size_t ti = 0; ti < ch.triangles.size(); ++ti) {
      const ChartTriangle& t = ch.triangles[ti];
      for (int k = 0; k < 3; ++k) {
        const Vec2& c = t.corner[k];
        if (!std::isfinite(c.x) || !std::isfinite(c.y)) {
          out.reason = "invalid-input: non-finite coordinate in chart faceId=" +
                       std::to_string(ch.faceId);
          return out;
        }
        const double tol = 1e-9;
        if (c.x < ch.rectMin.x - tol || c.x > ch.rectMax.x + tol ||
            c.y < ch.rectMin.y - tol || c.y > ch.rectMax.y + tol) {
          out.reason = "invalid-input: triangle corner outside rectangle extents, faceId=" +
                       std::to_string(ch.faceId);
          return out;
        }
      }
      const double area2 = detail::vcross(detail::vsub(t.corner[1], t.corner[0]),
                                          detail::vsub(t.corner[2], t.corner[0]));
      if (std::fabs(area2) <= 1e-12) {
        out.reason = "invalid-input: degenerate (zero-area) triangle in chart faceId=" +
                     std::to_string(ch.faceId);
        return out;
      }
    }
    totalTriangles += static_cast<long>(ch.triangles.size());
  }
  if (input.charts.size() > 64) {
    out.reason = "invalid-input: sub-chart budget exceeded (>64)";
    return out;
  }

  // --- adjacency validation -------------------------------------------------
  for (size_t ei = 0; ei < input.adjacency.size(); ++ei) {
    const SharedEdge& e = input.adjacency[ei];
    if (e.triA < 0 || e.triB < 0 || e.triA >= totalTriangles || e.triB >= totalTriangles) {
      out.reason = "invalid-input: adjacency edge " + std::to_string(ei) +
                   " references a triangle out of range";
      return out;
    }
    if (e.triA == e.triB) {
      out.reason = "invalid-input: adjacency edge " + std::to_string(ei) +
                   " references the same triangle twice";
      return out;
    }
    const int corners[4] = {e.cornerA0, e.cornerA1, e.cornerB0, e.cornerB1};
    bool badCorner = false;
    for (int k = 0; k < 4; ++k)
      if (corners[k] < 0 || corners[k] > 2) badCorner = true;
    if (badCorner || e.cornerA0 == e.cornerA1 || e.cornerB0 == e.cornerB1) {
      out.reason = "invalid-input: adjacency edge " + std::to_string(ei) +
                   " has invalid corner indices";
      return out;
    }
    // locate owning charts of both triangles
    int chartA = -1, chartB = -1;
    for (size_t ci = 0; ci < input.charts.size(); ++ci) {
      const long lo = chartOffset[ci];
      const long hi = lo + static_cast<long>(input.charts[ci].triangles.size());
      if (e.triA >= lo && e.triA < hi) chartA = static_cast<int>(ci);
      if (e.triB >= lo && e.triB < hi) chartB = static_cast<int>(ci);
    }
    if (chartA < 0 || chartB < 0 || chartA != chartB ||
        input.charts[static_cast<size_t>(chartA)].faceId != e.faceId) {
      out.reason = "invalid-input: adjacency edge " + std::to_string(ei) +
                   " has unknown or mismatched faceId";
      return out;
    }
  }

  // A split follows transverse mesh edges. Union everything else before
  // centroid assignment, preventing diagonal cuts and hidden developed tears.
  std::vector<int> group(static_cast<size_t>(totalTriangles));
  for (int i = 0; i < totalTriangles; ++i) group[i] = i;
  auto root = [&group](int i) {
    while (group[i] != i) { group[i] = group[group[i]]; i = group[i]; }
    return i;
  };
  for (const auto& e : input.adjacency) {
    size_t ci = 0;
    while (ci + 1 < chartOffset.size() && e.triA >= chartOffset[ci + 1]) ++ci;
    const auto& ch = input.charts[ci];
    const auto& a = ch.triangles[e.triA - chartOffset[ci]];
    const auto& b = ch.triangles[e.triB - chartOffset[ci]];
    auto close = [](Vec2 p, Vec2 q) {
      return std::abs(p.x-q.x) <= 1e-8 && std::abs(p.y-q.y) <= 1e-8;
    };
    if (!e.kernelSeam && !((close(a.corner[e.cornerA0],b.corner[e.cornerB0]) &&
                            close(a.corner[e.cornerA1],b.corner[e.cornerB1])) ||
                           (close(a.corner[e.cornerA0],b.corner[e.cornerB1]) &&
                            close(a.corner[e.cornerA1],b.corner[e.cornerB0])))) {
      out.reason = "torn-shared-edge: developed input discontinuity";
      out.summary.seamCounts.torn = 1;
      const auto delta = detail::vsub(a.corner[e.cornerA0],a.corner[e.cornerA1]);
      out.summary.seamCounts.diagonalSeamEdges =
          std::abs(delta.x)>1e-8 && std::abs(delta.y)>1e-8 ? 1 : 0;
      return out;
    }
    const bool longX = ch.rectMax.x-ch.rectMin.x >= ch.rectMax.y-ch.rectMin.y;
    const Vec2 delta = detail::vsub(a.corner[e.cornerA0],a.corner[e.cornerA1]);
    if (!e.kernelSeam && std::abs(longX ? delta.x : delta.y) > 1e-8)
      group[root(e.triB)] = root(e.triA);
  }
  std::vector<double> groupCentroid(static_cast<size_t>(totalTriangles),0);
  std::vector<int> groupCorners(static_cast<size_t>(totalTriangles),0);
  for (size_t ci = 0; ci < input.charts.size(); ++ci) {
    const auto& ch = input.charts[ci];
    const bool longX = ch.rectMax.x-ch.rectMin.x >= ch.rectMax.y-ch.rectMin.y;
    for (size_t ti = 0; ti < ch.triangles.size(); ++ti) {
      int id = root(static_cast<int>(chartOffset[ci]+ti));
      for (const auto& p : ch.triangles[ti].corner) {
        groupCentroid[id] += longX ? p.x : p.y;
        ++groupCorners[id];
      }
    }
  }
  for (size_t i = 0; i < group.size(); ++i)
    if (groupCorners[i]) groupCentroid[i] /= groupCorners[i];

  // --- strip splitting (D2) -------------------------------------------------
  struct SubChart {
    int chartIndex = 0;
    int faceId = 0;
    int segment = 0;
    std::vector<int> triangles;  // local triangle indices
    Vec2 origin;                 // bbox min of assigned triangles, mm
    Vec2 size;                   // bbox extents, mm
    double wTex = 0.0, hTex = 0.0;  // padded texel size
    double px = 0.0, py = 0.0;      // placement, texels
  };
  std::vector<SubChart> subs;
  const double N = static_cast<double>(n), pad = 2.0*g;
  auto tryPack = [&](double s) -> bool {
    subs.clear(); totalSplitLines = 0;
    int budget = 0;
    for (size_t ci = 0; ci < input.charts.size(); ++ci) {
      const ChartInput& ch = input.charts[ci];
      const double w = ch.rectMax.x - ch.rectMin.x;
      const double h = ch.rectMax.y - ch.rectMin.y;
      const bool longX = w >= h;
      const double L = longX ? w : h;
      const int k = detail::splitCount(L,s,N-pad);
      budget += k;
      if (budget > 64) return false;
      const size_t start = subs.size();
      const double lo0 = longX ? ch.rectMin.x : ch.rectMin.y;
      std::vector<std::vector<int>> bySegment(static_cast<size_t>(k));
      for (size_t ti = 0; ti < ch.triangles.size(); ++ti) {
        const double centroidLong = groupCentroid[root(static_cast<int>(chartOffset[ci]+ti))];
        int seg = static_cast<int>(std::floor((centroidLong - lo0) / L * k));
        if (seg < 0) seg = 0;
        if (seg > k - 1) seg = k - 1;
        bySegment[static_cast<size_t>(seg)].push_back(static_cast<int>(ti));
      }
      for (int seg = 0; seg < k; ++seg) {
        const std::vector<int>& tris = bySegment[static_cast<size_t>(seg)];
        if (tris.empty()) continue;  // sparse chart: empty segments are dropped
        SubChart sc;
        sc.chartIndex = static_cast<int>(ci);
        sc.faceId = ch.faceId;
        sc.segment = seg;
        sc.triangles = tris;
        Vec2 mn{0, 0}, mx{0, 0};
        bool first = true;
        for (int ti : tris) {
          for (int c = 0; c < 3; ++c) {
            const Vec2& p = ch.triangles[static_cast<size_t>(ti)].corner[c];
            if (first) {
              mn = p;
              mx = p;
              first = false;
            } else {
              mn.x = std::min(mn.x, p.x);
              mn.y = std::min(mn.y, p.y);
              mx.x = std::max(mx.x, p.x);
              mx.y = std::max(mx.y, p.y);
            }
          }
        }
        sc.origin = mn;
        sc.size = Vec2{mx.x - mn.x, mx.y - mn.y};
        subs.push_back(sc);
      }
      totalSplitLines += static_cast<long>(subs.size()-start)-1;
    }
    if (subs.empty()) return false;

    // deterministic packing order: padded height desc (equivalently mm height
    // desc, s-invariant), then faceId, chart index, sub-chart index.
    std::vector<size_t> order(subs.size());
    for (size_t i = 0; i < subs.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&subs](size_t a, size_t b) {
      const SubChart& A = subs[a];
      const SubChart& B = subs[b];
      if (A.size.y != B.size.y) return A.size.y > B.size.y;
      if (A.faceId != B.faceId) return A.faceId < B.faceId;
      if (A.chartIndex != B.chartIndex) return A.chartIndex < B.chartIndex;
      return A.segment < B.segment;
    });

    double x = 0.0, y = 0.0, rowH = 0.0;
    for (size_t oi = 0; oi < order.size(); ++oi) {
      SubChart& sc = subs[order[oi]];
      sc.wTex = sc.size.x * s + pad;
      sc.hTex = sc.size.y * s + pad;
      if (sc.wTex > N) return false;
      if (x + sc.wTex > N) {
        y += rowH;
        x = 0.0;
        rowH = 0.0;
      }
      if (y + sc.hTex > N) return false;
      sc.px = x;
      sc.py = y;
      x += sc.wTex;
      if (sc.hTex > rowH) rowH = sc.hTex;
    }
    return true;
  };

  // Enumerate density intervals where k is constant. A single bisection
  // over all densities is invalid: introducing a split can restore a fit.
  double area = 0;
  for (const auto& ch : input.charts) for (const auto& t : ch.triangles)
    area += std::abs(detail::vcross(detail::vsub(t.corner[1],t.corner[0]),
                                   detail::vsub(t.corner[2],t.corner[0]))) * 0.5;
  const double upper = N/std::sqrt(area); // even without gutters, area must fit
  std::vector<double> boundaries{0,upper};
  for (const auto& ch : input.charts) {
    const double L = std::max(ch.rectMax.x-ch.rectMin.x,ch.rectMax.y-ch.rectMin.y);
    for (int k=1;k<=64;++k) {
      const double boundary = k*(N-pad)/L;
      if (boundary < upper) boundaries.push_back(boundary);
    }
  }
  std::sort(boundaries.begin(),boundaries.end());
  boundaries.erase(std::unique(boundaries.begin(),boundaries.end()),boundaries.end());
  double s = 0;
  for (size_t i=1;i<boundaries.size();++i) {
    // Stay inside the interval even when multiplication rounds across a k boundary.
    double lo = boundaries[i-1] + (boundaries[i]-boundaries[i-1])*1e-12;
    double hi = boundaries[i] - (boundaries[i]-boundaries[i-1])*1e-12;
    if (!tryPack(lo)) continue;
    if (tryPack(hi)) { s=hi; continue; }
    for (int it=0;it<48;++it) {
      const double mid=0.5*(lo+hi);
      if (tryPack(mid)) lo=mid; else hi=mid;
    }
    s=std::max(s,lo);
  }
  if (!(s > 0) || !tryPack(s)) {
    out.reason = "internal-error: final density does not pack";
    return out;
  }

  // --- proof (a): pairwise padded-AABB --------------------------------------
  for (size_t i = 0; i < subs.size(); ++i) {
    for (size_t j = i + 1; j < subs.size(); ++j) {
      const detail::PlacedRect a{subs[i].px, subs[i].py, subs[i].wTex, subs[i].hTex};
      const detail::PlacedRect b{subs[j].px, subs[j].py, subs[j].wTex, subs[j].hTex};
      if (detail::rectsOverlap(a, b)) {
        out.reason = "internal-error: padded-AABB overlap between sub-charts " +
                     std::to_string(i) + " and " + std::to_string(j);
        return out;
      }
    }
  }

  // --- per-triangle texel coordinates ---------------------------------------
  // texelCoord[chart][localTri].corner[c]
  struct TriTexel {
    int subChart = 0;
    Vec2 texel[3];
  };
  std::vector<std::vector<TriTexel>> texels(input.charts.size());
  std::vector<int> triToSub(static_cast<size_t>(totalTriangles), -1);
  for (size_t si = 0; si < subs.size(); ++si) {
    const SubChart& sc = subs[si];
    for (int ti : sc.triangles) {
      TriTexel tt;
      tt.subChart = static_cast<int>(si);
      const ChartTriangle& t =
          input.charts[static_cast<size_t>(sc.chartIndex)].triangles[static_cast<size_t>(ti)];
      for (int c = 0; c < 3; ++c) {
        tt.texel[c] = Vec2{sc.px + static_cast<double>(g) + (t.corner[c].x - sc.origin.x) * s,
                           sc.py + static_cast<double>(g) + (t.corner[c].y - sc.origin.y) * s};
      }
      if (texels[static_cast<size_t>(sc.chartIndex)].empty())
        texels[static_cast<size_t>(sc.chartIndex)].resize(
            input.charts[static_cast<size_t>(sc.chartIndex)].triangles.size());
      texels[static_cast<size_t>(sc.chartIndex)][static_cast<size_t>(ti)] = tt;
      triToSub[chartOffset[static_cast<size_t>(sc.chartIndex)] + ti] = static_cast<int>(si);
    }
  }

  // --- proof (b): coverage bitmap with 1-texel dilation ----------------------
  std::vector<detail::PlacedTriangle> placed;
  placed.reserve(static_cast<size_t>(totalTriangles));
  for (size_t si = 0; si < subs.size(); ++si) {
    const SubChart& sc = subs[si];
    for (int ti : sc.triangles) {
      const TriTexel& tt = texels[static_cast<size_t>(sc.chartIndex)][static_cast<size_t>(ti)];
      detail::PlacedTriangle pt;
      pt.subChart = static_cast<int>(si);
      for (int c = 0; c < 3; ++c) pt.texel[c] = tt.texel[c];
      placed.push_back(pt);
    }
  }
  const detail::CoverageResult cov = detail::checkCoverage(placed, n);
  if (!cov.ok) {
    out.reason = cov.reason;
    return out;
  }

  // --- seam census (D4) ------------------------------------------------------
  SeamCounts counts;
  std::string tornReason;
  const double tol = 1e-6;  // texels
  for (size_t ei = 0; ei < input.adjacency.size(); ++ei) {
    const SharedEdge& e = input.adjacency[ei];
    // map global triangle -> (chart, local)
    int chartOf = -1;
    long localA = 0, localB = 0;
    for (size_t ci = 0; ci < input.charts.size(); ++ci) {
      const long loT = chartOffset[ci];
      const long hiT = loT + static_cast<long>(input.charts[ci].triangles.size());
      if (e.triA >= loT && e.triA < hiT) {
        chartOf = static_cast<int>(ci);
        localA = e.triA - loT;
      }
      if (e.triB >= loT && e.triB < hiT) localB = e.triB - loT;
    }
    const TriTexel& A = texels[static_cast<size_t>(chartOf)][static_cast<size_t>(localA)];
    const TriTexel& B = texels[static_cast<size_t>(chartOf)][static_cast<size_t>(localB)];
    const Vec2& a0 = A.texel[e.cornerA0];
    const Vec2& a1 = A.texel[e.cornerA1];
    const Vec2& b0 = B.texel[e.cornerB0];
    const Vec2& b1 = B.texel[e.cornerB1];
    auto close = [tol](const Vec2& p, const Vec2& q) {
      return std::fabs(p.x - q.x) <= tol && std::fabs(p.y - q.y) <= tol;
    };
    const bool continuous =
        (close(a0, b0) && close(a1, b1)) || (close(a0, b1) && close(a1, b0));
    if (continuous) {
      ++counts.continuous;
    } else if (e.kernelSeam || triToSub[static_cast<size_t>(e.triA)] !=
                                   triToSub[static_cast<size_t>(e.triB)]) {
      const auto& ch = input.charts[static_cast<size_t>(chartOf)];
      const auto& raw = ch.triangles[static_cast<size_t>(localA)];
      const auto delta = detail::vsub(raw.corner[e.cornerA0],raw.corner[e.cornerA1]);
      if (std::abs(delta.x)>1e-8 && std::abs(delta.y)>1e-8) {
        ++counts.diagonalSeamEdges;
        ++counts.torn;
        tornReason = "torn-shared-edge: face-internal diagonal seam";
      } else {
        ++counts.declared;
        if (e.kernelSeam) ++counts.kernelSeamEdges;
        else ++counts.splitSeamEdges;
      }
    } else {
      ++counts.torn;
      const auto& raw = input.charts[static_cast<size_t>(chartOf)].triangles[static_cast<size_t>(localA)];
      const auto delta = detail::vsub(raw.corner[e.cornerA0],raw.corner[e.cornerA1]);
      if (std::abs(delta.x)>1e-8 && std::abs(delta.y)>1e-8) ++counts.diagonalSeamEdges;
      if (tornReason.empty()) {
        tornReason = "torn-shared-edge: adjacency edge " + std::to_string(ei) +
                     " faceId=" + std::to_string(e.faceId) + " triA=" +
                     std::to_string(e.triA) + " triB=" + std::to_string(e.triB);
      }
    }
  }
  if (counts.torn > 0) {
    out.reason = tornReason + " (torn=" + std::to_string(counts.torn) + ")";
    out.summary.seamCounts = counts;
    return out;
  }

  // --- output ----------------------------------------------------------------
  out.triangles.reserve(static_cast<size_t>(totalTriangles));
  for (size_t ci = 0; ci < input.charts.size(); ++ci) {
    const ChartInput& ch = input.charts[ci];
    for (size_t ti = 0; ti < ch.triangles.size(); ++ti) {
      const TriTexel& tt = texels[ci][ti];
      TriangleUV r;
      r.faceId = ch.faceId;
      r.chartIndex = static_cast<int>(ci);
      r.triangleIndex = static_cast<int>(ti);
      r.subChartIndex = tt.subChart;
      for (int c = 0; c < 3; ++c) r.uv[c] = Vec2{tt.texel[c].x / N, tt.texel[c].y / N};
      out.triangles.push_back(r);
    }
  }
  out.summary.subChartCount = static_cast<int>(subs.size());
  out.summary.splitLineCount = static_cast<int>(totalSplitLines);
  out.summary.coveredTexels = cov.coveredTexels;
  out.summary.occupancy = static_cast<double>(cov.coveredTexels) / (N * N);
  out.summary.globalTexelsPerMM = s;
  out.summary.seamCounts = counts;
  out.ok = true;
  return out;
}

}  // namespace curveduv

#endif  // CURVED_UV_PACKER_HPP
