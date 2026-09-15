#pragma once
// Convex planar-region inset candidate only. The OCAF owner separately owns
// identity, placement, partition persistence, history and publication.
#include "NativeMeshRegionExtrude.hxx"

namespace core3d::meshedit {
struct RegionInsetCandidate {
  TopoDS_Shape shape;
  RegionPartition partition;
  std::uint32_t centerSeed = 0;
  std::vector<Point> innerBoundary;
};
namespace inset_detail {
using UV = std::array<double, 2>;
using Normal = std::array<float, 3>;
inline Point Sub(const Point &a, const Point &b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
inline double Dot(const Point &a, const Point &b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline Point Cross(const Point &a, const Point &b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
inline Point Scale(const Point &a, double s) {
  return {a[0] * s, a[1] * s, a[2] * s};
}
inline Point Add(const Point &a, const Point &b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
struct AffineUV {
  Point origin, x, y;
  UV base, dx, dy;
};
inline bool MakeAffineUV(const NativeTopologyCapture &source,
                         const PlanarRegion &region, int prefix,
                         AffineUV &out) {
  if (region.triangles.empty())
    return false;
  const auto t = region.triangles.front();
  if (t >= source.storedTriangles.size())
    return false;
  const auto &p = source.storedTriangles[t];
  const Point e = Sub(p[1], p[0]), f = Sub(p[2], p[0]);
  const double ee = Dot(e, e), ef = Dot(e, f), ff = Dot(f, f),
               det = ee * ff - ef * ef;
  if (!std::isfinite(det) || det <= ee * ff * 1.e-20)
    return false;
  auto uv = [&](int c) -> UV {
    const int n = source.triangleNodeIDs[t][c];
    return n > prefix && n <= int(source.storedUVs.size())
               ? source.storedUVs[n - 1]
               : UV{INFINITY, INFINITY};
  };
  const UV a = uv(0), b = uv(1), c = uv(2);
  for (double q : {a[0], a[1], b[0], b[1], c[0], c[1]})
    if (!std::isfinite(q))
      return false;
  out = {p[0], e, f, a, {b[0] - a[0], b[1] - a[1]}, {c[0] - a[0], c[1] - a[1]}};
  auto evaluate = [&](const Point &q) {
    const Point d = Sub(q, out.origin);
    const double de = Dot(d, e), df = Dot(d, f);
    const double u = (de * ff - df * ef) / det, v = (df * ee - de * ef) / det;
    return UV{a[0] + u * out.dx[0] + v * out.dy[0],
              a[1] + u * out.dx[1] + v * out.dy[1]};
  };
  double scale = 1;
  for (auto id : region.triangles)
    for (int k = 0; k < 3; ++k) {
      const int n = source.triangleNodeIDs[id][k];
      if (n <= prefix || n > int(source.storedUVs.size()))
        return false;
      const auto expected = source.storedUVs[n - 1],
                 actual = evaluate(source.storedTriangles[id][k]);
      for (int d = 0; d < 2; ++d) {
        scale = std::max(scale, std::abs(expected[d]));
        if (std::abs(actual[d] - expected[d]) > scale * 1.e-10)
          return false;
      }
    }
  return true;
}
inline UV Evaluate(const AffineUV &a, const Point &q) {
  const Point d = Sub(q, a.origin);
  const double ee = Dot(a.x, a.x), ef = Dot(a.x, a.y), ff = Dot(a.y, a.y),
               det = ee * ff - ef * ef;
  const double de = Dot(d, a.x), df = Dot(d, a.y),
               u = (de * ff - df * ef) / det, v = (df * ee - de * ef) / det;
  return {a.base[0] + u * a.dx[0] + v * a.dy[0],
          a.base[1] + u * a.dx[1] + v * a.dy[1]};
}
inline TopologyResult Build(const NativeTopologyCapture &source,
                            const std::vector<Triangle> &triangles,
                            const std::vector<std::array<UV, 3>> &uvs,
                            const std::vector<std::array<Normal, 3>> &normals,
                            TopoDS_Shape &out,
                            const std::atomic_bool &cancelled) {
  out.Nullify();
  if (triangles.empty() || triangles.size() > 4096 ||
      uvs.size() != triangles.size() || normals.size() != triangles.size())
    return TopologyResult::Invalid;
  Handle(Poly_Triangulation) mesh = new Poly_Triangulation(
      int(3 * triangles.size()), int(triangles.size()), true, true);
  mesh->Deflection(source.deflection);
  for (std::size_t t = 0; t < triangles.size(); ++t) {
    if (cancelled.load(std::memory_order_relaxed))
      return TopologyResult::Cancelled;
    for (int k = 0; k < 3; ++k) {
      const int n = int(3 * t) + k + 1;
      mesh->SetNode(n, gp_Pnt(triangles[t][k][0], triangles[t][k][1],
                              triangles[t][k][2]));
      mesh->SetUVNode(n, gp_Pnt2d(uvs[t][k][0], uvs[t][k][1]));
      mesh->SetNormal(
          n, gp_Vec3f(normals[t][k][0], normals[t][k][1], normals[t][k][2]));
    }
    mesh->SetTriangle(int(t) + 1, Poly_Triangle(int(3 * t) + 1, int(3 * t) + 2,
                                                int(3 * t) + 3));
  }
  mesh->UpdateCachedMinMax();
  BRepBuilderAPI_Copy copy(source.shape, Standard_True, Standard_True);
  auto candidate = copy.Shape();
  NativeMeshStorageCapture captured;
  if (candidate.IsNull())
    return TopologyResult::Invalid;
  const auto capture = CaptureNativeMeshStorage(candidate, captured, cancelled);
  if (capture != TopologyResult::Ready)
    return capture;
  BRep_Builder().UpdateFace(captured.face, mesh);
  out = candidate;
  return TopologyResult::Ready;
}
} // namespace inset_detail
inline TopologyResult PrepareConvexRegionInset(
    const NativeTopologyCapture &source, int prefix, const PlanarRegion &region,
    double localDistance, const RegionPartition *sourcePartition,
    RegionInsetCandidate &output, const std::atomic_bool &cancelled) noexcept {
  output = {};
  try {
    using namespace inset_detail;
    if (cancelled.load(std::memory_order_relaxed))
      return TopologyResult::Cancelled;
    const std::size_t n = region.boundaryVertices.size();
    if (!std::isfinite(localDistance) || localDistance <= 1.e-6 ||
        localDistance > 1.e5 || n < 3 || n > 64 || region.triangles.empty() ||
        region.triangles.size() > 256 || source.sourceMesh.IsNull())
      return TopologyResult::Invalid;
    NativeTopologyCapture fresh;
    auto state = CaptureNativeTopology(source.shape, fresh, cancelled);
    if (state != TopologyResult::Ready)
      return state;
    if (!SameRegionTopology(source, fresh) || fresh.sourceMesh.IsNull() ||
        !fresh.sourceMesh->HasUVNodes() ||
        !HasFlatCornerLayout(fresh, prefix) ||
        (sourcePartition && !ValidateRegionPartition(fresh, *sourcePartition)))
      return TopologyResult::Invalid;
    PlanarRegion resolved;
    state = ResolvePlanarRegion(source, region.triangles.front(), resolved,
                                cancelled, sourcePartition);
    if (state != TopologyResult::Ready)
      return state;
    if (!resolved.IsEqual(region))
      return TopologyResult::Invalid;
    std::vector<Point> outer;
    for (auto v : region.boundaryVertices) {
      if (v >= source.topology.vertices.size())
        return TopologyResult::Invalid;
      outer.push_back(source.topology.vertices[v].point);
    }
    Point x = Sub(outer[1], outer[0]);
    const double xl = std::sqrt(Dot(x, x));
    if (!std::isfinite(xl) || xl <= 0)
      return TopologyResult::Invalid;
    x = Scale(x, 1 / xl);
    Point y = Cross(region.unitNormal, x);
    const double yl = std::sqrt(Dot(y, y));
    if (!std::isfinite(yl) || yl <= 0)
      return TopologyResult::Invalid;
    y = Scale(y, 1 / yl);
    std::vector<std::array<double, 2>> p2(n), inner2(n);
    for (std::size_t i = 0; i < n; ++i) {
      const auto d = Sub(outer[i], outer[0]);
      p2[i] = {Dot(d, x), Dot(d, y)};
    }
    double area = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const auto &a = p2[i], &b = p2[(i + 1) % n];
      area += a[0] * b[1] - a[1] * b[0];
    }
    if (!std::isfinite(area) || area <= 1.e-12)
      return TopologyResult::Invalid;
    for (std::size_t i = 0; i < n; ++i) {
      const auto &a = p2[(i + n - 1) % n], &b = p2[i], &c = p2[(i + 1) % n];
      const double e1x = b[0] - a[0], e1y = b[1] - a[1], e2x = c[0] - b[0],
                   e2y = c[1] - b[1], cross = e1x * e2y - e1y * e2x;
      if (!std::isfinite(cross) ||
          cross <= std::max(1.0, std::abs(area)) * 1.e-12)
        return TopologyResult::Invalid;
      const double l1 = std::hypot(e1x, e1y), l2 = std::hypot(e2x, e2y);
      if (l1 <= 0 || l2 <= 0)
        return TopologyResult::Invalid;
      const std::array<double, 2> q1 = {b[0] + localDistance * (-e1y / l1),
                                        b[1] + localDistance * (e1x / l1)},
                                  q2 = {b[0] + localDistance * (-e2y / l2),
                                        b[1] + localDistance * (e2x / l2)};
      const double det = e1x * e2y - e1y * e2x,
                   t = ((q2[0] - q1[0]) * e2y - (q2[1] - q1[1]) * e2x) / det;
      inner2[i] = {q1[0] + t * e1x, q1[1] + t * e1y};
      if (!std::isfinite(inner2[i][0]) || !std::isfinite(inner2[i][1]))
        return TopologyResult::Invalid;
    }
    double innerArea = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const auto &a = inner2[i];
      const auto &b = inner2[(i + 1) % n];
      const auto &c = inner2[(i + 2) % n];
      innerArea += a[0] * b[1] - a[1] * b[0];
      const double turn =
          (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
      const double miter = std::hypot(a[0] - p2[i][0], a[1] - p2[i][1]);
      if (!std::isfinite(turn) ||
          turn <= std::max(1.0, std::abs(area)) * 1.e-12 ||
          !std::isfinite(miter) || miter > localDistance * 16)
        return TopologyResult::Invalid;
      for (std::size_t edge = 0; edge < n; ++edge) {
        const auto &u = p2[edge];
        const auto &v = p2[(edge + 1) % n];
        const double ex = v[0] - u[0], ey = v[1] - u[1],
                     length = std::hypot(ex, ey);
        const double inward = ex * (a[1] - u[1]) - ey * (a[0] - u[0]);
        if (!std::isfinite(inward) ||
            inward <
                localDistance * length - std::max(1.0, std::abs(area)) * 1.e-10)
          return TopologyResult::Invalid;
      }
    }
    if (!std::isfinite(innerArea) ||
        innerArea <= std::max(1.0, std::abs(area)) * 1.e-12)
      return TopologyResult::Invalid;
    std::vector<Point> inner;
    for (const auto &p : inner2)
      inner.push_back(Add(outer[0], Add(Scale(x, p[0]), Scale(y, p[1]))));
    AffineUV affine;
    if (!MakeAffineUV(source, region, prefix, affine))
      return TopologyResult::Invalid;
    const Normal flat = {float(region.unitNormal[0]),
                         float(region.unitNormal[1]),
                         float(region.unitNormal[2])};
    std::vector<bool> selected(source.storedTriangles.size(), false);
    for (auto t : region.triangles) {
      if (t >= selected.size() || selected[t])
        return TopologyResult::Invalid;
      selected[t] = true;
      for (int k = 0; k < 3; ++k) {
        const int id = source.triangleNodeIDs[t][k];
        if (id <= prefix || id > int(source.storedNormals.size()))
          return TopologyResult::Invalid;
        const auto &m = source.storedNormals[id - 1];
        const double dot = m[0] * region.unitNormal[0] +
                           m[1] * region.unitNormal[1] +
                           m[2] * region.unitNormal[2],
                     length =
                         std::sqrt(double(m[0]) * m[0] + double(m[1]) * m[1] +
                                   double(m[2]) * m[2]);
        if (!std::isfinite(dot) || !std::isfinite(length) || length <= 0 ||
            std::abs(dot / length - 1) > 1.e-5)
          return TopologyResult::Invalid;
      }
    }
    std::vector<Triangle> triangles;
    std::vector<std::array<UV, 3>> uvs;
    std::vector<std::array<Normal, 3>> normals;
    for (std::size_t t = 0; t < source.storedTriangles.size(); ++t)
      if (!selected[t]) {
        triangles.push_back(source.storedTriangles[t]);
        std::array<UV, 3> u;
        std::array<Normal, 3> m;
        for (int k = 0; k < 3; ++k) {
          const int id = source.triangleNodeIDs[t][k];
          if (id <= prefix || id > int(source.storedUVs.size()) ||
              id > int(source.storedNormals.size()))
            return TopologyResult::Invalid;
          u[k] = source.storedUVs[id - 1];
          m[k] = source.storedNormals[id - 1];
        }
        uvs.push_back(u);
        normals.push_back(m);
      }
    const std::uint32_t centerSeed = std::uint32_t(triangles.size());
    auto append = [&](const Triangle &t) {
      triangles.push_back(t);
      uvs.push_back({Evaluate(affine, t[0]), Evaluate(affine, t[1]),
                     Evaluate(affine, t[2])});
      normals.push_back({flat, flat, flat});
    };
    for (std::size_t i = 1; i + 1 < n; ++i)
      append({inner[0], inner[i], inner[i + 1]});
    for (std::size_t i = 0; i < n; ++i) {
      const auto j = (i + 1) % n;
      append({outer[i], outer[j], inner[j]});
      append({outer[i], inner[j], inner[i]});
    }
    if (triangles.size() > 4096)
      return TopologyResult::TooLarge;
    Topology topology;
    state = Analyze(triangles, topology, cancelled);
    if (state != TopologyResult::Ready || topology.boundaryEdges != 0)
      return state == TopologyResult::Ready ? TopologyResult::Invalid : state;
    meshcheck::ContactReport contacts;
    const std::vector<meshcheck::Triangle> contactTriangles(triangles.begin(),
                                                            triangles.end());
    const auto contact = meshcheck::AnalyzeTriangleContacts(
        contactTriangles, contacts, cancelled,
        {4096, 2000000, 1, std::chrono::milliseconds(100)});
    if (contact != meshcheck::ContactStatus::Ready ||
        !contacts.unexpectedPairs.empty())
      return contact == meshcheck::ContactStatus::Cancelled
                 ? TopologyResult::Cancelled
                 : TopologyResult::Invalid;
    TopoDS_Shape shape;
    state = Build(source, triangles, uvs, normals, shape, cancelled);
    if (state != TopologyResult::Ready)
      return state;
    NativeTopologyCapture verified;
    state = CaptureNativeTopology(shape, verified, cancelled);
    if (state != TopologyResult::Ready)
      return state;
    if (verified.storedTriangles != triangles ||
        verified.storedUVs.size() != 3 * triangles.size() ||
        verified.storedNormals.size() != 3 * triangles.size() ||
        verified.triangleNodeIDs.size() != triangles.size() ||
        !HasFlatCornerLayout(verified, 0) ||
        !verified.meshLocation.IsEqual(source.meshLocation) ||
        verified.face.Orientation() != source.face.Orientation())
      return TopologyResult::Invalid;
    for (std::size_t t = 0; t < triangles.size(); ++t)
      for (int corner = 0; corner < 3; ++corner) {
        const std::size_t node = 3 * t + corner;
        if (verified.storedUVs[node] != uvs[t][corner] ||
            verified.storedNormals[node] != normals[t][corner] ||
            verified.triangleNodeIDs[t][corner] != int(node) + 1)
          return TopologyResult::Invalid;
      }
    std::set<std::array<Point, 2>> barriers;
    if (sourcePartition)
      for (const auto &edge : source.topology.edges)
        if (BarrierBetween(source, sourcePartition, edge)) {
          std::array<Point, 2> e = {
              source.topology.vertices[edge.vertices[0]].point,
              source.topology.vertices[edge.vertices[1]].point};
          if (e[1] < e[0])
            std::swap(e[0], e[1]);
          barriers.insert(e);
        }
    for (std::size_t i = 0; i < n; ++i) {
      std::array<Point, 2> e = {inner[i], inner[(i + 1) % n]};
      if (e[1] < e[0])
        std::swap(e[0], e[1]);
      barriers.insert(e);
    }
    RegionPartition partition;
    if (!MakeRegionPartition(verified, barriers, partition))
      return TopologyResult::Invalid;
    RegionInsetCandidate result;
    result.shape = shape;
    result.partition = std::move(partition);
    result.centerSeed = centerSeed;
    result.innerBoundary = std::move(inner);
    output = std::move(result);
    return TopologyResult::Ready;
  } catch (const std::bad_alloc &) {
    output = {};
    return TopologyResult::TooLarge;
  } catch (...) {
    output = {};
    return TopologyResult::Invalid;
  }
}
} // namespace core3d::meshedit
