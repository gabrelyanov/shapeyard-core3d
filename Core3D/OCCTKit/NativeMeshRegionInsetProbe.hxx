#pragma once
#if DEBUG
#include "NativeMeshRegionInset.hxx"
#include <gp_Trsf.hxx>
namespace core3d::debug {
struct MeshRegionInsetProbeResult {
  std::array<bool, 20> checks{};
  double beforeVolume = 0, insetVolume = 0, extrudedVolume = 0;
  std::uint32_t insetTriangles = 0, centerTriangles = 0;
};
inline double SignedVolume(const std::vector<meshedit::Triangle> &triangles) {
  double v = 0;
  for (const auto &t : triangles)
    v += (t[0][0] * (t[1][1] * t[2][2] - t[1][2] * t[2][1]) +
          t[0][1] * (t[1][2] * t[2][0] - t[1][0] * t[2][2]) +
          t[0][2] * (t[1][0] * t[2][1] - t[1][1] * t[2][0])) /
         6;
  return std::abs(v);
}
inline TopoDS_Shape MakePrism(const std::vector<std::array<double, 2>> &yz,
                              double depth, bool transformed = false) {
  using namespace meshedit;
  std::vector<Triangle> triangles;
  const std::size_t n = yz.size();
  auto p = [&](double x, std::size_t i) {
    return Point{x, yz[i][0], yz[i][1]};
  };
  for (std::size_t i = 1; i + 1 < n; ++i)
    triangles.push_back({p(0, 0), p(0, i + 1), p(0, i)});
  for (std::size_t i = 1; i + 1 < n; ++i)
    triangles.push_back({p(depth, 0), p(depth, i), p(depth, i + 1)});
  for (std::size_t i = 0; i < n; ++i) {
    const auto j = (i + 1) % n;
    triangles.push_back({p(0, i), p(0, j), p(depth, j)});
    triangles.push_back({p(0, i), p(depth, j), p(depth, i)});
  }
  Handle(Poly_Triangulation) mesh = new Poly_Triangulation(
      int(3 * triangles.size()), int(triangles.size()), true, true);
  for (std::size_t t = 0; t < triangles.size(); ++t) {
    const auto a = inset_detail::Sub(triangles[t][1], triangles[t][0]),
               b = inset_detail::Sub(triangles[t][2], triangles[t][0]),
               cross = inset_detail::Cross(a, b);
    const double length = std::sqrt(inset_detail::Dot(cross, cross));
    const gp_Vec3f normal(float(cross[0] / length), float(cross[1] / length),
                          float(cross[2] / length));
    for (int k = 0; k < 3; ++k) {
      const int id = int(3 * t) + k + 1;
      const auto &q = triangles[t][k];
      mesh->SetNode(id, gp_Pnt(q[0], q[1], q[2]));
      mesh->SetUVNode(id, gp_Pnt2d(q[1] / 100, q[2] / 100));
      mesh->SetNormal(id, normal);
    }
    mesh->SetTriangle(int(t) + 1, Poly_Triangle(int(3 * t) + 1, int(3 * t) + 2,
                                                int(3 * t) + 3));
  }
  mesh->UpdateCachedMinMax();
  TopoDS_Face face;
  BRep_Builder().MakeFace(face, mesh);
  if (transformed) {
    constexpr double scale = 1.5, angle = 0.37;
    gp_Trsf transform;
    transform.SetValues(scale * std::cos(angle), -scale * std::sin(angle), 0,
                        17, scale * std::sin(angle), scale * std::cos(angle), 0,
                        -11, 0, 0, scale, 23);
    // TopoDS permits a positive non-unit similarity location only through its
    // explicit non-raising path; production import uses the same API contract.
    face.Location(TopLoc_Location(transform), Standard_False);
  }
  return face;
}
inline std::set<std::array<meshedit::Point, 2>>
BarrierEndpointSet(const meshedit::NativeTopologyCapture &source,
                   const meshedit::RegionPartition &partition) {
  using namespace meshedit;
  std::set<std::array<Point, 2>> result;
  for (const auto &edge : source.topology.edges)
    if (BarrierBetween(source, &partition, edge)) {
      std::array<Point, 2> points = {
          source.topology.vertices[edge.vertices[0]].point,
          source.topology.vertices[edge.vertices[1]].point};
      if (points[1] < points[0])
        std::swap(points[0], points[1]);
      result.insert(points);
    }
  return result;
}
inline std::set<std::array<meshedit::Point, 2>>
LoopEndpointSet(const std::vector<meshedit::Point> &loop) {
  using namespace meshedit;
  std::set<std::array<Point, 2>> result;
  for (std::size_t i = 0; i < loop.size(); ++i) {
    std::array<Point, 2> points = {loop[i], loop[(i + 1) % loop.size()]};
    if (points[1] < points[0])
      std::swap(points[0], points[1]);
    result.insert(points);
  }
  return result;
}
inline MeshRegionInsetProbeResult RunMeshRegionInsetKernelProbe() {
  using namespace meshedit;
  MeshRegionInsetProbeResult result;
  std::atomic_bool cancelled{false};
  try {
    const std::vector<std::array<double, 2>> rectangle = {
        {0, 0}, {80, 0}, {80, 60}, {0, 60}};
    const auto shape = MakePrism(rectangle, 100);
    NativeTopologyCapture source;
    if (CaptureNativeTopology(shape, source, cancelled) !=
        TopologyResult::Ready)
      return result;
    result.beforeVolume = SignedVolume(source.storedTriangles);
    PlanarRegion face;
    const std::uint32_t seed = std::uint32_t(rectangle.size() - 2);
    result.checks[0] = ResolvePlanarRegion(source, seed, face, cancelled) ==
                           TopologyResult::Ready &&
                       face.triangles.size() == 2 &&
                       face.boundaryVertices.size() == 4;
    RegionInsetCandidate inset;
    result.checks[1] =
        PrepareConvexRegionInset(source, 0, face, 10, nullptr, inset,
                                 cancelled) == TopologyResult::Ready &&
        !inset.shape.IsNull();
    NativeTopologyCapture insetSource;
    if (!result.checks[1] ||
        CaptureNativeTopology(inset.shape, insetSource, cancelled) !=
            TopologyResult::Ready)
      return result;
    result.insetVolume = SignedVolume(insetSource.storedTriangles);
    result.insetTriangles = std::uint32_t(insetSource.storedTriangles.size());
    PlanarRegion center;
    result.checks[2] =
        ValidateRegionPartition(insetSource, inset.partition) &&
        ResolvePlanarRegion(insetSource, inset.centerSeed, center, cancelled,
                            &inset.partition) == TopologyResult::Ready;
    result.centerTriangles = std::uint32_t(center.triangles.size());
    double ymin = INFINITY, ymax = -INFINITY, zmin = INFINITY, zmax = -INFINITY;
    for (const auto &p : inset.innerBoundary) {
      ymin = std::min(ymin, p[1]);
      ymax = std::max(ymax, p[1]);
      zmin = std::min(zmin, p[2]);
      zmax = std::max(zmax, p[2]);
    }
    result.checks[3] = ymin == 10 && ymax == 70 && zmin == 10 && zmax == 50 &&
                       result.beforeVolume == 480000 &&
                       result.insetVolume == 480000 &&
                       center.triangles.size() == 2 &&
                       insetSource.storedUVs.size() ==
                           3 * insetSource.storedTriangles.size() &&
                       insetSource.storedNormals.size() ==
                           3 * insetSource.storedTriangles.size();
    TopoDS_Shape droppedAuthority, extruded;
    RegionPartition afterPartition;
    result.checks[4] =
        PrepareRegionExtrusion(insetSource, 0, center, 6,
                               RegionSideUVPolicy::BoundaryStripNormalized,
                               droppedAuthority, cancelled, &inset.partition,
                               nullptr) == TopologyResult::Invalid &&
        droppedAuthority.IsNull() &&
        PrepareRegionExtrusion(insetSource, 0, center, 6,
                               RegionSideUVPolicy::BoundaryStripNormalized,
                               extruded, cancelled, &inset.partition,
                               &afterPartition) == TopologyResult::Ready;
    NativeTopologyCapture extrusion;
    if (result.checks[4] &&
        CaptureNativeTopology(extruded, extrusion, cancelled) ==
            TopologyResult::Ready) {
      result.extrudedVolume = SignedVolume(extrusion.storedTriangles);
      result.checks[5] =
          result.extrudedVolume == 494400 && afterPartition.empty();
    }
    std::vector<std::uint8_t> encoded;
    RegionPartition decoded;
    result.checks[6] =
        EncodeRegionPartition(inset.partition, encoded) &&
        DecodeRegionPartition(encoded.data(), encoded.size(), decoded) &&
        decoded == inset.partition &&
        ValidateRegionPartition(insetSource, decoded);
    RegionPartition zero = inset.partition;
    std::fill(zero.barriers.begin(), zero.barriers.end(), 0);
    std::vector<std::uint8_t> zeroBytes(45, 0);
    zeroBytes[0] = 0x53;
    zeroBytes[1] = 0x59;
    zeroBytes[2] = 0x52;
    zeroBytes[3] = 0x50;
    zeroBytes[4] = 1;
    zeroBytes[8] = 2;
    RegionPartition uncleared = inset.partition;
    RegionPartition absent;
    std::vector<std::uint8_t> rejected{1};
    result.checks[14] =
        !EncodeRegionPartition(absent, rejected) && rejected.empty() &&
        !EncodeRegionPartition(zero, rejected) && rejected.empty() &&
        !DecodeRegionPartition(zeroBytes.data(), zeroBytes.size(), uncleared) &&
        uncleared.empty();
    RegionPartition golden;
    for (std::size_t i = 0; i < golden.geometry.size(); ++i)
      golden.geometry[i] = std::uint8_t(i);
    golden.barriers = {3, 4};
    std::vector<std::uint8_t> goldenBytes;
    const std::vector<std::uint8_t> expectedGolden = {
        0x53, 0x59, 0x52, 0x50, 1,  0,  0,  0,  2,  0,  0,  0,  0,  1,  2,
        3,    4,    5,    6,    7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17,
        18,   19,   20,   21,   22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 0x23};
    RegionPartition decodedGolden;
    result.checks[15] =
        EncodeRegionPartition(golden, goldenBytes) &&
        goldenBytes == expectedGolden &&
        DecodeRegionPartition(expectedGolden.data(), expectedGolden.size(),
                              decodedGolden) &&
        decodedGolden == golden;
    auto asymmetric = inset.partition;
    bool flipped = false;
    for (auto &mask : asymmetric.barriers)
      if (mask) {
        mask ^= std::uint8_t(mask & -mask);
        flipped = true;
        break;
      }
    result.checks[7] =
        flipped && !ValidateRegionPartition(insetSource, asymmetric);
    auto reordered = insetSource.storedTriangles;
    std::swap(reordered[0], reordered[1]);
    RegionGeometryDigest digest;
    result.checks[8] = ([&] {
      auto changed = insetSource;
      changed.storedTriangles = reordered;
      return OrderedTriangleGeometryDigest(changed, digest) &&
             digest != inset.partition.geometry;
    })();
    RegionInsetCandidate belowCollapse, collapse, beyondCollapse;
    result.checks[9] =
        PrepareConvexRegionInset(source, 0, face, 29.999, nullptr,
                                 belowCollapse,
                                 cancelled) == TopologyResult::Ready &&
        !belowCollapse.shape.IsNull() &&
        PrepareConvexRegionInset(source, 0, face, 30, nullptr, collapse,
                                 cancelled) == TopologyResult::Invalid &&
        collapse.shape.IsNull() &&
        PrepareConvexRegionInset(source, 0, face, 31, nullptr, beyondCollapse,
                                 cancelled) == TopologyResult::Invalid &&
        beyondCollapse.shape.IsNull();
    const auto concaveShape =
        MakePrism({{0, 0}, {80, 0}, {40, 25}, {80, 60}, {0, 60}}, 20);
    NativeTopologyCapture concave;
    PlanarRegion concaveFace;
    RegionInsetCandidate refused;
    result.checks[10] =
        CaptureNativeTopology(concaveShape, concave, cancelled) ==
            TopologyResult::Ready &&
        ResolvePlanarRegion(concave, 3, concaveFace, cancelled) ==
            TopologyResult::Ready &&
        PrepareConvexRegionInset(concave, 0, concaveFace, 2, nullptr, refused,
                                 cancelled) == TopologyResult::Invalid;
    const auto oblique =
        MakePrism({{0, 0}, {55, -8}, {92, 31}, {61, 74}, {7, 58}}, 20, true);
    NativeTopologyCapture obliqueSource;
    PlanarRegion obliqueFace;
    RegionInsetCandidate obliqueInset;
    result.checks[11] =
        CaptureNativeTopology(oblique, obliqueSource, cancelled) ==
            TopologyResult::Ready &&
        ResolvePlanarRegion(obliqueSource, 3, obliqueFace, cancelled) ==
            TopologyResult::Ready &&
        PrepareConvexRegionInset(obliqueSource, 0, obliqueFace, 2, nullptr,
                                 obliqueInset,
                                 cancelled) == TopologyResult::Ready;
    if (result.checks[11]) {
      const auto &placement = obliqueSource.meshLocation.Transformation();
      std::vector<Point> placedOuter, placedInner;
      for (std::size_t i = 0; i < obliqueFace.boundaryVertices.size(); ++i) {
        const auto &raw =
            obliqueSource.topology.vertices[obliqueFace.boundaryVertices[i]]
                .point;
        const gp_Pnt outer =
            gp_Pnt(raw[0], raw[1], raw[2]).Transformed(placement);
        const auto &innerRaw = obliqueInset.innerBoundary[i];
        const gp_Pnt inner = gp_Pnt(innerRaw[0], innerRaw[1], innerRaw[2])
                                 .Transformed(placement);
        placedOuter.push_back({outer.X(), outer.Y(), outer.Z()});
        placedInner.push_back({inner.X(), inner.Y(), inner.Z()});
      }
      gp_Vec placedNormal(obliqueFace.unitNormal[0], obliqueFace.unitNormal[1],
                          obliqueFace.unitNormal[2]);
      placedNormal.Transform(placement);
      placedNormal.Normalize();
      const Point normal = {placedNormal.X(), placedNormal.Y(),
                            placedNormal.Z()};
      bool distances = true;
      for (std::size_t i = 0; i < placedOuter.size(); ++i) {
        const Point edge = inset_detail::Sub(
            placedOuter[(i + 1) % placedOuter.size()], placedOuter[i]);
        const Point delta = inset_detail::Sub(placedInner[i], placedOuter[i]);
        const Point cross = inset_detail::Cross(edge, delta);
        const double signedDistance = inset_detail::Dot(cross, normal) /
                                      std::sqrt(inset_detail::Dot(edge, edge));
        distances &= std::abs(signedDistance - 3) < 1.e-9;
      }
      Point areaVector{};
      for (std::size_t i = 1; i + 1 < placedInner.size(); ++i) {
        const Point a = inset_detail::Sub(placedInner[i], placedInner.front());
        const Point b =
            inset_detail::Sub(placedInner[i + 1], placedInner.front());
        areaVector = inset_detail::Add(areaVector, inset_detail::Cross(a, b));
      }
      const bool transformedCoordinates =
          placedOuter.front() !=
          obliqueSource.topology.vertices[obliqueFace.boundaryVertices.front()]
              .point;
      result.checks[11] = distances && transformedCoordinates &&
                          inset_detail::Dot(areaVector, normal) > 0;
    }
    TopoDS_Shape relocatedShape = inset.shape;
    gp_Trsf moved;
    moved.SetTranslation(gp_Vec(1, 2, 3));
    relocatedShape = relocatedShape.Moved(TopLoc_Location(moved));
    NativeTopologyCapture relocated;
    result.checks[12] =
        CaptureNativeTopology(relocatedShape, relocated, cancelled) ==
            TopologyResult::Ready &&
        !relocated.meshLocation.IsEqual(insetSource.meshLocation) &&
        !ValidateRegionPartition(relocated, inset.partition);
    RegionInsetCandidate repeated;
    PlanarRegion other;
    NativeTopologyCapture repeatedSource;
    const auto originalEndpoints =
        BarrierEndpointSet(insetSource, inset.partition);
    result.checks[13] =
        ResolvePlanarRegion(insetSource, 0, other, cancelled,
                            &inset.partition) == TopologyResult::Ready &&
        PrepareConvexRegionInset(insetSource, 0, other, 1, &inset.partition,
                                 repeated,
                                 cancelled) == TopologyResult::Ready &&
        CaptureNativeTopology(repeated.shape, repeatedSource, cancelled) ==
            TopologyResult::Ready &&
        ValidateRegionPartition(repeatedSource, repeated.partition);
    if (result.checks[13]) {
      const auto repeatedEndpoints =
          BarrierEndpointSet(repeatedSource, repeated.partition);
      const auto newEndpoints = LoopEndpointSet(repeated.innerBoundary);
      auto expectedEndpoints = originalEndpoints;
      expectedEndpoints.insert(newEndpoints.begin(), newEndpoints.end());
      result.checks[13] =
          newEndpoints.size() == repeated.innerBoundary.size() &&
          expectedEndpoints.size() ==
              originalEndpoints.size() + newEndpoints.size() &&
          repeatedEndpoints == expectedEndpoints;
    }
    RegionInsetCandidate nested;
    NativeTopologyCapture nestedSource;
    result.checks[16] =
        PrepareConvexRegionInset(insetSource, 0, center, 2, &inset.partition,
                                 nested, cancelled) == TopologyResult::Ready &&
        CaptureNativeTopology(nested.shape, nestedSource, cancelled) ==
            TopologyResult::Ready &&
        ValidateRegionPartition(nestedSource, nested.partition);
    if (result.checks[16]) {
      const auto nestedEndpoints =
          BarrierEndpointSet(nestedSource, nested.partition);
      const auto newEndpoints = LoopEndpointSet(nested.innerBoundary);
      auto expectedEndpoints = originalEndpoints;
      expectedEndpoints.insert(newEndpoints.begin(), newEndpoints.end());
      result.checks[16] = newEndpoints.size() == nested.innerBoundary.size() &&
                          expectedEndpoints.size() ==
                              originalEndpoints.size() + newEndpoints.size() &&
                          nestedEndpoints == expectedEndpoints;
    }
    NativeTopologyCapture emptyCapture;
    RegionInsetCandidate invalidInset = inset;
    TopoDS_Shape invalidExtrusion = shape;
    RegionPartition invalidPartition = inset.partition;
    result.checks[17] =
        PrepareConvexRegionInset(emptyCapture, 0, face, 10, nullptr,
                                 invalidInset,
                                 cancelled) == TopologyResult::Invalid &&
        invalidInset.shape.IsNull() && invalidInset.partition.empty() &&
        PrepareRegionExtrusion(emptyCapture, 0, face, 6,
                               RegionSideUVPolicy::BoundaryStripNormalized,
                               invalidExtrusion, cancelled, nullptr,
                               &invalidPartition) == TopologyResult::Invalid &&
        invalidExtrusion.IsNull() && invalidPartition.empty();
    auto mismatched = source;
    std::swap(mismatched.topology.triangleVertices[seed][0],
              mismatched.topology.triangleVertices[seed][1]);
    RegionInsetCandidate mismatchOutput = inset;
    TopoDS_Shape mismatchExtrusion = shape;
    RegionPartition mismatchPartition = inset.partition;
    result.checks[18] =
        PrepareConvexRegionInset(mismatched, 0, face, 10, nullptr,
                                 mismatchOutput,
                                 cancelled) == TopologyResult::Invalid &&
        mismatchOutput.shape.IsNull() && mismatchOutput.partition.empty() &&
        PrepareRegionExtrusion(mismatched, 0, face, 6,
                               RegionSideUVPolicy::BoundaryStripNormalized,
                               mismatchExtrusion, cancelled, nullptr,
                               &mismatchPartition) == TopologyResult::Invalid &&
        mismatchExtrusion.IsNull() && mismatchPartition.empty();
    std::atomic_bool stopped{true};
    RegionInsetCandidate stoppedInset = inset;
    TopoDS_Shape stoppedExtrusion = shape;
    RegionPartition stoppedPartition = inset.partition;
    result.checks[19] =
        PrepareConvexRegionInset(source, 0, face, 10, nullptr, stoppedInset,
                                 stopped) == TopologyResult::Cancelled &&
        stoppedInset.shape.IsNull() && stoppedInset.partition.empty() &&
        PrepareRegionExtrusion(
            source, 0, face, 6, RegionSideUVPolicy::BoundaryStripNormalized,
            stoppedExtrusion, stopped, nullptr,
            &stoppedPartition) == TopologyResult::Cancelled &&
        stoppedExtrusion.IsNull() && stoppedPartition.empty();
  } catch (...) {
  }
  return result;
}
} // namespace core3d::debug
#endif
