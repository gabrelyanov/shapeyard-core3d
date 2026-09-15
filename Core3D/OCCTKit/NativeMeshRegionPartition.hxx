#pragma once
// Private bounded face-partition kernel. Persistence and OCAF mutation remain
// with the document owner; this type grants no edit or selection authority.
#include "../Scene/AuthoredTangentArchive.hpp"
#include "NativeMeshTopologyCapture.hxx"
#include <CommonCrypto/CommonDigest.h>
#include <cstring>

namespace core3d::meshedit {
using RegionGeometryDigest = scene::authored::GeometryIdentity;
inline bool
OrderedTriangleGeometryDigest(const NativeTopologyCapture &source,
                              RegionGeometryDigest &output) noexcept {
  output.fill(0);
  const auto &triangles = source.storedTriangles;
  if (triangles.empty() || triangles.size() > 4096)
    return false;
  try {
    CC_SHA256_CTX context;
    if (CC_SHA256_Init(&context) != 1)
      return false;
    auto feed32 = [&](std::uint32_t value) {
      std::uint8_t b[4];
      scene::authored::Write32(b, value);
      return CC_SHA256_Update(&context, b, 4) == 1;
    };
    auto feed64 = [&](double value) {
      if (!std::isfinite(value) || std::abs(value) > 1.e6)
        return false;
      std::uint64_t bits;
      std::memcpy(&bits, &value, 8);
      std::uint8_t b[8];
      for (unsigned i = 0; i < 8; ++i)
        b[i] = std::uint8_t(bits >> (8 * i));
      return CC_SHA256_Update(&context, b, 8) == 1;
    };
    if (!feed32(0x50525953) || !feed32(1) ||
        !feed32(std::uint32_t(triangles.size())))
      return false; // SYRP v1
    for (std::size_t t = 0; t < triangles.size(); ++t) {
      if (!feed32(std::uint32_t(t)))
        return false;
      for (std::uint32_t c = 0; c < 3; ++c) {
        if (!feed32(c))
          return false;
        const auto &corner = triangles[t][c];
        const gp_Pnt placed =
            gp_Pnt(corner[0], corner[1], corner[2])
                .Transformed(source.meshLocation.Transformation());
        for (const double coordinate : {placed.X(), placed.Y(), placed.Z()})
          if (!feed64(coordinate))
            return false;
      }
    }
    RegionGeometryDigest digest;
    if (CC_SHA256_Final(digest.data(), &context) != 1)
      return false;
    output = digest;
    return true;
  } catch (...) {
    output.fill(0);
    return false;
  }
}
struct RegionPartition {
  RegionGeometryDigest geometry{};
  std::vector<std::uint8_t> barriers; // low three bits are triangle-local edges
  bool empty() const noexcept { return barriers.empty(); }
  bool operator==(const RegionPartition &x) const noexcept {
    return geometry == x.geometry && barriers == x.barriers;
  }
};
inline int LocalEdge(const Topology &topology, std::uint32_t triangle,
                     const std::array<std::uint32_t, 2> &edge) noexcept {
  if (triangle >= topology.triangleVertices.size())
    return -1;
  const auto &ids = topology.triangleVertices[triangle];
  for (int k = 0; k < 3; ++k)
    if (std::min(ids[k], ids[(k + 1) % 3]) == edge[0] &&
        std::max(ids[k], ids[(k + 1) % 3]) == edge[1])
      return k;
  return -1;
}
inline bool HasBarrier(const RegionPartition &partition, std::uint32_t triangle,
                       int edge) noexcept {
  return triangle < partition.barriers.size() && edge >= 0 && edge < 3 &&
         (partition.barriers[triangle] & (1U << edge));
}
inline bool ValidateRegionPartition(const NativeTopologyCapture &source,
                                    const RegionPartition &partition) noexcept {
  if (partition.barriers.size() != source.storedTriangles.size() ||
      partition.barriers.empty())
    return false;
  RegionGeometryDigest digest;
  if (!OrderedTriangleGeometryDigest(source, digest) ||
      digest != partition.geometry)
    return false;
  bool any = false;
  for (auto mask : partition.barriers) {
    if (mask & ~std::uint8_t(7))
      return false;
    any |= mask != 0;
  }
  if (!any)
    return false; // canonical no-barrier state is absence
  for (const auto &edge : source.topology.edges) {
    if (edge.uses.empty() || edge.uses.size() > 2)
      return false;
    std::array<bool, 2> marked{};
    for (std::size_t i = 0; i < edge.uses.size(); ++i) {
      const int local =
          LocalEdge(source.topology, edge.uses[i].triangle, edge.vertices);
      if (local < 0)
        return false;
      marked[i] = HasBarrier(partition, edge.uses[i].triangle, local);
    }
    if (edge.uses.size() == 1) {
      if (marked[0])
        return false;
    } else if (marked[0] != marked[1])
      return false;
  }
  return true;
}
inline bool BarrierBetween(const NativeTopologyCapture &source,
                           const RegionPartition *partition,
                           const Edge &edge) noexcept {
  if (!partition || edge.uses.size() != 2)
    return false;
  const int local =
      LocalEdge(source.topology, edge.uses[0].triangle, edge.vertices);
  return HasBarrier(*partition, edge.uses[0].triangle, local);
}
inline bool MakeRegionPartition(const NativeTopologyCapture &source,
                                const std::set<std::array<Point, 2>> &requested,
                                RegionPartition &output) noexcept {
  output = {};
  if (requested.empty())
    return true;
  try {
    RegionPartition value;
    value.barriers.assign(source.storedTriangles.size(), 0);
    std::set<std::array<Point, 2>> found;
    for (const auto &edge : source.topology.edges) {
      if (edge.uses.size() != 2)
        return false;
      std::array<Point, 2> points = {
          source.topology.vertices[edge.vertices[0]].point,
          source.topology.vertices[edge.vertices[1]].point};
      if (points[1] < points[0])
        std::swap(points[0], points[1]);
      if (!requested.count(points))
        continue;
      found.insert(points);
      for (const auto &use : edge.uses) {
        const int local =
            LocalEdge(source.topology, use.triangle, edge.vertices);
        if (local < 0)
          return false;
        value.barriers[use.triangle] |= std::uint8_t(1U << local);
      }
    }
    if (found != requested ||
        !OrderedTriangleGeometryDigest(source, value.geometry) ||
        !ValidateRegionPartition(source, value))
      return false;
    output = std::move(value);
    return true;
  } catch (...) {
    output = {};
    return false;
  }
}
inline void Write32(std::vector<std::uint8_t> &out, std::uint32_t value) {
  for (unsigned i = 0; i < 4; ++i)
    out.push_back(std::uint8_t(value >> (8 * i)));
}
inline bool Read32(const std::uint8_t *p, std::uint32_t &v) {
  v = std::uint32_t(p[0]) | (std::uint32_t(p[1]) << 8) |
      (std::uint32_t(p[2]) << 16) | (std::uint32_t(p[3]) << 24);
  return true;
}
inline bool EncodeRegionPartition(const RegionPartition &value,
                                  std::vector<std::uint8_t> &output) noexcept {
  output.clear();
  try {
    if (value.barriers.empty() || value.barriers.size() > 4096)
      return false;
    const std::size_t packed = (3 * value.barriers.size() + 7) / 8;
    output.reserve(44 + packed);
    Write32(output, 0x50525953);
    Write32(output, 1);
    Write32(output, std::uint32_t(value.barriers.size()));
    output.insert(output.end(), value.geometry.begin(), value.geometry.end());
    output.resize(44 + packed, 0);
    std::size_t bit = 0;
    bool any = false;
    for (auto mask : value.barriers) {
      if (mask & ~std::uint8_t(7))
        return output.clear(), false;
      any |= mask != 0;
      for (int e = 0; e < 3; ++e, ++bit)
        if (mask & (1U << e))
          output[44 + bit / 8] |= std::uint8_t(1U << (bit % 8));
    }
    if (!any)
      return output.clear(), false;
    return output.size() <= 1580;
  } catch (...) {
    output.clear();
    return false;
  }
}
inline bool DecodeRegionPartition(const std::uint8_t *bytes, std::size_t size,
                                  RegionPartition &output) noexcept {
  output = {};
  if (!bytes || size < 45 || size > 1580)
    return false;
  try {
    std::uint32_t magic = 0, version = 0, count = 0;
    Read32(bytes, magic);
    Read32(bytes + 4, version);
    Read32(bytes + 8, count);
    if (magic != 0x50525953 || version != 1 || count == 0 || count > 4096 ||
        size != 44 + (3 * std::size_t(count) + 7) / 8)
      return false;
    const unsigned used = (3 * count) % 8;
    if (used && ((bytes[size - 1] >> used) != 0))
      return false;
    RegionPartition value;
    std::copy(bytes + 12, bytes + 44, value.geometry.begin());
    value.barriers.assign(count, 0);
    std::size_t bit = 0;
    for (std::size_t t = 0; t < count; ++t)
      for (int e = 0; e < 3; ++e, ++bit)
        if (bytes[44 + bit / 8] & (1U << (bit % 8)))
          value.barriers[t] |= std::uint8_t(1U << e);
    bool any = false;
    for (auto mask : value.barriers)
      any |= mask != 0;
    if (!any)
      return false;
    output = std::move(value);
    return true;
  } catch (...) {
    output = {};
    return false;
  }
}
} // namespace core3d::meshedit
