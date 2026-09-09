#pragma once
// Resolve only against the exact topology
// retained by a native edit session. These ordinals are NOT persistent IDs.
// No geometry, material, UV or OCAF mutation occurs here.
#include "EditableMeshTopology.hpp"

namespace core3d::meshedit {
enum class ElementKind : std::uint8_t { Vertex, Edge, Triangle };
enum class ElementSelectionResult { Ready, Cancelled, Invalid, TooLarge };

// An edge selects its two geometric vertices; a triangle selects its three.
// Adjacent elements share vertices, so expand and deduplicate BEFORE applying
// the existing 64-vertex move budget. A triangle is not a merged polygon face.
inline ElementSelectionResult ResolveElementVertices(
    const Topology& topology, ElementKind kind,
    const std::vector<std::uint32_t>& elementIDs,
    std::vector<std::uint32_t>& vertices,
    const std::atomic_bool& cancelled) noexcept {
    vertices.clear();
    if (cancelled.load(std::memory_order_relaxed))
        return ElementSelectionResult::Cancelled;
    if (elementIDs.empty() || topology.vertices.empty()
        || topology.triangleVertices.empty()) return ElementSelectionResult::Invalid;
    if (elementIDs.size() > 64 || topology.vertices.size() > 12288
        || topology.triangleVertices.size() > 4096 || topology.edges.size() > 12288)
        return ElementSelectionResult::TooLarge;
    try {
        std::set<std::uint32_t> uniqueElements, uniqueVertices;
        auto include = [&](std::uint32_t vertex) {
            if (vertex >= topology.vertices.size()) return false;
            uniqueVertices.insert(vertex);
            return true;
        };
        for (const auto id : elementIDs) {
            if (cancelled.load(std::memory_order_relaxed))
                return ElementSelectionResult::Cancelled;
            if (!uniqueElements.insert(id).second) return ElementSelectionResult::Invalid;
            switch (kind) {
                case ElementKind::Vertex:
                    if (!include(id)) return ElementSelectionResult::Invalid;
                    break;
                case ElementKind::Edge: {
                    if (id >= topology.edges.size()) return ElementSelectionResult::Invalid;
                    const auto& edge = topology.edges[id];
                    if (edge.vertices[0] == edge.vertices[1] || edge.uses.empty()
                        || edge.uses.size() > 2 || !include(edge.vertices[0])
                        || !include(edge.vertices[1])) return ElementSelectionResult::Invalid;
                    break;
                }
                case ElementKind::Triangle: {
                    if (id >= topology.triangleVertices.size()) return ElementSelectionResult::Invalid;
                    const auto& triangle = topology.triangleVertices[id];
                    if (triangle[0] == triangle[1] || triangle[0] == triangle[2]
                        || triangle[1] == triangle[2] || !include(triangle[0])
                        || !include(triangle[1]) || !include(triangle[2]))
                        return ElementSelectionResult::Invalid;
                    break;
                }
                default: return ElementSelectionResult::Invalid;
            }
            if (uniqueVertices.size() > 64) return ElementSelectionResult::TooLarge;
        }
        if (cancelled.load(std::memory_order_relaxed))
            return ElementSelectionResult::Cancelled;
        vertices.assign(uniqueVertices.begin(), uniqueVertices.end());
        return ElementSelectionResult::Ready;
    } catch (...) {
        vertices.clear();
        return ElementSelectionResult::Invalid;
    }
}
} // namespace core3d::meshedit
