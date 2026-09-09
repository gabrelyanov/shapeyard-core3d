#pragma once
// Bounded private analysis for native mesh-element selection.
// Never an OCAF replacement, selected-object authority, or a geometric weld.
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <utility>
#include <vector>

namespace core3d::meshedit {
using Point = std::array<double,3>;
using Triangle = std::array<Point,3>;
enum class TopologyResult { Ready, Cancelled, TooLarge, Invalid, NonManifold };
struct EdgeUse { std::uint32_t triangle=0; bool forward=false; };
struct Edge {
    std::array<std::uint32_t,2> vertices{};
    std::vector<EdgeUse> uses;
};
struct Vertex {
    Point point{};
    // Each item is an immutable input triangle/corner ordinal. Native node
    // indices and UV-seam ownership remain separate in the caller's capture.
    std::vector<std::pair<std::uint32_t,std::uint8_t>> corners;
};
struct Topology {
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;
    std::vector<std::array<std::uint32_t,3>> triangleVertices;
    std::vector<Point> unitNormals;
    std::size_t boundaryEdges=0;
};
// Exact shared positions establish geometric adjacency only in this admitted
// oriented-manifold subset. Coincident disconnected vertex fans reject. No
// epsilon welding, index mutation, material merging or UV seam removal occurs.
inline TopologyResult Analyze(const std::vector<Triangle>& input, Topology& output,
                              const std::atomic_bool& cancelled) noexcept {
    output={};
    if (cancelled.load(std::memory_order_relaxed)) return TopologyResult::Cancelled;
    if (input.empty()) return TopologyResult::Invalid;
    if (input.size()>4096) return TopologyResult::TooLarge;
    try {
        Topology result;
        result.triangleVertices.reserve(input.size());result.unitNormals.reserve(input.size());
        std::map<Point,std::uint32_t> vertexIDs;
        std::map<std::array<std::uint32_t,2>,std::size_t> edgeIDs;
        std::set<std::array<std::uint32_t,3>> triangles;
        for (std::size_t t=0;t<input.size();++t) {
            if (cancelled.load(std::memory_order_relaxed)) return TopologyResult::Cancelled;
            const auto& points=input[t];std::array<std::uint32_t,3> ids{};
            for (std::uint8_t k=0;k<3;++k) {
                for (double coordinate:points[k]) if (!std::isfinite(coordinate) || std::abs(coordinate)>1.e6) return TopologyResult::Invalid;
                auto found=vertexIDs.find(points[k]);
                if (found==vertexIDs.end()) {
                    const auto id=static_cast<std::uint32_t>(result.vertices.size());
                    found=vertexIDs.emplace(points[k],id).first;result.vertices.push_back({points[k],{}});
                }
                ids[k]=found->second;
                result.vertices[ids[k]].corners.push_back({static_cast<std::uint32_t>(t),k});
            }
            if (ids[0]==ids[1] || ids[0]==ids[2] || ids[1]==ids[2]) return TopologyResult::Invalid;
            auto ordered=ids;std::sort(ordered.begin(),ordered.end());
            if (!triangles.insert(ordered).second) return TopologyResult::NonManifold;
            Point a{},b{},n{};double a2=0,b2=0,n2=0;
            for (int k=0;k<3;++k) {a[k]=points[1][k]-points[0][k];b[k]=points[2][k]-points[0][k];a2+=a[k]*a[k];b2+=b[k]*b[k];}
            n={a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
            for (double x:n)n2+=x*x;
            if (!std::isfinite(n2) || a2<=0 || b2<=0 || n2<=a2*b2*1.e-24) return TopologyResult::Invalid;
            for (double& x:n)x/=std::sqrt(n2);
            result.unitNormals.push_back(n);result.triangleVertices.push_back(ids);
            for (int k=0;k<3;++k) {
                const auto first=ids[k],second=ids[(k+1)%3];
                const std::array<std::uint32_t,2> key={std::min(first,second),std::max(first,second)};
                auto found=edgeIDs.find(key);
                if (found==edgeIDs.end()) {found=edgeIDs.emplace(key,result.edges.size()).first;result.edges.push_back({key,{}});}
                auto& uses=result.edges[found->second].uses;
                if (uses.size()==2 || (!uses.empty() && uses[0].forward==(first<second))) return TopologyResult::NonManifold;
                uses.push_back({static_cast<std::uint32_t>(t),first<second});
            }
        }
        // Link triangles locally around each vertex using incident edges. A
        // disconnected bow-tie fan or >2 boundary rays cannot be selected as a
        // single editable vertex without ambiguous changes to another shell.
        std::vector<std::vector<std::size_t>> incident(result.vertices.size());
        for (std::size_t e=0;e<result.edges.size();++e) {
            const auto& edge=result.edges[e];
            for (auto v:edge.vertices)incident[v].push_back(e);
            if (edge.uses.size()==1)++result.boundaryEdges;
        }
        for (std::size_t v=0;v<result.vertices.size();++v) {
            if (cancelled.load(std::memory_order_relaxed)) return TopologyResult::Cancelled;
            std::map<std::uint32_t,std::vector<std::uint32_t>> neighbors;
            for (const auto& corner:result.vertices[v].corners)neighbors[corner.first];
            std::size_t boundary=0;
            for (auto e:incident[v]) {
                const auto& uses=result.edges[e].uses;
                if (uses.size()==1) {++boundary;continue;}
                neighbors[uses[0].triangle].push_back(uses[1].triangle);
                neighbors[uses[1].triangle].push_back(uses[0].triangle);
            }
            if (boundary!=0 && boundary!=2) return TopologyResult::NonManifold;
            std::set<std::uint32_t> visited;std::vector<std::uint32_t> queue={neighbors.begin()->first};
            visited.insert(queue[0]);
            for (std::size_t q=0;q<queue.size();++q)for (auto next:neighbors.at(queue[q]))if (visited.insert(next).second)queue.push_back(next);
            if (visited.size()!=neighbors.size()) return TopologyResult::NonManifold;
        }
        if (cancelled.load(std::memory_order_relaxed)) return TopologyResult::Cancelled;
        output=std::move(result);return TopologyResult::Ready;
    } catch (...) {output={};return TopologyResult::Invalid;}
}
} // namespace core3d::meshedit
