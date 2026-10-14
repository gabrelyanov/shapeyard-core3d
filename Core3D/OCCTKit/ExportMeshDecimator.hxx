#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace core3d::export_mesh_decimator {

constexpr std::size_t kMaximumTriangles = 4096;
constexpr std::size_t kMaximumPositions = 12288;
constexpr std::uint64_t kMaximumCandidateEvaluations = 1u << 24;
constexpr std::uint64_t kMaximumWorkUnits = 1u << 27;

struct Point {
    double x = 0, y = 0, z = 0;
};

inline bool operator==(const Point& a, const Point& b) noexcept {
    return a.x == b.x && a.y == b.y && a.z == b.z;
}
inline bool operator<(const Point& a, const Point& b) noexcept {
    return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
}
inline Point operator+(const Point& a, const Point& b) noexcept {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline Point operator-(const Point& a, const Point& b) noexcept {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline Point operator*(const Point& a, double b) noexcept {
    return {a.x * b, a.y * b, a.z * b};
}
inline double Dot(const Point& a, const Point& b) noexcept {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Point Cross(const Point& a, const Point& b) noexcept {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}
inline bool Finite(const Point& p) noexcept {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

enum class Error {
    None,
    TargetBelowClosedMeshMinimum,
    InvalidTopology,
    NonFiniteInput,
    ResourceLimit,
    Cancelled,
    UnreachableTriangleBudget,
};

struct Input {
    std::vector<Point> positionsMM;
    std::vector<std::uint32_t> indices;
    std::vector<std::string> contributionTokens;
    std::size_t targetTriangles = 0;
};

struct Output {
    // Every corner is split so f2-normal-regen.v1 is unambiguous. Indices are
    // therefore the deterministic sequence 0...3F-1.
    std::vector<float> positionsMM;
    std::vector<float> flatNormals;
    std::vector<std::uint32_t> indices;
    std::vector<std::vector<std::string>> contributionTokens;
    std::size_t acceptedCollapses = 0;
};

namespace detail {

struct Quadric {
    // xx,xy,xz,xw,yy,yz,yw,zz,zw,ww
    std::array<double, 10> v{};
    Quadric& operator+=(const Quadric& other) noexcept {
        for (std::size_t i = 0; i < v.size(); ++i) v[i] += other.v[i];
        return *this;
    }
};
inline Quadric operator+(Quadric a, const Quadric& b) noexcept { return a += b; }

inline double Cost(const Quadric& q, const Point& p) noexcept {
    return q.v[0]*p.x*p.x + 2*q.v[1]*p.x*p.y + 2*q.v[2]*p.x*p.z
        + 2*q.v[3]*p.x + q.v[4]*p.y*p.y + 2*q.v[5]*p.y*p.z
        + 2*q.v[6]*p.y + q.v[7]*p.z*p.z + 2*q.v[8]*p.z + q.v[9];
}

struct Vertex {
    Point point;
    Quadric quadric;
    std::size_t rank = 0;
    bool alive = true;
    bool locked = false;
};
struct Face {
    std::array<std::size_t, 3> vertex{};
    std::vector<std::string> tokens;
    bool alive = true;
};
using Edge = std::pair<std::size_t, std::size_t>;
inline Edge MakeEdge(std::size_t a, std::size_t b) noexcept {
    return {std::min(a, b), std::max(a, b)};
}

inline Point Normal(const Face& f, const std::vector<Vertex>& vertices) noexcept {
    const Point& a = vertices[f.vertex[0]].point;
    const Point& b = vertices[f.vertex[1]].point;
    const Point& c = vertices[f.vertex[2]].point;
    return Cross(b - a, c - a);
}

inline bool Charge(std::uint64_t& work, const std::atomic_bool* stop,
                   std::uint64_t amount = 1) noexcept {
    if (amount > kMaximumWorkUnits - work) return false;
    work += amount;
    return !(stop && stop->load(std::memory_order_relaxed));
}

inline std::size_t FaceCount(const std::vector<Face>& faces) noexcept {
    return static_cast<std::size_t>(std::count_if(faces.begin(), faces.end(),
        [](const Face& f) { return f.alive; }));
}

// D819(b): the welded source's own Euler characteristic is the reference.
// Initial validation accepts any closed 2-manifold genus, and every accepted
// collapse must preserve that source characteristic.
inline int EulerCharacteristic(const std::vector<Face>& faces) {
    std::set<std::array<std::size_t, 3>> uniqueFaces;
    std::set<Edge> edges;
    std::set<std::size_t> used;
    for (const Face& face : faces) {
        if (!face.alive) continue;
        auto sorted = face.vertex;
        std::sort(sorted.begin(), sorted.end());
        uniqueFaces.insert(sorted);
        used.insert(face.vertex.begin(), face.vertex.end());
        edges.insert(MakeEdge(face.vertex[0], face.vertex[1]));
        edges.insert(MakeEdge(face.vertex[1], face.vertex[2]));
        edges.insert(MakeEdge(face.vertex[2], face.vertex[0]));
    }
    return static_cast<int>(used.size()) - static_cast<int>(edges.size())
        + static_cast<int>(uniqueFaces.size());
}

inline bool Validate(const std::vector<Vertex>& vertices,
                     const std::vector<Face>& faces, std::uint64_t& work,
                     const std::atomic_bool* stop, int expectedEuler) {
    std::map<Edge, std::pair<unsigned, int>> edges;
    std::set<std::array<std::size_t, 3>> uniqueFaces;
    std::set<std::size_t> used;
    double volumeSix = 0;
    for (const Face& face : faces) {
        if (!face.alive) continue;
        if (!Charge(work, stop)) return false;
        auto v = face.vertex;
        if (v[0] == v[1] || v[1] == v[2] || v[2] == v[0]
            || !vertices[v[0]].alive || !vertices[v[1]].alive || !vertices[v[2]].alive)
            return false;
        const Point n = Normal(face, vertices);
        if (!Finite(n) || Dot(n, n) == 0) return false;
        auto sorted = v; std::sort(sorted.begin(), sorted.end());
        if (!uniqueFaces.insert(sorted).second) return false;
        used.insert(v.begin(), v.end());
        const std::array<std::pair<std::size_t, std::size_t>, 3> directed{{
            {v[0], v[1]}, {v[1], v[2]}, {v[2], v[0]}}};
        for (const auto& e : directed) {
            const Edge key = MakeEdge(e.first, e.second);
            auto& value = edges[key]; ++value.first;
            value.second += e.first == key.first ? 1 : -1;
        }
        const Point& a = vertices[v[0]].point;
        volumeSix += Dot(a, Cross(vertices[v[1]].point, vertices[v[2]].point));
    }
    if (!(std::isfinite(volumeSix) && volumeSix > 0) || used.empty()) return false;
    for (const auto& item : edges)
        if (item.second.first != 2 || item.second.second != 0) return false;
    if (static_cast<int>(used.size()) - static_cast<int>(edges.size())
        + static_cast<int>(uniqueFaces.size()) != expectedEuler) return false;

    // A closed edge-incidence count does not catch two fans meeting at one
    // vertex. The neighbor link of every vertex must be one cycle.
    for (std::size_t center : used) {
        std::map<std::size_t, std::set<std::size_t>> link;
        for (const Face& face : faces) {
            if (!face.alive) continue;
            auto found = std::find(face.vertex.begin(), face.vertex.end(), center);
            if (found == face.vertex.end()) continue;
            const std::size_t i = static_cast<std::size_t>(found - face.vertex.begin());
            const std::size_t a = face.vertex[(i + 1) % 3];
            const std::size_t b = face.vertex[(i + 2) % 3];
            link[a].insert(b); link[b].insert(a);
        }
        if (link.empty()) return false;
        for (const auto& item : link) if (item.second.size() != 2) return false;
        std::set<std::size_t> reached;
        std::vector<std::size_t> stack{link.begin()->first};
        while (!stack.empty()) {
            const auto value = stack.back(); stack.pop_back();
            if (!reached.insert(value).second) continue;
            for (auto next : link[value]) stack.push_back(next);
        }
        if (reached.size() != link.size()) return false;
    }
    return true;
}

struct Candidate {
    double cost = 0;
    std::size_t low = 0, high = 0;
    unsigned kind = 0; // low, high, midpoint
    Point point;
};
inline bool operator<(const Candidate& a, const Candidate& b) noexcept {
    return std::tie(a.cost, a.low, a.high, a.kind)
         < std::tie(b.cost, b.low, b.high, b.kind);
}

inline std::vector<Edge> CurrentEdges(const std::vector<Face>& faces) {
    std::set<Edge> values;
    for (const Face& face : faces) if (face.alive) {
        values.insert(MakeEdge(face.vertex[0], face.vertex[1]));
        values.insert(MakeEdge(face.vertex[1], face.vertex[2]));
        values.insert(MakeEdge(face.vertex[2], face.vertex[0]));
    }
    return {values.begin(), values.end()};
}

inline bool TryCollapse(std::vector<Vertex>& vertices, std::vector<Face>& faces,
                        const Candidate& candidate, const std::set<Edge>& protectedEdges,
                        int expectedEuler, std::uint64_t& work,
                        const std::atomic_bool* stop) {
    const std::size_t keep = candidate.kind == 1 ? candidate.high : candidate.low;
    const std::size_t remove = keep == candidate.low ? candidate.high : candidate.low;
    if (!vertices[keep].alive || !vertices[remove].alive || vertices[remove].locked) return false;
    if (protectedEdges.count(MakeEdge(keep, remove))) return false;
    if (candidate.kind == 2 && (vertices[keep].locked || vertices[remove].locked)) return false;

    auto nextVertices = vertices;
    auto nextFaces = faces;
    const Point oldKeep = nextVertices[keep].point;
    nextVertices[keep].point = candidate.point;
    nextVertices[keep].quadric += nextVertices[remove].quadric;
    nextVertices[remove].alive = false;
    std::vector<std::string> patchTokens;
    std::vector<std::size_t> patch;
    std::size_t removedFaces = 0;
    for (std::size_t index = 0; index < nextFaces.size(); ++index) {
        Face& face = nextFaces[index];
        if (!face.alive) continue;
        const bool affected = std::find(face.vertex.begin(), face.vertex.end(), keep) != face.vertex.end()
                           || std::find(face.vertex.begin(), face.vertex.end(), remove) != face.vertex.end();
        if (!affected) continue;
        patchTokens.insert(patchTokens.end(), face.tokens.begin(), face.tokens.end());
        const Point before = Normal(faces[index], vertices);
        for (auto& vertex : face.vertex) if (vertex == remove) vertex = keep;
        if (face.vertex[0] == face.vertex[1] || face.vertex[1] == face.vertex[2]
            || face.vertex[2] == face.vertex[0]) {
            face.alive = false; ++removedFaces; continue;
        }
        const Point after = Normal(face, nextVertices);
        if (!Finite(after) || Dot(after, after) == 0 || Dot(before, after) <= 0) return false;
        patch.push_back(index);
    }
    if (removedFaces != 2) return false;
    std::sort(patchTokens.begin(), patchTokens.end());
    patchTokens.erase(std::unique(patchTokens.begin(), patchTokens.end()), patchTokens.end());
    for (std::size_t index : patch) if (nextFaces[index].alive) nextFaces[index].tokens = patchTokens;
    if (!Validate(nextVertices, nextFaces, work, stop, expectedEuler)) return false;
    const auto currentEdges = CurrentEdges(nextFaces);
    const std::set<Edge> current(currentEdges.begin(), currentEdges.end());
    for (const Edge& edge : protectedEdges) if (!current.count(edge)) return false;
    (void)oldKeep;
    vertices.swap(nextVertices); faces.swap(nextFaces);
    return true;
}

} // namespace detail

inline Error Decimate(const Input& input, Output& output,
                      const std::atomic_bool* stop = nullptr) {
    output = {};
    if (input.targetTriangles < 4) return Error::TargetBelowClosedMeshMinimum;
    if (input.indices.empty() || input.indices.size() % 3 != 0
        || input.indices.size() / 3 > kMaximumTriangles
        || input.positionsMM.size() > kMaximumPositions
        || input.contributionTokens.size() != input.indices.size() / 3)
        return Error::ResourceLimit;
    if (stop && stop->load(std::memory_order_relaxed)) return Error::Cancelled;

    std::vector<Point> canonical = input.positionsMM;
    for (Point& p : canonical) {
        if (!Finite(p)) return Error::NonFiniteInput;
        if (p.x == 0) p.x = 0; if (p.y == 0) p.y = 0; if (p.z == 0) p.z = 0;
    }
    std::sort(canonical.begin(), canonical.end());
    canonical.erase(std::unique(canonical.begin(), canonical.end()), canonical.end());
    std::vector<detail::Vertex> vertices;
    vertices.reserve(canonical.size());
    for (std::size_t rank = 0; rank < canonical.size(); ++rank)
        vertices.push_back({canonical[rank], {}, rank, true, false});
    std::vector<detail::Face> faces;
    faces.reserve(input.indices.size() / 3);
    for (std::size_t f = 0; f < input.indices.size() / 3; ++f) {
        detail::Face face;
        for (std::size_t c = 0; c < 3; ++c) {
            const std::uint32_t source = input.indices[3*f+c];
            if (source >= input.positionsMM.size()) return Error::InvalidTopology;
            const Point p = input.positionsMM[source];
            auto found = std::lower_bound(canonical.begin(), canonical.end(), p);
            if (found == canonical.end() || !(*found == p)) return Error::InvalidTopology;
            face.vertex[c] = static_cast<std::size_t>(found - canonical.begin());
        }
        if (input.contributionTokens[f].empty()) return Error::InvalidTopology;
        face.tokens = {input.contributionTokens[f]};
        faces.push_back(std::move(face));
    }

    std::uint64_t work = 0;
    const int sourceEuler = detail::EulerCharacteristic(faces);
    if (!detail::Validate(vertices, faces, work, stop, sourceEuler))
        return stop && stop->load(std::memory_order_relaxed) ? Error::Cancelled : Error::InvalidTopology;

    // Original, normalized, unweighted plane quadrics in canonical face order.
    std::vector<std::size_t> faceOrder(faces.size());
    for (std::size_t i = 0; i < faceOrder.size(); ++i) faceOrder[i] = i;
    std::sort(faceOrder.begin(), faceOrder.end(), [&](std::size_t l, std::size_t r) {
        auto a = faces[l].vertex, b = faces[r].vertex;
        std::rotate(a.begin(), std::min_element(a.begin(), a.end()), a.end());
        std::rotate(b.begin(), std::min_element(b.begin(), b.end()), b.end());
        return a < b;
    });
    for (std::size_t fi : faceOrder) {
        const auto& face = faces[fi];
        const Point& a = vertices[face.vertex[0]].point;
        const Point n = detail::Normal(face, vertices);
        const double s = Dot(n, n), d = -Dot(n, a);
        if (!(std::isfinite(s) && s > 0 && std::isfinite(d))) return Error::NonFiniteInput;
        const std::array<double, 4> p{{n.x, n.y, n.z, d}};
        detail::Quadric q;
        q.v = {{p[0]*p[0]/s,p[0]*p[1]/s,p[0]*p[2]/s,p[0]*p[3]/s,
                p[1]*p[1]/s,p[1]*p[2]/s,p[1]*p[3]/s,p[2]*p[2]/s,
                p[2]*p[3]/s,p[3]*p[3]/s}};
        for (auto vertex : face.vertex) vertices[vertex].quadric += q;
    }

    // Freeze the >=45 degree source hard graph.
    std::map<detail::Edge, std::vector<std::size_t>> incidents;
    for (std::size_t fi = 0; fi < faces.size(); ++fi) {
        const auto& v = faces[fi].vertex;
        incidents[detail::MakeEdge(v[0],v[1])].push_back(fi);
        incidents[detail::MakeEdge(v[1],v[2])].push_back(fi);
        incidents[detail::MakeEdge(v[2],v[0])].push_back(fi);
    }
    std::set<detail::Edge> protectedEdges;
    const double cosineThreshold = std::cos(45.0 * std::acos(-1.0) / 180.0);
    for (const auto& item : incidents) {
        if (item.second.size() != 2) return Error::InvalidTopology;
        const Point a = detail::Normal(faces[item.second[0]], vertices);
        const Point b = detail::Normal(faces[item.second[1]], vertices);
        const double divisor = std::sqrt(Dot(a,a) * Dot(b,b));
        if (!(std::isfinite(divisor) && divisor > 0)) return Error::InvalidTopology;
        const double cosine = Dot(a,b) / divisor;
        if (!std::isfinite(cosine)) return Error::InvalidTopology;
        if (cosine <= cosineThreshold) {
            protectedEdges.insert(item.first);
            vertices[item.first.first].locked = true;
            vertices[item.first.second].locked = true;
        }
    }

    std::uint64_t evaluations = 0;
    while (detail::FaceCount(faces) > input.targetTriangles) {
        if (!detail::Charge(work, stop))
            return stop && stop->load(std::memory_order_relaxed) ? Error::Cancelled : Error::ResourceLimit;
        std::vector<detail::Candidate> candidates;
        for (const auto& edge : detail::CurrentEdges(faces)) {
            if (protectedEdges.count(edge)) continue;
            const auto& low = vertices[edge.first]; const auto& high = vertices[edge.second];
            if (!high.locked) candidates.push_back({detail::Cost(low.quadric + high.quadric, low.point),
                                                     edge.first, edge.second, 0, low.point});
            if (!low.locked) candidates.push_back({detail::Cost(low.quadric + high.quadric, high.point),
                                                    edge.first, edge.second, 1, high.point});
            if (!low.locked && !high.locked) {
                const Point midpoint = (low.point + high.point) * 0.5;
                candidates.push_back({detail::Cost(low.quadric + high.quadric, midpoint),
                                      edge.first, edge.second, 2, midpoint});
            }
        }
        for (const auto& c : candidates) if (!std::isfinite(c.cost) || !Finite(c.point))
            return Error::NonFiniteInput;
        std::sort(candidates.begin(), candidates.end());
        bool accepted = false;
        for (const auto& candidate : candidates) {
            if (++evaluations > kMaximumCandidateEvaluations) return Error::ResourceLimit;
            if (!detail::Charge(work, stop))
                return stop && stop->load(std::memory_order_relaxed) ? Error::Cancelled : Error::ResourceLimit;
            if (detail::TryCollapse(vertices, faces, candidate, protectedEdges,
                                    sourceEuler, work, stop)) {
                ++output.acceptedCollapses; accepted = true; break;
            }
        }
        if (!accepted) return stop && stop->load(std::memory_order_relaxed)
            ? Error::Cancelled : Error::UnreachableTriangleBudget;
    }

    std::vector<std::size_t> order;
    for (std::size_t i = 0; i < faces.size(); ++i) if (faces[i].alive) order.push_back(i);
    std::sort(order.begin(), order.end(), [&](std::size_t l, std::size_t r) {
        auto a = faces[l].vertex, b = faces[r].vertex;
        std::rotate(a.begin(), std::min_element(a.begin(), a.end()), a.end());
        std::rotate(b.begin(), std::min_element(b.begin(), b.end()), b.end());
        return a < b;
    });
    for (std::size_t fi : order) {
        const auto& face = faces[fi];
        const Point normal = detail::Normal(face, vertices);
        const double length = std::sqrt(Dot(normal, normal));
        if (!(std::isfinite(length) && length > 0)) return Error::InvalidTopology;
        const Point unit = normal * (1.0 / length);
        for (auto vi : face.vertex) {
            const Point p = vertices[vi].point;
            for (double value : {p.x,p.y,p.z}) {
                const float encoded = static_cast<float>(value);
                if (!std::isfinite(encoded)) return Error::NonFiniteInput;
                output.positionsMM.push_back(encoded == 0 ? 0 : encoded);
            }
            output.flatNormals.insert(output.flatNormals.end(),
                {static_cast<float>(unit.x), static_cast<float>(unit.y), static_cast<float>(unit.z)});
            output.indices.push_back(static_cast<std::uint32_t>(output.indices.size()));
        }
        output.contributionTokens.push_back(face.tokens);
    }
    return Error::None;
}

} // namespace core3d::export_mesh_decimator
