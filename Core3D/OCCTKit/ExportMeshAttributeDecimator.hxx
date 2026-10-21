#pragma once

// F2-S3 attribute-aware decimation kernel (DR-F2 S3, head-dev D1048). The
// collapse respects material boundaries, UV seams, authored normal seams and
// the frozen >=45 degree geometric hard-edge graph: a collapse across any of
// those boundaries is refused for that pair and attribute values are never
// averaged across them. Normal/UV values are carried per face corner from the
// surviving wedge of each accepted collapse, never re-derived by smoothing.
// Every emitted triangle keeps explicit provenance to exactly one source face
// (its origin face ordinal) in addition to the v1 contribution-token union,
// and the accepted (keep, remove) collapse sequence is emitted so the opening
// and the contract verifier can independently replay every carried wedge.
//
// The v1 geometry law is reused unchanged from ExportMeshDecimator.hxx: the
// same welded canonical positions, unweighted plane quadrics, candidate cost
// and tie order, TryCollapse topology guards, source Euler preservation,
// protected-graph endpoint locks, charged work/Stop accounting and the same
// caps. Two deliberate, disclosed additions make attribute carrying provable:
//  - the protected graph is extended with material-boundary and
//    wedge-discontinuity edges (endpoints locked exactly like v1 hard edges);
//  - candidate placements are endpoint-only (a restriction, never a
//    relaxation), so every emitted corner sits exactly on a surviving source
//    stored position and its attributes come from one surviving source wedge.
// No v1 topology guard, bound or Stop handling is relaxed.

#include "ExportMeshDecimator.hxx"

namespace core3d::export_mesh_attribute_decimator {

namespace v1 = core3d::export_mesh_decimator;

// One stored source vertex's authored attributes. Normals are right-handed
// Z-up world directions exactly as independently observed; UVs are the
// authored TEXCOORD_0 values. Equality is bitwise: no smoothing tolerance.
struct Wedge {
    double nx = 0, ny = 0, nz = 0;
    double u = 0, v = 0;
};
inline bool operator==(const Wedge& a, const Wedge& b) noexcept {
    return a.nx == b.nx && a.ny == b.ny && a.nz == b.nz && a.u == b.u && a.v == b.v;
}

enum class Error {
    None,
    TargetBelowClosedMeshMinimum,
    InvalidTopology,
    NonFiniteInput,
    ResourceLimit,
    Cancelled,
    UnreachableTriangleBudget,
    ProvenanceUnprovable,
    AttributeDomainAmbiguous,
};

struct Input {
    std::vector<v1::Point> positionsMM;          // stored source vertices
    std::vector<std::uint32_t> indices;          // 3 per face, stored-vertex space
    std::vector<Wedge> wedges;                   // one per stored vertex
    std::vector<std::uint32_t> faceMaterials;    // one per face
    std::vector<std::string> contributionTokens; // one per face (v1 remap unions)
    std::vector<std::string> sourceFaceAnchors;  // one per face (single-face provenance)
    std::size_t targetTriangles = 0;
};

struct Output {
    // Same all-corners-split layout as v1: indices are the deterministic
    // sequence 0...3F-1 so carried wedges stay unambiguous per corner.
    std::vector<float> positionsMM;
    std::vector<float> normals;                  // carried wedge normals, 3 per corner
    std::vector<float> uvs;                      // carried wedge UVs, 2 per corner
    std::vector<std::uint32_t> indices;
    std::vector<std::uint32_t> faceMaterials;    // one per emitted triangle
    std::vector<std::uint32_t> sourceFaceOrdinals; // one per emitted triangle
    std::vector<std::vector<std::string>> contributionTokens;
    // Every accepted collapse as (keep, remove) welded-vertex ranks, in
    // acceptance order. Independent verifiers replay this sequence to prove
    // each carried wedge instead of trusting the kernel's bookkeeping.
    std::vector<std::pair<std::size_t, std::size_t>> collapseKeepRemove;
    std::size_t acceptedCollapses = 0;
};

inline Error Decimate(const Input& input, Output& output,
                      const std::atomic_bool* stop = nullptr) {
    output = {};
    if (input.targetTriangles < 4) return Error::TargetBelowClosedMeshMinimum;
    const std::size_t faceCount = input.indices.size() / 3;
    if (input.indices.empty() || input.indices.size() % 3 != 0
        || faceCount > v1::kMaximumTriangles
        || input.positionsMM.size() > v1::kMaximumPositions
        || input.contributionTokens.size() != faceCount
        || input.faceMaterials.size() != faceCount
        || input.sourceFaceAnchors.size() != faceCount
        || input.wedges.size() != input.positionsMM.size())
        return Error::ResourceLimit;
    if (stop && stop->load(std::memory_order_relaxed)) return Error::Cancelled;
    for (const std::string& anchor : input.sourceFaceAnchors)
        if (anchor.empty()) return Error::ProvenanceUnprovable;

    // v1 canonical weld: exact positions only, signed zero normalized.
    std::vector<v1::Point> canonical = input.positionsMM;
    for (v1::Point& p : canonical) {
        if (!v1::Finite(p)) return Error::NonFiniteInput;
        if (p.x == 0) p.x = 0; if (p.y == 0) p.y = 0; if (p.z == 0) p.z = 0;
    }
    std::sort(canonical.begin(), canonical.end());
    canonical.erase(std::unique(canonical.begin(), canonical.end()), canonical.end());
    std::vector<v1::detail::Vertex> vertices;
    vertices.reserve(canonical.size());
    for (std::size_t rank = 0; rank < canonical.size(); ++rank)
        vertices.push_back({canonical[rank], {}, rank, true, false});
    std::vector<v1::detail::Face> faces;
    faces.reserve(faceCount);
    // Per-corner carried wedges, one per (face, corner slot). Initially each
    // slot carries its stored source vertex's wedge; an accepted collapse
    // replaces the removed endpoint's slot wedges with the surviving wedge.
    std::vector<std::array<Wedge, 3>> faceWedges;
    faceWedges.reserve(faceCount);
    for (std::size_t f = 0; f < faceCount; ++f) {
        v1::detail::Face face;
        std::array<Wedge, 3> wedges{};
        for (std::size_t c = 0; c < 3; ++c) {
            const std::uint32_t source = input.indices[3 * f + c];
            if (source >= input.positionsMM.size()) return Error::InvalidTopology;
            wedges[c] = input.wedges[source];
            const v1::Point p = input.positionsMM[source];
            auto found = std::lower_bound(canonical.begin(), canonical.end(), p);
            if (found == canonical.end() || !(*found == p)) return Error::InvalidTopology;
            face.vertex[c] = static_cast<std::size_t>(found - canonical.begin());
        }
        if (input.contributionTokens[f].empty()) return Error::InvalidTopology;
        face.tokens = {input.contributionTokens[f]};
        faces.push_back(face);
        faceWedges.push_back(wedges);
    }
    for (const Wedge& wedge : input.wedges) {
        const double lengthSquared = wedge.nx * wedge.nx + wedge.ny * wedge.ny + wedge.nz * wedge.nz;
        if (!std::isfinite(wedge.nx) || !std::isfinite(wedge.ny) || !std::isfinite(wedge.nz)
            || !std::isfinite(lengthSquared) || lengthSquared <= 0
            || !std::isfinite(wedge.u) || !std::isfinite(wedge.v))
            return Error::NonFiniteInput;
    }

    std::uint64_t work = 0;
    const int sourceEuler = v1::detail::EulerCharacteristic(faces);
    if (!v1::detail::Validate(vertices, faces, work, stop, sourceEuler))
        return stop && stop->load(std::memory_order_relaxed) ? Error::Cancelled : Error::InvalidTopology;

    // v1 original, normalized, unweighted plane quadrics in canonical face order.
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
        const v1::Point& a = vertices[face.vertex[0]].point;
        const v1::Point n = v1::detail::Normal(face, vertices);
        const double s = v1::Dot(n, n), d = -v1::Dot(n, a);
        if (!(std::isfinite(s) && s > 0 && std::isfinite(d))) return Error::NonFiniteInput;
        const std::array<double, 4> p{{n.x, n.y, n.z, d}};
        v1::detail::Quadric q;
        q.v = {{p[0]*p[0]/s,p[0]*p[1]/s,p[0]*p[2]/s,p[0]*p[3]/s,
                p[1]*p[1]/s,p[1]*p[2]/s,p[1]*p[3]/s,p[2]*p[2]/s,
                p[2]*p[3]/s,p[3]*p[3]/s}};
        for (auto vertex : face.vertex) vertices[vertex].quadric += q;
    }

    // Protected graph: the frozen v1 >=45 degree geometric hard graph plus
    // material boundaries and stored-wedge discontinuities (UV seams and
    // authored normal seams). Endpoints lock exactly like v1 hard edges.
    std::map<v1::detail::Edge, std::vector<std::size_t>> incidents;
    for (std::size_t fi = 0; fi < faces.size(); ++fi) {
        const auto& v = faces[fi].vertex;
        incidents[v1::detail::MakeEdge(v[0], v[1])].push_back(fi);
        incidents[v1::detail::MakeEdge(v[1], v[2])].push_back(fi);
        incidents[v1::detail::MakeEdge(v[2], v[0])].push_back(fi);
    }
    std::set<v1::detail::Edge> protectedEdges;
    const double cosineThreshold = std::cos(45.0 * std::acos(-1.0) / 180.0);
    for (const auto& item : incidents) {
        if (item.second.size() != 2) return Error::InvalidTopology;
        const std::size_t first = item.second[0], second = item.second[1];
        const v1::Point a = v1::detail::Normal(faces[first], vertices);
        const v1::Point b = v1::detail::Normal(faces[second], vertices);
        const double divisor = std::sqrt(v1::Dot(a, a) * v1::Dot(b, b));
        if (!(std::isfinite(divisor) && divisor > 0)) return Error::InvalidTopology;
        const double cosine = v1::Dot(a, b) / divisor;
        if (!std::isfinite(cosine)) return Error::InvalidTopology;
        bool attributeBoundary = input.faceMaterials[first] != input.faceMaterials[second];
        for (std::size_t corner = 0; corner < 3 && !attributeBoundary; ++corner) {
            const std::size_t welded = faces[first].vertex[corner];
            if (welded != item.first.first && welded != item.first.second) continue;
            const auto other = std::find(faces[second].vertex.begin(),
                faces[second].vertex.end(), welded);
            if (other == faces[second].vertex.end()) return Error::InvalidTopology;
            const std::size_t otherCorner =
                static_cast<std::size_t>(other - faces[second].vertex.begin());
            // A wedge discontinuity at either shared endpoint cuts the pair:
            // the two faces may address different stored vertices (UV seam) or
            // carry unequal authored normals/UVs at the same welded position.
            if (!(faceWedges[first][corner] == faceWedges[second][otherCorner]))
                attributeBoundary = true;
        }
        if (cosine <= cosineThreshold || attributeBoundary) {
            protectedEdges.insert(item.first);
            vertices[item.first.first].locked = true;
            vertices[item.first.second].locked = true;
        }
    }

    // v1 collapse loop with endpoint-only placements. On each accepted
    // collapse the removed endpoint's slot wedges are replaced by the
    // surviving wedge: the shared keep-slot wedge of the two collapsed-edge
    // faces, which the unprotected edge guarantees is one identical value.
    std::uint64_t evaluations = 0;
    while (v1::detail::FaceCount(faces) > input.targetTriangles) {
        if (!v1::detail::Charge(work, stop))
            return stop && stop->load(std::memory_order_relaxed) ? Error::Cancelled : Error::ResourceLimit;
        std::vector<v1::detail::Candidate> candidates;
        for (const auto& edge : v1::detail::CurrentEdges(faces)) {
            if (protectedEdges.count(edge)) continue;
            const auto& low = vertices[edge.first];
            const auto& high = vertices[edge.second];
            if (!high.locked)
                candidates.push_back({v1::detail::Cost(low.quadric + high.quadric, low.point),
                                      edge.first, edge.second, 0, low.point});
            if (!low.locked)
                candidates.push_back({v1::detail::Cost(low.quadric + high.quadric, high.point),
                                      edge.first, edge.second, 1, high.point});
        }
        for (const auto& c : candidates) if (!std::isfinite(c.cost) || !v1::Finite(c.point))
            return Error::NonFiniteInput;
        std::sort(candidates.begin(), candidates.end());
        bool accepted = false;
        for (const auto& candidate : candidates) {
            if (++evaluations > v1::kMaximumCandidateEvaluations) return Error::ResourceLimit;
            if (!v1::detail::Charge(work, stop))
                return stop && stop->load(std::memory_order_relaxed) ? Error::Cancelled : Error::ResourceLimit;
            const std::size_t keep = candidate.kind == 1 ? candidate.high : candidate.low;
            const std::size_t remove = keep == candidate.low ? candidate.high : candidate.low;
            // Incidence snapshot for wedge carrying: the two collapsed-edge
            // faces (both endpoints) and every removed-endpoint slot.
            std::vector<std::pair<std::size_t, std::size_t>> deadKeepSlots;
            std::vector<std::pair<std::size_t, std::size_t>> removeSlots;
            for (std::size_t f = 0; f < faces.size(); ++f) {
                if (!faces[f].alive) continue;
                bool hasKeep = false, hasRemove = false;
                std::size_t keepSlot = 0;
                for (std::size_t c = 0; c < 3; ++c) {
                    if (faces[f].vertex[c] == keep) { hasKeep = true; keepSlot = c; }
                    if (faces[f].vertex[c] == remove) hasRemove = true;
                }
                if (hasKeep && hasRemove) {
                    deadKeepSlots.push_back({f, keepSlot});
                } else if (hasRemove) {
                    for (std::size_t c = 0; c < 3; ++c)
                        if (faces[f].vertex[c] == remove) removeSlots.push_back({f, c});
                }
            }
            if (v1::detail::TryCollapse(vertices, faces, candidate, protectedEdges,
                                        sourceEuler, work, stop)) {
                // v1 guarantees exactly the two collapsed-edge faces died.
                if (deadKeepSlots.size() != 2) return Error::AttributeDomainAmbiguous;
                const Wedge& carried =
                    faceWedges[deadKeepSlots[0].first][deadKeepSlots[0].second];
                if (!(carried == faceWedges[deadKeepSlots[1].first][deadKeepSlots[1].second]))
                    return Error::AttributeDomainAmbiguous;
                for (const auto& slot : removeSlots) faceWedges[slot.first][slot.second] = carried;
                output.collapseKeepRemove.push_back({keep, remove});
                ++output.acceptedCollapses; accepted = true; break;
            }
        }
        if (!accepted) return stop && stop->load(std::memory_order_relaxed)
            ? Error::Cancelled : Error::UnreachableTriangleBudget;
    }

    // v1 canonical emitted order; attributes from each corner's carried wedge.
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
        for (std::size_t c = 0; c < 3; ++c) {
            const v1::Point& p = vertices[face.vertex[c]].point;
            for (double value : {p.x, p.y, p.z}) {
                const float encoded = static_cast<float>(value);
                if (!std::isfinite(encoded)) return Error::NonFiniteInput;
                output.positionsMM.push_back(encoded == 0 ? 0 : encoded);
            }
            const Wedge& wedge = faceWedges[fi][c];
            for (double value : {wedge.nx, wedge.ny, wedge.nz}) {
                const float encoded = static_cast<float>(value);
                if (!std::isfinite(encoded)) return Error::NonFiniteInput;
                output.normals.push_back(encoded == 0 ? 0 : encoded);
            }
            for (double value : {wedge.u, wedge.v}) {
                const float encoded = static_cast<float>(value);
                if (!std::isfinite(encoded)) return Error::NonFiniteInput;
                output.uvs.push_back(encoded == 0 ? 0 : encoded);
            }
            output.indices.push_back(static_cast<std::uint32_t>(output.indices.size()));
        }
        output.faceMaterials.push_back(input.faceMaterials[fi]);
        output.sourceFaceOrdinals.push_back(static_cast<std::uint32_t>(fi));
        output.contributionTokens.push_back(face.tokens);
    }
    return Error::None;
}

} // namespace core3d::export_mesh_attribute_decimator
