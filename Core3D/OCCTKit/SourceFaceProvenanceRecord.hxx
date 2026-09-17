#pragma once
// Source-face provenance persistence record (schema v1) for source-retained
// mesh copies. This header is the serialization-independent payload codec and
// digest; OcctDocument owns the OCAF labels, attributes and strict reader.
//
// Schema and frame contract (fixed):
// - The record is stored on a dedicated child label under the copy's
//   definition label using only attribute types already admitted by the
//   narrow document reader (a TDataStd_UAttribute marker, TDataStd_Integer
//   scalars, and chunked TDataStd_AsciiString payload plus a 64-hex digest
//   attribute). No new attribute driver type exists, so shipped builds keep
//   opening these documents.
// - Per-face triangle ranges refer to the PRE-ATLAS emission order of the
//   copy's triangulation (the order produced by PrepareCurrentTessellationCopy:
//   source faces in exploration order, each face's triangles contiguous).
//   Order-preserving atlas rebuilds keep those ranges valid; any rebuild that
//   reorders triangles invalidates them, the strict reader reports Stale, and
//   consumers must ignore the record.
// - Analytic parameters are recorded in the PART-LOCAL frame of the copy
//   nodes, the same frame as CurrentTessellationMeshCopy.hxx: the
//   BRepAdaptor_Surface already applies the source face location exactly
//   once. The copy's eight-scalar object transform is persisted separately
//   and is never applied to these parameters.
// - The digest is sha256 over the copy's current triangulation in emission
//   order: per triangle its three corner indices as offsets relative to the
//   triangle's first corner (decimal text), then each referenced corner
//   node's xyz as exact %a hex-float text, in stored corner order. Absolute
//   node placement is deliberately not hashed: an order-preserving atlas
//   rebuild (the v1/v2 original-node prefix) shifts absolute indices without
//   changing geometry, winding, or triangle order, and must not invalidate
//   the record; per-face ranges refer to the pre-atlas emission order, which
//   such rebuilds preserve. ANY later mesh mutation
//   (MeshRegionExtrude/Inset/VertexMove, an atlas rebuild that reorders
//   triangles, or a winding change) changes the digest; the strict reader
//   then reports Stale and consumers must ignore the record. There is no
//   silent fallback and no source-to-copy link.
#include "CurrentTessellationMeshCopy.hxx"
#include <CommonCrypto/CommonDigest.h>
#include <Poly_Triangulation.hxx>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace core3d::provenance {
// Serialized surface-type values are persistent schema identifiers: never
// renumber or reuse them.
inline constexpr int kSurfaceTypePlane = 0;
inline constexpr int kSurfaceTypeCylinder = 1;
inline constexpr int kSurfaceTypeTorus = 2;
inline constexpr int kSurfaceTypeOther = 3;
inline constexpr int kMaximumProvenanceFaces = 256;
inline constexpr int kMaximumProvenanceTriangles = 4096;

struct SourceFaceProvenanceEntry {
    int surfaceType = kSurfaceTypeOther;
    // Zero-based offset of this source face's triangles in the PRE-ATLAS
    // emission order of the copy triangulation.
    int firstTriangle = 0, triangleCount = 0;
    bool reversed = false;
    // Exact analytic scalars in part-local frame, encoded as %a hex-float
    // text on disk. Plane: location xyz, normal xyz, x-direction xyz.
    // Cylinder: axis location xyz, axis direction xyz, x-direction xyz,
    // radius. Torus: same plus major and minor radius. Other: empty.
    std::vector<double> params;
};
struct SourceFaceProvenanceRecord {
    // 64 lower-hex sha256 over the copy triangulation; empty unless the
    // record is structurally valid.
    std::string digest;
    int triangleCount = 0;
    std::vector<SourceFaceProvenanceEntry> faces;
};
enum class CopySourceFaceProvenanceReadState : int { Absent = 0, Malformed = 1, Stale = 2, Present = 3 };

inline int ParameterCountForSurfaceType(int surfaceType) noexcept {
    switch (surfaceType) {
        case kSurfaceTypePlane: return 9;
        case kSurfaceTypeCylinder: return 10;
        case kSurfaceTypeTorus: return 11;
        default: return surfaceType == kSurfaceTypeOther ? 0 : -1;
    }
}

inline bool SameDoubleBits(double a, double b) noexcept {
    std::uint64_t x = 0, y = 0;
    static_assert(sizeof x == sizeof a, "Double bit comparison requires 64-bit doubles");
    std::memcpy(&x, &a, sizeof x); std::memcpy(&y, &b, sizeof y);
    return x == y;
}

inline bool SameEntries(const std::vector<SourceFaceProvenanceEntry>& a,
                        const std::vector<SourceFaceProvenanceEntry>& b) noexcept {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].surfaceType != b[i].surfaceType || a[i].firstTriangle != b[i].firstTriangle
            || a[i].triangleCount != b[i].triangleCount || a[i].reversed != b[i].reversed
            || a[i].params.size() != b[i].params.size()) return false;
        for (std::size_t p = 0; p < a[i].params.size(); ++p)
            if (!SameDoubleBits(a[i].params[p], b[i].params[p])) return false;
    }
    return true;
}

// Extract exact scalars from the N1 in-memory provenance. Never fitted from
// mesh nodes; every value must be finite.
inline bool EntriesFromSourceFaces(const std::vector<core3d::meshcopy::SourceFaceRecord>& faces,
                                   std::vector<SourceFaceProvenanceEntry>& output) noexcept {
    output.clear();
    if (faces.empty() || faces.size() > std::size_t(kMaximumProvenanceFaces)) return false;
    try {
        auto point = [](const gp_Pnt& p) { return std::vector<double>{p.X(), p.Y(), p.Z()}; };
        auto direction = [](const gp_Dir& d) { return std::vector<double>{d.X(), d.Y(), d.Z()}; };
        auto append = [](std::vector<double>& a, std::vector<double>&& b) {
            a.insert(a.end(), b.begin(), b.end());
        };
        std::vector<SourceFaceProvenanceEntry> entries;
        entries.reserve(faces.size());
        for (const auto& face : faces) {
            SourceFaceProvenanceEntry entry;
            entry.firstTriangle = face.firstTriangle;
            entry.triangleCount = face.triangleCount;
            entry.reversed = face.reversed;
            switch (face.surfaceType) {
                case core3d::meshcopy::SourceFaceRecord::SurfaceType::Plane: {
                    const auto* plane = std::get_if<gp_Pln>(&face.params);
                    if (!plane) return false;
                    entry.surfaceType = kSurfaceTypePlane;
                    append(entry.params, point(plane->Location()));
                    append(entry.params, direction(plane->Axis().Direction()));
                    append(entry.params, direction(plane->Position().XDirection()));
                    break;
                }
                case core3d::meshcopy::SourceFaceRecord::SurfaceType::Cylinder: {
                    const auto* cylinder = std::get_if<gp_Cylinder>(&face.params);
                    if (!cylinder || !std::isfinite(cylinder->Radius())) return false;
                    entry.surfaceType = kSurfaceTypeCylinder;
                    append(entry.params, point(cylinder->Axis().Location()));
                    append(entry.params, direction(cylinder->Axis().Direction()));
                    append(entry.params, direction(cylinder->Position().XDirection()));
                    entry.params.push_back(cylinder->Radius());
                    break;
                }
                case core3d::meshcopy::SourceFaceRecord::SurfaceType::Torus: {
                    const auto* torus = std::get_if<gp_Torus>(&face.params);
                    if (!torus || !std::isfinite(torus->MajorRadius()) || !std::isfinite(torus->MinorRadius()))
                        return false;
                    entry.surfaceType = kSurfaceTypeTorus;
                    append(entry.params, point(torus->Axis().Location()));
                    append(entry.params, direction(torus->Axis().Direction()));
                    append(entry.params, direction(torus->Position().XDirection()));
                    entry.params.push_back(torus->MajorRadius());
                    entry.params.push_back(torus->MinorRadius());
                    break;
                }
                case core3d::meshcopy::SourceFaceRecord::SurfaceType::Other:
                    entry.surfaceType = kSurfaceTypeOther;
                    break;
            }
            const int expected = ParameterCountForSurfaceType(entry.surfaceType);
            if (expected < 0 || entry.params.size() != std::size_t(expected)) return false;
            for (double value : entry.params) if (!std::isfinite(value)) return false;
            entries.push_back(std::move(entry));
        }
        output = std::move(entries);
        return true;
    } catch (...) { output.clear(); return false; }
}

inline bool SameSourceFaces(const std::vector<SourceFaceProvenanceEntry>& entries,
                            const std::vector<core3d::meshcopy::SourceFaceRecord>& faces) noexcept {
    std::vector<SourceFaceProvenanceEntry> expected;
    return EntriesFromSourceFaces(faces, expected) && SameEntries(entries, expected);
}

inline bool AppendHexFloat(std::string& output, double value) noexcept {
    char buffer[64];
    const int length = std::snprintf(buffer, sizeof buffer, "%a", value);
    if (length <= 0 || std::size_t(length) >= sizeof buffer) return false;
    output.append(buffer, std::size_t(length));
    return true;
}

// Payload text: one line per face, fields separated by single spaces, lines
// joined by '\n' with no trailing newline. Scalars are exact %a hex-floats.
inline bool EncodeEntries(const std::vector<SourceFaceProvenanceEntry>& faces,
                          std::string& output) noexcept {
    output.clear();
    if (faces.empty() || faces.size() > std::size_t(kMaximumProvenanceFaces)) return false;
    try {
        std::string text;
        for (std::size_t i = 0; i < faces.size(); ++i) {
            const auto& entry = faces[i];
            const int parameters = ParameterCountForSurfaceType(entry.surfaceType);
            if (parameters < 0 || entry.params.size() != std::size_t(parameters)
                || entry.firstTriangle < 0 || entry.firstTriangle > kMaximumProvenanceTriangles
                || entry.triangleCount <= 0 || entry.triangleCount > kMaximumProvenanceTriangles)
                return false;
            char head[64];
            const int length = std::snprintf(head, sizeof head, "%d %d %d %d", entry.surfaceType,
                entry.firstTriangle, entry.triangleCount, entry.reversed ? 1 : 0);
            if (length <= 0 || std::size_t(length) >= sizeof head) return false;
            if (i != 0) text.push_back('\n');
            text.append(head, std::size_t(length));
            for (double value : entry.params) {
                if (!std::isfinite(value)) return false;
                text.push_back(' ');
                if (!AppendHexFloat(text, value)) return false;
            }
        }
        output = std::move(text);
        return !output.empty();
    } catch (...) { output.clear(); return false; }
}

inline bool ParseNonNegativeInt(const std::string& token, int maximum, int& output) noexcept {
    if (token.empty() || token.size() > 8) return false;
    int value = 0;
    for (char c : token) {
        if (c < '0' || c > '9') return false;
        value = value * 10 + (c - '0');
        if (value > maximum) return false;
    }
    output = value;
    return true;
}

inline bool ParseHexFloat(const std::string& token, double& output) noexcept {
    if (token.empty() || token.size() > 63) return false;
    char* end = nullptr;
    const double value = std::strtod(token.c_str(), &end);
    if (end != token.c_str() + token.size() || !std::isfinite(value)) return false;
    output = value;
    return true;
}

inline bool DecodeEntries(const std::string& text,
                          std::vector<SourceFaceProvenanceEntry>& output) noexcept {
    output.clear();
    if (text.empty()) return false;
    try {
        std::vector<SourceFaceProvenanceEntry> entries;
        std::size_t begin = 0;
        while (begin <= text.size()) {
            const std::size_t newline = text.find('\n', begin);
            const std::string line = text.substr(begin,
                newline == std::string::npos ? newline : newline - begin);
            begin = newline == std::string::npos ? text.size() + 1 : newline + 1;
            if (line.empty() || entries.size() >= std::size_t(kMaximumProvenanceFaces)) return false;
            std::vector<std::string> tokens;
            std::size_t field = 0;
            while (field <= line.size()) {
                const std::size_t space = line.find(' ', field);
                tokens.push_back(line.substr(field, space == std::string::npos ? space : space - field));
                field = space == std::string::npos ? line.size() + 1 : space + 1;
            }
            if (tokens.size() < 4) return false;
            SourceFaceProvenanceEntry entry;
            int surfaceType = -1, reversed = -1;
            if (!ParseNonNegativeInt(tokens[0], kSurfaceTypeOther, surfaceType)
                || !ParseNonNegativeInt(tokens[1], kMaximumProvenanceTriangles, entry.firstTriangle)
                || !ParseNonNegativeInt(tokens[2], kMaximumProvenanceTriangles, entry.triangleCount)
                || entry.triangleCount <= 0
                || !ParseNonNegativeInt(tokens[3], 1, reversed)) return false;
            entry.surfaceType = surfaceType;
            entry.reversed = reversed == 1;
            const int parameters = ParameterCountForSurfaceType(entry.surfaceType);
            if (parameters < 0 || tokens.size() != std::size_t(4 + parameters)) return false;
            entry.params.reserve(std::size_t(parameters));
            for (int p = 0; p < parameters; ++p) {
                double value = 0;
                if (!ParseHexFloat(tokens[std::size_t(4 + p)], value)) return false;
                entry.params.push_back(value);
            }
            entries.push_back(std::move(entry));
        }
        output = std::move(entries);
        return !output.empty();
    } catch (...) { output.clear(); return false; }
}

// Payload chunk characters: %a hex-float text plus the field/line separators.
inline bool IsProvenancePayloadCharacter(char c) noexcept {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || c == 'x' || c == 'p'
        || c == '+' || c == '-' || c == '.' || c == ' ' || c == '\n';
}

// sha256 over the copy's triangulation in emission order: per triangle its
// three corner indices as offsets relative to the triangle's first corner
// (decimal text), then each referenced corner node's xyz as exact %a
// hex-float text, in stored corner order. Absolute node placement is not
// hashed, so an order-preserving atlas rebuild does not change the digest;
// geometry, winding, order, or count changes do. Recomputed on every read.
inline bool CopyTriangulationDigest(const Handle(Poly_Triangulation)& mesh,
                                    std::string& output) noexcept {
    output.clear();
    try {
        if (mesh.IsNull() || mesh->HasDeferredData() || !mesh->HasGeometry()
            || mesh->NbTriangles() <= 0 || mesh->NbNodes() <= 0) return false;
        std::string stream;
        stream.reserve(std::size_t(mesh->NbTriangles()) * 3 * 96);
        char buffer[96];
        for (int t = 1; t <= mesh->NbTriangles(); ++t) {
            int ids[3];
            mesh->Triangle(t).Get(ids[0], ids[1], ids[2]);
            for (int id : ids) if (id < 1 || id > mesh->NbNodes()) return false;
            int length = std::snprintf(buffer, sizeof buffer, "%d,%d,%d:",
                0, ids[1] - ids[0], ids[2] - ids[0]);
            if (length <= 0 || std::size_t(length) >= sizeof buffer) return false;
            stream.append(buffer, std::size_t(length));
            for (int id : ids) {
                const auto& point = mesh->Node(id);
                if (!std::isfinite(point.X()) || !std::isfinite(point.Y()) || !std::isfinite(point.Z()))
                    return false;
                length = std::snprintf(buffer, sizeof buffer, "%a,%a,%a;", point.X(), point.Y(), point.Z());
                if (length <= 0 || std::size_t(length) >= sizeof buffer) return false;
                stream.append(buffer, std::size_t(length));
            }
        }
        if (stream.empty() || stream.size() > 64U * 1024U * 1024U) return false;
        std::uint8_t hash[CC_SHA256_DIGEST_LENGTH];
        if (CC_SHA256(stream.data(), static_cast<CC_LONG>(stream.size()), hash) == nullptr) return false;
        static constexpr char digits[] = "0123456789abcdef";
        output.reserve(2 * CC_SHA256_DIGEST_LENGTH);
        for (std::uint8_t byte : hash) {
            output.push_back(digits[byte >> 4]);
            output.push_back(digits[byte & 15]);
        }
        return output.size() == 2 * CC_SHA256_DIGEST_LENGTH;
    } catch (...) { output.clear(); return false; }
}
} // namespace core3d::provenance
