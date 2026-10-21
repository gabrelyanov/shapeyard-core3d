#pragma once
// F2-S2 native owner of the durable decimation-derivative saved form
// (CONTRACT-F2-SAVEDFORM, D1004). Header-only bounded value/identity/receipt
// logic: canonical F2DR envelope codec, domain-tagged length-delimited
// SHA-256 witnesses and the persisted record table. No OCAF or document
// access happens here; OcctDocument owns the reserved Main-label carrier and
// Core3DDecimationOpening owns the producer join and the request fence.
// Kernel face/vertex ordinals are never persistent identity. The source
// revision binds the capture's source witness bytes, which already carry the
// document/generation/model/presentation revision epoch, so an edit-then-undo
// stays stale. F2-S2b (D1057 (i)) folds a level ordinal (0...7) into the
// store key, so one source-owner set holds up to 8 derivative rows; v1 rows
// still decode as level 0 with their level-less v1 key accepted on read.

#include "ExportMeshDecimationDefinition.hxx"

#include <CommonCrypto/CommonDigest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace core3d::export_decimation {

// Export-sheet disclosure, verbatim from D833. Persisted with every v1
// derivative record; never a runtime-composed UI string.
inline constexpr char kV1Disclosure[] =
    "not reducible under v1: prismatic sources are frozen by the "
    "protected-endpoint rule; T_locked = T_source at every declared density "
    "(D829 census)";

// Whole-carrier cap, including envelope header and checksum (contract S1).
inline constexpr std::size_t kMaxStoreBytes = 65536;
inline constexpr std::size_t kMaxStoreRecords = 64;
inline constexpr std::size_t kMaxSettingsBytes = 4096;
inline constexpr std::uint32_t kMaxContributionReferences = 262144;
// S1 kernel caps (ExportMeshDecimator.hxx): 4096 triangles, 12288 corners.
inline constexpr std::uint32_t kMaxRecordTriangles = 4096;
inline constexpr std::uint32_t kMaxRecordPositions = 12288;
// F2-S2b (D1057 (i)): level ordinal range; one source-owner set holds up to
// 8 derivative rows, one per level.
inline constexpr std::uint8_t kMaxDerivativeLevel = 7;

using F2UUID = std::array<std::uint8_t, 16>;
using F2Digest = std::array<std::uint8_t, 32>;

namespace detail {

inline bool IsLowerHexDigest(const std::string& value) noexcept {
    if (value.size() != 64) return false;
    for (const char c : value) {
        const bool digit = c >= '0' && c <= '9';
        const bool lower = c >= 'a' && c <= 'f';
        if (!digit && !lower) return false;
    }
    return true;
}

inline F2Digest SHA256(const std::vector<std::uint8_t>& bytes) noexcept {
    F2Digest digest{};
    CC_SHA256(bytes.empty() ? nullptr : bytes.data(),
        static_cast<CC_LONG>(bytes.size()), digest.data());
    return digest;
}

inline bool SHA256Hex(const std::vector<std::uint8_t>& bytes, std::string& out) noexcept {
    const F2Digest digest = SHA256(bytes);
    static const char* const kHex = "0123456789abcdef";
    try {
        out.resize(64);
    } catch (...) { return false; }
    for (std::size_t i = 0; i < digest.size(); ++i) {
        out[2 * i] = kHex[(digest[i] >> 4) & 0xF];
        out[2 * i + 1] = kHex[digest[i] & 0xF];
    }
    return true;
}

inline std::uint64_t Bits(double value) noexcept {
    // Canonical signed zero: -0.0 and 0.0 encode identically.
    if (value == 0.0) value = 0.0;
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "double size");
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

struct Writer {
    std::vector<std::uint8_t> bytes;
    void U8(std::uint8_t value) { bytes.push_back(value); }
    void U32(std::uint32_t value) {
        for (unsigned i = 0; i < 4; ++i) bytes.push_back(std::uint8_t(value >> (8 * i)));
    }
    void U64(std::uint64_t value) {
        for (unsigned i = 0; i < 8; ++i) bytes.push_back(std::uint8_t(value >> (8 * i)));
    }
    void F64(double value) { U64(Bits(value)); }
    void Text(const std::string& value) {
        U32(static_cast<std::uint32_t>(value.size()));
        bytes.insert(bytes.end(), value.begin(), value.end());
    }
    void Blob(const std::vector<std::uint8_t>& value) {
        U32(static_cast<std::uint32_t>(value.size()));
        bytes.insert(bytes.end(), value.begin(), value.end());
    }
    void Raw(const void* source, std::size_t count) {
        const auto* first = static_cast<const std::uint8_t*>(source);
        bytes.insert(bytes.end(), first, first + count);
    }
    void UUID(const F2UUID& value) { Raw(value.data(), value.size()); }
    void Digest(const F2Digest& value) { Raw(value.data(), value.size()); }
};

struct Reader {
    const std::uint8_t* at;
    std::size_t left;
    bool U8(std::uint8_t& value) {
        if (left < 1) return false;
        value = at[0]; at += 1; left -= 1; return true;
    }
    bool U32(std::uint32_t& value) {
        if (left < 4) return false;
        value = 0;
        for (unsigned i = 0; i < 4; ++i) value |= std::uint32_t(at[i]) << (8 * i);
        at += 4; left -= 4; return true;
    }
    bool U64(std::uint64_t& value) {
        if (left < 8) return false;
        value = 0;
        for (unsigned i = 0; i < 8; ++i) value |= std::uint64_t(at[i]) << (8 * i);
        at += 8; left -= 8; return true;
    }
    // Strict canonical form: non-finite input and negative zero are refused
    // on decode rather than silently normalized into a second representation.
    bool F64(double& value) {
        std::uint64_t bits = 0;
        if (!U64(bits)) return false;
        std::memcpy(&value, &bits, sizeof(value));
        if (!std::isfinite(value)) return false;
        if (value == 0.0 && std::signbit(value)) return false;
        return true;
    }
    bool Text(std::string& value, std::size_t maximum) {
        std::uint32_t count = 0;
        if (!U32(count) || count > maximum || left < count) return false;
        value.assign(reinterpret_cast<const char*>(at), count);
        at += count; left -= count; return true;
    }
    bool Blob(std::vector<std::uint8_t>& value, std::size_t maximum) {
        std::uint32_t count = 0;
        if (!U32(count) || count > maximum || left < count) return false;
        value.assign(at, at + count);
        at += count; left -= count; return true;
    }
    bool UUID(F2UUID& value) {
        if (left < value.size()) return false;
        std::memcpy(value.data(), at, value.size());
        at += value.size(); left -= value.size(); return true;
    }
    bool Digest(F2Digest& value) {
        if (left < value.size()) return false;
        std::memcpy(value.data(), at, value.size());
        at += value.size(); left -= value.size(); return true;
    }
    bool Done() const { return left == 0; }
};

} // namespace detail

inline bool Nonzero(const F2UUID& value) noexcept {
    for (const std::uint8_t byte : value) if (byte != 0) return true;
    return false;
}

// Strict 64-lowercase-hex SHA-256 text at the DTO boundary.
inline bool DigestFromHex(const std::string& text, F2Digest& out) noexcept {
    if (!detail::IsLowerHexDigest(text)) return false;
    auto half = [](char c) -> unsigned { return c >= 'a' ? unsigned(c - 'a' + 10) : unsigned(c - '0'); };
    for (std::size_t i = 0; i < out.size(); ++i)
        out[i] = std::uint8_t((half(text[2 * i]) << 4) | half(text[2 * i + 1]));
    return true;
}

inline std::string HexFromDigest(const F2Digest& value) {
    static const char* const kHex = "0123456789abcdef";
    std::string out(64, '0');
    for (std::size_t i = 0; i < value.size(); ++i) {
        out[2 * i] = kHex[(value[i] >> 4) & 0xF];
        out[2 * i + 1] = kHex[value[i] & 0xF];
    }
    return out;
}

// Canonical RFC UUID text form (36 chars, hyphens at 8/13/18/23). Hex digits
// of either case are accepted; the native producers (retained_solid UUIDText,
// NSUUID.UUIDString, Standard_GUID) all render uppercase.
inline bool UUIDFromText(const std::string& text, F2UUID& out) noexcept {
    if (text.size() != 36) return false;
    std::size_t n = 0;
    unsigned half = 0;
    for (const char c : text) {
        if (c == '-') {
            if (n != 4 && n != 6 && n != 8 && n != 10) return false;
            if (half % 2 != 0) return false;
            continue;
        }
        if (n >= 16) return false;
        unsigned digit;
        if (c >= '0' && c <= '9') digit = unsigned(c - '0');
        else if (c >= 'a' && c <= 'f') digit = unsigned(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') digit = unsigned(c - 'A' + 10);
        else return false;
        if (half % 2 == 0) out[n] = std::uint8_t(digit << 4);
        else out[n++] |= std::uint8_t(digit);
        ++half;
    }
    return n == 16 && half == 32;
}

inline std::string UUIDText(const F2UUID& value) {
    constexpr char kHex[] = "0123456789ABCDEF";
    std::string text;
    text.reserve(36);
    for (std::size_t i = 0; i < 16; ++i) {
        if (i == 4 || i == 6 || i == 8 || i == 10) text.push_back('-');
        text.push_back(kHex[value[i] >> 4]);
        text.push_back(kHex[value[i] & 15]);
    }
    return text;
}

// H(tag, fields...) = SHA-256 over Text(tag) followed by the fields'
// canonical typed encodings.
inline F2Digest TaggedDigest(const char* tag, const detail::Writer& fields) {
    detail::Writer w;
    w.Text(tag);
    w.Raw(fields.bytes.data(), fields.bytes.size());
    return detail::SHA256(w.bytes);
}

// ownerSetKey = H(owner-set, documentUUID, U32(2), entityUUID,
// definitionUUID, U8(level)). F2-S2b (D1057 (i)) folds the level ordinal into
// the key, so one source owner set admits up to 8 derivative rows (levels
// 0...7); feature history never renames the owner. Callers writing level 0
// keep the 3-argument form.
inline F2Digest OwnerSetKey(const F2UUID& documentUUID,
                            const F2UUID& entityUUID,
                            const F2UUID& definitionUUID,
                            std::uint8_t level = 0) {
    detail::Writer w;
    w.UUID(documentUUID);
    w.U32(2);
    w.UUID(entityUUID);
    w.UUID(definitionUUID);
    w.U8(level);
    return TaggedDigest("shapeyard.export-mesh-decimation.v1/owner-set", w);
}

// The level-less v1 key, H(owner-set, documentUUID, U32(1), entityUUID,
// definitionUUID). Read compatibility only: a v1 store row keeps this key
// (the derivative identifier binds it); nothing re-derives it on write.
inline F2Digest OwnerSetKeyV1(const F2UUID& documentUUID,
                              const F2UUID& entityUUID,
                              const F2UUID& definitionUUID) {
    detail::Writer w;
    w.UUID(documentUUID);
    w.U32(1);
    w.UUID(entityUUID);
    w.UUID(definitionUUID);
    return TaggedDigest("shapeyard.export-mesh-decimation.v1/owner-set", w);
}

inline F2Digest DerivativeIdentifier(const F2UUID& documentUUID,
                                     const F2Digest& ownerSetKey,
                                     const F2UUID& derivativeEntityUUID,
                                     const F2UUID& derivativeDefinitionUUID,
                                     const F2UUID& derivativeFeatureUUID) {
    detail::Writer w;
    w.UUID(documentUUID);
    w.Digest(ownerSetKey);
    w.UUID(derivativeEntityUUID);
    w.UUID(derivativeDefinitionUUID);
    w.UUID(derivativeFeatureUUID);
    return TaggedDigest("shapeyard.export-mesh-decimation.v1/derivative-owner", w);
}

// Computed constant binding the schema/algorithm/normal/arithmetic policy of
// the accepted producer build; never caller-supplied.
inline const F2Digest& AlgorithmBuildSHA256() {
    static const F2Digest value = [] {
        detail::Writer w;
        w.Text(kSchemaVersion);
        w.Text(kAlgorithmID);
        w.Text("f2-normal-regen.v1");
        w.Text("f2-arithmetic.binary64-rne.no-fast-math.v1");
        return TaggedDigest("shapeyard.export-mesh-decimation.v1/algorithm-build", w);
    }();
    return value;
}

// Content witness over the capture's complete source witness bytes. Those
// bytes bind the document identifier, generation/model/presentation
// revisions, unit and frame conventions and per-owner envelopes, so an
// edit-then-undo advances the epoch and stays stale.
inline bool SourceRevisionSHA256(const std::vector<std::uint8_t>& sourceWitnessBytes,
                                 F2Digest& out) {
    try {
        if (sourceWitnessBytes.empty()) return false;
        detail::Writer w;
        w.Blob(sourceWitnessBytes);
        out = TaggedDigest("shapeyard.export-mesh-decimation.v1/source-revision", w);
        return true;
    } catch (...) { return false; }
}

// Content witness over the exact kernel input geometry in mm (r1 domain kept:
// U64 scalar count, F64 each, U64 index count, U32 each).
inline bool SourceGeometrySHA256(const std::vector<double>& positionsMM,
                                 const std::vector<std::uint32_t>& indices,
                                 F2Digest& out) {
    try {
        if (positionsMM.empty() || positionsMM.size() % 3 != 0
            || positionsMM.size() / 3 > kMaxRecordPositions
            || indices.empty() || indices.size() % 3 != 0
            || indices.size() / 3 > kMaxRecordTriangles) return false;
        detail::Writer w;
        w.U64(positionsMM.size());
        for (const double value : positionsMM) {
            if (!std::isfinite(value)) return false;
            w.F64(value);
        }
        w.U64(indices.size());
        for (const std::uint32_t index : indices) w.U32(index);
        out = TaggedDigest("shapeyard.export-mesh-decimation.v1/source-geometry", w);
        return true;
    } catch (...) { return false; }
}

// Content witness over the native face-evidence digests in emitted order.
inline bool SourceFaceEvidenceSHA256(const std::vector<F2Digest>& faceEvidenceInEmittedOrder,
                                     F2Digest& out) {
    try {
        if (faceEvidenceInEmittedOrder.empty()
            || faceEvidenceInEmittedOrder.size() > kMaxRecordTriangles) return false;
        detail::Writer w;
        w.U64(faceEvidenceInEmittedOrder.size());
        for (const auto& digest : faceEvidenceInEmittedOrder) w.Digest(digest);
        out = TaggedDigest("shapeyard.export-mesh-decimation.v1/source-face-evidence", w);
        return true;
    } catch (...) { return false; }
}

// Content witness over the emitted derivative bytes (Float32 positions and
// UInt32 indices exactly as serialized). Validates finite scalars, index
// range against the vertex count and count consistency before hashing.
inline bool OutputGeometrySHA256(const std::vector<std::uint8_t>& positionFloat32Bytes,
                                 const std::vector<std::uint8_t>& indexUInt32Bytes,
                                 F2Digest& out) {
    try {
        if (positionFloat32Bytes.empty() || positionFloat32Bytes.size() % 12 != 0
            || positionFloat32Bytes.size() / 12 > kMaxRecordPositions
            || indexUInt32Bytes.empty() || indexUInt32Bytes.size() % 12 != 0
            || indexUInt32Bytes.size() / 12 > kMaxRecordTriangles) return false;
        const std::size_t vertexCount = positionFloat32Bytes.size() / 12;
        for (std::size_t i = 0; i < positionFloat32Bytes.size(); i += 4) {
            float value = 0;
            std::memcpy(&value, positionFloat32Bytes.data() + i, 4);
            if (!std::isfinite(value)) return false;
        }
        for (std::size_t i = 0; i < indexUInt32Bytes.size(); i += 4) {
            std::uint32_t index = 0;
            for (unsigned b = 0; b < 4; ++b)
                index |= std::uint32_t(indexUInt32Bytes[i + b]) << (8 * b);
            if (index >= vertexCount) return false;
        }
        detail::Writer w;
        w.Blob(positionFloat32Bytes);
        w.Blob(indexUInt32Bytes);
        out = TaggedDigest("shapeyard.export-mesh-decimation.v1/output-geometry", w);
        return true;
    } catch (...) { return false; }
}

// Commitment over the complete emitted artifact payload: positions, flat
// normals, indices, material bytes and the normal policy identity.
inline bool OutputArtifactSHA256(const std::vector<std::uint8_t>& positionFloat32Bytes,
                                 const std::vector<std::uint8_t>& normalFloat32Bytes,
                                 const std::vector<std::uint8_t>& indexUInt32Bytes,
                                 const std::vector<std::uint8_t>& materialBytes,
                                 const std::string& normalPolicy,
                                 F2Digest& out) {
    try {
        F2Digest geometry;
        if (normalFloat32Bytes.empty() || normalFloat32Bytes.size() % 12 != 0
            || materialBytes.empty()
            || !OutputGeometrySHA256(positionFloat32Bytes, indexUInt32Bytes, geometry))
            return false;
        for (std::size_t i = 0; i < normalFloat32Bytes.size(); i += 4) {
            float value = 0;
            std::memcpy(&value, normalFloat32Bytes.data() + i, 4);
            if (!std::isfinite(value)) return false;
        }
        detail::Writer w;
        w.Blob(positionFloat32Bytes);
        w.Blob(normalFloat32Bytes);
        w.Blob(indexUInt32Bytes);
        w.Blob(materialBytes);
        w.Text(normalPolicy);
        out = TaggedDigest("shapeyard.export-mesh-decimation.v1/output-artifact", w);
        return true;
    } catch (...) { return false; }
}

// One emitted triangle's content digest: the 9 actual emitted Float32
// position bytes, corners rotated to their least cyclic rotation by raw-byte
// lexicographic order, never reflected.
inline F2Digest OutputTriangleContentSHA256(const std::uint8_t* triangle36) noexcept {
    // Cyclic rotations only (ABC, BCA, CAB); reflection is never considered.
    std::uint8_t rotations[3][36];
    for (std::size_t rotation = 0; rotation < 3; ++rotation)
        for (std::size_t corner = 0; corner < 3; ++corner)
            std::memcpy(rotations[rotation] + 12 * corner,
                triangle36 + 12 * ((corner + rotation) % 3), 12);
    std::size_t least = 0;
    for (std::size_t rotation = 1; rotation < 3; ++rotation)
        if (std::memcmp(rotations[rotation], rotations[least], 36) < 0) least = rotation;
    try {
        detail::Writer w;
        w.Blob(std::vector<std::uint8_t>(rotations[least], rotations[least] + 36));
        return TaggedDigest("shapeyard.export-mesh-decimation.v1/output-triangle", w);
    } catch (...) { return F2Digest{}; }
}

// One emitted triangle's remap group: its content digest plus the strictly
// sorted unique 32-byte source contribution token digests.
struct RemapGroup {
    F2Digest contentDigest{};
    std::vector<F2Digest> sortedUniqueTokens;
};

// Remap receipt over the producer's complete canonical receipt: source
// digests, output geometry digest, emitted count and, in actual output order,
// each triangle's content digest, contributor count and sorted unique token
// digests. Never a hash of caller-supplied strings alone.
inline bool RemapReceiptSHA256(const F2Digest& sourceArtifactSHA256,
                               const F2Digest& sourceGeometrySHA256,
                               const F2Digest& sourceFaceEvidenceSHA256,
                               const F2Digest& outputGeometrySHA256,
                               std::uint32_t emittedTriangleCount,
                               const std::vector<RemapGroup>& groupsInEmittedOrder,
                               F2Digest& out) {
    try {
        if (emittedTriangleCount < kMinimumClosedMeshTriangles
            || emittedTriangleCount > kMaxRecordTriangles
            || groupsInEmittedOrder.size() != emittedTriangleCount) return false;
        std::uint64_t references = 0;
        detail::Writer w;
        w.Digest(sourceArtifactSHA256);
        w.Digest(sourceGeometrySHA256);
        w.Digest(sourceFaceEvidenceSHA256);
        w.Digest(outputGeometrySHA256);
        w.U32(emittedTriangleCount);
        for (const auto& group : groupsInEmittedOrder) {
            if (group.sortedUniqueTokens.empty()) return false; // a lost contributor refuses
            references += group.sortedUniqueTokens.size();
            if (references > kMaxContributionReferences) return false;
            w.Digest(group.contentDigest);
            w.U32(static_cast<std::uint32_t>(group.sortedUniqueTokens.size()));
            F2Digest prior{};
            bool havePrior = false;
            for (const auto& token : group.sortedUniqueTokens) {
                if (havePrior && !(prior < token)) return false; // strictly sorted unique
                prior = token;
                havePrior = true;
                w.Digest(token);
            }
        }
        if (references < emittedTriangleCount) return false;
        out = TaggedDigest(kRemapSchemaVersion, w);
        return true;
    } catch (...) { return false; }
}

// Complete canonical settings bytes: target, input policy, quality values,
// method/profile/camera identities, unit/frame and algorithm/build policy.
// The settings digest is never part of the bytes.
struct CanonicalSettings {
    std::uint32_t targetTriangleCount = 0;
    double maximumSymmetricSurfaceDistanceMM = 0;
    double maximumSilhouettePixelDeviation = 0;
    double hardEdgeAngleDegrees = 0;
    double contactClearanceMM = 0;
    std::uint32_t measurementSamplesPerMesh = 0;
    std::uint32_t silhouetteResolution = 0;
    F2Digest silhouetteCameraSetSHA256{};
    double metersPerUnit = 0;
};

inline bool IsValid(const CanonicalSettings& value) noexcept {
    return value.targetTriangleCount >= kMinimumClosedMeshTriangles
        && value.targetTriangleCount <= kMaximumTriangleTarget
        && std::isfinite(value.maximumSymmetricSurfaceDistanceMM)
        && value.maximumSymmetricSurfaceDistanceMM > 0
        && std::isfinite(value.maximumSilhouettePixelDeviation)
        && value.maximumSilhouettePixelDeviation >= 0
        && std::isfinite(value.hardEdgeAngleDegrees)
        && value.hardEdgeAngleDegrees > 0 && value.hardEdgeAngleDegrees < 180
        && std::isfinite(value.contactClearanceMM) && value.contactClearanceMM >= 0
        && value.measurementSamplesPerMesh > 0
        && value.measurementSamplesPerMesh <= kMaximumMeasurementSamples
        && value.silhouetteResolution > 0
        && value.silhouetteResolution <= kMaximumSilhouetteResolution
        && (value.metersPerUnit == 0.001 || value.metersPerUnit == 1.0);
}

// The ratified F2 quality profile (D1004 S7): a Current producer must use
// exactly these values; generic positive numbers are insufficient.
inline bool IsRatifiedProfile(const CanonicalSettings& value) noexcept {
    return value.maximumSymmetricSurfaceDistanceMM == 0.1
        && value.maximumSilhouettePixelDeviation == 1.0
        && value.hardEdgeAngleDegrees == 45.0
        && value.contactClearanceMM == 0.0
        && value.measurementSamplesPerMesh == 65536
        && value.silhouetteResolution == 1024;
}

inline bool EncodeSettings(const CanonicalSettings& value, std::vector<std::uint8_t>& out) {
    out.clear();
    try {
        if (!IsValid(value)) return false;
        detail::Writer w;
        w.Text("shapeyard.export-mesh-decimation.v1/settings");
        w.Text(kSchemaVersion);
        w.Text(kAlgorithmID);
        w.U8(static_cast<std::uint8_t>(InputClass::PositionOnlyUntexturedStatic));
        w.U32(value.targetTriangleCount);
        w.F64(value.maximumSymmetricSurfaceDistanceMM);
        w.F64(value.maximumSilhouettePixelDeviation);
        w.F64(value.hardEdgeAngleDegrees);
        w.F64(value.contactClearanceMM);
        w.U32(value.measurementSamplesPerMesh);
        w.U32(value.silhouetteResolution);
        w.Digest(value.silhouetteCameraSetSHA256);
        w.Text("native-rh-z-up_to_gltf-rh-y-up.v1");
        w.F64(value.metersPerUnit);
        w.Text("f2-normal-regen.v1");
        w.Text("f2-arithmetic.binary64-rne.no-fast-math.v1");
        w.Text("shapeyard.f2-quality-profile.v1");
        w.Text("f2-surface-distance.v1");
        w.Text("f2-silhouette.v1");
        w.Text("f2-contact.v1");
        w.Text("f2-hard-edge.v1");
        w.Text("f2-cameras.v1");
        w.Digest(AlgorithmBuildSHA256());
        if (w.bytes.empty() || w.bytes.size() > kMaxSettingsBytes) return false;
        out = std::move(w.bytes);
        return true;
    } catch (...) { out.clear(); return false; }
}

inline bool DecodeSettings(const std::vector<std::uint8_t>& bytes, CanonicalSettings& value) {
    value = {};
    try {
        if (bytes.empty() || bytes.size() > kMaxSettingsBytes) return false;
        detail::Reader reader{bytes.data(), bytes.size()};
        std::string tag, schema, algorithm, frame, normal, arithmetic;
        std::string profile, surface, silhouette, contact, hardEdge, cameras;
        std::uint8_t inputClass = 0;
        F2Digest build{};
        if (!reader.Text(tag, 256)
            || tag != "shapeyard.export-mesh-decimation.v1/settings"
            || !reader.Text(schema, 256) || schema != kSchemaVersion
            || !reader.Text(algorithm, 256) || algorithm != kAlgorithmID
            || !reader.U8(inputClass)
            || inputClass != static_cast<std::uint8_t>(InputClass::PositionOnlyUntexturedStatic)
            || !reader.U32(value.targetTriangleCount)
            || !reader.F64(value.maximumSymmetricSurfaceDistanceMM)
            || !reader.F64(value.maximumSilhouettePixelDeviation)
            || !reader.F64(value.hardEdgeAngleDegrees)
            || !reader.F64(value.contactClearanceMM)
            || !reader.U32(value.measurementSamplesPerMesh)
            || !reader.U32(value.silhouetteResolution)
            || !reader.Digest(value.silhouetteCameraSetSHA256)
            || !reader.Text(frame, 256)
            || frame != "native-rh-z-up_to_gltf-rh-y-up.v1"
            || !reader.F64(value.metersPerUnit)
            || !reader.Text(normal, 256) || normal != "f2-normal-regen.v1"
            || !reader.Text(arithmetic, 256)
            || arithmetic != "f2-arithmetic.binary64-rne.no-fast-math.v1"
            || !reader.Text(profile, 256) || profile != "shapeyard.f2-quality-profile.v1"
            || !reader.Text(surface, 256) || surface != "f2-surface-distance.v1"
            || !reader.Text(silhouette, 256) || silhouette != "f2-silhouette.v1"
            || !reader.Text(contact, 256) || contact != "f2-contact.v1"
            || !reader.Text(hardEdge, 256) || hardEdge != "f2-hard-edge.v1"
            || !reader.Text(cameras, 256) || cameras != "f2-cameras.v1"
            || !reader.Digest(build) || build != AlgorithmBuildSHA256()
            || !reader.Done()) {
            value = {};
            return false;
        }
        return IsValid(value);
    } catch (...) { value = {}; return false; }
}

// Recomputes the canonical settings digest from bytes; refuses malformed bytes.
inline bool SettingsSHA256(const std::vector<std::uint8_t>& canonicalBytes, F2Digest& out) {
    CanonicalSettings check;
    if (!DecodeSettings(canonicalBytes, check)) return false;
    out = detail::SHA256(canonicalBytes);
    return true;
}

// Persisted derivative provenance record. Status is encoded as canonical
// Current/0 or Stale/1 text-plus-observation pairs only, so a single boolean
// carries the whole state. F2-S2b: the level ordinal (0...7) is part of the
// row identity and folds into ownerSetKey; level defaults to 0.
struct F2Record {
    F2Digest ownerSetKey{};
    std::uint8_t level = 0; // F2-S2b level ordinal; commits into ownerSetKey
    F2UUID entityUUID{};
    F2UUID definitionUUID{};
    F2UUID sourceFeatureUUID{};
    F2UUID terminalFeatureUUID{}; // sixteen zero bytes only when the family has none
    F2UUID derivativeEntityUUID{};
    F2UUID derivativeDefinitionUUID{};
    F2UUID derivativeFeatureUUID{};
    F2Digest derivativeIdentifier{};
    F2UUID publicationSourceUUID{};
    std::uint64_t documentGeneration = 0;
    std::uint64_t modelRevision = 0;
    std::uint64_t presentationRevision = 0;
    F2Digest sourceRevisionSHA256{};
    F2Digest sourceArtifactSHA256{};
    F2Digest sourceGeometrySHA256{};
    F2Digest sourceFaceEvidenceSHA256{};
    std::vector<std::uint8_t> settingsCanonicalBytes;
    F2Digest settingsSHA256{};
    F2Digest algorithmBuildSHA256{};
    F2Digest outputArtifactSHA256{};
    F2Digest outputGeometrySHA256{};
    F2Digest remapReceiptSHA256{};
    F2Digest independentMeasurementSHA256{};
    std::uint32_t sourceTriangleCount = 0;
    std::uint32_t emittedTriangleCount = 0;
    std::uint32_t totalContributionReferences = 0;
    bool stale = false; // canonical pairs: Current/0, Stale/1
};

// Cross-field record validation against the store's document binding. Cold
// decoding recomputes only what is present (owner-set key, derivative
// identity, settings digest, build constant); external commitments are
// fixed 32-byte digests by construction.
inline bool IsValid(const F2Record& value, const F2UUID& documentUUID) {
    try {
        if (value.level > kMaxDerivativeLevel) return false; // F2-S2b: 0...7 only
        if (!Nonzero(documentUUID) || !Nonzero(value.entityUUID)
            || !Nonzero(value.definitionUUID) || !Nonzero(value.sourceFeatureUUID)
            || !Nonzero(value.derivativeEntityUUID) || !Nonzero(value.derivativeDefinitionUUID)
            || !Nonzero(value.derivativeFeatureUUID) || !Nonzero(value.publicationSourceUUID))
            return false;
        if (value.derivativeEntityUUID == value.derivativeDefinitionUUID
            || value.derivativeEntityUUID == value.derivativeFeatureUUID
            || value.derivativeDefinitionUUID == value.derivativeFeatureUUID)
            return false;
        const F2UUID source[4] = {value.entityUUID, value.definitionUUID,
            value.sourceFeatureUUID, value.terminalFeatureUUID};
        for (const F2UUID& owned : source) {
            if (!Nonzero(owned)) continue; // the terminal absent sentinel participates nowhere
            if (value.derivativeEntityUUID == owned
                || value.derivativeDefinitionUUID == owned
                || value.derivativeFeatureUUID == owned) return false;
        }
        // F2-S2b: the key recomputes from the stored fields INCLUDING the
        // level; a v1 row (always level 0) keeps its level-less v1 key, which
        // the derivative identifier binds.
        const bool keyMatches = value.ownerSetKey
                == OwnerSetKey(documentUUID, value.entityUUID,
                    value.definitionUUID, value.level)
            || (value.level == 0 && value.ownerSetKey
                == OwnerSetKeyV1(documentUUID, value.entityUUID,
                    value.definitionUUID));
        if (!keyMatches
            || value.derivativeIdentifier
                != DerivativeIdentifier(documentUUID, value.ownerSetKey,
                    value.derivativeEntityUUID, value.derivativeDefinitionUUID,
                    value.derivativeFeatureUUID))
            return false;
        CanonicalSettings settings;
        F2Digest settingsDigest;
        if (value.settingsCanonicalBytes.empty()
            || value.settingsCanonicalBytes.size() > kMaxSettingsBytes
            || !DecodeSettings(value.settingsCanonicalBytes, settings)
            || !SettingsSHA256(value.settingsCanonicalBytes, settingsDigest)
            || settingsDigest != value.settingsSHA256
            || value.algorithmBuildSHA256 != AlgorithmBuildSHA256())
            return false;
        if (value.sourceTriangleCount < kMinimumClosedMeshTriangles
            || value.sourceTriangleCount > kMaxRecordTriangles
            || value.emittedTriangleCount < kMinimumClosedMeshTriangles
            || value.emittedTriangleCount > kMaxRecordTriangles
            || value.emittedTriangleCount > settings.targetTriangleCount
            || value.totalContributionReferences < value.emittedTriangleCount
            || value.totalContributionReferences > kMaxContributionReferences)
            return false;
        return true;
    } catch (...) { return false; }
}

inline bool EncodeRecord(const F2Record& value, const F2UUID& documentUUID,
                         std::vector<std::uint8_t>& out) {
    out.clear();
    try {
        if (!IsValid(value, documentUUID)) return false;
        detail::Writer w;
        w.Digest(value.ownerSetKey);
        w.U32(1); // v1 owner count
        w.U8(value.level); // F2-S2b level ordinal (v2 record layout)
        w.UUID(value.entityUUID);
        w.UUID(value.definitionUUID);
        w.UUID(value.sourceFeatureUUID);
        w.UUID(value.terminalFeatureUUID);
        w.UUID(value.derivativeEntityUUID);
        w.UUID(value.derivativeDefinitionUUID);
        w.UUID(value.derivativeFeatureUUID);
        w.Digest(value.derivativeIdentifier);
        w.UUID(value.publicationSourceUUID);
        w.U64(value.documentGeneration);
        w.U64(value.modelRevision);
        w.U64(value.presentationRevision);
        w.Digest(value.sourceRevisionSHA256);
        w.Digest(value.sourceArtifactSHA256);
        w.Digest(value.sourceGeometrySHA256);
        w.Digest(value.sourceFaceEvidenceSHA256);
        w.Text("artifact-triangle-with-native-face-evidence.v1");
        w.Blob(value.settingsCanonicalBytes);
        w.Digest(value.settingsSHA256);
        w.Digest(value.algorithmBuildSHA256);
        w.Digest(value.outputArtifactSHA256);
        w.Digest(value.outputGeometrySHA256);
        w.Digest(value.remapReceiptSHA256);
        w.Digest(value.independentMeasurementSHA256);
        w.U32(value.sourceTriangleCount);
        w.U32(value.emittedTriangleCount);
        w.U32(value.totalContributionReferences);
        w.Text(kV1Disclosure);
        w.Text(value.stale ? "Stale" : "Current");
        w.U8(value.stale ? 1 : 0);
        if (w.bytes.size() > kMaxStoreBytes) return false;
        out = std::move(w.bytes);
        return true;
    } catch (...) { out.clear(); return false; }
}

// F2-S2b: v1Layout reads the level-less v1 record layout (a v1 row is level
// 0); v2 reads the level byte after the owner-count word.
inline bool DecodeRecord(const std::vector<std::uint8_t>& bytes,
                         const F2UUID& documentUUID, F2Record& value,
                         bool v1Layout = false) {
    value = {};
    try {
        if (bytes.empty() || bytes.size() > kMaxStoreBytes) return false;
        detail::Reader reader{bytes.data(), bytes.size()};
        std::uint32_t ownerCount = 0;
        std::string evidenceClass, disclosure, status;
        std::uint8_t historical = 0;
        std::uint8_t level = 0;
        if (!reader.Digest(value.ownerSetKey)
            || !reader.U32(ownerCount) || ownerCount != 1
            || !(v1Layout || reader.U8(level))
            || !reader.UUID(value.entityUUID)
            || !reader.UUID(value.definitionUUID)
            || !reader.UUID(value.sourceFeatureUUID)
            || !reader.UUID(value.terminalFeatureUUID)
            || !reader.UUID(value.derivativeEntityUUID)
            || !reader.UUID(value.derivativeDefinitionUUID)
            || !reader.UUID(value.derivativeFeatureUUID)
            || !reader.Digest(value.derivativeIdentifier)
            || !reader.UUID(value.publicationSourceUUID)
            || !reader.U64(value.documentGeneration)
            || !reader.U64(value.modelRevision)
            || !reader.U64(value.presentationRevision)
            || !reader.Digest(value.sourceRevisionSHA256)
            || !reader.Digest(value.sourceArtifactSHA256)
            || !reader.Digest(value.sourceGeometrySHA256)
            || !reader.Digest(value.sourceFaceEvidenceSHA256)
            || !reader.Text(evidenceClass, 128)
            || evidenceClass != "artifact-triangle-with-native-face-evidence.v1"
            || !reader.Blob(value.settingsCanonicalBytes, kMaxSettingsBytes)
            || value.settingsCanonicalBytes.empty()
            || !reader.Digest(value.settingsSHA256)
            || !reader.Digest(value.algorithmBuildSHA256)
            || !reader.Digest(value.outputArtifactSHA256)
            || !reader.Digest(value.outputGeometrySHA256)
            || !reader.Digest(value.remapReceiptSHA256)
            || !reader.Digest(value.independentMeasurementSHA256)
            || !reader.U32(value.sourceTriangleCount)
            || !reader.U32(value.emittedTriangleCount)
            || !reader.U32(value.totalContributionReferences)
            || !reader.Text(disclosure, 512)
            || disclosure != kV1Disclosure
            || !reader.Text(status, 16)
            || !reader.U8(historical)
            || !reader.Done()) {
            value = {};
            return false;
        }
        // Canonical status pairs only: Current/0 or Stale/1.
        if (status == "Current" && historical == 0) value.stale = false;
        else if (status == "Stale" && historical == 1) value.stale = true;
        else { value = {}; return false; }
        value.level = v1Layout ? 0 : level;
        if (v1Layout) {
            // F2-S2b v1 read compatibility: the stored key must be the
            // level-less v1 key; the row keeps it (the derivative identifier
            // binds it) and IsValid admits the v1 formula at level 0.
            // Re-encoding writes the v2 layout, so the byte-exact re-encode
            // proof below applies only to v2 input.
            if (value.ownerSetKey != OwnerSetKeyV1(documentUUID,
                    value.entityUUID, value.definitionUUID)) {
                value = {};
                return false;
            }
            if (!IsValid(value, documentUUID)) { value = {}; return false; }
            return true;
        }
        if (!IsValid(value, documentUUID)) { value = {}; return false; }
        std::vector<std::uint8_t> exact;
        if (!EncodeRecord(value, documentUUID, exact) || exact != bytes) {
            value = {};
            return false;
        }
        return true;
    } catch (...) { value = {}; return false; }
}

// The one physical carrier: a bounded table of logical records, up to 8 per
// source-owner set (levels 0...7, F2-S2b), sorted ascending by
// (ownerSetKey, level) and unique on the pair.
struct F2Store {
    F2UUID documentUUID{};
    std::vector<F2Record> records;
};

inline bool EncodeStore(const F2Store& value, std::vector<std::uint8_t>& out) {
    out.clear();
    try {
        if (!Nonzero(value.documentUUID) || value.records.empty()
            || value.records.size() > kMaxStoreRecords) return false;
        detail::Writer payload;
        payload.UUID(value.documentUUID);
        payload.U32(static_cast<std::uint32_t>(value.records.size()));
        F2Digest prior{};
        std::uint8_t priorLevel = 0;
        bool havePrior = false;
        for (const F2Record& record : value.records) {
            // F2-S2b: strict (ownerSetKey, level) ascending, unique on the
            // pair; duplicates and non-ascending input refuse without output.
            if (havePrior && !(prior < record.ownerSetKey
                || (prior == record.ownerSetKey && priorLevel < record.level)))
                return false;
            std::vector<std::uint8_t> encoded;
            if (!EncodeRecord(record, value.documentUUID, encoded)) return false;
            payload.Blob(encoded);
            prior = record.ownerSetKey;
            priorLevel = record.level;
            havePrior = true;
        }
        detail::Writer envelope;
        envelope.Raw("F2DR", 4);
        envelope.U32(2); // F2-S2b store schema: per-level derivative rows
        envelope.U32(static_cast<std::uint32_t>(payload.bytes.size()));
        envelope.Raw(payload.bytes.data(), payload.bytes.size());
        envelope.Digest(detail::SHA256(envelope.bytes)); // header || payload
        if (envelope.bytes.size() > kMaxStoreBytes) return false;
        out = std::move(envelope.bytes);
        return true;
    } catch (...) { out.clear(); return false; }
}

// Fail-closed canonical decode: frame and checksum before any nested parse,
// future schemas, bad counts and trailing bytes rejected, and a full
// decode/validate/re-encode byte-equality round-trip before publishing.
// F2-S2b: schema 1 (v1, level-less rows) still decodes -- every row is level
// 0 and keeps its v1 key -- but ENCODE always writes v2, so a decoded v1
// store re-encodes to different bytes and the byte-exact re-encode proof
// applies only to v2 input.
inline bool DecodeStore(const std::vector<std::uint8_t>& bytes, F2Store& value) {
    value = {};
    try {
        if (bytes.size() < 12 + 32 || bytes.size() > kMaxStoreBytes) return false;
        if (std::memcmp(bytes.data(), "F2DR", 4) != 0) return false;
        detail::Reader header{bytes.data() + 4, 8};
        std::uint32_t schema = 0, payloadCount = 0;
        if (!header.U32(schema) || (schema != 1 && schema != 2)
            || !header.U32(payloadCount)) return false;
        const bool v1Layout = schema == 1;
        const std::size_t expected = 12 + std::size_t(payloadCount) + 32;
        if (payloadCount == 0 || expected != bytes.size()) return false; // no trailing bytes
        F2Digest checksum{};
        std::memcpy(checksum.data(), bytes.data() + 12 + payloadCount, 32);
        std::vector<std::uint8_t> framed(bytes.begin(), bytes.begin() + 12 + payloadCount);
        if (detail::SHA256(framed) != checksum) return false;
        detail::Reader payload{bytes.data() + 12, payloadCount};
        std::uint32_t recordCount = 0;
        if (!payload.UUID(value.documentUUID) || !Nonzero(value.documentUUID)
            || !payload.U32(recordCount)
            || recordCount < 1 || recordCount > kMaxStoreRecords) {
            value = {};
            return false;
        }
        value.records.reserve(recordCount);
        for (std::uint32_t i = 0; i < recordCount; ++i) {
            std::vector<std::uint8_t> blob;
            if (!payload.Blob(blob, kMaxStoreBytes) || blob.empty()) {
                value = {};
                return false;
            }
            F2Record record;
            if (!DecodeRecord(blob, value.documentUUID, record, v1Layout)) {
                value = {};
                return false;
            }
            value.records.push_back(std::move(record));
        }
        if (!payload.Done()) { value = {}; return false; }
        // F2-S2b: strict (ownerSetKey, level) ascending, unique on the pair.
        for (std::size_t i = 1; i < value.records.size(); ++i)
            if (!(value.records[i - 1].ownerSetKey < value.records[i].ownerSetKey
                || (value.records[i - 1].ownerSetKey == value.records[i].ownerSetKey
                    && value.records[i - 1].level < value.records[i].level))) {
                value = {};
                return false;
            }
        // The byte-exact re-encode proof applies only to v2 input: v1 rows
        // re-encode in the v2 layout by design.
        if (!v1Layout) {
            std::vector<std::uint8_t> exact;
            if (!EncodeStore(value, exact) || exact != bytes) { value = {}; return false; }
        }
        return true;
    } catch (...) { value = {}; return false; }
}

// Explicit stale validation transition: status Stale plus the historical
// observation bit, preserving every other field byte-exactly.
inline void MarkStale(F2Record& value) noexcept { value.stale = true; }

// Tri-state whole-store read used by the OcctDocument port: distinct from
// freshness, and corrupt is never reported as absence.
enum class F2StoreOccurrence : std::uint8_t { Absent, Present, Corrupt };

struct F2StoreRead {
    F2StoreOccurrence occurrence = F2StoreOccurrence::Absent;
    std::vector<std::uint8_t> bytes; // exact carrier bytes when Present
    std::string refusal;             // diagnostic when Corrupt
    bool readable() const noexcept {
        return occurrence != F2StoreOccurrence::Corrupt;
    }
};

inline bool SameStore(const F2StoreRead& first, const F2StoreRead& second) noexcept {
    return first.occurrence == second.occurrence && first.bytes == second.bytes;
}

} // namespace core3d::export_decimation
