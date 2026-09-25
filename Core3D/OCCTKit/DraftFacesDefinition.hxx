#pragma once

#include "CompositeRecipeDefinition.hxx"
#include "RetainedRecipeIdentity.hxx"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <set>
#include <vector>

namespace core3d::draft_faces {

using UUID = retained_recipe::UUID;
using Digest = retained_recipe::Digest;

inline constexpr std::uint32_t FeatureKind = 0x00002004;
inline constexpr std::uint32_t CodecVersion = 1;
inline constexpr std::uint8_t SelectorVersion = 2;
inline constexpr std::uint8_t ProofVersion = 1;
inline constexpr std::size_t MaximumFaces = 16;
inline constexpr std::size_t MaximumPayloadBytes = 4096;

enum class SurfaceKind : std::uint8_t {
    Planar = 1,
    Cylindrical = 2,
    Conical = 3
};

enum class FaceOrientation : std::uint8_t { Outward = 1 };

// A B2 semantic selector receipt. sampleOnNeutralLocal lies on both the
// selected face and neutral plane. The stable selector intent is geometric;
// no transient TopoDS ordinal is persisted.
struct FaceIntent final {
    UUID identifier{};
    SurfaceKind surface = SurfaceKind::Planar;
    FaceOrientation orientation = FaceOrientation::Outward;
    std::uint16_t expectedBoundaryEdges = 0;
    std::uint16_t expectedCardinality = 1;
    std::array<double, 3> sampleOnNeutralLocal{};
    std::array<double, 3> outwardNormalLocal{};
    Digest selectorProof{};
    bool operator==(const FaceIntent&) const noexcept = default;
};

struct Definition final {
    std::uint32_t schema = CodecVersion;
    std::uint8_t selectorVersion = SelectorVersion;
    std::uint8_t proofVersion = ProofVersion;
    double metersPerLocalUnit = 0;
    std::array<double, 3> neutralOriginLocal{};
    std::array<double, 3> neutralNormalLocal{};
    std::array<double, 3> pullDirectionLocal{};
    double signedAngleRadians = 0;
    std::vector<FaceIntent> faces;
    bool operator==(const Definition&) const noexcept = default;
};

enum class Refusal : std::uint8_t {
    None = 0,
    MalformedPayload,
    UnsupportedVersion,
    UnsupportedSelectorVersion,
    UnsupportedProofVersion,
    InvalidUnit,
    InvalidPlane,
    InvalidAngle,
    InvalidFaceIntent,
    InvertedPullDirection,
    TangentPullDirection,
    ObliquePullDirection,
    AnchorMissing,
    AnchorAmbiguous,
    UnsupportedCylindricalFace,
    UnsupportedConicalFace,
    UnsupportedSurface,
    PropagatedUnselectedFace,
    AddDoneFailure,
    BuildFailure,
    InvalidCandidate,
    DisconnectedSolid,
    SourceMutation,
    CandidateNotSeparated,
    NeutralPlaneChanged,
    SignedAngleMismatch,
    AnalyticSectionMismatch,
    ReplayMismatch,
    Cancelled
};

inline const char* Reason(Refusal value) noexcept {
    switch (value) {
    case Refusal::None: return "b4.none";
    case Refusal::MalformedPayload: return "b4.malformed-payload";
    case Refusal::UnsupportedVersion: return "b4.unsupported-version";
    case Refusal::UnsupportedSelectorVersion: return "b4.unsupported-selector-version";
    case Refusal::UnsupportedProofVersion: return "b4.unsupported-proof-version";
    case Refusal::InvalidUnit: return "b4.invalid-unit";
    case Refusal::InvalidPlane: return "b4.invalid-neutral-plane";
    case Refusal::InvalidAngle: return "b4.invalid-signed-angle";
    case Refusal::InvalidFaceIntent: return "b4.invalid-face-intent";
    case Refusal::InvertedPullDirection: return "b4.inverted-pull-direction";
    case Refusal::TangentPullDirection: return "b4.tangent-pull-direction";
    case Refusal::ObliquePullDirection: return "b4.oblique-pull-direction";
    case Refusal::AnchorMissing: return "b4.face-anchor-missing";
    case Refusal::AnchorAmbiguous: return "b4.face-anchor-ambiguous";
    case Refusal::UnsupportedCylindricalFace: return "b4.unsupported-cylindrical-face";
    case Refusal::UnsupportedConicalFace: return "b4.unsupported-conical-face";
    case Refusal::UnsupportedSurface: return "b4.unsupported-surface";
    case Refusal::PropagatedUnselectedFace: return "b4.propagated-unselected-face";
    case Refusal::AddDoneFailure: return "b4.occt-add-done-failure";
    case Refusal::BuildFailure: return "b4.occt-build-failure";
    case Refusal::InvalidCandidate: return "b4.invalid-candidate";
    case Refusal::DisconnectedSolid: return "b4.disconnected-solid";
    case Refusal::SourceMutation: return "b4.source-mutated";
    case Refusal::CandidateNotSeparated: return "b4.candidate-not-separated";
    case Refusal::NeutralPlaneChanged: return "b4.neutral-plane-changed";
    case Refusal::SignedAngleMismatch: return "b4.signed-angle-mismatch";
    case Refusal::AnalyticSectionMismatch: return "b4.analytic-section-mismatch";
    case Refusal::ReplayMismatch: return "b4.replay-mismatch";
    case Refusal::Cancelled: return "b4.cancelled";
    }
    return "b4.unknown";
}

namespace detail {
inline bool finiteVector(const std::array<double, 3>& value) noexcept {
    return std::all_of(value.begin(), value.end(), [](double component) {
        return std::isfinite(component) && std::abs(component) <= 1e9;
    });
}
inline double dot(const std::array<double, 3>& a,
                  const std::array<double, 3>& b) noexcept {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline std::array<double, 3> delta(const std::array<double, 3>& a,
                                  const std::array<double, 3>& b) noexcept {
    return {{a[0] - b[0], a[1] - b[1], a[2] - b[2]}};
}
inline bool unit(const std::array<double, 3>& value) noexcept {
    return finiteVector(value) && std::abs(dot(value, value) - 1.0) <= 1e-12;
}
inline bool zeroBits(double value) noexcept {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits == 0 || bits == 0x8000000000000000ULL;
}

struct Writer final {
    std::vector<std::uint8_t> bytes;
    bool valid = true;
    void raw(const void* address, std::size_t count) {
        if (!valid || bytes.size() > MaximumPayloadBytes
            || count > MaximumPayloadBytes - bytes.size()) { valid = false; return; }
        const auto* first = static_cast<const std::uint8_t*>(address);
        bytes.insert(bytes.end(), first, first + count);
    }
    template<std::size_t N> void raw(const std::array<std::uint8_t, N>& value) {
        raw(value.data(), N);
    }
    void integer(std::uint64_t value, unsigned width) {
        std::uint8_t encoded[8]{};
        for (unsigned index = 0; index < width; ++index)
            encoded[index] = std::uint8_t(value >> (index * 8));
        raw(encoded, width);
    }
    void scalar(double value) {
        if (!std::isfinite(value)) { valid = false; return; }
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        integer(bits, 8);
    }
};

struct Reader final {
    const std::vector<std::uint8_t>& bytes;
    std::size_t position = 0;
    bool valid = true;
    bool raw(void* address, std::size_t count) {
        if (!valid || position > bytes.size() || count > bytes.size() - position) {
            valid = false; return false;
        }
        std::memcpy(address, bytes.data() + position, count);
        position += count; return true;
    }
    template<std::size_t N> bool raw(std::array<std::uint8_t, N>& value) {
        return raw(value.data(), N);
    }
    bool integer(unsigned width, std::uint64_t& value) {
        value = 0;
        if (!valid || position > bytes.size() || width > bytes.size() - position) {
            valid = false; return false;
        }
        for (unsigned index = 0; index < width; ++index)
            value |= std::uint64_t(bytes[position++]) << (index * 8);
        return true;
    }
    bool scalar(double& value) {
        std::uint64_t bits = 0;
        if (!integer(8, bits)) return false;
        std::memcpy(&value, &bits, sizeof(value));
        return std::isfinite(value);
    }
    bool complete() const noexcept { return valid && position == bytes.size(); }
};
} // namespace detail

inline bool Validate(const Definition& value, Refusal& refusal) noexcept {
    refusal = Refusal::MalformedPayload;
    try {
        if (value.schema != CodecVersion) { refusal = Refusal::UnsupportedVersion; return false; }
        if (value.selectorVersion != SelectorVersion) {
            refusal = Refusal::UnsupportedSelectorVersion; return false;
        }
        if (value.proofVersion != ProofVersion) {
            refusal = Refusal::UnsupportedProofVersion; return false;
        }
        if (!std::isfinite(value.metersPerLocalUnit) || value.metersPerLocalUnit <= 0
            || value.metersPerLocalUnit > 1) { refusal = Refusal::InvalidUnit; return false; }
        if (!detail::finiteVector(value.neutralOriginLocal)
            || !detail::unit(value.neutralNormalLocal)
            || !detail::unit(value.pullDirectionLocal)) {
            refusal = Refusal::InvalidPlane; return false;
        }
        if (!std::isfinite(value.signedAngleRadians)
            || detail::zeroBits(value.signedAngleRadians)
            || std::abs(value.signedAngleRadians) >= 1.5707963267948966) {
            refusal = Refusal::InvalidAngle; return false;
        }
        const double pullAlignment = detail::dot(value.neutralNormalLocal,
                                                 value.pullDirectionLocal);
        if (pullAlignment < 0) { refusal = Refusal::InvertedPullDirection; return false; }
        if (std::abs(pullAlignment) <= 1e-12) {
            refusal = Refusal::TangentPullDirection; return false;
        }
        if (std::abs(pullAlignment - 1.0) > 1e-12) {
            refusal = Refusal::ObliquePullDirection; return false;
        }
        if (value.faces.empty() || value.faces.size() > MaximumFaces) {
            refusal = Refusal::InvalidFaceIntent; return false;
        }
        std::set<UUID> identifiers;
        for (const auto& face : value.faces) {
            if (!retained_recipe::Nonzero(face.identifier)
                || !identifiers.insert(face.identifier).second
                || !retained_recipe::Nonzero(face.selectorProof)
                || face.orientation != FaceOrientation::Outward
                || face.expectedCardinality != 1
                || face.expectedBoundaryEdges < 3 || face.expectedBoundaryEdges > 64
                || !detail::finiteVector(face.sampleOnNeutralLocal)
                || !detail::unit(face.outwardNormalLocal)) {
                refusal = Refusal::InvalidFaceIntent; return false;
            }
            if (face.surface != SurfaceKind::Planar
                && face.surface != SurfaceKind::Cylindrical
                && face.surface != SurfaceKind::Conical) {
                refusal = Refusal::UnsupportedSurface; return false;
            }
            const auto neutralDelta = detail::delta(face.sampleOnNeutralLocal,
                                                     value.neutralOriginLocal);
            if (std::abs(detail::dot(neutralDelta, value.neutralNormalLocal)) > 1e-10) {
                refusal = Refusal::InvalidFaceIntent; return false;
            }
        }
        refusal = Refusal::None; return true;
    } catch (...) { refusal = Refusal::MalformedPayload; return false; }
}

inline bool Encode(const Definition& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear(); Refusal refusal;
    if (!Validate(value, refusal)) return false;
    try {
        detail::Writer writer;
        writer.raw("SYDF", 4); writer.integer(CodecVersion, 1);
        writer.integer(value.selectorVersion, 1); writer.integer(value.proofVersion, 1);
        writer.integer(0, 1); writer.scalar(value.metersPerLocalUnit);
        for (double component : value.neutralOriginLocal) writer.scalar(component);
        for (double component : value.neutralNormalLocal) writer.scalar(component);
        for (double component : value.pullDirectionLocal) writer.scalar(component);
        writer.scalar(value.signedAngleRadians);
        writer.integer(value.faces.size(), 2); writer.integer(0, 2);
        for (const auto& face : value.faces) {
            writer.raw(face.identifier); writer.integer(std::uint8_t(face.surface), 1);
            writer.integer(std::uint8_t(face.orientation), 1); writer.integer(0, 2);
            writer.integer(face.expectedBoundaryEdges, 2);
            writer.integer(face.expectedCardinality, 2);
            for (double component : face.sampleOnNeutralLocal) writer.scalar(component);
            for (double component : face.outwardNormalLocal) writer.scalar(component);
            writer.raw(face.selectorProof);
        }
        if (!writer.valid || writer.bytes.size() > MaximumPayloadBytes) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Definition& output) noexcept {
    output = {};
    try {
        if (bytes.size() < 204 || bytes.size() > MaximumPayloadBytes
            || std::memcmp(bytes.data(), "SYDF", 4) != 0) return false;
        detail::Reader reader{bytes}; std::array<std::uint8_t, 4> magic{};
        std::uint64_t schema = 0, selector = 0, proof = 0, reserved = 0, count = 0;
        Definition decoded;
        if (!reader.raw(magic) || !reader.integer(1, schema)
            || !reader.integer(1, selector) || !reader.integer(1, proof)
            || !reader.integer(1, reserved) || reserved != 0
            || !reader.scalar(decoded.metersPerLocalUnit)) return false;
        decoded.schema = std::uint32_t(schema);
        decoded.selectorVersion = std::uint8_t(selector);
        decoded.proofVersion = std::uint8_t(proof);
        for (double& component : decoded.neutralOriginLocal)
            if (!reader.scalar(component)) return false;
        for (double& component : decoded.neutralNormalLocal)
            if (!reader.scalar(component)) return false;
        for (double& component : decoded.pullDirectionLocal)
            if (!reader.scalar(component)) return false;
        if (!reader.scalar(decoded.signedAngleRadians)
            || !reader.integer(2, count) || !reader.integer(2, reserved)
            || reserved != 0 || count == 0 || count > MaximumFaces) return false;
        decoded.faces.resize(std::size_t(count));
        for (auto& face : decoded.faces) {
            std::uint64_t surface = 0, orientation = 0, boundary = 0, cardinality = 0;
            if (!reader.raw(face.identifier) || !reader.integer(1, surface)
                || !reader.integer(1, orientation) || !reader.integer(2, reserved)
                || reserved != 0 || !reader.integer(2, boundary)
                || !reader.integer(2, cardinality)) return false;
            face.surface = SurfaceKind(surface); face.orientation = FaceOrientation(orientation);
            face.expectedBoundaryEdges = std::uint16_t(boundary);
            face.expectedCardinality = std::uint16_t(cardinality);
            for (double& component : face.sampleOnNeutralLocal)
                if (!reader.scalar(component)) return false;
            for (double& component : face.outwardNormalLocal)
                if (!reader.scalar(component)) return false;
            if (!reader.raw(face.selectorProof)) return false;
        }
        Refusal refusal; std::vector<std::uint8_t> exact;
        if (!reader.complete() || !Validate(decoded, refusal) || !Encode(decoded, exact)
            || exact != bytes) return false;
        output = std::move(decoded); return true;
    } catch (...) { output = {}; return false; }
}

inline bool CanonicalPayload(const composite_recipe::FeatureNode& feature,
                             const std::vector<composite_recipe::Node>& prior) noexcept {
    if (feature.kind != FeatureKind || feature.codecVersion != CodecVersion
        || feature.inputs.size() != 1) return false;
    const auto input = std::find_if(prior.begin(), prior.end(), [&](const auto& node) {
        return composite_recipe::NodeID(node) == feature.inputs.front();
    });
    if (input == prior.end()) return false;
    Definition definition; std::vector<std::uint8_t> exact;
    return Decode(feature.parameters, definition) && Encode(definition, exact)
        && exact == feature.parameters;
}

} // namespace core3d::draft_faces
