#pragma once

#include <Core3D/RetainedFeatureRegistry.hxx>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace core3d::face_push_pull {
using UUID = retained_recipe::UUID;
inline constexpr std::uint32_t FeatureKind = retained_feature::FacePushPullKind;
inline constexpr std::uint32_t CodecVersion = 1;
inline constexpr std::uint8_t SelectorVersion = 1;
inline constexpr std::size_t MaximumBoundaryEdges = 256;
inline constexpr std::size_t MaximumBoundaryWires = 32;
inline constexpr std::size_t MaximumPayloadBytes = 144;

// Geometric intent in the captured source's local working frame. No face
// ordinal, native pointer or transient triangulation is persisted. Moments
// can collide: resolution ALWAYS requires exactly one matching face.
struct FaceSelector final {
    UUID identifier{};
    std::uint8_t version = SelectorVersion;
    std::uint16_t boundaryEdges = 0, boundaryWires = 0;
    std::array<double, 3> centroidLocal{}, outwardNormalLocal{};
    double areaLocal = 0, perimeterLocal = 0;
};

struct Definition final {
    std::uint32_t schema = CodecVersion;
    // Coordinates are local; a native opening must validate the placement as
    // uniform before supplying its absolute scale. This is not a shear adapter.
    double metersPerLocalUnit = 0, uniformScale = 1;
    double signedDistanceLocal = 0, insetLocal = 0;
    FaceSelector face;
    UUID outputFaceIdentifier{};
};

enum class Refusal : std::uint8_t {
    None, Unchanged, UnsupportedVersion, InvalidParameters, InvalidSelector,
    UnsupportedSurface, InvalidSource, AnchorMissing, AnchorAmbiguous,
    InvalidInset, InvalidCandidate, DisconnectedSolid, BuildFailure,
    SourceMutation, NonDeterministic, TopologyBudgetExceeded, Cancelled
};

inline const char* Reason(Refusal value) noexcept {
    switch (value) {
    case Refusal::None: return "none";
    case Refusal::Unchanged: return "unchanged";
    case Refusal::UnsupportedVersion: return "unsupported-version";
    case Refusal::InvalidParameters: return "invalid-parameters";
    case Refusal::InvalidSelector: return "invalid-selector";
    case Refusal::UnsupportedSurface: return "unsupported-surface";
    case Refusal::InvalidSource: return "invalid-source";
    case Refusal::AnchorMissing: return "anchor-missing";
    case Refusal::AnchorAmbiguous: return "anchor-ambiguous";
    case Refusal::InvalidInset: return "invalid-inset";
    case Refusal::InvalidCandidate: return "invalid-candidate";
    case Refusal::DisconnectedSolid: return "disconnected-solid";
    case Refusal::BuildFailure: return "build-failure";
    case Refusal::SourceMutation: return "source-mutation";
    case Refusal::NonDeterministic: return "non-deterministic";
    case Refusal::TopologyBudgetExceeded: return "topology-budget-exceeded";
    case Refusal::Cancelled: return "cancelled";
    }
    return "unknown";
}

inline bool ValidSelector(const FaceSelector& face) noexcept {
    double length2 = 0;
    for (double v : face.centroidLocal) if (!std::isfinite(v)) return false;
    for (double v : face.outwardNormalLocal) {
        if (!std::isfinite(v)) return false;
        length2 += v * v;
    }
    return face.version == SelectorVersion && retained_recipe::Nonzero(face.identifier)
        && face.boundaryEdges >= 1 && face.boundaryEdges <= MaximumBoundaryEdges
        && face.boundaryWires >= 1 && face.boundaryWires <= MaximumBoundaryWires
        && face.boundaryWires <= face.boundaryEdges
        && std::abs(length2 - 1) <= 1e-12
        && std::isfinite(face.areaLocal) && face.areaLocal > 0
        && std::isfinite(face.perimeterLocal) && face.perimeterLocal > 0;
}

inline bool Validate(const Definition& value, Refusal& refusal) noexcept {
    refusal = Refusal::UnsupportedVersion;
    if (value.schema != CodecVersion || value.face.version != SelectorVersion) return false;
    refusal = Refusal::InvalidParameters;
    if (!std::isfinite(value.metersPerLocalUnit) || value.metersPerLocalUnit <= 0
        || value.metersPerLocalUnit > 1 || !std::isfinite(value.uniformScale)
        || value.uniformScale <= 0
        || !std::isfinite(1000 * value.metersPerLocalUnit * value.uniformScale)
        || 1000 * value.metersPerLocalUnit * value.uniformScale <= 0
        || !std::isfinite(value.signedDistanceLocal)
        || !std::isfinite(value.insetLocal) || value.insetLocal < 0) return false;
    refusal = Refusal::InvalidSelector;
    if (!ValidSelector(value.face) || !retained_recipe::Nonzero(value.outputFaceIdentifier)
        || value.outputFaceIdentifier == value.face.identifier) return false;
    refusal = Refusal::None; return true;
}

inline bool MillimetersToLocal(double millimeters, double metersPerUnit,
                               double uniformScale, double& local) noexcept {
    local = 0;
    if (!std::isfinite(millimeters) || !std::isfinite(metersPerUnit)
        || metersPerUnit <= 0 || metersPerUnit > 1 || !std::isfinite(uniformScale)
        || uniformScale == 0) return false;
    const double denominator = 1000 * metersPerUnit * std::abs(uniformScale);
    if (!std::isfinite(denominator) || denominator <= 0) return false;
    const double converted = millimeters / denominator;
    if (!std::isfinite(converted)) return false;
    local = converted == 0 ? 0 : converted; return true;
}

namespace codec_detail {
inline void integer(std::vector<std::uint8_t>& bytes, std::uint64_t value, unsigned width) {
    for (unsigned i = 0; i < width; ++i) bytes.push_back(std::uint8_t(value >> (8 * i)));
}
inline void scalar(std::vector<std::uint8_t>& bytes, double value) {
    if (value == 0) value = 0; // one canonical zero encoding
    std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits)); integer(bytes, bits, 8);
}
struct Reader final {
    const std::vector<std::uint8_t>& bytes; std::size_t offset = 8;
    std::uint64_t integer(unsigned width) {
        std::uint64_t value = 0;
        for (unsigned i = 0; i < width; ++i) value |= std::uint64_t(bytes[offset++]) << (8 * i);
        return value;
    }
    double scalar() {
        const std::uint64_t bits = integer(8); double value = 0;
        std::memcpy(&value, &bits, sizeof(value)); return value;
    }
    UUID uuid() { UUID value{}; for (auto& v : value) v = bytes[offset++]; return value; }
};
} // namespace codec_detail

inline bool Encode(const Definition& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear(); Refusal refusal;
    if (!Validate(value, refusal)) return false;
    try {
        std::vector<std::uint8_t> bytes{'S','Y','P','P',CodecVersion,SelectorVersion,0,0};
        for (double v : {value.metersPerLocalUnit, value.uniformScale,
                         value.signedDistanceLocal, value.insetLocal}) codec_detail::scalar(bytes, v);
        bytes.insert(bytes.end(), value.face.identifier.begin(), value.face.identifier.end());
        bytes.insert(bytes.end(), value.outputFaceIdentifier.begin(), value.outputFaceIdentifier.end());
        codec_detail::integer(bytes, value.face.boundaryEdges, 2);
        codec_detail::integer(bytes, value.face.boundaryWires, 2);
        codec_detail::integer(bytes, 0, 4);
        for (double v : value.face.centroidLocal) codec_detail::scalar(bytes, v);
        for (double v : value.face.outwardNormalLocal) codec_detail::scalar(bytes, v);
        codec_detail::scalar(bytes, value.face.areaLocal);
        codec_detail::scalar(bytes, value.face.perimeterLocal);
        if (bytes.size() != MaximumPayloadBytes) return false;
        output = std::move(bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Definition& output) noexcept {
    output = {};
    try {
        if (bytes.size() != MaximumPayloadBytes || std::memcmp(bytes.data(), "SYPP", 4) != 0
            || bytes[4] != CodecVersion || bytes[5] != SelectorVersion || bytes[6] || bytes[7]) return false;
        codec_detail::Reader r{bytes}; Definition value;
        value.metersPerLocalUnit = r.scalar(); value.uniformScale = r.scalar();
        value.signedDistanceLocal = r.scalar(); value.insetLocal = r.scalar();
        value.face.identifier = r.uuid(); value.outputFaceIdentifier = r.uuid();
        value.face.boundaryEdges = std::uint16_t(r.integer(2));
        value.face.boundaryWires = std::uint16_t(r.integer(2));
        if (r.integer(4) != 0) return false;
        for (double& v : value.face.centroidLocal) v = r.scalar();
        for (double& v : value.face.outwardNormalLocal) v = r.scalar();
        value.face.areaLocal = r.scalar(); value.face.perimeterLocal = r.scalar();
        std::vector<std::uint8_t> canonical;
        if (r.offset != bytes.size() || !Encode(value, canonical) || canonical != bytes) return false;
        output = value; return true;
    } catch (...) { return false; }
}

inline bool CanonicalPayload(const composite_recipe::FeatureNode& feature,
                              const std::vector<composite_recipe::Node>& prior) noexcept {
    if (feature.kind != FeatureKind || feature.codecVersion != CodecVersion || feature.inputs.size() != 1)
        return false;
    std::size_t matches = 0;
    for (const auto& node : prior) if (composite_recipe::NodeID(node) == feature.inputs[0]) ++matches;
    Definition value;
    return matches == 1 && Decode(feature.parameters, value);
}
} // namespace core3d::face_push_pull
