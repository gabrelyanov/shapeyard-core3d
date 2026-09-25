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

namespace core3d::variable_radius_fillet {

using UUID = retained_recipe::UUID;
using Digest = retained_recipe::Digest;

inline constexpr std::uint32_t FeatureKind = 0x00002003;
inline constexpr std::uint32_t CodecVersion = 1;
inline constexpr std::size_t MaximumEdges = 16;
inline constexpr std::size_t MaximumPayloadBytes = 4096;

enum class LawKind : std::uint8_t { LinearStartEnd = 1 };
enum class ParameterConvention : std::uint8_t { OrientedNormalizedArcLength = 1 };
enum class CurveKind : std::uint8_t { Line = 1 };
enum class Orientation : std::uint8_t { Forward = 1, Reversed = 2 };

struct Station final {
    UUID identifier{};
    double parameter = 0;
    double radiusLocal = 0;
    bool operator==(const Station&) const noexcept = default;
};

// This is a complete replay anchor, not a transient topology index. pointLocal
// is an interior point; tangent and the two ordered outward normals establish
// the geometric edge and its direction. startLocal/endLocal establish the
// persisted t=0/t=1 convention independently of TopoDS orientation.
struct OrientedEdgeAnchor final {
    UUID identifier{};
    CurveKind curve = CurveKind::Line;
    Orientation orientation = Orientation::Forward;
    std::array<double, 3> pointLocal{}, tangent{}, normalA{}, normalB{};
    std::array<double, 3> startLocal{}, endLocal{};
    Digest selectorProof{};
    bool operator==(const OrientedEdgeAnchor&) const noexcept = default;
};

struct Definition final {
    std::uint32_t schema = CodecVersion;
    LawKind law = LawKind::LinearStartEnd;
    ParameterConvention convention = ParameterConvention::OrientedNormalizedArcLength;
    double metersPerLocalUnit = 0;
    std::array<Station, 2> stations{};
    std::vector<OrientedEdgeAnchor> edges;
    bool operator==(const Definition&) const noexcept = default;
};

enum class Refusal : std::uint8_t {
    None = 0, MalformedPayload, UnsupportedVersion, UnsupportedLaw,
    InvalidUnit, InvalidRadius, InvalidStationIdentity, InvalidEdgeIdentity,
    ClosedLoop, UnsupportedEdge, OrientationDrift, AnchorMissing,
    AnchorAmbiguous, Clearance, ContourExpansion, EndpointMismatch,
    SectionMismatch, SelfIntersection, NonRemoving, KernelFailure,
    ReplayMismatch, Budget, Cancelled
};

inline const char* Reason(Refusal value) noexcept {
    switch (value) {
    case Refusal::None: return "b3.none";
    case Refusal::MalformedPayload: return "b3.malformed-payload";
    case Refusal::UnsupportedVersion: return "b3.unsupported-version";
    case Refusal::UnsupportedLaw: return "b3.unsupported-law";
    case Refusal::InvalidUnit: return "b3.invalid-unit";
    case Refusal::InvalidRadius: return "b3.invalid-radius";
    case Refusal::InvalidStationIdentity: return "b3.invalid-station-identity";
    case Refusal::InvalidEdgeIdentity: return "b3.invalid-edge-identity";
    case Refusal::ClosedLoop: return "b3.closed-loop-seam-rule-required";
    case Refusal::UnsupportedEdge: return "b3.unsupported-edge";
    case Refusal::OrientationDrift: return "b3.orientation-drift";
    case Refusal::AnchorMissing: return "b3.anchor-missing";
    case Refusal::AnchorAmbiguous: return "b3.anchor-ambiguous";
    case Refusal::Clearance: return "b3.local-clearance";
    case Refusal::ContourExpansion: return "b3.contour-expansion";
    case Refusal::EndpointMismatch: return "b3.endpoint-mismatch";
    case Refusal::SectionMismatch: return "b3.measured-section";
    case Refusal::SelfIntersection: return "b3.self-intersection";
    case Refusal::NonRemoving: return "b3.non-removing";
    case Refusal::KernelFailure: return "b3.kernel-failure";
    case Refusal::ReplayMismatch: return "b3.replay-mismatch";
    case Refusal::Budget: return "b3.budget";
    case Refusal::Cancelled: return "b3.cancelled";
    }
    return "b3.unknown";
}

namespace detail {
inline bool finiteVector(const std::array<double, 3>& value) noexcept {
    return std::all_of(value.begin(), value.end(), [](double x) {
        return std::isfinite(x) && std::abs(x) <= 1e9;
    });
}
inline double dot(const std::array<double, 3>& a, const std::array<double, 3>& b) noexcept {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline std::array<double, 3> delta(const std::array<double, 3>& a,
                                   const std::array<double, 3>& b) noexcept {
    return {{b[0] - a[0], b[1] - a[1], b[2] - a[2]}};
}
inline bool unit(const std::array<double, 3>& value) noexcept {
    return finiteVector(value) && std::abs(dot(value, value) - 1.0) <= 1e-12;
}
inline bool exact(double value, double expected) noexcept {
    std::uint64_t a = 0, b = 0;
    std::memcpy(&a, &value, sizeof(a)); std::memcpy(&b, &expected, sizeof(b));
    return a == b;
}

struct Writer final {
    std::vector<std::uint8_t> bytes;
    bool valid = true;
    void raw(const void* address, std::size_t count) {
        if (!valid || count > MaximumPayloadBytes - bytes.size()) { valid = false; return; }
        const auto* first = static_cast<const std::uint8_t*>(address);
        bytes.insert(bytes.end(), first, first + count);
    }
    template<std::size_t N> void raw(const std::array<std::uint8_t, N>& value) {
        raw(value.data(), N);
    }
    void integer(std::uint64_t value, unsigned width) {
        std::uint8_t encoded[8]{};
        for (unsigned index = 0; index < width; ++index)
            encoded[index] = std::uint8_t(value >> (8 * index));
        raw(encoded, width);
    }
    void scalar(double value) {
        if (!std::isfinite(value)) { valid = false; return; }
        std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits)); integer(bits, 8);
    }
};

struct Reader final {
    const std::vector<std::uint8_t>& bytes;
    std::size_t position = 0;
    bool valid = true;
    bool raw(void* address, std::size_t count) {
        if (!valid || count > bytes.size() - position) { valid = false; return false; }
        std::memcpy(address, bytes.data() + position, count); position += count; return true;
    }
    template<std::size_t N> bool raw(std::array<std::uint8_t, N>& value) {
        return raw(value.data(), N);
    }
    bool integer(unsigned width, std::uint64_t& value) {
        value = 0;
        if (!valid || width > bytes.size() - position) { valid = false; return false; }
        for (unsigned index = 0; index < width; ++index)
            value |= std::uint64_t(bytes[position++]) << (8 * index);
        return true;
    }
    bool scalar(double& value) {
        std::uint64_t bits = 0;
        if (!integer(8, bits)) return false;
        std::memcpy(&value, &bits, sizeof(value)); return std::isfinite(value);
    }
    bool complete() const noexcept { return valid && position == bytes.size(); }
};
} // namespace detail

inline bool RadiusAt(const Definition& definition, double parameter,
                     double& radiusLocal) noexcept {
    radiusLocal = 0;
    if (!std::isfinite(parameter) || parameter < 0 || parameter > 1) return false;
    const double start = definition.stations[0].radiusLocal;
    const double end = definition.stations[1].radiusLocal;
    radiusLocal = start + (end - start) * parameter;
    return std::isfinite(radiusLocal) && radiusLocal > 0;
}

inline bool Validate(const Definition& value, Refusal& refusal) noexcept {
    refusal = Refusal::MalformedPayload;
    try {
        if (value.schema != CodecVersion) { refusal = Refusal::UnsupportedVersion; return false; }
        if (value.law != LawKind::LinearStartEnd
            || value.convention != ParameterConvention::OrientedNormalizedArcLength) {
            refusal = Refusal::UnsupportedLaw; return false;
        }
        if (!std::isfinite(value.metersPerLocalUnit) || value.metersPerLocalUnit <= 0
            || value.metersPerLocalUnit > 1) { refusal = Refusal::InvalidUnit; return false; }
        if (!retained_recipe::Nonzero(value.stations[0].identifier)
            || !retained_recipe::Nonzero(value.stations[1].identifier)
            || value.stations[0].identifier == value.stations[1].identifier
            || !detail::exact(value.stations[0].parameter, 0.0)
            || !detail::exact(value.stations[1].parameter, 1.0)) {
            refusal = Refusal::InvalidStationIdentity; return false;
        }
        for (const auto& station : value.stations) {
            const double radiusMM = station.radiusLocal * value.metersPerLocalUnit * 1000.0;
            if (!std::isfinite(station.radiusLocal) || station.radiusLocal <= 0
                || !std::isfinite(radiusMM) || radiusMM < 0.001 || radiusMM > 1e6) {
                refusal = Refusal::InvalidRadius; return false;
            }
        }
        double midpoint = 0;
        if (!RadiusAt(value, 0.5, midpoint)) { refusal = Refusal::InvalidRadius; return false; }
        if (value.edges.empty() || value.edges.size() > MaximumEdges) {
            refusal = Refusal::Budget; return false;
        }
        std::set<UUID> identifiers;
        for (const auto& edge : value.edges) {
            if (!retained_recipe::Nonzero(edge.identifier)
                || !identifiers.insert(edge.identifier).second
                || !retained_recipe::Nonzero(edge.selectorProof)) {
                refusal = Refusal::InvalidEdgeIdentity; return false;
            }
            if (edge.curve != CurveKind::Line) { refusal = Refusal::ClosedLoop; return false; }
            if ((edge.orientation != Orientation::Forward
                 && edge.orientation != Orientation::Reversed)
                || !detail::finiteVector(edge.pointLocal)
                || !detail::unit(edge.tangent) || !detail::unit(edge.normalA)
                || !detail::unit(edge.normalB) || !detail::finiteVector(edge.startLocal)
                || !detail::finiteVector(edge.endLocal)) {
                refusal = Refusal::InvalidEdgeIdentity; return false;
            }
            const auto direction = detail::delta(edge.startLocal, edge.endLocal);
            const double length = std::sqrt(detail::dot(direction, direction));
            if (!std::isfinite(length) || length <= 0) { refusal = Refusal::ClosedLoop; return false; }
            const double signedAlignment = detail::dot(direction, edge.tangent) / length;
            const double expected = edge.orientation == Orientation::Forward ? 1.0 : -1.0;
            if (std::abs(signedAlignment - expected) > 1e-10
                || std::abs(detail::dot(edge.normalA, edge.normalB)) > 1.0 - 1e-8) {
                refusal = Refusal::OrientationDrift; return false;
            }
            const auto fromStart = detail::delta(edge.startLocal, edge.pointLocal);
            const double along = detail::dot(fromStart, direction) / (length * length);
            const std::array<double, 3> projected{{
                edge.startLocal[0] + direction[0] * along,
                edge.startLocal[1] + direction[1] * along,
                edge.startLocal[2] + direction[2] * along}};
            const auto offset = detail::delta(projected, edge.pointLocal);
            if (along <= 0 || along >= 1 || std::sqrt(detail::dot(offset, offset)) > 1e-9) {
                refusal = Refusal::InvalidEdgeIdentity; return false;
            }
        }
        refusal = Refusal::None; return true;
    } catch (...) { refusal = Refusal::MalformedPayload; return false; }
}

inline bool Encode(const Definition& definition, std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    Refusal refusal;
    if (!Validate(definition, refusal)) return false;
    try {
        detail::Writer writer;
        writer.raw("SYVF", 4); writer.integer(CodecVersion, 1);
        writer.integer(std::uint8_t(definition.law), 1);
        writer.integer(std::uint8_t(definition.convention), 1); writer.integer(0, 1);
        writer.scalar(definition.metersPerLocalUnit);
        for (const auto& station : definition.stations) {
            writer.raw(station.identifier); writer.scalar(station.parameter);
            writer.scalar(station.radiusLocal);
        }
        writer.integer(definition.edges.size(), 2); writer.integer(0, 2);
        for (const auto& edge : definition.edges) {
            writer.raw(edge.identifier); writer.integer(std::uint8_t(edge.curve), 1);
            writer.integer(std::uint8_t(edge.orientation), 1); writer.integer(0, 2);
            for (double value : edge.pointLocal) writer.scalar(value);
            for (double value : edge.tangent) writer.scalar(value);
            for (double value : edge.normalA) writer.scalar(value);
            for (double value : edge.normalB) writer.scalar(value);
            for (double value : edge.startLocal) writer.scalar(value);
            for (double value : edge.endLocal) writer.scalar(value);
            writer.raw(edge.selectorProof);
        }
        if (!writer.valid || writer.bytes.size() > MaximumPayloadBytes) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Definition& output) noexcept {
    output = {};
    try {
        if (bytes.size() < 84 || bytes.size() > MaximumPayloadBytes
            || std::memcmp(bytes.data(), "SYVF", 4) != 0) return false;
        detail::Reader reader{bytes}; std::array<std::uint8_t, 4> magic{};
        std::uint64_t schema = 0, law = 0, convention = 0, reserved = 0, count = 0;
        Definition decoded;
        if (!reader.raw(magic) || !reader.integer(1, schema) || !reader.integer(1, law)
            || !reader.integer(1, convention) || !reader.integer(1, reserved) || reserved != 0
            || !reader.scalar(decoded.metersPerLocalUnit)) return false;
        decoded.schema = std::uint32_t(schema); decoded.law = LawKind(law);
        decoded.convention = ParameterConvention(convention);
        for (auto& station : decoded.stations)
            if (!reader.raw(station.identifier) || !reader.scalar(station.parameter)
                || !reader.scalar(station.radiusLocal)) return false;
        if (!reader.integer(2, count) || !reader.integer(2, reserved) || reserved != 0
            || count == 0 || count > MaximumEdges) return false;
        decoded.edges.resize(std::size_t(count));
        for (auto& edge : decoded.edges) {
            std::uint64_t curve = 0, orientation = 0;
            if (!reader.raw(edge.identifier) || !reader.integer(1, curve)
                || !reader.integer(1, orientation) || !reader.integer(2, reserved)
                || reserved != 0) return false;
            edge.curve = CurveKind(curve); edge.orientation = Orientation(orientation);
            for (double& value : edge.pointLocal) if (!reader.scalar(value)) return false;
            for (double& value : edge.tangent) if (!reader.scalar(value)) return false;
            for (double& value : edge.normalA) if (!reader.scalar(value)) return false;
            for (double& value : edge.normalB) if (!reader.scalar(value)) return false;
            for (double& value : edge.startLocal) if (!reader.scalar(value)) return false;
            for (double& value : edge.endLocal) if (!reader.scalar(value)) return false;
            if (!reader.raw(edge.selectorProof)) return false;
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
        return composite_recipe::NodeID(node) == feature.inputs[0];
    });
    if (input == prior.end()) return false;
    Definition definition; std::vector<std::uint8_t> exact;
    return Decode(feature.parameters, definition) && Encode(definition, exact)
        && exact == feature.parameters;
}

} // namespace core3d::variable_radius_fillet
