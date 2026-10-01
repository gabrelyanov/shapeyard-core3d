#pragma once

#include "RetainedRecipeIdentity.hxx"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <variant>
#include <vector>

namespace core3d::retained_face_selector {
using UUID = retained_recipe::UUID;

enum class Kind : std::uint8_t {
    PlanarFaceBoundary = 1,
    Line = 2,
    CircleRim = 3,
    SplineEdge = 4,
};
enum class Axis : std::uint8_t { X = 1, Y = 2, Z = 3 };
enum class Side : std::uint8_t { Min = 1, Max = 2 };
enum class BoundaryCurve : std::uint8_t { Line = 1 };
enum class FaceUseDirection : std::uint8_t { Forward = 1, Reversed = 2 };
enum class Coverage : std::uint8_t { EntireBoundary = 1, SelectedLine = 2 };

struct PlanarFaceScope {
    Axis axis = Axis::X;
    Side side = Side::Min;
    bool operator==(const PlanarFaceScope& other) const noexcept {
        return axis == other.axis && side == other.side;
    }
};
struct PlanarFaceBoundary {
    PlanarFaceScope face;
    BoundaryCurve edgeKind = BoundaryCurve::Line;
    std::uint32_t expectedCount = 0;
    bool operator==(const PlanarFaceBoundary& other) const noexcept {
        return face == other.face && edgeKind == other.edgeKind && expectedCount == other.expectedCount;
    }
};
struct Line {
    PlanarFaceScope face;
    std::array<double, 3> locationMM{};
    std::array<double, 3> direction{};
    std::uint32_t expectedCount = 0;
    bool operator==(const Line& other) const noexcept {
        return face == other.face && locationMM == other.locationMM
            && direction == other.direction && expectedCount == other.expectedCount;
    }
};
struct CircleRim {
    std::array<double, 3> centerMM{};
    std::array<double, 3> normal{};
    double radiusMM = 0;
    std::uint32_t expectedCount = 0;
    bool operator==(const CircleRim& other) const noexcept {
        return centerMM == other.centerMM && normal == other.normal
            && radiusMM == other.radiusMM && expectedCount == other.expectedCount;
    }
};
struct ReservedSplineEdge {
    UUID sourceFeatureID{};
    UUID featureLocalEdgeKey{};
    std::uint32_t expectedCount = 0;
    bool operator==(const ReservedSplineEdge& other) const noexcept {
        return sourceFeatureID == other.sourceFeatureID
            && featureLocalEdgeKey == other.featureLocalEdgeKey
            && expectedCount == other.expectedCount;
    }
};
using SelectorIntent = std::variant<PlanarFaceBoundary, Line, CircleRim, ReservedSplineEdge>;

struct PlaneWitness {
    std::array<double, 3> outwardNormal{};
    double offsetMM = 0;
    bool operator==(const PlaneWitness& other) const noexcept {
        return outwardNormal == other.outwardNormal && offsetMM == other.offsetMM;
    }
};
struct ReceiptEntry {
    UUID anchorKey{};
    FaceUseDirection direction = FaceUseDirection::Forward;
    bool operator==(const ReceiptEntry& other) const noexcept {
        return anchorKey == other.anchorKey && direction == other.direction;
    }
};
struct SelectorReceipt {
    std::uint32_t proofVersion = 1;
    SelectorIntent intent;
    PlaneWitness face;
    Coverage coverage = Coverage::EntireBoundary;
    std::uint32_t wireCount = 0;
    std::uint32_t boundaryUseCount = 0;
    std::uint32_t boundaryUniqueEdgeCount = 0;
    std::vector<ReceiptEntry> entries;
    bool operator==(const SelectorReceipt& other) const noexcept {
        return proofVersion == other.proofVersion && intent == other.intent
            && face == other.face && coverage == other.coverage
            && wireCount == other.wireCount && boundaryUseCount == other.boundaryUseCount
            && boundaryUniqueEdgeCount == other.boundaryUniqueEdgeCount && entries == other.entries;
    }
};

inline bool IsUnit(const std::array<double, 3>& value) noexcept {
    const double squared = value[0] * value[0] + value[1] * value[1] + value[2] * value[2];
    return std::isfinite(squared) && std::abs(squared - 1.0) <= 1e-12;
}
inline bool IsPosition(const std::array<double, 3>& value) noexcept {
    return std::all_of(value.begin(), value.end(), [](double component) {
        return std::isfinite(component) && std::abs(component) <= 1'000'000.0;
    });
}
inline std::uint32_t ExpectedCount(const SelectorIntent& intent) noexcept {
    return std::visit([](const auto& value) { return value.expectedCount; }, intent);
}
inline Kind IntentKind(const SelectorIntent& intent) noexcept {
    return std::visit([](const auto& value) -> Kind {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, PlanarFaceBoundary>) return Kind::PlanarFaceBoundary;
        if constexpr (std::is_same_v<T, Line>) return Kind::Line;
        if constexpr (std::is_same_v<T, CircleRim>) return Kind::CircleRim;
        return Kind::SplineEdge;
    }, intent);
}
inline bool ValidIntent(const SelectorIntent& intent, bool allowReserved = false) noexcept {
    return std::visit([allowReserved](const auto& value) {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, PlanarFaceBoundary>) {
            return value.edgeKind == BoundaryCurve::Line && value.expectedCount >= 1 && value.expectedCount <= 64;
        } else if constexpr (std::is_same_v<T, Line>) {
            return value.expectedCount == 1 && IsPosition(value.locationMM) && IsUnit(value.direction);
        } else if constexpr (std::is_same_v<T, CircleRim>) {
            return value.expectedCount == 1 && IsPosition(value.centerMM) && IsUnit(value.normal)
                && std::isfinite(value.radiusMM) && value.radiusMM > 0 && value.radiusMM <= 1'000'000.0;
        } else {
            return allowReserved && value.expectedCount >= 1 && value.expectedCount <= 64
                && retained_recipe::Nonzero(value.sourceFeatureID)
                && retained_recipe::Nonzero(value.featureLocalEdgeKey);
        }
    }, intent);
}
inline bool ValidReceipt(const SelectorReceipt& value) noexcept {
    if (value.proofVersion != 1 || !ValidIntent(value.intent) || !IsUnit(value.face.outwardNormal)
        || !std::isfinite(value.face.offsetMM) || value.entries.empty() || value.entries.size() > 64
        || value.entries.size() != ExpectedCount(value.intent) || value.wireCount < 1
        || value.wireCount > value.boundaryUseCount || value.boundaryUseCount > 64
        || value.boundaryUniqueEdgeCount != value.boundaryUseCount) return false;
    if (std::holds_alternative<Line>(value.intent)) {
        if (value.coverage != Coverage::SelectedLine || value.entries.size() != 1) return false;
    } else if (value.coverage != Coverage::EntireBoundary
        || value.entries.size() != value.boundaryUseCount) return false;
    UUID previous{};
    bool first = true;
    for (const auto& entry : value.entries) {
        if (!retained_recipe::Nonzero(entry.anchorKey)
            || (entry.direction != FaceUseDirection::Forward && entry.direction != FaceUseDirection::Reversed)
            || (!first && !(previous < entry.anchorKey))) return false;
        previous = entry.anchorKey;
        first = false;
    }
    return true;
}

template<class Writer>
bool WriteIntent(Writer& writer, const SelectorIntent& intent) {
    writer.u(std::uint8_t(IntentKind(intent)), 1);
    std::visit([&writer](const auto& value) {
        using T = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<T, PlanarFaceBoundary>) {
            writer.u(std::uint8_t(value.face.axis), 1); writer.u(std::uint8_t(value.face.side), 1);
            writer.u(std::uint8_t(value.edgeKind), 1); writer.u(value.expectedCount, 4);
        } else if constexpr (std::is_same_v<T, Line>) {
            writer.u(std::uint8_t(value.face.axis), 1); writer.u(std::uint8_t(value.face.side), 1);
            for (double part : value.locationMM) writer.d(part);
            for (double part : value.direction) writer.d(part);
            writer.u(value.expectedCount, 4);
        } else if constexpr (std::is_same_v<T, CircleRim>) {
            for (double part : value.centerMM) writer.d(part);
            for (double part : value.normal) writer.d(part);
            writer.d(value.radiusMM); writer.u(value.expectedCount, 4);
        } else {
            writer.raw(value.sourceFeatureID); writer.raw(value.featureLocalEdgeKey);
            writer.u(value.expectedCount, 4);
        }
    }, intent);
    return writer.ok;
}

template<class Reader>
bool ReadIntent(Reader& reader, SelectorIntent& output) {
    std::uint64_t tag = 0, value = 0;
    if (!reader.u(1, tag)) return false;
    if (tag == std::uint8_t(Kind::PlanarFaceBoundary)) {
        PlanarFaceBoundary intent;
        if (!reader.u(1, value)) return false; intent.face.axis = Axis(value);
        if (!reader.u(1, value)) return false; intent.face.side = Side(value);
        if (!reader.u(1, value)) return false; intent.edgeKind = BoundaryCurve(value);
        if (!reader.u(4, value)) return false; intent.expectedCount = std::uint32_t(value); output = intent;
    } else if (tag == std::uint8_t(Kind::Line)) {
        Line intent;
        if (!reader.u(1, value)) return false; intent.face.axis = Axis(value);
        if (!reader.u(1, value)) return false; intent.face.side = Side(value);
        for (double& part : intent.locationMM) if (!reader.d(part)) return false;
        for (double& part : intent.direction) if (!reader.d(part)) return false;
        if (!reader.u(4, value)) return false; intent.expectedCount = std::uint32_t(value); output = intent;
    } else if (tag == std::uint8_t(Kind::CircleRim)) {
        CircleRim intent;
        for (double& part : intent.centerMM) if (!reader.d(part)) return false;
        for (double& part : intent.normal) if (!reader.d(part)) return false;
        if (!reader.d(intent.radiusMM) || !reader.u(4, value)) return false;
        intent.expectedCount = std::uint32_t(value); output = intent;
    } else {
        // Tag 4 is reserved and every unknown tag is malformed before publication.
        return false;
    }
    return ValidIntent(output);
}

template<class Writer>
bool WriteReceipt(Writer& writer, const SelectorReceipt& receipt) {
    if (!ValidReceipt(receipt)) return false;
    writer.u(receipt.proofVersion, 4);
    if (!WriteIntent(writer, receipt.intent)) return false;
    for (double part : receipt.face.outwardNormal) writer.d(part);
    writer.d(receipt.face.offsetMM);
    writer.u(std::uint8_t(receipt.coverage), 1);
    writer.u(receipt.wireCount, 4); writer.u(receipt.boundaryUseCount, 4);
    writer.u(receipt.boundaryUniqueEdgeCount, 4); writer.u(receipt.entries.size(), 4);
    for (const auto& entry : receipt.entries) {
        writer.raw(entry.anchorKey); writer.u(std::uint8_t(entry.direction), 1);
    }
    return writer.ok;
}

template<class Reader>
bool ReadReceipt(Reader& reader, SelectorReceipt& receipt) {
    std::uint64_t value = 0;
    if (!reader.u(4, value)) return false; receipt.proofVersion = std::uint32_t(value);
    if (!ReadIntent(reader, receipt.intent)) return false;
    for (double& part : receipt.face.outwardNormal) if (!reader.d(part)) return false;
    if (!reader.d(receipt.face.offsetMM) || !reader.u(1, value)) return false;
    receipt.coverage = Coverage(value);
    if (!reader.u(4, value)) return false; receipt.wireCount = std::uint32_t(value);
    if (!reader.u(4, value)) return false; receipt.boundaryUseCount = std::uint32_t(value);
    if (!reader.u(4, value)) return false; receipt.boundaryUniqueEdgeCount = std::uint32_t(value);
    if (!reader.u(4, value) || value == 0 || value > 64) return false;
    receipt.entries.resize(std::size_t(value));
    for (auto& entry : receipt.entries) {
        if (!reader.raw(entry.anchorKey) || !reader.u(1, value)) return false;
        entry.direction = FaceUseDirection(value);
    }
    return ValidReceipt(receipt);
}
} // namespace core3d::retained_face_selector
