#pragma once

// Canonical C1 wire values. SYCV/1 is the reusable curve source. SYCO/1 is
// the C1-owned authority receipt; dependent features embed this exact receipt
// instead of defining another pole/tombstone ledger.
#include "BoundedCurveDefinition.hxx"
#include <CommonCrypto/CommonDigest.h>
#include <algorithm>
#include <cstring>
#include <limits>
#include <set>

namespace core3d::bounded_curve {
using Digest = retained_recipe::Digest;

inline constexpr std::size_t MaximumDefinitionBytes = 16 * 1024;
inline constexpr std::size_t MaximumDocumentAggregateBytes = 1024 * 1024;
inline constexpr std::size_t MaximumRecordsPerDocument = 256;
inline constexpr std::size_t MaximumOwnerBytes = 16 * 1024;
inline constexpr int MinimumRecordTag = 13;

struct ReadBudget {
    std::size_t definitionBytes = 0, ownerBytes = 0, records = 0;
    std::size_t limit = MaximumDocumentAggregateBytes;
    bool rejected = false;
    void reset() noexcept { definitionBytes = 0; ownerBytes = 0; records = 0; rejected = false; }
};

struct Value {
    UUID feature{};
    Definition definition;
};

struct OwnerState {
    retained_recipe::OwnerKey owner;
    UUID feature{};
    std::uint64_t definitionRevision = 0;
    std::uint64_t nextLocalID = 0;
    Digest canonicalDefinitionDigest{};
    std::vector<UUID> tombstones;
};

struct PersistedValue {
    Value value;
    OwnerState ownerState;
};

class Writer {
public:
    explicit Writer(std::size_t limit) : limit_(limit) {}
    std::vector<std::uint8_t> bytes;
    bool valid = true;
    void raw(const std::uint8_t* data, std::size_t count) {
        if (!valid || bytes.size() > limit_ || count > limit_ - bytes.size()) {
            valid = false; return;
        }
        bytes.insert(bytes.end(), data, data + count);
    }
    template<std::size_t N> void raw(const std::array<std::uint8_t, N>& value) {
        raw(value.data(), value.size());
    }
    void integer(std::uint64_t value, unsigned width) {
        if (width == 0 || width > 8) { valid = false; return; }
        std::uint8_t encoded[8]{};
        for (unsigned index = 0; index < width; ++index)
            encoded[index] = std::uint8_t(value >> (8 * index));
        raw(encoded, width);
    }
    void scalar(double value) {
        if (!Finite(value)) { valid = false; return; }
        std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits)); integer(bits, 8);
    }
private:
    std::size_t limit_;
};

class Reader {
public:
    Reader(const std::vector<std::uint8_t>& bytes, std::size_t limit)
        : bytes_(bytes), limit_(limit) {}
    bool raw(std::uint8_t* output, std::size_t count) {
        if (!ok_ || cursor_ > limit_ || count > limit_ - cursor_) {
            ok_ = false; return false;
        }
        std::copy_n(bytes_.begin() + cursor_, count, output); cursor_ += count; return true;
    }
    template<std::size_t N> bool raw(std::array<std::uint8_t, N>& output) {
        return raw(output.data(), output.size());
    }
    bool integer(unsigned width, std::uint64_t& output) {
        output = 0;
        if (!ok_ || width == 0 || width > 8 || cursor_ > limit_ || width > limit_ - cursor_) {
            ok_ = false; return false;
        }
        for (unsigned index = 0; index < width; ++index)
            output |= std::uint64_t(bytes_[cursor_++]) << (8 * index);
        return true;
    }
    bool scalar(double& output) {
        std::uint64_t bits = 0; if (!integer(8, bits)) return false;
        std::memcpy(&output, &bits, sizeof(output)); return Finite(output);
    }
    std::size_t remaining() const noexcept {
        return ok_ && cursor_ <= limit_ ? limit_ - cursor_ : 0;
    }
    bool complete() const noexcept { return ok_ && cursor_ == limit_; }
private:
    const std::vector<std::uint8_t>& bytes_;
    std::size_t limit_ = 0, cursor_ = 0;
    bool ok_ = true;
};

inline bool Hash(const std::vector<std::uint8_t>& bytes, std::size_t maximum,
                 Digest& output) noexcept {
    output.fill(0);
    return !bytes.empty() && bytes.size() <= maximum
        && bytes.size() <= std::size_t(std::numeric_limits<CC_LONG>::max())
        && CC_SHA256(bytes.data(), CC_LONG(bytes.size()), output.data()) != nullptr;
}

inline bool SameValue(const Value& a, const Value& b) noexcept {
    if (a.feature != b.feature || a.definition.schema != b.definition.schema
        || a.definition.domain != b.definition.domain
        || a.definition.frame.identifier != b.definition.frame.identifier
        || a.definition.frame.revision != b.definition.frame.revision
        || a.definition.frame.origin != b.definition.frame.origin
        || a.definition.frame.xAxis != b.definition.frame.xAxis
        || a.definition.frame.yAxis != b.definition.frame.yAxis
        || a.definition.frame.zAxis != b.definition.frame.zAxis
        || a.definition.frame.handedness != b.definition.frame.handedness
        || a.definition.degree != b.definition.degree
        || a.definition.controlPoints.size() != b.definition.controlPoints.size()
        || a.definition.knots.size() != b.definition.knots.size()
        || a.definition.weights != b.definition.weights) return false;
    for (std::size_t index = 0; index < a.definition.controlPoints.size(); ++index)
        if (a.definition.controlPoints[index].identifier != b.definition.controlPoints[index].identifier
            || a.definition.controlPoints[index].local != b.definition.controlPoints[index].local) return false;
    for (std::size_t index = 0; index < a.definition.knots.size(); ++index)
        if (a.definition.knots[index].value != b.definition.knots[index].value
            || a.definition.knots[index].multiplicity != b.definition.knots[index].multiplicity) return false;
    return true;
}

inline bool ValidateOwnerState(const OwnerState& value) noexcept {
    if (!retained_recipe::Valid(value.owner) || !Nonzero(value.feature)
        || value.definitionRevision == 0 || value.nextLocalID == 0
        || !retained_recipe::Nonzero(value.canonicalDefinitionDigest)
        || value.tombstones.size() > MaximumControlPoints) return false;
    UUID previous{}; bool first = true;
    for (const UUID& identifier : value.tombstones) {
        if (!Nonzero(identifier) || (!first && !(previous < identifier))) return false;
        previous = identifier; first = false;
    }
    return true;
}

inline bool Encode(const Value& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!Nonzero(value.feature) || Validate(value.definition) != Refusal::None) return false;
        Writer writer(MaximumDefinitionBytes);
        writer.raw(reinterpret_cast<const std::uint8_t*>("SYCV"), 4);
        writer.integer(Schema, 1); writer.integer(std::uint8_t(value.definition.domain), 1);
        writer.integer(0, 2); writer.raw(value.feature);
        const Frame& frame = value.definition.frame;
        writer.raw(frame.identifier); writer.integer(frame.revision, 8);
        for (double scalar : frame.origin) writer.scalar(scalar);
        for (double scalar : frame.xAxis) writer.scalar(scalar);
        for (double scalar : frame.yAxis) writer.scalar(scalar);
        for (double scalar : frame.zAxis) writer.scalar(scalar);
        writer.integer(std::uint8_t(frame.handedness), 1); writer.integer(0, 7);
        writer.integer(value.definition.degree, 1); writer.integer(0, 1);
        writer.integer(value.definition.controlPoints.size(), 2);
        for (const ControlPoint& point : value.definition.controlPoints) {
            writer.raw(point.identifier);
            for (double scalar : point.local) writer.scalar(scalar);
        }
        writer.integer(value.definition.knots.size(), 2); writer.integer(0, 2);
        for (const Knot& knot : value.definition.knots) {
            writer.scalar(knot.value); writer.integer(knot.multiplicity, 1); writer.integer(0, 7);
        }
        writer.integer(value.definition.weights.empty() ? 0 : 1, 1); writer.integer(0, 1);
        writer.integer(value.definition.weights.size(), 2);
        for (double weight : value.definition.weights) writer.scalar(weight);
        if (!writer.valid || writer.bytes.size() > MaximumDefinitionBytes - 32) return false;
        Digest digest{}; if (!Hash(writer.bytes, MaximumDefinitionBytes, digest)) return false;
        writer.raw(digest); if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Value& output) noexcept {
    output = {};
    try {
        constexpr std::size_t MinimumBytes = 8 + 16 + 16 + 8 + 24 + 72 + 8 + 4
            + 2 * (16 + 24) + 4 + 2 * 16 + 4 + 32;
        if (bytes.size() < MinimumBytes || bytes.size() > MaximumDefinitionBytes
            || std::memcmp(bytes.data(), "SYCV\1", 5) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{};
        if (!Hash(body, MaximumDefinitionBytes, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin()); if (actual != expected) return false;
        Reader reader(bytes, bytes.size() - 32); std::array<std::uint8_t, 4> magic{};
        std::uint64_t raw = 0, reserved = 0, pointCount = 0, knotCount = 0, weightCount = 0;
        Value value;
        if (!reader.raw(magic) || std::memcmp(magic.data(), "SYCV", 4) != 0
            || !reader.integer(1, raw) || raw != Schema
            || !reader.integer(1, raw)) return false;
        value.definition.domain = Domain(raw);
        if (!reader.integer(2, reserved) || reserved != 0 || !reader.raw(value.feature)
            || !reader.raw(value.definition.frame.identifier)
            || !reader.integer(8, value.definition.frame.revision)) return false;
        for (double& scalar : value.definition.frame.origin) if (!reader.scalar(scalar)) return false;
        for (double& scalar : value.definition.frame.xAxis) if (!reader.scalar(scalar)) return false;
        for (double& scalar : value.definition.frame.yAxis) if (!reader.scalar(scalar)) return false;
        for (double& scalar : value.definition.frame.zAxis) if (!reader.scalar(scalar)) return false;
        if (!reader.integer(1, raw)) return false;
        value.definition.frame.handedness = Handedness(raw);
        if (!reader.integer(7, reserved) || reserved != 0 || !reader.integer(1, raw)) return false;
        value.definition.degree = std::uint8_t(raw);
        if (!reader.integer(1, reserved) || reserved != 0 || !reader.integer(2, pointCount)
            || pointCount < std::uint64_t(value.definition.degree) + 1
            || pointCount > MaximumControlPoints || pointCount > reader.remaining() / 40) return false;
        value.definition.controlPoints.resize(std::size_t(pointCount));
        for (ControlPoint& point : value.definition.controlPoints) {
            if (!reader.raw(point.identifier)) return false;
            for (double& scalar : point.local) if (!reader.scalar(scalar)) return false;
        }
        if (!reader.integer(2, knotCount) || !reader.integer(2, reserved) || reserved != 0
            || knotCount < 2 || knotCount > MaximumDistinctKnotEntries
            || knotCount > reader.remaining() / 16) return false;
        value.definition.knots.resize(std::size_t(knotCount));
        for (Knot& knot : value.definition.knots) {
            if (!reader.scalar(knot.value) || !reader.integer(1, raw)) return false;
            knot.multiplicity = std::uint8_t(raw);
            if (!reader.integer(7, reserved) || reserved != 0) return false;
        }
        if (!reader.integer(1, raw) || raw > 1 || !reader.integer(1, reserved) || reserved != 0
            || !reader.integer(2, weightCount)
            || (raw == 0 && weightCount != 0) || (raw == 1 && weightCount != pointCount)
            || weightCount > reader.remaining() / 8) return false;
        value.definition.weights.resize(std::size_t(weightCount));
        for (double& weight : value.definition.weights) if (!reader.scalar(weight)) return false;
        std::vector<std::uint8_t> canonical;
        if (!reader.complete() || Validate(value.definition) != Refusal::None
            || !Encode(value, canonical) || canonical != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

inline bool EncodeOwnerState(const OwnerState& value,
                             std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!ValidateOwnerState(value)) return false;
        Writer writer(MaximumOwnerBytes);
        writer.raw(reinterpret_cast<const std::uint8_t*>("SYCO"), 4);
        writer.integer(1, 1); writer.integer(0, 3);
        writer.raw(value.owner.document); writer.raw(value.owner.entity); writer.raw(value.owner.definition);
        writer.raw(value.feature); writer.integer(value.definitionRevision, 8);
        writer.integer(value.nextLocalID, 8); writer.raw(value.canonicalDefinitionDigest);
        writer.integer(value.tombstones.size(), 2); writer.integer(0, 6);
        for (const UUID& identifier : value.tombstones) writer.raw(identifier);
        if (!writer.valid || writer.bytes.size() > MaximumOwnerBytes - 32) return false;
        Digest digest{}; if (!Hash(writer.bytes, MaximumOwnerBytes, digest)) return false;
        writer.raw(digest); if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool DecodeOwnerState(const std::vector<std::uint8_t>& bytes,
                             OwnerState& output) noexcept {
    output = {};
    try {
        constexpr std::size_t MinimumBytes = 8 + 48 + 16 + 8 + 8 + 32 + 8 + 32;
        if (bytes.size() < MinimumBytes || bytes.size() > MaximumOwnerBytes
            || std::memcmp(bytes.data(), "SYCO\1\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{};
        if (!Hash(body, MaximumOwnerBytes, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin()); if (actual != expected) return false;
        Reader reader(bytes, bytes.size() - 32); std::array<std::uint8_t, 8> prefix{};
        std::uint64_t count = 0, reserved = 0; OwnerState value;
        if (!reader.raw(prefix) || !reader.raw(value.owner.document) || !reader.raw(value.owner.entity)
            || !reader.raw(value.owner.definition) || !reader.raw(value.feature)
            || !reader.integer(8, value.definitionRevision) || !reader.integer(8, value.nextLocalID)
            || !reader.raw(value.canonicalDefinitionDigest) || !reader.integer(2, count)
            || !reader.integer(6, reserved) || reserved != 0 || count > MaximumControlPoints
            || count > reader.remaining() / 16) return false;
        value.tombstones.resize(std::size_t(count));
        for (UUID& identifier : value.tombstones) if (!reader.raw(identifier)) return false;
        std::vector<std::uint8_t> canonical;
        if (!reader.complete() || !ValidateOwnerState(value)
            || !EncodeOwnerState(value, canonical) || canonical != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

inline bool ValidatePersisted(const PersistedValue& persisted,
                              std::vector<std::uint8_t>* definitionBytes = nullptr,
                              std::vector<std::uint8_t>* ownerBytes = nullptr) noexcept {
    try {
        std::vector<std::uint8_t> encodedDefinition, encodedOwner;
        Digest digest{};
        if (!Encode(persisted.value, encodedDefinition)
            || !EncodeOwnerState(persisted.ownerState, encodedOwner)
            || persisted.value.feature != persisted.ownerState.feature
            || !Hash(encodedDefinition, MaximumDefinitionBytes, digest)
            || digest != persisted.ownerState.canonicalDefinitionDigest) return false;
        std::set<UUID> live;
        for (const ControlPoint& point : persisted.value.definition.controlPoints)
            if (!live.insert(point.identifier).second) return false;
        for (const UUID& tombstone : persisted.ownerState.tombstones)
            if (live.count(tombstone)) return false;
        const std::size_t issued = live.size() + persisted.ownerState.tombstones.size();
        if (issued >= persisted.ownerState.nextLocalID) return false;
        if (definitionBytes) *definitionBytes = std::move(encodedDefinition);
        if (ownerBytes) *ownerBytes = std::move(encodedOwner);
        return true;
    } catch (...) { return false; }
}
} // namespace core3d::bounded_curve
