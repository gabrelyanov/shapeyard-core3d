#pragma once
#include "SpatialSweepDefinition.hxx"
#include <CommonCrypto/CommonDigest.h>
#include <algorithm>
#include <cstring>
#include <limits>

namespace core3d::spatial_sweep {
class PayloadWriter {
public:
    std::vector<std::uint8_t> bytes;
    bool valid = true;
    void raw(const std::uint8_t* value, std::size_t count) {
        if (!valid || count > MaximumPayloadBytes - bytes.size()) { valid = false; return; }
        bytes.insert(bytes.end(), value, value + count);
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
        std::uint64_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        integer(bits, 8);
    }
};

class PayloadReader {
public:
    PayloadReader(const std::vector<std::uint8_t>& value, std::size_t limit)
        : bytes_(value), limit_(limit) {}
    bool raw(std::uint8_t* output, std::size_t count) {
        if (!ok_ || cursor_ > limit_ || count > limit_ - cursor_) {
            ok_ = false; return false;
        }
        std::copy_n(bytes_.begin() + cursor_, count, output);
        cursor_ += count;
        return true;
    }
    template<std::size_t N> bool raw(std::array<std::uint8_t, N>& output) {
        return raw(output.data(), output.size());
    }
    bool integer(unsigned width, std::uint64_t& output) {
        output = 0;
        if (!ok_ || width == 0 || width > 8 || cursor_ > limit_
            || width > limit_ - cursor_) { ok_ = false; return false; }
        for (unsigned index = 0; index < width; ++index)
            output |= std::uint64_t(bytes_[cursor_++]) << (8 * index);
        return true;
    }
    bool scalar(double& output) {
        std::uint64_t bits = 0;
        if (!integer(8, bits)) return false;
        std::memcpy(&output, &bits, sizeof(output));
        return Finite(output);
    }
    bool complete() const noexcept { return ok_ && cursor_ == limit_; }
    std::size_t remaining() const noexcept {
        return ok_ && cursor_ <= limit_ ? limit_ - cursor_ : 0;
    }
private:
    const std::vector<std::uint8_t>& bytes_;
    std::size_t limit_ = 0;
    std::size_t cursor_ = 0;
    bool ok_ = true;
};

inline bool PayloadHash(const std::vector<std::uint8_t>& bytes, Digest& output) noexcept {
    output.fill(0);
    return !bytes.empty() && bytes.size() <= MaximumPayloadBytes
        && CC_SHA256(bytes.data(), CC_LONG(bytes.size()), output.data()) != nullptr;
}

inline bool Encode(const Definition& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (Validate(value) != Refusal::None) return false;
        PayloadWriter writer;
        writer.raw(reinterpret_cast<const std::uint8_t*>("SCSW"), 4);
        writer.integer(Schema, 1); writer.integer(0, 1); writer.integer(0, 2);
        writer.raw(value.path.inputNode); writer.raw(value.path.curveFeature);
        writer.raw(value.path.sourceRecipeDigest);
        writer.integer(value.path.ownerState.definitionRevision, 8);
        writer.integer(value.path.ownerState.nextLocalID, 8);
        writer.raw(value.path.ownerState.canonicalDefinitionDigest);
        writer.integer(value.path.ownerState.tombstones.size(), 2);
        for (const UUID& identifier : value.path.ownerState.tombstones) writer.raw(identifier);
        writer.scalar(value.dimensionMetersPerUnit);
        writer.integer(std::uint8_t(value.section), 1); writer.integer(0, 1);
        writer.raw(value.sectionIdentifier);
        for (double component : value.orientation.authoredSeed) writer.scalar(component);
        writer.scalar(value.orientation.phaseRadians);
        writer.integer(std::uint8_t(value.transport), 1);
        writer.integer(std::uint8_t(value.parameterization), 1);
        writer.integer(std::uint8_t(value.radius.kind), 1);
        writer.integer(std::uint8_t(value.twist.kind), 1);
        writer.integer(std::uint8_t(value.closure), 1);
        writer.integer(value.witness.present ? 1 : 0, 1);
        writer.integer(0, 2);
        writer.scalar(value.radius.startRadius); writer.scalar(value.radius.endRadius);
        writer.scalar(value.twist.totalRadians);
        writer.integer(std::uint32_t(value.twist.windingTurns), 4);
        writer.scalar(value.witness.unwrappedHolonomyReference);
        writer.integer(std::uint32_t(value.witness.lift), 4);
        writer.integer(value.witness.solverVersion, 4);
        writer.integer(value.witness.proofVersion, 4);
        writer.integer(value.profile.algorithm, 4);
        writer.integer(value.profile.tolerance, 4);
        writer.integer(value.profile.proof, 4);
        writer.integer(value.profile.serializer, 4);
        if (!writer.valid || writer.bytes.size() > MaximumPayloadBytes - 32) return false;
        Digest digest{};
        if (!PayloadHash(writer.bytes, digest)) return false;
        writer.raw(digest);
        if (!writer.valid) return false;
        output = std::move(writer.bytes);
        return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Definition& output) noexcept {
    output = {};
    try {
        constexpr std::size_t MinimumBytes = 284;
        if (bytes.size() < MinimumBytes || bytes.size() > MaximumPayloadBytes
            || std::memcmp(bytes.data(), "SCSW\1\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{};
        if (!PayloadHash(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin());
        if (actual != expected) return false;
        PayloadReader reader(bytes, bytes.size() - 32);
        std::array<std::uint8_t, 8> prefix{};
        Definition value;
        std::uint64_t tombstoneCount = 0, raw = 0, reserved = 0;
        if (!reader.raw(prefix) || !reader.raw(value.path.inputNode)
            || !reader.raw(value.path.curveFeature) || !reader.raw(value.path.sourceRecipeDigest)
            || !reader.integer(8, value.path.ownerState.definitionRevision)
            || !reader.integer(8, value.path.ownerState.nextLocalID)
            || !reader.raw(value.path.ownerState.canonicalDefinitionDigest)
            || !reader.integer(2, tombstoneCount)
            || tombstoneCount > bounded_curve::MaximumControlPoints
            || tombstoneCount > reader.remaining() / 16) return false;
        value.path.ownerState.tombstones.resize(std::size_t(tombstoneCount));
        for (UUID& identifier : value.path.ownerState.tombstones)
            if (!reader.raw(identifier)) return false;
        if (!reader.scalar(value.dimensionMetersPerUnit)
            || !reader.integer(1, raw)) return false;
        value.section = SectionKind(raw);
        if (!reader.integer(1, reserved) || reserved != 0
            || !reader.raw(value.sectionIdentifier)) return false;
        for (double& component : value.orientation.authoredSeed)
            if (!reader.scalar(component)) return false;
        if (!reader.scalar(value.orientation.phaseRadians)
            || !reader.integer(1, raw)) return false;
        value.transport = TransportKind(raw);
        if (!reader.integer(1, raw)) return false;
        value.parameterization = ParameterizationKind(raw);
        if (!reader.integer(1, raw)) return false;
        value.radius.kind = RadiusLawKind(raw);
        if (!reader.integer(1, raw)) return false;
        value.twist.kind = TwistLawKind(raw);
        if (!reader.integer(1, raw)) return false;
        value.closure = ClosureKind(raw);
        if (!reader.integer(1, raw) || raw > 1) return false;
        value.witness.present = raw == 1;
        if (!reader.integer(2, reserved) || reserved != 0
            || !reader.scalar(value.radius.startRadius)
            || !reader.scalar(value.radius.endRadius)
            || !reader.scalar(value.twist.totalRadians)
            || !reader.integer(4, raw)) return false;
        value.twist.windingTurns = std::int32_t(std::uint32_t(raw));
        if (!reader.scalar(value.witness.unwrappedHolonomyReference)
            || !reader.integer(4, raw)) return false;
        value.witness.lift = std::int32_t(std::uint32_t(raw));
        if (!reader.integer(4, raw)) return false;
        value.witness.solverVersion = std::uint32_t(raw);
        if (!reader.integer(4, raw)) return false;
        value.witness.proofVersion = std::uint32_t(raw);
        if (!reader.integer(4, raw)) return false;
        value.profile.algorithm = std::uint32_t(raw);
        if (!reader.integer(4, raw)) return false;
        value.profile.tolerance = std::uint32_t(raw);
        if (!reader.integer(4, raw)) return false;
        value.profile.proof = std::uint32_t(raw);
        if (!reader.integer(4, raw)) return false;
        value.profile.serializer = std::uint32_t(raw);
        std::vector<std::uint8_t> canonical;
        if (!reader.complete() || Validate(value) != Refusal::None
            || !Encode(value, canonical) || canonical != bytes) return false;
        output = std::move(value);
        return true;
    } catch (...) { output = {}; return false; }
}

// This is deliberately the only K0 carrier hook exposed before the serial G0
// integration lands. It validates the complete feature payload and exact input
// binding, but always leaves native admission disabled.
inline Refusal ValidateCarrierFeature(const std::vector<std::uint8_t>& payload,
                                      const UUID& inputNode,
                                      std::uint32_t featureKind,
                                      std::uint32_t codecVersion) noexcept {
    Definition value;
    if (featureKind != SpatialCircleSweepFeatureKind
        || codecVersion != SpatialCircleSweepFeatureCodec
        || !Decode(payload, value) || value.path.inputNode != inputNode)
        return Refusal::NonCanonicalEncoding;
    return Refusal::RuleNotInstalled;
}

// A build witness is a derived receipt kept beside the bound result. It is not
// part of SCSW/1 and may never rewrite authored recipe values during a
// BinTools normalization roundtrip.
struct BuildWitnessReceipt {
    BuildProfile profile;
    std::uint32_t binaryFormatVersion = 0;
    Digest geometryBinaryDigest{};
    Digest surfaceCertificateDigest{};
};

inline bool ValidateBuildWitness(const Definition& definition,
                                 const BuildWitnessReceipt& witness) noexcept {
    return witness.profile.algorithm == definition.profile.algorithm
        && witness.profile.tolerance == definition.profile.tolerance
        && witness.profile.proof == definition.profile.proof
        && witness.profile.serializer == definition.profile.serializer
        && witness.binaryFormatVersion != 0
        && retained_recipe::Nonzero(witness.geometryBinaryDigest)
        && retained_recipe::Nonzero(witness.surfaceCertificateDigest);
}
} // namespace core3d::spatial_sweep
