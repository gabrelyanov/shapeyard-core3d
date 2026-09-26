#pragma once
// Plain-profile subtract (cut) retention codec — FIX-SPEC H1, design option C
// (r4-cloud8182 / R4 / TA / D79 / D81 / D83).
//
// Values-only codec for the dedicated SYCR/4 cut-only graph carried by the
// existing composite recipe attribute on the new result. This header owns no
// OCAF commands, holds no labels, implements no geometry workers and must not
// include CompositeRecipeCodec.hxx back into itself; the scalar-recipe wrapper
// that needs DecodeScalarRecipe lives in CompositeRecipeCodec.hxx after that
// helper. The feature key below is recognised only by the dedicated v4
// validator; it is deliberately absent from both production registries.
//
// Codec-1 frozen contracts (FIX-SPEC H1):
//  * Feature key {PlainProfileCutKind = 0x00001002, CodecVersion = 1}; graph
//    major GraphVersion = 4. SYPC magic; payload version 1.
//  * Integers little-endian at explicit widths; finite binary64 by exact bits;
//    UUIDs/digests as raw 16/32 bytes. No struct-memory writes, no locale text
//    numbers, no native-endian encoding, no normalisation of signed zero.
//  * Source recipe bytes remain the existing EncodeScalarRecipe(Profile,
//    SchemaFor(parameters), capture.recipeValues) result; the codec wrapper
//    decodes with profile::Decode, verifies the schema, re-encodes and
//    compares byte-for-byte, and requires the recipe meters-per-unit bits to
//    equal the leaf's source-units bits.
//  * Source geometry commitments use
//    receipt::GeometryDigestForPolicy(sourceLocalBinding, digest, false) at
//    staging/reopen (package B); this codec only carries and compares the
//    digest values and never recomputes them. The final result digest uses the
//    same flags-only geometry policy over the committed local root.
//  * Material/groups/placement commitments are SHA-256 over the canonical
//    substreams under the SYPC-*/1 domain prefixes below, computed from real
//    captured values including an explicit canonical absence representation —
//    never constant nonzero "proof" digests. They are evidence of original
//    metadata, not a new transfer policy; the old PBR/receipt digest helpers
//    keep their existing domains. Name state is covered by the final
//    feature/envelope hash.
#include "CompositeRecipeDefinition.hxx"
#include <CommonCrypto/CommonDigest.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <set>
#include <string>
#include <vector>

namespace core3d::plain_profile_cut {
using composite_recipe::MaximumEnvelopeBytes;
using composite_recipe::MaximumFeaturePayloadBytes;
using retained_recipe::Digest;
using retained_recipe::SourceIdentity;
using retained_recipe::UUID;

// Dedicated v4-only feature key. Rechecked unallocated on the admitted
// execution baseline (D67 reserved 0x00001001/0x00003001 and all existing
// kind/version pairs stay untouched). Never add it to a production registry.
inline constexpr std::uint32_t PlainProfileCutKind = 0x00001002;
inline constexpr std::uint32_t CodecVersion = 1;
inline constexpr std::uint32_t GraphVersion = 4;

// Closed chain grammar bounds (FIX-SPEC H1/H2): 1..7 cuts derived from the
// existing 8-source controller budget. Fits the composite 16-source/64-node/
// 16-depth bounds with at most 8 sources, 15 nodes and depth 8.
inline constexpr std::size_t MinimumCuts = 1;
inline constexpr std::size_t MaximumCuts = 7;
inline constexpr std::size_t MaximumSourcesPerChain = MaximumCuts + 1;
inline constexpr std::size_t MaximumChainNodes = 2 * MaximumCuts + 1;
inline constexpr std::size_t MaximumNameBytes = 1024;
inline constexpr std::size_t MaximumGroupMemberships = 16;
inline constexpr std::size_t VisualValueCount = 29;
// Frozen preview fuzzy value; the exact existing 1.0e-3 bits
// (BooleanOperationController.cpp:75,739-772). Compared by bits, never by
// tolerance.
inline constexpr double FuzzyValue = 1.0e-3;

// Codec-1 admits exactly these tags; any other value on the wire refuses.
enum class Operation : std::uint8_t { Subtract = 1 };
enum class Lifecycle : std::uint8_t { ConsumeCreate = 1 };
enum class Family : std::uint8_t { PolygonPrismAxialDisks = 1 };
inline constexpr std::uint8_t BuildPolicy = 1;
inline constexpr std::uint8_t PayloadVersion = 1;

// Bounded little-endian wire writer. The 16 KiB feature cap is enforced
// before any insertion; overflowing writes poison the stream.
class PayloadWriter {
public:
    std::vector<std::uint8_t> bytes;
    bool valid = true;
    void raw(const std::uint8_t* data, std::size_t count) {
        if (!valid || count > MaximumFeaturePayloadBytes - bytes.size()) { valid = false; return; }
        bytes.insert(bytes.end(), data, data + count);
    }
    template <std::size_t N> void raw(const std::array<std::uint8_t, N>& value) {
        raw(value.data(), N);
    }
    void integer(std::uint64_t value, unsigned width) {
        if (width == 0 || width > 8) { valid = false; return; }
        std::uint8_t encoded[8]{};
        for (unsigned index = 0; index < width; ++index) encoded[index] = std::uint8_t(value >> (8 * index));
        raw(encoded, width);
    }
    void scalar(double value) {
        if (!std::isfinite(value)) { valid = false; return; }
        std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits)); integer(bits, 8);
    }
};

// Bounded wire reader over a caller-owned byte range. Every length-prefix is
// checked against the remaining bytes before any allocation by the caller.
class PayloadReader {
public:
    PayloadReader(const std::uint8_t* data, std::size_t limit) : data_(data), limit_(limit) {}
    bool raw(std::uint8_t* output, std::size_t count) {
        if (!ok_ || count > limit_ - cursor_) { ok_ = false; return false; }
        std::copy_n(data_ + cursor_, count, output); cursor_ += count; return true;
    }
    template <std::size_t N> bool raw(std::array<std::uint8_t, N>& output) {
        return raw(output.data(), N);
    }
    bool integer(unsigned width, std::uint64_t& value) {
        value = 0; if (!ok_ || width == 0 || width > 8 || width > limit_ - cursor_) { ok_ = false; return false; }
        for (unsigned index = 0; index < width; ++index) value |= std::uint64_t(data_[cursor_++]) << (8 * index);
        return true;
    }
    bool scalar(double& value) {
        std::uint64_t bits = 0; if (!integer(8, bits)) return false;
        std::memcpy(&value, &bits, sizeof(value)); return std::isfinite(value);
    }
    std::size_t remaining() const noexcept { return ok_ ? limit_ - cursor_ : 0; }
    bool complete() const noexcept { return ok_ && cursor_ == limit_; }
private:
    const std::uint8_t* data_ = nullptr;
    std::size_t limit_ = 0, cursor_ = 0;
    bool ok_ = true;
};

// Plain SHA-256 of the exact recipe bytes; identical to the existing
// composite recipe commitment rule.
inline bool HashBytes(const std::vector<std::uint8_t>& bytes, Digest& output) noexcept {
    output.fill(0);
    return !bytes.empty() && bytes.size() <= MaximumEnvelopeBytes
        && CC_SHA256(bytes.data(), CC_LONG(bytes.size()), output.data()) != nullptr;
}

// SHA-256 over a versioned domain prefix plus a canonical substream.
inline bool DomainHash(const char* prefix, const std::vector<std::uint8_t>& body,
                       Digest& output) noexcept {
    output.fill(0);
    try {
        const std::size_t prefixBytes = std::strlen(prefix);
        if (prefixBytes == 0 || body.empty() || body.size() > MaximumFeaturePayloadBytes) return false;
        std::vector<std::uint8_t> joined;
        joined.reserve(prefixBytes + body.size());
        joined.insert(joined.end(), reinterpret_cast<const std::uint8_t*>(prefix),
                      reinterpret_cast<const std::uint8_t*>(prefix) + prefixBytes);
        joined.insert(joined.end(), body.begin(), body.end());
        return CC_SHA256(joined.data(), CC_LONG(joined.size()), output.data()) != nullptr;
    } catch (...) { output.fill(0); return false; }
}

// Whole-object scalar appearance witness, mirroring the fixed capture order of
// CaptureScalarAppearanceForSavedCut (OcctDocument.mm). Codec 1 rejects
// textured/per-face/unsupported appearance bindings at capture; they are never
// silently dropped or stored as guessed values. Absent material and absent
// legacy integers are distinct from authored default values.
struct AppearanceWitness {
    std::array<bool, 2> legacyPresent{};
    std::array<std::int32_t, 2> legacyValues{};
    bool materialPresent = false;
    bool localPBR = false;
    // Fixed 29-scalar order, exactly the captured visualValues sequence:
    // [0] faceCulling enum, [1] alphaMode enum, [2] alphaCutoff,
    // [3] PBR defined 0/1, [4] Common defined 0/1,
    // [5..7] PBR baseColor RGB, [8] baseColor alpha,
    // [9..11] PBR emissiveFactor, [12] metallic, [13] roughness,
    // [14] refractionIndex, [15..17] Common ambient RGB, [18..20] diffuse RGB,
    // [21..23] specular RGB, [24..26] emissive RGB, [27] shininess,
    // [28] transparency.
    std::array<double, VisualValueCount> visualValues{};
};

inline bool Valid(const AppearanceWitness& appearance) noexcept {
    if (!appearance.materialPresent) return !appearance.localPBR;
    for (double value : appearance.visualValues) if (!std::isfinite(value)) return false;
    const auto& v = appearance.visualValues;
    const auto integral = [](double value) {
        return value >= 0 && value <= 255 && value == std::floor(value);
    };
    if (!integral(v[0]) || !integral(v[1])) return false; // culling / alpha mode
    if ((v[3] != 0 && v[3] != 1) || (v[4] != 0 && v[4] != 1)) return false;
    return true;
}

struct GroupMembershipWitness {
    UUID group{};
    std::uint32_t ordinal = 0; // original membership ordinal
    bool hasName = false;
    std::string name; // 0..1024 UTF-8 bytes; hasName distinguishes absent from empty
};

// Ordered original-metadata witness for one consumed source. Groups are
// strictly sorted by UUID and carry stable UUIDs/ordinals/names — never
// label-entry strings or pointers.
struct SourceWitness {
    UUID node{};
    SourceIdentity original{};
    bool hasName = false;
    std::string name; // 0..1024 UTF-8 bytes; hasName distinguishes absent from empty
    AppearanceWitness appearance;
    bool hasGroups = false;
    std::vector<GroupMembershipWitness> groups; // <=16, strictly sorted by group UUID
};

// Canonical appearance-witness substream. Canonical absence is 0x00 0x00.
//   1    legacyMask (bit0/bit1; other bits refuse)
//   4x?  legacyValues[i] int32 LE for each set bit, in index order
//   1    materialPresent
//   1    flags (bit0 = localPBR; other bits refuse)      [present only]
//   29x8 visualValues f64 exact bits                     [present only]
inline bool EncodeAppearanceSubstream(const AppearanceWitness& appearance,
                                      std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!Valid(appearance)) return false;
        PayloadWriter writer;
        const std::uint64_t mask = (appearance.legacyPresent[0] ? 1 : 0)
            | (appearance.legacyPresent[1] ? 2 : 0);
        writer.integer(mask, 1);
        for (std::size_t index = 0; index < 2; ++index) {
            if (!appearance.legacyPresent[index]) continue;
            std::uint32_t bits = 0;
            std::memcpy(&bits, &appearance.legacyValues[index], sizeof(bits));
            writer.integer(bits, 4);
        }
        writer.integer(appearance.materialPresent ? 1 : 0, 1);
        if (appearance.materialPresent) {
            writer.integer(appearance.localPBR ? 1 : 0, 1);
            for (double value : appearance.visualValues) writer.scalar(value);
        }
        if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

// Canonical group-witness substream: groupsPresent byte; when present, a
// 1-byte count (0..16) and entries sorted ascending by group UUID. An authored
// empty membership list (present, count 0) differs from absent state.
// Entry: 16 group UUID | 4 ordinal uint32 LE | 1 namePresent
//        | [2 name length (0..1024) | UTF-8 bytes].
inline bool EncodeGroupsSubstream(bool hasGroups,
                                  const std::vector<GroupMembershipWitness>& groups,
                                  std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (groups.size() > MaximumGroupMemberships) return false;
        for (std::size_t index = 0; index < groups.size(); ++index) {
            const auto& group = groups[index];
            if (!retained_recipe::Nonzero(group.group) || group.name.size() > MaximumNameBytes
                || (index > 0 && !(groups[index - 1].group < group.group))) return false;
        }
        PayloadWriter writer;
        writer.integer(hasGroups ? 1 : 0, 1);
        if (hasGroups) {
            writer.integer(groups.size(), 1);
            for (const auto& group : groups) {
                writer.raw(group.group);
                writer.integer(group.ordinal, 4);
                writer.integer(group.hasName ? 1 : 0, 1);
                if (group.hasName) {
                    writer.integer(group.name.size(), 2);
                    writer.raw(reinterpret_cast<const std::uint8_t*>(group.name.data()),
                               group.name.size());
                }
            }
        }
        if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

// Material commitment: SHA-256 of the canonical appearance-witness substream
// under the SYPC-material/1 domain.
inline bool MaterialCommitment(const AppearanceWitness& appearance, Digest& output) noexcept {
    std::vector<std::uint8_t> substream;
    return EncodeAppearanceSubstream(appearance, substream)
        && DomainHash("SYPC-material/1", substream, output);
}

// Groups commitment: SHA-256 of the canonical group-witness substream under
// the SYPC-groups/1 domain.
inline bool GroupsCommitment(bool hasGroups,
                             const std::vector<GroupMembershipWitness>& groups,
                             Digest& output) noexcept {
    std::vector<std::uint8_t> substream;
    return EncodeGroupsSubstream(hasGroups, groups, substream)
        && DomainHash("SYPC-groups/1", substream, output);
}

// Source placement commitment: SHA-256 of the versioned SYPC-placement/1
// stream of the 16 matrix scalars followed by source/carrier units, all as
// exact finite binary64 bits. Verified bit-exactly at staging and reopen.
inline bool PlacementCommitment(const composite_recipe::InputPlacement& placement,
                                Digest& output) noexcept {
    try {
        PayloadWriter writer;
        for (double scalar : placement.matrix) writer.scalar(scalar);
        writer.scalar(placement.sourceMetersPerUnit);
        writer.scalar(placement.carrierMetersPerUnit);
        if (!writer.valid) { output.fill(0); return false; }
        return DomainHash("SYPC-placement/1", writer.bytes, output);
    } catch (...) { output.fill(0); return false; }
}

// One ordered binary cut. ordinal/total are 1-based within 1..7. Only the
// final step (ordinal == total) carries a nonzero result-binding digest and
// the complete ordered witness vector (total+1 entries, subject first);
// earlier steps require a zero digest and an empty witness vector. The
// feature's two ordered node UUIDs remain in FeatureNode.inputs; roles are
// never inferred from the wire order of source labels.
struct Step {
    std::uint32_t ordinal = 0;
    std::uint32_t total = 0;
    Digest resultBinding{};
    std::vector<SourceWitness> witnesses;
};

// Source witness body:
//   16   source node UUID (the leaf's graph node UUID)
//   16x4 original document/entity/definition/sourceFeature UUIDs
//   1    namePresent; when 1: 2-byte length (0..1024) + UTF-8 bytes
//   ..   appearance substream
//   ..   groups substream
inline bool EncodeSourceWitness(const SourceWitness& witness,
                                std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!retained_recipe::Nonzero(witness.node)
            || !retained_recipe::Valid(witness.original)
            || witness.name.size() > MaximumNameBytes) return false;
        PayloadWriter writer;
        writer.raw(witness.node);
        writer.raw(witness.original.document); writer.raw(witness.original.entity);
        writer.raw(witness.original.definition); writer.raw(witness.original.sourceFeature);
        writer.integer(witness.hasName ? 1 : 0, 1);
        if (witness.hasName) {
            writer.integer(witness.name.size(), 2);
            writer.raw(reinterpret_cast<const std::uint8_t*>(witness.name.data()),
                       witness.name.size());
        }
        std::vector<std::uint8_t> substream;
        if (!EncodeAppearanceSubstream(witness.appearance, substream)) return false;
        writer.raw(substream.data(), substream.size());
        if (!EncodeGroupsSubstream(witness.hasGroups, witness.groups, substream)) return false;
        writer.raw(substream.data(), substream.size());
        if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool DecodeSourceWitness(const std::vector<std::uint8_t>& bytes,
                                SourceWitness& output) noexcept {
    output = {};
    try {
        if (bytes.empty() || bytes.size() > MaximumFeaturePayloadBytes) return false;
        PayloadReader reader(bytes.data(), bytes.size());
        std::uint64_t flag = 0, length = 0;
        SourceWitness witness;
        if (!reader.raw(witness.node) || !reader.raw(witness.original.document)
            || !reader.raw(witness.original.entity) || !reader.raw(witness.original.definition)
            || !reader.raw(witness.original.sourceFeature)) return false;
        if (!reader.integer(1, flag) || flag > 1) return false;
        witness.hasName = flag == 1;
        if (witness.hasName) {
            if (!reader.integer(2, length) || length > MaximumNameBytes
                || length > reader.remaining()) return false;
            witness.name.resize(std::size_t(length));
            if (!reader.raw(reinterpret_cast<std::uint8_t*>(witness.name.data()),
                            witness.name.size())) return false;
        }
        // Appearance substream.
        if (!reader.integer(1, flag) || flag > 3) return false;
        witness.appearance.legacyPresent[0] = (flag & 1) != 0;
        witness.appearance.legacyPresent[1] = (flag & 2) != 0;
        for (std::size_t index = 0; index < 2; ++index) {
            if (!witness.appearance.legacyPresent[index]) continue;
            std::uint64_t bits = 0;
            if (!reader.integer(4, bits)) return false;
            const std::uint32_t narrowed = std::uint32_t(bits);
            std::memcpy(&witness.appearance.legacyValues[index], &narrowed, sizeof(narrowed));
        }
        if (!reader.integer(1, flag) || flag > 1) return false;
        witness.appearance.materialPresent = flag == 1;
        if (witness.appearance.materialPresent) {
            if (!reader.integer(1, flag) || flag > 1) return false;
            witness.appearance.localPBR = flag == 1;
            for (double& value : witness.appearance.visualValues)
                if (!reader.scalar(value)) return false;
        }
        if (!Valid(witness.appearance)) return false;
        // Groups substream.
        if (!reader.integer(1, flag) || flag > 1) return false;
        witness.hasGroups = flag == 1;
        if (witness.hasGroups) {
            std::uint64_t count = 0;
            if (!reader.integer(1, count) || count > MaximumGroupMemberships) return false;
            witness.groups.reserve(std::size_t(count));
            for (std::uint64_t index = 0; index < count; ++index) {
                GroupMembershipWitness group;
                if (!reader.raw(group.group) || !retained_recipe::Nonzero(group.group)) return false;
                std::uint64_t ordinal = 0;
                if (!reader.integer(4, ordinal)) return false;
                group.ordinal = std::uint32_t(ordinal);
                if (!reader.integer(1, flag) || flag > 1) return false;
                group.hasName = flag == 1;
                if (group.hasName) {
                    if (!reader.integer(2, length) || length > MaximumNameBytes
                        || length > reader.remaining()) return false;
                    group.name.resize(std::size_t(length));
                    if (!reader.raw(reinterpret_cast<std::uint8_t*>(group.name.data()),
                                    group.name.size())) return false;
                }
                if (!witness.groups.empty() && !(witness.groups.back().group < group.group))
                    return false; // strictly sorted, no duplicates
                witness.groups.push_back(std::move(group));
            }
        }
        if (!reader.complete()) return false;
        // Canonical form: every accepted witness re-encodes byte-identically.
        std::vector<std::uint8_t> exact;
        if (!EncodeSourceWitness(witness, exact) || exact != bytes) return false;
        output = std::move(witness); return true;
    } catch (...) { output = {}; return false; }
}

// SYPC payload v1 step envelope. Fixed 54-byte header, then length-prefixed
// witnesses; every fixed field, count and length is checked against the
// existing 16 KiB feature cap before allocating. Over-budget refuses, never
// truncates.
inline bool EncodeStep(const Step& step, std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (step.ordinal < MinimumCuts || step.ordinal > step.total
            || step.total > MaximumCuts) return false;
        const bool final = step.ordinal == step.total;
        if (final != retained_recipe::Nonzero(step.resultBinding)) return false;
        if (step.witnesses.size() != (final ? step.total + 1 : 0)) return false;
        PayloadWriter writer;
        writer.raw(reinterpret_cast<const std::uint8_t*>("SYPC"), 4);
        writer.integer(PayloadVersion, 1);
        writer.integer(std::uint8_t(Operation::Subtract), 1);
        writer.integer(std::uint8_t(Lifecycle::ConsumeCreate), 1);
        writer.integer(std::uint8_t(Family::PolygonPrismAxialDisks), 1);
        writer.integer(BuildPolicy, 1);
        writer.integer(step.ordinal, 1);
        writer.integer(step.total, 1);
        writer.integer(0, 1); // reserved
        writer.scalar(FuzzyValue);
        writer.raw(step.resultBinding);
        writer.integer(step.witnesses.size(), 2);
        for (const SourceWitness& witness : step.witnesses) {
            std::vector<std::uint8_t> body;
            if (!EncodeSourceWitness(witness, body)) return false;
            // Length-prefix checked against the feature cap before the bytes
            // are appended; an oversized witness refuses the whole payload.
            if (body.size() + 4 > MaximumFeaturePayloadBytes - writer.bytes.size()) return false;
            writer.integer(body.size(), 4);
            writer.raw(body.data(), body.size());
        }
        if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool DecodeStep(const std::vector<std::uint8_t>& bytes, Step& output) noexcept {
    output = {};
    try {
        if (bytes.size() < 54 || bytes.size() > MaximumFeaturePayloadBytes) return false;
        PayloadReader reader(bytes.data(), bytes.size());
        std::array<std::uint8_t, 4> magic{};
        std::uint64_t tag = 0, ordinal = 0, total = 0, count = 0;
        if (!reader.raw(magic) || std::memcmp(magic.data(), "SYPC", 4) != 0) return false;
        if (!reader.integer(1, tag) || tag != PayloadVersion) return false;
        if (!reader.integer(1, tag) || tag != std::uint8_t(Operation::Subtract)) return false;
        if (!reader.integer(1, tag) || tag != std::uint8_t(Lifecycle::ConsumeCreate)) return false;
        if (!reader.integer(1, tag) || tag != std::uint8_t(Family::PolygonPrismAxialDisks)) return false;
        if (!reader.integer(1, tag) || tag != BuildPolicy) return false;
        if (!reader.integer(1, ordinal) || !reader.integer(1, total)
            || ordinal < MinimumCuts || ordinal > total || total > MaximumCuts) return false;
        if (!reader.integer(1, tag) || tag != 0) return false; // reserved
        double fuzzy = 0;
        if (!reader.scalar(fuzzy)) return false;
        std::uint64_t expectedBits = 0, actualBits = 0;
        const double expected = FuzzyValue;
        std::memcpy(&expectedBits, &expected, sizeof(expectedBits));
        std::memcpy(&actualBits, &fuzzy, sizeof(actualBits));
        if (actualBits != expectedBits) return false; // exact 1.0e-3 bits
        Step step;
        if (!reader.raw(step.resultBinding) || !reader.integer(2, count)
            || count > MaximumSourcesPerChain) return false;
        const bool final = ordinal == total;
        // Only the final step has a nonzero final-result digest and the
        // complete ordered witness vector; earlier steps require a zero
        // digest and an empty witness vector.
        if (final != retained_recipe::Nonzero(step.resultBinding)) return false;
        if (count != (final ? total + 1 : 0)) return false;
        step.ordinal = std::uint32_t(ordinal);
        step.total = std::uint32_t(total);
        step.witnesses.reserve(std::size_t(count));
        for (std::uint64_t index = 0; index < count; ++index) {
            std::uint64_t length = 0;
            if (!reader.integer(4, length) || length == 0 || length > reader.remaining())
                return false;
            std::vector<std::uint8_t> body;
            body.resize(std::size_t(length));
            if (!reader.raw(body.data(), body.size())) return false;
            SourceWitness witness;
            if (!DecodeSourceWitness(body, witness)) return false;
            step.witnesses.push_back(std::move(witness));
        }
        if (!reader.complete()) return false; // surplus/trailing bytes refuse
        // Canonical form: every accepted payload re-encodes byte-identically.
        std::vector<std::uint8_t> exact;
        if (!EncodeStep(step, exact) || exact != bytes) return false;
        output = std::move(step); return true;
    } catch (...) { output = {}; return false; }
}

enum class ChainRefusal : std::uint8_t {
    None = 0, Envelope, Owner, Graph, Identity, Source, Units, Step, Link, Witness, Issuance,
};
struct ChainValidation final {
    ChainRefusal refusal = ChainRefusal::Envelope;
    const char* reason = "v4-envelope";
    bool valid() const noexcept { return refusal == ChainRefusal::None; }
};

// Closed whole-chain validator for the dedicated SYCR/4 cut-only graph.
// Structural/identity/bounds/payload/witness proof only; the profile-family
// checks that need DecodeScalarRecipe (first leaf polygon, tool disks, empty
// shell tails, scalar/unit bits, byte-exact recipe re-encode) are enforced by
// the codec wrapper ValidPlainProfileCutSourceRecipe in
// CompositeRecipeCodec.hxx. Geometry/placement qualification follows in H2.
// Any mixed graph containing a foreign feature kind refuses, even if a cut
// node is last.
inline ChainValidation ValidateChain(const composite_recipe::Definition& definition) noexcept {
    ChainValidation result;
    try {
        using composite_recipe::FeatureNode;
        using composite_recipe::LocalID;
        using composite_recipe::NodeID;
        using composite_recipe::SourceNode;
        if (definition.schemaVersion != GraphVersion) {
            result.reason = "v4-schema"; return result;
        }
        const std::size_t nodeCount = definition.nodes.size();
        if (nodeCount < 2 * MinimumCuts + 1 || nodeCount > MaximumChainNodes
            || nodeCount % 2 == 0) {
            result.reason = "v4-node-count"; return result;
        }
        const std::size_t cuts = (nodeCount - 1) / 2;
        const std::size_t sources = cuts + 1;
        if (!retained_recipe::Valid(definition.owner)
            || !retained_recipe::Valid(definition.issuance)
            || !retained_recipe::Nonzero(definition.outputNode)) {
            result.refusal = ChainRefusal::Owner; result.reason = "v4-owner"; return result;
        }
        // Contiguous partition: N+1 source leaves in subject/tool order, then
        // N cut nodes. Output is the last cut.
        std::vector<const SourceNode*> leaves;
        std::vector<const FeatureNode*> steps;
        leaves.reserve(sources); steps.reserve(cuts);
        for (std::size_t index = 0; index < nodeCount; ++index) {
            const auto& node = definition.nodes[index];
            if (index < sources) {
                const auto* source = std::get_if<SourceNode>(&node.value);
                if (!source) { result.refusal = ChainRefusal::Graph; result.reason = "v4-source-order"; return result; }
                leaves.push_back(source);
            } else {
                const auto* feature = std::get_if<FeatureNode>(&node.value);
                if (!feature) { result.refusal = ChainRefusal::Graph; result.reason = "v4-feature-order"; return result; }
                steps.push_back(feature);
            }
        }
        // Node/local-ID issuance: nonzero distinct UUIDs, strictly monotonic
        // local IDs below nextLocalID, no retired live identity.
        std::set<UUID> nodeIDs, featureUUIDs, sourceFeatures;
        std::set<std::uint64_t> localIDs;
        std::uint64_t previousLocal = 0;
        for (const auto& node : definition.nodes) {
            const UUID& id = NodeID(node);
            const std::uint64_t local = LocalID(node);
            if (!retained_recipe::Nonzero(id) || !nodeIDs.insert(id).second || local == 0
                || local <= previousLocal || local >= definition.issuance.nextLocalID
                || !localIDs.insert(local).second) {
                result.refusal = ChainRefusal::Identity; result.reason = "v4-node-identity"; return result;
            }
            previousLocal = local;
        }
        for (std::uint64_t retired : definition.issuance.retiredLocalIDs) {
            if (localIDs.count(retired)) {
                result.refusal = ChainRefusal::Issuance; result.reason = "v4-retired-live-local-id"; return result;
            }
        }
        // Source leaves: same-document distinct originals, owner distinct
        // from every original, slot == leaf index, Profile recipe family,
        // verified recipe/placement commitments, identical carrier units.
        double carrierUnits = 0;
        for (std::size_t index = 0; index < sources; ++index) {
            const SourceNode& source = *leaves[index];
            if (!retained_recipe::Valid(source.original)
                || source.original.document != definition.owner.document
                || source.original.entity == definition.owner.entity
                || source.original.definition == definition.owner.definition
                || !sourceFeatures.insert(source.original.sourceFeature).second) {
                result.refusal = ChainRefusal::Identity; result.reason = "v4-source-identity"; return result;
            }
            for (std::size_t other = 0; other < index; ++other) {
                const SourceIdentity& prior = leaves[other]->original;
                if (source.original.entity == prior.entity
                    || source.original.definition == prior.definition) {
                    result.refusal = ChainRefusal::Identity; result.reason = "v4-duplicate-original"; return result;
                }
            }
            if (source.shapeSlot != index) {
                result.refusal = ChainRefusal::Identity; result.reason = "v4-shape-slot"; return result;
            }
            if (source.recipe.kind != composite_recipe::RecipeKind::Profile
                || source.recipe.bytes.empty()
                || source.recipe.bytes.size() > MaximumEnvelopeBytes) {
                result.refusal = ChainRefusal::Source; result.reason = "v4-source-family"; return result;
            }
            if (!composite_recipe::Valid(source.commitments)
                || !composite_recipe::ValidPlacement(source.inputToCarrier)) {
                result.refusal = ChainRefusal::Source; result.reason = "v4-source-commitments"; return result;
            }
            Digest commitment;
            if (!HashBytes(source.recipe.bytes, commitment)
                || commitment != source.commitments.recipe) {
                result.refusal = ChainRefusal::Source; result.reason = "v4-recipe-commitment"; return result;
            }
            if (!PlacementCommitment(source.inputToCarrier, commitment)
                || commitment != source.commitments.placement) {
                result.refusal = ChainRefusal::Source; result.reason = "v4-placement-commitment"; return result;
            }
            if (index == 0) {
                carrierUnits = source.inputToCarrier.carrierMetersPerUnit;
            } else if (std::memcmp(&carrierUnits, &source.inputToCarrier.carrierMetersPerUnit,
                                   sizeof(double)) != 0) {
                // All leaves' carrier-units bits equal the archived creation unit.
                result.refusal = ChainRefusal::Units; result.reason = "v4-carrier-units"; return result;
            }
        }
        // Cut nodes: dedicated kind/codec only, exact chain links, canonical
        // step payloads, ordinals 1..N with one shared total.
        for (std::size_t index = 0; index < cuts; ++index) {
            const FeatureNode& feature = *steps[index];
            if (feature.kind != PlainProfileCutKind || feature.codecVersion != CodecVersion) {
                result.refusal = ChainRefusal::Step; result.reason = "v4-feature-kind"; return result;
            }
            if (!retained_recipe::Nonzero(feature.feature)
                || !featureUUIDs.insert(feature.feature).second
                || sourceFeatures.count(feature.feature) || nodeIDs.count(feature.feature)) {
                result.refusal = ChainRefusal::Identity; result.reason = "v4-feature-identity"; return result;
            }
            if (feature.inputs.size() != 2 || feature.inputs[0] == feature.inputs[1]) {
                result.refusal = ChainRefusal::Link; result.reason = "v4-input-arity"; return result;
            }
            // First left input = subject; later left input = previous cut;
            // right input = the corresponding distinct tool (used once each).
            const UUID& expectedLeft = index == 0 ? leaves[0]->node : steps[index - 1]->node;
            if (feature.inputs[0] != expectedLeft || feature.inputs[1] != leaves[index + 1]->node) {
                result.refusal = ChainRefusal::Link; result.reason = "v4-chain-link"; return result;
            }
            Step step;
            if (!DecodeStep(feature.parameters, step)) {
                result.refusal = ChainRefusal::Step; result.reason = "v4-step-payload"; return result;
            }
            if (step.ordinal != index + 1 || step.total != cuts) {
                result.refusal = ChainRefusal::Step; result.reason = "v4-step-ordinal"; return result;
            }
            if (index + 1 < cuts) continue; // non-final: empty witnesses by DecodeStep
            // Final step: the complete ordered witness vector binds each leaf
            // 1:1 — exact source node and original identity, plus material and
            // groups commitments rehashed from the witness substreams.
            if (step.witnesses.size() != sources) {
                result.refusal = ChainRefusal::Witness; result.reason = "v4-witness-count"; return result;
            }
            for (std::size_t leaf = 0; leaf < sources; ++leaf) {
                const SourceWitness& witness = step.witnesses[leaf];
                const SourceNode& source = *leaves[leaf];
                if (witness.node != source.node || !(witness.original == source.original)) {
                    result.refusal = ChainRefusal::Witness; result.reason = "v4-witness-identity"; return result;
                }
                Digest commitment;
                if (!MaterialCommitment(witness.appearance, commitment)
                    || commitment != source.commitments.material) {
                    result.refusal = ChainRefusal::Witness; result.reason = "v4-material-commitment"; return result;
                }
                if (!GroupsCommitment(witness.hasGroups, witness.groups, commitment)
                    || commitment != source.commitments.groups) {
                    result.refusal = ChainRefusal::Witness; result.reason = "v4-groups-commitment"; return result;
                }
            }
        }
        if (definition.outputNode != steps.back()->node) {
            result.refusal = ChainRefusal::Issuance; result.reason = "v4-output"; return result;
        }
        result.refusal = ChainRefusal::None; result.reason = "v4-valid"; return result;
    } catch (...) { result.reason = "v4-validation-exception"; return result; }
}
} // namespace core3d::plain_profile_cut
