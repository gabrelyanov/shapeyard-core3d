#pragma once

// E5-3 retained emboss/deboss relief recipe. SYTR/1 is the persisted record
// for one signed-depth relief on a planar host face. It stores identity,
// host receipt, unit scale, and SHA-256 references to C1 curve records; the
// canonical curve bytes themselves stay owned by the C1 slice. No OCCT, OCAF,
// UI, or planner dependency; validate completely before building anything.
#include "BoundedCurveCodec.hxx"
#include "TextReliefResourceAdmission.hxx"
#include <cstring>
#include <set>
#include <vector>

namespace core3d::text_relief {
using bounded_curve::Digest;
using bounded_curve::UUID;

enum class ReliefOperation : std::uint8_t { Emboss = 1, Deboss = 2 };
enum class LoopRole : std::uint8_t { Outer = 1, Hole = 2 };
enum class WindingRule : std::uint8_t { NonZero = 1 };

inline constexpr double MinimumReliefDepthMM = 0.01;
inline constexpr double MaximumReliefDepthMM = 10000.0;
inline constexpr std::size_t MaximumRecipeBytes = 65'536;
inline constexpr std::uint32_t Schema = 1;

// Immutable evidence of the planar host face the relief was authorized
// against. Any drift in identity, revision, or shape bytes is staleness.
struct HostReceipt {
    UUID entity{}, faceFeature{}, frame{};
    std::uint64_t frameRevision = 0;
    Digest shapeDigest{};
};

// Identity plus SHA-256 of the curve's canonical SYCV bytes. The bytes
// themselves are never duplicated into this recipe.
struct CurveRef {
    UUID feature{};
    Digest digest{};
};

struct LoopRecord {
    std::uint32_t component = 0, contour = 0, firstCurve = 0, curveCount = 0;
    LoopRole role = LoopRole::Outer;
};

struct ReliefRecipe {
    std::uint32_t schema = Schema;
    UUID feature{};
    retained_recipe::OwnerKey owner;
    HostReceipt host;
    ReliefOperation operation = ReliefOperation::Emboss;
    // Signed along the stored host frame's zAxis: Emboss fuses (depth > 0),
    // Deboss cuts (depth < 0).
    double depthMM = 0;
    double metersPerLocalUnit = 0;
    WindingRule winding = WindingRule::NonZero;
    Digest resourceSHA256{};
    std::vector<CurveRef> curves;
    std::vector<LoopRecord> loops;
};

// Named RecipeRefusal because text_relief::Refusal is already the E5-2
// resource-admission refusal enum in TextReliefResourceAdmission.hxx.
enum class RecipeRefusal : std::uint8_t {
    None = 0, BadSchema, BadIdentity, BadOwner, BadHost, BadOperation,
    BadDepth, BadUnitScale, BadResourceDigest, CurveLimit, BadCurveRef,
    DuplicateCurveID, ContourLimit, ComponentContourLimit, BadLoopPartition,
    BadLoopRole
};

inline bool Valid(const HostReceipt& receipt) noexcept {
    return bounded_curve::Nonzero(receipt.entity)
        && bounded_curve::Nonzero(receipt.faceFeature)
        && bounded_curve::Nonzero(receipt.frame) && receipt.frameRevision > 0
        && retained_recipe::Nonzero(receipt.shapeDigest);
}

inline RecipeRefusal Validate(const ReliefRecipe& value) noexcept {
    if (value.schema != Schema) return RecipeRefusal::BadSchema;
    if (!bounded_curve::Nonzero(value.feature)) return RecipeRefusal::BadIdentity;
    if (!retained_recipe::Valid(value.owner)) return RecipeRefusal::BadOwner;
    if (!Valid(value.host)) return RecipeRefusal::BadHost;
    if (value.operation != ReliefOperation::Emboss
        && value.operation != ReliefOperation::Deboss) return RecipeRefusal::BadOperation;
    const double depth = std::abs(value.depthMM);
    if (!bounded_curve::Finite(value.depthMM) || depth < MinimumReliefDepthMM
        || depth > MaximumReliefDepthMM
        || (value.operation == ReliefOperation::Emboss && value.depthMM <= 0)
        || (value.operation == ReliefOperation::Deboss && value.depthMM >= 0))
        return RecipeRefusal::BadDepth;
    if (!bounded_curve::Finite(value.metersPerLocalUnit)
        || value.metersPerLocalUnit <= 0 || value.metersPerLocalUnit > 1)
        return RecipeRefusal::BadUnitScale;
    if (value.winding != WindingRule::NonZero
        || !retained_recipe::Nonzero(value.resourceSHA256))
        return RecipeRefusal::BadResourceDigest;
    if (value.curves.empty() || value.curves.size() > MaximumCurves)
        return RecipeRefusal::CurveLimit;
    std::set<UUID> features;
    for (const CurveRef& curve : value.curves) {
        if (!bounded_curve::Nonzero(curve.feature)
            || !retained_recipe::Nonzero(curve.digest)) return RecipeRefusal::BadCurveRef;
        if (!features.insert(curve.feature).second) return RecipeRefusal::DuplicateCurveID;
    }
    if (value.loops.empty() || value.loops.size() > MaximumContours)
        return RecipeRefusal::ContourLimit;
    std::vector<std::size_t> perComponent;
    for (const LoopRecord& loop : value.loops) {
        if (loop.component >= perComponent.size()) perComponent.resize(loop.component + 1);
        if (++perComponent[loop.component] > MaximumContoursPerComponent)
            return RecipeRefusal::ComponentContourLimit;
    }
    std::uint64_t cursor = 0; bool sawOuter = false;
    for (const LoopRecord& loop : value.loops) {
        if (loop.curveCount == 0 || loop.firstCurve != cursor
            || cursor + loop.curveCount > value.curves.size())
            return RecipeRefusal::BadLoopPartition;
        cursor += loop.curveCount;
        if (loop.role != LoopRole::Outer && loop.role != LoopRole::Hole)
            return RecipeRefusal::BadLoopRole;
        // Holes are recorded after their outer; a hole may never lead.
        if (loop.role == LoopRole::Hole && !sawOuter) return RecipeRefusal::BadLoopRole;
        sawOuter = sawOuter || loop.role == LoopRole::Outer;
    }
    if (cursor != value.curves.size()) return RecipeRefusal::BadLoopPartition;
    if (!sawOuter) return RecipeRefusal::BadLoopRole;
    return RecipeRefusal::None;
}

// Deterministic version-5-style feature identity. Byte-identical inputs give
// byte-identical identity; any single field drift gives an unrelated UUID.
inline UUID DeriveReliefFeature(const UUID& seed, const HostReceipt& host,
                                const Digest& resourceSHA256,
                                ReliefOperation operation, double depthMM,
                                double metersPerLocalUnit,
                                const std::vector<Digest>& curveDigests) noexcept {
    UUID result{};
    try {
        std::vector<std::uint8_t> material;
        material.reserve(16 + 48 + 8 + 64 + 1 + 16 + 32 * curveDigests.size());
        const auto append = [](std::vector<std::uint8_t>& out,
                               const std::uint8_t* data, std::size_t count) {
            out.insert(out.end(), data, data + count);
        };
        append(material, seed.data(), seed.size());
        append(material, host.entity.data(), host.entity.size());
        append(material, host.faceFeature.data(), host.faceFeature.size());
        append(material, host.frame.data(), host.frame.size());
        for (unsigned i = 0; i < 8; ++i)
            material.push_back(std::uint8_t(host.frameRevision >> (8 * i)));
        append(material, host.shapeDigest.data(), host.shapeDigest.size());
        append(material, resourceSHA256.data(), resourceSHA256.size());
        material.push_back(std::uint8_t(operation));
        std::uint64_t bits = 0;
        std::memcpy(&bits, &depthMM, sizeof(bits));
        for (unsigned i = 0; i < 8; ++i) material.push_back(std::uint8_t(bits >> (8 * i)));
        std::memcpy(&bits, &metersPerLocalUnit, sizeof(bits));
        for (unsigned i = 0; i < 8; ++i) material.push_back(std::uint8_t(bits >> (8 * i)));
        for (const Digest& digest : curveDigests)
            append(material, digest.data(), digest.size());
        Digest hashed{};
        if (!bounded_curve::Hash(material, 8 * MaximumRecipeBytes, hashed)) return result;
        std::copy_n(hashed.begin(), result.size(), result.begin());
        result[6] = std::uint8_t((result[6] & 0x0f) | 0x50);
        result[8] = std::uint8_t((result[8] & 0x3f) | 0x80);
        return result;
    } catch (...) { return UUID{}; }
}

// Canonical SYTR/1 wire form: fixed little-endian header, curve reference
// array, loop partition array, SHA-256 trailer over the whole body.
inline bool Encode(const ReliefRecipe& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (Validate(value) != RecipeRefusal::None) return false;
        bounded_curve::Writer writer(MaximumRecipeBytes);
        writer.raw(reinterpret_cast<const std::uint8_t*>("SYTR"), 4);
        writer.integer(Schema, 1); writer.integer(0, 3);
        writer.raw(value.feature);
        writer.raw(value.owner.document); writer.raw(value.owner.entity);
        writer.raw(value.owner.definition);
        writer.raw(value.host.entity); writer.raw(value.host.faceFeature);
        writer.raw(value.host.frame); writer.integer(value.host.frameRevision, 8);
        writer.raw(value.host.shapeDigest);
        writer.integer(std::uint8_t(value.operation), 1);
        writer.integer(std::uint8_t(value.winding), 1); writer.integer(0, 6);
        writer.scalar(value.depthMM); writer.scalar(value.metersPerLocalUnit);
        writer.raw(value.resourceSHA256);
        writer.integer(value.curves.size(), 2); writer.integer(0, 6);
        for (const CurveRef& curve : value.curves) {
            writer.raw(curve.feature); writer.raw(curve.digest);
        }
        writer.integer(value.loops.size(), 2); writer.integer(0, 6);
        for (const LoopRecord& loop : value.loops) {
            writer.integer(loop.component, 4); writer.integer(loop.contour, 4);
            writer.integer(loop.firstCurve, 4); writer.integer(loop.curveCount, 4);
            writer.integer(std::uint8_t(loop.role), 1); writer.integer(0, 7);
        }
        if (!writer.valid || writer.bytes.size() > MaximumRecipeBytes - 32) return false;
        Digest digest{};
        if (!bounded_curve::Hash(writer.bytes, MaximumRecipeBytes, digest)) return false;
        writer.raw(digest);
        if (!writer.valid) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, ReliefRecipe& output) noexcept {
    output = {};
    try {
        constexpr std::size_t FixedBytes = 4 + 1 + 3 + 16 + 48 + 16 * 3 + 8 + 32
            + 1 + 1 + 6 + 8 + 8 + 32 + 2 + 6;
        constexpr std::size_t MinimumBytes = FixedBytes + 48 + 2 + 6 + 24 + 32;
        if (bytes.size() < MinimumBytes || bytes.size() > MaximumRecipeBytes
            || std::memcmp(bytes.data(), "SYTR\1\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{};
        if (!bounded_curve::Hash(body, MaximumRecipeBytes, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin());
        if (actual != expected) return false;
        bounded_curve::Reader reader(bytes, bytes.size() - 32);
        std::array<std::uint8_t, 8> prefix{};
        std::uint64_t raw = 0, reserved = 0, curveCount = 0, loopCount = 0;
        ReliefRecipe value;
        if (!reader.raw(prefix) || !reader.raw(value.feature)
            || !reader.raw(value.owner.document) || !reader.raw(value.owner.entity)
            || !reader.raw(value.owner.definition) || !reader.raw(value.host.entity)
            || !reader.raw(value.host.faceFeature) || !reader.raw(value.host.frame)
            || !reader.integer(8, value.host.frameRevision)
            || !reader.raw(value.host.shapeDigest)) return false;
        if (!reader.integer(1, raw)) return false;
        value.operation = ReliefOperation(raw);
        if (!reader.integer(1, raw)) return false;
        value.winding = WindingRule(raw);
        if (!reader.integer(6, reserved) || reserved != 0
            || !reader.scalar(value.depthMM) || !reader.scalar(value.metersPerLocalUnit)
            || !reader.raw(value.resourceSHA256)) return false;
        if (!reader.integer(2, curveCount) || !reader.integer(6, reserved)
            || reserved != 0 || curveCount == 0 || curveCount > MaximumCurves
            || curveCount > reader.remaining() / 48) return false;
        value.curves.resize(std::size_t(curveCount));
        for (CurveRef& curve : value.curves)
            if (!reader.raw(curve.feature) || !reader.raw(curve.digest)) return false;
        if (!reader.integer(2, loopCount) || !reader.integer(6, reserved)
            || reserved != 0 || loopCount == 0 || loopCount > MaximumContours
            || loopCount > reader.remaining() / 24) return false;
        value.loops.resize(std::size_t(loopCount));
        for (LoopRecord& loop : value.loops) {
            if (!reader.integer(4, raw)) return false;
            loop.component = std::uint32_t(raw);
            if (!reader.integer(4, raw)) return false;
            loop.contour = std::uint32_t(raw);
            if (!reader.integer(4, raw)) return false;
            loop.firstCurve = std::uint32_t(raw);
            if (!reader.integer(4, raw)) return false;
            loop.curveCount = std::uint32_t(raw);
            if (!reader.integer(1, raw)) return false;
            loop.role = LoopRole(raw);
            if (!reader.integer(7, reserved) || reserved != 0) return false;
        }
        std::vector<std::uint8_t> canonical;
        if (!reader.complete() || Validate(value) != RecipeRefusal::None
            || !Encode(value, canonical) || canonical != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}
} // namespace core3d::text_relief
