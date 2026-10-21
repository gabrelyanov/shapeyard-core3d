#pragma once

// E5-3 retained emboss/deboss relief recipe. SYTR/1 is the persisted record
// for one signed-depth relief on a planar host face. It stores identity,
// host receipt, unit scale, and SHA-256 references to C1 curve records; the
// canonical curve bytes themselves stay owned by the C1 slice. No OCCT, OCAF,
// UI, or planner dependency; validate completely before building anything.
#include "BoundedCurveCodec.hxx"
#include "TextReliefResourceAdmission.hxx"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <string>
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
        // Bound the untrusted component index before allocation: an outer
        // count cap alone does not bound perComponent.resize(component + 1).
        if (loop.component >= MaximumComponents)
            return RecipeRefusal::ComponentContourLimit;
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

// SYTR/2 is deliberately a separate authored-owner record.  The SYTR/1
// entry points above retain their exact grammar and emboss/deboss meaning.
inline constexpr std::uint8_t AuthoredSchema = 2;
inline constexpr std::size_t MaximumAuthoredRecordBytes = 65'536;
inline constexpr std::size_t MaximumAuthoredComponents = 128;
inline constexpr std::size_t MaximumAuthoredCurves = 128;
inline constexpr std::size_t MaximumAuthoredLoops = 256;
inline constexpr std::size_t MaximumSelectorProofBytes = 8'192;

enum class AuthoredKind : std::uint8_t { Font = 1, Emblem = 2 };
enum class AuthoredMode : std::uint8_t { Emboss = 1, Deboss = 2, Raised = 3 };
enum class AuthoredAlignment : std::uint8_t { Left = 1, Center = 2, Right = 3 };

struct AuthoredComponent final {
    UUID identifier{};
    std::uint32_t key = 0;
};

struct AuthoredHost final {
    UUID entity{}, faceFeature{}, frame{};
    std::uint64_t frameRevision = 0;
    Digest shapeDigest{}, sourceCommitment{}, geometryCommitment{};
    Digest placementCommitment{}, materialCommitment{}, groupCommitment{};
    std::array<double, 3> origin{}, xAxis{}, yAxis{}, zAxis{};
    std::vector<std::uint8_t> selectorProof;
};

struct AuthoredRecord final {
    retained_recipe::OwnerKey owner;
    UUID feature{}, constructionSeed{}, rootOutput{};
    std::uint64_t revision = 0;
    std::vector<AuthoredComponent> components;
    AuthoredHost host;
    AuthoredKind kind = AuthoredKind::Font;
    std::string catalogHandle;
    std::uint8_t resourceVersion = 1;
    std::string resolver = "shapeyard-syto-1";
    std::uint32_t resourceByteCount = 0;
    Digest resourceSHA256{};
    std::string textUTF8;
    std::string script = "latin";
    std::string emblemRoot;
    double heightMM = 0, trackingMM = 0, depthMM = 0;
    double metersPerLocalUnit = 0, offsetUMM = 0, offsetVMM = 0;
    double rotationRadians = 0;
    AuthoredAlignment alignment = AuthoredAlignment::Left;
    AuthoredMode mode = AuthoredMode::Raised;
    WindingRule winding = WindingRule::NonZero;
    std::vector<CurveRef> curves;
    std::vector<LoopRecord> loops;
};

enum class AuthoredRefusal : std::uint8_t {
    None = 0, BadIdentity, BadRevision, BadComponentMap, BadHost,
    BadResource, BadIntent, BadNumber, BadGeometry, TooLarge
};

namespace authored_detail {
inline bool NegativeZero(double value) noexcept {
    return value == 0.0 && std::signbit(value);
}
inline bool CanonicalNumber(double value) noexcept {
    return bounded_curve::Finite(value) && !NegativeZero(value);
}
inline bool UTF8(const std::string& text, std::size_t maximumBytes,
                 std::size_t maximumScalars = SIZE_MAX) noexcept {
    if (text.size() > maximumBytes) return false;
    std::vector<std::uint32_t> scalars;
    return DecodeUTF8(text, scalars) && scalars.size() <= maximumScalars;
}
inline void Put(std::vector<std::uint8_t>& bytes, std::uint64_t value,
                unsigned width) {
    for (unsigned i = 0; i < width; ++i)
        bytes.push_back(std::uint8_t(value >> (8 * i)));
}
template <std::size_t N> inline void Put(
    std::vector<std::uint8_t>& bytes, const std::array<std::uint8_t, N>& value) {
    bytes.insert(bytes.end(), value.begin(), value.end());
}
inline void PutDouble(std::vector<std::uint8_t>& bytes, double value) {
    std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof(bits));
    Put(bytes, bits, 8);
}
inline bool Take(const std::vector<std::uint8_t>& bytes, std::size_t end,
                 std::size_t& offset, unsigned width, std::uint64_t& value) {
    if (offset > end || width > end - offset) return false;
    value = 0;
    for (unsigned i = 0; i < width; ++i)
        value |= std::uint64_t(bytes[offset++]) << (8 * i);
    return true;
}
template <std::size_t N> inline bool Take(
    const std::vector<std::uint8_t>& bytes, std::size_t end,
    std::size_t& offset, std::array<std::uint8_t, N>& value) {
    if (offset > end || N > end - offset) return false;
    std::copy_n(bytes.begin() + offset, N, value.begin()); offset += N;
    return true;
}
inline bool TakeDouble(const std::vector<std::uint8_t>& bytes, std::size_t end,
                       std::size_t& offset, double& value) {
    std::uint64_t bits = 0;
    if (!Take(bytes, end, offset, 8, bits)) return false;
    std::memcpy(&value, &bits, sizeof(value)); return true;
}
inline void PutString(std::vector<std::uint8_t>& bytes,
                      const std::string& value) {
    Put(bytes, value.size(), 2);
    bytes.insert(bytes.end(), value.begin(), value.end());
}
inline bool TakeString(const std::vector<std::uint8_t>& bytes, std::size_t end,
                       std::size_t& offset, std::string& value) {
    std::uint64_t count = 0;
    if (!Take(bytes, end, offset, 2, count) || count > end - offset) return false;
    value.assign(reinterpret_cast<const char *>(bytes.data() + offset),
                 std::size_t(count));
    offset += std::size_t(count); return true;
}
} // namespace authored_detail

inline AuthoredRefusal Validate(const AuthoredRecord& value) noexcept {
    try {
        if (!retained_recipe::Valid(value.owner)
            || !bounded_curve::Nonzero(value.feature)
            || !bounded_curve::Nonzero(value.constructionSeed)
            || !bounded_curve::Nonzero(value.rootOutput))
            return AuthoredRefusal::BadIdentity;
        if (value.revision == 0) return AuthoredRefusal::BadRevision;
        if (value.components.empty()
            || value.components.size() > MaximumAuthoredComponents)
            return AuthoredRefusal::BadComponentMap;
        std::set<UUID> componentIDs; std::set<std::uint32_t> componentKeys;
        for (const auto& component : value.components)
            if (!bounded_curve::Nonzero(component.identifier)
                || !componentIDs.insert(component.identifier).second
                || !componentKeys.insert(component.key).second)
                return AuthoredRefusal::BadComponentMap;
        if (!bounded_curve::Nonzero(value.host.entity)
            || !bounded_curve::Nonzero(value.host.faceFeature)
            || !bounded_curve::Nonzero(value.host.frame)
            || value.host.frameRevision == 0
            || !retained_recipe::Nonzero(value.host.shapeDigest)
            || !retained_recipe::Nonzero(value.host.sourceCommitment)
            || !retained_recipe::Nonzero(value.host.geometryCommitment)
            || !retained_recipe::Nonzero(value.host.placementCommitment)
            || !retained_recipe::Nonzero(value.host.materialCommitment)
            || !retained_recipe::Nonzero(value.host.groupCommitment)
            || value.host.selectorProof.empty()
            || value.host.selectorProof.size() > MaximumSelectorProofBytes)
            return AuthoredRefusal::BadHost;
        for (double number : value.host.origin)
            if (!authored_detail::CanonicalNumber(number)) return AuthoredRefusal::BadNumber;
        for (double number : value.host.xAxis)
            if (!authored_detail::CanonicalNumber(number)) return AuthoredRefusal::BadNumber;
        for (double number : value.host.yAxis)
            if (!authored_detail::CanonicalNumber(number)) return AuthoredRefusal::BadNumber;
        for (double number : value.host.zAxis)
            if (!authored_detail::CanonicalNumber(number)) return AuthoredRefusal::BadNumber;
        if ((value.kind != AuthoredKind::Font && value.kind != AuthoredKind::Emblem)
            || value.resourceVersion != 1 || value.resolver != "shapeyard-syto-1"
            || value.resourceByteCount == 0 || value.resourceByteCount > 1'048'576
            || !retained_recipe::Nonzero(value.resourceSHA256)
            || !authored_detail::UTF8(value.catalogHandle, 128)
            || value.catalogHandle.empty()) return AuthoredRefusal::BadResource;
        if (value.mode != AuthoredMode::Raised && value.mode != AuthoredMode::Emboss
            && value.mode != AuthoredMode::Deboss) return AuthoredRefusal::BadIntent;
        if (value.alignment != AuthoredAlignment::Left
            && value.alignment != AuthoredAlignment::Center
            && value.alignment != AuthoredAlignment::Right)
            return AuthoredRefusal::BadIntent;
        if (value.winding != WindingRule::NonZero || value.script != "latin"
            || (!value.textUTF8.empty()
                && !authored_detail::UTF8(value.textUTF8, 256, 64))
            || (!value.emblemRoot.empty()
                && !authored_detail::UTF8(value.emblemRoot, 128)))
            return AuthoredRefusal::BadIntent;
        if ((value.kind == AuthoredKind::Font
                && (value.textUTF8.empty() || !value.emblemRoot.empty()))
            || (value.kind == AuthoredKind::Emblem
                && (!value.textUTF8.empty() || value.emblemRoot.empty())))
            return AuthoredRefusal::BadIntent;
        const double numbers[] = {value.heightMM, value.trackingMM, value.depthMM,
            value.metersPerLocalUnit, value.offsetUMM, value.offsetVMM,
            value.rotationRadians};
        for (double number : numbers)
            if (!authored_detail::CanonicalNumber(number)) return AuthoredRefusal::BadNumber;
        if (value.heightMM <= 0 || value.depthMM <= 0
            || value.metersPerLocalUnit <= 0 || value.metersPerLocalUnit > 1)
            return AuthoredRefusal::BadNumber;
        if (value.curves.empty() || value.curves.size() > MaximumAuthoredCurves
            || value.loops.empty() || value.loops.size() > MaximumAuthoredLoops)
            return AuthoredRefusal::BadGeometry;
        std::set<UUID> curves;
        for (const auto& curve : value.curves)
            if (!bounded_curve::Nonzero(curve.feature)
                || !retained_recipe::Nonzero(curve.digest)
                || !curves.insert(curve.feature).second)
                return AuthoredRefusal::BadGeometry;
        std::vector<std::size_t> perComponent(value.components.size());
        std::uint64_t cursor = 0;
        for (const auto& loop : value.loops) {
            if (loop.component >= value.components.size()
                || loop.component >= MaximumAuthoredComponents
                || ++perComponent[loop.component] > MaximumContoursPerComponent
                || loop.firstCurve != cursor || loop.curveCount == 0
                || cursor + loop.curveCount > value.curves.size()
                || (loop.role != LoopRole::Outer && loop.role != LoopRole::Hole))
                return AuthoredRefusal::BadGeometry;
            cursor += loop.curveCount;
        }
        if (cursor != value.curves.size()) return AuthoredRefusal::BadGeometry;
        return AuthoredRefusal::None;
    } catch (...) { return AuthoredRefusal::TooLarge; }
}

// Layout: 12-byte header; fixed owner/component section; fixed host plus
// bounded selector proof; resource pins; authored strings/numbers; C1 refs
// and loop partitions; SHA-256 of every preceding byte.
inline bool EncodeAuthored(const AuthoredRecord& value,
                           std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (Validate(value) != AuthoredRefusal::None) return false;
        using namespace authored_detail;
        std::vector<std::uint8_t> bytes{'S','Y','T','R', AuthoredSchema, 0, 0, 0,
                                        0, 0, 0, 0};
        Put(bytes, value.owner.document); Put(bytes, value.owner.entity);
        Put(bytes, value.owner.definition); Put(bytes, value.feature);
        Put(bytes, value.revision, 8); Put(bytes, value.constructionSeed);
        Put(bytes, value.rootOutput); Put(bytes, value.components.size(), 2);
        Put(bytes, 0, 2);
        for (const auto& component : value.components) {
            Put(bytes, component.identifier); Put(bytes, component.key, 4);
        }
        Put(bytes, value.host.entity); Put(bytes, value.host.faceFeature);
        Put(bytes, value.host.frame); Put(bytes, value.host.frameRevision, 8);
        Put(bytes, value.host.shapeDigest); Put(bytes, value.host.sourceCommitment);
        Put(bytes, value.host.geometryCommitment);
        Put(bytes, value.host.placementCommitment);
        Put(bytes, value.host.materialCommitment); Put(bytes, value.host.groupCommitment);
        for (double number : value.host.origin) PutDouble(bytes, number);
        for (double number : value.host.xAxis) PutDouble(bytes, number);
        for (double number : value.host.yAxis) PutDouble(bytes, number);
        for (double number : value.host.zAxis) PutDouble(bytes, number);
        Put(bytes, value.host.selectorProof.size(), 2); Put(bytes, 0, 2);
        bytes.insert(bytes.end(), value.host.selectorProof.begin(),
                     value.host.selectorProof.end());
        Put(bytes, std::uint8_t(value.kind), 1); Put(bytes, value.resourceVersion, 1);
        Put(bytes, 0, 2); Put(bytes, value.resourceByteCount, 4);
        Put(bytes, value.resourceSHA256); PutString(bytes, value.catalogHandle);
        PutString(bytes, value.resolver);
        Put(bytes, std::uint8_t(value.mode), 1);
        Put(bytes, std::uint8_t(value.alignment), 1);
        Put(bytes, std::uint8_t(value.winding), 1); Put(bytes, 0, 1);
        PutDouble(bytes, value.heightMM); PutDouble(bytes, value.trackingMM);
        PutDouble(bytes, value.depthMM); PutDouble(bytes, value.metersPerLocalUnit);
        PutDouble(bytes, value.offsetUMM); PutDouble(bytes, value.offsetVMM);
        PutDouble(bytes, value.rotationRadians);
        PutString(bytes, value.textUTF8); PutString(bytes, value.script);
        PutString(bytes, value.emblemRoot);
        Put(bytes, value.curves.size(), 2); Put(bytes, value.loops.size(), 2);
        for (const auto& curve : value.curves) {
            Put(bytes, curve.feature); Put(bytes, curve.digest);
        }
        for (const auto& loop : value.loops) {
            Put(bytes, loop.component, 4); Put(bytes, loop.contour, 4);
            Put(bytes, loop.firstCurve, 4); Put(bytes, loop.curveCount, 4);
            Put(bytes, std::uint8_t(loop.role), 1); Put(bytes, 0, 3);
        }
        if (bytes.size() > MaximumAuthoredRecordBytes - 32
            || bytes.size() > UINT32_MAX) return false;
        const std::uint32_t total = std::uint32_t(bytes.size() + 32);
        for (unsigned i = 0; i < 4; ++i) bytes[8 + i] = std::uint8_t(total >> (8 * i));
        Digest digest{};
        if (!bounded_curve::Hash(bytes, MaximumAuthoredRecordBytes, digest)) return false;
        Put(bytes, digest); output = std::move(bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool DecodeAuthored(const std::vector<std::uint8_t>& bytes,
                           AuthoredRecord& output) noexcept {
    output = {};
    try {
        using namespace authored_detail;
        if (bytes.size() < 12 + 32 || bytes.size() > MaximumAuthoredRecordBytes
            || std::memcmp(bytes.data(), "SYTR\2\0\0\0", 8) != 0) return false;
        const std::size_t end = bytes.size() - 32;
        std::size_t offset = 8; std::uint64_t raw = 0, reserved = 0;
        if (!Take(bytes, end, offset, 4, raw) || raw != bytes.size()) return false;
        Digest digest{};
        if (!bounded_curve::Hash(std::vector<std::uint8_t>(bytes.begin(),
                bytes.begin() + end), MaximumAuthoredRecordBytes, digest)
            || !std::equal(digest.begin(), digest.end(), bytes.begin() + end)) return false;
        AuthoredRecord value;
        if (!Take(bytes,end,offset,value.owner.document)
            || !Take(bytes,end,offset,value.owner.entity)
            || !Take(bytes,end,offset,value.owner.definition)
            || !Take(bytes,end,offset,value.feature)
            || !Take(bytes,end,offset,8,value.revision)
            || !Take(bytes,end,offset,value.constructionSeed)
            || !Take(bytes,end,offset,value.rootOutput)) return false;
        std::uint64_t componentCount = 0;
        if (!Take(bytes,end,offset,2,componentCount)
            || !Take(bytes,end,offset,2,reserved) || reserved != 0
            || componentCount == 0 || componentCount > MaximumAuthoredComponents
            || componentCount > (end - offset) / 20) return false;
        value.components.resize(std::size_t(componentCount));
        for (auto& component : value.components) {
            if (!Take(bytes,end,offset,component.identifier)
                || !Take(bytes,end,offset,4,raw)) return false;
            component.key = std::uint32_t(raw);
        }
        if (!Take(bytes,end,offset,value.host.entity)
            || !Take(bytes,end,offset,value.host.faceFeature)
            || !Take(bytes,end,offset,value.host.frame)
            || !Take(bytes,end,offset,8,value.host.frameRevision)
            || !Take(bytes,end,offset,value.host.shapeDigest)
            || !Take(bytes,end,offset,value.host.sourceCommitment)
            || !Take(bytes,end,offset,value.host.geometryCommitment)
            || !Take(bytes,end,offset,value.host.placementCommitment)
            || !Take(bytes,end,offset,value.host.materialCommitment)
            || !Take(bytes,end,offset,value.host.groupCommitment)) return false;
        for (double& number : value.host.origin) if (!TakeDouble(bytes,end,offset,number)) return false;
        for (double& number : value.host.xAxis) if (!TakeDouble(bytes,end,offset,number)) return false;
        for (double& number : value.host.yAxis) if (!TakeDouble(bytes,end,offset,number)) return false;
        for (double& number : value.host.zAxis) if (!TakeDouble(bytes,end,offset,number)) return false;
        std::uint64_t proofCount = 0;
        if (!Take(bytes,end,offset,2,proofCount)
            || !Take(bytes,end,offset,2,reserved) || reserved != 0
            || proofCount == 0 || proofCount > MaximumSelectorProofBytes
            || proofCount > end - offset) return false;
        value.host.selectorProof.assign(bytes.begin() + offset,
                                        bytes.begin() + offset + proofCount);
        offset += std::size_t(proofCount);
        if (!Take(bytes,end,offset,1,raw)) return false; value.kind = AuthoredKind(raw);
        if (!Take(bytes,end,offset,1,raw)) return false; value.resourceVersion = std::uint8_t(raw);
        if (!Take(bytes,end,offset,2,reserved) || reserved != 0
            || !Take(bytes,end,offset,4,raw)) return false;
        value.resourceByteCount = std::uint32_t(raw);
        if (!Take(bytes,end,offset,value.resourceSHA256)
            || !TakeString(bytes,end,offset,value.catalogHandle)
            || !TakeString(bytes,end,offset,value.resolver)) return false;
        if (!Take(bytes,end,offset,1,raw)) return false; value.mode = AuthoredMode(raw);
        if (!Take(bytes,end,offset,1,raw)) return false; value.alignment = AuthoredAlignment(raw);
        if (!Take(bytes,end,offset,1,raw)) return false; value.winding = WindingRule(raw);
        if (!Take(bytes,end,offset,1,reserved) || reserved != 0
            || !TakeDouble(bytes,end,offset,value.heightMM)
            || !TakeDouble(bytes,end,offset,value.trackingMM)
            || !TakeDouble(bytes,end,offset,value.depthMM)
            || !TakeDouble(bytes,end,offset,value.metersPerLocalUnit)
            || !TakeDouble(bytes,end,offset,value.offsetUMM)
            || !TakeDouble(bytes,end,offset,value.offsetVMM)
            || !TakeDouble(bytes,end,offset,value.rotationRadians)
            || !TakeString(bytes,end,offset,value.textUTF8)
            || !TakeString(bytes,end,offset,value.script)
            || !TakeString(bytes,end,offset,value.emblemRoot)) return false;
        std::uint64_t curveCount = 0, loopCount = 0;
        if (!Take(bytes,end,offset,2,curveCount)
            || !Take(bytes,end,offset,2,loopCount)
            || curveCount == 0 || curveCount > MaximumAuthoredCurves
            || loopCount == 0 || loopCount > MaximumAuthoredLoops
            || curveCount > (end - offset) / 48) return false;
        value.curves.resize(std::size_t(curveCount));
        for (auto& curve : value.curves)
            if (!Take(bytes,end,offset,curve.feature)
                || !Take(bytes,end,offset,curve.digest)) return false;
        if (loopCount > (end - offset) / 20) return false;
        value.loops.resize(std::size_t(loopCount));
        for (auto& loop : value.loops) {
            if (!Take(bytes,end,offset,4,raw)) return false; loop.component = std::uint32_t(raw);
            if (!Take(bytes,end,offset,4,raw)) return false; loop.contour = std::uint32_t(raw);
            if (!Take(bytes,end,offset,4,raw)) return false; loop.firstCurve = std::uint32_t(raw);
            if (!Take(bytes,end,offset,4,raw)) return false; loop.curveCount = std::uint32_t(raw);
            if (!Take(bytes,end,offset,1,raw)) return false; loop.role = LoopRole(raw);
            if (!Take(bytes,end,offset,3,reserved) || reserved != 0) return false;
        }
        std::vector<std::uint8_t> canonical;
        if (offset != end || Validate(value) != AuthoredRefusal::None
            || !EncodeAuthored(value, canonical) || canonical != bytes) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}
} // namespace core3d::text_relief
