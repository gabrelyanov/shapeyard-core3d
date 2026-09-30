#pragma once

// E1 retained per-part finishing. This value is a derivative of one retained
// parametric owner; it never grants authority to mutate the source BRep.
#include "RetainedRecipeIdentity.hxx"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <set>
#include <string>
#include <vector>

namespace core3d::retained_finishing {
using retained_recipe::Digest;
using retained_recipe::OwnerKey;
using retained_recipe::UUID;

inline constexpr char kSchemaVersion[] = "shapeyard.retained-finishing.v1";
inline constexpr std::size_t kMaximumBytes = 8U * 1024U * 1024U;
inline constexpr std::size_t kMaximumMaterials = 64;
inline constexpr std::size_t kMaximumAssignments = 256;
inline constexpr std::size_t kMaximumFinalCorners = 262144;

enum class UnwrapPolicy : std::uint8_t {
    Planar = 1, Cylindrical = 2, Toroidal = 3, Conical = 4, Spherical = 5,
    DiagnosedFallback = 6
};
enum class Quality : std::uint8_t { VerifiedChart = 1, DiagnosedFallback = 2 };
enum class Refusal : std::uint8_t {
    None, Malformed, Oversized, StaleSource, OwnerMismatch, AmbiguousSelector,
    FallbackQualityRefused, TransactionBusy, PersistenceFailure
};

struct SourceRevision final {
    std::uint64_t documentGeneration = 0;
    std::uint64_t modelRevision = 0;
    Digest geometry{}, recipe{}, placement{}, material{}, groups{};
    bool operator==(const SourceRevision& value) const noexcept {
        return documentGeneration == value.documentGeneration
            && modelRevision == value.modelRevision && geometry == value.geometry
            && recipe == value.recipe && placement == value.placement
            && material == value.material && groups == value.groups;
    }
};

struct TessellationProvenance final {
    Digest sourceRevision{}, settings{}, artifact{}, build{};
};

struct MaterialResource final {
    UUID identity{};       // document-owned identity; never a table index
    Digest content{};      // exact resource bytes/content witness
};

// This is the persisted B2 selector contract: semantic identity + proof, not a
// face ordinal. expectedCardinality is re-proved against the current source.
struct FaceAssignment final {
    UUID selector{}, material{};
    Digest selectorProof{};
    std::uint16_t expectedCardinality = 0;
};

// Content witnesses identify the final emitted corner. No vertex/face ordinal
// survives as durable identity; F2 may later consume these records in a remap.
struct FinalCorner final {
    Digest position{}, uv{}, normal{};
    UUID material{};
};

struct Definition final {
    OwnerKey owner;
    UUID finishing{};
    SourceRevision source;
    TessellationProvenance tessellation;
    UnwrapPolicy unwrap = UnwrapPolicy::Planar;
    Quality quality = Quality::VerifiedChart;
    Digest chartProof{};
    std::vector<MaterialResource> materials;
    std::vector<FaceAssignment> assignments;
    std::vector<FinalCorner> finalCorners;
};

inline bool Nonzero(const Digest& value) noexcept { return retained_recipe::Nonzero(value); }
inline bool Current(const Definition& value, const SourceRevision& observed) noexcept {
    return value.source == observed;
}

inline bool Valid(const Definition& value, Refusal& refusal) noexcept {
    refusal = Refusal::Malformed;
    try {
        if (!retained_recipe::Valid(value.owner) || !retained_recipe::Nonzero(value.finishing)
            || value.source.documentGeneration == 0 || value.source.modelRevision == 0
            || !Nonzero(value.source.geometry) || !Nonzero(value.source.recipe)
            || !Nonzero(value.source.placement) || !Nonzero(value.source.material)
            || !Nonzero(value.source.groups) || !Nonzero(value.tessellation.sourceRevision)
            || !Nonzero(value.tessellation.settings) || !Nonzero(value.tessellation.artifact)
            || !Nonzero(value.tessellation.build) || !Nonzero(value.chartProof)
            || value.materials.empty() || value.materials.size() > kMaximumMaterials
            || value.assignments.empty() || value.assignments.size() > kMaximumAssignments
            || value.finalCorners.empty() || value.finalCorners.size() > kMaximumFinalCorners)
            return false;
        const bool fallback = value.unwrap == UnwrapPolicy::DiagnosedFallback;
        if (fallback != (value.quality == Quality::DiagnosedFallback)) return false;
        if (value.unwrap < UnwrapPolicy::Planar || value.unwrap > UnwrapPolicy::DiagnosedFallback)
            return false;
        std::set<UUID> materials, selectors;
        for (const auto& material : value.materials)
            if (!retained_recipe::Nonzero(material.identity) || !Nonzero(material.content)
                || !materials.insert(material.identity).second) return false;
        for (const auto& assignment : value.assignments)
            if (!retained_recipe::Nonzero(assignment.selector)
                || !materials.count(assignment.material) || !Nonzero(assignment.selectorProof)
                || assignment.expectedCardinality == 0
                || !selectors.insert(assignment.selector).second) return false;
        for (const auto& corner : value.finalCorners)
            if (!Nonzero(corner.position) || !Nonzero(corner.uv) || !Nonzero(corner.normal)
                || !materials.count(corner.material)) return false;
        refusal = Refusal::None;
        return true;
    } catch (...) { return false; }
}

namespace detail {
class Writer final {
public:
    std::vector<std::uint8_t> bytes;
    bool ok = true;
    void raw(const std::uint8_t* value, std::size_t count) {
        if (!ok || count > kMaximumBytes - bytes.size()) { ok = false; return; }
        bytes.insert(bytes.end(), value, value + count);
    }
    template<std::size_t N> void raw(const std::array<std::uint8_t, N>& value) { raw(value.data(), N); }
    void integer(std::uint64_t value, unsigned width) {
        std::uint8_t encoded[8]{};
        if (width == 0 || width > 8) { ok = false; return; }
        for (unsigned index = 0; index < width; ++index) encoded[index] = std::uint8_t(value >> (8 * index));
        raw(encoded, width);
    }
};
class Reader final {
public:
    Reader(const std::vector<std::uint8_t>& value, std::size_t end) : bytes(value), limit(end) {}
    bool raw(std::uint8_t* output, std::size_t count) {
        if (!ok || cursor > limit || count > limit - cursor) { ok = false; return false; }
        std::copy_n(bytes.begin() + cursor, count, output); cursor += count; return true;
    }
    template<std::size_t N> bool raw(std::array<std::uint8_t, N>& output) { return raw(output.data(), N); }
    bool integer(unsigned width, std::uint64_t& output) {
        output = 0;
        if (!ok || width == 0 || width > 8 || cursor > limit || width > limit - cursor) { ok = false; return false; }
        for (unsigned index = 0; index < width; ++index) output |= std::uint64_t(bytes[cursor++]) << (8 * index);
        return true;
    }
    bool complete() const noexcept { return ok && cursor == limit; }
private:
    const std::vector<std::uint8_t>& bytes; std::size_t limit = 0, cursor = 0; bool ok = true;
};
} // namespace detail

inline bool Encode(const Definition& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear(); Refusal refusal;
    try {
        if (!Valid(value, refusal)) return false;
        detail::Writer writer; writer.raw(reinterpret_cast<const std::uint8_t*>("SYEF\1\0\0\0"), 8);
        writer.raw(value.owner.document); writer.raw(value.owner.entity); writer.raw(value.owner.definition);
        writer.raw(value.finishing); writer.integer(value.source.documentGeneration, 8);
        writer.integer(value.source.modelRevision, 8);
        for (const Digest* digest : {&value.source.geometry, &value.source.recipe,
             &value.source.placement, &value.source.material, &value.source.groups,
             &value.tessellation.sourceRevision, &value.tessellation.settings,
             &value.tessellation.artifact, &value.tessellation.build, &value.chartProof}) writer.raw(*digest);
        writer.integer(std::uint8_t(value.unwrap), 1); writer.integer(std::uint8_t(value.quality), 1);
        writer.integer(0, 2); writer.integer(value.materials.size(), 2);
        writer.integer(value.assignments.size(), 2); writer.integer(value.finalCorners.size(), 4);
        for (const auto& material : value.materials) { writer.raw(material.identity); writer.raw(material.content); }
        for (const auto& assignment : value.assignments) {
            writer.raw(assignment.selector); writer.raw(assignment.material);
            writer.raw(assignment.selectorProof); writer.integer(assignment.expectedCardinality, 2);
        }
        for (const auto& corner : value.finalCorners) {
            writer.raw(corner.position); writer.raw(corner.uv); writer.raw(corner.normal); writer.raw(corner.material);
        }
        if (!writer.ok || writer.bytes.size() > kMaximumBytes - 32) return false;
        Digest digest{}; if (!retained_solid::Hash(writer.bytes, digest)) return false;
        writer.raw(digest); if (!writer.ok) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Definition& output,
                   Refusal& refusal) noexcept {
    output = {}; refusal = bytes.size() > kMaximumBytes ? Refusal::Oversized : Refusal::Malformed;
    try {
        constexpr std::size_t fixed = 8 + 4 * 16 + 16 + 10 * 32 + 12 + 32;
        if (bytes.size() < fixed || bytes.size() > kMaximumBytes
            || std::memcmp(bytes.data(), "SYEF\1\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{}; if (!retained_solid::Hash(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin()); if (actual != expected) return false;
        detail::Reader reader(bytes, bytes.size() - 32); std::array<std::uint8_t, 8> prefix{};
        Definition value; std::uint64_t unwrap = 0, quality = 0, reserved = 0;
        std::uint64_t materialCount = 0, assignmentCount = 0, cornerCount = 0;
        if (!reader.raw(prefix) || !reader.raw(value.owner.document) || !reader.raw(value.owner.entity)
            || !reader.raw(value.owner.definition) || !reader.raw(value.finishing)
            || !reader.integer(8, value.source.documentGeneration)
            || !reader.integer(8, value.source.modelRevision)) return false;
        for (Digest* digest : {&value.source.geometry, &value.source.recipe,
             &value.source.placement, &value.source.material, &value.source.groups,
             &value.tessellation.sourceRevision, &value.tessellation.settings,
             &value.tessellation.artifact, &value.tessellation.build, &value.chartProof})
            if (!reader.raw(*digest)) return false;
        if (!reader.integer(1, unwrap) || !reader.integer(1, quality)
            || !reader.integer(2, reserved) || reserved != 0
            || !reader.integer(2, materialCount) || materialCount == 0 || materialCount > kMaximumMaterials
            || !reader.integer(2, assignmentCount) || assignmentCount == 0 || assignmentCount > kMaximumAssignments
            || !reader.integer(4, cornerCount) || cornerCount == 0 || cornerCount > kMaximumFinalCorners)
            return false;
        value.unwrap = UnwrapPolicy(unwrap); value.quality = Quality(quality);
        value.materials.resize(std::size_t(materialCount));
        for (auto& material : value.materials)
            if (!reader.raw(material.identity) || !reader.raw(material.content)) return false;
        value.assignments.resize(std::size_t(assignmentCount));
        for (auto& assignment : value.assignments) {
            std::uint64_t cardinality = 0;
            if (!reader.raw(assignment.selector) || !reader.raw(assignment.material)
                || !reader.raw(assignment.selectorProof) || !reader.integer(2, cardinality)
                || cardinality > UINT16_MAX) return false;
            assignment.expectedCardinality = std::uint16_t(cardinality);
        }
        value.finalCorners.resize(std::size_t(cornerCount));
        for (auto& corner : value.finalCorners)
            if (!reader.raw(corner.position) || !reader.raw(corner.uv)
                || !reader.raw(corner.normal) || !reader.raw(corner.material)) return false;
        std::vector<std::uint8_t> canonical; Refusal validation;
        if (!reader.complete() || !Valid(value, validation) || !Encode(value, canonical)
            || canonical != bytes) return false;
        output = std::move(value); refusal = Refusal::None; return true;
    } catch (...) { output = {}; return false; }
}

struct Transaction final {
    OwnerKey owner;
    SourceRevision captured;
    bool prepared = false;

    Refusal Prepare(const Definition& candidate, const OwnerKey& resolvedOwner,
                    const SourceRevision& observed) noexcept {
        Refusal refusal;
        if (prepared) return Refusal::TransactionBusy;
        if (!Valid(candidate, refusal)) return refusal;
        if (!(candidate.owner == resolvedOwner)) return Refusal::OwnerMismatch;
        if (!Current(candidate, observed)) return Refusal::StaleSource;
        owner = resolvedOwner; captured = observed; prepared = true; return Refusal::None;
    }
    Refusal Commit(const OwnerKey& resolvedOwner, const SourceRevision& observed) noexcept {
        if (!prepared) return Refusal::Malformed;
        prepared = false;
        if (!(owner == resolvedOwner)) return Refusal::OwnerMismatch;
        if (!(captured == observed)) return Refusal::StaleSource;
        return Refusal::None;
    }
    void Cancel() noexcept { prepared = false; }
};

} // namespace core3d::retained_finishing
