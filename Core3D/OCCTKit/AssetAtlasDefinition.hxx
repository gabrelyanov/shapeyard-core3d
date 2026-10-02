#pragma once

// E2a asset-wide atlas. One explicit ordered set of admitted retained
// finishing derivatives shares one atlas owner with common density/gutter
// settings, stable part/chart/resource identities and all-member revision
// fences. This value is a derivative of its members; it never grants
// authority to mutate any source BRep. The SYEA/1 codec below mirrors the
// SYEF/1 value seam in RetainedFinishingRecord.hxx and never widens it.
#include "RetainedFinishingRecord.hxx"
#include <CommonCrypto/CommonDigest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace core3d::asset_atlas {
using retained_recipe::Digest;
using retained_recipe::OwnerKey;
using retained_recipe::UUID;

inline constexpr char kSchemaVersion[] = "shapeyard.asset-atlas.v1";
inline constexpr std::size_t kMaximumBytes = 16U * 1024U * 1024U;
inline constexpr std::size_t kMaximumMembers = 8;
inline constexpr std::size_t kMaximumCharts = 512;
inline constexpr std::size_t kMaximumMaterials = 128;
inline constexpr std::size_t kMaximumFinalCorners = 524288;

enum class Refusal : std::uint8_t {
    None, Malformed, Oversized, StaleSource, MissingMember, ForeignMember,
    OverBudget, PaintedRebakeRequired, UnsupportedSurface, OwnerMismatch,
    TransactionBusy, PersistenceFailure
};

struct Key final {
    UUID document{}, atlas{};
    bool operator==(const Key& value) const noexcept {
        return document == value.document && atlas == value.atlas;
    }
};
struct Member final {
    UUID member{};                 // atlas-scoped stable part identity
    OwnerKey owner{};              // admitted retained solid owner
    UUID finishing{};              // fenced row-268 derivative receipt
    retained_finishing::SourceRevision source{}; // fenced all-member revision
    bool operator==(const Member& value) const noexcept {
        return member == value.member && owner == value.owner
            && finishing == value.finishing && source == value.source;
    }
};
struct Chart final {
    UUID chart{};                  // deterministic per the atlas chart rule
    UUID member{};
    UUID material{};
    std::uint32_t triangleCount = 0;
    double developedAreaMM2 = 0;
    std::array<double, 4> rectUV{}; // placed chart bounds in [0,1]^2
    bool operator==(const Chart& value) const noexcept {
        return chart == value.chart && member == value.member
            && material == value.material && triangleCount == value.triangleCount
            && developedAreaMM2 == value.developedAreaMM2 && rectUV == value.rectUV;
    }
};
struct Definition final {
    Key key{};
    int resolutionTexels = 0;
    int gutterTexels = 0;
    double globalTexelsPerMM = 0;
    std::vector<Member> members;
    std::vector<Chart> charts;
    std::vector<retained_finishing::MaterialResource> resources;
    Digest layoutProof{};
};

// Staging-only per-member UV assignment. It is never persisted as bytes: the
// committed corners are bound into Definition.layoutProof at build time and
// are deterministically reproducible from the fenced member state. Commit
// installs the atlas attribute whose layoutProof binds every assignment
// inside the caller's one open OCAF command.
struct MemberUVAssignment final {
    UUID member{};
    std::uint32_t triangleCount = 0;
    // Three committed UV corners per tessellation-copy triangle, in meshcopy
    // emission order (corner (3*t + k) belongs to triangle t, corner k).
    std::vector<std::array<double, 2>> corners;
};

inline bool Nonzero(const Digest& value) noexcept { return retained_recipe::Nonzero(value); }

inline bool Current(const Definition& value, const std::vector<Member>& observed) noexcept {
    try {
        if (value.members.size() != observed.size()) return false;
        std::vector<bool> matched(value.members.size(), false);
        for (const auto& member : observed) {
            bool found = false;
            for (std::size_t index = 0; index < value.members.size(); ++index)
                if (!matched[index] && value.members[index] == member) {
                    matched[index] = true; found = true; break;
                }
            if (!found) return false;
        }
        return true;
    } catch (...) { return false; }
}

inline bool Valid(const Definition& value, Refusal& refusal) noexcept {
    refusal = Refusal::Malformed;
    try {
        if (!retained_recipe::Nonzero(value.key.document)
            || !retained_recipe::Nonzero(value.key.atlas)
            || value.resolutionTexels < 256 || value.resolutionTexels > 4096
            || (value.resolutionTexels & (value.resolutionTexels - 1)) != 0
            || value.gutterTexels < 1 || value.gutterTexels > 8
            || !std::isfinite(value.globalTexelsPerMM) || value.globalTexelsPerMM <= 0
            || value.members.empty() || value.members.size() > kMaximumMembers
            || value.charts.empty() || value.charts.size() > kMaximumCharts
            || value.resources.empty() || value.resources.size() > kMaximumMaterials
            || !Nonzero(value.layoutProof))
            return false;
        std::uint64_t corners = 0;
        for (std::size_t first = 0; first < value.members.size(); ++first) {
            const auto& member = value.members[first];
            if (!retained_recipe::Nonzero(member.member)
                || !retained_recipe::Valid(member.owner)
                || !(member.owner.document == value.key.document)
                || !retained_recipe::Nonzero(member.finishing)
                || member.source.documentGeneration == 0 || member.source.modelRevision == 0
                || !Nonzero(member.source.geometry) || !Nonzero(member.source.recipe)
                || !Nonzero(member.source.placement) || !Nonzero(member.source.material)
                || !Nonzero(member.source.groups))
                return false;
            for (std::size_t other = first + 1; other < value.members.size(); ++other)
                if (value.members[other].member == member.member
                    || value.members[other].owner == member.owner) return false;
        }
        for (std::size_t first = 0; first < value.resources.size(); ++first) {
            const auto& resource = value.resources[first];
            if (!retained_recipe::Nonzero(resource.identity) || !Nonzero(resource.content))
                return false;
            for (std::size_t other = first + 1; other < value.resources.size(); ++other)
                if (value.resources[other].identity == resource.identity) return false;
        }
        std::vector<bool> chartedMembers(value.members.size(), false);
        for (std::size_t first = 0; first < value.charts.size(); ++first) {
            const auto& chart = value.charts[first];
            if (!retained_recipe::Nonzero(chart.chart) || chart.triangleCount == 0
                || corners > kMaximumFinalCorners - 3ULL * chart.triangleCount
                || !std::isfinite(chart.developedAreaMM2) || chart.developedAreaMM2 <= 0)
                return false;
            corners += 3ULL * chart.triangleCount;
            for (int bound = 0; bound < 4; ++bound)
                if (!std::isfinite(chart.rectUV[bound]) || chart.rectUV[bound] < 0
                    || chart.rectUV[bound] > 1) return false;
            if (!(chart.rectUV[0] < chart.rectUV[2]) || !(chart.rectUV[1] < chart.rectUV[3]))
                return false;
            bool memberFound = false, materialFound = false;
            for (std::size_t index = 0; index < value.members.size(); ++index)
                if (value.members[index].member == chart.member) {
                    memberFound = true; chartedMembers[index] = true; break;
                }
            for (const auto& resource : value.resources)
                if (resource.identity == chart.material) { materialFound = true; break; }
            if (!memberFound || !materialFound) return false;
            for (std::size_t other = first + 1; other < value.charts.size(); ++other)
                if (value.charts[other].chart == chart.chart) return false;
        }
        for (bool charted : chartedMembers) if (!charted) return false;
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
    void real(double value) {
        std::uint64_t bits = 0;
        static_assert(sizeof bits == sizeof value, "double width");
        std::memcpy(&bits, &value, sizeof bits);
        integer(bits, 8);
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
    bool real(double& output) {
        std::uint64_t bits = 0;
        if (!integer(8, bits)) return false;
        std::memcpy(&output, &bits, sizeof output);
        return true;
    }
    bool complete() const noexcept { return ok && cursor == limit; }
private:
    const std::vector<std::uint8_t>& bytes; std::size_t limit = 0, cursor = 0; bool ok = true;
};

// Canonical body writer shared by Encode and the layout-proof binding. The
// proof covers every field except layoutProof itself, which the caller zeros.
inline void EncodeBody(const Definition& value, Writer& writer) {
    writer.raw(reinterpret_cast<const std::uint8_t*>("SYEA\1\0\0\0"), 8);
    writer.raw(value.key.document); writer.raw(value.key.atlas);
    writer.integer(std::uint32_t(value.resolutionTexels), 4);
    writer.integer(std::uint32_t(value.gutterTexels), 4);
    writer.real(value.globalTexelsPerMM);
    writer.integer(0, 2);
    writer.integer(value.members.size(), 2);
    writer.integer(value.charts.size(), 2);
    writer.integer(value.resources.size(), 2);
    for (const auto& member : value.members) {
        writer.raw(member.member);
        writer.raw(member.owner.document); writer.raw(member.owner.entity);
        writer.raw(member.owner.definition); writer.raw(member.finishing);
        writer.integer(member.source.documentGeneration, 8);
        writer.integer(member.source.modelRevision, 8);
        for (const Digest* digest : {&member.source.geometry, &member.source.recipe,
                 &member.source.placement, &member.source.material, &member.source.groups})
            writer.raw(*digest);
    }
    for (const auto& chart : value.charts) {
        writer.raw(chart.chart); writer.raw(chart.member); writer.raw(chart.material);
        writer.integer(chart.triangleCount, 4);
        writer.real(chart.developedAreaMM2);
        for (double bound : chart.rectUV) writer.real(bound);
    }
    for (const auto& resource : value.resources) {
        writer.raw(resource.identity); writer.raw(resource.content);
    }
    writer.raw(value.layoutProof);
}
} // namespace detail

// SHA-256 over atlas-scale bytes, bounded by this module's own kMaximumBytes.
// retained_solid::Hash carries the 64 KiB retained-envelope bound and must
// not be used for atlas-scale bytes. Null, empty and over-bound input is
// refused with a zeroed digest.
static_assert(kMaximumBytes <= std::numeric_limits<CC_LONG>::max(),
              "atlas byte bound must fit CC_LONG");
inline bool HashAtlasBytes(const std::uint8_t* bytes, std::size_t size,
                           Digest& output) noexcept {
    output = {};
    if (!bytes || size == 0 || size > kMaximumBytes) return false;
    return CC_SHA256(bytes, CC_LONG(size), output.data()) != nullptr;
}
inline bool HashAtlasBytes(const std::vector<std::uint8_t>& bytes,
                           Digest& output) noexcept {
    return HashAtlasBytes(bytes.data(), bytes.size(), output);
}

// Binds the exact committed UV assignments into the persisted record. The
// proof is SHA-256 over the canonical encoding of every Definition field
// except layoutProof itself plus each member's UV corner bytes, interleaved
// per member in admission order. Assignments must cover every member exactly,
// in admission order, with three corners per triangle.
inline bool BindLayoutProof(Definition& value,
                            const std::vector<MemberUVAssignment>& assignments) noexcept {
    try {
        if (assignments.size() != value.members.size()) return false;
        Definition covered = value;
        covered.layoutProof = Digest{};
        detail::Writer writer;
        detail::EncodeBody(covered, writer);
        for (std::size_t index = 0; index < assignments.size(); ++index) {
            const auto& assignment = assignments[index];
            if (!(assignment.member == value.members[index].member)
                || assignment.triangleCount == 0
                || assignment.corners.size() != 3ULL * assignment.triangleCount)
                return false;
            writer.raw(assignment.member);
            writer.integer(assignment.triangleCount, 4);
            for (const auto& corner : assignment.corners)
                for (double scalar : corner) writer.real(scalar);
        }
        if (!writer.ok || writer.bytes.size() > kMaximumBytes) return false;
        Digest proof{};
        if (!HashAtlasBytes(writer.bytes, proof) || !Nonzero(proof)) return false;
        value.layoutProof = proof;
        return true;
    } catch (...) { return false; }
}

// Commit-side staging check: every staged assignment matches the candidate's
// members, counts and the [gutter/N, 1 - gutter/N] atlas bounds.
inline bool PlausibleUVAssignments(
    const Definition& value,
    const std::vector<MemberUVAssignment>& assignments) noexcept {
    try {
        if (assignments.size() != value.members.size()) return false;
        const double low = double(value.gutterTexels) / double(value.resolutionTexels);
        const double high = 1.0 - low;
        for (std::size_t index = 0; index < assignments.size(); ++index) {
            const auto& assignment = assignments[index];
            if (!(assignment.member == value.members[index].member)
                || assignment.triangleCount == 0
                || assignment.corners.size() != 3ULL * assignment.triangleCount)
                return false;
            for (const auto& corner : assignment.corners)
                for (double scalar : corner)
                    if (!std::isfinite(scalar) || scalar < low - 1.0e-12
                        || scalar > high + 1.0e-12) return false;
        }
        return true;
    } catch (...) { return false; }
}

inline bool Encode(const Definition& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear(); Refusal refusal;
    try {
        if (!Valid(value, refusal)) return false;
        detail::Writer writer;
        detail::EncodeBody(value, writer);
        if (!writer.ok || writer.bytes.size() > kMaximumBytes - 32) return false;
        Digest digest{}; if (!HashAtlasBytes(writer.bytes, digest)) return false;
        writer.raw(digest); if (!writer.ok) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Definition& output,
                   Refusal& refusal) noexcept {
    output = {}; refusal = bytes.size() > kMaximumBytes ? Refusal::Oversized : Refusal::Malformed;
    try {
        constexpr std::size_t fixed = 8 + 2 * 16 + 4 + 4 + 8 + 2 + 3 * 2 + 32 + 32;
        if (bytes.size() < fixed || bytes.size() > kMaximumBytes
            || std::memcmp(bytes.data(), "SYEA\1\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{}; if (!HashAtlasBytes(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin()); if (actual != expected) return false;
        detail::Reader reader(bytes, bytes.size() - 32); std::array<std::uint8_t, 8> prefix{};
        Definition value; std::uint64_t resolution = 0, gutter = 0, reserved = 0;
        std::uint64_t memberCount = 0, chartCount = 0, resourceCount = 0;
        if (!reader.raw(prefix) || !reader.raw(value.key.document) || !reader.raw(value.key.atlas)
            || !reader.integer(4, resolution) || !reader.integer(4, gutter)
            || !reader.real(value.globalTexelsPerMM)
            || !reader.integer(2, reserved) || reserved != 0
            || !reader.integer(2, memberCount) || memberCount == 0 || memberCount > kMaximumMembers
            || !reader.integer(2, chartCount) || chartCount == 0 || chartCount > kMaximumCharts
            || !reader.integer(2, resourceCount) || resourceCount == 0
            || resourceCount > kMaximumMaterials)
            return false;
        value.resolutionTexels = int(resolution); value.gutterTexels = int(gutter);
        value.members.resize(std::size_t(memberCount));
        for (auto& member : value.members) {
            if (!reader.raw(member.member) || !reader.raw(member.owner.document)
                || !reader.raw(member.owner.entity) || !reader.raw(member.owner.definition)
                || !reader.raw(member.finishing)
                || !reader.integer(8, member.source.documentGeneration)
                || !reader.integer(8, member.source.modelRevision)) return false;
            for (Digest* digest : {&member.source.geometry, &member.source.recipe,
                     &member.source.placement, &member.source.material, &member.source.groups})
                if (!reader.raw(*digest)) return false;
        }
        value.charts.resize(std::size_t(chartCount));
        for (auto& chart : value.charts) {
            std::uint64_t triangles = 0;
            if (!reader.raw(chart.chart) || !reader.raw(chart.member) || !reader.raw(chart.material)
                || !reader.integer(4, triangles) || triangles == 0
                || triangles > kMaximumFinalCorners / 3
                || !reader.real(chart.developedAreaMM2)) return false;
            chart.triangleCount = std::uint32_t(triangles);
            for (double& bound : chart.rectUV)
                if (!reader.real(bound)) return false;
        }
        value.resources.resize(std::size_t(resourceCount));
        for (auto& resource : value.resources)
            if (!reader.raw(resource.identity) || !reader.raw(resource.content)) return false;
        if (!reader.raw(value.layoutProof)) return false;
        std::vector<std::uint8_t> canonical; Refusal validation;
        if (!reader.complete() || !Valid(value, validation) || !Encode(value, canonical)
            || canonical != bytes) return false;
        output = std::move(value); refusal = Refusal::None; return true;
    } catch (...) { output = {}; return false; }
}

struct Transaction final {
    Key key;
    std::vector<Member> captured;
    bool prepared = false;

    static bool SameMembers(const std::vector<Member>& fenced,
                            const std::vector<Member>& observed) noexcept {
        try {
            if (fenced.size() != observed.size()) return false;
            std::vector<bool> matched(fenced.size(), false);
            for (const auto& member : observed) {
                bool found = false;
                for (std::size_t index = 0; index < fenced.size(); ++index)
                    if (!matched[index] && fenced[index] == member) {
                        matched[index] = true; found = true; break;
                    }
                if (!found) return false;
            }
            return true;
        } catch (...) { return false; }
    }
    Refusal Prepare(const Definition& candidate, const Key& resolvedKey,
                    const std::vector<Member>& observed) noexcept {
        Refusal refusal;
        if (prepared) return Refusal::TransactionBusy;
        if (!Valid(candidate, refusal)) return refusal;
        if (!(candidate.key == resolvedKey)) return Refusal::OwnerMismatch;
        if (!Current(candidate, observed)) return Refusal::StaleSource;
        key = resolvedKey; captured = observed; prepared = true; return Refusal::None;
    }
    Refusal Commit(const Key& resolvedKey, const std::vector<Member>& observed) noexcept {
        if (!prepared) return Refusal::Malformed;
        prepared = false;
        if (!(key == resolvedKey)) return Refusal::OwnerMismatch;
        if (!SameMembers(captured, observed)) return Refusal::StaleSource;
        return Refusal::None;
    }
    void Cancel() noexcept { prepared = false; }
};

} // namespace core3d::asset_atlas
