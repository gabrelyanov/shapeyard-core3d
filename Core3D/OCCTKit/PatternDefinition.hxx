#pragma once

// Durable, associative pattern intent. Independent copies are deliberately not
// represented by this type: callers that do not opt in retain the legacy array
// behavior, and an eventual explicit Detach operation must remove this record.
#include "RetainedRecipeIdentity.hxx"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <set>
#include <utility>
#include <vector>

namespace core3d::pattern {
using retained_recipe::IssuanceState;
using retained_recipe::OwnerKey;
using retained_recipe::SourceIdentity;
using retained_recipe::UUID;

inline constexpr std::uint32_t Schema = 1;
inline constexpr std::uint32_t MaximumInstances = 256;
inline constexpr std::size_t MaximumEnvelopeBytes = 128 * 1024;
inline constexpr std::size_t MaximumDocumentPatternBytes = 8 * 1024 * 1024;
inline constexpr double TwoPi = 6.283185307179586476925286766559;

enum class Kind : std::uint8_t { Linear = 1, Radial = 2, Grid = 3 };
enum class Axis : std::uint8_t { X = 0, Y = 1, Z = 2 };
enum class MemberState : std::uint8_t { Active = 1, Suppressed = 2, Removed = 3 };

struct Coordinate {
    std::uint32_t row = 0;
    std::uint32_t column = 0;
    bool operator==(const Coordinate& other) const noexcept {
        return row == other.row && column == other.column;
    }
    bool operator<(const Coordinate& other) const noexcept {
        return row < other.row || (row == other.row && column < other.column);
    }
};

struct Member {
    UUID identity{};
    std::uint64_t localID = 0;
    Coordinate coordinate;
    MemberState state = MemberState::Active;
};

struct Definition {
    std::uint32_t schemaVersion = Schema;
    OwnerKey owner;
    UUID feature{};
    SourceIdentity source;
    Kind kind = Kind::Linear;
    Axis columnAxis = Axis::X;
    Axis rowAxis = Axis::Y;
    std::uint32_t rowCount = 1;
    std::uint32_t columnCount = 2;
    double rowSpacing = 0;
    double columnSpacing = 1;
    double sweepRadians = TwoPi;
    std::array<double, 3> radialPivotLocal{{0, 0, 0}};
    // Row-major affine source frame. Translation occupies indices 3, 7, 11.
    std::array<double, 16> sourceFrame{{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}};
    IssuanceState issuance;
    std::vector<Member> members;
    std::vector<Member> removals;
};

struct AdmissionBudget {
    std::size_t existingDocumentBytes = 0;
    std::size_t documentLimitBytes = MaximumDocumentPatternBytes;
    std::size_t existingMemoryBytes = 0;
    std::size_t memoryLimitBytes = 256 * 1024 * 1024;
    std::size_t sourceDocumentBytes = 0;
    std::size_t sourceMemoryBytes = 0;
    std::size_t perMemberMemoryOverhead = 512;
    std::uint32_t maximumInstances = MaximumInstances;
};

struct Projection {
    std::uint32_t instances = 0;
    std::size_t projectedDocumentBytes = 0;
    std::size_t projectedMemoryBytes = 0;
    bool admitted = false;
};

using IssueUUID = std::function<bool(UUID&)>;

inline bool IsFiniteFrame(const std::array<double, 16>& frame) noexcept {
    for (double value : frame) if (!std::isfinite(value)) return false;
    if (frame[12] != 0 || frame[13] != 0 || frame[14] != 0 || frame[15] != 1) return false;
    const auto dot = [&](unsigned a, unsigned b) {
        return frame[a] * frame[b] + frame[4 + a] * frame[4 + b]
            + frame[8 + a] * frame[8 + b];
    };
    const double scale2 = dot(0, 0);
    const double tolerance = std::max(1.0, scale2) * 1e-12;
    if (!std::isfinite(scale2) || scale2 <= 0
        || std::abs(dot(1, 1) - scale2) > tolerance
        || std::abs(dot(2, 2) - scale2) > tolerance
        || std::abs(dot(0, 1)) > tolerance
        || std::abs(dot(0, 2)) > tolerance
        || std::abs(dot(1, 2)) > tolerance) return false;
    const double determinant =
        frame[0] * (frame[5] * frame[10] - frame[6] * frame[9])
      - frame[1] * (frame[4] * frame[10] - frame[6] * frame[8])
      + frame[2] * (frame[4] * frame[9] - frame[5] * frame[8]);
    return std::isfinite(determinant) && determinant > 0;
}

inline bool Product(std::uint32_t rows, std::uint32_t columns,
                    std::uint32_t& output) noexcept {
    const std::uint64_t product = std::uint64_t(rows) * columns;
    if (rows == 0 || columns == 0 || product > UINT32_MAX) return false;
    output = std::uint32_t(product); return true;
}

inline bool ParametersValid(const Definition& value) noexcept {
    std::uint32_t count = 0;
    if (!Product(value.rowCount, value.columnCount, count)
        || count < 2 || count > MaximumInstances
        || unsigned(value.columnAxis) > unsigned(Axis::Z)
        || unsigned(value.rowAxis) > unsigned(Axis::Z)
        || !std::isfinite(value.rowSpacing) || !std::isfinite(value.columnSpacing)
        || !std::isfinite(value.sweepRadians) || !IsFiniteFrame(value.sourceFrame)) return false;
    for (double pivot : value.radialPivotLocal) if (!std::isfinite(pivot)) return false;
    if (value.kind == Kind::Linear)
        return value.rowCount == 1 && value.columnCount >= 2 && value.columnSpacing != 0;
    if (value.kind == Kind::Radial)
        return value.rowCount == 1 && value.columnCount >= 2
            && value.sweepRadians != 0 && std::abs(value.sweepRadians) <= TwoPi;
    if (value.kind == Kind::Grid)
        return value.columnAxis != value.rowAxis
            && (value.columnCount == 1 || value.columnSpacing != 0)
            && (value.rowCount == 1 || value.rowSpacing != 0);
    return false;
}

inline bool InDomain(const Definition& value, const Coordinate& coordinate) noexcept {
    return coordinate.row < value.rowCount && coordinate.column < value.columnCount;
}

inline bool Valid(const Definition& value) noexcept {
    try {
        if (value.schemaVersion != Schema || !retained_recipe::Valid(value.owner)
            || !retained_recipe::Nonzero(value.feature)
            || !retained_recipe::Valid(value.source)
            || value.source.document != value.owner.document
            || !retained_recipe::Valid(value.issuance) || !ParametersValid(value)) return false;
        std::uint32_t expected = 0; if (!Product(value.rowCount, value.columnCount, expected)) return false;
        if (value.members.size() != expected || value.removals.size() > MaximumInstances * 8u) return false;
        std::set<UUID> identities; std::set<std::uint64_t> localIDs;
        std::set<Coordinate> coordinates; std::set<std::uint64_t> retired(
            value.issuance.retiredLocalIDs.begin(), value.issuance.retiredLocalIDs.end());
        bool foundSource = false;
        for (const Member& member : value.members) {
            if (!retained_recipe::Nonzero(member.identity) || member.localID == 0
                || member.localID >= value.issuance.nextLocalID
                || member.state == MemberState::Removed || !InDomain(value, member.coordinate)
                || !identities.insert(member.identity).second || !localIDs.insert(member.localID).second
                || !coordinates.insert(member.coordinate).second || retired.count(member.localID)) return false;
            if (member.coordinate == Coordinate{}) {
                if (member.identity != value.source.entity || member.state != MemberState::Active) return false;
                foundSource = true;
            }
        }
        for (const Member& member : value.removals) {
            if (!retained_recipe::Nonzero(member.identity) || member.localID == 0
                || member.localID >= value.issuance.nextLocalID || member.state != MemberState::Removed
                || !identities.insert(member.identity).second || !localIDs.insert(member.localID).second
                || !retired.count(member.localID)) return false;
        }
        return foundSource;
    } catch (...) { return false; }
}

inline bool CheckedAdd(std::size_t left, std::size_t right, std::size_t& output) noexcept {
    if (right > std::numeric_limits<std::size_t>::max() - left) return false;
    output = left + right; return true;
}
inline bool CheckedMultiply(std::size_t left, std::size_t right, std::size_t& output) noexcept {
    if (left != 0 && right > std::numeric_limits<std::size_t>::max() / left) return false;
    output = left * right; return true;
}

inline Projection Project(const Definition& value, const AdmissionBudget& budget) noexcept {
    Projection result; std::uint32_t count = 0;
    if (!ParametersValid(value) || !Product(value.rowCount, value.columnCount, count)
        || count > budget.maximumInstances || budget.documentLimitBytes > MaximumDocumentPatternBytes
        || budget.existingDocumentBytes > budget.documentLimitBytes
        || budget.existingMemoryBytes > budget.memoryLimitBytes) return result;
    result.instances = count;
    const std::size_t copies = count - 1;
    std::size_t copyDocument = 0, copyMemory = 0, perMemory = 0;
    const std::size_t recipeEstimate = 512 + (value.members.size() + value.removals.size() + count) * 64;
    if (!CheckedMultiply(copies, budget.sourceDocumentBytes, copyDocument)
        || !CheckedAdd(budget.sourceMemoryBytes, budget.perMemberMemoryOverhead, perMemory)
        || !CheckedMultiply(copies, perMemory, copyMemory)
        || !CheckedAdd(budget.existingDocumentBytes, copyDocument, result.projectedDocumentBytes)
        || !CheckedAdd(result.projectedDocumentBytes, recipeEstimate, result.projectedDocumentBytes)
        || !CheckedAdd(budget.existingMemoryBytes, copyMemory, result.projectedMemoryBytes)) return {};
    result.admitted = result.projectedDocumentBytes <= budget.documentLimitBytes
        && result.projectedMemoryBytes <= budget.memoryLimitBytes;
    return result;
}

inline bool IssueMember(Definition& value, Coordinate coordinate, const IssueUUID& issue,
                        Member& output) noexcept {
    output = {};
    if (!issue || value.issuance.nextLocalID == 0
        || value.issuance.nextLocalID == std::numeric_limits<std::uint64_t>::max()) return false;
    UUID identity{}; if (!issue(identity) || !retained_recipe::Nonzero(identity)) return false;
    for (const Member& member : value.members) if (member.identity == identity) return false;
    for (const Member& member : value.removals) if (member.identity == identity) return false;
    output.identity = identity; output.localID = value.issuance.nextLocalID++;
    output.coordinate = coordinate; output.state = MemberState::Active; return true;
}

inline bool Reconcile(Definition& value, std::uint32_t rows, std::uint32_t columns,
                      const IssueUUID& issue) noexcept {
    try {
        Definition candidate = value; candidate.rowCount = rows; candidate.columnCount = columns;
        if (!ParametersValid(candidate)) return false;
        std::vector<Member> survivors; survivors.reserve(std::size_t(rows) * columns);
        for (Member member : candidate.members) {
            if (InDomain(candidate, member.coordinate)) survivors.push_back(member);
            else {
                member.state = MemberState::Removed;
                candidate.removals.push_back(member);
                const auto position = std::lower_bound(candidate.issuance.retiredLocalIDs.begin(),
                    candidate.issuance.retiredLocalIDs.end(), member.localID);
                if (position != candidate.issuance.retiredLocalIDs.end()
                    && *position == member.localID) return false;
                candidate.issuance.retiredLocalIDs.insert(position, member.localID);
            }
        }
        candidate.members = std::move(survivors);
        for (std::uint32_t row = 0; row < rows; ++row) for (std::uint32_t column = 0; column < columns; ++column) {
            const Coordinate coordinate{row, column};
            if (std::find_if(candidate.members.begin(), candidate.members.end(), [&](const Member& member) {
                    return member.coordinate == coordinate;
                }) != candidate.members.end()) continue;
            Member member;
            if (coordinate == Coordinate{} && candidate.members.empty()) {
                member.identity = candidate.source.entity;
                member.localID = candidate.issuance.nextLocalID++;
                member.coordinate = coordinate; member.state = MemberState::Active;
            } else if (!IssueMember(candidate, coordinate, issue, member)) return false;
            candidate.members.push_back(member);
        }
        std::sort(candidate.members.begin(), candidate.members.end(), [](const Member& a, const Member& b) {
            return a.coordinate < b.coordinate;
        });
        if (!Valid(candidate)) return false; value = std::move(candidate); return true;
    } catch (...) { return false; }
}

inline bool SetSuppressed(Definition& value, Coordinate coordinate, bool suppressed) noexcept {
    if (coordinate == Coordinate{}) return false;
    Definition candidate = value;
    const auto found = std::find_if(candidate.members.begin(), candidate.members.end(),
        [&](const Member& member) { return member.coordinate == coordinate; });
    if (found == candidate.members.end()) return false;
    found->state = suppressed ? MemberState::Suppressed : MemberState::Active;
    if (!Valid(candidate)) return false; value = std::move(candidate); return true;
}
} // namespace core3d::pattern
