#pragma once

#include "BoundedCurveCodec.hxx"
#include "PatternDefinition.hxx"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <vector>

namespace core3d::path_array {
using retained_recipe::Digest;
using retained_recipe::IssuanceState;
using retained_recipe::OwnerKey;
using retained_recipe::SourceIdentity;
using retained_recipe::UUID;
using pattern::IssueUUID;
using pattern::Member;
using pattern::MemberState;

inline constexpr std::uint32_t Schema = 1;
inline constexpr std::uint32_t MaximumInstances = pattern::MaximumInstances;
inline constexpr std::size_t MaximumEnvelopeBytes = 128 * 1024;
inline constexpr std::size_t MaximumDocumentBytes = 8 * 1024 * 1024;
inline constexpr double Pi = 3.1415926535897932384626433832795;

enum class DistributionMode : std::uint8_t { Count = 1, Distance = 2 };
enum class OrientationPolicy : std::uint8_t { Fixed = 1, Tangent = 2, Bishop = 3 };

struct CurveReference {
    OwnerKey owner;
    UUID feature{};
    std::uint64_t definitionRevision = 0;
    Digest canonicalDefinitionDigest{};
};

struct DistributionLaw {
    DistributionMode mode = DistributionMode::Count;
    std::uint32_t count = 3;
    double distance = 1;
    bool includeStart = true;
    bool includeEnd = true;
};

struct OrientationLaw {
    OrientationPolicy policy = OrientationPolicy::Fixed;
    double rollRadians = 0;
    bool hasUpVector = false;
    std::array<double, 3> upVector{{0, 0, 1}};
    // Refuses sparse samples and curve corners that would produce an abrupt
    // occurrence frame. This is persisted and exposed by the native editor.
    double maximumFrameStepRadians = Pi * 0.75;
};

struct Definition {
    std::uint32_t schemaVersion = Schema;
    OwnerKey owner;
    UUID feature{};
    SourceIdentity source;
    CurveReference path;
    DistributionLaw distribution;
    OrientationLaw orientation;
    bool closedPath = false;
    double arcLengthTolerance = 1e-6;
    double minimumTangent = 1e-10;
    // Proper-rigid source occurrence frame, row-major with translation at
    // 3/7/11. It supplies Fixed orientation and Bishop's fallback up vector.
    std::array<double, 16> sourceFrame{{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}};
    IssuanceState issuance;
    std::vector<Member> members;
    std::vector<Member> removals;
};

struct AdmissionBudget {
    std::uint32_t maximumInstances = MaximumInstances;
    std::size_t sourceTopologyNodes = 0;
    std::size_t existingTopologyNodes = 0;
    std::size_t maximumAggregateTopologyNodes = 2'000'000;
    std::size_t sourceDocumentBytes = 0;
    std::size_t existingDocumentBytes = 0;
    std::size_t maximumDocumentBytes = MaximumDocumentBytes;
    std::size_t sourceMemoryBytes = 0;
    std::size_t existingMemoryBytes = 0;
    std::size_t maximumMemoryBytes = 256 * 1024 * 1024;
    std::size_t perInstanceMemoryOverhead = 512;
};

struct Projection {
    std::uint32_t instances = 0;
    std::size_t aggregateTopologyNodes = 0;
    std::size_t projectedDocumentBytes = 0;
    std::size_t projectedMemoryBytes = 0;
    bool admitted = false;
};

inline bool ValidCurveReference(const CurveReference& value) noexcept {
    return retained_recipe::Valid(value.owner)
        && retained_recipe::Nonzero(value.feature)
        && value.definitionRevision != 0
        && retained_recipe::Nonzero(value.canonicalDefinitionDigest);
}

inline bool Matches(const CurveReference& reference,
                    const bounded_curve::PersistedValue& path) noexcept {
    return bounded_curve::ValidatePersisted(path)
        && path.value.definition.domain == bounded_curve::Domain::Path3D
        && reference.owner == path.ownerState.owner
        && reference.feature == path.value.feature
        && reference.feature == path.ownerState.feature
        && reference.definitionRevision == path.ownerState.definitionRevision
        && reference.canonicalDefinitionDigest
            == path.ownerState.canonicalDefinitionDigest;
}

inline bool ValidLaw(const DistributionLaw& value) noexcept {
    if (value.count < 2 || value.count > MaximumInstances
        || !std::isfinite(value.distance)) return false;
    if (value.mode == DistributionMode::Count)
        return true;
    return value.mode == DistributionMode::Distance
        && value.distance > 0;
}

inline bool ValidOrientation(const OrientationLaw& value) noexcept {
    if (value.policy != OrientationPolicy::Fixed
        && value.policy != OrientationPolicy::Tangent
        && value.policy != OrientationPolicy::Bishop) return false;
    if (!std::isfinite(value.rollRadians) || std::abs(value.rollRadians) > 8 * Pi
        || !std::isfinite(value.maximumFrameStepRadians)
        || value.maximumFrameStepRadians <= 0
        || value.maximumFrameStepRadians >= Pi) return false;
    double length2 = 0;
    for (double scalar : value.upVector) {
        if (!std::isfinite(scalar)) return false;
        length2 += scalar * scalar;
    }
    return !value.hasUpVector || length2 > 1e-20;
}

inline bool Valid(const Definition& value) noexcept {
    try {
        if (value.schemaVersion != Schema || !retained_recipe::Valid(value.owner)
            || !retained_recipe::Nonzero(value.feature)
            || !retained_recipe::Valid(value.source)
            || value.source.document != value.owner.document
            || !ValidCurveReference(value.path)
            || value.path.owner.document != value.owner.document
            || !ValidLaw(value.distribution) || !ValidOrientation(value.orientation)
            || !std::isfinite(value.arcLengthTolerance) || value.arcLengthTolerance <= 0
            || value.arcLengthTolerance > 1
            || !std::isfinite(value.minimumTangent) || value.minimumTangent <= 0
            || !pattern::IsFiniteFrame(value.sourceFrame)
            || !retained_recipe::Valid(value.issuance)
            || value.members.size() < 2 || value.members.size() > MaximumInstances
            || value.removals.size() > MaximumInstances * 8u) return false;
        for (unsigned column = 0; column < 3; ++column) {
            const double length2 = value.sourceFrame[column] * value.sourceFrame[column]
                + value.sourceFrame[4 + column] * value.sourceFrame[4 + column]
                + value.sourceFrame[8 + column] * value.sourceFrame[8 + column];
            if (std::abs(length2 - 1) > 1e-12) return false;
        }
        std::set<UUID> identities; std::set<std::uint64_t> localIDs;
        std::set<std::uint64_t> retired(value.issuance.retiredLocalIDs.begin(),
                                        value.issuance.retiredLocalIDs.end());
        for (std::size_t index = 0; index < value.members.size(); ++index) {
            const Member& member = value.members[index];
            if (!retained_recipe::Nonzero(member.identity) || member.localID == 0
                || member.localID >= value.issuance.nextLocalID
                || member.state == MemberState::Removed || member.coordinate.row != 0
                || member.coordinate.column != index
                || !identities.insert(member.identity).second
                || !localIDs.insert(member.localID).second || retired.count(member.localID)) return false;
            if (index == 0 && (member.identity != value.source.entity
                              || member.state != MemberState::Active)) return false;
        }
        for (const Member& member : value.removals) {
            if (!retained_recipe::Nonzero(member.identity) || member.localID == 0
                || member.localID >= value.issuance.nextLocalID
                || member.state != MemberState::Removed || member.coordinate.row != 0
                || !identities.insert(member.identity).second
                || !localIDs.insert(member.localID).second || !retired.count(member.localID)) return false;
        }
        return true;
    } catch (...) { return false; }
}

// Stable correspondence is rank based: surviving sample ordinal i retains its
// identity through path/source edits and count changes. A reduced tail is
// retired permanently; later expansion issues fresh IDs even at old ordinals.
inline bool ReconcileMembers(Definition& value, std::uint32_t count,
                             const IssueUUID& issue) noexcept {
    try {
        if (count < 2 || count > MaximumInstances || !issue) return false;
        Definition candidate = value;
        while (candidate.members.size() > count) {
            Member removed = candidate.members.back(); candidate.members.pop_back();
            removed.state = MemberState::Removed; candidate.removals.push_back(removed);
            const auto position = std::lower_bound(candidate.issuance.retiredLocalIDs.begin(),
                candidate.issuance.retiredLocalIDs.end(), removed.localID);
            if (position != candidate.issuance.retiredLocalIDs.end()
                && *position == removed.localID) return false;
            candidate.issuance.retiredLocalIDs.insert(position, removed.localID);
        }
        while (candidate.members.size() < count) {
            Member member; member.coordinate = {0, std::uint32_t(candidate.members.size())};
            member.state = MemberState::Active;
            if (candidate.members.empty()) {
                if (candidate.issuance.nextLocalID == 0
                    || candidate.issuance.nextLocalID == std::numeric_limits<std::uint64_t>::max()) return false;
                member.identity = candidate.source.entity;
                member.localID = candidate.issuance.nextLocalID++;
            } else {
                UUID identity{};
                if (candidate.issuance.nextLocalID == 0
                    || candidate.issuance.nextLocalID == std::numeric_limits<std::uint64_t>::max()
                    || !issue(identity) || !retained_recipe::Nonzero(identity)) return false;
                for (const Member& existing : candidate.members) if (existing.identity == identity) return false;
                for (const Member& existing : candidate.removals) if (existing.identity == identity) return false;
                member.identity = identity; member.localID = candidate.issuance.nextLocalID++;
            }
            candidate.members.push_back(member);
        }
        if (!Valid(candidate)) return false;
        value = std::move(candidate); return true;
    } catch (...) { return false; }
}

inline bool SetSuppressed(Definition& value, std::uint32_t ordinal,
                          bool suppressed) noexcept {
    if (ordinal == 0 || ordinal >= value.members.size()) return false;
    Definition candidate = value;
    candidate.members[ordinal].state = suppressed
        ? MemberState::Suppressed : MemberState::Active;
    if (!Valid(candidate)) return false; value = std::move(candidate); return true;
}

inline bool CheckedAdd(std::size_t a, std::size_t b, std::size_t& output) noexcept {
    if (b > std::numeric_limits<std::size_t>::max() - a) return false;
    output = a + b; return true;
}
inline bool CheckedMultiply(std::size_t a, std::size_t b, std::size_t& output) noexcept {
    if (a != 0 && b > std::numeric_limits<std::size_t>::max() / a) return false;
    output = a * b; return true;
}

inline Projection Project(std::uint32_t count, const AdmissionBudget& budget) noexcept {
    Projection result; result.instances = count;
    if (count < 2 || count > MaximumInstances || count > budget.maximumInstances
        || budget.maximumDocumentBytes > MaximumDocumentBytes
        || budget.existingDocumentBytes > budget.maximumDocumentBytes
        || budget.existingMemoryBytes > budget.maximumMemoryBytes
        || budget.existingTopologyNodes > budget.maximumAggregateTopologyNodes) return result;
    std::size_t topology = 0, document = 0, memoryPer = 0, memory = 0;
    if (!CheckedMultiply(count, budget.sourceTopologyNodes, topology)
        || !CheckedAdd(budget.existingTopologyNodes, topology, result.aggregateTopologyNodes)
        || !CheckedMultiply(count - 1, budget.sourceDocumentBytes, document)
        || !CheckedAdd(budget.existingDocumentBytes, document, result.projectedDocumentBytes)
        || !CheckedAdd(result.projectedDocumentBytes, 1024 + std::size_t(count) * 64,
                       result.projectedDocumentBytes)
        || !CheckedAdd(budget.sourceMemoryBytes, budget.perInstanceMemoryOverhead, memoryPer)
        || !CheckedMultiply(count - 1, memoryPer, memory)
        || !CheckedAdd(budget.existingMemoryBytes, memory, result.projectedMemoryBytes)) return {};
    result.admitted = result.aggregateTopologyNodes <= budget.maximumAggregateTopologyNodes
        && result.projectedDocumentBytes <= budget.maximumDocumentBytes
        && result.projectedMemoryBytes <= budget.maximumMemoryBytes;
    return result;
}
} // namespace core3d::path_array
