#pragma once

// Retained pattern-of-cut-features intent. This is deliberately separate from
// RetainedBooleanProgram: the SYRS codec and its four-step / 32-section limits
// remain the legacy analytic cutHoleRing authority and are never overloaded.
#include "PatternDefinition.hxx"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <vector>

namespace core3d::feature_pattern {
using retained_recipe::OwnerKey;
using retained_recipe::SourceIdentity;
using retained_recipe::UUID;

inline constexpr std::uint32_t Schema = 1;
inline constexpr std::uint32_t MaximumGeneratedFeatures = 32;
inline constexpr std::size_t MaximumEnvelopeBytes = 128 * 1024;
inline constexpr std::size_t MaximumDocumentBytes = 8 * 1024 * 1024;

enum class HostSurface : std::uint8_t { Planar = 1, Curved = 2 };
enum class Refusal : std::uint8_t {
    None = 0,
    InvalidDefinition,
    MissingHostRecipe,
    MissingSourceCutFeature,
    CurvedHost,
    ExpansionBudget,
    GeneratedToolDoesNotCutHost,
    GeneratedToolsOverlap,
    InsufficientHostLigament,
    ResultBoundaryMismatch,
    RemovedVolumeMismatch,
    MissingGeneratedFeature,
    PlacementMismatch
};

struct Definition {
    std::uint32_t schemaVersion = Schema;
    OwnerKey host;
    UUID feature{};
    SourceIdentity sourceCut;
    // Stable retained-Boolean operand/feature step, never an ordinal in an
    // expanded ring. General feature patterns require one existing cut step.
    std::uint64_t sourceCutStepID = 0;
    double metersPerUnit = 0;
    double minimumHostLigamentMM = .002;
    std::uint32_t expectedBoundarySectionsPerFeature = 2;
    // D2 owns the linear/radial/grid law and monotonic member issuance.
    pattern::Definition distribution;
};

inline bool ActiveCount(const Definition& value, std::uint32_t& output) noexcept {
    output = 0;
    try {
        for (const pattern::Member& member : value.distribution.members)
            if (member.state == pattern::MemberState::Active) {
                if (output == std::numeric_limits<std::uint32_t>::max()) return false;
                ++output;
            }
        return true;
    } catch (...) { output = 0; return false; }
}

inline UUID ChildFeatureID(const Definition& value,
                           const pattern::Member& member) noexcept {
    // Coordinate zero is the retained source cut itself. Every other child
    // uses D2's durable member UUID as its feature UUID.
    return member.coordinate == pattern::Coordinate{}
        ? value.sourceCut.sourceFeature : member.identity;
}

inline bool Valid(const Definition& value) noexcept {
    try {
        if (value.schemaVersion != Schema || !retained_recipe::Valid(value.host)
            || !retained_recipe::Nonzero(value.feature)
            || !retained_recipe::Valid(value.sourceCut)
            || value.sourceCut.document != value.host.document
            || value.sourceCutStepID == 0
            || !std::isfinite(value.metersPerUnit) || value.metersPerUnit <= 0
            || !std::isfinite(value.minimumHostLigamentMM)
            || value.minimumHostLigamentMM < .002 || value.minimumHostLigamentMM > 1e6
            || value.expectedBoundarySectionsPerFeature == 0
            || value.expectedBoundarySectionsPerFeature > 64
            || !pattern::Valid(value.distribution)
            || !(value.distribution.owner == value.host)
            || value.distribution.feature != value.feature
            || !(value.distribution.source == value.sourceCut)
            || value.distribution.members.size() > MaximumGeneratedFeatures) return false;
        std::uint32_t active = 0;
        if (!ActiveCount(value, active) || active < 2
            || active > MaximumGeneratedFeatures) return false;
        std::set<UUID> childFeatures;
        for (const pattern::Member& member : value.distribution.members) {
            if (member.state != pattern::MemberState::Active) continue;
            const UUID child = ChildFeatureID(value, member);
            if (!retained_recipe::Nonzero(child)
                || !childFeatures.insert(child).second) return false;
        }
        return childFeatures.size() == active;
    } catch (...) { return false; }
}

struct ExpansionBudget {
    std::size_t existingDocumentBytes = 0;
    std::size_t documentLimitBytes = MaximumDocumentBytes;
    std::size_t existingMemoryBytes = 0;
    std::size_t memoryLimitBytes = 256 * 1024 * 1024;
    std::size_t hostTopologyNodes = 0;
    std::size_t sourceToolTopologyNodes = 0;
    std::size_t maximumTopologyNodes = 65536;
    std::size_t sourceRecipeBytes = 0;
    std::size_t perChildMemoryBytes = 4096;
};

struct Projection {
    std::uint32_t activeFeatures = 0;
    std::size_t projectedTopologyNodes = 0;
    std::size_t projectedDocumentBytes = 0;
    std::size_t projectedMemoryBytes = 0;
    bool admitted = false;
};

inline bool CheckedAdd(std::size_t left, std::size_t right,
                       std::size_t& output) noexcept {
    if (right > std::numeric_limits<std::size_t>::max() - left) return false;
    output = left + right; return true;
}

inline bool CheckedMultiply(std::size_t left, std::size_t right,
                            std::size_t& output) noexcept {
    if (left != 0 && right > std::numeric_limits<std::size_t>::max() / left) return false;
    output = left * right; return true;
}

inline Projection Project(const Definition& value,
                          const ExpansionBudget& budget) noexcept {
    Projection output; std::uint32_t active = 0;
    if (!Valid(value) || !ActiveCount(value, active)
        || budget.documentLimitBytes > MaximumDocumentBytes
        || budget.existingDocumentBytes > budget.documentLimitBytes
        || budget.existingMemoryBytes > budget.memoryLimitBytes
        || budget.hostTopologyNodes > budget.maximumTopologyNodes) return output;
    output.activeFeatures = active;
    std::size_t toolTopology = 0, recipeCopies = 0, childMemory = 0;
    if (!CheckedMultiply(active, budget.sourceToolTopologyNodes, toolTopology)
        || !CheckedAdd(budget.hostTopologyNodes, toolTopology, output.projectedTopologyNodes)
        || !CheckedMultiply(active - 1, budget.sourceRecipeBytes, recipeCopies)
        || !CheckedAdd(budget.existingDocumentBytes, recipeCopies, output.projectedDocumentBytes)
        || !CheckedAdd(output.projectedDocumentBytes, 1024 + active * 96,
                       output.projectedDocumentBytes)
        || !CheckedMultiply(active, budget.perChildMemoryBytes, childMemory)
        || !CheckedAdd(budget.existingMemoryBytes, childMemory,
                       output.projectedMemoryBytes)) return {};
    output.admitted = output.projectedTopologyNodes <= budget.maximumTopologyNodes
        && output.projectedDocumentBytes <= budget.documentLimitBytes
        && output.projectedMemoryBytes <= budget.memoryLimitBytes;
    return output;
}

// Count correspondence is D2 coordinate correspondence: surviving (row,
// column) members keep UUID/local ID; out-of-domain members are permanently
// retired; later expansion issues fresh IDs. Host/source edits never call this
// function and therefore cannot churn any child feature identity.
inline bool ReconcileCounts(Definition& value, std::uint32_t rows,
                            std::uint32_t columns,
                            const pattern::IssueUUID& issue) noexcept {
    try {
        const std::uint64_t count = std::uint64_t(rows) * columns;
        if (count < 2 || count > MaximumGeneratedFeatures) return false;
        Definition candidate = value;
        if (!pattern::Reconcile(candidate.distribution, rows, columns, issue)
            || !Valid(candidate)) return false;
        value = std::move(candidate); return true;
    } catch (...) { return false; }
}
} // namespace core3d::feature_pattern
