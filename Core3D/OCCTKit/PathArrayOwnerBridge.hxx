#pragma once

// D3-N production boundary above the landed SYPA/1 kernel. It consumes the
// C1-N path authority and D2-N label/recipe receipts; it never evaluates a
// copied curve or creates an independent per-member command.
#include "BoundedCurveCodec.hxx"
#include "OcctDocument.h"
#include "PathArrayBuild.hxx"
#include "PathArrayPersistence.hxx"
#include "PatternAllLabelAuthority.hxx"

#include <TDocStd_Document.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace core3d::bounded_curve::owner {
struct PathReceipt;
struct Prepared;
}

namespace core3d::path_array_owner {
using path_array::UUID;

namespace c1_owner = core3d::bounded_curve::owner;

enum class Refusal : std::uint8_t {
    None = 0, ClosedDocument, OpenCommand, CorruptTable, AmbiguousSelection,
    MissingSource, MissingMember, MissingPath, StaleSource, StaleMember,
    StalePath, UnsupportedSource, InvalidCandidate, ArcLengthFailure,
    ZeroTangent, Cusp, FrameFlip, InvalidUpVector, InversionFailure,
    CountBudget, DocumentBudget, MemoryBudget, TopologyBudget, StageFailed,
    ReadbackFailed, CommitUnknown
};

struct PathAuthority final {
    // Opaque because this draft must not edit C1-N's currently truncated
    // header hunk. The accepted C1 implementation owns and validates it.
    std::shared_ptr<const c1_owner::PathReceipt> receipt;
    path_array::CurveReference locator;
    bounded_curve::PersistedValue persisted;
    TDF_Label ownerLabel;

    bool currentFor(const path_array::CurveReference& reference) const noexcept {
        return !ownerLabel.IsNull()
            && receipt
            && locator.owner == reference.owner
            && locator.feature == reference.feature
            && locator.definitionRevision == reference.definitionRevision
            && locator.canonicalDefinitionDigest == reference.canonicalDefinitionDigest
            && path_array::Matches(reference, persisted)
            && path_array::Matches(locator, persisted);
    }
};

//! A C1 edit may prepare D3 replay against its detached candidate, but that
//! candidate is not live authority until C1 commits.  The C1 Prepared object
//! is therefore retained as an unforgeable seal and no PathReceipt is minted.
struct ProspectivePath final {
    std::shared_ptr<const c1_owner::Prepared> seal;
    path_array::CurveReference locator;
    bounded_curve::PersistedValue persisted;
    TDF_Label ownerLabel;

    bool sealedFor(const path_array::CurveReference& reference) const noexcept {
        return seal && !ownerLabel.IsNull()
            && locator.owner == reference.owner
            && locator.feature == reference.feature
            && locator.definitionRevision == reference.definitionRevision
            && locator.canonicalDefinitionDigest == reference.canonicalDefinitionDigest
            && path_array::Matches(reference, persisted)
            && path_array::Matches(locator, persisted);
    }
};

// Implemented by the accepted C1-N owner. The returned PersistedValue is read
// from the same live OCAF label as the receipt; callers cannot supply spline
// arrays or refresh a stale locator themselves.
struct CurrentPathResolver {
    virtual ~CurrentPathResolver() = default;
    virtual Refusal resolveCurrent(const path_array::CurveReference&,
                                   PathAuthority&) noexcept = 0;
};

struct SourceMetrics final {
    std::size_t sourceDocumentBytes = 0;
    std::size_t sourceMemoryBytes = 0;
    Standard_Size sourceTopologyNodes = 0;
    std::size_t patternDocumentBytes = 0;
    std::size_t compositeDocumentBytes = 0;
};

// Generic all-label receipt requested from D2-N. Its accepted implementation
// resolves the actual source and member OCAF labels, entity/definition UUIDs,
// shapes and copied recipe receipts as one opaque snapshot.
struct D2ArrayAuthority {
    virtual ~D2ArrayAuthority() = default;
    virtual Refusal captureCurrent(OcctDocument&,
        const std::vector<path_array::Record>&,
        const std::string& selectedEntity,
        path_array::Record& selectedRecord,
        std::shared_ptr<const pattern_owner::AllLabelSnapshot>&,
        SourceMetrics&) noexcept = 0;
    virtual bool isCurrent(OcctDocument&,
        const std::shared_ptr<const pattern_owner::AllLabelSnapshot>&,
        const path_array::Definition&) noexcept = 0;
};

struct Snapshot final {
    path_array::Record record;
    std::shared_ptr<const pattern_owner::AllLabelSnapshot> labels;
    PathAuthority path;
    SourceMetrics metrics;
    Standard_Integer documentTime = 0;
    std::size_t pathArrayDocumentBytes = 0;

    bool hasCompleteOrdinalAuthority() const noexcept {
        try {
        if (!labels || labels->family != pattern_owner::AllLabelFamily::PathArrayD3
            || labels->featureIdentifier != retained_solid::UUIDText(record.definition.feature)
            || labels->canonicalRecordBytes != record.bytes
            || labels->members.size() != record.definition.members.size()) return false;
        for (std::size_t index = 0; index < labels->members.size(); ++index) {
            const auto* ordinal = std::get_if<pattern_owner::D3Ordinal>(
                &labels->members[index].key);
            const auto& retained = record.definition.members[index];
            if (!ordinal || ordinal->ordinal != index
                || retained.coordinate.row != 0
                || retained.coordinate.column != index
                || labels->members[index].localIdentifier != retained.localID)
                return false;
        }
        return true;
        } catch (...) { return false; }
    }

    bool admitted() const noexcept {
        return !record.label.IsNull() && hasCompleteOrdinalAuthority()
            && path.currentFor(record.definition.path);
    }
};

struct Edit final {
    path_array::DistributionMode distribution = path_array::DistributionMode::Count;
    std::uint32_t count = 3;
    double distance = 1;
    bool includeStart = true;
    bool includeEnd = true;
    bool closedPath = false;
    path_array::OrientationPolicy orientation = path_array::OrientationPolicy::Fixed;
    double rollRadians = 0;
    bool hasUpVector = false;
    std::array<double, 3> upVector{{0, 0, 1}};
    double maximumFrameStepRadians = path_array::Pi * 0.75;
    double arcLengthTolerance = 1e-6;
    double minimumTangent = 1e-10;
    std::set<std::uint32_t> suppressedOrdinals;
    // Populated only by the C1-owned Replace Path picker or by C1's dependent
    // replay while staging a path edit. It is never reconstructed from values.
    std::optional<PathAuthority> replacementPath;
};

struct Limits final {
    std::size_t documentBytes = path_array::MaximumDocumentBytes;
    std::size_t memoryBytes = 256 * 1024 * 1024;
    Standard_Size topologyNodes = 2'000'000;
};

struct PreparedEdit final {
    Snapshot opening;
    PathAuthority candidatePath;
    path_array::Definition candidate;
    path_array::Projection projection;
    std::vector<path_array::Placement> placements;
    path_array::BuildReceipt buildReceipt;
    std::vector<UUID> survivorEntities;
    std::vector<UUID> removedEntities;
    std::shared_ptr<const ProspectivePath> prospectivePath;
    std::shared_ptr<const struct NativeMutation> native;
    Refusal refusal = Refusal::InvalidCandidate;

    bool admitted() const noexcept { return refusal == Refusal::None; }
};

inline Refusal Translate(path_array::BuildRefusal refusal) noexcept {
    switch (refusal) {
        case path_array::BuildRefusal::None: return Refusal::None;
        case path_array::BuildRefusal::StalePath: return Refusal::StalePath;
        case path_array::BuildRefusal::ZeroTangent: return Refusal::ZeroTangent;
        case path_array::BuildRefusal::Cusp: return Refusal::Cusp;
        case path_array::BuildRefusal::FrameFlip: return Refusal::FrameFlip;
        case path_array::BuildRefusal::InvalidUpVector: return Refusal::InvalidUpVector;
        case path_array::BuildRefusal::InversionFailure: return Refusal::InversionFailure;
        case path_array::BuildRefusal::TooFewInstances:
        case path_array::BuildRefusal::TooManyInstances: return Refusal::CountBudget;
        default: return Refusal::ArcLengthFailure;
    }
}

inline Refusal Capture(OcctDocument& owner, const std::string& selectedEntity,
                       D2ArrayAuthority& labels, CurrentPathResolver& paths,
                       Snapshot& output) noexcept {
    output = {};
    try {
        const Handle(TDocStd_Document) document = owner.Document();
        if (document.IsNull() || document->GetData().IsNull()) return Refusal::ClosedDocument;
        if (document->HasOpenCommand()) return Refusal::OpenCommand;
        std::vector<path_array::Record> records;
        if (!path_array::ReadAll(document, records)) return Refusal::CorruptTable;
        for (const auto& record : records) {
            output.pathArrayDocumentBytes += record.bytes.size();
        }
        const Refusal labelResult = labels.captureCurrent(owner, records, selectedEntity,
            output.record, output.labels, output.metrics);
        if (labelResult != Refusal::None || !output.labels) return labelResult;
        const Refusal pathResult = paths.resolveCurrent(output.record.definition.path, output.path);
        if (pathResult != Refusal::None || !output.path.currentFor(output.record.definition.path))
            return pathResult == Refusal::None ? Refusal::StalePath : pathResult;
        output.documentTime = document->GetData()->Time();
        return Refusal::None;
    } catch (...) { output = {}; return Refusal::CorruptTable; }
}

inline PreparedEdit PrepareWithPath(const Snapshot& opening, const Edit& edit,
                            const Limits& limits, const pattern::IssueUUID& issue,
                            const ProspectivePath* prospective) noexcept {
    PreparedEdit result;
    result.opening = opening;
    try {
        if (!opening.admitted()) return result;
        result.candidate = opening.record.definition;
        result.candidatePath = edit.replacementPath.value_or(opening.path);
        if (prospective) {
            result.prospectivePath = std::make_shared<ProspectivePath>(*prospective);
            result.candidatePath = {};
            result.candidatePath.locator = prospective->locator;
            result.candidatePath.persisted = prospective->persisted;
            result.candidatePath.ownerLabel = prospective->ownerLabel;
        }
        result.candidate.path = result.candidatePath.locator;
        if ((!prospective && !result.candidatePath.currentFor(result.candidate.path))
            || (prospective && !prospective->sealedFor(result.candidate.path))) {
            result.refusal = Refusal::StalePath; return result;
        }
        result.candidate.distribution = {edit.distribution, edit.count, edit.distance,
                                        edit.includeStart, edit.includeEnd};
        result.candidate.closedPath = edit.closedPath;
        result.candidate.orientation = {edit.orientation, edit.rollRadians,
            edit.hasUpVector, edit.upVector, edit.maximumFrameStepRadians};
        result.candidate.arcLengthTolerance = edit.arcLengthTolerance;
        result.candidate.minimumTangent = edit.minimumTangent;
        std::uint32_t required = 0;
        double totalLength = 0;
        const auto countResult = path_array::RequiredInstanceCount(result.candidate,
            result.candidatePath.persisted, required, totalLength);
        if (countResult != path_array::BuildRefusal::None) {
            result.refusal = Translate(countResult); return result;
        }
        if (!path_array::ReconcileMembers(result.candidate, required, issue)) return result;
        for (std::uint32_t ordinal = 1; ordinal < required; ++ordinal) {
            const bool suppressed = edit.suppressedOrdinals.count(ordinal) != 0;
            if ((result.candidate.members[ordinal].state == pattern::MemberState::Suppressed)
                    != suppressed
                && !path_array::SetSuppressed(result.candidate, ordinal, suppressed)) return result;
        }
        path_array::AdmissionBudget budget;
        budget.maximumInstances = path_array::MaximumInstances;
        budget.sourceTopologyNodes = opening.metrics.sourceTopologyNodes;
        budget.existingTopologyNodes = 0;
        budget.maximumAggregateTopologyNodes = limits.topologyNodes;
        budget.sourceDocumentBytes = opening.metrics.sourceDocumentBytes;
        budget.existingDocumentBytes = opening.pathArrayDocumentBytes
            + opening.metrics.patternDocumentBytes + opening.metrics.compositeDocumentBytes
            - opening.record.bytes.size();
        budget.maximumDocumentBytes = limits.documentBytes;
        budget.sourceMemoryBytes = opening.metrics.sourceMemoryBytes;
        budget.existingMemoryBytes = 0;
        budget.maximumMemoryBytes = limits.memoryBytes;
        result.projection = path_array::Project(required, budget);
        if (!result.projection.admitted) {
            if (result.projection.aggregateTopologyNodes > limits.topologyNodes)
                result.refusal = Refusal::TopologyBudget;
            else if (result.projection.projectedDocumentBytes > limits.documentBytes)
                result.refusal = Refusal::DocumentBudget;
            else result.refusal = Refusal::MemoryBudget;
            return result;
        }
        const auto build = path_array::BuildPlacements(result.candidate,
            result.candidatePath.persisted, result.placements, result.buildReceipt);
        if (build != path_array::BuildRefusal::None) {
            result.refusal = Translate(build); return result;
        }
        std::set<UUID> before;
        for (const auto& member : opening.record.definition.members) before.insert(member.identity);
        for (const auto& member : result.candidate.members)
            if (before.count(member.identity)) result.survivorEntities.push_back(member.identity);
        for (const auto& removed : result.candidate.removals)
            if (before.count(removed.identity)) result.removedEntities.push_back(removed.identity);
        result.refusal = Refusal::None;
        return result;
    } catch (...) { result.refusal = Refusal::InvalidCandidate; return result; }
}

inline PreparedEdit Prepare(const Snapshot& opening, const Edit& edit,
                            const Limits& limits,
                            const pattern::IssueUUID& issue) noexcept {
    return PrepareWithPath(opening, edit, limits, issue, nullptr);
}

inline PreparedEdit PrepareProspective(const Snapshot& opening, const Edit& edit,
                            const ProspectivePath& path, const Limits& limits,
                            const pattern::IssueUUID& issue) noexcept {
    return PrepareWithPath(opening, edit, limits, issue, &path);
}

// Concrete integration delegates to D2-N's shared all-label copier. It must
// update surviving labels in place, add only freshly issued members, remove
// only retired labels, stage copied current source recipes and SYPA/1, then
// verify every label/entity/definition/local-ID/shape and record byte.
struct Stager {
    virtual ~Stager() = default;
    virtual bool begin(const Snapshot&, const PreparedEdit&) noexcept = 0;
    virtual bool stageAll(const Snapshot&, const PreparedEdit&) noexcept = 0;
    virtual bool readBackAll(const path_array::Definition&) noexcept = 0;
    virtual bool commit() noexcept = 0;
    virtual bool abort() noexcept = 0;
};

inline bool StageInsideOwnedCommand(const Handle(TDocStd_Document)& document,
                                    const PreparedEdit& prepared,
                                    Stager& stager,
                                    path_array::Record& staged) noexcept {
    staged = {};
    return prepared.admitted() && !document.IsNull() && document->HasOpenCommand()
        && stager.stageAll(prepared.opening, prepared)
        && path_array::Stage(document, prepared.candidate, staged)
        && staged.definition.feature == prepared.candidate.feature
        && stager.readBackAll(prepared.candidate);
}

enum class ApplyOutcome : std::uint8_t { Refused = 0, Committed, OutcomeUnknown };

inline ApplyOutcome Apply(OcctDocument& owner, const PreparedEdit& prepared,
                          D2ArrayAuthority& labels, CurrentPathResolver& paths,
                          Stager& stager) noexcept {
    const Handle(TDocStd_Document) document = owner.Document();
    if (!prepared.admitted() || document.IsNull() || document->HasOpenCommand()
        || document->GetData()->Time() != prepared.opening.documentTime)
        return ApplyOutcome::Refused;
    path_array::Record currentRecord;
    PathAuthority currentPath, currentCandidatePath;
    if (!labels.isCurrent(owner, prepared.opening.labels,
            prepared.opening.record.definition)
        || !path_array::ReadFeature(document, prepared.opening.record.definition.feature,
            currentRecord)
        || currentRecord.label.IsNull()
        || currentRecord.bytes != prepared.opening.record.bytes
        || paths.resolveCurrent(prepared.opening.record.definition.path, currentPath)
            != Refusal::None
        || !currentPath.currentFor(prepared.opening.record.definition.path)
        || prepared.prospectivePath
        || paths.resolveCurrent(prepared.candidate.path, currentCandidatePath)
            != Refusal::None
        || !currentCandidatePath.currentFor(prepared.candidate.path)
        || !currentCandidatePath.ownerLabel.IsEqual(prepared.candidatePath.ownerLabel)
        || currentCandidatePath.persisted.ownerState.canonicalDefinitionDigest
            != prepared.candidatePath.persisted.ownerState.canonicalDefinitionDigest)
        return ApplyOutcome::Refused;
    if (!stager.begin(prepared.opening, prepared)
        || !document->HasOpenCommand()) return ApplyOutcome::Refused;
    const auto abort = [&]() {
        const bool closed = stager.abort();
        return !closed || document->HasOpenCommand()
            ? ApplyOutcome::OutcomeUnknown : ApplyOutcome::Refused;
    };
    try {
        path_array::Record staged;
        if (!StageInsideOwnedCommand(document, prepared, stager, staged)) return abort();
        const bool reported = stager.commit();
        if (!reported || document->HasOpenCommand()) return ApplyOutcome::OutcomeUnknown;
        path_array::Record readback;
        if (!path_array::ReadFeature(document, prepared.candidate.feature, readback)
            || readback.bytes != staged.bytes) return ApplyOutcome::OutcomeUnknown;
        return ApplyOutcome::Committed;
    } catch (...) { return abort(); }
}


//! Concrete production collaborators. These functions retain all native
//! implementation types in PathArrayNativeCollaborators.mm.
Refusal CaptureNative(OcctDocument&, const std::string& selectedEntity,
    const std::shared_ptr<native_opening::Context>&, Snapshot&) noexcept;
//! Re-reads the exact D3 record/labels and mints the replacement solely through
//! C1's current-path picker. No caller-supplied locator or receipt is accepted.
Refusal RefreshPathNative(OcctDocument&, const Snapshot&,
    const std::string& selectedPathEntity,
    const std::shared_ptr<native_opening::Context>&,
    Snapshot& refreshed, PathAuthority& replacement) noexcept;
PreparedEdit PrepareNative(OcctDocument&, const Snapshot&, const Edit&,
    const Limits&) noexcept;
ApplyOutcome ApplyNative(OcctDocument&, const PreparedEdit&,
    const std::shared_ptr<native_opening::Context>&) noexcept;

#if DEBUG
//! Row-265 lifecycle probe seam. Runs a genuine replacement-path apply session
//! for an already factory-captured opening: the replacement authority is
//! minted by the production LivePathResolver (never reconstructed from
//! values), then the same PrepareNative/ApplyNative pair the ordinary opening
//! drives runs against the same real lease boundary. A foreign or stale
//! replacement reference refuses at the real resolver.
ApplyOutcome ApplyReplacementPathForDebugProbe(
    OcctDocument&, const Snapshot&,
    const path_array::CurveReference& replacement, const Edit&,
    const std::shared_ptr<native_opening::Context>&) noexcept;
#endif

struct DependentReplayPlan final {
    std::shared_ptr<const c1_owner::Prepared> c1Seal;
    std::vector<PreparedEdit> arrays;
    bool admitted = false;
};

//! Enumerates every tag-72 record that references the edited C1 owner and
//! prepares all geometry before C1 acquires its one command lease.
bool PrepareDependentReplay(OcctDocument&,
    const std::shared_ptr<native_opening::Context>&,
    const std::shared_ptr<const c1_owner::Prepared>&,
    DependentReplayPlan&) noexcept;

//! Called only after the C1 replacement has staged in the caller-owned lease.
//! It uses StageInsideOwnedCommand directly and never calls Apply.
bool StageDependentReplayInsideOwnedCommand(OcctDocument&,
    native_opening::CommandLease&, const DependentReplayPlan&) noexcept;

//! Post-close exact record/label/recipe proof for every replayed array.
bool VerifyDependentReplayAfterCommit(OcctDocument&,
    const DependentReplayPlan&) noexcept;
} // namespace core3d::path_array_owner
