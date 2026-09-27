#pragma once

// Production owner boundary for the already-landed tag-73 cut-feature-pattern
// kernel. D2 owns distribution/issuance; this file does not redefine it.
#include "FeaturePatternBuild.hxx"
#include "FeaturePatternPersistence.hxx"
#include "PatternOwnerBridge.hxx"
#include "RetainedBooleanProgram.hxx"

#include <algorithm>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace core3d::feature_pattern_owner {
using feature_pattern::UUID;

enum class Refusal : std::uint8_t {
    None = 0,
    ClosedDocument,
    OpenCommand,
    CorruptTable,
    AmbiguousSelection,
    MissingHost,
    MissingSourceCarrier,
    MissingSourceCut,
    StaleHost,
    StaleSource,
    CurvedHost,
    InvalidCandidate,
    GeneratedToolDoesNotCutHost,
    GeneratedToolsOverlap,
    InsufficientHostLigament,
    ExpansionBudget,
    ResultBoundaryMismatch,
    MissingGeneratedFeature,
    WrongGeneratedFeature,
    StageFailed,
    ReadbackFailed,
    CommitUnknown
};

// A child is a feature inside the host result, not a second free shape. Its
// receipt is anchored to the actual current host label and carries independent
// selector/section evidence so bounds or volume alone can never prove it.
struct ChildReceipt {
    TDF_Label hostLabel;
    UUID childFeature{};
    UUID instanceIdentity{};
    std::uint64_t localID = 0;
    pattern::Coordinate coordinate;
    pattern::Matrix worldFrame{};
    std::uint32_t boundarySections = 0;
    std::vector<std::uint8_t> selectorEvidence;
};

struct Snapshot {
    feature_pattern::Record record;
    pattern_owner::LabelReceipt host;
    pattern_owner::LabelReceipt sourceCarrier;
    OcctCylindricalCutProgramSource sourceProgram;
    retained_boolean::Step sourceStep;
    std::vector<ChildReceipt> children;
    std::size_t featurePatternDocumentBytes = 0;
    Standard_Integer documentTime = 0;

    bool admitted() const noexcept {
        return !record.label.IsNull() && !host.label.IsNull()
            && !sourceCarrier.label.IsNull() && !sourceProgram.recipeBytes.empty()
            && !children.empty();
    }
};

struct Edit {
    std::uint64_t sourceCutStepID = 0;
    pattern::Kind kind = pattern::Kind::Linear;
    pattern::Axis rowAxis = pattern::Axis::Y;
    pattern::Axis columnAxis = pattern::Axis::X;
    std::uint32_t rows = 1;
    std::uint32_t columns = 2;
    double rowSpacing = 0;
    double columnSpacing = 1;
    double sweepRadians = pattern::TwoPi;
    std::array<double, 3> radialPivotLocal{{0, 0, 0}};
    std::set<pattern::Coordinate> suppressed;
};

struct Limits {
    std::size_t documentBytes = feature_pattern::MaximumDocumentBytes;
    std::size_t memoryBytes = 256 * 1024 * 1024;
    std::size_t topologyNodes = 65'536;
    std::size_t existingDocumentBytes = 0;
    std::size_t existingMemoryBytes = 0;
    std::size_t hostTopologyNodes = 0;
    std::size_t sourceToolTopologyNodes = 0;
    std::size_t sourceRecipeBytes = 0;
};

// Native geometry owns these receipts. The bridge never reconstructs authority
// from a dictionary or from the persisted UUIDs alone.
struct Observer {
    virtual ~Observer() = default;
    virtual bool planarHost(const pattern_owner::LabelReceipt&) noexcept = 0;
    virtual bool captureChildren(const TDF_Label&, const feature_pattern::Definition&,
                                 std::vector<ChildReceipt>&) noexcept = 0;
};

inline bool SameChild(const ChildReceipt& left,
                      const ChildReceipt& right) noexcept {
    return left.hostLabel.IsEqual(right.hostLabel)
        && left.childFeature == right.childFeature
        && left.instanceIdentity == right.instanceIdentity
        && left.localID == right.localID
        && left.coordinate == right.coordinate
        && feature_pattern::SameFrame(left.worldFrame, right.worldFrame)
        && left.boundarySections == right.boundarySections
        && !left.selectorEvidence.empty()
        && left.selectorEvidence == right.selectorEvidence;
}

inline bool LocateLabels(OcctDocument& owner,
                         const Handle(TDocStd_Document)& document,
                         std::map<UUID, pattern_owner::LabelReceipt>& labels) noexcept {
    labels.clear();
    try {
        const Handle(XCAFDoc_ShapeTool) shapes =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return false;
        TDF_LabelSequence roots;
        shapes->GetFreeShapes(roots);
        for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            pattern_owner::LabelReceipt receipt;
            if (!pattern_owner::ReadReceipt(owner, roots.Value(index), receipt)
                || !labels.emplace(receipt.entity, receipt).second) return false;
        }
        return true;
    } catch (...) {
        labels.clear();
        return false;
    }
}

inline Refusal Capture(OcctDocument& owner, const std::string& selectedEntity,
                       Observer& observer, Snapshot& output) noexcept {
    output = {};
    try {
        const Handle(TDocStd_Document) document = owner.Document();
        if (document.IsNull() || document->GetData().IsNull()) return Refusal::ClosedDocument;
        if (document->HasOpenCommand()) return Refusal::OpenCommand;
        UUID documentID{}, selected{};
        if (!pattern_owner::Parse(owner.DocumentIdentifier(), documentID)
            || !pattern_owner::Parse(selectedEntity, selected)) return Refusal::AmbiguousSelection;

        std::vector<feature_pattern::Record> records;
        if (!feature_pattern::ReadAll(document, records)) return Refusal::CorruptTable;
        std::optional<feature_pattern::Record> match;
        for (const auto& record : records) {
            output.featurePatternDocumentBytes += record.bytes.size();
            if (record.definition.host.entity == selected
                || record.definition.sourceCut.entity == selected) {
                if (match) return Refusal::AmbiguousSelection;
                match = record;
            }
        }
        if (!match || match->definition.host.document != documentID
            || match->definition.sourceCut.document != documentID)
            return Refusal::AmbiguousSelection;

        std::map<UUID, pattern_owner::LabelReceipt> labels;
        if (!LocateLabels(owner, document, labels)) return Refusal::CorruptTable;
        const auto host = labels.find(match->definition.host.entity);
        if (host == labels.end()
            || host->second.definition != match->definition.host.definition)
            return Refusal::MissingHost;
        const auto source = labels.find(match->definition.sourceCut.entity);
        if (source == labels.end()
            || source->second.definition != match->definition.sourceCut.definition)
            return Refusal::MissingSourceCarrier;
        if (!observer.planarHost(host->second)) return Refusal::CurvedHost;

        OcctCylindricalCutProgramSource program;
        if (!owner.CaptureCylindricalCutProgramSource(source->second.label, program))
            return Refusal::MissingSourceCut;
        const auto identities = retained_boolean::Identities(program.recipe);
        if (identities.document != match->definition.sourceCut.document
            || identities.entity != match->definition.sourceCut.entity
            || identities.definition != match->definition.sourceCut.definition
            || identities.sourceFeature != match->definition.sourceCut.sourceFeature)
            return Refusal::StaleSource;
        const auto* retained = std::get_if<retained_boolean::Program>(&program.recipe);
        if (!retained) return Refusal::MissingSourceCut;
        const auto step = std::find_if(retained->steps.begin(), retained->steps.end(),
            [&](const retained_boolean::Step& value) {
                return value.operation == analytic_boolean::Operation::Difference
                    && value.operand.identifier == match->definition.sourceCutStepID;
            });
        if (step == retained->steps.end()) return Refusal::MissingSourceCut;

        std::vector<ChildReceipt> children;
        if (!observer.captureChildren(host->second.label, match->definition, children))
            return Refusal::MissingGeneratedFeature;
        std::uint32_t active = 0;
        if (!feature_pattern::ActiveCount(match->definition, active)
            || children.size() != active) return Refusal::MissingGeneratedFeature;
        std::set<UUID> seen;
        for (const ChildReceipt& child : children) {
            if (!child.hostLabel.IsEqual(host->second.label)
                || child.selectorEvidence.empty()
                || child.boundarySections
                    != match->definition.expectedBoundarySectionsPerFeature
                || !seen.insert(child.childFeature).second)
                return Refusal::WrongGeneratedFeature;
        }

        output.record = *match;
        output.host = host->second;
        output.sourceCarrier = source->second;
        output.sourceProgram = std::move(program);
        output.sourceStep = *step;
        output.children = std::move(children);
        output.documentTime = document->GetData()->Time();
        return Refusal::None;
    } catch (...) {
        output = {};
        return Refusal::CorruptTable;
    }
}

struct RebuildProduct {
    TopoDS_Shape result;
    feature_pattern::AdmissionInput evidence;
    std::vector<ChildReceipt> children;
};

// Detached rebuild must transform the one retained source tool for every D2
// placement and cut the complete current host. It may not mutate OCAF.
struct Rebuilder {
    virtual ~Rebuilder() = default;
    virtual bool rebuild(const Snapshot&, const feature_pattern::Definition&,
                         RebuildProduct&) noexcept = 0;
};

struct PreparedEdit {
    Snapshot opening;
    feature_pattern::Definition candidate;
    feature_pattern::Projection projection;
    feature_pattern::Admission admission;
    RebuildProduct rebuilt;
    Refusal refusal = Refusal::InvalidCandidate;
    bool admitted() const noexcept { return refusal == Refusal::None; }
};

inline Refusal Map(feature_pattern::Refusal refusal) noexcept {
    switch (refusal) {
        case feature_pattern::Refusal::None: return Refusal::None;
        case feature_pattern::Refusal::CurvedHost: return Refusal::CurvedHost;
        case feature_pattern::Refusal::GeneratedToolDoesNotCutHost:
            return Refusal::GeneratedToolDoesNotCutHost;
        case feature_pattern::Refusal::GeneratedToolsOverlap:
            return Refusal::GeneratedToolsOverlap;
        case feature_pattern::Refusal::InsufficientHostLigament:
            return Refusal::InsufficientHostLigament;
        case feature_pattern::Refusal::ExpansionBudget:
            return Refusal::ExpansionBudget;
        case feature_pattern::Refusal::ResultBoundaryMismatch:
        case feature_pattern::Refusal::RemovedVolumeMismatch:
            return Refusal::ResultBoundaryMismatch;
        case feature_pattern::Refusal::MissingGeneratedFeature:
            return Refusal::MissingGeneratedFeature;
        case feature_pattern::Refusal::PlacementMismatch:
            return Refusal::WrongGeneratedFeature;
        default: return Refusal::InvalidCandidate;
    }
}

inline PreparedEdit Prepare(const Snapshot& opening, const Edit& edit,
                            const Limits& limits, const pattern::IssueUUID& issue,
                            Rebuilder& rebuilder) noexcept {
    PreparedEdit output;
    output.opening = opening;
    try {
        if (!opening.admitted() || edit.sourceCutStepID == 0) return output;
        output.candidate = opening.record.definition;
        output.candidate.sourceCutStepID = edit.sourceCutStepID;
        output.candidate.distribution.kind = edit.kind;
        output.candidate.distribution.rowAxis = edit.rowAxis;
        output.candidate.distribution.columnAxis = edit.columnAxis;
        output.candidate.distribution.rowSpacing = edit.rowSpacing;
        output.candidate.distribution.columnSpacing = edit.columnSpacing;
        output.candidate.distribution.sweepRadians = edit.sweepRadians;
        output.candidate.distribution.radialPivotLocal = edit.radialPivotLocal;
        if (!feature_pattern::ReconcileCounts(output.candidate, edit.rows,
                                               edit.columns, issue)) return output;
        for (const auto& member : output.candidate.distribution.members) {
            const bool suppressed = edit.suppressed.count(member.coordinate) != 0;
            if (member.coordinate == pattern::Coordinate{} && suppressed) return output;
            if ((member.state == pattern::MemberState::Suppressed) != suppressed
                && !pattern::SetSuppressed(output.candidate.distribution,
                                           member.coordinate, suppressed)) return output;
        }
        if (!feature_pattern::Valid(output.candidate)) return output;

        const auto* program = std::get_if<retained_boolean::Program>(&opening.sourceProgram.recipe);
        if (!program || std::none_of(program->steps.begin(), program->steps.end(),
                [&](const retained_boolean::Step& value) {
                    return value.operation == analytic_boolean::Operation::Difference
                        && value.operand.identifier == edit.sourceCutStepID;
                })) {
            output.refusal = Refusal::MissingSourceCut;
            return output;
        }
        feature_pattern::ExpansionBudget budget;
        budget.existingDocumentBytes = limits.existingDocumentBytes
            + opening.featurePatternDocumentBytes - opening.record.bytes.size();
        budget.documentLimitBytes = limits.documentBytes;
        budget.existingMemoryBytes = limits.existingMemoryBytes;
        budget.memoryLimitBytes = limits.memoryBytes;
        budget.hostTopologyNodes = limits.hostTopologyNodes;
        budget.sourceToolTopologyNodes = limits.sourceToolTopologyNodes;
        budget.maximumTopologyNodes = limits.topologyNodes;
        budget.sourceRecipeBytes = limits.sourceRecipeBytes;
        output.projection = feature_pattern::Project(output.candidate, budget);
        if (!output.projection.admitted) {
            output.refusal = Refusal::ExpansionBudget;
            return output;
        }
        if (!rebuilder.rebuild(opening, output.candidate, output.rebuilt)
            || output.rebuilt.result.IsNull()) return output;
        output.admission = feature_pattern::Admit(output.candidate,
                                                   output.rebuilt.evidence);
        output.refusal = Map(output.admission.refusal);
        if (output.refusal != Refusal::None) return output;
        if (output.rebuilt.children.size() != output.admission.attribution.size()) {
            output.refusal = Refusal::MissingGeneratedFeature;
            return output;
        }
        for (const auto& attribution : output.admission.attribution) {
            const auto child = std::find_if(output.rebuilt.children.begin(),
                output.rebuilt.children.end(), [&](const ChildReceipt& value) {
                    return value.childFeature == attribution.childFeature
                        && value.instanceIdentity == attribution.instanceIdentity
                        && value.localID == attribution.instanceLocalID
                        && value.coordinate == attribution.coordinate;
                });
            if (child == output.rebuilt.children.end()
                || child->selectorEvidence.empty()) {
                output.refusal = Refusal::WrongGeneratedFeature;
                return output;
            }
        }
        return output;
    } catch (...) {
        output.refusal = Refusal::InvalidCandidate;
        return output;
    }
}

struct Readback {
    pattern_owner::LabelReceipt host;
    pattern_owner::LabelReceipt sourceCarrier;
    std::vector<std::uint8_t> sourceProgramBytes;
    std::vector<ChildReceipt> children;
};

// One callback stages the rebuilt host, the complete retained source program,
// its source carrier receipt, and all feature receipts before tag 73 is staged.
struct Stager {
    virtual ~Stager() = default;
    virtual bool stageAll(const Snapshot&, const PreparedEdit&) noexcept = 0;
    virtual bool readBackAll(const feature_pattern::Definition&,
                             Readback&) noexcept = 0;
};

inline bool ExactReadback(const PreparedEdit& prepared,
                          const Readback& readback) noexcept {
    if (!readback.host.label.IsEqual(prepared.opening.host.label)
        || readback.host.entity != prepared.opening.host.entity
        || readback.host.definition != prepared.opening.host.definition
        || !readback.sourceCarrier.label.IsEqual(prepared.opening.sourceCarrier.label)
        || readback.sourceCarrier.entity != prepared.opening.sourceCarrier.entity
        || readback.sourceCarrier.definition != prepared.opening.sourceCarrier.definition
        || readback.sourceProgramBytes != prepared.opening.sourceProgram.recipeBytes
        || readback.children.size() != prepared.rebuilt.children.size()) return false;
    for (const ChildReceipt& expected : prepared.rebuilt.children) {
        const auto actual = std::find_if(readback.children.begin(), readback.children.end(),
            [&](const ChildReceipt& value) {
                return value.childFeature == expected.childFeature;
            });
        if (actual == readback.children.end() || !SameChild(expected, *actual)) return false;
    }
    return readback.host.shape.IsEqual(prepared.rebuilt.result);
}

enum class ApplyOutcome : std::uint8_t { Refused = 0, Committed, OutcomeUnknown };

inline ApplyOutcome Apply(OcctDocument& owner, const PreparedEdit& prepared,
                          Observer& observer, Stager& stager) noexcept {
    const Handle(TDocStd_Document) document = owner.Document();
    if (!prepared.admitted() || document.IsNull() || document->HasOpenCommand()
        || document->GetData()->Time() != prepared.opening.documentTime)
        return ApplyOutcome::Refused;
    Snapshot current;
    if (Capture(owner, owner.EntityIdentifierForLabel(prepared.opening.host.label),
                observer, current) != Refusal::None
        || current.record.bytes != prepared.opening.record.bytes
        || !current.host.label.IsEqual(prepared.opening.host.label)
        || !current.sourceCarrier.label.IsEqual(prepared.opening.sourceCarrier.label)
        || !current.host.shape.IsEqual(prepared.opening.host.shape)
        || !current.sourceCarrier.shape.IsEqual(prepared.opening.sourceCarrier.shape)
        || current.sourceProgram.recipeBytes != prepared.opening.sourceProgram.recipeBytes
        || current.children.size() != prepared.opening.children.size())
        return ApplyOutcome::Refused;
    for (const ChildReceipt& expected : prepared.opening.children) {
        const auto actual = std::find_if(current.children.begin(), current.children.end(),
            [&](const ChildReceipt& value) {
                return value.childFeature == expected.childFeature;
            });
        if (actual == current.children.end() || !SameChild(expected, *actual))
            return ApplyOutcome::Refused;
    }

    document->NewCommand();
    if (!document->HasOpenCommand()) return ApplyOutcome::Refused;
    const auto abort = [&]() {
        try {
            if (document->HasOpenCommand()) document->AbortCommand();
        } catch (...) {}
        return document->HasOpenCommand()
            ? ApplyOutcome::OutcomeUnknown : ApplyOutcome::Refused;
    };
    try {
        feature_pattern::Record staged;
        Readback readback;
        if (!stager.stageAll(current, prepared)
            || !feature_pattern::Stage(document, prepared.candidate, staged)
            || staged.definition.feature != prepared.candidate.feature
            || !stager.readBackAll(prepared.candidate, readback)
            || !ExactReadback(prepared, readback)) return abort();
        const Standard_Boolean reported = document->CommitCommand();
        if (document->HasOpenCommand()) return ApplyOutcome::OutcomeUnknown;
        feature_pattern::Record persisted;
        if (!reported
            || !feature_pattern::ReadFeature(document, prepared.candidate.feature,
                                             persisted)
            || persisted.bytes != staged.bytes) return ApplyOutcome::OutcomeUnknown;
        return ApplyOutcome::Committed;
    } catch (...) {
        return abort();
    }
}
} // namespace core3d::feature_pattern_owner
