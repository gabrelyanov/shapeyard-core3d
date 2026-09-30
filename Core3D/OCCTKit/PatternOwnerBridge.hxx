#pragma once

// Production boundary above the D2 value/lattice/codec kernel. This file does
// not redefine PatternDefinition, PatternBuild, or PatternPersistence.
#include "PatternBuild.hxx"
#include "PatternPersistence.hxx"
#include "PatternRecipeClone.hxx"
#include "PatternAllLabelAuthority.hxx"
#include "NativeOpeningContext.hxx"
#include "CompositeRecipeAttribute.hxx"
#include "OcctDocument.h"
#include <Standard_GUID.hxx>

#include <TDF_LabelMap.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#if DEBUG
#include <cstdio>
#endif

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace core3d::pattern_owner {
using pattern::UUID;

enum class Refusal : std::uint8_t {
    None = 0, ClosedDocument, OpenCommand, CorruptTable, AmbiguousSelection,
    MissingSource, MissingMember, StaleSource, StaleMember, UnsupportedSource,
    InvalidCandidate, CountBudget, DocumentBudget, MemoryBudget, TopologyBudget,
    StageFailed, ReadbackFailed, CommitUnknown
};

struct LabelReceipt {
    TDF_Label label;
    UUID entity{}, definition{};
    TopoDS_Shape shape;
    bool source = false;
    std::uint64_t localID = 0;
    pattern::Coordinate coordinate;
};

struct Snapshot {
    pattern::Record record;
    LabelReceipt source;
    std::vector<LabelReceipt> members;
    pattern_recipe_clone::Source sourceRecipe;
    OcctObjectNameState sourceProfileAndEnclosure;
    std::shared_ptr<const AllLabelSnapshot> allLabels;
    std::size_t patternDocumentBytes = 0;
    std::size_t compositeDocumentBytes = 0;
    Standard_Integer documentTime = 0;
    bool admitted() const noexcept {
        return !record.label.IsNull() && !source.label.IsNull()
            && allLabels && allLabels->family == AllLabelFamily::PatternD2
            && members.size() == record.definition.members.size()
            && allLabels->members.size() == members.size();
    }
};

struct Edit {
    std::uint32_t rows = 1, columns = 2;
    pattern::Axis rowAxis = pattern::Axis::Y, columnAxis = pattern::Axis::X;
    double rowSpacing = 0, columnSpacing = 1, sweepRadians = pattern::TwoPi;
    std::array<double, 3> radialPivotLocal{{0, 0, 0}};
    std::set<pattern::Coordinate> suppressed;
};

struct Limits {
    std::size_t documentBytes = pattern::MaximumDocumentPatternBytes;
    std::size_t memoryBytes = 256 * 1024 * 1024;
    Standard_Size topologyNodes = 32'768;
    std::size_t sourceDocumentBytes = 0, sourceMemoryBytes = 0;
    Standard_Size sourceTopologyNodes = 0;
};

struct PreparedEdit {
    Snapshot opening;
    pattern::Definition candidate;
    pattern::Projection projection;
    std::vector<pattern::Placement> placements;
    std::vector<LabelReceipt> survivors;
    std::vector<LabelReceipt> removals;
    Standard_Size projectedTopologyNodes = 0;
    std::shared_ptr<const AllLabelMutation> native;
    Refusal refusal = Refusal::InvalidCandidate;
    bool admitted() const noexcept { return refusal == Refusal::None; }
};

inline bool Parse(const std::string& text, UUID& value) noexcept {
    value.fill(0);
    if (text.size() != 36 || !Standard_GUID::CheckGUIDFormat(text.c_str())) return false;
    std::size_t index = 0;
    int high = -1;
    const auto nibble = [](char character) noexcept {
        return character >= '0' && character <= '9' ? character - '0'
             : character >= 'A' && character <= 'F' ? character - 'A' + 10
             : character >= 'a' && character <= 'f' ? character - 'a' + 10 : -1;
    };
    for (char character : text) {
        if (character == '-') continue;
        const int next = nibble(character);
        if (next < 0) return false;
        if (high < 0) high = next;
        else {
            if (index >= value.size()) return false;
            value[index++] = std::uint8_t(high * 16 + next); high = -1;
        }
    }
    return index == value.size() && high < 0 && retained_recipe::Nonzero(value);
}

inline bool ReadReceipt(OcctDocument& owner, const TDF_Label& label,
                        LabelReceipt& output) noexcept {
    output = {};
    try {
        if (label.IsNull() || !XCAFDoc_ShapeTool::IsFree(label)
            || !XCAFDoc_ShapeTool::IsSimpleShape(label)
            || !Parse(owner.EntityIdentifierForLabel(label), output.entity)
            || !Parse(owner.DefinitionIdentifierForLabel(label), output.definition)) return false;
        output.label = label; output.shape = XCAFDoc_ShapeTool::GetShape(label);
        return !output.shape.IsNull();
    } catch (...) { output = {}; return false; }
}

inline std::string RecipeFeatureIdentifier(
    const pattern_recipe_clone::Source& source) noexcept {
    try {
        if (source.family == pattern_recipe_clone::Family::Sweep)
            return source.sweep.identifier;
        if (source.family == pattern_recipe_clone::Family::Loft)
            return source.loft.identifier;
        if (source.family == pattern_recipe_clone::Family::AnalyticBoolean
            && source.analyticBoolean.value) {
            const auto& nodes = source.analyticBoolean.value->definition.nodes;
            if (!nodes.empty()) {
                const auto* feature = std::get_if<composite_recipe::FeatureNode>(
                    &nodes.back().value);
                if (feature) return retained_solid::UUIDText(feature->feature);
            }
        }
    } catch (...) {}
    return {};
}

// Selection may name any surviving member. Every member and the source are
// resolved from actual free OCAF labels; UUIDs in the table are locators only.
inline Refusal Capture(OcctDocument& owner, const std::string& selectedEntity,
                       Snapshot& output) noexcept {
    output = {};
    try {
        const Handle(TDocStd_Document) document = owner.Document();
        if (document.IsNull() || document->GetData().IsNull()) return Refusal::ClosedDocument;
        if (document->HasOpenCommand()) return Refusal::OpenCommand;
        UUID documentID{}, selected{};
        if (!Parse(owner.DocumentIdentifier(), documentID) || !Parse(selectedEntity, selected))
            return Refusal::AmbiguousSelection;
        std::vector<pattern::Record> records;
        if (!pattern::ReadAll(document, records)) return Refusal::CorruptTable;
        std::optional<pattern::Record> match;
        for (const auto& record : records) {
            for (const auto& member : record.definition.members) if (member.identity == selected) {
                if (match) return Refusal::AmbiguousSelection;
                match = record;
            }
            output.patternDocumentBytes += record.bytes.size();
        }
        if (!match || match->definition.owner.document != documentID
            || match->definition.source.document != documentID) return Refusal::AmbiguousSelection;

        TDF_LabelSequence roots;
        const Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return Refusal::ClosedDocument;
        shapes->GetFreeShapes(roots);
        std::map<UUID, LabelReceipt> labels;
        for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            LabelReceipt receipt;
            if (!ReadReceipt(owner, roots.Value(index), receipt)) return Refusal::CorruptTable;
            if (!labels.emplace(receipt.entity, receipt).second) return Refusal::CorruptTable;
        }
        const auto source = labels.find(match->definition.source.entity);
        if (source == labels.end() || source->second.definition != match->definition.source.definition)
            return Refusal::MissingSource;
        output.source = source->second; output.source.source = true;
        output.members.reserve(match->definition.members.size());
        for (const auto& member : match->definition.members) {
            const auto found = labels.find(member.identity);
            if (found == labels.end()) return Refusal::MissingMember;
            LabelReceipt receipt = found->second;
            receipt.localID = member.localID; receipt.coordinate = member.coordinate;
            receipt.source = member.coordinate == pattern::Coordinate{};
            if (receipt.source && !receipt.label.IsEqual(output.source.label)) return Refusal::StaleSource;
            output.members.push_back(std::move(receipt));
        }
        if (!owner.CaptureObjectNameStateForLabel(output.source.label,
                output.sourceProfileAndEnclosure))
            return Refusal::UnsupportedSource;
        // Profile and enclosure records are optional: absence is a valid
        // legacy state. Currency is required only for a record that is
        // actually present; a stale present record still refuses.
        const auto& profileRecord = output.sourceProfileAndEnclosure.object.profile;
        const auto& enclosureRecord = output.sourceProfileAndEnclosure.object.enclosure;
        if ((!profileRecord.label.IsNull()
                && !profileRecord.IsCurrent(document, output.source.label))
            || (!enclosureRecord.label.IsNull()
                && !enclosureRecord.IsCurrent(document, output.source.label))
            || !pattern_recipe_clone::Capture(document, output.source.label, output.sourceRecipe))
            return Refusal::UnsupportedSource;
        auto all = std::make_shared<AllLabelSnapshot>();
        all->documentData = document->GetData();
        all->documentIdentifier = owner.DocumentIdentifier();
        all->family = AllLabelFamily::PatternD2;
        all->recordLabel = match->label;
        all->featureIdentifier = retained_solid::UUIDText(match->definition.feature);
        all->canonicalRecordBytes = match->bytes;
        if (!owner.CaptureExactFreeLabel(output.source.label, all->source))
            return Refusal::StaleSource;
        all->members.reserve(output.members.size());
        for (std::size_t index = 0; index < output.members.size(); ++index) {
            const auto& member = match->definition.members[index];
            AllLabelSnapshot::Member exact;
            exact.key = D2Coordinate{member.coordinate.row, member.coordinate.column};
            exact.localIdentifier = member.localID;
            exact.suppressed = member.state == pattern::MemberState::Suppressed;
            if (!owner.CaptureExactFreeLabel(output.members[index].label, exact.receipt)
                || !pattern_recipe_clone::Capture(
                    document, output.members[index].label, exact.recipe))
                return Refusal::StaleMember;
            exact.featureIdentifier = RecipeFeatureIdentifier(exact.recipe);
            all->members.push_back(std::move(exact));
        }
        output.allLabels = std::move(all);
        std::vector<composite_recipe::Record> composites;
        if (!composite_recipe::ReadAll(document, composites, output.patternDocumentBytes))
            return Refusal::DocumentBudget;
        for (const auto& composite : composites) output.compositeDocumentBytes += composite.value->bytes.size();
        output.record = *match; output.documentTime = document->GetData()->Time();
        return Refusal::None;
    } catch (...) { output = {}; return Refusal::CorruptTable; }
}

inline PreparedEdit Prepare(const Snapshot& opening, const Edit& edit,
                            const Limits& limits, const pattern::IssueUUID& issue) noexcept {
    PreparedEdit result; result.opening = opening;
    try {
        if (!opening.admitted()) return result;
        result.candidate = opening.record.definition;
        result.candidate.rowAxis = edit.rowAxis; result.candidate.columnAxis = edit.columnAxis;
        result.candidate.rowSpacing = edit.rowSpacing;
        result.candidate.columnSpacing = edit.columnSpacing;
        result.candidate.sweepRadians = edit.sweepRadians;
        result.candidate.radialPivotLocal = edit.radialPivotLocal;
        if (!pattern::Reconcile(result.candidate, edit.rows, edit.columns, issue)) {
            result.refusal = Refusal::InvalidCandidate; return result;
        }
        for (const auto& member : result.candidate.members) {
            const bool suppressed = edit.suppressed.count(member.coordinate) != 0;
            if (member.coordinate == pattern::Coordinate{} && suppressed) return result;
            if (member.state == pattern::MemberState::Suppressed != suppressed
                && !pattern::SetSuppressed(result.candidate, member.coordinate, suppressed))
                return result;
        }
        pattern::AdmissionBudget budget;
        budget.existingDocumentBytes = opening.patternDocumentBytes + opening.compositeDocumentBytes
            - opening.record.bytes.size();
        budget.documentLimitBytes = limits.documentBytes;
        budget.existingMemoryBytes = 0; budget.memoryLimitBytes = limits.memoryBytes;
        budget.sourceDocumentBytes = limits.sourceDocumentBytes;
        budget.sourceMemoryBytes = limits.sourceMemoryBytes;
        result.projection = pattern::Project(result.candidate, budget);
        if (!result.projection.admitted) {
            result.refusal = result.projection.projectedDocumentBytes > limits.documentBytes
                ? Refusal::DocumentBudget : Refusal::MemoryBudget; return result;
        }
        if (!pattern::BuildPlacements(result.candidate, result.placements)) return result;
        if (limits.sourceTopologyNodes != 0
            && result.placements.size() > limits.topologyNodes / limits.sourceTopologyNodes) {
            result.refusal = Refusal::TopologyBudget; return result;
        }
        result.projectedTopologyNodes = limits.sourceTopologyNodes * result.placements.size();
        std::map<UUID, LabelReceipt> previous;
        for (const auto& member : opening.members) previous.emplace(member.entity, member);
        for (const auto& member : result.candidate.members) {
            const auto found = previous.find(member.identity);
            if (found != previous.end()) result.survivors.push_back(found->second);
        }
        for (const auto& removed : result.candidate.removals) {
            const auto found = previous.find(removed.identity);
            if (found != previous.end()) result.removals.push_back(found->second);
        }
        result.refusal = Refusal::None; return result;
    } catch (...) { result.refusal = Refusal::InvalidCandidate; return result; }
}

inline bool SameCompleteReadSet(const Snapshot& left,
                                const Snapshot& right) noexcept {
    try {
        if (!left.allLabels || !right.allLabels
            || left.record.bytes != right.record.bytes
            || left.sourceRecipe.family != right.sourceRecipe.family
            || !pattern_recipe_clone::IsEqual(left.sourceRecipe, right.sourceRecipe)
            || !left.allLabels->source.IsEqual(right.allLabels->source)
            || left.allLabels->members.size() != right.allLabels->members.size())
            return false;
        for (std::size_t index = 0; index < left.allLabels->members.size(); ++index) {
            const auto& a = left.allLabels->members[index];
            const auto& b = right.allLabels->members[index];
            const auto* ak = std::get_if<D2Coordinate>(&a.key);
            const auto* bk = std::get_if<D2Coordinate>(&b.key);
            if (!ak || !bk || ak->row != bk->row || ak->column != bk->column
                || a.localIdentifier != b.localIdentifier
                || a.suppressed != b.suppressed
                || a.featureIdentifier != b.featureIdentifier
                || !a.receipt.IsEqual(b.receipt)
                || !pattern_recipe_clone::IsEqual(a.recipe, b.recipe)) return false;
        }
        return true;
    } catch (...) { return false; }
}

enum class ReadPhase : std::uint8_t { InsideCommand = 0, AfterCommit };

// The production implementation owns one native command lease. These methods
// deliberately expose neither per-member commit nor retry.
struct Stager {
    virtual ~Stager() = default;
    virtual bool begin(const Snapshot&, const PreparedEdit&) noexcept = 0;
    virtual bool stageAll(const Snapshot&, const PreparedEdit&) noexcept = 0;
    virtual bool readBackAll(const pattern::Definition&, ReadPhase) noexcept = 0;
    virtual bool commit() noexcept = 0;
    virtual bool abort() noexcept = 0;
};

enum class ApplyOutcome : std::uint8_t { Refused = 0, Committed, OutcomeUnknown };

inline ApplyOutcome Apply(OcctDocument& owner, const PreparedEdit& prepared,
                          Stager& stager) noexcept {
    const Handle(TDocStd_Document) document = owner.Document();
    const auto traceRefusal = [](const char* predicate, bool failed) noexcept {
#if DEBUG
        if (failed) std::fprintf(stderr, "R179_D2_APPLY predicate=%s failed=1\n", predicate);
#else
        (void)predicate;
#endif
        return failed;
    };
    if (traceRefusal("prepared.not-admitted", !prepared.admitted())
        || traceRefusal("document.null", document.IsNull())
        || traceRefusal("document.open-command", document->HasOpenCommand())
        || traceRefusal("document.time-mismatch",
            document->GetData()->Time() != prepared.opening.documentTime)) return ApplyOutcome::Refused;
    Snapshot current;
    if (traceRefusal("Capture.current",
            Capture(owner, owner.EntityIdentifierForLabel(prepared.opening.source.label), current)
                != Refusal::None)
        || traceRefusal("SameCompleteReadSet.current", !SameCompleteReadSet(current, prepared.opening))
        || traceRefusal("stager.begin", !stager.begin(current, prepared))) return ApplyOutcome::Refused;
    const int undoBefore = document->GetAvailableUndos();
    pattern::Record staged;
    const auto abort = [&]() {
        if (!stager.abort()) return ApplyOutcome::OutcomeUnknown;
        Snapshot restored;
        return Capture(owner, owner.EntityIdentifierForLabel(
                    prepared.opening.source.label), restored) == Refusal::None
                && SameCompleteReadSet(restored, prepared.opening)
                && document->GetAvailableUndos() == undoBefore
            ? ApplyOutcome::Refused : ApplyOutcome::OutcomeUnknown;
    };
    try {
        if (traceRefusal("stager.stageAll", !stager.stageAll(current, prepared))
            || traceRefusal("pattern.Stage", !pattern::Stage(document, prepared.candidate, staged))
            || traceRefusal("staged.feature-mismatch", staged.definition.feature != prepared.candidate.feature)
            || traceRefusal("stager.readBackAll.inside-command",
                !stager.readBackAll(prepared.candidate, ReadPhase::InsideCommand))) return abort();
        if (!stager.commit()) return ApplyOutcome::OutcomeUnknown;
        Snapshot after;
        if (document->HasOpenCommand()
            || document->GetAvailableUndos() - undoBefore != 1
            || Capture(owner, owner.EntityIdentifierForLabel(
                    prepared.opening.source.label), after) != Refusal::None
            || after.record.bytes != staged.bytes
            || !stager.readBackAll(prepared.candidate,
                                   ReadPhase::AfterCommit))
            return ApplyOutcome::OutcomeUnknown;
        return ApplyOutcome::Committed;
    } catch (...) {
        traceRefusal("Apply.exception", true);
        return abort();
    }
}

//! Reserves all new label identities, builds every actual clone shape and its
//! retained recipe off-document, and returns a preparation consumable once.
PreparedEdit PrepareNative(OcctDocument&, const Snapshot&, const Edit&,
                           const Limits&) noexcept;

//! Production convenience entry point. A false/uncertain close is reported as
//! OutcomeUnknown exactly once; this function never retries a transaction.
ApplyOutcome ApplyNative(OcctDocument&, const PreparedEdit&,
    const std::shared_ptr<native_opening::Context>&) noexcept;
} // namespace core3d::pattern_owner
