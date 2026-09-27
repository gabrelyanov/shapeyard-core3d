#pragma once

// Production boundary above the D2 value/lattice/codec kernel. This file does
// not redefine PatternDefinition, PatternBuild, or PatternPersistence.
#include "PatternBuild.hxx"
#include "PatternPersistence.hxx"
#include "PatternRecipeClone.hxx"
#include "CompositeRecipeAttribute.hxx"
#include "OcctDocument.h"
#include <Standard_GUID.hxx>

#include <TDF_LabelMap.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <functional>
#include <map>
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
    std::size_t patternDocumentBytes = 0;
    std::size_t compositeDocumentBytes = 0;
    Standard_Integer documentTime = 0;
    bool admitted() const noexcept {
        return !record.label.IsNull() && !source.label.IsNull()
            && members.size() == record.definition.members.size();
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
                output.sourceProfileAndEnclosure)
            || !output.sourceProfileAndEnclosure.object.profile.IsCurrent(document, output.source.label)
            || !output.sourceProfileAndEnclosure.object.enclosure.IsCurrent(document, output.source.label)
            || !pattern_recipe_clone::Capture(document, output.source.label, output.sourceRecipe))
            return Refusal::UnsupportedSource;
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

// The host owns shape copying, D1a profile/enclosure/recipe staging, and label
// presentation. This callback is deliberately one all-label operation, not a
// per-member commit surface.
struct Stager {
    virtual ~Stager() = default;
    virtual bool stageAll(const Snapshot&, const PreparedEdit&) noexcept = 0;
    virtual bool readBackAll(const pattern::Definition&) noexcept = 0;
};

enum class ApplyOutcome : std::uint8_t { Refused = 0, Committed, OutcomeUnknown };

inline ApplyOutcome Apply(OcctDocument& owner, const PreparedEdit& prepared,
                          Stager& stager) noexcept {
    const Handle(TDocStd_Document) document = owner.Document();
    if (!prepared.admitted() || document.IsNull() || document->HasOpenCommand()
        || document->GetData()->Time() != prepared.opening.documentTime) return ApplyOutcome::Refused;
    Snapshot current;
    if (Capture(owner, owner.EntityIdentifierForLabel(prepared.opening.source.label), current)
            != Refusal::None
        || current.record.bytes != prepared.opening.record.bytes
        || current.members.size() != prepared.opening.members.size()) return ApplyOutcome::Refused;
    document->NewCommand();
    if (!document->HasOpenCommand()) return ApplyOutcome::Refused;
    pattern::Record staged;
    const auto abort = [&]() {
        try { if (document->HasOpenCommand()) document->AbortCommand(); } catch (...) {}
        return document->HasOpenCommand() ? ApplyOutcome::OutcomeUnknown : ApplyOutcome::Refused;
    };
    try {
        if (!stager.stageAll(current, prepared)
            || !pattern::Stage(document, prepared.candidate, staged)
            || staged.definition.feature != prepared.candidate.feature
            || !stager.readBackAll(prepared.candidate)) return abort();
        const Standard_Boolean reported = document->CommitCommand();
        if (document->HasOpenCommand()) return ApplyOutcome::OutcomeUnknown;
        pattern::Record readback;
        if (!reported || !pattern::ReadFeature(document, prepared.candidate.feature, readback)
            || readback.bytes != staged.bytes) return ApplyOutcome::OutcomeUnknown;
        return ApplyOutcome::Committed;
    } catch (...) { return abort(); }
}
} // namespace core3d::pattern_owner
