#import <Foundation/Foundation.h>

#include "FeaturePatternProfileContinuation.hxx"
#include "CompositeRecipeCodec.hxx"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TDF_TagSource.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>

namespace core3d::profile_d4 {
namespace {

using UUID = feature_pattern::UUID;

#if DEBUG
std::atomic<CreationFault> gCreationFault{CreationFault::None};

bool ConsumeCreationFault(CreationFault fault) noexcept {
    auto expected = fault;
    return gCreationFault.compare_exchange_strong(
        expected, CreationFault::None, std::memory_order_acq_rel);
}
#endif

bool ScalarRecipe(const profile::Parameters& parameters,
                  std::vector<std::uint8_t>& bytes) noexcept {
    bytes.clear();
    try {
        std::vector<double> values;
        return profile::Encode(parameters, values)
            && composite_recipe::EncodeScalarRecipe(
                composite_recipe::RecipeKind::Profile,
                std::uint32_t(profile::SchemaFor(parameters)), values, bytes);
    } catch (...) { bytes.clear(); return false; }
}

bool Issue(UUID& output) noexcept {
    output = {};
    try {
        for (unsigned attempt = 0; attempt < 16; ++attempt)
            if (pattern_owner::Parse(OcctDocument::NewProfileIdentifier(), output))
                return true;
    } catch (...) {}
    output = {}; return false;
}

double Volume(const TopoDS_Shape& shape) noexcept {
    try {
        if (shape.IsNull()) return 0;
        GProp_GProps properties;
        BRepGProp::VolumeProperties(shape, properties, Standard_True,
                                   Standard_False, Standard_False);
        return properties.Mass();
    } catch (...) { return 0; }
}

bool SameReceipt(const pattern_owner::LabelReceipt& left,
                 const pattern_owner::LabelReceipt& right) noexcept {
    return !left.label.IsNull() && left.label.IsEqual(right.label)
        && left.entity == right.entity && left.definition == right.definition
        && !left.shape.IsNull() && left.shape.IsEqual(right.shape);
}

bool NoExistingRelationship(const Handle(TDocStd_Document)& document,
                            const UUID& host, const UUID& source,
                            std::size_t& bytes) noexcept {
    bytes = 0;
    try {
        std::vector<feature_pattern::Record> records;
        if (!feature_pattern::ReadAll(document, records)) return false;
        for (const auto& record : records) {
            bytes += record.bytes.size();
            const auto& value = record.definition;
            if (value.host.entity == host || value.host.entity == source
                || value.sourceCut.entity == host || value.sourceCut.entity == source)
                return false;
        }
        std::vector<feature_pattern_child::PairedRecord> pairs;
        const auto status = feature_pattern_child::ReadPairs(document, pairs);
        return status == feature_pattern_child::PairStatus::Absent
            || status == feature_pattern_child::PairStatus::Valid;
    } catch (...) { bytes = 0; return false; }
}

bool CaptureInputs(OcctDocument& owner, const std::string& hostText,
                   const std::string& sourceText,
                   UUID& documentID,
                   pattern_owner::LabelReceipt& host,
                   pattern_owner::LabelReceipt& source,
                   profile::Record& profileRecord,
                   std::vector<std::uint8_t>& scalarRecipe,
                   OcctCylindricalCutProgramSource& program,
                   std::size_t& freeCount,
                   std::size_t& featureBytes) noexcept {
    documentID = {}; host = {}; source = {}; profileRecord = {}; scalarRecipe.clear();
    program = {}; freeCount = featureBytes = 0;
    try {
#if DEBUG
        const auto refuse = [](const char *gate) {
            std::fprintf(stderr, "C1H_CREATION_CAPTURE gate=%s\n", gate);
            return false;
        };
#else
        const auto refuse = [](const char *) { return false; };
#endif
        const auto document = owner.Document();
        UUID hostID{}, sourceID{};
        if (document.IsNull() || document->HasOpenCommand()
            || hostText == sourceText
            || !pattern_owner::Parse(owner.DocumentIdentifier(), documentID)
            || !pattern_owner::Parse(hostText, hostID)
            || !pattern_owner::Parse(sourceText, sourceID)) return refuse("identity");
        std::map<UUID, pattern_owner::LabelReceipt> labels;
        if (!feature_pattern_owner::LocateLabels(owner, document, labels))
            return refuse("label-table");
        freeCount = labels.size();
        const auto h = labels.find(hostID), s = labels.find(sourceID);
        if (h == labels.end() || s == labels.end()
            || h->second.label.IsEqual(s->second.label)
            || !NoExistingRelationship(document, hostID, sourceID, featureBytes))
            return refuse("labels-or-existing");
        if (!profile::Read(document, h->second.label, profileRecord)
            || profileRecord.label.IsNull()
            || !profileRecord.IsCurrent(document, h->second.label)
            || !ScalarRecipe(profileRecord.parameters, scalarRecipe)
            || !owner.CaptureCylindricalCutProgramSource(s->second.label, program))
            return refuse("profile-or-program");
        const auto* retained = std::get_if<retained_boolean::Program>(&program.recipe);
        if (!retained || retained->steps.empty()
            || retained->source.entity != s->second.entity
            || retained->source.definition != s->second.definition
            || retained->source.document != documentID
            || retained->source.entity == h->second.entity
            || program.recipeBytes.empty() || program.base.IsNull())
            return refuse("program-binding");
        host = h->second; source = s->second; return true;
    } catch (...) {
        documentID = {}; host = {}; source = {}; profileRecord = {}; scalarRecipe.clear();
        program = {}; freeCount = featureBytes = 0; return false;
    }
}

feature_pattern_owner::Edit ExistingEdit(
    const feature_pattern::Definition& value) {
    feature_pattern_owner::Edit edit;
    edit.sourceCutStepID = value.sourceCutStepID;
    edit.kind = value.distribution.kind;
    edit.rowAxis = value.distribution.rowAxis;
    edit.columnAxis = value.distribution.columnAxis;
    edit.rows = value.distribution.rowCount;
    edit.columns = value.distribution.columnCount;
    edit.rowSpacing = value.distribution.rowSpacing;
    edit.columnSpacing = value.distribution.columnSpacing;
    edit.sweepRadians = value.distribution.sweepRadians;
    edit.radialPivotLocal = value.distribution.radialPivotLocal;
    for (const auto& member : value.distribution.members)
        if (member.state == pattern::MemberState::Suppressed)
            edit.suppressed.insert(member.coordinate);
    return edit;
}

} // namespace

#if DEBUG
void DebugArmCreationFault(CreationFault fault) noexcept {
    gCreationFault.store(fault, std::memory_order_release);
}
#endif

bool CreationReadback(OcctDocument& owner, const PreparedCreation& prepared,
                      const TDF_Label& baselineLabel) noexcept {
    try {
        const auto& capture = *prepared.capture_;
        feature_pattern_child::PairedRecord pair;
        feature_pattern::Record patternRecord;
        std::shared_ptr<const feature_pattern_baseline::Payload> baseline;
        profile::Record profileRecord;
        OcctCylindricalCutProgramSource program;
        std::map<UUID, pattern_owner::LabelReceipt> labels;
        if (!owner.ReadFeaturePatternPair(prepared.definition_.feature, pair)
            || !feature_pattern::ReadFeature(owner.Document(),
                prepared.definition_.feature, patternRecord)
            || patternRecord.definition.feature != prepared.definition_.feature
            || !pair.host.IsEqual(capture.host_.label)
            || !pair.source.IsEqual(capture.source_.label)
            || !pair.baselineRecipe.IsEqual(baselineLabel)
            || pair.children.size() != prepared.built_.childReceipts.size()
            || !feature_pattern_baseline::Read(baselineLabel, baseline)
            || !baseline
            || baseline->envelope.exactRecipe != capture.scalarRecipe_
            || baseline->envelope.retainedRecipeFeature
                != prepared.baseline_.retainedRecipeFeature
            || baseline->envelope.baselineRecipeIdentity
                != prepared.baseline_.baselineRecipeIdentity
            || !profile::Read(owner.Document(), capture.host_.label, profileRecord)
            || profileRecord.identifier != capture.profile_.identifier
            || profileRecord.values != capture.profile_.values
            || !profileRecord.IsCurrent(owner.Document(), capture.host_.label)
            || !owner.CaptureCylindricalCutProgramSource(
                capture.source_.label, program)
            || program.recipeBytes != capture.sourceProgram_.recipeBytes
            || !feature_pattern_owner::LocateLabels(owner, owner.Document(), labels)
            || labels.size() != capture.freeLabelCount_) return false;
        const auto host = labels.find(capture.host_.entity);
        const auto source = labels.find(capture.source_.entity);
        return host != labels.end() && source != labels.end()
            && host->second.shape.IsEqual(prepared.built_.result)
            && source->second.shape.IsEqual(capture.source_.shape);
    } catch (...) { return false; }
}

CaptureStatus CaptureCreationHost(OcctDocument& owner,
    const std::string& hostEntity, const std::string& sourceEntity,
    const std::shared_ptr<native_opening::Context>& context,
    std::uint32_t width, std::uint32_t height,
    std::shared_ptr<const CreationCapture>& output) noexcept {
    output.reset();
    try {
        if (!context || width == 0 || height == 0
            || context->openingFence().document() != owner.Document()
            || context->openingFence().data() != owner.Document()->GetData()
            || !context->isCurrent(width, height)) return CaptureStatus::Refused;
        auto value = std::shared_ptr<CreationCapture>(new CreationCapture);
        if (!CaptureInputs(owner, hostEntity, sourceEntity, value->document_, value->host_,
                value->source_, value->profile_, value->scalarRecipe_,
                value->sourceProgram_, value->freeLabelCount_,
                value->featurePatternBytes_)) return CaptureStatus::Refused;
        value->context_ = context; value->width_ = width; value->height_ = height;
        value->documentTime_ = owner.Document()->GetData()->Time();
        output = std::move(value); return CaptureStatus::Current;
    } catch (...) { output.reset(); return CaptureStatus::Refused; }
}

std::shared_ptr<const PreparedCreation> PrepareCreation(
    const std::shared_ptr<const CreationCapture>& capture,
    const CreationEdit& edit, const std::atomic_bool& stop) noexcept {
    try {
        if (!capture || !capture->context_ || stop.load()) return {};
        const auto* program = std::get_if<retained_boolean::Program>(
            &capture->sourceProgram_.recipe);
        if (!program) return {};
        const auto selected = edit.sourceCutStepID == 0
            ? std::find_if(program->steps.begin(), program->steps.end(),
                [](const auto& value) {
                    return value.operation == analytic_boolean::Operation::Difference;
                })
            : std::find_if(program->steps.begin(), program->steps.end(),
                [&](const auto& value) {
                    return value.operation == analytic_boolean::Operation::Difference
                        && value.operand.identifier == edit.sourceCutStepID;
                });
        if (selected == program->steps.end()) return {};

        auto prepared = std::shared_ptr<PreparedCreation>(new PreparedCreation);
        prepared->capture_ = capture;
        auto& definition = prepared->definition_;
        definition.host = {capture->document_, capture->host_.entity,
                           capture->host_.definition};
        if (!Issue(definition.feature)) return {};
        definition.sourceCut = {program->source.document, program->source.entity,
            program->source.definition, program->source.derivedFeature};
        definition.sourceCutStepID = selected->operand.identifier;
        definition.metersPerUnit = capture->profile_.parameters.metersPerUnit;
        auto& distribution = definition.distribution;
        distribution.owner = definition.host;
        distribution.feature = definition.feature;
        distribution.source = definition.sourceCut;
        distribution.kind = edit.kind; distribution.rowAxis = edit.rowAxis;
        distribution.columnAxis = edit.columnAxis;
        distribution.rowSpacing = edit.rowSpacing;
        distribution.columnSpacing = edit.columnSpacing;
        distribution.sweepRadians = edit.sweepRadians;
        distribution.radialPivotLocal = edit.radialPivotLocal;
        distribution.sourceFrame = pattern::IdentityMatrix();
        distribution.issuance.nextLocalID = 1;
        const pattern::IssueUUID issue = [](UUID& output) { return Issue(output); };
        if (!feature_pattern::ReconcileCounts(definition, edit.rows,
                edit.columns, issue)) return {};
        for (const auto& member : definition.distribution.members) {
            const bool suppressed = edit.suppressed.count(member.coordinate) != 0;
            if ((member.coordinate == pattern::Coordinate{} && suppressed)
                || ((member.state == pattern::MemberState::Suppressed) != suppressed
                    && !pattern::SetSuppressed(definition.distribution,
                                               member.coordinate, suppressed)))
                return {};
        }
        if (!feature_pattern::Valid(definition) || stop.load()) return {};

        UUID profileFeature{};
        if (!pattern_owner::Parse(capture->profile_.identifier, profileFeature)) return {};
        auto& baseline = prepared->baseline_;
        baseline.host = definition.host;
        baseline.retainedRecipeFeature = profileFeature;
        if (!Issue(baseline.baselineRecipeIdentity)) return {};
        baseline.exactRecipe = capture->scalarRecipe_;
        baseline.solid = capture->host_.shape;
        baseline.retainedRecipeCurrent = true;

        feature_pattern_native::SourceProgram source;
        source.program = *program; source.exactProgram = capture->sourceProgram_.recipeBytes;
        source.retainedBase = capture->sourceProgram_.base;
        feature_pattern::ExpansionBudget budget;
        budget.existingDocumentBytes = capture->featurePatternBytes_;
        budget.sourceRecipeBytes = source.exactProgram.size();
        budget.existingMemoryBytes = capture->scalarRecipe_.size()
            + capture->sourceProgram_.recipeBytes.size();
        if (!feature_pattern_native::detail::CountTopology(
                baseline.solid, budget.maximumTopologyNodes,
                budget.hostTopologyNodes)) return {};
        feature_pattern_native::SourceTool sourceTool;
        if (feature_pattern_native::BuildSourceTool(source,
                definition.sourceCutStepID, stop, sourceTool)
                != feature_pattern_native::Status::Built
            || !feature_pattern_native::detail::CountTopology(
                sourceTool.detached.tool, budget.maximumTopologyNodes,
                budget.sourceToolTopologyNodes)) return {};
        prepared->built_ = feature_pattern_native::BuildAttributedPattern(
            baseline, source, definition, budget, stop);
        if (stop.load() || !prepared->built_.admitted()
            || prepared->built_.result.IsNull()) return {};
        return prepared;
    } catch (...) { return {}; }
}

bool ReviewPreparedCreation(const PreparedCreation& prepared,
                            PreparedReview& output) noexcept {
    output = {};
    try {
        const auto& built = prepared.built_;
        if (!prepared.capture_ || !built.admitted()
            || built.childReceipts.empty()
            || built.positiveRemovedVolumes.size() != built.childReceipts.size()
            || built.measuredBoundarySections == 0
            || !std::isfinite(built.measuredPairwiseLigamentMM)
            || built.measuredPairwiseLigamentMM <= 0
            || !built.admission.projection.admitted)
            return false;
        for (double volume : built.positiveRemovedVolumes)
            if (!std::isfinite(volume) || volume <= 0) return false;
        output.attributedChildCount = built.childReceipts.size();
        output.sectionCount = built.measuredBoundarySections;
        output.positiveRemovedVolumes = built.positiveRemovedVolumes;
        output.measuredPairwiseLigamentMM = built.measuredPairwiseLigamentMM;
        output.chargedProjection = built.admission.projection;
        return true;
    } catch (...) { output = {}; return false; }
}

CreationOutcome StageCreation(OcctDocument& owner,
    const std::shared_ptr<const PreparedCreation>& prepared) noexcept {
    if (!prepared || !prepared->capture_) return CreationOutcome::Refused;
#if DEBUG
    const auto refused = [](const char *gate) {
        std::fprintf(stderr, "C1H_CREATION_STAGE gate=%s\n", gate);
        return CreationOutcome::Refused;
    };
#else
    const auto refused = [](const char *) { return CreationOutcome::Refused; };
#endif
    bool expected = false;
    if (!prepared->consumed_.compare_exchange_strong(expected, true))
        return refused("consumed");
    const auto& capture = *prepared->capture_;
    try {
        std::shared_ptr<const CreationCapture> current;
        if (CaptureCreationHost(owner,
                retained_solid::UUIDText(capture.host_.entity),
                retained_solid::UUIDText(capture.source_.entity), capture.context_,
                capture.width_, capture.height_, current) != CaptureStatus::Current
            || !current || current->documentTime_ != capture.documentTime_
            || !SameReceipt(current->host_, capture.host_)
            || !SameReceipt(current->source_, capture.source_)
            || !current->profile_.IsEqual(capture.profile_)
            || current->scalarRecipe_ != capture.scalarRecipe_
            || current->sourceProgram_.recipeBytes
                != capture.sourceProgram_.recipeBytes) return refused("recapture");
        auto lease = capture.context_->beginCommandLease(
            capture.context_->openingFence(), capture.width_, capture.height_);
        if (!lease || !lease->ownsOpenCommand()) return refused("lease");
        const auto abort = [&](const char *gate) {
#if DEBUG
            std::fprintf(stderr, "C1H_CREATION_STAGE gate=%s\n", gate);
#endif
            return lease->abort() && !owner.Document()->HasOpenCommand()
                ? CreationOutcome::Refused : CreationOutcome::OutcomeUnknown;
        };
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(owner.Document()->Main());
        if (shapes.IsNull()) return abort("shape-tool");
        // Profile metadata uses explicit legacy child tags, so a previously
        // absent TagSource may initially enumerate occupied children. Advance
        // it under the owned command until it proves a genuinely fresh label.
        TDF_Label baselineLabel;
        for (std::size_t attempt = 0; attempt < profile::MaximumRecords; ++attempt) {
            const TDF_Label candidate = TDF_TagSource::NewChild(capture.host_.label);
            if (!candidate.HasAttribute()) { baselineLabel = candidate; break; }
        }
        if (baselineLabel.IsNull()) return abort("baseline-label-budget");
        if (!feature_pattern_baseline::Stage(baselineLabel, capture.host_.label,
                prepared->definition_.host, prepared->baseline_.retainedRecipeFeature,
                prepared->baseline_.baselineRecipeIdentity,
                prepared->baseline_.exactRecipe, prepared->baseline_.solid))
            return abort("baseline");
        shapes->SetShape(capture.host_.label, prepared->built_.result);
        const Handle(TDF_TagSource) tags = TDF_TagSource::Set(capture.host_.label);
        Standard_Integer highWater = tags->Get();
        for (TDF_ChildIterator child(capture.host_.label, Standard_False);
             child.More(); child.Next())
            highWater = std::max(highWater, child.Value().Tag());
        tags->Set(highWater);
        feature_pattern_child::PairedRecord pair;
        if (!owner.StageFeaturePatternPair(*lease, capture.host_.label,
                baselineLabel, capture.source_.label, prepared->definition_,
                prepared->built_.childReceipts, pair)) return abort("pair");
#if DEBUG
        if (ConsumeCreationFault(CreationFault::AfterLastChild))
            return abort("debug-after-last-child");
#endif
        if (!profile::Stage(owner.Document(), capture.host_.label,
                capture.profile_.parameters, capture.profile_.identifier))
            return abort("profile");
#if DEBUG
        if (ConsumeCreationFault(CreationFault::CreationReadback))
            return abort("readback");
#endif
        if (!CreationReadback(owner, *prepared, baselineLabel))
            return abort("readback");
        if (!lease->commit()) {
            native_opening::CommittedEditPublication publication;
            publication.replaced.push_back(
                {retained_solid::UUIDText(capture.host_.entity),
                 capture.host_.shape});
            capture.context_->retainUnprovenEdit(publication);
            return CreationOutcome::OutcomeUnknown;
        }
        native_opening::CommittedEditPublication publication;
        publication.replaced.push_back(
            {retained_solid::UUIDText(capture.host_.entity),
             capture.host_.shape});
        return capture.context_->publishCommittedEdit(publication)
            ? CreationOutcome::Committed : CreationOutcome::OutcomeUnknown;
    } catch (...) { return CreationOutcome::OutcomeUnknown; }
}

CaptureStatus CaptureCurrent(OcctDocument& owner, const std::string& hostEntity,
    const std::shared_ptr<native_opening::Context>& context,
    std::shared_ptr<const CurrentCapture>& output) noexcept {
    output.reset();
    try {
        feature_pattern_owner::Snapshot d4;
        const auto status = feature_pattern_owner::CaptureNative(
            owner, hostEntity, context, d4);
        if (status == feature_pattern_owner::Refusal::AmbiguousSelection) {
            UUID selected{}; std::vector<feature_pattern::Record> records;
            if (!pattern_owner::Parse(hostEntity, selected)
                || !feature_pattern::ReadAll(owner.Document(), records))
                return CaptureStatus::Refused;
            std::size_t matches = 0;
            for (const auto& record : records)
                if (record.definition.host.entity == selected
                    || record.definition.sourceCut.entity == selected) ++matches;
            return matches == 0 ? CaptureStatus::Absent : CaptureStatus::Refused;
        }
        if (status != feature_pattern_owner::Refusal::None
            || d4.record.definition.host.entity != d4.host.entity)
            return CaptureStatus::Refused;
        profile::Record profileRecord;
        std::vector<std::uint8_t> scalarRecipe;
        UUID profileFeature{};
        if (!profile::Read(owner.Document(), d4.host.label, profileRecord)
            || profileRecord.label.IsNull()
            || !profileRecord.IsCurrent(owner.Document(), d4.host.label)
            || !pattern_owner::Parse(profileRecord.identifier, profileFeature)
            || profileFeature != d4.hostBase.retained.retainedRecipeFeature
            || !ScalarRecipe(profileRecord.parameters, scalarRecipe)
            || scalarRecipe != d4.hostBase.retained.exactRecipe)
            return CaptureStatus::Refused;
        auto openingFence = context->recapture(64, 64);
        if (!openingFence
            || !native_opening::SameFence(
                context->openingFence(), *openingFence))
            return CaptureStatus::Refused;
        auto capture = std::shared_ptr<CurrentCapture>(new CurrentCapture);
        capture->d4_ = std::move(d4);
        capture->profile_ = std::move(profileRecord);
        capture->scalarRecipe_ = std::move(scalarRecipe);
        capture->openingFence_ = std::move(openingFence);
        output = std::move(capture);
        return CaptureStatus::Current;
    } catch (...) { output.reset(); return CaptureStatus::Refused; }
}

std::shared_ptr<const PreparedHostEdit> PrepareHostEdit(OcctDocument& owner,
    const std::shared_ptr<const CurrentCapture>& capture,
    const profile::Parameters& requested, const TopoDS_Shape& baseline,
    const std::atomic_bool& stop) noexcept {
    try {
        if (!capture || !capture->openingFence_ || stop.load()
            || baseline.IsNull() || baseline.ShapeType() != TopAbs_SOLID
            || requested.metersPerUnit
                != capture->profile_.parameters.metersPerUnit)
            return {};
        std::vector<std::uint8_t> scalarRecipe;
        if (!ScalarRecipe(requested, scalarRecipe) || stop.load()) return {};
        feature_pattern_owner::Snapshot prospective = capture->d4_;
        prospective.host.shape = baseline;
        prospective.hostBase.retained.solid = baseline;
        prospective.hostBase.retained.exactRecipe = scalarRecipe;
        prospective.hostBase.retained.retainedRecipeCurrent = true;
        auto prepared = feature_pattern_owner::PrepareNative(owner, prospective,
            ExistingEdit(capture->d4_.record.definition),
            feature_pattern_owner::Limits{});
        if (stop.load() || !prepared.admitted()
            || prepared.rebuilt.result.IsNull()) return {};
        auto result = std::shared_ptr<PreparedHostEdit>(new PreparedHostEdit);
        result->capture_ = capture;
        result->requested_ = requested;
        result->scalarRecipe_ = std::move(scalarRecipe);
        result->prepared_ = std::move(prepared);
        return result;
    } catch (...) { return {}; }
}

const TopoDS_Shape& Baseline(const PreparedHostEdit& value) noexcept {
    return value.prepared_.opening.hostBase.retained.solid;
}

const TopoDS_Shape& Result(const PreparedHostEdit& value) noexcept {
    return value.prepared_.rebuilt.result;
}

bool OpeningMatches(const PreparedHostEdit& value,
    const std::shared_ptr<native_opening::Context>& context) noexcept {
    try {
        return context && value.capture_ && value.capture_->openingFence_
            && native_opening::SameFence(
                *value.capture_->openingFence_, context->openingFence());
    } catch (...) { return false; }
}

dependent_replay::Refusal PrepareHostReplay(OcctDocument& owner,
    const std::shared_ptr<const PreparedHostEdit>& prepared,
    const dependent_replay::Limits& limits,
    std::shared_ptr<const dependent_replay::PreparedReplay>& output) noexcept {
    output.reset();
    try {
        if (!prepared || !prepared->capture_ || limits.records < 1)
            return dependent_replay::Refusal::MissingRecipe;
        const auto& opening = prepared->capture_->d4_;
        dependent_replay::Dependency dependency;
        dependency.family = dependent_replay::Family::FeaturePatternD4;
        dependency.feature = opening.record.definition.feature;
        dependency.inputEntity = opening.record.definition.host.entity;
        dependency.resultEntity = opening.record.definition.host.entity;
        dependency.recordLabel = opening.record.label;
        dependency.canonicalRecordBytes = opening.record.bytes;
        dependency.memberIdentities.reserve(opening.children.size());
        for (const auto& child : opening.children)
            dependency.memberIdentities.push_back(child.persisted.childFeature);
        ProfileHostPreparer preparer(prepared);
        auto refusal = preparer.prepare(owner, dependency,
            dependent_replay::Mutation::Replace, output);
        if (refusal != dependent_replay::Refusal::None) return refusal;
        if (!output
            || output->family() != dependent_replay::Family::FeaturePatternD4
            || output->feature() != dependency.feature
            || output->resultEntity() != dependency.resultEntity)
            return dependent_replay::Refusal::UnsupportedDescendant;
        const auto fits = [](std::size_t first, std::size_t second,
                             std::size_t limit) noexcept {
            return first <= limit && second <= limit - first;
        };
        if (!fits(dependency.canonicalRecordBytes.size(),
                  output->documentBytes(), limits.documentBytes))
            return dependent_replay::Refusal::DocumentBudget;
        if (output->memoryBytes() > limits.memoryBytes)
            return dependent_replay::Refusal::MemoryBudget;
        if (output->topologyNodes() > limits.topologyNodes)
            return dependent_replay::Refusal::TopologyBudget;
        if (!output->openingCurrent(owner))
            return dependent_replay::Refusal::MissingRecipe;
        return dependent_replay::Refusal::None;
    } catch (...) {
        output.reset();
        return dependent_replay::Refusal::MissingRecipe;
    }
}

bool ObserveCurrent(OcctDocument& owner, const std::string& hostEntity,
    const std::shared_ptr<native_opening::Context>& context,
    Observation& output) noexcept {
    output = {};
    try {
        std::shared_ptr<const CurrentCapture> capture;
        if (CaptureCurrent(owner, hostEntity, context, capture)
                != CaptureStatus::Current || !capture) return false;
        Observation value; value.current = true;
        value.hostEntity = retained_solid::UUIDText(capture->d4_.host.entity);
        value.sourceEntity = retained_solid::UUIDText(
            capture->d4_.sourceCarrier.entity);
        value.profileFeature = capture->profile_.identifier;
        value.patternFeature = retained_solid::UUIDText(
            capture->d4_.record.definition.feature);
        value.sourceCutStepID = capture->d4_.record.definition.sourceCutStepID;
        value.memberCount = capture->d4_.record.definition.distribution.members.size();
        value.childCount = capture->d4_.children.size();
        value.profileScalarCount = capture->profile_.values.size();
        value.sourceProgramBytes = capture->d4_.sourceProgram.recipeBytes.size();
        value.metersPerUnit = capture->profile_.parameters.metersPerUnit;
        value.baselineVolume = Volume(capture->d4_.hostBase.retained.solid);
        value.resultVolume = Volume(capture->d4_.host.shape);
        value.sourceVolume = Volume(capture->d4_.sourceCarrier.shape);
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

} // namespace core3d::profile_d4
