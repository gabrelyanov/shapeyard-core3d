#import <Foundation/Foundation.h>

#include "FeaturePatternOwnerBridge.hxx"
#include "NativeOpeningDependentReplay.hxx"

#include <TNaming_NamedShape.hxx>
#include <TNaming_Tool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <algorithm>
#include <atomic>
#include <set>

#if DEBUG
#include "NativeOpeningSurfaceProbe.hxx"
#endif

namespace core3d::feature_pattern_owner {
namespace {
constexpr std::uint32_t kFenceWidth = 64;
constexpr std::uint32_t kFenceHeight = 64;

std::string Text(const UUID& value) noexcept {
    try { return retained_solid::UUIDText(value); }
    catch (...) { return {}; }
}

class OcafD4Observer final : public Observer {
public:
    explicit OcafD4Observer(OcctDocument& owner) noexcept : owner_(owner) {}

    bool captureHostBase(const pattern_owner::LabelReceipt& host,
        const TDF_Label& recipeLabel, HostBaseReceipt& output) noexcept override {
        output = {};
        try {
            const Handle(TDocStd_Document) document = owner_.Document();
            if (document.IsNull() || host.label.IsNull() || recipeLabel.IsNull()
                || host.label.Data() != document->GetData()
                || recipeLabel.Data() != document->GetData()) return false;
            std::vector<feature_pattern_child::PairedRecord> pairs;
            if (feature_pattern_child::ReadPairs(document, pairs)
                    != feature_pattern_child::PairStatus::Valid) return false;
            const feature_pattern_child::PairedRecord* pair = nullptr;
            for (const auto& candidate : pairs)
                if (candidate.host.IsEqual(host.label)
                    && candidate.baselineRecipe.IsEqual(recipeLabel)) {
                    if (pair) return false;
                    pair = &candidate;
                }
            if (!pair || pair->children.empty()) return false;
            const UUID baseline = pair->children.front()->receipt.baselineRecipeIdentity;
            for (const auto& child : pair->children)
                if (!child || child->receipt.baselineRecipeIdentity != baseline)
                    return false;

            // The typed host baseline must validate through the same strict
            // census persistence and publication use; the requested host and
            // recipe label must name its exact validated placement.
            feature_pattern_child::BaselineRecord typed;
            {
                std::vector<feature_pattern_child::BaselineRecord> baselines;
                if (feature_pattern_child::ReadBaselines(document, baselines)
                        != feature_pattern_child::BaselineStatus::Valid)
                    return false;
                bool found = false;
                for (const auto& candidate : baselines) {
                    if (!candidate.host.IsEqual(host.label)) continue;
                    if (found || !candidate.label.IsEqual(recipeLabel)
                        || !candidate.value) return false;
                    found = true; typed = candidate;
                }
                if (!found) return false;
            }
            HostBaseReceipt value;
            value.recipeLabel = recipeLabel;
            value.retained.host.document = typed.value->envelope.document;
            value.retained.host.entity = typed.value->envelope.hostEntity;
            value.retained.host.definition = typed.value->envelope.hostDefinition;
            value.retained.retainedRecipeFeature =
                typed.value->envelope.retainedRecipeFeature;
            value.retained.baselineRecipeIdentity =
                typed.value->envelope.baselineRecipeIdentity;
            value.retained.solid = typed.solid;
            value.retained.exactRecipe = typed.value->envelope.exactRecipe;
            value.retained.retainedRecipeCurrent = true;
            if (value.retained.host.entity != host.entity
                || value.retained.host.definition != host.definition
                || value.retained.host.document
                    != pair->pattern.definition.host.document
                || value.retained.baselineRecipeIdentity != baseline
                || !value.admitted()) return false;
            output = std::move(value);
            return true;
        } catch (...) { output = {}; return false; }
    }

    bool captureChildren(const feature_pattern_child::PairedRecord& pair,
        std::vector<ChildReceipt>& output) noexcept override {
        output.clear();
        try {
            std::vector<pattern::Placement> placements;
            if (!pattern::BuildPlacements(pair.pattern.definition.distribution,
                                          placements)
                || placements.size() != pair.children.size()) return false;
            std::set<UUID> features;
            for (const auto& payload : pair.children) {
                if (!payload || !features.insert(payload->receipt.childFeature).second)
                    return false;
                const auto placement = std::find_if(placements.begin(), placements.end(),
                    [&](const pattern::Placement& value) {
                        return value.identity == payload->receipt.instanceIdentity
                            && value.localID == payload->receipt.localID
                            && std::int32_t(value.coordinate.row) == payload->receipt.row
                            && std::int32_t(value.coordinate.column) == payload->receipt.column;
                    });
                if (placement == placements.end()) return false;
                const auto section = std::find_if(payload->receipt.selectors.begin(),
                    payload->receipt.selectors.end(), [](const auto& value) {
                        return value.kind
                            == feature_pattern_child::SelectorKind::GeneratedSection;
                    });
                if (section == payload->receipt.selectors.end()
                    || section->ordinal
                        != pair.pattern.definition.expectedBoundarySectionsPerFeature)
                    return false;
                ChildReceipt child;
                for (TDF_ChildIterator record(payload->host, Standard_False);
                     record.More(); record.Next()) {
                    Handle(feature_pattern_child::Attribute) attribute;
                    if (record.Value().FindAttribute(
                            feature_pattern_child::AttributeID(), attribute)
                        && !attribute.IsNull() && attribute->value() == payload) {
                        child.recordLabel = record.Value(); break;
                    }
                }
                if (child.recordLabel.IsNull()) return false;
                child.persisted = payload->receipt;
                child.worldFrame = placement->worldFrame;
                child.boundarySections = section->ordinal;
                child.canonicalBytes = payload->canonicalBytes;
                std::vector<std::uint8_t> canonical;
                if (!feature_pattern_child::Encode(child.persisted, canonical)
                    || canonical != child.canonicalBytes) return false;
                output.push_back(std::move(child));
            }
            return output.size() == placements.size();
        } catch (...) { output.clear(); return false; }
    }

private:
    OcctDocument& owner_;
};

class OcctD4Rebuilder final : public Rebuilder {
public:
    bool rebuild(const Snapshot& opening, const retained_boolean::Step& step,
        const feature_pattern::Definition& definition,
        RebuildProduct& output) noexcept override {
        output = {};
        try {
            const auto* program = std::get_if<retained_boolean::Program>(
                &opening.sourceProgram.recipe);
            if (!program || step.operand.identifier != definition.sourceCutStepID)
                return false;
            feature_pattern_native::SourceProgram source;
            source.program = *program;
            source.exactProgram = opening.sourceProgram.recipeBytes;
            source.retainedBase = opening.sourceProgram.base;
            feature_pattern::ExpansionBudget budget;
            std::atomic_bool stop{false};
            auto built = feature_pattern_native::BuildAttributedPattern(
                opening.hostBase.retained, source, definition, budget, stop);
            if (!built.admitted()) return false;
            RebuildProduct value;
            value.result = built.result;
            value.verifiedAdmission = built.admission;
            for (std::size_t index = 0; index < built.childReceipts.size(); ++index) {
                ChildReceipt child;
                child.persisted = built.childReceipts[index];
                child.worldFrame = built.selectors[index].worldFrame;
                child.boundarySections = built.selectors[index].orientedBoundarySections;
                if (!feature_pattern_child::Encode(child.persisted,
                                                   child.canonicalBytes)) return false;
                value.children.push_back(std::move(child));
            }
            output = std::move(value);
            return true;
        } catch (...) { output = {}; return false; }
    }
};

class OcafD4Stager final : public Stager {
public:
    OcafD4Stager(OcctDocument& owner,
        std::shared_ptr<native_opening::Context> context) noexcept
        : owner_(owner), context_(std::move(context)), observer_(owner) {}
    OcafD4Stager(OcctDocument& owner,
        native_opening::CommandLease& lease) noexcept
        : owner_(owner), borrowed_(&lease), observer_(owner) {}

    bool begin(const Snapshot& current,
               const PreparedEdit& prepared) noexcept override {
        try {
            if ((!borrowed_ && !context_)
                || (!borrowed_ && !SameSnapshot(current, prepared.opening))
                || (context_ && (context_->openingFence().document() != owner_.Document()
                || context_->openingFence().data() != owner_.Document()->GetData())))
                return false;
            if (borrowed_) return borrowed_->ownsOpenCommand();
            lease_ = context_->beginCommandLease(context_->openingFence(),
                                                 kFenceWidth, kFenceHeight);
            return lease_ && lease_->ownsOpenCommand();
        } catch (...) { return false; }
    }

    bool stageAll(const Snapshot& current,
                  const PreparedEdit& prepared) noexcept override {
        try {
            auto* lease = borrowed_ ? borrowed_ : lease_.get();
            if (!lease || !lease->ownsOpenCommand()
                || (!borrowed_ && !SameSnapshot(current, prepared.opening))
                || (!borrowed_ && !prepared.candidateSourceProgramBytes.empty())) return false;
            const Handle(XCAFDoc_ShapeTool) shapes =
                XCAFDoc_DocumentTool::ShapeTool(owner_.Document()->Main());
            if (shapes.IsNull() || prepared.rebuilt.result.IsNull()) return false;
            shapes->SetShape(current.host.label, prepared.rebuilt.result);
            std::vector<feature_pattern_child::Receipt> receipts;
            for (const auto& child : prepared.rebuilt.children)
                receipts.push_back(child.persisted);
            return owner_.StageFeaturePatternPair(*lease, current.host.label,
                current.hostBase.recipeLabel, current.sourceCarrier.label,
                prepared.candidate, receipts, staged_);
        } catch (...) { return false; }
    }

    bool readBackAll(const feature_pattern::Definition& definition,
                     Readback& output) noexcept override {
        output = {};
        try {
            feature_pattern_child::PairedRecord pair;
            if (!owner_.ReadFeaturePatternPair(definition.feature, pair)) return false;
            Readback value;
            if (!pattern_owner::ReadReceipt(owner_, pair.host, value.host)
                || !pattern_owner::ReadReceipt(owner_, pair.source,
                                               value.sourceCarrier)
                || !observer_.captureHostBase(value.host, pair.baselineRecipe,
                                              value.hostBase)
                || !observer_.captureChildren(pair, value.children)) return false;
            OcctCylindricalCutProgramSource source;
            if (!owner_.CaptureCylindricalCutProgramSource(pair.source, source))
                return false;
            value.sourceProgramBytes = source.recipeBytes;
            value.paired = std::move(pair);
            output = std::move(value);
            return true;
        } catch (...) { output = {}; return false; }
    }

    bool commit() noexcept override {
        if (!lease_) return false;
        const bool result = lease_->commit(); lease_.reset(); return result;
    }
    bool abort() noexcept override {
        if (!lease_) return false;
        const bool result = lease_->abort(); lease_.reset(); staged_ = {};
        return result;
    }

private:
    OcctDocument& owner_;
    std::shared_ptr<native_opening::Context> context_;
    std::shared_ptr<native_opening::CommandLease> lease_;
    native_opening::CommandLease* borrowed_ = nullptr;
    OcafD4Observer observer_;
    feature_pattern_child::PairedRecord staged_;
};
} // namespace

Refusal CaptureNative(OcctDocument& owner, const std::string& selectedEntity,
    const std::shared_ptr<native_opening::Context>& context,
    Snapshot& output) noexcept {
    output = {};
    try {
        if (!context || context->openingFence().document() != owner.Document()
            || context->openingFence().data() != owner.Document()->GetData()
            || !context->isCurrent(kFenceWidth, kFenceHeight))
            return Refusal::StaleHost;
        OcafD4Observer observer(owner);
        return Capture(owner, selectedEntity, observer, output);
    } catch (...) { output = {}; return Refusal::CorruptTable; }
}

PreparedEdit PrepareNative(OcctDocument&, const Snapshot& opening,
    const Edit& edit, const Limits& limits) noexcept {
    const pattern::IssueUUID issue = [](UUID& output) {
        return pattern_owner::Parse(OcctDocument::NewProfileIdentifier(), output);
    };
    OcctD4Rebuilder rebuilder;
    return Prepare(opening, edit, limits, issue, rebuilder);
}

ApplyOutcome ApplyNative(OcctDocument& owner, const PreparedEdit& prepared,
    const std::shared_ptr<native_opening::Context>& context) noexcept {
    OcafD4Observer observer(owner);
    OcafD4Stager stager(owner, context);
    const auto outcome = Apply(owner, prepared, observer, stager);
    // A proven commit is not delivered until the owning viewer has reconciled
    // the rebuilt host's real AIS presentation to the committed OCAF geometry.
    // An unknown close retains the same plan for exact recovery instead.
    if (context && outcome != ApplyOutcome::Refused) {
        native_opening::CommittedEditPublication publication;
        publication.replaced.push_back(
            {Text(prepared.opening.host.entity), {}});
        if (outcome == ApplyOutcome::Committed)
            return context->publishCommittedEdit(publication)
                ? ApplyOutcome::Committed : ApplyOutcome::OutcomeUnknown;
        context->retainUnprovenEdit(publication);
    }
    return outcome;
}

namespace {
Edit RetainedD4Edit(const feature_pattern::Definition& value) {
    Edit edit;
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

bool CaptureD4InsideCommand(OcctDocument& owner, const Snapshot& opening,
                            Snapshot& output) noexcept {
    output = {};
    try {
        feature_pattern::Record record;
        feature_pattern_child::PairedRecord pair;
        if (!feature_pattern::ReadFeature(owner.Document(),
                opening.record.definition.feature, record)
            || record.bytes != opening.record.bytes
            || !owner.ReadFeaturePatternPair(
                opening.record.definition.feature, pair)) return false;
        OcafD4Observer observer(owner);
        Snapshot value; value.record = std::move(record); value.paired = pair;
        if (!pattern_owner::ReadReceipt(owner, pair.host, value.host)
            || !pattern_owner::ReadReceipt(owner, pair.source, value.sourceCarrier)
            || !observer.captureHostBase(value.host, pair.baselineRecipe,
                                         value.hostBase)
            || !observer.captureChildren(pair, value.children)
            || !owner.CaptureCylindricalCutProgramSource(
                pair.source, value.sourceProgram)) return false;
        const auto* program = std::get_if<retained_boolean::Program>(
            &value.sourceProgram.recipe);
        if (!program) return false;
        const auto step = std::find_if(program->steps.begin(), program->steps.end(),
            [&](const retained_boolean::Step& item) {
                return item.operation == analytic_boolean::Operation::Difference
                    && item.operand.identifier
                        == value.record.definition.sourceCutStepID;
            });
        if (step == program->steps.end()) return false;
        value.sourceStep = *step;
        output = std::move(value);
        return SameSnapshot(output, opening);
    } catch (...) { output = {}; return false; }
}

class D4DependentReplay final : public dependent_replay::PreparedReplay {
public:
    D4DependentReplay(dependent_replay::Dependency dependency, Snapshot opening,
        PreparedEdit prepared) noexcept
        : dependency_(std::move(dependency)), opening_(std::move(opening)),
          prepared_(std::move(prepared)) {}
    dependent_replay::Family family() const noexcept override {
        return dependent_replay::Family::FeaturePatternD4;
    }
    dependent_replay::UUID feature() const noexcept override { return dependency_.feature; }
    dependent_replay::UUID resultEntity() const noexcept override { return dependency_.resultEntity; }
    std::size_t documentBytes() const noexcept override {
        return opening_.featurePatternDocumentBytes
            + opening_.sourceProgram.recipeBytes.size()
            + opening_.hostBase.retained.exactRecipe.size();
    }
    std::size_t memoryBytes() const noexcept override {
        return prepared_.rebuilt.children.size() * sizeof(ChildReceipt);
    }
    std::size_t topologyNodes() const noexcept override {
        return prepared_.projection.projectedTopologyNodes;
    }
    bool openingCurrent(OcctDocument& owner) const noexcept override {
        Snapshot current;
        return CaptureD4InsideCommand(owner, opening_, current);
    }
    bool stage(OcctDocument& owner,
               native_opening::CommandLease& lease) const noexcept override {
        try {
            stager_ = std::make_unique<OcafD4Stager>(owner, lease);
            if (!stager_->begin(opening_, prepared_)
                || !stager_->stageAll(opening_, prepared_)) return false;
            // An authorized host replacement carries the prospective baseline
            // in the prepared opening; update its typed payload and the bound
            // solid in this same owned command. Baseline identities and the
            // paired references are preserved, and ordinary count, suppression
            // and source-step edits leave the baseline untouched.
            const auto& before = opening_.hostBase.retained;
            const auto& after = prepared_.opening.hostBase.retained;
            if ((after.exactRecipe != before.exactRecipe
                    || !after.solid.IsEqual(before.solid))
                && !feature_pattern_baseline::Replace(
                    opening_.hostBase.recipeLabel, after.exactRecipe,
                    after.solid)) return false;
            return true;
        } catch (...) { stager_.reset(); return false; }
    }
    bool read(OcctDocument&) const noexcept override {
        if (!stager_) return false;
        Readback readback;
        return stager_->readBackAll(prepared_.candidate, readback)
            && ExactReadback(prepared_, readback);
    }
private:
    dependent_replay::Dependency dependency_;
    Snapshot opening_;
    PreparedEdit prepared_;
    mutable std::unique_ptr<OcafD4Stager> stager_;
};
}

dependent_replay::Refusal PrepareFeaturePatternD4ReplayImpl(OcctDocument& owner,
    const dependent_replay::Dependency& dependency,
    dependent_replay::Mutation mutation,
    const dependent_replay::Candidate& candidate,
    const std::shared_ptr<native_opening::Context>& context,
    std::shared_ptr<const dependent_replay::PreparedReplay>& output) noexcept {
    output.reset();
    try {
        if (mutation != dependent_replay::Mutation::Replace)
            return dependent_replay::Refusal::UnsupportedDescendant;
        Snapshot opening;
        if (CaptureNative(owner, retained_solid::UUIDText(dependency.inputEntity),
                context, opening) != Refusal::None
            || opening.record.definition.feature != dependency.feature
            || opening.record.bytes != dependency.canonicalRecordBytes)
            return dependent_replay::Refusal::MissingRecipe;
        Snapshot prospective = opening;
        const auto& definition = opening.record.definition;
        if (dependency.inputEntity == definition.sourceCut.entity) {
            if (!candidate.hasRetainedRecipe
                || !std::holds_alternative<retained_boolean::Program>(
                    candidate.retainedRecipe))
                return dependent_replay::Refusal::MissingRecipe;
            prospective.sourceProgram.recipe = candidate.retainedRecipe;
            prospective.sourceProgram.recipeBytes = candidate.retainedRecipeBytes;
            prospective.sourceProgram.base = candidate.retainedBase;
            prospective.sourceCarrier.shape = candidate.resultShape;
        } else if (dependency.inputEntity == definition.host.entity) {
            if (!candidate.hasRetainedRecipe) return dependent_replay::Refusal::MissingRecipe;
            prospective.host.shape = candidate.resultShape;
            prospective.hostBase.retained.solid = candidate.resultShape;
            prospective.hostBase.retained.exactRecipe = candidate.retainedRecipeBytes;
            prospective.hostBase.retained.retainedRecipeCurrent = true;
        } else return dependent_replay::Refusal::UnsupportedDescendant;
        PreparedEdit prepared = PrepareNative(owner, prospective,
            RetainedD4Edit(definition), Limits{});
        if (!prepared.admitted()) return dependent_replay::Refusal::MissingRecipe;
        if (dependency.inputEntity == definition.sourceCut.entity) {
            // A source-step edit keeps the original opening and baseline; the
            // sealed prospective complete-program bytes travel separately.
            prepared.opening = opening;
            prepared.candidateSourceProgramBytes = candidate.retainedRecipeBytes;
        }
        // A host edit keeps the prospective opening: its baseline receipt
        // carries the authorized replacement solid and exact recipe the stager
        // persists into the typed baseline inside the same owned command.
        output = std::make_shared<D4DependentReplay>(dependency,
            std::move(opening), std::move(prepared));
        return dependent_replay::Refusal::None;
    } catch (...) { output.reset(); return dependent_replay::Refusal::MissingRecipe; }
}

#if DEBUG
extern "C" std::uint64_t
Core3DDebugFeaturePatternNativeCollaboratorsProbe(std::int32_t scenario) noexcept {
    try {
        if (scenario == 0) {
            PreparedEdit prepared;
            prepared.opening.sourceProgram.recipeBytes = {1, 2, 3};
            std::uint64_t bits = ExpectedSourceBytes(prepared)
                == prepared.opening.sourceProgram.recipeBytes ? 1 : 0;
            prepared.candidateSourceProgramBytes = {4, 5, 6};
            if (ExpectedSourceBytes(prepared) == prepared.candidateSourceProgramBytes)
                bits |= 2;
            if (ExpectedSourceBytes(prepared)
                    != prepared.opening.sourceProgram.recipeBytes) bits |= 4;
            prepared.candidateSourceProgramBytes.clear();
            if (ExpectedSourceBytes(prepared)
                    == prepared.opening.sourceProgram.recipeBytes) bits |= 8;
            if (!prepared.opening.sourceProgram.recipeBytes.empty()) bits |= 16;
            return bits;
        }
        if (scenario == 1) {
            ChildReceipt before, after;
            before.persisted = native_opening::debug::child_receipt_probe::Fixture();
            feature_pattern_child::Encode(before.persisted, before.canonicalBytes);
            before.worldFrame = pattern::IdentityMatrix(); before.boundarySections = 4;
            after = before;
            std::uint64_t bits = SameChild(before, after) ? 1 : 0;
            after.canonicalBytes.back() ^= 1;
            if (!SameChild(before, after)) bits |= 2;
            after = before; after.persisted.localID += 1;
            if (!SameChild(before, after)) bits |= 4;
            after = before; after.worldFrame[3] += 1;
            if (!SameChild(before, after)) bits |= 8;
            after = before; after.boundarySections += 1;
            if (!SameChild(before, after)) bits |= 16;
            return bits;
        }
        if (scenario == 2) {
            PreparedEdit prepared;
            prepared.opening.sourceProgram.recipeBytes = {9, 8, 7};
            prepared.rebuilt.children.resize(2);
            Readback read; read.sourceProgramBytes = ExpectedSourceBytes(prepared);
            std::uint64_t bits = read.sourceProgramBytes
                == prepared.opening.sourceProgram.recipeBytes ? 1 : 0;
            read.sourceProgramBytes.back() ^= 1;
            if (read.sourceProgramBytes != ExpectedSourceBytes(prepared)) bits |= 2;
            if (prepared.opening.sourceProgram.recipeBytes
                    == std::vector<std::uint8_t>({9, 8, 7})) bits |= 4;
            if (prepared.rebuilt.children.size() == 2) bits |= 8;
            prepared.rebuilt.children.clear();
            if (prepared.opening.sourceProgram.recipeBytes
                    == std::vector<std::uint8_t>({9, 8, 7})) bits |= 16;
            return bits;
        }
        if (scenario == 3) {
            auto definition = native_opening::debug::detached_tool_probe::Definition(3);
            const auto source = definition.distribution.members[0].identity;
            const auto survivor = definition.distribution.members[1].identity;
            const auto retired = definition.distribution.members[2].identity;
            const pattern::IssueUUID issue = [](UUID& value) {
                value = native_opening::debug::detached_tool_probe::ID(99); return true;
            };
            std::uint64_t bits = 0;
            if (feature_pattern::ReconcileCounts(definition, 1, 2, issue)) bits |= 1;
            if (definition.distribution.members[0].identity == source) bits |= 2;
            if (definition.distribution.members[1].identity == survivor) bits |= 4;
            if (feature_pattern::ReconcileCounts(definition, 1, 3, issue)
                && definition.distribution.members[2].identity != retired) bits |= 8;
            definition.sourceCutStepID += 1;
            if (definition.distribution.members[0].identity == source
                && definition.distribution.members[1].identity == survivor) bits |= 16;
            return bits;
        }
    } catch (...) {}
    return 0;
}
#endif
} // namespace core3d::feature_pattern_owner

namespace core3d::dependent_replay {
Refusal PrepareFeaturePatternD4Replay(OcctDocument& owner,
    const Dependency& dependency, Mutation mutation, const Candidate& candidate,
    const std::shared_ptr<native_opening::Context>& context,
    std::shared_ptr<const PreparedReplay>& output) noexcept {
    return feature_pattern_owner::PrepareFeaturePatternD4ReplayImpl(
        owner, dependency, mutation, candidate, context, output);
}
}
