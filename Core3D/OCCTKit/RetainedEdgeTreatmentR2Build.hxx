#pragma once

#include "RetainedEdgeTreatmentR2Snapshot.hxx"
#include "RetainedEdgeTreatmentBuild.hxx"
#include "RetainedTopologyBudget.hxx"
#include "CompositeRecipeCodec.hxx"
#include "AnalyticBooleanSolid.hxx"
#include "SavedBooleanResultCorrespondence.hxx" // saved-program wedge expectation contract
#include "SavedCutSourceEdit.hxx"
#include "RectangularLoftPersistence.hxx"
#include "ProfilePersistence.hxx"
#include "EnclosurePersistence.hxx"
#include "RetainedSolidEnvelope.hxx"
#include <BRepCheck_Analyzer.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BinTools.hxx>
#include <algorithm>

namespace core3d::retained_edge_treatment::r2 {
namespace tb = core3d::retained_topology_budget;
inline bool ValidateCompletePrefix(const RetainedBooleanBase& source,
                                   const BooleanBaseBinding& binding,
                                   et::Refusal& refusal) noexcept {
    try {
        if (!detail::Valid(binding)) { refusal = et::Refusal::MalformedCarrier; return false; }
        const auto exact = std::visit([](const auto& value) -> const std::vector<std::uint8_t>& {
            return value.canonicalPrefixBytes;
        }, source.source);
        if (exact.empty() || exact.size() > et::MaximumEnvelopeBytes) { refusal = et::Refusal::Budget; return false; }
        Digest digest{};
        if (!CC_SHA256(exact.data(), CC_LONG(exact.size()), digest.data()) || digest != binding.sourceRecipeDigest) {
            refusal = et::Refusal::NoncurrentSource; return false;
        }
        if (binding.format == PrefixFormat::SYRS) {
            const auto* legacy = std::get_if<LegacyBooleanBase>(&source.source);
            std::vector<std::uint8_t> encoded;
            if (!legacy || !retained_boolean::Encode(legacy->prefix, encoded) || encoded != exact) {
                refusal = et::Refusal::MalformedCarrier; return false;
            }
        } else if (!std::holds_alternative<CompositeBooleanBase>(source.source)) {
            refusal = et::Refusal::MalformedCarrier; return false;
        }
        refusal = et::Refusal::None; return true;
    } catch (...) { refusal = et::Refusal::MalformedCarrier; return false; }
}

// Geometry builders consume copied shapes and values only.  The document,
// labels, AIS objects and proof handles remain on the main-owned Work object.
inline bool ReplayTreatmentSuffix(const TopoDS_Shape& postBoolean,
                                  const Definition& definition,
                                  TopoDS_Shape& output,
                                  et::ReplayBudget& budget,
                                  et::Refusal& refusal) noexcept {
    if (postBoolean.IsNull()) {
        refusal = et::Refusal::UnsupportedBase; return false;
    }
    // C19: the analyzer input pass is debited before validity checking.
    const std::atomic_bool neverCancelled{false};
    if (tb::ChargeTraversal(postBoolean, budget, neverCancelled, tb::Site::C19R2Prefix)
        != tb::WalkStatus::Completed) {
        refusal = et::Refusal::Budget; return false;
    }
    if (!BRepCheck_Analyzer(postBoolean).IsValid()) {
        refusal = et::Refusal::UnsupportedBase; return false;
    }
    et::BaseBinding shadow;
    if (const auto* old = std::get_if<et::BaseBinding>(&definition.base)) shadow = *old;
    else {
        const auto& source = std::get<BooleanBaseBinding>(definition.base);
        shadow.source = source.source; shadow.sourceNode = source.sourceNode;
        shadow.sourceSchema = source.sourceSchema; shadow.sourceRecipeDigest = source.sourceRecipeDigest;
        shadow.metersPerLocalUnit = source.metersPerLocalUnit;
    }
    et::Definition suffix{definition.schema, definition.owner, shadow, definition.issuance,
                          definition.outputNode, definition.steps};
    std::vector<et::StepProof> proofs;
    return et::Replay(postBoolean, suffix, output, proofs, budget, refusal);
}

// Bounded migration replay-stage preparation. The complete captured recipe is
// replayed exactly once from the proven pre-Boolean source base: every
// analytic prefix step through analytic_boolean::Build (the same calls the
// prefix loop below makes), then each retained legacy fillet treatment step
// exactly once in order through the same et::Replay machinery
// ReplayTreatmentSuffix uses, on a suffix definition containing exactly those
// steps. It yields the true untreated post-Boolean base — never the captured
// current shape, which already includes any legacy fillets — the stage
// immediately before each legacy treatment step, and the post-existing-
// treatment stage (identical to the post-Boolean base when the recipe carries
// no legacy tail). Every intermediate shape is validated; the aggregate
// budget and cancellation thread through all stages and are never reset. Any
// mismatch refuses atomically; nothing is defaulted.
struct MigrationStages {
    TopoDS_Shape postBoolean;
    std::vector<TopoDS_Shape> stepStages;
    TopoDS_Shape postTreatment;
};
inline bool PrepareMigrationStages(const retained_boolean::Recipe& recipe,
                                   const TopoDS_Shape& sourceBase,
                                   const std::atomic_bool& stop,
                                   et::ReplayBudget& budget,
                                   MigrationStages& stages,
                                   et::Refusal& refusal) noexcept {
    stages = {};
    try {
        retained_boolean::Program program;
        if (const auto* captured = std::get_if<retained_boolean::Program>(&recipe)) program = *captured;
        else if (!retained_boolean::Promote(std::get<retained_boolean::Legacy>(recipe), program)) {
            refusal = et::Refusal::UnsupportedBase; return false;
        }
        if (!retained_boolean::Valid(program) || sourceBase.IsNull()) {
            refusal = et::Refusal::UnsupportedBase; return false;
        }
        // Replay the complete analytic prefix on a private copy of the proven
        // pre-Boolean source base, exactly once per step.
        // C19: the bounded census of the proven source base and the reserved
        // private-copy pass are debited before the copy is constructed; the
        // stage debit counts this prefix census stage.
        tb::Census baseCensus;
        const tb::WalkStatus baseWalk = tb::CensusTopology(sourceBase, budget, stop,
            baseCensus, tb::Site::C19R2Prefix, true);
        if (baseWalk == tb::WalkStatus::Cancelled) { refusal = et::Refusal::Cancelled; return false; }
        if (baseWalk != tb::WalkStatus::Completed
            || !tb::ReserveTraversal(baseCensus, budget, tb::Site::C19R2Prefix)) {
            refusal = et::Refusal::Budget; return false;
        }
        TopoDS_Shape current = BRepBuilderAPI_Copy(sourceBase).Shape();
        if (stop.load() || current.IsNull()) {
            refusal = stop.load() ? et::Refusal::Cancelled : et::Refusal::UnsupportedBase; return false;
        }
        // C19: the validity-analysis pass over the private copy is reserved
        // from the source census (the copy preserves the occurrence count).
        if (!tb::ReserveTraversal(baseCensus, budget, tb::Site::C19R2Prefix)) {
            refusal = et::Refusal::Budget; return false;
        }
        if (!BRepCheck_Analyzer(current).IsValid()) {
            refusal = stop.load() ? et::Refusal::Cancelled : et::Refusal::UnsupportedBase; return false;
        }
        for (const auto& step : program.steps) {
            if (stop.load() || !budget.chargeStage(1, tb::Site::C19R2Prefix)) {
                refusal = stop.load() ? et::Refusal::Cancelled : et::Refusal::Budget; return false;
            }
            analytic_boolean::Recipe stepRecipe;
            stepRecipe.operation = step.operation;
            stepRecipe.metersPerUnit = program.source.metersPerUnit;
            stepRecipe.tool = step.operand;
            analytic_boolean::Result stepResult;
            // Wedge expectation: derive the admitted removed volume from the
            // validated authored source recipe and the operand boundary proof
            // -- the same contract the saved-program path uses -- never from
            // the Boolean result under test. The independent volume check in
            // analytic_boolean::Build stays in force; non-wedge steps keep 0.
            double expectedWedgeVolume = 0;
            if (step.operand.kind == analytic_boolean::OperandKind::Wedge) {
                saved_cut_whole_result::Expected expected;
                const auto view = saved_boolean_result::detail::GeometryView(program, step.operand);
                if (!saved_cut_whole_result::ExpectedSource(view, expected)) {
                    refusal = et::Refusal::BuildFailed; return false;
                }
                const auto admitted = analytic_boolean_wedge::ExpectedBoundary(view, step.operand, expected);
                if (admitted.status != analytic_boolean_wedge::Status::Clear) {
                    refusal = et::Refusal::BuildFailed; return false;
                }
                expectedWedgeVolume = admitted.removedVolume;
            }
            // C19/C25/C26: the analytic step borrows this operation's shared
            // counter; its raw occurrences, passes and Boolean stage debit
            // are charged there. BudgetExceeded stays Budget through here.
            const auto status = analytic_boolean::Build(current, stepRecipe, stop, stepResult,
                expectedWedgeVolume, budget);
            if (status == analytic_boolean::Status::Cancelled) { refusal = et::Refusal::Cancelled; return false; }
            if (status == analytic_boolean::Status::BudgetExceeded) { refusal = et::Refusal::Budget; return false; }
            if (status != analytic_boolean::Status::Built || stepResult.solid.IsNull()) {
                refusal = et::Refusal::BuildFailed; return false;
            }
            current = stepResult.solid;
        }
        if (stop.load() || current.IsNull()) {
            refusal = stop.load() ? et::Refusal::Cancelled : et::Refusal::BuildFailed; return false;
        }
        // C19: the post-prefix validity-analysis pass is charged.
        const tb::WalkStatus postWalk = tb::ChargeTraversal(current, budget, stop,
            tb::Site::C19R2Prefix);
        if (postWalk == tb::WalkStatus::Cancelled) { refusal = et::Refusal::Cancelled; return false; }
        if (postWalk == tb::WalkStatus::BudgetDenied) { refusal = et::Refusal::Budget; return false; }
        if (postWalk != tb::WalkStatus::Completed) { refusal = et::Refusal::BuildFailed; return false; }
        if (!BRepCheck_Analyzer(current).IsValid()) {
            refusal = stop.load() ? et::Refusal::Cancelled : et::Refusal::BuildFailed; return false;
        }
        stages.postBoolean = current;
        if (program.filletSteps.empty()) {
            stages.postTreatment = current; refusal = et::Refusal::None; return true;
        }
        // Convert the legacy tail to the same suffix steps migration imports
        // (amount and witnesses in world millimetres) and replay each exactly
        // once on its own preceding stage. Identities are deterministic value
        // derivations of the legacy step/edge IDs; they never leave this
        // preparation. The synthetic binding commits to the exact recipe
        // bytes; only the replayed stages are consumed.
        et::Definition suffix;
        suffix.schema = 2;
        suffix.owner = {program.source.document, program.source.entity, program.source.definition};
        suffix.base.family = et::SourceFamily::Profile;
        suffix.base.source = {program.source.document, program.source.entity,
            program.source.definition, program.source.sourceFeature};
        suffix.base.sourceNode = program.source.derivedFeature;
        suffix.base.sourceSchema = program.source.schema;
        suffix.base.metersPerLocalUnit = program.source.metersPerUnit;
        std::vector<std::uint8_t> authority;
        if (!retained_boolean::Encode(program, authority)
            || !retained_solid::Hash(authority, suffix.base.sourceRecipeDigest)) {
            refusal = et::Refusal::MalformedCarrier; return false;
        }
        suffix.issuance.nextLocalID = program.nextFilletStepID;
        const double mmPerLocal = program.source.metersPerUnit * 1000;
        for (const auto& old : program.filletSteps) {
            et::Step step; step.localID = old.stepIdentifier;
            step.kind = et::Kind::ConstantFillet; step.amountMM = old.radiusLocal * mmPerLocal;
            for (unsigned byte = 0; byte < 8; ++byte) {
                step.node[byte] = std::uint8_t(old.stepIdentifier >> (8 * byte));
                step.feature[byte] = std::uint8_t(old.stepIdentifier >> (8 * byte));
            }
            step.node[8] = 1; step.feature[8] = 2;
            for (const auto& oldAnchor : old.anchors) {
                et::Anchor anchor;
                for (unsigned byte = 0; byte < 8; ++byte) {
                    anchor.key[byte] = std::uint8_t(old.stepIdentifier >> (8 * byte));
                    anchor.key[8 + byte] = std::uint8_t(oldAnchor.identifier >> (8 * byte));
                }
                anchor.curve = et::CurveKind(oldAnchor.curveKind);
                for (unsigned axis = 0; axis < 3; ++axis) {
                    anchor.pointMM[axis] = oldAnchor.anchorPoint[axis] * mmPerLocal;
                    anchor.tangent[axis] = oldAnchor.axis[axis];
                }
                anchor.normalA = {1, 0, 0}; anchor.normalB = {0, 1, 0};
                anchor.circleRadiusMM = oldAnchor.circleRadius * mmPerLocal;
                step.anchors.push_back(anchor);
            }
            std::sort(step.anchors.begin(), step.anchors.end(),
                [](const et::Anchor& a, const et::Anchor& b) { return a.key < b.key; });
            suffix.steps.push_back(step);
        }
        suffix.outputNode = suffix.steps.back().node;
        for (const auto& step : suffix.steps) {
            stages.stepStages.push_back(current);
            et::Definition single = suffix;
            single.steps = {step};
            single.outputNode = step.node;
            TopoDS_Shape next; std::vector<et::StepProof> proofs;
            if (!et::Replay(current, single, next, proofs, budget, refusal)) { stages = {}; return false; }
            if (proofs.size() != 1 || stop.load() || next.IsNull()) {
                stages = {};
                refusal = stop.load() ? et::Refusal::Cancelled : et::Refusal::ReplayMismatch; return false;
            }
            // C19: the per-stage validity-analysis pass after each old-tail
            // replay is charged to the same operation budget.
            const tb::WalkStatus tailWalk = tb::ChargeTraversal(next, budget, stop,
                tb::Site::C19R2Prefix);
            if (tailWalk == tb::WalkStatus::Cancelled) {
                stages = {}; refusal = et::Refusal::Cancelled; return false;
            }
            if (tailWalk == tb::WalkStatus::BudgetDenied) {
                stages = {}; refusal = et::Refusal::Budget; return false;
            }
            if (tailWalk != tb::WalkStatus::Completed
                || !BRepCheck_Analyzer(next).IsValid()) {
                stages = {};
                refusal = stop.load() ? et::Refusal::Cancelled : et::Refusal::ReplayMismatch; return false;
            }
            current = next;
        }
        stages.postTreatment = current;
        refusal = et::Refusal::None;
        return true;
    } catch (...) { stages = {}; refusal = et::Refusal::BuildFailed; return false; }
}

// Detached source-base rebuild for an admitted Boolean-input recipe edit.
// The envelope carries the proven edited recipe values and the legacy first
// operand; nullptr keeps the existing detached loft-base replay exactly.
using SourceBaseBuilder = bool (*)(const retained_solid::Envelope&,
    const std::atomic_bool&, et::ReplayBudget&, TopoDS_Shape&) noexcept;

// Bounded prefix rebuild for the admitted R2 source edits (analytic tool and
// profile/enclosure/rectangular-loft Boolean input). The complete edited
// program is validated and re-encoded BEFORE any geometry work; only the
// declared value fields may
// have moved (identity UUIDs, high-water IDs, step count/order, operand IDs,
// operations, unit and schema bits are proven exact). The prefix is then
// rebuilt from the proven pre-Boolean source base — itself rebuilt through
// the injected detached base rebuild for an input edit — by replaying
// every analytic step, and only the resulting prefix shape is handed to
// ReplayTreatmentSuffix. Any violation refuses; nothing is defaulted.
inline bool RebuildEditedPrefix(const RetainedBooleanBase& original,
                                const Edit& edit,
                                const TopoDS_Shape& sourceBase,
                                const std::atomic_bool& stop,
                                RetainedBooleanBase& editedSource,
                                std::vector<std::uint8_t>& editedBytes,
                                TopoDS_Shape& prefixResult,
                                TopoDS_Shape& editedSourceBase,
                                et::ReplayBudget& budget,
                                et::Refusal& refusal,
                                SourceBaseBuilder rebuildBase = nullptr) noexcept {
    try {
        editedBytes.clear(); editedSourceBase = TopoDS_Shape();
        const auto* legacy = std::get_if<LegacyBooleanBase>(&original.source);
        retained_boolean::Program promoted;
        const auto* program = legacy ? std::get_if<retained_boolean::Program>(&legacy->prefix) : nullptr;
        if (!program && legacy) {
            const auto* old = std::get_if<retained_boolean::Legacy>(&legacy->prefix);
            if (!old || !retained_boolean::Promote(*old, promoted)) {
                refusal = et::Refusal::MalformedCarrier; return false;
            }
            program = &promoted;
        }
        if (!program || sourceBase.IsNull()) { refusal = et::Refusal::UnsupportedBase; return false; }
        retained_boolean::Program edited = *program;
        if (const auto* tool = std::get_if<RebuildAnalyticTool>(&edit)) {
            auto step = std::find_if(edited.steps.begin(), edited.steps.end(),
                [&](const retained_boolean::Step& value) { return value.operand.identifier == tool->operandID; });
            if (!tool->operandID || step == edited.steps.end()
                || tool->requested.identifier != tool->operandID) {
                refusal = et::Refusal::IdentityMismatch; return false;
            }
            const analytic_boolean::Operand requested = tool->requested;
            for (double value : requested.point)
                if (!std::isfinite(value)) { refusal = et::Refusal::MalformedCarrier; return false; }
            // Kind-sensitive radius floor, mirroring analytic_boolean::Inspect:
            // a wedge carries radius exactly zero by contract; every other kind
            // keeps the finite positive requirement. The complete edited
            // program is re-validated below before any geometry work.
            const bool wedgeTool = requested.kind == analytic_boolean::OperandKind::Wedge;
            if (!std::isfinite(requested.radius)
                || (wedgeTool ? (requested.radius != 0 || std::signbit(requested.radius))
                              : requested.radius <= 0)
                || !std::isfinite(requested.boltCircleRadius) || requested.boltCircleRadius < 0
                || !std::isfinite(requested.hostRadiusRatio) || requested.hostRadiusRatio < 0
                || !std::isfinite(requested.directionAngle) || !std::isfinite(requested.halfWidthApex)
                || !std::isfinite(requested.halfWidthMouth) || !std::isfinite(requested.length)
                || requested.length < 0) { refusal = et::Refusal::MalformedCarrier; return false; }
            step->operand = requested;
        } else if (const auto* rebuild = std::get_if<RebuildBooleanInput>(&edit)) {
            // The requested recipe must name the proven source family exactly
            // and keep the unit bit-for-bit; only values and schema may move.
            if (const auto* p = std::get_if<profile::Parameters>(&rebuild->requested)) {
                if (edited.source.family != 1
                    || p->metersPerUnit != edited.source.metersPerUnit) {
                    refusal = et::Refusal::UnsupportedBase; return false;
                }
                std::vector<double> values;
                if (!profile::Encode(*p, values)) { refusal = et::Refusal::MalformedCarrier; return false; }
                edited.source.values = std::move(values);
                edited.source.schema = std::uint32_t(profile::SchemaFor(*p));
            } else if (const auto* e = std::get_if<enclosure::Parameters>(&rebuild->requested)) {
                if (edited.source.family != 2
                    || e->metersPerUnit != edited.source.metersPerUnit) {
                    refusal = et::Refusal::UnsupportedBase; return false;
                }
                std::vector<double> values;
                if (!enclosure::Encode(*e, values)) { refusal = et::Refusal::MalformedCarrier; return false; }
                edited.source.values = std::move(values);
                edited.source.schema = e->definition.constructionFrame ? 2 : 1;
            } else if (const auto* loft = std::get_if<rectangular_loft::Definition>(&rebuild->requested)) {
                if (edited.source.family != 3
                    || loft->dimensionMetersPerUnit != edited.source.metersPerUnit) {
                    refusal = et::Refusal::UnsupportedBase; return false;
                }
                std::vector<double> values;
                if (!loft_persistence::Encode(*loft, values)) { refusal = et::Refusal::MalformedCarrier; return false; }
                edited.source.values = std::move(values);
                edited.source.schema = loft_persistence::Schema;
            } else { refusal = et::Refusal::UnsupportedBase; return false; }
        } else { refusal = et::Refusal::UnsupportedDependency; return false; }
        // Identity fence: only the declared value fields may have moved.
        if (edited.source.document != program->source.document || edited.source.entity != program->source.entity
            || edited.source.definition != program->source.definition
            || edited.source.sourceFeature != program->source.sourceFeature
            || edited.source.derivedFeature != program->source.derivedFeature
            || edited.source.metersPerUnit != program->source.metersPerUnit
            || edited.nextOperandID != program->nextOperandID
            || edited.nextFilletStepID != program->nextFilletStepID
            || edited.nextFilletEdgeID != program->nextFilletEdgeID
            || edited.steps.size() != program->steps.size()
            || edited.filletSteps.size() != program->filletSteps.size()) {
            refusal = et::Refusal::IdentityMismatch; return false;
        }
        for (std::size_t index = 0; index < edited.steps.size(); ++index)
            if (edited.steps[index].operand.identifier != program->steps[index].operand.identifier
                || edited.steps[index].operation != program->steps[index].operation) {
                refusal = et::Refusal::IdentityMismatch; return false;
            }
        if (!retained_boolean::Valid(edited) || !retained_boolean::Encode(edited, editedBytes)) {
            refusal = et::Refusal::MalformedCarrier; return false;
        }
        // Rebuild the pre-Boolean source base when the input recipe moved.
        editedSourceBase = sourceBase;
        if (std::holds_alternative<RebuildBooleanInput>(edit)) {
            retained_solid::Envelope envelope;
            envelope.document = edited.source.document; envelope.entity = edited.source.entity;
            envelope.definition = edited.source.definition; envelope.sourceFeature = edited.source.sourceFeature;
            envelope.derivedFeature = edited.source.derivedFeature; envelope.sourceFamily = edited.source.family;
            envelope.sourceSchema = edited.source.schema; envelope.metersPerUnit = edited.source.metersPerUnit;
            envelope.sourceValues = edited.source.values;
            TopoDS_Shape rebuilt;
            if (rebuildBase) {
                // The boundary proof requires the complete legacy envelope:
                // the source fields above plus the first (promoted legacy)
                // operand, exactly as GeometryView forms it. Only the builder
                // path completes the operand fields; the default loft replay
                // below keeps its previous envelope byte-identically.
                const auto& first = edited.steps.front().operand;
                envelope.operandID = first.identifier; envelope.axis = std::uint8_t(first.axis);
                envelope.point = first.point;
                envelope.radius = first.kind == analytic_boolean::OperandKind::Wedge
                    ? .001 / (edited.source.metersPerUnit * 1000) : first.radius;
                if (!rebuildBase(envelope, stop, budget, rebuilt) || rebuilt.IsNull()) {
                    refusal = stop.load() ? et::Refusal::Cancelled : et::Refusal::BuildFailed; return false;
                }
            } else {
                const auto status = saved_cut_source_edit::RebuildLoftBase(envelope, stop, rebuilt);
                if (status == saved_cut_source_edit::LoftBaseStatus::Cancelled) { refusal = et::Refusal::Cancelled; return false; }
                if (status != saved_cut_source_edit::LoftBaseStatus::Built || rebuilt.IsNull()) {
                    refusal = et::Refusal::BuildFailed; return false;
                }
            }
            editedSourceBase = rebuilt;
        }
        // Replay the complete edited analytic prefix on the proven source base.
        // C20: no locally fresh counters — the operation's shared budget pays
        // the bounded input census and the reserved private-copy pass.
        tb::Census inputCensus;
        const tb::WalkStatus inputWalk = tb::CensusTopology(editedSourceBase, budget, stop,
            inputCensus, tb::Site::C20R2EditedPrefix, true);
        if (inputWalk == tb::WalkStatus::Cancelled) { refusal = et::Refusal::Cancelled; return false; }
        if (inputWalk != tb::WalkStatus::Completed
            || !tb::ReserveTraversal(inputCensus, budget, tb::Site::C20R2EditedPrefix)) {
            refusal = et::Refusal::Budget; return false;
        }
        const TopoDS_Shape copied = BRepBuilderAPI_Copy(editedSourceBase).Shape();
        // D365: the private copy is this operation's geometry producer, and
        // OCCT's binary persistence phase re-constructs direction records from
        // their stored components on readback (gp_Dir2d.hxx SetCoord), which
        // can move a component by one binary64 step; the C12 detachment proof
        // is right to refuse such a shape. Canonicalise at the producer through
        // the same bounded BinTools round trip the proof itself trusts, so the
        // replayed prefix commits to geometry whose exact readback reproduces
        // it. No tolerance, no canonicalisation at the proof, no budget change.
        et::detail::GeometryBuffer copyBuffer; std::ostream copyWriter(&copyBuffer);
        BinTools::Write(copied, copyWriter, Standard_False, Standard_False, BinTools_FormatVersion_VERSION_4);
        if (!copyWriter.good() || copyBuffer.size() == 0 || copyBuffer.size() >= et::detail::MaximumGeometryBytes) {
            refusal = et::Refusal::Budget; return false;
        }
        copyBuffer.read(); std::istream copyReader(&copyBuffer);
        TopoDS_Shape current;
        BinTools::Read(current, copyReader);
        if (!copyReader.good() || current.IsNull()) { refusal = et::Refusal::BuildFailed; return false; }
        for (const auto& step : edited.steps) {
            if (stop.load() || !budget.chargeStage(1, tb::Site::C20R2EditedPrefix)) {
                refusal = stop.load() ? et::Refusal::Cancelled : et::Refusal::Budget; return false;
            }
            analytic_boolean::Recipe stepRecipe;
            stepRecipe.operation = step.operation;
            stepRecipe.metersPerUnit = edited.source.metersPerUnit;
            stepRecipe.tool = step.operand;
            analytic_boolean::Result stepResult;
            // Wedge expectation: the same authored-source / operand boundary
            // proof as above, derived from the validated edited program.
            double expectedWedgeVolume = 0;
            if (step.operand.kind == analytic_boolean::OperandKind::Wedge) {
                saved_cut_whole_result::Expected expected;
                const auto view = saved_boolean_result::detail::GeometryView(edited, step.operand);
                if (!saved_cut_whole_result::ExpectedSource(view, expected)) {
                    refusal = et::Refusal::BuildFailed; return false;
                }
                const auto admitted = analytic_boolean_wedge::ExpectedBoundary(view, step.operand, expected);
                if (admitted.status != analytic_boolean_wedge::Status::Clear) {
                    refusal = et::Refusal::BuildFailed; return false;
                }
                expectedWedgeVolume = admitted.removedVolume;
            }
            const auto status = analytic_boolean::Build(current, stepRecipe, stop, stepResult,
                expectedWedgeVolume, budget);
            if (status == analytic_boolean::Status::Cancelled) { refusal = et::Refusal::Cancelled; return false; }
            if (status == analytic_boolean::Status::BudgetExceeded) { refusal = et::Refusal::Budget; return false; }
            if (status != analytic_boolean::Status::Built || stepResult.solid.IsNull()) {
                refusal = et::Refusal::BuildFailed; return false;
            }
            current = stepResult.solid;
        }
        if (stop.load() || current.IsNull()) {
            refusal = stop.load() ? et::Refusal::Cancelled : et::Refusal::BuildFailed; return false;
        }
        // C20: the candidate checks — post-prefix validity-analysis pass and
        // the bounded output census — are charged before publication.
        const tb::WalkStatus prefixWalk = tb::ChargeTraversal(current, budget, stop,
            tb::Site::C20R2EditedPrefix);
        if (prefixWalk == tb::WalkStatus::Cancelled) { refusal = et::Refusal::Cancelled; return false; }
        if (prefixWalk == tb::WalkStatus::BudgetDenied) { refusal = et::Refusal::Budget; return false; }
        if (prefixWalk != tb::WalkStatus::Completed
            || !BRepCheck_Analyzer(current).IsValid()) {
            refusal = stop.load() ? et::Refusal::Cancelled : et::Refusal::BuildFailed; return false;
        }
        tb::Census prefixCensus;
        const tb::WalkStatus censusWalk = tb::CensusTopology(current, budget, stop,
            prefixCensus, tb::Site::C20R2EditedPrefix, false);
        if (censusWalk == tb::WalkStatus::Cancelled) { refusal = et::Refusal::Cancelled; return false; }
        if (censusWalk != tb::WalkStatus::Completed) { refusal = et::Refusal::Budget; return false; }
        prefixResult = current;
        editedSource = RetainedBooleanBase{original.binding, LegacyBooleanBase{edited, editedBytes}};
        refusal = et::Refusal::None;
        return true;
    } catch (...) { refusal = et::Refusal::BuildFailed; return false; }
}

// R2 moved-witness rebind (CONTRACT.md "Source/input/tool edit"). The
// old-stage selector roles were captured on the main-thread authority side
// through et::CaptureSourceRebindRoles. Here, against the actually rebuilt
// prefix result, each unchanged semantic intent is freshly resolved, every
// stored witness is rebound through the unique source-local role
// correspondence — position excluded, which is exactly what a source edit
// moves — and the complete suffix is strictly replayed on its proper
// preceding stage, mirroring et::ApplySourceRebind. The R2 Boolean binding
// digest is not touched here: the caller re-digests it against the exact
// edited prefix bytes. Missing or ambiguous correspondence refuses; nothing
// is defaulted.
inline bool ApplySourceRebindR2(const et::SourceRebindRoles& roles,
                                const TopoDS_Shape& newStage,
                                Definition& candidate,
                                TopoDS_Shape& treated,
                                et::ReplayBudget& budget,
                                et::Refusal& refusal) noexcept {
    treated.Nullify();
    try {
        if (newStage.IsNull()) { refusal = et::Refusal::ReplayMismatch; return false; }
        const std::atomic_bool neverCancelled{false};
        // C18: the new-stage census is debited before the analyzer, and the
        // analyzer pass is reserved; a budget failure is never mapped to
        // ReplayMismatch.
        tb::Census newStageCensus;
        if (tb::CensusTopology(newStage, budget, neverCancelled, newStageCensus,
                tb::Site::C18SourceRebindNew, true) != tb::WalkStatus::Completed
            || !tb::ReserveTraversal(newStageCensus, budget, tb::Site::C18SourceRebindNew)) {
            refusal = et::Refusal::Budget; return false;
        }
        if (!BRepCheck_Analyzer(newStage).IsValid()) {
            refusal = et::Refusal::ReplayMismatch; return false;
        }
        const auto* binding = std::get_if<BooleanBaseBinding>(&candidate.base);
        if (!binding) { refusal = et::Refusal::MalformedCarrier; return false; }
        et::BaseBinding shadow;
        shadow.source = binding->source; shadow.sourceNode = binding->sourceNode;
        shadow.sourceSchema = binding->sourceSchema;
        shadow.sourceRecipeDigest = binding->sourceRecipeDigest;
        shadow.metersPerLocalUnit = binding->metersPerLocalUnit;
        et::Definition shadowBase{candidate.schema, candidate.owner, shadow,
                                  candidate.issuance, candidate.outputNode, {}};
        Definition rebound = candidate;
        TopoDS_Shape current = newStage;
        std::size_t selectorIndex = 0;
        for (std::size_t index = 0; index < rebound.steps.size(); ++index) {
            et::Step step = rebound.steps[index];
            if (step.selector) {
                if (selectorIndex >= roles.selectorSteps.size()
                    || roles.selectorSteps[selectorIndex].first != step.feature
                    || roles.selectorSteps[selectorIndex].second.size() != step.anchors.size()) {
                    refusal = et::Refusal::ReplayMismatch; return false;
                }
                const auto& oldRoles = roles.selectorSteps[selectorIndex].second;
                ++selectorIndex;
                retained_face_selector::Resolution resolution;
                const auto selectorRefusal = retained_face_selector::Resolve(current,
                    step.selector->intent, binding->metersPerLocalUnit, budget,
                    neverCancelled, resolution);
                if (selectorRefusal != retained_face_selector::Refusal::None || !resolution.proof) {
                    refusal = retained_face_selector::MapToB1(selectorRefusal); return false;
                }
                const auto& proof = *resolution.proof;
                if (proof.selectedEdgeCount() != step.anchors.size()) {
                    refusal = et::Refusal::ReplayMismatch; return false;
                }
                std::vector<et::SelectorUseRole> measured;
                for (const auto& use : proof.boundaryUses()) {
                    // C18: every new-side witness loop entry is charged.
                    if (!budget.visit(1, tb::Site::C18SourceRebindNew)) {
                        refusal = et::Refusal::Budget; return false;
                    }
                    et::SelectorUseRole role;
                    if (use.selected
                        && !et::detail::MeasureUseWitness(use, binding->metersPerLocalUnit,
                            role.curve, role.pointMM, role.tangent, role.normalA, role.normalB,
                            role.circleRadiusMM, &budget, tb::Site::C18SourceRebindNew)) {
                        refusal = budget.exhausted ? et::Refusal::Budget
                            : et::Refusal::UnsupportedEdge; return false;
                    }
                    measured.push_back(role);
                }
                // Unique one-to-one correspondence from each old selected use
                // to a new selected use by source-local boundary role.
                std::set<int> consumed;
                std::vector<et::SelectorUseRole> matchedNew;
                for (std::size_t anchorIndex = 0; anchorIndex < step.anchors.size();
                    ++anchorIndex) {
                    const et::SelectorUseRole& oldRole = oldRoles[anchorIndex];
                    if (oldRole.anchorKey != step.anchors[anchorIndex].key) {
                        refusal = et::Refusal::ReplayMismatch; return false;
                    }
                    int found = -1;
                    for (std::size_t useIndex = 0; useIndex < measured.size(); ++useIndex) {
                        // C18: every new-side role correspondence comparison
                        // is charged, including nonmatches.
                        if (!budget.visit(1, tb::Site::C18SourceRebindNew)) {
                            refusal = et::Refusal::Budget; return false;
                        }
                        if (!proof.boundaryUses()[useIndex].selected
                            || consumed.count(int(useIndex))) continue;
                        const et::SelectorUseRole& candidateRole = measured[useIndex];
                        if (candidateRole.curve == oldRole.curve
                            && et::detail::SameDirection(candidateRole.tangent, oldRole.tangent)
                            && et::detail::SameDirection(candidateRole.normalA, oldRole.normalA)
                            && et::detail::SameDirection(candidateRole.normalB, oldRole.normalB)
                            && std::abs(candidateRole.circleRadiusMM - oldRole.circleRadiusMM)
                                <= 1e-4) {
                            if (found >= 0) { refusal = et::Refusal::AnchorAmbiguous; return false; }
                            found = int(useIndex);
                        }
                    }
                    if (found < 0) { refusal = et::Refusal::AnchorMissing; return false; }
                    consumed.insert(found);
                    et::SelectorUseRole role = measured[std::size_t(found)];
                    role.anchorKey = oldRole.anchorKey;
                    role.direction = proof.boundaryUses()[std::size_t(found)].direction;
                    matchedNew.push_back(role);
                }
                if (consumed.size() != proof.selectedEdgeCount()) {
                    refusal = et::Refusal::ReplayMismatch; return false;
                }
                // Carry identity, amount and semantic intent forward verbatim;
                // refresh only the derived witness/receipt content permitted
                // by the proved correspondence.
                retained_face_selector::SelectorReceipt receipt = *step.selector;
                receipt.face = proof.plane();
                receipt.coverage = proof.coverage();
                receipt.wireCount = proof.wireCount();
                receipt.boundaryUseCount = std::uint32_t(proof.boundaryUses().size());
                receipt.boundaryUniqueEdgeCount = proof.uniqueEdgeCount();
                for (std::size_t anchorIndex = 0; anchorIndex < step.anchors.size();
                    ++anchorIndex) {
                    et::Anchor& anchor = step.anchors[anchorIndex];
                    const et::SelectorUseRole& role = matchedNew[anchorIndex];
                    anchor.curve = role.curve;
                    anchor.pointMM = role.pointMM;
                    anchor.tangent = role.tangent;
                    anchor.normalA = role.normalA;
                    anchor.normalB = role.normalB;
                    anchor.circleRadiusMM = role.circleRadiusMM;
                    receipt.entries[anchorIndex] = {role.anchorKey, role.direction};
                }
                step.selector = std::move(receipt);
            }
            et::Definition single = shadowBase;
            single.steps = {step};
            single.outputNode = step.node;
            TopoDS_Shape next; std::vector<et::StepProof> proofs;
            if (!et::Replay(current, single, next, proofs, budget, refusal)) {
                return false;
            }
            if (proofs.size() != 1) { refusal = et::Refusal::ReplayMismatch; return false; }
            rebound.steps[index] = step;
            current = next;
        }
        if (selectorIndex != roles.selectorSteps.size()) {
            refusal = et::Refusal::ReplayMismatch; return false;
        }
        candidate = std::move(rebound);
        treated = current;
        refusal = et::Refusal::None;
        return true;
    } catch (...) { treated.Nullify(); refusal = et::Refusal::BuildFailed; return false; }
}

// Compatibility signature: a fresh budget for independent callers outside a
// B2 operation. Operation-internal callers must use the budgeted overload.
inline bool RebuildEditedPrefix(const RetainedBooleanBase& original,
                                const Edit& edit,
                                const TopoDS_Shape& sourceBase,
                                const std::atomic_bool& stop,
                                RetainedBooleanBase& editedSource,
                                std::vector<std::uint8_t>& editedBytes,
                                TopoDS_Shape& prefixResult,
                                TopoDS_Shape& editedSourceBase,
                                et::Refusal& refusal) noexcept {
    et::ReplayBudget fresh;
    return RebuildEditedPrefix(original, edit, sourceBase, stop, editedSource, editedBytes,
        prefixResult, editedSourceBase, fresh, refusal);
}
} // namespace core3d::retained_edge_treatment::r2
