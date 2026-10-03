#pragma once

#include "RetainedEdgeTreatmentR2Snapshot.hxx"
#include "RetainedEdgeTreatmentBuild.hxx"
#include "RetainedTopologyBudget.hxx"
#include "CompositeRecipeCodec.hxx"
#include "AnalyticBooleanSolid.hxx"
#include "SavedBooleanResultCorrespondence.hxx" // saved-program wedge expectation contract
#include "SavedCutSourceEdit.hxx"
#include "RectangularLoftPersistence.hxx"
#include "RetainedSolidEnvelope.hxx"
#include <BRepCheck_Analyzer.hxx>
#include <BRepBuilderAPI_Copy.hxx>
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

// Bounded prefix rebuild for the admitted R2 source edits (analytic tool and
// rectangular-loft Boolean input). The complete edited program is validated
// and re-encoded BEFORE any geometry work; only the declared value fields may
// have moved (identity UUIDs, high-water IDs, step count/order, operand IDs,
// operations, unit and schema bits are proven exact). The prefix is then
// rebuilt from the proven pre-Boolean source base — itself rebuilt through
// the existing detached loft-base replay for an input edit — by replaying
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
                                et::Refusal& refusal) noexcept {
    try {
        editedBytes.clear(); editedSourceBase = TopoDS_Shape();
        const auto* legacy = std::get_if<LegacyBooleanBase>(&original.source);
        const auto* program = legacy ? std::get_if<retained_boolean::Program>(&legacy->prefix) : nullptr;
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
            if (!std::isfinite(requested.radius) || requested.radius <= 0
                || !std::isfinite(requested.boltCircleRadius) || requested.boltCircleRadius < 0
                || !std::isfinite(requested.hostRadiusRatio) || requested.hostRadiusRatio < 0
                || !std::isfinite(requested.directionAngle) || !std::isfinite(requested.halfWidthApex)
                || !std::isfinite(requested.halfWidthMouth) || !std::isfinite(requested.length)
                || requested.length < 0) { refusal = et::Refusal::MalformedCarrier; return false; }
            step->operand = requested;
        } else if (const auto* rebuild = std::get_if<RebuildBooleanInput>(&edit)) {
            const auto* loft = std::get_if<rectangular_loft::Definition>(&rebuild->requested);
            // Only the rectangular-loft input family has an existing detached
            // base rebuild; profile/enclosure input rebuilds refuse at
            // preparation rather than inventing a construction path.
            if (!loft || edited.source.family != 3
                || loft->dimensionMetersPerUnit != edited.source.metersPerUnit) {
                refusal = et::Refusal::UnsupportedBase; return false;
            }
            std::vector<double> values;
            if (!loft_persistence::Encode(*loft, values)) { refusal = et::Refusal::MalformedCarrier; return false; }
            edited.source.values = std::move(values);
            edited.source.schema = loft_persistence::Schema;
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
            const auto status = saved_cut_source_edit::RebuildLoftBase(envelope, stop, rebuilt);
            if (status == saved_cut_source_edit::LoftBaseStatus::Cancelled) { refusal = et::Refusal::Cancelled; return false; }
            if (status != saved_cut_source_edit::LoftBaseStatus::Built || rebuilt.IsNull()) {
                refusal = et::Refusal::BuildFailed; return false;
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
        TopoDS_Shape current = BRepBuilderAPI_Copy(editedSourceBase).Shape();
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
