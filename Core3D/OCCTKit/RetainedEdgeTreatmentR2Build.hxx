#pragma once

#include "RetainedEdgeTreatmentR2Snapshot.hxx"
#include "RetainedEdgeTreatmentBuild.hxx"
#include "RetainedProfileProducer.hxx"
#include "RetainedTopologyBudget.hxx"
#include "CompositeRecipeCodec.hxx"
#include "AnalyticBooleanSolid.hxx"
#include "SavedBooleanResultCorrespondence.hxx" // saved-program wedge expectation contract
#include "SavedCutSourceEdit.hxx"
#include "RectangularLoftPersistence.hxx"
#include "ProfilePersistence.hxx"
#include "ProfileSourceCorrespondence.hxx"
#include "EnclosurePersistence.hxx"
#include "RetainedSolidEnvelope.hxx"
#include <BRepCheck_Analyzer.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BinTools.hxx>
#include <algorithm>

namespace core3d::retained_edge_treatment::r2 {
namespace tb = core3d::retained_topology_budget;
enum class FramedLoftFixedPointStatus : std::uint8_t {
    Adopted, Distinct, Cancelled, BudgetExceeded
};

#if DEBUG
inline std::atomic<int> DebugFramedLoftFixedPointMode{0};
inline std::atomic<unsigned> DebugFramedLoftFixedPointReached{0};
inline std::atomic<int> DebugFramedLoftFixedPointLastStatus{-1};
#endif

// Finite private-producer materialization for a decoded, construction-framed
// rectangular-loft Boolean product. Exactly two V4 readbacks are compared by
// the unchanged V3 commitment. Callers must reject every result except Adopted.
inline FramedLoftFixedPointStatus MaterializePrivateFramedLoftFixedPointImpl(
    const TopoDS_Shape& privateProduct, const std::atomic_bool& stop,
    et::ReplayBudget& budget, TopoDS_Shape& output) noexcept {
    output.Nullify();
    if (stop.load()) return FramedLoftFixedPointStatus::Cancelled;
    if (budget.exhausted) return FramedLoftFixedPointStatus::BudgetExceeded;
    if (privateProduct.IsNull()) return FramedLoftFixedPointStatus::Distinct;
    try {
        const auto reconstruct = [&](const TopoDS_Shape& input,
            TopoDS_Shape& reconstructed) -> FramedLoftFixedPointStatus {
            tb::Census census;
            const tb::WalkStatus walked = tb::CensusTopology(input, budget, stop,
                census, tb::Site::C12Detachment, false);
            if (walked == tb::WalkStatus::Cancelled || stop.load())
                return FramedLoftFixedPointStatus::Cancelled;
            if (walked == tb::WalkStatus::BudgetDenied || budget.exhausted
                || !tb::ReserveTraversal(census, budget, tb::Site::C12Detachment))
                return FramedLoftFixedPointStatus::BudgetExceeded;
            if (walked != tb::WalkStatus::Completed
                || !et::detail::ReadbackGeometry(input, budget, reconstructed)) {
                if (stop.load()) return FramedLoftFixedPointStatus::Cancelled;
                return budget.exhausted ? FramedLoftFixedPointStatus::BudgetExceeded
                    : FramedLoftFixedPointStatus::Distinct;
            }
            return FramedLoftFixedPointStatus::Adopted;
        };
        TopoDS_Shape y1, y2;
        auto status = reconstruct(privateProduct, y1);
        if (status != FramedLoftFixedPointStatus::Adopted) return status;
        status = reconstruct(y1, y2);
        if (status != FramedLoftFixedPointStatus::Adopted) return status;
#if DEBUG
        if (DebugFramedLoftFixedPointMode.load(std::memory_order_relaxed) == 1)
            y2 = privateProduct;
#endif
        et::Digest y1Digest{}, y2Digest{};
        if (!et::detail::CommitGeometry(y1, budget, y1Digest)
            || !et::detail::CommitGeometry(y2, budget, y2Digest)) {
            if (stop.load()) return FramedLoftFixedPointStatus::Cancelled;
            return budget.exhausted ? FramedLoftFixedPointStatus::BudgetExceeded
                : FramedLoftFixedPointStatus::Distinct;
        }
        if (y1Digest != y2Digest) return FramedLoftFixedPointStatus::Distinct;
        output = y1;
        return FramedLoftFixedPointStatus::Adopted;
    } catch (...) {
        output.Nullify();
        return stop.load() ? FramedLoftFixedPointStatus::Cancelled
            : FramedLoftFixedPointStatus::Distinct;
    }
}

inline FramedLoftFixedPointStatus MaterializePrivateFramedLoftFixedPoint(
    const TopoDS_Shape& privateProduct, const std::atomic_bool& stop,
    et::ReplayBudget& budget, TopoDS_Shape& output) noexcept {
#if DEBUG
    DebugFramedLoftFixedPointReached.fetch_add(1, std::memory_order_relaxed);
#endif
    const auto status = MaterializePrivateFramedLoftFixedPointImpl(
        privateProduct, stop, budget, output);
#if DEBUG
    DebugFramedLoftFixedPointLastStatus.store(
        static_cast<int>(status), std::memory_order_relaxed);
#endif
    return status;
}

inline bool IsConstructionFramedLoft(const retained_boolean::Program& program) noexcept {
    if (program.source.family != 3) return false;
    rectangular_loft::Definition definition;
    return loft_persistence::Decode(program.source.values, definition)
        && bool(definition.constructionFrame);
}
inline bool EncodeCompleteProfileRequest(const profile::Parameters& requested,
                                         double capturedMetersPerUnit,
                                         std::vector<double>& values) noexcept {
    values.clear();
    try {
        profile::Parameters decoded;
        std::vector<double> roundTrip;
        if (retained_solid::Bits(requested.metersPerUnit)
                != retained_solid::Bits(capturedMetersPerUnit)
            || !profile::Encode(requested, values)
            || !profile::Decode(values, decoded)
            || !profile::Encode(decoded, roundTrip)
            || values.size() != roundTrip.size()) return false;
        for (std::size_t i = 0; i < values.size(); ++i)
            if (retained_solid::Bits(values[i])
                != retained_solid::Bits(roundTrip[i])) return false;
        return true;
    } catch (...) {
        values.clear();
        return false;
    }
}

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
inline bool ReplayTreatmentSuffixImpl(const TopoDS_Shape& postBoolean,
                                  const Definition& definition,
                                  TopoDS_Shape& output,
                                  et::ReplayBudget& budget,
                                  et::Refusal& refusal,
                                  const std::atomic_bool& stop,
                                  const et::SourceStageContext* sourceContext) noexcept {
    if (postBoolean.IsNull()) {
        refusal = et::Refusal::UnsupportedBase; return false;
    }
    // C19: the analyzer input pass is debited before validity checking.
    if (stop.load()) { refusal = et::Refusal::Cancelled; return false; }
    const auto inputWalk = tb::ChargeTraversal(
        postBoolean, budget, stop, tb::Site::C19R2Prefix);
    if (inputWalk
        != tb::WalkStatus::Completed) {
        refusal = inputWalk == tb::WalkStatus::Cancelled
            ? et::Refusal::Cancelled : et::Refusal::Budget; return false;
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
    return sourceContext
        ? et::Replay(postBoolean, suffix, output, proofs, budget, refusal,
            *sourceContext, stop)
        : et::Replay(postBoolean, suffix, output, proofs, budget, refusal, stop);
}
// A source-relative selector proves its literal predicate on the retained
// pre-Boolean source, while its immutable anchors identify the corresponding
// edges on each treatment working stage. Validate the authored receipt on
// that source stage first, then let the existing bounded replay resolve those
// anchors on the post-Boolean stage without incorrectly reapplying the
// source-only face predicate to a cut result (for example, an annular cap).
inline bool PrepareSourceRelativeReplay(const TopoDS_Shape& sourceStage,
                                  const Definition& definition,
                                  const et::SourceStageContext& sourceContext,
                                  Definition& runtimeDefinition,
                                  et::ReplayBudget& budget,
                                  et::Refusal& refusal,
                                  const std::atomic_bool& stop) noexcept {
    runtimeDefinition = definition;
    try {
        const auto* binding = std::get_if<BooleanBaseBinding>(&definition.base);
        if (sourceStage.IsNull()) {
            refusal = et::Refusal::UnsupportedBase; return false;
        }
        if (!binding) {
            refusal = et::Refusal::MalformedCarrier; return false;
        }
        for (std::size_t index = 0; index < definition.steps.size(); ++index) {
            const et::Step& step = definition.steps[index];
            if (!et::NeedsSourceStageContext(step)) continue;
            const auto* proof = et::RequireSourceStageProof(
                step, &sourceContext, refusal);
            if (!proof || !step.selector) return false;
            retained_face_selector::Resolution resolution;
            const auto selectorRefusal = retained_face_selector::Resolve(
                sourceStage, step.selector->intent, *proof,
                binding->metersPerLocalUnit, budget,
                stop, resolution);
            if (selectorRefusal != retained_face_selector::Refusal::None
                || !resolution.proof) {
                refusal = retained_face_selector::MapToB1(selectorRefusal);
                return false;
            }
            const auto receiptRefusal = retained_face_selector::VerifyReceipt(
                *resolution.proof, step, binding->metersPerLocalUnit, budget);
            if (receiptRefusal != retained_face_selector::Refusal::None) {
                refusal = retained_face_selector::MapToB1(receiptRefusal);
                return false;
            }
            runtimeDefinition.steps[index].selector.reset();
        }
        refusal = et::Refusal::None; return true;
    } catch (...) {
        refusal = stop.load() ? et::Refusal::Cancelled
            : et::Refusal::ReplayMismatch;
        return false;
    }
}
inline bool ReplayTreatmentSuffix(const TopoDS_Shape& sourceStage,
                                  const TopoDS_Shape& postBoolean,
                                  const Definition& definition,
                                  TopoDS_Shape& output,
                                  et::ReplayBudget& budget,
                                  et::Refusal& refusal,
                                  const et::SourceStageContext& sourceContext,
                                  const std::atomic_bool& stop) noexcept {
    Definition runtimeDefinition;
    if (!PrepareSourceRelativeReplay(sourceStage, definition, sourceContext,
            runtimeDefinition, budget, refusal, stop)) return false;
    return ReplayTreatmentSuffixImpl(postBoolean, runtimeDefinition, output,
        budget, refusal, stop, nullptr);
}
inline bool ReplayTreatmentSuffix(const TopoDS_Shape& postBoolean,
                                  const Definition& definition,
                                  TopoDS_Shape& output,
                                  et::ReplayBudget& budget,
                                  et::Refusal& refusal,
                                  const et::SourceStageContext& sourceContext,
                                  const std::atomic_bool& stop) noexcept {
    return ReplayTreatmentSuffixImpl(postBoolean, definition, output, budget,
        refusal, stop, &sourceContext);
}
inline bool ReplayTreatmentSuffix(const TopoDS_Shape& postBoolean,
                                  const Definition& definition,
                                  TopoDS_Shape& output,
                                  et::ReplayBudget& budget,
                                  et::Refusal& refusal,
                                  const std::atomic_bool& stop) noexcept {
    return ReplayTreatmentSuffixImpl(postBoolean, definition, output, budget,
        refusal, stop, nullptr);
}
inline bool ReplayTreatmentSuffix(const TopoDS_Shape& postBoolean,
                                  const Definition& definition,
                                  TopoDS_Shape& output,
                                  et::ReplayBudget& budget,
                                  et::Refusal& refusal,
                                  const et::SourceStageContext& sourceContext) noexcept {
    const std::atomic_bool neverCancelled{false};
    return ReplayTreatmentSuffix(postBoolean, definition, output, budget,
        refusal, sourceContext, neverCancelled);
}
inline bool ReplayTreatmentSuffix(const TopoDS_Shape& postBoolean,
                                  const Definition& definition,
                                  TopoDS_Shape& output,
                                  et::ReplayBudget& budget,
                                  et::Refusal& refusal) noexcept {
    const std::atomic_bool neverCancelled{false};
    return ReplayTreatmentSuffix(postBoolean, definition, output, budget,
        refusal, neverCancelled);
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
    bool stepStagesPhysicalMM = false;
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
        if (IsConstructionFramedLoft(program)) {
            TopoDS_Shape fixedPoint;
            const auto status = MaterializePrivateFramedLoftFixedPoint(
                current, stop, budget, fixedPoint);
            if (status == FramedLoftFixedPointStatus::Cancelled) {
                refusal = et::Refusal::Cancelled; return false;
            }
            if (status == FramedLoftFixedPointStatus::BudgetExceeded) {
                refusal = et::Refusal::Budget; return false;
            }
            if (status != FramedLoftFixedPointStatus::Adopted) {
                refusal = et::Refusal::ReplayMismatch; return false;
            }
            current = fixedPoint;
        } else if (program.source.family == 1 && program.source.values.size() > 2
            && program.source.values[2] == 1.0) {
            TopoDS_Shape fixedPoint;
            const auto status = retained_profile_producer::
                MaterializePrivateProfileProducerFixedPoint(
                    current, stop, budget, fixedPoint);
            if (status == retained_profile_producer::FixedPointStatus::Cancelled) {
                refusal = et::Refusal::Cancelled; return false;
            }
            if (status == retained_profile_producer::FixedPointStatus::BudgetExceeded) {
                refusal = et::Refusal::Budget; return false;
            }
            if (status == retained_profile_producer::FixedPointStatus::Adopted) {
                current = fixedPoint;
            }
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
        et::detail::WorkingStageSequence sequence;
        if (!et::detail::BeginWorkingStages(current, suffix, budget, refusal,
                stop, sequence)) { stages = {}; return false; }
        stages.stepStagesPhysicalMM = true;
        for (const auto& step : suffix.steps) {
            stages.stepStages.push_back(sequence.current);
            et::StepProof proof;
            if (!et::detail::BuildWorkingStage(suffix, step, sequence, proof,
                    budget, refusal, stop)) { stages = {}; return false; }
            if (stop.load() || sequence.current.IsNull()) {
                stages = {};
                refusal = stop.load() ? et::Refusal::Cancelled : et::Refusal::ReplayMismatch; return false;
            }
            // C19: the per-stage validity-analysis pass after each old-tail
            // replay is charged to the same operation budget.
            const tb::WalkStatus tailWalk = tb::ChargeTraversal(sequence.current, budget, stop,
                tb::Site::C19R2Prefix);
            if (tailWalk == tb::WalkStatus::Cancelled) {
                stages = {}; refusal = et::Refusal::Cancelled; return false;
            }
            if (tailWalk == tb::WalkStatus::BudgetDenied) {
                stages = {}; refusal = et::Refusal::Budget; return false;
            }
            if (tailWalk != tb::WalkStatus::Completed
                || !BRepCheck_Analyzer(sequence.current).IsValid()) {
                stages = {};
                refusal = stop.load() ? et::Refusal::Cancelled : et::Refusal::ReplayMismatch; return false;
            }
        }
        if (!et::detail::FinishWorkingStages(sequence, budget, refusal,
                stop, current)) { stages = {}; return false; }
        stages.postTreatment = current;
        refusal = et::Refusal::None;
        return true;
    } catch (...) { stages = {}; refusal = et::Refusal::BuildFailed; return false; }
}

} // namespace core3d::retained_edge_treatment::r2

namespace core3d::retained_face_selector {

// The only production issuer for source-relative selector authority. It binds
// immutable capture bytes to the exact native Profile source, the unchanged
// SYRS analytic prefix and the saved untreated post-Boolean treatment base.
// Callers receive only ephemeral per-feature proofs; no authority is persisted.
class SourceStageProofIssuer final {
    // Only authored identities and values are invariant across source rebind.
    // Witness coordinates, normals and receipt geometry are proved separately
    // by the independent commit suffix. This bounded value encoding performs
    // no topology traversal and preserves floating-point intent/amount bits.
    static bool InvariantBytes(const retained_edge_treatment::r2::Definition& definition,
        std::vector<std::uint8_t>& output) {
        namespace et = retained_edge_treatment;
        output.clear();
        if (definition.steps.size() > et::MaximumSteps
            || definition.issuance.retiredLocalIDs.size() > et::MaximumEnvelopeBytes / sizeof(std::uint64_t)) return false;
        et::detail::Writer writer;
        writer.u(definition.schema, 4);
        writer.raw(definition.owner.document); writer.raw(definition.owner.entity);
        writer.raw(definition.owner.definition); writer.raw(definition.outputNode);
        writer.u(definition.issuance.nextLocalID, 8);
        writer.u(definition.issuance.retiredLocalIDs.size(), 4);
        for (const auto retired : definition.issuance.retiredLocalIDs) writer.u(retired, 8);
        writer.u(definition.steps.size(), 4);
        for (const auto& step : definition.steps) {
            if (step.anchors.size() > et::MaximumAnchorsPerStep) return false;
            writer.raw(step.node); writer.raw(step.feature); writer.u(step.localID, 8);
            writer.u(std::uint8_t(step.kind), 1); writer.d(step.amountMM);
            writer.u(step.anchors.size(), 4);
            for (const auto& anchor : step.anchors) {
                writer.raw(anchor.key); writer.u(std::uint8_t(anchor.curve), 1);
            }
            writer.u(step.selector.has_value(), 1);
            if (step.selector) {
                writer.u(step.selector->proofVersion, 4);
                writer.u(std::uint8_t(step.selector->coverage), 1);
                if (!WriteIntent(writer, step.selector->intent)) return false;
            }
        }
        if (!writer.ok) return false;
        output = std::move(writer.b);
        return true;
    }
public:
    static bool Issue(
        const retained_edge_treatment::r2::RetainedBooleanBase& source,
        const retained_edge_treatment::r2::Definition& definition,
        const TopoDS_Shape& sourceBase, const TopoDS_Shape& savedTreatmentBase,
        bool initialIssuance, retained_edge_treatment::ReplayBudget& budget,
        const std::atomic_bool& stop,
        retained_edge_treatment::SourceStageContext& output,
        retained_edge_treatment::Refusal& refusal,
        std::shared_ptr<const SourceStageCertificate>* certificate = nullptr,
        std::uint64_t operationNonce = 0) noexcept {
        namespace et = retained_edge_treatment;
        namespace r2 = retained_edge_treatment::r2;
        output = {};
        if (certificate) certificate->reset();
        try {
            if (stop.load()) { refusal = et::Refusal::Cancelled; return false; }
            const auto* bound = std::get_if<r2::BooleanBaseBinding>(&definition.base);
            const auto* legacy = std::get_if<r2::LegacyBooleanBase>(&source.source);
            const auto* program = legacy
                ? std::get_if<retained_boolean::Program>(&legacy->prefix) : nullptr;
            const auto* prefix = bound
                ? std::get_if<r2::LegacyPrefixBinding>(&bound->prefix) : nullptr;
            if (!bound || !legacy || !program || !prefix
                || !(source.binding == *bound)
                || definition.schema != 4
                || bound->sourceContractRevision != r2::SourceContractRevision
                || bound->format != r2::PrefixFormat::SYRS
                || !r2::detail::Valid(*bound)
                || !retained_boolean::Valid(*program)
                || program->source.family != 1
                || !program->filletSteps.empty()
                || sourceBase.IsNull() || savedTreatmentBase.IsNull()) {
                refusal = et::Refusal::UnsupportedDependency; return false;
            }
            if (!r2::ValidateCompletePrefix(source, *bound, refusal)) return false;
            if (bound->source.document != program->source.document
                || bound->source.entity != program->source.entity
                || bound->source.definition != program->source.definition
                || bound->source.sourceFeature != program->source.sourceFeature
                || bound->sourceNode != program->source.derivedFeature
                || bound->sourceSchema != program->source.schema
                || retained_solid::Bits(bound->metersPerLocalUnit)
                    != retained_solid::Bits(program->source.metersPerUnit)) {
                refusal = et::Refusal::IdentityMismatch; return false;
            }

            profile::Parameters currentProfile;
            std::vector<double> currentCanonical;
            if (!profile::Decode(program->source.values, currentProfile)
                || currentProfile.definition.revolve
                || !currentProfile.shells.empty()
                || !profile::Encode(currentProfile, currentCanonical)
                || currentCanonical.size() != program->source.values.size()) {
                refusal = et::Refusal::UnsupportedBase; return false;
            }
            for (std::size_t index = 0; index < currentCanonical.size(); ++index) {
                if (retained_solid::Bits(currentCanonical[index])
                    != retained_solid::Bits(program->source.values[index])) {
                    refusal = et::Refusal::NoncurrentSource; return false;
                }
            }
            ProfileProducerAccounting accounting(budget, stop);
            const auto inspected = complete_profile_source::InspectCompleteProfileBase(
                sourceBase, currentProfile, accounting);
            if (!inspected) {
                refusal = stop.load() ? et::Refusal::Cancelled
                    : budget.exhausted ? et::Refusal::Budget
                    : et::Refusal::NoncurrentSource;
                return false;
            }
            r2::MigrationStages stages;
            if (!r2::PrepareMigrationStages(legacy->prefix, sourceBase, stop,
                    budget, stages, refusal)) return false;
            if (!et::EquivalentReplayGeometry(savedTreatmentBase,
                    stages.postBoolean, budget, refusal)) return false;

            et::SourceStageContext pending;
            for (const et::Step& step : definition.steps) {
                if (!step.selector) continue;
                const auto* relative = std::get_if<SourceRelativeV1>(
                    &step.selector->intent);
                if (!relative) continue;
                if (!ValidCaptureProfile(*relative)
                    || relative->sourceOwnerDocument != program->source.document
                    || relative->sourceOwnerEntity != program->source.entity
                    || relative->sourceOwnerDefinition != program->source.definition
                    || relative->inputRootNode != prefix->rootNode
                    || relative->sourceFeature != program->source.sourceFeature
                    || relative->captureProfileSchema != program->source.schema) {
                    refusal = et::Refusal::IdentityMismatch; return false;
                }
                std::vector<double> captureValues(
                    relative->captureProfileScalarBits.size());
                for (std::size_t index = 0; index < captureValues.size(); ++index)
                    std::memcpy(&captureValues[index],
                        &relative->captureProfileScalarBits[index], sizeof(double));
                profile::Parameters captureProfile;
                detail::Affine captureTransform, currentTransform;
                if (!profile::Decode(captureValues, captureProfile)
                    || retained_solid::Bits(captureProfile.metersPerUnit)
                        != retained_solid::Bits(currentProfile.metersPerUnit)
                    || !detail::ProfileAffine(captureProfile, captureTransform)
                    || !detail::ProfileAffine(currentProfile, currentTransform)) {
                    refusal = et::Refusal::UnitMismatch; return false;
                }
                if (initialIssuance) {
                    if (captureValues.size() != program->source.values.size()) {
                        refusal = et::Refusal::StaleSnapshot; return false;
                    }
                    for (std::size_t index = 0; index < captureValues.size(); ++index) {
                        if (relative->captureProfileScalarBits[index]
                            != retained_solid::Bits(program->source.values[index])) {
                            refusal = et::Refusal::StaleSnapshot; return false;
                        }
                    }
                }
                auto proof = std::shared_ptr<const SourceStageProof>(
                    new SourceStageProof(*relative, captureTransform,
                        currentTransform));
                pending.steps.push_back({step.feature, std::move(proof)});
            }
            if (pending.steps.empty()) {
                refusal = et::Refusal::UnsupportedDependency; return false;
            }
            output = std::move(pending);
            if (certificate) {
                // Seal the exact mathematical inputs only after all existing
                // issuance proofs succeed. Private production and one-use
                // command ownership are additionally required for transport.
                auto seal = std::shared_ptr<SourceStageCertificate>(new SourceStageCertificate);
                seal->binding = *bound;
                seal->canonicalPrefixBytes = legacy->canonicalPrefixBytes;
                seal->issuedContext = output;
                if (!InvariantBytes(definition, seal->invariantBytes)) {
                    output = {}; refusal = et::Refusal::MalformedCarrier; return false;
                }
                seal->nonce = operationNonce;
                seal->initialIssuance = initialIssuance;
                seal->prefixDigest = bound->sourceRecipeDigest;
                seal->document = bound->source.document;
                seal->entity = bound->source.entity;
                seal->definition = bound->source.definition;
                seal->sourceFeature = bound->source.sourceFeature;
                seal->sourceNode = bound->sourceNode;
                seal->sourceSchema = bound->sourceSchema;
                seal->unitBits = retained_solid::Bits(bound->metersPerLocalUnit);
                seal->sourceBaseShape = sourceBase.TShape();
                seal->sourceBaseLocation = sourceBase.Location();
                seal->sourceBaseOrientation = sourceBase.Orientation();
                seal->savedBaseShape = savedTreatmentBase.TShape();
                seal->savedBaseLocation = savedTreatmentBase.Location();
                seal->savedBaseOrientation = savedTreatmentBase.Orientation();
                *certificate = std::move(seal);
            }
            refusal = et::Refusal::None;
            return true;
        } catch (...) {
            output = {};
            refusal = stop.load() ? et::Refusal::Cancelled
                : et::Refusal::UnsupportedDependency;
            return false;
        }
    }

    // NP6: zero-stage exact-input binding validation of a sealed certificate
    // issued by Issue above. Revalidates the complete structural carrier
    // gates, the exact canonical source/prefix bytes and committed digest,
    // the source/root/owner identities, schema and unit bits, the
    // per-feature immutable intent/capture binding of every proof in the
    // context, the native owned identity of the exact proved source base and
    // post-Boolean base, and the operation nonce. No topology pass runs and
    // no stage is charged. Any changed input or staged binding refuses
    // typed; there is no reissue after a mismatch.
    static bool ValidateBound(
        const SourceStageCertificate& certificate,
        const retained_edge_treatment::SourceStageContext& context,
        const retained_edge_treatment::r2::RetainedBooleanBase& source,
        const retained_edge_treatment::r2::Definition& definition,
        const TopoDS_Shape& sourceBase, const TopoDS_Shape& savedTreatmentBase,
        std::uint64_t operationNonce,
        retained_edge_treatment::Refusal& refusal) noexcept {
        namespace et = retained_edge_treatment;
        namespace r2 = retained_edge_treatment::r2;
        try {
            const auto* bound = std::get_if<r2::BooleanBaseBinding>(&definition.base);
            const auto* legacy = std::get_if<r2::LegacyBooleanBase>(&source.source);
            const auto* program = legacy
                ? std::get_if<retained_boolean::Program>(&legacy->prefix) : nullptr;
            const auto* prefix = bound
                ? std::get_if<r2::LegacyPrefixBinding>(&bound->prefix) : nullptr;
            if (!bound || !legacy || !program || !prefix
                || !(source.binding == *bound)
                || definition.schema != 4
                || bound->sourceContractRevision != r2::SourceContractRevision
                || bound->format != r2::PrefixFormat::SYRS
                || !r2::detail::Valid(*bound)
                || !retained_boolean::Valid(*program)
                || program->source.family != 1
                || !program->filletSteps.empty()
                || sourceBase.IsNull() || savedTreatmentBase.IsNull()) {
                refusal = et::Refusal::UnsupportedDependency; return false;
            }
            // OCAF document time is zero on a legitimate cold-open capture.
            // Equality is the freshness value check; the private operation
            // lease at staging supplies one-command ownership and expiry.
            if (certificate.nonce != operationNonce) {
                refusal = et::Refusal::StaleSnapshot; return false;
            }
            // Exact canonical source/prefix bytes and the committed digest.
            if (!r2::ValidateCompletePrefix(source, *bound, refusal)) return false;
            if (!(certificate.prefixDigest == bound->sourceRecipeDigest)
                || certificate.canonicalPrefixBytes != legacy->canonicalPrefixBytes) {
                refusal = et::Refusal::NoncurrentSource; return false;
            }
            std::vector<std::uint8_t> invariantBytes;
            if (!(certificate.binding == *bound)
                || !InvariantBytes(definition, invariantBytes)
                || invariantBytes != certificate.invariantBytes
                || context.steps != certificate.issuedContext.steps) {
                refusal = et::Refusal::IdentityMismatch; return false;
            }
            // Source/root/owner identities, schema and unit bits.
            if (certificate.document != bound->source.document
                || certificate.entity != bound->source.entity
                || certificate.definition != bound->source.definition
                || certificate.sourceFeature != bound->source.sourceFeature
                || certificate.sourceNode != bound->sourceNode
                || certificate.sourceSchema != bound->sourceSchema
                || certificate.unitBits
                    != retained_solid::Bits(bound->metersPerLocalUnit)
                || bound->source.document != program->source.document
                || bound->source.entity != program->source.entity
                || bound->source.definition != program->source.definition
                || bound->source.sourceFeature != program->source.sourceFeature
                || bound->sourceNode != program->source.derivedFeature
                || bound->sourceSchema != program->source.schema
                || retained_solid::Bits(bound->metersPerLocalUnit)
                    != retained_solid::Bits(program->source.metersPerUnit)) {
                refusal = et::Refusal::IdentityMismatch; return false;
            }
            // Exact proved shape binding within the private producer's lease.
            // This identity check does not freeze a borrowed mutable TShape;
            // borrowed bases are ineligible for transport in Viewer/staging.
            if (sourceBase.TShape() != certificate.sourceBaseShape
                || !sourceBase.Location().IsEqual(certificate.sourceBaseLocation)
                || sourceBase.Orientation() != certificate.sourceBaseOrientation
                || savedTreatmentBase.TShape() != certificate.savedBaseShape
                || !savedTreatmentBase.Location().IsEqual(certificate.savedBaseLocation)
                || savedTreatmentBase.Orientation() != certificate.savedBaseOrientation) {
                refusal = et::Refusal::NoncurrentSource; return false;
            }
            // Per-feature immutable intent/capture binding against the
            // certificate-issuing proofs and the current source identities.
            std::size_t boundSteps = 0;
            for (const et::Step& step : definition.steps) {
                if (!et::NeedsSourceStageContext(step)) continue;
                ++boundSteps;
                const auto* proof = context.proofFor(step);
                if (!proof || !step.selector) {
                    refusal = et::Refusal::UnsupportedDependency; return false;
                }
                if (!(proof->binding_
                        == std::get<SourceRelativeV1>(step.selector->intent))) {
                    refusal = et::Refusal::IdentityMismatch; return false;
                }
                if (!ValidCaptureProfile(proof->binding_)
                    || proof->binding_.sourceOwnerDocument != program->source.document
                    || proof->binding_.sourceOwnerEntity != program->source.entity
                    || proof->binding_.sourceOwnerDefinition != program->source.definition
                    || proof->binding_.inputRootNode != prefix->rootNode
                    || proof->binding_.sourceFeature != program->source.sourceFeature
                    || proof->binding_.captureProfileSchema != program->source.schema) {
                    refusal = et::Refusal::IdentityMismatch; return false;
                }
            }
            if (!boundSteps || boundSteps != context.steps.size()) {
                refusal = et::Refusal::UnsupportedDependency; return false;
            }
            refusal = et::Refusal::None;
            return true;
        } catch (...) {
            refusal = et::Refusal::MalformedCarrier; return false;
        }
    }
};

} // namespace core3d::retained_face_selector

namespace core3d::retained_edge_treatment::r2 {

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
inline bool RebuildEditedPrefixImpl(const RetainedBooleanBase& original,
                                const Edit& edit,
                                const TopoDS_Shape& sourceBase,
                                const std::atomic_bool& stop,
                                RetainedBooleanBase& editedSource,
                                std::vector<std::uint8_t>& editedBytes,
                                TopoDS_Shape& prefixResult,
                                TopoDS_Shape& editedSourceBase,
                                et::ReplayBudget& budget,
                                et::Refusal& refusal,
                                SourceBaseBuilder rebuildBase,
                                bool sourceStagePrefix) noexcept {
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
                    || retained_solid::Bits(p->metersPerUnit)
                        != retained_solid::Bits(edited.source.metersPerUnit)) {
                    refusal = et::Refusal::UnsupportedBase; return false;
                }
                std::vector<double> values;
                if (!EncodeCompleteProfileRequest(*p, edited.source.metersPerUnit,
                        values)) {
                    refusal = et::Refusal::MalformedCarrier; return false;
                }
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
                    // Cancelled keeps priority; a sticky denial inside the
                    // budgeted base rebuild (the bounded persistence readback
                    // of a rounded or circular source) is reported as the
                    // exhaustion it is, never as a generic build failure.
                    refusal = stop.load() ? et::Refusal::Cancelled
                        : budget.exhausted ? et::Refusal::Budget
                        : et::Refusal::BuildFailed;
                    return false;
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
        if (sourceStagePrefix) {
            // SourceStageProofIssuer independently replays this exact retained
            // program through PrepareMigrationStages. Produce the schema-4
            // base through the same native prefix path, so issuance compares
            // like stages rather than two different private-copy histories.
            // No observed BRep is normalized and no byte proof is relaxed.
            if (!edited.filletSteps.empty()) {
                refusal = et::Refusal::UnsupportedDependency; return false;
            }
            MigrationStages stages;
            if (!PrepareMigrationStages(retained_boolean::Recipe{edited},
                    editedSourceBase, stop, budget, stages, refusal)) return false;
            prefixResult = stages.postBoolean;
            editedSource = RetainedBooleanBase{original.binding,
                LegacyBooleanBase{edited, editedBytes}};
            refusal = et::Refusal::None;
            return true;
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
        // C20: the bounded BinTools round trip below performs two full
        // topology passes over the private copy (the write traversal and the
        // read reconstruction) that the byte cap does not bound; debit both
        // on this operation's shared counter before the write runs — the
        // same source-pass discipline as et::DetachReplayGeometry.
        std::size_t copyOccurrences = 0;
        const tb::WalkStatus copyWalk = tb::ChargeTraversal(copied, budget, stop,
            tb::Site::C20R2EditedPrefix, &copyOccurrences);
        if (copyWalk == tb::WalkStatus::Cancelled) { refusal = et::Refusal::Cancelled; return false; }
        if (copyWalk != tb::WalkStatus::Completed
            || !budget.visit(copyOccurrences, tb::Site::C20R2EditedPrefix)) {
            refusal = et::Refusal::Budget; return false;
        }
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
        if (IsConstructionFramedLoft(edited)) {
            TopoDS_Shape fixedPoint;
            const auto status = MaterializePrivateFramedLoftFixedPoint(
                current, stop, budget, fixedPoint);
            if (status == FramedLoftFixedPointStatus::Cancelled) {
                refusal = et::Refusal::Cancelled; return false;
            }
            if (status == FramedLoftFixedPointStatus::BudgetExceeded) {
                refusal = et::Refusal::Budget; return false;
            }
            if (status != FramedLoftFixedPointStatus::Adopted) {
                refusal = et::Refusal::ReplayMismatch; return false;
            }
            current = fixedPoint;
        } else if (edited.source.family == 1 && edited.source.values.size() > 2
            && edited.source.values[2] == 1.0) {
            TopoDS_Shape fixedPoint;
            const auto status = retained_profile_producer::
                MaterializePrivateProfileProducerFixedPoint(
                    current, stop, budget, fixedPoint);
            if (status == retained_profile_producer::FixedPointStatus::Cancelled) {
                refusal = et::Refusal::Cancelled; return false;
            }
            if (status == retained_profile_producer::FixedPointStatus::BudgetExceeded) {
                refusal = et::Refusal::Budget; return false;
            }
            if (status == retained_profile_producer::FixedPointStatus::Adopted) {
                current = fixedPoint;
            }
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
    return RebuildEditedPrefixImpl(original, edit, sourceBase, stop,
        editedSource, editedBytes, prefixResult, editedSourceBase, budget,
        refusal, rebuildBase, false);
}

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
                                bool sourceStagePrefix,
                                SourceBaseBuilder rebuildBase) noexcept {
    return RebuildEditedPrefixImpl(original, edit, sourceBase, stop,
        editedSource, editedBytes, prefixResult, editedSourceBase, budget,
        refusal, rebuildBase, sourceStagePrefix);
}

// R2 capture has two distinct stages: SourceRelativeV1 receipts and roles
// belong to the retained pre-Boolean source, while ordinary selectors and
// every treatment build belong to the post-Boolean working sequence. Keep
// the B1 capture contract and its unique witness correspondence unchanged.
inline bool CaptureSourceRebindRolesR2(const TopoDS_Shape& sourceStage,
    const TopoDS_Shape& oldStage,
    const et::Definition& original, const std::vector<std::uint8_t>& originalBytes,
    et::ReplayBudget& budget, et::Refusal& refusal, et::SourceRebindRoles& output,
    const et::SourceStageContext& sourceContext) noexcept {
    if (std::none_of(original.steps.begin(), original.steps.end(),
            [](const et::Step& step) { return et::NeedsSourceStageContext(step); }))
        return et::CaptureSourceRebindRoles(oldStage, original, originalBytes,
            budget, refusal, output, sourceContext);
    output = {};
    if (sourceStage.IsNull()) { refusal = et::Refusal::UnsupportedBase; return false; }
    try {
        std::vector<std::uint8_t> canonical;
        if (oldStage.IsNull() || !et::Encode(original, canonical, refusal)
            || canonical != originalBytes) {
            refusal = et::Refusal::MalformedCarrier; return false;
        }
        if (!et::detail::ChargeTopology(oldStage, budget)) { refusal = et::Refusal::Budget; return false; }
        const std::atomic_bool neverCancelled{false};
        et::detail::WorkingStageSequence sequence;
        if (!et::detail::BeginWorkingStages(oldStage, original, budget, refusal,
                neverCancelled, sequence)) return false;
        // C15-style publication: roles accumulate into a private temporary
        // and are published only on whole-operation success.
        et::SourceRebindRoles pending;
        for (const et::Step& step : original.steps) {
            const TopoDS_Shape& current = sequence.current;
            if (step.selector) {
                const auto* sourceProof = et::RequireSourceStageProof(
                    step, &sourceContext, refusal);
                if (et::NeedsSourceStageContext(step) && !sourceProof) return false;
                const TopoDS_Shape& selectorStage = sourceProof ? sourceStage : current;
                const double selectorUnit = sourceProof
                    ? original.base.metersPerLocalUnit : 0.001;
                retained_face_selector::Resolution resolution;
                const auto selectorRefusal = sourceProof
                    ? retained_face_selector::Resolve(selectorStage,
                        step.selector->intent, *sourceProof, selectorUnit, budget,
                        neverCancelled, resolution)
                    : retained_face_selector::Resolve(selectorStage,
                        step.selector->intent, selectorUnit, budget,
                        neverCancelled, resolution);
                et::PlanarLineSubset planarSubset;
                bool usedPlanarSubset = false;
                retained_face_selector::Refusal receiptRefusal =
                    retained_face_selector::Refusal::ReplayMismatch;
                if (selectorRefusal == retained_face_selector::Refusal::None && resolution.proof) {
                    receiptRefusal = retained_face_selector::VerifyReceipt(*resolution.proof, step,
                        selectorUnit, budget);
                } else if ((selectorRefusal
                            == retained_face_selector::Refusal::IncompleteBoundary
                        || selectorRefusal
                            == retained_face_selector::Refusal::ExactCount)
                    && std::holds_alternative<
                        retained_face_selector::PlanarFaceBoundary>(
                            step.selector->intent)
                    && et::ResolvePlanarLineSubset(current, step,
                        selectorUnit, budget, true,
                        planarSubset, refusal)) {
                    usedPlanarSubset = true;
                }
                // C17: a budget refusal propagates as Budget instead of the
                // blanket ReplayMismatch; every other failure keeps the
                // existing mapping.
                const bool regularProof = selectorRefusal
                        == retained_face_selector::Refusal::None
                    && resolution.proof
                    && receiptRefusal == retained_face_selector::Refusal::None
                    && resolution.proof->selectedEdgeCount()
                        == step.anchors.size();
                if (!regularProof && (!usedPlanarSubset
                        || planarSubset.uses.size() != step.anchors.size())) {
                    refusal = (refusal == et::Refusal::Budget
                        || selectorRefusal == retained_face_selector::Refusal::Budget
                        || receiptRefusal == retained_face_selector::Refusal::Budget)
                        ? et::Refusal::Budget : et::Refusal::ReplayMismatch;
                    return false;
                }
                const auto& boundaryUses = usedPlanarSubset
                    ? planarSubset.uses : resolution.proof->boundaryUses();
                std::vector<et::SelectorUseRole> measured;
                for (const auto& use : boundaryUses) {
                    // C17: every captured witness loop entry is charged.
                    if (!budget.visit(1, tb::Site::C17SourceRebindOld)) {
                        refusal = et::Refusal::Budget; return false;
                    }
                    et::SelectorUseRole role;
                    if (use.selected
                        && !et::detail::MeasureUseWitness(use, selectorUnit,
                            role.curve, role.pointMM, role.tangent, role.normalA, role.normalB,
                            role.circleRadiusMM, &budget, tb::Site::C17SourceRebindOld)) {
                        refusal = budget.exhausted ? et::Refusal::Budget : et::Refusal::UnsupportedEdge;
                        return false;
                    }
                    measured.push_back(role);
                }
                // Confirm each stored anchor against the actual old native
                // geometry: exactly one selected old use must reproduce its
                // complete witness, including its position and use direction.
                std::set<int> matched;
                std::vector<et::SelectorUseRole> ordered;
                for (std::size_t index = 0; index < step.anchors.size(); ++index) {
                    const et::Anchor& anchor = step.anchors[index];
                    int found = -1;
                    for (std::size_t useIndex = 0; useIndex < measured.size(); ++useIndex) {
                        // C17: every old-side role/witness correspondence
                        // comparison is charged, including nonmatches.
                        if (!budget.visit(1, tb::Site::C17SourceRebindOld)) {
                            refusal = et::Refusal::Budget; return false;
                        }
                        if (!boundaryUses[useIndex].selected
                            || matched.count(int(useIndex))) continue;
                        const et::SelectorUseRole& role = measured[useIndex];
                        if (role.curve == anchor.curve
                            && boundaryUses[useIndex].direction
                                == step.selector->entries[index].direction
                            && et::detail::SameDirection(role.tangent, anchor.tangent)
                            && et::detail::SameDirection(role.normalA, anchor.normalA)
                            && et::detail::SameDirection(role.normalB, anchor.normalB)
                            && std::abs(role.circleRadiusMM - anchor.circleRadiusMM) <= 1e-4
                            && et::detail::SameWitnessPoint(role.pointMM, anchor.pointMM)) {
                            if (found >= 0) { refusal = et::Refusal::AnchorAmbiguous; return false; }
                            found = int(useIndex);
                        }
                    }
                    if (found < 0) { refusal = et::Refusal::ReplayMismatch; return false; }
                    matched.insert(found);
                    et::SelectorUseRole role = measured[std::size_t(found)];
                    role.anchorKey = anchor.key;
                    role.direction = boundaryUses[std::size_t(found)].direction;
                    ordered.push_back(role);
                }
                pending.selectorSteps.push_back({step.feature, std::move(ordered)});
            } else if (original.base.family == et::SourceFamily::Profile) {
                // CLOUD-8242: a raw-anchor step of an admitted Profile source
                // captures complete measured roles on the actual old pre-step
                // stage, and every stored anchor must be reproduced by exactly
                // one measured role before any rebind is attempted.
                std::vector<et::SelectorUseRole> measured;
                if (!et::detail::MeasureStageRawEdgeWitnesses(current,
                        0.001, budget,
                        tb::Site::C17SourceRebindOld, measured)) {
                    refusal = budget.exhausted ? et::Refusal::Budget : et::Refusal::UnsupportedEdge;
                    return false;
                }
                std::set<int> matched;
                std::vector<et::SelectorUseRole> ordered;
                for (const et::Anchor& anchor : step.anchors) {
                    int found = -1;
                    for (std::size_t useIndex = 0; useIndex < measured.size(); ++useIndex) {
                        // C17: every old-side raw role/witness correspondence
                        // comparison is charged, including nonmatches.
                        if (!budget.visit(1, tb::Site::C17SourceRebindOld)) {
                            refusal = et::Refusal::Budget; return false;
                        }
                        if (matched.count(int(useIndex))) continue;
                        if (et::detail::RawRoleMatchesAnchor(measured[useIndex], anchor)) {
                            if (found >= 0) { refusal = et::Refusal::AnchorAmbiguous; return false; }
                            found = int(useIndex);
                        }
                    }
                    if (found < 0) { refusal = et::Refusal::ReplayMismatch; return false; }
                    matched.insert(found);
                    et::SelectorUseRole role = measured[std::size_t(found)];
                    role.anchorKey = anchor.key;
                    ordered.push_back(role);
                }
                pending.rawSteps.push_back({step.feature, std::move(ordered)});
            }
            // The source receipt was proved above. Only the corresponding
            // anchors are replayed on the actual post-Boolean working stage.
            et::Step runtimeStep = step;
            if (et::NeedsSourceStageContext(step)) runtimeStep.selector.reset();
            et::StepProof proof;
            if (!et::detail::BuildWorkingStage(original, runtimeStep, sequence, proof,
                    budget, refusal, neverCancelled, &sourceContext)) return false;
        }
        TopoDS_Shape verified;
        if (!et::detail::FinishWorkingStages(sequence, budget, refusal,
                neverCancelled, verified)) return false;
        pending.original = original; pending.originalBytes = originalBytes;
        output = std::move(pending);
        refusal = et::Refusal::None; return true;
    } catch (...) { output = {}; refusal = et::Refusal::ReplayMismatch; return false; }
}

// R2 moved-witness rebind (CONTRACT.md "Source/input/tool edit"). The
// old-stage selector roles were captured on the main-thread authority side
// through the ordinary or source-aware R2 capture. Here, against the rebuilt
// source or prefix result, each unchanged semantic intent is resolved, every
// stored witness is rebound through the unique source-local role
// correspondence — position excluded, which is exactly what a source edit
// moves — and the complete suffix is strictly replayed on its proper
// preceding stage, mirroring et::ApplySourceRebind. The R2 Boolean binding
// digest is not touched here: the caller re-digests it against the exact
// edited prefix bytes. Missing or ambiguous correspondence refuses; nothing
// is defaulted.
inline bool ApplySourceRebindR2Impl(const et::SourceRebindRoles& roles,
                                const TopoDS_Shape& newStage,
                                Definition& candidate,
                                TopoDS_Shape& treated,
                                et::ReplayBudget& budget,
                                et::Refusal& refusal,
                                const std::atomic_bool& stop,
                                const et::SourceStageContext* sourceContext,
                                const TopoDS_Shape* sourceStage = nullptr) noexcept {
    treated.Nullify();
    try {
        if (newStage.IsNull()) { refusal = et::Refusal::ReplayMismatch; return false; }
        if (stop.load()) { refusal = et::Refusal::Cancelled; return false; }
        // C18: the new-stage census is debited before the analyzer, and the
        // analyzer pass is reserved; a budget failure is never mapped to
        // ReplayMismatch.
        tb::Census newStageCensus;
        const auto stageWalk = tb::CensusTopology(newStage, budget, stop, newStageCensus,
                tb::Site::C18SourceRebindNew, true) != tb::WalkStatus::Completed
            || !tb::ReserveTraversal(newStageCensus, budget, tb::Site::C18SourceRebindNew);
        if (stageWalk) {
            refusal = stop.load() ? et::Refusal::Cancelled : et::Refusal::Budget;
            return false;
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
                                  candidate.issuance, candidate.outputNode,
                                  candidate.steps};
        Definition rebound = candidate;
        // An all-source-relative suffix can rebind every receipt before one
        // source-aware replay. Mixed suffixes retain the ordinary pre-step
        // sequence, resolving only source-relative predicates on sourceStage.
        const bool sourceOnly = sourceStage && std::all_of(
            rebound.steps.begin(), rebound.steps.end(),
            [](const et::Step& step) { return et::NeedsSourceStageContext(step); });
        et::detail::WorkingStageSequence sequence;
        if (!sourceOnly && !et::detail::BeginWorkingStages(newStage, shadowBase,
                budget, refusal, stop, sequence)) return false;
        std::size_t selectorIndex = 0;
        for (std::size_t index = 0; index < rebound.steps.size(); ++index) {
            const TopoDS_Shape& current = sourceOnly ? newStage : sequence.current;
            et::Step step = rebound.steps[index];
            if (step.selector) {
                if (selectorIndex >= roles.selectorSteps.size()
                    || roles.selectorSteps[selectorIndex].first != step.feature
                    || roles.selectorSteps[selectorIndex].second.size() != step.anchors.size()) {
                    refusal = et::Refusal::ReplayMismatch; return false;
                }
                const auto& oldRoles = roles.selectorSteps[selectorIndex].second;
                ++selectorIndex;
                const auto* sourceProof = et::RequireSourceStageProof(
                    step, sourceContext, refusal);
                if (et::NeedsSourceStageContext(step) && !sourceProof)
                    return false;
                const bool onSource = sourceProof && sourceStage;
                const TopoDS_Shape& selectorStage = onSource ? *sourceStage : current;
                const double selectorUnit = onSource ? binding->metersPerLocalUnit : 0.001;
                retained_face_selector::Resolution resolution;
                const auto selectorRefusal = sourceProof
                    ? retained_face_selector::Resolve(selectorStage,
                        step.selector->intent, *sourceProof, selectorUnit, budget,
                        stop, resolution)
                    : retained_face_selector::Resolve(selectorStage,
                        step.selector->intent, selectorUnit, budget,
                        stop, resolution);
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
                        && !et::detail::MeasureUseWitness(use, selectorUnit,
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
                if (onSource && !sourceOnly) {
                    const auto receiptRefusal = retained_face_selector::VerifyReceipt(
                        proof, step, selectorUnit, budget);
                    if (receiptRefusal != retained_face_selector::Refusal::None) {
                        refusal = retained_face_selector::MapToB1(receiptRefusal);
                        return false;
                    }
                }
            }
            if (!sourceOnly) {
                et::Step runtimeStep = step;
                if (sourceStage && et::NeedsSourceStageContext(step))
                    runtimeStep.selector.reset();
                et::StepProof proof;
                if (!et::detail::BuildWorkingStage(shadowBase, runtimeStep, sequence,
                        proof, budget, refusal, stop, sourceContext)) return false;
            }
            rebound.steps[index] = step;
        }
        if (selectorIndex != roles.selectorSteps.size()) {
            refusal = et::Refusal::ReplayMismatch; return false;
        }
        candidate = std::move(rebound);
        if (sourceOnly)
            return ReplayTreatmentSuffix(*sourceStage, newStage, candidate,
                treated, budget, refusal, *sourceContext, stop);
        if (!et::detail::FinishWorkingStages(sequence, budget, refusal,
                stop, treated)) return false;
        refusal = et::Refusal::None;
        return true;
    } catch (...) { treated.Nullify(); refusal = et::Refusal::BuildFailed; return false; }
}

// Source-edit overload: the context must be issued for this edited source
// and its rebuilt post-Boolean base. A source-stage treatment is never built
// or returned; the output always comes from the post-Boolean suffix.
inline bool ApplySourceRebindR2(const et::SourceRebindRoles& roles,
                                const TopoDS_Shape& sourceStage,
                                const TopoDS_Shape& newStage,
                                Definition& candidate,
                                TopoDS_Shape& treated,
                                et::ReplayBudget& budget,
                                et::Refusal& refusal,
                                const et::SourceStageContext& sourceContext,
                                const std::atomic_bool& stop) noexcept {
    treated.Nullify();
    if (sourceStage.IsNull()) { refusal = et::Refusal::UnsupportedBase; return false; }
    return ApplySourceRebindR2Impl(roles, newStage, candidate, treated, budget,
        refusal, stop, &sourceContext, &sourceStage);
}

inline bool ApplySourceRebindR2(const et::SourceRebindRoles& roles,
                                const TopoDS_Shape& newStage,
                                Definition& candidate,
                                TopoDS_Shape& treated,
                                et::ReplayBudget& budget,
                                et::Refusal& refusal,
                                const et::SourceStageContext& sourceContext,
                                const std::atomic_bool& stop) noexcept {
    return ApplySourceRebindR2Impl(roles, newStage, candidate, treated, budget,
        refusal, stop, &sourceContext);
}

inline bool ApplySourceRebindR2(const et::SourceRebindRoles& roles,
                                const TopoDS_Shape& newStage,
                                Definition& candidate,
                                TopoDS_Shape& treated,
                                et::ReplayBudget& budget,
                                et::Refusal& refusal,
                                const std::atomic_bool& stop) noexcept {
    return ApplySourceRebindR2Impl(roles, newStage, candidate, treated, budget,
        refusal, stop, nullptr);
}

inline bool ApplySourceRebindR2(const et::SourceRebindRoles& roles,
                                const TopoDS_Shape& newStage,
                                Definition& candidate,
                                TopoDS_Shape& treated,
                                et::ReplayBudget& budget,
                                et::Refusal& refusal,
                                const et::SourceStageContext& sourceContext) noexcept {
    const std::atomic_bool neverCancelled{false};
    return ApplySourceRebindR2(roles, newStage, candidate, treated, budget,
        refusal, sourceContext, neverCancelled);
}

inline bool ApplySourceRebindR2(const et::SourceRebindRoles& roles,
                                const TopoDS_Shape& newStage,
                                Definition& candidate,
                                TopoDS_Shape& treated,
                                et::ReplayBudget& budget,
                                et::Refusal& refusal) noexcept {
    const std::atomic_bool neverCancelled{false};
    return ApplySourceRebindR2Impl(roles, newStage, candidate, treated, budget,
        refusal, neverCancelled, nullptr);
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
