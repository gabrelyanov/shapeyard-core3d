#pragma once

#include "PartBooleanBuild.hxx"
#include "RetainedProgramSuffixBuild.hxx"
#include <string>

namespace core3d::part_boolean::rebuild {

struct FixedPointEvidence final {
    bool bothRecipesExact = false;
    bool bothInputsByteExact = false;
    bool resultByteExact = false;
    bool bothBuildsAdmitted = false;
};

struct AnalyticFixedPointEvidence final {
    bool sourceRecipesExact = false;
    bool sourceShapesExact = false;
    bool resultShapeExact = false;
    bool bothBuildsAdmitted = false;
    bool completeDependencyClosure = false;
    bool fixedPoint() const noexcept {
        return sourceRecipesExact && sourceShapesExact && resultShapeExact
            && bothBuildsAdmitted && completeDependencyClosure;
    }
};

struct ShellFixedPointEvidence final {
    bool graphAndFeatureExact = false;
    bool bothSourceRecipesExact = false;
    bool bothSourceShapesExact = false;
    bool resultShapeExact = false;
    bool bothBuildsAdmitted = false;
    bool completeDependencyClosure = false;
    bool fixedPoint() const noexcept {
        return graphAndFeatureExact && bothSourceRecipesExact
            && bothSourceShapesExact && resultShapeExact
            && bothBuildsAdmitted && completeDependencyClosure;
    }
};

inline AnalyticFixedPointEvidence CheckAnalytic(
    const AnalyticDefinition& definition,
    double carrierMetersPerUnit,
    const retained_part_boolean::OperandReadSet& reads) noexcept {
    AnalyticFixedPointEvidence result;
    try {
        std::vector<std::uint8_t> firstRecipe, secondRecipe;
        AnalyticDefinition decoded;
        result.sourceRecipesExact = EncodeAnalytic(definition, firstRecipe)
            && DecodeAnalytic(firstRecipe, decoded)
            && EncodeAnalytic(decoded, secondRecipe) && firstRecipe == secondRecipe;
        const build::AnalyticBuild first = build::BuildAnalytic(
            definition, carrierMetersPerUnit, reads, reads);
        const build::AnalyticBuild second = build::BuildAnalytic(
            decoded, carrierMetersPerUnit, reads, reads);
        result.bothBuildsAdmitted = first.complete && second.complete;
        result.sourceShapesExact = result.bothBuildsAdmitted
            && first.sourceBytes == second.sourceBytes;
        std::string firstResult, secondResult;
        result.resultShapeExact = result.bothBuildsAdmitted
            && retained_part_boolean::ExactShapeBytes(first.candidate.solid, firstResult)
            && retained_part_boolean::ExactShapeBytes(second.candidate.solid, secondResult)
            && firstResult == secondResult;
        result.completeDependencyClosure = definition.inputs.size() == 2
            && reads.leftSource.locator.node == definition.inputs[0].rootNode
            && reads.rightSource.locator.node == definition.inputs[1].rootNode;
        return result;
    } catch (...) { return {}; }
}

// A1-OWNER-budget (R4 / D390 / D392): counted two-pass analytic fixed point.
// The same independent encode/decode/re-encode recipe passes and the same two
// fully detached rebuilds as CheckAnalytic, but the recipe serialization
// passes carry a stage debit and both rebuilds plus both result readbacks
// charge the caller's shared operation continuation before their work runs.
// Sticky Budget/Cancelled propagate without reclassification.
namespace tb = core3d::retained_topology_budget;

inline tb::WalkStatus CheckAnalyticCounted(
    const AnalyticDefinition& definition,
    double carrierMetersPerUnit,
    const retained_part_boolean::OperandReadSet& reads,
    tb::Counter& budget, const std::atomic_bool& cancelled,
    AnalyticFixedPointEvidence& result) noexcept {
    result = {};
    if (cancelled.load()) return tb::WalkStatus::Cancelled;
    try {
        std::vector<std::uint8_t> firstRecipe, secondRecipe;
        AnalyticDefinition decoded;
        // Recipe encode, decode and re-encode are three separate serialization
        // passes; each is debited before it runs.
        if (!budget.beginStage(tb::Site::C25AnalyticInput))
            return tb::WalkStatus::BudgetDenied;
        bool recipesExact = EncodeAnalytic(definition, firstRecipe);
        if (recipesExact) {
            if (cancelled.load()) return tb::WalkStatus::Cancelled;
            if (!budget.beginStage(tb::Site::C25AnalyticInput))
                return tb::WalkStatus::BudgetDenied;
            recipesExact = DecodeAnalytic(firstRecipe, decoded);
        }
        if (recipesExact) {
            if (cancelled.load()) return tb::WalkStatus::Cancelled;
            if (!budget.beginStage(tb::Site::C25AnalyticInput))
                return tb::WalkStatus::BudgetDenied;
            recipesExact = EncodeAnalytic(decoded, secondRecipe)
                && firstRecipe == secondRecipe;
        }
        result.sourceRecipesExact = recipesExact;
        build::AnalyticBuild first, second;
        tb::WalkStatus walk = build::BuildAnalyticCounted(
            definition, carrierMetersPerUnit, reads, reads, budget, cancelled, first);
        if (walk != tb::WalkStatus::Completed) return walk;
        walk = build::BuildAnalyticCounted(
            decoded, carrierMetersPerUnit, reads, reads, budget, cancelled, second);
        if (walk != tb::WalkStatus::Completed) return walk;
        result.bothBuildsAdmitted = first.complete && second.complete;
        result.sourceShapesExact = result.bothBuildsAdmitted
            && first.sourceBytes == second.sourceBytes;
        std::string firstResult, secondResult;
        if (result.bothBuildsAdmitted) {
            walk = retained_part_boolean::ExactShapeBytesCounted(
                first.candidate.solid, firstResult, budget, cancelled,
                tb::Site::C26AnalyticBoolean);
            if (walk == tb::WalkStatus::Cancelled
                || walk == tb::WalkStatus::BudgetDenied) return walk;
            bool exact = walk == tb::WalkStatus::Completed && !firstResult.empty();
            if (exact) {
                walk = retained_part_boolean::ExactShapeBytesCounted(
                    second.candidate.solid, secondResult, budget, cancelled,
                    tb::Site::C26AnalyticBoolean);
                if (walk == tb::WalkStatus::Cancelled
                    || walk == tb::WalkStatus::BudgetDenied) return walk;
                exact = walk == tb::WalkStatus::Completed && !secondResult.empty()
                    && firstResult == secondResult;
            }
            result.resultShapeExact = exact;
        }
        result.completeDependencyClosure = definition.inputs.size() == 2
            && reads.leftSource.locator.node == definition.inputs[0].rootNode
            && reads.rightSource.locator.node == definition.inputs[1].rootNode;
        return tb::WalkStatus::Completed;
    } catch (...) { result = {}; return tb::WalkStatus::Failed; }
}

struct ProgramOperandFixedPointEvidence final {
    bool completeRecipeBytesExact = false;
    bool originalBasesByteExact = false;
    bool completeResultsByteExact = false;
    bool bothCorrespondencesMatched = false;
    bool bothReadbacksStable = false;
    bool completeTailReplayed = false;
    bool fixedPoint() const noexcept {
        return completeRecipeBytesExact && originalBasesByteExact
            && completeResultsByteExact && bothCorrespondencesMatched
            && bothReadbacksStable && completeTailReplayed;
    }
};

struct ProgramOperandBooleanFixedPointEvidence final {
    bool completeProgramsExact = false;
    bool originalBasesExact = false;
    bool completeResultsExact = false;
    bool placedInputsExact = false;
    bool outerResultExact = false;
    bool materialSelectorLineageExact = false;
    bool bothBuildsAdmitted = false;
    bool fixedPoint() const noexcept {
        return completeProgramsExact && originalBasesExact
            && completeResultsExact && placedInputsExact && outerResultExact
            && materialSelectorLineageExact && bothBuildsAdmitted;
    }
};

// Two independent complete-program replays spend one caller-owned operation
// continuation.  Neither result is exposed until both bases, full programs,
// correspondence proofs and readbacks agree byte-for-byte.
inline program_operand::Refusal CheckProgramOperandFixedPoint(
    const program_operand::Value& value,
    const cut_display::Settings& display, const std::atomic_bool& cancelled,
    tb::Counter& budget, ProgramOperandFixedPointEvidence& evidence) noexcept {
    evidence = {};
    if (cancelled.load()) return program_operand::Refusal::Cancelled;
    try {
        const program_operand::ReplayResult first =
            program_operand::ReplayProgramOperand(value, display, cancelled, budget);
        if (first.refusal != program_operand::Refusal::None) return first.refusal;
        const program_operand::ReplayResult second =
            program_operand::ReplayProgramOperand(value, display, cancelled, budget);
        if (second.refusal != program_operand::Refusal::None) return second.refusal;

        evidence.completeRecipeBytesExact = !first.exactProgram.empty()
            && first.exactProgram == second.exactProgram
            && first.exactProgram == value.recipe.effectiveBytes;
        evidence.bothCorrespondencesMatched =
            first.correspondence.classification
                == saved_boolean_result::Classification::MatchedOrientedBoundary
            && second.correspondence.classification
                == saved_boolean_result::Classification::MatchedOrientedBoundary;
        evidence.bothReadbacksStable = first.readbackStable && second.readbackStable;
        evidence.completeTailReplayed = first.completeTailReplayed
            && second.completeTailReplayed;

        std::string firstBase, secondBase, firstResult, secondResult;
        tb::WalkStatus walk = retained_part_boolean::ExactShapeBytesCounted(
            first.originalBase, firstBase, budget, cancelled, tb::Site::C13CommitVerify);
        if (walk != tb::WalkStatus::Completed) return cancelled.load()
            ? program_operand::Refusal::Cancelled : program_operand::Refusal::Budget;
        walk = retained_part_boolean::ExactShapeBytesCounted(second.originalBase,
            secondBase, budget, cancelled, tb::Site::C13CommitVerify);
        if (walk != tb::WalkStatus::Completed) return cancelled.load()
            ? program_operand::Refusal::Cancelled : program_operand::Refusal::Budget;
        walk = retained_part_boolean::ExactShapeBytesCounted(first.solid,
            firstResult, budget, cancelled, tb::Site::C13CommitVerify);
        if (walk != tb::WalkStatus::Completed) return cancelled.load()
            ? program_operand::Refusal::Cancelled : program_operand::Refusal::Budget;
        walk = retained_part_boolean::ExactShapeBytesCounted(second.solid,
            secondResult, budget, cancelled, tb::Site::C13CommitVerify);
        if (walk != tb::WalkStatus::Completed) return cancelled.load()
            ? program_operand::Refusal::Cancelled : program_operand::Refusal::Budget;
        evidence.originalBasesByteExact = !firstBase.empty()
            && firstBase == secondBase;
        evidence.completeResultsByteExact = !firstResult.empty()
            && firstResult == secondResult;
        return evidence.fixedPoint() ? program_operand::Refusal::None
                                     : program_operand::Refusal::ProgramReplayMismatch;
    } catch (...) {
        evidence = {};
        return cancelled.load() ? program_operand::Refusal::Cancelled
            : budget.exhausted ? program_operand::Refusal::Budget
                               : program_operand::Refusal::BuildFailed;
    }
}

// P4 outer fixed point: repeat both complete operand replays and the counted
// outer Boolean on the same continuation, then compare every shape boundary
// and the appearance/selector lineage carried by the request.
inline program_operand::Refusal CheckProgramOperandBooleanFixedPoint(
    const program_operand::Request& request,
    const cut_display::Settings& display, const std::atomic_bool& cancelled,
    tb::Counter& budget,
    ProgramOperandBooleanFixedPointEvidence& evidence) noexcept {
    evidence = {};
    if (cancelled.load()) return program_operand::Refusal::Cancelled;
    try {
        const auto first = program_operand::BuildProgramOperandBoolean(
            request, display, cancelled, budget);
        if (!first.built()) return first.refusal;
        const auto second = program_operand::BuildProgramOperandBoolean(
            request, display, cancelled, budget);
        if (!second.built()) return second.refusal;
        evidence.bothBuildsAdmitted = true;
        evidence.completeProgramsExact = true;
        evidence.materialSelectorLineageExact = true;
        std::array<std::string, 2> firstBase, secondBase;
        std::array<std::string, 2> firstResult, secondResult;
        std::array<std::string, 2> firstPlaced, secondPlaced;
        for (std::size_t index = 0; index < 2; ++index) {
            evidence.completeProgramsExact = evidence.completeProgramsExact
                && !first.replays[index].exactProgram.empty()
                && first.replays[index].exactProgram
                    == second.replays[index].exactProgram
                && first.replays[index].exactProgram
                    == request.operands[index].recipe.effectiveBytes;
            const auto& appearance = request.operands[index].appearance;
            evidence.materialSelectorLineageExact =
                evidence.materialSelectorLineageExact
                && !appearance.effectiveMaterials.empty()
                && !appearance.regions.empty()
                && appearance.versions.proof > 0
                && appearance.versions.selector > 0
                && appearance.versions.material > 0;
            const auto exact = [&](const TopoDS_Shape& shape,
                                   std::string& bytes) {
                return retained_part_boolean::ExactShapeBytesCounted(
                    shape, bytes, budget, cancelled,
                    tb::Site::C13CommitVerify) == tb::WalkStatus::Completed;
            };
            if (!exact(first.replays[index].originalBase, firstBase[index])
                || !exact(second.replays[index].originalBase, secondBase[index])
                || !exact(first.replays[index].solid, firstResult[index])
                || !exact(second.replays[index].solid, secondResult[index])
                || !exact(first.placedInputs[index], firstPlaced[index])
                || !exact(second.placedInputs[index], secondPlaced[index]))
                return cancelled.load() ? program_operand::Refusal::Cancelled
                    : program_operand::Refusal::Budget;
        }
        evidence.originalBasesExact = firstBase == secondBase;
        evidence.completeResultsExact = firstResult == secondResult;
        evidence.placedInputsExact = firstPlaced == secondPlaced;
        std::string firstOuter, secondOuter;
        const auto firstWalk = retained_part_boolean::ExactShapeBytesCounted(
            first.candidate.solid, firstOuter, budget, cancelled,
            tb::Site::C13CommitVerify);
        const auto secondWalk = retained_part_boolean::ExactShapeBytesCounted(
            second.candidate.solid, secondOuter, budget, cancelled,
            tb::Site::C13CommitVerify);
        if (firstWalk != tb::WalkStatus::Completed
            || secondWalk != tb::WalkStatus::Completed)
            return cancelled.load() ? program_operand::Refusal::Cancelled
                : program_operand::Refusal::Budget;
        evidence.outerResultExact = !firstOuter.empty()
            && firstOuter == secondOuter;
        return evidence.fixedPoint() ? program_operand::Refusal::None
            : program_operand::Refusal::ProgramReplayMismatch;
    } catch (...) {
        evidence = {};
        return cancelled.load() ? program_operand::Refusal::Cancelled
            : budget.exhausted ? program_operand::Refusal::Budget
                               : program_operand::Refusal::BuildFailed;
    }
}

//! Recipe-driven production replay. This is intentionally separate from the
//! fixture overload below: no ShellFixture or scenario value can select this
//! path, and both operands are rebuilt from the captured graph on every pass.
inline ShellFixedPointEvidence CheckShellComposite(
    const composite_recipe::Definition& graph,
    const retained_part_boolean::OperandReadSet& reads) noexcept {
    ShellFixedPointEvidence result;
    try {
        build::ShellCompositeRequest firstRequest, secondRequest;
        std::vector<std::uint8_t> firstGraph, secondGraph;
        // Canonical graph equality is checked through the real SYCR codec.
        composite_recipe::Definition decoded;
        result.graphAndFeatureExact = composite_recipe::Encode(graph, firstGraph)
            && composite_recipe::Decode(firstGraph, decoded)
            && composite_recipe::Encode(decoded, secondGraph)
            && firstGraph == secondGraph;
        if (!result.graphAndFeatureExact
            || !build::DecodeShellComposite(graph, firstRequest)
            || !build::DecodeShellComposite(decoded, secondRequest)) return result;
        std::array<std::vector<double>, 2> firstValues, secondValues;
        result.bothSourceRecipesExact = true;
        for (std::size_t index = 0; index < 2; ++index) {
            result.bothSourceRecipesExact = result.bothSourceRecipesExact
                && profile::Encode(firstRequest.profiles[index], firstValues[index])
                && profile::Encode(secondRequest.profiles[index], secondValues[index])
                && firstValues[index] == secondValues[index];
        }
        const auto first = build::BuildShellComposite(firstRequest, reads, reads);
        const auto second = build::BuildShellComposite(secondRequest, reads, reads);
        result.bothBuildsAdmitted = first.complete && second.complete;
        result.bothSourceShapesExact = result.bothBuildsAdmitted
            && first.sourceBytes == second.sourceBytes;
        std::string firstResult, secondResult;
        result.resultShapeExact = result.bothBuildsAdmitted
            && retained_part_boolean::ExactShapeBytes(first.candidate.solid, firstResult)
            && retained_part_boolean::ExactShapeBytes(second.candidate.solid, secondResult)
            && firstResult == secondResult;
        const auto* feature = std::get_if<composite_recipe::FeatureNode>(
            &graph.nodes.back().value);
        result.completeDependencyClosure = feature && feature->inputs.size() == 2
            && reads.leftSource.locator.node == feature->inputs[0]
            && reads.rightSource.locator.node == feature->inputs[1];
        return result;
    } catch (...) { return {}; }
}

// Two independent detached replays.  No prior result cache, face pointer,
// document label, or history entry is an input to the second build.
inline FixedPointEvidence Check(
    build::ShellFixture fixture,
    retained_part_boolean::Operation operation,
    const TopoDS_Shape& tool) noexcept {
    FixedPointEvidence result;
    try {
        const build::ShellBuild firstShell = build::BuildShell(fixture);
        const build::ShellBuild secondShell = build::BuildShell(fixture);
        if (!firstShell.replayed || !secondShell.replayed) return result;
        result.bothRecipesExact = firstShell.canonicalRecipe
            == secondShell.canonicalRecipe;
        std::string firstInput, secondInput, toolBefore, toolAfter;
        result.bothInputsByteExact =
            retained_part_boolean::ExactShapeBytes(firstShell.shell, firstInput)
            && retained_part_boolean::ExactShapeBytes(secondShell.shell, secondInput)
            && retained_part_boolean::ExactShapeBytes(tool, toolBefore)
            && retained_part_boolean::ExactShapeBytes(tool, toolAfter)
            && firstInput == secondInput && toolBefore == toolAfter;
        const build::BooleanBuild first = build::BuildBoolean(
            operation, firstShell.shell, tool);
        const build::BooleanBuild second = build::BuildBoolean(
            operation, secondShell.shell, tool);
        result.bothBuildsAdmitted = first.candidate.admitted()
            && second.candidate.admitted();
        std::string firstResult, secondResult;
        result.resultByteExact = result.bothBuildsAdmitted
            && retained_part_boolean::ExactShapeBytes(
                first.candidate.solid, firstResult)
            && retained_part_boolean::ExactShapeBytes(
                second.candidate.solid, secondResult)
            && firstResult == secondResult;
        return result;
    } catch (...) {
        return {};
    }
}

} // namespace core3d::part_boolean::rebuild
