#pragma once

#include "PartBooleanBuild.hxx"
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
