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
