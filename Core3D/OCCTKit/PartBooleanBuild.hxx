#pragma once

// Detached P1b builder.  It reuses ShellOperationController's retained replay
// seam and A1's bounded Boolean helper.  It owns no label or document command.
#include "RetainedPartBoolean.hxx"
#include "../UI/ShellOperationController.hpp"
#include <BRepPrimAPI_MakeBox.hxx>
#include <gp_Pnt.hxx>
#include <atomic>
#include <array>
#include <cstdint>
#include <vector>

namespace core3d::part_boolean::build {

enum class ShellFixture : std::uint8_t {
    S = 1,
    S2 = 2,
    UpsideDown = 3,
    Thin = 4,
};

struct ShellBuild final {
    profile::Parameters recipe;
    std::vector<double> canonicalRecipe;
    TopoDS_Shape base;
    TopoDS_Shape shell;
    bool replayed = false;
};

struct BooleanBuild final {
    retained_part_boolean::Candidate candidate;
    retained_part_boolean::CandidateEvidence evidence;
};

inline profile::Parameters ShellRecipe(ShellFixture fixture) {
    profile::Parameters value;
    value.metersPerUnit = 0.001;
    value.definition.plane = 0;
    value.definition.depth = 30;
    value.definition.revolve = false;
    value.definition.points = {
        gp_Pnt2d(0, 0), gp_Pnt2d(40, 0),
        gp_Pnt2d(40, 30), gp_Pnt2d(0, 30),
    };
    profile::ShellStep shell;
    shell.thickness = fixture == ShellFixture::Thin ? 1 : 3;
    shell.metersPerLocalUnit = 0.001;
    if (fixture == ShellFixture::S2) shell.openings = {4, 5};
    else if (fixture == ShellFixture::UpsideDown) shell.openings = {4};
    else shell.openings = {5};
    value.shells.push_back(std::move(shell));
    return value;
}

inline bool IsRectangularP1ShellRecipe(
    const profile::Parameters& value) noexcept {
    try {
        if (profile::SchemaFor(value) != 5 || value.shells.size() != 1
            || value.definition.revolve || value.definition.plane != 0
            || value.definition.circle || value.definition.curves
            || !value.definition.holes.empty() || value.definition.points.size() != 4
            || value.shells.front().thickness <= 0
            || value.shells.front().thickness >= 15
            || value.shells.front().metersPerLocalUnit != value.metersPerUnit) return false;
        const std::vector<int>& openings = value.shells.front().openings;
        return openings == std::vector<int>{5}
            || openings == std::vector<int>({4, 5});
    } catch (...) {
        return false;
    }
}

inline ShellBuild BuildShell(ShellFixture fixture) noexcept {
    ShellBuild result;
    try {
        result.recipe = ShellRecipe(fixture);
        if (!profile::Encode(result.recipe, result.canonicalRecipe)) return result;
        profile::Parameters decoded;
        std::vector<double> reencoded;
        if (!profile::Decode(result.canonicalRecipe, decoded)
            || !profile::Encode(decoded, reencoded)
            || reencoded != result.canonicalRecipe) return result;
        result.base = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 40, 30, 30).Shape();
        std::atomic_bool cancelled{false};
        result.replayed = ReplayProfileShells(
            result.base, decoded, cancelled, result.shell)
            && !result.shell.IsNull();
        return result;
    } catch (...) {
        return {};
    }
}

inline retained_recipe::UUID Identifier(std::uint8_t seed) noexcept {
    retained_recipe::UUID value{};
    value.fill(seed);
    return value;
}

inline retained_recipe::Digest Digest(std::uint8_t seed) noexcept {
    retained_recipe::Digest value{};
    value.fill(seed);
    return value;
}

inline retained_recipe::DependencyRead Read(std::uint8_t seed) noexcept {
    retained_recipe::DependencyRead value;
    value.locator.owner = {Identifier(seed), Identifier(seed + 1), Identifier(seed + 2)};
    value.locator.node = Identifier(seed + 3);
    value.locator.sourceFeature = Identifier(seed + 4);
    value.geometry = Digest(seed + 5);
    value.recipe = Digest(seed + 6);
    value.placement = Digest(seed + 7);
    value.material = Digest(seed + 8);
    value.groups = Digest(seed + 9);
    return value;
}

inline retained_part_boolean::OperandReadSet FixtureReads() noexcept {
    return {Read(1), Read(21)};
}

inline TopoDS_Shape Box(const std::array<double, 6>& bounds) {
    return BRepPrimAPI_MakeBox(
        gp_Pnt(bounds[0], bounds[1], bounds[2]),
        bounds[3] - bounds[0], bounds[4] - bounds[1],
        bounds[5] - bounds[2]).Shape();
}

inline BooleanBuild BuildBoolean(
    retained_part_boolean::Operation operation,
    const TopoDS_Shape& left,
    const TopoDS_Shape& right) noexcept {
    BooleanBuild result;
    const auto reads = FixtureReads();
    result.candidate = retained_part_boolean::BuildDetachedCandidate(
        operation, left, right, 1e-7, reads, reads, &result.evidence);
    return result;
}

} // namespace core3d::part_boolean::build
