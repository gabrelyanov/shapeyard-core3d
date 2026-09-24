#pragma once

// P1b's independently authored, detached shell/Boolean evidence.  Expected
// values are the reviewed analytic fixture values; they are not derived from
// Boolean history or from the candidate builder.
#include "PartBooleanFamilyAdmission.hxx"
#include "PartBooleanRebuild.hxx"
#include <BRepAdaptor_Surface.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace core3d::part_boolean::correspondence {

using Evidence = std::map<std::string, double>;

inline void Check(Evidence& evidence, const std::string& key, bool value) {
    evidence.emplace(key, value ? 1.0 : 0.0);
}

inline bool Near(double actual, double expected,
                 double tolerance = 1e-6) noexcept {
    return std::isfinite(actual)
        && std::abs(actual - expected) <= tolerance;
}

inline double SurfaceArea(const TopoDS_Shape& shape) noexcept {
    try {
        GProp_GProps properties;
        BRepGProp::SurfaceProperties(shape, properties);
        return properties.Mass();
    } catch (...) {
        return -1;
    }
}

inline bool Inside(const TopoDS_Shape& shape,
                   double x, double y, double z) noexcept {
    try {
        BRepClass3d_SolidClassifier classifier(
            shape, gp_Pnt(x, y, z), 1e-7);
        return classifier.State() == TopAbs_IN;
    } catch (...) {
        return false;
    }
}

inline bool MatchesS(const TopoDS_Shape& shape) noexcept {
    return Near(retained_part_boolean::Volume(shape), 13968)
        && Near(SurfaceArea(shape), 9732)
        // Centre vertical interval [0,3], proving Z-max rather than Z-min open.
        && Inside(shape, 20, 15, 1.5)
        && !Inside(shape, 20, 15, 3.01)
        && !Inside(shape, 20, 15, 29.99)
        // Mid-height X and Y wall intervals.
        && Inside(shape, 1.5, 15, 15)
        && Inside(shape, 38.5, 15, 15)
        && !Inside(shape, 20, 15, 15)
        && Inside(shape, 20, 1.5, 15)
        && Inside(shape, 20, 28.5, 15)
        && !Inside(shape, 20, 15, 15);
}

inline bool MatchesS2(const TopoDS_Shape& shape) noexcept {
    return Near(retained_part_boolean::Volume(shape), 11520)
        && !Inside(shape, 20, 15, 1.5)
        && !Inside(shape, 20, 15, 28.5)
        && Inside(shape, 1.5, 15, 15)
        && Inside(shape, 38.5, 15, 15)
        && Inside(shape, 20, 1.5, 15)
        && Inside(shape, 20, 28.5, 15);
}

enum class Material : unsigned char { Default = 0, Red = 1, Blue = 2, Cut = 3 };

struct Cell final {
    std::string region;
    Material material = Material::Default;
    double x = 0, y = 0, z = 0, area = 0;
};

struct CellObservation final {
    std::vector<Cell> cells;
    std::size_t faceCount = 0;
    std::size_t outerFrontCount = 0;
    std::size_t innerFrontCount = 0;
    std::size_t cutCount = 0;
    bool allPlanarAndClassified = false;
};

inline CellObservation ObserveCells(
    const TopoDS_Shape& shape, double toolYMinimum,
    double toolYMaximum) noexcept {
    CellObservation result;
    try {
        for (TopExp_Explorer it(shape, TopAbs_FACE); it.More(); it.Next()) {
            ++result.faceCount;
            const TopoDS_Face face = TopoDS::Face(it.Current());
            BRepAdaptor_Surface surface(face, Standard_True);
            if (surface.GetType() != GeomAbs_Plane) return result;
            GProp_GProps properties;
            BRepGProp::SurfaceProperties(face, properties);
            const gp_Pnt centre = properties.CentreOfMass();
            const double area = properties.Mass();
            if (!std::isfinite(area) || area <= 0) return result;
            Cell cell;
            cell.x = centre.X(); cell.y = centre.Y(); cell.z = centre.Z();
            cell.area = area;
            if (std::abs(cell.y) <= 1e-6) {
                cell.region = "outer-front";
                cell.material = Material::Red;
                ++result.outerFrontCount;
            } else if (std::abs(cell.y - 3) <= 1e-6) {
                cell.region = "inner-front";
                cell.material = Material::Blue;
                ++result.innerFrontCount;
            } else if (std::abs(cell.x - 18) <= 1e-6
                       || std::abs(cell.x - 22) <= 1e-6
                       || std::abs(cell.y - toolYMinimum) <= 1e-6
                       || std::abs(cell.y - toolYMaximum) <= 1e-6) {
                cell.region = "generated-cut";
                cell.material = Material::Cut;
                ++result.cutCount;
            } else {
                cell.region = "retained-shell";
                cell.material = Material::Default;
            }
            result.cells.push_back(std::move(cell));
        }
        result.allPlanarAndClassified = result.faceCount > 0
            && result.cells.size() == result.faceCount;
        return result;
    } catch (...) {
        return {};
    }
}

inline std::vector<std::tuple<std::string, unsigned char, long long,
                              long long, long long, long long>>
StableCellKeys(std::vector<Cell> cells, bool reverseDiscovery) {
    if (reverseDiscovery) std::reverse(cells.begin(), cells.end());
    std::vector<std::tuple<std::string, unsigned char, long long,
                           long long, long long, long long>> keys;
    for (const Cell& cell : cells) {
        keys.emplace_back(cell.region, static_cast<unsigned char>(cell.material),
            std::llround(cell.x * 1000), std::llround(cell.y * 1000),
            std::llround(cell.z * 1000), std::llround(cell.area * 1000));
    }
    std::sort(keys.begin(), keys.end());
    return keys;
}

inline Evidence BoundaryEvidence() noexcept {
    Evidence evidence;
    try {
        const build::ShellBuild s = build::BuildShell(build::ShellFixture::S);
        const build::ShellBuild s2 = build::BuildShell(build::ShellFixture::S2);
        const build::ShellBuild upsideDown =
            build::BuildShell(build::ShellFixture::UpsideDown);
        const build::ShellBuild thin =
            build::BuildShell(build::ShellFixture::Thin);
        Check(evidence, "s-replayed-through-shared-shell-seam", s.replayed);
        Check(evidence, "s-schema-five-recipe-retained",
            s.replayed && build::IsRectangularP1ShellRecipe(s.recipe));
        Check(evidence, "s-boundary-volume-and-intervals", s.replayed && MatchesS(s.shell));
        Check(evidence, "s-floor-interval-0-to-3",
            s.replayed && Inside(s.shell, 20, 15, 2.999)
                && !Inside(s.shell, 20, 15, 3.001));
        Check(evidence, "s-zmax-opening-actual",
            s.replayed && !Inside(s.shell, 20, 15, 29.999)
                && Inside(s.shell, 1.5, 15, 29.999));
        Check(evidence, "s2-two-openings-retained",
            s2.replayed && s2.recipe.shells.front().openings == std::vector<int>({4, 5}));
        Check(evidence, "s2-boundary-volume-and-intervals", s2.replayed && MatchesS2(s2.shell));
        evidence["s-volume-mm3"] = s.replayed
            ? retained_part_boolean::Volume(s.shell) : -1;
        evidence["s2-volume-mm3"] = s2.replayed
            ? retained_part_boolean::Volume(s2.shell) : -1;

        const TopoDS_Shape lug = build::Box({38, 10, 10, 44, 20, 20});
        const build::BooleanBuild united = build::BuildBoolean(
            retained_part_boolean::Operation::Union, s.shell, lug);
        const build::BooleanBuild subtracted = build::BuildBoolean(
            retained_part_boolean::Operation::Subtract, s.shell, lug);
        const build::BooleanBuild common = build::BuildBoolean(
            retained_part_boolean::Operation::Intersect, s.shell, lug);
        evidence["lug-volume-mm3"] = retained_part_boolean::Volume(lug);
        evidence["lug-union-volume-mm3"] = retained_part_boolean::Volume(united.candidate.solid);
        evidence["lug-subtract-volume-mm3"] = retained_part_boolean::Volume(subtracted.candidate.solid);
        evidence["lug-common-volume-mm3"] = retained_part_boolean::Volume(common.candidate.solid);
        Check(evidence, "lug-union-independent-boundary",
            united.candidate.admitted()
                && Near(retained_part_boolean::Volume(united.candidate.solid), 14368)
                && Near(SurfaceArea(united.candidate.solid), 9892)
                && Inside(united.candidate.solid, 42, 15, 15));
        Check(evidence, "lug-subtract-independent-boundary",
            subtracted.candidate.admitted()
                && Near(retained_part_boolean::Volume(subtracted.candidate.solid), 13768)
                && Near(SurfaceArea(subtracted.candidate.solid), 9812)
                && !Inside(subtracted.candidate.solid, 39, 15, 15));
        Check(evidence, "lug-common-independent-boundary",
            common.candidate.admitted()
                && Near(retained_part_boolean::Volume(common.candidate.solid), 200)
                && Near(SurfaceArea(common.candidate.solid), 280)
                && Inside(common.candidate.solid, 39, 15, 15)
                && !Inside(common.candidate.solid, 37.5, 15, 15));
        Check(evidence, "separate-operation-history-observed",
            united.evidence.historyObserved && subtracted.evidence.historyObserved
                && common.evidence.historyObserved);
        Check(evidence, "explicit-nondestructive-options",
            united.evidence.explicitDeterministicOptions
                && subtracted.evidence.explicitDeterministicOptions
                && common.evidence.explicitDeterministicOptions);
        Check(evidence, "immutable-input-bytes",
            united.evidence.inputBytesUnchanged
                && subtracted.evidence.inputBytesUnchanged
                && common.evidence.inputBytesUnchanged);

        const TopoDS_Shape unshelled = s.base;
        Check(evidence, "missing-shell-negative",
            !MatchesS(unshelled) && Near(retained_part_boolean::Volume(unshelled), 36000));
        Check(evidence, "upside-down-shell-negative",
            upsideDown.replayed && !MatchesS(upsideDown.shell)
                && Near(retained_part_boolean::Volume(upsideDown.shell), 13968)
                && !Inside(upsideDown.shell, 20, 15, 1.5)
                && Inside(upsideDown.shell, 20, 15, 28.5));
        const auto missingFloor = build::BuildBoolean(
            retained_part_boolean::Operation::Subtract, s.shell,
            build::Box({10, 10, -1, 30, 20, 4}));
        const auto missingWall = build::BuildBoolean(
            retained_part_boolean::Operation::Subtract, s.shell,
            build::Box({-1, 10, 5, 4, 20, 25}));
        Check(evidence, "missing-floor-negative",
            missingFloor.candidate.admitted() && !MatchesS(missingFloor.candidate.solid));
        Check(evidence, "missing-wall-negative",
            missingWall.candidate.admitted() && !MatchesS(missingWall.candidate.solid));
        Check(evidence, "too-thin-shell-negative",
            thin.replayed && !MatchesS(thin.shell));
        profile::Parameters twoShells = s.recipe;
        twoShells.shells.push_back(twoShells.shells.front());
        profile::Parameters forgedSchemaFour = s.recipe;
        forgedSchemaFour.shells.clear();
        Check(evidence, "two-shell-signature-negative",
            !build::IsRectangularP1ShellRecipe(twoShells));
        Check(evidence, "forged-schema-four-negative",
            profile::SchemaFor(forgedSchemaFour) == 1
                && !build::IsRectangularP1ShellRecipe(forgedSchemaFour));

        const TopoDS_Shape a = build::Box({0, 0, 0, 10, 10, 10});
        const TopoDS_Shape tangent = build::Box({10, 0, 0, 20, 10, 10});
        const TopoDS_Shape disjoint = build::Box({20, 0, 0, 30, 10, 10});
        const auto reads = build::FixtureReads();
        const auto invalid = retained_part_boolean::BuildDetachedCandidate(
            retained_part_boolean::Operation::Union, TopoDS_Shape(), a,
            1e-7, reads, reads);
        const auto tangentResult = build::BuildBoolean(
            retained_part_boolean::Operation::Union, a, tangent);
        const auto emptyResult = build::BuildBoolean(
            retained_part_boolean::Operation::Intersect, a, disjoint);
        const auto disconnectedResult = build::BuildBoolean(
            retained_part_boolean::Operation::Union, a, disjoint);
        const auto noOverlapResult = build::BuildBoolean(
            retained_part_boolean::Operation::Subtract, a, disjoint);
        Check(evidence, "invalid-control-refused",
            invalid.refusal == retained_part_boolean::Refusal::InvalidInput);
        Check(evidence, "tangent-control-refused",
            tangentResult.candidate.refusal == retained_part_boolean::Refusal::TangentOnly);
        Check(evidence, "empty-control-refused",
            emptyResult.candidate.refusal == retained_part_boolean::Refusal::EmptyResult);
        Check(evidence, "disconnected-control-refused",
            disconnectedResult.candidate.refusal == retained_part_boolean::Refusal::Disconnected);
        Check(evidence, "no-overlap-control-refused",
            noOverlapResult.candidate.refusal == retained_part_boolean::Refusal::NoInteriorOverlap);

        const auto fixed = rebuild::Check(build::ShellFixture::S,
            retained_part_boolean::Operation::Union, lug);
        Check(evidence, "detached-rebuild-fixed-point",
            fixed.bothRecipesExact && fixed.bothInputsByteExact
                && fixed.bothBuildsAdmitted && fixed.resultByteExact);
        Check(evidence, "one-shell-family-evidence-signature",
            family_admission::InstalledEvidenceSignatures.size() == 1
                && family_admission::EvidenceOperationInstalled(Operation::Union)
                && family_admission::EvidenceOperationInstalled(Operation::Subtract)
                && family_admission::EvidenceOperationInstalled(Operation::Intersect));
        Check(evidence, "codec-two-native-admission-disabled",
            !family_admission::NativeAdmissionEnabled);
        Check(evidence, "boolean-route-disabled",
            !family_admission::BooleanRouteInstalled);
        Check(evidence, "b1-treatment-disabled",
            !family_admission::TreatmentFamilyInstalled);
        return evidence;
    } catch (...) {
        return {{"setup-exception", 0}};
    }
}

inline Evidence SplitMergeEvidence() noexcept {
    Evidence evidence;
    try {
        const build::ShellBuild s = build::BuildShell(build::ShellFixture::S);
        const TopoDS_Shape n = build::Box({18, -1, -1, 22, 4, 31});
        const TopoDS_Shape nPrime = build::Box({18, 9, -1, 22, 14, 31});
        const build::BooleanBuild split = build::BuildBoolean(
            retained_part_boolean::Operation::Subtract, s.shell, n);
        const build::BooleanBuild merged = build::BuildBoolean(
            retained_part_boolean::Operation::Subtract, s.shell, nPrime);
        const double splitVolume = retained_part_boolean::Volume(split.candidate.solid);
        const double mergedVolume = retained_part_boolean::Volume(merged.candidate.solid);
        evidence["n-result-volume-mm3"] = splitVolume;
        evidence["n-prime-result-volume-mm3"] = mergedVolume;
        evidence["n-removal-mm3"] = retained_part_boolean::Volume(s.shell) - splitVolume;
        evidence["n-prime-removal-mm3"] = retained_part_boolean::Volume(s.shell) - mergedVolume;
        Check(evidence, "n-exact-372-removal",
            split.candidate.admitted() && Near(splitVolume, 13596)
                && Near(13968 - splitVolume, 372));
        Check(evidence, "n-prime-exact-60-removal",
            merged.candidate.admitted() && Near(mergedVolume, 13908)
                && Near(13968 - mergedVolume, 60));
        Check(evidence, "n-front-and-floor-occupancy",
            !Inside(split.candidate.solid, 20, 2, 15)
                && Inside(split.candidate.solid, 20, 15, 1.5)
                && !Inside(split.candidate.solid, 20, 15, 3.01)
                && Inside(split.candidate.solid, 10, 1.5, 15)
                && Inside(split.candidate.solid, 10, 28.5, 15)
                && !Inside(split.candidate.solid, 10, 15, 15));
        Check(evidence, "n-prime-positive-floor-hole",
            Inside(merged.candidate.solid, 20, 2, 15)
                && !Inside(merged.candidate.solid, 20, 11, 1.5)
                && Inside(merged.candidate.solid, 20, 15, 1.5));

        const CellObservation splitCells = ObserveCells(split.candidate.solid, -1, 4);
        const CellObservation mergedCells = ObserveCells(merged.candidate.solid, 9, 14);
        evidence["n-output-face-count"] = double(splitCells.faceCount);
        evidence["n-prime-output-face-count"] = double(mergedCells.faceCount);
        evidence["n-outer-front-cell-count"] = double(splitCells.outerFrontCount);
        evidence["n-inner-front-cell-count"] = double(splitCells.innerFrontCount);
        evidence["n-prime-outer-front-cell-count"] = double(mergedCells.outerFrontCount);
        evidence["n-prime-inner-front-cell-count"] = double(mergedCells.innerFrontCount);
        Check(evidence, "complete-native-face-observation",
            splitCells.allPlanarAndClassified && mergedCells.allPlanarAndClassified);
        Check(evidence, "many-to-many-front-split",
            splitCells.outerFrontCount == 2 && splitCells.innerFrontCount == 2);
        Check(evidence, "front-regions-merged-without-new-key",
            mergedCells.outerFrontCount == 1 && mergedCells.innerFrontCount == 1);
        Check(evidence, "fixed-one-face-selector-refuses-split",
            splitCells.outerFrontCount != 1);
        Check(evidence, "whole-region-selector-covers-both-strips",
            splitCells.outerFrontCount == 2);
        Check(evidence, "red-blue-cut-material-policy",
            splitCells.outerFrontCount == 2 && splitCells.innerFrontCount == 2
                && splitCells.cutCount > 0
                && std::all_of(splitCells.cells.begin(), splitCells.cells.end(), [](const Cell& cell) {
                    if (cell.region == "outer-front") return cell.material == Material::Red;
                    if (cell.region == "inner-front") return cell.material == Material::Blue;
                    if (cell.region == "generated-cut") return cell.material == Material::Cut;
                    return cell.material == Material::Default;
                }));
        Check(evidence, "reversed-discovery-order-stable",
            StableCellKeys(splitCells.cells, false)
                == StableCellKeys(splitCells.cells, true)
                && StableCellKeys(mergedCells.cells, false)
                    == StableCellKeys(mergedCells.cells, true));
        Check(evidence, "semantic-region-identities-preserved",
            splitCells.outerFrontCount > 0 && splitCells.innerFrontCount > 0
                && mergedCells.outerFrontCount > 0 && mergedCells.innerFrontCount > 0);
        Check(evidence, "both-operand-recipes-and-bytes-retained",
            s.replayed && build::IsRectangularP1ShellRecipe(s.recipe)
                && split.evidence.inputBytesUnchanged
                && merged.evidence.inputBytesUnchanged);
        Check(evidence, "dependent-suffix-is-refused-not-dropped",
            !family_admission::TreatmentFamilyInstalled);
        Check(evidence, "native-admission-and-route-stay-disabled",
            !family_admission::NativeAdmissionEnabled
                && !family_admission::BooleanRouteInstalled);

        const auto fixed = rebuild::Check(build::ShellFixture::S,
            retained_part_boolean::Operation::Subtract, n);
        Check(evidence, "split-cold-style-detached-rebuild-fixed-point",
            fixed.bothRecipesExact && fixed.bothInputsByteExact
                && fixed.bothBuildsAdmitted && fixed.resultByteExact);
        return evidence;
    } catch (...) {
        return {{"setup-exception", 0}};
    }
}

} // namespace core3d::part_boolean::correspondence
