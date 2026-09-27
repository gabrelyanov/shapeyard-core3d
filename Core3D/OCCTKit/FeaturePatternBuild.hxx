#pragma once

#include "FeaturePatternDefinition.hxx"
#include "PatternBuild.hxx"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <vector>

namespace core3d::feature_pattern {

struct GeneratedMeasurement {
    UUID childFeature{};
    UUID instanceIdentity{};
    std::uint64_t localID = 0;
    pattern::Coordinate coordinate;
    pattern::Matrix worldFrame{};
    bool intersectsHost = false;
    bool overlapsGeneratedTool = false;
    double minimumHostLigamentMM = 0;
    double removedVolume = 0;
    std::uint32_t boundarySections = 0;
};

struct ResultMeasurement {
    double hostVolume = 0;
    double resultVolume = 0;
    std::uint32_t generatedBoundarySections = 0;
    std::vector<UUID> presentChildFeatures;
};

struct AdmissionInput {
    HostSurface hostSurface = HostSurface::Curved;
    bool hostRecipeAvailable = false;
    bool sourceCutFeatureAvailable = false;
    ExpansionBudget budget;
    std::vector<GeneratedMeasurement> generated;
    ResultMeasurement result;
};

struct Attribution {
    UUID sourceCutFeature{};
    std::uint64_t sourceCutStepID = 0;
    UUID childFeature{};
    UUID instanceIdentity{};
    std::uint64_t instanceLocalID = 0;
    pattern::Coordinate coordinate;
};

struct Admission {
    Refusal refusal = Refusal::InvalidDefinition;
    Projection projection;
    double measuredRemovedVolume = 0;
    std::uint32_t measuredBoundarySections = 0;
    std::vector<Attribution> attribution;
    bool admitted() const noexcept { return refusal == Refusal::None; }
};

inline bool Close(double left, double right, double absolute,
                  double relative = 1e-10) noexcept {
    if (!std::isfinite(left) || !std::isfinite(right)) return false;
    return std::abs(left - right)
        <= std::max(absolute, relative * std::max(std::abs(left), std::abs(right)));
}

inline bool SameFrame(const pattern::Matrix& left,
                      const pattern::Matrix& right) noexcept {
    for (std::size_t index = 0; index < left.size(); ++index)
        if (!Close(left[index], right[index], 1e-12)) return false;
    return true;
}

inline Admission Admit(const Definition& definition,
                       const AdmissionInput& input) noexcept {
    Admission output;
    try {
        if (!Valid(definition)) return output;
        if (!input.hostRecipeAvailable) {
            output.refusal = Refusal::MissingHostRecipe; return output;
        }
        if (!input.sourceCutFeatureAvailable) {
            output.refusal = Refusal::MissingSourceCutFeature; return output;
        }
        if (input.hostSurface != HostSurface::Planar) {
            output.refusal = Refusal::CurvedHost; return output;
        }
        output.projection = Project(definition, input.budget);
        if (!output.projection.admitted) {
            output.refusal = Refusal::ExpansionBudget; return output;
        }
        std::vector<pattern::Placement> placements;
        if (!pattern::BuildPlacements(definition.distribution, placements)
            || placements.size() != output.projection.activeFeatures
            || input.generated.size() != placements.size()) {
            output.refusal = Refusal::MissingGeneratedFeature; return output;
        }
        std::set<UUID> expectedFeatures, measuredFeatures;
        std::uint64_t boundaryTotal = 0;
        for (const pattern::Placement& placement : placements) {
            const auto member = std::find_if(definition.distribution.members.begin(),
                definition.distribution.members.end(), [&](const pattern::Member& item) {
                    return item.localID == placement.localID
                        && item.identity == placement.identity
                        && item.coordinate == placement.coordinate;
                });
            if (member == definition.distribution.members.end()) {
                output.refusal = Refusal::MissingGeneratedFeature; return output;
            }
            const UUID childFeature = ChildFeatureID(definition, *member);
            if (!expectedFeatures.insert(childFeature).second) {
                output.refusal = Refusal::MissingGeneratedFeature; return output;
            }
            const auto measured = std::find_if(input.generated.begin(), input.generated.end(),
                [&](const GeneratedMeasurement& item) {
                    return item.childFeature == childFeature
                        && item.instanceIdentity == placement.identity
                        && item.localID == placement.localID
                        && item.coordinate == placement.coordinate;
                });
            if (measured == input.generated.end()
                || !measuredFeatures.insert(measured->childFeature).second) {
                output.refusal = Refusal::MissingGeneratedFeature; return output;
            }
            if (!SameFrame(measured->worldFrame, placement.worldFrame)) {
                output.refusal = Refusal::PlacementMismatch; return output;
            }
            if (!measured->intersectsHost || !std::isfinite(measured->removedVolume)
                || measured->removedVolume <= 0) {
                output.refusal = Refusal::GeneratedToolDoesNotCutHost; return output;
            }
            if (measured->overlapsGeneratedTool) {
                output.refusal = Refusal::GeneratedToolsOverlap; return output;
            }
            if (!std::isfinite(measured->minimumHostLigamentMM)
                || measured->minimumHostLigamentMM < definition.minimumHostLigamentMM) {
                output.refusal = Refusal::InsufficientHostLigament; return output;
            }
            if (measured->boundarySections
                != definition.expectedBoundarySectionsPerFeature
                || boundaryTotal > UINT32_MAX - measured->boundarySections) {
                output.refusal = Refusal::ResultBoundaryMismatch; return output;
            }
            boundaryTotal += measured->boundarySections;
            output.measuredRemovedVolume += measured->removedVolume;
            output.attribution.push_back({definition.sourceCut.sourceFeature,
                definition.sourceCutStepID, childFeature, placement.identity,
                placement.localID, placement.coordinate});
        }
        if (expectedFeatures != measuredFeatures
            || input.result.presentChildFeatures.size() != expectedFeatures.size()) {
            output.refusal = Refusal::MissingGeneratedFeature; return output;
        }
        std::set<UUID> present(input.result.presentChildFeatures.begin(),
                               input.result.presentChildFeatures.end());
        if (present != expectedFeatures) {
            output.refusal = Refusal::MissingGeneratedFeature; return output;
        }
        if (boundaryTotal != input.result.generatedBoundarySections) {
            output.refusal = Refusal::ResultBoundaryMismatch; return output;
        }
        output.measuredBoundarySections = std::uint32_t(boundaryTotal);
        const double resultRemoved = input.result.hostVolume - input.result.resultVolume;
        const double tolerance = std::max(1e-12,
            std::abs(output.measuredRemovedVolume) * 1e-9);
        if (input.result.hostVolume <= 0 || input.result.resultVolume <= 0
            || input.result.resultVolume >= input.result.hostVolume
            || !Close(resultRemoved, output.measuredRemovedVolume, tolerance, 1e-9)) {
            output.refusal = Refusal::RemovedVolumeMismatch; return output;
        }
        output.refusal = Refusal::None; return output;
    } catch (...) { return {}; }
}
} // namespace core3d::feature_pattern
