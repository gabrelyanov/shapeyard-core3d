#pragma once

// Detached D4 geometry and attribution only.  This surface owns no OCAF
// command, label, driver registration, editor opening, or admission switch.
#include "FeaturePatternBuild.hxx"
#include "FeaturePatternChildReceipt.hxx"
#include "SavedBooleanProgramBuild.hxx"

#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#if DEBUG
#include <cstdio>
#endif
#include <limits>
#include <set>
#include <vector>

namespace core3d::feature_pattern_native {
using feature_pattern::UUID;
using Digest = feature_pattern_child::Digest;

enum class Status : std::uint8_t {
    Built, Cancelled, InvalidDefinition, MissingHostBaseline,
    HostIdentityMismatch, SourceIdentityMismatch, ProgramBytesMismatch,
    SelectedStepMissing, UnsupportedOperation, UnsupportedSourceTool,
    UnsupportedSourceSuffix, CurvedHost, PlacementFailure, BudgetExceeded,
    ToolDoesNotCut, GeneratedToolsOverlap, SelectorMissing,
    SelectorAmbiguous, ChildSetMismatch, AdmissionRefused, KernelFailure
};

struct HostBaseline final {
    retained_recipe::OwnerKey host;
    UUID retainedRecipeFeature{};
    UUID baselineRecipeIdentity{};
    std::vector<std::uint8_t> exactRecipe;
    TopoDS_Shape solid;
    bool retainedRecipeCurrent = false;
};

struct SourceProgram final {
    retained_boolean::Program program;
    std::vector<std::uint8_t> exactProgram;
    TopoDS_Shape retainedBase;
};

struct SourceTool final {
    saved_boolean_build::DetachedStep detached;
    UUID sourceFeature{};
    std::uint64_t selectedStableOperand = 0;
    Digest completeProgramDigest{};
};

// This is a semantic analytic selector program, not a topology locator.  It
// intentionally has no face/edge ordinal, TShape address, or traversal index.
struct SemanticSelector final {
    UUID childFeature{}, instanceIdentity{};
    std::uint64_t instanceLocalID = 0;
    pattern::Coordinate coordinate;
    std::uint64_t sourceStableOperand = 0;
    pattern::Matrix worldFrame{};
    std::array<double, 3> axisPoint{}, axisDirection{};
    double radius = 0;
    std::uint32_t orientedBoundarySections = 0;
    Digest proof{};
};

struct AttributedBuild final {
    Status status = Status::InvalidDefinition;
    TopoDS_Shape result;
    SourceTool sourceTool;
    saved_cut_source_edit::ShapeCommitment retainedHost, resultGeometry;
    feature_pattern::Admission admission;
    // Retained from the one measured build for an immutable pre-stage review.
    // These are observations already consumed by Admit; no caller recomputes
    // geometry or substitutes requested spacing/policy values.
    std::vector<double> positiveRemovedVolumes;
    std::uint32_t measuredBoundarySections = 0;
    double measuredPairwiseLigamentMM = 0;
    std::vector<SemanticSelector> selectors;
    std::vector<feature_pattern_child::Receipt> childReceipts;
    std::vector<std::uint8_t> completeProgramBytes;
    bool admitted() const noexcept { return status == Status::Built; }
};

namespace detail {
inline double Volume(const TopoDS_Shape& shape) {
    if (shape.IsNull()) return 0;
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties, Standard_True,
                               Standard_False, Standard_False);
    return properties.Mass();
}

inline bool CountTopology(const TopoDS_Shape& shape, std::size_t limit,
                          std::size_t& count) noexcept {
    count = 0;
    try {
        std::vector<std::pair<TopoDS_Shape, unsigned>> pending{{shape, 0}};
        while (!pending.empty()) {
            const auto entry = std::move(pending.back()); pending.pop_back();
            if (entry.first.IsNull() || entry.second > 64 || ++count > limit)
                return false;
            for (TopoDS_Iterator it(entry.first); it.More(); it.Next())
                pending.push_back({it.Value(), entry.second + 1});
        }
        return count != 0;
    } catch (...) { count = 0; return false; }
}

inline bool Transform(const pattern::Matrix& matrix, gp_Trsf& output) noexcept {
    try {
        if (!pattern::IsFiniteFrame(matrix)) return false;
        output.SetValues(matrix[0], matrix[1], matrix[2], matrix[3],
                         matrix[4], matrix[5], matrix[6], matrix[7],
                         matrix[8], matrix[9], matrix[10], matrix[11]);
        return output.Form() != gp_Other;
    } catch (...) { output = gp_Trsf(); return false; }
}

inline bool PlanarHost(const TopoDS_Shape& shape) noexcept {
    try {
        unsigned faces = 0, planar = 0;
        for (TopExp_Explorer it(shape, TopAbs_FACE); it.More(); it.Next()) {
            if (++faces > 4096) return false;
            if (BRepAdaptor_Surface(TopoDS::Face(it.Current())).GetType()
                == GeomAbs_Plane) ++planar;
        }
        return faces != 0 && planar != 0;
    } catch (...) { return false; }
}

inline void U64(std::vector<std::uint8_t>& bytes, std::uint64_t value) {
    for (unsigned index = 0; index < 8; ++index)
        bytes.push_back(std::uint8_t(value >> (index * 8)));
}

inline bool SelectorProof(const SemanticSelector& selector,
                          Digest& output) noexcept {
    try {
        std::vector<std::uint8_t> bytes{'D','4','S','E','L',1,0,0};
        bytes.insert(bytes.end(), selector.childFeature.begin(),
                     selector.childFeature.end());
        bytes.insert(bytes.end(), selector.instanceIdentity.begin(),
                     selector.instanceIdentity.end());
        U64(bytes, selector.instanceLocalID);
        U64(bytes, selector.coordinate.row);
        U64(bytes, selector.coordinate.column);
        U64(bytes, selector.sourceStableOperand);
        for (double value : selector.worldFrame)
            U64(bytes, retained_solid::Bits(value));
        for (double value : selector.axisPoint)
            U64(bytes, retained_solid::Bits(value));
        for (double value : selector.axisDirection)
            U64(bytes, retained_solid::Bits(value));
        U64(bytes, retained_solid::Bits(selector.radius));
        U64(bytes, selector.orientedBoundarySections);
        return feature_pattern_child::detail::Hash(bytes, output);
    } catch (...) { output = {}; return false; }
}

inline bool SameDirection(const gp_Dir& left, const gp_Dir& right,
                          double tolerance) noexcept {
    return std::abs(std::abs(left.Dot(right)) - 1) <= tolerance;
}

// Counts oriented boundary rim sections, not OCCT wires: each closed,
// non-degenerate boundary rim edge of the face, skipping seam uses (a seam
// edge has two pcurves on the face) and degenerated edges.  A through-cut
// cylindrical face has one wire but two rim sections; a blind or partial
// face does not.
inline std::uint32_t BoundarySections(const TopoDS_Face& face) noexcept {
    std::uint32_t count = 0;
    try {
        for (TopExp_Explorer it(face, TopAbs_EDGE); it.More(); it.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(it.Current());
            if (BRep_Tool::Degenerated(edge)
                || BRep_Tool::IsClosed(edge, face)
                || !BRep_Tool::IsClosed(edge)) continue;
            if (count != UINT32_MAX) ++count;
        }
    } catch (...) { return 0; }
    return count;
}

inline bool MatchCylinder(const TopoDS_Face& face,
                          const SemanticSelector& selector,
                          double metersPerUnit) noexcept {
    try {
        BRepAdaptor_Surface surface(face);
        if (surface.GetType() != GeomAbs_Cylinder) return false;
        const gp_Cylinder cylinder = surface.Cylinder();
        const gp_Pnt expected(selector.axisPoint[0], selector.axisPoint[1],
                              selector.axisPoint[2]);
        const gp_Dir direction(selector.axisDirection[0],
                               selector.axisDirection[1],
                               selector.axisDirection[2]);
        const double localTolerance = std::max(
            Precision::Confusion() * 64, 1e-5 / (metersPerUnit * 1000));
        const gp_Lin axis(cylinder.Axis());
#if DEBUG
        std::fprintf(stderr,
            "R179_D4_SELECTOR_CYLINDER localID=%llu sameDirection=%d "
            "axisDistance=%.17g radiusDelta=%.17g tolerance=%.17g "
            "boundaryWires=%u expectedSections=%u\n",
            static_cast<unsigned long long>(selector.instanceLocalID),
            int(SameDirection(cylinder.Axis().Direction(), direction, 1e-10)),
            axis.Distance(expected),
            std::abs(cylinder.Radius() - selector.radius), localTolerance,
            unsigned(BoundarySections(face)),
            unsigned(selector.orientedBoundarySections));
#endif
        return SameDirection(cylinder.Axis().Direction(), direction, 1e-10)
            && axis.Distance(expected) <= localTolerance
            && std::abs(cylinder.Radius() - selector.radius) <= localTolerance
            && BoundarySections(face) == selector.orientedBoundarySections;
    } catch (...) { return false; }
}

inline Status ResolveOne(const TopoDS_Shape& result,
                         const SemanticSelector& selector,
                         double metersPerUnit) noexcept {
    std::uint32_t matches = 0;
    try {
        for (TopExp_Explorer it(result, TopAbs_FACE); it.More(); it.Next()) {
            if (MatchCylinder(TopoDS::Face(it.Current()), selector,
                              metersPerUnit) && ++matches > 1)
                return Status::SelectorAmbiguous;
        }
    } catch (...) { return Status::KernelFailure; }
#if DEBUG
    if (matches == 0)
        std::fprintf(stderr, "R179_D4_SELECTOR_MISSING site=resolve-one localID=%llu\n",
            static_cast<unsigned long long>(selector.instanceLocalID));
#endif
    return matches == 1 ? Status::Built : Status::SelectorMissing;
}

inline feature_pattern_child::Receipt ReceiptFor(
    const feature_pattern::Definition& definition,
    const HostBaseline& baseline,
    const SemanticSelector& selector) {
    feature_pattern_child::Receipt receipt;
    receipt.document = definition.host.document;
    receipt.hostEntity = definition.host.entity;
    receipt.hostDefinition = definition.host.definition;
    receipt.patternFeature = definition.feature;
    receipt.childFeature = selector.childFeature;
    receipt.instanceIdentity = selector.instanceIdentity;
    receipt.baselineRecipeIdentity = baseline.baselineRecipeIdentity;
    receipt.localID = selector.instanceLocalID;
    receipt.row = std::int32_t(selector.coordinate.row);
    receipt.column = std::int32_t(selector.coordinate.column);
    feature_pattern_child::Selector source;
    source.kind = feature_pattern_child::SelectorKind::SourceOperand;
    source.semantic = selector.childFeature;
    source.ordinal = std::uint32_t(selector.sourceStableOperand);
    source.proof = selector.proof;
    feature_pattern_child::Selector host;
    host.kind = feature_pattern_child::SelectorKind::HostBoundary;
    host.semantic = selector.instanceIdentity;
    host.proof = selector.proof;
    feature_pattern_child::Selector section;
    section.kind = feature_pattern_child::SelectorKind::GeneratedSection;
    section.semantic = selector.childFeature;
    section.ordinal = selector.orientedBoundarySections;
    section.proof = selector.proof;
    receipt.selectors = {source, host, section};
    return receipt;
}
} // namespace detail

inline Status BuildSourceTool(const SourceProgram& source,
                              std::uint64_t selectedStableOperand,
                              const std::atomic_bool& stop,
                              SourceTool& output) noexcept {
    output = {};
    try {
        std::vector<std::uint8_t> canonical;
        if (!retained_boolean::Encode(source.program, canonical)
            || canonical != source.exactProgram)
            return Status::ProgramBytesMismatch;
        saved_boolean_build::DetachedStep detached;
        const auto status = saved_boolean_build::BuildSelectedCylinderTool(
            source.retainedBase, source.program, selectedStableOperand, stop,
            detached);
        switch (status) {
        case saved_boolean_build::DetachedStepStatus::Built: break;
        case saved_boolean_build::DetachedStepStatus::Cancelled:
            return Status::Cancelled;
        case saved_boolean_build::DetachedStepStatus::StepMissing:
            return Status::SelectedStepMissing;
        case saved_boolean_build::DetachedStepStatus::UnsupportedOperation:
            return Status::UnsupportedOperation;
        case saved_boolean_build::DetachedStepStatus::UnsupportedOperand:
            return Status::UnsupportedSourceTool;
        case saved_boolean_build::DetachedStepStatus::UnsupportedSuffix:
            return Status::UnsupportedSourceSuffix;
        default: return Status::KernelFailure;
        }
        SourceTool result;
        result.sourceFeature = source.program.source.derivedFeature;
        result.selectedStableOperand = selectedStableOperand;
        result.detached = std::move(detached);
        if (!retained_solid::Hash(result.detached.exactProgram,
                                  result.completeProgramDigest))
            return Status::ProgramBytesMismatch;
        output = std::move(result);
        return Status::Built;
    } catch (...) { output = {}; return Status::KernelFailure; }
}

inline Status ResolveSelectors(const TopoDS_Shape& currentResult,
    const feature_pattern::Definition& definition,
    const std::vector<SemanticSelector>& selectors) noexcept {
    try {
        std::vector<pattern::Placement> placements;
        if (!feature_pattern::Valid(definition)
            || !pattern::BuildPlacements(definition.distribution, placements)
            || placements.size() != selectors.size())
            return Status::ChildSetMismatch;
        std::set<UUID> expected, observed;
        for (const auto& member : definition.distribution.members)
            if (member.state == pattern::MemberState::Active)
                expected.insert(feature_pattern::ChildFeatureID(definition, member));
        for (const auto& selector : selectors) {
            if (!observed.insert(selector.childFeature).second
                || !expected.count(selector.childFeature))
                return Status::ChildSetMismatch;
            const auto placement = std::find_if(placements.begin(), placements.end(),
                [&](const pattern::Placement& value) {
                    return value.identity == selector.instanceIdentity
                        && value.localID == selector.instanceLocalID
                        && value.coordinate == selector.coordinate;
                });
            const auto member = std::find_if(definition.distribution.members.begin(),
                definition.distribution.members.end(), [&](const pattern::Member& value) {
                    return value.state == pattern::MemberState::Active
                        && value.identity == selector.instanceIdentity
                        && value.localID == selector.instanceLocalID
                        && value.coordinate == selector.coordinate;
                });
            if (placement == placements.end()
                || member == definition.distribution.members.end()
                || placement->worldFrame != selector.worldFrame
                || feature_pattern::ChildFeatureID(definition, *member)
                    != selector.childFeature)
                return Status::ChildSetMismatch;
            Digest proof{};
            if (!detail::SelectorProof(selector, proof)
                || proof != selector.proof) {
#if DEBUG
                std::fprintf(stderr, "R179_D4_SELECTOR_MISSING site=resolve-proof\n");
#endif
                return Status::SelectorMissing;
            }
            const Status resolved = detail::ResolveOne(
                currentResult, selector, definition.metersPerUnit);
            if (resolved != Status::Built) return resolved;
        }
        return observed == expected ? Status::Built : Status::ChildSetMismatch;
    } catch (...) { return Status::KernelFailure; }
}

inline AttributedBuild BuildAttributedPattern(
    const HostBaseline& baseline, const SourceProgram& source,
    const feature_pattern::Definition& definition,
    feature_pattern::ExpansionBudget budget,
    const std::atomic_bool& stop) noexcept {
    AttributedBuild output;
    const auto refuse = [&](Status status) {
#if DEBUG
        std::fprintf(stderr, "R179_D4_BUILD_REFUSED status=%u\n", unsigned(status));
#endif
        AttributedBuild empty; empty.status = status; return empty;
    };
    try {
        if (stop.load()) return refuse(Status::Cancelled);
        if (!feature_pattern::Valid(definition))
            return refuse(Status::InvalidDefinition);
        if (!baseline.retainedRecipeCurrent || baseline.solid.IsNull()
            || baseline.exactRecipe.empty()
            || !retained_recipe::Nonzero(baseline.retainedRecipeFeature)
            || !retained_recipe::Nonzero(baseline.baselineRecipeIdentity))
            return refuse(Status::MissingHostBaseline);
        if (!(baseline.host == definition.host))
            return refuse(Status::HostIdentityMismatch);
        const auto& identity = source.program.source;
        if (definition.sourceCut.document != identity.document
            || definition.sourceCut.entity != identity.entity
            || definition.sourceCut.definition != identity.definition
            || definition.sourceCut.sourceFeature != identity.derivedFeature)
            return refuse(Status::SourceIdentityMismatch);
        if (!detail::PlanarHost(baseline.solid)) return refuse(Status::CurvedHost);

        SourceTool selected;
        Status status = BuildSourceTool(source, definition.sourceCutStepID,
                                        stop, selected);
        if (status != Status::Built) return refuse(status);
        BRepBuilderAPI_Copy baseCopy(baseline.solid, Standard_True,
                                     Standard_False);
        if (!baseCopy.IsDone()) return refuse(Status::KernelFailure);
        TopoDS_Shape current = baseCopy.Shape();
        std::vector<pattern::Placement> placements;
        if (!pattern::BuildPlacements(definition.distribution, placements))
            return refuse(Status::PlacementFailure);
        std::vector<TopoDS_Shape> placedTools;
        std::vector<SemanticSelector> selectors;
        std::vector<feature_pattern::GeneratedMeasurement> measured;
        placedTools.reserve(placements.size()); selectors.reserve(placements.size());
        measured.reserve(placements.size());
        const double hostVolume = detail::Volume(current);
        if (!std::isfinite(hostVolume) || hostVolume <= 0)
            return refuse(Status::MissingHostBaseline);

        for (const auto& placement : placements) {
            if (stop.load()) return refuse(Status::Cancelled);
            gp_Trsf transform;
            if (!detail::Transform(placement.worldFrame, transform))
                return refuse(Status::PlacementFailure);
            BRepBuilderAPI_Transform moved(selected.detached.tool, transform,
                                           Standard_True);
            moved.Build();
            if (!moved.IsDone() || moved.Shape().IsNull())
                return refuse(Status::KernelFailure);
            const TopoDS_Shape tool = moved.Shape();
            for (const TopoDS_Shape& other : placedTools) {
                BRepAlgoAPI_Common common(tool, other);
                common.SetRunParallel(Standard_False); common.Build();
                if (!common.IsDone() || common.HasErrors())
                    return refuse(Status::KernelFailure);
                if (!common.Shape().IsNull()
                    && detail::Volume(common.Shape()) > std::max(1e-15,
                        detail::Volume(tool) * 1e-12))
                    return refuse(Status::GeneratedToolsOverlap);
            }
            TopTools_ListOfShape arguments, tools;
            arguments.Append(current); tools.Append(tool);
            BRepAlgoAPI_Cut cut;
            cut.SetArguments(arguments); cut.SetTools(tools);
            cut.SetRunParallel(Standard_False); cut.SetNonDestructive(Standard_True);
            cut.SetFuzzyValue(0); cut.SetUseOBB(Standard_True);
            cut.SetCheckInverted(Standard_True); cut.Build();
            if (!cut.IsDone() || cut.HasErrors() || cut.HasWarnings()
                || cut.Shape().IsNull()) return refuse(Status::KernelFailure);
            TopoDS_Shape next;
            if (!analytic_boolean::detail::SingleResult(cut.Shape(), next))
                return refuse(Status::KernelFailure);
            const double before = detail::Volume(current);
            const double after = detail::Volume(next);
            if (!std::isfinite(before) || !std::isfinite(after)
                || after <= 0 || before <= after)
                return refuse(Status::ToolDoesNotCut);
            unsigned history = 0;
            for (TopExp_Explorer face(tool, TopAbs_FACE); face.More(); face.Next()) {
                history += unsigned(cut.Modified(face.Current()).Extent());
                history += unsigned(cut.Generated(face.Current()).Extent());
            }
            if (history == 0) {
#if DEBUG
                std::fprintf(stderr, "R179_D4_SELECTOR_MISSING site=cut-history\n");
#endif
                return refuse(Status::SelectorMissing);
            }

            const auto member = std::find_if(definition.distribution.members.begin(),
                definition.distribution.members.end(), [&](const pattern::Member& item) {
                    return item.state == pattern::MemberState::Active
                        && item.identity == placement.identity
                        && item.localID == placement.localID
                        && item.coordinate == placement.coordinate;
                });
            if (member == definition.distribution.members.end())
                return refuse(Status::ChildSetMismatch);
            SemanticSelector selector;
            selector.childFeature = feature_pattern::ChildFeatureID(definition, *member);
            selector.instanceIdentity = placement.identity;
            selector.instanceLocalID = placement.localID;
            selector.coordinate = placement.coordinate;
            selector.sourceStableOperand = definition.sourceCutStepID;
            selector.worldFrame = placement.worldFrame;
            const auto& operand = source.program.steps[std::size_t(std::distance(
                source.program.steps.begin(), std::find_if(source.program.steps.begin(),
                    source.program.steps.end(), [&](const retained_boolean::Step& step) {
                        return step.operand.identifier == definition.sourceCutStepID;
                    })))].operand;
            std::array<double, 3> point = operand.point;
            point[unsigned(operand.axis)] = selected.detached.toolStart;
            gp_Pnt axisPoint(point[0], point[1], point[2]); axisPoint.Transform(transform);
            gp_Vec axis = operand.axis == analytic_boolean::Axis::X ? gp_Vec(1,0,0)
                        : operand.axis == analytic_boolean::Axis::Y ? gp_Vec(0,1,0)
                                                                   : gp_Vec(0,0,1);
            axis.Transform(transform);
            if (axis.SquareMagnitude() <= Precision::SquareConfusion())
                return refuse(Status::PlacementFailure);
            const gp_Dir direction(axis);
            selector.axisPoint = {{axisPoint.X(), axisPoint.Y(), axisPoint.Z()}};
            selector.axisDirection = {{direction.X(), direction.Y(), direction.Z()}};
            selector.radius = operand.radius * std::abs(transform.ScaleFactor());
            selector.orientedBoundarySections = definition.expectedBoundarySectionsPerFeature;
            if (!detail::SelectorProof(selector, selector.proof)) {
#if DEBUG
                std::fprintf(stderr, "R179_D4_SELECTOR_MISSING site=build-proof\n");
#endif
                return refuse(Status::SelectorMissing);
            }
            feature_pattern::GeneratedMeasurement observation;
            observation.childFeature = selector.childFeature;
            observation.instanceIdentity = selector.instanceIdentity;
            observation.localID = selector.instanceLocalID;
            observation.coordinate = selector.coordinate;
            observation.worldFrame = selector.worldFrame;
            observation.intersectsHost = true;
            observation.overlapsGeneratedTool = false;
            observation.minimumHostLigamentMM = std::numeric_limits<double>::max();
            observation.removedVolume = before - after;
            observation.boundarySections = selector.orientedBoundarySections;
            measured.push_back(observation);
            selectors.push_back(selector);
            placedTools.push_back(tool);
            current = std::move(next);
        }

        // Measure pairwise analytic clearance from the actual transformed tool
        // axes.  Actual Common results above independently reject overlap.
        for (std::size_t i = 0; i < selectors.size(); ++i)
            for (std::size_t j = i + 1; j < selectors.size(); ++j) {
                const gp_Pnt a(selectors[i].axisPoint[0], selectors[i].axisPoint[1],
                               selectors[i].axisPoint[2]);
                const gp_Pnt b(selectors[j].axisPoint[0], selectors[j].axisPoint[1],
                               selectors[j].axisPoint[2]);
                const double gapMM = (a.Distance(b) - selectors[i].radius
                    - selectors[j].radius) * definition.metersPerUnit * 1000;
                measured[i].minimumHostLigamentMM =
                    std::min(measured[i].minimumHostLigamentMM, gapMM);
                measured[j].minimumHostLigamentMM =
                    std::min(measured[j].minimumHostLigamentMM, gapMM);
            }

        status = ResolveSelectors(current, definition, selectors);
        if (status != Status::Built) return refuse(status);
        std::size_t hostNodes = 0, toolNodes = 0;
        if (!detail::CountTopology(baseline.solid, budget.maximumTopologyNodes,
                                  hostNodes)
            || !detail::CountTopology(selected.detached.tool,
                                      budget.maximumTopologyNodes, toolNodes))
            return refuse(Status::BudgetExceeded);
        budget.hostTopologyNodes = hostNodes;
        budget.sourceToolTopologyNodes = toolNodes;
        budget.sourceRecipeBytes = source.exactProgram.size();
        feature_pattern::AdmissionInput input;
        input.hostSurface = feature_pattern::HostSurface::Planar;
        input.hostRecipeAvailable = baseline.retainedRecipeCurrent;
        input.sourceCutFeatureAvailable = true;
        input.budget = budget;
        input.generated = measured;
        input.result.hostVolume = hostVolume;
        input.result.resultVolume = detail::Volume(current);
        for (const auto& item : measured) {
            if (input.result.generatedBoundarySections
                > UINT32_MAX - item.boundarySections)
                return refuse(Status::BudgetExceeded);
            input.result.generatedBoundarySections += item.boundarySections;
            input.result.presentChildFeatures.push_back(item.childFeature);
        }
        const feature_pattern::Admission admission =
            feature_pattern::Admit(definition, input);
        if (!admission.admitted()) return refuse(Status::AdmissionRefused);
        AttributedBuild result;
        result.status = Status::Built;
        result.result = std::move(current);
        result.sourceTool = std::move(selected);
        result.admission = admission;
        result.measuredBoundarySections = admission.measuredBoundarySections;
        result.measuredPairwiseLigamentMM =
            std::numeric_limits<double>::max();
        result.positiveRemovedVolumes.reserve(measured.size());
        for (const auto& item : measured) {
            result.positiveRemovedVolumes.push_back(item.removedVolume);
            result.measuredPairwiseLigamentMM = std::min(
                result.measuredPairwiseLigamentMM,
                item.minimumHostLigamentMM);
        }
        result.selectors = selectors;
        result.completeProgramBytes = source.exactProgram;
        std::size_t commitmentBytes = 0;
        if (!saved_cut_source_edit::Commit(baseline.solid, stop, commitmentBytes,
                                           result.retainedHost)
            || !saved_cut_source_edit::Commit(result.result, stop,
                                               commitmentBytes,
                                               result.resultGeometry))
            return refuse(Status::KernelFailure);
        for (const auto& selector : result.selectors) {
            auto receipt = detail::ReceiptFor(definition, baseline, selector);
            std::vector<std::uint8_t> canonical;
            if (!feature_pattern_child::Encode(receipt, canonical)) {
#if DEBUG
                std::fprintf(stderr, "R179_D4_SELECTOR_MISSING site=receipt-encode localID=%llu\n",
                    static_cast<unsigned long long>(selector.instanceLocalID));
#endif
                return refuse(Status::SelectorMissing);
            }
            result.childReceipts.push_back(std::move(receipt));
        }
        return result;
    } catch (...) {
        return refuse(stop.load() ? Status::Cancelled : Status::KernelFailure);
    }
}
} // namespace core3d::feature_pattern_native
