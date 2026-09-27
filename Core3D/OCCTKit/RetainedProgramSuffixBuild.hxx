#pragma once
// Detached A3/P2 suffix replay and whole-program proof on a PROVEN composite
// prefix result. No OCAF label, command, owner or AI authority. The host passed
// to every function is the independently replayed prefix solid, never the
// displayed candidate and never a persisted cache.
//
// Proof profile "suffix-v1" (D48-shaped: a new family-specific admission proof,
// never a weaker one):
//  - the prefix is proven by the composite owner's own fixed point before this
//    layer runs; this layer re-verifies graph/payload validity and re-proves
//    the complete suffix boundary;
//  - rings need a recipe-derived ligament certificate (LigamentCertificate);
//  - wedges need an independent deterministic Common-volume expectation, since
//    the analytic host-section certificate (SavedCutResultBoundaryExpectation)
//    is analytic-host-only and is NOT broadened here;
//  - the complete result is proven by InspectSuffix: full cell census, exact
//    removed-volume conservation, exact V3 representation bytes against an
//    independent recipe replay (with the bounded binary-readback alternative),
//    kernel validity and an outside-point classification;
//  - the fillet tail uses retained_fillet::BuildComposite, which keeps the
//    transverse-strength certificate (exact empty-difference containment plus
//    census and byte identity against an independent kernel round).
#include "RetainedProgramSuffixAdmission.hxx"
#include "SavedBooleanProgramBuild.hxx"
#include "RetainedFilletBuild.hxx"
#include "RetainedPartBoolean.hxx"
#include "PartBooleanPersistence.hxx"
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <array>

namespace core3d::retained_program_suffix::build {

using Budget = saved_boolean_build::Budget;
using Commitment = saved_cut_source_edit::ShapeCommitment;

enum class Status { Refused, Cancelled, Built };

struct Result final {
    Status status = Status::Refused;
    const char* phase = "input";
    TopoDS_Shape solid;
    Commitment retainedBase, finalResult;
    std::vector<std::uint8_t> exactPayload;
    saved_boolean_result::Inspection correspondence;
    Budget budget;
    retained_fillet::Outcome filletOutcome = retained_fillet::Outcome::Built;
    double removedVolume = 0;
};

// ---------------------------------------------------------------------------
// Recipe-derived host ligament certificate for ring steps.
// The admitted P2 hosts are rectilinear: every composite source is a
// rectangular Profile (schema 1-4, or the P1 schema-5 single cap shell whose
// floor keeps the full authored outer rectangle) in the identity placement
// with matching units. Anything else is HostExtentUnproven, not a fallback.
enum class Ligament : std::uint8_t { Clear, HostExtentUnproven, InsufficientLigament, ToolIntersection };

namespace detail {

struct Rect { double lowU = 0, highU = 0, lowV = 0, highV = 0; };

inline bool Inside(const Rect& outer, double lowU, double highU, double lowV, double highV) {
    return outer.lowU < lowU && highU < outer.highU && outer.lowV < lowV && highV < outer.highV;
}
inline double Gap(const Rect& a, const Rect& b) {
    const double du = std::max({a.lowU - b.highU, b.lowU - a.highU, 0.0});
    const double dv = std::max({a.lowV - b.highV, b.lowV - a.highV, 0.0});
    return std::hypot(du, dv);
}

// Authored rectangle of one composite source in the carrier frame, in MM,
// projected onto the plane perpendicular to `axis`. Fails closed.
inline bool SourceRect(const composite_recipe::SourceNode& source, unsigned axis,
                       double carrierMetersPerUnit, bool& shellSource, Rect& out) noexcept {
    try {
        shellSource = false;
        if (source.recipe.kind != composite_recipe::RecipeKind::Profile) return false;
        // Identity placement with equal units only; wider placements need
        // their own admitted transform policy before a ligament certificate.
        static constexpr double identity[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
        for (unsigned i = 0; i < 16; ++i)
            if (retained_solid::Bits(source.inputToCarrier.matrix[i]) != retained_solid::Bits(identity[i]))
                return false;
        if (retained_solid::Bits(source.inputToCarrier.sourceMetersPerUnit)
                != retained_solid::Bits(source.inputToCarrier.carrierMetersPerUnit)
            || retained_solid::Bits(source.inputToCarrier.carrierMetersPerUnit)
                != retained_solid::Bits(carrierMetersPerUnit)) return false;
        std::vector<double> values;
        profile::Parameters recipe;
        if (!composite_recipe::DecodeScalarRecipe(source.recipe, values)
            || !profile::Decode(values, recipe)
            || retained_solid::Bits(recipe.metersPerUnit)
                != retained_solid::Bits(source.inputToCarrier.sourceMetersPerUnit)) return false;
        if (recipe.definition.plane != 0 || recipe.definition.circle
            || recipe.definition.revolve || recipe.definition.curves
            || !recipe.definition.holes.empty() || recipe.constructionFrame
            || recipe.definition.points.size() != 4) return false;
        shellSource = !recipe.shells.empty();
        // The only admitted shell source is the P1 single cap shell: its floor
        // spans the complete authored outer rectangle.
        if (shellSource
            && (recipe.shells.size() != 1 || recipe.shells.front().thickness <= 0
                || recipe.shells.front().metersPerLocalUnit != recipe.metersPerUnit)) return false;
        double lowX = INFINITY, highX = -INFINITY, lowY = INFINITY, highY = -INFINITY;
        for (const auto& point : recipe.definition.points) {
            lowX = std::min(lowX, point.X()); highX = std::max(highX, point.X());
            lowY = std::min(lowY, point.Y()); highY = std::max(highY, point.Y());
        }
        const double mm = recipe.metersPerUnit * 1000;
        const double depth = recipe.definition.depth * mm;
        if (!std::isfinite(depth) || depth <= 0) return false;
        const double x0 = lowX * mm, x1 = highX * mm, y0 = lowY * mm, y1 = highY * mm;
        if (axis == 2) out = {x0, x1, y0, y1};
        else if (axis == 0) out = {y0, y1, 0, depth};
        else out = {x0, x1, 0, depth};
        // A shell's material certificate holds only through its floor (axis Z);
        // wall-transverse rings on a shell host are not yet provable.
        if (shellSource && axis != 2) return false;
        return out.lowU < out.highU && out.lowV < out.highV;
    } catch (...) { return false; }
}

inline bool OperationOf(const composite_recipe::Definition& graph,
                        part_boolean::Operation& operation) noexcept {
    try {
        if (graph.nodes.empty()) return false;
        const auto* feature = std::get_if<composite_recipe::FeatureNode>(&graph.nodes.back().value);
        if (!feature || graph.outputNode != feature->node
            || feature->kind != composite_recipe::PartBooleanFeatureKind) return false;
        if (graph.schemaVersion == 1
            && feature->codecVersion == composite_recipe::PartBooleanFeatureCodec) {
            part_boolean::AnalyticDefinition analytic;
            if (!part_boolean::DecodeAnalytic(feature->parameters, analytic)) return false;
            operation = analytic.operation;
            return true;
        }
        if (graph.schemaVersion == 2
            && feature->codecVersion == composite_recipe::PartBooleanShellFeatureCodec) {
            part_boolean::Definition shell;
            if (!part_boolean::Decode(feature->parameters, shell)) return false;
            operation = shell.operation;
            return true;
        }
        if (graph.schemaVersion != 3) return false;
        if (feature->codecVersion == composite_recipe::PartBooleanFeatureCodec) {
            part_boolean::AnalyticDefinition analytic;
            if (!composite_recipe::CanonicalAnalyticFeature(*feature, graph.nodes)
                || !part_boolean::DecodeAnalytic(feature->parameters, analytic)) return false;
            operation = analytic.operation;
            return true;
        }
        if (feature->codecVersion == composite_recipe::PartBooleanShellFeatureCodec) {
            part_boolean::Definition shell;
            if (!composite_recipe::CanonicalShellFeature(*feature, graph.nodes)
                || !part_boolean::Decode(feature->parameters, shell)) return false;
            operation = shell.operation;
            return true;
        }
        return false;
    } catch (...) { return false; }
}

} // namespace detail

// Conservative guaranteed-material certificate for one ring step, computed
// solely from the authored source recipes and the persisted operation. The
// bolt annulus (outer edge + 0.002 mm ligament, matching the retained ring
// admission margin) must lie strictly inside guaranteed material.
inline Ligament LigamentCertificate(const composite_recipe::Definition& graph,
                                    const analytic_boolean::Operand& ring,
                                    double carrierMetersPerUnit) noexcept {
    try {
        if (ring.kind != analytic_boolean::OperandKind::CylinderRing) return Ligament::HostExtentUnproven;
        const unsigned axis = unsigned(ring.axis);
        if (axis > 2 || !composite_recipe::Valid(graph)) return Ligament::HostExtentUnproven;
        part_boolean::Operation operation;
        if (!detail::OperationOf(graph, operation)) return Ligament::HostExtentUnproven;
        const auto& feature = std::get<composite_recipe::FeatureNode>(graph.nodes.back().value);
        if (feature.inputs.size() != 2) return Ligament::HostExtentUnproven;
        // Rects follow the persisted left/right INPUT binding order, never the
        // node listing order.
        std::vector<detail::Rect> rects;
        for (const retained_solid::UUID& input : feature.inputs) {
            const auto found = std::find_if(graph.nodes.begin(), graph.nodes.end(),
                [&](const composite_recipe::Node& node) {
                    return composite_recipe::NodeID(node) == input;
                });
            if (found == graph.nodes.end()) return Ligament::HostExtentUnproven;
            const auto* source = std::get_if<composite_recipe::SourceNode>(&found->value);
            if (!source) return Ligament::HostExtentUnproven;
            detail::Rect rect;
            bool shell = false;
            if (!detail::SourceRect(*source, axis, carrierMetersPerUnit, shell, rect))
                return Ligament::HostExtentUnproven;
            rects.push_back(rect);
        }
        if (rects.size() != 2) return Ligament::HostExtentUnproven;
        const double mm = carrierMetersPerUnit * 1000;
        const unsigned u = (axis + 1) % 3, v = (axis + 2) % 3;
        const double centerU = ring.point[u] * mm, centerV = ring.point[v] * mm;
        const double outer = (ring.boltCircleRadius + ring.radius) * mm;
        const double margin = 0.002;
        if (!std::isfinite(centerU) || !std::isfinite(centerV) || !std::isfinite(outer) || outer <= 0)
            return Ligament::HostExtentUnproven;
        // Axis-aligned square bound of the annulus, including the ligament.
        const double lowU = centerU - outer - margin, highU = centerU + outer + margin;
        const double lowV = centerV - outer - margin, highV = centerV + outer + margin;
        const detail::Rect annulus{lowU, highU, lowV, highV};
        const bool insideLeft = detail::Inside(rects[0], lowU, highU, lowV, highV);
        switch (operation) {
            case part_boolean::Operation::Subtract:
                // Material is left minus right: the annulus must sit inside
                // the left input and clear the removed right input entirely.
                if (!insideLeft) return Ligament::InsufficientLigament;
                return detail::Gap(annulus, rects[1]) >= margin
                    ? Ligament::Clear : Ligament::ToolIntersection;
            case part_boolean::Operation::Union:
                // Conservative: only the left input is certified material.
                return insideLeft ? Ligament::Clear : Ligament::InsufficientLigament;
            case part_boolean::Operation::Intersect:
                return insideLeft && detail::Inside(rects[1], lowU, highU, lowV, highV)
                    ? Ligament::Clear : Ligament::InsufficientLigament;
        }
        return Ligament::HostExtentUnproven;
    } catch (...) { return Ligament::HostExtentUnproven; }
}

// Independent deterministic Common-volume expectation for one wedge step. The
// analytic host-section certificate is analytic-host-only by design; this
// measures the tool/host intersection with the kernel's deterministic options
// and the same construction geometry the cut uses. A disagreement beyond the
// existing Build tolerance refuses the step; nothing is fitted.
inline bool MeasuredWedgeVolume(const TopoDS_Shape& host, const analytic_boolean::Operand& tool,
                                double metersPerUnit, const std::atomic_bool& stop,
                                Budget& budget, double& volume) noexcept {
    volume = 0;
    try {
        const double mm = metersPerUnit * 1000;
        const unsigned axis = unsigned(tool.axis);
        if (axis > 2 || !std::isfinite(mm) || mm <= 0) return false;
        const double tolerance = std::max(Precision::Confusion() * 32, 1e-5 / mm);
        const double margin = std::max(tolerance * 4, .001 / mm);
        std::array<double, 6> bounds;
        if (!analytic_boolean::detail::Bounds(host, mm, bounds)) return false;
        const double start = bounds[axis] - margin, end = bounds[axis + 3] + margin;
        const double length = end - start;
        if (!std::isfinite(length) || length <= 0) return false;
        BRepBuilderAPI_MakePolygon polygon;
        for (const auto& point : analytic_boolean_wedge::Expand(tool)) {
            gp_Pnt p;
            p.SetCoord(axis + 1, start);
            p.SetCoord((axis + 1) % 3 + 1, point[0]);
            p.SetCoord((axis + 2) % 3 + 1, point[1]);
            polygon.Add(p);
        }
        polygon.Close();
        if (!polygon.IsDone()) return false;
        BRepBuilderAPI_MakeFace face(polygon.Wire());
        if (!face.IsDone()) return false;
        gp_Vec direction;
        direction.SetCoord(axis + 1, length);
        BRepPrimAPI_MakePrism prism(face.Face(), direction);
        prism.Build();
        if (!prism.IsDone() || stop.load()) return false;
        if (!saved_boolean_build::Charge(prism.Shape(), stop, budget)) return false;
        TopTools_ListOfShape arguments, tools;
        arguments.Append(host);
        tools.Append(prism.Shape());
        BRepAlgoAPI_Common common;
        common.SetArguments(arguments);
        common.SetTools(tools);
        common.SetRunParallel(Standard_False);
        common.SetNonDestructive(Standard_True);
        common.SetFuzzyValue(0);
        common.SetUseOBB(Standard_True);
        common.SetCheckInverted(Standard_True);
        common.Build();
        if (stop.load() || !common.IsDone() || common.HasErrors() || common.Shape().IsNull()) return false;
        if (!saved_boolean_build::Charge(common.Shape(), stop, budget)) return false;
        volume = retained_part_boolean::Volume(common.Shape());
        return std::isfinite(volume) && volume > 0;
    } catch (...) { volume = 0; return false; }
}

// Complete suffix boundary proof: an independent recipe replay of the program
// on the proven prefix result, then census, exact removed-volume conservation,
// exact representation bytes (with the bounded readback alternative), kernel
// validity and an outside-point classification. Never compares the candidate
// with itself: the expectation chain starts from the caller-supplied proven
// prefix result and every step is rebuilt from the payload values.
inline saved_boolean_result::Inspection InspectSuffix(
    const TopoDS_Shape& result, const TopoDS_Shape& provenBase,
    const Definition& program, const cut_display::Settings& display,
    const std::atomic_bool& stop,
    Budget& budget) noexcept {
    saved_boolean_result::Inspection report;
    report.phase = "suffix-program-admission";
    const auto fail = [&]() {
        report.classification = stop.load()
            ? saved_boolean_result::Classification::Cancelled
            : saved_boolean_result::Classification::Refused;
        return report;
    };
    try {
        std::vector<retained_boolean::Disk> sections;
        std::vector<std::uint8_t> exact;
        if (stop.load() || !Valid(program) || !Encode(program, exact)
            || !ExpandedSections(program, sections)
            || result.IsNull() || result.ShapeType() != TopAbs_SOLID
            || result.Orientation() != TopAbs_FORWARD
            || provenBase.IsNull() || provenBase.ShapeType() != TopAbs_SOLID
            || provenBase.Orientation() != TopAbs_FORWARD) return fail();
        report.phase = "suffix-program-replay";
        TopoDS_Shape expected = provenBase;
        double removed = 0;
        for (const auto& step : program.steps) {
            if (stop.load() || budget.booleanSteps >= Budget::MaximumSteps) return fail();
            ++budget.booleanSteps;
            analytic_boolean::Recipe recipe;
            recipe.metersPerUnit = program.carrierMetersPerUnit;
            recipe.operation = step.operation;
            recipe.tool = step.operand;
            double wedgeVolume = 0;
            if (step.operand.kind == analytic_boolean::OperandKind::Wedge
                && !MeasuredWedgeVolume(expected, step.operand,
                    program.carrierMetersPerUnit, stop, budget, wedgeVolume)) return fail();
            analytic_boolean::Result built;
            if (analytic_boolean::Build(expected, recipe, stop, built, wedgeVolume)
                    != analytic_boolean::Status::Built
                || !saved_boolean_build::Charge(built.solid, stop, budget)) return fail();
            removed += built.removedVolume;
            expected = std::move(built.solid);
        }
        report.phase = "suffix-program-display-preparation";
        if (stop.load() || !cut_display::Prepare(expected, display, stop)) return fail();
        report.phase = "suffix-program-census";
        if (!analytic_boolean::detail::Bounded(result, 65536, stop)
            || !analytic_boolean::detail::Bounded(expected, 65536, stop)) return fail();
        for (auto type : {TopAbs_VERTEX, TopAbs_EDGE, TopAbs_WIRE,
                          TopAbs_FACE, TopAbs_SHELL, TopAbs_SOLID}) {
            TopTools_IndexedMapOfShape actualCells, expectedCells;
            TopExp::MapShapes(result, type, actualCells);
            TopExp::MapShapes(expected, type, expectedCells);
            if (actualCells.Extent() != expectedCells.Extent()) return fail();
            if (type == TopAbs_VERTEX) report.vertices = actualCells.Extent();
            if (type == TopAbs_EDGE) report.edges = actualCells.Extent();
            if (type == TopAbs_FACE) report.faces = actualCells.Extent();
        }
        report.phase = "suffix-program-integral";
        const double before = retained_part_boolean::Volume(provenBase);
        const double after = retained_part_boolean::Volume(result);
        if (!std::isfinite(before) || !std::isfinite(after) || !std::isfinite(removed)
            || removed <= 0
            || std::abs(before - after - removed) > std::abs(removed) * 1e-9) return fail();
        report.phase = "suffix-program-whole-representation";
        std::string actual, wanted;
        if (!retained_part_boolean::ExactShapeBytes(result, actual)
            || !retained_part_boolean::ExactShapeBytes(expected, wanted)) return fail();
        if (actual != wanted) {
            TopoDS_Shape reopened;
            if (!saved_boolean_build::ReadbackGeometry(expected, stop, budget, reopened)
                || !retained_part_boolean::ExactShapeBytes(reopened, wanted)
                || actual != wanted) return fail();
        }
        report.phase = "suffix-program-kernel-validity";
        if (stop.load() || !BRepCheck_Analyzer(result, Standard_True).IsValid()) return fail();
        BRepClass3d_SolidClassifier outside(result);
        outside.PerformInfinitePoint(Precision::Confusion());
        if (stop.load() || outside.State() != TopAbs_OUT) return fail();
        report.vertexLinks = report.vertices;
        report.phase = "matched-complete-suffix-program-boundary";
        report.classification = saved_boolean_result::Classification::MatchedOrientedBoundary;
        return report;
    } catch (...) { return fail(); }
}

// Detached complete-program replay on the proven prefix result. Mirrors the
// legacy saved_boolean_build::Build phases: display gates, exact payload,
// charged base commitment, per-step admission (ring ligament certificate,
// wedge Common expectation), sequential analytic cuts, display preparation,
// the complete pre-fillet boundary proof, the retained fillet tail through the
// composite certificate, then base-preservation and final commitments.
inline Result Build(const TopoDS_Shape& provenPrefixResult,
                    const composite_recipe::Definition& graph,
                    const Definition& program,
                    const cut_display::Settings& display,
                    const std::atomic_bool& stop, Budget& budget) noexcept {
    Result out;
    const auto refuse = [&]() {
        Result empty;
        empty.status = stop.load() ? Status::Cancelled : Status::Refused;
        empty.budget = budget;
        empty.phase = out.phase;
        empty.correspondence = out.correspondence;
        empty.filletOutcome = out.filletOutcome;
        return empty;
    };
    try {
        if ((display.type != Aspect_TOD_RELATIVE && display.type != Aspect_TOD_ABSOLUTE)
            || !display.automatic) return refuse();
        for (double value : display.values) if (!std::isfinite(value)) return refuse();
        if (display.values[0] <= 0 || display.values[1] <= 0 || display.values[1] >= M_PI
            || display.values[2] <= 0
            || (display.ownCoefficient && std::abs(display.values[0] - display.values[3]) > Precision::Confusion())
            || (display.ownAngle && std::abs(display.values[1] - display.values[4]) > Precision::Angular()))
            return refuse();
        out.phase = "graph-and-program";
        if (stop.load() || !composite_recipe::Valid(graph) || graph.nodes.empty()
            || !BaseFeatureIsPartBoolean(graph)
            || program.baseFeature != graph.outputNode
            || !Valid(program)
            || !Encode(program, out.exactPayload)
            || !saved_boolean_build::Charge(provenPrefixResult, stop, budget)
            || !saved_cut_source_edit::Commit(provenPrefixResult, stop,
                budget.streamBytes, out.retainedBase)) return refuse();
        std::vector<retained_boolean::Disk> sections;
        if (!ExpandedSections(program, sections)) return refuse();
        for (const auto& step : program.steps)
            if (step.operand.kind == analytic_boolean::OperandKind::CylinderRing
                && LigamentCertificate(graph, step.operand,
                    program.carrierMetersPerUnit) != Ligament::Clear) return refuse();
        // Each analytic build makes an independent deep geometry copy; the
        // proven prefix result is never replaced by the displayed candidate.
        TopoDS_Shape current = provenPrefixResult;
        double removed = 0;
        for (const auto& step : program.steps) {
            if (stop.load() || budget.booleanSteps >= Budget::MaximumSteps) return refuse();
            ++budget.booleanSteps;
            out.phase = "sequential-suffix-boolean";
            analytic_boolean::Recipe recipe;
            recipe.metersPerUnit = program.carrierMetersPerUnit;
            recipe.operation = step.operation;
            recipe.tool = step.operand;
            double wedgeVolume = 0;
            if (step.operand.kind == analytic_boolean::OperandKind::Wedge
                && !MeasuredWedgeVolume(current, step.operand,
                    program.carrierMetersPerUnit, stop, budget, wedgeVolume)) return refuse();
            analytic_boolean::Result built;
            if (analytic_boolean::Build(current, recipe, stop, built, wedgeVolume)
                    != analytic_boolean::Status::Built
                || !saved_boolean_build::Charge(built.solid, stop, budget)) return refuse();
            removed += built.removedVolume;
            current = std::move(built.solid);
        }
        out.removedVolume = removed;
        out.phase = "display-preparation";
        if (stop.load() || !cut_display::Prepare(current, display, stop)) return refuse();
        out.phase = "complete-suffix-correspondence";
        out.correspondence = InspectSuffix(current, provenPrefixResult, program, display, stop, budget);
        if (out.correspondence.classification
            != saved_boolean_result::Classification::MatchedOrientedBoundary) return refuse();
        if (!program.filletSteps.empty()) {
            out.phase = "retained-fillet";
            if (program.filletSteps.size() > 2 * retained_fillet::MaximumSteps - budget.filletSteps)
                return refuse();
            budget.filletSteps += program.filletSteps.size();
            const auto filleted = retained_fillet::BuildComposite(current, program,
                program.carrierMetersPerUnit, stop,
                [&](const TopoDS_Shape& shape) {
                    return saved_boolean_build::Charge(shape, stop, budget);
                },
                &budget.streamBytes);
            out.filletOutcome = filleted.outcome;
            if (filleted.outcome != retained_fillet::Outcome::Built) return refuse();
            current = filleted.solid;
            if (!cut_display::Prepare(current, display, stop)) return refuse();
        }
        out.phase = "base-preservation-and-final-commitment";
        Commitment afterBase;
        if (!saved_cut_source_edit::Commit(provenPrefixResult, stop, budget.streamBytes, afterBase)
            || !(afterBase == out.retainedBase)
            || !saved_cut_source_edit::Commit(current, stop, budget.streamBytes, out.finalResult)
            || stop.load()) return refuse();
        out.budget = budget;
        out.solid = std::move(current);
        out.status = Status::Built;
        out.phase = "built";
        return out;
    } catch (...) { return refuse(); }
}

inline Result Build(const TopoDS_Shape& provenPrefixResult,
                    const composite_recipe::Definition& graph,
                    const Definition& program,
                    const cut_display::Settings& display,
                    const std::atomic_bool& stop) noexcept {
    Budget budget;
    return Build(provenPrefixResult, graph, program, display, stop, budget);
}

// The Q(S) fixed point for a suffix graph: two INDEPENDENT prefix replays
// (supplied by the caller from separate owner rebuilds, never a cache) carry
// the identical suffix program to byte-identical results, and one bounded
// readback leaves the bytes unchanged.
struct FixedPointEvidence final {
    bool prefixResultsByteEqual = false;
    bool bothBuilt = false;
    bool payloadsExact = false;
    bool resultsByteExact = false;
    bool correspondencesMatched = false;
    bool readbackStable = false;
    bool fixedPoint() const noexcept {
        return prefixResultsByteEqual && bothBuilt && payloadsExact
            && resultsByteExact && correspondencesMatched && readbackStable;
    }
};

inline FixedPointEvidence CheckFixedPoint(const TopoDS_Shape& provenPrefixA,
                                          const TopoDS_Shape& provenPrefixB,
                                          const composite_recipe::Definition& graph,
                                          const Definition& program,
                                          const cut_display::Settings& display,
                                          const std::atomic_bool& stop) noexcept {
    FixedPointEvidence evidence;
    try {
        std::string firstBase, secondBase;
        evidence.prefixResultsByteEqual =
            retained_part_boolean::ExactShapeBytes(provenPrefixA, firstBase)
            && retained_part_boolean::ExactShapeBytes(provenPrefixB, secondBase)
            && firstBase == secondBase;
        Budget firstBudget, secondBudget;
        const Result first = Build(provenPrefixA, graph, program, display, stop, firstBudget);
        const Result second = Build(provenPrefixB, graph, program, display, stop, secondBudget);
        evidence.bothBuilt = first.status == Status::Built && second.status == Status::Built;
        evidence.payloadsExact = first.exactPayload == second.exactPayload
            && !first.exactPayload.empty();
        evidence.correspondencesMatched =
            first.correspondence.classification == saved_boolean_result::Classification::MatchedOrientedBoundary
            && second.correspondence.classification == saved_boolean_result::Classification::MatchedOrientedBoundary;
        std::string firstResult, secondResult;
        evidence.resultsByteExact = evidence.bothBuilt
            && retained_part_boolean::ExactShapeBytes(first.solid, firstResult)
            && retained_part_boolean::ExactShapeBytes(second.solid, secondResult)
            && firstResult == secondResult;
        if (evidence.bothBuilt) {
            Budget readbackBudget;
            TopoDS_Shape reopened;
            Commitment before, after;
            evidence.readbackStable =
                saved_boolean_build::ReadbackGeometry(first.solid, stop, readbackBudget, reopened)
                && saved_boolean_build::GeometryCommit(first.solid, stop, readbackBudget.streamBytes, before)
                && saved_boolean_build::GeometryCommit(reopened, stop, readbackBudget.streamBytes, after)
                && before == after;
        }
        return evidence;
    } catch (...) { return {}; }
}

} // namespace core3d::retained_program_suffix::build
