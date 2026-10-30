#pragma once

#include <Core3D/FacePushPullSelector.hxx>
#include <BOPAlgo_ArgumentAnalyzer.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepFeat_SplitShape.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Message_ProgressIndicator.hxx>
#include <TopoDS_Iterator.hxx>
#include <locale>
#include <sstream>

namespace core3d::face_push_pull {
struct BuildEvidence final {
    double sourceVolumeLocal = 0, resultVolumeLocal = 0, regionAreaLocal = 0;
    bool sourceBytesUnchanged = false, candidateBytesDistinct = false;
    std::size_t connectedSolids = 0;
};
struct BuildResult final {
    Refusal refusal = Refusal::BuildFailure;
    TopoDS_Shape solid;
    TopoDS_Face region, outputFace;
    FaceSelector outputSelector;
    BuildEvidence evidence;
    bool built() const noexcept { return refusal == Refusal::None && !solid.IsNull(); }
};

namespace detail {
class Progress final : public Message_ProgressIndicator {
public:
    explicit Progress(const std::atomic_bool& cancelled) : cancelled_(cancelled) {}
protected:
    Standard_Boolean UserBreak() override { return cancelled_.load(); }
    void Show(const Message_ProgressScope&, Standard_Boolean) override {}
private:
    const std::atomic_bool& cancelled_;
};
inline bool exactShapeBytes(const TopoDS_Shape& shape, std::vector<std::uint8_t>& bytes) {
    bytes.clear(); std::ostringstream stream; stream.imbue(std::locale::classic());
    BRepTools::Write(shape, stream, Standard_False, Standard_False, TopTools_FormatVersion_VERSION_3);
    if (!stream.good()) return false;
    const std::string value = stream.str();
    if (value.empty() || value.size() > composite_recipe::MaximumDocumentAggregateBytes) return false;
    bytes.assign(value.begin(), value.end()); return true;
}
inline double area(const TopoDS_Shape& shape) {
    GProp_GProps props; BRepGProp::SurfaceProperties(shape, props); return props.Mass();
}
inline double volume(const TopoDS_Shape& shape) {
    GProp_GProps props; BRepGProp::VolumeProperties(shape, props); return props.Mass();
}
inline bool selfIntersectionFree(const TopoDS_Shape& shape, const Handle(Progress)& progress) {
    BOPAlgo_ArgumentAnalyzer analyzer;
    analyzer.SetShape1(shape); analyzer.SetRunParallel(Standard_False);
    analyzer.StopOnFirstFaulty() = Standard_True;
    analyzer.ArgumentTypeMode() = Standard_False; analyzer.SelfInterMode() = Standard_True;
    analyzer.SmallEdgeMode() = Standard_False; analyzer.RebuildFaceMode() = Standard_False;
    analyzer.TangentMode() = Standard_False; analyzer.MergeVertexMode() = Standard_False;
    analyzer.MergeEdgeMode() = Standard_False; analyzer.ContinuityMode() = Standard_False;
    analyzer.CurveOnSurfaceMode() = Standard_False; analyzer.Perform(progress->Start());
    return !analyzer.HasErrors() && !analyzer.HasWarnings() && !analyzer.HasFaulty();
}
// Boolean results may wrap the solid in compounds. Every leaf must be that
// one solid: a loose face/shell/wire or a second solid is never discarded.
inline bool singleSolid(const TopoDS_Shape& shape, TopoDS_Shape& solid) {
    if (shape.IsNull()) return false;
    if (shape.ShapeType() == TopAbs_SOLID) {
        if (!solid.IsNull() || shape.Orientation() != TopAbs_FORWARD) return false;
        solid = shape; return true;
    }
    if (shape.ShapeType() != TopAbs_COMPOUND) return false;
    bool any = false;
    for (TopoDS_Iterator it(shape); it.More(); it.Next()) {
        any = true; if (!singleSolid(it.Value(), solid)) return false;
    }
    return any;
}
inline bool stage(Budget& budget, const std::atomic_bool& cancelled, Refusal& refusal) {
    if (cancelled.load()) { refusal = Refusal::Cancelled; return false; }
    if (!budget.beginStage(Site::C16KernelBuild)) { refusal = Refusal::TopologyBudgetExceeded; return false; }
    return true;
}
template<class Operation>
inline bool booleanShape(const TopoDS_Shape& a, const TopoDS_Shape& b, TopoDS_Shape& result,
                          Budget& budget, const std::atomic_bool& cancelled, Refusal& refusal) {
    result.Nullify(); Census ca, cb, cr;
    if (!census(a, budget, cancelled, Site::C26AnalyticBoolean, ca, refusal, false)
        || !census(b, budget, cancelled, Site::C26AnalyticBoolean, cb, refusal, false)
        || !stage(budget, cancelled, refusal)) return false;
    TopTools_ListOfShape args, tools; args.Append(a); tools.Append(b);
    Operation op; op.SetArguments(args); op.SetTools(tools);
    op.SetNonDestructive(Standard_True); op.SetRunParallel(Standard_False);
    Handle(Progress) progress = new Progress(cancelled); op.Build(progress->Start());
    if (cancelled.load()) { refusal = Refusal::Cancelled; return false; }
    if (!op.IsDone() || op.HasErrors() || op.Shape().IsNull()) { refusal = Refusal::BuildFailure; return false; }
    if (!census(op.Shape(), budget, cancelled, Site::C11GeometryCensus, cr, refusal, false)) return false;
    result = op.Shape(); return true;
}
struct Loop final { TopoDS_Face face; double area = 0; };
inline bool loops(const TopoDS_Shape& shape, const gp_Pln& plane, std::vector<Loop>& result,
                   Budget& budget, const std::atomic_bool& cancelled, Refusal& refusal) {
    result.clear(); Census count;
    if (!census(shape, budget, cancelled, Site::C04WireExplorers, count, refusal, false)) return false;
    for (TopExp_Explorer it(shape, TopAbs_WIRE); it.More(); it.Next()) {
        if (result.size() == MaximumBoundaryWires || !reserve(count, budget, Site::C04WireExplorers, refusal))
            return false;
        const auto wire = TopoDS::Wire(it.Current());
        if (!BRep_Tool::IsClosed(wire)) return false;
        BRepBuilderAPI_MakeFace maker(plane, wire, Standard_True);
        if (!maker.IsDone() || !BRepCheck_Analyzer(maker.Face()).IsValid()) return false;
        const double mass = area(maker.Face());
        if (!std::isfinite(mass) || mass <= 0) return false;
        result.push_back({maker.Face(), mass});
    }
    std::sort(result.begin(), result.end(), [](const auto& a, const auto& b) { return a.area > b.area; });
    return !result.empty();
}
inline bool contains(const TopoDS_Face& outer, const TopoDS_Face& inner, double areaTolerance,
                      Budget& budget, const std::atomic_bool& cancelled, Refusal& refusal) {
    TopoDS_Shape common;
    if (!booleanShape<BRepAlgoAPI_Common>(outer, inner, common, budget, cancelled, refusal)) return false;
    return std::abs(area(common) - area(inner)) <= areaTolerance;
}
inline bool insetRegion(const TopoDS_Face& selected, double inset, double tol, double kernelScale, TopoDS_Face& region,
                         Budget& budget, const std::atomic_bool& cancelled, Refusal& refusal) {
    region.Nullify();
    const auto forward = TopoDS::Face(selected.Oriented(TopAbs_FORWARD));
    const gp_Pln plane = BRepAdaptor_Surface(forward, Standard_True).Plane();
    std::vector<Loop> original;
    refusal = Refusal::InvalidInset;
    if (!loops(forward, plane, original, budget, cancelled, refusal)) return false;
    const double originalArea = area(forward);
    const double areaTolerance = std::max(tol * tol, originalArea * 1e-9);
    // OCCT's offset sign depends on loop orientation. Try both fixed signs,
    // accepting only verified material erosion, never guessing an island.
    // SYPP/1 always offsets in a centered physical-millimetre frame. OCCT's
    // medial-axis offset can dereference a null bisector on metre-sized
    // circular wires; a fixed unit conversion avoids that kernel domain.
    // There is no retry with a different algorithm or changed distance.
    gp_Trsf toKernel; toKernel.SetScale(gp_Pnt(0,0,0), kernelScale);
    const gp_Pnt origin = plane.Location();
    toKernel.SetTranslationPart(gp_Vec(-origin.X()*kernelScale, -origin.Y()*kernelScale, -origin.Z()*kernelScale));
    BRepBuilderAPI_Transform normalized(forward, toKernel, Standard_True);
    if (!normalized.IsDone()) return false;
    const auto kernelFace = TopoDS::Face(normalized.Shape());
    std::size_t accepted = 0;
    for (double sign : {-1.0, 1.0}) {
        if (!stage(budget, cancelled, refusal)) return false;
        BRepOffsetAPI_MakeOffset offset(kernelFace, GeomAbs_Intersection, Standard_False);
        offset.SetApprox(Standard_False); offset.Perform(sign * inset * kernelScale, 0);
        if (!offset.IsDone() || offset.Shape().IsNull()) continue;
        BRepBuilderAPI_Transform restored(offset.Shape(), toKernel.Inverted(), Standard_True);
        if (!restored.IsDone()) continue;
        std::vector<Loop> candidate;
        if (!loops(restored.Shape(), plane, candidate, budget, cancelled, refusal)) {
            if (budget.exhausted || cancelled.load()) return false;
            continue;
        }
        if (candidate.size() != original.size() || candidate[0].area >= original[0].area - areaTolerance) continue;
        if (!contains(original[0].face, candidate[0].face, areaTolerance, budget, cancelled, refusal)) {
            if (budget.exhausted || cancelled.load()) return false;
            continue;
        }
        bool good = true;
        // Each old hole must grow into exactly one new hole; each new hole
        // must lie inside the outer loop. This also rejects multiple islands.
        std::vector<bool> used(candidate.size(), false); used[0] = true;
        for (std::size_t i = 1; i < original.size() && good; ++i) {
            std::size_t matches = 0, match = 0;
            for (std::size_t j = 1; j < candidate.size(); ++j) {
                if (candidate[j].area <= original[i].area + areaTolerance) continue;
                if (contains(candidate[j].face, original[i].face, areaTolerance, budget, cancelled, refusal)) {
                    ++matches; match = j;
                }
                if (budget.exhausted || cancelled.load()) return false;
            }
            good = matches == 1 && !used[match];
            if (good) used[match] = true;
        }
        BRepBuilderAPI_MakeFace maker(plane, BRepTools::OuterWire(candidate[0].face), Standard_True);
        for (std::size_t i = 1; i < candidate.size() && good; ++i) {
            good = contains(candidate[0].face, candidate[i].face, areaTolerance, budget, cancelled, refusal);
            if (budget.exhausted || cancelled.load()) return false;
            if (good) maker.Add(TopoDS::Wire(BRepTools::OuterWire(candidate[i].face).Reversed()));
        }
        if (!good || !maker.IsDone() || !BRepCheck_Analyzer(maker.Face()).IsValid()) continue;
        const double innerArea = area(maker.Face());
        if (!std::isfinite(innerArea) || innerArea <= areaTolerance || innerArea >= originalArea - areaTolerance) continue;
        if (!contains(forward, maker.Face(), areaTolerance, budget, cancelled, refusal)) {
            if (budget.exhausted || cancelled.load()) return false;
            continue;
        }
        // In particular, an offset beyond a collapsed polygon cannot return
        // a reversed, smaller polygon and masquerade as a valid erosion.
        double clearance = INFINITY;
        for (const auto& a : original) for (const auto& b : candidate) {
            if (!stage(budget, cancelled, refusal)) return false;
            BRepExtrema_DistShapeShape distance(BRepTools::OuterWire(a.face), BRepTools::OuterWire(b.face));
            if (!distance.IsDone() || distance.NbSolution() == 0) { good = false; break; }
            clearance = std::min(clearance, distance.Value());
        }
        if (!good || clearance < inset - 4 * tol) continue;
        region = TopoDS::Face(maker.Face().Oriented(selected.Orientation())); ++accepted;
    }
    if (accepted != 1) { region.Nullify(); refusal = Refusal::InvalidInset; return false; }
    refusal = Refusal::None; return true;
}
} // namespace detail

inline BuildResult Build(const TopoDS_Shape& source, const Definition& definition,
                          const std::atomic_bool& cancelled, detail::Budget& budget) noexcept {
    BuildResult output;
    const auto decline = [&](Refusal value) {
        BuildResult result; result.refusal = cancelled.load() ? Refusal::Cancelled
            : budget.exhausted ? Refusal::TopologyBudgetExceeded : value;
        return result; // no partial shape or face escapes a refusal
    };
    try {
        if (cancelled.load()) return decline(Refusal::Cancelled);
        Refusal refusal;
        if (!Validate(definition, refusal)) return decline(refusal);
        if (source.IsNull() || source.ShapeType() != TopAbs_SOLID || source.Orientation() != TopAbs_FORWARD)
            return decline(Refusal::InvalidSource);
        detail::Census sourceCount;
        if (!detail::census(source, budget, cancelled, detail::Site::C11GeometryCensus, sourceCount, refusal))
            return decline(refusal);
        // Reserve validity, exact serialization, volume and deep copy passes.
        for (int i = 0; i < 4; ++i)
            if (!detail::reserve(sourceCount, budget, detail::Site::C12Detachment, refusal)) return decline(refusal);
        if (!BRepCheck_Analyzer(source).IsValid()) return decline(Refusal::InvalidSource);
        output.evidence.sourceVolumeLocal = detail::volume(source);
        if (!std::isfinite(output.evidence.sourceVolumeLocal) || output.evidence.sourceVolumeLocal <= 0)
            return decline(Refusal::InvalidSource);
        std::vector<std::uint8_t> before, after, bytes;
        if (!detail::exactShapeBytes(source, before)) return decline(Refusal::InvalidSource);
        if (!detail::stage(budget, cancelled, refusal)) return decline(refusal);
        BRepBuilderAPI_Copy copy(source, Standard_True, Standard_False);
        if (!copy.IsDone() || copy.Shape().IsNull()) return decline(Refusal::BuildFailure);
        TopoDS_Shape detached = copy.Shape(); TopoDS_Face selected;
        const double tol = detail::tolerance(definition);
        if (!Resolve(detached, definition.face, tol, selected, refusal, budget, cancelled)) return decline(refusal);
        if (definition.signedDistanceLocal == 0 && definition.insetLocal == 0) return decline(Refusal::Unchanged);
        if ((definition.signedDistanceLocal != 0 && std::abs(definition.signedDistanceLocal) <= tol)
            || (definition.insetLocal != 0 && definition.insetLocal <= tol)) return decline(Refusal::InvalidParameters);
        Handle(detail::Progress) progress = new detail::Progress(cancelled);
        if (!detail::stage(budget, cancelled, refusal)) return decline(refusal);
        if (!detail::selfIntersectionFree(detached, progress)) return decline(Refusal::InvalidSource);
        TopoDS_Face region = selected;
        if (definition.insetLocal > 0) {
            if (!detail::insetRegion(selected, definition.insetLocal, tol,
                    1000 * definition.metersPerLocalUnit * definition.uniformScale, region, budget, cancelled, refusal))
                return decline(refusal);
            if (!detail::stage(budget, cancelled, refusal)) return decline(refusal);
            BRepFeat_SplitShape splitter(detached);
            for (TopExp_Explorer it(region, TopAbs_WIRE); it.More(); it.Next())
                splitter.Add(TopoDS::Wire(it.Current()), selected);
            splitter.Build(progress->Start());
            if (!splitter.IsDone() || splitter.Shape().IsNull()) return decline(Refusal::InvalidInset);
            detail::Census splitCount;
            if (!detail::census(splitter.Shape(), budget, cancelled, detail::Site::C11GeometryCensus, splitCount, refusal, false))
                return decline(refusal);
            detached = splitter.Shape();
        }
        FaceSelector expected;
        if (!detail::describe(region, definition.outputFaceIdentifier, expected, budget, cancelled, refusal)) return decline(refusal);
        output.evidence.regionAreaLocal = expected.areaLocal;
        // Resolve the actual split inner face before using it as the prism base.
        if (definition.insetLocal > 0
            && !Resolve(detached, expected, tol, region, refusal, budget, cancelled)) return decline(refusal);
        TopoDS_Shape candidate = detached;
        if (definition.signedDistanceLocal != 0) {
            if (!detail::stage(budget, cancelled, refusal)) return decline(refusal);
            const gp_Vec movement = gp_Vec(detail::direction(definition.face.outwardNormalLocal))
                * definition.signedDistanceLocal;
            BRepPrimAPI_MakePrism prism(region, movement, Standard_True, Standard_True);
            if (!prism.IsDone() || prism.Shape().IsNull()) return decline(Refusal::BuildFailure);
            const bool success = definition.signedDistanceLocal > 0
                ? detail::booleanShape<BRepAlgoAPI_Fuse>(detached, prism.Shape(), candidate, budget, cancelled, refusal)
                : detail::booleanShape<BRepAlgoAPI_Cut>(detached, prism.Shape(), candidate, budget, cancelled, refusal);
            if (!success) return decline(refusal);
            for (std::size_t i = 0; i < 3; ++i)
                expected.centroidLocal[i] += definition.face.outwardNormalLocal[i] * definition.signedDistanceLocal;
        }
        detail::Census resultCount;
        if (!detail::census(candidate, budget, cancelled, detail::Site::C11GeometryCensus, resultCount, refusal))
            return decline(refusal);
        TopoDS_Shape solid;
        if (!detail::singleSolid(candidate, solid)) return decline(Refusal::DisconnectedSolid);
        for (int i = 0; i < 4; ++i)
            if (!detail::reserve(resultCount, budget, detail::Site::C08ReceiptVerify, refusal)) return decline(refusal);
        if (!BRepCheck_Analyzer(solid).IsValid()) return decline(Refusal::InvalidCandidate);
        if (!detail::stage(budget, cancelled, refusal)) return decline(refusal);
        if (!detail::selfIntersectionFree(solid, progress)) return decline(Refusal::InvalidCandidate);
        output.evidence.resultVolumeLocal = detail::volume(solid);
        const double volumeTolerance = std::max(std::pow(tol, 3), output.evidence.sourceVolumeLocal * 1e-9);
        const double delta = output.evidence.resultVolumeLocal - output.evidence.sourceVolumeLocal;
        if (!std::isfinite(output.evidence.resultVolumeLocal) || output.evidence.resultVolumeLocal <= volumeTolerance
            || (definition.signedDistanceLocal == 0 && std::abs(delta) > volumeTolerance)
            || (definition.signedDistanceLocal > 0 && delta <= volumeTolerance)
            || (definition.signedDistanceLocal < 0 && delta >= -volumeTolerance)) return decline(Refusal::InvalidCandidate);
        TopoDS_Face cap;
        if (!Resolve(solid, expected, tol, cap, refusal, budget, cancelled)) return decline(refusal);
        if (!detail::reserve(sourceCount, budget, detail::Site::C08ReceiptVerify, refusal)) return decline(refusal);
        if (!detail::exactShapeBytes(source, after) || before != after) return decline(Refusal::SourceMutation);
        if (solid.IsSame(source) || !detail::exactShapeBytes(solid, bytes) || bytes == before)
            return decline(Refusal::InvalidCandidate);
        if (cancelled.load()) return decline(Refusal::Cancelled);
        output.solid = solid; output.region = region; output.outputFace = cap; output.outputSelector = expected;
        output.evidence.connectedSolids = 1; output.evidence.sourceBytesUnchanged = true;
        output.evidence.candidateBytesDistinct = true; output.refusal = Refusal::None; return output;
    } catch (...) { return decline(Refusal::BuildFailure); }
}
inline BuildResult Build(const TopoDS_Shape& source, const Definition& definition,
                          const std::atomic_bool& cancelled) noexcept {
    detail::Budget budget; return Build(source, definition, cancelled, budget);
}
inline BuildResult BuildDeterministically(const TopoDS_Shape& source, const Definition& definition,
                                           const std::atomic_bool& cancelled, detail::Budget& budget) noexcept {
    BuildResult first = Build(source, definition, cancelled, budget);
    if (!first.built()) return first;
    BuildResult second = Build(source, definition, cancelled, budget);
    if (!second.built()) return second;
    try {
        if (retained_topology_budget::ChargeTraversal(first.solid, budget, cancelled, detail::Site::C08ReceiptVerify)
                != retained_topology_budget::WalkStatus::Completed
            || retained_topology_budget::ChargeTraversal(second.solid, budget, cancelled, detail::Site::C08ReceiptVerify)
                != retained_topology_budget::WalkStatus::Completed) {
            BuildResult result; result.refusal = cancelled.load() ? Refusal::Cancelled : Refusal::TopologyBudgetExceeded; return result;
        }
        std::vector<std::uint8_t> a, b;
        if (!detail::exactShapeBytes(first.solid, a) || !detail::exactShapeBytes(second.solid, b) || a != b) {
            BuildResult result; result.refusal = Refusal::NonDeterministic; return result;
        }
        return first;
    } catch (...) { BuildResult result; result.refusal = Refusal::BuildFailure; return result; }
}
inline BuildResult BuildDeterministically(const TopoDS_Shape& source, const Definition& definition,
                                           const std::atomic_bool& cancelled) noexcept {
    detail::Budget budget; return BuildDeterministically(source, definition, cancelled, budget);
}
} // namespace core3d::face_push_pull
