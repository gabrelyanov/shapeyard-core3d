#pragma once
// Detached geometry only. No OCAF command, owner, catalog, material, receipt or
// public UI/AI API is provided. Caller supplies an already detached base shape.
#include "AnalyticBooleanRingOperand.hxx"
#include "AnalyticBooleanWedgeOperand.hxx"
#include "RetainedTopologyBudget.hxx"
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
// Deliberate dependency on existing cancellation and exact self-interference
// helpers. No duplicated feature authority or alternate geometry engine.
#include "PlanarSweepSolid.hxx"
#include <BRepAlgoAPI_Cut.hxx>
#include <Precision.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <vector>

namespace core3d::analytic_boolean {
namespace tb = core3d::retained_topology_budget;
enum class Status { Built, InvalidRecipe, Cancelled, UnsupportedSource,
    UnsupportedTool, BudgetExceeded, KernelFailure, UnsupportedResult,
    NoRemovedVolume, VerificationFailed };
struct Result {
    TopoDS_Shape solid;
    std::array<double,6> sourceBounds{}, resultBounds{}; // xmin,ymin,zmin,xmax,ymax,zmax.
    double sourceVolume = 0, resultVolume = 0, removedVolume = 0;
    double toolStart = 0, toolEnd = 0; // Native coordinate along declared axis.
};
namespace detail {
inline bool Bounded(const TopoDS_Shape& shape, std::size_t limit,
    const std::atomic_bool& stop,
    tb::Counter* shared = nullptr, tb::Site site = tb::Site::None) {
    if (shape.IsNull()) return false;
    struct Item { TopoDS_Shape shape; unsigned depth; };
    std::vector<Item> pending{{shape,0}};
    std::size_t visited = 0;
    while (!pending.empty()) {
        if (stop.load() || ++visited > limit) return false;
        // C25/C26: with a shared operation counter every raw occurrence pays;
        // the legacy local bound above stays in force unchanged.
        if (shared && !shared->visit(1, site)) return false;
        const auto item = pending.back(); pending.pop_back();
        if (item.depth > 64) return false;
        for (TopoDS_Iterator it(item.shape); it.More(); it.Next()) {
            if (pending.size() >= limit || stop.load()) return false;
            pending.push_back({it.Value(),item.depth+1});
        }
    }
    return true; // Counts occurrences as well as shared unique subshapes.
}
inline bool Bounds(const TopoDS_Shape& shape, double mm, std::array<double,6>& out,
    tb::Counter* shared = nullptr, tb::Site site = tb::Site::None) {
    Bnd_Box box;
    // A trimmed planar polygon attains its extrema at its vertices. Measure
    // those stored points directly: evaluating the plane at UV limits can
    // move an extremum by one ULP when binary reload renormalizes its axes.
    // Curved faces/edges retain the kernel's analytic extrema calculation.
    // This uses actual topology, never the source recipe or rounded values.
    if (shape.IsNull() || shape.ShapeType()!=TopAbs_SOLID) return false;
    for (TopExp_Explorer faces(shape,TopAbs_FACE);faces.More();faces.Next()) {
        // C25: every inspected face/edge/vertex is charged to the shared
        // operation counter when one is borrowed.
        if (shared && !shared->visit(1, site)) return false;
        const auto face=TopoDS::Face(faces.Current());
        bool polygon=BRepAdaptor_Surface(face).GetType()==GeomAbs_Plane;
        unsigned edges=0;
        for (TopExp_Explorer it(face,TopAbs_EDGE);polygon&&it.More();it.Next()) {
            if (shared && !shared->visit(1, site)) return false;
            ++edges;
            polygon=BRepAdaptor_Curve(TopoDS::Edge(it.Current())).GetType()==GeomAbs_Line;
        }
        if (!polygon || edges<3) {
            BRepBndLib::AddOptimal(face,box,Standard_False,Standard_False);
            continue;
        }
        unsigned vertices=0;
        for (TopExp_Explorer it(face,TopAbs_VERTEX);it.More();it.Next()) {
            if (shared && !shared->visit(1, site)) return false;
            box.Add(BRep_Tool::Pnt(TopoDS::Vertex(it.Current())));
            ++vertices;
        }
        if (vertices<3) return false;
    }
    if (box.IsVoid() || box.IsWhole() || box.IsOpen()) return false;
    box.Get(out[0],out[1],out[2],out[3],out[4],out[5]);
    for (double x : out) if (!std::isfinite(x) || !std::isfinite(x*mm) || std::abs(x*mm)>1e6) return false;
    for (unsigned i=0;i<3;++i) if (out[i+3]<=out[i]) return false;
    return true;
}
inline bool ValidSolid(const TopoDS_Shape& shape, double& volume,
    tb::Counter* shared = nullptr, tb::Site site = tb::Site::None) {
    if (shape.IsNull() || shape.ShapeType()!=TopAbs_SOLID
        || !BRepCheck_Analyzer(shape,Standard_True).IsValid()) return false;
    unsigned shells=0;
    for (TopExp_Explorer it(shape,TopAbs_SHELL);it.More();it.Next()) {
        // C25: every inspected shell is charged to the shared counter.
        if (shared && !shared->visit(1, site)) return false;
        ++shells;if (!BRep_Tool::IsClosed(it.Current())) return false;
    }
    if (!shells) return false;
    BRepClass3d_SolidClassifier classifier(shape);
    classifier.PerformInfinitePoint(Precision::Confusion());
    if (classifier.State()!=TopAbs_OUT) return false; // Never auto-reverse source geometry.
    GProp_GProps props;
    BRepGProp::VolumeProperties(shape,props,Standard_True,Standard_False,Standard_False);
    volume=props.Mass(); return std::isfinite(volume) && volume>0;
}
inline bool SingleResult(const TopoDS_Shape& raw, TopoDS_Shape& solid,
    tb::Counter* shared = nullptr, tb::Site site = tb::Site::None) {
    if (raw.ShapeType()==TopAbs_SOLID) {solid=raw;return true;}
    if (raw.ShapeType()!=TopAbs_COMPOUND) return false;
    unsigned children=0;
    for (TopoDS_Iterator it(raw);it.More();it.Next()) {
        // C25: every inspected result child is charged to the shared counter.
        if (shared && !shared->visit(1, site)) return false;
        if (++children!=1 || it.Value().ShapeType()!=TopAbs_SOLID) return false;
        solid=it.Value();
    }
    return children==1; // No silently discarded second solid, sheet or wire.
}
}

// The D4 feature-pattern surface needs the exact primitive that a retained
// Boolean step would use, without replaying or mutating the surrounding
// program.  Keep that construction here so complete-program replay and the
// detached consumer cannot drift in margin, axis, or bounds semantics.
struct CylinderToolResult {
    TopoDS_Shape solid;
    std::array<double, 6> sourceBounds{};
    double sourceVolume = 0;
    double toolStart = 0, toolEnd = 0;
};

inline Status BuildCylinderToolShared(const TopoDS_Shape& detachedBase,
    const Recipe& recipe, const std::atomic_bool& stop,
    CylinderToolResult& output, tb::Counter* shared) noexcept {
    output = {};
    if (stop.load()) return Status::Cancelled;
    if (!Inspect(recipe)) return Status::InvalidRecipe;
    if (recipe.operation != Operation::Difference
        || recipe.tool.kind != OperandKind::Cylinder)
        return Status::UnsupportedTool;
    try {
        OCC_CATCH_SIGNALS
        if (!detail::Bounded(detachedBase, 1024, stop, shared, tb::Site::C25AnalyticInput))
            return stop.load() ? Status::Cancelled : Status::BudgetExceeded;
        if (detachedBase.ShapeType() != TopAbs_SOLID)
            return Status::UnsupportedSource;
        CylinderToolResult result;
        const double mm = recipe.metersPerUnit * 1000;
        if (shared) {
            // C25: validity, classification, volume and bounds passes over the
            // input are debited before they run.
            for (int pass = 0; pass < 4; ++pass) {
                const auto walk = tb::ChargeTraversal(detachedBase, *shared, stop,
                    tb::Site::C25AnalyticInput);
                if (walk == tb::WalkStatus::Cancelled) return Status::Cancelled;
                if (walk != tb::WalkStatus::Completed) return Status::BudgetExceeded;
            }
        }
        if (!detail::ValidSolid(detachedBase, result.sourceVolume, shared, tb::Site::C25AnalyticInput)
            || !detail::Bounds(detachedBase, mm, result.sourceBounds, shared, tb::Site::C25AnalyticInput))
            return shared && shared->exhausted ? Status::BudgetExceeded : Status::UnsupportedSource;
        const unsigned axis = static_cast<unsigned>(recipe.tool.axis);
        const double tolerance = std::max(Precision::Confusion() * 32, 1e-5 / mm);
        const double margin = std::max(tolerance * 4, .001 / mm);
        if (!std::isfinite(tolerance) || !std::isfinite(margin)
            || recipe.tool.radius <= tolerance) return Status::InvalidRecipe;
        result.toolStart = result.sourceBounds[axis] - margin;
        result.toolEnd = result.sourceBounds[axis + 3] + margin;
        const double length = result.toolEnd - result.toolStart;
        if (!std::isfinite(length) || length <= 0 || !std::isfinite(length * mm)
            || length * mm > 2e6 + 1) return Status::InvalidRecipe;
        auto origin = recipe.tool.point;
        origin[axis] = result.toolStart;
        const gp_Dir direction = axis == 0 ? gp::DX() : axis == 1 ? gp::DY() : gp::DZ();
        BRepPrimAPI_MakeCylinder cylinder(
            gp_Ax2(gp_Pnt(origin[0], origin[1], origin[2]), direction),
            recipe.tool.radius, length);
        cylinder.Build();
        if (!cylinder.IsDone() || stop.load())
            return stop.load() ? Status::Cancelled : Status::KernelFailure;
        double toolVolume = 0;
        if (!detail::ValidSolid(cylinder.Shape(), toolVolume, shared, tb::Site::C25AnalyticInput)
            || !detail::Bounded(cylinder.Shape(), 1024, stop, shared, tb::Site::C25AnalyticInput))
            return stop.load() ? Status::Cancelled
                : (shared && shared->exhausted ? Status::BudgetExceeded : Status::KernelFailure);
        result.solid = cylinder.Shape();
        output = std::move(result);
        return Status::Built;
    } catch (...) {
        output = {};
        return stop.load() ? Status::Cancelled : Status::KernelFailure;
    }
}

inline Status BuildCylinderTool(const TopoDS_Shape& detachedBase,
    const Recipe& recipe, const std::atomic_bool& stop,
    CylinderToolResult& output) noexcept {
    // Compatibility entry: no shared operation context.
    return BuildCylinderToolShared(detachedBase, recipe, stop, output, nullptr);
}
// C25/C26: budgeted overload for B2 prefix paths; charges the shared
// per-operation counter while keeping the legacy local bounds in force.
inline Status BuildCylinderTool(const TopoDS_Shape& detachedBase,
    const Recipe& recipe, const std::atomic_bool& stop,
    CylinderToolResult& output, tb::Counter& shared) noexcept {
    return BuildCylinderToolShared(detachedBase, recipe, stop, output, &shared);
}

inline Status BuildShared(const TopoDS_Shape& detachedBase, const Recipe& recipe,
    const std::atomic_bool& stop, Result& output, double expectedWedgeVolume,
    tb::Counter* shared) noexcept {
    output={};
    if (stop.load()) return Status::Cancelled;
    if (!Inspect(recipe)) return Status::InvalidRecipe;
    try {
        OCC_CATCH_SIGNALS
        // C25: the bounded input census charges every raw occurrence; the
        // legacy local 1,024/32,768 limits stay in force unchanged.
        if (!detail::Bounded(detachedBase,1024,stop,shared,tb::Site::C25AnalyticInput)) return stop.load()?Status::Cancelled:Status::BudgetExceeded;
        if (detachedBase.ShapeType()!=TopAbs_SOLID) return Status::UnsupportedSource;
        if (shared) {
            // C26: the private-copy pass is reserved before it runs.
            const auto copyWalk=tb::ChargeTraversal(detachedBase,*shared,stop,tb::Site::C26AnalyticBoolean);
            if (copyWalk==tb::WalkStatus::Cancelled) return Status::Cancelled;
            if (copyWalk!=tb::WalkStatus::Completed) return Status::BudgetExceeded;
        }
        // Even detached caller input is copied with geometry; OCCT work never
        // edits the supplied source. Copy excludes mesh caches deliberately.
        BRepBuilderAPI_Copy copy(detachedBase,Standard_True,Standard_False);
        if (!copy.IsDone()) return Status::KernelFailure;
        const auto base=copy.Shape();
        Result result;
        const double mm=recipe.metersPerUnit*1000;
        if (shared) {
            // C25: validity, classification, volume and bounds passes over the
            // copied base are debited before they run.
            for (int pass=0;pass<4;++pass) {
                const auto walk=tb::ChargeTraversal(base,*shared,stop,tb::Site::C25AnalyticInput);
                if (walk==tb::WalkStatus::Cancelled) return Status::Cancelled;
                if (walk!=tb::WalkStatus::Completed) return Status::BudgetExceeded;
            }
        }
        if (!detail::ValidSolid(base,result.sourceVolume,shared,tb::Site::C25AnalyticInput)
            || !detail::Bounds(base,mm,result.sourceBounds,shared,tb::Site::C25AnalyticInput))
            return shared&&shared->exhausted?Status::BudgetExceeded:Status::UnsupportedSource;
        if (shared) {
            // C26: the self-interference pass is reserved before it runs.
            const auto walk=tb::ChargeTraversal(base,*shared,stop,tb::Site::C26AnalyticBoolean);
            if (walk==tb::WalkStatus::Cancelled) return Status::Cancelled;
            if (walk!=tb::WalkStatus::Completed) return Status::BudgetExceeded;
        }
        Handle(Message_ProgressIndicator) indicator=new planar_sweep::detail::CancellationProgress(stop);
        Message_ProgressScope progress(indicator->Start(),"Cylindrical through-cut",3);
        if (planar_sweep::detail::CheckInterference(base,stop,progress.Next())!=planar_sweep::BuildStatus::Built)
            return stop.load()?Status::Cancelled:Status::UnsupportedSource;
        const unsigned axis=static_cast<unsigned>(recipe.tool.axis);
        const double tolerance=std::max(Precision::Confusion()*32,1e-5/mm);
        const double margin=std::max(tolerance*4,.001/mm);
        if (!std::isfinite(tolerance) || !std::isfinite(margin) || (recipe.tool.kind!=OperandKind::Wedge&&recipe.tool.radius<=tolerance))
            return Status::InvalidRecipe;
        result.toolStart=result.sourceBounds[axis]-margin;
        result.toolEnd=result.sourceBounds[axis+3]+margin;
        const double length=result.toolEnd-result.toolStart;
        if (!std::isfinite(length) || length<=0 || !std::isfinite(length*mm) || length*mm>2e6+1)
            return Status::InvalidRecipe;
        const bool ring=recipe.tool.kind==OperandKind::CylinderRing;
        const auto ringValue=analytic_boolean_ring::FromOperand(recipe.tool,recipe.metersPerUnit);
        if(ring&&analytic_boolean_ring::Inspect(ringValue,recipe.metersPerUnit)!=analytic_boolean_ring::Status::Clear)
            return Status::InvalidRecipe;
        const unsigned diskCount=ring?recipe.tool.count:1;
        TopTools_ListOfShape tools;
        const bool wedge=recipe.tool.kind==OperandKind::Wedge;
        if(wedge){
            if(!std::isfinite(expectedWedgeVolume)||expectedWedgeVolume<=0)return Status::InvalidRecipe;
            BRepBuilderAPI_MakePolygon polygon;const auto section=analytic_boolean_wedge::Expand(recipe.tool);
            for(const auto& point:section){gp_Pnt p;p.SetCoord(axis+1,result.toolStart);
                p.SetCoord((axis+1)%3+1,point[0]);p.SetCoord((axis+2)%3+1,point[1]);polygon.Add(p);}
            polygon.Close();if(!polygon.IsDone())return Status::KernelFailure;
            BRepBuilderAPI_MakeFace face(polygon.Wire());if(!face.IsDone())return Status::KernelFailure;
            gp_Vec direction;direction.SetCoord(axis+1,length);BRepPrimAPI_MakePrism prism(face.Face(),direction);
            prism.Build();if(!prism.IsDone()||stop.load())return stop.load()?Status::Cancelled:Status::KernelFailure;
            // C26: each constructed tool solid is one charged unit.
            if(shared&&!shared->visit(1,tb::Site::C26AnalyticBoolean))return Status::BudgetExceeded;
            tools.Append(prism.Shape());
        }else for(unsigned k=0;k<diskCount;++k){
            auto disk=ring?analytic_boolean_ring::Expand(ringValue,k,recipe.metersPerUnit):recipe.tool;
            if(!disk.identifier)return Status::InvalidRecipe;
            if(!ring){
                CylinderToolResult detached;
                const auto toolStatus=BuildCylinderToolShared(base,recipe,stop,detached,shared);
                if(toolStatus!=Status::Built)return toolStatus;
                result.toolStart=detached.toolStart;result.toolEnd=detached.toolEnd;
                tools.Append(detached.solid);continue;
            }
            auto origin=disk.point;origin[axis]=result.toolStart;
            const gp_Dir direction=axis==0?gp::DX():axis==1?gp::DY():gp::DZ();
            BRepPrimAPI_MakeCylinder cylinder(gp_Ax2(gp_Pnt(origin[0],origin[1],origin[2]),direction),recipe.tool.radius,length);
            cylinder.Build();if(!cylinder.IsDone())return Status::KernelFailure;
            if(stop.load())return Status::Cancelled;
            if(shared&&!shared->visit(1,tb::Site::C26AnalyticBoolean))return Status::BudgetExceeded;
            tools.Append(cylinder.Shape());
        }
        // The kernel's section curves and edge tolerances carry an absolute
        // model-unit precision, so at meter-scale local units they are
        // physically 1000x coarser against the same millimeter-sized geometry
        // (a measured 1.3e-4 relative removed-volume loss on a 3 mm
        // transverse bore). Run the kernel cut in millimeter coordinates and
        // scale the produced shape back; the millimeter unit system keeps the
        // legacy path and its exact bit pattern. The recipe, tools, budgets
        // and every product-side check stay in local coordinates.
        TopoDS_Shape kernelBase=base;
        TopTools_ListOfShape kernelTools;
        gp_Trsf kernelDown;
        if(mm!=1.0){
            gp_Trsf kernelUp;kernelUp.SetScale(gp::Origin(),mm);
            kernelDown.SetScale(gp::Origin(),1/mm);
            kernelBase=BRepBuilderAPI_Transform(base,kernelUp,Standard_True).Shape();
            for(const auto& tool:tools)
                kernelTools.Append(BRepBuilderAPI_Transform(tool,kernelUp,Standard_True).Shape());
        }else kernelTools=tools;
        TopTools_ListOfShape kernelArguments;kernelArguments.Append(kernelBase);
        if (shared) {
            // C26: the Boolean stage is debited and its input passes (base
            // plus every tool) are reserved before the kernel runs.
            if(!shared->beginStage(tb::Site::C26AnalyticBoolean))return Status::BudgetExceeded;
            const auto baseWalk=tb::ChargeTraversal(base,*shared,stop,tb::Site::C26AnalyticBoolean);
            if (baseWalk==tb::WalkStatus::Cancelled) return Status::Cancelled;
            if (baseWalk!=tb::WalkStatus::Completed) return Status::BudgetExceeded;
            for(const auto& tool:tools){
                const auto toolWalk=tb::ChargeTraversal(tool,*shared,stop,tb::Site::C26AnalyticBoolean);
                if (toolWalk==tb::WalkStatus::Cancelled) return Status::Cancelled;
                if (toolWalk!=tb::WalkStatus::Completed) return Status::BudgetExceeded;
            }
        }
        BRepAlgoAPI_Cut cut;
        cut.SetArguments(kernelArguments);cut.SetTools(kernelTools);
        cut.SetRunParallel(Standard_False);cut.SetNonDestructive(Standard_True);
        cut.SetFuzzyValue(0);cut.SetUseOBB(Standard_True);cut.SetCheckInverted(Standard_True);
        cut.Build(progress.Next());
        if (stop.load()) return Status::Cancelled;
        if (!cut.IsDone() || cut.HasErrors() || cut.HasWarnings() || cut.Shape().IsNull()) return Status::KernelFailure;
        TopoDS_Shape produced=cut.Shape();
        if(mm!=1.0)produced=BRepBuilderAPI_Transform(produced,kernelDown,Standard_True).Shape();
        if (shared) {
            // C26: the produced shape is censused before any consumer, and
            // the result validity/classification/volume/bounds passes are
            // debited before they run.
            tb::Census outputCensus;
            const auto censusWalk=tb::CensusTopology(produced,*shared,stop,outputCensus,
                tb::Site::C26AnalyticBoolean,false);
            if (censusWalk==tb::WalkStatus::Cancelled) return Status::Cancelled;
            if (censusWalk!=tb::WalkStatus::Completed) return Status::BudgetExceeded;
            for (int pass=0;pass<4;++pass) {
                const auto walk=tb::ChargeTraversal(produced,*shared,stop,tb::Site::C26AnalyticBoolean);
                if (walk==tb::WalkStatus::Cancelled) return Status::Cancelled;
                if (walk!=tb::WalkStatus::Completed) return Status::BudgetExceeded;
            }
        }
        if (!detail::Bounded(produced,32768,stop,shared,tb::Site::C26AnalyticBoolean)) return stop.load()?Status::Cancelled:Status::BudgetExceeded;
        if (!detail::SingleResult(produced,result.solid,shared,tb::Site::C26AnalyticBoolean)
            || !detail::ValidSolid(result.solid,result.resultVolume,shared,tb::Site::C26AnalyticBoolean)
            || !detail::Bounds(result.solid,mm,result.resultBounds,shared,tb::Site::C26AnalyticBoolean))
            return shared&&shared->exhausted?Status::BudgetExceeded:Status::UnsupportedResult;
        if (shared) {
            const auto walk=tb::ChargeTraversal(result.solid,*shared,stop,tb::Site::C26AnalyticBoolean);
            if (walk==tb::WalkStatus::Cancelled) return Status::Cancelled;
            if (walk!=tb::WalkStatus::Completed) return Status::BudgetExceeded;
        }
        if (planar_sweep::detail::CheckInterference(result.solid,stop,progress.Next())!=planar_sweep::BuildStatus::Built)
            return stop.load()?Status::Cancelled:Status::UnsupportedResult;
        result.removedVolume=result.sourceVolume-result.resultVolume;
        // Distinguish real subtraction from a tangent/disjoint/no-volume case.
        // This is an admission precision bound, not a geometric oracle tolerance.
        const double minRemoved=std::max(tolerance*tolerance*tolerance,result.sourceVolume*1e-12);
        if (!std::isfinite(result.removedVolume) || result.removedVolume<=minRemoved) return Status::NoRemovedVolume;
        if(wedge&&std::abs(result.removedVolume-expectedWedgeVolume)>std::max(1e-9*expectedWedgeVolume,tolerance*tolerance*tolerance))
            return Status::VerificationFailed;
        if(ring){
            const double required=diskCount*std::acos(-1.0)*recipe.tool.radius*recipe.tool.radius
                *(result.sourceBounds[axis+3]-result.sourceBounds[axis]);
            if(!std::isfinite(required)||result.removedVolume+std::max(minRemoved,required*1e-9)<required)
                return Status::NoRemovedVolume;
        }
        for (unsigned i=0;i<3;++i)
            if (result.resultBounds[i]<result.sourceBounds[i]-tolerance
                || result.resultBounds[i+3]>result.sourceBounds[i+3]+tolerance) return Status::VerificationFailed;
        if (stop.load()) return Status::Cancelled;
        output=std::move(result);return Status::Built;
    } catch (...) {output={};return stop.load()?Status::Cancelled:Status::KernelFailure;}
}

inline Status Build(const TopoDS_Shape& detachedBase, const Recipe& recipe,
    const std::atomic_bool& stop, Result& output, double expectedWedgeVolume=0) noexcept {
    // Compatibility entry: unrelated callers keep their existing local
    // behavior with no shared operation context.
    return BuildShared(detachedBase, recipe, stop, output, expectedWedgeVolume, nullptr);
}
// C25/C26: budgeted overload for B2 prefix paths; the shared per-operation
// counter pays the raw occurrences, passes and the Boolean stage while the
// legacy local bounds stay in force.
inline Status Build(const TopoDS_Shape& detachedBase, const Recipe& recipe,
    const std::atomic_bool& stop, Result& output, double expectedWedgeVolume,
    tb::Counter& shared) noexcept {
    return BuildShared(detachedBase, recipe, stop, output, expectedWedgeVolume, &shared);
}
} // namespace core3d::analytic_boolean
