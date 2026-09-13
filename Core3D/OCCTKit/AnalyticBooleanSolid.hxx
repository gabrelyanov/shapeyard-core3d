#pragma once
// Detached geometry only. No OCAF command, owner, catalog, material, receipt or
// public UI/AI API is provided. Caller supplies an already detached base shape.
#include "AnalyticBooleanOperand.hxx"
// Deliberate dependency on existing cancellation and exact self-interference
// helpers. No duplicated feature authority or alternate geometry engine.
#include "PlanarSweepSolid.hxx"
#include <BRepAlgoAPI_Cut.hxx>
#include <Precision.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListOfShape.hxx>
#include <vector>

namespace core3d::analytic_boolean {
enum class Status { Built, InvalidRecipe, Cancelled, UnsupportedSource,
    BudgetExceeded, KernelFailure, UnsupportedResult, NoRemovedVolume, VerificationFailed };
struct Result {
    TopoDS_Shape solid;
    std::array<double,6> sourceBounds{}, resultBounds{}; // xmin,ymin,zmin,xmax,ymax,zmax.
    double sourceVolume = 0, resultVolume = 0, removedVolume = 0;
    double toolStart = 0, toolEnd = 0; // Native coordinate along declared axis.
};
namespace detail {
inline bool Bounded(const TopoDS_Shape& shape, std::size_t limit,
    const std::atomic_bool& stop) {
    if (shape.IsNull()) return false;
    struct Item { TopoDS_Shape shape; unsigned depth; };
    std::vector<Item> pending{{shape,0}};
    std::size_t visited = 0;
    while (!pending.empty()) {
        if (stop.load() || ++visited > limit) return false;
        const auto item = pending.back(); pending.pop_back();
        if (item.depth > 64) return false;
        for (TopoDS_Iterator it(item.shape); it.More(); it.Next()) {
            if (pending.size() >= limit || stop.load()) return false;
            pending.push_back({it.Value(),item.depth+1});
        }
    }
    return true; // Counts occurrences as well as shared unique subshapes.
}
inline bool Bounds(const TopoDS_Shape& shape, double mm, std::array<double,6>& out) {
    Bnd_Box box; BRepBndLib::AddOptimal(shape,box,Standard_False,Standard_False);
    if (box.IsVoid() || box.IsWhole() || box.IsOpen()) return false;
    box.Get(out[0],out[1],out[2],out[3],out[4],out[5]);
    for (double x : out) if (!std::isfinite(x) || !std::isfinite(x*mm) || std::abs(x*mm)>1e6) return false;
    for (unsigned i=0;i<3;++i) if (out[i+3]<=out[i]) return false;
    return true;
}
inline bool ValidSolid(const TopoDS_Shape& shape, double& volume) {
    if (shape.IsNull() || shape.ShapeType()!=TopAbs_SOLID
        || !BRepCheck_Analyzer(shape,Standard_True).IsValid()) return false;
    unsigned shells=0;
    for (TopExp_Explorer it(shape,TopAbs_SHELL);it.More();it.Next()) {
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
inline bool SingleResult(const TopoDS_Shape& raw, TopoDS_Shape& solid) {
    if (raw.ShapeType()==TopAbs_SOLID) {solid=raw;return true;}
    if (raw.ShapeType()!=TopAbs_COMPOUND) return false;
    unsigned children=0;
    for (TopoDS_Iterator it(raw);it.More();it.Next()) {
        if (++children!=1 || it.Value().ShapeType()!=TopAbs_SOLID) return false;
        solid=it.Value();
    }
    return children==1; // No silently discarded second solid, sheet or wire.
}
}
inline Status Build(const TopoDS_Shape& detachedBase, const Recipe& recipe,
    const std::atomic_bool& stop, Result& output) noexcept {
    output={};
    if (stop.load()) return Status::Cancelled;
    if (!Inspect(recipe)) return Status::InvalidRecipe;
    try {
        OCC_CATCH_SIGNALS
        if (!detail::Bounded(detachedBase,1024,stop)) return stop.load()?Status::Cancelled:Status::BudgetExceeded;
        if (detachedBase.ShapeType()!=TopAbs_SOLID) return Status::UnsupportedSource;
        // Even detached caller input is copied with geometry; OCCT work never
        // edits the supplied source. Copy excludes mesh caches deliberately.
        BRepBuilderAPI_Copy copy(detachedBase,Standard_True,Standard_False);
        if (!copy.IsDone()) return Status::KernelFailure;
        const auto base=copy.Shape();
        Result result;
        const double mm=recipe.metersPerUnit*1000;
        if (!detail::ValidSolid(base,result.sourceVolume)
            || !detail::Bounds(base,mm,result.sourceBounds)) return Status::UnsupportedSource;
        Handle(Message_ProgressIndicator) indicator=new planar_sweep::detail::CancellationProgress(stop);
        Message_ProgressScope progress(indicator->Start(),"Cylindrical through-cut",3);
        if (planar_sweep::detail::CheckInterference(base,stop,progress.Next())!=planar_sweep::BuildStatus::Built)
            return stop.load()?Status::Cancelled:Status::UnsupportedSource;
        const unsigned axis=static_cast<unsigned>(recipe.tool.axis);
        const double tolerance=std::max(Precision::Confusion()*32,1e-5/mm);
        const double margin=std::max(tolerance*4,.001/mm);
        if (!std::isfinite(tolerance) || !std::isfinite(margin) || recipe.tool.radius<=tolerance)
            return Status::InvalidRecipe;
        result.toolStart=result.sourceBounds[axis]-margin;
        result.toolEnd=result.sourceBounds[axis+3]+margin;
        const double length=result.toolEnd-result.toolStart;
        if (!std::isfinite(length) || length<=0 || !std::isfinite(length*mm) || length*mm>2e6+1)
            return Status::InvalidRecipe;
        auto origin=recipe.tool.point;origin[axis]=result.toolStart;
        const gp_Dir direction=axis==0?gp::DX():axis==1?gp::DY():gp::DZ();
        BRepPrimAPI_MakeCylinder cylinder(gp_Ax2(gp_Pnt(origin[0],origin[1],origin[2]),direction),recipe.tool.radius,length);
        cylinder.Build();
        if (!cylinder.IsDone()) return Status::KernelFailure;
        if (stop.load()) return Status::Cancelled;
        TopTools_ListOfShape arguments,tools;arguments.Append(base);tools.Append(cylinder.Shape());
        BRepAlgoAPI_Cut cut;
        cut.SetArguments(arguments);cut.SetTools(tools);
        cut.SetRunParallel(Standard_False);cut.SetNonDestructive(Standard_True);
        cut.SetFuzzyValue(0);cut.SetUseOBB(Standard_True);cut.SetCheckInverted(Standard_True);
        cut.Build(progress.Next());
        if (stop.load()) return Status::Cancelled;
        if (!cut.IsDone() || cut.HasErrors() || cut.HasWarnings() || cut.Shape().IsNull()) return Status::KernelFailure;
        if (!detail::Bounded(cut.Shape(),32768,stop)) return stop.load()?Status::Cancelled:Status::BudgetExceeded;
        if (!detail::SingleResult(cut.Shape(),result.solid)
            || !detail::ValidSolid(result.solid,result.resultVolume)
            || !detail::Bounds(result.solid,mm,result.resultBounds)) return Status::UnsupportedResult;
        if (planar_sweep::detail::CheckInterference(result.solid,stop,progress.Next())!=planar_sweep::BuildStatus::Built)
            return stop.load()?Status::Cancelled:Status::UnsupportedResult;
        result.removedVolume=result.sourceVolume-result.resultVolume;
        // Distinguish real subtraction from a tangent/disjoint/no-volume case.
        // This is an admission precision bound, not a geometric oracle tolerance.
        const double minRemoved=std::max(tolerance*tolerance*tolerance,result.sourceVolume*1e-12);
        if (!std::isfinite(result.removedVolume) || result.removedVolume<=minRemoved) return Status::NoRemovedVolume;
        for (unsigned i=0;i<3;++i)
            if (result.resultBounds[i]<result.sourceBounds[i]-tolerance
                || result.resultBounds[i+3]>result.sourceBounds[i+3]+tolerance) return Status::VerificationFailed;
        if (stop.load()) return Status::Cancelled;
        output=std::move(result);return Status::Built;
    } catch (...) {output={};return stop.load()?Status::Cancelled:Status::KernelFailure;}
}
} // namespace core3d::analytic_boolean
