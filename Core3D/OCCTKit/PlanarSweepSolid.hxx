#pragma once

// Detached OCCT geometry only: no OCAF labels, AIS objects, commands or receipts.
#include "PlanarSweepDefinition.hxx"
#include "ProfileDefinition.hxx"
#include <BOPAlgo_ArgumentAnalyzer.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Circle.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Message_ProgressScope.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <Message_ProgressRange.hxx>
#include <gp.hxx>
#include <atomic>

namespace core3d::planar_sweep {
enum class BuildStatus { Built, InvalidDefinition, Cancelled, KernelFailure,
    InvalidSolid, SelfInterference, VerificationFailed };
struct SolidResult {
    TopoDS_Shape solid;
    std::array<double,6> bounds{}; // Native document units, including placement.
    double volume=0;
    double length=0; // Native document units, including absolute placement scale.
};
namespace detail {
class CancellationProgress final : public Message_ProgressIndicator {
    DEFINE_STANDARD_RTTI_INLINE(CancellationProgress,Message_ProgressIndicator)
public:
    explicit CancellationProgress(const std::atomic_bool& cancelled):cancelled_(cancelled) {}
protected:
    Standard_Boolean UserBreak() override { return cancelled_.load(std::memory_order_relaxed); }
    void Show(const Message_ProgressScope&,const Standard_Boolean) override {}
private:
    const std::atomic_bool& cancelled_; // Synchronous Build owns every progress range.
};
inline BuildStatus CheckInterference(const TopoDS_Shape& shape,const std::atomic_bool& cancelled,
    const Message_ProgressRange& range=Message_ProgressRange()) {
    if (cancelled.load()) return BuildStatus::Cancelled;
    if (shape.IsNull()) return BuildStatus::InvalidSolid;
    BOPAlgo_ArgumentAnalyzer interference;
    interference.SetShape1(shape);interference.SetRunParallel(Standard_False);
    interference.StopOnFirstFaulty()=Standard_True;
    interference.ArgumentTypeMode()=Standard_False;interference.SelfInterMode()=Standard_True;
    interference.SmallEdgeMode()=Standard_False;interference.RebuildFaceMode()=Standard_False;
    interference.TangentMode()=Standard_False;interference.MergeVertexMode()=Standard_False;
    interference.MergeEdgeMode()=Standard_False;interference.ContinuityMode()=Standard_False;
    interference.CurveOnSurfaceMode()=Standard_False;
    interference.Perform(range);
    if (cancelled.load()) return BuildStatus::Cancelled;
    if (interference.HasErrors() || interference.HasWarnings() || interference.HasFaulty())
        return BuildStatus::SelfInterference;
    return BuildStatus::Built;
}
}

inline BuildStatus Build(const std::shared_ptr<const Prepared>& prepared,
    const std::atomic_bool& cancelled,SolidResult& output) noexcept {
    output={};
    if (cancelled.load()) return BuildStatus::Cancelled;
    if (!prepared) return BuildStatus::InvalidDefinition;
    try {
        OCC_CATCH_SIGNALS
        const auto& input=prepared->definition;
        const auto& checked=prepared->inspection;
        const double mm=checked.millimetersPerUnit;
        const double scale=input.constructionFrame ? std::abs(input.constructionFrame->values[7]) : 1;
        if (input.radius*scale<32*Precision::Confusion()) return BuildStatus::InvalidDefinition;
        const gp_Dir normal=input.plane==0 ? gp::DZ() : input.plane==1 ? gp_Dir(0,-1,0) : gp::DX();
        const gp_Dir uDirection=input.plane==2 ? gp::DY() : gp::DX();
        std::vector<TopoDS_Vertex> vertices;vertices.reserve(input.vertices.size());
        for (const auto& vertex:input.vertices) {
            if (cancelled.load()) return BuildStatus::Cancelled;
            BRepBuilderAPI_MakeVertex maker(ProfilePointInPlane(vertex.point,input.plane));
            if (!maker.IsDone()) return BuildStatus::KernelFailure;
            vertices.push_back(maker.Vertex());
        }
        BRepBuilderAPI_MakeWire wire;
        const double radians=std::acos(-1.0)/180;
        for (std::size_t i=0;i<input.segments.size();++i) {
            if (cancelled.load()) return BuildStatus::Cancelled;
            const auto& segment=input.segments[i];TopoDS_Edge edge;
            if (segment.kind==ProfileCurveKind::Line) {
                BRepBuilderAPI_MakeEdge maker(vertices[i],vertices[i+1]);
                if (!maker.IsDone()) return BuildStatus::KernelFailure;
                edge=maker.Edge();
            } else {
                Handle(Geom_Circle) circle=new Geom_Circle(gp_Circ(gp_Ax2(
                    ProfilePointInPlane(segment.center,input.plane),normal,uDirection),segment.radius));
                const double start=segment.startDegrees*radians,end=(segment.startDegrees+segment.sweepDegrees)*radians;
                BRepBuilderAPI_MakeEdge maker(circle,
                    segment.sweepDegrees>0 ? vertices[i] : vertices[i+1],
                    segment.sweepDegrees>0 ? vertices[i+1] : vertices[i],std::min(start,end),std::max(start,end));
                if (!maker.IsDone()) return BuildStatus::KernelFailure;
                edge=segment.sweepDegrees>0 ? maker.Edge() : TopoDS::Edge(maker.Edge().Reversed());
            }
            if (edge.IsNull()) return BuildStatus::KernelFailure;
            wire.Add(edge);
            if (!wire.IsDone()) return BuildStatus::KernelFailure;
        }
        const auto spine=wire.Wire();
        if (spine.IsNull() || !BRepCheck_Analyzer(spine,Standard_True).IsValid()) return BuildStatus::InvalidSolid;
        const auto tangent=checked.firstTangent;
        const gp_Dir sectionNormal=input.plane==0 ? gp_Dir(tangent[0],tangent[1],0)
            : input.plane==1 ? gp_Dir(tangent[0],0,tangent[1]) : gp_Dir(0,tangent[0],tangent[1]);
        // U is the work-plane normal; V is sectionNormal cross U. Preserve this
        // deterministic authored basis rather than deriving a random circle seam.
        const gp_Ax2 sectionBasis(ProfilePointInPlane(input.vertices.front().point,input.plane),sectionNormal,normal);
        BRepBuilderAPI_MakeEdge circle(gp_Circ(sectionBasis,input.radius));
        if (!circle.IsDone()) return BuildStatus::KernelFailure;
        BRepBuilderAPI_MakeWire section(circle.Edge());
        if (!section.IsDone()) return BuildStatus::KernelFailure;
        Handle(Message_ProgressIndicator) indicator=new detail::CancellationProgress(cancelled);
        Message_ProgressScope progress(indicator->Start(),"Planar circle sweep",2);
        BRepOffsetAPI_MakePipeShell pipe(spine);
        pipe.SetMode(Standard_False); // Explicit corrected Frenet, no implicit corner repair.
        pipe.SetForceApproxC1(Standard_False);
        pipe.SetTransitionMode(BRepBuilderAPI_Transformed); // G1 admission excludes fractures.
        const double tolerance=std::max(Precision::Confusion(),1e-5/mm);
        pipe.SetTolerance(tolerance,tolerance,detail::tangentTolerance);
        pipe.SetMaxDegree(12);pipe.SetMaxSegments(64);
        pipe.Add(section.Wire(),vertices.front(),Standard_False,Standard_False);
        if (!pipe.IsReady()) return BuildStatus::KernelFailure;
        pipe.Build(progress.Next());
        if (cancelled.load()) return BuildStatus::Cancelled;
        if (!pipe.IsDone() || pipe.GetStatus()!=BRepBuilderAPI_PipeDone
            || !std::isfinite(pipe.ErrorOnSurface()) || pipe.ErrorOnSurface()>4*tolerance
            || !pipe.MakeSolid()) return BuildStatus::KernelFailure;
        TopoDS_Shape candidate=pipe.Shape();
        if (candidate.IsNull() || candidate.ShapeType()!=TopAbs_SOLID) return BuildStatus::InvalidSolid;
        if (input.constructionFrame) {
            gp_Trsf transform;
            if (!input.constructionFrame->Transform(transform)) return BuildStatus::InvalidDefinition;
            BRepBuilderAPI_Transform placed(candidate,transform,Standard_True,Standard_False);
            if (!placed.IsDone()) return BuildStatus::KernelFailure;
            candidate=placed.Shape();
        }
        if (cancelled.load()) return BuildStatus::Cancelled;
        if (candidate.IsNull() || candidate.ShapeType()!=TopAbs_SOLID) return BuildStatus::InvalidSolid;
        auto solid=TopoDS::Solid(candidate);
        TopTools_IndexedMapOfShape shells,faces,edges;
        TopExp::MapShapes(solid,TopAbs_SHELL,shells);TopExp::MapShapes(solid,TopAbs_FACE,faces);
        TopExp::MapShapes(solid,TopAbs_EDGE,edges);
        if (shells.Extent()!=1 || faces.Extent()<3 || faces.Extent()>258 || edges.Extent()>1024
            || !BRepLib::OrientClosedSolid(solid) || !BRepCheck_Analyzer(solid,Standard_True).IsValid())
            return BuildStatus::InvalidSolid;
        const auto interference=detail::CheckInterference(solid,cancelled,progress.Next());
        if (interference!=BuildStatus::Built) return interference;
        GProp_GProps properties;BRepGProp::VolumeProperties(solid,properties);
        const double volume=properties.Mass(),expected=checked.expectedVolume;
        if (!std::isfinite(volume) || volume<=0 || std::abs(volume-expected)>expected*1e-7)
            return BuildStatus::VerificationFailed;
        Bnd_Box box;BRepBndLib::AddOptimal(solid,box,Standard_False,Standard_False);
        if (box.IsVoid() || box.IsOpen()) return BuildStatus::VerificationFailed;
        SolidResult result;box.Get(result.bounds[0],result.bounds[1],result.bounds[2],
                                  result.bounds[3],result.bounds[4],result.bounds[5]);
        for (double value:result.bounds)
            if (!std::isfinite(value) || std::abs(value)>detail::maximumNative
                || !std::isfinite(value*mm) || std::abs(value*mm)>detail::maximumPhysical)
                return BuildStatus::VerificationFailed;
        result.solid=solid;result.volume=volume;result.length=checked.length*scale;
        if (cancelled.load()) return BuildStatus::Cancelled;
        output=std::move(result);return BuildStatus::Built;
    } catch (...) { output={};return cancelled.load() ? BuildStatus::Cancelled : BuildStatus::KernelFailure; }
}
} // namespace core3d::planar_sweep
