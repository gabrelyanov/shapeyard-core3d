#pragma once
#include "RectangularLoftDefinition.hxx"
// Deliberate dependency on the frozen detached sweep verification component:
// reuse only its cancellation progress and actual OCCT self-interference checker.
// This loft never calls the sweep builder or substitutes a swept shape.
#include "PlanarSweepSolid.hxx"
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS_Shell.hxx>
#include <gp_Pln.hxx>

namespace core3d::rectangular_loft {
using BuildStatus=planar_sweep::BuildStatus;
struct SolidResult {TopoDS_Shape solid;std::array<double,6> bounds{};double volume=0;};
namespace detail {
// Verification is numeric/shape-only, not document authority. Production Build
// passes only its original immutable inspection. DEBUG probes can challenge the
// independent checks with an intentionally wrong expected numeric measurement.
inline BuildStatus Verify(const TopoDS_Shape& candidate,const Inspection& checked,
    const std::atomic_bool& cancelled,SolidResult& output,
    const Message_ProgressRange& range=Message_ProgressRange()) {
    output={};
    if (cancelled.load()) return BuildStatus::Cancelled;
    if (!std::isfinite(checked.expectedVolume) || checked.expectedVolume<=0
        || !std::isfinite(checked.millimetersPerUnit) || checked.millimetersPerUnit<=0)
        return BuildStatus::VerificationFailed;
    for(double x:checked.expectedBounds) if(!std::isfinite(x))return BuildStatus::VerificationFailed;
    if (candidate.IsNull() || candidate.ShapeType()!=TopAbs_SOLID) return BuildStatus::InvalidSolid;
        auto solid=TopoDS::Solid(candidate);
        TopTools_IndexedMapOfShape shells,faces,edges;
        TopExp::MapShapes(solid,TopAbs_SHELL,shells);TopExp::MapShapes(solid,TopAbs_FACE,faces);
        TopExp::MapShapes(solid,TopAbs_EDGE,edges);
        if (shells.Extent()!=1 || faces.Extent()<6 || faces.Extent()>30 || edges.Extent()>64
            || !BRepLib::OrientClosedSolid(solid) || !BRepCheck_Analyzer(solid,Standard_True).IsValid())
            return BuildStatus::InvalidSolid;
        const auto interference=planar_sweep::detail::CheckInterference(solid,cancelled,range);
        if (interference!=BuildStatus::Built) return interference;
        GProp_GProps properties;BRepGProp::VolumeProperties(solid,properties);
        const double volume=properties.Mass(),expected=checked.expectedVolume;
        if (!std::isfinite(volume) || volume<=0 || std::abs(volume-expected)>expected*1e-7)
            return BuildStatus::VerificationFailed;
        Bnd_Box bounds;BRepBndLib::AddOptimal(solid,bounds,Standard_False,Standard_False);
        if (bounds.IsVoid() || bounds.IsOpen()) return BuildStatus::VerificationFailed;
        SolidResult result;bounds.Get(result.bounds[0],result.bounds[1],result.bounds[2],
            result.bounds[3],result.bounds[4],result.bounds[5]);
        for (std::size_t i=0;i<6;++i)
            if (!std::isfinite(result.bounds[i]) || std::abs(result.bounds[i]-checked.expectedBounds[i])
                >0.001/checked.millimetersPerUnit) return BuildStatus::VerificationFailed;
        result.solid=solid;result.volume=volume;
        if (cancelled.load()) return BuildStatus::Cancelled;
        output=std::move(result);return BuildStatus::Built;
}
} // namespace detail
inline BuildStatus Build(const std::shared_ptr<const Prepared>& prepared,
    const std::atomic_bool& cancelled,SolidResult& output) noexcept {
    output={};
    if (cancelled.load()) return BuildStatus::Cancelled;
    if (!prepared) return BuildStatus::InvalidDefinition;
    try {
        OCC_CATCH_SIGNALS
        const auto& d=prepared->definition;const auto& checked=prepared->inspection;
        Handle(Message_ProgressIndicator) indicator=new planar_sweep::detail::CancellationProgress(cancelled);
        Message_ProgressScope progress(indicator->Start(),"Ruled rectangular loft",2);
        const auto count=d.stations.size();
        Message_ProgressScope construction(progress.Next(),"Planar station faces",double(2*count+1));
        // Each authored corner and station/longitudinal edge has one TShape.
        // Reversed uses close adjacent faces without sewing or merging coplanar
        // faces: even a constant rectangular section retains every station edge.
        std::vector<std::array<gp_Pnt,4>> corners(count);
        std::vector<std::array<TopoDS_Vertex,4>> vertices(count);
        std::vector<std::array<TopoDS_Edge,4>> rings(count),rails(count-1);
        for (std::size_t station=0;station<count;++station) {
            if (cancelled.load()) return BuildStatus::Cancelled;
            corners[station]=detail::Corners(d.stations[station]);
            for (std::size_t i=0;i<4;++i) {
                BRepBuilderAPI_MakeVertex maker(corners[station][i]);
                if (!maker.IsDone()) return BuildStatus::KernelFailure;
                vertices[station][i]=maker.Vertex();
            }
            for (std::size_t i=0;i<4;++i) {
                BRepBuilderAPI_MakeEdge edge(vertices[station][i],vertices[station][(i+1)%4]);
                if (!edge.IsDone()) return BuildStatus::KernelFailure;
                rings[station][i]=edge.Edge();
                if (station) {
                    const gp_Dir initial(gp_Vec(corners[station-1][i],corners[station][i]));
                    const gp_Dir stable(initial.XYZ());
                    BRepBuilderAPI_MakeEdge rail(gp_Lin(corners[station-1][i],stable),
                        vertices[station-1][i],vertices[station][i]);
                    if (!rail.IsDone()) return BuildStatus::KernelFailure;
                    rails[station-1][i]=rail.Edge();
                }
            }
            construction.Next();
        }
        BRep_Builder builder;TopoDS_Shell shell;builder.MakeShell(shell);
        const auto reversed=[](const TopoDS_Edge& edge) {return TopoDS::Edge(edge.Reversed());};
        const auto addFace=[&](const std::array<TopoDS_Edge,4>& edges,const gp_Pln& plane,bool reverse=false) {
            BRepBuilderAPI_MakeWire wire;
            for (const auto& edge:edges) {
                wire.Add(edge);
                if (!wire.IsDone()) return false;
            }
            if (!wire.Wire().Closed()) return false;
            // Supplying the analytic plane avoids inferred spline supports and
            // gives each face its own affine planar parameterization.
            // Winding is authored above; preserve the explicit orientation.
            BRepBuilderAPI_MakeFace maker(plane,wire.Wire(),Standard_False);
            if (!maker.IsDone()) return false;
            auto face=maker.Face();if(reverse)face.Reverse();
            builder.Add(shell,face);return true;
        };
        for (std::size_t station=0;station+1<count;++station) {
            for (std::size_t i=0;i<4;++i) {
                if (cancelled.load()) return BuildStatus::Cancelled;
                const auto j=(i+1)%4;
                // Parallel ring edges and increasing Z guarantee a nonzero
                // normal, including rectangles and equal-length parallelograms.
                const gp_Vec along(corners[station][i],corners[station][j]);
                const gp_Vec rise(corners[station][i],corners[station+1][i]);
                const gp_Pln plane(corners[station][i],gp_Dir(gp_Dir(along.Crossed(rise)).XYZ()));
                if (!addFace({rings[station][i],rails[station][j],
                              reversed(rings[station+1][i]),reversed(rails[station][i])},plane))
                    return BuildStatus::KernelFailure;
            }
            construction.Next();
        }
        for (const auto station:{std::size_t(0),count-1}) {
            if (cancelled.load()) return BuildStatus::Cancelled;
            if (!addFace(rings[station],gp_Pln(corners[station][0],gp::DZ()),station==0))
                return BuildStatus::KernelFailure;
            construction.Next();
        }
        if (!BRep_Tool::IsClosed(shell)) return BuildStatus::InvalidSolid;
        shell.Closed(Standard_True);
        TopoDS_Solid solid;builder.MakeSolid(solid);builder.Add(solid,shell);
        TopoDS_Shape candidate=solid;
        if (d.constructionFrame) {
            gp_Trsf transform;
            if (!d.constructionFrame->Transform(transform)) return BuildStatus::InvalidDefinition;
            BRepBuilderAPI_Transform placed(candidate,transform,Standard_True,Standard_False);
            if (!placed.IsDone()) return BuildStatus::KernelFailure;
            candidate=placed.Shape();
        }
        if (cancelled.load()) return BuildStatus::Cancelled;
        if (candidate.IsNull() || candidate.ShapeType()!=TopAbs_SOLID) return BuildStatus::InvalidSolid;
        return detail::Verify(candidate,checked,cancelled,output,progress.Next());
    } catch (...) {output={};return cancelled.load() ? BuildStatus::Cancelled : BuildStatus::KernelFailure;}
}
} // namespace core3d::rectangular_loft
