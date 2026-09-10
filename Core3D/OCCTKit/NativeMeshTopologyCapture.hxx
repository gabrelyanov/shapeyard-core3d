#pragma once
// Private main-thread native capture only.
// Caller owns document/entity/geometry/selection authority and revalidation.
// This does not grant permission to edit a source, change metadata, or publish.
#include "EditableMeshTopology.hpp"
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp_Explorer.hxx>
#include <gp_Vec3f.hxx>

namespace core3d::meshedit {
struct NativeMeshStorageCapture {
    // Exact ownership references remain internal and on the native owner thread.
    TopoDS_Shape shape;
    TopoDS_Face face;
    Handle(Poly_Triangulation) sourceMesh;
    TopLoc_Location meshLocation;
    std::vector<Point> storedNodes;
    std::vector<std::array<double,2>> storedUVs;
    std::vector<std::array<float,3>> storedNormals;
    double deflection=0;
    // Native one-based node IDs indexed by zero-based triangle/corner ordinals.
    std::vector<std::array<int,3>> triangleNodeIDs;
    std::vector<Triangle> storedTriangles;
};
inline TopologyResult CaptureNativeMeshStorage(const TopoDS_Shape& source,
    NativeMeshStorageCapture& output, const std::atomic_bool& cancelled) noexcept {
    output={};
    try {
        if (cancelled.load(std::memory_order_relaxed)) return TopologyResult::Cancelled;
        if (source.IsNull()) return TopologyResult::Invalid;
        if (source.ShapeType()!=TopAbs_FACE) {
            if (source.ShapeType()!=TopAbs_COMPOUND) return TopologyResult::Invalid;
            TopoDS_Iterator it(source);
            if (!it.More() || it.Value().ShapeType()!=TopAbs_FACE) return TopologyResult::Invalid;
            it.Next();if(it.More())return TopologyResult::Invalid;
        }
        NativeMeshStorageCapture value;value.shape=source;
        int faces=0;
        for(TopExp_Explorer it(source,TopAbs_FACE);it.More();it.Next()) {
            if(++faces>1)return TopologyResult::Invalid;
            value.face=TopoDS::Face(it.Current());
        }
        if(faces!=1 || !BRep_Tool::Surface(value.face).IsNull()
            || (value.face.Orientation()!=TopAbs_FORWARD && value.face.Orientation()!=TopAbs_REVERSED))
            return TopologyResult::Invalid;
        // A mesh-only face must not conceal unrelated semantic edges/vertices.
        if(TopExp_Explorer(value.face,TopAbs_EDGE).More()
            || TopExp_Explorer(value.face,TopAbs_VERTEX).More())return TopologyResult::Invalid;
        value.sourceMesh=BRep_Tool::Triangulation(value.face,value.meshLocation);
        const auto& mesh=value.sourceMesh;
        if(mesh.IsNull() || mesh->HasDeferredData() || !mesh->HasGeometry()
            || mesh->NbNodes()<=0 || mesh->NbTriangles()<=0)return TopologyResult::Invalid;
        if(mesh->NbNodes()>24576 || mesh->NbTriangles()>4096)return TopologyResult::TooLarge;
        const auto& placement=value.meshLocation.Transformation();
        for(int row=1;row<=3;++row)for(int col=1;col<=4;++col)
            if(!std::isfinite(placement.Value(row,col)))return TopologyResult::Invalid;
        if(!std::isfinite(mesh->Deflection()) || mesh->Deflection()<0)return TopologyResult::Invalid;
        value.deflection=mesh->Deflection();
        value.storedNodes.reserve(mesh->NbNodes());
        if(mesh->HasUVNodes())value.storedUVs.reserve(mesh->NbNodes());
        if(mesh->HasNormals())value.storedNormals.reserve(mesh->NbNodes());
        for(int node=1;node<=mesh->NbNodes();++node) {
            if((node&255)==0 && cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
            const auto point=mesh->Node(node),placed=point.Transformed(placement);
            for(int axis=1;axis<=3;++axis)
                if(!std::isfinite(point.Coord(axis)) || std::abs(point.Coord(axis))>1.e6
                    || !std::isfinite(placed.Coord(axis)) || std::abs(placed.Coord(axis))>1.e6)
                    return TopologyResult::Invalid;
            value.storedNodes.push_back({point.X(),point.Y(),point.Z()});
            if(mesh->HasUVNodes()) {
                const auto uv=mesh->UVNode(node);
                if(!std::isfinite(uv.X()) || !std::isfinite(uv.Y()))return TopologyResult::Invalid;
                value.storedUVs.push_back({uv.X(),uv.Y()});
            }
            if(mesh->HasNormals()) {
                gp_Vec3f normal;mesh->Normal(node,normal);
                double norm=0;
                for(int axis=0;axis<3;++axis) {
                    const double component=normal[axis];
                    if(!std::isfinite(component))return TopologyResult::Invalid;
                    norm+=component*component;
                }
                if(norm<=1.e-24)return TopologyResult::Invalid;
                value.storedNormals.push_back({normal[0],normal[1],normal[2]});
            }
        }
        std::vector<Triangle> triangles;triangles.reserve(mesh->NbTriangles());
        value.triangleNodeIDs.reserve(mesh->NbTriangles());
        for(int t=1;t<=mesh->NbTriangles();++t) {
            if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
            std::array<int,3> ids;mesh->Triangle(t).Get(ids[0],ids[1],ids[2]);
            Triangle triangle;
            for(int k=0;k<3;++k) {
                if(ids[k]<1 || ids[k]>mesh->NbNodes())return TopologyResult::Invalid;
                triangle[k]=value.storedNodes[ids[k]-1];
            }
            value.triangleNodeIDs.push_back(ids);triangles.push_back(triangle);
        }
        value.storedTriangles=std::move(triangles);
        if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
        output=std::move(value);return TopologyResult::Ready;
    } catch(...) {output={};return TopologyResult::Invalid;}
}
// Existing picker callers still receive strict oriented topology. Raw storage
// capture grants neither topology validity nor document mutation authority.
struct NativeTopologyCapture : NativeMeshStorageCapture {
    std::vector<std::vector<int>> vertexNodeIDs;
    Topology topology;
};
inline TopologyResult CaptureNativeTopology(const TopoDS_Shape& source,
    NativeTopologyCapture& output,const std::atomic_bool& cancelled) noexcept {
    output={};
    try {
        NativeTopologyCapture value;
        auto& storage=static_cast<NativeMeshStorageCapture&>(value);
        const auto captured=CaptureNativeMeshStorage(source,storage,cancelled);
        if(captured!=TopologyResult::Ready)return captured;
        const auto result=Analyze(value.storedTriangles,value.topology,cancelled);
        if(result!=TopologyResult::Ready)return result;
        std::map<Point,std::size_t> vertexIDs;
        for(std::size_t v=0;v<value.topology.vertices.size();++v)
            vertexIDs.emplace(value.topology.vertices[v].point,v);
        value.vertexNodeIDs.resize(value.topology.vertices.size());
        for(std::size_t node=0;node<value.storedNodes.size();++node) {
            const auto found=vertexIDs.find(value.storedNodes[node]);
            if(found!=vertexIDs.end())value.vertexNodeIDs[found->second].push_back(int(node)+1);
        }
        if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
        output=std::move(value);return TopologyResult::Ready;
    } catch(...) {output={};return TopologyResult::Invalid;}
}
} // namespace core3d::meshedit
