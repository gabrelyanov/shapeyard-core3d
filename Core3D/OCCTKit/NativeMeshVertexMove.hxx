#pragma once
// Private geometry preparation; the owned controller admits publication.
// Caller must validate native document ownership, atlas provenance and material
// frame policy. In particular a supplied tangent archive may NOT be reused.
#include "NativeMeshTopologyCapture.hxx"
#include "NativeMeshRegionPartition.hxx"
#include "NativeTriangleContacts.hpp"
#include <chrono>
#include <new>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRep_Builder.hxx>

namespace core3d::meshedit {
inline bool HasFlatCornerLayout(const NativeTopologyCapture& source,int originalPrefixCount) noexcept {
    try {
        const auto& mesh=source.sourceMesh;
        if(mesh.IsNull() || !mesh->HasNormals() || mesh->NbTriangles()<=0
            || source.triangleNodeIDs.size()!=std::size_t(mesh->NbTriangles())
            || source.topology.unitNormals.size()!=std::size_t(mesh->NbTriangles()))return false;
        const int corners=3*mesh->NbTriangles();
        if((originalPrefixCount!=0 && originalPrefixCount!=corners)
            || mesh->NbNodes()!=originalPrefixCount+corners
            || (originalPrefixCount!=0 && !mesh->HasUVNodes()))return false;
        // Validate the exact triangle-major corner pattern and flat shading
        // before creating a candidate. Atlas prefix positions must match their
        // corresponding active corners; coordinate coincidence alone does not
        // establish a unique normal association after deformation.
        for(int t=0;t<mesh->NbTriangles();++t)for(int k=0;k<3;++k) {
            const int id=originalPrefixCount+3*t+k+1;
            if(source.triangleNodeIDs[t][k]!=id)return false;
            gp_Vec3f normal;mesh->Normal(id,normal);
            for(int axis=0;axis<3;++axis)
                if(std::abs(double(normal[axis])-source.topology.unitNormals[t][axis])>2.e-6)
                    return false;
            if(originalPrefixCount!=0) {
                const int original=3*t+k+1;
                if(!mesh->Node(original).IsEqual(mesh->Node(id),0.0))return false;
                gp_Vec3f prefixNormal;mesh->Normal(original,prefixNormal);
                if(prefixNormal!=normal)return false;
            }
        }
        return true;
    } catch(...) {return false;}
}

// First admission is independent flat corners in triangle order, either with
// no prefix or with the exact original triangle-corner prefix preserved by UV
// generation. This excludes arbitrary shared smooth normals; it never silently
// turns an imported smooth mesh into a flat-shaded one.
inline TopologyResult PrepareFlatVertexMove(const NativeTopologyCapture& source,
    const std::vector<std::uint32_t>& selectedVertices, const Point& delta,
    int originalPrefixCount, TopoDS_Shape& candidate,
    const std::atomic_bool& cancelled) noexcept {
    candidate.Nullify();
    try {
        if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
        const auto& mesh=source.sourceMesh;
        if(mesh.IsNull() || source.shape.IsNull() || selectedVertices.empty()
            || selectedVertices.size()>64 || source.topology.vertices.empty()
            || source.storedNodes.size()!=std::size_t(mesh->NbNodes())
            || source.triangleNodeIDs.size()!=std::size_t(mesh->NbTriangles())
            || source.vertexNodeIDs.size()!=source.topology.vertices.size()
            || !mesh->HasNormals())return TopologyResult::Invalid;
        NativeTopologyCapture fresh;
        const auto freshStatus=CaptureNativeTopology(source.shape,fresh,cancelled);
        if(freshStatus!=TopologyResult::Ready)return freshStatus;
        if(fresh.sourceMesh!=mesh || !fresh.face.IsEqual(source.face)
            || !fresh.meshLocation.IsEqual(source.meshLocation)
            || fresh.storedNodes!=source.storedNodes
            || fresh.storedUVs!=source.storedUVs
            || fresh.storedNormals!=source.storedNormals
            || fresh.deflection!=source.deflection
            || fresh.triangleNodeIDs!=source.triangleNodeIDs
            || fresh.vertexNodeIDs!=source.vertexNodeIDs
            || fresh.topology.triangleVertices!=source.topology.triangleVertices
            || fresh.topology.unitNormals!=source.topology.unitNormals
            || fresh.topology.vertices.size()!=source.topology.vertices.size()
            || fresh.topology.boundaryEdges!=source.topology.boundaryEdges)
            return TopologyResult::Invalid;
        const int corners=3*mesh->NbTriangles();
        if((originalPrefixCount!=0 && originalPrefixCount!=corners)
            || mesh->NbNodes()!=originalPrefixCount+corners
            || (originalPrefixCount!=0 && !mesh->HasUVNodes()))return TopologyResult::Invalid;
        bool moves=false;
        for(double value:delta) {
            if(!std::isfinite(value) || std::abs(value)>1.e6)return TopologyResult::Invalid;
            moves|=value!=0;
        }
        if(!moves)return TopologyResult::Invalid;
        std::set<std::uint32_t> unique;
        for(auto vertex:selectedVertices)
            if(vertex>=source.topology.vertices.size() || !unique.insert(vertex).second)
                return TopologyResult::Invalid;
        if(!HasFlatCornerLayout(source,originalPrefixCount))return TopologyResult::Invalid;
        Handle(Poly_Triangulation) moved=mesh->Copy();
        if(moved.IsNull() || moved==mesh)return TopologyResult::Invalid;
        for(auto vertex:selectedVertices)for(int node:source.vertexNodeIDs[vertex]) {
            const auto& before=source.storedNodes[node-1];
            gp_Pnt point(before[0]+delta[0],before[1]+delta[1],before[2]+delta[2]);
            for(int axis=1;axis<=3;++axis)
                if(!std::isfinite(point.Coord(axis)) || std::abs(point.Coord(axis))>1.e6)
                    return TopologyResult::Invalid;
            moved->SetNode(node,point);
        }
        std::vector<Triangle> triangles;triangles.reserve(mesh->NbTriangles());
        for(const auto& ids:source.triangleNodeIDs) {
            Triangle triangle;
            for(int k=0;k<3;++k) {
                const auto point=moved->Node(ids[k]);triangle[k]={point.X(),point.Y(),point.Z()};
            }
            triangles.push_back(triangle);
        }
        Topology topology;const auto status=Analyze(triangles,topology,cancelled);
        if(status!=TopologyResult::Ready)return status;
        if(topology.triangleVertices!=source.topology.triangleVertices
            || topology.boundaryEdges!=source.topology.boundaryEdges)
            return TopologyResult::Invalid;
        for(std::size_t t=0;t<triangles.size();++t) {
            if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
            const auto& normal=topology.unitNormals[t];double dot=0;
            for(int axis=0;axis<3;++axis)dot+=normal[axis]*source.topology.unitNormals[t][axis];
            // Bounded first edit excludes locally collapsed/flipped faces. It
            // does not certify absence of distant triangle intersections.
            if(dot<=1.e-6)return TopologyResult::Invalid;
            const gp_Vec3f packed{float(normal[0]),float(normal[1]),float(normal[2])};
            for(int k=0;k<3;++k) {
                moved->SetNormal(source.triangleNodeIDs[t][k],packed);
                if(originalPrefixCount!=0)moved->SetNormal(int(3*t)+k+1,packed);
            }
        }
        // Never mutate the original TShape or triangulation. Replace only the
        // independently copied mesh-only face; outer/child placements survive.
        BRepBuilderAPI_Copy copy(source.shape,Standard_True,Standard_True);
        auto result=copy.Shape();NativeTopologyCapture copied;
        auto copyStatus=CaptureNativeTopology(result,copied,cancelled);
        if(copyStatus!=TopologyResult::Ready)return copyStatus;
        if(result.IsSame(source.shape) || copied.face.IsSame(source.face)
            || copied.sourceMesh==mesh)return TopologyResult::Invalid;
        BRep_Builder().UpdateFace(copied.face,moved);
        NativeTopologyCapture verified;
        const auto verifiedStatus=CaptureNativeTopology(result,verified,cancelled);
        if(verifiedStatus!=TopologyResult::Ready)return verifiedStatus;
        if(verified.triangleNodeIDs!=source.triangleNodeIDs
            || verified.topology.triangleVertices!=source.topology.triangleVertices
            || !verified.meshLocation.IsEqual(source.meshLocation)
            || verified.face.Orientation()!=source.face.Orientation())return TopologyResult::Invalid;
        // UV values, including unused prefix values, are copied byte-for-value.
        if(mesh->HasUVNodes())for(int node=1;node<=mesh->NbNodes();++node)
            if(!mesh->UVNode(node).IsEqual(verified.sourceMesh->UVNode(node),0.0))return TopologyResult::Invalid;
        if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
        candidate=result;return TopologyResult::Ready;
    } catch(...) {candidate.Nullify();return TopologyResult::Invalid;}
}

struct PartitionedVertexMoveCandidate {
    TopoDS_Shape shape;
    RegionPartition partition;
};

inline bool SameVertexMoveEdgeLineage(const NativeTopologyCapture& before,
    const NativeTopologyCapture& after) noexcept {
    if(before.triangleNodeIDs!=after.triangleNodeIDs
        || before.vertexNodeIDs!=after.vertexNodeIDs
        || before.topology.triangleVertices!=after.topology.triangleVertices
        || before.topology.boundaryEdges!=after.topology.boundaryEdges
        || before.topology.vertices.size()!=after.topology.vertices.size()
        || before.topology.edges.size()!=after.topology.edges.size())return false;
    for(std::size_t vertex=0;vertex<before.topology.vertices.size();++vertex)
        if(before.topology.vertices[vertex].corners!=after.topology.vertices[vertex].corners)
            return false;
    for(std::size_t edge=0;edge<before.topology.edges.size();++edge) {
        const auto& a=before.topology.edges[edge];const auto& b=after.topology.edges[edge];
        if(a.vertices!=b.vertices || a.uses.size()!=b.uses.size())return false;
        for(std::size_t use=0;use<a.uses.size();++use)
            if(a.uses[use].triangle!=b.uses[use].triangle
                || a.uses[use].forward!=b.uses[use].forward)return false;
    }
    return true;
}

inline bool SameUntouchedVertexMoveSource(const NativeTopologyCapture& before,
    const NativeTopologyCapture& after) noexcept {
    if(!before.shape.IsEqual(after.shape) || !before.face.IsEqual(after.face)
        || before.sourceMesh!=after.sourceMesh
        || !before.meshLocation.IsEqual(after.meshLocation)
        || before.storedNodes!=after.storedNodes
        || before.storedUVs!=after.storedUVs
        || before.storedNormals!=after.storedNormals
        || before.deflection!=after.deflection
        || before.storedTriangles!=after.storedTriangles
        || before.topology.unitNormals!=after.topology.unitNormals
        || !SameVertexMoveEdgeLineage(before,after))return false;
    for(std::size_t vertex=0;vertex<before.topology.vertices.size();++vertex)
        if(before.topology.vertices[vertex].point!=after.topology.vertices[vertex].point)
            return false;
    return true;
}

// Partition-bearing coordinate edit. The existing shape-only entry remains the
// compatibility path for sources with no durable partition authority.
inline TopologyResult PreparePartitionedFlatVertexMove(
    const NativeTopologyCapture& source,
    const std::vector<std::uint32_t>& selectedVertices,
    const Point& delta,
    int originalPrefixCount,
    const RegionPartition& sourcePartition,
    PartitionedVertexMoveCandidate& candidate,
    const std::atomic_bool& cancelled) noexcept {
    candidate={};
    try {
        if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
        if(!ValidateRegionPartition(source,sourcePartition))return TopologyResult::Invalid;
        TopoDS_Shape moved;
        const auto prepared=PrepareFlatVertexMove(source,selectedVertices,delta,
            originalPrefixCount,moved,cancelled);
        if(prepared!=TopologyResult::Ready)return prepared;
        NativeTopologyCapture after;
        auto status=CaptureNativeTopology(moved,after,cancelled);
        if(status!=TopologyResult::Ready)return status;
        if(!SameVertexMoveEdgeLineage(source,after)
            || !after.meshLocation.IsEqual(source.meshLocation)
            || after.face.Orientation()!=source.face.Orientation())return TopologyResult::Invalid;
        meshcheck::ContactReport contacts;
        const std::vector<meshcheck::Triangle> triangles(
            after.storedTriangles.begin(),after.storedTriangles.end());
        const auto contact=meshcheck::AnalyzeTriangleContacts(triangles,contacts,cancelled,
            {4096,2000000,1,std::chrono::milliseconds(100)});
        if(contact!=meshcheck::ContactStatus::Ready || !contacts.unexpectedPairs.empty()) {
            if(contact==meshcheck::ContactStatus::Cancelled)return TopologyResult::Cancelled;
            if(contact==meshcheck::ContactStatus::TooLarge)return TopologyResult::TooLarge;
            return TopologyResult::Invalid;
        }
        RegionPartition rebound=sourcePartition;
        if(!OrderedTriangleGeometryDigest(after,rebound.geometry)
            || !ValidateRegionPartition(after,rebound))return TopologyResult::Invalid;
        NativeTopologyCapture untouched;
        status=CaptureNativeTopology(source.shape,untouched,cancelled);
        if(status!=TopologyResult::Ready)return status;
        if(!SameUntouchedVertexMoveSource(source,untouched))return TopologyResult::Invalid;
        PartitionedVertexMoveCandidate value;
        value.shape=moved;value.partition=std::move(rebound);
        candidate=std::move(value);return TopologyResult::Ready;
    } catch(const std::bad_alloc&) {candidate={};return TopologyResult::TooLarge;}
      catch(...) {candidate={};return TopologyResult::Invalid;}
}
} // namespace core3d::meshedit
