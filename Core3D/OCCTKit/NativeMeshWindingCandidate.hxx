#pragma once
// Private geometry preparer. The document owner separately validates unit,
// exact identity, atlas provenance, material recipe, generated normal binding,
// memory budget and ordinary-edit recovery. Authored/supplied frames are excluded.
#include "NativeMeshWindingPlan.hpp"
#include "NativeMeshVertexMove.hxx"

namespace core3d::meshedit {
inline bool SameMeshStorage(const NativeMeshStorageCapture& a,
                            const NativeMeshStorageCapture& b) noexcept {
    return a.shape.IsEqual(b.shape) && a.face.IsEqual(b.face)
        && a.sourceMesh==b.sourceMesh && a.meshLocation.IsEqual(b.meshLocation)
        && a.storedNodes==b.storedNodes && a.storedUVs==b.storedUVs
        && a.storedNormals==b.storedNormals && a.deflection==b.deflection
        && a.triangleNodeIDs==b.triangleNodeIDs && a.storedTriangles==b.storedTriangles;
}
// This preparer requires at least one reversal. A trusted owner reports an
// empty PlanConsistentWinding result as NoChange before calling it; no history.
inline TopologyResult PrepareFlatWindingRepair(const NativeMeshStorageCapture& source,
    int originalPrefixCount, TopoDS_Shape& candidate, WindingPlan& proposed,
    const std::atomic_bool& cancelled) noexcept {
    candidate.Nullify();proposed={};
    try {
        if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
        NativeMeshStorageCapture fresh;
        auto status=CaptureNativeMeshStorage(source.shape,fresh,cancelled);
        if(status!=TopologyResult::Ready)return status;
        if(!SameMeshStorage(source,fresh))return TopologyResult::Invalid;
        const auto& mesh=source.sourceMesh;
        if(mesh.IsNull() || source.storedTriangles.size()!=std::size_t(mesh->NbTriangles()))
            return TopologyResult::Invalid;
        // Reuse the existing exact flat-corner contract. Per-triangle analysis
        // checks geometry/normals without claiming cross-triangle orientability.
        NativeTopologyCapture flat;
        static_cast<NativeMeshStorageCapture&>(flat)=source;
        for(const auto& triangle:source.storedTriangles) {
            Topology isolated;
            status=Analyze({triangle},isolated,cancelled);
            if(status!=TopologyResult::Ready)return status;
            flat.topology.unitNormals.push_back(isolated.unitNormals.front());
        }
        if(!HasFlatCornerLayout(flat,originalPrefixCount))return TopologyResult::Invalid;
        WindingPlan plan;
        status=PlanConsistentWinding(source.storedTriangles,plan,cancelled);
        if(status!=TopologyResult::Ready)return status;
        if(plan.reversedTriangles.empty())return TopologyResult::Invalid;
        std::vector<bool> reversed(mesh->NbTriangles(),false);
        auto expectedTriangles=source.storedTriangles;
        for(auto t:plan.reversedTriangles) {
            if(t>=reversed.size() || reversed[t])return TopologyResult::Invalid;
            reversed[t]=true;std::swap(expectedTriangles[t][1],expectedTriangles[t][2]);
        }
        Topology corrected;
        status=Analyze(expectedTriangles,corrected,cancelled);
        if(status!=TopologyResult::Ready)return status;
        Handle(Poly_Triangulation) repaired=mesh->Copy();
        if(repaired.IsNull() || repaired==mesh)return TopologyResult::Invalid;
        auto expectedNodes=source.storedNodes;
        auto expectedUVs=source.storedUVs;
        auto expectedNormals=source.storedNormals;
        for(int t=0;t<mesh->NbTriangles();++t) {
            if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
            if(!reversed[t])continue;
            const auto& n=corrected.unitNormals[t];
            const gp_Vec3f normal{float(n[0]),float(n[1]),float(n[2])};
            // Keep triangle-major native indices and atlas prefix provenance.
            // Swap the exact positions AND corresponding UVs, never weld seams.
            for(int k=0;k<3;++k) {
                const int from=originalPrefixCount+3*t+(k==0?0:3-k)+1;
                const int to=originalPrefixCount+3*t+k+1;
                repaired->SetNode(to,mesh->Node(from));repaired->SetNormal(to,normal);
                expectedNodes[to-1]=source.storedNodes[from-1];
                expectedNormals[to-1]={normal[0],normal[1],normal[2]};
                if(mesh->HasUVNodes()) {
                    repaired->SetUVNode(to,mesh->UVNode(from));
                    expectedUVs[to-1]=source.storedUVs[from-1];
                }
                if(originalPrefixCount!=0) {
                    const int prefixFrom=from-originalPrefixCount,prefixTo=to-originalPrefixCount;
                    repaired->SetNode(prefixTo,mesh->Node(prefixFrom));
                    repaired->SetNormal(prefixTo,normal);
                    repaired->SetUVNode(prefixTo,mesh->UVNode(prefixFrom));
                    expectedNodes[prefixTo-1]=source.storedNodes[prefixFrom-1];
                    expectedNormals[prefixTo-1]={normal[0],normal[1],normal[2]};
                    expectedUVs[prefixTo-1]=source.storedUVs[prefixFrom-1];
                }
            }
        }
        BRepBuilderAPI_Copy copy(source.shape,Standard_True,Standard_True);
        auto result=copy.Shape();NativeMeshStorageCapture copied;
        status=CaptureNativeMeshStorage(result,copied,cancelled);
        if(status!=TopologyResult::Ready)return status;
        if(result.IsSame(source.shape) || copied.face.IsSame(source.face)
            || copied.sourceMesh==mesh)return TopologyResult::Invalid;
        BRep_Builder().UpdateFace(copied.face,repaired);
        NativeTopologyCapture verified;
        status=CaptureNativeTopology(result,verified,cancelled);
        if(status!=TopologyResult::Ready)return status;
        if(result.ShapeType()!=source.shape.ShapeType()
            || result.Orientation()!=source.shape.Orientation()
            || !result.Location().IsEqual(source.shape.Location())
            || verified.face.Orientation()!=source.face.Orientation()
            || !verified.meshLocation.IsEqual(source.meshLocation)
            || verified.triangleNodeIDs!=source.triangleNodeIDs
            || verified.storedTriangles!=expectedTriangles
            || verified.storedNodes!=expectedNodes || verified.storedUVs!=expectedUVs
            || verified.storedNormals!=expectedNormals || verified.deflection!=source.deflection
            || !HasFlatCornerLayout(verified,originalPrefixCount))return TopologyResult::Invalid;
        NativeMeshStorageCapture untouched;
        status=CaptureNativeMeshStorage(source.shape,untouched,cancelled);
        if(status!=TopologyResult::Ready)return status;
        if(!SameMeshStorage(source,untouched))return TopologyResult::Invalid;
        if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
        candidate=result;proposed=std::move(plan);return TopologyResult::Ready;
    } catch(...) {candidate.Nullify();proposed={};return TopologyResult::Invalid;}
}
} // namespace core3d::meshedit
