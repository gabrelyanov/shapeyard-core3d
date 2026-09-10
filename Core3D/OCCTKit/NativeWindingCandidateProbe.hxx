#pragma once
// Isolated native geometry probe; OCAF authority is checked by the owner.
#if DEBUG
#include "NativeMeshWindingCandidate.hxx"
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
namespace core3d::debug {
inline TopoDS_Shape MakeWindingProbeFixture(bool atlas,bool inconsistent=true,bool located=false) {
    const std::vector<meshedit::Point> points={{0,0,0},{10,0,0},{0,10,0},{0,0,10}};
    std::vector<std::array<int,3>> triangles={{0,2,1},{0,1,3},{1,2,3},{2,0,3}};
    if(inconsistent)std::swap(triangles[3][1],triangles[3][2]);
    const int prefix=atlas?12:0;
    Handle(Poly_Triangulation) mesh=new Poly_Triangulation(prefix+12,4,atlas,Standard_True);
    for(int t=0;t<4;++t) {
        std::array<gp_Pnt,3> p;
        for(int k=0;k<3;++k) {
            const auto& v=points[triangles[t][k]];p[k]=gp_Pnt(v[0],v[1],v[2]);
        }
        const gp_Dir normal(gp_Vec(p[0],p[1]).Crossed(gp_Vec(p[0],p[2])));
        for(int k=0;k<3;++k) {
            const int id=prefix+3*t+k+1;
            mesh->SetNode(id,p[k]);mesh->SetNormal(id,normal);
            if(atlas) {
                // Deliberately distinct active and prefix UVs; exact corner
                // association must survive without flattening either table.
                mesh->SetUVNode(id,gp_Pnt2d(0.1*t+0.02*k,0.1+0.03*k));
                mesh->SetNode(id-prefix,p[k]);mesh->SetNormal(id-prefix,normal);
                mesh->SetUVNode(id-prefix,gp_Pnt2d(0.7+0.01*t,0.8+0.02*k));
            }
        }
        mesh->SetTriangle(t+1,Poly_Triangle(prefix+3*t+1,prefix+3*t+2,prefix+3*t+3));
    }
    mesh->UpdateCachedMinMax();
    TopoDS_Face face;BRep_Builder().MakeFace(face,mesh);
    if(located) {
        gp_Trsf placement;placement.SetTranslation(gp_Vec(3,4,5));
        face.Location(TopLoc_Location(placement));face.Reverse();
    }
    return face;
}
inline std::array<bool,10> RunNativeWindingCandidateProbe() {
    using namespace meshedit;
    std::array<bool,10> result{};
    std::atomic_bool cancelled{false};
    try {
        for(int mode=0;mode<3;++mode) {
            const bool atlas=mode!=0;
            const auto shape=MakeWindingProbeFixture(atlas,true,mode==2);
            NativeMeshStorageCapture original;
            if(CaptureNativeMeshStorage(shape,original,cancelled)!=TopologyResult::Ready)continue;
            NativeTopologyCapture invalid;
            if(CaptureNativeTopology(shape,invalid,cancelled)!=TopologyResult::NonManifold)continue;
            TopoDS_Shape candidate;WindingPlan plan;
            if(PrepareFlatWindingRepair(original,atlas?12:0,candidate,plan,cancelled)!=TopologyResult::Ready)continue;
            NativeTopologyCapture edited;
            if(CaptureNativeTopology(candidate,edited,cancelled)!=TopologyResult::Ready)continue;
            NativeMeshStorageCapture after;
            bool preserved=CaptureNativeMeshStorage(shape,after,cancelled)==TopologyResult::Ready
                && SameMeshStorage(original,after) && !candidate.IsSame(shape)
                && plan.reversedTriangles==std::vector<std::uint32_t>{3}
                && plan.connectedComponents==1 && edited.topology.boundaryEdges==0
                && edited.topology.vertices.size()==4 && edited.sourceMesh!=original.sourceMesh
                && edited.triangleNodeIDs==original.triangleNodeIDs
                && edited.face.Orientation()==original.face.Orientation()
                && edited.meshLocation.IsEqual(original.meshLocation);
            const int prefix=atlas?12:0;
            for(int node=0;node<prefix+12;++node) {
                int expected=node;
                if(node%12==10)++expected;
                else if(node%12==11)--expected;
                preserved &= edited.storedNodes[node]==original.storedNodes[expected];
                if(atlas)preserved &= edited.storedUVs[node]==original.storedUVs[expected];
                if(node%12>=9)preserved &= edited.storedNormals[node]==std::array<float,3>{-1,0,0};
                else preserved &= edited.storedNormals[node]==original.storedNormals[node];
            }
            result[mode]=preserved;
        }
        for(int mode=0;mode<4;++mode) {
            const auto shape=MakeWindingProbeFixture(true);
            NativeMeshStorageCapture source;
            if(CaptureNativeMeshStorage(shape,source,cancelled)!=TopologyResult::Ready)continue;
            if(mode==0)source.sourceMesh->SetUVNode(13,gp_Pnt2d(0.33,0.44));
            if(mode==1)source.sourceMesh->SetNormal(13,gp_Vec3f{1,0,0});
            if(mode>=2) {
                // Fresh capture must still reject smooth/mismatched flat storage.
                source.sourceMesh->SetNormal(mode==2?13:1,gp_Vec3f{1,0,0});
                if(CaptureNativeMeshStorage(shape,source,cancelled)!=TopologyResult::Ready)continue;
            }
            TopoDS_Shape candidate;WindingPlan plan;
            result[3+mode]=PrepareFlatWindingRepair(source,12,candidate,plan,cancelled)==TopologyResult::Invalid
                && candidate.IsNull() && plan.reversedTriangles.empty();
        }
        {
            const auto shape=MakeWindingProbeFixture(false);
            NativeMeshStorageCapture source;
            if(CaptureNativeMeshStorage(shape,source,cancelled)==TopologyResult::Ready) {
                TopoDS_Shape candidate=shape;WindingPlan plan;plan.reversedTriangles={99};
                cancelled.store(true);
                result[7]=PrepareFlatWindingRepair(source,0,candidate,plan,cancelled)==TopologyResult::Cancelled
                    && candidate.IsNull() && plan.reversedTriangles.empty();
                cancelled.store(false);
            }
        }
        {
            const auto shape=MakeWindingProbeFixture(false,false);
            NativeMeshStorageCapture source;
            if(CaptureNativeMeshStorage(shape,source,cancelled)==TopologyResult::Ready) {
                WindingPlan plan;
                result[8]=PlanConsistentWinding(source.storedTriangles,plan,cancelled)==TopologyResult::Ready
                    && plan.reversedTriangles.empty() && plan.connectedComponents==1;
                // Owner must return NoChange here; preparer is never a no-op commit.
            }
        }
        {
            const auto shape=MakeWindingProbeFixture(false);
            NativeMeshStorageCapture source;
            if(CaptureNativeMeshStorage(shape,source,cancelled)==TopologyResult::Ready) {
                source.sourceMesh->SetTriangle(1,Poly_Triangle(1,2,99));
                NativeMeshStorageCapture invalid;
                result[9]=CaptureNativeMeshStorage(shape,invalid,cancelled)==TopologyResult::Invalid
                    && invalid.sourceMesh.IsNull() && invalid.storedNodes.empty();
            }
        }
    } catch(...) {}
    return result;
}
} // namespace core3d::debug
#endif
