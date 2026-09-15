#pragma once
#if DEBUG
#include "NativeMeshRegionInsetProbe.hxx"
#include "NativeMeshVertexMove.hxx"

namespace core3d::debug {
struct MeshPartitionVertexMoveProbeResult {
    std::array<bool,8> checks{};
    double movedVolume=0,extrudedVolume=0;
    std::uint32_t centerSeed=0;
};

inline bool MakeMarker2Prefix(const meshedit::NativeTopologyCapture& source,
                              TopoDS_Shape& output) {
    using namespace meshedit;
    output.Nullify();
    try {
        const auto& old=source.sourceMesh;
        if(old.IsNull() || !old->HasUVNodes() || !old->HasNormals()
            || !HasFlatCornerLayout(source,0))return false;
        const int triangles=old->NbTriangles(),prefix=3*triangles;
        Handle(Poly_Triangulation) mesh=new Poly_Triangulation(
            2*prefix,triangles,true,true);
        mesh->Deflection(old->Deflection());
        for(int triangle=0;triangle<triangles;++triangle) {
            int ids[3];old->Triangle(triangle+1).Get(ids[0],ids[1],ids[2]);
            for(int corner=0;corner<3;++corner) {
                const int oldID=ids[corner],prefixID=3*triangle+corner+1;
                const int activeID=prefix+prefixID;
                gp_Vec3f normal;old->Normal(oldID,normal);
                for(const int id:{prefixID,activeID}) {
                    mesh->SetNode(id,old->Node(oldID));mesh->SetNormal(id,normal);
                }
                mesh->SetUVNode(prefixID,gp_Pnt2d(0,0));
                mesh->SetUVNode(activeID,old->UVNode(oldID));
            }
            mesh->SetTriangle(triangle+1,Poly_Triangle(
                prefix+3*triangle+1,prefix+3*triangle+2,prefix+3*triangle+3));
        }
        BRepBuilderAPI_Copy copy(source.shape,Standard_True,Standard_True);
        TopoDS_Shape result=copy.Shape();NativeMeshStorageCapture copied;
        std::atomic_bool cancelled{false};
        if(CaptureNativeMeshStorage(result,copied,cancelled)!=TopologyResult::Ready
            || result.IsSame(source.shape) || copied.face.IsSame(source.face))return false;
        BRep_Builder().UpdateFace(copied.face,mesh);
        NativeTopologyCapture verified;
        if(CaptureNativeTopology(result,verified,cancelled)!=TopologyResult::Ready
            || !HasFlatCornerLayout(verified,prefix))return false;
        output=result;return true;
    } catch(...) {output.Nullify();return false;}
}

inline std::vector<std::uint32_t> ExactVertexIDs(
    const meshedit::NativeTopologyCapture& source,
    const std::vector<meshedit::Point>& points) {
    std::vector<std::uint32_t> result;
    for(const auto& point:points)for(std::size_t vertex=0;
        vertex<source.topology.vertices.size();++vertex)
        if(source.topology.vertices[vertex].point==point) {
            result.push_back(std::uint32_t(vertex));break;
        }
    return result;
}

inline MeshPartitionVertexMoveProbeResult RunMeshPartitionVertexMoveKernelProbe() {
    using namespace meshedit;
    MeshPartitionVertexMoveProbeResult result;
    std::atomic_bool cancelled{false};
    try {
        const auto box=MakePrism({{0,0},{80,0},{80,60},{0,60}},100);
        NativeTopologyCapture source;
        if(CaptureNativeTopology(box,source,cancelled)!=TopologyResult::Ready)
            return result;
        PlanarRegion face;
        if(ResolvePlanarRegion(source,2,face,cancelled)!=TopologyResult::Ready)
            return result;
        std::vector<Point> outerBoundary;
        for(const auto vertex:face.boundaryVertices)
            outerBoundary.push_back(source.topology.vertices[vertex].point);
        RegionInsetCandidate inset;
        if(PrepareConvexRegionInset(source,0,face,10,nullptr,inset,cancelled)
            !=TopologyResult::Ready || inset.centerSeed!=10)return result;
        NativeTopologyCapture authored;
        if(CaptureNativeTopology(inset.shape,authored,cancelled)!=TopologyResult::Ready)
            return result;
        const auto selected=ExactVertexIDs(authored,inset.innerBoundary);
        PartitionedVertexMoveCandidate moved;
        result.checks[0]=selected.size()==4
            && PreparePartitionedFlatVertexMove(authored,selected,{5,0,0},0,
                inset.partition,moved,cancelled)==TopologyResult::Ready;
        NativeTopologyCapture movedSource;PlanarRegion center;
        if(result.checks[0]
            && CaptureNativeTopology(moved.shape,movedSource,cancelled)==TopologyResult::Ready) {
            result.movedVolume=SignedVolume(movedSource.storedTriangles);
            result.centerSeed=inset.centerSeed;
            const bool centerReady=moved.partition.barriers==inset.partition.barriers
                && moved.partition.geometry!=inset.partition.geometry
                && ValidateRegionPartition(movedSource,moved.partition)
                && ResolvePlanarRegion(movedSource,inset.centerSeed,center,cancelled,
                    &moved.partition)==TopologyResult::Ready
                && center.triangles==std::vector<std::uint32_t>({10,11})
                && std::abs(result.movedVolume-497666.6666666667)<0.01;
            TopoDS_Shape extruded;RegionPartition afterExtrusion;
            NativeTopologyCapture extrusion;
            bool extrusionReady=false;
            if(centerReady
                && PrepareRegionExtrusion(movedSource,0,center,6,
                    RegionSideUVPolicy::BoundaryStripNormalized,extruded,cancelled,
                    &moved.partition,&afterExtrusion)==TopologyResult::Ready
                && CaptureNativeTopology(extruded,extrusion,cancelled)==TopologyResult::Ready) {
                result.extrudedVolume=SignedVolume(extrusion.storedTriangles);
                Point extrudedMinimum={INFINITY,INFINITY,INFINITY};
                Point extrudedMaximum={-INFINITY,-INFINITY,-INFINITY};
                for(const auto& vertex:extrusion.topology.vertices)
                    for(int axis=0;axis<3;++axis) {
                        extrudedMinimum[axis]=std::min(extrudedMinimum[axis],vertex.point[axis]);
                        extrudedMaximum[axis]=std::max(extrudedMaximum[axis],vertex.point[axis]);
                    }
                // The former partition boundary is now two noncoplanar creases;
                // canonical valid partition state is absence, as in Inset's probe.
                extrusionReady=afterExtrusion.empty()
                    && extrudedMinimum==Point{0,0,0}
                    && extrudedMaximum==Point{111,80,60}
                    && std::abs(result.extrudedVolume-512066.6666666667)<0.01;
            }
            result.checks[1]=centerReady && extrusionReady;
            bool allCenterMoved=true,outerUnchanged=true;
            Point minimum={INFINITY,INFINITY,INFINITY};
            Point maximum={-INFINITY,-INFINITY,-INFINITY};
            for(const auto& vertex:movedSource.topology.vertices)for(int axis=0;axis<3;++axis) {
                minimum[axis]=std::min(minimum[axis],vertex.point[axis]);
                maximum[axis]=std::max(maximum[axis],vertex.point[axis]);
            }
            for(const auto& point:inset.innerBoundary) {
                auto expected=point;expected[0]+=5;
                allCenterMoved=allCenterMoved && std::any_of(
                    movedSource.topology.vertices.begin(),movedSource.topology.vertices.end(),
                    [&](const auto& vertex){return vertex.point==expected;});
            }
            for(const auto& expected:outerBoundary)
                outerUnchanged=outerUnchanged && std::any_of(
                    movedSource.topology.vertices.begin(),movedSource.topology.vertices.end(),
                    [&](const auto& vertex){return vertex.point==expected;});
            result.checks[2]=allCenterMoved && outerUnchanged
                && minimum==Point{0,0,0} && maximum==Point{105,80,60};
        }
        TopoDS_Shape marker2Shape;NativeTopologyCapture marker2;
        result.checks[3]=MakeMarker2Prefix(authored,marker2Shape)
            && CaptureNativeTopology(marker2Shape,marker2,cancelled)==TopologyResult::Ready
            && ValidateRegionPartition(marker2,inset.partition);
        if(result.checks[3]) {
            const auto markerSelection=ExactVertexIDs(marker2,inset.innerBoundary);
            PartitionedVertexMoveCandidate markerMoved;
            const int prefix=3*marker2.sourceMesh->NbTriangles();
            result.checks[3]=markerSelection.size()==4
                && PreparePartitionedFlatVertexMove(marker2,markerSelection,{5,0,0},prefix,
                    inset.partition,markerMoved,cancelled)==TopologyResult::Ready;
            NativeTopologyCapture verified;
            result.checks[3]=result.checks[3]
                && CaptureNativeTopology(markerMoved.shape,verified,cancelled)==TopologyResult::Ready
                && HasFlatCornerLayout(verified,prefix)
                && verified.storedUVs==marker2.storedUVs
                && markerMoved.partition.barriers==inset.partition.barriers
                && ValidateRegionPartition(verified,markerMoved.partition);
        }
        RegionPartition malformed=inset.partition;
        for(auto& mask:malformed.barriers)if(mask) {mask^=std::uint8_t(mask&-mask);break;}
        PartitionedVertexMoveCandidate refused=moved;
        result.checks[4]=PreparePartitionedFlatVertexMove(authored,selected,{5,0,0},0,
            malformed,refused,cancelled)==TopologyResult::Invalid
            && refused.shape.IsNull() && refused.partition.empty();
        auto changedIncidence=movedSource;
        if(changedIncidence.topology.triangleVertices.size()>10)
            std::swap(changedIncidence.topology.triangleVertices[10][0],
                changedIncidence.topology.triangleVertices[10][1]);
        auto forgedSource=authored;
        std::swap(forgedSource.topology.triangleVertices[10][0],
            forgedSource.topology.triangleVertices[10][1]);
        refused=moved;
        result.checks[5]=result.checks[0]
            && SameVertexMoveEdgeLineage(authored,movedSource)
            && !SameVertexMoveEdgeLineage(authored,changedIncidence)
            && PreparePartitionedFlatVertexMove(forgedSource,selected,{5,0,0},0,
                inset.partition,refused,cancelled)==TopologyResult::Invalid
            && refused.shape.IsNull() && refused.partition.empty();
        TopoDS_Shape crossing;NativeTopologyCapture crossingSource;
        meshcheck::ContactReport crossingContacts;
        refused=moved;
        const auto crossingLegacy=PrepareFlatVertexMove(authored,selected,{-150,0,0},0,
            crossing,cancelled);
        const auto crossingCapture=crossingLegacy==TopologyResult::Ready
            ? CaptureNativeTopology(crossing,crossingSource,cancelled)
            : crossingLegacy;
        const std::vector<meshcheck::Triangle> crossingTriangles(
            crossingSource.storedTriangles.begin(),crossingSource.storedTriangles.end());
        const auto crossingContact=crossingCapture==TopologyResult::Ready
            && SameVertexMoveEdgeLineage(authored,crossingSource)
            ? meshcheck::AnalyzeTriangleContacts(crossingTriangles,crossingContacts,cancelled,
                {4096,2000000,2048,std::chrono::milliseconds(100)})
            : meshcheck::ContactStatus::Invalid;
        RegionPartition crossingRebound=inset.partition;
        const bool crossingReboundValid=crossingCapture==TopologyResult::Ready
            && OrderedTriangleGeometryDigest(crossingSource,crossingRebound.geometry)
            && ValidateRegionPartition(crossingSource,crossingRebound);
        const auto crossingTyped=PreparePartitionedFlatVertexMove(
            authored,selected,{-150,0,0},0,inset.partition,refused,cancelled);
        result.checks[6]=crossingLegacy==TopologyResult::Ready
            && crossingCapture==TopologyResult::Ready
            && SameVertexMoveEdgeLineage(authored,crossingSource)
            && crossingContact==meshcheck::ContactStatus::Ready
            && !crossingContacts.unexpectedPairs.empty()
            && crossingReboundValid
            && (crossingTyped==TopologyResult::Invalid
                || crossingTyped==TopologyResult::TooLarge)
            && refused.shape.IsNull() && refused.partition.empty();
        std::atomic_bool stopped{true};refused=moved;
        result.checks[7]=PreparePartitionedFlatVertexMove(authored,selected,{5,0,0},0,
            inset.partition,refused,stopped)==TopologyResult::Cancelled
            && refused.shape.IsNull() && refused.partition.empty();
    } catch(...) {}
    return result;
}
} // namespace core3d::debug
#endif
