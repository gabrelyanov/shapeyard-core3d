#pragma once
// Private numeric/native geometry preparation. The document owner separately
// binds definition identity, placement, UV provenance, material and OCAF history.
#include "NativeMeshVertexMove.hxx"
#include "NativeTriangleContacts.hpp"
#include <BRepBuilderAPI_Copy.hxx>
#include <BRep_Builder.hxx>
#include <Poly_Triangulation.hxx>
#include <deque>
#include <limits>

namespace core3d::meshedit {
enum class RegionSideUVPolicy : std::uint8_t { BoundaryStripNormalized = 1 };
struct PlanarRegion {
    std::vector<std::uint32_t> triangles;       // sorted source ordinals
    std::vector<std::uint32_t> boundaryVertices; // directed, canonical start
    Point unitNormal{};
    double planeOffset=0;
    bool IsEqual(const PlanarRegion& other) const noexcept {
        return triangles==other.triangles && boundaryVertices==other.boundaryVertices
            && unitNormal==other.unitNormal && planeOffset==other.planeOffset;
    }
};
inline bool SameRegionStorage(const NativeMeshStorageCapture& a,
                              const NativeMeshStorageCapture& b) noexcept {
    return a.shape.IsEqual(b.shape) && a.face.IsEqual(b.face)
        && a.sourceMesh==b.sourceMesh && a.meshLocation.IsEqual(b.meshLocation)
        && a.storedNodes==b.storedNodes && a.storedUVs==b.storedUVs
        && a.storedNormals==b.storedNormals && a.deflection==b.deflection
        && a.triangleNodeIDs==b.triangleNodeIDs && a.storedTriangles==b.storedTriangles;
}
inline TopologyResult ResolvePlanarRegion(const NativeTopologyCapture& source,
    std::uint32_t seed, PlanarRegion& output, const std::atomic_bool& cancelled) noexcept {
    output={};
    try {
        if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
        const auto& topology=source.topology;
        if(seed>=topology.triangleVertices.size() || topology.triangleVertices.size()>4096
            || topology.unitNormals.size()!=topology.triangleVertices.size()
            || topology.boundaryEdges!=0)return TopologyResult::Invalid;
        double extent=1;
        for(const auto& vertex:topology.vertices)for(double x:vertex.point)extent=std::max(extent,std::abs(x));
        const double planeTolerance=extent*1.e-10, angularTolerance=1.e-10;
        const Point normal=topology.unitNormals[seed];
        const Point anchor=topology.vertices[topology.triangleVertices[seed][0]].point;
        double offset=0;for(int k=0;k<3;++k)offset+=normal[k]*anchor[k];
        auto coplanar=[&](std::uint32_t triangle) {
            double dot=0;for(int k=0;k<3;++k)dot+=normal[k]*topology.unitNormals[triangle][k];
            if(!std::isfinite(dot) || dot<1-angularTolerance)return false;
            for(auto vertex:topology.triangleVertices[triangle]) {
                double plane=-offset;for(int k=0;k<3;++k)plane+=normal[k]*topology.vertices[vertex].point[k];
                if(!std::isfinite(plane) || std::abs(plane)>planeTolerance)return false;
            }
            return true;
        };
        std::vector<std::vector<std::uint32_t>> neighbors(topology.triangleVertices.size());
        for(const auto& edge:topology.edges) {
            if(edge.uses.size()!=2)return TopologyResult::Invalid;
            neighbors[edge.uses[0].triangle].push_back(edge.uses[1].triangle);
            neighbors[edge.uses[1].triangle].push_back(edge.uses[0].triangle);
        }
        std::set<std::uint32_t> selected{seed};std::deque<std::uint32_t> pending{seed};
        while(!pending.empty()) {
            if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
            const auto triangle=pending.front();pending.pop_front();
            for(auto next:neighbors[triangle])if(!selected.count(next)&&coplanar(next)) {
                if(selected.size()==256)return TopologyResult::TooLarge;
                selected.insert(next);pending.push_back(next);
            }
        }
        // Each selected boundary edge must border one retained source triangle.
        std::map<std::uint32_t,std::uint32_t> nextByVertex;
        std::map<std::uint32_t,std::uint32_t> incoming;
        for(const auto& edge:topology.edges) {
            const bool first=selected.count(edge.uses[0].triangle),second=selected.count(edge.uses[1].triangle);
            if(first==second)continue;
            if(edge.uses.size()!=2)return TopologyResult::Invalid;
            const auto triangle=first?edge.uses[0].triangle:edge.uses[1].triangle;
            const auto& ids=topology.triangleVertices[triangle];
            bool found=false;std::uint32_t from=0,to=0;
            for(int k=0;k<3;++k) {
                const auto a=ids[k],b=ids[(k+1)%3];
                if(std::min(a,b)==edge.vertices[0]&&std::max(a,b)==edge.vertices[1]){from=a;to=b;found=true;break;}
            }
            if(!found || from==to || !nextByVertex.emplace(from,to).second
                || !incoming.emplace(to,from).second)return TopologyResult::Invalid;
        }
        if(nextByVertex.size()<3 || nextByVertex.size()!=incoming.size())return TopologyResult::Invalid;
        const auto start=nextByVertex.begin()->first;
        std::vector<std::uint32_t> boundary;boundary.reserve(nextByVertex.size());
        auto current=start;
        do {
            if(boundary.size()==nextByVertex.size())return TopologyResult::Invalid;
            boundary.push_back(current);const auto found=nextByVertex.find(current);
            if(found==nextByVertex.end())return TopologyResult::Invalid;
            current=found->second;
        } while(current!=start);
        if(boundary.size()!=nextByVertex.size())return TopologyResult::Invalid;
        PlanarRegion region;region.triangles.assign(selected.begin(),selected.end());
        region.boundaryVertices=std::move(boundary);region.unitNormal=normal;region.planeOffset=offset;
        output=std::move(region);return TopologyResult::Ready;
    } catch(...) {output={};return TopologyResult::Invalid;}
}
inline TopologyResult PrepareRegionExtrusion(const NativeTopologyCapture& source,
    int sourcePrefix, const PlanarRegion& region, double localDistance,
    RegionSideUVPolicy policy, TopoDS_Shape& candidate,
    const std::atomic_bool& cancelled) noexcept {
    candidate.Nullify();
    try {
        if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
        if(policy!=RegionSideUVPolicy::BoundaryStripNormalized || !std::isfinite(localDistance)
            || localDistance<=0 || localDistance>1.e6 || region.triangles.empty()
            || region.triangles.size()>256 || region.boundaryVertices.size()<3
            || !source.sourceMesh->HasUVNodes() || !HasFlatCornerLayout(source,sourcePrefix))
            return TopologyResult::Invalid;
        NativeMeshStorageCapture fresh;
        auto status=CaptureNativeMeshStorage(source.shape,fresh,cancelled);
        if(status!=TopologyResult::Ready)return status;
        if(!SameRegionStorage(source,fresh))return TopologyResult::Invalid;
        PlanarRegion resolved;
        status=ResolvePlanarRegion(source,region.triangles.front(),resolved,cancelled);
        if(status!=TopologyResult::Ready)return status;
        if(!resolved.IsEqual(region))return TopologyResult::Invalid;
        const std::size_t sourceCount=source.storedTriangles.size();
        const std::size_t sideCount=2*region.boundaryVertices.size();
        if(sourceCount+sideCount>4096 || 3*(sourceCount+sideCount)>24576)return TopologyResult::TooLarge;
        std::vector<bool> selected(sourceCount,false);
        for(auto triangle:region.triangles) {
            if(triangle>=sourceCount || selected[triangle])return TopologyResult::Invalid;
            selected[triangle]=true;
        }
        auto shifted=[&](const Point& p) { Point q=p;for(int k=0;k<3;++k)q[k]+=region.unitNormal[k]*localDistance;return q; };
        std::vector<Triangle> triangles;triangles.reserve(sourceCount+sideCount);
        std::vector<std::array<std::array<double,2>,3>> uvs;uvs.reserve(sourceCount+sideCount);
        std::vector<std::array<std::array<float,3>,3>> normals;normals.reserve(sourceCount+sideCount);
        for(std::size_t t=0;t<sourceCount;++t) {
            Triangle triangle=source.storedTriangles[t];if(selected[t])for(auto& p:triangle)p=shifted(p);
            triangles.push_back(triangle);
            std::array<std::array<double,2>,3> triangleUV{};
            std::array<std::array<float,3>,3> triangleNormals{};
            for(int k=0;k<3;++k) {
                const int node=source.triangleNodeIDs[t][k];
                if(node<=sourcePrefix || node>source.sourceMesh->NbNodes())return TopologyResult::Invalid;
                triangleUV[k]=source.storedUVs[node-1];triangleNormals[k]=source.storedNormals[node-1];
            }
            uvs.push_back(triangleUV);normals.push_back(triangleNormals);
        }
        std::vector<double> lengths(region.boundaryVertices.size()+1,0);
        for(std::size_t i=0;i<region.boundaryVertices.size();++i) {
            const auto a=region.boundaryVertices[i],b=region.boundaryVertices[(i+1)%region.boundaryVertices.size()];
            if(a>=source.topology.vertices.size()||b>=source.topology.vertices.size())return TopologyResult::Invalid;
            double squared=0;for(int k=0;k<3;++k){const double d=source.topology.vertices[b].point[k]-source.topology.vertices[a].point[k];squared+=d*d;}
            if(!std::isfinite(squared)||squared<=0)return TopologyResult::Invalid;
            lengths[i+1]=lengths[i]+std::sqrt(squared);
        }
        const double perimeter=lengths.back();if(!std::isfinite(perimeter)||perimeter<=0)return TopologyResult::Invalid;
        for(std::size_t i=0;i<region.boundaryVertices.size();++i) {
            const Point a=source.topology.vertices[region.boundaryVertices[i]].point;
            const Point b=source.topology.vertices[region.boundaryVertices[(i+1)%region.boundaryVertices.size()]].point;
            const Point aa=shifted(a),bb=shifted(b);
            const double u0=lengths[i]/perimeter,u1=lengths[i+1]/perimeter;
            const std::array<Triangle,2> sides={Triangle{a,b,bb},Triangle{a,bb,aa}};
            const std::array<std::array<double,2>,3> firstUV={{{u0,0},{u1,0},{u1,1}}};
            const std::array<std::array<double,2>,3> secondUV={{{u0,0},{u1,1},{u0,1}}};
            const std::array<std::array<std::array<double,2>,3>,2> sideUVs={firstUV,secondUV};
            for(int s=0;s<2;++s) {
                Topology one;status=Analyze({sides[s]},one,cancelled);if(status!=TopologyResult::Ready)return status;
                const auto& n=one.unitNormals.front();const std::array<float,3> packed={float(n[0]),float(n[1]),float(n[2])};
                triangles.push_back(sides[s]);uvs.push_back(sideUVs[s]);normals.push_back({packed,packed,packed});
            }
        }
        Topology topology;status=Analyze(triangles,topology,cancelled);
        if(status!=TopologyResult::Ready || topology.boundaryEdges!=0)return status==TopologyResult::Ready?TopologyResult::Invalid:status;
        meshcheck::ContactReport contacts;
        const std::vector<meshcheck::Triangle> contactTriangles(triangles.begin(),triangles.end());
        const auto contact=meshcheck::AnalyzeTriangleContacts(contactTriangles,contacts,cancelled,
            {4096,2000000,1,std::chrono::milliseconds(100)});
        if(contact!=meshcheck::ContactStatus::Ready || !contacts.unexpectedPairs.empty())return TopologyResult::Invalid;
        Handle(Poly_Triangulation) resultMesh=new Poly_Triangulation(int(3*triangles.size()),int(triangles.size()),true,true);
        resultMesh->Deflection(source.deflection);
        for(std::size_t t=0;t<triangles.size();++t)for(int k=0;k<3;++k) {
            const int node=int(3*t)+k+1;const auto& p=triangles[t][k];
            resultMesh->SetNode(node,gp_Pnt(p[0],p[1],p[2]));
            resultMesh->SetUVNode(node,gp_Pnt2d(uvs[t][k][0],uvs[t][k][1]));
            resultMesh->SetNormal(node,gp_Vec3f(normals[t][k][0],normals[t][k][1],normals[t][k][2]));
        }
        for(std::size_t t=0;t<triangles.size();++t)resultMesh->SetTriangle(int(t)+1,Poly_Triangle(int(3*t)+1,int(3*t)+2,int(3*t)+3));
        BRepBuilderAPI_Copy copy(source.shape,Standard_True,Standard_True);auto result=copy.Shape();
        NativeMeshStorageCapture copied;status=CaptureNativeMeshStorage(result,copied,cancelled);
        if(status!=TopologyResult::Ready || result.IsSame(source.shape)||copied.face.IsSame(source.face)
            || copied.sourceMesh==source.sourceMesh)return TopologyResult::Invalid;
        BRep_Builder().UpdateFace(copied.face,resultMesh);
        NativeTopologyCapture verified;status=CaptureNativeTopology(result,verified,cancelled);
        if(status!=TopologyResult::Ready || verified.storedTriangles!=triangles
            || verified.storedUVs.size()!=3*triangles.size() || verified.storedNormals.size()!=3*triangles.size()
            || verified.triangleNodeIDs.size()!=triangles.size() || !HasFlatCornerLayout(verified,0)
            || !verified.meshLocation.IsEqual(source.meshLocation) || verified.face.Orientation()!=source.face.Orientation())
            return TopologyResult::Invalid;
        for(std::size_t t=0;t<triangles.size();++t)for(int k=0;k<3;++k) {
            const std::size_t node=3*t+k;
            if(verified.storedUVs[node]!=uvs[t][k] || verified.storedNormals[node]!=normals[t][k]
                || verified.triangleNodeIDs[t][k]!=int(node)+1)return TopologyResult::Invalid;
        }
        NativeMeshStorageCapture untouched;status=CaptureNativeMeshStorage(source.shape,untouched,cancelled);
        if(status!=TopologyResult::Ready || !SameRegionStorage(source,untouched))return TopologyResult::Invalid;
        candidate=result;return TopologyResult::Ready;
    } catch(const std::bad_alloc&){candidate.Nullify();return TopologyResult::TooLarge;}
      catch(...){candidate.Nullify();return TopologyResult::Invalid;}
}
} // namespace core3d::meshedit
