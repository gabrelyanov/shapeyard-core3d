#pragma once
// Private owner-thread capture. The caller supplies document/entity authority.
// No OCCT handle or source reference escapes in ContactMeshCoordinates.
#include "NativeTriangleContacts.hpp"
#include <new>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>

namespace core3d::meshcheck {
struct ContactMeshCoordinates {
    // Native stored document-local coordinates, in original triangle ordinal order.
    // One nonsingular occurrence placement preserves self-contact incidence;
    // do not round transformed coordinates before the exact predicate.
    std::vector<Triangle> triangles;
    std::size_t storedNodeCount=0;
    // Owner validation retains original topology and every stored node. Workers
    // receive only triangles, never these authority records or native handles.
    std::array<double,12> facePlacement{};
    int faceOrientation=0;
    std::vector<Point> storedNodes;
    std::vector<std::array<std::uint32_t,3>> triangleNodeIDs;
};
enum class ContactCaptureStatus {Ready,Unsupported,Invalid,TooLarge,Cancelled,TimedOut};
inline ContactCaptureStatus CaptureContactMeshCoordinates(const TopoDS_Shape& source,
    ContactMeshCoordinates& output,const std::atomic_bool& cancelled,
    std::chrono::milliseconds duration=std::chrono::milliseconds(100)) noexcept {
    output={};
    if(duration.count()<=0 || duration.count()>100)return ContactCaptureStatus::Invalid;
    const auto deadline=std::chrono::steady_clock::now()+duration;
    auto checkpoint=[&] {
        if(cancelled.load(std::memory_order_relaxed))throw ContactCaptureStatus::Cancelled;
        if(std::chrono::steady_clock::now()>=deadline)throw ContactCaptureStatus::TimedOut;
    };
    try {
        checkpoint();
        if(source.IsNull())return ContactCaptureStatus::Invalid;
        TopoDS_Face face;
        if(source.ShapeType()==TopAbs_FACE)face=TopoDS::Face(source);
        else {
            if(source.ShapeType()!=TopAbs_COMPOUND)return ContactCaptureStatus::Unsupported;
            TopoDS_Iterator it(source);
            if(!it.More() || it.Value().ShapeType()!=TopAbs_FACE)return ContactCaptureStatus::Unsupported;
            face=TopoDS::Face(it.Value());it.Next();
            if(it.More())return ContactCaptureStatus::Unsupported;
        }
        checkpoint();
        if(!BRep_Tool::Surface(face).IsNull()
            || (face.Orientation()!=TopAbs_FORWARD && face.Orientation()!=TopAbs_REVERSED)
            || TopExp_Explorer(face,TopAbs_EDGE).More()
            || TopExp_Explorer(face,TopAbs_VERTEX).More())return ContactCaptureStatus::Unsupported;
        TopLoc_Location location;
        const auto mesh=BRep_Tool::Triangulation(face,location);
        if(mesh.IsNull() || mesh->HasDeferredData() || !mesh->HasGeometry())return ContactCaptureStatus::Unsupported;
        if(mesh->NbNodes()<=0 || mesh->NbTriangles()<=0)return ContactCaptureStatus::Invalid;
        // Separate diagnostic admission: never widens mutation/picker limits.
        if(mesh->NbNodes()>60000 || mesh->NbTriangles()>20000)return ContactCaptureStatus::TooLarge;
        const auto& placement=location.Transformation();
        for(int row=1;row<=3;++row)for(int column=1;column<=4;++column)
            if(!std::isfinite(placement.Value(row,column)))return ContactCaptureStatus::Invalid;
        if(!std::isfinite(placement.ScaleFactor()) || placement.ScaleFactor()==0)
            return ContactCaptureStatus::Invalid;
        std::vector<Point> nodes;nodes.reserve(mesh->NbNodes());
        for(int node=1;node<=mesh->NbNodes();++node) {
            if((node&63)==1)checkpoint();
            const auto point=mesh->Node(node),placed=point.Transformed(placement);
            for(int axis=1;axis<=3;++axis)
                if(!std::isfinite(point.Coord(axis)) || std::abs(point.Coord(axis))>1.e6
                    || !std::isfinite(placed.Coord(axis)) || std::abs(placed.Coord(axis))>1.e6)
                    return ContactCaptureStatus::Invalid;
            nodes.push_back({point.X(),point.Y(),point.Z()});
        }
        ContactMeshCoordinates captured;captured.storedNodeCount=nodes.size();
        captured.faceOrientation=int(face.Orientation());
        for(int row=1;row<=3;++row)for(int column=1;column<=4;++column)
            captured.facePlacement[(row-1)*4+column-1]=placement.Value(row,column);
        captured.triangles.reserve(mesh->NbTriangles());
        captured.triangleNodeIDs.reserve(mesh->NbTriangles());
        for(int ordinal=1;ordinal<=mesh->NbTriangles();++ordinal) {
            if((ordinal&63)==1)checkpoint();
            int ids[3];mesh->Triangle(ordinal).Get(ids[0],ids[1],ids[2]);
            Triangle triangle;
            for(int corner=0;corner<3;++corner) {
                if(ids[corner]<1 || ids[corner]>mesh->NbNodes())return ContactCaptureStatus::Invalid;
                triangle[corner]=nodes[ids[corner]-1];
            }
            captured.triangles.push_back(triangle);
            captured.triangleNodeIDs.push_back({std::uint32_t(ids[0]),
                std::uint32_t(ids[1]),std::uint32_t(ids[2])});
        }
        captured.storedNodes=std::move(nodes);
        checkpoint();output=std::move(captured);return ContactCaptureStatus::Ready;
    } catch(ContactCaptureStatus status) {return status;}
      catch(const std::bad_alloc&) {return ContactCaptureStatus::TooLarge;}
      catch(const std::length_error&) {return ContactCaptureStatus::TooLarge;}
      catch(...) {return ContactCaptureStatus::Invalid;}
}
} // namespace core3d::meshcheck
