#pragma once
// Caller must own main-thread native access and
// capture/revalidate exact source metadata, transform and opening authority.
// This only prepares private geometry. It never creates a live object, copies
// appearance, hides a source, changes selection, or opens an OCAF transaction.
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Vec.hxx>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <utility>
#include <vector>

namespace core3d::meshcopy {
enum class PreparationResult { Ready, Cancelled, Unsupported, TooLarge, Invalid };
struct CurrentTessellationCopy {
    TopoDS_Face face;
    int sourceFaces=0,triangles=0;
    // Recorded source mesh metadata, not an independently proven geometric
    // error bound. This path performs no analytic remeshing/refinement.
    double recordedDeflection=0;
};
inline PreparationResult PrepareCurrentTessellationCopy(
    const TopoDS_Shape& source, CurrentTessellationCopy& output,
    const std::atomic_bool& cancelled) noexcept {
    output={};
    constexpr int maxFaces=256,maxTriangles=4096,maxSourceNodes=12288;
    constexpr double coordinateLimit=1.0e6;
    const auto finitePoint=[](const gp_Pnt& p) {
        return std::isfinite(p.X()) && std::isfinite(p.Y()) && std::isfinite(p.Z())
            && std::abs(p.X())<=coordinateLimit && std::abs(p.Y())<=coordinateLimit
            && std::abs(p.Z())<=coordinateLimit;
    };
    struct FaceSource { Handle(Poly_Triangulation) mesh; gp_Trsf location; bool reverse=false; };
    try {
        if (cancelled.load(std::memory_order_relaxed)) return PreparationResult::Cancelled;
        if (source.IsNull() || source.ShapeType()!=TopAbs_SOLID) return PreparationResult::Unsupported;
        std::vector<FaceSource> faces;faces.reserve(maxFaces);
        int triangles=0,nodes=0;double deflection=0;
        for (TopExp_Explorer it(source,TopAbs_FACE);it.More();it.Next()) {
            if (cancelled.load(std::memory_order_relaxed)) return PreparationResult::Cancelled;
            if (faces.size()==maxFaces) return PreparationResult::TooLarge;
            const auto face=TopoDS::Face(it.Current());
            if ((face.Orientation()!=TopAbs_FORWARD && face.Orientation()!=TopAbs_REVERSED)
                || BRep_Tool::Surface(face).IsNull()) return PreparationResult::Unsupported;
            TopLoc_Location location;const auto mesh=BRep_Tool::Triangulation(face,location);
            if (mesh.IsNull() || mesh->HasDeferredData() || !mesh->HasGeometry()
                || mesh->NbNodes()<=0 || mesh->NbTriangles()<=0) return PreparationResult::Unsupported;
            if (mesh->NbNodes()>maxSourceNodes-nodes || mesh->NbTriangles()>maxTriangles-triangles)
                return PreparationResult::TooLarge;
            nodes+=mesh->NbNodes();triangles+=mesh->NbTriangles();
            const auto tr=location.Transformation();
            for (int r=1;r<=3;++r) for (int c=1;c<=4;++c)
                if (!std::isfinite(tr.Value(r,c))) return PreparationResult::Invalid;
            const double d=mesh->Deflection()*std::abs(tr.ScaleFactor());
            if (!std::isfinite(d) || d<0) return PreparationResult::Invalid;
            deflection=std::max(deflection,d);
            // Validate unused stored nodes too. Later authoring must never
            // inherit a hidden nonfinite coordinate from an admitted source.
            for (int n=1;n<=mesh->NbNodes();++n) {
                if ((n&255)==0 && cancelled.load(std::memory_order_relaxed)) return PreparationResult::Cancelled;
                const auto point=mesh->Node(n);
                if (!finitePoint(point) || !finitePoint(point.Transformed(tr))) return PreparationResult::Invalid;
            }
            faces.push_back({mesh,tr,(face.Orientation()==TopAbs_REVERSED)!=bool(tr.IsNegative())});
        }
        if (faces.empty() || triangles<=0) return PreparationResult::Unsupported;
        // One private, forward, mesh-only face with independent corner nodes.
        // Flat geometric normals are intentional. Parametric surface UVs are
        // not usable atlas UVs and are deliberately not copied.
        Handle(Poly_Triangulation) result=new Poly_Triangulation(3*triangles,triangles,false,true);
        int triangle=0;
        for (const auto& f:faces) for (int t=1;t<=f.mesh->NbTriangles();++t) {
            if (cancelled.load(std::memory_order_relaxed)) return PreparationResult::Cancelled;
            int ids[3];f.mesh->Triangle(t).Get(ids[0],ids[1],ids[2]);
            for (int id:ids) if (id<1 || id>f.mesh->NbNodes()) return PreparationResult::Invalid;
            if (f.reverse) std::swap(ids[1],ids[2]);
            gp_Pnt points[3];for (int k=0;k<3;++k) points[k]=f.mesh->Node(ids[k]).Transformed(f.location);
            const gp_Vec edge(points[0],points[1]),other(points[0],points[2]);
            gp_Vec normal=edge.Crossed(other);const double area=normal.Magnitude();
            const double scale=edge.Magnitude()*other.Magnitude();
            if (!std::isfinite(area) || !std::isfinite(scale) || scale<=0 || area<=scale*1.0e-12)
                return PreparationResult::Invalid;
            normal/=area;const gp_Vec3f packed{float(normal.X()),float(normal.Y()),float(normal.Z())};
            const int first=3*triangle+1;
            for (int k=0;k<3;++k) {result->SetNode(first+k,points[k]);result->SetNormal(first+k,packed);}
            result->SetTriangle(++triangle,Poly_Triangle(first,first+1,first+2));
        }
        if (triangle!=triangles || cancelled.load(std::memory_order_relaxed))
            return cancelled.load(std::memory_order_relaxed)?PreparationResult::Cancelled:PreparationResult::Invalid;
        result->Deflection(deflection);TopoDS_Face face;BRep_Builder().MakeFace(face,result);
        output={face,int(faces.size()),triangles,deflection};return PreparationResult::Ready;
    } catch (...) {output={};return PreparationResult::Invalid;}
}
} // namespace core3d::meshcopy
