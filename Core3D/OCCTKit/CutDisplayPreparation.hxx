#pragma once
// Worker-only mutation of the private Boolean result, before OCAF publication.
// Captured presentation values contain no live drawer, AIS, document or labels.
#include "PlanarSweepSolid.hxx"
#include <Prs3d_Drawer.hxx>
#include <StdPrs_ToolTriangulatedShape.hxx>
#include <BRepMesh_DiscretFactory.hxx>
#include <BRepMesh_DiscretRoot.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <Precision.hxx>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>

namespace core3d::cut_display {
struct Settings {
    Aspect_TypeOfDeflection type=Aspect_TOD_RELATIVE;
    // Exact effective values, plus own/previous values which can trigger a
    // destructive ClearOnOwnDeflectionChange during subsequent AIS compute.
    std::array<double,5> values{}; // coefficient, angle, chord, previous coefficient, previous angle
    bool automatic=false,ownCoefficient=false,ownAngle=false;
    bool operator==(const Settings& other)const noexcept {
        return type==other.type&&automatic==other.automatic&&ownCoefficient==other.ownCoefficient
            &&ownAngle==other.ownAngle&&std::memcmp(values.data(),other.values.data(),values.size()*sizeof(double))==0;
    }
};
inline bool Capture(const Handle(Prs3d_Drawer)& drawer,Settings& out)noexcept {
    out={};try {
        if(drawer.IsNull())return false;
        Settings s;s.type=drawer->TypeOfDeflection();s.automatic=drawer->IsAutoTriangulation();
        s.ownCoefficient=drawer->HasOwnDeviationCoefficient();s.ownAngle=drawer->HasOwnDeviationAngle();
        s.values={drawer->DeviationCoefficient(),drawer->DeviationAngle(),drawer->MaximalChordialDeviation(),
            drawer->PreviousDeviationCoefficient(),drawer->PreviousDeviationAngle()};
        if((s.type!=Aspect_TOD_RELATIVE&&s.type!=Aspect_TOD_ABSOLUTE)||!s.automatic)return false;
        for(double v:s.values)if(!std::isfinite(v))return false;
        if(s.values[0]<=0||s.values[1]<=0||s.values[1]>=M_PI||s.values[2]<=0)return false;
        // An already pending drawer change must finish its normal presentation
        // update first. Never mark it consumed on the live drawer ourselves.
        if((s.ownCoefficient&&std::abs(s.values[0]-s.values[3])>Precision::Confusion())
            ||(s.ownAngle&&std::abs(s.values[1]-s.values[4])>Precision::Angular()))return false;
        out=s;return true;
    }catch(...){return false;}
}
inline bool Prepare(const TopoDS_Shape& shape,const Settings& settings,const std::atomic_bool& stop)noexcept {
    try {
        if(stop.load()||shape.IsNull()||shape.ShapeType()!=TopAbs_SOLID)return false;
        Handle(Prs3d_Drawer) drawer=new Prs3d_Drawer();
        drawer->SetTypeOfDeflection(settings.type);drawer->SetDeviationCoefficient(settings.values[0]);
        drawer->SetDeviationAngle(settings.values[1]);drawer->SetMaximalChordialDeviation(settings.values[2]);
        const double deflection=StdPrs_ToolTriangulatedShape::GetDeflection(shape,drawer);
        if(!std::isfinite(deflection)||deflection<=0||stop.load())return false;
        // Same factory/deflection/angle entry used by StdPrs::Tessellate, with
        // the existing native cancellation progress supplied to Perform.
        if(!StdPrs_ToolTriangulatedShape::IsTessellated(shape,drawer)){
            auto mesher=BRepMesh_DiscretFactory::Get().Discret(shape,deflection,settings.values[1]);
            if(mesher.IsNull())return false;
            Handle(Message_ProgressIndicator) progress=new planar_sweep::detail::CancellationProgress(stop);
            mesher->Perform(progress->Start());if(!mesher->IsDone()||stop.load())return false;
        }
        if(!StdPrs_ToolTriangulatedShape::IsTessellated(shape,drawer))return false;
        // Explicit retained-result mesh budget. OCCT's internal allocation
        // precedes this check; this is not a strict allocator bound.
        constexpr std::size_t maxFaces=1024,maxNodes=524288,maxTriangles=262144;
        std::size_t faces=0,nodes=0,triangles=0;
        for(TopExp_Explorer it(shape,TopAbs_FACE);it.More();it.Next()){
            if(stop.load()||++faces>maxFaces)return false;
            const auto face=TopoDS::Face(it.Current());TopLoc_Location location;
            const auto mesh=BRep_Tool::Triangulation(face,location);
            if(mesh.IsNull()||!mesh->HasGeometry()||!mesh->HasUVNodes()||mesh->HasDeferredData()
                ||mesh->NbNodes()<3||mesh->NbTriangles()<1
                ||std::size_t(mesh->NbNodes())>maxNodes-nodes||std::size_t(mesh->NbTriangles())>maxTriangles-triangles)return false;
            nodes+=mesh->NbNodes();triangles+=mesh->NbTriangles();
            // Same native normal materialization used by shaded FillTriangles.
            StdPrs_ToolTriangulatedShape::ComputeNormals(face,mesh);
            if(!mesh->HasNormals()||stop.load())return false;
            for(int n=1;n<=mesh->NbNodes();++n){
                if((n&255)==0&&stop.load())return false;
                const auto p=mesh->Node(n);const auto uv=mesh->UVNode(n);const auto normal=mesh->Normal(n);
                const auto world=p.Transformed(location.Transformation());
                for(double value:{p.X(),p.Y(),p.Z(),world.X(),world.Y(),world.Z(),uv.X(),uv.Y(),normal.X(),normal.Y(),normal.Z()})
                    if(!std::isfinite(value))return false;
                // Physical coordinate bounds were already proven by the Boolean
                // builder using recipe.metersPerUnit; do not impose a new native-unit bound.
            }
            for(int t=1;t<=mesh->NbTriangles();++t){
                if((t&255)==0&&stop.load())return false;
                int a,b,c;mesh->Triangle(t).Get(a,b,c);
                if(a<1||b<1||c<1||a>mesh->NbNodes()||b>mesh->NbNodes()||c>mesh->NbNodes()||a==b||b==c||a==c)return false;
            }
        }
        return faces>0&&!stop.load();
    }catch(...){return false;}
}
} // namespace core3d::cut_display
