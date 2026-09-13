#pragma once
// External, uncompiled read-only boundary classifier. No document/command authority.
#include "EnclosureBoundaryExpectation.hxx"
#include "EnclosureParameters.hxx"
#include <BRep_Tool.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_GCurve.hxx>
#include <BRep_TFace.hxx>
#include <BRep_TVertex.hxx>
#include <BRep_PointRepresentation.hxx>
#include <BRep_ListIteratorOfListOfPointRepresentation.hxx>
#include <BRep_CurveRepresentation.hxx>
#include <BRep_ListIteratorOfListOfCurveRepresentation.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_Circle.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <TopLoc_Datum3D.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp.hxx>
#include <TopAbs.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec2d.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Circ.hxx>
#include <gp_Circ2d.hxx>
#include <atomic>
#include <limits>
#include <set>

namespace core3d::enclosure_correspondence {
enum class Classification { Refused, Cancelled, MatchedBoundary };
#if DEBUG
struct Diagnostic483 {bool recorded=false;const char* branch="none";std::array<double,4> operands{};};
inline thread_local Diagnostic483* activeDiagnostic483=nullptr;
struct DiagnosticScope483 {Diagnostic483* prior;explicit DiagnosticScope483(Diagnostic483& value):prior(activeDiagnostic483){activeDiagnostic483=&value;}~DiagnosticScope483(){activeDiagnostic483=prior;}};
inline bool Check483(bool value,const char* branch,double a=0,double b=0,double c=0,double d=0){
    if(!value&&activeDiagnostic483&&!activeDiagnostic483->recorded){
        activeDiagnostic483->recorded=true;activeDiagnostic483->branch=branch;activeDiagnostic483->operands={a,b,c,d};}
    return value;
}
#define ENC483_CHECK(value,branch,a,b,c,d) ([&](){const bool result483=(value);return ::core3d::enclosure_correspondence::Check483(result483,(branch),(a),(b),(c),(d));}())
inline bool Equal483(double a,double b,const char* branch){return Check483(a==b,branch,a,b);}
#define ENC483_EQUAL(a,b,branch) ::core3d::enclosure_correspondence::Equal483((a),(b),(branch))
#define ENC483_REFUSE(branch,expression) (::core3d::enclosure_correspondence::Check483(false,(branch)),(expression))
#else
#define ENC483_CHECK(value,branch,a,b,c,d) (value)
#define ENC483_EQUAL(a,b,branch) ((a)==(b))
#define ENC483_REFUSE(branch,expression) (expression)
#endif
struct Inspection {
#if DEBUG
    Diagnostic483 diagnostic483;
#endif
    std::size_t vertices=0,edges=0,faces=0,wires=0,coedges=0,storedPCurves=0,generatedPCurves=0;
    double arithmeticMagnitudeMM=0,comparisonErrorMM=0,maximumKernelToleranceMM=0,minimumFeatureSeparationMM=0;
    double maximumLocationCompositionMagnitude=0;
    unsigned maximumLocationDepth=0;const char* phase="input";
};
namespace detail {
inline constexpr unsigned MaximumLocationDepth=8,MaximumWrappers=8,MaximumRepresentations=16;
inline constexpr double KernelToleranceMM=.001,ConditioningLimitMM=1e-6;
inline bool Finite(const gp_Pnt& p){return std::isfinite(p.X())&&std::isfinite(p.Y())&&std::isfinite(p.Z());}
inline bool Finite(const gp_Vec& p){return std::isfinite(p.X())&&std::isfinite(p.Y())&&std::isfinite(p.Z());}
inline bool Oriented(const TopoDS_Shape& s){return s.Orientation()==TopAbs_FORWARD||s.Orientation()==TopAbs_REVERSED;}
inline gp_Vec V(const gp_Pnt& p){return gp_Vec(p.X(),p.Y(),p.Z());}
inline gp_Pnt P(const gp_Vec& v){return gp_Pnt(v.X(),v.Y(),v.Z());}
inline bool Track(double v,double mm,Inspection& r){
    if(!std::isfinite(v)||v<0||!std::isfinite(v*mm))return false;
    r.arithmeticMagnitudeMM=std::max(r.arithmeticMagnitudeMM,v*mm);return true;
}
inline bool Matrix(const gp_Trsf& t){
    const double s=t.ScaleFactor();if(!std::isfinite(s)||s<1e-6||s>1e6)return false;
    for(int i=1;i<=3;++i)for(int j=1;j<=4;++j)if(!std::isfinite(t.Value(i,j))||std::abs(t.Value(i,j))>1e6)return false;return true;
}
// Inspect primitive datums/powers BEFORE evaluating a cumulative transform.
inline bool LocationChain(const TopLoc_Location& location,std::vector<gp_Trsf>& chain,Inspection& r){
    chain.clear();auto tail=location;
    while(!tail.IsIdentity()){
        if(chain.size()>=MaximumLocationDepth||tail.FirstDatum().IsNull())return false;
        const int power=tail.FirstPower();if(power!=1&&power!=-1)return false;
        auto datum=tail.FirstDatum()->Transformation();if(!Matrix(datum))return false;
        if(power==-1){datum.Invert();if(!Matrix(datum))return false;}
        chain.push_back(datum);tail=tail.NextLocation();
    }
    // Bound every intermediate affine matrix product, separately from later
    // point evaluations; cancellation in the final matrix cannot hide work.
    std::array<std::array<double,4>,3> magnitude{{{1,0,0,0},{0,1,0,0},{0,0,1,0}}};
    for(const auto& datum:chain){std::array<std::array<double,4>,3> next{};
        for(unsigned i=0;i<3;++i)for(unsigned j=0;j<4;++j){
            next[i][j]=j==3?std::abs(datum.Value(i+1,4)):0;
            for(unsigned k=0;k<3;++k)next[i][j]+=std::abs(datum.Value(i+1,k+1))*magnitude[k][j];
            if(!std::isfinite(next[i][j]))return false;
            r.maximumLocationCompositionMagnitude=std::max(r.maximumLocationCompositionMagnitude,next[i][j]);
        }magnitude=next;
    }
    r.maximumLocationDepth=std::max(r.maximumLocationDepth,unsigned(chain.size()));return true;
}
inline bool Location(const TopLoc_Location& location,Inspection& r){std::vector<gp_Trsf> chain;return LocationChain(location,chain,r);}
inline bool AffineMagnitude(std::array<double,3>& sums,const gp_Trsf& t,bool vector,double mm,Inspection& r){
    std::array<double,3> out{};
    for(int i=0;i<3;++i){out[i]=vector?0:std::abs(t.Value(i+1,4));for(int j=0;j<3;++j)out[i]+=std::abs(t.Value(i+1,j+1))*sums[j];if(!Track(out[i],mm,r))return false;}sums=out;return true;
}
inline bool LocatedMagnitude(const gp_Vec& absoluteInput,const TopLoc_Location& location,bool vector,double mm,Inspection& r){
    if(!Finite(absoluteInput))return false;std::vector<gp_Trsf> chain;if(!LocationChain(location,chain,r))return false;
    std::array<double,3> sums{std::abs(absoluteInput.X()),std::abs(absoluteInput.Y()),std::abs(absoluteInput.Z())};
    for(double x:sums)if(!Track(x,mm,r))return false;
    // TopLoc stores the first-applied datum at the head (SList constructor
    // premultiplies by its tail). Preserve this order, including magnitude.
    for(const auto& datum:chain)if(!AffineMagnitude(sums,datum,vector,mm,r))return false;
    return true;
}
inline bool Children(const TopoDS_Shape& parent,TopAbs_ShapeEnum type,unsigned cap,std::vector<TopoDS_Shape>& out,Inspection& r){
    out.clear();if(!Location(parent.Location(),r))return false;
    // No implicit cumLoc: inspect each input chain before composition.
    for(TopoDS_Iterator it(parent,Standard_False,Standard_False);it.More();it.Next()){
        auto child=it.Value();if(child.IsNull()||child.ShapeType()!=type||!Oriented(child)||out.size()>=cap||!Location(child.Location(),r))return false;
        std::vector<gp_Trsf> a,b;if(!LocationChain(parent.Location(),a,r)||!LocationChain(child.Location(),b,r)||a.size()+b.size()>MaximumLocationDepth)return false;
        const auto combined=parent.Location()*child.Location();if(!Location(combined,r))return false;
        child.Location(combined);child.Orientation(TopAbs::Compose(parent.Orientation(),child.Orientation()));out.push_back(child);
    }return true;
}
inline bool Tolerance(double value,const TopoDS_Shape& shape,double mm,Inspection& r){
    if(!Location(shape.Location(),r))return false;const double scaled=value*shape.Location().Transformation().ScaleFactor()*mm;
    if(!std::isfinite(scaled)||scaled<0||scaled>KernelToleranceMM)return false;r.maximumKernelToleranceMM=std::max(r.maximumKernelToleranceMM,scaled);return true;
}
struct TrimDomains {
    std::array<std::array<double,2>,MaximumWrappers> values{};unsigned count=0;
    std::size_t size()const noexcept{return count;}
    const std::array<double,2>* begin()const noexcept{return values.data();}
    const std::array<double,2>* end()const noexcept{return values.data()+count;}
};
struct Curve {bool circle=false;gp_Vec c,a,b;TopLoc_Location location;double first=0,last=0;TrimDomains trims;}; // line c+t*a; circle c+cos(t)*a+sin(t)*b
struct PCurve {bool circle=false;gp_Pnt2d c;gp_Vec2d a,b;double first=0,last=0;bool stored=false;TrimDomains trims;};
struct Surface {bool cylinder=false;gp_Vec c,x,y,z,rawC,rawX,rawY,rawZ;TopLoc_Location location;Handle(Geom_Surface) handle;std::vector<std::array<double,4>> boxes;};
inline bool Range(double first,double last){return std::isfinite(first)&&std::isfinite(last)&&first<last;}
// A BRep V3 topology interval is written with15 significant digits, whereas
// GeomTools trim bounds use17. Retain every domain; never clamp a range or
// mutate a wrapper. Full analytic physical charges are checked only against
// the independently fixed comparison error below.
inline bool RetainTrim(TrimDomains& trims,double first,double last,double lower,double upper){
    if(!Range(first,last)||!Range(lower,upper)||trims.size()>=MaximumWrappers)return false;
    double lo=std::max(first,lower),hi=std::min(last,upper);
    for(const auto& t:trims){lo=std::max(lo,t[0]);hi=std::min(hi,t[1]);}
    if(!(lo<hi))return false;trims.values[trims.count++]={lower,upper};return true;
}
// Positive arithmetic is rounded outward; overflow/nonfinite stays refusal.
inline bool TrimUpperAdd(double a,double b,double& out){
    if(!std::isfinite(a)||!std::isfinite(b)||a<0||b<0)return false;
    if(a==0){out=b;return true;}if(b==0){out=a;return true;}
    const double v=a+b;if(!std::isfinite(v))return false;
    out=std::nextafter(v,std::numeric_limits<double>::infinity());return std::isfinite(out);
}
inline bool TrimUpperMultiply(double a,double b,double& out){
    if(!std::isfinite(a)||!std::isfinite(b)||a<0||b<0)return false;
    if(a==0||b==0){out=0;return true;}const double v=a*b;
    if(!std::isfinite(v))return false;out=std::nextafter(v,std::numeric_limits<double>::infinity());return std::isfinite(out);
}
inline bool TrimUpperProduct3(double a,double b,double c,double& out){
    double ab=0;return TrimUpperMultiply(a,b,ab)&&TrimUpperMultiply(ab,c,out);
}
inline bool TrimL1(const gp_Vec& v,double& out){
    double xy=0;return TrimUpperAdd(std::abs(v.X()),std::abs(v.Y()),xy)&&TrimUpperAdd(xy,std::abs(v.Z()),out);
}
inline bool TrimSpan(const TrimDomains& trims,double first,double last,double& out){
    out=0;if(!Range(first,last)||trims.size()>MaximumWrappers)return false;
    double lo=first,hi=last;
    for(const auto& t:trims){
        if(!Range(t[0],t[1]))return false;lo=std::max(lo,t[0]);hi=std::min(hi,t[1]);
        const double left=t[0]-first,right=last-t[1];
        if(!std::isfinite(left)||!std::isfinite(right))return false;
        const double v=std::max({0.,left,right});
        if(v>0){const double u=std::nextafter(v,std::numeric_limits<double>::infinity());if(!std::isfinite(u))return false;out=std::max(out,u);}
    }return lo<hi;
}
inline bool CurveTrimCharge(const Curve& c,double mm,double& out){
    out=0;double span=0;if(!std::isfinite(mm)||mm<=0||!TrimSpan(c.trims,c.first,c.last,span))return false;
    if(span==0)return true;double a=0,b=0,speed=0,scaled=0;
    if(!TrimL1(c.a,a)||(c.circle&&!TrimL1(c.b,b))||!TrimUpperAdd(a,b,speed)||speed<=0
        ||!TrimUpperMultiply(speed,mm,scaled)||!TrimUpperMultiply(span,scaled,out))return false;return true;
}
inline bool PCurveTrimCharge(const PCurve& pc,const Surface& s,double mm,double& out){
    out=0;double span=0;if(!std::isfinite(mm)||mm<=0||!TrimSpan(pc.trims,pc.first,pc.last,span))return false;
    if(span==0)return true;double x=0,y=0,z=0,speed=0,scaled=0;
    if(!TrimL1(s.x,x)||!TrimL1(s.y,y)||!TrimL1(s.z,z))return false;
    if(s.cylinder){
        if(pc.circle)return false;double radial=0,u=0,v=0;
        if(!TrimUpperAdd(x,y,radial)||!TrimUpperMultiply(radial,std::abs(pc.a.X()),u)
            ||!TrimUpperMultiply(z,std::abs(pc.a.Y()),v)||!TrimUpperAdd(u,v,speed))return false;
    }else{
        double ux=0,vy=0,a=0,b=0;
        if(!TrimUpperMultiply(x,std::abs(pc.a.X()),ux)||!TrimUpperMultiply(y,std::abs(pc.a.Y()),vy)||!TrimUpperAdd(ux,vy,a))return false;
        if(pc.circle&&(!TrimUpperMultiply(x,std::abs(pc.b.X()),ux)||!TrimUpperMultiply(y,std::abs(pc.b.Y()),vy)||!TrimUpperAdd(ux,vy,b)))return false;
        if(!TrimUpperAdd(a,b,speed))return false;
    }
    return speed>0&&TrimUpperMultiply(speed,mm,scaled)&&TrimUpperMultiply(span,scaled,out);
}
inline bool TrimResidualWithin(const Curve& c,const PCurve& pc,const Surface& s,double mm,double residualMM,double error){
    if(!std::isfinite(residualMM)||residualMM<0||!std::isfinite(error)||error<0)return false;
    double curve=0,pcurve=0,total=0;
    return CurveTrimCharge(c,mm,curve)&&PCurveTrimCharge(pc,s,mm,pcurve)
        &&TrimUpperAdd(residualMM,curve,total)&&TrimUpperAdd(total,pcurve,total)&&total<=error;
}
inline bool ReadCurve(const TopoDS_Edge& edge,double mm,Inspection& r,Curve& out){
    TopLoc_Location loc;double first=0,last=0;auto curve=BRep_Tool::Curve(edge,loc,first,last);
    if(!ENC483_CHECK(Range(first,last),"curve-range",first,last,0,0)||!ENC483_CHECK(Location(loc,r),"curve-location",0,0,0,0))return false;
    Curve c;Handle(Geom_Line) line;Handle(Geom_Circle) circle;
    for(unsigned n=0;n<MaximumWrappers&&!curve.IsNull();++n){
        line=Handle(Geom_Line)::DownCast(curve);circle=Handle(Geom_Circle)::DownCast(curve);if(!line.IsNull()||!circle.IsNull())break;
        const auto trim=Handle(Geom_TrimmedCurve)::DownCast(curve);
        if(trim.IsNull()||!RetainTrim(c.trims,first,last,trim->FirstParameter(),trim->LastParameter()))return false;curve=trim->BasisCurve();
    }
    c.location=loc;c.first=first;c.last=last;
    if(!line.IsNull()){const auto l=line->Lin();c.c=V(l.Location());c.a=gp_Vec(l.Direction());}
    else if(!circle.IsNull()){
        if(std::max(std::abs(first),std::abs(last))>32*std::acos(-1.0)||last-first>std::acos(-1.0))return false;
        c.circle=true;const auto q=circle->Circ();c.c=V(q.Location());c.a=gp_Vec(q.Position().XDirection())*q.Radius();c.b=gp_Vec(q.Position().YDirection())*q.Radius();
    }else return false;
    const double extent=c.circle?1:std::max(std::abs(first),std::abs(last));
    const gp_Vec sums(std::abs(c.c.X())+extent*std::abs(c.a.X())+std::abs(c.b.X()),std::abs(c.c.Y())+extent*std::abs(c.a.Y())+std::abs(c.b.Y()),std::abs(c.c.Z())+extent*std::abs(c.a.Z())+std::abs(c.b.Z()));
    if(!LocatedMagnitude(sums,loc,false,mm,r))return false;
    const auto t=loc.Transformation();auto origin=P(c.c);origin.Transform(t);c.c=V(origin);c.a.Transform(t);c.b.Transform(t);
    if(!Finite(c.c)||!Finite(c.a)||!Finite(c.b))return false;out=c;return true;
}
inline bool ReadSurface(const TopoDS_Face& face,double mm,Inspection& r,Surface& out){
    TopLoc_Location loc;auto surface=BRep_Tool::Surface(face,loc);if(!Location(loc,r))return false;
    Surface s;s.handle=surface;Handle(Geom_Plane) plane;Handle(Geom_CylindricalSurface) cylinder;
    for(unsigned n=0;n<MaximumWrappers&&!surface.IsNull();++n){
        plane=Handle(Geom_Plane)::DownCast(surface);cylinder=Handle(Geom_CylindricalSurface)::DownCast(surface);if(!plane.IsNull()||!cylinder.IsNull())break;
        const auto trim=Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);if(trim.IsNull())return false;
        double u0,u1,v0,v1;trim->Bounds(u0,u1,v0,v1);if(!Range(u0,u1)||!Range(v0,v1))return false;
        s.boxes.push_back({u0,u1,v0,v1});surface=trim->BasisSurface();
    }
    gp_Ax3 axes;
    if(!plane.IsNull())axes=plane->Pln().Position();
    else if(!cylinder.IsNull()){s.cylinder=true;axes=cylinder->Cylinder().Position();}else return false;
    s.c=V(axes.Location());s.x=gp_Vec(axes.XDirection());s.y=gp_Vec(axes.YDirection());s.z=gp_Vec(axes.Direction());
    if(s.cylinder){const double radius=cylinder->Radius();if(!std::isfinite(radius)||radius<=0)return false;s.x*=radius;s.y*=radius;}
    if(!LocatedMagnitude(s.c,loc,false,mm,r)||!LocatedMagnitude(s.x,loc,true,mm,r)||!LocatedMagnitude(s.y,loc,true,mm,r)||!LocatedMagnitude(s.z,loc,true,mm,r))return false;
    s.rawC=s.c;s.rawX=s.x;s.rawY=s.y;s.rawZ=s.z;s.location=loc;
    const auto t=loc.Transformation();auto p=P(s.c);p.Transform(t);s.c=V(p);s.x.Transform(t);s.y.Transform(t);s.z.Transform(t);
    out=std::move(s);return Finite(out.c)&&Finite(out.x)&&Finite(out.y)&&Finite(out.z);
}
inline bool ReadPCurve(const TopoDS_Edge& edge,const Surface& surface,const Curve& c,PCurve& out,Inspection& r){
    // Caller has bounded this relative location pair before composition. Inspect
    // stored pcurves directly: never invoke the generic projection fallback.
    const auto relative=surface.location.Predivided(edge.Location());if(!Location(relative,r))return false;
    const auto data=Handle(BRep_TEdge)::DownCast(edge.TShape());
    Handle(Geom2d_Curve) curve;double f=c.first,l=c.last;unsigned matches=0;
    for(BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());it.More();it.Next()){
        if(!it.Value()->IsCurveOnSurface(surface.handle,relative))continue;
        const auto gc=Handle(BRep_GCurve)::DownCast(it.Value());
        if(gc.IsNull()||++matches!=1||gc->IsCurveOnClosedSurface())return false;
        gc->Range(f,l);curve=gc->PCurve();
    }
    if(!ENC483_EQUAL(f,c.first,"pcurve-first")||!ENC483_EQUAL(l,c.last,"pcurve-last")||!ENC483_CHECK(Range(f,l),"pcurve-range",f,l,0,0))return false;PCurve pc;pc.first=f;pc.last=l;pc.stored=matches!=0;
    if(matches==0){
        if(surface.cylinder)return false;
        // An absent planar pcurve is represented by its complete analytic
        // projection, preserving absence. The residual check below proves
        // that the entire 3D curve already lies on the exact plane support.
        const double xx=surface.x.SquareMagnitude(),yy=surface.y.SquareMagnitude();
        if(!std::isfinite(xx)||!std::isfinite(yy)||xx<=0||yy<=0)return false;
        const auto delta=c.c-surface.c;pc.circle=c.circle;
        pc.c=gp_Pnt2d(delta.Dot(surface.x)/xx,delta.Dot(surface.y)/yy);
        pc.a=gp_Vec2d(c.a.Dot(surface.x)/xx,c.a.Dot(surface.y)/yy);
        pc.b=gp_Vec2d(c.b.Dot(surface.x)/xx,c.b.Dot(surface.y)/yy);out=pc;return true;
    }
    for(unsigned n=0;n<MaximumWrappers&&!curve.IsNull();++n){
        const auto line=Handle(Geom2d_Line)::DownCast(curve);const auto circle=Handle(Geom2d_Circle)::DownCast(curve);
        if(!line.IsNull()){const auto x=line->Lin2d();pc.c=x.Location();pc.a=gp_Vec2d(x.Direction());out=pc;return true;}
        if(!circle.IsNull()){const auto x=circle->Circ2d();pc.circle=true;pc.c=x.Location();pc.a=gp_Vec2d(x.Position().XDirection())*x.Radius();pc.b=gp_Vec2d(x.Position().YDirection())*x.Radius();out=pc;return true;}
        const auto trim=Handle(Geom2d_TrimmedCurve)::DownCast(curve);if(trim.IsNull()||!RetainTrim(pc.trims,f,l,trim->FirstParameter(),trim->LastParameter()))return false;curve=trim->BasisCurve();
    }return false;
}
inline gp_Vec Evaluate(const Curve& c,double t){return c.circle?c.c+c.a*std::cos(t)+c.b*std::sin(t):c.c+c.a*t;}
inline double Norm(const gp_Vec& v){return Finite(v)?v.Magnitude():std::numeric_limits<double>::infinity();}
inline bool Close(const gp_Vec& a,const gp_Vec& b,double mm,double error){return std::isfinite(Norm(a-b)*mm)&&Norm(a-b)*mm<=error;}
inline bool PCurveMagnitude(const PCurve& pc,const Surface& s,double mm,Inspection& r){
    const double e=pc.circle?1:std::max(std::abs(pc.first),std::abs(pc.last));
    const double u=std::abs(pc.c.X())+e*std::abs(pc.a.X())+std::abs(pc.b.X());
    const double v=std::abs(pc.c.Y())+e*std::abs(pc.a.Y())+std::abs(pc.b.Y());
    if(!std::isfinite(u)||!std::isfinite(v)||(s.cylinder&&(pc.circle||u>32*std::acos(-1.0))))return false;
    gp_Vec raw;for(int i=1;i<=3;++i){
        const double b=std::abs(s.rawC.Coord(i))+(s.cylinder?std::abs(s.rawX.Coord(i))+std::abs(s.rawY.Coord(i))+v*std::abs(s.rawZ.Coord(i)):u*std::abs(s.rawX.Coord(i))+v*std::abs(s.rawY.Coord(i)));
        raw.SetCoord(i,b);
        const double finalBound=std::abs(s.c.Coord(i))+(s.cylinder?std::abs(s.x.Coord(i))+std::abs(s.y.Coord(i))+v*std::abs(s.z.Coord(i)):u*std::abs(s.x.Coord(i))+v*std::abs(s.y.Coord(i)));
        if(!Track(finalBound,mm,r))return false;
    }return LocatedMagnitude(raw,s.location,false,mm,r);
}
// Exact analytic coordinate extrema over finite line/circle intervals, including
// all derivative roots. Used only to prove wrapper-domain inclusion.
inline bool PCBounds(const PCurve& p,std::array<double,4>& box){
    for(unsigned axis=0;axis<2;++axis){const double c=axis?p.c.Y():p.c.X(),a=axis?p.a.Y():p.a.X(),b=axis?p.b.Y():p.b.X();
        auto val=[&](double t){return p.circle?c+a*std::cos(t)+b*std::sin(t):c+a*t;};
        double lo=std::min(val(p.first),val(p.last)),hi=std::max(val(p.first),val(p.last));
        if(p.circle){if(std::max(std::abs(p.first),std::abs(p.last))>32*std::acos(-1.0)||p.last-p.first>std::acos(-1.0))return false;
            const double angle=std::atan2(b,a),pi=std::acos(-1.0);
            for(int k=-34;k<=34;++k){const double t=angle+k*pi;if(t>=p.first&&t<=p.last){lo=std::min(lo,val(t));hi=std::max(hi,val(t));}}}
        if(!std::isfinite(lo)||!std::isfinite(hi))return false;box[axis*2]=lo;box[axis*2+1]=hi;
    }return true;
}
// Complete analytic parameter identity. Residual is a uniform interval bound,
// never endpoint sampling of an arc or a cylinder's nonlinear image.
inline bool PCurveIdentity(const Curve& c,const PCurve& pc,const Surface& s,double mm,double error){
    std::array<double,4> bounds;if(!ENC483_CHECK(PCBounds(pc,bounds),"pcurve-wrapper-bounds",pc.first,pc.last,pc.circle,0))return false;
    for(const auto& box:s.boxes)for(unsigned axis=0;axis<2;++axis)if(!ENC483_CHECK(!(bounds[axis*2]<box[axis*2]||bounds[axis*2+1]>box[axis*2+1]),"pcurve-wrapper-domain",bounds[axis*2],bounds[axis*2+1],box[axis*2],box[axis*2+1]))return false;
    Curve composed;composed.circle=c.circle;composed.first=c.first;composed.last=c.last;
    const double extent=std::max(std::abs(c.first),std::abs(c.last));double drift=0;
    if(!s.cylinder){
        if(pc.circle!=c.circle)return false;
        composed.c=s.c+s.x*pc.c.X()+s.y*pc.c.Y();composed.a=s.x*pc.a.X()+s.y*pc.a.Y();composed.b=s.x*pc.b.X()+s.y*pc.b.Y();
    }else{
        if(pc.circle)return false;const double phase=pc.c.X();
        const auto radial=s.x*std::cos(phase)+s.y*std::sin(phase);
        if(c.circle){
            const double rate=pc.a.X(),sign=rate<0?-1:1;if(std::abs(rate)<.5)return false;
            composed.c=s.c+s.z*pc.c.Y();composed.a=radial;composed.b=(s.x*(-std::sin(phase))+s.y*std::cos(phase))*sign;
            double radialSpeed=0,u=0,v=0;
            if(!TrimUpperAdd(Norm(s.x),Norm(s.y),radialSpeed)
                ||!TrimUpperProduct3(radialSpeed,std::abs(rate-sign),extent,u)
                ||!TrimUpperProduct3(Norm(s.z),std::abs(pc.a.Y()),extent,v)||!TrimUpperAdd(u,v,drift))return false;
        }else{
            composed.c=s.c+radial+s.z*pc.c.Y();composed.a=s.z*pc.a.Y();
            double radialSpeed=0;if(!TrimUpperAdd(Norm(s.x),Norm(s.y),radialSpeed)
                ||!TrimUpperProduct3(radialSpeed,std::abs(pc.a.X()),extent,drift))return false;
        }
    }
    double coefficient=0,residual=0,residualMM=0;
    if(c.circle){if(!TrimUpperAdd(Norm(composed.a-c.a),Norm(composed.b-c.b),coefficient))return false;}
    else if(!TrimUpperMultiply(extent,Norm(composed.a-c.a),coefficient))return false;
    if(!TrimUpperAdd(Norm(composed.c-c.c),coefficient,residual)||!TrimUpperAdd(residual,drift,residual)
        ||!TrimUpperMultiply(residual,mm,residualMM))return false;
    return ENC483_CHECK(TrimResidualWithin(c,pc,s,mm,residualMM,error),"pcurve-coefficient-and-trim-residual",residualMM,error,drift*mm,extent);
}
struct Use {TopoDS_Edge edge;Curve curve;PCurve pc;TopoDS_Vertex a,b;};
struct Face {TopoDS_Face face;Surface surface;std::vector<std::vector<Use>> wires;};
}
namespace detail {
inline bool PairLocation(const TopLoc_Location& a,const TopLoc_Location& b,Inspection& r){
    std::vector<gp_Trsf> x,y;return LocationChain(a,x,r)&&LocationChain(b,y,r)&&x.size()+y.size()<=MaximumLocationDepth;
}
inline bool EdgeRepresentations(const TopoDS_Edge& edge,Inspection& r){
    const auto data=Handle(BRep_TEdge)::DownCast(edge.TShape());if(data.IsNull()||!std::isfinite(data->Tolerance())||data->Tolerance()<0)return false;
    unsigned count=0;for(BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());it.More();it.Next()){
        if(++count>MaximumRepresentations||it.Value().IsNull()||!PairLocation(edge.Location(),it.Value()->Location(),r)||it.Value()->IsCurveOnClosedSurface()
            ||(it.Value()->IsRegularity()&&!PairLocation(edge.Location(),it.Value()->Location2(),r)))return false;
    }return true;
}
inline bool VertexPreflight(const TopoDS_Vertex& vertex,double mm,Inspection& r){
    if(vertex.IsNull()||!Oriented(vertex)||TopoDS_Iterator(vertex,Standard_False,Standard_False).More()||!Location(vertex.Location(),r))return false;
    const auto data=Handle(BRep_TVertex)::DownCast(vertex.TShape());if(data.IsNull()||!std::isfinite(data->Tolerance())||data->Tolerance()<0)return false;
    unsigned count=0;for(BRep_ListIteratorOfListOfPointRepresentation it(data->Points());it.More();it.Next()){
        if(++count>MaximumRepresentations||it.Value().IsNull()||!PairLocation(vertex.Location(),it.Value()->Location(),r)||!std::isfinite(it.Value()->Parameter()))return false;
    }
    return LocatedMagnitude(V(data->Pnt()),vertex.Location(),false,mm,r)&&Tolerance(BRep_Tool::Tolerance(vertex),vertex,mm,r);
}
inline bool SurfacePreflight(const TopoDS_Face& face,Inspection& r){const auto data=Handle(BRep_TFace)::DownCast(face.TShape());return !data.IsNull()&&std::isfinite(data->Tolerance())&&data->Tolerance()>=0&&PairLocation(face.Location(),data->Location(),r);}
inline bool MatchCurve(const Curve& c,const ExpectedEdge& e,const ExpectedBoundary& expected,bool forward,double mm,double error){
    if(!ENC483_CHECK(c.circle==e.circle,"curve-kind",c.circle,e.circle,0,0))return false;
    const auto start=V(expected.vertices[forward?e.start:e.end]),end=V(expected.vertices[forward?e.end:e.start]);
    if(!ENC483_CHECK(Close(Evaluate(c,c.first),start,mm,error)&&Close(Evaluate(c,c.last),end,mm,error),"curve-endpoints",Norm(Evaluate(c,c.first)-start)*mm,Norm(Evaluate(c,c.last)-end)*mm,error,mm))return false;
    if(!c.circle)return true; // analytic affine segment over complete finite range
    const double r=Norm(c.a),q=Norm(c.b);
    if(!ENC483_CHECK(Close(c.c,V(e.center),mm,error),"circle-center",Norm(c.c-V(e.center))*mm,error,0,0)||!std::isfinite(r)||r<=0||!std::isfinite(q)||q<=0
        ||!ENC483_CHECK(!(std::abs(r-e.radius)*mm>error),"circle-radius-a",r*mm,e.radius*mm,error,0)||!ENC483_CHECK(!(std::abs(q-e.radius)*mm>error),"circle-radius-b",q*mm,e.radius*mm,error,0)
        ||!ENC483_CHECK(!(std::abs(c.a.Dot(c.b))/(r*q)*e.radius*mm>error),"circle-orthogonality",std::abs(c.a.Dot(c.b))/(r*q)*e.radius*mm,error,0,0)
        ||!ENC483_CHECK(!(std::abs((c.last-c.first)-std::acos(-1.0)/2)*e.radius*mm>error),"circle-quarter-range",c.first,c.last,std::abs((c.last-c.first)-std::acos(-1.0)/2)*e.radius*mm,error))return false;
    // Center plus non-collinear endpoints fixes the circle plane; quarter-range
    // excludes the opposite major arc and any extra periodic covering.
    const auto wanted=(start-V(e.center)).Crossed(end-V(e.center));const auto actual=c.a.Crossed(c.b);
    const double a=Norm(actual),w=Norm(wanted);if(a<=0||w<=0)return false;
    return ENC483_CHECK(Norm(actual/a-wanted/w)*e.radius*mm<=error,"circle-directed-plane",Norm(actual/a-wanted/w)*e.radius*mm,error,0,0);
}
inline bool WireMatch(const std::vector<std::pair<unsigned,bool>>& actual,const std::vector<EdgeUse>& expected){
    if(actual.size()!=expected.size())return false;
    std::set<std::pair<unsigned,bool>> a(actual.begin(),actual.end()),b;
    for(const auto& use:expected)b.emplace(use.edge,use.forward);
    return a.size()==actual.size()&&a==b;
}
inline bool MatchSurface(const Face& face,const ExpectedFace& e,double mm,double error){
    const auto& s=face.surface;if(!ENC483_CHECK(s.cylinder==e.cylinder,"surface-kind",s.cylinder,e.cylinder,0,0))return false;
    const double side=face.face.Orientation()==TopAbs_FORWARD?1:-1;
    if(!s.cylinder){
        auto n=s.x.Crossed(s.y);const double length=Norm(n),wanted=Norm(e.normalOrAxis);if(length<=0||wanted<=0)return false;
        n*=side/length;
        // Parallel support and explicit signed normal; polygon/arc loops then
        // specify the complete finite trim, including the annular top face.
        return ENC483_CHECK(Norm(n-e.normalOrAxis/wanted)*std::max(Norm(s.x),Norm(s.y))*mm<=error,"plane-signed-normal",Norm(n-e.normalOrAxis/wanted)*std::max(Norm(s.x),Norm(s.y))*mm,error,0,0)
            &&ENC483_CHECK(std::abs((s.c-V(e.origin)).Dot(n))*mm<=error,"plane-support-distance",std::abs((s.c-V(e.origin)).Dot(n))*mm,error,0,0);
    }
    const double radius=Norm(s.x),other=Norm(s.y),axis=Norm(s.z),want=Norm(e.normalOrAxis);
    if(radius<=0||other<=0||axis<=0||want<=0||!ENC483_CHECK(!(std::abs(radius-e.radius)*mm>error),"cylinder-radius-x",radius*mm,e.radius*mm,error,0)||!ENC483_CHECK(!(std::abs(other-e.radius)*mm>error),"cylinder-radius-y",other*mm,e.radius*mm,error,0))return false;
    const auto unit=s.z/axis,wanted=e.normalOrAxis/want;
    if(!ENC483_CHECK(!(Norm(unit.Crossed(wanted))*e.radius*mm>error),"cylinder-axis",Norm(unit.Crossed(wanted))*e.radius*mm,error,0,0))return false;
    const auto offset=s.c-V(e.origin);if(!ENC483_CHECK(!(Norm(offset-unit*offset.Dot(unit))*mm>error),"cylinder-support-distance",Norm(offset-unit*offset.Dot(unit))*mm,error,0,0))return false;
    const double handed=s.x.Crossed(s.y).Dot(s.z);if(!std::isfinite(handed)||handed==0)return false;
    return ENC483_CHECK((handed>0?1:-1)*side==e.radialSign,"cylinder-orientation",handed,side,e.radialSign,0);
}
}
inline Classification InspectEnclosure(const TopoDS_Shape& retained,const enclosure::Parameters& parameters,
    const std::atomic_bool& stop,Inspection& output)noexcept{
    using namespace detail;output={};Inspection report;
    const auto refuse=[&](){output=report;return stop.load()?Classification::Cancelled:Classification::Refused;};
    try{
#if DEBUG
        DiagnosticScope483 diagnosticScope(report.diagnostic483);
#endif
        if(stop.load())return ENC483_REFUSE("InspectEnclosure-return-1",refuse());ExpectedBoundary expected;
        std::vector<double> encoded;if(!enclosure::Encode(parameters,encoded)||!BuildExpectedBoundary(parameters.definition,expected))return ENC483_REFUSE("InspectEnclosure-return-2",refuse());
        const double mm=parameters.metersPerUnit*1000;if(!std::isfinite(mm)||mm<=0)return ENC483_REFUSE("InspectEnclosure-return-3",refuse());
        // Source frame has eight bounded scalar inputs, with proper quaternion
        // and positive scale; its conversion is fixed-size, never a datum chain.
        gp_Trsf frame;if(parameters.definition.constructionFrame&&!parameters.definition.constructionFrame->Transform(frame))return ENC483_REFUSE("InspectEnclosure-return-4",refuse());
        if(!Matrix(frame))return ENC483_REFUSE("InspectEnclosure-return-5",refuse());
        const auto& dimensions=parameters.definition.dimensions;
        std::array<double,3> rawSums{dimensions.width+dimensions.depth+dimensions.height,dimensions.width+dimensions.depth+dimensions.height,dimensions.width+dimensions.depth+dimensions.height};
        if(!AffineMagnitude(rawSums,frame,false,mm,report))return ENC483_REFUSE("InspectEnclosure-return-6",refuse());
        for(const auto& v:expected.vertices)if(!Track(std::abs(v.X())+std::abs(v.Y())+std::abs(v.Z()),mm,report))return ENC483_REFUSE("InspectEnclosure-return-7",refuse());
        if(retained.IsNull()||retained.ShapeType()!=TopAbs_SOLID||retained.Orientation()!=TopAbs_FORWARD||!Location(retained.Location(),report))return ENC483_REFUSE("InspectEnclosure-return-8",refuse());
        report.phase="collect";std::vector<TopoDS_Shape> shells,rawFaces;
        if(!Children(retained,TopAbs_SHELL,1,shells,report)||shells.size()!=1||!Children(shells[0],TopAbs_FACE,19,rawFaces,report)||rawFaces.size()!=19)return ENC483_REFUSE("InspectEnclosure-return-9",refuse());
        std::vector<Face> faces;unsigned totalUses=0;TopTools_IndexedMapOfShape faceMap,wireMap;
        for(const auto& rawFace:rawFaces){
            if(stop.load())return ENC483_REFUSE("InspectEnclosure-return-10",refuse());Face face;face.face=TopoDS::Face(rawFace);
            if(faceMap.Contains(face.face)||!ENC483_CHECK(SurfacePreflight(face.face,report),"surface-preflight",report.arithmeticMagnitudeMM,report.maximumKernelToleranceMM,report.maximumLocationDepth,report.maximumLocationCompositionMagnitude)||!ENC483_CHECK(Tolerance(BRep_Tool::Tolerance(face.face),face.face,mm,report),"surface-tolerance",report.arithmeticMagnitudeMM,report.maximumKernelToleranceMM,report.maximumLocationDepth,report.maximumLocationCompositionMagnitude)||!ENC483_CHECK(ReadSurface(face.face,mm,report,face.surface),"surface-read",report.arithmeticMagnitudeMM,report.maximumKernelToleranceMM,report.maximumLocationDepth,report.maximumLocationCompositionMagnitude))return ENC483_REFUSE("InspectEnclosure-return-11",refuse());faceMap.Add(face.face);
            std::vector<TopoDS_Shape> wires;if(!Children(face.face,TopAbs_WIRE,2,wires,report)||wires.empty())return ENC483_REFUSE("InspectEnclosure-return-12",refuse());
            for(const auto& wire:wires){
                if(wireMap.Contains(wire))return ENC483_REFUSE("InspectEnclosure-return-13",refuse());wireMap.Add(wire);std::vector<TopoDS_Shape> edges;
                if(!Children(wire,TopAbs_EDGE,8,edges,report)||edges.size()<4)return ENC483_REFUSE("InspectEnclosure-return-14",refuse());std::vector<Use> uses;
                for(const auto& rawEdge:edges){
                    if(stop.load()||++totalUses>96)return ENC483_REFUSE("InspectEnclosure-return-15",refuse());Use use;use.edge=TopoDS::Edge(rawEdge);
                    const auto forward=TopoDS::Edge(use.edge.Oriented(TopAbs_FORWARD));
                    if(!ENC483_CHECK(EdgeRepresentations(forward,report),"edge-representations",report.arithmeticMagnitudeMM,report.maximumKernelToleranceMM,report.maximumLocationDepth,report.maximumLocationCompositionMagnitude)||!ENC483_CHECK(PairLocation(face.surface.location,forward.Location(),report),"edge-surface-location",report.arithmeticMagnitudeMM,report.maximumKernelToleranceMM,report.maximumLocationDepth,report.maximumLocationCompositionMagnitude)||BRep_Tool::Degenerated(forward)||BRep_Tool::IsClosed(forward,face.face)
                        ||!ENC483_CHECK(BRep_Tool::SameParameter(forward),"edge-same-parameter",report.arithmeticMagnitudeMM,report.maximumKernelToleranceMM,report.maximumLocationDepth,report.maximumLocationCompositionMagnitude)||!ENC483_CHECK(BRep_Tool::SameRange(forward),"edge-same-range",report.arithmeticMagnitudeMM,report.maximumKernelToleranceMM,report.maximumLocationDepth,report.maximumLocationCompositionMagnitude)||!ENC483_CHECK(Tolerance(BRep_Tool::Tolerance(forward),forward,mm,report),"edge-tolerance",report.arithmeticMagnitudeMM,report.maximumKernelToleranceMM,report.maximumLocationDepth,report.maximumLocationCompositionMagnitude))return ENC483_REFUSE("InspectEnclosure-return-16",refuse());
                    std::vector<TopoDS_Shape> vertices;if(!Children(forward,TopAbs_VERTEX,2,vertices,report)||vertices.size()!=2)return ENC483_REFUSE("InspectEnclosure-return-17",refuse());
                    for(const auto& v:vertices){if(v.Orientation()==TopAbs_FORWARD){if(!use.a.IsNull())return ENC483_REFUSE("InspectEnclosure-return-18",refuse());use.a=TopoDS::Vertex(v);}else{if(!use.b.IsNull())return ENC483_REFUSE("InspectEnclosure-return-19",refuse());use.b=TopoDS::Vertex(v);}}
                    if(!ENC483_CHECK(VertexPreflight(use.a,mm,report),"vertex-a-preflight",report.arithmeticMagnitudeMM,report.maximumKernelToleranceMM,report.maximumLocationDepth,report.maximumLocationCompositionMagnitude)||!ENC483_CHECK(VertexPreflight(use.b,mm,report),"vertex-b-preflight",report.arithmeticMagnitudeMM,report.maximumKernelToleranceMM,report.maximumLocationDepth,report.maximumLocationCompositionMagnitude)||!ENC483_CHECK(ReadCurve(forward,mm,report,use.curve),"curve-read",report.arithmeticMagnitudeMM,report.maximumKernelToleranceMM,report.maximumLocationDepth,report.maximumLocationCompositionMagnitude)||!ENC483_CHECK(PairLocation(use.curve.location,face.surface.location,report),"curve-surface-location",report.arithmeticMagnitudeMM,report.maximumKernelToleranceMM,report.maximumLocationDepth,report.maximumLocationCompositionMagnitude)||!ENC483_CHECK(ReadPCurve(forward,face.surface,use.curve,use.pc,report),"pcurve-read",report.arithmeticMagnitudeMM,report.maximumKernelToleranceMM,report.maximumLocationDepth,report.maximumLocationCompositionMagnitude)
                        ||!ENC483_CHECK(PCurveMagnitude(use.pc,face.surface,mm,report),"pcurve-magnitude",report.arithmeticMagnitudeMM,report.maximumKernelToleranceMM,report.maximumLocationDepth,report.maximumLocationCompositionMagnitude))return ENC483_REFUSE("InspectEnclosure-return-20",refuse());
                    if(!ENC483_CHECK(PairLocation(use.a.Location(),forward.Location(),report),"vertex-a-location",0,0,0,0)||!ENC483_CHECK(PairLocation(use.b.Location(),forward.Location(),report),"vertex-b-location",0,0,0,0)
                        ||!ENC483_EQUAL(BRep_Tool::Parameter(use.a,forward),use.curve.first,"vertex-first-parameter")||!ENC483_EQUAL(BRep_Tool::Parameter(use.b,forward),use.curve.last,"vertex-last-parameter"))return ENC483_REFUSE("InspectEnclosure-return-21",refuse());
                    uses.push_back(std::move(use));
                }face.wires.push_back(std::move(uses));
            }faces.push_back(std::move(face));
        }
        if(totalUses!=96||wireMap.Extent()!=20)return ENC483_REFUSE("InspectEnclosure-return-22",refuse());
        // Proposed numerical conditioning policy, NOT a universal proof of all
        // libm/kernel rounding. Location chains/powers and affine magnitudes are
        // bounded first; never grow this allowance during matching or from the
        // shape's kernel tolerances. Native adversarial qualification is pending.
        // Include affine composition intermediates even when a point is zero.
        if(!Track(report.maximumLocationCompositionMagnitude,mm,report))return ENC483_REFUSE("InspectEnclosure-return-23",refuse());
        report.comparisonErrorMM=std::max(1e-9,2048*std::numeric_limits<double>::epsilon()*report.arithmeticMagnitudeMM);
        if(!std::isfinite(report.comparisonErrorMM)||report.comparisonErrorMM>ConditioningLimitMM)return ENC483_REFUSE("InspectEnclosure-return-24",refuse());
        const double error=report.comparisonErrorMM;report.phase="boundary";
        report.minimumFeatureSeparationMM=std::numeric_limits<double>::infinity();
        for(unsigned i=0;i<32;++i)for(unsigned j=0;j<i;++j)report.minimumFeatureSeparationMM=std::min(report.minimumFeatureSeparationMM,expected.vertices[i].Distance(expected.vertices[j])*mm);
        // Also separate close nested walls/floor/radii, not just boundary vertices.
        report.minimumFeatureSeparationMM=std::min(report.minimumFeatureSeparationMM,std::min({dimensions.wall,dimensions.floor,dimensions.height-dimensions.floor,dimensions.cornerRadius-dimensions.wall,dimensions.width-2*dimensions.cornerRadius,dimensions.depth-2*dimensions.cornerRadius})*frame.ScaleFactor()*mm);
        if(!std::isfinite(report.minimumFeatureSeparationMM)||report.minimumFeatureSeparationMM<=2*error+4*report.maximumKernelToleranceMM)return ENC483_REFUSE("InspectEnclosure-return-25",refuse());
        TopTools_IndexedMapOfShape vertexMap,edgeMap;std::vector<unsigned> vertexIDs;
        std::array<bool,32> claimedVertices{};std::array<bool,48> claimedEdges{};std::array<bool,19> claimedFaces{};
        struct EdgeRecord {unsigned expected=0,a=0,b=0;TopoDS_Edge shape;unsigned plus=0,minus=0;std::vector<TopoDS_Face> faces;};std::vector<EdgeRecord> edgeRecords;
        auto vertex=[&](const TopoDS_Vertex& v,unsigned& id)->bool{
            const int prior=vertexMap.FindIndex(v);if(prior){id=vertexIDs[prior-1];return true;}
            if(vertexMap.Extent()>=32)return false;const auto p=BRep_Tool::Pnt(v);unsigned count=0,found=0;
            for(unsigned i=0;i<32;++i)if(Close(V(p),V(expected.vertices[i]),mm,error)){found=i;++count;}
            if(!ENC483_CHECK(count==1,"vertex-unique-match",count,p.X(),p.Y(),p.Z())||!ENC483_CHECK(!claimedVertices[found],"vertex-already-claimed",found,0,0,0))return false;claimedVertices[found]=true;vertexMap.Add(v);vertexIDs.push_back(found);id=found;return true;
        };
        for(const auto& face:faces){
            if(stop.load())return ENC483_REFUSE("InspectEnclosure-return-26",refuse());std::vector<std::vector<std::pair<unsigned,bool>>> actualWires;
            for(const auto& wire:face.wires){std::vector<std::pair<unsigned,bool>> actual;
                for(const auto& use:wire){
                    if(stop.load())return ENC483_REFUSE("InspectEnclosure-return-27",refuse());unsigned a=0,b=0;if(!vertex(use.a,a)||!vertex(use.b,b)||a==b)return ENC483_REFUSE("InspectEnclosure-return-28",refuse());
                    unsigned role=0,count=0;bool orientation=false;
                    for(unsigned i=0;i<48;++i){const auto& e=expected.edges[i];if((e.start==a&&e.end==b)||(e.start==b&&e.end==a)){role=i;orientation=e.start==a;++count;}}
                    if(!ENC483_CHECK(count==1,"edge-unique-match",count,a,b,0)||!ENC483_CHECK(MatchCurve(use.curve,expected.edges[role],expected,orientation,mm,error),"edge-curve-correspondence",role,error,use.curve.first,use.curve.last)||!ENC483_CHECK(PCurveIdentity(use.curve,use.pc,face.surface,mm,error),"edge-pcurve-correspondence",role,error,use.pc.first,use.pc.last))return ENC483_REFUSE("InspectEnclosure-return-29",refuse());
                    if(use.pc.stored)++report.storedPCurves;else ++report.generatedPCurves;
                    int prior=edgeMap.FindIndex(use.edge);if(!prior){
                        if(edgeMap.Extent()>=48||claimedEdges[role])return ENC483_REFUSE("InspectEnclosure-return-30",refuse());claimedEdges[role]=true;prior=edgeMap.Add(use.edge);
                        EdgeRecord e;e.expected=role;e.a=a;e.b=b;e.shape=TopoDS::Edge(use.edge.Oriented(TopAbs_FORWARD));edgeRecords.push_back(e);
                    }
                    auto& edge=edgeRecords[prior-1];if(edge.a!=a||edge.b!=b||edge.expected!=role||edge.faces.size()>=2)return ENC483_REFUSE("InspectEnclosure-return-31",refuse());
                    edge.faces.push_back(face.face);const bool useForward=use.edge.Orientation()==TopAbs_FORWARD;if(useForward)++edge.plus;else ++edge.minus;
                    actual.emplace_back(role,useForward==orientation);
                }actualWires.push_back(std::move(actual));
            }
            unsigned role=0,count=0;
            for(unsigned i=0;i<19;++i){const auto& wanted=expected.faces[i];if(wanted.wires.size()!=actualWires.size())continue;
                std::set<unsigned> matched;bool all=true;for(const auto& wire:actualWires){unsigned n=0,index=0;for(unsigned j=0;j<wanted.wires.size();++j)if(WireMatch(wire,wanted.wires[j])){index=j;++n;}if(n!=1||!matched.insert(index).second){all=false;break;}}
                if(all){role=i;++count;}
            }
            if(!ENC483_CHECK(count==1,"face-unique-match",count,role,0,0)||!ENC483_CHECK(!claimedFaces[role],"face-already-claimed",role,0,0,0)||!ENC483_CHECK(MatchSurface(face,expected.faces[role],mm,error),"face-support",role,error,Norm(face.surface.x),Norm(face.surface.y)))return ENC483_REFUSE("InspectEnclosure-return-32",refuse());claimedFaces[role]=true;
        }
        if(vertexMap.Extent()!=32||edgeMap.Extent()!=48||faceMap.Extent()!=19)return ENC483_REFUSE("InspectEnclosure-return-33",refuse());report.phase="representations";
        for(const auto& edge:edgeRecords){
            if(stop.load()||edge.plus!=1||edge.minus!=1||edge.faces.size()!=2)return ENC483_REFUSE("InspectEnclosure-return-34",refuse());
            const auto data=Handle(BRep_TEdge)::DownCast(edge.shape.TShape());unsigned curves=0;std::array<unsigned,2> pcurves{};
            for(BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());it.More();it.Next()){
                const auto& rep=it.Value();if(rep->IsCurve3D()){if(++curves!=1)return ENC483_REFUSE("InspectEnclosure-return-35",refuse());continue;}
                if(rep->IsCurveOnClosedSurface())return ENC483_REFUSE("InspectEnclosure-return-36",refuse());
                if(rep->IsCurveOnSurface()){unsigned matches=0;
                    for(unsigned j=0;j<2;++j){TopLoc_Location loc;const auto s=BRep_Tool::Surface(edge.faces[j],loc);
                        if(!PairLocation(loc,edge.shape.Location(),report))return ENC483_REFUSE("InspectEnclosure-return-37",refuse());
                        loc=loc.Predivided(edge.shape.Location());if(!Location(loc,report))return ENC483_REFUSE("InspectEnclosure-return-38",refuse());
                        if(rep->IsCurveOnSurface(s,loc)){++matches;if(++pcurves[j]>1)return ENC483_REFUSE("InspectEnclosure-return-39",refuse());}}
                    if(!ENC483_CHECK(matches==1,"pcurve-support-owners",matches,edge.expected,0,0))return ENC483_REFUSE("InspectEnclosure-return-40",refuse());
                }else if(rep->IsRegularity()){
                    TopLoc_Location a,b;const auto sa=BRep_Tool::Surface(edge.faces[0],a),sb=BRep_Tool::Surface(edge.faces[1],b);
                    if(!PairLocation(a,edge.shape.Location(),report)||!PairLocation(b,edge.shape.Location(),report))return ENC483_REFUSE("InspectEnclosure-return-41",refuse());
                    a=a.Predivided(edge.shape.Location());b=b.Predivided(edge.shape.Location());
                    if(!Location(a,report)||!Location(b,report))return ENC483_REFUSE("InspectEnclosure-return-42",refuse());
                    if(!ENC483_CHECK(rep->IsRegularity(sa,sb,a,b),"regularity-support-owners",edge.expected,0,0,0))return ENC483_REFUSE("InspectEnclosure-return-43",refuse());
                }else if(!rep->IsPolygon3D()&&!rep->IsPolygonOnTriangulation())return ENC483_REFUSE("InspectEnclosure-return-44",refuse());
            }if(curves!=1)return ENC483_REFUSE("InspectEnclosure-return-45",refuse());
        }
        // This classifier does not replace retained/native validity admission and
        // does not invoke a generic kernel analyzer over uninspected caches.
        // Every accepted boundary support, trim and incidence was checked above.
        if(stop.load())return ENC483_REFUSE("InspectEnclosure-return-46",refuse());report.vertices=32;report.edges=48;report.faces=19;report.wires=20;report.coedges=96;report.phase="matched";output=report;return Classification::MatchedBoundary;
    }catch(...){
#if DEBUG
        // The inner scope has restored any outer TLS before this catch.
        // Record into this result directly; never alter the prior inspection.
        if(!report.diagnostic483.recorded){report.diagnostic483.recorded=true;report.diagnostic483.branch="InspectEnclosure-exception";}
#endif
        return refuse();}
}
}

#undef ENC483_CHECK
#undef ENC483_EQUAL
#undef ENC483_REFUSE
