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
struct Inspection {
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
struct Curve {bool circle=false;gp_Vec c,a,b;TopLoc_Location location;double first=0,last=0;}; // line c+t*a; circle c+cos(t)*a+sin(t)*b
struct PCurve {bool circle=false;gp_Pnt2d c;gp_Vec2d a,b;double first=0,last=0;bool stored=false;};
struct Surface {bool cylinder=false;gp_Vec c,x,y,z,rawC,rawX,rawY,rawZ;TopLoc_Location location;Handle(Geom_Surface) handle;std::vector<std::array<double,4>> boxes;};
inline bool Range(double first,double last){return std::isfinite(first)&&std::isfinite(last)&&first<last;}
inline bool ReadCurve(const TopoDS_Edge& edge,double mm,Inspection& r,Curve& out){
    TopLoc_Location loc;double first=0,last=0;auto curve=BRep_Tool::Curve(edge,loc,first,last);
    if(!Range(first,last)||!Location(loc,r))return false;
    Handle(Geom_Line) line;Handle(Geom_Circle) circle;
    for(unsigned n=0;n<MaximumWrappers&&!curve.IsNull();++n){
        line=Handle(Geom_Line)::DownCast(curve);circle=Handle(Geom_Circle)::DownCast(curve);if(!line.IsNull()||!circle.IsNull())break;
        const auto trim=Handle(Geom_TrimmedCurve)::DownCast(curve);
        if(trim.IsNull()||first<trim->FirstParameter()||last>trim->LastParameter())return false;curve=trim->BasisCurve();
    }
    Curve c;c.location=loc;c.first=first;c.last=last;
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
    if(f!=c.first||l!=c.last||!Range(f,l))return false;PCurve pc;pc.first=f;pc.last=l;pc.stored=matches!=0;
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
        const auto trim=Handle(Geom2d_TrimmedCurve)::DownCast(curve);if(trim.IsNull()||f<trim->FirstParameter()||l>trim->LastParameter())return false;curve=trim->BasisCurve();
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
    std::array<double,4> bounds;if(!PCBounds(pc,bounds))return false;
    for(const auto& box:s.boxes)for(unsigned axis=0;axis<2;++axis)if(bounds[axis*2]<box[axis*2]||bounds[axis*2+1]>box[axis*2+1])return false;
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
            drift=(Norm(s.x)+Norm(s.y))*std::abs(rate-sign)*extent+Norm(s.z)*std::abs(pc.a.Y())*extent;
        }else{
            composed.c=s.c+radial+s.z*pc.c.Y();composed.a=s.z*pc.a.Y();
            drift=(Norm(s.x)+Norm(s.y))*std::abs(pc.a.X())*extent;
        }
    }
    const double residual=Norm(composed.c-c.c)+(c.circle?Norm(composed.a-c.a)+Norm(composed.b-c.b):extent*Norm(composed.a-c.a))+drift;
    return std::isfinite(residual*mm)&&residual*mm<=error;
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
    if(c.circle!=e.circle)return false;
    const auto start=V(expected.vertices[forward?e.start:e.end]),end=V(expected.vertices[forward?e.end:e.start]);
    if(!Close(Evaluate(c,c.first),start,mm,error)||!Close(Evaluate(c,c.last),end,mm,error))return false;
    if(!c.circle)return true; // analytic affine segment over complete finite range
    const double r=Norm(c.a),q=Norm(c.b);
    if(!Close(c.c,V(e.center),mm,error)||!std::isfinite(r)||r<=0||!std::isfinite(q)||q<=0
        ||std::abs(r-e.radius)*mm>error||std::abs(q-e.radius)*mm>error
        ||std::abs(c.a.Dot(c.b))/(r*q)*e.radius*mm>error
        ||std::abs((c.last-c.first)-std::acos(-1.0)/2)*e.radius*mm>error)return false;
    // Center plus non-collinear endpoints fixes the circle plane; quarter-range
    // excludes the opposite major arc and any extra periodic covering.
    const auto wanted=(start-V(e.center)).Crossed(end-V(e.center));const auto actual=c.a.Crossed(c.b);
    const double a=Norm(actual),w=Norm(wanted);if(a<=0||w<=0)return false;
    return Norm(actual/a-wanted/w)*e.radius*mm<=error;
}
inline bool WireMatch(const std::vector<std::pair<unsigned,bool>>& actual,const std::vector<EdgeUse>& expected){
    if(actual.size()!=expected.size())return false;
    std::set<std::pair<unsigned,bool>> a(actual.begin(),actual.end()),b;
    for(const auto& use:expected)b.emplace(use.edge,use.forward);
    return a.size()==actual.size()&&a==b;
}
inline bool MatchSurface(const Face& face,const ExpectedFace& e,double mm,double error){
    const auto& s=face.surface;if(s.cylinder!=e.cylinder)return false;
    const double side=face.face.Orientation()==TopAbs_FORWARD?1:-1;
    if(!s.cylinder){
        auto n=s.x.Crossed(s.y);const double length=Norm(n),wanted=Norm(e.normalOrAxis);if(length<=0||wanted<=0)return false;
        n*=side/length;
        // Parallel support and explicit signed normal; polygon/arc loops then
        // specify the complete finite trim, including the annular top face.
        return Norm(n-e.normalOrAxis/wanted)*std::max(Norm(s.x),Norm(s.y))*mm<=error
            &&std::abs((s.c-V(e.origin)).Dot(n))*mm<=error;
    }
    const double radius=Norm(s.x),other=Norm(s.y),axis=Norm(s.z),want=Norm(e.normalOrAxis);
    if(radius<=0||other<=0||axis<=0||want<=0||std::abs(radius-e.radius)*mm>error||std::abs(other-e.radius)*mm>error)return false;
    const auto unit=s.z/axis,wanted=e.normalOrAxis/want;
    if(Norm(unit.Crossed(wanted))*e.radius*mm>error)return false;
    const auto offset=s.c-V(e.origin);if(Norm(offset-unit*offset.Dot(unit))*mm>error)return false;
    const double handed=s.x.Crossed(s.y).Dot(s.z);if(!std::isfinite(handed)||handed==0)return false;
    return (handed>0?1:-1)*side==e.radialSign;
}
}
inline Classification InspectEnclosure(const TopoDS_Shape& retained,const enclosure::Parameters& parameters,
    const std::atomic_bool& stop,Inspection& output)noexcept{
    using namespace detail;output={};Inspection report;
    const auto refuse=[&](){output=report;return stop.load()?Classification::Cancelled:Classification::Refused;};
    try{
        if(stop.load())return refuse();ExpectedBoundary expected;
        std::vector<double> encoded;if(!enclosure::Encode(parameters,encoded)||!BuildExpectedBoundary(parameters.definition,expected))return refuse();
        const double mm=parameters.metersPerUnit*1000;if(!std::isfinite(mm)||mm<=0)return refuse();
        // Source frame has eight bounded scalar inputs, with proper quaternion
        // and positive scale; its conversion is fixed-size, never a datum chain.
        gp_Trsf frame;if(parameters.definition.constructionFrame&&!parameters.definition.constructionFrame->Transform(frame))return refuse();
        if(!Matrix(frame))return refuse();
        const auto& dimensions=parameters.definition.dimensions;
        std::array<double,3> rawSums{dimensions.width+dimensions.depth+dimensions.height,dimensions.width+dimensions.depth+dimensions.height,dimensions.width+dimensions.depth+dimensions.height};
        if(!AffineMagnitude(rawSums,frame,false,mm,report))return refuse();
        for(const auto& v:expected.vertices)if(!Track(std::abs(v.X())+std::abs(v.Y())+std::abs(v.Z()),mm,report))return refuse();
        if(retained.IsNull()||retained.ShapeType()!=TopAbs_SOLID||retained.Orientation()!=TopAbs_FORWARD||!Location(retained.Location(),report))return refuse();
        report.phase="collect";std::vector<TopoDS_Shape> shells,rawFaces;
        if(!Children(retained,TopAbs_SHELL,1,shells,report)||shells.size()!=1||!Children(shells[0],TopAbs_FACE,19,rawFaces,report)||rawFaces.size()!=19)return refuse();
        std::vector<Face> faces;unsigned totalUses=0;TopTools_IndexedMapOfShape faceMap,wireMap;
        for(const auto& rawFace:rawFaces){
            if(stop.load())return refuse();Face face;face.face=TopoDS::Face(rawFace);
            if(faceMap.Contains(face.face)||!SurfacePreflight(face.face,report)||!Tolerance(BRep_Tool::Tolerance(face.face),face.face,mm,report)||!ReadSurface(face.face,mm,report,face.surface))return refuse();faceMap.Add(face.face);
            std::vector<TopoDS_Shape> wires;if(!Children(face.face,TopAbs_WIRE,2,wires,report)||wires.empty())return refuse();
            for(const auto& wire:wires){
                if(wireMap.Contains(wire))return refuse();wireMap.Add(wire);std::vector<TopoDS_Shape> edges;
                if(!Children(wire,TopAbs_EDGE,8,edges,report)||edges.size()<4)return refuse();std::vector<Use> uses;
                for(const auto& rawEdge:edges){
                    if(stop.load()||++totalUses>96)return refuse();Use use;use.edge=TopoDS::Edge(rawEdge);
                    const auto forward=TopoDS::Edge(use.edge.Oriented(TopAbs_FORWARD));
                    if(!EdgeRepresentations(forward,report)||!PairLocation(face.surface.location,forward.Location(),report)||BRep_Tool::Degenerated(forward)||BRep_Tool::IsClosed(forward,face.face)
                        ||!BRep_Tool::SameParameter(forward)||!BRep_Tool::SameRange(forward)||!Tolerance(BRep_Tool::Tolerance(forward),forward,mm,report))return refuse();
                    std::vector<TopoDS_Shape> vertices;if(!Children(forward,TopAbs_VERTEX,2,vertices,report)||vertices.size()!=2)return refuse();
                    for(const auto& v:vertices){if(v.Orientation()==TopAbs_FORWARD){if(!use.a.IsNull())return refuse();use.a=TopoDS::Vertex(v);}else{if(!use.b.IsNull())return refuse();use.b=TopoDS::Vertex(v);}}
                    if(!VertexPreflight(use.a,mm,report)||!VertexPreflight(use.b,mm,report)||!ReadCurve(forward,mm,report,use.curve)||!PairLocation(use.curve.location,face.surface.location,report)||!ReadPCurve(forward,face.surface,use.curve,use.pc,report)
                        ||!PCurveMagnitude(use.pc,face.surface,mm,report))return refuse();
                    if(!PairLocation(use.a.Location(),forward.Location(),report)||!PairLocation(use.b.Location(),forward.Location(),report)
                        ||BRep_Tool::Parameter(use.a,forward)!=use.curve.first||BRep_Tool::Parameter(use.b,forward)!=use.curve.last)return refuse();
                    uses.push_back(std::move(use));
                }face.wires.push_back(std::move(uses));
            }faces.push_back(std::move(face));
        }
        if(totalUses!=96||wireMap.Extent()!=20)return refuse();
        // Proposed numerical conditioning policy, NOT a universal proof of all
        // libm/kernel rounding. Location chains/powers and affine magnitudes are
        // bounded first; never grow this allowance during matching or from the
        // shape's kernel tolerances. Native adversarial qualification is pending.
        // Include affine composition intermediates even when a point is zero.
        if(!Track(report.maximumLocationCompositionMagnitude,mm,report))return refuse();
        report.comparisonErrorMM=std::max(1e-9,2048*std::numeric_limits<double>::epsilon()*report.arithmeticMagnitudeMM);
        if(!std::isfinite(report.comparisonErrorMM)||report.comparisonErrorMM>ConditioningLimitMM)return refuse();
        const double error=report.comparisonErrorMM;report.phase="boundary";
        report.minimumFeatureSeparationMM=std::numeric_limits<double>::infinity();
        for(unsigned i=0;i<32;++i)for(unsigned j=0;j<i;++j)report.minimumFeatureSeparationMM=std::min(report.minimumFeatureSeparationMM,expected.vertices[i].Distance(expected.vertices[j])*mm);
        // Also separate close nested walls/floor/radii, not just boundary vertices.
        report.minimumFeatureSeparationMM=std::min(report.minimumFeatureSeparationMM,std::min({dimensions.wall,dimensions.floor,dimensions.height-dimensions.floor,dimensions.cornerRadius-dimensions.wall,dimensions.width-2*dimensions.cornerRadius,dimensions.depth-2*dimensions.cornerRadius})*frame.ScaleFactor()*mm);
        if(!std::isfinite(report.minimumFeatureSeparationMM)||report.minimumFeatureSeparationMM<=2*error+4*report.maximumKernelToleranceMM)return refuse();
        TopTools_IndexedMapOfShape vertexMap,edgeMap;std::vector<unsigned> vertexIDs;
        std::array<bool,32> claimedVertices{};std::array<bool,48> claimedEdges{};std::array<bool,19> claimedFaces{};
        struct EdgeRecord {unsigned expected=0,a=0,b=0;TopoDS_Edge shape;unsigned plus=0,minus=0;std::vector<TopoDS_Face> faces;};std::vector<EdgeRecord> edgeRecords;
        auto vertex=[&](const TopoDS_Vertex& v,unsigned& id)->bool{
            const int prior=vertexMap.FindIndex(v);if(prior){id=vertexIDs[prior-1];return true;}
            if(vertexMap.Extent()>=32)return false;const auto p=BRep_Tool::Pnt(v);unsigned count=0,found=0;
            for(unsigned i=0;i<32;++i)if(Close(V(p),V(expected.vertices[i]),mm,error)){found=i;++count;}
            if(count!=1||claimedVertices[found])return false;claimedVertices[found]=true;vertexMap.Add(v);vertexIDs.push_back(found);id=found;return true;
        };
        for(const auto& face:faces){
            if(stop.load())return refuse();std::vector<std::vector<std::pair<unsigned,bool>>> actualWires;
            for(const auto& wire:face.wires){std::vector<std::pair<unsigned,bool>> actual;
                for(const auto& use:wire){
                    if(stop.load())return refuse();unsigned a=0,b=0;if(!vertex(use.a,a)||!vertex(use.b,b)||a==b)return refuse();
                    unsigned role=0,count=0;bool orientation=false;
                    for(unsigned i=0;i<48;++i){const auto& e=expected.edges[i];if((e.start==a&&e.end==b)||(e.start==b&&e.end==a)){role=i;orientation=e.start==a;++count;}}
                    if(count!=1||!MatchCurve(use.curve,expected.edges[role],expected,orientation,mm,error)||!PCurveIdentity(use.curve,use.pc,face.surface,mm,error))return refuse();
                    if(use.pc.stored)++report.storedPCurves;else ++report.generatedPCurves;
                    int prior=edgeMap.FindIndex(use.edge);if(!prior){
                        if(edgeMap.Extent()>=48||claimedEdges[role])return refuse();claimedEdges[role]=true;prior=edgeMap.Add(use.edge);
                        EdgeRecord e;e.expected=role;e.a=a;e.b=b;e.shape=TopoDS::Edge(use.edge.Oriented(TopAbs_FORWARD));edgeRecords.push_back(e);
                    }
                    auto& edge=edgeRecords[prior-1];if(edge.a!=a||edge.b!=b||edge.expected!=role||edge.faces.size()>=2)return refuse();
                    edge.faces.push_back(face.face);const bool useForward=use.edge.Orientation()==TopAbs_FORWARD;if(useForward)++edge.plus;else ++edge.minus;
                    actual.emplace_back(role,useForward==orientation);
                }actualWires.push_back(std::move(actual));
            }
            unsigned role=0,count=0;
            for(unsigned i=0;i<19;++i){const auto& wanted=expected.faces[i];if(wanted.wires.size()!=actualWires.size())continue;
                std::set<unsigned> matched;bool all=true;for(const auto& wire:actualWires){unsigned n=0,index=0;for(unsigned j=0;j<wanted.wires.size();++j)if(WireMatch(wire,wanted.wires[j])){index=j;++n;}if(n!=1||!matched.insert(index).second){all=false;break;}}
                if(all){role=i;++count;}
            }
            if(count!=1||claimedFaces[role]||!MatchSurface(face,expected.faces[role],mm,error))return refuse();claimedFaces[role]=true;
        }
        if(vertexMap.Extent()!=32||edgeMap.Extent()!=48||faceMap.Extent()!=19)return refuse();report.phase="representations";
        for(const auto& edge:edgeRecords){
            if(stop.load()||edge.plus!=1||edge.minus!=1||edge.faces.size()!=2)return refuse();
            const auto data=Handle(BRep_TEdge)::DownCast(edge.shape.TShape());unsigned curves=0;std::array<unsigned,2> pcurves{};
            for(BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());it.More();it.Next()){
                const auto& rep=it.Value();if(rep->IsCurve3D()){if(++curves!=1)return refuse();continue;}
                if(rep->IsCurveOnClosedSurface())return refuse();
                if(rep->IsCurveOnSurface()){unsigned matches=0;
                    for(unsigned j=0;j<2;++j){TopLoc_Location loc;const auto s=BRep_Tool::Surface(edge.faces[j],loc);
                        if(!PairLocation(loc,edge.shape.Location(),report))return refuse();
                        loc=loc.Predivided(edge.shape.Location());if(!Location(loc,report))return refuse();
                        if(rep->IsCurveOnSurface(s,loc)){++matches;if(++pcurves[j]>1)return refuse();}}
                    if(matches!=1)return refuse();
                }else if(rep->IsRegularity()){
                    TopLoc_Location a,b;const auto sa=BRep_Tool::Surface(edge.faces[0],a),sb=BRep_Tool::Surface(edge.faces[1],b);
                    if(!PairLocation(a,edge.shape.Location(),report)||!PairLocation(b,edge.shape.Location(),report))return refuse();
                    a=a.Predivided(edge.shape.Location());b=b.Predivided(edge.shape.Location());
                    if(!Location(a,report)||!Location(b,report))return refuse();
                    if(!rep->IsRegularity(sa,sb,a,b))return refuse();
                }else if(!rep->IsPolygon3D()&&!rep->IsPolygonOnTriangulation())return refuse();
            }if(curves!=1)return refuse();
        }
        // This classifier does not replace retained/native validity admission and
        // does not invoke a generic kernel analyzer over uninspected caches.
        // Every accepted boundary support, trim and incidence was checked above.
        if(stop.load())return refuse();report.vertices=32;report.edges=48;report.faces=19;report.wires=20;report.coedges=96;report.phase="matched";output=report;return Classification::MatchedBoundary;
    }catch(...){return refuse();}
}
}
