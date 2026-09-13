#pragma once
// Detached engineering observation only. There is deliberately NO MatchedResult
// status or permission bit. Exterior cap/face correspondence remains unproved.
#include "SavedCutEnclosureExtractor.hxx" // exact frozen dependency, not modified
#include "SavedCutSourceBoreClearance.hxx"
#include <BRep_CurveOnClosedSurface.hxx>
#include <cstring>
#include <map>

namespace core3d::saved_cut_bore_result {
namespace d=core3d::enclosure_correspondence::detail;
using Budget=core3d::enclosure_correspondence::Inspection;
enum class Status { Refused, Cancelled, BoreWallObservedExteriorUnproven };
struct Report {
    Status status=Status::Refused;const char* phase="input";
    std::size_t faces=0,uniqueEdges=0,uniqueVertices=0,coedges=0;
    double errorMM=0,kernelToleranceMM=0,radiusMM=0;
    std::array<double,2> axisIntervalOriginalMM{};
    // Topological cycles/whole analytic wall, NOT exterior/cap correspondence.
    unsigned circularOpeningCycles=0,seamUses=0;
};
namespace detail {
inline bool Bits(double a,double b){return retained_solid::Bits(a)==retained_solid::Bits(b);}
inline bool FixedTool(const retained_solid::Envelope& a,const retained_solid::Envelope& b){
    if(!retained_solid::Valid(a)||!retained_solid::Valid(b)||a.document!=b.document||a.entity!=b.entity
        ||a.definition!=b.definition||a.sourceFeature!=b.sourceFeature||a.derivedFeature!=b.derivedFeature
        ||a.sourceFamily!=b.sourceFamily||a.sourceSchema!=b.sourceSchema||a.operandID!=b.operandID
        ||a.axis!=b.axis||!Bits(a.radius,b.radius)||!Bits(a.metersPerUnit,b.metersPerUnit))return false;
    for(unsigned i=0;i<3;++i)if(!Bits(a.point[i],b.point[i]))return false;return true;
}
inline bool Frame(const retained_solid::Envelope& e,std::optional<profile::ConstructionFrame>& frame,int& plane,double& wall){
    if(e.sourceFamily==1){profile::Parameters p;if(!profile::Decode(e.sourceValues,p))return false;
        frame=p.constructionFrame;plane=p.definition.plane;wall=p.definition.depth;return true;}
    if(e.sourceFamily==2){enclosure::Parameters p;if(!enclosure::Decode(int(e.sourceSchema),e.sourceValues,p))return false;
        frame=p.definition.constructionFrame;plane=p.definition.plane;wall=p.definition.dimensions.floor;return true;}
    return false;
}
inline bool SameFrame(const std::optional<profile::ConstructionFrame>& a,const std::optional<profile::ConstructionFrame>& b){
    if(bool(a)!=bool(b))return false;if(a)for(unsigned i=0;i<8;++i)if(!Bits(a->values[i],b->values[i]))return false;return true;
}
// Same bounded stored analytic extraction as the frozen enclosure reader, but
// a FULL circle is necessary here. No old reader's range policy is widened.
inline bool Curve(const TopoDS_Edge& edge,double mm,Budget& budget,d::Curve& out){
    TopLoc_Location loc;double first=0,last=0;auto curve=BRep_Tool::Curve(edge,loc,first,last);
    if(!d::Range(first,last)||!d::Location(loc,budget))return false;
    d::Curve c;Handle(Geom_Line) line;Handle(Geom_Circle) circle;
    for(unsigned n=0;n<d::MaximumWrappers&&!curve.IsNull();++n){
        line=Handle(Geom_Line)::DownCast(curve);circle=Handle(Geom_Circle)::DownCast(curve);
        if(!line.IsNull()||!circle.IsNull())break;
        const auto trim=Handle(Geom_TrimmedCurve)::DownCast(curve);
        if(trim.IsNull()||!d::RetainTrim(c.trims,first,last,trim->FirstParameter(),trim->LastParameter()))return false;curve=trim->BasisCurve();
    }
    c.first=first;c.last=last;c.location=loc;
    if(!line.IsNull()){const auto l=line->Lin();c.c=d::V(l.Location());c.a=gp_Vec(l.Direction());}
    else if(!circle.IsNull()){
        const double pi=std::acos(-1.0);
        if(std::max(std::abs(first),std::abs(last))>32*pi||last-first>2*pi+64*std::numeric_limits<double>::epsilon())return false;
        const auto x=circle->Circ();c.circle=true;c.c=d::V(x.Location());c.a=gp_Vec(x.Position().XDirection())*x.Radius();c.b=gp_Vec(x.Position().YDirection())*x.Radius();
    }else return false;
    const double extent=c.circle?1:std::max(std::abs(first),std::abs(last));
    const gp_Vec magnitude(std::abs(c.c.X())+extent*std::abs(c.a.X())+std::abs(c.b.X()),std::abs(c.c.Y())+extent*std::abs(c.a.Y())+std::abs(c.b.Y()),std::abs(c.c.Z())+extent*std::abs(c.a.Z())+std::abs(c.b.Z()));
    if(!d::LocatedMagnitude(magnitude,loc,false,mm,budget))return false;
    const auto tr=loc.Transformation();auto p=d::P(c.c);p.Transform(tr);c.c=d::V(p);c.a.Transform(tr);c.b.Transform(tr);
    if(!d::Finite(c.c)||!d::Finite(c.a)||!d::Finite(c.b))return false;out=c;return true;
}
struct Edge {
    TopoDS_Edge shape;d::Curve curve;std::array<TopoDS_Vertex,2> vertices;
    unsigned plus=0,minus=0;std::vector<unsigned> faceIDs;
};
struct Use {unsigned edge=0;bool forward=false;};
struct Face {TopoDS_Face shape;d::Surface surface;std::vector<std::vector<Use>> wires;};
// Inspect EVERY internal representation's raw location before any relative
// product or BRep_Tool call. Closed-surface pairs are retained explicitly.
inline bool Representations(const TopoDS_Edge& edge,Budget& b){
    const auto data=Handle(BRep_TEdge)::DownCast(edge.TShape());if(data.IsNull())return false;
    unsigned count=0,curves=0;
    for(BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());it.More();it.Next()){
        const auto& r=it.Value();if(++count>16||r.IsNull()||!d::PairLocation(edge.Location(),r->Location(),b))return false;
        if(r->IsCurve3D()){if(++curves>1)return false;}
        else if(r->IsCurveOnSurface()){if(r->PCurve().IsNull()||(r->IsCurveOnClosedSurface()&&r->PCurve2().IsNull()))return false;}
        else if(r->IsRegularity()){if(!d::PairLocation(edge.Location(),r->Location2(),b))return false;}
        else if(!r->IsPolygon3D()&&!r->IsPolygonOnTriangulation())return false;
    }return curves==1;
}
inline bool PCurves(const Edge& edge,const Face& face,double mm,Budget& b,std::vector<d::PCurve>& out){
    out.clear();if(!d::PairLocation(face.surface.location,edge.shape.Location(),b))return false;
    const auto relative=face.surface.location.Predivided(edge.shape.Location());if(!d::Location(relative,b))return false;
    const auto data=Handle(BRep_TEdge)::DownCast(edge.shape.TShape());unsigned found=0;
    for(BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());it.More();it.Next()){
        const auto& rep=it.Value();if(!rep->IsCurveOnSurface(face.surface.handle,relative))continue;
        const auto gc=Handle(BRep_GCurve)::DownCast(rep);if(++found!=1||gc.IsNull())return false;
        double first=0,last=0;gc->Range(first,last);if(!Bits(first,edge.curve.first)||!Bits(last,edge.curve.last))return false;
        for(unsigned k=0;k<(rep->IsCurveOnClosedSurface()?2u:1u);++k){
            d::PCurve value;auto pc=k?rep->PCurve2():rep->PCurve();Handle(Geom2d_Line) line;
            for(unsigned n=0;n<8&&!pc.IsNull();++n){line=Handle(Geom2d_Line)::DownCast(pc);if(!line.IsNull())break;
                const auto trim=Handle(Geom2d_TrimmedCurve)::DownCast(pc);if(trim.IsNull()||!d::RetainTrim(value.trims,first,last,trim->FirstParameter(),trim->LastParameter()))return false;pc=trim->BasisCurve();}
            if(line.IsNull())return false;value.first=first;value.last=last;value.stored=true;
            value.c=line->Location();value.a=gp_Vec2d(line->Direction());
            if(!d::PCurveMagnitude(value,face.surface,mm,b))return false;out.push_back(value);
        }
    }return found==1;
}
inline gp_Pnt2d At(const d::PCurve& p,double t){return {p.c.X()+t*p.a.X(),p.c.Y()+t*p.a.Y()};}
inline bool Close2(const gp_Pnt2d& a,const gp_Pnt2d& b,double uScale,double vScale,double error){
    return std::isfinite(a.X())&&std::isfinite(a.Y())&&std::isfinite(b.X())&&std::isfinite(b.Y())
        &&std::abs(a.X()-b.X())*uScale<=error&&std::abs(a.Y()-b.Y())*vScale<=error;
}
}
inline Report Inspect(const TopoDS_Shape& result,const retained_solid::Envelope& oldSource,
    const retained_solid::Envelope& newSource,const std::atomic_bool& stop) noexcept {
    using namespace detail;Report report;Budget budget;
    const auto fail=[&](){report.status=stop.load()?Status::Cancelled:Status::Refused;return report;};
    try {
        if(stop.load()||!FixedTool(oldSource,newSource))return fail();
        const auto oldClear=saved_cut_bore_clearance::Inspect(oldSource),newClear=saved_cut_bore_clearance::Inspect(newSource);
        if(oldClear.status!=saved_cut_bore_clearance::Status::ClearRecipeDisk||newClear.status!=saved_cut_bore_clearance::Status::ClearRecipeDisk)return fail();
        std::optional<profile::ConstructionFrame> oldFrame,frame;int oldPlane=-1,plane=-1;double oldWall=0,wall=0;
        if(!Frame(oldSource,oldFrame,oldPlane,oldWall)||!Frame(newSource,frame,plane,wall)||oldPlane!=plane||!SameFrame(oldFrame,frame))return fail();
        const double mm=newSource.metersPerUnit*1000,pi=std::acos(-1.0);gp_Trsf transform;
        if(frame&&!frame->Transform(transform))return fail();
        const auto sourceVector=enclosure_correspondence::PlaneVector(0,0,1,plane).Transformed(transform);
        const auto p0=enclosure_correspondence::PlanePoint(0,0,0,plane).Transformed(transform);
        const auto p1=enclosure_correspondence::PlanePoint(0,0,wall,plane).Transformed(transform);
        const gp_Vec axis=newSource.axis==0?gp_Vec(1,0,0):newSource.axis==1?gp_Vec(0,1,0):gp_Vec(0,0,1);
        const gp_Vec tool(newSource.point[0],newSource.point[1],newSource.point[2]);
        const double denominator=sourceVector.Dot(axis);if(!std::isfinite(denominator)||denominator==0)return fail();
        const double t0=(d::V(p0)-tool).Dot(sourceVector)/denominator,t1=(d::V(p1)-tool).Dot(sourceVector)/denominator;
        const double low=std::min(t0,t1),high=std::max(t0,t1);
        if(!std::isfinite(low)||!std::isfinite(high)||low>=high||!d::Matrix(transform))return fail();
        std::array<double,3> frameMagnitude{std::abs(wall),std::abs(wall),std::abs(wall)};
        if(!d::AffineMagnitude(frameMagnitude,transform,false,mm,budget)
            ||!d::Track(d::Norm(tool)+std::abs(low)+std::abs(high)+newSource.radius,mm,budget))return fail();
        report.phase="collect";
        if(result.IsNull()||result.ShapeType()!=TopAbs_SOLID||result.Orientation()!=TopAbs_FORWARD)return fail();
        std::vector<TopoDS_Shape> shells,rawFaces;
        if(!d::Children(result,TopAbs_SHELL,1,shells,budget)||shells.size()!=1
            ||!d::Children(shells[0],TopAbs_FACE,130,rawFaces,budget)||rawFaces.empty())return fail();
        std::vector<Face> faces;std::vector<Edge> edges;TopTools_IndexedMapOfShape faceMap,wireMap,edgeMap,vertexMap;
        for(const auto& rawFace:rawFaces){
            if(stop.load())return fail();Face face;face.shape=TopoDS::Face(rawFace);
            if(faceMap.Contains(face.shape)||!d::SurfacePreflight(face.shape,budget)
                ||!d::Tolerance(BRep_Tool::Tolerance(face.shape),face.shape,mm,budget)
                ||!d::ReadSurface(face.shape,mm,budget,face.surface))return fail();faceMap.Add(face.shape);
            std::vector<TopoDS_Shape> rawWires;if(!d::Children(face.shape,TopAbs_WIRE,4,rawWires,budget)||rawWires.empty())return fail();
            for(const auto& rawWire:rawWires){
                if(wireMap.Contains(rawWire))return fail();wireMap.Add(rawWire);
                std::vector<TopoDS_Shape> rawEdges;if(!d::Children(rawWire,TopAbs_EDGE,128,rawEdges,budget)||rawEdges.empty())return fail();
                std::vector<Use> uses;
                for(const auto& rawEdge:rawEdges){
                    if(stop.load()||++report.coedges>2048)return fail();
                    const auto edge=TopoDS::Edge(rawEdge.Oriented(TopAbs_FORWARD));int id=edgeMap.FindIndex(edge);
                    if(!id){
                        if(edges.size()>=1024||!Representations(edge,budget)||!BRep_Tool::SameParameter(edge)||!BRep_Tool::SameRange(edge)
                            ||BRep_Tool::Degenerated(edge)||!d::Tolerance(BRep_Tool::Tolerance(edge),edge,mm,budget))return fail();
                        Edge saved;saved.shape=edge;if(!Curve(edge,mm,budget,saved.curve))return fail();
                        std::vector<TopoDS_Shape> vertices;if(!d::Children(edge,TopAbs_VERTEX,2,vertices,budget)||vertices.size()!=2)return fail();
                        std::array<bool,2> seen{};for(const auto& v:vertices){
                            const unsigned n=v.Orientation()==TopAbs_FORWARD?0:1;if(seen[n])return fail();seen[n]=true;
                            saved.vertices[n]=TopoDS::Vertex(v);if(!d::VertexPreflight(saved.vertices[n],mm,budget))return fail();vertexMap.Add(v);
                        }
                        if(!seen[0]||!seen[1])return fail();id=edgeMap.Add(edge);edges.push_back(std::move(saved));
                    }
                    auto& saved=edges[id-1];if(saved.faceIDs.size()>=2)return fail();saved.faceIDs.push_back(unsigned(faces.size()));
                    const bool forward=rawEdge.Orientation()==TopAbs_FORWARD;if(forward)++saved.plus;else ++saved.minus;
                    uses.push_back({unsigned(id-1),forward});
                }face.wires.push_back(std::move(uses));
            }faces.push_back(std::move(face));
        }
        for(const auto& edge:edges)if(edge.plus!=1||edge.minus!=1||edge.faceIDs.size()!=2)return fail();
        // Read both seam pcurves and ALL cylindrical-face pcurves before fixing
        // the arithmetic allowance. Never grow it in response to a mismatch.
        std::map<std::pair<unsigned,unsigned>,std::vector<d::PCurve>> pcs;
        for(unsigned f=0;f<faces.size();++f)if(faces[f].surface.cylinder)
            for(const auto& wire:faces[f].wires)for(const auto& use:wire){
                if(stop.load())return fail();const auto key=std::make_pair(f,use.edge);if(pcs.count(key))continue;
                std::vector<d::PCurve> value;if(!PCurves(edges[use.edge],faces[f],mm,budget,value))return fail();pcs.emplace(key,std::move(value));
            }
        if(!d::Track(budget.maximumLocationCompositionMagnitude,mm,budget))return fail();
        const double error=std::max(1e-9,2048*std::numeric_limits<double>::epsilon()*budget.arithmeticMagnitudeMM);
        if(!std::isfinite(error)||error>1e-6)return fail();report.errorMM=error;report.kernelToleranceMM=budget.maximumKernelToleranceMM;
        report.faces=faces.size();report.uniqueEdges=edges.size();report.uniqueVertices=vertexMap.Extent();
        report.phase="curve-trim-domains";
        for(const auto& edge:edges){double charge=0;if(!d::CurveTrimCharge(edge.curve,mm,charge)||charge>error)return fail();}
        report.phase="bore-support";
        if(d::Norm(sourceVector.Crossed(axis))/d::Norm(sourceVector)*(high-low)*mm>error)return fail();
        unsigned bore=0,found=0;
        for(unsigned f=0;f<faces.size();++f){const auto& face=faces[f];const auto& s=face.surface;if(!s.cylinder)continue;
            const double radius=d::Norm(s.x),ys=d::Norm(s.y),zs=d::Norm(s.z);if(radius<=0||ys<=0||zs<=0)return fail();
            const auto direction=s.z/zs,delta=s.c-tool;
            if(std::abs(radius-newSource.radius)*mm>error||std::abs(ys-newSource.radius)*mm>error
                ||d::Norm(direction.Crossed(axis))*newSource.radius*mm>error
                ||d::Norm(delta-axis*delta.Dot(axis))*mm>error)continue;
            const double sign=s.x.Crossed(s.y).Dot(s.z)*(face.shape.Orientation()==TopAbs_FORWARD?1:-1);
            if(!std::isfinite(sign)||sign>=0)return fail();bore=f;++found;
        }
        if(found!=1)return fail();const auto& wallFace=faces[bore];const auto& surface=wallFace.surface;
        if(wallFace.wires.size()!=1||wallFace.wires[0].size()!=4)return fail();
        report.phase="bore-trims";
        std::map<unsigned,unsigned> multiplicity;for(const auto& use:wallFace.wires[0])++multiplicity[use.edge];
        if(multiplicity.size()!=3)return fail();unsigned seam=0,seams=0;std::vector<unsigned> openings;
        for(const auto& pair:multiplicity){const auto& edge=edges[pair.first];
            if(pair.second==2&&!edge.curve.circle){seam=pair.first;++seams;}
            else if(pair.second==1&&edge.curve.circle)openings.push_back(pair.first);else return fail();
        }if(seams!=1||openings.size()!=2)return fail();
        const auto& seamEdge=edges[seam];if(seamEdge.vertices[0].IsSame(seamEdge.vertices[1])
            ||seamEdge.faceIDs[0]!=bore||seamEdge.faceIDs[1]!=bore||pcs.at({bore,seam}).size()!=2)return fail();
        report.phase="bore-seam-endpoints";
        std::array<unsigned,2> seamLevel{};std::array<bool,2> seamLevelsSeen{};
        for(unsigned i=0;i<2;++i){
            const auto& vertex=seamEdge.vertices[i];const double parameter=i?seamEdge.curve.last:seamEdge.curve.first;
            if(!Bits(BRep_Tool::Parameter(vertex,seamEdge.shape),parameter)
                ||!d::Close(d::Evaluate(seamEdge.curve,parameter),d::V(BRep_Tool::Pnt(vertex)),mm,error))return fail();
            const double t=(d::V(BRep_Tool::Pnt(vertex))-tool).Dot(axis);unsigned count=0,level=0;
            if(std::abs(t-low)*mm<=error){level=0;++count;}
            if(std::abs(t-high)*mm<=error){level=1;++count;}
            if(count!=1||seamLevelsSeen[level])return fail();seamLevelsSeen[level]=true;seamLevel[i]=level;
        }
        report.phase="bore-trims";
        std::array<bool,2> levels{};std::array<unsigned,2> openingUsesOfSeamVertex{};std::set<unsigned> capFaces;
        for(unsigned id:openings){const auto& edge=edges[id];const auto& c=edge.curve;
            if(!edge.vertices[0].IsSame(edge.vertices[1])||pcs.at({bore,id}).size()!=1
                ||std::abs((c.last-c.first)-2*pi)*newSource.radius*mm>error)return fail();
            const double t=(c.c-tool).Dot(axis);const unsigned level=std::abs(t-low)<std::abs(t-high)?0:1;
            if(levels[level]||std::abs(t-(level?high:low))*mm>error||d::Norm(c.c-tool-axis*t)*mm>error)return fail();levels[level]=true;
            if(std::abs(d::Norm(c.a)-newSource.radius)*mm>error||std::abs(d::Norm(c.b)-newSource.radius)*mm>error
                ||std::abs(c.a.Dot(axis))*mm>error||std::abs(c.b.Dot(axis))*mm>error)return fail();
            report.phase="bore-vertex-sharing";
            unsigned matches=0,seamIndex=0;
            for(unsigned i=0;i<2;++i)if(edge.vertices[0].IsSame(seamEdge.vertices[i])){++matches;seamIndex=i;}
            if(matches!=1||seamLevel[seamIndex]!=level||++openingUsesOfSeamVertex[seamIndex]!=1)return fail();
            report.phase="bore-trims";
            if(!d::Close(d::Evaluate(c,c.first),d::V(BRep_Tool::Pnt(edge.vertices[0])),mm,error)
                ||!d::Close(d::Evaluate(c,c.last),d::V(BRep_Tool::Pnt(edge.vertices[1])),mm,error))return fail();
            const unsigned cap=edge.faceIDs[0]==bore?edge.faceIDs[1]:edge.faceIDs[0];if(cap==bore||!capFaces.insert(cap).second||faces[cap].surface.cylinder)return fail();
            unsigned completeCycles=0;for(const auto& wire:faces[cap].wires)if(wire.size()==1&&wire[0].edge==id)++completeCycles;
            if(completeCycles!=1)return fail();
            auto n=faces[cap].surface.x.Crossed(faces[cap].surface.y);const double len=d::Norm(n);if(len<=0)return fail();
            n*=((faces[cap].shape.Orientation()==TopAbs_FORWARD?1:-1)/len);
            if(d::Norm(n-axis*(level?1:-1))*newSource.radius*mm>error
                ||std::abs((faces[cap].surface.c-c.c).Dot(n))*mm>error)return fail();
        }
        if(!levels[0]||!levels[1]||openingUsesOfSeamVertex[0]!=1||openingUsesOfSeamVertex[1]!=1)return fail();
        // Coefficient identity proves every stored pcurve over its entire trim,
        // including both seam images. No generated projection is introduced.
        for(const auto& pair:multiplicity)for(const auto& pc:pcs.at({bore,pair.first}))
            if(!d::PCurveIdentity(edges[pair.first].curve,pc,surface,mm,error))return fail();
        // Four directed UV segments must form one rectangle, traversed once.
        // Try the two seam-image assignments; exactly one must close correctly.
        unsigned rectangles=0;const double uScale=newSource.radius*mm,vScale=d::Norm(surface.z)*mm;
        for(unsigned swap=0;swap<2;++swap){
            std::array<gp_Pnt2d,4> a,b;unsigned occurrence=0;
            for(unsigned i=0;i<4;++i){const auto use=wallFace.wires[0][i];const auto& edge=edges[use.edge];
                const auto& pc=pcs.at({bore,use.edge})[use.edge==seam?((occurrence++)^swap):0];
                a[i]=At(pc,use.forward?edge.curve.first:edge.curve.last);b[i]=At(pc,use.forward?edge.curve.last:edge.curve.first);
            }
            std::array<unsigned,4> next{};bool valid=true;
            for(unsigned i=0;i<4;++i){unsigned n=0;for(unsigned j=0;j<4;++j)if(i!=j&&Close2(b[i],a[j],uScale,vScale,error)){next[i]=j;++n;}if(n!=1)valid=false;}
            if(!valid)continue;std::array<bool,4> seen{};unsigned at=0;double area=0;
            for(unsigned i=0;i<4;++i){if(seen[at]){valid=false;break;}seen[at]=true;
                const auto x=a[at],y=b[at],origin=a[0];area+=(x.X()-origin.X())*(y.Y()-origin.Y())-(y.X()-origin.X())*(x.Y()-origin.Y());at=next[at];}
            if(!valid||at!=0||!std::isfinite(area)||area==0)continue;
            const int wanted=wallFace.shape.Orientation()==TopAbs_FORWARD?1:-1;if((area>0?1:-1)!=wanted)continue;
            double u0=a[0].X(),u1=u0,v0=a[0].Y(),v1=v0;
            for(const auto& p:a){u0=std::min(u0,p.X());u1=std::max(u1,p.X());v0=std::min(v0,p.Y());v1=std::max(v1,p.Y());}
            if(std::abs((u1-u0)-2*pi)*uScale>error||std::abs((v1-v0)*d::Norm(surface.z)-(high-low))*mm>error)continue;
            unsigned corners=0;for(const auto& p:a){const bool u=std::abs(p.X()-u0)*uScale<=error||std::abs(p.X()-u1)*uScale<=error;
                const bool v=std::abs(p.Y()-v0)*vScale<=error||std::abs(p.Y()-v1)*vScale<=error;if(u&&v)++corners;}
            if(corners==4)++rectangles;
        }
        if(rectangles!=1)return fail();if(stop.load())return fail();
        report.circularOpeningCycles=2;report.seamUses=2;report.radiusMM=newSource.radius*mm;
        report.axisIntervalOriginalMM={(newSource.point[newSource.axis]+low)*mm,(newSource.point[newSource.axis]+high)*mm};
        report.phase="exterior-unproved";report.status=Status::BoreWallObservedExteriorUnproven;return report;
    }catch(...){return fail();}
}
}
