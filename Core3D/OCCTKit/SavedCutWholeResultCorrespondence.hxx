#pragma once
#include "AnalyticBooleanRingOperand.hxx"
#include "SavedCutResultBoundaryExpectation.hxx"
#include <queue>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <Precision.hxx>
#include <BRep_PointOnCurve.hxx>
#include <BRep_PointOnCurveOnSurface.hxx>
#include <BRep_PointOnSurface.hxx>
namespace core3d::saved_cut_whole_result {
namespace d=core3d::enclosure_correspondence::detail;
namespace od=core3d::saved_cut_bore_result::detail;
using Budget=core3d::enclosure_correspondence::Inspection;
enum class Classification { Refused, Cancelled, MatchedOrientedBoundary };
struct Inspection {Classification classification=Classification::Refused;const char* phase="input";std::size_t vertices=0,edges=0,faces=0,vertexLinks=0;double errorMM=0;};
namespace detail {
struct Graph {std::vector<od::Face> faces;std::vector<od::Edge> edges;Budget budget;std::map<std::pair<unsigned,unsigned>,std::vector<d::PCurve>> pcurves;};
inline bool Collect(const TopoDS_Shape& result,const retained_solid::Envelope& newSource,const std::atomic_bool& stop,Graph& graph,std::size_t maximumFaceWires=4){
    using namespace od;const double mm=newSource.metersPerUnit*1000;
    Budget budget;saved_cut_bore_result::Report report;const auto fail=[](){return false;};
        report.phase="collect";
        if(maximumFaceWires==0||maximumFaceWires>2+analytic_boolean_ring::kMaximumExpandedDisks||result.IsNull()||result.ShapeType()!=TopAbs_SOLID||result.Orientation()!=TopAbs_FORWARD)return fail();
        std::vector<TopoDS_Shape> shells,rawFaces;
        if(!d::Children(result,TopAbs_SHELL,1,shells,budget)||shells.size()!=1
            ||!d::Children(shells[0],TopAbs_FACE,130,rawFaces,budget)||rawFaces.empty())return fail();
        std::vector<Face> faces;std::vector<Edge> edges;TopTools_IndexedMapOfShape faceMap,wireMap,edgeMap,vertexMap;
        for(const auto& rawFace:rawFaces){
            if(stop.load())return fail();Face face;face.shape=TopoDS::Face(rawFace);
            if(faceMap.Contains(face.shape)||!d::SurfacePreflight(face.shape,budget)
                ||!d::Tolerance(BRep_Tool::Tolerance(face.shape),face.shape,mm,budget)
                ||!d::ReadSurface(face.shape,mm,budget,face.surface))return fail();faceMap.Add(face.shape);
            std::vector<TopoDS_Shape> rawWires;if(!d::Children(face.shape,TopAbs_WIRE,maximumFaceWires,rawWires,budget)||rawWires.empty())return fail();
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
        graph.faces=std::move(faces);graph.edges=std::move(edges);graph.pcurves=std::move(pcs);graph.budget=budget;return true;
}
// Complete2pi planar opening role. Frozen quarter-arc/old matchers unchanged.
inline bool FullPlanarCircleIdentity(const d::Curve& c,const d::PCurve& pc,const d::Surface& s,double mm,double error){
    if(s.cylinder||!c.circle||!pc.circle)return false;
    for(const auto& box:s.boxes)for(unsigned axis=0;axis<2;++axis){
        const double center=axis?pc.c.Y():pc.c.X(),a=axis?pc.a.Y():pc.a.X(),b=axis?pc.b.Y():pc.b.X();
        const double radius=std::hypot(a,b);if(!std::isfinite(radius)||center-radius<box[axis*2]||center+radius>box[axis*2+1])return false;
    }
    const auto center=s.c+s.x*pc.c.X()+s.y*pc.c.Y(),a=s.x*pc.a.X()+s.y*pc.a.Y(),b=s.x*pc.b.X()+s.y*pc.b.Y();
    double ab=0,residual=0,residualMM=0;
    if(!d::TrimUpperAdd(d::Norm(a-c.a),d::Norm(b-c.b),ab)||!d::TrimUpperAdd(d::Norm(center-c.c),ab,residual)
        ||!d::TrimUpperMultiply(residual,mm,residualMM))return false;
    return d::TrimResidualWithin(c,pc,s,mm,residualMM,error);
}
inline bool OriginalCurve(const d::Curve& c,const enclosure_correspondence::ExpectedEdge& e,const Expected& expected,bool forward,double mm,double error){
    if(c.circle!=e.circle)return false;
    const auto start=d::V(expected.vertices[forward?e.start:e.end]),end=d::V(expected.vertices[forward?e.end:e.start]);
    if(!d::Close(d::Evaluate(c,c.first),start,mm,error)||!d::Close(d::Evaluate(c,c.last),end,mm,error))return false;
    if(!c.circle)return true;
    const double a=d::Norm(c.a),b=d::Norm(c.b),pi=std::acos(-1.0);if(a<=0||b<=0)return false;
    if(e.fullCircle){
        if(e.start!=e.end||!d::Close(d::Evaluate(c,c.first),d::Evaluate(c,c.last),mm,error)
            ||std::abs(c.last-c.first-2*pi)*e.radius*mm>error)return false;
        // A closed rim has no endpoint chord normal. Derive its directed plane
        // from its recipe cap coedge and its recipe cylinder's radial sign.
        gp_Vec wanted;unsigned matches=0;
        for(const auto& face:expected.faces)if(!face.cylinder)for(const auto& wire:face.wires)
            if(wire.size()==1&&wire[0].edge<expected.edges.size()&& &expected.edges[wire[0].edge]==&e){
                int radial=0;for(const auto& side:expected.faces)if(side.cylinder)
                    for(const auto& loop:side.wires)for(const auto& use:loop)if(use.edge==wire[0].edge)radial=side.radialSign;
                if(radial!=1&&radial!=-1)return false;
                wanted=face.normalOrAxis*((wire[0].forward?1:-1)*radial*(forward?1:-1));++matches;}
        const auto actual=c.a.Crossed(c.b);
        if(matches!=1||d::Norm(wanted)<=0||d::Norm(actual)<=0)return false;
        return d::Close(c.c,d::V(e.center),mm,error)&&std::abs(a-e.radius)*mm<=error&&std::abs(b-e.radius)*mm<=error
            &&std::abs(c.a.Dot(c.b))/(a*b)*e.radius*mm<=error
            &&d::Norm(actual/d::Norm(actual)-wanted/d::Norm(wanted))*e.radius*mm<=error;
    }
    const auto wanted=(start-d::V(e.center)).Crossed(end-d::V(e.center)),actual=c.a.Crossed(c.b);
    if(d::Norm(wanted)<=0||d::Norm(actual)<=0)return false;
    return d::Close(c.c,d::V(e.center),mm,error)&&std::abs(a-e.radius)*mm<=error&&std::abs(b-e.radius)*mm<=error
        &&std::abs(c.a.Dot(c.b))/(a*b)*e.radius*mm<=error&&std::abs(c.last-c.first-pi/2)*e.radius*mm<=error
        &&d::Norm(actual/d::Norm(actual)-wanted/d::Norm(wanted))*e.radius*mm<=error;
}
}
namespace detail {
inline bool RepresentationOwners(const Graph& g,unsigned seam,unsigned bore,Budget& budget,const std::map<unsigned,unsigned>& hostSeams={}){
    for(unsigned i=0;i<g.edges.size();++i){const auto& edge=g.edges[i];
        const std::set<unsigned> owners(edge.faceIDs.begin(),edge.faceIDs.end());
        std::map<unsigned,unsigned> pcCounts;unsigned curves=0;
        const auto data=Handle(BRep_TEdge)::DownCast(edge.shape.TShape());
        for(BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());it.More();it.Next()){
            const auto& rep=it.Value();if(rep->IsCurve3D()){if(++curves!=1)return false;continue;}
            if(rep->IsCurveOnSurface()){
                unsigned found=0,owner=0;for(unsigned f:owners){const auto& surface=g.faces[f].surface;
                    if(!d::PairLocation(surface.location,edge.shape.Location(),budget))return false;
                    const auto relative=surface.location.Predivided(edge.shape.Location());if(!d::Location(relative,budget))return false;
                    if(rep->IsCurveOnSurface(surface.handle,relative)){++found;owner=f;}}
                if(found!=1||++pcCounts[owner]!=1)return false;
                const auto host=hostSeams.find(i);const bool designated=i==seam||host!=hostSeams.end();
                const unsigned seamOwner=i==seam?bore:host!=hostSeams.end()?host->second:0;
                if(rep->IsCurveOnClosedSurface()&&(!designated||owner!=seamOwner||owners.size()!=1))return false;
                if(designated&&!rep->IsCurveOnClosedSurface())return false;
            }else if(rep->IsRegularity()){
                const auto& a=g.faces[edge.faceIDs[0]].surface;const auto& b=g.faces[edge.faceIDs[1]].surface;
                if(!d::PairLocation(a.location,edge.shape.Location(),budget)||!d::PairLocation(b.location,edge.shape.Location(),budget))return false;
                const auto la=a.location.Predivided(edge.shape.Location()),lb=b.location.Predivided(edge.shape.Location());
                if(!d::Location(la,budget)||!d::Location(lb,budget)||!rep->IsRegularity(a.handle,b.handle,la,lb))return false;
            }else if(!rep->IsPolygon3D()&&!rep->IsPolygonOnTriangulation())return false;
        }if(curves!=1)return false;
    }return true;
}
struct PointWitness {gp_Vec vertex,value;};
inline gp_Vec SurfacePoint(const d::Surface& s,double u,double v){
    return s.cylinder?s.c+s.x*std::cos(u)+s.y*std::sin(u)+s.z*v:s.c+s.x*u+s.y*v;
}
// Bind every native point representation to an actual incident support. OCCT's
// context validity check intentionally ignores foreign records; it is not this
// ownership proof. Do all support/location/parameter work before fixing error.
inline bool PointOwners(const Graph& g,double mm,const std::atomic_bool& stop,Budget& budget,std::vector<PointWitness>& witnesses){
    TopTools_IndexedMapOfShape visited;
    for(const auto& first:g.edges)for(const auto& vertex:first.vertices){
        if(stop.load())return false;if(visited.Contains(vertex))continue;visited.Add(vertex);
        std::vector<unsigned> incident;std::set<unsigned> faces;
        for(unsigned e=0;e<g.edges.size();++e)if(vertex.IsSame(g.edges[e].vertices[0])||vertex.IsSame(g.edges[e].vertices[1])){
            incident.push_back(e);faces.insert(g.edges[e].faceIDs.begin(),g.edges[e].faceIDs.end());}
        const auto data=Handle(BRep_TVertex)::DownCast(vertex.TShape());if(data.IsNull())return false;unsigned count=0;
        for(BRep_ListIteratorOfListOfPointRepresentation it(data->Points());it.More();it.Next()){
            if(stop.load())return false;const auto& record=it.Value();if(++count>16||record.IsNull()
                ||!d::PairLocation(vertex.Location(),record->Location(),budget)||!std::isfinite(record->Parameter()))return false;
            const bool curve=record->DynamicType()==STANDARD_TYPE(BRep_PointOnCurve);
            const bool onPC=record->DynamicType()==STANDARD_TYPE(BRep_PointOnCurveOnSurface);
            const bool surface=record->DynamicType()==STANDARD_TYPE(BRep_PointOnSurface);
            if(unsigned(curve)+unsigned(onPC)+unsigned(surface)!=1)return false;
            bool found=false;const double t=record->Parameter();
            if(curve||onPC)for(unsigned e:incident){const auto& edge=g.edges[e];bool endpoint=false;
                for(unsigned n=0;n<2;++n)if(vertex.IsSame(edge.vertices[n])&&od::Bits(t,n?edge.curve.last:edge.curve.first))endpoint=true;
                if(!endpoint)continue;
                if(curve){
                    if(record->Curve().IsNull()||!d::PairLocation(edge.curve.location,vertex.Location(),budget))return false;
                    const auto relative=edge.curve.location.Predivided(vertex.Location());if(!d::Location(relative,budget))return false;
                    TopLoc_Location location;double first=0,last=0;const auto support=BRep_Tool::Curve(edge.shape,location,first,last);
                    if(record->IsPointOnCurve(support,relative)){
                        if(witnesses.size()>=4096)return false;witnesses.push_back({d::V(BRep_Tool::Pnt(vertex)),d::Evaluate(edge.curve,t)});found=true;}
                }else for(unsigned f:std::set<unsigned>(edge.faceIDs.begin(),edge.faceIDs.end())){
                    const auto& support=g.faces[f].surface;
                    if(record->PCurve().IsNull()||record->Surface().IsNull()
                        ||!d::PairLocation(support.location,vertex.Location(),budget)
                        ||!d::PairLocation(support.location,edge.shape.Location(),budget))return false;
                    const auto relative=support.location.Predivided(vertex.Location()),edgeRelative=support.location.Predivided(edge.shape.Location());
                    if(!d::Location(relative,budget)||!d::Location(edgeRelative,budget))return false;
                    const auto edgeData=Handle(BRep_TEdge)::DownCast(edge.shape.TShape());
                    for(BRep_ListIteratorOfListOfCurveRepresentation ri(edgeData->Curves());ri.More();ri.Next()){
                        const auto& rep=ri.Value();if(!rep->IsCurveOnSurface(support.handle,edgeRelative))continue;
                        const auto pcs=g.pcurves.find({f,e});if(pcs==g.pcurves.end())return false;
                        for(unsigned k=0;k<(rep->IsCurveOnClosedSurface()?2u:1u);++k){
                            if(k>=pcs->second.size())return false;const auto handle=k?rep->PCurve2():rep->PCurve();
                            if(record->IsPointOnCurveOnSurface(handle,support.handle,relative)){
                                const auto& p=pcs->second[k];const double a=p.circle?std::cos(t):t,b=p.circle?std::sin(t):0;
                                const double u=p.c.X()+p.a.X()*a+p.b.X()*b,v=p.c.Y()+p.a.Y()*a+p.b.Y()*b;
                                if(witnesses.size()>=4096)return false;witnesses.push_back({d::V(BRep_Tool::Pnt(vertex)),SurfacePoint(support,u,v)});found=true;}
                        }
                    }
                }
            }
            if(surface){
                const double v=record->Parameter2();if(!std::isfinite(v)||record->Surface().IsNull())return false;
                for(unsigned f:faces){const auto& support=g.faces[f].surface;
                    if(!d::PairLocation(support.location,vertex.Location(),budget))return false;
                    const auto relative=support.location.Predivided(vertex.Location());if(!d::Location(relative,budget))return false;
                    if(!record->IsPointOnSurface(support.handle,relative))continue;
                    for(const auto& box:support.boxes)if(t<box[0]||t>box[1]||v<box[2]||v>box[3])return false;
                    d::PCurve constant;constant.c={t,v};constant.first=constant.last=0;
                    if(!d::PCurveMagnitude(constant,support,mm,budget))return false;
                    if(witnesses.size()>=4096)return false;witnesses.push_back({d::V(BRep_Tool::Pnt(vertex)),SurfacePoint(support,t,v)});found=true;
                }
            }
            if(!found)return false;
        }
    }return true;
}
// A periodic host side is selected exclusively from recipe cells and the
// already bijective face/edge correspondence. Observed topology cannot grant
// an exception to generic vertex-link or representation-ownership rules.
struct HostWall {unsigned face=0,seam=0;std::array<unsigned,2> rims{};};
inline bool HostWalls(const Graph& g,const Expected& expected,const std::vector<int>& originalEdge,
    const std::vector<int>& originalFace,double mm,double error,std::vector<HostWall>& walls){
    walls.clear();const double pi=std::acos(-1.0);
    for(unsigned role=0;role<expected.faces.size();++role){const auto& wanted=expected.faces[role];
        if(!wanted.cylinder)continue;
        bool periodic=false;for(const auto& wire:wanted.wires)for(const auto& use:wire)
            if(expected.edges.at(use.edge).fullCircle)periodic=true;
        if(!periodic)continue;
        if(wanted.wires.size()!=1||wanted.wires[0].size()!=4)return false;
        HostWall wall;unsigned faceCount=0;
        for(unsigned f=0;f<originalFace.size();++f)if(originalFace[f]==int(role)){wall.face=f;++faceCount;}
        if(faceCount!=1)return false;
        const auto& face=g.faces.at(wall.face);if(face.wires.size()!=1||face.wires[0].size()!=4)return false;
        std::map<unsigned,unsigned> counts;for(const auto& use:wanted.wires[0])++counts[use.edge];
        unsigned rims=0,seams=0;
        for(const auto& row:counts){unsigned actual=0,matches=0;
            for(unsigned e=0;e<originalEdge.size();++e)if(originalEdge[e]==int(row.first)){actual=e;++matches;}
            if(matches!=1)return false;
            if(expected.edges[row.first].fullCircle&&row.second==1){if(rims>=2)return false;wall.rims[rims++]=actual;}
            else if(!expected.edges[row.first].circle&&row.second==2){wall.seam=actual;++seams;}else return false;
        }
        if(rims!=2||seams!=1)return false;
        const auto& seam=g.edges.at(wall.seam);if(seam.vertices[0].IsSame(seam.vertices[1])
            ||seam.faceIDs!=std::vector<unsigned>{wall.face,wall.face})return false;
        const auto pcs=g.pcurves.find({wall.face,wall.seam});if(pcs==g.pcurves.end()||pcs->second.size()!=2)return false;
        std::array<unsigned,2> shared{};
        for(unsigned rim:wall.rims){const auto& edge=g.edges.at(rim);
            if(!edge.vertices[0].IsSame(edge.vertices[1]))return false;
            unsigned count=0,end=0;for(unsigned k=0;k<2;++k)if(edge.vertices[0].IsSame(seam.vertices[k])){++count;end=k;}
            if(count!=1||++shared[end]!=1)return false;
            const auto pc=g.pcurves.find({wall.face,rim});if(pc==g.pcurves.end()||pc->second.size()!=1)return false;
        }
        if(shared[0]!=1||shared[1]!=1)return false;
        // Complete stored UV boundary: exactly one directed rectangle of one
        // period and the recipe seam length, including both seam images.
        const auto& surface=face.surface;const double uScale=wanted.radius*mm,vScale=d::Norm(surface.z)*mm;
        double height=0;for(const auto& row:counts)if(row.second==2){const auto& e=expected.edges[row.first];height=expected.vertices[e.start].Distance(expected.vertices[e.end]);}
        unsigned rectangles=0;
        for(unsigned swap=0;swap<2;++swap){
            std::array<gp_Pnt2d,4> a,b;unsigned occurrence=0;
            for(unsigned k=0;k<4;++k){const auto use=face.wires[0][k];const auto& edge=g.edges[use.edge];
                const auto& pc=g.pcurves.at({wall.face,use.edge}).at(use.edge==wall.seam?((occurrence++)^swap):0);
                a[k]=od::At(pc,use.forward?edge.curve.first:edge.curve.last);b[k]=od::At(pc,use.forward?edge.curve.last:edge.curve.first);}
            if(occurrence!=2)return false;
            std::array<unsigned,4> next{};bool valid=true;
            for(unsigned k=0;k<4;++k){unsigned count=0;for(unsigned j=0;j<4;++j)if(k!=j&&od::Close2(b[k],a[j],uScale,vScale,error)){next[k]=j;++count;}if(count!=1)valid=false;}
            if(!valid)continue;std::array<bool,4> seen{};unsigned at=0;double area=0;
            for(unsigned k=0;k<4;++k){if(seen[at]){valid=false;break;}seen[at]=true;
                const auto x=a[at],y=b[at],origin=a[0];area+=(x.X()-origin.X())*(y.Y()-origin.Y())-(y.X()-origin.X())*(x.Y()-origin.Y());at=next[at];}
            if(!valid||at!=0||!std::isfinite(area)||area==0||(area>0?1:-1)!=(face.shape.Orientation()==TopAbs_FORWARD?1:-1))continue;
            double u0=a[0].X(),u1=u0,v0=a[0].Y(),v1=v0;
            for(const auto& p:a){u0=std::min(u0,p.X());u1=std::max(u1,p.X());v0=std::min(v0,p.Y());v1=std::max(v1,p.Y());}
            if(std::abs(u1-u0-2*pi)*uScale>error||std::abs((v1-v0)*d::Norm(surface.z)-height)*mm>error)continue;
            unsigned corners=0;for(const auto& p:a){const bool u=std::abs(p.X()-u0)*uScale<=error||std::abs(p.X()-u1)*uScale<=error;
                const bool v=std::abs(p.Y()-v0)*vScale<=error||std::abs(p.Y()-v1)*vScale<=error;if(u&&v)++corners;}
            if(corners==4)++rectangles;
        }
        if(rectangles!=1)return false;walls.push_back(wall);
    }return true;
}
inline bool VertexLinks(const Graph& g,const TopTools_IndexedMapOfShape& vertices,unsigned bore,unsigned seam,const std::vector<unsigned>& openings,const std::vector<HostWall>& hosts={}){
    using Adjacency=std::map<unsigned,std::set<unsigned>>;
    std::vector<Adjacency> links(vertices.Extent());std::vector<std::set<unsigned>> germs(vertices.Extent());
    const auto vertex=[&](const TopoDS_Vertex& v)->unsigned {return unsigned(vertices.FindIndex(v)-1);};
    for(unsigned e=0;e<g.edges.size();++e)for(unsigned end=0;end<2;++end)germs[vertex(g.edges[e].vertices[end])].insert(2*e+end);
    const auto arc=[&](unsigned v,unsigned a,unsigned b){
        if(v>=links.size()||a==b||!germs[v].count(a)||!germs[v].count(b))return false;
        return links[v][a].insert(b).second&&links[v][b].insert(a).second;
    };
    for(unsigned f=0;f<g.faces.size();++f){if(f==bore||std::any_of(hosts.begin(),hosts.end(),[&](const HostWall& host){return host.face==f;}))continue;
        for(const auto& wire:g.faces[f].wires){std::map<unsigned,unsigned> incoming,outgoing;
            for(const auto& use:wire){const auto& e=g.edges[use.edge];const unsigned start=use.forward?0:1,end=1-start;
                if(!outgoing.emplace(vertex(e.vertices[start]),2*use.edge+start).second
                    ||!incoming.emplace(vertex(e.vertices[end]),2*use.edge+end).second)return false;}
            if(incoming.size()!=outgoing.size())return false;
            for(const auto& row:incoming){const auto next=outgoing.find(row.first);if(next==outgoing.end()||!arc(row.first,row.second,next->second))return false;}
        }
    }
    // Seam images are distinct corners of the cylinder's rectangular domain.
    // The preceding bore certificate proves their UV adjacency and sharing.
    for(unsigned opening:openings){const auto& e=g.edges[opening];unsigned matched=0,index=0;
        for(unsigned n=0;n<2;++n)if(e.vertices[0].IsSame(g.edges[seam].vertices[n])){++matched;index=n;}
        if(matched!=1||!arc(vertex(e.vertices[0]),2*opening,2*seam+index)
            ||!arc(vertex(e.vertices[0]),2*opening+1,2*seam+index))return false;
    }
    for(const auto& host:hosts)for(unsigned rim:host.rims){const auto& e=g.edges[rim];unsigned matched=0,index=0;
        for(unsigned n=0;n<2;++n)if(e.vertices[0].IsSame(g.edges[host.seam].vertices[n])){++matched;index=n;}
        if(matched!=1||!arc(vertex(e.vertices[0]),2*rim,2*host.seam+index)
            ||!arc(vertex(e.vertices[0]),2*rim+1,2*host.seam+index))return false;
    }
    for(unsigned v=0;v<links.size();++v){
        if(germs[v].size()<3||links[v].size()!=germs[v].size())return false;
        for(unsigned germ:germs[v])if(links[v][germ].size()!=2)return false;
        std::set<unsigned> seen;std::vector<unsigned> pending{*germs[v].begin()};
        while(!pending.empty()){const auto next=pending.back();pending.pop_back();if(!seen.insert(next).second)continue;
            for(unsigned adjacent:links[v][next])pending.push_back(adjacent);}
        if(seen!=germs[v])return false;
    }return true;
}
}
inline Inspection Inspect(const TopoDS_Shape& result,const retained_solid::Envelope& oldSource,
    const retained_solid::Envelope& newSource,const std::atomic_bool& stop) noexcept {
    Inspection report;const auto fail=[&](){report.classification=stop.load()?Classification::Cancelled:Classification::Refused;return report;};
    try {
        if(stop.load())return fail();const auto boreEvidence=saved_cut_bore_result::Inspect(result,oldSource,newSource,stop);
        report.phase=boreEvidence.phase;
        if(boreEvidence.status!=saved_cut_bore_result::Status::BoreWallObservedExteriorUnproven)return fail();
        Expected expected;if(!ExpectedSource(newSource,expected))return fail();detail::Graph graph;report.phase="whole-collect";
        if(!detail::Collect(result,newSource,stop,graph))return fail();const double mm=newSource.metersPerUnit*1000,pi=std::acos(-1.0);
        if(graph.faces.size()!=expected.faces.size()+1||graph.edges.size()!=expected.edges.size()+3)return fail();
        for(const auto& p:expected.vertices)if(!d::Track(std::abs(p.X())+std::abs(p.Y())+std::abs(p.Z()),mm,graph.budget))return fail();
        // Finish ALL planar pcurve/trim magnitudes before geometric matching.
        for(unsigned f=0;f<graph.faces.size();++f)if(!graph.faces[f].surface.cylinder)
            for(const auto& wire:graph.faces[f].wires)for(const auto& use:wire){
                if(stop.load())return fail();const auto key=std::make_pair(f,use.edge);if(graph.pcurves.count(key))continue;
                const auto& edge=graph.edges[use.edge];const auto& surface=graph.faces[f].surface;d::PCurve pc;
                if(!d::PairLocation(edge.shape.Location(),surface.location,graph.budget)
                    ||!d::ReadPCurve(edge.shape,surface,edge.curve,pc,graph.budget)||!d::PCurveMagnitude(pc,surface,mm,graph.budget))return fail();
                graph.pcurves.emplace(key,std::vector<d::PCurve>{pc});
            }
        report.phase="whole-point-owners";std::vector<detail::PointWitness> pointWitnesses;
        if(!detail::PointOwners(graph,mm,stop,graph.budget,pointWitnesses))return fail();
        if(!d::Track(graph.budget.maximumLocationCompositionMagnitude,mm,graph.budget))return fail();
        const double error=std::max(1e-9,2048*std::numeric_limits<double>::epsilon()*graph.budget.arithmeticMagnitudeMM);
        if(!std::isfinite(error)||error>1e-6)return fail();report.errorMM=error;
        const double fixedArithmetic=graph.budget.arithmeticMagnitudeMM,fixedComposition=graph.budget.maximumLocationCompositionMagnitude;
        for(unsigned i=0;i<expected.vertices.size();++i)for(unsigned j=0;j<i;++j)
            if(expected.vertices[i].Distance(expected.vertices[j])*mm<=2*error+4*graph.budget.maximumKernelToleranceMM)return fail();
        for(const auto& point:pointWitnesses)if(!d::Close(point.vertex,point.value,mm,error))return fail();
        report.phase="whole-vertices";TopTools_IndexedMapOfShape vertices;std::vector<int> originalVertex;std::vector<bool> claimed(expected.vertices.size(),false);
        for(const auto& edge:graph.edges)for(const auto& v:edge.vertices){
            if(stop.load())return fail();if(vertices.Contains(v))continue;const auto point=BRep_Tool::Pnt(v);unsigned count=0,id=0;
            for(unsigned i=0;i<expected.vertices.size();++i)if(d::Close(d::V(point),d::V(expected.vertices[i]),mm,error)){++count;id=i;}
            if(count>1||(count==1&&claimed[id]))return fail();vertices.Add(v);originalVertex.push_back(count?int(id):-1);if(count)claimed[id]=true;
        }
        if(vertices.Extent()!=int(expected.vertices.size()+2)||std::find(claimed.begin(),claimed.end(),false)!=claimed.end())return fail();
        std::vector<int> originalEdge(graph.edges.size(),-1);std::vector<bool> edgeForward(graph.edges.size()),claimedEdges(expected.edges.size(),false);std::vector<unsigned> newEdges;
        for(unsigned i=0;i<graph.edges.size();++i){
            if(stop.load())return fail();const auto& edge=graph.edges[i];
            const int a=originalVertex[vertices.FindIndex(edge.vertices[0])-1],b=originalVertex[vertices.FindIndex(edge.vertices[1])-1];
            if(a<0||b<0){if(a>=0||b>=0)return fail();newEdges.push_back(i);continue;}
            unsigned count=0,id=0;bool forward=false;
            for(unsigned j=0;j<expected.edges.size();++j){const auto& e=expected.edges[j];if((e.start==unsigned(a)&&e.end==unsigned(b))||(e.start==unsigned(b)&&e.end==unsigned(a))){++count;id=j;forward=e.start==unsigned(a);}}
            if(count!=1||claimedEdges[id]||!detail::OriginalCurve(edge.curve,expected.edges[id],expected,forward,mm,error))return fail();
            if(!od::Bits(BRep_Tool::Parameter(edge.vertices[0],edge.shape),edge.curve.first)||!od::Bits(BRep_Tool::Parameter(edge.vertices[1],edge.shape),edge.curve.last))return fail();
            claimedEdges[id]=true;originalEdge[i]=int(id);edgeForward[i]=forward;
        }
        if(newEdges.size()!=3||std::find(claimedEdges.begin(),claimedEdges.end(),false)!=claimedEdges.end())return fail();
        report.phase="whole-faces";unsigned bore=0,boreCount=0;std::vector<unsigned> openings;unsigned seam=0,seamCount=0;
        for(unsigned i:newEdges)if(graph.edges[i].curve.circle)openings.push_back(i);else{seam=i;++seamCount;}
        if(openings.size()!=2||seamCount!=1)return fail();
        std::vector<bool> claimedFaces(expected.faces.size(),false);std::set<unsigned> piercedCaps;
        std::vector<int> originalFace(graph.faces.size(),-1);
        for(unsigned f=0;f<graph.faces.size();++f){
            if(stop.load())return fail();const auto& face=graph.faces[f];std::vector<std::vector<std::pair<unsigned,bool>>> originalWires;unsigned extraOpenings=0,totalOriginal=0,totalNew=0;
            for(const auto& wire:face.wires){std::vector<std::pair<unsigned,bool>> w;unsigned newCount=0;
                for(const auto& use:wire){if(originalEdge[use.edge]>=0){w.emplace_back(unsigned(originalEdge[use.edge]),use.forward==edgeForward[use.edge]);++totalOriginal;}else{++newCount;++totalNew;}}
                if(newCount&& !w.empty())return fail();
                if(!w.empty())originalWires.push_back(std::move(w));
                else if(wire.size()==1&&graph.edges[wire[0].edge].curve.circle){++extraOpenings;
                    const auto& c=graph.edges[wire[0].edge].curve;auto n=face.surface.x.Crossed(face.surface.y);const double len=d::Norm(n);if(!std::isfinite(len)||len<=0)return fail();
                    n*=((face.shape.Orientation()==TopAbs_FORWARD?1:-1)/len);
                    if(c.a.Crossed(c.b).Dot(n)*(wire[0].forward?1:-1)>=0)return fail();
                }
            }
            if(totalOriginal==0){if(totalNew!=4||face.wires.size()!=1||!face.surface.cylinder)return fail();bore=f;++boreCount;continue;}
            if(totalNew!=extraOpenings||extraOpenings>1)return fail();
            unsigned count=0,role=0;
            for(unsigned r=0;r<expected.faces.size();++r){const auto& wanted=expected.faces[r];if(wanted.wires.size()!=originalWires.size())continue;
                std::set<unsigned> used;bool match=true;for(const auto& actual:originalWires){unsigned n=0,id=0;
                    for(unsigned w=0;w<wanted.wires.size();++w)if(d::WireMatch(actual,wanted.wires[w])){++n;id=w;}
                    if(n!=1||!used.insert(id).second){match=false;break;}}
                if(match){++count;role=r;}
            }
            d::Face surfaceView;surfaceView.face=face.shape;surfaceView.surface=face.surface;
            if(count!=1||claimedFaces[role]||!d::MatchSurface(surfaceView,expected.faces[role],mm,error))return fail();claimedFaces[role]=true;originalFace[f]=int(role);
            const bool cap=role==expected.caps[0]||role==expected.caps[1];
            if(extraOpenings!=(cap?1u:0u))return fail();if(cap&&!piercedCaps.insert(role).second)return fail();
        }
        if(boreCount!=1||piercedCaps.size()!=2||std::find(claimedFaces.begin(),claimedFaces.end(),false)!=claimedFaces.end())return fail();
        report.phase="whole-pcurves";
        for(const auto& entry:graph.pcurves){
            if(stop.load())return fail();const auto& surface=graph.faces[entry.first.first].surface;const auto& c=graph.edges[entry.first.second].curve;
            for(const auto& pc:entry.second){const bool fullPlanar=!surface.cylinder&&c.circle&&c.last-c.first>pi;
                if(fullPlanar?!detail::FullPlanarCircleIdentity(c,pc,surface,mm,error):!d::PCurveIdentity(c,pc,surface,mm,error))return fail();}
        }
        report.phase="whole-host-seams";std::vector<detail::HostWall> hosts;
        if(!detail::HostWalls(graph,expected,originalEdge,originalFace,mm,error,hosts))return fail();
        std::map<unsigned,unsigned> hostSeams;for(const auto& host:hosts)if(!hostSeams.emplace(host.seam,host.face).second)return fail();
        report.phase="representation-owners";
        if(!detail::RepresentationOwners(graph,seam,bore,graph.budget,hostSeams))return fail();
        // Ownership inspection may evaluate bounded location products, but it
        // may not increase the already fixed arithmetic allowance.
        if(graph.budget.arithmeticMagnitudeMM>fixedArithmetic||graph.budget.maximumLocationCompositionMagnitude>fixedComposition)return fail();
        report.phase="whole-vertex-links";if(!detail::VertexLinks(graph,vertices,bore,seam,openings,hosts))return fail();
        long cellEuler=long(vertices.Extent())-long(graph.edges.size());
        for(const auto& face:graph.faces)cellEuler+=2-long(face.wires.size());
        if(cellEuler!=2-2*long(expected.hostGenus+1))return fail(); // supplementary to complete mapped cells/links
        report.phase="kernel-validity";if(stop.load()||!BRepCheck_Analyzer(result,Standard_True).IsValid())return fail();
        if(stop.load())return fail();BRepClass3d_SolidClassifier outside(result);
        outside.PerformInfinitePoint(Precision::Confusion());
        if(stop.load()||outside.State()!=TopAbs_OUT)return fail();
        if(stop.load())return fail();report.vertices=vertices.Extent();report.edges=graph.edges.size();report.faces=graph.faces.size();report.vertexLinks=vertices.Extent();
        report.phase="matched-oriented-boundary";report.classification=Classification::MatchedOrientedBoundary;return report;
    }catch(...){return fail();}
}
}
