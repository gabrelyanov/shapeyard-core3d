#pragma once
// Complete program geometry proof. Local bore observations NEVER grant authority.
#include "RetainedBooleanProgram.hxx"
#include "SavedCutWholeResultCorrespondence.hxx"
namespace core3d::saved_boolean_result {
namespace old=core3d::saved_cut_whole_result;
namespace d=core3d::enclosure_correspondence::detail;
namespace od=core3d::saved_cut_bore_result::detail;
using Budget=old::Budget;using Classification=old::Classification;using Inspection=old::Inspection;
namespace detail {
using Graph=old::detail::Graph;
// A value-only view for one EXPLICIT operand's existing analytic predicates.
// This does not leave this geometry layer as a snapshot, payload or permission;
// final success below requires EVERY program operand and the entire boundary.
inline retained_solid::Envelope GeometryView(const retained_boolean::Program& p,const analytic_boolean::Operand& t){
    const auto& s=p.source;retained_solid::Envelope e;
    e.document=s.document;e.entity=s.entity;e.definition=s.definition;e.sourceFeature=s.sourceFeature;e.derivedFeature=s.derivedFeature;
    e.sourceFamily=s.family;e.sourceSchema=s.schema;e.metersPerUnit=s.metersPerUnit;e.sourceValues=s.values;
    e.operandID=t.identifier;e.axis=std::uint8_t(t.axis);e.point=t.point;e.radius=t.radius;return e;
}
inline retained_solid::Envelope GeometryView(const retained_boolean::Program& p,std::size_t index){
    const auto& t=p.steps.at(index).operand;
    if(t.kind==analytic_boolean::OperandKind::Cylinder)return GeometryView(p,t);
    auto disk=analytic_boolean_ring::Expand(analytic_boolean_ring::FromOperand(t,p.source.metersPerUnit),0,p.source.metersPerUnit);
    disk.radius=t.radius;return GeometryView(p,disk);
}
// Admission requires EVERY unordered expanded-disk pair to be a separated same-axis
// disk pair. Checking only pair 0/1 (or only adjacent pairs) is unsound: with
// three or more bores a nonadjacent pair can overlap while every checked pair
// clears. ANY invalid, near-tangent, overlapping, coincident or different-axis
// pair refuses the whole program. One-disk programs stay refused here; the
// legacy one-hole path owns them. Interval arithmetic, rounding-mode gate,
// kernel separation and numeric-uncertainty bounds are unchanged per pair.
inline bool SeparateDisks(const retained_boolean::Program& p){
    namespace i=saved_cut_bore_clearance::detail;
    std::vector<retained_boolean::Disk> disks;
    if(!retained_boolean::ExpandedDisks(p,disks)||disks.size()<2||std::fegetround()!=FE_TONEAREST)return false;
    const unsigned axis=unsigned(p.steps[0].operand.axis);
    const unsigned u=(axis+1)%3,v=(axis+2)%3;
    for(std::size_t a=0;a<disks.size();++a){
        const auto& x=disks[a].operand;
        if(unsigned(x.axis)!=axis)return false;
        for(std::size_t b=0;b<a;++b){
            const auto& y=disks[b].operand;
            const auto distance=i::norm(i::sub(i::I(x.point[u]),i::I(y.point[u])),i::sub(i::I(x.point[v]),i::I(y.point[v])));
            const auto gap=i::mul(i::sub(distance,i::add(i::I(x.radius),i::I(y.radius))),i::mul(i::I(p.source.metersPerUnit),i::I(1000)));
            if(!i::good(gap)||gap.lo<=saved_cut_bore_clearance::KernelSeparationMM
                ||i::up(gap.hi-gap.lo)>saved_cut_bore_clearance::MaximumNumericUncertaintyMM)return false;
        }
    }return true;
}
// Independent source-recipe radial extent, never inferred from a cut result.
inline bool HostRadialExtent(const retained_boolean::Program& p,const analytic_boolean::Operand& t,double& extent){
    extent=0;old::Expected expected;
    if(!old::ExpectedSource(GeometryView(p,t),expected)||unsigned(t.axis)>2)return false;
    const unsigned axis=unsigned(t.axis),u=(axis+1)%3,v=(axis+2)%3;
    if(p.source.family==1){profile::Parameters source;if(!profile::Decode(p.source.values,source))return false;
        if(source.definition.circle){
            for(const auto& face:expected.faces)if(face.cylinder&&face.radialSign==1){
                gp_Vec direction(0,0,0);direction.SetCoord(axis+1,1);
                const gp_Vec delta(gp_Pnt(t.point[0],t.point[1],t.point[2]),face.origin);
                if(face.normalOrAxis.Crossed(direction).Magnitude()>Precision::Angular()
                    ||(delta-direction*delta.Dot(direction)).Magnitude()*p.source.metersPerUnit*1000>1e-9)return false;
                extent=face.radius;return std::isfinite(extent)&&extent>0;
            }return false;
        }
    }
    // Polygon/loft bounds are determined by their authored vertices. Curved
    // enclosure bounds are conservatively extended by each expected circle.
    std::array<double,3> low{{INFINITY,INFINITY,INFINITY}},high{{-INFINITY,-INFINITY,-INFINITY}};
    const auto add=[&](const gp_Pnt& point,double radius){for(unsigned j=0;j<3;++j){
        low[j]=std::min(low[j],point.Coord(j+1)-radius);high[j]=std::max(high[j],point.Coord(j+1)+radius);}};
    for(const auto& point:expected.vertices)add(point,0);
    for(const auto& edge:expected.edges)if(edge.circle)add(edge.center,edge.radius);
    const double du=std::max(std::abs(low[u]-t.point[u]),std::abs(high[u]-t.point[u]));
    const double dv=std::max(std::abs(low[v]-t.point[v]),std::abs(high[v]-t.point[v]));
    extent=std::hypot(du,dv);return std::isfinite(extent)&&extent>0;
}
inline bool AdmitDisks(const retained_boolean::Program& p){
    std::vector<retained_boolean::Disk> disks;
    if(!SeparateDisks(p)||!retained_boolean::ExpandedDisks(p,disks))return false;
    for(const auto& disk:disks)if(saved_cut_bore_clearance::Inspect(GeometryView(p,disk.operand)).status
        !=saved_cut_bore_clearance::Status::ClearRecipeDisk)return false;
    return true;
}
struct Bore {unsigned face=0,seam=0,operand=0;std::vector<unsigned> openings;};
inline bool RepresentationOwners(const Graph& g,const std::map<unsigned,unsigned>& seams,Budget& budget){
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
                const auto seam=seams.find(i);
                if(rep->IsCurveOnClosedSurface()&&(seam==seams.end()||owner!=seam->second||owners.size()!=1))return false;
                if(seam!=seams.end()&&!rep->IsCurveOnClosedSurface())return false;
            }else if(rep->IsRegularity()){
                const auto& a=g.faces[edge.faceIDs[0]].surface;const auto& b=g.faces[edge.faceIDs[1]].surface;
                if(!d::PairLocation(a.location,edge.shape.Location(),budget)||!d::PairLocation(b.location,edge.shape.Location(),budget))return false;
                const auto la=a.location.Predivided(edge.shape.Location()),lb=b.location.Predivided(edge.shape.Location());
                if(!d::Location(la,budget)||!d::Location(lb,budget)||!rep->IsRegularity(a.handle,b.handle,la,lb))return false;
            }else if(!rep->IsPolygon3D()&&!rep->IsPolygonOnTriangulation())return false;
        }if(curves!=1)return false;
    }return true;
}
inline bool VertexLinks(const Graph& g,const TopTools_IndexedMapOfShape& vertices,const std::vector<Bore>& bores,const std::vector<old::detail::HostWall>& hosts={}){
    using Adjacency=std::map<unsigned,std::set<unsigned>>;
    std::vector<Adjacency> links(vertices.Extent());std::vector<std::set<unsigned>> germs(vertices.Extent());
    const auto vertex=[&](const TopoDS_Vertex& v)->unsigned {return unsigned(vertices.FindIndex(v)-1);};
    for(unsigned e=0;e<g.edges.size();++e)for(unsigned end=0;end<2;++end)germs[vertex(g.edges[e].vertices[end])].insert(2*e+end);
    const auto arc=[&](unsigned v,unsigned a,unsigned b){
        if(v>=links.size()||a==b||!germs[v].count(a)||!germs[v].count(b))return false;
        return links[v][a].insert(b).second&&links[v][b].insert(a).second;
    };
    for(unsigned f=0;f<g.faces.size();++f){if(std::any_of(bores.begin(),bores.end(),[&](const Bore& b){return f==b.face;})
            ||std::any_of(hosts.begin(),hosts.end(),[&](const old::detail::HostWall& host){return f==host.face;}))continue;
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
    for(const auto& bore:bores)for(unsigned opening:bore.openings){const auto& e=g.edges[opening];const auto seam=bore.seam;unsigned matched=0,index=0;
        for(unsigned n=0;n<2;++n)if(e.vertices[0].IsSame(g.edges[seam].vertices[n])){++matched;index=n;}
        if(matched!=1||!arc(vertex(e.vertices[0]),2*opening,2*seam+index)
            ||!arc(vertex(e.vertices[0]),2*opening+1,2*seam+index))return false;
    }
    for(const auto& host:hosts)for(unsigned rim:host.rims){const auto& e=g.edges[rim];unsigned matched=0,index=0;
        for(unsigned k=0;k<2;++k)if(e.vertices[0].IsSame(g.edges[host.seam].vertices[k])){++matched;index=k;}
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
} // detail
inline Inspection Inspect(const TopoDS_Shape& result,const retained_boolean::Program& program,const std::atomic_bool& stop) noexcept {
    Inspection report;const auto fail=[&](){report.classification=stop.load()?Classification::Cancelled:Classification::Refused;return report;};
    try {
        std::vector<retained_boolean::Disk> disks;
        if(stop.load()||!retained_boolean::ExpandedDisks(program,disks)||!detail::SeparateDisks(program))return fail();
        const auto newSource=detail::GeometryView(program,0);old::Expected expected;
        if(!old::ExpectedSource(newSource,expected)||expected.caps[0]>=expected.faces.size()
            ||expected.caps[1]>=expected.faces.size()||expected.caps[0]==expected.caps[1])return fail();
        // Each pierced source cap gains one loop per validated expanded disk.
        // Derive the collection allowance from the independent recipe, never
        // from observed output; all exact face/loop matching below still applies.
        std::size_t maximumFaceWires=1;
        for(std::size_t f=0;f<expected.faces.size();++f){
            const auto sourceWires=expected.faces[f].wires.size();
            if(sourceWires==0||sourceWires>2)return fail();
            const auto openingWires=(f==expected.caps[0]||f==expected.caps[1])?disks.size():0;
            maximumFaceWires=std::max(maximumFaceWires,sourceWires+openingWires);
        }
        for(const auto& disk:disks){
            const auto view=detail::GeometryView(program,disk.operand);
            const auto bore=saved_cut_bore_result::Inspect(result,view,view,stop,maximumFaceWires);
            report.phase=bore.phase;
            if(bore.status!=saved_cut_bore_result::Status::BoreWallObservedExteriorUnproven)return fail();
        }
        detail::Graph graph;report.phase="program-collect";
        if(!old::detail::Collect(result,newSource,stop,graph,maximumFaceWires))return fail();
        const double mm=program.source.metersPerUnit*1000,pi=std::acos(-1.0);const auto n=disks.size();
        if(graph.faces.size()!=expected.faces.size()+n||graph.edges.size()!=expected.edges.size()+3*n)return fail();
        for(const auto& disk:disks)for(double coordinate:disk.operand.point)
            if(!d::Track(std::abs(coordinate),mm,graph.budget))return fail();
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
        report.phase="whole-point-owners";std::vector<old::detail::PointWitness> pointWitnesses;
        if(!old::detail::PointOwners(graph,mm,stop,graph.budget,pointWitnesses))return fail();
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
        if(vertices.Extent()!=int(expected.vertices.size()+2*n)||std::find(claimed.begin(),claimed.end(),false)!=claimed.end())return fail();
        std::vector<int> originalEdge(graph.edges.size(),-1);std::vector<bool> edgeForward(graph.edges.size()),claimedEdges(expected.edges.size(),false);std::vector<unsigned> newEdges;
        for(unsigned i=0;i<graph.edges.size();++i){
            if(stop.load())return fail();const auto& edge=graph.edges[i];
            const int a=originalVertex[vertices.FindIndex(edge.vertices[0])-1],b=originalVertex[vertices.FindIndex(edge.vertices[1])-1];
            if(a<0||b<0){if(a>=0||b>=0)return fail();newEdges.push_back(i);continue;}
            unsigned count=0,id=0;bool forward=false;
            for(unsigned j=0;j<expected.edges.size();++j){const auto& e=expected.edges[j];if((e.start==unsigned(a)&&e.end==unsigned(b))||(e.start==unsigned(b)&&e.end==unsigned(a))){++count;id=j;forward=e.start==unsigned(a);}}
            if(count!=1||claimedEdges[id]||!old::detail::OriginalCurve(edge.curve,expected.edges[id],expected,forward,mm,error))return fail();
            if(!od::Bits(BRep_Tool::Parameter(edge.vertices[0],edge.shape),edge.curve.first)||!od::Bits(BRep_Tool::Parameter(edge.vertices[1],edge.shape),edge.curve.last))return fail();
            claimedEdges[id]=true;originalEdge[i]=int(id);edgeForward[i]=forward;
        }
        if(newEdges.size()!=3*n||std::find(claimedEdges.begin(),claimedEdges.end(),false)!=claimedEdges.end())return fail();
        report.phase="program-faces";std::vector<detail::Bore> bores;
        std::vector<bool> claimedOperands(n,false),claimedFaces(expected.faces.size(),false);
        std::set<unsigned> piercedCaps;std::map<unsigned,unsigned> seams;
        std::map<unsigned,unsigned> openingOperand;
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
            if(totalOriginal==0){
                if(totalNew!=4||face.wires.size()!=1||!face.surface.cylinder)return fail();
                unsigned matches=0,operand=0;const auto& surface=face.surface;
                const double radius=d::Norm(surface.x),yr=d::Norm(surface.y),z=d::Norm(surface.z);
                if(radius<=0||yr<=0||z<=0)return fail();
                for(unsigned k=0;k<n;++k){const auto& tool=disks[k].operand;
                    gp_Vec axis(0,0,0);axis.SetCoord(unsigned(tool.axis)+1,1);
                    const gp_Vec center(tool.point[0],tool.point[1],tool.point[2]);const auto delta=surface.c-center;
                    if(std::abs(radius-tool.radius)*mm<=error&&std::abs(yr-tool.radius)*mm<=error
                        &&d::Norm((surface.z/z).Crossed(axis))*tool.radius*mm<=error
                        &&d::Norm(delta-axis*delta.Dot(axis))*mm<=error){++matches;operand=k;}}
                if(matches!=1||claimedOperands[operand])return fail();claimedOperands[operand]=true;
                detail::Bore bore;bore.face=f;bore.operand=operand;unsigned seamCount=0;
                std::map<unsigned,unsigned> multiplicity;for(const auto& use:face.wires[0])++multiplicity[use.edge];
                if(multiplicity.size()!=3)return fail();
                for(const auto& row:multiplicity){const auto& edge=graph.edges[row.first];
                    if(edge.curve.circle&&row.second==1){bore.openings.push_back(row.first);
                        if(!openingOperand.emplace(row.first,operand).second)return fail();}
                    else if(!edge.curve.circle&&row.second==2){bore.seam=row.first;++seamCount;
                        if(!seams.emplace(row.first,f).second)return fail();}else return fail();}
                if(seamCount!=1||bore.openings.size()!=2)return fail();bores.push_back(std::move(bore));continue;
            }
            if(totalNew!=extraOpenings||extraOpenings>n)return fail();
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
            if(extraOpenings!=(cap?n:0u))return fail();if(cap&&!piercedCaps.insert(role).second)return fail();
        }
        if(bores.size()!=n||seams.size()!=n||openingOperand.size()!=2*n||piercedCaps.size()!=2
            ||std::find(claimedOperands.begin(),claimedOperands.end(),false)!=claimedOperands.end()
            ||std::find(claimedFaces.begin(),claimedFaces.end(),false)!=claimedFaces.end())return fail();
        // Both actual cap faces contain exactly one opening from EACH bore.
        for(unsigned f=0;f<graph.faces.size();++f){std::set<unsigned> seen;
            for(const auto& wire:graph.faces[f].wires)if(wire.size()==1){const auto found=openingOperand.find(wire[0].edge);
                if(found!=openingOperand.end()&&!seen.insert(found->second).second)return fail();}
            if(!seen.empty()&&seen.size()!=n)return fail();
        }
        report.phase="whole-pcurves";
        for(const auto& entry:graph.pcurves){
            if(stop.load())return fail();const auto& surface=graph.faces[entry.first.first].surface;const auto& c=graph.edges[entry.first.second].curve;
            for(const auto& pc:entry.second){const bool fullPlanar=!surface.cylinder&&c.circle&&c.last-c.first>pi;
                if(fullPlanar?!old::detail::FullPlanarCircleIdentity(c,pc,surface,mm,error):!d::PCurveIdentity(c,pc,surface,mm,error))return fail();}
        }
        report.phase="whole-host-seams";std::vector<old::detail::HostWall> hosts;
        if(!old::detail::HostWalls(graph,expected,originalEdge,originalFace,mm,error,hosts))return fail();
        for(const auto& host:hosts)if(!seams.emplace(host.seam,host.face).second)return fail();
        report.phase="representation-owners";
        if(!detail::RepresentationOwners(graph,seams,graph.budget))return fail();
        // Ownership inspection may evaluate bounded location products, but it
        // may not increase the already fixed arithmetic allowance.
        if(graph.budget.arithmeticMagnitudeMM>fixedArithmetic||graph.budget.maximumLocationCompositionMagnitude>fixedComposition)return fail();
        report.phase="whole-vertex-links";if(!detail::VertexLinks(graph,vertices,bores,hosts))return fail();
        long cellEuler=long(vertices.Extent())-long(graph.edges.size());
        for(const auto& face:graph.faces)cellEuler+=2-long(face.wires.size());
        if(cellEuler!=2-2*long(expected.hostGenus+n))return fail(); // supplementary to complete mapped cells/links
        report.phase="kernel-validity";if(stop.load()||!BRepCheck_Analyzer(result,Standard_True).IsValid())return fail();
        if(stop.load())return fail();BRepClass3d_SolidClassifier outside(result);
        outside.PerformInfinitePoint(Precision::Confusion());
        if(stop.load()||outside.State()!=TopAbs_OUT)return fail();
        if(stop.load())return fail();report.vertices=vertices.Extent();report.edges=graph.edges.size();report.faces=graph.faces.size();report.vertexLinks=vertices.Extent();
        report.phase="matched-complete-program-boundary";report.classification=Classification::MatchedOrientedBoundary;return report;
    }catch(...){return fail();}
}
} // core3d::saved_boolean_result
