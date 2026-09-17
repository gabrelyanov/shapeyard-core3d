#pragma once
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include "SavedCutWholeResultCorrespondence.hxx"

namespace core3d::saved_cut_circular_host {
namespace whole=core3d::saved_cut_whole_result;
namespace d=core3d::enclosure_correspondence::detail;
namespace od=core3d::saved_cut_bore_result::detail;
namespace detail=core3d::saved_cut_whole_result::detail;
enum class Classification {Refused,Cancelled,MatchedBoundary};
using Inspection=whole::Inspection;
inline Classification InspectCylinder(const TopoDS_Shape& base,const ProfileDefinition& definition,
    const std::optional<profile::ConstructionFrame>& frame,double metersPerUnit,
    const std::atomic_bool& stop,Inspection& report) noexcept {
    report={};const auto fail=[&](){report.classification=stop.load()?whole::Classification::Cancelled:whole::Classification::Refused;
        return stop.load()?Classification::Cancelled:Classification::Refused;};
    try {
        if(stop.load()||!std::isfinite(metersPerUnit)||metersPerUnit<=0)return fail();
        Expectation cells;if(!BuildExpectedBoundary(definition,frame,cells))return fail();
        whole::Expected expected;expected.vertices=std::move(cells.vertices);expected.edges=std::move(cells.edges);
        expected.faces=std::move(cells.faces);expected.caps=cells.caps;expected.hostGenus=cells.hostGenus;
        retained_solid::Envelope units;units.metersPerUnit=metersPerUnit;
        detail::Graph graph;report.phase="host-collect";
        if(!detail::Collect(base,units,stop,graph))return fail();
        const double mm=metersPerUnit*1000,pi=std::acos(-1.0);
        if(graph.faces.size()!=expected.faces.size()||graph.edges.size()!=expected.edges.size())return fail();
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
        if(vertices.Extent()!=int(expected.vertices.size())||std::find(claimed.begin(),claimed.end(),false)!=claimed.end())return fail();
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
        if(!newEdges.empty()||std::find(claimedEdges.begin(),claimedEdges.end(),false)!=claimedEdges.end())return fail();
        report.phase="host-faces";std::vector<int> originalFace(graph.faces.size(),-1);
        std::vector<bool> claimedFaces(expected.faces.size(),false);
        for(unsigned f=0;f<graph.faces.size();++f){const auto& face=graph.faces[f];
            unsigned count=0,role=0;
            for(unsigned r=0;r<expected.faces.size();++r){const auto& wanted=expected.faces[r];
                if(wanted.wires.size()!=face.wires.size())continue;std::set<unsigned> used;bool match=true;
                for(const auto& wire:face.wires){std::vector<std::pair<unsigned,bool>> actual;
                    for(const auto& use:wire)actual.emplace_back(unsigned(originalEdge[use.edge]),use.forward==edgeForward[use.edge]);
                    unsigned n=0,id=0;for(unsigned w=0;w<wanted.wires.size();++w)if(d::WireMatch(actual,wanted.wires[w])){++n;id=w;}
                    if(n!=1||!used.insert(id).second){match=false;break;}}
                if(match){++count;role=r;}}
            d::Face surfaceView;surfaceView.face=face.shape;surfaceView.surface=face.surface;
            if(count!=1||claimedFaces[role]||!d::MatchSurface(surfaceView,expected.faces[role],mm,error))return fail();
            claimedFaces[role]=true;originalFace[f]=int(role);
        }
        if(std::find(claimedFaces.begin(),claimedFaces.end(),false)!=claimedFaces.end())return fail();
        report.phase="host-pcurves";
        for(const auto& entry:graph.pcurves){if(stop.load())return fail();
            const auto& surface=graph.faces[entry.first.first].surface;const auto& c=graph.edges[entry.first.second].curve;
            for(const auto& pc:entry.second){const bool fullPlanar=!surface.cylinder&&c.circle&&c.last-c.first>pi;
                if(fullPlanar?!detail::FullPlanarCircleIdentity(c,pc,surface,mm,error):!d::PCurveIdentity(c,pc,surface,mm,error))return fail();}}
        report.phase="host-seams";std::vector<detail::HostWall> hosts;
        if(!detail::HostWalls(graph,expected,originalEdge,originalFace,mm,error,hosts)||hosts.size()!=1+expected.hostGenus)return fail();
        std::map<unsigned,unsigned> hostSeams;for(const auto& host:hosts)if(!hostSeams.emplace(host.seam,host.face).second)return fail();
        const unsigned absent=std::numeric_limits<unsigned>::max();
        report.phase="representation-owners";
        if(!detail::RepresentationOwners(graph,absent,absent,graph.budget,hostSeams))return fail();
        if(graph.budget.arithmeticMagnitudeMM>fixedArithmetic||graph.budget.maximumLocationCompositionMagnitude>fixedComposition)return fail();
        report.phase="host-vertex-links";if(!detail::VertexLinks(graph,vertices,absent,absent,{},hosts))return fail();
        long euler=long(vertices.Extent())-long(graph.edges.size());for(const auto& face:graph.faces)euler+=2-long(face.wires.size());
        if(euler!=2-2*long(expected.hostGenus))return fail();
        report.phase="host-kernel-validity";if(stop.load()||!BRepCheck_Analyzer(base,Standard_True).IsValid())return fail();
        BRepClass3d_SolidClassifier outside(base);outside.PerformInfinitePoint(Precision::Confusion());
        if(stop.load()||outside.State()!=TopAbs_OUT)return fail();
        report.vertices=vertices.Extent();report.edges=graph.edges.size();report.faces=graph.faces.size();report.vertexLinks=vertices.Extent();
        report.phase="matched-circular-host-boundary";report.classification=whole::Classification::MatchedOrientedBoundary;
        return Classification::MatchedBoundary;
    }catch(...){return fail();}
}
} // namespace core3d::saved_cut_circular_host
