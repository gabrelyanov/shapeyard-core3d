#pragma once
// External, uncompiled prototype. A read-only analytic classifier, not native
// ownership/commit authority. Numeric policy and reopened fixtures unqualified.
#include "PrismBoundaryExpectation.hxx"
#include "PrismLocationPreflight.hxx"
#include "ProfileConstructionFrame.hxx"
#include <BRep_Tool.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_TVertex.hxx>
#include <BRep_ListIteratorOfListOfPointRepresentation.hxx>
#include <BRep_CurveRepresentation.hxx>
#include <BRep_ListIteratorOfListOfCurveRepresentation.hxx>
#include <Geom_Line.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_Plane.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Lin.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Pln.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <atomic>
#include <limits>
#include <set>

namespace core3d::saved_cut_prism_prototype {
enum class Classification { Refused, Cancelled, MatchedBoundary };
struct Inspection {
    std::size_t vertices=0,edges=0,faces=0,storedPCurves=0,generatedPCurves=0;
    double algebraicErrorMM=0,maximumKernelToleranceMM=0,minimumVertexSeparationMM=0,minimumBoundarySeparationMM=0,arithmeticMagnitudeMM=0,compositionErrorMM=0;
};
namespace extraction {
inline bool Oriented(const TopoDS_Shape& s){return s.Orientation()==TopAbs_FORWARD||s.Orientation()==TopAbs_REVERSED;}
inline bool Finite(const gp_Pnt& p){return std::isfinite(p.X())&&std::isfinite(p.Y())&&std::isfinite(p.Z());}
inline bool IsLine(Handle(Geom_Curve) c,double first,double last){
    for(unsigned i=0;i<8&&!c.IsNull();++i){
        const double lower=c->FirstParameter(),upper=c->LastParameter();
        if(std::isnan(lower)||std::isnan(upper)||lower>=upper||first<lower||last>upper)return false;
        if(!Handle(Geom_Line)::DownCast(c).IsNull())return true;
        const auto trimmed=Handle(Geom_TrimmedCurve)::DownCast(c);
        if(trimmed.IsNull())return false;c=trimmed->BasisCurve();
    }return false;
}
inline bool IsLine2D(Handle(Geom2d_Curve) c,double first,double last){
    for(unsigned i=0;i<8&&!c.IsNull();++i){
        const double lower=c->FirstParameter(),upper=c->LastParameter();
        if(std::isnan(lower)||std::isnan(upper)||lower>=upper||first<lower||last>upper)return false;
        if(!Handle(Geom2d_Line)::DownCast(c).IsNull())return true;
        const auto trimmed=Handle(Geom2d_TrimmedCurve)::DownCast(c);
        if(trimmed.IsNull())return false;c=trimmed->BasisCurve();
    }return false;
}
// Track absolute intermediate affine-operation sums, including parameter-origin
// cancellation. 128epsilon conservatively covers two evaluations and the
// explicit source/curve/plane/location affine operation chains (<64 rounded
// operations per component). This proposed contract still needs native tests.
inline bool Budget(const std::array<double,3>& inputSums,const gp_Trsf& transform,double factor,Inspection& report){
    for(unsigned r=0;r<3;++r){
        double sum=std::abs(transform.Value(r+1,4));
        for(unsigned c=0;c<3;++c){
            if(!std::isfinite(inputSums[c])||inputSums[c]<0)return false;
            sum+=std::abs(transform.Value(r+1,c+1))*inputSums[c];
        }
        const double magnitude=sum*factor;
        if(!std::isfinite(magnitude))return false;
        report.arithmeticMagnitudeMM=std::max(report.arithmeticMagnitudeMM,magnitude);
    }
    report.algebraicErrorMM=std::max({report.compositionErrorMM,1e-9,128*std::numeric_limits<double>::epsilon()*report.arithmeticMagnitudeMM});
    // Numeric conditioning refusal, not a geometry tolerance expansion.
    return report.algebraicErrorMM<=1e-6;
}
inline bool LocationBudget(const std::array<double,3>& inputSums,const TopLoc_Location& location,double factor,Inspection& report){
    locations::Matrix bound;double errorUnits=0;
    if(!locations::Composition(location,bound,errorUnits))return false;
    double magnitude=0;
    for(unsigned r=0;r<3;++r){
        double sum=bound[r][3];for(unsigned c=0;c<3;++c)sum+=bound[r][c]*inputSums[c];
        if(!std::isfinite(sum*factor))return false;magnitude=std::max(magnitude,sum*factor);
    }
    const double error=128*std::numeric_limits<double>::epsilon()*errorUnits*magnitude;
    if(!std::isfinite(error))return false;report.compositionErrorMM=std::max(report.compositionErrorMM,error);
    return Budget(inputSums,location.Transformation(),factor,report);
}
inline Handle(Geom_Line) LineBasis(Handle(Geom_Curve) curve){
    for(unsigned i=0;i<8&&!curve.IsNull();++i){
        const auto line=Handle(Geom_Line)::DownCast(curve);if(!line.IsNull())return line;
        const auto trim=Handle(Geom_TrimmedCurve)::DownCast(curve);if(trim.IsNull())break;curve=trim->BasisCurve();
    }return {};
}
inline Handle(Geom2d_Line) LineBasis2D(Handle(Geom2d_Curve) curve){
    for(unsigned i=0;i<8&&!curve.IsNull();++i){
        const auto line=Handle(Geom2d_Line)::DownCast(curve);if(!line.IsNull())return line;
        const auto trim=Handle(Geom2d_TrimmedCurve)::DownCast(curve);if(trim.IsNull())break;curve=trim->BasisCurve();
    }return {};
}
inline bool CurveBudget(const Handle(Geom_Curve)& curve,double first,double last,const TopLoc_Location& location,double factor,Inspection& report){
    const auto line=LineBasis(curve);if(line.IsNull())return false;const auto basis=line->Lin();
    const auto origin=basis.Location();const auto direction=basis.Direction();const double extent=std::max(std::abs(first),std::abs(last));
    return LocationBudget({std::abs(origin.X())+extent*std::abs(direction.X()),std::abs(origin.Y())+extent*std::abs(direction.Y()),
        std::abs(origin.Z())+extent*std::abs(direction.Z())},location,factor,report);
}
inline bool PCurveBudget(const Handle(Geom2d_Curve)& curve,double first,double last,const Handle(Geom_Plane)& plane,
    const TopLoc_Location& location,double factor,Inspection& report){
    const auto line=LineBasis2D(curve);if(line.IsNull())return false;const auto basis=line->Lin2d();
    const double extent=std::max(std::abs(first),std::abs(last));
    const double u=std::abs(basis.Location().X())+extent*std::abs(basis.Direction().X());
    const double v=std::abs(basis.Location().Y())+extent*std::abs(basis.Direction().Y());
    const auto axes=plane->Pln().Position();const auto p=axes.Location();const auto x=axes.XDirection(),y=axes.YDirection();
    return LocationBudget({std::abs(p.X())+u*std::abs(x.X())+v*std::abs(y.X()),std::abs(p.Y())+u*std::abs(x.Y())+v*std::abs(y.Y()),
        std::abs(p.Z())+u*std::abs(x.Z())+v*std::abs(y.Z())},location,factor,report);
}
struct Edge {
    TopoDS_Edge shape;Handle(Geom_Curve) curve;TopLoc_Location location;
    double first=0,last=0;std::size_t start=0,end=0;
    std::vector<TopoDS_Face> faces;unsigned forwardUses=0,reverseUses=0;
};
inline bool Close(const gp_Pnt& a,const gp_Pnt& b,double factor,double error){
    return Finite(a)&&Finite(b)&&std::isfinite(a.Distance(b)*factor)&&a.Distance(b)*factor<=error;
}
inline bool Tolerance(double value,double factor,const TopoDS_Shape& shape,Inspection& report){
    const double scale=shape.Location().Transformation().ScaleFactor();
    if(!std::isfinite(scale)||scale<=0)return false;
    const double mm=value*scale*factor;
    if(!std::isfinite(mm)||mm<0||mm>0.001)return false;
    report.maximumKernelToleranceMM=std::max(report.maximumKernelToleranceMM,mm);return true;
}
// Direct topology traversal, never a recursive set which erases multiplicity.
inline bool Children(const TopoDS_Shape& parent,TopAbs_ShapeEnum kind,std::size_t limit,std::vector<TopoDS_Shape>& out){
    out.clear();for(TopoDS_Iterator it(parent);it.More();it.Next()){
        const auto& child=it.Value();
        if(out.size()>=limit||child.IsNull()||child.ShapeType()!=kind||!Oriented(child))return false;
        out.push_back(child);
    }return true;
}
}
inline Classification InspectPrism(const TopoDS_Shape& retained,const ProfileDefinition& recipe,
    const std::optional<profile::ConstructionFrame>& sourceFrame,double metersPerUnit,
    const std::atomic_bool& stop,Inspection& output) noexcept {
    using namespace extraction;output={};Inspection report;
    try {
        if(stop.load())return Classification::Cancelled;
        Expectation expected;if(!BuildExpectedBoundary(recipe,expected))return Classification::Refused;
        const auto n=recipe.points.size();const double factor=metersPerUnit*1000;
        if(!std::isfinite(metersPerUnit)||metersPerUnit<=0||!std::isfinite(factor)||factor<=0)return Classification::Refused;
        gp_Trsf frame;
        if(sourceFrame&&(!sourceFrame->IsValid()||sourceFrame->values[7]<=0||!sourceFrame->Transform(frame)))return Classification::Refused;
        for(auto& p:expected.vertices){
            if(!Budget({std::abs(p.X()),std::abs(p.Y()),std::abs(p.Z())},frame,factor,report))return Classification::Refused;
            p.Transform(frame);if(!Finite(p))return Classification::Refused;
        }
        report.minimumVertexSeparationMM=std::numeric_limits<double>::infinity();
        for(std::size_t i=0;i<expected.vertices.size();++i)for(std::size_t j=0;j<i;++j)
            report.minimumVertexSeparationMM=std::min(report.minimumVertexSeparationMM,expected.vertices[i].Distance(expected.vertices[j])*factor);
        if(!std::isfinite(report.minimumVertexSeparationMM)||report.minimumVertexSeparationMM<=2*report.algebraicErrorMM)return Classification::Refused;
        // Nonincident polygon boundaries also need separated uncertainty
        // neighborhoods; unique vertices alone cannot protect a very thin notch.
        const double sourceScale=sourceFrame?sourceFrame->values[7]:1;
        report.minimumBoundarySeparationMM=recipe.depth*sourceScale*factor;
        for(std::size_t v=0;v<n;++v)for(std::size_t e=0;e<n;++e){
            const auto next=(e+1)%n;if(v==e||v==next)continue;
            const auto p=recipe.points[v],a=recipe.points[e],b=recipe.points[next];
            const double dx=b.X()-a.X(),dy=b.Y()-a.Y(),length2=dx*dx+dy*dy;
            if(!std::isfinite(length2)||length2<=0)return Classification::Refused;
            const double t=std::max(0.0,std::min(1.0,((p.X()-a.X())*dx+(p.Y()-a.Y())*dy)/length2));
            const double clearance=std::hypot(p.X()-a.X()-t*dx,p.Y()-a.Y()-t*dy)*sourceScale*factor;
            if(!std::isfinite(clearance))return Classification::Refused;
            report.minimumBoundarySeparationMM=std::min(report.minimumBoundarySeparationMM,clearance);
        }
        if(retained.IsNull()||retained.ShapeType()!=TopAbs_SOLID||retained.Orientation()!=TopAbs_FORWARD)return Classification::Refused;
        if(!locations::RawShape(retained,stop))return stop.load()?Classification::Cancelled:Classification::Refused;
        std::vector<TopoDS_Shape> shells,faces;
        if(!Children(retained,TopAbs_SHELL,1,shells)||shells.size()!=1
            ||!Children(shells[0],TopAbs_FACE,n+2,faces)||faces.size()!=n+2)return Classification::Refused;
        // Preflight ALL bounded affine representations before any vertex match.
        // The comparison allowance must not depend on face/edge iteration order.
        for(const auto& rawFace:faces){
            if(stop.load())return Classification::Cancelled;
            const auto face=TopoDS::Face(rawFace);TopLoc_Location surfaceLocation;
            const auto surface=BRep_Tool::Surface(face,surfaceLocation);const auto plane=Handle(Geom_Plane)::DownCast(surface);
            if(plane.IsNull()||surfaceLocation.Transformation().ScaleFactor()<=0)return Classification::Refused;
            std::vector<TopoDS_Shape> wires,uses;
            if(!Children(face,TopAbs_WIRE,1,wires)||wires.size()!=1
                ||!Children(wires[0],TopAbs_EDGE,std::max(n,std::size_t(4)),uses)||uses.size()<3)return Classification::Refused;
            for(const auto& rawEdge:uses){
                if(stop.load())return Classification::Cancelled;
                const auto edge=TopoDS::Edge(rawEdge.Oriented(TopAbs_FORWARD));
                const auto data=Handle(BRep_TEdge)::DownCast(edge.TShape());if(data.IsNull())return Classification::Refused;
                unsigned count=0;for(BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());it.More();it.Next())
                    if(++count>16||it.Value().IsNull())return Classification::Refused;
                std::vector<TopoDS_Shape> vertices;
                if(!Children(edge,TopAbs_VERTEX,2,vertices)||vertices.size()!=2)return Classification::Refused;
                for(const auto& vertex:vertices){
                    const auto dataVertex=Handle(BRep_TVertex)::DownCast(vertex.TShape());if(dataVertex.IsNull())return Classification::Refused;
                    unsigned points=0;for(BRep_ListIteratorOfListOfPointRepresentation it(dataVertex->Points());it.More();it.Next())
                        if(++points>16||it.Value().IsNull())return Classification::Refused;
                    const auto p=dataVertex->Pnt();if(!Finite(p)
                        ||!LocationBudget({std::abs(p.X()),std::abs(p.Y()),std::abs(p.Z())},vertex.Location(),factor,report))return Classification::Refused;
                }
                TopLoc_Location location;double first=0,last=0;
                const auto curve=BRep_Tool::Curve(edge,location,first,last);
                if(!std::isfinite(first)||!std::isfinite(last)||first>=last||location.Transformation().ScaleFactor()<=0
                    ||!IsLine(curve,first,last)||!CurveBudget(curve,first,last,location,factor,report))return Classification::Refused;
                double pFirst=0,pLast=0;const auto pc=BRep_Tool::CurveOnSurface(edge,face,pFirst,pLast);
                if(pFirst!=first||pLast!=last||!IsLine2D(pc,pFirst,pLast)
                    ||!PCurveBudget(pc,pFirst,pLast,plane,surfaceLocation,factor,report))return Classification::Refused;
            }
        }
        const double comparisonErrorMM=report.algebraicErrorMM;
        TopTools_IndexedMapOfShape vertexMap,edgeMap,faceMap,wireMap;
        std::vector<std::size_t> vertexExpected;std::vector<Edge> edges;
        std::vector<bool> expectedFaces(n+2,false),expectedVertices(2*n,false);
        auto vertexIndex=[&](const TopoDS_Vertex& vertex,std::size_t& index)->bool{
            if(vertex.IsNull()||!Oriented(vertex)||TopoDS_Iterator(vertex).More()
                ||!Tolerance(BRep_Tool::Tolerance(vertex),factor,vertex,report))return false;
            const int found=vertexMap.FindIndex(vertex);
            if(found){index=vertexExpected[found-1];return true;}
            if(vertexMap.Extent()>=int(2*n))return false;
            const gp_Pnt actual=BRep_Tool::Pnt(vertex);std::size_t match=0,count=0;
            for(std::size_t j=0;j<expected.vertices.size();++j)
                if(Close(actual,expected.vertices[j],factor,comparisonErrorMM)){match=j;++count;}
            if(count!=1||expectedVertices[match])return false;
            expectedVertices[match]=true;vertexMap.Add(vertex);vertexExpected.push_back(match);index=match;return true;
        };
        for(const auto& rawFace:faces){
            if(stop.load())return Classification::Cancelled;
            const auto face=TopoDS::Face(rawFace);
            if(faceMap.Contains(face)||!Tolerance(BRep_Tool::Tolerance(face),factor,face,report))return Classification::Refused;
            faceMap.Add(face);TopLoc_Location surfaceLocation;
            const auto surface=BRep_Tool::Surface(face,surfaceLocation);
            const auto plane=Handle(Geom_Plane)::DownCast(surface);
            // Untrimmed analytic planes only; explicit complete wires define the trim.
            if(plane.IsNull()||surfaceLocation.Transformation().ScaleFactor()<=0)return Classification::Refused;
            std::vector<TopoDS_Shape> wires,uses;
            if(!Children(face,TopAbs_WIRE,1,wires)||wires.size()!=1||wireMap.Contains(wires[0])
                ||!Children(wires[0],TopAbs_EDGE,std::max(n,std::size_t(4)),uses)||uses.size()<3)return Classification::Refused;
            wireMap.Add(wires[0]);std::set<std::pair<std::size_t,std::size_t>> directed;
            for(const auto& rawEdge:uses){
                if(stop.load())return Classification::Cancelled;
                const auto edge=TopoDS::Edge(rawEdge);
                if(BRep_Tool::Degenerated(edge)||BRep_Tool::IsClosed(edge,face)
                    ||!BRep_Tool::SameParameter(edge)||!BRep_Tool::SameRange(edge)
                    ||!Tolerance(BRep_Tool::Tolerance(edge),factor,edge,report))return Classification::Refused;
                std::vector<TopoDS_Shape> children;
                if(!Children(edge,TopAbs_VERTEX,2,children)||children.size()!=2)return Classification::Refused;
                // Use forward edge parameter order; keep effective face-use direction separately.
                const auto forward=TopoDS::Edge(edge.Oriented(TopAbs_FORWARD));TopoDS_Vertex a,b;
                TopExp::Vertices(forward,a,b,Standard_True);std::size_t ia=0,ib=0;
                if(!vertexIndex(a,ia)||!vertexIndex(b,ib)||ia==ib)return Classification::Refused;
                const bool useForward=edge.Orientation()==TopAbs_FORWARD;
                if(!directed.emplace(useForward?ia:ib,useForward?ib:ia).second)return Classification::Refused;
                int index=edgeMap.FindIndex(edge);
                if(!index){
                    if(edgeMap.Extent()>=int(3*n))return Classification::Refused;
                    Edge e;e.shape=forward;e.start=ia;e.end=ib;
                    e.curve=BRep_Tool::Curve(forward,e.location,e.first,e.last);
                    if(e.location.Transformation().ScaleFactor()<=0||!IsLine(e.curve,e.first,e.last)||!std::isfinite(e.first)||!std::isfinite(e.last)||e.first>=e.last)return Classification::Refused;
                    const auto start=e.curve->Value(e.first).Transformed(e.location.Transformation());
                    const auto end=e.curve->Value(e.last).Transformed(e.location.Transformation());
                    if(!Close(start,expected.vertices[ia],factor,comparisonErrorMM)
                        ||!Close(end,expected.vertices[ib],factor,comparisonErrorMM))return Classification::Refused;
                    // Parameters bound the same complete affine segment, not merely sampled support points.
                    const double va=BRep_Tool::Parameter(a,forward),vb=BRep_Tool::Parameter(b,forward);
                    if(va!=e.first||vb!=e.last)return Classification::Refused;
                    index=edgeMap.Add(edge);edges.push_back(e);
                }
                auto& e=edges[index-1];if(e.start!=ia||e.end!=ib||e.faces.size()>=2)return Classification::Refused;
                e.faces.push_back(face);if(useForward)++e.forwardUses;else ++e.reverseUses;
                double first=0,last=0;Standard_Boolean stored=Standard_False;
                const auto pc=BRep_Tool::CurveOnSurface(forward,face,first,last,&stored);
                if(!IsLine2D(pc,first,last)||first!=e.first||last!=e.last)return Classification::Refused;
                if(stored)++report.storedPCurves;else ++report.generatedPCurves;
                // Plane(line2D(t)) and line3D(t) are affine over the SAME complete
                // finite parameter interval. Endpoint agreement bounds the entire
                // interval by convexity; no curved geometry is inferred from samples.
                for(double parameter:{first,last}){
                    const auto uv=pc->Value(parameter);
                    if(!std::isfinite(uv.X())||!std::isfinite(uv.Y()))return Classification::Refused;
                    const auto onFace=surface->Value(uv.X(),uv.Y()).Transformed(surfaceLocation.Transformation());
                    const auto onCurve=e.curve->Value(parameter).Transformed(e.location.Transformation());
                    if(!Close(onFace,onCurve,factor,comparisonErrorMM))return Classification::Refused;
                }
            }
            std::size_t faceMatch=0,faceCount=0;
            for(std::size_t j=0;j<expected.faces.size();++j){
                const auto& cycle=expected.faces[j].vertices;std::set<std::pair<std::size_t,std::size_t>> wanted;
                for(std::size_t k=0;k<cycle.size();++k)wanted.emplace(cycle[k],cycle[(k+1)%cycle.size()]);
                if(wanted==directed){faceMatch=j;++faceCount;}
            }
            if(faceCount!=1||expectedFaces[faceMatch])return Classification::Refused;expectedFaces[faceMatch]=true;
            // Full ordered simple cycle determines the finite trimmed region.
            // Separately retain face orientation: a reversed support/wire pair
            // must not be mistaken for the expected outward solid.
            const auto& cycle=expected.faces[faceMatch].vertices;gp_Vec expectedNormal(0,0,0);
            const auto origin=expected.vertices[cycle[0]];
            for(std::size_t k=1;k+1<cycle.size();++k)
                expectedNormal+=gp_Vec(origin,expected.vertices[cycle[k]]).Crossed(gp_Vec(origin,expected.vertices[cycle[k+1]]));
            gp_Vec actualNormal(plane->Pln().Axis().Direction());actualNormal.Transform(surfaceLocation.Transformation());
            if(face.Orientation()==TopAbs_REVERSED)actualNormal.Reverse();
            const double sign=expectedNormal.Dot(actualNormal);
            if(!std::isfinite(sign)||sign<=0)return Classification::Refused;
        }
        if(vertexMap.Extent()!=int(2*n)||edgeMap.Extent()!=int(3*n)||faceMap.Extent()!=int(n+2))return Classification::Refused;
        for(const auto& e:edges){
            if(stop.load())return Classification::Cancelled;
            if(e.forwardUses!=1||e.reverseUses!=1||e.faces.size()!=2)return Classification::Refused;
            const auto data=Handle(BRep_TEdge)::DownCast(e.shape.TShape());if(data.IsNull())return Classification::Refused;
            unsigned count=0,curves3D=0;std::array<unsigned,2> facePCurves{};
            for(BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());it.More();it.Next()){
                if(++count>16||it.Value().IsNull())return Classification::Refused;const auto& rep=it.Value();
                if(rep->IsCurve3D()){if(++curves3D!=1)return Classification::Refused;continue;}
                if(rep->IsCurveOnClosedSurface())return Classification::Refused;
                if(rep->IsCurveOnSurface()){
                    unsigned matches=0;
                    for(unsigned j=0;j<2;++j){TopLoc_Location loc;const auto surf=BRep_Tool::Surface(e.faces[j],loc);
                        if(rep->IsCurveOnSurface(surf,loc.Predivided(e.shape.Location()))){++matches;if(++facePCurves[j]>1)return Classification::Refused;}}
                    if(matches!=1)return Classification::Refused;
                }else if(!rep->IsPolygon3D()&&!rep->IsPolygonOnTriangulation())return Classification::Refused;
                // Mesh cache representations are not geometric proof and remain unmodified.
            }
            if(curves3D!=1)return Classification::Refused;
        }
        if(std::min(report.minimumVertexSeparationMM,report.minimumBoundarySeparationMM)
            <=2*comparisonErrorMM+4*report.maximumKernelToleranceMM)return Classification::Refused;
        if(stop.load())return Classification::Cancelled;
        report.vertices=vertexMap.Extent();report.edges=edgeMap.Extent();report.faces=faceMap.Extent();
        output=report;return Classification::MatchedBoundary;
    }catch(...){return stop.load()?Classification::Cancelled:Classification::Refused;}
}
} // namespace core3d::saved_cut_prism_prototype
