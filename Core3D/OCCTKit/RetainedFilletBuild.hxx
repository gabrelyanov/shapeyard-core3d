#pragma once
#include "SavedBooleanResultCorrespondence.hxx"
#include <BRepAlgoAPI_Section.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <ElCLib.hxx>
#include <map>
#include <functional>
namespace core3d::retained_fillet {
inline bool IsDeclined(Outcome outcome) noexcept {
    switch(outcome){
        case Outcome::DeclinedRadiusAdmission:
        case Outcome::DeclinedAnchorNoMatch:
        case Outcome::DeclinedAnchorAmbiguous:
        case Outcome::DeclinedBudget:
        case Outcome::DeclinedOcctFailure:return true;
        case Outcome::Generic:
        case Outcome::Built:
        case Outcome::Cancelled:return false;
    }
    return false;
}
inline const char* Reason(Outcome o){switch(o){
    case Outcome::Generic:return "fillet.generic";
    case Outcome::DeclinedBudget:return "fillet.budget";
    case Outcome::Built:return "fillet.built";
    case Outcome::DeclinedRadiusAdmission:return "fillet.DeclinedRadiusAdmission";
    case Outcome::DeclinedAnchorNoMatch:return "fillet.DeclinedAnchorNoMatch";
    case Outcome::DeclinedAnchorAmbiguous:return "fillet.DeclinedAnchorAmbiguous";
    case Outcome::Cancelled:return "fillet.cancelled";
    case Outcome::DeclinedOcctFailure:return "fillet.DeclinedOcctFailure";
}return "fillet.unknown";}
#if DEBUG
inline std::atomic_int FailureCount{0};
inline bool ConsumeFailure(){int n=FailureCount.load();while(n>0){if(FailureCount.compare_exchange_weak(n,n-1))return true;}return false;}
#endif
inline gp_Pnt Point(const EdgeAnchor& a){return {a.anchorPoint[0],a.anchorPoint[1],a.anchorPoint[2]};}
inline gp_Dir Axis(const EdgeAnchor& a){return {a.axis[0],a.axis[1],a.axis[2]};}
inline bool OnSegment(const gp_Pnt& p,const gp_Pnt& a,const gp_Pnt& b,double tol){
    const gp_Vec v(a,b),w(a,p);const double length=v.Magnitude();if(length<=tol)return false;
    const double t=w.Dot(v)/length;return t>=-tol&&t<=length+tol&&w.Crossed(v).Magnitude()/length<=tol;
}
inline bool Matches(const TopoDS_Edge& edge,const EdgeAnchor& anchor,double mm){
    BRepAdaptor_Curve c(edge);const auto p=Point(anchor);const auto axis=Axis(anchor);const double tol=1e-4/mm;
    if(anchor.curveKind==CurveKind::Line){return c.GetType()==GeomAbs_Line
        &&c.Line().Direction().IsParallel(axis,1e-8)&&OnSegment(p,c.Value(c.FirstParameter()),c.Value(c.LastParameter()),tol);}
    if(c.GetType()!=GeomAbs_Circle)return false;const auto circle=c.Circle();
    if(!circle.Axis().Direction().IsParallel(axis,1e-8)||std::abs(circle.Radius()-anchor.circleRadius)>tol)return false;
    const gp_Vec delta(circle.Location(),p);const double height=delta.Dot(gp_Vec(circle.Axis().Direction()));
    if(std::abs(height)>tol||std::abs(std::sqrt(std::max(0.,delta.SquareMagnitude()-height*height))-circle.Radius())>tol)return false;
    // Full and trimmed circles both use endpoint-inclusive resolution.
    double u=ElCLib::Parameter(circle,p);const double first=c.FirstParameter(),last=c.LastParameter(),turn=2*std::acos(-1.);
    while(u<first-tol/circle.Radius())u+=turn;while(u>first+turn)u-=turn;
    return u>=first-tol/circle.Radius()&&u<=last+tol/circle.Radius();
}
inline Outcome Resolve(const TopoDS_Shape& shape,const EdgeAnchor& anchor,double mm,TopoDS_Edge& out) noexcept {
    out.Nullify();try {
        if(!ValidAnchor(anchor,mm)||shape.IsNull())return Outcome::DeclinedAnchorNoMatch;
        TopTools_IndexedMapOfShape edges;TopExp::MapShapes(shape,TopAbs_EDGE,edges);
        if(edges.Extent()>4096)return Outcome::DeclinedBudget;unsigned count=0;
        for(int i=1;i<=edges.Extent();++i){const auto edge=TopoDS::Edge(edges(i));
            if(BRep_Tool::Degenerated(edge)||!Matches(edge,anchor,mm))continue;
            if(++count>1){out.Nullify();return Outcome::DeclinedAnchorAmbiguous;}out=edge;}
        return count==1?Outcome::Built:Outcome::DeclinedAnchorNoMatch;
    }catch(...){out.Nullify();return Outcome::DeclinedOcctFailure;}
}
// A perpendicular section must contain exactly one nondegenerate straight
// chord through the anchor on EACH adjacent planar/cylindrical face. We never
// substitute a bounding-box dimension or a remote face's width.
inline bool RadiusAdmitted(const TopoDS_Shape& shape,const TopoDS_Edge& edge,const EdgeAnchor& anchor,
    double radius,double mm,double* minimumWidth=nullptr) noexcept {
    try {
        if(!Dimension(radius,mm))return false;
        BRepAdaptor_Curve curve(edge);gp_Vec tangent;
        if(curve.GetType()==GeomAbs_Line)tangent=gp_Vec(curve.Line().Direction());
        else if(curve.GetType()==GeomAbs_Circle)tangent=gp_Vec(curve.Circle().Axis().Direction()).Crossed(gp_Vec(curve.Circle().Location(),Point(anchor)));
        else return false;
        const gp_Pln plane(Point(anchor),gp_Dir(tangent));const double tol=1e-4/mm;
        TopTools_IndexedDataMapOfShapeListOfShape owners;TopExp::MapShapesAndAncestors(shape,TopAbs_EDGE,TopAbs_FACE,owners);
        if(!owners.Contains(edge))return false;TopTools_IndexedMapOfShape faces;
        for(TopTools_ListIteratorOfListOfShape it(owners.FindFromKey(edge));it.More();it.Next())faces.Add(it.Value());
        if(faces.Extent()!=2)return false;double width=INFINITY;
        for(int i=1;i<=faces.Extent();++i){const auto face=TopoDS::Face(faces(i));BRepAdaptor_Surface surface(face);
            if(surface.GetType()!=GeomAbs_Plane&&surface.GetType()!=GeomAbs_Cylinder)return false;
            BRepAlgoAPI_Section section(face,plane,Standard_False);section.Approximation(Standard_False);section.Build();
            if(!section.IsDone())return false;unsigned hits=0;double chord=0;
            TopTools_IndexedMapOfShape lines;TopExp::MapShapes(section.Shape(),TopAbs_EDGE,lines);
            for(int k=1;k<=lines.Extent();++k){BRepAdaptor_Curve c(TopoDS::Edge(lines(k)));if(c.GetType()!=GeomAbs_Line)continue;
                const auto a=c.Value(c.FirstParameter()),b=c.Value(c.LastParameter());
                if(OnSegment(Point(anchor),a,b,tol)){++hits;chord=a.Distance(b);}}
            if(hits!=1||!std::isfinite(chord)||chord<=tol)return false;width=std::min(width,chord);
        }
        if(minimumWidth)*minimumWidth=width;
        return radius<width/2;
    }catch(...){return false;}
}
struct Interval {double lower=0,upper=0;std::size_t corners=0;};
// D14: convex rounds REMOVE material, concave rounds ADD. The present proof
// admits convex planar intersections and full circular through-hole rims.
// Unsupported blends decline; no fitted volume or sampled output is an oracle.
inline double StraightLoss(double radius,double theta){return radius*radius*(1/std::tan(theta/2)-(std::acos(-1.)-theta)/2);}
inline double RimLoss(double radius,double boreRadius){const double pi=std::acos(-1.);
    return 2*pi*(boreRadius*radius*radius*(1-pi/4)+radius*radius*radius*(5./6-pi/4));}
inline bool ExpectedRemoval(const retained_boolean::Program& program,const Step& step,Interval& out,const std::vector<TopoDS_Edge>* resolved=nullptr) noexcept {
    out={};try {
        saved_cut_whole_result::Expected expected;
        const bool single=program.steps.size()==1&&program.steps[0].operand.kind==analytic_boolean::OperandKind::Cylinder;
        if(single){if(!saved_cut_whole_result::ExpectedSource(saved_boolean_result::detail::GeometryView(program,0),expected))return false;}
        else if(!saved_boolean_result::detail::ProgramBoundary(program,expected))return false;
        const double mm=program.source.metersPerUnit*1000,tol=1e-4/mm,pi=std::acos(-1.);
        std::map<unsigned,unsigned> corners;std::set<unsigned> usedLines;std::set<std::pair<unsigned,unsigned>> usedCircles;
        std::vector<retained_boolean::Disk> disks;if(!retained_boolean::ExpandedSections(program,disks))return false;
        if(resolved&&resolved->size()!=step.anchors.size())return false;
        double sum=0;std::size_t anchorIndex=0;
        for(const auto& a:step.anchors){
            const TopoDS_Edge* actual=resolved?&resolved->at(anchorIndex):nullptr;++anchorIndex;
            if(a.curveKind==CurveKind::Line){
                unsigned count=0,index=0;
                for(unsigned k=0;k<expected.edges.size();++k){const auto& e=expected.edges[k];if(e.circle)continue;
                    const auto& start=expected.vertices[e.start];const auto& end=expected.vertices[e.end];
                    if(OnSegment(Point(a),start,end,tol)&&gp_Dir(gp_Vec(start,end)).IsParallel(Axis(a),1e-8)){++count;index=k;}}
                if(count!=1||!usedLines.insert(index).second)return false;
                const auto& edge=expected.edges[index];
                if(actual){BRepAdaptor_Curve c(*actual);if(c.GetType()!=GeomAbs_Line)return false;
                    const auto lo=c.Value(c.FirstParameter()),hi=c.Value(c.LastParameter());
                    const auto start=expected.vertices[edge.start],end=expected.vertices[edge.end];
                    if(!((lo.Distance(start)<=tol&&hi.Distance(end)<=tol)||(lo.Distance(end)<=tol&&hi.Distance(start)<=tol)))return false;}
                std::vector<unsigned> owners;bool forward=true;
                for(unsigned f=0;f<expected.faces.size();++f)for(const auto& wire:expected.faces[f].wires)for(const auto& use:wire)
                    if(use.edge==index){if(owners.empty())forward=use.forward;owners.push_back(f);}
                if(owners.size()!=2)return false;const auto& f=expected.faces[owners[0]];const auto& g=expected.faces[owners[1]];
                if(f.cylinder||g.cylinder)return false;
                const gp_Vec n=f.normalOrAxis.Normalized(),m=g.normalOrAxis.Normalized();
                gp_Vec along(expected.vertices[edge.start],expected.vertices[edge.end]);if(!forward)along.Reverse();
                if(along.Dot(n.Crossed(m))<=0)return false; // concave or tangent: no convex bound
                const double theta=pi-std::acos(std::clamp(n.Dot(m),-1.,1.));
                if(theta<=1e-8||theta>=pi-1e-8)return false;
                // Normals, endpoints and hence taper angles all come from the
                // source/operand expectation, recomputed for every source edit.
                sum+=along.Magnitude()*StraightLoss(step.radiusLocal,theta);
                ++corners[edge.start];++corners[edge.end];
            }else{
                unsigned matches=0;
                for(unsigned d=0;d<disks.size();++d){const auto& t=disks[d].operand;
                    if(t.kind!=analytic_boolean::OperandKind::Cylinder||std::abs(t.radius-a.circleRadius)>tol)continue;
                    const unsigned axis=unsigned(t.axis);const gp_Vec direction(axis==0?1:0,axis==1?1:0,axis==2?1:0);
                    if(!gp_Dir(direction).IsParallel(Axis(a),1e-8))continue;
                    for(unsigned cap:expected.caps){const auto& face=expected.faces[cap];
                        if(face.cylinder||!gp_Dir(face.normalOrAxis).IsParallel(Axis(a),1e-8))continue;
                        auto coordinates=t.point;coordinates[axis]=face.origin.Coord(axis+1);const gp_Pnt center(coordinates[0],coordinates[1],coordinates[2]);
                        const gp_Vec delta(center,Point(a));
                        if(std::abs(delta.Dot(direction))>tol||std::abs(delta.Magnitude()-t.radius)>tol)continue;
                        if(actual){BRepAdaptor_Curve c(*actual);
                            if(c.GetType()!=GeomAbs_Circle||c.Circle().Location().Distance(center)>tol
                                ||std::abs(c.LastParameter()-c.FirstParameter()-2*pi)>1e-8)return false;}
                        if(!usedCircles.insert({d,cap}).second)return false;++matches;
                    }
                }
                if(matches!=1)return false;sum+=RimLoss(step.radiusLocal,a.circleRadius);
            }
        }
        for(const auto& entry:corners){if(entry.second>2)return false;if(entry.second==2)++out.corners;}
        if(!std::isfinite(sum)||sum<=0)return false;
        out.upper=sum;out.lower=sum-out.corners*std::pow(step.radiusLocal,3);
        // Relative endpoint tolerance is applied by the verifier, not fitted.
        return out.lower>=0;
    }catch(...){out={};return false;}
}
inline double Volume(const TopoDS_Shape& shape){GProp_GProps props;BRepGProp::VolumeProperties(shape,props,1e-10);return props.Mass();}
inline bool BoundsContained(const TopoDS_Shape& before,const TopoDS_Shape& after,double tol){
    Bnd_Box a,b;BRepBndLib::AddOptimal(before,a,Standard_False,Standard_False);BRepBndLib::AddOptimal(after,b,Standard_False,Standard_False);
    if(a.IsVoid()||b.IsVoid()||a.IsOpen()||b.IsOpen())return false;
    double aa[6],bb[6];a.Get(aa[0],aa[1],aa[2],aa[3],aa[4],aa[5]);b.Get(bb[0],bb[1],bb[2],bb[3],bb[4],bb[5]);
    for(unsigned i=0;i<3;++i)if(!std::isfinite(bb[i])||!std::isfinite(bb[i+3])||bb[i]<aa[i]-tol||bb[i+3]>aa[i+3]+tol||bb[i]>=bb[i+3])return false;
    return true;
}
struct Result {Outcome outcome=Outcome::DeclinedOcctFailure;TopoDS_Shape solid;std::vector<Interval> intervals;};
inline Result Build(const TopoDS_Shape& input,const retained_boolean::Program& program,const std::atomic_bool& stop,
    const std::function<bool(const TopoDS_Shape&)>& charge={}) noexcept {
    Result out;const auto fail=[&](Outcome why){Result r;r.outcome=stop.load()?Outcome::Cancelled:why;CORE3D_CUT_NOTE(Reason(r.outcome));return r;};
    try {
        if(stop.load())return fail(Outcome::Cancelled);
        if(!retained_boolean::Valid(program)||input.IsNull())return fail(Outcome::DeclinedOcctFailure);
        // Fillet never mutates the correspondence-certified pre-fillet carrier.
        BRepBuilderAPI_Copy copy(input,Standard_True,Standard_False);if(!copy.IsDone())return fail(Outcome::DeclinedOcctFailure);
        TopoDS_Shape current=copy.Shape();const double mm=program.source.metersPerUnit*1000;
        for(const auto& step:program.filletSteps){
            if(stop.load())return fail(Outcome::Cancelled);std::vector<TopoDS_Edge> selected;TopTools_IndexedMapOfShape unique;
            for(const auto& anchor:step.anchors){TopoDS_Edge edge;const auto resolved=Resolve(current,anchor,mm,edge);
                if(resolved!=Outcome::Built)return fail(resolved);
                if(unique.Contains(edge))return fail(Outcome::DeclinedAnchorAmbiguous);unique.Add(edge);
                if(!RadiusAdmitted(current,edge,anchor,step.radiusLocal,mm))return fail(Outcome::DeclinedRadiusAdmission);
                selected.push_back(edge);}
            Interval interval;if(!ExpectedRemoval(program,step,interval,&selected))return fail(Outcome::DeclinedOcctFailure);
#if DEBUG
            if(ConsumeFailure())return fail(Outcome::DeclinedOcctFailure);
#endif
            BRepFilletAPI_MakeFillet fillet(current);for(const auto& edge:selected)fillet.Add(step.radiusLocal,edge);
            fillet.Build();if(stop.load())return fail(Outcome::Cancelled);
            if(!fillet.IsDone()||fillet.Shape().IsNull())return fail(Outcome::DeclinedOcctFailure);
            TopoDS_Shape solid=fillet.Shape();
            if(solid.ShapeType()==TopAbs_COMPOUND){TopoDS_Iterator child(solid);
                if(!child.More()||child.Value().ShapeType()!=TopAbs_SOLID)return fail(Outcome::DeclinedOcctFailure);
                const TopoDS_Shape only=child.Value();child.Next();if(child.More())return fail(Outcome::DeclinedOcctFailure);solid=only;}
            if(solid.ShapeType()!=TopAbs_SOLID||solid.Orientation()!=TopAbs_FORWARD
                ||!BRepCheck_Analyzer(solid).IsValid())return fail(Outcome::DeclinedOcctFailure);
            // OCCT must consume precisely the requested edges; tangent chaining
            // to an unanchored edge is an unsupported operation, not a success.
            TopTools_IndexedMapOfShape consumed;
            for(int c=1;c<=fillet.NbContours();++c)for(int e=1;e<=fillet.NbEdges(c);++e)consumed.Add(fillet.Edge(c,e));
            if(consumed.Extent()!=unique.Extent())return fail(Outcome::DeclinedOcctFailure);
            for(int i=1;i<=unique.Extent();++i)if(!consumed.Contains(unique(i)))return fail(Outcome::DeclinedOcctFailure);
            const double before=Volume(current),after=Volume(solid),removed=before-after;
            if(!std::isfinite(before)||!std::isfinite(after)||after<=0
                ||removed<interval.lower-std::abs(interval.lower)*1e-6||removed>interval.upper+std::abs(interval.upper)*1e-6
                ||!BoundsContained(current,solid,1e-4/mm))return fail(Outcome::DeclinedOcctFailure);
            if(charge&&!charge(solid))return fail(Outcome::DeclinedBudget);
            current=solid;out.intervals.push_back(interval);
        }
        out.outcome=Outcome::Built;out.solid=current;return out;
    }catch(...){return fail(Outcome::DeclinedOcctFailure);}
}
}
