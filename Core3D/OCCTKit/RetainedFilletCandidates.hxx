#pragma once
#include "RetainedFilletBuild.hxx"
#include <BinTools.hxx>
#include <sstream>
#include <tuple>
namespace core3d::retained_fillet {
inline std::string CandidateShapeBytes(const TopoDS_Shape& shape) {
    std::ostringstream stream;
    BinTools::Write(shape,stream,Standard_True,Standard_True,BinTools_FormatVersion_VERSION_4);
    return stream.str();
}
// Same proof as Build, at the smallest supported physical radius. This is
// individual eligibility, not a promise that an arbitrary set/radius will build.
inline Candidates DiscoverCandidates(const TopoDS_Shape& shape,
    const retained_boolean::Recipe& recipe,const TopoDS_Shape& retainedBase) noexcept {
    Candidates out;
    try {
        if(shape.IsNull()||retainedBase.IsNull())return out;
        retained_boolean::Program program;
        if(const auto* legacy=std::get_if<retained_boolean::Legacy>(&recipe)) {
            if(!retained_boolean::Promote(*legacy,program))return out;
        }else program=std::get<retained_boolean::Program>(recipe);
        if(!retained_boolean::Valid(program)||program.steps.empty())return out;
        TopTools_IndexedMapOfShape inputEdges;TopExp::MapShapes(shape,TopAbs_EDGE,inputEdges);
        if(inputEdges.Extent()>4096){out.status=CandidateStatus::Budget;return out;}
        const auto shapeBefore=CandidateShapeBytes(shape),baseBefore=CandidateShapeBytes(retainedBase);
        std::vector<std::uint8_t> programBefore;
        if(!retained_boolean::Encode(recipe,programBefore)){out.status=CandidateStatus::Failed;return out;}
        BRepBuilderAPI_Copy copy(shape,Standard_True,Standard_False);
        if(!copy.IsDone()){out.status=CandidateStatus::Failed;return out;}
        const auto detached=copy.Shape();const double mm=program.source.metersPerUnit*1000;
        TopTools_IndexedMapOfShape edges;TopExp::MapShapes(detached,TopAbs_EDGE,edges);
        for(int i=1;i<=edges.Extent();++i){
            const auto edge=TopoDS::Edge(edges(i));if(BRep_Tool::Degenerated(edge))continue;
            BRepAdaptor_Curve curve(edge);Candidate value;gp_Pnt point;gp_Dir axis;
            if(curve.GetType()==GeomAbs_Line){
                point=curve.Value((curve.FirstParameter()+curve.LastParameter())/2);
                axis=curve.Line().Direction();
                value.lengthLocal=curve.Value(curve.FirstParameter()).Distance(curve.Value(curve.LastParameter()));
            }else if(curve.GetType()==GeomAbs_Circle){
                if(std::abs(curve.LastParameter()-curve.FirstParameter()-2*std::acos(-1.))>1e-8)continue;
                point=curve.Value(curve.FirstParameter()+std::acos(-1.)/2);
                axis=curve.Circle().Axis().Direction();value.anchor.curveKind=CurveKind::Circle;
                value.anchor.circleRadius=curve.Circle().Radius();value.lengthLocal=2*std::acos(-1.)*value.anchor.circleRadius;
            }else continue;
            value.anchor.anchorPoint={point.X(),point.Y(),point.Z()};value.anchor.axis={axis.X(),axis.Y(),axis.Z()};
            for(double component:value.anchor.axis){if(std::abs(component)<=1e-12)continue;
                if(component<0)for(double& x:value.anchor.axis)x=-x;break;}
            bool interior=true;
            for(TopExp_Explorer v(edge,TopAbs_VERTEX);v.More();v.Next())
                if(point.Distance(BRep_Tool::Pnt(TopoDS::Vertex(v.Current())))<=1e-4/mm)interior=false;
            TopoDS_Edge resolved;
            if(!interior||Resolve(detached,value.anchor,mm,resolved)!=Outcome::Built||!resolved.IsSame(edge))continue;
            Step step;step.radiusLocal=.001/mm;step.anchors={value.anchor};Interval interval;
            const std::vector<TopoDS_Edge> selected{edge};
            if(!ExpectedRemoval(program,step,interval,&selected)
                ||!RadiusAdmitted(detached,edge,value.anchor,step.radiusLocal,mm))continue;
            out.values.push_back(value);
        }
        std::sort(out.values.begin(),out.values.end(),[](const Candidate& a,const Candidate& b){
            return std::tie(a.anchor.curveKind,a.anchor.anchorPoint,a.anchor.axis,a.anchor.circleRadius,a.lengthLocal)
                <std::tie(b.anchor.curveKind,b.anchor.anchorPoint,b.anchor.axis,b.anchor.circleRadius,b.lengthLocal);
        });
        out.truncated=out.values.size()>MaximumCandidates;
        if(out.truncated)out.values.resize(MaximumCandidates);
        std::vector<std::uint8_t> programAfter;
        // Fail closed in production as well as asserting byte preservation in tests.
        if(!retained_boolean::Encode(recipe,programAfter)||programBefore!=programAfter
            ||shapeBefore!=CandidateShapeBytes(shape)||baseBefore!=CandidateShapeBytes(retainedBase))
            return {CandidateStatus::Failed,false,{}};
        out.status=CandidateStatus::Available;return out;
    }catch(...){return {CandidateStatus::Failed,false,{}};}
}
}
