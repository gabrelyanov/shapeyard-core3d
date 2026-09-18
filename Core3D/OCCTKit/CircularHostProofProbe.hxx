#pragma once
#if DEBUG
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include "CircularHostExtractor.hxx"
#include "SavedCutSourceEdit.hxx"
#include "SavedBooleanResultCorrespondence.hxx"
#include "SavedCutSourceBoreClearanceProbe.hxx"
#include "AnalyticBooleanSolid.hxx"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <sstream>

namespace core3d::saved_cut_circular_host::probe {
using Checks=std::map<std::string,bool>;
namespace fixture=saved_cut_bore_clearance::probe;
inline retained_solid::Envelope Wheel(double inner=0,int plane=0,double units=.001){
    auto e=fixture::identity(units);const double mm=units*1000;
    e.sourceFamily=1;e.sourceSchema=1;e.axis=fixture::axis(plane);e.radius=(inner?10:20)/mm;
    e.point=fixture::point(plane,(inner?95:0)/mm,0);
    profile::Parameters p;p.metersPerUnit=units;p.definition.plane=plane;p.definition.depth=40/mm;
    p.definition.circle=ProfileCircularSection{{0,0},150/mm,inner/mm};
    if(!profile::Encode(p,e.sourceValues))throw std::runtime_error("wheel encode");return e;
}
// Mirrors the production circular-profile face/prism builder, including Copy
// and Canonize flags. No cells or seams are obtained from the expectation.
inline TopoDS_Shape Base(const retained_solid::Envelope& e){
    profile::Parameters p;if(!profile::Decode(e.sourceValues,p)||!p.definition.circle)return {};
    const auto& d=p.definition;const auto& c=*d.circle;
    const auto center=enclosure_correspondence::PlanePoint(c.center.X(),c.center.Y(),0,d.plane);
    const gp_Dir normal=d.plane==0?gp::DZ():d.plane==1?gp_Dir(0,-1,0):gp::DX();
    const gp_Dir u=d.plane==2?gp::DY():gp::DX();const gp_Ax2 basis(center,normal,u);
    BRepBuilderAPI_MakeWire outer(BRepBuilderAPI_MakeEdge(gp_Circ(basis,c.outerRadius)).Edge());
    BRepBuilderAPI_MakeFace face(gp_Pln(center,normal),outer.Wire(),Standard_True);
    if(c.innerRadius>0){BRepBuilderAPI_MakeWire inner(BRepBuilderAPI_MakeEdge(gp_Circ(basis,c.innerRadius)).Edge());face.Add(TopoDS::Wire(inner.Wire().Reversed()));}
    auto shape=BRepPrimAPI_MakePrism(face.Face(),enclosure_correspondence::PlaneVector(0,0,d.depth,d.plane),Standard_True,Standard_True).Shape();
    if(p.constructionFrame){gp_Trsf frame;if(!p.constructionFrame->Transform(frame))return {};
        shape=BRepBuilderAPI_Transform(shape,frame,Standard_True,Standard_False).Shape();}
    return shape;
}
inline TopoDS_Shape Cut(const TopoDS_Shape& base,const retained_solid::Envelope& e){
    std::atomic_bool stop(false);analytic_boolean::Result out;
    return analytic_boolean::Build(base,cylindrical_cut::Recipe(e),stop,out)==analytic_boolean::Status::Built?out.solid:TopoDS_Shape{};
}
inline bool Matched(const TopoDS_Shape& shape,const retained_solid::Envelope& e){
    const std::atomic_bool stop(false);return whole::Inspect(shape,e,e,stop).classification==whole::Classification::MatchedOrientedBoundary;
}
inline double VolumeMM3(const TopoDS_Shape& shape,double units=.001){GProp_GProps p;BRepGProp::VolumeProperties(shape,p);const double mm=units*1000;return p.Mass()*mm*mm*mm;}
inline bool Volume(const TopoDS_Shape& shape,double timesPi,double units=.001){const double expected=timesPi*std::acos(-1.0);return !shape.IsNull()&&std::abs(VolumeMM3(shape,units)-expected)<=expected*1e-6;}
inline std::string Bytes(const TopoDS_Shape& shape){std::ostringstream out;out.imbue(std::locale::classic());BRepTools::Write(shape,out,Standard_True,Standard_True,TopTools_FormatVersion_VERSION_3);return out.str();}
inline TopoDS_Shape Clone(const TopoDS_Shape& shape){std::istringstream in(Bytes(shape));in.imbue(std::locale::classic());BRep_Builder b;TopoDS_Shape out;BRepTools::Read(out,in,b);return out;}
inline Checks RunExpectationDeterminism(){Checks checks;
    for(double units:{.001,1.0})for(int plane=0;plane<3;++plane)for(double inner:{0.,40.}){
        const auto key=std::to_string(units)+"/"+std::to_string(plane)+"/"+std::to_string(inner);
        auto e=Wheel(inner,plane,units);profile::Parameters p;profile::Decode(e.sourceValues,p);Expectation a,b;
        bool good=BuildExpectedBoundary(p.definition,{},a)&&BuildExpectedBoundary(p.definition,{},b);
        const unsigned rings=inner?2:1;good=good&&a.vertices.size()==2*rings&&a.edges.size()==3*rings&&a.faces.size()==2+rings&&a.hostGenus==rings-1;
        for(unsigned ring=0;good&&ring<rings;++ring)for(unsigned level=0;level<2;++level){
            const auto expected=enclosure_correspondence::PlanePoint((ring?inner:150)/(units*1000),0,level?40/(units*1000):0,plane);
            good=good&&a.vertices[2*ring+level].IsEqual(expected,0)&&a.vertices[2*ring+level].IsEqual(b.vertices[2*ring+level],0);
            good=good&&a.edges[3*ring+level].fullCircle&&a.edges[3*ring+level].start==a.edges[3*ring+level].end;
            good=good&&a.faces[level].wires[ring][0].forward==bool((level!=0)^(ring!=0)^(plane==1));}
        checks[key+"/recipe-cells-and-cap-winding"]=good;
        std::atomic_bool stop(false);Inspection report;
        checks[key+"/native-base-matches"]=InspectCylinder(Base(e),p.definition,{},units,stop,report)==Classification::MatchedBoundary;
    }return checks;
}
inline Checks RunHostSeamOwnership(){Checks checks;
    for(int plane=0;plane<3;++plane)for(double inner:{0.,40.}){auto e=Wheel(inner,plane);auto base=Base(e),cut=Cut(base,e);
        const std::string key=std::to_string(plane)+"/"+std::to_string(inner);const std::atomic_bool stop(false);
        const auto bytes=Bytes(cut);const auto proof=whole::Inspect(cut,e,e,stop);
        checks[key+"/whole-boundary-and-seam-certificates"]=proof.classification==whole::Classification::MatchedOrientedBoundary
            &&proof.vertices==(inner?6:4)&&proof.edges==(inner?9:6)&&proof.faces==(inner?5:4)&&proof.vertexLinks==proof.vertices;
        checks[key+"/exact-volume"]=Volume(cut,inner?832000:884000);
        checks[key+"/fresh-brep-and-unchanged-bytes"]=Matched(Clone(cut),e)&&Bytes(cut)==bytes;
        profile::Parameters p;profile::Decode(e.sourceValues,p);p.definition.circle->outerRadius=180;auto next=e;profile::Encode(p,next.sourceValues);
        const auto changed=Cut(Base(next),next);
        checks[key+"/radius-edit-boundary"]=whole::Inspect(changed,e,next,stop).classification==whole::Classification::MatchedOrientedBoundary;
    }return checks;
}
inline Checks RunFullCircleOriginalCurve(){Checks checks;auto e=Wheel();whole::Expected expected;whole::ExpectedSource(e,expected);
    d::Curve c;c.circle=true;c.c=gp_Vec(0,0,0);c.a=gp_Vec(150,0,0);c.b=gp_Vec(0,150,0);c.first=0;c.last=2*std::acos(-1.0);
    checks["full-circle-admitted"]=whole::detail::OriginalCurve(c,expected.edges[0],expected,true,1,1e-9);
    expected.edges[0].fullCircle=false;checks["full-circle-requires-explicit-flag"]=!whole::detail::OriginalCurve(c,expected.edges[0],expected,true,1,1e-9);expected.edges[0].fullCircle=true;
    c.last=3*std::acos(-1.0);checks["three-pi-refused"]=!whole::detail::OriginalCurve(c,expected.edges[0],expected,true,1,1e-9);
    c.last=std::acos(-1.0);checks["half-circle-refused"]=!whole::detail::OriginalCurve(c,expected.edges[0],expected,true,1,1e-9);
    c.last=2*std::acos(-1.0);c.b.Reverse();checks["wrong-directed-plane-refused"]=!whole::detail::OriginalCurve(c,expected.edges[0],expected,true,1,1e-9);
    return checks;
}
inline Checks RunClearanceIntervals(){Checks checks;using S=saved_cut_bore_clearance::Status;
    auto disk=Wheel(),ring=Wheel(40);
    checks["disk-clear"]=saved_cut_bore_clearance::Inspect(disk).status==S::ClearRecipeDisk;
    checks["annulus-clear"]=saved_cut_bore_clearance::Inspect(ring).status==S::ClearRecipeDisk;
    disk.point[0]=130;checks["tangent-rim-refused"]=saved_cut_bore_clearance::Inspect(disk).status==S::OutsideOrInsufficientLigament;
    ring.point[0]=49;checks["inner-hole-overlap-refused"]=saved_cut_bore_clearance::Inspect(ring).status==S::OutsideOrInsufficientLigament;
    ring.point[0]=50.001;checks["inner-hole-ligament-refused"]=saved_cut_bore_clearance::Inspect(ring).status==S::OutsideOrInsufficientLigament;
    const int previous=std::fegetround();std::fesetround(FE_UPWARD);
    const auto rounding=saved_cut_bore_clearance::Inspect(Wheel()).status;std::fesetround(previous);
    checks["rounding-mode-gate"]=rounding==S::NumericUncertain;
    return checks;
}
// Swap the actual low/high endpoint identities on the bore seam while keeping
// the rim edges and every seam curve/pcurve unchanged. The old/new incidence
// maps differ; geometric endpoint and opening ownership proofs must refuse.
inline bool SwapSeamVertices(const TopoDS_Shape& shape,double radius){
    for(TopExp_Explorer fi(shape,TopAbs_FACE);fi.More();fi.Next()){
        auto face=TopoDS::Face(fi.Current());enclosure_correspondence::Inspection budget;d::Surface surface;
        if(!d::ReadSurface(face,1,budget,surface)||!surface.cylinder||std::abs(d::Norm(surface.x)-radius)>1e-9)continue;
        for(TopExp_Explorer ei(face,TopAbs_EDGE);ei.More();ei.Next()){
            auto edge=TopoDS::Edge(ei.Current().Oriented(TopAbs_FORWARD));if(!BRep_Tool::IsClosed(edge,face))continue;
            std::vector<TopoDS_Shape> vertices;for(TopoDS_Iterator vi(edge);vi.More();vi.Next())vertices.push_back(vi.Value());
            if(vertices.size()!=2||vertices[0].IsSame(vertices[1]))return false;
            BRep_Builder b;const bool free=edge.Free();edge.Free(true);for(const auto& v:vertices)b.Remove(edge,v);
            b.Add(edge,vertices[1].Oriented(vertices[0].Orientation()));b.Add(edge,vertices[0].Oriented(vertices[1].Orientation()));edge.Free(free);
            std::vector<TopoDS_Shape> after;for(TopoDS_Iterator vi(edge);vi.More();vi.Next())after.push_back(vi.Value());
            return after.size()==2&&after[0].IsSame(vertices[1])&&after[1].IsSame(vertices[0]);
        }
    }return false;
}
inline TopoDS_Shape ReversedCap(const TopoDS_Shape& shape){BRep_Builder b;TopoDS_Shell shell;b.MakeShell(shell);bool changed=false;
    for(TopExp_Explorer fi(shape,TopAbs_FACE);fi.More();fi.Next()){auto face=TopoDS::Face(fi.Current());enclosure_correspondence::Inspection budget;d::Surface surface;
        if(!d::ReadSurface(face,1,budget,surface))return {};if(!changed&&!surface.cylinder){face.Reverse();changed=true;}b.Add(shell,face);}
    if(!changed)return {};TopoDS_Solid solid;b.MakeSolid(solid);b.Add(solid,shell);return solid;
}
inline Checks RunAdversarial(){Checks checks;const std::atomic_bool stop(false);
    auto e=Wheel();e.radius=10;e.point[0]=95;auto base=Base(e),cut=Cut(base,e);const auto bytes=Bytes(cut);
    checks["clear-seam-meridian-crossing-bore-matches"]=Matched(cut,e);
    auto swapped=Clone(cut);checks["swapped-seam-vertices-refused"]=SwapSeamVertices(swapped,10)&&!Matched(swapped,e);
    auto reversed=ReversedCap(Clone(cut));checks["reversed-cap-refused"]=!reversed.IsNull()&&!Matched(reversed,e);
    auto hostSwapped=Clone(base);profile::Parameters p;profile::Decode(e.sourceValues,p);Inspection report;
    checks["swapped-host-seam-refused"]=SwapSeamVertices(hostSwapped,150)&&InspectCylinder(hostSwapped,p.definition,{},.001,stop,report)==Classification::Refused;
    retained_solid::Payload payload;payload.envelope=e;retained_solid::Encode(e,payload.bytes);payload.base=base;
    saved_cut_source_values::CirclePatch patch;patch.outerRadius=100; saved_cut_source_edit::Values values;
    checks["shrink-losing-ligament-refused"]=!saved_cut_source_edit::PrepareValues(payload,patch,stop,values);
    patch.outerRadius=180;auto stale=payload;stale.bytes.back()^=1;
    checks["stale-bytes-refused"]=!saved_cut_source_edit::PrepareValues(stale,patch,stop,values);
    checks["patch-admitted"]=saved_cut_source_edit::PrepareValues(payload,patch,stop,values);
    auto fixed=values.newEnvelope;fixed.sourceValues=e.sourceValues;std::vector<std::uint8_t> fixedBytes;retained_solid::Encode(fixed,fixedBytes);
    checks["bores-and-omitted-scalars-verbatim"]=fixedBytes==payload.bytes;
    if(values.newEnvelope.sourceValues.size()!=e.sourceValues.size())checks["bores-and-omitted-scalars-verbatim"]=false;
    else for(unsigned i=0;i<e.sourceValues.size();++i)if(i!=9)checks["bores-and-omitted-scalars-verbatim"]=checks["bores-and-omitted-scalars-verbatim"]&&retained_solid::Bits(e.sourceValues[i])==retained_solid::Bits(values.newEnvelope.sourceValues[i]);
    checks["original-bytes-untouched"]=Bytes(cut)==bytes&&saved_cut_bore_clearance::probe::same(std::get<retained_solid::Envelope>(payload.envelope),e);
    auto hub=Wheel();auto next=hub;profile::Decode(next.sourceValues,p);p.definition.circle->outerRadius=180;profile::Encode(p,next.sourceValues);
    checks["edited-hub-exact-volume"]=Volume(Cut(Base(next),next),1280000);
    return checks;
}
inline Checks RunProgram(){Checks checks;auto e=Wheel();retained_boolean::Program p;
    p.source={e.document,e.entity,e.definition,e.sourceFeature,e.derivedFeature,e.sourceFamily,e.sourceSchema,e.metersPerUnit,e.sourceValues};
    auto first=cylindrical_cut::Recipe(e).tool;first.identifier=1;auto second=first;second.identifier=2;second.radius=10;second.point[0]=95;
    p.steps={{analytic_boolean::Operation::Difference,first},{analytic_boolean::Operation::Difference,second}};p.nextOperandID=3;
    auto base=Base(e),cut=Cut(base,e);auto next=e;next.radius=10;next.point[0]=95;next.operandID=2;cut=Cut(cut,next);
    const std::atomic_bool stop(false);const auto proof=saved_boolean_result::Inspect(cut,p,stop);
    checks["two-bores-whole-boundary"]=proof.classification==whole::Classification::MatchedOrientedBoundary&&proof.vertices==6&&proof.edges==9&&proof.faces==5;
    checks["two-bores-exact-volume"]=Volume(cut,880000);
    whole::detail::Graph overBudget,atBudget;
    checks["wire-budget-gate-34"]=!whole::detail::Collect(cut,e,stop,overBudget,35)
        &&whole::detail::Collect(cut,e,stop,atBudget,34);
    // Three separated bores give four cap wires; a fourth exceeds the legacy default.
    auto extra=e;extra.radius=10;extra.point=fixture::point(0,-95,0);
    auto threeBores=Cut(cut,extra);extra.point=fixture::point(0,0,95);
    auto fourBores=Cut(threeBores,extra);
    whole::detail::Graph legacyAtBudget,legacyOverBudget,explicitBudget;
    checks["legacy-default-wire-budget-unchanged"]=whole::detail::Collect(threeBores,e,stop,legacyAtBudget)
        &&!whole::detail::Collect(fourBores,e,stop,legacyOverBudget)
        &&whole::detail::Collect(fourBores,e,stop,explicitBudget,5);
    return checks;
}
inline Checks Run(unsigned scenario){try{switch(scenario){
    case 0:return RunExpectationDeterminism();case 1:return RunHostSeamOwnership();case 2:return RunFullCircleOriginalCurve();
    case 3:return RunClearanceIntervals();case 4:return RunAdversarial();case 5:return RunProgram();default:return {{"invalid-scenario",false}};
}}catch(...){return {{"probe-exception",false}};}}
} // namespace core3d::saved_cut_circular_host::probe
#endif
