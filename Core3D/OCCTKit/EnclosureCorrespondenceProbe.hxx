#pragma once
// Proposed guarded-native probe source only. No invocation/selection is added.
#if DEBUG
#include "SavedCutEnclosureExtractor.hxx"
#include "EnclosureGeometry.hxx"
#include "RetainedEnclosureCorrespondence.hxx"
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <sstream>
#include <map>
#include <string>

namespace core3d::enclosure_correspondence {
struct ProbeReport {
    std::map<std::string,bool> checks;
    Inspection before,reopened;
    std::string phase="setup";
    std::string diagnosticOriginal483,diagnosticReopened483;bool diagnosticArchiveRequested483=false;
};
inline bool ProbeBytes(const TopoDS_Shape& shape,std::string& bytes){
    bytes.clear();if(shape.IsNull())return false;std::ostringstream out;out.imbue(std::locale::classic());
    BRepTools::Write(shape,out,Standard_True,Standard_True,TopTools_FormatVersion_VERSION_3);
    if(!out.good())return false;bytes=out.str();return !bytes.empty()&&bytes.size()<=4*1024*1024;
}
// Real OCCT generator + private BRep write/read pipeline, never a callback mirror.
// This does NOT claim OCAF carrier save/reopen/Undo/Redo coverage.
inline ProbeReport ProbeGenerated(int plane,double metersPerUnit,bool framed,bool widened){
    ProbeReport r;try{
        if(plane<0||plane>2||(metersPerUnit!=.001&&metersPerUnit!=1))return r;
        const double k=.001/metersPerUnit;
        enclosure::Parameters p;p.metersPerUnit=metersPerUnit;p.definition.plane=plane;
        p.definition.dimensions={(widened?120.:100.)*k,60*k,30*k,2*k,2*k,4*k};
        if(framed){profile::ConstructionFrame f;const double a=std::acos(-1.0)/12;
            f.values={13*k,-7*k,5*k,0,0,std::sin(a),std::cos(a),1.25};p.definition.constructionFrame=f;}
        auto stop=std::make_shared<std::atomic_bool>(false);EnclosureSolidResult made;
        r.phase="generator";if(!BuildEnclosureSolidGeometry(p.definition,stop,made))return r;
        std::string before,after;r.phase="source-serialize";if(!ProbeBytes(made.solid,before))return r;
        r.phase="generated-correspondence";r.checks["generatedMatches"]=InspectEnclosure(made.solid,p,*stop,r.before)==Classification::MatchedBoundary;
        r.checks["sourceBytesUnchanged"]=ProbeBytes(made.solid,after)&&after==before;
        r.checks["exactBoundaryCounts"]=r.before.vertices==32&&r.before.edges==48&&r.before.faces==19&&r.before.wires==20&&r.before.coedges==96;
        r.phase="private-brep-reopen";std::istringstream input(before);input.imbue(std::locale::classic());TopoDS_Shape reopened;BRep_Builder builder;
        BRepTools::Read(reopened,input,builder);if(reopened.IsNull())return r;
        r.checks["privateBRepReopenMatches"]=InspectEnclosure(reopened,p,*stop,r.reopened)==Classification::MatchedBoundary;
        // Only AFTER the original failing predicate: capture post-inspection
        // diagnostic bytes, not evidence of pre-inspection preservation.
        r.diagnosticArchiveRequested483=framed&&plane==0&&metersPerUnit==.001&&!widened;
        if(r.diagnosticArchiveRequested483)try{
            std::string fresh;if(before.size()<=512*1024&&ProbeBytes(reopened,fresh)&&fresh.size()<=512*1024){
                r.diagnosticOriginal483=before;r.diagnosticReopened483=std::move(fresh);}
        }catch(...){r.diagnosticOriginal483.clear();r.diagnosticReopened483.clear();}
        auto wrong=p;wrong.definition.dimensions.width+=(widened?-20.:20.)*k;Inspection refused;
        r.checks["differentWidthRefuses"]=InspectEnclosure(made.solid,wrong,*stop,refused)==Classification::Refused;
        wrong=p;wrong.definition.dimensions.floor=3*k;
        r.checks["differentFloorRefuses"]=InspectEnclosure(made.solid,wrong,*stop,refused)==Classification::Refused;
        wrong=p;wrong.definition.dimensions.wall=2.1*k;
        r.checks["differentWallRefuses"]=InspectEnclosure(made.solid,wrong,*stop,refused)==Classification::Refused;
        auto reversed=made.solid.Reversed();r.checks["reversedRootRefuses"]=InspectEnclosure(reversed,p,*stop,refused)==Classification::Refused;
        stop->store(true);r.checks["alreadyStoppedCancels"]=InspectEnclosure(made.solid,p,*stop,refused)==Classification::Cancelled;stop->store(false);
        r.checks["sourceStillExactAfterRefusals"]=ProbeBytes(made.solid,after)&&after==before;
        r.phase="complete";return r;
    }catch(...){r.checks["exception"]=false;return r;}
}
// Direct tests of the same native coefficient functions, not a reimplementation.
inline std::map<std::string,bool> ProbeAnalyticRefusals(){
    using namespace detail;std::map<std::string,bool> checks;
    Surface s;s.cylinder=true;s.x={2,0,0};s.y={0,2,0};s.z={0,0,1};
    Curve c;c.circle=true;c.a=s.x;c.b=s.y;c.first=0;c.last=std::acos(-1.0)/2;
    PCurve p;p.a={1,0};p.first=c.first;p.last=c.last;
    checks["quarterCoefficientIdentity"]=PCurveIdentity(c,p,s,1,1e-9);
    auto changed=p;changed.a.SetX(5);checks["sameEndpointsExtraTurnRefuses"]=!PCurveIdentity(c,changed,s,1,1e-9);
    changed=p;changed.a.SetY(.01);checks["helicalDriftRefuses"]=!PCurveIdentity(c,changed,s,1,1e-9);
    changed=p;changed.c.SetX(.01);checks["phaseMismatchRefuses"]=!PCurveIdentity(c,changed,s,1,1e-9);
    Surface plane;plane.x={1,0,0};plane.y={0,1,0};plane.z={0,0,1};
    PCurve circle;circle.circle=true;circle.a={2,0};circle.b={0,2};circle.first=c.first;circle.last=c.last;
    checks["planeCircleCoefficients"]=PCurveIdentity(c,circle,plane,1,1e-9);
    auto ellipse=c;ellipse.b*=1.01;checks["ellipseCoefficientRefuses"]=!PCurveIdentity(ellipse,circle,plane,1,1e-9);
    auto wrapped=plane;wrapped.boxes.push_back({-2,1.99,-2,2});auto interior=c;auto interiorPC=circle;
    interior.first=interiorPC.first=-std::acos(-1.0)/4;interior.last=interiorPC.last=std::acos(-1.0)/4;
    checks["interiorTrimExtremumRefuses"]=!PCurveIdentity(interior,interiorPC,wrapped,1,1e-9);
    ExpectedBoundary expected;expected.vertices[0]={2,0,0};expected.vertices[1]={0,2,0};ExpectedEdge edge;edge.start=0;edge.end=1;edge.circle=true;edge.radius=2;
    checks["forwardArcSupport"]=MatchCurve(c,edge,expected,true,1,1e-9);
    auto backward=c;backward.a={0,2,0};backward.b={2,0,0};
    checks["reversedParameterArcSupport"]=MatchCurve(backward,edge,expected,false,1,1e-9);
    Inspection report;std::vector<gp_Trsf> chain;gp_Trsf t;t.SetTranslation(gp_Vec(1,2,3));TopLoc_Location single(t);
    checks["rawSingleLocationAdmitted"]=LocationChain(single,chain,report);
    checks["rawPowerTwoRefuses"]=!LocationChain(single.Powered(2),chain,report);
    TopLoc_Location deep;for(unsigned i=0;i<9;++i){gp_Trsf d;d.SetTranslation(gp_Vec(i+1,0,0));deep=deep*TopLoc_Location(d);}
    checks["rawNinthDatumRefuses"]=!LocationChain(deep,chain,report);
    return checks;
}
}
#endif
