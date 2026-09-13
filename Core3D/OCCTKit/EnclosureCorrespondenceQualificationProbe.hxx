#pragma once
#if DEBUG
// Detached qualification only. Does not register a document format or issue an edit.
#include "EnclosureCorrespondenceProbe.hxx"
#include <TopExp_Explorer.hxx>
#include <stdexcept>

namespace core3d::enclosure_correspondence::qualification_probe {
using Checks=std::map<std::string,bool>;
inline std::string FullBRep(const TopoDS_Shape& shape) {
    std::string bytes;
    if(!ProbeBytes(shape,bytes))throw std::invalid_argument("enclosure probe serialization");
    return bytes;
}
inline Checks Generated() {
    Checks out;
    const std::set<std::string> keys{"generatedMatches","sourceBytesUnchanged","exactBoundaryCounts",
        "privateBRepReopenMatches","differentWidthRefuses","differentFloorRefuses","differentWallRefuses",
        "reversedRootRefuses","alreadyStoppedCancels","sourceStillExactAfterRefusals"};
    for(int plane=0;plane<3;++plane)for(double unit:{.001,1.0})for(bool framed:{false,true})for(bool widened:{false,true}) {
        const std::string prefix=std::string(unit==.001?"mm":"metre")+".plane"+std::to_string(plane)
            +(framed?".framed":".plain")+(widened?".width120.":".width100.");
        const auto report=ProbeGenerated(plane,unit,framed,widened);
        std::set<std::string> actual;for(const auto& row:report.checks)actual.insert(row.first);
        // A partial/empty report is never accepted by vacuous all-true iteration.
        out[prefix+"phaseComplete"]=report.phase=="complete"&&actual==keys;
        for(const auto& key:keys) {
            const auto found=report.checks.find(key);
            out[prefix+key]=found!=report.checks.end()&&found->second;
        }
    }
    return out;
}
inline bool SameVector(const gp_Vec& a,const gp_Vec& b) {
    return retained_solid::Bits(a.X())==retained_solid::Bits(b.X())
        &&retained_solid::Bits(a.Y())==retained_solid::Bits(b.Y())
        &&retained_solid::Bits(a.Z())==retained_solid::Bits(b.Z());
}
inline bool SameCurve(const detail::Curve& a,const detail::Curve& b) {
    return a.circle==b.circle&&SameVector(a.c,b.c)&&SameVector(a.a,b.a)&&SameVector(a.b,b.b)
        &&retained_solid::Bits(a.first)==retained_solid::Bits(b.first)
        &&retained_solid::Bits(a.last)==retained_solid::Bits(b.last)&&a.location.IsEqual(b.location);
}
inline Checks ActualPCurveCorruption() {
    Checks out;
    for(double unit:{.001,1.0})for(int plane=0;plane<3;++plane) {
        const std::string prefix=std::string(unit==.001?"mm":"metre")+".plane"+std::to_string(plane)+".";
        const double k=.001/unit;
        enclosure::Parameters p;p.metersPerUnit=unit;p.definition.plane=plane;
        p.definition.dimensions={100*k,60*k,30*k,2*k,2*k,4*k};
        auto stop=std::make_shared<std::atomic_bool>(false);EnclosureSolidResult made;
        if(!BuildEnclosureSolidGeometry(p.definition,stop,made))throw std::invalid_argument("enclosure probe generation");
        const auto original=FullBRep(made.solid);Inspection report;
        out[prefix+"originalBoundaryMatches"]=InspectEnclosure(made.solid,p,*stop,report)==Classification::MatchedBoundary;
        // A real read creates independent topology and geometric objects before mutation.
        TopoDS_Shape corrupted;BRep_Builder builder;
        std::istringstream input(original);input.imbue(std::locale::classic());
        BRepTools::Read(corrupted,input,builder);
        if(corrupted.IsNull())throw std::invalid_argument("enclosure probe clone");
        const auto clonedBytes=FullBRep(corrupted);
        out[prefix+"independentFreshBoundaryMatches"]=!corrupted.IsPartner(made.solid)
            &&InspectEnclosure(corrupted,p,*stop,report)==Classification::MatchedBoundary;
        bool injected=false,shiftExact=false,curveUnchanged=false,flagsAndRange=false;
        unsigned faceCount=0;
        for(TopExp_Explorer faces(corrupted,TopAbs_FACE);faces.More()&&!injected;faces.Next()) {
            if(++faceCount>19)throw std::invalid_argument("enclosure probe face bound");
            const auto face=TopoDS::Face(faces.Current());detail::Surface surface;Inspection preparation;
            if(!detail::ReadSurface(face,unit*1000,preparation,surface)||!surface.cylinder)continue;
            unsigned edgeCount=0;
            for(TopExp_Explorer edges(face,TopAbs_EDGE);edges.More()&&!injected;edges.Next()) {
                if(++edgeCount>8)throw std::invalid_argument("enclosure probe edge bound");
                const auto edge=TopoDS::Edge(edges.Current().Oriented(TopAbs_FORWARD));
                detail::Curve beforeCurve;
                if(!detail::ReadCurve(edge,unit*1000,preparation,beforeCurve)||!beforeCurve.circle)continue;
                detail::PCurve beforePC;
                if(!detail::ReadPCurve(edge,surface,beforeCurve,beforePC,preparation)||!beforePC.stored||beforePC.circle)continue;
                double first=0,last=0;
                const auto pc=BRep_Tool::CurveOnSurface(edge,face,first,last);
                if(pc.IsNull()||first!=beforeCurve.first||last!=beforeCurve.last)continue;
                const auto oldLine=Handle(Geom2d_Line)::DownCast(pc);
                if(oldLine.IsNull())continue;
                Handle(Geom2d_Curve) shifted=Handle(Geom2d_Curve)::DownCast(pc->Copy());
                if(shifted.IsNull()||shifted==pc)throw std::invalid_argument("enclosure probe curve copy");
                const bool sameParameter=BRep_Tool::SameParameter(edge),sameRange=BRep_Tool::SameRange(edge);
                const double oldTolerance=BRep_Tool::Tolerance(edge);
                const auto oldOrigin=oldLine->Location();const auto oldDirection=oldLine->Direction();
                // On this actual cylinder, U is radians. Change only one stored
                // coedge's pcurve phase, retaining its whole original parameter range.
                shifted->Translate(gp_Vec2d(.125,0));
                builder.UpdateEdge(edge,shifted,face,oldTolerance);
                builder.Range(edge,face,first,last);
                injected=true;
                double readFirst=0,readLast=0;
                const auto observed=BRep_Tool::CurveOnSurface(edge,face,readFirst,readLast);
                const auto changed=Handle(Geom2d_Line)::DownCast(observed);
                shiftExact=!changed.IsNull()&&changed->Location().X()==oldOrigin.X()+.125
                    &&changed->Location().Y()==oldOrigin.Y()&&changed->Direction().X()==oldDirection.X()
                    &&changed->Direction().Y()==oldDirection.Y();
                detail::Curve afterCurve;
                curveUnchanged=detail::ReadCurve(edge,unit*1000,preparation,afterCurve)&&SameCurve(beforeCurve,afterCurve);
                flagsAndRange=readFirst==first&&readLast==last
                    &&BRep_Tool::SameParameter(edge)==sameParameter&&BRep_Tool::SameRange(edge)==sameRange
                    &&BRep_Tool::Tolerance(edge)==oldTolerance;
            }
        }
        out[prefix+"actualStoredCylinderPCurveShifted"]=injected&&shiftExact;
        out[prefix+"original3DCurveAndRangeExact"]=injected&&curveUnchanged;
        out[prefix+"parameterFlagsToleranceRangeExact"]=injected&&flagsAndRange;
        out[prefix+"mutatedCloneBytesDiffer"]=injected&&FullBRep(corrupted)!=clonedBytes;
        // Require collection to complete: refusal must reach actual boundary
        // matching, not merely a null shape or changed SameParameter flag.
        const auto classification=InspectEnclosure(corrupted,p,*stop,report);
        out[prefix+"fullMatcherRejectsAtBoundary"]=injected&&classification==Classification::Refused
            &&std::string(report.phase)=="boundary";
        out[prefix+"originalFullBRepUnchanged"]=FullBRep(made.solid)==original;
    }
    return out;
}
inline Checks Run(int scenario) {
    try {
        switch(scenario) {
        case 0:return Generated();
        case 1:return ProbeAnalyticRefusals();
        case 2:return ActualPCurveCorruption();
        default:return {{"invalidScenario",false}};
        }
    }catch(...){return {{"setupException",false}};}
}
}
#endif
