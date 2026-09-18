#pragma once
#if DEBUG
#include "SavedBooleanWedgeProbe.hxx"
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BinTools.hxx>
namespace core3d::saved_boolean_fillet_probe {
namespace rb=retained_boolean;namespace rf=retained_fillet;namespace wedge=saved_boolean_wedge_probe;
using Checks=std::map<std::string,bool>;
inline std::vector<rf::EdgeAnchor> Rims(){std::vector<rf::EdgeAnchor> out;
    for(double z:{20.,140.}){rf::EdgeAnchor a;a.curveKind=rf::CurveKind::Circle;a.anchorPoint={12,0,z};a.axis={0,0,1};a.circleRadius=12;out.push_back(a);}return out;}
inline std::vector<rf::EdgeAnchor> Caps(){std::vector<rf::EdgeAnchor> out;
    for(double z:{20.,140.}){for(double y:{-25.,25.}){rf::EdgeAnchor a;a.anchorPoint={0,y,z};a.axis={1,0,0};out.push_back(a);}
        for(double x:{-40.,40.})for(double y:x<0?std::vector<double>{0.}:std::vector<double>{-15.,15.}){
            rf::EdgeAnchor a;a.anchorPoint={x,y,z};a.axis={0,1,0};out.push_back(a);}}return out;}
inline rb::Program Fixture(unsigned steps=2){
    const auto cut=wedge::Append(wedge::Loft(),wedge::Wedge());if(!cut)throw std::runtime_error("fillet cut");rb::Recipe p=cut->recipe;
    for(unsigned i=0;i<steps;++i){const auto appended=rb::Apply(p,rb::AppendFilletStep{2,i==0?Rims():Caps()},1);
        if(!appended)throw std::runtime_error("fillet append");p=appended->recipe;}return std::get<rb::Program>(p);
}
inline TopoDS_Shape AmbiguousPrism(){
    // Genuine shared vertex (0,-25,0) on two collinear cap edges. A notch
    // instead has separated pieces and cannot prove endpoint ambiguity (D14).
    BRepBuilderAPI_MakePolygon polygon;for(const auto& p:std::vector<gp_Pnt>{{-40,-25,0},{0,-25,0},{40,-25,0},{40,25,0},{-40,25,0}})polygon.Add(p);
    polygon.Close();return BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(polygon.Wire()).Face(),gp_Vec(0,0,120)).Shape();
}
inline Checks Run(unsigned scenario) noexcept {
    Checks out;try {
        auto p=Fixture();
        if(scenario==0||scenario==3){
            if(scenario==3){const auto patch=saved_boolean_build::SourcePatch(p,saved_cut_source_values::LoftPatch{101,110,{}});
                out["station-patch-values"]=bool(patch);if(!patch)return out;p=std::get<rb::Program>(patch->recipe);}
            const auto result=wedge::Build(p);out["built"]=result.status==saved_boolean_build::Status::Built;
            const auto bare=p;auto noFillets=bare;noFillets.filletSteps.clear();auto rims=p;rims.filletSteps.resize(1);
            const auto a=wedge::Build(noFillets),b=wedge::Build(rims);
            out["pre-fillet-built"]=a.status==saved_boolean_build::Status::Built;out["rims-built"]=b.status==saved_boolean_build::Status::Built;
            if(a.solid.IsNull()||b.solid.IsNull()||result.solid.IsNull())return out;
            rf::Interval interval;out["recipe-bounds"]=rf::ExpectedRemoval(p,p.filletSteps[1],interval);
            const double rim=2*rf::RimLoss(2,12),actualRim=rf::Volume(a.solid)-rf::Volume(b.solid),actualCap=rf::Volume(b.solid)-rf::Volume(result.solid);
            out["convex-rims-remove-closed-form"]=std::abs(actualRim-rim)<=rim*1e-6;
            out["cap-removal-in-derived-interval"]=actualCap>=interval.lower*(1-1e-6)&&actualCap<=interval.upper*(1+1e-6);
            // Eight shared rectangle corners; four notch endpoints each touch
            // only ONE selected edge, so do not count them as shared corners.
            out["shared-corner-count"]=interval.corners==8;
            out["taper-derived-sum"]=std::abs(interval.upper-(scenario==3?318.378599948441:338.986871094873))<1e-8;
            out["brep-valid"]=BRepCheck_Analyzer(result.solid).IsValid();out["bounds-sanity"]=rf::BoundsContained(a.solid,result.solid,1e-4);
            const auto again=wedge::Build(p);out["deterministic-rebuild"]=again.status==saved_boolean_build::Status::Built&&again.finalResult==result.finalResult;
            const auto view=saved_boolean_result::detail::GeometryView(p,0);const auto base=wedge::Base(view);
            cut_display::Settings display;Handle(Prs3d_Drawer) drawer=new Prs3d_Drawer();const std::atomic_bool running(false);
            saved_boolean_build::Budget budget;cut_display::Capture(drawer,display);
            std::array<double,6> committedBounds{},reopenedBounds{},replayedBounds{};
            out["current-carrier-exact-verification"]=saved_boolean_build::VerifyCurrent(base,result.solid,p,display,running,budget,&committedBounds);
            std::stringstream stream;BinTools::Write(result.solid,stream,Standard_True,Standard_True,BinTools_FormatVersion_VERSION_4);
            TopoDS_Shape reopened;BinTools::Read(reopened,stream);saved_boolean_build::Budget reopenedBudget;
            std::stringstream baseStream;BinTools::Write(base,baseStream,Standard_True,Standard_True,BinTools_FormatVersion_VERSION_4);
            TopoDS_Shape reopenedBase;BinTools::Read(reopenedBase,baseStream);
            out["reopened-carrier-exact-verification"]=saved_boolean_build::VerifyCurrent(reopenedBase,reopened,p,display,running,reopenedBudget,&reopenedBounds);
            saved_boolean_build::Budget replayedBudget;
            out["replayed-carrier-exact-verification"]=saved_boolean_build::VerifyCurrent(base,again.solid,p,display,running,replayedBudget,&replayedBounds);
            const auto sameBits=[](const std::array<double,6>& a,const std::array<double,6>& b){
                for(std::size_t i=0;i<a.size();++i)if(retained_solid::Bits(a[i])!=retained_solid::Bits(b[i]))return false;
                return true;
            };
            out["reopened-bounds-bit-exact"]=sameBits(committedBounds,reopenedBounds);
            out["replayed-bounds-bit-exact"]=sameBits(committedBounds,replayedBounds);
            std::array<double,6> actualBounds{};
            out["canonical-bounds-match-committed-geometry"]=analytic_boolean::detail::Bounds(result.solid,1,actualBounds)
                &&sameBits(committedBounds,actualBounds);
            if(scenario==0)out["recipe-max-x-bit-exact"]=retained_solid::Bits(committedBounds[3])==retained_solid::Bits(56.);
            auto foreign=p;foreign.filletSteps[0].radiusLocal=1.9;saved_boolean_build::Budget foreignBudget;
            out["foreign-radius-current-refused"]=!saved_boolean_build::VerifyCurrent(base,result.solid,foreign,display,running,foreignBudget);
        }else if(scenario==1){
            const auto bytes=wedge::Bytes(p);rb::Program decoded;out["minor4-exact-bits"]=rb::Decode(bytes,decoded)&&wedge::Bytes(decoded)==bytes&&decoded.codecMinor==4;
            auto signedBits=p;signedBits.filletSteps[0].anchors[0].anchorPoint[1]=-0.;const auto signedBytes=wedge::Bytes(signedBits);
            out["signed-zero-exact"]=signedBytes!=bytes&&rb::Decode(signedBytes,decoded)&&wedge::Bytes(decoded)==signedBytes;
            auto corrupt=bytes;corrupt[corrupt.size()-33]^=1;out["digest-refused"]=!rb::Decode(corrupt,decoded);corrupt.pop_back();out["truncation-refused"]=!rb::Decode(corrupt,decoded);
            corrupt.resize(retained_solid::MaximumEnvelopeBytes+1);out["oversize-refused"]=!rb::Decode(corrupt,decoded);
            corrupt=bytes;corrupt[7]=3;out["old-minor-with-fillet-tail-refused"]=!rb::Decode(corrupt,decoded);
            const auto old=wedge::Run(1);for(const auto& row:old)out["legacy-"+row.first]=row.second;
            const auto minor3=wedge::Append(wedge::Loft(),wedge::Wedge());out["minor3-prechange-golden"]=minor3&&minor3->newBytes.size()==754&&wedge::TrailingDigest(minor3->newBytes)=="962f66fe068190185206b13dd980fc08659345c3373bb53b14a739dfa7ebfe3b";
            const auto removed=rb::Apply(p,rb::RemoveFilletStep{2},1);const auto empty=removed?rb::Apply(removed->recipe,rb::RemoveFilletStep{1},1):std::nullopt;
            out["empty-tail-keeps-high-waters"]=empty&&rb::Decode(empty->newBytes,decoded)&&decoded.codecMinor==4&&decoded.filletSteps.empty()&&decoded.nextFilletStepID==3&&decoded.nextFilletEdgeID==13;
            const auto again=empty?rb::Apply(empty->recipe,rb::AppendFilletStep{2,Rims()},1):std::nullopt;
            out["no-id-reuse"]=again&&std::get<rb::Program>(again->recipe).filletSteps[0].stepIdentifier==3&&std::get<rb::Program>(again->recipe).filletSteps[0].anchors[0].identifier==13;
            auto invalid=p;invalid.filletSteps[1].anchors[0].identifier=1;out["duplicate-id-refused"]=!rb::Valid(invalid);
            const auto noop=rb::Apply(p,rb::SetFilletRadius{2,2},1);out["radius-noop-exact"]=noop&&!noop->changed&&noop->newBytes==bytes;
            invalid=p;invalid.filletSteps[0].radiusLocal=NAN;out["nonfinite-radius-refused"]=!rb::Valid(invalid);
            invalid=p;invalid.filletSteps[0].anchors[0].axis={0,0,0};out["zero-axis-refused"]=!rb::Valid(invalid);
            invalid=p;invalid.filletSteps[1].anchors[0].circleRadius=-0.;out["line-radius-negative-zero-refused"]=!rb::Valid(invalid);
        }else if(scenario==2){
            out["declines-are-clean-refusals"]=rf::IsDeclined(rf::Outcome::DeclinedRadiusAdmission)
                &&rf::IsDeclined(rf::Outcome::DeclinedAnchorNoMatch)&&rf::IsDeclined(rf::Outcome::DeclinedAnchorAmbiguous)
                &&rf::IsDeclined(rf::Outcome::DeclinedOcctFailure)
                &&!rf::IsDeclined(rf::Outcome::Built)&&!rf::IsDeclined(rf::Outcome::Cancelled);
            const auto plain=wedge::Build(Fixture(0));out["baseline-built"]=plain.status==saved_boolean_build::Status::Built;
            for(const auto& anchors:{Rims(),Caps()})for(const auto& anchor:anchors){TopoDS_Edge edge;
                out["unique-"+std::to_string(anchor.anchorPoint[0])+","+std::to_string(anchor.anchorPoint[1])+","+std::to_string(anchor.anchorPoint[2])]=rf::Resolve(plain.solid,anchor,1,edge)==rf::Outcome::Built;}
            TopoDS_Edge edge;const auto anchor=Rims()[0];out["rim-resolves"]=rf::Resolve(plain.solid,anchor,1,edge)==rf::Outcome::Built;double width=0;
            out["below-half-admitted"]=rf::RadiusAdmitted(plain.solid,edge,anchor,3.999,1,&width);out["chord-exactly-eight"]=std::abs(width-8)<1e-8;
            out["exact-half-refused"]=!rf::RadiusAdmitted(plain.solid,edge,anchor,4,1);out["oversize-refused"]=!rf::RadiusAdmitted(plain.solid,edge,anchor,5,1);
            auto oversize=Fixture(1);oversize.filletSteps[0].radiusLocal=5;
            const auto refusal=wedge::Build(oversize);out["oversize-outcome"]=refusal.status==saved_boolean_build::Status::Refused&&refusal.filletOutcome==rf::Outcome::DeclinedRadiusAdmission&&refusal.solid.IsNull();
            rf::EdgeAnchor shared;shared.anchorPoint={0,-25,0};shared.axis={1,0,0};const auto prism=AmbiguousPrism();
            out["ambiguity-prism-valid"]=BRepCheck_Analyzer(prism).IsValid();out["shared-vertex-ambiguous"]=rf::Resolve(prism,shared,1,edge)==rf::Outcome::DeclinedAnchorAmbiguous;
            shared.anchorPoint[0]=1;out["shared-vertex-neighbour-unique"]=rf::Resolve(prism,shared,1,edge)==rf::Outcome::Built;
            shared.anchorPoint[0]=100;out["missing-anchor"]=rf::Resolve(prism,shared,1,edge)==rf::Outcome::DeclinedAnchorNoMatch;
            const auto lost=saved_boolean_build::SourcePatch(p,saved_cut_source_values::LoftPatch{102,70,{}});
            out["lost-station-values"]=bool(lost);if(lost){const auto declined=wedge::Build(std::get<rb::Program>(lost->recipe));
                out["lost-station-declines"]=declined.status==saved_boolean_build::Status::Refused&&declined.filletOutcome==rf::Outcome::DeclinedAnchorNoMatch&&declined.solid.IsNull();}
            rf::FailureCount.store(1);const auto failed=wedge::Build(p);out["fault-empty"]=failed.status==saved_boolean_build::Status::Refused&&failed.filletOutcome==rf::Outcome::DeclinedOcctFailure&&failed.solid.IsNull();
            out["fault-retry"]=wedge::Build(p).status==saved_boolean_build::Status::Built;
            auto straight=wedge::Loft();rectangular_loft::Definition loft;loft_persistence::Decode(straight.sourceValues,loft);
            for(auto& station:loft.stations){station.width=80;station.depth=50;station.centerX=0;}
            loft_persistence::Encode(loft,straight.sourceValues);
            const auto notched=wedge::Append(straight,wedge::Wedge());out["shared-carrier-values"]=bool(notched);
            if(notched){const auto built=wedge::Build(std::get<rb::Program>(notched->recipe));out["shared-carrier-built"]=built.status==saved_boolean_build::Status::Built;
                rf::EdgeAnchor sharedBand;sharedBand.anchorPoint={40,25,80};sharedBand.axis={0,0,1};
                out["shared-carrier-ambiguous"]=rf::Resolve(built.solid,sharedBand,1,edge)==rf::Outcome::DeclinedAnchorAmbiguous;}
            rb::Program single;out["single-promoted"]=rb::Promote(wedge::Loft(),single);
            const auto one=rb::Apply(single,rb::AppendFilletStep{2,Rims()},1);
            out["single-bore-fillets-built"]=one&&wedge::Build(std::get<rb::Program>(one->recipe)).status==saved_boolean_build::Status::Built;
            const auto radius=rb::Apply(p,rb::SetFilletRadius{2,1.5},1);
            out["radius-edit-built"]=radius&&wedge::Build(std::get<rb::Program>(radius->recipe)).status==saved_boolean_build::Status::Built;
            const auto remove=rb::Apply(p,rb::RemoveFilletStep{2},1);
            out["removal-built"]=remove&&wedge::Build(std::get<rb::Program>(remove->recipe)).status==saved_boolean_build::Status::Built;
            const std::atomic_bool cancelled(true);const auto stopped=rf::Build(plain.solid,p,cancelled);
            out["cancelled-empty"]=stopped.outcome==rf::Outcome::Cancelled&&stopped.solid.IsNull();
            auto narrow=p;const auto widths=rb::Apply(narrow,rb::SetWedgeWidths{2,.5,10.625},1);
            out["widened-notch-replay-built"]=widths&&wedge::Build(std::get<rb::Program>(widths->recipe)).status==saved_boolean_build::Status::Built;
        }else out["unknown-scenario"]=false;
    }catch(...){out["exception"]=false;}return out;
}
}
#endif
