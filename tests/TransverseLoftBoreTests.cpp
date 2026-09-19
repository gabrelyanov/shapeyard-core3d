#include "SavedCutSourceBoreClearance.hxx"
#include "AnalyticBooleanSolid.hxx"
#include "RectangularLoftSolid.hxx"
#include <BRepBuilderAPI_Transform.hxx>
#include <cassert>
#include <iostream>
#include <iomanip>
#include "CutDisplayPreparation.hxx"
#include "SavedCutSourceBoreClearanceProbe.hxx"
using namespace core3d;
static retained_solid::Envelope envelope(double unit=.001, bool y=false) {
    rectangular_loft::Definition d;d.loftIdentifier=1;d.correspondence={10,11,12,13};d.dimensionMetersPerUnit=unit;
    const double f=.001/unit;
    const double z[]={0,55,100,130},width[]={8,26,28,30},depth[]={30,30,32,34};
    for(unsigned i=0;i<4;++i){rectangular_loft::Station s;s.identifier=100+i;
        s.cornerIdentifiers={200+4*i,201+4*i,202+4*i,203+4*i};s.correspondence=d.correspondence;
        s.z=z[i]*f;s.width=(y?depth[i]:width[i])*f;s.depth=(y?width[i]:depth[i])*f;d.stations.push_back(s);}
    retained_solid::Envelope e;
    for(auto* id:{&e.document,&e.entity,&e.definition,&e.sourceFeature,&e.derivedFeature})(*id)[0]=1;
    e.derivedFeature[0]=2;e.sourceFamily=3;e.sourceSchema=1;e.metersPerUnit=unit;e.axis=y?1:0;
    e.point={0,0,100*f};e.radius=12*f;e.operandID=1;assert(loft_persistence::Encode(d,e.sourceValues));return e;
}
static TopoDS_Shape base(const retained_solid::Envelope& e){
    rectangular_loft::Definition d;assert(loft_persistence::Decode(e.sourceValues,d));
    rectangular_loft::Admission admission;auto prepared=rectangular_loft::Prepare(d,admission);assert(prepared);
    rectangular_loft::SolidResult result;std::atomic_bool stop(false);
    assert(rectangular_loft::Build(prepared,stop,result)==rectangular_loft::BuildStatus::Built);return result.solid;
}
static double volume(const TopoDS_Shape& s){GProp_GProps p;BRepGProp::VolumeProperties(s,p,1e-12,Standard_True);return p.Mass();}
static std::string bytes(const TopoDS_Shape& s){std::ostringstream stream;BRepTools::Write(s,stream,Standard_False,Standard_False,TopTools_FormatVersion_VERSION_3);return stream.str();}
int main(){
    using S=saved_cut_bore_clearance::Status;std::atomic_bool stop(false);
    for(const auto& check:saved_cut_bore_clearance::probe::Run()){
        if(!check.second)std::cerr<<check.first<<std::endl;
        assert(check.second);
    }
    for(double unit:{.001})for(bool y:{false,true}){
        auto e=envelope(unit,y);const double f=.001/unit;
        const auto clearance=saved_cut_bore_clearance::Inspect(e);
        assert(clearance.status==S::ClearRecipeTransverse&&clearance.ligamentLowerMM>3.7);
        const auto original=base(e);const auto originalBytes=bytes(original);
        const double baseExpected=(55.*(8+26)/2*30
            +45./6*(2*26*30+26*32+28*30+2*28*32)
            +30./6*(2*28*32+28*34+30*32+2*30*34))*f*f*f;
        assert(std::abs(volume(original)-baseExpected)<baseExpected*1e-12);
        analytic_boolean::Result cut;
        assert(analytic_boolean::Build(original,cylindrical_cut::Recipe(e),stop,cut)==analytic_boolean::Status::Built);
        // Exact B05 piecewise-affine integral, split at the centre station.
        const double removed=(28*std::acos(-1.)*144+(2./30-2./45)*2./3*1728)*f*f*f;
        std::cout<<std::setprecision(17)<<"unit="<<unit<<" axis="<<unsigned(e.axis)<<" removed="<<volume(original)-volume(cut.solid)<<" expected="<<removed<<std::endl;
        assert(std::abs(volume(original)-volume(cut.solid)-removed)<removed*1e-9);
        double integrated=0;assert(saved_cut_bore_clearance::TransverseRemovedVolume(e,integrated));
        assert(std::abs(integrated-removed)<removed*1e-12);
        assert(saved_cut_bore_clearance::VerifyTransverseResult(original,cut.solid,e,stop,cut.solid));
        assert(!saved_cut_bore_clearance::VerifyTransverseResult(original,original,e,stop,cut.solid));
        // Independently sized cylinder spans 200 mm, not the builder's bounds margin.
        const gp_Pnt start=y?gp_Pnt(0,-100*f,100*f):gp_Pnt(-100*f,0,100*f);
        BRepPrimAPI_MakeCylinder tool(gp_Ax2(start,y?gp::DY():gp::DX()),12*f,200*f);
        BRepBuilderAPI_Copy independentBase(original,Standard_True,Standard_False);
        BRepAlgoAPI_Cut independent(independentBase.Shape(),tool.Shape());assert(independent.IsDone());
        assert(std::abs(volume(independent.Shape())-volume(cut.solid))<removed*1e-9);
        std::vector<std::uint8_t> encoded,roundtrip;assert(retained_solid::Encode(e,encoded));
        retained_solid::Envelope decoded;assert(retained_solid::Decode(encoded,decoded));
        assert(retained_solid::Encode(decoded,roundtrip)&&roundtrip==encoded);
        analytic_boolean::Result replay;
        assert(analytic_boolean::Build(base(decoded),cylindrical_cut::Recipe(decoded),stop,replay)==analytic_boolean::Status::Built);
        assert(bytes(cut.solid)==bytes(replay.solid));assert(bytes(original)==originalBytes);
        cut_display::Settings settings;settings.values={.001,.5,.1,0,0};settings.automatic=true;
        assert(cut_display::Prepare(cut.solid,settings,stop));
        assert(saved_cut_bore_clearance::VerifyTransverseResult(original,cut.solid,e,stop,replay.solid));
        auto shifted=e;shifted.point[y?0:1]=.5;
        analytic_boolean::Result wrong;
        assert(analytic_boolean::Build(original,cylindrical_cut::Recipe(shifted),stop,wrong)==analytic_boolean::Status::Built);
        assert(!saved_cut_bore_clearance::VerifyTransverseResult(original,wrong.solid,e,stop,replay.solid));
        auto bad=e;bad.point[y?0:1]=5*f;
        assert(saved_cut_bore_clearance::Inspect(bad).status==S::OutsideOrInsufficientLigament);
        bad=e;bad.point[2]=120*f;assert(saved_cut_bore_clearance::Inspect(bad).status==S::OutsideOrInsufficientLigament);
        // Between authored stations: clearance must use interpolated walls.
        bad=e;bad.point[2]=75*f;bad.point[y?0:1]=4*f;
        assert(saved_cut_bore_clearance::Inspect(bad).status==S::OutsideOrInsufficientLigament);
        // Occurrence rotation is external to both source recipe and tool.
        gp_Trsf occurrence;occurrence.SetRotation(gp_Ax1(gp::Origin(),gp::DY()),std::acos(-1.)/2);
        occurrence.SetTranslationPart(gp_Vec(-100*f,0,280*f));
        auto placed=BRepBuilderAPI_Transform(cut.solid,occurrence,Standard_True).Shape();
        assert(std::abs(volume(placed)-volume(cut.solid))<removed*1e-9);
    }
    // Small document units expose OCCT intersection precision in this host
    // build. The independent volume proof must fail closed, never relax it.
    for(bool y:{false,true}){
        const auto e=envelope(1.,y);const auto original=base(e);analytic_boolean::Result cut;
        assert(saved_cut_bore_clearance::Inspect(e).status==S::ClearRecipeTransverse);
        assert(analytic_boolean::Build(original,cylindrical_cut::Recipe(e),stop,cut)==analytic_boolean::Status::Built);
        double removed=0;assert(saved_cut_bore_clearance::TransverseRemovedVolume(e,removed));
        const bool accurate=std::abs(volume(original)-volume(cut.solid)-removed)<=removed*1e-9;
        assert(saved_cut_bore_clearance::VerifyTransverseResult(original,cut.solid,e,stop,cut.solid)==accurate);
        std::cout<<"metre-document axis="<<unsigned(e.axis)<<" exact-volume="<<accurate<<" (inaccurate result refuses)\n";
    }
    // Strict 0.002 mm ligament in a constant section, both transverse axes.
    for(bool y:{false,true}){
        auto e=envelope(.001,y);rectangular_loft::Definition d;assert(loft_persistence::Decode(e.sourceValues,d));
        for(auto& s:d.stations){s.width=40;s.depth=40;}assert(loft_persistence::Encode(d,e.sourceValues));
        e.point[y?0:1]=8-.002;assert(saved_cut_bore_clearance::Inspect(e).status==S::OutsideOrInsufficientLigament);
        e.point[y?0:1]=8-.003;assert(saved_cut_bore_clearance::Inspect(e).status==S::ClearRecipeTransverse);
        e.axis=2;e.point={0,0,100};assert(saved_cut_bore_clearance::Inspect(e).status==S::ClearRecipeDisk);
    }
    // Recipe-frame quarter turn and positive scale; oblique axes still refuse.
    auto e=envelope();rectangular_loft::Definition d;assert(loft_persistence::Decode(e.sourceValues,d));
    profile::ConstructionFrame frame;const double q=std::sqrt(.5);frame.values={3,4,5,0,q,0,q,2};d.constructionFrame=frame;
    assert(loft_persistence::Encode(d,e.sourceValues));gp_Trsf t;assert(frame.Transform(t));
    const auto c=gp_Pnt(0,0,100).Transformed(t);e.point={c.X(),c.Y(),c.Z()};e.radius=24;e.axis=2;
    assert(saved_cut_bore_clearance::Inspect(e).status==S::ClearRecipeTransverse);
    analytic_boolean::Result cut;assert(analytic_boolean::Build(base(e),cylindrical_cut::Recipe(e),stop,cut)==analytic_boolean::Status::Built);
    assert(saved_cut_bore_clearance::VerifyTransverseResult(base(e),cut.solid,e,stop,cut.solid));
    frame.values[4]=std::sin(.3);frame.values[6]=std::cos(.3);d.constructionFrame=frame;
    assert(loft_persistence::Encode(d,e.sourceValues));assert(saved_cut_bore_clearance::Inspect(e).status==S::Nonparallel);
    stop.store(true);assert(!saved_cut_bore_clearance::VerifyTransverseResult(base(e),cut.solid,e,stop,cut.solid));
    std::cout<<"Transverse loft bore: exact B05 X/Y, frames, wall/cap/ligament refusal, independent volume, replay bytes and cancellation passed\n";
}
