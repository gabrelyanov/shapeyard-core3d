#pragma once
#if DEBUG
#include "SavedBooleanRingProbe.hxx"
#include "RectangularLoftSolid.hxx"
namespace core3d::saved_boolean_wedge_probe {
using Checks=std::map<std::string,bool>;
namespace rb=retained_boolean;
namespace ab=analytic_boolean;
inline retained_solid::Envelope Loft(){
    auto e=saved_cut_bore_clearance::probe::identity(.001);e.sourceFamily=3;e.sourceSchema=1;e.axis=2;e.radius=12;
    rectangular_loft::Definition d;d.loftIdentifier=1;d.correspondence={10,11,12,13};d.dimensionMetersPerUnit=.001;
    for(unsigned i=0;i<3;++i){rectangular_loft::Station s;s.identifier=100+i;s.cornerIdentifiers={200+4*i,201+4*i,202+4*i,203+4*i};s.correspondence=d.correspondence;
        s.z=20+60*i;s.centerX=i==1?6:0;s.width=i==1?100:80;s.depth=i==1?60:50;d.stations.push_back(s);}
    if(!loft_persistence::Encode(d,e.sourceValues))throw std::runtime_error("loft fixture");return e;
}
inline retained_solid::Envelope Prism(){
    auto e=Loft();e.sourceFamily=1;profile::Parameters p;p.metersPerUnit=.001;p.definition.plane=0;p.definition.depth=120;
    p.definition.points={{-40,-25},{40,-25},{40,25},{-40,25}};
    if(!profile::Encode(p,e.sourceValues))throw std::runtime_error("prism fixture");return e;
}
inline TopoDS_Shape Base(const retained_solid::Envelope& e){
    if(e.sourceFamily==3){TopoDS_Shape base;const std::atomic_bool stop(false);
        if(saved_cut_source_edit::RebuildLoftBase(e,stop,base)!=saved_cut_source_edit::LoftBaseStatus::Built)return {};return base;}
    if(e.sourceFamily==1){profile::Parameters p;if(!profile::Decode(e.sourceValues,p))return {};
        if(p.definition.circle)return saved_cut_circular_host::probe::Base(e);}
    auto stop=std::make_shared<std::atomic_bool>(false);bool matched=false;
    auto base=saved_cut_bore_result::probe::Base(e,stop,matched);return matched?base:TopoDS_Shape{};
}
inline wedge_cut::CreateEdit Wedge(bool open=true,double y=0){return {ab::Axis::Z,{20,y,0},0,.5,open?6.:2.,open?45.:10.};}
inline std::optional<rb::Change> Append(const rb::Recipe& p,const wedge_cut::CreateEdit& w){return rb::Apply(p,rb::AppendWedge{w},1);}
inline std::vector<std::uint8_t> Bytes(const rb::Recipe& p){std::vector<std::uint8_t> b;rb::Encode(p,b);return b;}
inline std::string TrailingDigest(const std::vector<std::uint8_t>& bytes){
    if(bytes.size()<32)return {};const char* hex="0123456789abcdef";std::string result;
    for(std::size_t k=bytes.size()-32;k<bytes.size();++k){result+=hex[bytes[k]>>4];result+=hex[bytes[k]&15];}return result;
}
inline saved_boolean_build::Result Build(const rb::Program& p){
    const auto view=saved_boolean_result::detail::GeometryView(p,0);const auto base=Base(view);
    cut_display::Settings display;Handle(Prs3d_Drawer) drawer=new Prs3d_Drawer();const std::atomic_bool stop(false);
    if(!cut_display::Capture(drawer,display))return {};return saved_boolean_build::Build(base,p,display,stop);
}
inline double Volume(const TopoDS_Shape& shape){if(shape.IsNull())return NAN;GProp_GProps p;BRepGProp::VolumeProperties(shape,p);return p.Mass();}
inline bool Removed(const rb::Program& p,double required){const auto built=Build(p);
    const double mm=p.source.metersPerUnit*1000;
    const double removed=(Volume(Base(saved_boolean_result::detail::GeometryView(p,0)))-Volume(built.solid))*mm*mm*mm;
    return built.status==saved_boolean_build::Status::Built&&std::abs(removed-required)<=required*1e-9;
}
inline Checks Run(unsigned scenario) noexcept {
    Checks out;
    try {
        const auto e=Loft();const auto appended=Append(e,Wedge());out["append-admitted"]=bool(appended);if(!appended)return out;
        const auto program=std::get<rb::Program>(appended->recipe);const double eye=17280*std::acos(-1.),open=136544./9;
        if(scenario==0){
            for(bool loft:{false,true})for(bool opened:{false,true}){
                const auto source=loft?e:Prism();const auto change=Append(source,Wedge(opened));const std::string key=(loft?"loft-":"prism-")+std::string(opened?"open":"closed");
                out[key+"-admitted"]=bool(change);if(!change)continue;
                const auto mixed=std::get<rb::Program>(change->recipe);auto single=mixed;single.steps.erase(single.steps.begin());
                const double wedge=opened?(loft?open:24800./3):3000;
                out[key+"-exact-wedge-volume"]=Removed(single,wedge);out[key+"-exact-mixed-volume"]=Removed(mixed,wedge+eye);
                const auto built=Build(single);const auto before=Base(source);TopTools_IndexedMapOfShape fs,es,vs;
                TopExp::MapShapes(before,TopAbs_FACE,fs);TopExp::MapShapes(before,TopAbs_EDGE,es);TopExp::MapShapes(before,TopAbs_VERTEX,vs);
                const unsigned bands=loft?2:1,df=opened?bands+3:4,de=opened?3*bands+9:12,dv=opened?2*bands+6:8;
                out[key+"-exact-recipe-topology"]=built.correspondence.classification==saved_boolean_result::Classification::MatchedOrientedBoundary
                    &&built.correspondence.faces==unsigned(fs.Extent())+df&&built.correspondence.edges==unsigned(es.Extent())+de&&built.correspondence.vertices==unsigned(vs.Extent())+dv;
            }
            for(double angle:{std::acos(-1.)/4,std::acos(-1.)/2}){auto rotated=Wedge(false);rotated.directionAngle=angle;
                const auto change=Append(Prism(),rotated);out["rotated-closed-"+std::to_string(angle)]=change&&Removed(std::get<rb::Program>(change->recipe),eye+3000);}
            for(double inner:{0.,30.}){const auto source=saved_cut_circular_host::probe::Wheel(inner);auto w=Wedge(false);w.localApex={-80,0,0};
                const auto change=Append(source,w);out["circular-host-"+std::to_string(inner)]=change&&Removed(std::get<rb::Program>(change->recipe),1000+source.radius*source.radius*40*std::acos(-1.));}
            auto box=saved_cut_bore_clearance::probe::box(.001);box.radius=2;auto w=Wedge(false);w.localApex={20,30,0};
            const auto boxed=Append(box,w);out["enclosure-floor-closed-slot"]=boxed&&Removed(std::get<rb::Program>(boxed->recipe),50+8*std::acos(-1.));
            auto metre=e;rectangular_loft::Definition definition;
            bool metreReady=loft_persistence::Decode(e.sourceValues,definition);definition.dimensionMetersPerUnit=1;
            for(auto& station:definition.stations){station.z*=.001;station.centerX*=.001;station.centerY*=.001;station.width*=.001;station.depth*=.001;}
            metre.metersPerUnit=1;metre.radius=.012;metreReady=metreReady&&loft_persistence::Encode(definition,metre.sourceValues);
            auto metreWedge=Wedge();metreWedge.localApex[0]=.02;
            const auto metreChange=rb::Apply(metre,rb::AppendWedge{metreWedge},1000);
            out["metre-units-exact-volume"]=metreReady&&metreChange&&Removed(std::get<rb::Program>(metreChange->recipe),eye+open);
            // The exact-volume assertion is mandatory, even for otherwise valid topology.
            const std::atomic_bool stop(false);ab::Recipe recipe;recipe.metersPerUnit=.001;recipe.tool=program.steps.back().operand;ab::Result bad;
            out["forced-volume-mismatch-refused"]=ab::Build(Base(e),recipe,stop,bad,open+1)==ab::Status::VerificationFailed&&bad.solid.IsNull();
            out["missing-volume-expectation-refused"]=ab::Build(Base(e),recipe,stop,bad)==ab::Status::InvalidRecipe&&bad.solid.IsNull();
        }else if(scenario==1){
            const auto widths=rb::Apply(appended->recipe,rb::SetWedgeWidths{2,1,7},1);
            const auto length=widths?rb::Apply(widths->recipe,rb::SetWedgeLength{2,50},1):std::optional<rb::Change>();
            out["widths-edit"]=bool(widths);out["length-edit"]=bool(length);
            if(widths){auto fixed=std::get<rb::Program>(widths->recipe);fixed.steps[1].operand.halfWidthApex=.5;fixed.steps[1].operand.halfWidthMouth=6;
                out["widths-only-byte-change"]=Bytes(fixed)==appended->newBytes;
                // Independent station integral: mean s=28, mean s^2=2416/3.
                out["widths-exact-volume"]=Removed(std::get<rb::Program>(widths->recipe),eye+120*(56+(6./45)*(2416./3)));}
            if(length){auto fixed=std::get<rb::Program>(length->recipe);fixed.steps[1].operand.length=45;
                out["length-only-byte-change"]=Bytes(fixed)==widths->newBytes;
                out["length-exact-volume"]=Removed(std::get<rb::Program>(length->recipe),eye+120*(56+(6./50)*(2416./3)));}
            rb::Program decoded;out["minor3-byte-exact"]=rb::Decode(appended->newBytes,decoded)&&Bytes(decoded)==appended->newBytes&&decoded.codecMinor==3;
            auto v1=program;v1.steps.pop_back();v1.codecMinor=1;v1.nextOperandID=2;
            const auto bytes1=Bytes(v1);out["minor1-byte-exact"]=!bytes1.empty()&&rb::Decode(bytes1,decoded)&&Bytes(decoded)==bytes1;
            // Golden commitments produced independently by the pre-P3 encoder
            // at Core3D 6c0e221; detect mutually compatible encoder/decoder drift.
            out["minor1-prechange-golden"]=bytes1.size()==606&&TrailingDigest(bytes1)=="bd28fde0933a8d1853d59f08eaaeb8801fe7e9184687a3e6ed9d5ee22aae9865";
            const auto wheel=saved_cut_circular_host::probe::Wheel();const auto ring=saved_boolean_ring_probe::Append(wheel);
            out["minor2-byte-exact"]=ring&&rb::Decode(ring->newBytes,decoded)&&Bytes(decoded)==ring->newBytes&&decoded.codecMinor==2;
            out["minor2-prechange-golden"]=ring&&ring->newBytes.size()==378&&TrailingDigest(ring->newBytes)=="7ecf10585ec7aa54b6b75733e17cd928b520c548a586fd406bb28aafebc0c5fb";
            bool zeros=true;for(const auto& step:decoded.steps)for(double value:{step.operand.directionAngle,step.operand.halfWidthApex,step.operand.halfWidthMouth,step.operand.length})zeros=zeros&&retained_solid::Bits(value)==0;
            out["old-minors-zero-fill"]=zeros;
            out["legacy-envelope-byte-exact"]=appended->oldBytes==Bytes(e);
            auto corrupt=appended->newBytes;corrupt[corrupt.size()-33]^=1;out["digest-refused"]=!rb::Decode(corrupt,decoded);
            corrupt.pop_back();out["truncation-refused"]=!rb::Decode(corrupt,decoded);
            corrupt.resize(retained_solid::MaximumEnvelopeBytes+1);out["oversize-refused"]=!rb::Decode(corrupt,decoded);
            const auto noop=rb::Apply(appended->recipe,rb::SetWedgeLength{2,45},1);out["no-op-byte-exact"]=noop&&!noop->changed&&noop->newBytes==appended->newBytes;
        }else if(scenario==2){
            for(double width:{0.,-.1,.049,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}){
                auto w=Wedge();w.worldHalfWidthApexMM=width;out["invalid-width-"+std::to_string(width)]=!Append(e,w);}
            auto w=Wedge();w.worldLengthMM=.09;out["short-degenerate-refused"]=!Append(e,w);
            w=Wedge();w.axis=ab::Axis::X;out["wrong-axis-refused"]=!Append(e,w);
            w=Wedge();w.localApex[0]=11;out["eye-overlap-refused"]=!Append(e,w);
            w=Wedge();w.localApex[1]=24.9;out["side-ligament-refused"]=!Append(e,w);
            w=Wedge();w.worldLengthMM=20;out["mouth-tangent-refused"]=!Append(e,w);
            w=Wedge();w.worldLengthMM=30;out["mouth-straddles-stations-refused"]=!Append(e,w);
            w=Wedge();w.directionAngle=.2;out["unnormal-open-refused"]=!Append(e,w);
            out["stale-id-refused"]=!rb::Apply(appended->recipe,rb::SetWedgeLength{999,50},1);
            out["wrong-kind-refused"]=!rb::Apply(appended->recipe,rb::SetWedgeWidths{1,1,6},1);
            auto budget=program;budget.steps[0].operand.kind=ab::OperandKind::CylinderRing;budget.steps[0].operand.radius=1;
            budget.steps[0].operand.boltCircleRadius=100;budget.steps[0].operand.hostRadiusRatio=.5;budget.steps[0].operand.count=16;
            auto ring=budget.steps[0];ring.operand.identifier=3;ring.operand.count=15;budget.steps.push_back(ring);budget.nextOperandID=4;
            out["32-sections-value-budget"]=rb::Valid(budget);budget.steps[2].operand.count=16;out["33-sections-value-refusal"]=!rb::Valid(budget);
            const analytic_boolean_wedge::SectionPoints a{{{-4,-1},{4,-1},{4,1},{-4,1}}},b{{{-1,-4},{1,-4},{1,4},{-1,4}}};
            out["crossed-sections-no-contained-vertex-refused"]=!analytic_boolean_wedge::detail::SeparatedPolygons(a,b,1);
            out["disk-tangent-refused"]=!analytic_boolean_wedge::detail::SeparatedDisk(a,{5,0},1,1);
            out["disk-clear-proven"]=analytic_boolean_wedge::detail::SeparatedDisk(a,{6,0},1,1);
            out["refusals-preserve-original-bytes"]=Bytes(program)==appended->newBytes;
        }else if(scenario==3){
            const auto patched=saved_boolean_build::SourcePatch(program,saved_cut_source_values::LoftPatch{101,110,{}});
            out["station-source-replay"]=bool(patched);
            saved_program_source_edit::Values prepared;const std::atomic_bool running(false);
            out["station-source-preparation"]=saved_boolean_build::PrepareSourceValues(appended->recipe,
                saved_cut_source_values::LoftPatch{101,110,{}},running,prepared,saved_program_source_edit::PrepareValues);
            if(patched){auto fixed=std::get<rb::Program>(patched->recipe);fixed.source.values=program.source.values;
                out["station-source-tools-byte-exact"]=Bytes(fixed)==appended->newBytes;
                // s at the middle station becomes 41; each band remains 60 mm.
                out["station-source-volume"]=Removed(std::get<rb::Program>(patched->recipe),eye+120*(30.5+(11./90)*(400+820+1681)/3));}
            const auto second=Append(appended->recipe,Wedge(true,14));out["two-open-wedges-admitted"]=bool(second);
            if(second){out["two-open-wedges-whole-boundary"]=Removed(std::get<rb::Program>(second->recipe),eye+2*open);
                const auto third=Append(second->recipe,Wedge(true,-14));out["three-open-wedges-admitted"]=bool(third);
                out["three-open-wedges-whole-boundary"]=third&&Removed(std::get<rb::Program>(third->recipe),eye+3*open);}
            const auto built=Build(program);const std::atomic_bool stop(false);
            out["reversed-cap-refused"]=saved_boolean_result::Inspect(saved_cut_circular_host::probe::ReversedCap(built.solid),program,stop).classification==saved_boolean_result::Classification::Refused;
            auto changed=program;changed.steps.back().operand.halfWidthMouth=6.1;
            out["foreign-width-boundary-refused"]=saved_boolean_result::Inspect(built.solid,changed,stop).classification==saved_boolean_result::Classification::Refused;
        }else out["unknown-scenario"]=false;
    }catch(...){out["exception"]=false;}return out;
}
} // namespace core3d::saved_boolean_wedge_probe
#endif
