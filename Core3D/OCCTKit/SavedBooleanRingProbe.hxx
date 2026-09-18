#pragma once
#if DEBUG
#include "CircularHostProofProbe.hxx"
#include "SavedCutBoreResultObservationProbe.hxx"
#include "SavedProgramSourceDetachedWork.hxx"
namespace core3d::saved_boolean_ring_probe {
using Checks=std::map<std::string,bool>;
namespace wheel=saved_cut_circular_host::probe;
inline std::vector<std::uint8_t> Bytes(const retained_boolean::Recipe& recipe){
    std::vector<std::uint8_t> bytes;retained_boolean::Encode(recipe,bytes);return bytes;
}
inline std::optional<retained_boolean::Change> Append(const retained_boolean::Recipe& original,
    double bolt=80,double hole=10,unsigned count=6,analytic_boolean::Axis axis=analytic_boolean::Axis::Z){
    return retained_boolean::Apply(original,retained_boolean::AppendRing{{axis,{},bolt,hole,count}},1);
}
inline TopoDS_Shape Build(const retained_boolean::Program& p){
    const auto e=saved_boolean_result::detail::GeometryView(p,0);profile::Parameters source;
    if(!profile::Decode(e.sourceValues,source))return {};
    TopoDS_Shape base;
    if(source.definition.circle)base=wheel::Base(e);
    else {auto stop=std::make_shared<std::atomic_bool>(false);bool matched=false;
        base=saved_cut_bore_result::probe::Base(e,stop,matched);if(!matched)return {};}
    cut_display::Settings display;Handle(Prs3d_Drawer) drawer=new Prs3d_Drawer();
    if(!cut_display::Capture(drawer,display))return {};
    const std::atomic_bool stop(false);auto built=saved_boolean_build::Build(base,p,display,stop);
    return built.status==saved_boolean_build::Status::Built?built.solid:TopoDS_Shape{};
}
inline Checks Observe(const TopoDS_Shape& shape,const retained_boolean::Program& p){
    Checks checks;const std::atomic_bool stop(false);std::vector<retained_boolean::Disk> disks;
    const bool expanded=retained_boolean::ExpandedDisks(p,disks);checks["validated-expanded-disks"]=expanded;
    const auto proof=saved_boolean_result::Inspect(shape,p,stop);
    checks["MatchedOrientedBoundary"]=proof.classification==saved_boolean_result::Classification::MatchedOrientedBoundary;
    saved_cut_whole_result::Expected expected;const auto e=saved_boolean_result::detail::GeometryView(p,0);
    saved_cut_whole_result::detail::Graph graph;
    if(!expanded||!saved_cut_whole_result::ExpectedSource(e,expected)
        ||!saved_cut_whole_result::detail::Collect(shape,e,stop,graph,2+disks.size())){
        checks["per-cap-one-opening-per-disk"]=false;return checks;
    }
    std::size_t caps=0;long euler=long(proof.vertices)-long(proof.edges);
    for(const auto& f:graph.faces){euler+=2-long(f.wires.size());
        if(f.wires.size()==1+expected.hostGenus+disks.size()&&!f.surface.cylinder)++caps;}
    checks["per-cap-one-opening-per-disk"]=caps==2;
    checks["genus-aware-Euler"]=euler==2-2*long(expected.hostGenus+disks.size());
    checks["per-disk-cell-counts"]=proof.vertices==expected.vertices.size()+2*disks.size()
        &&proof.edges==expected.edges.size()+3*disks.size()&&proof.faces==expected.faces.size()+disks.size();
    return checks;
}
inline Checks Run(unsigned scenario){
    Checks checks;const auto legacy=wheel::Wheel();const auto ring=Append(legacy);
    checks["ring-admitted"]=bool(ring);if(!ring)return checks;
    const auto p=std::get<retained_boolean::Program>(ring->recipe);const std::atomic_bool stop(false);
    if(scenario==0){
        const auto shape=Build(p);checks=Observe(shape,p);checks["exact-860000-pi"]=wheel::Volume(shape,860000);
        for(unsigned axis=0;axis<3;++axis)for(unsigned count:{3u,6u,16u}){
            analytic_boolean_ring::Ring r{17,analytic_boolean::Axis(axis),{3,4,5},80,2,8./15,count};
            bool good=true;for(unsigned k=0;k<count;++k){auto a=analytic_boolean_ring::Expand(r,k,.001),b=analytic_boolean_ring::Expand(r,k,.001);
                const unsigned u=axis==0?1:0,v=axis==2?1:2;const double angle=2*std::acos(-1.)*k/count;
                good=good&&a.point==b.point&&a.point[axis]==r.center[axis]&&a.identifier==17
                    &&a.point[u]==r.center[u]+80*std::cos(angle)&&a.point[v]==r.center[v]+80*std::sin(angle);}
            checks["deterministic-expansion-"+std::to_string(axis)+"-"+std::to_string(count)]=good;
        }
        for(double unit:{.001,1.0})for(int plane=0;plane<3;++plane){
            const auto e=wheel::Wheel(0,plane,unit);const double mm=unit*1000;
            const auto made=retained_boolean::Apply(e,retained_boolean::AppendRing{{analytic_boolean::Axis(e.axis),{},80/mm,10,6}},mm);
            checks["native-ring-axis-units-"+std::to_string(plane)+"-"+std::to_string(unit)]=made
                &&wheel::Volume(Build(std::get<retained_boolean::Program>(made->recipe)),860000,unit);
        }
        // The whole proof must still reject adversarial periodic ownership.
        auto corrupt=wheel::Clone(shape);
        checks["swapped-ring-seam-refused"]=wheel::SwapSeamVertices(corrupt,10)
            &&saved_boolean_result::Inspect(corrupt,p,stop).classification==saved_boolean_result::Classification::Refused;
        checks["reversed-cap-refused"]=saved_boolean_result::Inspect(wheel::ReversedCap(shape),p,stop).classification
            ==saved_boolean_result::Classification::Refused;
    }else if(scenario==1){
        const auto radius=retained_boolean::Apply(ring->recipe,retained_boolean::SetRingRadius{2,11},1);
        const auto count=retained_boolean::Apply(ring->recipe,retained_boolean::SetRingCount{2,8},1);
        checks["radius-edit-admitted"]=bool(radius);checks["count-edit-admitted"]=bool(count);
        if(radius){auto restored=std::get<retained_boolean::Program>(radius->recipe);restored.steps[1].operand.radius=p.steps[1].operand.radius;
            checks["ring-radius-byte-isolation"]=Bytes(restored)==ring->newBytes;}
        if(count){auto restored=std::get<retained_boolean::Program>(count->recipe);const auto shape=Build(restored);
            checks["exact-852000-pi"]=wheel::Volume(shape,852000);
            checks["eight-hole-boundary"]=saved_boolean_result::Inspect(shape,restored,stop).classification==saved_boolean_result::Classification::MatchedOrientedBoundary;
            restored.steps[1].operand.count=6;checks["ring-count-byte-isolation"]=Bytes(restored)==ring->newBytes;}
        retained_boolean::Program decoded;
        checks["v2-roundtrip"]=retained_boolean::Decode(ring->newBytes,decoded)&&Bytes(decoded)==ring->newBytes&&decoded.codecMinor==2;
        auto corrupted=ring->newBytes;corrupted[corrupted.size()-33]^=1;
        checks["digest-corruption-refused"]=!retained_boolean::Decode(corrupted,decoded);
        const auto bore=retained_boolean::Append(legacy,{analytic_boolean::Axis::Z,{95,0,0},10},1);
        checks["v1-program-byte-exact"]=bore&&bore->newBytes[7]==1&&retained_boolean::Decode(bore->newBytes,decoded)
            &&Bytes(decoded)==bore->newBytes&&decoded.codecMinor==1;
        bool zero=true;for(const auto& step:decoded.steps)zero=zero&&retained_solid::Bits(step.operand.boltCircleRadius)==0
            &&step.operand.count==0&&retained_solid::Bits(step.operand.hostRadiusRatio)==0;
        checks["v1-zero-filled-ring-fields"]=zero&&!decoded.steps.empty();
        checks["legacy-envelope-byte-exact"]=Bytes(legacy)==ring->oldBytes;
    }else if(scenario==2){
        checks["count-two-refused"]=!Append(legacy,80,10,2);checks["count-seventeen-refused"]=!Append(legacy,80,10,17);
        checks["chord-overlap-refused"]=!Append(legacy,80,41,6);
        checks["wrong-axis-refused"]=!Append(legacy,80,10,6,analytic_boolean::Axis::X);
        checks["rim-ligament-refused"]=!Append(legacy,140,10,6);
        checks["hub-ligament-refused"]=!Append(legacy,30,10,6);
        const auto first=Append(legacy,50,2,15);const auto second=first?Append(first->recipe,110,2,16):std::optional<retained_boolean::Change>();
        checks["32-disks-admitted"]=bool(second);
        if(second){const auto& max=std::get<retained_boolean::Program>(second->recipe);const auto bytes=Bytes(max);
            checks["32-disks-complete-boundary"]=!Build(max).IsNull();
            checks["33-disks-pre-kernel-refusal"]=!retained_boolean::Apply(second->recipe,retained_boolean::SetRingCount{2,16},1)&&Bytes(max)==bytes;}
        auto tangent=p.steps[1].operand;tangent.point[0]=240;analytic_boolean::Recipe r;r.metersPerUnit=.001;r.tool=tangent;
        analytic_boolean::Result output;checks["partial-tangent-ring-NoRemovedVolume"]=analytic_boolean::Build(wheel::Base(legacy),r,stop,output)==analytic_boolean::Status::NoRemovedVolume;
    }else if(scenario==3){
        saved_cut_source_values::CirclePatch patch;patch.outerRadius=180;
        saved_program_source_edit::Values values;
        checks["ring-source-preparation"]=saved_boolean_build::PrepareSourceValues(ring->recipe,patch,stop,values,saved_program_source_edit::PrepareValues);
        if(!checks["ring-source-preparation"])return checks;
        const auto& next=values.newProgram;checks["radius-80-to-96"]=next.steps[1].operand.boltCircleRadius==96;
        auto fixed=next;fixed.source.values=p.source.values;fixed.steps[1].operand.boltCircleRadius=80;
        checks["hub-and-all-other-fields-byte-exact"]=Bytes(fixed)==ring->newBytes;
        const auto shape=Build(next);checks["exact-1256000-pi"]=wheel::Volume(shape,1256000);
        checks["held-out-complete-boundary"]=saved_boolean_result::Inspect(shape,next,stop).classification==saved_boolean_result::Classification::MatchedOrientedBoundary;
        patch.outerRadius=50;checks["shrink-refused-wholesale"]=!saved_boolean_build::SourcePatch(p,patch)&&Bytes(p)==ring->newBytes;
        // A polygon host retains the original independent recipe-bounds rule.
        auto polygon=legacy;profile::Parameters rectangular;rectangular.metersPerUnit=.001;
        rectangular.definition.depth=40;rectangular.definition.plane=0;
        rectangular.definition.points={{-150,-150},{150,-150},{150,150},{-150,150}};
        profile::Encode(rectangular,polygon.sourceValues);
        const auto polygonRing=Append(polygon);checks["polygon-ring-admitted"]=bool(polygonRing);
        if(polygonRing){const auto& original=std::get<retained_boolean::Program>(polygonRing->recipe);
            saved_cut_source_values::PolygonPatch edit;
            for(unsigned i=0;i<4;++i){edit.coordinates.push_back({i,saved_cut_source_values::Component::U,rectangular.definition.points[i].X()*1.2});
                edit.coordinates.push_back({i,saved_cut_source_values::Component::V,rectangular.definition.points[i].Y()*1.2});}
            const auto moved=saved_boolean_build::SourcePatch(original,edit);
            checks["polygon-ratio-reanchored"]=moved&&std::abs(std::get<retained_boolean::Program>(moved->recipe).steps[1].operand.boltCircleRadius-96)<1e-12;
            checks["polygon-ring-boundary"]=!Build(original).IsNull();
            checks["polygon-held-out-boundary"]=moved&&!Build(std::get<retained_boolean::Program>(moved->recipe)).IsNull();
        }
        patch.outerRadius=150;const auto noop=saved_boolean_build::SourcePatch(p,patch);
        checks["same-source-byte-exact-noop"]=noop&&!noop->changed&&noop->newBytes==ring->newBytes;
    }else if(scenario==4){
        const auto shape=Build(p);saved_cut_whole_result::detail::Graph graph;
        checks["legacy-default-four-wire-observer-refuses-eight"]=saved_cut_bore_result::Inspect(shape,legacy,legacy,stop).status==saved_cut_bore_result::Status::Refused;
        checks["legacy-default-four-wire-collector-refuses-eight"]=!saved_cut_whole_result::detail::Collect(shape,legacy,stop,graph);
        checks["wire-budget-above34-refused"]=!saved_cut_whole_result::detail::Collect(shape,legacy,stop,graph,35);
        const auto one=wheel::Cut(wheel::Base(legacy),legacy);
        checks["legacy-single-bore-boundary"]=wheel::Matched(one,legacy)&&wheel::Volume(one,884000);
        // Reuse the historical polygon/enclosure recipes and independent builders.
        for(bool box:{false,true}){
            auto e=box?saved_cut_bore_clearance::probe::box(.001):saved_cut_bore_clearance::probe::bracket(.001);
            e.radius=1;retained_boolean::Recipe recipe=e;auto cancellation=std::make_shared<std::atomic_bool>(false);bool matched=false;
            const auto base=saved_cut_bore_result::probe::Base(e,cancellation,matched);
            auto current=wheel::Cut(base,e);bool good=matched&&!current.IsNull();
            for(unsigned n=2;n<=4&&good;++n){
                const std::array<double,3> point=box?std::array<double,3>{40.+10*n,30,0}:std::array<double,3>{30.+5*n,0,4};
                const auto append=retained_boolean::Append(recipe,{analytic_boolean::Axis(e.axis),point,1},1);
                good=bool(append);if(!good)break;recipe=append->recipe;const auto& program=std::get<retained_boolean::Program>(recipe);
                analytic_boolean::Recipe tool;tool.metersPerUnit=.001;tool.tool=program.steps.back().operand;analytic_boolean::Result result;
                good=analytic_boolean::Build(current,tool,stop,result)==analytic_boolean::Status::Built;
                if(good){current=result.solid;good=saved_boolean_result::Inspect(current,program,stop).classification==saved_boolean_result::Classification::MatchedOrientedBoundary;}
                retained_boolean::Program decoded;const auto bytes=Bytes(recipe);
                good=good&&bytes[7]==1&&retained_boolean::Decode(bytes,decoded)&&Bytes(decoded)==bytes;
                checks[std::string(box?"enclosure-":"polygon-")+std::to_string(n)+"-legacy-bounds"]=good;
            }
            checks[std::string(box?"enclosure":"polygon")+"-legacy-fixtures"]=good;
        }
    }else checks["valid-scenario"]=false;
    return checks;
}
} // namespace core3d::saved_boolean_ring_probe
#endif
