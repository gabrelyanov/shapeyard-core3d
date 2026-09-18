#include "SavedBooleanWedgeProbe.hxx"
#include "../Core3D/OCCTKit/DetachedLoftCutProbe.hxx"
#include "../Core3D/OCCTKit/SavedBooleanProgramBuild.hxx"
#include <cassert>
#include <iostream>
#include <limits>
using namespace core3d;
static rectangular_loft::Definition Definition(double unit=.001,unsigned n=3){
    rectangular_loft::Definition d;d.loftIdentifier=1;d.correspondence={10,11,12,13};d.dimensionMetersPerUnit=unit;
    const double f=.001/unit;
    for(unsigned i=0;i<n;++i){rectangular_loft::Station s;s.identifier=100+i;s.cornerIdentifiers={200+4*i,201+4*i,202+4*i,203+4*i};s.correspondence=d.correspondence;
        s.z=(20+60*i)*f;s.centerX=(i%2?6:-0.0)*f;s.centerY=0;s.width=(i%2?100:80)*f;s.depth=(i%2?60:50)*f;d.stations.push_back(s);}return d;
}
static retained_solid::Envelope Envelope(const rectangular_loft::Definition& d){
    retained_solid::Envelope e;for(auto* id:{&e.document,&e.entity,&e.definition,&e.sourceFeature,&e.derivedFeature})(*id)[0]=1;e.derivedFeature[0]=2;
    e.sourceFamily=3;e.sourceSchema=1;e.metersPerUnit=d.dimensionMetersPerUnit;e.radius=12*.001/e.metersPerUnit;
    assert(loft_persistence::Encode(d,e.sourceValues));return e;
}
static void LoftPatchAppliesOnlyAddressedStationScalars(){
    const auto e=Envelope(Definition());const saved_cut_source_values::LoftPatch patch{101,110,{}};
    const auto edited=saved_cut_source_values::Apply(e,patch);assert(edited&&edited->changed);
    for(std::size_t i=0;i<e.sourceValues.size();++i)assert(retained_solid::Bits(edited->envelope.sourceValues[i])==retained_solid::Bits(i==34?110:e.sourceValues[i]));
    auto fixed=edited->envelope;fixed.sourceValues=e.sourceValues;std::vector<std::uint8_t> before,after;
    assert(retained_solid::Encode(e,before)&&retained_solid::Encode(fixed,after)&&before==after);
    const auto same=saved_cut_source_values::Apply(e,saved_cut_source_values::LoftPatch{101,100,{}});assert(same&&!same->changed);
    retained_boolean::Program p;assert(retained_boolean::Promote(e,p));const auto program=saved_boolean_build::SourcePatch(p,patch);assert(program&&program->changed);
}
static void LoftPatchRejectsForeignBitsAndOutOfDomainWidths(){
    auto e=Envelope(Definition());
    for(double bad:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN(),1e20})
        assert(!saved_cut_source_values::Apply(e,saved_cut_source_values::LoftPatch{101,bad,{}}));
    assert(!saved_cut_source_values::Apply(e,saved_cut_source_values::LoftPatch{999,100,{}}));
    assert(!saved_cut_source_values::Apply(e,saved_cut_source_values::LoftPatch{101,{},{}}));
    auto foreign=e;foreign.sourceValues[22]=foreign.sourceValues[8];assert(!saved_cut_source_values::Apply(foreign,saved_cut_source_values::LoftPatch{101,110,{}}));
    foreign=e;foreign.sourceSchema=2;assert(!retained_solid::Valid(foreign));
    foreign=e;foreign.metersPerUnit=1;assert(!retained_solid::Valid(foreign));
    std::vector<std::uint8_t> bytes;assert(retained_solid::Encode(e,bytes));bytes.back()^=1;assert(!retained_solid::Decode(bytes,foreign));
}
static void Geometry(){
    const std::atomic_bool stop(false);
    for(double unit:{.001,1.})for(unsigned count:{2u,3u,8u}){
        const auto d=Definition(unit,count);auto e=Envelope(d);TopoDS_Shape base;
        assert(saved_cut_source_edit::RebuildLoftBase(e,stop,base)==saved_cut_source_edit::LoftBaseStatus::Built);
        saved_cut_loft::Expectation cells;assert(saved_cut_loft::BuildExpectedBoundary(d,cells));
        assert(cells.vertices.size()==4*count&&cells.edges.size()==8*count-4&&cells.faces.size()==4*count-2);
        assert(saved_cut_source_edit::InspectBase(base,e,stop));
        auto wrong=e;wrong.sourceValues[34]+=unit==1?.001:1;assert(!saved_cut_source_edit::InspectBase(base,wrong,stop));
        assert(!saved_cut_source_edit::InspectBase(base.Reversed(),e,stop));
        assert(saved_cut_bore_clearance::Inspect(e).status==saved_cut_bore_clearance::Status::ClearRecipeDisk);
        auto outside=e;outside.point[0]=100*.001/unit;assert(saved_cut_bore_clearance::Inspect(outside).status!=saved_cut_bore_clearance::Status::ClearRecipeDisk);
        outside=e;outside.point[1]=13*.001/unit;assert(saved_cut_bore_clearance::Inspect(outside).status!=saved_cut_bore_clearance::Status::ClearRecipeDisk);
        outside=e;outside.axis=0;assert(saved_cut_bore_clearance::Inspect(outside).status!=saved_cut_bore_clearance::Status::ClearRecipeDisk);
        analytic_boolean::Result cut;assert(analytic_boolean::Build(base,cylindrical_cut::Recipe(e),stop,cut)==analytic_boolean::Status::Built);
        const auto proof=saved_cut_whole_result::Inspect(cut.solid,e,e,stop);
        if(proof.classification!=saved_cut_whole_result::Classification::MatchedOrientedBoundary)std::cerr<<"proof failed "<<proof.phase<<" unit="<<unit<<" count="<<count<<std::endl;
        assert(proof.classification==saved_cut_whole_result::Classification::MatchedOrientedBoundary);
        rectangular_loft::Inspection expected;assert(rectangular_loft::Inspect(d,expected)==rectangular_loft::Admission::Accepted);
        GProp_GProps mass;BRepGProp::VolumeProperties(cut.solid,mass);
        const double wanted=expected.expectedVolume-std::acos(-1.)*e.radius*e.radius*(d.stations.back().z-d.stations.front().z);
        assert(std::abs(mass.Mass()-wanted)<=wanted*1e-9);
        GProp_GProps baseMass;BRepGProp::VolumeProperties(base,baseMass);
        const double removal=std::acos(-1.)*e.radius*e.radius*(d.stations.back().z-d.stations.front().z);
        assert(std::abs(baseMass.Mass()-mass.Mass()-removal)<=removal*1e-9);
        auto patch=saved_cut_source_values::Apply(e,saved_cut_source_values::LoftPatch{101,110*.001/unit,{}});assert(patch);
        TopoDS_Shape edited;assert(saved_cut_source_edit::RebuildLoftBase(patch->envelope,stop,edited)==saved_cut_source_edit::LoftBaseStatus::Built);
        assert(analytic_boolean::Build(edited,cylindrical_cut::Recipe(e),stop,cut)==analytic_boolean::Status::Built);
        assert(saved_cut_whole_result::Inspect(cut.solid,e,patch->envelope,stop).classification==saved_cut_whole_result::Classification::MatchedOrientedBoundary);
        retained_boolean::Program program;assert(retained_boolean::Promote(e,program));auto step=program.steps.front();step.operand.identifier=2;step.operand.point[0]=25*.001/unit;step.operand.radius=4*.001/unit;program.steps.push_back(step);program.nextOperandID=3;
        analytic_boolean::Result first,second;assert(analytic_boolean::Build(base,cylindrical_cut::Recipe(e),stop,first)==analytic_boolean::Status::Built);
        auto other=e;other.point=step.operand.point;other.radius=step.operand.radius;other.operandID=2;
        assert(analytic_boolean::Build(first.solid,cylindrical_cut::Recipe(other),stop,second)==analytic_boolean::Status::Built);
        const auto multi=saved_boolean_result::Inspect(second.solid,program,stop);
        if(multi.classification!=saved_boolean_result::Classification::MatchedOrientedBoundary)std::cerr<<"program failed "<<multi.phase<<std::endl;
        assert(multi.classification==saved_boolean_result::Classification::MatchedOrientedBoundary);
    }
}
static void ConstructionFrames(){
    const std::atomic_bool stop(false);
    for(double unit:{.001,1.})for(unsigned orientation=0;orientation<5;++orientation){
        auto d=Definition(unit);profile::ConstructionFrame frame;
        const double f=.001/unit,q=std::sqrt(.5);
        frame.values={3*f,4*f,5*f,0,0,0,1,2};
        if(orientation){frame.values[3+(orientation>2)]=(orientation%2?1:-1)*q;frame.values[6]=q;}
        d.constructionFrame=frame;auto e=Envelope(d);gp_Trsf transform;assert(frame.Transform(transform));
        const auto p=gp_Pnt(0,0,0).Transformed(transform);e.point={p.X(),p.Y(),p.Z()};e.radius*=2;
        e.axis=orientation==0?2:orientation<=2?1:0;
        TopoDS_Shape base;assert(saved_cut_source_edit::RebuildLoftBase(e,stop,base)==saved_cut_source_edit::LoftBaseStatus::Built);
        assert(saved_cut_bore_clearance::Inspect(e).status==saved_cut_bore_clearance::Status::ClearRecipeDisk);
        analytic_boolean::Result cut;assert(analytic_boolean::Build(base,cylindrical_cut::Recipe(e),stop,cut)==analytic_boolean::Status::Built);
        const auto proof=saved_cut_whole_result::Inspect(cut.solid,e,e,stop);
        if(proof.classification!=saved_cut_whole_result::Classification::MatchedOrientedBoundary)std::cerr<<"frame failed "<<orientation<<" "<<proof.phase<<std::endl;
        assert(proof.classification==saved_cut_whole_result::Classification::MatchedOrientedBoundary);
    }
    auto d=Definition();profile::ConstructionFrame negative;negative.values[7]=-1;d.constructionFrame=negative;
    const auto e=Envelope(d);TopoDS_Shape empty;
    assert(saved_cut_source_edit::RebuildLoftBase(e,stop,empty)==saved_cut_source_edit::LoftBaseStatus::BoundaryMismatch&&empty.IsNull());
    assert(saved_cut_bore_clearance::Inspect(e).status==saved_cut_bore_clearance::Status::UnsupportedFrame);
}
int main(){std::size_t wedgeChecks=0;for(unsigned scenario=0;scenario<4;++scenario)for(const auto& row:core3d::saved_boolean_wedge_probe::Run(scenario)){++wedgeChecks;if(!row.second)std::cerr<<"wedge "<<scenario<<" "<<row.first<<std::endl;assert(row.second);}std::cout<<"Wedge native checks passed: "<<wedgeChecks<<"\n";for(const auto& row:detached_loft_cut_probe::Run()){if(!row.second)std::cerr<<row.first<<std::endl;assert(row.second);}LoftPatchAppliesOnlyAddressedStationScalars();LoftPatchRejectsForeignBitsAndOutOfDomainWidths();Geometry();ConstructionFrames();std::cout<<"Loft cut patch and native boundary tests passed\n";}
