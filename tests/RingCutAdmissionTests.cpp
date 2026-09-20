// B04/D34 exact revolved-host admission, boundary and retained replay regression.
#include "CircularHostProofProbe.hxx"
#include "SavedBooleanProgramBuild.hxx"
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <cassert>
#include <iostream>

using namespace core3d;
namespace fixture = saved_cut_circular_host::probe;

static retained_solid::Envelope wheel(int plane=0) {
    auto e = saved_cut_bore_clearance::probe::identity(.001);
    e.sourceFamily = 1;
    e.sourceSchema = 1;
    e.axis = plane==0?1:2;
    e.radius = 20;
    profile::Parameters p;
    p.metersPerUnit = .001;
    p.definition.plane = plane;
    p.definition.revolve = true;
    p.definition.depth = 360;
    p.definition.points = {{0, -20}, {140, -20}, {140, 20}, {0, 20}};
    assert(profile::Encode(p, e.sourceValues));
    assert(retained_solid::Valid(e));
    return e;
}

// Independent production-style profile construction, with no proof cells.
static TopoDS_Shape revolvedBase(const retained_solid::Envelope& e) {
    profile::Parameters p;assert(profile::Decode(e.sourceValues,p));
    double area=0,volume=0;assert(ProfileDefinitionExpectedVolume(p.definition,area,volume));
    auto points=p.definition.points;if(area<0)std::reverse(points.begin(),points.end());
    BRepBuilderAPI_MakePolygon outline;
    for(const auto& point:points)outline.Add(enclosure_correspondence::PlanePoint(point.X(),point.Y(),0,p.definition.plane));
    outline.Close();assert(outline.IsDone());
    BRepBuilderAPI_MakeFace face(outline.Wire(),Standard_True);assert(face.IsDone());
    BRepPrimAPI_MakeRevol revolve(face.Face(),gp_Ax1(gp::Origin(),p.definition.plane==0?gp::DY():gp::DZ()),Standard_True);
    assert(revolve.IsDone());TopoDS_Shape base=revolve.Shape();
    if(p.constructionFrame){gp_Trsf transform;assert(p.constructionFrame->Transform(transform));
        base=BRepBuilderAPI_Transform(base,transform,Standard_True,Standard_False).Shape();}
    return base;
}

static void validVolume(const TopoDS_Shape& shape, double timesPi) {
    assert(!shape.IsNull());
    assert(BRepCheck_Analyzer(shape, Standard_True).IsValid());
    unsigned solids = 0;
    for (TopExp_Explorer it(shape, TopAbs_SOLID); it.More(); it.Next()) ++solids;
    assert(solids == 1);
    assert(fixture::Volume(shape, timesPi));
}

int main() {
    const auto e = wheel();
    retained_boolean::Program p;
    assert(retained_boolean::Promote(e, p));
    analytic_boolean_ring::Ring ring{2, analytic_boolean::Axis::Y, {}, 95, 15, 95. / 140., 6};
    assert(analytic_boolean_ring::Inspect(ring, .001) == analytic_boolean_ring::Status::Clear);
    auto operand = analytic_boolean_ring::Expand(ring, 0, .001);
    assert(operand.identifier == 2 && operand.radius == 15 && operand.point[0] == 95);

    saved_cut_whole_result::Expected expected;
    assert(saved_cut_whole_result::ExpectedSource(e, expected));
    assert(expected.faces.size()==3 && expected.edges.size()==3 && expected.vertices.size()==2);
    double extent = 0;
    auto center = operand; center.point = {};
    assert(saved_boolean_result::detail::HostRadialExtent(p,center,extent) && extent==140);
    assert(saved_cut_bore_clearance::Inspect(e).status == saved_cut_bore_clearance::Status::ClearRecipeDisk);
    const auto admitted=retained_boolean::Apply(e,
        retained_boolean::AppendRing{{analytic_boolean::Axis::Y, {}, 95, 15, 6}}, 1);
    assert(admitted && admitted->changed);
    const auto& program=std::get<retained_boolean::Program>(admitted->recipe);
    assert(program.source.values==e.sourceValues && program.steps.size()==2);
    assert(program.steps.back().operand.hostRadiusRatio==95./140.);
    std::vector<std::uint8_t> before,after;
    assert(retained_boolean::Encode(program,before));
    retained_boolean::Program decoded;
    assert(retained_boolean::Decode(before,decoded));
    assert(retained_boolean::Encode(decoded,after) && before==after);

    profile::Parameters source;
    assert(profile::Decode(e.sourceValues,source));
    const auto base = revolvedBase(e);
    validVolume(base, 784000);
    const auto baseBytes=fixture::Bytes(base);
    const std::atomic_bool stop(false);
    assert(saved_boolean_build::InspectSourceBase(base,e,stop));
    auto cut = fixture::Cut(base, e);
    validVolume(cut, 768000);
    const auto hubProof=saved_boolean_result::InspectAxialBore(cut,e,e,stop);
    std::cout << "hub proof: " << hubProof.phase << std::endl;
    assert(hubProof.classification==saved_boolean_result::Classification::MatchedOrientedBoundary);
    assert(hubProof.faces==4);
    for (unsigned n = 0; n < ring.count; ++n) {
        const auto disk = analytic_boolean_ring::Expand(ring, n, .001);
        const auto view = saved_boolean_result::detail::GeometryView(p, disk);
        assert(saved_cut_bore_clearance::Inspect(view).status == saved_cut_bore_clearance::Status::ClearRecipeDisk);
        cut = fixture::Cut(cut, view);
        validVolume(cut, 768000 - 9000 * (n + 1));
    }
    std::cout << "B04 detached kernel: R140 x 40 revolve, r20 hub, 6 x r15 @ R95; "
                 "valid single solid; volume=" << fixture::VolumeMM3(cut)
              << " mm^3 (714000*pi); rim gap=30; hub gap=60; adjacent chord gap=65 mm\n";

    const auto proof=saved_boolean_result::Inspect(cut,program,stop);
    std::cout << "ring proof: " << proof.phase << std::endl;
    assert(proof.classification==saved_boolean_result::Classification::MatchedOrientedBoundary);
    assert(proof.faces==10 && proof.edges==24 && proof.vertices==16);
    cut_display::Settings display;display.automatic=true;display.values={.001,.5,.001,.001,.5};
    const auto built=saved_boolean_build::Build(base,program,display,stop);
    std::cout << "build: " << built.phase << "/" << built.correspondence.phase << std::endl;
    assert(built.status==saved_boolean_build::Status::Built);
    validVolume(built.solid,714000);
    const auto replay=saved_boolean_build::Build(fixture::Clone(base),decoded,display,stop);
    assert(replay.status==saved_boolean_build::Status::Built);
    assert(replay.exactProgram==built.exactProgram);
    saved_boolean_build::Commitment a,b;std::size_t bytes=0;
    assert(saved_boolean_build::GeometryCommit(built.solid,stop,bytes,a));
    assert(saved_boolean_build::GeometryCommit(replay.solid,stop,bytes,b) && a==b);
    // Match the document's binary readback and the worker's deep-copy path.
    std::stringstream binary;BinTools::Write(base,binary,Standard_False,Standard_False,BinTools_FormatVersion_VERSION_4);
    TopoDS_Shape reopened;BinTools::Read(reopened,binary);assert(!reopened.IsNull());
    BRepBuilderAPI_Copy copy(reopened,Standard_True,Standard_False);assert(copy.IsDone());
    std::cout << "binary base=" << saved_boolean_build::InspectSourceBase(reopened,e,stop)
        << " copy=" << saved_boolean_build::InspectSourceBase(copy.Shape(),e,stop) << std::endl;
    const auto reopenedBuild=saved_boolean_build::Build(copy.Shape(),decoded,display,stop);
    assert(reopenedBuild.status==saved_boolean_build::Status::Built);
    assert(saved_boolean_build::GeometryCommit(reopenedBuild.solid,stop,bytes,b) && a==b);
    assert(!saved_boolean_build::InspectSourceBase(cut,e,stop));
    assert(saved_boolean_result::Inspect(base,program,stop).classification==saved_boolean_result::Classification::Refused);
    auto shifted=e;shifted.point[0]=1;
    const auto wrongHub=fixture::Cut(base,shifted);
    assert(saved_boolean_result::InspectAxialBore(wrongHub,e,e,stop).classification==saved_boolean_result::Classification::Refused);
    auto swapped=fixture::Clone(cut);
    assert(fixture::SwapSeamVertices(swapped,140));
    assert(saved_boolean_result::Inspect(swapped,program,stop).classification==saved_boolean_result::Classification::Refused);
    const auto ringStatus=[&](double bolt,double radius,unsigned count){
        analytic_boolean::Operand candidate;
        candidate.identifier=2;candidate.kind=analytic_boolean::OperandKind::CylinderRing;
        candidate.axis=analytic_boolean::Axis::Y;candidate.boltCircleRadius=bolt;
        candidate.radius=radius;candidate.hostRadiusRatio=bolt/140.;candidate.count=count;
        return saved_boolean_result::detail::RingAdmissionStatus(p,candidate);
    };
    assert(ringStatus(125,15,6)==analytic_boolean_ring::Status::OutsideOrInsufficientLigament);
    assert(ringStatus(124.999,15,6)==analytic_boolean_ring::Status::OutsideOrInsufficientLigament);
    assert(ringStatus(34,15,6)==analytic_boolean_ring::Status::OutsideOrInsufficientLigament);
    assert(ringStatus(60,25,8)==analytic_boolean_ring::Status::OverlappingHoles);
    // Exact rim tangency, insufficient .001 mm ligament, hub overlap,
    // and mutually overlapping ring disks must refuse without recipe mutation.
    for(const auto& bad:std::vector<cylindrical_cut::RingCreateEdit>{
        {analytic_boolean::Axis::Y,{},125,15,6},
        {analytic_boolean::Axis::Y,{},124.999,15,6},
        {analytic_boolean::Axis::Y,{},124.998,15,6},
        {analytic_boolean::Axis::Y,{},35.002,15,6},
        {analytic_boolean::Axis::Y,{},60,30,6},
        {analytic_boolean::Axis::Y,{},34,15,6},
        {analytic_boolean::Axis::Y,{},60,25,8}}){
        assert(!retained_boolean::Apply(e,retained_boolean::AppendRing{bad},1));
    }
    for(double bolt:{124.997,35.003})assert(retained_boolean::Apply(e,
        retained_boolean::AppendRing{{analytic_boolean::Axis::Y,{},bolt,15,6}},1));
    const analytic_boolean_ring::Ring overlapping{2,analytic_boolean::Axis::Y,{},60,25,60./140.,8};
    assert(analytic_boolean_ring::Inspect(overlapping,.001)==analytic_boolean_ring::Status::OverlappingHoles);
    auto tangent=e;tangent.point[0]=125;tangent.radius=15;
    assert(saved_cut_bore_clearance::Inspect(tangent).status==saved_cut_bore_clearance::Status::OutsideOrInsufficientLigament);
    auto wrongAxis=e;wrongAxis.axis=0;
    assert(saved_cut_bore_clearance::Inspect(wrongAxis).status==saved_cut_bore_clearance::Status::Nonparallel);
    assert(retained_boolean::Encode(program,after) && before==after);
    // A bounding cylinder must never certify a tapered or partial revolution.
    for(bool partial:{false,true}){
        auto invalid=e;auto definition=source;
        if(partial)definition.definition.depth=180;
        else definition.definition.points[2].SetX(130);
        assert(profile::Encode(definition,invalid.sourceValues));
        assert(!saved_cut_whole_result::ExpectedSource(invalid,expected));
        assert(saved_cut_bore_clearance::Inspect(invalid).status==saved_cut_bore_clearance::Status::UnsupportedFamily);
    }

    // All authored revolve planes use the revolve axis, not the sketch normal.
    // A translated/scaled source and reverse polygon winding retain that rule.
    for(int plane=0;plane<3;++plane)for(bool framed:{false,true}){
        auto sourceEnvelope=wheel(plane);profile::Parameters profile;assert(core3d::profile::Decode(sourceEnvelope.sourceValues,profile));
        double scale=1;std::array<double,3> center{};
        if(framed){scale=2;core3d::profile::ConstructionFrame frame;frame.values={3,4,5,0,0,0,1,scale};
            profile.constructionFrame=frame;center={3,4,5};sourceEnvelope.point=center;sourceEnvelope.radius*=scale;
            std::reverse(profile.definition.points.begin(),profile.definition.points.end());
            sourceEnvelope.sourceSchema=core3d::profile::SchemaFor(profile);
            assert(core3d::profile::Encode(profile,sourceEnvelope.sourceValues));}
        const auto sourceBase=revolvedBase(sourceEnvelope);
        assert(saved_boolean_build::InspectSourceBase(sourceBase,sourceEnvelope,stop));
        const auto hubShape=fixture::Cut(sourceBase,sourceEnvelope);
        assert(saved_boolean_result::InspectAxialBore(hubShape,sourceEnvelope,sourceEnvelope,stop).classification
            ==saved_boolean_result::Classification::MatchedOrientedBoundary);
        const auto append=retained_boolean::Apply(sourceEnvelope,retained_boolean::AppendRing{{
            analytic_boolean::Axis(sourceEnvelope.axis),center,95*scale,15*scale,6}},1);
        assert(append);
        const auto builtCase=saved_boolean_build::Build(sourceBase,std::get<retained_boolean::Program>(append->recipe),display,stop);
        std::cout << "plane=" << plane << " scale=" << scale << " phase=" << builtCase.phase << "/" << builtCase.correspondence.phase << std::endl;
        assert(builtCase.status==saved_boolean_build::Status::Built);
        validVolume(builtCase.solid,714000*scale*scale*scale);
    }
    const int previousRounding=std::fegetround();std::fesetround(FE_UPWARD);
    const auto roundingStatus=saved_cut_bore_clearance::Inspect(e).status;std::fesetround(previousRounding);
    assert(roundingStatus==saved_cut_bore_clearance::Status::NumericUncertain);
    assert(fixture::Bytes(base)==baseBytes);
    std::atomic_bool cancelled(true);
    assert(saved_boolean_build::Build(base,program,display,cancelled).status==saved_boolean_build::Status::Cancelled);
    assert(saved_boolean_result::Inspect(cut,program,cancelled).classification==saved_boolean_result::Classification::Cancelled);
    for(unsigned scenario=0;scenario<6;++scenario)for(const auto& check:fixture::Run(scenario)){
        if(!check.second)std::cout << "circular regression " << scenario << "/" << check.first << " FAILED" << std::endl;
        assert(check.second);
    }
    // R150 disk + r20 bore at 130 is tangent; annulus inner R40 + r10
    // bore at 49 overlaps by 1, and at 50.001 leaves only .001 mm (< .002).
    for (const auto& check : fixture::RunClearanceIntervals()) {
        std::cout << "existing clearance " << check.first << "=" << check.second << '\n';
        assert(check.second);
    }
    std::cout << "PASS B04 exact revolve + hub + six-hole ring admission, boundary and retained replay\n";
}
