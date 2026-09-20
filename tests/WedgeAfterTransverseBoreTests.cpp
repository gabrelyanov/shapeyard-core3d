// D35 complete transverse program admission, exact build and replay regression.
#include "SavedBooleanProgramBuild.hxx"
#include "RectangularLoftSolid.hxx"
#include "SavedBooleanWedgeProbe.hxx"
#include "RetainedFilletCandidates.hxx"
#include <cassert>
#include <iostream>
using namespace core3d;
static retained_solid::Envelope eye() {
    rectangular_loft::Definition d;
    d.loftIdentifier=1;d.correspondence={10,11,12,13};d.dimensionMetersPerUnit=.001;
    const double z[]={0,55,100,130},width[]={8,26,28,30},depth[]={30,30,32,34};
    for(unsigned i=0;i<4;++i){rectangular_loft::Station s;s.identifier=100+i;
        s.cornerIdentifiers={200+4*i,201+4*i,202+4*i,203+4*i};s.correspondence=d.correspondence;
        s.z=z[i];s.width=width[i];s.depth=depth[i];d.stations.push_back(s);}
    retained_solid::Envelope e;
    for(auto* id:{&e.document,&e.entity,&e.definition,&e.sourceFeature,&e.derivedFeature})(*id)[0]=1;
    e.derivedFeature[0]=2;e.sourceFamily=3;e.sourceSchema=1;e.metersPerUnit=.001;
    e.axis=0;e.point={0,0,100};e.radius=12;e.operandID=1;
    assert(loft_persistence::Encode(d,e.sourceValues));return e;
}
static TopoDS_Shape base(const retained_solid::Envelope& e){
    rectangular_loft::Definition d;assert(loft_persistence::Decode(e.sourceValues,d));
    rectangular_loft::Admission admission;auto prepared=rectangular_loft::Prepare(d,admission);assert(prepared);
    rectangular_loft::SolidResult result;std::atomic_bool stop(false);
    assert(rectangular_loft::Build(prepared,stop,result)==rectangular_loft::BuildStatus::Built);return result.solid;
}
static double volume(const TopoDS_Shape& s){GProp_GProps p;BRepGProp::VolumeProperties(s,p,1e-12,Standard_True);return p.Mass();}
static std::string bytes(const TopoDS_Shape& s){std::ostringstream stream;BRepTools::Write(s,stream,Standard_False,Standard_False,TopTools_FormatVersion_VERSION_3);return stream.str();}
static std::string displayBytes(const TopoDS_Shape& s){std::ostringstream stream;BRepTools::Write(s,stream,Standard_True,Standard_True,TopTools_FormatVersion_VERSION_3);return stream.str();}
static TopoDS_Shape nativeReadback(const TopoDS_Shape& s){
    std::stringstream stream;BinTools::Write(s,stream,Standard_True,Standard_True,BinTools_FormatVersion_VERSION_4);
    assert(stream.good());TopoDS_Shape readback;BinTools::Read(readback,stream);
    assert(stream.good()&&!readback.IsNull());return readback;
}
static void sameCensus(const TopoDS_Shape& a,const TopoDS_Shape& b){
    for(auto type:{TopAbs_VERTEX,TopAbs_EDGE,TopAbs_WIRE,TopAbs_FACE,TopAbs_SHELL,TopAbs_SOLID}){
        TopTools_IndexedMapOfShape first,second;TopExp::MapShapes(a,type,first);TopExp::MapShapes(b,type,second);
        assert(first.Extent()==second.Extent());
    }
}
static void coldReplayIdentity(const TopoDS_Shape& source,const retained_boolean::Program& p,
    const cut_display::Settings& settings,const std::atomic_bool& stop){
    const auto built=saved_boolean_build::Build(source,p,settings,stop);
    assert(built.status==saved_boolean_build::Status::Built);
    const auto sourceBytes=displayBytes(source);
    // A fresh deterministic build alone misses native gp_Dir/gp_Ax readback.
    // Compare the committed solid with TWO cold opens, including display data.
    auto reopened=built.solid;
    for(unsigned cycle=0;cycle<2;++cycle){
        reopened=nativeReadback(reopened);sameCensus(built.solid,reopened);
        assert(bytes(built.solid)==bytes(reopened));
        assert(displayBytes(built.solid)==displayBytes(reopened));
        saved_boolean_build::Budget budget;
        assert(saved_boolean_build::VerifyCurrent(nativeReadback(source),reopened,p,settings,stop,budget));
    }
    retained_boolean::Program decoded;assert(retained_boolean::Decode(built.exactProgram,decoded));
    std::vector<std::uint8_t> encoded;assert(retained_boolean::Encode(decoded,encoded)&&encoded==built.exactProgram);
    const auto replay=saved_boolean_build::Build(nativeReadback(source),decoded,settings,stop);
    assert(replay.status==saved_boolean_build::Status::Built&&replay.exactProgram==built.exactProgram);
    sameCensus(built.solid,replay.solid);
    assert(bytes(built.solid)==bytes(replay.solid));
    assert(displayBytes(built.solid)==displayBytes(replay.solid));
    assert(displayBytes(source)==sourceBytes);
}
int main(){
    const auto e=eye();std::atomic_bool stop(false);
    assert(saved_cut_bore_clearance::Inspect(e).status==saved_cut_bore_clearance::Status::ClearRecipeTransverse);
    // Promote copies the part-local axis verbatim. Neither the station axis nor
    // an occurrence rotation is an operand-axis encoding.
    retained_boolean::Program p;assert(retained_boolean::Promote(e,p));
    analytic_boolean::Operand wedge;wedge.identifier=2;wedge.kind=analytic_boolean::OperandKind::Wedge;
    wedge.axis=analytic_boolean::Axis::X;wedge.point={0,0,0};wedge.directionAngle=std::acos(-1.)/2;
    wedge.halfWidthApex=1;wedge.halfWidthMouth=6;wedge.length=55;
    assert(p.steps.front().operand.axis==wedge.axis);
    p.codecMinor=3;p.nextOperandID=3;p.steps.push_back({analytic_boolean::Operation::Difference,wedge});
    assert(retained_boolean::Valid(p));
    assert(saved_boolean_result::detail::SeparateConvexSections(p));
    std::vector<std::uint8_t> encoded,roundtrip;assert(retained_boolean::Encode(p,encoded));
    retained_boolean::Program decoded;assert(retained_boolean::Decode(encoded,decoded));
    assert(retained_boolean::Encode(decoded,roundtrip)&&encoded==roundtrip);
    assert(decoded.steps[0].operand.axis==decoded.steps[1].operand.axis);
    const auto view=saved_boolean_result::detail::GeometryView(p,wedge);
    saved_cut_whole_result::Expected expected;assert(saved_cut_whole_result::ExpectedSource(view,expected));
    const auto admission=analytic_boolean_wedge::ExpectedBoundary(view,wedge,expected);
    assert(admission.status==analytic_boolean_wedge::Status::Clear&&admission.open);
    assert(expected.transverseProgram&&std::abs(admission.removedVolume-7370.)<1e-9);
    assert(saved_boolean_result::detail::AdmitSections(p));
    wedge_cut::CreateEdit edit{wedge.axis,wedge.point,wedge.directionAngle,1,6,55};
    const auto applied=retained_boolean::Apply(e,retained_boolean::AppendWedge{edit},1);
    assert(applied&&applied->changed&&applied->newBytes==encoded);
    const auto original=base(e);const auto originalBytes=bytes(original);
    analytic_boolean::Result bored;assert(analytic_boolean::Build(original,cylindrical_cut::Recipe(e),stop,bored)==analytic_boolean::Status::Built);
    // Independent integral of affine thickness * affine slot width, z=0..55:
    // integral (8+18t)*(2+10t)*55 dt = 7370 mm^3.
    const double constant=8.*2.;
    const double linear=(8.*10.+18.*2.)/2.;
    const double quadratic=18.*10./3.;
    const double removed=55.*(constant+linear+quadratic);
    analytic_boolean::Recipe recipe;recipe.metersPerUnit=.001;recipe.operation=analytic_boolean::Operation::Difference;recipe.tool=wedge;
    analytic_boolean::Result cut;assert(analytic_boolean::Build(bored.solid,recipe,stop,cut,removed)==analytic_boolean::Status::Built);
    assert(std::abs(volume(bored.solid)-volume(cut.solid)-removed)<removed*1e-9);
    // Independently authored prism uses exact Y/Z coordinates and a fixed X
    // span, without Expand() or the production builder's bounds/margin.
    BRepBuilderAPI_MakePolygon outline;
    for(const auto& yz:std::array<std::array<double,2>,4>{{{1,0},{6,55},{-6,55},{-1,0}}})outline.Add(gp_Pnt(-100,yz[0],yz[1]));
    outline.Close();BRepBuilderAPI_MakeFace face(outline.Wire());
    BRepPrimAPI_MakePrism tool(face.Face(),gp_Vec(200,0,0));
    BRepBuilderAPI_Copy copy(bored.solid,Standard_True,Standard_False);
    BRepAlgoAPI_Cut independent(copy.Shape(),tool.Shape());assert(independent.IsDone());
    assert(BRepCheck_Analyzer(independent.Shape()).IsValid());
    assert(std::abs(volume(independent.Shape())-volume(cut.solid))<removed*1e-9);
    analytic_boolean::Result replayEye,replay;
    analytic_boolean::Recipe eyeRecipe;eyeRecipe.metersPerUnit=.001;eyeRecipe.tool=decoded.steps[0].operand;
    assert(analytic_boolean::Build(base(e),eyeRecipe,stop,replayEye)==analytic_boolean::Status::Built);
    recipe.tool=decoded.steps[1].operand;
    assert(analytic_boolean::Build(replayEye.solid,recipe,stop,replay,removed)==analytic_boolean::Status::Built);
    assert(bytes(cut.solid)==bytes(replay.solid));assert(bytes(original)==originalBytes);
    const auto proof=saved_boolean_result::Inspect(cut.solid,p,stop);
    if(proof.classification!=saved_boolean_result::Classification::MatchedOrientedBoundary)std::cerr<<proof.phase<<std::endl;
    assert(proof.classification==saved_boolean_result::Classification::MatchedOrientedBoundary);
    cut_display::Settings settings;settings.values={.001,.5,.1,0,0};settings.automatic=true;
    const auto built=saved_boolean_build::Build(original,p,settings,stop);
    if(built.status!=saved_boolean_build::Status::Built)std::cerr<<built.phase<<":"<<built.correspondence.phase<<std::endl;
    assert(built.status==saved_boolean_build::Status::Built&&built.exactProgram==encoded);
    const auto rebuilt=saved_boolean_build::Build(base(e),decoded,settings,stop);
    assert(rebuilt.status==saved_boolean_build::Status::Built&&rebuilt.exactProgram==encoded);
    assert(bytes(built.solid)==bytes(rebuilt.solid));
    coldReplayIdentity(original,p,settings,stop);
    // D37: exact B05 striking-face dimensions and radius, one retained step.
    const auto candidates=retained_fillet::DiscoverCandidates(built.solid,p,original);
    assert(candidates.status==retained_fillet::CandidateStatus::Available);
    retained_boolean::AppendFilletStep append;append.radiusMM=2;
    for(const auto& candidate:candidates.values)if(candidate.anchor.curveKind==retained_fillet::CurveKind::Line
        &&std::abs(candidate.anchor.anchorPoint[2]-130)<1e-9)append.anchors.push_back(candidate.anchor);
    assert(append.anchors.size()==4);
    const auto change=retained_boolean::Apply(p,append,1);assert(change);
    const auto roundedProgram=std::get<retained_boolean::Program>(change->recipe);
    assert(roundedProgram.filletSteps.size()==1&&roundedProgram.filletSteps[0].radiusLocal==2);
    const auto rounded=saved_boolean_build::Build(original,roundedProgram,settings,stop);
    if(rounded.status!=saved_boolean_build::Status::Built)std::cerr<<"D37 "<<rounded.phase<<":"<<retained_fillet::Reason(rounded.filletOutcome)<<std::endl;
    assert(rounded.status==saved_boolean_build::Status::Built);
    assert(saved_boolean_result::Inspect(built.solid,roundedProgram,stop).classification==saved_boolean_result::Classification::Refused);
    const auto fillet=retained_fillet::Build(built.solid,roundedProgram,stop);
    assert(fillet.outcome==retained_fillet::Outcome::Built&&fillet.intervals.size()==1);
    const double filletLoss=volume(built.solid)-volume(rounded.solid);
    assert(filletLoss>0&&fillet.intervals[0].lower==fillet.intervals[0].upper);
    assert(std::abs(filletLoss-fillet.intervals[0].lower)<filletLoss*1e-9);
    // Analytic angle/length bound is independent of both OCCT rounds.
    retained_fillet::Interval analytic;
    assert(retained_fillet::AnalyticExpectedRemoval(roundedProgram,roundedProgram.filletSteps[0],analytic));
    assert(filletLoss>=analytic.lower&&filletLoss<=analytic.upper);
    coldReplayIdentity(original,roundedProgram,settings,stop);
    saved_boolean_build::Budget mismatchBudget;
    auto wrongRadius=roundedProgram;wrongRadius.filletSteps[0].radiusLocal=1.5;
    assert(!saved_boolean_build::VerifyCurrent(original,rounded.solid,wrongRadius,settings,stop,mismatchBudget));
    const auto removal=retained_boolean::Apply(roundedProgram,
        retained_boolean::RemoveFilletStep{roundedProgram.filletSteps[0].stepIdentifier},1);assert(removal);
    const auto restored=saved_boolean_build::Build(original,std::get<retained_boolean::Program>(removal->recipe),settings,stop);
    assert(restored.status==saved_boolean_build::Status::Built&&bytes(restored.solid)==bytes(built.solid));
    // A convex edge created by the wedge, absent from the source oracle.
    retained_boolean::AppendFilletStep slotRound;slotRound.radiusMM=.25;
    for(const auto& candidate:candidates.values)if(std::abs(candidate.anchor.anchorPoint[2]-27.5)<1e-9
        &&candidate.anchor.anchorPoint[0]>0&&std::abs(candidate.anchor.anchorPoint[1]-3.5)<1e-9)slotRound.anchors.push_back(candidate.anchor);
    assert(slotRound.anchors.size()==1);
    const auto slotChange=retained_boolean::Apply(p,slotRound,1);assert(slotChange);
    const auto slotProgram=std::get<retained_boolean::Program>(slotChange->recipe);
    retained_fillet::Interval slotAnalytic;
    assert(!retained_fillet::AnalyticExpectedRemoval(slotProgram,slotProgram.filletSteps[0],slotAnalytic));
    const auto slotBuilt=saved_boolean_build::Build(original,slotProgram,settings,stop);
    // This oblique slot edge is geometrically provable, but gp_Dir readback
    // keeps changing its representation. Do not silently relax replay identity.
    assert(slotBuilt.status==saved_boolean_build::Status::Refused);
    assert(slotBuilt.filletOutcome==retained_fillet::Outcome::DeclinedReplayIdentity);
    const auto slotKernel=retained_fillet::Build(built.solid,slotProgram,stop);
    assert(slotKernel.outcome==retained_fillet::Outcome::Built);
    assert(volume(slotKernel.solid)<volume(built.solid));
    assert(retained_fillet::ExpectedRemoval(slotProgram,slotProgram.filletSteps[0],slotAnalytic));
    assert(std::abs(slotAnalytic.lower-(volume(built.solid)-volume(slotKernel.solid)))<slotAnalytic.lower*1e-6);
    auto slotReadback=slotKernel.solid;
    for(unsigned cycle=0;cycle<8;++cycle){const auto next=nativeReadback(slotReadback);
        sameCensus(slotReadback,next);assert(bytes(slotReadback)!=bytes(next));slotReadback=next;}
    std::cout<<"PASS refusal: wedge-created oblique line midpoint (8.5,3.5,27.5), radius .25, native bytes unstable across 8 readbacks.\n";
    TopTools_IndexedMapOfShape cutEdges;TopExp::MapShapes(built.solid,TopAbs_EDGE,cutEdges);
    std::map<int,unsigned> unsupportedCurves;
    for(int i=1;i<=cutEdges.Extent();++i){BRepAdaptor_Curve curve(TopoDS::Edge(cutEdges(i)));
        if(curve.GetType()!=GeomAbs_Line&&curve.GetType()!=GeomAbs_Circle)++unsupportedCurves[int(curve.GetType())];}
    assert(unsupportedCurves.size()==1&&unsupportedCurves[int(GeomAbs_Ellipse)]==6);
    std::cout<<"BLOCKED edge class: 6 elliptical bore trims cannot be represented by the retained Line/Circle anchor schema.\n";
    std::cout<<"PASS D37 four Z130 edges, radius 2: removal "<<filletLoss<<" mm^3; exact expectation and cold replay.\n";

    assert(saved_boolean_result::Inspect(bored.solid,p,stop).classification==saved_boolean_result::Classification::Refused);
    auto changed=p;changed.steps.back().operand.halfWidthMouth=7;
    assert(saved_boolean_result::Inspect(cut.solid,changed,stop).classification==saved_boolean_result::Classification::Refused);
    auto bad=p;bad.steps.back().operand.halfWidthMouth=16;
    const auto badView=saved_boolean_result::detail::GeometryView(bad,bad.steps.back().operand);
    assert(analytic_boolean_wedge::TransverseBoundary(badView,bad.steps.back().operand).status==analytic_boolean_wedge::Status::TransverseSideWall);
    assert(!saved_boolean_result::detail::AdmitSections(bad));
    assert(saved_boolean_build::Build(original,bad,settings,stop).status==saved_boolean_build::Status::Refused);
    auto closed=p;closed.steps.back().operand.point[2]=1;
    const auto closedAdmission=analytic_boolean_wedge::TransverseBoundary(view,closed.steps.back().operand);
    assert(closedAdmission.status==analytic_boolean_wedge::Status::Clear&&!closedAdmission.open);
    assert(saved_boolean_build::Build(original,closed,settings,stop).status==saved_boolean_build::Status::Built);
    coldReplayIdentity(original,closed,settings,stop);
    auto outside=p;outside.steps.back().operand.point[2]=-.01;
    assert(analytic_boolean_wedge::TransverseBoundary(view,outside.steps.back().operand).status==analytic_boolean_wedge::Status::BoundaryApex);
    // Native persistence readback remains bound to the recipe, including all
    // analytic representation bytes (no tolerance-based matching substitute).
    std::stringstream persisted;BinTools::Write(built.solid,persisted,Standard_False,Standard_False,BinTools_FormatVersion_VERSION_4);
    TopoDS_Shape readback;BinTools::Read(readback,persisted);
    assert(saved_boolean_result::Inspect(readback,p,stop).classification==saved_boolean_result::Classification::MatchedOrientedBoundary);
    auto ligament=p;ligament.steps.back().operand.halfWidthMouth=15-.002;
    assert(!saved_boolean_result::detail::AdmitSections(ligament));
    ligament.steps.back().operand.halfWidthMouth=15-.003;
    assert(saved_boolean_result::detail::AdmitSections(ligament));
    // A transverse Y tool uses the same strip and integral with width/depth
    // exchanged, without interpreting the station axis as the tool axis.
    auto y=p;rectangular_loft::Definition yd;assert(loft_persistence::Decode(y.source.values,yd));
    for(auto& st:yd.stations)std::swap(st.width,st.depth);
    assert(loft_persistence::Encode(yd,y.source.values));
    for(auto& step:y.steps)step.operand.axis=analytic_boolean::Axis::Y;
    y.steps.back().operand.directionAngle=0;
    const auto ybase=base(saved_boolean_result::detail::GeometryView(y,0));
    assert(saved_boolean_build::Build(ybase,y,settings,stop).status==saved_boolean_build::Status::Built);
    coldReplayIdentity(ybase,y,settings,stop);
    // Rigid recipe frame translation and scale preserve the open strip.
    auto framed=p;rectangular_loft::Definition fd;assert(loft_persistence::Decode(framed.source.values,fd));
    profile::ConstructionFrame frame;frame.values={3,4,5,0,0,0,1,2};fd.constructionFrame=frame;
    assert(loft_persistence::Encode(fd,framed.source.values));
    for(auto& step:framed.steps){auto& t=step.operand;t.point={2*t.point[0]+3,2*t.point[1]+4,2*t.point[2]+5};
        if(t.kind==analytic_boolean::OperandKind::Cylinder)t.radius*=2;
        else {t.halfWidthApex*=2;t.halfWidthMouth*=2;t.length*=2;}}
    assert(saved_boolean_build::Build(base(saved_boolean_result::detail::GeometryView(framed,0)),framed,settings,stop).status==saved_boolean_build::Status::Built);
    coldReplayIdentity(base(saved_boolean_result::detail::GeometryView(framed,0)),framed,settings,stop);
    // A quarter-turn recipe frame maps the original X tool to local Z.
    // Admission must use the recipe frame rather than the enum's axis number.
    auto rotated=framed;const double q=std::sqrt(.5);frame.values={3,4,5,0,q,0,q,2};fd.constructionFrame=frame;
    assert(loft_persistence::Encode(fd,rotated.source.values));gp_Trsf rotation;assert(frame.Transform(rotation));
    for(unsigned k=0;k<p.steps.size();++k){const auto& originalTool=p.steps[k].operand;auto& t=rotated.steps[k].operand;
        const auto point=gp_Pnt(originalTool.point[0],originalTool.point[1],originalTool.point[2]).Transformed(rotation);
        t.point={point.X(),point.Y(),point.Z()};t.axis=analytic_boolean::Axis::Z;}
    rotated.steps.back().operand.directionAngle=0;
    const auto rotatedBuild=saved_boolean_build::Build(base(saved_boolean_result::detail::GeometryView(rotated,0)),rotated,settings,stop);
    if(rotatedBuild.status!=saved_boolean_build::Status::Built)std::cerr<<"recipe rotation: "<<rotatedBuild.phase<<":"<<rotatedBuild.correspondence.phase<<std::endl;
    assert(rotatedBuild.status==saved_boolean_build::Status::Built);
    coldReplayIdentity(base(saved_boolean_result::detail::GeometryView(rotated,0)),rotated,settings,stop);
    auto narrowed=p;rectangular_loft::Definition nd;assert(loft_persistence::Decode(narrowed.source.values,nd));
    nd.stations[1].depth=8;assert(loft_persistence::Encode(nd,narrowed.source.values));
    narrowed.steps.back().operand.length=75;
    const auto narrowedView=saved_boolean_result::detail::GeometryView(narrowed,narrowed.steps.back().operand);
    assert(analytic_boolean_wedge::TransverseBoundary(narrowedView,narrowed.steps.back().operand).status==analytic_boolean_wedge::Status::TransverseSideWall);
    auto overlap=p;overlap.steps.back().operand.length=90;
    assert(!saved_boolean_result::detail::AdmitSections(overlap));
    // Numeric environment changes cannot produce retained authority.
    assert(std::fesetround(FE_UPWARD)==0);assert(!saved_boolean_result::detail::AdmitSections(p));
    assert(std::fesetround(FE_TONEAREST)==0);
    // Retain existing axial open/closed slots and negative correspondence cases.
    for(unsigned scenario=0;scenario<4;++scenario)for(const auto& check:saved_boolean_wedge_probe::Run(scenario)){
        if(!check.second)std::cerr<<"axial regression: "<<check.first<<std::endl;assert(check.second);}
    stop.store(true);
    assert(saved_boolean_result::Inspect(cut.solid,p,stop).classification==saved_boolean_result::Classification::Cancelled);
    assert(saved_boolean_build::Build(original,p,settings,stop).status==saved_boolean_build::Status::Cancelled);
    stop.store(false);
    // Isolate the separate flush-apex rule on a supported axial box loft.
    auto axial=e;rectangular_loft::Definition d;assert(loft_persistence::Decode(axial.sourceValues,d));
    for(auto& s:d.stations){s.width=30;s.depth=30;}
    assert(loft_persistence::Encode(d,axial.sourceValues));axial.axis=2;
    auto slot=wedge;slot.axis=analytic_boolean::Axis::Z;slot.directionAngle=0;slot.point={-15,0,0};
    assert(saved_cut_whole_result::ExpectedSource(axial,expected));
    assert(analytic_boolean_wedge::ExpectedBoundary(axial,slot,expected).status==analytic_boolean_wedge::Status::UnsupportedConfiguration);
    slot.point[0]=-14;assert(saved_cut_whole_result::ExpectedSource(axial,expected));
    const auto inside=analytic_boolean_wedge::ExpectedBoundary(axial,slot,expected);
    assert(inside.status==analytic_boolean_wedge::Status::Clear&&inside.open);
    std::cout<<"PASS B05 complete census: "<<proof.faces<<" faces, "<<proof.edges<<" edges, "<<proof.vertices<<" vertices.\n";
    std::cout<<"PASS axis X preserved; exact B05 wedge removes "<<removed<<" mm^3; independent cutter and replay match.\n"
        <<"PASS cold native readback and replay: identical census, geometry/display streams and program bytes (X/Y, open/closed, translated/scaled/rotated).\n"
        <<"PASS complete transverse admission, census, exact representation, closed/open slots, wall refusal and native readback.\n";
}
