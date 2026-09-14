#pragma once
#if DEBUG
#include "SavedBooleanProgramBuild.hxx"
#include "SavedCutWholeResultCorrespondenceProbe.hxx"
#include "RetainedSolidProbe.hxx"
namespace core3d::saved_boolean_build::probe {
namespace fixture=saved_cut_bore_clearance::probe;
namespace geometry=saved_cut_bore_result::probe;
namespace oldProbe=saved_cut_whole_result::probe;
struct Evidence {std::map<std::string,bool> checks;std::map<std::string,std::string> phases;};
inline std::vector<std::uint8_t> Bytes(const retained_boolean::Recipe& value){
    std::vector<std::uint8_t> out;if(!retained_boolean::Encode(value,out))return {};return out;
}
inline bool Volume(const TopoDS_Shape& shape,double mm,double expected){
    if(shape.IsNull())return false;GProp_GProps p;BRepGProp::VolumeProperties(shape,p,Standard_True,Standard_False,Standard_False);
    const double actual=p.Mass()*mm*mm*mm;return std::isfinite(actual)&&std::abs(actual-expected)<std::max(1e-6,std::abs(expected)*1e-9);
}
inline void Carrier(const retained_solid::Envelope& legacy,const Program& program,const TopoDS_Shape& base,
    const TopoDS_Shape& one,const TopoDS_Shape& two,const std::atomic_bool& stop,Evidence& out,const std::string& prefix){
    using Native=retained_solid::Probe;
    for(bool xcaf:{false,true})for(int version:{10,11,12}){
        const auto tag=prefix+(xcaf?"xcaf.":"ocaf.")+std::to_string(version)+".";
        for(const char* key:{"legacy-paired-fixture","migration-paired-command","undo-restores-v1","redo-restores-v2","fresh-native-reopen","fresh-complete-program","legacy-envelope-reader-refuses"})out.checks[tag+key]=false;
        try {
            Native::App app;const auto fixture=Native::New(app,xcaf,version,legacy.metersPerUnit);const auto doc=app.doc;
            const auto shapes=XCAFDoc_DocumentTool::ShapeTool(doc->Main());
            doc->NewCommand();shapes->SetShape(fixture.owner,one);TNaming_Builder(fixture.record).Select(one,one);
            const auto attribute=Native::Install(doc,fixture.record,retained_boolean::Recipe(legacy),base);
            out.checks[tag+"legacy-paired-fixture"]=Core3DValidateRetainedSolidDocument(doc)&&doc->CommitCommand();
            doc->ClearUndos();const auto first=attribute->value();
            doc->NewCommand();shapes->SetShape(fixture.owner,two);TNaming_Builder(fixture.record).Select(two,two);
            Native::Install(doc,fixture.record,retained_boolean::Recipe(program),base);
            const auto expected=Bytes(retained_boolean::Recipe(program));
            out.checks[tag+"migration-paired-command"]=Core3DValidateRetainedSolidDocument(doc)&&doc->CommitCommand()
                &&attribute->value()->bytes==expected&&attribute->value()->base.IsEqual(base);
            out.checks[tag+"undo-restores-v1"]=doc->Undo()&&attribute->value()==first
                &&std::holds_alternative<retained_solid::Envelope>(attribute->value()->envelope)
                &&XCAFDoc_ShapeTool::GetShape(fixture.owner).IsEqual(one);
            out.checks[tag+"redo-restores-v2"]=doc->Redo()&&attribute->value()->bytes==expected
                &&std::holds_alternative<Program>(attribute->value()->envelope)
                &&XCAFDoc_ShapeTool::GetShape(fixture.owner).IsEqual(two);
            const auto bytes=Native::Save(app);std::vector<retained_solid::Record> records;double unit=0;
            const bool reopened=Native::Open(bytes,false,&records,retained_solid::MaximumAggregateEnvelopeBytes,false,0,nullptr,&unit);
            out.checks[tag+"fresh-native-reopen"]=reopened&&records.size()==1&&retained_solid::Bits(unit)==retained_solid::Bits(legacy.metersPerUnit)
                &&records[0].value->bytes==expected&&std::holds_alternative<Program>(records[0].value->envelope);
            if(reopened&&records.size()==1){const auto& actual=std::get<Program>(records[0].value->envelope);
                out.checks[tag+"fresh-complete-program"]=saved_boolean_result::Inspect(records[0].current,actual,stop).classification
                    ==saved_boolean_result::Classification::MatchedOrientedBoundary
                    &&saved_cut_source_edit::InspectBase(records[0].value->base,saved_boolean_result::detail::GeometryView(actual,0),stop);
            }
            retained_solid::Envelope refused;
            out.checks[tag+"legacy-envelope-reader-refuses"]=!retained_solid::Decode(expected,refused);
        }catch(...){out.phases[tag+"exception"]="native-carrier-probe-exception";}
    }
}
inline Evidence Run(){
    Evidence evidence;auto stop=std::make_shared<std::atomic_bool>(false);
    Handle(Prs3d_Drawer) drawer=new Prs3d_Drawer();cut_display::Settings settings;
    evidence.checks["actual-drawer-capture"]=cut_display::Capture(drawer,settings);
    for(double unit:{.001,1.0})for(bool enclosure:{false,true}){
        const std::string tag=std::string(enclosure?"enclosure":"bracket")+(unit==.001?".mm.":".metre.");const double mm=unit*1000,pi=std::acos(-1.0);
        for(const char* key:{"legacy-byte-preservation","legacy-noop-remains-v1","append-two-stable-identities","migration-preserves-original-tool","v2-exact-roundtrip-old-reader-refusal","reserved-byte-corruption-refuses","duplicate-highwater-limit-refusals","actual-two-bore-construction","independent-original-volume","retained-original-stream-unchanged","single-hole-cannot-certify-two","independent-radius-edits","radius-edits-preserve-all-other-bytes","shrinking-rebuilds-original-base","source-patch-maps-complete-program","source-patch-preserves-both-tools","actual-source-and-both-bores-rebuild","actual-malformed-cap-refuses","coincident-tool-refusal","cancelled-output-cleared","first-radius-leaves-second-exact","second-radius-leaves-first-exact"})evidence.checks[tag+key]=false;
        auto original=enclosure?fixture::box(unit):fixture::bracket(unit);original.radius=(enclosure?3:2)/mm;
        original.point[original.axis]=-0.0;
        const retained_boolean::Recipe legacy=original;const auto originalBytes=Bytes(legacy);
        retained_boolean::Recipe decoded;std::vector<std::uint8_t> legacyDirect;
        evidence.checks[tag+"legacy-byte-preservation"]=retained_solid::Encode(original,legacyDirect)&&originalBytes==legacyDirect
            &&retained_boolean::Decode(originalBytes,decoded)&&std::holds_alternative<retained_boolean::Legacy>(decoded)&&Bytes(decoded)==originalBytes;
        const auto unchanged=retained_boolean::Radius(legacy,original.operandID,enclosure?3:2,mm);
        evidence.checks[tag+"legacy-noop-remains-v1"]=unchanged&&!unchanged->changed&&unchanged->newBytes==originalBytes
            &&std::holds_alternative<retained_boolean::Legacy>(unchanged->recipe);
        cylindrical_cut::CreateEdit create;create.axis=analytic_boolean::Axis(original.axis);
        create.localCenter=enclosure?std::array<double,3>{80/mm,30/mm,-0.0}:std::array<double,3>{50/mm,-0.0,4/mm};create.worldRadiusMM=enclosure?3:2;
        const auto appended=retained_boolean::Append(legacy,create,mm);
        evidence.checks[tag+"append-two-stable-identities"]=appended&&appended->changed&&appended->selectedOperandID==2;
        if(!appended)continue;const auto program=std::get<Program>(appended->recipe);
        evidence.checks[tag+"migration-preserves-original-tool"]=program.steps.size()==2&&program.nextOperandID==3
            &&fixture::same(saved_boolean_result::detail::GeometryView(program,0),original);
        retained_boolean::Program reopened;
        evidence.checks[tag+"v2-exact-roundtrip-old-reader-refusal"]=retained_boolean::Decode(appended->newBytes,reopened)
            &&Bytes(retained_boolean::Recipe(reopened))==appended->newBytes&&!retained_solid::Decode(appended->newBytes,original);
        // Decode's failure clears its output: restore immutable source fixture.
        original=std::get<retained_boolean::Legacy>(legacy);
        auto badBytes=appended->newBytes;badBytes[9]=1;retained_boolean::Program bad;
        evidence.checks[tag+"reserved-byte-corruption-refuses"]=!retained_boolean::Decode(badBytes,bad)&&bad.steps.empty();
        auto duplicate=program;duplicate.steps[1].operand.identifier=duplicate.steps[0].operand.identifier;
        auto exhausted=original;exhausted.operandID=UINT32_MAX;
        // The value layer now admits third and fourth appends; only the FIFTH
        // append hits the operand cap and refuses before any mutation. Value
        // appends carry no geometry authority, so coincident positions are
        // admitted here exactly as any out-of-clearance values would be.
        const auto third=retained_boolean::Append(appended->recipe,create,mm);
        const auto fourth=third?retained_boolean::Append(third->recipe,create,mm):std::optional<retained_boolean::Change>();
        evidence.checks[tag+"duplicate-highwater-limit-refusals"]=!retained_boolean::Valid(duplicate)
            &&third&&third->changed&&fourth&&fourth->changed
            &&std::get<Program>(fourth->recipe).steps.size()==4
            &&std::get<Program>(fourth->recipe).nextOperandID==5
            &&!retained_boolean::Append(fourth->recipe,create,mm)
            &&!retained_boolean::Append(retained_boolean::Recipe(exhausted),create,mm)
            &&!retained_boolean::Radius(appended->recipe,99,1,mm);
        bool baseMatches=false;const auto base=geometry::Base(original,stop,baseMatches);const auto before=geometry::Bytes(base);
        const auto built=Build(base,program,settings,*stop);evidence.phases[tag+"build"]=std::string(built.phase)+":"+built.correspondence.phase;
        evidence.checks[tag+"actual-two-bore-construction"]=baseMatches&&built.status==Status::Built;
        const double enclosureBase=(100*60-(4-pi)*16)*30-(96*56-(4-pi)*4)*28;
        const double expected=enclosure?enclosureBase-36*pi:6528-64*pi;
        evidence.checks[tag+"independent-original-volume"]=built.status==Status::Built&&Volume(built.solid,mm,expected);
        evidence.checks[tag+"retained-original-stream-unchanged"]=!before.empty()&&geometry::Bytes(base)==before;
        if(built.status!=Status::Built)continue;
        // One real hole is insufficient even when its own local observation passes.
        analytic_boolean::Result one;const bool oneBuilt=analytic_boolean::Build(base,cylindrical_cut::Recipe(original),*stop,one)==analytic_boolean::Status::Built;
        evidence.checks[tag+"single-hole-cannot-certify-two"]=oneBuilt
            &&saved_cut_bore_result::Inspect(one.solid,original,original,*stop).status==saved_cut_bore_result::Status::BoreWallObservedExteriorUnproven
            &&saved_boolean_result::Inspect(one.solid,program,*stop).classification==saved_boolean_result::Classification::Refused;
        if(oneBuilt)Carrier(original,program,base,one.solid,built.solid,*stop,evidence,tag);
        const auto first=retained_boolean::Radius(appended->recipe,1,enclosure?2:1,mm);
        const auto second=first?retained_boolean::Radius(first->recipe,2,enclosure?4:3,mm):std::optional<retained_boolean::Change>();
        evidence.checks[tag+"independent-radius-edits"]=first&&second&&first->changed&&second->changed;
        if(!second)continue;const auto edited=std::get<Program>(second->recipe);
        auto firstRestored=std::get<Program>(first->recipe);const auto firstRadius=firstRestored.steps[0].operand.radius;
        firstRestored.steps[0]=program.steps[0];
        evidence.checks[tag+"first-radius-leaves-second-exact"]=first->selectedOperandID==1
            &&retained_solid::Bits(firstRadius)==retained_solid::Bits((enclosure?2.0:1.0)/mm)
            &&Bytes(retained_boolean::Recipe(firstRestored))==appended->newBytes;
        auto secondRestored=edited;const auto secondRadius=secondRestored.steps[1].operand.radius;
        secondRestored.steps[1]=std::get<Program>(first->recipe).steps[1];
        evidence.checks[tag+"second-radius-leaves-first-exact"]=second->selectedOperandID==2
            &&retained_solid::Bits(secondRadius)==retained_solid::Bits((enclosure?4.0:3.0)/mm)
            &&Bytes(retained_boolean::Recipe(secondRestored))==first->newBytes;
        auto restore=edited;restore.steps[0]=program.steps[0];restore.steps[1]=program.steps[1];
        evidence.checks[tag+"radius-edits-preserve-all-other-bytes"]=Bytes(retained_boolean::Recipe(restore))==appended->newBytes;
        const auto resized=Build(base,edited,settings,*stop);
        evidence.checks[tag+"shrinking-rebuilds-original-base"]=resized.status==Status::Built
            &&Volume(resized.solid,mm,enclosure?enclosureBase-40*pi:6528-80*pi)&&geometry::Bytes(base)==before;
        saved_cut_source_values::Patch sourcePatch;
        if(enclosure){saved_cut_source_values::EnclosurePatch p;p.dimensions[0]=110/mm;sourcePatch=p;}
        else {saved_cut_source_values::PolygonPatch p;p.depth=9/mm;p.coordinates={{1,saved_cut_source_values::Component::U,65/mm},{2,saved_cut_source_values::Component::U,65/mm}};sourcePatch=p;}
        const auto sourceChanged=SourcePatch(edited,sourcePatch);evidence.checks[tag+"source-patch-maps-complete-program"]=sourceChanged&&sourceChanged->changed;
        if(!sourceChanged)continue;const auto changed=std::get<Program>(sourceChanged->recipe);
        auto restoreSource=changed;restoreSource.source.values=edited.source.values;
        evidence.checks[tag+"source-patch-preserves-both-tools"]=Bytes(retained_boolean::Recipe(restoreSource))==second->newBytes;
        bool newBaseMatches=false;const auto newView=saved_boolean_result::detail::GeometryView(changed,0);
        const auto newBase=geometry::Base(newView,stop,newBaseMatches);const auto newBefore=geometry::Bytes(newBase);
        const auto grown=Build(newBase,changed,settings,*stop);evidence.phases[tag+"source-rebuild"]=std::string(grown.phase)+":"+grown.correspondence.phase;
        const double newEnclosureBase=(110*60-(4-pi)*16)*30-(106*56-(4-pi)*4)*28;
        evidence.checks[tag+"actual-source-and-both-bores-rebuild"]=newBaseMatches&&grown.status==Status::Built
            &&Volume(grown.solid,mm,enclosure?newEnclosureBase-40*pi:7704-90*pi)&&geometry::Bytes(newBase)==newBefore;
        const auto originalResult=geometry::Bytes(built.solid);auto corrupted=oldProbe::Read(originalResult);
        const bool capChanged=oldProbe::ShiftCap(corrupted,program.steps[0].operand.radius,mm);
        evidence.checks[tag+"actual-malformed-cap-refuses"]=capChanged
            &&saved_boolean_result::Inspect(corrupted,program,*stop).classification==saved_boolean_result::Classification::Refused
            &&geometry::Bytes(built.solid)==originalResult;
        auto touching=program;touching.steps[1].operand.point=touching.steps[0].operand.point;
        evidence.checks[tag+"coincident-tool-refusal"]=!saved_boolean_result::detail::SeparateDisks(touching)
            &&Build(base,touching,settings,*stop).status==Status::Refused;
        stop->store(true);const auto stopped=Build(base,program,settings,*stop);stop->store(false);
        evidence.checks[tag+"cancelled-output-cleared"]=stopped.status==Status::Cancelled&&stopped.solid.IsNull()&&stopped.exactProgram.empty();
    }return evidence;
}
} // namespace core3d::saved_boolean_build::probe
#endif
