// Standalone retained macOS OCCT regression; no simulator or Xcode.
#include "PlanarSweepSolid.hxx"
#include "SweepPersistence.hxx"
#include "DetachedPlanarSweepProbe.hxx"
#include <BinXCAFDrivers.hxx>
#include <TDocStd_Application.hxx>
#include <sstream>
#include <iostream>
#include <stdexcept>
namespace s=core3d::planar_sweep;
namespace p=core3d::sweep_persistence;
static void require(bool ok,const char* message) { if(!ok)throw std::runtime_error(message); }
static s::SolidResult build(const s::Definition& d) {
    s::Admission admission;auto prepared=s::Prepare(d,admission);
    require(bool(prepared),"admission");std::atomic_bool cancelled{false};s::SolidResult result;
    const auto status=s::Build(prepared,cancelled,result);
    if(status!=s::BuildStatus::Built)std::cerr<<"radius="<<d.radius<<" end="<<d.EndRadius()<<" segments="<<d.segments.size()<<" unit="<<d.dimensionMetersPerUnit<<" "<<s::probe::Name(status)<<'\n';
    require(status==s::BuildStatus::Built,"build");return result;
}
static void persistenceRoundTrip() {
    Handle(TDocStd_Application) app=new TDocStd_Application;BinXCAFDrivers::DefineFormat(app);
    Handle(TDocStd_Document) doc;app->NewDocument(TCollection_ExtendedString("BinXCAF"),doc);
    XCAFDoc_DocumentTool::SetLengthUnit(doc,.001);doc->SetUndoLimit(10);
    const auto d=s::probe::Fixture(4,.001);const auto solid=build(d);
    const auto owner=XCAFDoc_DocumentTool::ShapeTool(doc->Main())->AddShape(solid.solid,Standard_False);
    const std::string id="A3FFCA1A-A4F9-421D-A2DD-C68C6565E060";
    doc->NewCommand();require(p::Stage(doc,owner,d,id),"stage v2");require(doc->CommitCommand(),"commit v2");
    p::Record record;require(p::Read(doc,owner,record)&&record.schema==2,"read v2");
    auto legacy=record.values;legacy.erase(legacy.begin()+5);
    doc->NewCommand();TDataStd_Integer::Set(record.label,p::SchemaID(),1);
    TDataStd_Integer::Set(record.label,p::CountID(),int(legacy.size()));
    for(TDF_ChildIterator it(record.label,Standard_False);it.More();it.Next())it.Value().ForgetAllAttributes(Standard_True);
    for(std::size_t i=0;i<legacy.size();++i)TDataStd_Real::Set(record.label.FindChild(int(i)+1,Standard_True),legacy[i]);
    require(doc->CommitCommand(),"commit legacy fixture");
    std::ostringstream saved(std::ios::binary);require(app->SaveAs(doc,saved)==PCDM_SS_OK,"save v1");
    app->Close(doc);doc.Nullify();std::istringstream input(saved.str(),std::ios::binary);
    require(app->Open(input,doc)==PCDM_RS_OK,"reopen v1");doc->SetUndoLimit(10);
    TDF_LabelSequence roots;XCAFDoc_DocumentTool::ShapeTool(doc->Main())->GetFreeShapes(roots);
    require(roots.Length()==1,"one owner");const auto reopenedOwner=roots.First();
    require(p::Read(doc,reopenedOwner,record)&&record.schema==1&&p::SameBits(record.values,legacy),"exact reopened v1");
    require(record.definition.EndRadius()==record.definition.radius,"v1 default radius");
    auto tapered=record.definition;tapered.endRadius=3;
    doc->NewCommand();require(p::Stage(doc,reopenedOwner,tapered,id),"stage v1 to v2");
    require(doc->CommitCommand(),"commit migration");p::Record upgraded;
    require(p::Read(doc,reopenedOwner,upgraded)&&upgraded.schema==2&&upgraded.values.size()==legacy.size()+1,"v2 growth");
    require(doc->Undo()&&p::Read(doc,reopenedOwner,record)&&record.schema==1&&p::SameBits(record.values,legacy),"undo exact v1");
    require(doc->Redo()&&p::Read(doc,reopenedOwner,record)&&record.IsEqual(upgraded),"redo v2");
    std::ostringstream finalSaved(std::ios::binary);require(app->SaveAs(doc,finalSaved)==PCDM_SS_OK,"save v2");
    app->Close(doc);doc.Nullify();std::istringstream finalInput(finalSaved.str(),std::ios::binary);
    require(app->Open(finalInput,doc)==PCDM_RS_OK,"reopen v2");roots.Clear();
    XCAFDoc_DocumentTool::ShapeTool(doc->Main())->GetFreeShapes(roots);
    require(roots.Length()==1&&p::Read(doc,roots.First(),record)&&record.schema==2
        &&p::SameBits(record.values,upgraded.values),"exact reopened v2");app->Close(doc);
}
int main() {
    try {
        for(double unit:{.001,1.0})for(int plane=0;plane<3;++plane) {
            auto d=s::probe::Fixture(4,unit,plane);const auto legacy=build(d);
            d.endRadius=d.radius;const auto explicitEqual=build(d);
            require(legacy.volume==explicitEqual.volume && legacy.bounds==explicitEqual.bounds,"constant builder regression");
            for(double ratio:{0.5,2.0}) {
                d.endRadius=d.radius*ratio;const auto taper=build(d);
                s::Inspection inspected;require(s::Inspect(d,inspected)==s::Admission::Accepted,"inspect taper");
                const double expected=std::acos(-1.0)*inspected.length*(d.radius*d.radius+d.radius*d.EndRadius()+d.EndRadius()*d.EndRadius())/3;
                require(std::abs(taper.volume-expected)<=expected*1e-6,"analytic frustum volume");
                std::vector<double> values,again;s::Definition decoded;
                require(p::Encode(d,values)&&values.size()==25&&p::Decode(values,decoded),"v2 codec");
                require(decoded.EndRadius()==d.EndRadius()&&p::Encode(decoded,again)&&p::SameBits(values,again),"v2 exact bits");
            }
            d.endRadius=d.radius;std::vector<double> values,again;s::Definition decoded;
            require(p::Encode(d,values),"legacy encode fixture");values.erase(values.begin()+5);
            require(p::Decode(values,decoded,1)&&decoded.EndRadius()==decoded.radius,"v1 endpoint default");
            require(p::Decode(values,decoded)&&decoded.EndRadius()==decoded.radius,"v1 automatic numeric reopen");
            require(p::Encode(decoded,again)&&again.size()==values.size()+1,"v1 upgrade");
            again.erase(again.begin()+5);require(p::SameBits(values,again),"v1 exact bits");
            require(!p::Decode(values,decoded,2)&&!p::Decode(values,decoded,3),"schema mismatch");
            for(double bad:{0.0,-1.0,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
                d.endRadius=bad;s::Inspection inspected;
                require(s::Inspect(d,inspected)==s::Admission::InvalidNumber,"invalid endpoint");
            }
        }
        auto curved=s::probe::Fixture(0,.001);curved.endRadius=21;s::Inspection inspected;
        require(s::Inspect(curved,inspected)==s::Admission::TightBend,"larger endpoint bend clearance");
        curved.endRadius=3;build(curved);
        auto close=s::probe::Fixture(4,.001);
        close.vertices={{100,{0,0}},{101,{20,0}},{102,{40,0}},{103,{60,0}}};
        close.segments={{200,100,101,core3d::ProfileCurveKind::Line,{},0,0,0},
            {201,101,102,core3d::ProfileCurveKind::Line,{},0,0,0},
            {202,102,103,core3d::ProfileCurveKind::Line,{},0,0,0}};
        require(s::Inspect(close,inspected)==s::Admission::Accepted,"constant clearance control");
        close.endRadius=11;
        require(s::Inspect(close,inspected)==s::Admission::NonlocalContact,"larger endpoint nonlocal clearance");
        persistenceRoundTrip();
        std::cout<<"PASS OCAF save/reopen/migrate/undo, tapered frustum, curved law, legacy builder, v1/v2 codec and endpoint admission\n";
    }catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}
}
