#pragma once
#if DEBUG
#include "SweepPersistence.hxx"
#include "DetachedPlanarSweepProbe.hxx"
#include "OcctDocument.h"
#include "Core3DBoundedAuthoredFrameDriver.hxx"
#include <BRepPrimAPI_MakeBox.hxx>
#include <PCDM_StoreStatus.hxx>
#include <PCDM_ReaderStatus.hxx>
#include <sstream>
#include <map>

namespace core3d::sweep_persistence::probe {
using Checks=std::map<std::string,bool>;
inline const std::string Identifier="A3FFCA1A-A4F9-421D-A2DD-C68C6565E060";
struct RawDocument {
    Handle(TDocStd_Application) app=new TDocStd_Application();
    Handle(TDocStd_Document) document;
    TDF_Label owner;
    explicit RawDocument(double unit=0.001) {
        // Actual approved scalar drivers, deliberately no final production owner gate.
        Core3DDebugDefineFrameBinXCAFFormat(app,std::make_shared<persistence::AuthoredFrameReadBudget>());
        app->NewDocument(TCollection_ExtendedString("BinXCAF"),document);
        XCAFDoc_DocumentTool::SetLengthUnit(document,unit); document->SetUndoLimit(10);
        // A fixed BRep is sufficient to test record binding. This is NOT a sweep geometry receipt.
        owner=XCAFDoc_DocumentTool::ShapeTool(document->Main())->AddShape(BRepPrimAPI_MakeBox(10,10,10).Shape(),Standard_False);
    }
    ~RawDocument() noexcept { try { if (!document.IsNull()) app->Close(document); } catch (...) {} }
};
inline Definition Fixture(int kind,double unit=0.001) {
    auto d=planar_sweep::probe::Fixture(kind,unit);
    d.vertices.front().point.SetX(-0.0);
    return d;
}
inline Checks Numeric() {
    Checks c; Definition decoded; std::vector<double> v;
    auto d=Fixture(4); c["minimum24"]=Encode(d,v)&&v.size()==24&&Decode(v,decoded);
    if (!c["minimum24"]) return c; // A failed positive fixture must not index empty output.
    c["signedZero"]=Bits(decoded.vertices.front().point.X())==Bits(-0.0);
    const auto rejected=[&](std::vector<double> candidate) { Definition out=Fixture(0); const bool ok=Decode(candidate,out); return !ok&&out.vertices.empty()&&out.pathIdentifier==0; };
    auto rejectAt=[&](const char* name,std::size_t index,double value) { auto a=v;a[index]=value;c[name]=rejected(a); };
    rejectAt("unknownSection",1,1);rejectAt("unknownPolicy",2,1);rejectAt("fractionalID",3,1.5);
    rejectAt("overflowID",3,4294967296.0);rejectAt("zeroID",3,0);rejectAt("nan",4,std::numeric_limits<double>::quiet_NaN());
    rejectAt("infiniteUnit",5,std::numeric_limits<double>::infinity());rejectAt("negativeUnit",5,-0.001);
    rejectAt("oversizedVertices",6,34);rejectAt("mismatchedCounts",7,2);rejectAt("invalidFrameFlag",8,2);
    rejectAt("duplicateID",9,1);rejectAt("wrongConnectivity",17,999);rejectAt("hiddenLineArc",21,2);
    auto shortValues=v;shortValues.pop_back();c["truncated"]=rejected(shortValues);
    auto longValues=v;longValues.push_back(0);c["trailing"]=rejected(longValues);
    c["over404"]=rejected(std::vector<double>(405,0));
    d.vertices.clear();d.segments.clear();d.radius=1;
    for (int i=0;i<33;++i) d.vertices.push_back({ProfileCurveID(100+i),gp_Pnt2d(i*20,0)});
    for (int i=0;i<32;++i) d.segments.push_back({ProfileCurveID(200+i),ProfileCurveID(100+i),ProfileCurveID(101+i),ProfileCurveKind::Line,{},0,0,0});
    profile::ConstructionFrame frame; frame.values={-0.0,2,3,-0.0,-0.0,-std::sqrt(0.5),-std::sqrt(0.5),-2};d.constructionFrame=frame;
    c["maximum404"]=Encode(d,v)&&v.size()==404&&Decode(v,decoded);
    if (!c["maximum404"]) return c; // Preserve a safe test failure before fixed-index checks.
    std::vector<double> again;c["frameBits"]=Encode(decoded,again)&&SameBits(v,again);
    auto equivalent=v;equivalent[9+3*33+9*32]=0;
    c["bitComparisonDistinguishesSignedZero"]=!SameBits(v,equivalent);
    return c;
}
inline Checks RoundTrip() {
    Checks c;
    for (int kind=0;kind<2;++kind) for (double unit:{0.001,1.0}) for (int framed=0;framed<2;++framed) {
        const auto key=std::to_string(kind)+":"+std::to_string(unit)+":"+std::to_string(framed);
        RawDocument source(unit),reader(unit);auto d=Fixture(kind,unit);
        if (framed) {profile::ConstructionFrame f;f.values={-0.0,2,3,-0.0,0,-std::sqrt(0.5),-std::sqrt(0.5),-2};d.constructionFrame=f;}
        source.document->NewCommand();
        if (!Stage(source.document,source.owner,d,Identifier)) {c[key]=false;source.document->AbortCommand();continue;}
        source.document->CommitCommand();Record original;
        bool ok=Read(source.document,source.owner,original)&&original.IsCurrent(source.document,source.owner);
        std::ostringstream output(std::ios::binary|std::ios::out);
        ok=ok&&source.app->SaveAs(source.document,output)==PCDM_SS_OK;
        const auto binary=output.str();ok=ok&&!binary.empty()&&binary.size()<=1024*1024;
        reader.app->Close(reader.document);reader.document.Nullify();
        Core3DBeginSafeBinaryRead();std::istringstream input(binary,std::ios::binary|std::ios::in);
        ok=ok&&reader.app->Open(input,reader.document)==PCDM_RS_OK&&!Core3DSafeBinaryReadWasRejected()&&!reader.document.IsNull();
        if (ok) {
            TDF_LabelSequence roots;XCAFDoc_DocumentTool::ShapeTool(reader.document->Main())->GetFreeShapes(roots);
            ok=roots.Length()==1;
            if (ok) {Record reopened;reader.owner=roots.First();ok=Read(reader.document,reader.owner,reopened)
                &&reopened.IsCurrent(reader.document,reader.owner)&&SameBits(original.values,reopened.values)
                &&reopened.identifier==Identifier&&!original.IsEqual(reopened)
                &&!original.IsCurrent(reader.document,reader.owner);}
        }
        c[key]=ok;
    }
    return c;
}
inline Checks Malformed() {
    Checks c;
    for (int mode=0;mode<17;++mode) {
        RawDocument raw;auto d=Fixture(0);raw.document->NewCommand();
        if (!Stage(raw.document,raw.owner,d,Identifier)) {c[std::to_string(mode)]=false;continue;}
        Record record;if (!Read(raw.document,raw.owner,record)) {c[std::to_string(mode)]=false;continue;}
        const auto label=record.label;const int count=int(record.values.size());
        switch (mode) {
            case 0:label.ForgetAttribute(SchemaID());break;
            case 1:TDataStd_Integer::Set(label,SchemaID(),2);break;
            case 2:TDataStd_Integer::Set(label,CountID(),405);break;
            case 3:TDataStd_Integer::Set(label,CountID(),23);break;
            case 4:label.FindChild(1).ForgetAllAttributes();break;
            case 5:TDataStd_Real::Set(label.FindChild(count+1,Standard_True),0);break;
            case 6:TDataStd_Real::Set(label.FindChild(1).FindChild(1,Standard_True),0);break;
            case 7:TDataStd_Integer::Set(label.FindChild(1),42);break;
            case 8:TDataStd_Real::Set(label.FindChild(1),std::numeric_limits<double>::infinity());break;
            case 9:TDataStd_AsciiString::Set(label,IdentityID(),TCollection_AsciiString("a3ffca1a-a4f9-421d-a2dd-c68c6565e060"));break;
            case 10:TDataStd_Integer::Set(raw.owner.FindChild(label.Tag()+1,Standard_True),SchemaID(),1);break;
            case 11:label.ForgetAttribute(TNaming_NamedShape::GetID());break;
            case 12:TDataStd_Real::Set(label,1);break;
            case 13:TDataStd_Integer::Set(raw.owner.FindChild(label.Tag()+1,Standard_True),profile::SchemaID(),1);break;
            case 14:TDataStd_Integer::Set(raw.owner.FindChild(label.Tag()+1,Standard_True),enclosure::SchemaID(),1);break;
            case 15:TDataStd_Real::Set(label.FindChild(4),1.25);break;
            case 16:TDataStd_AsciiString::Set(label,IdentityID(),TCollection_AsciiString("not-a-uuid"));break;
        }
        Record result;c[std::to_string(mode)]=!Read(raw.document,raw.owner,result)&&result.label.IsNull();
        raw.document->AbortCommand();
        Record absent;c["abort:"+std::to_string(mode)]=Read(raw.document,raw.owner,absent)&&absent.label.IsNull();
    }
    return c;
}
inline Checks BindingAndTransaction() {
    Checks c;RawDocument raw,foreign;auto d=Fixture(0);Record original;
    c["absent"]=Read(raw.document,raw.owner,original)&&original.label.IsNull();
    c["noImplicitCommand"]=!Stage(raw.document,raw.owner,d,Identifier)&&!raw.document->HasOpenCommand();
    raw.document->NewCommand();c["stage"]=Stage(raw.document,raw.owner,d,Identifier)&&Read(raw.document,raw.owner,original);
    c["commandStillOwned"]=raw.document->HasOpenCommand();raw.document->CommitCommand();
    Record check;c["committed"]=Read(raw.document,raw.owner,check)&&check.IsEqual(original);
    c["foreignDocument"]=!Read(foreign.document,raw.owner,check)&&!original.IsCurrent(foreign.document,raw.owner);
    c["foreignOwner"]=!Read(raw.document,foreign.owner,check)&&!original.IsCurrent(raw.document,foreign.owner);
    raw.document->NewCommand();
    c["cannotReplaceIdentity"]=!Stage(raw.document,raw.owner,d,"48AA1121-D28D-4F92-B218-602253859AFA");
    auto wrong=d;wrong.dimensionMetersPerUnit=1;c["wrongUnitStage"]=!Stage(raw.document,raw.owner,wrong,Identifier);
    c["unchangedAfterRefusal"]=Read(raw.document,raw.owner,check)&&check.IsEqual(original);
    // Empty allocation remains inert, including after abort.
    original.label.FindChild(1000,Standard_True).FindChild(2,Standard_True);
    c["emptyLabels"]=Read(raw.document,raw.owner,check)&&check.IsEqual(original);raw.document->AbortCommand();
    raw.document->NewCommand();XCAFDoc_DocumentTool::SetLengthUnit(raw.document,1);
    c["wrongRawDocumentUnit"]=!Read(raw.document,raw.owner,check)&&!original.IsCurrent(raw.document,raw.owner);raw.document->AbortCommand();
    raw.document->NewCommand();XCAFDoc_DocumentTool::ShapeTool(raw.document->Main())->SetShape(raw.owner,BRepPrimAPI_MakeBox(11,10,10).Shape());
    c["staleShapeBinding"]=!Read(raw.document,raw.owner,check)&&!original.IsCurrent(raw.document,raw.owner);raw.document->AbortCommand();
    c["abortedChangesRestoreExactRecord"]=Read(raw.document,raw.owner,check)&&check.IsEqual(original);
    c["rawUndo"]=raw.document->Undo()&&Read(raw.document,raw.owner,check)&&check.label.IsNull();
    c["rawRedo"]=raw.document->Redo()&&Read(raw.document,raw.owner,check)&&check.IsEqual(original);
    return c;
}
} // namespace core3d::sweep_persistence::probe
#endif
