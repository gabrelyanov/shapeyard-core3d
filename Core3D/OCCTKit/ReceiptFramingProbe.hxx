#pragma once
#if DEBUG
#include "ReceiptFramedTraversal.hxx"
#include <BinDrivers_DocumentStorageDriver.hxx>
#include <BinXCAFDrivers_DocumentStorageDriver.hxx>
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_TreeNode.hxx>
#include <XCAFDoc_Location.hxx>
#include <TNaming_Builder.hxx>
#include <TNaming_Tool.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepGProp.hxx>
#include <BRepBndLib.hxx>
#include <GProp_GProps.hxx>
#include <Bnd_Box.hxx>
#include <gp_Trsf.hxx>
#include <TopLoc_Location.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Message.hxx>
#include <TDF_Data.hxx>
#include <TColStd_SequenceOfAsciiString.hxx>
#include <PCDM_StoreStatus.hxx>
#include <map>
#include <cmath>
#include <vector>
#include <cstdio>

namespace core3d::debug::receipt_framing_probe {
namespace frame = core3d::persistence::receipt_framing;
// Fixed, bounded DEBUG phase labels only; no document or credential contents.
inline thread_local bool TraceEnabled=false;
inline thread_local int TraceFormat=0,TraceVersion=0;
inline void Trace(const char* phase,int detail=0) noexcept {
    if(TraceEnabled){std::fprintf(stderr,"[receipt-framing] format=%d version=%d phase=%s detail=%d\n",
        TraceFormat,TraceVersion,phase,detail);std::fflush(stderr);}
}
class ProbeAttribute : public TDF_Attribute {
public:
    DEFINE_STANDARD_RTTIEXT(ProbeAttribute,TDF_Attribute)
    static const Standard_GUID& Guid(){static const Standard_GUID g("52ab2b86-e38d-487a-9b26-83fb3b183c43");return g;}
    const Standard_GUID& ID() const override{return Guid();}
    Handle(TDF_Attribute) NewEmpty() const override{return new ProbeAttribute();}
    void Restore(const Handle(TDF_Attribute)& a) override {
        const auto p=Handle(ProbeAttribute)::DownCast(a);valid=!p.IsNull()&&p->valid;
    }
    void Paste(const Handle(TDF_Attribute)& a,const Handle(TDF_RelocationTable)&) const override{
        const auto p=Handle(ProbeAttribute)::DownCast(a);if(p.IsNull())Standard_Failure::Raise("Probe target type");p->Backup();p->valid=valid;
    }
    bool valid=false;
};
IMPLEMENT_STANDARD_RTTIEXT(ProbeAttribute,TDF_Attribute)
struct State {
    std::shared_ptr<frame::LoadBudget> budget=std::make_shared<frame::LoadBudget>();
    int pasteCalls=0;
    bool allowDirect=true,wrongTarget=false;
    int decodeFault=0;
    std::function<void()> afterPaste;
};
class Driver final : public frame::FrameDriver {
public:
    explicit Driver(std::shared_ptr<State> state)
        : FrameDriver(Message::DefaultMessenger(),"Core3D_ReceiptFramingProbe",state->budget),state_(std::move(state)){}
    const Standard_GUID& AttributeID() const override{return ProbeAttribute::Guid();}
    const Handle(Standard_Type)& SourceType() const override{return STANDARD_TYPE(ProbeAttribute);}
    Handle(TDF_Attribute) NewEmpty() const override {
        if(state_->wrongTarget)return new TDataStd_Integer();return new ProbeAttribute();
    }
    Standard_Boolean Paste(const BinObjMgt_Persistent& source,const Handle(TDF_Attribute)& target,
                           BinObjMgt_RRelocationTable&) const override {
        ++state_->pasteCalls;
        const auto p=Handle(ProbeAttribute)::DownCast(target);if(p.IsNull())return false;
        auto* stream=const_cast<BinObjMgt_Persistent&>(source).GetIStream();
        if(!stream)return false;
        if(state_->decodeFault==1){char b[9];stream->read(b,9);stream->clear();return true;}
        if(state_->decodeFault==2)return true;
        if(state_->decodeFault==3){stream->seekg(-1,std::ios::cur);stream->clear();return true;}
        if(state_->decodeFault==4)Standard_Failure::Raise("Private D1 decoder fault");
        char body[8]{};stream->read(body,8);
        if(!*stream || std::string(body,8)!="SYD1BODY")return false;
        p->valid=true;if(state_->afterPaste)state_->afterPaste();return true;
    }
    void Paste(const Handle(TDF_Attribute)& source,BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable&) const override {
        const auto p=Handle(ProbeAttribute)::DownCast(source);
        if(p.IsNull() || !p->valid || !state_->allowDirect)Standard_Failure::Raise("Private D1 writer refused");
        auto* stream=target.GetOStream();if(!stream)Standard_Failure::Raise("No direct stream");
        stream->write("SYD1BODY",8);
    }
private:
    const std::shared_ptr<State> state_;
};
template<class Base> class Writer final : public Base {
public:
    explicit Writer(std::shared_ptr<State> state):state_(std::move(state)),driver_(new Driver(state_)){}
    Handle(BinMDF_ADriverTable) AttributeDrivers(const Handle(Message_Messenger)& m) override{
        auto table=Base::AttributeDrivers(m);table->AddDriver(driver_);return table;
    }
    void Write(const Handle(CDM_Document)& doc,Standard_OStream& stream,const Message_ProgressRange& r) override{
        Prepare(doc);Base::Write(doc,stream,r);
    }
    void Write(const Handle(CDM_Document)& doc,const TCollection_ExtendedString& path,const Message_ProgressRange& r) override{
        Prepare(doc);Base::Write(doc,path,r);
    }
private:
    void Prepare(const Handle(CDM_Document)& doc){
        const auto d=Handle(TDocStd_Document)::DownCast(doc);
        state_->allowDirect=!d.IsNull()&&d->StorageFormatVersion()>=TDocStd_FormatVersion_VERSION_12;
        if(d.IsNull()||d->GetData().IsNull())Standard_Failure::Raise("Private D1 writer document");
        if(!state_->allowDirect){
            // Reject our actual private direct role BEFORE native WriteSubTree
            // creates buffers/shape-section state. Old-only version11 still
            // exercises the unchanged real native writer and reader below.
            const auto root=d->GetData()->Root();std::size_t labels=0;
            if(root.IsAttribute(ProbeAttribute::Guid()))Standard_Failure::Raise("Private D1 old-version role");
            for(TDF_ChildIterator it(root,Standard_True);it.More();it.Next()){
                if(++labels>1000)Standard_Failure::Raise("Private D1 preflight labels");
                if(it.Value().IsAttribute(ProbeAttribute::Guid())){
                    Trace("old-version-private-role-refused");
                    Standard_Failure::Raise("Private D1 old-version role");
                }
            }
        }
    }
    const std::shared_ptr<State> state_;
    const Handle(Driver) driver_;
};
class Cancel final : public Message_ProgressIndicator {
public: Standard_Boolean UserBreak() override{return true;}
    void Show(const Message_ProgressScope&,const Standard_Boolean) override{}
};
inline frame::TraversalLimits Limits(){
    return {1024*1024,1000,10000,64,100000,1024*1024,4*1024*1024};
}
struct AppScope {
    Handle(TDocStd_Application) app=new TDocStd_Application();
    Handle(TDocStd_Document) doc;
    ~AppScope(){Trace("scope-close-begin");try{if(!doc.IsNull())app->Close(doc);Trace("scope-close-complete");}
        catch(...){Trace("scope-close-exception");}}
};
using Factory=std::function<Handle(PCDM_RetrievalDriver)(const Handle(frame::FrameDriver)&,frame::TraversalLimits)>;
inline void Configure(const Handle(TDocStd_Application)& app,bool xcaf,const std::shared_ptr<State>& state,
                      bool modern,frame::TraversalLimits limits,const Factory& reader){
    Handle(PCDM_RetrievalDriver) r=reader(modern?Handle(frame::FrameDriver)(new Driver(state)):Handle(frame::FrameDriver)(),limits);
    Handle(PCDM_StorageDriver) w=xcaf?Handle(PCDM_StorageDriver)(new Writer<BinXCAFDrivers_DocumentStorageDriver>(state))
                                    :Handle(PCDM_StorageDriver)(new Writer<BinDrivers_DocumentStorageDriver>(state));
    app->DefineFormat(xcaf?"BinXCAF":"BinOcaf","Private D1 framing fixture",xcaf?"xbf":"cbf",r,w);
}
inline std::string Build(bool xcaf,int probes,bool nested,int chain,int version,const Factory& reader,bool roleAlias=false){
    TraceVersion=version;Trace("build-begin",probes);
    AppScope s;auto state=std::make_shared<State>();Configure(s.app,xcaf,state,true,Limits(),reader);
    s.app->NewDocument(xcaf?"BinXCAF":"BinOcaf",s.doc);if(s.doc.IsNull())return {};
    s.doc->ChangeStorageFormatVersion(static_cast<TDocStd_FormatVersion>(version));
    const auto original=BRepPrimAPI_MakeBox(2,3,4).Shape();gp_Trsf t;t.SetTranslation(gp_Vec(10,0,0));
    const TopLoc_Location moved(t);auto second=original;second.Location(moved);
    TNaming_Builder(s.doc->Main().FindChild(101,true)).Generated(original);
    TNaming_Builder(s.doc->Main().FindChild(102,true)).Generated(second);
    if(xcaf)XCAFDoc_Location::Set(s.doc->Main().FindChild(103,true),moved);
    const auto parent=TDataStd_TreeNode::Set(s.doc->Main().FindChild(110,true));
    const auto child=TDataStd_TreeNode::Set(s.doc->Main().FindChild(111,true));
    if(!parent->Append(child))return {};
    if(roleAlias)TDataStd_Integer::Set(s.doc->Main().FindChild(104,true),ProbeAttribute::Guid(),17);
    for(int i=0;i<probes;++i){
        auto label=s.doc->GetData()->Root().FindChild(80+i,true);
        if(nested)label=label.FindChild(1,true);
        Handle(ProbeAttribute) a=new ProbeAttribute();a->valid=true;label.AddAttribute(a);
    }
    if(chain){auto l=s.doc->Main().FindChild(200,true);for(int i=0;i<chain;++i)l=l.FindChild(1,true);TDataStd_Integer::Set(l,17);}
    std::ostringstream out(std::ios::out|std::ios::binary);
    Trace("save-begin",probes);
    try {const auto status=s.app->SaveAs(s.doc,out);Trace("save-return",int(status));if(status!=PCDM_SS_OK)return {};}
    catch(...){Trace("save-exception");return {};}
    Trace("build-complete");return out.str();
}
inline bool Geometry(const Handle(TDocStd_Document)& doc,bool xcaf){
    Trace("geometry-begin");
    if(doc.IsNull()||doc->GetData().IsNull()){Trace("geometry-missing-document");return false;}
    const auto main=doc->Main();if(main.IsNull()){Trace("geometry-missing-main");return false;}
    std::array<TopoDS_Shape,2> shapes;
    for(int i=0;i<2;++i){Handle(TNaming_NamedShape)n;
        const auto label=main.FindChild(101+i,false);
        if(label.IsNull()||!label.FindAttribute(TNaming_NamedShape::GetID(),n)||n.IsNull()){
            Trace("geometry-missing-shape",101+i);return false;}

        shapes[std::size_t(i)]=TNaming_Tool::GetShape(n);if(shapes[std::size_t(i)].IsNull())return false;
        GProp_GProps g;BRepGProp::VolumeProperties(shapes[std::size_t(i)],g);
        if(std::abs(g.Mass()-24)>1e-9)return false;
        Bnd_Box b;BRepBndLib::Add(shapes[std::size_t(i)],b,false);double a,c,d,e,f,h;b.Get(a,c,d,e,f,h);
        const double shift=10*i;
        if(std::abs(a-shift)>1e-6 || std::abs(c)>1e-6 || std::abs(d)>1e-6 ||
           std::abs(e-shift-2)>1e-6 || std::abs(f-3)>1e-6 || std::abs(h-4)>1e-6)return false;
    }
    Handle(TDataStd_TreeNode) parent,child;
    const auto parentLabel=main.FindChild(110,false),childLabel=main.FindChild(111,false);
    if(parentLabel.IsNull()||childLabel.IsNull() ||
       !parentLabel.FindAttribute(TDataStd_TreeNode::GetDefaultTreeID(),parent) ||
       !childLabel.FindAttribute(TDataStd_TreeNode::GetDefaultTreeID(),child) ||
       parent.IsNull() || child.IsNull() || parent->First()!=child || child->Father()!=parent)return false;
    if(xcaf){Handle(XCAFDoc_Location) loc;
        const auto label=main.FindChild(103,false);
        if(label.IsNull()||!label.FindAttribute(XCAFDoc_Location::GetID(),loc)||loc.IsNull())return false;
        const auto transform=loc->Get().Transformation();
        for(int row=1;row<=3;++row)for(int col=1;col<=4;++col){
            const double expected=col==4?(row==1?10:0):(row==col?1:0);
            if(transform.Value(row,col)!=expected)return false;
        }
    }
    Trace("geometry-complete");return shapes[0].TShape()==shapes[1].TShape();
}
struct Result{bool admitted=false,geometry=false,probe=false,rejected=false,reentryRefused=false;int calls=0;};
inline Result Open(const std::string& bytes,bool xcaf,bool modern,frame::TraversalLimits limits,
                   const Factory& reader,int fault=0,bool wrongTarget=false,bool cancel=false,
                   bool mutateBudget=false,bool filtered=false,bool reentrant=false){
    AppScope s;auto state=std::make_shared<State>();state->decodeFault=fault;state->wrongTarget=wrongTarget;
    if(mutateBudget){std::weak_ptr<State> weak=state;state->afterPaste=[weak](){if(auto s=weak.lock())s->budget->maximumWireBytes+=1;};}
    Configure(s.app,xcaf,state,modern,limits,reader);Core3DBeginSafeBinaryRead();
    bool reentryRefused=false;
    struct ClearCallback { std::shared_ptr<State> state;~ClearCallback(){state->afterPaste={};} } clearCallback{state};
    if(reentrant)state->afterPaste=[&](){
        std::istringstream nested(bytes,std::ios::in|std::ios::binary);Handle(TDocStd_Document) doc;
        const auto status=s.app->Open(nested,doc);
        reentryRefused=status!=PCDM_RS_OK&&state->budget->rejected;
        if(!doc.IsNull())s.app->Close(doc);
    };
    Trace("open-begin",modern?1:0);
    std::istringstream in(bytes,std::ios::in|std::ios::binary);PCDM_ReaderStatus status=PCDM_RS_DriverFailure;
    try{if(cancel){Handle(Cancel)c=new Cancel();status=s.app->Open(in,s.doc,c->Start());}
        else if(filtered){Handle(PCDM_ReaderFilter) f=new PCDM_ReaderFilter();status=s.app->Open(in,s.doc,f);}
        else status=s.app->Open(in,s.doc);}catch(...){Trace("open-exception");}
    Trace("open-return",int(status));
    Result r;r.rejected=Core3DSafeBinaryReadWasRejected();r.admitted=status==PCDM_RS_OK&&!r.rejected&&!s.doc.IsNull()&&!s.doc->GetData().IsNull();
    r.calls=state->pasteCalls;r.reentryRefused=reentryRefused;
    if(r.admitted){
        try{r.geometry=Geometry(s.doc,xcaf);}catch(...){Trace("geometry-exception");r.geometry=false;}
        Handle(ProbeAttribute)a;
        const auto l=s.doc->GetData()->Root().FindChild(80,false);
        r.probe=!l.IsNull()&&l.FindAttribute(ProbeAttribute::Guid(),a)&&!a.IsNull()&&a->valid;}
    Trace("open-complete",r.admitted?1:0);state->afterPaste={};return r;
}
inline std::vector<std::size_t> Bodies(const std::string& bytes){
    std::vector<std::size_t> result;std::size_t p=0;
    while((p=bytes.find("SYD1BODY",p))!=std::string::npos){result.push_back(p);p+=8;}
    return result;
}
inline bool Inverse(const std::string& bytes,std::size_t body){
    const auto* p=reinterpret_cast<const unsigned char*>(bytes.data()+body-8);
    if(frame::Decode64(p,false)==16)return false;
    if(frame::Decode64(p,true)==16)return true;
    Standard_Failure::Raise("Unexpected native frame extent");return false;
}
template<class T> inline void Replace(std::string& bytes,std::size_t position,T value,bool inverse){
    std::array<char,sizeof(T)> wire{};std::memcpy(wire.data(),&value,sizeof value);
    if(inverse)std::reverse(wire.begin(),wire.end());bytes.replace(position,sizeof(T),wire.data(),wire.size());
}
// Complete minimal native subtree, with physically supplied prefix bytes and
// matching label/file extents. This isolates zero-prefix admission from EOF.
inline bool CompletePrefixProbe(std::int32_t prefix,bool inverse,bool replaceDriver=false) {
    auto state=std::make_shared<State>();state->budget->maximumWireBytes=1024*1024;
    Handle(frame::FrameDriver) driver=new Driver(state);
    Handle(BinMDF_ADriverTable) table=new BinMDF_ADriverTable();table->AddDriver(driver);
    TColStd_SequenceOfAsciiString names;names.Append(driver->TypeName());table->AssignIds(names);
    std::string wire;
    auto append=[&](auto value){const auto old=wire.size();wire.resize(old+sizeof(value));Replace(wire,old,value,inverse);};
    append(std::int32_t(0));const auto rootSize=wire.size();append(UINT64_C(0));
    append(std::int32_t(BinLDrivers_ENDATTRLIST));append(std::int32_t(80));
    const auto childSize=wire.size();append(UINT64_C(0));
    append(std::int32_t(1));append(std::int32_t(-7));append(prefix);
    wire.append(std::size_t(prefix),'\0');append(UINT64_C(16));wire.append("SYD1BODY",8);
    append(std::int32_t(BinLDrivers_ENDATTRLIST));append(std::int32_t(BinLDrivers_ENDLABEL));
    Replace(wire,childSize,std::uint64_t(wire.size()-childSize),inverse);
    append(std::int32_t(BinLDrivers_ENDLABEL));Replace(wire,rootSize,std::uint64_t(wire.size()-rootSize),inverse);
    auto limits=Limits();bool rejected=false;frame::Traversal traversal(table,1,driver,limits,[&](){rejected=true;});
    if(replaceDriver)table->AddDriver(new Driver(std::make_shared<State>()));
    std::istringstream input(wire,std::ios::in|std::ios::binary);input.seekg(4,std::ios::beg);
    Handle(TDF_Data) data=new TDF_Data();BinObjMgt_RRelocationTable relocation;
    const auto n=traversal.Read(input,data->Root(),table,relocation,Handle(PCDM_ReaderFilter)(),true,Message_ProgressRange());
    if(prefix==0&&!replaceDriver){Handle(ProbeAttribute)a;const auto l=data->Root().FindChild(80,false);
        return n==1&&traversal.complete()&&!rejected&&state->pasteCalls==1&&!l.IsNull()&&
            l.FindAttribute(ProbeAttribute::Guid(),a)&&!a.IsNull()&&a->valid;}
    return n<0&&rejected&&state->budget->rejected&&state->pasteCalls==0;
}
inline std::map<std::string,bool> Run(int scenario,const Factory& reader){
    struct TraceScope { bool previous=TraceEnabled;int format=TraceFormat,version=TraceVersion;
        ~TraceScope(){TraceEnabled=previous;TraceFormat=format;TraceVersion=version;} } traceScope;
    TraceEnabled=scenario==0;
    std::map<std::string,bool> result;
    for(bool xcaf:{false,true}){
        TraceFormat=xcaf?1:0;TraceVersion=0;Trace("format-begin");
        const std::string prefix=xcaf?"xcaf.":"ocaf.";
        const auto good=Build(xcaf,1,false,0,12,reader);const auto locations=Bodies(good);
        if(good.empty()||locations.size()!=1||locations[0]<32){result[prefix+"setup"]=false;continue;}
        const auto body=locations[0];const auto inverse=Inverse(good,body);
        const auto read=Open(good,xcaf,true,Limits(),reader);
        result[prefix+"positive"]=read.admitted&&read.geometry&&read.probe&&read.calls==1;
        if(scenario==0){
            const auto old=Build(xcaf,0,false,0,12,reader);
            const auto legacy=Open(old,xcaf,false,Limits(),reader);
            const auto newOld=Open(old,xcaf,true,Limits(),reader);
            result[prefix+"oldOnly"]=!old.empty()&&old.find("Core3D_ReceiptFramingProbe")==std::string::npos&&
                legacy.admitted&&legacy.geometry&&legacy.calls==0&&newOld.admitted&&newOld.geometry&&newOld.calls==0;
            const auto closed=Open(good,xcaf,false,Limits(),reader);
            result[prefix+"oldReaderRefusesV3"]=!closed.admitted&&closed.rejected&&closed.calls==0;
            result[prefix+"oldV3WriteRefused"]=Build(xcaf,1,false,0,11,reader).empty();
            const auto old11=Build(xcaf,0,false,0,11,reader);const auto old11read=Open(old11,xcaf,true,Limits(),reader);
            result[prefix+"oldVersionStillLoads"]=!old11.empty()&&old11read.admitted&&old11read.geometry;
        }else if(scenario==1){
            result[prefix+"completePrefixPositive"]=CompletePrefixProbe(0,inverse);
            result[prefix+"completePrefix4"]=CompletePrefixProbe(4,inverse);
            result[prefix+"completePrefix16MiB"]=CompletePrefixProbe(16*1024*1024,inverse);
            for(auto v:{std::int32_t(-1),std::int32_t(16*1024*1024),std::int32_t(INT32_MAX)}){
                auto malformed=good;Replace(malformed,body-12,v,inverse);
                const auto r=Open(malformed,xcaf,true,Limits(),reader);
                result[prefix+"prefix"+std::to_string(v)]=!r.admitted&&r.rejected&&r.calls==0;
            }
            for(auto v:{UINT64_C(0),UINT64_C(7),UINT64_MAX}){
                auto malformed=good;Replace(malformed,body-8,v,inverse);const auto r=Open(malformed,xcaf,true,Limits(),reader);
                result[prefix+"extent"+std::to_string(v)]=!r.admitted&&r.rejected&&r.calls==0;
            }
            auto malformed=good;Replace(malformed,body-16,std::int32_t(INT32_MIN),inverse);
            auto r=Open(malformed,xcaf,true,Limits(),reader);result[prefix+"minimumID"]=!r.admitted&&r.rejected&&r.calls==0;
            malformed=good;Replace(malformed,body-28,UINT64_C(15),inverse);
            r=Open(malformed,xcaf,true,Limits(),reader);result[prefix+"labelExtent"]=!r.admitted&&r.rejected&&r.calls==0;
            malformed=good;malformed.resize(body+7);r=Open(malformed,xcaf,true,Limits(),reader);
            result[prefix+"truncated"]=!r.admitted;
        }else if(scenario==2){
            const auto aliasRole=Open(Build(xcaf,1,false,0,12,reader,true),xcaf,true,Limits(),reader);
            result[prefix+"roleAlias"]=!aliasRole.admitted&&aliasRole.rejected;
            auto two=Build(xcaf,2,false,0,12,reader);auto places=Bodies(two);
            auto r=Open(two,xcaf,true,Limits(),reader);result[prefix+"secondReceipt"]=!r.admitted&&r.rejected&&r.calls==1;
            if(places.size()!=2||places[0]<20||places[1]<20){result[prefix+"alias"]=false;}
            else{const auto* a=reinterpret_cast<const unsigned char*>(two.data()+places[0]-16);
                const auto id=frame::Signed32(frame::Decode32(a,inverse));Replace(two,places[1]-16,id,inverse);
                r=Open(two,xcaf,true,Limits(),reader);result[prefix+"alias"]=!r.admitted&&r.rejected&&r.calls==1;}
            r=Open(Build(xcaf,1,true,0,12,reader),xcaf,true,Limits(),reader);
            result[prefix+"nestedReceipt"]=!r.admitted&&r.rejected&&r.calls==0;
            auto limits=Limits();limits.depth=4;
            r=Open(Build(xcaf,1,false,10,12,reader),xcaf,true,limits,reader);
            result[prefix+"depth"]=!r.admitted&&r.rejected;
            limits=Limits();limits.labels=1;r=Open(good,xcaf,true,limits,reader);
            result[prefix+"labels"]=!r.admitted&&r.rejected&&r.calls==0;
            limits=Limits();limits.parserSteps=1;r=Open(good,xcaf,true,limits,reader);
            result[prefix+"work"]=!r.admitted&&r.rejected&&r.calls==0;
        }else if(scenario==3){
            result[prefix+"driverProvenance"]=CompletePrefixProbe(0,inverse,true);
            auto special=Open(good,xcaf,true,Limits(),reader,0,false,false,false,true);
            result[prefix+"fullLoadOnly"]=!special.admitted&&special.rejected&&special.calls==0;
            special=Open(good,xcaf,true,Limits(),reader,0,false,false,false,false,true);
            result[prefix+"reentry"]=!special.admitted&&special.rejected&&special.reentryRefused&&special.calls==1;
            for(int fault:{1,2,3,4}){const auto r=Open(good,xcaf,true,Limits(),reader,fault);
                result[prefix+"decoder"+std::to_string(fault)]=!r.admitted&&r.rejected&&r.calls==1;}
            auto r=Open(good,xcaf,true,Limits(),reader,0,true);result[prefix+"target"]=!r.admitted&&r.rejected&&r.calls==0;
            r=Open(good,xcaf,true,Limits(),reader,0,false,true);result[prefix+"cancel"]=!r.admitted&&r.calls==0;
            r=Open(good,xcaf,true,Limits(),reader,0,false,false,true);result[prefix+"budgetProvenance"]=!r.admitted&&r.rejected&&r.calls==1;
            auto limits=Limits();limits.wireBytes=27;r=Open(good,xcaf,true,limits,reader);
            result[prefix+"wireBudget"]=!r.admitted&&r.rejected&&r.calls==0;
            auto damaged=good;damaged[body]='X';r=Open(damaged,xcaf,true,Limits(),reader);
            result[prefix+"falsePaste"]=!r.admitted&&r.rejected&&r.calls==1;
            auto unknown=good;const auto where=unknown.find("Core3D_ReceiptFramingProbe");
            if(where==std::string::npos)result[prefix+"unknownType"]=false;
            else{unknown[where+24]='X';r=Open(unknown,xcaf,true,Limits(),reader);
                result[prefix+"unknownType"]=!r.admitted&&r.rejected&&r.calls==0;}
        }
    }
    return result;
}
} // namespace core3d::debug::receipt_framing_probe
#endif
