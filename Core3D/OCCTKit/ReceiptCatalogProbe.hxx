#pragma once
#if DEBUG
#include "ReceiptCatalogBinaryDriver.hxx"
#include "NativeModelingReceiptLegacyDebug.hxx"
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TNaming_Builder.hxx>
#include <TNaming_Tool.hxx>
#include <XCAFDoc_Location.hxx>
#include <TDataStd_TreeNode.hxx>
#include <PCDM_ReaderStatus.hxx>
#include <PCDM_StoreStatus.hxx>
#include <map>
#include <cstdio>
namespace core3d::receipt::v3 {
// Fixed DEBUG phase labels only, enabled for the format compatibility scenario.
inline thread_local bool TraceEnabled=false;
inline thread_local int TraceFormat=0;
inline void Trace(const char* phase)noexcept{
    if(TraceEnabled){std::fprintf(stderr,"[receipt-v3] format=%d phase=%s\n",TraceFormat,phase);std::fflush(stderr);}
}
// Synthetic codec/TDF fixtures only. Actual native-effect/Store workflows are
// separate Swift methods and never accept these synthetic records as authority.
struct DebugProbe {
    struct App {
        Handle(TDocStd_Application) app=new TDocStd_Application();Handle(TDocStd_Document) doc;
        ~App(){try{if(!doc.IsNull())app->Close(doc);}catch(...) {}}
    };
    static Record Fixture(unsigned index,const UUID& document,unsigned effects=1){
        Record r;r.key.accountScope.fill(11);r.key.command.fill(22);r.key.execution.fill(33);r.key.document=document;
        for(unsigned i=0;i<4;++i)r.key.request[15-i]=std::uint8_t(index>>(8*i));
        r.operation=Operation::CreateAssembly;
        for(unsigned i=0;i<effects;++i){Effect e;e.entity[15]=std::uint8_t(i+1);e.definition[15]=std::uint8_t(i+17);
            e.featureID[15]=std::uint8_t(i+33);e.geometry.fill(44);e.state.fill(55);r.effects.push_back(e);}return r;
    }
    static void New(App& holder,bool xcaf){
        Core3DDefineSafeBinXCAFFormat(holder.app);holder.app->NewDocument(xcaf?"BinXCAF":"BinOcaf",holder.doc);
        if(holder.doc.IsNull())throw std::invalid_argument("V3 fixture document");
        holder.doc->SetUndoLimit(40);holder.doc->ChangeStorageFormatVersion(TDocStd_FormatVersion_VERSION_12);
        static const Standard_GUID id("74386E4E-F620-498F-8092-E6D883AF33A4");
        TDataStd_AsciiString::Set(holder.doc->Main(),id,"e3b85b39-68bc-4947-bf27-79b80b5a0c65");
        auto shape=BRepPrimAPI_MakeBox(2,3,4).Shape();gp_Trsf transform;transform.SetTranslation(gp_Vec(10,0,0));
        const TopLoc_Location location(transform);auto second=shape;second.Location(location);
        TNaming_Builder(holder.doc->Main().FindChild(101,true)).Generated(shape);
        TNaming_Builder(holder.doc->Main().FindChild(102,true)).Generated(second);
        if(xcaf)XCAFDoc_Location::Set(holder.doc->Main().FindChild(103,true),location);
        const auto parent=TDataStd_TreeNode::Set(holder.doc->Main().FindChild(110,true));
        const auto child=TDataStd_TreeNode::Set(holder.doc->Main().FindChild(111,true));
        if(!parent->Append(child))throw std::invalid_argument("V3 fixture tree reference");
    }
    static bool Geometry(const Handle(TDocStd_Document)& doc,bool xcaf){
        if(doc.IsNull()||doc->GetData().IsNull())return false;
        const auto main=doc->Main();if(main.IsNull())return false;
        std::array<TopoDS_Shape,2> shapes;
        for(int i=0;i<2;++i){Handle(TNaming_NamedShape) a;
            const auto label=main.FindChild(101+i,false);
            if(label.IsNull()||!label.FindAttribute(TNaming_NamedShape::GetID(),a)||a.IsNull())return false;
            shapes[std::size_t(i)]=TNaming_Tool::GetShape(a);if(shapes[std::size_t(i)].IsNull())return false;
            GProp_GProps volume;BRepGProp::VolumeProperties(shapes[std::size_t(i)],volume);
            if(std::abs(volume.Mass()-24)>1e-9)return false;
            const auto t=shapes[std::size_t(i)].Location().Transformation();
            for(int r=1;r<=3;++r)for(int c=1;c<=4;++c)
                if(t.Value(r,c)!=(c==4?(r==1?10.0*i:0.0):(r==c?1.0:0.0)))return false;
        }
        if(shapes[0].TShape()!=shapes[1].TShape())return false;
        Handle(TDataStd_TreeNode) parent,child;
        const auto parentLabel=main.FindChild(110,false),childLabel=main.FindChild(111,false);
        if(parentLabel.IsNull()||childLabel.IsNull()
            ||!parentLabel.FindAttribute(TDataStd_TreeNode::GetDefaultTreeID(),parent)
            ||!childLabel.FindAttribute(TDataStd_TreeNode::GetDefaultTreeID(),child)
            ||parent.IsNull()||child.IsNull()||parent->First()!=child||child->Father()!=parent)return false;
        if(xcaf){Handle(XCAFDoc_Location) location;
            const auto label=main.FindChild(103,false);
            if(label.IsNull()||!label.FindAttribute(XCAFDoc_Location::GetID(),location)||location.IsNull()
                ||location->Get()!=shapes[1].Location())return false;}
        return true;
    }
    static Handle(Core3D_ModelingReceiptCatalog) Install(const Handle(TDocStd_Document)& doc,
        std::shared_ptr<const Tree> tree,int tag=80){
        if(!doc->HasOpenCommand())throw std::invalid_argument("V3 fixture transaction");
        Handle(Core3D_ModelingReceiptCatalog) attribute=new Core3D_ModelingReceiptCatalog();
        doc->GetData()->Root().FindChild(tag,true).AddAttribute(attribute);attribute->Backup();attribute->root_=std::move(tree);return attribute;
    }
    static std::shared_ptr<const Tree> EmptyFor(const Handle(TDocStd_Document)& doc){
        UUID id;if(!CatalogDocumentUUID(doc,id))throw std::invalid_argument("V3 fixture UUID");Digest empty;
        if(!ArchiveHash({},empty))throw std::invalid_argument("V3 empty digest");return Tree::Empty(id,empty,empty);
    }
    static std::string Save(App& holder){
        std::ostringstream stream(std::ios::out|std::ios::binary);
        if(holder.app->SaveAs(holder.doc,stream)!=PCDM_SS_OK)throw std::invalid_argument("V3 fixture save");return stream.str();
    }
    static bool Open(const std::string& bytes,bool xcaf,bool old,std::shared_ptr<const Tree>* result=nullptr){
        App holder;if(old)Core3DDebugDefineLegacyReceiptFormats(holder.app);else Core3DDefineSafeBinXCAFFormat(holder.app);
        std::istringstream stream(bytes,std::ios::in|std::ios::binary);Core3DBeginSafeBinaryRead();
        try{
            const auto status=holder.app->Open(stream,holder.doc);
            if(status!=PCDM_RS_OK||Core3DSafeBinaryReadWasRejected()||holder.doc.IsNull())return false;
            Catalog catalog;const auto read=Read(holder.doc,catalog);
            if(read!=ReadStatus::Valid&&read!=ReadStatus::Absent)return false;
            if(result)*result=catalog.tree;return Geometry(holder.doc,xcaf);
        }catch(...){return false;}
    }
    static std::map<std::string,bool> Migration(const Handle(OcctDocument)& owner){
        std::map<std::string,bool> out;
        if(owner.IsNull()||owner->Document().IsNull()||owner->Document()->HasOpenCommand())return out;
        auto doc=owner->Document();UUID document;
        if(!ParseUUID(owner->DocumentIdentifier(),document))return out;
        Catalog empty;if(Read(doc,empty)!=ReadStatus::Absent)return out;
        std::vector<Record> old,current;
        for(unsigned i=1;i<=64;++i){auto record=Fixture(i,document);record.policy=0;for(auto&e:record.effects)e.policy=0;old.push_back(record);}
        for(unsigned i=65;i<=128;++i)current.push_back(Fixture(i,document));
        std::vector<std::uint8_t> oldBytes,currentBytes;
        if(!legacy_debug::EncodeLegacy(old,oldBytes)||!Encode(current,currentBytes))return out;
        doc->NewCommand();if(!legacy_debug::Write(doc,oldBytes,true)||!legacy_debug::Write(doc,currentBytes,false)
            ||!doc->CommitCommand())throw std::invalid_argument("Mixed original receipt fixtures");
        Catalog prior;if(Read(doc,prior)!=ReadStatus::Valid)return out;
        out["prior128"]=prior.count()==128&&!prior.tree;
        auto append=Fixture(129,document);
        doc->NewCommand();const bool staged=Stage(owner,append,prior);Catalog candidate;
        const bool captured=Read(doc,candidate)==ReadStatus::Valid;
        out["migration129"]=staged&&captured&&candidate.tree&&candidate.count()==129;
        out["archivesExact"]=captured&&candidate.legacyBytes==oldBytes&&candidate.bytes==currentBytes;
        doc->AbortCommand();Catalog after;
        out["abortedMigrationExact"]=Read(doc,after)==ReadStatus::Valid&&after.matches(prior)&&!after.tree;
        doc->NewCommand();if(!Stage(owner,append,prior)||Read(doc,candidate)!=ReadStatus::Valid||!doc->CommitCommand())
            throw std::invalid_argument("Mixed receipt migration commit");
        auto root=candidate.tree;
        out["undoOldOnly"]=doc->Undo()&&Read(doc,after)==ReadStatus::Valid&&after.matches(prior)&&!after.tree;
        out["redoSameRoot"]=doc->Redo()&&Read(doc,after)==ReadStatus::Valid&&after.matches(candidate)&&after.tree==root;
        bool policies=true;
        for(const auto& record:old){Record exact;policies=policies&&root->lookup(record.key.request,exact)
            &&exact.policy==0&&exact.effects==record.effects&&exact.key==record.key;}
        out["originalLegacyPoliciesExact"]=policies;
        doc->NewCommand();out["duplicateOldRefused"]=!Stage(owner,Fixture(1,document),candidate);
        doc->AbortCommand();out["duplicateLeavesRoot"]=Read(doc,after)==ReadStatus::Valid&&after.matches(candidate);
        // Structurally valid unknown policies are present but cannot be appended.
        auto unknown=Fixture(130,document);unknown.policy=99;for(auto&e:unknown.effects)e.policy=99;
        std::array<std::uint8_t,MaximumRawRecord> raw{};std::size_t size=0;
        if(!RawRecord(unknown,raw,size))throw std::invalid_argument("Unknown policy fixture");
        const auto unknownTree=root->appendRaw(2,{raw.data(),size});
        Handle(Core3D_ModelingReceiptCatalog) attribute;
        if(!candidate.v3Label.FindAttribute(ScalableSchemaID(),attribute))throw std::invalid_argument("V3 role fixture");
        doc->NewCommand();attribute->Backup();attribute->root_=unknownTree;
        Catalog unsupported;out["unknownReadableNonappendable"]=Read(doc,unsupported)==ReadStatus::Valid
            &&unsupported.count()==130&&!unsupported.supportsAppend()&&!Stage(owner,Fixture(131,document),unsupported);
        doc->AbortCommand();out["unknownAbortExact"]=Read(doc,after)==ReadStatus::Valid&&after.matches(candidate);
        return out;
    }
    static std::map<std::string,bool> Run(int scenario){
        struct TraceScope{bool previous=TraceEnabled;int format=TraceFormat;
            ~TraceScope(){TraceEnabled=previous;TraceFormat=format;}} traceScope;
        TraceEnabled=scenario==1;
        std::map<std::string,bool> out;
        for(bool xcaf:{false,true}){
            TraceFormat=xcaf?1:0;Trace("format-begin");
            const std::string p=xcaf?"xcaf/":"ocaf/";App h;New(h,xcaf);auto tree=EmptyFor(h.doc);
            if(scenario==0){
                const auto before=AllocationBudget::processBytes();std::weak_ptr<const Tree> weak;
                h.doc->NewCommand();tree=tree->append(Fixture(1,tree->document()));auto attribute=Install(h.doc,tree);
                if(!h.doc->CommitCommand())throw std::invalid_argument("V3 fixture initial commit");
                auto original=tree;weak=original;
                auto empty=Handle(Core3D_ModelingReceiptCatalog)::DownCast(attribute->NewEmpty());
                out[p+"newEmptyInvalid"]=!empty.IsNull()&&!empty->root();
                auto backup=Handle(Core3D_ModelingReceiptCatalog)::DownCast(attribute->BackupCopy());
                out[p+"backupExact"]=!backup.IsNull()&&backup->root()==original;
                h.doc->NewCommand();attribute->Backup();attribute->root_=original->append(Fixture(2,tree->document()));
                const auto changed=attribute->root_;h.doc->AbortCommand();
                out[p+"abortExact"]=attribute->root()==original;
                h.doc->NewCommand();attribute->Backup();attribute->root_=changed;
                if(!h.doc->CommitCommand())throw std::invalid_argument("V3 fixture changed commit");
                out[p+"undoExact"]=h.doc->Undo()&&attribute->root()==original;
                out[p+"redoExact"]=h.doc->Redo()&&attribute->root()==changed;
                App foreign;New(foreign,xcaf);foreign.doc->NewCommand();auto target=Install(foreign.doc,{});
                attribute->Paste(target,new TDF_RelocationTable());
                out[p+"pasteDeepQuota"]=target->root()!=changed&&target->root()->budget()!=changed->budget()
                    &&target->root()->count()==changed->count()&&target->root()->document()==changed->document();
                Record copied;out[p+"pasteExactRecord"]=target->root()->lookup(Fixture(1,tree->document()).key.request,copied)
                    &&copied.effects==Fixture(1,tree->document()).effects;
                const auto pasted=target->root();target->Restore(attribute);
                out[p+"foreignRestoreDeepQuota"]=target->root()!=changed&&target->root()!=pasted
                    &&target->root()->budget()==pasted->budget()&&target->root()->budget()!=changed->budget()
                    &&target->root()->document()==changed->document()&&target->root()->count()==changed->count();
                static const Standard_GUID documentRole("74386E4E-F620-498F-8092-E6D883AF33A4");
                TDataStd_AsciiString::Set(foreign.doc->Main(),documentRole,"e3b85b39-68bc-4947-bf27-79b80b5a0c66");
                Catalog rejected;out[p+"foreignOwnerRefused"]=Read(foreign.doc,rejected)==ReadStatus::Malformed;
                foreign.doc->AbortCommand();target.Nullify();
                backup.Nullify();empty.Nullify();original.reset();tree.reset();
                // A retained immutable successor deliberately keeps shared leaves,
                // but does not retain the old Tree/root identity object itself.
                h.doc->ClearUndos();h.doc->ClearRedos();out[p+"oldRootReleased"]=weak.expired();
                out[p+"accountingLive"]=AllocationBudget::processBytes()>before;
            }else if(scenario==1){
                App missing;Core3DDefineSafeBinXCAFFormat(missing.app);
                missing.app->NewDocument(xcaf?"BinXCAF":"BinOcaf",missing.doc);
                if(missing.doc.IsNull())throw std::invalid_argument("Missing-geometry fixture document");
                out[p+"missingGeometryRefused"]=!Geometry(Handle(TDocStd_Document)(),xcaf)&&!Geometry(missing.doc,xcaf);
                Trace("old-only12-save");
                const auto oldOnly=Save(h);out[p+"oldOnlyOldReader"]=Open(oldOnly,xcaf,true);
                out[p+"oldOnlyNoV3Type"]=oldOnly.find("Core3D_ModelingReceiptCatalog")==std::string::npos;
                h.doc->ChangeStorageFormatVersion(TDocStd_FormatVersion_VERSION_11);
                Trace("old-only11-save");const auto old11=Save(h);
                out[p+"oldOnly11BothReaders"]=Open(old11,xcaf,true)&&Open(old11,xcaf,false);
                out[p+"oldOnly11SourceGeometry"]=Geometry(h.doc,xcaf)
                    &&old11.find("Core3D_ModelingReceiptCatalog")==std::string::npos;
                h.doc->ChangeStorageFormatVersion(TDocStd_FormatVersion_VERSION_12);
                for(unsigned i=1;i<=144;++i)tree=tree->append(Fixture(i,tree->document(),16));
                h.doc->NewCommand();Install(h.doc,tree);if(!h.doc->CommitCommand())throw std::invalid_argument("V3 install");
                const auto bytes=Save(h);std::shared_ptr<const Tree> reopened;
                out[p+"crossedBothOldLimits"]=tree->count()==144&&tree->wireBytes()>MaximumBytes;
                out[p+"bothFormatsSharedGeometry"]=Open(bytes,xcaf,false,&reopened);
                out[p+"oldReaderRefuses"]=!Open(bytes,xcaf,true);
                std::ostringstream before,after;BinaryDriver::WriteNumericTree(*tree,before);
                out[p+"numericReopenExact"]=reopened&&BinaryDriver::WriteNumericTree(*reopened,after)&&before.str()==after.str();
                Catalog beforeRefusal;if(Read(h.doc,beforeRefusal)!=ReadStatus::Valid)throw std::invalid_argument("V3 source before refusal");
                const auto undo=h.doc->GetAvailableUndos(),redo=h.doc->GetAvailableRedos();
                h.doc->ChangeStorageFormatVersion(TDocStd_FormatVersion_VERSION_11);
                // Use the actual production driver, not a mock Base callback.
                // No bytes can reach Base::Write before the role/version refusal.
                Handle(BinLDrivers_DocumentStorageDriver) writer=xcaf
                    ?Handle(BinLDrivers_DocumentStorageDriver)(new StorageDriver<BinXCAFDrivers_DocumentStorageDriver>())
                    :Handle(BinLDrivers_DocumentStorageDriver)(new StorageDriver<BinDrivers_DocumentStorageDriver>());
                std::ostringstream untouched;untouched<<"V3-preflight-sentinel";bool refused=false;
                Trace("v3-old-version-direct-refusal");
                try{writer->Write(h.doc,untouched);}catch(...){refused=true;}
                out[p+"oldVersionRefusedBeforeOutput"]=refused&&untouched.str()=="V3-preflight-sentinel";
                Trace("v3-old-version-app-refusal");
                try{Save(h);out[p+"oldVersionWriteRefused"]=false;}catch(...){out[p+"oldVersionWriteRefused"]=true;}
                Catalog afterRefusal;
                out[p+"refusalSourceCatalogHistoryExact"]=Read(h.doc,afterRefusal)==ReadStatus::Valid
                    &&afterRefusal.matches(beforeRefusal)&&Geometry(h.doc,xcaf)
                    &&h.doc->GetAvailableUndos()==undo&&h.doc->GetAvailableRedos()==redo
                    &&h.doc->StorageFormatVersion()==TDocStd_FormatVersion_VERSION_11;
                h.doc->ChangeStorageFormatVersion(TDocStd_FormatVersion_VERSION_12);
                Trace("v3-version12-retry-save");const auto recovered=Save(h);std::shared_ptr<const Tree> reopenedAfterRefusal;
                const bool openedAfterRefusal=Open(recovered,xcaf,false,&reopenedAfterRefusal);
                std::ostringstream retry;
                out[p+"version12RecoveryExact"]=openedAfterRefusal&&reopenedAfterRefusal
                    &&BinaryDriver::WriteNumericTree(*reopenedAfterRefusal,retry)&&retry.str()==before.str()
                    &&Read(h.doc,afterRefusal)==ReadStatus::Valid&&afterRefusal.matches(beforeRefusal)&&Geometry(h.doc,xcaf);
                Trace("format-complete");
            }else if(scenario==2){
                for(unsigned i=1;i<=129;++i)tree=tree->append(Fixture(i,tree->document()));
                h.doc->NewCommand();auto attribute=Install(h.doc,tree);if(!h.doc->CommitCommand())throw std::invalid_argument("V3 install");
                const auto bytes=Save(h);const auto start=bytes.find(std::string("SYR3CAT\0",8));
                if(start==std::string::npos||tree->wireBytes()>bytes.size()-start||bytes.find(std::string("SYR3CAT\0",8),start+1)!=std::string::npos)
                    throw std::invalid_argument("V3 exact unique payload");
                out[p+"positiveBeforeMutations"]=Open(bytes,xcaf,false);
                auto rehashed=[&](const std::function<void(std::string&)>& mutate){
                    auto changed=bytes;mutate(changed);Digest hash;
                    const auto payload=std::size_t(tree->wireBytes());
                    if(!CC_SHA256(changed.data()+start,CC_LONG(payload-32),hash.data()))throw std::invalid_argument("V3 fixture SHA");
                    std::copy(hash.begin(),hash.end(),changed.begin()+std::ptrdiff_t(start+payload-32));
                    return !Open(changed,xcaf,false);
                };
                // Each one-effect V2 entry has3 framing+245 exact record bytes.
                // Duplicate/order/policy/owner failures retain a valid body SHA.
                constexpr std::size_t firstRaw=104+3, secondRaw=104+248+3;
                if(tree->wireBytes()<secondRaw+245+32)throw std::invalid_argument("V3 complete mutation records");
                out[p+"duplicateRecordRehashed"]=rehashed([&](std::string& x){std::copy_n(x.begin()+std::ptrdiff_t(start+firstRaw+48),16,x.begin()+std::ptrdiff_t(start+secondRaw+48));});
                out[p+"unsortedRecordsRehashed"]=rehashed([&](std::string& x){for(std::size_t i=0;i<248;++i)std::swap(x[start+104+i],x[start+104+248+i]);});
                out[p+"zeroPolicyRehashed"]=rehashed([&](std::string& x){x[start+firstRaw+130]=0;x[start+firstRaw+131]=0;});
                out[p+"wrongRecordDocumentRehashed"]=rehashed([&](std::string& x){x[start+firstRaw+32]^=1;});
                out[p+"wrongArchiveBindingRehashed"]=rehashed([&](std::string& x){x[start+24]^=1;});
                out[p+"zeroDocumentRehashed"]=rehashed([&](std::string& x){std::fill_n(x.begin()+std::ptrdiff_t(start+8),16,char(0));});
                auto refused=[&](std::size_t offset,std::uint8_t value){auto changed=bytes;
                    if(start+offset>=changed.size())throw std::invalid_argument("V3 mutation bound");
                    changed[start+offset]=char(value);return !Open(changed,xcaf,false);};
                out[p+"unknownWireVersion"]=refused(3,'4');
                out[p+"hugeCount"]=refused(95,255);
                out[p+"hugeTotal"]=refused(103,255);
                out[p+"badChecksum"]=refused(std::size_t(tree->wireBytes()-1),std::uint8_t(bytes[start+tree->wireBytes()-1])^1);
                out[p+"truncated"]=!Open(bytes.substr(0,start+std::size_t(tree->wireBytes())-1),xcaf,false);
                h.doc->NewCommand();Install(h.doc,tree,81);Catalog rejected;
                out[p+"duplicateRole"]=Read(h.doc,rejected)==ReadStatus::Malformed;h.doc->AbortCommand();
                h.doc->NewCommand();attribute->Backup();attribute->root_.reset();
                out[p+"invalidPresentRefuses"]=Read(h.doc,rejected)==ReadStatus::Malformed;h.doc->AbortCommand();
                out[p+"abortRestoresValid"]=Read(h.doc,rejected)==ReadStatus::Valid&&rejected.tree==tree;
            }else if(scenario==3){
                // Actual allocator calls: deliberate one-byte-over admission is
                // rejected before allocation, not a mocked boolean budget.
                auto budget=MakeBudget();const auto prior=budget->bytes(),process=AllocationBudget::processBytes();
                try{auto* unexpected=Allocator<std::uint8_t>(budget).allocate(DocumentMemoryLimit+1);Allocator<std::uint8_t>(budget).deallocate(unexpected,DocumentMemoryLimit+1);out[p+"documentLimit"]=false;}
                catch(const std::bad_alloc&){out[p+"documentLimit"]=budget->bytes()==prior&&AllocationBudget::processBytes()==process;}
                try{auto* unexpected=ProcessAllocator<std::uint8_t>().allocate(ProcessMemoryLimit+1);ProcessAllocator<std::uint8_t>().deallocate(unexpected,ProcessMemoryLimit+1);out[p+"processLimit"]=false;}
                catch(const std::bad_alloc&){out[p+"processLimit"]=AllocationBudget::processBytes()==process;}
                auto* actual=Allocator<std::uint8_t>(budget).allocate(4096);
                out[p+"actualCharge"]=budget->bytes()==prior+4096&&AllocationBudget::processBytes()==process+4096;
                Allocator<std::uint8_t>(budget).deallocate(actual,4096);
                out[p+"actualRelease"]=budget->bytes()==prior&&AllocationBudget::processBytes()==process;
                tree=tree->append(Fixture(1,tree->document()));auto unsupported=Fixture(2,tree->document());
                unsupported.policy=99;for(auto&e:unsupported.effects)e.policy=99;
                std::array<std::uint8_t,MaximumRawRecord> raw{};std::size_t length=0;
                if(!RawRecord(unsupported,raw,length))throw std::invalid_argument("Unknown fixture");
                tree=tree->appendRaw(2,{raw.data(),length});Record same;
                out[p+"unknownReadable"]=tree->lookup(unsupported.key.request,same)&&same.policy==99;
                out[p+"unknownNotAppendable"]=!tree->supportsAppend();
                try{tree->append(Fixture(3,tree->document()));out[p+"unknownAppendRefused"]=false;}
                catch(...){out[p+"unknownAppendRefused"]=tree->count()==2;}
                try{tree->appendRaw(2,{raw.data(),length});out[p+"duplicateRefused"]=false;}
                catch(...){out[p+"duplicateRefused"]=tree->count()==2;}
            }else throw std::invalid_argument("V3 probe scenario");
        }
        return out;
    }
};
} // namespace core3d::receipt::v3
#endif
