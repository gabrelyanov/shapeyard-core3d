#pragma once
#include "ReceiptCatalogTree.hxx"
#include "ReceiptLegacyCatalog.hxx"
#include <TDF_Attribute.hxx>
#include <TDF_RelocationTable.hxx>
#include <Standard_Failure.hxx>
class OcctDocument;
namespace core3d::receipt {
struct Catalog;
inline bool Stage(const Handle(OcctDocument)&,const Record&,const Catalog&) noexcept;
namespace v3 { class BinaryDriver;
#if DEBUG
struct DebugProbe;
#endif
}
class Core3D_ModelingReceiptCatalog final : public TDF_Attribute {
public:
    DEFINE_STANDARD_RTTI_INLINE(Core3D_ModelingReceiptCatalog,TDF_Attribute)
    // Backup/NewEmpty attribute objects also pay the process pool. The final
    // class has one exact allocation size; ordinary TDF delta containers remain
    // framework-owned, as for the prior receipt representation.
    static void* operator new(std::size_t size){
        if(size!=sizeof(Core3D_ModelingReceiptCatalog))throw std::bad_alloc();
        return v3::ProcessAllocator<std::uint8_t>().allocate(size);
    }
    static void operator delete(void* value)noexcept{
        if(value)v3::ProcessAllocator<std::uint8_t>().deallocate(static_cast<std::uint8_t*>(value),sizeof(Core3D_ModelingReceiptCatalog));
    }
    const Standard_GUID& ID()const override{return ScalableSchemaID();}
    Handle(TDF_Attribute) NewEmpty()const override{return new Core3D_ModelingReceiptCatalog();}
    const std::shared_ptr<const v3::Tree>& root()const noexcept{return root_;}
    void Restore(const Handle(TDF_Attribute)& source) override {
        const auto original=Handle(Core3D_ModelingReceiptCatalog)::DownCast(source);
        if(original.IsNull())Standard_Failure::Raise("Receipt restore type");
        // Real TDF BackupCopy targets an unattached attribute; Undo/Redo restore
        // from that immutable backup without allocation. Direct cross-document
        // Restore must not transplant another live document's quota lineage.
        if(!Label().IsNull()&&!original->Label().IsNull()
            &&Label().Data()!=original->Label().Data()&&original->root_){
            const auto quota=root_?root_->budget():v3::MakeBudget();
            auto copy=original->root_->copyTo(quota);root_=std::move(copy);
        }else root_=original->root_;
    }
    void Paste(const Handle(TDF_Attribute)& target,const Handle(TDF_RelocationTable)&)const override {
        const auto destination=Handle(Core3D_ModelingReceiptCatalog)::DownCast(target);
        if(destination.IsNull()||!root_||Label().IsNull()||destination->Label().IsNull())
            Standard_Failure::Raise("Receipt paste source/target");
        std::shared_ptr<const v3::Tree> copy;
        if(Label().Data()==destination->Label().Data())copy=root_;
        else {
            // Cross-document copy keeps all IDs, hence remains foreign evidence
            // unless the actual destination document UUID already matches.
            const auto quota=destination->root_?destination->root_->budget():v3::MakeBudget();
            copy=root_->copyTo(quota);
        }
        destination->Backup();destination->root_=std::move(copy);
    }
private:
    friend class v3::BinaryDriver;
#if DEBUG
    friend struct v3::DebugProbe;
#endif
    friend bool Stage(const Handle(OcctDocument)&,const Record&,const Catalog&) noexcept;
    std::shared_ptr<const v3::Tree> root_;
};
struct Catalog : LegacyCatalog {
    std::shared_ptr<const v3::Tree> tree;
    bool matches(const Catalog& other)const noexcept{
        return LegacyCatalog::matches(other)&&tree==other.tree;
    }
    std::uint64_t count()const noexcept{return tree?tree->count():records.size();}
    bool contains(const UUID& request)const noexcept{
        if(tree)return tree->contains(request);
        return std::any_of(records.begin(),records.end(),[&](const auto&r){return r.key.request==request;});
    }
    bool lookup(const UUID& request,Record& result)const{
        if(tree)return tree->lookup(request,result);
        for(const auto& record:records)if(record.key.request==request){result=record;return true;}return false;
    }
    bool supportsAppend()const noexcept{return tree?tree->supportsAppend():LegacyCatalog::supportsAppend();}
    bool canAppend()const noexcept{
        if(tree)return tree->canAppendMaximum();
        // Legacy populations remain bounded at128; migration allocates at most
        // that population plus one path. Actual allocator/Stage still recheck.
        return supportsAppend()&&v3::AllocationBudget::processBytes()<v3::ProcessMemoryLimit-2*1024*1024;
    }
};
inline bool CatalogDocumentUUID(const Handle(TDocStd_Document)& document,UUID& uuid){
    static const Standard_GUID id("74386E4E-F620-498F-8092-E6D883AF33A4");
    Handle(TDataStd_AsciiString) value;
    return !document.IsNull()&&document->Main().FindAttribute(id,value)&&!value.IsNull()
        &&ParseUUID(value->Get().ToCString(),uuid);
}
inline ReadStatus Read(const Handle(TDocStd_Document)& doc,Catalog& out) noexcept {
    out={};try{
        LegacyCatalog old;const auto status=ReadLegacy(doc,old);
        if(status!=ReadStatus::Valid&&status!=ReadStatus::Absent)return status;
        Catalog found;static_cast<LegacyCatalog&>(found)=std::move(old);
        if(found.v3Label.IsNull()){out=std::move(found);return status;}
        Handle(Core3D_ModelingReceiptCatalog) attribute;
        if(!found.v3Label.FindAttribute(ScalableSchemaID(),attribute)||attribute.IsNull()||!attribute->root())return ReadStatus::Malformed;
        found.tree=attribute->root();UUID document;Digest legacy,versioned;
        if(!CatalogDocumentUUID(doc,document)||document!=found.tree->document()||found.tree->count()==0
            ||found.tree->wireBytes()>v3::WireLimit
            ||!v3::ArchiveHash(found.legacyBytes,legacy)||!v3::ArchiveHash(found.bytes,versioned)
            ||legacy!=found.tree->legacyHash()||versioned!=found.tree->versionedHash())return ReadStatus::Malformed;
        // Archives remain authoritative exact bytes. Their original records must
        // also be present with identical grammar/policy/effect/key values.
        const auto legacyCount=std::count_if(found.records.begin(),found.records.end(),[](const auto& r){return r.policy==0;});
        if(found.tree->legacyCount()!=std::uint64_t(legacyCount))return ReadStatus::Malformed;
        for(const auto& record:found.records){Record indexed;
            if(!found.tree->lookup(record.key.request,indexed)||!(indexed.key==record.key)||indexed.operation!=record.operation
                ||indexed.policy!=record.policy||indexed.effects!=record.effects)return ReadStatus::Malformed;}
        out=std::move(found);return ReadStatus::Valid;
    }catch(...){out={};return ReadStatus::Malformed;}
}
} // namespace core3d::receipt
