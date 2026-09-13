#pragma once
#include "ReceiptCatalogAttribute.hxx"
#include "ReceiptFramedTraversal.hxx"
#include <BinDrivers_DocumentStorageDriver.hxx>
#include <BinXCAFDrivers_DocumentStorageDriver.hxx>
#include <BinMDF_ADriverTable.hxx>
#include <BinMNaming_NamedShapeDriver.hxx>
#include <TNaming_NamedShape.hxx>
#include <TDocStd_FormatVersion.hxx>
#include <Message.hxx>
#include <istream>
#include <ostream>
namespace core3d::receipt::v3 {
inline persistence::receipt_framing::TraversalLimits ReaderLimits(){
    // Receipt body allowance plus its12-byte BinObjMgt header and8-byte extent.
    return {WireLimit+20,100000,1000000,256,8000000,128*1024*1024+64*1024,512*1024*1024};
}
class Wire final {
public:
    explicit Wire(std::istream& stream):input_(&stream){valid_=CC_SHA256_Init(&hash_)==1;}
    explicit Wire(std::ostream& stream):output_(&stream){valid_=CC_SHA256_Init(&hash_)==1;}
    bool data(void* buffer,std::size_t size){
        if(!valid_||size>WireLimit-position_)return false;
        if(input_){input_->read(static_cast<char*>(buffer),std::streamsize(size));if(!*input_)return false;}
        else{output_->write(static_cast<const char*>(buffer),std::streamsize(size));if(!*output_)return false;}
        if(CC_SHA256_Update(&hash_,buffer,CC_LONG(size))!=1){valid_=false;return false;}position_+=size;return true;
    }
    template<class A> bool array(A& value){return data(value.data(),value.size());}
    bool u64(std::uint64_t& value){
        std::array<std::uint8_t,8> bytes{};if(output_)for(unsigned i=0;i<8;++i)bytes[i]=std::uint8_t(value>>(8*i));
        if(!array(bytes))return false;if(input_){value=0;for(unsigned i=0;i<8;++i)value|=std::uint64_t(bytes[i])<<(8*i);}return true;
    }
    bool finish(){
        if(!valid_||32>WireLimit-position_)return false;Digest actual;
        if(CC_SHA256_Final(actual.data(),&hash_)!=1){valid_=false;return false;}
        if(input_){Digest expected;input_->read(reinterpret_cast<char*>(expected.data()),32);if(!*input_||actual!=expected)return false;}
        else {output_->write(reinterpret_cast<const char*>(actual.data()),32);if(!*output_)return false;}
        position_+=32;return true;
    }
    std::size_t position()const noexcept{return position_;}
private:
    std::istream* input_=nullptr;std::ostream* output_=nullptr;CC_SHA256_CTX hash_{};std::size_t position_=0;bool valid_=false;
};
class BinaryDriver final : public persistence::receipt_framing::FrameDriver {
public:
    explicit BinaryDriver(std::shared_ptr<persistence::receipt_framing::LoadBudget> framing,
                          std::shared_ptr<bool> writeAllowed=std::make_shared<bool>(false))
        :FrameDriver(Message::DefaultMessenger(),"Core3D_ModelingReceiptCatalog",std::move(framing)),writeAllowed_(std::move(writeAllowed)){}
    const Standard_GUID& AttributeID()const override{return ScalableSchemaID();}
    const Handle(Standard_Type)& SourceType()const override{return STANDARD_TYPE(Core3D_ModelingReceiptCatalog);}
    Handle(TDF_Attribute) NewEmpty()const override{return new Core3D_ModelingReceiptCatalog();}
    Standard_Boolean Paste(const BinObjMgt_Persistent& source,const Handle(TDF_Attribute)& target,
                           BinObjMgt_RRelocationTable&)const override {
        try{
            const auto attribute=Handle(Core3D_ModelingReceiptCatalog)::DownCast(target);
            if(attribute.IsNull()||attribute->root_||!budget()||budget()->rejected||!budget()->seenReceipt)return false;
            auto* stream=const_cast<BinObjMgt_Persistent&>(source).GetIStream();if(!stream)return false;
            Wire wire(*stream);std::array<std::uint8_t,8> magic{};UUID document;Digest legacy,versioned;
            std::uint64_t count=0,total=0;
            if(!wire.array(magic)||magic!=Magic()||!wire.array(document)||!wire.array(legacy)||!wire.array(versioned)
                ||!wire.u64(count)||!wire.u64(total)||total<FixedWireBytes||total>WireLimit||!count
                ||count>(total-FixedWireBytes)/246)return false;
            auto tree=Tree::Empty(document,legacy,versioned);UUID previous{};
            for(std::uint64_t i=0;i<count;++i){
                std::array<std::uint8_t,3> prefix{};if(!wire.array(prefix))return false;
                const auto length=std::size_t(prefix[1])+(std::size_t(prefix[2])<<8);
                if(length<243||length>MaximumRawRecord||wire.position()>total-32||length>total-32-wire.position())return false;
                std::array<std::uint8_t,MaximumRawRecord> raw{};if(!wire.data(raw.data(),length))return false;
                Record record;if(!DecodeRaw(prefix[0],{raw.data(),length},record)||!(previous<record.key.request))return false;
                previous=record.key.request;tree=tree->appendRaw(prefix[0],{raw.data(),length});
            }
            if(wire.position()!=total-32||!wire.finish()||wire.position()!=total||tree->wireBytes()!=total||tree->count()!=count)return false;
            // No partially parsed tree reaches a TDF attribute. Allocation quota
            // remains owned by all immutable nodes after driver/read teardown.
            attribute->root_=std::move(tree);return true;
        }catch(...){return false;}
    }
    void Paste(const Handle(TDF_Attribute)& source,BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable&)const override {
        const auto attribute=Handle(Core3D_ModelingReceiptCatalog)::DownCast(source);
        if(!writeAllowed_||!*writeAllowed_||attribute.IsNull()||!attribute->root_||attribute->Label().IsNull())
            Standard_Failure::Raise("Receipt write version/state");
        Catalog catalog;const auto document=TDocStd_Document::Get(attribute->Label());
        if(Read(document,catalog)!=ReadStatus::Valid||catalog.tree!=attribute->root_||catalog.v3Label!=attribute->Label())
            Standard_Failure::Raise("Receipt write source");
        auto* stream=target.GetOStream();if(!stream)Standard_Failure::Raise("Receipt write stream");
        if(!WriteNumericTree(*catalog.tree,*stream))Standard_Failure::Raise("Receipt body write");
    }
    // Pure exact numeric serialization, also used by bounded DEBUG evidence.
    static bool WriteNumericTree(const Tree& tree,std::ostream& stream){
        Wire wire(stream);auto magic=Magic();auto owner=tree.document();auto legacy=tree.legacyHash();
        auto versioned=tree.versionedHash();auto count=tree.count();auto total=tree.wireBytes();
        if(!count||total>WireLimit||!wire.array(magic)||!wire.array(owner)||!wire.array(legacy)||!wire.array(versioned)||!wire.u64(count)||!wire.u64(total))return false;
        const bool complete=tree.visit([&](const UUID&,std::uint8_t codec,std::span<const std::uint8_t> raw){
            std::array<std::uint8_t,3> prefix{codec,std::uint8_t(raw.size()),std::uint8_t(raw.size()>>8)};
            return wire.array(prefix)&&wire.data(const_cast<std::uint8_t*>(raw.data()),raw.size());
        });
        return complete&&wire.finish()&&wire.position()==total;
    }
private:
    static std::array<std::uint8_t,8> Magic(){return {'S','Y','R','3','C','A','T',0};}
    const std::shared_ptr<bool> writeAllowed_;
};
template<class Base> class StorageDriver final : public Base {
public:
    StorageDriver():writeAllowed_(std::make_shared<bool>(false)),driver_(new BinaryDriver(
        std::make_shared<persistence::receipt_framing::LoadBudget>(),writeAllowed_)){}
    Handle(BinMDF_ADriverTable) AttributeDrivers(const Handle(Message_Messenger)& messenger)override {
        auto table=Base::AttributeDrivers(messenger);table->AddDriver(driver_);return table;
    }
    void EnableQuickPartWriting(const Handle(Message_Messenger)& messenger,
                                const Standard_Boolean requestedMode)override {
        if(this->myDrivers.IsNull())this->myDrivers=this->AttributeDrivers(messenger);
        if(this->myDrivers.IsNull())Standard_Failure::Raise("Receipt shape driver table");
        Handle(BinMDF_ADriver) existing;
        this->myDrivers->GetDriver(STANDARD_TYPE(TNaming_NamedShape),existing);
        const auto shapes=Handle(BinMNaming_NamedShapeDriver)::DownCast(existing);
        if(shapes.IsNull())Standard_Failure::Raise("Receipt named shape driver type");
        // Nonquick section writes clear contents but retain the concrete cache.
        // Keep the same driver held by Location; recreate only its mode-specific
        // cache through OCCT's existing lazy ShapeSet construction.
        if(shapes->IsQuickPart()!=requestedMode)shapes->Clear();
        Base::EnableQuickPartWriting(messenger,requestedMode);
    }
    void Write(const Handle(CDM_Document)& doc,const TCollection_ExtendedString& file,
               const Message_ProgressRange& progress=Message_ProgressRange())override {
        Prepare(doc);Base::Write(doc,file,progress);
    }
    void Write(const Handle(CDM_Document)& doc,Standard_OStream& stream,
               const Message_ProgressRange& progress=Message_ProgressRange())override {
        Prepare(doc);Base::Write(doc,stream,progress);
    }
private:
    void Prepare(const Handle(CDM_Document)& doc){
        const auto native=Handle(TDocStd_Document)::DownCast(doc);
        *writeAllowed_=!native.IsNull()&&native->StorageFormatVersion()>=TDocStd_FormatVersion_VERSION_12;
        if(native.IsNull()||native->GetData().IsNull())Standard_Failure::Raise("Receipt writer document");
        if(!*writeAllowed_){
            // Reject the actual direct attribute BEFORE Base::Write allocates
            // native buffering/shape-section state. A real old-only version11
            // document still delegates to the unchanged legacy writer.
            const auto root=native->GetData()->Root();std::uint64_t labels=0;
            if(root.IsAttribute(ScalableSchemaID()))Standard_Failure::Raise("Receipt old-version role");
            for(TDF_ChildIterator it(root,Standard_True);it.More();it.Next()){
                if(++labels>ReaderLimits().labels)Standard_Failure::Raise("Receipt preflight label bound");
                if(it.Value().IsAttribute(ScalableSchemaID()))Standard_Failure::Raise("Receipt old-version role");
            }
        }
    }
    const std::shared_ptr<bool> writeAllowed_;
    const Handle(BinaryDriver) driver_;
};
} // namespace core3d::receipt::v3
