#pragma once
#include "ReceiptDirectFrameOCCTAdapter.hxx"
#include <BinMDF_ADriverTable.hxx>
#include <BinLDrivers_Marker.hxx>
#include <Message_ProgressScope.hxx>
#include <Standard_DomainError.hxx>
#include <Standard_GUID.hxx>
#include <TDF_Label.hxx>
#include <TDF_Attribute.hxx>
#include <TNaming_NamedShape.hxx>
#include <XCAFDoc_Location.hxx>
#include <PCDM_ReaderFilter.hxx>
#include <set>
#include <memory>
#include <functional>
#include <sstream>
#include <string>

namespace core3d::persistence::receipt_framing {

// Receipt drivers share this exact per-reader load budget. The V3 owner
// successor registers the production immutable catalog driver separately.
class FrameDriver : public BinMDF_ADriver {
public:
    virtual const Standard_GUID& AttributeID() const = 0;
    // Must return the fixed descriptor; inherited OCCT SourceType allocates NewEmpty.
    const Handle(Standard_Type)& SourceType() const override = 0;
    const std::shared_ptr<LoadBudget>& budget() const noexcept { return budget_; }
protected:
    FrameDriver(const Handle(Message_Messenger)& messenger, const char* typeName,
                std::shared_ptr<LoadBudget> budget)
        : BinMDF_ADriver(messenger,typeName), budget_(std::move(budget)) {}
private:
    const std::shared_ptr<LoadBudget> budget_;
};

struct TraversalLimits {
    std::uint64_t wireBytes = 0;
    std::uint64_t labels = 0;
    std::uint64_t attributes = 0;
    std::uint64_t depth = 0;
    std::uint64_t parserSteps = 0;
    std::uint64_t bufferedAttributeBytes = 0;
    std::uint64_t aggregateBufferedBytes = 0;
    bool valid() const noexcept {
        return wireBytes >= 20 && labels > 0 && labels <= 100000 &&
            attributes > 0 && attributes <= 1000000 && depth > 0 && depth <= 256 &&
            parserSteps > 0 && bufferedAttributeBytes > 0 &&
            bufferedAttributeBytes <= std::uint64_t(INT32_MAX)-12 &&
            aggregateBufferedBytes >= bufferedAttributeBytes;
    }
};

// BinObjMgt reads only from the cached12-byte header followed by the declared
// prefix. Its raw header is never read twice. Legacy direct streams remain raw.
class CachedHeaderBuffer final : public std::streambuf {
public:
    CachedHeaderBuffer(std::istream& source, const std::array<unsigned char,12>& header,
                       std::uint64_t payload, std::function<bool()> more)
        : source_(source), header_(header), total_(12+payload), more_(std::move(more)) {}
    bool complete() const noexcept { return !failed_ && position_ == total_; }
protected:
    std::streamsize xsgetn(char* out, std::streamsize n) override {
        if (n < 0 || failed_ || std::uint64_t(n) > total_-position_ || !more_()) {
            failed_=true;return 0;
        }
        std::streamsize done=0;
        while (done<n && position_<12) out[done++]=char(header_[std::size_t(position_++)]);
        while(done<n) {
            if(!more_()){failed_=true;break;}
            const auto count=std::min<std::streamsize>(n-done,4096);
            source_.read(out+done,count);const auto got=source_.gcount();
            if(got>0){done+=got;position_+=std::uint64_t(got);}
            if(got!=count || !source_){failed_=true;break;}
        }
        return done;
    }
private:
    std::istream& source_;
    std::array<unsigned char,12> header_;
    std::uint64_t total_,position_=0;
    std::function<bool()> more_;
    bool failed_=false;
};

class Traversal final {
    struct Refusal {};
    struct Less {
        Traversal* owner;
        bool operator()(Standard_Integer a,Standard_Integer b) const {
            owner->Step();return a<b;
        }
    };
public:
    // The caller must pass the actual assigned table which base Read will use.
    // Exact driver pointer, declared type name and shared budget are fenced again
    // at every attribute; they cannot be substituted during Paste callbacks.
    Traversal(Handle(BinMDF_ADriverTable) table, Standard_Integer typeID,
              Handle(FrameDriver) receipt, TraversalLimits limits,
              std::function<void()> reject)
        : table_(std::move(table)), typeID_(typeID), receipt_(std::move(receipt)),
          limits_(limits), reject_(std::move(reject)), ids_(Less{this}) {
        if(receipt_.IsNull() || table_.IsNull() || !limits_.valid() || typeID_<=0 ||
           table_->GetDriver(typeID_)!=receipt_ || receipt_->SourceType().IsNull() ||
           receipt_->TypeName().IsEmpty() || !receipt_->budget()) throw Refusal{};
        targetType_=receipt_->SourceType();typeName_=receipt_->TypeName();targetID_=receipt_->AttributeID();
        budget_=receipt_->budget();
        if(budget_->rejected || budget_->seenReceipt || budget_->chargedWireBytes!=0 ||
           budget_->maximumWireBytes!=limits_.wireBytes) throw Refusal{};
        // Derive wire order from this actual linked OCCT library, not a guessed
        // platform preprocessor setting. This record contains no variable data.
        BinObjMgt_Persistent probe;probe.SetTypeId(0x01020304);probe.SetId(1);
        std::ostringstream out(std::ios::out|std::ios::binary);probe.Write(out);
        const auto wire=out.str();
        if(!out || wire.size()!=12)throw Refusal{};
        const auto* bytes=reinterpret_cast<const unsigned char*>(wire.data());
        if(Decode32(bytes,false)==0x01020304)inverse_=false;
        else if(Decode32(bytes,true)==0x01020304)inverse_=true;
        else throw Refusal{};
    }
    Standard_Integer Read(std::istream& source,const TDF_Label& root,
                          const Handle(BinMDF_ADriverTable)& actualTable,
                          BinObjMgt_RRelocationTable& relocation,
                          const Handle(PCDM_ReaderFilter)& filter, bool quick,
                          const Message_ProgressRange& range) {
        if(started_ || !quick || !filter.IsNull() || root.IsNull() || root.Father().IsNull()==false ||
           actualTable!=table_)return Fail();
        started_=true;source_=&source;relocation_=&relocation;
        Message_ProgressScope progress(range,"Reading framed receipt document",1,true);
        progress_=&progress;
        struct Finish { Traversal* p;~Finish(){p->progress_=nullptr;p->source_=nullptr;p->relocation_=nullptr;} } finish{this};
        try {
            const auto begin=Position();
            // Base OCCT has read/ignored the root tag immediately before this
            // virtual hook. This validation is its first authoritative use.
            if(begin<4)throw Refusal{};
            source.seekg(0,std::ios::end);fileEnd_=Position();
            if(begin>fileEnd_)throw Refusal{};
            source.seekg(std::streamoff(begin-4),std::ios::beg);
            if(ReadInt(fileEnd_)!=0 || Position()!=begin)throw Refusal{};
            const auto count=ReadLabel(root,fileEnd_,0);
            if(!Fenced() || !budget_->seenReceipt || receiptCount_!=1 || count<=0)throw Refusal{};
            complete_=true;return count;
        } catch (...) {return Fail();}
    }
    bool complete() const noexcept { return complete_ && budget_ && !budget_->rejected; }
    bool cancelled() const noexcept {return cancelled_;}
    std::uint64_t labelsRead() const noexcept {return labels_;}
    std::uint64_t attributesRead() const noexcept {return attributes_;}
private:
    Standard_Integer Fail() noexcept {
        complete_=false;if(budget_)budget_->refuse();
        try{if(reject_)reject_();}catch(...){}
        return -1;
    }
    void Step() {
        if(steps_>=limits_.parserSteps)throw Refusal{};++steps_;
        if(progress_ && !progress_->More()){cancelled_=true;throw Refusal{};}
        if(!Fenced())throw Refusal{};
    }
    bool Fenced() const {
        return !table_.IsNull() && !receipt_.IsNull() && budget_ && !budget_->rejected &&
            receipt_->budget()==budget_ && budget_->maximumWireBytes==limits_.wireBytes &&
            budget_->chargedWireBytes<=limits_.wireBytes && receipt_->SourceType()==targetType_ &&
            receipt_->TypeName()==typeName_ && receipt_->AttributeID()==targetID_ && table_->GetDriver(typeID_)==receipt_;
    }
    std::uint64_t Position() const {
        if(!source_ || !*source_)throw Refusal{};
        const auto p=source_->tellg();if(p==std::streampos(-1) || p<std::streampos(0))throw Refusal{};
        return std::uint64_t(std::streamoff(p));
    }
    void ReadBytes(unsigned char* out,std::size_t n,std::uint64_t end) {
        Step();const auto pos=Position();if(pos>end || n>end-pos)throw Refusal{};
        source_->read(reinterpret_cast<char*>(out),std::streamsize(n));
        if(!*source_ || source_->gcount()!=std::streamsize(n))throw Refusal{};
    }
    Standard_Integer ReadInt(std::uint64_t end) {
        std::array<unsigned char,4>b{};ReadBytes(b.data(),b.size(),end);
        return Signed32(Decode32(b.data(),inverse_));
    }
    std::uint64_t ReadSize(std::uint64_t end) {
        std::array<unsigned char,8>b{};ReadBytes(b.data(),b.size(),end);
        return Decode64(b.data(),inverse_);
    }
    Standard_Integer ReadLabel(const TDF_Label& label,std::uint64_t parentEnd,std::uint64_t depth) {
        Step();if(depth>=limits_.depth || labels_>=limits_.labels)throw Refusal{};++labels_;
        const auto begin=Position();const auto size=ReadSize(parentEnd);
        if(size<16 || begin>parentEnd || size>parentEnd-begin)throw Refusal{};
        const auto end=begin+size;Standard_Integer count=0;
        for(;;){
            std::array<unsigned char,12> raw{};ReadBytes(raw.data(),4,end);
            const auto type=Signed32(Decode32(raw.data(),inverse_));
            if(type==BinLDrivers_ENDATTRLIST)break;
            if(type<=0)throw Refusal{};
            ReadBytes(raw.data()+4,8,end);
            const auto signedID=Signed32(Decode32(raw.data()+4,inverse_));
            const auto length=Signed32(Decode32(raw.data()+8,inverse_));
            if(signedID==0 || signedID==INT32_MIN || length<0 || length>INT32_MAX-12 ||
               attributes_>=limits_.attributes)throw Refusal{};
            const auto id=signedID<0?-signedID:signedID;
            if(!ids_.insert(id).second)throw Refusal{};
            const bool wasBound=relocation_->IsBound(id);
            ++attributes_;auto driver=table_->GetDriver(type);if(driver.IsNull())throw Refusal{};
            const auto pos=Position();if(pos>end || std::uint64_t(length)>end-pos)throw Refusal{};
            if(type==typeID_){
                if(receiptCount_!=0 || driver!=receipt_ || depth!=1 || wasBound)throw Refusal{};
                ValidatedHeader header;
                if(!ValidatedHeader::Parse(raw,typeID_,inverse_,*budget_,header))throw Refusal{};
                auto target=driver->NewEmpty();
                if(target.IsNull() || target->DynamicType()!=targetType_ || target->ID()!=targetID_ || !target->Label().IsNull() ||
                   label.IsAttribute(target->ID()))throw Refusal{};
                // Matches OCCT's attached-target contract; failed private loads
                // are discarded and cannot make a present V3 become Absent.
                label.AddAttribute(target);
                budget_->continueReading=[this](){Step();return true;};
                struct ClearProgress { std::shared_ptr<LoadBudget> budget; ~ClearProgress(){budget->continueReading={};} } clearProgress{budget_};
                if(!PasteReceiptFrame(*source_,header,inverse_,end-pos,*budget_,driver,target,*relocation_))throw Refusal{};
                Step();if(target->Label()!=label || target->DynamicType()!=targetType_ || target->ID()!=targetID_ ||
                    receiptCount_!=0 || !Fenced())throw Refusal{};
                relocation_->Bind(id,target);++receiptCount_;
            }else{
                if(driver->SourceType()==targetType_ ||
                   (signedID<0)!=(driver->SourceType()==STANDARD_TYPE(TNaming_NamedShape) ||
                                    driver->SourceType()==STANDARD_TYPE(XCAFDoc_Location)) ||
                   std::uint64_t(length)>limits_.bufferedAttributeBytes ||
                   buffered_>limits_.aggregateBufferedBytes ||
                   std::uint64_t(length)>limits_.aggregateBufferedBytes-buffered_)throw Refusal{};
                buffered_+=std::uint64_t(length);
                CachedHeaderBuffer buffer(*source_,raw,std::uint64_t(length),[this](){Step();return true;});
                std::istream cached(&buffer);BinObjMgt_Persistent persistent;persistent.Read(cached);
                if(!cached || !buffer.complete() || persistent.IsError() || persistent.TypeId()!=type ||
                   persistent.Id()!=id || persistent.Length()!=length || persistent.IsDirect()!=(signedID<0))throw Refusal{};
                std::uint64_t directEnd=Position();
                if(signedID<0){
                    const auto directBegin=Position();const auto extent=ReadSize(end);
                    if(extent<8 || extent>end-directBegin)throw Refusal{};directEnd=directBegin+extent;
                    // Existing GetIStream skips this size field; shape readers
                    // keep the original absolute stream and shared-reference cache.
                    source_->seekg(std::streamoff(directBegin),std::ios::beg);if(!*source_)throw Refusal{};
                    persistent.SetIStream(*source_);
                }
                persistent.SetIStream(*source_);
                auto target=wasBound ? Handle(TDF_Attribute)::DownCast(relocation_->Find(id)) : driver->NewEmpty();
                if(target.IsNull() || !target->Label().IsNull() || target->DynamicType()!=driver->SourceType())throw Refusal{};
                try{label.AddAttribute(target);}catch(const Standard_DomainError&){
                    // Preserve OCCT arbitrary-GUID scalar/array attribute loading.
                    static const Standard_GUID invalid;target->SetID(invalid);label.AddAttribute(target);
                }
                if(!driver->Paste(persistent,target,*relocation_))throw Refusal{};
                Step();if(Position()!=directEnd || target->Label()!=label || target->ID()==targetID_)throw Refusal{};
                if(!wasBound)relocation_->Bind(id,target);
            }
            if(count==INT32_MAX)throw Refusal{};++count;
        }
        // OCCT serializes children in ascending TDF sibling order. Requiring
        // that order also preserves FindChild's last-found-child fast path.
        Standard_Integer previousTag=-1;
        for(;;){
            const auto tag=ReadInt(end);if(tag==BinLDrivers_ENDLABEL)break;
            if(tag<=previousTag || depth+1>=limits_.depth || labels_>=limits_.labels)throw Refusal{};
            previousTag=tag;
            // Child allocation follows tag/budget validation, not arbitrary depth.
            const auto child=label.FindChild(tag,Standard_True);
            const auto n=ReadLabel(child,end,depth+1);
            if(n<0 || n>INT32_MAX-count)throw Refusal{};count+=n;
        }
        if(Position()!=end)throw Refusal{};return count;
    }
    const Handle(BinMDF_ADriverTable) table_;
    const Standard_Integer typeID_;
    const Handle(FrameDriver) receipt_;
    const TraversalLimits limits_;
    const std::function<void()> reject_;
    Handle(Standard_Type) targetType_;
    Standard_GUID targetID_;
    TCollection_AsciiString typeName_;
    std::shared_ptr<LoadBudget> budget_;
    std::set<Standard_Integer,Less> ids_;
    std::istream* source_=nullptr;
    BinObjMgt_RRelocationTable* relocation_=nullptr;
    Message_ProgressScope* progress_=nullptr;
    std::uint64_t fileEnd_=0,labels_=0,attributes_=0,steps_=0,buffered_=0,receiptCount_=0;
    bool inverse_=false,started_=false,complete_=false,cancelled_=false;
};
} // namespace core3d::persistence::receipt_framing
