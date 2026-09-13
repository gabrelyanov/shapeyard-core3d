#pragma once
// One native retained solid, encoded through the DOCUMENT'S existing shape set.
// This foundation grants no ordinary edit, prepared-request or AI authority.
#include "RectangularLoftPersistence.hxx"
#include <TDF_Attribute.hxx>
#include <TDF_RelocationTable.hxx>
#include <CommonCrypto/CommonDigest.h>
#include <memory>
#include <array>
#include <cstring>
#include <vector>
#include <cstdint>
#include <TDF_LabelMap.hxx>
#include <Standard_Failure.hxx>

class OcctDocument;
namespace core3d::retained_solid {
class BinaryDriver;
#if DEBUG
struct Probe;
#endif
inline constexpr std::size_t MaximumEnvelopeBytes=64*1024;
inline constexpr std::size_t MaximumAggregateEnvelopeBytes=8*1024*1024;
inline constexpr int MinimumRecordTag=13;
inline const Standard_GUID& AttributeID(){
    static const Standard_GUID id("72997251-32AF-4771-B094-8D097B41B7D2");return id;
}
using UUID=std::array<std::uint8_t,16>;
using Digest=std::array<std::uint8_t,32>;
inline bool Nonzero(const UUID& id){return std::any_of(id.begin(),id.end(),[](auto v){return v!=0;});}
inline std::uint64_t Bits(double value){std::uint64_t result;std::memcpy(&result,&value,8);return result;}
struct Envelope {
    UUID document{},entity{},definition{},sourceFeature{},derivedFeature{};
    // First schema retains a profile/enclosure base and one cylindrical
    // through-all Difference operand, expressed on object-local X/Y/Z.
    std::uint8_t sourceFamily=0,axis=2;
    std::uint32_t sourceSchema=0,operandID=1;
    double metersPerUnit=0,radius=0;
    std::array<double,3> point{};
    std::vector<double> sourceValues;
};
inline bool Valid(const Envelope& e){
    if(!Nonzero(e.document)||!Nonzero(e.entity)||!Nonzero(e.definition)
        ||!Nonzero(e.sourceFeature)||!Nonzero(e.derivedFeature)||e.sourceFeature==e.derivedFeature
        ||e.axis>2||!e.operandID||!std::isfinite(e.metersPerUnit)||e.metersPerUnit<=0
        ||e.sourceValues.empty()||e.sourceValues.size()>std::size_t(profile::MaximumScalars))return false;
    const double mm=e.metersPerUnit*1000;
    if(!std::isfinite(mm)||mm<=0||!std::isfinite(e.radius)||e.radius<=0
        ||!std::isfinite(e.radius*mm)||e.radius*mm<.001||e.radius*mm>1e6)return false;
    for(double p:e.point)if(!std::isfinite(p)||!std::isfinite(p*mm)||std::abs(p*mm)>1e6)return false;
    for(double v:e.sourceValues)if(!std::isfinite(v))return false;
    if(e.sourceFamily==1){
        if(e.sourceSchema<1||e.sourceSchema>4)return false;
        profile::Parameters p;
        return profile::Decode(e.sourceValues,p)&&profile::SchemaFor(p)==int(e.sourceSchema)&&Bits(p.metersPerUnit)==Bits(e.metersPerUnit);
    }
    if(e.sourceFamily==2){
        if(e.sourceSchema<1||e.sourceSchema>2)return false;
        enclosure::Parameters p;
        return enclosure::Decode(int(e.sourceSchema),e.sourceValues,p)&&Bits(p.metersPerUnit)==Bits(e.metersPerUnit);
    }
    return false;
}
inline void U64(std::vector<std::uint8_t>& out,std::uint64_t value){for(unsigned i=0;i<8;++i)out.push_back(std::uint8_t(value>>(8*i)));}
inline bool Hash(const std::vector<std::uint8_t>& bytes,Digest& out){
    return bytes.size()<=MaximumEnvelopeBytes&&CC_SHA256(bytes.data(),CC_LONG(bytes.size()),out.data())!=nullptr;
}
inline bool Encode(const Envelope& e,std::vector<std::uint8_t>& out){
    out.clear();try{
        if(!Valid(e))return false;
        std::vector<std::uint8_t> b{'S','Y','R','S',1,1,1,1,e.sourceFamily,e.axis};
        for(const auto& id:{e.document,e.entity,e.definition,e.sourceFeature,e.derivedFeature})b.insert(b.end(),id.begin(),id.end());
        U64(b,e.sourceSchema);U64(b,e.operandID);U64(b,Bits(e.metersPerUnit));
        for(double p:e.point)U64(b,Bits(p));U64(b,Bits(e.radius));U64(b,e.sourceValues.size());
        for(double v:e.sourceValues)U64(b,Bits(v));
        if(b.size()>MaximumEnvelopeBytes-32)return false;Digest digest;if(!Hash(b,digest))return false;
        b.insert(b.end(),digest.begin(),digest.end());out=std::move(b);return true;
    }catch(...){out.clear();return false;}
}
inline bool Decode(const std::vector<std::uint8_t>& bytes,Envelope& out){
    out={};try{
        constexpr std::size_t prefix=10+80+8*8;
        if(bytes.size()<prefix+8+32||bytes.size()>MaximumEnvelopeBytes
            ||std::memcmp(bytes.data(),"SYRS\1\1\1\1",8)!=0)return false;
        std::size_t at=10;Envelope e;e.sourceFamily=bytes[8];e.axis=bytes[9];
        for(auto* id:{&e.document,&e.entity,&e.definition,&e.sourceFeature,&e.derivedFeature}){
            std::copy_n(bytes.begin()+at,16,id->begin());at+=16;
        }
        auto integer=[&](){std::uint64_t v=0;for(unsigned i=0;i<8;++i)v|=std::uint64_t(bytes[at++])<<(8*i);return v;};
        auto scalar=[&](){auto bits=integer();double v;std::memcpy(&v,&bits,8);return v;};
        const auto schema=integer(),operand=integer();
        if(schema>UINT32_MAX||operand>UINT32_MAX)return false;e.sourceSchema=std::uint32_t(schema);e.operandID=std::uint32_t(operand);
        e.metersPerUnit=scalar();for(double& p:e.point)p=scalar();e.radius=scalar();const auto count=integer();
        if(count>std::size_t(profile::MaximumScalars)||count>(bytes.size()-at-32)/8||bytes.size()!=at+8*count+32)return false;
        e.sourceValues.reserve(std::size_t(count));for(std::uint64_t i=0;i<count;++i)e.sourceValues.push_back(scalar());
        std::vector<std::uint8_t> expected;if(!Encode(e,expected)||expected!=bytes)return false;
        out=std::move(e);return true;
    }catch(...){out={};return false;}
}
struct Payload final {
    Envelope envelope;
    std::vector<std::uint8_t> bytes;
    TopoDS_Shape base;
};
// This is not TNaming_NamedShape: TNaming::Displace must leave this original
// object-local base untouched. Kernel workers must deep-copy before mutation.
class Core3D_RetainedSolid final:public TDF_Attribute {
public:
    DEFINE_STANDARD_RTTI_INLINE(Core3D_RetainedSolid,TDF_Attribute)
    const Standard_GUID& ID()const override{return AttributeID();}
    Handle(TDF_Attribute) NewEmpty()const override{return new Core3D_RetainedSolid();}
    const std::shared_ptr<const Payload>& value()const noexcept{return value_;}
    void Restore(const Handle(TDF_Attribute)& source)override{
        const auto original=Handle(Core3D_RetainedSolid)::DownCast(source);
        if(original.IsNull())Standard_Failure::Raise("Retained solid restore type");
        // Backup/Undo/Redo share immutable payload ownership, never copy or
        // reparse geometry. No native worker receives this shape for mutation.
        value_=original->value_;
    }
    void Paste(const Handle(TDF_Attribute)& target,const Handle(TDF_RelocationTable)&)const override{
        const auto destination=Handle(Core3D_RetainedSolid)::DownCast(target);
        if(destination.IsNull()||!value_)Standard_Failure::Raise("Retained solid paste type");
        // IDs remain exact on cross-document copy; owner validation rejects a
        // foreign document/entity instead of manufacturing a new identity.
        destination->Backup();destination->value_=value_;
    }
private:
    friend class BinaryDriver;
    friend class ::OcctDocument;
#if DEBUG
    friend struct Probe;
#endif
    std::shared_ptr<const Payload> value_;
};
using Attribute=Core3D_RetainedSolid;
inline bool ReadUUID(const TDF_Label& label,const Standard_GUID& id,UUID& out){
    out={};Handle(TDataStd_AsciiString) attribute;
    if(label.IsNull()||!label.FindAttribute(id,attribute)||attribute.IsNull()||attribute->Get().Length()!=36)return false;
    const std::string text=attribute->Get().ToCString();if(!profile::IsIdentifier(text))return false;
    std::size_t n=0;unsigned half=0;
    for(char c:text){if(c=='-')continue;const unsigned digit=c>='A'?unsigned(c-'A'+10):unsigned(c-'0');
        if(half++%2==0)out[n]=std::uint8_t(digit<<4);else out[n++]|=std::uint8_t(digit);}
    return n==16&&Nonzero(out);
}
inline std::string UUIDText(const UUID& id){
    constexpr char hex[]="0123456789ABCDEF";std::string text;
    for(std::size_t i=0;i<16;++i){if(i==4||i==6||i==8||i==10)text.push_back('-');text.push_back(hex[id[i]>>4]);text.push_back(hex[id[i]&15]);}return text;
}
struct Record {
    TDF_Label label,owner;
    std::shared_ptr<const Payload> value;
    TopoDS_Shape current;
    bool IsEqual(const Record& other)const noexcept {
        try {
            if(label.IsNull()||other.label.IsNull())return label.IsNull()&&other.label.IsNull()&&!value&&!other.value;
            return label.IsEqual(other.label)&&label.Data()==other.label.Data()
                &&owner.IsEqual(other.owner)&&owner.Data()==other.owner.Data()
                &&value&&other.value&&value->bytes==other.value->bytes
                &&value->base.IsEqual(other.value->base)&&current.IsEqual(other.current);
        }catch(...){return false;}
    }
};
inline bool HasRecord(const TDF_Label& owner)noexcept{
    try {
    if(owner.IsNull())return false;
    if(owner.IsAttribute(AttributeID()))return true;
    int visited=0;for(TDF_ChildIterator it(owner,Standard_False);it.More();it.Next()){
        if(++visited>profile::MaximumLabels||it.Value().IsAttribute(AttributeID()))return true;
    }return false;
    }catch(...){return true;} // Unknown metadata cannot become bare-solid admission.
}
inline bool ReadAll(const Handle(TDocStd_Document)& doc,std::vector<Record>& out){
    out.clear();try{
        std::vector<Record> staged;
        if(doc.IsNull()||doc->GetData().IsNull())return false;
        const auto root=doc->GetData()->Root();if(root.IsAttribute(AttributeID()))return false;
        int visited=0;std::size_t total=0;TDF_LabelMap owners;
        for(TDF_ChildIterator it(root,Standard_True);it.More();it.Next()){
            if(++visited>profile::MaximumLabels)return false;
            const auto label=it.Value();if(!label.IsAttribute(AttributeID()))continue;
            Handle(Attribute) attribute;Handle(TNaming_NamedShape) current;
            const auto owner=label.Father();
            if(label.Tag()<MinimumRecordTag||!label.FindAttribute(AttributeID(),attribute)||attribute.IsNull()
                ||!attribute->value()||!label.FindAttribute(TNaming_NamedShape::GetID(),current)
                ||!XCAFDoc_ShapeTool::IsSimpleShape(owner)||!XCAFDoc_ShapeTool::IsFree(owner)
                ||!owners.Add(owner)||staged.size()>=std::size_t(profile::MaximumRecords)/2)return false;
            const auto value=attribute->value();std::vector<std::uint8_t> exact;
            if(!Encode(value->envelope,exact)||exact!=value->bytes||value->base.IsNull()
                ||value->base.ShapeType()!=TopAbs_SOLID||value->base.Orientation()!=TopAbs_FORWARD
                ||total>MaximumAggregateEnvelopeBytes||exact.size()>MaximumAggregateEnvelopeBytes-total)return false;
            total+=exact.size();UUID document,entity,definition;double unit=0;
            if(!ReadUUID(doc->Main(),Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"),document)
                ||!ReadUUID(owner,Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"),entity)
                ||!ReadUUID(owner,Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"),definition)
                ||document!=value->envelope.document||entity!=value->envelope.entity||definition!=value->envelope.definition
                ||!XCAFDoc_DocumentTool::GetLengthUnit(doc,unit)||Bits(unit)!=Bits(value->envelope.metersPerUnit)
                ||current->Get().IsNull()||!current->Get().IsEqual(XCAFDoc_ShapeTool::GetShape(owner)))return false;
            for(TDF_AttributeIterator a(label);a.More();a.Next())
                if(a.Value()->ID()!=AttributeID()&&a.Value()->ID()!=TNaming_NamedShape::GetID())return false;
            int nested=0;for(TDF_ChildIterator c(label,Standard_True);c.More();c.Next())
                if(++nested>profile::MaximumLabels||c.Value().HasAttribute())return false;
            for(TDF_ChildIterator c(owner,Standard_False);c.More();c.Next())
                if(profile::HasAttribute(c.Value())||enclosure::HasAttribute(c.Value())
                    ||sweep_persistence::HasAttribute(c.Value())||loft_persistence::HasAttribute(c.Value()))return false;
            staged.push_back({label,owner,value,current->Get()});
        }out=std::move(staged);return true;
    }catch(...){out.clear();return false;}
}
inline bool Read(const Handle(TDocStd_Document)& doc,const TDF_Label& owner,Record& out){
    out={};std::vector<Record> all;
    if(owner.IsNull()||doc.IsNull()||owner.Data()!=doc->GetData())return false;
    // Local absence matches the existing family-reader contract. Whole-document
    // admission remains mandatory at cut capture/scene capture and native load.
    // Do not traverse the full document once per unrelated unparametrized root.
    if(!HasRecord(owner))return true;
    if(!ReadAll(doc,all))return false;
    for(const auto& row:all)if(row.owner.IsEqual(owner)){out=row;return true;}
    return false; // A direct carrier marker must resolve to this exact owner.
}
struct ReadBudget {
    std::size_t envelopeBytes=0,records=0;
    std::size_t limit=MaximumAggregateEnvelopeBytes;
    bool rejected=false;
    void reset(){envelopeBytes=0;records=0;rejected=false;}
};
} // namespace core3d::retained_solid
