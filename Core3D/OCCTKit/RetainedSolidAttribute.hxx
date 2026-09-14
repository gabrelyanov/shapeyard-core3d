#pragma once
#include "RetainedBooleanProgram.hxx"
namespace core3d::retained_solid {
struct Payload final {
    retained_boolean::Recipe envelope;
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
            if(!retained_boolean::Encode(value->envelope,exact)||exact!=value->bytes||value->base.IsNull()
                ||value->base.ShapeType()!=TopAbs_SOLID||value->base.Orientation()!=TopAbs_FORWARD
                ||total>MaximumAggregateEnvelopeBytes||exact.size()>MaximumAggregateEnvelopeBytes-total)return false;
            total+=exact.size();const auto identity=retained_boolean::Identities(value->envelope);UUID document,entity,definition;double unit=0;
            if(!ReadUUID(doc->Main(),Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"),document)
                ||!ReadUUID(owner,Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"),entity)
                ||!ReadUUID(owner,Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"),definition)
                ||document!=identity.document||entity!=identity.entity||definition!=identity.definition
                ||!XCAFDoc_DocumentTool::GetLengthUnit(doc,unit)||Bits(unit)!=Bits(identity.metersPerUnit)
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
