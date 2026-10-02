#pragma once
#include "RetainedEdgeTreatmentDefinition.hxx"
#include "RetainedEdgeTreatmentR2Values.hxx"
#include "RetainedEdgeTreatmentStorageKey.hxx"
#include <TDF_Attribute.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDocStd_Document.hxx>
#include <TNaming_NamedShape.hxx>
#include <TNaming_Tool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TopoDS_Shape.hxx>
#include <memory>

namespace core3d::retained_edge_treatment {
struct Payload final { Definition definition; std::vector<std::uint8_t> bytes; TopoDS_Shape base; };
struct PayloadR2 final { r2::Definition definition; std::vector<std::uint8_t> bytes; TopoDS_Shape base; };
class BinaryDriver;
class Core3D_RetainedEdgeTreatment final:public TDF_Attribute {
public:
    const Standard_GUID& ID() const override{return AttributeID();}
    Handle(TDF_Attribute) NewEmpty() const override{return new Core3D_RetainedEdgeTreatment;}
    void Restore(const Handle(TDF_Attribute)& other) override{auto source=Handle(Core3D_RetainedEdgeTreatment)::DownCast(other);value_=source->value_;valueR2_=source->valueR2_;}
    void Paste(const Handle(TDF_Attribute)& into,const Handle(TDF_RelocationTable)&) const override{auto target=Handle(Core3D_RetainedEdgeTreatment)::DownCast(into);target->value_=value_;target->valueR2_=valueR2_;}
    const std::shared_ptr<const Payload>& value() const noexcept{return value_;}
    const std::shared_ptr<const PayloadR2>& valueR2() const noexcept{return valueR2_;}
    static Handle(Core3D_RetainedEdgeTreatment) Set(const TDF_Label& label,std::shared_ptr<const Payload> value){Handle(Core3D_RetainedEdgeTreatment)a; if(!label.FindAttribute(AttributeID(),a)){a=new Core3D_RetainedEdgeTreatment;label.AddAttribute(a);}a->Backup();a->valueR2_.reset();a->value_=std::move(value);return a;}
    static Handle(Core3D_RetainedEdgeTreatment) SetR2(const TDF_Label& label,std::shared_ptr<const PayloadR2> value){Handle(Core3D_RetainedEdgeTreatment)a; if(!label.FindAttribute(AttributeID(),a)){a=new Core3D_RetainedEdgeTreatment;label.AddAttribute(a);}a->Backup();a->value_.reset();a->valueR2_=std::move(value);return a;}
private: friend class BinaryDriver; std::shared_ptr<const Payload> value_;std::shared_ptr<const PayloadR2> valueR2_;
};
using Attribute=Core3D_RetainedEdgeTreatment;
struct Record { TDF_Label label,owner;std::shared_ptr<const Payload> value;TopoDS_Shape current;bool IsCurrent(const Handle(TDocStd_Document)& document,const TDF_Label& expected) const noexcept{return !document.IsNull()&&!label.IsNull()&&!owner.IsNull()&&owner.IsEqual(expected)&&owner.Data()==document->GetData()&&label.Father().IsEqual(owner)&&value&&!current.IsNull()&&current.IsEqual(XCAFDoc_ShapeTool::GetShape(owner));} };
struct RecordR2 { TDF_Label label,owner;std::shared_ptr<const PayloadR2> value;TopoDS_Shape current;bool IsCurrent(const Handle(TDocStd_Document)& document,const TDF_Label& expected) const noexcept{return !document.IsNull()&&!label.IsNull()&&!owner.IsNull()&&owner.IsEqual(expected)&&owner.Data()==document->GetData()&&label.Father().IsEqual(owner)&&value&&!current.IsNull()&&current.IsEqual(XCAFDoc_ShapeTool::GetShape(owner));} };
inline bool Read(const Handle(TDocStd_Document)& document,const TDF_Label& owner,std::optional<Record>& output,Refusal& refusal) noexcept {output.reset();try{if(document.IsNull()||owner.IsNull()||owner.Data()!=document->GetData()){refusal=Refusal::IdentityMismatch;return false;}Handle(Attribute) found;TDF_Label foundLabel;std::size_t count=0;for(TDF_ChildIterator it(owner);it.More();it.Next()){Handle(Attribute)a;if(it.Value().FindAttribute(AttributeID(),a)){found=a;foundLabel=it.Value();++count;}}if(count==0){refusal=Refusal::None;return true;}if(count!=1||found.IsNull()||!found->value()||foundLabel.Father()!=owner){refusal=Refusal::MalformedCarrier;return false;}std::optional<Definition> decoded;if(!Decode(found->value()->bytes,decoded,refusal)||!decoded||!(decoded->owner==found->value()->definition.owner)){refusal=Refusal::MalformedCarrier;return false;}Handle(TNaming_NamedShape) named;if(!foundLabel.FindAttribute(TNaming_NamedShape::GetID(),named)){refusal=Refusal::MalformedCarrier;return false;}Record record{foundLabel,owner,found->value(),TNaming_Tool::GetShape(named)};if(!record.IsCurrent(document,owner)){refusal=Refusal::NoncurrentSource;return false;}output=std::move(record);refusal=Refusal::None;return true;}catch(...){output.reset();refusal=Refusal::MalformedCarrier;return false;}}
inline bool ReadAll(const Handle(TDocStd_Document)& document,std::size_t priorBytes,std::vector<Record>& output,Refusal& refusal) noexcept {output.clear();try{if(document.IsNull()){refusal=Refusal::MalformedCarrier;return false;}std::size_t total=priorBytes;for(TDF_ChildIterator it(document->Main(),Standard_True);it.More();it.Next()){Handle(Attribute)a;if(!it.Value().FindAttribute(AttributeID(),a))continue;if(it.Value().Father().IsNull()||!a->value()){refusal=Refusal::MalformedCarrier;return false;}std::optional<Record> one;if(!Read(document,it.Value().Father(),one,refusal)||!one)return false;if(one->label!=it.Value()){refusal=Refusal::MultipleOwners;return false;}if(one->value->bytes.size()>8388608-total){refusal=Refusal::Budget;return false;}total+=one->value->bytes.size();output.push_back(*one);}refusal=Refusal::None;return true;}catch(...){output.clear();refusal=Refusal::MalformedCarrier;return false;}}

inline bool ReadR2(const Handle(TDocStd_Document)& document,const TDF_Label& owner,
    std::optional<RecordR2>& output,Refusal& refusal) noexcept {
    output.reset();
    try {
        if(document.IsNull()||owner.IsNull()||owner.Data()!=document->GetData()){refusal=Refusal::IdentityMismatch;return false;}
        Handle(Attribute) found;TDF_Label foundLabel;std::size_t count=0;
        for(TDF_ChildIterator it(owner);it.More();it.Next()){Handle(Attribute) a;if(it.Value().FindAttribute(AttributeID(),a)){found=a;foundLabel=it.Value();++count;}}
        if(count==0){refusal=Refusal::None;return true;}
        if(count!=1||found.IsNull()||found->value()||!found->valueR2()||!foundLabel.Father().IsEqual(owner)){refusal=Refusal::MalformedCarrier;return false;}
        std::optional<r2::Definition> decoded;
        if(!r2::Decode(found->valueR2()->bytes,decoded,refusal)||!decoded
            ||!(decoded->owner==found->valueR2()->definition.owner)){refusal=Refusal::MalformedCarrier;return false;}
        Handle(TNaming_NamedShape) named;if(!foundLabel.FindAttribute(TNaming_NamedShape::GetID(),named)){refusal=Refusal::MalformedCarrier;return false;}
        RecordR2 record{foundLabel,owner,found->valueR2(),TNaming_Tool::GetShape(named)};
        if(!record.IsCurrent(document,owner)){refusal=Refusal::NoncurrentSource;return false;}
        output=std::move(record);refusal=Refusal::None;return true;
    }catch(...){output.reset();refusal=Refusal::MalformedCarrier;return false;}
}

// Strict mixed-arm whole-document traversal for save/import/export validation.
// Every treatment attribute must carry exactly one payload arm; that arm — and
// only that arm — is validated by its full native reader (B1 Read / R2 ReadR2,
// including canonical decode and the exact current naming binding), typed
// records of both families are retained, one treatment per owner is enforced,
// and B1 and R2 bytes accumulate into the same existing aggregate. Never
// turned into "skip R2", "has R2, return true", a !value() continue, or a
// schema dispatch: B2's schema-2 Q2 carrier lives on the B1 arm. The
// standalone B1 reader (ReadAll) is unchanged for its existing callers.
inline bool ReadAllMixed(const Handle(TDocStd_Document)& document,std::size_t priorBytes,
    std::vector<Record>& output,std::vector<RecordR2>& outputR2,Refusal& refusal) noexcept {
    output.clear();outputR2.clear();
    try{
        if(document.IsNull()){refusal=Refusal::MalformedCarrier;return false;}
        std::size_t total=priorBytes;
        for(TDF_ChildIterator it(document->Main(),Standard_True);it.More();it.Next()){
            Handle(Attribute) a;if(!it.Value().FindAttribute(AttributeID(),a))continue;
            if(it.Value().Father().IsNull()||a.IsNull()||bool(a->value())==bool(a->valueR2())){
                refusal=Refusal::MalformedCarrier;return false;
            }
            if(a->value()){
                std::optional<Record> one;
                if(!Read(document,it.Value().Father(),one,refusal)||!one)return false;
                if(one->label!=it.Value()){refusal=Refusal::MultipleOwners;return false;}
                if(one->value->bytes.size()>8388608-total){refusal=Refusal::Budget;return false;}
                total+=one->value->bytes.size();output.push_back(*one);
            }else{
                std::optional<RecordR2> one;
                if(!ReadR2(document,it.Value().Father(),one,refusal)||!one)return false;
                if(!one->label.IsEqual(it.Value())){refusal=Refusal::MultipleOwners;return false;}
                if(one->value->bytes.size()>8388608-total){refusal=Refusal::Budget;return false;}
                total+=one->value->bytes.size();outputR2.push_back(*one);
            }
        }
        refusal=Refusal::None;return true;
    }catch(...){output.clear();outputR2.clear();refusal=Refusal::MalformedCarrier;return false;}
}
} // namespace core3d::retained_edge_treatment
