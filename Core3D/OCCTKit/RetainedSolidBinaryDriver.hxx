#pragma once
#include "RetainedSolidAttribute.hxx"
#include <BinMNaming_NamedShapeDriver.hxx>
#include <BinMXCAFDoc_LengthUnitDriver.hxx>
#include <XCAFDoc_LengthUnit.hxx>
#include <BinMDF_ADriverTable.hxx>
#include <BinTools_LocationSet.hxx>
#include <BinTools_ShapeSet.hxx>
#include <BinTools_ShapeReader.hxx>
#include <BinTools_ShapeWriter.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <TDocStd_FormatVersion.hxx>
#include <climits>
#include <sstream>

Standard_Boolean Core3DValidateRetainedSolidDocument(const Handle(TDocStd_Document)& document);

namespace core3d::retained_solid {
static_assert(sizeof(Standard_Integer)==4,"Native attribute prefix width changed");
class BinaryDriver final:public BinMDF_ADriver {
public:
    BinaryDriver(const Handle(Message_Messenger)& messenger,
        Handle(BinMNaming_NamedShapeDriver) shapes,std::shared_ptr<ReadBudget> budget,
        void (*reject)() noexcept=nullptr)
        :BinMDF_ADriver(messenger,STANDARD_TYPE(Attribute)->Name()),shapes_(std::move(shapes)),budget_(std::move(budget)),reject_(reject){}
    Handle(TDF_Attribute) NewEmpty()const override{return new Attribute();}
    const Handle(Standard_Type)& SourceType()const override{return STANDARD_TYPE(Attribute);}
    Standard_Boolean Paste(const BinObjMgt_Persistent& source,const Handle(TDF_Attribute)& target,
                           BinObjMgt_RRelocationTable& relocation)const override{
        try{
            auto attribute=Handle(Attribute)::DownCast(target);
            if(attribute.IsNull()||attribute->value_||shapes_.IsNull()||!budget_||budget_->rejected
                ||relocation.GetHeaderData().IsNull())return Refuse();
            const auto version=relocation.GetHeaderData()->StorageVersion();
            if(!version.IsIntegerValue()||version.IntegerValue()<TDocStd_FormatVersion_VERSION_10
                ||version.IntegerValue()>TDocStd_FormatVersion_CURRENT)return Refuse();
            const auto start=source.Position();const auto length=source.Length();
            if(length<0||length>INT_MAX-12||start<12||start>length+12)return Refuse();
            const auto end=length+12;
            Standard_Integer schema=0,count=0,mode=-1;
            if(!(source>>schema>>count>>mode)||schema!=1||count<=0||count>Standard_Integer(MaximumEnvelopeBytes)
                ||(mode!=0&&mode!=1)||mode!=int(shapes_->IsQuickPart())
                ||bool(mode)!=bool(const_cast<BinObjMgt_Persistent&>(source).IsDirect())
                ||source.Position()>end||count>end-source.Position()
                ||budget_->limit>MaximumAggregateEnvelopeBytes||budget_->envelopeBytes>budget_->limit
                ||std::size_t(count)>budget_->limit-budget_->envelopeBytes
                ||budget_->records>=std::size_t(profile::MaximumRecords)/2)return Refuse();
            std::vector<std::uint8_t> bytes(std::size_t(count),0);
            if(!source.GetByteArray(bytes.data(),count))return Refuse();
            retained_boolean::Recipe envelope;if(!retained_boolean::Decode(bytes,envelope))return Refuse();
            TopoDS_Shape base;
            // This exact shared instance is owned by the enclosing native
            // document driver. Never instantiate a second shape reader here.
            auto* shared=shapes_->ShapeSet(Standard_True);if(!shared)return Refuse();
            if(mode==1){
                if(!dynamic_cast<BinTools_ShapeReader*>(shared)||source.Position()!=end)return Refuse();
                auto* stream=const_cast<BinObjMgt_Persistent&>(source).GetIStream();if(!stream||!*stream)return Refuse();
                const auto body=stream->tellg();if(body==std::streampos(-1)||body<std::streampos(8))return Refuse();
                const auto begin=body-std::streamoff(8);stream->seekg(0,std::ios::end);const auto fileEnd=stream->tellg();
                if(fileEnd==std::streampos(-1)||fileEnd<body)return Refuse();stream->seekg(begin);
                std::array<std::uint8_t,8> size{};stream->read(reinterpret_cast<char*>(size.data()),8);if(!*stream)return Refuse();
                // Query actual linked OCCT wire order, as the existing direct
                // traversal does. No guessed host-endian or geometry parser.
                BinObjMgt_Persistent probe;probe.SetTypeId(0x01020304);probe.SetId(1);
                std::ostringstream order(std::ios::out|std::ios::binary);probe.Write(order);const auto marker=order.str();
                if(!order||marker.size()!=12)return Refuse();
                const bool little=std::memcmp(marker.data(),"\4\3\2\1",4)==0;
                const bool big=std::memcmp(marker.data(),"\1\2\3\4",4)==0;
                if(!little&&!big)return Refuse();std::uint64_t extent=0;
                for(unsigned i=0;i<8;++i)extent|=std::uint64_t(size[i])<<(8*(little?i:7-i));
                if(extent<8||extent>std::uint64_t(std::streamoff(fileEnd-begin)))return Refuse();
                const auto expected=begin+std::streamoff(extent);stream->seekg(body);
                shared->Read(*stream,base);if(!*stream||stream->tellg()!=expected)return Refuse();
            }else{
                Standard_Integer shapeID=0,locationID=-1,orientation=-1;
                if(!(source>>shapeID>>locationID>>orientation)||source.Position()!=end)return Refuse();
                auto* set=dynamic_cast<BinTools_ShapeSet*>(shared);
                if(!set||shapeID<=0||shapeID>set->NbShapes()||locationID<0
                    ||locationID>set->Locations().NbLocations()
                    ||orientation<int(TopAbs_FORWARD)||orientation>int(TopAbs_EXTERNAL))return Refuse();
                base=set->Shape(shapeID);
                base.Location(set->Locations().Location(locationID),Standard_False);
                base.Orientation(TopAbs_Orientation(orientation));
            }
            if(base.IsNull()||base.ShapeType()!=TopAbs_SOLID||base.Orientation()!=TopAbs_FORWARD)return Refuse();
            auto value=std::make_shared<Payload>();value->envelope=std::move(envelope);
            value->bytes=std::move(bytes);value->base=std::move(base);
            // Publish only after complete envelope and actual shared-codec read.
            // Document owner/topology admission runs before document adoption.
            attribute->value_=std::move(value);budget_->envelopeBytes+=std::size_t(count);++budget_->records;
            return Standard_True;
        }catch(...){return Refuse();}
    }
    void Paste(const Handle(TDF_Attribute)& source,BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable& relocation)const override{
        const auto attribute=Handle(Attribute)::DownCast(source);
        if(attribute.IsNull()||!attribute->value_||attribute->Label().IsNull()||shapes_.IsNull()
            ||TDocStd_Document::Get(attribute->Label()).IsNull()
            ||TDocStd_Document::Get(attribute->Label())->StorageFormatVersion()<TDocStd_FormatVersion_VERSION_10
            ||TDocStd_Document::Get(attribute->Label())->StorageFormatVersion()>TDocStd_FormatVersion_CURRENT)
            Standard_Failure::Raise("Retained solid writer source/version");
        const auto& value=*attribute->value_;std::vector<std::uint8_t> encoded;
        if(!retained_boolean::Encode(value.envelope,encoded)||encoded!=value.bytes||value.base.IsNull()
            ||value.base.ShapeType()!=TopAbs_SOLID||value.base.Orientation()!=TopAbs_FORWARD)
            Standard_Failure::Raise("Retained solid writer envelope/shape");
        target<<Standard_Integer(1)<<Standard_Integer(encoded.size())<<Standard_Integer(shapes_->IsQuickPart()?1:0);
        target.PutByteArray(encoded.data(),Standard_Integer(encoded.size()));
        auto* shared=shapes_->ShapeSet(Standard_False);if(!shared)Standard_Failure::Raise("Retained solid writer codec");
        if(shapes_->IsQuickPart()){
            if(!dynamic_cast<BinTools_ShapeWriter*>(shared))Standard_Failure::Raise("Retained solid stale shared writer mode");
            auto* stream=target.GetOStream();if(!stream)Standard_Failure::Raise("Retained solid writer stream");
            shared->Write(value.base,*stream);if(!*stream)Standard_Failure::Raise("Retained solid writer shape");
        }else{
            auto* set=dynamic_cast<BinTools_ShapeSet*>(shared);if(!set)Standard_Failure::Raise("Retained solid writer set");
            const auto id=set->Add(value.base);const auto location=set->Locations().Index(value.base.Location());
            if(id<=0||location<0)Standard_Failure::Raise("Retained solid writer reference");
            target<<id<<location<<Standard_Integer(value.base.Orientation());
        }
    }
private:
    Standard_Boolean Refuse()const noexcept{if(budget_)budget_->rejected=true;if(reject_)reject_();return Standard_False;}
    const Handle(BinMNaming_NamedShapeDriver) shapes_;
    const std::shared_ptr<ReadBudget> budget_;
    void (*reject_)() noexcept;
};
inline Handle(BinMDF_ADriver) Assigned(const Handle(BinMDF_ADriverTable)& table){
    Handle(BinMDF_ADriver) driver;
    return table->GetDriver(STANDARD_TYPE(Attribute),driver)>0?driver:Handle(BinMDF_ADriver)();
}
// Register after the native base table, obtaining the same NamedShape instance.
// A replaced/missing source driver is a configuration error, never a fallback.
inline void Register(const Handle(BinMDF_ADriverTable)& table,const Handle(Message_Messenger)& messenger,
                     const std::shared_ptr<ReadBudget>& budget,void (*reject)() noexcept=nullptr){
    Handle(BinMDF_ADriver) found;table->GetDriver(STANDARD_TYPE(TNaming_NamedShape),found);
    const auto shapes=Handle(BinMNaming_NamedShapeDriver)::DownCast(found);
    if(shapes.IsNull())Standard_Failure::Raise("Retained solid shared driver missing");
    table->AddDriver(new BinaryDriver(messenger,shapes,budget,reject));
}
template<class Base>class StorageDriver:public Base {
public:
    Handle(BinMDF_ADriverTable) AttributeDrivers(const Handle(Message_Messenger)& messenger)override{
        auto table=Base::AttributeDrivers(messenger);
        // BinOcaf's base table omits this authoritative XCAF document unit.
        // Add only the missing scalar writer; retain the existing XCAF driver
        // and exact shared NamedShape/Location instances in either format.
        Handle(BinMDF_ADriver) unitDriver;
        table->GetDriver(STANDARD_TYPE(XCAFDoc_LengthUnit),unitDriver);
        if(unitDriver.IsNull())table->AddDriver(new BinMXCAFDoc_LengthUnitDriver(messenger));
        Register(table,messenger,std::make_shared<ReadBudget>());return table;
    }
    void Write(const Handle(CDM_Document)& doc,const TCollection_ExtendedString& file,
               const Message_ProgressRange& progress=Message_ProgressRange())override{
        Prepare(doc);Base::Write(doc,file,progress);
    }
    void Write(const Handle(CDM_Document)& doc,Standard_OStream& stream,
               const Message_ProgressRange& progress=Message_ProgressRange())override{
        Prepare(doc);Base::Write(doc,stream,progress);
    }
private:
    static void Prepare(const Handle(CDM_Document)& value){
        const auto doc=Handle(TDocStd_Document)::DownCast(value);std::vector<Record> records;
        if(!ReadAll(doc,records))Standard_Failure::Raise("Retained solid writer role");
        if(!records.empty()&&!Core3DValidateRetainedSolidDocument(doc))
            Standard_Failure::Raise("Retained solid writer owner/aggregate geometry");
    }
};
} // namespace core3d::retained_solid
