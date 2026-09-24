#pragma once
#include "CompositeRecipeAttribute.hxx"
#include <BinMDF_ADriverTable.hxx>
#include <BinMNaming_NamedShapeDriver.hxx>
#include <BinMXCAFDoc_LengthUnitDriver.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <BinTools_LocationSet.hxx>
#include <BinTools_ShapeReader.hxx>
#include <BinTools_ShapeSet.hxx>
#include <BinTools_ShapeWriter.hxx>
#include <TDocStd_FormatVersion.hxx>
#include <XCAFDoc_LengthUnit.hxx>
#include <climits>
#include <sstream>

Standard_Boolean Core3DValidateCompositeRecipeDocument(const Handle(TDocStd_Document)& document);

namespace core3d::composite_recipe {
static_assert(sizeof(Standard_Integer) == 4, "Native attribute prefix width changed");

class BinaryDriver final : public BinMDF_ADriver {
public:
    BinaryDriver(const Handle(Message_Messenger)& messenger,
                 Handle(BinMNaming_NamedShapeDriver) shapes,
                 std::shared_ptr<ReadBudget> budget,
                 void (*reject)() noexcept = nullptr)
        : BinMDF_ADriver(messenger, STANDARD_TYPE(Attribute)->Name()),
          shapes_(std::move(shapes)), budget_(std::move(budget)), reject_(reject) {}
    Handle(TDF_Attribute) NewEmpty() const override { return new Attribute(); }
    const Handle(Standard_Type)& SourceType() const override { return STANDARD_TYPE(Attribute); }

    Standard_Boolean Paste(const BinObjMgt_Persistent& source,
                           const Handle(TDF_Attribute)& target,
                           BinObjMgt_RRelocationTable& relocation) const override {
        try {
            const auto attribute = Handle(Attribute)::DownCast(target);
            if (attribute.IsNull() || attribute->value_ || shapes_.IsNull() || !budget_
                || budget_->rejected || relocation.GetHeaderData().IsNull()) return Refuse();
            const auto version = relocation.GetHeaderData()->StorageVersion();
            if (!version.IsIntegerValue() || version.IntegerValue() < TDocStd_FormatVersion_VERSION_10
                || version.IntegerValue() > TDocStd_FormatVersion_CURRENT) return Refuse();
            const auto start = source.Position(), length = source.Length();
            if (length < 0 || length > INT_MAX - 12 || start < 12 || start > length + 12) return Refuse();
            const auto end = length + 12;
            Standard_Integer schema = 0, count = 0, mode = -1, shapeCount = 0;
            if (!(source >> schema >> count >> mode >> shapeCount) || schema != 1
                || count <= 0 || count > Standard_Integer(MaximumEnvelopeBytes)
                || shapeCount <= 0 || shapeCount > Standard_Integer(MaximumSourceNodes)
                || (mode != 0 && mode != 1) || mode != int(shapes_->IsQuickPart())
                || bool(mode) != bool(const_cast<BinObjMgt_Persistent&>(source).IsDirect())
                || budget_->limit > MaximumDocumentAggregateBytes
                || budget_->envelopeBytes > budget_->limit
                || std::size_t(count) > budget_->limit - budget_->envelopeBytes
                || budget_->records >= std::size_t(profile::MaximumRecords) / 2) return Refuse();
            const Standard_Integer fixedTail = mode == 0 ? 12 * shapeCount : 0;
            if (source.Position() > end || count > end - source.Position() - fixedTail) return Refuse();
            std::vector<std::uint8_t> bytes(std::size_t(count), 0);
            if (!source.GetByteArray(bytes.data(), count)) return Refuse();
            Definition definition; if (!Decode(bytes, definition)) return Refuse();
            std::size_t expectedSources = 0;
            std::vector<TopAbs_ShapeEnum> expectedShapeTypes(std::size_t(shapeCount), TopAbs_SHAPE);
            for (const Node& node : definition.nodes)
                if (const auto* sourceNode = std::get_if<SourceNode>(&node.value)) {
                    ++expectedSources;
                    if (sourceNode->shapeSlot >= expectedShapeTypes.size()
                        || expectedShapeTypes[sourceNode->shapeSlot] != TopAbs_SHAPE) return Refuse();
                    expectedShapeTypes[sourceNode->shapeSlot] =
                        TopologyKind(ExpectedSourceShapeKind(sourceNode->recipe));
                }
            if (expectedSources != std::size_t(shapeCount)) return Refuse();
            auto* shared = shapes_->ShapeSet(Standard_True); if (!shared) return Refuse();
            std::vector<TopoDS_Shape> sourceShapes; sourceShapes.reserve(std::size_t(shapeCount));
            if (mode == 1) {
                if (!dynamic_cast<BinTools_ShapeReader*>(shared) || source.Position() != end) return Refuse();
                auto* stream = const_cast<BinObjMgt_Persistent&>(source).GetIStream();
                if (!stream || !*stream) return Refuse();
                BinObjMgt_Persistent probe; probe.SetTypeId(0x01020304); probe.SetId(1);
                std::ostringstream order(std::ios::out|std::ios::binary); probe.Write(order);
                const auto marker=order.str();
                if(!order||marker.size()!=12)return Refuse();
                const bool little=std::memcmp(marker.data(),"\4\3\2\1",4)==0;
                const bool big=std::memcmp(marker.data(),"\1\2\3\4",4)==0;
                if(!little&&!big)return Refuse();
                for (Standard_Integer index = 0; index < shapeCount; ++index) {
                    TopoDS_Shape shape;
                    const auto cursor=stream->tellg();if(cursor==std::streampos(-1))return Refuse();
                    const auto begin=index==0?cursor-std::streamoff(8):cursor;
                    if(begin<std::streampos(0))return Refuse();
                    std::array<std::uint8_t,8> framing{};
                    stream->seekg(begin);stream->read(reinterpret_cast<char*>(framing.data()),8);
                    if(!*stream)return Refuse();
                    const auto body=stream->tellg();stream->seekg(0,std::ios::end);const auto fileEnd=stream->tellg();
                    if(fileEnd==std::streampos(-1)||fileEnd<body)return Refuse();
                    std::uint64_t extent=0;for(unsigned byte=0;byte<8;++byte)
                        extent|=std::uint64_t(framing[byte])<<(8*(little?byte:7-byte));
                    if(extent<8||extent>std::uint64_t(std::streamoff(fileEnd-begin)))return Refuse();
                    const auto expected=begin+std::streamoff(extent);stream->seekg(body);
                    shared->Read(*stream, shape);
                    if (!*stream || stream->tellg()!=expected || shape.IsNull()
                        || shape.ShapeType() != expectedShapeTypes[std::size_t(index)]
                        || shape.Orientation() != TopAbs_FORWARD) return Refuse();
                    sourceShapes.push_back(std::move(shape));
                }
            } else {
                auto* set = dynamic_cast<BinTools_ShapeSet*>(shared); if (!set) return Refuse();
                for (Standard_Integer index = 0; index < shapeCount; ++index) {
                    Standard_Integer shapeID = 0, locationID = -1, orientation = -1;
                    if (!(source >> shapeID >> locationID >> orientation)
                        || shapeID <= 0 || shapeID > set->NbShapes() || locationID < 0
                        || locationID > set->Locations().NbLocations()
                        || orientation < int(TopAbs_FORWARD) || orientation > int(TopAbs_EXTERNAL)) return Refuse();
                    TopoDS_Shape shape = set->Shape(shapeID);
                    shape.Location(set->Locations().Location(locationID), Standard_False);
                    shape.Orientation(TopAbs_Orientation(orientation));
                    if (shape.IsNull()
                        || shape.ShapeType() != expectedShapeTypes[std::size_t(index)]
                        || shape.Orientation() != TopAbs_FORWARD) return Refuse();
                    sourceShapes.push_back(std::move(shape));
                }
                if (source.Position() != end) return Refuse();
            }
            auto value = std::make_shared<Payload>(); value->definition = std::move(definition);
            value->bytes = std::move(bytes); value->sourceShapes = std::move(sourceShapes);
            attribute->value_ = std::move(value); budget_->envelopeBytes += std::size_t(count); ++budget_->records;
            return Standard_True;
        } catch (...) { return Refuse(); }
    }

    void Paste(const Handle(TDF_Attribute)& source, BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable&) const override {
        const auto attribute = Handle(Attribute)::DownCast(source);
        if (attribute.IsNull() || !attribute->value_ || attribute->Label().IsNull() || shapes_.IsNull())
            Standard_Failure::Raise("Composite recipe writer source");
        const auto document = TDocStd_Document::Get(attribute->Label());
        if (document.IsNull() || document->StorageFormatVersion() < TDocStd_FormatVersion_VERSION_10
            || document->StorageFormatVersion() > TDocStd_FormatVersion_CURRENT)
            Standard_Failure::Raise("Composite recipe writer version");
        const auto& value = *attribute->value_; std::vector<std::uint8_t> encoded;
        if (!Encode(value.definition, encoded) || encoded != value.bytes || value.sourceShapes.empty()
            || value.sourceShapes.size() > MaximumSourceNodes)
            Standard_Failure::Raise("Composite recipe writer envelope");
        std::vector<TopAbs_ShapeEnum> expectedShapeTypes(value.sourceShapes.size(), TopAbs_SHAPE);
        for (const Node& node : value.definition.nodes)
            if (const auto* sourceNode = std::get_if<SourceNode>(&node.value)) {
                if (sourceNode->shapeSlot >= expectedShapeTypes.size()
                    || expectedShapeTypes[sourceNode->shapeSlot] != TopAbs_SHAPE)
                    Standard_Failure::Raise("Composite recipe writer shape slot");
                expectedShapeTypes[sourceNode->shapeSlot] =
                    TopologyKind(ExpectedSourceShapeKind(sourceNode->recipe));
            }
        for (std::size_t index = 0; index < value.sourceShapes.size(); ++index) {
            const TopoDS_Shape& shape = value.sourceShapes[index];
            if (shape.IsNull() || shape.ShapeType() != expectedShapeTypes[index]
                || shape.Orientation() != TopAbs_FORWARD)
                Standard_Failure::Raise("Composite recipe writer shape");
        }
        target << Standard_Integer(1) << Standard_Integer(encoded.size())
               << Standard_Integer(shapes_->IsQuickPart() ? 1 : 0)
               << Standard_Integer(value.sourceShapes.size());
        target.PutByteArray(encoded.data(), Standard_Integer(encoded.size()));
        auto* shared = shapes_->ShapeSet(Standard_False);
        if (!shared) Standard_Failure::Raise("Composite recipe writer codec");
        if (shapes_->IsQuickPart()) {
            if (!dynamic_cast<BinTools_ShapeWriter*>(shared))
                Standard_Failure::Raise("Composite recipe stale shared writer mode");
            auto* stream = target.GetOStream(); if (!stream) Standard_Failure::Raise("Composite recipe stream");
            for (const TopoDS_Shape& shape : value.sourceShapes) shared->Write(shape, *stream);
            if (!*stream) Standard_Failure::Raise("Composite recipe writer shape");
        } else {
            auto* set = dynamic_cast<BinTools_ShapeSet*>(shared);
            if (!set) Standard_Failure::Raise("Composite recipe writer set");
            for (const TopoDS_Shape& shape : value.sourceShapes) {
                const auto id = set->Add(shape), location = set->Locations().Index(shape.Location());
                if (id <= 0 || location < 0) Standard_Failure::Raise("Composite recipe writer reference");
                target << id << location << Standard_Integer(shape.Orientation());
            }
        }
    }
private:
    Standard_Boolean Refuse() const noexcept {
        if (budget_) budget_->rejected = true; if (reject_) reject_(); return Standard_False;
    }
    const Handle(BinMNaming_NamedShapeDriver) shapes_;
    const std::shared_ptr<ReadBudget> budget_;
    void (*reject_)() noexcept;
};

inline void Register(const Handle(BinMDF_ADriverTable)& table,
                     const Handle(Message_Messenger)& messenger,
                     const std::shared_ptr<ReadBudget>& budget,
                     void (*reject)() noexcept = nullptr) {
    Handle(BinMDF_ADriver) found; table->GetDriver(STANDARD_TYPE(TNaming_NamedShape), found);
    const auto shapes = Handle(BinMNaming_NamedShapeDriver)::DownCast(found);
    if (shapes.IsNull()) Standard_Failure::Raise("Composite recipe shared driver missing");
    table->AddDriver(new BinaryDriver(messenger, shapes, budget, reject));
}

template<class Base> class StorageDriver : public Base {
public:
    Handle(BinMDF_ADriverTable) AttributeDrivers(const Handle(Message_Messenger)& messenger) override {
        auto table = Base::AttributeDrivers(messenger); Handle(BinMDF_ADriver) unitDriver;
        table->GetDriver(STANDARD_TYPE(XCAFDoc_LengthUnit), unitDriver);
        if (unitDriver.IsNull()) table->AddDriver(new BinMXCAFDoc_LengthUnitDriver(messenger));
        composite_recipe::Register(table, messenger, std::make_shared<ReadBudget>()); return table;
    }
    void Write(const Handle(CDM_Document)& document, const TCollection_ExtendedString& file,
               const Message_ProgressRange& progress = Message_ProgressRange()) override {
        Prepare(document); Base::Write(document, file, progress);
    }
    void Write(const Handle(CDM_Document)& document, Standard_OStream& stream,
               const Message_ProgressRange& progress = Message_ProgressRange()) override {
        Prepare(document); Base::Write(document, stream, progress);
    }
private:
    static void Prepare(const Handle(CDM_Document)& value) {
        const auto document = Handle(TDocStd_Document)::DownCast(value); std::vector<Record> records;
        if (!ReadAll(document, records) || (!records.empty() && !Core3DValidateCompositeRecipeDocument(document)))
            Standard_Failure::Raise("Composite recipe writer owner/aggregate geometry");
    }
};
} // namespace core3d::composite_recipe
