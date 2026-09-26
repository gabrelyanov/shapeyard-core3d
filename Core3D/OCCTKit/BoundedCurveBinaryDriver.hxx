#pragma once
#include "BoundedCurveAttribute.hxx"
#include <BinMDF_ADriverTable.hxx>
#include <BinMNaming_NamedShapeDriver.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <TDocStd_FormatVersion.hxx>
#include <climits>

Standard_Boolean Core3DValidateBoundedCurveDocument(const Handle(TDocStd_Document)& document);

namespace core3d::bounded_curve {
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
            Standard_Integer schema = 0, definitionCount = 0, ownerCount = 0;
            if (!(source >> schema >> definitionCount >> ownerCount) || schema != 1
                || definitionCount <= 0 || definitionCount > Standard_Integer(MaximumDefinitionBytes)
                || ownerCount <= 0 || ownerCount > Standard_Integer(MaximumOwnerBytes)
                || source.Position() > end || definitionCount > end - source.Position()
                || ownerCount > end - source.Position() - definitionCount
                || definitionCount + ownerCount != end - source.Position()
                || budget_->limit > MaximumDocumentAggregateBytes
                || budget_->definitionBytes > budget_->limit || budget_->ownerBytes > budget_->limit
                || std::size_t(definitionCount) > budget_->limit - budget_->definitionBytes
                || std::size_t(ownerCount) > budget_->limit - budget_->ownerBytes
                || budget_->records >= MaximumRecordsPerDocument) return Refuse();
            std::vector<std::uint8_t> definitionBytes(std::size_t(definitionCount), 0);
            std::vector<std::uint8_t> ownerBytes(std::size_t(ownerCount), 0);
            if (!source.GetByteArray(definitionBytes.data(), definitionCount)
                || !source.GetByteArray(ownerBytes.data(), ownerCount)
                || source.Position() != end) return Refuse();
            Value value; OwnerState ownerState;
            if (!Decode(definitionBytes, value) || !DecodeOwnerState(ownerBytes, ownerState)) return Refuse();
            PersistedValue persisted{std::move(value), std::move(ownerState)};
            std::vector<std::uint8_t> exactDefinition, exactOwner;
            if (!ValidatePersisted(persisted, &exactDefinition, &exactOwner)
                || exactDefinition != definitionBytes || exactOwner != ownerBytes) return Refuse();
            auto payload = std::make_shared<Payload>(); payload->persisted = std::move(persisted);
            payload->definitionBytes = std::move(definitionBytes); payload->ownerBytes = std::move(ownerBytes);
            attribute->value_ = std::move(payload);
            budget_->definitionBytes += std::size_t(definitionCount);
            budget_->ownerBytes += std::size_t(ownerCount); ++budget_->records;
            return Standard_True;
        } catch (...) { return Refuse(); }
    }

    void Paste(const Handle(TDF_Attribute)& source, BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable&) const override {
        const auto attribute = Handle(Attribute)::DownCast(source);
        if (attribute.IsNull() || !attribute->value_ || attribute->Label().IsNull() || shapes_.IsNull())
            Standard_Failure::Raise("Bounded curve writer source");
        const auto document = TDocStd_Document::Get(attribute->Label());
        if (document.IsNull() || document->StorageFormatVersion() < TDocStd_FormatVersion_VERSION_10
            || document->StorageFormatVersion() > TDocStd_FormatVersion_CURRENT)
            Standard_Failure::Raise("Bounded curve writer version");
        const auto& payload = *attribute->value_;
        std::vector<std::uint8_t> definitionBytes, ownerBytes;
        if (!ValidatePersisted(payload.persisted, &definitionBytes, &ownerBytes)
            || definitionBytes != payload.definitionBytes || ownerBytes != payload.ownerBytes)
            Standard_Failure::Raise("Bounded curve writer canonical value");
        target << Standard_Integer(1) << Standard_Integer(definitionBytes.size())
               << Standard_Integer(ownerBytes.size());
        target.PutByteArray(definitionBytes.data(), Standard_Integer(definitionBytes.size()));
        target.PutByteArray(ownerBytes.data(), Standard_Integer(ownerBytes.size()));
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
    if (shapes.IsNull()) Standard_Failure::Raise("Bounded curve shared driver missing");
    table->AddDriver(new BinaryDriver(messenger, shapes, budget, reject));
}

template<class Base> class StorageDriver : public Base {
public:
    Handle(BinMDF_ADriverTable) AttributeDrivers(const Handle(Message_Messenger)& messenger) override {
        auto table = Base::AttributeDrivers(messenger);
        core3d::bounded_curve::Register(table, messenger, std::make_shared<ReadBudget>()); return table;
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
        if (!ReadAll(document, records)
            || (!records.empty() && !Core3DValidateBoundedCurveDocument(document)))
            Standard_Failure::Raise("Bounded curve writer owner/aggregate");
    }
};
} // namespace core3d::bounded_curve
