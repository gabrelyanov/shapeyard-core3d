#pragma once
// E1 binary persistence driver for the retained finishing receipt. The wire
// payload is exactly the canonical SYEF/1 byte string; there is no shape
// codec, no second mutation route, and no tolerated trailing data.
#include "RetainedFinishingAttribute.hxx"
#include <BinMDF_ADriverTable.hxx>
#include <BinMXCAFDoc_LengthUnitDriver.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <Message_ProgressRange.hxx>
#include <TDocStd_FormatVersion.hxx>
#include <XCAFDoc_LengthUnit.hxx>
#include <climits>

Standard_Boolean Core3DValidateRetainedFinishingDocument(
    const Handle(TDocStd_Document)& document);

namespace core3d::retained_finishing {
static_assert(sizeof(Standard_Integer) == 4, "Native attribute prefix width changed");

class BinaryDriver final : public BinMDF_ADriver {
public:
    BinaryDriver(const Handle(Message_Messenger)& messenger,
                 std::shared_ptr<ReadBudget> budget,
                 void (*reject)() noexcept = nullptr)
        : BinMDF_ADriver(messenger, STANDARD_TYPE(Attribute)->Name()),
          budget_(std::move(budget)), reject_(reject) {}
    Handle(TDF_Attribute) NewEmpty() const override { return new Attribute(); }
    const Handle(Standard_Type)& SourceType() const override { return STANDARD_TYPE(Attribute); }

    Standard_Boolean Paste(const BinObjMgt_Persistent& source,
                           const Handle(TDF_Attribute)& target,
                           BinObjMgt_RRelocationTable& relocation) const override {
        try {
            const auto attribute = Handle(Attribute)::DownCast(target);
            if (attribute.IsNull() || attribute->value_ || !budget_ || budget_->rejected
                || relocation.GetHeaderData().IsNull()) return Refuse();
            const auto version = relocation.GetHeaderData()->StorageVersion();
            if (!version.IsIntegerValue()
                || version.IntegerValue() < TDocStd_FormatVersion_VERSION_10
                || version.IntegerValue() > TDocStd_FormatVersion_CURRENT) return Refuse();
            const auto start = source.Position(); const auto length = source.Length();
            if (length < 0 || length > INT_MAX - 12 || start < 12 || start > length + 12)
                return Refuse();
            const auto end = length + 12;
            Standard_Integer schema = 0, count = 0;
            // Whole-value admission: bounds are checked before allocation,
            // unknown schema/version refuses, and no byte is left unread.
            if (!(source >> schema >> count) || schema != 1
                || count <= 0 || count > Standard_Integer(kMaximumBytes)
                || source.Position() > end || count > end - source.Position()
                || budget_->limit > MaximumAggregateReceiptBytes
                || budget_->bytes > budget_->limit
                || std::size_t(count) > budget_->limit - budget_->bytes
                || budget_->records >= std::size_t(profile::MaximumRecords) / 2)
                return Refuse();
            std::vector<std::uint8_t> bytes(std::size_t(count), 0);
            if (!source.GetByteArray(bytes.data(), count)) return Refuse();
            Definition definition; Refusal refusal = Refusal::None;
            if (!Decode(bytes, definition, refusal) || source.Position() != end)
                return Refuse();
            auto value = std::make_shared<Payload>();
            value->definition = std::move(definition);
            value->bytes = std::move(bytes);
            // Publish only after complete digest-checked decode. Owner and
            // document admission runs before document adoption.
            attribute->value_ = std::move(value);
            budget_->bytes += std::size_t(count); ++budget_->records;
            return Standard_True;
        } catch (...) { return Refuse(); }
    }

    void Paste(const Handle(TDF_Attribute)& source, BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable&) const override {
        const auto attribute = Handle(Attribute)::DownCast(source);
        if (attribute.IsNull() || !attribute->value_ || attribute->Label().IsNull())
            Standard_Failure::Raise("Retained finishing writer source");
        const auto document = TDocStd_Document::Get(attribute->Label());
        if (document.IsNull()
            || document->StorageFormatVersion() < TDocStd_FormatVersion_VERSION_10
            || document->StorageFormatVersion() > TDocStd_FormatVersion_CURRENT)
            Standard_Failure::Raise("Retained finishing writer version");
        const auto& value = *attribute->value_;
        std::vector<std::uint8_t> encoded;
        if (!Encode(value.definition, encoded) || encoded != value.bytes)
            Standard_Failure::Raise("Retained finishing writer receipt");
        target << Standard_Integer(1) << Standard_Integer(encoded.size());
        target.PutByteArray(encoded.data(), Standard_Integer(encoded.size()));
    }
private:
    Standard_Boolean Refuse() const noexcept {
        if (budget_) budget_->rejected = true;
        if (reject_) reject_();
        return Standard_False;
    }
    const std::shared_ptr<ReadBudget> budget_;
    void (*reject_)() noexcept;
};

// Register into the narrow project reader/writer table. No shared shape or
// location driver is required; a missing table entry is a configuration
// error surfaced by document admission, never a fallback.
inline void Register(const Handle(BinMDF_ADriverTable)& table,
                     const Handle(Message_Messenger)& messenger,
                     const std::shared_ptr<ReadBudget>& budget,
                     void (*reject)() noexcept = nullptr) {
    if (table.IsNull()) Standard_Failure::Raise("Retained finishing driver table");
    table->AddDriver(new BinaryDriver(messenger, budget, reject));
}

template<class Base> class StorageDriver : public Base {
public:
    Handle(BinMDF_ADriverTable) AttributeDrivers(
        const Handle(Message_Messenger)& messenger) override {
        auto table = Base::AttributeDrivers(messenger);
        // BinOcaf's base table omits this authoritative XCAF document unit.
        Handle(BinMDF_ADriver) unitDriver;
        table->GetDriver(STANDARD_TYPE(XCAFDoc_LengthUnit), unitDriver);
        if (unitDriver.IsNull())
            table->AddDriver(new BinMXCAFDoc_LengthUnitDriver(messenger));
        Register(table, messenger, std::make_shared<ReadBudget>());
        return table;
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
        const auto document = Handle(TDocStd_Document)::DownCast(value);
        std::vector<Record> records;
        if (!ReadAll(document, records)
            || (!records.empty() && !Core3DValidateRetainedFinishingDocument(document)))
            Standard_Failure::Raise("Retained finishing writer owner/receipt");
    }
};
} // namespace core3d::retained_finishing
