#pragma once

#include "FeaturePatternChildAttribute.hxx"

#include <BinMDF_ADriverTable.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <TDocStd_Document.hxx>
#include <TDocStd_FormatVersion.hxx>
#include <climits>

namespace core3d::feature_pattern_child {
static_assert(sizeof(Standard_Integer) == 4, "Native attribute prefix width changed");

class BinaryDriver final : public BinMDF_ADriver {
public:
    BinaryDriver(const Handle(Message_Messenger)& messenger,
                 std::shared_ptr<Budget> budget,
                 void (*reject)() noexcept = nullptr)
        : BinMDF_ADriver(messenger, STANDARD_TYPE(Attribute)->Name()),
          budget_(std::move(budget)), reject_(reject) {}

    Handle(TDF_Attribute) NewEmpty() const override { return new Attribute(); }
    const Handle(Standard_Type)& SourceType() const override {
        return STANDARD_TYPE(Attribute);
    }

    Standard_Boolean Paste(const BinObjMgt_Persistent& source,
                           const Handle(TDF_Attribute)& target,
                           BinObjMgt_RRelocationTable& relocation) const override {
        try {
            const auto attribute = Handle(Attribute)::DownCast(target);
            if (attribute.IsNull() || attribute->value_ || !budget_
                || budget_->rejected || relocation.GetHeaderData().IsNull()) return Refuse();
            const auto storage = relocation.GetHeaderData()->StorageVersion();
            if (!storage.IsIntegerValue()
                || storage.IntegerValue() < TDocStd_FormatVersion_VERSION_10
                || storage.IntegerValue() > TDocStd_FormatVersion_CURRENT) return Refuse();
            // BinObjMgt_Persistent records begin after the native three-integer
            // header (BP_HEADSIZE in the bundled BinObjMgt_Persistent.lxx):
            // Length() excludes it while Position() is absolute. The two
            // schema/count integers below are record payload, not header.
            constexpr Standard_Integer NativeHeaderBytes =
                3 * Standard_Integer(sizeof(Standard_Integer));
            constexpr Standard_Integer RecordPrefixBytes =
                2 * Standard_Integer(sizeof(Standard_Integer));
            const auto start = source.Position(), length = source.Length();
            if (length < RecordPrefixBytes || length > INT_MAX - NativeHeaderBytes
                || start < NativeHeaderBytes) return Refuse();
            const auto end = length + NativeHeaderBytes;
            if (start > end) return Refuse();
            Standard_Integer schema = 0, count = 0;
            if (!(source >> schema >> count) || schema != 1 || count <= 0
                || count > Standard_Integer(MaximumReceiptBytes)
                || source.Position() > end || count != end - source.Position()) return Refuse();
            std::vector<std::uint8_t> bytes(std::size_t(count), 0);
            if (!source.GetByteArray(bytes.data(), count) || source.Position() != end)
                return Refuse();
            Receipt receipt;
            if (!Decode(bytes, receipt)
                || !Charge(*budget_, bytes.size(), receipt.selectors.size())) return Refuse();
            const TDF_Label record = attribute->Label();
            if (record.IsNull() || record.Father().IsNull()) return Refuse();
            auto payload = std::make_shared<Payload>(); payload->receipt = std::move(receipt);
            payload->canonicalBytes = std::move(bytes); payload->host = record.Father();
            // The baseline TDF_Reference and the source/pattern link children
            // may be visited after this parent attribute. They stay pending
            // (null) here; Read() resolves and validates all three native
            // links after traversal. Pending links convey no readable or
            // editable authority.
            attribute->value_ = std::move(payload);
            return Standard_True;
        } catch (...) { return Refuse(); }
    }

    void Paste(const Handle(TDF_Attribute)& source, BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable&) const override {
        const auto attribute = Handle(Attribute)::DownCast(source);
        if (attribute.IsNull() || !attribute->value_ || attribute->Label().IsNull())
            Standard_Failure::Raise("Feature child writer source");
        const auto document = TDocStd_Document::Get(attribute->Label());
        std::shared_ptr<const Payload> value;
        std::vector<std::uint8_t> exact;
        // Read() performs the strict post-traversal link resolution, so a
        // reloaded attribute whose native links were pending at Paste time
        // serializes the same validated canonical bytes.
        if (document.IsNull()
            || document->StorageFormatVersion() < TDocStd_FormatVersion_VERSION_10
            || document->StorageFormatVersion() > TDocStd_FormatVersion_CURRENT
            || !Read(attribute->Label(), value) || !value
            || value->source.IsNull() || value->patternRecord.IsNull()
            || !Encode(value->receipt, exact) || exact != value->canonicalBytes)
            Standard_Failure::Raise("Feature child writer receipt/reference");
        target << Standard_Integer(1) << Standard_Integer(exact.size());
        target.PutByteArray(exact.data(), Standard_Integer(exact.size()));
    }

private:
    Standard_Boolean Refuse() const noexcept {
        if (budget_) budget_->rejected = true;
        if (reject_) reject_();
        return Standard_False;
    }
    const std::shared_ptr<Budget> budget_;
    void (*reject_)() noexcept;
};

inline void Register(const Handle(BinMDF_ADriverTable)& table,
                     const Handle(Message_Messenger)& messenger,
                     const std::shared_ptr<Budget>& budget,
                     void (*reject)() noexcept = nullptr) {
    if (table.IsNull() || !budget) Standard_Failure::Raise("Feature child driver table");
    table->AddDriver(new BinaryDriver(messenger, budget, reject));
}

template<class Base> class StorageDriver : public Base {
public:
    Handle(BinMDF_ADriverTable) AttributeDrivers(
        const Handle(Message_Messenger)& messenger) override {
        auto table = Base::AttributeDrivers(messenger);
        Register(table, messenger, std::make_shared<Budget>()); return table;
    }
    void Write(const Handle(CDM_Document)& document,
               const TCollection_ExtendedString& file,
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
        std::vector<PairedRecord> pairs;
        const PairStatus status = ReadPairs(document, pairs);
        // Legacy tag-73-only files stay byte-for-byte legacy. Only malformed,
        // partial or mixed paired state is blocked from persistence.
        if (status == PairStatus::Invalid)
            Standard_Failure::Raise("Feature pattern receipt pair writer");
        // The shared strict baseline-role census guards the same boundary:
        // every nonlegacy paired D4 must keep exactly one validated typed
        // host baseline. Absence and legacy absence remain persistable.
        std::vector<BaselineRecord> baselines;
        if (ReadBaselines(document, baselines) == BaselineStatus::Invalid)
            Standard_Failure::Raise("Feature pattern baseline pair writer");
    }
};
} // namespace core3d::feature_pattern_child
