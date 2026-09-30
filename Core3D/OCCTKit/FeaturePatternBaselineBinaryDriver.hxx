#pragma once

#include "FeaturePatternChildAttribute.hxx"

#include <BinMDF_ADriverTable.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <TDocStd_Document.hxx>
#include <TDocStd_FormatVersion.hxx>
#include <climits>

namespace core3d::feature_pattern_baseline {
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
            // Record-relative framing exactly like the feature-child driver:
            // Length() excludes the native three-integer header while
            // Position() is absolute. The two schema/count integers below are
            // record payload, never stream-wide available bytes.
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
                || count > Standard_Integer(MaximumBaselineBytes)
                || source.Position() > end || count != end - source.Position()) return Refuse();
            std::vector<std::uint8_t> bytes(std::size_t(count), 0);
            if (!source.GetByteArray(bytes.data(), count) || source.Position() != end)
                return Refuse();
            Envelope envelope;
            if (!Decode(bytes, envelope) || !Charge(*budget_, bytes.size()))
                return Refuse();
            const TDF_Label label = attribute->Label();
            if (label.IsNull() || label.Father().IsNull()) return Refuse();
            auto payload = std::make_shared<Payload>();
            payload->envelope = std::move(envelope);
            payload->canonicalBytes = std::move(bytes);
            payload->host = label.Father();
            // Native references (the paired record/host roles and the bound
            // solid) are resolved after traversal by the shared strict census;
            // visitation order conveys no temporary authority.
            attribute->value_ = std::move(payload);
            return Standard_True;
        } catch (...) { return Refuse(); }
    }

    void Paste(const Handle(TDF_Attribute)& source, BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable&) const override {
        const auto attribute = Handle(Attribute)::DownCast(source);
        if (attribute.IsNull() || !attribute->value_ || attribute->Label().IsNull())
            Standard_Failure::Raise("Feature baseline writer source");
        const auto document = TDocStd_Document::Get(attribute->Label());
        std::shared_ptr<const Payload> value;
        std::vector<std::uint8_t> exact;
        // Read() re-validates the sealed canonical payload and its host anchor
        // before any byte is written, so a reloaded attribute serializes the
        // same validated canonical bytes it was admitted with.
        if (document.IsNull()
            || document->StorageFormatVersion() < TDocStd_FormatVersion_VERSION_10
            || document->StorageFormatVersion() > TDocStd_FormatVersion_CURRENT
            || !Read(attribute->Label(), value) || !value
            || !Encode(value->envelope, exact) || exact != value->canonicalBytes)
            Standard_Failure::Raise("Feature baseline writer payload");
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
    if (table.IsNull() || !budget) Standard_Failure::Raise("Feature baseline driver table");
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
        std::vector<core3d::feature_pattern_child::BaselineRecord> baselines;
        const auto status =
            core3d::feature_pattern_child::ReadBaselines(document, baselines);
        // Absence stays valid for documents without paired D4 and legacy
        // tag-73-only files stay byte-for-byte legacy. Only malformed, orphan,
        // duplicate or foreign baseline state is blocked from persistence.
        if (status == core3d::feature_pattern_child::BaselineStatus::Invalid)
            Standard_Failure::Raise("Feature pattern baseline writer");
    }
};
} // namespace core3d::feature_pattern_baseline
