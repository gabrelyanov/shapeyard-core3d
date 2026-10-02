#pragma once
// E2a asset-atlas persistence: exactly one dedicated OCAF attribute per
// atlas record, stored on child labels of a document-owned atlas root child
// of document->Main() (never a shape label). The value seam (SYEA/1 codec,
// budgets, all-member transaction) is AssetAtlasDefinition.hxx and is not
// redrafted here; this header binds the canonical bytes to the attribute and
// its binary driver, mirroring the row-268
// RetainedFinishingAttribute.hxx/RetainedFinishingBinaryDriver.hxx pair in a
// single header. The record is a derivative receipt: it never grants
// authority to mutate any source BRep.
#include "AssetAtlasDefinition.hxx"
#include "RetainedFinishingAttribute.hxx"
#include <TDF_Attribute.hxx>
#include <TDF_AttributeIterator.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <BinMDF_ADriverTable.hxx>
#include <BinMXCAFDoc_LengthUnitDriver.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <Message_ProgressRange.hxx>
#include <TDocStd_FormatVersion.hxx>
#include <XCAFDoc_LengthUnit.hxx>
#include <climits>
#include <memory>

Standard_Boolean Core3DValidateAssetAtlasDocument(
    const Handle(TDocStd_Document)& document);

class OcctDocument;
namespace core3d::asset_atlas {
#if DEBUG
struct AssetAtlasProbe;
#endif
namespace persistence {

class BinaryDriver;

// Persistent schema identifier. Never reuse or renumber.
inline const Standard_GUID& AttributeID() {
    static const Standard_GUID id("E271A7A5-5E7A-4B1C-9A2D-0EA5507A7101"); return id;
}
// Document-owned atlas root child tag under document->Main(). Fixed schema
// tag; verified disjoint from every other fixed Main-child tag (pattern 71,
// path-array 72, feature-pattern 73, debug-only 200).
inline constexpr int RootTag = 271;
inline constexpr std::size_t MaximumAggregateAtlasBytes = kMaximumBytes;

struct Payload final {
    Definition definition;
    std::vector<std::uint8_t> bytes; // canonical SYEA/1, digest-checked
};

class Core3D_AssetAtlas final : public TDF_Attribute {
public:
    DEFINE_STANDARD_RTTI_INLINE(Core3D_AssetAtlas, TDF_Attribute)
    const Standard_GUID& ID() const override { return AttributeID(); }
    Handle(TDF_Attribute) NewEmpty() const override { return new Core3D_AssetAtlas(); }
    const std::shared_ptr<const Payload>& value() const noexcept { return value_; }
    void Restore(const Handle(TDF_Attribute)& source) override {
        const auto original = Handle(Core3D_AssetAtlas)::DownCast(source);
        if (original.IsNull()) Standard_Failure::Raise("Asset atlas restore type");
        // Backup/Undo/Redo share immutable payload ownership; a record is
        // never reparsed or rewritten by history traversal.
        value_ = original->value_;
    }
    void Paste(const Handle(TDF_Attribute)& target, const Handle(TDF_RelocationTable)&) const override {
        const auto destination = Handle(Core3D_AssetAtlas)::DownCast(target);
        if (destination.IsNull() || !value_) Standard_Failure::Raise("Asset atlas paste type");
        // IDs remain exact on cross-document copy; read-back rejects a
        // foreign document instead of manufacturing a new identity.
        destination->Backup(); destination->value_ = value_;
    }
    // Whole-atlas commit seam. The caller holds the already-open OCAF command
    // under the existing document mutation owner; refusal leaves no delta,
    // and a later command abort restores the prior payload exactly.
    static bool StageCommitted(const Handle(TDocStd_Document)& document,
                               const std::shared_ptr<const Payload>& payload) noexcept {
        try {
            if (document.IsNull() || !document->HasOpenCommand() || !payload
                || document->GetData().IsNull()) return false;
            std::vector<std::uint8_t> canonical;
            if (!Encode(payload->definition, canonical) || canonical != payload->bytes)
                return false;
            UUID documentID{};
            if (!retained_solid::ReadUUID(document->Main(),
                    Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)
                || !(payload->definition.key.document == documentID)) return false;
            const TDF_Label root = document->Main().FindChild(RootTag, Standard_True);
            if (root.IsNull()) return false;
            TDF_Label record; int highest = 0; int visited = 0;
            for (TDF_ChildIterator child(root, Standard_False); child.More(); child.Next()) {
                if (++visited > profile::MaximumLabels) return false;
                highest = std::max(highest, child.Value().Tag());
                Handle(Core3D_AssetAtlas) existing;
                if (!child.Value().FindAttribute(AttributeID(), existing) || existing.IsNull()
                    || !existing->value()) continue;
                if (existing->value()->definition.key.atlas == payload->definition.key.atlas)
                    record = child.Value();
            }
            if (record.IsNull()) {
                if (highest >= profile::MaximumLabels) return false;
                record = root.FindChild(highest + 1, Standard_True);
                if (record.IsNull()) return false;
                for (TDF_AttributeIterator it(record); it.More(); it.Next()) return false;
            }
            Handle(Core3D_AssetAtlas) attribute;
            if (!record.FindAttribute(AttributeID(), attribute) || attribute.IsNull()) {
                for (TDF_AttributeIterator it(record); it.More(); it.Next()) return false;
                attribute = new Core3D_AssetAtlas();
                record.AddAttribute(attribute);
            }
            attribute->Backup();
            attribute->value_ = payload;
            return true;
        } catch (...) { return false; }
    }
private:
    friend class BinaryDriver;
    friend class ::OcctDocument;
#if DEBUG
    friend struct core3d::asset_atlas::AssetAtlasProbe;
#endif
    std::shared_ptr<const Payload> value_;
};
using Attribute = Core3D_AssetAtlas;

struct Record {
    TDF_Label label;
    std::shared_ptr<const Payload> value;
    std::vector<TDF_Label> memberOwners; // resolved live owner labels, member order
};

// Whole-document read-back: exact key/member bindings, canonical SYEA/1
// bytes and the aggregate budgets. Any malformed, duplicated or
// foreign-bound record refuses the document rather than degrading to
// absence. Each member owner must resolve to exactly one live free simple
// shape whose committed finishing receipt matches the fenced receipt.
inline bool ReadAll(const Handle(TDocStd_Document)& document,
                    std::vector<Record>& output) noexcept {
    output.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const TDF_Label main = document->Main();
        if (main.IsNull() || main.IsAttribute(AttributeID())) return false;
        const TDF_Label root = main.FindChild(RootTag, Standard_False);
        if (root.IsNull()) return true; // no atlas root: zero records
        if (root.HasAttribute()) return false;
        UUID documentID{};
        if (!retained_solid::ReadUUID(main,
                Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID))
            return false;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(main);
        if (shapes.IsNull()) return false;
        TDF_LabelSequence freeShapes; shapes->GetFreeShapes(freeShapes);
        std::vector<Record> staged;
        std::size_t aggregate = 0; int visited = 0;
        for (TDF_ChildIterator child(root, Standard_False); child.More(); child.Next()) {
            if (++visited > profile::MaximumLabels
                || staged.size() >= std::size_t(profile::MaximumRecords) / 2) return false;
            const TDF_Label label = child.Value();
            if (!label.IsAttribute(AttributeID())) {
                if (label.HasAttribute()) return false;
                continue;
            }
            Handle(Attribute) attribute;
            if (!label.FindAttribute(AttributeID(), attribute) || attribute.IsNull()
                || !attribute->value()) return false;
            for (TDF_AttributeIterator attributes(label); attributes.More(); attributes.Next())
                if (attributes.Value()->ID() != AttributeID()) return false;
            int descendants = 0;
            for (TDF_ChildIterator nested(label, Standard_True); nested.More(); nested.Next())
                if (++descendants > profile::MaximumLabels || nested.Value().HasAttribute())
                    return false;
            const auto value = attribute->value();
            std::vector<std::uint8_t> exact;
            if (!Encode(value->definition, exact) || exact != value->bytes
                || exact.size() > MaximumAggregateAtlasBytes - aggregate) return false;
            aggregate += exact.size();
            if (!(value->definition.key.document == documentID)) return false;
            for (const auto& prior : staged)
                if (prior.value->definition.key.atlas == value->definition.key.atlas)
                    return false;
            Record row; row.label = label; row.value = value;
            for (const auto& member : value->definition.members) {
                int matches = 0; TDF_Label owner;
                for (Standard_Integer index = 1; index <= freeShapes.Length(); ++index) {
                    UUID entity{}, definition{};
                    const TDF_Label candidate = freeShapes.Value(index);
                    if (!XCAFDoc_ShapeTool::IsSimpleShape(candidate)
                        || !retained_solid::ReadUUID(candidate,
                            Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"), entity)
                        || !retained_solid::ReadUUID(candidate,
                            Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"), definition)
                        || !(entity == member.owner.entity)
                        || !(definition == member.owner.definition)) continue;
                    if (++matches > 1) return false;
                    owner = candidate;
                }
                if (matches != 1) return false;
                // One retained solid belongs to at most one atlas, across
                // every record of the document.
                for (const auto& prior : staged)
                    for (const auto& priorOwner : prior.memberOwners)
                        if (priorOwner.IsEqual(owner)) return false;
                retained_finishing::Record receipt;
                if (!retained_finishing::Read(document, owner, receipt) || !receipt.value
                    || !(receipt.value->definition.finishing == member.finishing)
                    || !(receipt.value->definition.owner == member.owner)) return false;
                row.memberOwners.push_back(owner);
            }
            staged.push_back(std::move(row));
        }
        output = std::move(staged);
        return true;
    } catch (...) { output.clear(); return false; }
}

// Local absence matches the existing family-reader contract. Whole-document
// admission remains mandatory at save, native load and atlas use.
inline bool Read(const Handle(TDocStd_Document)& document, const Key& key,
                 Record& output) noexcept {
    output = {};
    try {
        if (document.IsNull() || !retained_recipe::Nonzero(key.document)
            || !retained_recipe::Nonzero(key.atlas)) return false;
        std::vector<Record> all;
        if (!ReadAll(document, all)) return false;
        for (const Record& record : all)
            if (record.value->definition.key == key) { output = record; break; }
        return true;
    } catch (...) { output = {}; return false; }
}

// Record removal (last-member membership edit). Runs inside the caller's
// open OCAF command; undo restores the attribute exactly.
inline bool Remove(const Handle(TDocStd_Document)& document, const Key& key) noexcept {
    try {
        if (document.IsNull() || !document->HasOpenCommand()) return false;
        Record record;
        if (!Read(document, key, record)) return false;
        if (!record.value) return true;
        Handle(Attribute) attribute;
        if (!record.label.FindAttribute(AttributeID(), attribute) || attribute.IsNull())
            return false;
        attribute->Backup();
        record.label.ForgetAttribute(AttributeID());
        return true;
    } catch (...) { return false; }
}

struct ReadBudget {
    std::size_t bytes = 0, records = 0;
    std::size_t limit = MaximumAggregateAtlasBytes;
    bool rejected = false;
    void reset() { bytes = 0; records = 0; rejected = false; }
};

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
                || budget_->limit > MaximumAggregateAtlasBytes
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
            // Publish only after complete digest-checked decode. Key, member
            // and document admission runs before document adoption.
            attribute->value_ = std::move(value);
            budget_->bytes += std::size_t(count); ++budget_->records;
            return Standard_True;
        } catch (...) { return Refuse(); }
    }

    void Paste(const Handle(TDF_Attribute)& source, BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable&) const override {
        const auto attribute = Handle(Attribute)::DownCast(source);
        if (attribute.IsNull() || !attribute->value_ || attribute->Label().IsNull())
            Standard_Failure::Raise("Asset atlas writer source");
        const auto document = TDocStd_Document::Get(attribute->Label());
        if (document.IsNull()
            || document->StorageFormatVersion() < TDocStd_FormatVersion_VERSION_10
            || document->StorageFormatVersion() > TDocStd_FormatVersion_CURRENT)
            Standard_Failure::Raise("Asset atlas writer version");
        const auto& value = *attribute->value_;
        std::vector<std::uint8_t> encoded;
        if (!Encode(value.definition, encoded) || encoded != value.bytes)
            Standard_Failure::Raise("Asset atlas writer record");
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
    if (table.IsNull()) Standard_Failure::Raise("Asset atlas driver table");
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
            || (!records.empty() && !Core3DValidateAssetAtlasDocument(document)))
            Standard_Failure::Raise("Asset atlas writer owner/record");
    }
};
} // namespace persistence

// All-member staging API. Implemented by AssetAtlasOwner.mm. The caller
// supplies the observed membership fences captured by the existing document
// authority; this seam never manufactures currentness itself and never opens
// its own mutation route.
namespace owner {
enum class Outcome : std::uint8_t {
    Prepared, Committed, Refused, StaleSource, MissingMember, ForeignMember,
    OverBudget, PaintedRebakeRequired, UnsupportedSurface, OwnerMismatch,
    Busy, Malformed, PersistenceFailure
};
struct Staging final {
    Transaction transaction;
    Definition candidate;
    std::vector<std::uint8_t> bytes;
    std::vector<MemberUVAssignment> assignments;
    std::vector<Member> observed;
};
bool ResolveAtlasLabel(const Handle(TDocStd_Document)& document, const Key& key,
                       TDF_Label& output) noexcept;
Outcome Prepare(Staging& staging, const Handle(TDocStd_Document)& document,
                const Definition& candidate,
                const std::vector<MemberUVAssignment>& assignments,
                const std::vector<Member>& observed) noexcept;
Outcome Commit(Staging& staging, const Handle(TDocStd_Document)& document,
               const std::vector<Member>& observed) noexcept;
void Cancel(Staging& staging) noexcept;
} // namespace owner
} // namespace core3d::asset_atlas
