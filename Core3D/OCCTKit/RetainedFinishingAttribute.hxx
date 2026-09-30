#pragma once
// E1 native persistence: exactly one retained finishing receipt per retained
// parametric owner. The value seam (SYEF/1 codec, bounds, one-owner
// transaction) is RetainedFinishingRecord.hxx and is not redrafted here;
// this header binds the canonical bytes to one dedicated OCAF attribute.
// The record is a derivative receipt: it never grants authority to mutate
// the source BRep, and it admits no tessellation sweep or fallback
// promotion. Cone/sphere verifiedChart consumers stay refused until an
// actual geometry/quality producer is admitted.
#include "RetainedFinishingRecord.hxx"
#include "RetainedSolidAttribute.hxx"
#include <TDF_Attribute.hxx>
#include <TDF_AttributeIterator.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelMap.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <memory>

class OcctDocument;
namespace core3d::retained_finishing {
class BinaryDriver;
#if DEBUG
struct PersistenceProbe;
#endif

// Persistent schema identifier. Never reuse or renumber.
inline const Standard_GUID& AttributeID() {
    static const Standard_GUID id("4E266F1A-9C3D-4B7E-8A51-2D6C0B1E5F94"); return id;
}
// Dedicated child label under the retained owner label. The recipe carriers
// own the tag-13 record labels; the finishing receipt never shares them and
// carries no TNaming_NamedShape of its own.
inline constexpr int RecordTag = 17;
inline constexpr std::size_t MaximumAggregateReceiptBytes = kMaximumBytes;

struct Payload final {
    Definition definition;
    std::vector<std::uint8_t> bytes; // canonical SYEF/1, digest-checked
};

class Core3D_RetainedFinishing final : public TDF_Attribute {
public:
    DEFINE_STANDARD_RTTI_INLINE(Core3D_RetainedFinishing, TDF_Attribute)
    const Standard_GUID& ID() const override { return AttributeID(); }
    Handle(TDF_Attribute) NewEmpty() const override { return new Core3D_RetainedFinishing(); }
    const std::shared_ptr<const Payload>& value() const noexcept { return value_; }
    void Restore(const Handle(TDF_Attribute)& source) override {
        const auto original = Handle(Core3D_RetainedFinishing)::DownCast(source);
        if (original.IsNull()) Standard_Failure::Raise("Retained finishing restore type");
        // Backup/Undo/Redo share immutable payload ownership; a receipt is
        // never reparsed or rewritten by history traversal.
        value_ = original->value_;
    }
    void Paste(const Handle(TDF_Attribute)& target, const Handle(TDF_RelocationTable)&) const override {
        const auto destination = Handle(Core3D_RetainedFinishing)::DownCast(target);
        if (destination.IsNull() || !value_) Standard_Failure::Raise("Retained finishing paste type");
        // IDs remain exact on cross-document copy; read-back rejects a
        // foreign document/entity instead of manufacturing a new identity.
        destination->Backup(); destination->value_ = value_;
    }
    // Single-owner commit seam. The caller holds the already-open OCAF
    // command under the existing document mutation owner; refusal leaves no
    // delta, and a later command abort restores the prior payload exactly.
    static bool StageCommitted(const Handle(TDocStd_Document)& document,
                               const TDF_Label& owner,
                               const std::shared_ptr<const Payload>& payload) noexcept {
        try {
            if (document.IsNull() || !document->HasOpenCommand() || owner.IsNull()
                || owner.Data() != document->GetData() || !payload
                || !XCAFDoc_ShapeTool::IsSimpleShape(owner) || !XCAFDoc_ShapeTool::IsFree(owner))
                return false;
            std::vector<std::uint8_t> canonical;
            if (!Encode(payload->definition, canonical) || canonical != payload->bytes)
                return false;
            UUID documentID{}, entity{}, definition{};
            if (!retained_solid::ReadUUID(document->Main(),
                    Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)
                || !retained_solid::ReadUUID(owner,
                    Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"), entity)
                || !retained_solid::ReadUUID(owner,
                    Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"), definition)
                || payload->definition.owner.document != documentID
                || payload->definition.owner.entity != entity
                || payload->definition.owner.definition != definition) return false;
            const TDF_Label record = owner.FindChild(RecordTag, Standard_True);
            if (record.IsNull()) return false;
            Handle(Core3D_RetainedFinishing) attribute;
            if (!record.FindAttribute(AttributeID(), attribute) || attribute.IsNull()) {
                for (TDF_AttributeIterator it(record); it.More(); it.Next()) return false;
                attribute = new Core3D_RetainedFinishing();
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
    friend struct PersistenceProbe;
#endif
    std::shared_ptr<const Payload> value_;
};
using Attribute = Core3D_RetainedFinishing;

struct Record {
    TDF_Label label, owner;
    std::shared_ptr<const Payload> value;
};

inline bool HasRecord(const TDF_Label& owner) noexcept {
    try {
        if (owner.IsNull()) return false;
        if (owner.IsAttribute(AttributeID())) return true;
        int visited = 0;
        for (TDF_ChildIterator child(owner, Standard_False); child.More(); child.Next())
            if (++visited > profile::MaximumLabels || child.Value().IsAttribute(AttributeID()))
                return true;
        return false;
    } catch (...) { return true; } // Unknown metadata cannot become absence.
}

// Whole-document read-back: exact owner bindings, canonical bytes and
// aggregate budget. Any malformed, duplicated or foreign-bound receipt
// refuses the document rather than degrading to absence.
inline bool ReadAll(const Handle(TDocStd_Document)& document,
                    std::vector<Record>& output) noexcept {
    output.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const TDF_Label root = document->GetData()->Root();
        if (root.IsAttribute(AttributeID())) return false;
        std::vector<Record> staged; TDF_LabelMap owners;
        std::size_t aggregate = 0; int visited = 0;
        UUID documentID{}; bool readDocumentID = false;
        for (TDF_ChildIterator iterator(root, Standard_True); iterator.More(); iterator.Next()) {
            if (++visited > profile::MaximumLabels) return false;
            const TDF_Label label = iterator.Value();
            if (!label.IsAttribute(AttributeID())) continue;
            Handle(Attribute) attribute;
            const TDF_Label owner = label.Father();
            if (label.Tag() != RecordTag
                || !label.FindAttribute(AttributeID(), attribute) || attribute.IsNull()
                || !attribute->value()
                || !XCAFDoc_ShapeTool::IsSimpleShape(owner) || !XCAFDoc_ShapeTool::IsFree(owner)
                || !owners.Add(owner)
                || staged.size() >= std::size_t(profile::MaximumRecords) / 2) return false;
            const auto value = attribute->value();
            std::vector<std::uint8_t> exact;
            if (!Encode(value->definition, exact) || exact != value->bytes
                || exact.size() > MaximumAggregateReceiptBytes - aggregate) return false;
            aggregate += exact.size();
            if (!readDocumentID) {
                if (!retained_solid::ReadUUID(document->Main(),
                        Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID))
                    return false;
                readDocumentID = true;
            }
            UUID entity{}, definition{};
            if (!retained_solid::ReadUUID(owner,
                    Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"), entity)
                || !retained_solid::ReadUUID(owner,
                    Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"), definition)
                || value->definition.owner.document != documentID
                || value->definition.owner.entity != entity
                || value->definition.owner.definition != definition) return false;
            for (TDF_AttributeIterator attributes(label); attributes.More(); attributes.Next())
                if (attributes.Value()->ID() != AttributeID()) return false;
            int descendants = 0;
            for (TDF_ChildIterator child(label, Standard_True); child.More(); child.Next())
                if (++descendants > profile::MaximumLabels || child.Value().HasAttribute())
                    return false;
            staged.push_back({label, owner, value});
        }
        output = std::move(staged);
        return true;
    } catch (...) { output.clear(); return false; }
}

// Local absence matches the existing family-reader contract. Whole-document
// admission remains mandatory at save, native load and finishing use.
inline bool Read(const Handle(TDocStd_Document)& document, const TDF_Label& owner,
                 Record& output) noexcept {
    output = {};
    if (document.IsNull() || owner.IsNull() || owner.Data() != document->GetData())
        return false;
    if (!HasRecord(owner)) return true;
    std::vector<Record> all;
    if (!ReadAll(document, all)) return false;
    for (const Record& record : all)
        if (record.owner.IsEqual(owner)) { output = record; return true; }
    return false;
}

struct ReadBudget {
    std::size_t bytes = 0, records = 0;
    std::size_t limit = MaximumAggregateReceiptBytes;
    bool rejected = false;
    void reset() { bytes = 0; records = 0; rejected = false; }
};

// One-owner staging API. Implemented by RetainedFinishingOwner.mm. The
// caller supplies the observed source fence captured by the existing
// document authority; this seam never manufactures currentness itself.
namespace owner {
enum class Outcome : std::uint8_t {
    Prepared, Committed, Refused, StaleSource, OwnerMismatch, Busy,
    Malformed, PersistenceFailure
};
struct Staging final {
    Transaction transaction;
    Definition candidate;
    std::vector<std::uint8_t> bytes;
    TDF_Label ownerLabel;
};
bool ResolveOwnerLabel(const Handle(TDocStd_Document)& document, const OwnerKey& key,
                       TDF_Label& output) noexcept;
Outcome Prepare(Staging& staging, const Handle(TDocStd_Document)& document,
                const Definition& candidate, const SourceRevision& observed) noexcept;
Outcome Commit(Staging& staging, const Handle(TDocStd_Document)& document,
               const SourceRevision& observed) noexcept;
void Cancel(Staging& staging) noexcept;
} // namespace owner
} // namespace core3d::retained_finishing
