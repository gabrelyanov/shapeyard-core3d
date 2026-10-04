#pragma once
// E3 face-image persistence (278b portion 1): two disjoint storage families.
//
// 1. Binding records (namespace bindings): the SYFI/1 binding metadata of one
//    retained solid owner lives on one dedicated fixed-tag child label under
//    the owner's label, using only attribute types already admitted by the
//    narrow document reader (a TDataStd_UAttribute marker, TDataStd_Integer
//    scalars, chunked TDataStd_AsciiString hex payload <= 320 chunks x 256
//    chars and a 64-hex digest attribute) with a strict fail-closed reader
//    that rejects unknown attributes, fragments below other descendants, bad
//    counts, bad hex and any non-canonical payload. No new binary driver is
//    introduced for this family, so shipped builds keep opening these
//    documents. The marker GUID is the review-frozen proposal
//    E278B1A5-1E3F-4C2A-8B3D-6F0E9A4C7D21; the sibling scalars share its
//    family suffix D22-D25 and are permanent serialized schema identifiers:
//    never reuse or renumber them.
//
// 2. Resource envelopes (namespace resources): the document-owned image
//    resource table. Each admitted envelope (verbatim original bytes plus
//    normalized working bytes, UUID/content identity, import
//    format/dimensions and role-independent provenance) is exactly one
//    dedicated OCAF attribute on a child label of the document-owned
//    resource root child of document->Main() (never a shape label), with one
//    bounded binary driver on the review-accepted unique-GUID route.
//    Registration into OcctDocument's two existing driver chains and the
//    whole-document admission hooks are portion 2; this header supplies the
//    closed attribute/reader/writer definitions only.
//
// Both families are derivative records: they never grant authority to mutate
// any source BRep.
#include "FaceImageDefinition.hxx"
#include "RetainedFinishingAttribute.hxx"
#include <TDF_Attribute.hxx>
#include <TDF_AttributeIterator.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_UAttribute.hxx>
#include <TCollection_AsciiString.hxx>
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
#include <string>

class OcctDocument;
namespace core3d::face_image {
#if DEBUG
struct FaceImageProbe;
#endif
namespace persistence {

// Hex codec for the binding child-label payload. Lower-case, two characters
// per byte; the strict reader admits exactly this alphabet.
inline bool EncodeHex(const std::vector<std::uint8_t>& bytes, std::string& output) noexcept {
    output.clear();
    try {
        if (bytes.empty() || bytes.size() > kMaximumBytes) return false;
        static constexpr char digits[] = "0123456789abcdef";
        std::string text;
        text.reserve(2 * bytes.size());
        for (std::uint8_t byte : bytes) {
            text.push_back(digits[byte >> 4]);
            text.push_back(digits[byte & 15]);
        }
        output = std::move(text);
        return true;
    } catch (...) { output.clear(); return false; }
}
inline bool DecodeHex(const std::string& text, std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (text.empty() || (text.size() & 1) != 0 || text.size() > 2 * kMaximumBytes) return false;
        std::vector<std::uint8_t> bytes;
        bytes.reserve(text.size() / 2);
        for (std::size_t index = 0; index < text.size(); index += 2) {
            unsigned value = 0;
            for (int half = 0; half < 2; ++half) {
                const char c = text[index + std::size_t(half)];
                if (c < '0' || (c > '9' && (c < 'a' || c > 'f'))) return false;
                value = value * 16 + unsigned(c >= 'a' ? c - 'a' + 10 : c - '0');
            }
            bytes.push_back(std::uint8_t(value));
        }
        output = std::move(bytes);
        return true;
    } catch (...) { output.clear(); return false; }
}
inline bool HashHex(const std::vector<std::uint8_t>& bytes, std::string& output) noexcept {
    output.clear();
    try {
        Digest digest{};
        if (!HashFaceImageBytes(bytes, digest)) return false;
        static constexpr char digits[] = "0123456789abcdef";
        std::string text;
        text.reserve(2 * digest.size());
        for (std::uint8_t byte : digest) {
            text.push_back(digits[byte >> 4]);
            text.push_back(digits[byte & 15]);
        }
        output = std::move(text);
        return true;
    } catch (...) { output.clear(); return false; }
}

namespace bindings {

inline const Standard_GUID& MarkerID() {
    static const Standard_GUID id("E278B1A5-1E3F-4C2A-8B3D-6F0E9A4C7D21"); return id;
}
inline const Standard_GUID& VersionID() {
    static const Standard_GUID id("E278B1A5-1E3F-4C2A-8B3D-6F0E9A4C7D22"); return id;
}
inline const Standard_GUID& BindingCountID() {
    static const Standard_GUID id("E278B1A5-1E3F-4C2A-8B3D-6F0E9A4C7D23"); return id;
}
inline const Standard_GUID& ChunkCountID() {
    static const Standard_GUID id("E278B1A5-1E3F-4C2A-8B3D-6F0E9A4C7D24"); return id;
}
inline const Standard_GUID& DigestID() {
    static const Standard_GUID id("E278B1A5-1E3F-4C2A-8B3D-6F0E9A4C7D25"); return id;
}
// Fixed dedicated child tag under the owner label. Verified disjoint from the
// persisted transform tags 1..8, the legacy appearance tags 11/12, the tag-13
// recipe carriers and their incrementally allocated provenance records, and
// the tag-17 finishing receipt.
inline constexpr int RecordTag = 278;
inline constexpr Standard_Size ChunkCharacters = 256;
inline constexpr Standard_Size MaximumChunks = 320;

enum class ReadState : int { Absent = 0, Malformed = 1, Present = 2 };

inline bool HasSchemaAttribute(const TDF_Label& label) noexcept {
    try {
        Handle(TDF_Attribute) value;
        return label.FindAttribute(MarkerID(), value)
            || label.FindAttribute(VersionID(), value)
            || label.FindAttribute(BindingCountID(), value)
            || label.FindAttribute(ChunkCountID(), value)
            || label.FindAttribute(DigestID(), value);
    } catch (...) { return true; } // Unknown metadata cannot become absence.
}

// Strict reader. Absent: no record. Malformed: any schema violation (schema
// attributes on the owner itself or below any other descendant, a record at
// the wrong tag, unknown or missing attributes, bad counts, bad digest text,
// bad hex, a non-canonical payload, or a binding-count mismatch). Present:
// exact canonical SYFI/1 bytes and digest. Semantic staleness against the
// current B2 receipts and resource manifest is the owner transaction's fence
// comparison, never this reader's verdict.
inline ReadState Read(const Handle(TDocStd_Document)& document, const TDF_Label& owner,
                      Definition& output, std::vector<std::uint8_t>* bytes = nullptr,
                      TDF_Label* recordLabel = nullptr) noexcept {
    output = {};
    if (bytes) bytes->clear();
    if (recordLabel) *recordLabel = TDF_Label();
    try {
        if (document.IsNull() || document->GetData().IsNull() || owner.IsNull()
            || owner.Data() != document->GetData() || HasSchemaAttribute(owner))
            return ReadState::Malformed;
        TDF_Label record;
        // An owner owns at most one direct record. Schema fragments below any
        // other descendant are malformed rather than silently absent.
        int descendants = 0;
        for (TDF_ChildIterator child(owner, Standard_True); child.More(); child.Next()) {
            if (++descendants > profile::MaximumLabels) return ReadState::Malformed;
            if (!HasSchemaAttribute(child.Value())) continue;
            Handle(TDF_Attribute) marker;
            if (!child.Value().Father().IsEqual(owner)
                || child.Value().Tag() != RecordTag
                || !child.Value().FindAttribute(MarkerID(), marker)
                || Handle(TDataStd_UAttribute)::DownCast(marker).IsNull()
                || !record.IsNull()) return ReadState::Malformed;
            record = child.Value();
        }
        if (record.IsNull()) return ReadState::Absent;
        Handle(TDataStd_Integer) version, bindingCount, chunks;
        Handle(TDataStd_AsciiString) digest;
        if (!record.FindAttribute(VersionID(), version)
            || !record.FindAttribute(BindingCountID(), bindingCount)
            || !record.FindAttribute(ChunkCountID(), chunks)
            || !record.FindAttribute(DigestID(), digest)
            || version.IsNull() || bindingCount.IsNull() || chunks.IsNull() || digest.IsNull()
            || version->Get() != 1
            || bindingCount->Get() <= 0 || bindingCount->Get() > Standard_Integer(kMaximumBindings)
            || chunks->Get() <= 0 || chunks->Get() > Standard_Integer(MaximumChunks))
            return ReadState::Malformed;
        int recordAttributes = 0;
        for (TDF_AttributeIterator attribute(record); attribute.More(); attribute.Next()) {
            const auto& id = attribute.Value()->ID();
            if (id != MarkerID() && id != VersionID() && id != BindingCountID()
                && id != ChunkCountID() && id != DigestID())
                return ReadState::Malformed;
            ++recordAttributes;
        }
        const std::string digestText = digest->Get().ToCString();
        if (recordAttributes != 5 || digestText.size() != 64) return ReadState::Malformed;
        for (char c : digestText) if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')))
            return ReadState::Malformed;
        std::string hex;
        for (Standard_Integer index = 1; index <= chunks->Get(); ++index) {
            const TDF_Label chunk = record.FindChild(index, Standard_False);
            Handle(TDataStd_AsciiString) text;
            int attributes = 0;
            if (chunk.IsNull()
                || !chunk.FindAttribute(TDataStd_AsciiString::GetID(), text) || text.IsNull())
                return ReadState::Malformed;
            for (TDF_AttributeIterator attribute(chunk); attribute.More(); attribute.Next()) {
                if (attribute.Value()->ID() != TDataStd_AsciiString::GetID())
                    return ReadState::Malformed;
                ++attributes;
            }
            const std::string value = text->Get().ToCString();
            if (attributes != 1 || value.empty() || value.size() > std::size_t(ChunkCharacters)
                || (index < chunks->Get() && value.size() != std::size_t(ChunkCharacters)))
                return ReadState::Malformed;
            for (char c : value) if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f')))
                return ReadState::Malformed;
            hex += value;
        }
        int directChildren = 0;
        for (TDF_ChildIterator child(record, Standard_False); child.More(); child.Next()) {
            if (++directChildren > profile::MaximumLabels) return ReadState::Malformed;
            if (child.Value().Tag() > chunks->Get() && child.Value().HasAttribute())
                return ReadState::Malformed;
            int nested = 0;
            for (TDF_ChildIterator below(child.Value(), Standard_True); below.More(); below.Next())
                if (++nested > profile::MaximumLabels || below.Value().HasAttribute())
                    return ReadState::Malformed;
        }
        std::vector<std::uint8_t> canonical;
        Definition value; Refusal refusal = Refusal::None;
        if (!DecodeHex(hex, canonical) || !Decode(canonical, value, refusal)
            || value.bindings.size() != std::size_t(bindingCount->Get()))
            return ReadState::Malformed;
        std::string actualDigest;
        if (!HashHex(canonical, actualDigest) || actualDigest != digestText)
            return ReadState::Malformed;
        output = std::move(value);
        if (bytes) *bytes = std::move(canonical);
        if (recordLabel) *recordLabel = record;
        return ReadState::Present;
    } catch (...) {
        output = {};
        if (bytes) bytes->clear();
        if (recordLabel) *recordLabel = TDF_Label();
        return ReadState::Malformed;
    }
}

// Write the schema attributes and payload chunks onto a fresh or cleared
// record label. The caller owns validation, command scope and readback.
inline bool WriteChunks(const TDF_Label& record, const std::string& hex,
                        Standard_Integer bindingCount, const std::string& digest) noexcept {
    try {
        const std::size_t chunkCount =
            (hex.size() + std::size_t(ChunkCharacters) - 1) / std::size_t(ChunkCharacters);
        if (hex.empty() || chunkCount == 0 || chunkCount > std::size_t(MaximumChunks)
            || bindingCount <= 0 || digest.size() != 64) return false;
        TDataStd_UAttribute::Set(record, MarkerID());
        TDataStd_Integer::Set(record, VersionID(), 1);
        TDataStd_Integer::Set(record, BindingCountID(), bindingCount);
        TDataStd_Integer::Set(record, ChunkCountID(), Standard_Integer(chunkCount));
        TDataStd_AsciiString::Set(record, DigestID(), TCollection_AsciiString(digest.c_str()));
        for (std::size_t index = 0; index < chunkCount; ++index) {
            const std::string chunk = hex.substr(index * std::size_t(ChunkCharacters),
                std::size_t(ChunkCharacters));
            TDataStd_AsciiString::Set(record.FindChild(Standard_Integer(index + 1), Standard_True),
                TCollection_AsciiString(chunk.c_str()));
        }
        return true;
    } catch (...) { return false; }
}

// Whole-record commit seam. The caller holds the already-open OCAF command
// under the existing document mutation owner; refusal leaves no delta, and a
// later command abort restores the prior record exactly. The candidate owner
// must match the resolved owner label's entity/definition identities and the
// document identity; the compact binding metadata must fit both frozen caps
// (the SYFI/1 record cap and the 320 x 256 chunk ceiling) before any
// mutation.
inline bool StageCommitted(const Handle(TDocStd_Document)& document,
                           const TDF_Label& owner, const Definition& candidate,
                           const std::vector<std::uint8_t>& canonical) noexcept {
    try {
        if (document.IsNull() || !document->HasOpenCommand() || owner.IsNull()
            || owner.Data() != document->GetData()
            || !XCAFDoc_ShapeTool::IsSimpleShape(owner) || !XCAFDoc_ShapeTool::IsFree(owner))
            return false;
        std::vector<std::uint8_t> exact;
        if (!Encode(candidate, exact) || exact != canonical) return false;
        UUID documentID{}, entity{}, definition{};
        if (!retained_solid::ReadUUID(document->Main(),
                Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)
            || !retained_solid::ReadUUID(owner,
                Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"), entity)
            || !retained_solid::ReadUUID(owner,
                Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"), definition)
            || !(candidate.owner.document == documentID)
            || !(candidate.owner.entity == entity)
            || !(candidate.owner.definition == definition)) return false;
        std::string hex, digest;
        if (!EncodeHex(canonical, hex)
            || hex.size() > std::size_t(MaximumChunks) * std::size_t(ChunkCharacters)
            || !HashHex(canonical, digest)) return false;
        Definition prior; TDF_Label record;
        const ReadState state = Read(document, owner, prior, nullptr, &record);
        if (state == ReadState::Malformed) return false;
        if (record.IsNull()) {
            record = owner.FindChild(RecordTag, Standard_True);
            if (record.IsNull()) return false;
            for (TDF_AttributeIterator it(record); it.More(); it.Next()) return false;
        } else {
            record.ForgetAllAttributes(Standard_True);
        }
        if (!WriteChunks(record, hex, Standard_Integer(candidate.bindings.size()), digest))
            return false;
        Definition readback;
        std::vector<std::uint8_t> readbackBytes;
        return Read(document, owner, readback, &readbackBytes) == ReadState::Present
            && readbackBytes == canonical && readback == candidate;
    } catch (...) { return false; }
}

// Record removal (last-binding removal). Runs inside the caller's open OCAF
// command; undo restores the attributes exactly.
inline bool Remove(const Handle(TDocStd_Document)& document, const TDF_Label& owner) noexcept {
    try {
        if (document.IsNull() || !document->HasOpenCommand()) return false;
        Definition prior; TDF_Label record;
        const ReadState state = Read(document, owner, prior, nullptr, &record);
        if (state == ReadState::Malformed) return false;
        if (state == ReadState::Absent || record.IsNull()) return true;
        record.ForgetAllAttributes(Standard_True);
        return Read(document, owner, prior) == ReadState::Absent;
    } catch (...) { return false; }
}
} // namespace bindings

namespace resources {

// Persistent schema identifier. Never reuse or renumber.
inline const Standard_GUID& AttributeID() {
    static const Standard_GUID id("E278B1A5-1E3F-4C2A-8B3D-6F0E9A4C7D31"); return id;
}
// Document-owned resource root child tag under document->Main(). Fixed
// schema tag; verified disjoint from every other fixed Main-child tag
// (pattern 71, path-array 72, feature-pattern 73, debug-only 200, atlas 271).
inline constexpr int RootTag = 278;
inline constexpr std::size_t MaximumAggregateEnvelopeBytes = kMaximumAggregateResourceBytes;

struct Payload final {
    ResourceEnvelope envelope;
    std::vector<std::uint8_t> bytes; // canonical SYFR/1, digest-checked
};

class Core3D_FaceImageResource final : public TDF_Attribute {
public:
    DEFINE_STANDARD_RTTI_INLINE(Core3D_FaceImageResource, TDF_Attribute)
    const Standard_GUID& ID() const override { return AttributeID(); }
    Handle(TDF_Attribute) NewEmpty() const override { return new Core3D_FaceImageResource(); }
    const std::shared_ptr<const Payload>& value() const noexcept { return value_; }
    void Restore(const Handle(TDF_Attribute)& source) override {
        const auto original = Handle(Core3D_FaceImageResource)::DownCast(source);
        if (original.IsNull()) Standard_Failure::Raise("Face image resource restore type");
        // Backup/Undo/Redo share immutable payload ownership; an envelope is
        // never reparsed or rewritten by history traversal.
        value_ = original->value_;
    }
    void Paste(const Handle(TDF_Attribute)& target, const Handle(TDF_RelocationTable)&) const override {
        const auto destination = Handle(Core3D_FaceImageResource)::DownCast(target);
        if (destination.IsNull() || !value_) Standard_Failure::Raise("Face image resource paste type");
        // IDs remain exact on cross-document copy; read-back rejects a
        // foreign identity instead of manufacturing a new one.
        destination->Backup(); destination->value_ = value_;
    }
    // Whole-envelope adoption seam. The caller holds the already-open OCAF
    // command under the existing document mutation owner; refusal leaves no
    // delta, and a later command abort restores the prior table exactly. The
    // candidate must encode canonically and the resulting table must stay
    // inside the single aggregate budget; a duplicate identity or a foreign
    // attribute in the table refuses.
    static bool StageCommitted(const Handle(TDocStd_Document)& document,
                               const std::shared_ptr<const Payload>& payload) noexcept {
        try {
            if (document.IsNull() || !document->HasOpenCommand() || !payload
                || document->GetData().IsNull()) return false;
            std::vector<std::uint8_t> canonical;
            if (!Encode(payload->envelope, canonical) || canonical != payload->bytes)
                return false;
            const TDF_Label root = document->Main().FindChild(RootTag, Standard_True);
            if (root.IsNull() || root.HasAttribute()) return false;
            std::size_t aggregate = canonical.size();
            std::size_t records = 0;
            int highest = 0; int visited = 0;
            for (TDF_ChildIterator child(root, Standard_False); child.More(); child.Next()) {
                if (++visited > profile::MaximumLabels) return false;
                highest = std::max(highest, child.Value().Tag());
                if (!child.Value().IsAttribute(AttributeID())) {
                    if (child.Value().HasAttribute()) return false;
                    continue;
                }
                Handle(Core3D_FaceImageResource) attribute;
                if (!child.Value().FindAttribute(AttributeID(), attribute) || attribute.IsNull()
                    || !attribute->value()) return false;
                std::vector<std::uint8_t> exact;
                if (!Encode(attribute->value()->envelope, exact)
                    || exact != attribute->value()->bytes) return false;
                if (attribute->value()->envelope.resource == payload->envelope.resource)
                    return false;
                if (exact.size() > MaximumAggregateEnvelopeBytes - aggregate) return false;
                aggregate += exact.size();
                ++records;
            }
            if (records >= std::size_t(profile::MaximumRecords) / 2
                || highest >= profile::MaximumLabels) return false;
            const TDF_Label record = root.FindChild(highest + 1, Standard_True);
            if (record.IsNull()) return false;
            for (TDF_AttributeIterator it(record); it.More(); it.Next()) return false;
            Handle(Core3D_FaceImageResource) attribute = new Core3D_FaceImageResource();
            record.AddAttribute(attribute);
            attribute->Backup();
            attribute->value_ = payload;
            return true;
        } catch (...) { return false; }
    }
private:
    friend class BinaryDriver;
    friend class ::OcctDocument;
#if DEBUG
    friend struct core3d::face_image::FaceImageProbe;
#endif
    std::shared_ptr<const Payload> value_;
};
using Attribute = Core3D_FaceImageResource;

struct Record {
    TDF_Label label;
    std::shared_ptr<const Payload> value;
};

class BinaryDriver;

// Whole-document read-back: exact resource identities, canonical SYFR/1
// bytes and the single aggregate budget. Any malformed, duplicated or
// over-budget envelope refuses the document rather than degrading to
// absence; there is no per-binding budget multiplication.
inline bool ReadAll(const Handle(TDocStd_Document)& document,
                    std::vector<Record>& output) noexcept {
    output.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const TDF_Label main = document->Main();
        if (main.IsNull() || main.IsAttribute(AttributeID())) return false;
        const TDF_Label root = main.FindChild(RootTag, Standard_False);
        if (root.IsNull()) return true; // no resource root: zero records
        if (root.HasAttribute()) return false;
        std::vector<Record> staged;
        std::size_t aggregate = 0; int visited = 0;
        for (TDF_ChildIterator child(root, Standard_False); child.More(); child.Next()) {
            if (++visited > profile::MaximumLabels
                || staged.size() >= std::size_t(profile::MaximumRecords) / 2) return false;
            const TDF_Label label = child.Value();
            if (!label.IsAttribute(AttributeID())) {
                if (label.HasAttribute()) return false;
                int nested = 0;
                for (TDF_ChildIterator below(label, Standard_True); below.More(); below.Next())
                    if (++nested > profile::MaximumLabels || below.Value().HasAttribute())
                        return false;
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
            if (!Encode(value->envelope, exact) || exact != value->bytes
                || exact.size() > MaximumAggregateEnvelopeBytes - aggregate) return false;
            aggregate += exact.size();
            for (const auto& prior : staged)
                if (prior.value->envelope.resource == value->envelope.resource) return false;
            staged.push_back({label, value});
        }
        output = std::move(staged);
        return true;
    } catch (...) { output.clear(); return false; }
}

// Local absence matches the existing family-reader contract. Whole-document
// admission remains mandatory at save, native load and resource use.
inline bool Read(const Handle(TDocStd_Document)& document, const UUID& resource,
                 Record& output) noexcept {
    output = {};
    try {
        if (document.IsNull() || !retained_recipe::Nonzero(resource)) return false;
        std::vector<Record> all;
        if (!ReadAll(document, all)) return false;
        for (const Record& record : all)
            if (record.value->envelope.resource == resource) { output = record; break; }
        return true;
    } catch (...) { output = {}; return false; }
}

// Envelope removal (explicit resource removal). Runs inside the caller's
// open OCAF command; undo restores the attribute exactly.
inline bool Remove(const Handle(TDocStd_Document)& document, const UUID& resource) noexcept {
    try {
        if (document.IsNull() || !document->HasOpenCommand()) return false;
        Record record;
        if (!Read(document, resource, record)) return false;
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
    std::size_t limit = MaximumAggregateEnvelopeBytes;
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
                || count <= 0 || count > Standard_Integer(kMaximumEnvelopeBytes)
                || source.Position() > end || count > end - source.Position()
                || budget_->limit > MaximumAggregateEnvelopeBytes
                || budget_->bytes > budget_->limit
                || std::size_t(count) > budget_->limit - budget_->bytes
                || budget_->records >= std::size_t(profile::MaximumRecords) / 2)
                return Refuse();
            std::vector<std::uint8_t> bytes(std::size_t(count), 0);
            if (!source.GetByteArray(bytes.data(), count)) return Refuse();
            ResourceEnvelope envelope; Refusal refusal = Refusal::None;
            if (!Decode(bytes, envelope, refusal) || source.Position() != end)
                return Refuse();
            auto value = std::make_shared<Payload>();
            value->envelope = std::move(envelope);
            value->bytes = std::move(bytes);
            // Publish only after complete digest-checked decode. Identity and
            // aggregate admission runs before document adoption.
            attribute->value_ = std::move(value);
            budget_->bytes += std::size_t(count); ++budget_->records;
            return Standard_True;
        } catch (...) { return Refuse(); }
    }

    void Paste(const Handle(TDF_Attribute)& source, BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable&) const override {
        const auto attribute = Handle(Attribute)::DownCast(source);
        if (attribute.IsNull() || !attribute->value_ || attribute->Label().IsNull())
            Standard_Failure::Raise("Face image resource writer source");
        const auto document = TDocStd_Document::Get(attribute->Label());
        if (document.IsNull()
            || document->StorageFormatVersion() < TDocStd_FormatVersion_VERSION_10
            || document->StorageFormatVersion() > TDocStd_FormatVersion_CURRENT)
            Standard_Failure::Raise("Face image resource writer version");
        const auto& value = *attribute->value_;
        std::vector<std::uint8_t> encoded;
        if (!Encode(value.envelope, encoded) || encoded != value.bytes)
            Standard_Failure::Raise("Face image resource writer record");
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
// error surfaced by document admission, never a fallback. Portion 2 calls
// this from OcctDocument's two existing driver chains.
inline void Register(const Handle(BinMDF_ADriverTable)& table,
                     const Handle(Message_Messenger)& messenger,
                     const std::shared_ptr<ReadBudget>& budget,
                     void (*reject)() noexcept = nullptr) {
    if (table.IsNull()) Standard_Failure::Raise("Face image resource driver table");
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
        // The strict whole-document read is the admission: any malformed,
        // duplicated or over-budget envelope fails the save closed.
        if (!ReadAll(document, records))
            Standard_Failure::Raise("Face image resource writer table");
    }
};
} // namespace resources
} // namespace persistence

// Bounded owner staging and resource store API. Implemented by
// FaceImageOwner.mm. The caller supplies the observed currentness fences
// captured by the existing document authority; this seam never manufactures
// currentness itself and never opens its own mutation route.
namespace owner {
enum class Outcome : std::uint8_t {
    Prepared, Committed, Refused, StaleSource, StaleFace, AmbiguousFaceRemap,
    UnsupportedSurface, MissingResource, StaleResource, ForeignResource,
    UnsupportedDownstream, OwnerMismatch, Busy, Malformed, PersistenceFailure
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
                const Definition& candidate, const Observed& observed) noexcept;
Outcome Commit(Staging& staging, const Handle(TDocStd_Document)& document,
               const Observed& observed) noexcept;
void Cancel(Staging& staging) noexcept;

// Bounded document-owned resource store. Adoption revalidates the envelope
// and its carried content hashes; a duplicate identity with identical bytes
// is a zero-delta refusal, with different bytes a foreign identity.
Outcome AdoptResource(const Handle(TDocStd_Document)& document,
                      const ResourceEnvelope& candidate) noexcept;
bool ReadResource(const Handle(TDocStd_Document)& document, const UUID& resource,
                  ResourceEnvelope& output) noexcept;
bool ResourceManifest(const Handle(TDocStd_Document)& document,
                      std::vector<ResourceFence>& output) noexcept;
Outcome RemoveResource(const Handle(TDocStd_Document)& document,
                       const UUID& resource) noexcept;
} // namespace owner
} // namespace core3d::face_image
