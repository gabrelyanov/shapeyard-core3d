#pragma once

#include "BoundedCurveAttribute.hxx"
#include "RetainedSolidAttribute.hxx"
#include "TextReliefDefinition.hxx"

#include <BinMDF_ADriver.hxx>
#include <BinMDF_ADriverTable.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <TDF_AttributeIterator.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelMap.hxx>
#include <TNaming_Builder.hxx>
#include <TNaming_NamedShape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>

#include <memory>
#include <map>
#include <set>
#include <vector>

namespace core3d::text_relief::persistence {

inline constexpr std::size_t MaximumRecords = 128;
inline constexpr std::size_t MaximumAggregateBytes = 8 * 1024 * 1024;
inline constexpr int MinimumRecordTag = 83;

inline const Standard_GUID& AttributeID() {
    static const Standard_GUID id("61032E4A-3506-44E2-995A-BFA5954AA14C");
    return id;
}

struct Payload final {
    AuthoredRecord value;
    std::vector<std::uint8_t> bytes;
};

class BinaryDriver;
class Attribute final : public TDF_Attribute {
public:
    // Unique persisted type name. The macro-derived bare "Attribute" collides
    // with sibling families (loft correspondence, general loft) in the
    // reader's name-keyed driver assignment (NCollection_DoubleMap::Bind out
    // of AssignIds), so this family registers its own unambiguous name.
    typedef TDF_Attribute base_type;
    static const char* get_type_name() { return "Core3D_TextRelief";
        OCCT_CHECK_BASE_CLASS(Attribute, TDF_Attribute) }
    static const Handle(Standard_Type)& get_type_descriptor() {
        return Standard_Type::Instance<Attribute>();
    }
    const Handle(Standard_Type)& DynamicType() const override {
        return get_type_descriptor();
    }
    const Standard_GUID& ID() const override { return AttributeID(); }
    Handle(TDF_Attribute) NewEmpty() const override { return new Attribute(); }
    void Restore(const Handle(TDF_Attribute)& source) override {
        const auto typed = Handle(Attribute)::DownCast(source);
        if (typed.IsNull()) Standard_Failure::Raise("SYTR/2 restore type");
        payload_ = typed->payload_;
    }
    void Paste(const Handle(TDF_Attribute)& target,
               const Handle(TDF_RelocationTable)&) const override {
        const auto typed = Handle(Attribute)::DownCast(target);
        if (typed.IsNull() || !payload_)
            Standard_Failure::Raise("SYTR/2 paste type");
        typed->Backup(); typed->payload_ = payload_;
    }
    const std::shared_ptr<const Payload>& payload() const noexcept {
        return payload_;
    }
private:
    friend class BinaryDriver;
    friend bool Install(const TDF_Label&, const AuthoredRecord&,
                        const TopoDS_Shape&) noexcept;
    friend bool Replace(const TDF_Label&, const AuthoredRecord&,
                        const TopoDS_Shape&) noexcept;
    std::shared_ptr<const Payload> payload_;
};

struct Record final {
    TDF_Label label, owner;
    std::shared_ptr<const Payload> payload;
    TopoDS_Shape shape;
};

inline bool HasRecord(const TDF_Label& owner) noexcept {
    try {
        if (owner.IsNull()) return false;
        if (owner.IsAttribute(AttributeID())) return true;
        std::size_t visited = 0;
        for (TDF_ChildIterator child(owner, Standard_False); child.More(); child.Next())
            if (++visited > 256 || child.Value().IsAttribute(AttributeID())) return true;
        return false;
    } catch (...) { return true; }
}

inline bool ReadAll(const Handle(TDocStd_Document)& document,
                    std::vector<Record>& output) noexcept {
    output.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()
            || !XCAFDoc_DocumentTool::CheckShapeTool(document->Main())) return false;
        const TDF_Label root = document->GetData()->Root();
        if (root.IsAttribute(AttributeID())) return false;
        std::vector<bounded_curve::Record> c1;
        if (!bounded_curve::ReadAll(document, c1)) return false;
        std::map<UUID, Digest> c1Digests;
        for (const auto& curve : c1) {
            Digest digest{};
            if (!curve.value
                || !bounded_curve::Hash(curve.value->definitionBytes,
                                        bounded_curve::MaximumDefinitionBytes, digest)
                || !c1Digests.emplace(curve.value->persisted.value.feature,
                                      digest).second) return false;
        }
        std::vector<Record> staged; std::set<UUID> features, roots;
        TDF_LabelMap owners, labels; std::size_t aggregate = 0, visited = 0;
        for (TDF_ChildIterator it(root, Standard_True); it.More(); it.Next()) {
            if (++visited > 50'000) return false;
            const TDF_Label label = it.Value();
            if (!label.IsAttribute(AttributeID())) continue;
            if (staged.size() >= MaximumRecords || !labels.Add(label)) return false;
            const TDF_Label owner = XCAFDoc_ShapeTool::IsFree(label)
                ? label : label.Father();
            Handle(Attribute) attribute;
            if (owner.IsNull() || !owners.Add(owner)
                || (!label.IsEqual(owner) && label.Tag() < MinimumRecordTag)
                || !label.FindAttribute(AttributeID(), attribute)
                || attribute.IsNull() || !attribute->payload()
                || (!label.IsEqual(owner)
                    && label.IsAttribute(TNaming_NamedShape::GetID()))) return false;
            const auto payload = attribute->payload(); AuthoredRecord decoded;
            std::vector<std::uint8_t> exact;
            if (payload->bytes.size() > MaximumAggregateBytes - aggregate
                || !DecodeAuthored(payload->bytes, decoded)
                || !EncodeAuthored(decoded, exact) || exact != payload->bytes
                || decoded.feature != payload->value.feature
                || decoded.owner != payload->value.owner
                || decoded.revision != payload->value.revision
                || decoded.rootOutput != decoded.owner.entity
                || !features.insert(decoded.feature).second
                || !roots.insert(decoded.rootOutput).second) return false;
            aggregate += payload->bytes.size();
            UUID documentID{}, entityID{}, definitionID{};
            if (!retained_solid::ReadUUID(document->Main(),
                    Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)
                || !retained_solid::ReadUUID(owner,
                    Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"), entityID)
                || !retained_solid::ReadUUID(owner,
                    Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"), definitionID)
                || decoded.owner.document != documentID
                || decoded.owner.entity != entityID
                || decoded.owner.definition != definitionID
                || !XCAFDoc_ShapeTool::IsFree(owner)
                || XCAFDoc_ShapeTool::IsReference(owner)
                || XCAFDoc_ShapeTool::IsComponent(owner)) return false;
            const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(owner);
            if (shape.IsNull()) return false;
            for (const auto& ref : decoded.curves) {
                const auto found = c1Digests.find(ref.feature);
                if (found == c1Digests.end() || found->second != ref.digest) return false;
            }
            if (!label.IsEqual(owner)) {
                for (TDF_AttributeIterator attrs(label); attrs.More(); attrs.Next())
                    if (attrs.Value()->ID() != AttributeID()) return false;
                for (TDF_ChildIterator child(label, Standard_True); child.More(); child.Next())
                    if (child.Value().HasAttribute()) return false;
            }
            staged.push_back({label, owner, payload, shape});
        }
        output = std::move(staged); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Read(const Handle(TDocStd_Document)& document, const UUID& feature,
                 Record& output) noexcept {
    output = {};
    std::vector<Record> records;
    if (!ReadAll(document, records)) return false;
    for (const auto& record : records)
        if (record.payload->value.feature == feature) {
            if (!output.label.IsNull()) { output = {}; return false; }
            output = record;
        }
    return true;
}

inline bool Install(const TDF_Label& label, const AuthoredRecord& value,
                    const TopoDS_Shape& shape) noexcept {
    try {
        if (label.IsNull() || shape.IsNull()
            || label.IsAttribute(AttributeID())) return false;
        std::vector<std::uint8_t> bytes;
        if (!EncodeAuthored(value, bytes)) return false;
        auto payload = std::make_shared<Payload>();
        payload->value = value; payload->bytes = std::move(bytes);
        Handle(Attribute) attribute = new Attribute(); attribute->payload_ = payload;
        label.AddAttribute(attribute);
        return true;
    } catch (...) { return false; }
}

inline bool Replace(const TDF_Label& label, const AuthoredRecord& value,
                    const TopoDS_Shape& shape) noexcept {
    try {
        Handle(Attribute) attribute;
        if (label.IsNull() || shape.IsNull()
            || !label.FindAttribute(AttributeID(), attribute)
            || attribute.IsNull()) return false;
        std::vector<std::uint8_t> bytes;
        if (!EncodeAuthored(value, bytes)) return false;
        auto payload = std::make_shared<Payload>();
        payload->value = value; payload->bytes = std::move(bytes);
        attribute->Backup(); attribute->payload_ = std::move(payload);
        return true;
    } catch (...) { return false; }
}

struct ReadBudget final {
    std::size_t bytes = 0, records = 0; bool rejected = false;
    void reset() noexcept { bytes = records = 0; rejected = false; }
};

class BinaryDriver final : public BinMDF_ADriver {
public:
    BinaryDriver(const Handle(Message_Messenger)& messenger,
                 std::shared_ptr<ReadBudget> budget,
                 void (*reject)() noexcept = nullptr)
        : BinMDF_ADriver(messenger, STANDARD_TYPE(Attribute)->Name()),
          budget_(std::move(budget)), reject_(reject) {}
    Handle(TDF_Attribute) NewEmpty() const override { return new Attribute(); }
    const Handle(Standard_Type)& SourceType() const override {
        return STANDARD_TYPE(Attribute);
    }
    Standard_Boolean Paste(const BinObjMgt_Persistent& source,
                           const Handle(TDF_Attribute)& target,
                           BinObjMgt_RRelocationTable&) const override {
        try {
            const auto attribute = Handle(Attribute)::DownCast(target);
            Standard_Integer count = 0;
            if (attribute.IsNull() || attribute->payload_ || !budget_
                || budget_->rejected || !(source >> count) || count <= 0
                || count > Standard_Integer(MaximumAuthoredRecordBytes)
                || budget_->records >= MaximumRecords
                || std::size_t(count) > MaximumAggregateBytes - budget_->bytes)
                return Refuse();
            std::vector<std::uint8_t> bytes(std::size_t(count), 0);
            if (!source.GetByteArray(bytes.data(), count)) return Refuse();
            AuthoredRecord value;
            if (!DecodeAuthored(bytes, value)) return Refuse();
            auto payload = std::make_shared<Payload>();
            payload->value = std::move(value); payload->bytes = std::move(bytes);
            attribute->payload_ = std::move(payload);
            budget_->bytes += std::size_t(count); ++budget_->records;
            return Standard_True;
        } catch (...) { return Refuse(); }
    }
    void Paste(const Handle(TDF_Attribute)& source, BinObjMgt_Persistent& target,
               BinObjMgt_SRelocationTable&) const override {
        const auto attribute = Handle(Attribute)::DownCast(source);
        AuthoredRecord decoded; std::vector<std::uint8_t> canonical;
        if (attribute.IsNull() || !attribute->payload_
            || !DecodeAuthored(attribute->payload_->bytes, decoded)
            || !EncodeAuthored(decoded, canonical)
            || canonical != attribute->payload_->bytes)
            Standard_Failure::Raise("SYTR/2 writer source");
        target << Standard_Integer(canonical.size());
        target.PutByteArray(canonical.data(), Standard_Integer(canonical.size()));
    }
private:
    Standard_Boolean Refuse() const noexcept {
        if (budget_) budget_->rejected = true;
        if (reject_) reject_();
        return Standard_False;
    }
    std::shared_ptr<ReadBudget> budget_;
    void (*reject_)() noexcept;
};

inline void Register(const Handle(BinMDF_ADriverTable)& table,
                     const Handle(Message_Messenger)& messenger,
                     const std::shared_ptr<ReadBudget>& budget,
                     void (*reject)() noexcept = nullptr) {
    table->AddDriver(new BinaryDriver(messenger, budget, reject));
}

template<class Base> class StorageDriver : public Base {
public:
    Handle(BinMDF_ADriverTable) AttributeDrivers(
        const Handle(Message_Messenger)& messenger) override {
        auto table = Base::AttributeDrivers(messenger);
        Register(table, messenger, std::make_shared<ReadBudget>()); return table;
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
    static void Prepare(const Handle(CDM_Document)& document) {
        const Handle(TDocStd_Document) ocaf =
            Handle(TDocStd_Document)::DownCast(document);
        if (ocaf.IsNull() || ocaf->GetData().IsNull())
            Standard_Failure::Raise("SYTR/2 writer validation");
        // Documents without any SYTR/2 attribute validate exactly as before
        // this driver existed; full record proof applies only when the
        // document actually carries the family.
        bool present = false;
        for (TDF_ChildIterator it(ocaf->GetData()->Root(), Standard_True);
             it.More(); it.Next())
            if (it.Value().IsAttribute(AttributeID())) { present = true; break; }
        if (!present) return;
        std::vector<Record> records;
        if (!ReadAll(ocaf, records))
            Standard_Failure::Raise("SYTR/2 writer validation");
    }
};

} // namespace core3d::text_relief::persistence
