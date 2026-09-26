#pragma once
#include "BoundedCurveCodec.hxx"
#include "RetainedSolidAttribute.hxx"
#include <TDF_Attribute.hxx>
#include <TDF_AttributeIterator.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelMap.hxx>
#include <TNaming_NamedShape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <memory>

class OcctDocument;
namespace core3d::bounded_curve {
class BinaryDriver;
#if DEBUG
struct PersistenceProbe;
#endif

inline const Standard_GUID& AttributeID() {
    static const Standard_GUID id("76259072-60B6-4E56-90ED-79C2D4245B77"); return id;
}

struct Payload final {
    PersistedValue persisted;
    std::vector<std::uint8_t> definitionBytes;
    std::vector<std::uint8_t> ownerBytes;
};

class Core3D_BoundedCurve final : public TDF_Attribute {
public:
    DEFINE_STANDARD_RTTI_INLINE(Core3D_BoundedCurve, TDF_Attribute)
    const Standard_GUID& ID() const override { return AttributeID(); }
    Handle(TDF_Attribute) NewEmpty() const override { return new Core3D_BoundedCurve(); }
    const std::shared_ptr<const Payload>& value() const noexcept { return value_; }
    void Restore(const Handle(TDF_Attribute)& source) override {
        const auto original = Handle(Core3D_BoundedCurve)::DownCast(source);
        if (original.IsNull()) Standard_Failure::Raise("Bounded curve restore type");
        value_ = original->value_;
    }
    void Paste(const Handle(TDF_Attribute)& target, const Handle(TDF_RelocationTable)&) const override {
        const auto destination = Handle(Core3D_BoundedCurve)::DownCast(target);
        if (destination.IsNull() || !value_) Standard_Failure::Raise("Bounded curve paste type");
        destination->Backup(); destination->value_ = value_;
    }
private:
    friend class BinaryDriver;
    friend class ::OcctDocument;
#if DEBUG
    friend struct PersistenceProbe;
#endif
    std::shared_ptr<const Payload> value_;
};
using Attribute = Core3D_BoundedCurve;

struct Record {
    TDF_Label label, owner;
    std::shared_ptr<const Payload> value;
    TopoDS_Shape current;
};

inline bool HasRecord(const TDF_Label& owner) noexcept {
    try {
        if (owner.IsNull()) return false;
        if (owner.IsAttribute(AttributeID())) return true;
        int visited = 0;
        for (TDF_ChildIterator child(owner, Standard_False); child.More(); child.Next())
            if (++visited > profile::MaximumLabels || child.Value().IsAttribute(AttributeID())) return true;
        return false;
    } catch (...) { return true; }
}

inline bool ReadAll(const Handle(TDocStd_Document)& document,
                    std::vector<Record>& output) noexcept {
    output.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const TDF_Label root = document->GetData()->Root();
        if (root.IsAttribute(AttributeID())) return false;
        std::vector<Record> staged; staged.reserve(MaximumRecordsPerDocument);
        TDF_LabelMap recordLabels; std::set<UUID> features;
        std::size_t definitionAggregate = 0, ownerAggregate = 0; int visited = 0;
        for (TDF_ChildIterator iterator(root, Standard_True); iterator.More(); iterator.Next()) {
            if (++visited > profile::MaximumLabels) return false;
            const TDF_Label label = iterator.Value(); if (!label.IsAttribute(AttributeID())) continue;
            if (staged.size() >= MaximumRecordsPerDocument || !recordLabels.Add(label)) return false;
            Handle(Attribute) attribute; Handle(TNaming_NamedShape) binding;
            const TDF_Label owner = label.Father();
            if (label.Tag() < MinimumRecordTag
                || !label.FindAttribute(AttributeID(), attribute) || attribute.IsNull()
                || !attribute->value() || !label.FindAttribute(TNaming_NamedShape::GetID(), binding)
                || binding.IsNull()) return false;
            const auto value = attribute->value(); std::vector<std::uint8_t> exactDefinition, exactOwner;
            if (!ValidatePersisted(value->persisted, &exactDefinition, &exactOwner)
                || exactDefinition != value->definitionBytes || exactOwner != value->ownerBytes
                || !features.insert(value->persisted.value.feature).second
                || exactDefinition.size() > MaximumDocumentAggregateBytes - definitionAggregate
                || exactOwner.size() > MaximumDocumentAggregateBytes - ownerAggregate) return false;
            definitionAggregate += exactDefinition.size(); ownerAggregate += exactOwner.size();
            UUID documentID{}, entityID{}, definitionID{};
            if (!retained_solid::ReadUUID(document->Main(),
                    Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)
                || !retained_solid::ReadUUID(owner,
                    Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"), entityID)
                || !retained_solid::ReadUUID(owner,
                    Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"), definitionID)
                || value->persisted.ownerState.owner.document != documentID
                || value->persisted.ownerState.owner.entity != entityID
                || value->persisted.ownerState.owner.definition != definitionID) return false;
            const TopoDS_Shape current = binding->Get();
            if (current.IsNull() || current.ShapeType() != TopAbs_WIRE
                || current.Orientation() != TopAbs_FORWARD) return false;
            for (TDF_AttributeIterator attributes(label); attributes.More(); attributes.Next())
                if (attributes.Value()->ID() != AttributeID()
                    && attributes.Value()->ID() != TNaming_NamedShape::GetID()) return false;
            int descendants = 0;
            for (TDF_ChildIterator child(label, Standard_True); child.More(); child.Next())
                if (++descendants > profile::MaximumLabels || child.Value().HasAttribute()) return false;
            staged.push_back({label, owner, value, current});
        }
        output = std::move(staged); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Read(const Handle(TDocStd_Document)& document, const TDF_Label& owner,
                 Record& output) noexcept {
    output = {};
    if (document.IsNull() || owner.IsNull() || owner.Data() != document->GetData()) return false;
    if (!HasRecord(owner)) return true;
    std::vector<Record> records; if (!ReadAll(document, records)) return false;
    for (const Record& record : records)
        if (record.owner.IsEqual(owner)) { output = record; return true; }
    return false;
}

} // namespace core3d::bounded_curve
