#pragma once
#include "CompositeRecipeCodec.hxx"
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
namespace core3d::composite_recipe {
class BinaryDriver;
#if DEBUG
struct Probe;
#endif

inline const Standard_GUID& AttributeID() {
    static const Standard_GUID id("8D89F123-AE6D-4C6D-8F71-38A4BC9B6B52"); return id;
}

struct Payload final {
    Definition definition;
    std::vector<std::uint8_t> bytes;
    // Indexed by SourceNode::shapeSlot. Immutable ownership is shared across
    // Undo/Redo; geometry workers must deep-copy before mutation.
    std::vector<TopoDS_Shape> sourceShapes;
};

class Core3D_CompositeRecipe final : public TDF_Attribute {
public:
    DEFINE_STANDARD_RTTI_INLINE(Core3D_CompositeRecipe, TDF_Attribute)
    const Standard_GUID& ID() const override { return AttributeID(); }
    Handle(TDF_Attribute) NewEmpty() const override { return new Core3D_CompositeRecipe(); }
    const std::shared_ptr<const Payload>& value() const noexcept { return value_; }
    void Restore(const Handle(TDF_Attribute)& source) override {
        const auto original = Handle(Core3D_CompositeRecipe)::DownCast(source);
        if (original.IsNull()) Standard_Failure::Raise("Composite recipe restore type");
        value_ = original->value_;
    }
    void Paste(const Handle(TDF_Attribute)& target, const Handle(TDF_RelocationTable)&) const override {
        const auto destination = Handle(Core3D_CompositeRecipe)::DownCast(target);
        if (destination.IsNull() || !value_) Standard_Failure::Raise("Composite recipe paste type");
        destination->Backup(); destination->value_ = value_;
    }
private:
    friend class BinaryDriver;
    friend class ::OcctDocument;
#if DEBUG
    friend struct Probe;
#endif
    std::shared_ptr<const Payload> value_;
};
using Attribute = Core3D_CompositeRecipe;

struct Record {
    TDF_Label label, owner;
    std::shared_ptr<const Payload> value;
    TopoDS_Shape current;
};

inline TopAbs_ShapeEnum TopologyKind(SourceShapeKind kind) noexcept {
    return kind == SourceShapeKind::Wire ? TopAbs_WIRE : TopAbs_SOLID;
}

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

inline bool ReadAll(const Handle(TDocStd_Document)& document, std::vector<Record>& output,
                    std::size_t legacyBytes = 0) noexcept {
    output.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()
            || legacyBytes > MaximumDocumentAggregateBytes) return false;
        UUID documentID{}; bool readDocumentID=false;
        std::vector<Record> staged; TDF_LabelMap owners;
        std::size_t aggregate = legacyBytes; int visited = 0;
        const TDF_Label root = document->GetData()->Root();
        if (root.IsAttribute(AttributeID())) return false;
        for (TDF_ChildIterator iterator(root, Standard_True); iterator.More(); iterator.Next()) {
            if (++visited > profile::MaximumLabels) return false;
            const TDF_Label label = iterator.Value(); if (!label.IsAttribute(AttributeID())) continue;
            Handle(Attribute) attribute; Handle(TNaming_NamedShape) binding;
            const TDF_Label owner = label.Father();
            if (label.Tag() < MinimumRecordTag || !label.FindAttribute(AttributeID(), attribute)
                || attribute.IsNull() || !attribute->value()
                || !label.FindAttribute(TNaming_NamedShape::GetID(), binding)
                || binding.IsNull() || !XCAFDoc_ShapeTool::IsSimpleShape(owner)
                || !XCAFDoc_ShapeTool::IsFree(owner) || !owners.Add(owner)) return false;
            const auto value = attribute->value(); std::vector<std::uint8_t> exact;
            if (!Encode(value->definition, exact) || exact != value->bytes
                || exact.size() > MaximumDocumentAggregateBytes - aggregate) return false;
            aggregate += exact.size();
            if(!readDocumentID){
                if(!retained_solid::ReadUUID(document->Main(),
                    Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"),documentID))return false;
                readDocumentID=true;
            }
            UUID entity{}, definition{};
            if (!retained_solid::ReadUUID(owner, Standard_GUID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62"), entity)
                || !retained_solid::ReadUUID(owner, Standard_GUID("3611F2B2-C694-4E12-AED8-A2A97A3D283B"), definition)
                || value->definition.owner.document != documentID
                || value->definition.owner.entity != entity
                || value->definition.owner.definition != definition) return false;
            const TopoDS_Shape current = binding->Get();
            if (current.IsNull() || !current.IsEqual(XCAFDoc_ShapeTool::GetShape(owner))
                || value->sourceShapes.empty() || value->sourceShapes.size() > MaximumSourceNodes) return false;
            std::size_t sources = 0;
            for (const Node& node : value->definition.nodes) if (const auto* source = std::get_if<SourceNode>(&node.value)) {
                ++sources;
                if (source->shapeSlot >= value->sourceShapes.size()) return false;
                const TopoDS_Shape& shape = value->sourceShapes[source->shapeSlot];
                if (shape.IsNull()
                    || shape.ShapeType() != TopologyKind(ExpectedSourceShapeKind(source->recipe))
                    || shape.Orientation() != TopAbs_FORWARD) return false;
            }
            if (sources != value->sourceShapes.size()) return false;
            for (TDF_AttributeIterator attributes(label); attributes.More(); attributes.Next())
                if (attributes.Value()->ID() != AttributeID()
                    && attributes.Value()->ID() != TNaming_NamedShape::GetID()) return false;
            int descendants = 0;
            for (TDF_ChildIterator child(label, Standard_True); child.More(); child.Next())
                if (++descendants > profile::MaximumLabels || child.Value().HasAttribute()) return false;
            // A composite is the sole recipe owner for the visible carrier.
            for (TDF_ChildIterator child(owner, Standard_False); child.More(); child.Next()) {
                if (child.Value().IsEqual(label)) continue;
                if (profile::HasAttribute(child.Value()) || enclosure::HasAttribute(child.Value())
                    || sweep_persistence::HasAttribute(child.Value()) || loft_persistence::HasAttribute(child.Value())
                    || retained_solid::HasRecord(child.Value())) return false;
            }
            staged.push_back({label, owner, value, current});
        }
        output = std::move(staged); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Read(const Handle(TDocStd_Document)& document, const TDF_Label& owner,
                 Record& output, std::size_t legacyBytes = 0) noexcept {
    output = {};
    if (document.IsNull() || owner.IsNull() || owner.Data() != document->GetData()) return false;
    if (!HasRecord(owner)) return true;
    std::vector<Record> all;
    if (!ReadAll(document, all, legacyBytes)) return false;
    for (const Record& record : all) if (record.owner.IsEqual(owner)) { output = record; return true; }
    return false;
}

// Both legacy and composite drivers receive the SAME instance in document
// retrieval, so the 8 MiB ceiling is charged before either codec allocates,
// independent of attribute order.
using ReadBudget = retained_solid::ReadBudget;
} // namespace core3d::composite_recipe
