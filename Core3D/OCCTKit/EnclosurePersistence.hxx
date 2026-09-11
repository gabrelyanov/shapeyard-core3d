#pragma once
// Saved enclosure dependency record. All mutation belongs to the existing
// ordinary OCAF transaction; no document command is opened by this codec.
#include "EnclosureParameters.hxx"
#include "ProfilePersistence.hxx"
namespace core3d::enclosure {
inline const Standard_GUID& SchemaID() {
    static const Standard_GUID id("22E0916D-E068-4556-9200-6F6D54F12D9E");return id;
}
inline const Standard_GUID& IdentityID() {
    static const Standard_GUID id("97B4EB8E-DF73-4FF0-93D6-1B86C9C18432");return id;
}
inline const Standard_GUID& CountID() {
    static const Standard_GUID id("01BB68EC-1C11-4FE3-A3A4-7DB4B2071A95");return id;
}
inline constexpr int MaximumLabels=profile::MaximumLabels;
inline constexpr int MaximumRecords=profile::MaximumRecords;
// OcctDocument reserves children1..8 for transforms and11/12 for appearance.
inline constexpr int MinimumRecordTag=13;
inline bool HasAttribute(const TDF_Label& label) {
    return label.IsAttribute(SchemaID()) || label.IsAttribute(IdentityID()) || label.IsAttribute(CountID());
}
struct Record {
    TDF_Label label;
    std::string identifier;
    Parameters parameters;
    std::vector<double> values;
    TopoDS_Shape boundShape;

    bool IsEqual(const Record& other) const {
        if (label.IsNull() || other.label.IsNull()) return label.IsNull() && other.label.IsNull();
        return label.IsEqual(other.label) && label.Data() == other.label.Data()
            && identifier == other.identifier && values == other.values
            && !boundShape.IsNull() && !other.boundShape.IsNull()
            && boundShape.IsEqual(other.boundShape);
    }
    bool IsCurrent(const Handle(TDocStd_Document)& document, const TDF_Label& owner) const {
        double unit = 0;
        return !label.IsNull() && !owner.IsNull() && !document.IsNull()
            && owner.Data() == document->GetData() && label.Father().IsEqual(owner)
            && !boundShape.IsNull() && boundShape.IsEqual(XCAFDoc_ShapeTool::GetShape(owner))
            && XCAFDoc_DocumentTool::GetLengthUnit(document, unit)
            && unit == parameters.metersPerUnit;
    }
};

// Absence is a valid legacy state. Any partial or malformed present record
// fails; it must never be silently treated as an absent editable feature.
inline bool Read(const Handle(TDocStd_Document)& document, const TDF_Label& owner, Record& output) noexcept {
    output = {};
    try {
        if (document.IsNull() || owner.IsNull() || owner.Data() != document->GetData()) return false;
        Record record;
        int inspected = 0;
        for (TDF_ChildIterator it(owner, Standard_False); it.More(); it.Next()) {
            if (++inspected > MaximumLabels) return false;
            if (!HasAttribute(it.Value())) continue;
            if (!record.label.IsNull()) return false;
            record.label = it.Value();
        }
        if (record.label.IsNull()) return true;
        if (record.label.Tag() < MinimumRecordTag) return false;
        // An owner cannot silently carry two independent rebuild authorities.
        profile::Record other;
        if (!profile::Read(document,owner,other) || !other.label.IsNull()) return false;
        if (!XCAFDoc_DocumentTool::CheckShapeTool(document->Main())
            || !XCAFDoc_ShapeTool::IsSimpleShape(owner) || !XCAFDoc_ShapeTool::IsFree(owner)) return false;
        const auto& label = record.label;
        Handle(TDataStd_Integer) schema, count;
        Handle(TDataStd_AsciiString) identity;
        Handle(TNaming_NamedShape) binding;
        if (!label.FindAttribute(SchemaID(), schema) || ScalarCountForSchema(schema->Get()) == 0
            || !label.FindAttribute(CountID(), count) || count->Get() != int(ScalarCountForSchema(schema->Get()))
            || !label.FindAttribute(IdentityID(), identity)
            || !label.FindAttribute(TNaming_NamedShape::GetID(), binding)) return false;
        const int scalarLimit = int(ScalarCountForSchema(schema->Get()));
        if (count->Get() > scalarLimit) return false;
        record.identifier = identity->Get().ToCString();
        if (!profile::IsIdentifier(record.identifier)) return false;
        record.boundShape = binding->Get();
        if (record.boundShape.IsNull() || record.boundShape.ShapeType() != TopAbs_SOLID) return false;
        for (TDF_AttributeIterator it(label); it.More(); it.Next()) {
            const auto& id = it.Value()->ID();
            if (id != SchemaID() && id != CountID() && id != IdentityID()
                && id != TNaming_NamedShape::GetID()) return false;
        }
        record.values.resize(count->Get());
        int present = 0, childrenInspected = 0;
        for (TDF_ChildIterator it(label, Standard_False); it.More(); it.Next()) {
            const auto child = it.Value();
            if (++childrenInspected > MaximumLabels || child.Tag() < 1) return false;
            // OCAF aborts attributes, but allocated labels survive the transaction.
            // Empty descendants carry no recipe data; any live attribute is invalid.
            for (TDF_ChildIterator nested(child, Standard_True); nested.More(); nested.Next())
                if (++childrenInspected > MaximumLabels || nested.Value().HasAttribute()) return false;
            if (!child.HasAttribute()) continue; // Empty labels from a shorter later recipe.
            if (child.Tag() > scalarLimit || child.Tag() > count->Get()) return false;
            Handle(TDataStd_Real) scalar;
            if (!child.FindAttribute(TDataStd_Real::GetID(), scalar) || !std::isfinite(scalar->Get())) return false;
            for (TDF_AttributeIterator attr(child); attr.More(); attr.Next())
                if (attr.Value()->ID() != TDataStd_Real::GetID()) return false;
            record.values[child.Tag() - 1] = scalar->Get(); ++present;
        }
        if (present != count->Get() || !Decode(schema->Get(), record.values, record.parameters)) return false;
        output = std::move(record); return true;
    } catch (...) { output = {}; return false; }
}

// The caller owns the ordinary OCAF transaction and must retain/abort it on
// failure. Parameters and geometry are staged before candidate publication.
inline bool Stage(const Handle(TDocStd_Document)& document, const TDF_Label& owner,
                  const Parameters& parameters, const std::string& identifier) noexcept {
    try {
        Record previous;
        std::vector<double> values;
        double unit = 0;
        if (document.IsNull() || !document->HasOpenCommand() || !profile::IsIdentifier(identifier)
            || !Encode(parameters, values) || !Read(document, owner, previous)
            || !XCAFDoc_ShapeTool::IsSimpleShape(owner) || !XCAFDoc_ShapeTool::IsFree(owner)
            || !XCAFDoc_DocumentTool::GetLengthUnit(document, unit) || unit != parameters.metersPerUnit
            || (!previous.label.IsNull() && previous.identifier != identifier)) return false;
        profile::Record other;
        if (!profile::Read(document,owner,other) || !other.label.IsNull()) return false;
        const auto shape = XCAFDoc_ShapeTool::GetShape(owner);
        if (shape.IsNull() || shape.ShapeType() != TopAbs_SOLID) return false;
        auto label = previous.label;
        if (label.IsNull()) {
            int maximumTag = MinimumRecordTag - 1;
            for (TDF_ChildIterator it(owner, Standard_False); it.More(); it.Next())
                maximumTag = std::max(maximumTag, it.Value().Tag());
            if (maximumTag == std::numeric_limits<int>::max()) return false;
            label = owner.FindChild(maximumTag + 1, Standard_True);
        }
        TDataStd_Integer::Set(label, SchemaID(), int(values.front()));
        TDataStd_Integer::Set(label, CountID(), int(values.size()));
        TDataStd_AsciiString::Set(label, IdentityID(), TCollection_AsciiString(identifier.c_str()));
        TNaming_Builder(label).Select(shape, shape);
        for (TDF_ChildIterator it(label, Standard_False); it.More(); it.Next())
            it.Value().ForgetAllAttributes(Standard_True);
        for (std::size_t i = 0; i < values.size(); ++i)
            TDataStd_Real::Set(label.FindChild(int(i) + 1, Standard_True), values[i]);
        Record stored;
        return Read(document, owner, stored) && stored.identifier == identifier
            && stored.values == values && stored.IsCurrent(document, owner);
    } catch (...) { return false; }
}

inline bool StageIndependentCopy(const Handle(TDocStd_Document)& document,
                           const TDF_Label& sourceOwner, const Record& original,
                           const TopoDS_Shape& originalOwnerShape,
                           const TDF_Label& destinationOwner,
                           const TopoDS_Shape& preparedBinding,
                           const std::string& newIdentifier,
                           Record& candidate,
                           const std::optional<gp_Trsf>& bakedTransform) noexcept {
    candidate = {};
    try {
        Record liveSource, previousDestination;
        if (document.IsNull() || !document->HasOpenCommand()
            || sourceOwner.IsNull() || destinationOwner.IsNull()
            || sourceOwner.Data() != document->GetData()
            || destinationOwner.Data() != document->GetData()
            || sourceOwner.IsEqual(destinationOwner)
            || originalOwnerShape.IsNull()
            || !originalOwnerShape.IsEqual(XCAFDoc_ShapeTool::GetShape(sourceOwner))
            || !Read(document, sourceOwner, liveSource) || !liveSource.IsEqual(original)
            || !Read(document, destinationOwner, previousDestination)
            || !previousDestination.label.IsNull()) return false;
        // Legacy shapes without recipes continue to duplicate without one.
        if (original.label.IsNull())
            return preparedBinding.IsNull() && newIdentifier.empty();
        const auto destinationShape = XCAFDoc_ShapeTool::GetShape(destinationOwner);
        Parameters destinationParameters = original.parameters;
        if (bakedTransform) {
            gp_Trsf originalFrame;
            if (original.parameters.definition.constructionFrame
                && !original.parameters.definition.constructionFrame->Transform(originalFrame)) return false;
            profile::ConstructionFrame composed;
            if (!profile::ConstructionFrame::Capture(*bakedTransform * originalFrame, composed)) return false;
            destinationParameters.definition.constructionFrame = composed;
        }
        std::vector<double> encoded, originalEncoded;
        if (!XCAFDoc_ShapeTool::IsSimpleShape(destinationOwner)
            || !XCAFDoc_ShapeTool::IsFree(destinationOwner)
            || !profile::IsIdentifier(newIdentifier) || newIdentifier == original.identifier
            || !Encode(original.parameters, originalEncoded) || originalEncoded != original.values
            || !Encode(destinationParameters, encoded)
            || destinationShape.IsNull() || destinationShape.ShapeType() != originalOwnerShape.ShapeType()
            || destinationShape.IsPartner(originalOwnerShape)
            || preparedBinding.IsNull() || preparedBinding.ShapeType() != TopAbs_SOLID
            || preparedBinding.IsPartner(original.boundShape)) return false;
        const bool boundToSourceRoot = original.boundShape.IsEqual(originalOwnerShape);
        // The independently copied current root must be used only when the
        // original recipe was actually bound to its root, even if units stale.
        if (preparedBinding.IsEqual(destinationShape) != boundToSourceRoot) return false;
        const bool expectedCurrent = original.IsCurrent(document, sourceOwner);
        int maximumTag = MinimumRecordTag - 1;
        for (TDF_ChildIterator it(destinationOwner, Standard_False); it.More(); it.Next())
            maximumTag = std::max(maximumTag, it.Value().Tag());
        if (maximumTag == std::numeric_limits<int>::max()) return false;
        const auto label = destinationOwner.FindChild(maximumTag + 1, Standard_True);
        TDataStd_Integer::Set(label, SchemaID(), int(encoded.front()));
        TDataStd_Integer::Set(label, CountID(), int(encoded.size()));
        TDataStd_AsciiString::Set(label, IdentityID(), TCollection_AsciiString(newIdentifier.c_str()));
        TNaming_Builder(label).Select(preparedBinding, preparedBinding);
        for (std::size_t i = 0; i < encoded.size(); ++i)
            TDataStd_Real::Set(label.FindChild(int(i) + 1, Standard_True), encoded[i]);
        Record stored;
        if (!Read(document, destinationOwner, stored)
            || stored.identifier != newIdentifier || stored.values != encoded
            || !stored.boundShape.IsEqual(preparedBinding)
            || stored.IsCurrent(document, destinationOwner) != expectedCurrent)
            return false;
        // Prove that adding a new feature did not touch the original labels.
        if (!Read(document, sourceOwner, liveSource) || !liveSource.IsEqual(original)
            || !originalOwnerShape.IsEqual(XCAFDoc_ShapeTool::GetShape(sourceOwner))) return false;
        candidate = std::move(stored);
        return true;
    } catch (...) { candidate = {}; return false; }
}

inline bool StageDuplicate(const Handle(TDocStd_Document)& document,
                           const TDF_Label& sourceOwner, const Record& original,
                           const TopoDS_Shape& originalOwnerShape,
                           const TDF_Label& destinationOwner,
                           const TopoDS_Shape& preparedBinding,
                           const std::string& newIdentifier,
                           Record& candidate) noexcept {
    return StageIndependentCopy(document, sourceOwner, original, originalOwnerShape,
                                destinationOwner, preparedBinding, newIdentifier,
                                candidate, std::nullopt);
}

inline bool StageTransformedCopy(const Handle(TDocStd_Document)& document,
                                const TDF_Label& sourceOwner, const Record& original,
                                const TopoDS_Shape& originalOwnerShape,
                                const TDF_Label& destinationOwner,
                                const TopoDS_Shape& preparedBinding,
                                const std::string& newIdentifier,
                                const gp_Trsf& bakedTransform,
                                Record& candidate) noexcept {
    return StageIndependentCopy(document, sourceOwner, original, originalOwnerShape,
                                destinationOwner, preparedBinding, newIdentifier,
                                candidate, bakedTransform);
}

inline bool ValidateDocument(const Handle(TDocStd_Document)& document, std::vector<Record>& records) noexcept {
    records.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const auto root = document->GetData()->Root();
        if (HasAttribute(root)) return false;
        std::set<std::string> identities;
        int count = 0;
        for (TDF_ChildIterator it(root, Standard_True); it.More(); it.Next()) {
            if (++count > MaximumLabels) return false;
            const auto label = it.Value();
            if (!HasAttribute(label)) continue;
            Record record;
            if (records.size() >= MaximumRecords || !Read(document, label.Father(), record)
                || record.label.IsNull() || !record.label.IsEqual(label)
                || !identities.insert(record.identifier).second) return false;
            records.push_back(std::move(record));
        }
        return true;
    } catch (...) { records.clear(); return false; }
}

// Rebuild admission recognizes only this feature's own metadata child;
// arbitrary face/subshape styling needs an explicit preservation policy.
inline bool HasOnlyMetadataSubshapes(const Handle(TDocStd_Document)& document, const TDF_Label& owner) {
    Record record;
    if (!Read(document,owner,record)) return false;
    TDF_LabelSequence children;
    XCAFDoc_ShapeTool::GetSubShapes(owner,children);
    for (int i=1;i<=children.Length();++i)
        if (record.label.IsNull() || !children.Value(i).IsEqual(record.label)) return false;
    return true;
}

// Combined gate must replace the profile-only call at the document boundary.
// MaximumRecords is shared, not doubled. Feature identifiers are globally unique.
inline bool ValidateFeatureRecords(const Handle(TDocStd_Document)& document,
    std::vector<profile::Record>& profiles, std::vector<Record>& enclosures) noexcept {
    profiles.clear();enclosures.clear();
    try {
        if (!profile::ValidateDocument(document,profiles) || !ValidateDocument(document,enclosures)
            || profiles.size()+enclosures.size()>std::size_t(MaximumRecords)) {
            profiles.clear();enclosures.clear();return false;
        }
        std::set<std::string> identifiers;
        for (const auto& record:profiles) identifiers.insert(record.identifier);
        for (const auto& record:enclosures) {
            if (!identifiers.insert(record.identifier).second) {
                profiles.clear();enclosures.clear();return false;
            }
        }
        return true;
    } catch (...) {profiles.clear();enclosures.clear();return false;}
}
} // namespace core3d::enclosure
