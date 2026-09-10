#pragma once

#include "ProfileDefinition.hxx"
#include <TDocStd_Document.hxx>
#include <TCollection_AsciiString.hxx>
#include <TopoDS_Shape.hxx>
#include <string>
#include <TDF_AttributeIterator.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_Real.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TNaming_Builder.hxx>
#include <TNaming_NamedShape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <Standard_GUID.hxx>
#include <limits>
#include <set>

namespace core3d::profile {

// Existing bounded scalar/string drivers only. Numeric arrays and function
// drivers are intentionally not part of the project-file reader contract.
inline const Standard_GUID& SchemaID() {
    static const Standard_GUID id("6085D37B-E101-4F09-95B3-8AFA6E1D08A1"); return id;
}
inline const Standard_GUID& IdentityID() {
    static const Standard_GUID id("6085D37B-E101-4F09-95B3-8AFA6E1D08A2"); return id;
}
inline const Standard_GUID& CountID() {
    static const Standard_GUID id("6085D37B-E101-4F09-95B3-8AFA6E1D08A3"); return id;
}
inline constexpr int MaximumScalars = 7 + 64 * 2 + 16 * 3;
inline constexpr int MaximumLabels = 100000;
inline constexpr int MaximumRecords = 4096;

struct Parameters {
    ProfileDefinition definition;
    double metersPerUnit = 0;
};

inline bool Encode(const Parameters& parameters, std::vector<double>& values) {
    values.clear();
    const auto& d = parameters.definition;
    double area = 0, volume = 0;
    if (!std::isfinite(parameters.metersPerUnit) || parameters.metersPerUnit <= 0
        || !ProfileDefinitionExpectedVolume(d.points, d.circle, d.holes,
            d.plane, d.depth, d.revolve, area, volume)) return false;
    values = {double(d.plane), d.depth, d.revolve ? 1.0 : 0.0,
        d.circle ? 1.0 : 0.0, double(d.points.size()), double(d.holes.size()),
        parameters.metersPerUnit};
    if (d.circle) {
        const auto& c = *d.circle;
        values.insert(values.end(), {c.center.X(), c.center.Y(), c.outerRadius, c.innerRadius});
    } else {
        for (const auto& point : d.points) {
            values.push_back(point.X()); values.push_back(point.Y());
        }
    }
    for (const auto& hole : d.holes)
        values.insert(values.end(), {hole.center.X(), hole.center.Y(), hole.radius});
    return values.size() <= MaximumScalars;
}

inline bool Decode(const std::vector<double>& values, Parameters& result) {
    result = {};
    if (values.size() < 7 || values.size() > MaximumScalars) return false;
    for (double value : values) if (!std::isfinite(value)) return false;
    const auto integer = [](double value, int maximum) {
        return value >= 0 && value <= maximum && value == std::floor(value);
    };
    if (!integer(values[0], 2) || !integer(values[2], 1) || !integer(values[3], 1)
        || !integer(values[4], 64) || !integer(values[5], 16)) return false;
    const int points = int(values[4]), holes = int(values[5]);
    const bool circular = values[3] == 1;
    if (values.size() != 7 + (circular ? 4 : points * 2) + holes * 3
        || (circular && points != 0)) return false;
    Parameters p;
    p.metersPerUnit = values[6];
    auto& d = p.definition;
    d.plane = int(values[0]); d.depth = values[1]; d.revolve = values[2] == 1;
    std::size_t offset = 7;
    if (circular) {
        d.circle = ProfileCircularSection{gp_Pnt2d(values[7], values[8]), values[9], values[10]};
        offset += 4;
    } else {
        for (int i = 0; i < points; ++i, offset += 2)
            d.points.emplace_back(values[offset], values[offset + 1]);
    }
    for (int i = 0; i < holes; ++i, offset += 3)
        d.holes.push_back({gp_Pnt2d(values[offset], values[offset + 1]), values[offset + 2]});
    std::vector<double> encoded;
    if (!Encode(p, encoded) || encoded != values) return false;
    result = std::move(p); return true;
}

inline bool IsIdentifier(const std::string& value) {
    if (value.size() != 36 || !Standard_GUID::CheckGUIDFormat(value.c_str())) return false;
    for (char c : value)
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || c == '-')) return false;
    return true;
}
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
        if (!XCAFDoc_DocumentTool::CheckShapeTool(document->Main())
            || !XCAFDoc_ShapeTool::IsSimpleShape(owner) || !XCAFDoc_ShapeTool::IsFree(owner)) return false;
        const auto& label = record.label;
        Handle(TDataStd_Integer) schema, count;
        Handle(TDataStd_AsciiString) identity;
        Handle(TNaming_NamedShape) binding;
        if (!label.FindAttribute(SchemaID(), schema) || schema->Get() != 1
            || !label.FindAttribute(CountID(), count) || count->Get() < 7 || count->Get() > MaximumScalars
            || !label.FindAttribute(IdentityID(), identity)
            || !label.FindAttribute(TNaming_NamedShape::GetID(), binding)) return false;
        record.identifier = identity->Get().ToCString();
        if (!IsIdentifier(record.identifier)) return false;
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
            if (++childrenInspected > MaximumScalars || child.Tag() < 1
                || child.Tag() > MaximumScalars || child.HasChild()) return false;
            if (!child.HasAttribute()) continue; // Empty labels from a shorter later recipe.
            if (child.Tag() < 1 || child.Tag() > count->Get()) return false;
            Handle(TDataStd_Real) scalar;
            if (!child.FindAttribute(TDataStd_Real::GetID(), scalar) || !std::isfinite(scalar->Get())) return false;
            for (TDF_AttributeIterator attr(child); attr.More(); attr.Next())
                if (attr.Value()->ID() != TDataStd_Real::GetID()) return false;
            record.values[child.Tag() - 1] = scalar->Get(); ++present;
        }
        if (present != count->Get() || !Decode(record.values, record.parameters)) return false;
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
        if (document.IsNull() || !document->HasOpenCommand() || !IsIdentifier(identifier)
            || !Encode(parameters, values) || !Read(document, owner, previous)
            || !XCAFDoc_ShapeTool::IsSimpleShape(owner) || !XCAFDoc_ShapeTool::IsFree(owner)
            || !XCAFDoc_DocumentTool::GetLengthUnit(document, unit) || unit != parameters.metersPerUnit
            || (!previous.label.IsNull() && previous.identifier != identifier)) return false;
        const auto shape = XCAFDoc_ShapeTool::GetShape(owner);
        if (shape.IsNull() || shape.ShapeType() != TopAbs_SOLID) return false;
        auto label = previous.label;
        if (label.IsNull()) {
            int maximumTag = 0;
            for (TDF_ChildIterator it(owner, Standard_False); it.More(); it.Next())
                maximumTag = std::max(maximumTag, it.Value().Tag());
            if (maximumTag == std::numeric_limits<int>::max()) return false;
            label = owner.FindChild(maximumTag + 1, Standard_True);
        }
        TDataStd_Integer::Set(label, SchemaID(), 1);
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

inline bool HasOnlyMetadataSubshapes(const Handle(TDocStd_Document)& document, const TDF_Label& owner) {
    Record record;
    if (!Read(document, owner, record)) return false;
    TDF_LabelSequence children;
    XCAFDoc_ShapeTool::GetSubShapes(owner, children);
    for (int i = 1; i <= children.Length(); ++i)
        if (record.label.IsNull() || !children.Value(i).IsEqual(record.label)) return false;
    return true;
}
} // namespace core3d::profile
