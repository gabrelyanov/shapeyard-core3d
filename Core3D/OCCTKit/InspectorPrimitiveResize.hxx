#pragma once

#include <TDocStd_Document.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_Real.hxx>
#include <array>
#include <cmath>
#include <TNaming_Builder.hxx>
#include <TNaming_NamedShape.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <Standard_GUID.hxx>
#include <algorithm>
#include <limits>

namespace core3d::inspector_primitive {

// A cylinder can be temporarily oval while Width and Depth are edited one at
// a time. Bind its origin to the exact current BRep, not its name or bounds.
// Existing integer/naming drivers preserve the binding across save and undo.
inline const Standard_GUID& KindID() {
    static const Standard_GUID id("C816A70B-A215-4C4D-9928-C8D8CD98B721");
    return id;
}
struct Record {
    TDF_Label label;
    TopoDS_Shape shape;
    std::array<double, 6> bounds{};
    int kind = 0; // 0 absent/inactive, 2 cylinder stretched in X/Y
    bool IsEqual(const Record& other) const {
        if (kind != other.kind || bounds != other.bounds) return false;
        if (label.IsNull() || other.label.IsNull()) return label.IsNull() && other.label.IsNull();
        return label.IsEqual(other.label) && (kind == 0
            ? shape.IsNull() && other.shape.IsNull()
            : !shape.IsNull() && shape.IsEqual(other.shape));
    }
    bool IsCurrent(const TopoDS_Shape& current) const {
        return kind == 2 && !shape.IsNull() && shape.IsEqual(current);
    }
};
inline bool Read(const TDF_Label& owner, Record& output) {
    output = {};
    int count = 0;
    for (TDF_ChildIterator it(owner, Standard_False); it.More(); it.Next()) {
        if (++count > 100000) return false;
        Handle(TDF_Attribute) raw;
        if (!it.Value().FindAttribute(KindID(), raw)) continue;
        const auto kind = Handle(TDataStd_Integer)::DownCast(raw);
        Handle(TNaming_NamedShape) named;
        if (!output.label.IsNull() || kind.IsNull() || (kind->Get() != 0 && kind->Get() != 2))
            return false;
        const bool hasShape = it.Value().FindAttribute(TNaming_NamedShape::GetID(), named);
        if (kind->Get() == 0) {
            if (hasShape) return false;
            output.label = it.Value();
        } else {
            if (!hasShape || named.IsNull() || named->Get().IsNull()) return false;
            output.label = it.Value(); output.shape = named->Get(); output.kind = 2;
            for (int i = 0; i < 6; ++i) {
                const auto child = it.Value().FindChild(i + 1, Standard_False);
                Handle(TDataStd_Real) value;
                if (child.IsNull() || !child.FindAttribute(TDataStd_Real::GetID(), value)
                    || value.IsNull() || !std::isfinite(value->Get())) return false;
                output.bounds[i] = value->Get();
            }
            for (int i = 0; i < 3; ++i)
                if (output.bounds[i] >= output.bounds[i + 3]) return false;
        }
    }
    return true;
}
inline bool Stage(const Handle(TDocStd_Document)& document,
                  const TDF_Label& owner, int kind, const std::array<double, 6>& bounds) {
    if (document.IsNull() || !document->HasOpenCommand()
        || owner.IsNull() || owner.Data() != document->GetData()
        || (kind != 0 && kind != 2)) return false;
    Record previous;
    if (!Read(owner, previous)) return false;
    if (kind == 0) {
        if (!previous.label.IsNull()) {
            // Keep one reusable slot. Repeated oval/round edits must not leave
            // an ever-growing series of empty OCAF children behind.
            previous.label.ForgetAttribute(TNaming_NamedShape::GetID());
            TDataStd_Integer::Set(previous.label, KindID(), 0);
            for (TDF_ChildIterator it(previous.label, Standard_False); it.More(); it.Next())
                it.Value().ForgetAllAttributes(Standard_True);
        }
        Record stored;
        return Read(owner, stored) && stored.kind == 0;
    }
    const auto shape = XCAFDoc_ShapeTool::GetShape(owner);
    if (shape.IsNull() || shape.ShapeType() != TopAbs_SOLID) return false;
    auto label = previous.label;
    if (label.IsNull()) {
        int tag = 12;
        for (TDF_ChildIterator it(owner, Standard_False); it.More(); it.Next())
            tag = std::max(tag, it.Value().Tag());
        if (tag == std::numeric_limits<int>::max()) return false;
        label = owner.FindChild(tag + 1, Standard_True);
    }
    for (int i = 0; i < 6; ++i)
        TDataStd_Real::Set(label.FindChild(i + 1, Standard_True), bounds[i]);
    TDataStd_Integer::Set(label, KindID(), kind);
    TNaming_Builder(label).Select(shape, shape);
    Record stored;
    return Read(owner, stored) && stored.kind == kind && stored.bounds == bounds && stored.IsCurrent(shape);
}
}
