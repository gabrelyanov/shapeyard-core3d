#pragma once

// Ordinary Boolean output domain. This is deliberately separate from the
// one-solid analytic, shell and retained-program families. A set is either a
// solid or a flat compound containing only distinct closed forward solids.
// The caller still owns the existing document/topology budget and Stop state.
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_MapOfShape.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shape.hxx>
#include <cmath>
#include <cstddef>
#include <vector>

namespace core3d::part_boolean::result_set {

// Matches the established legacy rail ceiling; no global budget is raised.
inline constexpr std::size_t MaximumBodies = 1024;

inline bool Bodies(const TopoDS_Shape& shape,
                   std::vector<TopoDS_Shape>& output) noexcept {
    output.clear();
    try {
        if (shape.IsNull() || shape.Orientation() != TopAbs_FORWARD) return false;
        std::vector<TopoDS_Shape> bodies;
        TopTools_MapOfShape distinct;
        const auto append = [&](const TopoDS_Shape& body) {
            if (body.IsNull() || body.ShapeType() != TopAbs_SOLID
                || body.Orientation() != TopAbs_FORWARD
                || bodies.size() >= MaximumBodies || !distinct.Add(body)) return false;
            // OCCT predicates may change TShape bookkeeping. Validate a copy,
            // preserving the canonical bytes of stored and shared shapes.
            BRepBuilderAPI_Copy copy(body, Standard_True, Standard_False);
            if (!copy.IsDone() || copy.Shape().IsNull()) return false;
            const TopoDS_Shape checked = copy.Shape();
            if (!BRepCheck_Analyzer(checked, Standard_True).IsValid()) return false;
            std::size_t shells = 0;
            for (TopExp_Explorer shell(checked, TopAbs_SHELL); shell.More(); shell.Next()) {
                ++shells;
                if (!BRep_Tool::IsClosed(shell.Current())) return false;
            }
            if (!shells) return false;
            BRepClass3d_SolidClassifier classifier(checked);
            classifier.PerformInfinitePoint(Precision::Confusion());
            GProp_GProps properties;
            BRepGProp::VolumeProperties(checked, properties);
            if (classifier.State() != TopAbs_OUT || !std::isfinite(properties.Mass())
                || properties.Mass() <= 0) return false;
            bodies.push_back(body);
            return true;
        };
        if (shape.ShapeType() == TopAbs_SOLID) {
            if (!append(shape)) return false;
        } else if (shape.ShapeType() == TopAbs_COMPOUND) {
            for (TopoDS_Iterator child(shape, Standard_True, Standard_True);
                 child.More(); child.Next())
                if (!append(child.Value())) return false;
        } else return false;
        if (bodies.empty()) return false;
        output = std::move(bodies);
        return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Valid(const TopoDS_Shape& shape) noexcept {
    std::vector<TopoDS_Shape> bodies;
    return Bodies(shape, bodies);
}

inline TopoDS_Shape Normalize(const TopoDS_Shape& shape) noexcept {
    std::vector<TopoDS_Shape> bodies;
    if (!Bodies(shape, bodies)) return {};
    return bodies.size() == 1 ? bodies.front() : shape;
}

} // namespace core3d::part_boolean::result_set
