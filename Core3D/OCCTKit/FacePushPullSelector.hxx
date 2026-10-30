#pragma once

#include <Core3D/FacePushPullDefinition.hxx>
#include <Core3D/RetainedTopologyBudget.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Pln.hxx>

namespace core3d::face_push_pull {
namespace detail {
using Budget = retained_topology_budget::Counter;
using Census = retained_topology_budget::Census;
using Site = retained_topology_budget::Site;
inline bool census(const TopoDS_Shape& shape, Budget& budget, const std::atomic_bool& cancelled,
                    Site site, Census& result, Refusal& refusal, bool debitStage = true) {
    const auto status = retained_topology_budget::CensusTopology(shape, budget, cancelled, result, site, debitStage);
    if (status == retained_topology_budget::WalkStatus::Completed) return true;
    refusal = cancelled.load() ? Refusal::Cancelled : budget.exhausted
        ? Refusal::TopologyBudgetExceeded : Refusal::BuildFailure;
    return false;
}
inline bool reserve(const Census& count, Budget& budget, Site site, Refusal& refusal) {
    if (retained_topology_budget::ReserveTraversal(count, budget, site)) return true;
    refusal = Refusal::TopologyBudgetExceeded; return false;
}
inline gp_Pnt point(const std::array<double, 3>& p) { return {p[0], p[1], p[2]}; }
inline gp_Dir direction(const std::array<double, 3>& p) { return {p[0], p[1], p[2]}; }
inline gp_Dir outward(const TopoDS_Face& face) {
    const gp_Pln plane = BRepAdaptor_Surface(face, Standard_True).Plane();
    gp_Dir result = plane.Axis().Direction();
    // A reflected analytic plane can carry an indirect Ax3. Its geometric
    // surface normal is X cross Y, opposite the stored axis in that case.
    if (!plane.Position().Direct()) result.Reverse();
    if (face.Orientation() == TopAbs_REVERSED) result.Reverse();
    return result;
}
inline double tolerance(const Definition& value) {
    return std::max(Precision::Confusion() * 4, 1e-9 / value.metersPerLocalUnit);
}
inline bool describe(const TopoDS_Face& face, const UUID& id, FaceSelector& output,
                      Budget& budget, const std::atomic_bool& cancelled, Refusal& refusal) {
    output = {}; Census count;
    if (!census(face, budget, cancelled, Site::C14AnchorResolve, count, refusal, false)) return false;
    if (face.Orientation() != TopAbs_FORWARD && face.Orientation() != TopAbs_REVERSED) {
        refusal = Refusal::InvalidSelector; return false;
    }
    if (BRepAdaptor_Surface(face, Standard_True).GetType() != GeomAbs_Plane) {
        refusal = Refusal::UnsupportedSurface; return false;
    }
    if (!reserve(count, budget, Site::C14AnchorResolve, refusal)) return false;
    std::size_t edges = 0, wires = 0;
    for (TopExp_Explorer it(face, TopAbs_EDGE); it.More(); it.Next()) ++edges;
    for (TopExp_Explorer it(face, TopAbs_WIRE); it.More(); it.Next()) ++wires;
    if (edges == 0 || edges > MaximumBoundaryEdges || wires == 0 || wires > MaximumBoundaryWires) {
        refusal = Refusal::InvalidSelector; return false;
    }
    if (!reserve(count, budget, Site::C14AnchorResolve, refusal)
        || !reserve(count, budget, Site::C14AnchorResolve, refusal)) return false;
    GProp_GProps area, perimeter;
    BRepGProp::SurfaceProperties(face, area); BRepGProp::LinearProperties(face, perimeter);
    const gp_Pnt center = area.CentreOfMass(); const gp_Dir normal = outward(face);
    FaceSelector value; value.identifier = id;
    value.boundaryEdges = std::uint16_t(edges); value.boundaryWires = std::uint16_t(wires);
    value.centroidLocal = {{center.X(), center.Y(), center.Z()}};
    value.outwardNormalLocal = {{normal.X(), normal.Y(), normal.Z()}};
    value.areaLocal = area.Mass(); value.perimeterLocal = perimeter.Mass();
    if (!ValidSelector(value)) { refusal = Refusal::InvalidSelector; return false; }
    output = value; refusal = Refusal::None; return true;
}
inline bool matches(const FaceSelector& a, const FaceSelector& b, double tol) {
    return a.boundaryEdges == b.boundaryEdges && a.boundaryWires == b.boundaryWires
        && point(a.centroidLocal).Distance(point(b.centroidLocal)) <= tol
        && direction(a.outwardNormalLocal).Dot(direction(b.outwardNormalLocal)) >= 1 - 1e-12
        && std::abs(a.areaLocal - b.areaLocal) <= tol * std::max(a.perimeterLocal, b.perimeterLocal)
        && std::abs(a.perimeterLocal - b.perimeterLocal) <= tol * a.boundaryEdges;
}
} // namespace detail

inline bool Resolve(const TopoDS_Shape& source, const FaceSelector& selector, double tolerance,
                     TopoDS_Face& output, Refusal& refusal, detail::Budget& budget,
                     const std::atomic_bool& cancelled) noexcept {
    output.Nullify(); refusal = Refusal::InvalidSelector;
    try {
        if (!ValidSelector(selector) || !std::isfinite(tolerance) || tolerance <= 0) return false;
        detail::Census count;
        if (!detail::census(source, budget, cancelled, detail::Site::C14AnchorResolve, count, refusal, false)) return false;
        TopoDS_Face match; std::size_t matches = 0;
        for (int i = 1; i <= count.faces.Extent(); ++i) {
            if (cancelled.load()) { refusal = Refusal::Cancelled; return false; }
            const auto face = TopoDS::Face(count.faces(i));
            if (BRepAdaptor_Surface(face, Standard_True).GetType() != GeomAbs_Plane) continue;
            FaceSelector candidate;
            if (!detail::describe(face, selector.identifier, candidate, budget, cancelled, refusal)) return false;
            if (detail::matches(candidate, selector, tolerance)) { match = face; ++matches; }
        }
        if (matches != 1) {
            refusal = matches == 0 ? Refusal::AnchorMissing : Refusal::AnchorAmbiguous; return false;
        }
        output = match; refusal = Refusal::None; return true;
    } catch (...) { refusal = Refusal::BuildFailure; return false; }
}

inline bool Capture(const TopoDS_Shape& source, const TopoDS_Face& selected, const UUID& identifier,
                     FaceSelector& output, Refusal& refusal, detail::Budget& budget,
                     const std::atomic_bool& cancelled) noexcept {
    output = {}; refusal = Refusal::InvalidSource;
    try {
        if (source.IsNull() || source.ShapeType() != TopAbs_SOLID || selected.IsNull()) return false;
        detail::Census count;
        if (!detail::census(source, budget, cancelled, detail::Site::C14AnchorResolve, count, refusal, false)) return false;
        const int index = count.faces.FindIndex(selected);
        if (!index || count.faces(index).Orientation() != selected.Orientation()) {
            refusal = Refusal::AnchorMissing; return false;
        }
        FaceSelector value;
        if (!detail::describe(TopoDS::Face(count.faces(index)), identifier, value, budget, cancelled, refusal)) return false;
        // The issued intent must be unique even on its original source.
        TopoDS_Face resolved;
        if (!Resolve(source, value, Precision::Confusion() * 4, resolved, refusal, budget, cancelled)) return false;
        output = value; return true;
    } catch (...) { refusal = Refusal::BuildFailure; return false; }
}
} // namespace core3d::face_push_pull
