#pragma once

// Positive-uniform-scale transport for the analytic Boolean working frame.
// OCCT 7.8 omits planar pcurves in BRepTools_TrsfModification; this wrapper
// restores only entries already present in the source edge's raw list.
#include <BRepBuilderAPI_Transform.hxx>
#include <BRep_CurveOnClosedSurface.hxx>
#include <BRep_CurveOnSurface.hxx>
#include <BRep_ListIteratorOfListOfCurveRepresentation.hxx>
#include <BRep_TEdge.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Line.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomLib.hxx>
#include <Precision.hxx>
#include <gp_GTrsf2d.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <map>

namespace core3d::analytic_boolean_scale {
namespace detail {

struct TargetRepresentation final {
    Handle(Geom_Surface) surface;
    TopLoc_Location location;
};

inline bool TargetIsSame(const TargetRepresentation& lhs,
                         const Handle(Geom_Surface)& surface,
                         const TopLoc_Location& location) {
    return lhs.surface == surface && lhs.location.IsEqual(location);
}

inline void TransformedParameters(const TopoDS_Edge& source,
                                  const gp_Trsf& transform,
                                  Standard_Real& first,
                                  Standard_Real& last) {
    TopoDS_Vertex vertex1, vertex2;
    TopExp::Vertices(source, vertex1, vertex2);
    const TopoDS_Edge forward = TopoDS::Edge(source.Oriented(TopAbs_FORWARD));
    const auto parameter = [&](const TopoDS_Vertex& vertex,
                               Standard_Real fallback) {
        if (vertex.IsNull()) return fallback;
        Standard_Real value = BRep_Tool::Parameter(vertex, forward);
        TopLoc_Location location;
        Standard_Real curveFirst = 0, curveLast = 0;
        const Handle(Geom_Curve) curve = BRep_Tool::Curve(
            forward, location, curveFirst, curveLast);
        if (!curve.IsNull()) value = curve->TransformedParameter(value, transform);
        return value;
    };
    first = parameter(vertex1, first);
    last = parameter(vertex2, last);
}

inline bool TransformPCurve(const Handle(Geom2d_Curve)& source,
                            const Handle(Geom_Surface)& surface,
                            const gp_Trsf& transform,
                            Standard_Real sourceFirst,
                            Standard_Real sourceLast,
                            Standard_Real targetFirst,
                            Standard_Real targetLast,
                            Handle(Geom2d_Curve)& output) {
    output.Nullify();
    if (source.IsNull()) return true;
    Handle(Geom2d_Curve) curve = Handle(Geom2d_Curve)::DownCast(source->Copy());
    if (curve.IsNull()) return false;
    if (curve->DynamicType() == STANDARD_TYPE(Geom2d_TrimmedCurve))
        curve = Handle(Geom2d_TrimmedCurve)::DownCast(curve)->BasisCurve();
    const Standard_Real basisFirst = curve->FirstParameter();
    const Standard_Real basisLast = curve->LastParameter();
    if (!curve->IsPeriodic()) {
        if (basisFirst - sourceFirst > Precision::PConfusion()) sourceFirst = basisFirst;
        if (sourceLast - basisLast > Precision::PConfusion()) sourceLast = basisLast;
        if (Abs(sourceLast - sourceFirst) < Precision::PConfusion()) {
            if (Abs(sourceFirst - basisFirst) < Precision::PConfusion()
                && !Precision::IsInfinite(basisLast)) sourceLast = basisLast;
            else if (!Precision::IsInfinite(basisFirst)) sourceFirst = basisFirst;
        }
    }
    Standard_Real transformedFirst = sourceFirst, transformedLast = sourceLast;
    if (Abs(transform.ScaleFactor()) != 1.) {
        curve = new Geom2d_TrimmedCurve(curve, sourceFirst, sourceLast);
        const gp_GTrsf2d parametric = surface->ParametricTransformation(transform);
        if (parametric.Form() != gp_Identity) {
            curve = GeomLib::GTransform(curve, parametric);
            if (curve.IsNull()) return false;
            transformedFirst = curve->FirstParameter();
            transformedLast = curve->LastParameter();
        }
    }
    GeomLib::SameRange(Precision::PConfusion(), curve,
        transformedFirst, transformedLast, targetFirst, targetLast, output);
    // The representation owns the finite range. Keep OCCT's analytic basis
    // type instead of persisting the temporary trimming wrapper used only for
    // GTransform/SameRange arithmetic (notably, Circle must remain Circle).
    if (!output.IsNull()
        && output->DynamicType() == STANDARD_TYPE(Geom2d_TrimmedCurve))
        output = Handle(Geom2d_TrimmedCurve)::DownCast(output)->BasisCurve();
    return !output.IsNull();
}

inline Handle(Geom2d_Curve) ConstructorCurve(
    const Handle(Geom2d_Curve)& first, const Handle(Geom2d_Curve)& second) {
    if (!first.IsNull()) return first;
    if (!second.IsNull()) return second;
    // OCCT's constructor dereferences its argument. This temporary is cleared
    // before attachment, so a stored-null entry remains stored and null.
    return new Geom2d_Line(gp::OX2d());
}

inline bool AppendTransported(const Handle(BRep_CurveOnSurface)& source,
                              const Handle(BRep_TEdge)& targetEdge,
                              const Handle(Geom_Surface)& sourceSurface,
                              const Handle(Geom_Surface)& targetSurface,
                              const TopLoc_Location& targetLocation,
                              const gp_Trsf& transform,
                              const TopoDS_Edge& sourceOccurrence) {
    const Standard_Real sourceFirst = source->First(), sourceLast = source->Last();
    Standard_Real targetFirst = sourceFirst, targetLast = sourceLast;
    TransformedParameters(sourceOccurrence, transform, targetFirst, targetLast);
    Handle(Geom2d_Curve) firstCurve, secondCurve;
    if (!TransformPCurve(source->PCurve(), sourceSurface, transform,
            sourceFirst, sourceLast, targetFirst, targetLast, firstCurve)) return false;
    const Handle(BRep_CurveOnClosedSurface) closed =
        Handle(BRep_CurveOnClosedSurface)::DownCast(source);
    if (!closed.IsNull()) {
        if (!TransformPCurve(closed->PCurve2(), sourceSurface, transform,
                sourceFirst, sourceLast, targetFirst, targetLast, secondCurve)) return false;
        Handle(BRep_CurveOnClosedSurface) result =
            new BRep_CurveOnClosedSurface(
                ConstructorCurve(firstCurve, secondCurve),
                ConstructorCurve(secondCurve, firstCurve), targetSurface,
                targetLocation, closed->Continuity());
        result->SetRange(targetFirst, targetLast);
        result->PCurve(firstCurve);
        result->PCurve2(secondCurve);
        targetEdge->ChangeCurves().Append(result);
    } else {
        Handle(BRep_CurveOnSurface) result = new BRep_CurveOnSurface(
            ConstructorCurve(firstCurve, secondCurve), targetSurface,
            targetLocation);
        result->SetRange(targetFirst, targetLast);
        result->PCurve(firstCurve);
        targetEdge->ChangeCurves().Append(result);
    }
    targetEdge->Modified(Standard_True);
    return true;
}

} // namespace detail

inline bool Scale(const TopoDS_Shape& source, const gp_Trsf& transform,
                  TopoDS_Shape& output) {
    output.Nullify();
    if (source.IsNull() || transform.Form() != gp_Scale
        || transform.ScaleFactor() <= 0) return false;
    BRepBuilderAPI_Transform scaler(source, transform, Standard_True);
    if (!scaler.IsDone() || scaler.Shape().IsNull()) return false;
    std::map<const BRep_CurveRepresentation*, detail::TargetRepresentation>
        transported;
    for (TopExp_Explorer faces(source, TopAbs_FACE); faces.More(); faces.Next()) {
        const TopoDS_Face sourceFace = TopoDS::Face(faces.Current());
        const TopoDS_Shape mappedFaceShape = scaler.ModifiedShape(sourceFace);
        if (mappedFaceShape.IsNull() || mappedFaceShape.ShapeType() != TopAbs_FACE)
            return false;
        const TopoDS_Face targetFace = TopoDS::Face(mappedFaceShape);
        TopLoc_Location sourceFaceLocation, targetFaceLocation;
        const Handle(Geom_Surface) sourceSurface =
            BRep_Tool::Surface(sourceFace, sourceFaceLocation);
        const Handle(Geom_Surface) targetSurface =
            BRep_Tool::Surface(targetFace, targetFaceLocation);
        if (sourceSurface.IsNull() || targetSurface.IsNull()) return false;
        if (GeomAdaptor_Surface(sourceSurface).GetType() != GeomAbs_Plane)
            continue;
        for (TopExp_Explorer edges(sourceFace, TopAbs_EDGE);
             edges.More(); edges.Next()) {
            const TopoDS_Edge sourceEdge = TopoDS::Edge(edges.Current());
            const TopoDS_Shape mappedEdgeShape = scaler.ModifiedShape(sourceEdge);
            if (mappedEdgeShape.IsNull() || mappedEdgeShape.ShapeType() != TopAbs_EDGE)
                return false;
            const TopoDS_Edge targetEdge = TopoDS::Edge(mappedEdgeShape);
            const Handle(BRep_TEdge) sourceTEdge =
                Handle(BRep_TEdge)::DownCast(sourceEdge.TShape());
            const Handle(BRep_TEdge) targetTEdge =
                Handle(BRep_TEdge)::DownCast(targetEdge.TShape());
            if (sourceTEdge.IsNull() || targetTEdge.IsNull()) return false;
            const TopLoc_Location sourceRelative =
                sourceFaceLocation.Predivided(sourceEdge.Location());
            const TopLoc_Location targetRelative =
                targetFaceLocation.Predivided(targetEdge.Location());
            for (BRep_ListIteratorOfListOfCurveRepresentation it(
                    sourceTEdge->Curves()); it.More(); it.Next()) {
                const Handle(BRep_CurveRepresentation)& representation = it.Value();
                if (representation.IsNull()
                    || !representation->IsCurveOnSurface(
                        sourceSurface, sourceRelative)) continue;
                const Handle(BRep_CurveOnSurface) curve =
                    Handle(BRep_CurveOnSurface)::DownCast(representation);
                if (curve.IsNull()) return false;
                const auto found = transported.find(representation.get());
                if (found != transported.end()) {
                    if (!detail::TargetIsSame(found->second, targetSurface,
                            targetRelative)) return false;
                    continue;
                }
                if (!detail::AppendTransported(curve, targetTEdge,
                        sourceSurface, targetSurface, targetRelative,
                        transform, sourceEdge)) return false;
                transported.emplace(representation.get(),
                    detail::TargetRepresentation{targetSurface, targetRelative});
            }
        }
    }
    output = scaler.Shape();
    return true;
}

} // namespace core3d::analytic_boolean_scale
