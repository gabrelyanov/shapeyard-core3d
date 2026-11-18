//
//  ShellOperationController.hpp
//  Core3D
//

#ifndef ShellOperationController_hpp
#define ShellOperationController_hpp

#include "AIS_InteractiveContext.hxx"
#include "OcctDocument.h"

#include <AIS_Shape.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Message_ProgressScope.hxx>
#include <Precision.hxx>
#include <TopoDS_Iterator.hxx>
#include <algorithm>
#include <cmath>
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <ShapeAnalysis_CanonicalRecognition.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Pln.hxx>
#include <BRep_Tool.hxx>
#include <GeomLib_IsPlanarSurface.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Defeaturing.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepLib.hxx>
#include <BRepOffsetAPI_MakeOffsetShape.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Vec.hxx>
#include <GeomLProp_SLProps.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Surface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pln.hxx>

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace core3d {

inline constexpr Standard_Size kMaximumShellOpeningFaces = 8;

enum class ShellOpeningAxis : std::uint8_t { X, Y, Z };
enum class ShellOpeningSide : std::uint8_t { Minimum, Maximum };
struct ShellOpeningSelector {
    ShellOpeningAxis axis;
    ShellOpeningSide side;
};

//! Local BRep frame, before occurrence placement. Bounded before any full map
//! or bounds traversal; refuses ambiguous, missing, non-planar and adjacent caps.
Standard_Boolean TryResolveShellOpeningSelectors(
    const TopoDS_Shape& shape,
    const std::vector<ShellOpeningSelector>& selectors,
    std::vector<TopoDS_Face>& faces) noexcept;

//! Detached profile rebuilds use exactly the Shell tool's admission and kernel
//! boundary. Failure/Stop publishes no shape and never touches OCAF.
Standard_Boolean ReplayProfileShells(
    const TopoDS_Shape& base, const profile::Parameters& parameters,
    std::atomic_bool& cancelled, TopoDS_Shape& result) noexcept;

//! The face's plane when it is flat: an analytic plane, or a spline that is
//! flat (an Add primitive resized with different Width/Depth/Height is stored
//! as splines). The returned plane carries the face's own geometry.
inline Standard_Boolean ShellFacePlane(
    const TopoDS_Face& theFace, gp_Pln& thePlane) noexcept
{
    if (theFace.IsNull()) {
        return Standard_False;
    }
    try {
        BRepAdaptor_Surface aSurface(theFace, Standard_True);
        if (aSurface.GetType() == GeomAbs_Plane) {
            thePlane = aSurface.Plane();
            return Standard_True;
        }
        if (aSurface.GetType() != GeomAbs_BSplineSurface
            && aSurface.GetType() != GeomAbs_BezierSurface) {
            return Standard_False;
        }
        TopLoc_Location aLocation;
        const Handle(Geom_Surface) aGeometry =
            BRep_Tool::Surface(theFace, aLocation);
        if (aGeometry.IsNull()) {
            return Standard_False;
        }
        GeomLib_IsPlanarSurface aPlanar(aGeometry, 1.0e-7);
        if (!aPlanar.IsPlanar()) {
            return Standard_False;
        }
        thePlane = aPlanar.Plan().Transformed(aLocation.Transformation());
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

inline Standard_Boolean ShellFaceIsSinglePlanarOpening(
    const TopoDS_Face& theFace) noexcept
{
    if (theFace.IsNull()) {
        return Standard_False;
    }
    try {
        gp_Pln aPlane;
        if (!ShellFacePlane(theFace, aPlane)) {
            return Standard_False;
        }
        TopTools_IndexedMapOfShape aWires;
        TopExp::MapShapes(theFace, TopAbs_WIRE, aWires);
        return aWires.Extent() == 1;
    } catch (...) {
        return Standard_False;
    }
}

// Called only after bounded source capture; indices are canonical and sorted.
inline Standard_Boolean ShellOpeningSetIsValid(
    const TopoDS_Shape& shape,
    const std::vector<TopoDS_Face>& faces,
    const std::vector<Standard_Size>& indices) noexcept
{
    if (shape.IsNull() || shape.ShapeType() != TopAbs_SOLID
        || faces.empty() || faces.size() > kMaximumShellOpeningFaces
        || faces.size() != indices.size()) return Standard_False;
    try {
        TopTools_IndexedMapOfShape canonicalFaces;
        TopExp::MapShapes(shape, TopAbs_FACE, canonicalFaces);
        TopTools_IndexedMapOfShape usedEdges;
        for (std::size_t i = 0; i < faces.size(); ++i) {
            if ((i > 0 && indices[i - 1] >= indices[i])
                || indices[i] >= static_cast<Standard_Size>(canonicalFaces.Extent())
                || !canonicalFaces.FindKey(static_cast<Standard_Integer>(indices[i] + 1)).IsEqual(faces[i])
                || (faces[i].Orientation() != TopAbs_FORWARD
                    && faces[i].Orientation() != TopAbs_REVERSED)
                || !ShellFaceIsSinglePlanarOpening(faces[i])) return Standard_False;
            TopTools_IndexedMapOfShape edges;
            TopExp::MapShapes(faces[i], TopAbs_EDGE, edges);
            for (Standard_Integer j = 1; j <= edges.Extent(); ++j) {
                if (usedEdges.Contains(edges(j))) return Standard_False;
                usedEdges.Add(edges(j));
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

//! Shared kernel boundary; single-opening flags and insertion order are unchanged.
//! Outward normal of a flat face (its plane axis, flipped for a reversed face).
inline Standard_Boolean ShellFaceOutward(const TopoDS_Face& face, gp_Pln& plane,
                                         gp_Dir& outward) noexcept
{
    if (!ShellFacePlane(face, plane)) return Standard_False;
    outward = plane.Axis().Direction();
    if (face.Orientation() == TopAbs_REVERSED) outward.Reverse();
    return Standard_True;
}

#ifdef DEBUG
//! Debug-only probes for the native Shell checks (which step refused a rebuild or a preview).
inline std::atomic<int> gShellDebugFaceShape{0};
inline std::atomic<int> gShellDebugConeStep{0};
inline std::atomic<int> gShellDebugComputeStage{0};
#define SHELL_DEBUG_PROBE(name, value) (name).store((value))
#else
#define SHELL_DEBUG_PROBE(name, value) ((void)0)
#endif

//! An Add cone resized in the Inspector (desk lamp shade, CTO 15 Nov 17:40) is stored
//! as splines like the cylinder below. One flat base square to an axis, at most one
//! parallel flat top and a round outline: rebuild the cone the user typed from its
//! bounds and the width of each flat end. Ends of equal width are a cylinder: no change.
inline Standard_Boolean ResizedConeForShell(
    const Bnd_Box& box, const std::vector<TopoDS_Face>& flat,
    const std::vector<gp_Pln>& planes, const Standard_Real tolerance,
    TopoDS_Shape& cone) noexcept
{
    cone.Nullify();
    try {
        if (flat.empty() || flat.size() > 2 || flat.size() != planes.size()) { SHELL_DEBUG_PROBE(gShellDebugConeStep, 1); return Standard_False; }
        const gp_Dir normal = planes[0].Axis().Direction();
        int k = -1;
        for (int c = 0; c < 3; ++c) {
            const gp_Dir unit(c == 0, c == 1, c == 2);
            if (std::abs(std::abs(unit.Dot(normal)) - 1.0) <= 1.0e-9) k = c;
        }
        if (k < 0) { SHELL_DEBUG_PROBE(gShellDebugConeStep, 2); return Standard_False; }
        for (const auto& plane : planes)
            if (!plane.Axis().Direction().IsParallel(normal, 1.0e-9)) { SHELL_DEBUG_PROBE(gShellDebugConeStep, 3); return Standard_False; }
        Standard_Real lo[3], hi[3];
        box.Get(lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]);
        const int i = (k + 1) % 3, j = (k + 2) % 3;
        const Standard_Real width = hi[i] - lo[i], depth = hi[j] - lo[j], height = hi[k] - lo[k];
        if (!(height > tolerance) || !(width > tolerance)
            || std::abs(width - depth) > 1.0e-3 * std::max(width, depth)) { SHELL_DEBUG_PROBE(gShellDebugConeStep, 4); return Standard_False; }
        const Standard_Real slack = std::max(tolerance, 1.0e-3 * height);
        Standard_Real radius[2] = {0.0, 0.0};
        for (std::size_t f = 0; f < flat.size(); ++f) {
            Bnd_Box capBox;
            BRepBndLib::AddOptimal(flat[f], capBox, Standard_False, Standard_False);
            if (capBox.IsVoid()) { SHELL_DEBUG_PROBE(gShellDebugConeStep, 5); return Standard_False; }
            Standard_Real c0[3], c1[3];
            capBox.Get(c0[0], c0[1], c0[2], c1[0], c1[1], c1[2]);
            const Standard_Real at = planes[f].Location().Coord(k + 1);
            const int end = std::abs(at - lo[k]) <= slack ? 0
                : (std::abs(at - hi[k]) <= slack ? 1 : -1);
            if (end < 0 || radius[end] > 0.0) { SHELL_DEBUG_PROBE(gShellDebugConeStep, 6); return Standard_False; }
            radius[end] = (c1[i] - c0[i]) / 2;
        }
        const Standard_Real wide = std::max(radius[0], radius[1]);
        if (std::abs(wide - width / 2) > 0.01 * width
            || std::abs(radius[0] - radius[1]) <= 0.02 * wide) { SHELL_DEBUG_PROBE(gShellDebugConeStep, 7); return Standard_False; }
        // The wide end is the bounds' width exactly, as the user typed it.
        radius[radius[0] >= radius[1] ? 0 : 1] = width / 2;
        gp_Pnt base;
        base.SetCoord(k + 1, lo[k]);
        base.SetCoord(i + 1, (lo[i] + hi[i]) / 2);
        base.SetCoord(j + 1, (lo[j] + hi[j]) / 2);
        const gp_Dir up(k == 0, k == 1, k == 2);
        SHELL_DEBUG_PROBE(gShellDebugConeStep, 99);
        cone = BRepPrimAPI_MakeCone(gp_Ax2(base, up), radius[0], radius[1], height).Shape();
        return !cone.IsNull();
    } catch (...) { cone.Nullify(); { SHELL_DEBUG_PROBE(gShellDebugConeStep, 8); return Standard_False; } }
}

//! An Add box or cylinder resized with different Width/Depth/Height is stored
//! as splines, and the offset kernel cannot hollow a spline cylinder. When the
//! source is exactly such a box (six flat faces facing the axes) or cylinder
//! (two parallel flat caps and one cylindrical wall), rebuild it as the exact
//! analytic primitive and map the opening faces onto it. Any doubt: no change.
inline Standard_Boolean AnalyticPrimitiveForShell(
    const TopoDS_Shape& source, const std::vector<TopoDS_Face>& openings,
    TopoDS_Shape& rebuilt, std::vector<TopoDS_Face>& rebuiltOpenings) noexcept
{
    rebuilt.Nullify(); rebuiltOpenings.clear();
    try {
        if (source.IsNull() || source.ShapeType() != TopAbs_SOLID) return Standard_False;
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(source, TopAbs_FACE, faces);
        Standard_Boolean anySpline = Standard_False;
        std::vector<TopoDS_Face> flat; std::vector<gp_Pln> planes; TopoDS_Face wall;
        for (Standard_Integer i = 1; i <= faces.Extent(); ++i) {
            const TopoDS_Face face = TopoDS::Face(faces(i));
            const BRepAdaptor_Surface surface(face, Standard_True);
            if (surface.GetType() != GeomAbs_Plane && surface.GetType() != GeomAbs_Cylinder)
                anySpline = Standard_True;
            gp_Pln plane;
            if (ShellFacePlane(face, plane)) { flat.push_back(face); planes.push_back(plane); }
            else if (wall.IsNull()) wall = face;
            else return Standard_False;
        }
        SHELL_DEBUG_PROBE(gShellDebugFaceShape, 100 * faces.Extent() + 10 * static_cast<int>(flat.size()) + (wall.IsNull() ? 0 : 1) + (anySpline ? 1000 : 0));
        if (!anySpline) return Standard_False;
        Bnd_Box box; BRepBndLib::AddOptimal(source, box, Standard_False, Standard_False);
        if (box.IsVoid()) return Standard_False;
        const Standard_Real tolerance = 1.0e-6 * std::max(1.0, std::sqrt(box.SquareExtent()));
        TopoDS_Shape candidate;
        if (faces.Extent() == 6 && flat.size() == 6 && wall.IsNull()) {
            for (const auto& plane : planes) {
                const gp_Dir n = plane.Axis().Direction();
                if (std::abs(std::abs(n.X()) + std::abs(n.Y()) + std::abs(n.Z()) - 1.0) > 1.0e-9)
                    return Standard_False;
            }
            Standard_Real x0, y0, z0, x1, y1, z1; box.Get(x0, y0, z0, x1, y1, z1);
            candidate = BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), gp_Pnt(x1, y1, z1)).Shape();
        } else if (!wall.IsNull()
                   && ((faces.Extent() == 2 && flat.size() == 1)
                       || (faces.Extent() == 3 && flat.size() == 2))
                   && ResizedConeForShell(box, flat, planes, tolerance, candidate)) {
            // A resized cone (pointed or cut flat on top): rebuilt from its bounds.
        } else if (faces.Extent() == 3 && flat.size() == 2 && !wall.IsNull()) {
            // The resize approximates the circle; recognise it loosely and
            // rebuild the cylinder the user typed (its bounding size).
            ShapeAnalysis_CanonicalRecognition recognition(wall);
            gp_Cylinder cylinder;
            if (!recognition.IsCylinder(tolerance * 1000, cylinder)) return Standard_False;
            const gp_Dir axis = cylinder.Axis().Direction();
            if (!planes[0].Axis().Direction().IsParallel(axis, 1.0e-9)
                || !planes[1].Axis().Direction().IsParallel(axis, 1.0e-9)) return Standard_False;
            const gp_Pnt origin = cylinder.Axis().Location();
            const gp_Vec along(axis);
            const Standard_Real a = gp_Vec(origin, planes[0].Location()).Dot(along);
            const Standard_Real b = gp_Vec(origin, planes[1].Location()).Dot(along);
            const Standard_Real low = std::min(a, b), high = std::max(a, b);
            if (high - low <= tolerance) return Standard_False;
            Standard_Real x0, y0, z0, x1, y1, z1; box.Get(x0, y0, z0, x1, y1, z1);
            const Standard_Real extents[3] = {x1 - x0, y1 - y0, z1 - z0};
            Standard_Real radius = cylinder.Radius();
            for (int k = 0; k < 3; ++k) {
                const gp_Dir unit(k == 0, k == 1, k == 2);
                if (std::abs(unit.Dot(axis)) < 1.0e-9) { radius = extents[k] / 2; break; }
            }
            if (std::abs(radius - cylinder.Radius()) > cylinder.Radius() * 0.02) return Standard_False;
            candidate = BRepPrimAPI_MakeCylinder(
                gp_Ax2(origin.Translated(along * low), axis), radius, high - low).Shape();
        } else {
            return Standard_False;
        }
        GProp_GProps before, after;
        BRepGProp::VolumeProperties(source, before);
        BRepGProp::VolumeProperties(candidate, after);
        if (!(before.Mass() > 0)
            || std::abs(after.Mass() - before.Mass()) > before.Mass() * 0.02) return Standard_False;
        SHELL_DEBUG_PROBE(gShellDebugFaceShape, gShellDebugFaceShape.load() + 10000);
        TopTools_IndexedMapOfShape candidateFaces;
        TopExp::MapShapes(candidate, TopAbs_FACE, candidateFaces);
        for (const auto& opening : openings) {
            gp_Pln plane; gp_Dir outward;
            if (!ShellFaceOutward(opening, plane, outward)) return Standard_False;
            TopoDS_Face match;
            for (Standard_Integer i = 1; i <= candidateFaces.Extent(); ++i) {
                const TopoDS_Face face = TopoDS::Face(candidateFaces(i));
                gp_Pln other; gp_Dir otherOutward;
                if (!ShellFaceOutward(face, other, otherOutward)) continue;
                // Same plane, either way round: a flat spline face's fitted plane may point
                // inward, and a box, cylinder or cone never has two faces on one plane.
                if (otherOutward.IsParallel(outward, 1.0e-7)
                    && other.Distance(plane.Location()) <= tolerance) {
                    if (!match.IsNull()) return Standard_False;
                    match = face;
                }
            }
            if (match.IsNull()) return Standard_False;
            rebuiltOpenings.push_back(match);
        }
        SHELL_DEBUG_PROBE(gShellDebugFaceShape, gShellDebugFaceShape.load() + 20000);
        rebuilt = candidate;
        return Standard_True;
    } catch (...) { rebuilt.Nullify(); rebuiltOpenings.clear(); return Standard_False; }
}

inline Standard_Integer UnifiedPlanarSourceForShell(
    const TopoDS_Shape& source, const std::vector<TopoDS_Face>& openings,
    TopoDS_Shape& unified, std::vector<TopoDS_Face>& unifiedFaces) noexcept;

//! Check seam (debug builds): while on, the guard below is skipped so a check
//! can record, in a process of its own, what the kernel does without it.
Standard_Boolean ShellOffsetGuardSkipped() noexcept;
void DebugSetShellOffsetGuardSkipped(Standard_Boolean skipped) noexcept;
//! How often the guard has held a shell back from the kernel in this process
//! (evidence for checks: a refusal that came from the guard, not the kernel).
void NoteShellOffsetGuardRefusal() noexcept;
Standard_Integer ShellOffsetGuardRefusalCount() noexcept;

//! Whether the offset kernel may be given this shell at all. The kernel does not
//! always refuse what it cannot hollow: for a cube with Bevel 3 on every edge,
//! opened at its bottom face, it reads through a null pointer and the app closes
//! (build 4, 15 Nov 2026: SIGSEGV in BRepOffset_Tool::TryProject under
//! ContextIntByArc). A crash cannot be caught, so the kernel is not called when
//! an opening face meets a neighbour smoothly along an edge (a flat face next
//! to a fillet): the kernel builds a rounded joint between the opening and each
//! neighbour's offset, and there is none to build. Probed without this guard,
//! one process per case (15 Nov): every such opening either closed the process
//! (fillets on every edge) or was refused by the kernel itself (a rounded
//! cylinder rim, one or four filleted edges), so nothing the kernel can make is
//! held back. A rounding smaller than the thickness away from the opening, a
//! holed part and a flat Bevel were probed too: the kernel refuses or makes
//! them, and they are left to it. Any doubt here is "not safe": the caller then
//! reports an ordinary failed preview.
inline Standard_Boolean ShellOffsetIsSafe(
    const TopoDS_Shape& source, const std::vector<TopoDS_Face>& openings) noexcept
{
    try {
        if (source.IsNull() || openings.empty()) return Standard_False;
        TopTools_IndexedDataMapOfShapeListOfShape facesOfEdges;
        TopExp::MapShapesAndAncestors(source, TopAbs_EDGE, TopAbs_FACE, facesOfEdges);
        // The surface normal of a face at a place on one of its edges. Its sign is not used:
        // an opening face reaches the worker as a copy whose orientation flag need not match
        // the original's (build 4b, first try: the guard compared signed normals, saw
        // "opposite" on the crash route and let the kernel have it).
        const auto surfaceNormal = [](const TopoDS_Face& face, const TopoDS_Edge& edge,
                                      const Standard_Real along, gp_Dir& normal) -> bool {
            Standard_Real first = 0.0, last = 0.0;
            const Handle(Geom2d_Curve) onSurface = BRep_Tool::CurveOnSurface(edge, face, first, last);
            const Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
            if (onSurface.IsNull() || surface.IsNull() || !std::isfinite(first) || !std::isfinite(last)
                || std::abs(first) > 1.0e100 || std::abs(last) > 1.0e100) return false;
            const gp_Pnt2d uv = onSurface->Value(first + (last - first) * along);
            GeomLProp_SLProps properties(surface, uv.X(), uv.Y(), 1, 1.0e-9);
            if (!properties.IsNormalDefined()) return false;
            normal = properties.Normal();
            return true;
        };
        for (const TopoDS_Face& opening : openings) {
            for (TopExp_Explorer walk(opening, TopAbs_EDGE); walk.More(); walk.Next()) {
                const TopoDS_Edge edge = TopoDS::Edge(walk.Current());
                if (BRep_Tool::Degenerated(edge)) continue;
                // Every edge of an opening has a neighbour in the solid it is cut from. An
                // edge without one means the opening is not a face of this shape: not judged
                // safe.
                if (!facesOfEdges.Contains(edge)) return Standard_False;
                const TopTools_ListOfShape& faces = facesOfEdges.FindFromKey(edge);
                Standard_Boolean hasNeighbour = Standard_False;
                for (TopTools_ListOfShape::Iterator other(faces); other.More(); other.Next()) {
                    if (other.Value().IsSame(opening)) continue;
                    hasNeighbour = Standard_True;
                    const TopoDS_Face neighbour = TopoDS::Face(other.Value());
                    // Smooth anywhere along the edge (the two surfaces' normals within about
                    // eight degrees of one line) is smooth. Two faces of a valid solid never
                    // fold back flat on each other, so the sign carries nothing.
                    for (const Standard_Real along : {0.2, 0.5, 0.8}) {
                        gp_Dir onOpening, onNeighbour;
                        if (!surfaceNormal(opening, edge, along, onOpening)
                            || !surfaceNormal(neighbour, edge, along, onNeighbour)
                            || std::abs(onOpening.Dot(onNeighbour)) >= 0.990) return Standard_False;
                    }
                }
                if (!hasNeighbour) return Standard_False;
            }
        }
        return Standard_True;
    } catch (...) { return Standard_False; }
}

//! The surface normal of a face at a place along one of its edges. Its sign is
//! not used anywhere (see ShellOffsetIsSafe).
inline Standard_Boolean ShellNormalOnEdge(
    const TopoDS_Face& face, const TopoDS_Edge& edge,
    const Standard_Real along, gp_Dir& normal) noexcept
{
    try {
        Standard_Real first = 0.0, last = 0.0;
        const Handle(Geom2d_Curve) onSurface = BRep_Tool::CurveOnSurface(edge, face, first, last);
        const Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
        if (onSurface.IsNull() || surface.IsNull() || !std::isfinite(first) || !std::isfinite(last)
            || std::abs(first) > 1.0e100 || std::abs(last) > 1.0e100) return Standard_False;
        const gp_Pnt2d uv = onSurface->Value(first + (last - first) * along);
        GeomLProp_SLProps properties(surface, uv.X(), uv.Y(), 1, 1.0e-9);
        if (!properties.IsNormalDefined()) return Standard_False;
        normal = properties.Normal();
        return Standard_True;
    } catch (...) { return Standard_False; }
}

//! How two faces of a solid meet along an edge they share: smoothly (a flat
//! face running into a rounding; the guard's own eight degrees), squarely (a
//! wall standing upright on the face), or at some other angle.
enum class ShellEdgeJoin : std::uint8_t { Unknown = 0, Sharp, Square, Smooth };

inline ShellEdgeJoin ShellJoinAlongEdge(
    const TopoDS_Face& face, const TopoDS_Face& neighbour, const TopoDS_Edge& edge) noexcept
{
    Standard_Boolean smooth = Standard_True, square = Standard_True;
    for (const Standard_Real along : {0.2, 0.5, 0.8}) {
        gp_Dir onFace, onNeighbour;
        if (!ShellNormalOnEdge(face, edge, along, onFace)
            || !ShellNormalOnEdge(neighbour, edge, along, onNeighbour)) return ShellEdgeJoin::Unknown;
        const Standard_Real dot = std::abs(onFace.Dot(onNeighbour));
        if (dot < 0.990) smooth = Standard_False;
        if (dot > 1.0e-6) square = Standard_False;
    }
    return smooth ? ShellEdgeJoin::Smooth : (square ? ShellEdgeJoin::Square : ShellEdgeJoin::Sharp);
}

//! The rounded faces of a part that an inward wall of this thickness would
//! swallow. A rounded face is a curved face that runs smoothly into a neighbour
//! along an edge (what Bevel and Round leave on an edge). The offset kernel
//! moves every face inward by the wall; a face that bulges outward with a
//! radius no larger than the wall has nowhere to go (its radius would be zero
//! or negative), and the kernel then fails on the whole part. This is the
//! reason "Shell 3" was refused on a part with a 0.8 mm Bevel.
struct ShellRoundedEdges {
    std::vector<TopoDS_Face> tooSmall;
    //! The smallest rounding radius among them, in the part's own units; 0 when none.
    Standard_Real smallestTooSmall = 0.0;
};

inline Standard_Boolean ShellRoundedEdgesOf(
    const TopoDS_Shape& source, const std::vector<TopoDS_Face>& openings,
    const Standard_Real thickness, const Standard_Real tolerance,
    ShellRoundedEdges& rounded) noexcept
{
    rounded = ShellRoundedEdges();
    try {
        if (source.IsNull() || !(thickness > 0.0)) return Standard_False;
        TopTools_IndexedDataMapOfShapeListOfShape facesOfEdges;
        TopExp::MapShapesAndAncestors(source, TopAbs_EDGE, TopAbs_FACE, facesOfEdges);
        // A rounding within two percent above the wall is counted too: the kernel
        // would have to make a rounding of nearly no radius from it.
        const Standard_Real reach = thickness + std::max(10.0 * tolerance, 0.02 * thickness);
        TopTools_IndexedMapOfShape seen;
        // The explorer gives each face the way round it sits in the solid.
        for (TopExp_Explorer walk(source, TopAbs_FACE); walk.More(); walk.Next()) {
            const TopoDS_Face face = TopoDS::Face(walk.Current());
            if (seen.Contains(face)) continue;
            seen.Add(face);
            Standard_Boolean isOpening = Standard_False;
            for (const TopoDS_Face& opening : openings)
                if (opening.IsSame(face)) isOpening = Standard_True;
            gp_Pln plane;
            if (isOpening || ShellFacePlane(face, plane)) continue;
            Standard_Boolean runsSmoothly = Standard_False;
            for (TopExp_Explorer edges(face, TopAbs_EDGE); edges.More() && !runsSmoothly; edges.Next()) {
                const TopoDS_Edge edge = TopoDS::Edge(edges.Current());
                if (BRep_Tool::Degenerated(edge) || !facesOfEdges.Contains(edge)) continue;
                for (TopTools_ListOfShape::Iterator other(facesOfEdges.FindFromKey(edge));
                     other.More(); other.Next()) {
                    // A seam has the face itself on both sides: no neighbour.
                    if (other.Value().IsSame(face)) continue;
                    if (ShellJoinAlongEdge(face, TopoDS::Face(other.Value()), edge)
                        == ShellEdgeJoin::Smooth) { runsSmoothly = Standard_True; break; }
                }
            }
            if (!runsSmoothly) continue;
            const Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
            if (surface.IsNull()) return Standard_False;
            Standard_Real u0 = 0.0, u1 = 0.0, v0 = 0.0, v1 = 0.0;
            BRepTools::UVBounds(face, u0, u1, v0, v1);
            if (!std::isfinite(u0) || !std::isfinite(u1) || !std::isfinite(v0) || !std::isfinite(v1)
                || std::abs(u0) > 1.0e100 || std::abs(u1) > 1.0e100
                || std::abs(v0) > 1.0e100 || std::abs(v1) > 1.0e100) return Standard_False;
            // Curvature is measured against the surface's own normal and is negative
            // where the surface bends away from it (a ball seen from outside). The
            // face's way round turns that normal into the solid's outward one.
            const Standard_Real outward = face.Orientation() == TopAbs_REVERSED ? -1.0 : 1.0;
            Standard_Real bulge = 0.0;
            for (const Standard_Real s : {0.25, 0.5, 0.75}) {
                for (const Standard_Real t : {0.25, 0.5, 0.75}) {
                    GeomLProp_SLProps at(surface, u0 + (u1 - u0) * s, v0 + (v1 - v0) * t, 2, 1.0e-9);
                    if (!at.IsCurvatureDefined()) continue;
                    bulge = std::max({bulge, -outward * at.MaxCurvature(), -outward * at.MinCurvature()});
                }
            }
            if (!std::isfinite(bulge) || !(bulge > 0.0)) continue;
            const Standard_Real radius = 1.0 / bulge;
            if (radius > reach) continue;
            rounded.tooSmall.push_back(face);
            if (rounded.smallestTooSmall <= 0.0 || radius < rounded.smallestTooSmall)
                rounded.smallestTooSmall = radius;
        }
        return Standard_True;
    } catch (...) { rounded = ShellRoundedEdges(); return Standard_False; }
}

//! For the refusal sentence: the rounding radius the wall has to stay under,
//! in the part's own units; 0 when no rounding stands in the way.
inline Standard_Real ShellRoundedEdgeLimit(
    const TopoDS_Shape& source, const std::vector<TopoDS_Face>& openings,
    const Standard_Real thickness, const Standard_Real tolerance) noexcept
{
    ShellRoundedEdges rounded;
    return ShellRoundedEdgesOf(source, openings, thickness, tolerance, rounded)
        ? rounded.smallestTooSmall : 0.0;
}

//! One closed skin made into a solid with its matter inside.
inline Standard_Boolean ShellSolidFromClosedSkin(const TopoDS_Shape& skin, TopoDS_Solid& solid) noexcept
{
    try {
        TopTools_IndexedMapOfShape shells;
        TopExp::MapShapes(skin, TopAbs_SHELL, shells);
        if (shells.Extent() != 1 || !BRep_Tool::IsClosed(shells(1))) return Standard_False;
        BRepBuilderAPI_MakeSolid maker(TopoDS::Shell(shells(1)));
        if (!maker.IsDone()) return Standard_False;
        solid = maker.Solid();
        if (!BRepLib::OrientClosedSolid(solid)) return Standard_False;
        GProp_GProps mass;
        BRepGProp::VolumeProperties(solid, mass);
        return std::isfinite(mass.Mass()) && mass.Mass() > 0.0;
    } catch (...) { return Standard_False; }
}

//! The one solid of a cut whose hollow has been opened: one solid, one skin.
inline Standard_Boolean ShellOneOpenedSolid(const TopoDS_Shape& shape, TopoDS_Shape& solid) noexcept
{
    try {
        TopTools_IndexedMapOfShape solids, shells;
        TopExp::MapShapes(shape, TopAbs_SOLID, solids);
        TopExp::MapShapes(shape, TopAbs_SHELL, shells);
        if (solids.Extent() != 1 || shells.Extent() != 1) return Standard_False;
        solid = solids(1);
        return Standard_True;
    } catch (...) { solid.Nullify(); return Standard_False; }
}

//! A flat face pulled straight out by a distance, as a solid with its matter inside.
inline Standard_Boolean ShellColumnOn(const TopoDS_Face& face, const gp_Vec& pull, TopoDS_Solid& column) noexcept
{
    try {
        BRepPrimAPI_MakePrism prism(face, pull);
        if (!prism.IsDone()) return Standard_False;
        TopTools_IndexedMapOfShape solids;
        TopExp::MapShapes(prism.Shape(), TopAbs_SOLID, solids);
        if (solids.Extent() != 1) return Standard_False;
        column = TopoDS::Solid(solids(1));
        return BRepLib::OrientClosedSolid(column);
    } catch (...) { return Standard_False; }
}

//! Shell through an opening that runs smoothly into a rounding (K01: a cube
//! with Bevel 3 on every edge, Shell 2 from its top). The thick-solid kernel
//! cannot be given this (see ShellOffsetIsSafe: it closes the app), because it
//! wants to stretch each neighbour's inner face up to the opening, and the
//! inner face of a rounding never gets there. The shell itself exists whenever
//! every rounding is larger than the wall, and is made here without that step:
//!   1. the whole part moved inward by the wall, as a closed body of its own
//!      (no opening is named, so the kernel never enters the code that failed);
//!   2. that body taken out of the part: a part with a closed hollow;
//!   3. for each opening, the flat face of the hollow lying under it, pulled
//!      straight out through the opening and taken out too.
//! Step 3 is exact only where the opening's neighbours run smoothly into it or
//! stand upright on it (the hollow's face under the opening is then the
//! opening itself, moved in and narrowed by the wall on the upright sides).
//! Any other neighbour: nothing is made, and the caller refuses as before.
inline Standard_Boolean ShellThroughSmoothOpening(
    const TopoDS_Shape& source, const std::vector<TopoDS_Face>& openings,
    const Standard_Real thickness, const Standard_Real tolerance,
    const Message_ProgressRange& progress, TopoDS_Shape& made) noexcept
{
    made.Nullify();
    try {
        if (source.IsNull() || source.ShapeType() != TopAbs_SOLID || openings.empty())
            return Standard_False;
        TopTools_IndexedDataMapOfShapeListOfShape facesOfEdges;
        TopExp::MapShapesAndAncestors(source, TopAbs_EDGE, TopAbs_FACE, facesOfEdges);
        for (const TopoDS_Face& opening : openings) {
            for (TopExp_Explorer walk(opening, TopAbs_EDGE); walk.More(); walk.Next()) {
                const TopoDS_Edge edge = TopoDS::Edge(walk.Current());
                if (BRep_Tool::Degenerated(edge)) continue;
                if (!facesOfEdges.Contains(edge)) return Standard_False;
                Standard_Boolean hasNeighbour = Standard_False;
                for (TopTools_ListOfShape::Iterator other(facesOfEdges.FindFromKey(edge));
                     other.More(); other.Next()) {
                    if (other.Value().IsSame(opening)) continue;
                    hasNeighbour = Standard_True;
                    const ShellEdgeJoin join =
                        ShellJoinAlongEdge(opening, TopoDS::Face(other.Value()), edge);
                    if (join != ShellEdgeJoin::Smooth && join != ShellEdgeJoin::Square)
                        return Standard_False;
                }
                if (!hasNeighbour) return Standard_False;
            }
        }
        BRepOffsetAPI_MakeOffsetShape offset;
        offset.PerformByJoin(source, -thickness, tolerance, BRepOffset_Skin,
            Standard_False, Standard_False, GeomAbs_Arc, Standard_False, progress);
        if (!offset.IsDone()) return Standard_False;
        TopoDS_Solid inner;
        if (!ShellSolidFromClosedSkin(offset.Shape(), inner)) return Standard_False;
        BRepAlgoAPI_Cut hollowed(source, inner);
        if (!hollowed.IsDone() || hollowed.HasErrors()) return Standard_False;
        TopoDS_Shape hollow = hollowed.Shape();
        const Standard_Real slack = std::max(10.0 * tolerance, 1.0e-7);
        // Past the opening's plane by a little, so that no face of the column lies on it.
        const Standard_Real lift = std::max(0.05 * thickness, 2.0 * tolerance);
        TopTools_IndexedMapOfShape innerFaces;
        TopExp::MapShapes(inner, TopAbs_FACE, innerFaces);
        for (const TopoDS_Face& opening : openings) {
            gp_Pln plane;
            if (!ShellFacePlane(opening, plane)) return Standard_False;
            const gp_Vec normal(plane.Axis().Direction());
            TopoDS_Face under;
            gp_Vec out;
            for (Standard_Integer i = 1; i <= innerFaces.Extent(); ++i) {
                const TopoDS_Face face = TopoDS::Face(innerFaces(i));
                gp_Pln other;
                if (!ShellFacePlane(face, other)
                    || !other.Axis().Direction().IsParallel(plane.Axis().Direction(), 1.0e-7)
                    || std::abs(plane.Distance(other.Location()) - thickness) > slack) continue;
                GProp_GProps area;
                BRepGProp::SurfaceProperties(face, area);
                const gp_Pnt centre = area.CentreOfMass();
                // From the face's middle straight to the opening's plane; the foot must be on the opening.
                const gp_Vec toPlane = normal * gp_Vec(centre, plane.Location()).Dot(normal);
                BRepClass_FaceClassifier where(opening, centre.Translated(toPlane), slack);
                if (where.State() != TopAbs_IN && where.State() != TopAbs_ON) continue;
                if (!under.IsNull()) return Standard_False; // two candidates: not sure which
                under = face;
                out = toPlane;
            }
            if (under.IsNull() || out.Magnitude() <= slack) return Standard_False;
            out.Normalize();
            TopoDS_Solid column;
            if (!ShellColumnOn(under, out * (thickness + lift), column)) return Standard_False;
            BRepAlgoAPI_Cut opened;
            TopTools_ListOfShape objects, tools;
            objects.Append(hollow);
            tools.Append(column);
            opened.SetArguments(objects);
            opened.SetTools(tools);
            opened.SetFuzzyValue(tolerance);
            opened.Build();
            if (!opened.IsDone() || opened.HasErrors()) return Standard_False;
            hollow = opened.Shape();
        }
        return ShellOneOpenedSolid(hollow, made);
    } catch (...) { made.Nullify(); return Standard_False; }
}

//! The routes that need no change to the part: the exact primitive behind a
//! resized one, the thick-solid kernel, the kernel again on merged flat faces,
//! and (only when `made` is given and `smoothOpenings` is true) the route above
//! for an opening the guard keeps from the kernel.
//! Returns false when the guard held the shell back and nothing was made;
//! `heldBack`, when given, is then set and the refusal is not counted here.
//! `stock`, when given, receives the solid the result was cut from.
inline Standard_Boolean MakeInwardShellPlain(
    BRepOffsetAPI_MakeThickSolid& builder,
    const TopoDS_Shape& source,
    const std::vector<TopoDS_Face>& faces,
    Standard_Real thickness, Standard_Real tolerance,
    const Message_ProgressRange& progress,
    Standard_Integer* normalizationStatus,
    const Standard_Boolean smoothOpenings,
    TopoDS_Shape* made, TopoDS_Shape* stock, Standard_Boolean* heldBack)
{
    if (normalizationStatus != nullptr) *normalizationStatus = 0;
    if (made != nullptr) made->Nullify();
    if (heldBack != nullptr) *heldBack = Standard_False;
    TopoDS_Shape analytic; std::vector<TopoDS_Face> analyticFaces;
    const Standard_Boolean rebuilt =
        AnalyticPrimitiveForShell(source, faces, analytic, analyticFaces);
    if (stock != nullptr) *stock = rebuilt ? analytic : source;
    Message_ProgressScope scope(progress, "Shell", 2);
    if (!ShellOffsetGuardSkipped()
        && !ShellOffsetIsSafe(rebuilt ? analytic : source, rebuilt ? analyticFaces : faces)) {
        if (smoothOpenings && made != nullptr
            && ShellThroughSmoothOpening(rebuilt ? analytic : source,
                                         rebuilt ? analyticFaces : faces,
                                         thickness, tolerance, scope.Next(), *made))
            return Standard_True;
        if (made != nullptr) made->Nullify();
        if (heldBack != nullptr) *heldBack = Standard_True;
        else NoteShellOffsetGuardRefusal();
        return Standard_False;
    }
    TopTools_ListOfShape openings;
    for (const auto& face : rebuilt ? analyticFaces : faces) openings.Append(face);
    builder.MakeThickSolidByJoin(rebuilt ? analytic : source, openings, -thickness, tolerance,
        BRepOffset_Skin, Standard_False, Standard_False, GeomAbs_Arc,
        Standard_False, scope.Next());
    if (rebuilt || !scope.More() || builder.MakeOffset().Error() == BRepOffset_UserBreak
        || (builder.IsDone() && builder.MakeOffset().Error() == BRepOffset_NoError))
        return Standard_True;
    TopoDS_Shape unified;
    std::vector<TopoDS_Face> unifiedFaces;
    const Standard_Integer normalized = UnifiedPlanarSourceForShell(source, faces, unified, unifiedFaces);
    if (normalizationStatus != nullptr) *normalizationStatus = normalized;
    if (normalized <= 0) return Standard_True;
    if (!scope.More()) return Standard_True;
    // Kernel history supplies the faces; exact boundary geometry proves that
    // the selected opening has not grown into a neighbouring coplanar patch.
    if (!ShellOffsetGuardSkipped() && !ShellOffsetIsSafe(unified, unifiedFaces)) {
        if (heldBack != nullptr) *heldBack = Standard_True;
        else NoteShellOffsetGuardRefusal();
        return Standard_False;
    }
    if (stock != nullptr) *stock = unified;
    TopTools_ListOfShape unifiedOpenings;
    for (const auto& face : unifiedFaces) unifiedOpenings.Append(face);
    builder.MakeThickSolidByJoin(unified, unifiedOpenings, -thickness, tolerance,
        BRepOffset_Skin, Standard_False, Standard_False, GeomAbs_Arc,
        Standard_False, scope.Next());
    return Standard_True;
}

//! Shell on a part whose roundings are smaller than the wall (K02: the lamp
//! shade, a cone cut flat with Bevel 0.8, Shell 2 or 3). The kernel cannot move
//! a 0.8 mm rounding inward by 3 mm, yet the shell exists: inside the part, at a
//! wall's distance from its skin, a rounding no larger than the wall leaves no
//! trace (the hollow has a sharp corner there, exactly as if the edge had never
//! been rounded). So the hollow is worked out on the part with those roundings
//! taken off (the kernel's own Remove Features), and then taken out of the part
//! AS IT IS. The part's history is not touched: the Bevel stays where it is in
//! the model and on the outside of the result.
//!   1. `rounds` (from ShellRoundedEdgesOf) are removed from a copy of the part;
//!   2. that copy is shelled by the ordinary routes above;
//!   3. the hollow (copy minus its shell) is lengthened a little out through each
//!      opening, so that it crosses the opening instead of ending on it;
//!   4. the hollow is taken out of the original part.
inline Standard_Boolean ShellAroundSmallRounds(
    const TopoDS_Shape& source, const std::vector<TopoDS_Face>& openings,
    const std::vector<TopoDS_Face>& rounds,
    const Standard_Real thickness, const Standard_Real tolerance,
    const Message_ProgressRange& progress, TopoDS_Shape& made) noexcept
{
    made.Nullify();
    try {
        if (source.IsNull() || source.ShapeType() != TopAbs_SOLID || rounds.empty())
            return Standard_False;
        Message_ProgressScope scope(progress, "Shell", 1);
        // Remove Features marks the shapes it reads (the "checked" flag) and the part's
        // shapes are the saved document's: it reads a deep copy, so Undo of this Shell
        // gives back the same saved bytes.
        BRepBuilderAPI_Copy copier(source, Standard_True, Standard_False);
        if (!copier.IsDone() || copier.Shape().IsNull()) return Standard_False;
        const auto copied = [&copier](const TopoDS_Face& face) -> TopoDS_Shape {
            const TopTools_ListOfShape& now = copier.Modified(face);
            return now.Extent() == 1 ? now.First() : TopoDS_Shape();
        };
        BRepAlgoAPI_Defeaturing remove;
        remove.SetShape(copier.Shape());
        TopTools_ListOfShape unwanted;
        for (const TopoDS_Face& face : rounds) {
            const TopoDS_Shape copy = copied(face);
            if (copy.IsNull()) return Standard_False;
            unwanted.Append(copy);
        }
        remove.AddFacesToRemove(unwanted);
        remove.Build();
        if (!remove.IsDone() || remove.HasErrors()) return Standard_False;
        TopTools_IndexedMapOfShape sharpSolids;
        TopExp::MapShapes(remove.Shape(), TopAbs_SOLID, sharpSolids);
        if (sharpSolids.Extent() != 1) return Standard_False;
        const TopoDS_Shape sharp = sharpSolids(1);
        if (!BRepCheck_Analyzer(sharp, Standard_True).IsValid()) return Standard_False;
        TopTools_IndexedMapOfShape sharpFaces;
        TopExp::MapShapes(sharp, TopAbs_FACE, sharpFaces);
        std::vector<TopoDS_Face> sharpOpenings;
        for (const TopoDS_Face& given : openings) {
            const TopoDS_Shape opening = copied(given);
            if (opening.IsNull() || remove.IsDeleted(opening)) return Standard_False;
            const TopTools_ListOfShape& changed = remove.Modified(opening);
            if (changed.Extent() > 1) return Standard_False;
            const TopoDS_Shape now = changed.IsEmpty() ? TopoDS_Shape(opening) : changed.First();
            const Standard_Integer index = sharpFaces.FindIndex(now);
            if (index == 0) return Standard_False;
            const TopoDS_Face face = TopoDS::Face(sharpFaces(index));
            if (!ShellFaceIsSinglePlanarOpening(face)) return Standard_False;
            sharpOpenings.push_back(face);
        }
        // The kernel leaves a feature it cannot remove in place and only warns:
        // look again, and go on only when no such rounding is left.
        ShellRoundedEdges left;
        if (!ShellRoundedEdgesOf(sharp, sharpOpenings, thickness, tolerance, left)
            || !left.tooSmall.empty()) return Standard_False;
        BRepOffsetAPI_MakeThickSolid builder;
        TopoDS_Shape shelled, stock;
        Standard_Boolean heldBack = Standard_False;
        if (!MakeInwardShellPlain(builder, sharp, sharpOpenings, thickness, tolerance,
                                  scope.Next(), nullptr, Standard_True, &shelled, &stock, &heldBack))
            return Standard_False;
        if (shelled.IsNull()) {
            if (!builder.IsDone() || builder.MakeOffset().Error() != BRepOffset_NoError)
                return Standard_False;
            shelled = builder.Shape();
        }
        if (stock.IsNull() || shelled.IsNull()
            || !BRepCheck_Analyzer(shelled, Standard_True).IsValid()) return Standard_False;
        BRepAlgoAPI_Cut emptied(stock, shelled);
        if (!emptied.IsDone() || emptied.HasErrors()) return Standard_False;
        TopTools_IndexedMapOfShape hollows;
        TopExp::MapShapes(emptied.Shape(), TopAbs_SOLID, hollows);
        if (hollows.Extent() != 1) return Standard_False;
        TopoDS_Shape hollow = hollows(1);
        GProp_GProps inside;
        BRepGProp::VolumeProperties(hollow, inside);
        if (!std::isfinite(inside.Mass()) || !(inside.Mass() > 0.0)) return Standard_False;
        const gp_Pnt middle = inside.CentreOfMass();
        const Standard_Real slack = std::max(10.0 * tolerance, 1.0e-7);
        const Standard_Real lift = std::max(0.05 * thickness, 2.0 * tolerance);
        for (const TopoDS_Face& opening : sharpOpenings) {
            gp_Pln plane;
            if (!ShellFacePlane(opening, plane)) return Standard_False;
            gp_Vec out(plane.Axis().Direction());
            if (gp_Vec(middle, plane.Location()).Dot(out) < 0.0) out.Reverse();
            TopTools_IndexedMapOfShape hollowFaces;
            TopExp::MapShapes(hollow, TopAbs_FACE, hollowFaces);
            TopoDS_Shape grown = hollow;
            Standard_Integer mouths = 0;
            for (Standard_Integer i = 1; i <= hollowFaces.Extent(); ++i) {
                const TopoDS_Face face = TopoDS::Face(hollowFaces(i));
                gp_Pln other;
                if (!ShellFacePlane(face, other)
                    || !other.Axis().Direction().IsParallel(plane.Axis().Direction(), 1.0e-7)
                    || plane.Distance(other.Location()) > slack) continue;
                TopoDS_Solid column;
                if (!ShellColumnOn(face, out * lift, column)) return Standard_False;
                BRepAlgoAPI_Fuse joined(grown, column);
                if (!joined.IsDone() || joined.HasErrors()) return Standard_False;
                grown = joined.Shape();
                ++mouths;
            }
            if (mouths == 0) return Standard_False;
            hollow = grown;
        }
        BRepAlgoAPI_Cut finished;
        TopTools_ListOfShape objects, tools;
        objects.Append(source);
        tools.Append(hollow);
        finished.SetArguments(objects);
        finished.SetTools(tools);
        finished.SetFuzzyValue(tolerance);
        finished.Build();
        if (!finished.IsDone() || finished.HasErrors()) return Standard_False;
        return ShellOneOpenedSolid(finished.Shape(), made);
    } catch (...) { made.Nullify(); return Standard_False; }
}

//! Makes an inward shell.
//! Without `made` (a saved profile's replay, a check of the kernel boundary):
//! exactly the routes that existed before 17 Nov 2026, and the caller reads the
//! builder.
//! With `made` (the Shell tool): the same routes first, so every shell that was
//! made before is made the same way, byte for byte. Only when they make nothing
//! are the two routes for a bevelled or rounded part tried; a solid they make is
//! handed back in `made` (the builder then holds nothing) and the caller
//! validates it like any other result.
//! Returns false only when the guard held the shell back and nothing was made.
inline Standard_Boolean MakeInwardShell(
    BRepOffsetAPI_MakeThickSolid& builder,
    const TopoDS_Shape& source,
    const std::vector<TopoDS_Face>& faces,
    Standard_Real thickness, Standard_Real tolerance,
    const Message_ProgressRange& progress = Message_ProgressRange(),
    Standard_Integer* normalizationStatus = nullptr,
    TopoDS_Shape* made = nullptr)
{
    if (made == nullptr)
        return MakeInwardShellPlain(builder, source, faces, thickness, tolerance, progress,
                                    normalizationStatus, Standard_False, nullptr, nullptr, nullptr);
    made->Nullify();
    ShellRoundedEdges rounded;
    const Standard_Boolean known = ShellRoundedEdgesOf(source, faces, thickness, tolerance, rounded);
    Message_ProgressScope scope(progress, "Shell", 2);
    Standard_Boolean heldBack = Standard_False;
    // The inner body of the smooth-opening route does not exist when a rounding
    // is smaller than the wall: that route is tried only when none is.
    const Standard_Boolean given = MakeInwardShellPlain(
        builder, source, faces, thickness, tolerance, scope.Next(), normalizationStatus,
        known && rounded.tooSmall.empty(), made, nullptr, &heldBack);
    const Standard_Boolean finished = !made->IsNull() || !scope.More()
        || (given && (builder.MakeOffset().Error() == BRepOffset_UserBreak
                      || (builder.IsDone() && builder.MakeOffset().Error() == BRepOffset_NoError)));
    if (!finished && known && !rounded.tooSmall.empty()
        && ShellAroundSmallRounds(source, faces, rounded.tooSmall, thickness, tolerance,
                                  scope.Next(), *made))
        return Standard_True;
    if (made->IsNull() && heldBack) NoteShellOffsetGuardRefusal();
    return given;
}

enum class ShellApplyResult : std::uint8_t {
    NoChange = 0,
    Applied,
    AppliedNeedsDocumentRedraw,
};

enum class ShellPreviewState : std::uint8_t {
    Unavailable = 0,
    Selecting,
    Computing,
    Ready,
    Committing,
    OutcomeUnknown,
    Failed,
};

//! Bounded, immutable proof that one oriented face is the canonical face of
//! one displayed editable BRep solid and that the presentation still exactly
//! mirrors its persisted XCAF definition and transform. Construction performs
//! the topology cap before any full face map or BRep validity traversal.
struct FaceOperationSourceProof {
    Handle(AIS_Shape) original;
    TDF_Label documentLabel;
    //! Exactly what the document stores and the presentation shows. Every
    //! "is this still the same part" check compares against this one.
    TopoDS_Shape shape;
    //! The one solid the tool builds from. The same object as `shape` for a
    //! plain part. App Store 5.34.1 Join/Cut results and bevels made before
    //! 15 Nov 2026 are stored as a compound around their one solid: then this
    //! is that solid, and the tool's plain result replaces the wrapper.
    TopoDS_Shape solid;
    // Single-face compatibility for Extrude; Shell consumes the ordered set below.
    TopoDS_Face openingFace;
    std::vector<TopoDS_Face> openingFaces;
    std::vector<Standard_Size> openingFaceTopologyIndices;
    gp_Trsf transform;
    Standard_Size faceTopologyIndex = 0;
    Standard_Size topologyNodeCount = 0;
    std::string entityIdentifier;
    std::string definitionIdentifier;
};

//! Read a plane without replacing the selected face. Affine dimension edits
//! store flat faces as splines; every control pole must lie in the plane and
//! the pole count is bounded before the adaptor copies any spline geometry.
Standard_Boolean TryFaceOperationPlane(
    const TopoDS_Face& face, gp_Pln& plane) noexcept;

//! The one solid a face tool builds from: the shape itself when it is a solid,
//! or the single forward solid inside a compound that wraps exactly one child
//! (taken with the wrapper's place and orientation; nothing is rebuilt). Null
//! for anything else.
TopoDS_Shape FaceOperationSolidOf(const TopoDS_Shape& shape) noexcept;

//! Produce the shared idle/begin admission proof for face-local solid tools.
//! theSelectedFace must have the same TShape, location, and orientation as the
//! canonical face embedded in the presentation; reversed and stale owners are
//! rejected rather than normalized silently.
Standard_Boolean TryPrepareFaceOperationSource(
    const Handle(AIS_InteractiveContext)& context,
    const Handle(OcctDocument)& document,
    const Handle(AIS_Shape)& presentation,
    const TopoDS_Face& selectedFace,
    Standard_Size maximumTopologyNodes,
    Standard_Size maximumStyledSubshapeLabels,
    FaceOperationSourceProof& proof,
    Standard_Boolean allowPlanarSpline = Standard_False,
    Standard_Boolean allowSavedCut = Standard_False,
    //! Push-pull and Split carry plain face colours across Apply (CarryFaceColours), so a part
    //! with a face painted in a plain colour is admitted for them; Shell still refuses it, and
    //! a face with a picture, an XCAF colour or a layer is refused for all of them.
    Standard_Boolean allowStyledFaces = Standard_False) noexcept;

//! Shell-only set capture; shares the bounded per-face admission with Extrude.
Standard_Boolean TryPrepareShellOperationSource(
    const Handle(AIS_InteractiveContext)& context,
    const Handle(OcctDocument)& document,
    const Handle(AIS_Shape)& presentation,
    const std::vector<TopoDS_Face>& selectedFaces,
    FaceOperationSourceProof& proof) noexcept;

//! Exact revalidation for an already bounded proof. Shell sets recheck their
//! canonical membership and shared edges; no kernel-wide validity traversal.
Standard_Boolean FaceOperationSourceProofIsCurrent(
    const Handle(AIS_InteractiveContext)& context,
    const Handle(OcctDocument)& document,
    const FaceOperationSourceProof& proof,
    Standard_Size maximumStyledSubshapeLabels,
    Standard_Boolean allowSavedCut = Standard_False,
    Standard_Boolean allowStyledFaces = Standard_False) noexcept;

//! DEBUG/diagnostic index resolver that preserves TopExp's canonical unique
//! face order but stops as soon as the requested face is known. Both the
//! requested index and visited Face occurrences are bounded; no full map of an
//! oversized source is constructed before production admission applies its
//! source-wide topology cap.
Standard_Boolean TryResolveCanonicalFaceTopologyIndexBounded(
    const TopoDS_Shape& shape,
    Standard_Size faceTopologyIndex,
    Standard_Size maximumFaceOccurrences,
    TopoDS_Face& face) noexcept;

//! An ordered planar-opening set captured on the main thread. The controller
//! deep-copies the source and all openings before dispatching any
//! offset work.
struct ShellSourceSelection {
    FaceOperationSourceProof proof;
    Standard_Integer selectionMode =
        AIS_Shape::SelectionMode(TopAbs_SHAPE);
};

//! Explicit renderer publication input. Scene builders never discover a Shell
//! preview by enumerating the shared AIS context.
struct ShellPreviewCapture {
    Handle(AIS_Shape) result;
    TDF_Label suppressedSourceLabel;
};

#ifdef DEBUG
struct ShellPreviewDebugState {
    // Last delivered result: 0 accepted; 1 precompute; 2 offset exception; 3 offset status;
    // 4 identity; 5 topology; 6 BRep validity; 7 volume; 8 bounds;
    // 9 containment; 10 source currentness; 11 preview installation.
    Standard_Integer planarNormalizationStatus = 0;
    Standard_Boolean usedPlanarNormalization = Standard_False;
    Standard_Integer failureStage = -1;
    Standard_Integer offsetError = -1;
    Standard_Boolean offsetDone = Standard_False;
    ShellPreviewState state = ShellPreviewState::Unavailable;
    std::uint64_t generation = 0;
    std::uint64_t submittedCount = 0;
    std::uint64_t startedCount = 0;
    std::uint64_t completedCount = 0;
    std::uint64_t cancelledCount = 0;
    std::uint64_t pendingReplacementCount = 0;
    std::uint64_t acceptedCount = 0;
    std::uint64_t staleSuppressionCount = 0;
    Standard_Boolean activeOperation = Standard_False;
    Standard_Boolean workerActive = Standard_False;
    Standard_Boolean workerPending = Standard_False;
    Standard_Boolean canApply = Standard_False;
    Standard_Boolean selectionFrozen = Standard_False;
    Standard_Boolean ownsDocumentCommand = Standard_False;
    Standard_Boolean documentCommandOpen = Standard_False;
    Standard_Boolean lastComputeWasMainThread = Standard_False;
    Standard_Real thickness = 0.0;
    Standard_Real minimumThickness = 0.0;
    Standard_Real maximumThickness = 0.0;
    Standard_Real metersPerUnit = 0.0;
    Standard_Integer capturedFaceTopologyIndex = -1;
    std::vector<Standard_Size> capturedFaceTopologyIndices;
    Standard_Size sourceTopologyNodeCount = 0;
    Standard_Size candidateTopologyNodeCount = 0;
    Standard_Size candidateSolidCount = 0;
    Standard_Integer debugFaceShape = 0, debugConeStep = 0, debugComputeStage = 0;
    Standard_Real sourceVolume = 0.0;
    Standard_Real candidateVolume = 0.0;
    Standard_Real candidateBounds[6] = {};
};
#endif

class ShellPreviewWorker;
struct ShellPreviewWorkerResult;

//! Owns the complete transient and transactional lifetime of one inward
//! hollowing operation. Kernel work is latest-wins and off-main; OCAF remains
//! untouched until Apply replaces the captured definition on the same label.
class ShellOperationController
    : public std::enable_shared_from_this<ShellOperationController> {
public:
    static constexpr Standard_Size kMaximumOpeningFaces = kMaximumShellOpeningFaces;
    static constexpr Standard_Size kMaximumSourceTopologyNodes = 1'024;
    static constexpr Standard_Size kMaximumStyledSubshapeLabels = 1'024;
    static constexpr Standard_Size kMaximumResultTopologyNodes = 32'768;
    static constexpr Standard_Size kMaximumResultSolids = 1;
    static constexpr Standard_Real kMaximumPhysicalThicknessMeters = 0.020;
    static constexpr Standard_Real kDefaultPhysicalThicknessMeters = 0.002;
    static constexpr Standard_Real kMaximumThicknessFraction = 0.45;
    static constexpr Standard_Real kDefaultThicknessFraction = 0.10;

    ShellOperationController() = delete;
    ShellOperationController(
        Handle(AIS_InteractiveContext) context,
        Handle(OcctDocument) document);
    ~ShellOperationController() noexcept;

    //! Full, read-only idle admission using the same preparation proof as
    //! begin(). It never cancels an existing operation or changes controller,
    //! document, selection, or presentation state.
    Standard_Boolean canBegin(
        const ShellSourceSelection& selection) const noexcept;
    Standard_Boolean begin(const ShellSourceSelection& selection) noexcept;
    Standard_Boolean setThickness(Standard_Real thickness) noexcept;
    Standard_Boolean toggleOpening(const Handle(AIS_Shape)& presentation,
                                   const TopoDS_Face& face) noexcept;
    ShellApplyResult apply() noexcept;
    Standard_Boolean cancel() noexcept;

    Standard_Real thickness() const noexcept;
    Standard_Real defaultThickness() const noexcept;
    Standard_Real metersPerUnit() const noexcept;
    std::pair<Standard_Real, Standard_Real> thicknessRange() const noexcept;
    //! When the current preview failed and a rounded edge smaller than the wall
    //! stands in the way: that edge's radius in the part's own units (the wall
    //! has to stay under it). 0 otherwise. Release behaviour: the refusal
    //! sentence is built from it.
    Standard_Real refusedRoundedEdgeRadius() const noexcept;
    Standard_Boolean canApply() const noexcept;
    Standard_Boolean hasActiveOperation() const noexcept;
    Standard_Boolean hasUnresolvedState() const noexcept;
    Standard_Boolean isSelectionFrozen() const noexcept;
    ShellPreviewState previewState() const noexcept;
    std::uint64_t previewGeneration() const noexcept;
    Standard_Boolean capturePreview(
        ShellPreviewCapture& capture) const noexcept;
    //! Returns only the owned displayed zero-mode result retained while the
    //! document outcome is unknown. It does not publish committed geometry.
    Standard_Boolean captureSelectionModeSuspendedPresentations(
        std::vector<Handle(AIS_Shape)>& presentations) const noexcept;
    void setPreviewStateChangedCallback(std::function<void()> callback);

#ifdef DEBUG
    ShellPreviewDebugState debugPreviewState() const noexcept;
    void debugSetWorkerBlocked(Standard_Boolean blocked) noexcept;
    void debugSetMaximumCaptureTopologyNodes(Standard_Size limit) noexcept;
    void debugSetMaximumResultTopologyNodes(Standard_Size limit) noexcept;
    void debugSetTransactionFailureCount(Standard_Size count) noexcept;
    void debugSetAbortFailureCount(Standard_Size count) noexcept;
    void debugSetPreviewEraseFailureCount(Standard_Size count) noexcept;
    //! Check seam: lets a check, in a process of its own, give the kernel what the
    //! offset guard holds back (see ShellOffsetIsSafe).
    static void debugSetOffsetGuardSkipped(Standard_Boolean skipped) noexcept;
    static Standard_Integer debugOffsetGuardRefusalCount() noexcept;
    void debugSetCommitMode(Standard_Integer mode) noexcept;
    void debugSetPostCommitInspectFailureCount(Standard_Size count) noexcept;
    Standard_Boolean debugMutateSourcePersistedTransform() noexcept;
    Standard_Boolean debugMutateSourcePersistedShape() noexcept;
#endif

private:
    struct Source {
        Handle(AIS_Shape) original;
        TDF_Label label;
        // What the document stores (identity checks) and the one solid inside
        // it (geometry); the same object for a plain part.
        TopoDS_Shape shape;
        TopoDS_Shape solid;
        std::vector<TopoDS_Face> openingFaces;
        gp_Trsf transform;
        Standard_Integer selectionMode =
            AIS_Shape::SelectionMode(TopAbs_SHAPE);
        std::vector<Standard_Size> openingFaceTopologyIndices;
        Standard_Size topologyNodeCount = 0;
        Standard_Real documentMetersPerUnit = 0.0;
        // Physical metres per local BRep thickness unit, including the
        // captured object's uniform presentation scale.
        Standard_Real metersPerUnit = 0.0;
        Standard_Real sourceVolume = 0.0;
        Standard_Real minimumThickness = 0.0;
        Standard_Real maximumThickness = 0.0;
        std::string entityIdentifier;
        std::string definitionIdentifier;
        profile::Record profileRecord;
        std::vector<int> profileShellOpenings;
        // A standalone saved Hole that Apply bakes inside its own command
        // (PrepareBooleanResultForFaceEdit); Undo restores the hole record.
        Standard_Boolean bakesSavedCut = Standard_False;
    };

    Standard_Boolean tryPrepareSource(
        const ShellSourceSelection& selection,
        std::unique_ptr<Source>& source) const noexcept;

    enum class DocumentState : std::uint8_t {
        Original = 0,
        Candidate,
        OpenCommand,
        Other,
        Unavailable,
    };

    Standard_Boolean enqueuePreviewRequest() noexcept;
    void acceptWorkerResult(ShellPreviewWorkerResult result) noexcept;
    void cancelWorkerRequests() noexcept;
    Standard_Boolean installPreview(const TopoDS_Shape& result) noexcept;
    Standard_Boolean discardPreview(Standard_Boolean restoreSource) noexcept;
    Standard_Boolean sourceIsCurrent() const noexcept;
    std::string selectionFingerprint(Standard_Boolean& complete) const;
    DocumentState inspectDocumentState() const noexcept;
    Standard_Boolean abortOwnedCommand() noexcept;
    ShellApplyResult finishCommittedOperation() noexcept;
    void notifyPreviewStateChanged() noexcept;
    void clearState() noexcept;

private:
    Handle(AIS_InteractiveContext) myContext;
    Handle(OcctDocument) myDoc;
    std::unique_ptr<Source> mySource;
    Handle(AIS_Shape) myPreviewResult;
    TopoDS_Shape myPendingCandidate;
    std::optional<profile::Parameters> myPendingProfile;
    std::shared_ptr<ordinary_naming::Record> myNamingHistory;
    std::shared_ptr<ShellPreviewWorker> myWorker;
    std::function<void()> myPreviewStateChangedCallback;
    ShellPreviewState myState = ShellPreviewState::Unavailable;
    Standard_Real myThickness = 0.0;
    std::uint64_t myGeneration = 0;
    std::string myRequestedFingerprint;
    Standard_Boolean myCanApply = Standard_False;
    Standard_Real myRefusedRoundedEdge = 0.0;
    std::uint64_t myRefusedRoundedEdgeGeneration = 0;
    Standard_Boolean mySelectionFrozen = Standard_False;
    Standard_Boolean myStateValid = Standard_True;
    Standard_Boolean myOwnsDocumentCommand = Standard_False;
#ifdef DEBUG
    Standard_Integer myDebugPlanarNormalizationStatus = 0;
    Standard_Boolean myDebugUsedPlanarNormalization = Standard_False;
    Standard_Integer myDebugFailureStage = -1;
    Standard_Integer myDebugOffsetError = -1;
    Standard_Boolean myDebugOffsetDone = Standard_False;
    Standard_Boolean myDebugWorkerBlocked = Standard_False;
    Standard_Size myDebugMaximumCaptureTopologyNodes =
        kMaximumSourceTopologyNodes;
    Standard_Size myDebugMaximumResultTopologyNodes =
        kMaximumResultTopologyNodes;
    Standard_Size myDebugTransactionFailureCount = 0;
    Standard_Size myDebugAbortFailureCount = 0;
    Standard_Size myDebugPreviewEraseFailureCount = 0;
    Standard_Integer myDebugCommitMode = 0;
    Standard_Size myDebugPostCommitInspectFailureCount = 0;
    std::uint64_t myDebugAcceptedCount = 0;
    std::uint64_t myDebugStaleSuppressionCount = 0;
#endif

    friend class ShellPreviewWorker;
};

//! Private offset input only. Callers have already bounded the complete source.
//! Refuse partial coplanar openings. A rebuilt face needs unique kernel history
//! and exactly the original 3D boundary curves, ranges and placements.
inline Standard_Integer UnifiedPlanarSourceForShell(
    const TopoDS_Shape& source, const std::vector<TopoDS_Face>& openings,
    TopoDS_Shape& unified, std::vector<TopoDS_Face>& unifiedFaces) noexcept
{
    unified.Nullify(); unifiedFaces.clear();
    try {
        if (source.IsNull() || source.ShapeType() != TopAbs_SOLID || openings.empty())
            return -__LINE__;
        // Bound occurrences, including shared topology, before any full map or
        // kernel traversal; use the existing Shell source budget unchanged.
        const auto bounded = [](const TopoDS_Shape& shape) {
            const Standard_Size maximum = ShellOperationController::kMaximumSourceTopologyNodes;
            Standard_Size visited = 0;
            std::vector<TopoDS_Shape> pending{shape};
            while (!pending.empty()) {
                const TopoDS_Shape current = pending.back(); pending.pop_back();
                if (current.IsNull() || ++visited > maximum) return false;
                for (TopoDS_Iterator child(current); child.More(); child.Next()) {
                    if (visited + pending.size() >= maximum) return false;
                    pending.push_back(child.Value());
                }
            }
            return visited > 0;
        };
        if (!bounded(source)) return -__LINE__;
        TopTools_IndexedMapOfShape faces;
        TopExp::MapShapes(source, TopAbs_FACE, faces);
        for (Standard_Integer i = 1; i <= faces.Extent(); ++i) {
            if (BRepAdaptor_Surface(TopoDS::Face(faces(i)), Standard_True).GetType()
                != GeomAbs_Plane) return -__LINE__;
        }
        ShapeUpgrade_UnifySameDomain merge(source, Standard_True, Standard_True, Standard_False);
        merge.SetSafeInputMode(Standard_True);
        for (const auto& opening : openings) {
            const Standard_Integer index = faces.FindIndex(opening);
            if (index == 0 || !faces(index).IsEqual(opening)) return -__LINE__;
            const gp_Pln plane = BRepAdaptor_Surface(opening, Standard_True).Plane();
            for (Standard_Integer i = 1; i <= faces.Extent(); ++i) {
                if (i == index) continue;
                const gp_Pln other = BRepAdaptor_Surface(TopoDS::Face(faces(i)), Standard_True).Plane();
                if (plane.Axis().Direction().IsParallel(other.Axis().Direction(), Precision::Angular())
                    && plane.Distance(other.Location()) <= Precision::Confusion())
                    return -__LINE__;
            }
            TopTools_IndexedMapOfShape boundary;
            TopExp::MapShapes(opening, TopAbs_EDGE, boundary);
            TopExp::MapShapes(opening, TopAbs_VERTEX, boundary);
            for (Standard_Integer i = 1; i <= boundary.Extent(); ++i) merge.KeepShape(boundary(i));
        }
        merge.Build();
        const TopoDS_Shape candidate = merge.Shape();
        if (candidate.IsNull() || candidate.ShapeType() != TopAbs_SOLID || candidate.IsSame(source)
            || !bounded(candidate))
            return -__LINE__;
        TopTools_IndexedMapOfShape candidateFaces;
        TopExp::MapShapes(candidate, TopAbs_FACE, candidateFaces);
        if (candidateFaces.Extent() >= faces.Extent()) return -__LINE__;
        const auto history = merge.History();
        if (history.IsNull()) return -__LINE__;
        std::vector<TopoDS_Face> mappedFaces;
        TopTools_IndexedMapOfShape usedFaces;
        const auto mapped = [&](const TopoDS_Shape& original,
                                const TopTools_IndexedMapOfShape& members) -> TopoDS_Shape {
            Standard_Integer index = members.FindIndex(original);
            if (index > 0) return members(index);
            if (history->IsRemoved(original) || !history->Generated(original).IsEmpty()) return {};
            const auto& changed = history->Modified(original);
            if (changed.Extent() != 1) return {};
            index = members.FindIndex(changed.First());
            return index > 0 ? members(index) : TopoDS_Shape();
        };
        for (const auto& opening : openings) {
            const TopoDS_Shape mappedShape = mapped(opening, candidateFaces);
            if (mappedShape.IsNull() || usedFaces.Contains(mappedShape)) return -__LINE__;
            const TopoDS_Face face = TopoDS::Face(mappedShape);
            if (!ShellFaceIsSinglePlanarOpening(face)) return -__LINE__;
            gp_Pln beforePlane, afterPlane; gp_Dir beforeNormal, afterNormal;
            if (!ShellFaceOutward(opening, beforePlane, beforeNormal)
                || !ShellFaceOutward(face, afterPlane, afterNormal)
                || !beforeNormal.IsEqual(afterNormal, Precision::Angular())
                || beforePlane.Distance(afterPlane.Location()) > Precision::Confusion())
                return -__LINE__;
            TopTools_IndexedMapOfShape oldEdges, newEdges, usedEdges;
            TopExp::MapShapes(opening, TopAbs_EDGE, oldEdges);
            TopExp::MapShapes(face, TopAbs_EDGE, newEdges);
            if (oldEdges.Extent() != newEdges.Extent()) return -__LINE__;
            for (Standard_Integer i = 1; i <= oldEdges.Extent(); ++i) {
                const TopoDS_Shape nextEdge = mapped(oldEdges(i), newEdges);
                if (nextEdge.IsNull() || usedEdges.Contains(nextEdge)) return -__LINE__;
                TopLoc_Location oldLocation, newLocation;
                Standard_Real oldFirst, oldLast, newFirst, newLast;
                const auto oldCurve = BRep_Tool::Curve(TopoDS::Edge(oldEdges(i)), oldLocation, oldFirst, oldLast);
                const auto newCurve = BRep_Tool::Curve(TopoDS::Edge(nextEdge), newLocation, newFirst, newLast);
                // Safe-input mode may copy a TEdge to update its pcurves. That
                // may not alter any actual 3D curve, trim range or placement.
                if (oldCurve.IsNull() || oldCurve != newCurve || !oldLocation.IsEqual(newLocation)
                    || oldFirst != newFirst || oldLast != newLast) return -__LINE__;
                usedEdges.Add(nextEdge);
            }
            usedFaces.Add(face); mappedFaces.push_back(face);
        }
        if (!BRepCheck_Analyzer(candidate, Standard_True).IsValid()) return -__LINE__;
        GProp_GProps before, after;
        BRepGProp::VolumeProperties(source, before);
        BRepGProp::VolumeProperties(candidate, after);
        if (!std::isfinite(before.Mass()) || !std::isfinite(after.Mass()) || before.Mass() <= 0
            || std::abs(after.Mass() - before.Mass()) > 1.0e-10 * std::max(1.0, before.Mass()))
            return -__LINE__;
        Bnd_Box beforeBox, afterBox;
        BRepBndLib::AddOptimal(source, beforeBox, Standard_False, Standard_False);
        BRepBndLib::AddOptimal(candidate, afterBox, Standard_False, Standard_False);
        if (beforeBox.IsVoid() || afterBox.IsVoid() || beforeBox.IsOpen() || afterBox.IsOpen())
            return -__LINE__;
        Standard_Real a[6], b[6];
        beforeBox.Get(a[0], a[1], a[2], a[3], a[4], a[5]);
        afterBox.Get(b[0], b[1], b[2], b[3], b[4], b[5]);
        for (int i = 0; i < 6; ++i)
            if (!std::isfinite(a[i]) || !std::isfinite(b[i])
                || std::abs(a[i] - b[i]) > Precision::Confusion()) return -__LINE__;
        unified = candidate;
        unifiedFaces = std::move(mappedFaces);
        return candidateFaces.Extent();
    } catch (...) { unified.Nullify(); unifiedFaces.clear(); return -__LINE__; }
}

} // namespace core3d

#endif // ShellOperationController_hpp
