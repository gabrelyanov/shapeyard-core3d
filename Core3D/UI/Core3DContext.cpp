//
//  Core3DContext.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 04.06.2024.
//

#include "Core3DContext.hpp"
#include "XCAFPrs_Style.hxx"
#include <AIS_Shape.hxx>
#include <Prs3d_IsoAspect.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <Select3D_SensitiveCylinder.hxx>
#include <Select3D_SensitiveTriangulation.hxx>
#include <BRep_Tool.hxx>
#include <TopoDS.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <TopExp_Explorer.hxx>
#include <V3d_View.hxx>
#include <gp_Lin.hxx>
#include <gp_Vec.hxx>
#include <Precision.hxx>
#include <Standard_ErrorHandler.hxx>
#include <StdSelect_ViewerSelector3d.hxx>
#include <algorithm>
#include <vector>
#include <cmath>
Core3DContext::Core3DContext(const Handle(V3d_Viewer)& MainViewer)
: AIS_InteractiveContext(MainViewer){
    
}

// A trimmed/hollow cylindrical face is not a solid cylinder. OCCT's cylinder
// sensitive can omit the visible wall altogether (including a zero-hit pick),
// so reordering detected candidates cannot repair it. Use the face's existing
// rendered triangulation, retaining the exact native owner and mode/filter
// authority. Never mesh, manufacture owners, or change the selected set here.
static bool UseRenderedCylindricalFaceSensitives(Core3DContext& context)
{
    const auto selector = context.MainSelector();
    const auto manager = context.SelectionManager();
    if (selector.IsNull() || manager.IsNull()) return false;
    AIS_ListOfInteractive displayed;
    context.DisplayedObjects(AIS_KOI_Shape, -1, displayed);
    if (displayed.Extent() > 256) return false;
    bool changed = false;
    const int mode = AIS_Shape::SelectionMode(TopAbs_FACE);
    for (AIS_ListIteratorOfListOfInteractive object(displayed); object.More(); object.Next()) {
        const auto shape = Handle(AIS_Shape)::DownCast(object.Value());
        if (shape.IsNull() || !selector->IsActive(shape, mode)) continue;
        const auto selection = shape->Selection(mode);
        if (selection.IsNull() || selection->Entities().Size() > 4096) continue;
        struct Replacement {
            int index;
            Handle(SelectMgr_SensitiveEntity) oldEntity, newEntity;
        };
        std::vector<Replacement> replacements;
        int index = 0;
        for (const auto& entity : selection->Entities()) {
            const int current = index++;
            if (entity.IsNull()) continue;
            const auto cylinder = Handle(Select3D_SensitiveCylinder)::DownCast(entity->BaseSensitive());
            if (cylinder.IsNull()) continue;
            const auto owner = Handle(StdSelect_BRepOwner)::DownCast(cylinder->OwnerId());
            if (owner.IsNull() || !owner->HasShape() || owner->Selectable() != shape
                || owner->Shape().ShapeType() != TopAbs_FACE) continue;
            TopLoc_Location location;
            const auto mesh = BRep_Tool::Triangulation(TopoDS::Face(owner->Shape()), location);
            if (mesh.IsNull() || mesh->NbTriangles() == 0 || mesh->NbTriangles() > 100000) continue;
            Handle(Select3D_SensitiveTriangulation) triangles =
                new Select3D_SensitiveTriangulation(owner, mesh, location, Standard_True);
            triangles->SetSensitivityFactor(cylinder->SensitivityFactor());
            Handle(SelectMgr_SensitiveEntity) replacement = new SelectMgr_SensitiveEntity(triangles);
            if (entity->IsActiveForSelection()) replacement->SetActiveForSelection();
            replacements.push_back({current, entity, replacement});
        }
        if (replacements.empty()) continue;
        context.ClearDetected(Standard_False);
        manager->ClearSelectionStructures(shape, mode);
        try {
            for (const auto& replacement : replacements)
                selection->ChangeEntities().ChangeValue(replacement.index) = replacement.newEntity;
            manager->RestoreSelectionStructures(shape, mode);
        } catch (...) {
            for (const auto& replacement : replacements)
                selection->ChangeEntities().ChangeValue(replacement.index) = replacement.oldEntity;
            manager->RestoreSelectionStructures(shape, mode);
            throw;
        }
        changed = true;
    }
    return changed;
}

// Every pick path that reads the detected owner after its own MoveTo (a tap's
// Select, the Cut/Join/Overlap and Shell taps' hitTest, the mirror-plane pick)
// goes through here, so the edge-on rule is the same for all of them (Fable
// review 17, finding 1).
AIS_StatusOfDetection Core3DContext::MoveTo(
    Standard_Integer x, Standard_Integer y,
    const Handle(V3d_View)& view, Standard_Boolean redraw)
{
    const auto status = moveToRefiningCylinders(x, y, view, redraw);
    passEdgeOnFaceHit(view, redraw);
    return status;
}

namespace {

// A face hit whose surface normal is at a right angle to the pick ray: the
// face is seen exactly edge-on. Edges and points carry no normal (0 vector).
bool IsEdgeOnFaceHit(const SelectMgr_SortCriterion& hit, const gp_Dir& ray)
{
    const Standard_Real length = hit.Normal.Modulus();
    if (!(length > 0.5)) return false;
    const Standard_Real cosine = (hit.Normal.x() * ray.X() + hit.Normal.y() * ray.Y()
                                + hit.Normal.z() * ray.Z()) / length;
    return std::abs(cosine) < 1.0e-4;
}

bool IsCameraFacingFaceHit(const SelectMgr_SortCriterion& hit, const gp_Dir& ray)
{
    return hit.Normal.Modulus() > 0.5 && !IsEdgeOnFaceHit(hit, ray)
        && !Precision::IsInfinite(hit.Point.X()) && !Precision::IsInfinite(hit.Point.Y())
        && !Precision::IsInfinite(hit.Point.Z());
}

Standard_Integer PickedRank(const Handle(StdSelect_ViewerSelector3d)& selector,
                            const Handle(SelectMgr_EntityOwner)& owner)
{
    const Standard_Integer count = std::min<Standard_Integer>(selector->NbPicked(), 256);
    for (Standard_Integer rank = 1; rank <= count; ++rank)
        if (selector->Picked(rank) == owner) return rank;
    return 0;
}

enum class RayMeets { Unknown, No, Yes };

// Where the exact pick ray meets the owner's own shape, nearest first, as a
// position along the ray (in the part's frame, moved back to the world). No:
// the ray passes beside it (the part is only near the tap, seen edge-on).
// Unknown: no shape, or too large to test on a tap; OCCT's choice then stays.
RayMeets NearestRayHitAlong(const Handle(SelectMgr_EntityOwner)& entity,
                            const gp_Pnt& origin, const gp_Dir& ray, Standard_Real& along)
{
    const auto owner = Handle(StdSelect_BRepOwner)::DownCast(entity);
    if (owner.IsNull() || !owner->HasShape() || !owner->HasSelectable()) return RayMeets::Unknown;
    const TopoDS_Shape& shape = owner->Shape();
    Standard_Integer faces = 0;
    for (TopExp_Explorer face(shape, TopAbs_FACE); face.More(); face.Next())
        if (++faces > 4096) return RayMeets::Unknown;
    if (faces == 0) return RayMeets::Unknown;
    const gp_Trsf toWorld = owner->Selectable()->Transformation();
    const gp_Trsf toPart = toWorld.Inverted();
    IntCurvesFace_ShapeIntersector intersector;
    intersector.Load(shape, 1.0e-7);
    intersector.Perform(gp_Lin(origin.Transformed(toPart), ray.Transformed(toPart)), -1.0e7, 1.0e7);
    if (!intersector.IsDone()) return RayMeets::Unknown;
    bool found = false;
    for (Standard_Integer index = 1; index <= intersector.NbPnt(); ++index) {
        const gp_Pnt world = intersector.Pnt(index).Transformed(toWorld);
        const Standard_Real distance = gp_Vec(origin, world).Dot(gp_Vec(ray));
        // Only what lies ahead of the pick's near point can be seen.
        if (!std::isfinite(distance) || distance < -1.0e-3) continue;
        if (!found || distance < along) { along = distance; found = true; }
    }
    return found ? RayMeets::Yes : RayMeets::No;
}

} // namespace

// OCCT ranks a triangle seen exactly edge-on (its plane holds the view ray) at
// a false depth near 0, so a covered cutter's side face won a tap on the body's
// top 12 mm above it (toy car, measure's COVERED-CUTTER-TAP-TRACE.md items 6-8).
// When the first hit is such a face, the exact ray decides: the part keeps the
// tap only where the ray really meets it no farther than the next face that
// faces the camera; otherwise that next face is the detected one. Edge and
// vertex hits, gizmo owners, other drawing layers and parts over 4096 faces are
// left to OCCT.
void Core3DContext::passEdgeOnFaceHit(const Handle(V3d_View)& view, Standard_Boolean redraw)
{
    if (view.IsNull() || !HasDetected()) return;
    try {
        OCC_CATCH_SIGNALS
        const Handle(StdSelect_ViewerSelector3d)& selector = MainSelector();
        if (selector.IsNull()) return;
        const Handle(SelectMgr_EntityOwner) first = DetectedOwner();
        const Standard_Integer firstRank = PickedRank(selector, first);
        if (firstRank < 1 || Handle(StdSelect_BRepOwner)::DownCast(first).IsNull()) return;
        // The selector's own near/far ray: the pixel convention of the pick.
        const auto& volume = selector->GetManager();
        const gp_Pnt origin = volume.GetNearPickedPnt(), far = volume.GetFarPickedPnt();
        const Standard_Real length = origin.Distance(far);
        if (!std::isfinite(length) || length <= Precision::Confusion()) return;
        const gp_Dir ray(gp_Vec(origin, far));
        if (!IsEdgeOnFaceHit(selector->PickedData(firstRank), ray)) return;

        // The next detected face that faces the camera, in OCCT's order (the
        // detected list already holds only what the filters let through).
        Handle(SelectMgr_EntityOwner) next;
        Standard_Integer nextRank = 0;
        bool passedFirst = false;
        for (InitDetected(); MoreDetected(); NextDetected()) {
            const Handle(SelectMgr_EntityOwner) owner = DetectedCurrentOwner();
            if (owner == first) { passedFirst = true; continue; }
            if (!passedFirst || Handle(StdSelect_BRepOwner)::DownCast(owner).IsNull()) continue;
            const Standard_Integer rank = PickedRank(selector, owner);
            if (rank < 1 || !IsCameraFacingFaceHit(selector->PickedData(rank), ray)) continue;
            next = owner; nextRank = rank;
            break;
        }
        if (next.IsNull()) return;
        const SelectMgr_SortCriterion& nextHit = selector->PickedData(nextRank);
        // Layers drawn on top win by OCCT's own rule, whatever the depth.
        if (nextHit.ZLayerPosition != selector->PickedData(firstRank).ZLayerPosition) return;
        const Standard_Real nextAlong = gp_Vec(origin, nextHit.Point).Dot(gp_Vec(ray));
        Standard_Real firstAlong = 0.0;
        const RayMeets meets = NearestRayHitAlong(first, origin, ray, firstAlong);
        if (meets == RayMeets::Unknown) return;
        if (meets == RayMeets::Yes && firstAlong <= nextAlong + std::max(nextHit.Tolerance, 1.0e-6))
            return; // the edge-on part really is in front (a part standing on another)
        // Public cycling keeps dynamic highlight, the detected index and XOR
        // selection in agreement; SelectDetected then takes this owner.
        const Standard_Integer steps = 2 * std::min<Standard_Integer>(selector->NbPicked(), 256) + 2;
        for (Standard_Integer step = 0; step < steps && DetectedOwner() != next; ++step)
            if (HilightNextDetected(view, Standard_False) == 0) break;
        if (DetectedOwner() != next) {
            // Not reachable through the detected list: leave OCCT's choice.
            for (Standard_Integer step = 0; step < steps && DetectedOwner() != first; ++step)
                if (HilightNextDetected(view, Standard_False) == 0) break;
        }
        if (redraw) view->RedrawImmediate();
    } catch (...) {
        // OCCT's own detection stays in charge; nothing was edited.
    }
}

AIS_StatusOfDetection Core3DContext::moveToRefiningCylinders(
    Standard_Integer x, Standard_Integer y,
    const Handle(V3d_View)& view, Standard_Boolean redraw)
{
    auto status = AIS_InteractiveContext::MoveTo(x, y, view, redraw);
    try {
        if (UseRenderedCylindricalFaceSensitives(*this))
            status = AIS_InteractiveContext::MoveTo(x, y, view, redraw);
    } catch (...) {
        // The replacement restores its original native entities on failure.
        status = AIS_InteractiveContext::MoveTo(x, y, view, redraw);
    }
    const auto original = Handle(StdSelect_BRepOwner)::DownCast(DetectedOwner());
    if (original.IsNull() || !original->HasShape()
        || original->Shape().ShapeType() != TopAbs_FACE || view.IsNull()) return status;

    try {
        const auto& selector = MainSelector();
        if (selector.IsNull() || selector->NbPicked() < 2 || selector->NbPicked() > 32
            || (!view->ClipPlanes().IsNull() && !view->ClipPlanes()->IsEmpty())) return status;
        bool hasCylinder = false;
        const int layer = selector->PickedData(1).ZLayerPosition;
        for (int rank = 1; rank <= selector->NbPicked(); ++rank) {
            if (!Handle(Select3D_SensitiveCylinder)::DownCast(selector->PickedEntity(rank)).IsNull())
                hasCylinder = true;
            if (selector->PickedData(rank).ZLayerPosition != layer) return status;
        }
        if (!hasCylinder) return status;

        // Keep OCCT's filtered owner list and its layer/clip/overlay authority.
        // A Boolean-trimmed cylinder sensitive can report a hit below the actual
        // face, making a hidden inner wall win. Compare exact intersections of
        // the already-detected faces; no document or selection-BVH mutation.
        std::vector<Handle(StdSelect_BRepOwner)> candidates;
        int edges = 0;
        for (InitDetected(); MoreDetected(); NextDetected()) {
            const auto owner = Handle(StdSelect_BRepOwner)::DownCast(DetectedCurrentOwner());
            if (owner.IsNull() || !owner->HasShape() || !owner->HasSelectable()
                || owner->Shape().ShapeType() != TopAbs_FACE) return status;
            const auto& object = owner->Selectable();
            if (!object->TransformPersistence().IsNull()
                || (!object->ClipPlanes().IsNull() && !object->ClipPlanes()->IsEmpty())) return status;
            for (TopExp_Explorer edge(owner->Shape(), TopAbs_EDGE); edge.More(); edge.Next())
                if (++edges > 256) return status;
            candidates.push_back(owner);
        }
        if (candidates.size() < 2) return status;

        // Use the selector's own near/far ray, preserving camera clipping and
        // exactly the pixel convention used for the original detection.
        const auto& volume = selector->GetManager();
        const gp_Pnt near = volume.GetNearPickedPnt(), far = volume.GetFarPickedPnt();
        const double length = near.Distance(far);
        if (!std::isfinite(length) || length <= Precision::Confusion()) return status;
        double closest = length;
        Handle(StdSelect_BRepOwner) best;
        for (const auto& owner : candidates) {
            // TopoDS locations reject authored scale/mirroring. Keep the face
            // intact and bring the finite camera ray into its stored frame.
            // Convert every hit back to world distance before comparing owners.
            const gp_Trsf inverse = owner->Selectable()->Transformation().Inverted();
            const gp_Pnt localNear = near.Transformed(inverse), localFar = far.Transformed(inverse);
            const double localLength = localNear.Distance(localFar);
            if (!std::isfinite(localLength) || localLength <= Precision::Confusion()) return status;
            const double worldPerLocal = length / localLength;
            const gp_Lin localRay(localNear, gp_Dir(gp_Vec(localNear, localFar)));
            IntCurvesFace_ShapeIntersector intersection;
            intersection.Load(owner->Shape(), Precision::Confusion());
            intersection.PerformNearest(localRay, 0, closest / worldPerLocal);
            if (!intersection.IsDone()) return status;
            for (int point = 1; point <= intersection.NbPnt(); ++point) {
                const double depth = intersection.WParameter(point) * worldPerLocal;
                if (depth >= 0 && depth < closest) { closest = depth; best = owner; }
            }
        }
        if (best.IsNull() || best == original) return status;
        // Public cycling keeps dynamic highlight, the detected index and XOR
        // selection in agreement. Never publish a fabricated BRep owner.
        for (std::size_t step = 0; step < candidates.size() && DetectedOwner() != best; ++step)
            HilightNextDetected(view, Standard_False);
        if (DetectedOwner() != best) {
            for (std::size_t step = 0; step < candidates.size() && DetectedOwner() != original; ++step)
                HilightNextDetected(view, Standard_False);
        }
        if (redraw) view->RedrawImmediate();
    } catch (...) {
        // An unavailable exact refinement leaves ordinary OCCT detection in
        // charge; no shapes, topology, selection sets or history were edited.
    }
    return status;
}

void Core3DContext::DisableSurfaceIsolines(const Handle(Prs3d_Drawer)& theDrawer)
{
    if (theDrawer.IsNull()) return;
    // Colour assignment and XCAF style dispatch create their own line aspects,
    // overriding the context default. Selection must show topological edges:
    // a dimension edit can turn an analytic plane into a planar B-spline, for
    // which IsoOnPlane(false) no longer suppresses interior parameter lines.
    for (const bool isU : {true, false})
    {
        const Handle(Prs3d_IsoAspect)& iso = isU
            ? theDrawer->UIsoAspect() : theDrawer->VIsoAspect();
        if (iso.IsNull() || iso->Number() == 0) continue;
        const auto& aspect = iso->Aspect();
        const Handle(Prs3d_IsoAspect) withoutLines = new Prs3d_IsoAspect(
            aspect->Color(), aspect->Type(), aspect->Width(), 0);
        if (isU) theDrawer->SetUIsoAspect(withoutLines);
        else theDrawer->SetVIsoAspect(withoutLines);
    }
}

void Core3DContext::ApplyDefaultMaterial(const Handle(AIS_InteractiveObject)& obj) {
    obj->SetMaterial(Graphic3d_NameOfMaterial_ShinyPlastified);
    obj->SetColor(Quantity_NOC_GRAY80);
}

void Core3DContext::Display (const Handle(AIS_InteractiveObject)& theIObj,
                             const Standard_Boolean               theToUpdateViewer) {  
    if (!Handle(AIS_Shape)::DownCast(theIObj).IsNull()) {
        DisableSurfaceIsolines(theIObj->Attributes());
    }
    AIS_InteractiveContext::Display(theIObj, theToUpdateViewer);
    
}

void Core3DContext::Display (const Handle(AIS_InteractiveObject)& theIObj,
                             const Standard_Integer               theDispMode,
                             const Standard_Integer               theSelectionMode,
                             const Standard_Boolean               theToUpdateViewer,
                             const PrsMgr_DisplayStatus           theDispStatus) {
    if (!Handle(AIS_Shape)::DownCast(theIObj).IsNull()) {
        DisableSurfaceIsolines(theIObj->Attributes());
    }
    AIS_InteractiveContext::Display(theIObj, theDispMode, theSelectionMode, theToUpdateViewer, theDispStatus);
}

#ifdef DEBUG
Standard_Boolean Core3DContext::DebugSetDetectedOwner(
    const Handle(SelectMgr_EntityOwner)& theOwner) noexcept
{
    if (theOwner.IsNull() || !theOwner->HasSelectable()) {
        return Standard_False;
    }
    try {
        myLastPicked = theOwner;
        return !myLastPicked.IsNull();
    } catch (...) {
        myLastPicked.Nullify();
        return Standard_False;
    }
}
#endif
