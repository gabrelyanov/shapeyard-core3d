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
#include <TopoDS.hxx>
#include <gp_Cylinder.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <ShapeAnalysis_CanonicalRecognition.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Pln.hxx>
#include <BRep_Tool.hxx>
#include <GeomLib_IsPlanarSurface.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pln.hxx>

#include <cstdint>
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
                if (otherOutward.IsEqual(outward, 1.0e-9)
                    && other.Distance(plane.Location()) <= tolerance) {
                    if (!match.IsNull()) return Standard_False;
                    match = face;
                }
            }
            if (match.IsNull()) return Standard_False;
            rebuiltOpenings.push_back(match);
        }
        rebuilt = candidate;
        return Standard_True;
    } catch (...) { rebuilt.Nullify(); rebuiltOpenings.clear(); return Standard_False; }
}

inline void MakeInwardShell(
    BRepOffsetAPI_MakeThickSolid& builder,
    const TopoDS_Shape& source,
    const std::vector<TopoDS_Face>& faces,
    Standard_Real thickness, Standard_Real tolerance,
    const Message_ProgressRange& progress = Message_ProgressRange())
{
    TopoDS_Shape analytic; std::vector<TopoDS_Face> analyticFaces;
    const Standard_Boolean rebuilt =
        AnalyticPrimitiveForShell(source, faces, analytic, analyticFaces);
    TopTools_ListOfShape openings;
    for (const auto& face : rebuilt ? analyticFaces : faces) openings.Append(face);
    builder.MakeThickSolidByJoin(rebuilt ? analytic : source, openings, -thickness, tolerance,
        BRepOffset_Skin, Standard_False, Standard_False, GeomAbs_Arc,
        Standard_False, progress);
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
    TopoDS_Shape shape;
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
    Standard_Boolean allowSavedCut = Standard_False) noexcept;

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
    Standard_Boolean allowSavedCut = Standard_False) noexcept;

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
    void debugSetCommitMode(Standard_Integer mode) noexcept;
    void debugSetPostCommitInspectFailureCount(Standard_Size count) noexcept;
    Standard_Boolean debugMutateSourcePersistedTransform() noexcept;
    Standard_Boolean debugMutateSourcePersistedShape() noexcept;
#endif

private:
    struct Source {
        Handle(AIS_Shape) original;
        TDF_Label label;
        TopoDS_Shape shape;
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
    std::shared_ptr<ShellPreviewWorker> myWorker;
    std::function<void()> myPreviewStateChangedCallback;
    ShellPreviewState myState = ShellPreviewState::Unavailable;
    Standard_Real myThickness = 0.0;
    std::uint64_t myGeneration = 0;
    std::string myRequestedFingerprint;
    Standard_Boolean myCanApply = Standard_False;
    Standard_Boolean mySelectionFrozen = Standard_False;
    Standard_Boolean myStateValid = Standard_True;
    Standard_Boolean myOwnsDocumentCommand = Standard_False;
#ifdef DEBUG
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

} // namespace core3d

#endif // ShellOperationController_hpp
