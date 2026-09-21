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
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <cstdint>
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

inline Standard_Boolean ShellFaceIsSinglePlanarOpening(
    const TopoDS_Face& theFace) noexcept
{
    if (theFace.IsNull()) {
        return Standard_False;
    }
    try {
        BRepAdaptor_Surface aSurface(theFace, Standard_True);
        if (aSurface.GetType() != GeomAbs_Plane) {
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
inline void MakeInwardShell(
    BRepOffsetAPI_MakeThickSolid& builder,
    const TopoDS_Shape& source,
    const std::vector<TopoDS_Face>& faces,
    Standard_Real thickness, Standard_Real tolerance,
    const Message_ProgressRange& progress = Message_ProgressRange())
{
    TopTools_ListOfShape openings;
    for (const auto& face : faces) openings.Append(face);
    builder.MakeThickSolidByJoin(source, openings, -thickness, tolerance,
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
    FaceOperationSourceProof& proof) noexcept;

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
    Standard_Size maximumStyledSubshapeLabels) noexcept;

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
