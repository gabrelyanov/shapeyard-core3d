//
//  ShellOperationController.hpp
//  Core3D
//

#ifndef ShellOperationController_hpp
#define ShellOperationController_hpp

#include "AIS_InteractiveContext.hxx"
#include "OcctDocument.h"

#include <AIS_Shape.hxx>
#include <TopoDS_Face.hxx>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace core3d {

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

//! One immutable planar-face selection captured on the main thread. The
//! controller deep-copies both the source and opening before dispatching any
//! offset work.
struct ShellSourceSelection {
    Handle(AIS_Shape) original;
    TDF_Label documentLabel;
    TopoDS_Face openingFace;
    //! Zero-based index in deterministic TopExp face traversal order.
    Standard_Size faceTopologyIndex = 0;
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

    Standard_Boolean begin(const ShellSourceSelection& selection) noexcept;
    Standard_Boolean setThickness(Standard_Real thickness) noexcept;
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
#endif

private:
    struct Source {
        Handle(AIS_Shape) original;
        TDF_Label label;
        TopoDS_Shape shape;
        TopoDS_Face openingFace;
        gp_Trsf transform;
        Standard_Integer selectionMode =
            AIS_Shape::SelectionMode(TopAbs_SHAPE);
        Standard_Size faceTopologyIndex = 0;
        Standard_Size topologyNodeCount = 0;
        Standard_Real metersPerUnit = 0.0;
        Standard_Real sourceVolume = 0.0;
        Standard_Real minimumThickness = 0.0;
        Standard_Real maximumThickness = 0.0;
        std::string entityIdentifier;
        std::string definitionIdentifier;
    };

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
