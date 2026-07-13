//
//  BevelOperationController.hpp
//  Core3D
//

#ifndef BevelOperationController_hpp
#define BevelOperationController_hpp

#include "AIS_InteractiveContext.hxx"
#include "OcctDocument.h"

#include <AIS_Shape.hxx>
#include <TopoDS_Edge.hxx>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace core3d {

enum class BevelApplyResult : std::uint8_t {
    NoChange = 0,
    Applied,
    AppliedNeedsDocumentRedraw,
};

enum class BevelPreviewState : std::uint8_t {
    Selecting = 0,
    Computing,
    Ready,
    Committing,
    Failed,
};

//! Immutable selection captured on the main thread before any worker starts.
//! Positive values produce a round fillet and negative values a flat chamfer.
struct BevelSourceSelection {
    Handle(AIS_Shape) original;
    TDF_Label documentLabel;
    std::vector<TopoDS_Edge> edges;
    //! Zero-based indices in deterministic TopExp edge traversal order.
    std::vector<Standard_Size> edgeTopologyIndices;
    Standard_Integer selectionMode = AIS_Shape::SelectionMode(TopAbs_SHAPE);
};

//! Explicit renderer publication input. The snapshot builder never discovers
//! transient Bevel geometry by enumerating the AIS context.
struct BevelPreviewCapture {
    std::vector<Handle(AIS_Shape)> results;
    std::vector<TDF_Label> suppressedSourceLabels;
};

#ifdef DEBUG
struct BevelPreviewDebugState {
    BevelPreviewState state = BevelPreviewState::Selecting;
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
    Standard_Boolean documentCommandOpen = Standard_False;
    Standard_Boolean lastComputeWasMainThread = Standard_False;
    Standard_Real value = 0.0;
    Standard_Size sourceCount = 0;
    Standard_Size edgeCount = 0;
    Standard_Size sourceEdgeCount = 0;
    Standard_Integer capturedEdgeTopologyIndex = -1;
    Standard_Real capturedEdgeLength = 0.0;
    Standard_Size candidateTopologyNodeCount = 0;
    Standard_Size candidateSolidCount = 0;
    Standard_Real candidateVolume = 0.0;
    Standard_Real candidateBounds[6] = {};
};
#endif

class BevelPreviewWorker;
struct BevelPreviewWorkerResult;

class BevelOperationController
    : public std::enable_shared_from_this<BevelOperationController> {
public:
    static constexpr std::size_t kMaxSourceBodies = 8;
    static constexpr Standard_Size kMaxSelectedEdges = 64;
    static constexpr Standard_Size kMaxSourceTopologyNodes = 1'024;
    static constexpr Standard_Size kMaxCaptureTopologyNodes = 8'192;
    static constexpr Standard_Size kMaxStyledSubshapeLabels = 1'024;
    static constexpr Standard_Size kMaxResultTopologyNodes = 32'768;
    static constexpr Standard_Size kMaxResultSolids = 8;
    static constexpr Standard_Real kMaxDistance = 20.0;

    BevelOperationController() = delete;
    BevelOperationController(
        Handle(AIS_InteractiveContext) context,
        Handle(OcctDocument) document);
    ~BevelOperationController() noexcept;

    Standard_Boolean begin(
        const std::vector<BevelSourceSelection>& selection) noexcept;
    Standard_Boolean setValue(Standard_Real signedDistance) noexcept;
    BevelApplyResult apply() noexcept;
    Standard_Boolean cancel() noexcept;

    Standard_Boolean canApply() const noexcept;
    Standard_Boolean hasActiveOperation() const noexcept;
    Standard_Boolean isSelectionFrozen() const noexcept;
    BevelPreviewState previewState() const noexcept;
    Standard_Boolean capturePreview(
        BevelPreviewCapture& capture) const noexcept;
    void setPreviewStateChangedCallback(std::function<void()> callback);

#ifdef DEBUG
    BevelPreviewDebugState debugPreviewState() const noexcept;
    void debugSetWorkerBlocked(Standard_Boolean blocked) noexcept;
    void debugSetMaximumCaptureTopologyNodes(Standard_Size limit) noexcept;
    void debugSetMaximumResultTopologyNodes(Standard_Size limit) noexcept;
    void debugSetMaximumResultSolids(Standard_Size limit) noexcept;
    void debugSetTransactionFailureCount(Standard_Size count) noexcept;
    void debugSetCancelDiscardFailureCount(Standard_Size count) noexcept;
    Standard_Boolean debugMutateFirstSourcePersistedTransform() noexcept;
#endif

private:
    struct Source {
        Handle(AIS_Shape) original;
        TDF_Label label;
        TopoDS_Shape shape;
        std::vector<TopoDS_Edge> edges;
        std::vector<Standard_Size> edgeTopologyIndices;
        gp_Trsf transform;
        Standard_Integer selectionMode =
            AIS_Shape::SelectionMode(TopAbs_SHAPE);
        std::string entityIdentifier;
    };

    Standard_Boolean enqueuePreviewRequest() noexcept;
    void acceptWorkerResult(BevelPreviewWorkerResult result) noexcept;
    void cancelWorkerRequests() noexcept;
    Standard_Boolean installPreview(
        const std::vector<TopoDS_Shape>& results) noexcept;
    Standard_Boolean discardPreview(
        Standard_Boolean restoreSources) noexcept;
    Standard_Boolean sourcesAreCurrent() const noexcept;
    std::string selectionFingerprint(Standard_Boolean& complete) const;
    void notifyPreviewStateChanged() noexcept;
    void clearState() noexcept;
    Standard_Boolean abortOpenCommand(
        const Handle(TDocStd_Document)& document) noexcept;

private:
    Handle(AIS_InteractiveContext) myContext;
    Handle(OcctDocument) myDoc;
    std::vector<Source> mySources;
    std::vector<Handle(AIS_Shape)> myPreviewResults;
    std::shared_ptr<BevelPreviewWorker> myWorker;
    std::function<void()> myPreviewStateChangedCallback;
    BevelPreviewState myState = BevelPreviewState::Selecting;
    Standard_Real myValue = 0.0;
    std::uint64_t myGeneration = 0;
    std::string myRequestedFingerprint;
    Standard_Boolean myCanApply = Standard_False;
    Standard_Boolean mySelectionFrozen = Standard_False;
    Standard_Boolean myStateValid = Standard_True;
#ifdef DEBUG
    Standard_Boolean myDebugWorkerBlocked = Standard_False;
    Standard_Size myDebugMaximumCaptureTopologyNodes =
        kMaxCaptureTopologyNodes;
    Standard_Size myDebugMaximumResultTopologyNodes =
        kMaxResultTopologyNodes;
    Standard_Size myDebugMaximumResultSolids = kMaxResultSolids;
    Standard_Size myDebugTransactionFailureCount = 0;
    Standard_Size myDebugCancelDiscardFailureCount = 0;
    std::uint64_t myDebugAcceptedCount = 0;
    std::uint64_t myDebugStaleSuppressionCount = 0;
#endif

    friend class BevelPreviewWorker;
};

} // namespace core3d

#endif // BevelOperationController_hpp
