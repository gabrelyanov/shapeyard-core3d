//
//  BooleanOperationController.hpp
//  Core3D
//

#ifndef BooleanOperationController_hpp
#define BooleanOperationController_hpp

#include "AIS_InteractiveContext.hxx"
#include "OcctDocument.h"

#include <AIS_Shape.hxx>

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace core3d {

enum BooleanAction {
    BooleanSubtract = 0,
    BooleanUnion = 1,
    BooleanIntersect = 2,
};

enum BooleanSelectionType {
    Undefined = -1,
    Actor = 0,
    Subject,
};

enum class BooleanApplyResult {
    NoChange = 0,
    Applied,
    AppliedNeedsDocumentRedraw,
};

enum class BooleanPreviewState : std::uint8_t {
    Selecting = 0,
    Computing,
    Ready,
    Committing,
    Failed,
};

#ifdef DEBUG
struct BooleanPreviewDebugState {
    BooleanPreviewState state = BooleanPreviewState::Selecting;
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
    Standard_Boolean documentCommandUnresolved = Standard_False;
    Standard_Boolean documentCommandOpen = Standard_False;
    Standard_Boolean lastComputeWasMainThread = Standard_False;
};
#endif

struct TemporalBooleanObject {
    Handle(AIS_InteractiveObject) original;
    BooleanSelectionType selectionType = BooleanSelectionType::Undefined;
    TDF_Label documentLabel;
    Graphic3d_NameOfMaterial materialName =
        Graphic3d_NameOfMaterial_ShinyPlastified;
    Quantity_NameOfColor colorName = Quantity_NOC_GRAY80;
};

//! Explicit, controller-owned publication input. The snapshot builder receives
//! only these handles and source labels; it never enumerates AIS context.
struct BooleanPreviewCapture {
    BooleanAction action = BooleanAction::BooleanSubtract;
    std::vector<Handle(AIS_Shape)> actors;
    std::vector<Handle(AIS_Shape)> results;
    std::vector<TDF_Label> suppressedSourceLabels;
};

class BooleanPreviewWorker;
struct BooleanPreviewWorkerResult;

class BooleanOperationController
    : public std::enable_shared_from_this<BooleanOperationController> {
public:
    static constexpr std::size_t kMaxSourceOperands = 8;
    static constexpr Standard_Size kMaxSourceTopologyNodes = 1'024;
    static constexpr Standard_Size kMaxCaptureTopologyNodes = 8'192;
    static constexpr Standard_Size kMaxResultTopologyNodes = 32'768;
    static constexpr Standard_Size kMaxResultSolids = 1'024;

    BooleanOperationController() = delete;
    BooleanOperationController(
        Handle(AIS_InteractiveContext),
        Handle(OcctDocument) doc);
    ~BooleanOperationController() noexcept;

    Standard_Boolean begin(BooleanAction action) noexcept;
    void updateDetectedState(
        Handle(AIS_InteractiveObject) detected,
        Handle(SelectMgr_EntityOwner) detectedOwner,
        Standard_Boolean forceActor,
        BooleanAction action) noexcept;
    Standard_Boolean setSelectionState(
        const Handle(AIS_InteractiveObject)& object,
        BooleanSelectionType type,
        BooleanAction action) noexcept;
    Standard_Boolean visualApply(BooleanAction action) noexcept;
    BooleanApplyResult apply(BooleanAction action) noexcept;
    void cancel(BooleanAction action) noexcept;
    void cancelActive() noexcept;

    Standard_Boolean canApply() const noexcept;
    Standard_Boolean hasActiveOperation() const noexcept;
    Standard_Boolean hasActiveOperation(BooleanAction action) const noexcept;
    Standard_Boolean hasSelectionState() const noexcept;
    Standard_Boolean hasUnresolvedState() const noexcept;
    Standard_Boolean isSelectionFrozen() const noexcept;
    Standard_Boolean capturePreview(
        BooleanPreviewCapture& capture) const noexcept;
    BooleanPreviewState previewState() const noexcept;
    std::uint64_t previewGeneration() const noexcept;
    void setPreviewStateChangedCallback(std::function<void()> callback);
#ifdef DEBUG
    //! Production result validator exposed only for deterministic fixtures.
    static Standard_Boolean debugValidateSolidResult(
        const TopoDS_Shape& shape) noexcept;
    BooleanPreviewDebugState debugPreviewState() const noexcept;
    void debugSetWorkerBlocked(Standard_Boolean blocked) noexcept;
    void debugSetMaximumCaptureTopologyNodes(Standard_Size limit) noexcept;
    void debugSetMaximumResultTopologyNodes(Standard_Size limit) noexcept;
    void debugSetMaximumResultSolids(Standard_Size limit) noexcept;
    void debugSetTransactionFailureCount(Standard_Size count) noexcept;
    void debugSetAbortFailureCount(Standard_Size count) noexcept;
#endif

private:
    Standard_Boolean actionMatches(BooleanAction action) const noexcept;
    Standard_Boolean enqueuePreviewRequest(BooleanAction action) noexcept;
    void acceptWorkerResult(BooleanPreviewWorkerResult result) noexcept;
    void cancelWorkerRequests() noexcept;
    void supersedeWorkerRequests() noexcept;
    void notifyPreviewStateChanged() noexcept;
    std::string currentSelectionFingerprint(
        BooleanAction action,
        Standard_Boolean& isComplete) const;
    Standard_Boolean resetCachedSelection() noexcept;
    Standard_Boolean installSubtractPreview(
        const std::vector<TopoDS_Shape>& results) noexcept;
    Standard_Boolean installSingleResultPreview(
        const TopoDS_Shape& result) noexcept;
    Standard_Boolean cancelImpl() noexcept;
    void clearOperationState() noexcept;
    void markInvalid() noexcept;
    void rememberOwned(
        const Handle(AIS_InteractiveObject)& presentation) noexcept;
    Standard_Boolean cleanupOwnedPresentations() noexcept;
    Standard_Boolean pruneOwnedPresentations() noexcept;
    void rollbackFailedTransaction(
        const Handle(TDocStd_Document)& document) noexcept;
    Standard_Boolean abortDocumentCommandNoThrow(
        const Handle(TDocStd_Document)& document) noexcept;

    void showInteractiveByType(
        const Handle(AIS_InteractiveObject)& shape,
        BooleanSelectionType type);
    void rememberSubjectSelection(const TDF_Label& label);
    void forgetSubjectSelection(const TDF_Label& label);
    std::vector<Handle(AIS_InteractiveObject)> orderedSubjectPresentations(
        Standard_Boolean& isComplete) const;
    std::vector<TDF_Label> orderedSourceLabels(
        Standard_Boolean& isComplete) const;
    void applyStyle(
        Handle(AIS_InteractiveObject)& object,
        const TemporalBooleanObject& style);
    void persistStyle(
        const TDF_Label& label,
        const TemporalBooleanObject& style);
    Handle(AIS_InteractiveObject) ioCopyWithStyle(
        const Handle(AIS_InteractiveObject)& original,
        const TemporalBooleanObject& style);

private:
    std::vector<Handle(AIS_InteractiveObject)> _actedIOArray;
    std::vector<Handle(AIS_InteractiveObject)> _actorIOArray;
    std::vector<TDF_Label> _subjectSelectionOrder;
    Handle(AIS_Shape) _singleTrialResult;
    std::vector<Handle(AIS_InteractiveObject)> _ownedPresentations;
    std::optional<BooleanAction> _activeAction;
    Standard_Boolean _canApply = Standard_False;
    Standard_Boolean _stateValid = Standard_True;
    Standard_Boolean _selectionFrozen = Standard_False;
    Standard_Boolean _documentCommandUnresolved = Standard_False;
    BooleanPreviewState _previewState = BooleanPreviewState::Selecting;
    std::uint64_t _previewGeneration = 0;
    std::uint64_t _controllerEpoch = 0;
    std::string _requestedFingerprint;
    std::shared_ptr<BooleanPreviewWorker> _previewWorker;
    std::function<void()> _previewStateChangedCallback;
#ifdef DEBUG
    Standard_Boolean _debugWorkerBlocked = Standard_False;
    Standard_Size _debugMaximumCaptureTopologyNodes =
        kMaxCaptureTopologyNodes;
    Standard_Size _debugMaximumResultTopologyNodes =
        kMaxResultTopologyNodes;
    Standard_Size _debugMaximumResultSolids = kMaxResultSolids;
    Standard_Size _debugTransactionFailureCount = 0;
    Standard_Size _debugAbortFailureCount = 0;
    std::uint64_t _debugAcceptedCount = 0;
    std::uint64_t _debugStaleSuppressionCount = 0;
#endif

    Handle(AIS_InteractiveContext) myContext;
    Handle(OcctDocument) myDoc;
    std::map<Handle(AIS_InteractiveObject), TemporalBooleanObject> _selectionMap;

    friend class BooleanPreviewWorker;
};

} // namespace core3d

#endif // BooleanOperationController_hpp
