//
//  LinearArrayOperationController.hpp
//  Core3D
//

#ifndef LinearArrayOperationController_hpp
#define LinearArrayOperationController_hpp

#include "Core3DContext.hpp"
#include "OcctDocument.h"

#include <AIS_Shape.hxx>

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace core3d {

enum class LinearArrayAxis : std::uint8_t {
    X = 0,
    Y,
    Z,
};

enum class LinearArrayApplyResult : std::uint8_t {
    NoChange = 0,
    Applied,
    AppliedNeedsDocumentRedraw,
};

enum class LinearArrayPreviewState : std::uint8_t {
    Unavailable = 0,
    Selecting,
    Ready,
    Committing,
    OutcomeUnknown,
    Failed,
};

#ifdef DEBUG
struct LinearArrayPreviewDebugState {
    LinearArrayPreviewState state =
        LinearArrayPreviewState::Unavailable;
    std::uint64_t generation = 0;
    Standard_Size previewBodyCount = 0;
    Standard_Size pendingResultCount = 0;
    Standard_Boolean activeOperation = Standard_False;
    Standard_Boolean previewValid = Standard_False;
    Standard_Boolean canApply = Standard_False;
    Standard_Boolean ownsDocumentCommand = Standard_False;
    Standard_Boolean documentCommandOpen = Standard_False;
    LinearArrayAxis axis = LinearArrayAxis::X;
    Standard_Integer count = 3;
    Standard_Real spacing = 0.0;
};
#endif

//! Owns the complete transient and transactional lifetime of one touch-first
//! linear array. The source definition is never changed. Preview bodies are
//! shallow AIS presentations; persisted results are independent deep copies.
class LinearArrayOperationController {
public:
    static constexpr Standard_Integer kMinimumCount = 2;
    static constexpr Standard_Integer kMaximumCount = 16;
    static constexpr Standard_Integer kDefaultCount = 3;
    static constexpr std::size_t kMaximumPreviewBodies = 15;
    static constexpr Standard_Size kMaximumSourceTopologyNodes = 1'024;
    static constexpr Standard_Size kMaximumAggregateTopologyNodes = 32'768;
    static constexpr Standard_Real kMaximumArraySpan = 10'000.0;

    LinearArrayOperationController() = delete;
    LinearArrayOperationController(
        Handle(AIS_InteractiveContext) context,
        Handle(OcctDocument) document);
    ~LinearArrayOperationController() noexcept;

    Standard_Boolean begin() noexcept;
    LinearArrayApplyResult apply() noexcept;
    Standard_Boolean cancel() noexcept;

    Standard_Boolean setAxis(LinearArrayAxis axis) noexcept;
    Standard_Boolean setCount(Standard_Integer count) noexcept;
    Standard_Boolean setSpacing(Standard_Real spacing) noexcept;

    LinearArrayAxis axis() const noexcept;
    Standard_Integer count() const noexcept;
    Standard_Real spacing() const noexcept;
    Standard_Real metersPerUnit() const noexcept;
    std::pair<Standard_Integer, Standard_Integer> countRange() const noexcept;
    std::pair<Standard_Real, Standard_Real> spacingRange() const noexcept;

    Standard_Boolean canApply() const noexcept;
    Standard_Boolean hasActiveOperation() const noexcept;
    Standard_Boolean hasUnresolvedState() const noexcept;
    LinearArrayPreviewState previewState() const noexcept;
    std::uint64_t previewGeneration() const noexcept;

    //! Captures only a complete, current Ready preview. False is a fail-closed
    //! request for the caller to retain OCCT presentation for this frame.
    Standard_Boolean capturePreview(
        std::vector<Handle(AIS_Shape)>& previewObjects) const noexcept;
    //! True only for the stable zero-spacing state after every owned preview
    //! body has been erased. This permits an explicit renderer-neutral clear
    //! without weakening failure or recovery presentation.
    Standard_Boolean canPublishEmptyPreview() const noexcept;
    void setPreviewStateChangedCallback(std::function<void()> callback);

#ifdef DEBUG
    LinearArrayPreviewDebugState debugPreviewState() const noexcept;
    void debugSetTransactionFailureCount(Standard_Size count) noexcept;
    void debugSetAbortFailureCount(Standard_Size count) noexcept;
    void debugSetEraseFailureCount(Standard_Size count) noexcept;
    void debugSetCommitMode(Standard_Integer mode) noexcept;
    void debugSetPostCommitInspectFailureCount(Standard_Size count) noexcept;
    void debugSetMaximumTopologyNodes(Standard_Size limit) noexcept;
    Standard_Boolean debugMutateFirstSourcePersistedTransform() noexcept;
#endif

private:
    struct SourceSnapshot {
        Handle(TDocStd_Document) document;
        Handle(AIS_Shape) presentation;
        TDF_Label label;
        std::string entityIdentifier;
        std::string definitionIdentifier;
        OcctGeometryRepresentation representation =
            OcctGeometryRepresentation::Invalid;
        OcctGeometryRepresentation destinationRepresentation =
            OcctGeometryRepresentation::Invalid;
        TopoDS_Shape storedShape;
        gp_Trsf transform;
        OcctReferenceAxisReadState referenceAxisState =
            OcctReferenceAxisReadState::Invalid;
        OcctReferenceAxis referenceAxis;
        Standard_Integer documentTime = 0;
        Standard_Size topologyNodeCount = 0;
        Standard_Real metersPerUnit = 0.0;
        Standard_Real worldMinimum[3] = {0.0, 0.0, 0.0};
        Standard_Real worldMaximum[3] = {0.0, 0.0, 0.0};
    };

    struct PendingResult {
        TDF_Label label;
        std::string entityIdentifier;
        std::string definitionIdentifier;
        OcctGeometryRepresentation representation =
            OcctGeometryRepresentation::Invalid;
        TopoDS_Shape expectedShape;
        gp_Trsf expectedTransform;
        OcctReferenceAxisReadState expectedReferenceAxisState =
            OcctReferenceAxisReadState::Invalid;
        OcctReferenceAxis expectedReferenceAxis;
    };

    enum class DocumentState : std::uint8_t {
        None = 0,
        AllCommitted,
        OpenCommand,
        PartialOrMismatched,
        Unavailable,
    };

    Standard_Boolean captureSelectedSource(
        SourceSnapshot& source) const noexcept;
    Standard_Boolean sourceIsCurrent(
        Standard_Boolean requireOriginalDocumentTime = Standard_True) const
        noexcept;
    Standard_Boolean sourceIsCurrent(
        const SourceSnapshot& source,
        Standard_Boolean requireOriginalDocumentTime) const noexcept;
    Standard_Boolean canAdmitCount(
        const SourceSnapshot& source,
        Standard_Integer count) const noexcept;
    Standard_Integer maximumAdmittedCount(
        const SourceSnapshot& source) const noexcept;
    std::pair<Standard_Real, Standard_Real> spacingRange(
        const SourceSnapshot& source,
        LinearArrayAxis axis,
        Standard_Integer count) const noexcept;
    Standard_Boolean rebuildPreview() noexcept;
    Standard_Boolean clearPreviewObjects() noexcept;
    Standard_Boolean abortOwnedCommand() noexcept;
    DocumentState inspectPendingResults() const noexcept;
    LinearArrayApplyResult finishCommittedOperation() noexcept;
    void clearInactiveState() noexcept;
    void notifyStateChanged() noexcept;

private:
    Handle(AIS_InteractiveContext) _context;
    Handle(OcctDocument) _document;
    std::optional<SourceSnapshot> _source;
    std::vector<Handle(AIS_Shape)> _previewObjects;
    std::vector<PendingResult> _pendingResults;
    LinearArrayAxis _axis = LinearArrayAxis::X;
    Standard_Integer _count = kDefaultCount;
    Standard_Integer _maximumAdmittedCount = kMaximumCount;
    Standard_Real _spacing = 0.0;
    Standard_Boolean _previewValid = Standard_False;
    Standard_Boolean _ownsDocumentCommand = Standard_False;
    LinearArrayPreviewState _state =
        LinearArrayPreviewState::Unavailable;
    std::uint64_t _generation = 0;
    std::function<void()> _previewStateChangedCallback;
#ifdef DEBUG
    Standard_Size _debugTransactionFailureCount = 0;
    Standard_Size _debugAbortFailureCount = 0;
    Standard_Size _debugEraseFailureCount = 0;
    Standard_Integer _debugCommitMode = 0;
    Standard_Size _debugPostCommitInspectFailureCount = 0;
    Standard_Size _debugMaximumTopologyNodes =
        kMaximumAggregateTopologyNodes;
#endif
};

} // namespace core3d

#endif // LinearArrayOperationController_hpp
