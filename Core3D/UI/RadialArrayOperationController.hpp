//
//  RadialArrayOperationController.hpp
//  Core3D
//

#ifndef RadialArrayOperationController_hpp
#define RadialArrayOperationController_hpp

#include "Core3DContext.hpp"
#include "OcctDocument.h"

#include <AIS_Shape.hxx>
#include <Graphic3d_NameOfMaterial.hxx>
#include <Quantity_Color.hxx>

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace core3d {

enum class RadialArrayApplyResult : std::uint8_t {
    NoChange = 0,
    Applied,
    AppliedNeedsDocumentRedraw,
};

enum class RadialArrayPreviewState : std::uint8_t {
    Unavailable = 0,
    Selecting,
    Ready,
    Committing,
    OutcomeUnknown,
    Failed,
};

enum class RadialArrayReferenceEditResult : std::uint8_t {
    NoChange = 0,
    Applied,
    OutcomeUnknown,
    RetryableFailure,
};

//! Complete renderer handoff for one lease-validated radial preview. The
//! source presentation is explicit because radial instances are expressed as
//! proper-rigid transforms relative to that exact committed source.
struct RadialArrayPreviewCapture {
    Handle(AIS_Shape) sourcePresentation;
    std::vector<Handle(AIS_Shape)> previewObjects;
};

#ifdef DEBUG
struct RadialArrayPreviewDebugState {
    RadialArrayPreviewState state = RadialArrayPreviewState::Unavailable;
    std::uint64_t generation = 0;
    std::uint64_t referenceAuthorityToken = 0;
    Standard_Size previewBodyCount = 0;
    Standard_Size pendingResultCount = 0;
    Standard_Boolean hasPendingReferenceEdit = Standard_False;
    Standard_Boolean activeOperation = Standard_False;
    Standard_Boolean previewValid = Standard_False;
    Standard_Boolean canApply = Standard_False;
    Standard_Boolean ownsDocumentCommand = Standard_False;
    Standard_Boolean documentCommandOpen = Standard_False;
    Standard_Integer count = 3;
    Standard_Real sweepDegrees = 360.0;
};
#endif

//! Owns one touch-first radial array operation. Preview presentations share
//! only immutable source topology; persisted results are deep, independent
//! geometry copies. Every OCAF mutation is protected by a per-operation owner
//! sentinel so recovery can never abort another tool's command.
class RadialArrayOperationController {
public:
    static constexpr Standard_Integer kMinimumCount = 2;
    static constexpr Standard_Integer kMaximumCount = 16;
    static constexpr Standard_Integer kDefaultCount = 3;
    static constexpr Standard_Real kMinimumSweepDegrees = -360.0;
    static constexpr Standard_Real kMaximumSweepDegrees = 360.0;
    static constexpr Standard_Real kDefaultSweepDegrees = 360.0;
    static constexpr std::size_t kMaximumPreviewBodies = 15;
    static constexpr Standard_Size kMaximumSourceTopologyNodes = 1'024;
    static constexpr Standard_Size kMaximumAggregateTopologyNodes = 32'768;
    static constexpr Standard_Real kMaximumPivotRadius = 1'000'000.0;

    RadialArrayOperationController() = delete;
    RadialArrayOperationController(
        Handle(AIS_InteractiveContext) context,
        Handle(OcctDocument) document);
    ~RadialArrayOperationController() noexcept;

    Standard_Boolean begin() noexcept;
    RadialArrayApplyResult apply() noexcept;
    Standard_Boolean cancel() noexcept;

    Standard_Boolean setCount(Standard_Integer count) noexcept;
    Standard_Boolean setSweepDegrees(Standard_Real sweepDegrees) noexcept;
    Standard_Integer count() const noexcept;
    Standard_Real sweepDegrees() const noexcept;
    Standard_Real metersPerUnit() const noexcept;
    std::pair<Standard_Integer, Standard_Integer> countRange() const noexcept;
    std::pair<Standard_Real, Standard_Real> sweepDegreesRange() const noexcept;

    OcctReferenceAxisReadState referenceAxis(
        OcctReferenceAxis& axis) const noexcept;
    std::uint64_t referenceAuthorityToken() const noexcept;
    Standard_Boolean convertReferenceAxisSpaces(
        OcctReferenceSpace pivotSpace,
        OcctReferenceSpace directionSpace,
        std::uint64_t expectedAuthorityToken,
        OcctReferenceAxis& axis) const noexcept;
    RadialArrayReferenceEditResult setReferenceAxis(
        const OcctReferenceAxis& axis,
        std::uint64_t expectedAuthorityToken) noexcept;
    RadialArrayReferenceEditResult resetReferenceAxis(
        std::uint64_t expectedAuthorityToken) noexcept;

    Standard_Boolean canApply() const noexcept;
    Standard_Boolean hasActiveOperation() const noexcept;
    Standard_Boolean hasUnresolvedState() const noexcept;
    RadialArrayPreviewState previewState() const noexcept;
    std::uint64_t previewGeneration() const noexcept;
    Standard_Boolean capturePreview(
        RadialArrayPreviewCapture& capture) const noexcept;
    Standard_Boolean canPublishEmptyPreview() const noexcept;
    void setPreviewStateChangedCallback(std::function<void()> callback);

#ifdef DEBUG
    RadialArrayPreviewDebugState debugPreviewState() const noexcept;
    void debugSetBeginOwnedCommandMismatchCount(
        Standard_Size count) noexcept;
    void debugSetTransactionFailureCount(Standard_Size count) noexcept;
    void debugSetAbortFailureCount(Standard_Size count) noexcept;
    void debugSetEraseFailureCount(Standard_Size count) noexcept;
    //! 0 normal, 1 false-after-close, 2 throw-after-close.
    void debugSetApplyCommitMode(Standard_Integer mode) noexcept;
    //! 0 normal, 1 one-shot unavailable, 2 one-shot partial/mismatched.
    void debugSetPostCommitInspectMode(Standard_Integer mode) noexcept;
    void debugSetMaximumTopologyNodes(Standard_Size limit) noexcept;
    Standard_Boolean debugMutateSourcePersistedTransform() noexcept;
    //! 0 normal, 1 false-after-close, 2 throw-after-close.
    void debugSetReferenceEditCommitMode(Standard_Integer mode) noexcept;
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
        Standard_Boolean hasStrictOverlayStyle = Standard_False;
        Graphic3d_NameOfMaterial materialName =
            Graphic3d_NameOfMaterial_ShinyPlastified;
        Quantity_Color color;
        Standard_Real transparency = 0.0;
        OcctReferenceAxisReadState referenceAxisState =
            OcctReferenceAxisReadState::Invalid;
        OcctReferenceAxis referenceAxis;
        gp_Ax1 worldAxis;
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

    struct PendingReferenceEdit {
        OcctReferenceAxisReadState previousState =
            OcctReferenceAxisReadState::Invalid;
        OcctReferenceAxis previousAxis;
        OcctReferenceAxisReadState expectedState =
            OcctReferenceAxisReadState::Invalid;
        OcctReferenceAxis expectedAxis;
    };

    enum class DocumentState : std::uint8_t {
        None = 0,
        AllCommitted,
        OpenCommand,
        PartialOrMismatched,
        Unavailable,
    };

    enum class ReferenceDocumentState : std::uint8_t {
        None = 0,
        Expected,
        Previous,
        OpenCommand,
        Mismatched,
        Unavailable,
    };

    Standard_Boolean captureSelectedSource(SourceSnapshot& source) const
        noexcept;
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
    Standard_Boolean rebuildPreview() noexcept;
    Standard_Boolean clearPreviewObjects() noexcept;
    Standard_Boolean refreshSourceAfterReferenceEdit() noexcept;
    Standard_Boolean axisAdmitsCurrentPreview(
        const gp_Ax1& worldAxis) const noexcept;

    Standard_Boolean beginOwnedCommand(
        const Handle(TDocStd_Document)& document) noexcept;
    Standard_Boolean ownedCommandIsCurrent(
        const Handle(TDocStd_Document)& document) const noexcept;
    Standard_Boolean abortOwnedCommand() noexcept;
    void clearCommandOwnership() noexcept;
    DocumentState inspectPendingResults() noexcept;
    ReferenceDocumentState inspectPendingReferenceEdit() noexcept;
    RadialArrayReferenceEditResult reconcileReferenceEdit() noexcept;
    RadialArrayReferenceEditResult editReferenceAxis(
        const std::optional<OcctReferenceAxis>& axis,
        std::uint64_t expectedAuthorityToken) noexcept;

    RadialArrayApplyResult finishCommittedOperation() noexcept;
    void clearInactiveState() noexcept;
    Standard_Boolean advanceReferenceAuthorityToken() noexcept;
    void notifyStateChanged() noexcept;

private:
    Handle(AIS_InteractiveContext) _context;
    Handle(OcctDocument) _document;
    std::optional<SourceSnapshot> _source;
    std::vector<Handle(AIS_Shape)> _previewObjects;
    std::vector<PendingResult> _pendingResults;
    std::optional<PendingReferenceEdit> _pendingReferenceEdit;
    Standard_Integer _count = kDefaultCount;
    Standard_Integer _maximumAdmittedCount = kMaximumCount;
    Standard_Real _sweepDegrees = kDefaultSweepDegrees;
    Standard_Boolean _previewValid = Standard_False;
    Standard_Boolean _ownsDocumentCommand = Standard_False;
    Standard_Integer _ownedTransaction = -1;
    Standard_Integer _ownedDocumentTime = -1;
    Standard_Integer _ownedMarkerValue = 0;
    RadialArrayPreviewState _state =
        RadialArrayPreviewState::Unavailable;
    std::uint64_t _generation = 0;
    std::uint64_t _referenceAuthorityToken = 0;
    std::function<void()> _previewStateChangedCallback;
#ifdef DEBUG
    Standard_Size _debugBeginOwnedCommandMismatchCount = 0;
    Standard_Size _debugTransactionFailureCount = 0;
    Standard_Size _debugAbortFailureCount = 0;
    Standard_Size _debugEraseFailureCount = 0;
    Standard_Integer _debugApplyCommitMode = 0;
    Standard_Integer _debugPostCommitInspectMode = 0;
    Standard_Size _debugMaximumTopologyNodes =
        kMaximumAggregateTopologyNodes;
    Standard_Integer _debugReferenceEditCommitMode = 0;
#endif
};

} // namespace core3d

#endif // RadialArrayOperationController_hpp
