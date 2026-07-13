//
//  TransformInspectorMeasurementController.hpp
//  Core3D
//

#ifndef TransformInspectorMeasurementController_hpp
#define TransformInspectorMeasurementController_hpp

#include "AIS_InteractiveContext.hxx"
#include "OcctDocument.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <AIS_Shape.hxx>
#include <gp_Trsf.hxx>

namespace core3d {

class ObjectInteractor;
class ShapeInteractor;
class TransformInspectorBoundsWorker;
struct TransformInspectorBoundsWorkerResult;

//! Terminal and nonterminal states returned by the read-only inspector.
//! Measuring is the only state that may be followed by the optional one-shot
//! completion. A completion is always delivered on the main thread.
enum class TransformInspectorMeasurementState : std::uint8_t {
    NoSelection = 0,
    MultipleSelection,
    Busy,
    Unsupported,
    Invalid,
    Measuring,
    Ready,
    BoundsUnavailable,
    MeasurementFailed,
};

//! Normalized representation contract exposed to the inspector. Valid legacy
//! analytic definitions are reported as BRep; invalid schema values never
//! escape capture.
enum class TransformInspectorGeometryRepresentation : std::uint8_t {
    Invalid = 0,
    BRep,
    TriangleMesh,
};

struct TransformInspectorVector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct TransformInspectorQuaternion {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

//! Immutable-by-value result captured from authoritative OCAF state. Bounds
//! are definition-local and ignore the persisted object translation/rotation.
//! `dimensions` equals `localDimensions * abs(uniformScale)` in document
//! model units; `metersPerUnit` converts those units to metres.
struct TransformInspectorMeasurement {
    TransformInspectorMeasurementState state =
        TransformInspectorMeasurementState::Invalid;
    std::size_t selectionCount = 0;
    std::uint64_t generation = 0;
    bool canEditPosition = false;
    std::uint64_t positionEditGeneration = 0;
    std::uint64_t documentEditGeneration = 0;
    std::uint64_t geometryEditGeneration = 0;
    std::string entityIdentifier;
    std::string definitionIdentifier;
    std::string name;
    TransformInspectorGeometryRepresentation representation =
        TransformInspectorGeometryRepresentation::Invalid;
    TransformInspectorVector3 position;
    TransformInspectorVector3 extrinsicXYZDegrees;
    TransformInspectorQuaternion quaternion;
    double uniformScale = 1.0;
    double metersPerUnit = 0.001;
    bool presentationMatchesDocument = false;
    //! Stable Core3DModelCapability bit mask computed from the admitted
    //! authoritative representation so the Objective-C bridge performs no
    //! separate OCAF read or policy reconstruction.
    std::uint64_t modelCapabilities = 0;
    //! xmin, ymin, zmin, xmax, ymax, zmax.
    std::array<double, 6> localBounds{};
    TransformInspectorVector3 localDimensions;
    TransformInspectorVector3 dimensions;
};

enum class TransformInspectorPositionAxis : std::uint8_t {
    X = 0,
    Y,
    Z,
};

enum class TransformInspectorPositionCommitResult : std::uint8_t {
    Committed = 0,
    Unchanged,
    InvalidValue,
    Busy,
    Stale,
    Unsupported,
    Unavailable,
    InternalFailure,
};

//! Public DTO values copied back into the native compare-and-swap gate. The
//! actual document/TShape lease remains private to the controller.
struct TransformInspectorPositionCommitRequest {
    std::uint64_t positionEditGeneration = 0;
    std::uint64_t documentEditGeneration = 0;
    std::uint64_t geometryEditGeneration = 0;
    std::string entityIdentifier;
    std::string definitionIdentifier;
    TransformInspectorVector3 expectedPosition;
    double expectedMetersPerUnit = 0.0;
    TransformInspectorGeometryRepresentation expectedRepresentation =
        TransformInspectorGeometryRepresentation::Invalid;
    std::uint64_t expectedModelCapabilities = 0;
    TransformInspectorPositionAxis axis = TransformInspectorPositionAxis::X;
    double value = 0.0;
};

struct TransformInspectorPositionCommitOutcome {
    TransformInspectorPositionCommitResult result =
        TransformInspectorPositionCommitResult::InternalFailure;
    Handle(AIS_Shape) presentation;
    gp_Trsf committedTransform;
};

//! Release-safe, read-only counters used by signed-device qualification.
//! Runtime behavior never adapts to these observations; production admission
//! remains deterministic across devices and builds.
struct TransformInspectorMeasurementPerformanceState {
    Standard_Real lastMeshSweepMilliseconds = 0.0;
    Standard_Real lastBRepCopyMilliseconds = 0.0;
    std::uint64_t meshSweepRunCount = 0;
    std::uint64_t watchdogFireCount = 0;
    Standard_Size cacheEntryCount = 0;
    Standard_Size negativeCacheEntryCount = 0;
};

using TransformInspectorMeasurementCompletion =
    std::function<void(TransformInspectorMeasurement)>;

#ifdef DEBUG
struct TransformInspectorMeasurementDebugState {
    std::uint64_t generation = 0;
    std::uint64_t submittedCount = 0;
    std::uint64_t startedCount = 0;
    std::uint64_t completedCount = 0;
    std::uint64_t cancelledOrSupersededCount = 0;
    std::uint64_t pendingReplacementCount = 0;
    std::uint64_t acceptedCount = 0;
    std::uint64_t staleCount = 0;
    Standard_Boolean workerActive = Standard_False;
    Standard_Boolean workerPending = Standard_False;
    Standard_Boolean lastComputeWasMainThread = Standard_False;
    Standard_Boolean workerBlocked = Standard_False;
    Standard_Boolean forcedBoundsFailure = Standard_False;
    Standard_Real lastMeshSweepMilliseconds = 0.0;
    Standard_Real lastBRepCopyMilliseconds = 0.0;
    Standard_Size maximumBRepTopologyNodes = 8'192;
    Standard_Size maximumTriangleMeshSweepNodes = 262'144;
    Standard_Real meshSweepWatchdogDeadlineMilliseconds = 0.0;
    Standard_Size meshSweepWatchdogPollNodes = 32'768;
    std::uint64_t meshSweepRunCount = 0;
    std::uint64_t watchdogFireCount = 0;
    Standard_Size cacheEntryCount = 0;
    Standard_Size negativeCacheEntryCount = 0;
};
#endif

//! Main-thread selection capture plus a bounded hybrid local-bounds engine.
//! The controller owns no UI and never mutates the document or presentation.
class TransformInspectorMeasurementController final
    : public std::enable_shared_from_this<
          TransformInspectorMeasurementController> {
public:
    static constexpr Standard_Size kMaximumBRepTopologyNodes = 8'192;
    static constexpr Standard_Size kMaximumTriangleMeshFaces = 4'096;
    static constexpr Standard_Size kMaximumTriangleMeshSweepNodes = 262'144;
    static constexpr std::size_t kMaximumBoundsCacheEntries = 16;
    static constexpr std::size_t kMaximumNegativeBoundsCacheEntries = 8;
    static constexpr Standard_Real kMainThreadBoundsBudgetMilliseconds = 5.0;
    static constexpr Standard_Real kMeshSweepWatchdogDeadlineMilliseconds = 20.0;
    static constexpr Standard_Size kMeshSweepWatchdogPollNodes = 32'768;

    TransformInspectorMeasurementController() = delete;
    TransformInspectorMeasurementController(
        Handle(AIS_InteractiveContext) context,
        Handle(OcctDocument) document);
    ~TransformInspectorMeasurementController() noexcept;

    //! Capture 0/1/N selection directly from OCAF/AIS on the main thread.
    //! A BRep cache miss returns Measuring and may invoke `completion` once
    //! with Ready or MeasurementFailed. Deterministic topology rejection
    //! returns BoundsUnavailable immediately. New capture/cancel suppresses
    //! older completions. Triangle meshes and cache hits are terminal immediately.
    TransformInspectorMeasurement capture(
        const std::shared_ptr<ObjectInteractor>& objectInteractor,
        const std::shared_ptr<ShapeInteractor>& shapeInteractor,
        TransformInspectorMeasurementCompletion completion = {}) noexcept;

    //! Compare-and-swap one raw model-unit Position scalar. Persistence is
    //! completed here; Core3DViewer owns presentation publication and the one
    //! document notification after a Committed outcome.
    TransformInspectorPositionCommitOutcome commitPosition(
        const std::shared_ptr<ObjectInteractor>& objectInteractor,
        const std::shared_ptr<ShapeInteractor>& shapeInteractor,
        const TransformInspectorPositionCommitRequest& request) noexcept;

    //! Invalidate the current generation and suppress its completion. Exact
    //! AddOptimal work already inside OCCT may finish and populate the cache.
    void cancelPendingMeasurement() noexcept;
    //! Idempotent lifecycle endpoint used before releasing interactors/context.
    void shutdown() noexcept;

    //! Lightweight evidence for opt-in signed-device performance acceptance.
    //! Safe in Release and read-only; callers must invoke it on the main thread.
    TransformInspectorMeasurementPerformanceState performanceState()
        const noexcept;

#ifdef DEBUG
    TransformInspectorMeasurementDebugState debugState() const noexcept;
    void debugSetWorkerBlocked(Standard_Boolean blocked) noexcept;
    void debugSetForcedBoundsFailure(Standard_Boolean failure) noexcept;
    void debugSetMaximumBRepTopologyNodes(Standard_Size limit) noexcept;
    void debugSetMaximumTriangleMeshSweepNodes(Standard_Size limit) noexcept;
    void debugSetMeshSweepWatchdog(
        Standard_Real deadlineMilliseconds,
        Standard_Size pollNodes) noexcept;
    //! One-shot transaction reconciliation fault. 0 is normal, 1 reports
    //! false after a real close, 2 throws after a real close, and 3 leaves
    //! the staged command open so production reconciliation must abort it.
    void debugSetPositionCommitMode(Standard_Integer mode) noexcept;
#endif

private:
    struct Impl;

    void acceptWorkerResult(
        TransformInspectorBoundsWorkerResult result) noexcept;

    Handle(AIS_InteractiveContext) myContext;
    Handle(OcctDocument) myDoc;
    std::weak_ptr<ObjectInteractor> myObjectInteractor;
    std::weak_ptr<ShapeInteractor> myShapeInteractor;
    std::unique_ptr<Impl> myImpl;
    TransformInspectorMeasurementCompletion myCompletion;
    std::uint64_t myGeneration = 0;
    Standard_Boolean myStopped = Standard_False;

    friend class TransformInspectorBoundsWorker;
};

} // namespace core3d

#endif // TransformInspectorMeasurementController_hpp
