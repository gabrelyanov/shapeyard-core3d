//
//  Core3DViewController.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 28.03.2024.
//

#import <UIKit/UIKit.h>
#import <Core3D/PrimitiveType.h>
#import <Core3D/PrimitiveSelectionType.h>
#import <Core3D/PrimitiveGizmoType.h>
#import <Core3D/UIStateChanging.h>
#import <Core3D/ExportType.h>
#import <Core3D/Core3DNativeExportOperation.h>
#import <Core3D/OrthoProjectionType.h>
#import <Core3D/Core3DMaterialController.h>
#import <Core3D/Core3DModelCapability.h>
#import <Core3D/Core3DTransformInspectorSnapshot.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DSceneSnapshot;
@class Core3DSceneFrameSnapshot;
@class Core3DScenePresentationOverlaySnapshot;

typedef NS_ENUM(NSInteger, Core3DAssetLoadResult) {
    Core3DAssetLoadResultSuccess = 0,
    Core3DAssetLoadResultInvalidData,
    Core3DAssetLoadResultTemporaryFileFailure,
    Core3DAssetLoadResultBusy,
    Core3DAssetLoadResultUnsupportedVersion,
    Core3DAssetLoadResultInternalFailure,
};

typedef NS_ENUM(NSInteger, Core3DViewportRenderingAPI) {
    Core3DViewportRenderingAPIUnavailable = 0,
    Core3DViewportRenderingAPIOpenGLES2 = 2,
    Core3DViewportRenderingAPIOpenGLES3 = 3,
};

#ifdef DEBUG
//! Bounded standalone BinXCAF documents used to exercise the persistent
//! definition-owned geometry representation contract through production load.
typedef NS_ENUM(NSInteger, Core3DDebugGeometryFixtureMode) {
    Core3DDebugGeometryFixtureEmpty = 0,
    Core3DDebugGeometryFixtureLegacyUnmarkedBRep,
    Core3DDebugGeometryFixtureMarkedBRep,
    Core3DDebugGeometryFixtureMarkedTriangleMesh,
    Core3DDebugGeometryFixtureValidBRepAndTriangleMeshRoots,
    Core3DDebugGeometryFixtureUnmarkedTriangleMesh,
    Core3DDebugGeometryFixtureUnmarkedMixedDefinition,
    Core3DDebugGeometryFixtureBRepMarkerOnTriangleMesh,
    Core3DDebugGeometryFixtureTriangleMeshMarkerOnBRep,
    Core3DDebugGeometryFixtureBRepMarkerOnMixedDefinition,
    Core3DDebugGeometryFixtureTriangleMeshMarkerOnMixedDefinition,
    Core3DDebugGeometryFixtureUnknownMarkerOnBRep,
    Core3DDebugGeometryFixtureOrphanMarker,
    //! Valid admitted mesh with one finite node not referenced by its triangle.
    //! Used to lock the inspector's full stored-vertex-table bounds contract.
    Core3DDebugGeometryFixtureMarkedTriangleMeshWithUnreferencedOutlier,
    //! Valid admitted BRep with a persisted definition-owned uniform scale.
    //! The fixture locks scale separation from authoritative local bounds.
    Core3DDebugGeometryFixtureMarkedBRepWithUniformScale,
    //! Marked BRep with a finite, nonzero persisted scale below the inspector's
    //! validity tolerance. Production loading can construct it safely, while
    //! the checked inspector must report Invalid.
    Core3DDebugGeometryFixtureMarkedBRepWithCorruptTransform,
    //! Valid admitted mesh whose authoritative face triangulation owns a
    //! nonidentity TopLoc_Location that must be applied to every stored node.
    Core3DDebugGeometryFixtureMarkedTriangleMeshWithLocatedTriangulation,
};
#endif

typedef struct {
    double min;
    double max;
} Boundaries;

//! Stable world-axis contract for a destructive Linear Array operation.
typedef NS_ENUM(NSInteger, Core3DLinearArrayAxis) {
    Core3DLinearArrayAxisX = 0,
    Core3DLinearArrayAxisY,
    Core3DLinearArrayAxisZ,
};

//! Native-authoritative parameters and bounds for one active Linear Array.
//! Count is the total number of objects, including the unchanged source.
typedef struct {
    Core3DLinearArrayAxis axis;
    NSInteger count;
    NSInteger minimumCount;
    NSInteger maximumCount;
    CGFloat spacing;
    CGFloat minimumSpacing;
    CGFloat maximumSpacing;
    //! Physical document scale used to present spacing in millimetres.
    double metersPerUnit;
} Core3DLinearArrayParameters;

//! Native-authoritative model-unit thickness for one active Shell preview.
//! Swift converts these values through metersPerUnit for millimetre display.
typedef struct {
    CGFloat thickness;
    CGFloat defaultThickness;
    CGFloat minimumThickness;
    CGFloat maximumThickness;
    double metersPerUnit;
} Core3DShellParameters;

//! Truthful outcome for an explicit modeling Apply or Cancel request. An
//! unknown outcome remains retained but must be reconciled with another Apply;
//! a retryable failure retains the ordinary Apply/Cancel recovery controls.
typedef NS_ENUM(NSInteger, Core3DModelingOperationResult) {
    Core3DModelingOperationResultSucceeded = 0,
    Core3DModelingOperationResultNoActiveOperation,
    Core3DModelingOperationResultNotReady,
    Core3DModelingOperationResultOutcomeUnknown,
    Core3DModelingOperationResultRetryableFailure,
    Core3DModelingOperationResultFailed,
};

//! Renderer-independent phase of one modeling preview. Committing is exposed
//! as Computing because neither phase is user-applicable.
typedef NS_ENUM(NSInteger, Core3DModelingPreviewState) {
    Core3DModelingPreviewStateUnavailable = 0,
    Core3DModelingPreviewStateSelecting,
    Core3DModelingPreviewStateComputing,
    Core3DModelingPreviewStateReady,
    Core3DModelingPreviewStateOutcomeUnknown,
    Core3DModelingPreviewStateFailed,
};

//! Release-safe, immutable observation of one modeling operation. Generation
//! is controller-owned and suppresses stale asynchronous preview results.
typedef struct {
    PrimitiveGizmoType operation;
    Core3DModelingPreviewState state;
    uint64_t generation;
    BOOL active;
    BOOL canApply;
} Core3DModelingPreviewStatus;

@protocol PrimitiveManagerProtocol<NSObject>

- (void)addPrimitive:(PrimitiveType)primitiveType;
- (void)addPrimitivesFromJSON:(NSString *)json;
- (void)deleteSelected;
- (void)selectAll;
- (void)duplicateSelected;
- (void)setChamfer:(CGFloat)value;
- (Boundaries)getChamferBoundaries;
- (Core3DModelingOperationResult)tryApplyChamfer
    NS_SWIFT_NAME(tryApplyChamfer());
- (Core3DModelingOperationResult)tryCancelChamfer
    NS_SWIFT_NAME(tryCancelChamfer());
- (void)applyChamfer;
- (BOOL)cancelChamfer;
- (void)setExtrusion:(CGFloat)value;
- (Boundaries)getExtrusionBoundaries;
- (Core3DModelingOperationResult)tryApplyExtrusion
    NS_SWIFT_NAME(tryApplyExtrusion());
- (Core3DModelingOperationResult)tryCancelExtrusion
    NS_SWIFT_NAME(tryCancelExtrusion());
- (BOOL)applyExtrusion;
- (BOOL)cancelExtrusion;
- (Core3DShellParameters)getShellParameters;
- (BOOL)setShellThickness:(CGFloat)thickness;
- (Core3DModelingOperationResult)tryApplyShell
    NS_SWIFT_NAME(tryApplyShell());
- (Core3DModelingOperationResult)tryCancelShell
    NS_SWIFT_NAME(tryCancelShell());
- (BOOL)applyShell;
- (BOOL)cancelShell;
- (Core3DModelingOperationResult)tryApplySubtract
    NS_SWIFT_NAME(tryApplySubtract());
- (Core3DModelingOperationResult)tryCancelSubtract
    NS_SWIFT_NAME(tryCancelSubtract());
- (Core3DModelingOperationResult)tryApplyMirror
    NS_SWIFT_NAME(tryApplyMirror());
- (Core3DModelingOperationResult)tryCancelMirror
    NS_SWIFT_NAME(tryCancelMirror());
- (void)applyMirror;
- (void)cancelMirror;
- (BOOL)beginMirrorPlanePicking;
- (BOOL)cancelMirrorPlanePicking;
- (BOOL)isPickingMirrorPlane;
- (BOOL)hasCustomMirrorPlane;
- (BOOL)setMirrorPlaneOffset:(CGFloat)offset;
- (Boundaries)getMirrorPlaneOffsetBoundaries;
- (BOOL)resetMirrorPlane;
- (Core3DLinearArrayParameters)getLinearArrayParameters;
- (BOOL)setLinearArrayAxis:(Core3DLinearArrayAxis)axis;
- (BOOL)setLinearArrayCount:(NSInteger)count;
- (BOOL)setLinearArraySpacing:(CGFloat)spacing;
- (Core3DModelingOperationResult)tryApplyLinearArray
    NS_SWIFT_NAME(tryApplyLinearArray());
- (Core3DModelingOperationResult)tryCancelLinearArray
    NS_SWIFT_NAME(tryCancelLinearArray());
- (void)applySubtract;
- (void)cancelSubtract;
- (Core3DModelingOperationResult)tryApplyUnion
    NS_SWIFT_NAME(tryApplyUnion());
- (Core3DModelingOperationResult)tryCancelUnion
    NS_SWIFT_NAME(tryCancelUnion());
- (void)applyUnion;
- (void)cancelUnion;
- (Core3DModelingOperationResult)tryApplyIntersect
    NS_SWIFT_NAME(tryApplyIntersect());
- (Core3DModelingOperationResult)tryCancelIntersect
    NS_SWIFT_NAME(tryCancelIntersect());
- (void)applyIntersect;
- (void)cancelIntersect;
- (Core3DModelingPreviewStatus)modelingPreviewStatusForGizmoType:
    (PrimitiveGizmoType)gizmoType
    NS_SWIFT_NAME(modelingPreviewStatus(for:));
- (void)undo;
- (void)redo;
- (NSString *_Nullable)getCoreInfoText;
- (void)setOrthoProjection:(OrthoProjectionType)orthoType;
- (void)setSnappingTranslation:(double)value;
- (double)getSnappingTranslation;
- (void)setSnappingRotation:(double)value;
- (double)getSnappingRotation;
- (void)setSnappingScale:(double)value;
- (double)getSnappingScale;

#ifdef DEBUG
- (void)handleDebug_A;
- (void)handleDebug_B;
#endif

@end

@protocol AvailabilityManagerProtocol<NSObject>

- (BOOL)canAdd;
- (BOOL)canDelete;
- (BOOL)canDuplicate;
- (BOOL)canUndo;
- (BOOL)canRedo;
- (BOOL)canApply;
- (BOOL)canApplyMaterial;

- (Core3DModelCapability)selectedModelCapabilities;
- (NSArray<NSNumber *> *_Nonnull)availableGizmoTypes;
- (BOOL)canExportType:(ExportType)exportType
    NS_SWIFT_NAME(canExport(type:));
- (NSArray<NSNumber *> *_Nonnull)availableExportTypes;

@end

@protocol ExportManagerProtocol<NSObject>

- (NSURL *_Nullable)exportWithType:(ExportType)exportType;
//! Freeze the committed document into a private handoff and return a worker
//! operation. OBJ, binary STL, and exact BRep STEP keep only the bounded XBF
//! save on the main thread; translation/writing, validation, and cleanup run
//! off-main.
- (Core3DNativeExportOperation *_Nullable)
    prepareNativeExportOperationWithType:(ExportType)exportType
    NS_SWIFT_NAME(prepareNativeExportOperation(with:));
//! Capture only committed exportable geometry. Unlike the presentation
//! snapshot seam, this returns nil while a Boolean, Mirror, Linear Array, or
//! Shell trial is active or unresolved, or while the OCAF document owns an open
//! command. Main-thread only; the returned value is an immutable deep copy
//! safe for background I/O.
- (Core3DSceneSnapshot *_Nullable)captureExportSceneSnapshot;

@end


@protocol Core3DViewControllerProtocol<NSObject>

@optional
- (void)viewDidLoadFromBundle;
- (void)viewDidFailToLoadFromBundle:(Core3DAssetLoadResult)result
    NS_SWIFT_NAME(viewDidFailToLoadFromBundle(_:));

@end


@interface Core3DViewController : UIViewController {
    UIStateChanging _currentStateChanging;
    PrimitiveSelectionType _currentSelectionType;
    PrimitiveGizmoType _currentGizmoType;
    NSArray<NSNumber *> *_availableGizmoTypes;
    BOOL _isViewerSetup;
    BOOL _isPreviewMode;
    NSString *_coreInfoText;
}

@property (nonatomic, strong, readonly) Core3DMaterialController* materialController;

@property (nonatomic, strong, nonnull) id glController NS_REFINED_FOR_SWIFT;
@property (nonatomic, assign, readonly) PrimitiveSelectionType currentSelectionType;
@property (nonatomic, assign, readonly) PrimitiveGizmoType currentGizmoType;
@property (nonatomic, readonly) UIView* glView;

@property (nonatomic, assign) BOOL can_add NS_REFINED_FOR_SWIFT;
@property (nonatomic, assign) BOOL can_delete NS_REFINED_FOR_SWIFT;
@property (nonatomic, assign) BOOL can_duplicate NS_REFINED_FOR_SWIFT;
@property (nonatomic, assign) BOOL can_undo NS_REFINED_FOR_SWIFT;
@property (nonatomic, assign) BOOL can_redo NS_REFINED_FOR_SWIFT;
@property (nonatomic, assign) BOOL can_apply NS_REFINED_FOR_SWIFT;
@property (nonatomic, assign) BOOL can_apply_material NS_REFINED_FOR_SWIFT;

@property (nonatomic, assign, readonly) BOOL isEmptyOfDisplayedObjects;
@property (nonatomic, assign, readonly) NSInteger numberOfDisplayedShapes;
@property (nonatomic, assign, readonly) NSUInteger viewportRenderedFrameCount;
@property (nonatomic, assign, readonly) CGSize viewportDrawableSize;
@property (nonatomic, assign, readonly, getter=isViewportRenderLoopRunning) BOOL viewportRenderLoopRunning;
@property (nonatomic, assign, readonly) Core3DViewportRenderingAPI viewportRenderingAPI;
//! Intersection across every selected editable definition. No selection,
//! unsafe selection, or invalid representation returns None.
@property (nonatomic, assign, readonly)
    Core3DModelCapability selectedModelCapabilities;

@property (nonatomic, weak, nullable) id<Core3DViewControllerProtocol> delegate;

- (void)setPreviewMode;

- (void)assetData:(void(^)(NSData *_Nullable))completion;
- (NSData *_Nullable)thumbData;

- (void)loadFromBundle:(NSURL *)bundleUrl;
//! Opens a manifest-authorized model without materializing it as NSData. Core3D
//! pins, streams, hashes, and privately stages the file before OCCT sees it.
- (void)loadFromAssetFile:(NSURL *)assetFileURL
        expectedByteCount:(unsigned long long)expectedByteCount
            expectedSHA256:(NSString *)expectedSHA256
    NS_SWIFT_NAME(load(fromAssetFile:expectedByteCount:expectedSHA256:));
- (void)saveSnapshot;

//! Deep-copy the committed model and semantic camera into renderer-neutral,
//! immutable values. Returns nil while a model transaction is open, while a
//! committed Shell result remains indeterminate, or before the native viewer
//! has finished setup. Main-thread only.
- (Core3DSceneSnapshot *_Nullable)captureSceneSnapshot;

//! Capture the current semantic camera and revision vector without traversing
//! or copying scene geometry. Returns nil until a full snapshot has established
//! the active document. Main-thread only.
- (Core3DSceneFrameSnapshot *_Nullable)captureSceneFrameSnapshot;

//! Capture the idle move/rotate gizmo paired with the last full scene. A
//! nonnull snapshot with empty arrays explicitly clears the overlay; nil means
//! the current interaction or tool is unsafe for alternate presentation.
- (Core3DScenePresentationOverlaySnapshot *_Nullable)
    captureScenePresentationOverlay;

//! Capture authoritative single-selection transform values. Main-thread only.
//! A BRep cache miss returns Measuring immediately and retains `completion`
//! for one terminal main-thread callback. Every other immediate state is
//! terminal and never invokes the optional block. A newer request or explicit
//! cancellation suppresses an older callback.
- (Core3DTransformInspectorSnapshot *)
    requestTransformInspectorSnapshotWithCompletion:
        (Core3DTransformInspectorCompletion _Nullable)completion
    NS_SWIFT_NAME(requestTransformInspectorSnapshot(completion:));

//! Invalidate the current inspector request and suppress its callback.
//! Already-running exact bounds work may finish privately and seed the cache.
- (void)cancelTransformInspectorSnapshotRequest
    NS_SWIFT_NAME(cancelTransformInspectorSnapshotRequest());

//! Atomically commit one Position component from an immutable inspector
//! snapshot. `value` is in raw document model units. The expected snapshot is
//! a compare-and-swap lease: Core3D independently rechecks document/geometry
//! generations, stable identity, selection, transform, representation, unit,
//! presentation, and all transient-operation guards before opening history.
//! Main-thread only.
- (Core3DTransformInspectorPositionCommitResult)
    commitTransformInspectorPositionValue:(double)value
                                      axis:(Core3DTransformInspectorAxis)axis
                          expectedSnapshot:
                              (Core3DTransformInspectorSnapshot *)snapshot
    NS_SWIFT_NAME(commitTransformInspectorPosition(_:axis:expected:));

//! Release-safe counters for opt-in signed-device performance qualification.
//! Values are observational only and never alter inspector admission policy.
- (NSDictionary<NSString *, NSNumber *> *)
    transformInspectorPerformanceState;

- (void)setSelectionType:(PrimitiveSelectionType)type;
- (void)setGizmoType:(PrimitiveGizmoType)type;

#ifdef DEBUG
//! Bounded counters and timings from the production inspector measurement
//! controller. Values are intended for deterministic XCTest evidence only.
- (NSDictionary<NSString *, NSNumber *> *)
    debugTransformInspectorMeasurementState;
//! Pause or release the inspector's serial bounds worker.
- (void)debugSetTransformInspectorWorkerBlocked:(BOOL)blocked;
//! Force admitted BRep bounds work to return MeasurementFailed.
- (void)debugSetTransformInspectorForcedMeasurementFailure:(BOOL)failure;
//! Lower the production BRep topology-copy admission cap for fail-closed tests.
- (void)debugSetMaximumTransformInspectorBRepTopologyNodes:(NSUInteger)limit;
//! Lower the production TriangleMesh node admission cap for deterministic tests.
- (void)debugSetMaximumTransformInspectorTriangleMeshSweepNodes:
    (NSUInteger)limit;
//! Override the normally-disabled DEBUG mesh watchdog for deterministic tests.
- (void)debugSetTransformInspectorMeshSweepWatchdogDeadlineMilliseconds:
    (double)deadlineMilliseconds
    pollNodes:(NSUInteger)pollNodes;
//! One-shot commit reconciliation mode: 0 normal, 1 false after close,
//! 2 throw after close, and 3 staged write left open for production abort.
- (void)debugSetTransformInspectorPositionCommitMode:(NSInteger)mode;
//! One-shot publication mode: 0 normal, 1 incremental failure followed by a
//! successful redraw, and 2 incremental plus redraw traversal failure.
- (void)debugSetTransformInspectorPositionPublicationFallbackMode:
    (NSInteger)mode;

//! Actual native enum values backing the persistent representation schema.
+ (NSDictionary<NSString *, NSNumber *> *)
    debugGeometryRepresentationSchemaValues;
//! Create one deterministic XBF representation fixture. Malformed modes are
//! intentionally writable but must be rejected by the production load gate.
- (NSData *_Nullable)debugGeometryRepresentationFixtureDataWithMode:
    (Core3DDebugGeometryFixtureMode)mode
    NS_SWIFT_NAME(debugGeometryRepresentationFixtureData(mode:));
//! One read-only state per active XCAF definition. `storedRawValue` is omitted
//! when the valid legacy definition has no marker.
- (NSArray<NSDictionary<NSString *, NSNumber *> *> *)
    debugGeometryRepresentationStates;
//! Test-only direct mirror-plane seam. This bypasses pointer hit testing while
//! preserving the authoritative mirror lifecycle and renderer invalidation.
- (BOOL)debugTryMirrorAxis:(NSInteger)axis
                  backward:(BOOL)backward
    NS_SWIFT_NAME(debugTryMirror(axis:backward:));
//! Mirror state values are Unavailable=0, Selecting=1, Ready=2,
//! Committing=3, OutcomeUnknown=4, and Failed=5.
- (NSDictionary<NSString *, NSNumber *> *)debugMirrorState;
//! Force the next N Mirror applies to fail after opening the OCAF command.
- (void)debugSetMirrorTransactionFailureCount:(NSUInteger)count;
//! Make the next N owned Mirror command aborts fail before touching OCAF.
- (void)debugSetMirrorAbortFailureCount:(NSUInteger)count;
//! Make the next N transient Mirror erases retain their owned presentation.
- (void)debugSetMirrorEraseFailureCount:(NSUInteger)count;
- (void)debugSetMirrorReferenceEraseFailureCount:(NSUInteger)count;
//! Inject CommitCommand behavior after a real close: 0 normal,
//! 1 false-after-close, and 2 throw-after-close.
- (void)debugSetMirrorCommitMode:(NSInteger)mode;
//! Make the next N post-commit label inspections report unavailable.
- (void)debugSetMirrorPostCommitInspectFailureCount:(NSUInteger)count;
//! Lower Mirror's aggregate source-and-result topology ceiling.
- (void)debugSetMaximumMirrorTopologyNodes:(NSUInteger)limit;
//! Lower only custom reference face-picking admission budgets.
- (void)debugSetMaximumMirrorReferenceTopologyNodes:(NSUInteger)limit;
- (void)debugSetMaximumMirrorReferenceFaces:(NSUInteger)limit;
//! Commit a persisted transform change behind the active Mirror snapshot while
//! leaving its presentation untouched, proving Apply rejects stale sources.
- (BOOL)debugMutateFirstMirrorSourcePersistedTransform;
- (BOOL)debugTryMirrorPlaneWithEntityIdentifier:(NSString *)entityIdentifier
                              faceTopologyIndex:(NSInteger)faceTopologyIndex
                                         offset:(CGFloat)offset
    NS_SWIFT_NAME(debugTryMirrorPlane(entityIdentifier:faceTopologyIndex:offset:));
//! Linear Array state values mirror the public modeling preview state. Count
//! is total occurrences including the unchanged source.
- (NSDictionary<NSString *, NSNumber *> *)debugLinearArrayState;
- (void)debugSetLinearArrayTransactionFailureCount:(NSUInteger)count;
- (void)debugSetLinearArrayAbortFailureCount:(NSUInteger)count;
- (void)debugSetLinearArrayEraseFailureCount:(NSUInteger)count;
- (void)debugSetLinearArrayCommitMode:(NSInteger)mode;
- (void)debugSetLinearArrayPostCommitInspectFailureCount:(NSUInteger)count;
- (void)debugSetMaximumLinearArrayTopologyNodes:(NSUInteger)limit;
- (BOOL)debugMutateFirstLinearArraySourcePersistedTransform;
//! Test-only deterministic Boolean seam. Identifiers must name committed
//! one-occurrence bodies; production selection, ownership, validation, preview,
//! transaction, and renderer publication paths remain authoritative.
- (BOOL)debugBeginBooleanWithGizmoType:(PrimitiveGizmoType)gizmoType
                actorEntityIdentifiers:(NSArray<NSString *> *)actorEntityIdentifiers
              subjectEntityIdentifiers:(NSArray<NSString *> *)subjectEntityIdentifiers
    NS_SWIFT_NAME(debugBeginBoolean(gizmoType:actorEntityIdentifiers:subjectEntityIdentifiers:));
//! Test-only recompute seam for the active Boolean trial. This exercises the
//! same replacement/ownership path used after an interactive adjustment.
- (BOOL)debugRecomputeBooleanPreview;
//! Real asynchronous Boolean worker/state counters. State values are
//! Selecting=0, Computing=1, Ready=2, Committing=3, Failed=4.
- (NSDictionary<NSString *, NSNumber *> *)debugBooleanPreviewState;
//! Pure-volume result admission fixture. Every returned value must be true:
//! solids and solid-only compounds are accepted; a loose face is rejected.
+ (NSDictionary<NSString *, NSNumber *> *)
    debugBooleanResultDimensionValidation;
//! Pause/release the process-global serial worker before BOP execution.
- (void)debugSetBooleanPreviewWorkerBlocked:(BOOL)blocked;
//! Lower production admission limits for deterministic fail-closed tests.
- (void)debugSetMaximumBooleanCaptureTopologyNodes:(NSUInteger)limit;
- (void)debugSetMaximumBooleanResultTopologyNodes:(NSUInteger)limit;
- (void)debugSetMaximumBooleanResultSolids:(NSUInteger)limit;
//! Force the next N Ready Boolean applies through transaction rollback.
- (void)debugSetBooleanTransactionFailureCount:(NSUInteger)count;
//! Make the next N Boolean transaction abort attempts fail before touching it.
- (void)debugSetBooleanAbortFailureCount:(NSUInteger)count;
//! Deliver the viewport's production memory-warning cancellation path.
- (void)debugSimulateBooleanMemoryWarning;
//! Deliver the same production path for an active Linear Array regression.
- (void)debugSimulateLinearArrayMemoryWarning;
//! Deliver the same production path for an active Shell regression.
- (void)debugSimulateShellMemoryWarning;
//! Deterministically capture a planar face by stable entity identifier and
//! zero-based TopExp face index, using the production extrusion admission.
- (BOOL)debugBeginExtrusionWithEntityIdentifier:(NSString *)entityIdentifier
                              faceTopologyIndex:(NSUInteger)faceTopologyIndex
    NS_SWIFT_NAME(debugBeginExtrusion(entityIdentifier:faceTopologyIndex:));
//! Bounded native extrusion preview/result counters and command state.
- (NSDictionary<NSString *, NSNumber *> *)debugExtrusionState;
//! Deterministically capture committed BRep edges by stable entity identifier
//! and zero-based TopExp edge indices. The production Bevel controller remains
//! authoritative for admission, asynchronous preview, and transaction apply.
- (BOOL)debugBeginBevelWithEntityIdentifier:(NSString *)entityIdentifier
                       edgeTopologyIndices:(NSArray<NSNumber *> *)edgeTopologyIndices
    NS_SWIFT_NAME(debugBeginBevel(entityIdentifier:edgeTopologyIndices:));
//! Multi-body variant used to prove aggregate Bevel result budgets. Every
//! entity has one nonempty array of zero-based TopExp edge indices.
- (BOOL)debugBeginBevelWithEntityIdentifiers:(NSArray<NSString *> *)entityIdentifiers
                 edgeTopologyIndicesByEntity:(NSArray<NSArray<NSNumber *> *> *)edgeTopologyIndicesByEntity
    NS_SWIFT_NAME(debugBeginBevel(entityIdentifiers:edgeTopologyIndicesByEntity:));
- (NSDictionary<NSString *, NSNumber *> *)debugBevelState;
- (void)debugSetBevelPreviewWorkerBlocked:(BOOL)blocked;
- (void)debugSetMaximumBevelCaptureTopologyNodes:(NSUInteger)limit;
- (void)debugSetMaximumBevelResultTopologyNodes:(NSUInteger)limit;
- (void)debugSetMaximumBevelResultSolids:(NSUInteger)limit;
- (void)debugSetBevelTransactionFailureCount:(NSUInteger)count;
//! Make the next N transient-preview discard attempts fail without clearing
//! controller state, so Cancel can be retried safely.
- (void)debugSetBevelCancelDiscardFailureCount:(NSUInteger)count;
//! Commit a real persisted XCAF transform change on the first captured source
//! without updating its retained AIS presentation.
- (BOOL)debugMutateFirstBevelSourcePersistedTransform;
//! Inject CommitCommand reporting/throw behavior after a real close: 0 normal,
//! 1 false-after-close, 2 throw-after-close.
- (void)debugSetExtrusionCommitMode:(NSInteger)mode;
//! Make the next N extrusion abort attempts fail before touching the command.
- (void)debugSetExtrusionAbortFailureCount:(NSUInteger)count;
//! Make the next N post-Commit label inspections report unavailable.
- (void)debugSetExtrusionPostCommitInspectFailureCount:(NSUInteger)count;
//! Deterministically capture a planar opening by stable entity identifier and
//! zero-based TopExp face index. Production Shell admission remains
//! authoritative.
- (BOOL)debugBeginShellWithEntityIdentifier:(NSString *)entityIdentifier
                          faceTopologyIndex:(NSUInteger)faceTopologyIndex
    NS_SWIFT_NAME(debugBeginShell(entityIdentifier:faceTopologyIndex:));
//! Shell state values are Unavailable=0, Selecting=1, Computing=2, Ready=3,
//! Committing=4, OutcomeUnknown=5, and Failed=6. Worker and result counters
//! expose latest-wins behavior without changing production scheduling.
- (NSDictionary<NSString *, NSNumber *> *)debugShellState;
- (void)debugSetShellPreviewWorkerBlocked:(BOOL)blocked;
- (void)debugSetMaximumShellCaptureTopologyNodes:(NSUInteger)limit;
- (void)debugSetMaximumShellResultTopologyNodes:(NSUInteger)limit;
- (void)debugSetShellTransactionFailureCount:(NSUInteger)count;
- (void)debugSetShellAbortFailureCount:(NSUInteger)count;
- (void)debugSetShellPreviewEraseFailureCount:(NSUInteger)count;
//! Inject CommitCommand reporting/throw behavior after a real close: 0 normal,
//! 1 false-after-close, 2 throw-after-close.
- (void)debugSetShellCommitMode:(NSInteger)mode;
//! Make the next N post-Commit stable-label inspections report unavailable.
- (void)debugSetShellPostCommitInspectFailureCount:(NSUInteger)count;
//! Commit a real persisted transform change behind the active Shell snapshot
//! without updating its retained AIS presentation.
- (BOOL)debugMutateShellSourcePersistedTransform;
//! Build a standalone pre-schema BinOcaf fixture with geometry and legacy
//! child-11/12 appearance, but no identity or visual-material infrastructure.
- (NSData *_Nullable)debugLegacyBinOcafFixtureData;
//! Standalone BinXCAF fixture whose XCAF document length unit is exactly one
//! meter per model unit. Used to prove unit metadata persistence end to end.
- (NSData *_Nullable)debugMeterLengthUnitBinXCAFFixtureData;
//! Standalone BinXCAF fixture whose sole box has a negative-determinant root
//! location. Used to prove mesh exporters preserve outward winding.
- (NSData *_Nullable)debugNegativeLocationBinXCAFFixtureData;
//! Import a private STEP artifact through the production reader for physical
//! unit, assembly, name, and appearance round-trip assertions.
- (BOOL)debugImportSTEPAtURL:(NSURL *)url
    NS_SWIFT_NAME(debugImportSTEP(at:));
//! Malformed BinXCAF fixture with a present negative document length unit.
- (NSData *_Nullable)debugInvalidLengthUnitBinXCAFFixtureData;
//! Explicit current XCAF length unit, or nil for a legacy unitless document.
- (NSNumber *_Nullable)debugDocumentMetersPerUnit;
//! Authoritative OCAF history depth for migration/transaction assertions.
- (NSInteger)debugDocumentUndoCount;
//! Current XCAF visual-material table size for ownership/GC regressions.
- (NSInteger)debugVisualMaterialDefinitionCount;
//! Lower the selected-object admission cap for aggregate texture-authoring
//! budget tests. Values above the production cap reset to that cap.
- (void)debugSetMaximumTextureAuthoringObjects:(NSUInteger)limit;
//! Lower the projected serialized texture-occurrence byte ceiling so budget
//! regressions can use small valid images. Values above 128 MiB reset to the
//! production ceiling.
- (void)debugSetMaximumSerializedTextureOccurrenceBytes:(NSUInteger)limit;
//! Lower the aggregate decoded unique-resource ceiling for writer tests.
- (void)debugSetMaximumDecodedTextureResourceBytes:(NSUInteger)limit;
//! Lower the immutable visual-material definition cap for batch replacement
//! tests. Values above 2048 reset to the production cap.
- (void)debugSetMaximumVisualMaterialDefinitions:(NSUInteger)limit;
//! Remove cached face triangulations without remeshing. Texture assignment
//! must fail closed while texture removal remains available.
- (BOOL)debugClearSelectedCachedTriangulationsForTextureTest;
//! Standalone malformed BinXCAF fixture used to prove that loading rejects
//! external texture paths and preserves the active document on failure.
- (NSData *_Nullable)debugExternalTextureBinXCAFFixtureData;
//! BinXCAF fixture containing a visual-material attribute outside the material
//! tool table. Raw persistence sees it, so project validation must reject it.
- (NSData *_Nullable)debugOrphanVisualMaterialBinXCAFFixtureData;
//! Orphan material attributes on document Main and the TDF data root.
- (NSData *_Nullable)debugMainOrphanVisualMaterialBinXCAFFixtureData;
- (NSData *_Nullable)debugDataRootOrphanVisualMaterialBinXCAFFixtureData;
- (NSData *_Nullable)debugDataRootSiblingOrphanVisualMaterialBinXCAFFixtureData;
//! Standalone BinXCAF whose embedded texture declares INT32_MAX bytes. The
//! bounded retrieval driver must reject it before allocating the payload.
- (NSData *_Nullable)debugOversizedTextureLengthBinXCAFFixtureData;
//! Standalone BinXCAF with a reachable assembly-reference cycle.
- (NSData *_Nullable)debugCyclicAssemblyBinXCAFFixtureData;
//! Standalone nested BinXCAF assembly with two differently colored and
//! translated occurrences sharing one box definition.
- (NSData *_Nullable)debugSharedDefinitionAssemblyBinXCAFFixtureData;
//! Standalone BinXCAF with two free roots sharing the same assembly subtree.
- (NSData *_Nullable)debugSharedSubtreeMultipleRootsBinXCAFFixtureData;
//! Standalone assembly containing visible and hidden path/layer variants.
- (NSData *_Nullable)debugHiddenAssemblyVisibilityBinXCAFFixtureData;
//! Standalone free located box used to verify TopoDS location ownership.
- (NSData *_Nullable)debugLocatedFreeShapeBinXCAFFixtureData;
//! Shared occurrences whose imported PBR definition also carries a legacy
//! app-authored preset/color override for precedence testing.
- (NSData *_Nullable)debugAuthoredLegacyAssemblyBinXCAFFixtureData;
//! Free cube with an explicitly styled face label; topology-changing tools
//! must reject it instead of silently discarding subshape appearance.
- (NSData *_Nullable)debugStyledSubshapeBinXCAFFixtureData;
//! Single 300-sided prism exceeding extrusion's synchronous v1 topology cap.
- (NSData *_Nullable)debugOversizedExtrusionSolidBinXCAFFixtureData;
//! Lower the display traversal ceiling for bounded aggregate-budget tests.
- (void)debugSetMaximumDisplayTraversalNodes:(NSUInteger)limit;
//! Lower the leaf-presentation ceiling for all-or-nothing admission tests.
- (void)debugSetMaximumLeafPresentations:(NSUInteger)limit;
//! Lower the project-load topology ceiling and expose which validation path ran.
- (void)debugSetMaximumProjectTopologyValidationNodes:(NSUInteger)limit;
- (void)debugResetProjectTopologyValidationCounters;
- (NSUInteger)debugBoundedProjectTopologyValidationCount;
- (NSUInteger)debugGeometricBRepValidationCount;
//! Force cached-triangulation admission for full snapshots: 0 = normal,
//! 1 = missing, 2 = incompatible. This never mutates live OCAF/AIS state.
- (void)debugSetSceneSnapshotTriangulationFailureMode:(NSInteger)mode;
//! Full snapshot publication is forbidden from owning a mesher. These seams
//! prove it remains at zero independently from OpenGL presentation meshing.
- (void)debugResetSceneSnapshotMesherInvocationCount;
- (NSUInteger)debugSceneSnapshotMesherInvocationCount;
//! Assign an invisible in-memory XCAF layer to the occurrence at the requested
//! accumulated X translation, then redraw the OpenGL fallback.
- (BOOL)debugHideOccurrenceWithInvisibleLayerAtTranslationX:(CGFloat)x
    NS_SWIFT_NAME(debugHideOccurrenceWithInvisibleLayer(translationX:));
//! Current OpenGL AIS shape presentation state. Test-only verification for
//! occurrence-local transforms and resolved fallback colors.
- (NSArray<NSDictionary<NSString *, NSNumber *> *> *)
    debugDisplayedShapePresentationStates;
//! Bounded color evidence from the current OpenGL view after an actual redraw.
- (NSDictionary<NSString *, NSNumber *> *)debugFramebufferStatistics;
//! Current OpenGL selection size. Assembly occurrences are expected to remain
//! zero after select-all because definition-addressed edits are unsafe.
- (NSInteger)debugSelectedShapeCount;
//! Valid imported PBR material whose Common fallback owns an embedded PNG.
//! Scalar authoring must remain read-only until that texture is app-owned.
- (NSData *_Nullable)debugCommonTextureBinXCAFFixtureData;
//! Valid XCAF material with a normal map that schema v4 cannot represent.
//! Snapshot publication must fail closed so OCCT remains authoritative.
- (NSData *_Nullable)debugUnsupportedPBRTextureBinXCAFFixtureData;
//! Same fail-closed fixture shape with a metallic-roughness map.
- (NSData *_Nullable)debugUnsupportedMetallicRoughnessTextureBinXCAFFixtureData;
//! Supported imported emissive map with a nonzero factor. Clearing the map
//! must preserve that factor because it was not auto-promoted by Shapeyard.
- (NSData *_Nullable)debugImportedEmissiveTextureBinXCAFFixtureData;
//! Valid imported untextured PBR material with non-default alpha and culling.
- (NSData *_Nullable)debugMaskedDoubleSidedPBRBinXCAFFixtureData;
//! Valid imported PBR material proving that explicit opaque alpha and
//! front-face culling survive snapshot transport without normalization.
- (NSData *_Nullable)debugOpaqueFrontCulledPBRBinXCAFFixtureData;
#endif

- (void)addTestPrimitives;

- (void)sendNotifyUIState:(UIStateChanging)state NS_REFINED_FOR_SWIFT;
- (void)viewWillUpdateUIState:(UIStateChanging)state;
- (void)viewDidSetup;
//! Called before the first raw primary touch can invalidate the viewport.
//! Coordinates use top-left-origin drawable pixels.
- (void)viewWillBeginPrimaryInteractionAtDrawablePoint:(CGPoint)point
                                          drawableSize:(CGSize)drawableSize
    NS_SWIFT_NAME(viewWillBeginPrimaryInteraction(atDrawablePoint:drawableSize:));
//! Called once after the last raw touch ends or is cancelled and OCCT has
//! committed or rolled back its authoritative interaction.
- (void)viewDidEndPrimaryInteractionCancelled:(BOOL)cancelled
    NS_SWIFT_NAME(viewDidEndPrimaryInteraction(cancelled:));
//! Called once for a recognized viewport tap, before OCCT applies its existing
//! selection semantics. Coordinates use top-left-origin drawable pixels.
- (void)viewWillSelectAtDrawablePoint:(CGPoint)point
                         drawableSize:(CGSize)drawableSize
    NS_SWIFT_NAME(viewWillSelect(atDrawablePoint:drawableSize:));
//! Called synchronously after Core3D finalizes selection, gizmo, and related
//! presentation state, before the debounced UI-state notification. Renderer
//! clients should re-evaluate fallback policy and capture a full presentation
//! snapshot rather than republishing a camera-only frame.
- (void)viewDidChangeViewportPresentationState;
//! Called on the main queue after a transient modeling parameter changes only
//! its bounded presentation overlay. Delivery may occur after the current
//! event turn. Renderer clients may republish that overlay against
//! their last compatible committed-scene snapshot; if no compatible base
//! exists, they should fall back to a full capture.
- (void)viewDidChangeViewportPresentationOverlay;
//! Called after the native viewport has been invalidated. Subclasses should
//! coalesce work and must not synchronously recapture full geometry per call.
- (void)viewDidInvalidateSceneSnapshot;
- (void)viewDidAssetModify;
- (void)viewDidLoadFromBundle;
- (void)viewDidFailToLoadFromBundle:(Core3DAssetLoadResult)result
    NS_SWIFT_NAME(viewDidFailToLoadFromBundle(_:));

@end

NS_ASSUME_NONNULL_END
