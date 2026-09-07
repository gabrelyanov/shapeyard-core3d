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

typedef NS_ENUM(NSInteger, Core3DProfilePlane) {
    Core3DProfilePlaneXY = 0, Core3DProfilePlaneXZ, Core3DProfilePlaneYZ,
};
typedef NS_ENUM(NSInteger, Core3DProfileConstructionResult) {
    Core3DProfileConstructionResultCommitted = 0,
    Core3DProfileConstructionResultRejected,
    Core3DProfileConstructionResultBusy,
    Core3DProfileConstructionResultRecoveryRequired,
    Core3DProfileConstructionResultFailed,
    Core3DProfileConstructionResultCancelled,
};

typedef NS_ENUM(NSInteger, Core3DObjectAlignmentAnchor) {
    Core3DObjectAlignmentAnchorMinimum = 0,
    Core3DObjectAlignmentAnchorCenter,
    Core3DObjectAlignmentAnchorMaximum,
    Core3DObjectAlignmentAnchorGround,
    //! Distribute 3–32 objects in center order, with stable entity-ID ties.
    //! Centers keeps endpoint centers fixed; Gaps preserves outer bounds and
    //! rejects insufficient space. Saved-group members remain separate parts.
    Core3DObjectAlignmentAnchorEqualCenters,
    Core3DObjectAlignmentAnchorEqualGaps,
};
typedef NS_ENUM(NSInteger, Core3DObjectAlignmentResult) {
    Core3DObjectAlignmentResultUnchanged = 0,
    Core3DObjectAlignmentResultCommitted,
    Core3DObjectAlignmentResultRejected,
    Core3DObjectAlignmentResultBusy,
    Core3DObjectAlignmentResultRecoveryRequired,
    Core3DObjectAlignmentResultFailed,
    Core3DObjectAlignmentResultCancelled,
};

typedef NS_ENUM(NSInteger, Core3DSavedGroupEditResult) {
    Core3DSavedGroupEditResultUnchanged = 0,
    Core3DSavedGroupEditResultCommitted,
    Core3DSavedGroupEditResultRejected,
    Core3DSavedGroupEditResultBusy,
    Core3DSavedGroupEditResultRecoveryRequired,
    Core3DSavedGroupEditResultFailed,
    Core3DSavedGroupEditResultBlockedByLayer,
};

typedef NS_ENUM(NSInteger, Core3DMeshUVAtlasResult) {
    Core3DMeshUVAtlasResultUnchanged,
    Core3DMeshUVAtlasResultCommitted,
    Core3DMeshUVAtlasResultRejected,
    Core3DMeshUVAtlasResultBusy,
    Core3DMeshUVAtlasResultRecoveryRequired,
    Core3DMeshUVAtlasResultFailed,
};

typedef NS_ENUM(NSInteger, Core3DObjectNameEditResult) {
    Core3DObjectNameEditResultUnchanged = 0,
    Core3DObjectNameEditResultCommitted,
    Core3DObjectNameEditResultRejected,
    Core3DObjectNameEditResultBusy,
    Core3DObjectNameEditResultRecoveryRequired,
    Core3DObjectNameEditResultFailed,
};

typedef NS_ENUM(NSInteger, Core3DObjectVisibilityEditResult) {
    Core3DObjectVisibilityEditResultUnchanged = 0,
    Core3DObjectVisibilityEditResultCommitted,
    Core3DObjectVisibilityEditResultRejected,
    Core3DObjectVisibilityEditResultBusy,
    Core3DObjectVisibilityEditResultRecoveryRequired,
    Core3DObjectVisibilityEditResultFailed,
    Core3DObjectVisibilityEditResultBlockedByLayer,
};

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
    //! Valid marked BRep whose definition root is one analytic Face.
    Core3DDebugGeometryFixtureMarkedBRepFaceRoot,
    //! Valid marked BRep roots made solely from Wire, Edge, and Vertex shapes.
    Core3DDebugGeometryFixtureMarkedBRepSubshapeRoots,
};

//! Standalone BinXCAF fixtures for the all-or-nothing Reference Axis schema.
typedef NS_ENUM(NSInteger, Core3DDebugReferenceAxisFixtureMode) {
    Core3DDebugReferenceAxisFixtureValidMixedSpace = 0,
    Core3DDebugReferenceAxisFixturePartialRecord,
    Core3DDebugReferenceAxisFixtureUnknownMode,
    Core3DDebugReferenceAxisFixtureWrongModeType,
    Core3DDebugReferenceAxisFixtureNonFinitePivot,
    Core3DDebugReferenceAxisFixtureOversizedPivot,
    Core3DDebugReferenceAxisFixtureZeroDirection,
    Core3DDebugReferenceAxisFixtureNonUnitDirection,
    Core3DDebugReferenceAxisFixtureOrphanRecord,
    //! No authored record: the implicit default must still reject a malformed
    //! persisted object transform during production document admission.
    Core3DDebugReferenceAxisFixtureImplicitDefaultCorruptTransform,
    //! No authored record: a cumulative assembly location that pushes the
    //! implicit Object Origin outside the model-coordinate bound.
    Core3DDebugReferenceAxisFixtureImplicitDefaultOversizedOccurrence,
    //! The private Duplicate ownership GUID is occupied by the wrong scalar
    //! type on document Main and must fail production admission.
    Core3DDebugReferenceAxisFixtureDuplicateSentinelWrongType,
    //! Even the correct private Duplicate sentinel type is corrupt when it is
    //! attached anywhere except document Main.
    Core3DDebugReferenceAxisFixtureDuplicateSentinelMisplaced,
    //! The private Radial Array ownership GUID is occupied by the wrong
    //! scalar type on document Main and must fail production admission.
    Core3DDebugReferenceAxisFixtureRadialSentinelWrongType,
    //! Even the correct private Radial Array sentinel type is corrupt when it
    //! is attached anywhere except document Main.
    Core3DDebugReferenceAxisFixtureRadialSentinelMisplaced,
    Core3DDebugReferenceAxisFixtureOrdinarySentinelWrongType,
    Core3DDebugReferenceAxisFixtureOrdinarySentinelMisplaced,
    Core3DDebugReferenceAxisFixtureOrdinarySentinelValid,
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

//! Coordinate authority for one Reference Axis component. Numeric values are
//! deliberately identical to the persistent native schema and must not be
//! renumbered or reused.
typedef NS_ENUM(NSInteger, Core3DReferenceSpace) {
    Core3DReferenceSpaceObject = 0,
    Core3DReferenceSpaceWorld = 1,
};

//! Whether the active Radial Array source owns an authored Reference Axis or
//! is using the explicit, non-mutating Object-Origin + World-Z default.
typedef NS_ENUM(NSInteger, Core3DReferenceAxisReadState) {
    Core3DReferenceAxisReadStateInvalid = -1,
    Core3DReferenceAxisReadStateImplicitDefault = 0,
    Core3DReferenceAxisReadStateAuthored = 1,
};

//! Complete finite oriented line. Pivot and direction have independent
//! coordinate spaces; native Core normalizes every accepted direction.
typedef struct {
    Core3DReferenceSpace pivotSpace;
    double pivotX;
    double pivotY;
    double pivotZ;
    Core3DReferenceSpace directionSpace;
    double directionX;
    double directionY;
    double directionZ;
} Core3DReferenceAxisValue;

//! One immutable compare-and-swap lease for editing the active Radial Array
//! source's persistent Reference Axis. Token zero is unavailable, never a
//! wildcard. Callers must submit this exact token to Set or Reset.
typedef struct {
    Core3DReferenceAxisReadState readState;
    Core3DReferenceAxisValue value;
    uint64_t authorityToken;
} Core3DRadialArrayReferenceAuthority;

//! Native-authoritative parameters and bounds for one active Radial Array.
//! Count includes the unchanged source; signed sweep is expressed in degrees.
typedef struct {
    NSInteger count;
    NSInteger minimumCount;
    NSInteger maximumCount;
    double sweepDegrees;
    double minimumSweepDegrees;
    double maximumSweepDegrees;
    //! Physical document scale used to present custom pivot coordinates.
    double metersPerUnit;
} Core3DRadialArrayParameters;

//! Truthful result for an atomic persistent Reference Axis edit. An unknown
//! outcome retains the same operation and recovery controls; retryable failure
//! means the edit was not accepted and may be attempted again with fresh state.
typedef NS_ENUM(NSInteger, Core3DRadialArrayReferenceEditResult) {
    Core3DRadialArrayReferenceEditResultNoChange = 0,
    Core3DRadialArrayReferenceEditResultApplied,
    Core3DRadialArrayReferenceEditResultOutcomeUnknown,
    Core3DRadialArrayReferenceEditResultRetryableFailure,
};

//! Native-authoritative local BRep thickness for one active Shell preview.
//! metersPerUnit includes the captured object's absolute uniform scale, so
//! Swift's millimetre display describes the resulting world-space thickness.
//! The document's stored length unit is not changed.
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
//! Metres per local BRep distance, including the active source's absolute
//! saved uniform scale. Zero means no valid active extrusion. Read-only.
- (double)getExtrusionMetersPerUnit;
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
- (Core3DRadialArrayParameters)getRadialArrayParameters;
- (Core3DRadialArrayReferenceAuthority)getRadialArrayReferenceAuthority;
//! Read the same captured world reference line expressed in the requested
//! pivot/direction spaces. This is non-mutating and token-checked; failure is
//! returned as Invalid with authorityToken zero.
- (Core3DRadialArrayReferenceAuthority)
    getRadialArrayReferenceAuthorityWithPivotSpace:
        (Core3DReferenceSpace)pivotSpace
    directionSpace:(Core3DReferenceSpace)directionSpace
    expectedAuthorityToken:(uint64_t)expectedAuthorityToken
    NS_SWIFT_NAME(getRadialArrayReferenceAuthority(pivotSpace:directionSpace:expectedAuthorityToken:));
- (BOOL)setRadialArrayCount:(NSInteger)count;
- (BOOL)setRadialArraySweepDegrees:(double)sweepDegrees;
- (Core3DRadialArrayReferenceEditResult)setRadialArrayReferenceAxis:
    (Core3DReferenceAxisValue)axis
    expectedAuthorityToken:(uint64_t)expectedAuthorityToken
    NS_SWIFT_NAME(setRadialArrayReferenceAxis(_:expectedAuthorityToken:));
- (Core3DRadialArrayReferenceEditResult)
    resetRadialArrayReferenceAxisWithExpectedAuthorityToken:
        (uint64_t)expectedAuthorityToken
    NS_SWIFT_NAME(resetRadialArrayReferenceAxis(expectedAuthorityToken:));
- (Core3DModelingOperationResult)tryApplyRadialArray
    NS_SWIFT_NAME(tryApplyRadialArray());
- (Core3DModelingOperationResult)tryCancelRadialArray
    NS_SWIFT_NAME(tryCancelRadialArray());
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
//! operation. GLB prepares immutable geometry for startSceneSnapshot; OBJ,
//! binary STL, and exact BRep STEP produce files with start. All keep the bounded XBF
//! save on the main thread; translation/writing, validation, and cleanup run
//! off-main.
- (Core3DNativeExportOperation *_Nullable)
    prepareNativeExportOperationWithType:(ExportType)exportType
    NS_SWIFT_NAME(prepareNativeExportOperation(with:));
//! Freeze the current visible whole-object selection for GLB, OBJ or binary STL export.
//! Other selected formats, empty/subshape selections and unresolved object
//! identities reject. NO uses all-visible for GLB and existing all-document
//! behavior for native file exporters.
- (Core3DNativeExportOperation *_Nullable)
    prepareNativeExportOperationWithType:(ExportType)exportType
                    selectedObjectsOnly:(BOOL)selectedObjectsOnly
    NS_SWIFT_NAME(prepareNativeExportOperation(with:selectedObjectsOnly:));
//! Capture an immutable GLB/OBJ/STL mesh quality without changing the viewport or
//! editable source. Invalid qualities and non-viewport STEP presets reject.
- (Core3DNativeExportOperation *_Nullable)
    prepareNativeExportOperationWithType:(ExportType)exportType
                    selectedObjectsOnly:(BOOL)selectedObjectsOnly
                            meshQuality:(Core3DExportMeshQuality)meshQuality
    NS_SWIFT_NAME(prepareNativeExportOperation(with:selectedObjectsOnly:meshQuality:));
//! Capture an explicit OBJ color convention. Other formats require Current;
//! unsupported values reject before private files are created. Older overloads
//! retain Current. This preference never changes authored model appearance.
- (Core3DNativeExportOperation *_Nullable)
    prepareNativeExportOperationWithType:(ExportType)exportType
                    selectedObjectsOnly:(BOOL)selectedObjectsOnly
                            meshQuality:(Core3DExportMeshQuality)meshQuality
                     objColorConvention:(Core3DOBJColorConvention)objColorConvention
    NS_SWIFT_NAME(prepareNativeExportOperation(with:selectedObjectsOnly:meshQuality:objColorConvention:));
//! Capture only committed exportable geometry. Unlike the presentation
//! snapshot seam, this returns nil while a Boolean, Mirror, Linear Array,
//! Radial Array, or Shell trial is active or unresolved, or while the OCAF
//! document owns an open command. Main-thread only; the returned value is an
//! immutable deep copy safe for background I/O.
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

//! Resolve an ordinary edit before closing. NO retains recovery and means
//! the caller must keep the editor/document alive and offer another retry.
- (BOOL)prepareOrdinaryEditForDocumentClose;
- (BOOL)hasUnresolvedOrdinaryEdit;
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
//! Change camera projection without changing target-plane scale or the model.
//! Returns NO when the viewer is unavailable or a modeling operation is active.
- (BOOL)setCameraOrthographic:(BOOL)orthographic
    NS_SWIFT_NAME(setCameraProjection(orthographic:));
//! Frame visible committed objects without changing geometry or undo history.
//! Selected-only framing requires Object mode and a nonempty selection.
//! Returns NO when unavailable; the previous camera is retained.
- (BOOL)frameModelWithSelectedObjectsOnly:(BOOL)selectedObjectsOnly
    NS_SWIFT_NAME(frameModel(selectedObjectsOnly:));
//! As above, fitting a finite nonempty rectangle within [0,1] x [0,1].
//! The target uses a top-left origin in the complete drawable viewport.
- (BOOL)frameModelWithSelectedObjectsOnly:(BOOL)selectedObjectsOnly
                normalizedViewportRect:(CGRect)normalizedViewportRect
    NS_SWIFT_NAME(frameModel(selectedObjectsOnly:normalizedViewportRect:));
//! Focus one visible model occurrence without selecting it. The expected
//! publication stream, document generation and model revision must still match.
//! Camera/selection-only changes do not invalidate this read-only request.
- (BOOL)frameObjectWithEntityIdentifier:(NSString *)entityIdentifier
                             expected:(Core3DSceneSnapshot *)expected
               normalizedViewportRect:(CGRect)normalizedViewportRect
    NS_SWIFT_NAME(frameObject(entityIdentifier:expected:normalizedViewportRect:));
//! Replace whole-object selection with one visible editable browser entry.
//! Requires Object mode and the same publication/document/model identity.
//! Does not move the camera or create a model-history entry.
//! Rename one supported object without changing selection or geometry.
//! The expected scene identity/revision must still match; names are not IDs.
//! Object mode only. Changes a free object's saved visibility without selecting it.
- (Core3DObjectVisibilityEditResult)setObjectVisibilityWithEntityIdentifier:(NSString *)entityIdentifier
                                                                  visible:(BOOL)visible
                                                                 expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(setObjectVisibility(entityIdentifier:visible:expected:));
- (Core3DSavedGroupEditResult)createSavedGroupWithEntityIdentifiers:(NSArray<NSString *> *)entityIdentifiers
    name:(NSString *)name expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(createSavedGroup(entityIdentifiers:name:expected:));
- (Core3DSavedGroupEditResult)renameSavedGroup:(NSString *)identifier name:(NSString *)name
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(renameSavedGroup(identifier:name:expected:));
- (Core3DSavedGroupEditResult)ungroupSavedGroup:(NSString *)identifier
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(ungroupSavedGroup(identifier:expected:));
- (Core3DSavedGroupEditResult)setSavedGroupVisibility:(NSString *)identifier visible:(BOOL)visible
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(setSavedGroupVisibility(identifier:visible:expected:));
- (BOOL)selectSavedGroup:(NSString *)identifier expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(selectSavedGroup(identifier:expected:));
//! Generate a padded triangle atlas on one selected untextured mesh; one Undo.
- (Core3DMeshUVAtlasResult)generateTriangleUVAtlasForEntityIdentifier:(NSString *)entityIdentifier
                                                         expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(generateTriangleUVAtlas(entityIdentifier:expected:));
//! Coherent planar charts with common texel density; regenerates untextured v1/v2.
- (Core3DMeshUVAtlasResult)generateCoherentUVAtlasForEntityIdentifier:(NSString *)entityIdentifier
                                                       resolution:(NSInteger)resolution
                                                     gutterPixels:(NSInteger)gutterPixels
                                                         expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(generateCoherentUVAtlas(entityIdentifier:resolution:gutterPixels:expected:));
- (Core3DObjectNameEditResult)renameObjectWithEntityIdentifier:(NSString *)entityIdentifier
                                                      name:(NSString *)name
                                                  expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(renameObject(entityIdentifier:name:expected:));
//! Align up to 32 selected editable objects by analytic world bounds. Ground
//! uses minimum Z = 0 and admits one object. Main-thread completion; one Undo.
- (void)alignSelectedObjectsOnAxis:(Core3DTransformInspectorAxis)axis
                           anchor:(Core3DObjectAlignmentAnchor)anchor
                         expected:(Core3DSceneSnapshot *)expected
                       completion:(void(^)(Core3DObjectAlignmentResult))completion
    NS_SWIFT_NAME(alignSelectedObjects(axis:anchor:expected:completion:));
//! Suppress pending mutation; its completion returns Cancelled after the worker
//! drains. A second measurement is Busy until then, even after cancellation.
- (void)cancelObjectAlignment;
//! Construct one solid from 3–64 non-intersecting outline points in mm, either
//! winding, with positive depth 0.001–1,000,000 mm. One undoable creation.
- (void)createExtrudedProfileWithPoints:(NSArray<NSValue *> *)points
                                plane:(Core3DProfilePlane)plane depth:(double)depth
                             expected:(Core3DSceneSnapshot *)expected
                           completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(createExtrudedProfile(points:plane:depth:expected:completion:));
//! Extrude a polygon with up to16 exact circular through-holes. Center/radius
//! arrays must have equal lengths. Holes require0.001mm minimum clearance from
//! all polygon edges and other holes; one owned cancellable creation command.
- (void)createExtrudedProfileWithPoints:(NSArray<NSValue *> *)points
                          holeCenters:(NSArray<NSValue *> *)holeCenters
                            holeRadii:(NSArray<NSNumber *> *)holeRadii
                                plane:(Core3DProfilePlane)plane depth:(double)depth
                             expected:(Core3DSceneSnapshot *)expected
                           completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(createExtrudedProfile(points:holeCenters:holeRadii:plane:depth:expected:completion:));
//! Revolve a simple closed section around its plane's vertical axis at horizontal
//! coordinate zero (XY: Y axis; XZ/YZ: Z axis). All horizontal coordinates must
//! be nonnegative. Positive sweep 0.001–360 degrees; one undoable solid creation.
- (void)createRevolvedProfileWithPoints:(NSArray<NSValue *> *)points
                                plane:(Core3DProfilePlane)plane angleDegrees:(double)angleDegrees
                             expected:(Core3DSceneSnapshot *)expected
                           completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(createRevolvedProfile(points:plane:angleDegrees:expected:completion:));
//! Extrude an exact circle or concentric ring in mm on an explicit work plane.
//! innerRadius zero creates a disk. Positive radii, depth and wall thickness
//! have a 0.001 mm minimum. One owned, cancellable and undoable solid creation.
- (void)createCircularProfileWithCenter:(CGPoint)center outerRadius:(double)outerRadius
                          innerRadius:(double)innerRadius plane:(Core3DProfilePlane)plane
                                depth:(double)depth expected:(Core3DSceneSnapshot *)expected
                           completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(createCircularProfile(center:outerRadius:innerRadius:plane:depth:expected:completion:));
//! Revolve an exact circular/annular section into a torus or curved tube.
//! Center U is the bend radius; the full outer circle must stay at least
//! 0.001 mm to the right of the work-plane revolution axis. Angle: 0.001...360.
- (void)createRevolvedCircularProfileWithCenter:(CGPoint)center outerRadius:(double)outerRadius
                                  innerRadius:(double)innerRadius plane:(Core3DProfilePlane)plane
                                 angleDegrees:(double)angleDegrees expected:(Core3DSceneSnapshot *)expected
                                   completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(createRevolvedCircularProfile(center:outerRadius:innerRadius:plane:angleDegrees:expected:completion:));
//! Keep the worker slot until completion; cancel at document load/close boundaries.
- (void)cancelProfileConstruction;
- (BOOL)selectObjectWithEntityIdentifier:(NSString *)entityIdentifier
                              expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(selectObject(entityIdentifier:expected:));

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

//! Set one absolute extrinsic XYZ angle in degrees (finite, +/-1,000,000).
//! Uses the same frozen inspector lease; preserves position, scale and BRep.
- (Core3DTransformInspectorPositionCommitResult)
    commitTransformInspectorRotationValue:(double)value
                                      axis:(Core3DTransformInspectorAxis)axis
                          expectedSnapshot:
                              (Core3DTransformInspectorSnapshot *)snapshot
    NS_SWIFT_NAME(commitTransformInspectorRotation(_:axis:expected:));

//! Set an absolute signed uniform scale factor, dimensionless. Magnitude must
//! be greater than OCCT confusion tolerance and at most 1,000,000. Uses the
//! frozen inspector lease and UniformScale capability; preserves Position,
//! rotation and definition-local geometry. Zero is invalid.
- (Core3DTransformInspectorPositionCommitResult)
    commitTransformInspectorUniformScaleValue:(double)value
                          expectedSnapshot:(Core3DTransformInspectorSnapshot *)snapshot
    NS_SWIFT_NAME(commitTransformInspectorUniformScale(_:expected:));

//! Set one positive dimension in raw model units, measured on definition-local
//! axes and including absolute persisted scale. Reshapes BRep around the local
//! bounds center while preserving Position, rotation and signed uniform scale.
//! Requires ready cached bounds and NonuniformScale capability.
- (Core3DTransformInspectorPositionCommitResult)
    commitTransformInspectorDimensionValue:(double)value
                                      axis:(Core3DTransformInspectorAxis)axis
                          expectedSnapshot:(Core3DTransformInspectorSnapshot *)snapshot
    NS_SWIFT_NAME(commitTransformInspectorDimension(_:axis:expected:));

//! Release-safe counters for opt-in signed-device performance qualification.
//! Values are observational only and never alter inspector admission policy.
- (NSDictionary<NSString *, NSNumber *> *)
    transformInspectorPerformanceState;

//! Request a transient topology-selection mode and synchronously report the
//! mode accepted by Core3D. A non-success result preserves the prior public
//! selection mode.
- (Core3DSelectionTypeChangeResult)
    trySetSelectionType:(PrimitiveSelectionType)type;
- (void)setSelectionType:(PrimitiveSelectionType)type;
- (void)setGizmoType:(PrimitiveGizmoType)type;

#ifdef DEBUG
//! Re-run the production selection callback without changing AIS owners.
- (void)debugRefreshSelectionState;
//! Select one exact operation-owned presentation whose modes are deliberately
//! suspended, so the nonempty production selection callback can be tested.
- (BOOL)debugSelectRetainedOperationPresentation;
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
//! successful redraw, 2 incremental plus redraw traversal failure, and 3 a
//! forced post-redraw exact-owner restoration miss.
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
//! Read the first free-simple definition's stored and resolved reference line.
//! `readState` is Invalid=-1, ImplicitDefault=0, Authored=1.
- (NSDictionary<NSString *, NSNumber *> *)debugFirstReferenceAxisState;
//! Commit one complete reference record on the first free-simple definition.
//! Required numeric keys are pivotSpace, directionSpace, pivotX/Y/Z, and
//! directionX/Y/Z. A finite nonzero direction is normalized by native Core.
- (BOOL)debugSetFirstReferenceAxis:
    (NSDictionary<NSString *, NSNumber *> *)values;
//! Commit removal of the complete authored record.
- (BOOL)debugResetFirstReferenceAxis;
//! Commit a test-only persisted uniform scale without touching camera,
//! manipulator, or bounds authority. Negative values exercise oriented-axis
//! reversal in the production resolver.
- (BOOL)debugSetFirstReferenceAxisPersistedUniformScale:(CGFloat)scale;
//! Probe staging/abort: 0...3 invalid, 4 signed transform, 5 missing default scalar.
- (NSDictionary<NSString *, id> *_Nullable)debugProbeTransformStorage:(NSInteger)mode;
//! Create one deterministic XBF fixture. Only ValidMixedSpace may pass the
//! production project-load gate; every malformed mode must fail closed.
- (NSData *_Nullable)debugReferenceAxisFixtureDataWithMode:
    (Core3DDebugReferenceAxisFixtureMode)mode
    NS_SWIFT_NAME(debugReferenceAxisFixtureData(mode:));
//! Inject Duplicate recovery behavior: 0 normal, 1 false-after-close,
//! 2 throw-after-close, and 3 fail the first post-commit presentation repair.
//! Synchronous DEBUG fixture over real ordinary command ownership faults.
- (NSDictionary<NSString *, NSNumber *> *_Nullable)debugProbeOrdinaryCommand:(NSInteger)mode;
- (NSDictionary<NSString *, NSNumber *> *)debugOrdinaryCommandMarkerState;
- (NSDictionary<NSString *, NSNumber *> *_Nullable)debugProbeOrdinaryController:(NSInteger)mode;
- (NSInteger)debugApplyViewerOrdinaryTransform:(NSInteger)mode paused:(BOOL)paused;
- (NSInteger)debugReconcileViewerOrdinaryEdit;
- (NSInteger)debugApplyViewerOrdinaryPivotRotation:(NSInteger)mode;
- (BOOL)debugConfigureOrdinaryCreationFault:(NSInteger)mode;
- (BOOL)debugConfigureOrdinaryGestureFault:(NSInteger)mode;
- (BOOL)debugConfigureOrdinaryNameFault:(NSInteger)mode;
- (BOOL)debugConfigureOrdinaryVisibilityFault:(NSInteger)mode;
- (void)debugSetViewerOrdinaryVisibilityAfterRepairFailures:(NSInteger)count;
- (void)debugSetViewerOrdinaryRepairFailures:(NSInteger)incremental redraw:(NSInteger)redraw;
- (NSDictionary<NSString *, id> *)debugViewerOrdinaryState;
- (void)debugSetDuplicateCommitMode:(NSInteger)mode;
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
//! Radial Array state values are Unavailable=0, Selecting=1, Ready=2,
//! Committing=3, OutcomeUnknown=4, and Failed=5. Count includes the source.
- (NSDictionary<NSString *, NSNumber *> *)debugRadialArrayState;
//! Make the next N begins observe an open command not owned by Radial Array.
- (void)debugSetRadialArrayBeginOwnedCommandMismatchCount:(NSUInteger)count;
//! Fail the next N applies after Radial has proven command ownership.
- (void)debugSetRadialArrayTransactionFailureCount:(NSUInteger)count;
- (void)debugSetRadialArrayAbortFailureCount:(NSUInteger)count;
- (void)debugSetRadialArrayEraseFailureCount:(NSUInteger)count;
//! Apply CommitCommand modes: 0 normal, 1 false-after-close, 2 throw-after-close.
- (void)debugSetRadialArrayApplyCommitMode:(NSInteger)mode;
//! Post-commit inspect modes: 0 normal, 1 unavailable, 2 partial/mismatched.
- (void)debugSetRadialArrayPostCommitInspectMode:(NSInteger)mode;
- (void)debugSetMaximumRadialArrayTopologyNodes:(NSUInteger)limit;
- (BOOL)debugMutateRadialArraySourcePersistedTransform;
//! Reference-edit CommitCommand modes: 0 normal, 1 false-after-close,
//! 2 throw-after-close.
- (void)debugSetRadialArrayReferenceEditCommitMode:(NSInteger)mode;
//! Read-only identity proof over every committed free-simple definition.
//! Pairwise shape partnership and TriangleMesh triangulation handle identity
//! are intentionally unavailable in renderer-neutral scene snapshots.
- (NSDictionary<NSString *, NSNumber *> *)debugGeometryCopyIndependenceState;
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
//! Make the next N post-commit Boolean document inspections unavailable.
- (void)debugSetBooleanPostCommitInspectFailureCount:(NSUInteger)count;
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
//! Commit a different valid solid to the active Shell label without replacing
//! its retained AIS shape, creating deterministic stored/presentation drift.
- (BOOL)debugMutateShellSourcePersistedShape;
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
//! Deterministic projected gestures through the production transform/commit
//! path, replacing only handle hit detection. 0 Move, 1 Rotate, 2 uniform
//! Scale, 3 axis Scale. Values begin at offset/degrees 0 or scale factor 1.
//! Each returned sample contains transformDelta and changedShapeCount.
- (NSArray<NSDictionary<NSString *, NSNumber *> *> *_Nullable)
    debugReplayGesture:(NSInteger)mode axis:(NSInteger)axis
    values:(NSArray<NSNumber *> *)values
    NS_SWIFT_NAME(debugReplayGesture(mode:axis:values:));
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
- (NSData *_Nullable)debugSavedGroupBinXCAFFixture:(NSInteger)mode;
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
//! A TopAbs_SOLID containing an intentionally open five-face shell. Project
//! loading admits the trusted fixture; face-tool admission must reject it via
//! its bounded source BRep validity proof.
- (NSData *_Nullable)debugInvalidBRepSolidBinXCAFFixtureData;
//! A solid with 96 occurrences of one shared shell. Unique topology is tiny,
//! while occurrence topology exceeds Bevel's synchronous source cap.
- (NSData *_Nullable)debugOccurrenceAmplifiedBevelSolidBinXCAFFixtureData;
//! Metadata-only fixture inspection (maximum 256 free roots). This does not
//! require renderer triangulations or weaken topology admission.
- (NSArray<NSString *> *_Nullable)debugFreeShapeEntityIdentifiers;
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
//! Last published identity for the current document; zero when unavailable.
//! Safe during an owned preview and never publishes uncommitted geometry.
- (uint64_t)debugPublishedDocumentGeneration;
//! Last published model revision, not the current working document revision.
//! Check debugPublishedDocumentGeneration for availability; an empty model is 0.
- (uint64_t)debugPublishedModelRevision;
//! One-shot browser selection failure: 1 owner change, 2 gizmo, 3 rollback.
- (void)debugSetBrowserSelectionFailureMode:(NSInteger)mode;
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
//! Renderer-independent topology-selection authority. The selected kind uses
//! 0 none, 1 object, 2 face, 3 edge, and 4 vertex. Topology indices are
//! zero-based; -1 means no exact single topology identity. Representation uses
//! -1 invalid, 0 legacy BRep, 1 BRep, and 2 TriangleMesh.
- (NSDictionary<NSString *, id> *)debugTopologySelectionState;
//! Test-only OCCT detection seam. Boundedly probes the drawable for one
//! committed presentation and leaves that owner as the current hover.
- (BOOL)debugDetectAnyDisplayedShape;
//! Publish a deterministic detected Face owner whose topology is canonical by
//! TShape/location but has the opposite orientation.
- (BOOL)debugDetectReversedFaceTopologyIndexWithEntityIdentifier:
			(NSString *)entityIdentifier
	faceTopologyIndex:(NSUInteger)faceTopologyIndex
	NS_SWIFT_NAME(debugDetectReversedFace(entityIdentifier:faceTopologyIndex:));
//! Edge counterpart to debugDetectReversedFace.
- (BOOL)debugDetectReversedEdgeTopologyIndexWithEntityIdentifier:
			(NSString *)entityIdentifier
	edgeTopologyIndex:(NSUInteger)edgeTopologyIndex
	NS_SWIFT_NAME(debugDetectReversedEdge(entityIdentifier:edgeTopologyIndex:));
//! The first virtual Selectable read reports entityIdentifier while the next
//! reports foreignEntityIdentifier, constructing the otherwise impossible
//! DetectedInteractive/DetectedOwner mismatch for snapshot fail-closed tests.
- (BOOL)debugDetectAlternatingForeignSelectableEdgeWithEntityIdentifier:
			(NSString *)entityIdentifier
	foreignEntityIdentifier:(NSString *)foreignEntityIdentifier
	edgeTopologyIndex:(NSUInteger)edgeTopologyIndex
	NS_SWIFT_NAME(debugDetectAlternatingForeignSelectableEdge(entityIdentifier:foreignEntityIdentifier:edgeTopologyIndex:));
//! Boundedly finds one committed OCCT owner and selects it through the same XOR
//! path as a production tap. The accepted Object/Face/Edge mode determines the
//! selected topology kind; success requires one exact owner.
- (BOOL)debugSelectAnyDisplayedTopologyElement;
//! Deterministically publishes distinct zero-based Face owners for one exact
//! committed BRep presentation. This is the multi-selection counterpart to the
//! production tap seam and exists to prove raw-owner admission cardinality.
- (BOOL)debugSelectFaceTopologyIndicesWithEntityIdentifier:
            (NSString *)entityIdentifier
    faceTopologyIndices:(NSArray<NSNumber *> *)faceTopologyIndices
    NS_SWIFT_NAME(debugSelectFaceTopologyIndices(entityIdentifier:faceTopologyIndices:));
//! Publish one owner whose Face is the exact canonical presentation face with
//! only its orientation reversed. The owner remains attached to the canonical
//! presentation, proving face-tool admission rejects IsSame-but-not-IsEqual.
- (BOOL)debugSelectReversedFaceTopologyIndexWithEntityIdentifier:
            (NSString *)entityIdentifier
    faceTopologyIndex:(NSUInteger)faceTopologyIndex
    NS_SWIFT_NAME(debugSelectReversedFace(entityIdentifier:faceTopologyIndex:));
//! Deterministically publish exact canonical Edge owners for one BRep source.
- (BOOL)debugSelectEdgeTopologyIndicesWithEntityIdentifier:
			(NSString *)entityIdentifier
	edgeTopologyIndices:(NSArray<NSNumber *> *)edgeTopologyIndices
	NS_SWIFT_NAME(debugSelectEdgeTopologyIndices(entityIdentifier:edgeTopologyIndices:));
//! Publish an IsSame-but-orientation-reversed Edge owner.
- (BOOL)debugSelectReversedEdgeTopologyIndexWithEntityIdentifier:
			(NSString *)entityIdentifier
	edgeTopologyIndex:(NSUInteger)edgeTopologyIndex
	NS_SWIFT_NAME(debugSelectReversedEdge(entityIdentifier:edgeTopologyIndex:));
//! Publish one valid Edge owner plus one owner whose edge belongs to the first
//! presentation but whose selectable is the second presentation.
- (BOOL)debugSelectValidAndForeignEdgeOwnersWithEntityIdentifier:
			(NSString *)entityIdentifier
	foreignEntityIdentifier:(NSString *)foreignEntityIdentifier
	edgeTopologyIndex:(NSUInteger)edgeTopologyIndex
	NS_SWIFT_NAME(debugSelectValidAndForeignEdgeOwners(entityIdentifier:foreignEntityIdentifier:edgeTopologyIndex:));
//! Change only the native picker mode, deliberately leaving the public cached
//! mode untouched. This proves shared tools fail closed when two individually
//! legal modes disagree about the active selection authority.
- (Core3DSelectionTypeChangeResult)
    debugTrySetNativeSelectionTypeWithoutPublicSync:
        (PrimitiveSelectionType)mode
    NS_SWIFT_NAME(debugTrySetNativeSelectionTypeWithoutPublicSync(_:));
//! Invoke the lower GL mutation boundaries directly, bypassing the public
//! controller's cancellation and authority checks. Active ledgers must still
//! reject both operations without changing document or selection state.
- (void)debugInvokeNativeSelectAll;
- (void)debugInvokeNativeDeleteSelected;
//! Direct native gizmo access for cache-reconciliation regressions. Setting the
//! native value deliberately leaves the public cached tool untouched.
- (PrimitiveGizmoType)debugNativeGizmoType;
- (void)debugSetNativeGizmoTypeWithoutPublicSync:
    (PrimitiveGizmoType)type;
//! Deliberately drifts the first committed presentation without changing the
//! accepted enum, so same-mode authority repair can be verified.
- (BOOL)debugSetFirstDisplayedShapeSelectionMode:
    (PrimitiveSelectionType)mode;
//! Deliberately drifts the one committed presentation matching an exact entity
//! identifier. Ambiguous or missing presentation ownership fails closed.
- (BOOL)debugSetDisplayedShapeSelectionModeWithEntityIdentifier:
            (NSString *)entityIdentifier
    mode:(PrimitiveSelectionType)mode
    NS_SWIFT_NAME(debugSetDisplayedShapeSelectionMode(entityIdentifier:mode:));
//! Fail the next N topology transitions after requested presentation modes are
//! staged but before old selected/detected owners are cleared.
- (void)debugSetSelectionModeVerificationFailureCount:(NSUInteger)count;
//! Valid imported PBR material whose Common fallback owns an embedded PNG.
//! Scalar authoring must remain read-only until that texture is app-owned.
- (NSData *_Nullable)debugCommonTextureBinXCAFFixtureData;
//! Independent mixed common/PBR/color-only fixture for OBJ color conventions.
- (NSData *_Nullable)debugOBJColorConventionBinXCAFFixtureData;
//! Valid XCAF material with a normal map that schema v6 cannot represent.
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
