//
//  Core3DViewController.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 28.03.2024.
//

#import <UIKit/UIKit.h>
#import <simd/simd.h>
#import <Core3D/PrimitiveType.h>
#import <Core3D/PrimitiveSelectionType.h>
#import <Core3D/PrimitiveGizmoType.h>
#import <Core3D/UIStateChanging.h>
#import <Core3D/ExportType.h>
#import <Core3D/Core3DNativeExportOperation.h>
#import <Core3D/Core3DMeshContactOperation.h>
#import <Core3D/OrthoProjectionType.h>
#import <Core3D/Core3DMaterialController.h>
#import <Core3D/Core3DModelCapability.h>
#import <Core3D/Core3DTransformInspectorSnapshot.h>

#import <Core3D/Core3DModelingTypes.h>

NS_ASSUME_NONNULL_BEGIN

typedef NS_ENUM(NSInteger, Core3DShellOpeningAxis) {
    Core3DShellOpeningAxisX, Core3DShellOpeningAxisY, Core3DShellOpeningAxisZ,
};
typedef NS_ENUM(NSInteger, Core3DShellOpeningSide) {
    Core3DShellOpeningSideMinimum, Core3DShellOpeningSideMaximum,
};
__attribute__((objc_subclassing_restricted))
@interface Core3DShellOpeningSelector : NSObject
@property(nonatomic,readonly) Core3DShellOpeningAxis axis;
@property(nonatomic,readonly) Core3DShellOpeningSide side;
- (nullable instancetype)initWithAxis:(Core3DShellOpeningAxis)axis side:(Core3DShellOpeningSide)side;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

typedef NS_ENUM(NSInteger, Core3DShellOperationState) {
    Core3DShellOperationStateCaptured,
    Core3DShellOperationStatePreparing,
    Core3DShellOperationStateReady,
    Core3DShellOperationStateCommitted,
    Core3DShellOperationStateCancelled,
    Core3DShellOperationStateRejected,
    Core3DShellOperationStateOutcomeUnknown,
    Core3DShellOperationStateRetryableFailure,
};
//! Opaque native capture, never deserialized from provider fields. Main-thread
//! only. Opening descriptors are descriptive; canonical faces stay private.
__attribute__((objc_subclassing_restricted))
@interface Core3DShellOperation : NSObject
@property(nonatomic,copy,readonly) NSString *entityIdentifier;
@property(nonatomic,copy,readonly) NSArray<Core3DShellOpeningSelector *> *openings;
@property(nonatomic,readonly) double thicknessMM;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
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

//! Preview one of the six world-axis planes at the current selection bounds.
//! This synchronous UI operation changes only the transient preview; Apply
//! owns the native document command. It is not a stable-target AI adapter.
- (BOOL)previewMirrorAxis:(Core3DMirrorAxis)axis backward:(BOOL)backward
    NS_SWIFT_NAME(previewMirror(axis:backward:));

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

// Success means accepted; terminal completion follows private cleanup.
- (Core3DAssetLoadResult)tryLoadFromBundle:(NSURL *)bundleURL
    NS_SWIFT_NAME(tryLoad(fromBundle:));
- (Core3DAssetLoadResult)tryLoadFromAssetFile:(NSURL *)URL
    expectedByteCount:(unsigned long long)count expectedSHA256:(NSString *)sha
    NS_SWIFT_NAME(tryLoad(fromAssetFile:expectedByteCount:expectedSHA256:));
- (void)viewDidChangeAssetLoadCleanupPending:(BOOL)pending
    NS_SWIFT_NAME(viewDidChangeAssetLoadCleanupPending(_:));
- (void)retryAssetLoadCleanup;
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
//! Main-thread exact single selected source, one use; no AI receipt authority.
- (Core3DPBRScalarPreparation *_Nullable)prepareScalarPBREdit:(Core3DPBRScalarEdit *)edit
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(prepareScalarPBR(edit:expected:));
- (Core3DPBRScalarResult)executeScalarPBRPreparation:(Core3DPBRScalarPreparation *)prepared
    NS_SWIFT_NAME(executeScalarPBR(_:));
- (void)cancelScalarPBRPreparation:(Core3DPBRScalarPreparation *)prepared
    NS_SWIFT_NAME(cancelScalarPBR(_:));
//! Observe one native frame and its invalidation, main-thread only. The callback
//! receives a camera only after actual Draw and successful presentation; a later
//! redraw/edit/lifecycle invalidation reports nil and releases the observer.
//! Caller supplies request identity and must cancel on owner disappearance.
//! Cached frame model revisions are not geometry or edit authority.
- (BOOL)observeNativeViewportPresentation:(NSUUID *)identifier
    changed:(void (^)(Core3DSceneFrameSnapshot *_Nullable frame, CGSize drawableSize))changed
    NS_SWIFT_NAME(observeNativeViewportPresentation(_:changed:));
- (void)cancelNativeViewportPresentation:(NSUUID *)identifier;
- (BOOL)isNativeViewportPresentationCurrent:(NSUUID *)identifier;
#ifdef DEBUG
// DEBUG-only lighting diagnosis; no native geometry or persisted material mutation.
- (NSDictionary<NSString *, id> *_Nullable)debugNativeLightingState;
- (BOOL)debugSetNativeHeadlightDirectionX:(double)x y:(double)y z:(double)z
    NS_SWIFT_NAME(debugSetNativeHeadlightDirection(x:y:z:));

// Replay native orbit touch-down/zero sample/drag/return-to-origin (modes0..3).
- (BOOL)debugReplayCameraTouch:(NSInteger)mode;
- (void)debugFailNextNativeViewportDraw;
- (void)debugSkipNextNativeViewportPresentation;
#endif

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
//! Fit selected source objects and an exact Ready axis-mirror preview without
//! changing geometry, selection, preview generation or Undo history. Rejects
//! stale scene/camera/drawable identity and retains the old camera on failure.
//! Custom reference-plane preview framing is not admitted by this contract.
- (BOOL)frameMirrorPreviewWithExpected:(Core3DSceneSnapshot *)expected
                           generation:(uint64_t)generation
               normalizedViewportRect:(CGRect)normalizedViewportRect
    NS_SWIFT_NAME(frameMirrorPreview(expected:generation:normalizedViewportRect:));
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
- (void)setSavedGroupBaseCenterOrigin:(NSString *)identifier expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DObjectAlignmentResult))completion
    NS_SWIFT_NAME(setSavedGroupBaseCenterOrigin(identifier:expected:completion:));
- (Core3DSavedGroupEditResult)resetSavedGroupOrigin:(NSString *)identifier
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(resetSavedGroupOrigin(identifier:expected:));
- (BOOL)selectSavedGroup:(NSString *)identifier expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(selectSavedGroup(identifier:expected:));
//! Create an independent mesh at current tessellation; retain/hide source; one Undo.
- (Core3DMeshVertexEditSnapshot *_Nullable)prepareMeshVertexEditForEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(prepareMeshVertexEdit(entityIdentifier:expected:));
- (Core3DMeshVertexEditResult)commitMeshVertexEdit:(Core3DMeshVertexEditSnapshot *)expected
    vertexIndices:(NSArray<NSNumber *> *)vertexIndices deltaX:(double)deltaX deltaY:(double)deltaY deltaZ:(double)deltaZ
    NS_SWIFT_NAME(commitMeshVertexEdit(_:vertexIndices:deltaX:deltaY:deltaZ:));
- (Core3DMeshVertexEditSnapshot *_Nullable)prepareMeshElementEditForEntityIdentifier:(NSString *)entityIdentifier
    kind:(Core3DMeshElementKind)kind expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(prepareMeshElementEdit(entityIdentifier:kind:expected:));
- (Core3DMeshVertexEditResult)commitMeshElementEdit:(Core3DMeshVertexEditSnapshot *)expected
    elementIndices:(NSArray<NSNumber *> *)elementIndices deltaX:(double)deltaX deltaY:(double)deltaY deltaZ:(double)deltaZ
    NS_SWIFT_NAME(commitMeshElementEdit(_:elementIndices:deltaX:deltaY:deltaZ:));
- (void)cancelMeshVertexEdit:(Core3DMeshVertexEditSnapshot *)expected;
//! Grow the seed across its native connected planar region; no mutation.
- (Core3DMeshRegionExtrudeSnapshot *_Nullable)prepareMeshRegionExtrudeForEntityIdentifier:(NSString *)entityIdentifier
    seedTriangle:(NSUInteger)seedTriangle expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(prepareMeshRegionExtrude(entityIdentifier:seedTriangle:expected:));
//! Positive world-millimetre distance; consumes the native session exactly once.
- (Core3DMeshRegionExtrudeResult)commitMeshRegionExtrude:(Core3DMeshRegionExtrudeSnapshot *)expected
    distanceMM:(double)distanceMM NS_SWIFT_NAME(commitMeshRegionExtrude(_:distanceMM:));
- (void)cancelMeshRegionExtrude:(Core3DMeshRegionExtrudeSnapshot *)expected;
//! Inset the native-resolved convex planar region; preview is immutable and one-use.
- (Core3DMeshRegionInsetSnapshot *_Nullable)prepareMeshRegionInsetForEntityIdentifier:(NSString *)entityIdentifier
    seedTriangle:(NSUInteger)seedTriangle expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(prepareMeshRegionInset(entityIdentifier:seedTriangle:expected:));
//! Positive world-millimetre distance; shape and replacement partition commit atomically.
- (Core3DMeshRegionInsetResult)commitMeshRegionInset:(Core3DMeshRegionInsetSnapshot *)expected
    distanceMM:(double)distanceMM NS_SWIFT_NAME(commitMeshRegionInset(_:distanceMM:));
- (void)cancelMeshRegionInset:(Core3DMeshRegionInsetSnapshot *)expected;

//! Read-only exact self-contact diagnostic for an explicit native mesh occurrence.
//! Never changes selection or history. Completion is asynchronous on main.
- (Core3DMeshContactOperation *)checkMeshContactsForEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected completion:(Core3DMeshContactCompletion)completion
    NS_SWIFT_NAME(checkMeshContacts(entityIdentifier:expected:completion:));
//! Only the latest report issued by this owner can be current. Native source
//! coordinates, topology, placement and document lifetime are revalidated.
- (BOOL)isMeshContactReportCurrent:(Core3DMeshContactReport *)report;
//! Release the latest issued report's retained source without changing the
//! document. Foreign or superseded reports cannot release another request.
- (void)discardMeshContactReport:(Core3DMeshContactReport *)report;
//! Read-only spatial inspection of an exact current owner-issued pair.
//! Negative/out-of-range indices, stale and foreign reports are rejected.
- (nullable Core3DMeshContactInspection *)makeMeshContactInspectionForReport:
    (Core3DMeshContactReport *)report pairIndex:(NSInteger)pairIndex
    NS_SWIFT_NAME(makeMeshContactInspection(report:pairIndex:));
#if DEBUG
//! One-shot observations on main around the next real native contact request.
- (void)debugSetNextMeshContactAfterCaptureHook:(void (^_Nullable)(void))afterCapture
                           beforeDeliveryHook:(void (^_Nullable)(void))beforeDelivery
    NS_SWIFT_NAME(debugSetNextMeshContactHooks(afterCapture:beforeDelivery:));
//! One-shot asynchronous test gate before native validation/public delivery.
//! Runs on main; retain resume while waiting, then invoke from any thread.
//! Repeated resumes are ignored. No main-thread blocking or geometry injection.
- (void)debugSetNextMeshContactDeliveryGate:(void (^_Nullable)(void (^resume)(void)))gate
    NS_SWIFT_NAME(debugSetNextMeshContactDeliveryGate(_:));
#endif

- (Core3DMeshCopyResult)createSourceRetainedMeshCopyForEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(createSourceRetainedMeshCopy(entityIdentifier:expected:));
//! Repair consistent winding of one admitted selected flat mesh; one ordinary Undo.
//! Exact publication, document, model, presentation and native selection are revalidated.
- (Core3DMeshWindingRepairResult)repairMeshWindingForEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(repairMeshWinding(entityIdentifier:expected:));
//! Generate a padded triangle atlas on one selected untextured mesh; one Undo.
- (Core3DMeshUVAtlasResult)generateTriangleUVAtlasForEntityIdentifier:(NSString *)entityIdentifier
                                                         expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(generateTriangleUVAtlas(entityIdentifier:expected:));
//! Inspect an owned v2 atlas and its saved settings without proposing regeneration.
//! Read-only under the exact selected entity and publication, including textured meshes.
- (Core3DMeshUVAtlasPreview *_Nullable)currentMeshUVAtlasPreviewForEntityIdentifier:(NSString *)entityIdentifier
                                                                    expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(currentMeshUVAtlasPreview(entityIdentifier:expected:));
- (Core3DMeshUVAtlasPreview *_Nullable)previewCoherentUVAtlasForEntityIdentifier:(NSString *)entityIdentifier
                                                       resolution:(NSInteger)resolution
                                                     gutterPixels:(NSInteger)gutterPixels
                                                         expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(previewCoherentUVAtlas(entityIdentifier:resolution:gutterPixels:expected:));
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
//! Create from immutable validated values with explicit units. The declared
//! unit must match the captured native document exactly; no implicit scaling.
//! Uses the same cancellable worker and ordinary history as touch construction.
//! Creates one saved native sweep through the existing ordinary worker/history path.
- (nullable Core3DStoredSweepSnapshot *)storedSweepWithEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(storedSweep(entityIdentifier:expected:));
- (void)rebuildStoredSweep:(Core3DStoredSweepSnapshot *)original
    definition:(Core3DSweepDefinition *)definition expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(rebuildStoredSweep(_:definition:expected:completion:));
- (nullable Core3DCylindricalCutSnapshot *)cylindricalCutSourceWithEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(cylindricalCutSource(entityIdentifier:expected:));
- (nullable Core3DCylindricalCutOperation *)beginCylindricalCut:(Core3DCylindricalCutSnapshot *)original
    definition:(Core3DCylindricalCutDefinition *)definition expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginCylindricalCut(_:definition:expected:completion:));
- (nullable Core3DCylindricalCutOperation *)beginCylindricalCutRadius:(Core3DCylindricalCutSnapshot *)original
    worldRadiusMM:(double)radius expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginCylindricalCutRadius(_:worldRadiusMM:expected:completion:));
//! Whole-program snapshot of a retained cut: every stable bore of the complete
//! recipe. Nil for bare sources and unchanged for legacy one-bore authority.
- (nullable Core3DCylindricalCutProgramSnapshot *)cylindricalCutProgramWithEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(cylindricalCutProgram(entityIdentifier:expected:));
//! Append one separated same-axis through-all bore. The next persisted
//! high-water operand identifier is issued natively; first-cut APIs unchanged.
- (nullable Core3DCylindricalCutOperation *)beginCylindricalCutAppendBore:(Core3DCylindricalCutProgramSnapshot *)original
    definition:(Core3DCylindricalCutDefinition *)definition expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginCylindricalCutAppendBore(_:definition:expected:completion:));
//! Change exactly the addressed stable operand's radius inside the original
//! full recipe. Unknown identifiers and legacy payloads refuse explicitly.
- (nullable Core3DCylindricalCutOperation *)beginCylindricalCutBoreRadius:(Core3DCylindricalCutProgramSnapshot *)original
    operandIdentifier:(uint32_t)operandIdentifier worldRadiusMM:(double)radius expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginCylindricalCutBoreRadius(_:operandIdentifier:worldRadiusMM:expected:completion:));
//! Main-thread, bounded, read-only query tied to the opening program and scene.
//! A nil program explicitly reports unsupported (e.g. a bare loft).
- (Core3DRetainedFilletCandidateQuery *)retainedFilletCandidates:(nullable Core3DCylindricalCutProgramSnapshot *)original
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(retainedFilletCandidates(_:expected:));
//! Cleared to generic at admission; populated before synchronous or asynchronous completion.
@property(nonatomic,readonly) Core3DRetainedFilletOutcome lastRetainedFilletOutcome;
//! Append/edit/remove retained rounds through one ordinary history step.
- (nullable Core3DCylindricalCutOperation *)beginFilletStepAppend:(Core3DCylindricalCutProgramSnapshot *)original
    anchors:(NSArray<Core3DRetainedFilletAnchor *> *)anchors radiusMM:(double)radius expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion NS_SWIFT_NAME(beginFilletStepAppend(_:anchors:radiusMM:expected:completion:));
- (nullable Core3DCylindricalCutOperation *)beginFilletStepRadius:(Core3DCylindricalCutProgramSnapshot *)original
    stepIdentifier:(uint64_t)identifier radiusMM:(double)radius expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion NS_SWIFT_NAME(beginFilletStepRadius(_:stepIdentifier:radiusMM:expected:completion:));
- (nullable Core3DCylindricalCutOperation *)beginFilletStepRemoval:(Core3DCylindricalCutProgramSnapshot *)original
    stepIdentifier:(uint64_t)identifier expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion NS_SWIFT_NAME(beginFilletStepRemoval(_:stepIdentifier:expected:completion:));
- (nullable Core3DCylindricalCutOperation *)beginWedgeCutAppend:(Core3DCylindricalCutProgramSnapshot *)original
    definition:(Core3DWedgeCutDefinition *)definition expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginWedgeCutAppend(_:definition:expected:completion:));
- (nullable Core3DCylindricalCutOperation *)beginWedgeCutWidths:(Core3DCylindricalCutProgramSnapshot *)original
    operandIdentifier:(uint32_t)identifier worldHalfWidthApexMM:(double)apex worldHalfWidthMouthMM:(double)mouth
    expected:(Core3DSceneSnapshot *)expected completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginWedgeCutWidths(_:operandIdentifier:worldHalfWidthApexMM:worldHalfWidthMouthMM:expected:completion:));
- (nullable Core3DCylindricalCutOperation *)beginWedgeCutLength:(Core3DCylindricalCutProgramSnapshot *)original
    operandIdentifier:(uint32_t)identifier worldLengthMM:(double)length expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginWedgeCutLength(_:operandIdentifier:worldLengthMM:expected:completion:));
//! One ring operand or one addressed ring edit, one ordinary history step.
- (nullable Core3DCylindricalCutOperation *)beginCylindricalCutAppendRing:(Core3DCylindricalCutProgramSnapshot *)original
    definition:(Core3DCylindricalCutRingDefinition *)definition expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginCylindricalCutAppendRing(_:definition:expected:completion:));
- (nullable Core3DCylindricalCutOperation *)beginCylindricalCutRingRadius:(Core3DCylindricalCutProgramSnapshot *)original
    operandIdentifier:(uint32_t)identifier worldHoleRadiusMM:(double)radius expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginCylindricalCutRingRadius(_:operandIdentifier:worldHoleRadiusMM:expected:completion:));
//! Descriptive admission category for the last synchronous ring/wedge refusal.
//! Never an edit permit; absent for stale authority or an unclassified failure.
@property(nonatomic,copy,readonly,nullable) NSString *lastCylindricalCutAdmissionReason;
- (nullable Core3DCylindricalCutOperation *)beginCylindricalCutRingBoltRadius:(Core3DCylindricalCutProgramSnapshot *)original
    operandIdentifier:(uint32_t)identifier worldBoltRadiusMM:(double)radius expected:(Core3DSceneSnapshot *)expected
    completion:(void (^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginCylindricalCutRingBoltRadius(_:operandIdentifier:worldBoltRadiusMM:expected:completion:));
- (nullable Core3DCylindricalCutOperation *)beginCylindricalCutRingCount:(Core3DCylindricalCutProgramSnapshot *)original
    operandIdentifier:(uint32_t)identifier count:(uint32_t)count expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginCylindricalCutRingCount(_:operandIdentifier:count:expected:completion:));
//! Native source edit; no UI/AI activation. Main callback may refuse synchronously.
- (nullable Core3DSavedCutSourceOperation *)beginSavedCutSourceEdit:(Core3DCylindricalCutSnapshot *)original
    patch:(Core3DSavedCutSourcePatch *)patch expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginSavedCutSourceEdit(_:patch:expected:completion:));
//! Explicit whole-program source edit of a retained multi-bore cut: the shared
//! polygon/enclosure source dimensions change while every bore's stable ID,
//! order, raw axis/center/radius, high-water ID, identity, units, placement,
//! name and scalar material are preserved. Native lease only; no UI/touch/AI
//! catalog route. Main callback may refuse synchronously; legacy API unchanged.
- (nullable Core3DSavedCutSourceOperation *)beginSavedProgramCutSourceEdit:(Core3DCylindricalCutProgramSnapshot *)original
    patch:(Core3DSavedCutSourcePatch *)patch expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginSavedProgramCutSourceEdit(_:patch:expected:completion:));
#if DEBUG
//! One-shot main delivery gate for real worker lifecycle qualification.
- (void)debugSetSavedCutSourceDeliveryGate:(void (^_Nullable)(void (^resume)(void)))gate
    NS_SWIFT_NAME(debugSetSavedCutSourceDeliveryGate(_:));
//! Holds the next native solid's completed geometry at its main-thread delivery boundary.
- (void)debugSetNextNativeSolidDeliveryGate:(void (^_Nullable)(void (^resume)(void)))gate
    NS_SWIFT_NAME(debugSetNextNativeSolidDeliveryGate(_:));
+ (NSDictionary<NSString *,NSNumber *> *)debugSavedCutSourceMMConversionProbe;
#endif
- (nullable Core3DStoredRectangularLoftSnapshot *)storedRectangularLoftWithEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(storedRectangularLoft(entityIdentifier:expected:));
- (void)rebuildStoredRectangularLoft:(Core3DStoredRectangularLoftSnapshot *)original
    edit:(Core3DRectangularLoftStationEdit *)edit expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult result))completion
    NS_SWIFT_NAME(rebuildStoredRectangularLoft(_:edit:expected:completion:));
//! Same ordinary edit, with an exact-work cancellation handle. Nil means no
//! asynchronous work was admitted; completion still reports the actual refusal.
- (nullable Core3DStoredLoftEditOperation *)beginStoredRectangularLoftEdit:(Core3DStoredRectangularLoftSnapshot *)original
    edit:(Core3DRectangularLoftStationEdit *)edit expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginStoredRectangularLoftEdit(_:edit:expected:completion:));
- (void)createRectangularLoftWithDefinition:(Core3DRectangularLoftDefinition *)definition
    expected:(Core3DSceneSnapshot *)expected completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(createRectangularLoft(definition:expected:completion:));
//! Same ordinary loft creation, with an exact-work cancellation handle. Nil
//! means no asynchronous work was admitted; completion still reports the actual
//! refusal. The handle can never execute, replace, retry or cancel a later job.
- (nullable Core3DStoredLoftEditOperation *)beginRectangularLoftCreateWithDefinition:(Core3DRectangularLoftDefinition *)definition
    expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginRectangularLoftCreate(definition:expected:completion:));
- (void)createSweepWithDefinition:(Core3DSweepDefinition *)definition
    expected:(Core3DSceneSnapshot *)expected completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(createSweep(definition:expected:completion:));

- (void)createProfileWithDefinition:(Core3DProfileDefinition *)definition
    expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(createProfile(definition:expected:completion:));
//! Create or edit the native enclosure dependency using the same worker slot,
//! cancellation and ordinary history as profile construction. Units must match
//! the document; rebuilding preserves placement and native entity identity.
- (void)createEnclosureWithDefinition:(Core3DEnclosureDefinition *)definition
    expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(createEnclosure(definition:expected:completion:));
- (nullable Core3DStoredEnclosureSnapshot *)storedEnclosureWithEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(storedEnclosure(entityIdentifier:expected:));
- (void)rebuildStoredEnclosure:(Core3DStoredEnclosureSnapshot *)original
    definition:(Core3DEnclosureDefinition *)definition expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(rebuildStoredEnclosure(_:definition:expected:completion:));
//! Prepare passive Object/MoveRotate/Scale state for a separate planning
//! capture. Preserves model and selected targets; rejects active work, gestures,
//! modeling previews, readonly views and recovery without cancelling them.
- (BOOL)prepareForModelingPlanning NS_SWIFT_NAME(prepareForModelingPlanning());
//! One committed AI history step, with selection/detection retired before OCAF
//! publishes its synchronous change notification. Refuses previews or busy work.
//! Returns YES only for one verified ordinary Undo on the same document.
- (BOOL)undoCommittedModelingStep NS_SWIFT_NAME(undoCommittedModelingStep());
//! Capture supersedes the previous unused lease. Camera changes alone remain
//! valid; any observed semantic selection/tool or native edit/history boundary
//! invalidates it, even if geometry or selection later returns to the same value.
- (nullable Core3DModelingPlanningContext *)captureModelingPlanningContext;
//! Additive plain-profile shell route. Capture is read-only and uses the exact
//! selected target and planning lease. Prepare consumes that lease once and
//! runs the existing bounded Shell preview. Poll state until Ready; Apply and
//! Cancel retain the native controller on unknown/retryable outcomes. Never
//! replay an unknown result. A retired handle cannot act on later touch work.
- (nullable Core3DShellOperation *)captureModelingShellWithOpenings:(NSArray<Core3DShellOpeningSelector *> *)openings
    context:(Core3DModelingPlanningContext *)context NS_SWIFT_NAME(captureModelingShell(openings:context:));
- (Core3DShellOperationState)prepareModelingShell:(Core3DShellOperation *)operation thicknessMM:(double)thickness
    NS_SWIFT_NAME(prepareModelingShell(_:thicknessMM:));
- (Core3DShellOperationState)modelingShellState:(Core3DShellOperation *)operation NS_SWIFT_NAME(modelingShellState(_:));
- (Core3DShellOperationState)applyModelingShell:(Core3DShellOperation *)operation NS_SWIFT_NAME(applyModelingShell(_:));
- (Core3DShellOperationState)cancelModelingShell:(Core3DShellOperation *)operation NS_SWIFT_NAME(cancelModelingShell(_:));
// Additive opt-in capture. Existing capture/advertised catalogs remain unchanged.
- (nullable Core3DModelingPlanningContext *)captureModelingPlanningContextIncludingSavedCutSource;
- (nullable Core3DSavedCutSourceOperation *)beginModelingSavedCutSourceEdit:(Core3DSavedCutSourcePatch *)patch
    context:(Core3DModelingPlanningContext *)context completion:(void (^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(beginModelingSavedCutSourceEdit(patch:context:completion:));
//! Composite original planning+inspector authority. No recapture after an await.
//! Public async dispatch is intentionally unavailable for this operation.
- (nullable Core3DModelingPlanningContext *)captureModelingPlacementContext;
- (nullable Core3DModelingPreparedRequest *)preparePlacementRequestWithValue:(double)value
    kind:(Core3DRigidPlacementKind)kind axis:(Core3DTransformInspectorAxis)axis
    context:(Core3DModelingPlanningContext *)context session:(Core3DModelingHostSession *)session requestID:(NSUUID *)requestID
    NS_SWIFT_NAME(preparePlacementRequest(value:kind:axis:context:session:requestID:));
//! Stage A read-only preparation; existing native lease/unit/recipe validators
//! stay authoritative. The host session must be currently installed on main.
- (nullable Core3DModelingPreparedRequest *)prepareEnclosureRequest:(Core3DEnclosureDefinition *)definition
    rebuild:(BOOL)rebuild context:(Core3DModelingPlanningContext *)context
    session:(Core3DModelingHostSession *)session requestID:(NSUUID *)requestID
    NS_SWIFT_NAME(prepareEnclosureRequest(definition:rebuild:context:session:requestID:));
- (nullable Core3DModelingPreparedRequest *)prepareProfileRequest:(Core3DProfileDefinition *)definition
    rebuild:(BOOL)rebuild context:(Core3DModelingPlanningContext *)context
    session:(Core3DModelingHostSession *)session requestID:(NSUUID *)requestID
    NS_SWIFT_NAME(prepareProfileRequest(definition:rebuild:context:session:requestID:));
//! Read-only selected-loft request. Public async execution remains unsupported.
//! The original selected recipe, unit, IDs, frame and exact effect are bound.
- (nullable Core3DModelingPreparedRequest *)prepareLoftStationRequest:(Core3DRectangularLoftStationEdit *)edit
    context:(Core3DModelingPlanningContext *)context session:(Core3DModelingHostSession *)session
    requestID:(NSUUID *)requestID NS_SWIFT_NAME(prepareLoftStationRequest(edit:context:session:requestID:));
- (nullable Core3DModelingPreparedRequest *)prepareAssemblyRequest:(NSArray<Core3DAssemblyPartDefinition *> *)parts
    context:(Core3DModelingPlanningContext *)context session:(Core3DModelingHostSession *)session
    requestID:(NSUUID *)requestID NS_SWIFT_NAME(prepareAssemblyRequest(parts:context:session:requestID:));
- (BOOL)isModelingPreparedRequestCurrent:(Core3DModelingPreparedRequest *)request
    NS_SWIFT_NAME(isModelingPreparedRequestCurrent(_:));
- (void)stopModelingPreparedRequest:(Core3DModelingPreparedRequest *)request
    NS_SWIFT_NAME(stopModelingPreparedRequest(_:));
//! Intentionally unavailable until durable reservation/ordinary coupling is
//! qualified. Does not consume context, reserve storage or change history.
- (Core3DModelingRequestExecutionResult)executeModelingPreparedRequest:(Core3DModelingPreparedRequest *)request
    NS_SWIFT_NAME(executeModelingPreparedRequest(_:));
//! Main-owned async enclosure/assembly creation or exact selected-loft station rebuild.
//! Exactly one main result for a nonnull completion; other operations remain unsupported.
//! Unchanged proves only this invocation opened no command/receipt/history step.
//! First actual permanent reservation is required. No fallback or replay.
- (void)executeModelingPreparedRequest:(nullable Core3DModelingPreparedRequest *)request
    completion:(void (^)(Core3DModelingAsyncOutcome *))completion
    NS_SWIFT_NAME(executeModelingPreparedRequest(_:completion:));


- (BOOL)isModelingPlanningContextCurrent:(Core3DModelingPlanningContext *)context
    NS_SWIFT_NAME(isModelingPlanningContextCurrent(_:));
//! Idempotently retires this exact lease. Stops only construction owned by it;
//! unrelated later touch work is never cancelled. Worker completion still drains.
- (void)retireModelingPlanningContext:(Core3DModelingPlanningContext *)context
    NS_SWIFT_NAME(retireModelingPlanningContext(_:));
//! One logical native command/Undo. Rejected includes foreign, stale, consumed,
//! malformed and unavailable authority. No automatic retry after consumption:
//! durable request receipts/crash reconciliation are not provided by this API.
- (void)createEnclosureWithDefinition:(Core3DEnclosureDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(createEnclosure(definition:context:completion:));
//! Create one stored profile through existing polygon/circle/line-arc
//! Extrude/Revolve admission. Unit must exactly match the captured document;
//! no caller construction frame is admitted. Selected-profile rebuild limits
//! do not narrow this creation API. Same one-use lease/Stop/history semantics.
- (void)createProfileWithDefinition:(Core3DProfileDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(createProfile(definition:context:completion:));
//! Stage a profile and its authored name in one ordinary creation command.
//! Geometry and placement are identical to the unnamed profile creator.
- (void)createProfileWithDefinition:(Core3DProfileDefinition *)definition name:(NSString *)name
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(createProfile(definition:name:context:completion:));
//! Target is exclusively the selected enclosure captured by this exact lease.
//! Definition must preserve its construction frame and declared native unit.
- (void)rebuildEnclosureWithDefinition:(Core3DEnclosureDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(rebuildEnclosure(definition:context:completion:));
//! Rebuild the exact selected canonical box/cylinder profile from its original
//! planning lease. Primitive kind and plane remain unchanged; native opening
//! construction frame and authored placement are retained by ordinary history.
- (void)rebuildProfileWithDefinition:(Core3DProfileDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(rebuildProfile(definition:context:completion:));
//! Validates one station's physical dimensions against the ORIGINAL current lease.
//! nil dimensions preserve original raw bits; at least one dimension is required.
//! Returns numeric data only; does not consume the lease, build, mutate or reserve.
//! Future reserved loft execution still requires its own descriptor/effect coupling.
- (nullable Core3DRectangularLoftStationEdit *)modelingLoftStationEdit:(uint32_t)stationIdentifier
    widthMM:(nullable NSNumber *)width depthMM:(nullable NSNumber *)depth
    context:(Core3DModelingPlanningContext *)context
    NS_SWIFT_NAME(modelingLoftStationEdit(stationIdentifier:widthMM:depthMM:context:));
//! Ordinary native context operations only: no reserved request/receipt tag is implied.
- (void)createSweepWithDefinition:(Core3DSweepDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult result))completion
    NS_SWIFT_NAME(createSweep(definition:context:completion:));
- (void)rebuildSweepRadiusWithDefinition:(Core3DSweepDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult result))completion
    NS_SWIFT_NAME(rebuildSweepRadius(definition:context:completion:));
//! Detached numeric helpers only; neither acquires document or history authority.
//! Result order: trimA x/y, trimB x/y, center x/y, radius, start/sweep degrees.
+ (nullable NSArray<NSNumber *> *)sweepTangentArcFrom:(CGPoint)a corner:(CGPoint)corner
    to:(CGPoint)b radius:(double)radius NS_SWIFT_NAME(sweepTangentArc(from:corner:to:radius:));
//! Returns the native admission name for the complete candidate, including invalid paths.
+ (NSString *)sweepAdmissionWithPathIdentifier:(uint32_t)identifier
    vertices:(NSArray<Core3DProfileCurveVertex *> *)vertices
    segments:(NSArray<Core3DSweepPathSegment *> *)segments plane:(Core3DProfilePlane)plane
    radius:(double)radius metersPerUnit:(double)metersPerUnit constructionFrameValues:(NSArray<NSNumber *> *)frame
    NS_SWIFT_NAME(sweepAdmission(pathIdentifier:vertices:segments:plane:radius:metersPerUnit:constructionFrameValues:));
+ (NSString *)sweepAdmissionWithPathIdentifier:(uint32_t)identifier
    vertices:(NSArray<Core3DProfileCurveVertex *> *)vertices
    segments:(NSArray<Core3DSweepPathSegment *> *)segments plane:(Core3DProfilePlane)plane
    radius:(double)radius endRadius:(double)endRadius metersPerUnit:(double)metersPerUnit constructionFrameValues:(NSArray<NSNumber *> *)frame
    NS_SWIFT_NAME(sweepAdmission(pathIdentifier:vertices:segments:plane:radius:endRadius:metersPerUnit:constructionFrameValues:));
//! Fixed saved path structure with fully re-admitted geometry; one SweepRebuild.
- (void)rebuildSweepPathWithDefinition:(Core3DSweepDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult result))completion
    NS_SWIFT_NAME(rebuildSweepPath(definition:context:completion:));
//! One original planning lease; only depth/angle may differ from its exact
//! saved recipe. Delegates to the ordinary profile worker/history path.
- (void)rebuildProfileRecipeWithDefinition:(Core3DProfileDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(rebuildProfileRecipe(definition:context:completion:));
//! One flat instruction, 1–16 separate named profile solids and one Undo.
//! Existing typed profile tolerances apply in document units. No fusion or
//! nested hierarchy is inferred. Stop retires the exact planning context.
- (void)createAssemblyWithParts:(NSArray<Core3DAssemblyPartDefinition *> *)parts
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(createAssembly(parts:context:completion:));
//! Cancels any native construction kind; admission stays Busy until completion.
- (void)cancelNativeConstruction;
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
//! Change the extrusion depth or revolution angle of the selected saved profile.
//! Lengths use the document's declared units; angles are degrees. Retains the
//! same entity, outline, holes and placement. Stale geometry bindings reject.
- (void)rebuildStoredProfileWithEntityIdentifier:(NSString *)entityIdentifier
                                     parameter:(double)parameter
                                      expected:(Core3DSceneSnapshot *)expected
                                    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(rebuildStoredProfile(entityIdentifier:parameter:expected:completion:));

- (nullable Core3DStoredProfileSnapshot *)storedProfileWithEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(storedProfile(entityIdentifier:expected:));
- (void)rebuildStoredProfile:(Core3DStoredProfileSnapshot *)original
    definition:(Core3DProfileDefinition *)definition expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(rebuildStoredProfile(_:definition:expected:completion:));
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

//! Freeze one bounded saved-recipe or unparametrized-solid placement intent against the original
//! inspector lease. Execution may still refuse stale/busy authority. Does not
//! reserve, refresh selection, mutate geometry or open history. Main only.
- (nullable Core3DRigidPlacementPreparation *)prepareRigidPlacementValue:(double)value
    kind:(Core3DRigidPlacementKind)kind axis:(Core3DTransformInspectorAxis)axis
    expected:(Core3DTransformInspectorSnapshot *)snapshot
    NS_SWIFT_NAME(prepareRigidPlacement(value:kind:axis:expected:));
//! One use, including stale/busy/failure. Exact native ordinary result only.
- (Core3DTransformInspectorPositionCommitResult)executeRigidPlacement:(Core3DRigidPlacementPreparation *)prepared
    NS_SWIFT_NAME(executeRigidPlacement(_:));
//! Retire only this preparation, never a newer inspector lease. Main only.
- (void)cancelRigidPlacement:(Core3DRigidPlacementPreparation *)prepared
    NS_SWIFT_NAME(cancelRigidPlacement(_:));
#if DEBUG
//! Component evidence only; no native verified receipt or retry permission.
- (BOOL)debugSetCutDisplayCoefficient:(double)coefficient pending:(BOOL)pending NS_SWIFT_NAME(debugSetCutDisplayCoefficient(_:pending:));
// DEBUG fixture dispatch only; no product Objective-C source-edit operation.
- (NSDictionary<NSString *,NSNumber *> *)debugSavedCutSourceViewerQualification:(Core3DCylindricalCutSnapshot *)original
    expected:(Core3DSceneSnapshot *)expected NS_SWIFT_NAME(debugSavedCutSourceViewerQualification(_:expected:));
- (NSDictionary<NSString *,id> *)debugSavedCutSourceFixtureEdit:(NSInteger)fixture valueMM:(double)valueMM depthMM:(double)depthMM
    original:(Core3DCylindricalCutSnapshot *)original expected:(Core3DSceneSnapshot *)expected
    cancelPoint:(NSInteger)cancelPoint beforeCommit:(nullable void (^)(void))beforeCommit
    NS_SWIFT_NAME(debugSavedCutSourceFixtureEdit(_:valueMM:depthMM:original:expected:cancelPoint:beforeCommit:));
- (NSDictionary<NSString *,NSNumber *> *)debugSavedCutBracketGeometry:(NSString *)entity lengthMM:(double)lengthMM depthMM:(double)depthMM
    NS_SWIFT_NAME(debugSavedCutBracketGeometry(_:lengthMM:depthMM:));
- (NSDictionary<NSString *,id> *)debugSavedCutSourceEnclosureWidth:(double)widthMM
    original:(Core3DCylindricalCutSnapshot *)original expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(debugSavedCutSourceEnclosureWidth(_:original:expected:));
- (NSDictionary<NSString *,NSNumber *> *)debugSavedCutEnclosureGeometry:(NSString *)entity widthMM:(double)widthMM
    NS_SWIFT_NAME(debugSavedCutEnclosureGeometry(_:widthMM:));
- (nullable NSDictionary<NSString *,id> *)debugCylindricalCutEvidence:(NSString *)entity NS_SWIFT_NAME(debugCylindricalCutEvidence(_:));
//! Read-only whole-program observation: complete recipe bytes, stable bore
//! values and independent geometry evidence. No edit or selection authority.
- (nullable NSDictionary<NSString *,id> *)debugCylindricalCutProgramEvidence:(NSString *)entity NS_SWIFT_NAME(debugCylindricalCutProgramEvidence(_:));
//! Bounded DEBUG read-only observation of the ACTUAL BRep cylindrical hole
//! faces: kernel-measured axis/location/radius/trim extents/orientation plus
//! topology validity, independent of any stored recipe value; the production
//! program matcher is not called. Refuses with an open or unresolved command.
//! No selection or edit authority is acquired or altered.
- (nullable NSDictionary<NSString *,id> *)debugCylindricalCutHoleFaces:(NSString *)entity NS_SWIFT_NAME(debugCylindricalCutHoleFaces(_:));
+ (NSDictionary<NSString *,NSNumber *> *)debugCylindricalCutSimilarityProbe;
+ (NSDictionary<NSString *,NSNumber *> *)debugNativeDocumentSessionLifecycleProbe;
- (NSDictionary<NSString *,NSNumber *> *)debugCutSceneGuardMutation:(NSInteger)mode target:(NSString *)target sibling:(NSString *)sibling;
- (nullable NSDictionary *)debugRigidPlacementEvidence:(NSString *)entity
    NS_SWIFT_NAME(debugRigidPlacementEvidence(_:));
- (nullable NSDictionary *)debugRigidPlacementAdmissionProbe
    NS_SWIFT_NAME(debugRigidPlacementAdmissionProbe());
//! Bounded DEBUG read-only kernel evidence for the ACTUAL retained BRep of one
//! saved entity: BRepCheck validity, unique topology counts, exact analytic
//! world bounds (AddOptimal, no triangulation, no shape tolerance) and
//! analytic world volume, composed with the persisted transform exactly as
//! production object-alignment measurement does. Ordinary capture only parses
//! stored recipe records; it never checks recipe currency, so a stale recipe
//! (e.g. a filleted seat) stays observable. Refuses off-main, busy, open or
//! unresolved edit, invalid unit, non-BRep or excessive topology with nil.
- (nullable NSDictionary<NSString *,id> *)debugNativeSolidEvidence:(NSString *)entity
    NS_SWIFT_NAME(debugNativeSolidEvidence(_:));
//! Signed enter/exit distances in mm along the normalized world direction from
//! originMM. Ordered material intervals on the whole line (not a ray); [] means
//! no material. Selection independent. Same admission/placement as the solid
//! observer. nil for unknown entity, nonfinite/zero input, ambiguous boundary,
//! non-solid geometry or excessive work (8192 nodes, 4096 hits, 64 intervals).
- (nullable NSArray<NSArray<NSNumber *> *> *)debugNativeSolidIntervals:(NSString *)entity
    originMM:(NSArray<NSNumber *> *)originMM direction:(NSArray<NSNumber *> *)direction
    NS_SWIFT_NAME(debugNativeSolidIntervals(_:originMM:direction:));
//! Additive exact-boundary observer. Returns the one-sided material limit of
//! L(t) + epsilon * approachDirection as positive epsilon tends to zero,
//! without moving L itself. Both directions must be finite/nonzero and the
//! approach must be transverse to L. Distances and normalization are in mm,
//! as above. [] is a measured empty section; nil means the boundary support,
//! input, solid or bounded work could not be resolved safely.
- (nullable NSArray<NSArray<NSNumber *> *> *)debugNativeSolidBoundaryIntervals:(NSString *)entity
    originMM:(NSArray<NSNumber *> *)originMM direction:(NSArray<NSNumber *> *)direction
    approachDirection:(NSArray<NSNumber *> *)approachDirection
    NS_SWIFT_NAME(debugNativeSolidBoundaryIntervals(_:originMM:direction:approachDirection:));
//! Detached bounded controls for boundary-only pathologies that cannot be
//! admitted into a document. No document, selection or history is touched.
+ (NSDictionary<NSString *,NSNumber *> *)debugNativeSolidBoundaryAdmissionProbe;
#endif

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
- (BOOL)debugConfigureMeshCopyFault:(NSInteger)mode;
- (BOOL)debugProbeMeshVertexStorageChange:(NSInteger)mode;
- (BOOL)debugProbeMeshUVRepackStorageRefusal:(NSInteger)mode;
- (BOOL)debugConfigureOrdinaryGestureFault:(NSInteger)mode;
- (BOOL)debugConfigureSavedGroupOriginPostStageFault:(NSInteger)memberCount;
- (NSArray<NSNumber *> *_Nullable)debugSavedGroupOrigin:(NSString *)identifier;
//! Exact current attached manipulator location; nil when no gizmo is attached.
//! Read-only DEBUG evidence for saved-group pivot attachment paths.
- (NSArray<NSNumber *> *_Nullable)debugAttachedManipulatorPosition;
- (BOOL)debugConfigureOrdinaryNameFault:(NSInteger)mode;
- (BOOL)debugConfigureOrdinaryVisibilityFault:(NSInteger)mode;
- (void)debugSetViewerOrdinaryVisibilityAfterRepairFailures:(NSInteger)count;
- (void)debugSetViewerOrdinaryRepairFailures:(NSInteger)incremental redraw:(NSInteger)redraw;
- (void)debugSetSavedGroupPivotFailures:(NSInteger)count;
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
//! Deliberately clear selection behind a Ready axis-preview ledger, proving
//! camera admission rejects source/selection drift without changing recovery.
- (BOOL)debugClearReadyMirrorSourceSelection;
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
- (void)debugSetLinearArrayProfileCopyFault:(NSInteger)mode;
- (void)debugSetMirrorProfileCopyFault:(NSInteger)mode;
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
- (void)debugSetRadialArrayProfileCopyFault:(NSInteger)mode;
- (void)debugSetMaximumRadialArrayTopologyNodes:(NSUInteger)limit;
- (BOOL)debugMutateRadialArraySourcePersistedTransform;
//! Reference-edit CommitCommand modes: 0 normal, 1 false-after-close,
//! 2 throw-after-close.
- (void)debugSetRadialArrayReferenceEditCommitMode:(NSInteger)mode;
//! Read-only identity proof over every committed free-simple definition.
//! Pairwise shape partnership and TriangleMesh triangulation handle identity
//! are intentionally unavailable in renderer-neutral scene snapshots.
- (NSDictionary<NSString *, NSNumber *> *)debugGeometryCopyIndependenceState;
//! DEBUG-only detached numeric intersection probes, never document authority.
//! Pair data is18 little-endian binary64 coordinates per pair, no expected bytes.
- (NSData *_Nullable)debugTriangleContactPairDecisions:(NSData *)coordinates;
//! Whole mesh data is9 little-endian binary64 coordinates per triangle.
- (NSDictionary<NSString *, id> *_Nullable)debugTriangleContactMesh:(NSData *)coordinates mode:(NSInteger)mode;

//! Isolated native winding geometry probes; no live document writes.
- (NSDictionary<NSString *, NSArray<NSNumber *> *> *_Nullable)debugWindingGeometryProbe;
- (NSDictionary<NSString *, id> *_Nullable)debugMeshRegionInsetKernelProbe;
- (NSDictionary<NSString *, id> *_Nullable)debugMeshPartitionVertexMoveKernelProbe;
//! Passive partition-persistence qualification only. These DEBUG seams do not
//! expose Inset or publish partition records in product operation paths.
- (NSDictionary<NSString *, NSNumber *> *_Nullable)debugMeshRegionPartitionState:(NSString *)entityIdentifier;
- (NSData *_Nullable)debugMeshRegionPartitionBytes:(NSString *)entityIdentifier;
- (BOOL)debugStageFirstMeshRegionPartition:(NSString *)entityIdentifier;
- (BOOL)debugClearMeshRegionPartition:(NSString *)entityIdentifier;
- (BOOL)debugCorruptMeshRegionPartition:(NSString *)entityIdentifier mode:(NSInteger)mode;
- (NSData *_Nullable)debugCorruptMeshRegionPartitionArchive:(NSString *)entityIdentifier mode:(NSInteger)mode;
//! Test-only exact-label cycle seam; empty begin and real detected-state transitions.
- (BOOL)debugBeginEmptyBooleanWithGizmoType:(PrimitiveGizmoType)gizmoType
    NS_SWIFT_NAME(debugBeginEmptyBoolean(gizmoType:));
- (BOOL)debugCycleBooleanSelectionForEntityIdentifier:(NSString *)entityIdentifier
    NS_SWIFT_NAME(debugCycleBooleanSelection(entityIdentifier:));
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
- (void)debugSetBooleanMetadataFailurePhase:(NSUInteger)phase;
//! Make the next N Boolean transaction abort attempts fail before touching it.
- (void)debugSetBooleanAbortFailureCount:(NSUInteger)count;
//! Make the next N post-commit Boolean document inspections unavailable.
- (void)debugSetBooleanPostCommitInspectFailureCount:(NSUInteger)count;
//! Deliver the viewport's production memory-warning cancellation path.
- (void)debugSimulateBooleanMemoryWarning;
//! Deliver the viewport memory-warning path for an active Mirror regression.
- (void)debugSimulateMirrorMemoryWarning;
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
- (BOOL)debugBeginShellWithEntityIdentifier:(NSString *)entityIdentifier
                        faceTopologyIndices:(NSArray<NSNumber *> *)faceTopologyIndices
    NS_SWIFT_NAME(debugBeginShell(entityIdentifier:faceTopologyIndices:));
- (NSArray<NSNumber *> *)debugShellOpeningFaceTopologyIndices;
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
- (NSData *_Nullable)debugLegacyNoLengthUnitMirrorBinXCAFFixtureData;
- (NSDictionary *_Nullable)debugNativeLegacyReceiptFixture:(NSInteger)kind policy:(NSInteger)policy
    NS_SWIFT_NAME(debugNativeLegacyReceiptFixture(_:policy:));
//! Actual isolated OCCT reader/writer framing fixtures, not receipt execution.
+ (NSDictionary<NSString *, NSNumber *> *)debugReceiptFramingProbe:(NSInteger)scenario
    NS_SWIFT_NAME(debugReceiptFramingProbe(_:));
+ (NSDictionary<NSString *, NSNumber *> *)debugRetainedSolidProbe:(NSInteger)scenario
    NS_SWIFT_NAME(debugRetainedSolidProbe(_:));
//! Native/value prerequisite probes only; no document command authority.
+ (NSDictionary<NSString *, NSNumber *> *)debugSavedCutSourcePrerequisiteProbe:(NSInteger)scenario
    NS_SWIFT_NAME(debugSavedCutSourcePrerequisiteProbe(_:));
//! Detached bounded enclosure correspondence qualification, not source editing.
+ (NSDictionary<NSString *, NSNumber *> *)debugEnclosureCorrespondenceProbe:(NSInteger)scenario
    NS_SWIFT_NAME(debugEnclosureCorrespondenceProbe(_:));
//! Read-only recipe/interval qualification, not a native edit capability.
+ (NSDictionary<NSString *, NSNumber *> *)debugSavedCutBoreClearanceProbe:(NSInteger)scenario
    NS_SWIFT_NAME(debugSavedCutBoreClearanceProbe(_:));
//! Read-only detached observer/whole-result native probes.
//! Last cut create/edit refusal gate; empty after a successful operation.
- (NSString *)debugLastCylindricalCutRefusalReason NS_SWIFT_NAME(debugLastCylindricalCutRefusalReason());
+ (NSDictionary<NSString *, NSNumber *> *)debugLoftCutProofProbe NS_SWIFT_NAME(debugLoftCutProofProbe());
- (NSDictionary<NSString *, NSNumber *> *)debugSavedBooleanWedgeGeometry:(NSString *)entity
    NS_SWIFT_NAME(debugSavedBooleanWedgeGeometry(_:));
+ (NSDictionary<NSString *, NSNumber *> *)debugSavedBooleanFilletProbe:(NSInteger)scenario NS_SWIFT_NAME(debugSavedBooleanFilletProbe(_:));
+ (void)debugSetRetainedFilletFailureCount:(NSInteger)count NS_SWIFT_NAME(debugSetRetainedFilletFailureCount(_:));
//! Pure DEBUG codec/history evidence; never observes or mutates the live document.
+ (NSDictionary<NSString *, NSNumber *> *)debugPartBooleanCodecProbe
    NS_SWIFT_NAME(debugPartBooleanCodecProbe());
+ (NSDictionary<NSString *, NSNumber *> *)debugSavedBooleanWedgeProbe:(NSInteger)scenario
    NS_SWIFT_NAME(debugSavedBooleanWedgeProbe(_:));
+ (NSDictionary<NSString *, NSNumber *> *)debugSavedBooleanRingProbe:(NSInteger)scenario
    NS_SWIFT_NAME(debugSavedBooleanRingProbe(_:));
- (NSDictionary<NSString *, NSNumber *> *)debugSavedBooleanRingGeometry:(NSString *)entity
    NS_SWIFT_NAME(debugSavedBooleanRingGeometry(_:));
+ (NSDictionary<NSString *, NSNumber *> *)debugCircularHostProofProbe:(NSInteger)scenario
    NS_SWIFT_NAME(debugCircularHostProofProbe(_:));
- (NSDictionary<NSString *,NSNumber *> *)debugCircularHostGeometry:(NSString *)entity
    outerMM:(double)outerMM innerMM:(double)innerMM depthMM:(double)depthMM
    NS_SWIFT_NAME(debugCircularHostGeometry(_:outerMM:innerMM:depthMM:));
+ (NSDictionary<NSString *, NSNumber *> *)debugSavedCutResultCorrespondenceProbe:(NSInteger)scenario
    NS_SWIFT_NAME(debugSavedCutResultCorrespondenceProbe(_:));
+ (NSDictionary<NSString *, NSNumber *> *)debugSavedBooleanProgramProbe
    NS_SWIFT_NAME(debugSavedBooleanProgramProbe());
//! Detached A1a evidence only; reports that the document command stayed closed.
- (NSDictionary<NSString *, NSNumber *> *)debugRetainedPartBooleanProbe
    NS_SWIFT_NAME(debugRetainedPartBooleanProbe());
//! Detached A3 P1b evidence only. Scenario 0 is S/S2/Lug boundary evidence;
//! scenario 1 is N->N' split/merge correspondence. No route or mutation.
- (NSDictionary<NSString *, NSNumber *> *)debugA3ShellBooleanEvidence:(NSInteger)scenario
    NS_SWIFT_NAME(debugA3ShellBooleanEvidence(_:));
//! Detached DEBUG trim-domain qualification only.
+ (NSDictionary<NSString *, NSNumber *> *)debugSavedCutTrimDomainProbe
    NS_SWIFT_NAME(debugSavedCutTrimDomainProbe());
- (NSDictionary *)debugReceiptCatalogSnapshot NS_SWIFT_NAME(debugReceiptCatalogSnapshot());
- (NSDictionary *)debugScalableReceiptSnapshot NS_SWIFT_NAME(debugScalableReceiptSnapshot());
- (NSDictionary *)debugScalableReceiptProbe:(NSInteger)scenario NS_SWIFT_NAME(debugScalableReceiptProbe(_:));
- (NSDictionary *)debugReceiptWire:(NSData *)data NS_SWIFT_NAME(debugReceiptWire(_:));
- (NSDictionary *_Nullable)debugReceiptDualCatalogProbe NS_SWIFT_NAME(debugReceiptDualCatalogProbe());
//! Synthetic component evidence only; no production reservation or verified result.
- (NSDictionary *_Nullable)debugLoftReceiptFixture:(NSInteger)variant metersPerUnit:(double)unit
    NS_SWIFT_NAME(debugLoftReceiptFixture(_:metersPerUnit:));
+ (NSDictionary *)debugLoftRequestCodec NS_SWIFT_NAME(debugLoftRequestCodec());
- (NSDictionary *_Nullable)debugNativeReceiptFixture:(NSInteger)kind
    NS_SWIFT_NAME(debugNativeReceiptFixture(_:));
/// DEBUG stream oracle only; no native geometry authority is issued from bytes.
- (NSDictionary *)debugReceiptGeometryStream:(NSData *)data chunkSize:(NSUInteger)chunkSize
    NS_SWIFT_NAME(debugReceiptGeometryStream(_:chunkSize:));
- (NSDictionary *)debugInspectNativeReceipt:(NSString *)requestID conflict:(BOOL)conflict
    NS_SWIFT_NAME(debugInspectNativeReceipt(_:conflict:));
- (void)debugNativeTombstoneProbe:(NSInteger)scenario completion:(void (^)(NSDictionary *result))completion
    NS_SWIFT_NAME(debugNativeTombstoneProbe(_:completion:));
- (BOOL)debugNativeTombstoneRejectsMainThread;
- (NSDictionary *)debugInspectModelingDescriptorData:(NSData *)data
    NS_SWIFT_NAME(debugInspectModelingDescriptorData(_:));
- (NSDictionary *)debugModelingAdmissionStateProbe NS_SWIFT_NAME(debugModelingAdmissionStateProbe());
- (nullable NSData *)debugModelingPreparedDescriptor:(Core3DModelingPreparedRequest *)request
    NS_SWIFT_NAME(debugModelingPreparedDescriptor(_:));

/// Creation coupling qualification only; requires an actual first-reserved private test capability.
- (Core3DTransformInspectorPositionCommitResult)debugExecuteReservedPlacement:(Core3DModelingPreparedRequest *)request
    receiptFailure:(BOOL)receiptFailure NS_SWIFT_NAME(debugExecuteReservedPlacement(_:receiptFailure:));
- (NSDictionary *)debugPlacementReceiptState:(Core3DModelingPreparedRequest *)request;
- (NSDictionary *)debugPlacementRequestCodecProbe;
- (BOOL)debugExecuteReservedCreation:(Core3DModelingPreparedRequest *)request receiptFailure:(BOOL)receiptFailure
    completion:(void (^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(debugExecuteReservedCreation(_:receiptFailure:completion:));
/// Exact issued-operation fault immediately before releasing the ordinary command stamp.
- (BOOL)debugFailCreationReceiptResolutionBeforeRelease:(Core3DModelingPreparedRequest *)request
    NS_SWIFT_NAME(debugFailCreationReceiptResolutionBeforeRelease(_:));
/// Read-only DEBUG owner state after a request has been released; no query or execution authority.
- (NSDictionary *)debugCreationReceiptOwnerState NS_SWIFT_NAME(debugCreationReceiptOwnerState());
/// Read-only document evidence, deliberately not a verified receipt query.
//! Corrupt a not-yet-admitted numeric proof to exercise fail-closed reservation.
- (BOOL)debugCorruptPreparedRebuildSourceEffect:(Core3DModelingPreparedRequest *)request
    NS_SWIFT_NAME(debugCorruptPreparedRebuildSourceEffect(_:));
//! DEBUG only: actual reserved rebuild; no production Store caller or verified query.
- (BOOL)debugExecuteReservedRebuild:(Core3DModelingPreparedRequest *)request receiptFailure:(BOOL)receiptFailure
    completion:(void (^)(Core3DProfileConstructionResult))completion
    NS_SWIFT_NAME(debugExecuteReservedRebuild(_:receiptFailure:completion:));
- (NSDictionary *)debugCreationReceiptState:(Core3DModelingPreparedRequest *)request
    NS_SWIFT_NAME(debugCreationReceiptState(_:));
//! DEBUG only: exact hashed effect bytes for a present request with one or two effects.
//! Read-only document evidence; no Store lookup, receipt mutation or verified query.
- (NSDictionary *)debugCreationReceiptEffectDiagnostics:(Core3DModelingPreparedRequest *)request
    NS_SWIFT_NAME(debugCreationReceiptEffectDiagnostics(_:));

/// Native-owned test configuration only; -1 uses the actual fixed-path Store.
/// Other bounded scenarios use an internally owned disposable POSIX Store.
/// No caller-selected path or fabricated reservation can enter this API.
- (BOOL)debugConfigureAsyncModelingRequest:(Core3DModelingPreparedRequest *)request
    storageScenario:(NSInteger)scenario
    deliveryGate:(void (^_Nullable)(void (^_Nonnull)(void)))gate
    receiptFailure:(BOOL)receiptFailure afterStart:(void (^_Nullable)(void))afterStart
    NS_SWIFT_NAME(debugConfigureAsyncModelingRequest(_:storageScenario:deliveryGate:receiptFailure:afterStart:));
/// DEBUG only: exact creation request's real geometry delivery, before owner
/// promotion/admission. The main-only gate cannot supply geometry or an outcome.
- (BOOL)debugGateAsyncCreationGeometryDelivery:(Core3DModelingPreparedRequest *)request
    gate:(void (^_Nonnull)(void (^_Nonnull)(void)))gate
    NS_SWIFT_NAME(debugGateAsyncCreationGeometryDelivery(_:gate:));
/// DEBUG only: main-owned observation after outcome sealing, before public callback.
/// It cannot supply a result, execute geometry or enter any utility worker.
- (BOOL)debugObserveAsyncModelingCompletion:(Core3DModelingPreparedRequest *)request
    observer:(void (^)(void))observer
    NS_SWIFT_NAME(debugObserveAsyncModelingCompletion(_:observer:));
/// Actual numeric dispatcher observation; no path or execution authority.
- (NSDictionary *)debugAsyncModelingStorageState:(Core3DModelingPreparedRequest *)request
    NS_SWIFT_NAME(debugAsyncModelingStorageState(_:));
/// A new default Store lookup for joint lifetime tests, not verified query authority.
- (void)debugLookupPermanentModelingRequest:(Core3DModelingPreparedRequest *)request
    completion:(void (^)(NSDictionary *))completion
    NS_SWIFT_NAME(debugLookupPermanentModelingRequest(_:completion:));
/// DEBUG-only actual private POSIX reservation; never runs production Apply.
- (BOOL)debugReserveModelingPreparedRequest:(Core3DModelingPreparedRequest *)request scenario:(NSInteger)scenario
    deliveryGate:(void (^_Nullable)(void (^_Nonnull)(void)))gate
    completion:(void (^_Nonnull)(NSDictionary *_Nonnull))completion
    NS_SWIFT_NAME(debugReserveModelingPreparedRequest(_:scenario:deliveryGate:completion:));
- (BOOL)debugConsumeReservedCapabilityWithoutGeometry:(Core3DModelingPreparedRequest *)request
    NS_SWIFT_NAME(debugConsumeReservedCapabilityWithoutGeometry(_:));
- (BOOL)debugExpireModelingReservation:(Core3DModelingPreparedRequest *)request
    NS_SWIFT_NAME(debugExpireModelingReservation(_:));
- (BOOL)debugInjectMismatchedModelingReservation:(Core3DModelingPreparedRequest *)request field:(NSInteger)field
    NS_SWIFT_NAME(debugInjectMismatchedModelingReservation(_:field:));
- (NSDictionary *)debugModelingReservationState:(Core3DModelingPreparedRequest *)request
    NS_SWIFT_NAME(debugModelingReservationState(_:));
- (NSData *_Nullable)debugMeterLengthUnitBinXCAFFixtureData;
//! Standalone fixture: saved profile binds its root but retains a different original unit.
- (NSData *_Nullable)debugUnitStaleProfileBinXCAFFixtureData;
- (NSData *_Nullable)debugGeometryStaleProfileBinXCAFFixtureData;
- (NSDictionary<NSString *, NSNumber *> *_Nullable)debugProfileDuplicateCapacityProbe:(NSInteger)mode;
- (NSDictionary<NSString *, NSNumber *> *_Nullable)debugProfileMirrorCapacityProbe:(NSInteger)mode;
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
//! Read-only observation of the actual native application's lifecycle/history.
//! These DEBUG diagnostics cannot authorize edits or persist an AI job.
//! Diagnostic fixed-memory mutation stamp; NOT a public edit token/readiness lease.
- (NSDictionary<NSString *, id> *_Nullable)debugNativeMutationStamp;
- (NSData *_Nullable)debugEnclosureBinXCAFFixturePlane:(NSInteger)plane width:(double)width fault:(NSInteger)fault
    NS_SWIFT_NAME(debugEnclosureFixture(plane:width:fault:));
+ (Core3DEnclosureDefinition *_Nullable)debugEnclosureDefinitionValues:(NSArray<NSNumber *> *)values
    NS_SWIFT_NAME(debugEnclosureDefinition(values:));
+ (NSArray<NSNumber *> *_Nullable)debugEncodedEnclosureDefinition:(Core3DEnclosureDefinition *)definition
    NS_SWIFT_NAME(debugEncodedEnclosure(_:));
+ (NSData *_Nullable)debugEnclosureFrameFixtureValues:(NSArray<NSNumber *> *)values state:(NSInteger)state
    NS_SWIFT_NAME(debugEnclosureFrameFixture(values:state:));
//! DEBUG fixture only: commit implicit transform defaults without notifying the
//! viewer revision, to prove an opening lease notices exact attribute presence.
- (BOOL)debugAuthorEnclosureTransformDefaultsWithoutRevision:(NSString *)identifier
    NS_SWIFT_NAME(debugAuthorEnclosureTransformDefaultsWithoutRevision(_:));
- (NSDictionary<NSString *, NSNumber *> *_Nullable)debugEnclosureCopyOwnershipState;
- (NSDictionary<NSString *, NSNumber *> *_Nullable)debugEnclosureDuplicateCapacityProbe:(NSInteger)mode;
- (NSDictionary<NSString *, NSNumber *> *_Nullable)debugEnclosureMirrorCapacityProbe:(NSInteger)mode;
- (NSDictionary<NSString *, id> *_Nullable)debugStoredEnclosureForEntityIdentifier:(NSString *)identifier
    NS_SWIFT_NAME(debugStoredEnclosure(entityIdentifier:));
- (BOOL)debugStoredEnclosureRejectsFault:(NSInteger)mode entityIdentifier:(NSString *)identifier
    NS_SWIFT_NAME(debugStoredEnclosureRejectsFault(_:entityIdentifier:));
+ (NSDictionary<NSString *, id> *)debugEnclosureValues:(NSArray<NSNumber *> *)input
    schema:(NSInteger)schema points:(NSArray<NSArray<NSNumber *> *> *)points cancelled:(BOOL)cancelled
    NS_SWIFT_NAME(debugEnclosure(values:schema:points:cancelled:));
+ (NSDictionary<NSString *, id> *)debugEnclosureUpdateValues:(NSArray<NSNumber *> *)input
    dimension:(NSInteger)dimension value:(double)value
    NS_SWIFT_NAME(debugEnclosureUpdate(values:dimension:value:));

//! Fixed detached sweep probes only. No document/selection/history authority.
+ (NSDictionary<NSString *, id> *)debugDetachedPlanarSweep:(NSInteger)fixture
    metersPerUnit:(double)metersPerUnit plane:(NSInteger)plane frame:(NSInteger)frame
    NS_SWIFT_NAME(debugDetachedPlanarSweep(fixture:metersPerUnit:plane:frame:));
+ (NSArray<NSDictionary<NSString *, id> *> *)debugDetachedPlanarSweepRejections;
+ (NSDictionary<NSString *, id> *)debugDetachedPlanarSweepCancellation;

//! Fixed detached ruled-loft probes only. No document or history authority.
+ (NSDictionary<NSString *, id> *)debugDetachedRectangularLoft:(NSInteger)fixture
    metersPerUnit:(double)metersPerUnit frame:(NSInteger)frame
    NS_SWIFT_NAME(debugDetachedRectangularLoft(fixture:metersPerUnit:frame:));
+ (NSArray<NSDictionary<NSString *, id> *> *)debugDetachedRectangularLoftRejections;
+ (NSDictionary<NSString *, id> *)debugDetachedRectangularLoftCancellation;
- (nullable NSDictionary<NSString *,id> *)debugStoredRectangularLoftForEntityIdentifier:(NSString *)identifier
    NS_SWIFT_NAME(debugStoredRectangularLoft(entityIdentifier:));
- (nullable NSData *)debugSavedRectangularLoftAdmissionFixture:(NSInteger)fault;
+ (NSDictionary<NSString *,id> *)debugRectangularLoftCodecValues:(NSArray<NSNumber *> *)values;
+ (NSDictionary<NSString *,id> *)debugRectangularLoftFamilyBudget;


//! Read-only existing saved record and fixed malformed production-loader fixtures.
- (nullable NSDictionary<NSString *, id> *)debugStoredSweepForEntityIdentifier:(NSString *)identifier
    NS_SWIFT_NAME(debugStoredSweep(entityIdentifier:));
- (nullable NSData *)debugSavedSweepAdmissionFixture:(NSInteger)fault;
- (BOOL)debugConfigureSavedSweepPairedWriteFailure;
- (BOOL)debugConfigureSavedSweepRebuildFault:(NSInteger)mode;
// Owned command probe: strict Read/Stage after an intentionally stale replacement;
// always aborts and checks the exact original. Never grants a rebind capability.
- (NSDictionary<NSString *,id> *)debugSavedSweepStrictBindingProbe;

//! Fixed raw OCAF component probes; no production sweep creation/load authority.
+ (NSDictionary<NSString *, id> *)debugSweepPersistenceScenario:(NSInteger)scenario
    NS_SWIFT_NAME(debugSweepPersistence(scenario:));

//! Pure native codec diagnostic; no live document or mutation authority.
+ (NSDictionary<NSString *, id> *)debugCurveProfileCodecValues:(NSArray<NSNumber *> *)values
    NS_SWIFT_NAME(debugCurveProfileCodec(values:));
+ (NSDictionary<NSString *, NSNumber *> *)debugProfileShellRequestCodec;
//! Read-only saved construction record from the actual OCAF object.
- (NSDictionary<NSString *, id> *_Nullable)debugStoredProfileDefinitionForEntityIdentifier:(NSString *)identifier
    NS_SWIFT_NAME(debugStoredProfileDefinition(entityIdentifier:));
- (BOOL)debugStoredProfileRejectsFault:(NSInteger)mode entityIdentifier:(NSString *)identifier
    NS_SWIFT_NAME(debugStoredProfileRejectsFault(_:entityIdentifier:));
//! Private real-document lifecycle plus isolated policy saturation/thread fixtures.
+ (NSDictionary<NSString *, NSNumber *> *)debugNativeMutationLifecycle;
- (BOOL)debugStartLiveTransactionProbe;
- (void)debugFailNextDocumentAdoption;
- (void)debugSetDocumentReplacementPreparationFailure:(BOOL)preparation restorationAttempts:(NSInteger)restorationAttempts;
#ifdef DEBUG
- (void)debugPauseQueuedAssetAdoption:(BOOL)paused;
- (void)debugFailQueuedAssetCleanup:(NSUInteger)count;
- (NSDictionary<NSString *, id> *)debugQueuedAssetLoadState;
#endif
- (void)debugStopLiveTransactionProbe;
- (NSDictionary<NSString *, id> *_Nullable)debugLiveTransactionProbe;
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
- (NSDictionary<NSString *,id> *_Nullable)debugScalarPBREvidence:(NSString *)entity;
//! Opt-in DEBUG observation of a native-validated unbound root; grants no edit authority.
- (NSDictionary<NSString *,id> *_Nullable)debugScalarPBREvidence:(NSString *)entity
    allowUnboundMaterial:(BOOL)allowUnboundMaterial
    NS_SWIFT_NAME(debugScalarPBREvidence(_:allowUnboundMaterial:));
- (BOOL)debugSeedScalarPBRCommonMismatch;
//! Lower the selected-object admission cap for aggregate texture-authoring
//! budget tests. Values above the production cap reset to that cap.
- (void)debugSetMaximumTextureAuthoringObjects:(NSUInteger)limit;
//! Lower the projected serialized texture-occurrence byte ceiling so budget
//! regressions can use small valid images. Values above 128 MiB reset to the
//! production ceiling.
- (void)debugSetMaximumSerializedTextureOccurrenceBytes:(NSUInteger)limit;
//! Lower the aggregate decoded unique-resource ceiling for writer tests.
- (void)debugSetMaximumDecodedTextureResourceBytes:(NSUInteger)limit;
//! Exercise production XCAF role wrappers without changing a document. Returns
//! decoded RGBA data and GPU/source identities for the same embedded image in
//! color and numeric roles; missing decoded data means numeric admission failed.
- (NSDictionary<NSString *, NSDictionary *> *_Nullable)
    debugTextureRolePreparation:(NSData *)data
    NS_SWIFT_NAME(debugTextureRolePreparation(_:));
//! Lower the immutable visual-material definition cap for batch replacement
//! tests. Values above 2048 reset to the production cap.
- (void)debugSetMaximumVisualMaterialDefinitions:(NSUInteger)limit;
//! Remove cached face triangulations without remeshing. Texture assignment
//! must fail closed while texture removal remains available.
- (BOOL)debugClearSelectedCachedTriangulationsForTextureTest;
//! Renderer-only normal binding; never edits native material/history. Nil clears.
//! DEBUG-only immutable derivative: rebinds the captured AO resource as normal.
//! Modes 1–10 deliberately violate the DTO contract; 11 is an authored sign
//! variant; 12 retains frames without a map; 13/14 reverse culling and reflect
//! the node; 15 has mirrored UVs; 16 mixes culling; 17 mixes signs within a
//! triangle and must reject. Never edits the document.
//! Private OCAF documents through the production builder; does not adopt or edit the live project.
- (NSDictionary<NSString *, id> *)debugNativeAuthoredFrameRendering:(NSData *)archive replacement:(NSData *)replacement normalPNG:(NSData *)normalPNG mode:(NSInteger)mode
    NS_SWIFT_NAME(debugNativeAuthoredFrameRendering(_:replacement:normalPNG:mode:));
- (NSDictionary<NSString *, id> *)debugAuthoredFramePublications:(NSData *)archive replacement:(NSData *)replacement mode:(NSInteger)mode
    NS_SWIFT_NAME(debugAuthoredFramePublications(_:replacement:mode:));
- (Core3DSceneSnapshot *_Nullable)debugCaptureNormalMappedSceneSnapshot:(NSInteger)validationMode;
- (BOOL)debugSetPresentationNormalTexture:(NSData *_Nullable)data;
- (NSUInteger)debugPreparedNativeTangentArrayCount;
- (NSArray<NSNumber *> *)debugNormalTextureRecipes;
//! Isolated valid control0 or malformed normal-recipe cases1...6.
- (NSData *_Nullable)debugNormalRecipeBinXCAFFixtureData:(NSInteger)mode;
//! Private fixture bytes:0 unbound,1 mapped supplied,2...7 malformed,8 mixed supplied/unbound Mikk geometry.
- (NSData *_Nullable)debugSuppliedFrameBinXCAFFixture:(NSData *)archive normalPNG:(NSData *)normalPNG mode:(NSInteger)mode
    NS_SWIFT_NAME(debugSuppliedFrameBinXCAFFixture(_:normalPNG:mode:));
- (NSArray<NSDictionary<NSString *, id> *> *_Nullable)debugNativeSuppliedFrameState;
- (BOOL)debugSetNormalPresentationHidden:(BOOL)hidden;
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
- (NSDictionary<NSString *, id> *_Nullable)debugViewportRGBA;
- (NSDictionary<NSString *, NSNumber *> *)debugFramebufferStatistics;
//! Read-only last native tangent-preparation failure and render counters.
- (NSDictionary<NSString *, NSNumber *> *)debugNativeFramePreparationState;
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
//! Called after a successful native draw and renderbuffer presentation. This
//! does not consume or replace observeNativeViewportPresentation ownership.
- (void)viewDidPresentNativeViewport;
- (void)viewDidAssetModify;
- (void)viewDidLoadFromBundle;
- (void)viewDidFailToLoadFromBundle:(Core3DAssetLoadResult)result
    NS_SWIFT_NAME(viewDidFailToLoadFromBundle(_:));

@end

NS_ASSUME_NONNULL_END
