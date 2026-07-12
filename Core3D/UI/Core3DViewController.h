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

typedef struct {
    double min;
    double max;
} Boundaries;

@protocol PrimitiveManagerProtocol<NSObject>

- (void)addPrimitive:(PrimitiveType)primitiveType;
- (void)addPrimitivesFromJSON:(NSString *)json;
- (void)deleteSelected;
- (void)selectAll;
- (void)duplicateSelected;
- (void)setChamfer:(CGFloat)value;
- (Boundaries)getChamferBoundaries;
- (void)applyChamfer;
- (void)cancelChamfer;
- (void)setExtrusion:(CGFloat)value;
- (Boundaries)getExtrusionBoundaries;
- (BOOL)applyExtrusion;
- (BOOL)cancelExtrusion;
- (void)applyMirror;
- (void)cancelMirror;
- (void)applySubtract;
- (void)cancelSubtract;
- (void)applyUnion;
- (void)cancelUnion;
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

- (NSArray<NSNumber *> *_Nonnull)availableGizmoTypes;

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
//! snapshot seam, this returns nil while a Boolean or Mirror trial is active,
//! unresolved, or while the OCAF document owns an open command. Main-thread
//! only; the returned value is an immutable deep copy safe for background I/O.
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
//! immutable values. Returns nil while a model transaction is open or before
//! the native viewer has finished setup. Main-thread only.
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

- (void)setSelectionType:(PrimitiveSelectionType)type;
- (void)setGizmoType:(PrimitiveGizmoType)type;

#ifdef DEBUG
//! Test-only direct mirror-plane seam. This bypasses pointer hit testing while
//! preserving the authoritative mirror lifecycle and renderer invalidation.
- (BOOL)debugTryMirrorAxis:(NSInteger)axis
                  backward:(BOOL)backward
    NS_SWIFT_NAME(debugTryMirror(axis:backward:));
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
//! Deterministically capture a planar face by stable entity identifier and
//! zero-based TopExp face index, using the production extrusion admission.
- (BOOL)debugBeginExtrusionWithEntityIdentifier:(NSString *)entityIdentifier
                              faceTopologyIndex:(NSUInteger)faceTopologyIndex
    NS_SWIFT_NAME(debugBeginExtrusion(entityIdentifier:faceTopologyIndex:));
//! Bounded native extrusion preview/result counters and command state.
- (NSDictionary<NSString *, NSNumber *> *)debugExtrusionState;
//! Inject CommitCommand reporting/throw behavior after a real close: 0 normal,
//! 1 false-after-close, 2 throw-after-close.
- (void)debugSetExtrusionCommitMode:(NSInteger)mode;
//! Make the next N extrusion abort attempts fail before touching the command.
- (void)debugSetExtrusionAbortFailureCount:(NSUInteger)count;
//! Make the next N post-Commit label inspections report unavailable.
- (void)debugSetExtrusionPostCommitInspectFailureCount:(NSUInteger)count;
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
//! Valid XCAF material with a normal map that schema v3 cannot represent.
//! Snapshot publication must fail closed so OCCT remains authoritative.
- (NSData *_Nullable)debugUnsupportedPBRTextureBinXCAFFixtureData;
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
//! Called after the native viewport has been invalidated. Subclasses should
//! coalesce work and must not synchronously recapture full geometry per call.
- (void)viewDidInvalidateSceneSnapshot;
- (void)viewDidAssetModify;
- (void)viewDidLoadFromBundle;
- (void)viewDidFailToLoadFromBundle:(Core3DAssetLoadResult)result
    NS_SWIFT_NAME(viewDidFailToLoadFromBundle(_:));

@end

NS_ASSUME_NONNULL_END
