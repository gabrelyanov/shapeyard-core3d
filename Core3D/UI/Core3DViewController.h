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
