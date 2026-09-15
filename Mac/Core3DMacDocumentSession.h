#import <AppKit/AppKit.h>
#import <Core3D/Core3DSceneSnapshot.h>
#import <Core3D/Core3DTransformInspectorSnapshot.h>
#include <stdint.h>

NS_ASSUME_NONNULL_BEGIN

typedef NS_ENUM(NSInteger, Core3DMacSessionActionResult) {
    Core3DMacSessionActionResultChanged = 0,
    Core3DMacSessionActionResultChangedPublicationUnavailable,
    Core3DMacSessionActionResultUnchanged,
    Core3DMacSessionActionResultBusy,
    Core3DMacSessionActionResultStale,
    Core3DMacSessionActionResultUnsupported,
    Core3DMacSessionActionResultUnavailable,
    Core3DMacSessionActionResultContextUnavailable,
    Core3DMacSessionActionResultInternalFailure,
};

typedef NS_ENUM(NSInteger, Core3DMacHistoryOutcome) {
    Core3DMacHistoryOutcomeUnavailable = 0,
    Core3DMacHistoryOutcomePreviewCancelled,
    Core3DMacHistoryOutcomePreviewCancellationFailed,
    Core3DMacHistoryOutcomeNoHistory,
    Core3DMacHistoryOutcomeChanged,
    Core3DMacHistoryOutcomeFailed,
};

typedef NS_ENUM(NSInteger, Core3DMacHistoryDirection) {
    Core3DMacHistoryDirectionUndo = 0,
    Core3DMacHistoryDirectionRedo,
};

typedef NS_ENUM(NSInteger, Core3DMacOrdinaryEditRecoveryResult) {
    Core3DMacOrdinaryEditRecoveryResultUnavailable = 0,
    Core3DMacOrdinaryEditRecoveryResultContextUnavailable,
    Core3DMacOrdinaryEditRecoveryResultNoUnresolvedEdit,
    Core3DMacOrdinaryEditRecoveryResultNoChange,
    Core3DMacOrdinaryEditRecoveryResultCommitted,
    Core3DMacOrdinaryEditRecoveryResultNoChangePublicationUnavailable,
    Core3DMacOrdinaryEditRecoveryResultCommittedPublicationUnavailable,
    Core3DMacOrdinaryEditRecoveryResultRetryableFailure,
    Core3DMacOrdinaryEditRecoveryResultOutcomeUnknown,
    Core3DMacOrdinaryEditRecoveryResultBusy,
    Core3DMacOrdinaryEditRecoveryResultInvalid,
};

__attribute__((objc_subclassing_restricted))
@interface Core3DMacScenePublication : NSObject
@property(nonatomic,strong,readonly) Core3DSceneSnapshot *scene;
@property(nonatomic,strong,readonly) Core3DScenePresentationOverlaySnapshot *overlay;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DMacHistoryResult : NSObject
@property(nonatomic,readonly) Core3DMacHistoryOutcome outcome;
@property(nonatomic,readonly) BOOL documentRedrawn;
@property(nonatomic,readonly) BOOL booleanReconciliationRequired;
@property(nonatomic,readonly) BOOL selectionRefreshRequested;
@property(nonatomic,readonly) BOOL renderRequested;
@property(nonatomic,readonly) BOOL primaryInteractionCancelled;
@property(nonatomic,strong,readonly,nullable) Core3DMacScenePublication *publication;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! First-increment main-thread adapter. The supplied NSOpenGLView must already
//! be attached to a window and own a context whose view is exactly that view.
__attribute__((objc_subclassing_restricted))
NS_SWIFT_UI_ACTOR
@interface Core3DMacDocumentSession : NSObject

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithAttachedOpenGLView:(NSOpenGLView *)view
    NS_DESIGNATED_INITIALIZER;

@property(nonatomic,strong,readonly) NSOpenGLView *engineView;
@property(nonatomic,strong,readonly,nullable) Core3DMacScenePublication *publication;
@property(nonatomic,readonly,getter=isClosed) BOOL closed;
@property(nonatomic,copy,nullable) dispatch_block_t booleanReconciliationHandler;
@property(nonatomic,copy,nullable) void (^selectionRefreshHandler)(
    Core3DMacScenePublication * _Nullable publication);
@property(nonatomic,copy,nullable) void (^renderRequestHandler)(
    Core3DMacScenePublication * _Nullable publication);
@property(nonatomic,copy,nullable) dispatch_block_t primaryInteractionCancelledHandler;

//! Captures the real committed scene plus the viewer's actual paired overlay.
//! A missing/invalid overlay is a refusal; no empty overlay is manufactured.
- (Core3DMacSessionActionResult)refreshPublicationWithWidth:(uint32_t)width
    height:(uint32_t)height;

//! Success requires a model-revision advance and exactly one additional
//! visible/selectable committed model instance in the retained native snapshot.
- (Core3DMacSessionActionResult)createCubeWithViewportWidth:(uint32_t)width
    height:(uint32_t)height;

//! expectedPublication must be this session's exact current publication.
//! The native identity also binds publication source and revision domains.
- (Core3DMacSessionActionResult)selectEntityIdentifier:(NSString *)identifier
    expectedPublication:(Core3DMacScenePublication *)expectedPublication
    viewportWidth:(uint32_t)width height:(uint32_t)height;

//! Immediate state is returned synchronously. Only Measuring may later deliver
//! completion, at most once on main after the native capture transition returns.
- (Core3DTransformInspectorSnapshot *)captureTransformMeasurementWithCompletion:
    (Core3DTransformInspectorCompletion _Nullable)completion;
- (void)cancelTransformMeasurement;

//! Commits one absolute raw document-model-unit scalar using the exact retained
//! native measurement. expectedMeasurement must be the exact DTO returned by
//! the latest completed capture. A foreign/stale DTO does not consume a newer
//! retained measurement; an admitted attempt consumes it before native entry.
- (Core3DTransformInspectorPositionCommitResult)commitPositionValue:(double)value
    axis:(Core3DTransformInspectorAxis)axis
    expectedMeasurement:(Core3DTransformInspectorSnapshot *)expectedMeasurement;

//! Completes the native transition first, restores the caller's prior GL
//! context, then emits effects in Boolean/selection/render/primary order.
- (Core3DMacHistoryResult *)performHistory:(Core3DMacHistoryDirection)direction;

//! Runs only the shared ordinary-edit recovery ledger. A resolved result also
//! requires a fresh paired scene publication; unresolved results remain retryable.
- (Core3DMacOrdinaryEditRecoveryResult)reconcileOrdinaryEditWithViewportWidth:
    (uint32_t)width height:(uint32_t)height;

//! Refuses while native recovery is unresolved. Successful close cancels
//! measurement callbacks and releases OCCT GPU state with the owned context current.
- (Core3DMacSessionActionResult)close;

@end

NS_ASSUME_NONNULL_END
