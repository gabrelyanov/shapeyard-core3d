#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

FOUNDATION_EXPORT NSErrorDomain const Core3DNativeExportErrorDomain;

typedef NS_ERROR_ENUM(
    Core3DNativeExportErrorDomain,
    Core3DNativeExportErrorCode
) {
    Core3DNativeExportErrorInvalidState = 1,
    Core3DNativeExportErrorCancelled,
    Core3DNativeExportErrorSnapshotOpenFailed,
    Core3DNativeExportErrorNoGeometry,
    Core3DNativeExportErrorMeshingFailed,
    Core3DNativeExportErrorWriterFailed,
    Core3DNativeExportErrorInvalidArtifact,
    Core3DNativeExportErrorInternalFailure,
};

//! Immutable ownership record for one native export. `packageRootURL` contains
//! the primary file and every writer-owned sidecar. The receiver must remove
//! `cleanupURL` after packaging or sharing completes.
NS_SWIFT_SENDABLE
@interface Core3DNativeExportArtifact : NSObject

@property(nonatomic, readonly, copy) NSURL *primaryURL;
@property(nonatomic, readonly, copy) NSURL *packageRootURL;
@property(nonatomic, readonly, copy) NSURL *cleanupURL;

- (instancetype)init NS_UNAVAILABLE;

@end

typedef void (NS_SWIFT_SENDABLE ^Core3DNativeExportCompletion)(
    Core3DNativeExportArtifact *_Nullable artifact,
    NSError *_Nullable error
);

//! Thread-safe, exactly-once native export operation. OCCT work is serialized
//! on a private worker queue and completion is always delivered on the main
//! thread. Cancellation is cooperative and owns cleanup of partial artifacts.
NS_SWIFT_SENDABLE
@interface Core3DNativeExportOperation : NSObject

@property(atomic, readonly, getter=isCancelled) BOOL cancelled;

- (instancetype)init NS_UNAVAILABLE;
- (void)startWithCompletion:(Core3DNativeExportCompletion)completion
    NS_SWIFT_NAME(start(completion:));
- (void)cancel;

#ifdef DEBUG
//! Deterministic XCTest seam for proving that start/cancel never blocks the
//! main run loop. Production builds do not expose or execute this barrier.
+ (void)debugSetWorkerPaused:(BOOL)paused
    NS_SWIFT_NAME(debugSetWorkerPaused(_:));
//! Simulates OCCT accepting an OBJ write while omitting every map_Kd entry.
//! This proves that source texture expectations fail closed during validation.
- (void)debugSimulateTextureReferenceOmission
    NS_SWIFT_NAME(debugSimulateTextureReferenceOmission());
#endif

@end

NS_ASSUME_NONNULL_END
