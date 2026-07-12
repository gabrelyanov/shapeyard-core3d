#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

FOUNDATION_EXPORT NSErrorDomain const Core3DNativeImportErrorDomain;

typedef NS_ERROR_ENUM(
    Core3DNativeImportErrorDomain,
    Core3DNativeImportErrorCode
) {
    Core3DNativeImportErrorInvalidSource = 1,
    Core3DNativeImportErrorCancelled,
    Core3DNativeImportErrorReadFailed,
    Core3DNativeImportErrorTransferFailed,
    Core3DNativeImportErrorNoGeometry,
    Core3DNativeImportErrorResourceLimit,
    Core3DNativeImportErrorMigrationFailed,
    Core3DNativeImportErrorWriterFailed,
    Core3DNativeImportErrorInvalidArtifact,
    Core3DNativeImportErrorInternalFailure,
    Core3DNativeImportErrorUnsupportedEncoding,
};

//! Immutable ownership record for one isolated native import. `primaryURL` is
//! a self-contained BinXCAF project document. The receiver must remove
//! `cleanupURL` after persistence or abandonment.
NS_SWIFT_SENDABLE
@interface Core3DNativeImportArtifact : NSObject

@property(nonatomic, readonly, copy) NSURL *primaryURL;
@property(nonatomic, readonly, copy) NSURL *packageRootURL;
@property(nonatomic, readonly, copy) NSURL *cleanupURL;
@property(nonatomic, readonly) NSUInteger shapeCount;

- (instancetype)init NS_UNAVAILABLE;

@end

typedef void (NS_SWIFT_SENDABLE ^Core3DNativeImportCompletion)(
    Core3DNativeImportArtifact *_Nullable artifact,
    NSError *_Nullable error
);

//! Thread-safe, exactly-once STEP-to-project import. The selected source is
//! securely copied into app-owned staging before OCCT sees it. Parsing,
//! migration, validation, serialization, and cleanup run on a private worker;
//! completion is always delivered on the main thread.
NS_SWIFT_SENDABLE
@interface Core3DNativeImportOperation : NSObject

@property(atomic, readonly, getter=isCancelled) BOOL cancelled;

- (instancetype)init NS_UNAVAILABLE;
- (nullable instancetype)initWithSTEPURL:(NSURL *)stepURL
    NS_SWIFT_NAME(init(stepURL:));
- (void)startWithCompletion:(Core3DNativeImportCompletion)completion
    NS_SWIFT_NAME(start(completion:));
- (void)cancel;

#ifdef DEBUG
//! Deterministic XCTest barrier for main-run-loop and cancellation coverage.
+ (void)debugSetWorkerPaused:(BOOL)paused
    NS_SWIFT_NAME(debugSetWorkerPaused(_:));
//! Pauses after the source identity is captured and at least one copy chunk is
//! staged, allowing XCTest to exercise same-size in-place mutation races.
+ (void)debugSetSourceIdentityBarrierPaused:(BOOL)paused
    NS_SWIFT_NAME(debugSetSourceIdentityBarrierPaused(_:));
+ (BOOL)debugIsSourceIdentityBarrierWaiting
    NS_SWIFT_NAME(debugIsSourceIdentityBarrierWaiting());
//! One-shot pause after the next import has entered its cancellable structural
//! topology walk. Later imports bypass the claimed barrier.
+ (void)debugSetStructuralValidationBarrierPaused:(BOOL)paused
    NS_SWIFT_NAME(debugSetStructuralValidationBarrierPaused(_:));
+ (BOOL)debugIsStructuralValidationBarrierWaiting
    NS_SWIFT_NAME(debugIsStructuralValidationBarrierWaiting());
#endif

@end

NS_ASSUME_NONNULL_END
