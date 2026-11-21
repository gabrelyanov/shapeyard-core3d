#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

FOUNDATION_EXPORT NSErrorDomain const Core3DNativeImportErrorDomain;

typedef NS_ENUM(NSInteger, Core3DNativeImportFormat) {
    Core3DNativeImportFormatSTEP NS_SWIFT_NAME(step) = 1,
    Core3DNativeImportFormatGLB NS_SWIFT_NAME(glb) = 2,
    Core3DNativeImportFormatSTL NS_SWIFT_NAME(stl) = 3,
    Core3DNativeImportFormatOBJ NS_SWIFT_NAME(obj) = 4,
};

//! Immutable copied buffers supplied by the application OBJ decoder. Vertices
//! use the Core3DSceneVertex ABI (eight Float32 values); indices are UInt32.
NS_SWIFT_SENDABLE
@interface Core3DNativeImportMesh : NSObject

@property(nonatomic, readonly, copy) NSData *vertexData;
@property(nonatomic, readonly, copy) NSData *indexData;
@property(nonatomic, readonly) NSUInteger vertexCount;
@property(nonatomic, readonly) NSUInteger indexCount;

- (instancetype)initWithVertexData:(NSData *)vertexData
                         indexData:(NSData *)indexData
                       vertexCount:(NSUInteger)vertexCount
                        indexCount:(NSUInteger)indexCount
    NS_DESIGNATED_INITIALIZER
    NS_SWIFT_NAME(init(vertexData:indexData:vertexCount:indexCount:));
- (instancetype)init NS_UNAVAILABLE;

@end

typedef BOOL (NS_SWIFT_SENDABLE ^Core3DNativeImportCancellationProbe)(void);
typedef Core3DNativeImportMesh *_Nullable
    (NS_SWIFT_SENDABLE ^Core3DNativeImportMeshDecoder)(
        NSData *stagedBytes,
        Core3DNativeImportCancellationProbe cancellationProbe,
        NSError *_Nullable *_Nullable error);

//! STL has no embedded length units. The importer converts into millimeters.
typedef NS_ENUM(NSInteger, Core3DNativeImportSTLUnit) {
    Core3DNativeImportSTLUnitMillimeters NS_SWIFT_NAME(millimeters) = 0,
    Core3DNativeImportSTLUnitCentimeters NS_SWIFT_NAME(centimeters) = 1,
    Core3DNativeImportSTLUnitMeters NS_SWIFT_NAME(meters) = 2,
    Core3DNativeImportSTLUnitInches NS_SWIFT_NAME(inches) = 3,
};

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

//! Thread-safe, exactly-once native-to-project import. The selected source is
//! securely copied into app-owned staging before OCCT sees it. Parsing,
//! migration, validation, serialization, and cleanup run on a private worker;
//! completion is always delivered on the main thread.
NS_SWIFT_SENDABLE
@interface Core3DNativeImportOperation : NSObject

@property(atomic, readonly, getter=isCancelled) BOOL cancelled;

- (instancetype)init NS_UNAVAILABLE;
- (nullable instancetype)initWithSourceURL:(NSURL *)sourceURL
                                    format:(Core3DNativeImportFormat)format
    NS_SWIFT_NAME(init(sourceURL:format:));
- (nullable instancetype)initWithSTEPURL:(NSURL *)stepURL
    NS_SWIFT_NAME(init(stepURL:));
//! Binary/ASCII STL geometry import with explicit authored units. The generic
//! sourceURL/format initializer defaults STL to millimeters. Nonstandard STL
//! colors are not imported; vertex winding determines the flat mesh normals.
- (nullable instancetype)initWithSTLURL:(NSURL *)stlURL
                                  unit:(Core3DNativeImportSTLUnit)unit
    NS_SWIFT_NAME(init(stlURL:unit:));
//! Geometry-only OBJ import. The operation owns source staging and invokes the
//! decoder once on its worker using immutable bytes from the pinned descriptor.
- (nullable instancetype)initWithOBJURL:(NSURL *)objURL
                    millimetersPerUnit:(double)millimetersPerUnit
                           meshDecoder:(Core3DNativeImportMeshDecoder)meshDecoder
    NS_SWIFT_NAME(init(objURL:millimetersPerUnit:meshDecoder:));
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
