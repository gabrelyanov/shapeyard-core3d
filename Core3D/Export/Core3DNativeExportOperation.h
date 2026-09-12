#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DSceneSnapshot;
typedef void (NS_SWIFT_SENDABLE ^Core3DExportSceneCompletion)(
    Core3DSceneSnapshot *_Nullable snapshot, NSError *_Nullable error);

//! Viewport preserves the legacy captured drawer settings. Other presets
//! rebuild only private geometric surfaces for OBJ/STL/GLB; authored meshes keep
//! their original triangles. STEP does not accept a tessellation preset.
typedef NS_ENUM(NSInteger, Core3DExportMeshQuality) {
    Core3DExportMeshQualityViewport = 0,
    Core3DExportMeshQualityCoarse,
    Core3DExportMeshQualityStandard,
    Core3DExportMeshQualityFine,
};

//! OBJ material coefficient convention. Current preserves the existing mixed
//! legacy output (linear authored PBR diffuse, OCCT sRGB common coefficients).
//! Linear and SRGB explicitly encode all diffuse/ambient/specular coefficients;
//! texture bytes and scalar opacity/metallic/roughness are never gamma converted.
//! Explicit modes use d for opacity; Current retains the legacy Tr directive.
typedef NS_ENUM(NSInteger, Core3DOBJColorConvention) {
    Core3DOBJColorConventionCurrent = 0,
    Core3DOBJColorConventionLinear,
    Core3DOBJColorConventionEncoded,
};

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
@property(nonatomic, readonly) Core3DExportMeshQuality meshQuality;
@property(nonatomic, readonly) Core3DOBJColorConvention objColorConvention;

- (instancetype)init NS_UNAVAILABLE;
- (void)startWithCompletion:(Core3DNativeExportCompletion)completion
    NS_SWIFT_NAME(start(completion:));
//! GLB preparation yields immutable values for the strict client writer. All
//! private document files are removed before completion; no geometry file is
//! produced. Call start(completion:) only for OBJ/STL/STEP operations.
- (void)startSceneSnapshotWithCompletion:(Core3DExportSceneCompletion)completion
    NS_SWIFT_NAME(startSceneSnapshot(completion:));
- (void)cancel;

#ifdef DEBUG
+ (NSDictionary<NSString *, id> *)debugParametricSTLRefinement:(double)metersPerUnit
    NS_SWIFT_NAME(debugParametricSTLRefinement(metersPerUnit:));
//! Deterministic XCTest seam for proving that start/cancel never blocks the
//! main run loop. Production builds do not expose or execute this barrier.
+ (void)debugSetWorkerPaused:(BOOL)paused
    NS_SWIFT_NAME(debugSetWorkerPaused(_:));
//! Simulates OCCT accepting an OBJ write while omitting every map_Kd entry.
//! This proves that source texture expectations fail closed during validation.
- (void)debugSimulateTextureReferenceOmission
    NS_SWIFT_NAME(debugSimulateTextureReferenceOmission());
//! Corrupts the binary STL triangle-count header after writing so XCTest can
//! prove that artifact validation fails closed and removes every private file.
- (void)debugSimulateSTLTriangleCountCorruption
    NS_SWIFT_NAME(debugSimulateSTLTriangleCountCorruption());
//! Appends non-whitespace after the STEP terminator so XCTest can prove that
//! structural validation fails closed and removes every private file.
- (void)debugSimulateSTEPTerminatorCorruption
    NS_SWIFT_NAME(debugSimulateSTEPTerminatorCorruption());
//! Lowers the STL resource ceiling below one cube so XCTest can prove that an
//! oversized flattened mesh fails before its duplicate allocation.
- (void)debugSimulateSTLResourceLimitExceeded
    NS_SWIFT_NAME(debugSimulateSTLResourceLimitExceeded());
#endif

@end

NS_ASSUME_NONNULL_END
