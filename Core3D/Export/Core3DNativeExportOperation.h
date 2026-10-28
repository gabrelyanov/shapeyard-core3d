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
    Core3DNativeExportErrorDecalCoverageRefused,
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

#ifdef DEBUG
//! Deterministic failure points in the private E4 worker path. They are
//! operation-local and can only be selected before start.
typedef NS_ENUM(NSInteger, Core3DDebugE4ExportFault) {
    Core3DDebugE4ExportFaultNone = 0,
    Core3DDebugE4ExportFaultCapture,
    Core3DDebugE4ExportFaultFinalProduction,
    Core3DDebugE4ExportFaultPublication,
    Core3DDebugE4ExportFaultAggregateBudget,
    //! Stops after the complete saved-atlas read set and measured native copies.
    Core3DDebugE4ExportFaultSavedAtlasCaptureCheckpoint,
    //! Refuses the saved-atlas preflight before its materializing allocation.
    Core3DDebugE4ExportFaultSavedAtlasCaptureDenyRead,
    //! Removes the operation-private saved SYEB record before capture.
    Core3DDebugE4ExportFaultSavedAtlasCaptureMissingBake,
    //! Corrupts the operation-private saved SYEB record version before capture.
    Core3DDebugE4ExportFaultSavedAtlasCaptureMalformedBake,
    //! Stales one saved SYEB binding fence before capture.
    Core3DDebugE4ExportFaultSavedAtlasCaptureStaleBinding,
    //! Erases one saved member's entity identity before capture.
    Core3DDebugE4ExportFaultSavedAtlasCaptureMissingMember,
    //! Rewrites one saved member's entity identity to a foreign value.
    Core3DDebugE4ExportFaultSavedAtlasCaptureForeignMember,
    //! Duplicates one saved member's identity onto the other member.
    Core3DDebugE4ExportFaultSavedAtlasCaptureDuplicateMember,
    //! Replaces one raw slot with a file-backed foreign texture.
    Core3DDebugE4ExportFaultSavedAtlasCaptureForeignRawSlot,
    //! Corrupts one saved image resource record before capture.
    Core3DDebugE4ExportFaultSavedAtlasCaptureMalformedResource,
};

//! Operation-local worker phases used only to coordinate deterministic tests.
//! Reaching a phase never changes the native result; the worker resumes after
//! explicit release or cancellation.
typedef NS_ENUM(NSInteger, Core3DDebugE4ExportPhase) {
    Core3DDebugE4ExportPhaseNone = 0,
    Core3DDebugE4ExportPhaseCapture,
    Core3DDebugE4ExportPhaseFinalization,
};

typedef void (NS_SWIFT_SENDABLE ^Core3DDebugE4PhaseReached)(void);
#endif

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
//! Returns the immutable saved-group all-member layout read for this
//! operation's exact returned-scene object. Any copied, prior, or foreign
//! scene returns nil. A valid alias can return a typed absent state.
- (nullable NSDictionary<NSString *, id> *)
    savedGroupAllMemberLayoutRecordForReturnedScene:
        (Core3DSceneSnapshot *)returnedScene
    NS_SWIFT_NAME(savedGroupAllMemberLayoutRecord(returnedScene:));
//! Returns the immutable operation-owned raw evidence only for this
//! operation's exact returned-scene alias. Foreign, copied, and prior aliases
//! return nil; the read is passive and never mutates or republishes evidence.
- (nullable NSDictionary<NSString *, id> *)
    operationOwnedRawEvidenceRecordForReturnedScene:
        (Core3DSceneSnapshot *)returnedScene
    NS_SWIFT_NAME(operationOwnedRawEvidenceRecord(returnedScene:));

#ifdef DEBUG
//! Arms one operation-local E4 fault before the worker starts.
//! Scalar reads from the actual operation-owned final saved bake evidence.
- (nullable NSArray<NSNumber *> *)debugSavedFinalQualityForReturnedScene:
    (Core3DSceneSnapshot *)returnedScene
    NS_SWIFT_NAME(debugSavedFinalQuality(returnedScene:));
- (BOOL)debugSetE4Fault:(Core3DDebugE4ExportFault)fault
    NS_SWIFT_NAME(debugSetE4Fault(_:));
//! Returns bounded values captured by this operation's executed native path.
//! The result contains no OCAF labels, handles, or authority tokens.
- (NSDictionary<NSString *, id> *)debugE4Observation
    NS_SWIFT_NAME(debugE4Observation());
//! Arms one pause before the named phase's existing protected cancellation
//! recheck. The callback is delivered on the main thread after the worker is
//! waiting. Only one phase can be armed and it must be selected before start.
- (BOOL)debugPauseAtE4Phase:(Core3DDebugE4ExportPhase)phase
                  onReached:(Core3DDebugE4PhaseReached)onReached
    NS_SWIFT_NAME(debugPause(atE4Phase:onReached:));
//! Releases this operation's phase pause. Safe to call from timeout cleanup.
- (void)debugResumeE4Phase NS_SWIFT_NAME(debugResumeE4Phase());
//! Actual shared E4 cache keys and resident-byte count at the instant queried.
+ (NSDictionary<NSString *, id> *)debugE4SharedCacheInventory
    NS_SWIFT_NAME(debugE4SharedCacheInventory());
//! Evicts the actual shared E4 cache. Private export must remain independent.
+ (void)debugE4EvictSharedCache
    NS_SWIFT_NAME(debugE4EvictSharedCache());

// DEBUG-only export calibration seam. Applies a local-definition absolute
// chord tolerance to the private export copy only. This is NOT a certified
// world-space global error bound under transforms.
- (BOOL)debugUseAbsoluteChordMM:(double)chordMM
                   angleDegrees:(double)angleDegrees
         initialInteriorControl:(BOOL)initialInteriorControl
    NS_SWIFT_NAME(debugUseAbsoluteChord(mm:angleDegrees:initialInteriorControl:));
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
