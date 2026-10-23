#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DPublishCapture;
@class Core3DViewController;
@class Core3DDecimationResult;
@class Core3DDecimationSettingsReadback;

FOUNDATION_EXPORT NSErrorDomain const Core3DDecimationErrorDomain;

typedef NS_ERROR_ENUM(Core3DDecimationErrorDomain, Core3DDecimationError) {
    Core3DDecimationErrorTargetBelowClosedMeshMinimum = 1,
    Core3DDecimationErrorInvalidTopology,
    Core3DDecimationErrorNonFiniteInput,
    Core3DDecimationErrorResourceLimit,
    Core3DDecimationErrorCancelled,
    Core3DDecimationErrorUnreachableTriangleBudget,
    Core3DDecimationErrorStaleSource,
    Core3DDecimationErrorMalformedRequest,
    Core3DDecimationErrorDerivativeNotPersisted,
    Core3DDecimationErrorDerivativePersistenceUnavailable,
    Core3DDecimationErrorMissingArtifactEvidence,
    Core3DDecimationErrorRecoveryRequired,
};

//! Immutable POSITION-only input to the native geometry kernel. Values are
//! already in right-handed Z-up world millimetres, as independently observed
//! from a completed GLB. The source artifact and settings digests bind those
//! bytes; analytic contribution tokens arrive in exact emitted-triangle order.
__attribute__((objc_subclassing_restricted))
@interface Core3DDecimationRequest : NSObject
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *positionsMM;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *indices;
@property(nonatomic, copy, readonly) NSArray<NSString *> *sourceContributionTokens;
@property(nonatomic, copy, readonly) NSData *materialBytes;
@property(nonatomic, copy, readonly) NSString *sourceArtifactSHA256;
@property(nonatomic, copy, readonly) NSString *settingsSHA256;
@property(nonatomic, assign, readonly) NSUInteger targetTriangleCount;
@property(nonatomic, assign, readonly) double metersPerUnit;
//! Optional F2 attribute payload. These four properties are either all nil
//! (the original flat-normal path) or all nonnil and aligned to the request.
//! The designated attribute initializer below is the only public writer.
@property(nonatomic, copy, readonly, nullable) NSArray<NSNumber *> *f2AttributeWedgeNormals;
@property(nonatomic, copy, readonly, nullable) NSArray<NSNumber *> *f2AttributeWedgeUVs;
@property(nonatomic, copy, readonly, nullable) NSArray<NSNumber *> *f2AttributeFaceMaterials;
@property(nonatomic, copy, readonly, nullable) NSArray<NSString *> *f2AttributeFaceAnchors;
- (instancetype)initWithPositionsMM:(NSArray<NSNumber *> *)positionsMM
                             indices:(NSArray<NSNumber *> *)indices
            sourceContributionTokens:(NSArray<NSString *> *)sourceContributionTokens
                       materialBytes:(NSData *)materialBytes
                sourceArtifactSHA256:(NSString *)sourceArtifactSHA256
                      settingsSHA256:(NSString *)settingsSHA256
                 targetTriangleCount:(NSUInteger)targetTriangleCount
                       metersPerUnit:(double)metersPerUnit NS_DESIGNATED_INITIALIZER;
//! Typed all-or-none form of the retained F2-S3 request seam. It returns nil
//! unless every attribute array is present and has the exact source-domain
//! cardinality; decimateRequest: still performs its complete validation.
- (nullable instancetype)initWithPositionsMM:(NSArray<NSNumber *> *)positionsMM
                                      indices:(NSArray<NSNumber *> *)indices
                     sourceContributionTokens:(NSArray<NSString *> *)sourceContributionTokens
                                materialBytes:(NSData *)materialBytes
                         sourceArtifactSHA256:(NSString *)sourceArtifactSHA256
                               settingsSHA256:(NSString *)settingsSHA256
                          targetTriangleCount:(NSUInteger)targetTriangleCount
                                metersPerUnit:(double)metersPerUnit
                        attributeWedgeNormals:(NSArray<NSNumber *> *)attributeWedgeNormals
                            attributeWedgeUVs:(NSArray<NSNumber *> *)attributeWedgeUVs
                       attributeFaceMaterials:(NSArray<NSNumber *> *)attributeFaceMaterials
                         attributeFaceAnchors:(NSArray<NSString *> *)attributeFaceAnchors;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Detached candidate bytes. Positions and indices are explicitly measured
//! again by the caller before admission. NORMAL is f2-normal-regen.v1 flat
//! per-corner data; materialBytes is an exact copy of the sole source domain.
__attribute__((objc_subclassing_restricted))
@interface Core3DDecimationResult : NSObject
@property(nonatomic, copy, readonly) NSData *positionFloat32Data;
@property(nonatomic, copy, readonly) NSData *normalFloat32Data;
@property(nonatomic, copy, readonly) NSData *indexUInt32Data;
@property(nonatomic, copy, readonly) NSData *materialBytes;
@property(nonatomic, copy, readonly) NSArray<NSArray<NSString *> *> *sourceContributionTokens;
@property(nonatomic, copy, readonly) NSString *normalPolicy;
@property(nonatomic, copy, readonly) NSString *appearanceLimitation;
@property(nonatomic, assign, readonly) NSUInteger triangleCount;
@property(nonatomic, assign, readonly) NSUInteger acceptedCollapseCount;
//! Typed read-only output of the F2-S3 attribute path. All six values are nil
//! on the original flat path and nonnil together on an attribute result.
@property(nonatomic, copy, readonly, nullable) NSData *f2AttributeUVFloat32Data;
@property(nonatomic, copy, readonly, nullable) NSArray<NSNumber *> *f2AttributeMaterialIDs;
@property(nonatomic, copy, readonly, nullable) NSArray<NSString *> *f2AttributeSourceFaceAnchors;
@property(nonatomic, copy, readonly, nullable) NSArray<NSString *> *f2AttributeTriangleContentSHA256;
@property(nonatomic, copy, readonly, nullable) NSArray<NSNumber *> *f2AttributeCollapseKeepRemove;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Immutable native decode of the complete canonical settings payload. Fixed
//! identities are exposed with the variable values so callers never infer
//! policy from defaults or duplicate the native codec.
__attribute__((objc_subclassing_restricted))
@interface Core3DDecimationSettingsReadback : NSObject
@property(nonatomic, copy, readonly) NSData *canonicalBytes;
@property(nonatomic, copy, readonly) NSString *settingsSHA256;
@property(nonatomic, assign, readonly) NSUInteger targetTriangleCount;
@property(nonatomic, assign, readonly) double maximumSymmetricSurfaceDistanceMM;
@property(nonatomic, assign, readonly) double maximumSilhouettePixelDeviation;
@property(nonatomic, assign, readonly) double hardEdgeAngleDegrees;
@property(nonatomic, assign, readonly) double contactClearanceMM;
@property(nonatomic, assign, readonly) NSUInteger measurementSamplesPerMesh;
@property(nonatomic, assign, readonly) NSUInteger silhouetteResolution;
@property(nonatomic, copy, readonly) NSString *silhouetteCameraSetSHA256;
@property(nonatomic, assign, readonly) double metersPerUnit;
@property(nonatomic, copy, readonly) NSString *schemaVersion;
@property(nonatomic, copy, readonly) NSString *algorithmIdentifier;
@property(nonatomic, copy, readonly) NSString *inputPolicy;
@property(nonatomic, copy, readonly) NSString *coordinateFramePolicy;
@property(nonatomic, copy, readonly) NSString *normalPolicy;
@property(nonatomic, copy, readonly) NSString *arithmeticPolicy;
@property(nonatomic, copy, readonly) NSString *qualityProfileIdentifier;
@property(nonatomic, copy, readonly) NSString *surfaceDistanceMethod;
@property(nonatomic, copy, readonly) NSString *silhouetteMethod;
@property(nonatomic, copy, readonly) NSString *contactMethod;
@property(nonatomic, copy, readonly) NSString *hardEdgeMethod;
@property(nonatomic, copy, readonly) NSString *cameraMethod;
@property(nonatomic, copy, readonly) NSString *algorithmBuildSHA256;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! F2-S2 persisted derivative provenance record, decoded from the document's
//! F2DR store. Digests are lowercase 64-hex SHA-256 content witnesses; status
//! is canonical text (Current/0 or Stale/1 pairs only). Every stored row is a
//! historical observation until live revalidation succeeds, including when
//! the stored status is Current.
__attribute__((objc_subclassing_restricted))
@interface Core3DPersistedDecimationDerivative : NSObject
@property(nonatomic, copy, readonly) NSString *derivativeIdentifier;      // 64-lower-hex
@property(nonatomic, copy, readonly) NSString *documentIdentifier;
@property(nonatomic, copy, readonly) NSString *sourceRevisionSHA256;
@property(nonatomic, copy, readonly) NSString *sourceArtifactSHA256;
@property(nonatomic, copy, readonly) NSString *sourceGeometrySHA256;
@property(nonatomic, copy, readonly) NSString *settingsSHA256;
@property(nonatomic, copy, readonly) NSData *settingsCanonicalBytes;
@property(nonatomic, copy, readonly) NSString *outputGeometrySHA256;
@property(nonatomic, copy, readonly) NSString *remapReceiptSHA256;
@property(nonatomic, copy, readonly) NSString *disclosure;
@property(nonatomic, copy, readonly) NSString *status;                    // @"Current" or @"Stale"
@property(nonatomic, assign, readonly) BOOL historicalObservation;
@property(nonatomic, assign, readonly) NSUInteger emittedTriangleCount;
@property(nonatomic, assign, readonly) NSUInteger level;                  // 0...7
#if DEBUG
//! Probe-only text form of the exact stored key; never used for persistence.
@property(nonatomic, copy, readonly) NSString *ownerSetKey;               // 64-lower-hex
#endif
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Request-owned, read-only opening. When constructed with a publish capture,
//! the complete native source fence is checked before and after mesh work. A
//! nil capture is reserved for literal native kernel vectors and cannot confer
//! Current status on a document derivative.
__attribute__((objc_subclassing_restricted))
@interface Core3DDecimationOpening : NSObject
+ (nullable instancetype)openingWithCapture:(Core3DPublishCapture * _Nullable)capture
                                      error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(init(capture:));
- (nullable Core3DDecimationResult *)decimateRequest:(Core3DDecimationRequest *)request
                                                error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(decimate(_:));
//! Sticky for this opening and never affects another request.
//! For asynchronous execution this only signals Stop; the completion is the
//! join and is delivered exactly once on main after numerical work has exited.
- (BOOL)cancel;
//! Capture-bound asynchronous execution. The immutable producer tuple and
//! native input are copied and fenced on main, only numerical mesh work runs
//! off-main, and the final currentness/evidence fence plus private seal happen
//! on main. A nil-capture opening cannot use this entry point.
- (void)decimateCapturedRequest:(Core3DDecimationRequest *)request
    completion:(void (^)(Core3DDecimationResult * _Nullable result,
                         NSError * _Nullable error))completion
    NS_SWIFT_NAME(decimateCaptured(_:completion:));
#if DEBUG
typedef NS_ENUM(NSUInteger, Core3DDecimationAsyncBoundary) {
    Core3DDecimationAsyncBoundaryBeforeNumericalWork = 1,
    Core3DDecimationAsyncBoundaryAfterNumericalWork = 2,
    Core3DDecimationAsyncBoundaryBeforeFinalFence = 3,
};
//! Scheduling observation only. The handler must invoke resume exactly once.
@property(nonatomic, copy, nullable) void (^f2DebugAsyncBoundaryHandler)(
    Core3DDecimationAsyncBoundary boundary, dispatch_block_t resume);
#endif

//! F2-S2 canonical complete settings bytes: target, input policy, quality
//! values, method/profile/camera identities, unit/frame and algorithm/build
//! policy. The settings digest is never part of the bytes. Non-finite or
//! negative-zero quality values are refused (nil).
+ (nullable NSData *)f2CanonicalSettingsBytesWithTargetTriangleCount:(NSUInteger)targetTriangleCount
                                maximumSymmetricSurfaceDistanceMM:(double)maximumSymmetricSurfaceDistanceMM
                                  maximumSilhouettePixelDeviation:(double)maximumSilhouettePixelDeviation
                                               hardEdgeAngleDegrees:(double)hardEdgeAngleDegrees
                                                contactClearanceMM:(double)contactClearanceMM
                                           measurementSamplesPerMesh:(NSUInteger)measurementSamplesPerMesh
                                                silhouetteResolution:(NSUInteger)silhouetteResolution
                                          silhouetteCameraSetSHA256:(NSString *)silhouetteCameraSetSHA256
                                                      metersPerUnit:(double)metersPerUnit
    NS_SWIFT_NAME(f2CanonicalSettingsBytes(targetTriangleCount:maximumSymmetricSurfaceDistanceMM:maximumSilhouettePixelDeviation:hardEdgeAngleDegrees:contactClearanceMM:measurementSamplesPerMesh:silhouetteResolution:silhouetteCameraSetSHA256:metersPerUnit:));
//! Recomputes the canonical settings digest from bytes; nil on malformed bytes.
+ (nullable NSString *)f2SettingsSHA256ForCanonicalBytes:(NSData *)bytes
    NS_SWIFT_NAME(f2SettingsSHA256(forCanonicalBytes:));

//! Decodes canonical bytes through the production native codec. Malformed,
//! trailing, non-finite or noncanonical settings refuse without a partial DTO.
+ (nullable Core3DDecimationSettingsReadback *)f2SettingsReadbackForCanonicalBytes:(NSData *)bytes
    error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(f2SettingsReadback(forCanonicalBytes:));

//! Persists the derivative record for the opening's owner set. Persistence
//! replays only a result this opening sealed under the same source witness:
//! the result must carry the producer seal from a decimate call whose capture
//! matches this capture's document and source witness bytes, the request must
//! equal the sealed producer input, and the capture must carry bound artifact
//! evidence (or the sealed evidence of the same source epoch). A fresh
//! capture may repeat an identical sealed candidate as a no-op: identical
//! candidate store bytes return the existing row with zero commands and zero
//! history delta. The write is one owned OCAF command; any uncertain closure
//! refuses with Core3DDecimationErrorRecoveryRequired.
- (nullable Core3DPersistedDecimationDerivative *)f2PersistDerivativeForController:(Core3DViewController *)controller
    request:(Core3DDecimationRequest *)request result:(Core3DDecimationResult *)result
    settingsCanonicalBytes:(NSData *)settingsCanonicalBytes error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(f2PersistDerivative(controller:request:result:settingsCanonicalBytes:));
//! Validates the same sealed F2 producer tuple as the ordinary persist path,
//! derives the requested level key (0...7), and stages the exact F2DR store
//! inside the caller's already-open native command. It never opens, commits,
//! aborts, or finalizes history. The caller must have fenced the capture
//! immediately before opening its command and must abort the whole command on
//! nil. Existing level-zero persistence and restore APIs are unchanged.
- (nullable Core3DPersistedDecimationDerivative *)f2StageDerivativeForController:(Core3DViewController *)controller
    level:(NSUInteger)level request:(Core3DDecimationRequest *)request
    result:(Core3DDecimationResult *)result settingsCanonicalBytes:(NSData *)settingsCanonicalBytes
    error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(f2StageDerivative(controller:level:request:result:settingsCanonicalBytes:));
//! Read-only restore predicate: the stored row exists, is Current/0, and its
//! source-revision and settings witnesses recompute equal against the current
//! capture and the supplied canonical settings bytes. Never writes; any
//! failure answers NO.
- (BOOL)f2CanRestorePersistedDerivativeForController:(Core3DViewController *)controller
    settingsCanonicalBytes:(NSData *)settingsCanonicalBytes
    NS_SWIFT_NAME(f2CanRestorePersistedDerivative(controller:settingsCanonicalBytes:));
//! Explicit stale validation: returns the DTO when the restore predicate
//! holds (no command). Otherwise records Stale/1 in one owned command,
//! preserving every other field byte-exactly, and returns nil with
//! Core3DDecimationErrorStaleSource; stale-again is a no-op returning the
//! same typed refusal.
- (nullable Core3DPersistedDecimationDerivative *)f2ValidatePersistedDerivativeForController:(Core3DViewController *)controller
    settingsCanonicalBytes:(NSData *)settingsCanonicalBytes error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(f2ValidatePersistedDerivative(controller:settingsCanonicalBytes:));
//! Raw historical read: returns the stored row exactly as persisted,
//! including its historical status fields, without fencing on capture
//! currentness. Absent row or store refuses with
//! Core3DDecimationErrorDerivativeNotPersisted; a corrupt store refuses with
//! Core3DDecimationErrorDerivativePersistenceUnavailable.
- (nullable Core3DPersistedDecimationDerivative *)f2PersistedDerivativeForController:(Core3DViewController *)controller
    error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(f2PersistedDerivative(controller:));
#if DEBUG
//! Read-only codec probe. Clones the controller's sole persisted level-0 row
//! at the requested levels, recomputes its key and derivative identity, and
//! returns canonical F2DR bytes. It never writes the document or carrier.
+ (nullable NSData *)f2DebugEncodeDerivativeStoreForController:(Core3DViewController *)controller
    levels:(NSArray<NSNumber *> *)levels error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(f2DebugEncodeDerivativeStore(controller:levels:));
//! Decodes schema-1 or schema-2 F2DR bytes through the production codec.
+ (nullable NSArray<Core3DPersistedDecimationDerivative *> *)f2DebugDecodeDerivativeStoreBytes:(NSData *)bytes
    error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(f2DebugDecodeDerivativeStore(_:));
//! Decodes then canonically re-encodes F2DR bytes through the production codec.
+ (nullable NSData *)f2DebugReencodeDerivativeStoreBytes:(NSData *)bytes
    error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(f2DebugReencodeDerivativeStore(_:));
#endif
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

NS_ASSUME_NONNULL_END
