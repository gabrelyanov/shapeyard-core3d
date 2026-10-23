#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DPublishCapture;
@class Core3DViewController;
@class Core3DDecimationResult;

FOUNDATION_EXPORT NSErrorDomain const Core3DLODSetErrorDomain;

typedef NS_ERROR_ENUM(Core3DLODSetErrorDomain, Core3DLODSetError) {
    Core3DLODSetErrorUnavailable = 1,
    Core3DLODSetErrorMissingProvenance,
    Core3DLODSetErrorInvalidDefinition,
    Core3DLODSetErrorForeignIdentity,
    Core3DLODSetErrorStaleSource,
    Core3DLODSetErrorStopped,
    Core3DLODSetErrorWorkBudget,
    Core3DLODSetErrorAttachmentParity,
    Core3DLODSetErrorPersistenceUnavailable,
    Core3DLODSetErrorCorruptRecord,
    Core3DLODSetErrorNotFound,
    Core3DLODSetErrorRecoveryRequired,
};

typedef NS_ENUM(NSInteger, Core3DLODSetCommitResult) {
    Core3DLODSetCommitResultUnavailable = 0,
    Core3DLODSetCommitResultCommitted,
    Core3DLODSetCommitResultRemoved,
    Core3DLODSetCommitResultRegenerateUnavailable,
    Core3DLODSetCommitResultRecoveryRequired,
};

typedef NS_ENUM(NSInteger, Core3DLODSetDurableState) {
    Core3DLODSetDurableStateUnbuilt = 0,
    Core3DLODSetDurableStateCurrent,
    Core3DLODSetDurableStateStale,
};

#if DEBUG
typedef NS_ENUM(NSInteger, Core3DLODSetDebugFault) {
    Core3DLODSetDebugFaultNone = 0,
    Core3DLODSetDebugFaultBeforeStage,
    Core3DLODSetDebugFaultAfterF2Stage,
    Core3DLODSetDebugFaultAfterSYLSStage,
    Core3DLODSetDebugFaultAfterReadback,
    Core3DLODSetDebugFaultUnknownClose,
};
#endif

__attribute__((objc_subclassing_restricted))
@interface Core3DLODLevelRecord : NSObject
@property(nonatomic, copy, readonly) NSString *identifier;
@property(nonatomic, copy, readonly) NSString *displayName;
@property(nonatomic, assign, readonly) NSUInteger ordinal;
@property(nonatomic, assign, readonly) NSUInteger targetTriangleCount;
@property(nonatomic, assign, readonly) Core3DLODSetDurableState state;
@property(nonatomic, copy, readonly) NSString *reason;
@property(nonatomic, assign, readonly) double maximumSurfaceDistanceMM;
@property(nonatomic, assign, readonly) double maximumSilhouetteDeviationPixels;
@property(nonatomic, assign, readonly) double hardEdgeAngleDegrees;
@property(nonatomic, assign, readonly) double contactClearanceMM;
@property(nonatomic, assign, readonly) NSUInteger measurementSampleCount;
@property(nonatomic, assign, readonly) NSUInteger silhouetteResolution;
@property(nonatomic, copy, readonly) NSString *silhouetteCameraSetSHA256;
@property(nonatomic, assign, readonly) double maximumUVStretch;
@property(nonatomic, assign, readonly) double maximumNormalDeviationDegrees;
@property(nonatomic, assign, readonly) NSUInteger uvMethodVersion;
@property(nonatomic, assign, readonly) NSUInteger normalMethodVersion;
@property(nonatomic, assign, readonly) NSUInteger overlapMethodVersion;
@property(nonatomic, copy, readonly, nullable) NSString *f2DerivativeIdentifier;
@property(nonatomic, copy, readonly, nullable) NSString *f2SettingsSHA256;
@property(nonatomic, assign, readonly) NSUInteger measuredTriangleCount;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DLODSetRecord : NSObject
@property(nonatomic, copy, readonly) NSString *identifier;
@property(nonatomic, copy, readonly) NSString *documentIdentifier;
@property(nonatomic, assign, readonly) NSUInteger revision;
@property(nonatomic, assign, readonly) double metersPerUnit;
@property(nonatomic, assign, readonly) Core3DLODSetDurableState state;
@property(nonatomic, copy, readonly) NSString *reason;
@property(nonatomic, copy, readonly) NSString *observedSourceSHA256;
@property(nonatomic, copy, readonly, nullable) NSString *lastBuiltSourceSHA256;
@property(nonatomic, copy, readonly) NSArray<Core3DLODLevelRecord *> *levels;
@property(nonatomic, copy, readonly) NSData *canonicalSYLSRecord;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Opaque identities minted by a native opening. Neither type has a public
//! constructor, so caller data cannot become an LOD-set or level identity.
__attribute__((objc_subclassing_restricted))
@interface Core3DLODSetHandle : NSObject
@property(nonatomic, copy, readonly) NSString *identifier;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DLODLevelHandle : NSObject
@property(nonatomic, copy, readonly) NSString *identifier;
@property(nonatomic, assign, readonly) NSUInteger ordinal;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Canonical attachment evidence. Empty attachment arrays are a valid exact
//! binding; when present, every level must carry this same ordered vector.
__attribute__((objc_subclassing_restricted))
@interface Core3DLODAttachmentBinding : NSObject
@property(nonatomic, copy, readonly) NSString *attachmentIdentifier;
@property(nonatomic, copy, readonly) NSString *canonicalMetreFrameSHA256;
- (instancetype)initWithAttachmentIdentifier:(NSString *)attachmentIdentifier
                 canonicalMetreFrameSHA256:(NSString *)canonicalMetreFrameSHA256
    NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Caller intent for one native-issued slot. The opening validates all scalar,
//! digest, identity and ordering fields before returning an immutable draft.
__attribute__((objc_subclassing_restricted))
@interface Core3DLODLevelRequest : NSObject
@property(nonatomic, strong, readonly) Core3DLODLevelHandle *handle;
@property(nonatomic, copy, readonly) NSString *displayName;
@property(nonatomic, assign, readonly) NSUInteger targetTriangleCount;
@property(nonatomic, assign, readonly) double maximumSurfaceDistanceMM;
@property(nonatomic, assign, readonly) double maximumSilhouetteDeviationPixels;
@property(nonatomic, assign, readonly) double hardEdgeAngleDegrees;
@property(nonatomic, assign, readonly) double contactClearanceMM;
@property(nonatomic, assign, readonly) NSUInteger measurementSampleCount;
@property(nonatomic, assign, readonly) NSUInteger silhouetteResolution;
@property(nonatomic, copy, readonly) NSString *silhouetteCameraSetSHA256;
@property(nonatomic, copy, readonly) NSString *settingsSHA256;
@property(nonatomic, copy, readonly) NSArray<Core3DLODAttachmentBinding *> *attachments;
- (instancetype)initWithHandle:(Core3DLODLevelHandle *)handle
                    displayName:(NSString *)displayName
            targetTriangleCount:(NSUInteger)targetTriangleCount
       maximumSurfaceDistanceMM:(double)maximumSurfaceDistanceMM
maximumSilhouetteDeviationPixels:(double)maximumSilhouetteDeviationPixels
           hardEdgeAngleDegrees:(double)hardEdgeAngleDegrees
             contactClearanceMM:(double)contactClearanceMM
         measurementSampleCount:(NSUInteger)measurementSampleCount
           silhouetteResolution:(NSUInteger)silhouetteResolution
      silhouetteCameraSetSHA256:(NSString *)silhouetteCameraSetSHA256
                 settingsSHA256:(NSString *)settingsSHA256
                    attachments:(NSArray<Core3DLODAttachmentBinding *> *)attachments
    NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Validated detached value. It has no command lease, document owner, F2
//! receipt, artifact manifest, persistence or Current-set authority.
__attribute__((objc_subclassing_restricted))
@interface Core3DLODSetDraft : NSObject
@property(nonatomic, strong, readonly) Core3DLODSetHandle *setHandle;
@property(nonatomic, copy, readonly) NSArray<Core3DLODLevelRequest *> *levels;
@property(nonatomic, copy, readonly) NSString *sourceDocumentIdentifier;
@property(nonatomic, copy, readonly) NSString *boundSourceRevisionSHA256;
@property(nonatomic, copy, readonly) NSString *observedSourceRevisionSHA256;
@property(nonatomic, copy, readonly) NSString *attachmentMetadataSHA256;
@property(nonatomic, copy, readonly) NSArray<Core3DLODAttachmentBinding *> *expectedAttachments;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Source-bound S1 opening. All work is detached, bounded and guarded by the
//! exact Core3DPublishCapture fence before and after native validation.
__attribute__((objc_subclassing_restricted))
@interface Core3DLODSetOpening : NSObject
@property(nonatomic, strong, readonly) Core3DLODSetHandle *setHandle;
@property(nonatomic, copy, readonly) NSArray<Core3DLODLevelHandle *> *levelHandles;
@property(nonatomic, copy, readonly) NSString *sourceRevisionSHA256;
@property(nonatomic, assign, readonly) NSUInteger completedWorkUnits;
@property(nonatomic, assign, readonly) NSUInteger workUnitLimit;

+ (nullable instancetype)openingWithCapture:(Core3DPublishCapture * _Nullable)capture
                                      error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(init(capture:));
- (nullable Core3DLODSetDraft *)draftWithLevels:(NSArray<Core3DLODLevelRequest *> *)levels
                       attachmentMetadataSHA256:(NSString *)attachmentMetadataSHA256
                            expectedAttachments:(NSArray<Core3DLODAttachmentBinding *> *)expectedAttachments
                                          error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(draft(levels:attachmentMetadataSHA256:expectedAttachments:));
//! Explicitly binds a detached value to one newly captured revision. Only the
//! source commitments change; native set/level identities remain fixed.
- (nullable Core3DLODSetDraft *)rebindDraft:(Core3DLODSetDraft *)draft
                                  toCapture:(Core3DPublishCapture *)capture
                                      error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(rebind(_:to:));
- (BOOL)isCurrent:(NSError * _Nullable * _Nullable)error;
- (BOOL)stop;

//! S2 owner ABI reservation. S1 always reports Unavailable and performs no
//! command, persistence, manifest creation or Current-set installation.
- (Core3DLODSetCommitResult)commitDraft:(Core3DLODSetDraft *)draft
    NS_SWIFT_NAME(commit(_:));

//! Controller-bound durable owner. The detached overload above remains the S1
//! guard and always answers Unavailable.
+ (nullable instancetype)openingWithCapture:(Core3DPublishCapture *)capture
                                 controller:(Core3DViewController *)controller
                      existingSetIdentifier:(nullable NSString *)existingSetIdentifier
                                      error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(init(capture:controller:existingSetIdentifier:));
- (Core3DLODSetCommitResult)commitDraft:(Core3DLODSetDraft *)draft
                              controller:(Core3DViewController *)controller
                                   error:(NSError * _Nullable * _Nullable)error
    __attribute__((swift_error(nonnull_error)))
    NS_SWIFT_NAME(commit(_:controller:));
+ (nullable NSArray<Core3DLODSetRecord *> *)persistedSetsForController:
    (Core3DViewController *)controller error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(persistedSets(controller:));
- (Core3DLODSetCommitResult)removeFromController:(Core3DViewController *)controller
                                           error:(NSError * _Nullable * _Nullable)error
    __attribute__((swift_error(nonnull_error)))
    NS_SWIFT_NAME(remove(controller:));
- (Core3DLODSetCommitResult)regenerateForController:(Core3DViewController *)controller
                                               error:(NSError * _Nullable * _Nullable)error
    __attribute__((swift_error(nonnull_error)))
    NS_SWIFT_NAME(regenerate(controller:));
//! Binds one already-persisted Current F2 row by exact production readback.
//! This does not generate an artifact; rows for other levels remain Unbuilt.
- (Core3DLODSetCommitResult)referencePersistedF2Level:(NSUInteger)level
                                             artifactName:(NSString *)artifactName
                                               controller:(Core3DViewController *)controller
                                                    error:(NSError * _Nullable * _Nullable)error
    __attribute__((swift_error(nonnull_error)))
    NS_SWIFT_NAME(referencePersistedF2(level:artifactName:controller:));

//! Production codec surface used by F05. Validation and re-encoding execute
//! the same strict SYLS/1 decoder used by ordinary document reads.
+ (nullable NSData *)reencodeSYLSRecord:(NSData *)record
                                   error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(reencodeSYLS(_:));
+ (NSUInteger)maximumSYLSRecordBytes;

#if DEBUG
//! Called from the real bounded validation loop. Tests can synchronously Stop
//! at a work boundary; Release contains neither this hook nor its branch.
@property(nonatomic, copy, nullable) void (^debugWorkBoundaryHandler)(NSUInteger completedWorkUnits);
//! Volatile observation of the same private all-level candidate helper used by
//! regenerate. It accepts one immutable synthetic source and returns detached
//! results only; it has no controller, command, F2DR or SYLS write surface.
- (nullable NSArray<Core3DDecimationResult *> *)debugCandidatesForLevels:
    (NSArray<Core3DLODLevelRequest *> *)levels
    positionsMM:(NSArray<NSNumber *> *)positionsMM
    indices:(NSArray<NSNumber *> *)indices
    sourceContributionTokens:(NSArray<NSString *> *)sourceContributionTokens
    materialBytes:(NSData *)materialBytes
    sourceArtifactSHA256:(NSString *)sourceArtifactSHA256
    metersPerUnit:(double)metersPerUnit
    error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(debugCandidates(levels:positionsMM:indices:sourceContributionTokens:materialBytes:sourceArtifactSHA256:metersPerUnit:));
+ (void)setDebugFault:(Core3DLODSetDebugFault)fault NS_SWIFT_NAME(setDebugFault(_:));
+ (BOOL)debugRemoveF2StoreForController:(Core3DViewController *)controller
                                  error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(debugRemoveF2Store(controller:));
#endif

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

NS_ASSUME_NONNULL_END
