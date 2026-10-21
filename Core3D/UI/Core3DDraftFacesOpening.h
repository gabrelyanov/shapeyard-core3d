#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DViewController;
@class Core3DDraftFacesOpening;
@class Core3DDraftFacesCandidate;

FOUNDATION_EXPORT NSErrorDomain const Core3DDraftFacesErrorDomain;
FOUNDATION_EXPORT NSString *const Core3DDraftFacesRefusalReasonKey;
FOUNDATION_EXPORT NSString *const Core3DDraftFacesRefusalRawValueKey;

typedef NS_ERROR_ENUM(Core3DDraftFacesErrorDomain, Core3DDraftFacesError) {
    Core3DDraftFacesErrorInvalidThread = 1,
    Core3DDraftFacesErrorUnavailable,
    Core3DDraftFacesErrorInvalidArgument,
    Core3DDraftFacesErrorInvalidSelection,
    Core3DDraftFacesErrorStale,
    Core3DDraftFacesErrorCancelled,
    Core3DDraftFacesErrorBudget,
    Core3DDraftFacesErrorBuildRefused,
    Core3DDraftFacesErrorApplyUnavailable,
};

typedef NS_ENUM(NSInteger, Core3DDraftFacesSurfaceKind) {
    Core3DDraftFacesSurfaceKindPlanar = 1,
    Core3DDraftFacesSurfaceKindCylindrical = 2,
    Core3DDraftFacesSurfaceKindConical = 3,
    Core3DDraftFacesSurfaceKindUnsupported = 255,
};

typedef NS_ENUM(NSInteger, Core3DDraftFacesApplyResult) {
    Core3DDraftFacesApplyResultUnavailable = 0,
};

//! Complete terminal/recovery classification returned by the retained S2
//! owner. Raw values intentionally match draft_faces::owner::Outcome.
typedef NS_ENUM(NSInteger, Core3DDraftFacesNativeOutcome) {
    Core3DDraftFacesNativeOutcomeRejected = 0,
    Core3DDraftFacesNativeOutcomeCancelled,
    Core3DDraftFacesNativeOutcomeUnchanged,
    Core3DDraftFacesNativeOutcomeCommitted,
    Core3DDraftFacesNativeOutcomeRecoveryRequired,
};

__attribute__((objc_subclassing_restricted))
@interface Core3DDraftFacesNativeReceipt : NSObject
@property(nonatomic, assign, readonly) Core3DDraftFacesNativeOutcome outcome;
@property(nonatomic, copy, readonly) NSString *reason;
@property(nonatomic, assign, readonly) uint64_t recoverySession;
@property(nonatomic, assign, readonly) NSInteger measuredHistoryDelta;
@property(nonatomic, assign, readonly, getter=isSettled) BOOL settled;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! One native-issued semantic face receipt. Coordinates are physical mm;
//! normals are unitless. `selected` describes the independently read SYDF/1
//! record, not caller state.
__attribute__((objc_subclassing_restricted))
@interface Core3DDraftFacesNativeFaceSnapshot : NSObject
@property(nonatomic, copy, readonly) NSData *identifier;
@property(nonatomic, copy, readonly) NSString *selectorDescription;
@property(nonatomic, assign, readonly) Core3DDraftFacesSurfaceKind surfaceKind;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *sampleOnNeutralMM;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *outwardNormal;
@property(nonatomic, assign, readonly, getter=isSelected) BOOL selected;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Independently read retained Draft Faces state. `revision` is the exact
//! canonical SYCR record bytes and is required again at prepare and commit.
__attribute__((objc_subclassing_restricted))
@interface Core3DDraftFacesNativeSnapshot : NSObject
@property(nonatomic, copy, readonly) NSData *ownerEntityIdentifier;
@property(nonatomic, copy, readonly) NSData *ownerDefinitionIdentifier;
@property(nonatomic, copy, readonly) NSData *baseFeatureIdentifier;
@property(nonatomic, copy, readonly) NSData *featureIdentifier;
@property(nonatomic, copy, readonly) NSArray<Core3DDraftFacesNativeFaceSnapshot *> *faces;
@property(nonatomic, copy, readonly) NSArray<Core3DDraftFacesNativeFaceSnapshot *> *availableFaces;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *neutralOriginMM;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *neutralNormal;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *pullDirection;
@property(nonatomic, assign, readonly) double signedAngleDegrees;
@property(nonatomic, copy, readonly) NSArray<NSData *> *orderedDownstreamTreatmentIdentifiers;
@property(nonatomic, assign, readonly) double metersPerUnit;
@property(nonatomic, copy, readonly) NSData *revision;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Release-safe production bridge over the one retained NativeOwner for a live
//! document. It never grants Objects or AI admission. Preview mints a frozen
//! edit capture; commit consumes only that capture at the exact read revision.
__attribute__((objc_subclassing_restricted))
@interface Core3DDraftFacesNativeEditingBridge : NSObject
+ (nullable instancetype)captureController:(Core3DViewController *)controller
                          entityIdentifier:(NSString *)entityIdentifier
                                      error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(capture(controller:entityIdentifier:));
//! Captures create authority from one current S1 opening and the exact
//! detached candidate it issued. No command is opened until `commitCreate`.
+ (nullable instancetype)captureCreateController:(Core3DViewController *)controller
                                         opening:(Core3DDraftFacesOpening *)opening
                                       candidate:(Core3DDraftFacesCandidate *)candidate
                                           error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(captureCreate(controller:opening:candidate:));
- (nullable Core3DDraftFacesNativeSnapshot *)readSnapshotWithError:
    (NSError * _Nullable * _Nullable)error NS_SWIFT_NAME(readSnapshot());
- (nullable Core3DDraftFacesNativeSnapshot *)prepareFeatureIdentifier:(NSData *)featureIdentifier
                                                     expectedRevision:(NSData *)expectedRevision
                                               selectedFaceIdentifiers:(NSArray<NSData *> *)selectedFaceIdentifiers
                                                       neutralOriginMM:(NSArray<NSNumber *> *)neutralOriginMM
                                                         neutralNormal:(NSArray<NSNumber *> *)neutralNormal
                                                         pullDirection:(NSArray<NSNumber *> *)pullDirection
                                                    signedAngleDegrees:(double)signedAngleDegrees
                                                                 error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(prepare(featureIdentifier:expectedRevision:selectedFaceIdentifiers:neutralOriginMM:neutralNormal:pullDirection:signedAngleDegrees:));
- (nullable Core3DDraftFacesNativeReceipt *)commitFeatureIdentifier:(NSData *)featureIdentifier
                                                   expectedRevision:(NSData *)expectedRevision
                                                              error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(commit(featureIdentifier:expectedRevision:));
//! Consumes the pending create capture with a live production Stop signal.
- (nullable Core3DDraftFacesNativeReceipt *)commitCreateWithError:
    (NSError * _Nullable * _Nullable)error NS_SWIFT_NAME(commitCreate());
//! Settles a pending create as a native Cancelled outcome without history.
- (nullable Core3DDraftFacesNativeReceipt *)cancelCreateWithError:
    (NSError * _Nullable * _Nullable)error NS_SWIFT_NAME(cancelCreate());
- (void)cancelPrepared;
- (Core3DDraftFacesNativeReceipt *)reconcileRecoverySession:(uint64_t)session
    NS_SWIFT_NAME(reconcile(session:));
@property(nonatomic, assign, readonly) BOOL blocksOtherWork;
@property(nonatomic, assign, readonly) double metersPerUnit;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Native-issued identity for a face in the captured selection. The receipt is
//! document/source/topology bound and cannot be constructed by callers.
__attribute__((objc_subclassing_restricted))
@interface Core3DDraftFacesIssuedFace : NSObject
@property(nonatomic, copy, readonly) NSData *identifier;
@property(nonatomic, copy, readonly) NSData *selectorProof;
@property(nonatomic, copy, readonly) NSString *entityIdentifier;
@property(nonatomic, assign, readonly) uint32_t topologyIndex;
@property(nonatomic, assign, readonly) Core3DDraftFacesSurfaceKind surfaceKind;
@property(nonatomic, assign, readonly) uint16_t boundaryEdgeCount;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *outwardNormalLocal;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Observation made from the emitted candidate topology. Values are physical
//! millimetres; request values are not exposed as observations.
__attribute__((objc_subclassing_restricted))
@interface Core3DDraftFacesSectionObservation : NSObject
@property(nonatomic, copy, readonly) NSData *faceIdentifier;
@property(nonatomic, assign, readonly) double pullOffsetMM;
@property(nonatomic, assign, readonly) double measuredNormalOffsetMM;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Detached S1 result. It owns no document command, retained owner, history or
//! publication authority.
__attribute__((objc_subclassing_restricted))
@interface Core3DDraftFacesCandidate : NSObject
@property(nonatomic, copy, readonly) NSData *candidateBRepBytes;
@property(nonatomic, copy, readonly) NSData *geometryDigest;
@property(nonatomic, copy, readonly) NSData *familyProof;
@property(nonatomic, copy, readonly) NSData *canonicalDefinitionBytes;
@property(nonatomic, copy, readonly) NSArray<NSData *> *consumedFaceIdentifiers;
@property(nonatomic, copy, readonly) NSArray<Core3DDraftFacesSectionObservation *> *sectionObservations;
@property(nonatomic, assign, readonly) double metersPerUnit;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *neutralOriginMM;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *neutralNormal;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *pullDirection;
@property(nonatomic, assign, readonly) double signedAngleDegrees;
@property(nonatomic, assign, readonly) double measuredSignedAngleRadians;
@property(nonatomic, assign, readonly) double neutralSectionPerimeterMM;
@property(nonatomic, assign, readonly) NSUInteger neutralSectionEdgeCount;
@property(nonatomic, assign, readonly) NSUInteger connectedSolidCount;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Viewer-issued opening bound to the captured document/source/face-selection
//! fence. All S1 work is detached; recapture mismatch suppresses delivery.
__attribute__((objc_subclassing_restricted))
@interface Core3DDraftFacesOpening : NSObject
@property(nonatomic, copy, readonly) NSArray<Core3DDraftFacesIssuedFace *> *issuedFaces;
@property(nonatomic, copy, readonly) NSString *documentIdentifier;
@property(nonatomic, copy, readonly) NSString *contextIdentifier;
@property(nonatomic, assign, readonly) double metersPerUnit;
@property(nonatomic, assign, readonly) NSInteger historyDepth;

+ (nullable instancetype)captureController:(Core3DViewController *)controller
                                     error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(capture(controller:));

- (nullable Core3DDraftFacesCandidate *)detachedCandidateWithNeutralOriginLocal:(NSArray<NSNumber *> *)neutralOriginLocal
                                                              neutralNormalLocal:(NSArray<NSNumber *> *)neutralNormalLocal
                                                               pullDirectionLocal:(NSArray<NSNumber *> *)pullDirectionLocal
                                                                 signedAngleRadians:(double)signedAngleRadians
                                                                              error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(detachedCandidate(neutralOriginLocal:neutralNormalLocal:pullDirectionLocal:signedAngleRadians:));

- (BOOL)isCurrent:(NSError * _Nullable * _Nullable)error;
- (void)cancel;

//! Reserved S2 lifecycle seams. They are deliberately unavailable in S1.
- (nullable Core3DDraftFacesOpening *)openingByPreparingCandidate:(Core3DDraftFacesCandidate *)candidate
                                                            error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(preparing(candidate:));
- (Core3DDraftFacesApplyResult)apply;
+ (BOOL)isPubliclyAvailable;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

NS_ASSUME_NONNULL_END
