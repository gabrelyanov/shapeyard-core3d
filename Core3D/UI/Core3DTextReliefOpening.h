#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DViewController;

//! E5-S1 text and emblem relief error domain. S1 issues a detached native
//! raised candidate only; no retained owner, document transaction,
//! CommandLease, publication or history entry exists at this stage.
FOUNDATION_EXPORT NSErrorDomain const Core3DTextReliefErrorDomain;

//! NSError userInfo key carrying the exact native refusal string
//! (for example @"e5.resource-hash-mismatch") whenever the refusal originates
//! from the native resource-admission or raised-build boundary.
FOUNDATION_EXPORT NSString *const Core3DTextReliefRefusalReasonKey;

//! NSError userInfo key carrying the native refusal ordinal (NSNumber int).
FOUNDATION_EXPORT NSString *const Core3DTextReliefRefusalRawValueKey;

typedef NS_ERROR_ENUM(Core3DTextReliefErrorDomain, Core3DTextReliefError) {
    Core3DTextReliefErrorInvalidThread = 1,
    Core3DTextReliefErrorUnavailable,
    Core3DTextReliefErrorInvalidArgument,
    //! The selected native face is missing, shared, duplicate or non-planar.
    Core3DTextReliefErrorInvalidSelection,
    Core3DTextReliefErrorStale,
    Core3DTextReliefErrorCancelled,
    Core3DTextReliefErrorBudget,
    //! SYTO resource admission refused; the exact e5.* reason is in
    //! Core3DTextReliefRefusalReasonKey.
    Core3DTextReliefErrorResourceRefused,
    //! The native raised builder refused; the exact e5.* reason is in
    //! Core3DTextReliefRefusalReasonKey.
    Core3DTextReliefErrorBuildRefused,
    //! Emboss/Deboss are unavailable in S1 and never fall into a raised or
    //! anonymous Boolean path.
    Core3DTextReliefErrorModeUnavailable,
    Core3DTextReliefErrorApplyUnavailable,
};

typedef NS_ENUM(NSInteger, Core3DTextReliefResourceKind) {
    Core3DTextReliefResourceKindFontOutline = 1,
    Core3DTextReliefResourceKindEmblemVector = 2,
};

typedef NS_ENUM(NSInteger, Core3DTextReliefMode) {
    Core3DTextReliefModeEmboss = 1,
    Core3DTextReliefModeDeboss = 2,
    Core3DTextReliefModeRaised = 3,
};

typedef NS_ENUM(NSInteger, Core3DTextReliefAlignment) {
    Core3DTextReliefAlignmentLeft = 1,
    Core3DTextReliefAlignmentCenter = 2,
    Core3DTextReliefAlignmentRight = 3,
};

//! Reserved S2 apply outcome. S1 always reports Unavailable.
typedef NS_ENUM(NSInteger, Core3DTextReliefApplyResult) {
    Core3DTextReliefApplyResultUnavailable = 0,
    Core3DTextReliefApplyResultApplied = 1,
    Core3DTextReliefApplyResultRefused = 2,
    Core3DTextReliefApplyResultRecoveryRequired = 3,
};

//! Native-issued planar-face identity and placement frame for one selected
//! host face. The frame origin lies on the face plane, frameZ is the outward
//! normal (reversed faces handled natively) and frameX/frameY are a
//! deterministic in-plane basis. Coordinates are document-local units of the
//! issuing document. The receipt is document/source/topology bound and cannot
//! be constructed by callers.
__attribute__((objc_subclassing_restricted))
@interface Core3DTextReliefIssuedFace : NSObject
@property(nonatomic, copy, readonly) NSData *identifier;
@property(nonatomic, copy, readonly) NSData *selectorProof;
@property(nonatomic, copy, readonly) NSString *entityIdentifier;
@property(nonatomic, assign, readonly) uint32_t topologyIndex;
@property(nonatomic, assign, readonly) uint16_t boundaryEdgeCount;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *frameOriginLocal;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *frameXAxis;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *frameYAxis;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *frameZAxis;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Observation of one detached raised component, measured from the emitted
//! candidate topology. Values are physical millimetres; request values are
//! not exposed as observations.
__attribute__((objc_subclassing_restricted))
@interface Core3DTextReliefComponentObservation : NSObject
@property(nonatomic, assign, readonly) uint32_t componentIndex;
@property(nonatomic, copy, readonly) NSData *identifier;
@property(nonatomic, assign, readonly) double areaMM2;
@property(nonatomic, assign, readonly) double volumeMM3;
@property(nonatomic, assign, readonly) double minAlongNormalMM;
@property(nonatomic, assign, readonly) double maxAlongNormalMM;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! A detached native E5 raised candidate. It carries geometry and
//! observations only: no retained owner, no document authority and no commit
//! capability.
__attribute__((objc_subclassing_restricted))
@interface Core3DTextReliefCandidate : NSObject
@property(nonatomic, copy, readonly) NSData *candidateBRepBytes;
@property(nonatomic, copy, readonly) NSData *geometryDigest;
@property(nonatomic, copy, readonly) NSData *resourceSHA256;
@property(nonatomic, copy, readonly) NSArray<NSData *> *canonicalCurveBytes;
@property(nonatomic, copy, readonly) NSArray<Core3DTextReliefComponentObservation *> *componentObservations;
@property(nonatomic, assign, readonly) double metersPerUnit;
@property(nonatomic, assign, readonly) double totalAreaMM2;
@property(nonatomic, assign, readonly) double totalVolumeMM3;
@property(nonatomic, assign, readonly) double textAdvanceMM;
@property(nonatomic, assign, readonly) NSUInteger connectedSolidCount;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Viewer-issued E5 text/emblem relief opening bound to the captured
//! document/source/planar-face-selection fence. All S1 work is detached;
//! recapture mismatch suppresses delivery. The opening retains its native
//! document handle and nonserializable currentness fence; an away-and-back
//! edit or any selection change makes it stale.
__attribute__((objc_subclassing_restricted))
@interface Core3DTextReliefOpening : NSObject
@property(nonatomic, strong, readonly) Core3DTextReliefIssuedFace *issuedFace;
@property(nonatomic, copy, readonly) NSString *documentIdentifier;
@property(nonatomic, copy, readonly) NSString *contextIdentifier;
@property(nonatomic, assign, readonly) double metersPerUnit;
@property(nonatomic, assign, readonly) NSInteger historyDepth;
//! Populated after a successful create/replace apply. These are observations
//! of the natively issued durable owner and feature, never caller authority.
@property(nonatomic, copy, readonly, nullable) NSData *appliedOwnerIdentifier;
@property(nonatomic, copy, readonly, nullable) NSData *appliedFeatureIdentifier;

//! Captures an opening from the controller's current single-face selection.
//! The selected face must be one planar face of a retained native source;
//! non-planar faces refuse with Core3DTextReliefErrorInvalidSelection and the
//! reason e5.non-planar-host.
+ (nullable instancetype)captureController:(Core3DViewController *)controller
                                     error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(capture(controller:));

//! Captures an existing retained relief while the controller has its original
//! host face selected. The feature identifier must be the 16-byte identifier
//! previously observed from a successful apply or persisted record.
+ (nullable instancetype)captureController:(Core3DViewController *)controller
                 existingFeatureIdentifier:(NSData *)featureIdentifier
                                      error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(capture(controller:existingFeatureIdentifier:));

//! Detached raised build against the issued face. The resource bytes are
//! trusted catalog bytes pinned by expectedSHA256 (64 lowercase hex); text is
//! the UTF-8 glyph string for fonts and must be empty for emblems; heightMM,
//! depthMM and trackingMM are physical millimetres; offsetUMM/offsetVMM shift
//! the placement frame along its in-plane axes; rotationRadians rotates the
//! frame in plane; alignment shifts the text block by 0, -advance/2 or
//! -advance along the advance direction. Only Raised mode builds; Emboss and
//! Deboss refuse with Core3DTextReliefErrorModeUnavailable. The candidate
//! shares no topology with the document and mutates nothing.
- (nullable Core3DTextReliefCandidate *)detachedRaisedCandidateWithResourceBytes:(NSData *)resourceBytes
                                                                 expectedSHA256:(NSString *)expectedSHA256
                                                                           kind:(Core3DTextReliefResourceKind)kind
                                                                           text:(NSString *)text
                                                                       heightMM:(double)heightMM
                                                                        depthMM:(double)depthMM
                                                                     trackingMM:(double)trackingMM
                                                                      alignment:(Core3DTextReliefAlignment)alignment
                                                                      offsetUMM:(double)offsetUMM
                                                                      offsetVMM:(double)offsetVMM
                                                                rotationRadians:(double)rotationRadians
                                                                           mode:(Core3DTextReliefMode)mode
                                                                          error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(detachedRaisedCandidate(resourceBytes:expectedSHA256:kind:text:heightMM:depthMM:trackingMM:alignment:offsetUMM:offsetVMM:rotationRadians:mode:));

//! Re-observes controller, document, context, selection, source and history.
- (BOOL)isCurrent:(NSError * _Nullable * _Nullable)error;
//! Stops outstanding and future work of this opening. A cancelled opening
//! issues no candidate and never mutates its source.
- (void)cancel;

//! Reserved S2 prepare entrance: binds a detached candidate to a future
//! document transaction. S1 always fails with
//! Core3DTextReliefErrorApplyUnavailable and changes no state.
- (nullable Core3DTextReliefOpening *)openingByPreparingCandidate:(Core3DTextReliefCandidate *)candidate
                                                            error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(preparing(candidate:));
//! S2 resource-bound prepare entrance. catalogHandle is the durable bundled
//! catalog identity whose exact bytes produced candidate.
- (nullable Core3DTextReliefOpening *)openingByPreparingCandidate:(Core3DTextReliefCandidate *)candidate
                                                    catalogHandle:(NSString *)catalogHandle
                                                            error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(preparing(candidate:catalogHandle:));
//! Prepares removal of the existing retained relief captured by the receiver.
- (nullable Core3DTextReliefOpening *)openingByPreparingRemovalWithError:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(preparingRemoval());
//! Applies exactly one prepared create, replace or remove through one native
//! command lease. The returned value distinguishes refusal from recovery.
- (Core3DTextReliefApplyResult)apply;
//! Settles one retained unknown close/publication after exact native readback.
//! The issuing opening is terminal after a successful reconciliation.
- (BOOL)reconcileRecovery;
+ (BOOL)isPubliclyAvailable;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

NS_ASSUME_NONNULL_END
