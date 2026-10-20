#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DViewController;

//! B3 variable-radius fillet error domain. S1 issues a detached native
//! candidate only; no retained owner, document transaction, CommandLease,
//! publication or history entry exists at this stage.
FOUNDATION_EXPORT NSErrorDomain const Core3DVariableFilletErrorDomain;

//! NSError userInfo key carrying the exact native refusal string
//! (for example @"b3.local-clearance") whenever the refusal originates from
//! the native law/anchor/build admission boundary.
FOUNDATION_EXPORT NSString *const Core3DVariableFilletRefusalReasonKey;

//! NSError userInfo key carrying the native refusal ordinal (NSNumber int).
FOUNDATION_EXPORT NSString *const Core3DVariableFilletRefusalRawValueKey;

typedef NS_ERROR_ENUM(Core3DVariableFilletErrorDomain, Core3DVariableFilletError) {
    Core3DVariableFilletErrorInvalidThread = 1,
    Core3DVariableFilletErrorUnavailable,
    Core3DVariableFilletErrorInvalidArgument,
    Core3DVariableFilletErrorInvalidLaw,
    Core3DVariableFilletErrorInvalidSelection,
    //! The selected native edge is closed, periodic, seamed or degenerate.
    //! No closed loop is admitted; the reason is
    //! b3.closed-loop-seam-rule-required or b3.unsupported-edge.
    Core3DVariableFilletErrorClosedLoopUnsupported,
    Core3DVariableFilletErrorStale,
    Core3DVariableFilletErrorCancelled,
    Core3DVariableFilletErrorBudget,
    //! The native deterministic builder refused; the exact b3.* reason is in
    //! Core3DVariableFilletRefusalReasonKey.
    Core3DVariableFilletErrorBuildRefused,
    Core3DVariableFilletErrorApplyUnavailable,
};

typedef NS_ENUM(NSInteger, Core3DVariableFilletEdgeOrientation) {
    Core3DVariableFilletEdgeOrientationForward = 1,
    Core3DVariableFilletEdgeOrientationReversed = 2,
};

//! Retained-owner apply outcome. OutcomeUnknown/RecoveryRequired retain the
//! native authority and candidate evidence; callers must reconcile and must
//! never retry the edit as a fresh command.
typedef NS_ENUM(NSInteger, Core3DVariableFilletApplyResult) {
    Core3DVariableFilletApplyResultUnavailable = 0,
    Core3DVariableFilletApplyResultRefused,
    Core3DVariableFilletApplyResultCancelled,
    Core3DVariableFilletApplyResultCommitted,
    Core3DVariableFilletApplyResultOutcomeUnknown,
    Core3DVariableFilletApplyResultRecoveryRequired,
};

//! One caller-authored interior-or-endpoint law station. The identifier is
//! exactly 16 nonzero bytes, t is the authored orientedNormalizedArcLength
//! parameter and radiusMM is the physical millimetre radius. Construction is
//! bounded; invalid values return nil and can never become a native station.
__attribute__((objc_subclassing_restricted))
@interface Core3DVariableFilletStation : NSObject
@property(nonatomic, copy, readonly) NSData *identifier;
@property(nonatomic, assign, readonly) double parameter;
@property(nonatomic, assign, readonly) double radiusMM;
- (nullable instancetype)initWithIdentifier:(NSData *)identifier
                                  parameter:(double)parameter
                                   radiusMM:(double)radiusMM
    NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Native-issued oriented-edge identity. Every value is captured from the
//! actual selected native edge; callers can observe but never construct it.
//! Coordinates are document-local units of the issuing document.
__attribute__((objc_subclassing_restricted))
@interface Core3DVariableFilletIssuedEdge : NSObject
@property(nonatomic, copy, readonly) NSData *identifier;
@property(nonatomic, copy, readonly) NSString *entityIdentifier;
@property(nonatomic, assign, readonly) uint32_t topologyIndex;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *pointLocal;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *tangent;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *normalA;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *normalB;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *startLocal;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *endLocal;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Realised-law pin read back on the physical edge after the native build.
//! Values are physical millimetres along the issued authored parameter.
__attribute__((objc_subclassing_restricted))
@interface Core3DVariableFilletLawPin : NSObject
@property(nonatomic, assign, readonly) double parameter;
@property(nonatomic, assign, readonly) double positionMM;
@property(nonatomic, assign, readonly) double radiusMM;
@property(nonatomic, assign, readonly) double derivativeMM;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Realised law observation of one detached build. Populated when the native
//! DEBUG law observer is compiled in; otherwise the pin list is empty and the
//! structural flags are NO. Never an oracle for section geometry.
__attribute__((objc_subclassing_restricted))
@interface Core3DVariableFilletLawObservation : NSObject
@property(nonatomic, assign, readonly) BOOL attempted;
@property(nonatomic, assign, readonly) BOOL supported;
@property(nonatomic, assign, readonly) BOOL c1OnPhysicalInterval;
@property(nonatomic, assign, readonly) BOOL nonnegativeDerivative;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *stationParameters;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *stationRadiiMM;
@property(nonatomic, copy, readonly) NSArray<Core3DVariableFilletLawPin *> *pins;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! One independent section observation of the emitted detached candidate
//! BRep. The section branch is selected from topology and support identity
//! only (unique nonplanar face adjacent to both identified supports), never
//! by proximity to any expected radius. Coordinates are physical millimetres
//! in the section plane frame: a is the distance from the first support
//! plane, b the distance from the second support plane, z the residual
//! against the pinned section plane.
__attribute__((objc_subclassing_restricted))
@interface Core3DVariableFilletSectionObservation : NSObject
@property(nonatomic, assign, readonly) double parameter;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *planeOriginMM;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *planeNormal;
//! Five ray intersections at phi = 0, pi/8, pi/4, 3pi/8, pi/2 from the
//! support-B contact toward the support-A contact; each entry is {a, b}.
@property(nonatomic, copy, readonly) NSArray<NSArray<NSNumber *> *> *rayPointsMM;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *rayRadiiMM;
@property(nonatomic, assign, readonly) double contactARadiusMM;
@property(nonatomic, assign, readonly) double contactBRadiusMM;
@property(nonatomic, assign, readonly) double midpointRadiusMM;
@property(nonatomic, assign, readonly) double symmetryDeviationMM;
@property(nonatomic, assign, readonly) double planeResidualMM;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! A detached native B3 candidate. It carries geometry and observations only:
//! no retained owner, no document authority and no commit capability.
__attribute__((objc_subclassing_restricted))
@interface Core3DVariableFilletCandidate : NSObject
@property(nonatomic, copy, readonly) NSData *candidateBRepBytes;
@property(nonatomic, copy, readonly) NSData *geometryDigest;
@property(nonatomic, copy, readonly) NSData *familyProof;
@property(nonatomic, copy, readonly) NSData *canonicalDefinitionBytes;
@property(nonatomic, copy, readonly) NSArray<NSData *> *consumedEdgeIdentifiers;
@property(nonatomic, assign, readonly) double metersPerUnit;
@property(nonatomic, assign, readonly) double minimumClearanceMM;
@property(nonatomic, assign, readonly) double removedVolumeMM3;
@property(nonatomic, copy, readonly) NSArray<Core3DVariableFilletLawObservation *> *lawObservations;
@property(nonatomic, copy, readonly) NSArray<Core3DVariableFilletSectionObservation *> *sectionObservations;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Opaque controller/viewer-issued B3 opening bound to one document, one
//! native source entity and one issued oriented-edge selection. S1 scope:
//! capture plus detached candidate builds. The opening retains its native
//! document handle and nonserializable currentness fence; an away-and-back
//! edit or any selection change makes it stale.
__attribute__((objc_subclassing_restricted))
@interface Core3DVariableFilletOpening : NSObject

@property(nonatomic, strong, readonly) Core3DVariableFilletIssuedEdge *issuedEdge;
@property(nonatomic, copy, readonly) NSString *documentIdentifier;
@property(nonatomic, copy, readonly) NSString *contextIdentifier;
@property(nonatomic, assign, readonly) double metersPerUnit;
@property(nonatomic, assign, readonly) NSInteger historyDepth;
//! Number of real D2/D3/D4 descendants in the prepared document-owned plan.
@property(nonatomic, assign, readonly) NSInteger dependentReplayCount;
//! Measured native history delta of the last apply (one only on commit).
@property(nonatomic, assign, readonly) NSInteger measuredHistoryDelta;

//! Captures an opening from the controller's current single-edge selection.
//! The selected edge must be a real open straight native edge with exactly
//! two planar adjacent faces; closed, periodic, seamed or degenerate edges
//! refuse here with Core3DVariableFilletErrorClosedLoopUnsupported.
+ (nullable instancetype)captureController:(Core3DViewController *)controller
                                     error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(capture(controller:));

//! Detached linear two-station build between the physical millimetre endpoint
//! radii. r0 applies at the issued authored t=0 endpoint, r1 at t=1. The
//! candidate shares no topology with the document and mutates nothing.
- (nullable Core3DVariableFilletCandidate *)detachedCandidateWithStartRadiusMM:(double)startRadiusMM
                                                                   endRadiusMM:(double)endRadiusMM
                                                                   orientation:(Core3DVariableFilletEdgeOrientation)orientation
                                                                         error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(detachedCandidate(startRadiusMM:endRadiusMM:orientation:));

//! Detached multi-station build. Stations must be 3...16 with unique nonzero
//! identifiers, exact t=0/t=1 endpoints and strictly increasing finite
//! interior parameters; every radius finite and inside the native range.
- (nullable Core3DVariableFilletCandidate *)detachedCandidateWithStations:(NSArray<Core3DVariableFilletStation *> *)stations
                                                              orientation:(Core3DVariableFilletEdgeOrientation)orientation
                                                                    error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(detachedCandidate(stations:orientation:));

//! Re-observes controller, document, context, selection, source and history.
- (BOOL)isCurrent:(NSError * _Nullable * _Nullable)error;
//! Stops outstanding and future work of this opening. A cancelled opening
//! issues no candidate and never mutates its source.
- (void)cancel;

//! Reads the canonical retained law from the native owner. The station arrays
//! include every endpoint/interior parameter and physical-mm radius.
- (nullable Core3DVariableFilletLawObservation *)readRetainedLawWithError:(NSError * _Nullable * _Nullable)error;
//! Binds a detached candidate to the exact source and prepares the complete
//! dependent closure before any document command exists.
- (nullable Core3DVariableFilletOpening *)openingByPreparingCandidate:(Core3DVariableFilletCandidate *)candidate
                                                                error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(preparing(candidate:));
//! Stages source, law and prepared descendants under one native lease, reads
//! the whole candidate, closes once and publishes through the same context.
- (Core3DVariableFilletApplyResult)apply;
//! Explicit recovery for an uncertain close/publication. This observes the
//! actual native state as exact prior or exact candidate; it never retries.
- (Core3DVariableFilletApplyResult)reconcile;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end

NS_ASSUME_NONNULL_END
