#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DViewController;

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
