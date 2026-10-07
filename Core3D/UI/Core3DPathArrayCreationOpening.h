//
//  Core3DPathArrayCreationOpening.h
//  Core3D
//
//  Production record-free D3 path-array candidate and first Apply.
//

#import <Core3D/Core3DModelingTypes.h>
#import <Core3D/Core3DViewController.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DPathArrayCreationOpening;

//! Read-only observation of one native placement. These values are evidence,
//! not authority and cannot be imported into another preparation.
__attribute__((objc_subclassing_restricted))
@interface Core3DPathArrayPlacementPreview : NSObject
@property(nonatomic,copy,readonly) NSString *entityIdentifier;
@property(nonatomic,readonly) NSUInteger localIdentifier;
@property(nonatomic,readonly) NSUInteger ordinal;
@property(nonatomic,readonly) double requestedArcLength;
@property(nonatomic,readonly) double measuredArcLength;
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *occurrenceFrameValues;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Typed evidence from exactly one frozen native capture and preparation.
//! Nullable measurements are explicitly unavailable at the observed refusal
//! phase; a measured zero is represented by a nonnull NSNumber containing 0.
__attribute__((objc_subclassing_restricted))
@interface Core3DPathArrayPreview : NSObject
@property(nonatomic,readonly,getter=isAdmitted) BOOL admitted;
@property(nonatomic,copy,readonly) NSString *phase;
@property(nonatomic,copy,readonly,nullable) NSString *refusalDomain;
@property(nonatomic,copy,readonly,nullable) NSString *refusalCode;
@property(nonatomic,strong,readonly,nullable) NSNumber *requiredInstanceCount;
@property(nonatomic,strong,readonly,nullable) NSNumber *totalLength;
@property(nonatomic,strong,readonly,nullable) NSNumber *requestedPlacementCount;
@property(nonatomic,strong,readonly,nullable) NSNumber *emittedPlacementCount;
@property(nonatomic,strong,readonly,nullable) NSNumber *maximumMeasuredArcError;
@property(nonatomic,strong,readonly,nullable) NSNumber *closedSeamCanonicalized;
@property(nonatomic,strong,readonly,nullable) NSNumber *projectedInstances;
@property(nonatomic,strong,readonly,nullable) NSNumber *projectedTopologyNodes;
@property(nonatomic,strong,readonly,nullable) NSNumber *projectedDocumentBytes;
@property(nonatomic,strong,readonly,nullable) NSNumber *projectedMemoryBytes;
@property(nonatomic,strong,readonly,nullable) NSNumber *sourceTopologyNodes;
@property(nonatomic,strong,readonly,nullable) NSNumber *sourceDocumentBytes;
@property(nonatomic,strong,readonly,nullable) NSNumber *sourceMemoryBytes;
@property(nonatomic,strong,readonly,nullable) NSNumber *issuedMemberIdentityCount;
@property(nonatomic,readonly) double documentMetersPerUnit;
@property(nonatomic,copy,readonly) NSString *sourceEntityIdentifier;
@property(nonatomic,copy,readonly) NSString *pathEntityIdentifier;
@property(nonatomic,copy,readonly,nullable) NSString *ownerEntityIdentifier;
@property(nonatomic,copy,readonly,nullable) NSString *featureIdentifier;
@property(nonatomic,copy,readonly) NSArray<Core3DPathArrayPlacementPreview *> *placements;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Opaque issuer-bound, single-use token. It exposes no receipt, initializer,
//! candidate setter, identity reservation, or independently redeemable value.
__attribute__((objc_subclassing_restricted))
@interface Core3DPathArrayPreparedCandidate : NSObject
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DPathArrayPreparation : NSObject
@property(nonatomic,strong,readonly) Core3DPathArrayPreview *preview;
@property(nonatomic,strong,readonly,nullable) Core3DPathArrayPreparedCandidate *prepared;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! One source-selection-bound D3 creation issuer. Preparation reserves native
//! member identities but writes no OCAF label, shape, recipe, record, history,
//! publication, or command. Apply re-captures and equality-reprepares with the
//! original reservations before staging the whole result in one command.
__attribute__((objc_subclassing_restricted))
@interface Core3DPathArrayCreationOpening : NSObject
- (Core3DPathArrayPreparation *)prepareCandidate:
    (NSDictionary<NSString *, id> *)candidate
    NS_SWIFT_NAME(prepare(candidate:));
- (void)applyPrepared:(Core3DPathArrayPreparedCandidate *)prepared
    completion:(void (^)(Core3DProfileConstructionResult result,
                          NSString *detail,
                          NSString * _Nullable sourceEntityIdentifier))completion
    NS_SWIFT_NAME(apply(prepared:completion:));
- (Core3DPathArrayCreationOpening * _Nullable)
    replacingPathWithEntityIdentifier:(NSString *)entityIdentifier
    NS_SWIFT_NAME(replacingPath(entityIdentifier:));
- (BOOL)cancel;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DViewController (PathArrayCreationOpening)
- (Core3DPathArrayCreationOpening * _Nullable)
    beginPathArrayCreationWithPathEntityIdentifier:(NSString *)entityIdentifier
    NS_SWIFT_NAME(beginPathArrayCreation(pathEntityIdentifier:));
#if DEBUG
//! Read-only production evidence selected by an actual returned identity.
//! Never creates, stages, repairs, or chooses the last record implicitly.
- (NSDictionary<NSString *, id> * _Nullable)
    debugPathArrayCreationEvidenceForEntityIdentifier:(NSString *)entityIdentifier
    NS_SWIFT_NAME(debugPathArrayCreationEvidence(entityIdentifier:));
#endif
@end

NS_ASSUME_NONNULL_END
