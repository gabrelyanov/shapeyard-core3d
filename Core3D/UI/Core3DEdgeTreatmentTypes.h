#import <Foundation/Foundation.h>
#import <Core3D/Core3DModelingTypes.h>

NS_ASSUME_NONNULL_BEGIN
typedef NS_ENUM(NSInteger,Core3DEdgeTreatmentKind){Core3DEdgeTreatmentKindChamfer=1,Core3DEdgeTreatmentKindConstantFillet=2};
typedef NS_ENUM(NSInteger,Core3DEdgeTreatmentCurveKind){Core3DEdgeTreatmentCurveKindLine=1,Core3DEdgeTreatmentCurveKindCircle=2};
typedef NS_ENUM(NSInteger,Core3DEdgeTreatmentStatus){Core3DEdgeTreatmentStatusCurrentEditable=1,Core3DEdgeTreatmentStatusAbsentLegacy=2,Core3DEdgeTreatmentStatusReadableNoncurrent=3,Core3DEdgeTreatmentStatusUnsupportedVersion=4,Core3DEdgeTreatmentStatusMalformed=5,Core3DEdgeTreatmentStatusAmbiguous=6};
typedef NS_ENUM(NSInteger,Core3DEdgeTreatmentOutcome){Core3DEdgeTreatmentOutcomeCommitted,Core3DEdgeTreatmentOutcomeUnchanged,Core3DEdgeTreatmentOutcomeRefused,Core3DEdgeTreatmentOutcomeCancelled,Core3DEdgeTreatmentOutcomeBusy,Core3DEdgeTreatmentOutcomeOutcomeUnknown};
typedef NS_ENUM(NSInteger,Core3DEdgeTreatmentSourceKind){Core3DEdgeTreatmentSourceKindProfile=1,Core3DEdgeTreatmentSourceKindEnclosure=2,Core3DEdgeTreatmentSourceKindRectangularLoft=4};

@interface Core3DEdgeTreatmentVector3:NSObject
@property(nonatomic,readonly)double x,y,z;
- (nullable instancetype)initWithX:(double)x y:(double)y z:(double)z NS_DESIGNATED_INITIALIZER; - (instancetype)init NS_UNAVAILABLE; +(instancetype)new NS_UNAVAILABLE;
@end
@interface Core3DEdgeTreatmentAnchor:NSObject
@property(nonatomic,copy,readonly)NSData *key;
@property(nonatomic,readonly)Core3DEdgeTreatmentCurveKind curve;
@property(nonatomic,strong,readonly)Core3DEdgeTreatmentVector3 *pointMM,*tangent,*normalA,*normalB;
@property(nonatomic,readonly)double circleRadiusMM;
- (instancetype)init NS_UNAVAILABLE; +(instancetype)new NS_UNAVAILABLE;
@end
@interface Core3DEdgeTreatmentStep:NSObject
@property(nonatomic,copy,readonly)NSString *nodeIdentifier,*featureIdentifier;
@property(nonatomic,readonly)uint64_t localID;
@property(nonatomic,readonly)Core3DEdgeTreatmentKind kind;
@property(nonatomic,readonly)double amountMM;
@property(nonatomic,copy,readonly)NSArray<Core3DEdgeTreatmentAnchor*> *anchors;
- (instancetype)init NS_UNAVAILABLE; +(instancetype)new NS_UNAVAILABLE;
@end
@interface Core3DEdgeTreatmentProfileSource:NSObject
@property(nonatomic,strong,readonly)Core3DProfileDefinition *definition;
@property(nonatomic,copy,readonly)NSArray *shells;
@property(nonatomic,copy,readonly)NSData *canonicalSourceBytes;
- (instancetype)init NS_UNAVAILABLE; +(instancetype)new NS_UNAVAILABLE;
@end
@interface Core3DEdgeTreatmentEnclosureSource:NSObject
@property(nonatomic,strong,readonly)Core3DEnclosureDefinition *definition;
@property(nonatomic,copy,readonly)NSData *canonicalSourceBytes;
- (instancetype)init NS_UNAVAILABLE; +(instancetype)new NS_UNAVAILABLE;
@end
@interface Core3DEdgeTreatmentLoftSource:NSObject
@property(nonatomic,strong,readonly)Core3DRectangularLoftDefinition *definition;
@property(nonatomic,copy,readonly)NSData *canonicalSourceBytes;
- (instancetype)init NS_UNAVAILABLE; +(instancetype)new NS_UNAVAILABLE;
@end
@interface Core3DEdgeTreatmentSnapshot:NSObject
@property(nonatomic,copy,readonly)NSString *documentIdentifier,*entityIdentifier,*definitionIdentifier,*sourceFeatureIdentifier,*baseNodeIdentifier,*outputNodeIdentifier;
@property(nonatomic,readonly)Core3DEdgeTreatmentSourceKind sourceKind;
@property(nonatomic,strong,readonly,nullable)Core3DEdgeTreatmentProfileSource *profileSource;
@property(nonatomic,strong,readonly,nullable)Core3DEdgeTreatmentEnclosureSource *enclosureSource;
@property(nonatomic,strong,readonly,nullable)Core3DEdgeTreatmentLoftSource *loftSource;
@property(nonatomic,copy,readonly)NSArray<Core3DEdgeTreatmentStep*> *steps;
@property(nonatomic,readonly)NSUInteger prefixOperandCount;
@property(nonatomic,copy,readonly)NSData *canonicalRecipeBytes;
@property(nonatomic,readonly,getter=isCurrent)BOOL current;
@property(nonatomic,readonly)double dimensionMetersPerUnit;
- (instancetype)init NS_UNAVAILABLE; +(instancetype)new NS_UNAVAILABLE;
@end
@interface Core3DEdgeTreatmentCapture:NSObject
@property(nonatomic,readonly)Core3DEdgeTreatmentStatus status;
@property(nonatomic,strong,readonly,nullable)Core3DEdgeTreatmentSnapshot *snapshot;
//! Echo of the requested owner identity and the native carrier, populated by
//! the native capture path even when the snapshot itself cannot be produced.
//! Lets a recovery receipt prove it refers to the same native operation and
//! carrier; never caller-writable and never synthesized from provider data.
@property(nonatomic,copy,readonly,nullable)NSString *documentIdentifier,*entityIdentifier,*carrier;
@property(nonatomic,copy,readonly)NSString *refusalCode,*refusalMessage;
- (instancetype)init NS_UNAVAILABLE; +(instancetype)new NS_UNAVAILABLE;
@end
@interface Core3DEdgeTreatmentTargetCapture:NSObject
@property(nonatomic,readonly)Core3DEdgeTreatmentStatus status;
@property(nonatomic,strong,readonly,nullable)Core3DEdgeTreatmentSnapshot *snapshot;
@property(nonatomic,copy,readonly)NSArray<Core3DEdgeTreatmentAnchor*> *anchors;
@property(nonatomic,copy,readonly)NSString *refusalCode,*refusalMessage;
- (instancetype)init NS_UNAVAILABLE; +(instancetype)new NS_UNAVAILABLE;
@end
@interface Core3DEdgeTreatmentResult:NSObject
@property(nonatomic,readonly)Core3DEdgeTreatmentOutcome outcome;
@property(nonatomic,copy,readonly)NSString *refusalCode,*refusalMessage;
@property(nonatomic,strong,readonly,nullable)NSNumber *measuredUndoDelta;
@property(nonatomic,copy,readonly)NSArray<NSString*> *replayedFeatureIdentifiers;
//! Native provenance binding this result to its operation, owner, actual unit,
//! source family/carrier/prefix and the original and candidate source/SYET
//! byte digests. Populated only by Core3D's own completion path from the
//! retained snapshot and detached result; nil means unavailable and is never
//! substituted with a zero-filled or fabricated value. The class has no public
//! initializer, so none of these fields can be minted by a caller.
@property(nonatomic,copy,readonly,nullable)NSString *documentIdentifier,*entityIdentifier,*sourceFeatureIdentifier;
@property(nonatomic,readonly)double dimensionMetersPerUnit;
@property(nonatomic,copy,readonly,nullable)NSString *sourceFamily,*carrier,*prefixState;
@property(nonatomic,copy,readonly,nullable)NSString *originalSourceSHA256,*candidateSourceSHA256,*originalRecipeSHA256,*candidateRecipeSHA256;
- (instancetype)init NS_UNAVAILABLE; +(instancetype)new NS_UNAVAILABLE;
@end
@interface Core3DEdgeTreatmentOperation:NSObject
- (instancetype)init NS_UNAVAILABLE; +(instancetype)new NS_UNAVAILABLE;
@end
NS_ASSUME_NONNULL_END
