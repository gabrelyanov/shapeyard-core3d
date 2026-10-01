#import <Foundation/Foundation.h>
#import <Core3D/Core3DModelingTypes.h>

NS_ASSUME_NONNULL_BEGIN
typedef NS_ENUM(NSInteger,Core3DEdgeTreatmentKind){Core3DEdgeTreatmentKindChamfer=1,Core3DEdgeTreatmentKindConstantFillet=2};
typedef NS_ENUM(NSInteger,Core3DEdgeTreatmentCurveKind){Core3DEdgeTreatmentCurveKindLine=1,Core3DEdgeTreatmentCurveKindCircle=2};
typedef NS_ENUM(NSInteger,Core3DEdgeTreatmentStatus){Core3DEdgeTreatmentStatusCurrentEditable=1,Core3DEdgeTreatmentStatusAbsentLegacy=2,Core3DEdgeTreatmentStatusReadableNoncurrent=3,Core3DEdgeTreatmentStatusUnsupportedVersion=4,Core3DEdgeTreatmentStatusMalformed=5,Core3DEdgeTreatmentStatusAmbiguous=6};
typedef NS_ENUM(NSInteger,Core3DEdgeTreatmentOutcome){Core3DEdgeTreatmentOutcomeCommitted,Core3DEdgeTreatmentOutcomeUnchanged,Core3DEdgeTreatmentOutcomeRefused,Core3DEdgeTreatmentOutcomeCancelled,Core3DEdgeTreatmentOutcomeBusy,Core3DEdgeTreatmentOutcomeOutcomeUnknown};
typedef NS_ENUM(NSInteger,Core3DEdgeTreatmentSourceKind){Core3DEdgeTreatmentSourceKindProfile=1,Core3DEdgeTreatmentSourceKindEnclosure=2};

@interface Core3DEdgeTreatmentVector3:NSObject
@property(nonatomic,readonly)double x,y,z;
- (instancetype)init NS_UNAVAILABLE; +(instancetype)new NS_UNAVAILABLE;
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
@interface Core3DEdgeTreatmentSnapshot:NSObject
@property(nonatomic,copy,readonly)NSString *documentIdentifier,*entityIdentifier,*definitionIdentifier,*sourceFeatureIdentifier,*baseNodeIdentifier,*outputNodeIdentifier;
@property(nonatomic,readonly)Core3DEdgeTreatmentSourceKind sourceKind;
@property(nonatomic,strong,readonly,nullable)Core3DEdgeTreatmentProfileSource *profileSource;
@property(nonatomic,strong,readonly,nullable)Core3DEdgeTreatmentEnclosureSource *enclosureSource;
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
- (instancetype)init NS_UNAVAILABLE; +(instancetype)new NS_UNAVAILABLE;
@end
@interface Core3DEdgeTreatmentOperation:NSObject
- (instancetype)init NS_UNAVAILABLE; +(instancetype)new NS_UNAVAILABLE;
@end
NS_ASSUME_NONNULL_END
