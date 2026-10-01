#import <Foundation/Foundation.h>
#import <Core3D/Core3DEdgeTreatmentTypes.h>
#import <Core3D/Core3DFaceSelectorTypes.h>

NS_ASSUME_NONNULL_BEGIN
typedef NS_ENUM(NSInteger, Core3DEdgeTreatmentSourceKindR2) {
    Core3DEdgeTreatmentSourceKindR2Profile = 1,
    Core3DEdgeTreatmentSourceKindR2Enclosure = 2,
    Core3DEdgeTreatmentSourceKindR2RetainedBoolean = 3,
};
typedef NS_ENUM(NSInteger, Core3DRetainedBooleanPrefixFormatR2) {
    Core3DRetainedBooleanPrefixFormatR2SYRS = 1,
    Core3DRetainedBooleanPrefixFormatR2A1Composite = 2,
};
typedef NS_ENUM(NSInteger, Core3DRetainedBooleanInputRoleR2) {
    Core3DRetainedBooleanInputRoleR2Left = 1,
    Core3DRetainedBooleanInputRoleR2Right = 2,
    Core3DRetainedBooleanInputRoleR2AnalyticTool = 3,
};
typedef NS_ENUM(NSInteger, Core3DRetainedBooleanEditKindR2) {
    Core3DRetainedBooleanEditKindR2SetAmount = 1,
    Core3DRetainedBooleanEditKindR2Remove = 2,
    Core3DRetainedBooleanEditKindR2RebuildBooleanInput = 3,
    Core3DRetainedBooleanEditKindR2RebuildAnalyticTool = 4,
    Core3DRetainedBooleanEditKindR2SetBooleanOperation = 5,
    Core3DRetainedBooleanEditKindR2SetInputPlacement = 6,
};

@interface Core3DRetainedBooleanRecipeLocatorR2 : NSObject
@property(nonatomic,copy,readonly) NSString *documentIdentifier, *entityIdentifier,
    *definitionIdentifier, *nodeIdentifier, *sourceFeatureIdentifier;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DRetainedBooleanAnalyticToolR2 : NSObject
@property(nonatomic,readonly) uint32_t operandID;
@property(nonatomic,readonly) NSInteger kind, extent, axis;
@property(nonatomic,strong,readonly) Core3DEdgeTreatmentVector3 *point;
@property(nonatomic,readonly) double radius, boltCircleRadius, hostRadiusRatio,
    directionAngle, halfWidthApex, halfWidthMouth, length;
@property(nonatomic,readonly) NSUInteger count;
@property(nonatomic,strong,readonly) Core3DRetainedBooleanRecipeLocatorR2 *locator;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DRetainedBooleanInputPlacementR2 : NSObject
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *rowMajorMatrix;
@property(nonatomic,readonly) double sourceMetersPerUnit, carrierMetersPerUnit;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DRetainedBooleanInputR2 : NSObject
@property(nonatomic,readonly) Core3DRetainedBooleanInputRoleR2 role;
@property(nonatomic,strong,readonly) Core3DRetainedBooleanRecipeLocatorR2 *locator;
@property(nonatomic,strong,readonly,nullable) Core3DProfileDefinition *profile;
@property(nonatomic,strong,readonly,nullable) Core3DEnclosureDefinition *enclosure;
@property(nonatomic,strong,readonly,nullable) Core3DRectangularLoftDefinition *rectangularLoft;
@property(nonatomic,strong,readonly,nullable) Core3DRetainedBooleanAnalyticToolR2 *analyticTool;
@property(nonatomic,strong,readonly,nullable) Core3DRetainedBooleanInputPlacementR2 *inputPlacement;
@property(nonatomic,copy,readonly) NSData *canonicalRecipeBytes;
@property(nonatomic,copy,readonly) NSString *geometryDigest, *recipeDigest,
    *placementDigest, *materialDigest, *groupsDigest;
@property(nonatomic,readonly) uint32_t recipeSchema, sourceShapeSlot;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DRetainedBooleanOperationR2 : NSObject
@property(nonatomic,copy,readonly) NSString *nodeIdentifier, *featureIdentifier,
    *leftNodeIdentifier, *rightNodeIdentifier;
@property(nonatomic,readonly) NSInteger operation;
@property(nonatomic,readonly) uint32_t codecVersion;
@property(nonatomic,strong,readonly,nullable) NSNumber *operandID;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DRetainedBooleanMigrationStepMapR2 : NSObject
@property(nonatomic,readonly) uint64_t oldStepID;
@property(nonatomic,copy,readonly) NSString *nodeIdentifier, *featureIdentifier;
@property(nonatomic,readonly,getter=isRetired) BOOL retired;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end
@interface Core3DRetainedBooleanMigrationAnchorMapR2 : NSObject
@property(nonatomic,readonly) uint64_t oldEdgeID;
@property(nonatomic,copy,readonly) NSData *key;
@property(nonatomic,readonly,getter=isRetired) BOOL retired;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DRetainedBooleanSourceR2 : NSObject
@property(nonatomic,readonly) uint32_t sourceContractRevision, prefixBindingVersion,
    sourceSchema, sourceWireMajor, sourceWireMinor;
@property(nonatomic,readonly) Core3DRetainedBooleanPrefixFormatR2 prefixFormat;
@property(nonatomic,copy,readonly) NSData *canonicalPrefixBytes;
@property(nonatomic,copy,readonly) NSArray<Core3DRetainedBooleanInputR2 *> *inputs;
@property(nonatomic,copy,readonly) NSArray<Core3DRetainedBooleanOperationR2 *> *operations;
@property(nonatomic,copy,readonly) NSString *sourceDocumentIdentifier, *sourceEntityIdentifier,
    *sourceDefinitionIdentifier, *sourceFeatureIdentifier, *prefixOutputNodeIdentifier,
    *sourceRecipeDigest;
@property(nonatomic,readonly) double metersPerLocalUnit;
@property(nonatomic,readonly) uint64_t nextOperandID, nextFilletStepID, nextFilletEdgeID;
@property(nonatomic,copy,readonly) NSArray<Core3DRetainedBooleanMigrationStepMapR2 *> *stepMappings;
@property(nonatomic,copy,readonly) NSArray<Core3DRetainedBooleanMigrationAnchorMapR2 *> *anchorMappings;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DEdgeTreatmentSnapshotR2 : NSObject
@property(nonatomic,readonly) uint32_t contractRevision;
@property(nonatomic,copy,readonly) NSString *documentIdentifier, *entityIdentifier,
    *definitionIdentifier, *sourceFeatureIdentifier, *baseNodeIdentifier, *outputNodeIdentifier;
@property(nonatomic,readonly) Core3DEdgeTreatmentSourceKindR2 sourceKind;
@property(nonatomic,strong,readonly,nullable) Core3DEdgeTreatmentProfileSource *profileSource;
@property(nonatomic,strong,readonly,nullable) Core3DEdgeTreatmentEnclosureSource *enclosureSource;
@property(nonatomic,strong,readonly,nullable) Core3DRetainedBooleanSourceR2 *retainedBooleanSource;
@property(nonatomic,copy,readonly) NSArray<Core3DEdgeTreatmentStep *> *steps;
@property(nonatomic,copy,readonly) NSData *canonicalRecipeBytes;
@property(nonatomic,readonly) NSUInteger prefixOperandCount;
@property(nonatomic,readonly,getter=isCurrent) BOOL current;
@property(nonatomic,readonly) double dimensionMetersPerUnit;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DEdgeTreatmentCaptureR2 : NSObject
@property(nonatomic,readonly) Core3DEdgeTreatmentStatus status;
@property(nonatomic,strong,readonly,nullable) Core3DEdgeTreatmentSnapshotR2 *snapshot;
@property(nonatomic,copy,readonly) NSString *refusalCode, *refusalMessage;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DRetainedBooleanLegacySelectorBindingR2 : NSObject
@property(nonatomic,readonly) uint64_t oldStepID;
@property(nonatomic,strong,readonly) Core3DFaceSelectorIntent *intent;
- (nullable instancetype)initWithOldStepID:(uint64_t)oldStepID
    intent:(Core3DFaceSelectorIntent *)intent NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;
@end
@interface Core3DRetainedBooleanSelectorAppendR2 : NSObject
@property(nonatomic,strong,readonly) Core3DFaceSelectorIntent *intent;
@property(nonatomic,readonly) double amountMM;
- (nullable instancetype)initWithIntent:(Core3DFaceSelectorIntent *)intent
    amountMM:(double)amountMM NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;
@end
@interface Core3DRetainedBooleanMigrationRequestR2 : NSObject
@property(nonatomic,readonly) uint32_t version;
@property(nonatomic,copy,readonly) NSArray<Core3DRetainedBooleanLegacySelectorBindingR2 *> *selectors;
@property(nonatomic,strong,readonly,nullable) Core3DRetainedBooleanSelectorAppendR2 *append;
- (nullable instancetype)initWithSelectors:(NSArray<Core3DRetainedBooleanLegacySelectorBindingR2 *> *)selectors
    append:(nullable Core3DRetainedBooleanSelectorAppendR2 *)append NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;
@end

@interface Core3DRetainedBooleanMigrationCaptureR2 : NSObject
@property(nonatomic,copy,readonly) NSString *documentIdentifier, *entityIdentifier, *definitionIdentifier;
@property(nonatomic,strong,readonly) Core3DRetainedBooleanSourceR2 *source;
@property(nonatomic,copy,readonly) NSArray<Core3DEdgeTreatmentStep *> *legacySteps;
@property(nonatomic,copy,readonly) NSData *canonicalOriginalBytes;
@property(nonatomic,readonly,getter=isCurrent) BOOL current;
@property(nonatomic,copy,readonly) NSString *refusalCode, *refusalMessage;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DFaceSelectorProofR2 : NSObject
@property(nonatomic,readonly) uint32_t contractRevision;
@property(nonatomic,strong,readonly) Core3DFaceSelectorProof *selectorProof;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end
@interface Core3DFaceSelectorQueryR2 : NSObject
@property(nonatomic,readonly) Core3DFaceSelectorStatus status;
@property(nonatomic,readonly) Core3DFaceSelectorRefusal refusal;
@property(nonatomic,copy,readonly) NSString *refusalCode, *refusalMessage;
@property(nonatomic,strong,readonly,nullable) Core3DFaceSelectorProofR2 *proof;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DRetainedBooleanMigrationProofR2 : NSObject
@property(nonatomic,copy,readonly) NSData *requestDigest;
@property(nonatomic,copy,readonly) NSArray<Core3DFaceSelectorProofR2 *> *existingStepProofs;
@property(nonatomic,strong,readonly,nullable) Core3DFaceSelectorProofR2 *appendProof;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end
@interface Core3DRetainedBooleanMigrationReviewR2 : NSObject
@property(nonatomic,strong,readonly,nullable) Core3DRetainedBooleanMigrationProofR2 *proof;
@property(nonatomic,copy,readonly) NSString *refusalCode, *refusalMessage;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DRetainedBooleanEditR2 : NSObject
@property(nonatomic,readonly) Core3DRetainedBooleanEditKindR2 kind;
@property(nonatomic,copy,readonly,nullable) NSString *featureIdentifier;
@property(nonatomic,strong,readonly,nullable) Core3DRetainedBooleanRecipeLocatorR2 *locator;
@property(nonatomic,strong,readonly,nullable) Core3DRetainedBooleanInputR2 *completeInput;
@property(nonatomic,strong,readonly,nullable) Core3DRetainedBooleanAnalyticToolR2 *completeAnalyticTool;
@property(nonatomic,strong,readonly,nullable) Core3DRetainedBooleanInputPlacementR2 *completeInputPlacement;
@property(nonatomic,strong,readonly,nullable) NSNumber *amountMM, *operation, *operandID;
- (instancetype)init NS_UNAVAILABLE; + (instancetype)new NS_UNAVAILABLE;
@end
NS_ASSUME_NONNULL_END
