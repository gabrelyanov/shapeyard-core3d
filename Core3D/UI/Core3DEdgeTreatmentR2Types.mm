#import "Core3DEdgeTreatmentR2Types.h"
#include "../OCCTKit/RetainedEdgeTreatmentR2Snapshot.hxx"
#include "../OCCTKit/ProfilePersistence.hxx"
#include "../OCCTKit/EnclosureParameters.hxx"
#include "../OCCTKit/RectangularLoftPersistence.hxx"
#include <CommonCrypto/CommonDigest.h>
#include <objc/runtime.h>

// Private native value accessors; implementations live in Core3DModelingTypes.mm.
@interface Core3DProfileDefinition ()
- (core3d::profile::Parameters)nativeParameters;
@end
@interface Core3DEnclosureDefinition ()
- (core3d::enclosure::Parameters)nativeParameters;
@end
@interface Core3DRectangularLoftDefinition ()
- (core3d::rectangular_loft::Definition)nativeDefinition;
@end

static BOOL R2UUIDValid(NSString *value) {
    return value.length > 0 && [[NSUUID alloc] initWithUUIDString:value] != nil;
}
static NSString *R2SHA256Hex(NSData *bytes) {
    if (!bytes.length) return @"";
    unsigned char digest[CC_SHA256_DIGEST_LENGTH];
    CC_SHA256(bytes.bytes, CC_LONG(bytes.length), digest);
    NSMutableString *hex = [NSMutableString stringWithString:@"sha256:"];
    for (std::size_t index = 0; index < CC_SHA256_DIGEST_LENGTH; ++index) [hex appendFormat:@"%02x", digest[index]];
    return hex;
}
static NSData *R2DoubleData(const std::vector<double>& values) {
    return values.empty() ? NSData.data : [NSData dataWithBytes:values.data() length:values.size() * sizeof(double)];
}

@implementation Core3DRetainedBooleanRecipeLocatorR2
- (nullable instancetype)initWithDocumentIdentifier:(NSString *)documentIdentifier
    entityIdentifier:(NSString *)entityIdentifier definitionIdentifier:(NSString *)definitionIdentifier
    nodeIdentifier:(NSString *)nodeIdentifier sourceFeatureIdentifier:(NSString *)sourceFeatureIdentifier {
    if (!R2UUIDValid(documentIdentifier) || !R2UUIDValid(entityIdentifier) || !R2UUIDValid(definitionIdentifier)
        || !R2UUIDValid(nodeIdentifier) || !R2UUIDValid(sourceFeatureIdentifier)) return nil;
    if ((self = [super init])) {
        _documentIdentifier = [documentIdentifier copy]; _entityIdentifier = [entityIdentifier copy];
        _definitionIdentifier = [definitionIdentifier copy]; _nodeIdentifier = [nodeIdentifier copy];
        _sourceFeatureIdentifier = [sourceFeatureIdentifier copy];
    }
    return self;
}
@end
@implementation Core3DRetainedBooleanAnalyticToolR2
- (nullable instancetype)initWithOperandID:(uint32_t)operandID kind:(NSInteger)kind extent:(NSInteger)extent
    axis:(NSInteger)axis point:(Core3DEdgeTreatmentVector3 *)point radius:(double)radius
    boltCircleRadius:(double)boltCircleRadius hostRadiusRatio:(double)hostRadiusRatio
    directionAngle:(double)directionAngle halfWidthApex:(double)halfWidthApex
    halfWidthMouth:(double)halfWidthMouth length:(double)length count:(NSUInteger)count {
    const double values[] = {radius, boltCircleRadius, hostRadiusRatio, directionAngle,
        halfWidthApex, halfWidthMouth, length};
    for (double value : values) if (!isfinite(value)) return nil;
    // Kind-sensitive radius floor (D347): the retained wedge (kind 3, matching
    // analytic_boolean::OperandKind::Wedge) carries radius exactly zero by
    // contract — its validity is the direction/apex/mouth/length set checked
    // natively — so zero is admitted for that kind only. Negative, non-finite
    // and over-cap radii stay refused for every kind; radius <= 0 stays
    // refused for cylinder and ring.
    const BOOL wedge = kind == 3;
    if (!operandID || kind < 1 || kind > 3 || extent != 1 || axis < 0 || axis > 2 || !point
        || (wedge ? radius < 0 : radius <= 0) || radius > 1.0e6 || boltCircleRadius < 0
        || hostRadiusRatio < 0
        || hostRadiusRatio > 1 || length < 0 || count > 256) return nil;
    if ((self = [super init])) {
        _operandID = operandID; _kind = kind; _extent = extent; _axis = axis; _point = point;
        _radius = radius; _boltCircleRadius = boltCircleRadius; _hostRadiusRatio = hostRadiusRatio;
        _directionAngle = directionAngle; _halfWidthApex = halfWidthApex;
        _halfWidthMouth = halfWidthMouth; _length = length; _count = count;
    }
    return self;
}
@end
@implementation Core3DRetainedBooleanInputPlacementR2
- (nullable instancetype)initWithRowMajorMatrix:(NSArray<NSNumber *> *)rowMajorMatrix
    sourceMetersPerUnit:(double)sourceMetersPerUnit carrierMetersPerUnit:(double)carrierMetersPerUnit {
    if (rowMajorMatrix.count != 16 || !isfinite(sourceMetersPerUnit) || sourceMetersPerUnit <= 0
        || !isfinite(carrierMetersPerUnit) || carrierMetersPerUnit <= 0) return nil;
    for (NSNumber *entry in rowMajorMatrix) if (!isfinite(entry.doubleValue)) return nil;
    if ((self = [super init])) {
        _rowMajorMatrix = [rowMajorMatrix copy];
        _sourceMetersPerUnit = sourceMetersPerUnit; _carrierMetersPerUnit = carrierMetersPerUnit;
    }
    return self;
}
@end
@interface Core3DRetainedBooleanInputR2 ()
- (instancetype)initPrivate;
@end
@implementation Core3DRetainedBooleanInputR2
- (instancetype)initPrivate { return [super init]; }
@end
@implementation Core3DRetainedBooleanOperationR2 @end
@implementation Core3DRetainedBooleanMigrationStepMapR2 @end
@implementation Core3DRetainedBooleanMigrationAnchorMapR2 @end
@implementation Core3DRetainedBooleanSourceR2 @end
@implementation Core3DEdgeTreatmentSnapshotR2 @end
@implementation Core3DEdgeTreatmentCaptureR2 @end
@implementation Core3DRetainedBooleanMigrationCaptureR2 @end
@implementation Core3DFaceSelectorProofR2 @end
@implementation Core3DFaceSelectorQueryR2 @end
@implementation Core3DRetainedBooleanMigrationProofR2 @end
@implementation Core3DRetainedBooleanMigrationReviewR2 @end
@implementation Core3DRetainedBooleanEnrollmentCaptureR2 @end
@implementation Core3DRetainedBooleanEnrollmentProofR2 @end
@implementation Core3DRetainedBooleanEnrollmentReviewR2 @end
@interface Core3DRetainedBooleanEditR2 ()
- (instancetype)initPrivate;
@end
@implementation Core3DRetainedBooleanEditR2
- (instancetype)initPrivate { return [super init]; }
+ (nullable instancetype)editSetAmountWithFeatureIdentifier:(NSString *)featureIdentifier amountMM:(double)amountMM {
    if (!R2UUIDValid(featureIdentifier) || !isfinite(amountMM) || amountMM <= 0 || amountMM > 20) return nil;
    auto edit = (Core3DRetainedBooleanEditR2 *)class_createInstance(Core3DRetainedBooleanEditR2.class, 0);
    [edit setValue:@(Core3DRetainedBooleanEditKindR2SetAmount) forKey:@"kind"];
    [edit setValue:featureIdentifier forKey:@"featureIdentifier"];
    [edit setValue:@(amountMM) forKey:@"amountMM"];
    return edit;
}
+ (nullable instancetype)editRemoveWithFeatureIdentifier:(NSString *)featureIdentifier {
    if (!R2UUIDValid(featureIdentifier)) return nil;
    auto edit = (Core3DRetainedBooleanEditR2 *)class_createInstance(Core3DRetainedBooleanEditR2.class, 0);
    [edit setValue:@(Core3DRetainedBooleanEditKindR2Remove) forKey:@"kind"];
    [edit setValue:featureIdentifier forKey:@"featureIdentifier"];
    return edit;
}
+ (nullable instancetype)editRebuildBooleanInputWithLocator:(Core3DRetainedBooleanRecipeLocatorR2 *)locator
    profile:(Core3DProfileDefinition *)profile enclosure:(Core3DEnclosureDefinition *)enclosure
    rectangularLoft:(Core3DRectangularLoftDefinition *)rectangularLoft {
    const NSUInteger supplied = (profile ? 1 : 0) + (enclosure ? 1 : 0) + (rectangularLoft ? 1 : 0);
    if (!locator || supplied != 1) return nil;
    std::vector<double> values; std::uint32_t schema = 0; BOOL encoded = NO;
    if (profile) {
        const auto parameters = [profile nativeParameters];
        encoded = core3d::profile::Encode(parameters, values); schema = std::uint32_t(core3d::profile::SchemaFor(parameters));
    } else if (enclosure) {
        const auto parameters = [enclosure nativeParameters];
        encoded = core3d::enclosure::Encode(parameters, values);
        schema = std::uint32_t(parameters.definition.constructionFrame
            ? core3d::enclosure::FramedSchemaVersion : core3d::enclosure::SchemaVersion);
    } else {
        const auto definition = [rectangularLoft nativeDefinition];
        encoded = core3d::loft_persistence::Encode(definition, values); schema = std::uint32_t(core3d::loft_persistence::Schema);
    }
    if (!encoded) return nil;
    NSData *canonical = R2DoubleData(values);
    auto input = [[Core3DRetainedBooleanInputR2 alloc] initPrivate];
    [input setValue:locator forKey:@"locator"];
    [input setValue:profile forKey:@"profile"];
    [input setValue:enclosure forKey:@"enclosure"];
    [input setValue:rectangularLoft forKey:@"rectangularLoft"];
    [input setValue:canonical forKey:@"canonicalRecipeBytes"];
    [input setValue:R2SHA256Hex(canonical) forKey:@"recipeDigest"];
    [input setValue:@(schema) forKey:@"recipeSchema"];
    auto edit = (Core3DRetainedBooleanEditR2 *)class_createInstance(Core3DRetainedBooleanEditR2.class, 0);
    [edit setValue:@(Core3DRetainedBooleanEditKindR2RebuildBooleanInput) forKey:@"kind"];
    [edit setValue:locator forKey:@"locator"];
    [edit setValue:input forKey:@"completeInput"];
    return edit;
}
+ (nullable instancetype)editRebuildAnalyticToolWithOperandID:(uint32_t)operandID
    completeAnalyticTool:(Core3DRetainedBooleanAnalyticToolR2 *)completeAnalyticTool {
    if (!operandID || !completeAnalyticTool || completeAnalyticTool.operandID != operandID) return nil;
    auto edit = (Core3DRetainedBooleanEditR2 *)class_createInstance(Core3DRetainedBooleanEditR2.class, 0);
    [edit setValue:@(Core3DRetainedBooleanEditKindR2RebuildAnalyticTool) forKey:@"kind"];
    [edit setValue:completeAnalyticTool forKey:@"completeAnalyticTool"];
    [edit setValue:@(operandID) forKey:@"operandID"];
    return edit;
}
+ (nullable instancetype)editSetBooleanOperationWithFeatureIdentifier:(NSString *)featureIdentifier operation:(NSInteger)operation {
    if (!R2UUIDValid(featureIdentifier) || operation < 1 || operation > 3) return nil;
    auto edit = (Core3DRetainedBooleanEditR2 *)class_createInstance(Core3DRetainedBooleanEditR2.class, 0);
    [edit setValue:@(Core3DRetainedBooleanEditKindR2SetBooleanOperation) forKey:@"kind"];
    [edit setValue:featureIdentifier forKey:@"featureIdentifier"];
    [edit setValue:@(operation) forKey:@"operation"];
    return edit;
}
+ (nullable instancetype)editSetInputPlacementWithLocator:(Core3DRetainedBooleanRecipeLocatorR2 *)locator
    completeInputPlacement:(Core3DRetainedBooleanInputPlacementR2 *)completeInputPlacement {
    if (!locator || !completeInputPlacement) return nil;
    auto edit = (Core3DRetainedBooleanEditR2 *)class_createInstance(Core3DRetainedBooleanEditR2.class, 0);
    [edit setValue:@(Core3DRetainedBooleanEditKindR2SetInputPlacement) forKey:@"kind"];
    [edit setValue:locator forKey:@"locator"];
    [edit setValue:completeInputPlacement forKey:@"completeInputPlacement"];
    return edit;
}
+ (nullable instancetype)editSetSelectorIntentWithFeatureIdentifier:(NSString *)featureIdentifier
    intent:(Core3DFaceSelectorIntent *)intent {
    if (!R2UUIDValid(featureIdentifier) || !intent) return nil;
    auto edit = (Core3DRetainedBooleanEditR2 *)class_createInstance(Core3DRetainedBooleanEditR2.class, 0);
    [edit setValue:@(Core3DRetainedBooleanEditKindR2SetSelectorIntent) forKey:@"kind"];
    [edit setValue:featureIdentifier forKey:@"featureIdentifier"];
    [edit setValue:intent forKey:@"selectorIntent"];
    return edit;
}
@end

@implementation Core3DRetainedBooleanLegacySelectorBindingR2
- (nullable instancetype)initWithOldStepID:(uint64_t)oldStepID intent:(Core3DFaceSelectorIntent *)intent {
    if (!oldStepID || !intent) return nil;
    if ((self = [super init])) { _oldStepID = oldStepID; _intent = intent; }
    return self;
}
@end
@implementation Core3DRetainedBooleanSelectorAppendR2
- (nullable instancetype)initWithIntent:(Core3DFaceSelectorIntent *)intent amountMM:(double)amountMM {
    if (!intent || !isfinite(amountMM) || amountMM <= 0 || amountMM > 20) return nil;
    if ((self = [super init])) { _intent = intent; _amountMM = amountMM; }
    return self;
}
@end
@implementation Core3DRetainedBooleanEnrollmentRequestR2
- (nullable instancetype)initWithIntent:(Core3DFaceSelectorIntent *)intent amountMM:(double)amountMM {
    if (!intent || !isfinite(amountMM) || amountMM <= 0 || amountMM > 20) return nil;
    if ((self = [super init])) { _version = 1; _intent = intent; _amountMM = amountMM; }
    return self;
}
@end
@implementation Core3DRetainedBooleanMigrationRequestR2
- (nullable instancetype)initWithSelectors:(NSArray<Core3DRetainedBooleanLegacySelectorBindingR2 *> *)selectors
    append:(Core3DRetainedBooleanSelectorAppendR2 *)append {
    if (selectors.count > 8) return nil;
    uint64_t previous = 0;
    for (Core3DRetainedBooleanLegacySelectorBindingR2 *binding in selectors) {
        if (binding.oldStepID <= previous) return nil;
        previous = binding.oldStepID;
    }
    if ((self = [super init])) { _version = 1; _selectors = [selectors copy]; _append = append; }
    return self;
}
@end

// Native-only subclasses in Core3DViewController.mm retain the corresponding
// r2 Snapshot/MigrationCapture/Review/SelectorTargetCapture.  No public DTO
// initializer can mint those proof-bearing values.
