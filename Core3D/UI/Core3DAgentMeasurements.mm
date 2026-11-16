#import "Core3DAgentMeasurements.h"

#include <cmath>

@implementation Core3DAgentMeasurementSource
- (nullable instancetype)initWithTargetAlias:(NSString *)targetAlias
                          documentIdentifier:(NSString *)documentIdentifier
                 publicationSourceIdentifier:(NSString *)publicationSourceIdentifier
                          documentGeneration:(NSString *)documentGeneration
                                modelRevision:(NSString *)modelRevision
                         presentationRevision:(NSString *)presentationRevision
                              metersPerUnit:(double)metersPerUnit
                                       method:(NSString *)method
                                 sourceSHA256:(NSString *)sourceSHA256 {
    self = [super init];
    if (self == nil) return nil;
    NSCharacterSet *allowed = [NSCharacterSet characterSetWithCharactersInString:
        @"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._:-"];
    BOOL aliasOK = targetAlias.length > 0 && targetAlias.length <= 128
        && [targetAlias rangeOfCharacterFromSet:allowed.invertedSet].location == NSNotFound;
    NSRegularExpression *sha = [NSRegularExpression regularExpressionWithPattern:@"^[0-9a-f]{64}$" options:0 error:nil];
    BOOL digestOK = [sha numberOfMatchesInString:sourceSHA256 options:0 range:NSMakeRange(0, sourceSHA256.length)] == 1;
    if (!aliasOK || documentIdentifier.length == 0 || publicationSourceIdentifier.length == 0
        || documentGeneration.length == 0 || modelRevision.length == 0
        || presentationRevision.length == 0 || !std::isfinite(metersPerUnit)
        || metersPerUnit <= 0 || method.length == 0 || !digestOK) return nil;
    _targetAlias = [targetAlias copy];
    _documentIdentifier = [documentIdentifier copy];
    _publicationSourceIdentifier = [publicationSourceIdentifier copy];
    _documentGeneration = [documentGeneration copy];
    _modelRevision = [modelRevision copy];
    _presentationRevision = [presentationRevision copy];
    _metersPerUnit = metersPerUnit;
    _method = [method copy];
    _sourceSHA256 = [sourceSHA256 copy];
    return self;
}
@end

// Engine-only producers (measure lane, CTO 15 Nov 21:50). Declared again as a
// category in Core3DViewController.mm, the only caller; there is no public
// initializer, so no caller outside the engine can forge a measured value.
@implementation Core3DAgentSolidMeasurement
- (instancetype)initCore3DUnmeasured:(Core3DAgentMeasurementStatus)status {
    self = [super init];
    if (self == nil) return nil;
    _status = status == Core3DAgentMeasurementStatusMeasured ? Core3DAgentMeasurementStatusUnavailable : status;
    _boundsMinMM = @[]; _boundsMaxMM = @[]; _localBoundsOptimalMinMM = @[]; _localBoundsOptimalMaxMM = @[];
    return self;
}
- (instancetype)initCore3DMeasuredWithSource:(Core3DAgentMeasurementSource *)source
                                  boundsMinMM:(NSArray<NSNumber *> *)boundsMinMM
                                  boundsMaxMM:(NSArray<NSNumber *> *)boundsMaxMM
                      localBoundsOptimalMinMM:(NSArray<NSNumber *> *)localMinMM
                      localBoundsOptimalMaxMM:(NSArray<NSNumber *> *)localMaxMM
                                    volumeMM3:(double)volumeMM3
                volumeIntegrationRelativeError:(double)volumeError
                                        valid:(BOOL)valid
                                       solids:(NSUInteger)solids shells:(NSUInteger)shells
                                        faces:(NSUInteger)faces edges:(NSUInteger)edges
                                     vertices:(NSUInteger)vertices topologyNodes:(NSUInteger)topologyNodes {
    self = [super init];
    if (self == nil) return nil;
    _status = Core3DAgentMeasurementStatusMeasured;
    _source = source;
    _boundsMinMM = [boundsMinMM copy]; _boundsMaxMM = [boundsMaxMM copy];
    _localBoundsOptimalMinMM = [localMinMM copy]; _localBoundsOptimalMaxMM = [localMaxMM copy];
    _volumeMM3 = volumeMM3; _volumeIntegrationRelativeError = volumeError; _valid = valid;
    _solids = solids; _shells = shells; _faces = faces; _edges = edges; _vertices = vertices;
    _topologyNodes = topologyNodes;
    return self;
}
@end

@implementation Core3DAgentLineSectionMeasurement
- (instancetype)initCore3DUnmeasured:(Core3DAgentMeasurementStatus)status {
    self = [super init];
    if (self == nil) return nil;
    _status = status == Core3DAgentMeasurementStatusMeasured ? Core3DAgentMeasurementStatusUnavailable : status;
    _probeIdentifier = @""; _originMM = @[]; _requestedDirection = @[]; _intervalsMM = @[];
    return self;
}
- (instancetype)initCore3DMeasuredWithSource:(Core3DAgentMeasurementSource *)source
                              probeIdentifier:(NSString *)probeIdentifier
                                     originMM:(NSArray<NSNumber *> *)originMM
                           requestedDirection:(NSArray<NSNumber *> *)requestedDirection
                                  intervalsMM:(NSArray<NSArray<NSNumber *> *> *)intervalsMM {
    self = [super init];
    if (self == nil) return nil;
    _status = Core3DAgentMeasurementStatusMeasured;
    _source = source;
    _probeIdentifier = [probeIdentifier copy]; _originMM = [originMM copy];
    _requestedDirection = [requestedDirection copy]; _intervalsMM = [intervalsMM copy];
    return self;
}
@end

@implementation Core3DAgentDistanceMeasurement
- (instancetype)initCore3DUnmeasured:(Core3DAgentMeasurementStatus)status {
    self = [super init];
    if (self == nil) return nil;
    _status = status == Core3DAgentMeasurementStatusMeasured ? Core3DAgentMeasurementStatusUnavailable : status;
    _probeIdentifier = @""; _requestedPointAMM = @[]; _requestedPointBMM = @[];
    _pointAMM = @[]; _pointBMM = @[]; _snapA = @""; _snapB = @"";
    return self;
}
- (instancetype)initCore3DMeasuredWithSource:(Core3DAgentMeasurementSource *)source
                              probeIdentifier:(NSString *)probeIdentifier
                            requestedPointAMM:(NSArray<NSNumber *> *)requestedA
                            requestedPointBMM:(NSArray<NSNumber *> *)requestedB
                                     pointAMM:(NSArray<NSNumber *> *)pointA
                                     pointBMM:(NSArray<NSNumber *> *)pointB
                                        snapA:(NSString *)snapA snapB:(NSString *)snapB
                                   distanceMM:(double)distanceMM {
    self = [super init];
    if (self == nil) return nil;
    _status = Core3DAgentMeasurementStatusMeasured;
    _source = source;
    _probeIdentifier = [probeIdentifier copy];
    _requestedPointAMM = [requestedA copy]; _requestedPointBMM = [requestedB copy];
    _pointAMM = [pointA copy]; _pointBMM = [pointB copy];
    _snapA = [snapA copy]; _snapB = [snapB copy];
    _distanceMM = distanceMM;
    return self;
}
@end

@implementation Core3DAgentAngleMeasurement
- (instancetype)initCore3DUnmeasured:(Core3DAgentMeasurementStatus)status {
    self = [super init];
    if (self == nil) return nil;
    _status = status == Core3DAgentMeasurementStatusMeasured ? Core3DAgentMeasurementStatusUnavailable : status;
    _probeIdentifier = @""; _faceAPointMM = @[]; _faceBPointMM = @[]; _faceANormal = @[]; _faceBNormal = @[];
    return self;
}
- (instancetype)initCore3DMeasuredWithSource:(Core3DAgentMeasurementSource *)source
                              probeIdentifier:(NSString *)probeIdentifier
                                 faceAPointMM:(NSArray<NSNumber *> *)faceAPoint
                                 faceBPointMM:(NSArray<NSNumber *> *)faceBPoint
                                  faceANormal:(NSArray<NSNumber *> *)faceANormal
                                  faceBNormal:(NSArray<NSNumber *> *)faceBNormal
                                 angleDegrees:(double)angleDegrees {
    self = [super init];
    if (self == nil) return nil;
    _status = Core3DAgentMeasurementStatusMeasured;
    _source = source;
    _probeIdentifier = [probeIdentifier copy];
    _faceAPointMM = [faceAPoint copy]; _faceBPointMM = [faceBPoint copy];
    _faceANormal = [faceANormal copy]; _faceBNormal = [faceBNormal copy];
    _angleDegrees = angleDegrees;
    return self;
}
@end

@implementation Core3DAgentMaterialMeasurement
@end


@implementation Core3DAgentWorldPlacementMeasurement
@end

@implementation Core3DAgentStateCapture
- (nullable instancetype)initWithDocumentIdentifier:(NSString *)documentIdentifier
                                 documentGeneration:(NSString *)documentGeneration
                                       modelRevision:(NSString *)modelRevision
                                presentationRevision:(NSString *)presentationRevision
                                     metersPerUnit:(double)metersPerUnit
                                           undoDepth:(NSInteger)undoDepth
                                           redoDepth:(NSInteger)redoDepth
                                  commitmentPayloads:(NSDictionary<NSString *,NSData *> *)commitmentPayloads {
    self = [super init];
    if (self == nil) return nil;
    NSSet<NSString *> *required = [NSSet setWithArray:Core3DAgentMeasurements.commitmentKeys];
    NSSet<NSString *> *supplied = [NSSet setWithArray:commitmentPayloads.allKeys];
    if (documentIdentifier.length == 0 || documentGeneration.length == 0
        || modelRevision.length == 0 || presentationRevision.length == 0
        || !std::isfinite(metersPerUnit) || metersPerUnit <= 0
        || undoDepth < 0 || redoDepth < 0 || ![required isEqualToSet:supplied]) return nil;
    for (NSString *key in required) {
        NSData *payload = commitmentPayloads[key];
        if (![payload isKindOfClass:NSData.class] || payload.length == 0) return nil;
    }
    _documentIdentifier = [documentIdentifier copy];
    _documentGeneration = [documentGeneration copy];
    _modelRevision = [modelRevision copy];
    _presentationRevision = [presentationRevision copy];
    _metersPerUnit = metersPerUnit;
    _undoDepth = undoDepth;
    _redoDepth = redoDepth;
    _commitmentPayloads = [[NSDictionary alloc] initWithDictionary:commitmentPayloads copyItems:YES];
    return self;
}
@end

@implementation Core3DAgentMeasurements
+ (BOOL)observedUnchangedQualified { return NO; }
+ (NSArray<NSString *> *)commitmentKeys {
    return @[@"ownerSet", @"recipes", @"shape", @"placement", @"material", @"selection", @"gizmo", @"mesh"];
}
@end
