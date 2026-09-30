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

@implementation Core3DAgentSolidMeasurement
@end

@implementation Core3DAgentLineSectionMeasurement
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
