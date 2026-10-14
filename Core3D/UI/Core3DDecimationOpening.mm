#import "Core3DDecimationOpening.h"
#import "Core3DPublishCapture.h"

#include "../OCCTKit/ExportMeshDecimator.hxx"
#include <atomic>
#include <cstring>
#include <memory>
#include <vector>

NSErrorDomain const Core3DDecimationErrorDomain = @"Core3DDecimationErrorDomain";

namespace {
using NativeError = core3d::export_mesh_decimator::Error;

bool LowerHexDigest(NSString *value) {
    if (value.length != 64) return false;
    NSCharacterSet *allowed = [NSCharacterSet characterSetWithCharactersInString:@"0123456789abcdef"];
    return [value rangeOfCharacterFromSet:allowed.invertedSet].location == NSNotFound;
}

Core3DDecimationError PublicError(NativeError value) {
    switch (value) {
        case NativeError::TargetBelowClosedMeshMinimum:
            return Core3DDecimationErrorTargetBelowClosedMeshMinimum;
        case NativeError::InvalidTopology: return Core3DDecimationErrorInvalidTopology;
        case NativeError::NonFiniteInput: return Core3DDecimationErrorNonFiniteInput;
        case NativeError::ResourceLimit: return Core3DDecimationErrorResourceLimit;
        case NativeError::Cancelled: return Core3DDecimationErrorCancelled;
        case NativeError::UnreachableTriangleBudget:
            return Core3DDecimationErrorUnreachableTriangleBudget;
        case NativeError::None: return Core3DDecimationErrorMalformedRequest;
    }
}

void SetError(NSError **error, Core3DDecimationError code, NSString *message) {
    if (error) *error = [NSError errorWithDomain:Core3DDecimationErrorDomain code:code
        userInfo:@{NSLocalizedDescriptionKey: message}];
}
}

@interface Core3DDecimationRequest ()
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *positionsMM;
@property(nonatomic, copy, readwrite) NSArray<NSNumber *> *indices;
@property(nonatomic, copy, readwrite) NSArray<NSString *> *sourceContributionTokens;
@property(nonatomic, copy, readwrite) NSData *materialBytes;
@property(nonatomic, copy, readwrite) NSString *sourceArtifactSHA256;
@property(nonatomic, copy, readwrite) NSString *settingsSHA256;
@property(nonatomic, assign, readwrite) NSUInteger targetTriangleCount;
@property(nonatomic, assign, readwrite) double metersPerUnit;
@end

@implementation Core3DDecimationRequest
- (instancetype)initWithPositionsMM:(NSArray<NSNumber *> *)positionsMM
                             indices:(NSArray<NSNumber *> *)indices
            sourceContributionTokens:(NSArray<NSString *> *)sourceContributionTokens
                       materialBytes:(NSData *)materialBytes
                sourceArtifactSHA256:(NSString *)sourceArtifactSHA256
                      settingsSHA256:(NSString *)settingsSHA256
                 targetTriangleCount:(NSUInteger)targetTriangleCount
                       metersPerUnit:(double)metersPerUnit {
    if ((self = [super init])) {
        _positionsMM = [positionsMM copy]; _indices = [indices copy];
        _sourceContributionTokens = [sourceContributionTokens copy];
        _materialBytes = [materialBytes copy];
        _sourceArtifactSHA256 = [sourceArtifactSHA256 copy];
        _settingsSHA256 = [settingsSHA256 copy];
        _targetTriangleCount = targetTriangleCount; _metersPerUnit = metersPerUnit;
    }
    return self;
}
@end

@interface Core3DDecimationResult ()
- (instancetype)initWithNative:(const core3d::export_mesh_decimator::Output&)native
                 materialBytes:(NSData *)materialBytes;
@property(nonatomic, copy, readwrite) NSData *positionFloat32Data;
@property(nonatomic, copy, readwrite) NSData *normalFloat32Data;
@property(nonatomic, copy, readwrite) NSData *indexUInt32Data;
@property(nonatomic, copy, readwrite) NSData *materialBytes;
@property(nonatomic, copy, readwrite) NSArray<NSArray<NSString *> *> *sourceContributionTokens;
@property(nonatomic, copy, readwrite) NSString *normalPolicy;
@property(nonatomic, copy, readwrite) NSString *appearanceLimitation;
@property(nonatomic, assign, readwrite) NSUInteger triangleCount;
@property(nonatomic, assign, readwrite) NSUInteger acceptedCollapseCount;
@end

@implementation Core3DDecimationResult
- (instancetype)initWithNative:(const core3d::export_mesh_decimator::Output&)native
                 materialBytes:(NSData *)materialBytes {
    if ((self = [super init])) {
        _positionFloat32Data = [NSData dataWithBytes:native.positionsMM.data()
            length:native.positionsMM.size() * sizeof(float)];
        _normalFloat32Data = [NSData dataWithBytes:native.flatNormals.data()
            length:native.flatNormals.size() * sizeof(float)];
        _indexUInt32Data = [NSData dataWithBytes:native.indices.data()
            length:native.indices.size() * sizeof(std::uint32_t)];
        _materialBytes = [materialBytes copy];
        NSMutableArray *groups = [NSMutableArray arrayWithCapacity:native.contributionTokens.size()];
        for (const auto& group : native.contributionTokens) {
            NSMutableArray *tokens = [NSMutableArray arrayWithCapacity:group.size()];
            for (const auto& token : group) {
                NSString *value = [[NSString alloc] initWithBytes:token.data()
                    length:token.size() encoding:NSUTF8StringEncoding];
                if (value) [tokens addObject:value];
            }
            [groups addObject:[tokens copy]];
        }
        _sourceContributionTokens = [groups copy];
        _normalPolicy = @"f2-normal-regen.v1";
        _appearanceLimitation = @"Flat per-corner normals preserve hard creases; curved regions remain faceted until S3 smooth-normal transfer.";
        _triangleCount = native.indices.size() / 3;
        _acceptedCollapseCount = native.acceptedCollapses;
    }
    return self;
}
@end

@interface Core3DDecimationOpening () {
    std::shared_ptr<std::atomic_bool> _stop;
}
@property(nonatomic, strong, nullable) Core3DPublishCapture *capture;
@end

@implementation Core3DDecimationOpening
+ (instancetype)openingWithCapture:(Core3DPublishCapture *)capture error:(NSError **)error {
    if (capture && ![capture isCurrent:error]) return nil;
    Core3DDecimationOpening *opening = [[self alloc] initPrivate];
    opening.capture = capture;
    return opening;
}
- (instancetype)initPrivate {
    if ((self = [super init])) _stop = std::make_shared<std::atomic_bool>(false);
    return self;
}
- (BOOL)cancel { return !_stop->exchange(true, std::memory_order_relaxed); }

- (Core3DDecimationResult *)decimateRequest:(Core3DDecimationRequest *)request
                                       error:(NSError **)error {
    if (![request isKindOfClass:Core3DDecimationRequest.class]
        || request.positionsMM.count == 0 || request.positionsMM.count % 3 != 0
        || request.indices.count == 0 || request.indices.count % 3 != 0
        || request.sourceContributionTokens.count != request.indices.count / 3
        || !LowerHexDigest(request.sourceArtifactSHA256)
        || !LowerHexDigest(request.settingsSHA256)
        || (request.metersPerUnit != 0.001 && request.metersPerUnit != 1.0)) {
        SetError(error, Core3DDecimationErrorMalformedRequest, @"Malformed F2 request.");
        return nil;
    }
    if (_stop->load(std::memory_order_relaxed)) {
        SetError(error, Core3DDecimationErrorCancelled, @"F2 request was cancelled."); return nil;
    }
    if (self.capture && ![self.capture isCurrent:error]) {
        SetError(error, Core3DDecimationErrorStaleSource, @"F2 source capture is stale."); return nil;
    }
    core3d::export_mesh_decimator::Input native;
    native.targetTriangles = request.targetTriangleCount;
    native.positionsMM.reserve(request.positionsMM.count / 3);
    for (NSUInteger i = 0; i < request.positionsMM.count; i += 3)
        native.positionsMM.push_back({request.positionsMM[i].doubleValue,
            request.positionsMM[i + 1].doubleValue, request.positionsMM[i + 2].doubleValue});
    native.indices.reserve(request.indices.count);
    for (NSNumber *number in request.indices) {
        const unsigned long long value = number.unsignedLongLongValue;
        if (value > std::numeric_limits<std::uint32_t>::max()) {
            SetError(error, Core3DDecimationErrorMalformedRequest, @"Index is out of range."); return nil;
        }
        native.indices.push_back(static_cast<std::uint32_t>(value));
    }
    for (NSString *token in request.sourceContributionTokens) {
        if (![token isKindOfClass:NSString.class] || token.length == 0) {
            SetError(error, Core3DDecimationErrorMalformedRequest, @"Missing provenance token."); return nil;
        }
        native.contributionTokens.emplace_back(token.UTF8String ?: "");
    }
    core3d::export_mesh_decimator::Output candidate;
    const NativeError status = core3d::export_mesh_decimator::Decimate(native, candidate, _stop.get());
    if (status != NativeError::None) {
        SetError(error, PublicError(status), @"F2 native decimation refused without output."); return nil;
    }
    if (_stop->load(std::memory_order_relaxed)) {
        SetError(error, Core3DDecimationErrorCancelled, @"F2 request was cancelled."); return nil;
    }
    if (self.capture && ![self.capture isCurrent:error]) {
        SetError(error, Core3DDecimationErrorStaleSource, @"F2 source changed during work."); return nil;
    }
    return [[Core3DDecimationResult alloc] initWithNative:candidate
        materialBytes:request.materialBytes];
}
@end
