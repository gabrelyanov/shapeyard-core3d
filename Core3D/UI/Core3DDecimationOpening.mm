#import "Core3DDecimationOpening.h"
#import "Core3DPublishCapture.h"
#import "Core3DViewController.h"

#import "../OCCTKit/GLViewController.h"
#import "Core3DViewer.h"

#include "../OCCTKit/ExportMeshDecimationNativeOwner.hxx"
#include "../OCCTKit/ExportMeshAttributeDecimator.hxx"
#include "../OCCTKit/ExportMeshDecimator.hxx"
#include "../OCCTKit/OcctDocument.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

NSErrorDomain const Core3DDecimationErrorDomain = @"Core3DDecimationErrorDomain";

namespace {
using NativeError = core3d::export_mesh_decimator::Error;

// Set only for the duration of the public borrowed-command staging call.
// Ordinary F2 entry points always observe -1 and retain their level-zero,
// command-owning behavior byte-for-byte.
thread_local NSInteger F2BorrowedStageLevel = -1;

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

using AttributeNativeError = core3d::export_mesh_attribute_decimator::Error;

// Attribute-kernel refusals map onto the declared public codes: provenance
// and domain-ambiguity refusals are evidence failures, never silent output.
Core3DDecimationError PublicAttributeError(AttributeNativeError value) {
    switch (value) {
        case AttributeNativeError::TargetBelowClosedMeshMinimum:
            return Core3DDecimationErrorTargetBelowClosedMeshMinimum;
        case AttributeNativeError::InvalidTopology: return Core3DDecimationErrorInvalidTopology;
        case AttributeNativeError::NonFiniteInput: return Core3DDecimationErrorNonFiniteInput;
        case AttributeNativeError::ResourceLimit: return Core3DDecimationErrorResourceLimit;
        case AttributeNativeError::Cancelled: return Core3DDecimationErrorCancelled;
        case AttributeNativeError::UnreachableTriangleBudget:
            return Core3DDecimationErrorUnreachableTriangleBudget;
        case AttributeNativeError::ProvenanceUnprovable:
        case AttributeNativeError::AttributeDomainAmbiguous:
            return Core3DDecimationErrorMissingArtifactEvidence;
        case AttributeNativeError::None: return Core3DDecimationErrorMalformedRequest;
    }
}

void SetError(NSError **error, Core3DDecimationError code, NSString *message) {
    if (error) *error = [NSError errorWithDomain:Core3DDecimationErrorDomain code:code
        userInfo:@{NSLocalizedDescriptionKey: message}];
}

NSString *F2String(const std::string& value) {
    NSString *result = [[NSString alloc] initWithBytes:value.data() length:value.size()
        encoding:NSUTF8StringEncoding];
    return result ?: @"";
}

std::vector<std::uint8_t> F2Bytes(NSData *data) {
    std::vector<std::uint8_t> out;
    if (data.length > 0) {
        const auto *bytes = static_cast<const std::uint8_t *>(data.bytes);
        out.assign(bytes, bytes + data.length);
    }
    return out;
}

NSData *F2Data(const std::vector<std::uint8_t>& bytes) {
    return bytes.empty() ? [NSData data]
        : [NSData dataWithBytes:bytes.data() length:bytes.size()];
}

Handle(OcctDocument) F2DocumentForController(Core3DViewController *controller) {
    if (!controller) return Handle(OcctDocument)();
    GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)controller.glController : nil;
    const std::shared_ptr<core3d::Core3DViewer> viewer = gl ? gl.viewer : nullptr;
    return viewer ? viewer->getDocument() : Handle(OcctDocument)();
}

// Canonical internal encoding of the full producer request, used only for the
// nonserializable producer seal equality proof; it never crosses the wire.
bool F2CanonicalRequestBytes(Core3DDecimationRequest *request,
                             std::vector<std::uint8_t>& out) {
    namespace f2 = core3d::export_decimation;
    out.clear();
    try { @try {
        f2::detail::Writer w;
        w.Text("shapeyard.export-mesh-decimation.v1/request");
        w.U64(request.positionsMM.count);
        for (NSNumber *number in request.positionsMM) {
            const double value = number.doubleValue;
            if (!std::isfinite(value)) return false;
            w.F64(value);
        }
        w.U64(request.indices.count);
        for (NSNumber *number in request.indices) {
            const unsigned long long value = number.unsignedLongLongValue;
            if (value > std::numeric_limits<std::uint32_t>::max()) return false;
            w.U32(static_cast<std::uint32_t>(value));
        }
        w.U64(request.sourceContributionTokens.count);
        for (NSString *token in request.sourceContributionTokens) {
            if (![token isKindOfClass:NSString.class]) return false;
            w.Text(token.UTF8String ?: "");
        }
        w.Blob(F2Bytes(request.materialBytes));
        w.Text(request.sourceArtifactSHA256.UTF8String ?: "");
        w.Text(request.settingsSHA256.UTF8String ?: "");
        if (request.targetTriangleCount > std::numeric_limits<std::uint32_t>::max()
            || !std::isfinite(request.metersPerUnit)) return false;
        w.U32(static_cast<std::uint32_t>(request.targetTriangleCount));
        w.F64(request.metersPerUnit);
        out = std::move(w.bytes);
        return true;
    } @catch (__unused NSException *exception) { out.clear(); return false; }
    } catch (...) { out.clear(); return false; }
}

bool F2NumberArraysEqual(NSArray<NSNumber *> *left, NSArray<NSNumber *> *right) {
    if (![left isKindOfClass:NSArray.class] || ![right isKindOfClass:NSArray.class]
        || left.count != right.count) return false;
    for (NSUInteger index = 0; index < left.count; ++index)
        if (left[index].doubleValue != right[index].doubleValue) return false;
    return true;
}

bool F2EvidenceEqual(Core3DPublishArtifactEvidence *left, Core3DPublishArtifactEvidence *right) {
    if (left == right) return true;
    if (![left isKindOfClass:Core3DPublishArtifactEvidence.class]
        || ![right isKindOfClass:Core3DPublishArtifactEvidence.class]) return false;
    return [left.artifactSHA256 isEqualToString:right.artifactSHA256]
        && F2NumberArraysEqual(left.sourcePositionsMM, right.sourcePositionsMM)
        && F2NumberArraysEqual(left.sourceIndices, right.sourceIndices)
        && [left.faceEvidenceSHA256InEmittedOrder
            isEqualToArray:right.faceEvidenceSHA256InEmittedOrder]
        && [left.measurementReceiptSHA256 isEqualToString:right.measurementReceiptSHA256]
        && left.addressedTriangleCount == right.addressedTriangleCount;
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
// F2-S3 attribute payload (all-or-none; nil on every field selects the v1
// flat path). Values are immutable request-owned copies: one world Z-up normal
// (3) and one UV (2) per stored source vertex, one material id and one
// 64-lower-hex source-face anchor per face.
@property(nonatomic, copy, readwrite, nullable) NSArray<NSNumber *> *f2AttributeWedgeNormals;
@property(nonatomic, copy, readwrite, nullable) NSArray<NSNumber *> *f2AttributeWedgeUVs;
@property(nonatomic, copy, readwrite, nullable) NSArray<NSNumber *> *f2AttributeFaceMaterials;
@property(nonatomic, copy, readwrite, nullable) NSArray<NSString *> *f2AttributeFaceAnchors;
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
- (instancetype)initWithPositionsMM:(NSArray<NSNumber *> *)positionsMM
                             indices:(NSArray<NSNumber *> *)indices
            sourceContributionTokens:(NSArray<NSString *> *)sourceContributionTokens
                       materialBytes:(NSData *)materialBytes
                sourceArtifactSHA256:(NSString *)sourceArtifactSHA256
                      settingsSHA256:(NSString *)settingsSHA256
                 targetTriangleCount:(NSUInteger)targetTriangleCount
                       metersPerUnit:(double)metersPerUnit
               attributeWedgeNormals:(NSArray<NSNumber *> *)attributeWedgeNormals
                   attributeWedgeUVs:(NSArray<NSNumber *> *)attributeWedgeUVs
              attributeFaceMaterials:(NSArray<NSNumber *> *)attributeFaceMaterials
                attributeFaceAnchors:(NSArray<NSString *> *)attributeFaceAnchors {
    const NSUInteger storedCount = positionsMM.count / 3;
    const NSUInteger faceCount = indices.count / 3;
    if (positionsMM.count == 0 || positionsMM.count % 3 != 0
        || indices.count == 0 || indices.count % 3 != 0
        || attributeWedgeNormals.count != positionsMM.count
        || attributeWedgeUVs.count != 2 * storedCount
        || attributeFaceMaterials.count != faceCount
        || attributeFaceAnchors.count != faceCount
        || sourceContributionTokens.count != faceCount) return nil;
    self = [self initWithPositionsMM:positionsMM indices:indices
        sourceContributionTokens:sourceContributionTokens materialBytes:materialBytes
        sourceArtifactSHA256:sourceArtifactSHA256 settingsSHA256:settingsSHA256
        targetTriangleCount:targetTriangleCount metersPerUnit:metersPerUnit];
    if (self) {
        _f2AttributeWedgeNormals = [attributeWedgeNormals copy];
        _f2AttributeWedgeUVs = [attributeWedgeUVs copy];
        _f2AttributeFaceMaterials = [attributeFaceMaterials copy];
        _f2AttributeFaceAnchors = [attributeFaceAnchors copy];
    }
    return self;
}
@end

//! The F2 producer seal: native-issued, nonserializable proof binding the
//! original capture, the artifact evidence and the complete canonical request
//! bytes. Created only by decimateRequest: on a capture-bound opening whose
//! capture carried bound evidence at mesh time.
@interface Core3DF2ProducerSeal : NSObject
@property(nonatomic, strong, readonly) Core3DPublishCapture *capture;
@property(nonatomic, strong, readonly) Core3DPublishArtifactEvidence *evidence;
@property(nonatomic, copy, readonly) NSData *canonicalRequestBytes;
- (instancetype)initWithCapture:(Core3DPublishCapture *)capture
                       evidence:(Core3DPublishArtifactEvidence *)evidence
           canonicalRequestBytes:(NSData *)canonicalRequestBytes;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

@implementation Core3DF2ProducerSeal
- (instancetype)initWithCapture:(Core3DPublishCapture *)capture
                       evidence:(Core3DPublishArtifactEvidence *)evidence
           canonicalRequestBytes:(NSData *)canonicalRequestBytes {
    if ((self = [super init])) {
        _capture = capture;
        _evidence = evidence;
        _canonicalRequestBytes = [canonicalRequestBytes copy];
    }
    return self;
}
@end

@interface Core3DDecimationResult ()
- (instancetype)initWithNative:(const core3d::export_mesh_decimator::Output&)native
                 materialBytes:(NSData *)materialBytes;
- (instancetype)initWithAttributeNative:
    (const core3d::export_mesh_attribute_decimator::Output&)native
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
//! F2-S3 attribute derivative evidence (attribute path only; nil on flat
//! results). The public surface is typed and read-only:
//! carried wedge UVs (2 Float32 per corner), one material id and one
//! source-face anchor per emitted triangle, and each emitted triangle's
//! content digest recomputed from the actual emitted Float32 position bytes
//! (never a pre-decimation face digest reused after reordering).
@property(nonatomic, copy, readwrite, nullable) NSData *f2AttributeUVFloat32Data;
@property(nonatomic, copy, readwrite, nullable) NSArray<NSNumber *> *f2AttributeMaterialIDs;
@property(nonatomic, copy, readwrite, nullable) NSArray<NSString *> *f2AttributeSourceFaceAnchors;
@property(nonatomic, copy, readwrite, nullable) NSArray<NSString *> *f2AttributeTriangleContentSHA256;
//! Accepted (keep, remove) welded-vertex rank pairs, flattened in acceptance
//! order, for independent collapse replay by the contract verifier.
@property(nonatomic, copy, readwrite, nullable) NSArray<NSNumber *> *f2AttributeCollapseKeepRemove;
//! Nonserializable F2 producer seal. Set only by decimateRequest: when the
//! opening's capture carried bound artifact evidence; lives only on the live
//! result object and never persists.
@property(nonatomic, strong, readwrite, nullable) Core3DF2ProducerSeal *f2ProducerSeal;
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
- (instancetype)initWithAttributeNative:
    (const core3d::export_mesh_attribute_decimator::Output&)native
                 materialBytes:(NSData *)materialBytes {
    if ((self = [super init])) {
        _positionFloat32Data = [NSData dataWithBytes:native.positionsMM.data()
            length:native.positionsMM.size() * sizeof(float)];
        _normalFloat32Data = [NSData dataWithBytes:native.normals.data()
            length:native.normals.size() * sizeof(float)];
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
        _normalPolicy = @(core3d::export_decimation::kAttributeNormalPolicy);
        _appearanceLimitation = @(core3d::export_decimation::kAttributeAppearanceLimitation);
        _triangleCount = native.indices.size() / 3;
        _acceptedCollapseCount = native.acceptedCollapses;
        _f2AttributeUVFloat32Data = [NSData dataWithBytes:native.uvs.data()
            length:native.uvs.size() * sizeof(float)];
        NSMutableArray *materials = [NSMutableArray arrayWithCapacity:native.faceMaterials.size()];
        for (const std::uint32_t material : native.faceMaterials)
            [materials addObject:@(material)];
        _f2AttributeMaterialIDs = [materials copy];
        NSMutableArray *collapses =
            [NSMutableArray arrayWithCapacity:2 * native.collapseKeepRemove.size()];
        for (const auto& collapse : native.collapseKeepRemove) {
            [collapses addObject:@(collapse.first)];
            [collapses addObject:@(collapse.second)];
        }
        _f2AttributeCollapseKeepRemove = [collapses copy];
    }
    return self;
}
@end

@interface Core3DDecimationSettingsReadback ()
- (instancetype)initWithSettings:(const core3d::export_decimation::CanonicalSettings&)settings
                  canonicalBytes:(NSData *)canonicalBytes
                   settingsDigest:(const core3d::export_decimation::F2Digest&)settingsDigest;
@property(nonatomic, copy, readwrite) NSData *canonicalBytes;
@property(nonatomic, copy, readwrite) NSString *settingsSHA256;
@property(nonatomic, assign, readwrite) NSUInteger targetTriangleCount;
@property(nonatomic, assign, readwrite) double maximumSymmetricSurfaceDistanceMM;
@property(nonatomic, assign, readwrite) double maximumSilhouettePixelDeviation;
@property(nonatomic, assign, readwrite) double hardEdgeAngleDegrees;
@property(nonatomic, assign, readwrite) double contactClearanceMM;
@property(nonatomic, assign, readwrite) NSUInteger measurementSamplesPerMesh;
@property(nonatomic, assign, readwrite) NSUInteger silhouetteResolution;
@property(nonatomic, copy, readwrite) NSString *silhouetteCameraSetSHA256;
@property(nonatomic, assign, readwrite) double metersPerUnit;
@property(nonatomic, copy, readwrite) NSString *schemaVersion;
@property(nonatomic, copy, readwrite) NSString *algorithmIdentifier;
@property(nonatomic, copy, readwrite) NSString *inputPolicy;
@property(nonatomic, copy, readwrite) NSString *coordinateFramePolicy;
@property(nonatomic, copy, readwrite) NSString *normalPolicy;
@property(nonatomic, copy, readwrite) NSString *arithmeticPolicy;
@property(nonatomic, copy, readwrite) NSString *qualityProfileIdentifier;
@property(nonatomic, copy, readwrite) NSString *surfaceDistanceMethod;
@property(nonatomic, copy, readwrite) NSString *silhouetteMethod;
@property(nonatomic, copy, readwrite) NSString *contactMethod;
@property(nonatomic, copy, readwrite) NSString *hardEdgeMethod;
@property(nonatomic, copy, readwrite) NSString *cameraMethod;
@property(nonatomic, copy, readwrite) NSString *algorithmBuildSHA256;
@end

@implementation Core3DDecimationSettingsReadback
- (instancetype)initWithSettings:(const core3d::export_decimation::CanonicalSettings&)settings
                  canonicalBytes:(NSData *)canonicalBytes
                   settingsDigest:(const core3d::export_decimation::F2Digest&)settingsDigest {
    namespace f2 = core3d::export_decimation;
    if ((self = [super init])) {
        _canonicalBytes = [canonicalBytes copy];
        _settingsSHA256 = F2String(f2::HexFromDigest(settingsDigest));
        _targetTriangleCount = settings.targetTriangleCount;
        _maximumSymmetricSurfaceDistanceMM = settings.maximumSymmetricSurfaceDistanceMM;
        _maximumSilhouettePixelDeviation = settings.maximumSilhouettePixelDeviation;
        _hardEdgeAngleDegrees = settings.hardEdgeAngleDegrees;
        _contactClearanceMM = settings.contactClearanceMM;
        _measurementSamplesPerMesh = settings.measurementSamplesPerMesh;
        _silhouetteResolution = settings.silhouetteResolution;
        _silhouetteCameraSetSHA256 =
            F2String(f2::HexFromDigest(settings.silhouetteCameraSetSHA256));
        _metersPerUnit = settings.metersPerUnit;
        _schemaVersion = @(f2::kSchemaVersion);
        _algorithmIdentifier = @(f2::kAlgorithmID);
        _inputPolicy = @"position-only-untextured-static.v1";
        _coordinateFramePolicy = @"native-rh-z-up_to_gltf-rh-y-up.v1";
        _normalPolicy = @"f2-normal-regen.v1";
        _arithmeticPolicy = @"f2-arithmetic.binary64-rne.no-fast-math.v1";
        _qualityProfileIdentifier = @"shapeyard.f2-quality-profile.v1";
        _surfaceDistanceMethod = @"f2-surface-distance.v1";
        _silhouetteMethod = @"f2-silhouette.v1";
        _contactMethod = @"f2-contact.v1";
        _hardEdgeMethod = @"f2-hard-edge.v1";
        _cameraMethod = @"f2-cameras.v1";
        _algorithmBuildSHA256 = F2String(f2::HexFromDigest(f2::AlgorithmBuildSHA256()));
    }
    return self;
}
@end

@interface Core3DPersistedDecimationDerivative ()
- (instancetype)initWithRecord:(const core3d::export_decimation::F2Record&)record
                  documentUUID:(const core3d::export_decimation::F2UUID&)documentUUID;
@property(nonatomic, copy, readwrite) NSString *derivativeIdentifier;
@property(nonatomic, copy, readwrite) NSString *documentIdentifier;
@property(nonatomic, copy, readwrite) NSString *sourceRevisionSHA256;
@property(nonatomic, copy, readwrite) NSString *sourceArtifactSHA256;
@property(nonatomic, copy, readwrite) NSString *sourceGeometrySHA256;
@property(nonatomic, copy, readwrite) NSString *settingsSHA256;
@property(nonatomic, copy, readwrite) NSData *settingsCanonicalBytes;
@property(nonatomic, copy, readwrite) NSString *outputGeometrySHA256;
@property(nonatomic, copy, readwrite) NSString *remapReceiptSHA256;
@property(nonatomic, copy, readwrite) NSString *disclosure;
@property(nonatomic, copy, readwrite) NSString *status;
@property(nonatomic, assign, readwrite) BOOL historicalObservation;
@property(nonatomic, assign, readwrite) NSUInteger emittedTriangleCount;
@property(nonatomic, assign, readwrite) NSUInteger level;
#if DEBUG
@property(nonatomic, copy, readwrite) NSString *ownerSetKey;
#endif
@end

@implementation Core3DPersistedDecimationDerivative
- (instancetype)initWithRecord:(const core3d::export_decimation::F2Record&)record
                  documentUUID:(const core3d::export_decimation::F2UUID&)documentUUID {
    namespace f2 = core3d::export_decimation;
    if ((self = [super init])) {
        _derivativeIdentifier = F2String(f2::HexFromDigest(record.derivativeIdentifier));
        _documentIdentifier = F2String(f2::UUIDText(documentUUID));
        _sourceRevisionSHA256 = F2String(f2::HexFromDigest(record.sourceRevisionSHA256));
        _sourceArtifactSHA256 = F2String(f2::HexFromDigest(record.sourceArtifactSHA256));
        _sourceGeometrySHA256 = F2String(f2::HexFromDigest(record.sourceGeometrySHA256));
        _settingsSHA256 = F2String(f2::HexFromDigest(record.settingsSHA256));
        _settingsCanonicalBytes = F2Data(record.settingsCanonicalBytes);
        _outputGeometrySHA256 = F2String(f2::HexFromDigest(record.outputGeometrySHA256));
        _remapReceiptSHA256 = F2String(f2::HexFromDigest(record.remapReceiptSHA256));
        _disclosure = F2String(f2::kV1Disclosure);
        _status = record.stale ? @"Stale" : @"Current";
        _historicalObservation = record.stale ? YES : NO;
        _emittedTriangleCount = record.emittedTriangleCount;
        _level = record.level;
#if DEBUG
        _ownerSetKey = F2String(f2::HexFromDigest(record.ownerSetKey));
#endif
    }
    return self;
}
@end

namespace {

using F2Record = core3d::export_decimation::F2Record;
using F2UUID = core3d::export_decimation::F2UUID;
using F2Digest = core3d::export_decimation::F2Digest;
namespace f2 = core3d::export_decimation;

const F2Record *F2FindRow(const f2::F2Store& store, const F2Digest& key) {
    for (const F2Record& record : store.records)
        if (record.ownerSetKey == key) return &record;
    return nullptr;
}

// Durable source identities carried by the capture; the terminal retained
// feature keeps the all-zero absent sentinel when the family has none.
bool F2OwnerIdentityFromCapture(Core3DPublishCapture *capture,
                                F2UUID& documentUUID, F2UUID& entityUUID,
                                F2UUID& definitionUUID, F2UUID& sourceFeatureUUID,
                                F2UUID& terminalFeatureUUID) {
    documentUUID = {}; entityUUID = {}; definitionUUID = {};
    sourceFeatureUUID = {}; terminalFeatureUUID = {};
    if (!capture.sourceEntityIdentifier || !capture.sourceDefinitionIdentifier
        || !capture.sourceFeatureIdentifier) return false;
    return f2::UUIDFromText(capture.documentIdentifier.UTF8String ?: "", documentUUID)
        && f2::Nonzero(documentUUID)
        && f2::UUIDFromText(capture.sourceEntityIdentifier.UTF8String ?: "", entityUUID)
        && f2::Nonzero(entityUUID)
        && f2::UUIDFromText(capture.sourceDefinitionIdentifier.UTF8String ?: "", definitionUUID)
        && f2::Nonzero(definitionUUID)
        && f2::UUIDFromText(capture.sourceFeatureIdentifier.UTF8String ?: "", sourceFeatureUUID)
        && f2::Nonzero(sourceFeatureUUID)
        && (!capture.retainedTerminalFeatureIdentifier
            || f2::UUIDFromText(capture.retainedTerminalFeatureIdentifier.UTF8String ?: "",
                terminalFeatureUUID));
}

// Derivative identities are minted natively at first creation only (NSUUID);
// a successful command installs them and later edits keep them.
bool F2MintDerivativeUUIDs(F2UUID out[3]) {
    try { @try {
        for (unsigned i = 0; i < 3; ++i) {
            NSUUID *value = [NSUUID UUID];
            if (!value) return false;
            uuid_t bytes;
            [value getUUIDBytes:bytes];
            std::memcpy(out[i].data(), bytes, 16);
        }
        return f2::Nonzero(out[0]) && f2::Nonzero(out[1]) && f2::Nonzero(out[2])
            && out[0] != out[1] && out[0] != out[2] && out[1] != out[2];
    } @catch (__unused NSException *exception) { return false; }
    } catch (...) { return false; }
}

bool F2RequestGeometry(Core3DDecimationRequest *request,
                       std::vector<double>& positions,
                       std::vector<std::uint32_t>& indices) {
    positions.clear(); indices.clear();
    try { @try {
        positions.reserve(request.positionsMM.count);
        for (NSNumber *number in request.positionsMM)
            positions.push_back(number.doubleValue);
        indices.reserve(request.indices.count);
        for (NSNumber *number in request.indices) {
            const unsigned long long value = number.unsignedLongLongValue;
            if (value > std::numeric_limits<std::uint32_t>::max()) return false;
            indices.push_back(static_cast<std::uint32_t>(value));
        }
        return true;
    } @catch (__unused NSException *exception) { positions.clear(); indices.clear(); return false; }
    } catch (...) { positions.clear(); indices.clear(); return false; }
}

bool F2FaceEvidenceDigests(Core3DPublishArtifactEvidence *evidence,
                           std::vector<F2Digest>& out) {
    out.clear();
    try { @try {
        for (NSString *text in evidence.faceEvidenceSHA256InEmittedOrder) {
            F2Digest digest;
            if (!f2::DigestFromHex(text.UTF8String ?: "", digest)) return false;
            out.push_back(digest);
        }
        return true;
    } @catch (__unused NSException *exception) { out.clear(); return false; }
    } catch (...) { out.clear(); return false; }
}

// Per-emitted-triangle remap groups from the actual result bytes: the content
// digest over the 9 emitted Float32 positions (least cyclic rotation) and the
// strictly sorted unique decoded token digests.
bool F2RemapGroups(Core3DDecimationResult *result, std::vector<f2::RemapGroup>& out) {
    out.clear();
    try { @try {
        NSData *positions = result.positionFloat32Data;
        NSData *indices = result.indexUInt32Data;
        const std::size_t triangleCount = result.sourceContributionTokens.count;
        if (!positions || !indices || positions.length % 12 != 0
            || indices.length % 12 != 0 || indices.length / 12 != triangleCount
            || triangleCount < 4 || triangleCount > f2::kMaxRecordTriangles) return false;
        const auto *positionBytes = static_cast<const std::uint8_t *>(positions.bytes);
        const auto *indexBytes = static_cast<const std::uint8_t *>(indices.bytes);
        const std::size_t vertexCount = positions.length / 12;
        out.reserve(triangleCount);
        for (std::size_t t = 0; t < triangleCount; ++t) {
            f2::RemapGroup group;
            std::uint8_t corners[36];
            for (unsigned corner = 0; corner < 3; ++corner) {
                std::uint32_t vertex = 0;
                for (unsigned b = 0; b < 4; ++b)
                    vertex |= std::uint32_t(indexBytes[12 * t + 4 * corner + b]) << (8 * b);
                if (vertex >= vertexCount) return false;
                std::memcpy(corners + 12 * corner,
                    positionBytes + 12 * static_cast<std::size_t>(vertex), 12);
            }
            group.contentDigest = f2::OutputTriangleContentSHA256(corners);
            NSArray<NSString *> *tokens = result.sourceContributionTokens[t];
            if (![tokens isKindOfClass:NSArray.class] || tokens.count == 0
                || tokens.count > f2::kMaxContributionReferences) return false;
            group.sortedUniqueTokens.reserve(tokens.count);
            for (NSString *token in tokens) {
                F2Digest digest;
                if (!f2::DigestFromHex(token.UTF8String ?: "", digest)) return false;
                group.sortedUniqueTokens.push_back(digest);
            }
            out.push_back(std::move(group));
        }
        return true;
    } @catch (__unused NSException *exception) { out.clear(); return false; }
    } catch (...) { out.clear(); return false; }
}

enum class F2WriteOutcome { Committed, Refused, Stale, Recovery };

// The one owned OCAF command for an F2 store write: NewCommand with no
// command already open, capture rechecked immediately before the command
// opens, expected-before staging with independent readback, one commit,
// post-commit readback byte equality, exactly one undo delta, then history
// finalization. The recheck must precede NewCommand: the native planning
// stamp refuses recapture while any command is open, so an in-command
// recheck could never succeed. Any precommit failure aborts and verifies
// restoration of the exact prior carrier state; uncertain closure is
// Recovery, never a silent outcome.
F2WriteOutcome F2WriteStoreAtomically(const Handle(OcctDocument)& document,
                                      Core3DPublishCapture *capture,
                                      const f2::F2StoreRead& before,
                                      const std::vector<std::uint8_t>& candidate) noexcept {
    try {
        const Handle(TDocStd_Document) ocaf = document.IsNull()
            ? Handle(TDocStd_Document)() : document->Document();
        if (ocaf.IsNull() || ocaf->HasOpenCommand() || candidate.empty()
            || candidate.size() > f2::kMaxStoreBytes || !before.readable())
            return F2WriteOutcome::Refused;
        const Standard_Integer undoBefore = ocaf->GetAvailableUndos();
        // No command is open yet, so a stale capture needs no abort: nothing
        // was staged and the prior carrier state is untouched.
        NSError *currentError = nil;
        if (![capture isCurrent:&currentError])
            return F2WriteOutcome::Stale;
        try {
            ocaf->NewCommand();
        } catch (...) { return F2WriteOutcome::Refused; }
        if (!ocaf->HasOpenCommand()) return F2WriteOutcome::Refused;
        // Aborts the owned command and independently verifies restoration of
        // the exact prior carrier bytes or true absence.
        auto abortAndVerify = [&]() -> bool {
            try {
                if (ocaf->HasOpenCommand()) ocaf->AbortCommand();
            } catch (...) { return false; }
            if (ocaf->HasOpenCommand()) return false;
            f2::F2StoreRead restored;
            return document->ReadF2DerivativeStore(restored)
                && f2::SameStore(before, restored);
        };
        f2::F2StoreRead staged;
        bool stageOK = false;
        try {
            stageOK = document->StageF2DerivativeStore(
                before, candidate, Standard_False, staged);
        } catch (...) { stageOK = false; }
        if (!stageOK || staged.occurrence != f2::F2StoreOccurrence::Present
            || staged.bytes != candidate)
            return abortAndVerify() ? F2WriteOutcome::Refused : F2WriteOutcome::Recovery;
        bool committed = false;
        try {
            committed = ocaf->CommitCommand() && !ocaf->HasOpenCommand();
        } catch (...) { committed = false; }
        if (!committed) return F2WriteOutcome::Recovery; // closure unknown
        f2::F2StoreRead readback;
        if (!document->ReadF2DerivativeStore(readback)
            || readback.occurrence != f2::F2StoreOccurrence::Present
            || readback.bytes != candidate)
            return F2WriteOutcome::Recovery;
        if (ocaf->GetAvailableUndos() - undoBefore != 1
            || !document->FinalizeF2DerivativeHistory())
            return F2WriteOutcome::Recovery;
        return F2WriteOutcome::Committed;
    } catch (...) { return F2WriteOutcome::Recovery; }
}

// F2-S3 independent output verification: re-derives the attribute domains
// from the request payload (canonical weld, protected graph, union-find
// domains, unique domain wedges) and requires every emitted corner to carry
// exactly the recomputed surviving-wedge values and every emitted triangle
// its origin face's material, anchor and contribution tokens. Any
// disagreement refuses the whole derivative; there is no partial admission.
bool F2VerifyAttributeOutput(
    const core3d::export_mesh_attribute_decimator::Input& input,
    const core3d::export_mesh_attribute_decimator::Output& output) {
    namespace attr = core3d::export_mesh_attribute_decimator;
    namespace v1 = core3d::export_mesh_decimator;
    try {
        const std::size_t faceCount = input.indices.size() / 3;
        const std::size_t emitted = output.faceMaterials.size();
        if (faceCount == 0 || emitted < 4
            || output.sourceFaceOrdinals.size() != emitted
            || output.indices.size() != 3 * emitted
            || output.positionsMM.size() != 9 * emitted
            || output.normals.size() != 9 * emitted
            || output.uvs.size() != 6 * emitted
            || output.contributionTokens.size() != emitted)
            return false;
        std::vector<v1::Point> canonical = input.positionsMM;
        for (v1::Point& p : canonical) {
            if (!v1::Finite(p)) return false;
            if (p.x == 0) p.x = 0; if (p.y == 0) p.y = 0; if (p.z == 0) p.z = 0;
        }
        std::sort(canonical.begin(), canonical.end());
        canonical.erase(std::unique(canonical.begin(), canonical.end()), canonical.end());
        std::vector<std::array<std::size_t, 3>> faces;
        std::vector<std::array<std::size_t, 3>> stored;
        faces.reserve(faceCount);
        stored.reserve(faceCount);
        for (std::size_t f = 0; f < faceCount; ++f) {
            std::array<std::size_t, 3> welded{}, storedTriple{};
            for (std::size_t c = 0; c < 3; ++c) {
                const std::uint32_t source = input.indices[3 * f + c];
                if (source >= input.positionsMM.size()) return false;
                storedTriple[c] = source;
                const v1::Point p = input.positionsMM[source];
                auto found = std::lower_bound(canonical.begin(), canonical.end(), p);
                if (found == canonical.end() || !(*found == p)) return false;
                welded[c] = static_cast<std::size_t>(found - canonical.begin());
            }
            faces.push_back(welded);
            stored.push_back(storedTriple);
        }
        std::map<v1::detail::Edge, std::vector<std::size_t>> incidents;
        for (std::size_t f = 0; f < faceCount; ++f) {
            incidents[v1::detail::MakeEdge(faces[f][0], faces[f][1])].push_back(f);
            incidents[v1::detail::MakeEdge(faces[f][1], faces[f][2])].push_back(f);
            incidents[v1::detail::MakeEdge(faces[f][2], faces[f][0])].push_back(f);
        }
        auto faceNormal = [&](std::size_t f) {
            const v1::Point& a = canonical[faces[f][0]];
            return v1::Cross(canonical[faces[f][1]] - a, canonical[faces[f][2]] - a);
        };
        std::set<v1::detail::Edge> protectedEdges;
        const double cosineThreshold = std::cos(45.0 * std::acos(-1.0) / 180.0);
        for (const auto& item : incidents) {
            if (item.second.size() != 2) return false;
            const std::size_t first = item.second[0], second = item.second[1];
            const v1::Point a = faceNormal(first), b = faceNormal(second);
            const double divisor = std::sqrt(v1::Dot(a, a) * v1::Dot(b, b));
            if (!(std::isfinite(divisor) && divisor > 0)) return false;
            const double cosine = v1::Dot(a, b) / divisor;
            if (!std::isfinite(cosine)) return false;
            bool attributeBoundary = input.faceMaterials[first] != input.faceMaterials[second];
            for (std::size_t corner = 0; corner < 3 && !attributeBoundary; ++corner) {
                const std::size_t welded = faces[first][corner];
                if (welded != item.first.first && welded != item.first.second) continue;
                const auto other = std::find(faces[second].begin(), faces[second].end(), welded);
                if (other == faces[second].end()) return false;
                const std::size_t otherCorner =
                    static_cast<std::size_t>(other - faces[second].begin());
                if (!(input.wedges[stored[first][corner]] == input.wedges[stored[second][otherCorner]]))
                    attributeBoundary = true;
            }
            if (cosine <= cosineThreshold || attributeBoundary) protectedEdges.insert(item.first);
        }
        std::set<std::size_t> lockedVertices;
        for (const auto& edge : protectedEdges) {
            lockedVertices.insert(edge.first);
            lockedVertices.insert(edge.second);
        }
        auto encode = [](double value) {
            const float encoded = static_cast<float>(value);
            return encoded == 0 ? 0.0f : encoded;
        };
        std::map<std::array<float, 3>, std::size_t> rankByPosition;
        for (std::size_t rank = 0; rank < canonical.size(); ++rank) {
            const std::array<float, 3> key{encode(canonical[rank].x),
                encode(canonical[rank].y), encode(canonical[rank].z)};
            if (!rankByPosition.emplace(key, rank).second) return false;
        }
        // Replay the emitted collapse sequence: every accepted collapse must
        // be an unprotected current edge with an unlocked removed endpoint,
        // must kill exactly its two incident faces and must carry one
        // identical surviving wedge into every rewritten slot.
        if (output.collapseKeepRemove.size() != output.acceptedCollapses) return false;
        std::vector<std::array<std::size_t, 3>> replayFaces = faces;
        std::vector<std::array<attr::Wedge, 3>> replayWedges(faceCount);
        for (std::size_t f = 0; f < faceCount; ++f)
            for (std::size_t c = 0; c < 3; ++c)
                replayWedges[f][c] = input.wedges[stored[f][c]];
        std::vector<bool> alive(faceCount, true);
        std::size_t aliveCount = faceCount;
        for (const auto& collapse : output.collapseKeepRemove) {
            const std::size_t keep = collapse.first, remove = collapse.second;
            if (keep >= canonical.size() || remove >= canonical.size() || keep == remove)
                return false;
            if (protectedEdges.count(v1::detail::MakeEdge(keep, remove))) return false;
            if (lockedVertices.count(remove)) return false;
            std::vector<std::pair<std::size_t, std::size_t>> deadKeepSlots;
            std::vector<std::pair<std::size_t, std::size_t>> rewrites;
            for (std::size_t f = 0; f < faceCount; ++f) {
                if (!alive[f]) continue;
                bool hasKeep = false, hasRemove = false;
                std::size_t keepSlot = 0;
                for (std::size_t c = 0; c < 3; ++c) {
                    if (replayFaces[f][c] == keep) { hasKeep = true; keepSlot = c; }
                    if (replayFaces[f][c] == remove) hasRemove = true;
                }
                if (hasKeep && hasRemove) {
                    deadKeepSlots.push_back({f, keepSlot});
                } else if (hasRemove) {
                    for (std::size_t c = 0; c < 3; ++c)
                        if (replayFaces[f][c] == remove) rewrites.push_back({f, c});
                }
            }
            if (deadKeepSlots.size() != 2) return false;
            const attr::Wedge carried =
                replayWedges[deadKeepSlots[0].first][deadKeepSlots[0].second];
            if (!(carried == replayWedges[deadKeepSlots[1].first][deadKeepSlots[1].second]))
                return false;
            for (const auto& dead : deadKeepSlots) { alive[dead.first] = false; --aliveCount; }
            for (const auto& slot : rewrites) {
                replayFaces[slot.first][slot.second] = keep;
                replayWedges[slot.first][slot.second] = carried;
            }
        }
        if (aliveCount != emitted) return false;
        for (std::size_t t = 0; t < emitted; ++t) {
            const std::uint32_t ordinal = output.sourceFaceOrdinals[t];
            if (ordinal >= faceCount || !alive[ordinal]) return false;
            if (output.faceMaterials[t] != input.faceMaterials[ordinal]) return false;
            if (output.contributionTokens[t].empty()) return false;
            for (std::size_t c = 0; c < 3; ++c) {
                const std::size_t corner = 3 * t + c;
                if (output.indices[corner] != corner) return false;
                const std::array<float, 3> key{output.positionsMM[3 * corner],
                    output.positionsMM[3 * corner + 1], output.positionsMM[3 * corner + 2]};
                const auto rank = rankByPosition.find(key);
                if (rank == rankByPosition.end()) return false;
                if (replayFaces[ordinal][c] != rank->second) return false;
                const attr::Wedge& wedge = replayWedges[ordinal][c];
                if (output.normals[3 * corner] != encode(wedge.nx)
                    || output.normals[3 * corner + 1] != encode(wedge.ny)
                    || output.normals[3 * corner + 2] != encode(wedge.nz)
                    || output.uvs[2 * corner] != encode(wedge.u)
                    || output.uvs[2 * corner + 1] != encode(wedge.v))
                    return false;
            }
        }
        return true;
    } catch (...) { return false; }
}

} // namespace

@interface Core3DDecimationOpening () {
    std::shared_ptr<std::atomic_bool> _stop;
    BOOL _asyncInFlight;
}
@property(nonatomic, strong, nullable) Core3DPublishCapture *capture;
- (void)f2ContinueAtAsyncBoundary:(NSUInteger)boundary
                     continuation:(dispatch_block_t)continuation;
@end

@implementation Core3DDecimationOpening
+ (instancetype)openingWithCapture:(Core3DPublishCapture *)capture error:(NSError **)error {
    if (capture && !NSThread.isMainThread) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 capture currentness is main-thread only."); return nil;
    }
    if (capture && ![capture isCurrent:error]) return nil;
    Core3DDecimationOpening *opening = [[self alloc] initPrivate];
    opening.capture = capture;
    return opening;
}
- (instancetype)initPrivate {
    if ((self = [super init])) {
        _stop = std::make_shared<std::atomic_bool>(false);
        _asyncInFlight = NO;
    }
    return self;
}
- (BOOL)cancel { return !_stop->exchange(true, std::memory_order_relaxed); }

- (void)f2ContinueAtAsyncBoundary:(NSUInteger)boundary
                     continuation:(dispatch_block_t)continuation {
    NSAssert(NSThread.isMainThread, @"F2 async boundaries are main-thread only");
#if DEBUG
    void (^handler)(Core3DDecimationAsyncBoundary, dispatch_block_t) =
        self.f2DebugAsyncBoundaryHandler;
    if (handler) {
        handler((Core3DDecimationAsyncBoundary)boundary, [continuation copy]);
        return;
    }
#else
    (void)boundary;
#endif
    continuation();
}

- (Core3DDecimationResult *)decimateRequest:(Core3DDecimationRequest *)request
                                       error:(NSError **)error {
    if (self.capture && !NSThread.isMainThread) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 captured execution is main-thread only."); return nil;
    }
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
    if (request.f2AttributeWedgeNormals || request.f2AttributeWedgeUVs
        || request.f2AttributeFaceMaterials || request.f2AttributeFaceAnchors)
        return [self f2DecimateAttributeRequest:request error:error];
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
    Core3DDecimationResult *result = [[Core3DDecimationResult alloc] initWithNative:candidate
        materialBytes:request.materialBytes];
    // Attach the nonserializable producer seal only when this capture-bound
    // opening's capture carried bound artifact evidence through the mesh work.
    Core3DPublishArtifactEvidence *evidence = self.capture.artifactEvidence;
    if (self.capture && evidence) {
        std::vector<std::uint8_t> canonical;
        if (F2CanonicalRequestBytes(request, canonical))
            result.f2ProducerSeal = [[Core3DF2ProducerSeal alloc] initWithCapture:self.capture
                evidence:evidence canonicalRequestBytes:F2Data(canonical)];
    }
    return result;
}

- (void)decimateCapturedRequest:(Core3DDecimationRequest *)request
                     completion:(void (^)(Core3DDecimationResult *, NSError *))completion {
    if (!completion) return;
    void (^finish)(Core3DDecimationResult *, NSError *) = [completion copy];
    if (!NSThread.isMainThread) {
        dispatch_async(dispatch_get_main_queue(), ^{
            finish(nil, [NSError errorWithDomain:Core3DDecimationErrorDomain
                code:Core3DDecimationErrorMalformedRequest
                userInfo:@{NSLocalizedDescriptionKey:
                    @"F2 captured execution must start on main."}]);
        });
        return;
    }
    Core3DPublishCapture *capture = self.capture;
    Core3DPublishArtifactEvidence *evidence = capture.artifactEvidence;
    if (!capture || !evidence || _asyncInFlight
        || ![request isKindOfClass:Core3DDecimationRequest.class]
        || request.positionsMM.count == 0 || request.positionsMM.count % 3 != 0
        || request.indices.count == 0 || request.indices.count % 3 != 0
        || request.sourceContributionTokens.count != request.indices.count / 3
        || !LowerHexDigest(request.sourceArtifactSHA256)
        || !LowerHexDigest(request.settingsSHA256)
        || (request.metersPerUnit != 0.001 && request.metersPerUnit != 1.0)
        || request.f2AttributeWedgeNormals || request.f2AttributeWedgeUVs
        || request.f2AttributeFaceMaterials || request.f2AttributeFaceAnchors) {
        finish(nil, [NSError errorWithDomain:Core3DDecimationErrorDomain
            code:(capture && !evidence ? Core3DDecimationErrorMissingArtifactEvidence
                                       : Core3DDecimationErrorMalformedRequest)
            userInfo:@{NSLocalizedDescriptionKey:
                @"Malformed or unbound F2 captured request."}]);
        return;
    }
    NSError *fenceError = nil;
    if (![capture isCurrent:&fenceError]) {
        finish(nil, fenceError ?: [NSError errorWithDomain:Core3DDecimationErrorDomain
            code:Core3DDecimationErrorStaleSource
            userInfo:@{NSLocalizedDescriptionKey:@"F2 source capture is stale."}]);
        return;
    }
    std::vector<std::uint8_t> canonicalRequest;
    if (!F2CanonicalRequestBytes(request, canonicalRequest)) {
        finish(nil, [NSError errorWithDomain:Core3DDecimationErrorDomain
            code:Core3DDecimationErrorMalformedRequest
            userInfo:@{NSLocalizedDescriptionKey:@"Malformed F2 canonical request."}]);
        return;
    }
    auto native = std::make_shared<core3d::export_mesh_decimator::Input>();
    native->targetTriangles = request.targetTriangleCount;
    native->positionsMM.reserve(request.positionsMM.count / 3);
    for (NSUInteger index = 0; index < request.positionsMM.count; index += 3) {
        native->positionsMM.push_back({request.positionsMM[index].doubleValue,
            request.positionsMM[index + 1].doubleValue,
            request.positionsMM[index + 2].doubleValue});
    }
    native->indices.reserve(request.indices.count);
    for (NSNumber *number in request.indices) {
        const unsigned long long value = number.unsignedLongLongValue;
        if (value > std::numeric_limits<std::uint32_t>::max()) {
            finish(nil, [NSError errorWithDomain:Core3DDecimationErrorDomain
                code:Core3DDecimationErrorMalformedRequest
                userInfo:@{NSLocalizedDescriptionKey:@"Index is out of range."}]);
            return;
        }
        native->indices.push_back(static_cast<std::uint32_t>(value));
    }
    for (NSString *token in request.sourceContributionTokens) {
        if (![token isKindOfClass:NSString.class] || token.length == 0) {
            finish(nil, [NSError errorWithDomain:Core3DDecimationErrorDomain
                code:Core3DDecimationErrorMalformedRequest
                userInfo:@{NSLocalizedDescriptionKey:@"Missing provenance token."}]);
            return;
        }
        native->contributionTokens.emplace_back(token.UTF8String ?: "");
    }
    NSData *materialBytes = [request.materialBytes copy];
    NSData *canonicalBytes = F2Data(canonicalRequest);
    std::shared_ptr<std::atomic_bool> stop = _stop;
    auto output = std::make_shared<core3d::export_mesh_decimator::Output>();
    auto status = std::make_shared<NativeError>(NativeError::None);
    _asyncInFlight = YES;
    __weak Core3DDecimationOpening *weakSelf = self;
    [self f2ContinueAtAsyncBoundary:1 continuation:^{
        dispatch_queue_attr_t attributes = dispatch_queue_attr_make_with_qos_class(
            DISPATCH_QUEUE_SERIAL, QOS_CLASS_USER_INITIATED, 0);
        dispatch_queue_t queue = dispatch_queue_create(
            "com.shapeyard.f2-decimation.request", attributes);
        dispatch_async(queue, ^{
            *status = core3d::export_mesh_decimator::Decimate(*native, *output, stop.get());
            dispatch_async(dispatch_get_main_queue(), ^{
                Core3DDecimationOpening *opening = weakSelf;
                if (!opening) {
                    finish(nil, [NSError errorWithDomain:Core3DDecimationErrorDomain
                        code:Core3DDecimationErrorCancelled
                        userInfo:@{NSLocalizedDescriptionKey:@"F2 request owner was released."}]);
                    return;
                }
                [opening f2ContinueAtAsyncBoundary:2 continuation:^{
                    [opening f2ContinueAtAsyncBoundary:3 continuation:^{
                        opening->_asyncInFlight = NO;
                        if (stop->load(std::memory_order_relaxed)
                            || *status == NativeError::Cancelled) {
                            finish(nil, [NSError errorWithDomain:Core3DDecimationErrorDomain
                                code:Core3DDecimationErrorCancelled
                                userInfo:@{NSLocalizedDescriptionKey:
                                    @"F2 request was cancelled and drained."}]);
                            return;
                        }
                        if (*status != NativeError::None) {
                            finish(nil, [NSError errorWithDomain:Core3DDecimationErrorDomain
                                code:PublicError(*status)
                                userInfo:@{NSLocalizedDescriptionKey:
                                    @"F2 native decimation refused without output."}]);
                            return;
                        }
                        NSError *finalError = nil;
                        if (![capture isCurrent:&finalError]) {
                            finish(nil, finalError ?: [NSError errorWithDomain:
                                Core3DDecimationErrorDomain code:Core3DDecimationErrorStaleSource
                                userInfo:@{NSLocalizedDescriptionKey:
                                    @"F2 source changed during work."}]);
                            return;
                        }
                        Core3DPublishArtifactEvidence *currentEvidence = capture.artifactEvidence;
                        if (!F2EvidenceEqual(evidence, currentEvidence)) {
                            finish(nil, [NSError errorWithDomain:Core3DDecimationErrorDomain
                                code:Core3DDecimationErrorMissingArtifactEvidence
                                userInfo:@{NSLocalizedDescriptionKey:
                                    @"F2 artifact evidence changed during work."}]);
                            return;
                        }
                        Core3DDecimationResult *result =
                            [[Core3DDecimationResult alloc] initWithNative:*output
                                materialBytes:materialBytes];
                        result.f2ProducerSeal = [[Core3DF2ProducerSeal alloc]
                            initWithCapture:capture evidence:evidence
                            canonicalRequestBytes:canonicalBytes];
                        finish(result, nil);
                    }];
                }];
            });
        });
    }];
}

//! F2-S3 attribute-aware path. Same ownership, fences and Stop stickiness as
//! the flat path; the attribute payload is validated all-or-none, the kernel
//! runs with the v1 work/Stop accounting, and the candidate is admitted only
//! after the independent domain re-derivation proves every carried wedge and
//! every per-triangle material/anchor. No producer seal is attached:
//! attribute derivatives are not persistable in this slice (the F2DR record
//! and its codec are unchanged), so a failed or unprovable attempt can never
//! leave a partial artifact or a Current claim behind.
- (Core3DDecimationResult *)f2DecimateAttributeRequest:(Core3DDecimationRequest *)request
                                                 error:(NSError **)error {
    namespace attr = core3d::export_mesh_attribute_decimator;
    NSArray<NSNumber *> *normals = request.f2AttributeWedgeNormals;
    NSArray<NSNumber *> *uvs = request.f2AttributeWedgeUVs;
    NSArray<NSNumber *> *materials = request.f2AttributeFaceMaterials;
    NSArray<NSString *> *anchors = request.f2AttributeFaceAnchors;
    const NSUInteger storedCount = request.positionsMM.count / 3;
    const NSUInteger faceCount = request.indices.count / 3;
    if (![normals isKindOfClass:NSArray.class] || ![uvs isKindOfClass:NSArray.class]
        || ![materials isKindOfClass:NSArray.class] || ![anchors isKindOfClass:NSArray.class]
        || normals.count != request.positionsMM.count
        || uvs.count != 2 * storedCount
        || materials.count != faceCount
        || anchors.count != request.sourceContributionTokens.count) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 attribute payload is incomplete or misaligned.");
        return nil;
    }
    for (NSString *anchor in anchors) {
        if (![anchor isKindOfClass:NSString.class] || !LowerHexDigest(anchor)) {
            SetError(error, Core3DDecimationErrorMissingArtifactEvidence,
                @"F2 attribute provenance anchor is not a 64-lower-hex digest.");
            return nil;
        }
    }
    attr::Input native;
    native.targetTriangles = request.targetTriangleCount;
    @try {
        native.positionsMM.reserve(storedCount);
        for (NSUInteger i = 0; i < request.positionsMM.count; i += 3)
            native.positionsMM.push_back({request.positionsMM[i].doubleValue,
                request.positionsMM[i + 1].doubleValue, request.positionsMM[i + 2].doubleValue});
        native.indices.reserve(request.indices.count);
        for (NSNumber *number in request.indices) {
            const unsigned long long value = number.unsignedLongLongValue;
            if (value > std::numeric_limits<std::uint32_t>::max()) {
                SetError(error, Core3DDecimationErrorMalformedRequest, @"Index is out of range.");
                return nil;
            }
            native.indices.push_back(static_cast<std::uint32_t>(value));
        }
        native.wedges.reserve(storedCount);
        for (NSUInteger i = 0; i < storedCount; ++i) {
            attr::Wedge wedge;
            wedge.nx = normals[3 * i].doubleValue;
            wedge.ny = normals[3 * i + 1].doubleValue;
            wedge.nz = normals[3 * i + 2].doubleValue;
            wedge.u = uvs[2 * i].doubleValue;
            wedge.v = uvs[2 * i + 1].doubleValue;
            native.wedges.push_back(wedge);
        }
        native.faceMaterials.reserve(faceCount);
        for (NSNumber *number in materials) {
            const unsigned long long value = number.unsignedLongLongValue;
            if (value > std::numeric_limits<std::uint32_t>::max()) {
                SetError(error, Core3DDecimationErrorMalformedRequest,
                    @"F2 attribute material id is out of range.");
                return nil;
            }
            native.faceMaterials.push_back(static_cast<std::uint32_t>(value));
        }
        for (NSString *token in request.sourceContributionTokens)
            native.contributionTokens.emplace_back(token.UTF8String ?: "");
        for (NSString *anchor in anchors)
            native.sourceFaceAnchors.emplace_back(anchor.UTF8String ?: "");
    } @catch (__unused NSException *exception) {
        SetError(error, Core3DDecimationErrorMalformedRequest, @"Malformed F2 attribute request.");
        return nil;
    }
    attr::Output candidate;
    const AttributeNativeError status = attr::Decimate(native, candidate, _stop.get());
    if (status != AttributeNativeError::None) {
        SetError(error, PublicAttributeError(status),
            @"F2 attribute decimation refused without output.");
        return nil;
    }
    if (_stop->load(std::memory_order_relaxed)) {
        SetError(error, Core3DDecimationErrorCancelled, @"F2 request was cancelled.");
        return nil;
    }
    if (!F2VerifyAttributeOutput(native, candidate)) {
        SetError(error, Core3DDecimationErrorMissingArtifactEvidence,
            @"F2 attribute output failed independent domain verification.");
        return nil;
    }
    if (self.capture && ![self.capture isCurrent:error]) {
        SetError(error, Core3DDecimationErrorStaleSource, @"F2 source changed during work.");
        return nil;
    }
    Core3DDecimationResult *result = [[Core3DDecimationResult alloc]
        initWithAttributeNative:candidate materialBytes:request.materialBytes];
    @try {
        NSMutableArray *outAnchors =
            [NSMutableArray arrayWithCapacity:candidate.sourceFaceOrdinals.size()];
        for (const std::uint32_t ordinal : candidate.sourceFaceOrdinals)
            [outAnchors addObject:anchors[ordinal]];
        result.f2AttributeSourceFaceAnchors = [outAnchors copy];
        // Per-triangle content digests recomputed from the actual emitted
        // Float32 position bytes in emitted order (the F2-S2 rule); split
        // corners are sequential, so each triangle is 36 consecutive bytes.
        NSData *positions = result.positionFloat32Data;
        if (positions.length != result.triangleCount * 36) {
            SetError(error, Core3DDecimationErrorMissingArtifactEvidence,
                @"F2 attribute output byte layout is inconsistent.");
            return nil;
        }
        const auto *positionBytes = static_cast<const std::uint8_t *>(positions.bytes);
        NSMutableArray *contents = [NSMutableArray arrayWithCapacity:result.triangleCount];
        for (std::size_t t = 0; t < result.triangleCount; ++t) {
            const f2::F2Digest digest = f2::OutputTriangleContentSHA256(positionBytes + 36 * t);
            [contents addObject:F2String(f2::HexFromDigest(digest))];
        }
        result.f2AttributeTriangleContentSHA256 = [contents copy];
    } @catch (__unused NSException *exception) {
        SetError(error, Core3DDecimationErrorMissingArtifactEvidence,
            @"F2 attribute receipt assembly failed.");
        return nil;
    }
    return result;
}

+ (NSData *)f2CanonicalSettingsBytesWithTargetTriangleCount:(NSUInteger)targetTriangleCount
                                maximumSymmetricSurfaceDistanceMM:(double)maximumSymmetricSurfaceDistanceMM
                                  maximumSilhouettePixelDeviation:(double)maximumSilhouettePixelDeviation
                                               hardEdgeAngleDegrees:(double)hardEdgeAngleDegrees
                                                contactClearanceMM:(double)contactClearanceMM
                                           measurementSamplesPerMesh:(NSUInteger)measurementSamplesPerMesh
                                                silhouetteResolution:(NSUInteger)silhouetteResolution
                                          silhouetteCameraSetSHA256:(NSString *)silhouetteCameraSetSHA256
                                                      metersPerUnit:(double)metersPerUnit {
    if (targetTriangleCount > std::numeric_limits<std::uint32_t>::max()
        || measurementSamplesPerMesh > std::numeric_limits<std::uint32_t>::max()
        || silhouetteResolution > std::numeric_limits<std::uint32_t>::max()) return nil;
    // Reject non-finite and negative-zero F64 inputs rather than admitting a
    // second representation the canonical decode would refuse.
    const double values[5] = {maximumSymmetricSurfaceDistanceMM,
        maximumSilhouettePixelDeviation, hardEdgeAngleDegrees, contactClearanceMM,
        metersPerUnit};
    for (const double value : values)
        if (!std::isfinite(value) || (value == 0.0 && std::signbit(value))) return nil;
    f2::CanonicalSettings settings;
    settings.targetTriangleCount = static_cast<std::uint32_t>(targetTriangleCount);
    settings.maximumSymmetricSurfaceDistanceMM = maximumSymmetricSurfaceDistanceMM;
    settings.maximumSilhouettePixelDeviation = maximumSilhouettePixelDeviation;
    settings.hardEdgeAngleDegrees = hardEdgeAngleDegrees;
    settings.contactClearanceMM = contactClearanceMM;
    settings.measurementSamplesPerMesh = static_cast<std::uint32_t>(measurementSamplesPerMesh);
    settings.silhouetteResolution = static_cast<std::uint32_t>(silhouetteResolution);
    if (!f2::DigestFromHex(silhouetteCameraSetSHA256.UTF8String ?: "",
            settings.silhouetteCameraSetSHA256)) return nil;
    settings.metersPerUnit = metersPerUnit;
    std::vector<std::uint8_t> bytes;
    if (!f2::EncodeSettings(settings, bytes)) return nil;
    return F2Data(bytes);
}

+ (NSString *)f2SettingsSHA256ForCanonicalBytes:(NSData *)bytes {
    F2Digest digest;
    if (!f2::SettingsSHA256(F2Bytes(bytes), digest)) return nil;
    return F2String(f2::HexFromDigest(digest));
}

+ (Core3DDecimationSettingsReadback *)f2SettingsReadbackForCanonicalBytes:(NSData *)bytes
    error:(NSError **)error {
    if (error) *error = nil;
    if (![bytes isKindOfClass:NSData.class]) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 canonical settings bytes are unavailable.");
        return nil;
    }
    f2::CanonicalSettings settings;
    F2Digest digest;
    if (!f2::DecodeSettings(F2Bytes(bytes), settings)
        || !f2::SettingsSHA256(F2Bytes(bytes), digest)) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 canonical settings bytes are malformed.");
        return nil;
    }
    return [[Core3DDecimationSettingsReadback alloc] initWithSettings:settings
        canonicalBytes:bytes settingsDigest:digest];
}

- (Core3DPersistedDecimationDerivative *)f2PersistDerivativeForController:(Core3DViewController *)controller
    request:(Core3DDecimationRequest *)request result:(Core3DDecimationResult *)result
    settingsCanonicalBytes:(NSData *)settingsCanonicalBytes error:(NSError **)error {
    if (error) *error = nil;
    Core3DPublishCapture *capture = self.capture;
    if (!NSThread.isMainThread) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 derivative persistence is main-thread only.");
        return nil;
    }
    if (!capture || ![request isKindOfClass:Core3DDecimationRequest.class]
        || ![result isKindOfClass:Core3DDecimationResult.class]
        || ![settingsCanonicalBytes isKindOfClass:NSData.class]) {
        SetError(error, Core3DDecimationErrorMissingArtifactEvidence,
            @"F2 persist requires a capture-bound opening with the sealed producer request and result.");
        return nil;
    }
    const NSInteger borrowedStageLevel = F2BorrowedStageLevel;
    const bool borrowedStage = borrowedStageLevel >= 0 && borrowedStageLevel <= 7;
    if (!borrowedStage && ![capture isCurrent:error]) {
        SetError(error, Core3DDecimationErrorStaleSource, @"F2 source capture is stale.");
        return nil;
    }
    const Handle(OcctDocument) document = F2DocumentForController(controller);
    if (document.IsNull()
        || document->DocumentIdentifier() != (capture.documentIdentifier.UTF8String ?: "")) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 derivative persistence is unavailable for this controller.");
        return nil;
    }
    // Effective evidence: the capture's bound evidence, or the sealed evidence
    // of the same source epoch across the F2 metadata write itself.
    Core3DF2ProducerSeal *seal = result.f2ProducerSeal;
    Core3DPublishArtifactEvidence *evidence = capture.artifactEvidence;
    if (!evidence && seal
        && [seal.capture.documentIdentifier isEqualToString:capture.documentIdentifier]
        && [seal.capture.sourceWitnessBytes isEqualToData:capture.sourceWitnessBytes])
        evidence = seal.evidence;
    if (!seal || !evidence) {
        SetError(error, Core3DDecimationErrorMissingArtifactEvidence,
            @"F2 result carries no producer seal with artifact evidence.");
        return nil;
    }
    std::vector<std::uint8_t> canonicalRequest;
    if (!F2CanonicalRequestBytes(request, canonicalRequest)
        || ![seal.canonicalRequestBytes isEqualToData:F2Data(canonicalRequest)]
        || !F2EvidenceEqual(seal.evidence, evidence)) {
        SetError(error, Core3DDecimationErrorMissingArtifactEvidence,
            @"F2 request is not the sealed producer input.");
        return nil;
    }
    // Complete equality of the request with the observed artifact.
    if (![request.sourceArtifactSHA256 isEqualToString:evidence.artifactSHA256]
        || !F2NumberArraysEqual(request.positionsMM, evidence.sourcePositionsMM)
        || !F2NumberArraysEqual(request.indices, evidence.sourceIndices)
        || request.metersPerUnit != capture.metersPerUnit) {
        SetError(error, Core3DDecimationErrorMissingArtifactEvidence,
            @"F2 request does not equal the observed artifact.");
        return nil;
    }
    // Settings: canonical, digest-matched to the request, and exactly the
    // ratified F2 quality profile.
    const std::vector<std::uint8_t> settingsBytes = F2Bytes(settingsCanonicalBytes);
    f2::CanonicalSettings settings;
    F2Digest settingsDigest;
    if (!f2::DecodeSettings(settingsBytes, settings)
        || !f2::SettingsSHA256(settingsBytes, settingsDigest)
        || f2::HexFromDigest(settingsDigest) != (request.settingsSHA256.UTF8String ?: "")
        || !f2::IsRatifiedProfile(settings)
        || settings.targetTriangleCount != request.targetTriangleCount
        || settings.metersPerUnit != request.metersPerUnit) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 settings bytes do not match the ratified request profile.");
        return nil;
    }
    // Result consistency and output commitments recomputed from actual bytes.
    const std::vector<std::uint8_t> resultPositions = F2Bytes(result.positionFloat32Data);
    const std::vector<std::uint8_t> resultNormals = F2Bytes(result.normalFloat32Data);
    const std::vector<std::uint8_t> resultIndices = F2Bytes(result.indexUInt32Data);
    const std::vector<std::uint8_t> resultMaterial = F2Bytes(result.materialBytes);
    F2Digest outputArtifact, outputGeometry;
    if (result.triangleCount < 4 || result.triangleCount > f2::kMaxRecordTriangles
        || result.indexUInt32Data.length != result.triangleCount * 12
        || result.sourceContributionTokens.count != result.triangleCount
        || result.materialBytes.length == 0
        || ![result.materialBytes isEqualToData:request.materialBytes]
        || ![result.normalPolicy isEqualToString:@"f2-normal-regen.v1"]
        || !f2::OutputArtifactSHA256(resultPositions, resultNormals, resultIndices,
            resultMaterial, "f2-normal-regen.v1", outputArtifact)
        || !f2::OutputGeometrySHA256(resultPositions, resultIndices, outputGeometry)) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 result is inconsistent with the request.");
        return nil;
    }
    // Source commitments from the evidence and the capture witness.
    std::vector<double> positions;
    std::vector<std::uint32_t> indices;
    std::vector<F2Digest> faceDigests;
    F2Digest sourceArtifact, sourceGeometry, sourceFaceEvidence, sourceRevision, measurement;
    if (!F2RequestGeometry(request, positions, indices)
        || indices.size() / 3 != evidence.addressedTriangleCount
        || !f2::DigestFromHex(evidence.artifactSHA256.UTF8String ?: "", sourceArtifact)
        || !f2::SourceGeometrySHA256(positions, indices, sourceGeometry)
        || !F2FaceEvidenceDigests(evidence, faceDigests)
        || !f2::SourceFaceEvidenceSHA256(faceDigests, sourceFaceEvidence)
        || !f2::SourceRevisionSHA256(F2Bytes(capture.sourceWitnessBytes), sourceRevision)
        || !f2::DigestFromHex(evidence.measurementReceiptSHA256.UTF8String ?: "", measurement)) {
        SetError(error, Core3DDecimationErrorMissingArtifactEvidence,
            @"F2 source evidence commitments refused.");
        return nil;
    }
    // Remap receipt over the actual emitted bytes and decoded token digests.
    std::vector<f2::RemapGroup> groups;
    F2Digest remap;
    std::uint64_t references = 0;
    if (!F2RemapGroups(result, groups)) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 remap groups are malformed.");
        return nil;
    }
    for (const auto& group : groups) references += group.sortedUniqueTokens.size();
    if (references > f2::kMaxContributionReferences
        || !f2::RemapReceiptSHA256(sourceArtifact, sourceGeometry, sourceFaceEvidence,
            outputGeometry, static_cast<std::uint32_t>(result.triangleCount), groups, remap)) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 remap receipt refused.");
        return nil;
    }
    // Durable owner identity and publication source.
    F2UUID documentUUID, entityUUID, definitionUUID, sourceFeatureUUID, terminalFeatureUUID;
    if (!F2OwnerIdentityFromCapture(capture, documentUUID, entityUUID, definitionUUID,
            sourceFeatureUUID, terminalFeatureUUID)) {
        SetError(error, Core3DDecimationErrorMissingArtifactEvidence,
            @"F2 source owner identity is unavailable.");
        return nil;
    }
    F2UUID publicationUUID;
    if (!f2::UUIDFromText(capture.publicationSourceIdentifier.UTF8String ?: "",
            publicationUUID) || !f2::Nonzero(publicationUUID)) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 publication source is not a UUID.");
        return nil;
    }
    // Read the store (typed tri-state; corrupt refuses, never recovers silently).
    f2::F2StoreRead before;
    if (!document->ReadF2DerivativeStore(before)) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 derivative store read unavailable.");
        return nil;
    }
    if (before.occurrence == f2::F2StoreOccurrence::Corrupt) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            [NSString stringWithFormat:@"F2 derivative store is corrupt: %s",
                before.refusal.c_str()]);
        return nil;
    }
    f2::F2Store store;
    store.documentUUID = documentUUID;
    if (before.occurrence == f2::F2StoreOccurrence::Present
        && (!f2::DecodeStore(before.bytes, store) || store.documentUUID != documentUUID)) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 derivative store binding mismatch.");
        return nil;
    }
    const std::uint8_t level = borrowedStage
        ? static_cast<std::uint8_t>(borrowedStageLevel) : 0;
    const F2Digest key = f2::OwnerSetKey(documentUUID, entityUUID, definitionUUID, level);
    const F2Record *existing = F2FindRow(store, key);
    // Build the candidate record: reuse the derivative identity tuple of an
    // existing row; mint one natively on first creation.
    F2Record candidate;
    candidate.ownerSetKey = key;
    candidate.level = level;
    candidate.entityUUID = entityUUID;
    candidate.definitionUUID = definitionUUID;
    candidate.sourceFeatureUUID = sourceFeatureUUID;
    candidate.terminalFeatureUUID = terminalFeatureUUID;
    if (existing) {
        candidate.derivativeEntityUUID = existing->derivativeEntityUUID;
        candidate.derivativeDefinitionUUID = existing->derivativeDefinitionUUID;
        candidate.derivativeFeatureUUID = existing->derivativeFeatureUUID;
    } else {
        F2UUID minted[3];
        if (!F2MintDerivativeUUIDs(minted)) {
            SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
                @"F2 derivative identity minting failed.");
            return nil;
        }
        candidate.derivativeEntityUUID = minted[0];
        candidate.derivativeDefinitionUUID = minted[1];
        candidate.derivativeFeatureUUID = minted[2];
    }
    candidate.derivativeIdentifier = f2::DerivativeIdentifier(documentUUID, key,
        candidate.derivativeEntityUUID, candidate.derivativeDefinitionUUID,
        candidate.derivativeFeatureUUID);
    candidate.publicationSourceUUID = publicationUUID;
    candidate.documentGeneration = capture.documentGeneration;
    candidate.modelRevision = capture.modelRevision;
    candidate.presentationRevision = capture.presentationRevision;
    candidate.sourceRevisionSHA256 = sourceRevision;
    candidate.sourceArtifactSHA256 = sourceArtifact;
    candidate.sourceGeometrySHA256 = sourceGeometry;
    candidate.sourceFaceEvidenceSHA256 = sourceFaceEvidence;
    candidate.settingsCanonicalBytes = settingsBytes;
    candidate.settingsSHA256 = settingsDigest;
    candidate.algorithmBuildSHA256 = f2::AlgorithmBuildSHA256();
    candidate.outputArtifactSHA256 = outputArtifact;
    candidate.outputGeometrySHA256 = outputGeometry;
    candidate.remapReceiptSHA256 = remap;
    candidate.independentMeasurementSHA256 = measurement;
    candidate.sourceTriangleCount = static_cast<std::uint32_t>(evidence.addressedTriangleCount);
    candidate.emittedTriangleCount = static_cast<std::uint32_t>(result.triangleCount);
    candidate.totalContributionReferences = static_cast<std::uint32_t>(references);
    candidate.stale = false;
    if (!f2::IsValid(candidate, documentUUID)) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 candidate record is invalid.");
        return nil;
    }
    // Insert or replace this owner set's row; never touch another owner's row.
    f2::F2Store candidateStore = store;
    bool replaced = false;
    for (auto& record : candidateStore.records)
        if (record.ownerSetKey == key) { record = candidate; replaced = true; break; }
    if (!replaced) candidateStore.records.push_back(candidate);
    if (candidateStore.records.size() > f2::kMaxStoreRecords) {
        SetError(error, Core3DDecimationErrorResourceLimit,
            @"F2 derivative store is full (64 records).");
        return nil;
    }
    std::sort(candidateStore.records.begin(), candidateStore.records.end(),
        [](const F2Record& a, const F2Record& b) { return a.ownerSetKey < b.ownerSetKey; });
    std::vector<std::uint8_t> candidateBytes;
    if (!f2::EncodeStore(candidateStore, candidateBytes)) {
        SetError(error, Core3DDecimationErrorResourceLimit,
            @"F2 derivative store exceeds the 65,536-byte carrier.");
        return nil;
    }
    // Same-input repeat: identical candidate bytes are a no-op with zero
    // commands and zero history delta.
    if (before.occurrence == f2::F2StoreOccurrence::Present
        && before.bytes == candidateBytes)
        return [[Core3DPersistedDecimationDerivative alloc] initWithRecord:candidate
            documentUUID:documentUUID];
    if (borrowedStage) {
        f2::F2StoreRead staged;
        if (!document->StageF2DerivativeStore(
                before, candidateBytes, Standard_False, staged)
            || staged.occurrence != f2::F2StoreOccurrence::Present
            || staged.bytes != candidateBytes) {
            SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
                @"F2 borrowed-command staging failed; the caller must abort.");
            return nil;
        }
        return [[Core3DPersistedDecimationDerivative alloc] initWithRecord:candidate
            documentUUID:documentUUID];
    }
    switch (F2WriteStoreAtomically(document, capture, before, candidateBytes)) {
        case F2WriteOutcome::Committed: break;
        case F2WriteOutcome::Stale:
            SetError(error, Core3DDecimationErrorStaleSource,
                @"F2 source changed before the write; nothing was persisted.");
            return nil;
        case F2WriteOutcome::Refused:
            SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
                @"F2 derivative write refused; the prior store is unchanged.");
            return nil;
        case F2WriteOutcome::Recovery:
            SetError(error, Core3DDecimationErrorRecoveryRequired,
                @"F2 derivative command closure is uncertain; recovery is required.");
            return nil;
    }
    return [[Core3DPersistedDecimationDerivative alloc] initWithRecord:candidate
        documentUUID:documentUUID];
}

- (Core3DPersistedDecimationDerivative *)f2StageDerivativeForController:(Core3DViewController *)controller
    level:(NSUInteger)level request:(Core3DDecimationRequest *)request
    result:(Core3DDecimationResult *)result settingsCanonicalBytes:(NSData *)settingsCanonicalBytes
    error:(NSError **)error {
    if (error) *error = nil;
    const Handle(OcctDocument) document = F2DocumentForController(controller);
    const Handle(TDocStd_Document) ocaf = document.IsNull()
        ? Handle(TDocStd_Document)() : document->Document();
    if (!NSThread.isMainThread || level > 7 || ocaf.IsNull()
        || !ocaf->HasOpenCommand() || F2BorrowedStageLevel != -1) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 borrowed staging requires level 0...7 and one caller-owned open command.");
        return nil;
    }
    F2BorrowedStageLevel = static_cast<NSInteger>(level);
    Core3DPersistedDecimationDerivative *value = nil;
    @try {
        value = [self f2PersistDerivativeForController:controller request:request
            result:result settingsCanonicalBytes:settingsCanonicalBytes error:error];
    } @finally {
        F2BorrowedStageLevel = -1;
    }
    return value;
}

- (BOOL)f2CanRestorePersistedDerivativeForController:(Core3DViewController *)controller
    settingsCanonicalBytes:(NSData *)settingsCanonicalBytes {
    try { @try {
        Core3DPublishCapture *capture = self.capture;
        if (!NSThread.isMainThread || !capture
            || ![settingsCanonicalBytes isKindOfClass:NSData.class]) return NO;
        const Handle(OcctDocument) document = F2DocumentForController(controller);
        if (document.IsNull()
            || document->DocumentIdentifier() != (capture.documentIdentifier.UTF8String ?: ""))
            return NO;
        // The witness recompute detects a changed-back document only for a
        // live capture; a stale capture never restores.
        if (![capture isCurrent:nil]) return NO;
        F2UUID documentUUID, entityUUID, definitionUUID, sourceFeatureUUID, terminalFeatureUUID;
        if (!F2OwnerIdentityFromCapture(capture, documentUUID, entityUUID, definitionUUID,
                sourceFeatureUUID, terminalFeatureUUID)) return NO;
        f2::F2StoreRead read;
        if (!document->ReadF2DerivativeStore(read)
            || read.occurrence != f2::F2StoreOccurrence::Present) return NO;
        f2::F2Store store;
        if (!f2::DecodeStore(read.bytes, store) || store.documentUUID != documentUUID)
            return NO;
        const F2Record *row = F2FindRow(store,
            f2::OwnerSetKey(documentUUID, entityUUID, definitionUUID));
        if (!row || row->stale) return NO;
        F2Digest revision, settingsDigest;
        if (!f2::SourceRevisionSHA256(F2Bytes(capture.sourceWitnessBytes), revision)
            || revision != row->sourceRevisionSHA256
            || !f2::SettingsSHA256(F2Bytes(settingsCanonicalBytes), settingsDigest)
            || settingsDigest != row->settingsSHA256) return NO;
        return YES;
    } @catch (__unused NSException *exception) { return NO; }
    } catch (...) { return NO; }
}

- (Core3DPersistedDecimationDerivative *)f2ValidatePersistedDerivativeForController:(Core3DViewController *)controller
    settingsCanonicalBytes:(NSData *)settingsCanonicalBytes error:(NSError **)error {
    if (error) *error = nil;
    Core3DPublishCapture *capture = self.capture;
    if (!NSThread.isMainThread) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 derivative persistence is main-thread only.");
        return nil;
    }
    if (!capture) {
        SetError(error, Core3DDecimationErrorMissingArtifactEvidence,
            @"F2 validate requires a capture-bound opening.");
        return nil;
    }
    if (![capture isCurrent:error]) {
        SetError(error, Core3DDecimationErrorStaleSource, @"F2 source capture is stale.");
        return nil;
    }
    const Handle(OcctDocument) document = F2DocumentForController(controller);
    if (document.IsNull()
        || document->DocumentIdentifier() != (capture.documentIdentifier.UTF8String ?: "")) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 derivative persistence is unavailable for this controller.");
        return nil;
    }
    F2UUID documentUUID, entityUUID, definitionUUID, sourceFeatureUUID, terminalFeatureUUID;
    if (!F2OwnerIdentityFromCapture(capture, documentUUID, entityUUID, definitionUUID,
            sourceFeatureUUID, terminalFeatureUUID)) {
        SetError(error, Core3DDecimationErrorMissingArtifactEvidence,
            @"F2 source owner identity is unavailable.");
        return nil;
    }
    f2::F2StoreRead before;
    if (!document->ReadF2DerivativeStore(before)
        || before.occurrence == f2::F2StoreOccurrence::Corrupt) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 derivative store read unavailable or corrupt.");
        return nil;
    }
    f2::F2Store store;
    if (before.occurrence == f2::F2StoreOccurrence::Absent
        || !f2::DecodeStore(before.bytes, store) || store.documentUUID != documentUUID) {
        SetError(error, Core3DDecimationErrorDerivativeNotPersisted,
            @"No F2 derivative record is persisted.");
        return nil;
    }
    const F2Record *row = F2FindRow(store,
        f2::OwnerSetKey(documentUUID, entityUUID, definitionUUID));
    if (!row) {
        SetError(error, Core3DDecimationErrorDerivativeNotPersisted,
            @"No F2 derivative record is persisted for this owner set.");
        return nil;
    }
    F2Digest revision, settingsDigest;
    const bool restorable = !row->stale
        && f2::SourceRevisionSHA256(F2Bytes(capture.sourceWitnessBytes), revision)
        && revision == row->sourceRevisionSHA256
        && f2::SettingsSHA256(F2Bytes(settingsCanonicalBytes), settingsDigest)
        && settingsDigest == row->settingsSHA256;
    if (restorable)
        return [[Core3DPersistedDecimationDerivative alloc] initWithRecord:*row
            documentUUID:store.documentUUID];
    if (row->stale) {
        // Stale-again is a no-op with the same typed refusal.
        SetError(error, Core3DDecimationErrorStaleSource,
            @"F2 derivative record is already stale.");
        return nil;
    }
    // Record the explicit stale transition in one command, preserving every
    // other field byte-exactly.
    F2Record replacement = *row;
    f2::MarkStale(replacement);
    f2::F2Store candidateStore = store;
    for (auto& record : candidateStore.records)
        if (record.ownerSetKey == row->ownerSetKey) { record = replacement; break; }
    std::vector<std::uint8_t> candidateBytes;
    if (!f2::EncodeStore(candidateStore, candidateBytes)
        || candidateBytes == before.bytes) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 stale transition candidate is invalid.");
        return nil;
    }
    switch (F2WriteStoreAtomically(document, capture, before, candidateBytes)) {
        case F2WriteOutcome::Committed:
            SetError(error, Core3DDecimationErrorStaleSource,
                @"F2 derivative record was marked stale.");
            return nil;
        case F2WriteOutcome::Stale:
            SetError(error, Core3DDecimationErrorStaleSource,
                @"F2 source changed before the stale transition; the record is unchanged.");
            return nil;
        case F2WriteOutcome::Refused:
            SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
                @"F2 stale transition refused; the prior store is unchanged.");
            return nil;
        case F2WriteOutcome::Recovery:
            SetError(error, Core3DDecimationErrorRecoveryRequired,
                @"F2 stale-transition closure is uncertain; recovery is required.");
            return nil;
    }
    return nil; // unreachable
}

- (Core3DPersistedDecimationDerivative *)f2PersistedDerivativeForController:(Core3DViewController *)controller
    error:(NSError **)error {
    if (error) *error = nil;
    Core3DPublishCapture *capture = self.capture;
    if (!NSThread.isMainThread) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 derivative persistence is main-thread only.");
        return nil;
    }
    const Handle(OcctDocument) document = F2DocumentForController(controller);
    if (document.IsNull()) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 derivative persistence is unavailable for this controller.");
        return nil;
    }
    // Raw historical read: no currentness fence; a stale capture still reads.
    if (!capture) {
        SetError(error, Core3DDecimationErrorDerivativeNotPersisted,
            @"No F2 derivative record is persisted.");
        return nil;
    }
    F2UUID documentUUID, entityUUID, definitionUUID, sourceFeatureUUID, terminalFeatureUUID;
    if (!F2OwnerIdentityFromCapture(capture, documentUUID, entityUUID, definitionUUID,
            sourceFeatureUUID, terminalFeatureUUID)) {
        SetError(error, Core3DDecimationErrorDerivativeNotPersisted,
            @"No F2 derivative record is persisted.");
        return nil;
    }
    f2::F2StoreRead read;
    if (!document->ReadF2DerivativeStore(read)
        || read.occurrence == f2::F2StoreOccurrence::Corrupt) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 derivative store read unavailable or corrupt.");
        return nil;
    }
    f2::F2Store store;
    if (read.occurrence == f2::F2StoreOccurrence::Absent
        || !f2::DecodeStore(read.bytes, store)) {
        SetError(error, Core3DDecimationErrorDerivativeNotPersisted,
            @"No F2 derivative record is persisted.");
        return nil;
    }
    const F2Record *row = F2FindRow(store,
        f2::OwnerSetKey(documentUUID, entityUUID, definitionUUID));
    if (!row) {
        SetError(error, Core3DDecimationErrorDerivativeNotPersisted,
            @"No F2 derivative record is persisted for this owner set.");
        return nil;
    }
    return [[Core3DPersistedDecimationDerivative alloc] initWithRecord:*row
        documentUUID:store.documentUUID];
}
#if DEBUG
+ (NSData *)f2DebugEncodeDerivativeStoreForController:(Core3DViewController *)controller
    levels:(NSArray<NSNumber *> *)levels error:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || ![levels isKindOfClass:NSArray.class]) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 store probe requires main-thread level input.");
        return nil;
    }
    const Handle(OcctDocument) document = F2DocumentForController(controller);
    if (document.IsNull()) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 store probe document is unavailable.");
        return nil;
    }
    f2::F2StoreRead read;
    if (!document->ReadF2DerivativeStore(read)
        || read.occurrence != f2::F2StoreOccurrence::Present) {
        SetError(error, Core3DDecimationErrorDerivativeNotPersisted,
            @"F2 store probe requires a persisted level-0 row.");
        return nil;
    }
    f2::F2Store stored;
    if (!f2::DecodeStore(read.bytes, stored)) {
        SetError(error, Core3DDecimationErrorDerivativePersistenceUnavailable,
            @"F2 store probe refused the persisted store.");
        return nil;
    }
    const F2Record *seed = nullptr;
    for (const F2Record& record : stored.records) {
        if (record.level != 0) continue;
        if (seed) {
            SetError(error, Core3DDecimationErrorMalformedRequest,
                @"F2 store probe requires exactly one level-0 row.");
            return nil;
        }
        seed = &record;
    }
    if (!seed) {
        SetError(error, Core3DDecimationErrorDerivativeNotPersisted,
            @"F2 store probe found no persisted level-0 row.");
        return nil;
    }

    f2::F2Store candidate;
    candidate.documentUUID = stored.documentUUID;
    std::array<bool, f2::kMaxDerivativeLevel + 1> seen{};
    for (NSNumber *number in levels) {
        if (![number isKindOfClass:NSNumber.class]) {
            SetError(error, Core3DDecimationErrorMalformedRequest,
                @"F2 store probe level is not numeric.");
            return nil;
        }
        const long long signedLevel = number.longLongValue;
        if (signedLevel < 0
            || signedLevel > static_cast<long long>(f2::kMaxDerivativeLevel)
            || number.doubleValue != static_cast<double>(signedLevel)) {
            SetError(error, Core3DDecimationErrorMalformedRequest,
                @"F2 store probe level is outside 0...7.");
            return nil;
        }
        const std::uint8_t level = static_cast<std::uint8_t>(signedLevel);
        if (seen[level]) {
            SetError(error, Core3DDecimationErrorMalformedRequest,
                @"F2 store probe duplicate (key, level) pair.");
            return nil;
        }
        seen[level] = true;
        F2Record row = *seed;
        row.level = level;
        row.ownerSetKey = f2::OwnerSetKey(candidate.documentUUID,
            row.entityUUID, row.definitionUUID, level);
        row.derivativeIdentifier = f2::DerivativeIdentifier(candidate.documentUUID,
            row.ownerSetKey, row.derivativeEntityUUID, row.derivativeDefinitionUUID,
            row.derivativeFeatureUUID);
        std::vector<std::uint8_t> encodedRecord;
        if (!f2::IsValid(row, candidate.documentUUID)
            || !f2::EncodeRecord(row, candidate.documentUUID, encodedRecord)) {
            SetError(error, Core3DDecimationErrorMalformedRequest,
                @"F2 store probe cloned row is invalid.");
            return nil;
        }
        candidate.records.push_back(std::move(row));
    }
    std::sort(candidate.records.begin(), candidate.records.end(),
        [](const F2Record& a, const F2Record& b) {
            return a.ownerSetKey < b.ownerSetKey
                || (a.ownerSetKey == b.ownerSetKey && a.level < b.level);
        });
    std::vector<std::uint8_t> encoded;
    if (!f2::EncodeStore(candidate, encoded)) {
        SetError(error, Core3DDecimationErrorResourceLimit,
            @"F2 store probe encoder refused without output.");
        return nil;
    }
    return F2Data(encoded);
}

+ (NSArray<Core3DPersistedDecimationDerivative *> *)f2DebugDecodeDerivativeStoreBytes:(NSData *)bytes
    error:(NSError **)error {
    if (error) *error = nil;
    if (![bytes isKindOfClass:NSData.class]) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 store probe bytes are unavailable.");
        return nil;
    }
    f2::F2Store decoded;
    if (!f2::DecodeStore(F2Bytes(bytes), decoded)) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 store probe decoder refused the bytes.");
        return nil;
    }
    NSMutableArray<Core3DPersistedDecimationDerivative *> *rows =
        [NSMutableArray arrayWithCapacity:decoded.records.size()];
    for (const F2Record& record : decoded.records) {
        Core3DPersistedDecimationDerivative *row =
            [[Core3DPersistedDecimationDerivative alloc] initWithRecord:record
                documentUUID:decoded.documentUUID];
        if (!row) {
            SetError(error, Core3DDecimationErrorMalformedRequest,
                @"F2 store probe could not form a decoded row.");
            return nil;
        }
        [rows addObject:row];
    }
    return [rows copy];
}

+ (NSData *)f2DebugReencodeDerivativeStoreBytes:(NSData *)bytes
    error:(NSError **)error {
    if (error) *error = nil;
    if (![bytes isKindOfClass:NSData.class]) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 store probe bytes are unavailable.");
        return nil;
    }
    f2::F2Store decoded;
    std::vector<std::uint8_t> encoded;
    if (!f2::DecodeStore(F2Bytes(bytes), decoded)
        || !f2::EncodeStore(decoded, encoded)) {
        SetError(error, Core3DDecimationErrorMalformedRequest,
            @"F2 store probe re-encode refused without output.");
        return nil;
    }
    return F2Data(encoded);
}
#endif
@end
