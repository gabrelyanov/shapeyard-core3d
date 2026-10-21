#import "Core3DDecimationOpening.h"
#import "Core3DPublishCapture.h"
#import "Core3DViewController.h"

#import "../OCCTKit/GLViewController.h"
#import "Core3DViewer.h"

#include "../OCCTKit/ExportMeshDecimationNativeOwner.hxx"
#include "../OCCTKit/ExportMeshDecimator.hxx"
#include "../OCCTKit/OcctDocument.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
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
@property(nonatomic, copy, readwrite) NSData *positionFloat32Data;
@property(nonatomic, copy, readwrite) NSData *normalFloat32Data;
@property(nonatomic, copy, readwrite) NSData *indexUInt32Data;
@property(nonatomic, copy, readwrite) NSData *materialBytes;
@property(nonatomic, copy, readwrite) NSArray<NSArray<NSString *> *> *sourceContributionTokens;
@property(nonatomic, copy, readwrite) NSString *normalPolicy;
@property(nonatomic, copy, readwrite) NSString *appearanceLimitation;
@property(nonatomic, assign, readwrite) NSUInteger triangleCount;
@property(nonatomic, assign, readwrite) NSUInteger acceptedCollapseCount;
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

} // namespace

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
    const F2Digest key = f2::OwnerSetKey(documentUUID, entityUUID, definitionUUID);
    const F2Record *existing = F2FindRow(store, key);
    // Build the candidate record: reuse the derivative identity tuple of an
    // existing row; mint one natively on first creation.
    F2Record candidate;
    candidate.ownerSetKey = key;
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
@end
