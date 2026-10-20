// F3-S1 source-bound detached opening. This translation unit deliberately has
// no OcctDocument, command-lease, persistence, manifest or F2-owner access.

#import "Core3DLODSetOpening.h"
#import "Core3DPublishCapture.h"

#include "../OCCTKit/RetainedLODSet.hxx"
#include <CommonCrypto/CommonDigest.h>
#include <atomic>
#include <limits>
#include <string>
#include <utility>

NSErrorDomain const Core3DLODSetErrorDomain = @"Core3DLODSetErrorDomain";

namespace {
namespace lod = core3d::retained_lod;
namespace decimation = core3d::export_decimation;

NSError *LODError(Core3DLODSetError code, NSString *reason) {
    return [NSError errorWithDomain:Core3DLODSetErrorDomain code:code
        userInfo:@{NSLocalizedDescriptionKey: reason ?: @"LOD set refused"}];
}

void SetError(NSError **error, Core3DLODSetError code, NSString *reason) {
    if (error) *error = LODError(code, reason);
}

std::string Text(NSString *value) {
    const char *bytes = value.UTF8String;
    return bytes ? std::string(bytes) : std::string();
}

NSString *Text(const std::string& value) {
    return [[NSString alloc] initWithBytes:value.data() length:value.size()
                                  encoding:NSUTF8StringEncoding] ?: @"";
}

NSString *Digest(NSData *data) {
    unsigned char digest[CC_SHA256_DIGEST_LENGTH];
    CC_SHA256(data.bytes, CC_LONG(data.length), digest);
    static const char hex[] = "0123456789abcdef";
    char result[CC_SHA256_DIGEST_LENGTH * 2];
    for (NSUInteger index = 0; index < CC_SHA256_DIGEST_LENGTH; ++index) {
        result[index * 2] = hex[digest[index] >> 4];
        result[index * 2 + 1] = hex[digest[index] & 15];
    }
    return [[NSString alloc] initWithBytes:result length:sizeof(result)
                                  encoding:NSASCIIStringEncoding];
}

core3d::publish::PublishSourceBinding SourceBinding(Core3DPublishCapture *capture) {
    core3d::publish::PublishSourceBinding binding;
    binding.documentIdentifier = Text(capture.documentIdentifier);
    binding.generationIdentifier = std::to_string(capture.documentGeneration);
    binding.modelRevision = std::to_string(capture.modelRevision);
    binding.selectionPolicy = core3d::publish::SourceSelectionPolicy::SelectedOutputs;
    binding.unitConvention = capture.metersPerUnit == 1.0 ? "document-m" : "document-mm";
    binding.frameConvention = Text(capture.frameConvention);
    for (Core3DPublishOwnerCommitments *record in capture.ownerCommitments) {
        core3d::publish::OwnerSourceCommitment owner;
        owner.ownerKey = Text(record.ownerIdentifier);
        owner.recipeDigest = Text(Digest(record.recipeWitnessBytes));
        owner.geometryDigest = Text(Digest(record.geometryWitnessBytes));
        owner.placementDigest = Text(Digest(record.placementWitnessBytes));
        owner.materialTextureDigest = Text(Digest(record.materialTextureWitnessBytes));
        binding.owners.push_back(std::move(owner));
    }
    return binding;
}

Core3DLODSetError PublicRefusal(lod::Refusal refusal) {
    switch (refusal) {
        case lod::Refusal::StaleSource: return Core3DLODSetErrorStaleSource;
        case lod::Refusal::AttachmentParityFailure:
            return Core3DLODSetErrorAttachmentParity;
        case lod::Refusal::BudgetExceeded: return Core3DLODSetErrorWorkBudget;
        default: return Core3DLODSetErrorInvalidDefinition;
    }
}
}

@interface Core3DLODSetHandle ()
@property(nonatomic, copy, readwrite) NSString *identifier;
- (instancetype)initPrivateWithIdentifier:(NSString *)identifier;
@end

@implementation Core3DLODSetHandle
- (instancetype)initPrivateWithIdentifier:(NSString *)identifier {
    if ((self = [super init])) _identifier = [identifier copy];
    return self;
}
@end

@interface Core3DLODLevelHandle ()
@property(nonatomic, copy, readwrite) NSString *identifier;
@property(nonatomic, assign, readwrite) NSUInteger ordinal;
- (instancetype)initPrivateWithIdentifier:(NSString *)identifier ordinal:(NSUInteger)ordinal;
@end

@implementation Core3DLODLevelHandle
- (instancetype)initPrivateWithIdentifier:(NSString *)identifier ordinal:(NSUInteger)ordinal {
    if ((self = [super init])) { _identifier = [identifier copy]; _ordinal = ordinal; }
    return self;
}
@end

@interface Core3DLODAttachmentBinding ()
@property(nonatomic, copy, readwrite) NSString *attachmentIdentifier;
@property(nonatomic, copy, readwrite) NSString *canonicalMetreFrameSHA256;
@end

@implementation Core3DLODAttachmentBinding
- (instancetype)initWithAttachmentIdentifier:(NSString *)attachmentIdentifier
                 canonicalMetreFrameSHA256:(NSString *)canonicalMetreFrameSHA256 {
    if ((self = [super init])) {
        _attachmentIdentifier = [attachmentIdentifier copy];
        _canonicalMetreFrameSHA256 = [canonicalMetreFrameSHA256 copy];
    }
    return self;
}
@end

@interface Core3DLODLevelRequest ()
@property(nonatomic, strong, readwrite) Core3DLODLevelHandle *handle;
@property(nonatomic, copy, readwrite) NSString *displayName;
@property(nonatomic, assign, readwrite) NSUInteger targetTriangleCount;
@property(nonatomic, assign, readwrite) double maximumSurfaceDistanceMM;
@property(nonatomic, assign, readwrite) double maximumSilhouetteDeviationPixels;
@property(nonatomic, assign, readwrite) double hardEdgeAngleDegrees;
@property(nonatomic, assign, readwrite) double contactClearanceMM;
@property(nonatomic, assign, readwrite) NSUInteger measurementSampleCount;
@property(nonatomic, assign, readwrite) NSUInteger silhouetteResolution;
@property(nonatomic, copy, readwrite) NSString *silhouetteCameraSetSHA256;
@property(nonatomic, copy, readwrite) NSString *settingsSHA256;
@property(nonatomic, copy, readwrite) NSArray<Core3DLODAttachmentBinding *> *attachments;
@end

@implementation Core3DLODLevelRequest
- (instancetype)initWithHandle:(Core3DLODLevelHandle *)handle
                    displayName:(NSString *)displayName
            targetTriangleCount:(NSUInteger)targetTriangleCount
       maximumSurfaceDistanceMM:(double)maximumSurfaceDistanceMM
maximumSilhouetteDeviationPixels:(double)maximumSilhouetteDeviationPixels
           hardEdgeAngleDegrees:(double)hardEdgeAngleDegrees
             contactClearanceMM:(double)contactClearanceMM
         measurementSampleCount:(NSUInteger)measurementSampleCount
           silhouetteResolution:(NSUInteger)silhouetteResolution
      silhouetteCameraSetSHA256:(NSString *)silhouetteCameraSetSHA256
                 settingsSHA256:(NSString *)settingsSHA256
                    attachments:(NSArray<Core3DLODAttachmentBinding *> *)attachments {
    if ((self = [super init])) {
        _handle = handle; _displayName = [displayName copy];
        _targetTriangleCount = targetTriangleCount;
        _maximumSurfaceDistanceMM = maximumSurfaceDistanceMM;
        _maximumSilhouetteDeviationPixels = maximumSilhouetteDeviationPixels;
        _hardEdgeAngleDegrees = hardEdgeAngleDegrees;
        _contactClearanceMM = contactClearanceMM;
        _measurementSampleCount = measurementSampleCount;
        _silhouetteResolution = silhouetteResolution;
        _silhouetteCameraSetSHA256 = [silhouetteCameraSetSHA256 copy];
        _settingsSHA256 = [settingsSHA256 copy];
        _attachments = [attachments copy];
    }
    return self;
}
@end

@interface Core3DLODSetDraft () {
@public
    lod::Definition _native;
}
@property(nonatomic, strong, readwrite) Core3DLODSetHandle *setHandle;
@property(nonatomic, copy, readwrite) NSArray<Core3DLODLevelRequest *> *levels;
@property(nonatomic, copy, readwrite) NSString *sourceDocumentIdentifier;
@property(nonatomic, copy, readwrite) NSString *boundSourceRevisionSHA256;
@property(nonatomic, copy, readwrite) NSString *observedSourceRevisionSHA256;
@property(nonatomic, copy, readwrite) NSString *attachmentMetadataSHA256;
@property(nonatomic, copy, readwrite) NSArray<Core3DLODAttachmentBinding *> *expectedAttachments;
@property(nonatomic, copy) NSString *issuerIdentifier;
- (instancetype)initPrivateWithDefinition:(lod::Definition)definition
                                 setHandle:(Core3DLODSetHandle *)setHandle
                                    levels:(NSArray<Core3DLODLevelRequest *> *)levels
                               attachments:(NSArray<Core3DLODAttachmentBinding *> *)attachments
                          issuerIdentifier:(NSString *)issuerIdentifier;
@end

@implementation Core3DLODSetDraft
- (instancetype)initPrivateWithDefinition:(lod::Definition)definition
                                 setHandle:(Core3DLODSetHandle *)setHandle
                                    levels:(NSArray<Core3DLODLevelRequest *> *)levels
                               attachments:(NSArray<Core3DLODAttachmentBinding *> *)attachments
                          issuerIdentifier:(NSString *)issuerIdentifier {
    if ((self = [super init])) {
        _native = std::move(definition); _setHandle = setHandle; _levels = [levels copy];
        _sourceDocumentIdentifier = Text(_native.source.documentIdentifier);
        _boundSourceRevisionSHA256 = Text(_native.boundSourceRevisionSHA256);
        _observedSourceRevisionSHA256 = Text(_native.observedSourceRevisionSHA256);
        _attachmentMetadataSHA256 = Text(_native.attachmentMetadataSHA256);
        _expectedAttachments = [attachments copy]; _issuerIdentifier = [issuerIdentifier copy];
    }
    return self;
}
@end

@interface Core3DLODSetOpening () {
    std::atomic_bool _stopped;
    std::atomic_size_t _completedWorkUnits;
}
@property(nonatomic, strong) Core3DPublishCapture *capture;
@property(nonatomic, strong, readwrite) Core3DLODSetHandle *setHandle;
@property(nonatomic, copy, readwrite) NSArray<Core3DLODLevelHandle *> *levelHandles;
@property(nonatomic, copy, readwrite) NSString *sourceRevisionSHA256;
@property(nonatomic, copy) NSString *issuerIdentifier;
- (instancetype)initPrivateWithCapture:(Core3DPublishCapture *)capture;
@end

@implementation Core3DLODSetOpening

+ (instancetype)openingWithCapture:(Core3DPublishCapture *)capture error:(NSError **)error {
    if (error) *error = nil;
    if (!capture) {
        SetError(error, Core3DLODSetErrorMissingProvenance,
                 @"A real Core3DPublishCapture is required.");
        return nil;
    }
    if (![capture isCurrent:error]) return nil;
    if (capture.selectedEntityIdentifiers.count == 0
        || capture.ownerCommitments.count != capture.selectedEntityIdentifiers.count
        || (capture.metersPerUnit != 0.001 && capture.metersPerUnit != 1.0)) {
        SetError(error, Core3DLODSetErrorMissingProvenance,
                 @"The capture has no complete native source binding.");
        return nil;
    }
    return [[self alloc] initPrivateWithCapture:capture];
}

- (instancetype)initPrivateWithCapture:(Core3DPublishCapture *)capture {
    if ((self = [super init])) {
        _capture = capture; _stopped.store(false); _completedWorkUnits.store(0);
        _issuerIdentifier = NSUUID.UUID.UUIDString.lowercaseString;
        _setHandle = [[Core3DLODSetHandle alloc]
            initPrivateWithIdentifier:NSUUID.UUID.UUIDString.lowercaseString];
        NSMutableArray *handles = [NSMutableArray arrayWithCapacity:lod::kMaximumLevels];
        for (NSUInteger index = 0; index < lod::kMaximumLevels; ++index)
            [handles addObject:[[Core3DLODLevelHandle alloc]
                initPrivateWithIdentifier:NSUUID.UUID.UUIDString.lowercaseString ordinal:index]];
        _levelHandles = [handles copy];
        _sourceRevisionSHA256 = Digest(capture.sourceWitnessBytes);
    }
    return self;
}

- (NSUInteger)completedWorkUnits { return _completedWorkUnits.load(); }
- (NSUInteger)workUnitLimit { return 64; }
- (BOOL)stop { return !_stopped.exchange(true); }

- (BOOL)consumeWork:(NSError **)error {
    if (_stopped.load()) {
        SetError(error, Core3DLODSetErrorStopped, @"LOD opening was stopped."); return NO;
    }
    const std::size_t completed = _completedWorkUnits.fetch_add(1) + 1;
    if (completed > self.workUnitLimit) {
        SetError(error, Core3DLODSetErrorWorkBudget, @"LOD opening work budget exhausted.");
        return NO;
    }
#if DEBUG
    if (self.debugWorkBoundaryHandler) self.debugWorkBoundaryHandler(completed);
#endif
    if (_stopped.load()) {
        SetError(error, Core3DLODSetErrorStopped, @"LOD opening was stopped."); return NO;
    }
    return YES;
}

- (BOOL)isCurrent:(NSError **)error {
    if (_stopped.load()) {
        SetError(error, Core3DLODSetErrorStopped, @"LOD opening was stopped."); return NO;
    }
    NSError *captureError = nil;
    if (![self.capture isCurrent:&captureError]) {
        SetError(error, Core3DLODSetErrorStaleSource,
                 captureError.localizedDescription ?: @"Captured source is stale.");
        return NO;
    }
    return YES;
}

- (BOOL)ownsRequest:(Core3DLODLevelRequest *)request {
    if (![request isKindOfClass:Core3DLODLevelRequest.class]
        || ![request.handle isKindOfClass:Core3DLODLevelHandle.class]
        || request.handle.ordinal >= self.levelHandles.count) return NO;
    return self.levelHandles[request.handle.ordinal] == request.handle;
}

- (BOOL)fillDefinition:(lod::Definition&)definition
                levels:(NSArray<Core3DLODLevelRequest *> *)levels
 attachmentMetadataSHA256:(NSString *)attachmentMetadataSHA256
   expectedAttachments:(NSArray<Core3DLODAttachmentBinding *> *)expectedAttachments
                 error:(NSError **)error {
    if (![levels isKindOfClass:NSArray.class]
        || ![expectedAttachments isKindOfClass:NSArray.class]) {
        SetError(error, Core3DLODSetErrorInvalidDefinition, @"Invalid LOD arrays."); return NO;
    }
    definition.lodSetID = Text(self.setHandle.identifier);
    definition.source = SourceBinding(self.capture);
    definition.boundSourceRevisionSHA256 = Text(self.sourceRevisionSHA256);
    definition.observedSourceRevisionSHA256 = definition.boundSourceRevisionSHA256;
    definition.attachmentMetadataSHA256 = Text(attachmentMetadataSHA256);
    for (Core3DLODAttachmentBinding *binding in expectedAttachments) {
        if (![binding isKindOfClass:Core3DLODAttachmentBinding.class]) {
            SetError(error, Core3DLODSetErrorAttachmentParity, @"Invalid attachment binding.");
            return NO;
        }
        definition.expectedAttachmentsInCanonicalOrder.push_back({
            Text(binding.attachmentIdentifier), Text(binding.canonicalMetreFrameSHA256)});
    }
    for (Core3DLODLevelRequest *request in levels) {
        if (![self ownsRequest:request]
            || request.targetTriangleCount > std::numeric_limits<std::uint32_t>::max()
            || request.measurementSampleCount > std::numeric_limits<std::uint32_t>::max()
            || request.silhouetteResolution > std::numeric_limits<std::uint32_t>::max()) {
            SetError(error, Core3DLODSetErrorForeignIdentity,
                     @"The level identity was not issued by this opening.");
            return NO;
        }
        lod::Level level;
        level.levelID = Text(request.handle.identifier); level.displayName = Text(request.displayName);
        level.triangleBudget = std::uint32_t(request.targetTriangleCount);
        level.quality.maximumSymmetricSurfaceDistanceMM = request.maximumSurfaceDistanceMM;
        level.quality.maximumSilhouettePixelDeviation = request.maximumSilhouetteDeviationPixels;
        level.quality.hardEdgeAngleDegrees = request.hardEdgeAngleDegrees;
        level.quality.contactClearanceMM = request.contactClearanceMM;
        level.quality.measurementSamplesPerMesh = std::uint32_t(request.measurementSampleCount);
        level.quality.silhouetteResolution = std::uint32_t(request.silhouetteResolution);
        level.quality.silhouetteCameraSetSHA256 = Text(request.silhouetteCameraSetSHA256);
        level.settingsSHA256 = Text(request.settingsSHA256);
        for (Core3DLODAttachmentBinding *binding in request.attachments) {
            if (![binding isKindOfClass:Core3DLODAttachmentBinding.class]) {
                SetError(error, Core3DLODSetErrorAttachmentParity,
                         @"Invalid level attachment binding.");
                return NO;
            }
            level.attachments.push_back({Text(binding.attachmentIdentifier),
                                         Text(binding.canonicalMetreFrameSHA256)});
        }
        definition.levels.push_back(std::move(level));
        if (![self consumeWork:error]) return NO;
    }
    return YES;
}

- (Core3DLODSetDraft *)draftWithLevels:(NSArray<Core3DLODLevelRequest *> *)levels
                       attachmentMetadataSHA256:(NSString *)attachmentMetadataSHA256
                            expectedAttachments:(NSArray<Core3DLODAttachmentBinding *> *)expectedAttachments
                                          error:(NSError **)error {
    if (error) *error = nil;
    if (![self isCurrent:error]) return nil;
    lod::Definition definition;
    if (![self fillDefinition:definition levels:levels
        attachmentMetadataSHA256:attachmentMetadataSHA256
        expectedAttachments:expectedAttachments error:error]) return nil;
    lod::Refusal refusal;
    if (!lod::ValidateDetachedDraft(definition, refusal)) {
        SetError(error, PublicRefusal(refusal), @"Detached LOD definition is invalid."); return nil;
    }
    if (![self isCurrent:error]) return nil;
    return [[Core3DLODSetDraft alloc] initPrivateWithDefinition:std::move(definition)
        setHandle:self.setHandle levels:levels attachments:expectedAttachments
        issuerIdentifier:self.issuerIdentifier];
}

- (Core3DLODSetDraft *)rebindDraft:(Core3DLODSetDraft *)draft
                          toCapture:(Core3DPublishCapture *)capture
                              error:(NSError **)error {
    if (error) *error = nil;
    if (_stopped.load()) {
        SetError(error, Core3DLODSetErrorStopped, @"LOD opening was stopped."); return nil;
    }
    if (![draft isKindOfClass:Core3DLODSetDraft.class]
        || ![draft.issuerIdentifier isEqualToString:self.issuerIdentifier]
        || draft.setHandle != self.setHandle) {
        SetError(error, Core3DLODSetErrorForeignIdentity, @"Foreign detached LOD draft.");
        return nil;
    }
    NSError *captureError = nil;
    if (!capture || ![capture isCurrent:&captureError]) {
        SetError(error, Core3DLODSetErrorStaleSource,
                 captureError.localizedDescription ?: @"Recaptured source is stale.");
        return nil;
    }
    if (![capture.documentIdentifier isEqualToString:draft.sourceDocumentIdentifier]
        || ![capture.selectedEntityIdentifiers isEqualToArray:self.capture.selectedEntityIdentifiers]) {
        SetError(error, Core3DLODSetErrorForeignIdentity, @"Recapture belongs to a foreign source.");
        return nil;
    }
    lod::Definition candidate = draft->_native;
    lod::Refusal refusal;
    NSString *revision = Digest(capture.sourceWitnessBytes);
    if (!lod::RebindDetachedDraft(candidate, SourceBinding(capture), Text(revision),
                                  candidate.attachmentMetadataSHA256,
                                  candidate.levels, refusal)) {
        SetError(error, PublicRefusal(refusal), @"Detached LOD rebind was refused."); return nil;
    }
    if (![self consumeWork:error]) return nil;
    if (![capture isCurrent:&captureError]) {
        SetError(error, Core3DLODSetErrorStaleSource,
                 @"Recaptured source changed during rebind."); return nil;
    }
    return [[Core3DLODSetDraft alloc] initPrivateWithDefinition:std::move(candidate)
        setHandle:draft.setHandle levels:draft.levels attachments:draft.expectedAttachments
        issuerIdentifier:self.issuerIdentifier];
}

- (Core3DLODSetCommitResult)commitDraft:(Core3DLODSetDraft *)draft {
    (void)draft;
    return Core3DLODSetCommitResultUnavailable;
}

@end
