// F3-S1 source-bound detached opening. This translation unit deliberately has
// no OcctDocument, command-lease, persistence, manifest or F2-owner access.

#import "Core3DLODSetOpening.h"
#import "Core3DDecimationOpening.h"
#import "Core3DPublishCapture.h"
#import "Core3DViewController.h"

#import "../OCCTKit/GLViewController.h"
#import "Core3DViewer.h"

#include "../OCCTKit/LODSetNativeOwner.hxx"
#include "../OCCTKit/OcctDocument.h"
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
namespace syls = core3d::lod_set_persistence;
namespace lod_owner = core3d::lod_set_owner;

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

std::vector<std::uint8_t> Bytes(NSData *data) {
    if (!data.length) return {};
    const auto *first = static_cast<const std::uint8_t *>(data.bytes);
    return {first, first + data.length};
}

NSData *Data(const std::vector<std::uint8_t>& bytes) {
    return bytes.empty() ? [NSData data]
        : [NSData dataWithBytes:bytes.data() length:bytes.size()];
}

Handle(OcctDocument) LODDocument(Core3DViewController *controller) {
    if (!controller) return Handle(OcctDocument)();
    GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)controller.glController : nil;
    const std::shared_ptr<core3d::Core3DViewer> viewer = gl ? gl.viewer : nullptr;
    return viewer ? viewer->getDocument() : Handle(OcctDocument)();
}

NSString *UUIDText(const syls::UUID& value) {
    return Text(decimation::UUIDText(value));
}

NSString *DigestText(const syls::Digest& value) {
    return Text(decimation::HexFromDigest(value));
}

bool UUID(NSString *value, syls::UUID& out) {
    return value && decimation::UUIDFromText(value.UTF8String ?: "", out)
        && decimation::Nonzero(out);
}

bool DigestValue(NSString *value, syls::Digest& out) {
    return value && decimation::DigestFromHex(value.UTF8String ?: "", out);
}

NSString *ReasonText(syls::Reason reason) {
    switch (reason) {
        case syls::Reason::None: return @"none";
        case syls::Reason::SourceChanged: return @"sourceChanged";
        case syls::Reason::SettingsChanged: return @"settingsChanged";
        case syls::Reason::MissingArtifact: return @"missingArtifact";
        case syls::Reason::ProvenanceInvalid: return @"provenanceInvalid";
    }
}

Core3DLODSetDurableState PublicState(syls::State state) {
    switch (state) {
        case syls::State::Current: return Core3DLODSetDurableStateCurrent;
        case syls::State::Stale: return Core3DLODSetDurableStateStale;
        case syls::State::Unbuilt: return Core3DLODSetDurableStateUnbuilt;
    }
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

@interface Core3DLODLevelRecord ()
@property(nonatomic, copy, readwrite) NSString *identifier;
@property(nonatomic, copy, readwrite) NSString *displayName;
@property(nonatomic, assign, readwrite) NSUInteger ordinal;
@property(nonatomic, assign, readwrite) NSUInteger targetTriangleCount;
@property(nonatomic, assign, readwrite) Core3DLODSetDurableState state;
@property(nonatomic, copy, readwrite) NSString *reason;
@property(nonatomic, assign, readwrite) double maximumSurfaceDistanceMM;
@property(nonatomic, assign, readwrite) double maximumSilhouetteDeviationPixels;
@property(nonatomic, assign, readwrite) double hardEdgeAngleDegrees;
@property(nonatomic, assign, readwrite) double contactClearanceMM;
@property(nonatomic, assign, readwrite) NSUInteger measurementSampleCount;
@property(nonatomic, assign, readwrite) NSUInteger silhouetteResolution;
@property(nonatomic, copy, readwrite) NSString *silhouetteCameraSetSHA256;
@property(nonatomic, assign, readwrite) double maximumUVStretch;
@property(nonatomic, assign, readwrite) double maximumNormalDeviationDegrees;
@property(nonatomic, assign, readwrite) NSUInteger uvMethodVersion;
@property(nonatomic, assign, readwrite) NSUInteger normalMethodVersion;
@property(nonatomic, assign, readwrite) NSUInteger overlapMethodVersion;
@property(nonatomic, copy, readwrite) NSString *intentSHA256;
@property(nonatomic, copy, readwrite, nullable) NSString *f2DerivativeIdentifier;
@property(nonatomic, copy, readwrite, nullable) NSString *f2SettingsSHA256;
@property(nonatomic, copy, readwrite, nullable) NSString *f2SourceArtifactSHA256;
@property(nonatomic, copy, readwrite, nullable) NSString *f2OutputArtifactSHA256;
@property(nonatomic, copy, readwrite, nullable) NSString *f2OutputGeometrySHA256;
@property(nonatomic, copy, readwrite, nullable) NSString *f2RemapReceiptSHA256;
@property(nonatomic, copy, readwrite, nullable) NSString *f2IndependentMeasurementSHA256;
@property(nonatomic, copy, readwrite, nullable) NSString *f2AlgorithmBuildSHA256;
@property(nonatomic, copy, readwrite, nullable) NSString *f2ArtifactName;
@property(nonatomic, assign, readwrite) NSUInteger measuredTriangleCount;
- (instancetype)initWithLevel:(const syls::Level&)level ordinal:(NSUInteger)ordinal;
@end

@implementation Core3DLODLevelRecord
- (instancetype)initWithLevel:(const syls::Level&)level ordinal:(NSUInteger)ordinal {
    if ((self = [super init])) {
        _identifier = UUIDText(level.identifier); _displayName = Text(level.name);
        _ordinal = ordinal; _targetTriangleCount = level.targetTriangleCount;
        _state = PublicState(level.state); _reason = ReasonText(level.reason);
        _maximumSurfaceDistanceMM = level.quality.maximumSurfaceDistanceMM;
        _maximumSilhouetteDeviationPixels = level.quality.maximumSilhouetteDeviationPixels;
        _hardEdgeAngleDegrees = level.quality.hardEdgeAngleDegrees;
        _contactClearanceMM = level.quality.contactClearanceMM;
        _measurementSampleCount = level.quality.measurementSampleCount;
        _silhouetteResolution = level.quality.silhouetteResolution;
        _silhouetteCameraSetSHA256 = DigestText(level.quality.silhouetteCameraSetSHA256);
        _maximumUVStretch = level.quality.maximumUVStretch;
        _maximumNormalDeviationDegrees = level.quality.maximumNormalDeviationDegrees;
        _uvMethodVersion = level.quality.uvMethodVersion;
        _normalMethodVersion = level.quality.normalMethodVersion;
        _overlapMethodVersion = level.quality.overlapMethodVersion;
        _intentSHA256 = DigestText(level.intentSHA256);
        if (level.hasF2Reference) {
            _f2DerivativeIdentifier = DigestText(level.f2.derivativeIdentifier);
            _f2SettingsSHA256 = DigestText(level.f2.settingsSHA256);
            _f2SourceArtifactSHA256 = DigestText(level.f2.sourceArtifactSHA256);
            _f2OutputArtifactSHA256 = DigestText(level.f2.outputArtifactSHA256);
            _f2OutputGeometrySHA256 = DigestText(level.f2.outputGeometrySHA256);
            _f2RemapReceiptSHA256 = DigestText(level.f2.remapReceiptSHA256);
            _f2IndependentMeasurementSHA256 =
                DigestText(level.f2.independentMeasurementSHA256);
            _f2AlgorithmBuildSHA256 = DigestText(level.f2.algorithmBuildSHA256);
            _f2ArtifactName = Text(level.f2.artifactName);
            _measuredTriangleCount = level.f2.measuredTriangleCount;
        }
    }
    return self;
}
@end

@interface Core3DLODSetRecord ()
@property(nonatomic, copy, readwrite) NSString *identifier;
@property(nonatomic, copy, readwrite) NSString *documentIdentifier;
@property(nonatomic, assign, readwrite) NSUInteger revision;
@property(nonatomic, assign, readwrite) NSUInteger nativeEditRevision;
@property(nonatomic, assign, readwrite) NSUInteger settingsRevision;
@property(nonatomic, assign, readwrite) double metersPerUnit;
@property(nonatomic, assign, readwrite) Core3DLODSetDurableState state;
@property(nonatomic, copy, readwrite) NSString *reason;
@property(nonatomic, copy, readwrite) NSString *observedSourceSHA256;
@property(nonatomic, copy, readwrite, nullable) NSString *lastBuiltSourceSHA256;
@property(nonatomic, copy, readwrite) NSString *attachmentMetadataSHA256;
@property(nonatomic, copy, readwrite) NSArray<Core3DLODLevelRecord *> *levels;
@property(nonatomic, copy, readwrite) NSData *canonicalSYLSRecord;
- (instancetype)initWithRecord:(const syls::Record&)record bytes:(NSData *)bytes;
@end

@implementation Core3DLODSetRecord
- (instancetype)initWithRecord:(const syls::Record&)record bytes:(NSData *)bytes {
    if ((self = [super init])) {
        _identifier = UUIDText(record.setIdentifier);
        _documentIdentifier = UUIDText(record.documentIdentifier);
        _revision = static_cast<NSUInteger>(record.revision);
        _nativeEditRevision = static_cast<NSUInteger>(record.nativeEditRevision);
        _settingsRevision = static_cast<NSUInteger>(record.settingsRevision);
        _metersPerUnit = record.metersPerUnit; _state = PublicState(record.state);
        _reason = ReasonText(record.reason);
        _observedSourceSHA256 = DigestText(record.observedSourceSHA256);
        _lastBuiltSourceSHA256 = record.hasLastBuiltSource
            ? DigestText(record.lastBuiltSourceSHA256) : nil;
        _attachmentMetadataSHA256 = DigestText(record.attachmentMetadataSHA256);
        NSMutableArray *levels = [NSMutableArray arrayWithCapacity:record.levels.size()];
        for (std::size_t index = 0; index < record.levels.size(); ++index)
            [levels addObject:[[Core3DLODLevelRecord alloc]
                initWithLevel:record.levels[index] ordinal:index]];
        _levels = [levels copy]; _canonicalSYLSRecord = [bytes copy];
    }
    return self;
}
@end

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
@property(nonatomic, copy, nullable) NSString *existingSetIdentifier;
- (instancetype)initPrivateWithCapture:(Core3DPublishCapture *)capture;
- (BOOL)buildAllLevelCandidates:(const std::vector<syls::Level>&)levels
                      f2Opening:(Core3DDecimationOpening *)f2Opening
                     positions:(NSArray<NSNumber *> *)positions
                        indices:(NSArray<NSNumber *> *)indices
                         tokens:(NSArray<NSString *> *)tokens
                       material:(NSData *)material
                 sourceArtifact:(NSString *)sourceArtifact
                  metersPerUnit:(double)metersPerUnit
                  settingsBytes:(NSArray<NSData *> * _Nullable * _Nonnull)settingsBytes
                       requests:(NSArray<Core3DDecimationRequest *> * _Nullable * _Nonnull)requests
                        results:(NSArray<Core3DDecimationResult *> * _Nullable * _Nonnull)results
                          error:(NSError **)error;
@end

namespace {
enum class LODWriteOutcome { Committed, Refused, Stale, Recovery };

LODWriteOutcome WriteLODStore(const Handle(OcctDocument)& document,
                              Core3DPublishCapture *capture,
                              const syls::StoreRead& before,
                              const std::vector<std::uint8_t>& candidate,
                              bool remove) noexcept {
    try {
        const Handle(TDocStd_Document) ocaf = document.IsNull()
            ? Handle(TDocStd_Document)() : document->Document();
        if (ocaf.IsNull() || ocaf->HasOpenCommand() || !before.readable()
            || (!remove && (candidate.empty()
                || candidate.size() > syls::kMaximumRecordBytes)))
            return LODWriteOutcome::Refused;
        NSError *currentError = nil;
        if (![capture isCurrent:&currentError]) return LODWriteOutcome::Stale;
        const Standard_Integer undoBefore = ocaf->GetAvailableUndos();
        try { ocaf->NewCommand(); } catch (...) { return LODWriteOutcome::Refused; }
        if (!ocaf->HasOpenCommand()) return LODWriteOutcome::Refused;
        auto abortAndVerify = [&]() -> bool {
            try { if (ocaf->HasOpenCommand()) ocaf->AbortCommand(); }
            catch (...) { return false; }
            syls::StoreRead restored;
            return !ocaf->HasOpenCommand() && document->ReadLODSetStore(restored)
                && syls::SameStore(before, restored);
        };
#if DEBUG
        const lod_owner::Fault fault = lod_owner::debugFault.load();
        if (fault == lod_owner::Fault::BeforeStage)
            return abortAndVerify() ? LODWriteOutcome::Refused : LODWriteOutcome::Recovery;
#endif
        syls::StoreRead staged;
        if (!document->StageLODSetStore(before, candidate,
                remove ? Standard_True : Standard_False, staged)
            || (remove && staged.occurrence != syls::StoreOccurrence::Absent)
            || (!remove && (staged.occurrence != syls::StoreOccurrence::Present
                || staged.bytes != candidate)))
            return abortAndVerify() ? LODWriteOutcome::Refused : LODWriteOutcome::Recovery;
#if DEBUG
        if (fault == lod_owner::Fault::AfterSYLSStage
            || fault == lod_owner::Fault::AfterReadback)
            return abortAndVerify() ? LODWriteOutcome::Refused : LODWriteOutcome::Recovery;
#endif
        bool committed = false;
        try { committed = ocaf->CommitCommand() && !ocaf->HasOpenCommand(); }
        catch (...) { committed = false; }
        if (!committed) return LODWriteOutcome::Recovery;
#if DEBUG
        if (fault == lod_owner::Fault::UnknownClose) return LODWriteOutcome::Recovery;
#endif
        syls::StoreRead readback;
        if (!document->ReadLODSetStore(readback)
            || (remove && readback.occurrence != syls::StoreOccurrence::Absent)
            || (!remove && (readback.occurrence != syls::StoreOccurrence::Present
                || readback.bytes != candidate))
            || ocaf->GetAvailableUndos() - undoBefore != 1
            || !document->FinalizeLODSetHistory()) return LODWriteOutcome::Recovery;
        return LODWriteOutcome::Committed;
    } catch (...) { return LODWriteOutcome::Recovery; }
}

bool DecodeStores(const Handle(OcctDocument)& document,
                  syls::StoreRead& read, syls::Store& store,
                  decimation::F2Store& f2Store, bool& hasF2,
                  NSError **error) {
    hasF2 = false; read = {}; store = {}; f2Store = {};
    if (document.IsNull() || !document->ReadLODSetStore(read)) {
        SetError(error, Core3DLODSetErrorPersistenceUnavailable,
            @"LOD-set store read is unavailable."); return false;
    }
    if (read.occurrence == syls::StoreOccurrence::Corrupt) {
        SetError(error, Core3DLODSetErrorCorruptRecord,
            [NSString stringWithFormat:@"Corrupt SYLS/1 record: %s", read.refusal.c_str()]);
        return false;
    }
    if (read.occurrence == syls::StoreOccurrence::Present
        && !syls::DecodeStore(read.bytes, store)) {
        SetError(error, Core3DLODSetErrorCorruptRecord, @"Noncanonical SYLS/1 record.");
        return false;
    }
    decimation::F2StoreRead f2Read;
    if (!document->ReadF2DerivativeStore(f2Read)
        || f2Read.occurrence == decimation::F2StoreOccurrence::Corrupt) {
        SetError(error, Core3DLODSetErrorCorruptRecord, @"F2 reference store is corrupt.");
        return false;
    }
    if (f2Read.occurrence == decimation::F2StoreOccurrence::Present) {
        if (!decimation::DecodeStore(f2Read.bytes, f2Store)) {
            SetError(error, Core3DLODSetErrorCorruptRecord, @"F2 reference store is noncanonical.");
            return false;
        }
        hasF2 = true;
    }
    return true;
}
} // namespace

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

+ (instancetype)openingWithCapture:(Core3DPublishCapture *)capture
                         controller:(Core3DViewController *)controller
              existingSetIdentifier:(NSString *)existingSetIdentifier
                              error:(NSError **)error {
    if (error) *error = nil;
    Core3DLODSetOpening *opening = [self openingWithCapture:capture error:error];
    if (!opening) return nil;
    const Handle(OcctDocument) document = LODDocument(controller);
    if (document.IsNull()
        || document->DocumentIdentifier() != (capture.documentIdentifier.UTF8String ?: "")) {
        SetError(error, Core3DLODSetErrorPersistenceUnavailable,
            @"The controller does not own this captured document."); return nil;
    }
    if (!existingSetIdentifier) return opening;
    syls::UUID identifier{};
    if (!UUID(existingSetIdentifier, identifier)) {
        SetError(error, Core3DLODSetErrorNotFound, @"LOD-set identifier is invalid."); return nil;
    }
    syls::StoreRead read; syls::Store store; decimation::F2Store f2Store; bool hasF2 = false;
    if (!DecodeStores(document, read, store, f2Store, hasF2, error)) return nil;
    const syls::Record *record = lod_owner::Find(store, identifier);
    lod_owner::Refusal refusal;
    if (!record || !lod_owner::ValidateReferences(*record, hasF2 ? &f2Store : nullptr, refusal)) {
        SetError(error, record ? Core3DLODSetErrorCorruptRecord : Core3DLODSetErrorNotFound,
            record ? @"LOD-set references do not exactly match F2 readback."
                   : @"LOD set was not found.");
        return nil;
    }
    opening.setHandle = [[Core3DLODSetHandle alloc]
        initPrivateWithIdentifier:UUIDText(record->setIdentifier).lowercaseString];
    NSMutableArray *handles = [NSMutableArray arrayWithCapacity:lod::kMaximumLevels];
    for (std::size_t index = 0; index < record->levels.size(); ++index)
        [handles addObject:[[Core3DLODLevelHandle alloc]
            initPrivateWithIdentifier:UUIDText(record->levels[index].identifier).lowercaseString
            ordinal:index]];
    while (handles.count < lod::kMaximumLevels)
        [handles addObject:[[Core3DLODLevelHandle alloc]
            initPrivateWithIdentifier:NSUUID.UUID.UUIDString.lowercaseString
            ordinal:handles.count]];
    opening.levelHandles = [handles copy];
    opening.existingSetIdentifier = existingSetIdentifier.lowercaseString;
    return opening;
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

- (BOOL)buildAllLevelCandidates:(const std::vector<syls::Level>&)levels
                      f2Opening:(Core3DDecimationOpening *)f2Opening
                     positions:(NSArray<NSNumber *> *)positions
                        indices:(NSArray<NSNumber *> *)indices
                         tokens:(NSArray<NSString *> *)tokens
                       material:(NSData *)material
                 sourceArtifact:(NSString *)sourceArtifact
                  metersPerUnit:(double)metersPerUnit
                  settingsBytes:(NSArray<NSData *> **)settingsBytes
                       requests:(NSArray<Core3DDecimationRequest *> **)requests
                        results:(NSArray<Core3DDecimationResult *> **)results
                          error:(NSError **)error {
    *settingsBytes = nil; *requests = nil; *results = nil;
    if (!f2Opening || levels.empty() || !positions || !indices || !tokens
        || !material.length || !sourceArtifact.length) {
        SetError(error, Core3DLODSetErrorMissingProvenance,
            @"Candidate generation requires one complete immutable source.");
        return NO;
    }
    NSMutableArray<NSData *> *allSettings = [NSMutableArray arrayWithCapacity:levels.size()];
    NSMutableArray<Core3DDecimationRequest *> *allRequests =
        [NSMutableArray arrayWithCapacity:levels.size()];
    NSMutableArray<Core3DDecimationResult *> *allResults =
        [NSMutableArray arrayWithCapacity:levels.size()];
    NSUInteger previousCount = NSUIntegerMax;
    for (std::size_t index = 0; index < levels.size(); ++index) {
        if (![self consumeWork:error]) return NO;
        const syls::Level& level = levels[index];
        NSData *settings = [Core3DDecimationOpening
            f2CanonicalSettingsBytesWithTargetTriangleCount:level.targetTriangleCount
            maximumSymmetricSurfaceDistanceMM:level.quality.maximumSurfaceDistanceMM
            maximumSilhouettePixelDeviation:level.quality.maximumSilhouetteDeviationPixels
            hardEdgeAngleDegrees:level.quality.hardEdgeAngleDegrees
            contactClearanceMM:level.quality.contactClearanceMM
            measurementSamplesPerMesh:level.quality.measurementSampleCount
            silhouetteResolution:level.quality.silhouetteResolution
            silhouetteCameraSetSHA256:DigestText(level.quality.silhouetteCameraSetSHA256)
            metersPerUnit:metersPerUnit];
        NSString *settingsDigest = settings
            ? [Core3DDecimationOpening f2SettingsSHA256ForCanonicalBytes:settings] : nil;
        if (!settings || !settingsDigest) {
            SetError(error, Core3DLODSetErrorInvalidDefinition,
                @"A level does not contain canonical F2 settings.");
            return NO;
        }
        Core3DDecimationRequest *request = [[Core3DDecimationRequest alloc]
            initWithPositionsMM:positions indices:indices
            sourceContributionTokens:tokens materialBytes:material
            sourceArtifactSHA256:sourceArtifact settingsSHA256:settingsDigest
            targetTriangleCount:level.targetTriangleCount metersPerUnit:metersPerUnit];
        NSError *f2Error = nil;
        Core3DDecimationResult *result = [f2Opening decimateRequest:request error:&f2Error];
        const BOOL nonReducingResult = result
            && (result.triangleCount != level.targetTriangleCount
                || result.triangleCount >= previousCount);
        const BOOL unreachableRefusal = !result
            && [f2Error.domain isEqualToString:Core3DDecimationErrorDomain]
            && f2Error.code == Core3DDecimationErrorUnreachableTriangleBudget;
        if (!result || result.triangleCount != level.targetTriangleCount
            || result.triangleCount >= previousCount) {
            NSString *reason = (unreachableRefusal || nonReducingResult)
                ? @"F2 source is not reducible to every strictly decreasing target."
                : (f2Error.localizedDescription
                    ?: @"F2 native decimation refused without output.");
            if (error) {
                NSDictionary *userInfo = f2Error
                    ? @{NSLocalizedDescriptionKey: reason, NSUnderlyingErrorKey: f2Error}
                    : @{NSLocalizedDescriptionKey: reason};
                *error = [NSError errorWithDomain:Core3DLODSetErrorDomain
                    code:Core3DLODSetErrorInvalidDefinition userInfo:userInfo];
            }
            return NO;
        }
        previousCount = result.triangleCount;
        [allSettings addObject:settings];
        [allRequests addObject:request];
        [allResults addObject:result];
#if DEBUG
        if (index == 0 && lod_owner::debugFault.load() == lod_owner::Fault::BeforeStage) {
            SetError(error, Core3DLODSetErrorPersistenceUnavailable,
                @"Injected all-level candidate refusal.");
            return NO;
        }
#endif
    }
    *settingsBytes = [allSettings copy];
    *requests = [allRequests copy];
    *results = [allResults copy];
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

- (Core3DLODSetCommitResult)commitDraft:(Core3DLODSetDraft *)draft
                              controller:(Core3DViewController *)controller
                                   error:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || ![draft isKindOfClass:Core3DLODSetDraft.class]
        || draft.setHandle != self.setHandle
        || ![draft.issuerIdentifier isEqualToString:self.issuerIdentifier]) {
        SetError(error, Core3DLODSetErrorForeignIdentity, @"Foreign durable LOD draft.");
        return Core3DLODSetCommitResultUnavailable;
    }
    if (![self isCurrent:error]) return Core3DLODSetCommitResultUnavailable;
    const Handle(OcctDocument) document = LODDocument(controller);
    if (document.IsNull()
        || document->DocumentIdentifier() != (self.capture.documentIdentifier.UTF8String ?: "")) {
        SetError(error, Core3DLODSetErrorPersistenceUnavailable,
            @"LOD persistence is unavailable for this controller.");
        return Core3DLODSetCommitResultUnavailable;
    }
    syls::StoreRead before; syls::Store stored; decimation::F2Store f2Store;
    bool hasF2 = false;
    if (!DecodeStores(document, before, stored, f2Store, hasF2, error))
        return Core3DLODSetCommitResultUnavailable;

    syls::Record candidate;
    if (!UUID(self.capture.documentIdentifier, candidate.documentIdentifier)
        || !UUID(draft.setHandle.identifier, candidate.setIdentifier)
        || !UUID(self.capture.sourceEntityIdentifier, candidate.sourceEntityIdentifier)
        || !UUID(self.capture.sourceDefinitionIdentifier, candidate.sourceDefinitionIdentifier)
        || !UUID(self.capture.sourceFeatureIdentifier, candidate.sourceFeatureIdentifier)
        || !DigestValue(draft.observedSourceRevisionSHA256, candidate.observedSourceSHA256)
        || !DigestValue(draft.attachmentMetadataSHA256, candidate.attachmentMetadataSHA256)) {
        SetError(error, Core3DLODSetErrorMissingProvenance,
            @"The durable source binding is incomplete.");
        return Core3DLODSetCommitResultUnavailable;
    }
    candidate.metersPerUnit = self.capture.metersPerUnit;
    candidate.nativeEditRevision = self.capture.modelRevision;
    candidate.state = syls::State::Unbuilt; candidate.reason = syls::Reason::None;
    const syls::Record *prior = lod_owner::Find(stored, candidate.setIdentifier);
    if ((self.existingSetIdentifier && !prior) || (!self.existingSetIdentifier && prior)) {
        SetError(error, Core3DLODSetErrorForeignIdentity,
            @"Create/edit identity does not match the durable owner.");
        return Core3DLODSetCommitResultUnavailable;
    }
    candidate.revision = prior ? prior->revision + 1 : 1;
    candidate.settingsRevision = prior ? prior->settingsRevision + 1 : 1;
    candidate.levels.reserve(draft.levels.count);
    for (Core3DLODLevelRequest *request in draft.levels) {
        syls::Level level;
        if (!UUID(request.handle.identifier, level.identifier)
            || !DigestValue(request.silhouetteCameraSetSHA256,
                level.quality.silhouetteCameraSetSHA256)
            || !DigestValue(request.settingsSHA256, level.intentSHA256)
            || request.targetTriangleCount > std::numeric_limits<std::uint32_t>::max()
            || request.measurementSampleCount > std::numeric_limits<std::uint32_t>::max()
            || request.silhouetteResolution > std::numeric_limits<std::uint32_t>::max()) {
            SetError(error, Core3DLODSetErrorInvalidDefinition,
                @"LOD level cannot be represented canonically.");
            return Core3DLODSetCommitResultUnavailable;
        }
        level.name = Text(request.displayName);
        level.targetTriangleCount = static_cast<std::uint32_t>(request.targetTriangleCount);
        level.quality.maximumSurfaceDistanceMM = request.maximumSurfaceDistanceMM;
        level.quality.maximumSilhouetteDeviationPixels =
            request.maximumSilhouetteDeviationPixels;
        level.quality.hardEdgeAngleDegrees = request.hardEdgeAngleDegrees;
        level.quality.contactClearanceMM = request.contactClearanceMM;
        level.quality.measurementSampleCount =
            static_cast<std::uint32_t>(request.measurementSampleCount);
        level.quality.silhouetteResolution =
            static_cast<std::uint32_t>(request.silhouetteResolution);
        // S2 retains the three still-fixed methods and the two currently
        // unsupported quality dimensions explicitly instead of dropping them.
        level.quality.maximumUVStretch = 0;
        level.quality.maximumNormalDeviationDegrees = 0;
        level.quality.uvMethodVersion = 1;
        level.quality.normalMethodVersion = 1;
        level.quality.overlapMethodVersion = 1;
        level.state = syls::State::Unbuilt; level.reason = syls::Reason::None;
        candidate.levels.push_back(std::move(level));
    }
    lod_owner::Refusal refusal; syls::Store result;
    const syls::Store *beforeStore = before.occurrence == syls::StoreOccurrence::Present
        ? &stored : nullptr;
    if (!lod_owner::Upsert(beforeStore, std::move(candidate),
            hasF2 ? &f2Store : nullptr, result, refusal)) {
        SetError(error, Core3DLODSetErrorInvalidDefinition,
            @"The canonical durable LOD candidate was refused.");
        return Core3DLODSetCommitResultUnavailable;
    }
    std::vector<std::uint8_t> encoded;
    if (!syls::EncodeStore(result, encoded)) {
        SetError(error, Core3DLODSetErrorPersistenceUnavailable,
            @"SYLS/1 exceeds its bounded carrier.");
        return Core3DLODSetCommitResultUnavailable;
    }
    switch (WriteLODStore(document, self.capture, before, encoded, false)) {
        case LODWriteOutcome::Committed:
            self.existingSetIdentifier = self.setHandle.identifier;
            return Core3DLODSetCommitResultCommitted;
        case LODWriteOutcome::Stale:
            SetError(error, Core3DLODSetErrorStaleSource, @"Source changed before commit.");
            return Core3DLODSetCommitResultUnavailable;
        case LODWriteOutcome::Refused:
            SetError(error, Core3DLODSetErrorPersistenceUnavailable,
                @"SYLS/1 staging failed; the prior record is preserved.");
            return Core3DLODSetCommitResultUnavailable;
        case LODWriteOutcome::Recovery:
            SetError(error, Core3DLODSetErrorRecoveryRequired,
                @"LOD command closure is uncertain; recovery is required.");
            return Core3DLODSetCommitResultRecoveryRequired;
    }
}

+ (NSArray<Core3DLODSetRecord *> *)persistedSetsForController:
    (Core3DViewController *)controller error:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread) {
        SetError(error, Core3DLODSetErrorPersistenceUnavailable, @"Main thread required.");
        return nil;
    }
    const Handle(OcctDocument) document = LODDocument(controller);
    syls::StoreRead read; syls::Store store; decimation::F2Store f2Store; bool hasF2 = false;
    if (!DecodeStores(document, read, store, f2Store, hasF2, error)) return nil;
    if (read.occurrence == syls::StoreOccurrence::Absent) return @[];
    NSMutableArray *records = [NSMutableArray arrayWithCapacity:store.records.size()];
    for (const syls::Record& record : store.records) {
        lod_owner::Refusal refusal;
        if (!lod_owner::ValidateReferences(record, hasF2 ? &f2Store : nullptr, refusal)) {
            SetError(error, Core3DLODSetErrorCorruptRecord,
                @"SYLS/1 contains a missing or mismatched F2 reference."); return nil;
        }
        [records addObject:[[Core3DLODSetRecord alloc]
            initWithRecord:record bytes:Data(read.bytes)]];
    }
    return [records copy];
}

- (Core3DLODSetCommitResult)removeFromController:(Core3DViewController *)controller
                                           error:(NSError **)error {
    if (error) *error = nil;
    if (!self.existingSetIdentifier || ![self isCurrent:error]) {
        if (!self.existingSetIdentifier)
            SetError(error, Core3DLODSetErrorNotFound, @"No durable LOD set is bound.");
        return Core3DLODSetCommitResultUnavailable;
    }
    const Handle(OcctDocument) document = LODDocument(controller);
    syls::StoreRead before; syls::Store store; decimation::F2Store f2Store; bool hasF2 = false;
    if (!DecodeStores(document, before, store, f2Store, hasF2, error))
        return Core3DLODSetCommitResultUnavailable;
    syls::UUID identifier{}; syls::Store result; lod_owner::Refusal refusal;
    if (!UUID(self.existingSetIdentifier, identifier)
        || !lod_owner::Remove(store, identifier, result, refusal)) {
        SetError(error, Core3DLODSetErrorNotFound, @"Durable LOD set was not found.");
        return Core3DLODSetCommitResultUnavailable;
    }
    const bool removeCarrier = result.records.empty(); std::vector<std::uint8_t> encoded;
    if (!removeCarrier && !syls::EncodeStore(result, encoded)) {
        SetError(error, Core3DLODSetErrorPersistenceUnavailable, @"SYLS/1 encode failed.");
        return Core3DLODSetCommitResultUnavailable;
    }
    const LODWriteOutcome outcome = WriteLODStore(
        document, self.capture, before, encoded, removeCarrier);
    if (outcome == LODWriteOutcome::Committed) return Core3DLODSetCommitResultRemoved;
    if (outcome == LODWriteOutcome::Recovery) {
        SetError(error, Core3DLODSetErrorRecoveryRequired, @"Removal closure is uncertain.");
        return Core3DLODSetCommitResultRecoveryRequired;
    }
    SetError(error, outcome == LODWriteOutcome::Stale ? Core3DLODSetErrorStaleSource
        : Core3DLODSetErrorPersistenceUnavailable, @"LOD removal was refused.");
    return Core3DLODSetCommitResultUnavailable;
}

- (Core3DLODSetCommitResult)regenerateForController:(Core3DViewController *)controller
                                               error:(NSError **)error {
    if (error) *error = nil;
    if (!NSThread.isMainThread || !self.existingSetIdentifier) {
        SetError(error, self.existingSetIdentifier ? Core3DLODSetErrorUnavailable
            : Core3DLODSetErrorNotFound, @"A durable LOD set is required.");
        return Core3DLODSetCommitResultRegenerateUnavailable;
    }
    if (![self isCurrent:error]) return Core3DLODSetCommitResultRegenerateUnavailable;
    const Handle(OcctDocument) document = LODDocument(controller);
    const Handle(TDocStd_Document) ocaf = document.IsNull()
        ? Handle(TDocStd_Document)() : document->Document();
    if (ocaf.IsNull() || ocaf->HasOpenCommand()
        || document->DocumentIdentifier() != (self.capture.documentIdentifier.UTF8String ?: "")) {
        SetError(error, Core3DLODSetErrorPersistenceUnavailable,
            @"LOD regeneration requires an idle captured document.");
        return Core3DLODSetCommitResultRegenerateUnavailable;
    }
    syls::StoreRead sylsBefore; syls::Store store; decimation::F2Store ignoredF2;
    bool ignoredHasF2 = false;
    if (!DecodeStores(document, sylsBefore, store, ignoredF2, ignoredHasF2, error))
        return Core3DLODSetCommitResultRegenerateUnavailable;
    syls::UUID setIdentifier{};
    const syls::Record *prior = nullptr;
    if (UUID(self.existingSetIdentifier, setIdentifier))
        prior = lod_owner::Find(store, setIdentifier);
    Core3DPublishArtifactEvidence *evidence = self.capture.artifactEvidence;
    Core3DPublishOwnerCommitments *owner = self.capture.ownerCommitments.firstObject;
    if (!prior || !evidence || !owner || prior->levels.empty()
        || evidence.sourcePositionsMM.count != evidence.addressedTriangleCount * 9
        || evidence.sourceIndices.count != evidence.addressedTriangleCount * 3
        || evidence.faceEvidenceSHA256InEmittedOrder.count != evidence.addressedTriangleCount
        || owner.materialTextureWitnessBytes.length == 0) {
        SetError(error, prior ? Core3DLODSetErrorMissingProvenance
            : Core3DLODSetErrorNotFound,
            @"Regeneration requires the durable set and one completed source artifact observation.");
        return Core3DLODSetCommitResultRegenerateUnavailable;
    }

    NSError *f2Error = nil;
    Core3DDecimationOpening *f2Opening =
        [Core3DDecimationOpening openingWithCapture:self.capture error:&f2Error];
    if (!f2Opening) {
        SetError(error, Core3DLODSetErrorMissingProvenance,
            f2Error.localizedDescription ?: @"F2 opening refused the captured source.");
        return Core3DLODSetCommitResultRegenerateUnavailable;
    }
    NSArray<NSData *> *settingsBytes = nil;
    NSArray<Core3DDecimationRequest *> *requests = nil;
    NSArray<Core3DDecimationResult *> *results = nil;
    if (![self buildAllLevelCandidates:prior->levels f2Opening:f2Opening
            positions:evidence.sourcePositionsMM indices:evidence.sourceIndices
            tokens:evidence.faceEvidenceSHA256InEmittedOrder
            material:owner.materialTextureWitnessBytes
            sourceArtifact:evidence.artifactSHA256 metersPerUnit:prior->metersPerUnit
            settingsBytes:&settingsBytes requests:&requests results:&results error:error])
        return Core3DLODSetCommitResultRegenerateUnavailable;
    if (![self isCurrent:error]) return Core3DLODSetCommitResultRegenerateUnavailable;

    decimation::F2StoreRead f2Before;
    if (!document->ReadF2DerivativeStore(f2Before) || !f2Before.readable()) {
        SetError(error, Core3DLODSetErrorPersistenceUnavailable,
            @"F2 store read failed before regeneration.");
        return Core3DLODSetCommitResultRegenerateUnavailable;
    }
    const Standard_Integer undoBefore = ocaf->GetAvailableUndos();
    try { ocaf->NewCommand(); } catch (...) {
        SetError(error, Core3DLODSetErrorPersistenceUnavailable,
            @"LOD regeneration command could not open.");
        return Core3DLODSetCommitResultRegenerateUnavailable;
    }
    auto abortAndVerify = [&]() -> bool {
        try { if (ocaf->HasOpenCommand()) ocaf->AbortCommand(); }
        catch (...) { return false; }
        syls::StoreRead restoredSYLS; decimation::F2StoreRead restoredF2;
        return !ocaf->HasOpenCommand()
            && document->ReadLODSetStore(restoredSYLS)
            && document->ReadF2DerivativeStore(restoredF2)
            && syls::SameStore(sylsBefore, restoredSYLS)
            && decimation::SameStore(f2Before, restoredF2);
    };
#if DEBUG
    const lod_owner::Fault fault = lod_owner::debugFault.load();
    if (fault == lod_owner::Fault::BeforeStage) {
        const bool restored = abortAndVerify();
        SetError(error, restored ? Core3DLODSetErrorPersistenceUnavailable
                                 : Core3DLODSetErrorRecoveryRequired,
            @"Injected regeneration refusal before staging.");
        return restored ? Core3DLODSetCommitResultRegenerateUnavailable
                        : Core3DLODSetCommitResultRecoveryRequired;
    }
#endif
    for (NSUInteger index = 0; index < results.count; ++index) {
        f2Error = nil;
        if (![f2Opening f2StageDerivativeForController:controller level:index
                request:requests[index] result:results[index]
                settingsCanonicalBytes:settingsBytes[index] error:&f2Error]) {
            const bool restored = abortAndVerify();
            SetError(error, restored ? Core3DLODSetErrorPersistenceUnavailable
                                     : Core3DLODSetErrorRecoveryRequired,
                f2Error.localizedDescription ?: @"F2 regeneration staging failed.");
            return restored ? Core3DLODSetCommitResultRegenerateUnavailable
                            : Core3DLODSetCommitResultRecoveryRequired;
        }
#if DEBUG
        if (fault == lod_owner::Fault::AfterF2Stage) {
            const bool restored = abortAndVerify();
            SetError(error, restored ? Core3DLODSetErrorPersistenceUnavailable
                                     : Core3DLODSetErrorRecoveryRequired,
                @"Injected regeneration refusal after F2 staging.");
            return restored ? Core3DLODSetCommitResultRegenerateUnavailable
                            : Core3DLODSetCommitResultRecoveryRequired;
        }
#endif
    }
    decimation::F2StoreRead stagedF2Read; decimation::F2Store stagedF2;
    if (!document->ReadF2DerivativeStore(stagedF2Read)
        || stagedF2Read.occurrence != decimation::F2StoreOccurrence::Present
        || !decimation::DecodeStore(stagedF2Read.bytes, stagedF2)) {
        const bool restored = abortAndVerify();
        SetError(error, restored ? Core3DLODSetErrorPersistenceUnavailable
                                 : Core3DLODSetErrorRecoveryRequired,
            @"Staged F2 readback was invalid.");
        return restored ? Core3DLODSetCommitResultRegenerateUnavailable
                        : Core3DLODSetCommitResultRecoveryRequired;
    }
    syls::Record candidate = *prior; ++candidate.revision;
    lod_owner::Refusal refusal = lod_owner::Refusal::None;
    for (std::size_t index = 0; index < candidate.levels.size(); ++index) {
        NSString *artifact = [NSString stringWithFormat:@"lod-level-%zu.glb", index];
        if (!lod_owner::AttachCurrentReference(
                candidate, index, stagedF2, Text(artifact), refusal)) {
            const bool restored = abortAndVerify();
            SetError(error, restored ? Core3DLODSetErrorMissingProvenance
                                     : Core3DLODSetErrorRecoveryRequired,
                @"A staged F2 row did not exactly bind its LOD level.");
            return restored ? Core3DLODSetCommitResultRegenerateUnavailable
                            : Core3DLODSetCommitResultRecoveryRequired;
        }
    }
    syls::Store candidateStore;
    if (!lod_owner::Upsert(&store, std::move(candidate), &stagedF2,
            candidateStore, refusal)) {
        const bool restored = abortAndVerify();
        SetError(error, restored ? Core3DLODSetErrorCorruptRecord
                                 : Core3DLODSetErrorRecoveryRequired,
            @"The complete regenerated set was refused.");
        return restored ? Core3DLODSetCommitResultRegenerateUnavailable
                        : Core3DLODSetCommitResultRecoveryRequired;
    }
    std::vector<std::uint8_t> sylsBytes; syls::StoreRead stagedSYLS;
    if (!syls::EncodeStore(candidateStore, sylsBytes)
        || !document->StageLODSetStore(
            sylsBefore, sylsBytes, Standard_False, stagedSYLS)
        || stagedSYLS.occurrence != syls::StoreOccurrence::Present
        || stagedSYLS.bytes != sylsBytes) {
        const bool restored = abortAndVerify();
        SetError(error, restored ? Core3DLODSetErrorPersistenceUnavailable
                                 : Core3DLODSetErrorRecoveryRequired,
            @"Complete SYLS regeneration staging failed.");
        return restored ? Core3DLODSetCommitResultRegenerateUnavailable
                        : Core3DLODSetCommitResultRecoveryRequired;
    }
#if DEBUG
    if (fault == lod_owner::Fault::AfterSYLSStage
        || fault == lod_owner::Fault::AfterReadback) {
        const bool restored = abortAndVerify();
        SetError(error, restored ? Core3DLODSetErrorPersistenceUnavailable
                                 : Core3DLODSetErrorRecoveryRequired,
            @"Injected regeneration refusal after SYLS staging.");
        return restored ? Core3DLODSetCommitResultRegenerateUnavailable
                        : Core3DLODSetCommitResultRecoveryRequired;
    }
#endif
    bool committed = false;
    try { committed = ocaf->CommitCommand() && !ocaf->HasOpenCommand(); }
    catch (...) { committed = false; }
    if (!committed) {
        SetError(error, Core3DLODSetErrorRecoveryRequired,
            @"Regeneration command closure is uncertain.");
        return Core3DLODSetCommitResultRecoveryRequired;
    }
#if DEBUG
    if (fault == lod_owner::Fault::UnknownClose) {
        SetError(error, Core3DLODSetErrorRecoveryRequired,
            @"Injected unknown regeneration closure.");
        return Core3DLODSetCommitResultRecoveryRequired;
    }
#endif
    syls::StoreRead finalSYLS; decimation::F2StoreRead finalF2;
    if (!document->ReadLODSetStore(finalSYLS) || finalSYLS.bytes != sylsBytes
        || !document->ReadF2DerivativeStore(finalF2)
        || finalF2.bytes != stagedF2Read.bytes
        || ocaf->GetAvailableUndos() - undoBefore != 1
        || !document->FinalizeF2DerivativeHistory()
        || !document->FinalizeLODSetHistory()) {
        SetError(error, Core3DLODSetErrorRecoveryRequired,
            @"Regeneration readback or history finalization failed.");
        return Core3DLODSetCommitResultRecoveryRequired;
    }
    return Core3DLODSetCommitResultCommitted;
}

- (Core3DLODSetCommitResult)referencePersistedF2Level:(NSUInteger)level
                                             artifactName:(NSString *)artifactName
                                               controller:(Core3DViewController *)controller
                                                    error:(NSError **)error {
    if (error) *error = nil;
    if (!self.existingSetIdentifier || ![self isCurrent:error]) {
        if (!self.existingSetIdentifier)
            SetError(error, Core3DLODSetErrorNotFound, @"No durable LOD set is bound.");
        return Core3DLODSetCommitResultUnavailable;
    }
    const Handle(OcctDocument) document = LODDocument(controller);
    syls::StoreRead before; syls::Store store; decimation::F2Store f2Store;
    bool hasF2 = false;
    if (!DecodeStores(document, before, store, f2Store, hasF2, error)
        || !hasF2 || level > std::numeric_limits<std::uint8_t>::max()) {
        if (error && !*error) SetError(error, Core3DLODSetErrorMissingProvenance,
            @"A persisted F2 row is required.");
        return Core3DLODSetCommitResultUnavailable;
    }
    syls::UUID identifier{};
    const syls::Record *prior = nullptr;
    if (UUID(self.existingSetIdentifier, identifier)) prior = lod_owner::Find(store, identifier);
    if (!prior) {
        SetError(error, Core3DLODSetErrorNotFound, @"Durable LOD set was not found.");
        return Core3DLODSetCommitResultUnavailable;
    }
    syls::Record candidate = *prior;
    ++candidate.revision;
    lod_owner::Refusal refusal;
    if (!lod_owner::AttachCurrentReference(candidate, level, f2Store,
            Text(artifactName), refusal)) {
        SetError(error, Core3DLODSetErrorMissingProvenance,
            @"The F2 row did not exactly match this set and level.");
        return Core3DLODSetCommitResultUnavailable;
    }
    syls::Store result;
    if (!lod_owner::Upsert(&store, std::move(candidate), &f2Store, result, refusal)) {
        SetError(error, Core3DLODSetErrorCorruptRecord,
            @"The exact F2 reference could not be adopted.");
        return Core3DLODSetCommitResultUnavailable;
    }
    std::vector<std::uint8_t> encoded;
    if (!syls::EncodeStore(result, encoded)) {
        SetError(error, Core3DLODSetErrorPersistenceUnavailable, @"SYLS/1 encode failed.");
        return Core3DLODSetCommitResultUnavailable;
    }
    const LODWriteOutcome outcome = WriteLODStore(document, self.capture, before, encoded, false);
    if (outcome == LODWriteOutcome::Committed) return Core3DLODSetCommitResultCommitted;
    if (outcome == LODWriteOutcome::Recovery) {
        SetError(error, Core3DLODSetErrorRecoveryRequired,
            @"F2-reference closure is uncertain; recovery is required.");
        return Core3DLODSetCommitResultRecoveryRequired;
    }
    SetError(error, outcome == LODWriteOutcome::Stale ? Core3DLODSetErrorStaleSource
        : Core3DLODSetErrorPersistenceUnavailable, @"F2-reference staging was refused.");
    return Core3DLODSetCommitResultUnavailable;
}

+ (NSData *)reencodeSYLSRecord:(NSData *)record error:(NSError **)error {
    if (error) *error = nil;
    syls::Store store; std::vector<std::uint8_t> encoded;
    if (![record isKindOfClass:NSData.class] || record.length > syls::kMaximumRecordBytes
        || !syls::DecodeStore(Bytes(record), store) || !syls::EncodeStore(store, encoded)) {
        SetError(error, Core3DLODSetErrorCorruptRecord, @"Strict SYLS/1 decode refused.");
        return nil;
    }
    return Data(encoded);
}

+ (NSUInteger)maximumSYLSRecordBytes { return syls::kMaximumRecordBytes; }

+ (NSData *)f2CanonicalSettingsBytesForLevel:(Core3DLODLevelRecord *)level
                                metersPerUnit:(double)metersPerUnit
                                         error:(NSError **)error {
    if (error) *error = nil;
    if (![level isKindOfClass:Core3DLODLevelRecord.class]) {
        SetError(error, Core3DLODSetErrorInvalidDefinition,
            @"A native persisted LOD level is required.");
        return nil;
    }
    NSData *bytes = [Core3DDecimationOpening
        f2CanonicalSettingsBytesWithTargetTriangleCount:level.targetTriangleCount
        maximumSymmetricSurfaceDistanceMM:level.maximumSurfaceDistanceMM
        maximumSilhouettePixelDeviation:level.maximumSilhouetteDeviationPixels
        hardEdgeAngleDegrees:level.hardEdgeAngleDegrees
        contactClearanceMM:level.contactClearanceMM
        measurementSamplesPerMesh:level.measurementSampleCount
        silhouetteResolution:level.silhouetteResolution
        silhouetteCameraSetSHA256:level.silhouetteCameraSetSHA256
        metersPerUnit:metersPerUnit];
    if (!bytes) {
        SetError(error, Core3DLODSetErrorInvalidDefinition,
            @"Persisted LOD intent cannot be represented as canonical F2 settings.");
        return nil;
    }
    return bytes;
}

#if DEBUG
- (NSArray<Core3DDecimationResult *> *)debugCandidatesForLevels:
    (NSArray<Core3DLODLevelRequest *> *)levels
    positionsMM:(NSArray<NSNumber *> *)positionsMM
    indices:(NSArray<NSNumber *> *)indices
    sourceContributionTokens:(NSArray<NSString *> *)sourceContributionTokens
    materialBytes:(NSData *)materialBytes
    sourceArtifactSHA256:(NSString *)sourceArtifactSHA256
    metersPerUnit:(double)metersPerUnit
    error:(NSError **)error {
    if (error) *error = nil;
    if (levels.count < 2 || levels.count > lod::kMaximumLevels) {
        SetError(error, Core3DLODSetErrorInvalidDefinition,
            @"Synthetic candidate observation requires two to eight levels.");
        return nil;
    }
    std::vector<syls::Level> nativeLevels;
    nativeLevels.reserve(levels.count);
    for (Core3DLODLevelRequest *request in levels) {
        syls::Level level;
        if (![self ownsRequest:request]
            || request.targetTriangleCount > std::numeric_limits<std::uint32_t>::max()
            || request.measurementSampleCount > std::numeric_limits<std::uint32_t>::max()
            || request.silhouetteResolution > std::numeric_limits<std::uint32_t>::max()
            || !DigestValue(request.silhouetteCameraSetSHA256,
                            level.quality.silhouetteCameraSetSHA256)) {
            SetError(error, Core3DLODSetErrorInvalidDefinition,
                @"Synthetic candidate level definition is invalid.");
            return nil;
        }
        level.targetTriangleCount = static_cast<std::uint32_t>(request.targetTriangleCount);
        level.quality.maximumSurfaceDistanceMM = request.maximumSurfaceDistanceMM;
        level.quality.maximumSilhouetteDeviationPixels =
            request.maximumSilhouetteDeviationPixels;
        level.quality.hardEdgeAngleDegrees = request.hardEdgeAngleDegrees;
        level.quality.contactClearanceMM = request.contactClearanceMM;
        level.quality.measurementSampleCount =
            static_cast<std::uint32_t>(request.measurementSampleCount);
        level.quality.silhouetteResolution =
            static_cast<std::uint32_t>(request.silhouetteResolution);
        nativeLevels.push_back(std::move(level));
    }
    NSError *f2Error = nil;
    Core3DDecimationOpening *f2Opening =
        [Core3DDecimationOpening openingWithCapture:nil error:&f2Error];
    if (!f2Opening) {
        SetError(error, Core3DLODSetErrorMissingProvenance,
            f2Error.localizedDescription ?: @"Synthetic F2 opening is unavailable.");
        return nil;
    }
    NSArray<NSData *> *settings = nil;
    NSArray<Core3DDecimationRequest *> *requests = nil;
    NSArray<Core3DDecimationResult *> *results = nil;
    if (![self buildAllLevelCandidates:nativeLevels f2Opening:f2Opening
            positions:positionsMM indices:indices tokens:sourceContributionTokens
            material:materialBytes sourceArtifact:sourceArtifactSHA256
            metersPerUnit:metersPerUnit settingsBytes:&settings
            requests:&requests results:&results error:error]) return nil;
    return results;
}

+ (void)setDebugFault:(Core3DLODSetDebugFault)fault {
    lod_owner::Fault value = lod_owner::Fault::None;
    switch (fault) {
        case Core3DLODSetDebugFaultBeforeStage: value = lod_owner::Fault::BeforeStage; break;
        case Core3DLODSetDebugFaultAfterF2Stage: value = lod_owner::Fault::AfterF2Stage; break;
        case Core3DLODSetDebugFaultAfterSYLSStage: value = lod_owner::Fault::AfterSYLSStage; break;
        case Core3DLODSetDebugFaultAfterReadback: value = lod_owner::Fault::AfterReadback; break;
        case Core3DLODSetDebugFaultUnknownClose: value = lod_owner::Fault::UnknownClose; break;
        case Core3DLODSetDebugFaultNone: break;
    }
    lod_owner::debugFault.store(value);
}

+ (BOOL)debugRemoveF2StoreForController:(Core3DViewController *)controller
                                  error:(NSError **)error {
    if (error) *error = nil;
    const Handle(OcctDocument) document = LODDocument(controller);
    const Handle(TDocStd_Document) ocaf = document.IsNull()
        ? Handle(TDocStd_Document)() : document->Document();
    decimation::F2StoreRead before;
    if (ocaf.IsNull() || ocaf->HasOpenCommand()
        || !document->ReadF2DerivativeStore(before)
        || before.occurrence != decimation::F2StoreOccurrence::Present) {
        SetError(error, Core3DLODSetErrorPersistenceUnavailable,
            @"A readable F2 store is required."); return NO;
    }
    try { ocaf->NewCommand(); } catch (...) { return NO; }
    decimation::F2StoreRead staged;
    if (!document->StageF2DerivativeStore(before, {}, Standard_True, staged)
        || staged.occurrence != decimation::F2StoreOccurrence::Absent) {
        if (ocaf->HasOpenCommand()) ocaf->AbortCommand();
        SetError(error, Core3DLODSetErrorPersistenceUnavailable,
            @"F2 removal staging failed."); return NO;
    }
    if (!ocaf->CommitCommand() || ocaf->HasOpenCommand()
        || !document->FinalizeF2DerivativeHistory()) {
        SetError(error, Core3DLODSetErrorRecoveryRequired, @"F2 removal closure is uncertain.");
        return NO;
    }
    return YES;
}
#endif

@end
