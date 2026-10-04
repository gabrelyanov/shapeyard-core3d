#import "Core3DModelingTypes.h"
#import "Core3DViewController.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/NativeOpeningContext.hxx"
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/ReceiptRecord.hxx"
#include "../OCCTKit/RetainedFinishingProducer.hxx"
#include "Core3DViewer.h"

#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <atomic>
#include <cmath>
#include <memory>
#include <string>

namespace {
using FinishingDefinition = core3d::retained_finishing::Definition;
using FinishingPolicy = core3d::retained_finishing::UnwrapPolicy;
using FinishingQuality = core3d::retained_finishing::Quality;
using FinishingSettings = core3d::retained_finishing::producer::Settings;
using OpeningContext = core3d::native_opening::Context;
using OwnerKey = core3d::retained_recipe::OwnerKey;

enum class FinishingOpeningState : std::uint8_t {
    Open, Prepared, Applying, Regenerating, Cancelled, Settled, Recovery
};

struct ControllerFinishingInput final {
    Handle(OcctDocument) owner;
    std::shared_ptr<OpeningContext> context;
    OwnerKey key;
    std::string selected;
};

NSString *Hex(const core3d::retained_recipe::Digest& value) {
    static constexpr char digits[] = "0123456789abcdef";
    char output[65]{};
    for (std::size_t index = 0; index < value.size(); ++index) {
        output[index * 2] = digits[value[index] >> 4];
        output[index * 2 + 1] = digits[value[index] & 15];
    }
    return [NSString stringWithUTF8String:output] ?: @"";
}

NSString *PolicyText(FinishingPolicy value) {
    switch (value) {
        case FinishingPolicy::Planar: return @"planar";
        case FinishingPolicy::Cylindrical: return @"cylindrical";
        case FinishingPolicy::Toroidal: return @"toroidal";
        case FinishingPolicy::Conical: return @"conical";
        case FinishingPolicy::Spherical: return @"spherical";
        case FinishingPolicy::DiagnosedFallback: return @"diagnosedFallback";
    }
}

bool ParsePolicy(id value, FinishingPolicy& output) {
    if (![value isKindOfClass:NSString.class]) return false;
    if ([value isEqualToString:@"planar"]) output = FinishingPolicy::Planar;
    else if ([value isEqualToString:@"cylindrical"]) output = FinishingPolicy::Cylindrical;
    else if ([value isEqualToString:@"toroidal"]) output = FinishingPolicy::Toroidal;
    else if ([value isEqualToString:@"conical"]) output = FinishingPolicy::Conical;
    else if ([value isEqualToString:@"spherical"]) output = FinishingPolicy::Spherical;
    else if ([value isEqualToString:@"diagnosedFallback"])
        output = FinishingPolicy::DiagnosedFallback;
    else return false;
    return true;
}

bool Integer(id value, int minimum, int maximum, int& output) {
    if (![value isKindOfClass:NSNumber.class]
        || CFGetTypeID((__bridge CFTypeRef)value) == CFBooleanGetTypeID()) return false;
    const double scalar = [value doubleValue];
    if (!std::isfinite(scalar) || scalar < minimum || scalar > maximum
        || scalar != std::trunc(scalar)) return false;
    output = int(scalar); return true;
}

bool ExactCandidate(NSDictionary *candidate, FinishingSettings& output) {
    if (![candidate isKindOfClass:NSDictionary.class] || candidate.count != 3) return false;
    NSSet *expected = [NSSet setWithArray:@[
        @"unwrapPolicy", @"resolutionTexels", @"gutterTexels"]];
    if (![[NSSet setWithArray:candidate.allKeys] isEqualToSet:expected]
        || !ParsePolicy(candidate[@"unwrapPolicy"], output.requested)
        || !Integer(candidate[@"resolutionTexels"], 256, 4096, output.resolutionTexels)
        || !Integer(candidate[@"gutterTexels"], 1, 8, output.gutterTexels)
        || (output.resolutionTexels & (output.resolutionTexels - 1)) != 0) return false;
    return output.requested != FinishingPolicy::DiagnosedFallback;
}

bool KeyForSelected(const Handle(OcctDocument)& owner, const std::string& selected,
                    OwnerKey& key) noexcept {
    key = {};
    try {
        if (owner.IsNull() || selected.empty()) return false;
        const auto document = owner->Document();
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return false;
        TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
        TDF_Label match; unsigned count = 0;
        for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            if (owner->EntityIdentifierForLabel(roots.Value(index)) == selected) {
                match = roots.Value(index); ++count;
            }
        }
        return count == 1 && !match.IsNull()
            && core3d::receipt::ParseUUID(owner->DocumentIdentifier(), key.document)
            && core3d::receipt::ParseUUID(owner->EntityIdentifierForLabel(match), key.entity)
            && core3d::receipt::ParseUUID(owner->DefinitionIdentifierForLabel(match), key.definition)
            && core3d::retained_recipe::Valid(key);
    } catch (...) { key = {}; return false; }
}

bool CaptureControllerFinishingTarget(Core3DViewController *controller,
                                      NSString *entityIdentifier,
                                      ControllerFinishingInput& output) noexcept {
    output = {};
    try {
        GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)controller.glController : nil;
        if (!gl) return false;
        const std::shared_ptr<core3d::Core3DViewer> viewer = gl.viewer;
        const CGSize drawable = controller.viewportDrawableSize;
        if (!viewer || !std::isfinite(drawable.width) || !std::isfinite(drawable.height)
            || drawable.width < 1 || drawable.height < 1
            || drawable.width > UINT32_MAX || drawable.height > UINT32_MAX) return false;
        output.owner = viewer->getDocument();
        output.selected = entityIdentifier.UTF8String ?: "";
        if (!KeyForSelected(output.owner, output.selected, output.key)) return false;
        output.context = viewer->captureNativeOpeningContext(
            std::uint32_t(drawable.width), std::uint32_t(drawable.height), {output.selected});
        return output.context && output.context->isCurrent(64, 64);
    } catch (...) { output = {}; return false; }
}

bool CaptureControllerFinishingInput(Core3DViewController *controller,
                                     ControllerFinishingInput& output) noexcept {
    output = {};
    try {
        if (!NSThread.isMainThread || !controller) return false;
        Core3DSceneSnapshot *scene = [controller captureSceneSnapshot];
        if (!scene || scene.selection.selectedElements.count != 1) return false;
        Core3DSceneElementIdentifier *element = scene.selection.selectedElements.firstObject;
        if (element.kind != Core3DSceneElementKindObject
            || element.entityIdentifier.length == 0
            || element.entityIdentifier.length > 128) return false;
        return CaptureControllerFinishingTarget(controller, element.entityIdentifier, output);
    } catch (...) { output = {}; return false; }
}

bool CaptureControllerFinishingInputForIdentifier(Core3DViewController *controller,
                                                  NSString *entityIdentifier,
                                                  ControllerFinishingInput& output) noexcept {
    output = {};
    try {
        if (!NSThread.isMainThread || !controller
            || entityIdentifier.length == 0
            || entityIdentifier.length > 128) return false;
        return CaptureControllerFinishingTarget(controller, entityIdentifier, output);
    } catch (...) { output = {}; return false; }
}

Core3DRetainedFinishingCurrentness PublicCurrentness(
    OcctRetainedFinishingCurrentness value) noexcept {
    switch (value) {
        case OcctRetainedFinishingCurrentness::Current:
            return Core3DRetainedFinishingCurrentnessCurrent;
        case OcctRetainedFinishingCurrentness::Stale:
            return Core3DRetainedFinishingCurrentnessStale;
        case OcctRetainedFinishingCurrentness::Absent:
            return Core3DRetainedFinishingCurrentnessAbsent;
    }
}

NSString *CurrentnessText(OcctRetainedFinishingCurrentness value) {
    switch (value) {
        case OcctRetainedFinishingCurrentness::Current: return @"current";
        case OcctRetainedFinishingCurrentness::Stale: return @"stale";
        case OcctRetainedFinishingCurrentness::Absent: return @"absent";
    }
}

Core3DProfileConstructionResult FinishMutation(
    const std::shared_ptr<core3d::native_opening::CommandLease>& lease,
    OcctRetainedFinishingOutcome outcome) noexcept {
    if (!lease) return Core3DProfileConstructionResultBusy;
    if (outcome == OcctRetainedFinishingOutcome::Committed)
        return lease->commit() ? Core3DProfileConstructionResultCommitted
                               : Core3DProfileConstructionResultRecoveryRequired;
    if (!lease->abort()) return Core3DProfileConstructionResultRecoveryRequired;
    if (outcome == OcctRetainedFinishingOutcome::Busy)
        return Core3DProfileConstructionResultBusy;
    if (outcome == OcctRetainedFinishingOutcome::Absent)
        return Core3DProfileConstructionResultUnchanged;
    return Core3DProfileConstructionResultRejected;
}

void DeliverPreparation(void (^completion)(Core3DRetainedFinishingPreparationResult,
                                            NSString *),
                        Core3DRetainedFinishingPreparationResult result,
                        NSString *detail) {
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, detail);
    else dispatch_async(dispatch_get_main_queue(), ^{ completion(result, detail); });
}

void DeliverMutation(void (^completion)(Core3DProfileConstructionResult, NSString *),
                     Core3DProfileConstructionResult result, NSString *detail) {
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, detail);
    else dispatch_async(dispatch_get_main_queue(), ^{ completion(result, detail); });
}
} // namespace

@interface Core3DRetainedFinishingOpening () {
@package
    Handle(OcctDocument) _owner;
    std::shared_ptr<OpeningContext> _context;
    OwnerKey _key;
    std::string _selected;
    FinishingSettings _settings;
    FinishingDefinition _prepared;
    std::string _preparedDiagnosis;
    std::atomic<FinishingOpeningState> _state;
}
- (instancetype)initWithInput:(ControllerFinishingInput&&)input;
@end

@implementation Core3DRetainedFinishingOpening
- (instancetype)initWithInput:(ControllerFinishingInput&&)input {
    if ((self = [super init])) {
        _owner = input.owner; _context = std::move(input.context);
        _key = input.key; _selected = std::move(input.selected);
        _settings = {}; _state.store(FinishingOpeningState::Open);
    }
    return self;
}

- (Core3DRetainedFinishingCurrentness)currentness {
    if (!NSThread.isMainThread || _owner.IsNull())
        return Core3DRetainedFinishingCurrentnessStale;
    return PublicCurrentness(_owner->RetainedFinishingCurrentness(_key));
}

- (NSDictionary<NSString *, id> *)descriptor {
    if (!NSThread.isMainThread || _owner.IsNull()) return @{};
    try {
        const auto nativeCurrentness = _owner->RetainedFinishingCurrentness(_key);
        FinishingDefinition value;
        (void)_owner->RetainedFinishingCurrentness(_key, &value);
        core3d::retained_finishing::producer::Capture capture;
        if (!core3d::retained_finishing::producer::CaptureSource(
                _owner->Document(), _key, capture)) return @{};
        std::string diagnosis;
        if (!core3d::retained_recipe::Nonzero(value.finishing)) {
            TDF_Label label;
            if (!core3d::retained_finishing::owner::ResolveOwnerLabel(
                    _owner->Document(), _key, label)
                || core3d::retained_finishing::producer::BuildDerivative(
                    _owner->Document(), label, capture, _settings, value, diagnosis)
                    != core3d::retained_finishing::producer::Status::Produced) return @{};
        }
        if (value.quality == FinishingQuality::DiagnosedFallback && diagnosis.empty())
            diagnosis = "One or more source faces used the explicit planar fallback.";
        NSString *diagnosisText = [NSString stringWithUTF8String:diagnosis.c_str()] ?: @"";
        NSString *entity = [NSString stringWithUTF8String:_selected.c_str()] ?: @"";
        return @{@"schema": @"shapeyard.retained-finishing.opening.v1",
            @"ownerEntityIdentifier": entity,
            @"unwrapPolicy": PolicyText(value.unwrap),
            @"resolutionTexels": @(_settings.resolutionTexels),
            @"gutterTexels": @(_settings.gutterTexels),
            @"quality": value.quality == FinishingQuality::VerifiedChart
                ? @"verifiedChart" : @"diagnosedFallback",
            @"diagnosis": diagnosisText,
            @"currentness": CurrentnessText(nativeCurrentness),
            @"sourceDocumentGeneration": @(capture.source.documentGeneration),
            @"sourceModelRevision": @(capture.source.modelRevision),
            @"materialCount": @(value.materials.size()),
            @"assignmentCount": @(value.assignments.size()),
            @"finalCornerCount": @(value.finalCorners.size()),
            @"sourceDigest": Hex(capture.source.geometry),
            @"derivativeDigest": Hex(value.chartProof)};
    } catch (...) { return @{}; }
}

- (void)prepareCandidate:(NSDictionary<NSString *, id> *)candidate
              completion:(void (^)(Core3DRetainedFinishingPreparationResult,
                                   NSString *))completion {
    if (!NSThread.isMainThread || _state.load() != FinishingOpeningState::Open
        || _owner.IsNull() || !_context) {
        DeliverPreparation(completion, Core3DRetainedFinishingPreparationResultRejected,
                           @"Opening is not available for preparation.");
        return;
    }
    if (_owner->RetainedFinishingCurrentness(_key)
        == OcctRetainedFinishingCurrentness::Stale) {
        DeliverPreparation(completion, Core3DRetainedFinishingPreparationResultStaleSource,
                           @"Source changed. Regenerate from the current retained solid.");
        return;
    }
    FinishingSettings settings;
    if (!ExactCandidate(candidate, settings)) {
        DeliverPreparation(completion, Core3DRetainedFinishingPreparationResultRejected,
                           @"Use one supported unwrap policy, a power-of-two 256...4096 resolution, and 1...8 gutter texels.");
        return;
    }
    if (settings.requested == FinishingPolicy::Conical
        || settings.requested == FinishingPolicy::Spherical) {
        DeliverPreparation(completion,
            Core3DRetainedFinishingPreparationResultUnsupportedSurface,
            @"Conical and spherical verified-chart finishing is not supported.");
        return;
    }
    core3d::retained_finishing::producer::Capture capture;
    TDF_Label label; FinishingDefinition prepared; std::string diagnosis;
    if (!core3d::retained_finishing::producer::CaptureSource(
            _owner->Document(), _key, capture)
        || !core3d::retained_finishing::owner::ResolveOwnerLabel(
            _owner->Document(), _key, label)) {
        DeliverPreparation(completion, Core3DRetainedFinishingPreparationResultRejected,
                           @"The retained source could not be captured.");
        return;
    }
    const auto outcome = core3d::retained_finishing::producer::BuildDerivative(
        _owner->Document(), label, capture, settings, prepared, diagnosis);
    if (outcome == core3d::retained_finishing::producer::Status::StaleSource) {
        DeliverPreparation(completion, Core3DRetainedFinishingPreparationResultStaleSource,
                           @"Source changed during preparation.");
        return;
    }
    if (outcome != core3d::retained_finishing::producer::Status::Produced) {
        DeliverPreparation(completion, Core3DRetainedFinishingPreparationResultRejected,
                           @"Native retained-finishing preparation refused without history.");
        return;
    }
    FinishingOpeningState expected = FinishingOpeningState::Open;
    if (!_state.compare_exchange_strong(expected, FinishingOpeningState::Prepared)) {
        DeliverPreparation(completion, Core3DRetainedFinishingPreparationResultRejected,
                           @"Opening changed during preparation.");
        return;
    }
    _settings = settings; _prepared = std::move(prepared);
    _preparedDiagnosis = std::move(diagnosis);
    DeliverPreparation(completion, Core3DRetainedFinishingPreparationResultPrepared,
                       @"Retained finishing prepared from the current solid.");
}

- (void)applyWithCompletion:(void (^)(Core3DProfileConstructionResult,
                                      NSString *))completion {
    FinishingOpeningState expected = FinishingOpeningState::Prepared;
    if (!NSThread.isMainThread
        || !_state.compare_exchange_strong(expected, FinishingOpeningState::Applying)
        || _owner.IsNull() || !_context) {
        DeliverMutation(completion, Core3DProfileConstructionResultRejected,
                        @"No current retained-finishing preparation.");
        return;
    }
    const auto lease = _context->beginCommandLease(_context->openingFence(), 64, 64);
    const auto result = FinishMutation(lease,
        lease ? _owner->ProduceRetainedFinishing(_key,
            {_settings.requested == FinishingPolicy::Planar ? 0 : int(_settings.requested),
             _settings.resolutionTexels, _settings.gutterTexels})
              : OcctRetainedFinishingOutcome::Busy);
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? FinishingOpeningState::Recovery : FinishingOpeningState::Settled);
    if (result != Core3DProfileConstructionResultRecoveryRequired) _context.reset();
    DeliverMutation(completion, result,
        result == Core3DProfileConstructionResultCommitted
            ? @"Retained finishing committed as one undoable owner change."
            : result == Core3DProfileConstructionResultRecoveryRequired
                ? @"Command close is unknown; native recovery ownership is retained."
                : @"Retained finishing was refused without history.");
}

- (void)regenerateWithCompletion:(void (^)(Core3DProfileConstructionResult,
                                           NSString *))completion {
    FinishingOpeningState value = _state.load();
    while (value == FinishingOpeningState::Open || value == FinishingOpeningState::Prepared) {
        if (_state.compare_exchange_weak(value, FinishingOpeningState::Regenerating)) break;
    }
    if (!NSThread.isMainThread || _state.load() != FinishingOpeningState::Regenerating
        || _owner.IsNull() || !_context) {
        DeliverMutation(completion, Core3DProfileConstructionResultRejected,
                        @"Opening is not available for regeneration.");
        return;
    }
    const auto lease = _context->beginCommandLease(_context->openingFence(), 64, 64);
    const auto result = FinishMutation(lease,
        lease ? _owner->RegenerateRetainedFinishing(_key)
              : OcctRetainedFinishingOutcome::Busy);
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? FinishingOpeningState::Recovery : FinishingOpeningState::Settled);
    if (result != Core3DProfileConstructionResultRecoveryRequired) _context.reset();
    DeliverMutation(completion, result,
        result == Core3DProfileConstructionResultCommitted
            ? @"Finishing regenerated from the current retained solid as one undoable change."
            : result == Core3DProfileConstructionResultRecoveryRequired
                ? @"Command close is unknown; native recovery ownership is retained."
                : @"Regeneration was refused without reusing the stale derivative.");
}

- (BOOL)cancel {
    FinishingOpeningState value = _state.load();
    while (value == FinishingOpeningState::Open || value == FinishingOpeningState::Prepared) {
        if (_state.compare_exchange_weak(value, FinishingOpeningState::Cancelled)) {
            _prepared = {}; _preparedDiagnosis.clear(); _context.reset(); return YES;
        }
    }
    return NO;
}
@end

@implementation Core3DViewController (RetainedFinishingOpenings)
- (Core3DRetainedFinishingOpening *)openRetainedFinishingEditor {
    ControllerFinishingInput input;
    return CaptureControllerFinishingInput(self, input)
        ? [[Core3DRetainedFinishingOpening alloc] initWithInput:std::move(input)] : nil;
}

- (Core3DRetainedFinishingOpening *)openRetainedFinishingOpeningForEntityIdentifier:(NSString *)entityIdentifier {
    ControllerFinishingInput input;
    return CaptureControllerFinishingInputForIdentifier(self, entityIdentifier, input)
        ? [[Core3DRetainedFinishingOpening alloc] initWithInput:std::move(input)] : nil;
}

#if DEBUG
+ (NSData *)debugRetainedFinishingFixtureAssetData:(double)metersPerUnit {
    if (!NSThread.isMainThread || (metersPerUnit != 0.001 && metersPerUnit != 1.0))
        return nil;
    return [self debugB04EmptyDocumentFixtureDataWithMetersPerUnit:metersPerUnit];
}
#endif
@end
