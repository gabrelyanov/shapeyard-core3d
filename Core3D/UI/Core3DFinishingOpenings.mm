#import "Core3DModelingTypes.h"
#import "Core3DViewController.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/NativeOpeningContext.hxx"
#include "../OCCTKit/NativeOpeningDependentReplay.hxx"
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/FeaturePatternPersistence.hxx"
#include "../OCCTKit/PathArrayPersistence.hxx"
#include "../OCCTKit/PatternPersistence.hxx"
#include "../OCCTKit/ReceiptRecord.hxx"
#include "../OCCTKit/RetainedFinishingProducer.hxx"
#include "Core3DViewer.h"

#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

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

using ReplayFamily = core3d::dependent_replay::Family;
using ReplayUUID = core3d::dependent_replay::UUID;

struct ObservationCensus final {
    std::size_t supported = 0;
    std::size_t unsupported = 0;
};

struct ObservationFrontier final {
    ReplayUUID entity{};
    std::vector<ReplayUUID> ancestry;
};

bool ProductionSupports(ReplayFamily family) noexcept {
    switch (family) {
        case ReplayFamily::PatternD2:
        case ReplayFamily::PathArrayD3:
        case ReplayFamily::FeaturePatternD4:
            return true;
    }
    return false;
}

// Read-only mirror of R179DiscoverDependentClosure. The three persistence
// readers are the production readers, membership/dedupe/frontier rules match
// the document closure, and every budget/cycle/corruption failure refuses the
// whole observation rather than publishing partial or guessed counts.
bool CaptureObservationCensus(const Handle(TDocStd_Document)& document,
                              const OwnerKey& owner,
                              ObservationCensus& output) noexcept {
    output = {};
    try {
        if (document.IsNull() || document->GetData().IsNull()
            || !core3d::retained_recipe::Valid(owner)) return false;
        std::vector<core3d::pattern::Record> d2;
        std::vector<core3d::path_array::Record> d3;
        std::vector<core3d::feature_pattern::Record> d4;
        if (!core3d::pattern::ReadAll(document, d2)
            || !core3d::path_array::ReadAll(document, d3)
            || !core3d::feature_pattern::ReadAll(document, d4)) return false;

        std::deque<ObservationFrontier> frontier;
        frontier.push_back({owner.entity, {owner.entity}});
        std::set<ReplayUUID> expanded;
        std::set<std::pair<ReplayFamily, ReplayUUID>> features;
        std::size_t recordBytes = 0;
        constexpr std::size_t maximumRecords = 128;
        constexpr std::size_t maximumDocumentBytes = 8 * 1024 * 1024;
        const auto append = [&](ReplayFamily family, const ReplayUUID& feature,
                                const ReplayUUID& result,
                                const std::vector<std::uint8_t>& bytes,
                                const ObservationFrontier& current,
                                bool expandResult) -> bool {
            if (expandResult && result != current.entity
                && std::find(current.ancestry.begin(), current.ancestry.end(), result)
                    != current.ancestry.end()) return false;
            if (!features.insert({family, feature}).second) return true;
            if (features.size() > maximumRecords
                || bytes.size() > maximumDocumentBytes
                    - std::min(recordBytes, maximumDocumentBytes)) return false;
            recordBytes += bytes.size();
            if (ProductionSupports(family)) ++output.supported;
            else ++output.unsupported;
            if (expandResult && result != current.entity) {
                ObservationFrontier next{result, current.ancestry};
                next.ancestry.push_back(result);
                frontier.push_back(std::move(next));
            }
            return true;
        };

        while (!frontier.empty()) {
            ObservationFrontier current = std::move(frontier.front());
            frontier.pop_front();
            if (!expanded.insert(current.entity).second) continue;
            for (const auto& record : d2) {
                const auto& definition = record.definition;
                if (definition.owner.document != owner.document
                    || definition.source.document != owner.document) return false;
                const bool source = definition.source.entity == current.entity;
                const bool host = definition.owner.entity == current.entity;
                if ((source || host) && !append(ReplayFamily::PatternD2,
                        definition.feature, definition.owner.entity, record.bytes,
                        current, source)) return false;
            }
            for (const auto& record : d3) {
                const auto& definition = record.definition;
                if (definition.owner.document != owner.document
                    || definition.source.document != owner.document
                    || definition.path.owner.document != owner.document) return false;
                const bool source = definition.source.entity == current.entity;
                const bool path = definition.path.owner.entity == current.entity;
                const bool host = definition.owner.entity == current.entity;
                if ((source || path || host) && !append(ReplayFamily::PathArrayD3,
                        definition.feature, definition.owner.entity, record.bytes,
                        current, source || path)) return false;
            }
            for (const auto& record : d4) {
                const auto& definition = record.definition;
                if (definition.host.document != owner.document
                    || definition.sourceCut.document != owner.document) return false;
                const bool source = definition.sourceCut.entity == current.entity;
                const bool host = definition.host.entity == current.entity;
                if ((source || host) && !append(ReplayFamily::FeaturePatternD4,
                        definition.feature, definition.host.entity, record.bytes,
                        current, source)) return false;
            }
        }
        return output.supported + output.unsupported == features.size();
    } catch (...) { output = {}; return false; }
}

NSDictionary<NSString *, id> *CaptureObservation(
    ControllerFinishingInput& input) noexcept {
    try {
        if (input.owner.IsNull() || !input.context) return nil;
        const auto document = input.owner->Document();
        const auto currentness = input.owner->RetainedFinishingCurrentness(input.key);
        if (currentness == OcctRetainedFinishingCurrentness::Stale) return nil;

        FinishingDefinition value;
        if (input.owner->RetainedFinishingCurrentness(input.key, &value) != currentness)
            return nil;
        core3d::retained_finishing::producer::Capture capture;
        if (!core3d::retained_finishing::producer::CaptureSource(
                document, input.key, capture)) return nil;
        FinishingSettings settings;
        std::string diagnosis;
        if (!core3d::retained_recipe::Nonzero(value.finishing)) {
            TDF_Label label;
            if (!core3d::retained_finishing::owner::ResolveOwnerLabel(
                    document, input.key, label)
                || core3d::retained_finishing::producer::BuildDerivative(
                    document, label, capture, settings, value, diagnosis)
                    != core3d::retained_finishing::producer::Status::Produced) return nil;
        }

        ObservationCensus census;
        if (!CaptureObservationCensus(document, input.key, census)) return nil;
        core3d::retained_finishing::producer::Capture refenced;
        if (!input.context->isCurrent(64, 64)
            || input.owner->RetainedFinishingCurrentness(input.key) != currentness
            || !core3d::retained_finishing::producer::CaptureSource(
                document, input.key, refenced)
            || !(refenced.source == capture.source)
            || refenced.resourceManifest != capture.resourceManifest) return nil;

        NSString *entity = [NSString stringWithUTF8String:input.selected.c_str()] ?: @"";
        NSString *generation = [NSString stringWithUTF8String:
            std::to_string(capture.source.documentGeneration).c_str()] ?: @"";
        NSString *revision = [NSString stringWithUTF8String:
            std::to_string(capture.source.modelRevision).c_str()] ?: @"";
        return @{
            @"descriptorSchema": @"shapeyard.retained-finishing.observation.v2",
            @"ownerEntityIdentifier": entity,
            @"sourceDocumentGeneration": generation,
            @"sourceModelRevision": revision,
            @"sourceGeometryDigest": Hex(capture.source.geometry),
            @"sourceRecipeDigest": Hex(capture.source.recipe),
            @"sourceResourceDigest": Hex(capture.resourceManifest),
            @"currentness": CurrentnessText(currentness),
            @"quality": value.quality == FinishingQuality::VerifiedChart
                ? @"verifiedChart" : @"diagnosedFallback",
            @"unwrapPolicy": PolicyText(value.unwrap),
            @"resolutionTexels": @(settings.resolutionTexels),
            @"gutterTexels": @(settings.gutterTexels),
            @"supportedDescendantCount": @(census.supported),
            @"unsupportedDescendantCount": @(census.unsupported),
        };
    } catch (...) { return nil; }
}

#if DEBUG
ReplayUUID ObservationUUID() noexcept {
    ReplayUUID value{};
    [NSUUID.UUID getUUIDBytes:value.data()];
    return value;
}
#endif

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

- (NSDictionary<NSString *, id> *)captureRetainedFinishingObservationForEntityIdentifier:(NSString *)entityIdentifier {
    ControllerFinishingInput input;
    return CaptureControllerFinishingInputForIdentifier(self, entityIdentifier, input)
        ? CaptureObservation(input) : nil;
}

#if DEBUG
+ (NSData *)debugRetainedFinishingFixtureAssetData:(double)metersPerUnit {
    if (!NSThread.isMainThread || (metersPerUnit != 0.001 && metersPerUnit != 1.0))
        return nil;
    return [self debugAssetAtlasFixtureAssetData:metersPerUnit];
}

- (NSDictionary<NSString *, id> *)debugRetainedFinishingCaptureEvidenceForEntityIdentifier:(NSString *)entityIdentifier {
    ControllerFinishingInput input;
    if (!CaptureControllerFinishingInputForIdentifier(self, entityIdentifier, input)) return nil;
    core3d::retained_finishing::producer::Capture capture;
    if (!core3d::retained_finishing::producer::CaptureSource(
            input.owner->Document(), input.key, capture)) return nil;
    return @{
        @"sourceDocumentGeneration": [NSString stringWithUTF8String:
            std::to_string(capture.source.documentGeneration).c_str()] ?: @"",
        @"sourceModelRevision": [NSString stringWithUTF8String:
            std::to_string(capture.source.modelRevision).c_str()] ?: @"",
        @"sourceGeometryDigest": Hex(capture.source.geometry),
        @"sourceRecipeDigest": Hex(capture.source.recipe),
        @"sourceResourceDigest": Hex(capture.resourceManifest),
    };
}

- (BOOL)debugConfigureRetainedFinishingObservationForEntityIdentifier:(NSString *)entityIdentifier
                                                              scenario:(NSString *)scenario {
    if (!NSThread.isMainThread || ![scenario isKindOfClass:NSString.class]) return NO;
    GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)self.glController : nil;
    const std::shared_ptr<core3d::Core3DViewer> viewer = gl ? gl.viewer : nullptr;
    const Handle(OcctDocument) native = viewer ? viewer->getDocument() : Handle(OcctDocument)();
    OwnerKey key;
    if (native.IsNull() || !KeyForSelected(native, entityIdentifier.UTF8String ?: "", key))
        return NO;
    const auto document = native->Document();
    if (document.IsNull() || document->HasOpenCommand()) return NO;
    try {
        if ([scenario isEqualToString:@"dependentPattern"]) {
            const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
            if (shapes.IsNull()) return NO;
            TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
            OwnerKey result;
            for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
                const std::string identifier = native->EntityIdentifierForLabel(roots.Value(index));
                if (identifier != (entityIdentifier.UTF8String ?: "")
                    && KeyForSelected(native, identifier, result)) break;
                result = {};
            }
            if (!core3d::retained_recipe::Valid(result)) return NO;
            core3d::pattern::Definition definition;
            definition.owner = result;
            definition.feature = ObservationUUID();
            definition.source = {key.document, key.entity, key.definition, ObservationUUID()};
            definition.kind = core3d::pattern::Kind::Linear;
            definition.rowCount = 1; definition.columnCount = 2;
            definition.columnSpacing = 1;
            definition.issuance.nextLocalID = 3;
            definition.members = {
                {key.entity, 1, {0, 0}, core3d::pattern::MemberState::Active},
                {result.entity, 2, {0, 1}, core3d::pattern::MemberState::Active},
            };
            document->NewCommand();
            core3d::pattern::Record record;
            if (!core3d::pattern::Stage(document, definition, record)
                || !document->CommitCommand()) {
                if (document->HasOpenCommand()) document->AbortCommand();
                return NO;
            }
            return YES;
        }
        if ([scenario isEqualToString:@"stale"]) {
            TDF_Label ownerLabel;
            core3d::retained_finishing::Record record;
            if (!core3d::retained_finishing::owner::ResolveOwnerLabel(
                    document, key, ownerLabel)
                || !core3d::retained_finishing::Read(document, ownerLabel, record)
                || !record.value) return NO;
            auto payload = std::make_shared<core3d::retained_finishing::Payload>();
            payload->definition = record.value->definition;
            payload->definition.source.geometry[0] ^= 0x80;
            if (!core3d::retained_finishing::Encode(
                    payload->definition, payload->bytes)) return NO;
            document->NewCommand();
            if (!core3d::retained_finishing::Attribute::StageCommitted(
                    document, ownerLabel, payload)
                || !document->CommitCommand()) {
                if (document->HasOpenCommand()) document->AbortCommand();
                return NO;
            }
            return YES;
        }
        if ([scenario isEqualToString:@"corrupt"]) {
            document->NewCommand();
            const TDF_Label root = document->Main().FindChild(
                core3d::pattern::DocumentRootTag, Standard_True);
            TDataStd_AsciiString::Set(root, TCollection_AsciiString(
                "corrupt retained pattern table"));
            if (!document->CommitCommand()) {
                if (document->HasOpenCommand()) document->AbortCommand();
                return NO;
            }
            return YES;
        }
    } catch (...) {
        if (!document.IsNull() && document->HasOpenCommand()) document->AbortCommand();
    }
    return NO;
}
#endif
@end
