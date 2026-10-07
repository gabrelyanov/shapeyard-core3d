#import "Core3DFeaturePatternCreationOpening.h"
#import "GLViewController+Trick.h"
#import "Core3DViewer.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/FeaturePatternProfileContinuation.hxx"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>

namespace {
using namespace core3d;
enum class State : std::uint8_t {
    Open, Prepared, Applying, Cancelled, Settled, Recovery
};

NSString *Text(const std::string& value) {
    return [[NSString alloc] initWithBytes:value.data() length:value.size()
                                  encoding:NSUTF8StringEncoding] ?: @"";
}
NSString *UUIDText(const feature_pattern::UUID& value) {
    try { return Text(retained_solid::UUIDText(value)); }
    catch (...) { return @""; }
}
NSData *Data(const std::vector<std::uint8_t>& value) {
    return [NSData dataWithBytes:value.data() length:value.size()];
}
NSString *Hex(const feature_pattern_child::Digest& value) {
    static constexpr char digits[] = "0123456789abcdef";
    std::string result; result.reserve(value.size() * 2);
    for (std::uint8_t byte : value) {
        result.push_back(digits[byte >> 4]);
        result.push_back(digits[byte & 0x0f]);
    }
    return Text(result);
}
bool Number(id value, double& output) {
    if (![value isKindOfClass:NSNumber.class]
        || CFGetTypeID((__bridge CFTypeRef)value) == CFBooleanGetTypeID())
        return false;
    output = [value doubleValue]; return std::isfinite(output);
}
bool Integer(id value, std::uint64_t maximum, std::uint64_t& output) {
    double scalar = 0;
    if (!Number(value, scalar) || scalar < 0 || scalar > double(maximum)
        || std::trunc(scalar) != scalar) return false;
    output = [value unsignedLongLongValue];
    return output <= maximum && double(output) == scalar;
}
bool Vector3(id value, std::array<double, 3>& output) {
    if (![value isKindOfClass:NSArray.class] || [value count] != 3) return false;
    for (NSUInteger index = 0; index < 3; ++index)
        if (!Number(value[index], output[index])) return false;
    return true;
}
bool ParseKind(id value, pattern::Kind& output) {
    if (![value isKindOfClass:NSString.class]) return false;
    if ([value isEqualToString:@"linear"]) output = pattern::Kind::Linear;
    else if ([value isEqualToString:@"radial"]) output = pattern::Kind::Radial;
    else if ([value isEqualToString:@"grid"]) output = pattern::Kind::Grid;
    else return false;
    return true;
}
bool ParseAxis(id value, pattern::Axis& output) {
    std::uint64_t axis = 0;
    if (!Integer(value, 2, axis)) return false;
    output = pattern::Axis(axis); return true;
}
bool ExactKeys(NSDictionary *value, NSArray<NSString *> *keys) {
    return [value isKindOfClass:NSDictionary.class] && value.count == keys.count
        && [[NSSet setWithArray:value.allKeys]
            isEqualToSet:[NSSet setWithArray:keys]];
}
void DeliverPrepared(void (^completion)(Core3DBoundedCurvePreparationResult,
                                         NSString *),
                     Core3DBoundedCurvePreparationResult result,
                     NSString *detail) {
    if (completion) completion(result, detail);
}
void DeliverCreated(void (^completion)(Core3DProfileConstructionResult,
                                        NSString *, NSString *),
                    Core3DProfileConstructionResult result,
                    NSString *detail, NSString *host) {
    if (completion) completion(result, detail, host);
}

struct OpeningCapture {
    Handle(OcctDocument) owner;
    std::shared_ptr<Core3DViewer> viewer;
    std::string host;
    std::uint32_t width = 0, height = 0;
    double metersPerUnit = 0;
    NSMutableArray<NSDictionary<NSString *, id> *> *choices = nil;
    std::set<std::pair<std::string, std::uint64_t>> admittedChoices;
};

bool CaptureOpening(Core3DViewController *controller,
                    OpeningCapture& output) noexcept {
    output = {};
    try {
        if (!NSThread.isMainThread || !controller) return false;
        Core3DSceneSnapshot *scene = controller.captureSceneSnapshot;
        if (!scene || scene.selection.selectedElements.count != 1) return false;
        Core3DSceneElementIdentifier *selected = scene.selection.selectedElements.firstObject;
        if (selected.kind != Core3DSceneElementKindObject
            || selected.entityIdentifier.length == 0
            || selected.entityIdentifier.length > 128) return false;
        GLViewController *gl = [controller.glController
            isKindOfClass:GLViewController.class]
            ? (GLViewController *)controller.glController : nil;
        const CGSize size = controller.viewportDrawableSize;
        if (!gl || !gl.viewer || !std::isfinite(size.width)
            || !std::isfinite(size.height) || size.width < 1 || size.height < 1
            || size.width > UINT32_MAX || size.height > UINT32_MAX) return false;
        OpeningCapture capture;
        capture.viewer = gl.viewer; capture.owner = capture.viewer->getDocument();
        capture.host = selected.entityIdentifier.UTF8String ?: "";
        capture.width = std::uint32_t(size.width);
        capture.height = std::uint32_t(size.height);
        if (capture.owner.IsNull() || capture.host.empty()) return false;
        const auto document = capture.owner->Document();
        if (document.IsNull() || document->HasOpenCommand()) return false;
        feature_pattern::UUID hostID{};
        if (!pattern_owner::Parse(capture.host, hostID)) return false;
        std::map<feature_pattern::UUID, pattern_owner::LabelReceipt> labels;
        if (!feature_pattern_owner::LocateLabels(*capture.owner, document, labels))
            return false;
        const auto host = labels.find(hostID);
        profile::Record profileRecord;
        if (host == labels.end()
            || !profile::Read(document, host->second.label, profileRecord)
            || !profileRecord.IsCurrent(document, host->second.label)) return false;
        capture.metersPerUnit = profileRecord.parameters.metersPerUnit;
        if (!std::isfinite(capture.metersPerUnit)
            || capture.metersPerUnit <= 0) return false;
        capture.choices = [NSMutableArray array];
        for (const auto& entry : labels) {
            if (entry.first == hostID) continue;
            OcctCylindricalCutProgramSource source;
            if (!capture.owner->CaptureCylindricalCutProgramSource(
                    entry.second.label, source)) continue;
            const auto *program = std::get_if<retained_boolean::Program>(&source.recipe);
            if (!program || !retained_boolean::Valid(*program)
                || !program->filletSteps.empty()) continue;
            for (const auto& step : program->steps) {
                if (step.operation != analytic_boolean::Operation::Difference
                    || step.operand.kind != analytic_boolean::OperandKind::Cylinder)
                    continue;
                const std::string sourceText = retained_solid::UUIDText(entry.first);
                const auto key = std::make_pair(sourceText,
                    std::uint64_t(step.operand.identifier));
                if (!capture.admittedChoices.insert(key).second) return false;
                const double mm = program->source.metersPerUnit * 1000.0;
                [capture.choices addObject:@{
                    @"sourceEntityIdentifier": Text(sourceText),
                    @"sourceDefinitionIdentifier": UUIDText(entry.second.definition),
                    @"sourceCutStepID": @(step.operand.identifier),
                    @"axis": @(unsigned(step.operand.axis)),
                    @"radiusMM": @(step.operand.radius * mm),
                    @"pointMM": @[@(step.operand.point[0] * mm),
                                   @(step.operand.point[1] * mm),
                                   @(step.operand.point[2] * mm)]
                }];
            }
        }
        if (capture.choices.count == 0) return false;
        output = std::move(capture); return true;
    } catch (...) { output = {}; return false; }
}

bool ParseCandidate(NSDictionary *candidate, const OpeningCapture& opening,
                    std::string& source, profile_d4::CreationEdit& edit) {
    if (!ExactKeys(candidate, @[@"sourceEntityIdentifier", @"sourceCutStepID",
            @"kind", @"rowAxis", @"columnAxis", @"rowCount",
            @"columnCount", @"rowSpacingMM", @"columnSpacingMM",
            @"radialPivotMM", @"sweepDegrees"])) return false;
    id sourceValue = candidate[@"sourceEntityIdentifier"];
    if (![sourceValue isKindOfClass:NSString.class]
        || [sourceValue length] == 0 || [sourceValue length] > 128) return false;
    source = [sourceValue UTF8String] ?: "";
    std::uint64_t step = 0, rows = 0, columns = 0;
    double rowSpacing = 0, columnSpacing = 0, sweep = 0;
    std::array<double, 3> pivot{};
    if (!Integer(candidate[@"sourceCutStepID"], UINT32_MAX, step)
        || !ParseKind(candidate[@"kind"], edit.kind)
        || !ParseAxis(candidate[@"rowAxis"], edit.rowAxis)
        || !ParseAxis(candidate[@"columnAxis"], edit.columnAxis)
        || !Integer(candidate[@"rowCount"], UINT32_MAX, rows)
        || !Integer(candidate[@"columnCount"], UINT32_MAX, columns)
        || !Number(candidate[@"rowSpacingMM"], rowSpacing)
        || !Number(candidate[@"columnSpacingMM"], columnSpacing)
        || !Vector3(candidate[@"radialPivotMM"], pivot)
        || !Number(candidate[@"sweepDegrees"], sweep)
        || !opening.admittedChoices.count({source, step})) return false;
    const double mmPerUnit = opening.metersPerUnit * 1000.0;
    edit.sourceCutStepID = step; edit.rows = std::uint32_t(rows);
    edit.columns = std::uint32_t(columns);
    edit.rowSpacing = rowSpacing / mmPerUnit;
    edit.columnSpacing = columnSpacing / mmPerUnit;
    edit.sweepRadians = sweep * std::acos(-1.0) / 180.0;
    for (unsigned index = 0; index < 3; ++index)
        edit.radialPivotLocal[index] = pivot[index] / mmPerUnit;
    return true;
}

NSDictionary *ReviewDictionary(const profile_d4::PreparedReview& review,
                               double metersPerUnit) {
    NSMutableArray *removed = [NSMutableArray arrayWithCapacity:
        review.positiveRemovedVolumes.size()];
    const double mm = metersPerUnit * 1000.0;
    const double volumeScale = mm * mm * mm;
    for (double value : review.positiveRemovedVolumes)
        [removed addObject:@(value * volumeScale)];
    return @{
        @"schema": @"shapeyard.d4-feature-pattern-review.v1",
        @"attributedChildCount": @(review.attributedChildCount),
        @"sectionCount": @(review.sectionCount),
        @"positiveRemovedVolumesMM3": removed,
        @"measuredPairwiseLigamentMM": @(review.measuredPairwiseLigamentMM),
        @"activeFeatures": @(review.chargedProjection.activeFeatures),
        @"projectedTopologyNodes": @(review.chargedProjection.projectedTopologyNodes),
        @"projectedDocumentBytes": @(review.chargedProjection.projectedDocumentBytes),
        @"projectedMemoryBytes": @(review.chargedProjection.projectedMemoryBytes)
    };
}
} // namespace

@interface Core3DFeaturePatternCreationOpening () {
@package
    OpeningCapture _opening;
    std::shared_ptr<native_opening::Context> _context;
    std::shared_ptr<const profile_d4::CreationCapture> _capture;
    std::shared_ptr<const profile_d4::PreparedCreation> _prepared;
    std::shared_ptr<profile_d4::PendingCreation> _pending;
    std::shared_ptr<std::atomic_bool> _stop;
    std::atomic<State> _state;
    NSDictionary *_review;
}
- (instancetype)initWithCapture:(OpeningCapture)capture;
@end

@implementation Core3DFeaturePatternCreationOpening
- (instancetype)initWithCapture:(OpeningCapture)capture {
    if ((self = [super init])) {
        _opening = std::move(capture); _state.store(State::Open);
        _stop = std::make_shared<std::atomic_bool>(false); _review = @{};
    }
    return self;
}
- (NSDictionary *)descriptor {
    if (_state.load() == State::Cancelled) return @{};
    return @{@"schema": @"shapeyard.d4-feature-pattern-creation.v1",
        @"hostEntityIdentifier": Text(_opening.host),
        @"documentMetersPerUnit": @(_opening.metersPerUnit),
        @"sourceChoices": [_opening.choices copy] ?: @[],
        @"maximumMembers": @(feature_pattern::MaximumGeneratedFeatures),
        @"minimumHostLigamentMM": @(.002),
        @"expectedBoundarySectionsPerFeature": @2};
}
- (NSDictionary *)review { return [_review copy] ?: @{}; }
- (void)prepareCandidate:(NSDictionary *)candidate
    completion:(void (^)(Core3DBoundedCurvePreparationResult, NSString *))completion {
    const State state = _state.load();
    if (state != State::Open && state != State::Prepared) {
        DeliverPrepared(completion, Core3DBoundedCurvePreparationResultRejected,
                        @"Creation opening is no longer available."); return;
    }
    _stop->store(true); _prepared.reset(); _capture.reset(); _context.reset();
    _stop = std::make_shared<std::atomic_bool>(false); _review = @{};
    std::string source; profile_d4::CreationEdit edit;
    if (!ParseCandidate(candidate, _opening, source, edit)) {
        _state.store(State::Open);
        DeliverPrepared(completion, Core3DBoundedCurvePreparationResultRejected,
                        @"Choose a current native source step and valid pattern values.");
        return;
    }
    _context = _opening.viewer->captureNativeOpeningContext(
        _opening.width, _opening.height, {_opening.host, source});
    if (!_context || profile_d4::CaptureCreationHost(*_opening.owner,
            _opening.host, source, _context, _opening.width, _opening.height,
            _capture) != profile_d4::CaptureStatus::Current) {
        _state.store(State::Open);
        DeliverPrepared(completion, Core3DBoundedCurvePreparationResultRejected,
                        @"The host, source, selection or document changed. Reopen creation.");
        return;
    }
    _prepared = profile_d4::PrepareCreation(_capture, edit, *_stop);
    profile_d4::PreparedReview review;
    if (!_prepared || !profile_d4::ReviewPreparedCreation(*_prepared, review)) {
        _state.store(State::Open); _prepared.reset(); _capture.reset();
        _context.reset();
        DeliverPrepared(completion, Core3DBoundedCurvePreparationResultRejected,
                        @"Native feature-pattern admission refused the candidate.");
        return;
    }
    _review = ReviewDictionary(review, _opening.metersPerUnit);
    _state.store(State::Prepared);
    DeliverPrepared(completion, Core3DBoundedCurvePreparationResultPrepared,
                    @"Review the retained native measurements before Apply.");
}
- (void)finishCreationOutcome:(profile_d4::CreationOutcome)outcome
    completion:(void (^)(Core3DProfileConstructionResult, NSString *, NSString *))completion {
    Core3DProfileConstructionResult result = Core3DProfileConstructionResultRejected;
    if (outcome == profile_d4::CreationOutcome::Cancelled)
        result = Core3DProfileConstructionResultCancelled;
    else if (outcome == profile_d4::CreationOutcome::Committed)
        result = Core3DProfileConstructionResultCommitted;
    else if (outcome == profile_d4::CreationOutcome::OutcomeUnknown)
        result = Core3DProfileConstructionResultRecoveryRequired;
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? State::Recovery : State::Settled);
    NSString *host = result == Core3DProfileConstructionResultCommitted
        ? Text(_opening.host) : nil;
    if (result != Core3DProfileConstructionResultRecoveryRequired) {
        _pending.reset(); _prepared.reset(); _capture.reset(); _context.reset();
    }
    DeliverCreated(completion, result,
        result == Core3DProfileConstructionResultCommitted
            ? @"Feature pattern created as one native history command."
            : result == Core3DProfileConstructionResultCancelled
                ? @"Stopped. The model is unchanged."
                : result == Core3DProfileConstructionResultRecoveryRequired
                    ? @"Creation outcome is unknown; native recovery ownership is retained."
                    : @"Creation was refused without a committed partial result.", host);
}
- (void)applyWithCompletion:
    (void (^)(Core3DProfileConstructionResult, NSString *, NSString *))completion {
    State expected = State::Prepared;
    if (!_state.compare_exchange_strong(expected, State::Applying)) {
        DeliverCreated(completion, Core3DProfileConstructionResultRejected,
                       @"No current prepared creation.", nil); return;
    }
    profile_d4::CreationOutcome immediate = profile_d4::CreationOutcome::Refused;
    _pending = profile_d4::StartCreation(*_opening.owner, _prepared, immediate);
    if (!_pending) { [self finishCreationOutcome:immediate completion:completion]; return; }
    // Return the main run loop to event processing with the exact command lease
    // still open at AfterLastChild. Stop can now reach that lease's abort path.
    [NSRunLoop.mainRunLoop performBlock:^{
        const auto outcome = profile_d4::FinishCreation(self->_pending, *self->_stop);
        [self finishCreationOutcome:outcome completion:completion];
    }];
}
- (BOOL)requestStop {
    if (!NSThread.isMainThread || _state.load() != State::Applying || !_pending)
        return NO;
    _stop->store(true); return YES;
}
- (BOOL)cancel {
    State value = _state.load();
    while (value == State::Open || value == State::Prepared) {
        if (_state.compare_exchange_weak(value, State::Cancelled)) {
            _stop->store(true); _prepared.reset(); _capture.reset();
            _context.reset(); _review = @{}; return YES;
        }
    }
    return NO;
}
#if DEBUG
- (void)debugArmCreationFault:(NSInteger)fault {
    if (_state.load() != State::Prepared) return;
    if (fault == 1) profile_d4::DebugArmCreationFault(
        profile_d4::CreationFault::AfterLastChild);
    else if (fault == 2) profile_d4::DebugArmCreationFault(
        profile_d4::CreationFault::CreationReadback);
    else profile_d4::DebugArmCreationFault(profile_d4::CreationFault::None);
}
#endif
@end

@implementation Core3DViewController (FeaturePatternCreationOpening)
- (Core3DFeaturePatternCreationOpening *)beginFeaturePatternCreation {
    OpeningCapture capture;
    if (!CaptureOpening(self, capture)) return nil;
    return [[Core3DFeaturePatternCreationOpening alloc]
        initWithCapture:std::move(capture)];
}
#if DEBUG
- (NSDictionary<NSString *, id> *)debugFeaturePatternCreationEvidenceForEntityIdentifier:
    (NSString *)entityIdentifier {
    if (!NSThread.isMainThread || entityIdentifier.length == 0
        || !GLController || !GLController.viewer) return nil;
    try {
        const auto viewer = GLController.viewer;
        const auto owner = viewer->getDocument();
        const CGSize size = self.viewportDrawableSize;
        const std::string host = entityIdentifier.UTF8String ?: "";
        if (owner.IsNull() || host.empty() || size.width < 1 || size.height < 1)
            return nil;
        auto context = viewer->captureNativeOpeningContext(
            std::uint32_t(size.width), std::uint32_t(size.height), {host});
        profile_d4::Observation value;
        if (!context || !profile_d4::ObserveCurrent(*owner, host, context, value))
            return @{@"current": @NO};
        const double mm = value.metersPerUnit * 1000.0;
        return @{@"schema": @"shapeyard.d4-feature-pattern-evidence.v1",
            @"current": @(value.current), @"hostEntity": Text(value.hostEntity),
            @"sourceEntity": Text(value.sourceEntity),
            @"profileFeature": Text(value.profileFeature),
            @"patternFeature": Text(value.patternFeature),
            @"sourceCutStepID": @(value.sourceCutStepID),
            @"memberCount": @(value.memberCount),
            @"childCount": @(value.childCount),
            @"sourceProgramBytes": @(value.sourceProgramBytes),
            @"metersPerUnit": @(value.metersPerUnit),
            @"baselineVolumeMM3": @(value.baselineVolume * mm * mm * mm),
            @"resultVolumeMM3": @(value.resultVolume * mm * mm * mm),
            @"sourceVolumeMM3": @(value.sourceVolume * mm * mm * mm)};
    } catch (...) { return nil; }
}

- (NSDictionary<NSString *, id> *)debugFeaturePatternReceiptEvidenceForEntityIdentifier:
    (NSString *)entityIdentifier {
    if (!NSThread.isMainThread || entityIdentifier.length == 0
        || !GLController || !GLController.viewer) return nil;
    try {
        const auto viewer = GLController.viewer;
        const auto owner = viewer->getDocument();
        const CGSize size = self.viewportDrawableSize;
        const std::string host = entityIdentifier.UTF8String ?: "";
        if (owner.IsNull() || host.empty() || size.width < 1 || size.height < 1)
            return nil;
        auto context = viewer->captureNativeOpeningContext(
            std::uint32_t(size.width), std::uint32_t(size.height), {host});
        profile_d4::ReceiptObservation value;
        if (!context || !profile_d4::ObserveReceipts(*owner, host, context, value))
            return @{@"current": @NO};
        NSMutableArray<NSDictionary<NSString *, id> *> *children =
            [NSMutableArray arrayWithCapacity:value.children.size()];
        const double mm = value.metersPerUnit * 1000.0;
        const double volumeScale = mm * mm * mm;
        for (const auto& child : value.children) {
            NSMutableArray<NSDictionary<NSString *, id> *> *selectors =
                [NSMutableArray arrayWithCapacity:child.selectors.size()];
            for (const auto& selector : child.selectors)
                [selectors addObject:@{@"kind": @(selector.kind),
                    @"semantic": UUIDText(selector.semantic),
                    @"ordinal": @(selector.ordinal),
                    @"proof": Hex(selector.proof)}];
            [children addObject:@{
                @"childFeatureIdentifier": UUIDText(child.childFeature),
                @"instanceIdentifier": UUIDText(child.instanceIdentity),
                @"baselineRecipeIdentity": UUIDText(
                    child.baselineRecipeIdentity),
                @"localID": @(child.localID), @"row": @(child.row),
                @"column": @(child.column),
                @"boundarySections": @(child.boundarySections),
                @"canonicalBytes": Data(child.canonicalBytes),
                @"selectors": selectors,
                @"resolvesOnCurrentBRep": @(child.resolvesOnCurrentBRep),
                @"buildStatus": @(child.buildStatus),
                @"positiveRemovedVolumeMM3": @(
                    child.positiveRemovedVolume * volumeScale),
                @"orientedBoundarySections": @(
                    child.orientedBoundarySections)}];
        }
        return @{@"schema": @"shapeyard.d4-feature-pattern-receipts.v1",
            @"current": @(value.current), @"hostEntity": Text(value.hostEntity),
            @"patternFeature": UUIDText(value.patternFeature),
            @"metersPerUnit": @(value.metersPerUnit),
            @"baseline": @{
                @"retainedRecipeFeature": UUIDText(value.retainedRecipeFeature),
                @"baselineRecipeIdentity": UUIDText(
                    value.baselineRecipeIdentity),
                @"exactRecipe": Data(value.exactRecipe),
                @"canonicalBytes": Data(value.baselineCanonicalBytes)},
            @"children": children,
            @"allReceiptsResolve": @(value.allReceiptsResolve)};
    } catch (...) { return nil; }
}
#endif
@end
