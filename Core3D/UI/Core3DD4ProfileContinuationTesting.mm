#if DEBUG
#import "Core3DViewController.h"
#import "Core3DD4ProfileContinuationTesting.h"
#import "GLViewController.h"

#include "Core3DViewer.h"
#include "../OCCTKit/FeaturePatternDefinition.hxx"
#include "../OCCTKit/FeaturePatternProfileContinuation.hxx"
#include "../OCCTKit/NativeOpeningDependentReplay.hxx"
#include "../OCCTKit/RetainedBooleanProgram.hxx"
#include "../OCCTKit/RetainedEdgeTreatmentSnapshot.hxx"

#include <cmath>
#include <thread>

namespace {
using namespace core3d;

bool Inputs(Core3DViewController *controller, NSString *host,
            std::shared_ptr<Core3DViewer>& viewer, Handle(OcctDocument)& owner,
            std::shared_ptr<native_opening::Context>& context,
            std::uint32_t& width, std::uint32_t& height) {
    if (!NSThread.isMainThread || !controller || !host || host.length == 0
        || host.length > 128) return false;
    GLViewController *gl = [controller.glController
        isKindOfClass:GLViewController.class]
        ? (GLViewController *)controller.glController : nil;
    const CGSize size = controller.viewportDrawableSize;
    if (!gl || !gl.viewer || !std::isfinite(size.width)
        || !std::isfinite(size.height) || size.width < 1 || size.height < 1
        || size.width > UINT32_MAX || size.height > UINT32_MAX) return false;
    viewer = gl.viewer; owner = viewer->getDocument();
    width = std::uint32_t(size.width); height = std::uint32_t(size.height);
    const char *text = host.UTF8String;
    if (owner.IsNull() || !text) return false;
    context = viewer->captureNativeOpeningContext(width, height, {text});
    return bool(context);
}

NSDictionary *Observe(const Handle(OcctDocument)& owner, NSString *host,
                      const std::shared_ptr<native_opening::Context>& context) {
    const char *text = host.UTF8String;
    if (!text) return @{@"current": @NO};
    profile_d4::Observation observed;
    if (!context || !profile_d4::ObserveCurrent(
            *owner, text, context, observed)) return @{@"current": @NO};
    const double millimetresPerUnit = observed.metersPerUnit * 1000.0;
    const double volumeScale = millimetresPerUnit * millimetresPerUnit
        * millimetresPerUnit;
    return @{
        @"schema": @"shapeyard.d4-profile-observation.v1",
        @"current": @(observed.current),
        @"hostEntity": [NSString stringWithUTF8String:observed.hostEntity.c_str()],
        @"sourceEntity": [NSString stringWithUTF8String:observed.sourceEntity.c_str()],
        @"profileFeature": [NSString stringWithUTF8String:observed.profileFeature.c_str()],
        @"patternFeature": [NSString stringWithUTF8String:observed.patternFeature.c_str()],
        @"sourceCutStepID": @(observed.sourceCutStepID),
        @"memberCount": @(observed.memberCount),
        @"childCount": @(observed.childCount),
        @"profileScalarCount": @(observed.profileScalarCount),
        @"sourceProgramBytes": @(observed.sourceProgramBytes),
        @"metersPerUnit": @(observed.metersPerUnit),
        @"baselineVolumeMM3": @(observed.baselineVolume * volumeScale),
        @"resultVolumeMM3": @(observed.resultVolume * volumeScale),
        @"sourceVolumeMM3": @(observed.sourceVolume * volumeScale),
    };
}
} // namespace

static NSDictionary *DebugCreate(Core3DViewController *controller,
                                 NSString *host, NSString *source,
                                 int32_t fault = 0) {
    @autoreleasepool {
        try {
            std::shared_ptr<core3d::Core3DViewer> viewer;
            Handle(OcctDocument) owner;
            std::shared_ptr<core3d::native_opening::Context> ignored;
            std::uint32_t width = 0, height = 0;
            if (!source || source.length == 0 || source.length > 128
                || !Inputs(controller, host, viewer, owner, ignored, width, height))
                return @{@"committed": @NO, @"phase": @"inputs"};
            const char *hostText = host.UTF8String, *sourceText = source.UTF8String;
            if (!hostText || !sourceText) return @{@"committed": @NO, @"phase": @"text"};
            ignored.reset();
            auto context = viewer->captureNativeOpeningContext(
                width, height, {hostText, sourceText});
            std::shared_ptr<const core3d::profile_d4::CreationCapture> capture;
            if (!context || core3d::profile_d4::CaptureCreationHost(*owner,
                    hostText, sourceText, context, width, height, capture)
                    != core3d::profile_d4::CaptureStatus::Current)
                return @{@"committed": @NO, @"phase": @"capture"};
            core3d::profile_d4::CreationEdit edit;
            const double unit = context->openingFence().metersPerUnit();
            edit.columnSpacing = 15.0 * 0.001 / unit;
            std::atomic_bool stop{false};
            auto prepared = core3d::profile_d4::PrepareCreation(
                capture, edit, stop);
            if (!prepared) return @{@"committed": @NO, @"phase": @"prepare"};
            const int beforeUndo = owner->Document()->GetAvailableUndos();
            const int beforeRedo = owner->Document()->GetAvailableRedos();
            if (fault == 1) {
                core3d::profile_d4::DebugArmCreationFault(
                    core3d::profile_d4::CreationFault::AfterLastChild);
            } else if (fault == 2) {
                core3d::profile_d4::DebugArmCreationFault(
                    core3d::profile_d4::CreationFault::CreationReadback);
            } else if (fault != 0 && fault != 3) {
                return @{@"committed": @NO, @"phase": @"fault-input"};
            }
            if (fault == 3) context->debugReportNextCloseUnproven();
            const auto outcome = core3d::profile_d4::StageCreation(*owner, prepared);
            if (outcome != core3d::profile_d4::CreationOutcome::Committed) {
                const bool unresolved = viewer->hasUnresolvedOrdinaryEdit();
                const bool closed = !owner->Document()->HasOpenCommand();
                const int afterUndo = owner->Document()->GetAvailableUndos();
                const int afterRedo = owner->Document()->GetAvailableRedos();
                prepared.reset(); capture.reset();
                auto blockedProbe = viewer->captureNativeOpeningContext(
                    width, height, {hostText, sourceText});
                const bool recoveryRequired = !blockedProbe;
                blockedProbe.reset();
                const bool recovered = outcome
                        == core3d::profile_d4::CreationOutcome::OutcomeUnknown
                    && context->reconcileRecovery(true);
                context.reset();
                auto recaptured = viewer->captureNativeOpeningContext(
                    width, height, {hostText, sourceText});
                bool absent = false;
                if (recaptured) {
                    std::shared_ptr<const core3d::profile_d4::CreationCapture> retry;
                    absent = core3d::profile_d4::CaptureCreationHost(*owner,
                        hostText, sourceText, recaptured, width, height, retry)
                        == core3d::profile_d4::CaptureStatus::Current;
                }
                return @{@"committed": @NO, @"phase": @"stage",
                    @"outcome": @(unsigned(outcome)), @"fault": @(fault),
                    @"closed": @(closed), @"unresolved": @(unresolved),
                    @"newOpeningBlocked": @(recoveryRequired),
                    @"recovered": @(recovered),
                    @"openingAfterRecovery": @(bool(recaptured)),
                    @"creationAbsent": @(absent),
                    @"undoBefore": @(beforeUndo), @"undoAfter": @(afterUndo),
                    @"redoBefore": @(beforeRedo), @"redoAfter": @(afterRedo)};
            }
            // Publication is terminal for the opening authority. The sealed
            // prepared value retains its capture, which in turn retains the
            // context, so retire the full chain before observing the committed
            // scene through a fresh single context.
            prepared.reset(); capture.reset(); context.reset();
            auto observed = viewer->captureNativeOpeningContext(
                width, height, {hostText});
            NSMutableDictionary *result = [Observe(
                owner, host, observed) mutableCopy];
            result[@"committed"] = @YES; result[@"phase"] = @"complete";
            return result;
        } catch (...) { return @{@"committed": @NO, @"phase": @"exception"}; }
    }
}

static NSDictionary *DebugObserve(Core3DViewController *controller,
                                  NSString *host) {
    @autoreleasepool {
        std::shared_ptr<core3d::Core3DViewer> viewer; Handle(OcctDocument) owner;
        std::shared_ptr<core3d::native_opening::Context> context;
        std::uint32_t width = 0, height = 0;
        if (!Inputs(controller, host, viewer, owner, context, width, height))
            return @{@"current": @NO};
        return Observe(owner, host, context);
    }
}

static NSDictionary *DebugCreationStop(Core3DViewController *controller,
                                       NSString *host, NSString *source) {
    @autoreleasepool {
        try {
            std::shared_ptr<core3d::Core3DViewer> viewer; Handle(OcctDocument) owner;
            std::shared_ptr<core3d::native_opening::Context> ignored;
            std::uint32_t width = 0, height = 0;
            if (!source || !Inputs(controller, host, viewer, owner,
                                  ignored, width, height))
                return @{@"cancelled": @NO, @"phase": @"inputs"};
            const char *hostText = host.UTF8String, *sourceText = source.UTF8String;
            if (!hostText || !sourceText)
                return @{@"cancelled": @NO, @"phase": @"text"};
            ignored.reset();
            auto context = viewer->captureNativeOpeningContext(
                width, height, {hostText, sourceText});
            std::shared_ptr<const core3d::profile_d4::CreationCapture> capture;
            if (!context || core3d::profile_d4::CaptureCreationHost(*owner,
                    hostText, sourceText, context, width, height, capture)
                    != core3d::profile_d4::CaptureStatus::Current)
                return @{@"cancelled": @NO, @"phase": @"capture"};
            core3d::profile_d4::CreationEdit edit;
            edit.kind = core3d::pattern::Kind::Grid;
            edit.rows = 4; edit.columns = 8;
            edit.rowSpacing = 5.0 * 0.001 / context->openingFence().metersPerUnit();
            edit.columnSpacing = edit.rowSpacing;
            std::atomic_bool stop{false}, entered{false};
            std::shared_ptr<const core3d::profile_d4::PreparedCreation> prepared;
            std::thread worker([&] {
                entered.store(true, std::memory_order_release);
                prepared = core3d::profile_d4::PrepareCreation(capture, edit, stop);
            });
            while (!entered.load(std::memory_order_acquire)) std::this_thread::yield();
            stop.store(true, std::memory_order_release);
            worker.join();
            const bool cancelled = !prepared;
            prepared.reset(); capture.reset(); context.reset();
            auto retryContext = viewer->captureNativeOpeningContext(
                width, height, {hostText, sourceText});
            std::shared_ptr<const core3d::profile_d4::CreationCapture> retry;
            const bool unchanged = retryContext
                && core3d::profile_d4::CaptureCreationHost(*owner, hostText,
                    sourceText, retryContext, width, height, retry)
                    == core3d::profile_d4::CaptureStatus::Current;
            return @{@"cancelled": @(cancelled), @"entered": @(entered.load()),
                     @"unchanged": @(unchanged),
                     @"closed": @(!owner->Document()->HasOpenCommand())};
        } catch (...) { return @{@"cancelled": @NO, @"phase": @"exception"}; }
    }
}

static NSDictionary *DebugEdit(Core3DViewController *controller,
                               NSString *host, int32_t scenario) {
    @autoreleasepool {
        try {
            std::shared_ptr<core3d::Core3DViewer> viewer; Handle(OcctDocument) owner;
            std::shared_ptr<core3d::native_opening::Context> context;
            std::uint32_t width = 0, height = 0;
            if (scenario < 0 || scenario > 9
                || !Inputs(controller, host, viewer, owner, context, width, height))
                return @{@"committed": @NO, @"phase": @"inputs"};
            core3d::feature_pattern_owner::Snapshot opening;
            if (core3d::feature_pattern_owner::CaptureNative(
                    *owner, host.UTF8String, context, opening)
                    != core3d::feature_pattern_owner::Refusal::None)
                return @{@"committed": @NO, @"phase": @"capture"};
            const auto& d = opening.record.definition;
            core3d::feature_pattern_owner::Edit edit;
            edit.sourceCutStepID = d.sourceCutStepID;
            edit.kind = d.distribution.kind; edit.rowAxis = d.distribution.rowAxis;
            edit.columnAxis = d.distribution.columnAxis;
            edit.rows = d.distribution.rowCount; edit.columns = d.distribution.columnCount;
            edit.rowSpacing = d.distribution.rowSpacing;
            edit.columnSpacing = d.distribution.columnSpacing;
            edit.sweepRadians = d.distribution.sweepRadians;
            edit.radialPivotLocal = d.distribution.radialPivotLocal;
            for (const auto& member : d.distribution.members)
                if (member.state == core3d::pattern::MemberState::Suppressed)
                    edit.suppressed.insert(member.coordinate);
            const double spacing = 15.0 * 0.001 / d.metersPerUnit;
            if (scenario == 0) edit.columns = 4;
            else if (scenario == 1) {
                const auto* program = std::get_if<core3d::retained_boolean::Program>(
                    &opening.sourceProgram.recipe);
                if (!program || program->steps.size() < 2)
                    return @{@"committed": @NO, @"phase": @"source-step"};
                edit.sourceCutStepID = program->steps[1].operand.identifier;
            } else if (scenario == 2) {
                edit.kind = core3d::pattern::Kind::Grid;
                edit.rows = 2; edit.columns = 3;
                edit.rowAxis = core3d::pattern::Axis::Y;
                edit.columnAxis = core3d::pattern::Axis::X;
                edit.rowSpacing = spacing; edit.columnSpacing = spacing;
            } else {
                if (scenario == 3) {
                    edit.kind = core3d::pattern::Kind::Linear;
                    edit.rows = 1; edit.columns = 3;
                    edit.columnAxis = core3d::pattern::Axis::Y;
                    edit.columnSpacing = spacing;
                } else if (scenario == 4) {
                    edit.columns = 1;
                } else if (scenario == 5) {
                    edit.columns = 33;
                } else if (scenario == 6) {
                    edit.columnSpacing = 0;
                } else if (scenario == 7) {
                    edit.sourceCutStepID = 0;
                } else if (scenario == 8) {
                    edit.columns = 2;
                } else {
                    edit.kind = core3d::pattern::Kind::Grid;
                    edit.rows = 4; edit.columns = 8;
                    edit.rowAxis = core3d::pattern::Axis::Y;
                    edit.columnAxis = core3d::pattern::Axis::X;
                    edit.rowSpacing = 5.0 * 0.001 / d.metersPerUnit;
                    edit.columnSpacing = 5.0 * 0.001 / d.metersPerUnit;
                }
            }
            const auto prepared = core3d::feature_pattern_owner::PrepareNative(
                *owner, opening, edit, core3d::feature_pattern_owner::Limits{});
            if (!prepared.admitted()) return @{@"committed": @NO,
                @"phase": @"prepare", @"refusal": @(unsigned(prepared.refusal))};
            const auto outcome = core3d::feature_pattern_owner::ApplyNative(
                *owner, prepared, context);
            if (outcome != core3d::feature_pattern_owner::ApplyOutcome::Committed)
                return @{@"committed": @NO, @"phase": @"apply",
                    @"outcome": @(unsigned(outcome))};
            // ApplyNative has completed publication and released its stager;
            // retire that opening before acquiring post-commit observation.
            context.reset();
            auto observed = viewer->captureNativeOpeningContext(
                width, height, {host.UTF8String});
            NSMutableDictionary *result = [Observe(
                owner, host, observed) mutableCopy];
            result[@"committed"] = @YES; result[@"phase"] = @"complete";
            return result;
        } catch (...) { return @{@"committed": @NO, @"phase": @"exception"}; }
    }
}

extern "C" void *Core3DDebugD4ProfileCreate(
    Core3DViewController *controller, NSString *host, NSString *source) {
    return (__bridge_retained void *)DebugCreate(controller, host, source);
}

extern "C" void *Core3DDebugD4ProfileCreateFault(
    Core3DViewController *controller, NSString *host, NSString *source,
    int32_t fault) {
    return (__bridge_retained void *)DebugCreate(controller, host, source, fault);
}

extern "C" void *Core3DDebugD4ProfileCreationStop(
    Core3DViewController *controller, NSString *host, NSString *source) {
    return (__bridge_retained void *)DebugCreationStop(controller, host, source);
}

extern "C" void *Core3DDebugD4ProfileObserve(
    Core3DViewController *controller, NSString *host) {
    return (__bridge_retained void *)DebugObserve(controller, host);
}

extern "C" void *Core3DDebugD4ProfileEdit(
    Core3DViewController *controller, NSString *host, int32_t scenario) {
    return (__bridge_retained void *)DebugEdit(controller, host, scenario);
}

extern "C" uint64_t Core3DDebugD4ProfileContinuationContractProbe(
    int32_t scenario, double metersPerUnit) {
    if (!(metersPerUnit == 0.001 || metersPerUnit == 1.0)) return 0;
    const double localPerMM = 0.001 / metersPerUnit;
    uint64_t bits = 0;
    if (std::isfinite(localPerMM) && localPerMM > 0) bits |= 1;
    if (core3d::feature_pattern::MaximumGeneratedFeatures == 32) bits |= 2;
    if (core3d::dependent_replay::Limits{}.records == 128) bits |= 4;
    if (core3d::retained_boolean::MaximumOperands == 4) bits |= 8;
    core3d::retained_edge_treatment::ReplayBudget debt;
    if (debt.valid()) bits |= 16;
    const double host8 = 80 * 50 * 8;
    const double result8 = host8 - 96 * std::acos(-1.0);
    const double host6 = 80 * 50 * 6;
    const double result6 = host6 - 72 * std::acos(-1.0);
    if (result8 < host8 && result6 < host6 && result6 < result8) bits |= 32;
    if (scenario >= 0 && scenario <= 9) bits |= 64;
    return bits;
}
#endif
