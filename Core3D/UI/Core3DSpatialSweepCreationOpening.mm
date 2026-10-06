// Production bridge for one complete inline C2 spatial-sweep creation.
#import "Core3DViewController.h"
#import "GLViewController+Trick.h"
#import "Core3DViewer.h"
#import "../OCCTKit/GLViewController.h"

#include "../OCCTKit/SpatialSweepOwner.hxx"

#include <atomic>
#include <cmath>
#include <memory>
#include <string>

namespace {
using core3d::spatial_sweep::ClosureKind;
using core3d::spatial_sweep::RadiusLawKind;
using core3d::spatial_sweep::TwistLawKind;
namespace sweep_owner = core3d::spatial_sweep::owner;

enum class State : std::uint8_t { Open, Applying, Cancelled, Settled, Recovery };

struct Settlement final {
    Core3DProfileConstructionResult result = Core3DProfileConstructionResultRejected;
    State state = State::Settled;
    bool retainsRecoveryOwnership = false;
};

Settlement Settle(const sweep_owner::Receipt& receipt) noexcept {
    Settlement settlement;
    if (receipt.outcome == sweep_owner::Outcome::committed)
        settlement.result = Core3DProfileConstructionResultCommitted;
    else if (receipt.outcome == sweep_owner::Outcome::busy)
        settlement.result = Core3DProfileConstructionResultBusy;
    else if (receipt.outcome == sweep_owner::Outcome::cancelled)
        settlement.result = Core3DProfileConstructionResultCancelled;
    else if (receipt.outcome == sweep_owner::Outcome::outcomeUnknown
             || receipt.outcome == sweep_owner::Outcome::recoveryRequired)
        settlement.result = Core3DProfileConstructionResultRecoveryRequired;
    settlement.retainsRecoveryOwnership =
        settlement.result == Core3DProfileConstructionResultRecoveryRequired;
    settlement.state = settlement.retainsRecoveryOwnership
        ? State::Recovery : State::Settled;
    return settlement;
}

bool ExactKeys(NSDictionary *value, NSArray<NSString *> *keys) {
    if (![value isKindOfClass:NSDictionary.class] || value.count != keys.count) return false;
    return [[NSSet setWithArray:value.allKeys]
        isEqualToSet:[NSSet setWithArray:keys]];
}

bool Number(id value, double& output) {
    if (![value isKindOfClass:NSNumber.class]
        || CFGetTypeID((__bridge CFTypeRef)value) == CFBooleanGetTypeID()) return false;
    output = [value doubleValue];
    return std::isfinite(output);
}

bool Integer(id value, std::int64_t minimum, std::int64_t maximum,
             std::int64_t& output) {
    double scalar = 0;
    if (!Number(value, scalar) || std::trunc(scalar) != scalar
        || scalar < double(minimum) || scalar > double(maximum)) return false;
    output = [value longLongValue];
    return double(output) == scalar;
}

bool Vector3(id value, std::array<double, 3>& output) {
    if (![value isKindOfClass:NSArray.class] || [(NSArray *)value count] != 3) return false;
    for (NSUInteger index = 0; index < 3; ++index)
        if (!Number(value[index], output[index])) return false;
    return true;
}

bool Text(id value, NSString *__strong& output) {
    if (![value isKindOfClass:NSString.class] || [(NSString *)value length] == 0) return false;
    output = value;
    return true;
}

void Deliver(void (^completion)(Core3DProfileConstructionResult, NSString *, NSString *),
             Core3DProfileConstructionResult result, NSString *detail,
             NSString *entity) {
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, detail, entity);
    else dispatch_async(dispatch_get_main_queue(), ^{ completion(result, detail, entity); });
}

bool ParseCandidate(NSDictionary *value, double metersPerUnit,
                    sweep_owner::CreateRequest& output) {
    using namespace core3d;
    using namespace bounded_curve;
    if (!ExactKeys(value, @[@"degree", @"poles", @"knots", @"weights", @"frame",
            @"seed", @"phaseDegrees", @"radiusLaw", @"startRadiusMM",
            @"endRadiusMM", @"twistLaw", @"totalTwistDegrees",
            @"windingTurns", @"closure"])
        || !std::isfinite(metersPerUnit) || metersPerUnit <= 0) return false;
    std::int64_t degree = 0;
    if (!Integer(value[@"degree"], MinimumDegree, MaximumDegree, degree)
        || ![value[@"poles"] isKindOfClass:NSArray.class]
        || ![value[@"knots"] isKindOfClass:NSArray.class]
        || ![value[@"frame"] isKindOfClass:NSDictionary.class]) return false;
    Definition curve;
    curve.domain = Domain::Path3D;
    curve.degree = std::uint8_t(degree);
    for (id item in (NSArray *)value[@"poles"]) {
        ControlPoint pole;
        if (![item isKindOfClass:NSDictionary.class]
            || !ExactKeys(item, @[@"local"])
            || !Vector3(item[@"local"], pole.local)) return false;
        curve.controlPoints.push_back(pole);
    }
    for (id item in (NSArray *)value[@"knots"]) {
        double scalar = 0;
        std::int64_t multiplicity = 0;
        if (![item isKindOfClass:NSDictionary.class]
            || !ExactKeys(item, @[@"value", @"multiplicity"])
            || !Number(item[@"value"], scalar)
            || !Integer(item[@"multiplicity"], 1, 8, multiplicity)) return false;
        curve.knots.push_back({scalar, std::uint8_t(multiplicity)});
    }
    id weights = value[@"weights"];
    if (weights != NSNull.null) {
        if (![weights isKindOfClass:NSArray.class]
            || [(NSArray *)weights count] != curve.controlPoints.size()) return false;
        for (id item in (NSArray *)weights) {
            double scalar = 0;
            if (!Number(item, scalar)) return false;
            curve.weights.push_back(scalar);
        }
    }
    NSDictionary *frame = value[@"frame"];
    if (!ExactKeys(frame, @[@"origin", @"xAxis", @"yAxis", @"zAxis"])
        || !Vector3(frame[@"origin"], curve.frame.origin)
        || !Vector3(frame[@"xAxis"], curve.frame.xAxis)
        || !Vector3(frame[@"yAxis"], curve.frame.yAxis)
        || !Vector3(frame[@"zAxis"], curve.frame.zAxis)) return false;

    spatial_sweep::Definition sweep;
    if (!Vector3(value[@"seed"], sweep.orientation.authoredSeed)) return false;
    double phase = 0, start = 0, end = 0, total = 0;
    std::int64_t winding = 0;
    NSString *radiusLaw = nil, *twistLaw = nil, *closure = nil;
    if (!Number(value[@"phaseDegrees"], phase) || phase < -180 || phase > 180
        || !Number(value[@"startRadiusMM"], start)
        || !Number(value[@"endRadiusMM"], end)
        || !Number(value[@"totalTwistDegrees"], total)
        || total < -720 || total > 720
        || !Integer(value[@"windingTurns"], -2, 2, winding)
        || !Text(value[@"radiusLaw"], radiusLaw)
        || !Text(value[@"twistLaw"], twistLaw)
        || !Text(value[@"closure"], closure)) return false;
    if ([radiusLaw isEqualToString:@"constant"])
        sweep.radius.kind = RadiusLawKind::Constant;
    else if ([radiusLaw isEqualToString:@"linearArcLength"])
        sweep.radius.kind = RadiusLawKind::LinearArcLength;
    else return false;
    if ([twistLaw isEqualToString:@"linearArcLength"])
        sweep.twist.kind = TwistLawKind::LinearArcLength;
    else if ([twistLaw isEqualToString:@"closeFrame"])
        sweep.twist.kind = TwistLawKind::CloseFrame;
    else return false;
    if ([closure isEqualToString:@"openFlatCaps"])
        sweep.closure = ClosureKind::OpenFlatCaps;
    else if ([closure isEqualToString:@"closedNoCaps"])
        sweep.closure = ClosureKind::ClosedNoCaps;
    else return false;
    const double millimetresPerUnit = metersPerUnit * 1000.0;
    sweep.orientation.phaseRadians = phase * spatial_sweep::Pi / 180.0;
    sweep.radius.startRadius = start / millimetresPerUnit;
    sweep.radius.endRadius = end / millimetresPerUnit;
    sweep.twist.totalRadians = total * spatial_sweep::Pi / 180.0;
    sweep.twist.windingTurns = std::int32_t(winding);
    if (sweep.closure == ClosureKind::OpenFlatCaps) {
        if (sweep.twist.kind != TwistLawKind::LinearArcLength || winding != 0) return false;
    } else if (sweep.radius.kind != RadiusLawKind::Constant
               || sweep.twist.kind != TwistLawKind::CloseFrame || total != 0) return false;
    output.curve = std::move(curve);
    output.sweep = sweep;
    return true;
}

struct Capture final {
    Handle(OcctDocument) owner;
    std::shared_ptr<core3d::native_opening::Context> context;
};

bool CaptureOpening(Core3DViewController *controller, Capture& output) noexcept {
    output = {};
    try {
        if (!NSThread.isMainThread || !controller) return false;
        GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)controller.glController : nil;
        if (!gl || !gl.viewer) return false;
        Capture value;
        value.owner = gl.viewer->getDocument();
        value.context = gl.viewer->captureNativeOpeningContext(64, 64, {});
        if (value.owner.IsNull() || value.owner->Document().IsNull()
            || value.owner->Document()->HasOpenCommand() || !value.context
            || value.context->openingFence().document() != value.owner->Document()
            || value.context->openingFence().data() != value.owner->Document()->GetData()
            || !value.context->isCurrent(64, 64)) return false;
        output = std::move(value);
        return true;
    } catch (...) { output = {}; return false; }
}

} // namespace

@interface Core3DSpatialSweepCreationOpening () {
@package
    Handle(OcctDocument) _owner;
    std::shared_ptr<core3d::native_opening::Context> _context;
    std::unique_ptr<sweep_owner::OcafOwner> _service;
    std::atomic<State> _state;
}
- (instancetype)initWithCapture:(Capture)capture;
@end

@implementation Core3DSpatialSweepCreationOpening

- (instancetype)initWithCapture:(Capture)capture {
    if ((self = [super init])) {
        _owner = capture.owner;
        _context = std::move(capture.context);
        _service = std::make_unique<sweep_owner::OcafOwner>(*_owner, _context);
        _state.store(State::Open);
    }
    return self;
}

- (NSDictionary<NSString *, id> *)descriptor {
    if (_state.load() != State::Open || !_context || _owner.IsNull()) return @{};
    return @{@"schema": @"shapeyard.c2-spatial-sweep-creation.v1",
        @"documentMetersPerUnit": @(_context->openingFence().metersPerUnit()),
        @"profile": @"c2-circle-1", @"section": @"solidCircle",
        @"transport": @"bishop.v1",
        @"parameterization": @"normalizedArcLength.v1",
        @"minimumDegree": @(core3d::bounded_curve::MinimumDegree),
        @"maximumDegree": @(core3d::bounded_curve::MaximumDegree),
        @"maximumPoles": @(core3d::bounded_curve::MaximumControlPoints),
        @"maximumCoordinate": @(core3d::bounded_curve::CoordinateLimit),
        @"minimumWeight": @(core3d::bounded_curve::MinimumWeight),
        @"maximumWeight": @(core3d::bounded_curve::MaximumWeight),
        @"minimumRadiusMM": @0.1, @"maximumRadiusMM": @100000.0,
        @"closures": @[@"openFlatCaps", @"closedNoCaps"]};
}

- (void)createCandidate:(NSDictionary *)candidate name:(NSString *)name
    completion:(void (^)(Core3DProfileConstructionResult, NSString *, NSString *))completion {
    State expected = State::Open;
    if (!_state.compare_exchange_strong(expected, State::Applying)
        || !_service || !_context || _owner.IsNull()
        || ![candidate isKindOfClass:NSDictionary.class]
        || ![name isKindOfClass:NSString.class] || name.length == 0 || name.length > 128) {
        Deliver(completion, Core3DProfileConstructionResultRejected,
                @"Creation opening is not available.", nil);
        return;
    }
    sweep_owner::CreateRequest request;
    request.expectedScene = {_context->openingFence().documentGeneration(),
        _context->openingFence().modelRevision(),
        _context->openingFence().metersPerUnit()};
    request.requestedName = name.UTF8String ?: "";
    if (!ParseCandidate(candidate, _context->openingFence().metersPerUnit(), request)) {
        _state.store(State::Open);
        Deliver(completion, Core3DProfileConstructionResultRejected,
            @"Malformed, nonfinite, unknown-key, or out-of-bounds spatial-sweep values.", nil);
        return;
    }
    const auto receipt = _service->create(request);
    const Settlement settlement = Settle(receipt);
    const Core3DProfileConstructionResult result = settlement.result;
    _state.store(settlement.state);
    if (!settlement.retainsRecoveryOwnership) {
        _service.reset(); _context.reset();
    }
    NSString *entity = result == Core3DProfileConstructionResultCommitted
        ? [NSString stringWithUTF8String:receipt.entityIdentifier.c_str()] : nil;
    Deliver(completion, result,
        result == Core3DProfileConstructionResultCommitted
            ? @"Spatial sweep created as one native history command."
            : result == Core3DProfileConstructionResultRecoveryRequired
                ? @"Spatial-sweep creation close is unknown; native recovery ownership is retained."
                : [NSString stringWithFormat:@"Spatial-sweep creation refused without history (%s).",
                    receipt.reason.c_str()], entity);
}

- (BOOL)cancel {
    State expected = State::Open;
    if (!_state.compare_exchange_strong(expected, State::Cancelled)) return NO;
    const auto receipt = _service ? _service->cancel(1) : sweep_owner::Receipt{};
    const BOOL cancelled = receipt.outcome == sweep_owner::Outcome::cancelled;
    if (!cancelled) _state.store(State::Recovery);
    else { _service.reset(); _context.reset(); }
    return cancelled;
}

@end

#if DEBUG
extern "C" std::uint64_t core3dDebugSpatialSweepCreationUnknownCloseProbe() {
    // Exercise the exact settlement helper used by createCandidate:, without
    // opening a document command. Bits prove mapping, retained ownership,
    // recovery state, rejected retry, and a single synthetic attempt.
    sweep_owner::Receipt receipt;
    receipt.outcome = sweep_owner::Outcome::outcomeUnknown;
    const Settlement settlement = Settle(receipt);
    std::atomic<State> state{State::Applying};
    bool ownsRecovery = true;
    std::uint64_t attempts = 1;
    state.store(settlement.state);
    if (!settlement.retainsRecoveryOwnership) ownsRecovery = false;
    State expected = State::Open;
    const bool retried = state.compare_exchange_strong(expected, State::Applying);
    if (retried) ++attempts;
    std::uint64_t result = 0;
    if (settlement.result == Core3DProfileConstructionResultRecoveryRequired) result |= 1;
    if (state.load() == State::Recovery) result |= 2;
    if (ownsRecovery) result |= 4;
    if (!retried) result |= 8;
    if (attempts == 1) result |= 16;
    return result;
}
#endif

@implementation Core3DViewController (SpatialSweepCreationOpening)

- (Core3DSpatialSweepCreationOpening *)openSpatialSweepCreation {
    Capture capture;
    if (!CaptureOpening(self, capture)) return nil;
    return [[Core3DSpatialSweepCreationOpening alloc]
        initWithCapture:std::move(capture)];
}

@end
