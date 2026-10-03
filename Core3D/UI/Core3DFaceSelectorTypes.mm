#import "Core3DFaceSelectorTypes.h"
#include "../OCCTKit/RetainedFaceSelector.hxx"
#import <objc/runtime.h>

static id B2Object(Class type) { return class_createInstance(type, 0); }

static BOOL B2FiniteVector(Core3DFaceSelectorVector3 *value, BOOL unit) {
    if (!value || !isfinite(value.x) || !isfinite(value.y) || !isfinite(value.z)) return NO;
    if (!unit) return fabs(value.x) <= 1e6 && fabs(value.y) <= 1e6 && fabs(value.z) <= 1e6;
    const double squared = value.x * value.x + value.y * value.y + value.z * value.z;
    return fabs(squared - 1.0) <= 1e-12;
}

@implementation Core3DFaceSelectorVector3
- (instancetype)initWithX:(double)x y:(double)y z:(double)z {
    if (!isfinite(x) || !isfinite(y) || !isfinite(z)) return nil;
    if ((self = [super init])) { _x = x == 0 ? 0 : x; _y = y == 0 ? 0 : y; _z = z == 0 ? 0 : z; }
    return self;
}
- (id)copyWithZone:(NSZone *)zone { return self; }
@end

@implementation Core3DPlanarFaceScope
- (instancetype)initWithAxis:(Core3DFaceSelectorAxis)axis side:(Core3DFaceSelectorSide)side {
    if (axis < Core3DFaceSelectorAxisX || axis > Core3DFaceSelectorAxisZ
        || side < Core3DFaceSelectorSideMinimum || side > Core3DFaceSelectorSideMaximum) return nil;
    if ((self = [super init])) { _axis = axis; _side = side; } return self;
}
- (id)copyWithZone:(NSZone *)zone { return self; }
@end

@implementation Core3DPlanarFaceBoundaryIntent
- (instancetype)initWithFace:(Core3DPlanarFaceScope *)face
    edgeKind:(Core3DFaceSelectorBoundaryCurve)edgeKind expectedCount:(NSUInteger)expectedCount {
    if (!face || edgeKind != Core3DFaceSelectorBoundaryCurveLine || expectedCount < 1 || expectedCount > 64) return nil;
    if ((self = [super init])) { _face = face; _edgeKind = edgeKind; _expectedCount = expectedCount; } return self;
}
- (id)copyWithZone:(NSZone *)zone { return self; }
@end

@implementation Core3DLineSelectorIntent
- (instancetype)initWithFace:(Core3DPlanarFaceScope *)face
    locationMM:(Core3DFaceSelectorVector3 *)locationMM
    direction:(Core3DFaceSelectorVector3 *)direction expectedCount:(NSUInteger)expectedCount {
    if (!face || !B2FiniteVector(locationMM, NO) || !B2FiniteVector(direction, YES) || expectedCount != 1) return nil;
    if ((self = [super init])) { _face = face; _locationMM = locationMM; _direction = direction; _expectedCount = 1; }
    return self;
}
- (id)copyWithZone:(NSZone *)zone { return self; }
@end

@implementation Core3DCircleRimIntent
- (instancetype)initWithCenterMM:(Core3DFaceSelectorVector3 *)centerMM
    normal:(Core3DFaceSelectorVector3 *)normal radiusMM:(double)radiusMM expectedCount:(NSUInteger)expectedCount {
    if (!B2FiniteVector(centerMM, NO) || !B2FiniteVector(normal, YES) || !isfinite(radiusMM)
        || radiusMM <= 0 || radiusMM > 1e6 || expectedCount != 1) return nil;
    if ((self = [super init])) { _centerMM = centerMM; _normal = normal; _radiusMM = radiusMM; _expectedCount = 1; }
    return self;
}
- (id)copyWithZone:(NSZone *)zone { return self; }
@end

@implementation Core3DReservedSplineEdgeIntent
- (instancetype)initWithSourceFeatureIdentifier:(NSUUID *)sourceFeatureIdentifier
    featureLocalEdgeKey:(NSUUID *)featureLocalEdgeKey expectedCount:(NSUInteger)expectedCount {
    if (!sourceFeatureIdentifier || !featureLocalEdgeKey) return nil;
    static const uuid_t zero = {};
    uuid_t source, key; [sourceFeatureIdentifier getUUIDBytes:source]; [featureLocalEdgeKey getUUIDBytes:key];
    if (!memcmp(source, zero, sizeof(zero))
        || !memcmp(key, zero, sizeof(zero)) || expectedCount < 1 || expectedCount > 64) return nil;
    if ((self = [super init])) { _sourceFeatureIdentifier = [sourceFeatureIdentifier copy];
        _featureLocalEdgeKey = [featureLocalEdgeKey copy]; _expectedCount = expectedCount; }
    return self;
}
- (id)copyWithZone:(NSZone *)zone { return self; }
@end

@interface Core3DFaceSelectorIntent ()
- (instancetype)initWithB2Kind:(Core3DFaceSelectorKind)kind payload:(id)payload;
@end
@implementation Core3DFaceSelectorIntent
- (instancetype)initWithB2Kind:(Core3DFaceSelectorKind)kind payload:(id)payload {
    if ((self = [super init])) {
        _kind = kind;
        if (kind == Core3DFaceSelectorKindPlanarFaceBoundary) _planarFaceBoundary = payload;
        else if (kind == Core3DFaceSelectorKindLine) _line = payload;
        else if (kind == Core3DFaceSelectorKindCircleRim) _circleRim = payload;
        else _splineEdge = payload;
    }
    return self;
}
+ (instancetype)planarFaceBoundary:(Core3DPlanarFaceBoundaryIntent *)value { return [[self alloc] initWithB2Kind:Core3DFaceSelectorKindPlanarFaceBoundary payload:value]; }
+ (instancetype)line:(Core3DLineSelectorIntent *)value { return [[self alloc] initWithB2Kind:Core3DFaceSelectorKindLine payload:value]; }
+ (instancetype)circleRim:(Core3DCircleRimIntent *)value { return [[self alloc] initWithB2Kind:Core3DFaceSelectorKindCircleRim payload:value]; }
+ (instancetype)splineEdge:(Core3DReservedSplineEdgeIntent *)value { return [[self alloc] initWithB2Kind:Core3DFaceSelectorKindSplineEdge payload:value]; }
- (id)copyWithZone:(NSZone *)zone { return self; }
@end

@implementation Core3DFaceBoundaryUse @end
@implementation Core3DFaceSelectorProof @end
@implementation Core3DFaceSelectorQuery @end
@implementation Core3DFaceSelectorProbeResult @end
#if DEBUG
@implementation Core3DB2TopologyBudgetInjection
- (instancetype)initWithKind:(Core3DB2TopologyBudgetInjectionKind)kind
    remainingVisits:(NSUInteger)remainingVisits remainingStages:(NSUInteger)remainingStages
    site:(NSInteger)site occurrence:(NSInteger)occurrence {
    if(kind<Core3DB2TopologyBudgetInjectionNone||kind>Core3DB2TopologyBudgetInjectionCorrupt)
        return nil;
    if((self=[super init])){_kind=kind;_remainingVisits=remainingVisits;
        _remainingStages=remainingStages;_site=site;_occurrence=occurrence;}
    return self;
}
@end
@implementation Core3DB2TopologyBudgetObservation @end
#endif

@interface Core3DFaceSelectorNativeProof : Core3DFaceSelectorProof {
@public
    std::shared_ptr<const core3d::retained_edge_treatment::SelectorTargetCapture> nativeTargets;
}
@end
@implementation Core3DFaceSelectorNativeProof @end

namespace core3d::face_selector_bridge {
using namespace retained_face_selector;
static std::array<double, 3> Vector(Core3DFaceSelectorVector3 *value) {
    return {value.x, value.y, value.z};
}
bool Intent(Core3DFaceSelectorIntent *source, SelectorIntent& output) {
    if (!source) return false;
    switch (source.kind) {
    case Core3DFaceSelectorKindPlanarFaceBoundary: {
        const auto *value = source.planarFaceBoundary;
        output = PlanarFaceBoundary{{Axis(value.face.axis), Side(value.face.side)},
            BoundaryCurve(value.edgeKind), std::uint32_t(value.expectedCount)}; break;
    }
    case Core3DFaceSelectorKindLine: {
        const auto *value = source.line;
        output = Line{{Axis(value.face.axis), Side(value.face.side)}, Vector(value.locationMM),
            Vector(value.direction), std::uint32_t(value.expectedCount)}; break;
    }
    case Core3DFaceSelectorKindCircleRim: {
        const auto *value = source.circleRim;
        output = CircleRim{Vector(value.centerMM), Vector(value.normal), value.radiusMM,
            std::uint32_t(value.expectedCount)}; break;
    }
    case Core3DFaceSelectorKindSplineEdge: {
        ReservedSplineEdge value; uuid_t sourceBytes, keyBytes;
        [source.splineEdge.sourceFeatureIdentifier getUUIDBytes:sourceBytes];
        [source.splineEdge.featureLocalEdgeKey getUUIDBytes:keyBytes];
        std::copy_n(sourceBytes, 16, value.sourceFeatureID.begin());
        std::copy_n(keyBytes, 16, value.featureLocalEdgeKey.begin());
        value.expectedCount = std::uint32_t(source.splineEdge.expectedCount); output = value; break;
    }
    default: return false;
    }
    return ValidIntent(output, true);
}
static Core3DFaceSelectorVector3 *Vector(const std::array<double, 3>& value) {
    return [[Core3DFaceSelectorVector3 alloc] initWithX:value[0] y:value[1] z:value[2]];
}
static Core3DFaceSelectorStatus Status(Refusal refusal) {
    if (refusal == Refusal::None) return Core3DFaceSelectorStatusProved;
    if (refusal == Refusal::StaleSource) return Core3DFaceSelectorStatusStale;
    if (refusal == Refusal::Cancelled) return Core3DFaceSelectorStatusCancelled;
    if (refusal == Refusal::Budget || refusal == Refusal::TruncatedDiscovery) return Core3DFaceSelectorStatusBudget;
    if (refusal == Refusal::Busy) return Core3DFaceSelectorStatusBusy;
    if (refusal == Refusal::NativeFailure || refusal == Refusal::ReadOnlyInvariant) return Core3DFaceSelectorStatusFailed;
    return Core3DFaceSelectorStatusRefused;
}
static Core3DFaceSelectorQuery *BuildQuery(Core3DFaceSelectorIntent *intent,
    const Resolution& resolution,
    const std::shared_ptr<const core3d::retained_edge_treatment::SelectorTargetCapture>& targets,
    const std::vector<core3d::retained_edge_treatment::CurveKind>* projectedKinds=nullptr) {
    Core3DFaceSelectorQuery *query = B2Object(Core3DFaceSelectorQuery.class);
    [query setValue:intent forKey:@"intent"];
    [query setValue:@(NSInteger(Status(resolution.refusal))) forKey:@"status"];
    [query setValue:@(NSInteger(resolution.refusal)) forKey:@"refusal"];
    [query setValue:[NSString stringWithUTF8String:RefusalCode(resolution.refusal)] forKey:@"refusalCode"];
    [query setValue:[NSString stringWithUTF8String:RefusalMessage(resolution.refusal)] forKey:@"refusalMessage"];
    [query setValue:@0 forKey:@"measuredUndoDelta"]; [query setValue:@0 forKey:@"measuredRedoDelta"];
    if (resolution.refusal != Refusal::None || !resolution.proof) return query;
    Core3DFaceSelectorProof *proof = targets
        ? B2Object(Core3DFaceSelectorNativeProof.class) : B2Object(Core3DFaceSelectorProof.class);
    if (targets) ((Core3DFaceSelectorNativeProof *)proof)->nativeTargets = targets;
    [proof setValue:intent forKey:@"intent"]; [proof setValue:Vector(resolution.proof->plane().outwardNormal) forKey:@"outwardNormal"];
    [proof setValue:@(resolution.proof->plane().offsetMM) forKey:@"planeOffsetMM"];
    [proof setValue:@(NSInteger(resolution.proof->coverage())) forKey:@"coverage"];
    [proof setValue:@1 forKey:@"matchedFaceCount"]; [proof setValue:@(resolution.proof->wireCount()) forKey:@"wireCount"];
    [proof setValue:@(resolution.proof->boundaryUses().size()) forKey:@"boundaryUseCount"];
    [proof setValue:@(resolution.proof->uniqueEdgeCount()) forKey:@"boundaryUniqueEdgeCount"];
    [proof setValue:@(resolution.proof->selectedEdgeCount()) forKey:@"selectedEdgeCount"];
    [proof setValue:@YES forKey:@"discoveryComplete"]; [proof setValue:@NO forKey:@"truncated"];
    NSMutableArray *uses = [NSMutableArray arrayWithCapacity:resolution.proof->boundaryUses().size()];
    std::size_t useIndex=0;
    for (const auto& native : resolution.proof->boundaryUses()) {
        Core3DFaceBoundaryUse *use = B2Object(Core3DFaceBoundaryUse.class);
        [use setValue:@(native.selected) forKey:@"selected"];
        [use setValue:@(NSInteger(native.direction)) forKey:@"canonicalFaceUseDirection"];
        [use setValue:@(native.ownerFaces.size()) forKey:@"ownerFaceCount"]; [use setValue:@YES forKey:@"selectedFaceIsOwner"];
        const auto kind=projectedKinds&&useIndex<projectedKinds->size()
            ?(*projectedKinds)[useIndex]
            :(BRepAdaptor_Curve(native.edge).GetType()==GeomAbs_Circle
                ?core3d::retained_edge_treatment::CurveKind::Circle
                :core3d::retained_edge_treatment::CurveKind::Line);
        [use setValue:@(kind==core3d::retained_edge_treatment::CurveKind::Circle
            ?Core3DEdgeTreatmentCurveKindCircle:Core3DEdgeTreatmentCurveKindLine) forKey:@"curveKind"];
        [use setValue:@(Core3DFaceSelectorWirePositionOuter) forKey:@"outerOrInnerWire"];
        [uses addObject:use];++useIndex;
    }
    [proof setValue:uses forKey:@"boundaryUses"]; [query setValue:proof forKey:@"proof"];
    return query;
}
Core3DFaceSelectorQuery *Query(Core3DFaceSelectorIntent *intent, const Resolution& resolution) {
    return BuildQuery(intent, resolution, {});
}
Core3DFaceSelectorQuery *BoundQuery(Core3DFaceSelectorIntent *intent,
    const std::shared_ptr<const core3d::retained_edge_treatment::SelectorTargetCapture>& targets,
    const std::vector<core3d::retained_edge_treatment::CurveKind>& projectedKinds) {
    Resolution resolution;
    if (!targets) { resolution.refusal = Refusal::StaleSource; return BuildQuery(intent, resolution, {}); }
    resolution.refusal = Refusal::None;
    resolution.proof = std::shared_ptr<const FaceMembershipProof>(targets, &targets->proof());
    return BuildQuery(intent, resolution, targets, &projectedKinds);
}
std::shared_ptr<const core3d::retained_edge_treatment::SelectorTargetCapture>
Targets(Core3DFaceSelectorProof *proof) noexcept {
    if (![proof isKindOfClass:Core3DFaceSelectorNativeProof.class]) return {};
    return ((Core3DFaceSelectorNativeProof *)proof)->nativeTargets;
}
} // namespace core3d::face_selector_bridge
