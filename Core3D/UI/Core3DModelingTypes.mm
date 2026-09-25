// Shared immutable modeling recipe implementations.
// Keep this file Objective-C++: the public Objective-C values own validated,
// detached native recipe values but no document, viewer, controller or UI state.
#import "Core3DModelingTypes.h"

#include "../OCCTKit/ProfileCurvePresets.hxx"
#include "../OCCTKit/ProfilePersistence.hxx"
#include "../OCCTKit/RectangularLoftRebuild.hxx"
#include "../OCCTKit/SweepPersistence.hxx"
#include "../OCCTKit/EnclosureParameters.hxx"

#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <utility>
#include <vector>

static bool Core3DPartBooleanFinite3(simd_double3 value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}
static bool Core3DPartBooleanFinite4(simd_double4 value) {
    return std::isfinite(value.x) && std::isfinite(value.y)
        && std::isfinite(value.z) && std::isfinite(value.w);
}

@implementation Core3DPartBooleanInputValues
- (instancetype)initWithRole:(NSString *)role name:(NSString *)name
    dimensionsMM:(simd_double3)dimensions translationMM:(simd_double3)translation
    rotationXYZW:(simd_double4)rotation metersPerUnit:(double)metersPerUnit
    baseColorSRGB:(simd_double3)baseColor metallic:(double)metallic roughness:(double)roughness {
    const double norm = simd_length_squared(rotation);
    if (![role isKindOfClass:NSString.class] || !([role isEqualToString:@"left"] || [role isEqualToString:@"right"])
        || ![name isKindOfClass:NSString.class] || name.length > 128
        || !Core3DPartBooleanFinite3(dimensions) || dimensions.x <= 0 || dimensions.y <= 0 || dimensions.z <= 0
        || !Core3DPartBooleanFinite3(translation) || !Core3DPartBooleanFinite4(rotation)
        || !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-10
        || !std::isfinite(metersPerUnit) || metersPerUnit <= 0
        || !Core3DPartBooleanFinite3(baseColor) || baseColor.x < 0 || baseColor.x > 1
        || baseColor.y < 0 || baseColor.y > 1 || baseColor.z < 0 || baseColor.z > 1
        || !std::isfinite(metallic) || metallic < 0 || metallic > 1
        || !std::isfinite(roughness) || roughness < 0 || roughness > 1) return nil;
    if ((self=[super init])) {
        _role=[role copy];_name=[name copy];_family=Core3DPartBooleanInputFamilyAnalyticRectangularPrism;
        _dimensionsMM=dimensions;_translationMM=translation;
        _rotationXYZW=rotation;_metersPerUnit=metersPerUnit;_baseColorSRGB=baseColor;
        _metallic=metallic;_roughness=roughness;
        _profilePointsMM=@[];_extrusionPlane=0;
        _constructionRotationXYZW=simd_make_double4(0,0,0,1);_constructionScale=1;
        _shellRotationXYZW=simd_make_double4(0,0,0,1);_shellScale=1;
        _shellOpeningKeys=@[];
    }
    return self;
}
- (instancetype)initWithShellRole:(NSString *)role name:(NSString *)name
    profilePointsMM:(NSArray<NSValue *> *)points extrusionDepthMM:(double)depth
    extrusionPlane:(NSInteger)plane
    constructionTranslationMM:(simd_double3)constructionTranslation
    constructionRotationXYZW:(simd_double4)constructionRotation
    constructionScale:(double)constructionScale
    inputTranslationMM:(simd_double3)translation inputRotationXYZW:(simd_double4)rotation
    metersPerUnit:(double)metersPerUnit shellThicknessMM:(double)thickness
    shellTranslationMM:(simd_double3)shellTranslation
    shellRotationXYZW:(simd_double4)shellRotation shellScale:(double)shellScale
    shellOpeningKeys:(NSArray<NSNumber *> *)openings
    baseColorSRGB:(simd_double3)baseColor metallic:(double)metallic roughness:(double)roughness {
    if (![role isKindOfClass:NSString.class]
        || !([role isEqualToString:@"left"] || [role isEqualToString:@"right"])
        || ![name isKindOfClass:NSString.class] || name.length > 128
        || ![points isKindOfClass:NSArray.class] || points.count != 4
        || ![openings isKindOfClass:NSArray.class] || openings.count < 1 || openings.count > 2
        || !std::isfinite(depth) || depth <= 0 || plane != 0
        || !Core3DPartBooleanFinite3(constructionTranslation)
        || !Core3DPartBooleanFinite4(constructionRotation)
        || !std::isfinite(constructionScale) || constructionScale <= 0
        || !Core3DPartBooleanFinite3(translation) || !Core3DPartBooleanFinite4(rotation)
        || !std::isfinite(metersPerUnit) || metersPerUnit <= 0
        || !std::isfinite(thickness) || thickness <= 0
        || !Core3DPartBooleanFinite3(shellTranslation)
        || !Core3DPartBooleanFinite4(shellRotation)
        || !std::isfinite(shellScale) || shellScale <= 0
        || std::abs(simd_length_squared(constructionRotation)-1)>1e-10
        || std::abs(simd_length_squared(rotation)-1)>1e-10
        || std::abs(simd_length_squared(shellRotation)-1)>1e-10
        || !Core3DPartBooleanFinite3(baseColor) || baseColor.x<0 || baseColor.x>1
        || baseColor.y<0 || baseColor.y>1 || baseColor.z<0 || baseColor.z>1
        || !std::isfinite(metallic) || metallic<0 || metallic>1
        || !std::isfinite(roughness) || roughness<0 || roughness>1) return nil;
    CGPoint decoded[4]{};
    for (NSUInteger index=0;index<4;++index) {
        if (![points[index] isKindOfClass:NSValue.class]
            || strcmp([points[index] objCType],@encode(CGPoint))!=0) return nil;
        [points[index] getValue:&decoded[index] size:sizeof(CGPoint)];
        if (!std::isfinite(decoded[index].x)||!std::isfinite(decoded[index].y)) return nil;
    }
    if (!(decoded[0].y==decoded[1].y && decoded[1].x==decoded[2].x
        && decoded[2].y==decoded[3].y && decoded[3].x==decoded[0].x
        && decoded[1].x>decoded[0].x && decoded[3].y>decoded[0].y)) return nil;
    NSInteger previous=-1;
    for (NSNumber *opening in openings) {
        if (![opening isKindOfClass:NSNumber.class] || opening.integerValue<=previous
            || opening.integerValue<0 || opening.integerValue>5) return nil;
        previous=opening.integerValue;
    }
    if ((self=[super init])) {
        _role=[role copy];_name=[name copy];_family=Core3DPartBooleanInputFamilyShellProfile;
        _profilePointsMM=[points copy];_dimensionsMM=simd_make_double3(
            decoded[1].x-decoded[0].x,decoded[3].y-decoded[0].y,depth);
        _extrusionPlane=plane;_constructionTranslationMM=constructionTranslation;
        _constructionRotationXYZW=constructionRotation;_constructionScale=constructionScale;
        _translationMM=translation;_rotationXYZW=rotation;_metersPerUnit=metersPerUnit;
        _shellThicknessMM=@(thickness);_shellTranslationMM=shellTranslation;
        _shellRotationXYZW=shellRotation;_shellScale=shellScale;
        _shellOpeningKeys=[openings copy];_baseColorSRGB=baseColor;
        _metallic=metallic;_roughness=roughness;
    }
    return self;
}
@end

@implementation Core3DPartBooleanValues
- (instancetype)initWithOperation:(Core3DPartBooleanOperation)operation
    inputs:(NSArray<Core3DPartBooleanInputValues *> *)inputs {
    if (operation < Core3DPartBooleanOperationUnion || operation > Core3DPartBooleanOperationIntersect
        || ![inputs isKindOfClass:NSArray.class] || inputs.count != 2
        || ![inputs[0] isMemberOfClass:Core3DPartBooleanInputValues.class]
        || ![inputs[1] isMemberOfClass:Core3DPartBooleanInputValues.class]
        || ![inputs[0].role isEqualToString:@"left"] || ![inputs[1].role isEqualToString:@"right"]) return nil;
    if ((self=[super init])) {_operation=operation;_inputs=[inputs copy];}
    return self;
}
@end

// Foundation's typed storage preserves the public CGPoint encoding on both
// platforms without depending on UIKit's NSValue convenience category.
static NSValue *Core3DBoxPoint(CGPoint point) {
    return [NSValue valueWithBytes:&point objCType:@encode(CGPoint)];
}

@implementation Core3DProfileCurveVertex
- (instancetype)initWithIdentifier:(uint32_t)identifier point:(CGPoint)point {
    if (identifier == 0 || !std::isfinite(point.x) || !std::isfinite(point.y)
        || std::abs(point.x) > 1e6 || std::abs(point.y) > 1e6) return nil;
    self = [super init];
    if (self) { _identifier = identifier; _point = point; }
    return self;
}
@end

@implementation Core3DProfileCurveSegment
- (instancetype)initWithIdentifier:(uint32_t)identifier
    startVertex:(uint32_t)startVertex endVertex:(uint32_t)endVertex
    kind:(Core3DProfileCurveKind)kind center:(CGPoint)center radius:(double)radius
    startDegrees:(double)startDegrees sweepDegrees:(double)sweepDegrees {
    if (identifier == 0 || startVertex == 0 || endVertex == 0 || startVertex == endVertex
        || !std::isfinite(center.x) || !std::isfinite(center.y)
        || !std::isfinite(radius) || !std::isfinite(startDegrees)
        || !std::isfinite(sweepDegrees)) return nil;
    if (kind == Core3DProfileCurveKindLine) {
        if (center.x != 0 || center.y != 0 || radius != 0
            || startDegrees != 0 || sweepDegrees != 0) return nil;
    } else if (kind == Core3DProfileCurveKindCircularArc) {
        if (std::abs(center.x) > 1e6 || std::abs(center.y) > 1e6
            || radius < 1e-3 || radius > 1e6 || std::abs(startDegrees) > 360
            || std::abs(sweepDegrees) < 1e-6 || std::abs(sweepDegrees) >= 360) return nil;
    } else return nil;
    self = [super init];
    if (self) {
        _identifier = identifier; _startVertex = startVertex; _endVertex = endVertex;
        _kind = kind; _center = center; _radius = radius;
        _startDegrees = startDegrees; _sweepDegrees = sweepDegrees;
    }
    return self;
}
@end

static Core3DProfileCurveLoop *Core3DPublicCurveLoop(const core3d::ProfileCurveLoop& loop);
template<std::size_t N>
static bool Core3DProfilePresetIdentifiers(uint32_t loopID,
    NSArray<NSNumber *> *vertices, NSArray<NSNumber *> *segments,
    core3d::ProfileCurvePresetIDs<N>& output) {
    if (loopID == 0 || ![vertices isKindOfClass:[NSArray class]]
        || ![segments isKindOfClass:[NSArray class]]
        || vertices.count != N || segments.count != N) return false;
    const auto read = [](id value, core3d::ProfileCurveID& identifier) {
        if (![value isKindOfClass:[NSNumber class]]) return false;
        const double scalar = [value doubleValue];
        if (!std::isfinite(scalar) || scalar < 1 || std::floor(scalar) != scalar
            || scalar > std::numeric_limits<std::uint32_t>::max()) return false;
        identifier = static_cast<std::uint32_t>(scalar); return true;
    };
    core3d::ProfileCurvePresetIDs<N> result; result.loop = loopID;
    for (std::size_t i = 0; i < N; ++i)
        if (!read(vertices[i],result.vertices[i]) || !read(segments[i],result.segments[i])) return false;
    output = result; return true;
}


@interface Core3DProfileCurveLoop ()
- (core3d::ProfileCurveLoop)nativeLoop;
@end
@implementation Core3DProfileCurveLoop {
    core3d::ProfileCurveLoop _native;
}
- (instancetype)initWithIdentifier:(uint32_t)identifier
    vertices:(NSArray<Core3DProfileCurveVertex *> *)vertices
    segments:(NSArray<Core3DProfileCurveSegment *> *)segments {
    if (identifier == 0 || ![vertices isKindOfClass:[NSArray class]]
        || ![segments isKindOfClass:[NSArray class]] || vertices.count < 2
        || vertices.count > 512 || vertices.count != segments.count) return nil;
    NSArray *ownedVertices = [vertices copy], *ownedSegments = [segments copy];
    try {
        core3d::ProfileCurveLoop loop; loop.identifier = identifier;
        loop.vertices.reserve(ownedVertices.count); loop.segments.reserve(ownedSegments.count);
        for (id value in ownedVertices) {
            if (![value isMemberOfClass:[Core3DProfileCurveVertex class]]) return nil;
            Core3DProfileCurveVertex *v = value;
            loop.vertices.push_back({v.identifier, gp_Pnt2d(v.point.x,v.point.y)});
        }
        for (id value in ownedSegments) {
            if (![value isMemberOfClass:[Core3DProfileCurveSegment class]]) return nil;
            Core3DProfileCurveSegment *v = value;
            loop.segments.push_back({v.identifier,v.startVertex,v.endVertex,
                static_cast<core3d::ProfileCurveKind>(v.kind),gp_Pnt2d(v.center.x,v.center.y),
                v.radius,v.startDegrees,v.sweepDegrees});
        }
        std::set<core3d::ProfileCurveID> identifiers;
        std::size_t vertexCount = 0, segmentCount = 0;
        core3d::ProfileCurveLoopInspection inspection;
        if (!core3d::InspectProfileCurveLoopStructure(loop,loop.vertices.front().point,
            identifiers,vertexCount,segmentCount,inspection)) return nil;
        self = [super init];
        if (self) {
            _native = std::move(loop); _identifier = identifier;
            _vertices = ownedVertices; _segments = ownedSegments;
        }
        return self;
    } catch (...) { return nil; }
}
- (core3d::ProfileCurveLoop)nativeLoop { return _native; }
+ (Core3DProfileCurveLoop *)roundedRectangleWithMinimum:(CGPoint)minimum
    maximum:(CGPoint)maximum radius:(double)radius identifier:(uint32_t)identifier
    vertexIdentifiers:(NSArray<NSNumber *> *)vertices segmentIdentifiers:(NSArray<NSNumber *> *)segments {
    try {
        core3d::ProfileCurvePresetIDs<8> ids;
        core3d::ProfileCurveLoop loop;
        if (!Core3DProfilePresetIdentifiers(identifier,vertices,segments,ids)
            || !core3d::BuildRoundedRectangleProfileLoop(gp_Pnt2d(minimum.x,minimum.y),
                gp_Pnt2d(maximum.x,maximum.y),radius,ids,loop)) return nil;
        return Core3DPublicCurveLoop(loop);
    } catch (...) { return nil; }
}

+ (Core3DProfileCurveLoop *)capsuleWithCenter:(CGPoint)center length:(double)length
    diameter:(double)diameter rotationDegrees:(double)rotationDegrees identifier:(uint32_t)identifier
    vertexIdentifiers:(NSArray<NSNumber *> *)vertices segmentIdentifiers:(NSArray<NSNumber *> *)segments {
    try {
        core3d::ProfileCurvePresetIDs<4> ids;
        core3d::ProfileCurveLoop loop;
        if (!Core3DProfilePresetIdentifiers(identifier,vertices,segments,ids)
            || !core3d::BuildCapsuleProfileLoop(gp_Pnt2d(center.x,center.y),length,
                diameter,rotationDegrees,ids,loop)) return nil;
        return Core3DPublicCurveLoop(loop);
    } catch (...) { return nil; }
}

@end

static Core3DProfileCurveLoop *Core3DPublicCurveLoop(const core3d::ProfileCurveLoop& loop) {
    NSMutableArray<Core3DProfileCurveVertex *> *vertices = [NSMutableArray arrayWithCapacity:loop.vertices.size()];
    NSMutableArray<Core3DProfileCurveSegment *> *segments = [NSMutableArray arrayWithCapacity:loop.segments.size()];
    for (const auto& v : loop.vertices) {
        auto value = [[Core3DProfileCurveVertex alloc] initWithIdentifier:v.identifier
            point:CGPointMake(v.point.X(),v.point.Y())];
        if (!value) return nil;
        [vertices addObject:value];
    }
    for (const auto& v : loop.segments) {
        auto value = [[Core3DProfileCurveSegment alloc] initWithIdentifier:v.identifier
            startVertex:v.startVertex endVertex:v.endVertex
            kind:static_cast<Core3DProfileCurveKind>(v.kind)
            center:CGPointMake(v.center.X(),v.center.Y()) radius:v.radius
            startDegrees:v.startDegrees sweepDegrees:v.sweepDegrees];
        if (!value) return nil;
        [segments addObject:value];
    }
    return [[Core3DProfileCurveLoop alloc] initWithIdentifier:loop.identifier vertices:vertices segments:segments];
}


@implementation Core3DSweepPathSegment
- (instancetype)initWithIdentifier:(uint32_t)identifier
    startVertex:(uint32_t)startVertex endVertex:(uint32_t)endVertex
    kind:(Core3DProfileCurveKind)kind center:(CGPoint)center radius:(double)radius
    startDegrees:(double)startDegrees sweepDegrees:(double)sweepDegrees {
    if (identifier == 0 || startVertex == 0 || endVertex == 0 || startVertex == endVertex
        || !std::isfinite(center.x) || !std::isfinite(center.y)
        || !std::isfinite(radius) || !std::isfinite(startDegrees)
        || !std::isfinite(sweepDegrees)) return nil;
    if (kind == Core3DProfileCurveKindLine) {
        if (center.x != 0 || center.y != 0 || radius != 0
            || startDegrees != 0 || sweepDegrees != 0) return nil;
    } else if (kind == Core3DProfileCurveKindCircularArc) {
        if (std::abs(center.x) > 1e6 || std::abs(center.y) > 1e6
            || radius <= 0 || radius > 1e6 || std::abs(startDegrees) > 360
            || std::abs(sweepDegrees) < 1e-6 || std::abs(sweepDegrees) >= 360) return nil;
    } else return nil;
    self = [super init];
    if (self) {
        _identifier = identifier; _startVertex = startVertex; _endVertex = endVertex;
        _kind = kind; _center = center; _radius = radius;
        _startDegrees = startDegrees; _sweepDegrees = sweepDegrees;
    }
    return self;
}
@end

@implementation Core3DRectangularLoftStation
- (instancetype)initWithIdentifier:(uint32_t)identifier cornerIdentifiers:(simd_uint4)corners
    correspondence:(simd_uint4)correspondence z:(double)z centerX:(double)centerX centerY:(double)centerY
    width:(double)width depth:(double)depth {
    if (!identifier || !std::isfinite(z) || !std::isfinite(centerX) || !std::isfinite(centerY)
        || !std::isfinite(width) || !std::isfinite(depth) || width<=0 || depth<=0
        || std::abs(z)>1e6 || std::abs(centerX)>1e6 || std::abs(centerY)>1e6 || width>1e6 || depth>1e6) return nil;
    for(int i=0;i<4;++i)if(!corners[i]||!correspondence[i])return nil;
    self=[super init];if(self){_identifier=identifier;_cornerIdentifiers=corners;_correspondence=correspondence;
        _z=z;_centerX=centerX;_centerY=centerY;_width=width;_depth=depth;}return self;
}
@end
@interface Core3DRectangularLoftStationEdit ()
- (core3d::rectangular_loft::StationDimensionEdit)nativeEdit;
@end
@implementation Core3DRectangularLoftStationEdit
- (instancetype)initWithStationIdentifier:(uint32_t)identifier width:(NSNumber *)width depth:(NSNumber *)depth {
    if(!identifier || (!width && !depth)
        || (width && ![width isKindOfClass:[NSNumber class]])
        || (depth && ![depth isKindOfClass:[NSNumber class]]))return nil;
    for(id value in @[width ?: NSNull.null,depth ?: NSNull.null]) {
        if(value==NSNull.null)continue;
        if(![value isKindOfClass:[NSNumber class]] || CFGetTypeID((__bridge CFTypeRef)value)==CFBooleanGetTypeID()
            || !std::isfinite([value doubleValue]) || [value doubleValue]<=0 || [value doubleValue]>1e6)return nil;
    }
    self=[super init];if(self){_stationIdentifier=identifier;_width=[width copy];_depth=[depth copy];}return self;
}
- (core3d::rectangular_loft::StationDimensionEdit)nativeEdit {
    core3d::rectangular_loft::StationDimensionEdit edit;edit.stationIdentifier=_stationIdentifier;
    if(_width)edit.width=_width.doubleValue;if(_depth)edit.depth=_depth.doubleValue;return edit;
}
@end
@interface Core3DRectangularLoftDefinition ()
- (core3d::rectangular_loft::Definition)nativeDefinition;
- (instancetype)initWithNativeDefinition:(const core3d::rectangular_loft::Definition&)definition;
@end
@implementation Core3DRectangularLoftDefinition {
    core3d::rectangular_loft::Definition _definition;
}
- (instancetype)initWithLoftIdentifier:(uint32_t)identifier correspondence:(simd_uint4)correspondence
    stations:(NSArray<Core3DRectangularLoftStation *> *)stations metersPerUnit:(double)metersPerUnit
    constructionFrameValues:(NSArray<NSNumber *> *)frame {
    if (![stations isKindOfClass:[NSArray class]] || stations.count<2 || stations.count>8
        || ![frame isKindOfClass:[NSArray class]] || (frame.count!=0&&frame.count!=8))return nil;
    try {
        core3d::rectangular_loft::Definition d;d.loftIdentifier=identifier;d.dimensionMetersPerUnit=metersPerUnit;
        for(int i=0;i<4;++i)d.correspondence[i]=correspondence[i];
        for(Core3DRectangularLoftStation *value in stations) {
            if(![value isKindOfClass:[Core3DRectangularLoftStation class]])return nil;
            core3d::rectangular_loft::Station v;v.identifier=value.identifier;
            for(int i=0;i<4;++i){v.cornerIdentifiers[i]=value.cornerIdentifiers[i];v.correspondence[i]=value.correspondence[i];}
            v.z=value.z;v.centerX=value.centerX;v.centerY=value.centerY;v.width=value.width;v.depth=value.depth;d.stations.push_back(v);
        }
        if(frame.count==8){core3d::profile::ConstructionFrame f;for(NSUInteger i=0;i<8;++i){
            if(![frame[i] isKindOfClass:[NSNumber class]]||CFGetTypeID((__bridge CFTypeRef)frame[i])==CFBooleanGetTypeID())return nil;
            f.values[i]=frame[i].doubleValue;}d.constructionFrame=f;}
        core3d::rectangular_loft::Inspection inspection;
        if(core3d::rectangular_loft::Inspect(d,inspection)!=core3d::rectangular_loft::Admission::Accepted)return nil;
        self=[super init];if(self){_definition=std::move(d);_stations=[stations copy];_constructionFrameValues=[frame copy];}
        return self;
    }catch(...){return nil;}
}
- (instancetype)initWithNativeDefinition:(const core3d::rectangular_loft::Definition&)definition {
    try {
        NSMutableArray<Core3DRectangularLoftStation *> *stations=[NSMutableArray array];
        NSMutableArray<NSNumber *> *frame=[NSMutableArray array];
        for(const auto& station:definition.stations) {
            auto value=[[Core3DRectangularLoftStation alloc] initWithIdentifier:station.identifier
                cornerIdentifiers:(simd_uint4){station.cornerIdentifiers[0],station.cornerIdentifiers[1],station.cornerIdentifiers[2],station.cornerIdentifiers[3]}
                correspondence:(simd_uint4){station.correspondence[0],station.correspondence[1],station.correspondence[2],station.correspondence[3]}
                z:station.z centerX:station.centerX centerY:station.centerY width:station.width depth:station.depth];
            if(!value)return nil;[stations addObject:value];
        }
        if(definition.constructionFrame)for(double v:definition.constructionFrame->values)[frame addObject:@(v)];
        return [self initWithLoftIdentifier:definition.loftIdentifier
            correspondence:(simd_uint4){definition.correspondence[0],definition.correspondence[1],definition.correspondence[2],definition.correspondence[3]}
            stations:stations metersPerUnit:definition.dimensionMetersPerUnit constructionFrameValues:frame];
    }catch(...){return nil;}
}
- (Core3DRectangularLoftDefinition *)changingStation:(Core3DRectangularLoftStationEdit *)edit {
    if(![edit isKindOfClass:[Core3DRectangularLoftStationEdit class]])return nil;
    try {
        core3d::rectangular_loft::Definition changed;
        if(!core3d::loft_rebuild::Apply(_definition,[edit nativeEdit],changed))return nil;
        return [[Core3DRectangularLoftDefinition alloc] initWithNativeDefinition:changed];
    }catch(...){return nil;}
}
- (uint32_t)loftIdentifier {return _definition.loftIdentifier;}
- (simd_uint4)correspondence {return {_definition.correspondence[0],_definition.correspondence[1],_definition.correspondence[2],_definition.correspondence[3]};}
- (double)metersPerUnit {return _definition.dimensionMetersPerUnit;}
- (core3d::rectangular_loft::Definition)nativeDefinition {return _definition;}
@end

@interface Core3DSweepDefinition ()
- (core3d::planar_sweep::Definition)nativeDefinition;
@end
@implementation Core3DSweepDefinition {
    core3d::planar_sweep::Definition _definition;
}
- (instancetype)initWithPathIdentifier:(uint32_t)identifier
    vertices:(NSArray<Core3DProfileCurveVertex *> *)vertices
    segments:(NSArray<Core3DSweepPathSegment *> *)segments plane:(Core3DProfilePlane)plane
    radius:(double)radius metersPerUnit:(double)metersPerUnit constructionFrameValues:(NSArray<NSNumber *> *)frame {
    return [self initWithPathIdentifier:identifier vertices:vertices segments:segments plane:plane radius:radius endRadius:radius metersPerUnit:metersPerUnit constructionFrameValues:frame];
}
- (instancetype)initWithPathIdentifier:(uint32_t)identifier
    vertices:(NSArray<Core3DProfileCurveVertex *> *)vertices
    segments:(NSArray<Core3DSweepPathSegment *> *)segments plane:(Core3DProfilePlane)plane
    radius:(double)radius endRadius:(double)endRadius metersPerUnit:(double)metersPerUnit constructionFrameValues:(NSArray<NSNumber *> *)frame {
    if (![vertices isKindOfClass:[NSArray class]] || vertices.count<2 || vertices.count>33
        || ![segments isKindOfClass:[NSArray class]] || segments.count<1 || segments.count>32
        || vertices.count!=segments.count+1 || ![frame isKindOfClass:[NSArray class]]
        || (frame.count!=0 && frame.count!=8)
        || plane<Core3DProfilePlaneXY || plane>Core3DProfilePlaneYZ) return nil;
    try {
        core3d::planar_sweep::Definition d;d.pathIdentifier=identifier;d.plane=int(plane);
        d.radius=radius;d.endRadius=endRadius;d.dimensionMetersPerUnit=metersPerUnit;
        for (Core3DProfileCurveVertex *v in vertices) {
            if (![v isKindOfClass:[Core3DProfileCurveVertex class]]) return nil;
            d.vertices.push_back({v.identifier,gp_Pnt2d(v.point.x,v.point.y)});
        }
        for (Core3DSweepPathSegment *s in segments) {
            if (![s isKindOfClass:[Core3DSweepPathSegment class]]) return nil;
            d.segments.push_back({s.identifier,s.startVertex,s.endVertex,static_cast<core3d::ProfileCurveKind>(s.kind),
                gp_Pnt2d(s.center.x,s.center.y),s.radius,s.startDegrees,s.sweepDegrees});
        }
        if (frame.count==8) {
            core3d::profile::ConstructionFrame f;
            for (NSUInteger i=0;i<8;++i) {
                if (![frame[i] isKindOfClass:[NSNumber class]] || CFGetTypeID((__bridge CFTypeRef)frame[i])==CFBooleanGetTypeID()) return nil;
                f.values[i]=frame[i].doubleValue;
            }
            d.constructionFrame=f;
        }
        core3d::planar_sweep::Inspection inspection;
        if (core3d::planar_sweep::Inspect(d,inspection)!=core3d::planar_sweep::Admission::Accepted) return nil;
        self=[super init];
        if (self) {_definition=std::move(d);_vertices=[vertices copy];_segments=[segments copy];_constructionFrameValues=[frame copy];}
        return self;
    } catch (...) {return nil;}
}
- (uint32_t)pathIdentifier {return _definition.pathIdentifier;}
- (Core3DProfilePlane)plane {return static_cast<Core3DProfilePlane>(_definition.plane);}
- (double)radius {return _definition.radius;}
- (double)endRadius {return _definition.EndRadius();}
- (double)metersPerUnit {return _definition.dimensionMetersPerUnit;}
- (Core3DSweepDefinition *)changingRadius:(double)radius {
    return [self changingStartRadius:radius endRadius:radius];
}
- (Core3DSweepDefinition *)changingStartRadius:(double)radius endRadius:(double)endRadius {
    try {
        auto d=_definition;d.radius=radius;d.endRadius=endRadius;
        core3d::planar_sweep::Inspection inspected;
        if(core3d::planar_sweep::Inspect(d,inspected)!=core3d::planar_sweep::Admission::Accepted)return nil;
        // Rebuild public readonly fields from the already frozen native values,
        // never from potentially subclassed objects supplied to the original init.
        NSMutableArray<Core3DProfileCurveVertex *> *vertices=[NSMutableArray array];
        NSMutableArray<Core3DSweepPathSegment *> *segments=[NSMutableArray array];
        NSMutableArray<NSNumber *> *frame=[NSMutableArray array];
        for(const auto& v:d.vertices){
            auto value=[[Core3DProfileCurveVertex alloc] initWithIdentifier:v.identifier point:CGPointMake(v.point.X(),v.point.Y())];
            if(!value)return nil;[vertices addObject:value];
        }
        for(const auto& v:d.segments){
            auto value=[[Core3DSweepPathSegment alloc] initWithIdentifier:v.identifier startVertex:v.startVertex endVertex:v.endVertex
                kind:static_cast<Core3DProfileCurveKind>(v.kind) center:CGPointMake(v.center.X(),v.center.Y()) radius:v.radius
                startDegrees:v.startDegrees sweepDegrees:v.sweepDegrees];
            if(!value)return nil;[segments addObject:value];
        }
        if(d.constructionFrame)for(double v:d.constructionFrame->values)[frame addObject:@(v)];
        Core3DSweepDefinition *result=[[Core3DSweepDefinition alloc] initWithPathIdentifier:d.pathIdentifier vertices:vertices segments:segments
            plane:static_cast<Core3DProfilePlane>(d.plane) radius:d.radius endRadius:d.EndRadius() metersPerUnit:d.dimensionMetersPerUnit constructionFrameValues:frame];
        if(!result)return nil;
        std::vector<double> expected,actual;
        if(!core3d::sweep_persistence::Encode(d,expected)||!core3d::sweep_persistence::Encode([result nativeDefinition],actual)
            ||!core3d::sweep_persistence::SameBits(expected,actual))return nil;
        return result;
    }catch(...){return nil;}
}
- (core3d::planar_sweep::Definition)nativeDefinition {return _definition;}
@end


@implementation Core3DSavedCutSourceCoordinate
- (instancetype)initWithOrdinal:(uint32_t)ordinal component:(Core3DSavedCutSourceComponent)component valueMM:(double)value {
    if((component!=Core3DSavedCutSourceComponentU&&component!=Core3DSavedCutSourceComponentV)
        ||!std::isfinite(value)||std::abs(value)>1e6)return nil;
    self=[super init];if(self){_ordinal=ordinal;_component=component;_valueMM=value;}return self;
}
@end

@implementation Core3DCylindricalCutDefinition
- (instancetype)initWithAxis:(Core3DCylindricalCutAxis)axis localX:(double)x localY:(double)y localZ:(double)z worldRadiusMM:(double)radius {
    if(axis<Core3DCylindricalCutAxisX||axis>Core3DCylindricalCutAxisZ||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)
        ||!std::isfinite(radius)||radius<.001||radius>1e6)return nil;
    self=[super init];if(self){_axis=axis;_localX=x;_localY=y;_localZ=z;_worldRadiusMM=radius;}return self;
}
@end

@implementation Core3DCylindricalCutRingDefinition
- (instancetype)initWithAxis:(Core3DCylindricalCutAxis)axis localX:(double)x localY:(double)y localZ:(double)z
    boltCircleRadius:(double)boltRadius worldHoleRadiusMM:(double)holeRadius count:(uint32_t)count {
    if(axis<Core3DCylindricalCutAxisX||axis>Core3DCylindricalCutAxisZ||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)
        ||!std::isfinite(boltRadius)||boltRadius<=0||boltRadius>1e6||!std::isfinite(holeRadius)||holeRadius<.001||holeRadius>1e6)return nil;
    self=[super init];if(self){_axis=axis;_localX=x;_localY=y;_localZ=z;
        _boltCircleRadius=boltRadius;_worldHoleRadiusMM=holeRadius;_count=count;}return self;
}
@end

@implementation Core3DRetainedFilletAnchor
- (instancetype)initWithCurveKind:(Core3DRetainedFilletCurveKind)kind
    localX:(double)x localY:(double)y localZ:(double)z axisX:(double)ax axisY:(double)ay axisZ:(double)az circleRadius:(double)radius {
    if((kind!=Core3DRetainedFilletCurveKindLine&&kind!=Core3DRetainedFilletCurveKindCircle)
        ||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||!std::isfinite(ax)||!std::isfinite(ay)||!std::isfinite(az)
        ||!std::isfinite(radius)||std::abs(ax*ax+ay*ay+az*az-1)>1e-12
        ||(kind==Core3DRetainedFilletCurveKindLine?(radius!=0||std::signbit(radius)):radius<=0))return nil;
    self=[super init];if(self){_curveKind=kind;_localX=x;_localY=y;_localZ=z;_axisX=ax;_axisY=ay;_axisZ=az;_circleRadius=radius;}return self;
}
@end

@implementation Core3DWedgeCutDefinition
- (instancetype)initWithAxis:(Core3DCylindricalCutAxis)axis localX:(double)x localY:(double)y localZ:(double)z
    directionAngle:(double)angle worldHalfWidthApexMM:(double)apex worldHalfWidthMouthMM:(double)mouth worldLengthMM:(double)length {
    if(axis<Core3DCylindricalCutAxisX||axis>Core3DCylindricalCutAxisZ||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)
        ||!std::isfinite(angle)||!std::isfinite(apex)||!std::isfinite(mouth)||!std::isfinite(length)
        ||apex<.05||mouth<.05||length<.1||apex>1e6||mouth>1e6||length>1e6)return nil;
    self=[super init];if(self){_axis=axis;_localX=x;_localY=y;_localZ=z;_directionAngle=angle;
        _worldHalfWidthApexMM=apex;_worldHalfWidthMouthMM=mouth;_worldLengthMM=length;}return self;
}
@end

@interface Core3DProfileDefinition ()
- (instancetype)initWithNativeParameters:(const core3d::profile::Parameters&)parameters;
- (core3d::profile::Parameters)nativeParameters;
@end
@implementation Core3DProfileDefinition {
    core3d::profile::Parameters _parameters;
}
- (instancetype)initWithNativeParameters:(const core3d::profile::Parameters&)parameters {
    self = [super init];
    if (self) {
        _parameters = parameters;
        const auto& d = parameters.definition;
        _curveInner = @[];
        if (d.curves) {
            _curveOuter = Core3DPublicCurveLoop(d.curves->outer);
            if (!_curveOuter) return nil;
            NSMutableArray<Core3DProfileCurveLoop *> *inner = [NSMutableArray arrayWithCapacity:d.curves->inner.size()];
            for (const auto& loop : d.curves->inner) {
                auto value = Core3DPublicCurveLoop(loop);
                if (!value) return nil;
                [inner addObject:value];
            }
            _curveInner = [inner copy];
        }
        NSMutableArray *points = [NSMutableArray arrayWithCapacity:d.points.size()];
        for (const auto& p : d.points) [points addObject:Core3DBoxPoint(CGPointMake(p.X(),p.Y()))];
        _points = [points copy];
        if (d.circle) _circleCenter = Core3DBoxPoint(CGPointMake(d.circle->center.X(),d.circle->center.Y()));
        NSMutableArray *centers = [NSMutableArray arrayWithCapacity:d.holes.size()];
        NSMutableArray *radii = [NSMutableArray arrayWithCapacity:d.holes.size()];
        for (const auto& h : d.holes) {
            [centers addObject:Core3DBoxPoint(CGPointMake(h.center.X(),h.center.Y()))];
            [radii addObject:@(h.radius)];
        }
        _holeCenters = [centers copy]; _holeRadii = [radii copy];
    }
    return self;
}
- (instancetype)initWithPoints:(NSArray<NSValue *> *)points
    circleCenter:(NSValue *)circleCenter outerRadius:(double)outerRadius innerRadius:(double)innerRadius
    holeCenters:(NSArray<NSValue *> *)holeCenters holeRadii:(NSArray<NSNumber *> *)holeRadii
    plane:(Core3DProfilePlane)plane parameter:(double)parameter revolve:(BOOL)revolve metersPerUnit:(double)metersPerUnit {
    if (![points isKindOfClass:[NSArray class]] || points.count > 64
        || ![holeCenters isKindOfClass:[NSArray class]] || ![holeRadii isKindOfClass:[NSArray class]]
        || holeCenters.count > 16 || holeCenters.count != holeRadii.count) return nil;
    try {
        core3d::profile::Parameters parameters;
        parameters.metersPerUnit = metersPerUnit;
        auto& d = parameters.definition;
        d.plane = static_cast<int>(plane); d.depth = parameter; d.revolve = revolve;
        auto pointValue = [](id value, gp_Pnt2d& output) {
            if (![value isKindOfClass:[NSValue class]] || std::strcmp([value objCType], @encode(CGPoint)) != 0) return false;
            CGPoint p{};
            [value getValue:&p size:sizeof(p)];
            if (!std::isfinite(p.x) || !std::isfinite(p.y)) return false;
            output = gp_Pnt2d(p.x,p.y); return true;
        };
        for (id value in points) {
            gp_Pnt2d p; if (!pointValue(value,p)) return nil; d.points.push_back(p);
        }
        if (circleCenter) {
            gp_Pnt2d p; if (!pointValue(circleCenter,p)) return nil;
            d.circle = core3d::ProfileCircularSection{p,outerRadius,innerRadius};
        } else if (outerRadius != 0 || innerRadius != 0) return nil;
        for (NSUInteger i=0; i<holeCenters.count; ++i) {
            gp_Pnt2d p; if (!pointValue(holeCenters[i],p) || ![holeRadii[i] isKindOfClass:[NSNumber class]]) return nil;
            d.holes.push_back({p,[holeRadii[i] doubleValue]});
        }
        std::vector<double> values;
        if (!core3d::profile::Encode(parameters,values)) return nil;
        return [self initWithNativeParameters:parameters];
    } catch (...) { return nil; }
}
- (instancetype)initWithCurveOuter:(Core3DProfileCurveLoop *)outer
    inner:(NSArray<Core3DProfileCurveLoop *> *)inner plane:(Core3DProfilePlane)plane
    parameter:(double)parameter revolve:(BOOL)revolve metersPerUnit:(double)metersPerUnit {
    if (![outer isMemberOfClass:[Core3DProfileCurveLoop class]]
        || ![inner isKindOfClass:[NSArray class]] || inner.count > 16) return nil;
    NSArray *ownedInner = [inner copy];
    try {
        std::size_t count = outer.vertices.count;
        for (id value in ownedInner) {
            if (![value isMemberOfClass:[Core3DProfileCurveLoop class]]) return nil;
            Core3DProfileCurveLoop *loop = value;
            if (loop.vertices.count > 512 - count) return nil;
            count += loop.vertices.count;
        }
        core3d::profile::Parameters parameters;
        parameters.metersPerUnit = metersPerUnit;
        auto& definition = parameters.definition;
        definition.plane = static_cast<int>(plane); definition.depth = parameter; definition.revolve = revolve;
        definition.curves.emplace(); definition.curves->outer = [outer nativeLoop];
        definition.curves->inner.reserve(ownedInner.count);
        for (Core3DProfileCurveLoop *loop in ownedInner)
            definition.curves->inner.push_back([loop nativeLoop]);
        std::vector<double> values;
        if (!core3d::profile::Encode(parameters,values)) return nil;
        return [self initWithNativeParameters:parameters];
    } catch (...) { return nil; }
}
- (Core3DProfileDefinition *)definitionByChangingParameter:(double)parameter {
    try {
        auto requested = _parameters;
        requested.definition.depth = parameter;
        std::vector<double> encoded;
        if (!core3d::profile::Encode(requested,encoded)) return nil;
        return [[Core3DProfileDefinition alloc] initWithNativeParameters:requested];
    } catch (...) { return nil; }
}
- (core3d::profile::Parameters)nativeParameters { return _parameters; }
- (double)outerRadius { return _parameters.definition.circle ? _parameters.definition.circle->outerRadius : 0; }
- (double)innerRadius { return _parameters.definition.circle ? _parameters.definition.circle->innerRadius : 0; }
- (Core3DProfilePlane)plane { return static_cast<Core3DProfilePlane>(_parameters.definition.plane); }
- (double)parameter { return _parameters.definition.depth; }
- (BOOL)revolve { return _parameters.definition.revolve; }
- (double)metersPerUnit { return _parameters.metersPerUnit; }
- (NSArray<NSNumber *> *)constructionFrameValues {
    if (!_parameters.constructionFrame) return @[];
    NSMutableArray<NSNumber *> *values = [NSMutableArray arrayWithCapacity:8];
    for (double value : _parameters.constructionFrame->values) [values addObject:@(value)];
    return [values copy];
}
@end

@interface Core3DEnclosureDefinition ()
- (instancetype)initWithNativeParameters:(const core3d::enclosure::Parameters&)parameters;
- (core3d::enclosure::Parameters)nativeParameters;
@end
@implementation Core3DEnclosureDefinition {
    core3d::enclosure::Parameters _parameters;
}
- (instancetype)initWithNativeParameters:(const core3d::enclosure::Parameters&)parameters {
    try {
        std::vector<double> values;
        if (!core3d::enclosure::Encode(parameters,values)) return nil;
        self = [super init];
        if (self) _parameters = parameters;
        return self;
    } catch (...) { return nil; }
}
- (instancetype)initWithWidth:(double)width depth:(double)depth height:(double)height
    wall:(double)wall floor:(double)floor cornerRadius:(double)cornerRadius
    plane:(Core3DProfilePlane)plane metersPerUnit:(double)metersPerUnit {
    // Check NSInteger before narrowing to the native int/enum representation.
    if (plane < Core3DProfilePlaneXY || plane > Core3DProfilePlaneYZ) return nil;
    core3d::enclosure::Parameters parameters;
    parameters.definition.dimensions = {width,depth,height,wall,floor,cornerRadius};
    parameters.definition.plane = static_cast<int>(plane);
    parameters.metersPerUnit = metersPerUnit;
    return [self initWithNativeParameters:parameters];
}
- (Core3DEnclosureDefinition *)definitionByUpdatingDimension:(Core3DEnclosureDimension)dimension value:(double)value {
    if (dimension < Core3DEnclosureDimensionWidth || dimension > Core3DEnclosureDimensionCornerRadius) return nil;
    try {
        const auto updated = core3d::UpdatedEnclosureDimension(_parameters.definition,
            static_cast<core3d::EnclosureDimension>(dimension),value);
        if (!updated) return nil;
        auto parameters = _parameters; parameters.definition = *updated;
        return [[Core3DEnclosureDefinition alloc] initWithNativeParameters:parameters];
    } catch (...) { return nil; }
}
- (Core3DEnclosureDefinition *)definitionByUpdatingWidth:(double)width depth:(double)depth
    height:(double)height wall:(double)wall floor:(double)floor cornerRadius:(double)cornerRadius
    plane:(Core3DProfilePlane)plane {
    if (plane < Core3DProfilePlaneXY || plane > Core3DProfilePlaneYZ) return nil;
    try {
        auto parameters = _parameters;
        parameters.definition.dimensions = {width,depth,height,wall,floor,cornerRadius};
        parameters.definition.plane = static_cast<int>(plane);
        return [[Core3DEnclosureDefinition alloc] initWithNativeParameters:parameters];
    } catch (...) { return nil; }
}
- (double)width { return _parameters.definition.dimensions.width; }
- (double)depth { return _parameters.definition.dimensions.depth; }
- (double)height { return _parameters.definition.dimensions.height; }
- (double)wall { return _parameters.definition.dimensions.wall; }
- (double)floor { return _parameters.definition.dimensions.floor; }
- (double)cornerRadius { return _parameters.definition.dimensions.cornerRadius; }
- (Core3DProfilePlane)plane { return static_cast<Core3DProfilePlane>(_parameters.definition.plane); }
- (double)metersPerUnit { return _parameters.metersPerUnit; }
- (core3d::enclosure::Parameters)nativeParameters { return _parameters; }
@end
