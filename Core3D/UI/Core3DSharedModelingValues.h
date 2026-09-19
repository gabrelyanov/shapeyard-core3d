#pragma once

// Platform-neutral immutable modeling recipes. These declarations have concrete
// implementations in Core3DModelingTypes.mm and carry no controller, viewer,
// document, selection, transaction, or UI authority.
#import <Foundation/Foundation.h>
#import <CoreGraphics/CoreGraphics.h>
#import <simd/simd.h>

NS_ASSUME_NONNULL_BEGIN

typedef NS_ENUM(NSInteger, Core3DProfilePlane) {
    Core3DProfilePlaneXY = 0, Core3DProfilePlaneXZ, Core3DProfilePlaneYZ,
};

//! Explicit curve values carry feature-local stable IDs. Coordinates and radii
//! use the enclosing profile's declared document unit; angles are degrees.
//! These immutable values contain no OCCT handles or live selection authority.
typedef NS_ENUM(NSInteger, Core3DProfileCurveKind) {
    Core3DProfileCurveKindLine = 0, Core3DProfileCurveKindCircularArc = 1,
};
__attribute__((objc_subclassing_restricted))
@interface Core3DProfileCurveVertex : NSObject
@property(nonatomic,readonly) uint32_t identifier;
@property(nonatomic,readonly) CGPoint point;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithIdentifier:(uint32_t)identifier point:(CGPoint)point
    NS_SWIFT_NAME(init(identifier:point:));
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DProfileCurveSegment : NSObject
@property(nonatomic,readonly) uint32_t identifier;
@property(nonatomic,readonly) uint32_t startVertex;
@property(nonatomic,readonly) uint32_t endVertex;
@property(nonatomic,readonly) Core3DProfileCurveKind kind;
//! Unused line parameters are exactly zero. Arc endpoints must agree with the
//! referenced vertices; this is validated when constructing the full profile.
@property(nonatomic,readonly) CGPoint center;
@property(nonatomic,readonly) double radius;
@property(nonatomic,readonly) double startDegrees;
@property(nonatomic,readonly) double sweepDegrees;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithIdentifier:(uint32_t)identifier
    startVertex:(uint32_t)startVertex endVertex:(uint32_t)endVertex
    kind:(Core3DProfileCurveKind)kind center:(CGPoint)center radius:(double)radius
    startDegrees:(double)startDegrees sweepDegrees:(double)sweepDegrees
    NS_SWIFT_NAME(init(identifier:startVertex:endVertex:kind:center:radius:startDegrees:sweepDegrees:));
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DProfileCurveLoop : NSObject
@property(nonatomic,readonly) uint32_t identifier;
@property(nonatomic,copy,readonly) NSArray<Core3DProfileCurveVertex *> *vertices;
@property(nonatomic,copy,readonly) NSArray<Core3DProfileCurveSegment *> *segments;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
//! Ordered shared endpoints, 2-512 vertices and equally many segments. Full
//! profile admission checks closed topology, intersections and hole clearance.
- (nullable instancetype)initWithIdentifier:(uint32_t)identifier
    vertices:(NSArray<Core3DProfileCurveVertex *> *)vertices
    segments:(NSArray<Core3DProfileCurveSegment *> *)segments
    NS_SWIFT_NAME(init(identifier:vertices:segments:));
//! Presets return explicit editable curves. The caller supplies stable IDs;
//! assembling a profile still validates cross-loop identity and clearance.
+ (nullable Core3DProfileCurveLoop *)roundedRectangleWithMinimum:(CGPoint)minimum
    maximum:(CGPoint)maximum radius:(double)radius identifier:(uint32_t)identifier
    vertexIdentifiers:(NSArray<NSNumber *> *)vertices segmentIdentifiers:(NSArray<NSNumber *> *)segments
    NS_SWIFT_NAME(roundedRectangle(minimum:maximum:radius:identifier:vertexIdentifiers:segmentIdentifiers:));
+ (nullable Core3DProfileCurveLoop *)capsuleWithCenter:(CGPoint)center length:(double)length
    diameter:(double)diameter rotationDegrees:(double)rotationDegrees identifier:(uint32_t)identifier
    vertexIdentifiers:(NSArray<NSNumber *> *)vertices segmentIdentifiers:(NSArray<NSNumber *> *)segments
    NS_SWIFT_NAME(capsule(center:length:diameter:rotationDegrees:identifier:vertexIdentifiers:segmentIdentifiers:));
@end

//! Sweep arc radii are positive native scalars; physical minima and G1 admission
//! belong to Core3DSweepDefinition. Existing profile DTO bounds are unchanged.
__attribute__((objc_subclassing_restricted))
@interface Core3DSweepPathSegment : NSObject
@property(nonatomic,readonly) uint32_t identifier;
@property(nonatomic,readonly) uint32_t startVertex;
@property(nonatomic,readonly) uint32_t endVertex;
@property(nonatomic,readonly) Core3DProfileCurveKind kind;
//! Unused line parameters are exactly zero. Arc endpoints must agree with the
//! referenced vertices; this is validated when constructing the full profile.
@property(nonatomic,readonly) CGPoint center;
@property(nonatomic,readonly) double radius;
@property(nonatomic,readonly) double startDegrees;
@property(nonatomic,readonly) double sweepDegrees;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithIdentifier:(uint32_t)identifier
    startVertex:(uint32_t)startVertex endVertex:(uint32_t)endVertex
    kind:(Core3DProfileCurveKind)kind center:(CGPoint)center radius:(double)radius
    startDegrees:(double)startDegrees sweepDegrees:(double)sweepDegrees
    NS_SWIFT_NAME(init(identifier:startVertex:endVertex:kind:center:radius:startDegrees:sweepDegrees:));
@end

//! Immutable bounded planar G1 open line/arc path with a constant or linearly tapered solid-circle section.
//! Lengths and frame translations use raw document units. No target/lease/receipt authority.
__attribute__((objc_subclassing_restricted))
@interface Core3DSweepDefinition : NSObject
@property(nonatomic,readonly) uint32_t pathIdentifier;
@property(nonatomic,copy,readonly) NSArray<Core3DProfileCurveVertex *> *vertices;
@property(nonatomic,copy,readonly) NSArray<Core3DSweepPathSegment *> *segments;
@property(nonatomic,readonly) Core3DProfilePlane plane;
@property(nonatomic,readonly) double radius;
@property(nonatomic,readonly) double endRadius;
@property(nonatomic,readonly) double metersPerUnit;
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *constructionFrameValues;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
//! Open path:2…33 vertices,1…32 segments. Frame is empty or XYZ/quaternion XYZW/signed scale.
- (nullable instancetype)initWithPathIdentifier:(uint32_t)identifier
    vertices:(NSArray<Core3DProfileCurveVertex *> *)vertices
    segments:(NSArray<Core3DSweepPathSegment *> *)segments plane:(Core3DProfilePlane)plane
    radius:(double)radius metersPerUnit:(double)metersPerUnit
    constructionFrameValues:(NSArray<NSNumber *> *)frame
    NS_SWIFT_NAME(init(pathIdentifier:vertices:segments:plane:radius:metersPerUnit:constructionFrameValues:));
- (nullable instancetype)initWithPathIdentifier:(uint32_t)identifier
    vertices:(NSArray<Core3DProfileCurveVertex *> *)vertices
    segments:(NSArray<Core3DSweepPathSegment *> *)segments plane:(Core3DProfilePlane)plane
    radius:(double)radius endRadius:(double)endRadius metersPerUnit:(double)metersPerUnit
    constructionFrameValues:(NSArray<NSNumber *> *)frame
    NS_SWIFT_NAME(init(pathIdentifier:vertices:segments:plane:radius:endRadius:metersPerUnit:constructionFrameValues:));
//! Clone only the raw feature-unit circle radius. All path/frame/unit bits are retained.
- (nullable Core3DSweepDefinition *)changingRadius:(double)radius NS_SWIFT_NAME(changingRadius(_:));
- (nullable Core3DSweepDefinition *)changingStartRadius:(double)radius endRadius:(double)endRadius NS_SWIFT_NAME(changingStartRadius(_:endRadius:));
@end

//! Immutable parallel-XY rectangular station. IDs are local recipe identity, not object UUIDs.
__attribute__((objc_subclassing_restricted))
@interface Core3DRectangularLoftStation : NSObject
@property(nonatomic,readonly) uint32_t identifier;
@property(nonatomic,readonly) simd_uint4 cornerIdentifiers;
@property(nonatomic,readonly) simd_uint4 correspondence;
@property(nonatomic,readonly) double z;
@property(nonatomic,readonly) double centerX;
@property(nonatomic,readonly) double centerY;
@property(nonatomic,readonly) double width;
@property(nonatomic,readonly) double depth;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithIdentifier:(uint32_t)identifier cornerIdentifiers:(simd_uint4)corners
    correspondence:(simd_uint4)correspondence z:(double)z centerX:(double)centerX centerY:(double)centerY
    width:(double)width depth:(double)depth
    NS_SWIFT_NAME(init(identifier:cornerIdentifiers:correspondence:z:centerX:centerY:width:depth:));
@end

//! One existing station's dimensions in raw feature units; nil preserves the original.
//! This numeric edit carries no document, OCAF or receipt authority.
__attribute__((objc_subclassing_restricted))
@interface Core3DRectangularLoftStationEdit : NSObject
@property(nonatomic,readonly) uint32_t stationIdentifier;
@property(nonatomic,copy,readonly,nullable) NSNumber *width;
@property(nonatomic,copy,readonly,nullable) NSNumber *depth;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithStationIdentifier:(uint32_t)identifier
    width:(nullable NSNumber *)width depth:(nullable NSNumber *)depth
    NS_SWIFT_NAME(init(stationIdentifier:width:depth:));
@end

//! Bounded ruled solid:2…8 strictly ordered parallel XY rectangular stations.
//! Numeric recipe IDs and all station order are immutable. Complete native admission applies.
__attribute__((objc_subclassing_restricted))
@interface Core3DRectangularLoftDefinition : NSObject
@property(nonatomic,readonly) uint32_t loftIdentifier;
@property(nonatomic,readonly) simd_uint4 correspondence;
@property(nonatomic,copy,readonly) NSArray<Core3DRectangularLoftStation *> *stations;
@property(nonatomic,readonly) double metersPerUnit;
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *constructionFrameValues;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithLoftIdentifier:(uint32_t)identifier correspondence:(simd_uint4)correspondence
    stations:(NSArray<Core3DRectangularLoftStation *> *)stations metersPerUnit:(double)metersPerUnit
    constructionFrameValues:(NSArray<NSNumber *> *)frame
    NS_SWIFT_NAME(init(loftIdentifier:correspondence:stations:metersPerUnit:constructionFrameValues:));
- (nullable Core3DRectangularLoftDefinition *)changingStation:(Core3DRectangularLoftStationEdit *)edit
    NS_SWIFT_NAME(changingStation(_:));
@end

//! One source-recipe polygon coordinate in physical millimetres.
typedef NS_ENUM(NSInteger, Core3DSavedCutSourceComponent) { Core3DSavedCutSourceComponentU NS_SWIFT_NAME(u)=0, Core3DSavedCutSourceComponentV NS_SWIFT_NAME(v)=1 };
__attribute__((objc_subclassing_restricted))
@interface Core3DSavedCutSourceCoordinate : NSObject
@property(nonatomic,readonly) uint32_t ordinal; // Original polygon order; not a persistent vertex ID.
@property(nonatomic,readonly) Core3DSavedCutSourceComponent component;
@property(nonatomic,readonly) double valueMM;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithOrdinal:(uint32_t)ordinal component:(Core3DSavedCutSourceComponent)component valueMM:(double)value
    NS_SWIFT_NAME(init(ordinal:component:valueMM:));
@end

//! Axis and centre are original object-local coordinates, in document units.
//! Radius is physical world millimetres; only positive uniform occurrence scale is supported.
typedef NS_ENUM(NSInteger, Core3DCylindricalCutAxis) { Core3DCylindricalCutAxisX=0,Core3DCylindricalCutAxisY=1,Core3DCylindricalCutAxisZ=2 };
__attribute__((objc_subclassing_restricted))
@interface Core3DCylindricalCutDefinition : NSObject
@property(nonatomic,readonly) Core3DCylindricalCutAxis axis;
@property(nonatomic,readonly) double localX;
@property(nonatomic,readonly) double localY;
@property(nonatomic,readonly) double localZ;
@property(nonatomic,readonly) double worldRadiusMM;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithAxis:(Core3DCylindricalCutAxis)axis localX:(double)x localY:(double)y localZ:(double)z worldRadiusMM:(double)radius;
@end

//! Ring values; count, chord and host clearances are admitted by the native edit.
__attribute__((objc_subclassing_restricted))
@interface Core3DCylindricalCutRingDefinition : NSObject
@property(nonatomic,readonly) Core3DCylindricalCutAxis axis;
@property(nonatomic,readonly) double localX;
@property(nonatomic,readonly) double localY;
@property(nonatomic,readonly) double localZ;
@property(nonatomic,readonly) double boltCircleRadius;
@property(nonatomic,readonly) double worldHoleRadiusMM;
@property(nonatomic,readonly) NSUInteger count;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithAxis:(Core3DCylindricalCutAxis)axis localX:(double)x localY:(double)y localZ:(double)z
    boltCircleRadius:(double)boltRadius worldHoleRadiusMM:(double)holeRadius count:(uint32_t)count
    NS_SWIFT_NAME(init(axis:localX:localY:localZ:boltCircleRadius:worldHoleRadiusMM:count:));
@end

//! Trapezoid through-cut values; the native program admits source and tool clearances.
__attribute__((objc_subclassing_restricted))
@interface Core3DWedgeCutDefinition : NSObject
@property(nonatomic,readonly) Core3DCylindricalCutAxis axis;
@property(nonatomic,readonly) double localX;
@property(nonatomic,readonly) double localY;
@property(nonatomic,readonly) double localZ;
@property(nonatomic,readonly) double directionAngle;
@property(nonatomic,readonly) double worldHalfWidthApexMM;
@property(nonatomic,readonly) double worldHalfWidthMouthMM;
@property(nonatomic,readonly) double worldLengthMM;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithAxis:(Core3DCylindricalCutAxis)axis localX:(double)x localY:(double)y localZ:(double)z
    directionAngle:(double)angle worldHalfWidthApexMM:(double)apex worldHalfWidthMouthMM:(double)mouth worldLengthMM:(double)length
    NS_SWIFT_NAME(init(axis:localX:localY:localZ:directionAngle:worldHalfWidthApexMM:worldHalfWidthMouthMM:worldLengthMM:));
@end

//! Edge anchors use the original object's document-local coordinates.
typedef NS_ENUM(NSInteger, Core3DRetainedFilletCurveKind) { Core3DRetainedFilletCurveKindLine=1, Core3DRetainedFilletCurveKindCircle=2 };
__attribute__((objc_subclassing_restricted))
@interface Core3DRetainedFilletAnchor : NSObject
@property(nonatomic,readonly) Core3DRetainedFilletCurveKind curveKind;
@property(nonatomic,readonly) double localX;
@property(nonatomic,readonly) double localY;
@property(nonatomic,readonly) double localZ;
@property(nonatomic,readonly) double axisX;
@property(nonatomic,readonly) double axisY;
@property(nonatomic,readonly) double axisZ;
@property(nonatomic,readonly) double circleRadius;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithCurveKind:(Core3DRetainedFilletCurveKind)kind
    localX:(double)x localY:(double)y localZ:(double)z axisX:(double)ax axisY:(double)ay axisZ:(double)az circleRadius:(double)radius
    NS_SWIFT_NAME(init(curveKind:localX:localY:localZ:axisX:axisY:axisZ:circleRadius:));
@end

//! Immutable validated construction values. Lengths use the declared document
//! unit; revolution parameters are degrees. No geometry or document handles.
@interface Core3DProfileDefinition : NSObject
//! Non-nil only for explicit line/arc profiles. Legacy outline/circle/hole
//! properties are empty in that case; callers must present the curve editor.
@property(nonatomic,strong,readonly,nullable) Core3DProfileCurveLoop *curveOuter;
@property(nonatomic,copy,readonly) NSArray<Core3DProfileCurveLoop *> *curveInner;
@property(nonatomic,copy,readonly) NSArray<NSValue *> *points;
@property(nonatomic,copy,readonly,nullable) NSValue *circleCenter;
@property(nonatomic,readonly) double outerRadius;
@property(nonatomic,readonly) double innerRadius;
@property(nonatomic,copy,readonly) NSArray<NSValue *> *holeCenters;
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *holeRadii;
@property(nonatomic,readonly) Core3DProfilePlane plane;
@property(nonatomic,readonly) double parameter;
@property(nonatomic,readonly) BOOL revolve;
@property(nonatomic,readonly) double metersPerUnit;
//! Empty for legacy identity. Otherwise translation XYZ in document units,
//! proper unit quaternion XYZW, signed uniform scale. Read-only local frame.
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *constructionFrameValues;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithPoints:(NSArray<NSValue *> *)points
    circleCenter:(nullable NSValue *)circleCenter outerRadius:(double)outerRadius innerRadius:(double)innerRadius
    holeCenters:(NSArray<NSValue *> *)holeCenters holeRadii:(NSArray<NSNumber *> *)holeRadii
    plane:(Core3DProfilePlane)plane parameter:(double)parameter revolve:(BOOL)revolve metersPerUnit:(double)metersPerUnit
    NS_SWIFT_NAME(init(points:circleCenter:outerRadius:innerRadius:holeCenters:holeRadii:plane:parameter:revolve:metersPerUnit:));
//! At most17 loops and512 total vertices/segments with unique positive IDs.
- (nullable instancetype)initWithCurveOuter:(Core3DProfileCurveLoop *)outer
    inner:(NSArray<Core3DProfileCurveLoop *> *)inner plane:(Core3DProfilePlane)plane
    parameter:(double)parameter revolve:(BOOL)revolve metersPerUnit:(double)metersPerUnit
    NS_SWIFT_NAME(init(curveOuter:inner:plane:parameter:revolve:metersPerUnit:));
//! Validated native clone changing only depth or angle. Retains all other scalar
//! bits, contours, stable curve IDs, units and construction frame unchanged.
//! No edit authority or document/history mutation is created by this value.
- (nullable Core3DProfileDefinition *)definitionByChangingParameter:(double)parameter
    NS_SWIFT_NAME(changingParameter(_:));
@end

typedef NS_ENUM(NSInteger, Core3DEnclosureDimension) {
    Core3DEnclosureDimensionWidth = 0,
    Core3DEnclosureDimensionDepth,
    Core3DEnclosureDimensionHeight,
    Core3DEnclosureDimensionWall,
    Core3DEnclosureDimensionFloor,
    Core3DEnclosureDimensionCornerRadius,
};

//! Immutable native construction values. Lengths use the declared document
//! unit. These values carry no geometry, document handles or edit authority.
__attribute__((objc_subclassing_restricted))
@interface Core3DEnclosureDefinition : NSObject
@property(nonatomic,readonly) double width;
@property(nonatomic,readonly) double depth;
@property(nonatomic,readonly) double height;
@property(nonatomic,readonly) double wall;
@property(nonatomic,readonly) double floor;
@property(nonatomic,readonly) double cornerRadius;
@property(nonatomic,readonly) Core3DProfilePlane plane;
@property(nonatomic,readonly) double metersPerUnit;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithWidth:(double)width depth:(double)depth height:(double)height
    wall:(double)wall floor:(double)floor cornerRadius:(double)cornerRadius
    plane:(Core3DProfilePlane)plane metersPerUnit:(double)metersPerUnit
    NS_SWIFT_NAME(init(width:depth:height:wall:floor:cornerRadius:plane:metersPerUnit:));
//! Returns validated values only; does not change a document or add history.
- (nullable Core3DEnclosureDefinition *)definitionByUpdatingDimension:(Core3DEnclosureDimension)dimension
    value:(double)value NS_SWIFT_NAME(updating(_:value:));
//! Validates all dimensional changes atomically, retaining exact units/frame.
- (nullable Core3DEnclosureDefinition *)definitionByUpdatingWidth:(double)width depth:(double)depth
    height:(double)height wall:(double)wall floor:(double)floor cornerRadius:(double)cornerRadius
    plane:(Core3DProfilePlane)plane NS_SWIFT_NAME(updating(width:depth:height:wall:floor:cornerRadius:plane:));
@end

NS_ASSUME_NONNULL_END

