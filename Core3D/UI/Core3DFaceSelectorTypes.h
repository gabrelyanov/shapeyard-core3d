#import <Foundation/Foundation.h>
#import <Core3D/Core3DEdgeTreatmentTypes.h>

NS_ASSUME_NONNULL_BEGIN

typedef NS_ENUM(NSInteger, Core3DFaceSelectorKind) {
    Core3DFaceSelectorKindPlanarFaceBoundary = 1,
    Core3DFaceSelectorKindLine = 2,
    Core3DFaceSelectorKindCircleRim = 3,
    Core3DFaceSelectorKindSplineEdge = 4,
};
typedef NS_ENUM(NSInteger, Core3DFaceSelectorAxis) {
    Core3DFaceSelectorAxisX = 1, Core3DFaceSelectorAxisY = 2, Core3DFaceSelectorAxisZ = 3,
};
typedef NS_ENUM(NSInteger, Core3DFaceSelectorSide) {
    Core3DFaceSelectorSideMinimum = 1, Core3DFaceSelectorSideMaximum = 2,
};
typedef NS_ENUM(NSInteger, Core3DFaceSelectorBoundaryCurve) { Core3DFaceSelectorBoundaryCurveLine = 1 };
typedef NS_ENUM(NSInteger, Core3DFaceUseDirection) {
    Core3DFaceUseDirectionForward = 1, Core3DFaceUseDirectionReversed = 2,
};
typedef NS_ENUM(NSInteger, Core3DFaceSelectorCoverage) {
    Core3DFaceSelectorCoverageEntireBoundary = 1, Core3DFaceSelectorCoverageSelectedLine = 2,
};
typedef NS_ENUM(NSInteger, Core3DFaceSelectorWirePosition) {
    Core3DFaceSelectorWirePositionOuter = 1, Core3DFaceSelectorWirePositionInner = 2,
};
typedef NS_ENUM(NSInteger, Core3DFaceSelectorStatus) {
    Core3DFaceSelectorStatusProved = 1, Core3DFaceSelectorStatusRefused = 2,
    Core3DFaceSelectorStatusStale = 3, Core3DFaceSelectorStatusCancelled = 4,
    Core3DFaceSelectorStatusBudget = 5, Core3DFaceSelectorStatusBusy = 6,
    Core3DFaceSelectorStatusFailed = 7,
};
typedef NS_ENUM(NSInteger, Core3DFaceSelectorRefusal) {
    Core3DFaceSelectorRefusalNone = 0, Core3DFaceSelectorRefusalInvalidIntent,
    Core3DFaceSelectorRefusalFaceMissing, Core3DFaceSelectorRefusalFaceAmbiguous,
    Core3DFaceSelectorRefusalIncompleteBoundary, Core3DFaceSelectorRefusalInvalidFaceUse,
    Core3DFaceSelectorRefusalOwnershipMismatch, Core3DFaceSelectorRefusalExactCount,
    Core3DFaceSelectorRefusalPartialCircle, Core3DFaceSelectorRefusalStaleSource,
    Core3DFaceSelectorRefusalUnsupportedTransform, Core3DFaceSelectorRefusalSplitEdge,
    Core3DFaceSelectorRefusalTruncatedDiscovery, Core3DFaceSelectorRefusalSplineIdentityUnavailable,
    Core3DFaceSelectorRefusalReplayMismatch, Core3DFaceSelectorRefusalUnsupportedCarrier,
    Core3DFaceSelectorRefusalAdmissionUnavailable, Core3DFaceSelectorRefusalBudget,
    Core3DFaceSelectorRefusalCancelled, Core3DFaceSelectorRefusalBusy,
    Core3DFaceSelectorRefusalNativeFailure, Core3DFaceSelectorRefusalReadOnlyInvariant,
};

__attribute__((objc_subclassing_restricted))
@interface Core3DFaceSelectorVector3 : NSObject <NSCopying>
@property(nonatomic, readonly) double x, y, z;
- (nullable instancetype)initWithX:(double)x y:(double)y z:(double)z NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DPlanarFaceScope : NSObject <NSCopying>
@property(nonatomic, readonly) Core3DFaceSelectorAxis axis;
@property(nonatomic, readonly) Core3DFaceSelectorSide side;
- (nullable instancetype)initWithAxis:(Core3DFaceSelectorAxis)axis
    side:(Core3DFaceSelectorSide)side NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DPlanarFaceBoundaryIntent : NSObject <NSCopying>
@property(nonatomic, strong, readonly) Core3DPlanarFaceScope *face;
@property(nonatomic, readonly) Core3DFaceSelectorBoundaryCurve edgeKind;
@property(nonatomic, readonly) NSUInteger expectedCount;
- (nullable instancetype)initWithFace:(Core3DPlanarFaceScope *)face
    edgeKind:(Core3DFaceSelectorBoundaryCurve)edgeKind expectedCount:(NSUInteger)expectedCount;
- (instancetype)init NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DLineSelectorIntent : NSObject <NSCopying>
@property(nonatomic, strong, readonly) Core3DPlanarFaceScope *face;
@property(nonatomic, strong, readonly) Core3DFaceSelectorVector3 *locationMM, *direction;
@property(nonatomic, readonly) NSUInteger expectedCount;
- (nullable instancetype)initWithFace:(Core3DPlanarFaceScope *)face
    locationMM:(Core3DFaceSelectorVector3 *)locationMM
    direction:(Core3DFaceSelectorVector3 *)direction expectedCount:(NSUInteger)expectedCount;
- (instancetype)init NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DCircleRimIntent : NSObject <NSCopying>
@property(nonatomic, strong, readonly) Core3DFaceSelectorVector3 *centerMM, *normal;
@property(nonatomic, readonly) double radiusMM;
@property(nonatomic, readonly) NSUInteger expectedCount;
- (nullable instancetype)initWithCenterMM:(Core3DFaceSelectorVector3 *)centerMM
    normal:(Core3DFaceSelectorVector3 *)normal radiusMM:(double)radiusMM expectedCount:(NSUInteger)expectedCount;
- (instancetype)init NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DReservedSplineEdgeIntent : NSObject <NSCopying>
@property(nonatomic, copy, readonly) NSUUID *sourceFeatureIdentifier, *featureLocalEdgeKey;
@property(nonatomic, readonly) NSUInteger expectedCount;
- (nullable instancetype)initWithSourceFeatureIdentifier:(NSUUID *)sourceFeatureIdentifier
    featureLocalEdgeKey:(NSUUID *)featureLocalEdgeKey expectedCount:(NSUInteger)expectedCount;
- (instancetype)init NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DFaceSelectorIntent : NSObject <NSCopying>
@property(nonatomic, readonly) Core3DFaceSelectorKind kind;
@property(nonatomic, strong, readonly, nullable) Core3DPlanarFaceBoundaryIntent *planarFaceBoundary;
@property(nonatomic, strong, readonly, nullable) Core3DLineSelectorIntent *line;
@property(nonatomic, strong, readonly, nullable) Core3DCircleRimIntent *circleRim;
@property(nonatomic, strong, readonly, nullable) Core3DReservedSplineEdgeIntent *splineEdge;
+ (instancetype)planarFaceBoundary:(Core3DPlanarFaceBoundaryIntent *)value;
+ (instancetype)line:(Core3DLineSelectorIntent *)value;
+ (instancetype)circleRim:(Core3DCircleRimIntent *)value;
+ (instancetype)splineEdge:(Core3DReservedSplineEdgeIntent *)value;
- (instancetype)init NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DFaceBoundaryUse : NSObject
@property(nonatomic, readonly) Core3DFaceSelectorWirePosition outerOrInnerWire;
@property(nonatomic, readonly, getter=isSelected) BOOL selected;
@property(nonatomic, readonly) Core3DEdgeTreatmentCurveKind curveKind;
@property(nonatomic, readonly) Core3DFaceUseDirection canonicalFaceUseDirection;
@property(nonatomic, readonly) NSUInteger ownerFaceCount;
@property(nonatomic, readonly) BOOL selectedFaceIsOwner;
- (instancetype)init NS_UNAVAILABLE;
@end

// Core3DFaceSelectorProof intentionally carries no objc_subclassing_restricted:
// the internal Core3DFaceSelectorNativeProof carrier subclasses it, matching
// Core3DFaceSelectorProofR2. init/new remain unavailable, so construction
// stays gated through the typed factories.
@interface Core3DFaceSelectorProof : NSObject
@property(nonatomic, strong, readonly) Core3DFaceSelectorIntent *intent;
@property(nonatomic, strong, readonly) Core3DFaceSelectorVector3 *outwardNormal;
@property(nonatomic, readonly) double planeOffsetMM;
@property(nonatomic, readonly) Core3DFaceSelectorCoverage coverage;
@property(nonatomic, readonly) NSUInteger matchedFaceCount, wireCount, boundaryUseCount;
@property(nonatomic, readonly) NSUInteger boundaryUniqueEdgeCount, selectedEdgeCount;
@property(nonatomic, readonly) BOOL discoveryComplete, truncated;
@property(nonatomic, copy, readonly) NSArray<Core3DFaceBoundaryUse *> *boundaryUses;
- (instancetype)init NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DFaceSelectorQuery : NSObject
@property(nonatomic, readonly) Core3DFaceSelectorStatus status;
@property(nonatomic, readonly) Core3DFaceSelectorRefusal refusal;
@property(nonatomic, copy, readonly) NSString *refusalCode, *refusalMessage;
@property(nonatomic, strong, readonly, nullable) Core3DFaceSelectorProof *proof;
@property(nonatomic, strong, readonly) Core3DFaceSelectorIntent *intent;
@property(nonatomic, readonly) int64_t measuredUndoDelta, measuredRedoDelta;
- (instancetype)init NS_UNAVAILABLE;
@end

typedef NS_ENUM(NSInteger, Core3DFaceSelectorFixture) {
    Core3DFaceSelectorFixtureBox, Core3DFaceSelectorFixtureBoxWithHole,
    Core3DFaceSelectorFixtureDisconnectedCoplanar, Core3DFaceSelectorFixtureEqualExtrema,
    Core3DFaceSelectorFixtureSeamUse, Core3DFaceSelectorFixturePartialCircle,
    Core3DFaceSelectorFixtureSplitLine, Core3DFaceSelectorFixtureTruncated65,
    Core3DFaceSelectorFixtureFullCircle, Core3DFaceSelectorFixtureOwnerMismatch,
};
__attribute__((objc_subclassing_restricted))
@interface Core3DFaceSelectorProbeResult : NSObject
@property(nonatomic, strong, readonly) Core3DFaceSelectorQuery *query;
@property(nonatomic, readonly) BOOL sourceBytesEqual, definitionBytesEqual;
@property(nonatomic, readonly) BOOL geometryBytesEqual, displayBytesEqual, historyEqual;
- (instancetype)init NS_UNAVAILABLE;
@end

NS_ASSUME_NONNULL_END
