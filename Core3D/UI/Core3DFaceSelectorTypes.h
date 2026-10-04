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

#if DEBUG
// Closed, native-only topology-budget observer vocabulary. These values are
// diagnostics, never selector/mutation authority and never persisted.
typedef NS_ENUM(NSInteger, Core3DB2TopologyBudgetScenario) {
    Core3DB2TopologyBudgetScenarioBox = 1,
    Core3DB2TopologyBudgetScenarioBoxWithHole,
    Core3DB2TopologyBudgetScenarioEdge4095,
    Core3DB2TopologyBudgetScenarioEdge4096,
    Core3DB2TopologyBudgetScenarioEdge4097,
    Core3DB2TopologyBudgetScenarioFace4095,
    Core3DB2TopologyBudgetScenarioFace4096,
    Core3DB2TopologyBudgetScenarioFace4097,
    Core3DB2TopologyBudgetScenarioMixed4097,
    Core3DB2TopologyBudgetScenarioRepeatedOccurrences,
    Core3DB2TopologyBudgetScenarioDeepContainers,
    Core3DB2TopologyBudgetScenarioVertexFanout,
    Core3DB2TopologyBudgetScenarioEmptyWireFanout,
    Core3DB2TopologyBudgetScenarioTruncated65,
    Core3DB2TopologyBudgetScenarioRepeatedAncestry,
    Core3DB2TopologyBudgetScenarioProfileOwner,
    Core3DB2TopologyBudgetScenarioEnclosureOwner,
    Core3DB2TopologyBudgetScenarioLoftOwner,
    Core3DB2TopologyBudgetScenarioLegacyOwner,
    Core3DB2TopologyBudgetScenarioR2Owner,
    Core3DB2TopologyBudgetScenarioEdgeHeavy4097,
    Core3DB2TopologyBudgetScenarioDurableIdentityIssuance,
};
typedef NS_ENUM(NSInteger, Core3DB2TopologyBudgetInjectionKind) {
    Core3DB2TopologyBudgetInjectionNone = 0,
    Core3DB2TopologyBudgetInjectionLeaveExactly,
    Core3DB2TopologyBudgetInjectionLeaveOneLess,
    Core3DB2TopologyBudgetInjectionCorrupt,
};
typedef NS_ENUM(NSInteger, Core3DB2TopologyBudgetDimension) {
    Core3DB2TopologyBudgetDimensionNone = -1,
    Core3DB2TopologyBudgetDimensionVisit = 0,
    Core3DB2TopologyBudgetDimensionBuildStage = 1,
    Core3DB2TopologyBudgetDimensionStageCensus = 2,
    Core3DB2TopologyBudgetDimensionDiscoveryUse = 3,
};

__attribute__((objc_subclassing_restricted))
@interface Core3DB2TopologyBudgetInjection : NSObject
@property(nonatomic, readonly) Core3DB2TopologyBudgetInjectionKind kind;
@property(nonatomic, readonly) NSUInteger remainingVisits, remainingStages;
@property(nonatomic, readonly) NSInteger site, occurrence;
- (instancetype)initWithKind:(Core3DB2TopologyBudgetInjectionKind)kind
    remainingVisits:(NSUInteger)remainingVisits remainingStages:(NSUInteger)remainingStages
    site:(NSInteger)site occurrence:(NSInteger)occurrence NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DB2TopologyBudgetObservation : NSObject
@property(nonatomic, readonly) Core3DB2TopologyBudgetScenario scenario;
@property(nonatomic, readonly) uint64_t metersPerLocalUnitBits, operationSequence;
@property(nonatomic, readonly) NSUInteger entryVisits, entryStages, exitVisits, exitStages;
@property(nonatomic, readonly) NSUInteger faceCount, edgeCount, occurrenceCount;
@property(nonatomic, readonly) NSInteger deniedSite;
@property(nonatomic, readonly) Core3DB2TopologyBudgetDimension deniedDimension;
@property(nonatomic, readonly) NSUInteger deniedRequested, deniedVisits, deniedStages;
@property(nonatomic, readonly) BOOL exhausted, protectedWorkStarted, partialOutputEscaped;
@property(nonatomic, readonly) BOOL supported, traceComplete;
@property(nonatomic, copy, readonly) NSString *refusalCode;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *visitsBySite, *stagesBySite;
// Root-container readback for the diagnostic input shape: TopAbs type of the
// root, its direct child count, and the first child's TopAbs type (-1 when
// the shape is null or has no children). Lets a test assert a surrounding
// diagnostic compound together with its occurrence count.
@property(nonatomic, readonly) NSInteger rootShapeType, rootChildShapeType;
@property(nonatomic, readonly) NSInteger rootChildCount;
// Owner-operation observation (begin/end bridge): bounded phase ring of the
// real operation's per-phase entry/exit counters, plus completion count.
@property(nonatomic, readonly) NSUInteger completionCount;
@property(nonatomic, copy, readonly) NSArray<NSString *> *phaseNames;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *phaseEntryVisits,
    *phaseExitVisits, *phaseEntryStages, *phaseExitStages;
// Read-only native issuance state for the currently selected retained owner.
// This is observation only: it never mints an identity or grants authority.
@property(nonatomic, readonly) uint64_t identityNextLocalID;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *identityRetiredLocalIDs;
- (instancetype)init NS_UNAVAILABLE;
@end
#endif

NS_ASSUME_NONNULL_END
