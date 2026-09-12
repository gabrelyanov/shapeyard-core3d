#import <Foundation/Foundation.h>
#import "Core3DSceneSnapshot.h"
NS_ASSUME_NONNULL_BEGIN

// Diagnostic outcomes are explicit. Only complete reports expose contact pairs.
typedef NS_ENUM(NSInteger, Core3DMeshContactState) {
    Core3DMeshContactStateComplete NS_SWIFT_NAME(complete),
    Core3DMeshContactStateUnsupported NS_SWIFT_NAME(unsupported),
    Core3DMeshContactStateInvalidGeometry NS_SWIFT_NAME(invalidGeometry),
    Core3DMeshContactStateResourceLimit NS_SWIFT_NAME(resourceLimit),
    Core3DMeshContactStateCancelled NS_SWIFT_NAME(cancelled),
    Core3DMeshContactStateTimedOut NS_SWIFT_NAME(timedOut),
    Core3DMeshContactStateStaleSource NS_SWIFT_NAME(staleSource),
    Core3DMeshContactStateInternalFailure NS_SWIFT_NAME(internalFailure),
};

NS_SWIFT_SENDABLE
@interface Core3DMeshContactPair : NSObject
// Zero-based native stored-triangle ordinals in this report's exact capture.
// These ordinals grant no authority over another mesh or revision.
@property(nonatomic, readonly) NSUInteger firstTriangle;
@property(nonatomic, readonly) NSUInteger secondTriangle;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

NS_SWIFT_SENDABLE
@interface Core3DMeshContactReport : NSObject
@property(nonatomic, readonly) Core3DMeshContactState state;
@property(nonatomic, readonly, copy) NSString *entityIdentifier;
@property(nonatomic, readonly, copy) NSString *definitionIdentifier;
@property(nonatomic, readonly, copy) NSString *publicationSourceIdentifier;
@property(nonatomic, readonly) uint64_t documentGeneration;
@property(nonatomic, readonly) uint64_t modelRevision;
@property(nonatomic, readonly) uint64_t geometryRevision;
@property(nonatomic, readonly) NSUInteger triangleCount;
@property(nonatomic, readonly) NSUInteger candidatePairCount;
@property(nonatomic, readonly, copy) NSArray<Core3DMeshContactPair *> *unexpectedPairs;
@property(nonatomic, readonly) NSTimeInterval elapsedSeconds;
// This immutable report diagnoses within one stored mesh, not other objects,
// manifoldness, physical wall suitability, collision or general printability.
// Non-complete results expose zero counts and an empty pair list.
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

// Immutable detached display geometry from one owner-validated contact pair.
// It grants no model editing/selection authority. Camera publications are main
// thread only and carry no geometry. An existing inspection is a historical
// view; callers must revalidate its report before presenting after model edits.
@interface Core3DMeshContactInspection : NSObject
@property(nonatomic, readonly, strong) Core3DSceneSnapshot *scene;
@property(nonatomic, readonly, strong) Core3DScenePresentationOverlaySnapshot *overlay;
@property(nonatomic, readonly) NSUInteger firstTriangle;
@property(nonatomic, readonly) NSUInteger secondTriangle;
// Radians; finite yaw +/-1e6, pitch +/-1.5, zoom .25...16. Zoom 1 fits the
// requested bounds with margin. Pixel dimensions must each be 1...16384.
- (nullable Core3DSceneFrameSnapshot *)frameWithYaw:(double)yaw
    pitch:(double)pitch zoom:(double)zoom focusPair:(BOOL)focusPair
    viewportSize:(simd_uint2)viewportSize
    NS_SWIFT_NAME(frame(yaw:pitch:zoom:focusPair:viewportSize:));
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

typedef void (NS_SWIFT_SENDABLE ^Core3DMeshContactCompletion)(Core3DMeshContactReport *report);

// Constructed and started only through the native document owner. There is no
// public numeric-input initializer. Completion is delivered once on main.
NS_SWIFT_SENDABLE
@interface Core3DMeshContactOperation : NSObject
@property(atomic, readonly, getter=isCancelled) BOOL cancelled;
- (void)cancel;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end
NS_ASSUME_NONNULL_END
