//
//  Core3DTransformInspectorSnapshot.h
//  Core3D
//
//  Immutable, document-authoritative transform inspector values.
//

#import <Foundation/Foundation.h>
#import <simd/simd.h>
#import <Core3D/Core3DModelCapability.h>

NS_ASSUME_NONNULL_BEGIN

#if __has_attribute(objc_subclassing_restricted)
#define CORE3D_TRANSFORM_INSPECTOR_FINAL_CLASS \
    __attribute__((objc_subclassing_restricted))
#else
#define CORE3D_TRANSFORM_INSPECTOR_FINAL_CLASS
#endif

//! Lifecycle and admission state for one transform-inspector request.
typedef NS_ENUM(NSInteger, Core3DTransformInspectorState) {
    //! The native viewer is unavailable or the request was made off-main.
    Core3DTransformInspectorStateUnavailable = 0,
    //! The view controller has not completed native viewer setup.
    Core3DTransformInspectorStateNotSetup,
    Core3DTransformInspectorStateNoSelection,
    Core3DTransformInspectorStateMultipleSelection,
    Core3DTransformInspectorStateUnsupportedSelection,
    //! A model operation or document command makes inspection unsafe.
    Core3DTransformInspectorStateBusy,
    //! Transform values are available while exact bounds are measured.
    Core3DTransformInspectorStateMeasuring,
    //! Transform and exact bounds values are available.
    Core3DTransformInspectorStateReady,
    //! The transform is valid, but exact bounds are unavailable under the
    //! bounded measurement policy.
    Core3DTransformInspectorStateMeasurementUnavailable,
    //! Measurement could not complete safely.
    Core3DTransformInspectorStateMeasurementFailed,
    //! Native values violated the public snapshot contract.
    Core3DTransformInspectorStateInvalid,
};

typedef NS_ENUM(NSInteger, Core3DTransformInspectorRepresentation) {
    Core3DTransformInspectorRepresentationUnknown = 0,
    Core3DTransformInspectorRepresentationBRep,
    Core3DTransformInspectorRepresentationTriangleMesh,
};

//! A document-authoritative snapshot for one inspector refresh. It never reads
//! renderer meshes or transient presentation transforms.
//!
//! Position, local bounds, and dimensions use document model units;
//! `metersPerUnit` converts one model unit to meters. Position is the selected
//! free definition's persisted object-origin translation in document space.
//! Quaternion components are ordered XYZW. Euler values are the canonical
//! degree representation that reconstructs rotation as Rz(z) * Ry(y) * Rx(x).
//! Local bounds are rotation-independent definition-space bounds. Dimensions
//! are their axis extents multiplied by the absolute persisted uniform scale.
//!
//! Selection identity and transform properties are meaningful for Measuring,
//! Ready, MeasurementUnavailable, and MeasurementFailed single-selection
//! states. Bounds and dimensions are meaningful only when `hasDimensions` is
//! true. All other states deliberately carry no model identity or geometry.
CORE3D_TRANSFORM_INSPECTOR_FINAL_CLASS NS_SWIFT_SENDABLE
@interface Core3DTransformInspectorSnapshot : NSObject

@property (nonatomic, assign, readonly) Core3DTransformInspectorState state;
@property (nonatomic, assign, readonly) NSUInteger selectedCount;
//! Monotonic token used to discard stale asynchronous measurement results.
@property (nonatomic, assign, readonly) uint64_t requestGeneration;

@property (nonatomic, copy, readonly, nullable) NSString *entityIdentifier;
@property (nonatomic, copy, readonly, nullable) NSString *definitionIdentifier;
@property (nonatomic, copy, readonly, nullable) NSString *displayName;
@property (nonatomic, assign, readonly)
    Core3DTransformInspectorRepresentation representation;

@property (nonatomic, assign, readonly) simd_double3 position;
@property (nonatomic, assign, readonly) simd_double4 quaternionXYZW;
@property (nonatomic, assign, readonly)
    simd_double3 canonicalEulerXYZDegrees;
@property (nonatomic, assign, readonly) double uniformScale;
@property (nonatomic, assign, readonly) double metersPerUnit;

@property (nonatomic, assign, readonly) simd_double3 localBoundsMinimum;
@property (nonatomic, assign, readonly) simd_double3 localBoundsMaximum;
@property (nonatomic, assign, readonly) simd_double3 dimensions;
@property (nonatomic, assign, readonly) BOOL hasDimensions;

//! True only when the displayed AIS transform still matches the persisted
//! document transform captured by this request.
@property (nonatomic, assign, readonly) BOOL presentationMatchesDocument;
@property (nonatomic, assign, readonly)
    Core3DModelCapability modelCapabilities;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end

//! Called at most once, on the main thread. A request retains this block only
//! when its immediate snapshot is Measuring; a terminal immediate state does
//! not invoke it.
typedef void (NS_SWIFT_SENDABLE ^Core3DTransformInspectorCompletion)(
    Core3DTransformInspectorSnapshot *snapshot
);

NS_ASSUME_NONNULL_END
