#pragma once

// Shared native modeling values and command contracts. Keep UIKit/AppKit
// presentation types out of this interface; platform hosts adapt these values.
#import <Foundation/Foundation.h>
#import <CoreGraphics/CoreGraphics.h>
#import <simd/simd.h>
#import <Core3D/PrimitiveType.h>
#import <Core3D/PrimitiveSelectionType.h>
#import <Core3D/PrimitiveGizmoType.h>
#import <Core3D/ExportType.h>
#import <Core3D/Core3DNativeExportOperation.h>
#import <Core3D/Core3DMeshContactOperation.h>
#import <Core3D/OrthoProjectionType.h>
#import <Core3D/Core3DModelCapability.h>
#import <Core3D/Core3DTransformInspectorSnapshot.h>
#import <Core3D/Core3DSharedModelingValues.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DSceneSnapshot;
@class Core3DStoredProfileSnapshot;
@class Core3DSceneFrameSnapshot;
@class Core3DScenePresentationOverlaySnapshot;

typedef NS_ENUM(NSInteger, Core3DProfileConstructionResult) {
    Core3DProfileConstructionResultCommitted = 0,
    Core3DProfileConstructionResultRejected,
    Core3DProfileConstructionResultBusy,
    Core3DProfileConstructionResultRecoveryRequired,
    Core3DProfileConstructionResultFailed,
    Core3DProfileConstructionResultCancelled,
    Core3DProfileConstructionResultUnchanged,
};


//! Native-issued, main-owned selected sweep snapshot. Wire fields cannot recreate it.
__attribute__((objc_subclassing_restricted))
@interface Core3DStoredSweepSnapshot : NSObject
@property(nonatomic,copy,readonly) NSString *entityIdentifier;
@property(nonatomic,copy,readonly) NSString *definitionIdentifier;
@property(nonatomic,copy,readonly) NSString *featureIdentifier;
@property(nonatomic,strong,readonly) Core3DSweepDefinition *definition;
@property(nonatomic,readonly) double effectiveDimensionMetersPerUnit;
@property(nonatomic,readonly) BOOL current;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Values use source-recipe millimetres, BEFORE construction-frame and occurrence
//! scaling. They do not change the bore, source plane, construction frame or IDs.
typedef NS_ENUM(NSInteger, Core3DSavedCutSourceFamily) { Core3DSavedCutSourceFamilyPolygon NS_SWIFT_NAME(polygon)=1, Core3DSavedCutSourceFamilyEnclosure NS_SWIFT_NAME(enclosure)=2 };
__attribute__((objc_subclassing_restricted))
@interface Core3DSavedCutSourcePatch : NSObject
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (nullable instancetype)initWithPolygonCoordinates:(NSArray<Core3DSavedCutSourceCoordinate *> *)coordinates depthMM:(nullable NSNumber *)depth
    NS_SWIFT_NAME(init(polygonCoordinates:depthMM:));
- (nullable instancetype)initWithEnclosureWidthMM:(nullable NSNumber *)width depthMM:(nullable NSNumber *)depth
    heightMM:(nullable NSNumber *)height wallMM:(nullable NSNumber *)wall floorMM:(nullable NSNumber *)floor
    cornerRadiusMM:(nullable NSNumber *)corner
    NS_SWIFT_NAME(init(enclosureWidthMM:depthMM:heightMM:wallMM:floorMM:cornerRadiusMM:));
@end
//! Native-issued descriptive values copied from the SAME cut snapshot. No new
//! capture occurs and these fields cannot recreate source/transaction authority.
__attribute__((objc_subclassing_restricted))
@interface Core3DSavedCutSourceValues : NSObject
@property(nonatomic,readonly) Core3DSavedCutSourceFamily family;
@property(nonatomic,readonly) Core3DProfilePlane plane;
@property(nonatomic,readonly) double metersPerUnit;
@property(nonatomic,copy,readonly) NSArray<NSValue *> *polygonPointsMM; // CGPoint U/V in original order; empty for enclosure.
@property(nonatomic,copy,readonly,nullable) NSNumber *widthMM;
@property(nonatomic,copy,readonly,nullable) NSNumber *depthMM;
@property(nonatomic,copy,readonly,nullable) NSNumber *heightMM;
@property(nonatomic,copy,readonly,nullable) NSNumber *wallMM;
@property(nonatomic,copy,readonly,nullable) NSNumber *floorMM;
@property(nonatomic,copy,readonly,nullable) NSNumber *cornerRadiusMM;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end
__attribute__((objc_subclassing_restricted))
@interface Core3DSavedCutSourceOperation : NSObject
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
//! Main-only, exact still-pending job. Settled, old and repeated Stop return NO.
//! Cancellation releases native authority now; callback waits for actual worker drain.
- (BOOL)cancel;
@end

//! Native-issued source identity is not reconstructible from serialized fields.
__attribute__((objc_subclassing_restricted))
@interface Core3DCylindricalCutSnapshot : NSObject
@property(nonatomic,copy,readonly) NSString *entityIdentifier;
@property(nonatomic,copy,readonly) NSString *definitionIdentifier;
@property(nonatomic,readonly) BOOL rebuilding;
@property(nonatomic,readonly) double worldRadiusMM; // zero until first creation
@property(nonatomic,strong,readonly,nullable) Core3DSavedCutSourceValues *sourceRecipeMM;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end
__attribute__((objc_subclassing_restricted))
@interface Core3DCylindricalCutOperation : NSObject
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (BOOL)cancel;
@end

//! Descriptive values for one stable bore of a retained cut program. Native
//! issue only; clients cannot manufacture authority from these fields.
__attribute__((objc_subclassing_restricted))
@interface Core3DCylindricalCutBore : NSObject
@property(nonatomic,readonly) uint32_t operandIdentifier;
@property(nonatomic,readonly) Core3DCylindricalCutAxis axis;
@property(nonatomic,readonly) double localX;
@property(nonatomic,readonly) double localY;
@property(nonatomic,readonly) double localZ;
//! Descriptive construction-frame position in physical millimetres from the
//! same complete capture. It includes positive occurrence uniform scale and
//! excludes occurrence rotation/translation. These values cannot recreate
//! edit authority; consumers must validate their own descriptive bounds.
@property(nonatomic,readonly) double physicalXMM;
@property(nonatomic,readonly) double physicalYMM;
@property(nonatomic,readonly) double physicalZMM;
@property(nonatomic,readonly) double worldRadiusMM;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end
//! Native-issued whole-program snapshot of a retained cut: the complete recipe
//! and every stable operand ride one opaque aggregate. No one-bore local view
//! is exposed as native authority, and wire fields cannot recreate it.
__attribute__((objc_subclassing_restricted))
@interface Core3DCylindricalCutProgramSnapshot : NSObject
@property(nonatomic,copy,readonly) NSString *entityIdentifier;
@property(nonatomic,copy,readonly) NSString *definitionIdentifier;
@property(nonatomic,copy,readonly) NSArray<Core3DCylindricalCutBore *> *bores;
//! Descriptive source values shared by every bore of the complete recipe, in
//! source-recipe millimetres BEFORE construction-frame and occurrence scaling.
//! Copied from THIS snapshot; they cannot recreate source/transaction authority.
@property(nonatomic,strong,readonly,nullable) Core3DSavedCutSourceValues *sourceRecipeMM;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Native-issued, main-owned selected rectangular-loft snapshot. Wire fields cannot recreate it.
__attribute__((objc_subclassing_restricted))
@interface Core3DStoredRectangularLoftSnapshot : NSObject
@property(nonatomic,copy,readonly) NSString *entityIdentifier;
@property(nonatomic,copy,readonly) NSString *definitionIdentifier;
@property(nonatomic,copy,readonly) NSString *featureIdentifier;
@property(nonatomic,strong,readonly) Core3DRectangularLoftDefinition *definition;
@property(nonatomic,readonly) double effectiveDimensionMetersPerUnit;
@property(nonatomic,readonly) BOOL current;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Cancellation only for the exact native work that created this handle.
//! Cannot execute, replace, retry or cancel a later operation.
__attribute__((objc_subclassing_restricted))
@interface Core3DStoredLoftEditOperation : NSObject
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (BOOL)cancel;
@end


//! Exact opening authority for one selected enclosure; a changed document/model
//! requires another read before applying dimensions. Values are not authority.
__attribute__((objc_subclassing_restricted))
@interface Core3DStoredEnclosureSnapshot : NSObject
@property(nonatomic,copy,readonly) NSString *entityIdentifier;
@property(nonatomic,copy,readonly) NSString *definitionIdentifier;
@property(nonatomic,copy,readonly) NSString *featureIdentifier;
@property(nonatomic,strong,readonly) Core3DEnclosureDefinition *definition;
//! Physical metres per definition length along its own axes, including authored
//! and construction scale. Meaningful for editing only while current is YES.
//! Pass this same opening snapshot to rebuild; changed authority is rejected.
@property(nonatomic,readonly) double dimensionMetersPerUnit;
@property(nonatomic,readonly) BOOL current;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

typedef NS_ENUM(NSInteger, Core3DAssemblyPartKind) {
    Core3DAssemblyPartKindBox = 0, Core3DAssemblyPartKindCylinder,
};
//! Physical millimetres. Box origin=min corner; cylinder origin=base centre;
//! height follows local +Z before the explicit proper quaternion/translation.
//! Part UUID is persisted as profile feature identity, never edit permission.
__attribute__((objc_subclassing_restricted))
@interface Core3DAssemblyPartDefinition : NSObject
@property(nonatomic,readonly) Core3DAssemblyPartKind kind;
@property(nonatomic,copy,readonly) NSString *name;
@property(nonatomic,copy,readonly) NSString *partIdentifier;
@property(nonatomic,readonly) double widthMM;
@property(nonatomic,readonly) double depthMM;
@property(nonatomic,readonly) double heightMM;
@property(nonatomic,readonly) double radiusMM;
@property(nonatomic,readonly) simd_double3 positionMM;
@property(nonatomic,readonly) simd_double4 rotationXYZW;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
+ (nullable instancetype)boxWithName:(NSString *)name widthMM:(double)width depthMM:(double)depth
    heightMM:(double)height positionMM:(simd_double3)position rotationXYZW:(simd_double4)rotation
    NS_SWIFT_NAME(box(name:widthMM:depthMM:heightMM:positionMM:rotationXYZW:));
+ (nullable instancetype)cylinderWithName:(NSString *)name radiusMM:(double)radius heightMM:(double)height
    positionMM:(simd_double3)position rotationXYZW:(simd_double4)rotation
    NS_SWIFT_NAME(cylinder(name:radiusMM:heightMM:positionMM:rotationXYZW:));
@end

//! Single component: Position is physical mm; Rotation is absolute extrinsic
//! XYZ degrees. This ordinary preparation is not an AI request or receipt.
typedef NS_ENUM(NSInteger, Core3DRigidPlacementKind) {
    Core3DRigidPlacementKindPosition=0, Core3DRigidPlacementKindRotation=1,
};
__attribute__((objc_subclassing_restricted))
@interface Core3DRigidPlacementPreparation : NSObject
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! One-use planning lease issued by one live native owner. Its immutable
//! descriptions may inform a provider; none of their serialized fields can
//! recreate permission. Keep this object on the main thread. Capture requires
//! passive object selection and no preview, load, construction or recovery.
__attribute__((objc_subclassing_restricted))
@interface Core3DModelingPlanningContext : NSObject
@property(nonatomic,strong,readonly) Core3DSceneSnapshot *scene;
@property(nonatomic,copy,readonly) NSString *documentIdentifier;
@property(nonatomic,strong,readonly,nullable) Core3DStoredEnclosureSnapshot *selectedEnclosure;
//! Current canonical rectangle or solid-circle extrusion dimensions. Bounded
//! arbitrary recipes are exposed separately below; manual editing is unchanged.
@property(nonatomic,strong,readonly,nullable) Core3DStoredProfileSnapshot *selectedProfile;
//! Exclusive alternative to selectedProfile for bounded polygon extrusions
//! and ordered line/arc extrude/revolve recipes. Only the existing operation's
//! depth/angle may be changed through rebuildProfileRecipe; no contour edits.
@property(nonatomic,strong,readonly,nullable) Core3DStoredProfileSnapshot *selectedProfileRecipe;
//! Exact selected saved sweep. Mutually exclusive with the three profile/enclosure fields.
//! Descriptive physical values use effectiveDimensionMetersPerUnit; authority stays native.
@property(nonatomic,strong,readonly,nullable) Core3DStoredSweepSnapshot *selectedSweep;
//! Exact saved ruled rectangular loft, exclusive of every other selected feature.
//! Descriptive prerequisite only: no AI command, reserved execution or receipt is enabled.
@property(nonatomic,strong,readonly,nullable) Core3DStoredRectangularLoftSnapshot *selectedLoft;
//! Mutually exclusive original opaque source targets from one stamped capture.
//! A complete program is never projected onto its first bore.
@property(nonatomic,strong,readonly,nullable) Core3DCylindricalCutSnapshot *selectedSavedCutSource;
@property(nonatomic,strong,readonly,nullable) Core3DCylindricalCutProgramSnapshot *selectedSavedCutProgramSource;
//! Descriptive values shared by the selected legacy or program source target.
@property(nonatomic,strong,readonly,nullable) Core3DSavedCutSourceValues *selectedSavedCutSourceRecipeMM;
//! Original composite placement capture only; recipe contexts return nil.
@property(nonatomic,strong,readonly,nullable) Core3DTransformInspectorSnapshot *placementSnapshot;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

// Insert after Core3DModelingPlanningContext's @end, before stored profile.
//! Main-thread host session binding. Core does not authenticate these values.
//! The trusted host session coordinator supplies a nonsecret namespace and
//! generation. Never pass credentials, provider fields or reconstructed tokens.
__attribute__((objc_subclassing_restricted))
@interface Core3DModelingHostSession : NSObject
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
+ (nullable instancetype)installHostAccountSession:(NSString *)scope generation:(NSString *)generation
    NS_SWIFT_NAME(installHostAccountSession(scope:generation:));
+ (void)retireHostAccountSession:(Core3DModelingHostSession *)session
    NS_SWIFT_NAME(retireHostAccountSession(_:));
@end

typedef NS_ENUM(NSInteger, Core3DModelingEvidenceCoverage) {
    Core3DModelingEvidenceCoverageCanonicalEffect = 0,
    Core3DModelingEvidenceCoverageUnverifiedProfile,
    Core3DModelingEvidenceCoverageExactLoftEffect,
    Core3DModelingEvidenceCoverageExactPlacementEffect,
};
typedef NS_ENUM(NSInteger, Core3DModelingRequestExecutionResult) {
    Core3DModelingRequestExecutionResultUnavailable = 0,
};
//! Immutable main-owned preparation only. Hashes describe the native typed
//! command; none recreates authority. Stage A cannot execute or reserve it.
__attribute__((objc_subclassing_restricted))
@interface Core3DModelingPreparedRequest : NSObject
@property(nonatomic,copy,readonly) NSUUID *requestIdentifier;
@property(nonatomic,copy,readonly) NSString *documentIdentifier;
@property(nonatomic,copy,readonly) NSData *commandSHA256;
@property(nonatomic,copy,readonly) NSData *executionSHA256;
@property(nonatomic,readonly) Core3DModelingEvidenceCoverage evidenceCoverage;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end


typedef NS_ENUM(NSInteger, Core3DModelingAsyncDisposition) {
    Core3DModelingAsyncDispositionCommitted, Core3DModelingAsyncDispositionCancelled,
    Core3DModelingAsyncDispositionRejected, Core3DModelingAsyncDispositionFailed,
    Core3DModelingAsyncDispositionPreviouslySeen, Core3DModelingAsyncDispositionConflict,
    Core3DModelingAsyncDispositionCapacity, Core3DModelingAsyncDispositionBusy,
    Core3DModelingAsyncDispositionUncertain, Core3DModelingAsyncDispositionUnsupported,
    //! Source-checked no-change invocation; no OCAF command or receipt was added.
    Core3DModelingAsyncDispositionUnchanged
};
typedef NS_ENUM(NSInteger, Core3DModelingStorageObservation) {
    Core3DModelingStorageObservationNotAttempted, Core3DModelingStorageObservationReserved,
    Core3DModelingStorageObservationPreviouslySeen, Core3DModelingStorageObservationConflict,
    Core3DModelingStorageObservationCapacity, Core3DModelingStorageObservationBusy,
    Core3DModelingStorageObservationUnknown
};
//! Result of one invocation, not saved-document durability or query authority.
//! No disposition authorizes automatic replay. Invalid/foreign calls omit IDs.
__attribute__((objc_subclassing_restricted))
@interface Core3DModelingAsyncOutcome : NSObject
@property(nonatomic,readonly) Core3DModelingAsyncDisposition disposition;
@property(nonatomic,readonly) Core3DModelingStorageObservation storageObservation;
@property(nonatomic,copy,readonly,nullable) NSUUID *requestIdentifier;
@property(nonatomic,copy,readonly,nullable) NSString *documentIdentifier;
@property(nonatomic,copy,readonly) NSArray<NSString *> *entityIdentifiers;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! A read of one selected native profile. A stale profile remains inspectable
//! but cannot replace subsequent geometry edits. Apply also checks the original
//! document/model revision; only the presentation may be refreshed.
@interface Core3DStoredProfileSnapshot : NSObject
@property(nonatomic,copy,readonly) NSString *entityIdentifier;
@property(nonatomic,copy,readonly) NSString *definitionIdentifier;
@property(nonatomic,copy,readonly) NSString *featureIdentifier;
@property(nonatomic,strong,readonly) Core3DProfileDefinition *definition;
//! Physical metres per definition length along its own axes, including both
//! stored construction and authored object scale. Meaningful for editing only
//! while current is YES and this value is positive. Zero means unavailable
//! for physical editing; manual profile inspection remains supported.
@property(nonatomic,readonly) double dimensionMetersPerUnit;
@property(nonatomic,readonly) BOOL current;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

typedef NS_ENUM(NSInteger, Core3DObjectAlignmentAnchor) {
    Core3DObjectAlignmentAnchorMinimum = 0,
    Core3DObjectAlignmentAnchorCenter,
    Core3DObjectAlignmentAnchorMaximum,
    Core3DObjectAlignmentAnchorGround,
    //! Distribute 3–32 objects in center order, with stable entity-ID ties.
    //! Centers keeps endpoint centers fixed; Gaps preserves outer bounds and
    //! rejects insufficient space. Saved-group members remain separate parts.
    Core3DObjectAlignmentAnchorEqualCenters,
    Core3DObjectAlignmentAnchorEqualGaps,
    //! Translate the selection as one assembly: aggregate X/Y center to the
    //! world origin and aggregate minimum Z to the ground plane.
    Core3DObjectAlignmentAnchorCenterGround,
    //! Internal saved-group base-center measurement; normal Align UI never offers it.
    Core3DObjectAlignmentAnchorGroupBaseOrigin,
};
typedef NS_ENUM(NSInteger, Core3DObjectAlignmentResult) {
    Core3DObjectAlignmentResultUnchanged = 0,
    Core3DObjectAlignmentResultCommitted,
    Core3DObjectAlignmentResultRejected,
    Core3DObjectAlignmentResultBusy,
    Core3DObjectAlignmentResultRecoveryRequired,
    Core3DObjectAlignmentResultFailed,
    Core3DObjectAlignmentResultCancelled,
};

typedef NS_ENUM(NSInteger, Core3DSavedGroupEditResult) {
    Core3DSavedGroupEditResultUnchanged = 0,
    Core3DSavedGroupEditResultCommitted,
    Core3DSavedGroupEditResultRejected,
    Core3DSavedGroupEditResultBusy,
    Core3DSavedGroupEditResultRecoveryRequired,
    Core3DSavedGroupEditResultFailed,
    Core3DSavedGroupEditResultBlockedByLayer,
};

typedef NS_ENUM(NSInteger, Core3DMeshCopyResult) {
    Core3DMeshCopyResultUnchanged,
    Core3DMeshCopyResultCommitted,
    Core3DMeshCopyResultRejected,
    Core3DMeshCopyResultBusy,
    Core3DMeshCopyResultRecoveryRequired,
    Core3DMeshCopyResultFailed,
};

typedef NS_ENUM(NSInteger, Core3DMeshVertexEditResult) {
    Core3DMeshVertexEditResultUnchanged,
    Core3DMeshVertexEditResultCommitted,
    Core3DMeshVertexEditResultRejected,
    Core3DMeshVertexEditResultBusy,
    Core3DMeshVertexEditResultRecoveryRequired,
    Core3DMeshVertexEditResultFailed,
};

typedef NS_ENUM(NSInteger, Core3DMeshElementKind) {
    Core3DMeshElementKindVertex = 0,
    Core3DMeshElementKindEdge,
    Core3DMeshElementKindTriangle,
};

//! Immutable session-local vertex IDs are array indices; positions are world mm.
//! The native viewer retains exact edit authority and all OCCT handles.
typedef NS_ENUM(NSInteger, Core3DMeshRegionExtrudeResult) {
    Core3DMeshRegionExtrudeResultUnchanged,
    Core3DMeshRegionExtrudeResultCommitted,
    Core3DMeshRegionExtrudeResultRejected,
    Core3DMeshRegionExtrudeResultBusy,
    Core3DMeshRegionExtrudeResultRecoveryRequired,
    Core3DMeshRegionExtrudeResultFailed,
};


typedef NS_ENUM(NSInteger, Core3DMeshRegionInsetResult) {
    Core3DMeshRegionInsetResultUnchanged,
    Core3DMeshRegionInsetResultCommitted,
    Core3DMeshRegionInsetResultRejected,
    Core3DMeshRegionInsetResultBusy,
    Core3DMeshRegionInsetResultRecoveryRequired,
    Core3DMeshRegionInsetResultFailed,
};

//! Native-resolved planar region; indices are valid only for this one-use session.
@interface Core3DMeshRegionExtrudeSnapshot : NSObject
@property(nonatomic,copy,readonly) NSString *entityIdentifier;
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *triangleIndices;
@property(nonatomic,copy,readonly) NSArray<NSArray<NSNumber *> *> *worldBoundary;
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *worldUnitNormal;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DMeshRegionInsetSnapshot : NSObject
@property(nonatomic,copy,readonly) NSString *entityIdentifier;
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *triangleIndices;
@property(nonatomic,copy,readonly) NSArray<NSArray<NSNumber *> *> *worldBoundary;
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *worldUnitNormal;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DMeshVertexEditSnapshot : NSObject
@property(nonatomic,copy,readonly) NSString *entityIdentifier;
@property(nonatomic,copy,readonly) NSArray<NSArray<NSNumber *> *> *worldVertices;
@property(nonatomic,readonly) Core3DMeshElementKind elementKind;
//! Session-local native vertex indices. Only the requested domain is populated.
@property(nonatomic,copy,readonly) NSArray<NSArray<NSNumber *> *> *edgeVertexIndices;
@property(nonatomic,copy,readonly) NSArray<NSArray<NSNumber *> *> *triangleVertexIndices;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

typedef NS_ENUM(NSInteger, Core3DMeshWindingRepairResult) {
    Core3DMeshWindingRepairResultUnchanged,
    Core3DMeshWindingRepairResultCommitted,
    Core3DMeshWindingRepairResultRejected,
    Core3DMeshWindingRepairResultBusy,
    Core3DMeshWindingRepairResultRecoveryRequired,
    Core3DMeshWindingRepairResultFailed,
};

typedef NS_ENUM(NSInteger, Core3DMeshUVAtlasResult) {
    Core3DMeshUVAtlasResultUnchanged,
    Core3DMeshUVAtlasResultCommitted,
    Core3DMeshUVAtlasResultRejected,
    Core3DMeshUVAtlasResultBusy,
    Core3DMeshUVAtlasResultRecoveryRequired,
    Core3DMeshUVAtlasResultFailed,
};

typedef NS_ENUM(NSInteger, Core3DObjectNameEditResult) {
    Core3DObjectNameEditResultUnchanged = 0,
    Core3DObjectNameEditResultCommitted,
    Core3DObjectNameEditResultRejected,
    Core3DObjectNameEditResultBusy,
    Core3DObjectNameEditResultRecoveryRequired,
    Core3DObjectNameEditResultFailed,
};

typedef NS_ENUM(NSInteger, Core3DObjectVisibilityEditResult) {
    Core3DObjectVisibilityEditResultUnchanged = 0,
    Core3DObjectVisibilityEditResultCommitted,
    Core3DObjectVisibilityEditResultRejected,
    Core3DObjectVisibilityEditResultBusy,
    Core3DObjectVisibilityEditResultRecoveryRequired,
    Core3DObjectVisibilityEditResultFailed,
    Core3DObjectVisibilityEditResultBlockedByLayer,
};

typedef NS_ENUM(NSInteger, Core3DAssetLoadResult) {
    Core3DAssetLoadResultSuccess = 0,
    Core3DAssetLoadResultInvalidData,
    Core3DAssetLoadResultTemporaryFileFailure,
    Core3DAssetLoadResultBusy,
    Core3DAssetLoadResultUnsupportedVersion,
    Core3DAssetLoadResultInternalFailure,
};

typedef NS_ENUM(NSInteger, Core3DViewportRenderingAPI) {
    Core3DViewportRenderingAPIUnavailable = 0,
    Core3DViewportRenderingAPIOpenGLES2 = 2,
    Core3DViewportRenderingAPIOpenGLES3 = 3,
};

#ifdef DEBUG
//! Bounded standalone BinXCAF documents used to exercise the persistent
//! definition-owned geometry representation contract through production load.
typedef NS_ENUM(NSInteger, Core3DDebugGeometryFixtureMode) {
    Core3DDebugGeometryFixtureEmpty = 0,
    Core3DDebugGeometryFixtureLegacyUnmarkedBRep,
    Core3DDebugGeometryFixtureMarkedBRep,
    Core3DDebugGeometryFixtureMarkedTriangleMesh,
    Core3DDebugGeometryFixtureValidBRepAndTriangleMeshRoots,
    Core3DDebugGeometryFixtureUnmarkedTriangleMesh,
    Core3DDebugGeometryFixtureUnmarkedMixedDefinition,
    Core3DDebugGeometryFixtureBRepMarkerOnTriangleMesh,
    Core3DDebugGeometryFixtureTriangleMeshMarkerOnBRep,
    Core3DDebugGeometryFixtureBRepMarkerOnMixedDefinition,
    Core3DDebugGeometryFixtureTriangleMeshMarkerOnMixedDefinition,
    Core3DDebugGeometryFixtureUnknownMarkerOnBRep,
    Core3DDebugGeometryFixtureOrphanMarker,
    //! Valid admitted mesh with one finite node not referenced by its triangle.
    //! Used to lock the inspector's full stored-vertex-table bounds contract.
    Core3DDebugGeometryFixtureMarkedTriangleMeshWithUnreferencedOutlier,
    //! Valid admitted BRep with a persisted definition-owned uniform scale.
    //! The fixture locks scale separation from authoritative local bounds.
    Core3DDebugGeometryFixtureMarkedBRepWithUniformScale,
    //! Marked BRep with a finite, nonzero persisted scale below the inspector's
    //! validity tolerance. Production loading can construct it safely, while
    //! the checked inspector must report Invalid.
    Core3DDebugGeometryFixtureMarkedBRepWithCorruptTransform,
    //! Valid admitted mesh whose authoritative face triangulation owns a
    //! nonidentity TopLoc_Location that must be applied to every stored node.
    Core3DDebugGeometryFixtureMarkedTriangleMeshWithLocatedTriangulation,
    //! Valid marked BRep whose definition root is one analytic Face.
    Core3DDebugGeometryFixtureMarkedBRepFaceRoot,
    //! Valid marked BRep roots made solely from Wire, Edge, and Vertex shapes.
    Core3DDebugGeometryFixtureMarkedBRepSubshapeRoots,
};

//! Standalone BinXCAF fixtures for the all-or-nothing Reference Axis schema.
typedef NS_ENUM(NSInteger, Core3DDebugReferenceAxisFixtureMode) {
    Core3DDebugReferenceAxisFixtureValidMixedSpace = 0,
    Core3DDebugReferenceAxisFixturePartialRecord,
    Core3DDebugReferenceAxisFixtureUnknownMode,
    Core3DDebugReferenceAxisFixtureWrongModeType,
    Core3DDebugReferenceAxisFixtureNonFinitePivot,
    Core3DDebugReferenceAxisFixtureOversizedPivot,
    Core3DDebugReferenceAxisFixtureZeroDirection,
    Core3DDebugReferenceAxisFixtureNonUnitDirection,
    Core3DDebugReferenceAxisFixtureOrphanRecord,
    //! No authored record: the implicit default must still reject a malformed
    //! persisted object transform during production document admission.
    Core3DDebugReferenceAxisFixtureImplicitDefaultCorruptTransform,
    //! No authored record: a cumulative assembly location that pushes the
    //! implicit Object Origin outside the model-coordinate bound.
    Core3DDebugReferenceAxisFixtureImplicitDefaultOversizedOccurrence,
    //! The private Duplicate ownership GUID is occupied by the wrong scalar
    //! type on document Main and must fail production admission.
    Core3DDebugReferenceAxisFixtureDuplicateSentinelWrongType,
    //! Even the correct private Duplicate sentinel type is corrupt when it is
    //! attached anywhere except document Main.
    Core3DDebugReferenceAxisFixtureDuplicateSentinelMisplaced,
    //! The private Radial Array ownership GUID is occupied by the wrong
    //! scalar type on document Main and must fail production admission.
    Core3DDebugReferenceAxisFixtureRadialSentinelWrongType,
    //! Even the correct private Radial Array sentinel type is corrupt when it
    //! is attached anywhere except document Main.
    Core3DDebugReferenceAxisFixtureRadialSentinelMisplaced,
    Core3DDebugReferenceAxisFixtureOrdinarySentinelWrongType,
    Core3DDebugReferenceAxisFixtureOrdinarySentinelMisplaced,
    Core3DDebugReferenceAxisFixtureOrdinarySentinelValid,
};
#endif

typedef struct {
    double min;
    double max;
} Boundaries;

//! World-axis normal for a selection-bounds Mirror plane.
typedef NS_ENUM(NSInteger, Core3DMirrorAxis) {
    Core3DMirrorAxisX = 0,
    Core3DMirrorAxisY,
    Core3DMirrorAxisZ,
};

//! Stable world-axis contract for a destructive Linear Array operation.
typedef NS_ENUM(NSInteger, Core3DLinearArrayAxis) {
    Core3DLinearArrayAxisX = 0,
    Core3DLinearArrayAxisY,
    Core3DLinearArrayAxisZ,
};

//! Native-authoritative parameters and bounds for one active Linear Array.
//! Count is the total number of objects, including the unchanged source.
typedef struct {
    Core3DLinearArrayAxis axis;
    NSInteger count;
    NSInteger minimumCount;
    NSInteger maximumCount;
    CGFloat spacing;
    CGFloat minimumSpacing;
    CGFloat maximumSpacing;
    //! Physical document scale used to present spacing in millimetres.
    double metersPerUnit;
} Core3DLinearArrayParameters;

//! Coordinate authority for one Reference Axis component. Numeric values are
//! deliberately identical to the persistent native schema and must not be
//! renumbered or reused.
typedef NS_ENUM(NSInteger, Core3DReferenceSpace) {
    Core3DReferenceSpaceObject = 0,
    Core3DReferenceSpaceWorld = 1,
};

//! Whether the active Radial Array source owns an authored Reference Axis or
//! is using the explicit, non-mutating Object-Origin + World-Z default.
typedef NS_ENUM(NSInteger, Core3DReferenceAxisReadState) {
    Core3DReferenceAxisReadStateInvalid = -1,
    Core3DReferenceAxisReadStateImplicitDefault = 0,
    Core3DReferenceAxisReadStateAuthored = 1,
};

//! Complete finite oriented line. Pivot and direction have independent
//! coordinate spaces; native Core normalizes every accepted direction.
typedef struct {
    Core3DReferenceSpace pivotSpace;
    double pivotX;
    double pivotY;
    double pivotZ;
    Core3DReferenceSpace directionSpace;
    double directionX;
    double directionY;
    double directionZ;
} Core3DReferenceAxisValue;

//! One immutable compare-and-swap lease for editing the active Radial Array
//! source's persistent Reference Axis. Token zero is unavailable, never a
//! wildcard. Callers must submit this exact token to Set or Reset.
typedef struct {
    Core3DReferenceAxisReadState readState;
    Core3DReferenceAxisValue value;
    uint64_t authorityToken;
} Core3DRadialArrayReferenceAuthority;

//! Native-authoritative parameters and bounds for one active Radial Array.
//! Count includes the unchanged source; signed sweep is expressed in degrees.
typedef struct {
    NSInteger count;
    NSInteger minimumCount;
    NSInteger maximumCount;
    double sweepDegrees;
    double minimumSweepDegrees;
    double maximumSweepDegrees;
    //! Physical document scale used to present custom pivot coordinates.
    double metersPerUnit;
} Core3DRadialArrayParameters;

//! Truthful result for an atomic persistent Reference Axis edit. An unknown
//! outcome retains the same operation and recovery controls; retryable failure
//! means the edit was not accepted and may be attempted again with fresh state.
typedef NS_ENUM(NSInteger, Core3DRadialArrayReferenceEditResult) {
    Core3DRadialArrayReferenceEditResultNoChange = 0,
    Core3DRadialArrayReferenceEditResultApplied,
    Core3DRadialArrayReferenceEditResultOutcomeUnknown,
    Core3DRadialArrayReferenceEditResultRetryableFailure,
};

//! Native-authoritative local BRep thickness for one active Shell preview.
//! metersPerUnit includes the captured object's absolute uniform scale, so
//! Swift's millimetre display describes the resulting world-space thickness.
//! The document's stored length unit is not changed.
typedef struct {
    CGFloat thickness;
    CGFloat defaultThickness;
    CGFloat minimumThickness;
    CGFloat maximumThickness;
    double metersPerUnit;
} Core3DShellParameters;

//! Truthful outcome for an explicit modeling Apply or Cancel request. An
//! unknown outcome remains retained but must be reconciled with another Apply;
//! a retryable failure retains the ordinary Apply/Cancel recovery controls.
typedef NS_ENUM(NSInteger, Core3DModelingOperationResult) {
    Core3DModelingOperationResultSucceeded = 0,
    Core3DModelingOperationResultNoActiveOperation,
    Core3DModelingOperationResultNotReady,
    Core3DModelingOperationResultOutcomeUnknown,
    Core3DModelingOperationResultRetryableFailure,
    Core3DModelingOperationResultFailed,
};

//! Renderer-independent phase of one modeling preview. Committing is exposed
//! as Computing because neither phase is user-applicable.
typedef NS_ENUM(NSInteger, Core3DModelingPreviewState) {
    Core3DModelingPreviewStateUnavailable = 0,
    Core3DModelingPreviewStateSelecting,
    Core3DModelingPreviewStateComputing,
    Core3DModelingPreviewStateReady,
    Core3DModelingPreviewStateOutcomeUnknown,
    Core3DModelingPreviewStateFailed,
};

//! Release-safe, immutable observation of one modeling operation. Generation
//! is controller-owned and suppresses stale asynchronous preview results.
typedef struct {
    PrimitiveGizmoType operation;
    Core3DModelingPreviewState state;
    uint64_t generation;
    BOOL active;
    BOOL canApply;
} Core3DModelingPreviewStatus;

@protocol PrimitiveManagerProtocol<NSObject>

- (void)addPrimitive:(PrimitiveType)primitiveType;
- (void)addPrimitivesFromJSON:(NSString *)json;
- (void)deleteSelected;
- (void)selectAll;
- (void)duplicateSelected;
- (void)setChamfer:(CGFloat)value;
- (Boundaries)getChamferBoundaries;
- (Core3DModelingOperationResult)tryApplyChamfer
    NS_SWIFT_NAME(tryApplyChamfer());
- (Core3DModelingOperationResult)tryCancelChamfer
    NS_SWIFT_NAME(tryCancelChamfer());
- (void)applyChamfer;
- (BOOL)cancelChamfer;
- (void)setExtrusion:(CGFloat)value;
- (Boundaries)getExtrusionBoundaries;
//! Metres per local BRep distance, including the active source's absolute
//! saved uniform scale. Zero means no valid active extrusion. Read-only.
- (double)getExtrusionMetersPerUnit;
- (Core3DModelingOperationResult)tryApplyExtrusion
    NS_SWIFT_NAME(tryApplyExtrusion());
- (Core3DModelingOperationResult)tryCancelExtrusion
    NS_SWIFT_NAME(tryCancelExtrusion());
- (BOOL)applyExtrusion;
- (BOOL)cancelExtrusion;
- (Core3DShellParameters)getShellParameters;
- (BOOL)setShellThickness:(CGFloat)thickness;
- (Core3DModelingOperationResult)tryApplyShell
    NS_SWIFT_NAME(tryApplyShell());
- (Core3DModelingOperationResult)tryCancelShell
    NS_SWIFT_NAME(tryCancelShell());
- (BOOL)applyShell;
- (BOOL)cancelShell;
- (Core3DModelingOperationResult)tryApplySubtract
    NS_SWIFT_NAME(tryApplySubtract());
- (Core3DModelingOperationResult)tryCancelSubtract
    NS_SWIFT_NAME(tryCancelSubtract());
- (Core3DModelingOperationResult)tryApplyMirror
    NS_SWIFT_NAME(tryApplyMirror());
- (Core3DModelingOperationResult)tryCancelMirror
    NS_SWIFT_NAME(tryCancelMirror());
- (void)applyMirror;
- (void)cancelMirror;
//! Preview one of the six world-axis planes at the current selection bounds.
//! This synchronous UI operation changes only the transient preview; Apply
//! owns the native document command. It is not a stable-target AI adapter.
- (BOOL)previewMirrorAxis:(Core3DMirrorAxis)axis backward:(BOOL)backward
    NS_SWIFT_NAME(previewMirror(axis:backward:));
- (BOOL)beginMirrorPlanePicking;
- (BOOL)cancelMirrorPlanePicking;
- (BOOL)isPickingMirrorPlane;
- (BOOL)hasCustomMirrorPlane;
- (BOOL)setMirrorPlaneOffset:(CGFloat)offset;
- (Boundaries)getMirrorPlaneOffsetBoundaries;
- (BOOL)resetMirrorPlane;
- (Core3DLinearArrayParameters)getLinearArrayParameters;
- (BOOL)setLinearArrayAxis:(Core3DLinearArrayAxis)axis;
- (BOOL)setLinearArrayCount:(NSInteger)count;
- (BOOL)setLinearArraySpacing:(CGFloat)spacing;
- (Core3DModelingOperationResult)tryApplyLinearArray
    NS_SWIFT_NAME(tryApplyLinearArray());
- (Core3DModelingOperationResult)tryCancelLinearArray
    NS_SWIFT_NAME(tryCancelLinearArray());
- (Core3DRadialArrayParameters)getRadialArrayParameters;
- (Core3DRadialArrayReferenceAuthority)getRadialArrayReferenceAuthority;
//! Read the same captured world reference line expressed in the requested
//! pivot/direction spaces. This is non-mutating and token-checked; failure is
//! returned as Invalid with authorityToken zero.
- (Core3DRadialArrayReferenceAuthority)
    getRadialArrayReferenceAuthorityWithPivotSpace:
        (Core3DReferenceSpace)pivotSpace
    directionSpace:(Core3DReferenceSpace)directionSpace
    expectedAuthorityToken:(uint64_t)expectedAuthorityToken
    NS_SWIFT_NAME(getRadialArrayReferenceAuthority(pivotSpace:directionSpace:expectedAuthorityToken:));
- (BOOL)setRadialArrayCount:(NSInteger)count;
- (BOOL)setRadialArraySweepDegrees:(double)sweepDegrees;
- (Core3DRadialArrayReferenceEditResult)setRadialArrayReferenceAxis:
    (Core3DReferenceAxisValue)axis
    expectedAuthorityToken:(uint64_t)expectedAuthorityToken
    NS_SWIFT_NAME(setRadialArrayReferenceAxis(_:expectedAuthorityToken:));
- (Core3DRadialArrayReferenceEditResult)
    resetRadialArrayReferenceAxisWithExpectedAuthorityToken:
        (uint64_t)expectedAuthorityToken
    NS_SWIFT_NAME(resetRadialArrayReferenceAxis(expectedAuthorityToken:));
- (Core3DModelingOperationResult)tryApplyRadialArray
    NS_SWIFT_NAME(tryApplyRadialArray());
- (Core3DModelingOperationResult)tryCancelRadialArray
    NS_SWIFT_NAME(tryCancelRadialArray());
- (void)applySubtract;
- (void)cancelSubtract;
- (Core3DModelingOperationResult)tryApplyUnion
    NS_SWIFT_NAME(tryApplyUnion());
- (Core3DModelingOperationResult)tryCancelUnion
    NS_SWIFT_NAME(tryCancelUnion());
- (void)applyUnion;
- (void)cancelUnion;
- (Core3DModelingOperationResult)tryApplyIntersect
    NS_SWIFT_NAME(tryApplyIntersect());
- (Core3DModelingOperationResult)tryCancelIntersect
    NS_SWIFT_NAME(tryCancelIntersect());
- (void)applyIntersect;
- (void)cancelIntersect;
- (Core3DModelingPreviewStatus)modelingPreviewStatusForGizmoType:
    (PrimitiveGizmoType)gizmoType
    NS_SWIFT_NAME(modelingPreviewStatus(for:));
- (void)undo;
- (void)redo;
- (NSString *_Nullable)getCoreInfoText;
- (void)setOrthoProjection:(OrthoProjectionType)orthoType;
- (void)setSnappingTranslation:(double)value;
- (double)getSnappingTranslation;
- (void)setSnappingRotation:(double)value;
- (double)getSnappingRotation;
- (void)setSnappingScale:(double)value;
- (double)getSnappingScale;

#ifdef DEBUG
- (void)handleDebug_A;
- (void)handleDebug_B;
#endif

@end

@protocol AvailabilityManagerProtocol<NSObject>

- (BOOL)canAdd;
- (BOOL)canDelete;
- (BOOL)canDuplicate;
- (BOOL)canUndo;
- (BOOL)canRedo;
- (BOOL)canApply;
- (BOOL)canApplyMaterial;

- (Core3DModelCapability)selectedModelCapabilities;
- (NSArray<NSNumber *> *_Nonnull)availableGizmoTypes;
- (BOOL)canExportType:(ExportType)exportType
    NS_SWIFT_NAME(canExport(type:));
- (NSArray<NSNumber *> *_Nonnull)availableExportTypes;

@end

@protocol ExportManagerProtocol<NSObject>

- (NSURL *_Nullable)exportWithType:(ExportType)exportType;
//! Freeze the committed document into a private handoff and return a worker
//! operation. GLB prepares immutable geometry for startSceneSnapshot; OBJ,
//! binary STL, and exact BRep STEP produce files with start. All keep the bounded XBF
//! save on the main thread; translation/writing, validation, and cleanup run
//! off-main.
- (Core3DNativeExportOperation *_Nullable)
    prepareNativeExportOperationWithType:(ExportType)exportType
    NS_SWIFT_NAME(prepareNativeExportOperation(with:));
//! Freeze the current visible whole-object selection for GLB, OBJ or binary STL export.
//! Other selected formats, empty/subshape selections and unresolved object
//! identities reject. NO uses all-visible for GLB and existing all-document
//! behavior for native file exporters.
- (Core3DNativeExportOperation *_Nullable)
    prepareNativeExportOperationWithType:(ExportType)exportType
                    selectedObjectsOnly:(BOOL)selectedObjectsOnly
    NS_SWIFT_NAME(prepareNativeExportOperation(with:selectedObjectsOnly:));
//! Capture an immutable GLB/OBJ/STL mesh quality without changing the viewport or
//! editable source. Invalid qualities and non-viewport STEP presets reject.
- (Core3DNativeExportOperation *_Nullable)
    prepareNativeExportOperationWithType:(ExportType)exportType
                    selectedObjectsOnly:(BOOL)selectedObjectsOnly
                            meshQuality:(Core3DExportMeshQuality)meshQuality
    NS_SWIFT_NAME(prepareNativeExportOperation(with:selectedObjectsOnly:meshQuality:));
//! Capture an explicit OBJ color convention. Other formats require Current;
//! unsupported values reject before private files are created. Older overloads
//! retain Current. This preference never changes authored model appearance.
- (Core3DNativeExportOperation *_Nullable)
    prepareNativeExportOperationWithType:(ExportType)exportType
                    selectedObjectsOnly:(BOOL)selectedObjectsOnly
                            meshQuality:(Core3DExportMeshQuality)meshQuality
                     objColorConvention:(Core3DOBJColorConvention)objColorConvention
    NS_SWIFT_NAME(prepareNativeExportOperation(with:selectedObjectsOnly:meshQuality:objColorConvention:));
//! Capture only committed exportable geometry. Unlike the presentation
//! snapshot seam, this returns nil while a Boolean, Mirror, Linear Array,
//! Radial Array, or Shell trial is active or unresolved, or while the OCAF
//! document owns an open command. Main-thread only; the returned value is an
//! immutable deep copy safe for background I/O.
- (Core3DSceneSnapshot *_Nullable)captureExportSceneSnapshot;

@end





//! Immutable read-only derivative; UV byte payload is six doubles per triangle.
@interface Core3DMeshUVAtlasPreview : NSObject
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@property (nonatomic, readonly, copy) NSData *triangleUVData;
@property (nonatomic, readonly) NSInteger authoredResolution;
@property (nonatomic, readonly) NSInteger authoredGutterPixels;
@property (nonatomic, readonly) NSInteger chartCount;
@property (nonatomic, readonly) double occupancy;
@end

typedef NS_ENUM(NSInteger, Core3DPBRScalarResult) {
    Core3DPBRScalarResultCommitted=0, Core3DPBRScalarResultUnchanged=1,
    Core3DPBRScalarResultInvalid=2, Core3DPBRScalarResultBusy=3, Core3DPBRScalarResultOutcomeUnknown=4
};
__attribute__((objc_subclassing_restricted))
@interface Core3DPBRScalarPreparation : NSObject
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

NS_ASSUME_NONNULL_END
