//
//  Core3DSceneSnapshot.h
//  Core3D
//
//  Immutable renderer-neutral scene values exposed to Objective-C and Swift.
//


#import <Foundation/Foundation.h>
#import <simd/simd.h>

NS_ASSUME_NONNULL_BEGIN

#if __has_attribute(objc_subclassing_restricted)
#define CORE3D_SCENE_FINAL_CLASS __attribute__((objc_subclassing_restricted))
#else
#define CORE3D_SCENE_FINAL_CLASS
#endif

//! Stable scalar vertex layout. Do not replace these members with SIMD vectors:
//! their alignment would change the public buffer ABI.
typedef struct NS_SWIFT_SENDABLE Core3DSceneVertex {
    float positionX;
    float positionY;
    float positionZ;
    float normalX;
    float normalY;
    float normalZ;
    float textureU;
    float textureV;
} Core3DSceneVertex;

typedef NS_ENUM(NSInteger, Core3DSceneProjection) {
    Core3DSceneProjectionPerspective = 0,
    Core3DSceneProjectionOrthographic = 1,
};

typedef NS_ENUM(NSInteger, Core3DSceneAlphaMode) {
    Core3DSceneAlphaModeOpaque = 0,
    Core3DSceneAlphaModeMask = 1,
    Core3DSceneAlphaModeBlend = 2,
};

typedef NS_ENUM(NSInteger, Core3DSceneRenderRole) {
    Core3DSceneRenderRoleModel = 0,
    Core3DSceneRenderRoleSelectionHighlight,
    Core3DSceneRenderRoleBooleanActor,
    Core3DSceneRenderRoleBooleanSubject,
    Core3DSceneRenderRoleChamferPreview,
    Core3DSceneRenderRoleMirrorPreview,
    Core3DSceneRenderRoleGizmo,
    Core3DSceneRenderRoleGrid,
    Core3DSceneRenderRoleTrihedron,
};

typedef NS_ENUM(NSInteger, Core3DSceneCoordinateSpace) {
    Core3DSceneCoordinateSpaceWorld = 0,
    Core3DSceneCoordinateSpaceWorldAnchorPixels,
};

typedef NS_ENUM(NSInteger, Core3DSceneDepthPolicy) {
    Core3DSceneDepthPolicyScene = 0,
    Core3DSceneDepthPolicyTopmost,
};

typedef NS_ENUM(NSInteger, Core3DSceneRenderStyle) {
    Core3DSceneRenderStyleShaded = 0,
    Core3DSceneRenderStyleWireframe,
};

typedef NS_ENUM(NSInteger, Core3DScenePresentationOverlayKind) {
    Core3DScenePresentationOverlayKindNone = 0,
    Core3DScenePresentationOverlayKindMoveRotateGizmo,
    Core3DScenePresentationOverlayKindScaleGizmo,
};

typedef NS_ENUM(NSInteger, Core3DSceneElementKind) {
    Core3DSceneElementKindNone = 0,
    Core3DSceneElementKindObject,
    Core3DSceneElementKindFace,
    Core3DSceneElementKindEdge,
    Core3DSceneElementKindVertex,
};

//! Whether an instance preserves or reverses the winding defined by its mesh.
typedef NS_ENUM(NSInteger, Core3DSceneWinding) {
    Core3DSceneWindingAsDefined = 0,
    Core3DSceneWindingReversed = 1,
};

CORE3D_SCENE_FINAL_CLASS NS_SWIFT_SENDABLE
@interface Core3DSceneRevisionVector : NSObject

//! Monotonic revision of every published snapshot.
@property (nonatomic, assign, readonly) uint64_t snapshotRevision;
@property (nonatomic, assign, readonly) uint64_t documentGeneration;
@property (nonatomic, assign, readonly) uint64_t modelRevision;
@property (nonatomic, assign, readonly) uint64_t presentationRevision;
@property (nonatomic, assign, readonly) uint64_t cameraRevision;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end

CORE3D_SCENE_FINAL_CLASS NS_SWIFT_SENDABLE
@interface Core3DSceneBounds : NSObject

@property (nonatomic, assign, readonly) simd_double3 minimum;
@property (nonatomic, assign, readonly) simd_double3 maximum;
@property (nonatomic, assign, readonly, getter=isValid) BOOL valid;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end

CORE3D_SCENE_FINAL_CLASS NS_SWIFT_SENDABLE
@interface Core3DSceneCameraSnapshot : NSObject

@property (nonatomic, assign, readonly) simd_double3 eye;
@property (nonatomic, assign, readonly) simd_double3 center;
@property (nonatomic, assign, readonly) simd_double3 up;
@property (nonatomic, assign, readonly) Core3DSceneProjection projection;
//! True vertical projection values, independent of viewport aspect.
@property (nonatomic, assign, readonly) double verticalFieldOfViewRadians;
@property (nonatomic, assign, readonly) double orthographicHeight;
@property (nonatomic, assign, readonly) double nearPlane;
@property (nonatomic, assign, readonly) double farPlane;
@property (nonatomic, assign, readonly) double aspectRatio;
@property (nonatomic, assign, readonly) simd_uint2 viewportSizePixels;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end

CORE3D_SCENE_FINAL_CLASS NS_SWIFT_SENDABLE
@interface Core3DSceneMaterialSnapshot : NSObject

@property (nonatomic, copy, readonly) NSString *identifier;
@property (nonatomic, assign, readonly) simd_float4 linearBaseColorRGBA;
@property (nonatomic, assign, readonly) simd_float3 linearEmissionRGB;
@property (nonatomic, assign, readonly) float metallic;
@property (nonatomic, assign, readonly) float roughness;
@property (nonatomic, assign, readonly) float indexOfRefraction;
@property (nonatomic, assign, readonly) Core3DSceneAlphaMode alphaMode;
@property (nonatomic, assign, readonly) float alphaCutoff;
@property (nonatomic, assign, readonly, getter=isDoubleSided) BOOL doubleSided;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end


//! A face-aligned index range owned by a reusable mesh definition.
CORE3D_SCENE_FINAL_CLASS NS_SWIFT_SENDABLE
@interface Core3DSceneFacePrimitiveSnapshot : NSObject

@property (nonatomic, assign, readonly) uint32_t firstIndex;
@property (nonatomic, assign, readonly) uint32_t indexCount;
@property (nonatomic, assign, readonly) uint32_t faceIndex;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end


//! Per-instance styling and picking for the corresponding mesh primitive.
CORE3D_SCENE_FINAL_CLASS NS_SWIFT_SENDABLE
@interface Core3DScenePrimitiveBindingSnapshot : NSObject

@property (nonatomic, assign, readonly) uint32_t materialIndex;
//! Index into Core3DSceneSnapshot.pickTable. Zero always means no hit.
@property (nonatomic, assign, readonly) uint32_t pickToken;
//! Hidden faces remain in mesh topology but are skipped by renderers and picking.
@property (nonatomic, assign, readonly, getter=isVisible) BOOL visible;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end


CORE3D_SCENE_FINAL_CLASS NS_SWIFT_SENDABLE
@interface Core3DSceneMeshSnapshot : NSObject

@property (nonatomic, copy, readonly) NSString *definitionIdentifier;
@property (nonatomic, assign, readonly) uint64_t geometryRevision;
@property (nonatomic, strong, readonly) Core3DSceneBounds *localBounds;

//! Owned interleaved Core3DSceneVertex bytes.
@property (nonatomic, copy, readonly) NSData *vertexData;
//! Owned uint32_t index bytes.
@property (nonatomic, copy, readonly) NSData *indexData;
@property (nonatomic, assign, readonly) NSUInteger vertexCount;
@property (nonatomic, assign, readonly) NSUInteger indexCount;
@property (nonatomic, assign, readonly) NSUInteger vertexStride;
@property (nonatomic, assign, readonly) NSUInteger indexStride;
@property (nonatomic, copy, readonly) NSArray<Core3DSceneFacePrimitiveSnapshot *> *facePrimitives;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end


CORE3D_SCENE_FINAL_CLASS NS_SWIFT_SENDABLE
@interface Core3DSceneRenderItemSnapshot : NSObject

@property (nonatomic, copy, readonly) NSString *entityIdentifier;
@property (nonatomic, assign, readonly) uint32_t meshIndex;
@property (nonatomic, assign, readonly) simd_double4x4 worldTransform;
@property (nonatomic, assign, readonly) Core3DSceneWinding winding;
@property (nonatomic, assign, readonly, getter=isVisible) BOOL visible;
@property (nonatomic, assign, readonly, getter=isSelectable) BOOL selectable;
@property (nonatomic, assign, readonly, getter=isSelected) BOOL selected;
@property (nonatomic, copy, readonly) NSString *name;
@property (nonatomic, assign, readonly) Core3DSceneRenderRole renderRole;
@property (nonatomic, assign, readonly) Core3DSceneCoordinateSpace coordinateSpace;
@property (nonatomic, assign, readonly) Core3DSceneDepthPolicy depthPolicy;
@property (nonatomic, assign, readonly) Core3DSceneRenderStyle renderStyle;
//! One binding for each face primitive in the referenced mesh.
@property (nonatomic, copy, readonly) NSArray<Core3DScenePrimitiveBindingSnapshot *> *primitiveBindings;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end


CORE3D_SCENE_FINAL_CLASS NS_SWIFT_SENDABLE
@interface Core3DSceneElementIdentifier : NSObject

@property (nonatomic, copy, readonly) NSString *entityIdentifier;
@property (nonatomic, assign, readonly) Core3DSceneElementKind kind;
@property (nonatomic, assign, readonly) uint32_t topologyIndex;
@property (nonatomic, assign, readonly) uint64_t geometryRevision;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end


CORE3D_SCENE_FINAL_CLASS NS_SWIFT_SENDABLE
@interface Core3DSceneSelectionSnapshot : NSObject

@property (nonatomic, copy, readonly) NSArray<Core3DSceneElementIdentifier *> *selectedElements;
@property (nonatomic, strong, readonly, nullable) Core3DSceneElementIdentifier *hoveredElement;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end


//! Lightweight camera publication paired with the most recent committed full
//! scene. Document, model, and presentation revisions stay anchored to that
//! scene; snapshot and camera revisions may advance. This object contains no
//! mesh or material payload.
CORE3D_SCENE_FINAL_CLASS NS_SWIFT_SENDABLE
@interface Core3DSceneFrameSnapshot : NSObject

@property (nonatomic, copy, readonly) NSString *publicationSourceIdentifier;
@property (nonatomic, strong, readonly) Core3DSceneRevisionVector *revisions;
@property (nonatomic, strong, readonly) Core3DSceneCameraSnapshot *camera;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end


//! Transient renderer-neutral presentation paired with one exact committed
//! full-scene publication. Empty arrays are a valid explicit overlay clear.
CORE3D_SCENE_FINAL_CLASS NS_SWIFT_SENDABLE
@interface Core3DScenePresentationOverlaySnapshot : NSObject

@property (nonatomic, assign, readonly) uint32_t schemaVersion;
@property (nonatomic, assign, readonly) Core3DScenePresentationOverlayKind kind;
@property (nonatomic, copy, readonly) NSString *publicationSourceIdentifier;
@property (nonatomic, assign, readonly) uint64_t baseSnapshotRevision;
@property (nonatomic, assign, readonly) uint64_t baseDocumentGeneration;
@property (nonatomic, assign, readonly) uint64_t baseModelRevision;
@property (nonatomic, assign, readonly) uint64_t basePresentationRevision;
@property (nonatomic, assign, readonly) uint64_t overlayRevision;
@property (nonatomic, copy, readonly) NSArray<Core3DSceneMeshSnapshot *> *meshes;
@property (nonatomic, copy, readonly) NSArray<Core3DSceneRenderItemSnapshot *> *renderItems;
@property (nonatomic, copy, readonly) NSArray<Core3DSceneMaterialSnapshot *> *materials;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end


CORE3D_SCENE_FINAL_CLASS NS_SWIFT_SENDABLE
@interface Core3DSceneSnapshot : NSObject

@property (nonatomic, assign, readonly) uint32_t schemaVersion;
@property (nonatomic, copy, readonly) NSString *publicationSourceIdentifier;
@property (nonatomic, strong, readonly) Core3DSceneRevisionVector *revisions;
//! Convenience alias for revisions.snapshotRevision.
@property (nonatomic, assign, readonly) uint64_t snapshotRevision;
//! World-space origin subtracted before float vertex publication.
@property (nonatomic, assign, readonly) simd_double3 renderOrigin;
@property (nonatomic, copy, readonly) NSArray<Core3DSceneMeshSnapshot *> *meshes;
@property (nonatomic, copy, readonly) NSArray<Core3DSceneRenderItemSnapshot *> *renderItems;
@property (nonatomic, copy, readonly) NSArray<Core3DSceneMaterialSnapshot *> *materials;
//! GPU pick token to semantic element mapping. Index zero is no hit.
@property (nonatomic, copy, readonly) NSArray<Core3DSceneElementIdentifier *> *pickTable;
@property (nonatomic, strong, readonly) Core3DSceneCameraSnapshot *camera;
@property (nonatomic, strong, readonly) Core3DSceneSelectionSnapshot *selection;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end

NS_ASSUME_NONNULL_END

#undef CORE3D_SCENE_FINAL_CLASS
