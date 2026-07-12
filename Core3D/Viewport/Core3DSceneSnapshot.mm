//
//  Core3DSceneSnapshot.mm
//  Core3D
//


#import "Core3DSceneSnapshot.h"
#import "Core3DSceneSnapshotFactory.hpp"

#include "../Scene/SceneSnapshot.hpp"
#include <Quantity_Color.hxx>
#include <Quantity_NameOfColor.hxx>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace core3d::scene;

static_assert(sizeof(Core3DSceneVertex) == sizeof(float) * 8,
              "Public scene vertices must remain exactly eight scalar floats");
static_assert(sizeof(Core3DSceneVertex) == sizeof(Vertex),
              "Public and renderer-neutral vertex layouts must agree");
static_assert(offsetof(Core3DSceneVertex, positionX) == offsetof(Vertex, positionX));
static_assert(offsetof(Core3DSceneVertex, positionY) == offsetof(Vertex, positionY));
static_assert(offsetof(Core3DSceneVertex, positionZ) == offsetof(Vertex, positionZ));
static_assert(offsetof(Core3DSceneVertex, normalX) == offsetof(Vertex, normalX));
static_assert(offsetof(Core3DSceneVertex, normalY) == offsetof(Vertex, normalY));
static_assert(offsetof(Core3DSceneVertex, normalZ) == offsetof(Vertex, normalZ));
static_assert(offsetof(Core3DSceneVertex, textureU) == offsetof(Vertex, textureU));
static_assert(offsetof(Core3DSceneVertex, textureV) == offsetof(Vertex, textureV));
static_assert(sizeof(std::uint32_t) == 4,
              "Scene indices require a four-byte uint32_t ABI");

@interface Core3DSceneRevisionVector ()
- (instancetype)initWithSnapshotRevision:(uint64_t)snapshotRevision
                       documentGeneration:(uint64_t)documentGeneration
                            modelRevision:(uint64_t)modelRevision
                     presentationRevision:(uint64_t)presentationRevision
                           cameraRevision:(uint64_t)cameraRevision;
@end

@interface Core3DSceneBounds ()
- (instancetype)initWithMinimum:(simd_double3)minimum
                         maximum:(simd_double3)maximum
                           valid:(BOOL)valid;
@end

@interface Core3DSceneCameraSnapshot ()
- (instancetype)initWithEye:(simd_double3)eye
                     center:(simd_double3)center
                         up:(simd_double3)up
                 projection:(Core3DSceneProjection)projection
 verticalFieldOfViewRadians:(double)verticalFieldOfViewRadians
         orthographicHeight:(double)orthographicHeight
                  nearPlane:(double)nearPlane
                   farPlane:(double)farPlane
                aspectRatio:(double)aspectRatio
         viewportSizePixels:(simd_uint2)viewportSizePixels;
@end

@interface Core3DSceneMaterialSnapshot ()
- (instancetype)initWithIdentifier:(NSString *)identifier
                linearBaseColorRGBA:(simd_float4)linearBaseColorRGBA
                  linearEmissionRGB:(simd_float3)linearEmissionRGB
                           metallic:(float)metallic
                          roughness:(float)roughness
                  indexOfRefraction:(float)indexOfRefraction
                          alphaMode:(Core3DSceneAlphaMode)alphaMode
                        alphaCutoff:(float)alphaCutoff
                        doubleSided:(BOOL)doubleSided;
@end

@interface Core3DSceneFacePrimitiveSnapshot ()
- (instancetype)initWithFirstIndex:(uint32_t)firstIndex
                         indexCount:(uint32_t)indexCount
                          faceIndex:(uint32_t)faceIndex;
@end

@interface Core3DScenePrimitiveBindingSnapshot ()
- (instancetype)initWithMaterialIndex:(uint32_t)materialIndex
                            pickToken:(uint32_t)pickToken
                              visible:(BOOL)visible;
@end

@interface Core3DSceneMeshSnapshot ()
- (instancetype)initWithDefinitionIdentifier:(NSString *)definitionIdentifier
                             geometryRevision:(uint64_t)geometryRevision
                                  localBounds:(Core3DSceneBounds *)localBounds
                                   vertexData:(NSData *)vertexData
                                    indexData:(NSData *)indexData
                                  vertexCount:(NSUInteger)vertexCount
                                   indexCount:(NSUInteger)indexCount
                               facePrimitives:(NSArray<Core3DSceneFacePrimitiveSnapshot *> *)facePrimitives;
@end

@interface Core3DSceneRenderItemSnapshot ()
- (instancetype)initWithEntityIdentifier:(NSString *)entityIdentifier
                                meshIndex:(uint32_t)meshIndex
                           worldTransform:(simd_double4x4)worldTransform
                                  winding:(Core3DSceneWinding)winding
                                  visible:(BOOL)visible
                               selectable:(BOOL)selectable
                                 selected:(BOOL)selected
                                     name:(NSString *)name
                               renderRole:(Core3DSceneRenderRole)renderRole
                          coordinateSpace:(Core3DSceneCoordinateSpace)coordinateSpace
                              depthPolicy:(Core3DSceneDepthPolicy)depthPolicy
                              renderStyle:(Core3DSceneRenderStyle)renderStyle
                        primitiveBindings:(NSArray<Core3DScenePrimitiveBindingSnapshot *> *)primitiveBindings;
@end

@interface Core3DSceneElementIdentifier ()
- (instancetype)initWithEntityIdentifier:(NSString *)entityIdentifier
                                     kind:(Core3DSceneElementKind)kind
                            topologyIndex:(uint32_t)topologyIndex
                         geometryRevision:(uint64_t)geometryRevision;
@end

@interface Core3DSceneSelectionSnapshot ()
- (instancetype)initWithSelectedElements:(NSArray<Core3DSceneElementIdentifier *> *)selectedElements
                           hoveredElement:(nullable Core3DSceneElementIdentifier *)hoveredElement;
@end

@interface Core3DSceneFrameSnapshot ()
- (instancetype)initWithPublicationSourceIdentifier:(NSString *)publicationSourceIdentifier
                                           revisions:(Core3DSceneRevisionVector *)revisions
                            camera:(Core3DSceneCameraSnapshot *)camera;
@end

@interface Core3DScenePresentationOverlaySnapshot ()
- (instancetype)initWithSchemaVersion:(uint32_t)schemaVersion
                                  kind:(Core3DScenePresentationOverlayKind)kind
           publicationSourceIdentifier:(NSString *)publicationSourceIdentifier
                  baseSnapshotRevision:(uint64_t)baseSnapshotRevision
                baseDocumentGeneration:(uint64_t)baseDocumentGeneration
                     baseModelRevision:(uint64_t)baseModelRevision
              basePresentationRevision:(uint64_t)basePresentationRevision
                       overlayRevision:(uint64_t)overlayRevision
                                meshes:(NSArray<Core3DSceneMeshSnapshot *> *)meshes
                           renderItems:(NSArray<Core3DSceneRenderItemSnapshot *> *)renderItems
                             materials:(NSArray<Core3DSceneMaterialSnapshot *> *)materials
           suppressedEntityIdentifiers:(NSArray<NSString *> *)suppressedEntityIdentifiers;
@end

@interface Core3DSceneSnapshot ()
- (instancetype)initWithSchemaVersion:(uint32_t)schemaVersion
           publicationSourceIdentifier:(NSString *)publicationSourceIdentifier
                             revisions:(Core3DSceneRevisionVector *)revisions
                          renderOrigin:(simd_double3)renderOrigin
                                meshes:(NSArray<Core3DSceneMeshSnapshot *> *)meshes
                           renderItems:(NSArray<Core3DSceneRenderItemSnapshot *> *)renderItems
                             materials:(NSArray<Core3DSceneMaterialSnapshot *> *)materials
                             pickTable:(NSArray<Core3DSceneElementIdentifier *> *)pickTable
                                camera:(Core3DSceneCameraSnapshot *)camera
                             selection:(Core3DSceneSelectionSnapshot *)selection;
@end


@implementation Core3DSceneRevisionVector

- (instancetype)initWithSnapshotRevision:(uint64_t)snapshotRevision
                       documentGeneration:(uint64_t)documentGeneration
                            modelRevision:(uint64_t)modelRevision
                     presentationRevision:(uint64_t)presentationRevision
                           cameraRevision:(uint64_t)cameraRevision {
    self = [super init];
    if (self) {
        _snapshotRevision = snapshotRevision;
        _documentGeneration = documentGeneration;
        _modelRevision = modelRevision;
        _presentationRevision = presentationRevision;
        _cameraRevision = cameraRevision;
    }
    return self;
}

@end


@implementation Core3DSceneBounds

- (instancetype)initWithMinimum:(simd_double3)minimum
                         maximum:(simd_double3)maximum
                           valid:(BOOL)valid {
    self = [super init];
    if (self) {
        _minimum = minimum;
        _maximum = maximum;
        _valid = valid;
    }
    return self;
}

@end


@implementation Core3DSceneCameraSnapshot

- (instancetype)initWithEye:(simd_double3)eye
                     center:(simd_double3)center
                         up:(simd_double3)up
                 projection:(Core3DSceneProjection)projection
 verticalFieldOfViewRadians:(double)verticalFieldOfViewRadians
         orthographicHeight:(double)orthographicHeight
                  nearPlane:(double)nearPlane
                   farPlane:(double)farPlane
                aspectRatio:(double)aspectRatio
         viewportSizePixels:(simd_uint2)viewportSizePixels {
    self = [super init];
    if (self) {
        _eye = eye;
        _center = center;
        _up = up;
        _projection = projection;
        _verticalFieldOfViewRadians = verticalFieldOfViewRadians;
        _orthographicHeight = orthographicHeight;
        _nearPlane = nearPlane;
        _farPlane = farPlane;
        _aspectRatio = aspectRatio;
        _viewportSizePixels = viewportSizePixels;
    }
    return self;
}

@end


@implementation Core3DSceneMaterialSnapshot

- (instancetype)initWithIdentifier:(NSString *)identifier
                linearBaseColorRGBA:(simd_float4)linearBaseColorRGBA
                  linearEmissionRGB:(simd_float3)linearEmissionRGB
                           metallic:(float)metallic
                          roughness:(float)roughness
                  indexOfRefraction:(float)indexOfRefraction
                          alphaMode:(Core3DSceneAlphaMode)alphaMode
                        alphaCutoff:(float)alphaCutoff
                        doubleSided:(BOOL)doubleSided {
    self = [super init];
    if (self) {
        _identifier = [identifier copy];
        _linearBaseColorRGBA = linearBaseColorRGBA;
        _linearEmissionRGB = linearEmissionRGB;
        _metallic = metallic;
        _roughness = roughness;
        _indexOfRefraction = indexOfRefraction;
        _alphaMode = alphaMode;
        _alphaCutoff = alphaCutoff;
        _doubleSided = doubleSided;
    }
    return self;
}

@end


@implementation Core3DSceneFacePrimitiveSnapshot

- (instancetype)initWithFirstIndex:(uint32_t)firstIndex
                         indexCount:(uint32_t)indexCount
                          faceIndex:(uint32_t)faceIndex {
    self = [super init];
    if (self) {
        _firstIndex = firstIndex;
        _indexCount = indexCount;
        _faceIndex = faceIndex;
    }
    return self;
}

@end


@implementation Core3DScenePrimitiveBindingSnapshot

- (instancetype)initWithMaterialIndex:(uint32_t)materialIndex
                            pickToken:(uint32_t)pickToken
                              visible:(BOOL)visible {
    self = [super init];
    if (self) {
        _materialIndex = materialIndex;
        _pickToken = pickToken;
        _visible = visible;
    }
    return self;
}

@end


@implementation Core3DSceneMeshSnapshot

- (instancetype)initWithDefinitionIdentifier:(NSString *)definitionIdentifier
                             geometryRevision:(uint64_t)geometryRevision
                                  localBounds:(Core3DSceneBounds *)localBounds
                                   vertexData:(NSData *)vertexData
                                    indexData:(NSData *)indexData
                                  vertexCount:(NSUInteger)vertexCount
                                   indexCount:(NSUInteger)indexCount
                               facePrimitives:(NSArray<Core3DSceneFacePrimitiveSnapshot *> *)facePrimitives {
    self = [super init];
    if (self) {
        NSParameterAssert(vertexData.length % sizeof(Core3DSceneVertex) == 0);
        NSParameterAssert(indexData.length % sizeof(uint32_t) == 0);
        NSParameterAssert(vertexData.length / sizeof(Core3DSceneVertex) == vertexCount);
        NSParameterAssert(indexData.length / sizeof(uint32_t) == indexCount);

        _definitionIdentifier = [definitionIdentifier copy];
        _geometryRevision = geometryRevision;
        _localBounds = localBounds;
        _vertexData = [vertexData copy];
        _indexData = [indexData copy];
        _vertexCount = vertexCount;
        _indexCount = indexCount;
        _vertexStride = sizeof(Core3DSceneVertex);
        _indexStride = sizeof(uint32_t);
        _facePrimitives = [facePrimitives copy];
    }
    return self;
}

@end


@implementation Core3DSceneRenderItemSnapshot

- (instancetype)initWithEntityIdentifier:(NSString *)entityIdentifier
                                meshIndex:(uint32_t)meshIndex
                           worldTransform:(simd_double4x4)worldTransform
                                  winding:(Core3DSceneWinding)winding
                                  visible:(BOOL)visible
                               selectable:(BOOL)selectable
                                 selected:(BOOL)selected
                                     name:(NSString *)name
                               renderRole:(Core3DSceneRenderRole)renderRole
                          coordinateSpace:(Core3DSceneCoordinateSpace)coordinateSpace
                              depthPolicy:(Core3DSceneDepthPolicy)depthPolicy
                              renderStyle:(Core3DSceneRenderStyle)renderStyle
                        primitiveBindings:(NSArray<Core3DScenePrimitiveBindingSnapshot *> *)primitiveBindings {
    self = [super init];
    if (self) {
        _entityIdentifier = [entityIdentifier copy];
        _meshIndex = meshIndex;
        _worldTransform = worldTransform;
        _winding = winding;
        _visible = visible;
        _selectable = selectable;
        _selected = selected;
        _name = [name copy];
        _renderRole = renderRole;
        _coordinateSpace = coordinateSpace;
        _depthPolicy = depthPolicy;
        _renderStyle = renderStyle;
        _primitiveBindings = [primitiveBindings copy];
    }
    return self;
}

@end


@implementation Core3DSceneElementIdentifier

- (instancetype)initWithEntityIdentifier:(NSString *)entityIdentifier
                                     kind:(Core3DSceneElementKind)kind
                            topologyIndex:(uint32_t)topologyIndex
                         geometryRevision:(uint64_t)geometryRevision {
    self = [super init];
    if (self) {
        _entityIdentifier = [entityIdentifier copy];
        _kind = kind;
        _topologyIndex = topologyIndex;
        _geometryRevision = geometryRevision;
    }
    return self;
}

@end


@implementation Core3DSceneSelectionSnapshot

- (instancetype)initWithSelectedElements:(NSArray<Core3DSceneElementIdentifier *> *)selectedElements
                           hoveredElement:(Core3DSceneElementIdentifier *)hoveredElement {
    self = [super init];
    if (self) {
        _selectedElements = [selectedElements copy];
        _hoveredElement = hoveredElement;
    }
    return self;
}

@end


@implementation Core3DSceneFrameSnapshot

- (instancetype)initWithPublicationSourceIdentifier:(NSString *)publicationSourceIdentifier
                                           revisions:(Core3DSceneRevisionVector *)revisions
                            camera:(Core3DSceneCameraSnapshot *)camera {
    self = [super init];
    if (self) {
        _publicationSourceIdentifier = [publicationSourceIdentifier copy];
        _revisions = revisions;
        _camera = camera;
    }
    return self;
}

@end


@implementation Core3DScenePresentationOverlaySnapshot

- (instancetype)initWithSchemaVersion:(uint32_t)schemaVersion
                                  kind:(Core3DScenePresentationOverlayKind)kind
           publicationSourceIdentifier:(NSString *)publicationSourceIdentifier
                  baseSnapshotRevision:(uint64_t)baseSnapshotRevision
                baseDocumentGeneration:(uint64_t)baseDocumentGeneration
                     baseModelRevision:(uint64_t)baseModelRevision
              basePresentationRevision:(uint64_t)basePresentationRevision
                       overlayRevision:(uint64_t)overlayRevision
                                meshes:(NSArray<Core3DSceneMeshSnapshot *> *)meshes
                           renderItems:(NSArray<Core3DSceneRenderItemSnapshot *> *)renderItems
                             materials:(NSArray<Core3DSceneMaterialSnapshot *> *)materials
           suppressedEntityIdentifiers:(NSArray<NSString *> *)suppressedEntityIdentifiers {
    self = [super init];
    if (self) {
        _schemaVersion = schemaVersion;
        _kind = kind;
        _publicationSourceIdentifier = [publicationSourceIdentifier copy];
        _baseSnapshotRevision = baseSnapshotRevision;
        _baseDocumentGeneration = baseDocumentGeneration;
        _baseModelRevision = baseModelRevision;
        _basePresentationRevision = basePresentationRevision;
        _overlayRevision = overlayRevision;
        _meshes = [meshes copy];
        _renderItems = [renderItems copy];
        _materials = [materials copy];
        _suppressedEntityIdentifiers = [suppressedEntityIdentifiers copy];
    }
    return self;
}

@end


@implementation Core3DSceneSnapshot

- (instancetype)initWithSchemaVersion:(uint32_t)schemaVersion
           publicationSourceIdentifier:(NSString *)publicationSourceIdentifier
                             revisions:(Core3DSceneRevisionVector *)revisions
                          renderOrigin:(simd_double3)renderOrigin
                                meshes:(NSArray<Core3DSceneMeshSnapshot *> *)meshes
                           renderItems:(NSArray<Core3DSceneRenderItemSnapshot *> *)renderItems
                             materials:(NSArray<Core3DSceneMaterialSnapshot *> *)materials
                             pickTable:(NSArray<Core3DSceneElementIdentifier *> *)pickTable
                                camera:(Core3DSceneCameraSnapshot *)camera
                             selection:(Core3DSceneSelectionSnapshot *)selection {
    self = [super init];
    if (self) {
        _schemaVersion = schemaVersion;
        _publicationSourceIdentifier = [publicationSourceIdentifier copy];
        _revisions = revisions;
        _renderOrigin = renderOrigin;
        _meshes = [meshes copy];
        _renderItems = [renderItems copy];
        _materials = [materials copy];
        _pickTable = [pickTable copy];
        _camera = camera;
        _selection = selection;
    }
    return self;
}

- (uint64_t)snapshotRevision {
    return self.revisions.snapshotRevision;
}

@end


namespace {

constexpr std::size_t kMaximumDTOMeshes = 50'000;
constexpr std::size_t kMaximumDTOInstances = 50'000;
constexpr std::size_t kMaximumDTOMaterials = 50'000;
constexpr std::size_t kMaximumDTOPrimitives = 250'000;
constexpr std::size_t kMaximumDTOBindings = 250'000;
constexpr std::size_t kMaximumDTOPickEntries = 250'001;
constexpr std::size_t kMaximumDTOSelectedElements = 50'000;
constexpr std::size_t kMaximumDTOVertices = 1'500'000;
constexpr std::size_t kMaximumDTOIndices = 4'500'000;
constexpr std::size_t kMaximumDTONumericBytes = 96ULL * 1024ULL * 1024ULL;
constexpr std::size_t kMaximumDTOStringBytes = 16ULL * 1024ULL * 1024ULL;
constexpr std::size_t kMaximumIdentifierBytes = 128;
constexpr std::size_t kMaximumNameBytes = 4'096;
constexpr std::size_t kMaximumOverlayMeshes = 16;
constexpr std::size_t kMaximumOverlayInstances = 16;
constexpr std::size_t kMaximumOverlayMaterials = 16;
constexpr std::size_t kMaximumOverlayVertices = 100'000;
constexpr std::size_t kMaximumOverlayIndices = 300'000;
constexpr std::size_t kMaximumOverlayPrimitives = 25'000;
constexpr std::size_t kMaximumOverlayBindings = 25'000;
constexpr std::size_t kMaximumOverlayNumericBytes = 16ULL * 1024ULL * 1024ULL;
constexpr std::size_t kMaximumMirrorPreviewBodies = 8;
constexpr std::size_t kMaximumBooleanSourceOperands = 8;
constexpr std::array<const char*, 6> kMirrorEntityIdentifiers = {
    "gizmo/mirroring/x/negative",
    "gizmo/mirroring/y/negative",
    "gizmo/mirroring/z/negative",
    "gizmo/mirroring/x/positive",
    "gizmo/mirroring/y/positive",
    "gizmo/mirroring/z/positive",
};
constexpr std::array<const char*, 6> kMirrorMeshIdentifiers = {
    "gizmo/mirroring/x/negative/mesh",
    "gizmo/mirroring/y/negative/mesh",
    "gizmo/mirroring/z/negative/mesh",
    "gizmo/mirroring/x/positive/mesh",
    "gizmo/mirroring/y/positive/mesh",
    "gizmo/mirroring/z/positive/mesh",
};
constexpr std::array<const char*, 6> kMirrorMaterialIdentifiers = {
    "gizmo/material/mirroring/x/negative",
    "gizmo/material/mirroring/y/negative",
    "gizmo/material/mirroring/z/negative",
    "gizmo/material/mirroring/x/positive",
    "gizmo/material/mirroring/y/positive",
    "gizmo/material/mirroring/z/positive",
};
constexpr std::array<const char*, 6> kMirrorNames = {
    "Negative X mirror plane",
    "Negative Y mirror plane",
    "Negative Z mirror plane",
    "Positive X mirror plane",
    "Positive Y mirror plane",
    "Positive Z mirror plane",
};

bool CheckedAdd(
    const std::size_t left,
    const std::size_t right,
    std::size_t& result) noexcept {
    if (right > std::numeric_limits<std::size_t>::max() - left) {
        return false;
    }
    result = left + right;
    return true;
}

bool CheckedMultiply(
    const std::size_t left,
    const std::size_t right,
    std::size_t& result) noexcept {
    if (left != 0 && right > std::numeric_limits<std::size_t>::max() / left) {
        return false;
    }
    result = left * right;
    return true;
}

bool IsValidIdentifier(const std::string& value) noexcept {
    if (value.empty() || value.size() > kMaximumIdentifierBytes) {
        return false;
    }
    for (const unsigned char character : value) {
        if (character < 0x21U || character > 0x7eU) {
            return false;
        }
    }
    return true;
}

bool IsValidPublicationSourceIdentifier(const std::string& value) noexcept {
    if (value.size() != 36) {
        return false;
    }
    for (std::size_t index = 0; index < value.size(); ++index) {
        const bool isSeparator = index == 8 || index == 13
            || index == 18 || index == 23;
        const unsigned char character = value[index];
        if (isSeparator) {
            if (character != '-') {
                return false;
            }
            continue;
        }
        const bool isHex = (character >= '0' && character <= '9')
            || (character >= 'a' && character <= 'f')
            || (character >= 'A' && character <= 'F');
        if (!isHex) {
            return false;
        }
    }
    return true;
}

struct ElementKey {
    std::string entityIdentifier;
    ElementKind kind = ElementKind::None;
    std::uint32_t topologyIndex = 0;
    std::uint64_t geometryRevision = 0;

    bool operator==(const ElementKey& other) const noexcept {
        return entityIdentifier == other.entityIdentifier
            && kind == other.kind
            && topologyIndex == other.topologyIndex
            && geometryRevision == other.geometryRevision;
    }
};

struct ElementKeyHash {
    std::size_t operator()(const ElementKey& value) const noexcept {
        std::size_t result = std::hash<std::string>{}(value.entityIdentifier);
        const auto combine = [&result](const std::size_t component) {
            result ^= component + static_cast<std::size_t>(0x9e3779b9U)
                + (result << 6U) + (result >> 2U);
        };
        combine(std::hash<unsigned>{}(
            static_cast<unsigned>(value.kind)));
        combine(std::hash<std::uint32_t>{}(value.topologyIndex));
        combine(std::hash<std::uint64_t>{}(value.geometryRevision));
        return result;
    }
};

ElementKey MakeElementKey(const ElementIdentifier& value) {
    return {
        value.entityIdentifier,
        value.kind,
        value.topologyIndex,
        value.geometryRevision,
    };
}

bool IsFinite(const float value) noexcept {
    return std::isfinite(value);
}

bool IsFinite(const double value) noexcept {
    return std::isfinite(value);
}

bool IsFinite(const Double3& value) noexcept {
    return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

bool IsFinite(const Float3& value) noexcept {
    return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

bool IsFinite(const Float4& value) noexcept {
    return IsFinite(value.x) && IsFinite(value.y)
        && IsFinite(value.z) && IsFinite(value.w);
}

bool IsValid(const Bounds3d& value) noexcept {
    if (!value.valid) {
        return true;
    }
    return IsFinite(value.minimum) && IsFinite(value.maximum)
        && value.minimum.x <= value.maximum.x
        && value.minimum.y <= value.maximum.y
        && value.minimum.z <= value.maximum.z;
}

bool IsValid(const Projection value) noexcept {
    switch (value) {
        case Projection::Perspective:
        case Projection::Orthographic:
            return true;
    }
    return false;
}

bool IsValid(const AlphaMode value) noexcept {
    switch (value) {
        case AlphaMode::Opaque:
        case AlphaMode::Mask:
        case AlphaMode::Blend:
            return true;
    }
    return false;
}

bool IsValid(const RenderRole value) noexcept {
    switch (value) {
        case RenderRole::Model:
        case RenderRole::SelectionHighlight:
        case RenderRole::BooleanActor:
        case RenderRole::BooleanSubject:
        case RenderRole::ChamferPreview:
        case RenderRole::MirrorPreview:
        case RenderRole::Gizmo:
        case RenderRole::Grid:
        case RenderRole::Trihedron:
            return true;
    }
    return false;
}

bool IsValid(const CoordinateSpace value) noexcept {
    switch (value) {
        case CoordinateSpace::World:
        case CoordinateSpace::WorldAnchorPixels:
            return true;
    }
    return false;
}

bool IsValid(const DepthPolicy value) noexcept {
    switch (value) {
        case DepthPolicy::Scene:
        case DepthPolicy::Topmost:
            return true;
    }
    return false;
}

bool IsValid(const RenderStyle value) noexcept {
    switch (value) {
        case RenderStyle::Shaded:
        case RenderStyle::Wireframe:
            return true;
    }
    return false;
}

bool IsValid(const ElementKind value) noexcept {
    switch (value) {
        case ElementKind::None:
        case ElementKind::Object:
        case ElementKind::Face:
        case ElementKind::Edge:
        case ElementKind::Vertex:
            return true;
    }
    return false;
}

bool IsValidElement(
    const ElementIdentifier& value,
    const bool allowNoHit) noexcept {
    if (!IsValid(value.kind)) {
        return false;
    }
    if (value.kind == ElementKind::None) {
        return allowNoHit && value.entityIdentifier.empty();
    }
    return IsValidIdentifier(value.entityIdentifier);
}

bool IsValid(const CameraSnapshot& value) noexcept {
    if (!IsValid(value.projection)
        || !IsFinite(value.eye) || !IsFinite(value.center)
        || !IsFinite(value.up) || !IsFinite(value.verticalFovRadians)
        || !IsFinite(value.orthographicHeight) || !IsFinite(value.nearPlane)
        || !IsFinite(value.farPlane) || !IsFinite(value.aspect)
        || value.viewportPixels.x == 0 || value.viewportPixels.y == 0
        || value.aspect <= 0.0 || value.orthographicHeight <= 0.0
        || value.nearPlane >= value.farPlane) {
        return false;
    }

    const Double3 direction = {
        value.center.x - value.eye.x,
        value.center.y - value.eye.y,
        value.center.z - value.eye.z,
    };
    const Double3 cross = {
        direction.y * value.up.z - direction.z * value.up.y,
        direction.z * value.up.x - direction.x * value.up.z,
        direction.x * value.up.y - direction.y * value.up.x,
    };
    const double directionSquared = direction.x * direction.x
        + direction.y * direction.y + direction.z * direction.z;
    const double upSquared = value.up.x * value.up.x
        + value.up.y * value.up.y + value.up.z * value.up.z;
    const double crossSquared = cross.x * cross.x
        + cross.y * cross.y + cross.z * cross.z;
    if (!IsFinite(directionSquared) || !IsFinite(upSquared)
        || !IsFinite(crossSquared) || directionSquared <= 1.0e-24
        || upSquared <= 1.0e-24
        || crossSquared <= 1.0e-24 * directionSquared * upSquared) {
        return false;
    }

    if (value.projection == Projection::Perspective) {
        constexpr double kPi = 3.14159265358979323846;
        return value.nearPlane > 0.0
            && value.verticalFovRadians > 0.0
            && value.verticalFovRadians < kPi;
    }
    return true;
}

bool IsValid(const MaterialSnapshot& value) noexcept {
    return IsValidIdentifier(value.identifier)
        && IsFinite(value.baseColor) && IsFinite(value.emission)
        && IsFinite(value.metallic) && IsFinite(value.roughness)
        && IsFinite(value.indexOfRefraction) && IsFinite(value.alphaCutoff)
        && value.metallic >= 0.0f && value.metallic <= 1.0f
        && value.roughness >= 0.0f && value.roughness <= 1.0f
        && value.indexOfRefraction > 0.0f
        && value.alphaCutoff >= 0.0f && value.alphaCutoff <= 1.0f
        && IsValid(value.alphaMode);
}

bool IsValidSceneSnapshotImpl(const SceneSnapshot& snapshot) {
    if (snapshot.schemaVersion != kSceneSnapshotSchemaVersion
        || !IsValidPublicationSourceIdentifier(
            snapshot.publicationSourceIdentifier)
        || snapshot.revisions.snapshot == 0
        || snapshot.revisions.documentGeneration == 0
        || snapshot.revisions.model == 0
        || snapshot.revisions.presentation == 0
        || snapshot.revisions.camera == 0
        || !IsFinite(snapshot.renderOrigin)
        || !IsValid(snapshot.camera)
        || snapshot.meshes.size() > kMaximumDTOMeshes
        || snapshot.instances.size() > kMaximumDTOInstances
        || snapshot.materials.size() > kMaximumDTOMaterials
        || snapshot.pickTable.empty()
        || snapshot.pickTable.size() > kMaximumDTOPickEntries
        || snapshot.selection.selected.size()
            > kMaximumDTOSelectedElements
        || !IsValidElement(snapshot.pickTable.front(), true)
        || snapshot.pickTable.front().kind != ElementKind::None
        || snapshot.pickTable.front().topologyIndex != 0
        || snapshot.pickTable.front().geometryRevision != 0) {
        return false;
    }

    std::size_t totalStringBytes = 0;
    const auto accountString = [&totalStringBytes](const std::string& value) {
        return CheckedAdd(totalStringBytes, value.size(), totalStringBytes)
            && totalStringBytes <= kMaximumDTOStringBytes;
    };
    if (!accountString(snapshot.publicationSourceIdentifier)) {
        return false;
    }

    std::unordered_set<std::string> materialIdentifiers;
    materialIdentifiers.reserve(snapshot.materials.size());
    for (const MaterialSnapshot& material : snapshot.materials) {
        if (!IsValid(material) || !accountString(material.identifier)
            || !materialIdentifiers.insert(material.identifier).second) {
            return false;
        }
    }

    std::size_t totalVertices = 0;
    std::size_t totalIndices = 0;
    std::size_t totalPrimitives = 0;
    std::size_t totalNumericBytes = 0;
    std::unordered_set<std::string> definitionIdentifiers;
    definitionIdentifiers.reserve(snapshot.meshes.size());
    for (const MeshSnapshot& mesh : snapshot.meshes) {
        if (!IsValidIdentifier(mesh.definitionIdentifier)
            || !accountString(mesh.definitionIdentifier)
            || !definitionIdentifiers.insert(mesh.definitionIdentifier).second
            || mesh.geometryRevision == 0
            || mesh.vertices.empty() || mesh.indices.empty()
            || !mesh.localBounds.valid || !IsValid(mesh.localBounds)
            || !CheckedAdd(totalVertices, mesh.vertices.size(), totalVertices)
            || totalVertices > kMaximumDTOVertices
            || !CheckedAdd(totalIndices, mesh.indices.size(), totalIndices)
            || totalIndices > kMaximumDTOIndices
            || !CheckedAdd(totalPrimitives,
                           mesh.primitives.size(),
                           totalPrimitives)
            || totalPrimitives > kMaximumDTOPrimitives) {
            return false;
        }

        std::size_t vertexBytes = 0;
        std::size_t indexBytes = 0;
        if (!CheckedMultiply(mesh.vertices.size(), sizeof(Vertex), vertexBytes)
            || !CheckedMultiply(mesh.indices.size(),
                                sizeof(std::uint32_t),
                                indexBytes)
            || !CheckedAdd(totalNumericBytes,
                           vertexBytes,
                           totalNumericBytes)
            || !CheckedAdd(totalNumericBytes,
                           indexBytes,
                           totalNumericBytes)
            || totalNumericBytes > kMaximumDTONumericBytes) {
            return false;
        }

        for (const Vertex& vertex : mesh.vertices) {
            if (!IsFinite(vertex.positionX) || !IsFinite(vertex.positionY)
                || !IsFinite(vertex.positionZ) || !IsFinite(vertex.normalX)
                || !IsFinite(vertex.normalY) || !IsFinite(vertex.normalZ)
                || !IsFinite(vertex.textureU) || !IsFinite(vertex.textureV)) {
                return false;
            }
        }
        for (const std::uint32_t vertexIndex : mesh.indices) {
            if (vertexIndex >= mesh.vertices.size()) {
                return false;
            }
        }

        std::unordered_set<std::uint32_t> faceIndices;
        faceIndices.reserve(mesh.primitives.size());
        std::size_t expectedFirstIndex = 0;
        for (const MeshPrimitive& primitive : mesh.primitives) {
            const std::size_t firstIndex = primitive.firstIndex;
            const std::size_t indexCount = primitive.indexCount;
            if (firstIndex != expectedFirstIndex || indexCount == 0
                || indexCount % 3 != 0
                || firstIndex > mesh.indices.size()
                || indexCount > mesh.indices.size() - firstIndex
                || !faceIndices.insert(primitive.faceIndex).second) {
                return false;
            }
            expectedFirstIndex = firstIndex + indexCount;
        }
        if (mesh.primitives.empty() || expectedFirstIndex != mesh.indices.size()) {
            return false;
        }
    }

    std::size_t totalBindings = 0;
    std::unordered_map<std::string, std::size_t> instancesByIdentifier;
    instancesByIdentifier.reserve(snapshot.instances.size());
    for (std::size_t instanceIndex = 0;
         instanceIndex < snapshot.instances.size(); ++instanceIndex) {
        const InstanceSnapshot& instance = snapshot.instances[instanceIndex];
        if (!IsValidIdentifier(instance.entityIdentifier)
            || !accountString(instance.entityIdentifier)
            || instance.name.size() > kMaximumNameBytes
            || !accountString(instance.name)
            || !instancesByIdentifier.emplace(
                instance.entityIdentifier, instanceIndex).second
            || instance.meshIndex >= snapshot.meshes.size()
            || !IsValid(instance.role)
            || !IsValid(instance.coordinateSpace)
            || !IsValid(instance.depthPolicy)
            || !IsValid(instance.renderStyle)
            || (!instance.visible && instance.selectable)
            || !CheckedAdd(totalBindings,
                           instance.primitiveBindings.size(),
                           totalBindings)
            || totalBindings > kMaximumDTOBindings) {
            return false;
        }
        for (const double value : instance.worldFromObject.values) {
            if (!IsFinite(value)) {
                return false;
            }
        }
        const MeshSnapshot& mesh = snapshot.meshes[instance.meshIndex];
        if (instance.primitiveBindings.size() != mesh.primitives.size()) {
            return false;
        }
    }

    std::unordered_set<ElementKey, ElementKeyHash> pickElements;
    pickElements.reserve(snapshot.pickTable.size());
    for (std::size_t index = 1; index < snapshot.pickTable.size(); ++index) {
        const ElementIdentifier& element = snapshot.pickTable[index];
        if (!IsValidElement(element, false)
            || !accountString(element.entityIdentifier)
            || !pickElements.insert(MakeElementKey(element)).second) {
            return false;
        }
    }

    std::vector<std::uint8_t> referencedPickTokens(
        snapshot.pickTable.size(), 0);
    for (const InstanceSnapshot& instance : snapshot.instances) {
        const MeshSnapshot& mesh = snapshot.meshes[instance.meshIndex];
        for (std::size_t primitiveIndex = 0;
             primitiveIndex < instance.primitiveBindings.size(); ++primitiveIndex) {
            const PrimitiveBinding& binding =
                instance.primitiveBindings[primitiveIndex];
            const bool shouldBePickable = instance.selectable && binding.visible;
            if (binding.materialIndex >= snapshot.materials.size()
                || binding.pickToken >= snapshot.pickTable.size()
                || (shouldBePickable && binding.pickToken == 0)
                || (!shouldBePickable && binding.pickToken != 0)) {
                return false;
            }
            if (binding.pickToken == 0) {
                continue;
            }

            const ElementIdentifier& picked =
                snapshot.pickTable[binding.pickToken];
            const MeshPrimitive& primitive = mesh.primitives[primitiveIndex];
            if (picked.entityIdentifier != instance.entityIdentifier
                || picked.geometryRevision != mesh.geometryRevision
                || (picked.kind == ElementKind::Face
                    && picked.topologyIndex != primitive.faceIndex)
                || (picked.kind == ElementKind::Object
                    && picked.topologyIndex != 0)
                || (picked.kind != ElementKind::Face
                    && picked.kind != ElementKind::Object)) {
                return false;
            }
            referencedPickTokens[binding.pickToken] = 1;
        }
    }
    for (std::size_t token = 1; token < referencedPickTokens.size(); ++token) {
        if (referencedPickTokens[token] == 0) {
            return false;
        }
    }

    const auto isPublishedElement = [&](const ElementIdentifier& element) {
        if (!IsValidElement(element, false)) {
            return false;
        }
        const auto instanceFound =
            instancesByIdentifier.find(element.entityIdentifier);
        if (instanceFound == instancesByIdentifier.end()) {
            return false;
        }
        const InstanceSnapshot& instance =
            snapshot.instances[instanceFound->second];
        const MeshSnapshot& mesh = snapshot.meshes[instance.meshIndex];
        if (element.geometryRevision != mesh.geometryRevision) {
            return false;
        }
        if (element.kind == ElementKind::Object) {
            return element.topologyIndex == 0;
        }
        return element.kind == ElementKind::Face
            && pickElements.find(MakeElementKey(element)) != pickElements.end();
    };

    std::unordered_set<ElementKey, ElementKeyHash> selectedElements;
    std::unordered_set<std::string> selectedEntities;
    selectedElements.reserve(snapshot.selection.selected.size());
    selectedEntities.reserve(snapshot.selection.selected.size());
    for (const ElementIdentifier& selected : snapshot.selection.selected) {
        if (!accountString(selected.entityIdentifier)
            || !isPublishedElement(selected)
            || !selectedElements.insert(MakeElementKey(selected)).second) {
            return false;
        }
        selectedEntities.insert(selected.entityIdentifier);
    }
    if (snapshot.selection.hovered.has_value()) {
        const ElementIdentifier& hovered = *snapshot.selection.hovered;
        if (!accountString(hovered.entityIdentifier)
            || !isPublishedElement(hovered)) {
            return false;
        }
    }
    for (const InstanceSnapshot& instance : snapshot.instances) {
        if (instance.selected
            != (selectedEntities.find(instance.entityIdentifier)
                != selectedEntities.end())) {
            return false;
        }
    }
    return true;
}

bool IsValidSceneSnapshot(const SceneSnapshot& snapshot) noexcept {
    try {
        return IsValidSceneSnapshotImpl(snapshot);
    } catch (...) {
        return false;
    }
}

bool IsRigidWorldAnchorTransform(const Matrix4d& value) noexcept {
    for (const double component : value.values) {
        if (!IsFinite(component)) {
            return false;
        }
    }
    constexpr double tolerance = 1.0e-6;
    if (std::abs(value.values[3]) > tolerance
        || std::abs(value.values[7]) > tolerance
        || std::abs(value.values[11]) > tolerance
        || std::abs(value.values[15] - 1.0) > tolerance) {
        return false;
    }
    const Double3 x = {value.values[0], value.values[1], value.values[2]};
    const Double3 y = {value.values[4], value.values[5], value.values[6]};
    const Double3 z = {value.values[8], value.values[9], value.values[10]};
    const auto dot = [](const Double3& left, const Double3& right) {
        return left.x * right.x + left.y * right.y + left.z * right.z;
    };
    const double determinant =
        x.x * (y.y * z.z - y.z * z.y)
        - y.x * (x.y * z.z - x.z * z.y)
        + z.x * (x.y * y.z - x.z * y.y);
    return std::abs(dot(x, x) - 1.0) <= tolerance
        && std::abs(dot(y, y) - 1.0) <= tolerance
        && std::abs(dot(z, z) - 1.0) <= tolerance
        && std::abs(dot(x, y)) <= tolerance
        && std::abs(dot(x, z)) <= tolerance
        && std::abs(dot(y, z)) <= tolerance
        && std::abs(determinant - 1.0) <= tolerance;
}

bool IsTranslationOnlyWorldTransform(const Matrix4d& value) noexcept {
    if (!IsRigidWorldAnchorTransform(value)) {
        return false;
    }
    constexpr double tolerance = 1.0e-6;
    const std::array<double, 12> expected = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
    };
    for (std::size_t index = 0; index < expected.size(); ++index) {
        if (std::abs(value.values[index] - expected[index])
            > tolerance) {
            return false;
        }
    }
    return true;
}

bool IsCenteredLocalBounds(const Bounds3d& value) noexcept {
    if (!value.valid || !IsValid(value)) {
        return false;
    }
    const double scale = std::max({
        1.0,
        std::abs(value.minimum.x),
        std::abs(value.minimum.y),
        std::abs(value.minimum.z),
        std::abs(value.maximum.x),
        std::abs(value.maximum.y),
        std::abs(value.maximum.z),
    });
    const double tolerance = scale
        * 32.0 * std::numeric_limits<float>::epsilon();
    return std::abs((value.minimum.x + value.maximum.x) * 0.5)
            <= tolerance
        && std::abs((value.minimum.y + value.maximum.y) * 0.5)
            <= tolerance
        && std::abs((value.minimum.z + value.maximum.z) * 0.5)
            <= tolerance;
}

bool IsValidPresentationOverlaySnapshotImpl(
    const PresentationOverlaySnapshot& snapshot) {
    if (snapshot.schemaVersion
            != kPresentationOverlaySnapshotSchemaVersion
        || !IsValidPublicationSourceIdentifier(
            snapshot.publicationSourceIdentifier)
        || snapshot.baseSnapshotRevision == 0
        || snapshot.baseDocumentGeneration == 0
        || snapshot.baseModelRevision == 0
        || snapshot.basePresentationRevision == 0
        || snapshot.overlayRevision == 0
        || snapshot.meshes.size() > kMaximumOverlayMeshes
        || snapshot.instances.size() > kMaximumOverlayInstances
        || snapshot.materials.size() > kMaximumOverlayMaterials) {
        return false;
    }

    const bool isEmpty = snapshot.meshes.empty()
        && snapshot.instances.empty() && snapshot.materials.empty();
    std::size_t stringBytes = snapshot.publicationSourceIdentifier.size();
    bool hasMirrorPlanePrefix = false;
    std::size_t mirrorPreviewCount = 0;
    std::size_t booleanActorCount = 0;
    switch (snapshot.kind) {
        case PresentationOverlayKind::None:
            return isEmpty && snapshot.suppressedEntityIdentifiers.empty();
        case PresentationOverlayKind::MoveRotateGizmo:
            if (isEmpty || snapshot.meshes.size() != 7
                || snapshot.instances.size() != 7
                || snapshot.materials.size() != 4) {
                return false;
            }
            break;
        case PresentationOverlayKind::ScaleGizmo:
            if (isEmpty || snapshot.meshes.size() != 5
                || snapshot.instances.size() != 5
                || snapshot.materials.size() != 4) {
                return false;
            }
            break;
        case PresentationOverlayKind::MirrorGizmo:
            if (isEmpty || snapshot.meshes.size() != 6
                || snapshot.instances.size() != 6
                || snapshot.materials.size() != 6) {
                return false;
            }
            hasMirrorPlanePrefix = true;
            break;
        case PresentationOverlayKind::MirrorPreview:
            if (snapshot.meshes.size() <= 6
                || snapshot.meshes.size() != snapshot.instances.size()
                || snapshot.meshes.size() != snapshot.materials.size()) {
                return false;
            }
            mirrorPreviewCount = snapshot.meshes.size() - 6;
            if (mirrorPreviewCount == 0
                || mirrorPreviewCount > kMaximumMirrorPreviewBodies) {
                return false;
            }
            hasMirrorPlanePrefix = true;
            break;
        case PresentationOverlayKind::BooleanSubtractPreview: {
            const std::size_t itemCount = snapshot.instances.size();
            if (itemCount < 2 || itemCount > kMaximumBooleanSourceOperands
                || snapshot.meshes.size() != itemCount
                || snapshot.materials.size() != itemCount
                || snapshot.suppressedEntityIdentifiers.size() != itemCount) {
                return false;
            }
            bool reachedResults = false;
            for (const InstanceSnapshot& instance : snapshot.instances) {
                if (instance.role == RenderRole::BooleanActor
                    && !reachedResults) {
                    ++booleanActorCount;
                } else if (instance.role == RenderRole::BooleanSubject) {
                    reachedResults = true;
                } else {
                    return false;
                }
            }
            if (booleanActorCount == 0 || booleanActorCount == itemCount) {
                return false;
            }
            break;
        }
        case PresentationOverlayKind::BooleanUnionPreview:
            if (snapshot.meshes.size() != 1
                || snapshot.instances.size() != 1
                || snapshot.materials.size() != 1
                || snapshot.suppressedEntityIdentifiers.size() < 2
                || snapshot.suppressedEntityIdentifiers.size()
                    > kMaximumBooleanSourceOperands) {
                return false;
            }
            break;
        default:
            return false;
    }

    const bool isBooleanPreview =
        snapshot.kind == PresentationOverlayKind::BooleanSubtractPreview
        || snapshot.kind == PresentationOverlayKind::BooleanUnionPreview;
    const auto booleanEntityIdentifier = [&](const std::size_t index) {
        if (snapshot.kind == PresentationOverlayKind::BooleanUnionPreview) {
            return std::string("boolean/union/result/0");
        }
        if (index < booleanActorCount) {
            return std::string("boolean/subtract/actor/")
                + std::to_string(index);
        }
        return std::string("boolean/subtract/result/")
            + std::to_string(index - booleanActorCount);
    };
    const auto booleanName = [&](const std::size_t index) {
        if (snapshot.kind == PresentationOverlayKind::BooleanUnionPreview) {
            return std::string("Boolean union result 0");
        }
        if (index < booleanActorCount) {
            return std::string("Boolean subtract actor ")
                + std::to_string(index);
        }
        return std::string("Boolean subtract result ")
            + std::to_string(index - booleanActorCount);
    };
    if (!isBooleanPreview
        && !snapshot.suppressedEntityIdentifiers.empty()) {
        return false;
    }
    std::unordered_set<std::string> suppressedIdentifiers;
    suppressedIdentifiers.reserve(snapshot.suppressedEntityIdentifiers.size());
    for (const std::string& identifier :
         snapshot.suppressedEntityIdentifiers) {
        if (!IsValidIdentifier(identifier)
            || !suppressedIdentifiers.insert(identifier).second
            || !CheckedAdd(stringBytes, identifier.size(), stringBytes)
            || stringBytes > kMaximumDTOStringBytes) {
            return false;
        }
    }

    std::unordered_set<std::string> materialIdentifiers;
    materialIdentifiers.reserve(snapshot.materials.size());
    for (std::size_t materialIndex = 0;
         materialIndex < snapshot.materials.size(); ++materialIndex) {
        const MaterialSnapshot& material = snapshot.materials[materialIndex];
        const auto isUnit = [](const float component) {
            return IsFinite(component)
                && component >= 0.0f && component <= 1.0f;
        };
        const bool isMirrorPlane =
            hasMirrorPlanePrefix && materialIndex < 6;
        const bool isMirrorPreview =
            snapshot.kind == PresentationOverlayKind::MirrorPreview
            && materialIndex >= 6;
        const bool isBooleanMaterial = isBooleanPreview;
        bool hasExpectedIdentifier = true;
        bool hasExpectedAlpha = true;
        bool hasExpectedColor = true;
        if (isMirrorPlane) {
            hasExpectedIdentifier = material.identifier
                == kMirrorMaterialIdentifiers[materialIndex];
            const bool expectsBlend = materialIndex >= 3;
            hasExpectedAlpha = expectsBlend
                ? material.alphaMode == AlphaMode::Blend
                    && std::abs(material.baseColor.w - 0.75f)
                        <= 1.0e-6f
                : material.alphaMode == AlphaMode::Opaque
                    && material.baseColor.w == 1.0f;
        } else if (isMirrorPreview) {
            hasExpectedIdentifier = material.identifier
                == "mirror/preview/"
                    + std::to_string(materialIndex - 6)
                    + "/material";
            hasExpectedAlpha = material.baseColor.w == 1.0f
                && (material.alphaMode == AlphaMode::Opaque
                    || material.alphaMode == AlphaMode::Mask);
        } else if (isBooleanMaterial) {
            hasExpectedIdentifier = material.identifier
                == booleanEntityIdentifier(materialIndex) + "/material";
            hasExpectedAlpha = material.alphaMode == AlphaMode::Opaque
                && material.baseColor.w == 1.0f;
            const Quantity_Color expectedColor(
                materialIndex < booleanActorCount
                    ? Quantity_NOC_ORANGE
                    : Quantity_NOC_LIGHTSKYBLUE);
            hasExpectedColor =
                std::abs(material.baseColor.x - expectedColor.Red())
                        <= 1.0e-6f
                && std::abs(material.baseColor.y - expectedColor.Green())
                        <= 1.0e-6f
                && std::abs(material.baseColor.z - expectedColor.Blue())
                        <= 1.0e-6f;
        } else {
            hasExpectedAlpha = material.alphaMode == AlphaMode::Opaque
                && material.baseColor.w == 1.0f;
        }
        if (!hasExpectedIdentifier || !hasExpectedColor
            || !IsValid(material)
            || !hasExpectedAlpha
            || !isUnit(material.baseColor.x)
            || !isUnit(material.baseColor.y)
            || !isUnit(material.baseColor.z)
            || material.emission.x < 0.0f
            || material.emission.y < 0.0f
            || material.emission.z < 0.0f
            || !materialIdentifiers.insert(material.identifier).second
            || !CheckedAdd(stringBytes,
                           material.identifier.size(),
                           stringBytes)
            || stringBytes > kMaximumDTOStringBytes) {
            return false;
        }
    }

    std::size_t totalVertices = 0;
    std::size_t totalIndices = 0;
    std::size_t totalPrimitives = 0;
    std::size_t totalNumericBytes = 0;
    std::unordered_set<std::string> definitionIdentifiers;
    definitionIdentifiers.reserve(snapshot.meshes.size());
    for (std::size_t meshIndex = 0;
         meshIndex < snapshot.meshes.size(); ++meshIndex) {
        const MeshSnapshot& mesh = snapshot.meshes[meshIndex];
        const bool isMirrorPlane = hasMirrorPlanePrefix && meshIndex < 6;
        const bool isMirrorPreview =
            snapshot.kind == PresentationOverlayKind::MirrorPreview
            && meshIndex >= 6;
        const bool isBooleanMesh = isBooleanPreview;
        const std::string expectedPreviewIdentifier = isMirrorPreview
            ? "mirror/preview/" + std::to_string(meshIndex - 6) + "/mesh"
            : std::string();
        if ((isMirrorPlane
                && mesh.definitionIdentifier
                    != kMirrorMeshIdentifiers[meshIndex])
            || (isMirrorPreview
                && mesh.definitionIdentifier
                    != expectedPreviewIdentifier)
            || (isBooleanMesh
                && mesh.definitionIdentifier
                    != booleanEntityIdentifier(meshIndex) + "/mesh")
            || !IsValidIdentifier(mesh.definitionIdentifier)
            || !definitionIdentifiers.insert(
                mesh.definitionIdentifier).second
            || mesh.geometryRevision == 0
            || !mesh.localBounds.valid || !IsValid(mesh.localBounds)
            || ((isMirrorPreview || isBooleanMesh)
                && !IsCenteredLocalBounds(mesh.localBounds))
            || mesh.vertices.empty() || mesh.indices.empty()
            || mesh.primitives.empty()
            || (!isMirrorPreview && !isBooleanMesh
                && mesh.primitives.size() != 1)
            || !CheckedAdd(totalPrimitives,
                           mesh.primitives.size(),
                           totalPrimitives)
            || totalPrimitives > kMaximumOverlayPrimitives
            || !CheckedAdd(totalVertices,
                           mesh.vertices.size(),
                           totalVertices)
            || totalVertices > kMaximumOverlayVertices
            || !CheckedAdd(totalIndices,
                           mesh.indices.size(),
                           totalIndices)
            || totalIndices > kMaximumOverlayIndices
            || !CheckedAdd(stringBytes,
                           mesh.definitionIdentifier.size(),
                           stringBytes)
            || stringBytes > kMaximumDTOStringBytes) {
            return false;
        }
        std::size_t expectedFirstIndex = 0;
        for (std::size_t primitiveIndex = 0;
             primitiveIndex < mesh.primitives.size(); ++primitiveIndex) {
            const MeshPrimitive& primitive =
                mesh.primitives[primitiveIndex];
            std::size_t indexEnd = 0;
            if (primitive.firstIndex != expectedFirstIndex
                || primitive.indexCount == 0
                || primitive.indexCount % 3 != 0
                || primitive.faceIndex != primitiveIndex
                || !CheckedAdd(
                    static_cast<std::size_t>(primitive.firstIndex),
                    static_cast<std::size_t>(primitive.indexCount),
                    indexEnd)
                || indexEnd > mesh.indices.size()) {
                return false;
            }
            expectedFirstIndex = indexEnd;
        }
        if (expectedFirstIndex != mesh.indices.size()) {
            return false;
        }
        std::size_t vertexBytes = 0;
        std::size_t indexBytes = 0;
        if (!CheckedMultiply(mesh.vertices.size(), sizeof(Vertex), vertexBytes)
            || !CheckedMultiply(mesh.indices.size(),
                                sizeof(std::uint32_t),
                                indexBytes)
            || !CheckedAdd(totalNumericBytes,
                           vertexBytes,
                           totalNumericBytes)
            || !CheckedAdd(totalNumericBytes,
                           indexBytes,
                           totalNumericBytes)
            || totalNumericBytes > kMaximumOverlayNumericBytes) {
            return false;
        }
        for (const Vertex& vertex : mesh.vertices) {
            const double normalSquared =
                static_cast<double>(vertex.normalX) * vertex.normalX
                + static_cast<double>(vertex.normalY) * vertex.normalY
                + static_cast<double>(vertex.normalZ) * vertex.normalZ;
            if (!IsFinite(vertex.positionX) || !IsFinite(vertex.positionY)
                || !IsFinite(vertex.positionZ) || !IsFinite(vertex.normalX)
                || !IsFinite(vertex.normalY) || !IsFinite(vertex.normalZ)
                || !IsFinite(vertex.textureU) || !IsFinite(vertex.textureV)
                || !IsFinite(normalSquared) || normalSquared <= 1.0e-12) {
                return false;
            }
        }
        for (const std::uint32_t index : mesh.indices) {
            if (index >= mesh.vertices.size()) {
                return false;
            }
        }
    }

    std::vector<std::uint8_t> meshReferences(snapshot.meshes.size(), 0);
    std::vector<std::uint8_t> materialReferences(snapshot.materials.size(), 0);
    std::unordered_set<std::string> entityIdentifiers;
    entityIdentifiers.reserve(snapshot.instances.size());
    std::optional<std::array<double, 16>> mirrorWorldAnchor;
    std::size_t totalBindings = 0;
    for (std::size_t instanceIndex = 0;
         instanceIndex < snapshot.instances.size(); ++instanceIndex) {
        const InstanceSnapshot& instance = snapshot.instances[instanceIndex];
        const bool isMirrorPlane =
            hasMirrorPlanePrefix && instanceIndex < 6;
        const bool isMirrorPreview =
            snapshot.kind == PresentationOverlayKind::MirrorPreview
            && instanceIndex >= 6;
        const bool isBooleanItem = isBooleanPreview;
        const std::size_t previewIndex = isMirrorPreview
            ? instanceIndex - 6
            : 0;
        const std::string expectedPreviewIdentifier = isMirrorPreview
            ? "mirror/preview/" + std::to_string(previewIndex)
            : std::string();
        const bool hasExpectedIdentity = isMirrorPlane
            ? instance.entityIdentifier
                    == kMirrorEntityIdentifiers[instanceIndex]
                && instance.name == kMirrorNames[instanceIndex]
                && instance.meshIndex == instanceIndex
            : isMirrorPreview
                ? instance.entityIdentifier == expectedPreviewIdentifier
                    && instance.name
                        == "Mirror preview " + std::to_string(previewIndex)
                    && instance.meshIndex == instanceIndex
                : isBooleanItem
                    ? instance.entityIdentifier
                            == booleanEntityIdentifier(instanceIndex)
                        && instance.name == booleanName(instanceIndex)
                        && instance.meshIndex == instanceIndex
                    : true;
        const bool hasExpectedSemantics = isBooleanItem
            ? instance.coordinateSpace == CoordinateSpace::World
                && instance.depthPolicy == DepthPolicy::Scene
                && (instanceIndex < booleanActorCount
                    ? instance.role == RenderRole::BooleanActor
                        && instance.renderStyle == RenderStyle::Wireframe
                    : instance.role == RenderRole::BooleanSubject
                        && instance.renderStyle == RenderStyle::Shaded)
            : isMirrorPreview
            ? instance.role == RenderRole::MirrorPreview
                && instance.coordinateSpace == CoordinateSpace::World
                && instance.depthPolicy == DepthPolicy::Scene
            : instance.role == RenderRole::Gizmo
                && instance.coordinateSpace
                    == CoordinateSpace::WorldAnchorPixels
                && instance.depthPolicy == DepthPolicy::Topmost;
        const std::size_t expectedBindingCount =
            (isMirrorPreview || isBooleanItem)
            ? snapshot.meshes[instanceIndex].primitives.size()
            : 1;
        if (!hasExpectedIdentity
            || !IsValidIdentifier(instance.entityIdentifier)
            || !entityIdentifiers.insert(instance.entityIdentifier).second
            || instance.meshIndex >= snapshot.meshes.size()
            || instance.reversesWinding || !instance.visible
            || instance.selectable || instance.selected
            || !hasExpectedSemantics
            || (!isBooleanItem
                && instance.renderStyle != RenderStyle::Shaded)
            || instance.name.size() > kMaximumNameBytes
            || !CheckedAdd(stringBytes,
                           instance.entityIdentifier.size(),
                           stringBytes)
            || !CheckedAdd(stringBytes,
                           instance.name.size(),
                           stringBytes)
            || stringBytes > kMaximumDTOStringBytes
            || ((isMirrorPreview || isBooleanItem)
                ? !IsTranslationOnlyWorldTransform(
                    instance.worldFromObject)
                : !IsRigidWorldAnchorTransform(
                    instance.worldFromObject))
            || instance.primitiveBindings.size()
                != expectedBindingCount
            || !CheckedAdd(totalBindings,
                           instance.primitiveBindings.size(),
                           totalBindings)
            || totalBindings > kMaximumOverlayBindings) {
            return false;
        }
        if (++meshReferences[instance.meshIndex] != 1) {
            return false;
        }
        for (const PrimitiveBinding& binding :
             instance.primitiveBindings) {
            if (((isMirrorPlane || isMirrorPreview || isBooleanItem)
                    && binding.materialIndex != instanceIndex)
                || binding.materialIndex >= snapshot.materials.size()
                || binding.pickToken != 0 || !binding.visible) {
                return false;
            }
            materialReferences[binding.materialIndex] = 1;
        }
        if (isMirrorPlane) {
            if (!mirrorWorldAnchor.has_value()) {
                mirrorWorldAnchor = instance.worldFromObject.values;
            } else if (*mirrorWorldAnchor
                       != instance.worldFromObject.values) {
                return false;
            }
        }
    }
    for (const std::string& suppressed :
         snapshot.suppressedEntityIdentifiers) {
        if (entityIdentifiers.find(suppressed)
            != entityIdentifiers.end()) {
            return false;
        }
    }
    return std::all_of(meshReferences.begin(), meshReferences.end(),
                       [](const std::uint8_t count) { return count == 1; })
        && std::all_of(materialReferences.begin(), materialReferences.end(),
                       [](const std::uint8_t count) { return count == 1; });
}

bool IsValidPresentationOverlaySnapshot(
    const PresentationOverlaySnapshot& snapshot) noexcept {
    try {
        return IsValidPresentationOverlaySnapshotImpl(snapshot);
    } catch (...) {
        return false;
    }
}

NSString *StringFromUTF8(const std::string& value) {
    if (value.empty()) {
        return @"";
    }
    NSString *string = [[NSString alloc]
        initWithBytes:value.data()
               length:value.size()
             encoding:NSUTF8StringEncoding];
    if (string != nil) {
        return string;
    }

    // Identifiers are expected to be UTF-8, but preserve malformed producer
    // bytes deterministically rather than returning a nullable public value.
    NSData *bytes = [NSData dataWithBytes:value.data() length:value.size()];
    return [[NSString alloc] initWithData:bytes encoding:NSISOLatin1StringEncoding] ?: @"";
}

simd_double3 Double3FromScene(const Double3& value) {
    return (simd_double3){value.x, value.y, value.z};
}

simd_float3 Float3FromScene(const Float3& value) {
    return (simd_float3){value.x, value.y, value.z};
}

simd_float4 Float4FromScene(const Float4& value) {
    return (simd_float4){value.x, value.y, value.z, value.w};
}

simd_uint2 UInt2FromScene(const UInt2& value) {
    return (simd_uint2){value.x, value.y};
}

simd_double4x4 MatrixFromScene(const Matrix4d& value) {
    simd_double4x4 matrix = matrix_identity_double4x4;
    for (std::size_t column = 0; column < 4; ++column) {
        for (std::size_t row = 0; row < 4; ++row) {
            matrix.columns[column][row] = value.values[column * 4 + row];
        }
    }
    return matrix;
}

Core3DSceneProjection ProjectionFromScene(Projection value) {
    switch (value) {
        case Projection::Perspective:
            return Core3DSceneProjectionPerspective;
        case Projection::Orthographic:
            return Core3DSceneProjectionOrthographic;
    }

    NSCAssert(NO, @"Unknown scene projection value: %u", static_cast<unsigned>(value));
    return Core3DSceneProjectionPerspective;
}

Core3DSceneAlphaMode AlphaModeFromScene(AlphaMode value) {
    switch (value) {
        case AlphaMode::Opaque:
            return Core3DSceneAlphaModeOpaque;
        case AlphaMode::Mask:
            return Core3DSceneAlphaModeMask;
        case AlphaMode::Blend:
            return Core3DSceneAlphaModeBlend;
    }

    NSCAssert(NO, @"Unknown scene alpha mode value: %u", static_cast<unsigned>(value));
    return Core3DSceneAlphaModeOpaque;
}

Core3DSceneRenderRole RenderRoleFromScene(RenderRole value) {
    switch (value) {
        case RenderRole::Model:
            return Core3DSceneRenderRoleModel;
        case RenderRole::SelectionHighlight:
            return Core3DSceneRenderRoleSelectionHighlight;
        case RenderRole::BooleanActor:
            return Core3DSceneRenderRoleBooleanActor;
        case RenderRole::BooleanSubject:
            return Core3DSceneRenderRoleBooleanSubject;
        case RenderRole::ChamferPreview:
            return Core3DSceneRenderRoleChamferPreview;
        case RenderRole::MirrorPreview:
            return Core3DSceneRenderRoleMirrorPreview;
        case RenderRole::Gizmo:
            return Core3DSceneRenderRoleGizmo;
        case RenderRole::Grid:
            return Core3DSceneRenderRoleGrid;
        case RenderRole::Trihedron:
            return Core3DSceneRenderRoleTrihedron;
    }

    NSCAssert(NO, @"Unknown scene render role value: %u", static_cast<unsigned>(value));
    return Core3DSceneRenderRoleModel;
}

Core3DSceneCoordinateSpace CoordinateSpaceFromScene(CoordinateSpace value) {
    switch (value) {
        case CoordinateSpace::World:
            return Core3DSceneCoordinateSpaceWorld;
        case CoordinateSpace::WorldAnchorPixels:
            return Core3DSceneCoordinateSpaceWorldAnchorPixels;
    }

    NSCAssert(NO, @"Unknown scene coordinate-space value: %u",
              static_cast<unsigned>(value));
    return Core3DSceneCoordinateSpaceWorld;
}

Core3DSceneDepthPolicy DepthPolicyFromScene(DepthPolicy value) {
    switch (value) {
        case DepthPolicy::Scene:
            return Core3DSceneDepthPolicyScene;
        case DepthPolicy::Topmost:
            return Core3DSceneDepthPolicyTopmost;
    }

    NSCAssert(NO, @"Unknown scene depth-policy value: %u",
              static_cast<unsigned>(value));
    return Core3DSceneDepthPolicyScene;
}

Core3DSceneRenderStyle RenderStyleFromScene(RenderStyle value) {
    switch (value) {
        case RenderStyle::Shaded:
            return Core3DSceneRenderStyleShaded;
        case RenderStyle::Wireframe:
            return Core3DSceneRenderStyleWireframe;
    }

    NSCAssert(NO, @"Unknown scene render-style value: %u",
              static_cast<unsigned>(value));
    return Core3DSceneRenderStyleShaded;
}

Core3DScenePresentationOverlayKind PresentationOverlayKindFromScene(
    PresentationOverlayKind value) {
    switch (value) {
        case PresentationOverlayKind::None:
            return Core3DScenePresentationOverlayKindNone;
        case PresentationOverlayKind::MoveRotateGizmo:
            return Core3DScenePresentationOverlayKindMoveRotateGizmo;
        case PresentationOverlayKind::ScaleGizmo:
            return Core3DScenePresentationOverlayKindScaleGizmo;
        case PresentationOverlayKind::MirrorGizmo:
            return Core3DScenePresentationOverlayKindMirrorGizmo;
        case PresentationOverlayKind::MirrorPreview:
            return Core3DScenePresentationOverlayKindMirrorPreview;
        case PresentationOverlayKind::BooleanSubtractPreview:
            return Core3DScenePresentationOverlayKindBooleanSubtractPreview;
        case PresentationOverlayKind::BooleanUnionPreview:
            return Core3DScenePresentationOverlayKindBooleanUnionPreview;
    }

    NSCAssert(NO, @"Unknown presentation-overlay kind: %u",
              static_cast<unsigned>(value));
    return Core3DScenePresentationOverlayKindNone;
}

Core3DSceneElementKind ElementKindFromScene(ElementKind value) {
    switch (value) {
        case ElementKind::None:
            return Core3DSceneElementKindNone;
        case ElementKind::Object:
            return Core3DSceneElementKindObject;
        case ElementKind::Face:
            return Core3DSceneElementKindFace;
        case ElementKind::Edge:
            return Core3DSceneElementKindEdge;
        case ElementKind::Vertex:
            return Core3DSceneElementKindVertex;
    }

    NSCAssert(NO, @"Unknown scene element kind value: %u", static_cast<unsigned>(value));
    return Core3DSceneElementKindNone;
}

Core3DSceneRevisionVector *RevisionVectorFromScene(const RevisionVector& value) {
    return [[Core3DSceneRevisionVector alloc]
        initWithSnapshotRevision:value.snapshot
              documentGeneration:value.documentGeneration
                   modelRevision:value.model
            presentationRevision:value.presentation
                  cameraRevision:value.camera];
}

Core3DSceneBounds *BoundsFromScene(const Bounds3d& value) {
    return [[Core3DSceneBounds alloc]
        initWithMinimum:Double3FromScene(value.minimum)
                maximum:Double3FromScene(value.maximum)
                  valid:value.valid];
}

Core3DSceneCameraSnapshot *CameraFromScene(const CameraSnapshot& value) {
    return [[Core3DSceneCameraSnapshot alloc]
        initWithEye:Double3FromScene(value.eye)
             center:Double3FromScene(value.center)
                 up:Double3FromScene(value.up)
         projection:ProjectionFromScene(value.projection)
 verticalFieldOfViewRadians:value.verticalFovRadians
         orthographicHeight:value.orthographicHeight
                  nearPlane:value.nearPlane
                   farPlane:value.farPlane
                aspectRatio:value.aspect
         viewportSizePixels:UInt2FromScene(value.viewportPixels)];
}

Core3DSceneMaterialSnapshot *MaterialFromScene(const MaterialSnapshot& value) {
    return [[Core3DSceneMaterialSnapshot alloc]
        initWithIdentifier:StringFromUTF8(value.identifier)
        linearBaseColorRGBA:Float4FromScene(value.baseColor)
          linearEmissionRGB:Float3FromScene(value.emission)
                   metallic:value.metallic
                  roughness:value.roughness
          indexOfRefraction:value.indexOfRefraction
                  alphaMode:AlphaModeFromScene(value.alphaMode)
                alphaCutoff:value.alphaCutoff
                doubleSided:value.doubleSided];
}

Core3DSceneFacePrimitiveSnapshot *FacePrimitiveFromScene(const MeshPrimitive& value) {
    return [[Core3DSceneFacePrimitiveSnapshot alloc]
        initWithFirstIndex:value.firstIndex
                indexCount:value.indexCount
                 faceIndex:value.faceIndex];
}

Core3DScenePrimitiveBindingSnapshot *PrimitiveBindingFromScene(
    const PrimitiveBinding& value) {
    return [[Core3DScenePrimitiveBindingSnapshot alloc]
        initWithMaterialIndex:value.materialIndex
                   pickToken:value.pickToken
                     visible:value.visible];
}

Core3DSceneElementIdentifier *ElementIdentifierFromScene(
    const ElementIdentifier& value) {
    return [[Core3DSceneElementIdentifier alloc]
        initWithEntityIdentifier:StringFromUTF8(value.entityIdentifier)
                            kind:ElementKindFromScene(value.kind)
                   topologyIndex:value.topologyIndex
                geometryRevision:value.geometryRevision];
}

template <typename Input, typename Output, typename Transform>
NSArray<Output *> *ObjectArrayFromVector(
    const std::vector<Input>& values,
    Transform transform) {
    NSMutableArray<Output *> *objects =
        [[NSMutableArray alloc] initWithCapacity:values.size()];
    for (const Input& value : values) {
        [objects addObject:transform(value)];
    }
    return [objects copy];
}

Core3DSceneMeshSnapshot *MeshFromScene(const MeshSnapshot& value) {
    NSData *vertexData = value.vertices.empty()
        ? NSData.data
        : [NSData dataWithBytes:value.vertices.data()
                         length:value.vertices.size() * sizeof(Vertex)];
    NSData *indexData = value.indices.empty()
        ? NSData.data
        : [NSData dataWithBytes:value.indices.data()
                         length:value.indices.size() * sizeof(std::uint32_t)];
    NSArray<Core3DSceneFacePrimitiveSnapshot *> *primitives =
        ObjectArrayFromVector<MeshPrimitive, Core3DSceneFacePrimitiveSnapshot>(
            value.primitives,
            FacePrimitiveFromScene);

    return [[Core3DSceneMeshSnapshot alloc]
        initWithDefinitionIdentifier:StringFromUTF8(value.definitionIdentifier)
                    geometryRevision:value.geometryRevision
                         localBounds:BoundsFromScene(value.localBounds)
                          vertexData:vertexData
                           indexData:indexData
                         vertexCount:value.vertices.size()
                          indexCount:value.indices.size()
                      facePrimitives:primitives];
}

Core3DSceneRenderItemSnapshot *RenderItemFromScene(const InstanceSnapshot& value) {
    NSArray<Core3DScenePrimitiveBindingSnapshot *> *bindings =
        ObjectArrayFromVector<PrimitiveBinding, Core3DScenePrimitiveBindingSnapshot>(
            value.primitiveBindings,
            PrimitiveBindingFromScene);
    return [[Core3DSceneRenderItemSnapshot alloc]
        initWithEntityIdentifier:StringFromUTF8(value.entityIdentifier)
                       meshIndex:value.meshIndex
                  worldTransform:MatrixFromScene(value.worldFromObject)
                         winding:value.reversesWinding
                             ? Core3DSceneWindingReversed
                             : Core3DSceneWindingAsDefined
                         visible:value.visible
                      selectable:value.selectable
                        selected:value.selected
                            name:StringFromUTF8(value.name)
                      renderRole:RenderRoleFromScene(value.role)
                 coordinateSpace:CoordinateSpaceFromScene(value.coordinateSpace)
                     depthPolicy:DepthPolicyFromScene(value.depthPolicy)
                     renderStyle:RenderStyleFromScene(value.renderStyle)
               primitiveBindings:bindings];
}

Core3DSceneSelectionSnapshot *SelectionFromScene(const SelectionSnapshot& value) {
    NSArray<Core3DSceneElementIdentifier *> *selected =
        ObjectArrayFromVector<ElementIdentifier, Core3DSceneElementIdentifier>(
            value.selected,
            ElementIdentifierFromScene);
    Core3DSceneElementIdentifier *hovered = value.hovered.has_value()
        ? ElementIdentifierFromScene(*value.hovered)
        : nil;
    return [[Core3DSceneSelectionSnapshot alloc]
        initWithSelectedElements:selected
                  hoveredElement:hovered];
}

} // namespace


Core3DSceneSnapshot *Core3DCreateSceneSnapshotDTO(
    const SceneSnapshot& snapshot) noexcept {
    if (!IsValidSceneSnapshot(snapshot)) {
        return nil;
    }

    try {
        NSArray<Core3DSceneMeshSnapshot *> *meshes =
            ObjectArrayFromVector<MeshSnapshot, Core3DSceneMeshSnapshot>(
                snapshot.meshes,
                MeshFromScene);
        NSArray<Core3DSceneRenderItemSnapshot *> *renderItems =
            ObjectArrayFromVector<InstanceSnapshot, Core3DSceneRenderItemSnapshot>(
                snapshot.instances,
                RenderItemFromScene);
        NSArray<Core3DSceneMaterialSnapshot *> *materials =
            ObjectArrayFromVector<MaterialSnapshot, Core3DSceneMaterialSnapshot>(
                snapshot.materials,
                MaterialFromScene);
        NSArray<Core3DSceneElementIdentifier *> *pickTable =
            ObjectArrayFromVector<ElementIdentifier, Core3DSceneElementIdentifier>(
                snapshot.pickTable,
                ElementIdentifierFromScene);

        return [[Core3DSceneSnapshot alloc]
            initWithSchemaVersion:snapshot.schemaVersion
            publicationSourceIdentifier:StringFromUTF8(
                snapshot.publicationSourceIdentifier)
                        revisions:RevisionVectorFromScene(snapshot.revisions)
                     renderOrigin:Double3FromScene(snapshot.renderOrigin)
                           meshes:meshes
                      renderItems:renderItems
                        materials:materials
                        pickTable:pickTable
                           camera:CameraFromScene(snapshot.camera)
                        selection:SelectionFromScene(snapshot.selection)];
    } catch (...) {
        return nil;
    }
}

Core3DSceneFrameSnapshot *Core3DCreateSceneFrameSnapshotDTO(
    const FrameSnapshot& snapshot) noexcept {
    if (!IsValidPublicationSourceIdentifier(
            snapshot.publicationSourceIdentifier)
        || snapshot.revisions.snapshot == 0
        || snapshot.revisions.documentGeneration == 0
        || snapshot.revisions.model == 0
        || snapshot.revisions.presentation == 0
        || snapshot.revisions.camera == 0
        || !IsValid(snapshot.camera)) {
        return nil;
    }

    try {
        return [[Core3DSceneFrameSnapshot alloc]
            initWithPublicationSourceIdentifier:StringFromUTF8(
                snapshot.publicationSourceIdentifier)
                                           revisions:RevisionVectorFromScene(
                                               snapshot.revisions)
                                              camera:CameraFromScene(
                                                  snapshot.camera)];
    } catch (...) {
        return nil;
    }
}

Core3DScenePresentationOverlaySnapshot *
Core3DCreateScenePresentationOverlaySnapshotDTO(
    const PresentationOverlaySnapshot& snapshot) noexcept {
    if (!IsValidPresentationOverlaySnapshot(snapshot)) {
        return nil;
    }

    try {
        NSArray<Core3DSceneMeshSnapshot *> *meshes =
            ObjectArrayFromVector<MeshSnapshot, Core3DSceneMeshSnapshot>(
                snapshot.meshes,
                MeshFromScene);
        NSArray<Core3DSceneRenderItemSnapshot *> *renderItems =
            ObjectArrayFromVector<InstanceSnapshot,
                                  Core3DSceneRenderItemSnapshot>(
                snapshot.instances,
                RenderItemFromScene);
        NSArray<Core3DSceneMaterialSnapshot *> *materials =
            ObjectArrayFromVector<MaterialSnapshot,
                                  Core3DSceneMaterialSnapshot>(
                snapshot.materials,
                MaterialFromScene);
        NSMutableArray<NSString *> *suppressedEntityIdentifiers =
            [NSMutableArray arrayWithCapacity:
                snapshot.suppressedEntityIdentifiers.size()];
        for (const std::string& identifier :
             snapshot.suppressedEntityIdentifiers) {
            [suppressedEntityIdentifiers addObject:StringFromUTF8(identifier)];
        }
        return [[Core3DScenePresentationOverlaySnapshot alloc]
            initWithSchemaVersion:snapshot.schemaVersion
            kind:PresentationOverlayKindFromScene(snapshot.kind)
            publicationSourceIdentifier:StringFromUTF8(
                snapshot.publicationSourceIdentifier)
            baseSnapshotRevision:snapshot.baseSnapshotRevision
            baseDocumentGeneration:snapshot.baseDocumentGeneration
            baseModelRevision:snapshot.baseModelRevision
            basePresentationRevision:snapshot.basePresentationRevision
            overlayRevision:snapshot.overlayRevision
            meshes:meshes
            renderItems:renderItems
            materials:materials
            suppressedEntityIdentifiers:suppressedEntityIdentifiers];
    } catch (...) {
        return nil;
    }
}
