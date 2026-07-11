//
//  Core3DSceneSnapshot.mm
//  Core3D
//


#import "Core3DSceneSnapshot.h"
#import "Core3DSceneSnapshotFactory.hpp"

#include "../Scene/SceneSnapshot.hpp"

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

@interface Core3DSceneSnapshot ()
- (instancetype)initWithSchemaVersion:(uint32_t)schemaVersion
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


@implementation Core3DSceneSnapshot

- (instancetype)initWithSchemaVersion:(uint32_t)schemaVersion
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
