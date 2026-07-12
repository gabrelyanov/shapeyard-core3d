//
//  SceneSnapshot.hpp
//  Core3D
//
//  Immutable, renderer-neutral scene values. Keep this header free of OCCT,
//  UIKit, Foundation, and graphics API types so published snapshots can safely
//  cross from the model owner to any renderer.
//

#ifndef Core3D_SceneSnapshot_hpp
#define Core3D_SceneSnapshot_hpp

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace core3d::scene {

inline constexpr std::uint32_t kSceneSnapshotSchemaVersion = 3;
inline constexpr std::uint32_t kPresentationOverlaySnapshotSchemaVersion = 5;

struct Float2 {
    float x = 0.0f;
    float y = 0.0f;
};

struct Float3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Float4 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float w = 0.0f;
};

struct Double3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct UInt2 {
    std::uint32_t x = 0;
    std::uint32_t y = 0;
};

//! Column-major matrix applied to a column vector: world = M * object.
struct Matrix4d {
    std::array<double, 16> values = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
    };
};

struct Bounds3d {
    Double3 minimum;
    Double3 maximum;
    bool valid = false;
};

enum class Projection : std::uint8_t {
    Perspective = 0,
    Orthographic = 1,
};

enum class AlphaMode : std::uint8_t {
    Opaque = 0,
    Mask = 1,
    Blend = 2,
};

enum class CullMode : std::uint8_t {
    None = 0,
    Back = 1,
    Front = 2,
};

enum class TextureEncoding : std::uint8_t {
    PNG = 0,
    JPEG,
    GIF,
    TIFF,
    BMP,
    WebP,
};

enum class RenderRole : std::uint8_t {
    Model = 0,
    SelectionHighlight,
    BooleanActor,
    BooleanSubject,
    ChamferPreview,
    MirrorPreview,
    Gizmo,
    Grid,
    Trihedron,
};

enum class CoordinateSpace : std::uint8_t {
    World = 0,
    WorldAnchorPixels,
};

enum class DepthPolicy : std::uint8_t {
    Scene = 0,
    Topmost,
};

enum class RenderStyle : std::uint8_t {
    Shaded = 0,
    Wireframe,
};

enum class PresentationOverlayKind : std::uint8_t {
    None = 0,
    MoveRotateGizmo,
    ScaleGizmo,
    MirrorGizmo,
    MirrorPreview,
    BooleanSubtractPreview,
    BooleanUnionPreview,
};

enum class ElementKind : std::uint8_t {
    None = 0,
    Object,
    Face,
    Edge,
    Vertex,
};

struct ElementIdentifier {
    std::string entityIdentifier;
    ElementKind kind = ElementKind::None;
    std::uint32_t topologyIndex = 0;
    std::uint64_t geometryRevision = 0;
};

struct RevisionVector {
    std::uint64_t snapshot = 0;
    std::uint64_t documentGeneration = 0;
    std::uint64_t model = 0;
    std::uint64_t presentation = 0;
    std::uint64_t camera = 0;
};

//! Stable interleaved vertex layout copied into public immutable NSData.
struct Vertex {
    float positionX = 0.0f;
    float positionY = 0.0f;
    float positionZ = 0.0f;
    float normalX = 0.0f;
    float normalY = 0.0f;
    float normalZ = 1.0f;
    float textureU = 0.0f;
    float textureV = 0.0f;
};

static_assert(sizeof(Vertex) == sizeof(float) * 8,
              "Scene vertex layout must remain an explicit eight-float ABI");

struct MaterialSnapshot {
    std::string identifier;
    Float4 baseColor = {0.6f, 0.6f, 0.6f, 1.0f};
    Float3 emission;
    float metallic = 0.0f;
    float roughness = 0.5f;
    float indexOfRefraction = 1.5f;
    AlphaMode alphaMode = AlphaMode::Opaque;
    float alphaCutoff = 0.5f;
    CullMode cullMode = CullMode::None;
    //! Index into SceneSnapshot::textures, or -1 when the material is scalar.
    std::int32_t baseColorTextureIndex = -1;
};

//! Immutable app-owned encoded raster copied from an embedded XCAF texture.
//! Renderers decode this single-frame payload within the published dimensions
//! and aggregate budgets; no document-owned pointer or external path escapes.
struct TextureResourceSnapshot {
    std::string identifier;
    TextureEncoding encoding = TextureEncoding::PNG;
    std::uint32_t pixelWidth = 0;
    std::uint32_t pixelHeight = 0;
    std::vector<std::uint8_t> encodedBytes;
};

struct MeshPrimitive {
    std::uint32_t firstIndex = 0;
    std::uint32_t indexCount = 0;
    std::uint32_t faceIndex = 0;
    //! True only when every vertex in this primitive owns authored/generated UVs.
    //! Zero-filled fallback coordinates are not texture coordinates.
    bool hasTextureCoordinates = false;
};

struct PrimitiveBinding {
    std::uint32_t materialIndex = 0;
    std::uint32_t pickToken = 0;
    bool visible = true;
};

struct MeshSnapshot {
    std::string definitionIdentifier;
    std::uint64_t geometryRevision = 0;
    Bounds3d localBounds;
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;
    std::vector<MeshPrimitive> primitives;
};

struct InstanceSnapshot {
    std::string entityIdentifier;
    std::uint32_t meshIndex = 0;
    Matrix4d worldFromObject;
    bool reversesWinding = false;
    bool visible = true;
    bool selectable = true;
    bool selected = false;
    std::string name;
    RenderRole role = RenderRole::Model;
    CoordinateSpace coordinateSpace = CoordinateSpace::World;
    DepthPolicy depthPolicy = DepthPolicy::Scene;
    RenderStyle renderStyle = RenderStyle::Shaded;
    std::vector<PrimitiveBinding> primitiveBindings;
};

struct CameraSnapshot {
    Projection projection = Projection::Perspective;
    Double3 eye;
    Double3 center;
    Double3 up = {0.0, 1.0, 0.0};
    //! True vertical projection values, independent of viewport aspect.
    double verticalFovRadians = 0.0;
    double orthographicHeight = 0.0;
    double nearPlane = 0.0;
    double farPlane = 0.0;
    double aspect = 1.0;
    UInt2 viewportPixels;
};

struct SelectionSnapshot {
    std::vector<ElementIdentifier> selected;
    std::optional<ElementIdentifier> hovered;
};

struct SceneSnapshot {
    std::uint32_t schemaVersion = kSceneSnapshotSchemaVersion;
    //! Stable identity of the builder publication stream. This deliberately
    //! sits outside RevisionVector because it is an identity, not a revision.
    std::string publicationSourceIdentifier;
    RevisionVector revisions;
    //! Real-world length of one scene coordinate unit. Shapeyard-authored
    //! documents use millimetres, so their value is 0.001.
    double metersPerUnit = 0.001;
    Double3 renderOrigin;
    std::vector<MeshSnapshot> meshes;
    std::vector<InstanceSnapshot> instances;
    std::vector<MaterialSnapshot> materials;
    std::vector<TextureResourceSnapshot> textures;
    //! Index zero is always reserved for "no hit".
    std::vector<ElementIdentifier> pickTable = {ElementIdentifier()};
    CameraSnapshot camera;
    SelectionSnapshot selection;
};

//! Lightweight publication used between full scene snapshots. It intentionally
//! carries no geometry so interactive camera motion and drawable-size changes
//! never retraverse or remesh the mutable CAD document.
struct FrameSnapshot {
    std::string publicationSourceIdentifier;
    RevisionVector revisions;
    CameraSnapshot camera;
};

//! Renderer-neutral transient presentation values produced without traversing
//! or mutating the committed OCAF document.
struct PresentationOverlayContent {
    PresentationOverlayKind kind = PresentationOverlayKind::None;
    std::vector<MeshSnapshot> meshes;
    std::vector<InstanceSnapshot> instances;
    std::vector<MaterialSnapshot> materials;
    //! Committed scene occurrences replaced by this transient presentation.
    //! Renderers omit these base instances from both color and pick passes.
    std::vector<std::string> suppressedEntityIdentifiers;
};

//! Immutable transient presentation paired with one exact full scene. Overlay
//! publication has its own revision domain and never changes modelRevision.
struct PresentationOverlaySnapshot {
    std::uint32_t schemaVersion = kPresentationOverlaySnapshotSchemaVersion;
    PresentationOverlayKind kind = PresentationOverlayKind::None;
    std::string publicationSourceIdentifier;
    std::uint64_t baseSnapshotRevision = 0;
    std::uint64_t baseDocumentGeneration = 0;
    std::uint64_t baseModelRevision = 0;
    std::uint64_t basePresentationRevision = 0;
    std::uint64_t overlayRevision = 0;
    std::vector<MeshSnapshot> meshes;
    std::vector<InstanceSnapshot> instances;
    std::vector<MaterialSnapshot> materials;
    std::vector<std::string> suppressedEntityIdentifiers;
};

} // namespace core3d::scene

#endif // Core3D_SceneSnapshot_hpp
