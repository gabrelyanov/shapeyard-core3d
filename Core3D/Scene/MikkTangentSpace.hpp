#ifndef Core3D_MikkTangentSpace_hpp
#define Core3D_MikkTangentSpace_hpp

#include "SceneSnapshot.hpp"

namespace core3d::scene {

// Initial normal-map authoring envelope matches the bounded mesh UV workflow.
inline constexpr std::size_t kMaximumTangentTriangles = 4096;
inline constexpr std::size_t kMaximumTangentVertices = 24576;

enum class TangentSpaceError {
    None, MissingUVs, InvalidLayout, ResourceLimit, InvalidIndex,
    NonFiniteAttribute, InvalidNormal, DegenerateSurface, DegenerateUV,
    GenerationFailed, InvalidFrame
};

// Returns one float4 (unit tangent xyz, bitangent sign w) per INDEX CORNER.
// Never average these back onto the input vertex indices. A private renderer/
// export mesh must split corners wherever their tangent frames differ.
// Object-space input only; transform tangents with the linear model matrix,
// normals with its inverse transpose, and handedness with determinant sign.
// No input mutation; failure clears output, including allocation failure.
TangentSpaceError GenerateMikkCornerTangents(
    const std::vector<Vertex>& vertices,
    const std::vector<std::uint32_t>& triangleIndices,
    bool hasTextureCoordinates,
    std::vector<Float4>& cornerTangents) noexcept;

} // namespace core3d::scene
#endif
