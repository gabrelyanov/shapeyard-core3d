#include "MikkTangentSpace.hpp"
#include "../Common/Core3DMobileResourceLimits.h"
#include "../ThirdParty/MikkTSpace/mikktspace.h"
#include <cmath>
#include <limits>

namespace core3d::scene {
namespace {
struct Input {
    const std::vector<Vertex>& vertices;
    const std::vector<std::uint32_t>& indices;
    std::vector<Float4>& frames;
    const Vertex& vertex(int face, int corner) const {
        return vertices[indices[std::size_t(face) * 3 + corner]];
    }
};
Input& Get(const SMikkTSpaceContext* context) {
    return *static_cast<Input*>(context->m_pUserData);
}
double NormalLength(const Vertex& v) {
    return std::hypot(double(v.normalX), double(v.normalY), double(v.normalZ));
}
TangentSpaceError Generate(const std::vector<Vertex>& vertices,
                          const std::vector<std::uint32_t>& indices,
                          bool hasUV, std::vector<Float4>& frames) {
    if (!hasUV) return TangentSpaceError::MissingUVs;
    if (vertices.empty() || indices.empty() || indices.size() % 3 != 0)
        return TangentSpaceError::InvalidLayout;
    if (vertices.size() > kMaximumTangentVertices
        || indices.size() / 3 > kMaximumTangentTriangles)
        return TangentSpaceError::ResourceLimit;
    // Validate only referenced vertices. An unused source prefix is preserved
    // by UV authoring and must not accidentally acquire rendering authority.
    for (const auto index : indices) {
        if (index >= vertices.size()) return TangentSpaceError::InvalidIndex;
        const auto& v = vertices[index];
        const float values[] = {v.positionX, v.positionY, v.positionZ,
            v.normalX, v.normalY, v.normalZ, v.textureU, v.textureV};
        for (float value : values)
            if (!std::isfinite(value)) return TangentSpaceError::NonFiniteAttribute;
        // Bound float arithmetic before entering upstream's float generator.
        if (std::abs(v.positionX) > limits::kMaximumModelCoordinateMagnitude
            || std::abs(v.positionY) > limits::kMaximumModelCoordinateMagnitude
            || std::abs(v.positionZ) > limits::kMaximumModelCoordinateMagnitude
            || std::abs(v.textureU) > 1.0e6 || std::abs(v.textureV) > 1.0e6)
            return TangentSpaceError::ResourceLimit;
        if (NormalLength(v) < 1.0e-12) return TangentSpaceError::InvalidNormal;
    }
    for (std::size_t i = 0; i < indices.size(); i += 3) {
        const auto& a = vertices[indices[i]];
        const auto& b = vertices[indices[i + 1]];
        const auto& c = vertices[indices[i + 2]];
        const double x1 = double(b.positionX) - a.positionX;
        const double y1 = double(b.positionY) - a.positionY;
        const double z1 = double(b.positionZ) - a.positionZ;
        const double x2 = double(c.positionX) - a.positionX;
        const double y2 = double(c.positionY) - a.positionY;
        const double z2 = double(c.positionZ) - a.positionZ;
        // Explicitly reject isolated/degenerate faces rather than silently
        // publishing upstream's fallback frame as normal-map qualification.
        if (std::hypot(y1*z2-z1*y2, z1*x2-x1*z2, x1*y2-y1*x2) < 1.0e-12)
            return TangentSpaceError::DegenerateSurface;
        const double determinant = (double(b.textureU)-a.textureU)*(double(c.textureV)-a.textureV)
            - (double(b.textureV)-a.textureV)*(double(c.textureU)-a.textureU);
        if (std::abs(determinant) < 1.0e-12)
            return TangentSpaceError::DegenerateUV;
    }
    frames.assign(indices.size(), {0, 0, 0, 0});
    Input input{vertices, indices, frames};
    SMikkTSpaceInterface api{};
    api.m_getNumFaces = [](const SMikkTSpaceContext* c) { return int(Get(c).indices.size() / 3); };
    api.m_getNumVerticesOfFace = [](const SMikkTSpaceContext*, int) { return 3; };
    api.m_getPosition = [](const SMikkTSpaceContext* c, float out[], int f, int k) {
        const auto& v = Get(c).vertex(f, k);
        out[0] = v.positionX; out[1] = v.positionY; out[2] = v.positionZ;
    };
    api.m_getNormal = [](const SMikkTSpaceContext* c, float out[], int f, int k) {
        const auto& v = Get(c).vertex(f, k);
        const double length = NormalLength(v);
        out[0] = float(v.normalX/length); out[1] = float(v.normalY/length); out[2] = float(v.normalZ/length);
    };
    api.m_getTexCoord = [](const SMikkTSpaceContext* c, float out[], int f, int k) {
        const auto& v = Get(c).vertex(f, k);
        out[0] = v.textureU; out[1] = v.textureV;
    };
    api.m_setTSpaceBasic = [](const SMikkTSpaceContext* c, const float t[], float sign, int f, int k) {
        Get(c).frames[std::size_t(f)*3+k] = {t[0], t[1], t[2], sign};
    };
    SMikkTSpaceContext context{&api, &input};
    if (!genTangSpaceDefault(&context)) return TangentSpaceError::GenerationFailed;
    for (std::size_t i = 0; i < frames.size(); ++i) {
        const auto& t = frames[i];
        const auto& v = vertices[indices[i]];
        const double length = std::hypot(double(t.x), double(t.y), double(t.z));
        const double dot = (double(t.x)*v.normalX + double(t.y)*v.normalY + double(t.z)*v.normalZ) / NormalLength(v);
        if (!std::isfinite(length) || !std::isfinite(dot)
            || std::abs(length - 1) > 1.0e-4 || std::abs(dot) > 1.0e-4
            || (t.w != 1 && t.w != -1)) return TangentSpaceError::InvalidFrame;
    }
    return TangentSpaceError::None;
}
} // namespace

TangentSpaceError GenerateMikkCornerTangents(const std::vector<Vertex>& vertices,
    const std::vector<std::uint32_t>& indices, bool hasUV,
    std::vector<Float4>& output) noexcept {
    output.clear();
    try {
        std::vector<Float4> frames;
        const auto result = Generate(vertices, indices, hasUV, frames);
        if (result == TangentSpaceError::None) output.swap(frames);
        return result;
    } catch (...) { return TangentSpaceError::GenerationFailed; }
}
} // namespace core3d::scene
