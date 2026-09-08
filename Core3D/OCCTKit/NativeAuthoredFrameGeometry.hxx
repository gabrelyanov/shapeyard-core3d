#pragma once
// Private geometry association foundation; not used by document admission.
#include "../Scene/AuthoredTangentArchive.hpp"
#include "../Common/Core3DMobileResourceLimits.h"
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopoDS_Face.hxx>
#include <TopLoc_Location.hxx>

namespace core3d::persistence {
using scene::authored::GeometryIdentity;

// Hashes committed local geometry, including unused nodes. Version 1 admits
// only a forward mesh face with baked identity location in millimetres.
// Rendering origin, object transform, material and label identity are excluded.
inline bool NativeAuthoredGeometryIdentity(const TopoDS_Face& face,
                                           GeometryIdentity& identity) noexcept {
    identity.fill(0);
    try {
        if (face.IsNull() || face.Orientation() != TopAbs_FORWARD
            || !face.Location().IsIdentity() || !BRep_Tool::Surface(face).IsNull()) return false;
        TopLoc_Location location;
        const auto& mesh = BRep_Tool::Triangulation(face, location);
        if (!location.IsIdentity() || mesh.IsNull() || mesh->HasDeferredData()
            || !mesh->HasGeometry() || !mesh->HasNormals() || !mesh->HasUVNodes()
            || mesh->NbNodes() <= 0 || mesh->NbNodes() > int(scene::kMaximumTangentVertices)
            || mesh->NbTriangles() <= 0 || mesh->NbTriangles() > int(scene::kMaximumTangentTriangles)) return false;
        CC_SHA256_CTX context; CC_SHA256_Init(&context);
        auto feed32 = [&](std::uint32_t value) {
            std::uint8_t bytes[4]; scene::authored::Write32(bytes, value);
            CC_SHA256_Update(&context, bytes, sizeof(bytes));
        };
        auto feed64 = [&](double value) {
            static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
            std::uint64_t bits; std::memcpy(&bits, &value, sizeof(bits));
            std::uint8_t bytes[8];
            for (unsigned i = 0; i < 8; ++i) bytes[i] = std::uint8_t(bits >> (8 * i));
            CC_SHA256_Update(&context, bytes, sizeof(bytes));
        };
        auto feedFloat = [&](float value) {
            std::uint8_t bytes[4]; scene::authored::WriteFloat(bytes, value);
            CC_SHA256_Update(&context, bytes, sizeof(bytes));
        };
        // Explicit stream: SYTG, version 1, mm/local/forward policy 1,
        // required normal+UV flags 3, node count, triangle count.
        feed32(0x47545953); feed32(1); feed32(1); feed32(3);
        feed32(std::uint32_t(mesh->NbNodes())); feed32(std::uint32_t(mesh->NbTriangles()));
        for (int n = 1; n <= mesh->NbNodes(); ++n) {
            const auto point = mesh->Node(n); const auto uv = mesh->UVNode(n);
            gp_Vec3f normal; mesh->Normal(n, normal);
            for (double value : {point.X(), point.Y(), point.Z()})
                if (!std::isfinite(value) || std::abs(value) > limits::kMaximumModelCoordinateMagnitude) return false;
            for (double value : {uv.X(), uv.Y()})
                if (!std::isfinite(value) || std::abs(value) > double(std::numeric_limits<float>::max())) return false;
            const double lengthSquared = double(normal.x()) * normal.x()
                + double(normal.y()) * normal.y() + double(normal.z()) * normal.z();
            if (!std::isfinite(lengthSquared) || std::abs(lengthSquared - 1.0) > 1.e-3) return false;
            feed64(point.X()); feed64(point.Y()); feed64(point.Z());
            feedFloat(normal.x()); feedFloat(normal.y()); feedFloat(normal.z());
            feed64(uv.X()); feed64(uv.Y());
        }
        for (int t = 1; t <= mesh->NbTriangles(); ++t) {
            int nodes[3]; mesh->Triangle(t).Get(nodes[0], nodes[1], nodes[2]);
            for (int node : nodes) if (node < 1 || node > mesh->NbNodes()) return false;
            if (nodes[0] == nodes[1] || nodes[1] == nodes[2] || nodes[2] == nodes[0]) return false;
            for (int node : nodes) feed32(std::uint32_t(node));
        }
        GeometryIdentity result; CC_SHA256_Final(result.data(), &context);
        identity = result; return true;
    } catch (...) { identity.fill(0); return false; }
}
} // namespace core3d::persistence
