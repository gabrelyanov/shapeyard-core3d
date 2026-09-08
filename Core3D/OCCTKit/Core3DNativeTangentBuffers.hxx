#pragma once
#include <OpenGLES/ES2/gl.h>
#include "../Scene/MikkTangentSpace.hpp"
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <OpenGl_Group.hxx>
#include <OpenGl_PrimitiveArray.hxx>
#include <OpenGl_Context.hxx>
#include <Graphic3d_ShaderProgram.hxx>
#include <PrsMgr_Presentation.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <cstring>
#include <unordered_map>

namespace core3d::render {

inline bool NeedsNativeTangents(const Handle(Graphic3d_Aspects)& aspect) {
    return !aspect.IsNull() && !aspect->ShaderProgram().IsNull()
        && aspect->ShaderProgram()->GetId().StartsWith("shapeyard-data-maps-v1-normal-");
}

// Convert an OCCT presentation array into a private unindexed derivative.
// Original indices/attributes stay untouched; all original attributes are
// copied per corner, with a separate float4 custom tangent at location 4.
inline Handle(Graphic3d_Buffer) BuildNativeTangentBuffer(
    const Handle(Graphic3d_Buffer)& attributes,
    const Handle(Graphic3d_IndexBuffer)& indexBuffer) {
    using namespace core3d::scene;
    if (attributes.IsNull() || attributes->Data() == nullptr
        || attributes->NbElements < 1 || attributes->NbElements > kMaximumTangentVertices
        || attributes->NbAttributes < 3 || attributes->NbAttributes > 4)
        return {};
    if (!indexBuffer.IsNull() && (indexBuffer->Data() == nullptr
        || (indexBuffer->Stride != 2 && indexBuffer->Stride != 4))) return {};
    const int count = indexBuffer.IsNull() ? attributes->NbElements : indexBuffer->NbElements;
    if (count <= 0 || count % 3 != 0 || count / 3 > kMaximumTangentTriangles) return {};

    struct Field { const Standard_Byte* bytes; Standard_Size stride; int offset; int size; };
    std::vector<Field> fields;
    std::vector<Graphic3d_Attribute> descriptors;
    unsigned found = 0;
    int vertexStride = 0;
    for (int i = 0; i < attributes->NbAttributes; ++i) {
        const auto descriptor = attributes->Attribute(i);
        const auto id = descriptor.Id;
        if (id < Graphic3d_TOA_POS || id > Graphic3d_TOA_COLOR
            || (found & (1 << int(id)))) return {};
        found |= 1 << int(id);
        if ((id == Graphic3d_TOA_POS || id == Graphic3d_TOA_NORM)
                && descriptor.DataType != Graphic3d_TOD_VEC3) return {};
        if (id == Graphic3d_TOA_UV && descriptor.DataType != Graphic3d_TOD_VEC2) return {};
        if (id == Graphic3d_TOA_COLOR && descriptor.DataType != Graphic3d_TOD_VEC4
            && descriptor.DataType != Graphic3d_TOD_VEC4UB) return {};
        int foundIndex = -1;
        Standard_Size stride = 0;
        const auto bytes = attributes->AttributeData(id, foundIndex, stride);
        if (!bytes || foundIndex != i || stride < descriptor.Stride()) return {};
        fields.push_back({bytes, stride, vertexStride, descriptor.Stride()});
        descriptors.push_back(descriptor);
        vertexStride += descriptor.Stride();
    }
    if ((found & 7) != 7) return {};
    std::vector<Vertex> vertices(attributes->NbElements);
    for (std::size_t i = 0; i < fields.size(); ++i) {
        const auto& field = fields[i];
        const auto id = descriptors[i].Id;
        if (id == Graphic3d_TOA_COLOR) continue;
        for (int v = 0; v < attributes->NbElements; ++v) {
            float* destination = id == Graphic3d_TOA_POS ? &vertices[v].positionX
                : id == Graphic3d_TOA_NORM ? &vertices[v].normalX : &vertices[v].textureU;
            std::memcpy(destination, field.bytes + std::size_t(v) * field.stride, field.size);
        }
    }
    std::vector<std::uint32_t> indices(count);
    for (int i = 0; i < count; ++i) {
        std::uint32_t value = i;
        if (!indexBuffer.IsNull()) {
            if (indexBuffer->Stride == 2) {
                std::uint16_t small; std::memcpy(&small, indexBuffer->Data()+i*2, 2); value = small;
            } else { std::memcpy(&value, indexBuffer->Data()+i*4, 4); }
        }
        if (value >= vertices.size()) return {};
        indices[i] = value;
    }
    std::vector<Float4> frames;
    if (GenerateMikkCornerTangents(vertices, indices, true, frames) != TangentSpaceError::None) return {};
    descriptors.push_back({Graphic3d_TOA_CUSTOM, Graphic3d_TOD_VEC4});
    Handle(Graphic3d_Buffer) result = new Graphic3d_Buffer(Graphic3d_Buffer::DefaultAllocator());
    if (!result->Init(count, descriptors.data(), int(descriptors.size()))) return {};
    for (int corner = 0; corner < count; ++corner) {
        auto* out = result->changeValue(corner);
        for (const auto& field : fields)
            std::memcpy(out+field.offset, field.bytes+std::size_t(indices[corner])*field.stride, field.size);
        std::memcpy(out+vertexStride, &frames[corner], sizeof(Float4));
    }
    return result;
}

// UID bookkeeping survives OCCT releasing CPU arrays after VBO upload. Rebuild
// only new presentations; drop absent UIDs each frame rather than accumulating
// an unbounded edit history. No global keepArrayData setting is changed.
inline bool PrepareNativeTangentPresentations(
    const Handle(AIS_InteractiveContext)& context,
    const Handle(OpenGl_Context)& gl,
    std::unordered_map<Standard_Size, std::size_t>& prepared) {
    if (context.IsNull() || gl.IsNull()) return false;
    std::unordered_map<Standard_Size, std::size_t> active;
    struct Pending { OpenGl_PrimitiveArray* primitive; Handle(Graphic3d_Buffer) attributes; };
    std::vector<Pending> pending;
    std::size_t totalBytes = 0;
    AIS_ListOfInteractive displayed;
    context->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
    for (AIS_ListIteratorOfListOfInteractive it(displayed); it.More(); it.Next()) {
        auto shape = Handle(AIS_Shape)::DownCast(it.Value());
        if (shape.IsNull()) continue;
        // Current normal authoring is whole-object only. Avoid visiting every
        // triangle group of ordinary models on every camera frame.
        const auto drawer = shape->Attributes();
        if (drawer.IsNull() || drawer->ShadingAspect().IsNull()
            || !NeedsNativeTangents(drawer->ShadingAspect()->Aspect())) continue;
        for (const auto& presentation : shape->Presentations()) {
            if (presentation.IsNull()) continue;
            for (const auto& genericGroup : presentation->Groups()) {
                auto group = Handle(OpenGl_Group)::DownCast(genericGroup);
                if (group.IsNull()) continue;
                Handle(Graphic3d_Aspects) aspect = group->Aspects();
                for (auto* node = group->FirstNode(); node; node = node->next) {
                    if (auto* inlineAspect = dynamic_cast<OpenGl_Aspects*>(node->elem)) {
                        aspect = inlineAspect->Aspect(); continue;
                    }
                    if (!NeedsNativeTangents(aspect)) continue;
                    auto* primitive = dynamic_cast<OpenGl_PrimitiveArray*>(node->elem);
                    if (!primitive || !primitive->IsFillDrawMode()) continue;
                    if (primitive->DrawMode() != GL_TRIANGLES) return false;
                    const auto uid = primitive->GetUID();
                    if (active.count(uid)) continue;
                    const auto existing = prepared.find(uid);
                    std::size_t bytes = 0;
                    if (existing != prepared.end()) {
                        bytes = existing->second;
                    } else {
                        const auto attributes = BuildNativeTangentBuffer(primitive->Attributes(), primitive->Indices());
                        if (attributes.IsNull()) return false;
                        const auto& bounds = primitive->Bounds();
                        if (!bounds.IsNull()) {
                            std::size_t corners = 0;
                            if (bounds->NbBounds < 0 || bounds->NbBounds > bounds->NbMaxBounds
                                || (bounds->NbBounds && !bounds->Bounds)) return false;
                            for (int b = 0; b < bounds->NbBounds; ++b) {
                                if (bounds->Bounds[b] <= 0 || bounds->Bounds[b] % 3) return false;
                                corners += bounds->Bounds[b];
                            }
                            if (corners != attributes->NbElements) return false;
                        }
                        bytes = std::size_t(attributes->Stride) * attributes->NbElements;
                        pending.push_back({primitive, attributes});
                    }
                    totalBytes += bytes;
                    if (totalBytes > 64*1024*1024) return false;
                    active.emplace(uid, bytes);
                }
            }
        }
    }
    for (const auto& item : pending) {
        const auto bounds = item.primitive->Bounds();
        item.primitive->InitBuffers(gl, Graphic3d_TOPA_TRIANGLES, {}, item.attributes, bounds);
        // Record each successful replacement even if a later replacement
        // throws. A retry must not reinterpret its custom tangent as source.
        prepared[item.primitive->GetUID()] = active.at(item.primitive->GetUID());
    }
    prepared.swap(active);
    return true;
}
} // namespace core3d::render
