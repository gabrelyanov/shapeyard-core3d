#pragma once
#include <OpenGLES/ES2/gl.h>
#include "../Scene/MikkTangentSpace.hpp"
#include "Core3DNativeTangentState.hxx"
#include "NativeAuthoredFrameGeometry.hxx"
#include "OcctDocument.h"
#include "CafShapePrs.h"
#include <TDocStd_Document.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
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

// Resolve only a validated native owner. Presentation geometry is located,
// unlike the local mesh payload published by the scene snapshot builder.
struct NativeTangentOwner {
    OcctAuthoredFrameRecord record;
    TopoDS_Face localFace;
    gp_Trsf placement;
    std::array<std::uint8_t, 32> identity = {};
};

inline bool ReadNativeTangentOwner(const Handle(AIS_Shape)& shape,
                                  NativeTangentOwner& owner) {
    owner = {};
    const auto caf = Handle(CafShapePrs)::DownCast(shape);
    if (caf.IsNull()) return true;
    const auto label = caf->GetLabel();
    if (label.IsNull()) return false;
    const auto doc = TDocStd_Document::Get(label);
    if (doc.IsNull()) return false;
    // Native interactive previews may hold a command open. Validate the current
    // owner/geometry without altering that command; publication has its own gate.
    const auto state = Core3DReadAuthoredFrameOwner(doc, label, owner.record);
    if (state == OcctAuthoredFrameReadState::Invalid) return false;
    if (state == OcctAuthoredFrameReadState::Absent) return true;
    auto local = XCAFDoc_ShapeTool::GetShape(label);
    owner.placement = local.Location().Transformation();
    // Native owned geometry admits rigid outer placement. Signed object scale
    // remains in AIS's local transform and the normal shader's world matrix.
    if (std::abs(owner.placement.ScaleFactor() - 1.0) > 1.e-12
        || owner.placement.VectorialPart().Determinant() <= 0) return false;
    local.Location(TopLoc_Location());
    if (local.ShapeType() == TopAbs_FACE) owner.localFace = TopoDS::Face(local);
    else if (local.ShapeType() == TopAbs_COMPOUND) {
        TopoDS_Iterator child(local);
        if (!child.More() || child.Value().ShapeType() != TopAbs_FACE) return false;
        owner.localFace = TopoDS::Face(child.Value());
        child.Next(); if (child.More()) return false;
    } else return false;
    CC_SHA256_CTX hash; CC_SHA256_Init(&hash);
    CC_SHA256_Update(&hash, owner.record.identity.data(), CC_LONG(owner.record.identity.size()));
    for (int row = 1; row <= 3; ++row) for (int column = 1; column <= 4; ++column) {
        const double value = owner.placement.Value(row, column);
        if (!std::isfinite(value)) return false;
        CC_SHA256_Update(&hash, &value, sizeof(value));
    }
    CC_SHA256_Final(owner.identity.data(), &hash);
    return true;
}

inline bool NativeSuppliedCornerTangents(const NativeTangentOwner& owner,
                                        const std::vector<scene::Vertex>& vertices,
                                        const std::vector<std::uint32_t>& indices,
                                        std::vector<scene::Float4>& output,
                                        NativeTangentPreparationTrace* trace = nullptr) {
    if (trace) trace->stage = 130;
    output.clear();
    std::vector<scene::Float4> frames;
    if (!persistence::DecodeNativeAuthoredFrames(owner.localFace,
            owner.record.archive.data(), owner.record.archive.size(), frames)
        || frames.size() != indices.size()) return false;
    if (trace) trace->stage = 131;
    TopLoc_Location location;
    const auto mesh = BRep_Tool::Triangulation(owner.localFace, location);
    if (mesh.IsNull() || !location.IsIdentity()) return false;
    // Compare expanded corners, not arbitrary presentation vertex numbering.
    // This preserves different authored frames at shared source indices.
    for (int t = 1; t <= mesh->NbTriangles(); ++t) {
        int nodes[3]; mesh->Triangle(t).Get(nodes[0], nodes[1], nodes[2]);
        for (int c = 0; c < 3; ++c) {
            const auto corner = std::size_t(t - 1) * 3 + c;
            if (indices[corner] >= vertices.size()) return false;
            const auto point = mesh->Node(nodes[c]).Transformed(owner.placement);
            const auto normal = mesh->Normal(nodes[c]).Transformed(owner.placement);
            const auto uv = mesh->UVNode(nodes[c]);
            const scene::Vertex expected = {float(point.X()), float(point.Y()), float(point.Z()),
                float(normal.X()), float(normal.Y()), float(normal.Z()), float(uv.X()), float(uv.Y())};
            const auto& actual = vertices[indices[corner]];
            if (std::memcmp(&expected, &actual, sizeof(expected)) != 0) {
                if (trace) {
                    trace->stage = 132; trace->mismatchCorner = int(corner + 1);
                    std::uint32_t expectedWords[8], actualWords[8];
                    static_assert(sizeof(expectedWords) == sizeof(expected));
                    std::memcpy(expectedWords, &expected, sizeof(expected));
                    std::memcpy(actualWords, &actual, sizeof(actual));
                    for (int component = 0; component < 8; ++component) {
                        if (expectedWords[component] != actualWords[component]) {
                            trace->mismatchComponent = component + 1;
                            trace->expectedBits = expectedWords[component];
                            trace->actualBits = actualWords[component]; break;
                        }
                    }
                }
                return false;
            }
            auto& frame = frames[corner];
            // Keep identity placement byte-exact, including signed zero.
            if (owner.placement.Form() != gp_Identity) {
                const auto tangent = gp_Vec(frame.x, frame.y, frame.z).Transformed(owner.placement);
                frame.x = float(tangent.X()); frame.y = float(tangent.Y()); frame.z = float(tangent.Z());
            }
        }
    }
    output.swap(frames); return true;
}

// Convert an OCCT presentation array into a private unindexed derivative.
// Original indices/attributes stay untouched; all original attributes are
// copied per corner, with a separate float4 custom tangent at location 4.
inline Handle(Graphic3d_Buffer) BuildNativeTangentBuffer(
    const Handle(Graphic3d_Buffer)& attributes,
    const Handle(Graphic3d_IndexBuffer)& indexBuffer,
    const NativeTangentOwner* owner = nullptr,
    NativeTangentPreparationTrace* trace = nullptr) {
    using namespace core3d::scene;
    if (trace) {
        trace->stage = 10;
        trace->sourceElements = attributes.IsNull() ? 0 : attributes->NbElements;
        trace->sourceAttributes = attributes.IsNull() ? 0 : attributes->NbAttributes;
        trace->sourceCPUData = !attributes.IsNull() && attributes->Data() != nullptr;
    }
    if (attributes.IsNull() || attributes->Data() == nullptr
        || attributes->NbElements < 1 || attributes->NbElements > kMaximumTangentVertices
        || attributes->NbAttributes < 3 || attributes->NbAttributes > 4)
        return {};
    if (!indexBuffer.IsNull() && (indexBuffer->Data() == nullptr
        || (indexBuffer->Stride != 2 && indexBuffer->Stride != 4))) return {};
    const int count = indexBuffer.IsNull() ? attributes->NbElements : indexBuffer->NbElements;
    if (count <= 0 || count % 3 != 0 || count / 3 > kMaximumTangentTriangles) return {};

    if (trace) trace->stage = 11;
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
    if (trace) trace->stage = 12;
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
    if (trace) trace->stage = 13;
    if (owner && !owner->record.archive.empty()) {
        if (!NativeSuppliedCornerTangents(*owner, vertices, indices, frames, trace)) return {};
    } else if (GenerateMikkCornerTangents(vertices, indices, true, frames) != TangentSpaceError::None) return {};
    if (trace) trace->stage = 14;
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
    std::unordered_map<Standard_Size, NativeTangentArrayState>& prepared,
    NativeTangentPreparationTrace* trace = nullptr) {
    if (trace) { *trace = {}; trace->stage = 1; }
    if (context.IsNull() || gl.IsNull()) return false;
    std::unordered_map<Standard_Size, NativeTangentArrayState> active;
    struct Pending { OpenGl_PrimitiveArray* primitive; Handle(Graphic3d_Buffer) attributes; };
    std::vector<Pending> pending;
    std::size_t totalBytes = 0;
    AIS_ListOfInteractive resident;
    // Erase retains OCCT presentations/VBOs. Keep their ready UIDs until the
    // native object is actually removed, even while it is hidden.
    context->ObjectsInside(resident, AIS_KOI_Shape, -1);
    for (AIS_ListIteratorOfListOfInteractive it(resident); it.More(); it.Next()) {
        auto shape = Handle(AIS_Shape)::DownCast(it.Value());
        if (shape.IsNull()) continue;
        // Current normal authoring is whole-object only. Avoid visiting every
        // triangle group of ordinary models on every camera frame.
        const auto drawer = shape->Attributes();
        if (drawer.IsNull() || drawer->ShadingAspect().IsNull()
            || !NeedsNativeTangents(drawer->ShadingAspect()->Aspect())) continue;
        NativeTangentOwner owner;
        if (trace) trace->stage = 2;
        if (!ReadNativeTangentOwner(shape, owner)) return false;
        if (trace) trace->stage = 3;
        bool changedBasis = false, sourceArraysReleased = false;
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
                    auto* primitive = dynamic_cast<OpenGl_PrimitiveArray*>(node->elem);
                    if (!primitive) continue;
                    const auto old = prepared.find(primitive->GetUID());
                    if (old != prepared.end() && old->second.basisIdentity != owner.identity) changedBasis = true;
                    if (old == prepared.end() && primitive->IsFillDrawMode() && NeedsNativeTangents(aspect)) {
                        const auto& attributes = primitive->Attributes();
                        const auto& indices = primitive->Indices();
                        sourceArraysReleased = sourceArraysReleased || attributes.IsNull()
                            || attributes->Data() == nullptr
                            || (!indices.IsNull() && indices->Data() == nullptr);
                    }
                }
            }
        }
        if (changedBasis || sourceArraysReleased) {
            // An earlier ordinary draw can upload and release its CPU arrays
            // before the normal shader becomes active. Rebuild once from the
            // exact native shape; never reinterpret a VBO/private derivative as
            // source or bypass supplied-corner validation to recover a frame.
            const auto caf = Handle(CafShapePrs)::DownCast(shape);
            if (caf.IsNull()) return false;
            if (!caf->Shape().IsEqual(XCAFDoc_ShapeTool::GetShape(caf->GetLabel())))
                caf->DispatchStyles(Standard_False);
            // Presentation-only recomputation preserves selectable owners and
            // OCAF history. The normal budget/array checks below still apply.
            context->RecomputePrsOnly(shape, Standard_False, Standard_True);
        }
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
                    const auto alreadyActive = active.find(uid);
                    if (alreadyActive != active.end()) {
                        if (alreadyActive->second.basisIdentity != owner.identity) return false;
                        continue;
                    }
                    const auto existing = prepared.find(uid);
                    std::size_t bytes = 0;
                    if (existing != prepared.end()) {
                        if (existing->second.basisIdentity != owner.identity) return false;
                        bytes = existing->second.bytes;
                    } else {
                        const auto attributes = BuildNativeTangentBuffer(primitive->Attributes(), primitive->Indices(), &owner, trace);
                        if (attributes.IsNull()) return false;
                        if (trace) trace->stage = 4;
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
                        bytes = std::size_t(attributes->Stride) * attributes->NbElements + owner.record.archive.size();
                        pending.push_back({primitive, attributes});
                    }
                    totalBytes += bytes;
                    if (totalBytes > 64*1024*1024) return false;
                    active.emplace(uid, NativeTangentArrayState{bytes, owner.identity});
                }
            }
        }
    }
    if (trace) trace->stage = 5;
    for (const auto& item : pending) {
        const auto bounds = item.primitive->Bounds();
        item.primitive->InitBuffers(gl, Graphic3d_TOPA_TRIANGLES, {}, item.attributes, bounds);
        // Record each successful replacement even if a later replacement
        // throws. A retry must not reinterpret its custom tangent as source.
        prepared[item.primitive->GetUID()] = active.at(item.primitive->GetUID());
    }
    prepared.swap(active);
    if (trace) trace->stage = 0;
    return true;
}
} // namespace core3d::render
