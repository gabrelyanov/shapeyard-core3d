// Copyright (c) 2017 OPEN CASCADE SAS
//
// This file is part of the examples of the Open CASCADE Technology software library.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE

#import <Foundation/Foundation.h>

#import "GLViewController.h"
#import "GLViewController+QueuedAssetLoading.h"
#include <dirent.h>
#include <Graphic3d_ShaderProgram.hxx>
#import "GLView.h"

#import "NSString+StdString.h"
#import <CommonCrypto/CommonDigest.h>

#include <BRepAdaptor_Curve.hxx>
#include "ConstructorManipulator.hpp"
#include "CafShapePrs.h"
#include "OcctDocument.h"
#include "../Common/Core3DMobileResourceLimits.h"

#include <Graphic3d_TextureParams.hxx>
#include <Image_PixMap.hxx>
#include <Image_SupportedFormats.hxx>
#include <gp_Quaternion.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <Graphic3d_ArrayOfTriangles.hxx>
#include <Graphic3d_ArrayOfSegments.hxx>
#include <TopLoc_Datum3D.hxx>
#include <unordered_map>
#include <map>
#include <stdexcept>
#include <Graphic3d_Group.hxx>
#include <Graphic3d_Structure.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <TColStd_ListIteratorOfListOfInteger.hxx>
#include <TColStd_ListOfInteger.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#ifdef DEBUG
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <BRepAdaptor_Surface.hxx>
#endif
#include <XCAFPrs_Texture.hxx>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <unordered_set>
#include <sys/stat.h>
#include <unistd.h>

#if DEBUG
// D333/D326 probe (defined in OcctDocument.mm): real measurements of
// relocation-table payloads still bound when a bounded retrieval driver dies.
void Core3DDebugResetBoundedRetrievalDriverTeardownStats();
void Core3DDebugBoundedRetrievalDriverTeardownStats(
    unsigned long long& teardowns,
    unsigned long long& retainedEntries,
    unsigned long long& retainedNamedShapes);
void Core3DDebugSetRelocationReadThrowAfterNamedShapeBinding(int armed);
#endif

using namespace core3d;

namespace {

void RemoveNativeSelectionStructure(Handle(Graphic3d_Structure)& structure)
{
    if (structure.IsNull()) return;
    if (!structure->IsDeleted()) {
        // Remove destroys the graphics structure; it does not unregister the
        // displayed structure from the views. Erase while its layer and bounds
        // are still alive, otherwise redraw and viewer teardown use freed data.
        structure->Erase();
        structure->Remove();
    }
    // Exception cleanup can hold another handle to an already removed overlay.
    structure.Nullify();
}

// One depth step of GLView's supported ES2 16-bit fallback keeps coplanar
// native edges visible. Two steps can expose a hidden edge in perspective
// when the near plane is small compared with the model's camera distance.
// This only resolves coincident lines: normal scene depth testing stays on,
// widths/model coordinates stay unchanged, and occColor supplies each style.
Handle(Graphic3d_ShaderProgram) NativeBoundaryLineShader(bool screenContour = false)
{
    Handle(Graphic3d_ShaderProgram) program = new Graphic3d_ShaderProgram();
    program->SetId(screenContour ? "shapeyard-native-contour-depth-v2" : "shapeyard-native-boundary-depth-v3");
    program->SetHeader("#version 100");
    program->SetDefaultSampler(Standard_False);
    program->SetNbLightsMax(0);
    program->SetNbClipPlanesMax(8);
    program->AttachShader(Graphic3d_ShaderObject::CreateFromSource(Graphic3d_TOS_VERTEX, screenContour ? R"GLSL(
varying vec3 syBoundaryWorld;
void main() {
    vec4 world = occModelWorldMatrix * occVertex;
    syBoundaryWorld = world.xyz / world.w;
    gl_Position = occProjectionMatrix * occWorldViewMatrix * world;
    // Offsets are normalized screen displacements, not material UVs. Expand
    // with the final draw matrix; AutoZFit can update depth after preparation.
    gl_Position.xy += occTexCoord.xy * gl_Position.w;
    if (gl_Position.z >= -gl_Position.w)
        gl_Position.z = max(-gl_Position.w, gl_Position.z - gl_Position.w * (2.0 / 65535.0));
}
)GLSL" : R"GLSL(
varying vec3 syBoundaryWorld;
void main() {
    vec4 world = occModelWorldMatrix * occVertex;
    syBoundaryWorld = world.xyz / world.w;
    gl_Position = occProjectionMatrix * occWorldViewMatrix * world;
    // OpenGL NDC spans [-1,1]: two NDC steps are one framebuffer step.
    gl_Position.z -= gl_Position.w * (2.0 / 65535.0);
}
)GLSL"));
    program->AttachShader(Graphic3d_ShaderObject::CreateFromSource(Graphic3d_TOS_FRAGMENT, R"GLSL(
varying vec3 syBoundaryWorld;
void main() {
    if (occFragEarlyReturn()) return;
    int nextPlane = 0;
    for (int i = 0; i < 8; ++i) {
        if (i >= nextPlane && i < occClipPlaneCount) {
            if (dot(occClipPlaneEquations[i], vec4(syBoundaryWorld, 1.0)) < 0.0) {
                if (occClipPlaneChains[i] <= 1) discard;
                nextPlane = i + 1;
            } else {
                nextPlane = i + (occClipPlaneChains[i] > 1 ? occClipPlaneChains[i] : 1);
            }
        }
    }
    occSetFragColor(occColor);
}
)GLSL"));
    return program;
}

// Optional selection geometry, built only from an already-published mesh.
// These caps cover the entire selection, not each object independently.
constexpr std::size_t kNativeContourVertexBudget = 500'000;
constexpr std::size_t kNativeContourIndexBudget = 1'500'000;
struct NativeContourGeometry {
    struct Triangle { std::array<std::uint32_t,3> vertices; };
    struct Edge {
        std::uint32_t a, b;
        std::array<std::uint32_t,2> triangles;
        unsigned count = 0;
    };
    std::string entity;
    std::vector<gp_Pnt> vertices;
    std::vector<Triangle> triangles;
    std::vector<Edge> edges;
};

NativeContourGeometry BuildNativeContourGeometry(const core3d::scene::MeshSnapshot& mesh,
                                                 const core3d::scene::InstanceSnapshot& item,
                                                 const std::unordered_set<std::uint32_t>* selectedFaces,
                                                 std::size_t& vertexBudget,
                                                 std::size_t& indexBudget)
{
    if (mesh.vertices.size() > kNativeContourVertexBudget-vertexBudget
        || mesh.indices.size() > kNativeContourIndexBudget-indexBudget)
        throw std::length_error("Native contour geometry budget");
    vertexBudget += mesh.vertices.size(); indexBudget += mesh.indices.size();
    NativeContourGeometry result; result.entity = item.entityIdentifier;
    // Mesh vertices are float positions. Weld duplicate positions across UV and
    // normal seams, without changing the source mesh or its topology. The grid
    // is far below a drawable pixel and bounded to the float precision of this
    // mesh. Adjacent cells are searched so a quantization boundary is not a seam.
    double extent = 0;
    for (const auto& v : mesh.vertices)
        extent = std::max(extent, std::max({std::abs(double(v.positionX)),
            std::abs(double(v.positionY)),std::abs(double(v.positionZ))}));
    const double epsilon = std::max(1.0e-9,extent*2.0e-7);
    using Cell = std::array<std::int64_t,3>;
    std::map<Cell,std::vector<std::uint32_t>> cells;
    std::vector<gp_Pnt> local;
    std::vector<std::uint32_t> welded; welded.reserve(mesh.vertices.size());
    const auto& m = item.worldFromObject.values;
    for (const auto& v : mesh.vertices) {
        const gp_Pnt point(v.positionX,v.positionY,v.positionZ);
        if (!std::isfinite(point.X()) || !std::isfinite(point.Y()) || !std::isfinite(point.Z()))
            throw std::runtime_error("Native contour finite vertex");
        const Cell cell = {std::int64_t(std::floor(point.X()/epsilon)),
            std::int64_t(std::floor(point.Y()/epsilon)),std::int64_t(std::floor(point.Z()/epsilon))};
        std::uint32_t index = std::uint32_t(local.size());
        for (int x=-1;x<=1;++x) for (int y=-1;y<=1;++y) for (int z=-1;z<=1;++z) {
            const auto found = cells.find({cell[0]+x,cell[1]+y,cell[2]+z});
            if (found == cells.end()) continue;
            for (const auto candidate : found->second) {
                if (point.SquareDistance(local[candidate]) <= epsilon*epsilon)
                    index = std::min(index,candidate);
            }
        }
        if (index == local.size()) {
            local.push_back(point); cells[cell].push_back(index);
            const double x=point.X(), y=point.Y(), z=point.Z();
            result.vertices.emplace_back(m[0]*x+m[4]*y+m[8]*z+m[12],
                m[1]*x+m[5]*y+m[9]*z+m[13],m[2]*x+m[6]*y+m[10]*z+m[14]);
        }
        welded.push_back(index);
    }
    std::map<std::pair<std::uint32_t,std::uint32_t>,std::size_t> edgeMap;
    for (std::size_t p=0;p<mesh.primitives.size();++p) {
        if (!item.primitiveBindings.at(p).visible) continue;
        const auto& primitive = mesh.primitives[p];
        if (selectedFaces && !selectedFaces->count(primitive.faceIndex)) continue;
        if (primitive.indexCount%3 != 0 || std::size_t(primitive.firstIndex)+primitive.indexCount > mesh.indices.size())
            throw std::runtime_error("Native contour triangle range");
        for (std::size_t i=primitive.firstIndex;i<std::size_t(primitive.firstIndex)+primitive.indexCount;i+=3) {
            NativeContourGeometry::Triangle triangle{{welded.at(mesh.indices[i]),
                welded.at(mesh.indices[i+1]),welded.at(mesh.indices[i+2])}};
            if (item.reversesWinding) std::swap(triangle.vertices[1],triangle.vertices[2]);
            if (triangle.vertices[0] == triangle.vertices[1] || triangle.vertices[1] == triangle.vertices[2]
                || triangle.vertices[2] == triangle.vertices[0]) continue;
            const auto ti = std::uint32_t(result.triangles.size());
            result.triangles.push_back(triangle);
            for (int e=0;e<3;++e) {
                const auto a=triangle.vertices[e], b=triangle.vertices[(e+1)%3];
                const auto key=std::minmax(a,b);
                const auto inserted=edgeMap.emplace(key,result.edges.size());
                if (inserted.second) result.edges.push_back({a,b,{0,0},0});
                auto& edge=result.edges[inserted.first->second];
                if (edge.count < 2) edge.triangles[edge.count]=ti;
                ++edge.count;
            }
        }
    }
    return result;
}

struct NativeContourVertex { gp_Pnt position; gp_Pnt2d offset; };
struct NativeProjectedContour {
    struct Segment { gp_Pnt a,b; double nx,ny; };
    std::vector<Segment> segments;
    std::vector<NativeContourVertex> triangles;
    // Screen buckets bound semantic-edge matching to nearby silhouette pieces.
    std::map<std::pair<int,int>,std::vector<std::size_t>> buckets;
    std::size_t bucketEntries = 0;

    bool containsBoundary(const gp_Pnt& a, const gp_Pnt& b) const {
        const double x=(a.X()+b.X())*0.5, y=(a.Y()+b.Y())*0.5;
        if (!std::isfinite(x) || !std::isfinite(y)
            || std::abs(x/8) >= std::numeric_limits<int>::max()-1
            || std::abs(y/8) >= std::numeric_limits<int>::max()-1) return false;
        const auto bucket=buckets.find({int(std::floor(x/8)),int(std::floor(y/8))});
        if (bucket == buckets.end()) return false;
        const double ux=b.X()-a.X(), uy=b.Y()-a.Y(), ul=std::hypot(ux,uy);
        if (ul < 1.0e-9) return true;
        for (const auto i:bucket->second) {
            const auto& segment=segments[i];
            const double dx=segment.b.X()-segment.a.X(), dy=segment.b.Y()-segment.a.Y();
            const double length2=dx*dx+dy*dy;
            const double t=std::clamp(((x-segment.a.X())*dx+(y-segment.a.Y())*dy)/length2,0.0,1.0);
            if (std::hypot(x-segment.a.X()-t*dx,y-segment.a.Y()-t*dy) <= 1.25
                && std::abs(ux*dx+uy*dy) >= 0.95*ul*std::sqrt(length2)) return true;
        }
        return false;
    }
};

NativeProjectedContour ProjectNativeContours(const NativeContourGeometry& geometry,
                                             const Handle(Graphic3d_Camera)& camera,
                                             double width, double height, double stroke)
{
    NativeProjectedContour result;
    std::vector<gp_Pnt> projected; projected.reserve(geometry.vertices.size());
    std::vector<bool> inFront; inFront.reserve(geometry.vertices.size());
    for (const auto& point:geometry.vertices) {
        inFront.push_back(camera->IsOrthographic()
            || gp_Vec(camera->Eye(),point).Dot(gp_Vec(camera->Direction())) > 1.0e-9);
        const auto p=camera->Project(point);
        projected.emplace_back((p.X()+1)*width*0.5,(p.Y()+1)*height*0.5,p.Z());
    }
    std::vector<double> facing; facing.reserve(geometry.triangles.size());
    for (const auto& triangle:geometry.triangles) {
        const auto& a=projected[triangle.vertices[0]];
        const auto& b=projected[triangle.vertices[1]];
        const auto& c=projected[triangle.vertices[2]];
        // Redraw can refit near/far after preparation. Only reject points
        // behind the eye here; the final GPU matrix owns frustum clipping.
        const bool behind = !inFront[triangle.vertices[0]] || !inFront[triangle.vertices[1]]
            || !inFront[triangle.vertices[2]];
        facing.push_back(behind ? 0 : (b.X()-a.X())*(c.Y()-a.Y())-(b.Y()-a.Y())*(c.X()-a.X()));
    }
    const auto vertex=[&](std::uint32_t index,double nx,double ny) -> NativeContourVertex {
        return {geometry.vertices[index],gp_Pnt2d(2*nx*stroke/width,2*ny*stroke/height)};
    };
    std::map<std::uint32_t,std::vector<std::array<double,2>>> joins;
    for (const auto& edge:geometry.edges) {
        if (edge.count > 2 || edge.count == 0) continue;
        const bool frontA=facing[edge.triangles[0]] > 1.0e-8;
        const bool frontB=edge.count == 2 && facing[edge.triangles[1]] > 1.0e-8;
        if (edge.count == 2 && frontA == frontB) continue;
        const auto ti=edge.triangles[frontA || edge.count == 1 ? 0:1];
        if (std::abs(facing[ti]) <= 1.0e-8) continue;
        const auto& a=projected[edge.a]; const auto& b=projected[edge.b];
        // The ordinary shader applies the final frustum and all user clip
        // planes. A previous frame's near/far range cannot filter this mesh.
        if (!std::isfinite(a.X()) || !std::isfinite(a.Y()) || !std::isfinite(b.X()) || !std::isfinite(b.Y())
            || !inFront[edge.a] || !inFront[edge.b]) continue;
        const double dx=b.X()-a.X(),dy=b.Y()-a.Y(),length=std::hypot(dx,dy);
        if (length < 1.0e-6) continue;
        std::uint32_t third=edge.a;
        for (const auto v:geometry.triangles[ti].vertices) if (v != edge.a && v != edge.b) third=v;
        const auto& c=projected[third];
        const double sign=(dx*(c.Y()-a.Y())-dy*(c.X()-a.X())) > 0 ? 1:-1;
        const double nx=sign*dy/length,ny=-sign*dx/length;
        if (std::max(a.X(),b.X()) < -stroke || std::min(a.X(),b.X()) > width+stroke
            || std::max(a.Y(),b.Y()) < -stroke || std::min(a.Y(),b.Y()) > height+stroke) continue;
        const auto index=result.segments.size();
        result.segments.push_back({a,b,nx,ny});
        // The whole two-point stroke lies outside the projected surface. A
        // centered GL line loses its inner half behind a curved surface, even
        // with the small coplanar bias used for ordinary semantic edges.
        const auto a0=vertex(edge.a,0,0),b0=vertex(edge.b,0,0);
        const auto a1=vertex(edge.a,nx,ny),b1=vertex(edge.b,nx,ny);
        result.triangles.insert(result.triangles.end(),{a0,b0,b1,a0,b1,a1});
        joins[edge.a].push_back({nx,ny}); joins[edge.b].push_back({nx,ny});
        if (result.triangles.size() > 1'500'000) throw std::length_error("Native contour stroke budget");
        const int x0=int(std::floor(std::max(-8.0,std::min(a.X(),b.X())-2)/8));
        const int x1=int(std::floor(std::min(width+8,std::max(a.X(),b.X())+2)/8));
        const int y0=int(std::floor(std::max(-8.0,std::min(a.Y(),b.Y())-2)/8));
        const int y1=int(std::floor(std::min(height+8,std::max(a.Y(),b.Y())+2)/8));
        for (int x=x0;x<=x1;++x) for (int y=y0;y<=y1;++y) {
            if (++result.bucketEntries > 1'500'000) throw std::length_error("Native contour lookup budget");
            result.buckets[{x,y}].push_back(index);
        }
    }
    // Round joins retain the full stroke radius at tessellation vertices.
    // Bevel joins pinch a shallow curved silhouette at its exact equator.
    for (const auto& join:joins) {
        if (join.second.size() != 2) continue;
        const auto& a=join.second[0]; const auto& b=join.second[1];
        const double length=std::hypot(a[0]+b[0],a[1]+b[1]);
        if (length < 1.0e-6) continue;
        if (result.triangles.size() > 1'500'000-6) throw std::length_error("Native contour join budget");
        const auto center=vertex(join.first,0,0);
        const auto middle=vertex(join.first,(a[0]+b[0])/length,(a[1]+b[1])/length);
        result.triangles.insert(result.triangles.end(),{
            center,vertex(join.first,a[0],a[1]),middle,
            center,middle,vertex(join.first,b[0],b[1])});
    }
    return result;
}

constexpr char kCbfMagic[] = "BINFILE";

bool TryBooleanActionForGizmo(
    const PrimitiveGizmoType theType,
    BooleanAction& theAction) noexcept
{
    switch (theType) {
        case PrimitiveGizmoTypeSubtract:
            theAction = BooleanAction::BooleanSubtract;
            return true;
        case PrimitiveGizmoTypeUnion:
            theAction = BooleanAction::BooleanUnion;
            return true;
        case PrimitiveGizmoTypeIntersect:
            theAction = BooleanAction::BooleanIntersect;
            return true;
        default:
            return false;
    }
}

bool IsBooleanGizmo(const PrimitiveGizmoType theType) noexcept
{
    BooleanAction anAction = BooleanAction::BooleanSubtract;
    return TryBooleanActionForGizmo(theType, anAction);
}

bool SelectionModeAllowsGizmo(
    const ShapeSelectionMode theMode,
    const PrimitiveGizmoType theType) noexcept
{
    if (theType == PrimitiveGizmoTypeNone) {
        return true;
    }
    switch (theMode) {
        case ShapeSelectionMode::WholeShape:
            switch (theType) {
                case PrimitiveGizmoTypeMoveRotate:
                case PrimitiveGizmoTypeScale:
                case PrimitiveGizmoTypeChamfer:
                case PrimitiveGizmoTypeSubtract:
                case PrimitiveGizmoTypeUnion:
                case PrimitiveGizmoTypeMirror:
                case PrimitiveGizmoTypeMaterial:
                case PrimitiveGizmoTypeIntersect:
                case PrimitiveGizmoTypeLinearArray:
                case PrimitiveGizmoTypeRadialArray:
                    return true;
                case PrimitiveGizmoTypeNone:
                case PrimitiveGizmoTypeExtrude:
                case PrimitiveGizmoTypeShell:
                    return false;
            }
            return false;
        case ShapeSelectionMode::Face:
            return theType == PrimitiveGizmoTypeChamfer
                || theType == PrimitiveGizmoTypeExtrude
                || theType == PrimitiveGizmoTypeShell;
        case ShapeSelectionMode::Edge:
            return theType == PrimitiveGizmoTypeChamfer;
        case ShapeSelectionMode::Vertex:
        case ShapeSelectionMode::Wire:
            return false;
    }
    return false;
}

bool TryShapeSelectionMode(const PrimitiveSelectionType theType,
                           ShapeSelectionMode& theMode) noexcept
{
    switch (theType) {
        case PrimitiveSelectionTypeShape:
            theMode = ShapeSelectionMode::WholeShape;
            return true;
        case PrimitiveSelectionTypeFace:
            theMode = ShapeSelectionMode::Face;
            return true;
        case PrimitiveSelectionTypeEdge:
            theMode = ShapeSelectionMode::Edge;
            return true;
        case PrimitiveSelectionTypeVertex:
        case PrimitiveSelectionTypeNone:
        default:
            return false;
    }
}

Core3DSelectionTypeChangeResult PublicSelectionTypeChangeResult(
    const ShapeSelectionModeChangeResult theResult) noexcept
{
    switch (theResult) {
        case ShapeSelectionModeChangeResult::Succeeded:
            return Core3DSelectionTypeChangeResultSucceeded;
        case ShapeSelectionModeChangeResult::Unsupported:
            return Core3DSelectionTypeChangeResultUnsupported;
        case ShapeSelectionModeChangeResult::NotReady:
            return Core3DSelectionTypeChangeResultNotReady;
        case ShapeSelectionModeChangeResult::Busy:
            return Core3DSelectionTypeChangeResultBusy;
        case ShapeSelectionModeChangeResult::PresentationFailure:
            return Core3DSelectionTypeChangeResultPresentationFailure;
    }
    return Core3DSelectionTypeChangeResultPresentationFailure;
}

bool TryNativeReferenceSpace(
    const Core3DReferenceSpace theSpace,
    OcctReferenceSpace& theNativeSpace) noexcept
{
    switch (theSpace) {
        case Core3DReferenceSpaceObject:
            theNativeSpace = OcctReferenceSpace::Object;
            return true;
        case Core3DReferenceSpaceWorld:
            theNativeSpace = OcctReferenceSpace::World;
            return true;
        default:
            return false;
    }
}

bool TryPublicReferenceSpace(
    const OcctReferenceSpace theSpace,
    Core3DReferenceSpace& thePublicSpace) noexcept
{
    switch (theSpace) {
        case OcctReferenceSpace::Object:
            thePublicSpace = Core3DReferenceSpaceObject;
            return true;
        case OcctReferenceSpace::World:
            thePublicSpace = Core3DReferenceSpaceWorld;
            return true;
    }
    return false;
}

Core3DReferenceAxisReadState PublicReferenceAxisReadState(
    const OcctReferenceAxisReadState theState) noexcept
{
    switch (theState) {
        case OcctReferenceAxisReadState::Invalid:
            return Core3DReferenceAxisReadStateInvalid;
        case OcctReferenceAxisReadState::ImplicitDefault:
            return Core3DReferenceAxisReadStateImplicitDefault;
        case OcctReferenceAxisReadState::Authored:
            return Core3DReferenceAxisReadStateAuthored;
    }
    return Core3DReferenceAxisReadStateInvalid;
}

Core3DRadialArrayReferenceEditResult PublicRadialReferenceEditResult(
    const RadialArrayReferenceEditResult theResult) noexcept
{
    switch (theResult) {
        case RadialArrayReferenceEditResult::NoChange:
            return Core3DRadialArrayReferenceEditResultNoChange;
        case RadialArrayReferenceEditResult::Applied:
            return Core3DRadialArrayReferenceEditResultApplied;
        case RadialArrayReferenceEditResult::OutcomeUnknown:
            return Core3DRadialArrayReferenceEditResultOutcomeUnknown;
        case RadialArrayReferenceEditResult::RetryableFailure:
            return Core3DRadialArrayReferenceEditResultRetryableFailure;
    }
    return Core3DRadialArrayReferenceEditResultRetryableFailure;
}

BOOL HasCbfMagic(NSData *data) {
    constexpr NSUInteger magicLength = sizeof(kCbfMagic) - 1;
    return data != nil
        && data.length >= magicLength
        && std::memcmp(data.bytes, kCbfMagic, magicLength) == 0;
}

struct VerifiedFileIdentity {
    dev_t device = 0;
    ino_t inode = 0;
    off_t size = 0;
    mode_t mode = 0;
    nlink_t links = 0;
    timespec modification = {};
    timespec change = {};
};

VerifiedFileIdentity FileIdentity(const struct stat& value) {
    return {
        value.st_dev,
        value.st_ino,
        value.st_size,
        value.st_mode,
        value.st_nlink,
        value.st_mtimespec,
        value.st_ctimespec,
    };
}

bool operator==(const VerifiedFileIdentity& left,
                const VerifiedFileIdentity& right) {
    return left.device == right.device
        && left.inode == right.inode
        && left.size == right.size
        && left.mode == right.mode
        && left.links == right.links
        && left.modification.tv_sec == right.modification.tv_sec
        && left.modification.tv_nsec == right.modification.tv_nsec
        && left.change.tv_sec == right.change.tv_sec
        && left.change.tv_nsec == right.change.tv_nsec;
}

bool DecodeSHA256(NSString *value,
                  std::array<unsigned char, CC_SHA256_DIGEST_LENGTH>& result) {
    if (value == nil || value.length != CC_SHA256_DIGEST_LENGTH * 2) {
        return false;
    }
    const auto nibble = [](const unichar character, unsigned char& output) {
        if (character >= '0' && character <= '9') {
            output = static_cast<unsigned char>(character - '0');
            return true;
        }
        if (character >= 'a' && character <= 'f') {
            output = static_cast<unsigned char>(character - 'a' + 10);
            return true;
        }
        return false;
    };
    for (NSUInteger index = 0; index < result.size(); ++index) {
        unsigned char high = 0;
        unsigned char low = 0;
        if (!nibble([value characterAtIndex:index * 2], high)
            || !nibble([value characterAtIndex:index * 2 + 1], low)) {
            return false;
        }
        result[index] = static_cast<unsigned char>((high << 4) | low);
    }
    return true;
}

Core3DAssetLoadResult AssetLoadResultFromImportResult(AssetImportResult result) {
    switch (result) {
        case AssetImportResult::Success:
            return Core3DAssetLoadResultSuccess;
        case AssetImportResult::InvalidData:
            return Core3DAssetLoadResultInvalidData;
        case AssetImportResult::TemporaryFileFailure:
            return Core3DAssetLoadResultTemporaryFileFailure;
        case AssetImportResult::Busy:
            return Core3DAssetLoadResultBusy;
        case AssetImportResult::UnsupportedVersion:
            return Core3DAssetLoadResultUnsupportedVersion;
        case AssetImportResult::InternalFailure:
            return Core3DAssetLoadResultInternalFailure;
    }
}

void CompleteAssetLoadOnMain(void (^completion)(Core3DAssetLoadResult),
                             Core3DAssetLoadResult result) {
    dispatch_async(dispatch_get_main_queue(), ^{
        completion(result);
    });
}

#ifdef DEBUG
bool TryFindExactDebugPresentation(
	const std::shared_ptr<Core3DViewer>& theViewer,
	NSString *theEntityIdentifier,
	Handle(AIS_Shape)& thePresentation) noexcept
{
	thePresentation.Nullify();
	if (theViewer == nullptr || theViewer->AisContext().IsNull()
		|| theViewer->getDocument().IsNull()
		|| ![theEntityIdentifier isKindOfClass:NSString.class]
		|| theEntityIdentifier.length == 0
		|| theEntityIdentifier.UTF8String == nullptr) {
		return false;
	}
	try {
		OCC_CATCH_SIGNALS
		const std::string anIdentifier(theEntityIdentifier.UTF8String);
		AIS_ListOfInteractive aDisplayed;
		theViewer->AisContext()->DisplayedObjects(
			AIS_KOI_Shape, -1, aDisplayed);
		for (AIS_ListIteratorOfListOfInteractive anObject(aDisplayed);
			 anObject.More(); anObject.Next()) {
			const Handle(AIS_InteractiveObject)& anInteractive =
				anObject.Value();
			const TDF_Label aLabel =
				theViewer->getDocument()->ShapeLabel(anInteractive);
			if (aLabel.IsNull()
				|| theViewer->getDocument()->EntityIdentifierForLabel(aLabel)
					!= anIdentifier) {
				continue;
			}
			const Handle(AIS_Shape) aCandidate =
				Handle(AIS_Shape)::DownCast(anInteractive);
			const OcctGeometryRepresentation aRepresentation =
				theViewer->getDocument()
					->GeometryRepresentationForLabel(aLabel);
			if (!thePresentation.IsNull() || aCandidate.IsNull()
				|| aCandidate->Shape().IsNull()
				|| !theViewer->getDocument()
					->IsPresentationEditable(aCandidate)
				|| (aRepresentation != OcctGeometryRepresentation::BRep
					&& aRepresentation
						!= OcctGeometryRepresentation::LegacyUnknown)) {
				return false;
			}
			thePresentation = aCandidate;
		}
		return !thePresentation.IsNull();
	} catch (...) {
		thePresentation.Nullify();
		return false;
	}
}

//! AIS normally derives DetectedInteractive from the same virtual Selectable
//! read exposed by DetectedOwner, so a stable mismatch cannot be constructed.
//! Alternating the two reads creates the exact adversarial boundary guarded by
//! the production snapshot builder without changing production AIS behavior.
class DebugAlternatingSelectableBRepOwner final
	: public StdSelect_BRepOwner {
public:
	DebugAlternatingSelectableBRepOwner(
		const TopoDS_Shape& theShape,
		const Handle(AIS_Shape)& theReportedPresentation,
		const Handle(AIS_Shape)& theForeignPresentation)
	: StdSelect_BRepOwner(
		theShape, theReportedPresentation, 0, Standard_True),
	  myReportedPresentation(theReportedPresentation),
	  myForeignPresentation(theForeignPresentation) {}

	Handle(SelectMgr_SelectableObject) Selectable() const override
	{
		return mySelectableReadCount++ == 0
			? myReportedPresentation
			: myForeignPresentation;
	}

private:
	Handle(SelectMgr_SelectableObject) myReportedPresentation;
	Handle(SelectMgr_SelectableObject) myForeignPresentation;
	mutable Standard_Size mySelectableReadCount = 0;
};
#endif

} // namespace

// Allocate this receipt BEFORE exclusive creation. After open succeeds, publish
// it to the exact request and call markCreatedWithDescriptor immediately. Never
// delete a temporary path directly once this receipt owns it.
@interface Core3DQueuedPrivateFile : NSObject
@property(nonatomic, strong, readonly) NSURL *URL;
- (instancetype)initWithURL:(NSURL *)URL;
- (BOOL)markCreatedWithDescriptor:(int)descriptor;
- (BOOL)sealWithDescriptor:(int)descriptor;
- (BOOL)hasExactSealedIdentity;
- (BOOL)removeOwnedFile;
@end

@implementation Core3DQueuedPrivateFile {
    BOOL _created;
    BOOL _identityKnown;
    BOOL _sealed;
    dev_t _device;
    ino_t _inode;
    struct stat _sealedStatus;
}
- (instancetype)initWithURL:(NSURL *)URL {
    if (!URL.isFileURL || URL.fileSystemRepresentation == nullptr) return nil;
    self = [super init];
    if (self) _URL = [URL copy];
    return self;
}
- (BOOL)markCreatedWithDescriptor:(int)descriptor {
    if (_created) return NO;
    _created = YES; // Even fstat failure now requires retained cleanup ownership.
    struct stat status = {};
    if (descriptor < 0 || ::fstat(descriptor, &status) != 0
        || !S_ISREG(status.st_mode) || status.st_nlink != 1
        || status.st_uid != ::geteuid()) return NO;
    _device = status.st_dev;
    _inode = status.st_ino;
    _identityKnown = YES;
    return YES;
}
- (BOOL)sealWithDescriptor:(int)descriptor {
    if (!_created || !_identityKnown || _sealed) return NO;
    struct stat status = {};
    if (descriptor < 0 || ::fstat(descriptor, &status) != 0
        || !S_ISREG(status.st_mode) || status.st_nlink != 1
        || status.st_uid != ::geteuid() || status.st_dev != _device
        || status.st_ino != _inode) return NO;
    _sealedStatus = status;
    _sealed = YES;
    return [self hasExactSealedIdentity];
}
- (BOOL)hasExactSealedIdentity {
    if (!_created || !_identityKnown || !_sealed) return NO;
    struct stat current = {};
    return ::lstat(_URL.fileSystemRepresentation, &current) == 0
        && FileIdentity(current) == FileIdentity(_sealedStatus);
}
- (BOOL)removeOwnedFile {
    if (!_created) return YES; // Failed exclusive open never owns that pathname.
    const char *path = _URL.fileSystemRepresentation;
    if (path == nullptr) return NO;
    struct stat current = {};
    if (::lstat(path, &current) != 0) return errno == ENOENT;
    if (!_identityKnown || current.st_dev != _device || current.st_ino != _inode
        || !S_ISREG(current.st_mode) || current.st_nlink != 1
        || current.st_uid != ::geteuid()) return NO;
    if (::unlink(path) != 0 && errno != ENOENT) return NO;
    struct stat remaining = {};
    return ::lstat(path, &remaining) != 0 && errno == ENOENT;
}
@end

namespace {
// Private staging grants no native edit authority. The caller retains the exact
// queued request through adoption and verified cleanup.
class QueuedStageDescriptor final {
public:
    explicit QueuedStageDescriptor(int value) noexcept : value_(value) {}
    ~QueuedStageDescriptor() { if (value_ >= 0) ::close(value_); }
    QueuedStageDescriptor(const QueuedStageDescriptor&) = delete;
    QueuedStageDescriptor& operator=(const QueuedStageDescriptor&) = delete;
    int get() const noexcept { return value_; }
    int release() noexcept { int value = value_; value_ = -1; return value; }
    bool closeChecked() noexcept {
        const int value = release();
        return value >= 0 && ::close(value) == 0;
    }
private:
    int value_;
};

class QueuedStageDirectory final {
public:
    explicit QueuedStageDirectory(DIR *value) noexcept : value_(value) {}
    ~QueuedStageDirectory() { if (value_) ::closedir(value_); }
    QueuedStageDirectory(const QueuedStageDirectory&) = delete;
    QueuedStageDirectory& operator=(const QueuedStageDirectory&) = delete;
    DIR *get() const noexcept { return value_; }
private:
    DIR *value_;
};

// Bounded private staging for the legacy directory bundle. This does not
// authenticate a manifest or grant access to a public native-file importer.
// Ambiguous bundles with multiple .asset entries are rejected, instead of
// selecting whichever entry happens to be returned first by the filesystem.
// IMPORTANT: stagedURL may be nonnil on FAILURE. Once a file is created, its
// exact request completion owner owns cleanup on every outcome. Do not reuse
// the existing StageVerifiedAssetFile early-return pattern that drops the URL.
// Native queued ownership cannot settle until cleanup has actually succeeded.
Core3DAssetLoadResult StageLegacyAssetBundle(
    NSURL *frozenBundleURL, Core3DQueuedPrivateFile **stagedURL) {
    constexpr unsigned long long maximumBytes = 256ull * 1024ull * 1024ull;
    constexpr size_t maximumDirectoryEntries = 4096;
    constexpr char magic[] = "BINFILE";
    if (stagedURL == nullptr) return Core3DAssetLoadResultInternalFailure;
    *stagedURL = nil;
    if (!frozenBundleURL.isFileURL || frozenBundleURL.fileSystemRepresentation == nullptr)
        return Core3DAssetLoadResultInvalidData;

    const BOOL scoped = [frozenBundleURL startAccessingSecurityScopedResource];
    @try {
        try {
            const char *directoryPath = frozenBundleURL.fileSystemRepresentation;
            struct stat directoryBefore = {};
            if (::lstat(directoryPath, &directoryBefore) != 0
                || !S_ISDIR(directoryBefore.st_mode))
                return Core3DAssetLoadResultInvalidData;
            QueuedStageDescriptor directoryFD(::open(directoryPath,
                O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW));
            struct stat directoryOpened = {};
            if (directoryFD.get() < 0
                || ::fstat(directoryFD.get(), &directoryOpened) != 0
                || !(FileIdentity(directoryBefore) == FileIdentity(directoryOpened)))
                return Core3DAssetLoadResultInvalidData;
            DIR *rawDirectory = ::fdopendir(directoryFD.get());
            if (rawDirectory == nullptr)
                return Core3DAssetLoadResultTemporaryFileFailure;
            (void)directoryFD.release(); // DIR now owns the descriptor.
            QueuedStageDirectory directory(rawDirectory);
            const int parentFD = ::dirfd(directory.get());
            std::string assetName;
            size_t entries = 0;
            for (;;) {
                errno = 0;
                const dirent *entry = ::readdir(directory.get());
                if (entry == nullptr) {
                    if (errno != 0) return Core3DAssetLoadResultTemporaryFileFailure;
                    break;
                }
                if (++entries > maximumDirectoryEntries)
                    return Core3DAssetLoadResultInvalidData;
                const std::string name(entry->d_name);
                if (name.size() < 6 || name.compare(name.size() - 6, 6, ".asset") != 0)
                    continue;
                if (!assetName.empty()) return Core3DAssetLoadResultInvalidData;
                assetName = name;
            }
            if (assetName.empty()) return Core3DAssetLoadResultInvalidData;
            struct stat before = {};
            if (::fstatat(parentFD, assetName.c_str(), &before, AT_SYMLINK_NOFOLLOW) != 0
                || !S_ISREG(before.st_mode) || before.st_nlink != 1
                || before.st_size < static_cast<off_t>(sizeof(magic) - 1)
                || static_cast<unsigned long long>(before.st_size) > maximumBytes)
                return Core3DAssetLoadResultInvalidData;
            QueuedStageDescriptor source(::openat(parentFD, assetName.c_str(),
                O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK));
            // O_NONBLOCK is inert for regular files. If the entry becomes a
            // FIFO between fstatat and openat, it prevents an unbounded wait;
            // the descriptor identity/mode check below rejects that change.
            struct stat opened = {};
            if (source.get() < 0 || ::fstat(source.get(), &opened) != 0
                || !(FileIdentity(before) == FileIdentity(opened)))
                return Core3DAssetLoadResultInvalidData;

            NSURL *temporaryURL = [NSFileManager.defaultManager.temporaryDirectory
                URLByAppendingPathComponent:[NSString stringWithFormat:
                    @"%@.queued-legacy.cbf", NSUUID.UUID.UUIDString]];
            const char *temporaryPath = temporaryURL.fileSystemRepresentation;
            if (temporaryPath == nullptr)
                return Core3DAssetLoadResultTemporaryFileFailure;
            Core3DQueuedPrivateFile *receipt = [[Core3DQueuedPrivateFile alloc] initWithURL:temporaryURL];
            if (receipt == nil) return Core3DAssetLoadResultTemporaryFileFailure;
            QueuedStageDescriptor destination(::open(temporaryPath,
                O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
                S_IRUSR | S_IWUSR));
            if (destination.get() < 0)
                return Core3DAssetLoadResultTemporaryFileFailure;
            // Publish private-file ownership before any operation can fail.
            // This function closes descriptors; the completion owner deletes
            // this file after either adoption or a failed preparation.
            *stagedURL = receipt;
            if (![receipt markCreatedWithDescriptor:destination.get()])
                return Core3DAssetLoadResultTemporaryFileFailure;
            std::array<unsigned char, 64 * 1024> buffer = {};
            std::array<unsigned char, sizeof(magic) - 1> prefix = {};
            size_t prefixCount = 0;
            unsigned long long copied = 0;
            const auto expected = static_cast<unsigned long long>(opened.st_size);
            for (;;) {
                const ssize_t count = ::read(source.get(), buffer.data(), buffer.size());
                if (count < 0 && errno == EINTR) continue;
                if (count < 0) return Core3DAssetLoadResultTemporaryFileFailure;
                if (count == 0) break;
                if (static_cast<unsigned long long>(count) > expected - copied)
                    return Core3DAssetLoadResultInvalidData;
                const size_t prefixBytes = std::min(prefix.size() - prefixCount,
                    static_cast<size_t>(count));
                std::copy_n(buffer.data(), prefixBytes, prefix.data() + prefixCount);
                prefixCount += prefixBytes;
                copied += static_cast<unsigned long long>(count);
                ssize_t written = 0;
                while (written < count) {
                    const ssize_t size = ::write(destination.get(), buffer.data() + written,
                        static_cast<size_t>(count - written));
                    if (size < 0 && errno == EINTR) continue;
                    if (size <= 0) return Core3DAssetLoadResultTemporaryFileFailure;
                    written += size;
                }
            }
            struct stat after = {}, pathAfter = {}, directoryAfter = {}, directoryPathAfter = {};
            if (copied != expected || prefixCount != prefix.size()
                || std::memcmp(prefix.data(), magic, prefix.size()) != 0
                || ::fstat(source.get(), &after) != 0
                || !(FileIdentity(opened) == FileIdentity(after))
                || ::fstatat(parentFD, assetName.c_str(), &pathAfter, AT_SYMLINK_NOFOLLOW) != 0
                || !(FileIdentity(opened) == FileIdentity(pathAfter))
                || ::fstat(parentFD, &directoryAfter) != 0
                || !(FileIdentity(directoryOpened) == FileIdentity(directoryAfter))
                || ::lstat(directoryPath, &directoryPathAfter) != 0
                || !(FileIdentity(directoryOpened) == FileIdentity(directoryPathAfter)))
                return Core3DAssetLoadResultInvalidData;
            if (![receipt sealWithDescriptor:destination.get()])
                return Core3DAssetLoadResultTemporaryFileFailure;
            if (!destination.closeChecked())
                return Core3DAssetLoadResultTemporaryFileFailure;
            return Core3DAssetLoadResultSuccess;
        } catch (...) {
            return Core3DAssetLoadResultInternalFailure;
        }
    } @catch (NSException *exception) {
        return Core3DAssetLoadResultInternalFailure;
    } @finally {
        if (scoped) [frozenBundleURL stopAccessingSecurityScopedResource];
    }
}

// An existing private file transfers to the completion owner even on failure.
// That owner must verify cleanup before ending queued ownership.
Core3DAssetLoadResult StageQueuedVerifiedAssetFile(
    NSURL *sourceURL, const unsigned long long expectedByteCount,
    NSString *expectedSHA256, Core3DQueuedPrivateFile **stagedURL) {
    constexpr unsigned long long maximumBytes = 256ull * 1024ull * 1024ull;
    if (stagedURL == nullptr) return Core3DAssetLoadResultInternalFailure;
    *stagedURL = nil;
    std::array<unsigned char, CC_SHA256_DIGEST_LENGTH> expectedDigest = {};
    if (!sourceURL.isFileURL || sourceURL.fileSystemRepresentation == nullptr
        || expectedByteCount < sizeof(kCbfMagic) - 1
        || expectedByteCount > maximumBytes
        || !DecodeSHA256(expectedSHA256, expectedDigest))
        return Core3DAssetLoadResultInvalidData;

    const BOOL scoped = [sourceURL startAccessingSecurityScopedResource];
    @try {
        try {
            const char *sourcePath = sourceURL.fileSystemRepresentation;
            struct stat before = {}, opened = {};
            if (::lstat(sourcePath, &before) != 0 || !S_ISREG(before.st_mode)
                || before.st_nlink != 1 || before.st_size < 0
                || static_cast<unsigned long long>(before.st_size) != expectedByteCount)
                return Core3DAssetLoadResultInvalidData;
            QueuedStageDescriptor source(::open(sourcePath,
                O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK));
            // Reject replacement with a FIFO/device before reading any bytes;
            // O_NONBLOCK prevents a FIFO open race from hanging the worker.
            if (source.get() < 0 || ::fstat(source.get(), &opened) != 0
                || !(FileIdentity(before) == FileIdentity(opened)))
                return Core3DAssetLoadResultInvalidData;

            NSURL *temporaryURL = [NSFileManager.defaultManager.temporaryDirectory
                URLByAppendingPathComponent:[NSString stringWithFormat:
                    @"%@.queued-verified.cbf", NSUUID.UUID.UUIDString]];
            const char *temporaryPath = temporaryURL.fileSystemRepresentation;
            if (temporaryPath == nullptr)
                return Core3DAssetLoadResultTemporaryFileFailure;
            Core3DQueuedPrivateFile *receipt = [[Core3DQueuedPrivateFile alloc] initWithURL:temporaryURL];
            if (receipt == nil) return Core3DAssetLoadResultTemporaryFileFailure;
            QueuedStageDescriptor destination(::open(temporaryPath,
                O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
                S_IRUSR | S_IWUSR));
            if (destination.get() < 0)
                return Core3DAssetLoadResultTemporaryFileFailure;
            *stagedURL = receipt;
            if (![receipt markCreatedWithDescriptor:destination.get()])
                return Core3DAssetLoadResultTemporaryFileFailure;

            CC_SHA256_CTX hash = {};
            if (CC_SHA256_Init(&hash) != 1)
                return Core3DAssetLoadResultInternalFailure;
            std::array<unsigned char, 64 * 1024> buffer = {};
            std::array<unsigned char, sizeof(kCbfMagic) - 1> prefix = {};
            size_t prefixCount = 0;
            unsigned long long copied = 0;
            for (;;) {
                const ssize_t count = ::read(source.get(), buffer.data(), buffer.size());
                if (count < 0 && errno == EINTR) continue;
                if (count < 0) return Core3DAssetLoadResultTemporaryFileFailure;
                if (count == 0) break;
                if (static_cast<unsigned long long>(count) > expectedByteCount - copied)
                    return Core3DAssetLoadResultInvalidData;
                if (CC_SHA256_Update(&hash, buffer.data(), static_cast<CC_LONG>(count)) != 1)
                    return Core3DAssetLoadResultInternalFailure;
                const size_t prefixBytes = std::min(prefix.size() - prefixCount,
                    static_cast<size_t>(count));
                std::copy_n(buffer.data(), prefixBytes, prefix.data() + prefixCount);
                prefixCount += prefixBytes;
                copied += static_cast<unsigned long long>(count);
                ssize_t written = 0;
                while (written < count) {
                    const ssize_t amount = ::write(destination.get(), buffer.data() + written,
                        static_cast<size_t>(count - written));
                    if (amount < 0 && errno == EINTR) continue;
                    if (amount <= 0) return Core3DAssetLoadResultTemporaryFileFailure;
                    written += amount;
                }
            }
            struct stat after = {}, pathAfter = {};
            std::array<unsigned char, CC_SHA256_DIGEST_LENGTH> actualDigest = {};
            if (copied != expectedByteCount || prefixCount != prefix.size()
                || std::memcmp(prefix.data(), kCbfMagic, prefix.size()) != 0
                || CC_SHA256_Final(actualDigest.data(), &hash) != 1
                || actualDigest != expectedDigest
                || ::fstat(source.get(), &after) != 0
                || !(FileIdentity(opened) == FileIdentity(after))
                || ::lstat(sourcePath, &pathAfter) != 0
                || !(FileIdentity(opened) == FileIdentity(pathAfter)))
                return Core3DAssetLoadResultInvalidData;
            if (![receipt sealWithDescriptor:destination.get()])
                return Core3DAssetLoadResultTemporaryFileFailure;
            if (!destination.closeChecked())
                return Core3DAssetLoadResultTemporaryFileFailure;
            return Core3DAssetLoadResultSuccess;
        } catch (...) {
            return Core3DAssetLoadResultInternalFailure;
        }
    } @catch (NSException *exception) {
        return Core3DAssetLoadResultInternalFailure;
    } @finally {
        if (scoped) [sourceURL stopAccessingSecurityScopedResource];
    }
}

// Physical input freezing precedes reservation.
// After file creation, every return transfers the path to the completion owner.
Core3DAssetLoadResult StageQueuedAssetData(NSData *frozenData, Core3DQueuedPrivateFile **stagedURL) {
    if (stagedURL == nullptr) return Core3DAssetLoadResultInternalFailure;
    *stagedURL = nil;
    if (frozenData.length > 256ull * 1024ull * 1024ull || !HasCbfMagic(frozenData))
        return Core3DAssetLoadResultInvalidData;
    @try {
        NSURL *url = [NSFileManager.defaultManager.temporaryDirectory
            URLByAppendingPathComponent:[NSString stringWithFormat:
                @"%@.queued-data.cbf", NSUUID.UUID.UUIDString]];
        const char *path = url.fileSystemRepresentation;
        if (path == nullptr) return Core3DAssetLoadResultTemporaryFileFailure;
        Core3DQueuedPrivateFile *receipt = [[Core3DQueuedPrivateFile alloc] initWithURL:url];
        if (receipt == nil) return Core3DAssetLoadResultTemporaryFileFailure;
        QueuedStageDescriptor destination(::open(path,
            O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
            S_IRUSR | S_IWUSR));
        if (destination.get() < 0) return Core3DAssetLoadResultTemporaryFileFailure;
        *stagedURL = receipt;
        if (![receipt markCreatedWithDescriptor:destination.get()])
            return Core3DAssetLoadResultTemporaryFileFailure;
        const unsigned char *bytes = static_cast<const unsigned char *>(frozenData.bytes);
        NSUInteger written = 0;
        while (written < frozenData.length) {
            const size_t count = std::min<NSUInteger>(64 * 1024, frozenData.length - written);
            const ssize_t amount = ::write(destination.get(), bytes + written, count);
            if (amount < 0 && errno == EINTR) continue;
            if (amount <= 0) return Core3DAssetLoadResultTemporaryFileFailure;
            written += static_cast<NSUInteger>(amount);
        }
        if (![receipt sealWithDescriptor:destination.get()])
            return Core3DAssetLoadResultTemporaryFileFailure;
        return destination.closeChecked() ? Core3DAssetLoadResultSuccess
            : Core3DAssetLoadResultTemporaryFileFailure;
    } @catch (NSException *exception) {
        return Core3DAssetLoadResultInternalFailure;
    }
}

Core3DAssetLoadResult StageQueuedAssetInput(Core3DQueuedAssetInput *input, Core3DQueuedPrivateFile **stagedURL) {
    if (stagedURL == nullptr) return Core3DAssetLoadResultInternalFailure;
    *stagedURL = nil;
    if (input == nil) return Core3DAssetLoadResultInvalidData;
    switch (input.kind) {
        case Core3DQueuedAssetInputKindData:
            return StageQueuedAssetData(input.data, stagedURL);
        case Core3DQueuedAssetInputKindVerifiedFile:
            return StageQueuedVerifiedAssetFile(input.fileURL, input.expectedByteCount,
                input.expectedSHA256, stagedURL);
        case Core3DQueuedAssetInputKindBundle:
            return StageLegacyAssetBundle(input.fileURL, stagedURL);
    }
    return Core3DAssetLoadResultInvalidData;
}

} // namespace

// This carrier retains native ownership and the rendering lease through cleanup.
// It alone does not authorize a public AI operation or load dispatch.
@interface Core3DQueuedAssetLoadOwner : NSObject
@property(nonatomic, strong, readonly) Core3DQueuedAssetInput *input;
// Only the actual main-thread GL/controller admission path may call this, after
// rejecting unsettled controller slots and checking the accepted request.
+ (instancetype)reserveInput:(Core3DQueuedAssetInput *)input
                glController:(GLViewController *)controller;
- (GLViewController *)renderingControllerOnMain;
- (BOOL)ownsGLController:(GLViewController *)controller;
- (BOOL)claimPrivateStartForGLController:(GLViewController *)controller;
- (void)abandonForGLController:(GLViewController *)controller;
- (Core3DAssetLoadResult)adoptPrivateFile:(NSURL *)file
                         glController:(GLViewController *)controller;
// Main only. A cleanup failure MUST retain this owner and its private file.
// After YES, GL removes its exact owner slot, then calls releaseRenderingLease.
- (BOOL)settleNativeAfterPrivateCleanup:(BOOL)cleanupSucceeded
                         glController:(GLViewController *)controller;
- (BOOL)releaseRenderingLeaseForGLController:(GLViewController *)controller;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DQueuedAssetLoadOwner () {
    GLViewController *_renderingLease;
    __weak id<GLViewControllerProtocol> _origin;
    BOOL _hadOrigin;
    std::shared_ptr<core3d::Core3DViewer> _nativeViewer;
    std::shared_ptr<core3d::QueuedAssetLoadWork> _nativeWork;
    BOOL _privateStarted;
    BOOL _adoptionAttempted;
    BOOL _abandoned;
    BOOL _nativeSettled;
}
- (instancetype)initWithInput:(Core3DQueuedAssetInput *)input
                   controller:(GLViewController *)controller
                       viewer:(const std::shared_ptr<core3d::Core3DViewer>&)viewer
                         work:(const std::shared_ptr<core3d::QueuedAssetLoadWork>&)work;
@end

@implementation Core3DQueuedAssetLoadOwner
- (instancetype)initWithInput:(Core3DQueuedAssetInput *)input
                   controller:(GLViewController *)controller
                       viewer:(const std::shared_ptr<core3d::Core3DViewer>&)viewer
                         work:(const std::shared_ptr<core3d::QueuedAssetLoadWork>&)work {
    self = [super init];
    if (self) {
        _input = input;
        _renderingLease = controller;
        _origin = controller.delegate;
        _hadOrigin = controller.delegate != nil;
        _nativeViewer = viewer;
        _nativeWork = work;
    }
    return self;
}
+ (instancetype)reserveInput:(Core3DQueuedAssetInput *)input
                glController:(GLViewController *)controller {
    if (![NSThread isMainThread] || input == nil || controller == nil) return nil;
    const auto viewer = controller.viewer;
    if (!viewer) return nil;
    auto work = viewer->beginQueuedAssetLoad();
    if (!work) return nil;
    Core3DQueuedAssetLoadOwner *owner = [[self alloc] initWithInput:input
        controller:controller viewer:viewer work:work];
    if (owner == nil) {
        // No private work was submitted and no replacement was promoted.
        (void)viewer->finishQueuedAssetLoadPrivateWork(work, true);
    }
    return owner;
}
- (GLViewController *)renderingControllerOnMain {
    return [NSThread isMainThread] ? _renderingLease : nil;
}
- (BOOL)ownsGLController:(GLViewController *)controller {
    return [NSThread isMainThread] && controller != nil
        && _renderingLease == controller;
}
- (BOOL)claimPrivateStartForGLController:(GLViewController *)controller {
    if (![self ownsGLController:controller] || _privateStarted || _abandoned
        || _nativeSettled || !_nativeViewer || !_nativeWork
        || !_nativeViewer->ownsQueuedAssetLoad(_nativeWork)) return NO;
    _privateStarted = YES;
    return YES;
}
- (void)abandonForGLController:(GLViewController *)controller {
    if ([self ownsGLController:controller]) _abandoned = YES;
    // This only blocks adoption. It never drops private files or native work.
}
- (Core3DAssetLoadResult)adoptPrivateFile:(NSURL *)file
                         glController:(GLViewController *)controller {
    if (![self ownsGLController:controller] || !_privateStarted || _abandoned
        || _adoptionAttempted || _nativeSettled || !_nativeViewer || !_nativeWork
        || !_nativeViewer->canAdoptQueuedAssetLoad(_nativeWork))
        return Core3DAssetLoadResultBusy;
    id<GLViewControllerProtocol> origin = _origin;
    if (_hadOrigin && (origin == nil || controller.delegate != origin))
        return Core3DAssetLoadResultBusy;
    if ([origin isKindOfClass:Core3DViewController.class]
        && ((Core3DViewController *)origin).glController != controller)
        return Core3DAssetLoadResultBusy;
    if (!file.isFileURL || file.fileSystemRepresentation == nullptr)
        return Core3DAssetLoadResultInvalidData;
    _adoptionAttempted = YES;
    try {
        // Exact-owner native overload validates privately and promotes the
        // queued reservation atomically into retained replacement ownership.
        return AssetLoadResultFromImportResult(_nativeViewer->ImportCbf(
            file.fileSystemRepresentation, _nativeWork));
    } catch (...) {
        return Core3DAssetLoadResultInternalFailure;
    }
}
- (BOOL)settleNativeAfterPrivateCleanup:(BOOL)cleanupSucceeded
                         glController:(GLViewController *)controller {
    if (![self ownsGLController:controller] || _nativeSettled
        || !cleanupSucceeded || !_nativeViewer || !_nativeWork) return NO;
    if (!_nativeViewer->finishQueuedAssetLoadPrivateWork(_nativeWork, true))
        return NO;
    // These explicit main-thread resets matter even when a worker still owns
    // the Objective-C carrier. Its later destruction must own no OCCT handles.
    _nativeWork.reset();
    _nativeViewer.reset();
    _nativeSettled = YES;
    return YES;
}
- (BOOL)releaseRenderingLeaseForGLController:(GLViewController *)controller {
    if (![self ownsGLController:controller] || !_nativeSettled
        || _nativeWork || _nativeViewer) return NO;
    // The caller must retain a main-thread local controller while clearing its
    // exact GL request slot and this lease, breaking the temporary cycle.
    _renderingLease = nil;
    return YES;
}
- (void)dealloc {
    NSCAssert(!_nativeWork && !_nativeViewer && _renderingLease == nil,
        @"Queued native work and rendering lease must settle on main before release");
}
@end

@interface GLViewController () <UIGestureRecognizerDelegate>
- (void)endActiveRenderingInteractions;
- (void)endRawPrimaryInteractionIfNeededCancelled:(BOOL)cancelled;
- (void)checkSelections;
- (void)updateSelectedEdgeAppearance;
- (void)updateSelectedSurfaceAppearance;
- (void)clearSelectedFaceWash;
- (void)clearCommittedBoundaries;
- (void)updateCommittedBoundaries;
- (void)updateSplitBoundaries:(const Handle(AIS_Shape)&)presentation;
- (void)performNativeHistoryDirection:(NativeHistoryDirection)direction;
- (void)addCube:(UIBarButtonItem *)sender;
- (BOOL)restoreBooleanActionForRetainedGizmoType:(PrimitiveGizmoType)type;
- (BOOL)retireBooleanActionForGizmoType:(PrimitiveGizmoType)type;
- (BOOL)debugSelectEdgeTopologyIndicesWithEntityIdentifier:
			(NSString *)entityIdentifier
	edgeTopologyIndices:(NSArray<NSNumber *> *)edgeTopologyIndices
	reverseFirstEdge:(BOOL)reverseFirstEdge
	foreignEntityIdentifier:(NSString *_Nullable)foreignEntityIdentifier;
- (BOOL)debugPublishDetectedTopologyWithEntityIdentifier:
			(NSString *)entityIdentifier
	topologyIndex:(NSUInteger)topologyIndex
	isFace:(BOOL)isFace
	foreignEntityIdentifier:(NSString *_Nullable)foreignEntityIdentifier;
@end

@interface Core3DViewController (VisibleRawR2BevelRouting)
- (BOOL)core3d_hasVisibleRawR2Bevel;
- (BOOL)core3d_canBeginVisibleRawR2Bevel;
- (BOOL)core3d_beginVisibleRawR2Bevel;
- (void)core3d_setVisibleRawR2BevelValue:(double)localValue;
- (BOOL)core3d_applyVisibleRawR2Bevel;
- (BOOL)core3d_cancelVisibleRawR2Bevel;
- (Core3DModelingPreviewStatus)core3d_visibleRawR2BevelStatus;
- (double)core3d_visibleRawR2BevelMetersPerUnit;
- (Core3DRoundFailureObservation)
    core3d_visibleRawR2RoundFailureObservation;
@end

static Core3DViewController *Core3DVisibleRawR2BevelOwner(
    GLViewController *controller) {
    id delegate = controller.delegate;
    return [delegate isKindOfClass:Core3DViewController.class]
        ? static_cast<Core3DViewController *>(delegate) : nil;
}

@implementation GLViewController {
    dispatch_queue_t _assetDataQueue;
    NSUInteger _pendingAssetDataCount;
    Core3DQueuedAssetLoadOwner *_queuedAssetOwner;
    void (^_queuedAssetCompletion)(Core3DAssetLoadResult);
    void (^_queuedAssetProgress)(BOOL);
    Core3DQueuedPrivateFile *_queuedPrivateFile;
    Core3DAssetLoadResult _queuedAssetResult;
    BOOL _queuedCleanupInFlight;
    BOOL _queuedCleanupPending;
    BOOL _queuedPreparationReceived;
    UIView *_queuedInteractionView;
    BOOL _queuedViewWasInteractive;
#ifdef DEBUG
    BOOL _debugPauseQueuedAdoption;
    void (^_debugResumeQueuedAdoption)(void);
    Core3DQueuedPrivateFile *_debugStagedPrivateFile;
    NSUInteger _debugQueuedCleanupFailures;
    NSMutableArray<NSString *> *_debugNativeFaceTapTrace;
#endif
    BOOL _cancelTouches;
    BOOL _didSetupViewer;
    UIColor *_selectedEdgeColor;
    CGFloat _selectedEdgeWidthPoints;
    CGFloat _selectedEdgeWidthPixels;
    BOOL _selectedEdgeAppearanceDirty;
    Handle(Prs3d_Drawer) _ordinaryLocalSelectionStyle;
    Handle(Prs3d_Drawer) _selectedEdgeStyle;
    UIColor *_selectedSurfaceColor;
    CGFloat _selectedSurfaceWidthPoints;
    CGFloat _selectedSurfaceWidthPixels;
    CGFloat _selectedFaceWashOpacity;
    BOOL _selectedSurfaceAppearanceDirty;
    BOOL _selectedSurfaceWasFace;
    Handle(Graphic3d_Structure) _selectedFaceWash;
    std::vector<TopoDS_Shape> _selectedWashFaces;
    std::vector<NativeContourGeometry> _selectedContourGeometry;
    Standard_Size _selectedContourViewState;
    Standard_Size _selectedContourProjectionState;
    CGSize _selectedContourDrawable;
    const void *_selectedContourDocument;

    std::vector<Handle(Graphic3d_Structure)> _committedBoundaries;
    std::string _committedBoundaryDocument;
    const void *_committedBoundaryDocumentOwner;
    BOOL _committedBoundariesDirty;
    Handle(AIS_Shape) _splitBoundaryPresentation;
    BOOL _rawTouchRendering;
    BOOL _rawTouchWasCancelled;
    NSMutableSet<UITouch *> *_rawViewportTouches;
    BOOL _pinchRendering;
    BOOL _panRendering;
    CGPoint _pinchPreviousTouch[2];
    UITapGestureRecognizer *_tapRecognizer;
    BOOL _suppressTapSelectionForManipulatorInteraction;
    BOOL _visibleRawR2BevelOwnsChamferTool;
}

- (void)dealloc {
    [[NSNotificationCenter defaultCenter] removeObserver:self];
    // The block shares this single handle so its final reference is released
    // before the viewer destroys the structure manager and GL resources.
    __block Handle(Graphic3d_Structure) selectedWash = _selectedFaceWash;
    __block auto boundaries = std::move(_committedBoundaries);
    _splitBoundaryPresentation.Nullify();
    _selectedFaceWash.Nullify();
    _selectedWashFaces.clear();
    if (_viewer != nullptr) {
        GLView *view = self.isViewLoaded ? [self viewportView] : nil;
        if (view != nil) {
            const std::shared_ptr<Core3DViewer> viewer = _viewer;
            const BOOL didRelease = [view performWithRenderingContext:^{
                if (!selectedWash.IsNull()) {
                    RemoveNativeSelectionStructure(selectedWash);
                }
                for (auto& structure : boundaries) RemoveNativeSelectionStructure(structure);
                boundaries.clear();
                viewer->release();
            }];
            if (!didRelease) {
                NSLog(@"Core3D rendering context unavailable during teardown; using safe fallback.");
                EAGLContext *previousContext = EAGLContext.currentContext;
                [EAGLContext setCurrentContext:nil];
                if (!selectedWash.IsNull()) {
                    RemoveNativeSelectionStructure(selectedWash);
                }
                for (auto& structure : boundaries) RemoveNativeSelectionStructure(structure);
                boundaries.clear();
                viewer->release();
                [EAGLContext setCurrentContext:previousContext];
            }
        } else {
            // No GL view means InitViewer never created graphics resources.
            if (!selectedWash.IsNull()) {
                RemoveNativeSelectionStructure(selectedWash);
            }
            for (auto& structure : boundaries) RemoveNativeSelectionStructure(structure);
            boundaries.clear();
            _viewer->release();
        }
    }
    _viewer = nullptr;
    NSLog(@"~GLViewController");
}

// =======================================================================
// function : init
// purpose  :
// =======================================================================
- (id) init
{
    self = [super init];

    if (self) {
        _viewer = std::make_shared<Core3DViewer>();
        _committedBoundariesDirty = YES;
        _assetDataQueue = dispatch_queue_create("com.shapeyard.sync", NULL);
		_rawViewportTouches = [NSMutableSet set];
		_isConstructorMode = false;
        [[NSNotificationCenter defaultCenter]
            addObserver:self
            selector:@selector(documentDidChange:)
            name:@"OcctDocumentChanges"
            object:nil];
        [[NSNotificationCenter defaultCenter]
            addObserver:self
            selector:@selector(applicationWillResignActive:)
            name:UIApplicationWillResignActiveNotification
            object:nil];
        [[NSNotificationCenter defaultCenter]
            addObserver:self
            selector:@selector(sceneWillDeactivate:)
            name:UISceneWillDeactivateNotification
            object:nil];
    }

    return self;
}

-(std::shared_ptr<core3d::Core3DViewer>) viewer {
    return _viewer;
}

- (GLView *)viewportView
{
    return [self.view isKindOfClass:GLView.class] ? (GLView *)self.view : nil;
}

- (CGFloat)drawableScale
{
    GLView *view = [self viewportView];
    if (view.bounds.size.width > 0.0 && view.drawableSize.width > 0.0) {
        return view.drawableSize.width / view.bounds.size.width;
    }
    return self.view.window.screen.scale ?: UIScreen.mainScreen.scale;
}

- (CGPoint)drawablePointForPoint:(CGPoint)point
{
    const CGFloat scale = [self drawableScale];
    return CGPointMake(lround(point.x * scale), lround(point.y * scale));
}

- (void)requestRender
{
    [[self viewportView] requestRender];
}

#ifdef DEBUG
- (void)debugRequestRender
{
    [self requestRender];
}
#endif

- (NSUInteger)renderedFrameCount
{
    return [self viewportView].renderedFrameCount;
}

- (CGSize)drawableSize
{
    return [self viewportView].drawableSize;
}

- (BOOL)isRenderLoopRunning
{
    return [self viewportView].isRenderLoopRunning;
}

- (NSInteger)renderingAPIVersion
{
    switch ([self viewportView].renderingAPI) {
        case kEAGLRenderingAPIOpenGLES3:
            return 3;
        case kEAGLRenderingAPIOpenGLES2:
            return 2;
        default:
            return 0;
    }
}

- (void)documentDidChange:(NSNotification *)notification
{
    if ([notification.object isKindOfClass:NSValue.class]
        && _viewer != nullptr
        && !_viewer->getDocument().IsNull()
        && [(NSValue *)notification.object pointerValue]
            != _viewer->getDocument().get()) {
        return;
    }
    // Face visibility and appearance can change without a new TShape or
    // placement. Rebuild the optional overlay from the next committed scene.
    _selectedSurfaceAppearanceDirty = YES;
    _committedBoundariesDirty = YES;
    [self requestRender];
}

- (void)applicationWillResignActive:(NSNotification *)notification
{
    // Scene-enabled shipping hosts use the exact scene below. Keep deliberate
    // nil-object notifications used by standalone native hosts and legacy apps.
    if (notification.object == UIApplication.sharedApplication
        && self.viewIfLoaded.window.windowScene != nil
        && [NSBundle.mainBundle objectForInfoDictionaryKey:@"UIApplicationSceneManifest"] != nil) {
        return;
    }
    [self endActiveRenderingInteractions];
}

- (void)sceneWillDeactivate:(NSNotification *)notification
{
    UIWindowScene *owner = self.viewIfLoaded.window.windowScene;
    if (owner != nil && notification.object == owner) {
        [self endActiveRenderingInteractions];
    }
}

- (void)viewWillDisappear:(BOOL)animated
{
    [super viewWillDisappear:animated];
    [self endActiveRenderingInteractions];
}

- (void)endActiveRenderingInteractions
{
    GLView *view = [self viewportView];
    const BOOL hadRawPrimaryInteraction = _rawTouchRendering;
    const BOOL hadActiveInteraction =
        _rawTouchRendering || _pinchRendering || _panRendering;
    const BOOL hadUnresolvedMirrorObjects =
        _viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && _viewer->getObjectInteractor()->hasUnresolvedMirrorObjects();
    const BOOL hadBooleanOperation =
        _viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && (_viewer->getObjectInteractor()->hasActiveBoolean()
            || _viewer->getObjectInteractor()->hasUnresolvedBoolean());
    const BOOL hadLinearArray =
        _viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && (_viewer->getObjectInteractor()->hasActiveLinearArray()
            || _viewer->getObjectInteractor()->hasUnresolvedLinearArray());
    const BOOL hadRadialArray =
        _viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && (_viewer->getObjectInteractor()->hasActiveRadialArray()
            || _viewer->getObjectInteractor()->hasUnresolvedRadialArray());
    const PrimitiveGizmoType booleanGizmoType = hadBooleanOperation
        ? [self getGizmoType]
        : PrimitiveGizmoTypeNone;
    const BOOL hadExtrusion =
        _viewer != nullptr
        && _viewer->getShapeInteractor() != nullptr
        && _viewer->getShapeInteractor()->hasActiveExtrusion();
    const BOOL hadShell =
        _viewer != nullptr
        && _viewer->getShapeInteractor() != nullptr
        && (_viewer->getShapeInteractor()->hasActiveShell()
            || _viewer->getShapeInteractor()->hasUnresolvedShell());
    const BOOL hadBevel =
        _viewer != nullptr
        && _viewer->getShapeInteractor() != nullptr
        && _viewer->getShapeInteractor()->hasActiveBevel();
    if (hadActiveInteraction && _viewer != nullptr) {
        _viewer->CancelInteraction(0, 0);
    }
    if (hadUnresolvedMirrorObjects) {
		const std::shared_ptr<ObjectInteractor> aMirrorInteractor =
			_viewer->getObjectInteractor();
		const BOOL hadCustomPlane =
			aMirrorInteractor->hasCustomMirrorPlaneState();
		if (aMirrorInteractor->isPickingMirrorPlane()) {
			(void)aMirrorInteractor->cancelMirrorPlanePicking();
		}
		if (hadCustomPlane
			|| aMirrorInteractor->hasCustomMirrorPlaneState()) {
			(void)aMirrorInteractor->resetMirrorPlane();
		} else {
			(void)aMirrorInteractor->clearTrialMirrorObjects();
		}
		if (aMirrorInteractor->hasCustomMirrorPlaneState()
			&& aMirrorInteractor->mirrorPreviewState()
				!= MirrorPreviewState::OutcomeUnknown) {
			// A transient graphics erase may be retryable. Retry the same reset
			// so a retained Mirror gizmo converges to Selecting; cancelMirror()
			// would instead leave the public gizmo paired with Unavailable state.
			(void)aMirrorInteractor->resetMirrorPlane();
		}
    }
    if (hadBooleanOperation) {
        _viewer->getObjectInteractor()->cancelActiveBoolean();
        const BOOL didResolveBoolean =
            !_viewer->getObjectInteractor()->hasActiveBoolean()
            && !_viewer->getObjectInteractor()->hasUnresolvedBoolean();
        // Background/view disappearance retires transient geometry but keeps an
        // empty action provenance while the public tool mode remains Boolean, so
        // foreground taps can start a fresh operation without a mode toggle.
        if (!_isPreviewMode) {
            (void)[self restoreBooleanActionForRetainedGizmoType:
                booleanGizmoType];
        } else if (didResolveBoolean
                   && IsBooleanGizmo(booleanGizmoType)) {
            if (_delegate != nil
                && [_delegate respondsToSelector:
                    @selector(viewerDidFailToRetainBooleanMode:)]) {
                [_delegate viewerDidFailToRetainBooleanMode:self];
            } else {
                _viewer->getObjectInteractor()->setManipulatorType(
                    PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
            }
        }
    }
    if (hadLinearArray) {
        (void)_viewer->getObjectInteractor()->cancelLinearArray();
    }
    if (hadRadialArray) {
        (void)_viewer->getObjectInteractor()->cancelRadialArray();
    }
    if (hadExtrusion) {
        _viewer->getShapeInteractor()->cancelExtrusion();
    }
    if (hadShell) {
        (void)_viewer->getShapeInteractor()->cancelShell();
    }
    if (hadBevel) {
        (void)_viewer->getShapeInteractor()->cancelChamfer();
    }
    if (_rawTouchRendering) {
        _rawTouchRendering = NO;
        [view endInteractiveRendering];
    }
    [_rawViewportTouches removeAllObjects];
    if (_pinchRendering) {
        _pinchRendering = NO;
        [view endInteractiveRendering];
    }
    if (_panRendering) {
        _panRendering = NO;
        [view endInteractiveRendering];
    }
    if (hadActiveInteraction || hadUnresolvedMirrorObjects
        || hadBooleanOperation || hadLinearArray || hadRadialArray
        || hadExtrusion || hadShell || hadBevel) {
        [self requestRender];
    }
    if (hadUnresolvedMirrorObjects || hadBooleanOperation
		|| hadLinearArray || hadRadialArray || hadExtrusion || hadShell
        || hadBevel) {
        // Selection notification is also the renderer-neutral presentation
        // invalidation and Apply-state refresh for lifecycle cancellation.
        [self checkSelections];
    }
    if ((hadRawPrimaryInteraction || hadUnresolvedMirrorObjects
         || hadBooleanOperation || hadLinearArray || hadRadialArray
         || hadExtrusion || hadShell || hadBevel)
        && _delegate
        && [_delegate respondsToSelector:
            @selector(viewer:didEndPrimaryInteractionCancelled:)]) {
        [_delegate viewer:self didEndPrimaryInteractionCancelled:YES];
    }
    _rawTouchWasCancelled = NO;
}
// =======================================================================
// function : Draw
// purpose  :
// =======================================================================
- (BOOL) Draw
{
    if (_didSetupViewer && _viewer != nullptr) {
        // Appearance is optional presentation work. An allocation/OCCT style
        // failure must never suppress the model's normal native frame.
        try { [self updateCommittedBoundaries]; } catch (...) {
            try { [self clearCommittedBoundaries]; } catch (...) {}
        }
        try { [self updateSelectedSurfaceAppearance]; } catch (...) {
            try {
                [self clearSelectedFaceWash];
                _selectedContourGeometry.clear();
                const auto context = _viewer->AisContext();
                if (!context.IsNull() && !_viewer->hasUnresolvedEdit())
                    context->HilightSelected(Standard_False);
            } catch (...) {}
        }
        return _viewer->RenderFrame();
    }
    return NO;
}

- (void)setSelectedEdgeAppearanceColor:(UIColor *)color widthPoints:(CGFloat)width {
    CGFloat r = 0, g = 0, b = 0, a = 0;
    if (![NSThread isMainThread] || !std::isfinite(width) || width <= 0
        || ![color getRed:&r green:&g blue:&b alpha:&a]
        || !std::isfinite(r) || !std::isfinite(g) || !std::isfinite(b) || !std::isfinite(a)) return;
    if ([_selectedEdgeColor isEqual:color] && _selectedEdgeWidthPoints == width) return;
    _selectedEdgeColor = color;
    _selectedEdgeWidthPoints = width;
    _selectedEdgeAppearanceDirty = YES;
    if (_didSetupViewer) [self requestRender];
}

- (void)setSelectedSurfaceAppearanceColor:(UIColor *)color widthPoints:(CGFloat)width faceWashOpacity:(CGFloat)opacity {
    CGFloat r = 0, g = 0, b = 0, a = 0;
    if (!NSThread.isMainThread || !std::isfinite(width) || width <= 0
        || !std::isfinite(opacity) || opacity < 0 || opacity > 1
        || ![color getRed:&r green:&g blue:&b alpha:&a]
        || !std::isfinite(r) || !std::isfinite(g) || !std::isfinite(b) || !std::isfinite(a)) return;
    if ([_selectedSurfaceColor isEqual:color] && _selectedSurfaceWidthPoints == width
        && _selectedFaceWashOpacity == opacity) return;
    _selectedSurfaceColor = color;
    _selectedSurfaceWidthPoints = width;
    _selectedFaceWashOpacity = opacity;
    _selectedSurfaceAppearanceDirty = YES;
    // The next Draw replaces the overlay with its rendering context current.
    if (_didSetupViewer) [self requestRender];
}

- (void)clearCommittedBoundaries {
    for (auto& structure : _committedBoundaries) RemoveNativeSelectionStructure(structure);
    _committedBoundaries.clear();
    _splitBoundaryPresentation.Nullify();
    _committedBoundariesDirty = YES;
}

- (void)updateSplitBoundaries:(const Handle(AIS_Shape)&)presentation {
    if (!_committedBoundariesDirty && _splitBoundaryPresentation == presentation) return;
    std::vector<Handle(Graphic3d_Structure)> replacements;
    try {
        // Use the candidate's display tessellation, never triangle wireframe.
        // Keep coplanar Split edges; exclude periodic seams and degenerate edges
        // exactly as the committed scene extractor does. No document writes or
        // remeshing are needed, including while the preview command is open.
        std::vector<gp_Pnt> points;
        TopTools_IndexedMapOfShape faces, drawnEdges;
        TopExp::MapShapes(presentation->Shape(), TopAbs_FACE, faces);
        const auto transform = presentation->LocalTransformation();
        for (Standard_Integer index = 1; index <= faces.Extent(); ++index) {
            const auto face = TopoDS::Face(faces(index));
            TopLoc_Location location;
            const auto triangulation = BRep_Tool::Triangulation(face, location);
            if (triangulation.IsNull()) continue;
            TopTools_IndexedMapOfShape edges;
            TopExp::MapShapes(face, TopAbs_EDGE, edges);
            for (Standard_Integer edgeIndex = 1; edgeIndex <= edges.Extent(); ++edgeIndex) {
                const auto edge = TopoDS::Edge(edges(edgeIndex));
                if (BRep_Tool::Degenerated(edge) || BRep_Tool::IsClosed(edge, face)
                    || drawnEdges.Contains(edge)) continue;
                const auto polygon = BRep_Tool::PolygonOnTriangulation(edge, triangulation, location);
                if (polygon.IsNull() || polygon->NbNodes() < 2) continue;
                drawnEdges.Add(edge);
                for (Standard_Integer node = 1; node < polygon->NbNodes(); ++node) {
                    const auto first = polygon->Node(node), last = polygon->Node(node + 1);
                    if (first < 1 || last < 1 || first > triangulation->NbNodes()
                        || last > triangulation->NbNodes())
                        throw std::runtime_error("Split boundary triangulation index");
                    if (points.size() > 1'500'000 - 2)
                        throw std::length_error("Split boundary vertex budget");
                    for (const auto vertex : {first, last}) {
                        const auto point = triangulation->Node(vertex)
                            .Transformed(location.Transformation()).Transformed(transform);
                        if (!std::isfinite(point.X()) || !std::isfinite(point.Y()) || !std::isfinite(point.Z()))
                            throw std::runtime_error("Nonfinite Split boundary vertex");
                        points.push_back(point);
                    }
                }
            }
        }
        if (!points.empty()) {
            Handle(Graphic3d_Structure) boundaries = new Graphic3d_Structure(_viewer->V3dViewer()->StructureManager());
            replacements.push_back(boundaries);
            boundaries->SetZLayer(Graphic3d_ZLayerId_Default);
            boundaries->SetDisplayPriority(Graphic3d_DisplayPriority_Above);
            const gp_XYZ origin = points.front().XYZ();
            gp_Trsf placement; placement.SetTranslation(gp_Vec(origin));
            boundaries->SetTransformation(new TopLoc_Datum3D(placement));
            Handle(Graphic3d_ArrayOfSegments) lines = new Graphic3d_ArrayOfSegments(int(points.size()));
            for (const auto& point : points) lines->AddVertex(gp_Pnt(point.XYZ()-origin));
            const auto group = boundaries->NewGroup();
            const Handle(Prs3d_LineAspect) aspect = new Prs3d_LineAspect(Quantity_NOC_BLACK,Aspect_TOL_SOLID,1);
            aspect->Aspect()->SetShaderProgram(NativeBoundaryLineShader());
            group->SetGroupPrimitivesAspect(aspect->Aspect());
            group->AddPrimitiveArray(lines);
            boundaries->CalculateBoundBox();
        }
        [self clearCommittedBoundaries];
        _committedBoundaries = std::move(replacements);
        for (const auto& structure : _committedBoundaries) structure->Display();
        _splitBoundaryPresentation = presentation;
        _committedBoundariesDirty = NO;
    } catch (...) {
        for (auto& structure : replacements) RemoveNativeSelectionStructure(structure);
        [self clearCommittedBoundaries];
        throw;
    }
}

- (void)updateCommittedBoundaries {
    if (_viewer == nullptr) return;
    const auto shapeInteractor = _viewer->getShapeInteractor();
    const auto splitPresentation = shapeInteractor
        ? shapeInteractor->splitBoundaryPresentation() : Handle(AIS_Shape)();
    if (!splitPresentation.IsNull()) {
        [self updateSplitBoundaries:splitPresentation];
        return;
    }
    const auto interactor = _viewer->getObjectInteractor();
    if (_viewer->hasUnresolvedEdit() || !_viewer->canBeginCommittedEdit()
        || (interactor && interactor->isManipulatorGestureActive())) {
        // Live operations own their source suppression and preview graphics.
        [self clearCommittedBoundaries];
        return;
    }
    const auto document = _viewer->getDocument();
    if (document.IsNull()) { [self clearCommittedBoundaries]; return; }
    const auto documentIdentifier = document->DocumentIdentifier();
    if (!_committedBoundariesDirty && _splitBoundaryPresentation.IsNull()
        && _committedBoundaryDocument == documentIdentifier
        && _committedBoundaryDocumentOwner == document->Document().get()) return;
    const CGSize drawable = self.drawableSize;
    if (drawable.width <= 0 || drawable.height <= 0) { [self clearCommittedBoundaries]; return; }
    const auto scene = _viewer->captureSceneSnapshot(drawable.width, drawable.height);
    if (!scene) { [self clearCommittedBoundaries]; return; }
    std::vector<Handle(Graphic3d_Structure)> replacements;
    std::size_t totalPoints = 0;
    try {
        for (const auto& item : scene->instances) {
            if (!item.visible) continue;
            const auto& mesh = scene->meshes.at(item.meshIndex);
            if (item.primitiveBindings.size() != mesh.primitives.size()) {
                throw std::runtime_error("Native boundary primitive bindings");
            }
            std::unordered_set<std::uint32_t> visibleFaces;
            for (std::size_t index = 0; index < mesh.primitives.size(); ++index) {
                if (item.primitiveBindings[index].visible)
                    visibleFaces.insert(mesh.primitives[index].faceIndex);
            }
            const auto& m = item.worldFromObject.values;
            const auto world = [&](float x, float y, float z) {
                return gp_Pnt(m[0]*x+m[4]*y+m[8]*z+m[12],
                              m[1]*x+m[5]*y+m[9]*z+m[13],
                              m[2]*x+m[6]*y+m[10]*z+m[14]);
            };
            std::vector<gp_Pnt> points;
            std::unordered_map<std::uint32_t, std::uint32_t> incidentFace;
            for (const auto& edge : mesh.edgeSegments) {
                if (!visibleFaces.count(edge.faceIndex)) continue;
                const auto inserted = incidentFace.emplace(edge.edgeIndex, edge.faceIndex);
                if (!inserted.second && inserted.first->second != edge.faceIndex) continue;
                // Match the selected-overlay cap. Never publish a partial scene.
                if (totalPoints > 1'500'000 - 2) throw std::length_error("Native boundary vertex budget");
                totalPoints += 2;
                points.push_back(world(edge.startX,edge.startY,edge.startZ));
                points.push_back(world(edge.endX,edge.endY,edge.endZ));
            }
            if (points.empty()) continue;
            Handle(Graphic3d_Structure) boundaries = new Graphic3d_Structure(_viewer->V3dViewer()->StructureManager());
            replacements.push_back(boundaries);
            boundaries->SetZLayer(Graphic3d_ZLayerId_Default);
            // Surface first, black boundaries next, white selection last; all use depth.
            boundaries->SetDisplayPriority(Graphic3d_DisplayPriority_Above);
            const gp_XYZ origin = points.front().XYZ();
            gp_Trsf placement; placement.SetTranslation(gp_Vec(origin));
            boundaries->SetTransformation(new TopLoc_Datum3D(placement));
            Handle(Graphic3d_ArrayOfSegments) lines = new Graphic3d_ArrayOfSegments(int(points.size()));
            for (const auto& point : points) lines->AddVertex(gp_Pnt(point.XYZ()-origin));
            const auto group = boundaries->NewGroup();
            // Existing committed-edge look shared with Metal: black, one drawable pixel.
            const Handle(Prs3d_LineAspect) aspect = new Prs3d_LineAspect(Quantity_NOC_BLACK,Aspect_TOL_SOLID,1);
            aspect->Aspect()->SetShaderProgram(NativeBoundaryLineShader());
            group->SetGroupPrimitivesAspect(aspect->Aspect());
            group->AddPrimitiveArray(lines);
            boundaries->CalculateBoundBox();
        }
        [self clearCommittedBoundaries];
        _committedBoundaries = std::move(replacements);
        for (const auto& structure : _committedBoundaries) structure->Display();
        _committedBoundaryDocument = documentIdentifier;
        _committedBoundaryDocumentOwner = document->Document().get();
        _committedBoundariesDirty = NO;
    } catch (...) {
        for (auto& structure : replacements) RemoveNativeSelectionStructure(structure);
        [self clearCommittedBoundaries];
        throw;
    }
}

- (void)clearSelectedFaceWash {
    RemoveNativeSelectionStructure(_selectedFaceWash);
    _selectedWashFaces.clear();
}

- (void)updateSelectedSurfaceAppearance {
    [self updateSelectedEdgeAppearance];
    if (_viewer == nullptr || _selectedSurfaceColor == nil) return;
    const auto context = _viewer->AisContext();
    const auto interactor = _viewer->getShapeInteractor();
    if (context.IsNull() || !interactor) return;
    const auto mode = interactor->getSelectionMode();
    const bool faceMode = mode == ShapeSelectionMode::Face;
    if (mode != ShapeSelectionMode::WholeShape && !faceMode) {
        [self clearSelectedFaceWash];
        return;
    }
    // Restore ordinary highlights if the optional overlay cannot be built.
    const auto fallback = [&] {
        const bool hadOverlay = !_selectedFaceWash.IsNull();
        [self clearSelectedFaceWash];
        _selectedContourGeometry.clear();
        if (hadOverlay) context->HilightSelected(Standard_False);
    };
    if (_viewer->hasUnresolvedEdit() || !_viewer->canBeginCommittedEdit()) {
        // A live operation owns its highlighting; only retire our overlay.
        [self clearSelectedFaceWash]; return;
    }
    const CGFloat pixels = _selectedSurfaceWidthPoints * [self drawableScale];
    if (!std::isfinite(pixels) || pixels <= 0) { fallback(); return; }
    std::vector<TopoDS_Shape> faces;
    const auto document = _viewer->getDocument();
    for (context->InitSelected(); context->MoreSelected(); context->NextSelected()) {
        if (document.IsNull() || faces.size() >= 50'000) {
            fallback(); return;
        }
        const auto label = document->ShapeLabel(context->SelectedInteractive());
        if (label.IsNull()) { fallback(); return; }
        TopoDS_Shape shape;
        if (context->HasSelectedShape()) shape = context->SelectedShape();
        else if (!faceMode) {
            const auto presentation = Handle(AIS_Shape)::DownCast(context->SelectedInteractive());
            if (!presentation.IsNull()) shape = presentation->Shape();
        }
        if (shape.IsNull() || (faceMode && shape.ShapeType() != TopAbs_FACE)) { fallback(); return; }
        faces.push_back(shape);
    }
    if (faces.empty()) { [self clearSelectedFaceWash]; _selectedContourGeometry.clear(); return; }
    const auto camera = _viewer->ActiveView()->Camera();
    const CGSize drawable = self.drawableSize;
    const bool sameGeometry = !_selectedSurfaceAppearanceDirty
        && _selectedContourDocument == document->Document().get()
        && _selectedSurfaceWasFace == faceMode && faces.size() == _selectedWashFaces.size()
        && std::equal(faces.begin(), faces.end(), _selectedWashFaces.begin(),
            [](const TopoDS_Shape& a, const TopoDS_Shape& b) {
                if (a.TShape() != b.TShape() || a.Orientation() != b.Orientation()) return false;
                // Owner::Location creates fresh datum handles for transforms.
                // Exact coefficients retain unchanged frames without hiding motion.
                const auto& x = a.Location().Transformation();
                const auto& y = b.Location().Transformation();
                for (int row = 1; row <= 3; ++row)
                    for (int column = 1; column <= 4; ++column)
                        if (x.Value(row,column) != y.Value(row,column)) return false;
                return true;
            });
    if (sameGeometry && _selectedSurfaceWidthPixels == pixels
        && _selectedContourViewState == camera->WorldViewState()
        && _selectedContourProjectionState == camera->ProjectionState()
        && CGSizeEqualToSize(_selectedContourDrawable,drawable)) {
        context->UnhilightSelected(Standard_False);
        return;
    }
    if (drawable.width <= 0 || drawable.height <= 0) { fallback(); return; }
    const auto scene = _viewer->captureSceneSnapshot(drawable.width, drawable.height);
    if (!scene) { fallback(); return; }
    struct SelectedFaces { bool object = false; std::unordered_set<std::uint32_t> faces; };
    std::unordered_map<std::string, SelectedFaces> selected;
    for (const auto& element : scene->selection.selected) {
        auto& entry = selected[element.entityIdentifier];
        if (element.kind == core3d::scene::ElementKind::Object) entry.object = true;
        else if (element.kind == core3d::scene::ElementKind::Face) entry.faces.insert(element.topologyIndex);
        else { fallback(); return; }
    }
    std::vector<gp_Pnt> points;
    std::vector<NativeContourVertex> contourTriangles;
    if (!sameGeometry) _selectedContourGeometry.clear();
    std::size_t contourVertexBudget=0, contourIndexBudget=0;
    for (const auto& item : scene->instances) {
        const auto selection = selected.find(item.entityIdentifier);
        if (selection == selected.end() || !item.visible) continue;
        const auto& mesh = scene->meshes.at(item.meshIndex);
        if (item.primitiveBindings.size() != mesh.primitives.size()) { fallback(); return; }
        std::unordered_set<std::uint32_t> visibleFaces;
        for (std::size_t index = 0; index < mesh.primitives.size(); ++index) {
            if (item.primitiveBindings[index].visible)
                visibleFaces.insert(mesh.primitives[index].faceIndex);
        }
        for (const auto face : selection->second.faces) {
            if (!visibleFaces.count(face)) { fallback(); return; }
        }
        const auto& m = item.worldFromObject.values;
        const auto world = [&](float x, float y, float z) {
            return gp_Pnt(m[0]*x+m[4]*y+m[8]*z+m[12],
                          m[1]*x+m[5]*y+m[9]*z+m[13],
                          m[2]*x+m[6]*y+m[10]*z+m[14]);
        };
        // Planar BRep parts keep the qualified semantic-edge path. Smooth
        // surfaces and polygon imports also need camera-dependent silhouettes.
        // In Face mode build the silhouette from only the chosen surfaces.
        // Including their unselected neighbours would outline the whole solid
        // when a user selected one cap, and suppress the face's open boundary.
        const auto* selectedFaces = faceMode ? &selection->second.faces : nullptr;
        const bool needsContour = mesh.edgeSegments.empty()
            || std::any_of(mesh.primitives.begin(),mesh.primitives.end(),[&](const auto& p) {
                return visibleFaces.count(p.faceIndex)
                    && (!selectedFaces || selectedFaces->count(p.faceIndex))
                    && (!p.analyticSourceFace.available || p.analyticSourceFace.surfaceType != 0);
            });
        NativeProjectedContour contour;
        if (needsContour) {
            auto cached = std::find_if(_selectedContourGeometry.begin(),_selectedContourGeometry.end(),
                [&](const auto& entry) { return entry.entity == item.entityIdentifier; });
            if (cached == _selectedContourGeometry.end()) {
                _selectedContourGeometry.push_back(BuildNativeContourGeometry(mesh,item,selectedFaces,
                    contourVertexBudget,contourIndexBudget));
                cached = std::prev(_selectedContourGeometry.end());

            }
            contour = ProjectNativeContours(*cached,camera,drawable.width,drawable.height,pixels);
            if (contourTriangles.size()+contour.triangles.size() > 1'500'000) { fallback(); return; }
            contourTriangles.insert(contourTriangles.end(),contour.triangles.begin(),contour.triangles.end());
        }
        const auto screen = [&](const gp_Pnt& point) {
            const auto p = camera->Project(point);
            return gp_Pnt((p.X()+1)*drawable.width*0.5,(p.Y()+1)*drawable.height*0.5,p.Z());
        };
        std::unordered_map<std::uint32_t, std::uint32_t> incidentFace;
        const auto before = points.size();
        for (const auto& edge : mesh.edgeSegments) {
            if (!visibleFaces.count(edge.faceIndex)) continue;
            if (!selection->second.object && !selection->second.faces.count(edge.faceIndex)) continue;
            const auto inserted = incidentFace.emplace(edge.edgeIndex, edge.faceIndex);
            // Keep every segment of a curved edge, but only one incident face.
            if (!inserted.second && inserted.first->second != edge.faceIndex) continue;
            if (points.size() > 1'500'000 - 2) { fallback(); return; }
            const auto a=world(edge.startX,edge.startY,edge.startZ);
            const auto b=world(edge.endX,edge.endY,edge.endZ);
            // Do not stack a centered cap edge on its outward silhouette.
            if (needsContour && contour.containsBoundary(screen(a),screen(b))) continue;
            points.push_back(a); points.push_back(b);
        }
        if (points.size() == before && contour.triangles.empty()) { fallback(); return; }
    }
    if (points.empty() && contourTriangles.empty()) { fallback(); return; }
    CGFloat r = 0, g = 0, b = 0, a = 0;
    if (![_selectedSurfaceColor getRed:&r green:&g blue:&b alpha:&a]) { fallback(); return; }
    const Quantity_Color color(r,g,b,Quantity_TOC_sRGB);
    Handle(Graphic3d_Structure) wash = new Graphic3d_Structure(_viewer->V3dViewer()->StructureManager());
    // Share the model depth buffer, including its transparent pass. The
    // separate Top-layer transparent presentation can lose that occlusion.
    wash->SetZLayer(Graphic3d_ZLayerId_Default);
    // This is an ordinary scene structure with explicit line and translucent
    // fill aspects, not OCCT's special wire-only highlight presentation.
    wash->SetDisplayPriority(Graphic3d_DisplayPriority_Topmost);
    // Keep GPU vertices close to the selection even for distant instances.
    const gp_XYZ origin = (points.empty() ? contourTriangles.front().position : points.front()).XYZ();
    gp_Trsf placement; placement.SetTranslation(gp_Vec(origin));
    wash->SetTransformation(new TopLoc_Datum3D(placement));
    try {
        if (!points.empty()) {
            Handle(Graphic3d_ArrayOfSegments) lines = new Graphic3d_ArrayOfSegments(int(points.size()));
            for (const auto& point : points) lines->AddVertex(gp_Pnt(point.XYZ()-origin));
            const auto lineGroup = wash->NewGroup();
            const Handle(Prs3d_LineAspect) lineAspect = new Prs3d_LineAspect(color,Aspect_TOL_SOLID,pixels);
            lineAspect->Aspect()->SetShaderProgram(NativeBoundaryLineShader());
            lineGroup->SetGroupPrimitivesAspect(lineAspect->Aspect());
            lineGroup->AddPrimitiveArray(lines);
        }
        if (!contourTriangles.empty()) {
            Handle(Graphic3d_ArrayOfTriangles) triangles = new Graphic3d_ArrayOfTriangles(
                int(contourTriangles.size()),0,Standard_False,Standard_False,Standard_True);
            for (const auto& p : contourTriangles) triangles->AddVertex(gp_Pnt(p.position.XYZ()-origin),p.offset);
            Handle(Prs3d_ShadingAspect) shading = new Prs3d_ShadingAspect();
            shading->SetColor(color);
            shading->Aspect()->SetShadingModel(Graphic3d_TypeOfShadingModel_Unlit);
            shading->Aspect()->SetFaceCulling(Graphic3d_TypeOfBackfacingModel_DoubleSided);
            shading->Aspect()->SetShaderProgram(NativeBoundaryLineShader(true));
            const auto group = wash->NewGroup();
            group->SetGroupPrimitivesAspect(shading->Aspect());
            group->AddPrimitiveArray(triangles);
        }
        if (faceMode) {
            Handle(Prs3d_ShadingAspect) shading = new Prs3d_ShadingAspect();
            shading->SetColor(color);
            shading->SetTransparency(1.0f-_selectedFaceWashOpacity);
            const auto aspect = shading->Aspect();
            aspect->SetShadingModel(Graphic3d_TypeOfShadingModel_Unlit);
            aspect->SetAlphaMode(Graphic3d_AlphaMode_Blend);
            aspect->SetPolygonOffsets(Aspect_POM_Fill,-1,-1);
            std::size_t vertices = 0, indices = 0;
            for (const auto& shape : faces) {
                TopLoc_Location location;
                const auto triangulation = BRep_Tool::Triangulation(TopoDS::Face(shape),location);
                if (triangulation.IsNull() || triangulation->NbNodes() <= 0 || triangulation->NbTriangles() <= 0) {
                    RemoveNativeSelectionStructure(wash); fallback(); return;
                }
                vertices += std::size_t(triangulation->NbNodes());
                indices += std::size_t(triangulation->NbTriangles())*3;
                if (vertices > 1'500'000 || indices > 4'500'000) { RemoveNativeSelectionStructure(wash); fallback(); return; }
                Handle(Graphic3d_ArrayOfTriangles) triangles = new Graphic3d_ArrayOfTriangles(
                    triangulation->NbNodes(),triangulation->NbTriangles()*3);
                for (int index=1; index<=triangulation->NbNodes(); ++index) {
                    const auto p = triangulation->Node(index).Transformed(location.Transformation());
                    triangles->AddVertex(gp_Pnt(p.XYZ()-origin));
                }
                for (int index=1; index<=triangulation->NbTriangles(); ++index) {
                    int a,b,c; triangulation->Triangle(index).Get(a,b,c);
                    if (shape.Orientation() == TopAbs_REVERSED) std::swap(b,c);
                    triangles->AddEdges(a,b,c);
                }
                const auto group = wash->NewGroup();
                group->SetGroupPrimitivesAspect(aspect);
                group->AddPrimitiveArray(triangles);
            }
        }
        [self clearSelectedFaceWash];
        _selectedFaceWash = wash;
        _selectedWashFaces = std::move(faces);
        _selectedSurfaceWidthPixels = pixels;
        _selectedSurfaceWasFace = faceMode;
        _selectedSurfaceAppearanceDirty = NO;
        _selectedContourViewState = camera->WorldViewState();
        _selectedContourProjectionState = camera->ProjectionState();
        _selectedContourDrawable = drawable;
        _selectedContourDocument = document->Document().get();
        wash->CalculateBoundBox();
        wash->Display();
        // This does not change IsSelected or the authoritative owner set.
        context->UnhilightSelected(Standard_False);
    } catch (...) {
        RemoveNativeSelectionStructure(wash); fallback(); throw;
    }
}

- (void)updateSelectedEdgeAppearance {
    // Operation previews own their presentation styles. Ordinary selection
    // styling neither enters their lifecycle nor changes selected AIS owners.
    if (_selectedEdgeColor == nil || _viewer == nullptr) return;
    const auto interactor = _viewer->getShapeInteractor();
    const auto context = _viewer->AisContext();
    if (!interactor || context.IsNull()) return;
    const bool edgeMode = interactor->getSelectionMode() == ShapeSelectionMode::Edge;
    const auto current = context->HighlightStyle(Prs3d_TypeOfHighlight_LocalSelected);
    const CGFloat pixels = _selectedEdgeWidthPoints * [self drawableScale];
    // Unchanged frames do not enumerate presentation authority or regenerate
    // highlights. The actual selection transition keeps its existing guards.
    if (!edgeMode && (_selectedEdgeStyle.IsNull() || current != _selectedEdgeStyle)) return;
    if (edgeMode && current == _selectedEdgeStyle && !_selectedEdgeAppearanceDirty
        && _selectedEdgeWidthPixels == pixels) return;
    if (_viewer->hasUnresolvedEdit() || !_viewer->canBeginCommittedEdit()) return;
    if (edgeMode && [self getSelectionType] != PrimitiveSelectionTypeEdge) return;
    if (_ordinaryLocalSelectionStyle.IsNull()
        || (current != _ordinaryLocalSelectionStyle && current != _selectedEdgeStyle)) {
        // A newly installed context or another presentation owner supplies a
        // fresh default. Retain that exact drawer for Face/Object restoration.
        _ordinaryLocalSelectionStyle = current;
        _selectedEdgeStyle.Nullify();
    }
    if (!edgeMode) {
        if (!_selectedEdgeStyle.IsNull() && current == _selectedEdgeStyle) {
            context->SetHighlightStyle(Prs3d_TypeOfHighlight_LocalSelected, _ordinaryLocalSelectionStyle);
            context->HilightSelected(Standard_False);
        }
        return;
    }
    if (!std::isfinite(pixels) || pixels <= 0) return;
    if (_selectedEdgeStyle.IsNull()) {
        _selectedEdgeStyle = new Prs3d_Drawer();
        _selectedEdgeStyle->SetLink(_ordinaryLocalSelectionStyle);
        _selectedEdgeStyle->SetDisplayMode(0);
        // Top inherits the scene depth. Topmost would expose hidden edges.
        _selectedEdgeStyle->SetZLayer(Graphic3d_ZLayerId_Top);
        _selectedEdgeAppearanceDirty = YES;
    }
    BOOL changed = current != _selectedEdgeStyle;
    if (_selectedEdgeAppearanceDirty || _selectedEdgeWidthPixels != pixels) {
        CGFloat r = 0, g = 0, b = 0, a = 0;
        if (![_selectedEdgeColor getRed:&r green:&g blue:&b alpha:&a]) return;
        const Quantity_Color color(r, g, b, Quantity_TOC_sRGB);
        _selectedEdgeStyle->SetColor(color);
        _selectedEdgeStyle->SetTransparency(1.0f - a);
        const Handle(Prs3d_LineAspect) line = new Prs3d_LineAspect(color, Aspect_TOL_SOLID, pixels);
        line->Aspect()->SetShaderProgram(NativeBoundaryLineShader());
        _selectedEdgeStyle->SetLineAspect(line);
        _selectedEdgeStyle->SetWireAspect(line);
        _selectedEdgeStyle->SetFreeBoundaryAspect(line);
        _selectedEdgeStyle->SetUnFreeBoundaryAspect(line);
        // Highlight overrides consume Graphic3d presentation attributes;
        // drawer line width alone does not control the rendered highlight.
        const Handle(Graphic3d_AspectFillArea3d) aspect = new Graphic3d_AspectFillArea3d();
        aspect->SetInteriorColor(Quantity_ColorRGBA(color, a));
        aspect->SetLineWidth(pixels);
        aspect->SetShaderProgram(NativeBoundaryLineShader());
        aspect->SetShadingModel(Graphic3d_TypeOfShadingModel_Unlit);
        _selectedEdgeStyle->SetBasicFillAreaAspect(aspect);
        _selectedEdgeWidthPixels = pixels;
        _selectedEdgeAppearanceDirty = NO;
        changed = YES;
    }
    if (changed) {
        context->SetHighlightStyle(Prs3d_TypeOfHighlight_LocalSelected, _selectedEdgeStyle);
        context->HilightSelected(Standard_False);
    }
}

// =======================================================================
// function : Setup
// purpose  :
// =======================================================================
- (void) Setup {
    if (_didSetupViewer) {
        _viewer->Resize();
        [self requestRender];
        return;
    }

    if (!_viewer->InitViewer(self.view)) {
        NSLog(@"Failed to init viewer");
        return;
    }

    __weak typeof(self) weakSelf = self;
    _viewer->setBooleanPreviewStateChangedCallback([weakSelf]() {
        dispatch_async(dispatch_get_main_queue(), ^{
            __strong typeof(weakSelf) strongSelf = weakSelf;
            if (strongSelf == nil) {
                return;
            }
            [strongSelf checkSelections];
            [strongSelf requestRender];
        });
    });
    _viewer->setBevelPreviewStateChangedCallback([weakSelf]() {
        dispatch_async(dispatch_get_main_queue(), ^{
            __strong typeof(weakSelf) strongSelf = weakSelf;
            if (strongSelf == nil) {
                return;
            }
            [strongSelf checkSelections];
            [strongSelf requestRender];
        });
    });
    _viewer->setShellPreviewStateChangedCallback([weakSelf]() {
        dispatch_async(dispatch_get_main_queue(), ^{
            __strong typeof(weakSelf) strongSelf = weakSelf;
            if (strongSelf == nil) {
                return;
            }
            if (strongSelf->_delegate
                && [strongSelf->_delegate respondsToSelector:
                    @selector(viewerDidChangeShellPresentationOverlay:)]) {
                [strongSelf->_delegate
                    viewerDidChangeShellPresentationOverlay:strongSelf];
            } else {
                [strongSelf checkSelections];
            }
            [strongSelf requestRender];
        });
    });
    _viewer->setLinearArrayPreviewStateChangedCallback([weakSelf]() {
        dispatch_async(dispatch_get_main_queue(), ^{
            __strong typeof(weakSelf) strongSelf = weakSelf;
            if (strongSelf == nil) {
                return;
            }
            if (strongSelf->_delegate
                && [strongSelf->_delegate respondsToSelector:
                    @selector(viewerDidChangeLinearArrayPresentationOverlay:)]) {
                [strongSelf->_delegate
                    viewerDidChangeLinearArrayPresentationOverlay:strongSelf];
            }
            [strongSelf requestRender];
        });
    });
    _viewer->setRadialArrayPreviewStateChangedCallback([weakSelf]() {
        dispatch_async(dispatch_get_main_queue(), ^{
            __strong typeof(weakSelf) strongSelf = weakSelf;
            if (strongSelf == nil) {
                return;
            }
            if (strongSelf->_delegate
                && [strongSelf->_delegate respondsToSelector:
                    @selector(viewerDidChangeRadialArrayPresentationOverlay:)]) {
                [strongSelf->_delegate
                    viewerDidChangeRadialArrayPresentationOverlay:strongSelf];
            }
            [strongSelf requestRender];
        });
    });

    _didSetupViewer = YES;
    _viewer->showGrid(!_isPreviewMode);
    if (_isPreviewMode) {
        _viewer->setPreviewMode();
    }
    //    [self importScrew:nullptr];
    if (_delegate && [_delegate respondsToSelector:@selector(didSetupViewer:)]) {
        __weak typeof(self) weakSelf = self;
        [_delegate didSetupViewer:weakSelf];
    }
    [self requestRender];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    if (_viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && _viewer->getObjectInteractor()->hasActiveMirror()) {
        // Cancel only through the native owner. An unknown commit or a failed
        // graphics erase must retain every owned handle and recovery control.
        (void)_viewer->getObjectInteractor()->cancelMirror();
        [self checkSelections];
        [self requestRender];
        if (_delegate
            && [_delegate respondsToSelector:
                @selector(viewer:didEndPrimaryInteractionCancelled:)]) {
            [_delegate viewer:self
                didEndPrimaryInteractionCancelled:YES];
        }
    }
    if (_viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && (_viewer->getObjectInteractor()->hasActiveBoolean()
            || _viewer->getObjectInteractor()->hasUnresolvedBoolean())) {
        const PrimitiveGizmoType currentType = [self getGizmoType];
        _viewer->getObjectInteractor()->cancelActiveBoolean();
        (void)[self restoreBooleanActionForRetainedGizmoType:currentType];
        [self checkSelections];
        [self requestRender];
    }
    if (_viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && _viewer->getObjectInteractor()->hasActiveLinearArray()) {
        (void)_viewer->getObjectInteractor()->cancelLinearArray();
        [self checkSelections];
        [self requestRender];
        // Publish both successful retirement and retryable cleanup failure.
        // The parent resolves an inactive Array tool, or retains an active
        // failed operation so the user can retry Cancel safely.
        if (_delegate
            && [_delegate respondsToSelector:
                @selector(viewer:didEndPrimaryInteractionCancelled:)]) {
            [_delegate viewer:self
                didEndPrimaryInteractionCancelled:YES];
        }
    }
    if (_viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && _viewer->getObjectInteractor()->hasActiveRadialArray()) {
        (void)_viewer->getObjectInteractor()->cancelRadialArray();
        [self checkSelections];
        [self requestRender];
        // Publish both successful retirement and retryable cleanup failure. The
        // parent either closes an inactive tool or retains its recovery controls.
        if (_delegate
            && [_delegate respondsToSelector:
                @selector(viewer:didEndPrimaryInteractionCancelled:)]) {
            [_delegate viewer:self
                didEndPrimaryInteractionCancelled:YES];
        }
    }
    if (_viewer != nullptr
        && _viewer->getShapeInteractor() != nullptr
        && _viewer->getShapeInteractor()->hasActiveBevel()) {
        (void)_viewer->getShapeInteractor()->cancelChamfer();
        [self checkSelections];
        [self requestRender];
        if (_delegate
            && [_delegate respondsToSelector:
                @selector(viewer:didEndPrimaryInteractionCancelled:)]) {
            [_delegate viewer:self
                didEndPrimaryInteractionCancelled:YES];
        }
    }
    if (_viewer != nullptr
        && _viewer->getShapeInteractor() != nullptr
        && _viewer->getShapeInteractor()->hasActiveShell()) {
        (void)_viewer->getShapeInteractor()->cancelShell();
        [self checkSelections];
        [self requestRender];
        if (_delegate
            && [_delegate respondsToSelector:
                @selector(viewer:didEndPrimaryInteractionCancelled:)]) {
            [_delegate viewer:self
                didEndPrimaryInteractionCancelled:YES];
        }
    }
}

// =======================================================================
// function : loadView
// purpose  :
// =======================================================================
- (void) loadView
{
    GLView* aGLView = [[GLView alloc] init];
    aGLView->myController = self;
    self.view = aGLView;
}

- (void) setConstructorMode
{
	_isConstructorMode = true;
}

// =======================================================================
// function : touchesBegan
// purpose  :
// =======================================================================
- (void)touchesBegan:(NSSet<UITouch *> *)theTouches withEvent:(UIEvent *)theEvent
{
    [super touchesBegan:theTouches withEvent:theEvent];

    if (_isPreviewMode) {
        return;
    }

	const BOOL didBeginPrimaryInteraction = _rawViewportTouches.count == 0;
	[_rawViewportTouches unionSet:theTouches];
	if (didBeginPrimaryInteraction) {
		_cancelTouches = NO;
		_rawTouchWasCancelled = NO;
		_rawTouchRendering = YES;
		_suppressTapSelectionForManipulatorInteraction = NO;
		[[self viewportView] beginInteractiveRendering];
	}

    UITouch *aTouch = [theTouches anyObject];
    if (aTouch != NULL) {
        const CGPoint point = [self drawablePointForPoint:[aTouch locationInView:self.view]];
#ifdef DEBUG
        [self debugTraceNativeFaceTap:point stage:@"touch-begin"];
#endif
        if (didBeginPrimaryInteraction
            && _delegate
            && [_delegate respondsToSelector:
                @selector(viewer:willBeginPrimaryInteractionAtDrawablePoint:drawableSize:)]) {
            [_delegate viewer:self
                willBeginPrimaryInteractionAtDrawablePoint:point
                                             drawableSize:self.drawableSize];
        }
        _tapRecognizer.cancelsTouchesInView = YES;
        _viewer->StartRotation((int)point.x, (int)point.y);
#ifdef DEBUG
        [self debugTraceNativeFaceTap:point stage:@"after-touch-begin"];
#endif
        const std::shared_ptr<ObjectInteractor> anInteractor =
            _viewer->getObjectInteractor();
        _suppressTapSelectionForManipulatorInteraction =
            anInteractor != nullptr
            && anInteractor->isManipulatorInteractionActive();
        if (_suppressTapSelectionForManipulatorInteraction) {
            // The raw touch-up resolves both transforms and operation handles
            // such as mirror planes. Keep the recognizer from cancelling that
            // lifecycle; tapHandler still suppresses ordinary selection.
            _tapRecognizer.cancelsTouchesInView = NO;
        }
        [self requestRender];
    }
}

- (void)endRawPrimaryInteractionIfNeededCancelled:(BOOL)cancelled {
    if (!_rawTouchRendering || _rawViewportTouches.count != 0) {
        return;
    }
    _rawTouchRendering = NO;
    [[self viewportView] endInteractiveRendering];
    const BOOL resolvedAsCancelled = cancelled || _rawTouchWasCancelled;
    _rawTouchWasCancelled = NO;
    if (_delegate
        && [_delegate respondsToSelector:
            @selector(viewer:didEndPrimaryInteractionCancelled:)]) {
        [_delegate viewer:self
            didEndPrimaryInteractionCancelled:resolvedAsCancelled];
    }
}

// =======================================================================
// function : touchesMoved
// purpose  :
// =======================================================================
- (void)touchesMoved:(NSSet *)theTouches withEvent:(UIEvent *)theEvent
{
	if (_isPreviewMode) {
		return;
	}
    if(_cancelTouches) {
        [self touchesCancelled:theTouches withEvent:theEvent];
        return;
    }

    [super touchesMoved:theTouches withEvent:theEvent];
    
    UITouch *aTouch = [theTouches anyObject];
    if ((aTouch != NULL) && theEvent.allTouches.count == 1) {
        const CGPoint point = [self drawablePointForPoint:[aTouch locationInView:self.view]];
        _viewer->Rotation((int)point.x, (int)point.y);
        [self requestRender];

#ifdef DEBUG
        if (_delegate && [_delegate respondsToSelector:@selector(didChangeStatusString:)]) {
            [_delegate didChangeStatusString:[self statusString]];
        }
#endif
    }

    return;
}

-(void) cancellTouchEvents {
    _cancelTouches = YES;
}

-(void) touchesEnded:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event {
    [super touchesEnded:touches withEvent:event];

    UITouch *aTouch = [touches anyObject];
    if (!_isPreviewMode && aTouch != NULL) {
        const CGPoint point = [self drawablePointForPoint:[aTouch locationInView:self.view]];
        _viewer->FinishInteraction((int)point.x, (int)point.y);
        [self requestRender];

#ifdef DEBUG
        if (_delegate && [_delegate respondsToSelector:@selector(didChangeStatusString:)]) {
            [_delegate didChangeStatusString:[self statusString]];
        }
#endif
	}
	[_rawViewportTouches minusSet:touches];
	[self endRawPrimaryInteractionIfNeededCancelled:NO];

    return;
}

-(void) touchesCancelled:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event{
    [super touchesCancelled:touches withEvent:event];
	_rawTouchWasCancelled = YES;

    UITouch *aTouch = [touches anyObject];
    if (!_isPreviewMode && aTouch != NULL) {
        const CGPoint point = [self drawablePointForPoint:[aTouch locationInView:self.view]];
        _viewer->CancelInteraction((int)point.x, (int)point.y);
#ifdef DEBUG
        [self debugTraceNativeFaceTap:point stage:@"touch-cancel"];
#endif
        [self requestRender];

#ifdef DEBUG
        if (_delegate && [_delegate respondsToSelector:@selector(didChangeStatusString:)]) {
            [_delegate didChangeStatusString:[self statusString]];
        }
#endif
	}
	[_rawViewportTouches minusSet:touches];
	[self endRawPrimaryInteractionIfNeededCancelled:YES];

    return;
}

// =======================================================================
// function : viewDidLoad
// purpose  :
// =======================================================================
-(void)viewDidLoad
{
    [super viewDidLoad];

    // add zoom recognizer
    UIPinchGestureRecognizer *aZoomRecognizer = [[UIPinchGestureRecognizer alloc]
                                                 initWithTarget:self
                                                 action:@selector(zoomHandler:)];
    aZoomRecognizer.delegate = self;

    [[self view] addGestureRecognizer:aZoomRecognizer];

    // add pan recognizer
    UIPanGestureRecognizer *aPanRecognizer = [[UIPanGestureRecognizer alloc]
                                              initWithTarget:self
                                              action:@selector(panHandler:)];

    aPanRecognizer.maximumNumberOfTouches = 2;
    aPanRecognizer.minimumNumberOfTouches = 2;
    aPanRecognizer.delegate = self;

    [[self view] addGestureRecognizer:aPanRecognizer];

    _tapRecognizer = [[UITapGestureRecognizer alloc]
                      initWithTarget:self
                      action:@selector(tapHandler:)];
    [[self view] addGestureRecognizer:_tapRecognizer];


    // add import buttons
    UIBarButtonItem *importScrewBtn = [[UIBarButtonItem alloc]
                                       initWithTitle:@"Sample 1"
                                       style:UIBarButtonItemStylePlain
                                       target:self
                                       action:@selector(importScrew:)];

    UIBarButtonItem *importLinkrodsBtn = [[UIBarButtonItem alloc]
                                          initWithTitle:@"Sample 2"
                                          style:UIBarButtonItemStylePlain
                                          target:self
                                          action:@selector(importLinkrods:)];


    UIBarButtonItem *addCubeBtn = [[UIBarButtonItem alloc]
                                   initWithTitle:@"Add cube"
                                   style:UIBarButtonItemStylePlain
                                   target:self
                                   action:@selector(addCube:)];

    UIBarButtonItem *displayAboutDlgBtn = [[UIBarButtonItem alloc]
                                           initWithTitle:@"About"
                                           style:UIBarButtonItemStylePlain
                                           target:self
                                           action:@selector(displayAboutDlg:)];

    [self.navigationItem setLeftBarButtonItems:[NSArray arrayWithObjects:importScrewBtn, importLinkrodsBtn, addCubeBtn, nil]];
    [self.navigationItem setRightBarButtonItem: displayAboutDlgBtn];
}

- (BOOL)gestureRecognizer:(UIGestureRecognizer *)gestureRecognizer
        shouldRecognizeSimultaneouslyWithGestureRecognizer:(UIGestureRecognizer *)otherGestureRecognizer
{
    const BOOL isPinchPanPair =
        ([gestureRecognizer isKindOfClass:UIPinchGestureRecognizer.class]
         && [otherGestureRecognizer isKindOfClass:UIPanGestureRecognizer.class])
        || ([gestureRecognizer isKindOfClass:UIPanGestureRecognizer.class]
            && [otherGestureRecognizer isKindOfClass:UIPinchGestureRecognizer.class]);
    return isPinchPanPair;
}

// =======================================================================
// function : zoomHandler
// purpose  :
// =======================================================================
- (void)zoomHandler:(UIPinchGestureRecognizer *)pinchRecognizer
{
    const UIGestureRecognizerState state = pinchRecognizer.state;
    if (state == UIGestureRecognizerStateEnded
        || state == UIGestureRecognizerStateCancelled
        || state == UIGestureRecognizerStateFailed) {
        if (_pinchRendering) {
            _pinchRendering = NO;
            [[self viewportView] endInteractiveRendering];
        }
        return;
    }

    if (_isPreviewMode) {
        return;
    }

    if (pinchRecognizer.numberOfTouches > 1) {
        if (state == UIGestureRecognizerStateBegan) {
            if (!_pinchRendering) {
                _pinchRendering = YES;
                [[self viewportView] beginInteractiveRendering];
            }
            _pinchPreviousTouch[0] = [pinchRecognizer locationOfTouch:0 inView:self.view];
            _pinchPreviousTouch[1] = [pinchRecognizer locationOfTouch:1 inView:self.view];
        } else if (state == UIGestureRecognizerStateChanged) {
            CGPoint aLastTouch[2] = {
                [pinchRecognizer locationOfTouch:0 inView:self.view],
                [pinchRecognizer locationOfTouch:1 inView:self.view]
            };

            const CGFloat drawableScale = [self drawableScale];
            double aPinchCenterXStart =
                (_pinchPreviousTouch[0].x + _pinchPreviousTouch[1].x) * drawableScale / 2.0;
            double aPinchCenterYStart =
                (_pinchPreviousTouch[0].y + _pinchPreviousTouch[1].y) * drawableScale / 2.0;

            double aStartDist = Sqrt( ( _pinchPreviousTouch[0].x - _pinchPreviousTouch[1].x ) * ( _pinchPreviousTouch[0].x - _pinchPreviousTouch[1].x ) +
                                     ( _pinchPreviousTouch[0].y - _pinchPreviousTouch[1].y ) * ( _pinchPreviousTouch[0].y - _pinchPreviousTouch[1].y ) );
            double anEndDist = Sqrt( ( aLastTouch[0].x - aLastTouch[1].x ) * ( aLastTouch[0].x - aLastTouch[1].x ) +
                                    ( aLastTouch[0].y - aLastTouch[1].y ) * ( aLastTouch[0].y - aLastTouch[1].y ) );

            double aDeltaDist = (anEndDist - aStartDist) * drawableScale;

            _viewer->Zoom(aPinchCenterXStart, aPinchCenterYStart, aDeltaDist);
            [self requestRender];

            _pinchPreviousTouch[0] = aLastTouch[0];
            _pinchPreviousTouch[1] = aLastTouch[1];
        }
    }
}

// =======================================================================
// function : panHandler
// purpose  :
// =======================================================================
- (void)panHandler:(UIPanGestureRecognizer *)panRecognizer
{
    const UIGestureRecognizerState state = panRecognizer.state;
    if (state == UIGestureRecognizerStateEnded
        || state == UIGestureRecognizerStateCancelled
        || state == UIGestureRecognizerStateFailed) {
        if (_panRendering) {
            _panRendering = NO;
            [[self viewportView] endInteractiveRendering];
        }
        return;
    }

    if (_isPreviewMode) {
        return;
    }

    if (panRecognizer.numberOfTouches > 1) {
        if (state == UIGestureRecognizerStateBegan) {
            if (!_panRendering) {
                _panRendering = YES;
                [[self viewportView] beginInteractiveRendering];
            }
            [panRecognizer setTranslation:CGPointZero inView:self.view];

        } else if (state == UIGestureRecognizerStateChanged) {
            const CGFloat drawableScale = [self drawableScale];
            const CGPoint translation = [panRecognizer translationInView:self.view];
            [panRecognizer setTranslation:CGPointZero inView:self.view];
            const int deltaX = (int)lround(translation.x * drawableScale);
            const int deltaY = (int)lround(-translation.y * drawableScale);
            if (deltaX != 0 || deltaY != 0) {
                // Starting a fresh relative pan from the current camera for each
                // incremental delta keeps panning composable with simultaneous
                // pinch updates instead of repeatedly restoring an old camera.
                _viewer->Pan(deltaX, deltaY, Standard_True);
                [self requestRender];
            }
        }
    }
}

// =======================================================================
// function : tapHandler
// purpose  :
// =======================================================================
#ifdef DEBUG
// The lesson's raster hit token is not the native selection result. Retain
// actual UIKit coordinates and native owners at each boundary without moving
// detection, changing selection, or asking the full scene builder to publish.
- (NSArray<NSString *> *)debugNativeFaceTapTrace {
    return NSThread.isMainThread ? ([_debugNativeFaceTapTrace copy] ?: @[]) : @[];
}

- (void)debugTraceNativeFaceTap:(CGPoint)point stage:(NSString *)stage {
    const auto environment = NSProcessInfo.processInfo.environment;
    if (![environment[@"SHAPEYARD_NATIVE_FACE_TRACE"] isEqualToString:@"1"]
        && ![environment[@"SHAPEYARD_MUG_LESSON_STL_ORACLE"] isEqualToString:@"1"]
        && ![NSProcessInfo.processInfo.arguments containsObject:@"-ShapeyardMugLessonStart"]) return;
    if (!_viewer || !_viewer->getShapeInteractor()
        || _viewer->getShapeInteractor()->getSelectionMode() != ShapeSelectionMode::Face) return;
    try {
        const auto context = _viewer->AisContext(); const auto view = _viewer->ActiveView();
        if (context.IsNull() || view.IsNull()) return;
        int width=0,height=0; view->Window()->Size(width,height);
        int detected=-1,nearest=-1; double radius=-1;
        NSMutableArray<NSString *> *ranks = [NSMutableArray array];
        const auto owner = Handle(StdSelect_BRepOwner)::DownCast(context->DetectedOwner());
        const auto shape = owner.IsNull() ? Handle(AIS_Shape)() : Handle(AIS_Shape)::DownCast(owner->Selectable());
        if (!shape.IsNull() && owner->HasShape()) {
            TopTools_IndexedMapOfShape faces;
            for (TopExp_Explorer face(shape->Shape(),TopAbs_FACE); face.More() && faces.Extent()<=128; face.Next())
                faces.Add(face.Current());
            if (faces.Extent()<=128) {
                detected=faces.FindIndex(owner->Shape())-1;
                if (owner->Shape().ShapeType()==TopAbs_FACE) {
                    BRepAdaptor_Surface surface(TopoDS::Face(owner->Shape()),Standard_True);
                    if (surface.GetType()==GeomAbs_Cylinder)
                        radius=surface.Cylinder().Radius()*std::abs(shape->Transformation().ScaleFactor());
                }
                const auto selector=context->MainSelector();
                for (int rank=1; !selector.IsNull() && rank<=selector->NbPicked() && rank<=8; ++rank) {
                    const auto candidate=Handle(StdSelect_BRepOwner)::DownCast(selector->Picked(rank));
                    if (!candidate.IsNull() && candidate->HasShape())
                        [ranks addObject:[NSString stringWithFormat:@"%d:%.6f",faces.FindIndex(candidate->Shape())-1,
                            selector->PickedData(rank).Depth]];
                }
                // Only intersect after the native pick. Earlier stages observe
                // counts and the previous hover, never prime a detection probe.
                if ([stage isEqualToString:@"after-select"]) {
                    double x,y,z,dx,dy,dz;
                    view->ConvertWithProj((int)point.x,(int)point.y,x,y,z,dx,dy,dz);
                    const auto inverse=shape->Transformation().Inverted();
                    IntCurvesFace_ShapeIntersector ray; ray.Load(shape->Shape(),1e-7);
                    ray.Perform(gp_Lin(gp_Pnt(x,y,z).Transformed(inverse),gp_Dir(dx,dy,dz).Transformed(inverse)),0,1e6);
                    double depth=1e6;
                    if (ray.IsDone()) for (int hit=1;hit<=ray.NbPnt();++hit)
                        if (ray.WParameter(hit)>=0 && ray.WParameter(hit)<depth) {
                            depth=ray.WParameter(hit); nearest=faces.FindIndex(ray.Face(hit))-1;
                        }
                }
            }
        }
        const auto eye=view->Camera()->Eye();
        NSString *entry = [NSString stringWithFormat:@"stage=%@ xy=%.0f,%.0f viewport=%d,%d count=%d hit=%d face=%d radius=%.6f ray=%d ranks=%@ eye=%.6f,%.6f,%.6f clip=%.6f,%.6f",
            stage,point.x,point.y,width,height,_viewer->selectedCount(),context->HasDetected(),detected,radius,nearest,
            [ranks componentsJoinedByString:@","],eye.X(),eye.Y(),eye.Z(),view->Camera()->ZNear(),view->Camera()->ZFar()];
        if (!_debugNativeFaceTapTrace) _debugNativeFaceTapTrace = [NSMutableArray array];
        [_debugNativeFaceTapTrace addObject:entry];
        if (_debugNativeFaceTapTrace.count > 32) [_debugNativeFaceTapTrace removeObjectAtIndex:0];
        NSLog(@"SHAPEYARD_NATIVE_FACE_TAP %@", entry);
    } catch (...) {
        NSLog(@"SHAPEYARD_NATIVE_FACE_TAP stage=%@ unavailable",stage);
    }
}
#endif

- (void)tapHandler:(UITapGestureRecognizer *)tapRecognizer
{
    if (_suppressTapSelectionForManipulatorInteraction) {
        _suppressTapSelectionForManipulatorInteraction = NO;
        return;
    }
    if (_isPreviewMode) {
        return;
    }
    const std::shared_ptr<ObjectInteractor> objectInteractor =
        _viewer == nullptr ? nullptr : _viewer->getObjectInteractor();
    if (objectInteractor != nullptr
        && (objectInteractor->hasActiveLinearArray()
            || objectInteractor->hasActiveRadialArray())) {
        // Array tools capture exactly one source. Keep viewport taps from
        // changing the visible selection while Apply still targets that
        // captured source; camera gestures remain available.
        return;
    }
    const std::shared_ptr<ShapeInteractor> shapeInteractor =
        _viewer == nullptr ? nullptr : _viewer->getShapeInteractor();
    if (shapeInteractor != nullptr
        && shapeInteractor->isShellSelectionFrozen()) {
        // Shell owns one source and an opening set. Camera gestures stay
        // available, but a tap cannot retarget the immutable preview lease.
        return;
    }

    const CGPoint aTapPoint =
        [self drawablePointForPoint:[tapRecognizer locationInView:self.view]];
	if (!_isConstructorMode) {
#ifdef DEBUG
        [self debugTraceNativeFaceTap:aTapPoint stage:@"before-delegate"];
#endif
		if (_delegate
			&& [_delegate respondsToSelector:
				@selector(viewer:willSelectAtDrawablePoint:drawableSize:)]) {
			[_delegate viewer:self
				willSelectAtDrawablePoint:aTapPoint
				             drawableSize:self.drawableSize];
		}
#ifdef DEBUG
        [self debugTraceNativeFaceTap:aTapPoint stage:@"before-select"];
#endif
		_viewer->Select((int)aTapPoint.x, (int)aTapPoint.y);
#ifdef DEBUG
        [self debugTraceNativeFaceTap:aTapPoint stage:@"after-select"];
#endif
	}

	[self checkSelections];
#ifdef DEBUG
    [self debugTraceNativeFaceTap:aTapPoint stage:@"after-publication"];
#endif
	[self requestRender];
}

-(void) checkSelections {
    _committedBoundariesDirty = YES;
    // Observe synchronously, before the debounced UI delegate. This also
    // catches a select/deselect round trip between two immutable scene reads.
    if (_viewer != nullptr) _viewer->observeNativePlanningInteraction();
	bool isSelected = _viewer->getObjectInteractor()->isSelected();
	bool isManipulatorAttached = _viewer->getObjectInteractor()->isManipulatorAttached();
	unsigned char selections = Core3DViewer::kSelectionTypeNone;
	if (isSelected) {
		selections |= Core3DViewer::kSelectionTypeObject;
	}
	if (isManipulatorAttached) {
		selections |= Core3DViewer::kSelectionTypeManipulator;
	}
	
	if (_delegate && [_delegate respondsToSelector:@selector(viewer:didChangeSelections:)]) {
		__weak typeof(self) weakSelf = self;
		[_delegate viewer:weakSelf didChangeSelections:selections];
	}
}

// =======================================================================
// function : importScrew
// purpose  :
// =======================================================================
- (void)importScrew:(UIBarButtonItem *)theSender
{
    NSString* aNsPath = [[NSBundle mainBundle] pathForResource:@"screw"
                                                        ofType:@"step"];
    std::string aPath = std::string([aNsPath UTF8String]);

    _viewer->ImportSTEP(aPath);
    _viewer->FitAll();
    [self requestRender];
}

// =======================================================================
// function : importLinkrods
// purpose  :
// =======================================================================
- (void)importLinkrods:(UIBarButtonItem *)theSender
{
    NSString* aNsPath = [[NSBundle mainBundle] pathForResource:@"linkrods"
                                                        ofType:@"step"];
    std::string aPath = std::string([aNsPath UTF8String]);

    _viewer->ImportSTEP(aPath);
    _viewer->FitAll();
    [self requestRender];
}

- (void)addCube:(UIBarButtonItem *)theSender {
    if (_viewer == nullptr || !_viewer->canBeginCommittedEdit()) {
        return;
    }
    __auto_type stlFilename = _viewer->addTestPrimitives();
    _viewer->FitAll();
    [self requestRender];
    [self shareFile:stlFilename];
}

- (void)shareFile:(NSString *)filepath {
    __auto_type activityController = [[UIActivityViewController alloc] initWithActivityItems:@[[NSURL fileURLWithPath:filepath]]
                                                                       applicationActivities:nil];
    if ([UIDevice currentDevice].userInterfaceIdiom == UIUserInterfaceIdiomPad) {
        activityController.popoverPresentationController.sourceView = self.view;
        activityController.popoverPresentationController.permittedArrowDirections = UIPopoverArrowDirectionAny;
        activityController.popoverPresentationController.sourceRect = CGRectMake(self.view.frame.size.width/2.0,
                                                                                 self.view.frame.size.height/2.0, 1.0, 1.0);
    }
    [self presentViewController:activityController animated:YES completion:nil];
}

- (void)addTestPrimitives {
    if (_viewer == nullptr || !_viewer->canBeginCommittedEdit()) {
        return;
    }
    _viewer->addTestPrimitives();
    _viewer->FitAll();
    [self requestRender];
}

- (void)addPrimitivesFromJSON:(NSString *)json {
    if (_viewer == nullptr || !_viewer->canBeginCommittedEdit()) {
        return;
    }
    _viewer->addPrimitivesFromJSON(json);
    [self requestRender];
}

- (void)addPrimitive:(PrimitiveType)primitiveType {
    if (_viewer == nullptr || !_viewer->canBeginCommittedEdit()) {
        return;
    }
    _viewer->addPrimitive(primitiveType);
    [self requestRender];
}

- (void)selectLastObject {
    if (_viewer == nullptr || !_viewer->canBeginCommittedEdit()) return;
    _viewer->getObjectInteractor()->selectLastObject();
    [self requestRender];
}

- (void)deleteSelected {
    if (![NSThread isMainThread]
        || _viewer == nullptr
        || !_viewer->canBeginCommittedEdit()
        || _viewer->getShapeInteractor() == nullptr
        || _viewer->getObjectInteractor() == nullptr
        || !_viewer->getShapeInteractor()
            ->selectionModeAuthorityIsExact()
        || _viewer->getShapeInteractor()->getSelectionMode()
            != ShapeSelectionMode::WholeShape) {
        return;
    }
    _viewer->getObjectInteractor()->deleteSelected();
    [self requestRender];
}

-(NSString*) statusString {
	auto interactor = _viewer->getObjectInteractor();
	auto transform = interactor->manipulatorTransform();
	if (transform.IsNull()) {
		return @"No active selection";
	}
	auto pos = interactor->manipulatorPosition();
	auto rot = transform->Trsf().GetRotation();
	Standard_Real rotX, rotY, rotZ;
	rot.GetEulerAngles(gp_YawPitchRoll, rotX, rotY, rotZ);
	NSString* status = [NSString stringWithFormat:@"Coord: x%.3f, y%.3f, z%.3f. Angle:x%.3f, y%.3f, z%.3f",
			pos.X(), pos.Y(), pos.Z(), rotX / M_PI * 180, rotY / M_PI * 180, rotZ / M_PI * 180];
	return status;
}

- (void)selectAll {
	if (![NSThread isMainThread]
        || _viewer == nullptr
        || !_viewer->canBeginCommittedEdit()
        || _viewer->getShapeInteractor() == nullptr
        || _viewer->getObjectInteractor() == nullptr
        || !_viewer->getShapeInteractor()
            ->selectionModeAuthorityIsExact()
        || _viewer->getShapeInteractor()->getSelectionMode()
            != ShapeSelectionMode::WholeShape) {
        return;
    }
	_viewer->getObjectInteractor()->selectAll();
	[self checkSelections];
	[self requestRender];
}

- (void)duplicateSelected {
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->getObjectInteractor() == nullptr) {
        return;
    }
    if (_viewer->hasUnresolvedOrdinaryEdit()) { return; }
    const BOOL recoveryPending = _viewer->hasUnresolvedDuplicate();
    if (!recoveryPending
        && (!_viewer->canBeginCommittedEdit()
            || _viewer->getShapeInteractor() == nullptr
            || !_viewer->getShapeInteractor()
                ->selectionModeAuthorityIsExact()
            || _viewer->getShapeInteractor()->getSelectionMode()
                != ShapeSelectionMode::WholeShape)) {
        return;
    }
    _viewer->getObjectInteractor()->duplicateSelected();
    [self checkSelections];
    [self requestRender];
}

- (BOOL)hasUnresolvedDuplicate {
    return _viewer != nullptr && _viewer->hasUnresolvedDuplicate();
}
- (BOOL)hasUnresolvedEdit {
    return _viewer != nullptr && _viewer->hasUnresolvedEdit();
}
- (BOOL)hasUnresolvedOrdinaryEdit {
    return _viewer != nullptr && _viewer->hasUnresolvedOrdinaryEdit();
}
- (BOOL)prepareOrdinaryEditForDocumentClose {
    if (![NSThread isMainThread] || _viewer == nullptr) { return NO; }
    if (!_viewer->hasUnresolvedOrdinaryEdit()) { return YES; }
    const auto viewer = _viewer;
    GLView* view = self.isViewLoaded ? [self viewportView] : nil;
    if (view == nil) { return NO; }
    const BOOL performed = [view performWithRenderingContext:^{
        (void)viewer->reconcileOrdinaryEdit();
    }];
    if (!performed || viewer->hasUnresolvedOrdinaryEdit()) { return NO; }
    [self refreshSelectionState];
    [self requestRender];
    return YES;
}

- (void)deselectAll {
    if (_viewer != nullptr && _viewer->getShapeInteractor() != nullptr
        && !_viewer->getShapeInteractor()->cancelExtrusion()) {
        [self requestRender];
        return;
    }
    if (_viewer != nullptr && _viewer->getShapeInteractor() != nullptr
        && !_viewer->getShapeInteractor()->cancelShell()) {
        [self requestRender];
        return;
    }
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr
        && _viewer->getObjectInteractor()->hasActiveLinearArray()
        && !_viewer->getObjectInteractor()->cancelLinearArray()) {
        [self requestRender];
        return;
    }
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr
        && _viewer->getObjectInteractor()->hasActiveRadialArray()
        && !_viewer->getObjectInteractor()->cancelRadialArray()) {
        [self requestRender];
        return;
    }
    _viewer->deselectAll();
    [self requestRender];
}

- (BOOL)canIssueModelingPlanningContext {
    return [NSThread isMainThread] && _didSetupViewer && !_isPreviewMode
        && !_isConstructorMode && _viewer != nullptr && _viewer->canBeginCommittedEdit();
}

- (BOOL)canIssueModelingPlanningContextHoldingSplineProfileOpening:(Core3DSplineProfileEditingOpening *)opening {
    return [NSThread isMainThread] && _didSetupViewer && !_isPreviewMode
        && !_isConstructorMode && _viewer != nullptr
        && _viewer->canBeginCommittedEditHolding([opening core3d_nativeOpeningContext]);
}

- (void)refreshSelectionState {
    [self checkSelections];
}

- (BOOL)isSelected {
    return _viewer->getObjectInteractor()->isSelected();
}

- (void)fitAll {
    _viewer->FitAll();
    [self requestRender];
}

- (void)setPreviewMode {
    _isPreviewMode = YES;
    [self endActiveRenderingInteractions];
}

- (void)undo {
    [self performNativeHistoryDirection:NativeHistoryDirection::Undo];
}

- (void)redo {
    [self performNativeHistoryDirection:NativeHistoryDirection::Redo];
}

- (void)performNativeHistoryDirection:(NativeHistoryDirection)direction {
    if (_viewer == nullptr) return;
    Core3DViewController *rawOwner = Core3DVisibleRawR2BevelOwner(self);
    if ([rawOwner core3d_hasVisibleRawR2Bevel]) {
        if ([rawOwner core3d_cancelVisibleRawR2Bevel]) {
            [self checkSelections];
            [self requestRender];
            if (_delegate != nil && [_delegate respondsToSelector:
                @selector(viewer:didEndPrimaryInteractionCancelled:)]) {
                [_delegate viewer:self didEndPrimaryInteractionCancelled:YES];
            }
        }
        return;
    }
    const NativeHistoryTransition result = _viewer->performHistory(direction);
    if (result.reconcileBooleanTool) {
        if (_delegate != nil && [_delegate respondsToSelector:
            @selector(viewerDidFailToRetainBooleanMode:)]) {
            [_delegate viewerDidFailToRetainBooleanMode:self];
        } else {
            _viewer->getObjectInteractor()->setManipulatorType(
                PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
        }
    }
    if (result.refreshSelection) [self checkSelections];
    if (result.requestRender) [self requestRender];
    if (result.primaryInteractionCancelled && _delegate != nil
        && [_delegate respondsToSelector:
            @selector(viewer:didEndPrimaryInteractionCancelled:)]) {
        [_delegate viewer:self didEndPrimaryInteractionCancelled:YES];
    }
}

- (Core3DSelectionTypeChangeResult)
    trySetSelectionType:(PrimitiveSelectionType)type
{
    if (_viewer != nullptr) _viewer->observeNativePlanningInteraction();
    if (![NSThread isMainThread]) {
        return Core3DSelectionTypeChangeResultWrongThread;
    }
    ShapeSelectionMode selectionMode = ShapeSelectionMode::WholeShape;
    if (!TryShapeSelectionMode(type, selectionMode)) {
        return Core3DSelectionTypeChangeResultUnsupported;
    }
    if (!_didSetupViewer
        || _viewer == nullptr
        || _viewer->AisContext().IsNull()
        || _viewer->ActiveView().IsNull()
        || _viewer->getDocument().IsNull()
        || _viewer->getDocument()->Document().IsNull()
        || _viewer->getObjectInteractor() == nullptr
        || _viewer->getShapeInteractor() == nullptr) {
        return Core3DSelectionTypeChangeResultNotReady;
    }

    const std::shared_ptr<ObjectInteractor> objectInteractor =
        _viewer->getObjectInteractor();
    const std::shared_ptr<ShapeInteractor> shapeInteractor =
        _viewer->getShapeInteractor();
    // Outcome-unknown controllers own the only authoritative reconciliation
    // state for a possibly committed operation. Refuse before cancelling a
    // gesture or preview so Busy remains fully side-effect free. Ordinary
    // Selecting/Ready/Failed previews continue through their typed cancel path.
    if ((_viewer->hasUnresolvedOrdinaryEdit() || objectInteractor->hasUnresolvedDuplicate())
        || objectInteractor->hasUnresolvedBoolean()
        || objectInteractor->mirrorPreviewState()
            == MirrorPreviewState::OutcomeUnknown
        || objectInteractor->linearArrayPreviewState()
            == LinearArrayPreviewState::OutcomeUnknown
        || objectInteractor->radialArrayPreviewState()
            == RadialArrayPreviewState::OutcomeUnknown
        || shapeInteractor->shellPreviewState()
            == ShellPreviewState::OutcomeUnknown
        || shapeInteractor->extrusionPreviewState()
            == ExtrusionPreviewState::OutcomeUnknown) {
        return Core3DSelectionTypeChangeResultBusy;
    }

    const PrimitiveGizmoType currentType = [self getGizmoType];
    if (selectionMode == ShapeSelectionMode::WholeShape
        && shapeInteractor->getSelectionMode()
            == ShapeSelectionMode::WholeShape
        && currentType == PrimitiveGizmoTypeMirror
        && objectInteractor->isPickingMirrorPlane()
        && objectInteractor->mirrorPlanePickingAuthorityMatches()) {
        // Mirror face picking intentionally adds Face mode alongside the
        // accepted Object mode on committed bodies. Until its saved-mode
        // ledger restores those presentations, a same-mode rail tap is Busy:
        // it must neither claim ordinary authority nor destroy the picker.
        return Core3DSelectionTypeChangeResultBusy;
    }
    const BOOL readyExtrusionExactlyOwnsSuspendedPresentations =
        currentType == PrimitiveGizmoTypeExtrude
        && _viewer->selectionModeAuthorityAllowsRetainedOperation();
    if (shapeInteractor->getSelectionMode() == selectionMode
        && (shapeInteractor->selectionModeAuthorityIsExact()
            || readyExtrusionExactlyOwnsSuspendedPresentations)) {
        // A Ready Extrusion deliberately deactivates exactly its original and
        // candidate. Re-tapping the accepted rail mode preserves it only after
        // every unowned committed presentation and picker tolerance are proven
        // canonical. OutcomeUnknown was rejected above; unrelated same-enum
        // drift still enters the normal cancellation-and-repair path.
        return Core3DSelectionTypeChangeResultSucceeded;
    }

    try {
        // Validation and readiness checks above intentionally precede every
        // cancellation so malformed or premature requests are side-effect
        // free.
        objectInteractor->cancelInteraction();
        if (currentType == PrimitiveGizmoTypeChamfer) {
            if (!shapeInteractor->resetWireframeTemplateShape()) {
                [self checkSelections];
                [self requestRender];
                return Core3DSelectionTypeChangeResultPresentationFailure;
            }
        } else if (IsBooleanGizmo(currentType)) {
            if (![self retireBooleanActionForGizmoType:currentType]) {
                [self checkSelections];
                [self requestRender];
                return Core3DSelectionTypeChangeResultPresentationFailure;
            }
        } else if (currentType == PrimitiveGizmoTypeMirror) {
            if (!objectInteractor->cancelMirror()) {
                [self checkSelections];
                [self requestRender];
                return Core3DSelectionTypeChangeResultPresentationFailure;
            }
        } else if (currentType == PrimitiveGizmoTypeLinearArray) {
            if (!objectInteractor->cancelLinearArray()) {
                [self checkSelections];
                [self requestRender];
                return Core3DSelectionTypeChangeResultPresentationFailure;
            }
        } else if (currentType == PrimitiveGizmoTypeRadialArray) {
            if (!objectInteractor->cancelRadialArray()) {
                [self checkSelections];
                [self requestRender];
                return Core3DSelectionTypeChangeResultPresentationFailure;
            }
        } else if (currentType == PrimitiveGizmoTypeExtrude) {
            if (!shapeInteractor->cancelExtrusion()) {
                [self requestRender];
                return Core3DSelectionTypeChangeResultPresentationFailure;
            }
        } else if (currentType == PrimitiveGizmoTypeShell) {
            if (!shapeInteractor->cancelShell()) {
                [self requestRender];
                return Core3DSelectionTypeChangeResultPresentationFailure;
            }
        }
    } catch (...) {
        [self checkSelections];
        [self requestRender];
        return Core3DSelectionTypeChangeResultPresentationFailure;
    }

    const ShapeSelectionModeChangeResult nativeResult =
        shapeInteractor->setSelectionMode(selectionMode);
    if (nativeResult != ShapeSelectionModeChangeResult::Succeeded) {
        if (IsBooleanGizmo(currentType)) {
            (void)[self restoreBooleanActionForRetainedGizmoType:currentType];
        }
        [self checkSelections];
        [self requestRender];
        return PublicSelectionTypeChangeResult(nativeResult);
    }

    // This preserves the legacy retained-Boolean reconciliation. A failed
    // restore notifies the owner through the existing delegate path; the
    // successfully accepted selection authority remains truthful.
    (void)[self restoreBooleanActionForRetainedGizmoType:currentType];
    // setSelectionMode() clears the old owners only after the requested modes
    // are verified. Reconcile the selection-dependent manipulator immediately,
    // including same-enum authority repair, before publishing public state.
    objectInteractor->attachManipulatorToSelection(false);
    [self requestRender];
    return Core3DSelectionTypeChangeResultSucceeded;
}

- (void)setSelectionType:(PrimitiveSelectionType)type {
    (void)[self trySetSelectionType:type];
}

- (PrimitiveSelectionType)getSelectionType {
    if (_viewer == nullptr || _viewer->getShapeInteractor() == nullptr) {
        return PrimitiveSelectionTypeNone;
    }
    if (!_viewer->getShapeInteractor()->selectionModeAuthorityIsExact()) {
        return PrimitiveSelectionTypeNone;
    }
    switch (_viewer->getShapeInteractor()->getSelectionMode()) {
        case ShapeSelectionMode::WholeShape:
            return PrimitiveSelectionTypeShape;
        case ShapeSelectionMode::Edge:
            return PrimitiveSelectionTypeEdge;
        case ShapeSelectionMode::Face:
            return PrimitiveSelectionTypeFace;
        case ShapeSelectionMode::Vertex:
            return PrimitiveSelectionTypeVertex;
        case ShapeSelectionMode::Wire:
        default:
            return PrimitiveSelectionTypeNone;
    }
}

- (void)setGizmo:(Handle(Core3DManipulator))manipulator {
	if (_viewer == nullptr || _viewer->hasUnresolvedEdit()) {
		return;
	}
	_viewer->getObjectInteractor()->setManipulator(manipulator);
	[self requestRender];
}

- (void)setPrimitiveTransparent:(Handle(AIS_InteractiveObject))primitive transparent:(bool)set {
	_viewer->getObjectInteractor()->setObjectTransparent(primitive, set);
	[self requestRender];
}

- (void)setGizmoType:(PrimitiveGizmoType)type {
    if (_viewer != nullptr) _viewer->observeNativePlanningInteraction();
	if (_viewer == nullptr || _viewer->hasUnresolvedEdit()) {
		return;
	}
	const PrimitiveGizmoType previousType = [self getGizmoType];
	if (type != PrimitiveGizmoTypeNone) {
		const std::shared_ptr<ShapeInteractor> shapeInteractor =
			_viewer->getShapeInteractor();
		const std::shared_ptr<ObjectInteractor> objectInteractor =
			_viewer->getObjectInteractor();
		if (shapeInteractor == nullptr
			|| objectInteractor == nullptr
			|| !SelectionModeAllowsGizmo(
				shapeInteractor->getSelectionMode(), type)) {
			return;
		}
		const bool hasExactOrdinaryAuthority =
			shapeInteractor->selectionModeAuthorityIsExact();
		const bool hasRetainedOperationAuthority = previousType == type
			&& _viewer->selectionModeAuthorityAllowsRetainedOperation();
		const bool hasMirrorPlanePickingAuthority =
			previousType == PrimitiveGizmoTypeMirror
			&& type == PrimitiveGizmoTypeMirror
			&& shapeInteractor->getSelectionMode()
				== ShapeSelectionMode::WholeShape
			&& [self isPickingMirrorPlane]
			&& objectInteractor->mirrorPlanePickingAuthorityMatches();
		if (!hasExactOrdinaryAuthority
			&& !hasRetainedOperationAuthority
			&& !hasMirrorPlanePickingAuthority) {
			return;
		}
		// Chamfer admission is proven before any prior-tool retirement or
		// manipulator mutation. begin recaptures after this preflight so stale
		// selection cannot cross the operation boundary.
		if (type == PrimitiveGizmoTypeChamfer
			&& !shapeInteractor->hasActiveBevel()
			&& !shapeInteractor->canBeginBevelSelection()) {
			return;
		}
	}
	if (previousType == type) {
		if (type == PrimitiveGizmoTypeChamfer
			&& _viewer != nullptr
			&& _viewer->getShapeInteractor() != nullptr
			&& (!_viewer->getShapeInteractor()->hasActiveBevel()
				|| (_viewer->getShapeInteractor()->bevelPreviewState()
						== BevelPreviewState::Selecting
					&& !_viewer->getShapeInteractor()
						->isBevelSelectionFrozen()))
			&& _viewer->getShapeInteractor()->saveSelectionEdges() == 0
			&& !_viewer->getShapeInteractor()->hasActiveBevel()) {
			_viewer->getObjectInteractor()->setManipulatorType(
				PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
		}
		if (type == PrimitiveGizmoTypeExtrude
			&& _viewer != nullptr
			&& _viewer->getShapeInteractor() != nullptr
			&& !_viewer->getShapeInteractor()->hasActiveExtrusion()
			&& !_viewer->getShapeInteractor()->beginExtrusionSelection()) {
			_viewer->getObjectInteractor()->setManipulatorType(
				PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
		}
		if (type == PrimitiveGizmoTypeShell
			&& _viewer != nullptr
			&& _viewer->getShapeInteractor() != nullptr
			&& !_viewer->getShapeInteractor()->hasActiveShell()
			&& !_viewer->getShapeInteractor()->beginShellSelection()) {
			_viewer->getObjectInteractor()->setManipulatorType(
				PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
		}
		if (IsBooleanGizmo(type)) {
			(void)[self restoreBooleanActionForRetainedGizmoType:type];
		}
		if (type == PrimitiveGizmoTypeLinearArray
			&& _viewer != nullptr
			&& _viewer->getObjectInteractor() != nullptr
			&& !_viewer->getObjectInteractor()->hasActiveLinearArray()) {
			(void)_viewer->getObjectInteractor()->beginLinearArray();
		}
		if (type == PrimitiveGizmoTypeRadialArray
			&& _viewer != nullptr
			&& _viewer->getObjectInteractor() != nullptr
			&& !_viewer->getObjectInteractor()->hasActiveRadialArray()) {
			(void)_viewer->getObjectInteractor()->beginRadialArray();
		}
		[self requestRender];
		return;
	}

	// Resolve the previous tool before changing manipulator mode or capturing
	// selection for the next tool. Its AIS previews may refer to document labels
	// that the resolution step replaces or removes.
	if (previousType == PrimitiveGizmoTypeChamfer) {
		if (!_viewer->getShapeInteractor()->resetWireframeTemplateShape()) {
			[self checkSelections];
			[self requestRender];
			return;
		}
	} else if (IsBooleanGizmo(previousType)) {
		if (![self retireBooleanActionForGizmoType:previousType]) {
			[self checkSelections];
			[self requestRender];
			return;
		}
	} else if (previousType == PrimitiveGizmoTypeMirror) {
		if (!_viewer->getObjectInteractor()->cancelMirror()) {
			[self checkSelections];
			[self requestRender];
			return;
		}
	} else if (previousType == PrimitiveGizmoTypeLinearArray) {
		if (!_viewer->getObjectInteractor()->cancelLinearArray()) {
			[self checkSelections];
			[self requestRender];
			return;
		}
	} else if (previousType == PrimitiveGizmoTypeRadialArray) {
		if (!_viewer->getObjectInteractor()->cancelRadialArray()) {
			[self checkSelections];
			[self requestRender];
			return;
		}
	} else if (previousType == PrimitiveGizmoTypeExtrude) {
		if (!_viewer->getShapeInteractor()->cancelExtrusion()) {
			[self requestRender];
			return;
		}
	} else if (previousType == PrimitiveGizmoTypeShell) {
		if (!_viewer->getShapeInteractor()->cancelShell()) {
			[self requestRender];
			return;
		}
	}

    PrimitiveManipulatorType manipulatorType;
    switch (type) {
        case PrimitiveGizmoTypeMoveRotate:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate;
            break;
        case PrimitiveGizmoTypeScale:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeScale;
            break;
        case PrimitiveGizmoTypeChamfer:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer;
            break;
        case PrimitiveGizmoTypeNone:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
            break;
        case PrimitiveGizmoTypeSubtract:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract;
            break;
        case PrimitiveGizmoTypeUnion:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeUnion;
            break;
        case PrimitiveGizmoTypeIntersect:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeIntersect;
            break;
        case PrimitiveGizmoTypeMirror:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeMirror;
            break;
        case PrimitiveGizmoTypeMaterial:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeMaterial;
            break;
        case PrimitiveGizmoTypeExtrude:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeExtrude;
            break;
        case PrimitiveGizmoTypeLinearArray:
            manipulatorType =
                PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray;
            break;
        case PrimitiveGizmoTypeShell:
            manipulatorType =
                PrimitiveManipulatorType::PrimitiveGizmoTypeShell;
            break;
        case PrimitiveGizmoTypeRadialArray:
            manipulatorType =
                PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray;
            break;
        default:
            assert(false);
            break;
    }
    _viewer->getObjectInteractor()->setManipulatorType(manipulatorType);
	if (_viewer->getObjectInteractor()->getManipulatorType()
		!= manipulatorType) {
		[self requestRender];
		return;
	}
    if (type != PrimitiveGizmoTypeChamfer) {
        _visibleRawR2BevelOwnsChamferTool = NO;
    }
    if (type == PrimitiveGizmoTypeChamfer) {
		if (_viewer->getShapeInteractor()->saveSelectionEdges() == 0) {
			_viewer->getObjectInteractor()->setManipulatorType(
				PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
		}
	} else if (IsBooleanGizmo(type)) {
			Standard_Boolean forceActor = (type == PrimitiveGizmoTypeSubtract);
			BooleanAction action = BooleanAction::BooleanSubtract;
			if (!TryBooleanActionForGizmo(type, action)) {
				[self requestRender];
				return;
			}
			if (_viewer->getObjectInteractor()->beginBoolean(action)) {
				_viewer->getObjectInteractor()->fillSelectedState(
					forceActor,
					action);
			}
	} else if (type == PrimitiveGizmoTypeExtrude) {
		if (!_viewer->getShapeInteractor()->beginExtrusionSelection()) {
			_viewer->getObjectInteractor()->setManipulatorType(
				PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
		}
	} else if (type == PrimitiveGizmoTypeShell) {
		if (!_viewer->getShapeInteractor()->beginShellSelection()) {
			_viewer->getObjectInteractor()->setManipulatorType(
				PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
		}
	}
	[self requestRender];
}

- (PrimitiveGizmoType)getGizmoType {
    Core3DViewController *rawOwner = Core3DVisibleRawR2BevelOwner(self);
    if ([rawOwner core3d_hasVisibleRawR2Bevel]) {
        _visibleRawR2BevelOwnsChamferTool = YES;
    }
    if (_visibleRawR2BevelOwnsChamferTool)
        return PrimitiveGizmoTypeChamfer;
    if (_viewer == nullptr) { return PrimitiveGizmoTypeNone; }
    // Ordinary Round owns the Chamfer control surface through its typed
    // Bevel ledger. The ObjectInteractor can legitimately have no attached
    // manipulator after renderer publication; that presentation detail must
    // not retire Radius while the operation is still active.
    const std::shared_ptr<ShapeInteractor> shapeInteractor =
        _viewer->getShapeInteractor();
    if (shapeInteractor != nullptr && shapeInteractor->hasActiveBevel()) {
        return PrimitiveGizmoTypeChamfer;
    }
    const auto interactor = _viewer->getObjectInteractor();
    if (interactor == nullptr) { return PrimitiveGizmoTypeNone; }
    PrimitiveGizmoType type = PrimitiveGizmoTypeNone;
    switch (interactor->getManipulatorType()) {
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate:
            type = PrimitiveGizmoTypeMoveRotate;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeScale:
            type = PrimitiveGizmoTypeScale;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer:
            type = PrimitiveGizmoTypeChamfer;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeNone:
            type = PrimitiveGizmoTypeNone;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract:
            type = PrimitiveGizmoTypeSubtract;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeUnion:
            type = PrimitiveGizmoTypeUnion;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeIntersect:
            type = PrimitiveGizmoTypeIntersect;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMirror:
            type = PrimitiveGizmoTypeMirror;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMaterial:
            type = PrimitiveGizmoTypeMaterial;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeExtrude:
            type = PrimitiveGizmoTypeExtrude;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray:
            type = PrimitiveGizmoTypeLinearArray;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeShell:
            type = PrimitiveGizmoTypeShell;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray:
            type = PrimitiveGizmoTypeRadialArray;
            break;
        default:
            break;
    }
    return type;
}

- (void)setChamfer:(CGFloat)value {
    assert([self getGizmoType] == PrimitiveGizmoTypeChamfer);
    Core3DViewController *rawOwner = Core3DVisibleRawR2BevelOwner(self);
    if ([rawOwner core3d_hasVisibleRawR2Bevel]) {
        [rawOwner core3d_setVisibleRawR2BevelValue:value];
        [self requestRender];
        return;
    }
    (void)_viewer->getShapeInteractor()->setChamferValueForSelection(value);
    [self requestRender];
    //if (!result)
    //	std::cout << "incorrect chamfer value" << std::endl;
}

- (BOOL)canBeginRound {
    if (_viewer == nullptr || _viewer->hasUnresolvedEdit()
        || [self getSelectionType] != PrimitiveSelectionTypeEdge
        || [self getGizmoType] != PrimitiveGizmoTypeNone) {
        return NO;
    }
    const std::shared_ptr<ShapeInteractor> interactor =
        _viewer->getShapeInteractor();
    if (interactor == nullptr || !interactor->selectionModeAuthorityIsExact()) {
        return NO;
    }
    Core3DViewController *rawOwner = Core3DVisibleRawR2BevelOwner(self);
    return [rawOwner core3d_canBeginVisibleRawR2Bevel]
        || interactor->canBeginRoundSelection();
}

- (BOOL)beginRound {
    // Repeat the live proof in this call. A prior availability answer never
    // authorizes activation after the selection has changed.
    if (![self canBeginRound] || _viewer == nullptr) {
        return NO;
    }
    const std::shared_ptr<ShapeInteractor> shapeInteractor =
        _viewer->getShapeInteractor();
    const std::shared_ptr<ObjectInteractor> objectInteractor =
        _viewer->getObjectInteractor();
    if (shapeInteractor == nullptr || objectInteractor == nullptr) {
        return NO;
    }
    Core3DViewController *rawOwner = Core3DVisibleRawR2BevelOwner(self);
    if ([rawOwner core3d_canBeginVisibleRawR2Bevel]) {
        if (![rawOwner core3d_beginVisibleRawR2Bevel]
            || ![rawOwner core3d_hasVisibleRawR2Bevel]) {
            [self requestRender];
            return NO;
        }
        _visibleRawR2BevelOwnsChamferTool = YES;
        [self requestRender];
        return YES;
    }

    if (shapeInteractor->canBeginRoundSelection()) {
        objectInteractor->setManipulatorType(
            PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer);
        if (objectInteractor->getManipulatorType()
                != PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer
            || !shapeInteractor->beginRoundSelectionFromCurrentSelection()
            || !shapeInteractor->hasActiveBevel()) {
            (void)shapeInteractor->cancelChamfer();
            objectInteractor->setManipulatorType(
                PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
            [self requestRender];
            return NO;
        }
        _visibleRawR2BevelOwnsChamferTool = NO;
        [self requestRender];
        return YES;
    }

    [self requestRender];
    return NO;
}

- (BOOL)setRoundRadius:(CGFloat)radius {
    Core3DViewController *rawOwner = Core3DVisibleRawR2BevelOwner(self);
    if ([rawOwner core3d_hasVisibleRawR2Bevel]) {
        const double metersPerUnit =
            [rawOwner core3d_visibleRawR2BevelMetersPerUnit];
        const double amountMM = radius * metersPerUnit * 1000.0;
        if (!std::isfinite(radius) || radius <= 0.0
            || !std::isfinite(amountMM) || amountMM <= 0.0
            || amountMM > 20.0) return NO;
        [rawOwner core3d_setVisibleRawR2BevelValue:radius];
        [self requestRender];
        return YES;
    }
    if (_viewer == nullptr || [self getGizmoType] != PrimitiveGizmoTypeChamfer
        || _viewer->getShapeInteractor() == nullptr) {
        return NO;
    }
    const BOOL didSet = _viewer->getShapeInteractor()
        ->setRoundRadiusForSelection(radius);
    [self requestRender];
    return didSet;
}

- (double)roundMetersPerUnit {
    Core3DViewController *rawOwner = Core3DVisibleRawR2BevelOwner(self);
    if ([rawOwner core3d_hasVisibleRawR2Bevel]) {
        return [rawOwner core3d_visibleRawR2BevelMetersPerUnit];
    }
    if (_viewer == nullptr || [self getGizmoType] != PrimitiveGizmoTypeChamfer
        || _viewer->getShapeInteractor() == nullptr) {
        return 0.0;
    }
    return _viewer->getShapeInteractor()->roundMetersPerUnit();
}

- (double)roundMaximumRadiusMM {
    Core3DViewController *rawOwner = Core3DVisibleRawR2BevelOwner(self);
    if ([rawOwner core3d_hasVisibleRawR2Bevel]) return 20.0;
    if (_viewer == nullptr || [self getGizmoType] != PrimitiveGizmoTypeChamfer
        || _viewer->getShapeInteractor() == nullptr) {
        return 0.0;
    }
    return _viewer->getShapeInteractor()->roundMaximumRadiusMM();
}

- (Core3DRoundFailureObservation)roundFailureObservation {
    Core3DRoundFailureObservation result = {
        0,
        Core3DRoundFailureKindNone,
    };
    Core3DViewController *rawOwner = Core3DVisibleRawR2BevelOwner(self);
    if ([rawOwner core3d_hasVisibleRawR2Bevel]) {
        return [rawOwner core3d_visibleRawR2RoundFailureObservation];
    }
    if (_viewer == nullptr || [self getGizmoType] != PrimitiveGizmoTypeChamfer
        || _viewer->getShapeInteractor() == nullptr) {
        return result;
    }
    const BevelFailureObservation observation =
        _viewer->getShapeInteractor()->roundFailureObservation();
    result.generation = observation.generation;
    switch (observation.kind) {
        case BevelFailureKind::None:
            result.kind = Core3DRoundFailureKindNone;
            break;
        case BevelFailureKind::RadiusRefused:
            result.kind = Core3DRoundFailureKindRadiusRefused;
            break;
        case BevelFailureKind::Other:
            result.kind = Core3DRoundFailureKindOther;
            break;
    }
    return result;
}

- (BOOL)applyChamfer {
    Core3DViewController *rawOwner = Core3DVisibleRawR2BevelOwner(self);
    if ([rawOwner core3d_hasVisibleRawR2Bevel]) {
        const BOOL committed = [rawOwner core3d_applyVisibleRawR2Bevel];
        if (committed) {
            _visibleRawR2BevelOwnsChamferTool = NO;
        }
        [self checkSelections];
        [self requestRender];
        return committed;
    }
    if ([self getGizmoType] != PrimitiveGizmoTypeChamfer
        || _viewer == nullptr
        || _viewer->getShapeInteractor() == nullptr) {
        return NO;
    }
    const BevelApplyResult result =
        _viewer->getShapeInteractor()->applyBevel();
    if (result == BevelApplyResult::AppliedNeedsDocumentRedraw) {
        _viewer->redrawDocument();
    }
    [self checkSelections];
    [self requestRender];
    return result != BevelApplyResult::NoChange;
}

- (BOOL)cancelChamfer {
	Core3DViewController *rawOwner = Core3DVisibleRawR2BevelOwner(self);
	if ([rawOwner core3d_hasVisibleRawR2Bevel]) {
		const BOOL didCancel = [rawOwner core3d_cancelVisibleRawR2Bevel];
		if (didCancel) {
			_visibleRawR2BevelOwnsChamferTool = NO;
		}
		[self checkSelections];
		[self requestRender];
		return didCancel;
	}
	if (_viewer == nullptr || _viewer->getShapeInteractor() == nullptr) {
		return NO;
	}
	const BOOL didCancel =
		_viewer->getShapeInteractor()->cancelChamfer();
	[self checkSelections];
	[self requestRender];
	return didCancel;
}

- (BOOL)canApplyChamfer {
    Core3DViewController *rawOwner = Core3DVisibleRawR2BevelOwner(self);
    if ([rawOwner core3d_hasVisibleRawR2Bevel])
        return [rawOwner core3d_visibleRawR2BevelStatus].canApply;
    return _viewer != nullptr
        && _viewer->getShapeInteractor() != nullptr
        && _viewer->getShapeInteractor()->canApplyBevel();
}

- (BOOL)hasActiveBevel {
    Core3DViewController *rawOwner = Core3DVisibleRawR2BevelOwner(self);
    if ([rawOwner core3d_hasVisibleRawR2Bevel]) return YES;
    return _viewer != nullptr
        && _viewer->getShapeInteractor() != nullptr
        && _viewer->getShapeInteractor()->hasActiveBevel();
}

- (double)getExtrusionMetersPerUnit {
    if (![NSThread isMainThread] || _viewer == nullptr
        || [self getGizmoType] != PrimitiveGizmoTypeExtrude
        || _viewer->getShapeInteractor() == nullptr) {
        return 0.0;
    }
    return _viewer->getShapeInteractor()->extrusionMetersPerUnit();
}

- (BOOL)setExtrusion:(CGFloat)value {
    if ([self getGizmoType] != PrimitiveGizmoTypeExtrude
        || _viewer == nullptr
        || _viewer->getShapeInteractor() == nullptr) {
        return NO;
    }
    const BOOL result = _viewer->getShapeInteractor()
        ->setExtrusionValueForSelection(
            static_cast<Standard_Real>(value));
    [self requestRender];
    return result;
}

- (BOOL)applyExtrusion {
    if ([self getGizmoType] != PrimitiveGizmoTypeExtrude
        || _viewer == nullptr
        || _viewer->getShapeInteractor() == nullptr) {
        return NO;
    }
    const BOOL applied =
        _viewer->getShapeInteractor()->applyExtrusion();
    if (applied) {
        _viewer->redrawDocument();
    }
    [self requestRender];
    return applied;
}

- (BOOL)cancelExtrusion {
    const BOOL cancelled = _viewer == nullptr
        || _viewer->getShapeInteractor() == nullptr
        || _viewer->getShapeInteractor()->cancelExtrusion();
    [self requestRender];
    return cancelled;
}

- (NSArray<NSDictionary<NSString *, id> *> *)extrusionSplitEdges {
    if (_viewer == nullptr || _viewer->getShapeInteractor() == nullptr) {
        return @[];
    }
    NSMutableArray *result = [NSMutableArray array];
    for (const auto &edge : _viewer->getShapeInteractor()->extrusionSplitEdges()) {
        [result addObject:@{
            @"side": [NSString stringWithUTF8String:edge.first.c_str()],
            @"length": @(edge.second)
        }];
    }
    return result;
}

- (BOOL)setExtrusionSplitReferenceEdge:(NSInteger)index {
    if ([self getGizmoType] != PrimitiveGizmoTypeExtrude
        || _viewer == nullptr
        || _viewer->getShapeInteractor() == nullptr) {
        return NO;
    }
    const BOOL result = _viewer->getShapeInteractor()
        ->setExtrusionSplitReferenceEdge(static_cast<Standard_Integer>(index));
    [self requestRender];
    return result;
}

- (NSInteger)extrusionSplitReferenceEdge {
    if (_viewer == nullptr || _viewer->getShapeInteractor() == nullptr) {
        return -1;
    }
    return _viewer->getShapeInteractor()->extrusionSplitReferenceEdge();
}

- (BOOL)canApplyExtrusion {
    return _viewer != nullptr
        && _viewer->getShapeInteractor() != nullptr
        && (_viewer->getShapeInteractor()->canApplyExtrusion()
            || _viewer->getShapeInteractor()
                ->canRetryExtrusionResolution());
}

- (Core3DShellParameters)getShellParameters {
    Core3DShellParameters parameters = {};
    if (_viewer == nullptr || _viewer->getShapeInteractor() == nullptr) {
        return parameters;
    }
    const std::shared_ptr<ShapeInteractor> interactor =
        _viewer->getShapeInteractor();
    const std::pair<Standard_Real, Standard_Real> range =
        interactor->shellThicknessRange();
    parameters.thickness = interactor->shellThickness();
    parameters.defaultThickness = interactor->shellDefaultThickness();
    parameters.minimumThickness = range.first;
    parameters.maximumThickness = range.second;
    parameters.metersPerUnit = interactor->shellMetersPerUnit();
    return parameters;
}

- (BOOL)setShellThickness:(CGFloat)thickness {
    if ([self getGizmoType] != PrimitiveGizmoTypeShell
        || _viewer == nullptr
        || _viewer->getShapeInteractor() == nullptr) {
        return NO;
    }
    const BOOL didSet = _viewer->getShapeInteractor()->setShellThickness(
        static_cast<Standard_Real>(thickness));
    [self requestRender];
    return didSet;
}

- (BOOL)applyShell {
    if ([self getGizmoType] != PrimitiveGizmoTypeShell
        || _viewer == nullptr
        || _viewer->getShapeInteractor() == nullptr) {
        return NO;
    }
    const ShellApplyResult result =
        _viewer->getShapeInteractor()->applyShell();
    if (result == ShellApplyResult::AppliedNeedsDocumentRedraw) {
        _viewer->redrawDocument();
    }
    [self checkSelections];
    [self requestRender];
    return result != ShellApplyResult::NoChange;
}

- (BOOL)cancelShell {
    const BOOL didCancel = _viewer == nullptr
        || _viewer->getShapeInteractor() == nullptr
        || _viewer->getShapeInteractor()->cancelShell();
    [self checkSelections];
    [self requestRender];
    return didCancel;
}

- (BOOL)canApplyShell {
    return _viewer != nullptr
        && _viewer->getShapeInteractor() != nullptr
        && _viewer->getShapeInteractor()->canApplyShell();
}

- (BOOL)hasActiveShell {
    return _viewer != nullptr
        && _viewer->getShapeInteractor() != nullptr
        && _viewer->getShapeInteractor()->hasActiveShell();
}

- (BOOL)tryMirrorWorldPlaneWithNormalAxis:(Core3DMirrorAxis)axis offset:(double)offset {
    if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr
        || [self getGizmoType] != PrimitiveGizmoTypeMirror
        || axis < Core3DMirrorAxisX || axis > Core3DMirrorAxisZ) {
        return NO;
    }
    const BOOL ready = _viewer->getObjectInteractor()->tryMirrorWorldPlane(
        static_cast<Standard_Integer>(axis), static_cast<Standard_Real>(offset));
    [self requestRender];
    return ready;
}

- (BOOL) applyMirror {
	if (_viewer == nullptr
		|| _viewer->getObjectInteractor() == nullptr
		|| [self getGizmoType] != PrimitiveGizmoTypeMirror) {
		return NO;
	}
	const MirrorApplyResult aResult =
		_viewer->getObjectInteractor()->applyMirror();
	if (aResult == MirrorApplyResult::AppliedNeedsDocumentRedraw) {
		_viewer->redrawDocument();
	}
	[self requestRender];
	return aResult != MirrorApplyResult::NoChange;
}

- (BOOL) cancelMirror {
	if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr) {
		return NO;
	}
	const BOOL didCancel =
		_viewer->getObjectInteractor()->cancelMirror();
	[self requestRender];
	return didCancel;
}

- (BOOL)beginMirrorPlanePicking {
	if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr
		|| _viewer->getShapeInteractor() == nullptr
		|| [self getGizmoType] != PrimitiveGizmoTypeMirror) {
		return NO;
	}
	const std::shared_ptr<ObjectInteractor> objectInteractor =
		_viewer->getObjectInteractor();
	if (objectInteractor->isPickingMirrorPlane()) {
		return objectInteractor->mirrorPlanePickingAuthorityMatches();
	}
	const std::shared_ptr<ShapeInteractor> shapeInteractor =
		_viewer->getShapeInteractor();
	if (shapeInteractor->getSelectionMode()
			!= ShapeSelectionMode::WholeShape
		|| !shapeInteractor->selectionModeAuthorityIsExact()) {
		// The pick ledger may only suspend a proven canonical Object mode.
		// Capturing pre-existing presentation drift would make the temporary
		// Face detector look authoritative and later restore that same drift.
		return NO;
	}
	const BOOL didBegin =
		objectInteractor->beginMirrorPlanePicking();
	[self checkSelections];
	[self requestRender];
	return didBegin;
}

- (BOOL)cancelMirrorPlanePicking {
	if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr) {
		return NO;
	}
	const BOOL didCancel =
		_viewer->getObjectInteractor()->cancelMirrorPlanePicking();
	[self checkSelections];
	[self requestRender];
	return didCancel;
}

- (BOOL)isPickingMirrorPlane {
	return _viewer != nullptr
		&& _viewer->getObjectInteractor() != nullptr
		&& _viewer->getObjectInteractor()->isPickingMirrorPlane();
}

- (BOOL)hasCustomMirrorPlane {
	return _viewer != nullptr
		&& _viewer->getObjectInteractor() != nullptr
		&& _viewer->getObjectInteractor()->hasCustomMirrorPlane();
}

- (BOOL)setMirrorPlaneOffset:(CGFloat)offset {
	if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr
		|| [self getGizmoType] != PrimitiveGizmoTypeMirror) {
		return NO;
	}
	const BOOL didSet = _viewer->getObjectInteractor()
		->setMirrorPlaneOffset(static_cast<Standard_Real>(offset));
	[self checkSelections];
	[self requestRender];
	return didSet;
}

- (Boundaries)getMirrorPlaneOffsetBoundaries {
	if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr) {
		return {.min = 0.0, .max = 0.0};
	}
	const auto aRange =
		_viewer->getObjectInteractor()->mirrorPlaneOffsetRange();
	return {.min = aRange.first, .max = aRange.second};
}

- (BOOL)resetMirrorPlane {
	if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr
		|| [self getGizmoType] != PrimitiveGizmoTypeMirror) {
		return NO;
	}
	const BOOL didReset =
		_viewer->getObjectInteractor()->resetMirrorPlane();
	[self checkSelections];
	[self requestRender];
	return didReset;
}

- (Core3DLinearArrayParameters)getLinearArrayParameters {
    Core3DLinearArrayParameters parameters = {
        .axis = Core3DLinearArrayAxisX,
        .count = 0,
        .minimumCount = 0,
        .maximumCount = 0,
        .spacing = 0.0,
		.minimumSpacing = 0.0,
		.maximumSpacing = 0.0,
		.metersPerUnit = 0.0,
    };
    if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr) {
        return parameters;
    }
    const std::shared_ptr<ObjectInteractor> interactor =
        _viewer->getObjectInteractor();
    switch (interactor->linearArrayAxis()) {
        case LinearArrayAxis::X:
            parameters.axis = Core3DLinearArrayAxisX;
            break;
        case LinearArrayAxis::Y:
            parameters.axis = Core3DLinearArrayAxisY;
            break;
        case LinearArrayAxis::Z:
            parameters.axis = Core3DLinearArrayAxisZ;
            break;
    }
    const auto countRange = interactor->linearArrayCountRange();
    const auto spacingRange = interactor->linearArraySpacingRange();
    parameters.count = interactor->linearArrayCount();
    parameters.minimumCount = countRange.first;
    parameters.maximumCount = countRange.second;
    parameters.spacing = interactor->linearArraySpacing();
    parameters.minimumSpacing = spacingRange.first;
	parameters.maximumSpacing = spacingRange.second;
	parameters.metersPerUnit = interactor->linearArrayMetersPerUnit();
	return parameters;
}

- (BOOL)setLinearArrayAxis:(Core3DLinearArrayAxis)axis {
    if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr
        || [self getGizmoType] != PrimitiveGizmoTypeLinearArray) {
        return NO;
    }
    LinearArrayAxis nativeAxis = LinearArrayAxis::X;
    switch (axis) {
        case Core3DLinearArrayAxisX:
            nativeAxis = LinearArrayAxis::X;
            break;
        case Core3DLinearArrayAxisY:
            nativeAxis = LinearArrayAxis::Y;
            break;
        case Core3DLinearArrayAxisZ:
            nativeAxis = LinearArrayAxis::Z;
            break;
        default:
            return NO;
    }
    const BOOL didSet = _viewer->getObjectInteractor()
        ->setLinearArrayAxis(nativeAxis);
    [self requestRender];
    return didSet;
}

- (BOOL)setLinearArrayCount:(NSInteger)count {
    if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr
        || [self getGizmoType] != PrimitiveGizmoTypeLinearArray
        || count < 0
        || count > std::numeric_limits<Standard_Integer>::max()) {
        return NO;
    }
    const BOOL didSet = _viewer->getObjectInteractor()
        ->setLinearArrayCount(static_cast<Standard_Integer>(count));
    [self requestRender];
    return didSet;
}

- (BOOL)setLinearArraySpacing:(CGFloat)spacing {
    if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr
        || [self getGizmoType] != PrimitiveGizmoTypeLinearArray) {
        return NO;
    }
    const BOOL didSet = _viewer->getObjectInteractor()
        ->setLinearArraySpacing(static_cast<Standard_Real>(spacing));
    [self requestRender];
    return didSet;
}

- (BOOL)applyLinearArray {
    if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr
        || [self getGizmoType] != PrimitiveGizmoTypeLinearArray) {
        return NO;
    }
    const LinearArrayApplyResult result =
        _viewer->getObjectInteractor()->applyLinearArray();
	if (result == LinearArrayApplyResult::AppliedNeedsDocumentRedraw) {
		_viewer->redrawDocument();
	}
	// A committed redraw recreates the native interactor as None before the
	// Objective-C operation wrapper has reconciled its retained Array tool. Do
	// not emit a selection callback across that deliberate one-stack-frame gap;
	// every non-commit result must still exercise the normal invariant.
	if (result != LinearArrayApplyResult::AppliedNeedsDocumentRedraw) {
		[self checkSelections];
	}
	[self requestRender];
    return result != LinearArrayApplyResult::NoChange;
}

- (BOOL)cancelLinearArray {
    if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr) {
        return NO;
    }
    const BOOL didCancel =
        _viewer->getObjectInteractor()->cancelLinearArray();
    [self checkSelections];
    [self requestRender];
    return didCancel;
}

- (BOOL)canApplyLinearArray {
    return _viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && _viewer->getObjectInteractor()->canApplyLinearArray();
}

- (BOOL)hasActiveLinearArray {
    return _viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && _viewer->getObjectInteractor()->hasActiveLinearArray();
}

- (Core3DRadialArrayParameters)getRadialArrayParameters {
    Core3DRadialArrayParameters parameters = {};
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->getObjectInteractor() == nullptr) {
        return parameters;
    }
    const std::shared_ptr<ObjectInteractor> interactor =
        _viewer->getObjectInteractor();
    const auto countRange = interactor->radialArrayCountRange();
    const auto sweepRange = interactor->radialArraySweepDegreesRange();
    parameters.count = interactor->radialArrayCount();
    parameters.minimumCount = countRange.first;
    parameters.maximumCount = countRange.second;
    parameters.sweepDegrees = interactor->radialArraySweepDegrees();
    parameters.minimumSweepDegrees = sweepRange.first;
    parameters.maximumSweepDegrees = sweepRange.second;
    parameters.metersPerUnit = interactor->radialArrayMetersPerUnit();
    return parameters;
}

- (Core3DRadialArrayReferenceAuthority)getRadialArrayReferenceAuthority {
    Core3DRadialArrayReferenceAuthority authority = {
        .readState = Core3DReferenceAxisReadStateInvalid,
        .value = {
            .pivotSpace = Core3DReferenceSpaceObject,
            .pivotX = 0.0,
            .pivotY = 0.0,
            .pivotZ = 0.0,
            .directionSpace = Core3DReferenceSpaceWorld,
            .directionX = 0.0,
            .directionY = 0.0,
            .directionZ = 1.0,
        },
        .authorityToken = 0,
    };
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->getObjectInteractor() == nullptr) {
        return authority;
    }

    const std::shared_ptr<ObjectInteractor> interactor =
        _viewer->getObjectInteractor();
    OcctReferenceAxis axis;
    const OcctReferenceAxisReadState state =
        interactor->radialArrayReferenceAxis(axis);
    Core3DReferenceSpace pivotSpace = Core3DReferenceSpaceObject;
    Core3DReferenceSpace directionSpace = Core3DReferenceSpaceWorld;
    if (state == OcctReferenceAxisReadState::Invalid
        || !TryPublicReferenceSpace(axis.pivotSpace, pivotSpace)
        || !TryPublicReferenceSpace(axis.directionSpace, directionSpace)) {
        return authority;
    }

    authority.readState = PublicReferenceAxisReadState(state);
    authority.value.pivotSpace = pivotSpace;
    authority.value.pivotX = axis.pivot.X();
    authority.value.pivotY = axis.pivot.Y();
    authority.value.pivotZ = axis.pivot.Z();
    authority.value.directionSpace = directionSpace;
    authority.value.directionX = axis.direction.X();
    authority.value.directionY = axis.direction.Y();
    authority.value.directionZ = axis.direction.Z();
    authority.authorityToken =
        interactor->radialArrayReferenceAuthorityToken();
    return authority;
}

- (Core3DRadialArrayReferenceAuthority)
    getRadialArrayReferenceAuthorityWithPivotSpace:
        (Core3DReferenceSpace)pivotSpace
    directionSpace:(Core3DReferenceSpace)directionSpace
    expectedAuthorityToken:(uint64_t)expectedAuthorityToken {
    Core3DRadialArrayReferenceAuthority invalidAuthority = {
        .readState = Core3DReferenceAxisReadStateInvalid,
        .value = {
            .pivotSpace = Core3DReferenceSpaceObject,
            .pivotX = 0.0,
            .pivotY = 0.0,
            .pivotZ = 0.0,
            .directionSpace = Core3DReferenceSpaceWorld,
            .directionX = 0.0,
            .directionY = 0.0,
            .directionZ = 1.0,
        },
        .authorityToken = 0,
    };
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->getObjectInteractor() == nullptr
        || [self getGizmoType] != PrimitiveGizmoTypeRadialArray
        || expectedAuthorityToken == 0) {
        return invalidAuthority;
    }

    OcctReferenceSpace nativePivotSpace = OcctReferenceSpace::Object;
    OcctReferenceSpace nativeDirectionSpace = OcctReferenceSpace::World;
    if (!TryNativeReferenceSpace(pivotSpace, nativePivotSpace)
        || !TryNativeReferenceSpace(
            directionSpace, nativeDirectionSpace)) {
        return invalidAuthority;
    }

    const std::shared_ptr<ObjectInteractor> interactor =
        _viewer->getObjectInteractor();
    OcctReferenceAxis sourceAxis;
    const OcctReferenceAxisReadState sourceState =
        interactor->radialArrayReferenceAxis(sourceAxis);
    if (sourceState == OcctReferenceAxisReadState::Invalid
        || interactor->radialArrayReferenceAuthorityToken()
            != expectedAuthorityToken) {
        return invalidAuthority;
    }

    OcctReferenceAxis convertedAxis;
    if (!interactor->convertRadialArrayReferenceAxisSpaces(
            nativePivotSpace,
            nativeDirectionSpace,
            expectedAuthorityToken,
            convertedAxis)) {
        return invalidAuthority;
    }

    invalidAuthority.readState = PublicReferenceAxisReadState(sourceState);
    invalidAuthority.value.pivotSpace = pivotSpace;
    invalidAuthority.value.pivotX = convertedAxis.pivot.X();
    invalidAuthority.value.pivotY = convertedAxis.pivot.Y();
    invalidAuthority.value.pivotZ = convertedAxis.pivot.Z();
    invalidAuthority.value.directionSpace = directionSpace;
    invalidAuthority.value.directionX = convertedAxis.direction.X();
    invalidAuthority.value.directionY = convertedAxis.direction.Y();
    invalidAuthority.value.directionZ = convertedAxis.direction.Z();
    invalidAuthority.authorityToken = expectedAuthorityToken;
    return invalidAuthority;
}

- (BOOL)setRadialArrayCount:(NSInteger)count {
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->getObjectInteractor() == nullptr
        || [self getGizmoType] != PrimitiveGizmoTypeRadialArray
        || count < 0
        || count > std::numeric_limits<Standard_Integer>::max()) {
        return NO;
    }
    const BOOL didSet = _viewer->getObjectInteractor()
        ->setRadialArrayCount(static_cast<Standard_Integer>(count));
    [self requestRender];
    return didSet;
}

- (BOOL)setRadialArraySweepDegrees:(double)sweepDegrees {
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->getObjectInteractor() == nullptr
        || [self getGizmoType] != PrimitiveGizmoTypeRadialArray
        || !std::isfinite(sweepDegrees)) {
        return NO;
    }
    const BOOL didSet = _viewer->getObjectInteractor()
        ->setRadialArraySweepDegrees(
            static_cast<Standard_Real>(sweepDegrees));
    [self requestRender];
    return didSet;
}

- (Core3DRadialArrayReferenceEditResult)setRadialArrayReferenceAxis:
    (Core3DReferenceAxisValue)value
    expectedAuthorityToken:(uint64_t)expectedAuthorityToken {
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->getObjectInteractor() == nullptr
        || [self getGizmoType] != PrimitiveGizmoTypeRadialArray
        || expectedAuthorityToken == 0) {
        return Core3DRadialArrayReferenceEditResultRetryableFailure;
    }

    OcctReferenceAxis axis;
    if (!TryNativeReferenceSpace(value.pivotSpace, axis.pivotSpace)
        || !TryNativeReferenceSpace(
            value.directionSpace, axis.directionSpace)) {
        return Core3DRadialArrayReferenceEditResultRetryableFailure;
    }
    try {
        axis.pivot = gp_Pnt(value.pivotX, value.pivotY, value.pivotZ);
        axis.direction = gp_Dir(
            value.directionX, value.directionY, value.directionZ);
    } catch (...) {
        return Core3DRadialArrayReferenceEditResultRetryableFailure;
    }

    const RadialArrayReferenceEditResult nativeResult =
        _viewer->getObjectInteractor()->setRadialArrayReferenceAxis(
            axis, expectedAuthorityToken);
    const Core3DRadialArrayReferenceEditResult result =
        PublicRadialReferenceEditResult(nativeResult);
    if (result != Core3DRadialArrayReferenceEditResultNoChange) {
        [self checkSelections];
        [self requestRender];
    }
    return result;
}

- (Core3DRadialArrayReferenceEditResult)
    resetRadialArrayReferenceAxisWithExpectedAuthorityToken:
        (uint64_t)expectedAuthorityToken {
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->getObjectInteractor() == nullptr
        || [self getGizmoType] != PrimitiveGizmoTypeRadialArray
        || expectedAuthorityToken == 0) {
        return Core3DRadialArrayReferenceEditResultRetryableFailure;
    }
    const RadialArrayReferenceEditResult nativeResult =
        _viewer->getObjectInteractor()->resetRadialArrayReferenceAxis(
            expectedAuthorityToken);
    const Core3DRadialArrayReferenceEditResult result =
        PublicRadialReferenceEditResult(nativeResult);
    if (result != Core3DRadialArrayReferenceEditResultNoChange) {
        [self checkSelections];
        [self requestRender];
    }
    return result;
}

- (BOOL)applyRadialArray {
    if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr
        || [self getGizmoType] != PrimitiveGizmoTypeRadialArray) {
        return NO;
    }
    const RadialArrayApplyResult result =
        _viewer->getObjectInteractor()->applyRadialArray();
    if (result == RadialArrayApplyResult::AppliedNeedsDocumentRedraw) {
        _viewer->redrawDocument();
    }
    // A committed redraw recreates the interactor as None before the public
    // operation wrapper reconciles its retained tool. Avoid that transient
    // mismatch exactly as Linear Array does.
    if (result != RadialArrayApplyResult::AppliedNeedsDocumentRedraw) {
        [self checkSelections];
    }
    [self requestRender];
    return result != RadialArrayApplyResult::NoChange;
}

- (BOOL)cancelRadialArray {
    if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr) {
        return NO;
    }
    const BOOL didCancel =
        _viewer->getObjectInteractor()->cancelRadialArray();
    [self checkSelections];
    [self requestRender];
    return didCancel;
}

- (BOOL)canApplyRadialArray {
    return _viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && _viewer->getObjectInteractor()->canApplyRadialArray();
}

- (BOOL)hasActiveRadialArray {
    return _viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && _viewer->getObjectInteractor()->hasActiveRadialArray();
}

- (BOOL) applySubtract {
	const BooleanApplyResult result =
		_viewer->getObjectInteractor()->applyBoolean(BooleanAction::BooleanSubtract);
	if (result == BooleanApplyResult::AppliedNeedsDocumentRedraw) {
		_viewer->redrawDocument();
	}
    [self requestRender];
    return result != BooleanApplyResult::NoChange;
}

- (BOOL) cancelSubtract {
    const std::shared_ptr<ObjectInteractor> anInteractor =
        _viewer->getObjectInteractor();
    anInteractor->cancelBoolean(BooleanAction::BooleanSubtract);
    [self requestRender];
    return !anInteractor->hasUnresolvedBoolean()
        && !anInteractor->hasActiveBoolean(
            BooleanAction::BooleanSubtract);
}

- (BOOL) applyUnion {
	const BooleanApplyResult result =
		_viewer->getObjectInteractor()->applyBoolean(BooleanAction::BooleanUnion);
	if (result == BooleanApplyResult::AppliedNeedsDocumentRedraw) {
		_viewer->redrawDocument();
    }
    [self requestRender];
    return result != BooleanApplyResult::NoChange;
}

- (BOOL) cancelUnion {
    const std::shared_ptr<ObjectInteractor> anInteractor =
        _viewer->getObjectInteractor();
    anInteractor->cancelBoolean(BooleanAction::BooleanUnion);
    [self requestRender];
    return !anInteractor->hasUnresolvedBoolean()
        && !anInteractor->hasActiveBoolean(BooleanAction::BooleanUnion);
}

- (BOOL) applyIntersect {
    const BooleanApplyResult result =
        _viewer->getObjectInteractor()->applyBoolean(
            BooleanAction::BooleanIntersect);
    if (result == BooleanApplyResult::AppliedNeedsDocumentRedraw) {
        _viewer->redrawDocument();
    }
    [self requestRender];
    return result != BooleanApplyResult::NoChange;
}

- (BOOL) cancelIntersect {
    const std::shared_ptr<ObjectInteractor> anInteractor =
        _viewer->getObjectInteractor();
    anInteractor->cancelBoolean(BooleanAction::BooleanIntersect);
    [self requestRender];
    return !anInteractor->hasUnresolvedBoolean()
        && !anInteractor->hasActiveBoolean(
            BooleanAction::BooleanIntersect);
}

- (BOOL) canApplyBoolean {
    return _viewer->getObjectInteractor()->canApplyBoolean();
}

- (BOOL)hasActiveBoolean {
    if (_viewer == nullptr
        || _viewer->getObjectInteractor() == nullptr) {
        return NO;
    }
    const PrimitiveGizmoType currentType = [self getGizmoType];
    BooleanAction action = BooleanAction::BooleanSubtract;
    if (TryBooleanActionForGizmo(currentType, action)) {
        return _viewer->getObjectInteractor()->hasActiveBoolean(action);
    }
    return NO;
}

- (BOOL)restoreBooleanActionForRetainedGizmoType:(PrimitiveGizmoType)type {
    BooleanAction action = BooleanAction::BooleanSubtract;
    if (!TryBooleanActionForGizmo(type, action)) return YES;
    if (_viewer == nullptr) return NO;
    const NativeBooleanRetention result = _viewer->restoreBooleanAction(action);
    if (result == NativeBooleanRetention::Retained) return YES;
    if (result == NativeBooleanRetention::HostReconciliationRequired) {
        if (_delegate != nil && [_delegate respondsToSelector:
            @selector(viewerDidFailToRetainBooleanMode:)]) {
            [_delegate viewerDidFailToRetainBooleanMode:self];
        } else {
            _viewer->getObjectInteractor()->setManipulatorType(
                PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
        }
    }
    return NO;
}

- (BOOL)retireBooleanActionForGizmoType:(PrimitiveGizmoType)type {
    BooleanAction action = BooleanAction::BooleanSubtract;
    if (!TryBooleanActionForGizmo(type, action)) return YES;
    return _viewer != nullptr && _viewer->retireBooleanAction(action);
}

- (BOOL) hasTrialMirrorObjects {
	return _viewer != nullptr
		&& _viewer->getObjectInteractor() != nullptr
		&& _viewer->getObjectInteractor()->hasTrialMirrorObjects();
}

- (BOOL)isEmptyOfDisplayedObjects {
    return _viewer->getShapeInteractor()->isEmptyOfDisplayedObjects();
}

- (NSInteger)numberOfDisplayedShapes {
    return _viewer->getShapeInteractor()->getNumberOfDisplayedShapes();
}

#ifdef DEBUG
- (void)debugSetDuplicateCommitMode:(NSInteger)mode {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()->debugSetDuplicateCommitMode(
            static_cast<Standard_Integer>(mode));
    }
}

- (void)debugSetMaximumDisplayTraversalNodes:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->SetDebugMaximumDisplayTraversalNodes(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetMaximumLeafPresentations:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->SetDebugMaximumLeafPresentations(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetMaximumProjectTopologyValidationNodes:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->SetDebugMaximumProjectTopologyValidationNodes(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugResetProjectTopologyValidationCounters {
    if (_viewer != nullptr) {
        _viewer->DebugResetProjectTopologyValidationCounters();
    }
}

- (NSUInteger)debugBoundedProjectTopologyValidationCount {
    return _viewer == nullptr
        ? 0
        : static_cast<NSUInteger>(
            _viewer->DebugBoundedProjectTopologyValidationCount());
}

- (NSUInteger)debugGeometricBRepValidationCount {
    return _viewer == nullptr
        ? 0
        : static_cast<NSUInteger>(
            _viewer->DebugGeometricBRepValidationCount());
}

- (NSInteger)debugSelectedShapeCount {
    return _viewer == nullptr ? 0 : _viewer->selectedCount();
}

- (NSDictionary<NSString *, id> *)debugTopologySelectionState {
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->getShapeInteractor() == nullptr) {
        return @{
            @"ready": @NO,
            @"acceptedMode": @(PrimitiveSelectionTypeNone),
            @"rawSelectedOwnerCount": @0,
            @"selectedCount": @0,
            @"invalidSelectedOwnerCount": @0,
            @"selectedKind": @0,
            @"topologyIndex": @(-1),
            @"presentationRepresentation": @(-1),
            @"entityIdentifier": @"",
            @"hasDetected": @NO,
            @"singleSelectionExact": @NO,
            @"selectionMatchesMode": @NO,
            @"manipulatorAttached": @NO,
            @"rawR2Record30Projection": @[],
            @"rawR2CameraProjection": @"unavailable",
        };
    }

    const TopologySelectionDebugState state =
        _viewer->getShapeInteractor()->debugTopologySelectionState();
    PrimitiveSelectionType acceptedMode = PrimitiveSelectionTypeNone;
    switch (state.acceptedMode) {
        case ShapeSelectionMode::WholeShape:
            acceptedMode = PrimitiveSelectionTypeShape;
            break;
        case ShapeSelectionMode::Face:
            acceptedMode = PrimitiveSelectionTypeFace;
            break;
        case ShapeSelectionMode::Edge:
            acceptedMode = PrimitiveSelectionTypeEdge;
            break;
        case ShapeSelectionMode::Vertex:
            acceptedMode = PrimitiveSelectionTypeVertex;
            break;
        case ShapeSelectionMode::Wire:
            break;
    }
    NSString *entityIdentifier =
        [NSString stringForStdString:state.entityIdentifier];
    if (entityIdentifier == nil) {
        entityIdentifier = @"";
    }
    NSMutableDictionary<NSString *, id> *answer = [@{
        @"ready": @(state.ready != Standard_False),
        @"acceptedMode": @(acceptedMode),
        @"rawSelectedOwnerCount": @(state.rawSelectedOwnerCount),
        @"selectedCount": @(state.selectedCount),
        @"invalidSelectedOwnerCount": @(
            state.invalidSelectedOwnerCount),
        @"selectedKind": @(
            static_cast<NSUInteger>(state.selectedKind)),
        @"topologyIndex": @(state.topologyIndex),
        @"presentationRepresentation": @(
            static_cast<NSInteger>(state.representation)),
        @"entityIdentifier": entityIdentifier,
        @"hasDetected": @(state.hasDetected != Standard_False),
        @"singleSelectionExact": @(
            state.singleSelectionExact != Standard_False),
        @"selectionMatchesMode": @(
            state.selectionMatchesMode != Standard_False),
        @"manipulatorAttached": @(
            _viewer->getObjectInteractor() != nullptr
                && _viewer->getObjectInteractor()->isManipulatorAttached()),
        @"rawR2Record30Projection": @[],
        @"rawR2CameraProjection": @"unavailable",
    } mutableCopy];

    // R4/T-B/P3 DEBUG-only locator. It observes the single selected object and
    // projects the three interior samples of edge record 30 through the exact
    // active view used by production picking. It neither changes selection nor
    // creates/consumes raw-R2 authority.
    try {
        OCC_CATCH_SIGNALS
        const Handle(V3d_View) view = _viewer->ActiveView();
        const Handle(AIS_InteractiveContext) context = _viewer->AisContext();
        if (view.IsNull() || context.IsNull()) return [answer copy];
        answer[@"rawR2CameraProjection"] =
            view->Camera()->ProjectionType()
                == Graphic3d_Camera::Projection_Orthographic
            ? @"orthographic" : @"perspective";
        context->InitSelected();
        if (!context->MoreSelected()) return [answer copy];
        const Handle(AIS_Shape) presentation =
            Handle(AIS_Shape)::DownCast(context->SelectedInteractive());
        context->NextSelected();
        if (context->MoreSelected() || presentation.IsNull()
            || presentation->Shape().IsNull()) return [answer copy];

        TopTools_IndexedMapOfShape edges;
        TopExp::MapShapes(presentation->Shape(), TopAbs_EDGE, edges);
        constexpr Standard_Integer record30OneBased = 31;
        if (edges.Extent() < record30OneBased) return [answer copy];
        const TopoDS_Edge edge = TopoDS::Edge(
            edges.FindKey(record30OneBased));
        BRepAdaptor_Curve curve(edge);
        const Standard_Real first = curve.FirstParameter();
        const Standard_Real last = curve.LastParameter();
        if (!std::isfinite(first) || !std::isfinite(last)
            || last <= first) return [answer copy];
        Standard_Integer width = 0;
        Standard_Integer height = 0;
        view->Window()->Size(width, height);
        if (width <= 0 || height <= 0) return [answer copy];

        NSMutableArray<NSDictionary<NSString *, NSNumber *> *> *points =
            [NSMutableArray arrayWithCapacity:3];
        for (const Standard_Real fraction : {0.25, 0.5, 0.75}) {
            gp_Pnt point;
            curve.D0(first + (last - first) * fraction, point);
            Standard_Integer x = 0;
            Standard_Integer y = 0;
            view->Convert(point.X(), point.Y(), point.Z(), x, y);
            [points addObject:@{
                @"fraction": @(fraction),
                @"x": @(static_cast<double>(x) / width),
                @"y": @(static_cast<double>(y) / height),
            }];
        }
        answer[@"rawR2Record30Projection"] = [points copy];
    } catch (...) {
        answer[@"rawR2Record30Projection"] = @[];
    }
    return [answer copy];
}

- (BOOL)debugDetectAnyDisplayedShape {
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->AisContext().IsNull()
        || _viewer->ActiveView().IsNull()
        || _viewer->getDocument().IsNull()) {
        return NO;
    }
    try {
        OCC_CATCH_SIGNALS
        const CGSize drawable = self.drawableSize;
        const Standard_Integer width = static_cast<Standard_Integer>(
            std::max<CGFloat>(1.0, drawable.width));
        const Standard_Integer height = static_cast<Standard_Integer>(
            std::max<CGFloat>(1.0, drawable.height));
        constexpr Standard_Integer probeCount = 16;
        for (Standard_Integer row = 0; row <= probeCount; ++row) {
            const Standard_Integer y = row * (height - 1) / probeCount;
            for (Standard_Integer column = 0; column <= probeCount;
                 ++column) {
                const Standard_Integer x =
                    column * (width - 1) / probeCount;
                (void)_viewer->AisContext()->MoveTo(
                    x, y, _viewer->ActiveView(), Standard_False);
                if (!_viewer->AisContext()->HasDetected()) {
                    continue;
                }
                const Handle(AIS_InteractiveObject) detected =
                    _viewer->AisContext()->DetectedInteractive();
                const TDF_Label label =
                    _viewer->getDocument()->ShapeLabel(detected);
                if (!label.IsNull()
                    && _viewer->getDocument()
                        ->GeometryRepresentationForLabel(label)
                        != OcctGeometryRepresentation::Invalid) {
                    return YES;
                }
            }
        }
        _viewer->AisContext()->ClearDetected(Standard_False);
        return NO;
    } catch (...) {
        try {
            OCC_CATCH_SIGNALS
            _viewer->AisContext()->ClearDetected(Standard_False);
        } catch (...) {
        }
        return NO;
    }
}

- (BOOL)debugDetectReversedFaceTopologyIndexWithEntityIdentifier:
			(NSString *)entityIdentifier
	faceTopologyIndex:(NSUInteger)faceTopologyIndex {
	return [self
		debugPublishDetectedTopologyWithEntityIdentifier:entityIdentifier
		topologyIndex:faceTopologyIndex
		isFace:YES
		foreignEntityIdentifier:nil];
}

- (BOOL)debugDetectReversedEdgeTopologyIndexWithEntityIdentifier:
			(NSString *)entityIdentifier
	edgeTopologyIndex:(NSUInteger)edgeTopologyIndex {
	return [self
		debugPublishDetectedTopologyWithEntityIdentifier:entityIdentifier
		topologyIndex:edgeTopologyIndex
		isFace:NO
		foreignEntityIdentifier:nil];
}

- (BOOL)debugDetectAlternatingForeignSelectableEdgeWithEntityIdentifier:
			(NSString *)entityIdentifier
	foreignEntityIdentifier:(NSString *)foreignEntityIdentifier
	edgeTopologyIndex:(NSUInteger)edgeTopologyIndex {
	return [self
		debugPublishDetectedTopologyWithEntityIdentifier:entityIdentifier
		topologyIndex:edgeTopologyIndex
		isFace:NO
		foreignEntityIdentifier:foreignEntityIdentifier];
}

- (BOOL)debugPublishDetectedTopologyWithEntityIdentifier:
			(NSString *)entityIdentifier
	topologyIndex:(NSUInteger)topologyIndex
	isFace:(BOOL)isFace
	foreignEntityIdentifier:(NSString *_Nullable)foreignEntityIdentifier {
	const std::shared_ptr<ShapeInteractor> shapeInteractor =
		_viewer == nullptr ? nullptr : _viewer->getShapeInteractor();
	const ShapeSelectionMode expectedMode = isFace
		? ShapeSelectionMode::Face
		: ShapeSelectionMode::Edge;
	if (![NSThread isMainThread] || _viewer == nullptr
		|| _viewer->AisContext().IsNull()
		|| _viewer->getDocument().IsNull()
		|| shapeInteractor == nullptr
		|| shapeInteractor->getSelectionMode() != expectedMode
		|| !shapeInteractor->selectionModeAuthorityIsExact()
		|| _viewer->selectedCount() != 0
		|| ![entityIdentifier isKindOfClass:NSString.class]
		|| entityIdentifier.length == 0
		|| entityIdentifier.UTF8String == nullptr
		|| (foreignEntityIdentifier != nil
			&& (isFace
				|| ![foreignEntityIdentifier isKindOfClass:NSString.class]
				|| foreignEntityIdentifier.length == 0
				|| foreignEntityIdentifier.UTF8String == nullptr
				|| [foreignEntityIdentifier isEqualToString:
					entityIdentifier]))) {
		return NO;
	}
	try {
		OCC_CATCH_SIGNALS
		Handle(AIS_Shape) aPresentation;
		if (!TryFindExactDebugPresentation(
				_viewer, entityIdentifier, aPresentation)) {
			return NO;
		}

		Handle(SelectMgr_EntityOwner) anOwner;
		if (isFace) {
			TopoDS_Face aCanonicalFace;
			if (!TryResolveCanonicalFaceTopologyIndexBounded(
					aPresentation->Shape(),
					static_cast<Standard_Size>(topologyIndex),
					ShellOperationController::kMaximumSourceTopologyNodes,
					aCanonicalFace)
				|| (aCanonicalFace.Orientation() != TopAbs_FORWARD
					&& aCanonicalFace.Orientation() != TopAbs_REVERSED)) {
				return NO;
			}
			const TopoDS_Face aReversedFace =
				TopoDS::Face(aCanonicalFace.Reversed());
			if (!aReversedFace.IsSame(aCanonicalFace)
				|| aReversedFace.IsEqual(aCanonicalFace)) {
				return NO;
			}
			anOwner = new StdSelect_BRepOwner(
				aReversedFace, aPresentation, 0, Standard_True);
		} else {
			TopoDS_Edge aCanonicalEdge;
			if (!TryResolveCanonicalBevelEdgeTopologyIndexBounded(
					aPresentation->Shape(),
					static_cast<Standard_Size>(topologyIndex),
					BevelOperationController::kMaxSourceTopologyNodes,
					aCanonicalEdge)) {
				return NO;
			}
			if (foreignEntityIdentifier == nil) {
				if (aCanonicalEdge.Orientation() != TopAbs_FORWARD
					&& aCanonicalEdge.Orientation() != TopAbs_REVERSED) {
					return NO;
				}
				const TopoDS_Edge aReversedEdge =
					TopoDS::Edge(aCanonicalEdge.Reversed());
				if (!aReversedEdge.IsSame(aCanonicalEdge)
					|| aReversedEdge.IsEqual(aCanonicalEdge)) {
					return NO;
				}
				anOwner = new StdSelect_BRepOwner(
					aReversedEdge, aPresentation, 0, Standard_True);
			} else {
				Handle(AIS_Shape) aForeignPresentation;
				if (!TryFindExactDebugPresentation(
						_viewer,
						foreignEntityIdentifier,
						aForeignPresentation)) {
					return NO;
				}
				anOwner = new DebugAlternatingSelectableBRepOwner(
					aCanonicalEdge,
					aPresentation,
					aForeignPresentation);
			}
		}
		return _viewer->DebugSetDetectedOwner(anOwner) != Standard_False;
	} catch (...) {
		try {
			OCC_CATCH_SIGNALS
			(void)_viewer->AisContext()->ClearDetected(Standard_False);
		} catch (...) {
		}
		return NO;
	}
}

- (BOOL)debugSelectAnyDisplayedTopologyElement {
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->AisContext().IsNull()
        || _viewer->ActiveView().IsNull()
        || _viewer->getDocument().IsNull()
        || _viewer->getShapeInteractor() == nullptr
        || _viewer->selectedCount() != 0) {
        return NO;
    }
    try {
        OCC_CATCH_SIGNALS
        const CGSize drawable = self.drawableSize;
        const Standard_Integer width = static_cast<Standard_Integer>(
            std::max<CGFloat>(1.0, drawable.width));
        const Standard_Integer height = static_cast<Standard_Integer>(
            std::max<CGFloat>(1.0, drawable.height));
        constexpr Standard_Integer probeCount = 16;
        for (Standard_Integer row = 0; row <= probeCount; ++row) {
            const Standard_Integer y = row * (height - 1) / probeCount;
            for (Standard_Integer column = 0; column <= probeCount;
                 ++column) {
                const Standard_Integer x =
                    column * (width - 1) / probeCount;
                (void)_viewer->AisContext()->MoveTo(
                    x, y, _viewer->ActiveView(), Standard_False);
                if (!_viewer->AisContext()->HasDetected()) {
                    continue;
                }
                const Handle(AIS_InteractiveObject) detected =
                    _viewer->AisContext()->DetectedInteractive();
                const TDF_Label label =
                    _viewer->getDocument()->ShapeLabel(detected);
                if (label.IsNull()
                    || _viewer->getDocument()
                        ->GeometryRepresentationForLabel(label)
                        == OcctGeometryRepresentation::Invalid) {
                    continue;
                }

                _viewer->Select(x, y);
                [self checkSelections];
                [self requestRender];
                const NSDictionary<NSString *, id> *state =
                    [self debugTopologySelectionState];
                if ([state[@"ready"] boolValue]
                    && [state[@"rawSelectedOwnerCount"] integerValue] == 1
                    && [state[@"selectedCount"] integerValue] == 1
                    && [state[@"invalidSelectedOwnerCount"] integerValue] == 0
                    && [state[@"singleSelectionExact"] boolValue]
                    && [state[@"selectionMatchesMode"] boolValue]) {
                    return YES;
                }
                _viewer->deselectAll();
            }
        }
        _viewer->AisContext()->ClearDetected(Standard_False);
        [self checkSelections];
        [self requestRender];
        return NO;
    } catch (...) {
        try {
            OCC_CATCH_SIGNALS
            _viewer->deselectAll();
            _viewer->AisContext()->ClearDetected(Standard_False);
        } catch (...) {
        }
        [self checkSelections];
        [self requestRender];
        return NO;
    }
}

- (BOOL)debugSelectFaceTopologyIndicesWithEntityIdentifier:
            (NSString *)entityIdentifier
    faceTopologyIndices:(NSArray<NSNumber *> *)faceTopologyIndices {
    return [self
        debugSelectFaceTopologyIndicesWithEntityIdentifier:entityIdentifier
        faceTopologyIndices:faceTopologyIndices
        reverseFirstFace:NO];
}

- (BOOL)debugSelectReversedFaceTopologyIndexWithEntityIdentifier:
            (NSString *)entityIdentifier
    faceTopologyIndex:(NSUInteger)faceTopologyIndex {
    return [self
        debugSelectFaceTopologyIndicesWithEntityIdentifier:entityIdentifier
        faceTopologyIndices:@[@(faceTopologyIndex)]
        reverseFirstFace:YES];
}

- (BOOL)debugSelectFaceTopologyIndicesWithEntityIdentifier:
            (NSString *)entityIdentifier
    faceTopologyIndices:(NSArray<NSNumber *> *)faceTopologyIndices
    reverseFirstFace:(BOOL)reverseFirstFace {
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->AisContext().IsNull()
        || _viewer->getDocument().IsNull()
        || _viewer->getShapeInteractor() == nullptr
        || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0
        || entityIdentifier.UTF8String == nullptr
        || ![faceTopologyIndices isKindOfClass:NSArray.class]
        || faceTopologyIndices.count < 1
        || faceTopologyIndices.count > 64
        || _viewer->selectedCount() != 0
        || _viewer->getShapeInteractor()->getSelectionMode()
            != ShapeSelectionMode::Face
        || !_viewer->getShapeInteractor()->selectionModeAuthorityIsExact()) {
        return NO;
    }
    try {
        OCC_CATCH_SIGNALS
        std::vector<Standard_Size> requestedIndices;
        requestedIndices.reserve(faceTopologyIndices.count);
        std::unordered_set<Standard_Size> uniqueIndices;
        uniqueIndices.reserve(faceTopologyIndices.count);
        for (id value in faceTopologyIndices) {
            if (![value isKindOfClass:NSNumber.class]) {
                return NO;
            }
            const long long signedIndex = [value longLongValue];
            if (signedIndex < 0
                || static_cast<unsigned long long>(signedIndex)
                    > static_cast<unsigned long long>(
                        std::numeric_limits<Standard_Integer>::max() - 1)) {
                return NO;
            }
            const Standard_Size index =
                static_cast<Standard_Size>(signedIndex);
            if (!uniqueIndices.insert(index).second) {
                return NO;
            }
            requestedIndices.push_back(index);
        }

        const std::string requestedIdentifier(entityIdentifier.UTF8String);
        Handle(AIS_Shape) matchedPresentation;
        AIS_ListOfInteractive displayedObjects;
        _viewer->AisContext()->DisplayedObjects(
            AIS_KOI_Shape, -1, displayedObjects);
        for (AIS_ListIteratorOfListOfInteractive presentation(
                 displayedObjects);
             presentation.More(); presentation.Next()) {
            const Handle(AIS_InteractiveObject)& object =
                presentation.Value();
            const TDF_Label label =
                _viewer->getDocument()->ShapeLabel(object);
            if (label.IsNull()
                || _viewer->getDocument()->EntityIdentifierForLabel(label)
                    != requestedIdentifier) {
                continue;
            }
            const OcctGeometryRepresentation representation =
                _viewer->getDocument()->GeometryRepresentationForLabel(label);
            const Handle(AIS_Shape) candidate =
                Handle(AIS_Shape)::DownCast(object);
            if (!matchedPresentation.IsNull() || candidate.IsNull()
                || candidate->Shape().IsNull()
                || !_viewer->getDocument()->IsPresentationEditable(candidate)
                || (representation != OcctGeometryRepresentation::BRep
                    && representation
                        != OcctGeometryRepresentation::LegacyUnknown)) {
                return NO;
            }
            matchedPresentation = candidate;
        }
        if (matchedPresentation.IsNull()) {
            return NO;
        }

        std::vector<Handle(StdSelect_BRepOwner)> owners;
        owners.reserve(requestedIndices.size());
        for (std::size_t requestOffset = 0;
            requestOffset < requestedIndices.size(); ++requestOffset) {
            const Standard_Size index = requestedIndices[requestOffset];
            TopoDS_Face canonicalFace;
            if (!TryResolveCanonicalFaceTopologyIndexBounded(
                    matchedPresentation->Shape(),
                    index,
                    ShellOperationController::
                        kMaximumSourceTopologyNodes,
                    canonicalFace)) {
                return NO;
            }
            TopoDS_Face ownerFace = canonicalFace;
            if (reverseFirstFace && requestOffset == 0) {
                if (canonicalFace.Orientation() != TopAbs_FORWARD
                    && canonicalFace.Orientation() != TopAbs_REVERSED) {
                    return NO;
                }
                ownerFace = TopoDS::Face(canonicalFace.Reversed());
                if (!ownerFace.IsSame(canonicalFace)
                    || ownerFace.IsEqual(canonicalFace)) {
                    return NO;
                }
            }
            owners.push_back(new StdSelect_BRepOwner(
                ownerFace,
                matchedPresentation,
                0,
                Standard_True));
        }

        _viewer->AisContext()->ClearDetected(Standard_False);
        _viewer->AisContext()->ClearSelected(Standard_False);
        for (const Handle(StdSelect_BRepOwner)& owner : owners) {
            _viewer->AisContext()->AddOrRemoveSelected(
                owner, Standard_False);
            if (!owner->IsSelected()) {
                throw Standard_Failure(
                    "Unable to publish deterministic Face owner");
            }
        }

        std::unordered_set<const SelectMgr_EntityOwner*> expectedOwners;
        expectedOwners.reserve(owners.size());
        for (const Handle(StdSelect_BRepOwner)& owner : owners) {
            expectedOwners.insert(owner.get());
        }
        Standard_Size rawSelectedOwnerCount = 0;
        for (_viewer->AisContext()->InitSelected();
             _viewer->AisContext()->MoreSelected();
             _viewer->AisContext()->NextSelected()) {
            ++rawSelectedOwnerCount;
            const Handle(SelectMgr_EntityOwner) selectedOwner =
                _viewer->AisContext()->SelectedOwner();
            if (selectedOwner.IsNull()
                || expectedOwners.erase(selectedOwner.get()) != 1
                || _viewer->AisContext()->SelectedInteractive()
                    != matchedPresentation) {
                throw Standard_Failure(
                    "Deterministic Face owner publication drifted");
            }
        }
        if (rawSelectedOwnerCount != owners.size()
            || !expectedOwners.empty()) {
            throw Standard_Failure(
                "Deterministic Face owner count mismatch");
        }
        [self checkSelections];
        [self requestRender];
        return YES;
    } catch (...) {
        try {
            OCC_CATCH_SIGNALS
            _viewer->AisContext()->ClearSelected(Standard_False);
            _viewer->AisContext()->ClearDetected(Standard_False);
        } catch (...) {
        }
        [self checkSelections];
        [self requestRender];
        return NO;
    }
}

- (BOOL)debugSelectEdgeTopologyIndicesWithEntityIdentifier:
			(NSString *)entityIdentifier
	edgeTopologyIndices:(NSArray<NSNumber *> *)edgeTopologyIndices {
	return [self
		debugSelectEdgeTopologyIndicesWithEntityIdentifier:entityIdentifier
		edgeTopologyIndices:edgeTopologyIndices
		reverseFirstEdge:NO
		foreignEntityIdentifier:nil];
}

- (BOOL)debugSelectReversedEdgeTopologyIndexWithEntityIdentifier:
			(NSString *)entityIdentifier
	edgeTopologyIndex:(NSUInteger)edgeTopologyIndex {
	return [self
		debugSelectEdgeTopologyIndicesWithEntityIdentifier:entityIdentifier
		edgeTopologyIndices:@[@(edgeTopologyIndex)]
		reverseFirstEdge:YES
		foreignEntityIdentifier:nil];
}

- (BOOL)debugSelectValidAndForeignEdgeOwnersWithEntityIdentifier:
			(NSString *)entityIdentifier
	foreignEntityIdentifier:(NSString *)foreignEntityIdentifier
	edgeTopologyIndex:(NSUInteger)edgeTopologyIndex {
	return [self
		debugSelectEdgeTopologyIndicesWithEntityIdentifier:entityIdentifier
		edgeTopologyIndices:@[@(edgeTopologyIndex)]
		reverseFirstEdge:NO
		foreignEntityIdentifier:foreignEntityIdentifier];
}

- (BOOL)debugSelectEdgeTopologyIndicesWithEntityIdentifier:
			(NSString *)entityIdentifier
	edgeTopologyIndices:(NSArray<NSNumber *> *)edgeTopologyIndices
	reverseFirstEdge:(BOOL)reverseFirstEdge
	foreignEntityIdentifier:(NSString *_Nullable)foreignEntityIdentifier {
	const std::shared_ptr<ShapeInteractor> shapeInteractor =
		_viewer == nullptr ? nullptr : _viewer->getShapeInteractor();
	const BOOL refreshesIdleBevel = shapeInteractor != nullptr
		&& shapeInteractor->hasActiveBevel()
		&& shapeInteractor->bevelPreviewState()
			== BevelPreviewState::Selecting
		&& !shapeInteractor->isBevelSelectionFrozen();
	if (![NSThread isMainThread] || _viewer == nullptr
		|| _viewer->AisContext().IsNull()
		|| _viewer->getDocument().IsNull()
		|| shapeInteractor == nullptr
		|| ![entityIdentifier isKindOfClass:NSString.class]
		|| entityIdentifier.length == 0
		|| entityIdentifier.UTF8String == nullptr
		|| ![edgeTopologyIndices isKindOfClass:NSArray.class]
		|| edgeTopologyIndices.count < 1
		|| edgeTopologyIndices.count
			> BevelOperationController::kMaxSelectedEdges
		|| (_viewer->selectedCount() != 0 && !refreshesIdleBevel)
		|| shapeInteractor->getSelectionMode() != ShapeSelectionMode::Edge
		|| !shapeInteractor->selectionModeAuthorityIsExact()
		|| (foreignEntityIdentifier != nil
			&& (![foreignEntityIdentifier isKindOfClass:NSString.class]
				|| foreignEntityIdentifier.length == 0
				|| foreignEntityIdentifier.UTF8String == nullptr
				|| [foreignEntityIdentifier isEqualToString:
					entityIdentifier]
				|| edgeTopologyIndices.count != 1
				|| reverseFirstEdge))) {
		return NO;
	}
	try {
		OCC_CATCH_SIGNALS
		std::vector<Standard_Size> requestedIndices;
		requestedIndices.reserve(edgeTopologyIndices.count);
		std::unordered_set<Standard_Size> uniqueIndices;
		uniqueIndices.reserve(edgeTopologyIndices.count);
		for (id value in edgeTopologyIndices) {
			if (![value isKindOfClass:NSNumber.class]
				|| CFGetTypeID((__bridge CFTypeRef)value)
					== CFBooleanGetTypeID()) {
				return NO;
			}
			const long long signedIndex = [value longLongValue];
			if (signedIndex < 0
				|| static_cast<unsigned long long>(signedIndex)
					> static_cast<unsigned long long>(
						std::numeric_limits<Standard_Integer>::max() - 1)) {
				return NO;
			}
			const Standard_Size index =
				static_cast<Standard_Size>(signedIndex);
			if (!uniqueIndices.insert(index).second) {
				return NO;
			}
			requestedIndices.push_back(index);
		}

		const auto findExactPresentation =
			[&](NSString *identifier, Handle(AIS_Shape)& result) {
				result.Nullify();
				const std::string requested(identifier.UTF8String);
				AIS_ListOfInteractive displayedObjects;
				_viewer->AisContext()->DisplayedObjects(
					AIS_KOI_Shape, -1, displayedObjects);
				for (AIS_ListIteratorOfListOfInteractive presentation(
						displayedObjects);
					 presentation.More(); presentation.Next()) {
					const Handle(AIS_InteractiveObject)& object =
						presentation.Value();
					const TDF_Label label =
						_viewer->getDocument()->ShapeLabel(object);
					if (label.IsNull()
						|| _viewer->getDocument()
							->EntityIdentifierForLabel(label) != requested) {
						continue;
					}
					const OcctGeometryRepresentation representation =
						_viewer->getDocument()
							->GeometryRepresentationForLabel(label);
					const Handle(AIS_Shape) candidate =
						Handle(AIS_Shape)::DownCast(object);
					if (!result.IsNull() || candidate.IsNull()
						|| candidate->Shape().IsNull()
						|| !_viewer->getDocument()
							->IsPresentationEditable(candidate)
						|| (representation
								!= OcctGeometryRepresentation::BRep
							&& representation
								!= OcctGeometryRepresentation::LegacyUnknown)) {
						return Standard_False;
					}
					result = candidate;
				}
				return !result.IsNull();
			};

		Handle(AIS_Shape) matchedPresentation;
		if (!findExactPresentation(entityIdentifier, matchedPresentation)) {
			return NO;
		}
		Handle(AIS_Shape) foreignPresentation;
		if (foreignEntityIdentifier != nil
			&& !findExactPresentation(
				foreignEntityIdentifier, foreignPresentation)) {
			return NO;
		}

		std::vector<Handle(StdSelect_BRepOwner)> owners;
		owners.reserve(requestedIndices.size()
			+ (foreignPresentation.IsNull() ? 0U : 1U));
		for (std::size_t offset = 0;
			 offset < requestedIndices.size(); ++offset) {
			TopoDS_Edge canonicalEdge;
			if (!TryResolveCanonicalBevelEdgeTopologyIndexBounded(
					matchedPresentation->Shape(),
					requestedIndices[offset],
					BevelOperationController::kMaxSourceTopologyNodes,
					canonicalEdge)) {
				return NO;
			}
			TopoDS_Edge ownerEdge = canonicalEdge;
			if (reverseFirstEdge && offset == 0) {
				if (canonicalEdge.Orientation() != TopAbs_FORWARD
					&& canonicalEdge.Orientation() != TopAbs_REVERSED) {
					return NO;
				}
				ownerEdge = TopoDS::Edge(canonicalEdge.Reversed());
				if (!ownerEdge.IsSame(canonicalEdge)
					|| ownerEdge.IsEqual(canonicalEdge)) {
					return NO;
				}
			}
			owners.push_back(new StdSelect_BRepOwner(
				ownerEdge, matchedPresentation, 0, Standard_True));
			if (!foreignPresentation.IsNull()) {
				owners.push_back(new StdSelect_BRepOwner(
					canonicalEdge,
					foreignPresentation,
					0,
					Standard_True));
			}
		}

		_viewer->AisContext()->ClearDetected(Standard_False);
		_viewer->AisContext()->ClearSelected(Standard_False);
		for (const Handle(StdSelect_BRepOwner)& owner : owners) {
			_viewer->AisContext()->AddOrRemoveSelected(owner, Standard_False);
			if (!owner->IsSelected()) {
				throw Standard_Failure(
					"Unable to publish deterministic Edge owner");
			}
		}

		std::unordered_set<const SelectMgr_EntityOwner*> expectedOwners;
		expectedOwners.reserve(owners.size());
		for (const Handle(StdSelect_BRepOwner)& owner : owners) {
			expectedOwners.insert(owner.get());
		}
		Standard_Size rawSelectedOwnerCount = 0;
		for (_viewer->AisContext()->InitSelected();
			 _viewer->AisContext()->MoreSelected();
			 _viewer->AisContext()->NextSelected()) {
			++rawSelectedOwnerCount;
			const Handle(SelectMgr_EntityOwner) selectedOwner =
				_viewer->AisContext()->SelectedOwner();
			if (selectedOwner.IsNull()
				|| expectedOwners.erase(selectedOwner.get()) != 1) {
				throw Standard_Failure(
					"Deterministic Edge owner publication drifted");
			}
		}
		if (rawSelectedOwnerCount != owners.size()
			|| !expectedOwners.empty()) {
			throw Standard_Failure(
				"Deterministic Edge owner count mismatch");
		}
		if (refreshesIdleBevel) {
			(void)shapeInteractor->saveSelectionEdges();
		}
		[self checkSelections];
		[self requestRender];
		return YES;
	} catch (...) {
		try {
			OCC_CATCH_SIGNALS
			_viewer->AisContext()->ClearSelected(Standard_False);
			_viewer->AisContext()->ClearDetected(Standard_False);
			if (refreshesIdleBevel) {
				(void)shapeInteractor->saveSelectionEdges();
			}
		} catch (...) {
		}
		[self checkSelections];
		[self requestRender];
		return NO;
	}
}

- (BOOL)debugSelectRetainedOperationPresentation {
    if (![NSThread isMainThread] || _viewer == nullptr) {
        return NO;
    }
    const BOOL selected =
        _viewer->DebugSelectRetainedOperationPresentation() != Standard_False;
    if (selected) {
        [self requestRender];
    }
    return selected;
}

- (BOOL)debugSetFirstDisplayedShapeSelectionMode:
    (PrimitiveSelectionType)mode {
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->AisContext().IsNull()
        || _viewer->getDocument().IsNull()) {
        return NO;
    }
    ShapeSelectionMode ignoredMode = ShapeSelectionMode::WholeShape;
    if (!TryShapeSelectionMode(mode, ignoredMode)) {
        return NO;
    }
    TopAbs_ShapeEnum topologyType = TopAbs_SHAPE;
    switch (ignoredMode) {
        case ShapeSelectionMode::WholeShape:
            topologyType = TopAbs_SHAPE;
            break;
        case ShapeSelectionMode::Face:
            topologyType = TopAbs_FACE;
            break;
        case ShapeSelectionMode::Edge:
            topologyType = TopAbs_EDGE;
            break;
        case ShapeSelectionMode::Vertex:
            topologyType = TopAbs_VERTEX;
            break;
        case ShapeSelectionMode::Wire:
            return NO;
    }
    try {
        OCC_CATCH_SIGNALS
        AIS_ListOfInteractive displayedObjects;
        _viewer->AisContext()->DisplayedObjects(
            AIS_KOI_Shape, -1, displayedObjects);
        for (AIS_ListIteratorOfListOfInteractive presentation(
                 displayedObjects);
             presentation.More(); presentation.Next()) {
            const Handle(AIS_InteractiveObject)& object =
                presentation.Value();
            const TDF_Label label =
                _viewer->getDocument()->ShapeLabel(object);
            if (label.IsNull()
                || _viewer->getDocument()
                    ->GeometryRepresentationForLabel(label)
                    == OcctGeometryRepresentation::Invalid) {
                continue;
            }
            _viewer->AisContext()->Deactivate(object);
            _viewer->AisContext()->SetSelectionModeActive(
                object,
                AIS_Shape::SelectionMode(topologyType),
                Standard_True,
                AIS_SelectionModesConcurrency_Single,
                Standard_True);
            return YES;
        }
        return NO;
    } catch (...) {
        return NO;
    }
}

- (BOOL)debugSetDisplayedShapeSelectionModeWithEntityIdentifier:
            (NSString *)entityIdentifier
    mode:(PrimitiveSelectionType)mode {
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->AisContext().IsNull()
        || _viewer->getDocument().IsNull()
        || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0
        || entityIdentifier.UTF8String == nullptr) {
        return NO;
    }
    ShapeSelectionMode ignoredMode = ShapeSelectionMode::WholeShape;
    if (!TryShapeSelectionMode(mode, ignoredMode)) {
        return NO;
    }
    TopAbs_ShapeEnum topologyType = TopAbs_SHAPE;
    switch (ignoredMode) {
        case ShapeSelectionMode::WholeShape:
            topologyType = TopAbs_SHAPE;
            break;
        case ShapeSelectionMode::Face:
            topologyType = TopAbs_FACE;
            break;
        case ShapeSelectionMode::Edge:
            topologyType = TopAbs_EDGE;
            break;
        case ShapeSelectionMode::Vertex:
            topologyType = TopAbs_VERTEX;
            break;
        case ShapeSelectionMode::Wire:
            return NO;
    }
    try {
        OCC_CATCH_SIGNALS
        const std::string requestedIdentifier(entityIdentifier.UTF8String);
        Handle(AIS_InteractiveObject) matchedPresentation;
        AIS_ListOfInteractive displayedObjects;
        _viewer->AisContext()->DisplayedObjects(
            AIS_KOI_Shape, -1, displayedObjects);
        for (AIS_ListIteratorOfListOfInteractive presentation(
                 displayedObjects);
             presentation.More(); presentation.Next()) {
            const Handle(AIS_InteractiveObject)& object =
                presentation.Value();
            const TDF_Label label =
                _viewer->getDocument()->ShapeLabel(object);
            if (label.IsNull()
                || _viewer->getDocument()
                       ->GeometryRepresentationForLabel(label)
                    == OcctGeometryRepresentation::Invalid
                || _viewer->getDocument()->EntityIdentifierForLabel(label)
                    != requestedIdentifier) {
                continue;
            }
            if (!matchedPresentation.IsNull()) {
                // One entity may temporarily own original and candidate AIS
                // objects. This fault seam never guesses which one to poison.
                return NO;
            }
            matchedPresentation = object;
        }
        if (matchedPresentation.IsNull()) {
            return NO;
        }
        _viewer->AisContext()->Deactivate(matchedPresentation);
        _viewer->AisContext()->SetSelectionModeActive(
            matchedPresentation,
            AIS_Shape::SelectionMode(topologyType),
            Standard_True,
            AIS_SelectionModesConcurrency_Single,
            Standard_True);
        return YES;
    } catch (...) {
        return NO;
    }
}

- (void)debugSetSelectionModeVerificationFailureCount:(NSUInteger)count {
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->getShapeInteractor() == nullptr) {
        return;
    }
    _viewer->getShapeInteractor()
        ->debugSetSelectionModeVerificationFailureCount(
            static_cast<Standard_Size>(count));
}

- (NSDictionary<NSString *, NSNumber *> *)debugMirrorState {
    if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr) {
        return @{};
    }
    const MirrorPreviewDebugState state =
        _viewer->getObjectInteractor()->debugMirrorPreviewState();
    return @{
        @"state": @(static_cast<NSUInteger>(state.state)),
        @"generation": @(
            static_cast<unsigned long long>(state.generation)),
        @"previewBodyCount": @(state.previewBodyCount),
        @"pendingResultCount": @(state.pendingResultCount),
        @"activeOperation": @(
            state.activeOperation != Standard_False),
        @"previewValid": @(state.previewValid != Standard_False),
        @"canApply": @(state.canApply != Standard_False),
        @"ownsDocumentCommand": @(
            state.ownsDocumentCommand != Standard_False),
        @"documentCommandOpen": @(
            state.documentCommandOpen != Standard_False),
		@"pickingCustomPlane": @(
			state.pickingCustomPlane != Standard_False),
		@"hasCustomPlane": @(state.hasCustomPlane != Standard_False),
		@"previewUsesCustomPlane": @(
			state.previewUsesCustomPlane != Standard_False),
		@"manipulatorAttached": @(
			state.manipulatorAttached != Standard_False),
		@"referencePresentationCount": @(
			state.referencePresentationCount),
		@"customPlaneOffset": @(state.customPlaneOffset),
		@"customPlaneMinimumOffset": @(
			state.customPlaneMinimumOffset),
		@"customPlaneMaximumOffset": @(
			state.customPlaneMaximumOffset),
    };
}

- (void)debugSetMirrorTransactionFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()
            ->debugSetMirrorTransactionFailureCount(
                static_cast<Standard_Size>(count));
    }
}

- (void)debugSetMirrorAbortFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()->debugSetMirrorAbortFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetMirrorEraseFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()->debugSetMirrorEraseFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetMirrorReferenceEraseFailureCount:(NSUInteger)count {
	if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
		_viewer->getObjectInteractor()
			->debugSetMirrorReferenceEraseFailureCount(
				static_cast<Standard_Size>(count));
	}
}

- (void)debugSetMirrorCommitMode:(NSInteger)mode {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()->debugSetMirrorCommitMode(
            static_cast<Standard_Integer>(mode));
    }
}

- (void)debugSetMirrorPostCommitInspectFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()
            ->debugSetMirrorPostCommitInspectFailureCount(
                static_cast<Standard_Size>(count));
    }
}

- (void)debugSetMaximumMirrorTopologyNodes:(NSUInteger)limit {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()->debugSetMaximumMirrorTopologyNodes(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetMaximumMirrorReferenceTopologyNodes:(NSUInteger)limit {
	if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
		_viewer->getObjectInteractor()
			->debugSetMaximumMirrorReferenceTopologyNodes(
				static_cast<Standard_Size>(limit));
	}
}

- (void)debugSetMaximumMirrorReferenceFaces:(NSUInteger)limit {
	if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
		_viewer->getObjectInteractor()
			->debugSetMaximumMirrorReferenceFaces(
				static_cast<Standard_Size>(limit));
	}
}

- (BOOL)debugMutateFirstMirrorSourcePersistedTransform {
    return _viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && _viewer->getObjectInteractor()
            ->debugMutateFirstMirrorSourcePersistedTransform();
}

- (BOOL)debugTryMirrorPlaneWithEntityIdentifier:(NSString *)entityIdentifier
                              faceTopologyIndex:(NSInteger)faceTopologyIndex
                                         offset:(CGFloat)offset {
	if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr
		|| entityIdentifier.length == 0 || faceTopologyIndex < 0) {
		return NO;
	}
	const BOOL didCreate = _viewer->getObjectInteractor()
		->debugTryMirrorPlane(
			entityIdentifier.UTF8String,
			static_cast<Standard_Integer>(faceTopologyIndex),
			static_cast<Standard_Real>(offset));
	[self checkSelections];
	[self requestRender];
	return didCreate;
}

- (NSDictionary<NSString *, NSNumber *> *)debugLinearArrayState {
    if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr) {
        return @{};
    }
    const LinearArrayPreviewDebugState state =
        _viewer->getObjectInteractor()->debugLinearArrayPreviewState();
    return @{
        @"state": @(static_cast<NSUInteger>(state.state)),
        @"generation": @(
            static_cast<unsigned long long>(state.generation)),
        @"previewBodyCount": @(state.previewBodyCount),
        @"pendingResultCount": @(state.pendingResultCount),
        @"activeOperation": @(
            state.activeOperation != Standard_False),
        @"previewValid": @(state.previewValid != Standard_False),
        @"canApply": @(state.canApply != Standard_False),
        @"ownsDocumentCommand": @(
            state.ownsDocumentCommand != Standard_False),
        @"documentCommandOpen": @(
            state.documentCommandOpen != Standard_False),
        @"axis": @(static_cast<NSUInteger>(state.axis)),
        @"count": @(state.count),
        @"spacing": @(state.spacing),
    };
}

- (void)debugSetLinearArrayTransactionFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()
            ->debugSetLinearArrayTransactionFailureCount(
                static_cast<Standard_Size>(count));
    }
}

- (void)debugSetLinearArrayAbortFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()->debugSetLinearArrayAbortFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetLinearArrayEraseFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()->debugSetLinearArrayEraseFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetLinearArrayCommitMode:(NSInteger)mode {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()->debugSetLinearArrayCommitMode(
            static_cast<Standard_Integer>(mode));
    }
}

- (void)debugSetLinearArrayPostCommitInspectFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()
            ->debugSetLinearArrayPostCommitInspectFailureCount(
                static_cast<Standard_Size>(count));
    }
}

- (void)debugSetLinearArrayProfileCopyFault:(NSInteger)mode {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()->debugSetLinearArrayProfileCopyFault(
            static_cast<Standard_Integer>(mode));
    }
}

- (void)debugSetMirrorProfileCopyFault:(NSInteger)mode {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()->debugSetMirrorProfileCopyFault(
            static_cast<Standard_Integer>(mode));
    }
}

- (void)debugSetMaximumLinearArrayTopologyNodes:(NSUInteger)limit {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()
            ->debugSetMaximumLinearArrayTopologyNodes(
                static_cast<Standard_Size>(limit));
    }
}

- (BOOL)debugMutateFirstLinearArraySourcePersistedTransform {
    return _viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && _viewer->getObjectInteractor()
            ->debugMutateFirstLinearArraySourcePersistedTransform();
}

- (NSDictionary<NSString *, NSNumber *> *)debugRadialArrayState {
    if (_viewer == nullptr || _viewer->getObjectInteractor() == nullptr) {
        return @{};
    }
    const RadialArrayPreviewDebugState state =
        _viewer->getObjectInteractor()->debugRadialArrayPreviewState();
    return @{
        @"state": @(static_cast<NSUInteger>(state.state)),
        @"generation": @(
            static_cast<unsigned long long>(state.generation)),
        @"referenceAuthorityToken": @(
            static_cast<unsigned long long>(state.referenceAuthorityToken)),
        @"previewBodyCount": @(state.previewBodyCount),
        @"pendingResultCount": @(state.pendingResultCount),
        @"hasPendingReferenceEdit": @(
            state.hasPendingReferenceEdit != Standard_False),
        @"activeOperation": @(
            state.activeOperation != Standard_False),
        @"previewValid": @(state.previewValid != Standard_False),
        @"canApply": @(state.canApply != Standard_False),
        @"ownsDocumentCommand": @(
            state.ownsDocumentCommand != Standard_False),
        @"documentCommandOpen": @(
            state.documentCommandOpen != Standard_False),
        @"count": @(state.count),
        @"sweepDegrees": @(state.sweepDegrees),
    };
}

- (void)debugSetRadialArrayBeginOwnedCommandMismatchCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()
            ->debugSetRadialArrayBeginOwnedCommandMismatchCount(
                static_cast<Standard_Size>(count));
    }
}

- (void)debugSetRadialArrayTransactionFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()
            ->debugSetRadialArrayTransactionFailureCount(
                static_cast<Standard_Size>(count));
    }
}

- (void)debugSetRadialArrayAbortFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()->debugSetRadialArrayAbortFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetRadialArrayEraseFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()->debugSetRadialArrayEraseFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetRadialArrayApplyCommitMode:(NSInteger)mode {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()->debugSetRadialArrayApplyCommitMode(
            static_cast<Standard_Integer>(mode));
    }
}

- (void)debugSetRadialArrayPostCommitInspectMode:(NSInteger)mode {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()
            ->debugSetRadialArrayPostCommitInspectMode(
                static_cast<Standard_Integer>(mode));
    }
}

- (void)debugSetRadialArrayProfileCopyFault:(NSInteger)mode {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()->debugSetRadialArrayProfileCopyFault(
            static_cast<Standard_Integer>(mode));
    }
}

- (void)debugSetMaximumRadialArrayTopologyNodes:(NSUInteger)limit {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()
            ->debugSetMaximumRadialArrayTopologyNodes(
                static_cast<Standard_Size>(limit));
    }
}

- (BOOL)debugMutateRadialArraySourcePersistedTransform {
    return _viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && _viewer->getObjectInteractor()
            ->debugMutateRadialArraySourcePersistedTransform();
}

- (void)debugSetRadialArrayReferenceEditCommitMode:(NSInteger)mode {
    if (_viewer != nullptr && _viewer->getObjectInteractor() != nullptr) {
        _viewer->getObjectInteractor()
            ->debugSetRadialArrayReferenceEditCommitMode(
                static_cast<Standard_Integer>(mode));
    }
}

- (NSDictionary<NSString *, NSNumber *> *)debugBooleanPreviewState {
    if (_viewer == nullptr) {
        return @{};
    }
    const BooleanPreviewDebugState state =
        _viewer->DebugBooleanPreviewState();
    return @{
        @"state": @(static_cast<NSUInteger>(state.state)),
        @"generation": @(state.generation),
        @"submittedCount": @(state.submittedCount),
        @"startedCount": @(state.startedCount),
        @"completedCount": @(state.completedCount),
        @"cancelledCount": @(state.cancelledCount),
        @"pendingReplacementCount": @(state.pendingReplacementCount),
        @"acceptedCount": @(state.acceptedCount),
        @"staleSuppressionCount": @(state.staleSuppressionCount),
        @"actorOperandCount": @(state.actorOperandCount),
        @"subjectOperandCount": @(state.subjectOperandCount),
        @"activeOperation": @(state.activeOperation != Standard_False),
        @"workerActive": @(state.workerActive != Standard_False),
        @"workerPending": @(state.workerPending != Standard_False),
        @"canApply": @(state.canApply != Standard_False),
        @"documentCommandUnresolved": @(
            state.documentCommandUnresolved != Standard_False),
        @"documentCommandOpen": @(
            state.documentCommandOpen != Standard_False),
        @"lastComputeWasMainThread": @(
            state.lastComputeWasMainThread != Standard_False),
    };
}

- (void)debugSetBooleanPreviewWorkerBlocked:(BOOL)blocked {
    if (_viewer != nullptr) {
        _viewer->DebugSetBooleanPreviewWorkerBlocked(blocked);
    }
}

- (void)debugSetMaximumBooleanCaptureTopologyNodes:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->DebugSetMaximumBooleanCaptureTopologyNodes(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetMaximumBooleanResultTopologyNodes:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->DebugSetMaximumBooleanResultTopologyNodes(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetMaximumBooleanResultSolids:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->DebugSetMaximumBooleanResultSolids(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetBooleanTransactionFailureCount:(NSUInteger)count {
    if (_viewer != nullptr) {
        _viewer->DebugSetBooleanTransactionFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetBooleanMetadataFailurePhase:(NSUInteger)phase {
    if (_viewer != nullptr) {
        _viewer->DebugSetBooleanMetadataFailurePhase(
            static_cast<Standard_Size>(phase));
    }
}

- (void)debugSetBooleanAbortFailureCount:(NSUInteger)count {
    if (_viewer != nullptr) {
        _viewer->DebugSetBooleanAbortFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetBooleanPostCommitInspectFailureCount:(NSUInteger)count {
    if (_viewer != nullptr) {
        _viewer->DebugSetBooleanPostCommitInspectFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetExtrusionCommitMode:(NSInteger)mode {
    if (_viewer != nullptr && _viewer->getShapeInteractor() != nullptr) {
        _viewer->getShapeInteractor()->debugSetExtrusionCommitMode(
            static_cast<Standard_Integer>(mode));
    }
}

- (void)debugSetExtrusionAbortFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getShapeInteractor() != nullptr) {
        _viewer->getShapeInteractor()->debugSetExtrusionAbortFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetExtrusionPostCommitInspectFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getShapeInteractor() != nullptr) {
        _viewer->getShapeInteractor()
            ->debugSetExtrusionPostCommitInspectFailureCount(
                static_cast<Standard_Size>(count));
    }
}

- (NSDictionary<NSString *, NSNumber *> *)debugExtrusionState {
    if (_viewer == nullptr || _viewer->getShapeInteractor() == nullptr) {
        return @{};
    }
    const ExtrusionDebugState state =
        _viewer->getShapeInteractor()->debugExtrusionState();
    return @{
        @"selectionReady": @(state.selectionReady != Standard_False),
        @"previewActive": @(state.previewActive != Standard_False),
        @"canApply": @(state.canApply != Standard_False),
        @"resolutionRetryable": @(
            state.resolutionRetryable != Standard_False),
        @"commandOpen": @(state.commandOpen != Standard_False),
        @"lastApplySucceeded": @(
            state.lastApplySucceeded != Standard_False),
        @"distance": @(state.distance),
        @"sourceSubshapeCount": @(state.sourceSubshapeCount),
        @"profileEdgeCount": @(state.profileEdgeCount),
        @"candidateSubshapeCount": @(state.candidateSubshapeCount),
        @"candidateSolidCount": @(state.candidateSolidCount),
        @"lastFailureStage": @(state.lastFailureStage),
        @"lastFeatureStatus": @(state.lastFeatureStatus),
        @"lastCandidateShapeType": @(state.lastCandidateShapeType),
        @"rawDirectChildCount": @(state.rawDirectChildCount),
        @"rawDirectChildShapeType": @(state.rawDirectChildShapeType),
        @"historyHasModified": @(
            state.historyHasModified != Standard_False),
        @"historyHasGenerated": @(
            state.historyHasGenerated != Standard_False),
        @"historyFaceDeleted": @(
            state.historyFaceDeleted != Standard_False),
        @"candidateVolume": @(state.candidateVolume),
        @"candidateMinX": @(state.candidateMinX),
        @"candidateMinY": @(state.candidateMinY),
        @"candidateMinZ": @(state.candidateMinZ),
        @"candidateMaxX": @(state.candidateMaxX),
        @"candidateMaxY": @(state.candidateMaxY),
        @"candidateMaxZ": @(state.candidateMaxZ),
    };
}

- (BOOL)debugBeginShellWithEntityIdentifier:(NSString *)entityIdentifier
                          faceTopologyIndex:(NSUInteger)faceTopologyIndex {
    if (![NSThread isMainThread] || _viewer == nullptr
        || _viewer->getShapeInteractor() == nullptr
        || _viewer->getObjectInteractor() == nullptr
        || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0
        || entityIdentifier.UTF8String == nullptr) {
        return NO;
    }
    try {
        const BOOL didBegin = _viewer->debugBeginShellSelection(
            std::string(entityIdentifier.UTF8String),
            static_cast<Standard_Size>(faceTopologyIndex));
        if (!didBegin) {
            [self requestRender];
            return NO;
        }
        _viewer->getObjectInteractor()->setManipulatorType(
            PrimitiveManipulatorType::PrimitiveGizmoTypeShell);
        if (_viewer->getObjectInteractor()->getManipulatorType()
            != PrimitiveManipulatorType::PrimitiveGizmoTypeShell) {
            (void)_viewer->getShapeInteractor()->cancelShell();
            [self requestRender];
            return NO;
        }
        // Core3DViewController synchronizes its public gizmo mirror after this
        // DEBUG seam returns, then emits the selection/UI-state notification.
        // Calling checkSelections here would re-enter the delegate while the
        // wrapper still reflects the previous gizmo and violate that invariant.
        [self requestRender];
        return YES;
    } catch (...) {
        (void)_viewer->getShapeInteractor()->cancelShell();
        [self requestRender];
        return NO;
    }
}

- (NSDictionary<NSString *, NSNumber *> *)debugShellState {
    if (_viewer == nullptr || _viewer->getShapeInteractor() == nullptr) {
        return @{};
    }
    const ShellPreviewDebugState state =
        _viewer->DebugShellPreviewState();
    const BOOL previewActive =
        state.state == ShellPreviewState::Ready
        && state.canApply != Standard_False;
    return @{
        @"state": @(static_cast<NSUInteger>(state.state)),
        @"generation": @(
            static_cast<unsigned long long>(state.generation)),
        @"selectionReady": @(state.activeOperation != Standard_False),
        @"activeOperation": @(state.activeOperation != Standard_False),
        @"previewActive": @(previewActive),
        @"canApply": @(state.canApply != Standard_False),
        @"selectionFrozen": @(state.selectionFrozen != Standard_False),
        @"ownsDocumentCommand": @(
            state.ownsDocumentCommand != Standard_False),
        @"commandOpen": @(state.documentCommandOpen != Standard_False),
        @"documentCommandOpen": @(
            state.documentCommandOpen != Standard_False),
        @"workerActive": @(state.workerActive != Standard_False),
        @"workerPending": @(state.workerPending != Standard_False),
        @"lastComputeWasMainThread": @(
            state.lastComputeWasMainThread != Standard_False),
        @"submittedCount": @(
            static_cast<unsigned long long>(state.submittedCount)),
        @"startedCount": @(
            static_cast<unsigned long long>(state.startedCount)),
        @"completedCount": @(
            static_cast<unsigned long long>(state.completedCount)),
        @"cancelledCount": @(
            static_cast<unsigned long long>(state.cancelledCount)),
        @"pendingReplacementCount": @(
            static_cast<unsigned long long>(
                state.pendingReplacementCount)),
        @"acceptedCount": @(
            static_cast<unsigned long long>(state.acceptedCount)),
        @"staleSuppressionCount": @(
            static_cast<unsigned long long>(
                state.staleSuppressionCount)),
        @"thickness": @(state.thickness),
        @"minimumThickness": @(state.minimumThickness),
        @"maximumThickness": @(state.maximumThickness),
        @"metersPerUnit": @(state.metersPerUnit),
        @"openingFaceCount": @(state.capturedFaceTopologyIndices.size()),
        @"capturedFaceTopologyIndex": @(
            state.capturedFaceTopologyIndex),
        @"sourceTopologyNodeCount": @(
            state.sourceTopologyNodeCount),
        @"candidateTopologyNodeCount": @(
            state.candidateTopologyNodeCount),
        @"candidateSolidCount": @(state.candidateSolidCount),
        @"sourceVolume": @(state.sourceVolume),
        @"candidateVolume": @(state.candidateVolume),
        @"candidateMinX": @(state.candidateBounds[0]),
        @"candidateMinY": @(state.candidateBounds[1]),
        @"candidateMinZ": @(state.candidateBounds[2]),
        @"candidateMaxX": @(state.candidateBounds[3]),
        @"candidateMaxY": @(state.candidateBounds[4]),
        @"candidateMaxZ": @(state.candidateBounds[5]),
    };
}

- (void)debugSetShellPreviewWorkerBlocked:(BOOL)blocked {
    if (_viewer != nullptr) {
        _viewer->DebugSetShellPreviewWorkerBlocked(blocked);
    }
}

- (void)debugSetMaximumShellCaptureTopologyNodes:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->DebugSetMaximumShellCaptureTopologyNodes(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetMaximumShellResultTopologyNodes:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->DebugSetMaximumShellResultTopologyNodes(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetShellTransactionFailureCount:(NSUInteger)count {
    if (_viewer != nullptr) {
        _viewer->DebugSetShellTransactionFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetShellAbortFailureCount:(NSUInteger)count {
    if (_viewer != nullptr) {
        _viewer->DebugSetShellAbortFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetShellPreviewEraseFailureCount:(NSUInteger)count {
    if (_viewer != nullptr) {
        _viewer->DebugSetShellPreviewEraseFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetShellCommitMode:(NSInteger)mode {
    if (_viewer != nullptr) {
        _viewer->DebugSetShellCommitMode(
            static_cast<Standard_Integer>(mode));
    }
}

- (void)debugSetShellPostCommitInspectFailureCount:(NSUInteger)count {
    if (_viewer != nullptr) {
        _viewer->DebugSetShellPostCommitInspectFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (BOOL)debugMutateShellSourcePersistedTransform {
    return _viewer != nullptr
        && _viewer->DebugMutateShellSourcePersistedTransform();
}

- (BOOL)debugMutateShellSourcePersistedShape {
    return _viewer != nullptr
        && _viewer->DebugMutateShellSourcePersistedShape();
}

- (NSDictionary<NSString *, NSNumber *> *)debugBevelState {
    if (_viewer == nullptr || _viewer->getShapeInteractor() == nullptr) {
        return @{};
    }
    const BevelPreviewDebugState state =
        _viewer->DebugBevelPreviewState();
    const BOOL previewActive =
        state.state == BevelPreviewState::Ready && state.canApply;
    return @{
        @"state": @(static_cast<NSUInteger>(state.state)),
        @"generation": @(state.generation),
        @"selectionReady": @(state.activeOperation != Standard_False),
        @"activeOperation": @(state.activeOperation != Standard_False),
        @"previewActive": @(previewActive),
        @"canApply": @(state.canApply != Standard_False),
        @"selectionFrozen": @(state.selectionFrozen != Standard_False),
        @"commandOpen": @(state.documentCommandOpen != Standard_False),
        @"workerActive": @(state.workerActive != Standard_False),
        @"workerPending": @(state.workerPending != Standard_False),
        @"lastComputeWasMainThread": @(
            state.lastComputeWasMainThread != Standard_False),
        @"submittedCount": @(state.submittedCount),
        @"startedCount": @(state.startedCount),
        @"completedCount": @(state.completedCount),
        @"cancelledCount": @(state.cancelledCount),
        @"pendingReplacementCount": @(state.pendingReplacementCount),
        @"acceptedCount": @(state.acceptedCount),
        @"staleSuppressionCount": @(state.staleSuppressionCount),
        @"sourceCount": @(state.sourceCount),
        @"sourceEdgeCount": @(state.sourceEdgeCount),
        @"capturedEdgeCount": @(state.edgeCount),
        @"capturedEdgeTopologyIndex": @(
            state.capturedEdgeTopologyIndex),
        @"capturedEdgeLength": @(state.capturedEdgeLength),
        @"signedDistance": @(state.value),
        @"isFillet": @(state.value > 0.0),
        @"isChamfer": @(state.value < 0.0),
        @"candidateTopologyNodeCount": @(
            state.candidateTopologyNodeCount),
        @"candidateSolidCount": @(state.candidateSolidCount),
        @"candidateVolume": @(state.candidateVolume),
        @"candidateMinX": @(state.candidateBounds[0]),
        @"candidateMinY": @(state.candidateBounds[1]),
        @"candidateMinZ": @(state.candidateBounds[2]),
        @"candidateMaxX": @(state.candidateBounds[3]),
        @"candidateMaxY": @(state.candidateBounds[4]),
        @"candidateMaxZ": @(state.candidateBounds[5]),
    };
}

- (void)debugSetBevelPreviewWorkerBlocked:(BOOL)blocked {
    if (_viewer != nullptr) {
        _viewer->DebugSetBevelPreviewWorkerBlocked(blocked);
    }
}

- (void)debugSetMaximumBevelCaptureTopologyNodes:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->DebugSetMaximumBevelCaptureTopologyNodes(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetMaximumBevelResultTopologyNodes:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->DebugSetMaximumBevelResultTopologyNodes(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetMaximumBevelResultSolids:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->DebugSetMaximumBevelResultSolids(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetBevelTransactionFailureCount:(NSUInteger)count {
    if (_viewer != nullptr) {
        _viewer->DebugSetBevelTransactionFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetBevelCancelDiscardFailureCount:(NSUInteger)count {
    if (_viewer != nullptr) {
        _viewer->DebugSetBevelCancelDiscardFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (BOOL)debugMutateFirstBevelSourcePersistedTransform {
    return _viewer != nullptr
        && _viewer->DebugMutateFirstBevelSourcePersistedTransform();
}

- (NSDictionary<NSString *, id> *_Nullable)debugViewportRGBA {
    GLView *view = self.isViewLoaded ? [self viewportView] : nil;
    return _viewer == nullptr || view == nil ? nil : [view debugDrawAndReadViewportRGBA];
}

- (NSDictionary<NSString *, NSNumber *> *)debugFramebufferStatistics {
    GLView *view = self.isViewLoaded ? [self viewportView] : nil;
    constexpr NSUInteger width = 96;
    constexpr NSUInteger height = 96;
    NSData *frame = _viewer == nullptr || view == nil
        ? nil
        : [view debugDrawAndReadCenteredRGBAWithWidth:width
                                               height:height];
    if (frame == nil || frame.length != width * height * 4) {
        return @{};
    }

    const Standard_Byte *pixels =
        static_cast<const Standard_Byte *>(frame.bytes);
    Standard_Size nearBlackCount = 0;
    Standard_Size chromaticCount = 0;
    Standard_Size redDominantCount = 0;
    Standard_Size greenDominantCount = 0;
    Standard_Size blueDominantCount = 0;
    Standard_Size brightNeutralCount = 0;
    Standard_Integer minimumRed = 255;
    Standard_Integer minimumGreen = 255;
    Standard_Integer minimumBlue = 255;
    Standard_Integer maximumRed = 0;
    Standard_Integer maximumGreen = 0;
    Standard_Integer maximumBlue = 0;
    for (NSUInteger pixelIndex = 0;
         pixelIndex < width * height;
         ++pixelIndex) {
        const Standard_Integer red = pixels[pixelIndex * 4];
        const Standard_Integer green = pixels[pixelIndex * 4 + 1];
        const Standard_Integer blue = pixels[pixelIndex * 4 + 2];
        minimumRed = std::min(minimumRed, red);
        minimumGreen = std::min(minimumGreen, green);
        minimumBlue = std::min(minimumBlue, blue);
        maximumRed = std::max(maximumRed, red);
        maximumGreen = std::max(maximumGreen, green);
        maximumBlue = std::max(maximumBlue, blue);
        const Standard_Integer maximum = std::max({red, green, blue});
        const Standard_Integer minimum = std::min({red, green, blue});
        if (maximum <= 12) {
            ++nearBlackCount;
        }
        if (maximum >= 32 && maximum - minimum >= 24) {
            ++chromaticCount;
        }
        if (red >= green + 24 && red >= blue + 24) {
            ++redDominantCount;
        }
        if (green >= red + 24 && green >= blue + 24) {
            ++greenDominantCount;
        }
        if (blue >= red + 24 && blue >= green + 24) {
            ++blueDominantCount;
        }
        if (minimum >= 96 && maximum - minimum < 24) {
            ++brightNeutralCount;
        }
    }
    const Standard_Size pixelCount = width * height;
    return @{
        @"captured": @YES,
        @"depthCaptured": @NO,
        @"width": @(width),
        @"height": @(height),
        @"centerRed": @(pixels[((height / 2) * width + width / 2) * 4]),
        @"centerGreen": @(pixels[((height / 2) * width + width / 2) * 4 + 1]),
        @"centerBlue": @(pixels[((height / 2) * width + width / 2) * 4 + 2]),
        @"pixelCount": @(pixelCount),
        @"nearBlackCount": @(nearBlackCount),
        @"chromaticCount": @(chromaticCount),
        @"redDominantCount": @(redDominantCount),
        @"greenDominantCount": @(greenDominantCount),
        @"blueDominantCount": @(blueDominantCount),
        @"brightNeutralCount": @(brightNeutralCount),
        @"modelPixelCount": @(pixelCount),
        @"modelNearBlackCount": @(nearBlackCount),
        @"modelChromaticCount": @(chromaticCount),
        @"modelRedDominantCount": @(redDominantCount),
        @"modelGreenDominantCount": @(greenDominantCount),
        @"modelBlueDominantCount": @(blueDominantCount),
        @"modelBrightNeutralCount": @(brightNeutralCount),
        @"minimumRed": @(minimumRed),
        @"minimumGreen": @(minimumGreen),
        @"minimumBlue": @(minimumBlue),
        @"maximumRed": @(maximumRed),
        @"maximumGreen": @(maximumGreen),
        @"maximumBlue": @(maximumBlue),
    };
}

- (NSArray<NSDictionary<NSString *, NSNumber *> *> *)
    debugDisplayedShapePresentationStates {
    if (_viewer == nullptr || _viewer->AisContext().IsNull()
        || _viewer->getDocument().IsNull()) {
        return @[];
    }
    AIS_ListOfInteractive displayed;
    _viewer->AisContext()->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
    NSMutableArray<NSDictionary<NSString *, NSNumber *> *> *states =
        [NSMutableArray arrayWithCapacity:
            static_cast<NSUInteger>(displayed.Size())];
    for (AIS_ListIteratorOfListOfInteractive item(displayed);
         item.More(); item.Next()) {
        const Handle(AIS_Shape) shape =
            Handle(AIS_Shape)::DownCast(item.Value());
        if (shape.IsNull() || shape->Shape().IsNull()) {
            continue;
        }
        const gp_Trsf aLocalTransform = shape->LocalTransformation();
        const gp_XYZ translation = aLocalTransform.TranslationPart();
        const gp_Quaternion rotation = aLocalTransform.GetRotation();
        gp_Trsf aWorldTransform = aLocalTransform;
        aWorldTransform.Multiply(
            shape->Shape().Location().Transformation());
        const gp_XYZ aWorldTranslation =
            aWorldTransform.TranslationPart();
        Quantity_Color color(Quantity_NOC_BLACK);
        if (shape->HasColor()) {
            shape->Color(color);
        }
        Standard_Real red = 0.0;
        Standard_Real green = 0.0;
        Standard_Real blue = 0.0;
        color.Values(red, green, blue, Quantity_TOC_sRGB);
        Standard_Real metallic = -1.0;
        Standard_Real roughness = -1.0;
        Standard_Integer alphaMode = 999;
        Standard_Real alphaCutoff = -1.0;
        Standard_Integer faceCulling = 999;
        Standard_Boolean textureMapOn = Standard_False;
        Standard_Integer textureSetSize = 0;
        Standard_Boolean textureIsXCAFBridge = Standard_False;
        Standard_Integer textureUnit = -1;
        Standard_Size textureSourceByteCount = 0;
        Standard_Boolean textureDecodedImageAvailable = Standard_False;
        Standard_Integer textureDecodedWidth = 0;
        Standard_Integer textureDecodedHeight = 0;
        Standard_Integer textureDecodedFormat = -1;
        Standard_Boolean textureDecodedTopDown = Standard_False;
        std::array<Standard_Integer, 16> textureDecodedQuadrants;
        textureDecodedQuadrants.fill(-1);
        Standard_Boolean documentHasBaseColorTexture = Standard_False;
        Standard_Boolean documentHasEmissiveTexture = Standard_False;
        Standard_Boolean textureSourceMatchesDocument = Standard_False;
        Standard_Boolean textureSetHasBaseColorUnit = Standard_False;
        Standard_Boolean textureSetHasEmissiveUnit = Standard_False;
        Standard_Boolean baseColorTextureSourceMatchesDocument =
            Standard_False;
        Standard_Boolean emissiveTextureSourceMatchesDocument =
            Standard_False;
        Handle(Image_Texture) documentBaseColorTexture;
        Handle(Image_Texture) documentEmissiveTexture;
        const TDF_Label presentationLabel =
            _viewer->getDocument()->ShapeLabel(shape);
        const OcctGeometryRepresentation presentationRepresentation =
            presentationLabel.IsNull()
                ? OcctGeometryRepresentation::Invalid
                : _viewer->getDocument()->GeometryRepresentationForLabel(
                    presentationLabel);
        XCAFDoc_VisMaterialPBR effectivePresentationMaterial;
        if (!presentationLabel.IsNull()
            && _viewer->getDocument()->TryEffectivePBRMaterialForLabel(
                presentationLabel, effectivePresentationMaterial)) {
            if (!effectivePresentationMaterial.BaseColorTexture.IsNull()) {
                documentHasBaseColorTexture = Standard_True;
                documentBaseColorTexture =
                    effectivePresentationMaterial.BaseColorTexture;
            }
            if (!effectivePresentationMaterial.EmissiveTexture.IsNull()) {
                documentHasEmissiveTexture = Standard_True;
                documentEmissiveTexture =
                    effectivePresentationMaterial.EmissiveTexture;
            }
        }
        bool hasAppDataMapShader = false;
        const auto isDataMapShader = [](const Handle(Graphic3d_AspectFillArea3d)& aspect) {
            return !aspect.IsNull() && !aspect->ShaderProgram().IsNull()
                && aspect->ShaderProgram()->GetId().StartsWith("shapeyard-data-maps-v1-");
        };
        const Handle(Prs3d_Drawer)& drawer = shape->Attributes();
        if (!drawer.IsNull() && !drawer->ShadingAspect().IsNull()) {
            const Graphic3d_PBRMaterial& pbrMaterial =
                drawer->ShadingAspect()->Material().PBRMaterial();
            metallic = pbrMaterial.Metallic();
            roughness = pbrMaterial.NormalizedRoughness();
            const Handle(Graphic3d_AspectFillArea3d)& fillAspect =
                drawer->ShadingAspect()->Aspect();
            hasAppDataMapShader = isDataMapShader(fillAspect);
            if (!fillAspect.IsNull()) {
                alphaMode = static_cast<Standard_Integer>(
                    fillAspect->AlphaMode());
                alphaCutoff = fillAspect->AlphaCutoff();
                faceCulling = static_cast<Standard_Integer>(
                    fillAspect->FaceCulling());
                textureMapOn = fillAspect->ToMapTexture();
                const Handle(Graphic3d_TextureSet)& textureSet =
                    fillAspect->TextureSet();
                textureSetSize = textureSet.IsNull()
                    ? 0
                    : textureSet->Size();
                if (!textureSet.IsNull() && !textureSet->IsEmpty()) {
                    const Handle(XCAFPrs_Texture) texture =
                        Handle(XCAFPrs_Texture)::DownCast(
                            textureSet->First());
                    if (!texture.IsNull()) {
                        textureIsXCAFBridge = Standard_True;
                        if (!texture->GetParams().IsNull()) {
                            textureUnit = static_cast<Standard_Integer>(
                                texture->GetParams()->TextureUnit());
                        }
                        const Handle(Image_Texture)& source =
                            texture->GetImageSource();
                        if (!source.IsNull()
                            && !source->DataBuffer().IsNull()) {
                            textureSourceByteCount =
                                source->DataBuffer()->Size();
                            // Exercise the renderer wrapper: numeric bindings
                            // deliberately decode differently from color ones.
                            const Handle(Image_PixMap) decoded =
                                texture->GetImage(
                                    Handle(Image_SupportedFormats)());
                            if (!decoded.IsNull()
                                && !decoded->IsEmpty()) {
                                textureDecodedImageAvailable =
                                    Standard_True;
                                textureDecodedWidth =
                                    static_cast<Standard_Integer>(
                                        decoded->SizeX());
                                textureDecodedHeight =
                                    static_cast<Standard_Integer>(
                                        decoded->SizeY());
                                textureDecodedFormat =
                                    static_cast<Standard_Integer>(
                                        decoded->Format());
                                textureDecodedTopDown =
                                    decoded->IsTopDown();
                                if (decoded->Format()
                                        == Image_Format_RGBA) {
                                    const Standard_Size aQuarterX =
                                        decoded->SizeX() / 4;
                                    const Standard_Size aQuarterY =
                                        decoded->SizeY() / 4;
                                    const Standard_Size threeQuarterX =
                                        decoded->SizeX() * 3 / 4;
                                    const Standard_Size threeQuarterY =
                                        decoded->SizeY() * 3 / 4;
                                    const std::array<
                                        std::array<Standard_Size, 2>,
                                        4> coordinates = {{
                                            {{aQuarterX, aQuarterY}},
                                            {{threeQuarterX, aQuarterY}},
                                            {{aQuarterX, threeQuarterY}},
                                            {{threeQuarterX,
                                              threeQuarterY}},
                                        }};
                                    for (Standard_Integer aSample = 0;
                                         aSample < 4;
                                         ++aSample) {
                                        const Standard_Byte* aPixel =
                                            decoded->RawValueXY(
                                                coordinates[aSample][0],
                                                coordinates[aSample][1]);
                                        for (Standard_Integer aChannel = 0;
                                             aChannel < 4;
                                             ++aChannel) {
                                            textureDecodedQuadrants[
                                                aSample * 4 + aChannel] =
                                                aPixel[aChannel];
                                        }
                                    }
                                }
                            }
                        }
                        if (!documentBaseColorTexture.IsNull()) {
                            textureSourceMatchesDocument =
                                Core3DBaseColorTexturesMatch(
                                    source,
                                    documentBaseColorTexture);
                        }
                    }
                    for (Standard_Integer aTextureIndex = textureSet->Lower();
                         aTextureIndex <= textureSet->Upper();
                         ++aTextureIndex) {
                        const Handle(XCAFPrs_Texture) aTexture =
                            Handle(XCAFPrs_Texture)::DownCast(
                                textureSet->Value(aTextureIndex));
                        if (aTexture.IsNull()
                            || aTexture->GetParams().IsNull()) {
                            continue;
                        }
                        const Graphic3d_TextureUnit aUnit =
                            aTexture->GetParams()->TextureUnit();
                        const Handle(Image_Texture)& aSource =
                            aTexture->GetImageSource();
                        if (aUnit == Graphic3d_TextureUnit_BaseColor) {
                            textureSetHasBaseColorUnit = Standard_True;
                            if (!documentBaseColorTexture.IsNull()) {
                                baseColorTextureSourceMatchesDocument =
                                    Core3DTexturesMatch(
                                        aSource, documentBaseColorTexture);
                            }
                        } else if (aUnit
                                   == Graphic3d_TextureUnit_Emissive) {
                            textureSetHasEmissiveUnit = Standard_True;
                            if (!documentEmissiveTexture.IsNull()) {
                                emissiveTextureSourceMatchesDocument =
                                    Core3DTexturesMatch(
                                        aSource, documentEmissiveTexture);
                            }
                        }
                    }
                }
            }
        }
        const Handle(CafShapePrs) cafShape =
            Handle(CafShapePrs)::DownCast(shape);
        Standard_Real defaultStyleRed = -1.0;
        Standard_Real defaultStyleGreen = -1.0;
        Standard_Real defaultStyleBlue = -1.0;
        Standard_Real defaultStyleMetallic = -1.0;
        Standard_Real defaultStyleRoughness = -1.0;
        Standard_Boolean defaultStyleHasMaterial = Standard_False;
        if (!cafShape.IsNull()) {
            XCAFPrs_Style defaultStyle;
            cafShape->DefaultStyle(defaultStyle);
            Quantity_Color defaultStyleColor(Quantity_NOC_BLACK);
            if (defaultStyle.IsSetColorSurf()) {
                defaultStyleColor = defaultStyle.GetColorSurf();
            } else if (defaultStyle.IsSetColorCurv()) {
                defaultStyleColor = defaultStyle.GetColorCurv();
            } else if (!defaultStyle.Material().IsNull()) {
                defaultStyleColor =
                    defaultStyle.Material()->BaseColor().GetRGB();
            }
            defaultStyleColor.Values(
                defaultStyleRed,
                defaultStyleGreen,
                defaultStyleBlue,
                Quantity_TOC_sRGB);
            if (!defaultStyle.Material().IsNull()) {
                defaultStyleHasMaterial = Standard_True;
                Graphic3d_MaterialAspect defaultStyleAspect;
                defaultStyle.Material()->FillMaterialAspect(
                    defaultStyleAspect);
                const Graphic3d_PBRMaterial& defaultStylePBR =
                    defaultStyleAspect.PBRMaterial();
                defaultStyleMetallic = defaultStylePBR.Metallic();
                defaultStyleRoughness =
                    defaultStylePBR.NormalizedRoughness();
            }
        }
        Handle(AIS_ColoredDrawer) rootCustomAspects;
        const Standard_Boolean hasRootCustomAspects =
            !cafShape.IsNull()
            && cafShape->FindCustomAspects(
                shape->Shape(), rootCustomAspects);
        Standard_Integer customMaterialOverrideCount = 0;
        Standard_Integer customColorOverrideCount = 0;
        Standard_Integer customTextureMapOnCount = 0;
        Standard_Integer customDataMapShaderCount = 0;
        Standard_Integer customNonDefaultAlphaCount = 0;
        Standard_Integer customNonAutoFaceCullingCount = 0;
        // Live primitives and reloaded XCAF presentations both support face
        // drawers. Inspect the shared base instead of hiding live overrides.
        const auto coloredShape = Handle(AIS_ColoredShape)::DownCast(shape);
        if (!coloredShape.IsNull()) {
            for (AIS_DataMapOfShapeDrawer::Iterator anOverride(
                     coloredShape->CustomAspectsMap());
                 anOverride.More(); anOverride.Next()) {
                const Handle(AIS_ColoredDrawer)& drawer =
                    anOverride.Value();
                if (!drawer.IsNull() && drawer->HasOwnMaterial()) {
                    ++customMaterialOverrideCount;
                }
                if (!drawer.IsNull() && drawer->HasOwnColor()) {
                    ++customColorOverrideCount;
                }
                if (!drawer.IsNull() && drawer->HasOwnShadingAspect()
                    && !drawer->ShadingAspect().IsNull()
                    && !drawer->ShadingAspect()->Aspect().IsNull()) {
                    const Handle(Graphic3d_AspectFillArea3d)& aspect =
                        drawer->ShadingAspect()->Aspect();
                    if (aspect->ToMapTexture()) {
                        ++customTextureMapOnCount;
                    }
                    if (isDataMapShader(aspect)) ++customDataMapShaderCount;
                    if (aspect->AlphaMode()
                            != Graphic3d_AlphaMode_BlendAuto
                        || std::abs(aspect->AlphaCutoff() - 0.5f)
                            > 1.0e-6f) {
                        ++customNonDefaultAlphaCount;
                    }
                    if (aspect->FaceCulling()
                            != Graphic3d_TypeOfBackfacingModel_Auto) {
                        ++customNonAutoFaceCullingCount;
                    }
                }
            }
        }
        TColStd_ListOfInteger activeSelectionModes;
        _viewer->AisContext()->ActivatedModes(
            shape, activeSelectionModes);
        Standard_Boolean objectSelectionModeActive = Standard_False;
        Standard_Boolean faceSelectionModeActive = Standard_False;
        Standard_Boolean edgeSelectionModeActive = Standard_False;
        Standard_Boolean vertexSelectionModeActive = Standard_False;
        for (TColStd_ListIteratorOfListOfInteger mode(activeSelectionModes);
             mode.More(); mode.Next()) {
            const Standard_Integer value = mode.Value();
            objectSelectionModeActive = objectSelectionModeActive
                || value == AIS_Shape::SelectionMode(TopAbs_SHAPE);
            faceSelectionModeActive = faceSelectionModeActive
                || value == AIS_Shape::SelectionMode(TopAbs_FACE);
            edgeSelectionModeActive = edgeSelectionModeActive
                || value == AIS_Shape::SelectionMode(TopAbs_EDGE);
            vertexSelectionModeActive = vertexSelectionModeActive
                || value == AIS_Shape::SelectionMode(TopAbs_VERTEX);
        }
        [states addObject:@{
            @"translationX": @(translation.X()),
            @"translationY": @(translation.Y()),
            @"translationZ": @(translation.Z()),
            @"rotationX": @(rotation.X()),
            @"rotationY": @(rotation.Y()),
            @"rotationZ": @(rotation.Z()),
            @"rotationW": @(rotation.W()),
            @"scaleFactor": @(aLocalTransform.ScaleFactor()),
            @"transform11": @(aLocalTransform.Value(1, 1)),
            @"transform12": @(aLocalTransform.Value(1, 2)),
            @"transform13": @(aLocalTransform.Value(1, 3)),
            @"transform14": @(aLocalTransform.Value(1, 4)),
            @"transform21": @(aLocalTransform.Value(2, 1)),
            @"transform22": @(aLocalTransform.Value(2, 2)),
            @"transform23": @(aLocalTransform.Value(2, 3)),
            @"transform24": @(aLocalTransform.Value(2, 4)),
            @"transform31": @(aLocalTransform.Value(3, 1)),
            @"transform32": @(aLocalTransform.Value(3, 2)),
            @"transform33": @(aLocalTransform.Value(3, 3)),
            @"transform34": @(aLocalTransform.Value(3, 4)),
            @"worldTranslationX": @(aWorldTranslation.X()),
            @"worldTranslationY": @(aWorldTranslation.Y()),
            @"worldTranslationZ": @(aWorldTranslation.Z()),
            @"hasColor": @(shape->HasColor()),
            @"red": @(red),
            @"green": @(green),
            @"blue": @(blue),
            @"metallic": @(metallic),
            @"roughness": @(roughness),
            @"alphaMode": @(alphaMode),
            @"alphaCutoff": @(alphaCutoff),
            @"faceCulling": @(faceCulling),
            @"isCafPresentation": @(!cafShape.IsNull()),
            @"textureMapOn": @(textureMapOn != Standard_False),
            @"textureSetSize": @(textureSetSize),
            @"textureIsXCAFBridge": @(
                textureIsXCAFBridge != Standard_False),
            @"textureUnit": @(textureUnit),
            @"textureSourceByteCount": @(textureSourceByteCount),
            @"textureDecodedImageAvailable": @(
                textureDecodedImageAvailable != Standard_False),
            @"textureDecodedWidth": @(textureDecodedWidth),
            @"textureDecodedHeight": @(textureDecodedHeight),
            @"textureDecodedFormat": @(textureDecodedFormat),
            @"textureDecodedIsRGBA": @(
                textureDecodedFormat
                    == static_cast<Standard_Integer>(
                        Image_Format_RGBA)),
            @"textureDecodedTopDown": @(
                textureDecodedTopDown != Standard_False),
            @"textureDecodedTopLeftRed": @(
                textureDecodedQuadrants[0]),
            @"textureDecodedTopLeftGreen": @(
                textureDecodedQuadrants[1]),
            @"textureDecodedTopLeftBlue": @(
                textureDecodedQuadrants[2]),
            @"textureDecodedTopLeftAlpha": @(
                textureDecodedQuadrants[3]),
            @"textureDecodedTopRightRed": @(
                textureDecodedQuadrants[4]),
            @"textureDecodedTopRightGreen": @(
                textureDecodedQuadrants[5]),
            @"textureDecodedTopRightBlue": @(
                textureDecodedQuadrants[6]),
            @"textureDecodedTopRightAlpha": @(
                textureDecodedQuadrants[7]),
            @"textureDecodedBottomLeftRed": @(
                textureDecodedQuadrants[8]),
            @"textureDecodedBottomLeftGreen": @(
                textureDecodedQuadrants[9]),
            @"textureDecodedBottomLeftBlue": @(
                textureDecodedQuadrants[10]),
            @"textureDecodedBottomLeftAlpha": @(
                textureDecodedQuadrants[11]),
            @"textureDecodedBottomRightRed": @(
                textureDecodedQuadrants[12]),
            @"textureDecodedBottomRightGreen": @(
                textureDecodedQuadrants[13]),
            @"textureDecodedBottomRightBlue": @(
                textureDecodedQuadrants[14]),
            @"textureDecodedBottomRightAlpha": @(
                textureDecodedQuadrants[15]),
            @"documentHasBaseColorTexture": @(
                documentHasBaseColorTexture != Standard_False),
            @"documentHasEmissiveTexture": @(
                documentHasEmissiveTexture != Standard_False),
            @"textureSourceMatchesDocument": @(
                textureSourceMatchesDocument != Standard_False),
            @"textureSetHasBaseColorUnit": @(
                textureSetHasBaseColorUnit != Standard_False),
            @"textureSetHasEmissiveUnit": @(
                textureSetHasEmissiveUnit != Standard_False),
            @"baseColorTextureSourceMatchesDocument": @(
                baseColorTextureSourceMatchesDocument != Standard_False),
            @"emissiveTextureSourceMatchesDocument": @(
                emissiveTextureSourceMatchesDocument != Standard_False),
            @"defaultStyleHasMaterial": @(defaultStyleHasMaterial),
            @"defaultStyleRed": @(defaultStyleRed),
            @"defaultStyleGreen": @(defaultStyleGreen),
            @"defaultStyleBlue": @(defaultStyleBlue),
            @"defaultStyleMetallic": @(defaultStyleMetallic),
            @"defaultStyleRoughness": @(defaultStyleRoughness),
            @"isAssemblyOccurrence": @(
                !cafShape.IsNull()
                && !cafShape->IsEditablePresentation()),
            @"isEditable": @(
                _viewer->getDocument()->IsPresentationEditable(shape)),
            @"hasRootCustomAspects": @(hasRootCustomAspects),
            @"customMaterialOverrideCount": @(
                customMaterialOverrideCount),
            @"customColorOverrideCount": @(
                customColorOverrideCount),
            @"customTextureMapOnCount": @(
                customTextureMapOnCount),
            @"hasAppDataMapShader": @(hasAppDataMapShader),
            @"customDataMapShaderCount": @(customDataMapShaderCount),
            @"customNonDefaultAlphaCount": @(
                customNonDefaultAlphaCount),
            @"customNonAutoFaceCullingCount": @(
                customNonAutoFaceCullingCount),
            @"activeSelectionModeCount": @(
                activeSelectionModes.Extent()),
            @"geometryRepresentation": @(
                static_cast<NSInteger>(presentationRepresentation)),
            @"objectSelectionModeActive": @(
                objectSelectionModeActive != Standard_False),
            @"faceSelectionModeActive": @(
                faceSelectionModeActive != Standard_False),
            @"edgeSelectionModeActive": @(
                edgeSelectionModeActive != Standard_False),
            @"vertexSelectionModeActive": @(
                vertexSelectionModeActive != Standard_False),
        }];
    }
    return states;
}
#endif

- (NSInteger)numberOfDetectedEdges {
    return _viewer->getShapeInteractor()->getNumberOfDetectedEdges();
}

- (void)assetData:(void(^)(NSData *_Nullable))completion {
    if (![NSThread isMainThread] || _queuedAssetOwner != nil
        || _pendingAssetDataCount == NSUIntegerMax) {
        dispatch_async(dispatch_get_main_queue(), ^{ completion(nil); });
        return;
    }
    ++_pendingAssetDataCount;
    __weak typeof(self) weakSelf = self;
    dispatch_async(_assetDataQueue, ^{
        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf) {
            dispatch_async(dispatch_get_main_queue(), ^{
                completion(nil);
            });
            return;
        }

        NSString *tmpFilename = [NSString stringWithFormat:@"%@.tmp", NSUUID.UUID.UUIDString];
        NSURL *tmpDirectory = [NSFileManager.defaultManager temporaryDirectory];
        NSURL *baseURL = [tmpDirectory URLByAppendingPathComponent:tmpFilename];
        NSString *expectedCbfPath = [baseURL.path stringByAppendingString:@".cbf"];
        NSString *expectedXbfPath = [baseURL.path stringByAppendingString:@".xbf"];
        const std::string fn = baseURL.path.UTF8String;

        __block std::string cbfFilePath;
        dispatch_sync(dispatch_get_main_queue(), ^{
            try {
                if (strongSelf->_viewer != nullptr) {
                    const auto objectInteractor =
                        strongSelf->_viewer->getObjectInteractor();
                    const auto shapeInteractor =
                        strongSelf->_viewer->getShapeInteractor();
                    const bool hasTransientModeling =
                        objectInteractor == nullptr
                        || shapeInteractor == nullptr
                        || strongSelf->_viewer->hasUnresolvedEdit()
                        || objectInteractor->hasActiveBoolean()
                        || objectInteractor->hasUnresolvedBoolean()
                        || objectInteractor->hasUnresolvedMirrorObjects()
                        || objectInteractor->hasActiveLinearArray()
                        || objectInteractor->hasUnresolvedLinearArray()
                        || objectInteractor->hasActiveRadialArray()
                        || objectInteractor->hasUnresolvedRadialArray()
                        || shapeInteractor->hasActiveExtrusion()
                        || shapeInteractor->hasActiveShell()
                        || shapeInteractor->hasUnresolvedShell();
                    // Bevel previews own only transient AIS presentations and
                    // never retain an open OCAF command. Apply is synchronous
                    // on this main queue, so serializing here always captures
                    // a coherent committed document (not the transient trial).
                    // Keep recovery autosaves working while a user evaluates
                    // or retries a Bevel preview.
                    if (!hasTransientModeling) {
                        cbfFilePath = strongSelf->_viewer
                            ->getDocument()->save(fn);
                    }
                    if (!cbfFilePath.empty()) {
                        const AssetImportResult validation =
                            strongSelf->_viewer->ValidateCbf(cbfFilePath);
                        if (validation != AssetImportResult::Success) {
                            NSLog(@"CBF validation failed after save: %d at %@",
                                  static_cast<int>(validation),
                                  [NSString stringForStdString:cbfFilePath]);
                            cbfFilePath.clear();
                        }
                    }
                }
            } catch (...) {
                cbfFilePath.clear();
            }
        });

        NSString *dataPath = cbfFilePath.empty()
            ? nil
            : [NSString stringForStdString:cbfFilePath];
        NSError *error = nil;
        NSData *data = dataPath == nil
            ? nil
            : [NSData dataWithContentsOfFile:dataPath options:kNilOptions error:&error];

        NSFileManager *fileManager = NSFileManager.defaultManager;
        [fileManager removeItemAtURL:baseURL error:nil];
        [fileManager removeItemAtPath:expectedCbfPath error:nil];
        [fileManager removeItemAtPath:expectedXbfPath error:nil];
        if (dataPath != nil
            && ![dataPath isEqualToString:expectedCbfPath]
            && ![dataPath isEqualToString:expectedXbfPath]) {
            [fileManager removeItemAtPath:dataPath error:nil];
        }

        if (error != nil || !HasCbfMagic(data)) {
            data = nil;
        }

        dispatch_async(dispatch_get_main_queue(), ^{
            // An earlier serialization owns its completion before a replacement
            // may reserve this controller's queue. Clear before client reentry.
            if (strongSelf->_pendingAssetDataCount > 0) --strongSelf->_pendingAssetDataCount;
            completion(data);
        });
    });
}

// Ready-only admission retains exact request ownership through preparation,
// native adoption and verified private-file cleanup. No public AI authority.

- (Core3DAssetLoadResult)acceptQueuedAssetInput:(Core3DQueuedAssetInput *)input
                                       owner:(Core3DQueuedAssetLoadOwner **)acceptedOwner
                                    progress:(void (^)(BOOL))progress
                                  completion:(void (^)(Core3DAssetLoadResult))completion {
    if (acceptedOwner != nullptr) *acceptedOwner = nil;
    if (![NSThread isMainThread] || _queuedAssetOwner != nil || _pendingAssetDataCount != 0
        || _rawTouchRendering || _pinchRendering || _panRendering)
        return Core3DAssetLoadResultBusy;
    if (input == nil || completion == nil || acceptedOwner == nullptr)
        return Core3DAssetLoadResultInvalidData;
    id origin = self.delegate;
    if ([origin isKindOfClass:Core3DViewController.class]
        && ![(Core3DViewController *)origin core3d_canReserveQueuedInput:input fromGLController:self])
        return Core3DAssetLoadResultBusy;
    Core3DQueuedAssetLoadOwner *owner = [Core3DQueuedAssetLoadOwner
        reserveInput:input glController:self];
    if (owner == nil) return Core3DAssetLoadResultBusy;
    _queuedAssetOwner = owner;
    _queuedAssetCompletion = [completion copy];
    _queuedAssetProgress = [progress copy];
    _queuedPreparationReceived = NO;
    _queuedPrivateFile = nil;
    _queuedCleanupPending = NO;
    _queuedCleanupInFlight = NO;
    *acceptedOwner = owner;
    // Ready-only admission rejected existing raw/pan/pinch gestures. Keep new
    // touches from starting on this viewport until its exact load has settled.
    _queuedInteractionView = self.isViewLoaded ? self.view : nil;
    _queuedViewWasInteractive = _queuedInteractionView.userInteractionEnabled;
    _queuedInteractionView.userInteractionEnabled = NO;
    if (![owner claimPrivateStartForGLController:self]) {
        // Keep the exact owner installed even on an unexpected native-start
        // rejection. Never drop an unsettled reservation via a local release.
        _queuedAssetResult = Core3DAssetLoadResultInternalFailure;
        dispatch_async(dispatch_get_main_queue(), ^{
            GLViewController *controller = [owner renderingControllerOnMain];
            [controller core3d_cleanupQueuedOwner:owner];
        });
        return Core3DAssetLoadResultSuccess; // Accepted; terminal result is async.
    }
    // Only the carrier crosses the worker boundary. It explicitly clears every
    // native handle and its GL lease on main before it can be last-released here.
    dispatch_async(_assetDataQueue, ^{
        Core3DQueuedPrivateFile *file = nil;
        const Core3DAssetLoadResult staged = StageQueuedAssetInput(owner.input, &file);
        dispatch_async(dispatch_get_main_queue(), ^{
            GLViewController *controller = [owner renderingControllerOnMain];
#ifdef DEBUG
            if (controller->_debugPauseQueuedAdoption) {
                controller->_debugStagedPrivateFile = file;
                controller->_debugResumeQueuedAdoption = ^{
                    [controller core3d_receiveStagedQueuedOwner:owner file:file result:staged];
                };
                return;
            }
#endif
            [controller core3d_receiveStagedQueuedOwner:owner file:file result:staged];
        });
    });
    return Core3DAssetLoadResultSuccess;
}

- (void)core3d_receiveStagedQueuedOwner:(Core3DQueuedAssetLoadOwner *)owner
                                  file:(Core3DQueuedPrivateFile *)file
                                result:(Core3DAssetLoadResult)result {
    NSCAssert([NSThread isMainThread], @"Queued native adoption belongs to main");
    NSCAssert(_queuedAssetOwner == owner && [owner ownsGLController:self],
        @"Only the exact accepted owner can complete private preparation");
    if (_queuedAssetOwner != owner || ![owner ownsGLController:self]) return;
    if (_queuedPreparationReceived) return;
    _queuedPreparationReceived = YES;
    _queuedPrivateFile = file; // May be nonnil even when preparation failed.
    _queuedAssetResult = result;
    if (result == Core3DAssetLoadResultSuccess) {
        _queuedAssetResult = file == nil || ![file hasExactSealedIdentity]
            ? Core3DAssetLoadResultInvalidData
            : [owner adoptPrivateFile:file.URL glController:self];
        if (_queuedAssetResult == Core3DAssetLoadResultSuccess) [self requestRender];
    }
    [self core3d_cleanupQueuedOwner:owner];
}

- (void)abandonQueuedAssetOwner:(Core3DQueuedAssetLoadOwner *)owner {
    if (![NSThread isMainThread] || _queuedAssetOwner != owner) return;
    [owner abandonForGLController:self];
    // Abandonment suppresses adoption, not required cleanup or settlement.
}

- (void)retryQueuedAssetCleanup {
    if ([NSThread isMainThread] && _queuedAssetOwner != nil && _queuedCleanupPending)
        [self core3d_cleanupQueuedOwner:_queuedAssetOwner];
}

- (void)core3d_cleanupQueuedOwner:(Core3DQueuedAssetLoadOwner *)owner {
    if (![NSThread isMainThread] || _queuedAssetOwner != owner
        || _queuedCleanupInFlight || ![owner ownsGLController:self]) return;
    _queuedCleanupInFlight = YES;
    Core3DQueuedPrivateFile *file = _queuedPrivateFile;
    BOOL forceFailure = NO;
#ifdef DEBUG
    forceFailure = _debugQueuedCleanupFailures > 0;
    if (forceFailure) --_debugQueuedCleanupFailures;
#endif
    dispatch_async(_assetDataQueue, ^{
        // This path comes only from the accepted stager's exclusive creation,
        // never from model/user input. No directory scan or recursive removal.
        const BOOL removed = !forceFailure && (file == nil || [file removeOwnedFile]);
        dispatch_async(dispatch_get_main_queue(), ^{
            GLViewController *controller = [owner renderingControllerOnMain];
            [controller core3d_finishCleanupForQueuedOwner:owner removed:removed];
        });
    });
}

- (void)core3d_finishCleanupForQueuedOwner:(Core3DQueuedAssetLoadOwner *)owner
                                  removed:(BOOL)removed {
    if (![NSThread isMainThread] || _queuedAssetOwner != owner
        || ![owner ownsGLController:self] || !_queuedCleanupInFlight) return;
    _queuedCleanupInFlight = NO;
    if (!removed || ![owner settleNativeAfterPrivateCleanup:YES glController:self]) {
        const BOOL wasPending = _queuedCleanupPending;
        _queuedCleanupPending = YES;
        if (!wasPending && _queuedAssetProgress != nil) _queuedAssetProgress(YES);
        // Retain the exact request/path and native fence. A retry also survives
        // Core editor teardown because the carrier retains its rendering lease.
        // Compose a distinct progress callback; do not send terminal failure or
        // clear Core's accepted request while native ownership is still held.
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 30 * NSEC_PER_SEC),
            dispatch_get_main_queue(), ^{
                GLViewController *controller = [owner renderingControllerOnMain];
                [controller retryQueuedAssetCleanup];
            });
        return;
    }
    _queuedCleanupPending = NO;
    _queuedPrivateFile = nil;
    const Core3DAssetLoadResult result = _queuedAssetResult;
    void (^completion)(Core3DAssetLoadResult) = _queuedAssetCompletion;
    _queuedAssetCompletion = nil;
    _queuedAssetProgress = nil;
    _queuedAssetOwner = nil;
    if (self.isViewLoaded && self.view == _queuedInteractionView)
        _queuedInteractionView.userInteractionEnabled = _queuedViewWasInteractive;
    _queuedInteractionView = nil;
    // self is a strong local in the main-queue caller. The carrier explicitly
    // empties its GL lease before the worker's carrier reference can disappear.
    const BOOL released = [owner releaseRenderingLeaseForGLController:self];
    NSCAssert(released, @"A settled queued owner must release its rendering lease");
    if (released && completion != nil) completion(result);
}

- (void)setAssetData:(NSData *)data completion:(void (^)(Core3DAssetLoadResult))completion {
    Core3DAssetLoadResult result = Core3DAssetLoadResultBusy;
    if ([NSThread isMainThread] && _queuedAssetOwner == nil) {
        Core3DQueuedAssetInput *input = [Core3DQueuedAssetInput freezeData:data];
        Core3DQueuedAssetLoadOwner *owner = nil;
        result = input == nil ? Core3DAssetLoadResultInvalidData
            : [self acceptQueuedAssetInput:input owner:&owner progress:nil completion:completion];
    }
    if (result != Core3DAssetLoadResultSuccess) CompleteAssetLoadOnMain(completion, result);
}
- (void)setAssetFileURL:(NSURL *)URL expectedByteCount:(unsigned long long)count
    expectedSHA256:(NSString *)sha completion:(void (^)(Core3DAssetLoadResult))completion {
    Core3DAssetLoadResult result = Core3DAssetLoadResultBusy;
    if ([NSThread isMainThread] && _queuedAssetOwner == nil) {
        Core3DQueuedAssetInput *input = [Core3DQueuedAssetInput freezeVerifiedFile:URL
            expectedByteCount:count expectedSHA256:sha];
        Core3DQueuedAssetLoadOwner *owner = nil;
        result = input == nil ? Core3DAssetLoadResultInvalidData
            : [self acceptQueuedAssetInput:input owner:&owner progress:nil completion:completion];
    }
    if (result != Core3DAssetLoadResultSuccess) CompleteAssetLoadOnMain(completion, result);
}

#ifdef DEBUG
- (void)debugResumeQueuedAssetAdoption { [self debugPauseQueuedAssetAdoption:NO]; }
- (void)debugPauseQueuedAssetAdoption:(BOOL)paused {
    if (![NSThread isMainThread]) return;
    _debugPauseQueuedAdoption = paused;
    if (!paused && _debugResumeQueuedAdoption != nil) {
        void (^resume)(void) = _debugResumeQueuedAdoption;
        _debugResumeQueuedAdoption = nil;
        _debugStagedPrivateFile = nil;
        resume();
    }
}
- (void)debugFailQueuedAssetCleanup:(NSUInteger)count {
    if ([NSThread isMainThread]) _debugQueuedCleanupFailures = MIN(count, 3u);
}
- (NSDictionary<NSString *, id> *)debugQueuedAssetLoadState {
    if (![NSThread isMainThread]) return @{};
    Core3DQueuedPrivateFile *file = _queuedPrivateFile ?: _debugStagedPrivateFile;
    return @{ @"active": @(_queuedAssetOwner != nil),
        @"paused": @(_debugResumeQueuedAdoption != nil),
        @"cleanupPending": @(_queuedCleanupPending),
        @"nativeReady": @(_viewer != nullptr && _viewer->canBeginCommittedEdit()),
        @"privatePath": file.URL.path ?: @"",
        @"privateFileExists": @(file != nil && [NSFileManager.defaultManager fileExistsAtPath:file.URL.path]) };
}
// D333/D326: exposes the real teardown measurement from OcctDocument.mm. A
// leftover relocation-table payload at driver death is the state whose
// destructor terminates the Xcode Cloud test host.
- (void)debugResetBoundedRetrievalDriverTeardownStats {
    if ([NSThread isMainThread])
        Core3DDebugResetBoundedRetrievalDriverTeardownStats();
}
- (NSDictionary<NSString *, id> *)debugBoundedRetrievalDriverTeardownStats {
    if (![NSThread isMainThread]) return @{};
    unsigned long long teardowns = 0, retainedEntries = 0, retainedNamedShapes = 0;
    Core3DDebugBoundedRetrievalDriverTeardownStats(
        teardowns, retainedEntries, retainedNamedShapes);
    return @{ @"teardowns": @(teardowns),
        @"retainedEntries": @(retainedEntries),
        @"retainedNamedShapes": @(retainedNamedShapes) };
}
// Arms the D333 fault injection: the bounded retrieval driver throws out of a
// subtree read once its relocation table really holds a TNaming_NamedShape,
// simulating the cloud crash precondition (an OCCT exception escaping a failed
// retrieval after bindings, which skips the base reader's table Clear).
- (void)debugSetRelocationReadThrowAfterNamedShapeBinding:(NSNumber *)enabled {
    if ([NSThread isMainThread])
        Core3DDebugSetRelocationReadThrowAfterNamedShapeBinding(
            enabled.boolValue ? 1 : 0);
}
#endif

- (NSData *)thumbData {
    NSString *tmpSnapthotFilename = [NSString stringWithFormat:@"%@.tmp.png", NSUUID.UUID.UUIDString];
    NSURL *tmpUrl = [[NSFileManager.defaultManager temporaryDirectory] URLByAppendingPathComponent:tmpSnapthotFilename];
    NSLog(@"Snapshot: %@", tmpUrl);
    if (![self saveSnapshot:tmpUrl]) {
        [NSFileManager.defaultManager removeItemAtURL:tmpUrl error:nil];
        return NULL;
    }
    NSError *error = nil;
    __auto_type data = [NSData dataWithContentsOfURL:tmpUrl options:kNilOptions error:&error];

    [NSFileManager.defaultManager removeItemAtURL:tmpUrl error:nil];

    if (error) {
        NSLog(@"ERROR: with thumb: %@", error.localizedDescription);
        return NULL;
    }

    return data;
}

- (BOOL)saveSnapshot {
    NSData *data = [self thumbData];
    UIImage *image = data == nil ? nil : [UIImage imageWithData:data];
    if (image == nil) {
        return NO;
    }
    dispatch_async(dispatch_get_main_queue(), ^{
        UIImageWriteToSavedPhotosAlbum(image, nil, nil, nil);
    });
    return YES;
}

- (BOOL)saveSnapshot:(NSURL *)tmpUrl {
    if (_viewer == nullptr || _viewer->hasUnresolvedEdit()) {
        return NO;
    }
    const auto fn = TCollection_AsciiString(tmpUrl.path.UTF8String);
    return _viewer->dumpOfDisplayedColoredObjects(500, 500, tmpUrl.path.UTF8String);
}

- (NSURL *_Nullable)exportWithType:(ExportType)exportType {
    if (_viewer == nullptr) {
        return nil;
    }
    const Handle(OcctDocument) document = _viewer->getDocument();
    const Handle(TDocStd_Document) transaction = document.IsNull()
        ? Handle(TDocStd_Document)()
        : document->ChangeDocument();
    const auto objectInteractor = _viewer->getObjectInteractor();
    const auto shapeInteractor = _viewer->getShapeInteractor();
    OcctGeometryExportFormat geometryExportFormat;
    switch (exportType) {
        case ExportTypeObj:
            geometryExportFormat = OcctGeometryExportFormat::Obj;
            break;
        case ExportTypeStl:
            geometryExportFormat = OcctGeometryExportFormat::Stl;
            break;
        case ExportTypeGltf:
            geometryExportFormat = OcctGeometryExportFormat::Gltf;
            break;
        case ExportTypeStep:
            geometryExportFormat = OcctGeometryExportFormat::Step;
            break;
        default:
            return nil;
    }
    if (transaction.IsNull() || transaction->HasOpenCommand()
        || !document->CanExportGeometry(geometryExportFormat)
        || objectInteractor == nullptr
        || shapeInteractor == nullptr
        || (_viewer->hasUnresolvedOrdinaryEdit() || objectInteractor->hasUnresolvedDuplicate())
        || objectInteractor->hasActiveBoolean()
        || objectInteractor->hasUnresolvedBoolean()
        || objectInteractor->hasUnresolvedMirrorObjects()
        || objectInteractor->hasActiveLinearArray()
        || objectInteractor->hasUnresolvedLinearArray()
        || objectInteractor->hasActiveRadialArray()
        || objectInteractor->hasUnresolvedRadialArray()
        || shapeInteractor->hasActiveExtrusion()
        || shapeInteractor->hasActiveBevel()
        || shapeInteractor->hasActiveShell()
        || shapeInteractor->hasUnresolvedShell()) {
        return nil;
    }
    NSString *pathExtension = NULL;
    switch (exportType) {
        case ExportTypeObj:
            pathExtension = @"obj";
            break;
        case ExportTypeStl:
            pathExtension = @"stl";
            break;
        case ExportTypeGltf:
            pathExtension = @"glb";
            break;
        case ExportTypeStep:
            pathExtension = @"step";
            break;
        default:
            assert(false);
            break;
    }

    NSString *exportFilename = [NSUUID.UUID.UUIDString
        stringByAppendingPathExtension:pathExtension];
    NSURL *exportUrl = [[NSFileManager.defaultManager temporaryDirectory] URLByAppendingPathComponent:exportFilename];
    void (^removeExportArtifacts)(void) = ^{
        [NSFileManager.defaultManager removeItemAtURL:exportUrl error:nil];
        if (exportType == ExportTypeObj) {
            NSURL *baseUrl = [exportUrl URLByDeletingPathExtension];
            [NSFileManager.defaultManager
                removeItemAtURL:[baseUrl URLByAppendingPathExtension:@"mtl"]
                         error:nil];
            [NSFileManager.defaultManager removeItemAtURL:baseUrl error:nil];
            NSURL *textureDirectory = [NSURL fileURLWithPath:
                [baseUrl.path stringByAppendingString:@"_textures"]];
            [NSFileManager.defaultManager
                removeItemAtURL:textureDirectory
                         error:nil];
        }
    };

    const auto exportPath = std::string(exportUrl.path.UTF8String);

    switch (exportType) {
        case ExportTypeObj:
            _viewer->getShapeInteractor()->exportToObj(exportPath);
            break;
        case ExportTypeStl:
            _viewer->getShapeInteractor()->exportToStl(exportPath);
            break;
        case ExportTypeGltf:
            _viewer->getShapeInteractor()->exportToGltf(exportPath);
            break;
        case ExportTypeStep:
            _viewer->getShapeInteractor()->exportToStep(exportPath);
            break;
        default:
            assert(false);
            break;
    }


    // Verify the file was actually created
    if (![NSFileManager.defaultManager fileExistsAtPath:exportUrl.path]) {
        NSLog(@"[Export] File was NOT created at: %@", exportUrl.path);
        removeExportArtifacts();
        return nil;
    }
    
    NSDictionary *attrs = [NSFileManager.defaultManager attributesOfItemAtPath:exportUrl.path error:nil];
    NSLog(@"[Export] File created: %@ (%llu bytes)", exportUrl.lastPathComponent, [attrs fileSize]);
    
    if ([attrs fileSize] == 0) {
        NSLog(@"[Export] File is empty, returning nil");
        removeExportArtifacts();
        return nil;
    }
    
    return exportUrl;
}

- (BOOL)isPreviewMode {
    return _isPreviewMode;
}

- (void)setOrthoProjection:(OrthoProjectionType)orthoType {
    _viewer->setOrthoProjection(orthoType);
    [self requestRender];
}

// =======================================================================
// function : displayAboutDlg
// purpose  :
// =======================================================================
- (void)displayAboutDlg:(UIBarButtonItem *)theSender
{
  UIAlertController* anAbout = [UIAlertController alertControllerWithTitle:@"About"
                                message:@"UIKit based application for tutorial to Open CASCADE Technology.\n\n"
                                      @"Copyright (c) 2017 OPEN CASCADE SAS"
                                preferredStyle:UIAlertControllerStyleAlert];
  
  UIAlertAction* aDefaultAction = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault
                                                         handler:^(UIAlertAction * action) {}];
  
  [anAbout addAction:aDefaultAction];
  [self presentViewController:anAbout animated:YES completion:nil];
}

@end
