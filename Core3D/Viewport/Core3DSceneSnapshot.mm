//
//  Core3DSceneSnapshot.mm
//  Core3D
//


#import "Core3DSceneSnapshot.h"
#import "Core3DSceneSnapshotFactory.hpp"
#import <ImageIO/ImageIO.h>
#import <CommonCrypto/CommonDigest.h>

#include "../Common/Core3DMobileResourceLimits.h"
#include "../Scene/SceneSnapshot.hpp"
#include "../Scene/MikkTangentSpace.hpp"
#if DEBUG
#include "../Scene/AuthoredTangentArchive.hpp"
#include "../OCCTKit/Core3DBoundedAuthoredFrameDriver.hxx"
#include "../OCCTKit/NativeAuthoredFrameGeometry.hxx"
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/NativeTransactionObserverProbe.hxx"
#include "../OCCTKit/NativeLiveTransactionObserverProbe.hxx"
#include "../OCCTKit/NativeManualIntentPolicyProbe.hxx"
#include "../OCCTKit/NativeQueuedLoadPolicyProbe.hxx"
#include "../OCCTKit/NativeReplacementPolicyProbe.hxx"
#include "../OCCTKit/NativeIntentReplacementPromotionProbe.hxx"
#include "../OCCTKit/CurrentTessellationMeshCopy.hxx"
#include "../OCCTKit/NativeMeshElementSelectionTests.hxx"
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeTorus.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepTools.hxx>
#include "../Import/Core3DGLBReader.hpp"
#include <Message_ProgressRange.hxx>
#include <cstdio>
#include <unistd.h>
#include "../UI/OrdinaryEditController.hpp"
#include <BRep_Builder.hxx>
#include <TopoDS.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_Real.hxx>
#include <TDF_Tool.hxx>
#include <gp_Ax1.hxx>
#include <TDF_ChildIterator.hxx>
#include <TopoDS_Compound.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <Message.hxx>
#include <sstream>
#endif
#include <cstring>
#include <Quantity_Color.hxx>
#include <Quantity_NameOfColor.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
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

@interface Core3DSceneTextureSnapshot ()
- (instancetype)initWithIdentifier:(NSString *)identifier
                           encoding:(Core3DSceneTextureEncoding)encoding
                         pixelWidth:(uint32_t)pixelWidth
                        pixelHeight:(uint32_t)pixelHeight
                        encodedData:(NSData *)encodedData;
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
                           cullMode:(Core3DSceneCullMode)cullMode
              baseColorTextureIndex:(NSInteger)baseColorTextureIndex
               emissiveTextureIndex:(NSInteger)emissiveTextureIndex
      metallicRoughnessTextureIndex:(NSInteger)metallicRoughnessTextureIndex
              occlusionTextureIndex:(NSInteger)occlusionTextureIndex
                 normalTextureIndex:(NSInteger)normalTextureIndex;
@end

@interface Core3DSceneFacePrimitiveSnapshot ()
- (instancetype)initWithFirstIndex:(uint32_t)firstIndex
                         indexCount:(uint32_t)indexCount
                          faceIndex:(uint32_t)faceIndex
              hasTextureCoordinates:(BOOL)hasTextureCoordinates;
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
                                    faceCount:(uint32_t)faceCount
                                    edgeCount:(uint32_t)edgeCount
                           topologyVertexCount:(uint32_t)topologyVertexCount
                            cornerTangentData:(NSData *)cornerTangentData
                            tangentIdentifier:(NSString *)tangentIdentifier
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
                         hasReferenceAxis:(BOOL)hasReferenceAxis
                      referencePivotWorld:(simd_double3)referencePivotWorld
                  referenceDirectionWorld:(simd_double3)referenceDirectionWorld
                      referencePivotSpace:(Core3DSceneReferenceSpace)referencePivotSpace
                  referenceDirectionSpace:(Core3DSceneReferenceSpace)referenceDirectionSpace
                    referenceAxisAuthored:(BOOL)referenceAxisAuthored
                                  winding:(Core3DSceneWinding)winding
                                  visible:(BOOL)visible
                               selectable:(BOOL)selectable
                                 selected:(BOOL)selected
                                     name:(NSString *)name
                          groupIdentifier:(NSString *)groupIdentifier
                                groupName:(NSString *)groupName
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
                         metersPerUnit:(double)metersPerUnit
                          renderOrigin:(simd_double3)renderOrigin
                         selectionMode:(Core3DSceneElementKind)selectionMode
                                meshes:(NSArray<Core3DSceneMeshSnapshot *> *)meshes
                           renderItems:(NSArray<Core3DSceneRenderItemSnapshot *> *)renderItems
                             materials:(NSArray<Core3DSceneMaterialSnapshot *> *)materials
                              textures:(NSArray<Core3DSceneTextureSnapshot *> *)textures
                             pickTable:(NSArray<Core3DSceneElementIdentifier *> *)pickTable
                                camera:(Core3DSceneCameraSnapshot *)camera
                             selection:(Core3DSceneSelectionSnapshot *)selection;
@end


#if DEBUG
TopoDS_Face Core3DDebugAuthoredGeometryFixture(NSInteger mode) {
    Handle(Poly_Triangulation) mesh = new Poly_Triangulation(mode == 22 ? 5 : 4, mode == 22 ? 2 : 1, Standard_True, Standard_True);
    mesh->SetNode(1, gp_Pnt(9, -0.0, 3)); // Deliberately unused, still authoritative.
    mesh->SetNode(2, gp_Pnt(0, 0, 0)); mesh->SetNode(3, gp_Pnt(0, 0, 10));
    mesh->SetNode(4, gp_Pnt(8, -6, 0));
    mesh->SetUVNode(1, gp_Pnt2d(-0.0, 0.25)); mesh->SetUVNode(2, gp_Pnt2d(0, 0));
    mesh->SetUVNode(3, gp_Pnt2d(1, 0)); mesh->SetUVNode(4, gp_Pnt2d(0, 1));
    for (int n = 1; n <= 4; ++n) mesh->SetNormal(n, gp_Vec3f(0.6f, 0.8f, -0.0f));
    mesh->SetTriangle(1, Poly_Triangle(2, 3, 4));
    if (mode == 22) {
        mesh->SetNode(5, gp_Pnt(8,-6,10)); mesh->SetUVNode(5, gp_Pnt2d(1,1));
        mesh->SetNormal(5, gp_Vec3f(0.6f,0.8f,-0.0f)); mesh->SetTriangle(2, Poly_Triangle(3,5,4));
    }
    if (mode == 1) mesh = mesh->Copy();
    if (mode == 2) mesh->SetNode(3, gp_Pnt(0, 0, 11));
    if (mode == 3) mesh->SetNormal(3, gp_Vec3f(0.8f, 0.6f, 0));
    if (mode == 4) mesh->SetUVNode(4, gp_Pnt2d(0, 0.75));
    if (mode == 5) mesh->SetTriangle(1, Poly_Triangle(2, 4, 3));
    if (mode == 6) mesh->SetNode(1, gp_Pnt(9, 0, 4));
    if (mode == 9) mesh->RemoveNormals();
    if (mode == 10) mesh->RemoveUVNodes();
    if (mode == 11) mesh->SetTriangle(1, Poly_Triangle(2, 3, 5));
    if (mode == 12) mesh->SetNode(1, gp_Pnt(std::numeric_limits<double>::infinity(), 0, 0));
    if (mode == 13) mesh->SetNormal(1, gp_Vec3f(0, 0, 0));
    if (mode == 15) mesh->ResizeNodes(int(core3d::scene::kMaximumTangentVertices) + 1, Standard_True);
    if (mode == 16) mesh->ResizeTriangles(int(core3d::scene::kMaximumTangentTriangles) + 1, Standard_True);
    if (mode == 17) mesh->SetUVNode(1, gp_Pnt2d(std::numeric_limits<double>::infinity(), 0));
    if (mode == 18) mesh->SetNode(1, gp_Pnt(1.e6 + 1, 0, 0));
    if (mode == 19) mesh->SetNormal(1, gp_Vec3f(std::numeric_limits<float>::quiet_NaN(), 0, 0));
    if (mode == 20) mesh->SetTriangle(1, Poly_Triangle(2, 2, 4));
    BRep_Builder builder; TopoDS_Face face;
    if (mode == 21) builder.MakeFace(face); else builder.MakeFace(face, mesh);
    if (mode == 7) face.Reverse();
    if (mode == 8) { gp_Trsf move; move.SetTranslation(gp_Vec(1, 2, 3)); face.Location(TopLoc_Location(move)); }
    return face;
}
#endif

@implementation Core3DSceneTangentSpace
+ (nullable NSData *)cornerTangentsForVertexData:(NSData *)vertexData
                              triangleIndexData:(NSData *)triangleIndexData
                          hasTextureCoordinates:(BOOL)hasTextureCoordinates
                                          error:(NSError * _Nullable * _Nullable)error {
    auto reject = [error](TangentSpaceError reason) -> NSData* {
        if (error) *error = [NSError errorWithDomain:@"Core3DSceneTangentSpace"
            code:static_cast<NSInteger>(reason)
            userInfo:@{NSLocalizedDescriptionKey: @"This mesh cannot produce supported tangent frames."}];
        return nil;
    };
    if (error) *error = nil;
    if (!hasTextureCoordinates) return reject(TangentSpaceError::MissingUVs);
    if (vertexData.length == 0 || vertexData.length % sizeof(Vertex) != 0
        || triangleIndexData.length == 0 || triangleIndexData.length % (3*sizeof(uint32_t)) != 0)
        return reject(TangentSpaceError::InvalidLayout);
    if (vertexData.length / sizeof(Vertex) > kMaximumTangentVertices
        || triangleIndexData.length / (3*sizeof(uint32_t)) > kMaximumTangentTriangles)
        return reject(TangentSpaceError::ResourceLimit);
    try {
        // NSData's byte address need not be aligned. Own aligned inputs before
        // C++ access; the immutable caller buffers and their indices survive.
        std::vector<Vertex> vertices(vertexData.length / sizeof(Vertex));
        std::vector<uint32_t> indices(triangleIndexData.length / sizeof(uint32_t));
        std::memcpy(vertices.data(), vertexData.bytes, vertexData.length);
        std::memcpy(indices.data(), triangleIndexData.bytes, triangleIndexData.length);
        std::vector<Float4> frames;
        const auto result = GenerateMikkCornerTangents(vertices, indices, true, frames);
        if (result != TangentSpaceError::None) return reject(result);
        static_assert(sizeof(Float4) == 4*sizeof(float));
        return [NSData dataWithBytes:frames.data() length:frames.size()*sizeof(Float4)];
    } catch (...) { return reject(TangentSpaceError::GenerationFailed); }
}
#if DEBUG
+ (nullable NSData *)debugEncodeAuthoredFrames:(NSData *)frames
                             geometryIdentity:(NSData *)geometryIdentity
                                        error:(NSError * _Nullable * _Nullable)error {
    using namespace core3d::scene::authored;
    auto reject = [error](ArchiveStatus status) -> NSData* {
        if (error) *error = [NSError errorWithDomain:@"Core3DAuthoredFrameArchive" code:NSInteger(status)
            userInfo:@{NSLocalizedDescriptionKey: @"Invalid bounded authored-frame archive."}];
        return nil;
    };
    if (error) *error = nil;
    if (geometryIdentity.length != 32 || frames.length == 0 || frames.length % 16)
        return reject(ArchiveStatus::InvalidLayout);
    if (frames.length / 16 > kMaximumFrames) return reject(ArchiveStatus::ResourceLimit);
    try {
        GeometryIdentity identity; std::memcpy(identity.data(), geometryIdentity.bytes, 32);
        std::vector<Float4> values(frames.length / 16);
        static_assert(sizeof(Float4) == 16);
        std::memcpy(values.data(), frames.bytes, frames.length);
        std::vector<std::uint8_t> archive;
        const auto status = Encode(values, identity, archive);
        if (status != ArchiveStatus::Valid) return reject(status);
        return [NSData dataWithBytes:archive.data() length:archive.size()];
    } catch (...) { return reject(ArchiveStatus::AllocationFailure); }
}
+ (nullable NSData *)debugDecodeAuthoredFrames:(NSData *)archive
                             geometryIdentity:(NSData *)geometryIdentity
                                        error:(NSError * _Nullable * _Nullable)error {
    using namespace core3d::scene::authored;
    auto reject = [error](ArchiveStatus status) -> NSData* {
        if (error) *error = [NSError errorWithDomain:@"Core3DAuthoredFrameArchive" code:NSInteger(status)
            userInfo:@{NSLocalizedDescriptionKey: @"Invalid bounded authored-frame archive."}];
        return nil;
    };
    if (error) *error = nil;
    if (geometryIdentity.length != 32) return reject(ArchiveStatus::InvalidLayout);
    GeometryIdentity identity; std::memcpy(identity.data(), geometryIdentity.bytes, 32);
    std::vector<Float4> frames;
    const auto status = Decode(static_cast<const std::uint8_t*>(archive.bytes), archive.length, identity, frames);
    if (status != ArchiveStatus::Valid) return reject(status);
    return [NSData dataWithBytes:frames.data() length:frames.size() * 16];
}
#endif
#if DEBUG
+ (NSDictionary<NSString *, id> *)debugPersistentFrameRecord:(NSData *)archive
                                                      mode:(NSInteger)mode
                                                    budget:(NSUInteger)limit {
    using namespace core3d::scene::authored;
    using namespace core3d::persistence;
    if (archive.length < 128 || archive.length > kMaximumArchiveBytes || mode < 0 || mode > 22)
        return @{@"error": @"Invalid isolated frame fixture"};
    try {
        const auto messenger = Message::DefaultMessenger();
        auto budget = std::make_shared<AuthoredFrameReadBudget>(); budget->limit = limit;
        BoundedAuthoredFrameDriver bounded(messenger, budget);
        BinMDataStd_ByteArrayDriver stock(messenger);
        Handle(TDataStd_ByteArray) attribute = new TDataStd_ByteArray();
        attribute->Init(0, Standard_Integer(archive.length) - 1);
        attribute->SetID(AuthoredFrameAttributeID()); attribute->SetDelta(Standard_False);
        const auto* input = static_cast<const std::uint8_t*>(archive.bytes);
        for (NSUInteger i = 0; i < archive.length; ++i) attribute->SetValue(Standard_Integer(i), input[i]);
        BinObjMgt_SRelocationTable writeRelocation;
        BinObjMgt_Persistent record; record.SetTypeId(1); record.SetId(1);
        stock.Paste(attribute, record, writeRelocation); record.Truncate();
        auto wire = [](BinObjMgt_Persistent& value) {
            std::ostringstream stream(std::ios::binary | std::ios::out);
            value.Write(stream); return stream.str();
        };
        const auto stockBytes = wire(record);
        // OCCT Write resets the record cursor and logical size. Read back only
        // the bounded bytes just produced here; arbitrary outer headers are not admitted.
        if (stockBytes.size() > kMaximumArchiveBytes + 64)
            return @{@"error": @"Unexpected stock frame record size"};
        std::istringstream stockStream(stockBytes, std::ios::binary | std::ios::in);
        if (!record.Read(stockStream) || record.Length() < 0
            || std::size_t(record.Length()) + 12 != stockBytes.size())
            return @{@"error": @"Stock frame record reconstruction failed"};
        BinObjMgt_Persistent written; written.SetTypeId(1); written.SetId(1);
        if (mode >= 20) {
            if (mode == 20) attribute->SetID(TDataStd_ByteArray::GetID());
            if (mode == 21) attribute->SetValue(48, attribute->Value(48) ^ 1);
            if (mode == 22) attribute->SetDelta(Standard_True);
            const auto before = written.Position(); bool rejected = false;
            try { bounded.Paste(attribute, written, writeRelocation); }
            catch (const Standard_Failure&) { rejected = true; }
            return @{@"writerRejected": @(rejected), @"writerPositionUnchanged": @(written.Position() == before)};
        }
        bounded.Paste(attribute, written, writeRelocation); written.Truncate();
        const bool exactWriter = stockBytes == wire(written);
        // Mutate only the small, internally built record. A huge declared
        // array count never requires creating a correspondingly large array.
        const Standard_Integer originalEnd = record.Length() + 12;
        if (mode >= 1 && mode <= 4) {
            record.SetPosition(12);
            record << Standard_Integer(mode == 2 ? -1 : mode == 3 ? 1 : 0);
            record << Standard_Integer(mode == 1 ? std::numeric_limits<Standard_Integer>::max()
                : mode == 4 ? -1 : Standard_Integer(archive.length) - 1);
        }
        if (mode == 5 || mode == 6) {
            record.SetPosition(mode == 5 ? 24 : originalEnd - 16); record.Truncate();
        }
        if (mode == 7 || mode == 8 || mode == 10) {
            record.Init(); record.SetTypeId(1); record.SetId(1);
            record << Standard_Integer(0) << Standard_Integer(archive.length - 1);
            std::vector<std::uint8_t> bytes(input, input + archive.length);
            if (mode == 10) bytes[48] ^= 1;
            record.PutByteArray(bytes.data(), Standard_Integer(bytes.size()));
            record << Standard_Byte(mode == 8 ? 1 : 0);
            record << (mode == 7 ? TDataStd_ByteArray::GetID() : AuthoredFrameAttributeID());
            record.Truncate();
        }
        if (mode == 9) { record.SetPosition(originalEnd); record << Standard_Byte(0x7f); record.Truncate(); }
        if (mode > 14 && mode < 20) return @{@"error": @"Undefined isolated frame mode"};
        BinObjMgt_RRelocationTable readRelocation;
        if (mode != 12) {
            Handle(Storage_HeaderData) header = new Storage_HeaderData();
            header->SetStorageVersion(mode == 11 ? 9 : mode == 13 ? 99 : 12);
            readRelocation.SetHeaderData(header);
        }
        record.SetPosition(12);
        const auto firstTarget = bounded.NewEmpty();
        const bool firstAccepted = bounded.Paste(record, firstTarget, readRelocation);
        bool accepted = firstAccepted;
        Handle(TDF_Attribute) target = firstTarget;
        if (mode == 14 && firstAccepted) {
            record.SetPosition(12); target = bounded.NewEmpty();
            accepted = bounded.Paste(record, target, readRelocation);
        }
        NSMutableData* result = [NSMutableData data];
        if (accepted) {
            const auto value = Handle(TDataStd_ByteArray)::DownCast(target);
            result.length = value->Length();
            auto* out = static_cast<std::uint8_t*>(result.mutableBytes);
            for (Standard_Integer i = 0; i < value->Length(); ++i) out[i] = value->Value(i);
        }
        return @{@"accepted": @(accepted), @"firstAccepted": @(firstAccepted),
            @"rejected": @(budget->rejected), @"chargedBytes": @(budget->bytes),
            @"bytes": result, @"stockWriterMatched": @(exactWriter),
            @"consumedAll": @(record.Position() == record.Length() + 12)};
    } catch (const Standard_Failure& failure) {
        return @{@"error": [NSString stringWithUTF8String:failure.GetMessageString()] ?: @"OCCT failure"};
    } catch (...) { return @{@"error": @"Isolated frame fixture failed"}; }
}
#endif
#if DEBUG
+ (NSDictionary<NSString *, id> *)debugAuthoredGeometryIdentity:(NSInteger)mode {
    using namespace core3d::persistence;
    if (mode < 0 || mode > 21) return @{@"error": @"Undefined geometry fixture"};
    try {
        const TopoDS_Face face = Core3DDebugAuthoredGeometryFixture(mode);
        GeometryIdentity identity; const bool accepted = NativeAuthoredGeometryIdentity(face, identity);
        NSMutableDictionary* result = [@{@"accepted": @(accepted),
            @"identity": [NSData dataWithBytes:identity.data() length:identity.size()]} mutableCopy];
        if (mode == 14 && accepted) {
            // Private documents only, using the existing safe application format.
            // This proves geometry serialization, not a live project adoption.
            struct PrivateDocument {
                Handle(TDocStd_Application) app = new TDocStd_Application();
                Handle(TDocStd_Document) document;
                ~PrivateDocument() noexcept {
                    try { if (!document.IsNull()) app->Close(document); } catch (...) {}
                }
            } writer, reader;
            Core3DDefineSafeBinXCAFFormat(writer.app); Core3DDefineSafeBinXCAFFormat(reader.app);
            writer.app->NewDocument(TCollection_ExtendedString("BinXCAF"), writer.document);
            XCAFDoc_DocumentTool::SetLengthUnit(writer.document, 0.001);
            auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(writer.document->Main());
            const TDF_Label label = shapeTool->AddShape(face, Standard_False);
            if (label.IsNull()) return @{@"error": @"Private geometry label creation failed"};
            std::ostringstream output(std::ios::binary | std::ios::out);
            const auto storeStatus = writer.app->SaveAs(writer.document, output);
            const auto bytes = output.str();
            if (storeStatus != PCDM_SS_OK || bytes.empty() || bytes.size() > 1024 * 1024)
                return @{@"error": @"Private geometry serialization failed"};
            std::istringstream input(bytes, std::ios::binary | std::ios::in);
            Core3DBeginSafeBinaryRead();
            const auto readStatus = reader.app->Open(input, reader.document);
            if (readStatus != PCDM_RS_OK || Core3DSafeBinaryReadWasRejected() || reader.document.IsNull())
                return @{@"error": @"Private geometry retrieval failed"};
            double metersPerUnit = 0;
            if (!XCAFDoc_DocumentTool::GetLengthUnit(reader.document, metersPerUnit) || metersPerUnit != 0.001)
                return @{@"error": @"Private geometry retrieval changed units"};
            result[@"metersPerUnit"] = @(metersPerUnit);
            TDF_LabelSequence labels;
            XCAFDoc_DocumentTool::ShapeTool(reader.document->Main())->GetFreeShapes(labels);
            if (labels.Length() != 1) return @{@"error": @"Private geometry retrieval changed roots"};
            const auto reopenedShape = XCAFDoc_ShapeTool::GetShape(labels.Value(1));
            if (reopenedShape.IsNull() || reopenedShape.ShapeType() != TopAbs_FACE)
                return @{@"error": @"Private geometry retrieval changed shape kind"};
            GeometryIdentity reopened;
            const bool reopenedAccepted = NativeAuthoredGeometryIdentity(TopoDS::Face(reopenedShape), reopened);
            result[@"reopenedAccepted"] = @(reopenedAccepted);
            result[@"reopenedIdentity"] = [NSData dataWithBytes:reopened.data() length:reopened.size()];
            result[@"serializedBytes"] = @(bytes.size());
            result[@"freshNativeDocument"] = @(reader.document != writer.document);
        }
        return result;
    } catch (const Standard_Failure& failure) {
        return @{@"error": [NSString stringWithUTF8String:failure.GetMessageString()] ?: @"OCCT failure"};
    } catch (...) { return @{@"error": @"Geometry fixture failed"}; }
}
#endif
#if DEBUG
+ (NSDictionary<NSString *, id> *)debugNativeFrameAssociation:(NSData *)archive mode:(NSInteger)mode {
    if (mode < 0 || mode > 21) return @{@"error": @"Undefined geometry fixture"};
    try {
        const auto face = Core3DDebugAuthoredGeometryFixture(mode);
        // Seed an old value to verify rejected reads clear existing output.
        std::vector<core3d::scene::Float4> frames(1, {1, 0, 0, 1});
        const bool accepted = core3d::persistence::DecodeNativeAuthoredFrames(face,
            static_cast<const std::uint8_t*>(archive.bytes), archive.length, frames);
        return @{@"accepted": @(accepted), @"frames":
            [NSData dataWithBytes:frames.data() length:frames.size() * sizeof(core3d::scene::Float4)]};
    } catch (...) { return @{@"error": @"Native frame association fixture failed"}; }
}
#endif
#if DEBUG
+ (NSDictionary<NSString *, id> *)debugFrameOwner:(NSData *)archive replacement:(NSData *)replacement mode:(NSInteger)mode {
    if (mode < 0 || mode > 30 || ![NSThread isMainThread]) return @{@"error": @"Undefined frame-owner fixture"};
    try {
        using namespace core3d::persistence;
        struct PrivateDocument {
            Handle(TDocStd_Application) app = new TDocStd_Application();
            Handle(TDocStd_Document) document;
            ~PrivateDocument() noexcept { try { if (!document.IsNull()) app->Close(document); } catch (...) {} }
        } owner;
        Core3DDefineSafeBinXCAFFormat(owner.app);
        owner.app->NewDocument(TCollection_ExtendedString("BinXCAF"), owner.document);
        owner.document->SetUndoLimit(20);
        XCAFDoc_DocumentTool::SetLengthUnit(owner.document, mode == 5 ? 1.0 : 0.001);
        OcctDocument wrapper; wrapper.ChangeDocument() = owner.document;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(owner.document->Main());
        auto fixture = Core3DDebugAuthoredGeometryFixture(mode == 15 ? 2 : mode == 26 ? 4 : mode == 27 ? 1 : 0);
        if (mode == 17) fixture.Reverse();
        if (mode == 16) { gp_Trsf move; move.SetTranslation(gp_Vec(1,2,3)); fixture.Location(TopLoc_Location(move)); }
        TopoDS_Shape shape = fixture;
        if (mode == 1 || mode == 3 || mode == 16) {
            BRep_Builder builder; TopoDS_Compound compound; builder.MakeCompound(compound); builder.Add(compound, fixture); shape = compound;
        }
        if (mode == 2 || mode == 3) { gp_Trsf move; move.SetTranslation(gp_Vec(2,3,4)); shape.Location(TopLoc_Location(move)); }
        auto unplaced = shape; unplaced.Location(TopLoc_Location());
        const auto label = shapes->AddShape(unplaced, Standard_False);
        if (label.IsNull()) Standard_Failure::Raise("No frame-owner definition.");
        if (!shape.Location().IsIdentity()) shapes->SetShape(label, shape);
        TDF_Label second;
        if (mode == 20 || mode == 21) second = shapes->AddShape(Core3DDebugAuthoredGeometryFixture(0), Standard_False);
        owner.document->NewCommand();
        if (!wrapper.SetGeometryRepresentationForLabel(label, OcctGeometryRepresentation::TriangleMesh))
            Standard_Failure::Raise("Could not mark fixture mesh.");
        if (!second.IsNull() && !wrapper.SetGeometryRepresentationForLabel(second, OcctGeometryRepresentation::TriangleMesh))
            Standard_Failure::Raise("Could not mark second fixture mesh.");
        owner.document->CommitCommand(); owner.document->ClearUndos();
        if (!wrapper.MigrateLegacyIdentifiers(owner.document)) Standard_Failure::Raise("Fixture identity migration failed.");
        if (mode == 4) label.ForgetAttribute(Standard_GUID("67E669F4-00C0-4C45-BC55-9CC5DA22A2B5"));
        auto assign = [&](const TDF_Label& target, NSData* value, bool defaultID = false) {
            const int first = mode == 18 ? 1 : 0;
            const auto array = TDataStd_ByteArray::Set(target,
                defaultID ? TDataStd_ByteArray::GetID() : mode == 29 ? Standard_GUID("1BF816B1-BDFA-488D-B209-C06851D26C18") : AuthoredFrameAttributeID(),
                first, first + int(value.length) - 1, mode == 19);
            const auto* input = static_cast<const std::uint8_t*>(value.bytes);
            for (NSUInteger i = 0; i < value.length; ++i) array->SetValue(first + int(i), input[i]);
        };
        TDF_Label frameLabel = label;
        if (mode == 7) frameLabel = owner.document->GetData()->Root();
        if (mode == 8) frameLabel = owner.document->Main();
        if (mode == 9) frameLabel = label.FindChild(91, Standard_True);
        if (mode == 10) shapes->RemoveShape(label, Standard_True);
        if (mode == 11) {
            const auto assembly = shapes->NewShape();
            frameLabel = shapes->AddComponent(assembly, label, TopLoc_Location());
            if (frameLabel.IsNull()) Standard_Failure::Raise("No reference fixture label.");
        }
        if (mode == 28) XCAFDoc_DocumentTool::ColorTool(owner.document->Main())->SetVisibility(label, Standard_False);
        NSMutableData* input = [archive mutableCopy];
        if (mode == 14 && input.length) static_cast<std::uint8_t*>(input.mutableBytes)[input.length - 1] ^= 1;
        owner.document->NewCommand();
        if (mode == 12) TDataStd_Integer::Set(frameLabel, AuthoredFrameAttributeID(), 1);
        else if (mode != 22) assign(frameLabel, input, mode == 13);
        if (!second.IsNull()) assign(second, archive);
        if (mode == 23) assign(owner.document->GetData()->Root(), archive, true);
        if (mode == 24) TDataStd_Integer::Set(owner.document->GetData()->Root(), AuthoredFrameAttributeID(), 1);
        owner.document->CommitCommand();
        auto countLabels = [&]() { int count = 0; for (TDF_ChildIterator it(owner.document->GetData()->Root(), Standard_True); it.More(); it.Next()) ++count; return count; };
        const int beforeTime = owner.document->GetData()->Time(), beforeLabels = countLabels();
        const int beforeUndo = owner.document->GetAvailableUndos(), beforeRedo = owner.document->GetAvailableRedos();
        Handle(TDocStd_Document) foreign = new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
        const auto readLabel = mode == 6 ? foreign->Main() : frameLabel;
        OcctAuthoredFrameRecord record; record.archive = {42}; record.identity.fill(42); record.nativeBytes = 777; record.cornerCount = 99;
        const auto state = Core3DReadAuthoredFrameOwner(owner.document, readLabel, record);
        Standard_Size documentBytes = 777;
        const Standard_Size limit = mode == 20 ? 640 : mode == 21 ? 639 : mode == 22 ? 0 : mode == 25 ? 64U * 1024U * 1024U + 1U : 64U * 1024U * 1024U;
        const bool validDocument = Core3DValidateAuthoredFrameOwners(owner.document, documentBytes, limit);
        OcctObjectTransformState snapshot;
        const bool captured = wrapper.CaptureObjectTransformStateForLabel(readLabel, snapshot);
        NSMutableDictionary* result = [@{@"readState": @(int(state)), @"archive": [NSData dataWithBytes:record.archive.data() length:record.archive.size()],
            @"identity": [NSData dataWithBytes:record.identity.data() length:record.identity.size()], @"cornerCount": @(record.cornerCount), @"nativeBytes": @(record.nativeBytes),
            @"documentAccepted": @(validDocument), @"documentNativeBytes": @(documentBytes), @"captureAccepted": @(captured),
            @"snapshotPresent": @(snapshot.authoredFramesPresent), @"snapshotIdentity": [NSData dataWithBytes:snapshot.authoredFramesIdentity.data() length:snapshot.authoredFramesIdentity.size()],
            @"readOnly": @(beforeTime == owner.document->GetData()->Time() && beforeLabels == countLabels()
                && beforeUndo == owner.document->GetAvailableUndos() && beforeRedo == owner.document->GetAvailableRedos())} mutableCopy];
        if (mode == 30) {
            if (!captured || state != OcctAuthoredFrameReadState::Authored) Standard_Failure::Raise("Missing initial edit state.");
            auto capture = [&]() { OcctObjectTransformState value; if (!wrapper.CaptureObjectTransformStateForLabel(label, value)) Standard_Failure::Raise("Frame edit state capture failed."); return value; };
            owner.document->NewCommand(); assign(label, replacement);
            if (!owner.document->CommitCommand()) Standard_Failure::Raise("No frame replacement delta.");
            const auto changed = capture();
            result[@"replacementInvalidatesOriginal"] = @(!snapshot.IsEqual(changed));
            result[@"originalSnapshotUnchanged"] = @(snapshot.authoredFramesIdentity == record.identity);
            if (!owner.document->Undo()) Standard_Failure::Raise("Frame edit undo failed.");
            result[@"undoRestoresOriginal"] = @(snapshot.IsEqual(capture()));
            if (!owner.document->Redo()) Standard_Failure::Raise("Frame edit redo failed.");
            result[@"redoRestoresReplacement"] = @(changed.IsEqual(capture()));
            owner.document->NewCommand(); assign(label, archive); owner.document->AbortCommand();
            result[@"abortRestoresReplacement"] = @(changed.IsEqual(capture()));
            owner.document->NewCommand(); label.ForgetAttribute(AuthoredFrameAttributeID());
            if (!owner.document->CommitCommand()) Standard_Failure::Raise("No frame removal delta.");
            const auto absent = capture();
            result[@"removalInvalidatesReplacement"] = @(!absent.authoredFramesPresent && !changed.IsEqual(absent));
            if (!owner.document->Undo()) Standard_Failure::Raise("Frame removal undo failed.");
            result[@"removalUndoRestoresReplacement"] = @(changed.IsEqual(capture()));
        }
        return result;
    } catch (const Standard_Failure& failure) {
        return @{@"error": [NSString stringWithUTF8String:failure.GetMessageString()] ?: @"OCCT failure"};
    } catch (...) { return @{@"error": @"Frame-owner fixture failed"}; }
}

+ (NSDictionary<NSString *, id> *)debugGLBPrimitiveOwnership:(NSData *)data {
    if (![NSThread isMainThread] || data.length == 0 || data.length > 16384)
        return @{@"error": @"Private primitive probe requires1–16384bytes on the main thread"};
    try {
        const std::unique_ptr<FILE, int(*)(FILE*)> file(std::tmpfile(), &std::fclose);
        if (!file || std::fwrite(data.bytes, 1, data.length, file.get()) != data.length || std::fflush(file.get()) != 0)
            return @{@"error": @"Private pinned probe storage unavailable"};
        const int descriptor = ::fileno(file.get());
        const auto preflight = core3d::gltf::PreflightPinnedGLB(descriptor, data.length, nullptr);
        if (!preflight.IsValid()) return @{@"preflight": @(int(preflight.status)),
            @"message": [NSString stringWithUTF8String:preflight.message.c_str()]};
        struct Owner {
            Handle(TDocStd_Application) app = new TDocStd_Application();
            Handle(TDocStd_Document) document;
            ~Owner() noexcept { try { if (!document.IsNull()) app->Close(document); } catch (...) {} }
        } owner;
        Core3DDefineSafeBinXCAFFormat(owner.app);
        owner.app->NewDocument(TCollection_ExtendedString("BinXCAF"), owner.document);
        XCAFDoc_DocumentTool::SetLengthUnit(owner.document, 0.001);
        core3d::gltf::GLBDebugTrace trace;
        const auto result = core3d::gltf::ImportPinnedGLB(descriptor, data.length, preflight,
            owner.document, nullptr, Message_ProgressRange(), &trace);
        NSMutableArray* primitives = [NSMutableArray array];
        for (const auto& primitive : trace.primitives) {
            NSMutableArray* streams = [NSMutableArray array];
            for (const auto& stream : primitive.streams) {
                [streams addObject:@{@"type": @(stream.type), @"accessorID": @(stream.accessorID),
                    @"streamOffset": @(stream.streamOffset), @"streamLength": @(stream.streamLength),
                    @"accessorOffset": @(stream.accessorOffset), @"count": @(stream.count),
                    @"stride": @(stream.stride), @"pinned": @(stream.pinned)}];
            }
            [primitives addObject:@{@"meshID": [NSString stringWithUTF8String:primitive.meshID.c_str()], @"streams": streams}];
        }
        auto numbers = [](const auto& values) {
            NSMutableArray* result = [NSMutableArray arrayWithCapacity:values.size()];
            for (const auto& value : values) [result addObject:@(value)];
            return result;
        };
        NSMutableArray* occurrences = [NSMutableArray array];
        for (const auto& occurrence : trace.occurrences) {
            [occurrences addObject:@{@"primitive": @(occurrence.primitive),
                @"label": [NSString stringWithUTF8String:occurrence.label.c_str()],
                @"positions": numbers(occurrence.positions), @"normals": numbers(occurrence.normals),
                @"uvs": numbers(occurrence.uvs), @"indices": numbers(occurrence.indices)}];
        }
        NSMutableArray* pendingFrames=[NSMutableArray array];
        for (const auto& frame:result.authoredFrames) {
            TCollection_AsciiString entry;TDF_Tool::Entry(frame.label,entry);
            NSData* archive=[NSData dataWithBytes:frame.archive.data() length:frame.archive.size()];
            [pendingFrames addObject:@{@"label":[NSString stringWithUTF8String:entry.ToCString()],
                @"archiveBase64":[archive base64EncodedStringWithOptions:0]}];
        }
        std::vector<std::uint8_t> after(data.length);
        const auto read = ::pread(descriptor, after.data(), after.size(), 0);
        const bool unchanged = read == ssize_t(data.length) && std::memcmp(after.data(), data.bytes, data.length) == 0;
        return @{@"preflight": @(int(preflight.status)), @"status": @(int(result.status)),
            @"message": [NSString stringWithUTF8String:result.message.c_str()],
            @"sourceUnchanged": @(unchanged), @"objects": @(result.flattenedObjectCount),
            @"vertices": @(result.vertexCount), @"indices": @(result.indexCount),
            @"primitives": primitives, @"occurrences": occurrences,
            @"pendingFrameCount": @(pendingFrames.count), @"pendingFrames": pendingFrames,
            @"undos": @(owner.document->GetAvailableUndos()), @"redos": @(owner.document->GetAvailableRedos()),
            @"open": @(owner.document->HasOpenCommand())};
    } catch (const Standard_Failure& e) {
        return @{@"error": [NSString stringWithUTF8String:e.GetMessageString() ?: "Private primitive probe failure"]};
    } catch (...) { return @{@"error": @"Private primitive ownership probe failed"}; }
}

+ (NSDictionary<NSString *, NSNumber *> *)debugMeshElementSelectionPolicy {
    try {
        NSMutableDictionary<NSString *, NSNumber *> *result=[NSMutableDictionary dictionary];
        for (const auto& check:core3d::meshedit::DebugElementSelectionPolicy()) {
            result[[NSString stringWithUTF8String:check.first.c_str()]]=@(check.second);
        }
        return [result copy];
    } catch (...) { return @{}; }
}

+ (NSDictionary<NSString *, id> *)debugCurrentTessellationMeshCopy:(NSInteger)mode {
    if (![NSThread isMainThread]) return @{@"error":@"Main thread required"};
    using namespace core3d::meshcopy;
    try {
        TopoDS_Shape source = BRepPrimAPI_MakeBox(20,30,40).Shape();
        if (mode == 1) {
            source = BRepAlgoAPI_Cut(
                BRepPrimAPI_MakeBox(gp_Pnt(-60,-40,0),120,80,40).Shape(),
                BRepPrimAPI_MakeBox(gp_Pnt(-58,-38,2),116,76,38).Shape()).Shape();
        }
        // Analytic provenance fixtures: curved solids with exact authored
        // parameters, located/reflected exactly once below.
        if (mode == 19 || mode == 20) source = BRepPrimAPI_MakeCylinder(17.5,44).Shape();
        else if (mode == 21 || mode == 24)
            source = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0,0,0),gp_Dir(1,0,0)),17.5,44).Shape();
        else if (mode == 22) source = BRepPrimAPI_MakeTorus(41.5,11.25).Shape();
        else if (mode == 23)
            source = BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0,0,0),gp_Dir(0,0,1)),17.5,44,3.9269908169872414).Shape();
        if (source.IsNull()) return @{@"error":@"Fixture construction failed"};
        // Mode 22 torus (R=41.5,r=11.25) at 0.1mm yields ~9600 triangles, over the 4096 copy cap; 0.8mm keeps it near 1000.
        const double fixtureDeflection = (mode == 22) ? 0.8 : 0.1;
        BRepMesh_IncrementalMesh mesher(source,fixtureDeflection,false,0.5,false);
        if (!mesher.IsDone()) return @{@"error":@"Fixture tessellation failed"};
        if (mode == 2 || mode == 20) {
            gp_Trsf tr; tr.SetTranslation(gp_Vec(71,-19,23));
            source.Location(TopLoc_Location(tr));
        } else if (mode == 3 || mode == 21) {
            gp_Trsf tr; tr.SetMirror(gp_Ax2(gp_Pnt(0,0,0),gp_Dir(1,0,0)));
            // Adversarial private fixture: default TopoDS setter forbids reflection.
            source.Location(TopLoc_Location(tr),Standard_False);
        } else if (mode == 4) source.Reverse();
        const auto firstFace = TopoDS::Face(TopExp_Explorer(source,TopAbs_FACE).Current());
        TopLoc_Location firstLocation;
        auto firstMesh = BRep_Tool::Triangulation(firstFace,firstLocation);
        if (firstMesh.IsNull()) return @{@"error":@"Fixture has no first mesh"};
        if (mode == 5) BRepTools::Clean(source);
        else if (mode == 6) firstMesh->SetNode(1,gp_Pnt(std::numeric_limits<double>::quiet_NaN(),0,0));
        else if (mode == 7) firstMesh->SetTriangle(1,Poly_Triangle(1,1,1));
        else if (mode == 8) {
            Handle(Poly_Triangulation) large = new Poly_Triangulation(3,4097,false);
            large->SetNode(1,gp_Pnt(0,0,0)); large->SetNode(2,gp_Pnt(1,0,0)); large->SetNode(3,gp_Pnt(0,1,0));
            for (int t=1;t<=4097;++t) large->SetTriangle(t,Poly_Triangle(1,2,3));
            BRep_Builder().UpdateFace(firstFace,large);
        } else if (mode == 9) {
            Handle(Poly_Triangulation) invalid = new Poly_Triangulation(4,1,false);
            invalid->SetNode(1,gp_Pnt(0,0,0)); invalid->SetNode(2,gp_Pnt(1,0,0)); invalid->SetNode(3,gp_Pnt(0,1,0));
            invalid->SetNode(4,gp_Pnt(0,std::numeric_limits<double>::infinity(),0));
            invalid->SetTriangle(1,Poly_Triangle(1,2,3)); BRep_Builder().UpdateFace(firstFace,invalid);
        }
        else if (mode == 11) {
            Handle(Poly_Triangulation) large = new Poly_Triangulation(12289,1,false);
            for (int n=1;n<=12289;++n) large->SetNode(n,gp_Pnt(n%2,n%3,0));
            large->SetTriangle(1,Poly_Triangle(1,2,3)); BRep_Builder().UpdateFace(firstFace,large);
        } else if (mode == 12) firstMesh->SetTriangle(1,Poly_Triangle(0,2,3));
        else if (mode == 13) firstMesh->SetNode(1,gp_Pnt(1000001,0,0));
        if (mode>=14 && mode<=18) {
            const int depth=mode==17?9:1;
            for (int i=0;i<depth;++i) {
                TopoDS_Compound wrapper;BRep_Builder builder;builder.MakeCompound(wrapper);
                if (mode!=18) builder.Add(wrapper,source);
                if (mode==15) {
                    gp_Trsf tr;tr.SetTranslation(gp_Vec(100,0,0));
                    builder.Add(wrapper,source.Located(TopLoc_Location(tr)));
                }
                if (mode==16) builder.Add(wrapper,firstFace);
                source=wrapper;
            }
        }
        // Capture actual source handles and exact stored node/triangle bytes,
        // including NaN payloads in rejected inputs, without serializing copies.
        struct MeshProof {
            TopoDS_Face face; Handle(Poly_Triangulation) mesh; TopLoc_Location location;
            std::vector<std::array<double,3>> nodes;
            std::vector<std::array<int,3>> triangles;
        };
        std::vector<MeshProof> proof;
        for (TopExp_Explorer it(source,TopAbs_FACE);it.More();it.Next()) {
            MeshProof p; p.face=TopoDS::Face(it.Current()); p.mesh=BRep_Tool::Triangulation(p.face,p.location);
            if (!p.mesh.IsNull()) {
                for (int n=1;n<=p.mesh->NbNodes();++n) {
                    const auto point=p.mesh->Node(n);p.nodes.push_back({point.X(),point.Y(),point.Z()});
                }
                for (int t=1;t<=p.mesh->NbTriangles();++t) {
                    std::array<int,3> ids; p.mesh->Triangle(t).Get(ids[0],ids[1],ids[2]); p.triangles.push_back(ids);
                }
            }
            proof.push_back(std::move(p));
        }
        std::atomic_bool cancelled(mode==10);
        CurrentTessellationCopy copy;
        copy.face=firstFace;copy.triangles=99; // Reject paths must clear a prior output too.
        const auto result=PrepareCurrentTessellationCopy(source,copy,cancelled);
        bool unchanged=true,independent=true;
        for (const auto& p:proof) {
            TopLoc_Location location; const auto mesh=BRep_Tool::Triangulation(p.face,location);
            unchanged &= mesh==p.mesh && location.IsEqual(p.location);
            if (mesh.IsNull()) continue;
            unchanged &= mesh->NbNodes()==p.nodes.size() && mesh->NbTriangles()==p.triangles.size();
            for (int n=1;n<=mesh->NbNodes();++n) {
                const auto point=mesh->Node(n);const std::array<double,3> now={point.X(),point.Y(),point.Z()};
                unchanged &= std::memcmp(now.data(),p.nodes[n-1].data(),sizeof(double)*3)==0;
            }
            for (int t=1;t<=mesh->NbTriangles();++t) {
                std::array<int,3> ids;mesh->Triangle(t).Get(ids[0],ids[1],ids[2]); unchanged &= ids==p.triangles[t-1];
            }
        }
        NSMutableDictionary* report=[@{@"result":@(int(result)),@"sourceUnchanged":@(unchanged),
            @"sourceKind":@(int(source.ShapeType())),@"outputEmpty":@(copy.face.IsNull()),@"sourceFaces":@(copy.sourceFaces),@"triangles":@(copy.triangles)} mutableCopy];
        // Per-source-face analytic provenance in emission order; empty on
        // every rejection because failure paths clear the output.
        NSMutableArray* faceRecords=[NSMutableArray arrayWithCapacity:copy.faces.size()];
        for (const auto& record:copy.faces) {
            NSMutableDictionary* entry=[@{@"firstTriangle":@(record.firstTriangle),
                @"triangleCount":@(record.triangleCount),@"reversed":@(record.reversed)} mutableCopy];
            auto point=[](const gp_Pnt& p)->NSArray* {return @[@(p.X()),@(p.Y()),@(p.Z())];};
            auto direction=[](const gp_Dir& d)->NSArray* {return @[@(d.X()),@(d.Y()),@(d.Z())];};
            switch (record.surfaceType) {
                case SourceFaceRecord::SurfaceType::Plane:
                    entry[@"type"]=@"plane";
                    if (const auto* plane=std::get_if<gp_Pln>(&record.params)) {
                        entry[@"location"]=point(plane->Location());
                        entry[@"direction"]=direction(plane->Axis().Direction());
                    }
                    break;
                case SourceFaceRecord::SurfaceType::Cylinder:
                    entry[@"type"]=@"cylinder";
                    if (const auto* cylinder=std::get_if<gp_Cylinder>(&record.params)) {
                        entry[@"location"]=point(cylinder->Axis().Location());
                        entry[@"direction"]=direction(cylinder->Axis().Direction());
                        entry[@"radius"]=@(cylinder->Radius());
                    }
                    break;
                case SourceFaceRecord::SurfaceType::Torus:
                    entry[@"type"]=@"torus";
                    if (const auto* torus=std::get_if<gp_Torus>(&record.params)) {
                        entry[@"location"]=point(torus->Axis().Location());
                        entry[@"direction"]=direction(torus->Axis().Direction());
                        entry[@"majorRadius"]=@(torus->MajorRadius());
                        entry[@"minorRadius"]=@(torus->MinorRadius());
                    }
                    break;
                case SourceFaceRecord::SurfaceType::Other: entry[@"type"]=@"other"; break;
            }
            [faceRecords addObject:entry];
        }
        report[@"faces"]=faceRecords;
        if (result!=PreparationResult::Ready) return report;
        TopLoc_Location location; const auto mesh=BRep_Tool::Triangulation(copy.face,location);
        if (mesh.IsNull()) return @{@"error":@"Ready output has no triangulation"};
        for (const auto& p:proof) independent &= p.mesh!=mesh;
        std::array<double,6> bounds={INFINITY,INFINITY,INFINITY,-INFINITY,-INFINITY,-INFINITY};
        double volume=0;bool normals=true,corners=true;
        for (int t=1;t<=mesh->NbTriangles();++t) {
            int a,b,c;mesh->Triangle(t).Get(a,b,c);corners &= a==3*t-2 && b==3*t-1 && c==3*t;
            const auto p=mesh->Node(a),q=mesh->Node(b),r=mesh->Node(c);
            volume+=gp_Vec(p.XYZ()).Dot(gp_Vec(q.XYZ()).Crossed(gp_Vec(r.XYZ())))/6;
            const gp_Dir normal(gp_Vec(p,q).Crossed(gp_Vec(p,r)));
            for (int n:{a,b,c}) {
                const auto point=mesh->Node(n);
                for(int axis=0;axis<3;++axis) {
                    bounds[axis]=std::min(bounds[axis],point.Coord(axis+1));
                    bounds[axis+3]=std::max(bounds[axis+3],point.Coord(axis+1));
                }
                normals &= mesh->HasNormals() && mesh->Normal(n).Dot(normal)>1-1e-6;
            }
        }
        NSMutableArray* boundArray=[NSMutableArray array];for (double x:bounds) [boundArray addObject:@(x)];
        report[@"bounds"]=boundArray;report[@"signedVolume"]=@(volume);report[@"independentMesh"]=@(independent);
        report[@"flatNormalsAgree"]=@(normals);report[@"independentCorners"]=@(corners);
        report[@"nodes"]=@(mesh->NbNodes());report[@"hasUV"]=@(mesh->HasUVNodes());
        report[@"meshOnly"]=@(BRep_Tool::Surface(copy.face).IsNull());
        return report;
    } catch (const Standard_Failure& e) {
        return @{@"error":[NSString stringWithUTF8String:e.GetMessageString() ?: "Mesh copy probe failed"]};
    } catch (...) {return @{@"error":@"Mesh copy probe failed"};}
}

+ (NSDictionary<NSString *, id> *)debugCopySourceFaceProvenance:(NSData *)data
                                               entityIdentifier:(NSString *)identifier
                                                           mode:(NSInteger)mode {
    if (![NSThread isMainThread] || data.length == 0 || data.length > 64U*1024U*1024U
        || identifier.length == 0 || identifier.length > 256 || mode < 0 || mode > 1)
        return @{@"error":@"Private copy provenance probe requires 1byte–64MiB of document data, an entity identifier and mode 0/1 on the main thread"};
    using core3d::provenance::CopySourceFaceProvenanceReadState;
    try {
        // The narrow private-snapshot reader resolves the format from the
        // .xbf extension, so the bytes land in a uniquely named temporary
        // handoff that only this probe removes (same pattern as thumbData).
        NSString* directory = NSTemporaryDirectory();
        if (directory.length == 0) return @{@"error":@"Private copy provenance probe storage unavailable"};
        NSURL* fileURL = [NSURL fileURLWithPath:[directory stringByAppendingPathComponent:
            [NSString stringWithFormat:@"copy-provenance-%@.xbf", NSUUID.UUID.UUIDString]]];
        if (![data writeToURL:fileURL options:NSDataWritingAtomic error:nil])
            return @{@"error":@"Private copy provenance probe storage unavailable"};
        struct Owner {
            Handle(OcctDocument) document = new OcctDocument();
            ~Owner() noexcept { try { document->ClosePrivateExportSnapshot(); } catch (...) {} }
        } owner;
        const bool opened = owner.document->OpenPrivateExportSnapshot(
            fileURL.fileSystemRepresentation, Message_ProgressRange());
        // The document is fully resident after Open; only this probe's unique
        // handoff file is removed (same pattern as thumbData).
        [[NSFileManager defaultManager] removeItemAtURL:fileURL error:nil];
        if (!opened)
            return @{@"error":@"Private copy provenance probe document rejected"};
        TDF_Label target;
        TDF_LabelSequence labels;
        XCAFDoc_DocumentTool::ShapeTool(owner.document->Document()->Main())->GetFreeShapes(labels);
        for (int i = 1; i <= labels.Length(); ++i)
            if (owner.document->EntityIdentifierForLabel(labels.Value(i)) == identifier.UTF8String)
                target = labels.Value(i);
        if (target.IsNull()) return @{@"error":@"Copy provenance entity not found"};
        auto report = ^NSDictionary<NSString *, id> *(const core3d::provenance::SourceFaceProvenanceRecord& record,
                                                      CopySourceFaceProvenanceReadState state) {
            NSMutableDictionary* result = [@{@"state":@(int(state))} mutableCopy];
            if (state == CopySourceFaceProvenanceReadState::Present
                || state == CopySourceFaceProvenanceReadState::Stale) {
                result[@"digest"] = [NSString stringWithUTF8String:record.digest.c_str()];
                result[@"triangleCount"] = @(record.triangleCount);
                NSMutableArray* faces = [NSMutableArray arrayWithCapacity:record.faces.size()];
                for (const auto& entry : record.faces) {
                    NSMutableArray* params = [NSMutableArray arrayWithCapacity:entry.params.size()];
                    for (double value : entry.params) [params addObject:@(value)];
                    [faces addObject:@{@"surfaceType":@(entry.surfaceType),
                        @"firstTriangle":@(entry.firstTriangle), @"triangleCount":@(entry.triangleCount),
                        @"reversed":@(entry.reversed), @"params":params}];
                }
                result[@"faces"] = faces;
            }
            return result;
        };
        core3d::provenance::SourceFaceProvenanceRecord record;
        const auto state = owner.document->TryCopySourceFaceProvenanceForLabel(target, record);
        NSMutableDictionary* result = [report(record, state) mutableCopy];
        if (mode == 1) {
            // The helper observes the malformed digest and restores the
            // original attribute directly on this private snapshot.
            Standard_Integer corruptedState = -1;
            const auto code = owner.document->DebugCorruptCopySourceFaceProvenance(target, corruptedState);
            core3d::provenance::SourceFaceProvenanceRecord restored;
            const auto restoredState = owner.document->TryCopySourceFaceProvenanceForLabel(target, restored);
            result[@"corruptionAccepted"] = @(code == 0);
            result[@"corruptionCode"] = @(code);
            result[@"corruptedState"] = @(int(corruptedState));
            result[@"restored"] = report(restored, restoredState);
        }
        return result;
    } catch (const Standard_Failure& e) {
        return @{@"error":[NSString stringWithUTF8String:e.GetMessageString() ?: "Copy provenance probe failure"]};
    } catch (...) { return @{@"error":@"Copy provenance probe failed"}; }
}

+ (NSDictionary<NSString *, NSNumber *> *)debugNativeIntentPolicy:(NSInteger)family {
    if (![NSThread isMainThread] || family < 0 || family > 3) return @{};
    try {
        NSMutableDictionary<NSString *, NSNumber *> *report = [NSMutableDictionary dictionary];
        auto retain = [&](const auto& outcomes) {
            for (std::size_t index = 0; index < outcomes.size(); ++index) {
                report[[NSString stringWithFormat:@"case%zu", index]] = @(outcomes[index]);
            }
        };
        if (family == 0) retain(core3d::debug::RunNativeManualIntentPolicyProbe());
        else if (family == 1) retain(core3d::debug::RunNativeReplacementPolicyProbe());
        else if (family == 2) retain(core3d::debug::RunNativeIntentReplacementPromotionProbe());
        else retain(core3d::debug::RunNativeQueuedLoadPolicyProbe());
        return report;
    } catch (...) { return @{}; }
}

+ (NSDictionary<NSString *, id> *)debugLiveTransactionProbeLifecycle {
    if (![NSThread isMainThread]) return @{@"error":@"Main thread required"};
    using namespace core3d::debug;
    try {
        struct Owner {
            Handle(OcctDocument) document = new OcctDocument();
            ~Owner() noexcept {
                try {
                    document->DebugStopLiveTransactionProbe();
                    const auto native = document->Document();
                    if (!native.IsNull()) {
                        if (native->HasOpenCommand()) native->AbortCommand();
                        const auto app = Handle(TDocStd_Application)::DownCast(native->Application());
                        if (!app.IsNull()) app->Close(native);
                    }
                } catch (...) {}
            }
        } owner;
        const auto document = owner.document;
        const bool unattachedRejected = !document->DebugStartLiveTransactionProbe();
        document->InitDoc();
        if (!document->DebugStartLiveTransactionProbe()) return @{@"error":@"Attach failed"};
        auto state = document->DebugLiveTransactionProbe();
        const auto oldNative = document->Document();
        oldNative->NewCommand();
        TDataStd_Integer::Set(oldNative->Main().FindChild(91), 42);
        document->InitDoc();
        NSMutableArray* lifecycle = [NSMutableArray array];
        for (std::size_t i = 0; i < state->count; ++i) {
            const auto& e = state->events[i];
            [lifecycle addObject:@{@"kind":@(unsigned(e.kind)), @"opening":@(e.opening),
                @"active":@(e.active), @"open":@(e.commandOpen), @"undos":@(e.undos), @"redos":@(e.redos)}];
        }
        const bool resetValid = document->DebugLiveTransactionProbeValid()
            && state->opening == 2 && state->active == document->Document().get()
            && document->Document() != oldNative && !document->Document()->HasOpenCommand();
        std::weak_ptr<const LiveTransactionProbeState> released = state;
        document->DebugStopLiveTransactionProbe(); state.reset();
        const bool stateReleased = released.expired();
        if (!document->DebugStartLiveTransactionProbe()) return @{@"error":@"Reattach failed"};
        const auto native = document->Document();
        for (int i = 0; i < 130; ++i) { native->NewCommand(); native->AbortCommand(); }
        state = document->DebugLiveTransactionProbe();
        const auto boundedCount = state->count;
        const bool overflowRejected = !document->DebugLiveTransactionProbeValid()
            && state->count == state->events.size() && !native->HasOpenCommand();
        document->DebugStopLiveTransactionProbe(); state.reset();
        const bool invalidDetached = !document->DebugLiveTransactionProbe();
        if (!document->DebugStartLiveTransactionProbe()) return @{@"error":@"Attach after overflow failed"};
        state = document->DebugLiveTransactionProbe();
        const auto countBeforeForeign = state->count;
        const auto app = Handle(TDocStd_Application)::DownCast(native->Application());
        // Deliberately invoke only the diagnostic hook. No OCAF document is
        // passed to, read by, or mutated on this foreign thread.
        std::thread foreign([app] { app->OnOpenTransaction(Handle(TDocStd_Document)()); });
        foreign.join();
        const bool foreignRejected = !document->DebugLiveTransactionProbeValid()
            && state->count == countBeforeForeign && !native->HasOpenCommand();
        document->DebugStopLiveTransactionProbe(); state.reset();
        const bool foreignDetached = !document->DebugLiveTransactionProbe();
        return @{@"uninitializedAttachRejected":@(unattachedRejected), @"resetValid":@(resetValid),
            @"lifecycle":lifecycle, @"stateReleased":@(stateReleased), @"boundedCount":@(boundedCount),
            @"overflowRejected":@(overflowRejected), @"invalidDetached":@(invalidDetached),
            @"foreignRejected":@(foreignRejected), @"foreignDetached":@(foreignDetached)};
    } catch (const Standard_Failure& e) {
        return @{@"error":[NSString stringWithUTF8String:e.GetMessageString() ?: "Lifecycle failure"]};
    } catch (...) { return @{@"error":@"Live document lifecycle probe failed"}; }
}

+ (NSDictionary<NSString *, id> *)debugTransactionObserver {
    if (![NSThread isMainThread]) return @{@"error": @"Main thread required"};
    using namespace core3d::debug;
    try {
        struct Owner {
            std::shared_ptr<TransactionProbeState> state = std::make_shared<TransactionProbeState>();
            Handle(TDocStd_Application) app = new ObservedApplication(state);
            std::vector<Handle(TDocStd_Document)> documents;
            ~Owner() noexcept {
                state->adopted = nullptr;
                for (const auto& doc : documents) {
                    try { if (doc->HasOpenCommand()) doc->AbortCommand(); app->Close(doc); } catch (...) {}
                }
            }
            Handle(TDocStd_Document) make() {
                Handle(TDocStd_Document) doc;
                app->NewDocument(TCollection_ExtendedString("BinXCAF"), doc);
                if (doc.IsNull()) Standard_Failure::Raise("Observer fixture document missing");
                documents.push_back(doc);
                doc->SetUndoLimit(20);
                TDataStd_Integer::Set(doc->Main().FindChild(91), 0);
                doc->ClearUndos();
                return doc;
            }
        } owner;
        Core3DDefineSafeBinXCAFFormat(owner.app);
        const auto live = owner.make();
        const auto candidate = owner.make();
        owner.state->adopted = live.get();
        NSMutableArray* rows = [NSMutableArray array];
        std::size_t cursor = 0;
        auto row = [&](NSString* name, const Handle(TDocStd_Document)& doc) {
            NSMutableArray* events = [NSMutableArray array];
            for (; cursor < owner.state->count; ++cursor) {
                const auto& e = owner.state->events[cursor];
                [events addObject:@{@"kind": @(static_cast<int>(e.kind)), @"adopted": @(e.adopted),
                    @"open": @(e.commandOpen), @"undos": @(e.undos), @"redos": @(e.redos)}];
            }
            Handle(TDataStd_Integer) value;
            if (!doc->Main().FindChild(91, Standard_False).FindAttribute(TDataStd_Integer::GetID(), value))
                Standard_Failure::Raise("Observer fixture integer lost");
            [rows addObject:@{@"step": name, @"events": events, @"value": @(value->Get()),
                @"adopted": @(doc.get() == owner.state->adopted), @"open": @(doc->HasOpenCommand()),
                @"undos": @(doc->GetAvailableUndos()), @"redos": @(doc->GetAvailableRedos()),
                @"observerValid": @(owner.state->valid)}];
        };
        auto write = [&](const Handle(TDocStd_Document)& doc, int value) {
            TDataStd_Integer::Set(doc->Main().FindChild(91, Standard_False), value);
        };
        // Ignore setup callbacks: no adopted document existed during construction.
        cursor = owner.state->count;
        row(@"initial", live);
        live->NewCommand(); live->CommitCommand(); row(@"empty", live);
        live->NewCommand(); write(live, 1); live->CommitCommand(); row(@"commit", live);
        // Deliberate private presentation seam: the already committed authority cannot rewind.
        bool presentationFailed = false;
        try { Standard_Failure::Raise("Deliberate after-commit presentation failure"); }
        catch (const Standard_Failure&) { presentationFailed = true; }
        row(@"afterPresentationFailure", live);
        live->NewCommand(); write(live, 2); live->AbortCommand(); row(@"abort", live);
        live->Undo(); row(@"undo", live);
        live->Redo(); row(@"redo", live);
        candidate->NewCommand(); write(candidate, 8); candidate->CommitCommand(); row(@"candidateCommit", candidate);
        row(@"liveAfterCandidate", live);
        const auto rejected = owner.make();
        rejected->NewCommand(); write(rejected, 7); rejected->CommitCommand(); row(@"rejectedCandidate", rejected);
        owner.app->Close(rejected);
        owner.documents.erase(std::remove(owner.documents.begin(), owner.documents.end(), rejected), owner.documents.end());
        row(@"afterRejectedClose", live);
        // Explicit adoption identity is independent of commit observation.
        owner.state->adopted = candidate.get(); row(@"adoptCandidate", candidate);
        candidate->NewCommand(); write(candidate, 9); candidate->CommitCommand(); row(@"adoptedCommit", candidate);
        owner.state->adopted = live.get();
        live->NewCommand(); write(live, 3); live->NewCommand(); row(@"implicitNewCommand", live);
        live->AbortCommand(); row(@"abortImplicitEmpty", live);
        live->NewCommand(); write(live, 4); live->SetUndoLimit(10); row(@"implicitUndoLimit", live);
        if (live->HasOpenCommand()) live->AbortCommand();
        live->SetUndoLimit(0); row(@"undoDisabled", live);
        live->NewCommand(); write(live, 5); live->CommitCommand(); row(@"disabledCommit", live);
        write(live, 6); row(@"directWrite", live);
        const auto countBeforePrivate = owner.state->count;
        {
            Owner isolated;
            Core3DDefineSafeBinXCAFFormat(isolated.app);
            const auto doc = isolated.make();
            doc->NewCommand(); write(doc, 99); doc->CommitCommand();
        }
        row(@"privateApplication", live);
        const bool privateIgnored = countBeforePrivate == owner.state->count;
        // Losing the observer cannot leave a retained native ownership cycle.
        auto expiring = std::make_shared<TransactionProbeState>();
        std::weak_ptr<TransactionProbeState> expired = expiring;
        Handle(TDocStd_Application) expiredApp = new ObservedApplication(expiring);
        expiring.reset();
        expiredApp->OnCommitTransaction(live); // Explicit lifetime seam, not a native commit.
        // Capacity seam uses virtual dispatch only; never label it a kernel mutation.
        auto bounded = std::make_shared<TransactionProbeState>();
        Handle(TDocStd_Application) boundedApp = new ObservedApplication(bounded);
        for (int i = 0; i < 129; ++i) boundedApp->OnOpenTransaction(live);
        return @{@"rows": rows, @"presentationFailureObserved": @(presentationFailed), @"privateApplicationIgnored": @(privateIgnored),
            @"expiredStateReleased": @(expired.expired()), @"boundedCount": @(bounded->count),
            @"overflowRejected": @(!bounded->valid), @"observerValid": @(owner.state->valid)};
    } catch (const Standard_Failure& e) {
        return @{@"error": [NSString stringWithUTF8String:e.GetMessageString() ?: "OCCT observer failure"]};
    } catch (...) { return @{@"error": @"Observer fixture failed"}; }
}

+ (NSDictionary<NSString *, id> *)debugFrameUVEdit:(NSData *)archive mode:(NSInteger)mode {
    if (mode < 0 || mode > 4 || ![NSThread isMainThread]) return @{@"error": @"Undefined frame UV fixture"};
    try {
        using namespace core3d;
        using namespace core3d::persistence;
        struct PrivateDocument {
            Handle(TDocStd_Application) app = new TDocStd_Application();
            Handle(TDocStd_Document) document;
            ~PrivateDocument() noexcept { try { if (!document.IsNull()) app->Close(document); } catch (...) {} }
        } owner;
        Core3DDebugDefineFrameBinXCAFFormat(owner.app, std::make_shared<AuthoredFrameReadBudget>());
        owner.app->NewDocument(TCollection_ExtendedString("BinXCAF"), owner.document);
        XCAFDoc_DocumentTool::SetLengthUnit(owner.document, 0.001);
        Handle(OcctDocument) wrapper = new OcctDocument(); wrapper->ChangeDocument() = owner.document;
        const auto shape = Core3DDebugAuthoredGeometryFixture(0);
        const auto label = XCAFDoc_DocumentTool::ShapeTool(owner.document->Main())->AddShape(shape, Standard_False);
        owner.document->SetUndoLimit(20); owner.document->NewCommand();
        Handle(AIS_Shape) presentation = new AIS_Shape(shape);
        if (!wrapper->SetGeometryRepresentationForLabel(label, OcctGeometryRepresentation::TriangleMesh))
            Standard_Failure::Raise("Frame UV fixture representation failed.");
        owner.document->CommitCommand(); owner.document->ClearUndos();
        if (!wrapper->MigrateLegacyIdentifiers()) Standard_Failure::Raise("Frame UV fixture migration failed.");
        // SaveObjectTransform seals an immutable state and requires IDs first.
        owner.document->NewCommand();
        if (!wrapper->SaveObjectTransform(label, presentation)) Standard_Failure::Raise("Frame UV fixture transform failed.");
        owner.document->CommitCommand(); owner.document->ClearUndos();
        if (archive.length < 128 || archive.length > core3d::scene::authored::kMaximumArchiveBytes) Standard_Failure::Raise("Invalid UV frame archive length.");
        const auto bytes = TDataStd_ByteArray::Set(label, AuthoredFrameAttributeID(), 0, Standard_Integer(archive.length)-1, Standard_False);
        const auto* data = static_cast<const std::uint8_t*>(archive.bytes);
        for (NSUInteger i=0; i<archive.length; ++i) bytes->SetValue(Standard_Integer(i), data[i]);
        auto capture = [&]() { OcctObjectTransformState state; if (!wrapper->CaptureObjectTransformStateForLabel(label,state)) Standard_Failure::Raise("Frame UV capture failed."); return state; };
        const auto original = capture();
        if (mode == 3) bytes->SetValue(bytes->Upper(), bytes->Value(bytes->Upper()) ^ 1);
        const int beforeTime = owner.document->GetData()->Time();
        TopoDS_Shape candidate; const OcctMeshUVAtlasOptions options{2,2048,8};
        const bool prepared = wrapper->PrepareTriangleUVAtlas(label,candidate,options);
        NSMutableDictionary* result = [@{@"prepared": @(prepared), @"prepareReadOnly": @(beforeTime == owner.document->GetData()->Time())} mutableCopy];
        if (mode == 3) {
            result[@"corruptPreserved"] = @(!prepared && bytes->Value(bytes->Upper()) == (data[archive.length-1] ^ 1)
                && owner.document->GetAvailableUndos() == 0 && !owner.document->HasOpenCommand());
            return result;
        }
        if (!prepared) Standard_Failure::Raise("Frame UV preparation failed.");
        struct Host final : OrdinaryEditPresentationHost {
            bool admitTransform(OrdinaryTransformLedger&) noexcept override { return true; }
            bool repairTransform(const OrdinaryTransformLedger& ledger, bool committed) noexcept override {
                try { for (const auto& record : ledger.records) {
                    const auto& expected = committed ? record.candidate : record.previous;
                    record.requested.presentation->SetShape(expected.shape);
                    record.requested.presentation->SetLocalTransformation(expected.transform);
                } return true; } catch (...) { return false; }
            }
        } host;
        const auto controller = std::make_shared<OrdinaryEditController>(wrapper,host);
        OrdinaryTransformChange change; change.label=label; change.presentation=presentation;
        change.shape=candidate; change.transform=original.transform; change.operation=OrdinaryTransformOperation::MeshUVAtlas;
        change.meshUVAtlasOptions=options;
        if (mode == 4) { change.shape=original.shape; change.operation=OrdinaryTransformOperation::Translate; change.transform.SetTranslationPart(gp_Vec(12,3,4)); }
        if (mode == 2) controller->debugCommandStamp().debugSetCommitMode(1);
        OrdinaryEditResult failure = OrdinaryEditResult::Invalid;
        auto lease = controller->beginTransform({change},&failure);
        if (!lease) Standard_Failure::Raise("Frame UV ordinary admission failed.");
        const auto outcome = mode == 1 ? lease.cancel() : lease.stageAndCommit();
        const auto after = capture();
        result[@"outcome"] = @(int(outcome)); result[@"closed"] = @(!owner.document->HasOpenCommand());
        result[@"idle"] = @(controller->state() == OrdinaryEditState::Idle);
        result[@"undoCount"] = @(owner.document->GetAvailableUndos());
        if (mode == 1 || mode == 2) {
            result[@"exactOriginalRestored"] = @(original.IsEqual(after));
        } else {
            result[@"idsPreserved"] = @(after.entityIdentifier == original.entityIdentifier && after.definitionIdentifier == original.definitionIdentifier);
            result[@"framesPresent"] = @(after.authoredFramesPresent);
            result[@"frameIdentityPreserved"] = @(after.authoredFramesIdentity == original.authoredFramesIdentity);
            result[@"shapeChanged"] = @(!after.shape.IsEqual(original.shape));
            result[@"atlasVersion"] = @(after.meshUVAtlasVersion);
            result[@"atlasSettings"] = @[@(after.meshUVAtlasSettings[0]),@(after.meshUVAtlasSettings[1]),@(after.meshUVAtlasSettings[2])];
            if (!owner.document->Undo()) Standard_Failure::Raise("Frame UV undo failed.");
            result[@"undoRestoresOriginal"] = @(original.IsEqual(capture()));
            if (!owner.document->Redo()) Standard_Failure::Raise("Frame UV redo failed.");
            result[@"redoRestoresCandidate"] = @(after.IsEqual(capture()));
        }
        Standard_Size resident=0; result[@"ownersValid"] = @(Core3DValidateAuthoredFrameOwners(owner.document,resident));
        return result;
    } catch (const Standard_Failure& failure) { return @{@"error": [NSString stringWithUTF8String:failure.GetMessageString()] ?: @"OCCT failure"}; }
      catch (...) { return @{@"error": @"Native frame UV fixture failed"}; }
}

+ (NSDictionary<NSString *, id> *)debugFrameCopy:(NSData *)archive replacement:(NSData *)replacement mode:(NSInteger)mode {
    if (mode < 0 || mode > 9 || ![NSThread isMainThread]) return @{@"error": @"Undefined frame-copy fixture"};
    try {
        using namespace core3d::persistence;
        struct PrivateDocument {
            Handle(TDocStd_Application) app = new TDocStd_Application();
            Handle(TDocStd_Document) document;
            ~PrivateDocument() noexcept { try { if (!document.IsNull()) app->Close(document); } catch (...) {} }
        } owner, reader;
        Core3DDebugDefineFrameBinXCAFFormat(owner.app, std::make_shared<AuthoredFrameReadBudget>());
        Core3DDebugDefineFrameBinXCAFFormat(reader.app, std::make_shared<AuthoredFrameReadBudget>());
        owner.app->NewDocument(TCollection_ExtendedString("BinXCAF"), owner.document);
        XCAFDoc_DocumentTool::SetLengthUnit(owner.document, 0.001);
        OcctDocument wrapper; wrapper.ChangeDocument() = owner.document;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(owner.document->Main());
        auto makeShape = [&](int fixtureMode) {
            TopoDS_Shape shape = Core3DDebugAuthoredGeometryFixture(fixtureMode);
            if (mode == 2) { BRep_Builder builder; TopoDS_Compound compound; builder.MakeCompound(compound); builder.Add(compound, shape); shape = compound; }
            return shape;
        };
        auto a = makeShape(mode == 9 ? 2 : mode == 4 ? 22 : 0);
        auto b = makeShape(mode == 4 ? 22 : mode == 5 ? 2 : mode == 6 ? 6 : mode == 7 ? 4 : mode == 8 ? 3 : 0);
        const auto source = shapes->AddShape(a, Standard_False), destination = shapes->AddShape(b, Standard_False);
        if (source.IsNull() || destination.IsNull() || source.IsEqual(destination)) Standard_Failure::Raise("Missing copy definitions.");
        if (mode == 1 || mode == 2) {
            gp_Trsf placed; placed.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0,0,1)), M_PI / 2);
            placed.SetTranslationPart(gp_Vec(2,3,4)); b.Location(TopLoc_Location(placed)); shapes->SetShape(destination, b);
        }
        auto assign = [](const TDF_Label& label, NSData* data) {
            if (data.length < 128 || data.length > core3d::scene::authored::kMaximumArchiveBytes) Standard_Failure::Raise("Invalid fixture archive size.");
            const auto bytes = TDataStd_ByteArray::Set(label, AuthoredFrameAttributeID(), 0, Standard_Integer(data.length)-1, Standard_False);
            const auto* input = static_cast<const std::uint8_t*>(data.bytes);
            for (NSUInteger i=0; i<data.length; ++i) bytes->SetValue(Standard_Integer(i), input[i]);
        };
        owner.document->SetUndoLimit(20); owner.document->NewCommand();
        if (!wrapper.SetGeometryRepresentationForLabel(source, OcctGeometryRepresentation::TriangleMesh)
            || !wrapper.SetGeometryRepresentationForLabel(destination, OcctGeometryRepresentation::TriangleMesh)) Standard_Failure::Raise("Copy representation fixture failed.");
        if (mode == 3) {
            TDataStd_Real::Set(source.FindChild(8, Standard_True), -2.0);
            TDataStd_Real::Set(destination.FindChild(8, Standard_True), 3.0);
        }
        owner.document->CommitCommand(); owner.document->ClearUndos();
        if (!wrapper.MigrateLegacyIdentifiers()) Standard_Failure::Raise("Copy fixture identifier migration failed.");
        if (mode != 9) assign(source, archive);
        if (mode <= 4 || mode == 9) assign(destination, replacement);
        auto capture = [&](const TDF_Label& label) { OcctObjectTransformState value; if (!wrapper.CaptureObjectTransformStateForLabel(label,value)) Standard_Failure::Raise("Copy fixture capture failed."); return value; };
        auto stored = [&](const TDF_Label& label, const Handle(TDocStd_Document)& doc) -> NSData* {
            OcctAuthoredFrameRecord record;
            if (Core3DReadAuthoredFrameOwner(doc,label,record) == OcctAuthoredFrameReadState::Invalid) Standard_Failure::Raise("Copied frame owner invalid.");
            return [NSData dataWithBytes:record.archive.data() length:record.archive.size()];
        };
        const auto original = capture(source), previous = capture(destination);
        const bool closedRejected = !wrapper.CopyGeometryOwnedMeshMetadata(source,destination);
        owner.document->NewCommand();
        const int beforeTime = owner.document->GetData()->Time();
        const bool accepted = wrapper.CopyGeometryOwnedMeshMetadata(source,destination);
        NSMutableDictionary* result = [@{@"accepted": @(accepted), @"closedRejected": @(closedRejected)} mutableCopy];
        if (mode >= 5) {
            result[@"rejectedWithoutMutation"] = @(!accepted && original.IsEqual(capture(source)) && previous.IsEqual(capture(destination)) && beforeTime == owner.document->GetData()->Time());
            owner.document->AbortCommand(); return result;
        }
        if (!accepted || !owner.document->CommitCommand()) Standard_Failure::Raise("Frame copy did not commit.");
        const auto copied = capture(destination);
        result[@"sourceUnchanged"] = @(original.IsEqual(capture(source)));
        result[@"targetGeometryAndTransformPreserved"] = @(copied.shape.IsEqual(previous.shape) && copied.scalars == previous.scalars
            && copied.entityIdentifier == previous.entityIdentifier && copied.definitionIdentifier == previous.definitionIdentifier
            && copied.entityIdentifier != original.entityIdentifier && copied.definitionIdentifier != original.definitionIdentifier);
        NSMutableArray* bytes = [NSMutableArray arrayWithObject:stored(destination,owner.document)];
        NSMutableArray* history = [NSMutableArray arrayWithObject:@[@(owner.document->GetAvailableUndos()),@(owner.document->GetAvailableRedos())]];
        auto keep = [&]() { [bytes addObject:stored(destination,owner.document)]; [history addObject:@[@(owner.document->GetAvailableUndos()),@(owner.document->GetAvailableRedos())]]; };
        if (!owner.document->Undo()) Standard_Failure::Raise("Frame copy undo failed."); keep();
        if (!owner.document->Redo()) Standard_Failure::Raise("Frame copy redo failed."); keep();
        owner.document->NewCommand(); source.ForgetAttribute(AuthoredFrameAttributeID());
        if (!wrapper.CopyGeometryOwnedMeshMetadata(source,destination) || !owner.document->CommitCommand()) Standard_Failure::Raise("Absent-source frame clear failed."); keep();
        if (!owner.document->Undo()) Standard_Failure::Raise("Absent-source frame clear undo failed."); keep();
        owner.document->NewCommand(); assign(source,replacement);
        if (!wrapper.CopyGeometryOwnedMeshMetadata(source,destination)) Standard_Failure::Raise("Aborted frame copy stage failed.");
        owner.document->AbortCommand(); keep();
        result[@"sourceRestoredAfterAbort"] = @(original.IsEqual(capture(source)));
        owner.document->NewCommand();
        Handle(TDocStd_Document) foreign = new TDocStd_Document(TCollection_ExtendedString("BinXCAF"));
        result[@"foreignAndSelfRejected"] = @(!wrapper.CopyGeometryOwnedMeshMetadata(foreign->Main(),destination)
            && !wrapper.CopyGeometryOwnedMeshMetadata(source,foreign->Main()) && !wrapper.CopyGeometryOwnedMeshMetadata(source,source));
        owner.document->AbortCommand();
        TCollection_AsciiString entry; TDF_Tool::Entry(destination,entry);
        std::ostringstream output(std::ios::binary | std::ios::out);
        if (owner.app->SaveAs(owner.document,output) != PCDM_SS_OK) Standard_Failure::Raise("Copied private document save failed.");
        const auto wire=output.str(); if (wire.empty() || wire.size()>1024*1024) Standard_Failure::Raise("Copied private document bound exceeded.");
        Core3DBeginSafeBinaryRead(); std::istringstream input(wire,std::ios::binary | std::ios::in);
        if (reader.app->Open(input,reader.document) != PCDM_RS_OK || Core3DSafeBinaryReadWasRejected() || reader.document.IsNull()) Standard_Failure::Raise("Copied private document reopen failed.");
        TDF_Label reopened; TDF_Tool::Label(reader.document->GetData(),entry.ToCString(),reopened,Standard_False);
        result[@"reopenedArchive"] = stored(reopened,reader.document);
        result[@"archives"] = bytes; result[@"history"] = history;
        Standard_Size nativeBytes=0; result[@"ownersValid"] = @(Core3DValidateAuthoredFrameOwners(reader.document,nativeBytes)); result[@"nativeBytes"] = @(nativeBytes);
        return result;
    } catch (const Standard_Failure& failure) { return @{@"error": [NSString stringWithUTF8String:failure.GetMessageString()] ?: @"OCCT failure"}; }
      catch (...) { return @{@"error": @"Native frame copy fixture failed"}; }
}

+ (NSDictionary<NSString *, id> *)debugFrameDocument:(NSData *)archive
                                       replacement:(NSData *)replacement mode:(NSInteger)mode {
    using namespace core3d::persistence;
    using core3d::scene::Float4;
    if (mode < 0 || mode > 11) return @{@"error": @"Undefined frame document fixture"};
    try {
        const auto fixture = Core3DDebugAuthoredGeometryFixture(0);
        std::vector<Float4> validated;
        for (NSData* input in @[archive, replacement])
            if (!DecodeNativeAuthoredFrames(fixture, static_cast<const std::uint8_t*>(input.bytes), input.length, validated))
                return @{@"error": @"Invalid frame document source fixture"};
        struct PrivateDocument {
            Handle(TDocStd_Application) app = new TDocStd_Application();
            Handle(TDocStd_Document) document;
            void close() {
                if (!document.IsNull()) { app->Close(document); document.Nullify(); }
            }
            ~PrivateDocument() noexcept { try { close(); } catch (...) {} }
        };
        auto start = [](PrivateDocument& owner) {
            Core3DDefineSafeBinXCAFFormat(owner.app);
            owner.app->NewDocument(TCollection_ExtendedString("BinXCAF"), owner.document);
            XCAFDoc_DocumentTool::SetLengthUnit(owner.document, 0.001);
        };
        auto assign = [](const TDF_Label& label, NSData* bytes, bool defaultID) {
            const auto& id = defaultID ? TDataStd_ByteArray::GetID() : AuthoredFrameAttributeID();
            const auto attribute = TDataStd_ByteArray::Set(label, id, 0, Standard_Integer(bytes.length) - 1, Standard_False);
            if (attribute->Length() != Standard_Integer(bytes.length)) attribute->Init(0, Standard_Integer(bytes.length) - 1);
            const auto* source = static_cast<const std::uint8_t*>(bytes.bytes);
            for (NSUInteger i = 0; i < bytes.length; ++i) attribute->SetValue(Standard_Integer(i), source[i]);
        };
        auto storedBytes = [](const TDF_Label& label) -> NSData* {
            Handle(TDataStd_ByteArray) attribute;
            if (!label.FindAttribute(AuthoredFrameAttributeID(), attribute)) return [NSData data];
            if (attribute.IsNull() || attribute->Lower() != 0 || attribute->Upper() < 127
                || attribute->Upper() >= int(core3d::scene::authored::kMaximumArchiveBytes) || attribute->GetDelta())
                Standard_Failure::Raise("Invalid private frame attribute.");
            NSMutableData* bytes = [NSMutableData dataWithLength:attribute->Length()];
            auto* out = static_cast<std::uint8_t*>(bytes.mutableBytes);
            for (int i = 0; i < attribute->Length(); ++i) out[i] = attribute->Value(i);
            return bytes;
        };
        auto serialize = [](PrivateDocument& owner) {
            if (owner.document->HasOpenCommand()) Standard_Failure::Raise("Open private command at save.");
            std::ostringstream output(std::ios::binary | std::ios::out);
            if (owner.app->SaveAs(owner.document, output) != PCDM_SS_OK) Standard_Failure::Raise("Private frame save failed.");
            const auto bytes = output.str();
            if (bytes.empty() || bytes.size() > 1024 * 1024) Standard_Failure::Raise("Private frame wire exceeded fixture bound.");
            return bytes;
        };
        auto makeWire = [&](int count, int damage, int version = 12) {
            PrivateDocument writer; start(writer);
            writer.document->ChangeStorageFormatVersion(static_cast<TDocStd_FormatVersion>(version));
            const auto shapes = XCAFDoc_DocumentTool::ShapeTool(writer.document->Main());
            for (int i = 0; i < std::max(count, 1); ++i) {
                const auto label = shapes->AddShape(Core3DDebugAuthoredGeometryFixture(0), Standard_False);
                if (label.IsNull()) Standard_Failure::Raise("Private frame shape creation failed.");
                TDataStd_Name::Set(label, TCollection_ExtendedString("Private frame geometry"));
                if (count) {
                    NSMutableData* bytes = [archive mutableCopy];
                    if (damage == 2) static_cast<std::uint8_t*>(bytes.mutableBytes)[bytes.length - 1] ^= 1;
                    assign(label, bytes, damage == 1);
                }
            }
            return serialize(writer);
        };
        NSMutableDictionary* result = [NSMutableDictionary dictionary];
        std::vector<std::pair<std::string, int>> wires;
        if (mode >= 7 && mode <= 9) {
            auto damaged = makeWire(0, 0);
            const std::string oldText = mode == 7 ? "START_TYPES" : mode == 8 ? "END_TYPES" : "TDataStd_Name";
            const std::string newText = mode == 7 ? "WRONG_TYPES" : mode == 8 ? "BAD_TYPES" : "TDataStd_Xxxx";
            const auto at = damaged.find(oldText);
            if (at == std::string::npos || damaged.find(oldText, at + oldText.size()) != std::string::npos
                || oldText.size() != newText.size()) Standard_Failure::Raise("Ambiguous type-table fixture.");
            damaged.replace(at, oldText.size(), newText);
            wires.emplace_back(std::move(damaged), 0); wires.emplace_back(makeWire(0, 0), 0);
        } else if (mode == 11) {
            wires.emplace_back(makeWire(0, 0, 99), 0); wires.emplace_back(makeWire(0, 0), 0);
        } else if (mode == 10) {
            wires.emplace_back(makeWire(0, 0, 7), 0); wires.emplace_back(makeWire(0, 0, 7), 0);
        } else if (mode == 6) {
            PrivateDocument writer; start(writer);
            const auto label = XCAFDoc_DocumentTool::ShapeTool(writer.document->Main())->AddShape(fixture, Standard_False);
            writer.document->ClearUndos(); writer.document->SetUndoLimit(20);
            NSMutableArray* history = [NSMutableArray array]; NSMutableArray* counts = [NSMutableArray array];
            bool geometryStable = true;
            GeometryIdentity original; NativeAuthoredGeometryIdentity(fixture, original);
            auto capture = [&]() {
                [history addObject:storedBytes(label)];
                [counts addObject:@[@(writer.document->GetAvailableUndos()), @(writer.document->GetAvailableRedos())]];
                GeometryIdentity current;
                const auto shape = XCAFDoc_ShapeTool::GetShape(label);
                geometryStable = geometryStable && !shape.IsNull() && shape.ShapeType() == TopAbs_FACE
                    && NativeAuthoredGeometryIdentity(TopoDS::Face(shape), current) && current == original;
            };
            auto change = [&](NSData* bytes) {
                writer.document->NewCommand(); assign(label, bytes, false);
                if (!writer.document->CommitCommand()) Standard_Failure::Raise("Private frame command was empty.");
                capture();
            };
            change(archive); change(replacement);
            if (!writer.document->Undo()) Standard_Failure::Raise("Private frame undo failed."); capture();
            if (!writer.document->Redo()) Standard_Failure::Raise("Private frame redo failed."); capture();
            writer.document->NewCommand();
            if (!label.ForgetAttribute(AuthoredFrameAttributeID()) || !writer.document->CommitCommand())
                Standard_Failure::Raise("Private frame removal failed.");
            capture();
            if (!writer.document->Undo()) Standard_Failure::Raise("Private frame removal undo failed."); capture();
            if (!writer.document->Redo()) Standard_Failure::Raise("Private frame removal redo failed."); capture();
            if (!writer.document->Undo()) Standard_Failure::Raise("Private frame restore failed."); capture();
            const int beforeUndo = writer.document->GetAvailableUndos();
            writer.document->NewCommand(); assign(label, archive, false); writer.document->AbortCommand();
            result[@"abortRestoredBytes"] = storedBytes(label);
            result[@"abortAddedNoUndo"] = @(writer.document->GetAvailableUndos() == beforeUndo);
            result[@"historyBytes"] = history; result[@"historyCounts"] = counts;
            result[@"geometryStable"] = @(geometryStable);
            const int beforeSaveUndo = writer.document->GetAvailableUndos(), beforeSaveRedo = writer.document->GetAvailableRedos();
            wires.emplace_back(serialize(writer), 1);
            result[@"savePreservedHistory"] = @(beforeSaveUndo == writer.document->GetAvailableUndos()
                && beforeSaveRedo == writer.document->GetAvailableRedos());
        } else if (mode == 1) {
            wires.emplace_back(makeWire(2, 0), 2); wires.emplace_back(makeWire(1, 0), 1);
            wires.emplace_back(makeWire(2, 0), 2); wires.emplace_back(makeWire(1, 0), 1);
        } else if (mode == 2) {
            wires.emplace_back(makeWire(2, 0), 2); wires.emplace_back(makeWire(2, 0), 2);
        } else if (mode == 3 || mode == 4) {
            wires.emplace_back(makeWire(1, int(mode - 2)), 1); wires.emplace_back(makeWire(1, 0), 1);
        } else if (mode == 5) {
            wires.emplace_back(makeWire(1, 0), 1); wires.emplace_back(makeWire(0, 0), 0);
        } else {
            wires.emplace_back(makeWire(1, 0), 1); wires.emplace_back(makeWire(1, 0), 1);
        }
        PrivateDocument reader;
        auto budget = std::make_shared<AuthoredFrameReadBudget>();
        budget->limit = archive.length * (mode == 1 || mode == 2 ? 2 : 1) - (mode == 1 ? 1 : 0);
        const bool prototype = mode <= 4 || mode == 6;
        if (!prototype) Core3DDefineSafeBinXCAFFormat(reader.app);
        else Core3DDebugDefineFrameBinXCAFFormat(reader.app, budget);
        NSMutableArray* reads = [NSMutableArray array];
        for (const auto& wire : wires) {
            // Exercise entry reset as well as terminal/Clear reset on every read.
            if (prototype) { budget->bytes = budget->limit; budget->rejected = true; }
            const int documentsBefore = reader.app->NbDocuments();
            Core3DBeginSafeBinaryRead(); int status = -1; bool threw = false;
            std::istringstream input(wire.first, std::ios::binary | std::ios::in);
            try { status = int(reader.app->Open(input, reader.document)); } catch (...) { threw = true; }
            const bool rejected = Core3DSafeBinaryReadWasRejected();
            const bool readerAccepted = !threw && status == int(PCDM_RS_OK) && !rejected && !reader.document.IsNull();
            bool accepted = readerAccepted;
            const bool budgetCleared = budget->bytes == 0 && !budget->rejected;
            NSMutableArray* archives = [NSMutableArray array]; NSMutableArray* frameValues = [NSMutableArray array];
            if (accepted) {
                double unit = 0;
                accepted = XCAFDoc_DocumentTool::GetLengthUnit(reader.document, unit) && unit == 0.001;
                TDF_LabelSequence labels; XCAFDoc_DocumentTool::ShapeTool(reader.document->Main())->GetFreeShapes(labels);
                accepted = accepted && labels.Length() == std::max(wire.second, 1);
                for (int i = 1; accepted && i <= labels.Length(); ++i) {
                    NSData* bytes = storedBytes(labels.Value(i));
                    if (wire.second == 0) { accepted = bytes.length == 0; continue; }
                    const auto shape = XCAFDoc_ShapeTool::GetShape(labels.Value(i));
                    std::vector<Float4> frames;
                    accepted = !shape.IsNull() && shape.ShapeType() == TopAbs_FACE
                        && DecodeNativeAuthoredFrames(TopoDS::Face(shape), static_cast<const std::uint8_t*>(bytes.bytes), bytes.length, frames);
                    if (accepted) {
                        [archives addObject:bytes];
                        [frameValues addObject:[NSData dataWithBytes:frames.data() length:frames.size() * sizeof(Float4)]];
                    }
                }
            }
            reader.close();
            [reads addObject:@{@"accepted": @(accepted), @"readerAccepted": @(readerAccepted), @"rejected": @(rejected), @"status": @(status),
                @"threw": @(threw), @"budgetCleared": @(budgetCleared),
                @"rejectedAfterClose": @(Core3DSafeBinaryReadWasRejected()), @"closed": @(reader.document.IsNull()),
                @"sessionCountRestored": @(reader.app->NbDocuments() == documentsBefore),
                @"archives": archives, @"frames": frameValues}];
        }
        result[@"reads"] = reads; result[@"budgetLimit"] = @(budget->limit);
        return result;
    } catch (const Standard_Failure& failure) {
        return @{@"error": [NSString stringWithUTF8String:failure.GetMessageString()] ?: @"OCCT failure"};
    } catch (...) { return @{@"error": @"Private frame document fixture failed"}; }
}
#endif
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


@implementation Core3DSceneTextureSnapshot

- (instancetype)initWithIdentifier:(NSString *)identifier
                           encoding:(Core3DSceneTextureEncoding)encoding
                         pixelWidth:(uint32_t)pixelWidth
                        pixelHeight:(uint32_t)pixelHeight
                        encodedData:(NSData *)encodedData {
    self = [super init];
    if (self) {
        _identifier = [identifier copy];
        _encoding = encoding;
        _pixelWidth = pixelWidth;
        _pixelHeight = pixelHeight;
        _encodedData = [encodedData copy];
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
                           cullMode:(Core3DSceneCullMode)cullMode
              baseColorTextureIndex:(NSInteger)baseColorTextureIndex
               emissiveTextureIndex:(NSInteger)emissiveTextureIndex
      metallicRoughnessTextureIndex:(NSInteger)metallicRoughnessTextureIndex
              occlusionTextureIndex:(NSInteger)occlusionTextureIndex
                 normalTextureIndex:(NSInteger)normalTextureIndex {
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
        _cullMode = cullMode;
        _doubleSided = cullMode == Core3DSceneCullModeNone;
        _baseColorTextureIndex = baseColorTextureIndex;
        _hasBaseColorTexture = baseColorTextureIndex >= 0;
        _emissiveTextureIndex = emissiveTextureIndex;
        _hasEmissiveTexture = emissiveTextureIndex >= 0;
        _metallicRoughnessTextureIndex = metallicRoughnessTextureIndex;
        _hasMetallicRoughnessTexture = metallicRoughnessTextureIndex >= 0;
        _occlusionTextureIndex = occlusionTextureIndex;
        _hasOcclusionTexture = occlusionTextureIndex >= 0;
        _normalTextureIndex = normalTextureIndex;
        _hasNormalTexture = normalTextureIndex >= 0;
    }
    return self;
}

@end


@implementation Core3DSceneFacePrimitiveSnapshot

- (instancetype)initWithFirstIndex:(uint32_t)firstIndex
                         indexCount:(uint32_t)indexCount
                          faceIndex:(uint32_t)faceIndex
              hasTextureCoordinates:(BOOL)hasTextureCoordinates {
    self = [super init];
    if (self) {
        _firstIndex = firstIndex;
        _indexCount = indexCount;
        _faceIndex = faceIndex;
        _hasTextureCoordinates = hasTextureCoordinates;
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
                                    faceCount:(uint32_t)faceCount
                                    edgeCount:(uint32_t)edgeCount
                           topologyVertexCount:(uint32_t)topologyVertexCount
                            cornerTangentData:(NSData *)cornerTangentData
                            tangentIdentifier:(NSString *)tangentIdentifier
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
        _faceCount = faceCount;
        _edgeCount = edgeCount;
        _topologyVertexCount = topologyVertexCount;
        _cornerTangentData = [cornerTangentData copy];
        _tangentIdentifier = [tangentIdentifier copy];
        _hasCornerTangents = cornerTangentData.length != 0;
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
                         hasReferenceAxis:(BOOL)hasReferenceAxis
                      referencePivotWorld:(simd_double3)referencePivotWorld
                  referenceDirectionWorld:(simd_double3)referenceDirectionWorld
                      referencePivotSpace:(Core3DSceneReferenceSpace)referencePivotSpace
                  referenceDirectionSpace:(Core3DSceneReferenceSpace)referenceDirectionSpace
                    referenceAxisAuthored:(BOOL)referenceAxisAuthored
                                  winding:(Core3DSceneWinding)winding
                                  visible:(BOOL)visible
                               selectable:(BOOL)selectable
                                 selected:(BOOL)selected
                                     name:(NSString *)name
                          groupIdentifier:(NSString *)groupIdentifier
                                groupName:(NSString *)groupName
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
        _hasReferenceAxis = hasReferenceAxis;
        _referencePivotWorld = referencePivotWorld;
        _referenceDirectionWorld = referenceDirectionWorld;
        _referencePivotSpace = referencePivotSpace;
        _referenceDirectionSpace = referenceDirectionSpace;
        _referenceAxisAuthored = referenceAxisAuthored;
        _winding = winding;
        _visible = visible;
        _selectable = selectable;
        _selected = selected;
        _name = [name copy];
        _groupIdentifier = [groupIdentifier copy];
        _groupName = [groupName copy];
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
                         metersPerUnit:(double)metersPerUnit
                          renderOrigin:(simd_double3)renderOrigin
                         selectionMode:(Core3DSceneElementKind)selectionMode
                                meshes:(NSArray<Core3DSceneMeshSnapshot *> *)meshes
                           renderItems:(NSArray<Core3DSceneRenderItemSnapshot *> *)renderItems
                             materials:(NSArray<Core3DSceneMaterialSnapshot *> *)materials
                              textures:(NSArray<Core3DSceneTextureSnapshot *> *)textures
                             pickTable:(NSArray<Core3DSceneElementIdentifier *> *)pickTable
                                camera:(Core3DSceneCameraSnapshot *)camera
                             selection:(Core3DSceneSelectionSnapshot *)selection {
    self = [super init];
    if (self) {
        _schemaVersion = schemaVersion;
        _publicationSourceIdentifier = [publicationSourceIdentifier copy];
        _revisions = revisions;
        _metersPerUnit = metersPerUnit;
        _renderOrigin = renderOrigin;
        _selectionMode = selectionMode;
        _meshes = [meshes copy];
        _renderItems = [renderItems copy];
        _materials = [materials copy];
        _textures = [textures copy];
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
constexpr std::size_t kMaximumDTOTextures = 256;
constexpr std::size_t kMaximumDTOPrimitives = 250'000;
constexpr std::size_t kMaximumDTOBindings = 250'000;
constexpr std::size_t kMaximumDTOPickEntries = 250'001;
constexpr std::size_t kMaximumDTOSelectedElements = 50'000;
constexpr std::size_t kMaximumDTOVertices = 1'500'000;
constexpr std::size_t kMaximumDTOIndices = 4'500'000;
constexpr std::size_t kMaximumDTONumericBytes = 96ULL * 1024ULL * 1024ULL;
constexpr std::size_t kReferenceAxisInstanceNumericBytes =
    sizeof(Matrix4d) + sizeof(ReferenceAxisSnapshot);
constexpr std::size_t kMaximumDTOTextureBytes = 64ULL * 1024ULL * 1024ULL;
constexpr std::size_t kMaximumDTOPerTextureBytes = 32ULL * 1024ULL * 1024ULL;
constexpr std::size_t kMaximumDTODecodedTextureBytes =
    128ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t kMaximumDTOTextureDimension = 8192;
constexpr std::uint64_t kMaximumDTOTexturePixels = 4096ULL * 4096ULL;
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
constexpr std::size_t kMaximumChamferPreviewBodies = 8;
constexpr std::size_t kMaximumLinearArrayPreviewBodies = 15;
constexpr std::size_t kMaximumRadialArrayPreviewBodies = 15;
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

bool IsValid(const CullMode value) noexcept {
    switch (value) {
        case CullMode::None:
        case CullMode::Back:
        case CullMode::Front:
            return true;
    }
    return false;
}

bool IsValid(const TextureEncoding value) noexcept {
    switch (value) {
        case TextureEncoding::PNG:
        case TextureEncoding::JPEG:
        case TextureEncoding::GIF:
        case TextureEncoding::TIFF:
        case TextureEncoding::BMP:
        case TextureEncoding::WebP:
            return true;
    }
    return false;
}

bool HasExpectedSignature(const TextureResourceSnapshot& value) noexcept {
    const std::vector<std::uint8_t>& bytes = value.encodedBytes;
    switch (value.encoding) {
        case TextureEncoding::PNG:
            return bytes.size() >= 8
                && bytes[0] == 0x89U && bytes[1] == 0x50U
                && bytes[2] == 0x4eU && bytes[3] == 0x47U
                && bytes[4] == 0x0dU && bytes[5] == 0x0aU
                && bytes[6] == 0x1aU && bytes[7] == 0x0aU;
        case TextureEncoding::JPEG:
            return bytes.size() >= 3
                && bytes[0] == 0xffU && bytes[1] == 0xd8U
                && bytes[2] == 0xffU;
        case TextureEncoding::GIF:
            return bytes.size() >= 6 && bytes[0] == 'G' && bytes[1] == 'I'
                && bytes[2] == 'F' && bytes[3] == '8'
                && (bytes[4] == '7' || bytes[4] == '9')
                && bytes[5] == 'a';
        case TextureEncoding::TIFF:
            return bytes.size() >= 4
                && ((bytes[0] == 'I' && bytes[1] == 'I'
                        && bytes[2] == 0x2aU && bytes[3] == 0x00U)
                    || (bytes[0] == 'M' && bytes[1] == 'M'
                        && bytes[2] == 0x00U && bytes[3] == 0x2aU));
        case TextureEncoding::BMP:
            return bytes.size() >= 2 && bytes[0] == 'B' && bytes[1] == 'M';
        case TextureEncoding::WebP:
            return bytes.size() >= 12
                && bytes[0] == 'R' && bytes[1] == 'I'
                && bytes[2] == 'F' && bytes[3] == 'F'
                && bytes[8] == 'W' && bytes[9] == 'E'
                && bytes[10] == 'B' && bytes[11] == 'P';
    }
    return false;
}

bool HasValidImageMetadata(const TextureResourceSnapshot& value) noexcept {
    if (value.encodedBytes.empty()) {
        return false;
    }
    CFDataRef data = CFDataCreateWithBytesNoCopy(
        kCFAllocatorDefault,
        reinterpret_cast<const UInt8*>(value.encodedBytes.data()),
        static_cast<CFIndex>(value.encodedBytes.size()),
        kCFAllocatorNull);
    if (data == nullptr) {
        return false;
    }
    const void* optionKeys[] = {kCGImageSourceShouldCache};
    const void* optionValues[] = {kCFBooleanFalse};
    CFDictionaryRef options = CFDictionaryCreate(
        kCFAllocatorDefault,
        optionKeys,
        optionValues,
        1,
        &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);
    CGImageSourceRef source = CGImageSourceCreateWithData(data, options);
    if (options != nullptr) {
        CFRelease(options);
    }
    CFRelease(data);
    if (source == nullptr || CGImageSourceGetType(source) == nullptr
        || CGImageSourceGetCount(source) != 1
        || CGImageSourceGetStatus(source) != kCGImageStatusComplete
        || CGImageSourceGetStatusAtIndex(source, 0)
            != kCGImageStatusComplete) {
        if (source != nullptr) {
            CFRelease(source);
        }
        return false;
    }
    CFDictionaryRef properties =
        CGImageSourceCopyPropertiesAtIndex(source, 0, nullptr);
    CFRelease(source);
    if (properties == nullptr) {
        return false;
    }
    const CFTypeRef widthValue = CFDictionaryGetValue(
        properties, kCGImagePropertyPixelWidth);
    const CFTypeRef heightValue = CFDictionaryGetValue(
        properties, kCGImagePropertyPixelHeight);
    const CFTypeRef depthValue = CFDictionaryGetValue(
        properties, kCGImagePropertyDepth);
    std::int64_t width = 0;
    std::int64_t height = 0;
    std::int64_t depth = 0;
    const bool isValid = widthValue != nullptr && heightValue != nullptr
        && depthValue != nullptr
        && CFGetTypeID(widthValue) == CFNumberGetTypeID()
        && CFGetTypeID(heightValue) == CFNumberGetTypeID()
        && CFGetTypeID(depthValue) == CFNumberGetTypeID()
        && CFNumberGetValue(static_cast<CFNumberRef>(widthValue),
                            kCFNumberSInt64Type,
                            &width)
        && CFNumberGetValue(static_cast<CFNumberRef>(heightValue),
                            kCFNumberSInt64Type,
                            &height)
        && CFNumberGetValue(static_cast<CFNumberRef>(depthValue),
                            kCFNumberSInt64Type,
                            &depth)
        && width == value.pixelWidth
        && height == value.pixelHeight
        && depth > 0 && depth <= 8;
    CFRelease(properties);
    return isValid;
}

bool HasContentAddressedIdentifier(const std::string& value) noexcept {
    static constexpr std::string_view prefix = "texture-sha256-";
    if (value.size() != prefix.size() + 64
        || value.compare(0, prefix.size(), prefix) != 0) {
        return false;
    }
    return std::all_of(value.begin() + prefix.size(), value.end(),
                       [](const char character) {
        return (character >= '0' && character <= '9')
            || (character >= 'a' && character <= 'f');
    });
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
        case RenderRole::LinearArrayPreview:
        case RenderRole::ShellPreview:
        case RenderRole::RadialArrayPreview:
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

bool IsValid(const ReferenceSpace value) noexcept {
    switch (value) {
        case ReferenceSpace::Object:
        case ReferenceSpace::World:
            return true;
    }
    return false;
}

bool IsValid(const ReferenceAxisSnapshot& value) noexcept {
    const double directionSquared =
        value.worldDirection.x * value.worldDirection.x
        + value.worldDirection.y * value.worldDirection.y
        + value.worldDirection.z * value.worldDirection.z;
    return IsValid(value.pivotSpace) && IsValid(value.directionSpace)
        && IsFinite(value.worldPivot) && IsFinite(value.worldDirection)
        && std::abs(value.worldPivot.x)
            <= core3d::limits::kMaximumModelCoordinateMagnitude
        && std::abs(value.worldPivot.y)
            <= core3d::limits::kMaximumModelCoordinateMagnitude
        && std::abs(value.worldPivot.z)
            <= core3d::limits::kMaximumModelCoordinateMagnitude
        && IsFinite(directionSquared)
        && std::abs(directionSquared - 1.0) <= 1.0e-10;
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

bool IsValidSceneSelectionMode(const ElementKind value) noexcept {
    switch (value) {
        case ElementKind::Object:
        case ElementKind::Face:
        case ElementKind::Edge:
            return true;
        case ElementKind::None:
        case ElementKind::Vertex:
            return false;
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
        && value.baseColorTextureIndex >= -1
        && value.emissiveTextureIndex >= -1
        && value.metallicRoughnessTextureIndex >= -1
        && value.occlusionTextureIndex >= -1
        && value.normalTextureIndex >= -1
        && IsValid(value.alphaMode)
        && IsValid(value.cullMode);
}

bool HasValidCornerTangents(const MeshSnapshot& mesh) noexcept {
    if (mesh.cornerTangents.empty()) { return mesh.tangentBasis == TangentBasis::None; }
    if ((mesh.tangentBasis != TangentBasis::MikkTSpace
         && mesh.tangentBasis != TangentBasis::Authored)
        || mesh.cornerTangents.size() != mesh.indices.size()
        || mesh.indices.size() > kMaximumTangentTriangles * 3
        || mesh.vertices.size() > kMaximumTangentVertices
        || !std::all_of(mesh.primitives.begin(), mesh.primitives.end(),
            [](const MeshPrimitive& p) { return p.hasTextureCoordinates; })) { return false; }
    for (std::size_t i = 0; i < mesh.indices.size(); ++i) {
        if (mesh.indices[i] >= mesh.vertices.size()) { return false; }
        const auto& v = mesh.vertices[mesh.indices[i]];
        const auto& t = mesh.cornerTangents[i];
        // Interpolating opposite handedness inside one triangle has no defined
        // tangent space. UV seams must split between triangles, not within one.
        if (i % 3 != 0 && t.w != mesh.cornerTangents[i - i % 3].w) { return false; }
        const double n2 = double(v.normalX)*v.normalX + double(v.normalY)*v.normalY
            + double(v.normalZ)*v.normalZ;
        const double t2 = double(t.x)*t.x + double(t.y)*t.y + double(t.z)*t.z;
        const double dot = double(t.x)*v.normalX + double(t.y)*v.normalY + double(t.z)*v.normalZ;
        if (!IsFinite(t) || !std::isfinite(n2) || n2 < 1.0e-24
            || std::abs(t2 - 1.0) > 1.0e-4
            || std::abs(dot) > 1.0e-4 * std::sqrt(n2)
            || (t.w != -1.0f && t.w != 1.0f)) { return false; }
    }
    return true;
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
        || !IsFinite(snapshot.metersPerUnit)
        || snapshot.metersPerUnit <= 0.0
        || !IsFinite(snapshot.renderOrigin)
        || !IsValidSceneSelectionMode(snapshot.selectionMode)
        || !IsValid(snapshot.camera)
        || snapshot.meshes.size() > kMaximumDTOMeshes
        || snapshot.instances.size() > kMaximumDTOInstances
        || snapshot.materials.size() > kMaximumDTOMaterials
        || snapshot.textures.size() > kMaximumDTOTextures
        || snapshot.pickTable.empty()
        || snapshot.pickTable.size() > kMaximumDTOPickEntries
        || (snapshot.selectionMode == ElementKind::Edge
            && snapshot.pickTable.size() != 1)
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

    std::size_t totalEncodedTextureBytes = 0;
    std::size_t totalDecodedTextureBytes = 0;
    std::unordered_set<std::string> textureIdentifiers;
    textureIdentifiers.reserve(snapshot.textures.size());
    for (const TextureResourceSnapshot& texture : snapshot.textures) {
        const std::uint64_t width = texture.pixelWidth;
        const std::uint64_t height = texture.pixelHeight;
        const std::uint64_t pixelCount = width * height;
        const std::uint64_t decodedByteCount = pixelCount * 4ULL;
        if (!IsValidIdentifier(texture.identifier)
            || !HasContentAddressedIdentifier(texture.identifier)
            || !accountString(texture.identifier)
            || !textureIdentifiers.insert(texture.identifier).second
            || !IsValid(texture.encoding)
            || !HasExpectedSignature(texture)
            || !HasValidImageMetadata(texture)
            || width == 0 || height == 0
            || width > kMaximumDTOTextureDimension
            || height > kMaximumDTOTextureDimension
            || pixelCount > kMaximumDTOTexturePixels
            || texture.encodedBytes.empty()
            || texture.encodedBytes.size() > kMaximumDTOPerTextureBytes
            || !CheckedAdd(totalEncodedTextureBytes,
                           texture.encodedBytes.size(),
                           totalEncodedTextureBytes)
            || totalEncodedTextureBytes > kMaximumDTOTextureBytes
            || decodedByteCount > kMaximumDTODecodedTextureBytes
            || !CheckedAdd(totalDecodedTextureBytes,
                           static_cast<std::size_t>(decodedByteCount),
                           totalDecodedTextureBytes)
            || totalDecodedTextureBytes > kMaximumDTODecodedTextureBytes) {
            return false;
        }
    }

    std::vector<std::uint8_t> referencedTextures(snapshot.textures.size(), 0);
    std::unordered_set<std::string> materialIdentifiers;
    materialIdentifiers.reserve(snapshot.materials.size());
    for (const MaterialSnapshot& material : snapshot.materials) {
        if (!IsValid(material) || !accountString(material.identifier)
            || !materialIdentifiers.insert(material.identifier).second) {
            return false;
        }
        if (material.baseColorTextureIndex >= 0) {
            const std::size_t textureIndex = static_cast<std::size_t>(
                material.baseColorTextureIndex);
            if (textureIndex >= snapshot.textures.size()) {
                return false;
            }
            referencedTextures[textureIndex] = 1;
        }
        if (material.emissiveTextureIndex >= 0) {
            const std::size_t textureIndex = static_cast<std::size_t>(
                material.emissiveTextureIndex);
            if (textureIndex >= snapshot.textures.size()) {
                return false;
            }
            referencedTextures[textureIndex] = 1;
        }
        if (material.metallicRoughnessTextureIndex >= 0) {
            const std::size_t textureIndex = static_cast<std::size_t>(
                material.metallicRoughnessTextureIndex);
            if (textureIndex >= snapshot.textures.size()) {
                return false;
            }
            referencedTextures[textureIndex] = 1;
        }
        if (material.normalTextureIndex >= 0) {
            const auto index = static_cast<std::size_t>(material.normalTextureIndex);
            if (index >= snapshot.textures.size()) { return false; }
            referencedTextures[index] = 1;
        }
        if (material.occlusionTextureIndex >= 0) {
            const std::size_t textureIndex = static_cast<std::size_t>(
                material.occlusionTextureIndex);
            if (textureIndex >= snapshot.textures.size()) {
                return false;
            }
            referencedTextures[textureIndex] = 1;
        }
    }
    if (!std::all_of(referencedTextures.begin(), referencedTextures.end(),
                     [](const std::uint8_t referenced) {
                         return referenced != 0;
                     })) {
        return false;
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
            || !HasValidCornerTangents(mesh)
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
        std::size_t tangentBytes = 0;
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
            || !CheckedMultiply(mesh.cornerTangents.size(), sizeof(Float4), tangentBytes)
            || !CheckedAdd(totalNumericBytes, tangentBytes, totalNumericBytes)
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
                || (mesh.topology.faceCount != 0
                    && primitive.faceIndex >= mesh.topology.faceCount)
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
    std::unordered_map<std::string, std::pair<std::string, std::size_t>> savedGroups;
    for (std::size_t instanceIndex = 0;
         instanceIndex < snapshot.instances.size(); ++instanceIndex) {
        const InstanceSnapshot& instance = snapshot.instances[instanceIndex];
        if (!IsValidIdentifier(instance.entityIdentifier)
            || !accountString(instance.entityIdentifier)
            || instance.name.size() > kMaximumNameBytes
            || !accountString(instance.name)
            || !accountString(instance.groupIdentifier) || !accountString(instance.groupName)
            || instance.groupName.size() > 1024
            || instance.groupIdentifier.empty() != instance.groupName.empty()
            || !instancesByIdentifier.emplace(
                instance.entityIdentifier, instanceIndex).second
            || instance.meshIndex >= snapshot.meshes.size()
            || !IsValid(instance.role)
            || !IsValid(instance.coordinateSpace)
            || !IsValid(instance.depthPolicy)
            || !IsValid(instance.renderStyle)
            || !instance.referenceAxis.has_value()
            || !IsValid(*instance.referenceAxis)
            || (!instance.visible && instance.selectable)
            || !CheckedAdd(totalBindings,
                           instance.primitiveBindings.size(),
                           totalBindings)
            || totalBindings > kMaximumDTOBindings
            || !CheckedAdd(totalNumericBytes,
                           kReferenceAxisInstanceNumericBytes,
                           totalNumericBytes)
            || totalNumericBytes > kMaximumDTONumericBytes) {
            return false;
        }
        for (const double value : instance.worldFromObject.values) {
            if (!IsFinite(value)) {
                return false;
            }
        }
        if (!instance.groupIdentifier.empty()) {
            const auto& id = instance.groupIdentifier;
            if (instance.role != RenderRole::Model || id.size() != 36) { return false; }
            for (std::size_t i = 0; i < id.size(); ++i) {
                const char c = id[i];
                if ((i == 8 || i == 13 || i == 18 || i == 23)
                    ? c != '-' : !((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) { return false; }
            }
            auto pair = savedGroups.emplace(id, std::make_pair(instance.groupName, 0));
            if (savedGroups.size() > 128 || pair.first->second.first != instance.groupName
                || ++pair.first->second.second > 32) { return false; }
        }
        const MeshSnapshot& mesh = snapshot.meshes[instance.meshIndex];
        if (instance.primitiveBindings.size() != mesh.primitives.size()
            || (snapshot.selectionMode == ElementKind::Face
                && instance.selectable
                && mesh.topology.faceCount == 0)
            || (snapshot.selectionMode == ElementKind::Edge
                && instance.selectable
                && mesh.topology.edgeCount == 0)) {
            return false;
        }
    }

    std::unordered_set<ElementKey, ElementKeyHash> pickElements;
    pickElements.reserve(snapshot.pickTable.size());
    for (std::size_t index = 1; index < snapshot.pickTable.size(); ++index) {
        const ElementIdentifier& element = snapshot.pickTable[index];
        if (!IsValidElement(element, false)
            || (element.kind != ElementKind::Object
                && element.kind != ElementKind::Face)
            || element.kind != snapshot.selectionMode
            || !accountString(element.entityIdentifier)
            || !pickElements.insert(MakeElementKey(element)).second) {
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
        if (element.geometryRevision != mesh.geometryRevision
            || (element.kind == ElementKind::Object
                && element.topologyIndex != 0)
            || (element.kind == ElementKind::Face
                && element.topologyIndex >= mesh.topology.faceCount)) {
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
            // GPU hit-testability is derived exactly from the published
            // mode, instance, and per-primitive visibility authorities. Edge
            // selection remains semantic/CPU-side and never owns GPU tokens.
            const bool shouldBePickable =
                snapshot.selectionMode != ElementKind::Edge
                && instance.selectable
                && instance.visible && binding.visible;
            if (binding.materialIndex >= snapshot.materials.size()
                || binding.pickToken >= snapshot.pickTable.size()
                || (shouldBePickable && binding.pickToken == 0)
                || (!shouldBePickable && binding.pickToken != 0)) {
                return false;
            }
            const MaterialSnapshot& material = snapshot.materials[binding.materialIndex];
            if (material.normalTextureIndex >= 0
                && (mesh.cornerTangents.empty()
                    || !mesh.primitives[primitiveIndex].hasTextureCoordinates)) { return false; }
            if (binding.pickToken == 0) {
                continue;
            }

            const ElementIdentifier& picked =
                snapshot.pickTable[binding.pickToken];
            const MeshPrimitive& primitive = mesh.primitives[primitiveIndex];
            if (picked.entityIdentifier != instance.entityIdentifier
                || picked.geometryRevision != mesh.geometryRevision
                || picked.kind != snapshot.selectionMode
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

    // Semantic selection is topology-authoritative, not pick-table or
    // tessellation-authoritative. Edge mode intentionally has no GPU picks;
    // Face picks, when present, were independently verified above against an
    // exact render-primitive topology identity.
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
        if (!instance.visible || !instance.selectable
            || element.geometryRevision != mesh.geometryRevision) {
            return false;
        }
        if (element.kind == ElementKind::Object) {
            return snapshot.selectionMode == ElementKind::Object
                && element.topologyIndex == 0;
        }
        if (element.kind == ElementKind::Edge) {
            return snapshot.selectionMode == ElementKind::Edge
                && element.topologyIndex < mesh.topology.edgeCount;
        }
        if (element.kind != ElementKind::Face
            || snapshot.selectionMode != ElementKind::Face
            || element.topologyIndex >= mesh.topology.faceCount) {
            return false;
        }
        return true;
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

bool ValidateSceneSnapshotPayload(const SceneSnapshot& snapshot) noexcept {
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

bool IsProperRigidBoundedWorldTransform(const Matrix4d& value) noexcept {
    return IsRigidWorldAnchorTransform(value)
        && std::abs(value.values[12])
            <= core3d::limits::kMaximumModelCoordinateMagnitude
        && std::abs(value.values[13])
            <= core3d::limits::kMaximumModelCoordinateMagnitude
        && std::abs(value.values[14])
            <= core3d::limits::kMaximumModelCoordinateMagnitude;
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
        case PresentationOverlayKind::BooleanIntersectPreview:
            if (snapshot.meshes.size() != 1
                || snapshot.instances.size() != 1
                || snapshot.materials.size() != 1
                || snapshot.suppressedEntityIdentifiers.size() < 2
                || snapshot.suppressedEntityIdentifiers.size()
                    > kMaximumBooleanSourceOperands) {
                return false;
            }
            break;
        case PresentationOverlayKind::ChamferPreview: {
            const std::size_t itemCount = snapshot.instances.size();
            if (itemCount == 0
                || itemCount > kMaximumChamferPreviewBodies
                || snapshot.meshes.size() != itemCount
                || snapshot.materials.size() != itemCount
                || snapshot.suppressedEntityIdentifiers.size()
                    != itemCount) {
                return false;
            }
            break;
        }
        case PresentationOverlayKind::LinearArrayPreview:
            if (!isEmpty
                && (snapshot.meshes.size() != 1
                    || snapshot.materials.size() != 1
                    || snapshot.instances.empty()
                    || snapshot.instances.size()
                        > kMaximumLinearArrayPreviewBodies)) {
                return false;
            }
            break;
        case PresentationOverlayKind::RadialArrayPreview:
            if (!isEmpty
                && (snapshot.meshes.size() != 1
                    || snapshot.materials.size() != 1
                    || snapshot.instances.empty()
                    || snapshot.instances.size()
                        > kMaximumRadialArrayPreviewBodies)) {
                return false;
            }
            break;
        case PresentationOverlayKind::ShellPreview:
            if (snapshot.meshes.size() != 1
                || snapshot.instances.size() != 1
                || snapshot.materials.size() != 1
                || snapshot.suppressedEntityIdentifiers.size() != 1) {
                return false;
            }
            break;
        default:
            return false;
    }

    const bool isBooleanPreview =
        snapshot.kind == PresentationOverlayKind::BooleanSubtractPreview
        || snapshot.kind == PresentationOverlayKind::BooleanUnionPreview
        || snapshot.kind == PresentationOverlayKind::BooleanIntersectPreview;
    const bool isChamferPreview =
        snapshot.kind == PresentationOverlayKind::ChamferPreview;
    const bool isLinearArrayPreview =
        snapshot.kind == PresentationOverlayKind::LinearArrayPreview;
    const bool isRadialArrayPreview =
        snapshot.kind == PresentationOverlayKind::RadialArrayPreview;
    const bool isShellPreview =
        snapshot.kind == PresentationOverlayKind::ShellPreview;
    const auto booleanEntityIdentifier = [&](const std::size_t index) {
        if (snapshot.kind == PresentationOverlayKind::BooleanUnionPreview) {
            return std::string("boolean/union/result/0");
        }
        if (snapshot.kind
            == PresentationOverlayKind::BooleanIntersectPreview) {
            return std::string("boolean/intersect/result/0");
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
        if (snapshot.kind
            == PresentationOverlayKind::BooleanIntersectPreview) {
            return std::string("Boolean intersect result 0");
        }
        if (index < booleanActorCount) {
            return std::string("Boolean subtract actor ")
                + std::to_string(index);
        }
        return std::string("Boolean subtract result ")
            + std::to_string(index - booleanActorCount);
    };
    const auto chamferEntityIdentifier = [](const std::size_t index) {
        return std::string("chamfer/preview/") + std::to_string(index);
    };
    constexpr const char* kShellPreviewIdentifier = "shell/preview/0";
    if (!isBooleanPreview && !isChamferPreview && !isShellPreview
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
        const bool isChamferMaterial = isChamferPreview;
        const bool isLinearArrayMaterial = isLinearArrayPreview;
        const bool isRadialArrayMaterial = isRadialArrayPreview;
        const bool isShellMaterial = isShellPreview;
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
        } else if (isChamferMaterial) {
            hasExpectedIdentifier = material.identifier
                == chamferEntityIdentifier(materialIndex) + "/material";
            hasExpectedAlpha = material.alphaMode == AlphaMode::Opaque
                && material.baseColor.w == 1.0f;
        } else if (isLinearArrayMaterial) {
            hasExpectedIdentifier = materialIndex == 0
                && material.identifier
                    == "linear-array/source/0/material";
            hasExpectedAlpha = material.alphaMode == AlphaMode::Opaque
                && material.baseColor.w == 1.0f;
        } else if (isRadialArrayMaterial) {
            hasExpectedIdentifier = materialIndex == 0
                && material.identifier
                    == "radial-array/source/0/material";
            hasExpectedAlpha = material.alphaMode == AlphaMode::Opaque
                && material.baseColor.w == 1.0f;
        } else if (isShellMaterial) {
            hasExpectedIdentifier = materialIndex == 0
                && material.identifier
                    == std::string(kShellPreviewIdentifier) + "/material";
            hasExpectedAlpha = material.alphaMode == AlphaMode::Opaque
                && material.baseColor.w == 1.0f;
        } else {
            hasExpectedAlpha = material.alphaMode == AlphaMode::Opaque
                && material.baseColor.w == 1.0f;
        }
        if (!hasExpectedIdentifier || !hasExpectedColor
            || !IsValid(material)
            || material.baseColorTextureIndex != -1
            || material.emissiveTextureIndex != -1
            || material.metallicRoughnessTextureIndex != -1
            || material.occlusionTextureIndex != -1
            || material.normalTextureIndex != -1
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
        const bool isChamferMesh = isChamferPreview;
        const bool isLinearArrayMesh = isLinearArrayPreview;
        const bool isRadialArrayMesh = isRadialArrayPreview;
        const bool isShellMesh = isShellPreview;
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
            || (isChamferMesh
                && mesh.definitionIdentifier
                    != chamferEntityIdentifier(meshIndex) + "/mesh")
            || (isLinearArrayMesh
                && (meshIndex != 0
                    || mesh.definitionIdentifier
                        != "linear-array/source/0/mesh"))
            || (isRadialArrayMesh
                && (meshIndex != 0
                    || mesh.definitionIdentifier
                        != "radial-array/source/0/mesh"))
            || (isShellMesh
                && (meshIndex != 0
                    || mesh.definitionIdentifier
                        != std::string(kShellPreviewIdentifier) + "/mesh"))
            || !IsValidIdentifier(mesh.definitionIdentifier)
            || !definitionIdentifiers.insert(
                mesh.definitionIdentifier).second
            || mesh.geometryRevision == 0
            || !mesh.localBounds.valid || !IsValid(mesh.localBounds)
            || ((isMirrorPreview || isBooleanMesh || isChamferMesh
                    || isLinearArrayMesh || isRadialArrayMesh || isShellMesh)
                && !IsCenteredLocalBounds(mesh.localBounds))
            || mesh.vertices.empty() || mesh.indices.empty()
            || mesh.primitives.empty()
            || !mesh.cornerTangents.empty() || mesh.tangentBasis != TangentBasis::None
            || (!isMirrorPreview && !isBooleanMesh && !isChamferMesh
                    && !isLinearArrayMesh && !isRadialArrayMesh && !isShellMesh
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
        const bool isChamferItem = isChamferPreview;
        const bool isLinearArrayItem = isLinearArrayPreview;
        const bool isRadialArrayItem = isRadialArrayPreview;
        const bool isShellItem = isShellPreview;
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
                    : isChamferItem
                        ? instance.entityIdentifier
                                == chamferEntityIdentifier(instanceIndex)
                            && instance.name
                                == "Chamfer preview "
                                    + std::to_string(instanceIndex)
                            && instance.meshIndex == instanceIndex
                    : isLinearArrayItem
                        ? instance.entityIdentifier
                                == "linear-array/preview/0/"
                                    + std::to_string(instanceIndex + 1U)
                            && instance.name
                                == "Linear array preview "
                                    + std::to_string(instanceIndex + 1U)
                            && instance.meshIndex == 0
                    : isRadialArrayItem
                        ? instance.entityIdentifier
                                == "radial-array/preview/0/"
                                    + std::to_string(instanceIndex + 1U)
                            && instance.name
                                == "Radial array preview "
                                    + std::to_string(instanceIndex + 1U)
                            && instance.meshIndex == 0
                    : isShellItem
                        ? instance.entityIdentifier
                                == kShellPreviewIdentifier
                            && instance.name == "Shell preview 0"
                            && instance.meshIndex == 0
                    : true;
        const bool hasExpectedSemantics = isBooleanItem
            ? instance.coordinateSpace == CoordinateSpace::World
                && instance.depthPolicy == DepthPolicy::Scene
                && (instanceIndex < booleanActorCount
                    ? instance.role == RenderRole::BooleanActor
                        && instance.renderStyle == RenderStyle::Wireframe
                    : instance.role == RenderRole::BooleanSubject
                        && instance.renderStyle == RenderStyle::Shaded)
            : (isMirrorPreview || isChamferItem || isLinearArrayItem
                    || isRadialArrayItem
                    || isShellItem)
                ? instance.role == (isChamferItem
                        ? RenderRole::ChamferPreview
                        : isLinearArrayItem
                            ? RenderRole::LinearArrayPreview
                            : isRadialArrayItem
                                ? RenderRole::RadialArrayPreview
                            : isShellItem
                                ? RenderRole::ShellPreview
                            : RenderRole::MirrorPreview)
                    && instance.coordinateSpace == CoordinateSpace::World
                    && instance.depthPolicy == DepthPolicy::Scene
                : instance.role == RenderRole::Gizmo
                    && instance.coordinateSpace
                        == CoordinateSpace::WorldAnchorPixels
                    && instance.depthPolicy == DepthPolicy::Topmost;
        const std::size_t expectedBindingCount =
            (isMirrorPreview || isBooleanItem || isChamferItem
                || isLinearArrayItem || isRadialArrayItem || isShellItem)
            ? snapshot.meshes[(isLinearArrayItem || isRadialArrayItem)
                    ? 0 : instanceIndex]
                .primitives.size()
            : 1;
        if (!hasExpectedIdentity
            || !IsValidIdentifier(instance.entityIdentifier)
            || !entityIdentifiers.insert(instance.entityIdentifier).second
            || instance.meshIndex >= snapshot.meshes.size()
            || instance.reversesWinding || !instance.visible
            || instance.selectable || instance.selected
            || instance.referenceAxis.has_value()
            || !hasExpectedSemantics
            || (!isBooleanItem && !isChamferItem && !isShellItem
                && instance.renderStyle != RenderStyle::Shaded)
            || ((isChamferItem || isShellItem)
                && instance.renderStyle != RenderStyle::Shaded)
            || !instance.groupIdentifier.empty() || !instance.groupName.empty()
            || instance.name.size() > kMaximumNameBytes
            || !CheckedAdd(stringBytes,
                           instance.entityIdentifier.size(),
                           stringBytes)
            || !CheckedAdd(stringBytes,
                           instance.name.size(),
                           stringBytes)
            || stringBytes > kMaximumDTOStringBytes
            || ((isMirrorPreview || isBooleanItem || isChamferItem
                    || isLinearArrayItem || isRadialArrayItem || isShellItem)
                ? (isRadialArrayItem
                    ? !IsProperRigidBoundedWorldTransform(
                        instance.worldFromObject)
                    : !IsTranslationOnlyWorldTransform(
                        instance.worldFromObject))
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
        if (++meshReferences[instance.meshIndex] != 1
            && !isLinearArrayItem && !isRadialArrayItem) {
            return false;
        }
        for (const PrimitiveBinding& binding :
             instance.primitiveBindings) {
            if (((isMirrorPlane || isMirrorPreview
                    || isBooleanItem || isChamferItem
                    || isLinearArrayItem || isRadialArrayItem || isShellItem)
                    && binding.materialIndex
                        != ((isLinearArrayItem || isRadialArrayItem)
                            ? 0 : instanceIndex))
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
    if (isLinearArrayPreview || isRadialArrayPreview) {
        return isEmpty
            || (meshReferences.size() == 1
            && meshReferences[0] == snapshot.instances.size()
            && materialReferences.size() == 1
            && materialReferences[0] == 1);
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

Core3DSceneCullMode CullModeFromScene(CullMode value) {
    switch (value) {
        case CullMode::None:
            return Core3DSceneCullModeNone;
        case CullMode::Back:
            return Core3DSceneCullModeBack;
        case CullMode::Front:
            return Core3DSceneCullModeFront;
    }

    NSCAssert(NO, @"Unknown scene cull mode value: %u", static_cast<unsigned>(value));
    return Core3DSceneCullModeBack;
}

Core3DSceneTextureEncoding TextureEncodingFromScene(TextureEncoding value) {
    switch (value) {
        case TextureEncoding::PNG:
            return Core3DSceneTextureEncodingPNG;
        case TextureEncoding::JPEG:
            return Core3DSceneTextureEncodingJPEG;
        case TextureEncoding::GIF:
            return Core3DSceneTextureEncodingGIF;
        case TextureEncoding::TIFF:
            return Core3DSceneTextureEncodingTIFF;
        case TextureEncoding::BMP:
            return Core3DSceneTextureEncodingBMP;
        case TextureEncoding::WebP:
            return Core3DSceneTextureEncodingWebP;
    }

    NSCAssert(NO, @"Unknown scene texture encoding: %u",
              static_cast<unsigned>(value));
    return Core3DSceneTextureEncodingPNG;
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
        case RenderRole::LinearArrayPreview:
            return Core3DSceneRenderRoleLinearArrayPreview;
        case RenderRole::ShellPreview:
            return Core3DSceneRenderRoleShellPreview;
        case RenderRole::RadialArrayPreview:
            return Core3DSceneRenderRoleRadialArrayPreview;
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

Core3DSceneReferenceSpace ReferenceSpaceFromScene(ReferenceSpace value) {
    switch (value) {
        case ReferenceSpace::Object:
            return Core3DSceneReferenceSpaceObject;
        case ReferenceSpace::World:
            return Core3DSceneReferenceSpaceWorld;
    }

    NSCAssert(NO, @"Unknown reference-space value: %u",
              static_cast<unsigned>(value));
    return Core3DSceneReferenceSpaceObject;
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
        case PresentationOverlayKind::BooleanIntersectPreview:
            return Core3DScenePresentationOverlayKindBooleanIntersectPreview;
        case PresentationOverlayKind::ChamferPreview:
            return Core3DScenePresentationOverlayKindChamferPreview;
        case PresentationOverlayKind::LinearArrayPreview:
            return Core3DScenePresentationOverlayKindLinearArrayPreview;
        case PresentationOverlayKind::ShellPreview:
            return Core3DScenePresentationOverlayKindShellPreview;
        case PresentationOverlayKind::RadialArrayPreview:
            return Core3DScenePresentationOverlayKindRadialArrayPreview;
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
                   cullMode:CullModeFromScene(value.cullMode)
      baseColorTextureIndex:value.baseColorTextureIndex
       emissiveTextureIndex:value.emissiveTextureIndex
metallicRoughnessTextureIndex:value.metallicRoughnessTextureIndex
      occlusionTextureIndex:value.occlusionTextureIndex
         normalTextureIndex:value.normalTextureIndex];
}

Core3DSceneTextureSnapshot *TextureFromScene(
    const TextureResourceSnapshot& value) {
    NSData *encodedData = value.encodedBytes.empty()
        ? NSData.data
        : [NSData dataWithBytes:value.encodedBytes.data()
                         length:value.encodedBytes.size()];
    return [[Core3DSceneTextureSnapshot alloc]
        initWithIdentifier:StringFromUTF8(value.identifier)
                   encoding:TextureEncodingFromScene(value.encoding)
                 pixelWidth:value.pixelWidth
                pixelHeight:value.pixelHeight
                encodedData:encodedData];
}

Core3DSceneFacePrimitiveSnapshot *FacePrimitiveFromScene(const MeshPrimitive& value) {
    return [[Core3DSceneFacePrimitiveSnapshot alloc]
        initWithFirstIndex:value.firstIndex
                indexCount:value.indexCount
                 faceIndex:value.faceIndex
     hasTextureCoordinates:value.hasTextureCoordinates];
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
    NSData *tangents = value.cornerTangents.empty() ? NSData.data :
        [NSData dataWithBytes:value.cornerTangents.data()
                       length:value.cornerTangents.size() * sizeof(Float4)];
    NSString *tangentIdentifier = @"";
    if (tangents.length != 0) {
        unsigned char digest[CC_SHA256_DIGEST_LENGTH];
        CC_SHA256(tangents.bytes, static_cast<CC_LONG>(tangents.length), digest);
        NSMutableString *identifier = [NSMutableString stringWithFormat:@"basis%u-",
            static_cast<unsigned>(value.tangentBasis)];
        for (unsigned char byte : digest) { [identifier appendFormat:@"%02x", byte]; }
        tangentIdentifier = identifier;
    }
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
                           faceCount:value.topology.faceCount
                           edgeCount:value.topology.edgeCount
                  topologyVertexCount:value.topology.vertexCount
                   cornerTangentData:tangents
                   tangentIdentifier:tangentIdentifier
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
    const ReferenceAxisSnapshot referenceAxis = value.referenceAxis.value_or(
        ReferenceAxisSnapshot());
    return [[Core3DSceneRenderItemSnapshot alloc]
        initWithEntityIdentifier:StringFromUTF8(value.entityIdentifier)
                       meshIndex:value.meshIndex
                  worldTransform:MatrixFromScene(value.worldFromObject)
                hasReferenceAxis:value.referenceAxis.has_value()
             referencePivotWorld:Double3FromScene(referenceAxis.worldPivot)
         referenceDirectionWorld:Double3FromScene(
             referenceAxis.worldDirection)
             referencePivotSpace:ReferenceSpaceFromScene(
                 referenceAxis.pivotSpace)
         referenceDirectionSpace:ReferenceSpaceFromScene(
             referenceAxis.directionSpace)
           referenceAxisAuthored:referenceAxis.authored
                         winding:value.reversesWinding
                             ? Core3DSceneWindingReversed
                             : Core3DSceneWindingAsDefined
                         visible:value.visible
                      selectable:value.selectable
                        selected:value.selected
                            name:StringFromUTF8(value.name)
                 groupIdentifier:StringFromUTF8(value.groupIdentifier)
                       groupName:StringFromUTF8(value.groupName)
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

bool core3d::scene::IsValidSceneSnapshot(
    const SceneSnapshot& snapshot) noexcept {
    return ValidateSceneSnapshotPayload(snapshot);
}

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
        NSArray<Core3DSceneTextureSnapshot *> *textures =
            ObjectArrayFromVector<TextureResourceSnapshot,
                                  Core3DSceneTextureSnapshot>(
                snapshot.textures,
                TextureFromScene);
        NSArray<Core3DSceneElementIdentifier *> *pickTable =
            ObjectArrayFromVector<ElementIdentifier, Core3DSceneElementIdentifier>(
                snapshot.pickTable,
                ElementIdentifierFromScene);

        return [[Core3DSceneSnapshot alloc]
            initWithSchemaVersion:snapshot.schemaVersion
            publicationSourceIdentifier:StringFromUTF8(
                snapshot.publicationSourceIdentifier)
                        revisions:RevisionVectorFromScene(snapshot.revisions)
                    metersPerUnit:snapshot.metersPerUnit
                     renderOrigin:Double3FromScene(snapshot.renderOrigin)
                    selectionMode:ElementKindFromScene(snapshot.selectionMode)
                           meshes:meshes
                      renderItems:renderItems
                        materials:materials
                         textures:textures
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
