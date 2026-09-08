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
#include <BRep_Builder.hxx>
#include <TopoDS.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <TDF_LabelSequence.hxx>
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
        Handle(Poly_Triangulation) mesh = new Poly_Triangulation(4, 1, Standard_True, Standard_True);
        mesh->SetNode(1, gp_Pnt(9, -0.0, 3)); // Deliberately unused, still authoritative.
        mesh->SetNode(2, gp_Pnt(0, 0, 0)); mesh->SetNode(3, gp_Pnt(0, 0, 10));
        mesh->SetNode(4, gp_Pnt(8, -6, 0));
        mesh->SetUVNode(1, gp_Pnt2d(-0.0, 0.25)); mesh->SetUVNode(2, gp_Pnt2d(0, 0));
        mesh->SetUVNode(3, gp_Pnt2d(1, 0)); mesh->SetUVNode(4, gp_Pnt2d(0, 1));
        for (int n = 1; n <= 4; ++n) mesh->SetNormal(n, gp_Vec3f(0.6f, 0.8f, -0.0f));
        mesh->SetTriangle(1, Poly_Triangle(2, 3, 4));
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
