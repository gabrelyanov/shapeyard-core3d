#include "../Scene/MikkTangentSpace.hpp"
#if DEBUG
#include "../OCCTKit/Core3DBoundedAuthoredFrameDriver.hxx"
#include "../OCCTKit/Core3DNativeTangentBuffers.hxx"
#include <sstream>
#endif
//
//  Core3DViewController.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 28.03.2024.
//

#import "GLViewController.h"
#import "Core3DViewController.h"
#import "Core3DViewController+AvailabilityManager.h"
#import "Core3DViewController+PrimitiveManager.h"
#import "Core3DViewController+GLViewControllerProtocol.h"
#import <Core3D/AssetBundle.h>
#import "../Viewport/Core3DSceneSnapshotFactory.hpp"
#import "Core3DTransformInspectorSnapshotFactory.hpp"

#include "GLViewController+Trick.h"
#include "BooleanOperationController.hpp"
#include "OrdinaryEditCommand.hpp"
#include "OrdinaryEditController.hpp"
#include "TransformInspectorMeasurementController.hpp"
#include "../OCCTKit/OcctDocument.h"
#include "../Common/dispatch_cancelable_block.h"
#include "XCAFDoc_DocumentTool.hxx"
#include "XCAFDoc_ColorTool.hxx"
#include "XCAFDoc_LayerTool.hxx"
#include "XCAFDoc_VisMaterial.hxx"
#include "XCAFDoc_VisMaterialTool.hxx"
#include "Image_Texture.hxx"
#include <Image_PixMap.hxx>
#include <XCAFPrs_Texture.hxx>
#include <Graphic3d_TextureSet.hxx>
#include <Graphic3d_TextureParams.hxx>
#include "BRep_Tool.hxx"
#include "BRepCheck_Analyzer.hxx"
#include "BRepTools.hxx"
#include "BRepPrimAPI_MakeBox.hxx"
#include "BRepPrimAPI_MakePrism.hxx"
#include "BRepBuilderAPI_MakePolygon.hxx"
#include "BRepBuilderAPI_MakeFace.hxx"
#include "BRepBuilderAPI_Transform.hxx"
#include "BRepBuilderAPI_Copy.hxx"
#include "BRep_Builder.hxx"
#include "TopoDS_Compound.hxx"
#include "TopoDS_CompSolid.hxx"
#include "TopoDS.hxx"
#include "TopoDS_Face.hxx"
#include "TopoDS_Shell.hxx"
#include "TopoDS_Solid.hxx"
#include "TopoDS_Iterator.hxx"
#include "Poly_Triangle.hxx"
#include "Poly_ListOfTriangulation.hxx"
#include "Poly_Triangulation.hxx"
#include "Standard_ErrorHandler.hxx"
#include "Standard_Failure.hxx"
#include "NCollection_Buffer.hxx"
#include "TDataStd_Integer.hxx"
#include "TDataStd_AsciiString.hxx"
#include "TDataStd_Name.hxx"
#include "TDataStd_Real.hxx"
#include "TDF_LabelSequence.hxx"
#include "Standard_GUID.hxx"
#include "gp_Ax2.hxx"
#include "gp_Dir.hxx"
#include "gp_Pnt2d.hxx"
#include "gp_Quaternion.hxx"
#include "gp_Trsf.hxx"
#include "gp_Vec.hxx"
#include "TopLoc_Location.hxx"
#include "TopExp_Explorer.hxx"
#include "XCAFPrs_DocumentExplorer.hxx"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

bool Core3DTryBooleanActionForGizmo(
    const PrimitiveGizmoType type,
    core3d::BooleanAction& action) noexcept {
    switch (type) {
        case PrimitiveGizmoTypeSubtract:
            action = core3d::BooleanAction::BooleanSubtract;
            return true;
        case PrimitiveGizmoTypeUnion:
            action = core3d::BooleanAction::BooleanUnion;
            return true;
        case PrimitiveGizmoTypeIntersect:
            action = core3d::BooleanAction::BooleanIntersect;
            return true;
        default:
            return false;
    }
}

bool Core3DIsModelingOperationGizmo(
    const PrimitiveGizmoType type) noexcept {
    switch (type) {
        case PrimitiveGizmoTypeChamfer:
        case PrimitiveGizmoTypeSubtract:
        case PrimitiveGizmoTypeUnion:
        case PrimitiveGizmoTypeMirror:
        case PrimitiveGizmoTypeExtrude:
        case PrimitiveGizmoTypeIntersect:
        case PrimitiveGizmoTypeLinearArray:
        case PrimitiveGizmoTypeShell:
        case PrimitiveGizmoTypeRadialArray:
            return true;
        case PrimitiveGizmoTypeNone:
        case PrimitiveGizmoTypeMoveRotate:
        case PrimitiveGizmoTypeScale:
        case PrimitiveGizmoTypeMaterial:
            return false;
    }
    return false;
}

bool Core3DSelectionTypeAllowsGizmo(
    const PrimitiveSelectionType selectionType,
    const PrimitiveGizmoType gizmoType) noexcept {
    if (gizmoType == PrimitiveGizmoTypeNone) {
        return true;
    }
    switch (selectionType) {
        case PrimitiveSelectionTypeShape:
            switch (gizmoType) {
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
        case PrimitiveSelectionTypeFace:
            return gizmoType == PrimitiveGizmoTypeChamfer
                || gizmoType == PrimitiveGizmoTypeExtrude
                || gizmoType == PrimitiveGizmoTypeShell;
        case PrimitiveSelectionTypeEdge:
            return gizmoType == PrimitiveGizmoTypeChamfer;
        case PrimitiveSelectionTypeNone:
        case PrimitiveSelectionTypeVertex:
            return false;
    }
    return false;
}

void Core3DAbortCommandNoThrow(
    const Handle(TDocStd_Document)& document) noexcept {
    try {
        if (!document.IsNull() && document->HasOpenCommand()) {
            document->AbortCommand();
        }
    } catch (...) {
    }
}

NSString* const Core3DTextureAuthoringErrorDomain =
    @"Core3DTextureAuthoringError";

typedef NS_ENUM(NSInteger, Core3DTextureAuthoringErrorCode) {
    Core3DTextureAuthoringErrorInvalidInput = 1,
    Core3DTextureAuthoringErrorUnavailable,
    Core3DTextureAuthoringErrorNoSelection,
    Core3DTextureAuthoringErrorUnsupportedSelection,
    Core3DTextureAuthoringErrorMissingTextureCoordinates,
    Core3DTextureAuthoringErrorTransactionFailed,
};

BOOL Core3DTextureAuthoringFailure(
    NSError* _Nullable * _Nullable error,
    const Core3DTextureAuthoringErrorCode code,
    NSString* description) {
    if (error != nullptr) {
        *error = [NSError errorWithDomain:Core3DTextureAuthoringErrorDomain
                                     code:code
                                 userInfo:@{NSLocalizedDescriptionKey:
                                     description ?: @"The texture edit failed."}];
    }
    return NO;
}

constexpr Standard_Size kMaximumTextureAuthoringFaces = 4096;
constexpr Standard_Size kMaximumTextureAuthoringNodes = 1000000;
// Matches the immutable snapshot's 3,000,000-index per-mesh ceiling.
constexpr Standard_Size kMaximumTextureAuthoringTriangles = 1000000;
constexpr Standard_Size kMaximumTextureAuthoringObjects = 256;
constexpr Standard_Real kMaximumTextureCoordinateMagnitude = 1.0e9;

struct Core3DTextureAuthoringBudget {
    Standard_Size maximumObjects = kMaximumTextureAuthoringObjects;
    Standard_Size objects = 0;
    Standard_Size faces = 0;
    Standard_Size nodes = 0;
    Standard_Size triangles = 0;
};

bool Core3DHasCompleteCachedTextureCoordinates(
    const TopoDS_Shape& shape,
    Core3DTextureAuthoringBudget& budget) {
    if (shape.IsNull() || budget.maximumObjects == 0
        || budget.maximumObjects > kMaximumTextureAuthoringObjects
        || budget.objects >= budget.maximumObjects) {
        return false;
    }
    ++budget.objects;
    Standard_Size objectFaceCount = 0;
    for (TopExp_Explorer faces(shape, TopAbs_FACE);
         faces.More(); faces.Next()) {
        if (++objectFaceCount > kMaximumTextureAuthoringFaces
            || budget.faces >= kMaximumTextureAuthoringFaces) {
            return false;
        }
        ++budget.faces;
        const TopoDS_Face face = TopoDS::Face(faces.Current());
        TopLoc_Location location;
        const Handle(Poly_Triangulation)& triangulation =
            BRep_Tool::Triangulation(face, location);
        if (triangulation.IsNull() || !triangulation->HasGeometry()
            || !triangulation->HasUVNodes()
            || triangulation->NbNodes() <= 0
            || triangulation->NbTriangles() <= 0) {
            // Deliberately do not invoke a mesher here. Texture assignment is
            // a presentation edit and must never hide an unbounded geometry
            // rebuild on the main thread.
            return false;
        }
        const Standard_Size nodes =
            static_cast<Standard_Size>(triangulation->NbNodes());
        const Standard_Size triangles =
            static_cast<Standard_Size>(triangulation->NbTriangles());
        if (budget.nodes > kMaximumTextureAuthoringNodes
            || nodes > kMaximumTextureAuthoringNodes - budget.nodes
            || budget.triangles > kMaximumTextureAuthoringTriangles
            || triangles
                > kMaximumTextureAuthoringTriangles - budget.triangles) {
            return false;
        }
        budget.nodes += nodes;
        budget.triangles += triangles;
        for (Standard_Integer node = 1;
             node <= triangulation->NbNodes(); ++node) {
            const gp_Pnt2d uv = triangulation->UVNode(node);
            if (!std::isfinite(uv.X()) || !std::isfinite(uv.Y())
                || std::abs(uv.X()) > kMaximumTextureCoordinateMagnitude
                || std::abs(uv.Y()) > kMaximumTextureCoordinateMagnitude) {
                return false;
            }
        }
        bool hasUsableUVArea = false;
        for (Standard_Integer triangle = 1;
             triangle <= triangulation->NbTriangles(); ++triangle) {
            Standard_Integer indices[3] = {0, 0, 0};
            triangulation->Triangle(triangle).Get(
                indices[0], indices[1], indices[2]);
            for (const Standard_Integer index : indices) {
                if (index < 1 || index > triangulation->NbNodes()) {
                    return false;
                }
            }
            const gp_Pnt2d uv0 = triangulation->UVNode(indices[0]);
            const gp_Pnt2d uv1 = triangulation->UVNode(indices[1]);
            const gp_Pnt2d uv2 = triangulation->UVNode(indices[2]);
            const Standard_Real twiceArea =
                (uv1.X() - uv0.X()) * (uv2.Y() - uv0.Y())
                - (uv1.Y() - uv0.Y()) * (uv2.X() - uv0.X());
            if (!std::isfinite(twiceArea)) {
                return false;
            }
            hasUsableUVArea = hasUsableUVArea
                || std::abs(twiceArea) > 0.0;
        }
        if (!hasUsableUVArea) {
            return false;
        }
    }
    return objectFaceCount > 0;
}

Core3DPBRMaterial* Core3DMakePBRMaterial(
    const XCAFDoc_VisMaterialPBR& material,
    const BOOL supportsScalarEditing,
    const BOOL supportsBaseColorTextureEditing,
    const BOOL supportsEmissiveTextureEditing,
    const BOOL supportsMetallicRoughnessTextureEditing,
    const BOOL supportsOcclusionTextureEditing,
    const BOOL supportsNormalTextureEditing) {
    if (!material.IsDefined) {
        return nil;
    }
    Standard_Real red = 0.0;
    Standard_Real green = 0.0;
    Standard_Real blue = 0.0;
    material.BaseColor.GetRGB().Values(
        red, green, blue, Quantity_TOC_sRGB);
    if (!std::isfinite(red) || !std::isfinite(green)
        || !std::isfinite(blue)) {
        return nil;
    }
    UIColor* color = [UIColor colorWithRed:red
                                     green:green
                                      blue:blue
                                     alpha:material.BaseColor.Alpha()];
    return [[Core3DPBRMaterial alloc]
        initWithBaseColor:color
                 metallic:material.Metallic
                roughness:material.Roughness
    supportsScalarEditing:supportsScalarEditing
      hasBaseColorTexture:!material.BaseColorTexture.IsNull()
supportsBaseColorTextureEditing:supportsBaseColorTextureEditing
      hasEmissiveTexture:!material.EmissiveTexture.IsNull()
supportsEmissiveTextureEditing:supportsEmissiveTextureEditing
hasMetallicRoughnessTexture:!material.MetallicRoughnessTexture.IsNull()
supportsMetallicRoughnessTextureEditing:supportsMetallicRoughnessTextureEditing
hasOcclusionTexture:!material.OcclusionTexture.IsNull()
supportsOcclusionTextureEditing:supportsOcclusionTextureEditing
hasNormalTexture:!material.NormalTexture.IsNull()
supportsNormalTextureEditing:supportsNormalTextureEditing];
}

bool Core3DApplyNativePBRScalars(Core3DPBRMaterial* source,
                                 XCAFDoc_VisMaterialPBR& result) {
    if (source == nil || !std::isfinite(source.metallic)
        || !std::isfinite(source.roughness)
        || source.metallic < 0.0 || source.metallic > 1.0
        || source.roughness < 0.0 || source.roughness > 1.0) {
        return false;
    }
    CGFloat red = 0.0;
    CGFloat green = 0.0;
    CGFloat blue = 0.0;
    CGFloat alpha = 0.0;
    if (![source.baseColor getRed:&red green:&green blue:&blue alpha:&alpha]
        || !std::isfinite(red) || !std::isfinite(green)
        || !std::isfinite(blue) || !std::isfinite(alpha)
        || alpha < 0.0 || alpha > 1.0) {
        return false;
    }
    // UIColorWell may return Display-P3 / extended-sRGB components outside
    // the persistable [0, 1] material gamut. Explicit clipping gives those
    // legitimate colors a deterministic sRGB representation instead of a
    // silent no-op.
    red = std::clamp(red, static_cast<CGFloat>(0.0),
                    static_cast<CGFloat>(1.0));
    green = std::clamp(green, static_cast<CGFloat>(0.0),
                      static_cast<CGFloat>(1.0));
    blue = std::clamp(blue, static_cast<CGFloat>(0.0),
                     static_cast<CGFloat>(1.0));
    const Standard_ShortReal preservedAlpha = result.IsDefined
        ? result.BaseColor.Alpha()
        : static_cast<Standard_ShortReal>(alpha);
    if (!std::isfinite(preservedAlpha)
        || preservedAlpha < 0.0f || preservedAlpha > 1.0f) {
        return false;
    }
    result.BaseColor = Quantity_ColorRGBA(
        Quantity_Color(red, green, blue, Quantity_TOC_sRGB),
        preservedAlpha);
    result.Metallic = static_cast<Standard_ShortReal>(source.metallic);
    result.Roughness = static_cast<Standard_ShortReal>(source.roughness);
    result.IsDefined = Standard_True;
    return true;
}

XCAFDoc_VisMaterialPBR Core3DLegacyPBRMaterial(
    const Graphic3d_NameOfMaterial materialName,
    const Quantity_NameOfColor colorName) {
    const Graphic3d_MaterialAspect aspect(materialName);
    const Graphic3d_PBRMaterial& preset = aspect.PBRMaterial();
    XCAFDoc_VisMaterialPBR result;
    result.BaseColor = Quantity_ColorRGBA(
        Quantity_Color(colorName), preset.Alpha());
    result.EmissiveFactor = preset.Emission();
    result.Metallic = preset.Metallic();
    result.Roughness = preset.NormalizedRoughness();
    result.RefractionIndex = preset.IOR();
    result.IsDefined = Standard_True;
    return result;
}

NSData* Core3DCreateDebugBinXCAFFixture(
    NSString* suffix,
    const std::function<void(const Handle(TDocStd_Document)&)>& populate) {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL* baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.%@", NSUUID.UUID.UUIDString, suffix]];
    NSString* xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSData* result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        if (document.IsNull()) {
            throw Standard_Failure("Unable to create debug XCAF fixture");
        }
        XCAFDoc_DocumentTool::SetLengthUnit(document, 0.001);
        populate(document);
        if (application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure("Unable to save debug XCAF fixture");
        }
        result = [NSData dataWithContentsOfFile:xbfPath];
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return result;
}

#ifdef DEBUG
const Standard_GUID& Core3DDebugGeometryRepresentationAttributeID() {
    // This duplicate is deliberate test evidence for the persistent schema:
    // valid fixtures stop loading if production ever changes the GUID.
    static const Standard_GUID identifier(
        "67E669F4-00C0-4C45-BC55-9CC5DA22A2B5");
    return identifier;
}

const std::array<Standard_GUID, 7>& Core3DDebugReferenceAxisAttributeIDs() {
    // These duplicates deliberately lock the serialized production GUIDs.
    static const std::array<Standard_GUID, 7> identifiers = {{
        Standard_GUID("26128380-D69C-4530-B856-0C2AEEF47E60"),
        Standard_GUID("11531A1D-14DB-4F78-AD91-814981846842"),
        Standard_GUID("BA2AE804-64BD-480B-8910-B1144DA1AAD3"),
        Standard_GUID("7CDD4B6E-5375-48F2-BAA9-AD76ACF6D47A"),
        Standard_GUID("CCEC34C3-8D3A-447A-8C70-EDB35A5B7E1C"),
        Standard_GUID("1DDBE964-693E-460A-9095-E47713BA28C0"),
        Standard_GUID("09CC9F05-9628-4C84-B214-2676C8BED8AA"),
    }};
    return identifiers;
}

const Standard_GUID& Core3DDebugDuplicateCommandOwnerAttributeID() {
    // Deliberately locks the private persistent recovery GUID in fixtures.
    static const Standard_GUID identifier(
        "D7598D08-A879-4E17-8D23-CC92568EAD5C");
    return identifier;
}

const Standard_GUID& Core3DDebugRadialArrayCommandOwnerAttributeID() {
    // Deliberately locks the private persistent recovery GUID in fixtures.
    static const Standard_GUID identifier(
        "2DF9F8BE-F297-4CFD-8D93-A40346EBEF3A");
    return identifier;
}

void Core3DWriteDebugReferenceAxisRecord(
    const TDF_Label& label,
    const Core3DDebugReferenceAxisFixtureMode mode) {
    if (label.IsNull()) {
        throw Standard_Failure("Reference-axis fixture label is null");
    }
    const auto& identifiers = Core3DDebugReferenceAxisAttributeIDs();
    if (mode == Core3DDebugReferenceAxisFixturePartialRecord) {
        TDataStd_Integer::Set(label, identifiers[0], 0x0102);
        return;
    }

    if (mode == Core3DDebugReferenceAxisFixtureWrongModeType) {
        TDataStd_Real::Set(label, identifiers[0], 258.0);
    } else {
        TDataStd_Integer::Set(
            label,
            identifiers[0],
            mode == Core3DDebugReferenceAxisFixtureUnknownMode
                ? 0x0202 : 0x0102);
    }

    Standard_Real values[6] = {1.0, 2.0, 3.0, 0.0, 0.6, 0.8};
    switch (mode) {
        case Core3DDebugReferenceAxisFixtureNonFinitePivot:
            values[0] = std::numeric_limits<Standard_Real>::quiet_NaN();
            break;
        case Core3DDebugReferenceAxisFixtureOversizedPivot:
            values[0] = 1'000'001.0;
            break;
        case Core3DDebugReferenceAxisFixtureZeroDirection:
            values[3] = 0.0;
            values[4] = 0.0;
            values[5] = 0.0;
            break;
        case Core3DDebugReferenceAxisFixtureNonUnitDirection:
            values[3] = 0.0;
            values[4] = 3.0;
            values[5] = 4.0;
            break;
        default:
            break;
    }
    for (std::size_t index = 0; index < 6U; ++index) {
        TDataStd_Real::Set(label, identifiers[index + 1U], values[index]);
    }
}

TDF_Label Core3DFirstFreeSimpleDefinition(
    const Handle(TDocStd_Document)& document) {
    if (document.IsNull()
        || !XCAFDoc_DocumentTool::CheckShapeTool(document->Main())) {
        return {};
    }
    const Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(document->Main());
    if (shapeTool.IsNull()) {
        return {};
    }
    TDF_LabelSequence labels;
    shapeTool->GetFreeShapes(labels);
    for (Standard_Integer index = 1; index <= labels.Length(); ++index) {
        const TDF_Label& label = labels.Value(index);
        if (!label.IsNull() && shapeTool->IsShape(label)
            && XCAFDoc_ShapeTool::IsFree(label)
            && XCAFDoc_ShapeTool::IsSimpleShape(label)
            && !XCAFDoc_ShapeTool::IsReference(label)
            && !XCAFDoc_ShapeTool::IsComponent(label)
            && !XCAFDoc_ShapeTool::IsAssembly(label)
            && !XCAFDoc_ShapeTool::IsSubShape(label)) {
            return label;
        }
    }
    return {};
}

TopoDS_Face Core3DMakeDebugTriangleMeshFace(
    const Standard_Real translationX = 0.0,
    const Standard_Boolean includeUnreferencedOutlier = Standard_False) {
    const Standard_Integer nodeCount =
        includeUnreferencedOutlier ? 4 : 3;
    Handle(Poly_Triangulation) triangulation =
        new Poly_Triangulation(
            nodeCount, 1, Standard_True, Standard_True);
    if (triangulation.IsNull()) {
        throw Standard_Failure("Unable to allocate triangle fixture");
    }
    triangulation->SetNode(
        1, gp_Pnt(translationX - 10.0, -10.0, 0.0));
    triangulation->SetNode(
        2, gp_Pnt(translationX + 10.0, -10.0, 0.0));
    triangulation->SetNode(
        3, gp_Pnt(translationX, 10.0, 0.0));
    triangulation->SetUVNode(1, gp_Pnt2d(0.0, 0.0));
    triangulation->SetUVNode(2, gp_Pnt2d(1.0, 0.0));
    triangulation->SetUVNode(3, gp_Pnt2d(0.5, 1.0));
    if (includeUnreferencedOutlier) {
        triangulation->SetNode(
            4, gp_Pnt(translationX + 1'000.0, 0.0, 0.0));
        triangulation->SetUVNode(4, gp_Pnt2d(0.5, 0.5));
    }
    for (Standard_Integer node = 1; node <= nodeCount; ++node) {
        triangulation->SetNormal(node, gp_Dir(0.0, 0.0, 1.0));
    }
    triangulation->SetTriangle(1, Poly_Triangle(1, 2, 3));
    triangulation->Deflection(0.1);

    BRep_Builder builder;
    TopoDS_Face face;
    builder.MakeFace(face, triangulation);
    if (face.IsNull() || !BRep_Tool::Surface(face).IsNull()) {
        throw Standard_Failure(
            "Triangle fixture unexpectedly owns an analytic surface");
    }
    return face;
}

TopoDS_Compound Core3DMakeDebugLocatedTriangleMeshDefinition() {
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);

    TopoDS_Face face = Core3DMakeDebugTriangleMeshFace();
    gp_Trsf transform;
    transform.SetTranslation(gp_Vec(25.0, 0.0, 0.0));
    const TopLoc_Location location(transform);
    if (location.IsIdentity()) {
        throw Standard_Failure(
            "Located triangle fixture unexpectedly has identity location");
    }
    face.Location(location);
    if (face.Location().IsIdentity()) {
        throw Standard_Failure(
            "Unable to assign triangle fixture face location");
    }
    // Keep the definition root at identity: only the authoritative face
    // triangulation owns this location, so a bounds implementation cannot
    // pass by accidentally applying the definition's object transform.
    builder.Add(compound, face);
    return compound;
}

TopoDS_Compound Core3DMakeDebugMixedGeometryDefinition() {
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    builder.Add(
        compound,
        BRepPrimAPI_MakeBox(
            gp_Pnt(-35.0, -10.0, -10.0), 20.0, 20.0, 20.0)
            .Shape());
    builder.Add(compound, Core3DMakeDebugTriangleMeshFace(25.0));
    return compound;
}

TDF_Label Core3DAddDebugGeometryDefinition(
    const Handle(XCAFDoc_ShapeTool)& shapeTool,
    const TopoDS_Shape& shape) {
    if (shapeTool.IsNull() || shape.IsNull()) {
        throw Standard_Failure("Invalid geometry fixture definition");
    }
    const TDF_Label label = shapeTool->AddShape(
        shape, Standard_False, Standard_True);
    if (label.IsNull()) {
        throw Standard_Failure("Unable to add geometry fixture definition");
    }
    return label;
}

void Core3DSetDebugGeometryRepresentation(
    const TDF_Label& label,
    const Standard_Integer rawValue) {
    if (label.IsNull()
        || TDataStd_Integer::Set(
            label,
            Core3DDebugGeometryRepresentationAttributeID(),
            rawValue).IsNull()) {
        throw Standard_Failure("Unable to mark geometry fixture definition");
    }
}

void Core3DSetDebugObjectScale(
    const TDF_Label& label,
    const Standard_Real scale) {
    // Even the deliberately corrupt fixture remains finite and nonzero. This
    // lets the legacy production loader safely construct its gp_Trsf while
    // the inspector's stricter persisted-transform validation rejects it.
    if (label.IsNull() || !std::isfinite(scale) || scale == 0.0
        || TDataStd_Real::Set(
            label.FindChild(8, Standard_True), scale).IsNull()) {
        throw Standard_Failure("Unable to set geometry fixture object scale");
    }
}
#endif

void Core3DAddDebugOrphanVisualMaterial(
    const TDF_Label& label,
    const char* identifier) {
    NSData* png = [[NSData alloc] initWithBase64EncodedString:
        @"iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAYAAACqaXHeAAAApklEQVR42u3aQQ3AQAzEwFyZF3kKY06qTWAtK8+cndmBnJ1X7j9y/AYKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNAXQApoCaAFNAbSApgBaQFMALaApgBbQnJml/wF2vQsoQAG0gKYAWkBTAC2gKYAW0BRAC2gKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNL8P8AESbQf6Ta5RUwAAAABJRU5ErkJggg=="
        options:0];
    Handle(NCollection_Buffer) buffer = new NCollection_Buffer(
        NCollection_BaseAllocator::CommonBaseAllocator(), png.length);
    if (label.IsNull() || identifier == nullptr || *identifier == '\0'
        || png.length == 0 || buffer.IsNull()
        || buffer->ChangeData() == nullptr) {
        throw Standard_Failure(
            "Unable to create orphan visual material resource");
    }
    std::memcpy(buffer->ChangeData(), png.bytes, png.length);
    const Handle(Image_Texture) texture = new Image_Texture(
        buffer, TCollection_AsciiString(identifier));
    XCAFDoc_VisMaterialPBR pbr;
    pbr.BaseColor = Quantity_ColorRGBA(
        Quantity_Color(0.28, 0.52, 0.74, Quantity_TOC_sRGB),
        1.0f);
    pbr.Metallic = 0.3f;
    pbr.Roughness = 0.7f;
    pbr.BaseColorTexture = texture;
    pbr.IsDefined = Standard_True;
    Handle(XCAFDoc_VisMaterial) material =
        new XCAFDoc_VisMaterial();
    material->SetPbrMaterial(pbr);
    XCAFDoc_VisMaterialCommon common =
        material->ConvertToCommonMaterial();
    common.DiffuseTexture = texture;
    material->SetCommonMaterial(common);
    label.AddAttribute(material);
}

} // namespace

@interface Core3DMeshUVAtlasPreview ()
- (instancetype)initWithNativePreview:(const OcctMeshUVAtlasPreview&)preview;
@end
@implementation Core3DMeshUVAtlasPreview
- (instancetype)initWithNativePreview:(const OcctMeshUVAtlasPreview&)preview {
    self=[super init];
    if(self) {
        _triangleUVData=[NSData dataWithBytes:preview.triangleUVs.data() length:preview.triangleUVs.size()*sizeof(double)];
        _chartCount=preview.chartCount;_occupancy=preview.occupancy;
        _authoredResolution=preview.authoredResolution;_authoredGutterPixels=preview.authoredGutterPixels;
    }
    return self;
}
@end

@interface Core3DViewController () {
    BOOL _isSetuped;
    NSURL *_shouldLoadBundleUrl;
    NSURL *_shouldLoadAssetFileURL;
    unsigned long long _shouldLoadAssetByteCount;
    NSString *_shouldLoadAssetSHA256;
    std::atomic_bool _isLoading;
    std::shared_ptr<core3d::ProfileSolidWork> _profileSolidWork;
    BOOL _profileSolidCancelled;
    std::shared_ptr<core3d::ObjectAlignmentWork> _objectAlignmentWork;
    BOOL _objectAlignmentCancelled;
#ifdef DEBUG
    NSUInteger _debugMaximumTextureAuthoringObjects;
#endif
}

- (BOOL)core3d_canBeginCommittedEdit;
- (Core3DMeshUVAtlasPreview *)meshUVPreviewForEntityIdentifier:(NSString *)entityIdentifier
                                                   options:(const std::optional<OcctMeshUVAtlasOptions>&)options
                                                  expected:(Core3DSceneSnapshot *)expected;
- (BOOL)frameCommittedSceneSelectedOnly:(BOOL)selectedObjectsOnly
                                 rect:(CGRect)rect
                       objectIdentity:(const core3d::ObjectFrameIdentity*)identity;

- (BOOL)core3d_updateSelectionWithTextureData:(NSData*)data mediaType:(NSString*)mediaType
    slot:(OcctMaterialTextureSlot)slot error:(NSError* _Nullable * _Nullable)error;
- (BOOL)core3d_clearSelectionTexture:(OcctMaterialTextureSlot)slot
    error:(NSError* _Nullable * _Nullable)error;
@end

@interface Core3DViewController (ProfileConstructionPrivate)
- (void)constructProfileWithPoints:(NSArray<NSValue *> *)points
                            plane:(Core3DProfilePlane)plane parameter:(double)parameter
                          revolve:(BOOL)revolve
                           circle:(const std::optional<core3d::ProfileCircularSection>&)circle
                      holeCenters:(NSArray<NSValue *> *)holeCenters holeRadii:(NSArray<NSNumber *> *)holeRadii
                         expected:(Core3DSceneSnapshot *)expected
                       completion:(void(^)(Core3DProfileConstructionResult))completion;
@end

@interface Core3DViewController (NumericTransformPrivate)
- (Core3DTransformInspectorPositionCommitResult)
    commitTransformInspectorValue:(double)value
                              axis:(Core3DTransformInspectorAxis)axis
                  expectedSnapshot:(Core3DTransformInspectorSnapshot *)snapshot
                          kind:(core3d::TransformInspectorEditKind)kind;
@end

@implementation Core3DViewController {
    __weak dispatch_cancelable_block_t _uiStateChangingBlock;
    UIStateChanging _sendingState;

    __weak dispatch_cancelable_block_t _modifiedAssetBlock;
}

- (void)dealloc {
    core3d::Core3DViewer::cancelObjectAlignment(_objectAlignmentWork);
    core3d::Core3DViewer::cancelProfileSolid(_profileSolidWork);
    [[NSNotificationCenter defaultCenter] removeObserver:self];
    _glController = nil;
    NSLog(@"~Core3DViewController");
}

- (instancetype)init {
    if (self = [super init]) {
        [self commontInit];
    }

    return self;
}

- (instancetype)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil {
    if (self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil]) {
        [self commontInit];
    }

    return self;
}

- (instancetype)initWithCoder:(NSCoder *)coder {
    if (self = [super initWithCoder:coder]) {
        [self commontInit];
    }
    return self;
}

- (void)commontInit {
    if (!_glController) {
        _materialController = [[Core3DMaterialController alloc] init];
        _materialController.pass = (id<Core3DMaterialControllerCalls>)self;
        
        _glController = [[GLViewController alloc] init];
        GLController.delegate = self;

        _currentStateChanging = UIStateChangingNone;

        _currentSelectionType = PrimitiveSelectionTypeShape;
        _currentGizmoType = PrimitiveGizmoTypeNone;

        _can_add = YES;
        _can_redo = YES;
        _can_undo = YES;
        _can_delete = NO;
        _can_duplicate = NO;

        _availableGizmoTypes = @[];
#ifdef DEBUG
        _debugMaximumTextureAuthoringObjects =
            kMaximumTextureAuthoringObjects;
#endif
        
        [[NSNotificationCenter defaultCenter] addObserver:self
            selector:@selector(receiveOcctDocumentNotification:)
            name:@"OcctDocumentChanges"
            object:nil];
    }
}

- (BOOL)core3d_canBeginCommittedEdit {
    return [NSThread isMainThread]
        && GLController != nil
        && GLController.viewer != nullptr
        && GLController.viewer->canBeginCommittedEdit();
}

-(void) updateSelectionWithMaterial:(Core3DMaterial*)material color:(Core3DColor*)color {
    if ((self.selectedModelCapabilities
            & Core3DModelCapabilityMaterial) == 0
        || ![self core3d_canBeginCommittedEdit]) {
        return;
    }
    auto context = GLController.viewer->AisContext();
    auto doc = GLController.viewer->getDocument();
	auto transaction = doc->ChangeDocument();
	if (transaction.IsNull() || transaction->HasOpenCommand()) {
		return;
	}

	struct PendingStyle {
		Handle(AIS_Shape) shape;
		TDF_Label label;
		Graphic3d_NameOfMaterial material;
		Quantity_NameOfColor color;
	};
	std::vector<PendingStyle> pendingStyles;
    NSMutableArray* materials = [NSMutableArray array];
    NSMutableArray* colors = [NSMutableArray array];

    for (context->InitSelected(); context->MoreSelected(); context->NextSelected()) {
        auto selected = context->SelectedInteractive();
        Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(selected);
        if (shape.IsNull() || shape->Shape().IsNull()) {
            continue;
        }
		const TDF_Label label = doc->ShapeLabel(selected);
		if (label.IsNull()) {
			return;
        }

		Graphic3d_NameOfMaterial materialName = doc->MaterialNameForLabel(label);
		Quantity_NameOfColor colorName = doc->ColorNameForLabel(label);
		if (material == nil && color == nil) {
			materialName = Graphic3d_NameOfMaterial_ShinyPlastified;
			colorName = Quantity_NOC_GRAY80;
		} else {
			if (material != nil) {
				materialName = (Graphic3d_NameOfMaterial)material.identity;
				if (color == nil) {
					Graphic3d_MaterialAspect materialAspect(materialName);
					colorName = (Quantity_NameOfColor)materialAspect.Color().Name();
				}
			}
			if (color != nil) {
				colorName = (Quantity_NameOfColor)color.identity;
			}
        }
		pendingStyles.push_back({shape, label, materialName, colorName});

        if(material != nil) {
            [materials addObject:material];
        }
        if(color != nil) {
            [colors addObject:color];
        }
    }
	if (pendingStyles.empty()) {
		return;
	}

	try {
		transaction->NewCommand();
		if (!transaction->HasOpenCommand()) {
			return;
		}
		for (const PendingStyle& style : pendingStyles) {
			if (!doc->ClearObjectVisualMaterial(style.label)) {
				Core3DAbortCommandNoThrow(transaction);
				return;
			}
			doc->SaveObjectMaterial(style.label, style.material);
			doc->SaveObjectColor(style.label, style.color);
		}
		if (!transaction->CommitCommand()) {
			Core3DAbortCommandNoThrow(transaction);
			return;
		}
	} catch (...) {
		Core3DAbortCommandNoThrow(transaction);
		return;
	}
	doc->NotifyChanges();

	for (const PendingStyle& style : pendingStyles) {
		style.shape->UnsetColor();
		doc->LoadObjectMeterial(style.label, style.shape);
	}
	[self.materialController didChangeSelectionWithMaterials:[materials copy] colors:[colors copy]];
	NSMutableArray<Core3DPBRMaterial*>* selectedPBR =
		[NSMutableArray arrayWithCapacity:pendingStyles.size()];
	for (const PendingStyle& style : pendingStyles) {
		const Graphic3d_MaterialAspect presetAspect(style.material);
		const Graphic3d_PBRMaterial& preset = presetAspect.PBRMaterial();
		XCAFDoc_VisMaterialPBR editable;
		editable.BaseColor = Quantity_ColorRGBA(
			Quantity_Color(style.color), preset.Alpha());
		editable.EmissiveFactor = preset.Emission();
		editable.Metallic = preset.Metallic();
		editable.Roughness = preset.NormalizedRoughness();
		editable.RefractionIndex = preset.IOR();
			Core3DPBRMaterial* pbr = Core3DMakePBRMaterial(
	            editable, YES, YES, YES, YES, YES,
                doc->SupportsNormalTextureGeometryForLabel(style.label));
		if (pbr != nil) {
			[selectedPBR addObject:pbr];
		}
	}
	[self.materialController didChangeSelectionWithPBRMaterials:selectedPBR];
    context->UpdateCurrentViewer();
    [self sendNotifyUIState: UIStateChangingApplyMaterial
                            | UIStateChangingHistory];
}

-(void) updateSelectionWithPBRMaterial:(Core3DPBRMaterial*)material {
    if (material == nil || !material.supportsScalarEditing) {
        return;
    }
	if ((self.selectedModelCapabilities
			& Core3DModelCapabilityMaterial) == 0
		|| ![self core3d_canBeginCommittedEdit]) {
		return;
	}

    auto context = GLController.viewer->AisContext();
    auto doc = GLController.viewer->getDocument();
    auto transaction = doc->ChangeDocument();
    if (transaction.IsNull() || transaction->HasOpenCommand()) {
        return;
    }

    struct PendingPBRStyle {
        Handle(AIS_Shape) shape;
        TDF_Label label;
        XCAFDoc_VisMaterialPBR material;
        Handle(Image_Texture) prevalidatedBaseColorTexture;
        Handle(Image_Texture) prevalidatedEmissiveTexture;
    };
    std::vector<PendingPBRStyle> pendingStyles;
    std::unordered_set<std::string> validatedTextureIdentifiers;
    for (context->InitSelected(); context->MoreSelected(); context->NextSelected()) {
        const Handle(AIS_InteractiveObject) selected =
            context->SelectedInteractive();
        const Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(selected);
        if (shape.IsNull() || shape->Shape().IsNull()) {
            continue;
        }
        const TDF_Label label = doc->ShapeLabel(selected);
        if (label.IsNull()) {
            return;
        }
        if (!doc->SupportsScalarPBRMaterialEditingForLabel(label)) {
            return;
        }
        XCAFDoc_VisMaterialPBR nativeMaterial;
        if (!doc->TryEffectivePBRMaterialForLabel(
                label, nativeMaterial)) {
            nativeMaterial = Core3DLegacyPBRMaterial(
                doc->MaterialNameForLabel(label),
                doc->ColorNameForLabel(label));
        }
        if (!Core3DApplyNativePBRScalars(
                material, nativeMaterial)) {
            return;
        }
        Handle(Image_Texture) validatedBaseColorTexture;
        if (!nativeMaterial.BaseColorTexture.IsNull()) {
            const std::string identifier(
                nativeMaterial.BaseColorTexture->TextureId().ToCString());
            if (identifier.empty()) {
                return;
            }
            if (validatedTextureIdentifiers.insert(identifier).second
                && !Core3DValidateAuthoredTexture(
                    nativeMaterial.BaseColorTexture)) {
                return;
            }
            validatedBaseColorTexture = nativeMaterial.BaseColorTexture;
        }
        Handle(Image_Texture) validatedEmissiveTexture;
        if (!nativeMaterial.EmissiveTexture.IsNull()) {
            const std::string identifier(
                nativeMaterial.EmissiveTexture->TextureId().ToCString());
            if (identifier.empty()) {
                return;
            }
            if (validatedTextureIdentifiers.insert(identifier).second
                && !Core3DValidateAuthoredTexture(
                    nativeMaterial.EmissiveTexture)) {
                return;
            }
            validatedEmissiveTexture = nativeMaterial.EmissiveTexture;
        }
        pendingStyles.push_back({
            shape, label, nativeMaterial, validatedBaseColorTexture,
            validatedEmissiveTexture});
    }
    if (pendingStyles.empty()) {
        return;
    }

    try {
        transaction->NewCommand();
        if (!transaction->HasOpenCommand()) {
            return;
        }
        std::vector<OcctPBRMaterialUpdate> materialUpdates;
        materialUpdates.reserve(pendingStyles.size());
        for (const PendingPBRStyle& style : pendingStyles) {
            materialUpdates.push_back({
                style.label,
                style.material,
                style.prevalidatedBaseColorTexture,
                style.prevalidatedEmissiveTexture});
        }
        if (!doc->SaveObjectPBRMaterials(materialUpdates)) {
            Core3DAbortCommandNoThrow(transaction);
            return;
        }
        if (!transaction->CommitCommand()) {
            Core3DAbortCommandNoThrow(transaction);
            return;
        }
    } catch (...) {
        Core3DAbortCommandNoThrow(transaction);
        return;
    }

    doc->NotifyChanges();
    for (const PendingPBRStyle& style : pendingStyles) {
        style.shape->UnsetColor();
        doc->LoadObjectMeterial(style.label, style.shape);
    }

    NSMutableArray<Core3DPBRMaterial*>* selectedPBR =
        [NSMutableArray arrayWithCapacity:pendingStyles.size()];
    for (const PendingPBRStyle& style : pendingStyles) {
        Core3DPBRMaterial* publishedMaterial =
            Core3DMakePBRMaterial(
                style.material,
                doc->SupportsScalarPBRMaterialEditingForLabel(style.label),
                doc->SupportsBaseColorTextureEditingForLabel(style.label),
                doc->SupportsEmissiveTextureEditingForLabel(style.label),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::MetallicRoughness),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::Occlusion),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::Normal));
        if (publishedMaterial != nil) {
            [selectedPBR addObject:publishedMaterial];
        }
    }
    [self.materialController didChangeSelectionWithMaterials:@[] colors:@[]];
    [self.materialController didChangeSelectionWithPBRMaterials:selectedPBR];
    context->UpdateCurrentViewer();
    [self sendNotifyUIState:UIStateChangingApplyMaterial
                           | UIStateChangingHistory];
}

-(BOOL)updateSelectionWithBaseColorTextureData:(NSData*)textureData
                                     mediaType:(NSString*)mediaType
                                         error:(NSError* _Nullable * _Nullable)error {
    if (error != nullptr) {
        *error = nil;
    }
    if (![NSThread isMainThread] || textureData == nil
        || mediaType == nil || textureData.length == 0
        || textureData.bytes == nullptr || mediaType.UTF8String == nullptr) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorInvalidInput,
            @"Choose a valid PNG or JPEG image.");
    }

    Handle(Image_Texture) authoredTexture;
    if (!Core3DCreateAuthoredTexture(
            static_cast<const Standard_Byte*>(textureData.bytes),
            static_cast<Standard_Size>(textureData.length),
            std::string(mediaType.UTF8String), authoredTexture)) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorInvalidInput,
            @"The image is invalid or exceeds the texture safety limits.");
    }
    if (GLController == nil || GLController.viewer == nullptr) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnavailable,
            @"The material editor is not ready.");
    }

    auto context = GLController.viewer->AisContext();
    auto doc = GLController.viewer->getDocument();
    if (context.IsNull() || doc.IsNull()) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnavailable,
            @"The material editor is not ready.");
    }
    if ((self.selectedModelCapabilities
            & Core3DModelCapabilityMaterial) == 0) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnsupportedSelection,
            @"Select only editable model objects before editing a texture.");
    }
    auto transaction = doc->ChangeDocument();
    if (![self core3d_canBeginCommittedEdit]
        || transaction.IsNull() || transaction->HasOpenCommand()) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnavailable,
            @"Finish the current modeling operation before editing a texture.");
    }

    struct PendingTextureStyle {
        Handle(AIS_Shape) shape;
        TDF_Label label;
        XCAFDoc_VisMaterialPBR material;
        Handle(Image_Texture) prevalidatedEmissiveTexture;
        bool changed = false;
    };
    std::vector<PendingTextureStyle> pendingStyles;
    Core3DTextureAuthoringBudget authoringBudget;
    std::unordered_map<std::string, bool>
        identicalExistingTextureByIdentifier;
#ifdef DEBUG
    authoringBudget.maximumObjects =
        static_cast<Standard_Size>(_debugMaximumTextureAuthoringObjects);
#endif
    bool hasChanges = false;
    for (context->InitSelected(); context->MoreSelected();
         context->NextSelected()) {
        const Handle(AIS_InteractiveObject) selected =
            context->SelectedInteractive();
        const Handle(AIS_Shape) shape =
            Handle(AIS_Shape)::DownCast(selected);
        if (shape.IsNull() || shape->Shape().IsNull()
            || !doc->IsPresentationEditable(selected)) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorUnsupportedSelection,
                @"Textures can be edited only on whole editable objects.");
        }
        const TDF_Label label = doc->ShapeLabel(selected);
        if (label.IsNull()
            || !doc->IsEditableFreeSimpleDefinitionLabel(label)
            || !doc->SupportsBaseColorTextureEditingForLabel(label)) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorUnsupportedSelection,
                @"This material contains texture maps that cannot be edited safely.");
        }
        if (!Core3DHasCompleteCachedTextureCoordinates(
                shape->Shape(), authoringBudget)) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorMissingTextureCoordinates,
                @"Every selected face needs existing usable texture coordinates.");
        }

        XCAFDoc_VisMaterialPBR nativeMaterial;
        if (!doc->TryEffectivePBRMaterialForLabel(
                label, nativeMaterial)) {
            nativeMaterial = Core3DLegacyPBRMaterial(
                doc->MaterialNameForLabel(label),
                doc->ColorNameForLabel(label));
        }
        bool hasIdenticalTexture = false;
        if (!nativeMaterial.BaseColorTexture.IsNull()) {
            const std::string existingIdentifier(
                nativeMaterial.BaseColorTexture->TextureId().ToCString());
            const auto cached = identicalExistingTextureByIdentifier.find(
                existingIdentifier);
            if (cached != identicalExistingTextureByIdentifier.end()) {
                hasIdenticalTexture = cached->second;
            } else {
                hasIdenticalTexture = Core3DTexturesMatch(
                    nativeMaterial.BaseColorTexture, authoredTexture);
                identicalExistingTextureByIdentifier.emplace(
                    existingIdentifier, hasIdenticalTexture);
            }
        }
        const bool isOwnedIdenticalTexture = hasIdenticalTexture
            && doc->SupportsScalarPBRMaterialEditingForLabel(label);
        Handle(Image_Texture) validatedEmissiveTexture;
        if (!nativeMaterial.EmissiveTexture.IsNull()) {
            if (!Core3DValidateAuthoredTexture(
                    nativeMaterial.EmissiveTexture)) {
                return Core3DTextureAuthoringFailure(
                    error, Core3DTextureAuthoringErrorUnsupportedSelection,
                    @"The existing emissive texture cannot be preserved safely.");
            }
            validatedEmissiveTexture = nativeMaterial.EmissiveTexture;
        }
        nativeMaterial.BaseColorTexture = authoredTexture;
        const bool changed = !isOwnedIdenticalTexture;
        pendingStyles.push_back({
            shape, label, nativeMaterial,
            validatedEmissiveTexture, changed});
        hasChanges = hasChanges || changed;
    }
    if (pendingStyles.empty()) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorNoSelection,
            @"Select at least one editable object.");
    }
    if (!hasChanges) {
        return YES;
    }

    try {
        transaction->NewCommand();
        if (!transaction->HasOpenCommand()) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorTransactionFailed,
                @"The texture edit could not be started.");
        }
        std::vector<OcctPBRMaterialUpdate> materialUpdates;
        materialUpdates.reserve(pendingStyles.size());
        for (const PendingTextureStyle& style : pendingStyles) {
            if (style.changed) {
                materialUpdates.push_back({
                    style.label, style.material, authoredTexture,
                    style.prevalidatedEmissiveTexture});
            }
        }
        if (materialUpdates.empty()
            || !doc->SaveObjectPBRMaterials(materialUpdates)) {
            Core3DAbortCommandNoThrow(transaction);
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorTransactionFailed,
                @"The texture edit could not be saved.");
        }
        if (!transaction->CommitCommand()) {
            Core3DAbortCommandNoThrow(transaction);
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorTransactionFailed,
                @"The texture edit could not be committed.");
        }
    } catch (...) {
        Core3DAbortCommandNoThrow(transaction);
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorTransactionFailed,
            @"The texture edit could not be committed.");
    }

    doc->NotifyChanges();
    NSMutableArray<Core3DPBRMaterial*>* selectedPBR =
        [NSMutableArray arrayWithCapacity:pendingStyles.size()];
    for (const PendingTextureStyle& style : pendingStyles) {
        if (style.changed) {
            style.shape->UnsetColor();
            doc->LoadObjectMeterial(style.label, style.shape);
            style.shape->SetToUpdate();
            context->Redisplay(style.shape, Standard_False);
        }
        Core3DPBRMaterial* published = Core3DMakePBRMaterial(
            style.material,
            doc->SupportsScalarPBRMaterialEditingForLabel(style.label),
            doc->SupportsBaseColorTextureEditingForLabel(style.label),
            doc->SupportsEmissiveTextureEditingForLabel(style.label),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::MetallicRoughness),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::Occlusion),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::Normal));
        if (published != nil) {
            [selectedPBR addObject:published];
        }
    }
    [self.materialController didChangeSelectionWithMaterials:@[] colors:@[]];
    [self.materialController didChangeSelectionWithPBRMaterials:selectedPBR];
    context->UpdateCurrentViewer();
    [self viewDidChangeViewportPresentationState];
    [self sendNotifyUIState:UIStateChangingApplyMaterial
                           | UIStateChangingHistory];
    return YES;
}

-(BOOL)clearSelectionBaseColorTextureWithError:(NSError* _Nullable * _Nullable)error {
    if (error != nullptr) {
        *error = nil;
    }
    if (![NSThread isMainThread]) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnavailable,
            @"Texture edits must run on the main thread.");
    }
    if (GLController == nil || GLController.viewer == nullptr) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnavailable,
            @"The material editor is not ready.");
    }
    auto context = GLController.viewer->AisContext();
    auto doc = GLController.viewer->getDocument();
    if (context.IsNull() || doc.IsNull()) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnavailable,
            @"The material editor is not ready.");
    }
    if ((self.selectedModelCapabilities
            & Core3DModelCapabilityMaterial) == 0) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnsupportedSelection,
            @"Select only editable model objects before editing a texture.");
    }
    auto transaction = doc->ChangeDocument();
    if (![self core3d_canBeginCommittedEdit]
        || transaction.IsNull() || transaction->HasOpenCommand()) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnavailable,
            @"Finish the current modeling operation before editing a texture.");
    }

    struct PendingTextureStyle {
        Handle(AIS_Shape) shape;
        TDF_Label label;
        XCAFDoc_VisMaterialPBR material;
        Handle(Image_Texture) prevalidatedEmissiveTexture;
        bool changed = false;
    };
    std::vector<PendingTextureStyle> pendingStyles;
    Core3DTextureAuthoringBudget authoringBudget;
#ifdef DEBUG
    authoringBudget.maximumObjects =
        static_cast<Standard_Size>(_debugMaximumTextureAuthoringObjects);
#endif
    bool hasChanges = false;
    for (context->InitSelected(); context->MoreSelected();
         context->NextSelected()) {
        const Handle(AIS_InteractiveObject) selected =
            context->SelectedInteractive();
        const Handle(AIS_Shape) shape =
            Handle(AIS_Shape)::DownCast(selected);
        if (shape.IsNull() || shape->Shape().IsNull()
            || !doc->IsPresentationEditable(selected)) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorUnsupportedSelection,
                @"Textures can be edited only on whole editable objects.");
        }
        const TDF_Label label = doc->ShapeLabel(selected);
        if (label.IsNull()
            || !doc->IsEditableFreeSimpleDefinitionLabel(label)
            || !doc->SupportsBaseColorTextureEditingForLabel(label)) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorUnsupportedSelection,
                @"This material contains texture maps that cannot be edited safely.");
        }
        if (authoringBudget.maximumObjects == 0
            || authoringBudget.objects
                >= authoringBudget.maximumObjects) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorUnsupportedSelection,
                @"Too many objects are selected for one texture edit.");
        }
        ++authoringBudget.objects;
        XCAFDoc_VisMaterialPBR nativeMaterial;
        if (!doc->TryEffectivePBRMaterialForLabel(
                label, nativeMaterial)) {
            nativeMaterial = Core3DLegacyPBRMaterial(
                doc->MaterialNameForLabel(label),
                doc->ColorNameForLabel(label));
        }
        Handle(Image_Texture) validatedEmissiveTexture;
        if (!nativeMaterial.EmissiveTexture.IsNull()) {
            if (!Core3DValidateAuthoredTexture(
                    nativeMaterial.EmissiveTexture)) {
                return Core3DTextureAuthoringFailure(
                    error, Core3DTextureAuthoringErrorUnsupportedSelection,
                    @"The existing emissive texture cannot be preserved safely.");
            }
            validatedEmissiveTexture = nativeMaterial.EmissiveTexture;
        }
        const bool changed = !nativeMaterial.BaseColorTexture.IsNull();
        nativeMaterial.BaseColorTexture.Nullify();
        pendingStyles.push_back({
            shape, label, nativeMaterial,
            validatedEmissiveTexture, changed});
        hasChanges = hasChanges || changed;
    }
    if (pendingStyles.empty()) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorNoSelection,
            @"Select at least one editable object.");
    }
    if (!hasChanges) {
        return YES;
    }

    try {
        transaction->NewCommand();
        if (!transaction->HasOpenCommand()) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorTransactionFailed,
                @"The texture edit could not be started.");
        }
        std::vector<OcctPBRMaterialUpdate> materialUpdates;
        materialUpdates.reserve(pendingStyles.size());
        for (const PendingTextureStyle& style : pendingStyles) {
            if (style.changed) {
                materialUpdates.push_back({
                    style.label,
                    style.material,
                    Handle(Image_Texture)(),
                    style.prevalidatedEmissiveTexture});
            }
        }
        if (materialUpdates.empty()
            || !doc->SaveObjectPBRMaterials(materialUpdates)) {
            Core3DAbortCommandNoThrow(transaction);
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorTransactionFailed,
                @"The texture edit could not be saved.");
        }
        if (!transaction->CommitCommand()) {
            Core3DAbortCommandNoThrow(transaction);
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorTransactionFailed,
                @"The texture edit could not be committed.");
        }
    } catch (...) {
        Core3DAbortCommandNoThrow(transaction);
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorTransactionFailed,
            @"The texture edit could not be committed.");
    }

    doc->NotifyChanges();
    NSMutableArray<Core3DPBRMaterial*>* selectedPBR =
        [NSMutableArray arrayWithCapacity:pendingStyles.size()];
    for (const PendingTextureStyle& style : pendingStyles) {
        if (style.changed) {
            style.shape->UnsetColor();
            doc->LoadObjectMeterial(style.label, style.shape);
            style.shape->SetToUpdate();
            context->Redisplay(style.shape, Standard_False);
        }
        Core3DPBRMaterial* published = Core3DMakePBRMaterial(
            style.material,
            doc->SupportsScalarPBRMaterialEditingForLabel(style.label),
            doc->SupportsBaseColorTextureEditingForLabel(style.label),
            doc->SupportsEmissiveTextureEditingForLabel(style.label),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::MetallicRoughness),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::Occlusion),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::Normal));
        if (published != nil) {
            [selectedPBR addObject:published];
        }
    }
    [self.materialController didChangeSelectionWithMaterials:@[] colors:@[]];
    [self.materialController didChangeSelectionWithPBRMaterials:selectedPBR];
    context->UpdateCurrentViewer();
    [self viewDidChangeViewportPresentationState];
    [self sendNotifyUIState:UIStateChangingApplyMaterial
                           | UIStateChangingHistory];
    return YES;
}

-(BOOL)updateSelectionWithEmissiveTextureData:(NSData*)textureData
                                  mediaType:(NSString*)mediaType
                                      error:(NSError* _Nullable * _Nullable)error {
    return [self core3d_updateSelectionWithTextureData:textureData mediaType:mediaType
        slot:OcctMaterialTextureSlot::Emissive error:error];
}
-(BOOL)clearSelectionEmissiveTextureWithError:(NSError* _Nullable * _Nullable)error {
    return [self core3d_clearSelectionTexture:OcctMaterialTextureSlot::Emissive error:error];
}

-(BOOL)updateSelectionWithMetallicRoughnessTextureData:(NSData*)textureData
                                  mediaType:(NSString*)mediaType
                                      error:(NSError* _Nullable * _Nullable)error {
    return [self core3d_updateSelectionWithTextureData:textureData mediaType:mediaType
        slot:OcctMaterialTextureSlot::MetallicRoughness error:error];
}
-(BOOL)clearSelectionMetallicRoughnessTextureWithError:(NSError* _Nullable * _Nullable)error {
    return [self core3d_clearSelectionTexture:OcctMaterialTextureSlot::MetallicRoughness error:error];
}

-(BOOL)updateSelectionWithOcclusionTextureData:(NSData*)textureData
                                  mediaType:(NSString*)mediaType
                                      error:(NSError* _Nullable * _Nullable)error {
    return [self core3d_updateSelectionWithTextureData:textureData mediaType:mediaType
        slot:OcctMaterialTextureSlot::Occlusion error:error];
}
-(BOOL)clearSelectionOcclusionTextureWithError:(NSError* _Nullable * _Nullable)error {
    return [self core3d_clearSelectionTexture:OcctMaterialTextureSlot::Occlusion error:error];
}

-(BOOL)updateSelectionWithNormalTextureData:(NSData*)textureData
                                  mediaType:(NSString*)mediaType
                                      error:(NSError* _Nullable * _Nullable)error {
    return [self core3d_updateSelectionWithTextureData:textureData mediaType:mediaType
        slot:OcctMaterialTextureSlot::Normal error:error];
}
-(BOOL)clearSelectionNormalTextureWithError:(NSError* _Nullable * _Nullable)error {
    return [self core3d_clearSelectionTexture:OcctMaterialTextureSlot::Normal error:error];
}

-(BOOL)core3d_updateSelectionWithTextureData:(NSData*)textureData
                                     mediaType:(NSString*)mediaType
                                          slot:(OcctMaterialTextureSlot)slot
                                         error:(NSError* _Nullable * _Nullable)error {
    if (error != nullptr) {
        *error = nil;
    }
    if (![NSThread isMainThread] || textureData == nil
        || mediaType == nil || textureData.length == 0
        || textureData.bytes == nullptr || mediaType.UTF8String == nullptr) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorInvalidInput,
            @"Choose a valid PNG or JPEG image.");
    }

    Handle(Image_Texture) authoredTexture;
    if (!Core3DCreateAuthoredTexture(
            static_cast<const Standard_Byte*>(textureData.bytes),
            static_cast<Standard_Size>(textureData.length),
            std::string(mediaType.UTF8String), authoredTexture)) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorInvalidInput,
            @"The image is invalid or exceeds the texture safety limits.");
    }
    if ((slot == OcctMaterialTextureSlot::MetallicRoughness || slot == OcctMaterialTextureSlot::Occlusion
        || slot == OcctMaterialTextureSlot::Normal)
        && !Core3DValidateNumericTexture(authoredTexture)) {
        return Core3DTextureAuthoringFailure(error, Core3DTextureAuthoringErrorInvalidInput,
            @"Data maps require an upright, opaque 8-bit RGB or grayscale PNG.");
    }
    if (GLController == nil || GLController.viewer == nullptr) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnavailable,
            @"The material editor is not ready.");
    }

    auto context = GLController.viewer->AisContext();
    auto doc = GLController.viewer->getDocument();
    if (context.IsNull() || doc.IsNull()) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnavailable,
            @"The material editor is not ready.");
    }
    if ((self.selectedModelCapabilities
            & Core3DModelCapabilityMaterial) == 0) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnsupportedSelection,
            @"Select only editable model objects before editing a texture.");
    }
    auto transaction = doc->ChangeDocument();
    if (![self core3d_canBeginCommittedEdit]
        || transaction.IsNull() || transaction->HasOpenCommand()) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnavailable,
            @"Finish the current modeling operation before editing a texture.");
    }

    struct PendingTextureStyle {
        Handle(AIS_Shape) shape;
        TDF_Label label;
        XCAFDoc_VisMaterialPBR material;
        Handle(Image_Texture) prevalidatedBaseColorTexture;
        bool autoPromotedEmissiveFactor = false;
        bool changed = false;
    };
    std::vector<PendingTextureStyle> pendingStyles;
    Core3DTextureAuthoringBudget authoringBudget;
    std::unordered_map<std::string, bool>
        identicalExistingTextureByIdentifier;
#ifdef DEBUG
    authoringBudget.maximumObjects =
        static_cast<Standard_Size>(_debugMaximumTextureAuthoringObjects);
#endif
    bool hasChanges = false;
    for (context->InitSelected(); context->MoreSelected();
         context->NextSelected()) {
        const Handle(AIS_InteractiveObject) selected =
            context->SelectedInteractive();
        const Handle(AIS_Shape) shape =
            Handle(AIS_Shape)::DownCast(selected);
        if (shape.IsNull() || shape->Shape().IsNull()
            || !doc->IsPresentationEditable(selected)) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorUnsupportedSelection,
                @"Textures can be edited only on whole editable objects.");
        }
        const TDF_Label label = doc->ShapeLabel(selected);
        if (label.IsNull()
            || !doc->IsEditableFreeSimpleDefinitionLabel(label)
            || !doc->SupportsMaterialTextureEditingForLabel(label, slot)) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorUnsupportedSelection,
                @"This material contains texture maps that cannot be edited safely.");
        }
        if (!Core3DHasCompleteCachedTextureCoordinates(
                shape->Shape(), authoringBudget)) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorMissingTextureCoordinates,
                @"Every selected face needs existing usable texture coordinates.");
        }

        XCAFDoc_VisMaterialPBR nativeMaterial;
        if (!doc->TryEffectivePBRMaterialForLabel(
                label, nativeMaterial)) {
            nativeMaterial = Core3DLegacyPBRMaterial(
                doc->MaterialNameForLabel(label),
                doc->ColorNameForLabel(label));
        }
        Handle(Image_Texture) validatedBaseColorTexture;
        if (!nativeMaterial.BaseColorTexture.IsNull()) {
            if (!Core3DValidateAuthoredTexture(
                    nativeMaterial.BaseColorTexture)) {
                return Core3DTextureAuthoringFailure(
                    error, Core3DTextureAuthoringErrorUnsupportedSelection,
                    @"The existing base color texture cannot be preserved safely.");
            }
            validatedBaseColorTexture = nativeMaterial.BaseColorTexture;
        }

        bool hasIdenticalTexture = false;
        if (!Core3DMaterialTexture(nativeMaterial, slot).IsNull()) {
            const std::string existingIdentifier(
                Core3DMaterialTexture(nativeMaterial, slot)->TextureId().ToCString());
            const auto cached = identicalExistingTextureByIdentifier.find(
                existingIdentifier);
            if (cached != identicalExistingTextureByIdentifier.end()) {
                hasIdenticalTexture = cached->second;
            } else {
                hasIdenticalTexture = Core3DTexturesMatch(
                    Core3DMaterialTexture(nativeMaterial, slot), authoredTexture);
                identicalExistingTextureByIdentifier.emplace(
                    existingIdentifier, hasIdenticalTexture);
            }
        }
        const bool isOwnedIdenticalTexture = hasIdenticalTexture
            && doc->SupportsScalarPBRMaterialEditingForLabel(label);
        const bool isFirstEmissiveTexture =
            Core3DMaterialTexture(nativeMaterial, slot).IsNull();
        bool autoPromotedEmissiveFactor =
            doc->IsEmissiveTextureFactorAutoPromotedForLabel(label);
        if (slot == OcctMaterialTextureSlot::Emissive && isFirstEmissiveTexture
            && nativeMaterial.EmissiveFactor.x() == 0.0f
            && nativeMaterial.EmissiveFactor.y() == 0.0f
            && nativeMaterial.EmissiveFactor.z() == 0.0f) {
            // Emissive texels multiply this factor in glTF and both viewports.
            // Promote only the default black factor; preserve any authored or
            // imported nonzero tint/intensity exactly.
            nativeMaterial.EmissiveFactor = Graphic3d_Vec3(
                1.0f, 1.0f, 1.0f);
            autoPromotedEmissiveFactor = true;
        }
        Core3DMaterialTexture(nativeMaterial, slot) = authoredTexture;
        const bool changed = !isOwnedIdenticalTexture;
        pendingStyles.push_back({
            shape, label, nativeMaterial,
            validatedBaseColorTexture,
            autoPromotedEmissiveFactor, changed});
        hasChanges = hasChanges || changed;
    }
    if (pendingStyles.empty()) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorNoSelection,
            @"Select at least one editable object.");
    }
    if (!hasChanges) {
        return YES;
    }

    try {
        transaction->NewCommand();
        if (!transaction->HasOpenCommand()) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorTransactionFailed,
                @"The texture edit could not be started.");
        }
        std::vector<OcctPBRMaterialUpdate> materialUpdates;
        materialUpdates.reserve(pendingStyles.size());
        for (const PendingTextureStyle& style : pendingStyles) {
            if (style.changed) {
                materialUpdates.push_back({
                    style.label, style.material,
                    style.prevalidatedBaseColorTexture, style.material.EmissiveTexture});
            }
        }
        if (materialUpdates.empty()
            || !doc->SaveObjectPBRMaterials(materialUpdates)) {
            Core3DAbortCommandNoThrow(transaction);
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorTransactionFailed,
                @"The texture edit could not be saved.");
        }
        for (const PendingTextureStyle& style : pendingStyles) {
            if (style.changed
                && !doc->SetEmissiveTextureFactorAutoPromotedForLabel(
                    style.label,
                    style.autoPromotedEmissiveFactor
                        ? Standard_True : Standard_False)) {
                Core3DAbortCommandNoThrow(transaction);
                return Core3DTextureAuthoringFailure(
                    error, Core3DTextureAuthoringErrorTransactionFailed,
                    @"The texture edit could not be saved.");
            }
        }
        if (!transaction->CommitCommand()) {
            Core3DAbortCommandNoThrow(transaction);
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorTransactionFailed,
                @"The texture edit could not be committed.");
        }
    } catch (...) {
        Core3DAbortCommandNoThrow(transaction);
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorTransactionFailed,
            @"The texture edit could not be committed.");
    }

    doc->NotifyChanges();
    NSMutableArray<Core3DPBRMaterial*>* selectedPBR =
        [NSMutableArray arrayWithCapacity:pendingStyles.size()];
    for (const PendingTextureStyle& style : pendingStyles) {
        if (style.changed) {
            style.shape->UnsetColor();
            doc->LoadObjectMeterial(style.label, style.shape);
            style.shape->SetToUpdate();
            context->Redisplay(style.shape, Standard_False);
        }
        Core3DPBRMaterial* published = Core3DMakePBRMaterial(
            style.material,
            doc->SupportsScalarPBRMaterialEditingForLabel(style.label),
            doc->SupportsBaseColorTextureEditingForLabel(style.label),
            doc->SupportsEmissiveTextureEditingForLabel(style.label),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::MetallicRoughness),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::Occlusion),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::Normal));
        if (published != nil) {
            [selectedPBR addObject:published];
        }
    }
    [self.materialController didChangeSelectionWithMaterials:@[] colors:@[]];
    [self.materialController didChangeSelectionWithPBRMaterials:selectedPBR];
    context->UpdateCurrentViewer();
    [self viewDidChangeViewportPresentationState];
    [self sendNotifyUIState:UIStateChangingApplyMaterial
                           | UIStateChangingHistory];
    return YES;
}

-(BOOL)core3d_clearSelectionTexture:(OcctMaterialTextureSlot)slot error:(NSError* _Nullable * _Nullable)error {
    if (error != nullptr) {
        *error = nil;
    }
    if (![NSThread isMainThread]) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnavailable,
            @"Texture edits must run on the main thread.");
    }
    if (GLController == nil || GLController.viewer == nullptr) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnavailable,
            @"The material editor is not ready.");
    }
    auto context = GLController.viewer->AisContext();
    auto doc = GLController.viewer->getDocument();
    if (context.IsNull() || doc.IsNull()) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnavailable,
            @"The material editor is not ready.");
    }
    if ((self.selectedModelCapabilities
            & Core3DModelCapabilityMaterial) == 0) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnsupportedSelection,
            @"Select only editable model objects before editing a texture.");
    }
    auto transaction = doc->ChangeDocument();
    if (![self core3d_canBeginCommittedEdit]
        || transaction.IsNull() || transaction->HasOpenCommand()) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorUnavailable,
            @"Finish the current modeling operation before editing a texture.");
    }

    struct PendingTextureStyle {
        Handle(AIS_Shape) shape;
        TDF_Label label;
        XCAFDoc_VisMaterialPBR material;
        Handle(Image_Texture) prevalidatedBaseColorTexture;
        bool autoPromotedEmissiveFactor = false;
        bool changed = false;
    };
    std::vector<PendingTextureStyle> pendingStyles;
    Core3DTextureAuthoringBudget authoringBudget;
#ifdef DEBUG
    authoringBudget.maximumObjects =
        static_cast<Standard_Size>(_debugMaximumTextureAuthoringObjects);
#endif
    bool hasChanges = false;
    for (context->InitSelected(); context->MoreSelected();
         context->NextSelected()) {
        const Handle(AIS_InteractiveObject) selected =
            context->SelectedInteractive();
        const Handle(AIS_Shape) shape =
            Handle(AIS_Shape)::DownCast(selected);
        if (shape.IsNull() || shape->Shape().IsNull()
            || !doc->IsPresentationEditable(selected)) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorUnsupportedSelection,
                @"Textures can be edited only on whole editable objects.");
        }
        const TDF_Label label = doc->ShapeLabel(selected);
        if (label.IsNull()
            || !doc->IsEditableFreeSimpleDefinitionLabel(label)
            || !doc->SupportsMaterialTextureEditingForLabel(label, slot)) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorUnsupportedSelection,
                @"This material contains texture maps that cannot be edited safely.");
        }
        if (authoringBudget.maximumObjects == 0
            || authoringBudget.objects
                >= authoringBudget.maximumObjects) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorUnsupportedSelection,
                @"Too many objects are selected for one texture edit.");
        }
        ++authoringBudget.objects;

        XCAFDoc_VisMaterialPBR nativeMaterial;
        if (!doc->TryEffectivePBRMaterialForLabel(
                label, nativeMaterial)) {
            nativeMaterial = Core3DLegacyPBRMaterial(
                doc->MaterialNameForLabel(label),
                doc->ColorNameForLabel(label));
        }
        Handle(Image_Texture) validatedBaseColorTexture;
        if (!nativeMaterial.BaseColorTexture.IsNull()) {
            if (!Core3DValidateAuthoredTexture(
                    nativeMaterial.BaseColorTexture)) {
                return Core3DTextureAuthoringFailure(
                    error, Core3DTextureAuthoringErrorUnsupportedSelection,
                    @"The existing base color texture cannot be preserved safely.");
            }
            validatedBaseColorTexture = nativeMaterial.BaseColorTexture;
        }
        const bool changed = !Core3DMaterialTexture(nativeMaterial, slot).IsNull();
        const bool autoPromotedEmissiveFactor =
            doc->IsEmissiveTextureFactorAutoPromotedForLabel(label);
        Core3DMaterialTexture(nativeMaterial, slot).Nullify();
        if (slot == OcctMaterialTextureSlot::Emissive && autoPromotedEmissiveFactor) {
            nativeMaterial.EmissiveFactor = Graphic3d_Vec3(
                0.0f, 0.0f, 0.0f);
        }
        pendingStyles.push_back({
            shape, label, nativeMaterial,
            validatedBaseColorTexture,
            autoPromotedEmissiveFactor, changed});
        hasChanges = hasChanges || changed;
    }
    if (pendingStyles.empty()) {
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorNoSelection,
            @"Select at least one editable object.");
    }
    if (!hasChanges) {
        return YES;
    }

    try {
        transaction->NewCommand();
        if (!transaction->HasOpenCommand()) {
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorTransactionFailed,
                @"The texture edit could not be started.");
        }
        std::vector<OcctPBRMaterialUpdate> materialUpdates;
        materialUpdates.reserve(pendingStyles.size());
        for (const PendingTextureStyle& style : pendingStyles) {
            if (style.changed) {
                materialUpdates.push_back({
                    style.label, style.material,
                    style.prevalidatedBaseColorTexture,
                    Handle(Image_Texture)()});
            }
        }
        if (materialUpdates.empty()
            || !doc->SaveObjectPBRMaterials(materialUpdates)) {
            Core3DAbortCommandNoThrow(transaction);
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorTransactionFailed,
                @"The texture edit could not be saved.");
        }
        for (const PendingTextureStyle& style : pendingStyles) {
            if (style.changed
                && !doc->SetEmissiveTextureFactorAutoPromotedForLabel(
                    style.label, slot == OcctMaterialTextureSlot::Emissive
                        ? Standard_False : style.autoPromotedEmissiveFactor)) {
                Core3DAbortCommandNoThrow(transaction);
                return Core3DTextureAuthoringFailure(
                    error, Core3DTextureAuthoringErrorTransactionFailed,
                    @"The texture edit could not be saved.");
            }
        }
        if (!transaction->CommitCommand()) {
            Core3DAbortCommandNoThrow(transaction);
            return Core3DTextureAuthoringFailure(
                error, Core3DTextureAuthoringErrorTransactionFailed,
                @"The texture edit could not be committed.");
        }
    } catch (...) {
        Core3DAbortCommandNoThrow(transaction);
        return Core3DTextureAuthoringFailure(
            error, Core3DTextureAuthoringErrorTransactionFailed,
            @"The texture edit could not be committed.");
    }

    doc->NotifyChanges();
    NSMutableArray<Core3DPBRMaterial*>* selectedPBR =
        [NSMutableArray arrayWithCapacity:pendingStyles.size()];
    for (const PendingTextureStyle& style : pendingStyles) {
        if (style.changed) {
            style.shape->UnsetColor();
            doc->LoadObjectMeterial(style.label, style.shape);
            style.shape->SetToUpdate();
            context->Redisplay(style.shape, Standard_False);
        }
        Core3DPBRMaterial* published = Core3DMakePBRMaterial(
            style.material,
            doc->SupportsScalarPBRMaterialEditingForLabel(style.label),
            doc->SupportsBaseColorTextureEditingForLabel(style.label),
            doc->SupportsEmissiveTextureEditingForLabel(style.label),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::MetallicRoughness),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::Occlusion),
                doc->SupportsMaterialTextureEditingForLabel(style.label, OcctMaterialTextureSlot::Normal));
        if (published != nil) {
            [selectedPBR addObject:published];
        }
    }
    [self.materialController didChangeSelectionWithMaterials:@[] colors:@[]];
    [self.materialController didChangeSelectionWithPBRMaterials:selectedPBR];
    context->UpdateCurrentViewer();
    [self viewDidChangeViewportPresentationState];
    [self sendNotifyUIState:UIStateChangingApplyMaterial
                           | UIStateChangingHistory];
    return YES;
}

-(void) receiveOcctDocumentNotification:(NSNotification *) notification {
    if (![[notification name] isEqualToString:@"OcctDocumentChanges"]
        || ![notification.object isKindOfClass:NSValue.class]) {
        return;
    }

    auto activeDocument = GLController.viewer->getDocument();
    if (activeDocument.IsNull()
        || [(NSValue *)notification.object pointerValue] != activeDocument.get()) {
        return;
    }

    [self viewDidAssetModify];
}

#ifdef DEBUG
+ (NSDictionary<NSString *, NSNumber *> *)
    debugGeometryRepresentationSchemaValues {
    return @{
        @"invalid": @(
            static_cast<Standard_Integer>(
                OcctGeometryRepresentation::Invalid)),
        @"legacyUnknown": @(
            static_cast<Standard_Integer>(
                OcctGeometryRepresentation::LegacyUnknown)),
        @"bRep": @(
            static_cast<Standard_Integer>(
                OcctGeometryRepresentation::BRep)),
        @"triangleMesh": @(
            static_cast<Standard_Integer>(
                OcctGeometryRepresentation::TriangleMesh)),
    };
}

- (NSData *_Nullable)debugGeometryRepresentationFixtureDataWithMode:
    (Core3DDebugGeometryFixtureMode)mode {
    NSString* suffix = nil;
    switch (mode) {
        case Core3DDebugGeometryFixtureEmpty:
            suffix = @"empty-geometry-representation";
            break;
        case Core3DDebugGeometryFixtureLegacyUnmarkedBRep:
            suffix = @"legacy-unmarked-brep";
            break;
        case Core3DDebugGeometryFixtureMarkedBRep:
            suffix = @"marked-brep";
            break;
        case Core3DDebugGeometryFixtureMarkedTriangleMesh:
            suffix = @"marked-triangle-mesh";
            break;
        case Core3DDebugGeometryFixtureValidBRepAndTriangleMeshRoots:
            suffix = @"valid-brep-and-triangle-mesh-roots";
            break;
        case Core3DDebugGeometryFixtureUnmarkedTriangleMesh:
            suffix = @"unmarked-triangle-mesh";
            break;
        case Core3DDebugGeometryFixtureUnmarkedMixedDefinition:
            suffix = @"unmarked-mixed-definition";
            break;
        case Core3DDebugGeometryFixtureBRepMarkerOnTriangleMesh:
            suffix = @"brep-marker-on-triangle-mesh";
            break;
        case Core3DDebugGeometryFixtureTriangleMeshMarkerOnBRep:
            suffix = @"triangle-mesh-marker-on-brep";
            break;
        case Core3DDebugGeometryFixtureBRepMarkerOnMixedDefinition:
            suffix = @"brep-marker-on-mixed-definition";
            break;
        case Core3DDebugGeometryFixtureTriangleMeshMarkerOnMixedDefinition:
            suffix = @"triangle-mesh-marker-on-mixed-definition";
            break;
        case Core3DDebugGeometryFixtureUnknownMarkerOnBRep:
            suffix = @"unknown-marker-on-brep";
            break;
        case Core3DDebugGeometryFixtureOrphanMarker:
            suffix = @"orphan-geometry-representation-marker";
            break;
        case Core3DDebugGeometryFixtureMarkedTriangleMeshWithUnreferencedOutlier:
            suffix = @"marked-triangle-mesh-unreferenced-outlier";
            break;
        case Core3DDebugGeometryFixtureMarkedBRepWithUniformScale:
            suffix = @"marked-brep-uniform-scale";
            break;
        case Core3DDebugGeometryFixtureMarkedBRepWithCorruptTransform:
            suffix = @"marked-brep-corrupt-transform";
            break;
        case Core3DDebugGeometryFixtureMarkedTriangleMeshWithLocatedTriangulation:
            suffix = @"marked-triangle-mesh-located-triangulation";
            break;
        case Core3DDebugGeometryFixtureMarkedBRepFaceRoot:
            suffix = @"marked-brep-face-root";
            break;
        case Core3DDebugGeometryFixtureMarkedBRepSubshapeRoots:
            suffix = @"marked-brep-subshape-roots";
            break;
    }
    if (suffix == nil) {
        return nil;
    }

    return Core3DCreateDebugBinXCAFFixture(
        suffix,
        [mode](const Handle(TDocStd_Document)& document) {
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            if (shapeTool.IsNull()) {
                throw Standard_Failure(
                    "Unable to create geometry fixture shape tool");
            }
            const Standard_Integer bRep =
                static_cast<Standard_Integer>(
                    OcctGeometryRepresentation::BRep);
            const Standard_Integer triangleMesh =
                static_cast<Standard_Integer>(
                    OcctGeometryRepresentation::TriangleMesh);
            const auto addBRep = [&]() {
                return Core3DAddDebugGeometryDefinition(
                    shapeTool,
                    BRepPrimAPI_MakeBox(
                        gp_Pnt(-35.0, -10.0, -10.0),
                        20.0, 20.0, 20.0).Shape());
            };
            const auto addTriangleMesh = [&]() {
                return Core3DAddDebugGeometryDefinition(
                    shapeTool,
                    Core3DMakeDebugTriangleMeshFace(25.0));
            };
            const auto addTriangleMeshWithUnreferencedOutlier = [&]() {
                return Core3DAddDebugGeometryDefinition(
                    shapeTool,
                    Core3DMakeDebugTriangleMeshFace(
                        25.0, Standard_True));
            };
            const auto addLocatedTriangleMesh = [&]() {
                return Core3DAddDebugGeometryDefinition(
                    shapeTool,
                    Core3DMakeDebugLocatedTriangleMeshDefinition());
            };
            const auto addMixed = [&]() {
                return Core3DAddDebugGeometryDefinition(
                    shapeTool,
                    Core3DMakeDebugMixedGeometryDefinition());
            };
            const auto addBRepRoot = [&](const TopAbs_ShapeEnum rootType,
                                         const Standard_Real x) {
                const TopoDS_Shape solid = BRepPrimAPI_MakeBox(
                    gp_Pnt(x, -10.0, -10.0),
                    20.0, 20.0, 20.0).Shape();
                TopExp_Explorer root(solid, rootType);
                if (!root.More() || root.Current().IsNull()) {
                    throw Standard_Failure(
                        "Unable to extract BRep root fixture");
                }
                return Core3DAddDebugGeometryDefinition(
                    shapeTool, root.Current());
            };

            switch (mode) {
                case Core3DDebugGeometryFixtureEmpty:
                    return;
                case Core3DDebugGeometryFixtureLegacyUnmarkedBRep:
                    (void)addBRep();
                    return;
                case Core3DDebugGeometryFixtureMarkedBRep:
                    Core3DSetDebugGeometryRepresentation(addBRep(), bRep);
                    return;
                case Core3DDebugGeometryFixtureMarkedTriangleMesh:
                    Core3DSetDebugGeometryRepresentation(
                        addTriangleMesh(), triangleMesh);
                    return;
                case Core3DDebugGeometryFixtureValidBRepAndTriangleMeshRoots:
                    Core3DSetDebugGeometryRepresentation(addBRep(), bRep);
                    Core3DSetDebugGeometryRepresentation(
                        addTriangleMesh(), triangleMesh);
                    return;
                case Core3DDebugGeometryFixtureUnmarkedTriangleMesh:
                    (void)addTriangleMesh();
                    return;
                case Core3DDebugGeometryFixtureUnmarkedMixedDefinition:
                    (void)addMixed();
                    return;
                case Core3DDebugGeometryFixtureBRepMarkerOnTriangleMesh:
                    Core3DSetDebugGeometryRepresentation(
                        addTriangleMesh(), bRep);
                    return;
                case Core3DDebugGeometryFixtureTriangleMeshMarkerOnBRep:
                    Core3DSetDebugGeometryRepresentation(
                        addBRep(), triangleMesh);
                    return;
                case Core3DDebugGeometryFixtureBRepMarkerOnMixedDefinition:
                    Core3DSetDebugGeometryRepresentation(addMixed(), bRep);
                    return;
                case Core3DDebugGeometryFixtureTriangleMeshMarkerOnMixedDefinition:
                    Core3DSetDebugGeometryRepresentation(
                        addMixed(), triangleMesh);
                    return;
                case Core3DDebugGeometryFixtureUnknownMarkerOnBRep:
                    Core3DSetDebugGeometryRepresentation(addBRep(), 99);
                    return;
                case Core3DDebugGeometryFixtureOrphanMarker: {
                    Core3DSetDebugGeometryRepresentation(addBRep(), bRep);
                    const TDF_Label orphan =
                        document->Main().FindChild(97, Standard_True);
                    Core3DSetDebugGeometryRepresentation(orphan, bRep);
                    return;
                }
                case Core3DDebugGeometryFixtureMarkedTriangleMeshWithUnreferencedOutlier:
                    Core3DSetDebugGeometryRepresentation(
                        addTriangleMeshWithUnreferencedOutlier(),
                        triangleMesh);
                    return;
                case Core3DDebugGeometryFixtureMarkedBRepWithUniformScale: {
                    const TDF_Label label = addBRep();
                    Core3DSetDebugGeometryRepresentation(label, bRep);
                    Core3DSetDebugObjectScale(label, 2.5);
                    return;
                }
                case Core3DDebugGeometryFixtureMarkedBRepWithCorruptTransform: {
                    const TDF_Label label = addBRep();
                    Core3DSetDebugGeometryRepresentation(label, bRep);
                    // gp_Trsf accepts this finite, nonzero scale during load;
                    // the checked inspector rejects it because it is below
                    // Precision::Confusion().
                    Core3DSetDebugObjectScale(label, 1.0e-12);
                    return;
                }
                case Core3DDebugGeometryFixtureMarkedTriangleMeshWithLocatedTriangulation:
                    Core3DSetDebugGeometryRepresentation(
                        addLocatedTriangleMesh(), triangleMesh);
                    return;
                case Core3DDebugGeometryFixtureMarkedBRepFaceRoot:
                    Core3DSetDebugGeometryRepresentation(
                        addBRepRoot(TopAbs_FACE, -10.0), bRep);
                    return;
                case Core3DDebugGeometryFixtureMarkedBRepSubshapeRoots:
                    Core3DSetDebugGeometryRepresentation(
                        addBRepRoot(TopAbs_WIRE, -40.0), bRep);
                    Core3DSetDebugGeometryRepresentation(
                        addBRepRoot(TopAbs_EDGE, 0.0), bRep);
                    Core3DSetDebugGeometryRepresentation(
                        addBRepRoot(TopAbs_VERTEX, 40.0), bRep);
                    return;
            }
        });
}

- (NSArray<NSDictionary<NSString *, NSNumber *> *> *)
    debugGeometryRepresentationStates {
    if (GLController == nil || GLController.viewer == nullptr) {
        return @[];
    }
    const Handle(OcctDocument) document =
        GLController.viewer->getDocument();
    const Handle(TDocStd_Document) ocaf = document.IsNull()
        ? Handle(TDocStd_Document)()
        : document->ChangeDocument();
    if (document.IsNull() || ocaf.IsNull()
        || !XCAFDoc_DocumentTool::CheckShapeTool(ocaf->Main())) {
        return @[];
    }
    const Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(ocaf->Main());
    if (shapeTool.IsNull()) {
        return @[];
    }

    NSMutableArray<NSDictionary<NSString *, NSNumber *> *> *states =
        [NSMutableArray array];
    try {
        TDF_LabelSequence labels;
        shapeTool->GetShapes(labels);
        for (Standard_Integer index = 1;
             index <= labels.Length(); ++index) {
            const TDF_Label& label = labels.Value(index);
            if (label.IsNull()
                || !shapeTool->IsShape(label)
                || XCAFDoc_ShapeTool::IsReference(label)
                || XCAFDoc_ShapeTool::IsComponent(label)
                || XCAFDoc_ShapeTool::IsSubShape(label)) {
                continue;
            }

            Standard_Integer analyticFaceCount = 0;
            Standard_Integer triangleOnlyFaceCount = 0;
            Standard_Integer invalidFaceCount = 0;
            const TopoDS_Shape shape =
                XCAFDoc_ShapeTool::GetShape(label);
            for (TopExp_Explorer faces(shape, TopAbs_FACE);
                 faces.More(); faces.Next()) {
                const TopoDS_Face face =
                    TopoDS::Face(faces.Current());
                if (!BRep_Tool::Surface(face).IsNull()) {
                    ++analyticFaceCount;
                    continue;
                }
                ++triangleOnlyFaceCount;
                TopLoc_Location location;
                const Handle(Poly_Triangulation)& triangulation =
                    BRep_Tool::Triangulation(face, location);
                if (triangulation.IsNull()
                    || !triangulation->HasGeometry()
                    || triangulation->NbNodes() <= 0
                    || triangulation->NbTriangles() <= 0) {
                    ++invalidFaceCount;
                }
            }

            const OcctGeometryRepresentation resolved =
                document->GeometryRepresentationForLabel(label);
            Handle(TDataStd_Integer) marker;
            const bool markerPresent = label.FindAttribute(
                Core3DDebugGeometryRepresentationAttributeID(), marker);
            NSMutableDictionary<NSString *, NSNumber *> *state =
                [@{
                    @"resolvedRawValue": @(
                        static_cast<Standard_Integer>(resolved)),
                    @"markerPresent": @(markerPresent),
                    @"analyticFaceCount": @(analyticFaceCount),
                    @"triangleOnlyFaceCount": @(triangleOnlyFaceCount),
                    @"invalidFaceCount": @(invalidFaceCount),
                } mutableCopy];
            if (markerPresent && !marker.IsNull()) {
                state[@"storedRawValue"] = @(marker->Get());
            }
            OcctObjectTransformState transformState;
            if (document->CaptureObjectTransformStateForLabel(label, transformState)) {
                state[@"meshUVAtlasVersion"] = @(transformState.meshUVAtlasVersion);
                state[@"meshUVAtlasResolution"] = @(transformState.meshUVAtlasSettings[0]);
                state[@"meshUVAtlasGutterPixels"] = @(transformState.meshUVAtlasSettings[1]);
                state[@"meshUVAtlasOriginalNodes"] = @(transformState.meshUVAtlasSettings[2]);
            }
            [states addObject:state];
        }
    } catch (...) {
        return @[];
    }
    return states;
}

- (NSDictionary<NSString *, NSNumber *> *)debugFirstReferenceAxisState {
    if (![NSThread isMainThread] || GLController == nil
        || GLController.viewer == nullptr) {
        return @{ @"readState": @(-1), @"valid": @NO };
    }
    const Handle(OcctDocument) document =
        GLController.viewer->getDocument();
    const Handle(TDocStd_Document) ocaf = document.IsNull()
        ? Handle(TDocStd_Document)() : document->ChangeDocument();
    const TDF_Label label = Core3DFirstFreeSimpleDefinition(ocaf);
    if (document.IsNull() || ocaf.IsNull() || label.IsNull()) {
        return @{ @"readState": @(-1), @"valid": @NO };
    }

    try {
        OCC_CATCH_SIGNALS
        OcctReferenceAxis reference;
        const OcctReferenceAxisReadState readState =
            document->ReadReferenceAxisForLabel(label, reference);
        gp_Ax1 resolved;
        const BOOL valid = readState != OcctReferenceAxisReadState::Invalid
            && document->ResolveReferenceAxisInWorld(
                label, TopLoc_Location(), resolved);
        if (!valid) {
            return @{
                @"readState": @(
                    static_cast<Standard_Integer>(readState)),
                @"valid": @NO,
                @"documentTime": @(ocaf->GetData()->Time()),
                @"undoCount": @(ocaf->GetAvailableUndos()),
            };
        }
        return @{
            @"readState": @(
                static_cast<Standard_Integer>(readState)),
            @"valid": @YES,
            @"pivotSpace": @(
                static_cast<Standard_Integer>(reference.pivotSpace)),
            @"directionSpace": @(
                static_cast<Standard_Integer>(reference.directionSpace)),
            @"pivotX": @(reference.pivot.X()),
            @"pivotY": @(reference.pivot.Y()),
            @"pivotZ": @(reference.pivot.Z()),
            @"directionX": @(reference.direction.X()),
            @"directionY": @(reference.direction.Y()),
            @"directionZ": @(reference.direction.Z()),
            @"worldPivotX": @(resolved.Location().X()),
            @"worldPivotY": @(resolved.Location().Y()),
            @"worldPivotZ": @(resolved.Location().Z()),
            @"worldDirectionX": @(resolved.Direction().X()),
            @"worldDirectionY": @(resolved.Direction().Y()),
            @"worldDirectionZ": @(resolved.Direction().Z()),
            @"documentTime": @(ocaf->GetData()->Time()),
            @"undoCount": @(ocaf->GetAvailableUndos()),
        };
    } catch (...) {
        return @{ @"readState": @(-1), @"valid": @NO };
    }
}

- (BOOL)debugSetFirstReferenceAxis:
    (NSDictionary<NSString *, NSNumber *> *)values {
    if (![NSThread isMainThread] || values == nil || GLController == nil
        || GLController.viewer == nullptr) {
        return NO;
    }
    NSArray<NSString *> *keys = @[
        @"pivotSpace", @"directionSpace",
        @"pivotX", @"pivotY", @"pivotZ",
        @"directionX", @"directionY", @"directionZ",
    ];
    for (NSString *key in keys) {
        if (![values[key] isKindOfClass:NSNumber.class]) {
            return NO;
        }
    }

    const NSInteger pivotSpace = values[@"pivotSpace"].integerValue;
    const NSInteger directionSpace =
        values[@"directionSpace"].integerValue;
    const Standard_Real pivotX = values[@"pivotX"].doubleValue;
    const Standard_Real pivotY = values[@"pivotY"].doubleValue;
    const Standard_Real pivotZ = values[@"pivotZ"].doubleValue;
    const Standard_Real directionX = values[@"directionX"].doubleValue;
    const Standard_Real directionY = values[@"directionY"].doubleValue;
    const Standard_Real directionZ = values[@"directionZ"].doubleValue;
    const Standard_Real directionSquared =
        directionX * directionX + directionY * directionY
        + directionZ * directionZ;
    if ((pivotSpace != 0 && pivotSpace != 1)
        || (directionSpace != 0 && directionSpace != 1)
        || !std::isfinite(pivotX) || !std::isfinite(pivotY)
        || !std::isfinite(pivotZ) || !std::isfinite(directionX)
        || !std::isfinite(directionY) || !std::isfinite(directionZ)
        || !std::isfinite(directionSquared)
        || directionSquared
            <= std::numeric_limits<Standard_Real>::epsilon()) {
        return NO;
    }

    const Handle(OcctDocument) document =
        GLController.viewer->getDocument();
    const Handle(TDocStd_Document) ocaf = document.IsNull()
        ? Handle(TDocStd_Document)() : document->ChangeDocument();
    const TDF_Label label = Core3DFirstFreeSimpleDefinition(ocaf);
    if (document.IsNull() || ocaf.IsNull() || label.IsNull()
        || ocaf->HasOpenCommand()) {
        return NO;
    }

    try {
        OCC_CATCH_SIGNALS
        OcctReferenceAxis candidate;
        candidate.pivotSpace = pivotSpace == 0
            ? OcctReferenceSpace::Object : OcctReferenceSpace::World;
        candidate.pivot = gp_Pnt(pivotX, pivotY, pivotZ);
        candidate.directionSpace = directionSpace == 0
            ? OcctReferenceSpace::Object : OcctReferenceSpace::World;
        candidate.direction = gp_Dir(
            directionX, directionY, directionZ);

        OcctReferenceAxis existing;
        const OcctReferenceAxisReadState existingState =
            document->ReadReferenceAxisForLabel(label, existing);
        if (existingState == OcctReferenceAxisReadState::Authored
            && existing.pivotSpace == candidate.pivotSpace
            && existing.directionSpace == candidate.directionSpace
            && existing.pivot.IsEqual(candidate.pivot, 1.0e-12)
            && existing.direction.IsEqual(
                candidate.direction, 1.0e-12)) {
            return YES;
        }

        ocaf->NewCommand();
        if (!ocaf->HasOpenCommand()
            || !document->SetReferenceAxisForLabel(label, candidate)
            || !document->ValidateReferenceAxes()
            || !document->ValidateGeometryRepresentations()
            || !ocaf->CommitCommand()) {
            Core3DAbortCommandNoThrow(ocaf);
            return NO;
        }
        document->NotifyChanges();
        return YES;
    } catch (...) {
        Core3DAbortCommandNoThrow(ocaf);
        return NO;
    }
}

- (BOOL)debugResetFirstReferenceAxis {
    if (![NSThread isMainThread] || GLController == nil
        || GLController.viewer == nullptr) {
        return NO;
    }
    const Handle(OcctDocument) document =
        GLController.viewer->getDocument();
    const Handle(TDocStd_Document) ocaf = document.IsNull()
        ? Handle(TDocStd_Document)() : document->ChangeDocument();
    const TDF_Label label = Core3DFirstFreeSimpleDefinition(ocaf);
    if (document.IsNull() || ocaf.IsNull() || label.IsNull()
        || ocaf->HasOpenCommand()) {
        return NO;
    }
    try {
        OCC_CATCH_SIGNALS
        OcctReferenceAxis existing;
        if (document->ReadReferenceAxisForLabel(label, existing)
                == OcctReferenceAxisReadState::ImplicitDefault) {
            return YES;
        }
        ocaf->NewCommand();
        if (!ocaf->HasOpenCommand()
            || !document->ResetReferenceAxisForLabel(label)
            || !document->ValidateReferenceAxes()
            || !document->ValidateGeometryRepresentations()
            || !ocaf->CommitCommand()) {
            Core3DAbortCommandNoThrow(ocaf);
            return NO;
        }
        document->NotifyChanges();
        return YES;
    } catch (...) {
        Core3DAbortCommandNoThrow(ocaf);
        return NO;
    }
}

- (BOOL)debugSetFirstReferenceAxisPersistedUniformScale:(CGFloat)scale {
    if (![NSThread isMainThread] || !std::isfinite(scale)
        || std::abs(scale)
            <= std::numeric_limits<Standard_Real>::epsilon()
        || GLController == nil || GLController.viewer == nullptr) {
        return NO;
    }
    const Handle(OcctDocument) document =
        GLController.viewer->getDocument();
    const Handle(TDocStd_Document) ocaf = document.IsNull()
        ? Handle(TDocStd_Document)() : document->ChangeDocument();
    const TDF_Label label = Core3DFirstFreeSimpleDefinition(ocaf);
    if (document.IsNull() || ocaf.IsNull() || label.IsNull()
        || ocaf->HasOpenCommand()) {
        return NO;
    }
    try {
        OCC_CATCH_SIGNALS
        const TDF_Label scaleLabel = label.FindChild(8, Standard_True);
        if (scaleLabel.IsNull()) {
            return NO;
        }
        Handle(TDataStd_Real) existing;
        if (scaleLabel.FindAttribute(TDataStd_Real::GetID(), existing)
            && !existing.IsNull() && existing->Get() == scale) {
            return YES;
        }
        ocaf->NewCommand();
        if (!ocaf->HasOpenCommand()) {
            return NO;
        }
        TDataStd_Real::Set(scaleLabel, scale);
        gp_Trsf storedTransform;
        if (!document->TryObjectTransformForLabel(label, storedTransform)
            || storedTransform.ScaleFactor() != scale
            || !document->ValidateReferenceAxes()
            || !document->ValidateGeometryRepresentations()
            || !ocaf->CommitCommand()) {
            Core3DAbortCommandNoThrow(ocaf);
            return NO;
        }
        document->NotifyChanges();
        return YES;
    } catch (...) {
        Core3DAbortCommandNoThrow(ocaf);
        return NO;
    }
}

- (NSDictionary<NSString *, id> *_Nullable)debugProbeTransformStorage:(NSInteger)mode {
    if (![NSThread isMainThread] || mode < 0 || mode > 5
        || GLController == nil || GLController.viewer == nullptr) {
        return nil;
    }
    const Handle(OcctDocument) document = GLController.viewer->getDocument();
    const Handle(TDocStd_Document) ocaf = document.IsNull()
        ? Handle(TDocStd_Document)() : document->ChangeDocument();
    const TDF_Label label = Core3DFirstFreeSimpleDefinition(ocaf);
    if (document.IsNull() || ocaf.IsNull() || label.IsNull()
        || ocaf->HasOpenCommand()) {
        return nil;
    }
    // Exercise staging without publishing or committing a document edit.
    // Swift checks the measured state and history after the owned abort.
    try {
        OcctObjectTransformState beforeState;
        gp_Trsf baseline;
        if (!document->CaptureObjectTransformStateForLabel(label, beforeState)
            || !document->TryObjectTransformForLabel(label, baseline)) {
            return nil;
        }
        gp_Trsf candidate;
        candidate.SetRotationPart(gp_Quaternion(gp_Vec(0, 0, 1), M_PI_2));
        candidate.SetScaleFactor(-2.0);
        candidate.SetTranslationPart(gp_XYZ(mode == 3 ? 1'000'001.0 : 12.5, -7.25, 3.0));
        Handle(AIS_Shape) presentation = new AIS_Shape(XCAFDoc_ShapeTool::GetShape(label));
        presentation->SetLocalTransformation(candidate);
        if (mode != 0) {
            ocaf->NewCommand();
            if (!ocaf->HasOpenCommand()) {
                return nil;
            }
        }
        bool accepted = false;
        if (mode == 5) {
            // Same effective zero translation, different exact authored state.
            label.FindChild(1, Standard_False).ForgetAttribute(TDataStd_Real::GetID());
        } else {
            accepted = document->SaveObjectTransform(
                mode == 2 ? ocaf->Main() : label,
                mode == 1 ? Handle(AIS_Shape)() : presentation);
        }
        OcctObjectTransformState stagedState;
        const bool stateReadable = document->CaptureObjectTransformStateForLabel(label, stagedState);
        const bool stateUnchanged = stateReadable && beforeState.IsEqual(stagedState);
        // Invalid capture must erase previous output instead of leaving usable proof.
        OcctObjectTransformState invalidState = beforeState;
        const bool invalidRejected = !document->CaptureObjectTransformStateForLabel(ocaf->Main(), invalidState)
            && invalidState.label.IsNull() && !invalidState.IsEqual(beforeState);
        gp_Trsf staged;
        const bool readable = document->TryObjectTransformForLabel(label, staged);
        NSMutableArray<NSNumber *> *matrix = [NSMutableArray arrayWithCapacity:12];
        for (Standard_Integer row = 1; row <= 3; ++row) {
            for (Standard_Integer column = 1; column <= 4; ++column) {
                [matrix addObject:@(staged.Value(row, column))];
            }
        }
        if (mode != 0) {
            ocaf->AbortCommand();
        }
        gp_Trsf restored;
        bool unchanged = document->TryObjectTransformForLabel(label, restored);
        for (Standard_Integer row = 1; row <= 3; ++row) {
            for (Standard_Integer column = 1; column <= 4; ++column) {
                unchanged = unchanged
                    && restored.Value(row, column) == baseline.Value(row, column);
            }
        }
        OcctObjectTransformState restoredState;
        const bool exactRestored = document->CaptureObjectTransformStateForLabel(label, restoredState)
            && beforeState.IsEqual(restoredState);
        return @{@"accepted": @(accepted), @"readable": @(readable),
                 @"matrix": matrix, @"restored": @(unchanged),
                 @"closed": @(!ocaf->HasOpenCommand()),
                 @"stateReadable": @(stateReadable), @"stateUnchanged": @(stateUnchanged),
                 @"exactRestored": @(exactRestored), @"invalidRejected": @(invalidRejected)};
    } catch (...) {
        // Entry established that no other command existed; this synchronous
        // debug-only probe invokes no callbacks before cleanup.
        if (mode != 0) { Core3DAbortCommandNoThrow(ocaf); }
        return nil;
    }
}

- (NSInteger)debugApplyViewerOrdinaryTransform:(NSInteger)mode paused:(BOOL)paused {
    if (![NSThread isMainThread] || mode < 0 || mode > 5
        || GLController == nil || GLController.viewer == nullptr) { return 5; }
    const auto viewer = GLController.viewer;
    const auto document = viewer->getDocument();
    const auto context = viewer->AisContext();
    if (document.IsNull() || context.IsNull()) { return 5; }
    try {
        std::vector<core3d::OrdinaryTransformChange> changes;
        for (context->InitSelected(); context->MoreSelected(); context->NextSelected()) {
            core3d::OrdinaryTransformChange change;
            change.presentation = Handle(AIS_Shape)::DownCast(context->SelectedInteractive());
            change.label = document->ShapeLabel(change.presentation);
            OcctObjectTransformState before;
            if (!document->CaptureObjectTransformStateForLabel(change.label, before)) { return 5; }
            change.shape = before.shape;
            change.transform = before.transform;
            if (mode == 0) {
                auto translation = change.transform.TranslationPart();
                translation.SetX(translation.X() + 12.5);
                change.transform.SetTranslationPart(translation);
            } else if (mode == 1) {
                change.operation = core3d::OrdinaryTransformOperation::Rotate;
                change.transform.SetRotationPart(gp_Quaternion(gp_Vec(0, 0, 1), M_PI_2));
            } else {
                change.operation = core3d::OrdinaryTransformOperation::Scale;
                gp_Trsf scale;
                scale.SetScale(gp_Pnt(0, 0, 0), 2.0);
                if (mode == 2) {
                    // BRep transformation can be a no-op on a triangle-only
                    // shape. The mesh rejection probe must actually replace it.
                    change.shape = before.resolvedRepresentation == OcctGeometryRepresentation::TriangleMesh
                        ? BRepBuilderAPI_Copy(before.shape, Standard_True, Standard_True).Shape()
                        : BRepBuilderAPI_Transform(before.shape, scale, Standard_True).Shape();
                } else {
                    change.transform.SetScaleFactor(2.0);
                    if (mode == 4) {
                        auto position = change.transform.TranslationPart();
                        position.SetX(position.X() + 1.0);
                        change.transform.SetTranslationPart(position);
                    } else if (mode == 5) {
                        change.transform.SetRotationPart(gp_Quaternion(gp_Vec(0, 0, 1), M_PI_2));
                    }
                }
            }
            changes.push_back(change);
        }
        const auto controller = viewer->debugOrdinaryEditController();
        if (!controller) { return 5; }
        if (paused) { controller->debugSetTruthUnavailableCount(1); }
        core3d::OrdinaryEditResult failure = core3d::OrdinaryEditResult::Invalid;
        auto lease = viewer->beginOrdinaryTransform(changes, &failure);
        return static_cast<NSInteger>(lease ? lease.stageAndCommit() : failure);
    } catch (...) { return 5; }
}

- (void)debugSetViewerOrdinaryRepairFailures:(NSInteger)incremental redraw:(NSInteger)redraw {
    if (![NSThread isMainThread] || GLController == nil || GLController.viewer == nullptr) { return; }
    GLController.viewer->debugSetOrdinaryRepairFailures((int)incremental, (int)redraw);
}

- (void)debugSetViewerOrdinaryVisibilityAfterRepairFailures:(NSInteger)count {
    if (![NSThread isMainThread] || GLController == nil || GLController.viewer == nullptr) { return; }
    GLController.viewer->debugSetOrdinaryVisibilityAfterRepairFailures(static_cast<int>(count));
}

- (BOOL)debugConfigureOrdinaryVisibilityFault:(NSInteger)mode {
    return [self debugConfigureOrdinaryNameFault:mode];
}

- (BOOL)debugConfigureOrdinaryNameFault:(NSInteger)mode {
    if (![self debugConfigureOrdinaryGestureFault:mode]) { return NO; }
    GLController.viewer->debugOrdinaryEditController()->debugSetStageFailureIndex(mode == 4 ? 0 : -1);
    return YES;
}

- (BOOL)debugConfigureOrdinaryCreationFault:(NSInteger)mode {
    if (mode < 0 || mode > 13 || ![self debugConfigureOrdinaryGestureFault:mode <= 5 ? mode : 0]) { return NO; }
    const auto viewer = GLController.viewer;
    const auto controller = viewer->debugOrdinaryEditController();
    auto& stamp = controller->debugCommandStamp();
    viewer->debugSetOrdinaryCreationAfterRepairFailures(mode == 13 ? 1 : 0);
    if (mode >= 6 && mode <= 8) { stamp.debugSetNewCommandMode(static_cast<int>(mode - 5)); }
    if (mode == 9) { stamp.debugSetCommitMode(2); }
    if (mode == 10 || mode == 11) {
        stamp.debugSetCommitMode(1);
        stamp.debugSetAbortMode(mode == 10 ? 1 : 2);
    }
    if (mode == 12) { stamp.debugSetPostCommitInspectionFailureCount(1); }
    return YES;
}

- (BOOL)debugConfigureOrdinaryGestureFault:(NSInteger)mode {
    if (![NSThread isMainThread] || mode < 0 || mode > 5
        || GLController == nil || GLController.viewer == nullptr) { return NO; }
    const auto controller = GLController.viewer->debugOrdinaryEditController();
    if (!controller || controller->blocksNormalWork()) { return NO; }
    auto& stamp = controller->debugCommandStamp();
    stamp.debugSetNewCommandMode(0);
    stamp.debugSetAbortMode(0);
    stamp.debugSetInspectionFailureCount(0);
    stamp.debugSetPostCommitInspectionFailureCount(0);
    stamp.debugSetCommitMode(mode == 1 ? 3 : mode == 2 ? 4 : mode == 3 ? 1 : 0);
    controller->debugSetStageFailureIndex(mode == 4 ? 1 : -1);
    controller->debugSetTruthUnavailableCount(mode == 5 ? 1 : 0);
    return YES;
}

- (NSInteger)debugApplyViewerOrdinaryPivotRotation:(NSInteger)mode {
    if (![NSThread isMainThread] || mode < 0 || mode > 7 || GLController == nil || GLController.viewer == nullptr) { return 5; }
    const auto viewer = GLController.viewer;
    const auto document = viewer->getDocument();
    const auto context = viewer->AisContext();
    if (document.IsNull() || context.IsNull()) { return 5; }
    try {
        core3d::OrdinaryRotationAroundPivot rotation;
        rotation.pivot = gp_Pnt(5, -3, 2);
        rotation.delta.SetRotation(gp_Ax1(rotation.pivot, gp_Dir(0, 0, 1)), M_PI_2);
        if (mode == 7) { rotation.delta = gp_Trsf(); }
        std::vector<core3d::OrdinaryTransformChange> changes;
        for (context->InitSelected(); context->MoreSelected(); context->NextSelected()) {
            core3d::OrdinaryTransformChange change;
            change.presentation = Handle(AIS_Shape)::DownCast(context->SelectedInteractive());
            change.label = document->ShapeLabel(change.presentation);
            OcctObjectTransformState before;
            if (!document->CaptureObjectTransformStateForLabel(change.label, before)) { return 5; }
            change.shape = before.shape;
            change.transform = rotation.delta * before.transform;
            change.operation = core3d::OrdinaryTransformOperation::Rotate;
            change.rotationAroundPivot = rotation;
            if (mode == 1) { change.rotationAroundPivot->pivot.SetX(6); }
            if (mode == 2) { auto pos = change.transform.TranslationPart(); pos.SetX(pos.X() + 1); change.transform.SetTranslationPart(pos); }
            if (mode == 3) { change.rotationAroundPivot->delta.SetScaleFactor(2); }
            if (mode == 4 && !changes.empty()) { change.rotationAroundPivot->pivot.SetZ(3); }
            if (mode == 5) { change.operation = core3d::OrdinaryTransformOperation::Translate; }
            if (mode == 6 && !changes.empty()) { change.rotationAroundPivot.reset(); }
            changes.push_back(change);
        }
        core3d::OrdinaryEditResult failure = core3d::OrdinaryEditResult::Invalid;
        auto lease = viewer->beginOrdinaryTransform(changes, &failure);
        return static_cast<NSInteger>(lease ? lease.stageAndCommit() : failure);
    } catch (...) { return 5; }
}

- (NSInteger)debugReconcileViewerOrdinaryEdit {
    if (![NSThread isMainThread] || GLController == nil || GLController.viewer == nullptr) { return 5; }
    return static_cast<NSInteger>(GLController.viewer->reconcileOrdinaryEdit());
}

- (NSDictionary<NSString *, id> *)debugViewerOrdinaryState {
    if (![NSThread isMainThread] || GLController == nil || GLController.viewer == nullptr) { return @{}; }
    const auto viewer = GLController.viewer;
    const auto controller = viewer->debugOrdinaryEditController();
    const auto document = viewer->getDocument();
    if (!controller || document.IsNull() || document->Document().IsNull()) { return @{}; }
    const bool blocked = viewer->hasUnresolvedOrdinaryEdit();
    NSMutableArray* matrices = [NSMutableArray array];
    TDF_LabelSequence labels;
    XCAFDoc_DocumentTool::ShapeTool(document->Document()->Main())->GetFreeShapes(labels);
    for (int index = 1; index <= labels.Length(); ++index) {
        gp_Trsf transform;
        if (!document->TryObjectTransformForLabel(labels.Value(index), transform)) { continue; }
        NSMutableArray* values = [NSMutableArray array];
        for (int row = 1; row <= 3; ++row) {
            for (int column = 1; column <= 4; ++column) { [values addObject:@(transform.Value(row, column))]; }
        }
        [matrices addObject:values];
    }
    return @{@"blocked": @(blocked), @"state": @((int)controller->state()),
             @"canBegin": @(viewer->canBeginCommittedEdit()), @"selected": @(viewer->selectedCount()),
             @"undoCount": @(document->Document()->GetAvailableUndos()), @"matrices": matrices,
             @"redrawAttempts": @(viewer->debugOrdinaryRedrawAttempts()),
             @"snapshotBlocked": @(blocked && !viewer->captureSceneSnapshot(800, 600)),
             @"loadBlocked": @(blocked && viewer->ImportCbf("") == core3d::AssetImportResult::Busy),
             @"redrawBlocked": @(blocked && !viewer->redrawDocument())};
}

- (NSDictionary<NSString *, NSNumber *> *_Nullable)debugProbeOrdinaryController:(NSInteger)mode {
    if (![NSThread isMainThread] || mode < 0 || mode > 19
        || GLController == nil || GLController.viewer == nullptr) { return nil; }
    const auto viewer = GLController.viewer;
    const Handle(OcctDocument) document = viewer->getDocument();
    const auto ocaf = document.IsNull() ? Handle(TDocStd_Document)() : document->Document();
    const auto context = viewer->AisContext();
    if (ocaf.IsNull() || ocaf->HasOpenCommand() || context.IsNull()) { return nil; }
    using Result = core3d::OrdinaryEditResult;
    using State = core3d::OrdinaryEditState;
    struct FixtureHost final : core3d::OrdinaryEditPresentationHost {
        Handle(AIS_InteractiveContext) context;
        core3d::OrdinaryEditController* controller = nullptr;
        int failureMode = 0;
        int repairs = 0;
        bool reentrantRepairBlocked = true;
        bool admitTransform(core3d::OrdinaryTransformLedger& ledger) noexcept override {
            for (const auto& record : ledger.records) {
                if (!context->IsDisplayed(record.requested.presentation)) { return false; }
            }
            return true;
        }
        bool repairTransform(const core3d::OrdinaryTransformLedger& ledger, bool committed) noexcept override {
            ++repairs;
            reentrantRepairBlocked = reentrantRepairBlocked && controller
                && controller->blocksNormalWork() && controller->reconcile() == Result::Busy;
            const int fault = std::exchange(failureMode, 0);
            if (fault == 1) { return false; }
            try {
                int index = 0;
                for (const auto& record : ledger.records) {
                    const auto& expected = committed ? record.candidate : record.previous;
                    record.requested.presentation->SetShape(expected.shape);
                    record.requested.presentation->SetLocalTransformation(expected.transform);
                    context->Redisplay(record.requested.presentation, Standard_False);
                    if (fault == 2 && index++ == 0) { return false; }
                }
                return true;
            } catch (...) { return false; }
        }
    } host;
    host.context = context;
    std::vector<core3d::OrdinaryTransformChange> changes;
    std::vector<OcctObjectTransformState> previous;
    AIS_ListOfInteractive displayed;
    context->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
    for (AIS_ListIteratorOfListOfInteractive it(displayed); it.More() && changes.size() < 2; it.Next()) {
        const auto presentation = Handle(AIS_Shape)::DownCast(it.Value());
        if (presentation.IsNull()) { continue; }
        const TDF_Label label = document->ShapeLabel(presentation);
        OcctObjectTransformState baseline;
        if (!document->CaptureObjectTransformStateForLabel(label, baseline)) { continue; }
        core3d::OrdinaryTransformChange change;
        change.label = label;
        change.presentation = presentation;
        change.shape = baseline.shape;
        change.transform = baseline.transform;
        if (mode != 14) {
            auto translation = change.transform.TranslationPart();
            translation.SetX(translation.X() + 12.5 * (changes.size() + 1));
            change.transform.SetTranslationPart(translation);
        }
        if (mode == 18) {
            change.transform.SetRotationPart(gp_Quaternion(gp_Vec(0, 0, 1), M_PI_2));
        }
        if (mode == 19) { change.operation = core3d::OrdinaryTransformOperation::Rotate; }
        changes.push_back(change);
        previous.push_back(baseline);
    }
    if (changes.size() != 2) { return nil; }
    if (mode == 15) { changes.push_back(changes.front()); }
    const auto controller = std::make_shared<core3d::OrdinaryEditController>(document, host);
    host.controller = controller.get();
    if (mode == 1) { controller->debugCommandStamp().debugSetCommitMode(3); }
    if (mode == 2) { controller->debugCommandStamp().debugSetCommitMode(4); }
    if (mode == 3 || mode == 10) { controller->debugCommandStamp().debugSetCommitMode(1); }
    if (mode == 4) { controller->debugCommandStamp().debugSetCommitMode(2); }
    if (mode == 5 || mode == 12) { controller->debugCommandStamp().debugSetNewCommandMode(3); }
    if (mode == 6) { controller->debugSetStageFailureIndex(1); }
    if (mode == 7) { controller->debugSetTruthUnavailableCount(1); }
    if (mode == 8) { host.failureMode = 1; }
    if (mode == 9) { host.failureMode = 2; }
    if (mode == 10 || mode == 12) { controller->debugCommandStamp().debugSetAbortMode(1); }
    if (mode == 13) { controller->debugCommandStamp().debugSetPostCommitInspectionFailureCount(1); }
    __block int notifications = 0;
    __block bool publishingBlocked = true;
    id observer = [NSNotificationCenter.defaultCenter addObserverForName:@"OcctDocumentChanges"
        object:nil queue:nil usingBlock:^(NSNotification* notification) {
        if (![notification.object isKindOfClass:NSValue.class]
            || [(NSValue*)notification.object pointerValue] != document.get()) { return; }
        ++notifications;
        Result failure = Result::Invalid;
        auto reentrant = controller->beginTransform(changes, &failure);
        publishingBlocked = publishingBlocked && !reentrant && failure == Result::Busy
            && controller->state() == State::Publishing && controller->blocksNormalWork();
    }];
    const Standard_Integer undoBefore = ocaf->GetAvailableUndos();
    try {
        Result failure = Result::Invalid;
        Result first = Result::Invalid;
        {
            auto lease = controller->beginTransform(changes, &failure);
            if (!lease) { first = failure; }
            else if (mode == 11) { first = lease.cancel(); }
            else if (mode == 16) {
                auto movedLease = std::move(lease);
                first = Result::NoChange;
            } else if (mode == 17) {
                std::thread worker([abandoned = std::move(lease)]() mutable {
                    abandoned = core3d::OrdinaryEditLease();
                });
                worker.join();
                first = Result::OutcomeUnknown;
            } else { first = lease.stageAndCommit(); }
        }
        const bool blocked = controller->blocksNormalWork();
        const auto firstState = controller->state();
        const auto historyAfterFirst = ocaf->GetAvailableUndos();
        // Only the fixture owner closes its deliberately unproven begin after
        // verifying that ordinary reconciliation does not claim abort authority.
        bool unprovenProtected = true;
        if (mode == 12) {
            unprovenProtected = ocaf->HasOpenCommand()
                && controller->reconcile() == Result::OutcomeUnknown
                && ocaf->HasOpenCommand();
            ocaf->AbortCommand();
        }
        const Result retry = blocked ? controller->reconcile() : first;
        const bool committed = retry == Result::Committed;
        bool exact = true;
        for (std::size_t index = 0; index < previous.size(); ++index) {
            OcctObjectTransformState actual;
            exact = exact && document->CaptureObjectTransformStateForLabel(previous[index].label, actual);
            if (committed) {
                exact = exact && actual.scalars[0] == changes[index].transform.TranslationPart().X()
                    && actual.shape.IsEqual(previous[index].shape)
                    && actual.entityIdentifier == previous[index].entityIdentifier;
            } else { exact = exact && actual.IsEqual(previous[index]); }
        }
        const int notifyCount = notifications;
        const int repairCount = host.repairs;
        const bool idle = !controller->blocksNormalWork() && controller->state() == State::Idle;
        const auto delta = ocaf->GetAvailableUndos() - undoBefore;
        const bool retryDidNotAddHistory = ocaf->GetAvailableUndos() == historyAfterFirst;
        [NSNotificationCenter.defaultCenter removeObserver:observer];
        observer = nil;
        if (committed) { ocaf->Undo(); }
        bool restored = true;
        for (std::size_t index = 0; index < previous.size(); ++index) {
            OcctObjectTransformState actual;
            restored = restored && document->CaptureObjectTransformStateForLabel(previous[index].label, actual)
                && actual.IsEqual(previous[index]);
            changes[index].presentation->SetShape(previous[index].shape);
            changes[index].presentation->SetLocalTransformation(previous[index].transform);
            context->Redisplay(changes[index].presentation, Standard_False);
        }
        return @{@"first": @((int)first), @"retry": @((int)retry), @"firstState": @((int)firstState),
                 @"blocked": @(blocked), @"idle": @(idle), @"exact": @(exact), @"restored": @(restored),
                 @"undoDelta": @(delta), @"retryDidNotAddHistory": @(retryDidNotAddHistory),
                 @"notifications": @(notifyCount), @"repairs": @(repairCount),
                 @"publishingBlocked": @(publishingBlocked), @"repairReentryBlocked": @(host.reentrantRepairBlocked),
                 @"unprovenProtected": @(unprovenProtected), @"closed": @(!ocaf->HasOpenCommand())};
    } catch (...) {
        [NSNotificationCenter.defaultCenter removeObserver:observer];
        Core3DAbortCommandNoThrow(ocaf);
        return nil;
    }
}

- (NSDictionary<NSString *, NSNumber *> *)debugOrdinaryCommandMarkerState {
    if (![NSThread isMainThread] || GLController == nil || GLController.viewer == nullptr) {
        return @{@"valid": @NO};
    }
    try {
        const Handle(OcctDocument) document = GLController.viewer->getDocument();
        if (document.IsNull() || document->Document().IsNull()) { return @{@"valid": @NO}; }
        Handle(TDF_Attribute) attribute;
        const bool present = document->Document()->Main().FindAttribute(
            Core3DOrdinaryEditCommandOwnerAttributeID(), attribute);
        const Handle(TDataStd_Integer) marker = Handle(TDataStd_Integer)::DownCast(attribute);
        return @{@"valid": @(!present || !marker.IsNull()), @"present": @(present),
                 @"value": @(marker.IsNull() ? 0 : marker->Get())};
    } catch (...) { return @{@"valid": @NO}; }
}

- (NSDictionary<NSString *, NSNumber *> *_Nullable)debugProbeOrdinaryCommand:(NSInteger)mode {
    if (![NSThread isMainThread] || mode < 0 || mode > 15
        || GLController == nil || GLController.viewer == nullptr) { return nil; }
    const Handle(OcctDocument) document = GLController.viewer->getDocument();
    const Handle(TDocStd_Document) ocaf = document.IsNull()
        ? Handle(TDocStd_Document)() : document->ChangeDocument();
    if (ocaf.IsNull() || ocaf->HasOpenCommand()) { return nil; }
    const TDF_Label label = Core3DFirstFreeSimpleDefinition(ocaf);
    OcctObjectTransformState previous;
    if (!document->CaptureObjectTransformStateForLabel(label, previous)) { return nil; }
    const Standard_Integer undoBefore = ocaf->GetAvailableUndos();
    core3d::OrdinaryEditCommandStamp stamp;
    using Begin = core3d::OrdinaryCommandBeginResult;
    using Observation = core3d::OrdinaryCommandObservation;
    try {
        if (mode >= 1 && mode <= 3) { stamp.debugSetNewCommandMode((int)mode); }
        if (mode == 14) {
            stamp.debugSetNewCommandMode(3);
            stamp.debugSetAbortMode(1);
        }
        if (mode == 12) { ocaf->NewCommand(); }
        Begin begin = Begin::Invalid;
        if (mode == 15) {
            // Synchronous test seam; production immediately rejects before
            // inspecting any OCAF handle on this non-main thread.
            std::thread worker([&] { begin = stamp.begin(document); });
            worker.join();
        } else {
            begin = stamp.begin(document);
        }
        const bool retainedAfterBegin = stamp.isRetained();
        Observation first = Observation::Unavailable;
        Observation retried = Observation::Unavailable;
        bool protectedForeign = true;
        bool staged = false;
        if (begin == Begin::Started) {
            staged = document->SetObjectPositionComponentForLabel(label, 0, 12.5);
            if (!staged) { (void)stamp.abortAndObserve(); return nil; }
            if (mode == 11) {
                Handle(TDataStd_Integer) marker;
                if (!ocaf->Main().FindAttribute(Core3DOrdinaryEditCommandOwnerAttributeID(), marker)
                    || marker.IsNull()) { throw Standard_Failure("Missing owned marker"); }
                const Standard_Integer ownedValue = marker->Get();
                TDataStd_Integer::Set(ocaf->Main(), Core3DOrdinaryEditCommandOwnerAttributeID(), ownedValue + 17);
                first = stamp.abortAndObserve();
                protectedForeign = ocaf->HasOpenCommand();
                TDataStd_Integer::Set(ocaf->Main(), Core3DOrdinaryEditCommandOwnerAttributeID(), ownedValue);
                retried = stamp.abortAndObserve();
                protectedForeign = protectedForeign && ocaf->HasOpenCommand()
                    && !stamp.releaseClosed();
            } else if (mode == 0 || mode == 9 || mode == 10) {
                if (mode == 9) { stamp.debugSetAbortMode(1); }
                if (mode == 10) { stamp.debugSetAbortMode(2); }
                first = stamp.abortAndObserve();
                retried = stamp.abortAndObserve();
            } else {
                if (mode >= 5 && mode <= 8) { stamp.debugSetCommitMode((int)mode - 4); }
                if (mode == 13) { stamp.debugSetPostCommitInspectionFailureCount(1); }
                first = stamp.commitAndObserve();
                retried = first == Observation::OpenOwned
                    ? stamp.abortAndObserve() : stamp.observe();
            }
        } else if (mode == 12 || mode == 14) {
            first = stamp.abortAndObserve();
            retried = stamp.observe();
            protectedForeign = ocaf->HasOpenCommand() && !stamp.releaseClosed();
        }
        const bool retainedBeforeCleanup = stamp.isRetained();
        const bool openBeforeCleanup = ocaf->HasOpenCommand();
        const bool committed = mode == 4 || mode == 7 || mode == 8 || mode == 13;
        OcctObjectTransformState actual;
        const bool captured = document->CaptureObjectTransformStateForLabel(label, actual);
        const bool exactExpected = captured && (committed
            ? actual.scalars[0] == 12.5 && actual.shape.IsEqual(previous.shape)
                && actual.entityIdentifier == previous.entityIdentifier
                && actual.definitionIdentifier == previous.definitionIdentifier
            : openBeforeCleanup || actual.IsEqual(previous));
        const Standard_Integer delta = ocaf->GetAvailableUndos() - undoBefore;
        // Only the fixture driver cleans its deliberately foreign/unproven
        // command, after recording that the production stamp refused to do so.
        // No callbacks or another tool have run during this synchronous probe.
        if (openBeforeCleanup) { ocaf->AbortCommand(); }
        const bool released = !stamp.isRetained() || stamp.releaseClosed();
        // Restore committed fixtures through the real history machinery.
        if (committed) { ocaf->Undo(); }
        OcctObjectTransformState restored;
        const bool exactRestored = document->CaptureObjectTransformStateForLabel(label, restored)
            && restored.IsEqual(previous);
        return @{@"begin": @((int)begin), @"first": @((int)first), @"retry": @((int)retried),
                 @"retainedAfterBegin": @(retainedAfterBegin),
                 @"retainedBeforeCleanup": @(retainedBeforeCleanup),
                 @"openBeforeCleanup": @(openBeforeCleanup), @"protectedForeign": @(protectedForeign),
                 @"staged": @(staged), @"exactExpected": @(exactExpected),
                 @"undoDelta": @(delta), @"released": @(released),
                 @"restored": @(exactRestored), @"closed": @(!ocaf->HasOpenCommand())};
    } catch (...) {
        Core3DAbortCommandNoThrow(ocaf);
        return nil;
    }
}

- (void)debugSetDuplicateCommitMode:(NSInteger)mode {
    [GLController debugSetDuplicateCommitMode:mode];
}

- (NSData *_Nullable)debugReferenceAxisFixtureDataWithMode:
    (Core3DDebugReferenceAxisFixtureMode)mode {
    if (mode < Core3DDebugReferenceAxisFixtureValidMixedSpace
        || mode
            > Core3DDebugReferenceAxisFixtureOrdinarySentinelValid) {
        return nil;
    }
    return Core3DCreateDebugBinXCAFFixture(
        [NSString stringWithFormat:@"reference-axis-%ld", (long)mode],
        [mode](const Handle(TDocStd_Document)& document) {
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            if (shapeTool.IsNull()) {
                throw Standard_Failure(
                    "Unable to create reference-axis fixture shape tool");
            }
            const TDF_Label label = Core3DAddDebugGeometryDefinition(
                shapeTool,
                BRepPrimAPI_MakeBox(
                    gp_Pnt(-10.0, -10.0, -10.0),
                    20.0, 20.0, 20.0).Shape());
            Core3DSetDebugGeometryRepresentation(
                label,
                static_cast<Standard_Integer>(
                    OcctGeometryRepresentation::BRep));
            if (mode
                == Core3DDebugReferenceAxisFixtureDuplicateSentinelWrongType) {
                TDataStd_Real::Set(
                    document->Main(),
                    Core3DDebugDuplicateCommandOwnerAttributeID(),
                    1.0);
                return;
            }
            if (mode
                == Core3DDebugReferenceAxisFixtureDuplicateSentinelMisplaced) {
                TDataStd_Integer::Set(
                    label,
                    Core3DDebugDuplicateCommandOwnerAttributeID(),
                    1);
                return;
            }
            if (mode
                == Core3DDebugReferenceAxisFixtureRadialSentinelWrongType) {
                TDataStd_Real::Set(
                    document->Main(),
                    Core3DDebugRadialArrayCommandOwnerAttributeID(),
                    1.0);
                return;
            }
            if (mode
                == Core3DDebugReferenceAxisFixtureRadialSentinelMisplaced) {
                TDataStd_Integer::Set(
                    label,
                    Core3DDebugRadialArrayCommandOwnerAttributeID(),
                    1);
                return;
            }
            if (mode == Core3DDebugReferenceAxisFixtureOrdinarySentinelWrongType) {
                TDataStd_Real::Set(document->Main(),
                    Core3DOrdinaryEditCommandOwnerAttributeID(), 1.0);
                return;
            }
            if (mode == Core3DDebugReferenceAxisFixtureOrdinarySentinelMisplaced) {
                TDataStd_Integer::Set(label,
                    Core3DOrdinaryEditCommandOwnerAttributeID(), 1);
                return;
            }
            if (mode
                == Core3DDebugReferenceAxisFixtureImplicitDefaultCorruptTransform) {
                const TDF_Label scaleLabel =
                    label.FindChild(8, Standard_True);
                if (scaleLabel.IsNull()) {
                    throw Standard_Failure(
                        "Unable to create implicit-axis corrupt transform");
                }
                TDataStd_Real::Set(scaleLabel, 0.0);
                return;
            }
            if (mode
                == Core3DDebugReferenceAxisFixtureImplicitDefaultOversizedOccurrence) {
                const TDF_Label rootAssembly = shapeTool->NewShape();
                gp_Trsf oversizedLocation;
                oversizedLocation.SetTranslation(
                    gp_Vec(1'000'001.0, 0.0, 0.0));
                const TDF_Label occurrence = shapeTool->AddComponent(
                    rootAssembly,
                    label,
                    TopLoc_Location(oversizedLocation));
                if (rootAssembly.IsNull() || occurrence.IsNull()) {
                    throw Standard_Failure(
                        "Unable to create oversized axis occurrence");
                }
                return;
            }
            const TDF_Label target =
                mode == Core3DDebugReferenceAxisFixtureOrphanRecord
                ? document->Main().FindChild(97, Standard_True)
                : label;
            if (mode == Core3DDebugReferenceAxisFixtureOrdinarySentinelValid) {
                TDataStd_Integer::Set(document->Main(),
                    Core3DOrdinaryEditCommandOwnerAttributeID(),
                    std::numeric_limits<Standard_Integer>::max());
            }
            Core3DWriteDebugReferenceAxisRecord(target,
                mode == Core3DDebugReferenceAxisFixtureOrdinarySentinelValid
                    ? Core3DDebugReferenceAxisFixtureValidMixedSpace : mode);
        });
}

- (BOOL)debugImportSTEPAtURL:(NSURL *)url {
    if (![NSThread isMainThread]
        || !url.isFileURL
        || GLController == nil) {
        return NO;
    }
    const char *path = url.path.fileSystemRepresentation;
    const std::shared_ptr<core3d::Core3DViewer> viewer =
        GLController.viewer;
    if (path == nullptr
        || viewer == nullptr
        || !viewer->ImportSTEP(std::string(path))) {
        return NO;
    }
    viewer->FitAll();
    [GLController requestRender];
    return YES;
}

- (NSData *_Nullable)debugMeterLengthUnitBinXCAFFixtureData {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL* baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.meter-length-unit-fixture", NSUUID.UUID.UUIDString]];
    NSString* xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSData* result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        if (document.IsNull()) {
            throw Standard_Failure(
                "Unable to create meter length-unit fixture document");
        }
        Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapeTool.IsNull()) {
            throw Standard_Failure(
                "Unable to create meter length-unit fixture shape tool");
        }
        XCAFDoc_DocumentTool::SetLengthUnit(document, 1.0);
        const TDF_Label shapeLabel = shapeTool->AddShape(
            BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape(),
            Standard_False,
            Standard_True);
        if (shapeLabel.IsNull()) {
            throw Standard_Failure(
                "Unable to create meter length-unit fixture shape");
        }
        if (application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure(
                "Unable to save meter length-unit fixture");
        }
        result = [NSData dataWithContentsOfFile:xbfPath];
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return result;
}

- (NSData *_Nullable)debugNegativeLocationBinXCAFFixtureData {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL* baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.negative-location-fixture", NSUUID.UUID.UUIDString]];
    NSString* xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSData* result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        Handle(XCAFDoc_ShapeTool) shapeTool = document.IsNull()
            ? Handle(XCAFDoc_ShapeTool)()
            : XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (document.IsNull() || shapeTool.IsNull()) {
            throw Standard_Failure(
                "Unable to create negative-location fixture document");
        }
        XCAFDoc_DocumentTool::SetLengthUnit(document, 0.001);
        TopoDS_Shape box = BRepPrimAPI_MakeBox(
            gp_Pnt(10.0, 0.0, 0.0),
            10.0,
            20.0,
            30.0).Shape();
        gp_Trsf mirror;
        mirror.SetMirror(gp_Ax2(
            gp_Pnt(0.0, 0.0, 0.0),
            gp_Dir(1.0, 0.0, 0.0)));
        if (!mirror.IsNegative()) {
            throw Standard_Failure(
                "Negative-location fixture transform is not mirrored");
        }
        box.Location(TopLoc_Location(mirror), Standard_False);
        if (shapeTool->AddShape(
                box,
                Standard_False,
                Standard_True).IsNull()
            || application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure(
                "Unable to save negative-location fixture");
        }
        result = [NSData dataWithContentsOfFile:xbfPath];
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return result;
}

- (NSData *_Nullable)debugInvalidLengthUnitBinXCAFFixtureData {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL* baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.invalid-length-unit-fixture", NSUUID.UUID.UUIDString]];
    NSString* xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSData* result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        Handle(XCAFDoc_ShapeTool) shapeTool = document.IsNull()
            ? Handle(XCAFDoc_ShapeTool)()
            : XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (document.IsNull() || shapeTool.IsNull()) {
            throw Standard_Failure(
                "Unable to create invalid length-unit fixture");
        }
        XCAFDoc_DocumentTool::SetLengthUnit(document, -1.0);
        if (shapeTool->AddShape(
                BRepPrimAPI_MakeBox(1.0, 1.0, 1.0).Shape(),
                Standard_False,
                Standard_True).IsNull()
            || application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure(
                "Unable to save invalid length-unit fixture");
        }
        result = [NSData dataWithContentsOfFile:xbfPath];
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return result;
}

- (NSData *_Nullable)debugLegacyBinOcafFixtureData {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL* baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.legacy-fixture", NSUUID.UUID.UUIDString]];
    NSString* cbfPath = [baseURL.path stringByAppendingString:@".cbf"];
    NSData* result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinOcaf"), document);
        if (document.IsNull()) {
            throw Standard_Failure("Unable to create legacy fixture document");
        }
        Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapeTool.IsNull()) {
            throw Standard_Failure("Unable to create legacy shape tool");
        }
        const TopoDS_Shape cube = BRepPrimAPI_MakeBox(
            gp_Pnt(-25.0, -25.0, 0.0), 50.0, 50.0, 50.0).Shape();
        const TDF_Label label = shapeTool->AddShape(
            cube, Standard_False, Standard_True);
        if (label.IsNull()) {
            throw Standard_Failure("Unable to create legacy fixture shape");
        }
        TDataStd_Integer::Set(
            label.FindChild(11),
            Graphic3d_NameOfMaterial_ShinyPlastified);
        TDataStd_Integer::Set(
            label.FindChild(12), Quantity_NOC_BLUE);
        if (application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure("Unable to save legacy fixture");
        }
        result = [NSData dataWithContentsOfFile:cbfPath];
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:cbfPath error:nil];
    return result;
}

- (NSInteger)debugDocumentUndoCount {
    auto document = GLController.viewer->getDocument()->ChangeDocument();
    return document.IsNull()
        ? 0
        : static_cast<NSInteger>(document->GetAvailableUndos());
}

- (NSArray<NSDictionary<NSString *, NSNumber *> *> *_Nullable)
    debugReplayGesture:(NSInteger)mode axis:(NSInteger)axis
    values:(NSArray<NSNumber *> *)values {
    if (![NSThread isMainThread] || mode < 0 || mode > 3 || axis < 0 || axis > 2
        || values.count < 2 || values.count > 32 || GLController.viewer == nullptr) {
        return nil;
    }
    [self setGizmoType:mode >= 2 ? PrimitiveGizmoTypeScale : PrimitiveGizmoTypeMoveRotate];
    const auto interactor = GLController.viewer->getObjectInteractor();
    if (interactor == nullptr) { return nil; }
    std::vector<Standard_Real> samples;
    for (NSNumber *value in values) {
        if (![value isKindOfClass:NSNumber.class]) { return nil; }
        samples.push_back(value.doubleValue);
    }
    std::vector<std::array<Standard_Real, 2>> observed;
    if (!interactor->debugReplayGesture(
            static_cast<Standard_Integer>(mode), static_cast<Standard_Integer>(axis),
            samples, observed)) {
        return nil;
    }
    NSMutableArray<NSDictionary<NSString *, NSNumber *> *> *result =
        [NSMutableArray arrayWithCapacity:observed.size()];
    for (const auto& sample : observed) {
        [result addObject:@{
            @"transformDelta": @(sample[0]),
            @"changedShapeCount": @(sample[1]),
        }];
    }
    return result;
}

- (NSNumber *_Nullable)debugDocumentMetersPerUnit {
    auto document = GLController.viewer->getDocument()->ChangeDocument();
    Standard_Real metersPerUnit = 0.0;
    if (document.IsNull()
        || !XCAFDoc_DocumentTool::GetLengthUnit(document, metersPerUnit)) {
        return nil;
    }
    return @(metersPerUnit);
}

- (NSDictionary<NSString *, id> *)debugNativeAuthoredFrameRendering:(NSData *)archive replacement:(NSData *)replacement normalPNG:(NSData *)normalPNG mode:(NSInteger)mode {
    if (![NSThread isMainThread] || !_isSetuped || !GLController.viewer || mode < 0 || mode > 4)
        return @{@"error": @"Unavailable native frame fixture"};
    const auto viewer = GLController.viewer;
    const auto context = viewer->AisContext(); const auto view = viewer->ActiveView();
    if (context.IsNull() || view.IsNull()) return @{@"error": @"No native viewport"};
    AIS_ListOfInteractive resident; context->ObjectsInside(resident, AIS_KOI_Shape, -1);
    if (!resident.IsEmpty()) return @{@"error": @"Native fixture requires an empty viewport"};
    struct Scope {
        Handle(TDocStd_Application) app = new TDocStd_Application();
        Handle(TDocStd_Application) reopenApp = new TDocStd_Application();
        Handle(TDocStd_Document) document, reopened;
        Handle(AIS_InteractiveContext) context;
        Handle(V3d_View) view;
        Handle(Graphic3d_Camera) camera;
        Handle(CafShapePrs) presentation;
        Handle(V3d_Viewer) viewer;
        bool gridActive = false, automaticHilight = true;
        Aspect_GridType gridType = Aspect_GT_Rectangular;
        Aspect_GridDrawMode gridDrawMode = Aspect_GDM_Lines;
        ~Scope() noexcept {
            try { if (!presentation.IsNull()) context->Remove(presentation, Standard_False); } catch (...) {}
            try { view->SetCamera(camera); context->SetAutomaticHilight(automaticHilight); } catch (...) {}
            try { if (gridActive) viewer->ActivateGrid(gridType, gridDrawMode); else viewer->DeactivateGrid(); } catch (...) {}
            try { if (!document.IsNull()) app->Close(document); } catch (...) {}
            try { if (!reopened.IsNull()) reopenApp->Close(reopened); } catch (...) {}
        }
    } scope;
    scope.context = context; scope.view = view; scope.camera = new Graphic3d_Camera(view->Camera());
    scope.viewer = viewer->V3dViewer(); scope.gridActive = scope.viewer->IsGridActive();
    scope.gridType = scope.viewer->GridType(); scope.gridDrawMode = scope.viewer->GridDrawMode();
    scope.automaticHilight = context->AutomaticHilight();
    scope.viewer->DeactivateGrid(); context->SetAutomaticHilight(Standard_False);
    int fixtureStep = -1;
    try {
        using namespace core3d::persistence;
        Core3DDebugDefineFrameBinXCAFFormat(scope.app, std::make_shared<AuthoredFrameReadBudget>());
        Core3DDebugDefineFrameBinXCAFFormat(scope.reopenApp, std::make_shared<AuthoredFrameReadBudget>());
        scope.app->NewDocument(TCollection_ExtendedString("BinXCAF"), scope.document);
        XCAFDoc_DocumentTool::SetLengthUnit(scope.document, 0.001);
        const Handle(OcctDocument) wrapper = new OcctDocument(); wrapper->ChangeDocument() = scope.document;
        auto shapes = XCAFDoc_DocumentTool::ShapeTool(scope.document->Main());
        auto shape = TopoDS_Shape(Core3DDebugAuthoredGeometryFixture(mode == 4 ? 22 : 0));
        if (mode == 2) { BRep_Builder b; TopoDS_Compound compound; b.MakeCompound(compound); b.Add(compound, shape); shape = compound; }
        TDF_Label label = shapes->AddShape(shape, Standard_False);
        gp_Trsf placement;
        if (mode == 1 || mode == 2) {
            placement.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0,0,1)), M_PI / 2);
            placement.SetTranslationPart(gp_Vec(2,3,4)); shape.Location(TopLoc_Location(placement)); shapes->SetShape(label, shape);
        }
        scope.document->SetUndoLimit(20); scope.document->NewCommand();
        if (!wrapper->SetGeometryRepresentationForLabel(label, OcctGeometryRepresentation::TriangleMesh)) Standard_Failure::Raise("Native fixture representation failed.");
        Handle(Image_Texture) image;
        if (!Core3DCreateAuthoredTexture(static_cast<const Standard_Byte*>(normalPNG.bytes), normalPNG.length, "image/png", image)
            || !Core3DValidateNumericTexture(image)) Standard_Failure::Raise("Native fixture image rejected.");
        Handle(XCAFDoc_VisMaterial) material = new XCAFDoc_VisMaterial();
        XCAFDoc_VisMaterialPBR pbr; pbr.BaseColor = Quantity_ColorRGBA(1,1,1,1); pbr.Metallic = 0; pbr.Roughness = 0.8f; pbr.NormalTexture = image;
        material->SetPbrMaterial(pbr); material->SetFaceCulling(Graphic3d_TypeOfBackfacingModel_DoubleSided);
        const auto materials = XCAFDoc_DocumentTool::VisMaterialTool(scope.document->Main());
        const auto materialLabel = materials->AddMaterial(material, "Native frame fixture"); materials->SetShapeMaterial(label, materialLabel);
        auto assign = [&](NSData* value) {
            if (value.length < 128 || value.length > core3d::scene::authored::kMaximumArchiveBytes) Standard_Failure::Raise("Invalid native frame archive bound.");
            const auto attribute = TDataStd_ByteArray::Set(label, AuthoredFrameAttributeID(), 0, int(value.length)-1, Standard_False);
            const auto* bytes = static_cast<const std::uint8_t*>(value.bytes);
            for (NSUInteger i = 0; i < value.length; ++i) attribute->SetValue(int(i), bytes[i]);
        };
        assign(archive); scope.document->CommitCommand(); scope.document->ClearUndos();
        auto display = [&]() {
            const auto nativeMaterial = XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
            XCAFPrs_Style style; style.SetMaterial(nativeMaterial);
            scope.presentation = new CafShapePrs(label, style, Graphic3d_MaterialAspect(Graphic3d_NameOfMaterial_ShinyPlastified));
            scope.presentation->DispatchStyles(Standard_False);
            if (mode == 3) { gp_Trsf scalar; scalar.SetScale(gp_Pnt(0,0,0), -2); scope.presentation->SetLocalTransformation(scalar); }
            context->Display(scope.presentation, AIS_Shaded, -1, Standard_False);
        };
        display();
        context->Activate(scope.presentation, 0, Standard_False);
        context->AddOrRemoveSelected(scope.presentation, Standard_False);
        if (context->NbSelected() != 1) Standard_Failure::Raise("Native fixture selection unavailable.");
        const auto direction = gp_Dir(0.6,0.8,0).Transformed(placement);
        view->SetProj(direction.X(), direction.Y(), direction.Z());
        view->Camera()->SetProjectionType(Graphic3d_Camera::Projection_Orthographic);
        view->Camera()->SetUp(gp_Dir(0,0,1));
        auto center = gp_Pnt(8.0/3, -2, 10.0/3).Transformed(placement);
        if (mode == 3) { center.SetCoord(-2*center.X(), -2*center.Y(), -2*center.Z()); }
        view->Camera()->SetCenter(center); view->Camera()->SetScale(mode == 3 ? 26 : 13);
        NSMutableArray* states = [NSMutableArray array]; NSMutableDictionary* captures = [NSMutableDictionary dictionary];
        bool readOnly = true, selectionUnchanged = true;
        auto selectedOwners = [&]() {
            std::vector<Handle(SelectMgr_EntityOwner)> owners;
            for (context->InitSelected(); context->MoreSelected(); context->NextSelected()) owners.push_back(context->SelectedOwner());
            return owners;
        };
        auto selectedModes = [&]() {
            TColStd_ListOfInteger modes; context->ActivatedModes(scope.presentation, modes);
            std::vector<int> result;
            for (TColStd_ListIteratorOfListOfInteger it(modes); it.More(); it.Next()) result.push_back(it.Value());
            std::sort(result.begin(), result.end()); return result;
        };
        auto keep = [&](int step, bool freshBuffer, bool captureImage) {
            fixtureStep = step;
            const auto ownersBefore = selectedOwners(); const auto modesBefore = selectedModes();
            const int time = wrapper->Document()->GetData()->Time(), undo = wrapper->Document()->GetAvailableUndos(), redo = wrapper->Document()->GetAvailableRedos();
            NSMutableData* frames = [NSMutableData data]; NSMutableArray* uids = [NSMutableArray array];
            bool observed = false;
            struct ObserverReset { OcctViewer* viewer; ~ObserverReset() { viewer->DebugSetAfterTangentPreparation({}); } } observerReset{viewer.get()};
            viewer->DebugSetAfterTangentPreparation([&]() {
                for (const auto& prs : scope.presentation->Presentations()) {
                    if (prs.IsNull()) continue;
                    for (const auto& generic : prs->Groups()) {
                        auto group = Handle(OpenGl_Group)::DownCast(generic); if (group.IsNull()) continue;
                        for (auto* node = group->FirstNode(); node; node = node->next) {
                            auto* primitive = dynamic_cast<OpenGl_PrimitiveArray*>(node->elem);
                            if (!primitive || !primitive->IsFillDrawMode()) continue;
                            [uids addObject:@(primitive->GetUID())];
                            const auto attributes = primitive->Attributes();
                            if (!freshBuffer) continue;
                            if (attributes.IsNull() || !attributes->Data()) Standard_Failure::Raise("New native frame buffer missing.");
                            int index = -1; Standard_Size stride = 0;
                            const auto bytes = attributes->AttributeData(Graphic3d_TOA_CUSTOM, index, stride);
                            if (!bytes || index < 0 || attributes->Attribute(index).DataType != Graphic3d_TOD_VEC4) Standard_Failure::Raise("Native tangent attribute missing.");
                            for (int corner = 0; corner < attributes->NbElements; ++corner) [frames appendBytes:bytes+std::size_t(corner)*stride length:16];
                        }
                    }
                }
                observed = true; return true;
            });
            const auto stats = [GLController debugFramebufferStatistics];
            viewer->DebugSetAfterTangentPreparation({});
            if (!observed || ![stats[@"captured"] boolValue]) Standard_Failure::Raise("Native framebuffer preparation or capture failed.");
            selectionUnchanged = selectionUnchanged && ownersBefore == selectedOwners() && modesBefore == selectedModes();
            readOnly = readOnly && time == wrapper->Document()->GetData()->Time() && undo == wrapper->Document()->GetAvailableUndos() && redo == wrapper->Document()->GetAvailableRedos();
            [states addObject:@{@"step": @(step), @"frames": frames, @"uids": uids, @"stats": stats,
                @"prepared": @(viewer->DebugPreparedTangentArrayCount()), @"selectedOwners": @(ownersBefore.size()), @"selectionUnchanged": @(selectionUnchanged), @"history": @[@(undo),@(redo)]}];
            if (captureImage) {
                const auto pixels = [GLController debugViewportRGBA];
                if (!pixels) Standard_Failure::Raise("Native viewport image failed.");
                captures[[NSString stringWithFormat:@"%d",step]] = pixels;
            }
        };
        keep(0, true, true); keep(1, false, false);
        scope.document->NewCommand(); assign(replacement); scope.document->CommitCommand(); keep(2, true, true);
        if (!scope.document->Undo()) Standard_Failure::Raise("Native frame undo failed."); keep(3, true, false);
        if (!scope.document->Redo()) Standard_Failure::Raise("Native frame redo failed."); keep(4, true, false);
        context->Erase(scope.presentation, Standard_False);
        scope.document->NewCommand(); assign(archive); scope.document->CommitCommand(); keep(5, true, false);
        context->Display(scope.presentation, AIS_Shaded, -1, Standard_False); keep(6, false, false);
        scope.document->NewCommand(); label.ForgetAttribute(AuthoredFrameAttributeID()); scope.document->CommitCommand(); keep(7, true, false);
        if (!scope.document->Undo()) Standard_Failure::Raise("Native frame removal undo failed."); keep(8, true, false);
        scope.document->NewCommand(); NSMutableData* invalid = [archive mutableCopy]; static_cast<std::uint8_t*>(invalid.mutableBytes)[invalid.length-1] ^= 1; assign(invalid); scope.document->CommitCommand();
        const auto invalidOwners = selectedOwners(); const auto invalidModes = selectedModes();
        const bool invalidRejected = ![[GLController debugFramebufferStatistics][@"captured"] boolValue];
        selectionUnchanged = selectionUnchanged && invalidOwners == selectedOwners() && invalidModes == selectedModes();
        if (!scope.document->Undo()) Standard_Failure::Raise("Invalid native frame cleanup failed."); keep(9, false, false);
        std::ostringstream output(std::ios::binary | std::ios::out);
        if (scope.app->SaveAs(scope.document, output) != PCDM_SS_OK) Standard_Failure::Raise("Native renderer fixture save failed.");
        const auto binary = output.str(); if (binary.empty() || binary.size() > 1024*1024) Standard_Failure::Raise("Native renderer fixture save exceeded bound.");
        Core3DBeginSafeBinaryRead(); std::istringstream input(binary, std::ios::binary | std::ios::in);
        if (scope.reopenApp->Open(input, scope.reopened) != PCDM_RS_OK || Core3DSafeBinaryReadWasRejected() || scope.reopened.IsNull()) Standard_Failure::Raise("Native renderer fixture reopen failed.");
        context->Remove(scope.presentation, Standard_False); scope.presentation.Nullify();
        wrapper->ChangeDocument() = scope.reopened;
        TDF_LabelSequence labels; XCAFDoc_DocumentTool::ShapeTool(scope.reopened->Main())->GetFreeShapes(labels);
        if (labels.Length() != 1) Standard_Failure::Raise("Native renderer fixture reopened owner missing."); label = labels.First(); display(); keep(10, true, true);
        context->Remove(scope.presentation, Standard_False); scope.presentation.Nullify();
        if (![[GLController debugFramebufferStatistics][@"captured"] boolValue] || viewer->DebugPreparedTangentArrayCount() != 0 || context->NbSelected() != 0) Standard_Failure::Raise("Native frame cache cleanup failed.");
        return @{@"states": states, @"captures": captures, @"readOnly": @(readOnly), @"selectionUnchanged": @(selectionUnchanged), @"invalidRejected": @(invalidRejected), @"cacheCleared": @YES};
    } catch (const Standard_Failure& failure) {
        return @{@"error": [NSString stringWithUTF8String:failure.GetMessageString()] ?: @"OCCT failure", @"step": @(fixtureStep)};
    } catch (...) { return @{@"error": @"Native authored-frame rendering fixture failed", @"step": @(fixtureStep)}; }
}

- (NSDictionary<NSString *, id> *)debugAuthoredFramePublications:(NSData *)archive replacement:(NSData *)replacement mode:(NSInteger)mode {
    if (![NSThread isMainThread] || !_isSetuped || !GLController.viewer || mode < 0 || mode > 8)
        return @{@"error": @"Unavailable authored-frame publication fixture"};
    try {
        using namespace core3d::persistence;
        using namespace core3d::scene;
        struct PrivateDocument {
            Handle(TDocStd_Application) app = new TDocStd_Application();
            Handle(TDocStd_Document) document;
            ~PrivateDocument() noexcept { try { if (!document.IsNull()) app->Close(document); } catch (...) {} }
        } writer, reader;
        Core3DDebugDefineFrameBinXCAFFormat(writer.app, std::make_shared<AuthoredFrameReadBudget>());
        Core3DDebugDefineFrameBinXCAFFormat(reader.app, std::make_shared<AuthoredFrameReadBudget>());
        writer.app->NewDocument(TCollection_ExtendedString("BinXCAF"), writer.document);
        XCAFDoc_DocumentTool::SetLengthUnit(writer.document, 0.001);
        const Handle(OcctDocument) document = new OcctDocument(); document->ChangeDocument() = writer.document;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(writer.document->Main());
        XCAFDoc_DocumentTool::ColorTool(writer.document->Main());
        XCAFDoc_DocumentTool::LayerTool(writer.document->Main());
        TopoDS_Shape shape = Core3DDebugAuthoredGeometryFixture(mode == 8 ? 22 : 0);
        if (mode == 2 || mode == 5) {
            BRep_Builder builder; TopoDS_Compound compound; builder.MakeCompound(compound); builder.Add(compound, shape); shape = compound;
        }
        const auto label = shapes->AddShape(shape, Standard_False);
        if (label.IsNull()) Standard_Failure::Raise("No publication fixture definition.");
        if (mode == 1 || mode == 2 || mode == 5) {
            gp_Trsf placed; placed.SetRotation(gp_Ax1(gp_Pnt(0,0,0), gp_Dir(0,0,1)), M_PI / 2);
            placed.SetTranslationPart(gp_Vec(2,3,4));
            shape.Location(TopLoc_Location(placed)); shapes->SetShape(label, shape);
        }
        writer.document->SetUndoLimit(20); writer.document->NewCommand();
        if (!document->SetGeometryRepresentationForLabel(label, OcctGeometryRepresentation::TriangleMesh))
            Standard_Failure::Raise("Publication fixture representation failed.");
        if (mode >= 3 && mode <= 5) {
            TDataStd_Real::Set(label.FindChild(8, Standard_True), mode == 3 ? 2.0 : -2.0);
            for (int axis = 1; axis <= 3; ++axis) TDataStd_Real::Set(label.FindChild(axis, Standard_True), axis * 10.0);
        }
        writer.document->CommitCommand(); writer.document->ClearUndos();
        if (!document->MigrateLegacyIdentifiers()) Standard_Failure::Raise("Publication fixture identity migration failed.");
        OcctSceneSnapshotBuilder builder;
        NSMutableArray* snapshots = [NSMutableArray array]; NSMutableArray* history = [NSMutableArray array];
        bool readOnly = true;
        auto capture = [&]() {
            const auto doc = document->Document();
            const int beforeTime = doc->GetData()->Time(), beforeUndo = doc->GetAvailableUndos(), beforeRedo = doc->GetAvailableRedos();
            const auto scene = builder.Build(document, GLController.viewer->AisContext(), GLController.viewer->ActiveView(), {800,600}, ElementKind::Object);
            readOnly = readOnly && beforeTime == doc->GetData()->Time() && beforeUndo == doc->GetAvailableUndos() && beforeRedo == doc->GetAvailableRedos();
            return scene;
        };
        auto keep = [&]() {
            const auto scene = capture();
            if (!scene) Standard_Failure::Raise("Native authored-frame publication rejected.");
            auto dto = Core3DCreateSceneSnapshotDTO(*scene);
            if (!dto) Standard_Failure::Raise("Authored-frame DTO rejected.");
            [snapshots addObject:dto];
            [history addObject:@[@(document->Document()->GetAvailableUndos()), @(document->Document()->GetAvailableRedos())]];
        };
        auto assign = [&](const TDF_Label& target, NSData* value) {
            if (value.length < 128 || value.length > core3d::scene::authored::kMaximumArchiveBytes)
                Standard_Failure::Raise("Invalid fixture archive bound.");
            const auto record = TDataStd_ByteArray::Set(target, AuthoredFrameAttributeID(), 0, int(value.length) - 1, Standard_False);
            const auto* bytes = static_cast<const std::uint8_t*>(value.bytes);
            for (NSUInteger i = 0; i < value.length; ++i) record->SetValue(int(i), bytes[i]);
        };
        keep();
        NSMutableDictionary* result = [NSMutableDictionary dictionary];
        if (mode == 6 || mode == 7) {
            writer.document->NewCommand();
            NSMutableData* invalid = [archive mutableCopy];
            if (mode == 6) static_cast<std::uint8_t*>(invalid.mutableBytes)[invalid.length - 1] ^= 1;
            assign(mode == 7 ? writer.document->GetData()->Root() : label, invalid);
            writer.document->CommitCommand();
            result[@"invalidPublicationRejected"] = @(!capture());
            if (!writer.document->Undo()) Standard_Failure::Raise("Invalid publication cleanup failed.");
            keep();
        } else {
            auto change = [&](NSData* value) { writer.document->NewCommand(); assign(label, value); if (!writer.document->CommitCommand()) Standard_Failure::Raise("Empty frame edit."); keep(); };
            change(archive); change(replacement);
            if (!writer.document->Undo()) Standard_Failure::Raise("Publication frame undo failed."); keep();
            if (!writer.document->Redo()) Standard_Failure::Raise("Publication frame redo failed."); keep();
            writer.document->NewCommand(); label.ForgetAttribute(AuthoredFrameAttributeID());
            if (!writer.document->CommitCommand()) Standard_Failure::Raise("Publication frame removal failed."); keep();
            if (!writer.document->Undo()) Standard_Failure::Raise("Publication removal undo failed."); keep();
            std::ostringstream output(std::ios::binary | std::ios::out);
            if (writer.app->SaveAs(writer.document, output) != PCDM_SS_OK) Standard_Failure::Raise("Publication fixture save failed.");
            const auto bytes = output.str();
            if (bytes.empty() || bytes.size() > 1024 * 1024) Standard_Failure::Raise("Publication fixture save exceeded bound.");
            Core3DBeginSafeBinaryRead(); std::istringstream input(bytes, std::ios::binary | std::ios::in);
            if (reader.app->Open(input, reader.document) != PCDM_RS_OK || Core3DSafeBinaryReadWasRejected() || reader.document.IsNull())
                Standard_Failure::Raise("Publication fixture reopen failed.");
            document->ChangeDocument() = reader.document; keep();
        }
        result[@"snapshots"] = snapshots; result[@"history"] = history; result[@"readOnly"] = @(readOnly);
        return result;
    } catch (const Standard_Failure& failure) {
        return @{@"error": [NSString stringWithUTF8String:failure.GetMessageString()] ?: @"OCCT failure"};
    } catch (...) { return @{@"error": @"Authored-frame publication fixture failed"}; }
}

- (Core3DSceneSnapshot *_Nullable)debugCaptureNormalMappedSceneSnapshot:(NSInteger)mode {
    if (![NSThread isMainThread] || !_isSetuped || !GLController.viewer) { return nil; }
    try {
        const auto size = GLController.drawableSize;
        if (!std::isfinite(size.width) || !std::isfinite(size.height)
            || size.width < 1 || size.height < 1 || size.width > UINT32_MAX || size.height > UINT32_MAX) { return nil; }
        const auto source = GLController.viewer->captureSceneSnapshot(
            static_cast<uint32_t>(std::llround(size.width)), static_cast<uint32_t>(std::llround(size.height)));
        if (!source || source->meshes.empty() || source->materials.empty()) { return nil; }
        auto result = *source;
        bool bound = false;
        for (auto& material : result.materials) {
            if (material.occlusionTextureIndex >= 0 && mode != 12) {
                material.normalTextureIndex = material.occlusionTextureIndex;
                material.occlusionTextureIndex = -1;
                bound = true;
            }
        }
        if (!bound && mode != 12) { return nil; }
        using namespace core3d::scene;
        if (mode == 15) {
            // A synthetic immutable export fixture with shared source indices
            // across opposite UV handedness. The live OCAF document is untouched.
            result.meshes.resize(1); result.instances.resize(1);
            auto& mesh = result.meshes.front();
            mesh.geometryRevision += 1;
            mesh.vertices = {{0,0,0, 0,0,1, 0,0}, {10,0,0, 0,0,1, 1,0},
                {0,10,0, 0,0,1, 0,1}, {-10,0,0, 0,0,1, 1,0}};
            mesh.indices = {0,1,2, 0,2,3};
            mesh.primitives = {{0,6,0,true}};
            mesh.topology = {1,0,0};
            mesh.localBounds = {{-10,0,0}, {10,10,0}, true};
            auto& instance = result.instances.front();
            instance.meshIndex = 0; instance.selected = false;
            instance.primitiveBindings = {{0,1,true}};
            result.selectionMode = ElementKind::Object;
            result.selection = {};
            result.pickTable = {ElementIdentifier(), {instance.entityIdentifier,
                ElementKind::Object, 0, mesh.geometryRevision}};
            result.revisions.model += 1; result.revisions.snapshot += 1;
        }
        for (auto& mesh : result.meshes) {
            const bool hasUV = std::all_of(mesh.primitives.begin(), mesh.primitives.end(),
                [](const auto& p) { return p.hasTextureCoordinates; });
            if (GenerateMikkCornerTangents(mesh.vertices, mesh.indices, hasUV,
                    mesh.cornerTangents) != TangentSpaceError::None) { return nil; }
            mesh.tangentBasis = TangentBasis::MikkTSpace;
        }
        auto& mesh = result.meshes.front();
        auto& tangent = mesh.cornerTangents.front();
        switch (mode) {
            case 0: case 12: case 15: break;
            case 1: mesh.cornerTangents.pop_back(); break;
            case 2: tangent.x = std::numeric_limits<float>::quiet_NaN(); break;
            case 3: tangent.w = 0; break;
            case 17: tangent.w = -tangent.w; break;
            case 4: tangent.x *= 2; tangent.y *= 2; tangent.z *= 2; break;
            case 5: {
                const auto& vertex = mesh.vertices[mesh.indices.front()];
                tangent = {vertex.normalX, vertex.normalY, vertex.normalZ, 1}; break;
            }
            case 6: mesh.primitives.front().hasTextureCoordinates = false; break;
            case 7: mesh.tangentBasis = TangentBasis::None; break;
            case 8: mesh.cornerTangents.clear(); mesh.tangentBasis = TangentBasis::None; break;
            case 9: mesh.indices.front() = static_cast<uint32_t>(mesh.vertices.size()); break;
            case 10: result.materials.front().normalTextureIndex = static_cast<int32_t>(result.textures.size()); break;
            case 16: {
                auto front = result.materials.front();
                front.identifier += "-front"; front.cullMode = CullMode::Front;
                const auto index = static_cast<uint32_t>(result.materials.size());
                result.materials.push_back(front);
                for (auto& item : result.instances) {
                    for (size_t i = 1; i < item.primitiveBindings.size(); i += 2) {
                        item.primitiveBindings[i].materialIndex = index;
                    }
                }
                break;
            }
            case 13: case 14:
                for (auto& material : result.materials) { material.cullMode = CullMode::Front; }
                if (mode == 14) {
                    for (auto& item : result.instances) {
                        for (int r = 0; r < 3; ++r) {
                            item.worldFromObject.values[r] *= -2;
                            item.worldFromObject.values[4+r] *= 3;
                            item.worldFromObject.values[8+r] *= 1.5;
                        }
                        item.reversesWinding = !item.reversesWinding;
                    }
                }
                break;
            case 11:
                for (auto& m : result.meshes) {
                    m.tangentBasis = TangentBasis::Authored;
                    for (auto& t : m.cornerTangents) { t.w = -t.w; }
                }
                break;
            default: return nil;
        }
        return Core3DCreateSceneSnapshotDTO(result);
    } catch (...) { return nil; }
}

- (BOOL)debugSetPresentationNormalTexture:(NSData *_Nullable)data {
    if (![NSThread isMainThread] || !GLController.viewer) return NO;
    const auto context = GLController.viewer->AisContext();
    if (context.IsNull()) return NO;
    Handle(Image_Texture) source;
    if (data && (!Core3DCreateAuthoredTexture(static_cast<const Standard_Byte*>(data.bytes),
        data.length, "image/png", source) || !Core3DValidateNumericTexture(source))) return NO;
    AIS_ListOfInteractive displayed;
    context->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
    if (displayed.Size() != 1) return NO;
    const auto shape = Handle(AIS_Shape)::DownCast(displayed.First());
    if (shape.IsNull() || !Handle(CafShapePrs)::DownCast(shape).IsNull()
        || shape->Attributes()->ShadingAspect().IsNull()) return NO;
    const auto aspect = shape->Attributes()->ShadingAspect()->Aspect();
    if (aspect.IsNull() || aspect->TextureSet().IsNull()) return NO;
    std::vector<Handle(Graphic3d_TextureMap)> bindings;
    for (Graphic3d_TextureSet::Iterator it(aspect->TextureSet()); it.More(); it.Next()) {
        if (it.Value().IsNull() || it.Value()->GetParams().IsNull()) return NO;
        if (it.Value()->GetParams()->TextureUnit() != Graphic3d_TextureUnit_Normal) bindings.push_back(it.Value());
    }
    if (data) bindings.push_back(new XCAFPrs_Texture(source, Graphic3d_TextureUnit_Normal));
    std::sort(bindings.begin(), bindings.end(), [](const auto& a, const auto& b) {
        return a->GetParams()->TextureUnit() < b->GetParams()->TextureUnit();
    });
    if (bindings.empty()) return NO;
    Handle(Graphic3d_TextureSet) textures = new Graphic3d_TextureSet(int(bindings.size()));
    for (int i = 0; i < bindings.size(); ++i) textures->SetValue(i, bindings[i]);
    aspect->SetTextureSet(textures); aspect->SetTextureMapOn();
    Core3DPrepareRendererTextures(aspect);
    shape->SetToUpdate();
    context->Redisplay(shape, Standard_False, Standard_True);
    GLController.viewer->Invalidate();
    return YES;
}

- (BOOL)debugSetNormalPresentationHidden:(BOOL)hidden {
    if (![NSThread isMainThread] || !GLController.viewer) return NO;
    const auto context = GLController.viewer->AisContext();
    if (context.IsNull()) return NO;
    AIS_ListOfInteractive resident;
    context->ObjectsInside(resident, AIS_KOI_Shape, -1);
    if (resident.Size() != 1) return NO;
    const auto shape = Handle(AIS_Shape)::DownCast(resident.First());
    if (shape.IsNull()) return NO;
    if (hidden) context->Erase(shape, Standard_False);
    else {
        context->Display(shape, Standard_False);
        // Display may activate OCCT's default whole-object selection. Restore
        // the editor's exact selection authority before snapshot publication.
        if ([self trySetSelectionType:_currentSelectionType] != Core3DSelectionTypeChangeResultSucceeded) return NO;
    }
    GLController.viewer->Invalidate();
    return YES;
}

- (NSUInteger)debugPreparedNativeTangentArrayCount {
    return GLController.viewer ? GLController.viewer->DebugPreparedTangentArrayCount() : 0;
}

- (NSArray<NSNumber *> *)debugNormalTextureRecipes {
    if (![NSThread isMainThread] || !GLController.viewer) return @[];
    const auto document = GLController.viewer->getDocument()->ChangeDocument();
    if (document.IsNull()) return @[];
    const auto tool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
    if (tool.IsNull()) return @[];
    TDF_LabelSequence roots; tool->GetFreeShapes(roots);
    if (roots.Length() < 0 || roots.Length() > 50000) return @[];
    NSMutableArray<NSNumber *> *values = [NSMutableArray arrayWithCapacity:roots.Length()];
    for (Standard_Integer index = 1; index <= roots.Length(); ++index)
        [values addObject:@(Core3DNormalTextureRecipeForLabel(roots.Value(index)))];
    return values;
}

- (NSInteger)debugVisualMaterialDefinitionCount {
    auto document = GLController.viewer->getDocument()->ChangeDocument();
    if (document.IsNull()
        || !XCAFDoc_DocumentTool::CheckVisMaterialTool(document->Main())) {
        return 0;
    }
    Handle(XCAFDoc_VisMaterialTool) tool =
        XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
    if (tool.IsNull()) {
        return 0;
    }
    TDF_LabelSequence labels;
    tool->GetMaterials(labels);
    return static_cast<NSInteger>(labels.Length());
}

- (void)debugSetMaximumTextureAuthoringObjects:(NSUInteger)limit {
    _debugMaximumTextureAuthoringObjects = std::min<NSUInteger>(
        limit, static_cast<NSUInteger>(kMaximumTextureAuthoringObjects));
}

- (void)debugSetMaximumSerializedTextureOccurrenceBytes:(NSUInteger)limit {
    if (GLController == nil || GLController.viewer == nullptr) {
        return;
    }
    const Handle(OcctDocument) document =
        GLController.viewer->getDocument();
    if (!document.IsNull()) {
        document->SetMaximumSerializedTextureOccurrenceBytesForTesting(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetMaximumDecodedTextureResourceBytes:(NSUInteger)limit {
    if (GLController == nil || GLController.viewer == nullptr) {
        return;
    }
    const Handle(OcctDocument) document =
        GLController.viewer->getDocument();
    if (!document.IsNull()) {
        document->SetMaximumDecodedTextureResourceBytesForTesting(
            static_cast<Standard_Size>(limit));
    }
}

- (NSDictionary<NSString *, NSDictionary *> *)debugTextureRolePreparation:(NSData *)data {
    Handle(Image_Texture) image;
    const std::string type = data.length >= 2
        && static_cast<const unsigned char*>(data.bytes)[0] == 0xff
        ? "image/jpeg" : "image/png";
    if (!Core3DCreateAuthoredTexture(static_cast<const Standard_Byte*>(data.bytes),
                                    data.length, type, image)) return nil;
    Handle(XCAFDoc_VisMaterial) material = new XCAFDoc_VisMaterial();
    XCAFDoc_VisMaterialPBR pbr;
    pbr.BaseColorTexture = image;
    pbr.EmissiveTexture = image;
    pbr.MetallicRoughnessTexture = image;
    pbr.OcclusionTexture = image;
    material->SetPbrMaterial(pbr);
    Handle(Graphic3d_AspectFillArea3d) aspect = new Graphic3d_AspectFillArea3d();
    material->FillAspect(aspect);
    Core3DPrepareRendererTextures(aspect);
    const Handle(Graphic3d_TextureSet) first = aspect->TextureSet();
    Core3DPrepareRendererTextures(aspect);
    NSMutableDictionary* result = [NSMutableDictionary dictionary];
    for (Graphic3d_TextureSet::Iterator iterator(aspect->TextureSet());
         iterator.More(); iterator.Next()) {
        Handle(XCAFPrs_Texture) texture = Handle(XCAFPrs_Texture)::DownCast(iterator.Value());
        if (texture.IsNull()) return nil;
        NSString* role = nil;
        switch (texture->GetParams()->TextureUnit()) {
            case Graphic3d_TextureUnit_BaseColor: role = @"baseColor"; break;
            case Graphic3d_TextureUnit_Emissive: role = @"emissive"; break;
            case Graphic3d_TextureUnit_MetallicRoughness: role = @"metallicRoughness"; break;
            case Graphic3d_TextureUnit_Occlusion: role = @"occlusion"; break;
            default: return nil;
        }
        NSMutableDictionary* record = [@{
            @"gpuID": [NSString stringWithUTF8String:texture->GetId().ToCString()],
            @"sourceID": [NSString stringWithUTF8String:texture->GetImageSource()->TextureId().ToCString()],
            @"color": @(texture->IsColorMap()),
            @"sourceMatches": @(Core3DTexturesMatch(image, texture->GetImageSource())),
            @"sourceValidated": @(Core3DValidateAuthoredTexture(texture->GetImageSource())),
            @"idempotent": @(first == aspect->TextureSet())
        } mutableCopy];
        const Handle(Image_PixMap) decoded = texture->GetImage(Handle(Image_SupportedFormats)());
        if (!decoded.IsNull() && decoded->Format() == Image_Format_RGBA) {
            // This inspection seam is only for small deterministic test vectors.
            // Decode itself still follows the production limits above.
            if (decoded->SizeX() * decoded->SizeY() > 1024) return nil;
            NSMutableData* pixels = [NSMutableData data];
            for (Standard_Size y = 0; y < decoded->SizeY(); ++y)
                [pixels appendBytes:decoded->Row(y) length:decoded->SizeX() * 4];
            record[@"rgba"] = pixels;
            record[@"width"] = @(decoded->SizeX());
            record[@"height"] = @(decoded->SizeY());
            record[@"topDown"] = @(decoded->IsTopDown());
        }
        result[role] = record;
    }
    return result;
}

- (void)debugSetMaximumVisualMaterialDefinitions:(NSUInteger)limit {
    if (GLController == nil || GLController.viewer == nullptr) {
        return;
    }
    const Handle(OcctDocument) document =
        GLController.viewer->getDocument();
    if (!document.IsNull()) {
        document->SetMaximumVisualMaterialDefinitionsForTesting(
            static_cast<Standard_Size>(limit));
    }
}

- (BOOL)debugClearSelectedCachedTriangulationsForTextureTest {
    if (![NSThread isMainThread] || GLController == nil
        || GLController.viewer == nullptr) {
        return NO;
    }
    const Handle(AIS_InteractiveContext) context =
        GLController.viewer->AisContext();
    if (context.IsNull()) {
        return NO;
    }
    Standard_Size cleaned = 0;
    for (context->InitSelected(); context->MoreSelected();
         context->NextSelected()) {
        const Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(
            context->SelectedInteractive());
        if (shape.IsNull() || shape->Shape().IsNull()) {
            return NO;
        }
        BRepTools::Clean(shape->Shape(), Standard_True);
        ++cleaned;
    }
    return cleaned > 0;
}

- (NSData *_Nullable)debugExternalTextureBinXCAFFixtureData {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL* baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.external-texture-fixture", NSUUID.UUID.UUIDString]];
    NSString* xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSData* result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        if (document.IsNull()) {
            throw Standard_Failure("Unable to create XCAF fixture document");
        }
        Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        Handle(XCAFDoc_VisMaterialTool) tool =
            XCAFDoc_DocumentTool::VisMaterialTool(
                document->Main());
        if (shapeTool.IsNull() || tool.IsNull()) {
            throw Standard_Failure("Unable to create XCAF fixture tools");
        }
        const TopoDS_Shape cube = BRepPrimAPI_MakeBox(
            gp_Pnt(-25.0, -25.0, 0.0), 50.0, 50.0, 50.0).Shape();
        const TDF_Label shapeLabel = shapeTool->AddShape(
            cube, Standard_False, Standard_True);
        if (shapeLabel.IsNull()) {
            throw Standard_Failure("Unable to create XCAF fixture shape");
        }
        XCAFDoc_VisMaterialPBR pbr;
        pbr.BaseColor = Quantity_ColorRGBA(
            Quantity_Color(0.3, 0.6, 0.9, Quantity_TOC_sRGB),
            1.0f);
        pbr.Metallic = 0.2f;
        pbr.Roughness = 0.7f;
        pbr.BaseColorTexture = new Image_Texture(
            TCollection_AsciiString(
                "/tmp/shapeyard-unowned-texture.png"));
        Handle(XCAFDoc_VisMaterial) material =
            new XCAFDoc_VisMaterial();
        material->SetPbrMaterial(pbr);
        material->SetCommonMaterial(
            material->ConvertToCommonMaterial());
        const TDF_Label materialLabel = tool->AddMaterial(
            material,
            TCollection_AsciiString("External texture fixture"));
        if (materialLabel.IsNull()) {
            throw Standard_Failure("Unable to create XCAF fixture material");
        }
        tool->SetShapeMaterial(shapeLabel, materialLabel);
        if (application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure("Unable to save XCAF fixture");
        }
        result = [NSData dataWithContentsOfFile:xbfPath];
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return result;
}

- (NSData *_Nullable)debugNormalRecipeBinXCAFFixtureData:(NSInteger)mode {
    if (mode < 0 || mode > 6) return nil;
    return Core3DCreateDebugBinXCAFFixture(@"normal-recipe-fixture",
        [mode](const Handle(TDocStd_Document)& document) {
            const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
            const auto materials = XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
            const TDF_Label label = shapes->AddShape(Core3DMakeDebugTriangleMeshFace(),
                Standard_False, Standard_True);
            Core3DSetDebugGeometryRepresentation(label,
                static_cast<Standard_Integer>(OcctGeometryRepresentation::TriangleMesh));
            if (!Core3DValidateNormalTextureGeometry(label))
                throw Standard_Failure("Normal recipe control geometry is invalid");
            NSData *png = [[NSData alloc] initWithBase64EncodedString:
                @"iVBORw0KGgoAAAANSUhEUgAAAAQAAAAECAIAAAAmkwkpAAAABGdBTUEAALGPC/xhBQAAABFJREFUeJxjcNvyEI4YiOMAAPqCHbEKj/fTAAAAAElFTkSuQmCC" options:0];
            XCAFDoc_VisMaterialPBR pbr;
            pbr.IsDefined = Standard_True;
            if (label.IsNull() || !Core3DCreateAuthoredTexture(
                    static_cast<const Standard_Byte *>(png.bytes), png.length,
                    "image/png", pbr.NormalTexture)
                || !Core3DValidateNumericTexture(pbr.NormalTexture))
                throw Standard_Failure("Unable to construct normal recipe control");
            if (mode == 6) pbr.NormalTexture.Nullify();
            Handle(XCAFDoc_VisMaterial) material = new XCAFDoc_VisMaterial();
            material->SetPbrMaterial(pbr);
            material->SetCommonMaterial(material->ConvertToCommonMaterial());
            const auto materialLabel = materials->AddMaterial(material, "Normal recipe fixture");
            materials->SetShapeMaterial(label, materialLabel);
            TDataStd_Integer::Set(label,
                Standard_GUID("248A5203-4A22-4F2B-85C4-BE0BA89A5E4D"), 1);
            const Standard_GUID recipe("98EAD304-EB49-4F0E-ABFC-AEF94C250161");
            if (mode != 1) {
                if (mode == 3) TDataStd_Real::Set(label, recipe, 1.0);
                else TDataStd_Integer::Set(label, recipe, mode == 2 ? 2 : 1);
            }
            if (mode == 4) TDataStd_Integer::Set(document->GetData()->Root(), recipe, 1);
            if (mode == 5) TDataStd_Integer::Set(label.FindChild(97, Standard_True), recipe, 1);
        });
}

- (NSData *_Nullable)debugOrphanVisualMaterialBinXCAFFixtureData {
    return Core3DCreateDebugBinXCAFFixture(
        @"orphan-visual-material-fixture",
        [](const Handle(TDocStd_Document)& document) {
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            const Handle(XCAFDoc_VisMaterialTool) materialTool =
                XCAFDoc_DocumentTool::VisMaterialTool(
                    document->Main());
            if (shapeTool.IsNull() || materialTool.IsNull()) {
                throw Standard_Failure(
                    "Unable to create orphan material fixture tools");
            }
            const TDF_Label shapeLabel = shapeTool->AddShape(
                BRepPrimAPI_MakeBox(
                    gp_Pnt(-10.0, -10.0, 0.0),
                    20.0, 20.0, 20.0).Shape(),
                Standard_False,
                Standard_True);
            if (shapeLabel.IsNull()) {
                throw Standard_Failure(
                    "Unable to create orphan material fixture shape");
            }
            const TDF_Label orphanLabel =
                shapeLabel.FindChild(97, Standard_True);
            if (orphanLabel.IsNull()) {
                throw Standard_Failure(
                    "Unable to create orphan material fixture label");
            }
            Core3DAddDebugOrphanVisualMaterial(
                orphanLabel, "orphan-child-texture");
        });
}

- (NSData *_Nullable)debugMainOrphanVisualMaterialBinXCAFFixtureData {
    return Core3DCreateDebugBinXCAFFixture(
        @"main-orphan-visual-material-fixture",
        [](const Handle(TDocStd_Document)& document) {
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            (void)XCAFDoc_DocumentTool::VisMaterialTool(
                document->Main());
            if (shapeTool.IsNull()
                || shapeTool->AddShape(
                    BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape(),
                    Standard_False, Standard_True).IsNull()) {
                throw Standard_Failure(
                    "Unable to create Main orphan fixture shape");
            }
            Core3DAddDebugOrphanVisualMaterial(
                document->Main(), "orphan-main-texture");
        });
}

- (NSData *_Nullable)debugDataRootOrphanVisualMaterialBinXCAFFixtureData {
    return Core3DCreateDebugBinXCAFFixture(
        @"data-root-orphan-visual-material-fixture",
        [](const Handle(TDocStd_Document)& document) {
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            (void)XCAFDoc_DocumentTool::VisMaterialTool(
                document->Main());
            const Handle(TDF_Data)& data = document->GetData();
            if (shapeTool.IsNull() || data.IsNull()
                || shapeTool->AddShape(
                    BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape(),
                    Standard_False, Standard_True).IsNull()) {
                throw Standard_Failure(
                    "Unable to create data-root orphan fixture shape");
            }
            Core3DAddDebugOrphanVisualMaterial(
                data->Root(), "orphan-data-root-texture");
        });
}

- (NSData *_Nullable)debugDataRootSiblingOrphanVisualMaterialBinXCAFFixtureData {
    return Core3DCreateDebugBinXCAFFixture(
        @"data-root-sibling-orphan-visual-material-fixture",
        [](const Handle(TDocStd_Document)& document) {
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            (void)XCAFDoc_DocumentTool::VisMaterialTool(
                document->Main());
            const Handle(TDF_Data)& data = document->GetData();
            if (shapeTool.IsNull() || data.IsNull()
                || shapeTool->AddShape(
                    BRepPrimAPI_MakeBox(20.0, 20.0, 20.0).Shape(),
                    Standard_False, Standard_True).IsNull()) {
                throw Standard_Failure(
                    "Unable to create root-sibling orphan fixture shape");
            }
            const TDF_Label sibling =
                data->Root().FindChild(97, Standard_True);
            Core3DAddDebugOrphanVisualMaterial(
                sibling, "orphan-data-root-sibling-texture");
        });
}

- (NSData *_Nullable)debugOversizedTextureLengthBinXCAFFixtureData {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL* baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.oversized-texture-fixture", NSUUID.UUID.UUIDString]];
    NSString* xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSMutableData* result = nil;
    static NSString* const marker =
        @"texturebuf://shapeyard-oversized-texture-length";
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        Handle(XCAFDoc_VisMaterialTool) tool =
            XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
        if (document.IsNull() || shapeTool.IsNull() || tool.IsNull()) {
            throw Standard_Failure("Unable to create oversized fixture");
        }
        const TDF_Label shapeLabel = shapeTool->AddShape(
            BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(),
            Standard_False,
            Standard_True);
        NSData* png = [[NSData alloc] initWithBase64EncodedString:
            @"iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAYAAACqaXHeAAAApklEQVR42u3aQQ3AQAzEwFyZF3kKY06qTWAtK8+cndmBnJ1X7j9y/AYKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNAXQApoCaAFNAbSApgBaQFMALaApgBbQnJml/wF2vQsoQAG0gKYAWkBTAC2gKYAW0BRAC2gKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNL8P8AESbQf6Ta5RUwAAAABJRU5ErkJggg=="
            options:0];
        Handle(NCollection_Buffer) buffer = new NCollection_Buffer(
            NCollection_BaseAllocator::CommonBaseAllocator(), png.length);
        if (shapeLabel.IsNull() || png.length == 0 || buffer.IsNull()) {
            throw Standard_Failure("Unable to create texture buffer");
        }
        std::memcpy(buffer->ChangeData(), png.bytes, png.length);
        XCAFDoc_VisMaterialPBR pbr;
        pbr.BaseColorTexture = new Image_Texture(
            buffer,
            TCollection_AsciiString(
                "shapeyard-oversized-texture-length"));
        Handle(XCAFDoc_VisMaterial) material =
            new XCAFDoc_VisMaterial();
        material->SetPbrMaterial(pbr);
        const TDF_Label materialLabel = tool->AddMaterial(
            material,
            TCollection_AsciiString("Oversized texture fixture"));
        if (materialLabel.IsNull()) {
            throw Standard_Failure("Unable to add oversized fixture material");
        }
        tool->SetShapeMaterial(shapeLabel, materialLabel);
        if (application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure("Unable to save oversized fixture");
        }
        result = [[NSData dataWithContentsOfFile:xbfPath] mutableCopy];
        if (result == nil) {
            throw Standard_Failure("Unable to read oversized fixture");
        }
        NSData* markerData = [marker dataUsingEncoding:NSUTF8StringEncoding];
        const NSRange markerRange = [result rangeOfData:markerData
                                                options:0
                                                  range:NSMakeRange(
                                                      0, result.length)];
        if (markerRange.location == NSNotFound) {
            throw Standard_Failure("Unable to locate texture length marker");
        }
        const NSUInteger afterTerminator = NSMaxRange(markerRange) + 1;
        const NSUInteger booleanOffset = (afterTerminator + 3) & ~NSUInteger(3);
        const NSUInteger lengthOffset = booleanOffset + sizeof(std::int32_t);
        if (lengthOffset > result.length - sizeof(std::int32_t)) {
            throw Standard_Failure("Invalid texture length offset");
        }
        const std::uint32_t maliciousLength = CFSwapInt32HostToLittle(
            static_cast<std::uint32_t>(INT32_MAX));
        [result replaceBytesInRange:NSMakeRange(
                                        lengthOffset,
                                        sizeof(maliciousLength))
                           withBytes:&maliciousLength];
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return result;
}

- (NSData *_Nullable)debugCyclicAssemblyBinXCAFFixtureData {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL* baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.cyclic-assembly-fixture", NSUUID.UUID.UUIDString]];
    NSString* xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSData* result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (document.IsNull() || shapeTool.IsNull()) {
            throw Standard_Failure("Unable to create cyclic fixture");
        }
        const TDF_Label root = shapeTool->NewShape();
        const TDF_Label first = shapeTool->NewShape();
        const TDF_Label second = shapeTool->NewShape();
        if (root.IsNull() || first.IsNull() || second.IsNull()
            || shapeTool->AddComponent(
                root, first, TopLoc_Location()).IsNull()
            || shapeTool->AddComponent(
                first, second, TopLoc_Location()).IsNull()
            || shapeTool->AddComponent(
                second, first, TopLoc_Location()).IsNull()) {
            throw Standard_Failure("Unable to create assembly cycle");
        }
        if (application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure("Unable to save cyclic fixture");
        }
        result = [NSData dataWithContentsOfFile:xbfPath];
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return result;
}

- (NSData *_Nullable)debugSharedDefinitionAssemblyBinXCAFFixtureData {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL* baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.shared-definition-assembly-fixture",
            NSUUID.UUID.UUIDString]];
    NSString* xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSData* result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        Handle(XCAFDoc_ShapeTool) shapeTool = document.IsNull()
            ? Handle(XCAFDoc_ShapeTool)()
            : XCAFDoc_DocumentTool::ShapeTool(document->Main());
        Handle(XCAFDoc_ColorTool) colorTool = document.IsNull()
            ? Handle(XCAFDoc_ColorTool)()
            : XCAFDoc_DocumentTool::ColorTool(document->Main());
        Handle(XCAFDoc_VisMaterialTool) materialTool = document.IsNull()
            ? Handle(XCAFDoc_VisMaterialTool)()
            : XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
        if (document.IsNull() || shapeTool.IsNull()
            || colorTool.IsNull() || materialTool.IsNull()) {
            throw Standard_Failure(
                "Unable to create shared-definition assembly fixture");
        }
        XCAFDoc_DocumentTool::SetLengthUnit(document, 0.001);
        const TDF_Label definition = shapeTool->AddShape(
            BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(),
            Standard_False,
            Standard_True);
        const TDF_Label nestedAssembly = shapeTool->NewShape();
        const TDF_Label rootAssembly = shapeTool->NewShape();
        gp_Trsf firstTransform;
        firstTransform.SetTranslation(gp_Vec(10.0, 0.0, 0.0));
        gp_Trsf secondTransform;
        secondTransform.SetTranslation(gp_Vec(50.0, 0.0, 0.0));
        gp_Trsf parentTransform;
        parentTransform.SetTranslation(gp_Vec(100.0, 0.0, 0.0));
        const TDF_Label firstOccurrence = shapeTool->AddComponent(
            nestedAssembly,
            definition,
            TopLoc_Location(firstTransform));
        const TDF_Label secondOccurrence = shapeTool->AddComponent(
            nestedAssembly,
            definition,
            TopLoc_Location(secondTransform));
        const TDF_Label parentOccurrence = shapeTool->AddComponent(
            rootAssembly,
            nestedAssembly,
            TopLoc_Location(parentTransform));
        if (definition.IsNull() || nestedAssembly.IsNull()
            || rootAssembly.IsNull() || firstOccurrence.IsNull()
            || secondOccurrence.IsNull() || parentOccurrence.IsNull()) {
            throw Standard_Failure(
                "Unable to populate shared-definition assembly fixture");
        }

        // Imported definition material must supply non-color PBR properties,
        // while each occurrence surface color remains the final resolved tint.
        // Re-applying the definition material after XCAF style resolution
        // would incorrectly turn both presentations green.
        XCAFDoc_VisMaterialPBR definitionPBR;
        definitionPBR.BaseColor = Quantity_ColorRGBA(
            Quantity_Color(Quantity_NOC_GREEN), 1.0f);
        definitionPBR.Metallic = 0.35f;
        definitionPBR.Roughness = 0.65f;
        Handle(XCAFDoc_VisMaterial) definitionMaterial =
            new XCAFDoc_VisMaterial();
        definitionMaterial->SetPbrMaterial(definitionPBR);
        definitionMaterial->SetCommonMaterial(
            definitionMaterial->ConvertToCommonMaterial());
        const TDF_Label definitionMaterialLabel = materialTool->AddMaterial(
            definitionMaterial,
            TCollection_AsciiString("Imported green definition material"));
        if (definitionMaterialLabel.IsNull()) {
            throw Standard_Failure(
                "Unable to create shared-definition material fixture");
        }
        materialTool->SetShapeMaterial(
            definition, definitionMaterialLabel);
        colorTool->SetColor(
            firstOccurrence,
            Quantity_Color(Quantity_NOC_RED),
            XCAFDoc_ColorSurf);
        colorTool->SetColor(
            secondOccurrence,
            Quantity_Color(Quantity_NOC_BLUE1),
            XCAFDoc_ColorSurf);
        if (application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure(
                "Unable to save shared-definition assembly fixture");
        }
        result = [NSData dataWithContentsOfFile:xbfPath];
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return result;
}

- (NSData *_Nullable)debugSharedSubtreeMultipleRootsBinXCAFFixtureData {
    return Core3DCreateDebugBinXCAFFixture(
        @"shared-subtree-multiple-roots-fixture",
        [](const Handle(TDocStd_Document)& document) {
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            if (shapeTool.IsNull()) {
                throw Standard_Failure("Unable to create shape tool");
            }
            const TDF_Label definition = shapeTool->AddShape(
                BRepPrimAPI_MakeBox(4.0, 4.0, 4.0).Shape(),
                Standard_False,
                Standard_True);
            const TDF_Label sharedAssembly = shapeTool->NewShape();
            const TDF_Label firstRoot = shapeTool->NewShape();
            const TDF_Label secondRoot = shapeTool->NewShape();
            gp_Trsf firstLeafTransform;
            firstLeafTransform.SetTranslation(gp_Vec(10.0, 0.0, 0.0));
            gp_Trsf secondLeafTransform;
            secondLeafTransform.SetTranslation(gp_Vec(20.0, 0.0, 0.0));
            gp_Trsf firstRootTransform;
            firstRootTransform.SetTranslation(gp_Vec(100.0, 0.0, 0.0));
            gp_Trsf secondRootTransform;
            secondRootTransform.SetTranslation(gp_Vec(200.0, 0.0, 0.0));
            if (definition.IsNull() || sharedAssembly.IsNull()
                || firstRoot.IsNull() || secondRoot.IsNull()
                || shapeTool->AddComponent(
                    sharedAssembly,
                    definition,
                    TopLoc_Location(firstLeafTransform)).IsNull()
                || shapeTool->AddComponent(
                    sharedAssembly,
                    definition,
                    TopLoc_Location(secondLeafTransform)).IsNull()
                || shapeTool->AddComponent(
                    firstRoot,
                    sharedAssembly,
                    TopLoc_Location(firstRootTransform)).IsNull()
                || shapeTool->AddComponent(
                    secondRoot,
                    sharedAssembly,
                    TopLoc_Location(secondRootTransform)).IsNull()) {
                throw Standard_Failure(
                    "Unable to populate shared multi-root fixture");
            }
        });
}

- (NSData *_Nullable)debugSavedGroupBinXCAFFixture:(NSInteger)mode {
    if (mode < 0 || mode > 12) { return nil; }
    return Core3DCreateDebugBinXCAFFixture(@"saved-group-schema", [mode](const Handle(TDocStd_Document)& document) {
        // Independent on-disk schema fixture, deliberately repeats persistent
        // GUIDs instead of asking production helpers to author a valid record.
        const Standard_GUID containerID("EC7B5F15-218F-47E4-BF6A-61BF42861401");
        const Standard_GUID recordID("EC7B5F15-218F-47E4-BF6A-61BF42861402");
        const Standard_GUID nameID("EC7B5F15-218F-47E4-BF6A-61BF42861403");
        const Standard_GUID memberID("EC7B5F15-218F-47E4-BF6A-61BF42861404");
        const Standard_GUID entityID("0074F7C2-9EAA-4F89-B2DE-8716E155FF62");
        const Standard_GUID definitionID("3611F2B2-C694-4E12-AED8-A2A97A3D283B");
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        const auto root = document->GetData()->Root();
        const auto container = mode == 2 ? document->Main().FindChild(47, Standard_True) : root.FindChild(42, Standard_True);
        if (mode == 1) { TDataStd_Real::Set(container, containerID, 1.0); }
        else { TDataStd_Integer::Set(container, containerID, mode == 3 ? 2 : 1); }
        const std::string id = "9824816B-0E48-45A5-B874-1D703D68D23D";
        const auto record = container.FindChild(1, Standard_True);
        TDataStd_AsciiString::Set(record, recordID, TCollection_AsciiString(mode == 4 ? "bad-identifier" : id.c_str()));
        if (mode != 5) {
            TDataStd_Name::Set(record, nameID, TCollection_ExtendedString(mode == 6 ? " bad name " : "Imported group"));
        }
        if (mode == 7) {
            const auto duplicate = container.FindChild(2, Standard_True);
            TDataStd_AsciiString::Set(duplicate, recordID, TCollection_AsciiString(id.c_str()));
            TDataStd_Name::Set(duplicate, nameID, TCollection_ExtendedString("Duplicate ID"));
        }
        if (mode == 8) { TDataStd_AsciiString::Set(root, memberID, TCollection_AsciiString(id.c_str())); }
        if (mode == 9) {
            for (int i = 2; i <= 129; ++i) {
                const auto extra = container.FindChild(i, Standard_True);
                TDataStd_AsciiString::Set(extra, recordID, TCollection_AsciiString(NSUUID.UUID.UUIDString.UTF8String));
                TDataStd_Name::Set(extra, nameID, TCollection_ExtendedString("Extra group"));
            }
        }
        const int count = mode == 10 ? 33 : 2;
        for (int i = 0; i < count; ++i) {
            const auto label = Core3DAddDebugGeometryDefinition(shapes,
                BRepPrimAPI_MakeBox(gp_Pnt(i * 5.0, 0, 0), 2, 2, 2).Shape());
            Core3DSetDebugGeometryRepresentation(label, static_cast<Standard_Integer>(OcctGeometryRepresentation::BRep));
            TDataStd_AsciiString::Set(label, entityID, TCollection_AsciiString(NSUUID.UUID.UUIDString.UTF8String));
            TDataStd_AsciiString::Set(label, definitionID, TCollection_AsciiString(NSUUID.UUID.UUIDString.UTF8String));
            if (mode == 11) { TDataStd_Integer::Set(label, memberID, 1); }
            else { TDataStd_AsciiString::Set(label, memberID, TCollection_AsciiString(mode == 12 ? "9824816B-0E48-45A5-B874-1D703D68D23E" : id.c_str())); }
        }
    });
}

- (NSData *_Nullable)debugHiddenAssemblyVisibilityBinXCAFFixtureData {
    return Core3DCreateDebugBinXCAFFixture(
        @"hidden-assembly-visibility-fixture",
        [](const Handle(TDocStd_Document)& document) {
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            const Handle(XCAFDoc_ColorTool) colorTool =
                XCAFDoc_DocumentTool::ColorTool(document->Main());
            if (shapeTool.IsNull() || colorTool.IsNull()) {
                throw Standard_Failure(
                    "Unable to create visibility fixture tools");
            }

            const TDF_Label visibleDefinition = shapeTool->AddShape(
                BRepPrimAPI_MakeBox(4.0, 4.0, 4.0).Shape(),
                Standard_False,
                Standard_True);
            const TDF_Label hiddenDefinition = shapeTool->AddShape(
                BRepPrimAPI_MakeBox(5.0, 5.0, 5.0).Shape(),
                Standard_False,
                Standard_True);
            const TDF_Label hiddenParent = shapeTool->NewShape();
            const TDF_Label rootAssembly = shapeTool->NewShape();
            gp_Trsf at10;
            at10.SetTranslation(gp_Vec(10.0, 0.0, 0.0));
            gp_Trsf at20;
            at20.SetTranslation(gp_Vec(20.0, 0.0, 0.0));
            gp_Trsf at30;
            at30.SetTranslation(gp_Vec(30.0, 0.0, 0.0));
            gp_Trsf at40;
            at40.SetTranslation(gp_Vec(40.0, 0.0, 0.0));
            gp_Trsf at50;
            at50.SetTranslation(gp_Vec(50.0, 0.0, 0.0));
            const TDF_Label visibleOccurrence = shapeTool->AddComponent(
                rootAssembly,
                visibleDefinition,
                TopLoc_Location(at10));
            const TDF_Label hiddenOccurrence = shapeTool->AddComponent(
                rootAssembly,
                visibleDefinition,
                TopLoc_Location(at20));
            const TDF_Label hiddenDefinitionOccurrence =
                shapeTool->AddComponent(
                    rootAssembly,
                    hiddenDefinition,
                    TopLoc_Location(at30));
            const TDF_Label hiddenParentLeaf = shapeTool->AddComponent(
                hiddenParent,
                visibleDefinition,
                TopLoc_Location());
            const TDF_Label hiddenParentOccurrence =
                shapeTool->AddComponent(
                    rootAssembly,
                    hiddenParent,
                    TopLoc_Location(at40));
            const TDF_Label hiddenLayerOccurrence =
                shapeTool->AddComponent(
                    rootAssembly,
                    visibleDefinition,
                    TopLoc_Location(at50));
            if (visibleDefinition.IsNull() || hiddenDefinition.IsNull()
                || hiddenParent.IsNull() || rootAssembly.IsNull()
                || visibleOccurrence.IsNull()
                || hiddenOccurrence.IsNull()
                || hiddenDefinitionOccurrence.IsNull()
                || hiddenParentLeaf.IsNull()
                || hiddenParentOccurrence.IsNull()
                || hiddenLayerOccurrence.IsNull()) {
                throw Standard_Failure(
                    "Unable to populate visibility fixture");
            }

            colorTool->SetColor(
                visibleOccurrence,
                Quantity_Color(Quantity_NOC_RED),
                XCAFDoc_ColorSurf);
            colorTool->SetVisibility(
                hiddenOccurrence, Standard_False);
            colorTool->SetVisibility(
                hiddenDefinition, Standard_False);
            colorTool->SetVisibility(
                hiddenParent, Standard_False);
        });
}

- (NSData *_Nullable)debugLocatedFreeShapeBinXCAFFixtureData {
    return Core3DCreateDebugBinXCAFFixture(
        @"located-free-shape-fixture",
        [](const Handle(TDocStd_Document)& document) {
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            const Handle(XCAFDoc_VisMaterialTool) materialTool =
                XCAFDoc_DocumentTool::VisMaterialTool(
                    document->Main());
            if (shapeTool.IsNull() || materialTool.IsNull()) {
                throw Standard_Failure(
                    "Unable to create located fixture tools");
            }
            gp_Trsf locationTransform;
            locationTransform.SetTranslation(gp_Vec(42.0, 0.0, 0.0));
            TopoDS_Shape locatedShape =
                BRepPrimAPI_MakeBox(4.0, 4.0, 4.0).Shape();
            locatedShape.Location(TopLoc_Location(locationTransform));
            const TDF_Label definition = shapeTool->NewShape();
            if (definition.IsNull()) {
                throw Standard_Failure(
                    "Unable to create located fixture definition");
            }
            shapeTool->SetShape(definition, locatedShape);

            XCAFDoc_VisMaterialPBR importedPBR;
            importedPBR.BaseColor = Quantity_ColorRGBA(
                Quantity_Color(Quantity_NOC_GREEN), 1.0f);
            importedPBR.Metallic = 0.35f;
            importedPBR.Roughness = 0.65f;
            Handle(XCAFDoc_VisMaterial) importedMaterial =
                new XCAFDoc_VisMaterial();
            importedMaterial->SetPbrMaterial(importedPBR);
            importedMaterial->SetCommonMaterial(
                importedMaterial->ConvertToCommonMaterial());
            const TDF_Label materialLabel = materialTool->AddMaterial(
                importedMaterial,
                TCollection_AsciiString("Located imported PBR"));
            if (materialLabel.IsNull()) {
                throw Standard_Failure(
                    "Unable to create located fixture material");
            }
            materialTool->SetShapeMaterial(definition, materialLabel);
        });
}

- (NSData *_Nullable)debugAuthoredLegacyAssemblyBinXCAFFixtureData {
    return Core3DCreateDebugBinXCAFFixture(
        @"authored-legacy-assembly-fixture",
        [](const Handle(TDocStd_Document)& document) {
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            const Handle(XCAFDoc_ColorTool) colorTool =
                XCAFDoc_DocumentTool::ColorTool(document->Main());
            const Handle(XCAFDoc_VisMaterialTool) materialTool =
                XCAFDoc_DocumentTool::VisMaterialTool(
                    document->Main());
            if (shapeTool.IsNull() || colorTool.IsNull()
                || materialTool.IsNull()) {
                throw Standard_Failure(
                    "Unable to create authored assembly tools");
            }
            const TDF_Label definition = shapeTool->AddShape(
                BRepPrimAPI_MakeBox(4.0, 4.0, 4.0).Shape(),
                Standard_False,
                Standard_True);
            const TDF_Label rootAssembly = shapeTool->NewShape();
            gp_Trsf at10;
            at10.SetTranslation(gp_Vec(10.0, 0.0, 0.0));
            gp_Trsf at50;
            at50.SetTranslation(gp_Vec(50.0, 0.0, 0.0));
            const TDF_Label firstOccurrence = shapeTool->AddComponent(
                rootAssembly, definition, TopLoc_Location(at10));
            const TDF_Label secondOccurrence = shapeTool->AddComponent(
                rootAssembly, definition, TopLoc_Location(at50));
            if (definition.IsNull() || rootAssembly.IsNull()
                || firstOccurrence.IsNull()
                || secondOccurrence.IsNull()) {
                throw Standard_Failure(
                    "Unable to populate authored assembly fixture");
            }

            XCAFDoc_VisMaterialPBR importedPBR;
            importedPBR.BaseColor = Quantity_ColorRGBA(
                Quantity_Color(Quantity_NOC_GREEN), 1.0f);
            importedPBR.Metallic = 0.35f;
            importedPBR.Roughness = 0.65f;
            Handle(XCAFDoc_VisMaterial) importedMaterial =
                new XCAFDoc_VisMaterial();
            importedMaterial->SetPbrMaterial(importedPBR);
            importedMaterial->SetCommonMaterial(
                importedMaterial->ConvertToCommonMaterial());
            const TDF_Label importedMaterialLabel =
                materialTool->AddMaterial(
                    importedMaterial,
                    TCollection_AsciiString(
                        "Lower-priority imported PBR"));
            if (importedMaterialLabel.IsNull()) {
                throw Standard_Failure(
                    "Unable to create authored assembly material");
            }
            materialTool->SetShapeMaterial(
                definition, importedMaterialLabel);

            TopExp_Explorer aFace(definition.IsNull()
                ? TopoDS_Shape()
                : XCAFDoc_ShapeTool::GetShape(definition), TopAbs_FACE);
            if (!aFace.More()) {
                throw Standard_Failure(
                    "Unable to find authored assembly face");
            }
            const TDF_Label faceLabel = shapeTool->AddSubShape(
                definition, aFace.Current());
            XCAFDoc_VisMaterialPBR facePBR;
            facePBR.BaseColor = Quantity_ColorRGBA(
                Quantity_Color(Quantity_NOC_BLUE1), 1.0f);
            facePBR.Metallic = 0.05f;
            facePBR.Roughness = 0.95f;
            Handle(XCAFDoc_VisMaterial) faceMaterial =
                new XCAFDoc_VisMaterial();
            faceMaterial->SetPbrMaterial(facePBR);
            faceMaterial->SetCommonMaterial(
                faceMaterial->ConvertToCommonMaterial());
            const TDF_Label faceMaterialLabel =
                materialTool->AddMaterial(
                    faceMaterial,
                    TCollection_AsciiString(
                        "Lower-priority imported face PBR"));
            if (faceLabel.IsNull() || faceMaterialLabel.IsNull()) {
                throw Standard_Failure(
                    "Unable to create authored assembly face style");
            }
            materialTool->SetShapeMaterial(
                faceLabel, faceMaterialLabel);
            colorTool->SetColor(
                faceLabel,
                Quantity_Color(Quantity_NOC_BLUE1),
                XCAFDoc_ColorSurf);
            colorTool->SetColor(
                firstOccurrence,
                Quantity_Color(Quantity_NOC_BLUE1),
                XCAFDoc_ColorSurf);
            colorTool->SetColor(
                secondOccurrence,
                Quantity_Color(Quantity_NOC_YELLOW),
                XCAFDoc_ColorSurf);

            // Historical documents may contain both an imported XDE material
            // and the app's legacy preset/color attributes. The authored
            // values are deliberately authoritative in that coexistence case.
            TDataStd_Integer::Set(
                definition.FindChild(11),
                Graphic3d_NameOfMaterial_Gold);
            TDataStd_Integer::Set(
                definition.FindChild(12),
                Quantity_NOC_RED);
        });
}

- (NSArray<NSDictionary<NSString *, NSNumber *> *> *)
    debugDisplayedShapePresentationStates {
    return [GLController debugDisplayedShapePresentationStates];
}

- (NSDictionary<NSString *, id> *_Nullable)debugViewportRGBA {
    return [GLController debugViewportRGBA];
}

- (NSDictionary<NSString *, NSNumber *> *)debugFramebufferStatistics {
    return [GLController debugFramebufferStatistics];
}

- (NSInteger)debugSelectedShapeCount {
    return [GLController debugSelectedShapeCount];
}

- (NSDictionary<NSString *, id> *)debugTopologySelectionState {
    NSMutableDictionary<NSString *, id> *state = [@{
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
        @"publicMode": @(PrimitiveSelectionTypeNone),
        @"publicMatchesNative": @NO,
        @"activeTool": @(PrimitiveGizmoTypeNone),
        @"canApply": @NO,
        @"canDelete": @NO,
        @"canDuplicate": @NO,
        @"canApplyMaterial": @NO,
        @"availableToolMask": @0,
    } mutableCopy];
    if (![NSThread isMainThread]) {
        // The native telemetry bridge also fails closed off-main. Do not defeat
        // that boundary by reading public ivars or capability/controller state.
        return [state copy];
    }
    if (GLController != nil) {
        NSDictionary<NSString *, id> *nativeState =
            [GLController debugTopologySelectionState];
        if (nativeState != nil) {
            [state addEntriesFromDictionary:nativeState];
        }
    }

    unsigned long long availableToolMask = 0;
    // Use the public capability-filtered rail, not its raw candidate backing
    // array. TriangleMesh selections intentionally retain object tools while
    // filtering Scale and BRep-only topology operations from admission.
    for (NSNumber *tool in [self availableGizmoTypes]) {
        const NSUInteger rawValue = tool.unsignedIntegerValue;
        if (rawValue < 64) {
            availableToolMask |= 1ULL << rawValue;
        }
    }
    const PrimitiveSelectionType acceptedMode =
        (PrimitiveSelectionType)[state[@"acceptedMode"] unsignedIntegerValue];
    state[@"publicMode"] = @(_currentSelectionType);
    state[@"publicMatchesNative"] = @(
        [state[@"ready"] boolValue]
        && acceptedMode == _currentSelectionType);
    state[@"activeTool"] = @(_currentGizmoType);
    state[@"canApply"] = @([self canApply]);
    state[@"canDelete"] = @([self canDelete]);
    state[@"canDuplicate"] = @([self canDuplicate]);
    state[@"canApplyMaterial"] = @(self.can_apply_material);
    state[@"availableToolMask"] = @(availableToolMask);
    return [state copy];
}

- (BOOL)debugDetectAnyDisplayedShape {
    return [GLController debugDetectAnyDisplayedShape];
}

- (BOOL)debugDetectReversedFaceTopologyIndexWithEntityIdentifier:
			(NSString *)entityIdentifier
	faceTopologyIndex:(NSUInteger)faceTopologyIndex {
	return [GLController
		debugDetectReversedFaceTopologyIndexWithEntityIdentifier:
			entityIdentifier
		faceTopologyIndex:faceTopologyIndex];
}

- (BOOL)debugDetectReversedEdgeTopologyIndexWithEntityIdentifier:
			(NSString *)entityIdentifier
	edgeTopologyIndex:(NSUInteger)edgeTopologyIndex {
	return [GLController
		debugDetectReversedEdgeTopologyIndexWithEntityIdentifier:
			entityIdentifier
		edgeTopologyIndex:edgeTopologyIndex];
}

- (BOOL)debugDetectAlternatingForeignSelectableEdgeWithEntityIdentifier:
			(NSString *)entityIdentifier
	foreignEntityIdentifier:(NSString *)foreignEntityIdentifier
	edgeTopologyIndex:(NSUInteger)edgeTopologyIndex {
	return [GLController
		debugDetectAlternatingForeignSelectableEdgeWithEntityIdentifier:
			entityIdentifier
		foreignEntityIdentifier:foreignEntityIdentifier
		edgeTopologyIndex:edgeTopologyIndex];
}

- (BOOL)debugSelectAnyDisplayedTopologyElement {
    return [GLController debugSelectAnyDisplayedTopologyElement];
}

- (BOOL)debugSelectFaceTopologyIndicesWithEntityIdentifier:
            (NSString *)entityIdentifier
    faceTopologyIndices:(NSArray<NSNumber *> *)faceTopologyIndices {
    return [GLController
        debugSelectFaceTopologyIndicesWithEntityIdentifier:entityIdentifier
        faceTopologyIndices:faceTopologyIndices];
}

- (BOOL)debugSelectReversedFaceTopologyIndexWithEntityIdentifier:
            (NSString *)entityIdentifier
    faceTopologyIndex:(NSUInteger)faceTopologyIndex {
    return [GLController
        debugSelectReversedFaceTopologyIndexWithEntityIdentifier:
            entityIdentifier
        faceTopologyIndex:faceTopologyIndex];
}

- (BOOL)debugSelectEdgeTopologyIndicesWithEntityIdentifier:
			(NSString *)entityIdentifier
	edgeTopologyIndices:(NSArray<NSNumber *> *)edgeTopologyIndices {
	return [GLController
		debugSelectEdgeTopologyIndicesWithEntityIdentifier:entityIdentifier
		edgeTopologyIndices:edgeTopologyIndices];
}

- (BOOL)debugSelectReversedEdgeTopologyIndexWithEntityIdentifier:
			(NSString *)entityIdentifier
	edgeTopologyIndex:(NSUInteger)edgeTopologyIndex {
	return [GLController
		debugSelectReversedEdgeTopologyIndexWithEntityIdentifier:
			entityIdentifier
		edgeTopologyIndex:edgeTopologyIndex];
}

- (BOOL)debugSelectValidAndForeignEdgeOwnersWithEntityIdentifier:
			(NSString *)entityIdentifier
	foreignEntityIdentifier:(NSString *)foreignEntityIdentifier
	edgeTopologyIndex:(NSUInteger)edgeTopologyIndex {
	return [GLController
		debugSelectValidAndForeignEdgeOwnersWithEntityIdentifier:
			entityIdentifier
		foreignEntityIdentifier:foreignEntityIdentifier
		edgeTopologyIndex:edgeTopologyIndex];
}

- (Core3DSelectionTypeChangeResult)
    debugTrySetNativeSelectionTypeWithoutPublicSync:
        (PrimitiveSelectionType)mode {
    if (![NSThread isMainThread] || GLController == nil) {
        return Core3DSelectionTypeChangeResultNotReady;
    }
    return [GLController trySetSelectionType:mode];
}

- (void)debugInvokeNativeSelectAll {
    [GLController selectAll];
}

- (void)debugInvokeNativeDeleteSelected {
    [GLController deleteSelected];
}

- (PrimitiveGizmoType)debugNativeGizmoType {
    return GLController == nil
        ? PrimitiveGizmoTypeNone
        : [GLController getGizmoType];
}

- (void)debugSetNativeGizmoTypeWithoutPublicSync:
    (PrimitiveGizmoType)type {
    [GLController setGizmoType:type];
}

- (void)debugRefreshSelectionState {
    [GLController refreshSelectionState];
}

- (BOOL)debugSelectRetainedOperationPresentation {
    return [GLController debugSelectRetainedOperationPresentation];
}

- (BOOL)debugSetFirstDisplayedShapeSelectionMode:
    (PrimitiveSelectionType)mode {
    return [GLController debugSetFirstDisplayedShapeSelectionMode:mode];
}

- (BOOL)debugSetDisplayedShapeSelectionModeWithEntityIdentifier:
            (NSString *)entityIdentifier
    mode:(PrimitiveSelectionType)mode {
    return [GLController
        debugSetDisplayedShapeSelectionModeWithEntityIdentifier:
            entityIdentifier
        mode:mode];
}

- (void)debugSetSelectionModeVerificationFailureCount:(NSUInteger)count {
    [GLController debugSetSelectionModeVerificationFailureCount:count];
}

- (NSArray<NSString *> *_Nullable)debugFreeShapeEntityIdentifiers {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr) { return nil; }
    try {
        OCC_CATCH_SIGNALS
        const auto document = GLController.viewer->getDocument();
        if (document.IsNull()) { return nil; }
        const auto transaction = document->ChangeDocument();
        if (transaction.IsNull() || transaction->HasOpenCommand()
            || !XCAFDoc_DocumentTool::CheckShapeTool(transaction->Main())) { return nil; }
        const auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(transaction->Main());
        if (shapeTool.IsNull()) { return nil; }
        TDF_LabelSequence labels;
        shapeTool->GetFreeShapes(labels);
        if (labels.Length() > 256) { return nil; }
        NSMutableArray<NSString *> *identifiers = [NSMutableArray array];
        std::unordered_set<std::string> unique;
        for (Standard_Integer index = 1; index <= labels.Length(); ++index) {
            const auto identifier = document->EntityIdentifierForLabel(labels.Value(index));
            if (identifier.empty() || identifier.size() > 128
                || !unique.insert(identifier).second) { return nil; }
            NSString *value = [[NSString alloc] initWithBytes:identifier.data()
                length:identifier.size() encoding:NSUTF8StringEncoding];
            if (value == nil) { return nil; }
            [identifiers addObject:value];
        }
        return [identifiers copy];
    } catch (...) { return nil; }
}

- (void)debugSetMaximumDisplayTraversalNodes:(NSUInteger)limit {
    [GLController debugSetMaximumDisplayTraversalNodes:limit];
}

- (void)debugSetMaximumLeafPresentations:(NSUInteger)limit {
    [GLController debugSetMaximumLeafPresentations:limit];
}

- (void)debugSetMaximumProjectTopologyValidationNodes:(NSUInteger)limit {
    [GLController debugSetMaximumProjectTopologyValidationNodes:limit];
}

- (void)debugResetProjectTopologyValidationCounters {
    [GLController debugResetProjectTopologyValidationCounters];
}

- (NSUInteger)debugBoundedProjectTopologyValidationCount {
    return [GLController debugBoundedProjectTopologyValidationCount];
}

- (NSUInteger)debugGeometricBRepValidationCount {
    return [GLController debugGeometricBRepValidationCount];
}

- (void)debugSetSceneSnapshotTriangulationFailureMode:(NSInteger)mode {
    if (GLController == nil || GLController.viewer == nullptr) {
        return;
    }
    using Failure =
        core3d::scene::OcctSceneSnapshotBuilder::DebugTriangulationFailure;
    Failure failure = Failure::None;
    if (mode == 1) {
        failure = Failure::Missing;
    } else if (mode == 2) {
        failure = Failure::Incompatible;
    }
    GLController.viewer->DebugSetSceneSnapshotTriangulationFailure(failure);
}

- (void)debugResetSceneSnapshotMesherInvocationCount {
    if (GLController != nil && GLController.viewer != nullptr) {
        GLController.viewer->DebugResetSceneSnapshotMesherInvocationCount();
    }
}

- (NSUInteger)debugSceneSnapshotMesherInvocationCount {
    return GLController == nil || GLController.viewer == nullptr
        ? 0
        : static_cast<NSUInteger>(
            GLController.viewer->DebugSceneSnapshotMesherInvocationCount());
}

- (uint64_t)debugPublishedDocumentGeneration {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr) {
        return 0;
    }
    return GLController.viewer->DebugPublishedDocumentGeneration();
}

- (uint64_t)debugPublishedModelRevision {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr) {
        return 0;
    }
    return GLController.viewer->DebugPublishedModelRevision();
}

- (void)debugSetBrowserSelectionFailureMode:(NSInteger)mode {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr) { return; }
    const auto interactor = GLController.viewer->getObjectInteractor();
    if (interactor != nullptr) {
        interactor->debugSetBrowserSelectionFailureMode(static_cast<Standard_Integer>(mode));
    }
}

- (BOOL)debugHideOccurrenceWithInvisibleLayerAtTranslationX:(CGFloat)x {
    if (GLController == nil || GLController.viewer == nullptr) {
        return NO;
    }
    const Handle(OcctDocument) occtDocument =
        GLController.viewer->getDocument();
    const Handle(TDocStd_Document) document = occtDocument.IsNull()
        ? Handle(TDocStd_Document)()
        : occtDocument->Document();
    const Handle(XCAFDoc_LayerTool) layerTool = document.IsNull()
        ? Handle(XCAFDoc_LayerTool)()
        : XCAFDoc_DocumentTool::LayerTool(document->Main());
    if (document.IsNull() || layerTool.IsNull()) {
        return NO;
    }
    TDF_Label occurrence;
    XCAFPrs_DocumentExplorer explorer(
        document,
        XCAFPrs_DocumentExplorerFlags_OnlyLeafNodes
            | XCAFPrs_DocumentExplorerFlags_NoStyle);
    for (; explorer.More(); explorer.Next()) {
        const XCAFPrs_DocumentNode& node = explorer.Current();
        if (!node.Label.IsNull()
            && Abs(node.Location.Transformation()
                       .TranslationPart().X() - x) < 0.001) {
            occurrence = node.Label;
            break;
        }
    }
    if (occurrence.IsNull()) {
        return NO;
    }
    const TDF_Label hiddenLayer = layerTool->AddLayer(
        TCollection_ExtendedString("Debug invisible layer"));
    if (hiddenLayer.IsNull()) {
        return NO;
    }
    layerTool->SetVisibility(hiddenLayer, Standard_False);
    layerTool->SetLayer(occurrence, hiddenLayer, Standard_False);
    GLController.viewer->redrawDocument();
    [GLController requestRender];
    return YES;
}

- (NSData *_Nullable)debugStyledSubshapeBinXCAFFixtureData {
    return Core3DCreateDebugBinXCAFFixture(
        @"styled-subshape-extrusion-fixture",
        [](const Handle(TDocStd_Document)& document) {
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            const Handle(XCAFDoc_ColorTool) colorTool =
                XCAFDoc_DocumentTool::ColorTool(document->Main());
            if (shapeTool.IsNull() || colorTool.IsNull()) {
                throw Standard_Failure(
                    "Unable to create styled subshape fixture tools");
            }
            const TopoDS_Shape box =
                BRepPrimAPI_MakeBox(50.0, 50.0, 50.0).Shape();
            const TDF_Label definition = shapeTool->AddShape(
                box, Standard_False, Standard_True);
            TopExp_Explorer face(box, TopAbs_FACE);
            if (definition.IsNull() || !face.More()) {
                throw Standard_Failure(
                    "Unable to create styled subshape fixture geometry");
            }
            const TDF_Label faceLabel = shapeTool->AddSubShape(
                definition, face.Current());
            if (faceLabel.IsNull()) {
                throw Standard_Failure(
                    "Unable to label styled fixture face");
            }
            colorTool->SetColor(
                faceLabel,
                Quantity_Color(Quantity_NOC_BLUE1),
                XCAFDoc_ColorSurf);
        });
}

- (NSData *_Nullable)debugOversizedExtrusionSolidBinXCAFFixtureData {
    return Core3DCreateDebugBinXCAFFixture(
        @"oversized-extrusion-solid-fixture",
        [](const Handle(TDocStd_Document)& document) {
            constexpr Standard_Integer sideCount = 300;
            constexpr Standard_Real pi =
                3.141592653589793238462643383279502884;
            BRepBuilderAPI_MakePolygon polygon;
            for (Standard_Integer index = 0;
                 index < sideCount;
                 ++index) {
                const Standard_Real angle =
                    2.0 * pi * static_cast<Standard_Real>(index)
                    / static_cast<Standard_Real>(sideCount);
                polygon.Add(gp_Pnt(
                    50.0 * std::cos(angle),
                    50.0 * std::sin(angle),
                    0.0));
            }
            polygon.Close();
            if (!polygon.IsDone()) {
                throw Standard_Failure(
                    "Unable to create oversized fixture wire");
            }
            BRepBuilderAPI_MakeFace face(polygon.Wire());
            if (!face.IsDone()) {
                throw Standard_Failure(
                    "Unable to create oversized fixture face");
            }
            const TopoDS_Shape solid = BRepPrimAPI_MakePrism(
                face.Face(), gp_Vec(0.0, 0.0, 20.0)).Shape();
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            if (solid.IsNull() || solid.ShapeType() != TopAbs_SOLID
                || shapeTool.IsNull()
                || shapeTool->AddShape(
                    solid, Standard_False, Standard_True).IsNull()) {
                throw Standard_Failure(
                    "Unable to create oversized extrusion fixture solid");
            }
        });
}

- (NSData *_Nullable)debugInvalidBRepSolidBinXCAFFixtureData {
    return Core3DCreateDebugBinXCAFFixture(
        @"invalid-brep-solid-face-admission-fixture",
        [](const Handle(TDocStd_Document)& document) {
            const TopoDS_Shape box =
                BRepPrimAPI_MakeBox(50.0, 50.0, 50.0).Shape();
            BRep_Builder builder;
            TopoDS_Shell openShell;
            builder.MakeShell(openShell);
            Standard_Integer faceCount = 0;
            for (TopExp_Explorer face(box, TopAbs_FACE);
                 face.More() && faceCount < 5; face.Next()) {
                builder.Add(openShell, face.Current());
                ++faceCount;
            }
            TopoDS_Solid invalidSolid;
            builder.MakeSolid(invalidSolid);
            builder.Add(invalidSolid, openShell);
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            if (faceCount != 5 || invalidSolid.IsNull()
                || invalidSolid.ShapeType() != TopAbs_SOLID
                || BRepCheck_Analyzer(
                    invalidSolid, Standard_True).IsValid()
                || shapeTool.IsNull()
                || shapeTool->AddShape(
                    invalidSolid,
                    Standard_False,
                    Standard_True).IsNull()) {
                throw Standard_Failure(
                    "Unable to create invalid BRep admission fixture");
            }
        });
}

- (NSData *_Nullable)debugOccurrenceAmplifiedBevelSolidBinXCAFFixtureData {
	return Core3DCreateDebugBinXCAFFixture(
		@"occurrence-amplified-bevel-solid-fixture",
		[](const Handle(TDocStd_Document)& document) {
			const TopoDS_Shape box =
				BRepPrimAPI_MakeBox(50.0, 50.0, 50.0).Shape();
			TopExp_Explorer shellExplorer(box, TopAbs_SHELL);
			if (!shellExplorer.More()) {
				throw Standard_Failure(
					"Unable to resolve shared Bevel fixture shell");
			}
			const TopoDS_Shell sharedShell =
				TopoDS::Shell(shellExplorer.Current());
			BRep_Builder builder;
			TopoDS_Solid amplifiedSolid;
			builder.MakeSolid(amplifiedSolid);
			constexpr Standard_Size occurrenceCount = 96;
			for (Standard_Size index = 0; index < occurrenceCount; ++index) {
				builder.Add(amplifiedSolid, sharedShell);
			}
			Standard_Size directShellCount = 0;
			for (TopoDS_Iterator child(
					 amplifiedSolid, Standard_False, Standard_False);
				 child.More(); child.Next()) {
				++directShellCount;
			}
			const Handle(XCAFDoc_ShapeTool) shapeTool =
				XCAFDoc_DocumentTool::ShapeTool(document->Main());
			if (amplifiedSolid.IsNull()
				|| amplifiedSolid.ShapeType() != TopAbs_SOLID
				|| directShellCount != occurrenceCount
				|| shapeTool.IsNull()
				|| shapeTool->AddShape(
					amplifiedSolid,
					Standard_False,
					Standard_True).IsNull()) {
				throw Standard_Failure(
					"Unable to create occurrence-amplified Bevel fixture");
			}
		});
}

- (NSData *_Nullable)debugOBJColorConventionBinXCAFFixtureData {
    return Core3DCreateDebugBinXCAFFixture(@"obj-color-conventions",
        [](const Handle(TDocStd_Document)& document) {
            const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
            const auto materials = XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
            const auto colors = XCAFDoc_DocumentTool::ColorTool(document->Main());
            const char* names[] = {"Common", "PBR", "TexturedPBR", "SurfaceColor"};
            for (int index = 0; index < 4; ++index) {
                const auto label = shapes->AddShape(BRepPrimAPI_MakeBox(
                    gp_Pnt(index * 20.0, 0, 0), 10.0, 10.0, 10.0).Shape(), Standard_False);
                TDataStd_Name::Set(label, TCollection_ExtendedString(names[index]));
                if (index == 3) {
                    colors->SetColor(label, Quantity_ColorRGBA(
                        Quantity_Color(0.01, 0.09, 0.49, Quantity_TOC_RGB), 0.6f), XCAFDoc_ColorSurf);
                    continue;
                }
                Handle(XCAFDoc_VisMaterial) material = new XCAFDoc_VisMaterial();
                if (index == 0) {
                    XCAFDoc_VisMaterialCommon common;
                    common.DiffuseColor = Quantity_Color(0.04, 0.25, 0.64, Quantity_TOC_RGB);
                    common.AmbientColor = Quantity_Color(0.01, 0.02, 0.03, Quantity_TOC_RGB);
                    common.SpecularColor = Quantity_Color(0.16, 0.16, 0.16, Quantity_TOC_RGB);
                    common.Transparency = 0.35f;
                    material->SetCommonMaterial(common);
                } else {
                    XCAFDoc_VisMaterialPBR pbr;
                    pbr.BaseColor = Quantity_ColorRGBA(index == 1
                        ? Quantity_Color(0.16, 0.36, 0.81, Quantity_TOC_RGB)
                        : Quantity_Color(0.25, 0.49, 0.09, Quantity_TOC_RGB), index == 1 ? 0.4f : 0.8f);
                    pbr.Metallic = 0.3f;
                    pbr.Roughness = 0.7f;
                    if (index == 2) {
                        NSData* png = [[NSData alloc] initWithBase64EncodedString:
                            @"iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAYAAACqaXHeAAAApklEQVR42u3aQQ3AQAzEwFyZF3kKY06qTWAtK8+cndmBnJ1X7j9y/AYKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNAXQApoCaAFNAbSApgBaQFMALaApgBbQnJml/wF2vQsoQAG0gKYAWkBTAC2gKYAW0BRAC2gKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNL8P8AESbQf6Ta5RUwAAAABJRU5ErkJggg==" options:0];
                        Handle(NCollection_Buffer) buffer = new NCollection_Buffer(
                            NCollection_BaseAllocator::CommonBaseAllocator(), png.length);
                        std::memcpy(buffer->ChangeData(), png.bytes, png.length);
                        pbr.BaseColorTexture = new Image_Texture(buffer, TCollection_AsciiString("obj-color-fixture"));
                    }
                    material->SetPbrMaterial(pbr);
                }
                const auto materialLabel = materials->AddMaterial(material, TCollection_AsciiString(names[index]));
                materials->SetShapeMaterial(label, materialLabel);
            }
        });
}

- (NSData *_Nullable)debugCommonTextureBinXCAFFixtureData {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL* baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.common-texture-fixture", NSUUID.UUID.UUIDString]];
    NSString* xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSData* result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        Handle(XCAFDoc_VisMaterialTool) tool =
            XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
        if (document.IsNull() || shapeTool.IsNull() || tool.IsNull()) {
            throw Standard_Failure("Unable to create Common texture fixture");
        }
        const TDF_Label shapeLabel = shapeTool->AddShape(
            BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(),
            Standard_False,
            Standard_True);
        NSData* png = [[NSData alloc] initWithBase64EncodedString:
            @"iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAYAAACqaXHeAAAApklEQVR42u3aQQ3AQAzEwFyZF3kKY06qTWAtK8+cndmBnJ1X7j9y/AYKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNAXQApoCaAFNAbSApgBaQFMALaApgBbQnJml/wF2vQsoQAG0gKYAWkBTAC2gKYAW0BRAC2gKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNL8P8AESbQf6Ta5RUwAAAABJRU5ErkJggg=="
            options:0];
        Handle(NCollection_Buffer) buffer = new NCollection_Buffer(
            NCollection_BaseAllocator::CommonBaseAllocator(), png.length);
        if (shapeLabel.IsNull() || png.length == 0 || buffer.IsNull()) {
            throw Standard_Failure("Unable to create Common texture buffer");
        }
        std::memcpy(buffer->ChangeData(), png.bytes, png.length);

        XCAFDoc_VisMaterialPBR pbr;
        pbr.BaseColor = Quantity_ColorRGBA(
            Quantity_Color(0.25, 0.55, 0.85, Quantity_TOC_sRGB),
            1.0f);
        pbr.Metallic = 0.35f;
        pbr.Roughness = 0.65f;
        Handle(XCAFDoc_VisMaterial) material =
            new XCAFDoc_VisMaterial();
        material->SetPbrMaterial(pbr);
        XCAFDoc_VisMaterialCommon common =
            material->ConvertToCommonMaterial();
        common.DiffuseTexture = new Image_Texture(
            buffer,
            TCollection_AsciiString("shapeyard-common-diffuse"));
        material->SetCommonMaterial(common);
        const TDF_Label materialLabel = tool->AddMaterial(
            material,
            TCollection_AsciiString("Common texture fixture"));
        if (materialLabel.IsNull()) {
            throw Standard_Failure("Unable to add Common texture material");
        }
        tool->SetShapeMaterial(shapeLabel, materialLabel);
        if (application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure("Unable to save Common texture fixture");
        }
        result = [NSData dataWithContentsOfFile:xbfPath];
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return result;
}

- (NSData *_Nullable)debugUnsupportedPBRTextureBinXCAFFixtureData {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL* baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.unsupported-pbr-texture-fixture", NSUUID.UUID.UUIDString]];
    NSString* xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSData* result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        Handle(XCAFDoc_ShapeTool) shapeTool = document.IsNull()
            ? Handle(XCAFDoc_ShapeTool)()
            : XCAFDoc_DocumentTool::ShapeTool(document->Main());
        Handle(XCAFDoc_VisMaterialTool) materialTool = document.IsNull()
            ? Handle(XCAFDoc_VisMaterialTool)()
            : XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
        if (shapeTool.IsNull() || materialTool.IsNull()) {
            throw Standard_Failure(
                "Unable to create unsupported PBR texture fixture");
        }
        const TDF_Label shapeLabel = shapeTool->AddShape(
            BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(),
            Standard_False,
            Standard_True);
        NSData* png = [[NSData alloc] initWithBase64EncodedString:
            @"iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAYAAACqaXHeAAAApklEQVR42u3aQQ3AQAzEwFyZF3kKY06qTWAtK8+cndmBnJ1X7j9y/AYKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNAXQApoCaAFNAbSApgBaQFMALaApgBbQnJml/wF2vQsoQAG0gKYAWkBTAC2gKYAW0BRAC2gKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNL8P8AESbQf6Ta5RUwAAAABJRU5ErkJggg=="
            options:0];
        Handle(NCollection_Buffer) buffer = new NCollection_Buffer(
            NCollection_BaseAllocator::CommonBaseAllocator(), png.length);
        if (shapeLabel.IsNull() || png.length == 0 || buffer.IsNull()) {
            throw Standard_Failure(
                "Unable to create unsupported PBR texture buffer");
        }
        std::memcpy(buffer->ChangeData(), png.bytes, png.length);
        XCAFDoc_VisMaterialPBR pbr;
        pbr.BaseColor = Quantity_ColorRGBA(
            Quantity_Color(0.6, 0.6, 0.6, Quantity_TOC_sRGB), 1.0f);
        pbr.Metallic = 0.2f;
        pbr.Roughness = 0.7f;
        pbr.NormalTexture = new Image_Texture(
            buffer, TCollection_AsciiString("shapeyard-normal-map"));
        Handle(XCAFDoc_VisMaterial) material =
            new XCAFDoc_VisMaterial();
        material->SetPbrMaterial(pbr);
        const TDF_Label materialLabel = materialTool->AddMaterial(
            material,
            TCollection_AsciiString("Unsupported normal-map fixture"));
        if (materialLabel.IsNull()) {
            throw Standard_Failure(
                "Unable to add unsupported PBR texture material");
        }
        materialTool->SetShapeMaterial(shapeLabel, materialLabel);
        if (application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure(
                "Unable to save unsupported PBR texture fixture");
        }
        result = [NSData dataWithContentsOfFile:xbfPath];
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return result;
}

- (NSData *_Nullable)debugUnsupportedMetallicRoughnessTextureBinXCAFFixtureData {
    return Core3DCreateDebugBinXCAFFixture(
        @"unsupported-metallic-roughness-texture-fixture",
        [](const Handle(TDocStd_Document)& document) {
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            const Handle(XCAFDoc_VisMaterialTool) materialTool =
                XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
            if (shapeTool.IsNull() || materialTool.IsNull()) {
                throw Standard_Failure(
                    "Unable to create metallic-roughness fixture tools");
            }
            const TDF_Label shapeLabel = shapeTool->AddShape(
                BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(),
                Standard_False,
                Standard_True);
            NSData* png = [[NSData alloc] initWithBase64EncodedString:
                @"iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAYAAACqaXHeAAAApklEQVR42u3aQQ3AQAzEwFyZF3kKY06qTWAtK8+cndmBnJ1X7j9y/AYKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNAXQApoCaAFNAbSApgBaQFMALaApgBbQnJml/wF2vQsoQAG0gKYAWkBTAC2gKYAW0BRAC2gKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNL8P8AESbQf6Ta5RUwAAAABJRU5ErkJggg=="
                options:0];
            Handle(NCollection_Buffer) buffer = new NCollection_Buffer(
                NCollection_BaseAllocator::CommonBaseAllocator(),
                png.length);
            if (shapeLabel.IsNull() || png.length == 0 || buffer.IsNull()
                || buffer->ChangeData() == nullptr) {
                throw Standard_Failure(
                    "Unable to create metallic-roughness fixture buffer");
            }
            std::memcpy(buffer->ChangeData(), png.bytes, png.length);
            XCAFDoc_VisMaterialPBR pbr;
            pbr.BaseColor = Quantity_ColorRGBA(
                Quantity_Color(0.6, 0.6, 0.6, Quantity_TOC_sRGB), 1.0f);
            pbr.Metallic = 0.2f;
            pbr.Roughness = 0.7f;
            pbr.MetallicRoughnessTexture = new Image_Texture(
                buffer,
                TCollection_AsciiString("shapeyard-metallic-roughness-map"));
            Handle(XCAFDoc_VisMaterial) material =
                new XCAFDoc_VisMaterial();
            material->SetPbrMaterial(pbr);
            const TDF_Label materialLabel = materialTool->AddMaterial(
                material,
                TCollection_AsciiString(
                    "Unsupported metallic-roughness fixture"));
            if (materialLabel.IsNull()) {
                throw Standard_Failure(
                    "Unable to add metallic-roughness fixture material");
            }
            materialTool->SetShapeMaterial(shapeLabel, materialLabel);
        });
}

- (NSData *_Nullable)debugImportedEmissiveTextureBinXCAFFixtureData {
    return Core3DCreateDebugBinXCAFFixture(
        @"imported-emissive-texture-fixture",
        [](const Handle(TDocStd_Document)& document) {
            const Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            const Handle(XCAFDoc_VisMaterialTool) materialTool =
                XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
            if (shapeTool.IsNull() || materialTool.IsNull()) {
                throw Standard_Failure(
                    "Unable to create imported emissive fixture tools");
            }
            const TDF_Label shapeLabel = shapeTool->AddShape(
                BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(),
                Standard_False,
                Standard_True);
            NSData* png = [[NSData alloc] initWithBase64EncodedString:
                @"iVBORw0KGgoAAAANSUhEUgAAAEAAAABACAYAAACqaXHeAAAApklEQVR42u3aQQ3AQAzEwFyZF3kKY06qTWAtK8+cndmBnJ1X7j9y/AYKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNAXQApoCaAFNAbSApgBaQFMALaApgBbQnJml/wF2vQsoQAG0gKYAWkBTAC2gKYAW0BRAC2gKoAU0BdACmgJoAU0BtICmAFpAUwAtoCmAFtAUQAtoCqAFNL8P8AESbQf6Ta5RUwAAAABJRU5ErkJggg=="
                options:0];
            Handle(NCollection_Buffer) buffer = new NCollection_Buffer(
                NCollection_BaseAllocator::CommonBaseAllocator(),
                png.length);
            if (shapeLabel.IsNull() || png.length == 0 || buffer.IsNull()
                || buffer->ChangeData() == nullptr) {
                throw Standard_Failure(
                    "Unable to create imported emissive fixture buffer");
            }
            std::memcpy(buffer->ChangeData(), png.bytes, png.length);
            XCAFDoc_VisMaterialPBR pbr;
            pbr.BaseColor = Quantity_ColorRGBA(
                Quantity_Color(0.2, 0.3, 0.4, Quantity_TOC_sRGB), 1.0f);
            pbr.EmissiveFactor = Graphic3d_Vec3(0.25f, 0.5f, 0.75f);
            pbr.Metallic = 0.2f;
            pbr.Roughness = 0.7f;
            pbr.EmissiveTexture = new Image_Texture(
                buffer,
                TCollection_AsciiString("imported-emissive-map"));
            Handle(XCAFDoc_VisMaterial) material =
                new XCAFDoc_VisMaterial();
            material->SetPbrMaterial(pbr);
            // A masked material makes the fixture prove dynamically that the
            // emissive image's alpha channel never controls draw/pick coverage.
            // Base alpha remains one, so only an incorrect emissive-alpha
            // dependency could discard the low-alpha test texels.
            material->SetAlphaMode(
                Graphic3d_AlphaMode_Mask,
                0.75f);
            const TDF_Label materialLabel = materialTool->AddMaterial(
                material,
                TCollection_AsciiString("Imported emissive fixture"));
            if (materialLabel.IsNull()) {
                throw Standard_Failure(
                    "Unable to add imported emissive fixture material");
            }
            materialTool->SetShapeMaterial(shapeLabel, materialLabel);
        });
}

- (NSData *_Nullable)debugMaskedDoubleSidedPBRBinXCAFFixtureData {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL* baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.masked-pbr-fixture", NSUUID.UUID.UUIDString]];
    NSString* xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSData* result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        Handle(XCAFDoc_VisMaterialTool) tool =
            XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
        if (document.IsNull() || shapeTool.IsNull() || tool.IsNull()) {
            throw Standard_Failure("Unable to create masked PBR fixture");
        }
        const TDF_Label shapeLabel = shapeTool->AddShape(
            BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(),
            Standard_False,
            Standard_True);
        XCAFDoc_VisMaterialPBR pbr;
        pbr.BaseColor = Quantity_ColorRGBA(
            Quantity_Color(0.2, 0.7, 0.4, Quantity_TOC_sRGB),
            0.73f);
        pbr.Metallic = 0.27f;
        pbr.Roughness = 0.63f;
        Handle(XCAFDoc_VisMaterial) material =
            new XCAFDoc_VisMaterial();
        material->SetPbrMaterial(pbr);
        material->SetCommonMaterial(
            material->ConvertToCommonMaterial());
        material->SetAlphaMode(Graphic3d_AlphaMode_Mask, 0.37f);
        material->SetFaceCulling(
            Graphic3d_TypeOfBackfacingModel_DoubleSided);
        const TDF_Label materialLabel = tool->AddMaterial(
            material,
            TCollection_AsciiString("Masked double-sided fixture"));
        if (shapeLabel.IsNull() || materialLabel.IsNull()) {
            throw Standard_Failure("Unable to add masked PBR material");
        }
        tool->SetShapeMaterial(shapeLabel, materialLabel);
        if (application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure("Unable to save masked PBR fixture");
        }
        result = [NSData dataWithContentsOfFile:xbfPath];
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return result;
}

- (NSData *_Nullable)debugOpaqueFrontCulledPBRBinXCAFFixtureData {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL* baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.opaque-front-pbr-fixture", NSUUID.UUID.UUIDString]];
    NSString* xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSData* result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        Handle(XCAFDoc_VisMaterialTool) tool =
            XCAFDoc_DocumentTool::VisMaterialTool(document->Main());
        if (document.IsNull() || shapeTool.IsNull() || tool.IsNull()) {
            throw Standard_Failure(
                "Unable to create opaque front-culled PBR fixture");
        }
        const TDF_Label shapeLabel = shapeTool->AddShape(
            BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape(),
            Standard_False,
            Standard_True);
        XCAFDoc_VisMaterialPBR pbr;
        pbr.BaseColor = Quantity_ColorRGBA(
            Quantity_Color(0.8, 0.3, 0.1, Quantity_TOC_sRGB),
            0.41f);
        pbr.Metallic = 0.12f;
        pbr.Roughness = 0.78f;
        Handle(XCAFDoc_VisMaterial) material =
            new XCAFDoc_VisMaterial();
        material->SetPbrMaterial(pbr);
        material->SetCommonMaterial(
            material->ConvertToCommonMaterial());
        material->SetAlphaMode(Graphic3d_AlphaMode_Opaque, 0.62f);
        material->SetFaceCulling(
            Graphic3d_TypeOfBackfacingModel_FrontCulled);
        const TDF_Label materialLabel = tool->AddMaterial(
            material,
            TCollection_AsciiString("Opaque front-culled fixture"));
        if (shapeLabel.IsNull() || materialLabel.IsNull()) {
            throw Standard_Failure(
                "Unable to add opaque front-culled PBR material");
        }
        tool->SetShapeMaterial(shapeLabel, materialLabel);
        if (application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure(
                "Unable to save opaque front-culled PBR fixture");
        }
        result = [NSData dataWithContentsOfFile:xbfPath];
    } catch (...) {
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull()) {
            application->Close(document);
        }
    } catch (...) {
    }
    [NSFileManager.defaultManager removeItemAtURL:baseURL error:nil];
    [NSFileManager.defaultManager removeItemAtPath:xbfPath error:nil];
    return result;
}
#endif

- (void)viewWillAppear:(BOOL)animated {
    [super viewWillAppear:animated];
    [GLController requestRender];
    [self sendNotifyUIState: UIStateChangingGizmo
                            | UIStateChangingSelection
                            | UIStateChangingHistory
                            | UIStateChangingDelete
                            | UIStateChangingDuplicate
                            | UIStateChangingApplyMaterial
                            | UIStateChangingSnappingType];
}

-(UIView*) glView {
    return GLController.view;
}

- (void)viewDidLoad {
    [super viewDidLoad];

    [self addChildViewController:GLController];
    [self.view insertSubview:GLController.view atIndex:0];
    GLController.view.translatesAutoresizingMaskIntoConstraints = NO;
    [GLController.view.topAnchor constraintEqualToAnchor:self.view.topAnchor constant:0.0].active = YES;
    [GLController.view.bottomAnchor constraintEqualToAnchor:self.view.bottomAnchor constant:0.0].active = YES;
    [GLController.view.leadingAnchor constraintEqualToAnchor:self.view.leadingAnchor constant:0.0].active = YES;
    [GLController.view.trailingAnchor constraintEqualToAnchor:self.view.trailingAnchor constant:0.0].active = YES;
    [GLController didMoveToParentViewController:self];
}

- (NSUInteger)viewportRenderedFrameCount {
    return GLController.renderedFrameCount;
}

- (CGSize)viewportDrawableSize {
    return GLController.drawableSize;
}

- (BOOL)isViewportRenderLoopRunning {
    return GLController.isRenderLoopRunning;
}

- (Core3DViewportRenderingAPI)viewportRenderingAPI {
    return (Core3DViewportRenderingAPI)GLController.renderingAPIVersion;
}

- (BOOL)setCameraOrthographic:(BOOL)orthographic {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) { return NO; }
    const auto viewer = GLController.viewer;
    if (viewer == nullptr || !viewer->setCameraOrthographic(orthographic)) { return NO; }
    [GLController requestRender];
    [self viewDidInvalidateSceneSnapshot];
    return YES;
}

- (BOOL)frameModelWithSelectedObjectsOnly:(BOOL)selectedObjectsOnly {
    return [self frameModelWithSelectedObjectsOnly:selectedObjectsOnly
                           normalizedViewportRect:CGRectMake(0, 0, 1, 1)];
}

- (BOOL)frameModelWithSelectedObjectsOnly:(BOOL)selectedObjectsOnly
                normalizedViewportRect:(CGRect)rect {
    return [self frameCommittedSceneSelectedOnly:selectedObjectsOnly
                                           rect:rect objectIdentity:nullptr];
}

- (BOOL)frameObjectWithEntityIdentifier:(NSString *)entityIdentifier
                             expected:(Core3DSceneSnapshot *)expected
               normalizedViewportRect:(CGRect)rect {
    if (![NSThread isMainThread] || entityIdentifier.length == 0
        || entityIdentifier.length > 128 || expected == nil
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128) {
        return NO;
    }
    const char* entity = entityIdentifier.UTF8String;
    const char* publication = expected.publicationSourceIdentifier.UTF8String;
    if (entity == nullptr || publication == nullptr) { return NO; }
    try {
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entity,
            [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        return [self frameCommittedSceneSelectedOnly:NO rect:rect objectIdentity:&identity];
    } catch (...) { return NO; }
}

- (Core3DSavedGroupEditResult)editSavedGroupOperation:(NSInteger)operation identifier:(NSString *)entityIdentifier
    entities:(NSArray<NSString *> *)entities name:(NSString *)name expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr || expected == nil
        || name.length > 256 || entities.count > 32
        || entityIdentifier.length > 128
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128) { return Core3DSavedGroupEditResultRejected; }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) { return Core3DSavedGroupEditResultRejected; }
    const char* entity = entityIdentifier.UTF8String;
    const char* publication = expected.publicationSourceIdentifier.UTF8String;
    if (entity == nullptr || publication == nullptr) { return Core3DSavedGroupEditResultRejected; }
    try {
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entity, [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        // Copy UTF-16 by explicit length. An embedded zero must reach native
        // validation, rather than silently truncating a requested name.
        TCollection_ExtendedString requested;
        for (NSUInteger index = 0; index < name.length; ++index) {
            requested += static_cast<Standard_ExtCharacter>([name characterAtIndex:index]);
        }
        std::vector<std::string> memberIDs;
        for (NSString* member in entities) {
            if (![member isKindOfClass:NSString.class] || member.length == 0 || member.length > 128 || member.UTF8String == nullptr) {
                return Core3DSavedGroupEditResultRejected;
            }
            memberIDs.emplace_back(member.UTF8String, [member lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        }
        bool blockedByLayer = false;
        const auto result = GLController.viewer->editSavedGroup(static_cast<int>(operation), identity.entityIdentifier,
            memberIDs, requested, identity, expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),
            static_cast<std::uint32_t>(std::llround(size.height)), &blockedByLayer);
        if (blockedByLayer) { return Core3DSavedGroupEditResultBlockedByLayer; }
        switch (result) {
            case core3d::OrdinaryEditResult::NoChange: return Core3DSavedGroupEditResultUnchanged;
            case core3d::OrdinaryEditResult::Committed:
                // NotifyChanges owns document publication. Updating controls
                // here does not rebuild geometry or replace selection owners.
                [GLController refreshSelectionState];
                [self viewDidChangeViewportPresentationState];
                [self sendNotifyUIState:UIStateChangingHistory | UIStateChangingSelection | UIStateChangingGizmo
                    | UIStateChangingDelete | UIStateChangingDuplicate | UIStateChangingApply | UIStateChangingApplyMaterial];
                return Core3DSavedGroupEditResultCommitted;
            case core3d::OrdinaryEditResult::Busy: return Core3DSavedGroupEditResultBusy;
            case core3d::OrdinaryEditResult::Invalid: return Core3DSavedGroupEditResultRejected;
            case core3d::OrdinaryEditResult::OutcomeUnknown: return Core3DSavedGroupEditResultRecoveryRequired;
            case core3d::OrdinaryEditResult::RetryableFailure: return Core3DSavedGroupEditResultFailed;
        }
    } catch (...) {}
    return Core3DSavedGroupEditResultRejected;
}

- (Core3DSavedGroupEditResult)createSavedGroupWithEntityIdentifiers:(NSArray<NSString *> *)entities name:(NSString *)name expected:(Core3DSceneSnapshot *)expected {
    return [self editSavedGroupOperation:0 identifier:@"" entities:entities name:name expected:expected];
}
- (Core3DSavedGroupEditResult)renameSavedGroup:(NSString *)identifier name:(NSString *)name expected:(Core3DSceneSnapshot *)expected {
    return [self editSavedGroupOperation:1 identifier:identifier entities:@[] name:name expected:expected];
}
- (Core3DSavedGroupEditResult)ungroupSavedGroup:(NSString *)identifier expected:(Core3DSceneSnapshot *)expected {
    return [self editSavedGroupOperation:2 identifier:identifier entities:@[] name:@"" expected:expected];
}
- (Core3DSavedGroupEditResult)setSavedGroupVisibility:(NSString *)identifier visible:(BOOL)visible expected:(Core3DSceneSnapshot *)expected {
    return [self editSavedGroupOperation:visible ? 4 : 3 identifier:identifier entities:@[] name:@"" expected:expected];
}

- (BOOL)selectSavedGroup:(NSString *)entityIdentifier
                              expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr || expected == nil
        || expected.selectionMode != Core3DSceneElementKindObject
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128) {
        return NO;
    }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) { return NO; }
    const char* entity = entityIdentifier.UTF8String;
    const char* publication = expected.publicationSourceIdentifier.UTF8String;
    if (entity == nullptr || publication == nullptr) { return NO; }
    try {
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entity,
            [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        bool selectionWasTouched = false;
        const bool selected = GLController.viewer->selectSavedGroup(
            identity, expected.revisions.presentationRevision, static_cast<std::uint32_t>(std::llround(size.width)),
            static_cast<std::uint32_t>(std::llround(size.height)), selectionWasTouched);
        if (selectionWasTouched) {
            [GLController refreshSelectionState];
            [GLController requestRender];
            [self viewDidInvalidateSceneSnapshot];
        }
        return selected;
    } catch (...) { return NO; }
}

- (Core3DMeshUVAtlasResult)generateTriangleUVAtlasForEntityIdentifier:(NSString *)entityIdentifier
                                                         expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr || expected == nil
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128) { return Core3DMeshUVAtlasResultRejected; }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) { return Core3DMeshUVAtlasResultRejected; }
    const char* entity = entityIdentifier.UTF8String;
    const char* publication = expected.publicationSourceIdentifier.UTF8String;
    if (entity == nullptr || publication == nullptr) { return Core3DMeshUVAtlasResultRejected; }
    try {
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entity, [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto result = GLController.viewer->generateTriangleUVAtlas(identity,
            static_cast<std::uint32_t>(std::llround(size.width)),
            static_cast<std::uint32_t>(std::llround(size.height)));
        switch (result) {
            case core3d::OrdinaryEditResult::NoChange: return Core3DMeshUVAtlasResultUnchanged;
            case core3d::OrdinaryEditResult::Committed:
                [GLController refreshSelectionState];
                [GLController requestRender];
                [self viewDidInvalidateSceneSnapshot];
                [self sendNotifyUIState:UIStateChangingHistory];
                return Core3DMeshUVAtlasResultCommitted;
            case core3d::OrdinaryEditResult::Busy: return Core3DMeshUVAtlasResultBusy;
            case core3d::OrdinaryEditResult::Invalid: return Core3DMeshUVAtlasResultRejected;
            case core3d::OrdinaryEditResult::OutcomeUnknown: return Core3DMeshUVAtlasResultRecoveryRequired;
            case core3d::OrdinaryEditResult::RetryableFailure: return Core3DMeshUVAtlasResultFailed;
        }
    } catch (...) {}
    return Core3DMeshUVAtlasResultRejected;
}

- (Core3DMeshUVAtlasPreview *)currentMeshUVAtlasPreviewForEntityIdentifier:(NSString *)entityIdentifier
                                                               expected:(Core3DSceneSnapshot *)expected {
    return [self meshUVPreviewForEntityIdentifier:entityIdentifier options:std::nullopt expected:expected];
}

- (Core3DMeshUVAtlasPreview *)previewCoherentUVAtlasForEntityIdentifier:(NSString *)entityIdentifier
                                                       resolution:(NSInteger)resolution
                                                     gutterPixels:(NSInteger)gutterPixels
                                                         expected:(Core3DSceneSnapshot *)expected {
    if (resolution < 256 || resolution > 4096 || gutterPixels < 1 || gutterPixels > 32) return nil;
    return [self meshUVPreviewForEntityIdentifier:entityIdentifier
        options:OcctMeshUVAtlasOptions{2,static_cast<int>(resolution),static_cast<int>(gutterPixels)} expected:expected];
}

- (Core3DMeshUVAtlasPreview *)meshUVPreviewForEntityIdentifier:(NSString *)entityIdentifier
                                                   options:(const std::optional<OcctMeshUVAtlasOptions>&)options
                                                  expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr || expected == nil
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128) { return nil; }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) { return nil; }
    const char* entity = entityIdentifier.UTF8String;
    const char* publication = expected.publicationSourceIdentifier.UTF8String;
    if (entity == nullptr || publication == nullptr) { return nil; }
    try {
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entity, [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto result = GLController.viewer->previewCoherentUVAtlas(identity,
            static_cast<std::uint32_t>(std::llround(size.width)),
            static_cast<std::uint32_t>(std::llround(size.height)),
            options);
        return result ? [[Core3DMeshUVAtlasPreview alloc] initWithNativePreview:*result] : nil;
    } catch (...) {}
    return nil;
}

- (Core3DMeshUVAtlasResult)generateCoherentUVAtlasForEntityIdentifier:(NSString *)entityIdentifier
                                                       resolution:(NSInteger)resolution
                                                     gutterPixels:(NSInteger)gutterPixels
                                                         expected:(Core3DSceneSnapshot *)expected {
    if (resolution < 256 || resolution > 4096 || gutterPixels < 1 || gutterPixels > 32) return Core3DMeshUVAtlasResultRejected;
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr || expected == nil
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128) { return Core3DMeshUVAtlasResultRejected; }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) { return Core3DMeshUVAtlasResultRejected; }
    const char* entity = entityIdentifier.UTF8String;
    const char* publication = expected.publicationSourceIdentifier.UTF8String;
    if (entity == nullptr || publication == nullptr) { return Core3DMeshUVAtlasResultRejected; }
    try {
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entity, [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto result = GLController.viewer->generateTriangleUVAtlas(identity,
            static_cast<std::uint32_t>(std::llround(size.width)),
            static_cast<std::uint32_t>(std::llround(size.height)),
            OcctMeshUVAtlasOptions{2,static_cast<int>(resolution),static_cast<int>(gutterPixels)});
        switch (result) {
            case core3d::OrdinaryEditResult::NoChange: return Core3DMeshUVAtlasResultUnchanged;
            case core3d::OrdinaryEditResult::Committed:
                [GLController refreshSelectionState];
                [GLController requestRender];
                [self viewDidInvalidateSceneSnapshot];
                [self sendNotifyUIState:UIStateChangingHistory];
                return Core3DMeshUVAtlasResultCommitted;
            case core3d::OrdinaryEditResult::Busy: return Core3DMeshUVAtlasResultBusy;
            case core3d::OrdinaryEditResult::Invalid: return Core3DMeshUVAtlasResultRejected;
            case core3d::OrdinaryEditResult::OutcomeUnknown: return Core3DMeshUVAtlasResultRecoveryRequired;
            case core3d::OrdinaryEditResult::RetryableFailure: return Core3DMeshUVAtlasResultFailed;
        }
    } catch (...) {}
    return Core3DMeshUVAtlasResultRejected;
}

- (Core3DObjectNameEditResult)renameObjectWithEntityIdentifier:(NSString *)entityIdentifier
                                                      name:(NSString *)name
                                                  expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr || expected == nil
        || name.length == 0 || name.length > 256
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128) { return Core3DObjectNameEditResultRejected; }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) { return Core3DObjectNameEditResultRejected; }
    const char* entity = entityIdentifier.UTF8String;
    const char* publication = expected.publicationSourceIdentifier.UTF8String;
    if (entity == nullptr || publication == nullptr) { return Core3DObjectNameEditResultRejected; }
    try {
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entity, [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        // Copy UTF-16 by explicit length. An embedded zero must reach native
        // validation, rather than silently truncating a requested name.
        TCollection_ExtendedString requested;
        for (NSUInteger index = 0; index < name.length; ++index) {
            requested += static_cast<Standard_ExtCharacter>([name characterAtIndex:index]);
        }
        const auto result = GLController.viewer->renameObjectFromBrowser(identity, requested,
            static_cast<std::uint32_t>(std::llround(size.width)),
            static_cast<std::uint32_t>(std::llround(size.height)));
        switch (result) {
            case core3d::OrdinaryEditResult::NoChange: return Core3DObjectNameEditResultUnchanged;
            case core3d::OrdinaryEditResult::Committed:
                // NotifyChanges owns document publication. Updating controls
                // here does not rebuild geometry or replace selection owners.
                [self sendNotifyUIState:UIStateChangingHistory];
                return Core3DObjectNameEditResultCommitted;
            case core3d::OrdinaryEditResult::Busy: return Core3DObjectNameEditResultBusy;
            case core3d::OrdinaryEditResult::Invalid: return Core3DObjectNameEditResultRejected;
            case core3d::OrdinaryEditResult::OutcomeUnknown: return Core3DObjectNameEditResultRecoveryRequired;
            case core3d::OrdinaryEditResult::RetryableFailure: return Core3DObjectNameEditResultFailed;
        }
    } catch (...) {}
    return Core3DObjectNameEditResultRejected;
}

- (Core3DObjectVisibilityEditResult)setObjectVisibilityWithEntityIdentifier:(NSString *)entityIdentifier
                                                      visible:(BOOL)visible
                                                  expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr || expected == nil
        || expected.selectionMode != Core3DSceneElementKindObject
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128) { return Core3DObjectVisibilityEditResultRejected; }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) { return Core3DObjectVisibilityEditResultRejected; }
    const char* entity = entityIdentifier.UTF8String;
    const char* publication = expected.publicationSourceIdentifier.UTF8String;
    if (entity == nullptr || publication == nullptr) { return Core3DObjectVisibilityEditResultRejected; }
    try {
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entity, [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        bool blockedByLayer = false;
        const auto result = GLController.viewer->setObjectVisibilityFromBrowser(identity, visible,
            expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),
            static_cast<std::uint32_t>(std::llround(size.height)), &blockedByLayer);
        if (blockedByLayer) { return Core3DObjectVisibilityEditResultBlockedByLayer; }
        switch (result) {
            case core3d::OrdinaryEditResult::NoChange: return Core3DObjectVisibilityEditResultUnchanged;
            case core3d::OrdinaryEditResult::Committed:
                // Publish controls only after native presentation and history settle.
                [GLController refreshSelectionState];
                [self viewDidChangeViewportPresentationState];
                [self sendNotifyUIState:UIStateChangingSelection | UIStateChangingGizmo
                    | UIStateChangingDelete | UIStateChangingDuplicate | UIStateChangingApply
                    | UIStateChangingApplyMaterial | UIStateChangingHistory];
                return Core3DObjectVisibilityEditResultCommitted;
            case core3d::OrdinaryEditResult::Busy: return Core3DObjectVisibilityEditResultBusy;
            case core3d::OrdinaryEditResult::Invalid: return Core3DObjectVisibilityEditResultRejected;
            case core3d::OrdinaryEditResult::OutcomeUnknown: return Core3DObjectVisibilityEditResultRecoveryRequired;
            case core3d::OrdinaryEditResult::RetryableFailure: return Core3DObjectVisibilityEditResultFailed;
        }
    } catch (...) {}
    return Core3DObjectVisibilityEditResultRejected;
}


- (void)cancelProfileConstruction {
    if (![NSThread isMainThread]) { return; }
    _profileSolidCancelled = YES;
    core3d::Core3DViewer::cancelProfileSolid(_profileSolidWork);
}

- (void)createExtrudedProfileWithPoints:(NSArray<NSValue *> *)points
                                plane:(Core3DProfilePlane)plane
                                depth:(double)depth
                             expected:(Core3DSceneSnapshot *)expected
                           completion:(void(^)(Core3DProfileConstructionResult))completion {
    [self constructProfileWithPoints:points plane:plane parameter:depth revolve:NO circle:std::nullopt holeCenters:@[] holeRadii:@[] expected:expected completion:completion];
}

- (void)createExtrudedProfileWithPoints:(NSArray<NSValue *> *)points
                          holeCenters:(NSArray<NSValue *> *)holeCenters
                            holeRadii:(NSArray<NSNumber *> *)holeRadii
                                plane:(Core3DProfilePlane)plane depth:(double)depth
                             expected:(Core3DSceneSnapshot *)expected
                           completion:(void(^)(Core3DProfileConstructionResult))completion {
    [self constructProfileWithPoints:points plane:plane parameter:depth revolve:NO circle:std::nullopt
                        holeCenters:holeCenters holeRadii:holeRadii expected:expected completion:completion];
}

- (void)createRevolvedProfileWithPoints:(NSArray<NSValue *> *)points
                                plane:(Core3DProfilePlane)plane
                         angleDegrees:(double)angleDegrees
                             expected:(Core3DSceneSnapshot *)expected
                           completion:(void(^)(Core3DProfileConstructionResult))completion {
    [self constructProfileWithPoints:points plane:plane parameter:angleDegrees revolve:YES circle:std::nullopt holeCenters:@[] holeRadii:@[] expected:expected completion:completion];
}

- (void)createCircularProfileWithCenter:(CGPoint)center outerRadius:(double)outerRadius
                          innerRadius:(double)innerRadius plane:(Core3DProfilePlane)plane
                                depth:(double)depth expected:(Core3DSceneSnapshot *)expected
                           completion:(void(^)(Core3DProfileConstructionResult))completion {
    const std::optional<core3d::ProfileCircularSection> circle = core3d::ProfileCircularSection{
        gp_Pnt2d(center.x, center.y), outerRadius, innerRadius};
    [self constructProfileWithPoints:@[] plane:plane parameter:depth revolve:NO circle:circle holeCenters:@[] holeRadii:@[]
                           expected:expected completion:completion];
}

- (void)createRevolvedCircularProfileWithCenter:(CGPoint)center outerRadius:(double)outerRadius
                          innerRadius:(double)innerRadius plane:(Core3DProfilePlane)plane
                                angleDegrees:(double)angleDegrees expected:(Core3DSceneSnapshot *)expected
                           completion:(void(^)(Core3DProfileConstructionResult))completion {
    const std::optional<core3d::ProfileCircularSection> circle = core3d::ProfileCircularSection{
        gp_Pnt2d(center.x, center.y), outerRadius, innerRadius};
    [self constructProfileWithPoints:@[] plane:plane parameter:angleDegrees revolve:YES circle:circle holeCenters:@[] holeRadii:@[]
                           expected:expected completion:completion];
}

- (void)constructProfileWithPoints:(NSArray<NSValue *> *)points
                            plane:(Core3DProfilePlane)plane parameter:(double)depth
                          revolve:(BOOL)revolve
                           circle:(const std::optional<core3d::ProfileCircularSection>&)circle
                      holeCenters:(NSArray<NSValue *> *)holeCenters holeRadii:(NSArray<NSNumber *> *)holeRadii
                         expected:(Core3DSceneSnapshot *)expected
                       completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) { return; }
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ completion(Core3DProfileConstructionResultRejected); });
        return;
    }
    if (_profileSolidWork || _isLoading.load()) { completion(Core3DProfileConstructionResultBusy); return; }
    if (!_isSetuped || GLController == nil || GLController.viewer == nullptr
        || ![points isKindOfClass:[NSArray class]]
        || (circle ? points.count != 0 : (points.count < 3 || points.count > 64))
        || ![holeCenters isKindOfClass:[NSArray class]] || ![holeRadii isKindOfClass:[NSArray class]]
        || holeCenters.count != holeRadii.count || holeCenters.count > 16
        || (holeCenters.count > 0 && (circle || revolve))
        || !std::isfinite(depth) || depth < 1e-3 || depth > (revolve ? 360.0 : 1e6)
        || plane < Core3DProfilePlaneXY || plane > Core3DProfilePlaneYZ
        || expected == nil || expected.selectionMode != Core3DSceneElementKindObject
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128) {
        completion(Core3DProfileConstructionResultRejected); return;
    }
    const auto viewer = GLController.viewer;
    if (!viewer->canBeginCommittedEdit()) { completion(Core3DProfileConstructionResultBusy); return; }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height) || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) {
        completion(Core3DProfileConstructionResultRejected); return;
    }
    try {
        std::vector<gp_Pnt2d> outline;
        outline.reserve(points.count);
        for (id value in points) {
            if (![value isKindOfClass:[NSValue class]] || std::strcmp([value objCType], @encode(CGPoint)) != 0) {
                completion(Core3DProfileConstructionResultRejected); return;
            }
            const CGPoint point = [value CGPointValue];
            if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
                completion(Core3DProfileConstructionResultRejected); return;
            }
            outline.emplace_back(point.x, point.y);
        }
        std::vector<core3d::ProfileCircularHole> holes;
        holes.reserve(holeCenters.count);
        for (NSUInteger i = 0; i < holeCenters.count; ++i) {
            id centerValue = holeCenters[i], radiusValue = holeRadii[i];
            if (![centerValue isKindOfClass:[NSValue class]]
                || std::strcmp([centerValue objCType], @encode(CGPoint)) != 0
                || ![radiusValue isKindOfClass:[NSNumber class]]) {
                completion(Core3DProfileConstructionResultRejected); return;
            }
            const CGPoint center = [centerValue CGPointValue];
            const double radius = [radiusValue doubleValue];
            if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(radius)) {
                completion(Core3DProfileConstructionResultRejected); return;
            }
            holes.push_back({gp_Pnt2d(center.x, center.y), radius});
        }
        const char* publication = expected.publicationSourceIdentifier.UTF8String;
        if (!publication) { completion(Core3DProfileConstructionResultRejected); return; }
        core3d::ObjectFrameIdentity identity;
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto work = viewer->prepareProfileSolid(outline, static_cast<int>(plane), depth,
            identity, expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)), static_cast<std::uint32_t>(std::llround(size.height)), revolve, circle, holes);
        if (!work) { completion(Core3DProfileConstructionResultRejected); return; }
        _profileSolidWork = work; _profileSolidCancelled = NO;
        const auto geometry = core3d::Core3DViewer::profileSolidGeometry(work);
        const std::weak_ptr<core3d::Core3DViewer> expectedViewer = viewer;
        __weak Core3DViewController* weakSelf = self;
        dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^{
            const bool built = core3d::Core3DViewer::buildProfileSolidGeometry(geometry);
            dispatch_async(dispatch_get_main_queue(), ^{
                Core3DViewController* controller = weakSelf;
                if (!controller) { return; }
                const BOOL cancelled = controller->_profileSolidCancelled;
                // Main-thread work owns live scene authority. The worker's only
                // strong payload contains new private geometry and scalar values.
                const auto pendingWork = std::move(controller->_profileSolidWork);
                const auto currentViewer = expectedViewer.lock();
                if (cancelled) { completion(Core3DProfileConstructionResultCancelled); return; }
                if (!pendingWork || !currentViewer || controller->_isLoading.load() || !controller->_isSetuped
                    || ((GLViewController *)controller.glController) == nil
                    || ((GLViewController *)controller.glController).viewer != currentViewer) {
                    completion(Core3DProfileConstructionResultRejected); return;
                }
                if (!built) { completion(Core3DProfileConstructionResultFailed); return; }
                const auto native = currentViewer->commitProfileSolid(pendingWork);
                Core3DProfileConstructionResult result = Core3DProfileConstructionResultRejected;
                switch (native) {
                    case core3d::OrdinaryEditResult::Committed:
                        result = Core3DProfileConstructionResultCommitted;
                        [((GLViewController *)controller.glController) refreshSelectionState];
                        [controller viewDidChangeViewportPresentationState];
                        [controller sendNotifyUIState:UIStateChangingSelection | UIStateChangingGizmo
                            | UIStateChangingDelete | UIStateChangingDuplicate | UIStateChangingApply
                            | UIStateChangingApplyMaterial | UIStateChangingHistory];
                        break;
                    case core3d::OrdinaryEditResult::Busy: result = Core3DProfileConstructionResultBusy; break;
                    case core3d::OrdinaryEditResult::NoChange:
                    case core3d::OrdinaryEditResult::Invalid: result = Core3DProfileConstructionResultRejected; break;
                    case core3d::OrdinaryEditResult::OutcomeUnknown: result = Core3DProfileConstructionResultRecoveryRequired; break;
                    case core3d::OrdinaryEditResult::RetryableFailure: result = Core3DProfileConstructionResultFailed; break;
                }
                completion(result);
            });
        });
    } catch (...) { completion(Core3DProfileConstructionResultRejected); }
}

- (void)cancelObjectAlignment {
    if (![NSThread isMainThread]) { return; }
    _objectAlignmentCancelled = YES;
    core3d::Core3DViewer::cancelObjectAlignment(_objectAlignmentWork);
}

- (void)alignSelectedObjectsOnAxis:(Core3DTransformInspectorAxis)axis
                           anchor:(Core3DObjectAlignmentAnchor)anchor
                         expected:(Core3DSceneSnapshot *)expected
                       completion:(void(^)(Core3DObjectAlignmentResult))completion {
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ completion(Core3DObjectAlignmentResultRejected); });
        return;
    }
    if (_objectAlignmentWork || _isLoading.load()) { completion(Core3DObjectAlignmentResultBusy); return; }
    if (!_isSetuped || GLController == nil || GLController.viewer == nullptr
        || expected == nil || expected.selectionMode != Core3DSceneElementKindObject
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128
        || axis < Core3DTransformInspectorAxisX || axis > Core3DTransformInspectorAxisZ
        || anchor < Core3DObjectAlignmentAnchorMinimum || anchor > Core3DObjectAlignmentAnchorEqualGaps) {
        completion(Core3DObjectAlignmentResultRejected); return;
    }
    const auto viewer = GLController.viewer;
    if (!viewer->canBeginCommittedEdit()) { completion(Core3DObjectAlignmentResultBusy); return; }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height) || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) {
        completion(Core3DObjectAlignmentResultRejected); return;
    }
    try {
        const char* publication = expected.publicationSourceIdentifier.UTF8String;
        if (!publication) { completion(Core3DObjectAlignmentResultRejected); return; }
        core3d::ObjectFrameIdentity identity;
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto work = viewer->prepareObjectAlignment(static_cast<int>(axis),
            static_cast<core3d::ObjectAlignmentAnchor>(anchor), identity,
            expected.revisions.presentationRevision, static_cast<std::uint32_t>(std::llround(size.width)),
            static_cast<std::uint32_t>(std::llround(size.height)));
        if (!work) { completion(Core3DObjectAlignmentResultRejected); return; }
        _objectAlignmentWork = work; _objectAlignmentCancelled = NO;
        const auto measurement = core3d::Core3DViewer::objectAlignmentMeasurement(work);
        const std::weak_ptr<core3d::Core3DViewer> expectedViewer = viewer;
        __weak Core3DViewController* weakSelf = self;
        dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^{
            const bool measured = core3d::Core3DViewer::measureObjectAlignment(measurement);
            dispatch_async(dispatch_get_main_queue(), ^{
                Core3DViewController* controller = weakSelf;
                if (!controller) { return; }
                const BOOL cancelled = controller->_objectAlignmentCancelled;
                // The single in-flight slot is retained through cancellation.
                // Recover its live authority only after returning to main.
                const auto pendingWork = std::move(controller->_objectAlignmentWork);
                const auto currentViewer = expectedViewer.lock();
                if (cancelled) { completion(Core3DObjectAlignmentResultCancelled); return; }
                if (!pendingWork || !currentViewer || controller->_isLoading.load() || !controller->_isSetuped || ((GLViewController *)controller.glController) == nil
                    || ((GLViewController *)controller.glController).viewer != currentViewer) {
                    completion(Core3DObjectAlignmentResultRejected); return;
                }
                if (!measured) { completion(Core3DObjectAlignmentResultFailed); return; }
                const auto native = currentViewer->commitObjectAlignment(pendingWork);
                Core3DObjectAlignmentResult result = Core3DObjectAlignmentResultRejected;
                switch (native) {
                    case core3d::OrdinaryEditResult::NoChange: result = Core3DObjectAlignmentResultUnchanged; break;
                    case core3d::OrdinaryEditResult::Committed:
                        result = Core3DObjectAlignmentResultCommitted;
                        [((GLViewController *)controller.glController) refreshSelectionState];
                        [controller viewDidChangeViewportPresentationState];
                        [controller sendNotifyUIState:UIStateChangingSelection | UIStateChangingGizmo
                            | UIStateChangingDelete | UIStateChangingDuplicate | UIStateChangingApply
                            | UIStateChangingApplyMaterial | UIStateChangingHistory];
                        break;
                    case core3d::OrdinaryEditResult::Busy: result = Core3DObjectAlignmentResultBusy; break;
                    case core3d::OrdinaryEditResult::Invalid: result = Core3DObjectAlignmentResultRejected; break;
                    case core3d::OrdinaryEditResult::OutcomeUnknown: result = Core3DObjectAlignmentResultRecoveryRequired; break;
                    case core3d::OrdinaryEditResult::RetryableFailure: result = Core3DObjectAlignmentResultFailed; break;
                }
                completion(result);
            });
        });
    } catch (...) { completion(Core3DObjectAlignmentResultRejected); }
}

- (BOOL)selectObjectWithEntityIdentifier:(NSString *)entityIdentifier
                              expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr || expected == nil
        || expected.selectionMode != Core3DSceneElementKindObject
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128) {
        return NO;
    }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) { return NO; }
    const char* entity = entityIdentifier.UTF8String;
    const char* publication = expected.publicationSourceIdentifier.UTF8String;
    if (entity == nullptr || publication == nullptr) { return NO; }
    try {
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entity,
            [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        bool selectionWasTouched = false;
        const bool selected = GLController.viewer->selectObjectFromBrowser(
            identity, static_cast<std::uint32_t>(std::llround(size.width)),
            static_cast<std::uint32_t>(std::llround(size.height)), selectionWasTouched);
        if (selectionWasTouched) {
            [GLController refreshSelectionState];
            [GLController requestRender];
            [self viewDidInvalidateSceneSnapshot];
        }
        return selected;
    } catch (...) { return NO; }
}

- (BOOL)frameCommittedSceneSelectedOnly:(BOOL)selectedObjectsOnly
                                 rect:(CGRect)rect
                       objectIdentity:(const core3d::ObjectFrameIdentity*)identity {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) {
        return NO;
    }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width < 1.0 || size.height < 1.0
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) {
        return NO;
    }
    const auto viewer = GLController.viewer;
    if (viewer == nullptr || !viewer->frameModel(selectedObjectsOnly,
            static_cast<std::uint32_t>(std::llround(size.width)),
            static_cast<std::uint32_t>(std::llround(size.height)),
            rect.origin.x, rect.origin.y, rect.size.width, rect.size.height, identity)) {
        return NO;
    }
    [GLController requestRender];
    [self viewDidInvalidateSceneSnapshot];
    return YES;
}

- (Core3DSceneSnapshot *)captureSceneSnapshot {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) {
        return nil;
    }

    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer = GLController.viewer;
        if (viewer == nullptr) {
            return nil;
        }

        const CGSize drawableSize = GLController.drawableSize;
        if (!std::isfinite(drawableSize.width)
            || !std::isfinite(drawableSize.height)
            || drawableSize.width < 1.0
            || drawableSize.height < 1.0
            || drawableSize.width > std::numeric_limits<std::uint32_t>::max()
            || drawableSize.height > std::numeric_limits<std::uint32_t>::max()) {
            return nil;
        }

        const auto snapshot = viewer->captureSceneSnapshot(
            static_cast<std::uint32_t>(std::llround(drawableSize.width)),
            static_cast<std::uint32_t>(std::llround(drawableSize.height)));
        return snapshot == nullptr
            ? nil
            : Core3DCreateSceneSnapshotDTO(*snapshot);
    } catch (...) {
        return nil;
    }
}

- (Core3DSceneFrameSnapshot *)captureSceneFrameSnapshot {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) {
        return nil;
    }

    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer = GLController.viewer;
        if (viewer == nullptr) {
            return nil;
        }

        const CGSize drawableSize = GLController.drawableSize;
        if (!std::isfinite(drawableSize.width)
            || !std::isfinite(drawableSize.height)
            || drawableSize.width < 1.0
            || drawableSize.height < 1.0
            || drawableSize.width > std::numeric_limits<std::uint32_t>::max()
            || drawableSize.height > std::numeric_limits<std::uint32_t>::max()) {
            return nil;
        }

        const auto frame = viewer->captureSceneFrameSnapshot(
            static_cast<std::uint32_t>(std::llround(drawableSize.width)),
            static_cast<std::uint32_t>(std::llround(drawableSize.height)));
        return !frame.has_value()
            ? nil
            : Core3DCreateSceneFrameSnapshotDTO(*frame);
    } catch (...) {
        return nil;
    }
}

- (BOOL)prepareOrdinaryEditForDocumentClose {
    [self cancelObjectAlignment];
    [self cancelProfileConstruction];
    return [NSThread isMainThread] && _isSetuped && GLController != nil
        && [GLController prepareOrdinaryEditForDocumentClose];
}
- (BOOL)hasUnresolvedOrdinaryEdit {
    return GLController != nil && [GLController hasUnresolvedOrdinaryEdit];
}

- (Core3DScenePresentationOverlaySnapshot *)captureScenePresentationOverlay {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) {
        return nil;
    }

    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer = GLController.viewer;
        if (viewer == nullptr) {
            return nil;
        }
        const auto overlay = viewer->captureScenePresentationOverlay();
        return overlay == nullptr
            ? nil
            : Core3DCreateScenePresentationOverlaySnapshotDTO(*overlay);
    } catch (...) {
        return nil;
    }
}

- (Core3DTransformInspectorSnapshot *)
    requestTransformInspectorSnapshotWithCompletion:
        (Core3DTransformInspectorCompletion)completion {
    if (![NSThread isMainThread]) {
        return Core3DCreateTransformInspectorStateSnapshotDTO(
            Core3DTransformInspectorStateUnavailable);
    }
    if (!_isSetuped) {
        return Core3DCreateTransformInspectorStateSnapshotDTO(
            Core3DTransformInspectorStateNotSetup);
    }
    if (GLController == nil || GLController.viewer == nullptr) {
        return Core3DCreateTransformInspectorStateSnapshotDTO(
            Core3DTransformInspectorStateUnavailable);
    }

    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        if (viewer == nullptr) {
            return Core3DCreateTransformInspectorStateSnapshotDTO(
                Core3DTransformInspectorStateUnavailable);
        }

        __weak typeof(self) weakSelf = self;
        Core3DTransformInspectorCompletion completionCopy =
            [completion copy];
        core3d::TransformInspectorMeasurementCompletion
            nativeCompletion;
        if (completionCopy != nil) {
            nativeCompletion = [weakSelf, completionCopy](
                core3d::TransformInspectorMeasurement measurement) {
                Core3DTransformInspectorSnapshot *snapshot =
                    Core3DCreateTransformInspectorSnapshotDTO(measurement);
                dispatch_block_t deliver = ^{
                    if (weakSelf == nil) {
                        return;
                    }
                    completionCopy(snapshot);
                };
                if ([NSThread isMainThread]) {
                    deliver();
                } else {
                    dispatch_async(dispatch_get_main_queue(), deliver);
                }
            };
        }

        const core3d::TransformInspectorMeasurement measurement =
            viewer->captureTransformInspectorMeasurement(
                std::move(nativeCompletion));
        Core3DTransformInspectorSnapshot *snapshot =
            Core3DCreateTransformInspectorSnapshotDTO(measurement);
        if (measurement.state
                == core3d::TransformInspectorMeasurementState::Measuring
            && snapshot.state
                != Core3DTransformInspectorStateMeasuring) {
            viewer->cancelTransformInspectorMeasurement();
        }
        return snapshot;
    } catch (...) {
        return Core3DCreateTransformInspectorStateSnapshotDTO(
            Core3DTransformInspectorStateUnavailable);
    }
}

- (void)cancelTransformInspectorSnapshotRequest {
    if (![NSThread isMainThread]
        || GLController == nil || GLController.viewer == nullptr) {
        return;
    }
    try {
        GLController.viewer->cancelTransformInspectorMeasurement();
    } catch (...) {
    }
}

- (Core3DTransformInspectorPositionCommitResult)
    commitTransformInspectorPositionValue:(double)value
                                      axis:(Core3DTransformInspectorAxis)axis
                          expectedSnapshot:
                              (Core3DTransformInspectorSnapshot *)snapshot {
    return [self commitTransformInspectorValue:value axis:axis expectedSnapshot:snapshot kind:core3d::TransformInspectorEditKind::Position];
}

- (Core3DTransformInspectorPositionCommitResult)
    commitTransformInspectorRotationValue:(double)value
                                      axis:(Core3DTransformInspectorAxis)axis
                          expectedSnapshot:
                              (Core3DTransformInspectorSnapshot *)snapshot {
    return [self commitTransformInspectorValue:value axis:axis expectedSnapshot:snapshot kind:core3d::TransformInspectorEditKind::Rotation];
}

- (Core3DTransformInspectorPositionCommitResult)
    commitTransformInspectorUniformScaleValue:(double)value
                          expectedSnapshot:(Core3DTransformInspectorSnapshot *)snapshot {
    return [self commitTransformInspectorValue:value axis:Core3DTransformInspectorAxisX
        expectedSnapshot:snapshot kind:core3d::TransformInspectorEditKind::UniformScale];
}

- (Core3DTransformInspectorPositionCommitResult)
    commitTransformInspectorDimensionValue:(double)value
                                      axis:(Core3DTransformInspectorAxis)axis
                          expectedSnapshot:(Core3DTransformInspectorSnapshot *)snapshot {
    return [self commitTransformInspectorValue:value axis:axis expectedSnapshot:snapshot
        kind:core3d::TransformInspectorEditKind::Dimension];
}

- (Core3DTransformInspectorPositionCommitResult)
    commitTransformInspectorValue:(double)value
                              axis:(Core3DTransformInspectorAxis)axis
                  expectedSnapshot:(Core3DTransformInspectorSnapshot *)snapshot
                          kind:(core3d::TransformInspectorEditKind)kind {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr) {
        return Core3DTransformInspectorPositionCommitResultUnavailable;
    }
    if (snapshot == nil || !snapshot.canEditPosition
        || !snapshot.presentationMatchesDocument
        || snapshot.positionEditGeneration == 0
        || snapshot.documentEditGeneration == 0
        || snapshot.geometryEditGeneration == 0
        || snapshot.entityIdentifier.length == 0
        || snapshot.definitionIdentifier.length == 0
        || snapshot.entityIdentifier.UTF8String == nullptr
        || snapshot.definitionIdentifier.UTF8String == nullptr) {
        return Core3DTransformInspectorPositionCommitResultUnsupported;
    }

    core3d::TransformInspectorPositionAxis nativeAxis;
    switch (axis) {
        case Core3DTransformInspectorAxisX:
            nativeAxis = core3d::TransformInspectorPositionAxis::X;
            break;
        case Core3DTransformInspectorAxisY:
            nativeAxis = core3d::TransformInspectorPositionAxis::Y;
            break;
        case Core3DTransformInspectorAxisZ:
            nativeAxis = core3d::TransformInspectorPositionAxis::Z;
            break;
        default:
            return Core3DTransformInspectorPositionCommitResultInvalidValue;
    }

    core3d::TransformInspectorGeometryRepresentation nativeRepresentation;
    switch (snapshot.representation) {
        case Core3DTransformInspectorRepresentationBRep:
            nativeRepresentation =
                core3d::TransformInspectorGeometryRepresentation::BRep;
            break;
        case Core3DTransformInspectorRepresentationTriangleMesh:
            nativeRepresentation =
                core3d::TransformInspectorGeometryRepresentation::
                    TriangleMesh;
            break;
        case Core3DTransformInspectorRepresentationUnknown:
        default:
            return Core3DTransformInspectorPositionCommitResultUnsupported;
    }

    core3d::TransformInspectorPositionCommitResult nativeResult =
        core3d::TransformInspectorPositionCommitResult::InternalFailure;
    try {
        // std::string construction may allocate. Keep request assembly inside
        // the exception boundary so this Objective-C entry point never lets a
        // low-memory C++ exception cross into Swift.
        core3d::TransformInspectorPositionCommitRequest request;
        request.positionEditGeneration = snapshot.positionEditGeneration;
        request.documentEditGeneration = snapshot.documentEditGeneration;
        request.geometryEditGeneration = snapshot.geometryEditGeneration;
        request.entityIdentifier = snapshot.entityIdentifier.UTF8String;
        request.definitionIdentifier =
            snapshot.definitionIdentifier.UTF8String;
        request.expectedPosition = {
            snapshot.position.x,
            snapshot.position.y,
            snapshot.position.z,
        };
        request.expectedMetersPerUnit = snapshot.metersPerUnit;
        request.expectedRepresentation = nativeRepresentation;
        request.expectedModelCapabilities =
            static_cast<std::uint64_t>(snapshot.modelCapabilities);
        request.axis = nativeAxis;
        request.value = value;
        request.kind = kind;
        nativeResult = GLController.viewer
            ->commitTransformInspectorPosition(request);
    } catch (...) {
        nativeResult =
            core3d::TransformInspectorPositionCommitResult::InternalFailure;
    }

    Core3DTransformInspectorPositionCommitResult result =
        Core3DTransformInspectorPositionCommitResultInternalFailure;
    switch (nativeResult) {
        case core3d::TransformInspectorPositionCommitResult::Committed:
            result = Core3DTransformInspectorPositionCommitResultCommitted;
            break;
        case core3d::TransformInspectorPositionCommitResult::Unchanged:
            result = Core3DTransformInspectorPositionCommitResultUnchanged;
            break;
        case core3d::TransformInspectorPositionCommitResult::InvalidValue:
            result = Core3DTransformInspectorPositionCommitResultInvalidValue;
            break;
        case core3d::TransformInspectorPositionCommitResult::Busy:
            result = Core3DTransformInspectorPositionCommitResultBusy;
            break;
        case core3d::TransformInspectorPositionCommitResult::Stale:
            result = Core3DTransformInspectorPositionCommitResultStale;
            break;
        case core3d::TransformInspectorPositionCommitResult::Unsupported:
            result = Core3DTransformInspectorPositionCommitResultUnsupported;
            break;
        case core3d::TransformInspectorPositionCommitResult::Unavailable:
            result = Core3DTransformInspectorPositionCommitResultUnavailable;
            break;
        case core3d::TransformInspectorPositionCommitResult::InternalFailure:
            result = Core3DTransformInspectorPositionCommitResultInternalFailure;
            break;
    }

    if (result == Core3DTransformInspectorPositionCommitResultCommitted) {
        // NotifyChanges already schedules the OpenGL frame and its one scene
        // invalidation callback. redrawDocument() may have recreated the
        // interactors without uniquely restoring the selected owner; publish
        // capabilities only after the recreated-gizmo callback and an exact
        // synchronous selection refresh.
        [GLController refreshSelectionState];
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingSelection
                                 | UIStateChangingGizmo
                                 | UIStateChangingDelete
                                 | UIStateChangingDuplicate
                                 | UIStateChangingApply
                                 | UIStateChangingApplyMaterial
                                 | UIStateChangingHistory];
    }
    return result;
}

- (NSDictionary<NSString *, NSNumber *> *)
    transformInspectorPerformanceState {
    if (![NSThread isMainThread] || !_isSetuped
        || GLController == nil || GLController.viewer == nullptr) {
        return @{};
    }
    try {
        const core3d::TransformInspectorMeasurementPerformanceState state =
            GLController.viewer
                ->transformInspectorMeasurementPerformanceState();
        return @{
            @"lastMeshSweepMilliseconds": @(
                static_cast<double>(state.lastMeshSweepMilliseconds)),
            @"lastBRepCopyMilliseconds": @(
                static_cast<double>(state.lastBRepCopyMilliseconds)),
            @"meshSweepRunCount": @(
                static_cast<unsigned long long>(state.meshSweepRunCount)),
            @"watchdogFireCount": @(
                static_cast<unsigned long long>(state.watchdogFireCount)),
            @"cacheEntryCount": @(
                static_cast<unsigned long long>(state.cacheEntryCount)),
            @"negativeCacheEntryCount": @(
                static_cast<unsigned long long>(
                    state.negativeCacheEntryCount)),
        };
    } catch (...) {
        return @{};
    }
}

#ifdef DEBUG
- (NSDictionary<NSString *, NSNumber *> *)
    debugTransformInspectorMeasurementState {
    if (![NSThread isMainThread] || !_isSetuped
        || GLController == nil || GLController.viewer == nullptr) {
        return @{};
    }
    try {
        const core3d::TransformInspectorMeasurementDebugState state =
            GLController.viewer->DebugTransformInspectorMeasurementState();
        return @{
            @"generation": @(
                static_cast<unsigned long long>(state.generation)),
            @"submittedCount": @(
                static_cast<unsigned long long>(state.submittedCount)),
            @"startedCount": @(
                static_cast<unsigned long long>(state.startedCount)),
            @"completedCount": @(
                static_cast<unsigned long long>(state.completedCount)),
            @"cancelledOrSupersededCount": @(
                static_cast<unsigned long long>(
                    state.cancelledOrSupersededCount)),
            @"pendingReplacementCount": @(
                static_cast<unsigned long long>(
                    state.pendingReplacementCount)),
            @"acceptedCount": @(
                static_cast<unsigned long long>(state.acceptedCount)),
            @"staleCount": @(
                static_cast<unsigned long long>(state.staleCount)),
            @"workerActive": @(state.workerActive == Standard_True),
            @"workerPending": @(state.workerPending == Standard_True),
            @"lastComputeWasMainThread": @(
                state.lastComputeWasMainThread == Standard_True),
            @"workerBlocked": @(state.workerBlocked == Standard_True),
            @"forcedBoundsFailure": @(
                state.forcedBoundsFailure == Standard_True),
            @"lastMeshSweepMilliseconds": @(
                static_cast<double>(state.lastMeshSweepMilliseconds)),
            @"lastBRepCopyMilliseconds": @(
                static_cast<double>(state.lastBRepCopyMilliseconds)),
            @"maximumBRepTopologyNodes": @(
                static_cast<unsigned long long>(
                    state.maximumBRepTopologyNodes)),
            @"maximumTriangleMeshSweepNodes": @(
                static_cast<unsigned long long>(
                    state.maximumTriangleMeshSweepNodes)),
            @"meshSweepWatchdogDeadlineMilliseconds": @(
                static_cast<double>(
                    state.meshSweepWatchdogDeadlineMilliseconds)),
            @"meshSweepWatchdogPollNodes": @(
                static_cast<unsigned long long>(
                    state.meshSweepWatchdogPollNodes)),
            @"meshSweepRunCount": @(
                static_cast<unsigned long long>(state.meshSweepRunCount)),
            @"watchdogFireCount": @(
                static_cast<unsigned long long>(state.watchdogFireCount)),
            @"cacheEntryCount": @(
                static_cast<unsigned long long>(state.cacheEntryCount)),
            @"negativeCacheEntryCount": @(
                static_cast<unsigned long long>(
                    state.negativeCacheEntryCount)),
        };
    } catch (...) {
        return @{};
    }
}

- (void)debugSetTransformInspectorWorkerBlocked:(BOOL)blocked {
    if (![NSThread isMainThread]
        || GLController == nil || GLController.viewer == nullptr) {
        return;
    }
    GLController.viewer->DebugSetTransformInspectorWorkerBlocked(blocked);
}

- (void)debugSetTransformInspectorForcedMeasurementFailure:(BOOL)failure {
    if (![NSThread isMainThread]
        || GLController == nil || GLController.viewer == nullptr) {
        return;
    }
    GLController.viewer->DebugSetTransformInspectorForcedMeasurementFailure(
        failure);
}

- (void)debugSetMaximumTransformInspectorBRepTopologyNodes:
    (NSUInteger)limit {
    if (![NSThread isMainThread]
        || GLController == nil || GLController.viewer == nullptr
        || limit > std::numeric_limits<Standard_Size>::max()) {
        return;
    }
    GLController.viewer->DebugSetMaximumTransformInspectorBRepTopologyNodes(
        static_cast<Standard_Size>(limit));
}

- (void)debugSetMaximumTransformInspectorTriangleMeshSweepNodes:
    (NSUInteger)limit {
    if (![NSThread isMainThread]
        || GLController == nil || GLController.viewer == nullptr
        || limit > std::numeric_limits<Standard_Size>::max()) {
        return;
    }
    GLController.viewer
        ->DebugSetMaximumTransformInspectorTriangleMeshSweepNodes(
            static_cast<Standard_Size>(limit));
}

- (void)debugSetTransformInspectorMeshSweepWatchdogDeadlineMilliseconds:
    (double)deadlineMilliseconds
    pollNodes:(NSUInteger)pollNodes {
    if (![NSThread isMainThread]
        || GLController == nil || GLController.viewer == nullptr
        || pollNodes > std::numeric_limits<Standard_Size>::max()) {
        return;
    }
    GLController.viewer->DebugSetTransformInspectorMeshSweepWatchdog(
        static_cast<Standard_Real>(deadlineMilliseconds),
        static_cast<Standard_Size>(pollNodes));
}

- (void)debugSetTransformInspectorPositionCommitMode:(NSInteger)mode {
    if (![NSThread isMainThread]
        || GLController == nil || GLController.viewer == nullptr
        || mode < 0 || mode > 3) {
        return;
    }
    GLController.viewer->DebugSetTransformInspectorPositionCommitMode(
        static_cast<Standard_Integer>(mode));
}

- (void)debugSetTransformInspectorPositionPublicationFallbackMode:
    (NSInteger)mode {
    if (![NSThread isMainThread]
        || GLController == nil || GLController.viewer == nullptr
        || mode < 0 || mode > 3) {
        return;
    }
    GLController.viewer
        ->DebugSetTransformInspectorPositionPublicationFallbackMode(
            static_cast<Standard_Integer>(mode));
}

- (BOOL)debugTryMirrorAxis:(NSInteger)axis backward:(BOOL)backward {
    if (![NSThread isMainThread]
        || !_isSetuped
        || _currentGizmoType != PrimitiveGizmoTypeMirror
        || axis < 0 || axis > 2) {
        return NO;
    }

    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        if (viewer == nullptr
            || viewer->getObjectInteractor() == nullptr) {
            return NO;
        }
        const BOOL didCreate = viewer->getObjectInteractor()->tryMirror(
            static_cast<Standard_Integer>(axis),
            backward);
        [GLController requestRender];
        // Reuse the same finalized lifecycle path as an authoritative touch
        // release so Apply state and alternate-renderer capture stay aligned.
        [self viewDidEndPrimaryInteractionCancelled:NO];
        return didCreate;
    } catch (...) {
        return NO;
    }
}

- (BOOL)debugTryMirrorPlaneWithEntityIdentifier:(NSString *)entityIdentifier
                              faceTopologyIndex:(NSInteger)faceTopologyIndex
                                         offset:(CGFloat)offset {
	if (![NSThread isMainThread] || !_isSetuped
		|| _currentGizmoType != PrimitiveGizmoTypeMirror
		|| entityIdentifier.length == 0 || faceTopologyIndex < 0) {
		return NO;
	}
	const BOOL didCreate =
		[GLController debugTryMirrorPlaneWithEntityIdentifier:entityIdentifier
		                                      faceTopologyIndex:faceTopologyIndex
		                                                 offset:offset];
	[self viewDidEndPrimaryInteractionCancelled:NO];
	return didCreate;
}

- (NSDictionary<NSString *, NSNumber *> *)debugLinearArrayState {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) {
        return @{};
    }
    return [GLController debugLinearArrayState];
}

- (void)debugSetLinearArrayTransactionFailureCount:(NSUInteger)count {
    [GLController debugSetLinearArrayTransactionFailureCount:count];
}

- (void)debugSetLinearArrayAbortFailureCount:(NSUInteger)count {
    [GLController debugSetLinearArrayAbortFailureCount:count];
}

- (void)debugSetLinearArrayEraseFailureCount:(NSUInteger)count {
    [GLController debugSetLinearArrayEraseFailureCount:count];
}

- (void)debugSetLinearArrayCommitMode:(NSInteger)mode {
    [GLController debugSetLinearArrayCommitMode:mode];
}

- (void)debugSetLinearArrayPostCommitInspectFailureCount:(NSUInteger)count {
    [GLController debugSetLinearArrayPostCommitInspectFailureCount:count];
}

- (void)debugSetMaximumLinearArrayTopologyNodes:(NSUInteger)limit {
    [GLController debugSetMaximumLinearArrayTopologyNodes:limit];
}

- (BOOL)debugMutateFirstLinearArraySourcePersistedTransform {
    return [GLController debugMutateFirstLinearArraySourcePersistedTransform];
}

- (NSDictionary<NSString *, NSNumber *> *)debugRadialArrayState {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) {
        return @{};
    }
    return [GLController debugRadialArrayState];
}

- (void)debugSetRadialArrayBeginOwnedCommandMismatchCount:(NSUInteger)count {
    [GLController debugSetRadialArrayBeginOwnedCommandMismatchCount:count];
}

- (void)debugSetRadialArrayTransactionFailureCount:(NSUInteger)count {
    [GLController debugSetRadialArrayTransactionFailureCount:count];
}

- (void)debugSetRadialArrayAbortFailureCount:(NSUInteger)count {
    [GLController debugSetRadialArrayAbortFailureCount:count];
}

- (void)debugSetRadialArrayEraseFailureCount:(NSUInteger)count {
    [GLController debugSetRadialArrayEraseFailureCount:count];
}

- (void)debugSetRadialArrayApplyCommitMode:(NSInteger)mode {
    [GLController debugSetRadialArrayApplyCommitMode:mode];
}

- (void)debugSetRadialArrayPostCommitInspectMode:(NSInteger)mode {
    [GLController debugSetRadialArrayPostCommitInspectMode:mode];
}

- (void)debugSetMaximumRadialArrayTopologyNodes:(NSUInteger)limit {
    [GLController debugSetMaximumRadialArrayTopologyNodes:limit];
}

- (BOOL)debugMutateRadialArraySourcePersistedTransform {
    return [GLController debugMutateRadialArraySourcePersistedTransform];
}

- (void)debugSetRadialArrayReferenceEditCommitMode:(NSInteger)mode {
    [GLController debugSetRadialArrayReferenceEditCommitMode:mode];
}

- (NSDictionary<NSString *, NSNumber *> *)debugGeometryCopyIndependenceState {
    Standard_Size definitionCount = 0;
    Standard_Size triangleMeshDefinitionCount = 0;
    Standard_Size triangleMeshHandleCount = 0;
    Standard_Size uniqueTriangleMeshHandleCount = 0;
    Standard_Boolean pairwiseNonPartner = Standard_True;
    Standard_Boolean pairwiseDistinctTriangleMeshHandles = Standard_True;
    Standard_Boolean valid = Standard_False;
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr) {
        return @{
            @"definitionCount": @0,
            @"pairwiseNonPartner": @NO,
            @"triangleMeshDefinitionCount": @0,
            @"triangleMeshHandleCount": @0,
            @"uniqueTriangleMeshHandleCount": @0,
            @"pairwiseDistinctTriangleMeshHandles": @NO,
            @"valid": @NO,
        };
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(OcctDocument) occtDocument =
            GLController.viewer->getDocument();
        const Handle(TDocStd_Document) document = occtDocument.IsNull()
            ? Handle(TDocStd_Document)() : occtDocument->Document();
        if (document.IsNull() || document->HasOpenCommand()
            || !XCAFDoc_DocumentTool::CheckShapeTool(document->Main())) {
            throw Standard_Failure("Copy identity document is unavailable");
        }
        const Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapeTool.IsNull()) {
            throw Standard_Failure("Copy identity shape tool is unavailable");
        }
        TDF_LabelSequence freeShapes;
        shapeTool->GetFreeShapes(freeShapes);
        std::vector<TopoDS_Shape> definitions;
        std::unordered_set<const Poly_Triangulation*>
            uniqueTriangleMeshHandles;
        for (Standard_Integer index = 1;
             index <= freeShapes.Length(); ++index) {
            const TDF_Label& label = freeShapes.Value(index);
            if (label.IsNull() || !shapeTool->IsShape(label)
                || !XCAFDoc_ShapeTool::IsFree(label)
                || !XCAFDoc_ShapeTool::IsSimpleShape(label)
                || XCAFDoc_ShapeTool::IsReference(label)
                || XCAFDoc_ShapeTool::IsComponent(label)
                || XCAFDoc_ShapeTool::IsAssembly(label)
                || XCAFDoc_ShapeTool::IsSubShape(label)) {
                continue;
            }
            const OcctGeometryRepresentation representation =
                occtDocument->GeometryRepresentationForLabel(label);
            const TopoDS_Shape shape =
                XCAFDoc_ShapeTool::GetShape(label);
            if (representation == OcctGeometryRepresentation::Invalid
                || shape.IsNull()) {
                throw Standard_Failure(
                    "Copy identity definition is malformed");
            }
            for (const TopoDS_Shape& existing : definitions) {
                if (shape.IsPartner(existing)) {
                    pairwiseNonPartner = Standard_False;
                }
            }
            definitions.push_back(shape);
            ++definitionCount;

            if (representation
                != OcctGeometryRepresentation::TriangleMesh) {
                continue;
            }
            ++triangleMeshDefinitionCount;
            Standard_Size faceCount = 0;
            for (TopExp_Explorer faces(shape, TopAbs_FACE);
                 faces.More(); faces.Next()) {
                const TopoDS_Face face = TopoDS::Face(faces.Current());
                TopLoc_Location location;
                const Poly_ListOfTriangulation& triangulations =
                    BRep_Tool::Triangulations(face, location);
                const Handle(Poly_Triangulation)& triangulation =
                    BRep_Tool::Triangulation(face, location);
                if (triangulations.Size() != 1
                    || triangulation.IsNull()) {
                    throw Standard_Failure(
                        "Copy identity TriangleMesh face is malformed");
                }
                ++faceCount;
                ++triangleMeshHandleCount;
                if (!uniqueTriangleMeshHandles.insert(
                        triangulation.get()).second) {
                    pairwiseDistinctTriangleMeshHandles = Standard_False;
                }
            }
            if (faceCount == 0) {
                throw Standard_Failure(
                    "Copy identity TriangleMesh has no faces");
            }
        }
        uniqueTriangleMeshHandleCount =
            uniqueTriangleMeshHandles.size();
        if (uniqueTriangleMeshHandleCount
            != triangleMeshHandleCount) {
            pairwiseDistinctTriangleMeshHandles = Standard_False;
        }
        valid = Standard_True;
    } catch (...) {
        valid = Standard_False;
    }
    return @{
        @"definitionCount": @(definitionCount),
        @"pairwiseNonPartner": @(pairwiseNonPartner),
        @"triangleMeshDefinitionCount": @(triangleMeshDefinitionCount),
        @"triangleMeshHandleCount": @(triangleMeshHandleCount),
        @"uniqueTriangleMeshHandleCount": @(uniqueTriangleMeshHandleCount),
        @"pairwiseDistinctTriangleMeshHandles":
            @(pairwiseDistinctTriangleMeshHandles),
        @"valid": @(valid),
    };
}

- (NSDictionary<NSString *, NSNumber *> *)debugMirrorState {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) {
        return @{};
    }
    return [GLController debugMirrorState];
}

- (void)debugSetMirrorTransactionFailureCount:(NSUInteger)count {
    [GLController debugSetMirrorTransactionFailureCount:count];
}

- (void)debugSetMirrorAbortFailureCount:(NSUInteger)count {
    [GLController debugSetMirrorAbortFailureCount:count];
}

- (void)debugSetMirrorEraseFailureCount:(NSUInteger)count {
    [GLController debugSetMirrorEraseFailureCount:count];
}

- (void)debugSetMirrorReferenceEraseFailureCount:(NSUInteger)count {
	[GLController debugSetMirrorReferenceEraseFailureCount:count];
}

- (void)debugSetMirrorCommitMode:(NSInteger)mode {
    [GLController debugSetMirrorCommitMode:mode];
}

- (void)debugSetMirrorPostCommitInspectFailureCount:(NSUInteger)count {
    [GLController debugSetMirrorPostCommitInspectFailureCount:count];
}

- (void)debugSetMaximumMirrorTopologyNodes:(NSUInteger)limit {
    [GLController debugSetMaximumMirrorTopologyNodes:limit];
}

- (void)debugSetMaximumMirrorReferenceTopologyNodes:(NSUInteger)limit {
	[GLController debugSetMaximumMirrorReferenceTopologyNodes:limit];
}

- (void)debugSetMaximumMirrorReferenceFaces:(NSUInteger)limit {
	[GLController debugSetMaximumMirrorReferenceFaces:limit];
}

- (BOOL)debugMutateFirstMirrorSourcePersistedTransform {
    return [GLController debugMutateFirstMirrorSourcePersistedTransform];
}

- (BOOL)debugBeginBooleanWithGizmoType:(PrimitiveGizmoType)gizmoType
                actorEntityIdentifiers:(NSArray<NSString *> *)actorEntityIdentifiers
              subjectEntityIdentifiers:(NSArray<NSString *> *)subjectEntityIdentifiers {
    core3d::BooleanAction action =
        core3d::BooleanAction::BooleanSubtract;
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || !Core3DTryBooleanActionForGizmo(gizmoType, action)
        || actorEntityIdentifiers == nil
        || subjectEntityIdentifiers == nil) {
        return NO;
    }
    try {
        std::vector<std::string> actors;
        std::vector<std::string> subjects;
        actors.reserve(actorEntityIdentifiers.count);
        subjects.reserve(subjectEntityIdentifiers.count);
        for (NSString *identifier in actorEntityIdentifiers) {
            if (![identifier isKindOfClass:NSString.class]
                || identifier.length == 0 || identifier.UTF8String == nullptr) {
                return NO;
            }
            actors.emplace_back(identifier.UTF8String);
        }
        for (NSString *identifier in subjectEntityIdentifiers) {
            if (![identifier isKindOfClass:NSString.class]
                || identifier.length == 0 || identifier.UTF8String == nullptr) {
                return NO;
            }
            subjects.emplace_back(identifier.UTF8String);
        }

        [self setGizmoType:gizmoType];
        if (_currentGizmoType != gizmoType
            || [GLController getGizmoType] != gizmoType) {
            self.can_apply = NO;
            [GLController debugRequestRender];
            [self viewDidChangeViewportPresentationState];
            [self sendNotifyUIState:UIStateChangingApply];
            return NO;
        }
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        const BOOL didBegin = viewer != nullptr
            && viewer->debugBeginBooleanSelection(
                action,
                actors,
                subjects);
        self.can_apply = didBegin && [GLController canApplyBoolean];
        [GLController debugRequestRender];
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply];
        return didBegin;
    } catch (...) {
        self.can_apply = NO;
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply];
        return NO;
    }
}

- (BOOL)debugRecomputeBooleanPreview {
    core3d::BooleanAction action =
        core3d::BooleanAction::BooleanSubtract;
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || !Core3DTryBooleanActionForGizmo(
            _currentGizmoType, action)) {
        return NO;
    }
    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        if (viewer == nullptr) {
            return NO;
        }
        const std::shared_ptr<core3d::ObjectInteractor> interactor =
            viewer->getObjectInteractor();
        if (interactor == nullptr) {
            return NO;
        }
        const BOOL didRecompute =
            interactor->debugRecomputeBooleanPreview(action);
        self.can_apply = didRecompute && [GLController canApplyBoolean];
        [GLController debugRequestRender];
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply];
        return didRecompute;
    } catch (...) {
        self.can_apply = NO;
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply];
        return NO;
    }
}

- (NSDictionary<NSString *, NSNumber *> *)debugBooleanPreviewState {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) {
        return @{};
    }
    return [GLController debugBooleanPreviewState];
}

+ (NSDictionary<NSString *, NSNumber *> *)
    debugBooleanResultDimensionValidation {
    try {
        const TopoDS_Shape solid =
            BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

        BRep_Builder builder;
        TopoDS_Compound pureCompound;
        builder.MakeCompound(pureCompound);
        builder.Add(pureCompound, solid);
        builder.Add(
            pureCompound,
            BRepPrimAPI_MakeBox(
                gp_Pnt(20.0, 0.0, 0.0), 10.0, 10.0, 10.0)
                .Shape());

        TopoDS_CompSolid pureCompSolid;
        builder.MakeCompSolid(pureCompSolid);
        builder.Add(pureCompSolid, solid);

        TopoDS_Compound mixedCompound;
        builder.MakeCompound(mixedCompound);
        builder.Add(mixedCompound, solid);
        const TopoDS_Shape separateSolid =
            BRepPrimAPI_MakeBox(
                gp_Pnt(40.0, 0.0, 0.0), 10.0, 10.0, 10.0)
                .Shape();
        TopExp_Explorer aFace(separateSolid, TopAbs_FACE);
        if (!aFace.More()) {
            return @{};
        }
        builder.Add(mixedCompound, aFace.Current());

        return @{
            @"solidAccepted": @(
                core3d::BooleanOperationController::
                    debugValidateSolidResult(solid)),
            @"pureCompoundAccepted": @(
                core3d::BooleanOperationController::
                    debugValidateSolidResult(pureCompound)),
            @"pureCompSolidAccepted": @(
                core3d::BooleanOperationController::
                    debugValidateSolidResult(pureCompSolid)),
            @"mixedCompoundRejected": @(
                !core3d::BooleanOperationController::
                    debugValidateSolidResult(mixedCompound)),
        };
    } catch (...) {
        return @{};
    }
}

- (void)debugSetBooleanPreviewWorkerBlocked:(BOOL)blocked {
    [GLController debugSetBooleanPreviewWorkerBlocked:blocked];
}

- (void)debugSetMaximumBooleanCaptureTopologyNodes:(NSUInteger)limit {
    [GLController debugSetMaximumBooleanCaptureTopologyNodes:limit];
}

- (void)debugSetMaximumBooleanResultTopologyNodes:(NSUInteger)limit {
    [GLController debugSetMaximumBooleanResultTopologyNodes:limit];
}

- (void)debugSetMaximumBooleanResultSolids:(NSUInteger)limit {
    [GLController debugSetMaximumBooleanResultSolids:limit];
}

- (void)debugSetBooleanTransactionFailureCount:(NSUInteger)count {
    [GLController debugSetBooleanTransactionFailureCount:count];
}

- (void)debugSetBooleanAbortFailureCount:(NSUInteger)count {
    [GLController debugSetBooleanAbortFailureCount:count];
}

- (void)debugSetBooleanPostCommitInspectFailureCount:(NSUInteger)count {
    [GLController debugSetBooleanPostCommitInspectFailureCount:count];
}

- (void)debugSimulateBooleanMemoryWarning {
    [GLController didReceiveMemoryWarning];
}

- (void)debugSimulateLinearArrayMemoryWarning {
    [GLController didReceiveMemoryWarning];
}

- (void)debugSimulateShellMemoryWarning {
    [GLController didReceiveMemoryWarning];
}

- (BOOL)debugBeginExtrusionWithEntityIdentifier:(NSString *)entityIdentifier
                              faceTopologyIndex:(NSUInteger)faceTopologyIndex {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0
        || entityIdentifier.UTF8String == nullptr) {
        return NO;
	}
	try {
		const std::shared_ptr<core3d::Core3DViewer> viewer =
			GLController.viewer;
		const BOOL didBegin = viewer != nullptr
            && viewer->debugBeginExtrusionSelection(
				std::string(entityIdentifier.UTF8String),
				static_cast<Standard_Size>(faceTopologyIndex));
		if (didBegin && viewer->getObjectInteractor() != nullptr) {
			viewer->getObjectInteractor()->setManipulatorType(
				core3d::PrimitiveManipulatorType::PrimitiveGizmoTypeExtrude);
		}
		if (didBegin) {
			_availableGizmoTypes = @[
				@(PrimitiveGizmoTypeChamfer),
				@(PrimitiveGizmoTypeExtrude)
			];
		}
		_currentGizmoType = [GLController getGizmoType];
        self.can_apply = NO;
        [GLController debugRequestRender];
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:(UIStateChangingGizmo
            | UIStateChangingSelection
            | UIStateChangingApply)];
        return didBegin;
    } catch (...) {
        self.can_apply = NO;
        [self sendNotifyUIState:(UIStateChangingGizmo
            | UIStateChangingSelection
            | UIStateChangingApply)];
        return NO;
    }
}

- (NSDictionary<NSString *, NSNumber *> *)debugExtrusionState {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) {
        return @{};
    }
    return [GLController debugExtrusionState];
}

- (void)debugSetExtrusionCommitMode:(NSInteger)mode {
    [GLController debugSetExtrusionCommitMode:mode];
}

- (void)debugSetExtrusionAbortFailureCount:(NSUInteger)count {
    [GLController debugSetExtrusionAbortFailureCount:count];
}

- (void)debugSetExtrusionPostCommitInspectFailureCount:(NSUInteger)count {
    [GLController debugSetExtrusionPostCommitInspectFailureCount:count];
}

- (BOOL)debugBeginShellWithEntityIdentifier:(NSString *)entityIdentifier
                          faceTopologyIndex:(NSUInteger)faceTopologyIndex {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0
        || entityIdentifier.UTF8String == nullptr) {
        return NO;
    }
    try {
        const BOOL didBegin =
            [GLController debugBeginShellWithEntityIdentifier:
                entityIdentifier
                faceTopologyIndex:faceTopologyIndex];
        _currentGizmoType = [GLController getGizmoType];
        if (didBegin) {
            _availableGizmoTypes = @[
                @(PrimitiveGizmoTypeChamfer),
                @(PrimitiveGizmoTypeExtrude),
                @(PrimitiveGizmoTypeShell)
            ];
        }
        self.can_apply = didBegin && [GLController canApplyShell];
        [GLController debugRequestRender];
        [self viewDidChangeViewportPresentationOverlay];
        [self sendNotifyUIState:(UIStateChangingGizmo
            | UIStateChangingSelection
            | UIStateChangingApply)];
        return didBegin;
    } catch (...) {
        self.can_apply = NO;
        [self viewDidChangeViewportPresentationOverlay];
        [self sendNotifyUIState:(UIStateChangingGizmo
            | UIStateChangingSelection
            | UIStateChangingApply)];
        return NO;
    }
}

- (NSDictionary<NSString *, NSNumber *> *)debugShellState {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) {
        return @{};
    }
    return [GLController debugShellState];
}

- (void)debugSetShellPreviewWorkerBlocked:(BOOL)blocked {
    [GLController debugSetShellPreviewWorkerBlocked:blocked];
}

- (void)debugSetMaximumShellCaptureTopologyNodes:(NSUInteger)limit {
    [GLController debugSetMaximumShellCaptureTopologyNodes:limit];
}

- (void)debugSetMaximumShellResultTopologyNodes:(NSUInteger)limit {
    [GLController debugSetMaximumShellResultTopologyNodes:limit];
}

- (void)debugSetShellTransactionFailureCount:(NSUInteger)count {
    [GLController debugSetShellTransactionFailureCount:count];
}

- (void)debugSetShellAbortFailureCount:(NSUInteger)count {
    [GLController debugSetShellAbortFailureCount:count];
}

- (void)debugSetShellPreviewEraseFailureCount:(NSUInteger)count {
    [GLController debugSetShellPreviewEraseFailureCount:count];
}

- (void)debugSetShellCommitMode:(NSInteger)mode {
    [GLController debugSetShellCommitMode:mode];
}

- (void)debugSetShellPostCommitInspectFailureCount:(NSUInteger)count {
    [GLController debugSetShellPostCommitInspectFailureCount:count];
}

- (BOOL)debugMutateShellSourcePersistedTransform {
    return [GLController debugMutateShellSourcePersistedTransform];
}

- (BOOL)debugMutateShellSourcePersistedShape {
    return [GLController debugMutateShellSourcePersistedShape];
}

- (BOOL)debugBeginBevelWithEntityIdentifier:(NSString *)entityIdentifier
                       edgeTopologyIndices:(NSArray<NSNumber *> *)edgeTopologyIndices {
    if (![entityIdentifier isKindOfClass:NSString.class]
        || ![edgeTopologyIndices isKindOfClass:NSArray.class]) {
        return NO;
    }
    return [self debugBeginBevelWithEntityIdentifiers:@[entityIdentifier]
                          edgeTopologyIndicesByEntity:@[edgeTopologyIndices]];
}

- (BOOL)debugBeginBevelWithEntityIdentifiers:(NSArray<NSString *> *)entityIdentifiers
                 edgeTopologyIndicesByEntity:(NSArray<NSArray<NSNumber *> *> *)edgeTopologyIndicesByEntity {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || ![entityIdentifiers isKindOfClass:NSArray.class]
        || ![edgeTopologyIndicesByEntity isKindOfClass:NSArray.class]
        || entityIdentifiers.count == 0
        || entityIdentifiers.count != edgeTopologyIndicesByEntity.count
        || entityIdentifiers.count
            > core3d::BevelOperationController::kMaxSourceBodies) {
        return NO;
    }
    try {
        std::vector<std::string> identifiers;
        std::vector<std::vector<Standard_Size>> allIndices;
        identifiers.reserve(entityIdentifiers.count);
        allIndices.reserve(entityIdentifiers.count);
        std::unordered_set<std::string> uniqueIdentifiers;
        Standard_Size aggregateEdgeCount = 0;
        for (NSUInteger sourceIndex = 0;
             sourceIndex < entityIdentifiers.count; ++sourceIndex) {
            id identifierValue = entityIdentifiers[sourceIndex];
            id indicesValue = edgeTopologyIndicesByEntity[sourceIndex];
            if (![identifierValue isKindOfClass:NSString.class]
                || ![indicesValue isKindOfClass:NSArray.class]) {
                return NO;
            }
            NSString* identifier = static_cast<NSString*>(identifierValue);
            NSArray* sourceIndices = static_cast<NSArray*>(indicesValue);
            if (identifier.length == 0 || identifier.UTF8String == nullptr
                || sourceIndices.count == 0
                || sourceIndices.count
                    > core3d::BevelOperationController::kMaxSelectedEdges
                || aggregateEdgeCount
                    > core3d::BevelOperationController::kMaxSelectedEdges
                        - sourceIndices.count) {
                return NO;
            }
            const std::string identifierString(identifier.UTF8String);
            if (!uniqueIdentifiers.insert(identifierString).second) {
                return NO;
            }
            aggregateEdgeCount += sourceIndices.count;
            std::vector<Standard_Size> indices;
            indices.reserve(sourceIndices.count);
            std::unordered_set<Standard_Size> uniqueIndices;
            for (id value in sourceIndices) {
                if (![value isKindOfClass:NSNumber.class]
                    || CFGetTypeID((__bridge CFTypeRef)value)
                        == CFBooleanGetTypeID()) {
                    return NO;
                }
                NSNumber* number = static_cast<NSNumber*>(value);
                const double raw = number.doubleValue;
                if (!std::isfinite(raw) || raw < 0.0
                    || std::floor(raw) != raw
                    || raw > static_cast<double>(
                        std::numeric_limits<Standard_Size>::max())) {
                    return NO;
                }
                const Standard_Size index =
                    static_cast<Standard_Size>(raw);
                if (!uniqueIndices.insert(index).second) {
                    return NO;
                }
                indices.push_back(index);
            }
            identifiers.push_back(identifierString);
            allIndices.push_back(std::move(indices));
        }
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        const BOOL didBegin = viewer != nullptr
            && viewer->debugBeginBevelSelection(
                identifiers, allIndices);
        _currentGizmoType = [GLController getGizmoType];
        self.can_apply = NO;
        [GLController debugRequestRender];
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingGizmo
                                 | UIStateChangingApply];
        return didBegin;
    } catch (...) {
        self.can_apply = NO;
        [self sendNotifyUIState:UIStateChangingApply];
        return NO;
    }
}

- (NSDictionary<NSString *, NSNumber *> *)debugBevelState {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) {
        return @{};
    }
    return [GLController debugBevelState];
}

- (void)debugSetBevelPreviewWorkerBlocked:(BOOL)blocked {
    [GLController debugSetBevelPreviewWorkerBlocked:blocked];
}

- (void)debugSetMaximumBevelCaptureTopologyNodes:(NSUInteger)limit {
    [GLController debugSetMaximumBevelCaptureTopologyNodes:limit];
}

- (void)debugSetMaximumBevelResultTopologyNodes:(NSUInteger)limit {
    [GLController debugSetMaximumBevelResultTopologyNodes:limit];
}

- (void)debugSetMaximumBevelResultSolids:(NSUInteger)limit {
    [GLController debugSetMaximumBevelResultSolids:limit];
}

- (void)debugSetBevelTransactionFailureCount:(NSUInteger)count {
    [GLController debugSetBevelTransactionFailureCount:count];
}

- (void)debugSetBevelCancelDiscardFailureCount:(NSUInteger)count {
    [GLController debugSetBevelCancelDiscardFailureCount:count];
}

- (BOOL)debugMutateFirstBevelSourcePersistedTransform {
    return [GLController debugMutateFirstBevelSourcePersistedTransform];
}
#endif

- (void)viewDidInvalidateSceneSnapshot {
    // Renderer-neutral extension point. The OpenGL backend owns invalidation;
    // clients may coalesce immutable snapshot publication for another renderer.
}

- (void)viewDidChangeViewportPresentationState {
    // Renderer-neutral observation point. Core3D's editing state is already
    // authoritative, while the UI notification remains deliberately debounced.
}

- (void)viewDidChangeViewportPresentationOverlay {
    // Renderer-neutral observation point for bounded transient modeling
    // overlays whose committed scene and selection remain unchanged.
}

- (void)viewWillSelectAtDrawablePoint:(CGPoint)point
                         drawableSize:(CGSize)drawableSize {
    (void)point;
    (void)drawableSize;
    // Renderer-neutral observation point. OCCT remains authoritative and the
    // tap continues through its normal selection/tool path after this returns.
}

- (void)viewWillBeginPrimaryInteractionAtDrawablePoint:(CGPoint)point
                                          drawableSize:(CGSize)drawableSize {
    (void)point;
    (void)drawableSize;
    // Renderer-neutral observation point used to retain a presented frame
    // before the authoritative OCCT interaction invalidates it.
}

- (void)viewDidEndPrimaryInteractionCancelled:(BOOL)cancelled {
    (void)cancelled;
    // OCCT has already resolved the interaction. Mirror Apply is valid only
    // while an authoritative transient body exists; lifecycle cancellation
    // clears that body through the same renderer-neutral callback.
    if (_currentGizmoType == PrimitiveGizmoTypeMirror) {
        const Core3DModelingPreviewStatus aStatus =
            [self modelingPreviewStatusForGizmoType:
                PrimitiveGizmoTypeMirror];
        self.can_apply = aStatus.canApply;
        // Lifecycle cancellation can clear an idle trial without an active
        // raw touch. Publish the finalized mirror presentation state here so
        // alternate renderers recapture the overlay instead of reusing the
        // last camera/frame publication with stale preview geometry.
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply];
    }
    if (_currentGizmoType == PrimitiveGizmoTypeLinearArray) {
        const Core3DModelingPreviewStatus status =
            [self modelingPreviewStatusForGizmoType:
                PrimitiveGizmoTypeLinearArray];
        if (!status.active) {
            [self completeOperationInteraction];
        } else {
            self.can_apply = status.canApply;
            [self viewDidChangeViewportPresentationState];
            [self sendNotifyUIState:UIStateChangingApply];
        }
    }
    if (_currentGizmoType == PrimitiveGizmoTypeRadialArray) {
        const Core3DModelingPreviewStatus status =
            [self modelingPreviewStatusForGizmoType:
                PrimitiveGizmoTypeRadialArray];
        if (!status.active) {
            [self completeOperationInteraction];
        } else {
            self.can_apply = status.canApply;
            [self viewDidChangeViewportPresentationState];
            [self sendNotifyUIState:UIStateChangingApply];
        }
    }
    if (_currentGizmoType == PrimitiveGizmoTypeShell) {
        const Core3DModelingPreviewStatus status =
            [self modelingPreviewStatusForGizmoType:
                PrimitiveGizmoTypeShell];
        if (!status.active) {
            [self completeOperationInteraction];
        } else {
            self.can_apply = status.canApply;
            [self viewDidChangeViewportPresentationOverlay];
            [self sendNotifyUIState:UIStateChangingApply];
        }
    }
    if (_currentGizmoType == PrimitiveGizmoTypeChamfer
        && ![GLController hasActiveBevel]) {
        [self completeOperationInteraction];
    }
}

- (Core3DSelectionTypeChangeResult)
    trySetSelectionType:(PrimitiveSelectionType)type
{
    if (![NSThread isMainThread]) {
        return Core3DSelectionTypeChangeResultWrongThread;
    }
    if (GLController == nil) {
        // Objective-C scalar messaging to nil returns zero, which is the raw
        // value for Succeeded. Guard the public authority before forwarding so
        // an uninstalled viewer is reported truthfully and without side effects.
        return Core3DSelectionTypeChangeResultNotReady;
    }
    const BOOL requestedCurrentPublicMode = _currentSelectionType == type;
    const BOOL wasExactSameMode = requestedCurrentPublicMode
        && [GLController getSelectionType] == type;
    const PrimitiveGizmoType toolBeforeRequest = _currentGizmoType;
    const BOOL tracksModelingOperation =
        Core3DIsModelingOperationGizmo(toolBeforeRequest);
    const Core3DModelingPreviewStatus operationBeforeRequest =
        tracksModelingOperation
            ? [self modelingPreviewStatusForGizmoType:toolBeforeRequest]
            : Core3DModelingPreviewStatus{};
    const Core3DSelectionTypeChangeResult result =
        [GLController trySetSelectionType:type];
    const Core3DModelingPreviewStatus operationAfterRequest =
        tracksModelingOperation
            ? [self modelingPreviewStatusForGizmoType:toolBeforeRequest]
            : Core3DModelingPreviewStatus{};
    const BOOL retainedSameModeOperation =
        result == Core3DSelectionTypeChangeResultSucceeded
        && requestedCurrentPublicMode
        && tracksModelingOperation
        && operationBeforeRequest.active
        && operationAfterRequest.active
        && operationAfterRequest.operation == operationBeforeRequest.operation
        && operationAfterRequest.generation
            == operationBeforeRequest.generation
        && operationAfterRequest.state == operationBeforeRequest.state
        && operationAfterRequest.canApply == operationBeforeRequest.canApply;
    if (retainedSameModeOperation) {
        // Native operation-owned presentations are intentionally deactivated,
        // so the strict ordinary-selection getter may report None while the
        // accepted logical mode and typed operation remain unchanged. Treat
        // that proven native no-op as success without closing its recovery UI.
        return Core3DSelectionTypeChangeResultSucceeded;
    }
    if (result != Core3DSelectionTypeChangeResultSucceeded
        || [GLController getSelectionType] != type) {
        const Core3DSelectionTypeChangeResult publicResult =
            result == Core3DSelectionTypeChangeResultSucceeded
                ? Core3DSelectionTypeChangeResultPresentationFailure
                : result;
        if (publicResult == Core3DSelectionTypeChangeResultBusy) {
            // An already outcome-unknown controller owns the only recovery
            // authority. Busy is a strict no-op at both native and public layers.
            return publicResult;
        }
        if (publicResult
                == Core3DSelectionTypeChangeResultPresentationFailure) {
            const BOOL operationCleanupBegan =
                tracksModelingOperation
                && operationBeforeRequest.active
                && (!operationAfterRequest.active
                    || operationAfterRequest.generation
                        != operationBeforeRequest.generation
                    || operationAfterRequest.state
                        != operationBeforeRequest.state);
            if (operationCleanupBegan) {
                // A typed controller already mutated or retired its ledger.
                // Close stale operation UI rather than fabricating recovery
                // from the retained enum.
                [self completeOperationInteraction];
            } else {
                // Presentation verification can fail after native OCCT has
                // restored the prior modes, tolerance, and still-valid selected
                // owners. Transient detection may already have been invalidated
                // by mode reconfiguration. Merely reporting that failure must
                // not call completeOperationInteraction(), which would destroy
                // the successful best-effort rollback.
                _currentSelectionType = [GLController getSelectionType];
                _currentGizmoType = [GLController getGizmoType];
                if (tracksModelingOperation) {
                    self.can_apply = operationAfterRequest.canApply;
                }
                [self viewDidChangeViewportPresentationState];
                [self sendNotifyUIState:UIStateChangingSelection
                                         | UIStateChangingGizmo
                                         | UIStateChangingApply];
            }
        } else if (publicResult
                       != Core3DSelectionTypeChangeResultUnsupported
                   && publicResult
                       != Core3DSelectionTypeChangeResultNotReady) {
            [self viewDidChangeViewportPresentationState];
            [self sendNotifyUIState:UIStateChangingSelection
                                 | UIStateChangingGizmo
                                 | UIStateChangingApply];
        }
        return publicResult;
    }
    if (wasExactSameMode) {
        // GL checks every outcome-unknown recovery barrier before its exact
        // same-mode fast path. A successful exact request therefore performed
        // no native mutation; preserve a Ready operation's typed tool UI.
        return Core3DSelectionTypeChangeResultSucceeded;
    }
    // A changed mode or successful same-enum authority repair may have cleared
    // stale owners. Recompute public state only after native state is final.
    _currentSelectionType = type;

        _availableGizmoTypes = @[];
        self.can_apply = NO;
        self.can_delete = NO;
        self.can_duplicate = NO;
        self.can_apply_material = NO;
        switch (_currentSelectionType) {
            case PrimitiveSelectionTypeShape:
                if ([GLController isSelected]) {
                    _availableGizmoTypes = @[@(PrimitiveGizmoTypeMoveRotate),
                                             @(PrimitiveGizmoTypeScale),
                                             @(PrimitiveGizmoTypeChamfer),
                                             @(PrimitiveGizmoTypeMirror),
                                             @(PrimitiveGizmoTypeLinearArray),
                                             @(PrimitiveGizmoTypeRadialArray),
                                             @(PrimitiveGizmoTypeSubtract),
                                             @(PrimitiveGizmoTypeUnion),
                                             @(PrimitiveGizmoTypeIntersect),
                                             @(PrimitiveGizmoTypeMaterial)];
                } else {
                    [GLController setGizmoType:
                        PrimitiveGizmoTypeMoveRotate];
                    _currentGizmoType = [GLController getGizmoType];
                }
                _can_delete = [GLController isSelected];
                _can_duplicate = [GLController isSelected];
                break;
            case PrimitiveSelectionTypeEdge:
            {
                BOOL isEmptyOfDisplayedObjects = [GLController isEmptyOfDisplayedObjects];
                BOOL isSelected = [GLController isSelected];
                if (!isEmptyOfDisplayedObjects && isSelected) {
                    _availableGizmoTypes = @[@(PrimitiveGizmoTypeChamfer)];
                }
            }
                _can_delete = NO;
                _can_duplicate = NO;
                [GLController setGizmoType:PrimitiveGizmoTypeNone];
                _currentGizmoType = [GLController getGizmoType];
                break;
            case PrimitiveSelectionTypeFace:
            {
                BOOL isEmptyOfDisplayedObjects =
                    [GLController isEmptyOfDisplayedObjects];
                BOOL isSelected = [GLController isSelected];
                if (!isEmptyOfDisplayedObjects && isSelected) {
                    _availableGizmoTypes = @[
                        @(PrimitiveGizmoTypeChamfer),
                        @(PrimitiveGizmoTypeExtrude),
                        @(PrimitiveGizmoTypeShell)
                    ];
                }
            }
                _can_delete = NO;
                _can_duplicate = NO;
                [GLController setGizmoType:PrimitiveGizmoTypeNone];
                _currentGizmoType = [GLController getGizmoType];
                break;

            default:
                break;
        }

        [GLController refreshSelectionState];
        // GLController emits render invalidations while this method is still
        // reconciling the public selection/gizmo state. Observe once more only
        // after that state is authoritative so alternate renderers cannot stay
        // gated by an intermediate mode until the debounced UI refresh.
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingSelection
                                 | UIStateChangingGizmo
                                 | UIStateChangingDelete
                                 | UIStateChangingDuplicate];
    return Core3DSelectionTypeChangeResultSucceeded;
}

- (void)setSelectionType:(PrimitiveSelectionType)type {
    (void)[self trySetSelectionType:type];
}

- (void)setGizmoType:(PrimitiveGizmoType)type {
    const PrimitiveSelectionType nativeSelectionType =
        [GLController getSelectionType];
    if (type != PrimitiveGizmoTypeNone
        && (_currentSelectionType != nativeSelectionType
            || !Core3DSelectionTypeAllowsGizmo(
                _currentSelectionType, type))) {
        return;
    }
    const PrimitiveGizmoType nativeGizmoType =
        [GLController getGizmoType];
    if (_currentGizmoType != type || nativeGizmoType != type
		|| type == PrimitiveGizmoTypeChamfer) {
        [GLController setGizmoType:type];
		_currentGizmoType = [GLController getGizmoType];
        switch (_currentGizmoType) {
            case PrimitiveGizmoTypeChamfer:
                self.can_apply = [_glController canApplyChamfer];
                break;
            case PrimitiveGizmoTypeSubtract:
            case PrimitiveGizmoTypeUnion:
            case PrimitiveGizmoTypeIntersect:
                self.can_apply = [_glController canApplyBoolean];
                break;
            case PrimitiveGizmoTypeMirror:
            {
                const Core3DModelingPreviewStatus aStatus =
                    [self modelingPreviewStatusForGizmoType:
                        PrimitiveGizmoTypeMirror];
                self.can_apply = aStatus.canApply;
                break;
            }
            case PrimitiveGizmoTypeLinearArray:
            {
                const Core3DModelingPreviewStatus status =
                    [self modelingPreviewStatusForGizmoType:
                        PrimitiveGizmoTypeLinearArray];
                self.can_apply = status.canApply;
                break;
            }
            case PrimitiveGizmoTypeRadialArray:
            {
                const Core3DModelingPreviewStatus status =
                    [self modelingPreviewStatusForGizmoType:
                        PrimitiveGizmoTypeRadialArray];
                self.can_apply = status.canApply;
                break;
            }
            case PrimitiveGizmoTypeShell:
            {
                const Core3DModelingPreviewStatus status =
                    [self modelingPreviewStatusForGizmoType:
                        PrimitiveGizmoTypeShell];
                self.can_apply = status.canApply;
                break;
            }
            case PrimitiveGizmoTypeExtrude:
                self.can_apply = [GLController canApplyExtrusion];
                break;
            default:
                break;
        }
        // GLController invalidates the native view before the public gizmo and
        // capability state above is final. Publish one final-state observation
        // so alternate renderers cannot remain promoted over an OCCT preview.
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingGizmo | UIStateChangingApply];
    }
}

- (void)addTestPrimitives {
    [_glController addTestPrimitives];
}

- (void)viewWillUpdateUIState:(UIStateChanging)state {}

- (void)viewDidSetup {
    if (!_isSetuped) {
        _isSetuped = YES;
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController == nil ? nullptr : GLController.viewer;
        if (viewer != nullptr) {
            __weak typeof(self) weakSelf = self;
            viewer->setInteractorRecreatedCallback(
                [weakSelf](
                    const core3d::PrimitiveManipulatorType nativeType) {
                    __strong typeof(weakSelf) strongSelf = weakSelf;
                    if (strongSelf == nil || ![NSThread isMainThread]) {
                        return;
                    }
                    // redrawDocument() may deliberately downgrade a
                    // selection-dependent Scale tool to None. Mirror the exact
                    // native value synchronously: GLController publishes its
                    // selection callback immediately after redraw and asserts
                    // that these two authorities already agree.
                    strongSelf->_currentGizmoType =
                        static_cast<PrimitiveGizmoType>(nativeType);
                    strongSelf.can_apply = NO;
                });
        }
        if (_shouldLoadAssetFileURL != nil) {
            [self loadFromAssetFile:_shouldLoadAssetFileURL
                 expectedByteCount:_shouldLoadAssetByteCount
                     expectedSHA256:_shouldLoadAssetSHA256];
        } else if (_shouldLoadBundleUrl != NULL) {
            [self loadFromBundle:_shouldLoadBundleUrl];
        }
    }
}

- (void)viewDidLoadFromBundle {
    if (_delegate && [_delegate respondsToSelector:@selector(viewDidLoadFromBundle)]) {
        [_delegate viewDidLoadFromBundle];
    }
}

- (void)viewDidFailToLoadFromBundle:(Core3DAssetLoadResult)result {
    if (_delegate && [_delegate respondsToSelector:@selector(viewDidFailToLoadFromBundle:)]) {
        [_delegate viewDidFailToLoadFromBundle:result];
    }
}

- (void)viewDidAssetModify {}

- (void)sendNotifyUIState:(UIStateChanging)state {
#ifdef DEBUG
    if (state == UIStateChangingCoreInfoText) {
        [self viewWillUpdateUIState:state];
    }
#endif
    if (_uiStateChangingBlock != nil) {
        cancel_block(_uiStateChangingBlock);
        _uiStateChangingBlock = nil;
    }
    _sendingState |= state;
    __weak typeof(self) weakSelf = self;
    _uiStateChangingBlock = dispatch_after_delay(0.1, ^{
        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf) return;
        if (strongSelf->_sendingState != UIStateChangingNone) {
            [strongSelf viewWillUpdateUIState:strongSelf->_sendingState];
            strongSelf->_sendingState = UIStateChangingNone;
        }
    });
}

- (void)sendNotifyAssetModified {
    if (_modifiedAssetBlock != nil) {
        cancel_block(_modifiedAssetBlock);
        _modifiedAssetBlock = nil;
    }
    __weak typeof(self) weakSelf = self;
    _modifiedAssetBlock = dispatch_after_delay(0.3, ^{
        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf) return;
        [strongSelf viewDidAssetModify];
    });
}

- (PrimitiveSelectionType) currentSelectionType {
    return _currentSelectionType;
}

- (PrimitiveGizmoType) currentGizmoType {
    return _currentGizmoType;
}

- (void) setPreviewMode {
    _isPreviewMode = YES;
    [GLController setPreviewMode];
}

- (void)assetData:(void(^)(NSData *_Nullable))completion {
    if (_isLoading || !_isSetuped) {
        completion(NULL);
        return;
    }
    [GLController assetData:^(NSData * _Nullable data) {
        completion(data);
    }];
}

- (NSData *)thumbData {
    return [GLController thumbData];
}

- (void)reconcilePublicStateAfterDocumentLifecycleActivatingPassiveTool:
    (BOOL)activatePassiveTool {
    if (GLController == nil) {
        return;
    }
    // Interactor recreation can clear owners or downgrade a selection-dependent
    // tool. Reset derived public state first, then synchronously rebuild it from
    // the exact native selection callback. This same path handles both a fresh
    // replacement and a failed ImportCbf rollback.
    _currentSelectionType = [GLController getSelectionType];
    _currentGizmoType = [GLController getGizmoType];
    _availableGizmoTypes = @[];
    self.can_apply = false;
    self.can_delete = false;
    self.can_duplicate = false;
    self.can_apply_material = false;
    if (activatePassiveTool) {
        // A successful replacement is a fresh native boundary: WholeShape, no
        // owners, and None. Move/Rotate is the explicit passive post-load tool;
        // never infer an operation from the replaced document's old enum.
        [self setGizmoType:PrimitiveGizmoTypeMoveRotate];
    }
    [GLController refreshSelectionState];

    const Core3DModelingPreviewStatus retainedOperation =
        Core3DIsModelingOperationGizmo(_currentGizmoType)
            ? [self modelingPreviewStatusForGizmoType:_currentGizmoType]
            : Core3DModelingPreviewStatus{};
    if (retainedOperation.active) {
        // Busy is a strict document no-op. If a typed operation retained its
        // ledger, keep its one recovery control visible after the refresh.
        _availableGizmoTypes = @[@(_currentGizmoType)];
        self.can_apply = retainedOperation.canApply;
    }
    [self sendNotifyUIState:UIStateChangingSelection
                             | UIStateChangingGizmo
                             | UIStateChangingApply
                             | UIStateChangingDelete
                             | UIStateChangingDuplicate
                             | UIStateChangingApplyMaterial
                             | UIStateChangingHistory];
}

- (void)activatePassiveStateAfterDocumentReplacement {
    [self reconcilePublicStateAfterDocumentLifecycleActivatingPassiveTool:YES];
}

- (void)loadFromBundle:(NSURL *)bundleUrl {
    if(bundleUrl == nil) {
        [self viewDidFailToLoadFromBundle:Core3DAssetLoadResultInternalFailure];
        return;
    }
    
    [self cancelObjectAlignment];
    [self cancelProfileConstruction];
    _isLoading = true;
    _shouldLoadAssetFileURL = nil;
    _shouldLoadAssetByteCount = 0;
    _shouldLoadAssetSHA256 = nil;
    if (!_isSetuped) {
        _shouldLoadBundleUrl = bundleUrl;
        return;
    }
    _shouldLoadBundleUrl = nil;
    NSData *assetData = nil;
    BOOL foundAssetItem = NO;
    Core3DAssetLoadResult readFailure = Core3DAssetLoadResultInvalidData;
    __auto_type bundleReader = [AssetBundle makeReaderWithUrl:bundleUrl];
    for (AssetBundleItem *item in bundleReader.items) {
        if (item.type == AssetBundleItemTypeAsset) {
            foundAssetItem = YES;
            assetData = item.data;
            if (!assetData) {
                NSLog(@"ERROR: load from bundle: NULL DATA");
                readFailure = Core3DAssetLoadResultTemporaryFileFailure;
            }
            break;
        }
    }

    if (assetData) {
        __weak typeof(self) weakSelf = self;
        [GLController setAssetData:assetData completion:^(Core3DAssetLoadResult result) {
            __strong typeof(weakSelf) strongSelf = weakSelf;
            if (!strongSelf) return;
            strongSelf->_isLoading = false;
            if (result != Core3DAssetLoadResultSuccess) {
                [strongSelf
                    reconcilePublicStateAfterDocumentLifecycleActivatingPassiveTool:
                        NO];
                [strongSelf viewDidFailToLoadFromBundle:result];
                return;
            }
            [(GLViewController *)strongSelf.glController fitAll];
            [strongSelf activatePassiveStateAfterDocumentReplacement];
            [strongSelf viewDidLoadFromBundle];
        }];
    } else {
        _isLoading = false;
        [self viewDidFailToLoadFromBundle:foundAssetItem
            ? readFailure
            : Core3DAssetLoadResultInvalidData];
    }
}

- (void)loadFromAssetFile:(NSURL *)assetFileURL
        expectedByteCount:(unsigned long long)expectedByteCount
            expectedSHA256:(NSString *)expectedSHA256 {
    if (assetFileURL == nil || expectedByteCount == 0
        || expectedSHA256 == nil) {
        [self viewDidFailToLoadFromBundle:
            Core3DAssetLoadResultInvalidData];
        return;
    }

    [self cancelObjectAlignment];
    [self cancelProfileConstruction];
    _isLoading = true;
    if (!_isSetuped) {
        _shouldLoadBundleUrl = nil;
        _shouldLoadAssetFileURL = assetFileURL;
        _shouldLoadAssetByteCount = expectedByteCount;
        _shouldLoadAssetSHA256 = expectedSHA256;
        return;
    }
    _shouldLoadAssetFileURL = nil;
    _shouldLoadAssetByteCount = 0;
    _shouldLoadAssetSHA256 = nil;
    _shouldLoadBundleUrl = nil;

    __weak typeof(self) weakSelf = self;
    [GLController setAssetFileURL:assetFileURL
               expectedByteCount:expectedByteCount
                   expectedSHA256:expectedSHA256
                       completion:^(Core3DAssetLoadResult result) {
        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf) return;
        strongSelf->_isLoading = false;
        if (result != Core3DAssetLoadResultSuccess) {
            [strongSelf
                reconcilePublicStateAfterDocumentLifecycleActivatingPassiveTool:
                    NO];
            [strongSelf viewDidFailToLoadFromBundle:result];
            return;
        }
        [(GLViewController *)strongSelf.glController fitAll];
        [strongSelf activatePassiveStateAfterDocumentReplacement];
        [strongSelf viewDidLoadFromBundle];
    }];
}

- (void)saveSnapshot {
    [GLController saveSnapshot];
}

- (BOOL)isEmptyOfDisplayedObjects {
    return [GLController isEmptyOfDisplayedObjects];
}

- (NSInteger)numberOfDisplayedShapes {
    return [GLController numberOfDisplayedShapes];
}

@end
