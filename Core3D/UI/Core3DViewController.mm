#if DEBUG
#include "../OCCTKit/SavedCutSourceChangedQualification.hxx"
#include <thread>
#endif
#if DEBUG
#include "../OCCTKit/DetachedRectangularLoftProbe.hxx"
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <BRepBndLib.hxx>
#endif
#if DEBUG
#include "../OCCTKit/DetachedPlanarSweepProbe.hxx"
#include <BRepMesh_IncrementalMesh.hxx>
#include <StlAPI_Writer.hxx>
#endif
#if DEBUG
#include "../OCCTKit/SweepPersistenceProbe.hxx"
#include "../OCCTKit/SavedFeatureRecords.hxx"
#include "../OCCTKit/EnclosureParameters.hxx"
#include "../OCCTKit/EnclosureGeometry.hxx"
#include <BRepClass3d_SolidClassifier.hxx>
#include <Graphic3d_CLight.hxx>
#include <Prs3d_ShadingAspect.hxx>
#endif
#include "../OCCTKit/ProfileCurvePresets.hxx"
#import "../OCCTKit/GLViewController+QueuedAssetLoading.h"
#import "../OCCTKit/Core3DQueuedAssetRequest.h"
#if DEBUG
#include "../OCCTKit/NativeWindingPlanProbe.hxx"
#include "../OCCTKit/NativeTriangleContacts.hpp"
#include "../OCCTKit/NativeWindingCandidateProbe.hxx"
#endif
#if DEBUG
#include "../OCCTKit/NativeLiveTransactionObserverProbe.hxx"
#endif
#include "../Scene/MikkTangentSpace.hpp"
#if DEBUG
#include "../OCCTKit/Core3DBoundedAuthoredFrameDriver.hxx"
#include "../OCCTKit/Core3DNativeTangentBuffers.hxx"
#include <OpenGl_ShaderProgram.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <sstream>
#endif
//
//  Core3DViewController.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 28.03.2024.
//

#import "GLViewController.h"
#import "GLView.h"
#import "Core3DViewController.h"
#import "Core3DViewController+AvailabilityManager.h"
#import "Core3DViewController+PrimitiveManager.h"
#import "Core3DViewController+GLViewControllerProtocol.h"
#import <Core3D/AssetBundle.h>
#import "../Viewport/Core3DSceneSnapshotFactory.hpp"
#import "Core3DTransformInspectorSnapshotFactory.hpp"
#import "../MeshCheck/Core3DMeshContactOperation+Private.h"

#include "GLViewController+Trick.h"
#include "BooleanOperationController.hpp"
#include "OrdinaryEditCommand.hpp"
#include "OrdinaryEditController.hpp"
#include "TransformInspectorMeasurementController.hpp"
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/SavedCutSourceDetachedWork.hxx"
#include <set>
#include "../OCCTKit/NativeModelingRequest.hxx"
#include "../OCCTKit/NativeModelingTombstone.hxx"
#include "../OCCTKit/NativeRigidPlacementEvidence.hxx"
#if DEBUG
#include "../OCCTKit/NativeModelingReceipt.hxx"
#include "../OCCTKit/ReceiptCatalogBinaryDriver.hxx"
#if DEBUG
#include "../OCCTKit/NativeModelingReceiptLegacyDebug.hxx"
#include "../OCCTKit/ReceiptCatalogProbe.hxx"
#endif
#include "../OCCTKit/SweepRebuildDefinition.hxx"
#include <XCAFDoc_ShapeMapTool.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TDF_Tool.hxx>
#include <thread>
#include <atomic>
#include <stdexcept>
#endif

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
#include <BRepAdaptor_Surface.hxx>
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
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
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

@implementation Core3DProfileCurveVertex
- (instancetype)initWithIdentifier:(uint32_t)identifier point:(CGPoint)point {
    if (identifier == 0 || !std::isfinite(point.x) || !std::isfinite(point.y)
        || std::abs(point.x) > 1e6 || std::abs(point.y) > 1e6) return nil;
    self = [super init];
    if (self) { _identifier = identifier; _point = point; }
    return self;
}
@end

@implementation Core3DProfileCurveSegment
- (instancetype)initWithIdentifier:(uint32_t)identifier
    startVertex:(uint32_t)startVertex endVertex:(uint32_t)endVertex
    kind:(Core3DProfileCurveKind)kind center:(CGPoint)center radius:(double)radius
    startDegrees:(double)startDegrees sweepDegrees:(double)sweepDegrees {
    if (identifier == 0 || startVertex == 0 || endVertex == 0 || startVertex == endVertex
        || !std::isfinite(center.x) || !std::isfinite(center.y)
        || !std::isfinite(radius) || !std::isfinite(startDegrees)
        || !std::isfinite(sweepDegrees)) return nil;
    if (kind == Core3DProfileCurveKindLine) {
        if (center.x != 0 || center.y != 0 || radius != 0
            || startDegrees != 0 || sweepDegrees != 0) return nil;
    } else if (kind == Core3DProfileCurveKindCircularArc) {
        if (std::abs(center.x) > 1e6 || std::abs(center.y) > 1e6
            || radius < 1e-3 || radius > 1e6 || std::abs(startDegrees) > 360
            || std::abs(sweepDegrees) < 1e-6 || std::abs(sweepDegrees) >= 360) return nil;
    } else return nil;
    self = [super init];
    if (self) {
        _identifier = identifier; _startVertex = startVertex; _endVertex = endVertex;
        _kind = kind; _center = center; _radius = radius;
        _startDegrees = startDegrees; _sweepDegrees = sweepDegrees;
    }
    return self;
}
@end

static Core3DProfileCurveLoop *Core3DPublicCurveLoop(const core3d::ProfileCurveLoop& loop);
template<std::size_t N>
static bool Core3DProfilePresetIdentifiers(uint32_t loopID,
    NSArray<NSNumber *> *vertices, NSArray<NSNumber *> *segments,
    core3d::ProfileCurvePresetIDs<N>& output) {
    if (loopID == 0 || ![vertices isKindOfClass:[NSArray class]]
        || ![segments isKindOfClass:[NSArray class]]
        || vertices.count != N || segments.count != N) return false;
    const auto read = [](id value, core3d::ProfileCurveID& identifier) {
        if (![value isKindOfClass:[NSNumber class]]) return false;
        const double scalar = [value doubleValue];
        if (!std::isfinite(scalar) || scalar < 1 || std::floor(scalar) != scalar
            || scalar > std::numeric_limits<std::uint32_t>::max()) return false;
        identifier = static_cast<std::uint32_t>(scalar); return true;
    };
    core3d::ProfileCurvePresetIDs<N> result; result.loop = loopID;
    for (std::size_t i = 0; i < N; ++i)
        if (!read(vertices[i],result.vertices[i]) || !read(segments[i],result.segments[i])) return false;
    output = result; return true;
}


@interface Core3DProfileCurveLoop ()
- (core3d::ProfileCurveLoop)nativeLoop;
@end
@implementation Core3DProfileCurveLoop {
    core3d::ProfileCurveLoop _native;
}
- (instancetype)initWithIdentifier:(uint32_t)identifier
    vertices:(NSArray<Core3DProfileCurveVertex *> *)vertices
    segments:(NSArray<Core3DProfileCurveSegment *> *)segments {
    if (identifier == 0 || ![vertices isKindOfClass:[NSArray class]]
        || ![segments isKindOfClass:[NSArray class]] || vertices.count < 2
        || vertices.count > 512 || vertices.count != segments.count) return nil;
    NSArray *ownedVertices = [vertices copy], *ownedSegments = [segments copy];
    try {
        core3d::ProfileCurveLoop loop; loop.identifier = identifier;
        loop.vertices.reserve(ownedVertices.count); loop.segments.reserve(ownedSegments.count);
        for (id value in ownedVertices) {
            if (![value isMemberOfClass:[Core3DProfileCurveVertex class]]) return nil;
            Core3DProfileCurveVertex *v = value;
            loop.vertices.push_back({v.identifier, gp_Pnt2d(v.point.x,v.point.y)});
        }
        for (id value in ownedSegments) {
            if (![value isMemberOfClass:[Core3DProfileCurveSegment class]]) return nil;
            Core3DProfileCurveSegment *v = value;
            loop.segments.push_back({v.identifier,v.startVertex,v.endVertex,
                static_cast<core3d::ProfileCurveKind>(v.kind),gp_Pnt2d(v.center.x,v.center.y),
                v.radius,v.startDegrees,v.sweepDegrees});
        }
        std::set<core3d::ProfileCurveID> identifiers;
        std::size_t vertexCount = 0, segmentCount = 0;
        core3d::ProfileCurveLoopInspection inspection;
        if (!core3d::InspectProfileCurveLoopStructure(loop,loop.vertices.front().point,
            identifiers,vertexCount,segmentCount,inspection)) return nil;
        self = [super init];
        if (self) {
            _native = std::move(loop); _identifier = identifier;
            _vertices = ownedVertices; _segments = ownedSegments;
        }
        return self;
    } catch (...) { return nil; }
}
- (core3d::ProfileCurveLoop)nativeLoop { return _native; }
+ (Core3DProfileCurveLoop *)roundedRectangleWithMinimum:(CGPoint)minimum
    maximum:(CGPoint)maximum radius:(double)radius identifier:(uint32_t)identifier
    vertexIdentifiers:(NSArray<NSNumber *> *)vertices segmentIdentifiers:(NSArray<NSNumber *> *)segments {
    try {
        core3d::ProfileCurvePresetIDs<8> ids;
        core3d::ProfileCurveLoop loop;
        if (!Core3DProfilePresetIdentifiers(identifier,vertices,segments,ids)
            || !core3d::BuildRoundedRectangleProfileLoop(gp_Pnt2d(minimum.x,minimum.y),
                gp_Pnt2d(maximum.x,maximum.y),radius,ids,loop)) return nil;
        return Core3DPublicCurveLoop(loop);
    } catch (...) { return nil; }
}

+ (Core3DProfileCurveLoop *)capsuleWithCenter:(CGPoint)center length:(double)length
    diameter:(double)diameter rotationDegrees:(double)rotationDegrees identifier:(uint32_t)identifier
    vertexIdentifiers:(NSArray<NSNumber *> *)vertices segmentIdentifiers:(NSArray<NSNumber *> *)segments {
    try {
        core3d::ProfileCurvePresetIDs<4> ids;
        core3d::ProfileCurveLoop loop;
        if (!Core3DProfilePresetIdentifiers(identifier,vertices,segments,ids)
            || !core3d::BuildCapsuleProfileLoop(gp_Pnt2d(center.x,center.y),length,
                diameter,rotationDegrees,ids,loop)) return nil;
        return Core3DPublicCurveLoop(loop);
    } catch (...) { return nil; }
}

@end

static Core3DProfileCurveLoop *Core3DPublicCurveLoop(const core3d::ProfileCurveLoop& loop) {
    NSMutableArray<Core3DProfileCurveVertex *> *vertices = [NSMutableArray arrayWithCapacity:loop.vertices.size()];
    NSMutableArray<Core3DProfileCurveSegment *> *segments = [NSMutableArray arrayWithCapacity:loop.segments.size()];
    for (const auto& v : loop.vertices) {
        auto value = [[Core3DProfileCurveVertex alloc] initWithIdentifier:v.identifier
            point:CGPointMake(v.point.X(),v.point.Y())];
        if (!value) return nil;
        [vertices addObject:value];
    }
    for (const auto& v : loop.segments) {
        auto value = [[Core3DProfileCurveSegment alloc] initWithIdentifier:v.identifier
            startVertex:v.startVertex endVertex:v.endVertex
            kind:static_cast<Core3DProfileCurveKind>(v.kind)
            center:CGPointMake(v.center.X(),v.center.Y()) radius:v.radius
            startDegrees:v.startDegrees sweepDegrees:v.sweepDegrees];
        if (!value) return nil;
        [segments addObject:value];
    }
    return [[Core3DProfileCurveLoop alloc] initWithIdentifier:loop.identifier vertices:vertices segments:segments];
}


@implementation Core3DSweepPathSegment
- (instancetype)initWithIdentifier:(uint32_t)identifier
    startVertex:(uint32_t)startVertex endVertex:(uint32_t)endVertex
    kind:(Core3DProfileCurveKind)kind center:(CGPoint)center radius:(double)radius
    startDegrees:(double)startDegrees sweepDegrees:(double)sweepDegrees {
    if (identifier == 0 || startVertex == 0 || endVertex == 0 || startVertex == endVertex
        || !std::isfinite(center.x) || !std::isfinite(center.y)
        || !std::isfinite(radius) || !std::isfinite(startDegrees)
        || !std::isfinite(sweepDegrees)) return nil;
    if (kind == Core3DProfileCurveKindLine) {
        if (center.x != 0 || center.y != 0 || radius != 0
            || startDegrees != 0 || sweepDegrees != 0) return nil;
    } else if (kind == Core3DProfileCurveKindCircularArc) {
        if (std::abs(center.x) > 1e6 || std::abs(center.y) > 1e6
            || radius <= 0 || radius > 1e6 || std::abs(startDegrees) > 360
            || std::abs(sweepDegrees) < 1e-6 || std::abs(sweepDegrees) >= 360) return nil;
    } else return nil;
    self = [super init];
    if (self) {
        _identifier = identifier; _startVertex = startVertex; _endVertex = endVertex;
        _kind = kind; _center = center; _radius = radius;
        _startDegrees = startDegrees; _sweepDegrees = sweepDegrees;
    }
    return self;
}
@end

@implementation Core3DRectangularLoftStation
- (instancetype)initWithIdentifier:(uint32_t)identifier cornerIdentifiers:(simd_uint4)corners
    correspondence:(simd_uint4)correspondence z:(double)z centerX:(double)centerX centerY:(double)centerY
    width:(double)width depth:(double)depth {
    if (!identifier || !std::isfinite(z) || !std::isfinite(centerX) || !std::isfinite(centerY)
        || !std::isfinite(width) || !std::isfinite(depth) || width<=0 || depth<=0
        || std::abs(z)>1e6 || std::abs(centerX)>1e6 || std::abs(centerY)>1e6 || width>1e6 || depth>1e6) return nil;
    for(int i=0;i<4;++i)if(!corners[i]||!correspondence[i])return nil;
    self=[super init];if(self){_identifier=identifier;_cornerIdentifiers=corners;_correspondence=correspondence;
        _z=z;_centerX=centerX;_centerY=centerY;_width=width;_depth=depth;}return self;
}
@end
@interface Core3DRectangularLoftStationEdit ()
- (core3d::rectangular_loft::StationDimensionEdit)nativeEdit;
@end
@implementation Core3DRectangularLoftStationEdit
- (instancetype)initWithStationIdentifier:(uint32_t)identifier width:(NSNumber *)width depth:(NSNumber *)depth {
    if(!identifier || (!width && !depth)
        || (width && ![width isKindOfClass:[NSNumber class]])
        || (depth && ![depth isKindOfClass:[NSNumber class]]))return nil;
    for(id value in @[width ?: NSNull.null,depth ?: NSNull.null]) {
        if(value==NSNull.null)continue;
        if(![value isKindOfClass:[NSNumber class]] || CFGetTypeID((__bridge CFTypeRef)value)==CFBooleanGetTypeID()
            || !std::isfinite([value doubleValue]) || [value doubleValue]<=0 || [value doubleValue]>1e6)return nil;
    }
    self=[super init];if(self){_stationIdentifier=identifier;_width=[width copy];_depth=[depth copy];}return self;
}
- (core3d::rectangular_loft::StationDimensionEdit)nativeEdit {
    core3d::rectangular_loft::StationDimensionEdit edit;edit.stationIdentifier=_stationIdentifier;
    if(_width)edit.width=_width.doubleValue;if(_depth)edit.depth=_depth.doubleValue;return edit;
}
@end
@interface Core3DRectangularLoftDefinition ()
- (core3d::rectangular_loft::Definition)nativeDefinition;
- (instancetype)initWithNativeDefinition:(const core3d::rectangular_loft::Definition&)definition;
@end
@implementation Core3DRectangularLoftDefinition {
    core3d::rectangular_loft::Definition _definition;
}
- (instancetype)initWithLoftIdentifier:(uint32_t)identifier correspondence:(simd_uint4)correspondence
    stations:(NSArray<Core3DRectangularLoftStation *> *)stations metersPerUnit:(double)metersPerUnit
    constructionFrameValues:(NSArray<NSNumber *> *)frame {
    if (![stations isKindOfClass:[NSArray class]] || stations.count<2 || stations.count>8
        || ![frame isKindOfClass:[NSArray class]] || (frame.count!=0&&frame.count!=8))return nil;
    try {
        core3d::rectangular_loft::Definition d;d.loftIdentifier=identifier;d.dimensionMetersPerUnit=metersPerUnit;
        for(int i=0;i<4;++i)d.correspondence[i]=correspondence[i];
        for(Core3DRectangularLoftStation *value in stations) {
            if(![value isKindOfClass:[Core3DRectangularLoftStation class]])return nil;
            core3d::rectangular_loft::Station v;v.identifier=value.identifier;
            for(int i=0;i<4;++i){v.cornerIdentifiers[i]=value.cornerIdentifiers[i];v.correspondence[i]=value.correspondence[i];}
            v.z=value.z;v.centerX=value.centerX;v.centerY=value.centerY;v.width=value.width;v.depth=value.depth;d.stations.push_back(v);
        }
        if(frame.count==8){core3d::profile::ConstructionFrame f;for(NSUInteger i=0;i<8;++i){
            if(![frame[i] isKindOfClass:[NSNumber class]]||CFGetTypeID((__bridge CFTypeRef)frame[i])==CFBooleanGetTypeID())return nil;
            f.values[i]=frame[i].doubleValue;}d.constructionFrame=f;}
        core3d::rectangular_loft::Inspection inspection;
        if(core3d::rectangular_loft::Inspect(d,inspection)!=core3d::rectangular_loft::Admission::Accepted)return nil;
        self=[super init];if(self){_definition=std::move(d);_stations=[stations copy];_constructionFrameValues=[frame copy];}
        return self;
    }catch(...){return nil;}
}
- (instancetype)initWithNativeDefinition:(const core3d::rectangular_loft::Definition&)definition {
    try {
        NSMutableArray<Core3DRectangularLoftStation *> *stations=[NSMutableArray array];
        NSMutableArray<NSNumber *> *frame=[NSMutableArray array];
        for(const auto& station:definition.stations) {
            auto value=[[Core3DRectangularLoftStation alloc] initWithIdentifier:station.identifier
                cornerIdentifiers:(simd_uint4){station.cornerIdentifiers[0],station.cornerIdentifiers[1],station.cornerIdentifiers[2],station.cornerIdentifiers[3]}
                correspondence:(simd_uint4){station.correspondence[0],station.correspondence[1],station.correspondence[2],station.correspondence[3]}
                z:station.z centerX:station.centerX centerY:station.centerY width:station.width depth:station.depth];
            if(!value)return nil;[stations addObject:value];
        }
        if(definition.constructionFrame)for(double v:definition.constructionFrame->values)[frame addObject:@(v)];
        return [self initWithLoftIdentifier:definition.loftIdentifier
            correspondence:(simd_uint4){definition.correspondence[0],definition.correspondence[1],definition.correspondence[2],definition.correspondence[3]}
            stations:stations metersPerUnit:definition.dimensionMetersPerUnit constructionFrameValues:frame];
    }catch(...){return nil;}
}
- (Core3DRectangularLoftDefinition *)changingStation:(Core3DRectangularLoftStationEdit *)edit {
    if(![edit isKindOfClass:[Core3DRectangularLoftStationEdit class]])return nil;
    try {
        core3d::rectangular_loft::Definition changed;
        if(!core3d::loft_rebuild::Apply(_definition,[edit nativeEdit],changed))return nil;
        return [[Core3DRectangularLoftDefinition alloc] initWithNativeDefinition:changed];
    }catch(...){return nil;}
}
- (uint32_t)loftIdentifier {return _definition.loftIdentifier;}
- (simd_uint4)correspondence {return {_definition.correspondence[0],_definition.correspondence[1],_definition.correspondence[2],_definition.correspondence[3]};}
- (double)metersPerUnit {return _definition.dimensionMetersPerUnit;}
- (core3d::rectangular_loft::Definition)nativeDefinition {return _definition;}
@end

@interface Core3DSweepDefinition ()
- (core3d::planar_sweep::Definition)nativeDefinition;
@end
@implementation Core3DSweepDefinition {
    core3d::planar_sweep::Definition _definition;
}
- (instancetype)initWithPathIdentifier:(uint32_t)identifier
    vertices:(NSArray<Core3DProfileCurveVertex *> *)vertices
    segments:(NSArray<Core3DSweepPathSegment *> *)segments plane:(Core3DProfilePlane)plane
    radius:(double)radius metersPerUnit:(double)metersPerUnit constructionFrameValues:(NSArray<NSNumber *> *)frame {
    if (![vertices isKindOfClass:[NSArray class]] || vertices.count<2 || vertices.count>33
        || ![segments isKindOfClass:[NSArray class]] || segments.count<1 || segments.count>32
        || vertices.count!=segments.count+1 || ![frame isKindOfClass:[NSArray class]]
        || (frame.count!=0 && frame.count!=8)
        || plane<Core3DProfilePlaneXY || plane>Core3DProfilePlaneYZ) return nil;
    try {
        core3d::planar_sweep::Definition d;d.pathIdentifier=identifier;d.plane=int(plane);
        d.radius=radius;d.dimensionMetersPerUnit=metersPerUnit;
        for (Core3DProfileCurveVertex *v in vertices) {
            if (![v isKindOfClass:[Core3DProfileCurveVertex class]]) return nil;
            d.vertices.push_back({v.identifier,gp_Pnt2d(v.point.x,v.point.y)});
        }
        for (Core3DSweepPathSegment *s in segments) {
            if (![s isKindOfClass:[Core3DSweepPathSegment class]]) return nil;
            d.segments.push_back({s.identifier,s.startVertex,s.endVertex,static_cast<core3d::ProfileCurveKind>(s.kind),
                gp_Pnt2d(s.center.x,s.center.y),s.radius,s.startDegrees,s.sweepDegrees});
        }
        if (frame.count==8) {
            core3d::profile::ConstructionFrame f;
            for (NSUInteger i=0;i<8;++i) {
                if (![frame[i] isKindOfClass:[NSNumber class]] || CFGetTypeID((__bridge CFTypeRef)frame[i])==CFBooleanGetTypeID()) return nil;
                f.values[i]=frame[i].doubleValue;
            }
            d.constructionFrame=f;
        }
        core3d::planar_sweep::Inspection inspection;
        if (core3d::planar_sweep::Inspect(d,inspection)!=core3d::planar_sweep::Admission::Accepted) return nil;
        self=[super init];
        if (self) {_definition=std::move(d);_vertices=[vertices copy];_segments=[segments copy];_constructionFrameValues=[frame copy];}
        return self;
    } catch (...) {return nil;}
}
- (uint32_t)pathIdentifier {return _definition.pathIdentifier;}
- (Core3DProfilePlane)plane {return static_cast<Core3DProfilePlane>(_definition.plane);}
- (double)radius {return _definition.radius;}
- (double)metersPerUnit {return _definition.dimensionMetersPerUnit;}
- (Core3DSweepDefinition *)changingRadius:(double)radius {
    try {
        auto d=_definition;d.radius=radius;
        core3d::planar_sweep::Inspection inspected;
        if(core3d::planar_sweep::Inspect(d,inspected)!=core3d::planar_sweep::Admission::Accepted)return nil;
        // Rebuild public readonly fields from the already frozen native values,
        // never from potentially subclassed objects supplied to the original init.
        NSMutableArray<Core3DProfileCurveVertex *> *vertices=[NSMutableArray array];
        NSMutableArray<Core3DSweepPathSegment *> *segments=[NSMutableArray array];
        NSMutableArray<NSNumber *> *frame=[NSMutableArray array];
        for(const auto& v:d.vertices){
            auto value=[[Core3DProfileCurveVertex alloc] initWithIdentifier:v.identifier point:CGPointMake(v.point.X(),v.point.Y())];
            if(!value)return nil;[vertices addObject:value];
        }
        for(const auto& v:d.segments){
            auto value=[[Core3DSweepPathSegment alloc] initWithIdentifier:v.identifier startVertex:v.startVertex endVertex:v.endVertex
                kind:static_cast<Core3DProfileCurveKind>(v.kind) center:CGPointMake(v.center.X(),v.center.Y()) radius:v.radius
                startDegrees:v.startDegrees sweepDegrees:v.sweepDegrees];
            if(!value)return nil;[segments addObject:value];
        }
        if(d.constructionFrame)for(double v:d.constructionFrame->values)[frame addObject:@(v)];
        Core3DSweepDefinition *result=[[Core3DSweepDefinition alloc] initWithPathIdentifier:d.pathIdentifier vertices:vertices segments:segments
            plane:static_cast<Core3DProfilePlane>(d.plane) radius:d.radius metersPerUnit:d.dimensionMetersPerUnit constructionFrameValues:frame];
        if(!result)return nil;
        std::vector<double> expected,actual;
        if(!core3d::sweep_persistence::Encode(d,expected)||!core3d::sweep_persistence::Encode([result nativeDefinition],actual)
            ||!core3d::sweep_persistence::SameBits(expected,actual))return nil;
        return result;
    }catch(...){return nil;}
}
- (core3d::planar_sweep::Definition)nativeDefinition {return _definition;}
@end

namespace {
bool Core3DSourceMMNumber(NSNumber *number,std::optional<double>& out) {
    out.reset();if(!number)return true;
    if(![number isKindOfClass:NSNumber.class]||CFGetTypeID((__bridge CFTypeRef)number)==CFBooleanGetTypeID())return false;
    const double value=number.doubleValue;
    if(!std::isfinite(value)||std::abs(value)>1e6)return false;
    out=value;return true;
}
bool Core3DSourceRecipeToMM(double original,double factor,double& out) {
    if(!std::isfinite(original)||!std::isfinite(factor)||factor<=0)return false;
    out=original*factor;
    return std::isfinite(out)&&!(original!=0&&out==0);
}
bool Core3DSourceMMToRecipe(double requested,double original,double factor,double& out) {
    if(!std::isfinite(requested)||std::abs(requested)>1e6||!std::isfinite(original)
        ||!std::isfinite(factor)||factor<=0)return false;
    double shown=0;
    if(!Core3DSourceRecipeToMM(original,factor,shown))return false;
    // Exact displayed-value equality preserves authored bits (including -0).
    // This is not a tolerance or a second unit conversion of the original.
    if(requested==shown){out=original;return true;}
    out=requested/factor;
    return std::isfinite(out)&&!(requested!=0&&out==0);
}
}
@implementation Core3DSavedCutSourceCoordinate
- (instancetype)initWithOrdinal:(uint32_t)ordinal component:(Core3DSavedCutSourceComponent)component valueMM:(double)value {
    if((component!=Core3DSavedCutSourceComponentU&&component!=Core3DSavedCutSourceComponentV)
        ||!std::isfinite(value)||std::abs(value)>1e6)return nil;
    self=[super init];if(self){_ordinal=ordinal;_component=component;_valueMM=value;}return self;
}
@end
@interface Core3DSavedCutSourcePatch ()
- (std::optional<core3d::saved_cut_source_edit::Patch>)nativePatchFor:(const core3d::retained_solid::Envelope&)source;
// Same one-time typed conversion against the COMPLETE original program's
// shared source; validated across every operand view, never a permit.
- (std::optional<core3d::saved_cut_source_edit::Patch>)nativePatchForProgram:(const core3d::retained_boolean::Program&)program;
@end
@implementation Core3DSavedCutSourcePatch {
    core3d::saved_cut_source_edit::Patch _millimetres;
}
- (instancetype)initWithPolygonCoordinates:(NSArray<Core3DSavedCutSourceCoordinate *> *)coordinates depthMM:(NSNumber *)depth {
    try {
        if(![coordinates isKindOfClass:NSArray.class]||coordinates.count>core3d::profile::MaximumScalars)return nil;
        core3d::saved_cut_source_values::PolygonPatch p;
        if(!Core3DSourceMMNumber(depth,p.depth))return nil;
        std::set<std::pair<uint32_t,unsigned>> seen;
        for(Core3DSavedCutSourceCoordinate *v in coordinates){
            if(![v isKindOfClass:Core3DSavedCutSourceCoordinate.class]
                ||(v.component!=Core3DSavedCutSourceComponentU&&v.component!=Core3DSavedCutSourceComponentV)
                ||!std::isfinite(v.valueMM)||std::abs(v.valueMM)>1e6
                ||!seen.emplace(v.ordinal,unsigned(v.component)).second)return nil;
            p.coordinates.push_back({v.ordinal,static_cast<core3d::saved_cut_source_values::Component>(v.component),v.valueMM});
        }
        self=[super init];if(self)_millimetres=std::move(p);return self;
    }catch(...){return nil;}
}
- (instancetype)initWithEnclosureWidthMM:(NSNumber *)width depthMM:(NSNumber *)depth heightMM:(NSNumber *)height
    wallMM:(NSNumber *)wall floorMM:(NSNumber *)floor cornerRadiusMM:(NSNumber *)corner {
    try {
        core3d::saved_cut_source_values::EnclosurePatch p;
        if(!Core3DSourceMMNumber(width,p.dimensions[0])||!Core3DSourceMMNumber(depth,p.dimensions[1])
            ||!Core3DSourceMMNumber(height,p.dimensions[2])||!Core3DSourceMMNumber(wall,p.dimensions[3])
            ||!Core3DSourceMMNumber(floor,p.dimensions[4])||!Core3DSourceMMNumber(corner,p.dimensions[5]))return nil;
        self=[super init];if(self)_millimetres=std::move(p);return self;
    }catch(...){return nil;}
}
- (std::optional<core3d::saved_cut_source_edit::Patch>)nativePatchFor:(const core3d::retained_solid::Envelope&)source {
    try {
        if(!NSThread.isMainThread||!core3d::retained_solid::Valid(source))return {};
        const double factor=source.metersPerUnit*1000;
        auto converted=_millimetres;
        if(auto* p=std::get_if<core3d::saved_cut_source_values::PolygonPatch>(&converted)){
            core3d::profile::Parameters decoded;
            if(source.sourceFamily!=1||!core3d::profile::Decode(source.sourceValues,decoded))return {};
            if(p->depth&&!Core3DSourceMMToRecipe(*p->depth,decoded.definition.depth,factor,*p->depth))return {};
            for(auto& v:p->coordinates){
                if(v.ordinal>=decoded.definition.points.size())return {};
                const auto& point=decoded.definition.points[v.ordinal];
                const double original=v.component==core3d::saved_cut_source_values::Component::U?point.X():point.Y();
                if(!Core3DSourceMMToRecipe(v.value,original,factor,v.value))return {};
            }
        }else{
            auto& enclosurePatch=std::get<core3d::saved_cut_source_values::EnclosurePatch>(converted);
            core3d::enclosure::Parameters decoded;
            if(source.sourceFamily!=2||!core3d::enclosure::Decode(int(source.sourceSchema),source.sourceValues,decoded))return {};
            const auto& d=decoded.definition.dimensions;
            const std::array<double,6> old={d.width,d.depth,d.height,d.wall,d.floor,d.cornerRadius};
            for(std::size_t i=0;i<old.size();++i)if(enclosurePatch.dimensions[i]
                &&!Core3DSourceMMToRecipe(*enclosurePatch.dimensions[i],old[i],factor,*enclosurePatch.dimensions[i]))return {};
        }
        // Complete combination and untouched bytes use the existing native
        // value codec once, without regenerating any source DTO or geometry.
        return core3d::saved_cut_source_values::Apply(source,converted)?std::optional<core3d::saved_cut_source_edit::Patch>(std::move(converted)):std::nullopt;
    }catch(...){return {};}
}
- (std::optional<core3d::saved_cut_source_edit::Patch>)nativePatchForProgram:(const core3d::retained_boolean::Program&)program {
    try {
        if(!NSThread.isMainThread||!core3d::retained_boolean::Valid(program))return {};
        const double factor=program.source.metersPerUnit*1000;
        auto converted=_millimetres;
        if(auto* p=std::get_if<core3d::saved_cut_source_values::PolygonPatch>(&converted)){
            core3d::profile::Parameters decoded;
            if(program.source.family!=1||!core3d::profile::Decode(program.source.values,decoded))return {};
            if(p->depth&&!Core3DSourceMMToRecipe(*p->depth,decoded.definition.depth,factor,*p->depth))return {};
            for(auto& v:p->coordinates){
                if(v.ordinal>=decoded.definition.points.size())return {};
                const auto& point=decoded.definition.points[v.ordinal];
                const double original=v.component==core3d::saved_cut_source_values::Component::U?point.X():point.Y();
                if(!Core3DSourceMMToRecipe(v.value,original,factor,v.value))return {};
            }
        }else{
            auto& enclosurePatch=std::get<core3d::saved_cut_source_values::EnclosurePatch>(converted);
            core3d::enclosure::Parameters decoded;
            if(program.source.family!=2||!core3d::enclosure::Decode(int(program.source.schema),program.source.values,decoded))return {};
            const auto& d=decoded.definition.dimensions;
            const std::array<double,6> old={d.width,d.depth,d.height,d.wall,d.floor,d.cornerRadius};
            for(std::size_t i=0;i<old.size();++i)if(enclosurePatch.dimensions[i]
                &&!Core3DSourceMMToRecipe(*enclosurePatch.dimensions[i],old[i],factor,*enclosurePatch.dimensions[i]))return {};
        }
        // The complete typed transition is validated once against EVERY operand
        // view of the original program; value-only, never a permit.
        return core3d::saved_boolean_build::SourcePatch(program,converted)
            ?std::optional<core3d::saved_cut_source_edit::Patch>(std::move(converted)):std::nullopt;
    }catch(...){return {};}
}
@end
@interface Core3DSavedCutSourceValues ()
- (instancetype)initWithEnvelope:(const core3d::retained_solid::Envelope&)source;
// Whole-program shared source view; verified by the caller against EVERY
// operand of the complete recipe before this descriptive copy is built.
- (instancetype)initWithProgramSource:(const core3d::retained_boolean::Source&)source;
- (instancetype)initWithFamily:(std::uint8_t)family schema:(std::uint32_t)schema
    metersPerUnit:(double)metersPerUnit values:(const std::vector<double>&)values;
@end
@implementation Core3DSavedCutSourceValues
- (instancetype)initWithEnvelope:(const core3d::retained_solid::Envelope&)source {
    try {
        if(!NSThread.isMainThread||!core3d::retained_solid::Valid(source))return nil;
        // Empty patch verifies this exact descriptor belongs to a supported
        // source family; it does not confer retained-solid correspondence.
        core3d::saved_cut_source_edit::Patch empty;
        if(source.sourceFamily==1)empty=core3d::saved_cut_source_values::PolygonPatch{};
        else if(source.sourceFamily==2)empty=core3d::saved_cut_source_values::EnclosurePatch{};
        else return nil;
        if(!core3d::saved_cut_source_values::Apply(source,empty))return nil;
        return [self initWithFamily:source.sourceFamily schema:source.sourceSchema
            metersPerUnit:source.metersPerUnit values:source.sourceValues];
    }catch(...){return nil;}
}
- (instancetype)initWithProgramSource:(const core3d::retained_boolean::Source&)source {
    try {
        if(!NSThread.isMainThread)return nil;
        return [self initWithFamily:source.family schema:source.schema
            metersPerUnit:source.metersPerUnit values:source.values];
    }catch(...){return nil;}
}
- (instancetype)initWithFamily:(std::uint8_t)family schema:(std::uint32_t)schema
    metersPerUnit:(double)metersPerUnit values:(const std::vector<double>&)values {
    try {
        if(!NSThread.isMainThread)return nil;
        const double factor=metersPerUnit*1000;
        if(!std::isfinite(factor)||factor<=0)return nil;
        if(family!=1&&family!=2)return nil;
        self=[super init];if(!self)return nil;
        _family=static_cast<Core3DSavedCutSourceFamily>(family);_metersPerUnit=metersPerUnit;
        _polygonPointsMM=@[];
        if(family==1){
            core3d::profile::Parameters p;if(!core3d::profile::Decode(values,p))return nil;
            _plane=static_cast<Core3DProfilePlane>(p.definition.plane);
            NSMutableArray<NSValue *> *points=[NSMutableArray arrayWithCapacity:p.definition.points.size()];
            for(const auto& point:p.definition.points){
                double u=0,v=0;
                if(!Core3DSourceRecipeToMM(point.X(),factor,u)||!Core3DSourceRecipeToMM(point.Y(),factor,v)
                    ||std::abs(u)>1e6||std::abs(v)>1e6)return nil;
                [points addObject:[NSValue valueWithCGPoint:CGPointMake(u,v)]];
            }
            double depth=0;if(!Core3DSourceRecipeToMM(p.definition.depth,factor,depth)||depth<=0||depth>1e6)return nil;
            _polygonPointsMM=[points copy];_depthMM=@(depth);
        }else{
            core3d::enclosure::Parameters p;if(!core3d::enclosure::Decode(int(schema),values,p))return nil;
            _plane=static_cast<Core3DProfilePlane>(p.definition.plane);const auto& d=p.definition.dimensions;
            const std::array<double,6> recipe={d.width,d.depth,d.height,d.wall,d.floor,d.cornerRadius};
            std::array<double,6> values{};
            for(std::size_t i=0;i<values.size();++i)if(!Core3DSourceRecipeToMM(recipe[i],factor,values[i])
                ||values[i]<=0||values[i]>1e6)return nil;
            _widthMM=@(values[0]);_depthMM=@(values[1]);_heightMM=@(values[2]);_wallMM=@(values[3]);_floorMM=@(values[4]);_cornerRadiusMM=@(values[5]);
        }
        return self;
    }catch(...){return nil;}
}
@end

@implementation Core3DCylindricalCutDefinition
- (instancetype)initWithAxis:(Core3DCylindricalCutAxis)axis localX:(double)x localY:(double)y localZ:(double)z worldRadiusMM:(double)radius {
    if(axis<Core3DCylindricalCutAxisX||axis>Core3DCylindricalCutAxisZ||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)
        ||!std::isfinite(radius)||radius<.001||radius>1e6)return nil;
    self=[super init];if(self){_axis=axis;_localX=x;_localY=y;_localZ=z;_worldRadiusMM=radius;}return self;
}
@end
@interface Core3DCylindricalCutSnapshot ()
- (instancetype)initWithNative:(const core3d::CylindricalCutSnapshot&)native owner:(Core3DViewController *)owner viewer:(const std::shared_ptr<core3d::Core3DViewer>&)viewer;
- (core3d::CylindricalCutSnapshot)nativeSnapshot;
- (BOOL)matchesOwner:(Core3DViewController *)owner viewer:(const std::shared_ptr<core3d::Core3DViewer>&)viewer;
@end
@implementation Core3DCylindricalCutSnapshot {
    core3d::CylindricalCutSnapshot _native;
    __weak Core3DViewController *_owner;
    std::weak_ptr<core3d::Core3DViewer> _viewer;
}
- (instancetype)initWithNative:(const core3d::CylindricalCutSnapshot&)native owner:(Core3DViewController *)owner viewer:(const std::shared_ptr<core3d::Core3DViewer>&)viewer {
    self=[super init];if(self){_native=native;_owner=owner;_viewer=viewer;
        _entityIdentifier=[NSString stringWithUTF8String:native.source.original.entityIdentifier.c_str()];
        _definitionIdentifier=[NSString stringWithUTF8String:native.source.original.definitionIdentifier.c_str()];
        _rebuilding=native.source.rebuilding;_worldRadiusMM=_rebuilding?native.source.envelope.radius*native.source.effectiveMM:0;
    }return self;
}
- (core3d::CylindricalCutSnapshot)nativeSnapshot {return _native;}
- (Core3DSavedCutSourceValues *)sourceRecipeMM {
    if(!NSThread.isMainThread||!_native.source.rebuilding||!_native.source.original.retained.value)return nil;
    return [[Core3DSavedCutSourceValues alloc] initWithEnvelope:_native.source.envelope];
}
- (BOOL)matchesOwner:(Core3DViewController *)owner viewer:(const std::shared_ptr<core3d::Core3DViewer>&)viewer {
    return NSThread.isMainThread&&owner&&_owner==owner&&viewer&&_viewer.lock()==viewer;
}
@end
@interface Core3DViewController (CylindricalCutOperationCancellation)
- (BOOL)core3d_cancelCutWork:(const std::shared_ptr<core3d::NativeSolidWork>&)work;
@end
@interface Core3DCylindricalCutOperation ()
- (instancetype)initWithOwner:(Core3DViewController *)owner work:(const std::shared_ptr<core3d::NativeSolidWork>&)work;
@end
@implementation Core3DCylindricalCutOperation {
    __weak Core3DViewController *_owner;std::weak_ptr<core3d::NativeSolidWork> _work;
}
- (instancetype)initWithOwner:(Core3DViewController *)owner work:(const std::shared_ptr<core3d::NativeSolidWork>&)work {
    self=[super init];if(self){_owner=owner;_work=work;}return self;
}
- (BOOL)cancel {if(!NSThread.isMainThread)return NO;Core3DViewController *owner=_owner;const auto work=_work.lock();return owner&&work?[owner core3d_cancelCutWork:work]:NO;}
@end

@interface Core3DCylindricalCutBore ()
- (instancetype)initWithOperand:(const core3d::analytic_boolean::Operand&)operand effectiveMM:(double)effectiveMM;
@end
@implementation Core3DCylindricalCutBore
- (instancetype)initWithOperand:(const core3d::analytic_boolean::Operand&)operand effectiveMM:(double)effectiveMM {
    const double world=operand.radius*effectiveMM;
    const double x=operand.point[0]*effectiveMM,y=operand.point[1]*effectiveMM,z=operand.point[2]*effectiveMM;
    // Preserve the pre-existing native/manual admission exactly. Physical
    // coordinate projection is additive descriptive data; the AI boundary
    // rejects nonfinite or out-of-range projections before serialization.
    if(!operand.identifier||static_cast<unsigned>(operand.axis)>2||!std::isfinite(world)||world<.001||world>1e6)return nil;
    self=[super init];if(self){_operandIdentifier=operand.identifier;
        _axis=static_cast<Core3DCylindricalCutAxis>(operand.axis);
        _localX=operand.point[0];_localY=operand.point[1];_localZ=operand.point[2];
        _physicalXMM=x;_physicalYMM=y;_physicalZMM=z;_worldRadiusMM=world;}return self;
}
@end
@interface Core3DCylindricalCutProgramSnapshot ()
- (instancetype)initWithNative:(const core3d::CylindricalCutProgramSnapshot&)native owner:(Core3DViewController *)owner viewer:(const std::shared_ptr<core3d::Core3DViewer>&)viewer;
- (core3d::CylindricalCutProgramSnapshot)nativeSnapshot;
- (BOOL)matchesOwner:(Core3DViewController *)owner viewer:(const std::shared_ptr<core3d::Core3DViewer>&)viewer;
@end
@implementation Core3DCylindricalCutProgramSnapshot {
    core3d::CylindricalCutProgramSnapshot _native;
    __weak Core3DViewController *_owner;
    std::weak_ptr<core3d::Core3DViewer> _viewer;
}
- (instancetype)initWithNative:(const core3d::CylindricalCutProgramSnapshot&)native owner:(Core3DViewController *)owner viewer:(const std::shared_ptr<core3d::Core3DViewer>&)viewer {
    self=[super init];if(!self)return nil;
    // Descriptive bore values projected from the complete captured recipe.
    NSMutableArray<Core3DCylindricalCutBore *> *bores=[NSMutableArray array];
    if(const auto* legacy=std::get_if<core3d::retained_solid::Envelope>(&native.source.recipe)){
        core3d::analytic_boolean::Operand operand;operand.identifier=legacy->operandID;
        operand.axis=static_cast<core3d::analytic_boolean::Axis>(legacy->axis);
        operand.point=legacy->point;operand.radius=legacy->radius;
        Core3DCylindricalCutBore *bore=[[Core3DCylindricalCutBore alloc] initWithOperand:operand effectiveMM:native.source.effectiveMM];
        if(!bore)return nil;[bores addObject:bore];
    }else{
        for(const auto& step:std::get<core3d::retained_boolean::Program>(native.source.recipe).steps){
            Core3DCylindricalCutBore *bore=[[Core3DCylindricalCutBore alloc] initWithOperand:step.operand effectiveMM:native.source.effectiveMM];
            if(!bore)return nil;[bores addObject:bore];
        }
    }
    _native=native;_owner=owner;_viewer=viewer;
    _entityIdentifier=[NSString stringWithUTF8String:native.source.original.entityIdentifier.c_str()];
    _definitionIdentifier=[NSString stringWithUTF8String:native.source.original.definitionIdentifier.c_str()];
    _bores=bores;return self;
}
- (core3d::CylindricalCutProgramSnapshot)nativeSnapshot {return _native;}
- (Core3DSavedCutSourceValues *)sourceRecipeMM {
    if(!NSThread.isMainThread||!_native.source.original.retained.value)return nil;
    try {
        if(const auto* legacy=std::get_if<core3d::retained_solid::Envelope>(&_native.source.recipe))
            return [[Core3DSavedCutSourceValues alloc] initWithEnvelope:*legacy];
        const auto& program=std::get<core3d::retained_boolean::Program>(_native.source.recipe);
        // Empty patch of the matching family verifies this exact shared source
        // stays supported across EVERY operand of the complete recipe.
        core3d::saved_cut_source_edit::Patch empty;
        if(program.source.family==1)empty=core3d::saved_cut_source_values::PolygonPatch{};
        else if(program.source.family==2)empty=core3d::saved_cut_source_values::EnclosurePatch{};
        else return nil;
        if(!core3d::saved_boolean_build::SourcePatch(program,empty))return nil;
        return [[Core3DSavedCutSourceValues alloc] initWithProgramSource:program.source];
    }catch(...){return nil;}
}
- (BOOL)matchesOwner:(Core3DViewController *)owner viewer:(const std::shared_ptr<core3d::Core3DViewer>&)viewer {
    return NSThread.isMainThread&&owner&&_owner==owner&&viewer&&_viewer.lock()==viewer;
}
@end

@interface Core3DStoredRectangularLoftSnapshot ()
- (instancetype)initWithNativeSnapshot:(const core3d::StoredRectangularLoftSnapshot&)snapshot;
- (core3d::StoredRectangularLoftSnapshot)nativeSnapshot;
@end
@implementation Core3DStoredRectangularLoftSnapshot {
    core3d::StoredRectangularLoftSnapshot _native;
}
- (instancetype)initWithNativeSnapshot:(const core3d::StoredRectangularLoftSnapshot&)snapshot {
    self=[super init];if(self) {
        _definition=[[Core3DRectangularLoftDefinition alloc] initWithNativeDefinition:snapshot.definition];
        if(!_definition)return nil;
        _native=snapshot;
        _entityIdentifier=[[NSString alloc] initWithUTF8String:snapshot.identity.entityIdentifier.c_str()];
        _definitionIdentifier=[[NSString alloc] initWithUTF8String:snapshot.definitionIdentifier.c_str()];
        _featureIdentifier=[[NSString alloc] initWithUTF8String:snapshot.featureIdentifier.c_str()];
        _effectiveDimensionMetersPerUnit=snapshot.effectiveDimensionMetersPerUnit;_current=snapshot.current;
    }return self;
}
- (core3d::StoredRectangularLoftSnapshot)nativeSnapshot {return _native;}
@end

// Private owner dispatch keeps slot inspection inside the owning controller.
@interface Core3DViewController (StoredLoftOperationCancellation)
- (BOOL)core3d_cancelStoredLoftWork:(const std::shared_ptr<core3d::NativeSolidWork>&)work;
@end
@interface Core3DStoredLoftEditOperation ()
- (instancetype)initWithOwner:(Core3DViewController *)owner work:(const std::shared_ptr<core3d::NativeSolidWork>&)work;
@end
@implementation Core3DStoredLoftEditOperation {
    __weak Core3DViewController *_owner;
    std::weak_ptr<core3d::NativeSolidWork> _work;
}
- (instancetype)initWithOwner:(Core3DViewController *)owner work:(const std::shared_ptr<core3d::NativeSolidWork>&)work {
    self=[super init];if(self){_owner=owner;_work=work;}return self;
}
- (BOOL)cancel {
    if(![NSThread isMainThread])return NO;
    Core3DViewController *owner=_owner;
    const auto work=_work.lock();
    return owner && work ? [owner core3d_cancelStoredLoftWork:work] : NO;
}
@end

@interface Core3DStoredSweepSnapshot ()
- (instancetype)initWithNativeSnapshot:(const core3d::StoredSweepSnapshot&)snapshot;
- (core3d::StoredSweepSnapshot)nativeSnapshot;
@end
@implementation Core3DStoredSweepSnapshot {
    core3d::StoredSweepSnapshot _native;
}
- (instancetype)initWithNativeSnapshot:(const core3d::StoredSweepSnapshot&)snapshot {
    self=[super init];
    if (self) {
        const auto& d=snapshot.definition;
        NSMutableArray<Core3DProfileCurveVertex *> *vertices=[NSMutableArray array];
        NSMutableArray<Core3DSweepPathSegment *> *segments=[NSMutableArray array];
        NSMutableArray<NSNumber *> *frame=[NSMutableArray array];
        for (const auto& v:d.vertices) {
            auto value=[[Core3DProfileCurveVertex alloc] initWithIdentifier:v.identifier point:CGPointMake(v.point.X(),v.point.Y())];
            if (!value) return nil;[vertices addObject:value];
        }
        for (const auto& s:d.segments) {
            auto value=[[Core3DSweepPathSegment alloc] initWithIdentifier:s.identifier startVertex:s.startVertex endVertex:s.endVertex
                kind:static_cast<Core3DProfileCurveKind>(s.kind) center:CGPointMake(s.center.X(),s.center.Y()) radius:s.radius
                startDegrees:s.startDegrees sweepDegrees:s.sweepDegrees];
            if(!value)return nil;[segments addObject:value];
        }
        if (d.constructionFrame) for (double v:d.constructionFrame->values) [frame addObject:@(v)];
        _definition=[[Core3DSweepDefinition alloc] initWithPathIdentifier:d.pathIdentifier vertices:vertices segments:segments
            plane:static_cast<Core3DProfilePlane>(d.plane) radius:d.radius metersPerUnit:d.dimensionMetersPerUnit constructionFrameValues:frame];
        if (!_definition) return nil;
        _native=snapshot;
        _entityIdentifier=[[NSString alloc] initWithUTF8String:snapshot.identity.entityIdentifier.c_str()];
        _definitionIdentifier=[[NSString alloc] initWithUTF8String:snapshot.definitionIdentifier.c_str()];
        _featureIdentifier=[[NSString alloc] initWithUTF8String:snapshot.featureIdentifier.c_str()];
        _effectiveDimensionMetersPerUnit=snapshot.effectiveDimensionMetersPerUnit;_current=snapshot.current;
    }
    return self;
}
- (core3d::StoredSweepSnapshot)nativeSnapshot {return _native;}
@end

@interface Core3DProfileDefinition ()
- (instancetype)initWithNativeParameters:(const core3d::profile::Parameters&)parameters;
- (core3d::profile::Parameters)nativeParameters;
@end
@implementation Core3DProfileDefinition {
    core3d::profile::Parameters _parameters;
}
- (instancetype)initWithNativeParameters:(const core3d::profile::Parameters&)parameters {
    self = [super init];
    if (self) {
        _parameters = parameters;
        const auto& d = parameters.definition;
        _curveInner = @[];
        if (d.curves) {
            _curveOuter = Core3DPublicCurveLoop(d.curves->outer);
            if (!_curveOuter) return nil;
            NSMutableArray<Core3DProfileCurveLoop *> *inner = [NSMutableArray arrayWithCapacity:d.curves->inner.size()];
            for (const auto& loop : d.curves->inner) {
                auto value = Core3DPublicCurveLoop(loop);
                if (!value) return nil;
                [inner addObject:value];
            }
            _curveInner = [inner copy];
        }
        NSMutableArray *points = [NSMutableArray arrayWithCapacity:d.points.size()];
        for (const auto& p : d.points) [points addObject:[NSValue valueWithCGPoint:CGPointMake(p.X(),p.Y())]];
        _points = [points copy];
        if (d.circle) _circleCenter = [NSValue valueWithCGPoint:CGPointMake(d.circle->center.X(),d.circle->center.Y())];
        NSMutableArray *centers = [NSMutableArray arrayWithCapacity:d.holes.size()];
        NSMutableArray *radii = [NSMutableArray arrayWithCapacity:d.holes.size()];
        for (const auto& h : d.holes) {
            [centers addObject:[NSValue valueWithCGPoint:CGPointMake(h.center.X(),h.center.Y())]];
            [radii addObject:@(h.radius)];
        }
        _holeCenters = [centers copy]; _holeRadii = [radii copy];
    }
    return self;
}
- (instancetype)initWithPoints:(NSArray<NSValue *> *)points
    circleCenter:(NSValue *)circleCenter outerRadius:(double)outerRadius innerRadius:(double)innerRadius
    holeCenters:(NSArray<NSValue *> *)holeCenters holeRadii:(NSArray<NSNumber *> *)holeRadii
    plane:(Core3DProfilePlane)plane parameter:(double)parameter revolve:(BOOL)revolve metersPerUnit:(double)metersPerUnit {
    if (![points isKindOfClass:[NSArray class]] || points.count > 64
        || ![holeCenters isKindOfClass:[NSArray class]] || ![holeRadii isKindOfClass:[NSArray class]]
        || holeCenters.count > 16 || holeCenters.count != holeRadii.count) return nil;
    try {
        core3d::profile::Parameters parameters;
        parameters.metersPerUnit = metersPerUnit;
        auto& d = parameters.definition;
        d.plane = static_cast<int>(plane); d.depth = parameter; d.revolve = revolve;
        auto pointValue = [](id value, gp_Pnt2d& output) {
            if (![value isKindOfClass:[NSValue class]] || std::strcmp([value objCType], @encode(CGPoint)) != 0) return false;
            const CGPoint p = [value CGPointValue];
            if (!std::isfinite(p.x) || !std::isfinite(p.y)) return false;
            output = gp_Pnt2d(p.x,p.y); return true;
        };
        for (id value in points) {
            gp_Pnt2d p; if (!pointValue(value,p)) return nil; d.points.push_back(p);
        }
        if (circleCenter) {
            gp_Pnt2d p; if (!pointValue(circleCenter,p)) return nil;
            d.circle = core3d::ProfileCircularSection{p,outerRadius,innerRadius};
        } else if (outerRadius != 0 || innerRadius != 0) return nil;
        for (NSUInteger i=0; i<holeCenters.count; ++i) {
            gp_Pnt2d p; if (!pointValue(holeCenters[i],p) || ![holeRadii[i] isKindOfClass:[NSNumber class]]) return nil;
            d.holes.push_back({p,[holeRadii[i] doubleValue]});
        }
        std::vector<double> values;
        if (!core3d::profile::Encode(parameters,values)) return nil;
        return [self initWithNativeParameters:parameters];
    } catch (...) { return nil; }
}
- (instancetype)initWithCurveOuter:(Core3DProfileCurveLoop *)outer
    inner:(NSArray<Core3DProfileCurveLoop *> *)inner plane:(Core3DProfilePlane)plane
    parameter:(double)parameter revolve:(BOOL)revolve metersPerUnit:(double)metersPerUnit {
    if (![outer isMemberOfClass:[Core3DProfileCurveLoop class]]
        || ![inner isKindOfClass:[NSArray class]] || inner.count > 16) return nil;
    NSArray *ownedInner = [inner copy];
    try {
        std::size_t count = outer.vertices.count;
        for (id value in ownedInner) {
            if (![value isMemberOfClass:[Core3DProfileCurveLoop class]]) return nil;
            Core3DProfileCurveLoop *loop = value;
            if (loop.vertices.count > 512 - count) return nil;
            count += loop.vertices.count;
        }
        core3d::profile::Parameters parameters;
        parameters.metersPerUnit = metersPerUnit;
        auto& definition = parameters.definition;
        definition.plane = static_cast<int>(plane); definition.depth = parameter; definition.revolve = revolve;
        definition.curves.emplace(); definition.curves->outer = [outer nativeLoop];
        definition.curves->inner.reserve(ownedInner.count);
        for (Core3DProfileCurveLoop *loop in ownedInner)
            definition.curves->inner.push_back([loop nativeLoop]);
        std::vector<double> values;
        if (!core3d::profile::Encode(parameters,values)) return nil;
        return [self initWithNativeParameters:parameters];
    } catch (...) { return nil; }
}
- (Core3DProfileDefinition *)definitionByChangingParameter:(double)parameter {
    try {
        auto requested = _parameters;
        requested.definition.depth = parameter;
        std::vector<double> encoded;
        if (!core3d::profile::Encode(requested,encoded)) return nil;
        return [[Core3DProfileDefinition alloc] initWithNativeParameters:requested];
    } catch (...) { return nil; }
}
- (core3d::profile::Parameters)nativeParameters { return _parameters; }
- (double)outerRadius { return _parameters.definition.circle ? _parameters.definition.circle->outerRadius : 0; }
- (double)innerRadius { return _parameters.definition.circle ? _parameters.definition.circle->innerRadius : 0; }
- (Core3DProfilePlane)plane { return static_cast<Core3DProfilePlane>(_parameters.definition.plane); }
- (double)parameter { return _parameters.definition.depth; }
- (BOOL)revolve { return _parameters.definition.revolve; }
- (double)metersPerUnit { return _parameters.metersPerUnit; }
- (NSArray<NSNumber *> *)constructionFrameValues {
    if (!_parameters.constructionFrame) return @[];
    NSMutableArray<NSNumber *> *values = [NSMutableArray arrayWithCapacity:8];
    for (double value : _parameters.constructionFrame->values) [values addObject:@(value)];
    return [values copy];
}
@end

@interface Core3DEnclosureDefinition ()
- (instancetype)initWithNativeParameters:(const core3d::enclosure::Parameters&)parameters;
- (core3d::enclosure::Parameters)nativeParameters;
@end
@implementation Core3DEnclosureDefinition {
    core3d::enclosure::Parameters _parameters;
}
- (instancetype)initWithNativeParameters:(const core3d::enclosure::Parameters&)parameters {
    try {
        std::vector<double> values;
        if (!core3d::enclosure::Encode(parameters,values)) return nil;
        self = [super init];
        if (self) _parameters = parameters;
        return self;
    } catch (...) { return nil; }
}
- (instancetype)initWithWidth:(double)width depth:(double)depth height:(double)height
    wall:(double)wall floor:(double)floor cornerRadius:(double)cornerRadius
    plane:(Core3DProfilePlane)plane metersPerUnit:(double)metersPerUnit {
    // Check NSInteger before narrowing to the native int/enum representation.
    if (plane < Core3DProfilePlaneXY || plane > Core3DProfilePlaneYZ) return nil;
    core3d::enclosure::Parameters parameters;
    parameters.definition.dimensions = {width,depth,height,wall,floor,cornerRadius};
    parameters.definition.plane = static_cast<int>(plane);
    parameters.metersPerUnit = metersPerUnit;
    return [self initWithNativeParameters:parameters];
}
- (Core3DEnclosureDefinition *)definitionByUpdatingDimension:(Core3DEnclosureDimension)dimension value:(double)value {
    if (dimension < Core3DEnclosureDimensionWidth || dimension > Core3DEnclosureDimensionCornerRadius) return nil;
    try {
        const auto updated = core3d::UpdatedEnclosureDimension(_parameters.definition,
            static_cast<core3d::EnclosureDimension>(dimension),value);
        if (!updated) return nil;
        auto parameters = _parameters; parameters.definition = *updated;
        return [[Core3DEnclosureDefinition alloc] initWithNativeParameters:parameters];
    } catch (...) { return nil; }
}
- (Core3DEnclosureDefinition *)definitionByUpdatingWidth:(double)width depth:(double)depth
    height:(double)height wall:(double)wall floor:(double)floor cornerRadius:(double)cornerRadius
    plane:(Core3DProfilePlane)plane {
    if (plane < Core3DProfilePlaneXY || plane > Core3DProfilePlaneYZ) return nil;
    try {
        auto parameters = _parameters;
        parameters.definition.dimensions = {width,depth,height,wall,floor,cornerRadius};
        parameters.definition.plane = static_cast<int>(plane);
        return [[Core3DEnclosureDefinition alloc] initWithNativeParameters:parameters];
    } catch (...) { return nil; }
}
- (double)width { return _parameters.definition.dimensions.width; }
- (double)depth { return _parameters.definition.dimensions.depth; }
- (double)height { return _parameters.definition.dimensions.height; }
- (double)wall { return _parameters.definition.dimensions.wall; }
- (double)floor { return _parameters.definition.dimensions.floor; }
- (double)cornerRadius { return _parameters.definition.dimensions.cornerRadius; }
- (Core3DProfilePlane)plane { return static_cast<Core3DProfilePlane>(_parameters.definition.plane); }
- (double)metersPerUnit { return _parameters.metersPerUnit; }
- (core3d::enclosure::Parameters)nativeParameters { return _parameters; }
@end

@interface Core3DStoredEnclosureSnapshot ()
- (instancetype)initWithNativeSnapshot:(const core3d::StoredEnclosureSnapshot&)snapshot;
- (core3d::StoredEnclosureSnapshot)nativeSnapshot;
@end
@implementation Core3DStoredEnclosureSnapshot {
    core3d::StoredEnclosureSnapshot _native;
}
- (instancetype)initWithNativeSnapshot:(const core3d::StoredEnclosureSnapshot&)snapshot {
    self = [super init];
    if (self) {
        _native = snapshot;
        _entityIdentifier = [[NSString alloc] initWithUTF8String:snapshot.identity.entityIdentifier.c_str()];
        _definitionIdentifier = [[NSString alloc] initWithUTF8String:snapshot.definitionIdentifier.c_str()];
        _featureIdentifier = [[NSString alloc] initWithUTF8String:snapshot.featureIdentifier.c_str()];
        _definition = [[Core3DEnclosureDefinition alloc] initWithNativeParameters:snapshot.parameters];
        _dimensionMetersPerUnit = snapshot.dimensionMetersPerUnit;
        _current = snapshot.current;
    }
    return self;
}
- (core3d::StoredEnclosureSnapshot)nativeSnapshot { return _native; }
@end

@interface Core3DStoredProfileSnapshot ()
- (instancetype)initWithNativeSnapshot:(const core3d::StoredProfileSnapshot&)snapshot;
- (core3d::StoredProfileSnapshot)nativeSnapshot;
@end
@implementation Core3DStoredProfileSnapshot {
    core3d::StoredProfileSnapshot _native;
}
- (instancetype)initWithNativeSnapshot:(const core3d::StoredProfileSnapshot&)snapshot {
    self = [super init];
    if (self) {
        _native = snapshot;
        _entityIdentifier = [[NSString alloc] initWithUTF8String:snapshot.identity.entityIdentifier.c_str()];
        _definitionIdentifier = [[NSString alloc] initWithUTF8String:snapshot.definitionIdentifier.c_str()];
        _featureIdentifier = [[NSString alloc] initWithUTF8String:snapshot.featureIdentifier.c_str()];
        _definition = [[Core3DProfileDefinition alloc] initWithNativeParameters:snapshot.parameters];
        _dimensionMetersPerUnit = snapshot.dimensionMetersPerUnit;
        _current = snapshot.current;
    }
    return self;
}
- (core3d::StoredProfileSnapshot)nativeSnapshot { return _native; }
@end

@interface Core3DMeshVertexEditSnapshot ()
@property(nonatomic,copy,readonly) NSString *sessionIdentifier;
- (instancetype)initWithNativeSnapshot:(const core3d::MeshVertexEditSnapshot&)snapshot;
@end
@implementation Core3DMeshVertexEditSnapshot
- (instancetype)initWithNativeSnapshot:(const core3d::MeshVertexEditSnapshot&)snapshot {
    self=[super init];
    if(self) {
        _sessionIdentifier=[[NSString alloc] initWithBytes:snapshot.sessionIdentifier.data()
            length:snapshot.sessionIdentifier.size() encoding:NSUTF8StringEncoding];
        _entityIdentifier=[[NSString alloc] initWithBytes:snapshot.entityIdentifier.data()
            length:snapshot.entityIdentifier.size() encoding:NSUTF8StringEncoding];
        NSMutableArray *positions=[NSMutableArray arrayWithCapacity:snapshot.worldVertices.size()];
        for(const auto& point:snapshot.worldVertices)[positions addObject:@[@(point[0]),@(point[1]),@(point[2])]];
        _worldVertices=[positions copy];
        _elementKind=static_cast<Core3DMeshElementKind>(snapshot.elementKind);
        NSMutableArray *edges=[NSMutableArray arrayWithCapacity:snapshot.edgeVertices.size()];
        for(const auto& edge:snapshot.edgeVertices)[edges addObject:@[@(edge[0]),@(edge[1])]];
        _edgeVertexIndices=[edges copy];
        NSMutableArray *triangles=[NSMutableArray arrayWithCapacity:snapshot.triangleVertices.size()];
        for(const auto& triangle:snapshot.triangleVertices)[triangles addObject:@[@(triangle[0]),@(triangle[1]),@(triangle[2])]];
        _triangleVertexIndices=[triangles copy];
    }
    return self;
}
@end

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

// Main-thread-owned callback storage. Client callbacks can retain native
// context snapshots, so they must never be part of the geometry worker's
// capture graph. Only an immutable completion UUID crosses that boundary.
static NSMutableDictionary<NSUUID *, id> *Core3DNativeSolidCompletionRegistry() {
    static NSMutableDictionary<NSUUID *, id> *callbacks;
    static dispatch_once_t once;
    dispatch_once(&once, ^{ callbacks = [NSMutableDictionary dictionary]; });
    return callbacks;
}

static NSUUID *Core3DRegisterNativeSolidCompletion(void (^completion)(Core3DProfileConstructionResult)) {
    if (![NSThread isMainThread] || !completion) return nil;
    NSMutableDictionary<NSUUID *, id> *callbacks = Core3DNativeSolidCompletionRegistry();
    if (callbacks.count >= 32) return nil;
    NSUUID *token = [NSUUID UUID];
    callbacks[token] = [completion copy];
    return token;
}

static void Core3DDeliverNativeSolidCompletion(NSUUID *token, Core3DProfileConstructionResult result) {
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ Core3DDeliverNativeSolidCompletion(token, result); });
        return;
    }
    NSMutableDictionary<NSUUID *, id> *callbacks = Core3DNativeSolidCompletionRegistry();
    void (^completion)(Core3DProfileConstructionResult) = callbacks[token];
    [callbacks removeObjectForKey:token]; // Remove before invoking reentrant user code.
    if (completion) completion(result);
}

#if DEBUG
// Test closures remain on main, keyed by the same immutable completion token.
// The actual utility worker still carries no client callback or live owner.
static NSMutableDictionary<NSUUID *, id> *Core3DNativeSolidGeometryDeliveryGates() {
    static NSMutableDictionary<NSUUID *, id> *gates;
    static dispatch_once_t once;
    dispatch_once(&once, ^{ gates=[NSMutableDictionary dictionary]; });
    return gates;
}
static void Core3DGateNativeSolidGeometryDelivery(NSUUID *token,dispatch_block_t delivery) {
    NSCAssert(NSThread.isMainThread,@"Geometry delivery gate is main-owned");
    auto gates=Core3DNativeSolidGeometryDeliveryGates();
    void (^gate)(void (^)(void))=gates[token];
    [gates removeObjectForKey:token];
    if(!gate){delivery();return;}
    // Clear the exact pending delivery before admission or any reentrant call.
    // Repeated/off-main resumes cannot promote an owner or consume newer work.
    __block dispatch_block_t pending=[delivery copy];
    gate(^{
        dispatch_block_t resume=^{
            dispatch_block_t once=pending;pending=nil;
            if(once)once();
        };
        if(NSThread.isMainThread)resume();
        else dispatch_async(dispatch_get_main_queue(),resume);
    });
}
#endif

@interface Core3DAssemblyPartDefinition ()
- (instancetype)initWithKind:(Core3DAssemblyPartKind)kind name:(NSString *)name
    width:(double)width depth:(double)depth height:(double)height radius:(double)radius
    position:(simd_double3)position rotation:(simd_double4)rotation;
- (std::optional<core3d::AssemblyPartDefinition>)nativePartForMetersPerUnit:(double)unit;
@end
@implementation Core3DAssemblyPartDefinition
- (instancetype)initWithKind:(Core3DAssemblyPartKind)kind name:(NSString *)name
    width:(double)width depth:(double)depth height:(double)height radius:(double)radius
    position:(simd_double3)position rotation:(simd_double4)rotation {
    if (![name isKindOfClass:NSString.class] || name.length==0 || name.length>64) return nil;
    TCollection_ExtendedString nativeName;
    for (NSUInteger i=0;i<name.length;++i) nativeName+=static_cast<Standard_ExtCharacter>([name characterAtIndex:i]);
    if (!OcctObjectNameIsValid(nativeName)) return nil;
    const auto length=[](double x){return std::isfinite(x)&&x>=1e-3&&x<=1e6;};
    if (!length(height) || (kind==Core3DAssemblyPartKindBox ? (!length(width)||!length(depth)) : !length(radius))) return nil;
    core3d::profile::ConstructionFrame frame;
    frame.values={position.x,position.y,position.z,rotation.x,rotation.y,rotation.z,rotation.w,1};
    if (!frame.IsValid()) return nil;
    if ((self=[super init])) {
        _kind=kind;_name=[name copy];_partIdentifier=NSUUID.UUID.UUIDString;
        _widthMM=width;_depthMM=depth;_heightMM=height;_radiusMM=radius;
        _positionMM=position;_rotationXYZW=rotation;
        if (!_partIdentifier) return nil;
    }
    return self;
}
+ (instancetype)boxWithName:(NSString *)name widthMM:(double)width depthMM:(double)depth heightMM:(double)height
    positionMM:(simd_double3)position rotationXYZW:(simd_double4)rotation {
    return [[self alloc] initWithKind:Core3DAssemblyPartKindBox name:name width:width depth:depth height:height radius:0 position:position rotation:rotation];
}
+ (instancetype)cylinderWithName:(NSString *)name radiusMM:(double)radius heightMM:(double)height
    positionMM:(simd_double3)position rotationXYZW:(simd_double4)rotation {
    return [[self alloc] initWithKind:Core3DAssemblyPartKindCylinder name:name width:0 depth:0 height:height radius:radius position:position rotation:rotation];
}
- (std::optional<core3d::AssemblyPartDefinition>)nativePartForMetersPerUnit:(double)unit {
    if (!std::isfinite(unit)||unit<=0) return {};
    try {
        const double scale=.001/unit;
        core3d::AssemblyPartDefinition part;
        part.identifier=_partIdentifier.UTF8String;
        for (NSUInteger i=0;i<_name.length;++i) part.name+=static_cast<Standard_ExtCharacter>([_name characterAtIndex:i]);
        auto& p=part.parameters;p.metersPerUnit=unit;p.definition.plane=0;p.definition.depth=_heightMM*scale;
        if (_kind==Core3DAssemblyPartKindBox) p.definition.points={{0,0},{_widthMM*scale,0},{_widthMM*scale,_depthMM*scale},{0,_depthMM*scale}};
        else p.definition.circle=core3d::ProfileCircularSection{gp_Pnt2d(0,0),_radiusMM*scale,0};
        core3d::profile::ConstructionFrame frame;
        frame.values={_positionMM.x*scale,_positionMM.y*scale,_positionMM.z*scale,
            _rotationXYZW.x,_rotationXYZW.y,_rotationXYZW.z,_rotationXYZW.w,1};
        p.constructionFrame=frame;
        std::vector<double> encoded;
        if (!core3d::profile::Encode(p,encoded)) return {};
        return part;
    } catch (...) {return {};}
}
@end

// Descriptive AI subset only. Arbitrary profiles keep the full manual path.
// Exact canonical coordinates avoid reinterpreting an offset/concave outline.
// Kind is private metadata, never authority or a renderer-derived classification.
static unsigned Core3DCanonicalModelingProfileKind(const core3d::profile::Parameters& parameters) noexcept {
    const auto& d = parameters.definition;
    if (d.revolve || d.curves || !d.holes.empty() || d.plane < 0 || d.plane > 2
        || !std::isfinite(d.depth) || d.depth <= 0) return 0;
    if (d.circle) {
        const auto& c = *d.circle;
        return d.points.empty() && c.center.X() == 0 && c.center.Y() == 0
            && c.innerRadius == 0 && std::isfinite(c.outerRadius) && c.outerRadius > 0 ? 2 : 0;
    }
    if (d.points.size() != 4) return 0;
    const auto& p = d.points;
    const double width=p[1].X(), depth=p[2].Y();
    return std::isfinite(width) && width > 0 && std::isfinite(depth) && depth > 0
        && p[0].X() == 0 && p[0].Y() == 0 && p[1].Y() == 0
        && p[2].X() == width && p[3].X() == 0 && p[3].Y() == depth ? 1 : 0;
}

// This is a provider-description bound, not a new geometry validator. Current
// saved features already passed native profile admission; actual rebuild still
// uses that admission. Do not reorder connectivity to make a recipe fit JSON.
static bool Core3DModelingProfileRecipeSupported(const core3d::profile::Parameters& parameters,
    double dimensionMetersPerUnit) noexcept {
    try {
        const auto& d=parameters.definition;
        if (!std::isfinite(dimensionMetersPerUnit) || dimensionMetersPerUnit<=0
            || !std::isfinite(parameters.metersPerUnit) || parameters.metersPerUnit<=0
            || d.plane<0 || d.plane>2 || d.circle) return false;
        const double millimetersPerUnit=dimensionMetersPerUnit/0.001;
        const auto mm=[&](double value) {return value*millimetersPerUnit;};
        const auto coordinate=[&](double value) {const double physical=mm(value);
            return std::isfinite(physical) && std::abs(physical)<=1e6;};
        const auto length=[&](double value) {const double physical=mm(value);
            return std::isfinite(physical) && physical>=0.001 && physical<=1e6;};
        const auto point=[&](const gp_Pnt2d& value) {return coordinate(value.X()) && coordinate(value.Y());};
        if (d.revolve) {
            if (!std::isfinite(d.depth) || d.depth<0.001 || d.depth>360) return false;
        } else if (!length(d.depth)) return false;
        if (!d.curves) {
            if (d.revolve || d.points.size()<3 || d.points.size()>64 || d.holes.size()>16) return false;
            for (const auto& value:d.points) if (!point(value)) return false;
            for (const auto& hole:d.holes) if (!point(hole.center) || !length(hole.radius)) return false;
            return true;
        }
        if (!d.points.empty() || !d.holes.empty() || d.curves->inner.size()>4) return false;
        std::size_t total=0;
        const auto loop=[&](const core3d::ProfileCurveLoop& value) {
            const std::size_t count=value.vertices.size();
            if (count<2 || count>64-total || value.segments.size()!=count) return false;
            total+=count;
            for (std::size_t i=0;i<count;++i) {
                const auto& vertex=value.vertices[i];const auto& edge=value.segments[i];
                if (!point(vertex.point) || edge.startVertex!=vertex.identifier
                    || edge.endVertex!=value.vertices[(i+1)%count].identifier) return false;
                if (edge.kind==core3d::ProfileCurveKind::Line) {
                    if (edge.center.X()!=0 || edge.center.Y()!=0 || edge.radius!=0
                        || edge.startDegrees!=0 || edge.sweepDegrees!=0) return false;
                } else if (edge.kind==core3d::ProfileCurveKind::CircularArc) {
                    if (!point(edge.center) || !length(edge.radius) || !std::isfinite(edge.startDegrees)
                        || std::abs(edge.startDegrees)>360 || !std::isfinite(edge.sweepDegrees)
                        || std::abs(edge.sweepDegrees)<0.001 || std::abs(edge.sweepDegrees)>=360) return false;
                } else return false;
            }
            return true;
        };
        if (!loop(d.curves->outer)) return false;
        for (const auto& inner:d.curves->inner) if (!loop(inner)) return false;
        return true;
    } catch (...) {return false;}
}

// Descriptive feature-local physical wire limits. Native path/kernel admission
// and the stored snapshot's full-root metadata/source guard remain authoritative.
static bool Core3DModelingSweepSupported(const core3d::planar_sweep::Definition& d,double effectiveUnit) {
    try {
        core3d::planar_sweep::Inspection inspected;
        if(core3d::planar_sweep::Inspect(d,inspected)!=core3d::planar_sweep::Admission::Accepted
            ||!std::isfinite(effectiveUnit)||effectiveUnit<=0)return false;
        const double mm=effectiveUnit*1000;
        if(!std::isfinite(mm)||mm<=0)return false;
        const auto scalar=[&](double v){return std::isfinite(v*mm)&&std::abs(v*mm)<=1e6;};
        const auto length=[&](double v){return scalar(v)&&v*mm>=0.001;};
        if(!length(d.radius))return false;
        for(const auto& v:d.vertices)if(!scalar(v.point.X())||!scalar(v.point.Y()))return false;
        for(const auto& edge:d.segments)if(edge.kind==core3d::ProfileCurveKind::CircularArc
            &&(!scalar(edge.center.X())||!scalar(edge.center.Y())||!length(edge.radius)))return false;
        return true;
    }catch(...){return false;}
}

// Bounded physical description of the existing native ruled-loft recipe.
// Inspect retains exact schema/frame/order/correspondence and kernel limits;
// this adds only feature-local physical bounds for a later provider contract.
static bool Core3DModelingLoftSupported(const core3d::rectangular_loft::Definition& d,double effectiveUnit) {
    core3d::rectangular_loft::Inspection inspected;
    if(core3d::rectangular_loft::Inspect(d,inspected)!=core3d::rectangular_loft::Admission::Accepted
        ||!std::isfinite(effectiveUnit)||effectiveUnit<=0)return false;
    const double mm=effectiveUnit*1000;
    if(!std::isfinite(mm)||mm<=0)return false;
    const auto coordinate=[&](double v){return std::isfinite(v*mm)&&std::abs(v*mm)<=1e6;};
    const auto length=[&](double v){return coordinate(v)&&v*mm>=0.001;};
    for(std::size_t i=0;i<d.stations.size();++i){const auto& station=d.stations[i];
        if(!coordinate(station.z)||!coordinate(station.centerX)||!coordinate(station.centerY)
            ||!length(station.width)||!length(station.depth)
            ||(i&&!length(station.z-d.stations[i-1].z)))return false;
    }
    return true;
}

@interface Core3DRigidPlacementPreparation () {
@public
    __weak Core3DViewController *_placementOwner;
    std::weak_ptr<core3d::Core3DViewer> _placementViewer;
    core3d::authority::Stamp _placementStamp;
    Core3DTransformInspectorSnapshot *_placementSnapshot;
    core3d::placement::Evidence _placementEvidence;
    double _placementRawValue;
    Core3DRigidPlacementKind _placementKind;
    Core3DTransformInspectorAxis _placementAxis;
    BOOL _placementConsumed;
}
- (instancetype)initPrivate;
@end
@implementation Core3DRigidPlacementPreparation
- (instancetype)initPrivate { return [super init]; }
@end

@class Core3DSavedCutSourceJob;
@interface Core3DModelingPlanningContext () {
@public
    __weak Core3DViewController *_planningOwner;
    std::weak_ptr<core3d::Core3DViewer> _planningViewer;
    core3d::authority::Stamp _planningStamp;
    uint64_t _planningOverlayRevision;
    Core3DTransformInspectorSnapshot *_planningPlacementSnapshot;
    std::optional<core3d::placement::Evidence> _planningPlacementEvidence;
    Core3DCylindricalCutSnapshot *_planningSavedCutSource;
    Core3DCylindricalCutProgramSnapshot *_planningSavedCutProgramSource;
    Core3DSavedCutSourceValues *_planningSavedCutValues;
    __weak Core3DSavedCutSourceJob *_planningSavedCutJob;
    BOOL _planningConsumed;
    BOOL _planningRetired;
}
- (instancetype)initWithScene:(Core3DSceneSnapshot *)scene
    documentIdentifier:(NSString *)documentIdentifier
    enclosure:(Core3DStoredEnclosureSnapshot *)enclosure
    profile:(Core3DStoredProfileSnapshot *)profile
    recipe:(Core3DStoredProfileSnapshot *)recipe
    sweep:(Core3DStoredSweepSnapshot *)sweep
    loft:(Core3DStoredRectangularLoftSnapshot *)loft;
@end
@implementation Core3DModelingPlanningContext
- (instancetype)initWithScene:(Core3DSceneSnapshot *)scene
    documentIdentifier:(NSString *)documentIdentifier
    enclosure:(Core3DStoredEnclosureSnapshot *)enclosure
    profile:(Core3DStoredProfileSnapshot *)profile
    recipe:(Core3DStoredProfileSnapshot *)recipe
    sweep:(Core3DStoredSweepSnapshot *)sweep
    loft:(Core3DStoredRectangularLoftSnapshot *)loft {
    if ((self = [super init])) {
        _scene = scene; _documentIdentifier = [documentIdentifier copy];
        _selectedEnclosure = enclosure; _selectedProfile = profile; _selectedProfileRecipe = recipe; _selectedSweep = sweep; _selectedLoft = loft;
    }
    return self;
}
- (Core3DTransformInspectorSnapshot *)placementSnapshot { return _planningPlacementSnapshot; }
- (Core3DCylindricalCutSnapshot *)selectedSavedCutSource { return _planningSavedCutSource; }
- (Core3DCylindricalCutProgramSnapshot *)selectedSavedCutProgramSource { return _planningSavedCutProgramSource; }
- (Core3DSavedCutSourceValues *)selectedSavedCutSourceRecipeMM { return _planningSavedCutValues; }
@end

#if DEBUG
static NSDictionary *Core3DReceiptEffectFields(const core3d::receipt::Effect& effect) {
    namespace r = core3d::receipt;
    auto hex=[](const auto& bytes) { return [NSString stringWithUTF8String:r::Hex(bytes).c_str()]; };
    return @{@"feature":@(static_cast<unsigned>(effect.feature)),@"entityHex":hex(effect.entity),
        @"definitionHex":hex(effect.definition),@"featureIDHex":hex(effect.featureID),
        @"geometrySHA":hex(effect.geometry),@"stateSHA":hex(effect.state)};
}
static NSDictionary *Core3DReceiptEffectDiagnostic(const Handle(OcctDocument)& owner,
    const core3d::receipt::Effect& expected,
    const core3d::receipt::DebugEffectCapture *issuedCapture=nullptr) {
    namespace r = core3d::receipt;
    r::Effect live; r::DebugEffectCapture capture; bool captured=false; unsigned matches=0;
    if(issuedCapture){capture=*issuedCapture;live=expected;captured=true;matches=1;}
    else if (!owner.IsNull() && !owner->Document().IsNull()) {
        TDF_LabelSequence roots; XCAFDoc_DocumentTool::ShapeTool(owner->Document()->Main())->GetFreeShapes(roots);
        TDF_Label label;
        if (roots.Length()<=50000) for(int i=1;i<=roots.Length();++i) {
            r::UUID id;
            if(r::ParseUUID(owner->EntityIdentifierForLabel(roots.Value(i)),id)&&id==expected.entity) {
                label=roots.Value(i);++matches;
            }
        }
        if(matches==1)captured=r::CaptureEffect(owner,label,live,&capture);
    }
    auto base64=[](const std::vector<std::uint8_t>& bytes) {
        return [[NSData dataWithBytes:bytes.data() length:bytes.size()] base64EncodedStringWithOptions:0];
    };
    return @{@"expected":Core3DReceiptEffectFields(expected),@"live":Core3DReceiptEffectFields(live),
        @"captured":@(captured),@"matches":@(matches),@"stage":[NSString stringWithUTF8String:capture.stage],
        @"matchesExpected":@(captured&&live==expected),@"geometryBase64":base64(capture.geometryBytes),
        @"stateBase64":base64(capture.stateBytes)};
}
// DEBUG-only preservation oracle. Reads existing native labels and values;
// no tool is lazily created. Allocation counter values are intentionally not
// compared: adding a shape may advance them; their GUID/type must survive.
static NSDictionary *Core3DReceiptPreservation(const Handle(OcctDocument)& owner) {
    if (owner.IsNull() || owner->Document().IsNull()) return nil;
    const auto doc = owner->Document();
    if (!XCAFDoc_DocumentTool::CheckShapeTool(doc->Main())
        || !XCAFDoc_DocumentTool::CheckColorTool(doc->Main())
        || !XCAFDoc_DocumentTool::CheckVisMaterialTool(doc->Main())) return nil;
    auto entry = [](const TDF_Label& label) -> NSString * {
        TCollection_AsciiString text; TDF_Tool::Entry(label,text);
        return [NSString stringWithUTF8String:text.ToCString()];
    };
    auto utf16 = [](const TCollection_ExtendedString& value) -> NSArray * {
        NSMutableArray *units = [NSMutableArray array];
        for (int i=1;i<=value.Length();++i) [units addObject:@(std::uint16_t(value.Value(i)))];
        return units;
    };
    NSMutableArray *infrastructure = [NSMutableArray array];
    const std::array<TDF_Label,4> labels = {doc->Main(),
        XCAFDoc_DocumentTool::ShapeTool(doc->Main())->Label(),
        XCAFDoc_DocumentTool::ColorTool(doc->Main())->Label(),
        XCAFDoc_DocumentTool::VisMaterialTool(doc->Main())->Label()};
    for (const auto& label:labels) {
        NSMutableDictionary *attributes = [NSMutableDictionary dictionary];
        for (TDF_AttributeIterator it(label);it.More();it.Next()) {
            const auto a = it.Value(); char guid[37]; a->ID().ToCString(guid);
            NSMutableDictionary *value = [@{@"type":[NSString stringWithUTF8String:a->DynamicType()->Name()]} mutableCopy];
            if (const auto integer = Handle(TDataStd_Integer)::DownCast(a); !integer.IsNull()) value[@"integer"] = @(integer->Get());
            if (const auto real = Handle(TDataStd_Real)::DownCast(a); !real.IsNull()) {
                std::uint64_t bits; const double scalar=real->Get(); std::memcpy(&bits,&scalar,8);
                value[@"realBits"] = @(bits);
            }
            if (const auto ascii = Handle(TDataStd_AsciiString)::DownCast(a); !ascii.IsNull())
                value[@"ascii"] = [NSString stringWithUTF8String:ascii->Get().ToCString()];
            if (const auto name = Handle(TDataStd_Name)::DownCast(a); !name.IsNull()) value[@"utf16"] = utf16(name->Get());
            attributes[[NSString stringWithUTF8String:guid]] = value;
        }
        [infrastructure addObject:@{@"entry":entry(label),@"attributes":attributes}];
    }
    OcctSavedGroupState groups;
    if (!owner->CaptureSavedGroups(groups) || groups.groups.size()!=1 || groups.groups[0].members.size()!=1) return nil;
    const auto& group=groups.groups[0]; OcctObjectNameState guard;
    if (!owner->CaptureObjectNameStateForLabel(group.members[0],guard)
        || !guard.namePresent || guard.name!=TCollection_ExtendedString("Receipt guard")
        || group.name!=TCollection_ExtendedString("Receipt guard group")) return nil;
    core3d::receipt::Digest geometry;
    if (!core3d::receipt::GeometryDigest(guard.object.shape,geometry)) return nil;
    NSMutableArray *scalars = [NSMutableArray array], *present = [NSMutableArray array];
    for (std::size_t i=0;i<guard.object.scalars.size();++i) {
        std::uint64_t bits; std::memcpy(&bits,&guard.object.scalars[i],8);
        [scalars addObject:@(bits)]; [present addObject:@(guard.object.present[i])];
    }
    return @{@"infrastructure":infrastructure,@"guardEntityID":[NSString stringWithUTF8String:guard.object.entityIdentifier.c_str()],
        @"guardDefinitionID":[NSString stringWithUTF8String:guard.object.definitionIdentifier.c_str()],
        @"guardEntry":entry(group.members[0]),@"guardName":utf16(guard.name),
        @"guardGeometrySHA":[NSString stringWithUTF8String:core3d::receipt::Hex(geometry).c_str()],
        @"guardScalarBits":scalars,@"guardScalarPresence":present,
        @"groupID":[NSString stringWithUTF8String:group.identifier.c_str()],@"groupName":utf16(group.name),
        @"groupContainer":entry(groups.container),@"groupEntry":entry(group.recordLabel),@"groupMember":entry(group.members[0])};
}

static NSDictionary *Core3DRunNativeTombstoneProbe(NSInteger scenario) {
    namespace t = core3d::tombstone;
    NSMutableDictionary *checks = [NSMutableDictionary dictionary];
    NSURL *temporary = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:NSUUID.UUID.UUIDString isDirectory:YES];
    if (![NSFileManager.defaultManager createDirectoryAtURL:temporary withIntermediateDirectories:NO
        attributes:@{NSFilePosixPermissions:@0700} error:nil]) return @{@"fixtureCreated":@NO};
    auto expect = [&](NSString *name,bool passed){checks[name]=@(passed);};
    auto key = [](unsigned id) {
        t::Key k;k.accountScope.fill(1);k.command.fill(2);k.execution.fill(3);k.document.fill(4);
        k.request[0]=std::uint8_t(id>>8);k.request[1]=std::uint8_t(id);return k;
    };
    auto parent = [&](NSString *name) {
        NSURL *url = [temporary URLByAppendingPathComponent:name isDirectory:YES];
        if (![NSFileManager.defaultManager createDirectoryAtURL:url withIntermediateDirectories:NO
            attributes:@{NSFilePosixPermissions:@0700} error:nil]) throw std::runtime_error("Native fixture parent");
        return std::string(url.path.fileSystemRepresentation);
    };
    auto read = [&](const std::string& path) {
        t::detail::FD file(::open(path.c_str(),O_RDONLY|O_CLOEXEC|O_NOFOLLOW));
        std::vector<std::uint8_t> bytes;
        if(!t::detail::ReadAll(file.value,t::MaximumBytes,bytes))throw std::runtime_error("Native fixture read");
        return bytes;
    };
    auto overwrite = [&](const std::string& path,const std::vector<std::uint8_t>& bytes) {
        t::detail::FD file(::open(path.c_str(),O_WRONLY|O_TRUNC|O_CLOEXEC|O_NOFOLLOW));
        if(file.value<0||!t::detail::WriteAll(file.value,bytes.data(),bytes.size())||!t::detail::FileSync(file.value))
            throw std::runtime_error("Native fixture write");
    };
    try {
        if (scenario==0) {
            const auto path=parent(@"basic");auto store=t::Store::ForTesting(path);const auto original=key(1);
            expect(@"firstReserved",store.reserve(original)==t::Reservation::Reserved);
            auto reopened=t::Store::ForTesting(path);
            expect(@"freshInstanceNeverReadmits",reopened.reserve(original)==t::Reservation::AlreadyReserved);
            expect(@"lookupMatches",reopened.lookup(original)==t::Presence::Match);
            auto foreign=original;foreign.accountScope[0]^=1;
            expect(@"scopeConflict",reopened.reserve(foreign)==t::Reservation::Conflict);
            foreign=original;foreign.document[0]^=1;
            expect(@"documentConflict",reopened.reserve(foreign)==t::Reservation::Conflict);
            foreign=original;foreign.command[0]^=1;
            expect(@"commandConflict",reopened.reserve(foreign)==t::Reservation::Conflict);
            foreign=original;foreign.execution[0]^=1;
            expect(@"executionConflict",reopened.reserve(foreign)==t::Reservation::Conflict);
            expect(@"unknownAbsent",reopened.lookup(key(2))==t::Presence::Absent);
            const auto rootBytes=read(path+"/NativeModelingRequests/records");t::detail::IndexRoot index;
            t::detail::FD root(::open((path+"/NativeModelingRequests").c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW));
            auto pages=t::detail::OpenPages(root.value,false);std::vector<t::detail::Step> steps;t::detail::Page leaf;bool empty=true;
            expect(@"exactStoredBinding",t::detail::ReadRoot(rootBytes,index)&&index.count==1
                &&t::detail::Find(pages.value,index,original.request,steps,leaf,empty)&&!empty&&leaf.bit==128&&leaf.key==original);
            std::vector<std::uint8_t> bytes;std::vector<t::Key> decoded;
            expect(@"legacyCodecUnchanged",t::Encode({original},bytes)&&t::Decode(bytes,decoded)&&decoded.size()==1&&decoded[0]==original);
            expect(@"legacyReaderRefusesIndex",!t::Decode(rootBytes,decoded)&&decoded.empty());
            if(bytes.size()!=40+t::RecordBytes)throw std::runtime_error("Legacy codec fixture length");
            auto damaged=bytes;damaged.back()^=1;
            expect(@"checksumClosed",!t::Decode(damaged,decoded)&&decoded.empty());
            damaged=bytes;damaged.pop_back();expect(@"truncatedClosed",!t::Decode(damaged,decoded)&&decoded.empty());
            damaged=bytes;damaged[4]=2;expect(@"futureClosed",!t::Decode(damaged,decoded)&&decoded.empty());
            std::vector<std::uint8_t> encoded;expect(@"duplicateCodecClosed",!t::Encode({original,original},encoded)&&encoded.empty());
            expect(@"boundedBytes",bytes.size()==40+t::RecordBytes&&rootBytes.size()==t::detail::IndexRootBytes);
        } else if (scenario==1) {
            const auto path=parent(@"concurrent");constexpr unsigned count=8;
            std::array<t::Reservation,count> results;std::array<std::thread,count> workers;
            std::atomic<unsigned> ready{0};std::atomic<bool> start{false};
            try {
                for(unsigned i=0;i<count;++i)workers[i]=std::thread([&,i]{auto s=t::Store::ForTesting(path);ready.fetch_add(1);
                    while(!start.load())std::this_thread::yield();results[i]=s.reserve(key(1));});
            } catch (...) {
                start.store(true);for(auto& worker:workers)if(worker.joinable())worker.join();throw;
            }
            while(ready.load()!=count)std::this_thread::yield();start.store(true);
            for(auto& worker:workers)worker.join();
            const auto won=std::count(results.begin(),results.end(),t::Reservation::Reserved);
            expect(@"oneConcurrentReservation",won==1);
            expect(@"othersRefusedOrBusy",std::all_of(results.begin(),results.end(),[](auto value){return value==t::Reservation::Reserved
                ||value==t::Reservation::AlreadyReserved||value==t::Reservation::Busy;}));
            auto final=t::Store::ForTesting(path);expect(@"freshReadStillReserved",final.reserve(key(1))==t::Reservation::AlreadyReserved);
            t::detail::FD lock(::open((path+"/NativeModelingRequests/lock").c_str(),O_RDWR|O_CLOEXEC|O_NOFOLLOW));
            if(lock.value<0||::flock(lock.value,LOCK_EX|LOCK_NB)!=0)throw std::runtime_error("Probe lock");
            expect(@"externalFileLockBusy",final.reserve(key(2))==t::Reservation::Busy);
            expect(@"externalReadLockBusy",final.lookup(key(1))==t::Presence::Busy);
            ::flock(lock.value,LOCK_UN);
            expect(@"unlockAdmitsNewRequest",final.reserve(key(2))==t::Reservation::Reserved);
        } else if (scenario==2) {
            const auto path=parent(@"capacity");auto store=t::Store::ForTesting(path);bool all=true;
            for(unsigned i=1;i<=t::MaximumRecords;++i)all=store.reserve(key(i))==t::Reservation::Reserved&&all;
            expect(@"all128Retained",all);
            expect(@"request129Admitted",store.reserve(key(129))==t::Reservation::Reserved);
            expect(@"request256Admitted",store.reserve(key(256))==t::Reservation::Reserved);
            auto reopened=t::Store::ForTesting(path);bool retained=true;
            for(unsigned i=1;i<=129;++i)retained=reopened.reserve(key(i))==t::Reservation::AlreadyReserved&&retained;
            expect(@"capacityDoesNotEvict",retained&&reopened.reserve(key(256))==t::Reservation::AlreadyReserved);
            expect(@"oldestNeverReadmitted",reopened.reserve(key(1))==t::Reservation::AlreadyReserved);
            expect(@"newestNeverReadmitted",reopened.reserve(key(256))==t::Reservation::AlreadyReserved);
            expect(@"boundedRootBytes",read(path+"/NativeModelingRequests/records").size()==t::detail::IndexRootBytes);
        } else if (scenario==3) {
            const std::array<t::Store::Fault,4> faults{t::Store::Fault::AfterPartialPendingWrite,t::Store::Fault::AfterPendingSync,
                t::Store::Fault::AfterRename,t::Store::Fault::AfterDirectorySync};
            for(unsigned i=0;i<faults.size();++i){
                const auto path=parent([NSString stringWithFormat:@"fault%u",i]);auto store=t::Store::ForTesting(path);
                if(store.reserve(key(1))!=t::Reservation::Reserved)throw std::runtime_error("Fault fixture initialization");
                store.setNextFault(faults[i]);
                expect([NSString stringWithFormat:@"uncertain%u",i],store.reserve(key(2))==t::Reservation::Unavailable);
                auto fresh=t::Store::ForTesting(path);
                expect([NSString stringWithFormat:@"neverReadmits%u",i],fresh.reserve(key(2))==
                    (i<2?t::Reservation::Unavailable:t::Reservation::AlreadyReserved));
                expect([NSString stringWithFormat:@"priorReservation%u",i],fresh.lookup(key(1))==
                    (i<2?t::Presence::Unavailable:t::Presence::Match));
            }
            const auto initial=parent(@"uncertainInitialization");auto fresh=t::Store::ForTesting(initial);
            fresh.setNextFault(t::Store::Fault::AfterPendingSync);
            expect(@"initialFailureClosed",fresh.reserve(key(1))==t::Reservation::Unavailable);
            auto later=t::Store::ForTesting(initial);
            expect(@"initialFailureNotFreshStore",later.reserve(key(1))==t::Reservation::Unavailable);
        } else if (scenario==4) {
            for(unsigned issue=0;issue<6;++issue){
                const auto path=parent([NSString stringWithFormat:@"invalid%u",issue]);auto store=t::Store::ForTesting(path);
                if(store.reserve(key(1))!=t::Reservation::Reserved)throw std::runtime_error("Invalid fixture initialization");
                const auto records=path+"/NativeModelingRequests/records";
                if(issue==0){auto bytes=read(records);bytes.back()^=1;overwrite(records,bytes);}
                if(issue==1&&::unlink(records.c_str())!=0)throw std::runtime_error("Remove owned test catalog");
                if(issue==2){if(::rename(records.c_str(),(records+".retained").c_str())!=0
                    ||::symlink("records.retained",records.c_str())!=0)throw std::runtime_error("Symlink test catalog");}
                if(issue==3&&::chmod(records.c_str(),0644)!=0)throw std::runtime_error("Permission test catalog");
                if(issue==4){std::vector<std::uint8_t> huge(t::MaximumBytes+1,0);overwrite(records,huge);}
                if(issue==5&&::unlink((path+"/NativeModelingRequests/lock").c_str())!=0)throw std::runtime_error("Remove owned test marker");
                auto fresh=t::Store::ForTesting(path);
                expect([NSString stringWithFormat:@"malformedAdmission%u",issue],fresh.reserve(key(2))==t::Reservation::Unavailable);
                expect([NSString stringWithFormat:@"malformedLookup%u",issue],fresh.lookup(key(1))==t::Presence::Unavailable);
            }
        } else if (scenario==5) {
            const auto path=parent(@"privacy");auto store=t::Store::ForTesting(path);
            expect(@"reserved",store.reserve(key(1))==t::Reservation::Reserved);
            const auto root=path+"/NativeModelingRequests";
            t::detail::FD directory(::open(root.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW));
            t::detail::FD lock(::openat(directory.value,"lock",O_RDONLY|O_CLOEXEC|O_NOFOLLOW));
            t::detail::FD records(::openat(directory.value,"records",O_RDONLY|O_CLOEXEC|O_NOFOLLOW));
            expect(@"directory0700",t::detail::Owned(directory.value,true,0700));
            expect(@"lock0600",t::detail::Owned(lock.value,false,0600));
            expect(@"records0600",t::detail::Owned(records.value,false,0600));
            expect(@"noPendingAfterAcknowledgement",t::detail::Absent(directory.value,"pending"));
            expect(@"hardLinkRefused",::link((root+"/records").c_str(),(root+"/extra").c_str())==0
                &&store.lookup(key(1))==t::Presence::Unavailable);
            const auto symlinkParent=parent(@"symlinkRoot");
            expect(@"rootSymlinkRefused",::symlink(root.c_str(),(symlinkParent+"/NativeModelingRequests").c_str())==0
                &&t::Store::ForTesting(symlinkParent).reserve(key(2))==t::Reservation::Unavailable);
        } else expect(@"knownScenario",false);
    } catch (...) { checks[@"fixtureCompletedWithoutException"]=@NO; }
    [NSFileManager.defaultManager removeItemAtURL:temporary error:nil];
    return checks;
}
#endif

// Insert after Core3DModelingPlanningContext @implementation/@end.
@interface Core3DModelingHostSession () {
@public
    core3d::request::Digest _hostScopeDigest;
    core3d::request::Digest _hostGenerationDigest;
    core3d::request::UUID _hostIdentity;
    BOOL _hostRetired;
    std::shared_ptr<core3d::NativeModelingEpoch> _hostEpoch;
}
- (instancetype)initPrivate;
@end
static Core3DModelingHostSession *Core3DInstalledHostSession; // main only
static bool Core3DRequestString(NSString *value,std::size_t limit,std::string&out) {
    out.clear();if (![value isKindOfClass:NSString.class]||value.length==0||value.length>limit) return false;
    // UTF-8 requires at least as many bytes as UTF-16 code units. Refuse
    // oversized input before allocating its encoded representation.
    NSData *bytes=[value dataUsingEncoding:NSUTF8StringEncoding allowLossyConversion:NO];
    if (!bytes || bytes.length==0 || bytes.length>limit) return false;
    out.assign(static_cast<const char *>(bytes.bytes),bytes.length);
    return core3d::request::UTF8(out,limit);
}
static bool Core3DRequestUUID(NSUUID *value,core3d::request::UUID&out) {
    out.fill(0);if (![value isKindOfClass:NSUUID.class]) return false;
    [value getUUIDBytes:out.data()];return core3d::request::Nonzero(out);
}
static bool Core3DHostSessionCurrent(Core3DModelingHostSession *session) {
    return [NSThread isMainThread] && [session isKindOfClass:Core3DModelingHostSession.class]
        && session==Core3DInstalledHostSession && !session->_hostRetired;
}
@implementation Core3DModelingHostSession
- (instancetype)initPrivate {return [super init];}
+ (instancetype)installHostAccountSession:(NSString *)scope generation:(NSString *)generation {
    if (![NSThread isMainThread]) return nil;
    // Even an invalid replacement retires old host authority; login failure
    // must not accidentally leave the previous account active.
    if (Core3DInstalledHostSession) { Core3DInstalledHostSession->_hostRetired=YES;
        if(Core3DInstalledHostSession->_hostEpoch)Core3DInstalledHostSession->_hostEpoch->retired=true; }
    Core3DInstalledHostSession=nil;
    try {
        std::string nativeScope,nativeGeneration;
        if (!Core3DRequestString(scope,512,nativeScope)||!Core3DRequestString(generation,128,nativeGeneration)) return nil;
        Core3DModelingHostSession *session=[[self alloc] initPrivate];
        if (!core3d::request::ScopeHash(nativeScope,session->_hostScopeDigest)
            || !core3d::request::Hash("Shapeyard/host-generation/v1",
                std::vector<std::uint8_t>(nativeGeneration.begin(),nativeGeneration.end()),session->_hostGenerationDigest)
            || !Core3DRequestUUID(NSUUID.UUID,session->_hostIdentity)) return nil;
        session->_hostEpoch=std::make_shared<core3d::NativeModelingEpoch>();
        Core3DInstalledHostSession=session;return session;
    } catch (...) {return nil;}
}
+ (void)retireHostAccountSession:(Core3DModelingHostSession *)session {
    if (![NSThread isMainThread]||![session isKindOfClass:Core3DModelingHostSession.class]) return;
    session->_hostRetired=YES;
    if(session->_hostEpoch)session->_hostEpoch->retired=true;
    if (Core3DInstalledHostSession==session) Core3DInstalledHostSession=nil;
}
@end

@interface Core3DModelingPreparedRequest () {
@public
    __weak Core3DViewController *_requestOwner;
    Core3DModelingPlanningContext *_requestContext;
    Core3DModelingHostSession *_requestSession;
    core3d::request::Descriptor _requestDescriptor;
    core3d::request::Key _requestKey;
    std::vector<core3d::request::UUID> _requestFeatureIDs;
    std::optional<core3d::profile::Parameters> _requestProfile;
    std::optional<core3d::rectangular_loft::StationDimensionEdit> _requestLoftEdit;
    Core3DRigidPlacementPreparation *_requestPlacement;
    std::optional<core3d::enclosure::Parameters> _requestEnclosure;
    std::vector<core3d::AssemblyPartDefinition> _requestAssembly;
    BOOL _requestRetired;
    std::shared_ptr<core3d::NativeModelingEpoch> _requestEpoch;
    std::shared_ptr<core3d::NativeModelingCommitPermit> _requestCommitPermit;
    std::optional<core3d::receipt::Catalog> _requestCreationCatalog;
    std::optional<core3d::receipt::Effect> _requestSourceEffect; // Rebuild-only numeric proof.
    BOOL _requestAdmitted;
    core3d::request::UUID _requestReservationDeliveryID;
#if DEBUG
    NSDictionary *_requestAsyncStorageSnapshot;
    BOOL _requestAsyncConfigured;
    NSInteger _requestAsyncScenario;
    BOOL _requestAsyncReceiptFailure;
    void (^_requestAsyncGate)(void (^resume)(void));
    void (^_requestAsyncAfterStart)(void);
    void (^_requestAsyncGeometryDeliveryGate)(void (^resume)(void));
    void (^_requestAsyncBeforeCompletion)(void);
#endif
}
- (instancetype)initPrivateWithRequest:(NSUUID *)request document:(NSString *)document
    key:(const core3d::request::Key&)key coverage:(Core3DModelingEvidenceCoverage)coverage;
@end
@implementation Core3DModelingPreparedRequest
- (instancetype)initPrivateWithRequest:(NSUUID *)request document:(NSString *)document
    key:(const core3d::request::Key&)key coverage:(Core3DModelingEvidenceCoverage)coverage {
    if ((self=[super init])) {
        _requestIdentifier=[request copy];_documentIdentifier=[document copy];_requestKey=key;
        _commandSHA256=[NSData dataWithBytes:key.command.data() length:key.command.size()];
        _executionSHA256=[NSData dataWithBytes:key.execution.data() length:key.execution.size()];
        _evidenceCoverage=coverage;
    }return self;
}
@end

// Insert after Stage A private request types. Main-owned objects never enter
// utility blocks. This seam has no production caller before Stage C coupling.
namespace core3d::request {
struct ReservationDispatch { UUID deliveryID{}; Key key{}; };
struct ReservationDelivery {
    UUID deliveryID{}; Key key{};
    tombstone::Reservation disposition=tombstone::Reservation::Unavailable;
#if DEBUG
    tombstone::Presence observed=tombstone::Presence::Unavailable;
    bool privateStore=false,cleanup=false,workerMain=false;
#endif
};
inline tombstone::Key TombstoneKey(const Key& key) {
    tombstone::Key value;value.accountScope=key.accountScope;value.document=key.document;
    value.request=key.request;value.command=key.command;value.execution=key.execution;return value;
}
inline ReserveReply Reply(tombstone::Reservation reply) {
    switch(reply){case tombstone::Reservation::Reserved:return ReserveReply::FirstReserved;
        case tombstone::Reservation::AlreadyReserved:return ReserveReply::PreviouslySeen;
        case tombstone::Reservation::Conflict:return ReserveReply::Conflict;
        case tombstone::Reservation::Capacity:return ReserveReply::Capacity;
        case tombstone::Reservation::Busy:return ReserveReply::Busy;
        case tombstone::Reservation::Unavailable:return ReserveReply::Uncertain;}
    return ReserveReply::Uncertain;
}
}
typedef NS_ENUM(NSInteger, Core3DReservationOutcome) {
    Core3DReservationOutcomeReservedForCoupling,
    Core3DReservationOutcomeCancelled, Core3DReservationOutcomeRejected,
    Core3DReservationOutcomePreviouslySeen, Core3DReservationOutcomeConflict,
    Core3DReservationOutcomeCapacity, Core3DReservationOutcomeBusy,
    Core3DReservationOutcomeUncertain
};
typedef void (^Core3DReservationCompletion)(Core3DReservationOutcome,
    const core3d::request::ReservationDelivery&, BOOL);
@interface Core3DModelingReservationEntry : NSObject {
@public
    __weak Core3DViewController *_owner;
    Core3DModelingPreparedRequest *_prepared;
    Core3DReservationCompletion _completion;
    core3d::request::ReservationDispatch _dispatch;
    std::optional<core3d::request::Admission> _admission;
    BOOL _stopped;
#if DEBUG
    void (^_gate)(void (^resume)(void));
    BOOL _manualTestDeadline;
#endif
}
@end
@implementation Core3DModelingReservationEntry
@end
@interface Core3DModelingReservedCapability : NSObject {
@public
    Core3DModelingReservationEntry *_entry;
    core3d::request::ReservationDelivery _delivery;
    BOOL _taken;
}
@end
@implementation Core3DModelingReservedCapability
@end
@interface Core3DViewController (NativeReservationPrivate)
- (BOOL)core3d_reservationMatches:(Core3DModelingPreparedRequest *)request;
- (void)core3d_releaseReservation:(Core3DModelingPreparedRequest *)request;
- (BOOL)core3d_acceptReservedCapability:(Core3DModelingReservedCapability *)capability;
@end
static NSMutableDictionary<NSUUID *,Core3DModelingReservationEntry *> *Core3DReservationEntries;
static NSUUID *Core3DReservationIdentifier(const core3d::request::UUID& value) {
    return [[NSUUID alloc] initWithUUIDBytes:value.data()];
}
static Core3DReservationOutcome Core3DReservationClosedOutcome(core3d::request::Terminal terminal) {
    using T=core3d::request::Terminal;
    switch(terminal){case T::Cancelled:return Core3DReservationOutcomeCancelled;
        case T::PreviouslySeen:return Core3DReservationOutcomePreviouslySeen;
        case T::Conflict:return Core3DReservationOutcomeConflict;
        case T::Capacity:return Core3DReservationOutcomeCapacity;
        case T::Busy:return Core3DReservationOutcomeBusy;
        case T::Uncertain:return Core3DReservationOutcomeUncertain;
        default:return Core3DReservationOutcomeRejected;}
}
static void Core3DFinishReservation(Core3DModelingReservationEntry *entry,
    const core3d::request::ReservationDelivery& delivery,Core3DReservationOutcome outcome,BOOL storageKnown) {
    NSCAssert(NSThread.isMainThread,@"Native reservation completion must remain main-owned");
    if(outcome==Core3DReservationOutcomeRejected&&entry->_admission->phase()==core3d::request::Phase::Reserved)
        entry->_admission->beginGeometry(false); // closes policy; never starts geometry
    // Remove authority/slot before invoking potentially reentrant client code.
    [entry->_owner core3d_releaseReservation:entry->_prepared];
    entry->_prepared->_requestRetired=YES;
    entry->_prepared->_requestContext->_planningRetired=YES;
    Core3DReservationCompletion completion=entry->_completion;entry->_completion=nil;
#if DEBUG
    entry->_gate=nil;
#endif
    if(completion)completion(outcome,delivery,storageKnown);
}
static void Core3DDeliverReservation(const core3d::request::ReservationDelivery& delivery) {
    NSCAssert(NSThread.isMainThread,@"Native reservation delivery must remain on main");
    NSUUID *identifier=Core3DReservationIdentifier(delivery.deliveryID);
    Core3DModelingReservationEntry *entry=Core3DReservationEntries[identifier];
    // A mismatched or duplicate worker message has no authority over any slot.
    if(!entry||!(entry->_dispatch.key==delivery.key))return;
    [Core3DReservationEntries removeObjectForKey:identifier];
    if(entry->_stopped) {
        Core3DFinishReservation(entry,delivery,Core3DReservationOutcomeCancelled,delivery.disposition!=core3d::tombstone::Reservation::Unavailable);return;
    }
    entry->_admission->reservation(delivery.key,core3d::request::Reply(delivery.disposition));
    if(entry->_admission->phase()!=core3d::request::Phase::Reserved) {
        Core3DFinishReservation(entry,delivery,Core3DReservationClosedOutcome(entry->_admission->terminal()),delivery.disposition!=core3d::tombstone::Reservation::Unavailable);return;
    }
    Core3DViewController *owner=entry->_owner;
    if(!owner||![owner core3d_reservationMatches:entry->_prepared]) {
        Core3DFinishReservation(entry,delivery,Core3DReservationOutcomeRejected,YES);return;
    }
    // Only this actual first-reserved path can manufacture the private object.
    // Neither a decoded key nor an enum/status can reconstruct this capability.
    Core3DModelingReservedCapability *capability=[Core3DModelingReservedCapability new];
    capability->_entry=entry;capability->_delivery=delivery;
    if(![owner core3d_acceptReservedCapability:capability]) {
        Core3DFinishReservation(entry,delivery,Core3DReservationOutcomeRejected,YES);return;
    }
    Core3DReservationCompletion completion=entry->_completion;entry->_completion=nil;
#if DEBUG
    entry->_gate=nil;
#endif
    // This is reservation completion, never a model committed/retry-safe result.
    if(completion)completion(Core3DReservationOutcomeReservedForCoupling,delivery,YES);
}
static void Core3DExpireReservation(const core3d::request::UUID& deliveryID) {
    NSCAssert(NSThread.isMainThread,@"Native reservation deadline must run on main");
    NSUUID *identifier=Core3DReservationIdentifier(deliveryID);
    Core3DModelingReservationEntry *entry=Core3DReservationEntries[identifier];if(!entry)return;
    [Core3DReservationEntries removeObjectForKey:identifier];
    core3d::request::ReservationDelivery unknown;unknown.deliveryID=deliveryID;unknown.key=entry->_dispatch.key;
    // Even if Stop honestly cancelled geometry, storage may still finish later.
    // No completion or expiry makes this consumed request replayable.
    if(!entry->_stopped)entry->_admission->reservation(unknown.key,core3d::request::ReserveReply::Uncertain);
    Core3DFinishReservation(entry,unknown,entry->_stopped?Core3DReservationOutcomeCancelled:
        Core3DReservationOutcomeUncertain,NO);
}
static void Core3DDeadlineReservation(const core3d::request::UUID& deliveryID) {
#if DEBUG
    // Explicit deterministic capacity qualification only. The production
    // deadline stays30s; debugExpire still exercises the exact expiry path.
    Core3DModelingReservationEntry *entry=Core3DReservationEntries[Core3DReservationIdentifier(deliveryID)];
    if(entry&&entry->_manualTestDeadline)return;
#endif
    Core3DExpireReservation(deliveryID);
}
static void Core3DRouteReservationDelivery(const core3d::request::ReservationDelivery& delivery) {
    NSCAssert(NSThread.isMainThread,@"Native reservation routing must remain on main");
#if DEBUG
    Core3DModelingReservationEntry *entry=Core3DReservationEntries[Core3DReservationIdentifier(delivery.deliveryID)];
    if(entry&&entry->_gate&&entry->_dispatch.key==delivery.key) {
        void (^gate)(void (^)(void))=entry->_gate;entry->_gate=nil;
        const auto once=std::make_shared<std::atomic_bool>(false);
        const auto copiedDelivery=delivery;
        // Gate and arbitrary test callback are read/invoked/destroyed on main.
        // The resume block captures only numeric delivery + an atomic latch.
        gate(^{if(!once->exchange(true))dispatch_async(dispatch_get_main_queue(),^{Core3DDeliverReservation(copiedDelivery);});});
        return;
    }
#endif
    Core3DDeliverReservation(delivery);
}
static void Core3DRunReservation(const core3d::request::ReservationDispatch& work) {
    // Called only by the async entry after main-owned exact admission.
    // A first actual Reserved reply alone can continue into ordinary creation.
    const auto copiedWork=work;
    dispatch_async(dispatch_get_global_queue(QOS_CLASS_UTILITY,0),^{
        core3d::request::ReservationDelivery delivery;delivery.deliveryID=copiedWork.deliveryID;delivery.key=copiedWork.key;
#if DEBUG
        delivery.workerMain=::pthread_main_np();
#endif
        delivery.disposition=core3d::tombstone::Store().reserve(core3d::request::TombstoneKey(copiedWork.key));
        dispatch_async(dispatch_get_main_queue(),^{Core3DRouteReservationDelivery(delivery);});
    });
}

#if DEBUG
#include <cstdlib>
#include <stdexcept>
// Native-issued test configuration is copied path bytes + a bounded scenario.
// The worker never captures Foundation objects, callbacks or native handles.
struct Core3DReservationTestConfig {std::array<char,1024> parentPattern{};unsigned scenario=0;};
static core3d::request::ReservationDelivery Core3DRunPrivateReservation(
    core3d::request::ReservationDispatch work,Core3DReservationTestConfig config) {
    namespace t=core3d::tombstone;
    core3d::request::ReservationDelivery reply;reply.deliveryID=work.deliveryID;reply.key=work.key;
    reply.workerMain=::pthread_main_np();reply.privateStore=true;
    if(reply.workerMain||config.scenario>11||!::mkdtemp(config.parentPattern.data()))return reply;
    const std::string parent=config.parentPattern.data(),root=parent+"/NativeModelingRequests";
    try {
        auto store=t::Store::ForTesting(parent);const auto key=core3d::request::TombstoneKey(work.key);
        // Initialization is itself durable. Faults below target request admission,
        // not a mocked status or a fabricated proof capability.
        if(store.lookup(key)==t::Presence::Absent) {
            if(config.scenario==1)(void)store.reserve(key);
            if(config.scenario==2){auto other=key;other.command[0]^=1;(void)store.reserve(other);}
            if(config.scenario==3){
                t::detail::FD damaged(::open((root+"/records").c_str(),O_WRONLY|O_TRUNC|O_NOFOLLOW|O_CLOEXEC));
                const std::uint8_t bytes[]={1,2,3};
                if(damaged.value>=0){(void)t::detail::WriteAll(damaged.value,bytes,3);(void)t::detail::FileSync(damaged.value);}
            }
            if(config.scenario==4)(void)::unlink((root+"/records").c_str());
            if(config.scenario==5)store.setNextFault(t::Store::Fault::AfterRename);
            if(config.scenario==9)store.setNextFault(t::Store::Fault::AfterPartialPendingWrite);
            if(config.scenario==10)store.setNextFault(t::Store::Fault::AfterPendingSync);
            if(config.scenario==11)store.setNextFault(t::Store::Fault::AfterDirectorySync);
            if(config.scenario==7) {
                // Seed a valid legacy128-record catalog containing this UUID
                // with a different binding. V2 must migrate and refuse it as a
                // conflict, rather than evict it or retain the obsolete cap.
                std::vector<t::Key> full;
                for(unsigned i=1;i<=t::MaximumRecords;++i){auto occupied=key;occupied.request.fill(0);
                    occupied.request[0]=0x80;occupied.request[1]=key.request[1]^0xff;
                    occupied.request[15]=std::uint8_t(i);full.push_back(occupied);}
                full.pop_back();auto migratedConflict=key;migratedConflict.command[0]^=1;full.push_back(migratedConflict);
                std::vector<std::uint8_t> bytes;
                t::detail::FD seeded(::open((root+"/records").c_str(),O_WRONLY|O_TRUNC|O_NOFOLLOW|O_CLOEXEC));
                if(!t::Encode(full,bytes)||!t::detail::Owned(seeded.value,false,0600)
                    ||!t::detail::WriteAll(seeded.value,bytes.data(),bytes.size())||!t::detail::FileSync(seeded.value))
                    throw std::runtime_error("Private reservation legacy migration fixture");
            }
            t::detail::FD heldLock(config.scenario==8?
                ::open((root+"/lock").c_str(),O_RDWR|O_CLOEXEC|O_NOFOLLOW):-1);
            if(config.scenario==8&&(heldLock.value<0||::flock(heldLock.value,LOCK_EX|LOCK_NB)!=0))
                throw std::runtime_error("Private reservation busy fixture");
            reply.disposition=store.reserve(key);
            reply.observed=store.lookup(key);
        }
    }catch(...){reply.disposition=t::Reservation::Unavailable;}
    // This is a disposable DEBUG qualification root, never the permanent native
    // production store. Its actual readback precedes bounded known-file cleanup.
    bool clean=true;
    { t::detail::FD directory(::open(root.c_str(),O_RDONLY|O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW));
      if(!t::detail::Owned(directory.value,true,0700)||!t::detail::RemoveTestingPages(directory.value))clean=false; }
    for(const char *name:{"pending","records","lock","legacy-v1"})
        if(::unlink((root+"/"+name).c_str())!=0&&errno!=ENOENT)clean=false;
    if(::rmdir(root.c_str())!=0&&errno!=ENOENT)clean=false;
    if(::rmdir(parent.c_str())!=0)clean=false;
    reply.cleanup=clean;return reply;
}
static NSString *Core3DReservationOutcomeName(Core3DReservationOutcome value) {
    switch(value){case Core3DReservationOutcomeReservedForCoupling:return @"reservedForCoupling";
        case Core3DReservationOutcomeCancelled:return @"cancelled";case Core3DReservationOutcomeRejected:return @"rejected";
        case Core3DReservationOutcomePreviouslySeen:return @"previouslySeen";case Core3DReservationOutcomeConflict:return @"conflict";
        case Core3DReservationOutcomeCapacity:return @"capacity";case Core3DReservationOutcomeBusy:return @"busy";
        case Core3DReservationOutcomeUncertain:return @"uncertain";}
    return @"invalid";
}
static NSDictionary *Core3DReservationDebugResult(Core3DReservationOutcome outcome,
    const core3d::request::ReservationDelivery& delivery,BOOL known) {
    using R=core3d::tombstone::Reservation;NSString *storage=@"unavailable";
    switch(delivery.disposition){case R::Reserved:storage=@"reserved";break;case R::AlreadyReserved:storage=@"alreadyReserved";break;
        case R::Conflict:storage=@"conflict";break;case R::Capacity:storage=@"capacity";break;case R::Busy:storage=@"busy";break;case R::Unavailable:break;}
    using P=core3d::tombstone::Presence;NSString *observed=@"unavailable";
    switch(delivery.observed){case P::Absent:observed=@"absent";break;case P::Match:observed=@"match";break;
        case P::Conflict:observed=@"conflict";break;case P::Busy:observed=@"busy";break;case P::Unavailable:break;}
    return @{@"outcome":Core3DReservationOutcomeName(outcome),@"storage":storage,@"storageKnown":@(known),
        @"observed":observed,@"privateStore":@(delivery.privateStore),@"cleanup":@(delivery.cleanup),
        @"workerMain":@(delivery.workerMain),@"completionMain":@(NSThread.isMainThread)};
}
#endif


@interface Core3DModelingAsyncOutcome ()
- (instancetype)initWithDisposition:(Core3DModelingAsyncDisposition)disposition
    storage:(Core3DModelingStorageObservation)storage request:(NSUUID *)request
    document:(NSString *)document entities:(NSArray<NSString *> *)entities;
@end
@implementation Core3DModelingAsyncOutcome
- (instancetype)initWithDisposition:(Core3DModelingAsyncDisposition)disposition
    storage:(Core3DModelingStorageObservation)storage request:(NSUUID *)request
    document:(NSString *)document entities:(NSArray<NSString *> *)entities {
    if((self=[super init])){_disposition=disposition;_storageObservation=storage;
        _requestIdentifier=[request copy];_documentIdentifier=[document copy];_entityIdentifiers=[entities copy];}
    return self;
}
@end
// Callback and immutable resolution live exclusively on main. In particular,
// this box does not retain the native controller or prepared request.
@interface Core3DModelingAsyncCompletion : NSObject {
@public
    __weak Core3DViewController *_owner;
    __weak Core3DModelingPreparedRequest *_prepared;
    NSUUID *_requestID;
    NSString *_documentID;
    core3d::request::Key _key;
    std::shared_ptr<core3d::NativeModelingReceiptResolution> _resolution;
    Core3DModelingStorageObservation _storage;
    void (^_completion)(Core3DModelingAsyncOutcome *);
    BOOL _finished;
    BOOL _placementResolutionBound;
    core3d::receipt::UUID _placementEntity;
    BOOL _loftResolutionBound;
    core3d::receipt::UUID _loftEntity;
#if DEBUG
    void (^_beforeCompletion)(void);
#endif
}
@end
@implementation Core3DModelingAsyncCompletion
@end
static void Core3DFinishAsyncModeling(Core3DModelingAsyncCompletion *box,
    Core3DModelingAsyncDisposition disposition) {
    NSCAssert(NSThread.isMainThread,@"Async native result must remain main-owned");
    if(!box||box->_finished)return;
    NSMutableArray<NSString *> *entities=[NSMutableArray array];
    const auto resolution=box->_resolution;
    if(resolution&&resolution->state()==core3d::NativeModelingReceiptResolution::State::Committed){
        const auto& record=resolution->record();
        const bool same=record.key.accountScope==box->_key.accountScope&&record.key.document==box->_key.document
            &&record.key.request==box->_key.request&&record.key.command==box->_key.command
            &&record.key.execution==box->_key.execution&&!record.effects.empty()&&record.effects.size()<=16
            &&(!box->_loftResolutionBound||(record.operation==core3d::receipt::Operation::RebuildLoftStation
                &&record.policy==core3d::receipt::ExactLoftPolicy4097&&record.effects.size()==1
                &&record.effects.front().entity==box->_loftEntity))
            &&(!box->_placementResolutionBound||(record.operation==core3d::receipt::Operation::SetPlacement
                &&record.policy==core3d::receipt::ExactPlacementPolicy8193&&record.effects.size()==1
                &&record.effects.front().entity==box->_placementEntity));
        if(same){
            disposition=Core3DModelingAsyncDispositionCommitted;
            for(const auto&effect:record.effects)
                [entities addObject:[[NSUUID alloc] initWithUUIDBytes:effect.entity.data()].UUIDString];
        }else disposition=Core3DModelingAsyncDispositionUncertain;
    }else if(resolution&&resolution->state()==core3d::NativeModelingReceiptResolution::State::Unchanged){
        // This state is sealed only by the source/permit/prior-catalog-checked
        // ordinary no-change path. It has no receipt record or saved proof.
        if(box->_placementResolutionBound&&box->_storage==Core3DModelingStorageObservationReserved){
            disposition=Core3DModelingAsyncDispositionUnchanged;
            [entities addObject:[[NSUUID alloc] initWithUUIDBytes:box->_placementEntity.data()].UUIDString];
        }else if(box->_loftResolutionBound&&box->_storage==Core3DModelingStorageObservationReserved){
            disposition=Core3DModelingAsyncDispositionUnchanged;
            [entities addObject:[[NSUUID alloc] initWithUUIDBytes:box->_loftEntity.data()].UUIDString];
        }else disposition=Core3DModelingAsyncDispositionUncertain;
    }else if(disposition==Core3DModelingAsyncDispositionCommitted
        ||disposition==Core3DModelingAsyncDispositionUnchanged){
        // A generic geometry result cannot manufacture a committed receipt.
        disposition=Core3DModelingAsyncDispositionUncertain;
    }
    box->_finished=YES;
    auto completion=box->_completion;box->_completion=nil;
#if DEBUG
    Core3DModelingPreparedRequest *prepared=box->_prepared;
    if(prepared){prepared->_requestAsyncGate=nil;prepared->_requestAsyncAfterStart=nil;prepared->_requestAsyncBeforeCompletion=nil;prepared->_requestAsyncGeometryDeliveryGate=nil;}
    auto observer=box->_beforeCompletion;box->_beforeCompletion=nil;
#endif
    box->_prepared=nil;box->_owner=nil;box->_resolution.reset();
    Core3DModelingAsyncOutcome *outcome=[[Core3DModelingAsyncOutcome alloc]
        initWithDisposition:disposition storage:box->_storage request:box->_requestID
        document:box->_documentID entities:entities];
#if DEBUG
    // Ownership/result are already detached and immutable before reentrancy.
    if(observer)observer();
#endif
    if(completion)completion(outcome);
}
static Core3DModelingStorageObservation Core3DAsyncStorage(
    const core3d::request::ReservationDelivery& delivery,BOOL known) {
    if(!known)return Core3DModelingStorageObservationUnknown;
    using R=core3d::tombstone::Reservation;
    switch(delivery.disposition){
        case R::Reserved:return Core3DModelingStorageObservationReserved;
        case R::AlreadyReserved:return Core3DModelingStorageObservationPreviouslySeen;
        case R::Conflict:return Core3DModelingStorageObservationConflict;
        case R::Capacity:return Core3DModelingStorageObservationCapacity;
        case R::Busy:return Core3DModelingStorageObservationBusy;
        case R::Unavailable:return Core3DModelingStorageObservationUnknown;
    }
    return Core3DModelingStorageObservationUnknown;
}
static Core3DModelingAsyncDisposition Core3DAsyncReservationDisposition(Core3DReservationOutcome value){
    switch(value){
        case Core3DReservationOutcomeCancelled:return Core3DModelingAsyncDispositionCancelled;
        case Core3DReservationOutcomeRejected:return Core3DModelingAsyncDispositionRejected;
        case Core3DReservationOutcomePreviouslySeen:return Core3DModelingAsyncDispositionPreviouslySeen;
        case Core3DReservationOutcomeConflict:return Core3DModelingAsyncDispositionConflict;
        case Core3DReservationOutcomeCapacity:return Core3DModelingAsyncDispositionCapacity;
        case Core3DReservationOutcomeBusy:return Core3DModelingAsyncDispositionBusy;
        default:return Core3DModelingAsyncDispositionUncertain;
    }
}
static Core3DModelingAsyncDisposition Core3DAsyncConstructionDisposition(Core3DProfileConstructionResult value){
    switch(value){
        case Core3DProfileConstructionResultCommitted:return Core3DModelingAsyncDispositionCommitted;
        case Core3DProfileConstructionResultUnchanged:return Core3DModelingAsyncDispositionUnchanged;
        case Core3DProfileConstructionResultCancelled:return Core3DModelingAsyncDispositionCancelled;
        case Core3DProfileConstructionResultRejected:return Core3DModelingAsyncDispositionRejected;
        case Core3DProfileConstructionResultFailed:return Core3DModelingAsyncDispositionFailed;
        case Core3DProfileConstructionResultBusy:return Core3DModelingAsyncDispositionBusy;
        default:return Core3DModelingAsyncDispositionUncertain;
    }
}

#if DEBUG
static NSMutableDictionary<NSUUID *, id> *Core3DAsyncStoreProbeCallbacks;
#endif

static bool Core3DRebuildSourceMatches(Core3DModelingPreparedRequest *request,
    const Handle(OcctDocument)& owner) noexcept {
    try {
        if (!NSThread.isMainThread || !request || !request->_requestSourceEffect || owner.IsNull()
            || owner->Document().IsNull() || !XCAFDoc_DocumentTool::CheckShapeTool(owner->Document()->Main())) return false;
        TDF_LabelSequence roots; XCAFDoc_DocumentTool::ShapeTool(owner->Document()->Main())->GetFreeShapes(roots);
        if (roots.Length()>50000) return false;
        bool found=false;
        for (int i=1;i<=roots.Length();++i) {
            core3d::receipt::UUID entity;
            if (!core3d::receipt::ParseUUID(owner->EntityIdentifierForLabel(roots.Value(i)),entity)) return false;
            if (entity!=request->_requestSourceEffect->entity) continue;
            core3d::receipt::Effect live;
            if (found || !core3d::receipt::CaptureEffect(owner,roots.Value(i),live)
                || !(live==*request->_requestSourceEffect)) return false;
            found=true;
        }
        return found;
    } catch (...) { return false; }
}

static bool Core3DPlacementSourceMatches(Core3DModelingPreparedRequest *request,const Handle(OcctDocument)& owner) noexcept {
    try{
        if(!NSThread.isMainThread||!request||!request->_requestContext||!request->_requestPlacement||!request->_requestPlacement->_placementSnapshot
            ||!request->_requestPlacement->_placementSnapshot.entityIdentifier.UTF8String||request->_requestPlacement->_placementConsumed
            ||!request->_requestContext->_planningPlacementEvidence||!request->_requestSourceEffect
            ||request->_requestPlacement->_placementOwner!=request->_requestOwner
            ||request->_requestPlacement->_placementViewer.lock()!=request->_requestContext->_planningViewer.lock()
            ||!(request->_requestPlacement->_placementStamp==request->_requestContext->_planningStamp))return false;
        const auto& original=*request->_requestContext->_planningPlacementEvidence;
        TDF_Label label;core3d::placement::Evidence live;
        if(!core3d::placement::Find(owner,request->_requestPlacement->_placementSnapshot.entityIdentifier.UTF8String,label)
            ||!core3d::placement::Capture(owner,label,live)||!(live==original)
            ||!(live==request->_requestPlacement->_placementEvidence)
            ||!(core3d::placement::Effect(live)==*request->_requestSourceEffect))return false;
        core3d::request::Descriptor actual;std::vector<std::uint8_t> a,b;
        auto* p=request->_requestPlacement;
        const auto axis=p->_placementAxis==Core3DTransformInspectorAxisX?0:p->_placementAxis==Core3DTransformInspectorAxisY?1:2;
        return core3d::placement::Descriptor(live,p->_placementSnapshot.metersPerUnit,
            p->_placementKind==Core3DRigidPlacementKindRotation?1:0,axis,p->_placementRawValue,actual)
            &&core3d::request::Encode(actual,a)&&core3d::request::Encode(request->_requestDescriptor,b)&&a==b;
    }catch(...){return false;}
}
namespace core3d {
// Defined only at the main-owned actual-reservation boundary. No public
// constructor, serialized ticket, callback outcome or numeric enum issues one.
struct NativeModelingPermitIssuer final {
    static bool bindAsyncLoft(Core3DModelingAsyncCompletion *box, Core3DModelingPreparedRequest *prepared,
        const std::shared_ptr<NativeModelingCommitPermit>& permit) noexcept {
        if(!NSThread.isMainThread||!box||box->_finished||box->_loftResolutionBound||box->_resolution
            ||box->_prepared!=prepared||!prepared||box->_owner!=prepared->_requestOwner
            ||box->_storage!=Core3DModelingStorageObservationReserved||!permit||!permit->current()
            ||permit!=prepared->_requestCommitPermit||!permit->resolution_
            ||permit->resolution_->state()!=NativeModelingReceiptResolution::State::Pending
            ||permit->operation_!=receipt::Operation::RebuildLoftStation
            ||prepared.evidenceCoverage!=Core3DModelingEvidenceCoverageExactLoftEffect
            ||prepared->_requestDescriptor.operation!=request::Operation::RebuildLoftStation
            ||!permit->expectedSource_||!request::Nonzero(permit->expectedSource_->entity)
            ||!(box->_key==prepared->_requestKey)
            ||permit->key_.accountScope!=box->_key.accountScope||permit->key_.document!=box->_key.document
            ||permit->key_.request!=box->_key.request||permit->key_.command!=box->_key.command
            ||permit->key_.execution!=box->_key.execution)return false;
        box->_loftEntity=permit->expectedSource_->entity;
        box->_resolution=permit->resolution_; // shared_ptr copy is nonthrowing
        box->_loftResolutionBound=YES;
        return true;
    }
#if DEBUG
    static bool failBeforeRelease(Core3DModelingPreparedRequest *request) {
        if(!NSThread.isMainThread||!request||!request->_requestCommitPermit
            ||!request->_requestCommitPermit->current()
            ||request->_requestCommitPermit->resolution()->state()!=NativeModelingReceiptResolution::State::Pending)return false;
        request->_requestCommitPermit->debugBeforeReleaseFailure_=true;return true;
    }
    static void failReceiptStage(Core3DModelingPreparedRequest *request) {
        if(NSThread.isMainThread&&request&&request->_requestCommitPermit)request->_requestCommitPermit->debugStageFailure_=true;
    }
#endif
    static std::shared_ptr<NativeModelingCommitPermit> take(Core3DModelingReservedCapability *capability,
        const Handle(OcctDocument)& owner) {
        if(!NSThread.isMainThread||!capability||capability->_taken||!capability->_entry
            ||owner.IsNull()||owner->Document().IsNull()||owner->Document()->HasOpenCommand())return {};
        auto *entry=capability->_entry;auto *prepared=entry->_prepared;
        if(!prepared||!entry->_owner||![entry->_owner core3d_reservationMatches:prepared]
            ||capability->_delivery.disposition!=tombstone::Reservation::Reserved
            ||!(entry->_dispatch.key==capability->_delivery.key)||!entry->_admission
            ||entry->_admission->phase()!=request::Phase::Reserved
            ||!prepared->_requestEpoch||prepared->_requestEpoch->retired
            ||!prepared->_requestSession->_hostEpoch||prepared->_requestSession->_hostEpoch->retired)return {};
        const auto operation=prepared->_requestDescriptor.operation;
        if(operation!=request::Operation::CreateEnclosure&&operation!=request::Operation::CreateAssembly)return {};
        receipt::Catalog catalog;const auto status=receipt::Read(owner->Document(),catalog);
        receipt::UUID document;
        if((status!=receipt::ReadStatus::Absent&&status!=receipt::ReadStatus::Valid)
            ||!prepared->_requestCreationCatalog
            ||!catalog.matches(*prepared->_requestCreationCatalog)||!catalog.supportsAppend()
            ||!catalog.canAppend()
            ||!receipt::ParseUUID(owner->DocumentIdentifier(),document)||document!=prepared->_requestKey.document)return {};
        if(catalog.contains(prepared->_requestKey.request))return {};
        auto p=std::shared_ptr<NativeModelingCommitPermit>(new NativeModelingCommitPermit);
        p->key_.accountScope=prepared->_requestKey.accountScope;p->key_.document=prepared->_requestKey.document;
        p->key_.request=prepared->_requestKey.request;p->key_.command=prepared->_requestKey.command;p->key_.execution=prepared->_requestKey.execution;
        p->operation_=static_cast<receipt::Operation>(operation);p->descriptor_=prepared->_requestDescriptor;
        p->featureIDs_=prepared->_requestFeatureIDs;p->document_=owner->Document();p->previous_=std::move(catalog);
        p->session_=prepared->_requestSession->_hostEpoch;p->request_=prepared->_requestEpoch;
        p->resolution_=std::make_shared<NativeModelingReceiptResolution>();p->admission_=entry->_admission;
        if(!p->admission_->beginGeometry(true))return {};
        capability->_taken=YES;return p;
    }
    static std::shared_ptr<NativeModelingCommitPermit> takeRebuild(Core3DModelingReservedCapability *capability,
        const Handle(OcctDocument)& owner) {
        if(!NSThread.isMainThread||!capability||capability->_taken||!capability->_entry
            ||owner.IsNull()||owner->Document().IsNull()||owner->Document()->HasOpenCommand())return {};
        auto *entry=capability->_entry;auto *prepared=entry->_prepared;
        if(!prepared||!entry->_owner||![entry->_owner core3d_reservationMatches:prepared]
            ||capability->_delivery.disposition!=tombstone::Reservation::Reserved
            ||!(entry->_dispatch.key==capability->_delivery.key)||!entry->_admission
            ||entry->_admission->phase()!=request::Phase::Reserved
            ||!prepared->_requestEpoch||prepared->_requestEpoch->retired
            ||!prepared->_requestSession->_hostEpoch||prepared->_requestSession->_hostEpoch->retired)return {};
        const auto operation=prepared->_requestDescriptor.operation;
        if(operation!=request::Operation::RebuildEnclosure&&operation!=request::Operation::RebuildProfile
            &&operation!=request::Operation::RebuildLoftStation)return {};
        if(!Core3DRebuildSourceMatches(prepared,owner))return {};
        receipt::Catalog catalog;const auto status=receipt::Read(owner->Document(),catalog);
        receipt::UUID document;
        if((status!=receipt::ReadStatus::Absent&&status!=receipt::ReadStatus::Valid)
            ||!prepared->_requestCreationCatalog
            ||!catalog.matches(*prepared->_requestCreationCatalog)||!catalog.supportsAppend()
            ||!catalog.canAppend()
            ||!receipt::ParseUUID(owner->DocumentIdentifier(),document)||document!=prepared->_requestKey.document)return {};
        if(catalog.contains(prepared->_requestKey.request))return {};
        auto p=std::shared_ptr<NativeModelingCommitPermit>(new NativeModelingCommitPermit);
        p->key_.accountScope=prepared->_requestKey.accountScope;p->key_.document=prepared->_requestKey.document;
        p->key_.request=prepared->_requestKey.request;p->key_.command=prepared->_requestKey.command;p->key_.execution=prepared->_requestKey.execution;
        p->operation_=static_cast<receipt::Operation>(operation);p->descriptor_=prepared->_requestDescriptor;
        p->featureIDs_=prepared->_requestFeatureIDs;p->document_=owner->Document();p->previous_=std::move(catalog);
        p->expectedSource_=prepared->_requestSourceEffect;
        p->session_=prepared->_requestSession->_hostEpoch;p->request_=prepared->_requestEpoch;
        p->resolution_=std::make_shared<NativeModelingReceiptResolution>();p->admission_=entry->_admission;
        if(!p->admission_->beginGeometry(true))return {};
        capability->_taken=YES;return p;
    }
    static std::shared_ptr<NativeModelingCommitPermit> takePlacement(Core3DModelingReservedCapability *capability,
        const Handle(OcctDocument)& owner) {
        if(!NSThread.isMainThread||!capability||capability->_taken||!capability->_entry
            ||owner.IsNull()||owner->Document().IsNull()||owner->Document()->HasOpenCommand())return {};
        auto *entry=capability->_entry;auto *prepared=entry->_prepared;
        if(!prepared||!entry->_owner||![entry->_owner core3d_reservationMatches:prepared]
            ||capability->_delivery.disposition!=tombstone::Reservation::Reserved
            ||!(entry->_dispatch.key==capability->_delivery.key)||!entry->_admission
            ||entry->_admission->phase()!=request::Phase::Reserved
            ||!prepared->_requestEpoch||prepared->_requestEpoch->retired
            ||!prepared->_requestSession->_hostEpoch||prepared->_requestSession->_hostEpoch->retired)return {};
        const auto operation=prepared->_requestDescriptor.operation;
        if(operation!=request::Operation::SetPlacement||!prepared->_requestFeatureIDs.empty())return {};
        if(!Core3DPlacementSourceMatches(prepared,owner))return {};
        receipt::Catalog catalog;const auto status=receipt::Read(owner->Document(),catalog);
        receipt::UUID document;
        if((status!=receipt::ReadStatus::Absent&&status!=receipt::ReadStatus::Valid)
            ||!prepared->_requestCreationCatalog
            ||!catalog.matches(*prepared->_requestCreationCatalog)||!catalog.supportsAppend()
            ||!catalog.canAppend()
            ||!receipt::ParseUUID(owner->DocumentIdentifier(),document)||document!=prepared->_requestKey.document)return {};
        if(catalog.contains(prepared->_requestKey.request))return {};
        auto p=std::shared_ptr<NativeModelingCommitPermit>(new NativeModelingCommitPermit);
        p->key_.accountScope=prepared->_requestKey.accountScope;p->key_.document=prepared->_requestKey.document;
        p->key_.request=prepared->_requestKey.request;p->key_.command=prepared->_requestKey.command;p->key_.execution=prepared->_requestKey.execution;
        p->operation_=static_cast<receipt::Operation>(operation);p->descriptor_=prepared->_requestDescriptor;
        p->featureIDs_=prepared->_requestFeatureIDs;p->document_=owner->Document();p->previous_=std::move(catalog);
        p->expectedSource_=prepared->_requestSourceEffect;
        p->expectedPlacement_=prepared->_requestContext->_planningPlacementEvidence;
        p->session_=prepared->_requestSession->_hostEpoch;p->request_=prepared->_requestEpoch;
        p->resolution_=std::make_shared<NativeModelingReceiptResolution>();p->admission_=entry->_admission;
        // No geometry worker is needed for a numeric rigid transform. Move
        // the real Reserved capability through the same one-use admission.
        if(!p->admission_->beginGeometry(true)||!p->admission_->geometryReady(true,true)
            ||!p->admission_->takeForOrdinary(true))return {};
        p->attached_=true;capability->_taken=YES;return p;
    }
};
}

// Main-owned lease/callback bookkeeping. No instance of this class crosses the
// utility dispatch boundary; only the native numeric job and UUID do.
@interface Core3DSavedCutSourceJob : NSObject {
@public
    // Explicitly tagged legacy/whole-program representation: exactly one
    // alternative is populated for the job's whole lifetime. The two native
    // authorities stay separate while sharing this main-owned bookkeeping.
    std::variant<std::monostate,std::shared_ptr<core3d::SavedCutSourceEditWork>,
        std::shared_ptr<core3d::SavedProgramSourceEditWork>> _lease;
    std::variant<std::monostate,std::shared_ptr<core3d::SavedCutSourceEditCancellation>,
        std::shared_ptr<core3d::SavedProgramSourceEditCancellation>> _cancellation;
    std::weak_ptr<core3d::Core3DViewer> _viewer;
    NSUUID *_completionToken;
    __weak Core3DModelingPlanningContext *_planningContext;
    BOOL _cancelled;
    BOOL _finished;
}
- (BOOL)cancel;
- (void)finish:(Core3DProfileConstructionResult)result;
// Thread-safe token signal only; no main lease/scene mutation. Any thread.
- (void)cancelToken;
@end
@implementation Core3DSavedCutSourceJob
- (void)cancelToken {
    if(const auto* token=std::get_if<std::shared_ptr<core3d::SavedCutSourceEditCancellation>>(&_cancellation))
        (void)core3d::Core3DViewer::cancelSavedCutSourceEdit(*token);
    else if(const auto* token=std::get_if<std::shared_ptr<core3d::SavedProgramSourceEditCancellation>>(&_cancellation))
        (void)core3d::Core3DViewer::cancelSavedProgramSourceEdit(*token);
}
- (BOOL)cancel {
    if(!NSThread.isMainThread||_finished||_cancelled)return NO;
    if(const auto* token=std::get_if<std::shared_ptr<core3d::SavedCutSourceEditCancellation>>(&_cancellation)){
        if(!core3d::Core3DViewer::cancelSavedCutSourceEdit(*token))return NO;
    }else if(const auto* token=std::get_if<std::shared_ptr<core3d::SavedProgramSourceEditCancellation>>(&_cancellation)){
        if(!core3d::Core3DViewer::cancelSavedProgramSourceEdit(*token))return NO;
    }else return NO;
    _cancelled=YES;
    const auto viewer=_viewer.lock();
    if(viewer){
        if(const auto* lease=std::get_if<std::shared_ptr<core3d::SavedCutSourceEditWork>>(&_lease))
            (void)viewer->discardSavedCutSourceEdit(*lease);
        else if(const auto* lease=std::get_if<std::shared_ptr<core3d::SavedProgramSourceEditWork>>(&_lease))
            (void)viewer->discardSavedProgramSourceEdit(*lease);
    }
    _lease=std::monostate(); // Native scene authority releases on main; numeric work drains.
    return YES;
}
- (void)finish:(Core3DProfileConstructionResult)result {
    NSCAssert(NSThread.isMainThread,@"Saved-cut source delivery is main-owned");
    if(_finished)return;
    _finished=YES;
    Core3DModelingPlanningContext *context=_planningContext;_planningContext=nil;
    if(context&&context->_planningSavedCutJob==self){
        context->_planningSavedCutJob=nil;context->_planningRetired=YES;
    }
    const auto viewer=_viewer.lock();
    if(viewer){
        if(const auto* lease=std::get_if<std::shared_ptr<core3d::SavedCutSourceEditWork>>(&_lease))
            (void)viewer->discardSavedCutSourceEdit(*lease);
        else if(const auto* lease=std::get_if<std::shared_ptr<core3d::SavedProgramSourceEditWork>>(&_lease))
            (void)viewer->discardSavedProgramSourceEdit(*lease);
    }
    _lease=std::monostate();_cancellation=std::monostate();_viewer.reset();
    NSUUID *token=_completionToken;_completionToken=nil;
    if(token)Core3DDeliverNativeSolidCompletion(token,result); // Detaches before reentrant client code.
}
- (void)dealloc {
    // Defensive final-release routing: even an off-main controller teardown
    // must not destroy its native main lease on that thread.
    [self cancelToken];
    auto lease=std::move(_lease);const auto viewer=_viewer;NSUUID *token=_completionToken;
    if(!std::holds_alternative<std::monostate>(lease)||token){
        dispatch_block_t retire=^{
            const auto owner=viewer.lock();
            if(owner){
                if(const auto* held=std::get_if<std::shared_ptr<core3d::SavedCutSourceEditWork>>(&lease))
                    (void)owner->discardSavedCutSourceEdit(*held);
                else if(const auto* held=std::get_if<std::shared_ptr<core3d::SavedProgramSourceEditWork>>(&lease))
                    (void)owner->discardSavedProgramSourceEdit(*held);
            }
            if(token)Core3DDeliverNativeSolidCompletion(token,Core3DProfileConstructionResultRejected);
        };
        // Always queue: this block owns the moved lease until main releases it.
        dispatch_async(dispatch_get_main_queue(),retire);
    }
}
@end
@interface Core3DSavedCutSourceOperation ()
- (instancetype)initWithJob:(Core3DSavedCutSourceJob *)job;
@end
@implementation Core3DSavedCutSourceOperation {
    __weak Core3DSavedCutSourceJob *_job;
}
- (instancetype)initWithJob:(Core3DSavedCutSourceJob *)job {self=[super init];if(self)_job=job;return self;}
- (BOOL)cancel {if(!NSThread.isMainThread)return NO;Core3DSavedCutSourceJob *job=_job;return job?[job cancel]:NO;}
@end

@interface Core3DViewController () {
    NSHashTable<Core3DModelingPreparedRequest *> *_issuedModelingPreparedRequests;
    Core3DModelingPreparedRequest *_pendingModelingReservation;
    Core3DModelingReservedCapability *_reservedModelingCapability;
    BOOL _isSetuped;
    Core3DQueuedAssetRequestSlot *_queuedAssetRequestSlot;
    Core3DQueuedAssetRequest *_queuedAssetRequest;
    Core3DQueuedAssetLoadOwner *_queuedNativeLoadOwner;
    __weak GLViewController *_queuedRequestGL;
    std::atomic_bool _isLoading;
    std::shared_ptr<core3d::NativeSolidWork> _nativeSolidWork;
    Core3DSavedCutSourceJob *_savedCutSourceJob; // Main lease; retained through actual worker drain.
#if DEBUG
    void (^_debugSavedCutSourceDeliveryGate)(void (^resume)(void));
#endif
    BOOL _nativeSolidCancelled;
    __weak Core3DModelingPlanningContext *_issuedModelingPlanningContext;
    Core3DModelingPlanningContext *_modelingConstructionContext;
    std::shared_ptr<core3d::ObjectAlignmentWork> _objectAlignmentWork;
    BOOL _objectAlignmentCancelled;
    Core3DMeshContactOperation *_meshContactOperation;
    NSObject *_meshContactToken;
    __weak Core3DMeshContactReport *_issuedMeshContactReport;
    std::shared_ptr<const core3d::meshcheck::ContactSourceCapture> _meshContactSource;
#ifdef DEBUG
    NSUInteger _debugMaximumTextureAuthoringObjects;
    void (^_debugMeshContactAfterCapture)(void);
    void (^_debugMeshContactBeforeDelivery)(void);
    void (^_debugMeshContactDeliveryGate)(void (^resume)(void));
#endif
}

- (BOOL)core3d_startReservedRebuild:(Core3DModelingPreparedRequest *)request completion:(void (^)(Core3DProfileConstructionResult))completion;
- (BOOL)core3d_startReservedRebuild:(Core3DModelingPreparedRequest *)request
    asyncCompletion:(Core3DModelingAsyncCompletion *)box completion:(void (^)(Core3DProfileConstructionResult))completion;
- (BOOL)core3d_startReservedCreation:(Core3DModelingPreparedRequest *)request completion:(void (^)(Core3DProfileConstructionResult))completion;
- (BOOL)core3d_canBeginCommittedEdit;
- (Core3DModelingPreparedRequest *)core3d_prepareRequest:(const core3d::request::Descriptor&)descriptor
    context:(Core3DModelingPlanningContext *)context session:(Core3DModelingHostSession *)session
    requestID:(NSUUID *)requestID featureIDs:(const std::vector<core3d::request::UUID>&)featureIDs
    coverage:(Core3DModelingEvidenceCoverage)coverage;
- (nullable Core3DSavedCutSourceOperation *)core3d_beginSavedCutSourceEdit:(Core3DCylindricalCutSnapshot *)original
    patch:(Core3DSavedCutSourcePatch *)patch expected:(Core3DSceneSnapshot *)expected
    planningContext:(nullable Core3DModelingPlanningContext *)planningContext
    completion:(void(^)(Core3DProfileConstructionResult))completion;
- (nullable Core3DSavedCutSourceOperation *)core3d_beginSavedProgramCutSourceEdit:(Core3DCylindricalCutProgramSnapshot *)original
    patch:(Core3DSavedCutSourcePatch *)patch expected:(Core3DSceneSnapshot *)expected
    planningContext:(nullable Core3DModelingPlanningContext *)planningContext
    completion:(void(^)(Core3DProfileConstructionResult))completion;
- (BOOL)core3d_modelingContext:(Core3DModelingPlanningContext *)context
    matchesAllowingConsumed:(BOOL)allowConsumed;
- (BOOL)core3d_canCaptureModelingContext;
- (BOOL)core3d_canCaptureModelingContextForReservation:(Core3DModelingPlanningContext *)context;
- (void)core3d_retireReservationContext:(Core3DModelingPlanningContext *)context;
- (std::optional<core3d::request::ReservationDispatch>)core3d_registerReservation:
    (Core3DModelingPreparedRequest *)request completion:(Core3DReservationCompletion)completion;
- (void)core3d_executeEnclosure:(Core3DEnclosureDefinition *)definition
    context:(Core3DModelingPlanningContext *)context rebuild:(BOOL)rebuild
    completion:(void(^)(Core3DProfileConstructionResult))completion;
- (core3d::meshcheck::ContactSourceStatus)core3d_captureMeshContactSource:
    (const core3d::meshcheck::ContactSourceIdentity&)identity
    cancelled:(const std::atomic_bool&)cancelled
    output:(core3d::meshcheck::ContactSourceCapture&)output;
- (BOOL)core3d_hasCompetingLoadOrControllerWork;
- (Core3DAssetLoadResult)core3d_acceptQueuedInput:(Core3DQueuedAssetInput *)input;
- (Core3DAssetLoadResult)core3d_startQueuedRequest:(Core3DQueuedAssetRequest *)request;
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
- (void)runNativeSolidWork:(const std::shared_ptr<core3d::NativeSolidWork>&)work
                completion:(void(^)(Core3DProfileConstructionResult))completion;
#if DEBUG
- (void)runNativeSolidWork:(const std::shared_ptr<core3d::NativeSolidWork>&)work
    debugGeometryDeliveryGate:(void (^)(void (^)(void)))gate
    completion:(void(^)(Core3DProfileConstructionResult))completion;
#endif
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
- (Core3DTransformInspectorPositionCommitResult)commitTransformInspectorValue:(double)value
    axis:(Core3DTransformInspectorAxis)axis expectedSnapshot:(Core3DTransformInspectorSnapshot *)snapshot
    kind:(core3d::TransformInspectorEditKind)kind placementPermit:(std::shared_ptr<core3d::NativeModelingCommitPermit>)permit;
- (Core3DTransformInspectorPositionCommitResult)core3d_executeReservedPlacement:(Core3DModelingPreparedRequest *)request
    receiptFailure:(BOOL)receiptFailure;
- (Core3DTransformInspectorPositionCommitResult)core3d_executeReservedPlacement:(Core3DModelingPreparedRequest *)request
    receiptFailure:(BOOL)receiptFailure asyncCompletion:(Core3DModelingAsyncCompletion *)box;
@end

@interface Core3DPBRScalarPreparation () {
@public
    __weak Core3DViewController* _scalarOwner;
    std::shared_ptr<core3d::NativePBRScalarWork> _scalarWork;
}
- (instancetype)initPrivate;
@end
@implementation Core3DPBRScalarPreparation
- (instancetype)initPrivate {return [super init];}
@end

@implementation Core3DViewController {
    __weak dispatch_cancelable_block_t _uiStateChangingBlock;
    UIStateChanging _sendingState;

    __weak dispatch_cancelable_block_t _modifiedAssetBlock;
}

- (void)dealloc {
    Core3DSavedCutSourceJob *sourceJob=_savedCutSourceJob;_savedCutSourceJob=nil;
    if(sourceJob){
        [sourceJob cancelToken];
        if(NSThread.isMainThread)[sourceJob finish:Core3DProfileConstructionResultRejected];
        else dispatch_async(dispatch_get_main_queue(),^{[sourceJob finish:Core3DProfileConstructionResultRejected];});
    }
    core3d::Core3DViewer::cancelObjectAlignment(_objectAlignmentWork);
    core3d::Core3DViewer::cancelNativeSolid(_nativeSolidWork);
    [[NSNotificationCenter defaultCenter] removeObserver:self];
    if ([_glController isKindOfClass:GLViewController.class] && _queuedNativeLoadOwner != nil)
        [(GLViewController *)_glController abandonQueuedAssetOwner:_queuedNativeLoadOwner];
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

#if DEBUG
+ (NSDictionary<NSString *,NSNumber *> *)debugCylindricalCutSimilarityProbe {
    using core3d::cylindrical_cut::PositiveUniformSimilarity;
    const std::array<double,12> identity{1,0,0,0,0,1,0,0,0,0,1,0};double scale=0;
    NSMutableDictionary *out=[NSMutableDictionary dictionary];out[@"identity"]=@(PositiveUniformSimilarity(identity,scale)&&scale==1);
    auto rotated=std::array<double,12>{0,-2,0,12,2,0,0,-4,0,0,2,8};out[@"positiveRotated"]=@(PositiveUniformSimilarity(rotated,scale)&&scale==2);
    auto nonuniform=identity;nonuniform[0]=2;out[@"nonuniform"]=@(!PositiveUniformSimilarity(nonuniform,scale));
    auto shear=identity;shear[1]=.2;out[@"shear"]=@(!PositiveUniformSimilarity(shear,scale));
    auto mirror=identity;mirror[0]=-1;out[@"reflection"]=@(!PositiveUniformSimilarity(mirror,scale));
    auto singular=identity;singular[0]=0;out[@"singular"]=@(!PositiveUniformSimilarity(singular,scale));
    auto infinity=identity;infinity[3]=std::numeric_limits<double>::infinity();out[@"nonfinite"]=@(!PositiveUniformSimilarity(infinity,scale));
    gp_Trsf t;double factor=0;out[@"unitOverflow"]=@(!core3d::cylindrical_cut::EffectiveMM(t,std::numeric_limits<double>::max(),scale,factor));
    return out;
}
- (NSDictionary<NSString *,NSNumber *> *)debugCutSceneGuardMutation:(NSInteger)mode target:(NSString *)entity sibling:(NSString *)sibling {
    if(!NSThread.isMainThread||!GLController||!GLController.viewer||!entity||!sibling||mode<0||mode>2)return @{};
    const auto owner=GLController.viewer->getDocument();if(owner.IsNull())return @{};const auto document=owner->Document();
    if(document.IsNull()||document->HasOpenCommand())return @{};
    try {
        TDF_Label target,other;TDF_LabelSequence roots;XCAFDoc_DocumentTool::ShapeTool(document->Main())->GetFreeShapes(roots);
        if(roots.Length()>50000)return @{};
        for(int i=1;i<=roots.Length();++i){const auto label=roots.Value(i);const auto id=owner->EntityIdentifierForLabel(label);
            if(id==(entity.UTF8String?:""))target=label;if(id==(sibling.UTF8String?:""))other=label;}
        if(target.IsNull()||other.IsNull()||target==other)return @{};
        const auto original=owner->CaptureSavedCutSceneState(target);const bool positive=original&&owner->SavedCutSceneStateMatches(original);
        if(!positive)return @{@"positive":@NO};
        document->NewCommand();if(!document->HasOpenCommand())return @{};
        if(mode==0){const auto material=XCAFDoc_VisMaterialTool::GetShapeMaterial(other);
            if(material.IsNull()||!material->HasPbrMaterial()){Core3DAbortCommandNoThrow(document);return @{};}
            auto pbr=material->PbrMaterial();pbr.Roughness=pbr.Roughness==.25f?.5f:.25f;material->SetPbrMaterial(pbr);
        }else if(mode==1){TDataStd_Name::Set(other,TCollection_ExtendedString("Changed unrelated cut guard name"));}
        else{OcctReferenceAxis axis;axis.pivotSpace=OcctReferenceSpace::Object;axis.pivot=gp_Pnt(1,2,3);
            if(!owner->SetReferenceAxisForLabel(other,axis)){Core3DAbortCommandNoThrow(document);return @{};}}
        // Require separately valid recapture: this isolates exact old-source
        // comparison from a generic malformed-document refusal.
        const bool valid=bool(owner->CaptureSavedCutSceneState(target));const bool refused=!owner->SavedCutSceneStateMatches(original);
        Core3DAbortCommandNoThrow(document);
        const bool restored=!document->HasOpenCommand()&&owner->SavedCutSceneStateMatches(original);
        return @{@"positive":@(positive),@"changedValid":@(valid),@"changedRejected":@(refused),@"restored":@(restored)};
    }catch(...){Core3DAbortCommandNoThrow(document);return @{};}
}
- (BOOL)debugSetCutDisplayCoefficient:(double)coefficient pending:(BOOL)pending {
    return NSThread.isMainThread&&GLController&&GLController.viewer
        &&GLController.viewer->debugSetCutDisplayCoefficient(coefficient,pending);
}
- (NSDictionary<NSString *,NSNumber *> *)debugSavedCutSourceViewerQualification:(Core3DCylindricalCutSnapshot *)original expected:(Core3DSceneSnapshot *)expected {
    if(!NSThread.isMainThread||!GLController||!GLController.viewer
        ||![original isKindOfClass:Core3DCylindricalCutSnapshot.class]||![original matchesOwner:self viewer:GLController.viewer])return @{};
    const auto current=[self cylindricalCutSourceWithEntityIdentifier:original.entityIdentifier expected:expected];
    if(!current)return @{};
    const CGSize size=GLController.drawableSize;NSMutableDictionary *out=[NSMutableDictionary dictionary];
    for(const auto& row:GLController.viewer->debugSavedCutSourceViewerQualification([original nativeSnapshot],
        [current nativeSnapshot].identity,expected.revisions.presentationRevision,
        static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height))))
        out[[NSString stringWithUTF8String:row.first.c_str()]]=@(row.second);
    return out;
}
- (NSDictionary<NSString *,id> *)debugSavedCutSourceEnclosureWidth:(double)widthMM
    original:(Core3DCylindricalCutSnapshot *)original expected:(Core3DSceneSnapshot *)expected {
    return [self debugSavedCutSourceFixtureEdit:0 valueMM:widthMM depthMM:8 original:original expected:expected cancelPoint:0 beforeCommit:nil];
}
- (NSDictionary<NSString *,id> *)debugSavedCutSourceFixtureEdit:(NSInteger)fixture valueMM:(double)widthMM depthMM:(double)depthMM
    original:(Core3DCylindricalCutSnapshot *)original expected:(Core3DSceneSnapshot *)expected
    cancelPoint:(NSInteger)cancelPoint beforeCommit:(void (^)(void))beforeCommit {
    if(!NSThread.isMainThread||!std::isfinite(widthMM)||widthMM<1||widthMM>140||!std::isfinite(depthMM)||depthMM<1||depthMM>20
        ||fixture<0||fixture>1||cancelPoint<0||cancelPoint>2
        ||![original isKindOfClass:Core3DCylindricalCutSnapshot.class]||!GLController||!GLController.viewer
        ||![original matchesOwner:self viewer:GLController.viewer])return @{@"phase":@"owner",@"prepared":@NO};
    try {
        const auto live=[self cylindricalCutSourceWithEntityIdentifier:original.entityIdentifier expected:expected];
        if(!live)return @{@"phase":@"live-source",@"prepared":@NO};
        const auto before=[original nativeSnapshot];const auto identity=[live nativeSnapshot].identity;
        const auto viewer=GLController.viewer;const auto owner=viewer->getDocument();
        const double mm=before.source.envelope.metersPerUnit*1000;
        if(before.source.envelope.sourceFamily!=(fixture==0?2:1)||!std::isfinite(mm)||mm<=0)return @{@"phase":@"source-family",@"prepared":@NO};
        core3d::saved_cut_source_edit::Patch patch;
        if(fixture==0){core3d::saved_cut_source_values::EnclosurePatch value;value.dimensions[0]=widthMM/mm;patch=value;}
        else {core3d::saved_cut_source_values::PolygonPatch value;value.depth=depthMM/mm;
            value.coordinates={{1,core3d::saved_cut_source_values::Component::U,widthMM/mm},
                {2,core3d::saved_cut_source_values::Component::U,widthMM/mm}};patch=value;}
        const auto unchangedGuard=owner->CaptureSavedCutSceneState(before.source.original.label);
        const CGSize size=GLController.drawableSize;
        const auto lease=core3d::Core3DViewer::makeSavedCutSourceEditWork();
        if(!viewer->prepareSavedCutSourceEdit(lease,before,patch,identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height))))
            return @{@"phase":@"prepare",@"prepared":@NO};
        const auto geometry=core3d::Core3DViewer::savedCutSourceEditGeometry(lease);
        const auto token=core3d::Core3DViewer::savedCutSourceEditCancellation(lease);
        bool stopAccepted=false;
        if(cancelPoint==1)stopAccepted=core3d::Core3DViewer::cancelSavedCutSourceEdit(token);
        std::shared_ptr<const core3d::SavedCutSourceDetachedResult> built;
        // DEBUG harness only. This is the actual detached worker, no DTO/result injection.
        // Main is deliberately joined for deterministic fixture sequencing, not a product async API.
        if(geometry){std::thread worker([geometry,&built]{built=core3d::Core3DViewer::buildSavedCutSourceDetached(geometry);});worker.join();}
        if(!built){(void)viewer->discardSavedCutSourceEdit(lease);return @{@"phase":@"worker",@"prepared":@YES,@"built":@NO,
            @"stopAccepted":@(stopAccepted),@"unchangedScene":@(owner->SavedCutSceneStateMatches(unchangedGuard))};}
        if(cancelPoint==2)stopAccepted=core3d::Core3DViewer::cancelSavedCutSourceEdit(token);
        if(beforeCommit)beforeCommit();
        const auto outcome=viewer->commitSavedCutSourceEdit(lease,built);
        NSMutableDictionary *out=[@{@"phase":@"ordinary",@"prepared":@YES,@"built":@YES,
            @"outcome":@(static_cast<unsigned>(outcome)),@"committed":@(outcome==core3d::OrdinaryEditResult::Committed),
            @"stopAccepted":@(stopAccepted),@"unchangedScene":@(owner->SavedCutSceneStateMatches(unchangedGuard)),
            @"repeatedOutcome":@(static_cast<unsigned>(viewer->commitSavedCutSourceEdit(lease,built)))} mutableCopy];
        // Drop remains main-only, including cancelled/invalid currentness paths.
        (void)viewer->discardSavedCutSourceEdit(lease);
        OcctCylindricalCutSource current;
        if(owner.IsNull()||!owner->CaptureCylindricalCutSource(before.source.original.label,current)){
            out[@"phase"]=@"current-readback";return out;
        }
        core3d::saved_cut_source_edit::Values expectedValues;const std::atomic_bool running(false);
        const bool typed=before.source.original.retained.value
            &&core3d::saved_cut_source_edit::PrepareValues(*before.source.original.retained.value,patch,running,expectedValues)
            &&current.original.retained.value&&current.original.retained.value->bytes==expectedValues.newBytes;
        auto fixed=before.source.envelope;fixed.sourceValues=current.envelope.sourceValues;
        std::vector<std::uint8_t> fixedBytes;
        const bool stable=core3d::retained_solid::Encode(fixed,fixedBytes)&&current.original.retained.value
            &&fixedBytes==current.original.retained.value->bytes
            &&current.original.entityIdentifier==before.source.original.entityIdentifier
            &&current.original.definitionIdentifier==before.source.original.definitionIdentifier
            &&current.original.present==before.source.original.present
            &&core3d::sweep_rebuild::SameRawScalars(current.original.scalars,before.source.original.scalars);
        out[@"typedPatchBytes"]=@(typed);out[@"fixedEnvelopeIDsOccurrence"]=@(stable);
        NSMutableDictionary *checks=[NSMutableDictionary dictionary];
        for(const auto& row:(fixture==0?core3d::saved_cut_source_changed_probe::Enclosure(current,widthMM)
            :core3d::saved_cut_source_changed_probe::Bracket(current,widthMM,depthMM)))
            checks[[NSString stringWithUTF8String:row.first.c_str()]]=@(row.second);
        out[@"geometryChecks"]=checks;return out;
    }catch(...){return @{@"phase":@"exception",@"prepared":@NO};}
}
- (NSDictionary<NSString *,NSNumber *> *)debugSavedCutEnclosureGeometry:(NSString *)entity widthMM:(double)widthMM {
    if(!NSThread.isMainThread||!GLController||!GLController.viewer||!entity||entity.length==0||entity.length>128)return @{};
    try {
        // History redraw may clear selection. Observation uses the saved entity;
        // it must neither acquire edit authority nor alter the user's selection.
        const auto owner=GLController.viewer->getDocument();
        if(owner.IsNull()||owner->Document().IsNull()||owner->Document()->HasOpenCommand()
            ||GLController.viewer->hasUnresolvedOrdinaryEdit())return @{};
        TDF_LabelSequence roots;XCAFDoc_DocumentTool::ShapeTool(owner->Document()->Main())->GetFreeShapes(roots);
        if(roots.Length()>50000)return @{};
        TDF_Label target;
        for(int i=1;i<=roots.Length();++i)if(owner->EntityIdentifierForLabel(roots.Value(i))==(entity.UTF8String?:"")){
            if(!target.IsNull())return @{};target=roots.Value(i);
        }
        OcctCylindricalCutSource source;
        if(target.IsNull()||!owner->CaptureCylindricalCutSource(target,source))return @{};
        NSMutableDictionary *checks=[NSMutableDictionary dictionary];
        for(const auto& row:core3d::saved_cut_source_changed_probe::Enclosure(source,widthMM))
            checks[[NSString stringWithUTF8String:row.first.c_str()]]=@(row.second);
        return checks;
    }catch(...){return @{};}
}
- (NSDictionary<NSString *,NSNumber *> *)debugSavedCutBracketGeometry:(NSString *)entity lengthMM:(double)lengthMM depthMM:(double)depthMM {
    if(!NSThread.isMainThread||!GLController||!GLController.viewer||!entity||entity.length==0||entity.length>128)return @{};
    try {
        // History redraw may clear selection. Observation uses the saved entity;
        // it must neither acquire edit authority nor alter the user's selection.
        const auto owner=GLController.viewer->getDocument();
        if(owner.IsNull()||owner->Document().IsNull()||owner->Document()->HasOpenCommand()
            ||GLController.viewer->hasUnresolvedOrdinaryEdit())return @{};
        TDF_LabelSequence roots;XCAFDoc_DocumentTool::ShapeTool(owner->Document()->Main())->GetFreeShapes(roots);
        if(roots.Length()>50000)return @{};
        TDF_Label target;
        for(int i=1;i<=roots.Length();++i)if(owner->EntityIdentifierForLabel(roots.Value(i))==(entity.UTF8String?:"")){
            if(!target.IsNull())return @{};target=roots.Value(i);
        }
        OcctCylindricalCutSource source;
        if(target.IsNull()||!owner->CaptureCylindricalCutSource(target,source))return @{};
        NSMutableDictionary *checks=[NSMutableDictionary dictionary];
        for(const auto& row:core3d::saved_cut_source_changed_probe::Bracket(source,lengthMM,depthMM))
            checks[[NSString stringWithUTF8String:row.first.c_str()]]=@(row.second);
        return checks;
    }catch(...){return @{};}
}
- (NSDictionary<NSString *,id> *)debugCylindricalCutEvidence:(NSString *)entity {
    if(!NSThread.isMainThread||!GLController||!GLController.viewer||!entity||entity.length>128)return nil;
    try {
        const auto owner=GLController.viewer->getDocument();if(owner.IsNull()||owner->Document().IsNull())return nil;
        TDF_LabelSequence roots;XCAFDoc_DocumentTool::ShapeTool(owner->Document()->Main())->GetFreeShapes(roots);if(roots.Length()>50000)return nil;
        TDF_Label target;for(int i=1;i<=roots.Length();++i)if(owner->EntityIdentifierForLabel(roots.Value(i))==(entity.UTF8String?:"")){if(!target.IsNull())return nil;target=roots.Value(i);}
        OcctCylindricalCutSource source;if(!owner->CaptureCylindricalCutSource(target,source))return nil;
        const auto& state=source.original;GProp_GProps actual,base;
        BRepGProp::VolumeProperties(state.shape,actual);BRepGProp::VolumeProperties(source.base,base);
        double unit=0;if(!XCAFDoc_DocumentTool::GetLengthUnit(owner->Document(),unit))return nil;
        const double mm=unit*1000;Bnd_Box box;BRepBndLib::AddOptimal(state.shape,box,Standard_False,Standard_False);
        if(box.IsVoid()||box.IsOpen())return nil;double bounds[6];box.Get(bounds[0],bounds[1],bounds[2],bounds[3],bounds[4],bounds[5]);
        NSMutableArray *boxMM=[NSMutableArray array],*bits=[NSMutableArray array],*matrix=[NSMutableArray array],*present=[NSMutableArray array],*recipe=[NSMutableArray array];
        for(double x:bounds)[boxMM addObject:@(x*mm)];for(double x:state.scalars)[bits addObject:@(core3d::retained_solid::Bits(x))];
        for(bool x:state.present)[present addObject:@(x)];for(int i=1;i<=3;++i)for(int j=1;j<=4;++j)[matrix addObject:@(state.transform.Value(i,j))];
        for(double x:source.envelope.sourceValues)[recipe addObject:@(core3d::retained_solid::Bits(x))];
        OcctScalarAppearanceState appearance;if(!owner->CaptureScalarAppearanceForSavedCut(target,appearance))return nil;
        NSMutableArray *appearanceBits=[NSMutableArray array];for(double x:appearance.visualValues)[appearanceBits addObject:@(core3d::retained_solid::Bits(x))];
        for(int i=0;i<2;++i){[appearanceBits addObject:@(appearance.legacyPresent[i])];[appearanceBits addObject:@(appearance.legacyValues[i])];}[appearanceBits addObject:@(appearance.localPBR)];
        NSMutableDictionary *out=[@{@"entity":entity,@"definition":[NSString stringWithUTF8String:state.definitionIdentifier.c_str()],
            @"sourceFeature":[NSString stringWithUTF8String:core3d::retained_solid::UUIDText(source.envelope.sourceFeature).c_str()],
            @"sourceBits":recipe,@"transformBits":bits,@"present":present,@"matrix":matrix,@"appearanceBits":appearanceBits,
            @"boundsMM":boxMM,@"volumeMM3":@(actual.Mass()*mm*mm*mm),@"baseVolumeMM3":@(base.Mass()*mm*mm*mm),
            @"rebuilding":@(source.rebuilding),@"effectiveMM":@(source.effectiveMM)} mutableCopy];
        if(state.retained.value){const auto& payload=*state.retained.value;
            out[@"envelope"]=[NSData dataWithBytes:payload.bytes.data() length:payload.bytes.size()];
            out[@"feature"]=[NSString stringWithUTF8String:core3d::retained_solid::UUIDText(std::get<core3d::retained_solid::Envelope>(payload.envelope).derivedFeature).c_str()];
            out[@"radiusBits"]=@(core3d::retained_solid::Bits(std::get<core3d::retained_solid::Envelope>(payload.envelope).radius));
            out[@"axis"]=@(static_cast<unsigned>(std::get<core3d::retained_solid::Envelope>(payload.envelope).axis));
            out[@"worldRadiusMM"]=@(std::get<core3d::retained_solid::Envelope>(payload.envelope).radius*source.effectiveMM);
        }
        return out;
    }catch(...){return nil;}
}
- (NSDictionary<NSString *,id> *)debugCylindricalCutProgramEvidence:(NSString *)entity {
    if(!NSThread.isMainThread||!GLController||!GLController.viewer||!entity||entity.length>128)return nil;
    try {
        // Read-only whole-program observation of the exact saved entity. No
        // selection, edit or command authority is acquired or altered. An open
        // OCAF command or an unresolved ordinary edit refuses observation at
        // this DEBUG boundary, matching the neighboring geometry observers.
        const auto owner=GLController.viewer->getDocument();
        if(owner.IsNull()||owner->Document().IsNull()||owner->Document()->HasOpenCommand()
            ||GLController.viewer->hasUnresolvedOrdinaryEdit())return nil;
        TDF_LabelSequence roots;XCAFDoc_DocumentTool::ShapeTool(owner->Document()->Main())->GetFreeShapes(roots);if(roots.Length()>50000)return nil;
        TDF_Label target;for(int i=1;i<=roots.Length();++i)if(owner->EntityIdentifierForLabel(roots.Value(i))==(entity.UTF8String?:"")){if(!target.IsNull())return nil;target=roots.Value(i);}
        OcctCylindricalCutProgramSource source;if(target.IsNull()||!owner->CaptureCylindricalCutProgramSource(target,source))return nil;
        const auto& state=source.original;GProp_GProps actual,base;
        BRepGProp::VolumeProperties(state.shape,actual);BRepGProp::VolumeProperties(source.base,base);
        double unit=0;if(!XCAFDoc_DocumentTool::GetLengthUnit(owner->Document(),unit))return nil;
        const double mm=unit*1000;Bnd_Box box;BRepBndLib::AddOptimal(state.shape,box,Standard_False,Standard_False);
        if(box.IsVoid()||box.IsOpen())return nil;double bounds[6];box.Get(bounds[0],bounds[1],bounds[2],bounds[3],bounds[4],bounds[5]);
        NSMutableArray *boxMM=[NSMutableArray array],*bits=[NSMutableArray array],*matrix=[NSMutableArray array],*present=[NSMutableArray array],*recipe=[NSMutableArray array];
        for(double x:bounds)[boxMM addObject:@(x*mm)];for(double x:state.scalars)[bits addObject:@(core3d::retained_solid::Bits(x))];
        for(bool x:state.present)[present addObject:@(x)];for(int i=1;i<=3;++i)for(int j=1;j<=4;++j)[matrix addObject:@(state.transform.Value(i,j))];
        const auto identity=core3d::retained_boolean::Identities(source.recipe);
        std::vector<double> sourceValues;std::uint64_t nextOperand=0;
        NSMutableArray *bores=[NSMutableArray array];
        const auto appendBore=[&](const core3d::analytic_boolean::Operand& operand){
            NSMutableArray *pointBits=[NSMutableArray array];
            for(double x:operand.point)[pointBits addObject:@(core3d::retained_solid::Bits(x))];
            [bores addObject:@{@"operand":@(operand.identifier),@"axis":@(static_cast<unsigned>(operand.axis)),
                @"pointBits":pointBits,@"radiusBits":@(core3d::retained_solid::Bits(operand.radius)),
                @"worldRadiusMM":@(operand.radius*source.effectiveMM)}];
        };
        if(const auto* legacy=std::get_if<core3d::retained_solid::Envelope>(&source.recipe)){
            sourceValues=legacy->sourceValues;nextOperand=std::uint64_t(legacy->operandID)+1;
            core3d::analytic_boolean::Operand operand;operand.identifier=legacy->operandID;
            operand.axis=static_cast<core3d::analytic_boolean::Axis>(legacy->axis);
            operand.point=legacy->point;operand.radius=legacy->radius;appendBore(operand);
        }else{
            const auto& program=std::get<core3d::retained_boolean::Program>(source.recipe);
            sourceValues=program.source.values;nextOperand=program.nextOperandID;
            for(const auto& step:program.steps)appendBore(step.operand);
        }
        for(double x:sourceValues)[recipe addObject:@(core3d::retained_solid::Bits(x))];
        OcctScalarAppearanceState appearance;if(!owner->CaptureScalarAppearanceForSavedCut(target,appearance))return nil;
        NSMutableArray *appearanceBits=[NSMutableArray array];for(double x:appearance.visualValues)[appearanceBits addObject:@(core3d::retained_solid::Bits(x))];
        for(int i=0;i<2;++i){[appearanceBits addObject:@(appearance.legacyPresent[i])];[appearanceBits addObject:@(appearance.legacyValues[i])];}[appearanceBits addObject:@(appearance.localPBR)];
        return @{@"entity":entity,@"definition":[NSString stringWithUTF8String:state.definitionIdentifier.c_str()],
            @"sourceFeature":[NSString stringWithUTF8String:core3d::retained_solid::UUIDText(identity.sourceFeature).c_str()],
            @"feature":[NSString stringWithUTF8String:core3d::retained_solid::UUIDText(identity.derivedFeature).c_str()],
            @"sourceBits":recipe,@"transformBits":bits,@"present":present,@"matrix":matrix,@"appearanceBits":appearanceBits,
            @"boundsMM":boxMM,@"volumeMM3":@(actual.Mass()*mm*mm*mm),@"baseVolumeMM3":@(base.Mass()*mm*mm*mm),
            @"effectiveMM":@(source.effectiveMM),@"bores":bores,@"nextOperand":@(nextOperand),
            @"program":[NSData dataWithBytes:source.recipeBytes.data() length:source.recipeBytes.size()]};
    }catch(...){return nil;}
}
- (NSDictionary<NSString *,id> *)debugCylindricalCutHoleFaces:(NSString *)entity {
    if(!NSThread.isMainThread||!GLController||!GLController.viewer||!entity||entity.length>128)return nil;
    try {
        // Bounded DEBUG read-only observation of the ACTUAL cylindrical hole
        // faces of the retained result solid. Every reported value is measured
        // from kernel surfaces and trim bounds, never projected from the stored
        // recipe, and the production program matcher is not called. An open
        // OCAF command or an unresolved ordinary edit refuses the observation
        // outright at this DEBUG boundary; the shared capture itself stays
        // Stage-compatible and unchanged. No selection, edit or command
        // authority is acquired.
        const auto owner=GLController.viewer->getDocument();if(owner.IsNull()||owner->Document().IsNull())return nil;
        if(owner->Document()->HasOpenCommand()||GLController.viewer->hasUnresolvedOrdinaryEdit())return nil;
        TDF_LabelSequence roots;XCAFDoc_DocumentTool::ShapeTool(owner->Document()->Main())->GetFreeShapes(roots);if(roots.Length()>50000)return nil;
        TDF_Label target;for(int i=1;i<=roots.Length();++i)if(owner->EntityIdentifierForLabel(roots.Value(i))==(entity.UTF8String?:"")){if(!target.IsNull())return nil;target=roots.Value(i);}
        OcctCylindricalCutProgramSource source;if(target.IsNull()||!owner->CaptureCylindricalCutProgramSource(target,source))return nil;
        double unit=0;if(!XCAFDoc_DocumentTool::GetLengthUnit(owner->Document(),unit))return nil;
        const double mm=unit*1000;int faces=0;NSMutableArray *holes=[NSMutableArray array];
        for(TopExp_Explorer it(source.original.shape,TopAbs_FACE);it.More();it.Next()){
            if(++faces>10000)return nil;
            const auto face=TopoDS::Face(it.Current());
            if(face.Orientation()!=TopAbs_FORWARD&&face.Orientation()!=TopAbs_REVERSED)return nil;
            const BRepAdaptor_Surface surface(face,Standard_True);
            if(surface.GetType()!=GeomAbs_Cylinder)continue;
            const auto cylinder=surface.Cylinder();const auto axis=cylinder.Axis();
            [holes addObject:@{@"axis":@[@(axis.Direction().X()),@(axis.Direction().Y()),@(axis.Direction().Z())],
                @"location":@[@(axis.Location().X()*mm),@(axis.Location().Y()*mm),@(axis.Location().Z()*mm)],
                @"radiusMM":@(cylinder.Radius()*mm),
                @"uSpan":@(surface.LastUParameter()-surface.FirstUParameter()),
                @"vMinMM":@(surface.FirstVParameter()*mm),@"vMaxMM":@(surface.LastVParameter()*mm),
                @"reversed":@(face.Orientation()==TopAbs_REVERSED)}];
        }
        return @{@"entity":entity,@"faces":@(faces),@"holes":holes,
            @"valid":@(BRepCheck_Analyzer(source.original.shape,Standard_True).IsValid()?YES:NO)};
    }catch(...){return nil;}
}
- (NSDictionary<NSString *,id> *)debugScalarPBREvidence:(NSString *)entity {
    if(![NSThread isMainThread]||!entity||entity.length>128||!GLController||!GLController.viewer)return nil;
    try{
        auto owner=GLController.viewer->getDocument();if(owner.IsNull()||owner->Document().IsNull())return nil;
        TDF_LabelSequence roots;XCAFDoc_DocumentTool::ShapeTool(owner->Document()->Main())->GetFreeShapes(roots);if(roots.Length()>50000)return nil;
        TDF_Label target;for(int i=1;i<=roots.Length();++i)if(owner->EntityIdentifierForLabel(roots.Value(i))==(entity.UTF8String?:"")){if(!target.IsNull())return nil;target=roots.Value(i);}
        const auto e=owner->DebugPBRScalarEvidence(target);if(!e)return nil;
        NSMutableDictionary* geometry=[NSMutableDictionary dictionary];for(const auto& [key,digest]:e->geometry)geometry[[NSString stringWithUTF8String:key.c_str()]]=[NSData dataWithBytes:digest.data() length:digest.size()];
        NSMutableDictionary* streams=[NSMutableDictionary dictionary];for(const auto& [key,bytes]:e->geometryStreams)streams[[NSString stringWithUTF8String:key.c_str()]]=[NSData dataWithBytes:bytes.data() length:bytes.size()];
        return @{@"geometryStreams":streams,@"materialBytes":[NSData dataWithBytes:e->material.data() length:e->material.size()],@"preservedBytes":[NSData dataWithBytes:e->preserved.data() length:e->preserved.size()],
            @"tableBytes":[NSData dataWithBytes:e->table.data() length:e->table.size()],@"geometry":geometry};
    }catch(...){return nil;}
}
- (BOOL)debugSeedScalarPBRCommonMismatch {
    if(![NSThread isMainThread]||!GLController||!GLController.viewer||![self core3d_canBeginCommittedEdit])return NO;
    auto viewer=GLController.viewer;auto owner=viewer->getDocument();auto context=viewer->AisContext();
    if(owner.IsNull()||context.IsNull()||context->NbSelected()!=1)return NO;
    auto document=owner->Document();if(document.IsNull()||document->HasOpenCommand())return NO;
    try{
        context->InitSelected();const auto shape=Handle(AIS_Shape)::DownCast(context->SelectedInteractive());const auto label=owner->ShapeLabel(shape);
        const auto material=XCAFDoc_VisMaterialTool::GetShapeMaterial(label);if(material.IsNull()||!material->HasPbrMaterial())return NO;
        auto common=material->CommonMaterial();common.IsDefined=true;common.AmbientColor=Quantity_Color(.11,.12,.13,Quantity_TOC_RGB);
        common.DiffuseColor=Quantity_Color(.6,.4,.2,Quantity_TOC_RGB);common.SpecularColor=Quantity_Color(.3,.4,.5,Quantity_TOC_RGB);
        common.EmissiveColor=Quantity_Color(.1,.2,.3,Quantity_TOC_RGB);common.Shininess=.17f;common.Transparency=.23f;
        document->NewCommand();if(!document->HasOpenCommand())return NO;material->SetCommonMaterial(common);
        if(!document->CommitCommand()){Core3DAbortCommandNoThrow(document);return NO;}
        owner->NotifyChanges();owner->LoadObjectMeterial(label,shape);context->Redisplay(shape,Standard_False);context->UpdateCurrentViewer();return YES;
    }catch(...){Core3DAbortCommandNoThrow(document);return NO;}
}
#endif

- (Core3DPBRScalarPreparation *)prepareScalarPBREdit:(Core3DPBRScalarEdit *)edit expected:(Core3DSceneSnapshot *)expected {
    if(![NSThread isMainThread]||!edit||edit.class!=Core3DPBRScalarEdit.class||!expected||expected.class!=Core3DSceneSnapshot.class||!_isSetuped||!GLController||!GLController.viewer
        ||!GLController.viewer->canBeginCommittedEdit())return nil;
    const CGSize size=GLController.drawableSize;
    if(!std::isfinite(size.width)||!std::isfinite(size.height)||size.width<1||size.height<1
        ||size.width>UINT32_MAX||size.height>UINT32_MAX)return nil;
    try{
        core3d::ObjectFrameIdentity identity;const char* source=expected.publicationSourceIdentifier.UTF8String;if(!source)return nil;
        identity.publicationSourceIdentifier.assign(source,[expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration=expected.revisions.documentGeneration;identity.modelRevision=expected.revisions.modelRevision;
        OcctPBRScalarPatch patch;if(edit.baseColorSRGB){if(edit.baseColorSRGB.count!=3)return nil;patch.baseColorSRGB=std::array<double,3>{edit.baseColorSRGB[0].doubleValue,edit.baseColorSRGB[1].doubleValue,edit.baseColorSRGB[2].doubleValue};}
        if(edit.metallic)patch.metallic=edit.metallic.doubleValue;if(edit.roughness)patch.roughness=edit.roughness.doubleValue;
        auto work=GLController.viewer->preparePBRScalar(patch,identity,expected.revisions.presentationRevision,std::uint32_t(std::llround(size.width)),std::uint32_t(std::llround(size.height)));
        if(!work)return nil;auto prepared=[[Core3DPBRScalarPreparation alloc]initPrivate];if(!prepared)return nil;
        prepared->_scalarOwner=self;prepared->_scalarWork=std::move(work);return prepared;
    }catch(...){return nil;}
}
- (void)cancelScalarPBRPreparation:(Core3DPBRScalarPreparation *)prepared {
    if(![NSThread isMainThread]||!prepared||prepared.class!=Core3DPBRScalarPreparation.class||prepared->_scalarOwner!=self)return;
    auto work=std::move(prepared->_scalarWork);
    if(GLController&&GLController.viewer)GLController.viewer->cancelPBRScalar(work);
}
- (Core3DPBRScalarResult)executeScalarPBRPreparation:(Core3DPBRScalarPreparation *)prepared {
    if(![NSThread isMainThread]||!prepared||prepared.class!=Core3DPBRScalarPreparation.class||prepared->_scalarOwner!=self
        ||!prepared->_scalarWork||!GLController||!GLController.viewer)return Core3DPBRScalarResultInvalid;
    auto work=std::move(prepared->_scalarWork); // Consume before any publication/reentry.
    const auto viewer=GLController.viewer;const auto result=viewer->executePBRScalar(work);
    if(result==core3d::OrdinaryEditResult::Committed||result==core3d::OrdinaryEditResult::NoChange){
        // Actual durable result is already sealed; publication cannot downgrade it.
        try{
            auto context=viewer->AisContext();auto doc=viewer->getDocument();NSMutableArray<Core3DPBRMaterial*>* values=[NSMutableArray array];
            if(!context.IsNull()&&!doc.IsNull())for(context->InitSelected();context->MoreSelected();context->NextSelected()){
                const auto label=doc->ShapeLabel(context->SelectedInteractive());XCAFDoc_VisMaterialPBR p;
                if(!label.IsNull()&&doc->TryEffectivePBRMaterialForLabel(label,p)){
                    auto value=Core3DMakePBRMaterial(p,doc->SupportsScalarPBRMaterialEditingForLabel(label),doc->SupportsBaseColorTextureEditingForLabel(label),
                        doc->SupportsEmissiveTextureEditingForLabel(label),doc->SupportsMaterialTextureEditingForLabel(label,OcctMaterialTextureSlot::MetallicRoughness),
                        doc->SupportsMaterialTextureEditingForLabel(label,OcctMaterialTextureSlot::Occlusion),doc->SupportsMaterialTextureEditingForLabel(label,OcctMaterialTextureSlot::Normal));
                    if(value)[values addObject:value];
                }
            }
            [self.materialController didChangeSelectionWithMaterials:@[] colors:@[]];[self.materialController didChangeSelectionWithPBRMaterials:values];
            [self sendNotifyUIState:UIStateChangingApplyMaterial|UIStateChangingHistory];
        }catch(...){}
    }
    switch(result){
        case core3d::OrdinaryEditResult::Committed:return Core3DPBRScalarResultCommitted;
        case core3d::OrdinaryEditResult::NoChange:return Core3DPBRScalarResultUnchanged;
        case core3d::OrdinaryEditResult::Busy:return Core3DPBRScalarResultBusy;
        case core3d::OrdinaryEditResult::OutcomeUnknown:return Core3DPBRScalarResultOutcomeUnknown;
        default:return Core3DPBRScalarResultInvalid;
    }
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

    // Existing single-selected touch values now share immutable source/ledger
    // authority. Multiselection keeps the unchanged legacy batch path below.
    auto selectedContext=GLController.viewer->AisContext();
    bool preserveLegacyStamping=false;
    if(!selectedContext.IsNull()&&selectedContext->NbSelected()==1){
        selectedContext->InitSelected();const auto doc=GLController.viewer->getDocument();
        if(!doc.IsNull())preserveLegacyStamping=doc->StoredGeometryRepresentationForLabel(doc->ShapeLabel(selectedContext->SelectedInteractive()))==OcctGeometryRepresentation::LegacyUnknown;
    }
    // The existing legacy-touch contract stamps BRep inside its ordinary color
    // command. Choose that contract before any prepared admission, never as a retry.
    if(!selectedContext.IsNull()&&selectedContext->NbSelected()==1&&!preserveLegacyStamping){
        CGFloat r=0,g=0,b=0,a=0;
        if(![material.baseColor getRed:&r green:&g blue:&b alpha:&a]||!std::isfinite(r)||!std::isfinite(g)||!std::isfinite(b)
            ||!std::isfinite(a)||a<0||a>1)return;
        auto edit=[[Core3DPBRScalarEdit alloc]initWithBaseColorSRGB:@[@(std::clamp(r,CGFloat(0),CGFloat(1))),@(std::clamp(g,CGFloat(0),CGFloat(1))),@(std::clamp(b,CGFloat(0),CGFloat(1)))]
            metallic:@(material.metallic) roughness:@(material.roughness)];
        auto snapshot=[self captureSceneSnapshot];
        auto prepared=edit&&snapshot?[self prepareScalarPBREdit:edit expected:snapshot]:nil;
        if(prepared)[self executeScalarPBRPreparation:prepared];
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

- (BOOL)debugProbeMeshVertexStorageChange:(NSInteger)mode {
    if (![NSThread isMainThread] || mode < 0 || mode > 8
        || GLController == nil || GLController.viewer == nullptr) return NO;
    return GLController.viewer->debugProbeMeshVertexStorageChange(static_cast<int>(mode));
}

- (BOOL)debugConfigureMeshCopyFault:(NSInteger)mode {
    if (mode<0 || mode>16 || ![self debugConfigureOrdinaryCreationFault:mode<=13?mode:0]) return NO;
    const auto controller=GLController.viewer->debugOrdinaryEditController();
    if (mode>=14) controller->debugSetStageFailureIndex(mode==14?2:mode==15?3:0);
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

// DEBUG-only standalone import fixture; never mutates the live document.
// DEBUG-only real-document admission probe. All setup is in a standalone OCAF
// document; the production limits and CanDuplicateGeometryDefinitions are used.
- (NSDictionary<NSString *, NSNumber *> *)debugEnclosureCopyOwnershipState {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr) return nil;
    try {
        const auto owner = GLController.viewer->getDocument();
        if (owner.IsNull() || !owner->ValidateGeometryRepresentations()) return nil;
        const auto document = owner->Document();
        if (document.IsNull() || document->HasOpenCommand()) return nil;
        const auto tool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (tool.IsNull()) return nil;
        TDF_LabelSequence roots; tool->GetFreeShapes(roots);
        if (roots.Length() > 4096) return nil;
        std::unordered_set<const TopoDS_TShape *> previousTopologies;
        std::set<std::string> identities;
        Standard_Size enclosures = 0, current = 0, retainedBindings = 0, allNodes = 0;
        bool independent = true, uniqueIdentifiers = true;
        for (int index = 1; index <= roots.Length(); ++index) {
            const auto label = roots.Value(index);
            core3d::enclosure::Record record;
            if (!core3d::enclosure::Read(document, label, record)) return nil;
            if (record.label.IsNull()) continue;
            ++enclosures;
            uniqueIdentifiers = identities.insert(record.identifier).second && uniqueIdentifiers;
            current += record.IsCurrent(document, label) ? 1 : 0;
            const auto root = XCAFDoc_ShapeTool::GetShape(label);
            if (root.IsNull() || record.boundShape.IsNull()) return nil;
            const bool sameRoot = record.boundShape.IsEqual(root);
            retainedBindings += sameRoot ? 0 : 1;
            std::unordered_set<const TopoDS_TShape *> ownTopologies;
            const auto include = [&](const TopoDS_Shape& shape) {
                TopTools_IndexedMapOfShape nodes; TopExp::MapShapes(shape, nodes);
                for (int node = 1; node <= nodes.Extent(); ++node)
                    ownTopologies.insert(nodes(node).TShape().get());
            };
            include(root); if (!sameRoot) include(record.boundShape);
            if (ownTopologies.size() > 131072U - allNodes) return nil;
            allNodes += ownTopologies.size();
            for (const auto topology : ownTopologies)
                if (!previousTopologies.insert(topology).second) independent = false;
        }
        return @{@"enclosureCount":@(enclosures), @"currentCount":@(current),
            @"retainedBindingCount":@(retainedBindings), @"topologyNodes":@(allNodes),
            @"pairwiseIndependentRootsAndBindings":@(independent),
            @"uniqueFeatureIdentifiers":@(uniqueIdentifiers)};
    } catch (...) { return nil; }
}

// Modes0/1: exact label boundary for legacy9/framed17 scalar recipes.
// Mode2: current root + separate stale bound topology multiplicity.
- (NSDictionary<NSString *, NSNumber *> *_Nullable)debugEnclosureDuplicateCapacityProbe:(NSInteger)mode {
    if (![NSThread isMainThread] || mode < 0 || mode > 2) return nil;
    struct Scope {
        Handle(OcctDocument) wrapper = new OcctDocument();
        Handle(TDocStd_Document) document;
        ~Scope() noexcept { try { if (!document.IsNull()) {
            if (document->HasOpenCommand()) document->AbortCommand();
            const auto app = Handle(TDocStd_Application)::DownCast(document->Application());
            if (!app.IsNull()) app->Close(document);
        } } catch (...) {} }
    } scope;
    try {
        scope.wrapper->InitDoc(); scope.document = scope.wrapper->Document();
        const auto& document = scope.document;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        core3d::enclosure::Parameters parameters;
        parameters.metersPerUnit = 0.001;
        parameters.definition.dimensions = {100,60,30,2,2,4};
        parameters.definition.plane = 0;
        if (mode != 0) {
            parameters.definition.constructionFrame.emplace();
            parameters.definition.constructionFrame->values = {13,-7,2,0,0,0.25881904510252074,0.9659258262890683,-1.25};
        }
        core3d::EnclosureSolidResult geometry;
        if (!core3d::BuildEnclosureSolidGeometry(parameters.definition,
                std::make_shared<std::atomic_bool>(false), geometry)) return nil;
        const auto original = geometry.solid;
        document->NewCommand();
        const auto label = shapes->AddShape(original, Standard_False);
        if (!core3d::enclosure::Stage(document, label, parameters, NSUUID.UUID.UUIDString.UTF8String)
            || !document->CommitCommand()) return nil;
        core3d::enclosure::Record record;
        if (!core3d::enclosure::Read(document, label, record) || !record.IsCurrent(document, label)) return nil;
        Standard_Size admittedCopies = 1, shapeCost = 0;
        auto countLabels = [&]() {
            Standard_Size count = 0;
            for (TDF_ChildIterator it(document->GetData()->Root(), Standard_True); it.More(); it.Next()) ++count;
            return count;
        };
        if (mode <= 1) {
            // Exact production 100000-label boundary: a new definition and its
            // eight transform labels, plus the recipe and its actual scalars.
            const Standard_Size destinationLabels = 9U + 1U + record.values.size();
            const Standard_Size target = 100000U - destinationLabels;
            const auto filler = document->GetData()->Root().FindChild(900000, Standard_True);
            const Standard_Size before = countLabels();
            if (before >= target) return nil;
            for (Standard_Size i = 1; i <= target - before; ++i)
                filler.FindChild(static_cast<Standard_Integer>(i), Standard_True);
            if (countLabels() != target) return nil;
        } else {
            // Retain the original recipe binding while replacing its root with
            // a distinct larger box, just as a later solid edit can do.
            const auto later = BRepPrimAPI_MakeBox(65.0, 40.0, 30.0).Shape();
            document->NewCommand(); shapes->SetShape(label, later);
            if (!document->CommitCommand()) return nil;
            core3d::enclosure::Record stale;
            if (!core3d::enclosure::Read(document, label, stale) || !stale.IsEqual(record)
                || stale.IsCurrent(document, label)) return nil;
            TopTools_IndexedMapOfShape currentTopology, retainedTopology;
            TopExp::MapShapes(later, currentTopology); TopExp::MapShapes(original, retainedTopology);
            shapeCost = currentTopology.Extent() + retainedTopology.Extent();
            if (shapeCost == 0 || shapeCost >= 131072U) return nil;
            admittedCopies = (131072U - shapeCost) / shapeCost;
            // At this boundary the definition and label limits remain loose.
            if (admittedCopies < 2 || admittedCopies + 2 >= 4096U
                || countLabels() + (admittedCopies + 1) * (10U + record.values.size()) >= 100000U) return nil;
        }
        const auto labelsBefore = countLabels();
        const auto timeBefore = document->GetData()->Time();
        const auto undoBefore = document->GetAvailableUndos();
        const auto rootBefore = XCAFDoc_ShapeTool::GetShape(label);
        const bool valid = scope.wrapper->ValidateGeometryRepresentations();
        const bool defaultRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, 1U}});
        const bool bakingRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, 1U, true, true}});
        const bool atBoundary = scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, admittedCopies, false, true}});
        const bool overflowRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, std::numeric_limits<Standard_Size>::max(), false, true}});
        const bool zeroRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, 0U, false, true}});
        const bool duplicateRequestRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, 1U, false, true}, {label, 1U, false, true}});
        core3d::enclosure::Record after;
        const bool queryUnchanged = labelsBefore == countLabels() && timeBefore == document->GetData()->Time()
            && undoBefore == document->GetAvailableUndos() && !document->HasOpenCommand()
            && rootBefore.IsEqual(XCAFDoc_ShapeTool::GetShape(label))
            && core3d::enclosure::Read(document, label, after) && after.IsEqual(record);
        bool beyondRejected = false;
        if (mode <= 1) {
            const auto filler = document->GetData()->Root().FindChild(900000, Standard_False);
            filler.FindChild(200000, Standard_True); // Exactly one extra label.
            if (countLabels() != labelsBefore + 1U) return nil;
            beyondRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
                std::vector<OcctGeometryDuplicationRequest>{{label, 1U, false, true}});
        } else {
            beyondRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
                std::vector<OcctGeometryDuplicationRequest>{{label, admittedCopies + 1U, false, true}});
        }
        return @{@"defaultRejected":@(defaultRejected), @"bakingRejected":@(bakingRejected), @"valid": @(valid), @"atBoundary": @(atBoundary), @"beyondRejected": @(beyondRejected),
            @"overflowRejected": @(overflowRejected), @"zeroRejected": @(zeroRejected),
            @"duplicateRequestRejected": @(duplicateRequestRejected), @"queryUnchanged": @(queryUnchanged),
            @"labelsBefore": @(labelsBefore), @"admittedCopies": @(admittedCopies), @"shapeCost": @(shapeCost)};
    } catch (...) { return nil; }
}

- (NSDictionary<NSString *, NSNumber *> *_Nullable)debugProfileDuplicateCapacityProbe:(NSInteger)mode {
    if (![NSThread isMainThread] || mode < 0 || mode > 1) return nil;
    struct Scope {
        Handle(OcctDocument) wrapper = new OcctDocument();
        Handle(TDocStd_Document) document;
        ~Scope() noexcept { try { if (!document.IsNull()) {
            if (document->HasOpenCommand()) document->AbortCommand();
            const auto app = Handle(TDocStd_Application)::DownCast(document->Application());
            if (!app.IsNull()) app->Close(document);
        } } catch (...) {} }
    } scope;
    try {
        scope.wrapper->InitDoc(); scope.document = scope.wrapper->Document();
        const auto& document = scope.document;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        const auto original = BRepPrimAPI_MakeBox(60.0, 40.0, 30.0).Shape();
        document->NewCommand();
        const auto label = shapes->AddShape(original, Standard_False);
        core3d::profile::Parameters parameters;
        parameters.metersPerUnit = 0.001;
        parameters.definition.points = {gp_Pnt2d(0,0), gp_Pnt2d(60,0), gp_Pnt2d(60,40), gp_Pnt2d(0,40)};
        parameters.definition.plane = 0; parameters.definition.depth = 30;
        if (!core3d::profile::Stage(document, label, parameters, NSUUID.UUID.UUIDString.UTF8String)
            || !document->CommitCommand()) return nil;
        core3d::profile::Record record;
        if (!core3d::profile::Read(document, label, record) || !record.IsCurrent(document, label)) return nil;
        Standard_Size admittedCopies = 1, shapeCost = 0;
        auto countLabels = [&]() {
            Standard_Size count = 0;
            for (TDF_ChildIterator it(document->GetData()->Root(), Standard_True); it.More(); it.Next()) ++count;
            return count;
        };
        if (mode == 0) {
            // Exact production 100000-label boundary: a new definition and its
            // eight transform labels, plus the recipe and its actual scalars.
            const Standard_Size destinationLabels = 9U + 1U + record.values.size();
            const Standard_Size target = 100000U - destinationLabels;
            const auto filler = document->GetData()->Root().FindChild(900000, Standard_True);
            const Standard_Size before = countLabels();
            if (before >= target) return nil;
            for (Standard_Size i = 1; i <= target - before; ++i)
                filler.FindChild(static_cast<Standard_Integer>(i), Standard_True);
            if (countLabels() != target) return nil;
        } else {
            // Retain the original recipe binding while replacing its root with
            // a distinct larger box, just as a later solid edit can do.
            const auto later = BRepPrimAPI_MakeBox(65.0, 40.0, 30.0).Shape();
            document->NewCommand(); shapes->SetShape(label, later);
            if (!document->CommitCommand()) return nil;
            core3d::profile::Record stale;
            if (!core3d::profile::Read(document, label, stale) || !stale.IsEqual(record)
                || stale.IsCurrent(document, label)) return nil;
            TopTools_IndexedMapOfShape currentTopology, retainedTopology;
            TopExp::MapShapes(later, currentTopology); TopExp::MapShapes(original, retainedTopology);
            shapeCost = currentTopology.Extent() + retainedTopology.Extent();
            if (shapeCost == 0 || shapeCost >= 131072U) return nil;
            admittedCopies = (131072U - shapeCost) / shapeCost;
            // At this boundary the definition and label limits remain loose.
            if (admittedCopies < 2 || admittedCopies + 2 >= 4096U
                || countLabels() + (admittedCopies + 1) * (10U + record.values.size()) >= 100000U) return nil;
        }
        const auto labelsBefore = countLabels();
        const auto timeBefore = document->GetData()->Time();
        const auto undoBefore = document->GetAvailableUndos();
        const auto rootBefore = XCAFDoc_ShapeTool::GetShape(label);
        const bool valid = scope.wrapper->ValidateGeometryRepresentations();
        const bool atBoundary = scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, admittedCopies}});
        const bool overflowRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, std::numeric_limits<Standard_Size>::max()}});
        const bool zeroRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, 0U}});
        const bool duplicateRequestRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, 1U}, {label, 1U}});
        core3d::profile::Record after;
        const bool queryUnchanged = labelsBefore == countLabels() && timeBefore == document->GetData()->Time()
            && undoBefore == document->GetAvailableUndos() && !document->HasOpenCommand()
            && rootBefore.IsEqual(XCAFDoc_ShapeTool::GetShape(label))
            && core3d::profile::Read(document, label, after) && after.IsEqual(record);
        bool beyondRejected = false;
        if (mode == 0) {
            const auto filler = document->GetData()->Root().FindChild(900000, Standard_False);
            filler.FindChild(200000, Standard_True); // Exactly one extra label.
            if (countLabels() != labelsBefore + 1U) return nil;
            beyondRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
                std::vector<OcctGeometryDuplicationRequest>{{label, 1U}});
        } else {
            beyondRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
                std::vector<OcctGeometryDuplicationRequest>{{label, admittedCopies + 1U}});
        }
        return @{@"valid": @(valid), @"atBoundary": @(atBoundary), @"beyondRejected": @(beyondRejected),
            @"overflowRejected": @(overflowRejected), @"zeroRejected": @(zeroRejected),
            @"duplicateRequestRejected": @(duplicateRequestRejected), @"queryUnchanged": @(queryUnchanged),
            @"labelsBefore": @(labelsBefore), @"admittedCopies": @(admittedCopies), @"shapeCost": @(shapeCost)};
    } catch (...) { return nil; }
}

- (NSDictionary<NSString *, NSNumber *> *_Nullable)debugEnclosureMirrorCapacityProbe:(NSInteger)mode {
    if (![NSThread isMainThread] || mode < 0 || mode > 1) return nil;
    struct Scope {
        Handle(OcctDocument) wrapper = new OcctDocument();
        Handle(TDocStd_Document) document;
        ~Scope() noexcept { try { if (!document.IsNull()) {
            if (document->HasOpenCommand()) document->AbortCommand();
            const auto app = Handle(TDocStd_Application)::DownCast(document->Application());
            if (!app.IsNull()) app->Close(document);
        } } catch (...) {} }
    } scope;
    try {
        scope.wrapper->InitDoc(); scope.document = scope.wrapper->Document();
        const auto& document = scope.document;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        core3d::enclosure::Parameters parameters;
        parameters.metersPerUnit = 0.001;
        parameters.definition.dimensions = {100,60,30,2,2,4};
        parameters.definition.plane = 0;
        if (mode == 1) {
            parameters.definition.constructionFrame.emplace();
            parameters.definition.constructionFrame->values = {13,-7,2,0,0,0.25881904510252074,0.9659258262890683,-1.25};
        }
        core3d::EnclosureSolidResult geometry;
        if (!core3d::BuildEnclosureSolidGeometry(parameters.definition,
                std::make_shared<std::atomic_bool>(false), geometry)) return nil;
        const auto original = geometry.solid;
        document->NewCommand();
        const auto label = shapes->AddShape(original, Standard_False);
        const auto bareLabel = shapes->AddShape(BRepPrimAPI_MakeBox(10.0,12.0,14.0).Shape(), Standard_False);
        if (bareLabel.IsNull()) return nil;
        if (!core3d::enclosure::Stage(document, label, parameters, NSUUID.UUID.UUIDString.UTF8String)
            || !document->CommitCommand()) return nil;
        core3d::enclosure::Record record;
        if (!core3d::enclosure::Read(document, label, record) || !record.IsCurrent(document, label)) return nil;
        Standard_Size admittedCopies = 1, shapeCost = 0;
        auto countLabels = [&]() {
            Standard_Size count = 0;
            for (TDF_ChildIterator it(document->GetData()->Root(), Standard_True); it.More(); it.Next()) ++count;
            return count;
        };
        const Standard_Size extraFrameScalars = mode == 0 ? 8U : 0U;
        const Standard_Size destinationLabels = 10U + record.values.size() + extraFrameScalars;
        if (destinationLabels != 27U || record.values.size() != (mode == 0 ? 9U : 17U)) return nil;
        const Standard_Size target = 100000U - destinationLabels;
        const auto filler = document->GetData()->Root().FindChild(900000, Standard_True);
        const Standard_Size before = countLabels();
        if (before >= target) return nil;
        for (Standard_Size i = 1; i <= target - before; ++i)
            filler.FindChild(static_cast<Standard_Integer>(i), Standard_True);
        if (countLabels() != target) return nil;
        const auto labelsBefore = countLabels();
        const auto timeBefore = document->GetData()->Time();
        const auto undoBefore = document->GetAvailableUndos();
        const auto rootBefore = XCAFDoc_ShapeTool::GetShape(label);
        const bool valid = scope.wrapper->ValidateGeometryRepresentations();
        const bool atBoundary = scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, admittedCopies, false, true, true}});
        const bool overflowRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, std::numeric_limits<Standard_Size>::max(), false, true, true}});
        const bool zeroRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, 0U, false, true, true}});
        const bool duplicateRequestRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, 1U, false, true, true}, {label, 1U, false, true, true}});
        const bool missingPreservationRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, 1U, false, false, true}});
        const bool conflictingFramePoliciesRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, 1U, true, true, true}});
        const bool absentEnclosureFrameRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{bareLabel, 1U, false, true, true}});
        core3d::enclosure::Record after;
        const bool queryUnchanged = labelsBefore == countLabels() && timeBefore == document->GetData()->Time()
            && undoBefore == document->GetAvailableUndos() && !document->HasOpenCommand()
            && rootBefore.IsEqual(XCAFDoc_ShapeTool::GetShape(label))
            && core3d::enclosure::Read(document, label, after) && after.IsEqual(record);
        filler.FindChild(200000, Standard_True);
        if (countLabels() != labelsBefore + 1U) return nil;
        const bool beyondRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, 1U, false, true, true}});
        return @{@"valid": @(valid), @"atBoundary": @(atBoundary), @"beyondRejected": @(beyondRejected),
            @"overflowRejected": @(overflowRejected), @"zeroRejected": @(zeroRejected),
            @"absentEnclosureFrameRejected": @(absentEnclosureFrameRejected),
            @"missingPreservationRejected": @(missingPreservationRejected),
            @"conflictingFramePoliciesRejected": @(conflictingFramePoliciesRejected),
            @"duplicateRequestRejected": @(duplicateRequestRejected), @"queryUnchanged": @(queryUnchanged),
            @"sourceScalars": @(record.values.size()), @"extraFrameScalars": @(extraFrameScalars),
            @"destinationLabels": @(destinationLabels), @"labelsBefore": @(labelsBefore), @"admittedCopies": @(admittedCopies), @"shapeCost": @(shapeCost)};
    } catch (...) { return nil; }
}

- (NSDictionary<NSString *, NSNumber *> *_Nullable)debugProfileMirrorCapacityProbe:(NSInteger)mode {
    if (![NSThread isMainThread] || mode < 0 || mode > 1) return nil;
    struct Scope {
        Handle(OcctDocument) wrapper = new OcctDocument();
        Handle(TDocStd_Document) document;
        ~Scope() noexcept { try { if (!document.IsNull()) {
            if (document->HasOpenCommand()) document->AbortCommand();
            const auto app = Handle(TDocStd_Application)::DownCast(document->Application());
            if (!app.IsNull()) app->Close(document);
        } } catch (...) {} }
    } scope;
    try {
        scope.wrapper->InitDoc(); scope.document = scope.wrapper->Document();
        const auto& document = scope.document;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        const auto original = BRepPrimAPI_MakeBox(60.0, 40.0, 30.0).Shape();
        document->NewCommand();
        const auto label = shapes->AddShape(original, Standard_False);
        core3d::profile::Parameters parameters;
        parameters.metersPerUnit = 0.001;
        parameters.definition.points = {gp_Pnt2d(0,0), gp_Pnt2d(60,0), gp_Pnt2d(60,40), gp_Pnt2d(0,40)};
        parameters.definition.plane = 0; parameters.definition.depth = 30;
        if (mode == 1) parameters.constructionFrame.emplace();
        if (!core3d::profile::Stage(document, label, parameters, NSUUID.UUID.UUIDString.UTF8String)
            || !document->CommitCommand()) return nil;
        core3d::profile::Record record;
        if (!core3d::profile::Read(document, label, record) || !record.IsCurrent(document, label)) return nil;
        Standard_Size admittedCopies = 1, shapeCost = 0;
        auto countLabels = [&]() {
            Standard_Size count = 0;
            for (TDF_ChildIterator it(document->GetData()->Root(), Standard_True); it.More(); it.Next()) ++count;
            return count;
        };
        const Standard_Size extraFrameScalars = mode == 0 ? 8U : 0U;
        const Standard_Size destinationLabels = 10U + record.values.size() + extraFrameScalars;
        if (destinationLabels != 33U || record.values.size() != (mode == 0 ? 15U : 23U)) return nil;
        const Standard_Size target = 100000U - destinationLabels;
        const auto filler = document->GetData()->Root().FindChild(900000, Standard_True);
        const Standard_Size before = countLabels();
        if (before >= target) return nil;
        for (Standard_Size i = 1; i <= target - before; ++i)
            filler.FindChild(static_cast<Standard_Integer>(i), Standard_True);
        if (countLabels() != target) return nil;
        const auto labelsBefore = countLabels();
        const auto timeBefore = document->GetData()->Time();
        const auto undoBefore = document->GetAvailableUndos();
        const auto rootBefore = XCAFDoc_ShapeTool::GetShape(label);
        const bool valid = scope.wrapper->ValidateGeometryRepresentations();
        const bool atBoundary = scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, admittedCopies, true}});
        const bool overflowRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, std::numeric_limits<Standard_Size>::max(), true}});
        const bool zeroRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, 0U, true}});
        const bool duplicateRequestRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, 1U, true}, {label, 1U, true}});
        core3d::profile::Record after;
        const bool queryUnchanged = labelsBefore == countLabels() && timeBefore == document->GetData()->Time()
            && undoBefore == document->GetAvailableUndos() && !document->HasOpenCommand()
            && rootBefore.IsEqual(XCAFDoc_ShapeTool::GetShape(label))
            && core3d::profile::Read(document, label, after) && after.IsEqual(record);
        filler.FindChild(200000, Standard_True);
        if (countLabels() != labelsBefore + 1U) return nil;
        const bool beyondRejected = !scope.wrapper->CanDuplicateGeometryDefinitions(
            std::vector<OcctGeometryDuplicationRequest>{{label, 1U, true}});
        return @{@"valid": @(valid), @"atBoundary": @(atBoundary), @"beyondRejected": @(beyondRejected),
            @"overflowRejected": @(overflowRejected), @"zeroRejected": @(zeroRejected),
            @"duplicateRequestRejected": @(duplicateRequestRejected), @"queryUnchanged": @(queryUnchanged),
            @"sourceScalars": @(record.values.size()), @"extraFrameScalars": @(extraFrameScalars),
            @"destinationLabels": @(destinationLabels), @"labelsBefore": @(labelsBefore), @"admittedCopies": @(admittedCopies), @"shapeCost": @(shapeCost)};
    } catch (...) { return nil; }
}

- (NSData *_Nullable)debugUnitStaleProfileBinXCAFFixtureData {
    if (![NSThread isMainThread]) return nil;
    return Core3DCreateDebugBinXCAFFixture(@"unit-stale-profile", [](const Handle(TDocStd_Document)& document) {
        const auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        const auto shape = BRepPrimAPI_MakeBox(60.0, 40.0, 30.0).Shape();
        const auto label = shapeTool->AddShape(shape, Standard_False, Standard_True);
        if (label.IsNull()) throw Standard_Failure("Unable to create unit-stale profile fixture");
        Core3DSetDebugGeometryRepresentation(label, 1);
        core3d::profile::Parameters parameters;
        parameters.metersPerUnit = 0.001;
        parameters.definition.points = {gp_Pnt2d(0, 0), gp_Pnt2d(60, 0), gp_Pnt2d(60, 40), gp_Pnt2d(0, 40)};
        parameters.definition.plane = 0;
        parameters.definition.depth = 30;
        document->SetUndoLimit(10);
        document->NewCommand();
        if (!core3d::profile::Stage(document, label, parameters, NSUUID.UUID.UUIDString.UTF8String)
            || !document->CommitCommand() || document->HasOpenCommand())
            throw Standard_Failure("Unable to stage fixture saved profile");
        core3d::profile::Record original;
        if (!core3d::profile::Read(document, label, original) || !original.IsCurrent(document, label))
            throw Standard_Failure("Fixture profile was not initially current");
        // Deliberately change only the private fixture's declared document unit.
        // The record remains bound to the exact same oriented root geometry.
        XCAFDoc_DocumentTool::SetLengthUnit(document, 1.0);
        core3d::profile::Record stale;
        if (!core3d::profile::Read(document, label, stale) || !stale.IsEqual(original)
            || !stale.boundShape.IsEqual(XCAFDoc_ShapeTool::GetShape(label))
            || stale.IsCurrent(document, label))
            throw Standard_Failure("Fixture did not preserve unit-only staleness");
    });
}

- (NSData *_Nullable)debugGeometryStaleProfileBinXCAFFixtureData {
    if (![NSThread isMainThread]) return nil;
    return Core3DCreateDebugBinXCAFFixture(@"geometry-stale-profile", [](const Handle(TDocStd_Document)& document) {
        const auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        const auto shape = BRepPrimAPI_MakeBox(60.0, 40.0, 30.0).Shape();
        const auto label = shapeTool->AddShape(shape, Standard_False, Standard_True);
        if (label.IsNull()) throw Standard_Failure("Unable to create geometry-stale profile fixture");
        Core3DSetDebugGeometryRepresentation(label, 1);
        core3d::profile::Parameters parameters;
        parameters.metersPerUnit = 0.001;
        parameters.definition.points = {gp_Pnt2d(0, 0), gp_Pnt2d(60, 0), gp_Pnt2d(60, 40), gp_Pnt2d(0, 40)};
        parameters.definition.plane = 0;
        parameters.definition.depth = 30;
        document->SetUndoLimit(10);
        document->NewCommand();
        if (!core3d::profile::Stage(document, label, parameters, NSUUID.UUID.UUIDString.UTF8String)
            || !document->CommitCommand() || document->HasOpenCommand())
            throw Standard_Failure("Unable to stage fixture saved profile");
        core3d::profile::Record original;
        if (!core3d::profile::Read(document, label, original) || !original.IsCurrent(document, label)
            || original.label.Tag() < core3d::profile::MinimumRecordTag)
            throw Standard_Failure("Fixture profile was not current or overlapped reserved object fields");
        // Each box has 34 unique shapes but 86 recursive occurrences:
        // 1 solid + 1 shell + 6 faces + 6 wires + 24 edges + 48 vertices.
        // Arrays bound occurrence traversal, including shared edges/vertices.
        // Keep the 60mm recipe bound to its original solid and a later 65mm root.
        const auto later = BRepPrimAPI_MakeBox(65.0, 40.0, 30.0).Shape();
        TopTools_IndexedMapOfShape oldTopology, newTopology;
        TopExp::MapShapes(shape, oldTopology); TopExp::MapShapes(later, newTopology);
        if (oldTopology.Extent() != 34 || newTopology.Extent() != 34 || shape.IsPartner(later))
            throw Standard_Failure("Unexpected capacity fixture topology");
        const auto verifyBoxOccurrences = [](const TopoDS_Shape& box) {
            std::array<std::size_t, 8> counts{};
            std::vector<TopoDS_Shape> pending{box};
            std::size_t total = 0;
            while (!pending.empty()) {
                const auto current = pending.back(); pending.pop_back();
                if (current.IsNull() || ++total > 86)
                    throw Standard_Failure("Unexpected fixture occurrence count");
                ++counts[static_cast<std::size_t>(current.ShapeType())];
                for (TopoDS_Iterator child(current, Standard_True, Standard_True); child.More(); child.Next()) {
                    if (pending.size() + total >= 86)
                        throw Standard_Failure("Unexpected fixture occurrence capacity");
                    pending.push_back(child.Value());
                }
            }
            const std::array<std::size_t, 8> expected{0, 0, 1, 1, 6, 6, 24, 48};
            if (total != 86 || counts != expected)
                throw Standard_Failure("Unexpected fixture occurrence structure");
        };
        verifyBoxOccurrences(shape); verifyBoxOccurrences(later);
        document->NewCommand(); shapeTool->SetShape(label, later);
        if (!document->CommitCommand() || document->HasOpenCommand())
            throw Standard_Failure("Unable to commit later capacity fixture solid");
        core3d::profile::Record stale;
        if (!core3d::profile::Read(document, label, stale) || !stale.IsEqual(original)
            || stale.boundShape.IsEqual(XCAFDoc_ShapeTool::GetShape(label))
            || stale.IsCurrent(document, label))
            throw Standard_Failure("Fixture did not preserve geometry-only staleness");
    });
}

- (NSData *_Nullable)debugLegacyNoLengthUnitMirrorBinXCAFFixtureData {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSURL* baseURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.legacy-no-unit-mirror-fixture", NSUUID.UUID.UUIDString]];
    NSString* xbfPath = [baseURL.path stringByAppendingString:@".xbf"];
    NSData* result = nil;
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(
            TCollection_ExtendedString("BinXCAF"), document);
        if (document.IsNull()) {
            throw Standard_Failure(
                "Unable to create legacy no-unit mirror fixture document");
        }
        Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapeTool.IsNull()) {
            throw Standard_Failure(
                "Unable to create legacy no-unit mirror fixture shape tool");
        }
        double unit = 0.001;
        if (XCAFDoc_DocumentTool::GetLengthUnit(document, unit))
            throw Standard_Failure("Legacy fixture must omit explicit unit metadata");
        const TDF_Label shapeLabel = shapeTool->AddShape(
            BRepPrimAPI_MakeBox(60.0, 40.0, 30.0).Shape(),
            Standard_False,
            Standard_True);
        if (shapeLabel.IsNull()) {
            throw Standard_Failure(
                "Unable to create legacy no-unit mirror fixture shape");
        }
        if (application->SaveAs(
                document, baseURL.path.UTF8String) != PCDM_SS_OK) {
            throw Standard_Failure(
                "Unable to save legacy no-unit mirror fixture");
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

// Fixture-only creation: one real OCAF command contains geometry, recipe and
// record. This does NOT exercise or enable production AI receipt integration.
- (NSDictionary *_Nullable)core3d_debugReceiptFixture:(NSInteger)kind legacyPolicy:(NSInteger)legacyPolicy
    metersPerUnit:(double)unit {
    if (![NSThread isMainThread] || kind < 0 || kind > 4 || legacyPolicy < -1 || legacyPolicy > 2
        || (unit!=0.001&&unit!=1) || (kind<3&&unit!=0.001) || (kind>=3&&legacyPolicy!=-1)) return nil;
    namespace r = core3d::receipt;
    Handle(OcctDocument) owner;
    NSURL *base = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSUUID.UUID.UUIDString stringByAppendingString:@".receipt-fixture"]];
    // OCCT 7.8 TDocStd_PathParser needs a dotted basename to populate Trek.
    // Preserve the real BinXCAF save path used by existing UUID.suffix fixtures.
    NSString *saved = nil;
    NSDictionary *result = nil;
    try {
        owner = new OcctDocument(); owner->InitDoc();
        const auto document = owner->Document();
        if (document.IsNull()) throw Standard_Failure("Receipt fixture document");
        if(kind>=3)XCAFDoc_DocumentTool::SetLengthUnit(document,unit);
        // Existing ordinary shape and saved group predate this instruction.
        // Clearing only fixture setup history leaves one real receipt Undo.
        document->NewCommand();
        Handle(AIS_Shape) guardAIS = new AIS_Shape(BRepPrimAPI_MakeBox(gp_Pnt(200,0,0),1,2,3).Shape());
        const auto guardLabel = owner->AddShape(guardAIS,OcctGeometryRepresentation::BRep);
        OcctSavedGroup guardGroup; guardGroup.identifier=OcctDocument::NewSavedGroupIdentifier();
        guardGroup.name=TCollection_ExtendedString("Receipt guard group"); guardGroup.members={guardLabel};
        if (guardLabel.IsNull() || !owner->SetObjectNameForLabel(guardLabel,TCollection_ExtendedString("Receipt guard"))
            || !owner->StageSavedGroups({guardGroup}) || !document->CommitCommand())
            throw Standard_Failure("Receipt guard fixture");
        if(kind>=3){
            // A genuine preexisting legacy component shares this private document.
            // Its guard profile matches the original placed1x2x3 box exactly.
            core3d::profile::Parameters guardProfile;guardProfile.metersPerUnit=unit;
            guardProfile.definition.points={{0,0},{1,0},{1,2},{0,2}};guardProfile.definition.depth=3;
            core3d::profile::ConstructionFrame f;f.values={200,0,0,0,0,0,1,1};guardProfile.constructionFrame=f;
            document->NewCommand();
            if(!core3d::profile::Stage(document,guardLabel,guardProfile,NSUUID.UUID.UUIDString.UTF8String))
                throw Standard_Failure("Loft legacy guard profile");
            r::Effect old;r::DebugEffectCapture oldCapture;
            if(!r::legacy_debug::Capture(owner,guardLabel,2,old,oldCapture))throw Standard_Failure("Loft legacy guard effect");
            r::Record legacy;legacy.policy=0;legacy.operation=r::Operation::CreateAssembly;legacy.effects={old};
            legacy.key.accountScope.fill(8);legacy.key.command.fill(9);legacy.key.execution.fill(10);
            if(!r::ParseUUID(owner->DocumentIdentifier(),legacy.key.document)
                ||!r::ParseUUID(NSUUID.UUID.UUIDString.UTF8String,legacy.key.request))throw Standard_Failure("Loft legacy guard key");
            r::Catalog empty;if(r::Read(document,empty)!=r::ReadStatus::Absent
                ||!r::legacy_debug::StageLegacy(owner,legacy,empty)||!document->CommitCommand())
                throw Standard_Failure("Loft legacy guard catalog");
        }
        document->ClearUndos();
        OcctObjectNameState guardBefore; OcctSavedGroupState groupsBefore;
        NSDictionary *preservation=Core3DReceiptPreservation(owner);
        if (!preservation || !owner->CaptureObjectNameStateForLabel(guardLabel,guardBefore)
            || !owner->CaptureSavedGroups(groupsBefore)) throw Standard_Failure("Receipt guard capture");
        auto preserved = [&]() {
            OcctObjectNameState guardAfter; OcctSavedGroupState groupsAfter;
            return owner->CaptureObjectNameStateForLabel(guardLabel,guardAfter)
                && guardBefore.IsEqual(guardAfter) && owner->CaptureSavedGroups(groupsAfter)
                && groupsBefore.IsEqual(groupsAfter)
                && [preservation isEqual:Core3DReceiptPreservation(owner)];
        };
        TopoDS_Shape shape;
        core3d::rectangular_loft::Definition loft;
        core3d::profile::Parameters profile; profile.metersPerUnit = 0.001;
        profile.definition.depth = 30;
        core3d::enclosure::Parameters enclosure; enclosure.metersPerUnit = 0.001;
        if (kind == 0) {
            profile.definition.points = {{0,0},{100,0},{100,60},{0,60}};
            shape = BRepPrimAPI_MakeBox(100,60,30).Shape();
        } else if (kind == 1) {
            profile.definition.circle = core3d::ProfileCircularSection{gp_Pnt2d(0,0),20,0};
            shape = BRepPrimAPI_MakeCylinder(20,30).Shape();
        } else if(kind>=3) {
            loft=core3d::rectangular_loft::probe::Fixture(int(kind-3),unit);
            core3d::rectangular_loft::Admission admission;
            const auto prepared=core3d::rectangular_loft::Prepare(loft,admission);
            std::atomic_bool cancelled{false};core3d::rectangular_loft::SolidResult built;
            if(core3d::rectangular_loft::Build(prepared,cancelled,built)!=core3d::rectangular_loft::BuildStatus::Built)
                throw Standard_Failure("Receipt loft fixture");
            shape=built.solid;
        } else {
            core3d::EnclosureSolidResult built;
            if (!core3d::BuildEnclosureSolidGeometry(enclosure.definition,
                std::make_shared<std::atomic_bool>(false), built))
                throw Standard_Failure("Receipt enclosure fixture");
            shape = built.solid;
        }
        const std::string feature = NSUUID.UUID.UUIDString.UTF8String;
        r::Record record; record.policy=kind>=3?r::ExactLoftPolicy4097:(legacyPolicy<0?r::AnalyticZeroPolicy1:0);
        // A synthetic component record, not proof of an executed AI operation.
        record.operation = kind>=3?r::Operation::RebuildLoftStation:(kind == 2 ? r::Operation::CreateEnclosure : r::Operation::CreateAssembly);
        record.key.accountScope.fill(1); record.key.command.fill(2); record.key.execution.fill(3);
        NSString *request = NSUUID.UUID.UUIDString;
        if (!r::ParseUUID(owner->DocumentIdentifier(), record.key.document)
            || !r::ParseUUID(request.UTF8String, record.key.request))
            throw Standard_Failure("Receipt fixture key");
        r::Catalog before;
        if (r::Read(document,before) != (kind>=3?r::ReadStatus::Valid:r::ReadStatus::Absent))
            throw Standard_Failure("Receipt fixture prior catalog");
        document->NewCommand();
        Handle(AIS_Shape) ais = new AIS_Shape(shape);
        const auto label = owner->AddShape(ais,OcctGeometryRepresentation::BRep);
        if (label.IsNull() || !(kind>=3?core3d::loft_persistence::Stage(document,label,loft,feature):
            (kind == 2 ? core3d::enclosure::Stage(document,label,enclosure,feature)
            : core3d::profile::Stage(document,label,profile,feature))))
            throw Standard_Failure("Receipt fixture recipe");
        r::Effect effect;
        r::DebugEffectCapture issuedCapture;
        if (!(legacyPolicy<0?r::CaptureEffect(owner,label,effect,&issuedCapture):r::legacy_debug::Capture(owner,label,int(legacyPolicy),effect,issuedCapture))) throw Standard_Failure("Receipt fixture effect");
        record.effects = {effect};
        if (!(legacyPolicy<0?r::Stage(owner,record,before):r::legacy_debug::StageLegacy(owner,record,before)) || !document->CommitCommand())
            throw Standard_Failure("Receipt fixture commit");
        NSDictionary *initialEffectDiagnostic=Core3DReceiptEffectDiagnostic(owner,effect,&issuedCapture);
        const bool stagePreserved = preserved();
        const bool oneUndo = document->GetAvailableUndos() == 1;
        const auto first = r::InspectDocument(owner,record.key);
        const bool initiallyCurrent = first.presence == r::DocumentPresence::Present && first.effectsCurrent;
        const bool undone = document->Undo();
        r::Catalog absent; const bool undoAbsent = r::Read(document,absent) == (kind>=3?r::ReadStatus::Valid:r::ReadStatus::Absent)
            && absent.matches(before);
        TDF_LabelSequence roots; XCAFDoc_DocumentTool::ShapeTool(document->Main())->GetFreeShapes(roots);
        const bool undoGuardOnly = roots.Length()==1 && roots.Value(1)==guardLabel;
        const bool undoPreserved = preserved();
        const bool redone = document->Redo();
        const auto restored = r::InspectDocument(owner,record.key);
        const bool redoCurrent = restored.presence == r::DocumentPresence::Present && restored.effectsCurrent;
        const bool redoPreserved = preserved();
        r::Catalog catalog;
        if (r::Read(document,catalog) != r::ReadStatus::Valid) throw Standard_Failure("Receipt fixture readback");
        // Unknown policies stay readable but unresolved/nonappendable, including
        // preexisting profile/enclosure4097 records. This is a private fixture
        // command, never admission or retrospective policy reinterpretation.
        bool unknownPolicyPreserved=true;
        if(legacyPolicy<0){
            auto unknown=record;unknown.policy=kind>=3?r::ExactLoftPolicy4097+1:r::ExactLoftPolicy4097;
            for(auto&e:unknown.effects)e.policy=unknown.policy;
            std::vector<std::uint8_t> raw;
            if(!r::Encode({unknown},raw))throw Standard_Failure("Loft unknown policy encode");
            document->NewCommand();
            if(!r::legacy_debug::Write(document,raw,false,catalog.label)||!document->CommitCommand())
                throw Standard_Failure("Loft unknown policy fixture");
            r::Catalog loaded;const auto inspected=r::InspectDocument(owner,record.key);
            unknownPolicyPreserved=r::Read(document,loaded)==r::ReadStatus::Valid&&!loaded.supportsAppend()
                &&loaded.bytes==raw&&inspected.presence==r::DocumentPresence::Present
                &&inspected.evidence==r::EffectEvidenceStatus::UnsupportedPolicy&&!inspected.effectsCurrent;
            document->NewCommand();
            auto forbidden=record;forbidden.key.request[0]^=0x20;
            unknownPolicyPreserved=unknownPolicyPreserved&&!r::Stage(owner,forbidden,loaded);
            document->AbortCommand();
            unknownPolicyPreserved=unknownPolicyPreserved&&document->Undo();
            r::Catalog restoredUnknown;
            unknownPolicyPreserved=unknownPolicyPreserved&&r::Read(document,restoredUnknown)==r::ReadStatus::Valid
                &&restoredUnknown.matches(catalog)&&preserved();
        }
        // Codec rejection must not leave partially decoded evidence.
        auto damaged = (legacyPolicy<0?catalog.bytes:catalog.legacyBytes); damaged.back() ^= 1;
        std::vector<r::Record> decoded;
        const bool checksumRejected = r::Decode(damaged,decoded) == r::ReadStatus::Malformed && decoded.empty();
        damaged = (legacyPolicy<0?catalog.bytes:catalog.legacyBytes); damaged.pop_back();
        const bool truncatedRejected = r::Decode(damaged,decoded) == r::ReadStatus::Malformed && decoded.empty();
        auto future = (legacyPolicy<0?catalog.bytes:catalog.legacyBytes); future[4] = 3;
        const bool futureUnavailable = r::Decode(future,decoded) == r::ReadStatus::Unsupported && decoded.empty();
        std::vector<std::uint8_t> encoded;
        const bool duplicateRejected = !(legacyPolicy<0?r::Encode({record,record},encoded):r::legacy_debug::EncodeLegacy({record,record},encoded)) && encoded.empty();
        // Failed staging is caller-aborted in the same command; existing bytes survive.
        auto next = record; next.key.request[0] ^= 0x80;
        document->NewCommand();
        if (!(legacyPolicy<0?r::Stage(owner,next,catalog):r::legacy_debug::StageLegacy(owner,next,catalog))) throw Standard_Failure("Receipt abort fixture stage");
        document->AbortCommand();
        r::Catalog afterAbort;
        const bool abortRestored = r::Read(document,afterAbort) == r::ReadStatus::Valid && afterAbort.matches(catalog);
        const bool abortPreserved = preserved();
        // Reserved catalog identifiers fail closed in every forbidden location.
        bool placementRejected = true;
        for (int placement=0;placement<4;++placement) {
            document->NewCommand();
            const auto root=document->GetData()->Root();
            TDF_Label invalid;
            if (placement==0) invalid=root;
            else if (placement==1) invalid=document->Main();
            else if (placement==2) invalid=(legacyPolicy<0?catalog.label:catalog.legacyLabel).FindChild(5000,Standard_True);
            else invalid=root.FindChild((legacyPolicy<0?catalog.label:catalog.legacyLabel).Tag()+1,Standard_True);
            TDataStd_Integer::Set(invalid,legacyPolicy<0?r::VersionedSchemaID():r::SchemaID(),legacyPolicy<0?2:1);
            r::Catalog rejected;
            placementRejected = placementRejected && r::Read(document,rejected)==r::ReadStatus::Malformed
                && rejected.label.IsNull() && rejected.bytes.empty();
            document->AbortCommand();
            r::Catalog restoredCatalog;
            placementRejected = placementRejected && r::Read(document,restoredCatalog)==r::ReadStatus::Valid
                && restoredCatalog.matches(catalog) && preserved();
        }
        const std::string path = owner->save(base.path.UTF8String);
        if (path.empty()) throw Standard_Failure("Receipt fixture save");
        const auto postSave=r::InspectDocument(owner,record.key);
        NSDictionary *postSaveEffectDiagnostic=Core3DReceiptEffectDiagnostic(owner,effect);
        saved = [NSString stringWithUTF8String:path.c_str()];
        NSData *data = [NSData dataWithContentsOfFile:saved];
        if (!data) throw Standard_Failure("Receipt fixture bytes");
        result = @{@"data":data, @"requestID":request,
            @"entityID":[NSString stringWithUTF8String:owner->EntityIdentifierForLabel(label).c_str()],
            @"effectDiagnostic":@{@"initial":initialEffectDiagnostic,@"postSave":postSaveEffectDiagnostic,
                @"postSaveCurrent":@(postSave.effectsCurrent)},
            @"geometrySHA":[NSString stringWithUTF8String:r::Hex(effect.geometry).c_str()],
            @"stateSHA":[NSString stringWithUTF8String:r::Hex(effect.state).c_str()],
            @"catalogBytes":[NSData dataWithBytes:(legacyPolicy<0?catalog.bytes:catalog.legacyBytes).data() length:(legacyPolicy<0?catalog.bytes:catalog.legacyBytes).size()],
            @"stateBytes":[NSData dataWithBytes:issuedCapture.stateBytes.data() length:issuedCapture.stateBytes.size()],
            @"legacyCatalogBytes":[NSData dataWithBytes:catalog.legacyBytes.data() length:catalog.legacyBytes.size()],
            @"legacyPolicy":@(legacyPolicy),@"initiallyUnresolved":@(first.presence==r::DocumentPresence::Present&&first.evidence==r::EffectEvidenceStatus::LegacyUnversioned&&!first.effectsCurrent),
            @"redoUnresolved":@(restored.presence==r::DocumentPresence::Present&&restored.evidence==r::EffectEvidenceStatus::LegacyUnversioned&&!restored.effectsCurrent),
            @"unknownPolicyPreserved":@(unknownPolicyPreserved),
            @"oneUndo":@(oneUndo), @"initiallyCurrent":@(initiallyCurrent),
            @"undoAbsent":@(undone && undoAbsent && undoGuardOnly), @"redoCurrent":@(redone && redoCurrent),
            @"abortRestored":@(abortRestored), @"checksumRejected":@(checksumRejected),
            @"truncatedRejected":@(truncatedRejected), @"futureUnavailable":@(futureUnavailable),
            @"duplicateRejected":@(duplicateRejected), @"preservation":preservation,
            @"stagePreserved":@(stagePreserved), @"undoPreserved":@(undoPreserved),
            @"redoPreserved":@(redoPreserved), @"abortPreserved":@(abortPreserved),
            @"placementRejected":@(placementRejected)};
    } catch (const Standard_Failure& failure) {
        // Keep substantive fixture failure visible; nil still fails XCTest.
        NSString *message = failure.GetMessageString()
            ? [NSString stringWithUTF8String:failure.GetMessageString()] : nil;
        message = message ?: @"Native exception without UTF-8 detail";
        NSLog(@"Native receipt fixture failed (kind %ld): %@", (long)kind,
            [message substringToIndex:MIN(message.length, (NSUInteger)512)]);
        result = nil;
    } catch (...) {
        NSLog(@"Native receipt fixture failed (kind %ld): unknown exception", (long)kind);
        result = nil;
    }
    try {
        if (!owner.IsNull() && !owner->Document().IsNull()) {
            const auto document = owner->Document();
            if (document->HasOpenCommand()) document->AbortCommand();
            const auto application = Handle(TDocStd_Application)::DownCast(document->Application());
            if (!application.IsNull()) application->Close(document);
        }
    } catch (...) {}
    [NSFileManager.defaultManager removeItemAtURL:base error:nil];
    if (saved) [NSFileManager.defaultManager removeItemAtPath:saved error:nil];
    [NSFileManager.defaultManager removeItemAtPath:[base.path stringByAppendingString:@".xbf"] error:nil];
    return result;
}

- (NSDictionary *_Nullable)debugNativeReceiptFixture:(NSInteger)kind {
    if(kind<0||kind>2)return nil;
    return [self core3d_debugReceiptFixture:kind legacyPolicy:-1 metersPerUnit:0.001];
}
- (NSDictionary *_Nullable)debugNativeLegacyReceiptFixture:(NSInteger)kind policy:(NSInteger)policy {
    if(kind<0||kind>2||policy<0||policy>2)return nil;
    return [self core3d_debugReceiptFixture:kind legacyPolicy:policy metersPerUnit:0.001];
}

- (NSDictionary *_Nullable)debugLoftReceiptFixture:(NSInteger)variant metersPerUnit:(double)unit {
    if(variant<0||variant>1)return nil;
    return [self core3d_debugReceiptFixture:variant+3 legacyPolicy:-1 metersPerUnit:unit];
}
+ (NSDictionary *)debugLoftRequestCodec {
    if(!NSThread.isMainThread)return @{};
    namespace n=core3d::request;namespace r=core3d::receipt;
    try {
        NSMutableArray *legacy=[NSMutableArray array];
        for(unsigned operation=1;operation<=5;++operation){n::Descriptor d;d.operation=n::Operation(operation);
            n::Part p;p.recipe=(operation==1||operation==2)?n::Recipe::Enclosure:n::Recipe::Profile;p.schema=1;p.values={1.25,-0.0};d.parts={p};
            std::vector<std::uint8_t> bytes;n::Descriptor decoded;
            if(!n::Encode(d,bytes)||!n::Decode(bytes,decoded))return @{};
            [legacy addObject:[NSData dataWithBytes:bytes.data() length:bytes.size()]];
        }
        auto source=core3d::rectangular_loft::probe::Fixture(0,0.001);
        core3d::rectangular_loft::StationDimensionEdit edit;edit.stationIdentifier=101;edit.width=36;
        n::Descriptor d;if(!r::LoftStationDescriptor(source,edit,d))return @{};
        std::vector<std::uint8_t> bytes;n::Digest digest;n::Descriptor decoded;
        if(!n::Encode(d,bytes)||!n::CommandHash(d,digest)||!n::Decode(bytes,decoded))return @{};
        std::vector<std::uint8_t> roundtrip;if(!n::Encode(decoded,roundtrip))return @{};
        NSMutableArray *variants=[NSMutableArray array];
        for(int mode=0;mode<6;++mode){auto changed=source;auto wanted=edit;
            if(mode==0)wanted.depth=20; // presence matters even when value is unchanged
            if(mode==1)wanted.width=38;
            if(mode==2)wanted.stationIdentifier=100;
            if(mode==3)changed.stations.back().centerY=-0.0;
            if(mode==4){auto f=core3d::rectangular_loft::probe::Fixture(0,0.001,3);changed=f;}
            if(mode==5)changed.loftIdentifier=999;
            n::Descriptor variant;n::Digest hash;
            if(!r::LoftStationDescriptor(changed,wanted,variant)||!n::CommandHash(variant,hash))return @{};
            [variants addObject:@(r::Hex(hash).c_str())];
        }
        bool malformed=true;
        for(int mode=0;mode<6;++mode){auto bad=bytes;
            if(mode==0)bad[4]=1;
            if(mode==1)bad.push_back(0);
            if(mode==2)bad.pop_back();
            if(mode==3)bad[5]=5;
            if(mode==4)bad[bad.size()-9]=0; // LSED mask (one width scalar)
            if(mode==5)bad[bad.size()-9]=4;
            n::Descriptor refused;malformed=malformed&&!n::Decode(bad,refused)&&refused.parts.empty();
        }
        auto invalid=edit;invalid.stationIdentifier=999;n::Descriptor refused;
        const bool missing=!r::LoftStationDescriptor(source,invalid,refused)&&refused.parts.empty();
        auto mixed=d;mixed.operation=n::Operation::CreateProfile;std::vector<std::uint8_t> discarded;
        const bool legacyCannotCarryLoft=!n::Encode(mixed,discarded)&&discarded.empty();
        // The raw loft stream must preserve zero spellings under arbitrary chunks.
        const std::string raw="CASCADE Topology V3, (c) Open Cascade\n\nLocations 0\n\nCurve2ds 1\n1 -0 0 1 -0 \nCurves 0\nSurfaces 1\n1 0 0 0 0 0 1 1 -0 0 -0 1 -0 \n";
        bool rawExact=true;
        for(std::size_t chunk:{1u,7u,512u}){r::GeometryStream stream(false);std::vector<std::uint8_t> actual;stream.debugBytes=&actual;
            std::ostream out(&stream);for(std::size_t offset=0;offset<raw.size();offset+=chunk)out.write(raw.data()+offset,std::min(chunk,raw.size()-offset));
            r::Digest hash;rawExact=rawExact&&out.good()&&stream.finish(hash)&&actual==std::vector<std::uint8_t>(raw.begin(),raw.end());
        }
        return @{@"legacy":legacy,@"descriptor":[NSData dataWithBytes:bytes.data() length:bytes.size()],
            @"roundtrip":[NSData dataWithBytes:roundtrip.data() length:roundtrip.size()],@"commandSHA":@(r::Hex(digest).c_str()),
            @"variants":variants,@"malformedRejected":@(malformed),@"missingStationRejected":@(missing),
            @"legacyCannotCarryLoft":@(legacyCannotCarryLoft),@"rawStreamExact":@(rawExact)};
    }catch(...){return @{};}
}

// Read-only DEBUG visibility into unverified component evidence. Public verified
// reconciliation remains unavailable, including for current matching effects.
// Bounded read-only visibility. The returned catalogs confer no authority.
+ (NSDictionary<NSString *, NSNumber *> *)debugReceiptFramingProbe:(NSInteger)scenario {
    if (![NSThread isMainThread] || scenario < 0 || scenario > 3) return @{ @"invalidScenario": @NO };
    try {
        const auto checks = Core3DDebugReceiptFramingProbe(static_cast<Standard_Integer>(scenario));
        NSMutableDictionary<NSString *, NSNumber *> *result = [NSMutableDictionary dictionary];
        for (const auto& check : checks) {
            NSString *key = [NSString stringWithUTF8String:check.first.c_str()];
            if (key == nil) return @{ @"invalidKey": @NO };
            result[key] = @(check.second);
        }
        return result;
    } catch (...) { return @{ @"setupException": @NO }; }
}

+ (NSDictionary<NSString *, NSNumber *> *)debugRetainedSolidProbe:(NSInteger)scenario {
    if (![NSThread isMainThread] || scenario < 0 || scenario > 3) return @{ @"invalidScenario": @NO };
    try {
        const auto checks = Core3DDebugRetainedSolidProbe(static_cast<Standard_Integer>(scenario));
        NSMutableDictionary<NSString *, NSNumber *> *result = [NSMutableDictionary dictionary];
        for (const auto& check : checks) {
            NSString *key = [NSString stringWithUTF8String:check.first.c_str()];
            if (key == nil) return @{ @"invalidKey": @NO };
            result[key] = @(check.second);
        }
        return result;
    } catch (...) { return @{ @"setupException": @NO }; }
}

+ (NSDictionary<NSString *, NSNumber *> *)debugSavedCutSourcePrerequisiteProbe:(NSInteger)scenario {
    if (![NSThread isMainThread] || scenario < 0 || scenario > 3) return @{ @"invalidScenario": @NO };
    try {
        const auto checks = Core3DDebugSavedCutSourcePrerequisiteProbe(static_cast<Standard_Integer>(scenario));
        NSMutableDictionary<NSString *, NSNumber *> *result = [NSMutableDictionary dictionary];
        for (const auto& check : checks) {
            NSString *key = [NSString stringWithUTF8String:check.first.c_str()];
            if (key == nil) return @{ @"invalidKey": @NO };
            result[key] = @(check.second);
        }
        return result;
    } catch (...) { return @{ @"setupException": @NO }; }
}

+ (NSDictionary<NSString *, NSNumber *> *)debugEnclosureCorrespondenceProbe:(NSInteger)scenario {
    if (![NSThread isMainThread] || scenario < 0 || scenario > 2) return @{ @"invalidScenario": @NO };
    try {
        const auto checks = Core3DDebugEnclosureCorrespondenceProbe(static_cast<Standard_Integer>(scenario));
        NSMutableDictionary<NSString *, NSNumber *> *result = [NSMutableDictionary dictionary];
        for (const auto& check : checks) {
            NSString *key = [NSString stringWithUTF8String:check.first.c_str()];
            if (key == nil) return @{ @"invalidKey": @NO };
            result[key] = @(check.second);
        }
        return result;
    } catch (...) { return @{ @"setupException": @NO }; }
}

+ (NSDictionary<NSString *, NSNumber *> *)debugSavedCutBoreClearanceProbe:(NSInteger)scenario {
    if (![NSThread isMainThread] || scenario < 0 || scenario > 2) return @{ @"invalidScenario": @NO };
    try {
        const auto checks = Core3DDebugSavedCutBoreClearanceProbe(static_cast<Standard_Integer>(scenario));
        NSMutableDictionary<NSString *, NSNumber *> *result = [NSMutableDictionary dictionary];
        for (const auto& check : checks) {
            NSString *key = [NSString stringWithUTF8String:check.first.c_str()];
            if (key == nil) return @{ @"invalidKey": @NO };
            result[key] = @(check.second);
        }
        return result;
    } catch (...) { return @{ @"setupException": @NO }; }
}

+ (NSDictionary<NSString *, NSNumber *> *)debugSavedCutResultCorrespondenceProbe:(NSInteger)scenario {
    if (![NSThread isMainThread] || scenario < 0 || scenario > 1) return @{ @"invalidScenario": @NO };
    try {
        const auto checks = Core3DDebugSavedCutResultCorrespondenceProbe(static_cast<Standard_Integer>(scenario));
        NSMutableDictionary<NSString *, NSNumber *> *result = [NSMutableDictionary dictionary];
        for (const auto& check : checks) {
            NSString *key = [NSString stringWithUTF8String:check.first.c_str()];
            if (key == nil) return @{ @"invalidKey": @NO };
            result[key] = @(check.second);
        }
        return result;
    } catch (...) { return @{ @"setupException": @NO }; }
}

+ (NSDictionary<NSString *, NSNumber *> *)debugSavedBooleanProgramProbe {
    if (![NSThread isMainThread]) return @{ @"invalidThread": @NO };
    try {
        const auto checks = Core3DDebugSavedBooleanProgramProbe();
        NSMutableDictionary<NSString *, NSNumber *> *result = [NSMutableDictionary dictionary];
        for (const auto& check : checks) {
            NSString *key = [NSString stringWithUTF8String:check.first.c_str()];
            if (key == nil) return @{ @"invalidKey": @NO };
            result[key] = @(check.second);
        }
        return result;
    } catch (...) { return @{ @"setupException": @NO }; }
}

+ (NSDictionary<NSString *, NSNumber *> *)debugSavedCutTrimDomainProbe {
    if (![NSThread isMainThread]) return @{ @"invalidThread": @NO };
    try {
        const auto checks = Core3DDebugSavedCutTrimDomainProbe();
        NSMutableDictionary<NSString *, NSNumber *> *result = [NSMutableDictionary dictionary];
        for (const auto& check : checks) {
            NSString *key = [NSString stringWithUTF8String:check.first.c_str()];
            if (key == nil) return @{ @"invalidKey": @NO };
            result[key] = @(check.second);
        }
        return result;
    } catch (...) { return @{ @"setupException": @NO }; }
}

- (NSDictionary *)debugScalableReceiptProbe:(NSInteger)scenario {
    if(!NSThread.isMainThread||scenario<0||scenario>4||!GLController||!GLController.viewer)return @{};
    const auto owner=GLController.viewer->getDocument();
    try{
        const auto rows=scenario==4?core3d::receipt::v3::DebugProbe::Migration(owner)
            :core3d::receipt::v3::DebugProbe::Run(int(scenario));
        NSMutableDictionary *result=[NSMutableDictionary dictionary];
        for(const auto& row:rows)result[[NSString stringWithUTF8String:row.first.c_str()]]=@(row.second);
        return result;
    }catch(...){if(!owner.IsNull()&&!owner->Document().IsNull()&&owner->Document()->HasOpenCommand())owner->Document()->AbortCommand();return @{};}
}
- (NSDictionary *)debugScalableReceiptSnapshot {
    namespace r=core3d::receipt;
    if(!NSThread.isMainThread||!GLController||!GLController.viewer)return @{@"valid":@NO};
    try {
        const auto owner=GLController.viewer->getDocument();r::Catalog catalog;
        if(owner.IsNull()||owner->Document().IsNull()||owner->Document()->HasOpenCommand())return @{@"valid":@NO};
        const auto status=r::Read(owner->Document(),catalog);
        if(status!=r::ReadStatus::Absent&&status!=r::ReadStatus::Valid)return @{@"valid":@NO,@"read":@(int(status))};
        std::ostringstream raw(std::ios::out|std::ios::binary);
        if(catalog.tree&&!r::v3::BinaryDriver::WriteNumericTree(*catalog.tree,raw))return @{@"valid":@NO};
        const auto bytes=raw.str();
        return @{@"valid":@YES,@"read":@(int(status)),@"count":@(catalog.count()),@"v3":@(bool(catalog.tree)),
            @"wireBytes":@(catalog.tree?catalog.tree->wireBytes():0),
            @"graphBytes":@(catalog.tree?catalog.tree->budget()->bytes():0),
            @"processBytes":@(r::v3::AllocationBudget::processBytes()),
            @"rootWire":[NSData dataWithBytes:bytes.data() length:bytes.size()],
            @"legacyBytes":[NSData dataWithBytes:catalog.legacyBytes.data() length:catalog.legacyBytes.size()],
            @"versionedBytes":[NSData dataWithBytes:catalog.bytes.data() length:catalog.bytes.size()],
            @"supportsAppend":@(catalog.supportsAppend()),@"verifiedQueryAvailable":@NO};
    }catch(...){return @{@"valid":@NO};}
}
- (NSDictionary *)debugReceiptCatalogSnapshot {
    namespace r=core3d::receipt;
    if(!NSThread.isMainThread||!GLController||!GLController.viewer)return @{@"valid":@NO};
    const auto owner=GLController.viewer->getDocument();r::Catalog catalog;
    if(owner.IsNull()||owner->Document().IsNull()||owner->Document()->HasOpenCommand())return @{@"valid":@NO};
    const auto status=r::Read(owner->Document(),catalog);
    NSMutableArray *policies=[NSMutableArray array];for(const auto& record:catalog.records)[policies addObject:@(record.policy)];
    return @{@"valid":@(status==r::ReadStatus::Valid||status==r::ReadStatus::Absent),@"read":@(int(status)),
        @"legacyBytes":[NSData dataWithBytes:catalog.legacyBytes.data() length:catalog.legacyBytes.size()],
        @"versionedBytes":[NSData dataWithBytes:catalog.bytes.data() length:catalog.bytes.size()],
        @"policies":policies,@"appendSupported":@(catalog.supportsAppend()),
        @"verified":@(r::QueryVerifiedReceipt()!=r::VerifiedQueryStatus::Unavailable)};
}
- (NSDictionary *)debugReceiptWire:(NSData *)data {
    namespace r=core3d::receipt;
    if(!NSThread.isMainThread||![data isKindOfClass:NSData.class]||data.length==0||data.length>r::MaximumBytes)return @{@"valid":@NO};
    try {
        const auto p=static_cast<const std::uint8_t*>(data.bytes);std::vector<std::uint8_t> bytes(p,p+data.length),encoded;
        std::vector<r::Record> records;const auto status=r::Decode(bytes,records);
        if(status!=r::ReadStatus::Valid)return @{@"valid":@NO,@"status":@(int(status)),@"count":@(records.size())};
        const bool canEncode=r::Encode(records,encoded);NSMutableArray *policies=[NSMutableArray array];
        for(const auto& record:records)[policies addObject:@(record.policy)];
        return @{@"valid":@YES,@"count":@(records.size()),@"policies":policies,@"canEncode":@(canEncode),
            @"original":data,@"encoded":[NSData dataWithBytes:encoded.data() length:encoded.size()]};
    }catch(...){return @{@"valid":@NO};}
}
// Synthetic keys are confined to this disposable native document. No Store or
// production prepare/execute entry is called by this structural component probe.
- (NSDictionary *_Nullable)debugReceiptDualCatalogProbe {
    if(!NSThread.isMainThread)return nil;namespace r=core3d::receipt;
    Handle(OcctDocument) owner;NSMutableDictionary *result=[NSMutableDictionary dictionary];
    try {
        owner=new OcctDocument();owner->InitDoc();const auto doc=owner->Document();
        r::Record old;old.policy=0;old.operation=r::Operation::CreateAssembly;
        old.key.accountScope.fill(1);old.key.command.fill(2);old.key.execution.fill(3);old.key.request.fill(4);
        if(!r::ParseUUID(owner->DocumentIdentifier(),old.key.document))throw Standard_Failure("Probe document");
        r::Effect e;e.policy=0;e.entity.fill(5);e.definition.fill(6);e.featureID.fill(7);e.geometry.fill(8);e.state.fill(9);old.effects={e};
        std::vector<std::uint8_t> bytes;if(!r::legacy_debug::EncodeLegacy({old},bytes))throw Standard_Failure("Probe legacy");
        doc->NewCommand();if(!r::legacy_debug::Write(doc,bytes,true)||!doc->CommitCommand())throw Standard_Failure("Probe install");
        r::Catalog legacy;if(r::Read(doc,legacy)!=r::ReadStatus::Valid)throw Standard_Failure("Probe read");
        const auto initial=r::InspectDocument(owner,old.key);
        result[@"legacyPresentUnresolved"]=@(initial.presence==r::DocumentPresence::Present&&initial.evidence==r::EffectEvidenceStatus::LegacyUnversioned&&!initial.effectsCurrent);
        bool keysClosed=true;
        for(int field=0;field<4;++field){auto key=old.key;if(field==0)key.accountScope[0]^=1;if(field==1)key.document[0]^=1;if(field==2)key.command[0]^=1;if(field==3)key.execution[0]^=1;
            keysClosed=keysClosed&&r::InspectDocument(owner,key).presence==r::DocumentPresence::Conflict;}
        result[@"fullKeysClosed"]=@(keysClosed);
        auto next=old;next.policy=1;next.effects[0].policy=1;next.key.request[0]=12;
        doc->NewCommand();const bool staged=r::Stage(owner,next,legacy);doc->AbortCommand();r::Catalog after;
        result[@"abortExact"]=@(staged&&r::Read(doc,after)==r::ReadStatus::Valid&&after.matches(legacy));
        doc->NewCommand();if(!r::Stage(owner,next,legacy)||!doc->CommitCommand())throw Standard_Failure("Probe append");
        r::Catalog dual;if(r::Read(doc,dual)!=r::ReadStatus::Valid)throw Standard_Failure("Probe dual");
        result[@"dualExact"]=@(dual.records.size()==2&&dual.legacyBytes==legacy.legacyBytes&&dual.legacyLabel==legacy.legacyLabel&&!dual.bytes.empty());
        doc->NewCommand();auto third=next;third.key.request[0]=13;
        result[@"staleVersionedRefused"]=@(!r::Stage(owner,third,legacy));doc->AbortCommand();
        auto changed=old;changed.effects[0].state[0]^=1;std::vector<std::uint8_t> changedBytes;
        if(!r::legacy_debug::EncodeLegacy({changed},changedBytes))throw Standard_Failure("Probe changed legacy");
        doc->NewCommand();if(!r::legacy_debug::Write(doc,changedBytes,true,dual.legacyLabel))throw Standard_Failure("Probe changed raw");
        r::Catalog changedCatalog;result[@"staleLegacyRefused"]=@(r::Read(doc,changedCatalog)==r::ReadStatus::Valid&&!changedCatalog.matches(dual)&&!r::Stage(owner,third,dual));doc->AbortCommand();
        auto same=next;same.key.request=old.key.request;std::vector<std::uint8_t> duplicate;
        if(!r::Encode({same},duplicate))throw Standard_Failure("Probe duplicate encoding");
        doc->NewCommand();if(!r::legacy_debug::Write(doc,duplicate,false,dual.label))throw Standard_Failure("Probe duplicate staging");
        r::Catalog rejected;result[@"crossVersionDuplicateRefused"]=@(r::Read(doc,rejected)==r::ReadStatus::Malformed&&rejected.records.empty());doc->AbortCommand();
        auto unknown=next;unknown.policy=32767;unknown.effects[0].policy=32767;std::vector<std::uint8_t> unknownBytes;
        if(!r::Encode({unknown},unknownBytes))throw Standard_Failure("Probe unknown encoding");
        doc->NewCommand();if(!r::legacy_debug::Write(doc,unknownBytes,false,dual.label)||!doc->CommitCommand())throw Standard_Failure("Probe unknown commit");
        r::Catalog unsupported;const auto inspected=r::InspectDocument(owner,unknown.key);
        result[@"unknownRetainedUnresolved"]=@(r::Read(doc,unsupported)==r::ReadStatus::Valid&&unsupported.bytes==unknownBytes&&!unsupported.supportsAppend()
            &&inspected.presence==r::DocumentPresence::Present&&inspected.evidence==r::EffectEvidenceStatus::UnsupportedPolicy&&!inspected.effectsCurrent);
        doc->NewCommand();result[@"unknownAppendRefused"]=@(!r::Stage(owner,third,unsupported));doc->AbortCommand();
        if(!doc->Undo())throw Standard_Failure("Probe unknown undo");
        bool placement=true;for(bool oldIDs:{true,false})for(int where=0;where<4;++where){doc->NewCommand();const auto root=doc->GetData()->Root();TDF_Label target;
            if(where==0)target=root;else if(where==1)target=doc->Main();else if(where==2)target=dual.label.FindChild(9000,Standard_True);else target=root.FindChild(dual.label.Tag()+1,Standard_True);
            TDataStd_Integer::Set(target,oldIDs?r::SchemaID():r::VersionedSchemaID(),oldIDs?1:2);
            placement=placement&&r::Read(doc,rejected)==r::ReadStatus::Malformed&&rejected.records.empty();doc->AbortCommand();}
        result[@"reservedPlacementRefused"]=@(placement);
        doc->NewCommand();auto value=TDataStd_AsciiString::Set(dual.legacyLabel.FindChild(1),TCollection_AsciiString("not-hex"));
        result[@"legacyMalformedClosesDual"]=@(r::Read(doc,rejected)==r::ReadStatus::Malformed);doc->AbortCommand();
        doc->NewCommand();TDataStd_Integer::Set(dual.label,r::VersionedSchemaID(),3);
        result[@"unknownSchemaUnavailable"]=@(r::Read(doc,rejected)==r::ReadStatus::Unsupported);doc->AbortCommand();
        // Old components retain their shared128 representation ceiling. The new
        // V3 migration path is exercised independently; StageLegacy stays closed.
        std::vector<r::Record> left,right;for(unsigned i=0;i<64;++i){auto a=old;a.key.request.fill(0);a.key.request[15]=std::uint8_t(i+1);left.push_back(a);
            auto b=next;b.key.request.fill(0);b.key.request[15]=std::uint8_t(i+65);right.push_back(b);}
        std::vector<std::uint8_t> leftBytes,rightBytes;if(!r::legacy_debug::EncodeLegacy(left,leftBytes)||!r::Encode(right,rightBytes))throw Standard_Failure("Probe budget encode");
        doc->NewCommand();if(!r::legacy_debug::Write(doc,leftBytes,true,dual.legacyLabel)||!r::legacy_debug::Write(doc,rightBytes,false,dual.label))throw Standard_Failure("Probe budget write");
        r::Catalog full;const bool read128=r::Read(doc,full)==r::ReadStatus::Valid&&full.records.size()==128;
        result[@"sharedRecordBudget"]=@(read128&&!r::StageLegacy(owner,third,full));
        auto extra=next;extra.key.request.fill(0);extra.key.request[15]=129;right.push_back(extra);
        if(!r::Encode(right,rightBytes)||!r::legacy_debug::Write(doc,rightBytes,false,dual.label))throw Standard_Failure("Probe129");
        result[@"read129Refused"]=@(r::Read(doc,rejected)==r::ReadStatus::Malformed);doc->AbortCommand();
        doc->NewCommand();TDataStd_Integer::Set(dual.legacyLabel,r::CountID(),int(r::MaximumChunks));TDataStd_Integer::Set(dual.label,r::VersionedCountID(),1);
        result[@"malformedChunkPopulation"]=@(r::Read(doc,rejected)==r::ReadStatus::Malformed);doc->AbortCommand();
        std::vector<r::Record> decoded;std::vector<std::uint8_t> tooLarge(r::MaximumBytes+1,0);
        result[@"singleWireByteOverflow"]=@(r::Decode(tooLarge,decoded)==r::ReadStatus::Malformed&&decoded.empty());
        // Both components below are independently encoded, decoded and Read as
        // complete catalogs. All installation attempts stay in disposable DEBUG
        // transactions and abort back to the exact original two-catalog snapshot.
        // Fixed full hex chunks make the chunk ceiling dominate the byte ceiling:
        // do not describe the second fixture as independent byte-branch coverage.
        auto aggregateBudget=[&](bool exceedBytes)->NSDictionary * {
            std::vector<r::Record> lhs,rhs;
            for(unsigned i=0;i<68;++i){
                auto a=old,b=next;a.effects.clear();b.effects.clear();
                a.key.request.fill(0);b.key.request.fill(0);
                a.key.request[15]=std::uint8_t(i+1);b.key.request[15]=std::uint8_t(i+69);
                for(unsigned j=0;j<16;++j){
                    auto le=e,re=e;le.policy=0;re.policy=1;
                    le.entity[15]=re.entity[15]=std::uint8_t(j+1);
                    le.definition[15]=re.definition[15]=std::uint8_t(j+1);
                    le.featureID[15]=re.featureID[15]=std::uint8_t(j+1);
                    // The chunk-only fixture has 15 fewer effects split 2/13
                    // across the last records, retaining valid nonempty records.
                    if(exceedBytes||i!=67||j<14)a.effects.push_back(le);
                    if(exceedBytes||i!=67||j<3)b.effects.push_back(re);
                }
                lhs.push_back(std::move(a));rhs.push_back(std::move(b));
            }
            std::vector<std::uint8_t> lb,rb;std::vector<r::Record> ld,rd;
            if(!r::legacy_debug::EncodeLegacy(lhs,lb)||!r::Encode(rhs,rb))throw Standard_Failure("Aggregate encode");
            const bool leftDecoded=r::Decode(lb,ld)==r::ReadStatus::Valid&&ld.size()==lhs.size();
            const bool rightDecoded=r::Decode(rb,rd)==r::ReadStatus::Valid&&rd.size()==rhs.size();
            std::set<r::UUID> requests;bool unique=true;std::size_t effects=0;
            for(const auto *records:{&lhs,&rhs})for(const auto& record:*records){unique=unique&&requests.insert(record.key.request).second;effects+=record.effects.size();}
            auto reset=[&]{dual.legacyLabel.ForgetAllAttributes(Standard_True);dual.label.ForgetAllAttributes(Standard_True);};
            auto exactPopulation=[&](const TDF_Label& label,bool legacy,const std::vector<std::uint8_t>& wire)->bool {
                Handle(TDataStd_Integer) count;
                if(!label.FindAttribute(legacy?r::CountID():r::VersionedCountID(),count))return false;
                const auto expectedHex=r::Hex(wire);const auto expectedCount=(expectedHex.size()+r::ChunkBytes-1)/r::ChunkBytes;
                if(count->Get()!=int(expectedCount))return false;
                std::size_t populated=0;
                for(TDF_ChildIterator it(label,Standard_False);it.More();it.Next())if(it.Value().HasAttribute())++populated;
                if(populated!=expectedCount)return false;
                for(std::size_t i=0;i<expectedCount;++i){
                    Handle(TDataStd_AsciiString) value;
                    if(!label.FindChild(int(i+1),Standard_False).FindAttribute(TDataStd_AsciiString::GetID(),value)
                        ||std::string(value->Get().ToCString())!=expectedHex.substr(i*r::ChunkBytes,r::ChunkBytes))return false;
                }
                return true;
            };
            auto restored=[&]{r::Catalog snapshot;return r::Read(doc,snapshot)==r::ReadStatus::Valid&&snapshot.matches(dual);};
            doc->NewCommand();reset();
            if(!r::legacy_debug::Write(doc,lb,true,dual.legacyLabel))throw Standard_Failure("Aggregate left install");
            r::Catalog alone;const bool leftPopulation=exactPopulation(dual.legacyLabel,true,lb);
            const bool leftRead=r::Read(doc,alone)==r::ReadStatus::Valid&&alone.legacyBytes==lb&&alone.bytes.empty()
                &&alone.records.size()==lhs.size()&&alone.label.IsNull();
            doc->AbortCommand();const bool leftAbort=restored();
            doc->NewCommand();reset();
            if(!r::legacy_debug::Write(doc,rb,false,dual.label))throw Standard_Failure("Aggregate right install");
            const bool rightPopulation=exactPopulation(dual.label,false,rb);
            const bool rightRead=r::Read(doc,alone)==r::ReadStatus::Valid&&alone.bytes==rb&&alone.legacyBytes.empty()
                &&alone.records.size()==rhs.size()&&alone.legacyLabel.IsNull();
            doc->AbortCommand();const bool rightAbort=restored();
            doc->NewCommand();reset();
            if(!r::legacy_debug::Write(doc,lb,true,dual.legacyLabel)||!r::legacy_debug::Write(doc,rb,false,dual.label))throw Standard_Failure("Aggregate pair install");
            const bool pairPopulation=exactPopulation(dual.legacyLabel,true,lb)&&exactPopulation(dual.label,false,rb);
            r::Catalog failed;const bool pairRejected=r::Read(doc,failed)==r::ReadStatus::Malformed;
            const bool unpublished=failed.records.empty()&&failed.bytes.empty()&&failed.legacyBytes.empty()&&failed.label.IsNull()&&failed.legacyLabel.IsNull();
            doc->AbortCommand();const bool pairAbort=restored();
            return @{@"leftDecoded":@(leftDecoded),@"rightDecoded":@(rightDecoded),@"leftRead":@(leftRead),@"rightRead":@(rightRead),
                @"uniqueRequests":@(unique),@"leftPopulation":@(leftPopulation),@"rightPopulation":@(rightPopulation),@"pairPopulation":@(pairPopulation),
                @"pairRejected":@(pairRejected),@"unpublished":@(unpublished),@"abortExact":@(leftAbort&&rightAbort&&pairAbort),
                @"leftBytes":@(lb.size()),@"rightBytes":@(rb.size()),@"leftChunks":@((lb.size()*2+r::ChunkBytes-1)/r::ChunkBytes),
                @"rightChunks":@((rb.size()*2+r::ChunkBytes-1)/r::ChunkBytes),@"leftRecords":@(lhs.size()),@"rightRecords":@(rhs.size()),
                @"effects":@(effects),@"maximumBytes":@(r::MaximumBytes),@"maximumChunks":@(r::MaximumChunks),@"maximumRecords":@(r::MaximumRecords)};
        };
        result[@"aggregateChunkBudget"]=aggregateBudget(false);
        result[@"aggregateByteBudget"]=aggregateBudget(true);
        doc->NewCommand();const auto branch=doc->Main().FindChild(20000,Standard_True);
        for(int i=1;i<=100001;++i)branch.FindChild(i,Standard_True);
        result[@"sharedLabelBudget"]=@(r::Read(doc,rejected)==r::ReadStatus::Malformed);doc->AbortCommand();
        result[@"verifiedUnavailable"]=@(r::QueryVerifiedReceipt()==r::VerifiedQueryStatus::Unavailable);
    }catch(...){result=nil;}
    try{if(!owner.IsNull()&&!owner->Document().IsNull()){const auto doc=owner->Document();if(doc->HasOpenCommand())doc->AbortCommand();
        const auto app=Handle(TDocStd_Application)::DownCast(doc->Application());if(!app.IsNull())app->Close(doc);}}catch(...){}
    return result;
}

- (NSDictionary *)debugReceiptGeometryStream:(NSData *)data chunkSize:(NSUInteger)chunkSize {
    if(!NSThread.isMainThread||![data isKindOfClass:NSData.class]||data.length==0
        ||data.length>8*1024*1024||chunkSize==0||chunkSize>4096)return @{@"valid":@NO};
    try {
        core3d::receipt::GeometryStream buffer;std::vector<std::uint8_t> bytes;
        buffer.debugBytes=&bytes;std::ostream stream(&buffer);
        const char *input=static_cast<const char *>(data.bytes);
        for(NSUInteger offset=0;offset<data.length;offset+=chunkSize)
            stream.write(input+offset,std::streamsize(MIN(chunkSize,data.length-offset)));
        core3d::receipt::Digest digest;
        if(!stream.good()||!buffer.finish(digest))return @{@"valid":@NO};
        return @{@"valid":@YES,@"bytes":[NSData dataWithBytes:bytes.data() length:bytes.size()],
            @"sha256":[NSData dataWithBytes:digest.data() length:digest.size()]};
    }catch(...){return @{@"valid":@NO};}
}

- (NSDictionary *)debugInspectNativeReceipt:(NSString *)requestID conflict:(BOOL)conflict {
    namespace r = core3d::receipt;
    if (![NSThread isMainThread] || !GLController || !GLController.viewer)
        return @{@"presence":@"unavailable",@"effectsCurrent":@NO,@"verified":@"unavailable"};
    const auto owner = GLController.viewer->getDocument();
    r::Catalog catalog; r::Inspection inspected;
    r::UUID request;
    if (!owner.IsNull() && r::ParseUUID(requestID.UTF8String,request)) {
        const auto status = r::Read(owner->Document(),catalog);
        if (status == r::ReadStatus::Absent) inspected.presence = r::DocumentPresence::Absent;
        else if (status == r::ReadStatus::Malformed) inspected.presence = r::DocumentPresence::Conflict;
        else if (status == r::ReadStatus::Valid) {
            inspected.presence = r::DocumentPresence::Absent;
            r::Record record;
            if (catalog.lookup(request,record)) {
                auto key = record.key; if (conflict) key.command[0] ^= 1;
                inspected = r::InspectDocument(owner,key);
            }
        }
    }
    NSString *presence = @"unavailable";
    switch (inspected.presence) {
        case r::DocumentPresence::Absent: presence = @"absent"; break;
        case r::DocumentPresence::Present: presence = @"present"; break;
        case r::DocumentPresence::Conflict: presence = @"conflict"; break;
        case r::DocumentPresence::Unavailable: break;
    }
    NSString *evidence=@"unavailable";
    switch(inspected.evidence){
        case r::EffectEvidenceStatus::Current:evidence=@"current";break;
        case r::EffectEvidenceStatus::Mismatch:evidence=@"mismatch";break;
        case r::EffectEvidenceStatus::LegacyUnversioned:evidence=@"legacy-unversioned";break;
        case r::EffectEvidenceStatus::UnsupportedPolicy:evidence=@"unsupported-policy";break;
        case r::EffectEvidenceStatus::Unavailable:break;
    }
    NSMutableDictionary *result = [@{@"evidence":evidence,@"presence":presence,@"effectsCurrent":@(inspected.effectsCurrent),
        @"verified":r::QueryVerifiedReceipt() == r::VerifiedQueryStatus::Unavailable ? @"unavailable" : @"invalid"} mutableCopy];
    if (NSDictionary *preservation=Core3DReceiptPreservation(owner)) result[@"preservation"]=preservation;
    if (inspected.effects.size() == 1) {
        result[@"effectDiagnostic"]=Core3DReceiptEffectDiagnostic(owner,inspected.effects[0]);
        result[@"geometrySHA"] = [NSString stringWithUTF8String:r::Hex(inspected.effects[0].geometry).c_str()];
        result[@"stateSHA"] = [NSString stringWithUTF8String:r::Hex(inspected.effects[0].state).c_str()];
    }
    return result;
}

- (void)debugNativeTombstoneProbe:(NSInteger)scenario completion:(void (^)(NSDictionary *))completion {
    dispatch_async(dispatch_get_global_queue(QOS_CLASS_UTILITY,0), ^{
        NSDictionary *result = Core3DRunNativeTombstoneProbe(scenario);
        dispatch_async(dispatch_get_main_queue(), ^{completion(result);});
    });
}
- (BOOL)debugNativeTombstoneRejectsMainThread {
    core3d::tombstone::Key key;
    key.accountScope.fill(1);key.document.fill(2);key.request.fill(3);key.command.fill(4);key.execution.fill(5);
    return [NSThread isMainThread]
        && core3d::tombstone::Store::ForTesting("/unused-by-main-thread-refusal").reserve(key)
            == core3d::tombstone::Reservation::Unavailable;
}

- (NSDictionary *)debugInspectModelingDescriptorData:(NSData *)data {
    namespace r=core3d::request;
    if (![NSThread isMainThread]||![data isKindOfClass:NSData.class]||data.length==0||data.length>r::MaximumBytes) return @{@"valid":@NO};
    try {
        const auto bytes=static_cast<const std::uint8_t*>(data.bytes);
        r::Descriptor descriptor;std::vector<std::uint8_t> canonical;r::Digest hash;
        if (!r::Decode(std::vector<std::uint8_t>(bytes,bytes+data.length),descriptor)
            ||!r::Encode(descriptor,canonical)||!r::CommandHash(descriptor,hash)) return @{@"valid":@NO};
        NSMutableArray *values=[NSMutableArray array];
        for(const auto&part:descriptor.parts) {
            NSMutableArray *bits=[NSMutableArray array];for(double x:part.values){std::uint64_t b;std::memcpy(&b,&x,8);[bits addObject:@(b)];}
            [values addObject:bits];
        }
        return @{@"valid":@YES,@"bytes":[NSData dataWithBytes:canonical.data() length:canonical.size()],
            @"commandSHA256":[NSData dataWithBytes:hash.data() length:hash.size()],@"valueBits":values};
    } catch (...) {return @{@"valid":@NO};}
}
- (NSData *)debugModelingPreparedDescriptor:(Core3DModelingPreparedRequest *)request {
    if (![self isModelingPreparedRequestCurrent:request]) return nil;
    std::vector<std::uint8_t> bytes;
    return core3d::request::Encode(request->_requestDescriptor,bytes)?[NSData dataWithBytes:bytes.data() length:bytes.size()]:nil;
}
- (NSDictionary *)debugModelingAdmissionStateProbe {
    namespace r=core3d::request;
    r::Key key;key.accountScope.fill(1);key.document.fill(2);key.request.fill(3);key.command.fill(4);key.execution.fill(5);
    int bindings=0,closedReplies=0,stopPhases=0;
    for(int field=0;field<5;++field){auto foreign=key;
        if(field==0)foreign.accountScope[0]^=1;else if(field==1)foreign.document[0]^=1;
        else if(field==2)foreign.request[0]^=1;else if(field==3)foreign.command[0]^=1;else foreign.execution[0]^=1;
        r::Admission a(key);if(a.begin()&&!a.reservation(foreign,r::ReserveReply::FirstReserved)&&a.phase()==r::Phase::Reserving
            &&a.reservation(key,r::ReserveReply::FirstReserved)&&!a.reservation(key,r::ReserveReply::FirstReserved))++bindings;
    }
    const std::array<r::ReserveReply,5> replies={r::ReserveReply::PreviouslySeen,r::ReserveReply::Conflict,r::ReserveReply::Capacity,r::ReserveReply::Busy,r::ReserveReply::Uncertain};
    const std::array<r::Terminal,5> terminals={r::Terminal::PreviouslySeen,r::Terminal::Conflict,r::Terminal::Capacity,r::Terminal::Busy,r::Terminal::Uncertain};
    for(std::size_t i=0;i<replies.size();++i){r::Admission a(key);
        if(a.begin()&&!a.reservation(key,replies[i])&&a.phase()==r::Phase::Terminal&&a.terminal()==terminals[i]
            &&!a.begin()&&!a.beginGeometry(true)&&!a.reservation(key,r::ReserveReply::FirstReserved))++closedReplies;
    }
    for(int phase=0;phase<5;++phase){r::Admission a(key);
        if(phase>=1)a.begin();if(phase>=2)a.reservation(key,r::ReserveReply::FirstReserved);
        if(phase>=3)a.beginGeometry(true);if(phase>=4)a.geometryReady(true,true);
        if(a.stop()&&a.terminal()==r::Terminal::Cancelled&&!a.stop()&&!a.begin()
            &&!a.reservation(key,r::ReserveReply::FirstReserved)&&!a.takeForOrdinary(true))++stopPhases;
    }
    r::Admission delivered(key);const bool once=delivered.begin()&&delivered.reservation(key,r::ReserveReply::FirstReserved)
        &&delivered.beginGeometry(true)&&delivered.geometryReady(true,true)&&delivered.takeForOrdinary(true)
        &&!delivered.takeForOrdinary(true)&&!delivered.stop()&&delivered.terminal()==r::Terminal::HandedToOrdinary;
    int fences=0;
    for(int phase=0;phase<3;++phase){r::Admission a(key);a.begin();a.reservation(key,r::ReserveReply::FirstReserved);
        bool refused=false;if(phase==0)refused=!a.beginGeometry(false);
        else {a.beginGeometry(true);if(phase==1)refused=!a.geometryReady(true,false);
            else {a.geometryReady(true,true);refused=!a.takeForOrdinary(false);}}
        if(refused&&a.terminal()==r::Terminal::Rejected&&!a.takeForOrdinary(true))++fences;
    }
    r::Admission failed(key);failed.begin();failed.reservation(key,r::ReserveReply::FirstReserved);failed.beginGeometry(true);
    const bool buildRefused=!failed.geometryReady(false,true)&&failed.terminal()==r::Terminal::Rejected;
    r::Admission invalid(r::Key{});
    return @{@"bindingRejections":@(bindings),@"closedReplies":@(closedReplies),@"stopPhases":@(stopPhases),
        @"fenceRejections":@(fences),@"once":@(once),@"buildRefused":@(buildRefused),@"invalidKeyRefused":@(!invalid.begin())};
}

- (NSDictionary *)debugCreationReceiptOwnerState {
    if(!NSThread.isMainThread||!GLController||!GLController.viewer)return @{};
    const auto owner=GLController.viewer->getDocument();
    if(owner.IsNull()||owner->Document().IsNull())return @{};
    core3d::receipt::Catalog catalog;const auto status=core3d::receipt::Read(owner->Document(),catalog);
    return @{@"read":@(int(status)),@"recordCount":@(catalog.count()),
        @"hasConstructionContext":@(_modelingConstructionContext!=nil),
        @"hasPendingReservation":@(_pendingModelingReservation!=nil),
        @"openCommand":@(owner->Document()->HasOpenCommand())};
}
- (BOOL)debugFailCreationReceiptResolutionBeforeRelease:(Core3DModelingPreparedRequest *)request {
    if(!NSThread.isMainThread||!request||request->_requestOwner!=self)return NO;
    return core3d::NativeModelingPermitIssuer::failBeforeRelease(request);
}
- (Core3DTransformInspectorPositionCommitResult)debugExecuteReservedPlacement:(Core3DModelingPreparedRequest *)request
    receiptFailure:(BOOL)receiptFailure {
    return [self core3d_executeReservedPlacement:request receiptFailure:receiptFailure];
}
- (NSDictionary *)debugPlacementRequestCodecProbe {
    namespace q=core3d::request;namespace r=core3d::receipt;
    try{
        q::Descriptor legacy;legacy.operation=q::Operation::CreateProfile;q::Part part;part.schema=1;part.values={0.0,-0.0};legacy.parts={part};
        std::vector<std::uint8_t> bytes;const bool oldCommand=q::Encode(legacy,bytes)&&r::Hex(bytes)=="53594d44010501000101000000000000000200000000000000000000000000000000000080";
        r::Record record;record.operation=r::Operation::RebuildProfile;
        record.key.accountScope.fill(1);record.key.document.fill(2);record.key.request.fill(3);record.key.command.fill(4);record.key.execution.fill(5);
        r::Effect effect;effect.entity.fill(6);effect.definition.fill(7);effect.featureID.fill(8);effect.geometry.fill(9);effect.state.fill(10);record.effects={effect};
        const bool oldReceipt=r::Encode({record},bytes)&&r::Hex(bytes)=="53595243020001000101010101010101010101010101010101010101010101010101010101010101020202020202020202020202020202020303030303030303030303030303030304040404040404040404040404040404040404040404040404040404040404040505050505050505050505050505050505050505050505050505050505050505040101000106060606060606060606060606060606070707070707070707070707070707070808080808080808080808080808080809090909090909090909090909090909090909090909090909090909090909090a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a0a86ea7dbde0df6079095b8c7e74b3b2f6aba9de9fcff7d61e9de2aa9cc7acd398";
        unsigned oldUnknown=0;
        for(auto op:{r::Operation::CreateEnclosure,r::Operation::RebuildEnclosure,r::Operation::CreateAssembly,r::Operation::RebuildProfile,r::Operation::RebuildLoftStation}){
            auto unknown=record;unknown.operation=op;unknown.policy=r::ExactPlacementPolicy8193;unknown.effects.front().policy=unknown.policy;
            unknown.effects.front().feature=op==r::Operation::RebuildLoftStation?r::Feature::RectangularLoft:
                (op==r::Operation::CreateEnclosure||op==r::Operation::RebuildEnclosure?r::Feature::Enclosure:r::Feature::Profile);
            std::vector<r::Record> decoded;r::Catalog catalog;
            if(r::Encode({unknown},bytes)&&r::Decode(bytes,decoded)==r::ReadStatus::Valid){
                catalog.records=decoded;std::vector<std::uint8_t> reencoded;
                if(!catalog.supportsAppend()&&r::Encode(decoded,reencoded)&&reencoded==bytes)++oldUnknown;
            }
        }
        q::Descriptor placement;placement.operation=q::Operation::SetPlacement;q::PlacementIntent intent;
        intent.entity.fill(6);intent.definition.fill(7);intent.geometry.fill(9);intent.recipe.fill(11);intent.state.fill(10);
        intent.family=5;intent.schema=0;intent.metersPerUnit=0.001;intent.value=-0.0;placement.placement=intent;
        q::Descriptor decoded;const bool bare=q::Encode(placement,bytes)&&bytes.size()==180&&bytes[4]==3&&q::Decode(bytes,decoded);
        unsigned negatives=0;
        for(unsigned i=0;i<12;++i){auto bad=placement;switch(i){
            case 0:bad.parts={part};break;case 1:bad.placement->feature.fill(1);break;case 2:bad.placement->schema=1;break;
            case 3:bad.placement->family=1;break;case 4:bad.placement->axis=3;break;case 5:bad.placement->kind=2;break;
            case 6:bad.placement->value=std::numeric_limits<double>::quiet_NaN();break;
            case 7:bad.placement->metersPerUnit=std::numeric_limits<double>::max();break;
            case 8:bad.placement->geometry.fill(0);break;case 9:bad.placement->state.fill(0);break;
            case 10:bad.operation=q::Operation::CreateProfile;break;case 11:bad.loftStationEdit=q::LoftStationEdit{};break;}
            std::vector<std::uint8_t> refused;if(!q::Encode(bad,refused)&&refused.empty())++negatives;
        }
        if(!q::Encode(placement,bytes)||bytes.size()!=180)return @{};
        unsigned malformed=0;
        for(unsigned i=0;i<4;++i){auto bad=bytes;if(i==0)bad[4]=1;else if(i==1)bad.push_back(0);else if(i==2)bad.pop_back();else bad[6]=1;
            q::Descriptor refused;if(!q::Decode(bad,refused))++malformed;}
        auto placed=record;placed.operation=r::Operation::SetPlacement;placed.policy=r::ExactPlacementPolicy8193;
        placed.effects.front().policy=placed.policy;placed.effects.front().feature=r::Feature::PlacementBareSolid;placed.effects.front().featureID.fill(0);
        std::vector<r::Record> decodedRecords;const bool bareRecord=r::Encode({placed},bytes)&&r::Decode(bytes,decodedRecords)==r::ReadStatus::Valid;
        auto invalid=placed;invalid.effects.front().feature=r::Feature::PlacementObject;
        const bool absentRefused=!r::Valid(invalid);invalid=record;invalid.effects.front().featureID.fill(0);
        const bool legacyAbsentRefused=!r::Valid(invalid);
        placed.policy=8194;placed.effects.front().policy=8194;
        r::Catalog unsupported;const bool unknownPlacement=r::Encode({placed},bytes)&&r::Decode(bytes,unsupported.records)==r::ReadStatus::Valid&&!unsupported.supportsAppend();
        return @{@"oldCommand":@(oldCommand),@"oldReceipt":@(oldReceipt),@"oldUnknown":@(oldUnknown),@"bare":@(bare),
            @"negatives":@(negatives),@"malformed":@(malformed),@"bareRecord":@(bareRecord),@"absentRefused":@(absentRefused),
            @"legacyAbsentRefused":@(legacyAbsentRefused),@"unknownPlacement":@(unknownPlacement)};
    }catch(...){return @{};}
}
- (NSDictionary *)debugPlacementReceiptState:(Core3DModelingPreparedRequest *)request {
    if(!NSThread.isMainThread||!request||request->_requestOwner!=self||!GLController||!GLController.viewer)return @{};
    try{
        const auto document=GLController.viewer->getDocument();core3d::receipt::Catalog catalog;
        if(document.IsNull()||document->Document().IsNull())return @{};
        const auto read=core3d::receipt::Read(document->Document(),catalog);
        bool current=false;std::size_t matching=0;
        core3d::receipt::Record record;
        if(catalog.lookup(request->_requestKey.request,record)){
            ++matching;
            if(record.operation==core3d::receipt::Operation::SetPlacement&&record.policy==core3d::receipt::ExactPlacementPolicy8193
                &&record.effects.size()==1&&request->_requestContext->_planningPlacementSnapshot){
                TDF_Label label;core3d::placement::Evidence evidence;
                const auto entity=request->_requestContext->_planningPlacementSnapshot.entityIdentifier;
                current=entity.UTF8String&&core3d::placement::Find(document,entity.UTF8String,label)
                    &&core3d::placement::Capture(document,label,evidence)&&core3d::placement::Effect(evidence)==record.effects.front();
            }
        }
        const auto resolution=request->_requestCommitPermit?request->_requestCommitPermit->resolution():nullptr;
        return @{@"read":@(int(read)),@"records":@(catalog.count()),@"matching":@(matching),@"current":@(current),
            @"resolution":@(resolution?int(resolution->state()):-1),@"openCommand":@(document->Document()->HasOpenCommand()),
            @"versionedBytes":[NSData dataWithBytes:catalog.bytes.data() length:catalog.bytes.size()],
            @"legacyBytes":[NSData dataWithBytes:catalog.legacyBytes.data() length:catalog.legacyBytes.size()]};
    }catch(...){return @{};}
}
- (BOOL)debugExecuteReservedCreation:(Core3DModelingPreparedRequest *)request
    receiptFailure:(BOOL)receiptFailure completion:(void (^)(Core3DProfileConstructionResult))completion {
    if(!NSThread.isMainThread||!completion||!GLController||!GLController.viewer)return NO;
    const BOOL accepted=[self core3d_startReservedCreation:request completion:completion];
    if(accepted&&receiptFailure)core3d::NativeModelingPermitIssuer::failReceiptStage(request);
    return accepted;
}
- (BOOL)debugCorruptPreparedRebuildSourceEffect:(Core3DModelingPreparedRequest *)request {
    if(!NSThread.isMainThread||!request||request->_requestOwner!=self||request->_requestAdmitted
        ||!request->_requestSourceEffect)return NO;
    request->_requestSourceEffect->geometry[0]^=1;return YES;
}
- (BOOL)debugExecuteReservedRebuild:(Core3DModelingPreparedRequest *)request
    receiptFailure:(BOOL)receiptFailure completion:(void (^)(Core3DProfileConstructionResult))completion {
    if(!NSThread.isMainThread||!completion||!GLController||!GLController.viewer)return NO;
    const BOOL accepted=[self core3d_startReservedRebuild:request completion:completion];
    if(accepted&&receiptFailure)core3d::NativeModelingPermitIssuer::failReceiptStage(request);
    return accepted;
}
- (NSDictionary *)debugCreationReceiptState:(Core3DModelingPreparedRequest *)request {
    if(!NSThread.isMainThread||![request isKindOfClass:Core3DModelingPreparedRequest.class]
        ||!GLController||!GLController.viewer)return @{};
    const auto owner=GLController.viewer->getDocument();if(owner.IsNull())return @{};
    core3d::receipt::Key key;key.accountScope=request->_requestKey.accountScope;key.document=request->_requestKey.document;
    key.request=request->_requestKey.request;key.command=request->_requestKey.command;key.execution=request->_requestKey.execution;
    core3d::receipt::Catalog catalog;const auto read=core3d::receipt::Read(owner->Document(),catalog);
    const auto inspection=core3d::receipt::InspectDocument(owner,key);
    NSMutableArray *features=[NSMutableArray array],*entities=[NSMutableArray array],*definitions=[NSMutableArray array];
    for(const auto&effect:inspection.effects){
        [features addObject:[[NSUUID alloc] initWithUUIDBytes:effect.featureID.data()].UUIDString];
        [entities addObject:[[NSUUID alloc] initWithUUIDBytes:effect.entity.data()].UUIDString];
        [definitions addObject:[[NSUUID alloc] initWithUUIDBytes:effect.definition.data()].UUIDString];
    }
    NSMutableArray *frozen=[NSMutableArray array];
    for(const auto&identifier:request->_requestFeatureIDs)[frozen addObject:[[NSUUID alloc] initWithUUIDBytes:identifier.data()].UUIDString];
    const auto resolution=request->_requestCommitPermit?request->_requestCommitPermit->resolution():nullptr;
    const auto ordinary=GLController.viewer->debugOrdinaryEditController();
    const bool stampRetained=ordinary&&ordinary->debugCommandStamp().isRetained();
    // This is explicitly document-only DEBUG evidence. It neither performs a
    // Store lookup nor upgrades QueryVerifiedReceipt from Unavailable.
    return @{@"commandStampRetained":@(stampRetained),@"read":@(int(read)),@"presence":@(int(inspection.presence)),@"effectsCurrent":@(inspection.effectsCurrent),
        @"legacyCatalogBytes":[NSData dataWithBytes:catalog.legacyBytes.data() length:catalog.legacyBytes.size()],
        @"catalogBytes":[NSData dataWithBytes:catalog.bytes.data() length:catalog.bytes.size()],@"recordCount":@(catalog.count()),
        @"featureIDs":features,@"entityIDs":entities,@"definitionIDs":definitions,@"frozenFeatureIDs":frozen,
        @"resolution":@(resolution?int(resolution->state()):-1),@"openCommand":@(owner->Document()->HasOpenCommand()),
        @"verifiedQueryAvailable":@(core3d::receipt::QueryVerifiedReceipt()!=core3d::receipt::VerifiedQueryStatus::Unavailable)};
}


- (NSDictionary *)debugCreationReceiptEffectDiagnostics:(Core3DModelingPreparedRequest *)request {
    if(!NSThread.isMainThread||![request isKindOfClass:Core3DModelingPreparedRequest.class]
        ||!GLController||!GLController.viewer)return @{@"version":@1,@"available":@NO};
    try {
        namespace r=core3d::receipt;
        const auto owner=GLController.viewer->getDocument();
        if(owner.IsNull()||owner->Document().IsNull()||owner->Document()->HasOpenCommand())
            return @{@"version":@1,@"available":@NO};
        r::Key key;key.accountScope=request->_requestKey.accountScope;key.document=request->_requestKey.document;
        key.request=request->_requestKey.request;key.command=request->_requestKey.command;key.execution=request->_requestKey.execution;
        r::Catalog catalog;const auto read=r::Read(owner->Document(),catalog);
        const auto inspection=r::InspectDocument(owner,key);
        // These two failing fixtures contain at most two effects. Bound this
        // optional DEBUG capture independently of the public 16-effect catalog.
        // Each existing geometry sink remains limited to 8MiB of exact bytes.
        if(read!=r::ReadStatus::Valid||inspection.presence!=r::DocumentPresence::Present
            ||inspection.effects.empty()||inspection.effects.size()>2)
            return @{@"version":@1,@"available":@NO,@"read":@(int(read)),@"presence":@(int(inspection.presence))};
        NSMutableArray *effects=[NSMutableArray arrayWithCapacity:inspection.effects.size()];
        for(const auto& effect:inspection.effects)
            [effects addObject:Core3DReceiptEffectDiagnostic(owner,effect)];
        auto hex=[](const auto& bytes){return [NSString stringWithUTF8String:r::Hex(bytes).c_str()];};
        NSString *catalogBase64=[[NSData dataWithBytes:catalog.bytes.data() length:catalog.bytes.size()]
            base64EncodedStringWithOptions:0];
        return @{@"version":@1,@"available":@YES,@"read":@(int(read)),@"presence":@(int(inspection.presence)),
            @"effectsCurrent":@(inspection.effectsCurrent),@"requestHex":hex(key.request),@"documentHex":hex(key.document),
            @"commandHex":hex(key.command),@"executionHex":hex(key.execution),@"catalogBase64":catalogBase64,@"effects":effects};
    } catch (...) {return @{@"version":@1,@"available":@NO};}
}


- (BOOL)debugConfigureAsyncModelingRequest:(Core3DModelingPreparedRequest *)request
    storageScenario:(NSInteger)scenario deliveryGate:(void (^)(void (^)(void)))gate
    receiptFailure:(BOOL)receiptFailure afterStart:(void (^)(void))afterStart {
    if(!NSThread.isMainThread||scenario< -1||scenario>11||scenario==6
        ||![self isModelingPreparedRequestCurrent:request]||request->_requestAsyncConfigured)return NO;
    request->_requestAsyncConfigured=YES;request->_requestAsyncScenario=scenario;
    request->_requestAsyncGate=[gate copy];request->_requestAsyncReceiptFailure=receiptFailure;
    request->_requestAsyncAfterStart=[afterStart copy];return YES;
}
- (BOOL)debugGateAsyncCreationGeometryDelivery:(Core3DModelingPreparedRequest *)request
    gate:(void (^)(void (^)(void)))gate {
    if(!NSThread.isMainThread||!gate||![self isModelingPreparedRequestCurrent:request]
        ||request->_requestAsyncGeometryDeliveryGate
        ||request.evidenceCoverage!=Core3DModelingEvidenceCoverageCanonicalEffect
        ||(request->_requestDescriptor.operation!=core3d::request::Operation::CreateEnclosure
            &&request->_requestDescriptor.operation!=core3d::request::Operation::CreateAssembly))return NO;
    request->_requestAsyncGeometryDeliveryGate=[gate copy];return YES;
}
- (BOOL)debugObserveAsyncModelingCompletion:(Core3DModelingPreparedRequest *)request
    observer:(void (^)(void))observer {
    if(!NSThread.isMainThread||!observer||![self isModelingPreparedRequestCurrent:request]
        ||request->_requestAsyncBeforeCompletion)return NO;
    request->_requestAsyncBeforeCompletion=[observer copy];return YES;
}
- (void)debugLookupPermanentModelingRequest:(Core3DModelingPreparedRequest *)request
    completion:(void (^)(NSDictionary *))completion {
    if(!NSThread.isMainThread||!completion)return;
    if(![request isKindOfClass:Core3DModelingPreparedRequest.class]||!core3d::request::ValidKey(request->_requestKey)){
        completion(@{@"valid":@NO});return;
    }
    // Read-only DEBUG proof, never a public verified query or capability. The
    // main registry owns its callback; utility work receives numeric values only.
    core3d::request::UUID identifier;
    if(!Core3DRequestUUID(NSUUID.UUID,identifier)){completion(@{@"valid":@NO});return;}
    if(!Core3DAsyncStoreProbeCallbacks)Core3DAsyncStoreProbeCallbacks=[NSMutableDictionary dictionary];
    if(Core3DAsyncStoreProbeCallbacks.count>=32){completion(@{@"valid":@NO});return;}
    NSUUID *token=Core3DReservationIdentifier(identifier);
    Core3DAsyncStoreProbeCallbacks[token]=[completion copy];
    const auto key=request->_requestKey;
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW,10*NSEC_PER_SEC),dispatch_get_main_queue(),^{
        NSUUID *expired=Core3DReservationIdentifier(identifier);
        void (^callback)(NSDictionary *)=Core3DAsyncStoreProbeCallbacks[expired];
        [Core3DAsyncStoreProbeCallbacks removeObjectForKey:expired];
        if(callback)callback(@{@"valid":@NO,@"deadline":@YES,@"verifiedQueryAvailable":@NO});
    });
    dispatch_async(dispatch_get_global_queue(QOS_CLASS_UTILITY,0),^{
        const auto presence=core3d::tombstone::Store().lookup(core3d::request::TombstoneKey(key));
        const bool workerMain=::pthread_main_np();
        dispatch_async(dispatch_get_main_queue(),^{
            NSUUID *finished=Core3DReservationIdentifier(identifier);
            void (^callback)(NSDictionary *)=Core3DAsyncStoreProbeCallbacks[finished];
            [Core3DAsyncStoreProbeCallbacks removeObjectForKey:finished];
            if(callback)callback(@{@"valid":@YES,@"presence":@(int(presence)),@"workerMain":@(workerMain),
                @"completionMain":@(NSThread.isMainThread),@"verifiedQueryAvailable":@NO});
        });
    });
}

- (NSDictionary *)debugAsyncModelingStorageState:(Core3DModelingPreparedRequest *)request {
    if(!NSThread.isMainThread||![request isKindOfClass:Core3DModelingPreparedRequest.class]
        ||request->_requestOwner!=self)return @{};
    return request->_requestAsyncStorageSnapshot?:@{};
}

- (BOOL)debugReserveModelingPreparedRequest:(Core3DModelingPreparedRequest *)request scenario:(NSInteger)scenario
    deliveryGate:(void (^)(void (^)(void)))gate completion:(void (^)(NSDictionary *))completion {
    if(!NSThread.isMainThread||!completion||scenario<0||scenario>8)return NO;
    Core3DReservationTestConfig configuration;configuration.scenario=scenario==6?0:unsigned(scenario);
    NSString *pattern=[NSTemporaryDirectory() stringByAppendingPathComponent:@"native-reservation-XXXXXX"];
    std::string path;if(!Core3DRequestString(pattern,configuration.parentPattern.size()-1,path))return NO;
    std::copy(path.begin(),path.end(),configuration.parentPattern.begin());
    auto work=[self core3d_registerReservation:request completion:
        ^(Core3DReservationOutcome outcome,const core3d::request::ReservationDelivery& value,BOOL known) {
            completion(Core3DReservationDebugResult(outcome,value,known));
        }];
    if(!work)return NO;
    Core3DModelingReservationEntry *entry=Core3DReservationEntries[Core3DReservationIdentifier(work->deliveryID)];
    entry->_gate=[gate copy];entry->_manualTestDeadline=scenario==6;
    const auto dispatch=*work;
    dispatch_async(dispatch_get_global_queue(QOS_CLASS_UTILITY,0),^{
        const auto reply=Core3DRunPrivateReservation(dispatch,configuration);
        dispatch_async(dispatch_get_main_queue(),^{Core3DRouteReservationDelivery(reply);});
    });
    return YES;
}
- (BOOL)debugConsumeReservedCapabilityWithoutGeometry:(Core3DModelingPreparedRequest *)request {
    if(!NSThread.isMainThread||!_reservedModelingCapability
        ||_reservedModelingCapability->_entry->_prepared!=request||_reservedModelingCapability->_taken
        ||![self core3d_reservationMatches:request])return NO;
    _reservedModelingCapability->_taken=YES;
    // Only observe/retire the actual private capability. Stage B deliberately
    // has no GeometryReady/ordinary handoff shortcut or model execution.
    [self stopModelingPreparedRequest:request];return YES;
}
- (BOOL)debugExpireModelingReservation:(Core3DModelingPreparedRequest *)request {
    if(!NSThread.isMainThread||!request||request->_requestOwner!=self)return NO;
    Core3DModelingReservationEntry *entry=Core3DReservationEntries[Core3DReservationIdentifier(request->_requestReservationDeliveryID)];
    if(!entry||entry->_prepared!=request)return NO;
    Core3DExpireReservation(entry->_dispatch.deliveryID);return YES;
}
- (BOOL)debugInjectMismatchedModelingReservation:(Core3DModelingPreparedRequest *)request field:(NSInteger)field {
    if(!NSThread.isMainThread||!request||request->_requestOwner!=self||field<0||field>5)return NO;
    Core3DModelingReservationEntry *entry=Core3DReservationEntries[Core3DReservationIdentifier(request->_requestReservationDeliveryID)];
    if(!entry||entry->_prepared!=request)return NO;
    core3d::request::ReservationDelivery invalid;invalid.deliveryID=entry->_dispatch.deliveryID;
    invalid.key=entry->_dispatch.key;invalid.disposition=core3d::tombstone::Reservation::Reserved;
    switch(field){case 0:invalid.key.accountScope[0]^=1;break;case 1:invalid.key.document[0]^=1;break;
        case 2:invalid.key.request[0]^=1;break;case 3:invalid.key.command[0]^=1;break;
        case 4:invalid.key.execution[0]^=1;break;case 5:invalid.deliveryID[0]^=1;break;}
    Core3DDeliverReservation(invalid);
    return Core3DReservationEntries[Core3DReservationIdentifier(entry->_dispatch.deliveryID)]==entry;
}
- (NSDictionary *)debugModelingReservationState:(Core3DModelingPreparedRequest *)request {
    if(!NSThread.isMainThread||!GLController||!GLController.viewer)return @{};
    const auto owner=GLController.viewer->getDocument();
    if(owner.IsNull()||owner->Document().IsNull())return @{};
    const BOOL ours=request&&request->_requestOwner==self;
    return @{@"openCommand":@(owner->Document()->HasOpenCommand()),
        @"pending":@(ours&&_pendingModelingReservation==request),
        @"admitted":@(ours&&request->_requestAdmitted),
        @"reserved":@(ours&&_reservedModelingCapability&&_reservedModelingCapability->_entry->_prepared==request),
        @"registryCount":@(Core3DReservationEntries.count)};
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

- (void)debugSetDocumentReplacementPreparationFailure:(BOOL)preparation restorationAttempts:(NSInteger)attempts {
    if (![NSThread isMainThread] || GLController == nil || GLController.viewer == nullptr) return;
    GLController.viewer->DebugSetDocumentReplacementFaults(preparation, (int)std::max<NSInteger>(0, std::min<NSInteger>(attempts, 100)));
}

- (void)debugFailNextDocumentAdoption {
    if (![NSThread isMainThread] || GLController == nil || GLController.viewer == nullptr) return;
    GLController.viewer->DebugFailNextDocumentAdoption();
}
// No live viewer, document access, mutation or persistent test settings.
+ (NSDictionary<NSString *, id> *)debugEnclosureValues:(NSArray<NSNumber *> *)input
    schema:(NSInteger)schema points:(NSArray<NSArray<NSNumber *> *> *)points cancelled:(BOOL)cancelled {
    if (![input isKindOfClass:[NSArray class]] || input.count > 32
        || ![points isKindOfClass:[NSArray class]] || points.count > 4096)
        return @{@"accepted": @NO, @"bridgeRejected": @YES};
    try {
        std::vector<double> values;values.reserve(input.count);
        for (id value in input) {
            if (![value isKindOfClass:[NSNumber class]]) return @{@"accepted":@NO,@"bridgeRejected":@YES};
            values.push_back([value doubleValue]);
        }
        if (schema < std::numeric_limits<int>::min() || schema > std::numeric_limits<int>::max())
            return @{@"accepted":@NO,@"bridgeRejected":@YES};
        core3d::enclosure::Parameters parameters;
        const bool decoded=core3d::enclosure::Decode(int(schema),values,parameters);
        if (!decoded) return @{@"accepted":@NO,@"bridgeRejected":@NO};
        std::vector<gp_Pnt> samples;samples.reserve(points.count);
        for (id row in points) {
            if (![row isKindOfClass:[NSArray class]] || [row count]!=3)
                return @{@"accepted":@NO,@"bridgeRejected":@YES};
            double coordinates[3];
            for (NSUInteger i=0;i<3;++i) {
                id number=row[i];
                if (![number isKindOfClass:[NSNumber class]]) return @{@"accepted":@NO,@"bridgeRejected":@YES};
                coordinates[i]=[number doubleValue];
                if (!std::isfinite(coordinates[i]) || std::abs(coordinates[i])>1e6)
                    return @{@"accepted":@NO,@"bridgeRejected":@YES};
            }
            samples.emplace_back(coordinates[0],coordinates[1],coordinates[2]);
        }
        auto token=std::make_shared<std::atomic_bool>(bool(cancelled));
        core3d::EnclosureSolidResult geometry;
        const bool built=core3d::BuildEnclosureSolidGeometry(parameters.definition,token,geometry);
        if (!built) return @{@"accepted":@YES,@"built":@NO,@"emptyResult":@(geometry.solid.IsNull()),@"volume":@(geometry.volume)};
        std::vector<double> encoded;const bool recoded=core3d::enclosure::Encode(parameters,encoded);
        NSMutableArray *roundtrip=[NSMutableArray arrayWithCapacity:encoded.size()];
        for (double value:encoded) [roundtrip addObject:@(value)];
        NSMutableArray *bounds=[NSMutableArray arrayWithCapacity:6];
        for (double value:geometry.bounds) [bounds addObject:@(value)];
        NSMutableArray *states=[NSMutableArray arrayWithCapacity:samples.size()];
        for (const auto& point:samples) {
            BRepClass3d_SolidClassifier classifier(geometry.solid,point,1e-7);
            switch(classifier.State()) {
                case TopAbs_IN:[states addObject:@"inside"];break;
                case TopAbs_OUT:[states addObject:@"outside"];break;
                case TopAbs_ON:[states addObject:@"boundary"];break;
                default:[states addObject:@"unknown"];break;
            }
        }
        return @{@"accepted":@YES,@"built":@YES,@"recoded":@(recoded),@"values":[roundtrip copy],
            @"bounds":[bounds copy],@"volume":@(geometry.volume),@"classifications":[states copy],
            @"schema":@(int(encoded.front()))};
    } catch (...) {return @{@"accepted":@NO,@"exception":@YES};}
}

+ (NSDictionary<NSString *, id> *)debugEnclosureUpdateValues:(NSArray<NSNumber *> *)input
    dimension:(NSInteger)dimension value:(double)value {
    if (![input isKindOfClass:[NSArray class]] || input.count!=9 || dimension<0 || dimension>5)
        return @{@"accepted":@NO};
    try {
        std::vector<double> values;values.reserve(input.count);
        for (id number in input) {
            if (![number isKindOfClass:[NSNumber class]]) return @{@"accepted":@NO};
            values.push_back([number doubleValue]);
        }
        core3d::enclosure::Parameters parameters;
        if (!core3d::enclosure::Decode(1,values,parameters)) return @{@"accepted":@NO};
        const auto candidate=core3d::UpdatedEnclosureDimension(parameters.definition,
            static_cast<core3d::EnclosureDimension>(dimension),value);
        if (!candidate) return @{@"accepted":@NO};
        const bool unchanged=candidate->plane==parameters.definition.plane
            && candidate->dimensions.IsEqual(parameters.definition.dimensions);
        parameters.definition=*candidate;std::vector<double> encoded;
        if (!core3d::enclosure::Encode(parameters,encoded)) return @{@"accepted":@NO};
        NSMutableArray *output=[NSMutableArray arrayWithCapacity:encoded.size()];
        for (double scalar:encoded) [output addObject:@(scalar)];
        return @{@"accepted":@YES,@"unchanged":@(unchanged),@"values":[output copy]};
    } catch (...) {return @{@"accepted":@NO,@"exception":@YES};}
}

+ (NSDictionary<NSString *, id> *)debugDetachedPlanarSweep:(NSInteger)fixture
    metersPerUnit:(double)metersPerUnit plane:(NSInteger)plane frame:(NSInteger)frame {
    namespace sweep=core3d::planar_sweep;
    if (![NSThread isMainThread] || fixture<0 || fixture>4 || plane<0 || plane>2 || frame<0 || frame>3
        || (metersPerUnit!=0.001 && metersPerUnit!=1)) return @{@"status":@"bridge-rejected"};
    try {
        auto source=sweep::probe::Fixture(int(fixture),metersPerUnit,int(plane),int(frame));
        const auto before=sweep::probe::Snapshot(source);
        sweep::Admission admission;const auto prepared=sweep::Prepare(source,admission);
        if (!prepared) return @{@"status":@"admission",@"admission":@(sweep::probe::Name(admission))};
        // Mutating the original authoring value after Prepare must not alter the
        // immutable worker payload. No native authority is constructed by this probe.
        source.radius*=10;
        std::atomic_bool cancelled{false};sweep::SolidResult result;
        const auto status=sweep::Build(prepared,cancelled,result);
        if (status!=sweep::BuildStatus::Built)
            return @{@"status":@(sweep::probe::Name(status)),@"empty":@(result.solid.IsNull())};
        const double mm=prepared->inspection.millimetersPerUnit;
        NSMutableArray *bounds=[NSMutableArray arrayWithCapacity:6];
        for (double value:result.bounds) [bounds addObject:@(value*mm)];
        NSMutableArray *ids=[NSMutableArray arrayWithObject:@(prepared->definition.pathIdentifier)];
        for (const auto& vertex:prepared->definition.vertices) [ids addObject:@(vertex.identifier)];
        for (const auto& segment:prepared->definition.segments) [ids addObject:@(segment.identifier)];
        const auto after=sweep::probe::Snapshot(prepared->definition);
        NSMutableDictionary *record=[@{@"status":@"built",@"boundsMM":[bounds copy],
            @"volumeMM3":@(result.volume*mm*mm*mm),@"lengthMM":@(result.length*mm),@"ids":[ids copy],
            @"frozenBefore":[NSData dataWithBytes:before.data() length:before.size()*sizeof(std::uint64_t)],
            @"frozenAfter":[NSData dataWithBytes:after.data() length:after.size()*sizeof(std::uint64_t)],
            @"authoringValueChanged":@(sweep::probe::Snapshot(source)!=before)} mutableCopy];
        // Independent interior/exterior points are expressed in physical local
        // coordinates. Compare against the real solid, including a saved-style frame.
        struct Sample { double u,off,v;bool inside; };
        std::vector<Sample> samples;
        if (fixture==0 || fixture==2) samples={{0,0,150,true},{7,0,150,false},
            {80-80/std::sqrt(2.0),0,300+80/std::sqrt(2.0),true},
            {80-80/std::sqrt(2.0),7,300+80/std::sqrt(2.0),false}};
        else if (fixture==1 || fixture==3) {
            const double r=fixture==3 ? 80 : 60;
            samples={{0,0,100,true},{2*r,0,100,true},{r,0,200+r,true},{r,0,209+r,false}};
        } else samples={{50,0,0,true},{50,7,0,false}};
        NSMutableArray *states=[NSMutableArray array],*expectedStates=[NSMutableArray array];
        for (const auto& sample:samples) {
            gp_Pnt p=plane==0 ? gp_Pnt(sample.u/mm,sample.v/mm,sample.off/mm)
                : plane==1 ? gp_Pnt(sample.u/mm,sample.off/mm,sample.v/mm)
                : gp_Pnt(sample.off/mm,sample.u/mm,sample.v/mm);
            if (prepared->definition.constructionFrame) {
                gp_Trsf transform;
                if (!prepared->definition.constructionFrame->Transform(transform)) return @{@"status":@"frame"};
                p.Transform(transform);
            }
            BRepClass3d_SolidClassifier classifier(result.solid,p,Precision::Confusion());
            [states addObject:classifier.State()==TopAbs_IN ? @"inside" : classifier.State()==TopAbs_OUT ? @"outside" : @"boundary-or-unknown"];
            [expectedStates addObject:sample.inside ? @"inside" : @"outside"];
        }
        record[@"pointStates"]=[states copy];record[@"expectedPointStates"]=[expectedStates copy];
        // Two bounded DEBUG selected-shape exports, not the production document
        // exporter. Both fixed fixtures are already in native millimetres.
        if ((fixture==0 || fixture==1) && metersPerUnit==0.001 && plane==1 && frame==0) {
            NSURL *directory=[[NSURL fileURLWithPath:NSTemporaryDirectory() isDirectory:YES]
                URLByAppendingPathComponent:[@"shapeyard-sweep-probe-" stringByAppendingString:NSUUID.UUID.UUIDString] isDirectory:YES];
            if (![[NSFileManager defaultManager] createDirectoryAtURL:directory withIntermediateDirectories:NO attributes:nil error:nil])
                return @{@"status":@"export-directory"};
            @try {
                NSURL *file=[directory URLByAppendingPathComponent:@"detached.stl"];
                BRepMesh_IncrementalMesh mesh(result.solid,0.02,Standard_False,0.05,Standard_False);
                if (!mesh.IsDone() || mesh.GetStatusFlags()!=0) return @{@"status":@"mesh"};
                StlAPI_Writer writer;writer.ASCIIMode()=Standard_False;
                if (!writer.Write(result.solid,file.fileSystemRepresentation)) return @{@"status":@"export"};
                NSDictionary *attributes=[[NSFileManager defaultManager] attributesOfItemAtPath:file.path error:nil];
                unsigned long long size=[attributes[NSFileSize] unsignedLongLongValue];
                if (size<84 || size>8*1024*1024) return @{@"status":@"export-size"};
                NSData *data=[NSData dataWithContentsOfURL:file options:0 error:nil];
                if (!data || data.length!=size) return @{@"status":@"export-read"};
                record[@"stl"]=data;
            } @finally { [[NSFileManager defaultManager] removeItemAtURL:directory error:nil]; }
        }
        return [record copy];
    } catch (...) { return @{@"status":@"exception"}; }
}

+ (NSArray<NSDictionary<NSString *, id> *> *)debugDetachedPlanarSweepRejections {
    namespace sweep=core3d::planar_sweep;
    NSMutableArray *rows=[NSMutableArray array];
    for (const auto& value:sweep::probe::Rejections()) {
        sweep::Inspection inspected;const auto admission=sweep::Inspect(value.definition,inspected);
        sweep::Admission second;const auto prepared=sweep::Prepare(value.definition,second);
        [rows addObject:@{@"name":@(value.name.c_str()),@"actual":@(sweep::probe::Name(admission)),
            @"expected":@(sweep::probe::Name(value.expected)),@"prepared":@(bool(prepared)),
            @"second":@(sweep::probe::Name(second)),@"emptyInspection":@(inspected.length==0 && inspected.expectedVolume==0)}];
    }
    return [rows copy];
}

+ (NSDictionary<NSString *, id> *)debugDetachedPlanarSweepCancellation {
    namespace sweep=core3d::planar_sweep;
    try {
        sweep::Admission admission;const auto prepared=sweep::Prepare(sweep::probe::Fixture(4,0.001),admission);
        std::atomic_bool cancelled{false};sweep::SolidResult output;
        const auto initial=sweep::Build(prepared,cancelled,output);
        const bool initiallyNonempty=!output.solid.IsNull();
        cancelled.store(true);const auto stopped=sweep::Build(prepared,cancelled,output);
        const bool stoppedEmpty=output.solid.IsNull() && output.volume==0 && output.length==0;
        cancelled.store(false);const auto invalid=sweep::Build({},cancelled,output);
        const bool invalidEmpty=output.solid.IsNull() && output.volume==0;
        auto outside=sweep::probe::Fixture(4,0.001);outside.constructionFrame=core3d::profile::ConstructionFrame{};
        outside.constructionFrame->values[0]=1e6; // Valid frame scalars, actual solid exits world bounds.
        const auto outOfBounds=sweep::Prepare(std::move(outside),admission);
        const auto verification=sweep::Build(outOfBounds,cancelled,output);
        const bool outsideEmpty=output.solid.IsNull() && output.volume==0 && output.length==0;
        const auto pair=[](double offset) {
            BRep_Builder builder;TopoDS_Compound compound;builder.MakeCompound(compound);
            builder.Add(compound,BRepPrimAPI_MakeBox(gp_Pnt(0,0,0),10,10,10).Shape());
            builder.Add(compound,BRepPrimAPI_MakeBox(gp_Pnt(offset,offset,offset),10,10,10).Shape());
            return compound;
        };
        // Exercise the production interference component with actual intersecting
        // and disjoint BReps. This isolated check issues no prepared sweep authority.
        const auto overlap=sweep::detail::CheckInterference(pair(5),cancelled);
        const auto separated=sweep::detail::CheckInterference(pair(20),cancelled);
        cancelled.store(true);
        const auto stoppedCheck=sweep::detail::CheckInterference(pair(5),cancelled);
        return @{@"initial":@(sweep::probe::Name(initial)),@"initiallyNonempty":@(initiallyNonempty),
            @"stopped":@(sweep::probe::Name(stopped)),@"stoppedEmpty":@(stoppedEmpty),
            @"invalid":@(sweep::probe::Name(invalid)),@"invalidEmpty":@(invalidEmpty),
            @"outsidePrepared":@(bool(outOfBounds)),@"outside":@(sweep::probe::Name(verification)),
            @"outsideEmpty":@(outsideEmpty),@"interferenceOverlap":@(sweep::probe::Name(overlap)),
            @"interferenceSeparated":@(sweep::probe::Name(separated)),@"interferenceCancelled":@(sweep::probe::Name(stoppedCheck))};
    } catch (...) {return @{@"status":@"exception"};}
}

+ (NSDictionary<NSString *, id> *)debugDetachedRectangularLoft:(NSInteger)fixture
    metersPerUnit:(double)metersPerUnit frame:(NSInteger)frame {
    namespace loft=core3d::rectangular_loft;
    if (![NSThread isMainThread] || fixture<0 || fixture>3 || frame<0 || frame>3
        || (metersPerUnit!=0.001 && metersPerUnit!=1)) return @{@"status":@"bridge-rejected"};
    try {
        auto source=loft::probe::Fixture(int(fixture),metersPerUnit,int(frame));
        const auto before=loft::probe::Snapshot(source);
        loft::Admission admission;const auto prepared=loft::Prepare(source,admission);
        if (!prepared) return @{@"status":@"admission",@"admission":@(loft::probe::Name(admission))};
        source.stations.front().width*=2; // The frozen numeric copy must remain exact.
        std::atomic_bool cancelled{false};loft::SolidResult result;
        const auto status=loft::Build(prepared,cancelled,result);
        if (status!=loft::BuildStatus::Built)
            return @{@"status":@(loft::probe::Name(status)),@"empty":@(result.solid.IsNull())};
        const auto& d=prepared->definition;const double mm=prepared->inspection.millimetersPerUnit;
        NSMutableArray *bounds=[NSMutableArray array],*ids=[NSMutableArray arrayWithObject:@(d.loftIdentifier)];
        for(double x:result.bounds)[bounds addObject:@(x*mm)];
        for(auto id:d.correspondence)[ids addObject:@(id)];
        for(const auto& s:d.stations) {[ids addObject:@(s.identifier)];for(auto id:s.cornerIdentifiers)[ids addObject:@(id)];}
        const auto after=loft::probe::Snapshot(d);
        NSMutableDictionary *record=[@{@"status":@"built",@"boundsMM":[bounds copy],
            @"volumeMM3":@(result.volume*mm*mm*mm),@"ids":[ids copy],@"stationCount":@(d.stations.size()),
            @"frozenBefore":[NSData dataWithBytes:before.data() length:before.size()*sizeof(std::uint64_t)],
            @"frozenAfter":[NSData dataWithBytes:after.data() length:after.size()*sizeof(std::uint64_t)],
            @"authoringValueChanged":@(loft::probe::Snapshot(source)!=before)} mutableCopy];
        // Independent fixture cross-sections at z30/z90: x-right/y-top boundary
        // probes offset by0.1mm. The expected inside/outside sequence is in Swift.
        struct Point {double x,y,z;};std::vector<Point> points;
        if(fixture==0 || fixture==1) {
            const double y=fixture==1 ? 9 : 8;
            points={{15.4,0,30},{15.6,0,30},{3,y-.1,30},{3,y+.1,30},
                {14.9,0,90},{15.1,0,90},{3,y-.1,90},{3,y+.1,90}};
        } else points={{9.9,0,30},{10.1,0,30},{0,5.9,30},{0,6.1,30},
            {9.9,0,90},{10.1,0,90},{0,5.9,90},{0,6.1,90}};
        NSMutableArray *states=[NSMutableArray array];
        for(const auto& sample:points) {
            gp_Pnt point(sample.x/mm,sample.y/mm,sample.z/mm);
            if(d.constructionFrame) {gp_Trsf transform;if(!d.constructionFrame->Transform(transform))return @{@"status":@"frame"};point.Transform(transform);}
            BRepClass3d_SolidClassifier classifier(result.solid,point,Precision::Confusion());
            [states addObject:classifier.State()==TopAbs_IN ? @"inside" : classifier.State()==TopAbs_OUT ? @"outside" : @"boundary-or-unknown"];
        }
        record[@"pointStates"]=[states copy];
        if ((fixture==0 || fixture==1) && metersPerUnit==0.001 && frame==0) {
            NSURL *directory=[[NSURL fileURLWithPath:NSTemporaryDirectory() isDirectory:YES]
                URLByAppendingPathComponent:[@"shapeyard-loft-probe-" stringByAppendingString:NSUUID.UUID.UUIDString] isDirectory:YES];
            if(![[NSFileManager defaultManager]createDirectoryAtURL:directory withIntermediateDirectories:NO attributes:nil error:nil])return @{@"status":@"export-directory"};
            @try {
                NSURL *file=[directory URLByAppendingPathComponent:@"detached.stl"];
                BRepMesh_IncrementalMesh mesh(result.solid,0.02,Standard_False,0.05,Standard_False);
                if(!mesh.IsDone() || mesh.GetStatusFlags()!=0)return @{@"status":@"mesh"};
                StlAPI_Writer writer;writer.ASCIIMode()=Standard_False;
                if(!writer.Write(result.solid,file.fileSystemRepresentation))return @{@"status":@"export"};
                NSDictionary *attributes=[[NSFileManager defaultManager]attributesOfItemAtPath:file.path error:nil];
                const auto size=[attributes[NSFileSize]unsignedLongLongValue];
                if(size<84 || size>8*1024*1024)return @{@"status":@"export-size"};
                NSData *data=[NSData dataWithContentsOfURL:file options:0 error:nil];
                if(!data || data.length!=size)return @{@"status":@"export-read"};record[@"stl"]=data;
            } @finally {[[NSFileManager defaultManager]removeItemAtURL:directory error:nil];}
        }
        return [record copy];
    } catch (...) {return @{@"status":@"exception"};}
}

+ (NSArray<NSDictionary<NSString *, id> *> *)debugDetachedRectangularLoftRejections {
    if(![NSThread isMainThread])return @[];
    namespace loft=core3d::rectangular_loft;NSMutableArray *rows=[NSMutableArray array];
    for(const auto& value:loft::probe::Rejections()) {
        loft::Inspection inspected;const auto actual=loft::Inspect(value.definition,inspected);
        loft::Admission second;const auto prepared=loft::Prepare(value.definition,second);
        [rows addObject:@{@"name":@(value.name.c_str()),@"actual":@(loft::probe::Name(actual)),
            @"expected":@(loft::probe::Name(value.expected)),@"second":@(loft::probe::Name(second)),
            @"prepared":@(bool(prepared)),@"emptyInspection":@(inspected.expectedVolume==0 && inspected.millimetersPerUnit==0)}];
    }
    return [rows copy];
}

+ (NSDictionary<NSString *, id> *)debugDetachedRectangularLoftCancellation {
    if(![NSThread isMainThread])return @{@"status":@"bridge-rejected"};
    namespace loft=core3d::rectangular_loft;
    try {
        loft::Admission admission;const auto prepared=loft::Prepare(loft::probe::Fixture(0,.001),admission);
        std::atomic_bool cancelled{false};loft::SolidResult output;
        const auto initial=loft::Build(prepared,cancelled,output);
        const bool initiallyNonempty=!output.solid.IsNull();const auto realSolid=output.solid;
        cancelled.store(true);const auto stopped=loft::Build(prepared,cancelled,output);
        const bool stoppedEmpty=output.solid.IsNull() && output.volume==0;
        cancelled.store(false);const auto invalid=loft::Build({},cancelled,output);
        const bool invalidEmpty=output.solid.IsNull() && output.volume==0;
        if(!prepared || !initiallyNonempty)return @{@"status":@"initial-failed"};
        auto wrongVolume=prepared->inspection;wrongVolume.expectedVolume*=2;
        output.solid=realSolid;output.volume=47920;
        const auto volume=loft::detail::Verify(realSolid,wrongVolume,cancelled,output);
        const bool volumeEmpty=output.solid.IsNull() && output.volume==0;
        auto wrongBounds=prepared->inspection;wrongBounds.expectedBounds[0]-=1;
        output.solid=realSolid;output.volume=47920;
        const auto bounds=loft::detail::Verify(realSolid,wrongBounds,cancelled,output);
        const bool boundsEmpty=output.solid.IsNull() && output.volume==0;
        cancelled.store(true);output.solid=realSolid;
        const auto lateStop=loft::detail::Verify(realSolid,prepared->inspection,cancelled,output);
        const bool lateStopEmpty=output.solid.IsNull() && output.volume==0;
        cancelled.store(false);const auto recovered=loft::Build(prepared,cancelled,output);
        return @{@"initial":@(loft::probe::Name(initial)),@"initiallyNonempty":@(initiallyNonempty),
            @"stopped":@(loft::probe::Name(stopped)),@"stoppedEmpty":@(stoppedEmpty),
            @"invalid":@(loft::probe::Name(invalid)),@"invalidEmpty":@(invalidEmpty),
            @"wrongVolume":@(loft::probe::Name(volume)),@"wrongVolumeEmpty":@(volumeEmpty),
            @"wrongBounds":@(loft::probe::Name(bounds)),@"wrongBoundsEmpty":@(boundsEmpty),
            @"lateStop":@(loft::probe::Name(lateStop)),@"lateStopEmpty":@(lateStopEmpty),
            @"rebuilt":@(loft::probe::Name(recovered)),@"rebuiltVolume":@(output.volume)};
    } catch (...) {return @{@"status":@"exception"};}
}

- (NSDictionary<NSString *, id> *)debugStoredRectangularLoftForEntityIdentifier:(NSString *)identifier {
    if (![NSThread isMainThread] || !_isSetuped || !GLController.viewer
        || ![identifier isKindOfClass:[NSString class]] || identifier.length==0 || identifier.length>128) return nil;
    try {
        const auto doc=GLController.viewer->getDocument();
        if (doc.IsNull() || doc->Document().IsNull()) return nil;
        TDF_LabelSequence labels;XCAFDoc_DocumentTool::ShapeTool(doc->Document()->Main())->GetFreeShapes(labels);
        if (labels.Length()>50000) return nil;
        for (int i=1;i<=labels.Length();++i) {
            const auto label=labels.Value(i);
            if (doc->EntityIdentifierForLabel(label)!=std::string(identifier.UTF8String?:"")) continue;
            OcctObjectNameState state;
            if (!doc->CaptureObjectNameStateForLabel(label,state) || state.object.loft.label.IsNull()) return nil;
            const auto& record=state.object.loft;
            NSMutableArray<NSNumber *> *bits=[NSMutableArray array],*values=[NSMutableArray array];
            NSMutableArray<NSNumber *> *objectBits=[NSMutableArray array];
            for (double value:state.object.scalars) [objectBits addObject:@(core3d::loft_persistence::Bits(value))];
            NSMutableArray<NSNumber *> *appearanceBits=[NSMutableArray array];
            OcctScalarAppearanceState appearance;
            if (doc->CaptureScalarAppearanceForSavedSweepRebuild(label,appearance)) {
                for (int i=0;i<2;++i) {[appearanceBits addObject:@(appearance.legacyPresent[i])];[appearanceBits addObject:@(appearance.legacyValues[i])];}
                [appearanceBits addObject:@(appearance.localPBR)];
                for (double value:appearance.visualValues) [appearanceBits addObject:@(core3d::loft_persistence::Bits(value))];
            }
            for (double value:record.values) {[bits addObject:@(core3d::loft_persistence::Bits(value))];[values addObject:@(value)];}
            const double mm=record.definition.dimensionMetersPerUnit/0.001;
            GProp_GProps properties;BRepGProp::VolumeProperties(record.boundShape,properties);
            Bnd_Box box;BRepBndLib::Add(record.boundShape,box,Standard_False);
            if(box.IsVoid()||box.IsOpen())return nil;
            double x0,y0,z0,x1,y1,z1;box.Get(x0,y0,z0,x1,y1,z1);
            NSArray *bounds=@[@(x0*mm),@(y0*mm),@(z0*mm),@(x1*mm),@(y1*mm),@(z1*mm)];
            return @{@"featureIdentifier": [NSString stringWithUTF8String:record.identifier.c_str()],
                @"definitionIdentifier": [NSString stringWithUTF8String:state.object.definitionIdentifier.c_str()],
                @"entityIdentifier": identifier,@"bits":bits,@"values":values,
                @"appearanceBits":appearanceBits,@"objectTransformBits":objectBits,@"rawMetersPerUnit":@(record.definition.dimensionMetersPerUnit),
                @"current":@(record.IsCurrent(doc->Document(),label)),@"volumeMM3":@(properties.Mass()*mm*mm*mm),@"boundsMM":bounds};
        }
        return nil;
    } catch (...) {return nil;}
}
- (NSDictionary<NSString *, id> *)debugStoredSweepForEntityIdentifier:(NSString *)identifier {
    if (![NSThread isMainThread] || !_isSetuped || !GLController.viewer
        || ![identifier isKindOfClass:[NSString class]] || identifier.length==0 || identifier.length>128) return nil;
    try {
        const auto doc=GLController.viewer->getDocument();
        if (doc.IsNull() || doc->Document().IsNull()) return nil;
        TDF_LabelSequence labels;XCAFDoc_DocumentTool::ShapeTool(doc->Document()->Main())->GetFreeShapes(labels);
        if (labels.Length()>50000) return nil;
        for (int i=1;i<=labels.Length();++i) {
            const auto label=labels.Value(i);
            if (doc->EntityIdentifierForLabel(label)!=std::string(identifier.UTF8String?:"")) continue;
            OcctObjectNameState state;
            if (!doc->CaptureObjectNameStateForLabel(label,state) || state.object.sweep.label.IsNull()) return nil;
            const auto& record=state.object.sweep;
            NSMutableArray<NSNumber *> *bits=[NSMutableArray array],*values=[NSMutableArray array];
            NSMutableArray<NSNumber *> *objectBits=[NSMutableArray array];
            for (double value:state.object.scalars) [objectBits addObject:@(core3d::sweep_persistence::Bits(value))];
            NSMutableArray<NSNumber *> *appearanceBits=[NSMutableArray array];
            OcctScalarAppearanceState appearance;
            if (doc->CaptureScalarAppearanceForSavedSweepRebuild(label,appearance)) {
                for (int i=0;i<2;++i) {[appearanceBits addObject:@(appearance.legacyPresent[i])];[appearanceBits addObject:@(appearance.legacyValues[i])];}
                [appearanceBits addObject:@(appearance.localPBR)];
                for (double value:appearance.visualValues) [appearanceBits addObject:@(core3d::sweep_persistence::Bits(value))];
            }
            for (double value:record.values) {[bits addObject:@(core3d::sweep_persistence::Bits(value))];[values addObject:@(value)];}
            return @{@"featureIdentifier": [NSString stringWithUTF8String:record.identifier.c_str()],
                @"definitionIdentifier": [NSString stringWithUTF8String:state.object.definitionIdentifier.c_str()],
                @"entityIdentifier": identifier,@"bits":bits,@"values":values,
                @"appearanceBits":appearanceBits,@"objectTransformBits":objectBits,@"rawMetersPerUnit":@(record.definition.dimensionMetersPerUnit),
                @"current":@(record.IsCurrent(doc->Document(),label))};
        }
        return nil;
    } catch (...) {return nil;}
}

// Fixed malformed candidate documents exercise the actual production loader gate.
- (BOOL)debugConfigureSavedSweepRebuildFault:(NSInteger)mode {
    if (mode<0 || mode>15 || ![self debugConfigureOrdinaryGestureFault:mode<=5?mode:0])return NO;
    const auto viewer=GLController.viewer;const auto controller=viewer->debugOrdinaryEditController();
    if (!controller || controller->blocksNormalWork())return NO;
    auto& stamp=controller->debugCommandStamp();
    viewer->debugSetOrdinaryRepairFailures(mode==13?1:0,mode==13?1:0);
    controller->debugSetStageFailureIndex(mode==4?0:mode==14?2:mode==15?3:-1);
    if(mode>=6&&mode<=8)stamp.debugSetNewCommandMode(int(mode-5));
    if(mode==9)stamp.debugSetCommitMode(2);
    if(mode==10||mode==11){stamp.debugSetCommitMode(1);stamp.debugSetAbortMode(mode==10?1:2);}
    if(mode==12)stamp.debugSetPostCommitInspectionFailureCount(1);
    return YES;
}
- (BOOL)debugConfigureSavedSweepPairedWriteFailure {
    if (![self debugConfigureOrdinaryGestureFault:0])return NO;
    const auto controller=GLController.viewer->debugOrdinaryEditController();
    if (!controller || controller->blocksNormalWork())return NO;
    controller->debugSetStageFailureIndex(2);return YES;
}
- (NSDictionary<NSString *,id> *)debugSavedSweepStrictBindingProbe {
    if (!NSThread.isMainThread || !_isSetuped || !GLController.viewer || !GLController.viewer->canBeginCommittedEdit()) return @{};
    try {
        const auto owner=GLController.viewer->getDocument();const auto context=GLController.viewer->AisContext();
        if(owner.IsNull()||context.IsNull())return @{};
        context->InitSelected();if(!context->MoreSelected())return @{};
        const auto selected=Handle(AIS_Shape)::DownCast(context->SelectedInteractive());context->NextSelected();
        if(selected.IsNull()||context->MoreSelected())return @{};
        const auto document=owner->Document();OcctObjectTransformState before;
        const auto label=owner->ShapeLabel(selected);
        if(document.IsNull()||document->HasOpenCommand()||!owner->CaptureObjectTransformStateForLabel(label,before)
            ||before.sweep.label.IsNull())return @{};
        document->NewCommand();
        // SetShape changes the transient subshape map, whose OCCT Restore is
        // deliberately empty. This fixture's direct rollback must refresh that
        // cache from the restored label before asking AddSubShape about a face.
        // It must not retain the injected box's cache or rewrite the saved BRep.
        struct Abort {
            Handle(TDocStd_Document) doc; TDF_Label label;
            void restore() {
                if (!doc.IsNull() && doc->HasOpenCommand()) doc->AbortCommand();
                Handle(XCAFDoc_ShapeMapTool) map;
                if (!label.IsNull() && label.FindAttribute(XCAFDoc_ShapeMapTool::GetID(),map))
                    map->SetShape(XCAFDoc_ShapeTool::GetShape(label));
            }
            ~Abort(){try{restore();}catch(...){}}
        } abort{document,label};
        const auto shapes=XCAFDoc_DocumentTool::ShapeTool(document->Main());
        shapes->SetShape(label,BRepPrimAPI_MakeBox(10,11,12).Solid());
        core3d::sweep_persistence::Record stale;
        const bool readRefused=!core3d::sweep_persistence::Read(document,label,stale);
        const bool stageRefused=!core3d::sweep_persistence::Stage(document,label,before.sweep.definition,before.sweep.identifier);
        abort.restore();OcctObjectTransformState after;
        const bool restored=owner->CaptureObjectTransformStateForLabel(label,after)&&after.IsEqual(before)
            &&core3d::sweep_rebuild::SameRawScalars(after.scalars,before.scalars);
        document->NewCommand();
        TopExp_Explorer face(after.shape,TopAbs_FACE);if(!face.More() || !shapes->IsSubShape(label,face.Current()))return @{};
        const auto faceLabel=shapes->AddSubShape(label,face.Current());if(faceLabel.IsNull())return @{};
        TDataStd_Name::Set(faceLabel,TCollection_ExtendedString("Preserved face style guard"));
        core3d::sweep_persistence::Record stillCurrent;
        const bool faceRefused=core3d::sweep_persistence::Read(document,label,stillCurrent)
            &&!core3d::sweep_rebuild::HasOnlyMetadataSubshapes(document,label);
        abort.restore();OcctObjectTransformState finalState;
        const bool finalRestored=owner->CaptureObjectTransformStateForLabel(label,finalState)&&finalState.IsEqual(before)
            &&core3d::sweep_rebuild::SameRawScalars(finalState.scalars,before.scalars);
        return @{@"readRefused":@(readRefused),@"stageRefused":@(stageRefused),
            @"faceMetadataRefused":@(faceRefused),@"restored":@(restored&&finalRestored)};
    }catch(...){return @{};}
}

- (NSData *)debugSavedRectangularLoftAdmissionFixture:(NSInteger)fault {
    if (![NSThread isMainThread] || fault<0 || fault>12) return nil;
    return Core3DCreateDebugBinXCAFFixture(@"loft-admission",[fault](const Handle(TDocStd_Document)& document) {
        namespace p=core3d::loft_persistence;
        const auto shapes=XCAFDoc_DocumentTool::ShapeTool(document->Main());
        const auto shape=BRepPrimAPI_MakeBox(10,10,10).Shape();
        const auto owner=shapes->AddShape(shape,Standard_False,Standard_True);
        Core3DSetDebugGeometryRepresentation(owner,1);
        auto definition=core3d::rectangular_loft::probe::Fixture(0,0.001,0);
        const std::string id=NSUUID.UUID.UUIDString.UTF8String;
        document->SetUndoLimit(10);document->NewCommand();
        if (!p::Stage(document,owner,definition,id) || !document->CommitCommand())
            throw Standard_Failure("Sweep admission fixture staging failed");
        p::Record record;if (!p::Read(document,owner,record)) throw Standard_Failure("Sweep fixture read failed");
        if (fault==0) TDataStd_Integer::Set(record.label,p::SchemaID(),99);
        if (fault==1) record.label.FindChild(4).ForgetAllAttributes();
        if (fault==2) TDataStd_Integer::Set(document->GetData()->Root(),p::CountID(),50);
        if (fault==3) XCAFDoc_DocumentTool::SetLengthUnit(document,1.0);
        if(fault==6)shapes->SetShape(owner,BRepPrimAPI_MakeBox(12,10,10).Shape());
        if(fault==7)TDataStd_Integer::Set(owner.FindChild(99,Standard_True).FindChild(1,Standard_True),p::SchemaID(),1);
        if(fault==8)TDataStd_Integer::Set(record.label,core3d::sweep_persistence::SchemaID(),1);
        if(fault==9)TDataStd_AsciiString::Set(record.label,p::IdentityID(),TCollection_AsciiString("invalid-uuid"));
        if(fault==10)TDataStd_Real::Set(record.label.FindChild(51,Standard_True),1.0);
        if (fault==4 || fault==5 || fault==11 || fault==12) {
            const auto second=shapes->AddShape(BRepPrimAPI_MakeBox(11,10,10).Shape(),Standard_False,Standard_True);
            Core3DSetDebugGeometryRepresentation(second,1);
            document->NewCommand();
            if (fault==4) {
                if (!p::Stage(document,second,definition,id)) throw Standard_Failure("Duplicate sweep fixture failed");
            } else if(fault==11) {
                core3d::enclosure::Parameters enclosure;enclosure.metersPerUnit=.001;
                if(!core3d::enclosure::Stage(document,second,enclosure,id))throw Standard_Failure("Loft/enclosure duplicate fixture failed");
            } else if(fault==12) {
                const auto sweep=core3d::planar_sweep::probe::Fixture(4,.001);
                if(!core3d::sweep_persistence::Stage(document,second,sweep,id))throw Standard_Failure("Loft/sweep duplicate fixture failed");
            } else {
                core3d::profile::Parameters profile;profile.metersPerUnit=0.001;
                profile.definition.points={{0,0},{10,0},{10,10},{0,10}};profile.definition.depth=10;
                if (!core3d::profile::Stage(document,second,profile,id)) throw Standard_Failure("Cross-family identity fixture failed");
            }
            if (!document->CommitCommand()) throw Standard_Failure("Duplicate identity fixture commit failed");
        }
    });
}
- (NSData *)debugSavedSweepAdmissionFixture:(NSInteger)fault {
    if (![NSThread isMainThread] || fault<0 || fault>5) return nil;
    return Core3DCreateDebugBinXCAFFixture(@"sweep-admission",[fault](const Handle(TDocStd_Document)& document) {
        namespace p=core3d::sweep_persistence;
        const auto shapes=XCAFDoc_DocumentTool::ShapeTool(document->Main());
        const auto shape=BRepPrimAPI_MakeBox(10,10,10).Shape();
        const auto owner=shapes->AddShape(shape,Standard_False,Standard_True);
        Core3DSetDebugGeometryRepresentation(owner,1);
        auto definition=core3d::planar_sweep::probe::Fixture(4,0.001);
        const std::string id=NSUUID.UUID.UUIDString.UTF8String;
        document->SetUndoLimit(10);document->NewCommand();
        if (!p::Stage(document,owner,definition,id) || !document->CommitCommand())
            throw Standard_Failure("Sweep admission fixture staging failed");
        p::Record record;if (!p::Read(document,owner,record)) throw Standard_Failure("Sweep fixture read failed");
        if (fault==0) TDataStd_Integer::Set(record.label,p::SchemaID(),99);
        if (fault==1) record.label.FindChild(4).ForgetAllAttributes();
        if (fault==2) TDataStd_Integer::Set(document->GetData()->Root(),p::CountID(),24);
        if (fault==3) XCAFDoc_DocumentTool::SetLengthUnit(document,1.0);
        if (fault==4 || fault==5) {
            const auto second=shapes->AddShape(BRepPrimAPI_MakeBox(11,10,10).Shape(),Standard_False,Standard_True);
            Core3DSetDebugGeometryRepresentation(second,1);
            document->NewCommand();
            if (fault==4) {
                if (!p::Stage(document,second,definition,id)) throw Standard_Failure("Duplicate sweep fixture failed");
            } else {
                core3d::profile::Parameters profile;profile.metersPerUnit=0.001;
                profile.definition.points={{0,0},{10,0},{10,10},{0,10}};profile.definition.depth=10;
                if (!core3d::profile::Stage(document,second,profile,id)) throw Standard_Failure("Cross-family identity fixture failed");
            }
            if (!document->CommitCommand()) throw Standard_Failure("Duplicate identity fixture commit failed");
        }
    });
}

+ (NSDictionary<NSString *,id> *)debugRectangularLoftFamilyBudget {
    if(!NSThread.isMainThread)return @{};
    try {
        namespace p=core3d::profile;namespace e=core3d::enclosure;namespace sw=core3d::sweep_persistence;namespace lf=core3d::loft_persistence;
        sw::probe::RawDocument raw;const auto doc=raw.document;
        const auto shapes=XCAFDoc_DocumentTool::ShapeTool(doc->Main());const auto box=XCAFDoc_ShapeTool::GetShape(raw.owner);
        std::vector<p::Record> profiles;std::vector<e::Record> enclosures;std::vector<sw::Record> sweeps;std::vector<lf::Record> lofts;
        const auto validate=[&]{return core3d::saved_features::Validate(doc,profiles,enclosures,sweeps,lofts);};
        const auto add=[&](int kind,int ordinal){
            const auto owner=ordinal==0?raw.owner:shapes->NewShape();if(ordinal!=0)shapes->SetShape(owner,box);
            const std::string identity=NSUUID.UUID.UUIDString.UTF8String;
            if(kind==3)return lf::Stage(doc,owner,core3d::rectangular_loft::probe::Fixture(0,0.001,0),identity);
            if(kind==2)return sw::Stage(doc,owner,core3d::planar_sweep::probe::Fixture(4,0.001),identity);
            if(kind==1){e::Parameters d;d.metersPerUnit=.001;d.definition.dimensions={100,80,40,3,3,5};return e::Stage(doc,owner,d,identity);}
            p::Parameters d;d.metersPerUnit=.001;d.definition.points={{0,0},{10,0},{10,10},{0,10}};d.definition.depth=10;
            return p::Stage(doc,owner,d,identity);
        };
        doc->NewCommand();for(int i=0;i<4096;++i)if(!add(i<4?i:0,i))return @{@"fixture":@NO};
        const bool atLimit=validate()&&profiles.size()==4093&&enclosures.size()==1&&sweeps.size()==1&&lofts.size()==1;
        if(!add(0,4096))return @{@"fixture":@NO};
        const bool overLimit=!validate()&&profiles.empty()&&enclosures.empty()&&sweeps.empty()&&lofts.empty();
        doc->AbortCommand();
        // Independent fresh document: test the actual shared root traversal boundary.
        sw::probe::RawDocument labels;const auto root=labels.document->GetData()->Root();
        int count=0,maxTag=0;for(TDF_ChildIterator it(root,Standard_True);it.More();it.Next())++count;
        for(TDF_ChildIterator it(root,Standard_False);it.More();it.Next())maxTag=std::max(maxTag,it.Value().Tag());
        if(count>=100000||maxTag>=std::numeric_limits<int>::max()-100001)return @{@"fixture":@NO};
        while(count<100000){root.FindChild(++maxTag,Standard_True);++count;}
        const bool labelLimit=core3d::saved_features::Validate(labels.document,profiles,enclosures,sweeps,lofts);
        root.FindChild(++maxTag,Standard_True);
        const bool labelOver=!core3d::saved_features::Validate(labels.document,profiles,enclosures,sweeps,lofts)
            &&profiles.empty()&&enclosures.empty()&&sweeps.empty()&&lofts.empty();
        return @{@"fixture":@YES,@"records4096":@(atLimit),@"records4097Refused":@(overLimit),
            @"labels100000":@(labelLimit),@"labels100001Refused":@(labelOver)};
    }catch(...){return @{@"fixture":@NO};}
}

+ (NSDictionary<NSString *,id> *)debugRectangularLoftCodecValues:(NSArray<NSNumber *> *)input {
    if(![input isKindOfClass:[NSArray class]]||input.count>129)return @{@"accepted":@NO};
    try {std::vector<double> values;for(id v in input){if(![v isKindOfClass:[NSNumber class]])return @{@"accepted":@NO};values.push_back([v doubleValue]);}
        core3d::rectangular_loft::Definition d;const bool accepted=core3d::loft_persistence::Decode(values,d);
        std::vector<double> encoded;NSMutableArray *bits=[NSMutableArray array];
        if(accepted&&core3d::loft_persistence::Encode(d,encoded))for(double v:encoded)[bits addObject:@(core3d::loft_persistence::Bits(v))];
        return @{@"accepted":@(accepted),@"bits":bits,@"stationCount":@(d.stations.size()),@"id":@(d.loftIdentifier)};
    }catch(...){return @{@"accepted":@NO};}
}

+ (NSDictionary<NSString *, id> *)debugSweepPersistenceScenario:(NSInteger)scenario {
    if (![NSThread isMainThread] || scenario<0 || scenario>3)
        return @{@"error": @"Unavailable sweep codec fixture"};
    try {
        using namespace core3d::sweep_persistence::probe;
        Checks checks;
        switch (scenario) {
            case 0: checks=Numeric(); break;
            case 1: checks=RoundTrip(); break;
            case 2: checks=Malformed(); break;
            case 3: checks=BindingAndTransaction(); break;
        }
        NSMutableDictionary<NSString *, NSNumber *> *result=[NSMutableDictionary dictionary];
        for (const auto& check:checks) result[[NSString stringWithUTF8String:check.first.c_str()]]=@(check.second);
        return @{@"checks": result};
    } catch (const Standard_Failure& failure) {
        return @{@"error": [NSString stringWithUTF8String:failure.GetMessageString()] ?: @"OCCT codec failure"};
    } catch (...) { return @{@"error": @"Native sweep codec fixture failed"}; }
}

+ (NSDictionary<NSString *, id> *)debugCurveProfileCodecValues:(NSArray<NSNumber *> *)input {
    if (![input isKindOfClass:[NSArray class]] || input.count > 6211)
        return @{@"accepted": @NO, @"bridgeRejected": @YES};
    try {
        std::vector<double> values; values.reserve(input.count);
        for (id value in input) {
            if (![value isKindOfClass:[NSNumber class]])
                return @{@"accepted": @NO, @"bridgeRejected": @YES};
            // Preserve NaN/Inf so the native decoder, not this diagnostic,
            // demonstrates rejection of non-finite serialized scalars.
            values.push_back([value doubleValue]);
        }
        core3d::profile::Parameters parameters;
        const bool decoded = core3d::profile::Decode(values,parameters);
        if (!decoded) return @{@"accepted": @NO, @"bridgeRejected": @NO};
        std::vector<double> encoded;
        if (!core3d::profile::Encode(parameters,encoded))
            return @{@"accepted": @YES, @"encodeFailed": @YES};
        NSMutableArray<NSNumber *> *output = [NSMutableArray arrayWithCapacity:encoded.size()];
        for (double value : encoded) [output addObject:@(value)];
        double area = 0, volume = 0;
        const bool measured = core3d::ProfileDefinitionExpectedVolume(parameters.definition,area,volume);
        return @{@"accepted": @YES, @"bridgeRejected": @NO,
            @"schema": @(core3d::profile::SchemaFor(parameters)), @"values": [output copy],
            @"hasCurves": @(parameters.definition.curves.has_value()),
            @"hasFrame": @(parameters.constructionFrame.has_value()),
            @"measured": @(measured), @"area": @(area), @"volume": @(volume)};
    } catch (...) {
        return @{@"accepted": @NO, @"exception": @YES};
    }
}


+ (NSDictionary<NSString *, NSNumber *> *)debugNativeMutationLifecycle {
    if (![NSThread isMainThread]) return @{@"mainThread":@NO};
    NSMutableDictionary<NSString *, NSNumber *> *result = [NSMutableDictionary dictionary];
    try {
        Handle(OcctDocument) document = new OcctDocument();
        document->InitDoc();
        const auto initial = document->DebugNativeMutationStamp();
        result[@"initialized"] = @(initial.has_value());
        if (!initial) return result;
        auto native = document->ChangeDocument();
        native->NewCommand();
        result[@"openCommandRefused"] = @(!document->DebugNativeMutationStamp().has_value());
        native->AbortCommand();
        auto current = document->DebugNativeMutationStamp();
        result[@"closedAbortAdvanced"] = @(current && current->edit > initial->edit
            && current->opening == initial->opening && native->GetAvailableUndos() == 0);
        if (!current) return result;
        const auto aborted = *current;
        native->NewCommand();
        (void)native->CommitCommand();
        current = document->DebugNativeMutationStamp();
        result[@"emptyCommitAdvancedWithoutHistory"] = @(current && current->edit > aborted.edit
            && native->GetAvailableUndos() == 0);
        if (!current) return result;
        const auto beforeHistory = *current;
        const bool unavailableUndo = !document->undo();
        const bool unavailableRedo = !document->redo();
        current = document->DebugNativeMutationStamp();
        result[@"unavailableHistoryAttemptsAdvanced"] = @(unavailableUndo && unavailableRedo
            && current && current->edit > beforeHistory.edit && native->GetAvailableUndos() == 0);
        if (!current) return result;
        const auto beforeReset = *current;
        const auto identifier = document->DocumentIdentifier();
        document->InitDoc();
        current = document->DebugNativeMutationStamp();
        result[@"resetCreatesNewOpening"] = @(current && current->opening > beforeReset.opening
            && current->edit > beforeReset.edit && current->selection > beforeReset.selection
            && current->instanceNonce == beforeReset.instanceNonce
            && document->DocumentIdentifier() != identifier);
        // Exhaust the separate bounded DEBUG event log using actual native
        // callbacks, then prove production observation still advances.
        bool diagnosticIndependent = document->DebugStartLiveTransactionProbe();
        if (diagnosticIndependent) {
            native = document->ChangeDocument();
            for (unsigned attempt = 0; attempt < 140; ++attempt) {
                native->NewCommand();
                native->AbortCommand();
            }
            const auto afterLogOverflow = document->DebugNativeMutationStamp();
            diagnosticIndependent = !document->DebugLiveTransactionProbeValid()
                && afterLogOverflow.has_value();
            native->NewCommand();
            native->AbortCommand();
            const auto afterMoreWork = document->DebugNativeMutationStamp();
            diagnosticIndependent = diagnosticIndependent && afterMoreWork
                && afterMoreWork->edit > afterLogOverflow->edit
                && afterMoreWork->opening == afterLogOverflow->opening
                && native->GetAvailableUndos() == 0;
            document->DebugStopLiveTransactionProbe();
        }
        result[@"diagnosticOverflowDoesNotLimitProductionObservation"] = @(diagnosticIndependent);
        // Keep the application strongly owned on this thread. The foreign
        // notification passes an empty handle; adapter guards must reject it
        // before dereferencing any native document or accessing the weak state.
        const auto application = Handle(TDocStd_Application)::DownCast(
            document->Document()->Application());
        if (application.IsNull()) {
            result[@"foreignNotificationPoisonsActualAdapter"] = @NO;
            return result;
        }
        auto* applicationPointer = application.get();
        std::thread foreignCallback([applicationPointer] {
            const Handle(TDocStd_Document) empty;
            applicationPointer->OnOpenTransaction(empty);
        });
        foreignCallback.join();
        result[@"foreignNotificationPoisonsActualAdapter"] = @(!document->DebugNativeMutationStamp().has_value());
        // Separate pure-policy tests. Identity sentinels are never dereferenced
        // as OCAF documents and do not qualify live selection coverage.
        std::array<std::uint8_t, 16> nonce{}; nonce[0] = 1;
        for (unsigned counter = 0; counter < 3; ++counter) {
            core3d::authority::NativeEditAuthority policy(nonce);
            int first = 0, second = 0;
            bool checked = policy.Adopt(&first) && policy.DebugExhaustCounter(counter);
            if (counter == 0) (void)policy.Adopt(&second);
            else if (counter == 1) policy.TransactionBoundary(&first);
            else policy.SelectionBoundary(&first);
            checked = checked && !policy.Valid() && !policy.Capture(&first, true, false)
                && !policy.Adopt(&second);
            result[[NSString stringWithFormat:@"counter%uSaturatesWithoutWrap", counter]] = @(checked);
        }
        core3d::authority::NativeEditAuthority policy(nonce);
        int identity = 0;
        (void)policy.Adopt(&identity);
        std::thread foreignPolicy([&policy, &identity] { policy.SelectionBoundary(&identity); });
        foreignPolicy.join();
        result[@"foreignPolicyAccessPoisons"] = @(!policy.Valid() && !policy.Capture(&identity, true, false));
    } catch (...) {
        result[@"exception"] = @YES;
    }
    return result;
}

- (NSDictionary<NSString *, id> *)debugNativeMutationStamp {
    if (![NSThread isMainThread] || GLController == nil || GLController.viewer == nullptr) return nil;
    const auto document = GLController.viewer->getDocument();
    if (document.IsNull()) return nil;
    const auto stamp = document->DebugNativeMutationStamp();
    if (!stamp) return nil;
    const auto nonce = [[NSData dataWithBytes:stamp->instanceNonce.data()
                                      length:stamp->instanceNonce.size()] base64EncodedStringWithOptions:0];
    NSString *identifier = [NSString stringWithUTF8String:document->DocumentIdentifier().c_str()];
    if (identifier.length == 0) return nil;
    return @{@"instanceNonce":nonce, @"documentIdentifier":identifier,
             @"opening":@(stamp->opening), @"edit":@(stamp->edit),
             @"selection":@(stamp->selection)};
}

// Standalone persistence fixture, not an ordinary creation API.
- (NSData *_Nullable)debugEnclosureBinXCAFFixturePlane:(NSInteger)plane width:(double)width fault:(NSInteger)fault {
    if (![NSThread isMainThread] || plane < 0 || plane > 2
        || (width != 100 && width != 120) || fault < 0 || fault > 1) return nil;
    return Core3DCreateDebugBinXCAFFixture(@"enclosure-dependency", [plane,width,fault](const Handle(TDocStd_Document)& document) {
        core3d::enclosure::Parameters parameters;
        parameters.definition.plane = int(plane);
        parameters.definition.dimensions = {width,60,30,2,2,4};
        parameters.metersPerUnit = 0.001;
        core3d::EnclosureSolidResult geometry;
        if (!core3d::BuildEnclosureSolidGeometry(parameters.definition,
                std::make_shared<std::atomic_bool>(false), geometry))
            throw Standard_Failure("Unable to build enclosure fixture");
        const auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        const auto label = shapeTool->AddShape(geometry.solid, Standard_False, Standard_True);
        if (label.IsNull()) throw Standard_Failure("Unable to add enclosure fixture");
        Core3DSetDebugGeometryRepresentation(label, 1);
        document->SetUndoLimit(10); document->NewCommand();
        if (!core3d::enclosure::Stage(document,label,parameters,NSUUID.UUID.UUIDString.UTF8String)
            || !document->CommitCommand()) throw Standard_Failure("Unable to stage enclosure fixture");
        core3d::enclosure::Record record;
        if (!core3d::enclosure::Read(document,label,record) || !record.IsCurrent(document,label))
            throw Standard_Failure("Enclosure fixture binding mismatch");
        if (fault == 1) TDataStd_Integer::Set(record.label,core3d::enclosure::SchemaID(),99);
    });
}

+ (Core3DEnclosureDefinition *)debugEnclosureDefinitionValues:(NSArray<NSNumber *> *)input {
    if (![input isKindOfClass:[NSArray class]] || (input.count != 9 && input.count != 17)) return nil;
    try {
        std::vector<double> values; values.reserve(input.count);
        for (id value in input) {
            if (![value isKindOfClass:[NSNumber class]]) return nil;
            values.push_back([value doubleValue]);
        }
        if (values.front() != 1 && values.front() != 2) return nil;
        core3d::enclosure::Parameters parameters;
        if (!core3d::enclosure::Decode(int(values.front()),values,parameters)) return nil;
        return [[Core3DEnclosureDefinition alloc] initWithNativeParameters:parameters];
    } catch (...) { return nil; }
}

+ (NSArray<NSNumber *> *)debugEncodedEnclosureDefinition:(Core3DEnclosureDefinition *)definition {
    if (![definition isKindOfClass:[Core3DEnclosureDefinition class]]) return nil;
    try {
        std::vector<double> values;
        if (!core3d::enclosure::Encode([definition nativeParameters],values)) return nil;
        NSMutableArray<NSNumber *> *result = [NSMutableArray arrayWithCapacity:values.size()];
        for (double value : values) [result addObject:@(value)];
        return [result copy];
    } catch (...) { return nil; }
}

+ (NSData *)debugEnclosureFrameFixtureValues:(NSArray<NSNumber *> *)input state:(NSInteger)state {
    if (![NSThread isMainThread] || state < 0 || state > 2) return nil;
    Core3DEnclosureDefinition *definition = [self debugEnclosureDefinitionValues:input];
    if (definition == nil) return nil;
    const auto parameters = [definition nativeParameters];
    if (!parameters.definition.constructionFrame) return nil;
    return Core3DCreateDebugBinXCAFFixture(@"enclosure-frame", [parameters,state](const Handle(TDocStd_Document)& document) {
        XCAFDoc_DocumentTool::SetLengthUnit(document,parameters.metersPerUnit);
        core3d::EnclosureSolidResult geometry;
        if (!core3d::BuildEnclosureSolidGeometry(parameters.definition,
                std::make_shared<std::atomic_bool>(false),geometry))
            throw Standard_Failure("Unable to build framed enclosure fixture");
        const auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        const auto label = shapeTool->AddShape(geometry.solid,Standard_False,Standard_True);
        if (label.IsNull()) throw Standard_Failure("Unable to add framed enclosure fixture");
        Core3DSetDebugGeometryRepresentation(label,1);
        TDataStd_Name::Set(label,TCollection_ExtendedString("Framed enclosure"));
        document->SetUndoLimit(10);document->NewCommand();
        if (!core3d::enclosure::Stage(document,label,parameters,NSUUID.UUID.UUIDString.UTF8String)
            || !document->CommitCommand()) throw Standard_Failure("Unable to stage framed enclosure fixture");
        core3d::enclosure::Record record;
        if (!core3d::enclosure::Read(document,label,record) || !record.IsCurrent(document,label))
            throw Standard_Failure("Framed enclosure fixture binding mismatch");
        if (state == 1) {
            XCAFDoc_DocumentTool::SetLengthUnit(document,parameters.metersPerUnit == 0.001 ? 1.0 : 0.001);
        } else if (state == 2) {
            gp_Trsf moved; moved.SetTranslation(gp_Vec(7,0,0));
            BRepBuilderAPI_Transform transformed(geometry.solid,moved,Standard_True,Standard_False);
            if (!transformed.IsDone() || transformed.Shape().IsNull())
                throw Standard_Failure("Unable to make stale framed enclosure root");
            shapeTool->SetShape(label,transformed.Shape());
        }
        core3d::enclosure::Record stored;
        if (!core3d::enclosure::Read(document,label,stored)
            || stored.IsCurrent(document,label) != (state == 0))
            throw Standard_Failure("Framed enclosure fixture current/stale state mismatch");
    });
}

- (NSDictionary<NSString *, id> *)debugStoredEnclosureForEntityIdentifier:(NSString *)identifier {
    if (![NSThread isMainThread] || identifier.length == 0 || identifier.length > 256
        || GLController == nil || GLController.viewer == nullptr) return nil;
    try {
        const auto owner = GLController.viewer->getDocument();
        if (owner.IsNull() || owner->Document().IsNull()) return nil;
        const auto document = owner->Document();
        TDF_LabelSequence labels;
        XCAFDoc_DocumentTool::ShapeTool(document->Main())->GetFreeShapes(labels);
        for (int i = 1; i <= labels.Length(); ++i) {
            const auto label = labels.Value(i);
            if (owner->EntityIdentifierForLabel(label) != identifier.UTF8String) continue;
            core3d::enclosure::Record record;
            if (!core3d::enclosure::Read(document,label,record)) return nil;
            if (record.label.IsNull()) return @{@"present":@NO};
            NSMutableArray<NSNumber *> *values = [NSMutableArray arrayWithCapacity:record.values.size()];
            for (double value : record.values) [values addObject:@(value)];
            GProp_GProps properties; BRepGProp::VolumeProperties(record.boundShape,properties);
            OcctObjectTransformState state;
            const bool captured = owner->CaptureObjectTransformStateForLabel(label,state);
            return @{@"present":@YES,@"current":@(record.IsCurrent(document,label)),
                @"identifier":[NSString stringWithUTF8String:record.identifier.c_str()],@"values":values,
                @"nativeVolume":@(properties.Mass()),@"authorityCaptured":@(captured),
                @"copyAdmitted":@(owner->CanDuplicateGeometryDefinitions({{label,1U}}))};
        }
    } catch (...) {}
    return nil;
}

// DEBUG fixture helper: a real closed OCAF metadata command with deliberately
// unchanged viewer revision. Numeric placement and geometry remain identical.
- (BOOL)debugAuthorEnclosureTransformDefaultsWithoutRevision:(NSString *)identifier {
    if (![NSThread isMainThread] || identifier.length == 0 || identifier.length > 256
        || GLController == nil || GLController.viewer == nullptr) return NO;
    Handle(TDocStd_Document) document; bool opened = false;
    try {
        const auto owner = GLController.viewer->getDocument();
        if (owner.IsNull()) return NO;
        document = owner->Document();
        if (document.IsNull() || document->HasOpenCommand()) return NO;
        TDF_LabelSequence labels;
        XCAFDoc_DocumentTool::ShapeTool(document->Main())->GetFreeShapes(labels);
        TDF_Label target;
        for (int i = 1; i <= labels.Length(); ++i)
            if (owner->EntityIdentifierForLabel(labels.Value(i)) == identifier.UTF8String)
                target = labels.Value(i);
        OcctObjectTransformState before;
        if (target.IsNull() || !owner->CaptureObjectTransformStateForLabel(target,before)
            || before.enclosure.label.IsNull() || !before.enclosure.IsCurrent(document,target)) return NO;
        if (std::all_of(before.present.begin(),before.present.end(),
                [](Standard_Boolean value) { return value; })) return NO;
        const auto undo = document->GetAvailableUndos();
        Handle(AIS_Shape) temporary = new AIS_Shape(before.shape);
        temporary->SetLocalTransformation(before.transform);
        document->NewCommand(); opened = true;
        if (!owner->SaveObjectTransform(target,temporary))
            throw Standard_Failure("Unable to stage explicit transform defaults");
        OcctObjectTransformState staged;
        if (!owner->CaptureObjectTransformStateForLabel(target,staged)
            || staged.scalars != before.scalars || staged.present == before.present
            || !staged.shape.IsEqual(before.shape) || !staged.enclosure.IsCurrent(document,target))
            throw Standard_Failure("Explicit transform defaults changed numeric geometry");
        const bool committed = document->CommitCommand();
        opened = document->HasOpenCommand();
        if (opened) { document->AbortCommand(); opened = false; return NO; }
        if (!committed) return NO;
        OcctObjectTransformState after;
        return document->GetAvailableUndos() == undo+1
            && owner->CaptureObjectTransformStateForLabel(target,after)
            && staged.IsEqual(after) && owner->ValidateGeometryRepresentations();
    } catch (...) {
        if (opened && !document.IsNull()) { try { document->AbortCommand(); } catch (...) {} }
        return NO;
    }
}

// Faults are confined to one owned synchronous DEBUG command, always aborted.
// Mode12 changes valid metadata to prove exact operation authority notices it.
- (BOOL)debugStoredEnclosureRejectsFault:(NSInteger)mode entityIdentifier:(NSString *)identifier {
    if (![NSThread isMainThread] || mode < 1 || mode > 12 || identifier.length == 0
        || GLController == nil || GLController.viewer == nullptr) return NO;
    Handle(TDocStd_Document) document; bool opened = false;
    try {
        const auto owner = GLController.viewer->getDocument();
        if (owner.IsNull()) return NO;
        document = owner->Document();
        if (document.IsNull() || document->HasOpenCommand()) return NO;
        TDF_LabelSequence labels;
        XCAFDoc_DocumentTool::ShapeTool(document->Main())->GetFreeShapes(labels);
        TDF_Label target;
        for (int i = 1; i <= labels.Length(); ++i)
            if (owner->EntityIdentifierForLabel(labels.Value(i)) == identifier.UTF8String) target = labels.Value(i);
        core3d::enclosure::Record before;
        OcctObjectTransformState original;
        if (target.IsNull() || !core3d::enclosure::Read(document,target,before) || before.label.IsNull()
            || !owner->CaptureObjectTransformStateForLabel(target,original)) return NO;
        const auto undo = document->GetAvailableUndos(), redo = document->GetAvailableRedos();
        document->NewCommand(); opened = true;
        switch (mode) {
            case 1: TDataStd_Integer::Set(before.label,core3d::enclosure::SchemaID(),99); break;
            case 2: TDataStd_Integer::Set(before.label,core3d::enclosure::CountID(),10); break;
            case 3: before.label.ForgetAttribute(TNaming_NamedShape::GetID()); break;
            case 4: TDataStd_Real::Set(before.label.FindChild(3,Standard_False),std::numeric_limits<double>::quiet_NaN()); break;
            case 5: TDataStd_AsciiString::Set(before.label,core3d::enclosure::IdentityID(),TCollection_AsciiString("invalid")); break;
            case 6: TDataStd_Real::Set(before.label,1); break;
            case 7: TDataStd_Integer::Set(document->Main(),core3d::enclosure::SchemaID(),1); break;
            case 8: TDataStd_Real::Set(before.label.FindChild(3,Standard_False),0); break;
            case 9: TDataStd_Real::Set(before.label.FindChild(3,Standard_False).FindChild(1,Standard_True),1); break;
            case 10: TDataStd_Integer::Set(before.label.FindChild(3,Standard_False),1); break;
            case 11: TDataStd_Integer::Set(target.FindChild(900,Standard_True),core3d::profile::SchemaID(),1); break;
            case 12: TDataStd_Real::Set(before.label.FindChild(3,Standard_False),before.parameters.definition.dimensions.width+20); break;
        }
        OcctObjectTransformState changed;
        const bool checked = mode == 12
            ? owner->CaptureObjectTransformStateForLabel(target,changed) && !original.IsEqual(changed)
                && owner->ValidateGeometryRepresentations()
            : !owner->ValidateGeometryRepresentations();
        document->AbortCommand(); opened = false;
        OcctObjectTransformState restored;
        return checked && owner->ValidateGeometryRepresentations()
            && document->GetAvailableUndos() == undo && document->GetAvailableRedos() == redo
            && owner->CaptureObjectTransformStateForLabel(target,restored) && original.IsEqual(restored);
    } catch (...) {
        if (opened && !document.IsNull()) { try { document->AbortCommand(); } catch (...) {} }
        return NO;
    }
}

- (NSDictionary<NSString *, id> *)debugStoredProfileDefinitionForEntityIdentifier:(NSString *)identifier {
    if (![NSThread isMainThread] || identifier.length == 0 || identifier.length > 256
        || GLController == nil || GLController.viewer == nullptr) return nil;
    try {
        const auto owner = GLController.viewer->getDocument();
        if (owner.IsNull() || owner->Document().IsNull()) return nil;
        const auto document = owner->Document();
        TDF_LabelSequence labels;
        XCAFDoc_DocumentTool::ShapeTool(document->Main())->GetFreeShapes(labels);
        for (int i = 1; i <= labels.Length(); ++i) {
            const auto label = labels.Value(i);
            if (owner->EntityIdentifierForLabel(label) != identifier.UTF8String) continue;
            core3d::profile::Record record;
            if (!core3d::profile::Read(document, label, record)) return nil;
            if (record.label.IsNull()) return @{@"present":@NO};
            NSMutableArray<NSNumber *> *values = [NSMutableArray arrayWithCapacity:record.values.size()];
            for (double value : record.values) [values addObject:@(value)];
            return @{@"present":@YES, @"current":@(record.IsCurrent(document, label)),
                @"identifier":[NSString stringWithUTF8String:record.identifier.c_str()],
                @"values":values, @"metersPerUnit":@(record.parameters.metersPerUnit)};
        }
    } catch (...) {}
    return nil;
}

- (BOOL)debugStoredProfileRejectsFault:(NSInteger)mode entityIdentifier:(NSString *)identifier {
    if (![NSThread isMainThread] || mode < 1 || mode > 19 || identifier.length == 0
        || GLController == nil || GLController.viewer == nullptr) return NO;
    Handle(TDocStd_Document) document;
    bool opened = false;
    try {
        const auto owner = GLController.viewer->getDocument();
        if (owner.IsNull()) return NO;
        document = owner->Document();
        if (document.IsNull() || document->HasOpenCommand()) return NO;
        TDF_LabelSequence labels;
        XCAFDoc_DocumentTool::ShapeTool(document->Main())->GetFreeShapes(labels);
        TDF_Label target;
        for (int i = 1; i <= labels.Length(); ++i)
            if (owner->EntityIdentifierForLabel(labels.Value(i)) == identifier.UTF8String) target = labels.Value(i);
        core3d::profile::Record before;
        if (target.IsNull() || !core3d::profile::Read(document, target, before) || before.label.IsNull()) return NO;
        if (mode >= 9 && !before.parameters.constructionFrame) return NO;
        const int frameStart = static_cast<int>(before.values.size()) - 8 + 1;
        const auto undo = document->GetAvailableUndos();
        document->NewCommand(); opened = true;
        switch (mode) {
            case 1: TDataStd_Integer::Set(before.label, core3d::profile::SchemaID(), 99); break;
            case 2: TDataStd_Integer::Set(before.label, core3d::profile::CountID(), core3d::profile::MaximumScalars + 1); break;
            case 3: before.label.ForgetAttribute(TNaming_NamedShape::GetID()); break;
            case 4: TDataStd_Real::Set(before.label.FindChild(2, Standard_False), std::numeric_limits<double>::quiet_NaN()); break;
            case 5: TDataStd_AsciiString::Set(before.label, core3d::profile::IdentityID(), TCollection_AsciiString("invalid")); break;
            case 6: TDataStd_Real::Set(before.label, TDataStd_Real::GetID(), 1); break;
            case 7: TDataStd_Integer::Set(document->Main(), core3d::profile::SchemaID(), 1); break;
            case 8: TDataStd_Real::Set(before.label.FindChild(2, Standard_False), 0); break;
            case 9: TDataStd_Integer::Set(before.label, core3d::profile::SchemaID(), 1); break;
            case 10: TDataStd_Integer::Set(before.label, core3d::profile::CountID(), static_cast<int>(before.values.size()) - 1); break;
            case 11: TDataStd_Real::Set(before.label.FindChild(frameStart + 3, Standard_False), 2); break;
            case 12:
                for (int i = 3; i <= 6; ++i) TDataStd_Real::Set(before.label.FindChild(frameStart + i, Standard_False), 0);
                break;
            case 13: TDataStd_Real::Set(before.label.FindChild(frameStart + 7, Standard_False), 0); break;
            case 14: TDataStd_Real::Set(before.label.FindChild(frameStart + 7, Standard_False), 1e6 + 1); break;
            case 15: TDataStd_Real::Set(before.label.FindChild(frameStart, Standard_False), 1e6 + 1); break;
            case 16: TDataStd_Real::Set(before.label.FindChild(frameStart, Standard_False), std::numeric_limits<double>::quiet_NaN()); break;
            case 17: TDataStd_Integer::Set(before.label.FindChild(frameStart, Standard_False), 1); break;
            case 18: TDataStd_Real::Set(before.label.FindChild(frameStart, Standard_False).FindChild(1, Standard_True), 1); break;
            case 19: TDataStd_Real::Set(before.label.FindChild(frameStart, Standard_False).FindChild(1, Standard_True).FindChild(1, Standard_True), 1); break;
        }
        const bool rejected = !owner->ValidateGeometryRepresentations();
        document->AbortCommand(); opened = false;
        core3d::profile::Record after;
        return rejected && owner->ValidateGeometryRepresentations()
            && document->GetAvailableUndos() == undo && core3d::profile::Read(document, target, after)
            && before.IsEqual(after) && after.IsCurrent(document, target);
    } catch (...) {
        if (opened && !document.IsNull()) { try { document->AbortCommand(); } catch (...) {} }
        return NO;
    }
}

- (BOOL)debugStartLiveTransactionProbe {
    if (![NSThread isMainThread] || GLController == nil || GLController.viewer == nullptr) return NO;
    auto document = GLController.viewer->getDocument();
    return !document.IsNull() && document->DebugStartLiveTransactionProbe();
}
- (void)debugStopLiveTransactionProbe {
    if (![NSThread isMainThread] || GLController == nil || GLController.viewer == nullptr) return;
    auto document = GLController.viewer->getDocument();
    if (!document.IsNull()) document->DebugStopLiveTransactionProbe();
}
- (NSDictionary<NSString *, id> *)debugLiveTransactionProbe {
    if (![NSThread isMainThread] || GLController == nil || GLController.viewer == nullptr) return nil;
    auto document = GLController.viewer->getDocument();
    if (document.IsNull()) return nil;
    const auto state = document->DebugLiveTransactionProbe();
    if (!state) return nil;
    static NSArray<NSString *> *names = @[@"attached", @"open", @"commit", @"abort",
        @"undoCompleted", @"redoCompleted", @"adopted", @"detached"];
    NSMutableArray *events = [NSMutableArray arrayWithCapacity:state->count];
    for (std::size_t index = 0; index < state->count; ++index) {
        const auto& event = state->events[index];
        [events addObject:@{@"kind":names[static_cast<unsigned>(event.kind)],
            @"sequence":@(event.sequence), @"opening":@(event.opening),
            @"active":@(event.active), @"commandOpen":@(event.commandOpen),
            @"undos":@(event.undos), @"redos":@(event.redos)}];
    }
    return @{@"valid":@(document->DebugLiveTransactionProbeValid()),
        @"opening":@(state->opening), @"count":@(state->count), @"events":events};
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
            bool observed = false; NSMutableArray* shaderState = [NSMutableArray array];
            struct ObserverReset { OcctViewer* viewer; ~ObserverReset() { viewer->DebugSetFrameObserver({}); } } observerReset{viewer.get()};
            viewer->DebugSetFrameObserver([&](bool rendered) {
                if (rendered) {
                    const auto driver = Handle(OpenGl_GraphicDriver)::DownCast(viewer->V3dViewer()->Driver());
                    const auto gl = driver->GetSharedContext(true);
                    for (const auto& prs : scope.presentation->Presentations()) {
                        if (prs.IsNull()) continue;
                        for (const auto& generic : prs->Groups()) {
                            const auto group = Handle(OpenGl_Group)::DownCast(generic); if (group.IsNull()) continue;
                            auto inspect = [&](const Handle(Graphic3d_Aspects)& aspect) {
                                if (!core3d::render::NeedsNativeTangents(aspect)) return;
                                Handle(OpenGl_ShaderProgram) program;
                                if (!gl->GetResource(aspect->ShaderProgram()->GetId(), program) || program.IsNull()) return;
                                const GLuint identifier = program->ProgramId();
                                NSMutableDictionary* row = [NSMutableDictionary dictionaryWithDictionary:@{ @"program": @(identifier) }];
                                for (const char* name : {"syNormalMap", "occSamplerNormal", "occModelWorldMatrix", "occWorldViewMatrix", "occTextureTrsf2d"}) {
                                    const GLint location = glGetUniformLocation(identifier, name); if (location < 0) continue;
                                    NSMutableArray* values = [NSMutableArray array];
                                    if (std::strcmp(name, "syNormalMap") == 0 || std::strcmp(name, "occSamplerNormal") == 0) { GLint value = -1; glGetUniformiv(identifier, location, &value); [values addObject:@(value)]; }
                                    else { GLfloat value[16] = {}; glGetUniformfv(identifier, location, value); for (int i = 0; i < 16; ++i) [values addObject:@(value[i])]; }
                                    row[[NSString stringWithUTF8String:name]] = values;
                                }
                                GLint front = 0; glGetIntegerv(GL_FRONT_FACE, &front); row[@"frontFaceAfterDraw"] = @(front);
                                row[@"highlighted"] = @(context->IsHilighted(scope.presentation));
                                [shaderState addObject:row];
                            };
                            inspect(group->Aspects());
                            for (auto* node = group->FirstNode(); node; node = node->next)
                                if (const auto aspect = dynamic_cast<OpenGl_Aspects*>(node->elem)) inspect(aspect->Aspect());
                        }
                    }
                    return true;
                }
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
            viewer->DebugSetFrameObserver({});
            if (!observed || ![stats[@"captured"] boolValue]) Standard_Failure::Raise("Native framebuffer preparation or capture failed.");
            selectionUnchanged = selectionUnchanged && ownersBefore == selectedOwners() && modesBefore == selectedModes();
            readOnly = readOnly && time == wrapper->Document()->GetData()->Time() && undo == wrapper->Document()->GetAvailableUndos() && redo == wrapper->Document()->GetAvailableRedos();
            [states addObject:@{@"step": @(step), @"frames": frames, @"uids": uids, @"stats": stats, @"shaderState": shaderState,
                @"prepared": @(viewer->DebugPreparedTangentArrayCount()), @"selectedOwners": @(ownersBefore.size()), @"selectionUnchanged": @(selectionUnchanged), @"history": @[@(undo),@(redo)]}];
            if (captureImage) {
                const auto pixels = [GLController debugViewportRGBA];
                if (!pixels) Standard_Failure::Raise("Native viewport image failed.");
                captures[[NSString stringWithFormat:@"%d",step]] = pixels;
            }
        };
        scope.document->NewCommand();
        keep(0, true, true);
        bool previewCommandsPreserved = scope.document->HasOpenCommand();
        scope.document->AbortCommand(); keep(1, false, false);
        scope.document->NewCommand(); assign(replacement); scope.document->CommitCommand(); keep(2, true, true);
        if (!scope.document->Undo()) Standard_Failure::Raise("Native frame undo failed."); keep(3, true, false);
        if (!scope.document->Redo()) Standard_Failure::Raise("Native frame redo failed."); keep(4, true, false);
        context->Erase(scope.presentation, Standard_False);
        scope.document->NewCommand(); assign(archive); scope.document->CommitCommand(); keep(5, true, false);
        context->Display(scope.presentation, AIS_Shaded, -1, Standard_False); keep(6, false, false);
        scope.document->NewCommand(); label.ForgetAttribute(AuthoredFrameAttributeID()); scope.document->CommitCommand();
        scope.document->NewCommand(); keep(7, true, false);
        previewCommandsPreserved = previewCommandsPreserved && scope.document->HasOpenCommand();
        scope.document->AbortCommand();
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
        return @{@"states": states, @"captures": captures, @"readOnly": @(readOnly), @"selectionUnchanged": @(selectionUnchanged), @"invalidRejected": @(invalidRejected), @"previewCommandsPreserved": @(previewCommandsPreserved), @"cacheCleared": @YES};
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

- (NSData *_Nullable)debugSuppliedFrameBinXCAFFixture:(NSData *)archive normalPNG:(NSData *)normalPNG mode:(NSInteger)mode {
    if (![NSThread isMainThread] || mode < 0 || mode > 8 || archive.length < 128
        || archive.length > core3d::scene::authored::kMaximumArchiveBytes || normalPNG.length > 16384) return nil;
    struct Scope {
        Handle(OcctDocument) wrapper = new OcctDocument();
        Handle(TDocStd_Document) document;
        ~Scope() noexcept { try { if (!document.IsNull()) {
            const auto app = Handle(TDocStd_Application)::DownCast(document->Application());
            if (!app.IsNull()) app->Close(document);
        } } catch (...) {} }
    } scope;
    try {
        using namespace core3d::persistence;
        scope.wrapper->InitDoc(); scope.document = scope.wrapper->Document();
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(scope.document->Main());
        scope.document->NewCommand();
        const auto label = shapes->AddShape(Core3DDebugAuthoredGeometryFixture(0), Standard_False);
        if (mode == 8) {
            const auto second = shapes->AddShape(Core3DDebugAuthoredGeometryFixture(0),Standard_False);
            if (second.IsNull() || !scope.wrapper->SetGeometryRepresentationForLabel(second,OcctGeometryRepresentation::TriangleMesh))
                Standard_Failure::Raise("Mixed native basis fixture setup failed.");
        }
        if (label.IsNull() || !scope.wrapper->SetGeometryRepresentationForLabel(label, OcctGeometryRepresentation::TriangleMesh)
            || !scope.document->CommitCommand())
            Standard_Failure::Raise("Supplied frame fixture geometry setup failed.");
        // Schema migration is allowed only before user history. This fixture's
        // seed geometry is construction setup, not a user edit to preserve.
        scope.document->ClearUndos();
        if (!scope.wrapper->MigrateLegacyIdentifiers())
            Standard_Failure::Raise("Supplied frame fixture identity setup failed.");
        auto assign = [&](const TDF_Label& owner) {
            const auto attribute = TDataStd_ByteArray::Set(owner, AuthoredFrameAttributeID(), 0, int(archive.length)-1, Standard_False);
            const auto* bytes = static_cast<const std::uint8_t*>(archive.bytes);
            for (NSUInteger i=0; i<archive.length; ++i) attribute->SetValue(int(i),bytes[i]);
            return attribute;
        };
        scope.document->NewCommand();
        auto frame = assign(label);
        OcctAuthoredFrameRecord record;
        if (Core3DReadAuthoredFrameOwner(scope.document,label,record) != OcctAuthoredFrameReadState::Authored)
            Standard_Failure::Raise("Supplied frame fixture archive/geometry mismatch.");
        if (mode != 0 && mode != 8) {
            XCAFDoc_VisMaterialPBR material; material.BaseColor = Quantity_ColorRGBA(1,1,1,1);
            material.Metallic = 0; material.Roughness = 0.8f;
            if (!Core3DCreateAuthoredTexture(static_cast<const Standard_Byte*>(normalPNG.bytes), normalPNG.length,
                    "image/png", material.NormalTexture)
                || !scope.wrapper->SaveObjectPBRMaterial(label,material)
                || Core3DNormalTextureRecipeForLabel(label) != 2)
                Standard_Failure::Raise("Supplied frame fixture normal binding failed.");
            XCAFDoc_VisMaterialTool::GetShapeMaterial(label)->SetFaceCulling(Graphic3d_TypeOfBackfacingModel_DoubleSided);
        }
        if (!scope.document->CommitCommand()) Standard_Failure::Raise("Supplied frame fixture did not commit.");
        scope.document->ClearUndos();
        // Deliberate private malformed candidates exercise the actual production reader/adoption gate.
        if (mode == 2) TDataStd_Integer::Set(label, Standard_GUID("98EAD304-EB49-4F0E-ABFC-AEF94C250161"),1);
        if (mode == 3) label.ForgetAttribute(AuthoredFrameAttributeID());
        if (mode == 4) shapes->SetShape(label,Core3DDebugAuthoredGeometryFixture(2));
        if (mode == 5) { assign(scope.document->GetData()->Root()); label.ForgetAttribute(AuthoredFrameAttributeID()); }
        if (mode == 6) frame->SetID(TDataStd_ByteArray::GetID());
        if (mode == 7) frame->SetValue(frame->Upper(),frame->Value(frame->Upper()) ^ 1);
        std::ostringstream stream(std::ios::out|std::ios::binary);
        const auto app = Handle(TDocStd_Application)::DownCast(scope.document->Application());
        if (app->SaveAs(scope.document,stream) != PCDM_SS_OK) return nil;
        const auto bytes=stream.str(); if (bytes.empty() || bytes.size()>1024*1024) return nil;
        return [NSData dataWithBytes:bytes.data() length:bytes.size()];
    } catch (const Standard_Failure& failure) {
        NSLog(@"Supplied frame fixture failed: %s",failure.GetMessageString()); return nil;
    } catch (...) { return nil; }
}

- (NSArray<NSDictionary<NSString *, id> *> *_Nullable)debugNativeSuppliedFrameState {
    if (![NSThread isMainThread] || !_isSetuped || !GLController.viewer) return nil;
    try {
        const auto wrapper = GLController.viewer->getDocument(); const auto document=wrapper->Document();
        Standard_Size bytes=0,resident=0,exact=0,below=123;
        if (!Core3DValidateAuthoredFrameOwners(document,bytes) || !Core3DValidateOwnedFrameUsage(document,resident)) return nil;
        const bool fitsExact=Core3DValidateOwnedFrameUsage(document,exact,resident);
        const bool rejectsBelow=resident>0 && !Core3DValidateOwnedFrameUsage(document,below,resident-1);
        TDF_LabelSequence roots; XCAFDoc_DocumentTool::ShapeTool(document->Main())->GetFreeShapes(roots);
        if (roots.Length()>32) return nil;
        NSMutableArray* result=[NSMutableArray array];
        for (int i=1;i<=roots.Length();++i) {
            const auto& label=roots.Value(i); OcctAuthoredFrameRecord record;
            if (Core3DReadAuthoredFrameOwner(document,label,record)==OcctAuthoredFrameReadState::Invalid) return nil;
            Standard_Size extra=0; const auto basis=Core3DNormalTextureBasisForLabel(document,label,&extra);
            [result addObject:@{@"entity": [NSString stringWithUTF8String:wrapper->EntityIdentifierForLabel(label).c_str()],
                @"recipe": @(Core3DNormalTextureRecipeForLabel(label)), @"basis": @(basis),
                @"archive": [NSData dataWithBytes:record.archive.data() length:record.archive.size()],
                @"ownerBytes": @(record.nativeBytes), @"additionalNormalBytes": @(extra), @"allOwnerBytes": @(bytes),
                @"allResidentBytes": @(resident), @"exactLimitAccepted": @(fitsExact && exact==resident),
                @"oneByteLessRejected": @(rejectsBelow), @"rejectedOutputCleared": @(below==0)}];
        }
        return result;
    } catch (...) { return nil; }
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

- (NSDictionary<NSString *, NSNumber *> *)debugNativeFramePreparationState {
    if (![NSThread isMainThread] || GLController == nil || GLController.viewer == nullptr) return @{};
    const auto viewer = GLController.viewer;
    const auto& trace = viewer->DebugNativeFrameFailure();
    return @{ @"failures": @(viewer->DebugNativeFrameFailures()),
              @"successes": @(viewer->DebugNativeFrameSuccesses()),
              @"stage": @(trace.stage), @"elements": @(trace.sourceElements),
              @"attributes": @(trace.sourceAttributes), @"cpu": @(trace.sourceCPUData),
              @"corner": @(trace.mismatchCorner), @"component": @(trace.mismatchComponent),
              @"expected": @(trace.expectedBits), @"actual": @(trace.actualBits) };
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

- (BOOL)previewMirrorAxis:(Core3DMirrorAxis)axis backward:(BOOL)backward {
    if (![NSThread isMainThread]
        || !_isSetuped
        || _currentGizmoType != PrimitiveGizmoTypeMirror
        || axis < Core3DMirrorAxisX || axis > Core3DMirrorAxisZ) {
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
        // Axis-menu and viewport-handle previews share the authoritative
        // native operation and finalized UI/publication lifecycle.
        [self viewDidEndPrimaryInteractionCancelled:NO];
        return didCreate;
    } catch (...) {
        return NO;
    }
}

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

#ifdef DEBUG
- (NSDictionary<NSString *, id> *)debugNativeLightingState {
    if (!NSThread.isMainThread || !_isSetuped || !GLController.isViewLoaded) return nil;
    const auto viewer = GLController.viewer;
    if (!viewer || viewer->ActiveView().IsNull()) return nil;
    try {
        const auto& view = viewer->ActiveView();
        NSMutableArray* lights = [NSMutableArray array];
        for (auto it = view->ActiveLightIterator(); it.More(); it.Next()) {
            if (lights.count >= 8 || it.Value().IsNull()) return nil;
            const auto& light = it.Value();
            NSMutableDictionary* row = [@{@"type": @(light->Type()),
                @"directional": @(light->Type() == Graphic3d_TOLS_DIRECTIONAL),
                @"headlight": @(light->IsHeadlight()), @"intensity": @(light->Intensity())} mutableCopy];
            if (light->Type() == Graphic3d_TOLS_DIRECTIONAL) {
                const auto direction = light->Direction();
                row[@"direction"] = @[@(direction.X()), @(direction.Y()), @(direction.Z())];
            }
            [lights addObject:row];
        }
        NSMutableArray* shapes = [NSMutableArray array];
        if (!viewer->AisContext().IsNull()) {
            AIS_ListOfInteractive displayed;
            viewer->AisContext()->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
            for (AIS_ListIteratorOfListOfInteractive it(displayed); it.More(); it.Next()) {
                if (shapes.count >= 16) return nil;
                const auto shape = Handle(AIS_Shape)::DownCast(it.Value());
                if (shape.IsNull() || shape->Attributes().IsNull()
                    || shape->Attributes()->ShadingAspect().IsNull()) continue;
                const auto aspect = shape->Attributes()->ShadingAspect()->Aspect();
                if (aspect.IsNull()) continue;
                const auto rgb = [](const Quantity_Color& c) {
                    return @[@(c.Red()), @(c.Green()), @(c.Blue())];
                };
                const auto& front = aspect->FrontMaterial();
                const auto& back = aspect->BackMaterial();
                [shapes addObject:@{@"shadingModel": @(aspect->ShadingModel()),
                    @"customShader": @(!aspect->ShaderProgram().IsNull()),
                    @"distinguishMaterials": @(aspect->Distinguish()),
                    @"frontAmbient": rgb(front.AmbientColor()), @"frontDiffuse": rgb(front.DiffuseColor()),
                    @"frontSpecular": rgb(front.SpecularColor()), @"frontShininess": @(front.Shininess()),
                    @"backAmbient": rgb(back.AmbientColor()), @"backDiffuse": rgb(back.DiffuseColor()),
                    @"interiorColor": rgb(aspect->InteriorColor())}];
            }
        }
        return @{@"lights": lights, @"shadingModel": @(view->RenderingParams().ShadingModel),
            @"displayedShapeAspects": shapes};
    } catch (...) { return nil; }
}
- (BOOL)debugSetNativeHeadlightDirectionX:(double)x y:(double)y z:(double)z {
    if (!NSThread.isMainThread || !_isSetuped || !GLController.isViewLoaded
        || !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)
        || std::abs(x) > 16 || std::abs(y) > 16 || std::abs(z) > 16
        || x*x+y*y+z*z < 1e-12) return NO;
    const auto viewer = GLController.viewer;
    if (!viewer || !viewer->canBeginCommittedEdit() || viewer->ActiveView().IsNull()) return NO;
    try {
        Handle(V3d_Light) selected;
        for (auto it = viewer->ActiveView()->ActiveLightIterator(); it.More(); it.Next()) {
            const auto& light = it.Value();
            if (!light.IsNull() && light->Type() == Graphic3d_TOLS_DIRECTIONAL && light->IsHeadlight()) {
                if (!selected.IsNull()) return NO;
                selected = light;
            }
        }
        if (selected.IsNull()) return NO;
        selected->SetDirection(gp_Dir(x,y,z));
        viewer->ActiveView()->Invalidate();
        [GLController requestRender];
        return YES;
    } catch (...) { return NO; }
}

- (BOOL)debugReplayCameraTouch:(NSInteger)mode {
    if (!NSThread.isMainThread || !_isSetuped || !GLController.isViewLoaded
        || mode < 0 || mode > 3 || _currentGizmoType != PrimitiveGizmoTypeNone)
        return NO;
    const auto viewer = GLController.viewer;
    if (!viewer || !viewer->canBeginCommittedEdit()) return NO;
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width < 100 || size.height < 100) return NO;
    const int x = static_cast<int>(size.width * 0.5);
    const int y = static_cast<int>(size.height * 0.5);
    try {
        viewer->StartRotation(x, y);
        if (mode == 1) viewer->Rotation(x, y);
        if (mode >= 2) viewer->Rotation(x + 24, y + 16);
        if (mode == 3) viewer->Rotation(x, y);
        viewer->FinishInteraction(x, y);
        [GLController requestRender];
        return YES;
    } catch (...) { return NO; }
}
- (void)debugFailNextNativeViewportDraw {
    if (!NSThread.isMainThread || !GLController.isViewLoaded) return;
    const auto viewer = GLController.viewer;
    if (viewer == nullptr) return;
    const auto armed = std::make_shared<bool>(true);
    viewer->DebugSetFrameObserver([armed](bool rendered) {
        if (!rendered && *armed) { *armed = false; return false; }
        return true;
    });
}
- (void)debugSkipNextNativeViewportPresentation {
    if (!NSThread.isMainThread || !GLController.isViewLoaded) return;
    [(GLView *)GLController.view debugSkipNextPresentation];
}
#endif
- (BOOL)observeNativeViewportPresentation:(NSUUID *)identifier
    changed:(void (^)(Core3DSceneFrameSnapshot *_Nullable, CGSize))changed {
    if (!NSThread.isMainThread || !_isSetuped || !self.isViewLoaded
        || ![GLController.view isKindOfClass:GLView.class]) return NO;
    __weak Core3DViewController *weakSelf = self;
    return [(GLView *)GLController.view observeNextPresentation:identifier capture:^{
        return [weakSelf captureSceneFrameSnapshot];
    } changed:changed];
}
- (void)cancelNativeViewportPresentation:(NSUUID *)identifier {
    if (!NSThread.isMainThread || !GLController.isViewLoaded) return;
    [(GLView *)GLController.view cancelPresentationObservation:identifier];
}
- (BOOL)isNativeViewportPresentationCurrent:(NSUUID *)identifier {
    return NSThread.isMainThread && GLController.isViewLoaded
        && [(GLView *)GLController.view isPresentationObservationCurrent:identifier];
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

- (BOOL)frameMirrorPreviewWithExpected:(Core3DSceneSnapshot *)expected
                           generation:(uint64_t)generation
               normalizedViewportRect:(CGRect)rect {
    if (![NSThread isMainThread] || !_isSetuped || _nativeSolidWork
        || _isLoading.load() || GLController == nil || expected == nil
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128) { return NO; }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width < 1.0 || size.height < 1.0
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) { return NO; }
    const auto width = static_cast<std::uint32_t>(std::llround(size.width));
    const auto height = static_cast<std::uint32_t>(std::llround(size.height));
    const auto pixels = expected.camera.viewportSizePixels;
    if (pixels.x != width || pixels.y != height) { return NO; }
    const char* publication = expected.publicationSourceIdentifier.UTF8String;
    if (publication == nullptr) { return NO; }
    try {
        core3d::MirrorPreviewFrameIdentity identity;
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.revisions.documentGeneration = expected.revisions.documentGeneration;
        identity.revisions.model = expected.revisions.modelRevision;
        identity.revisions.presentation = expected.revisions.presentationRevision;
        identity.revisions.camera = expected.revisions.cameraRevision;
        identity.previewGeneration = generation;
        const auto viewer = GLController.viewer;
        if (viewer == nullptr || !viewer->frameMirrorPreview(identity, width, height,
            rect.origin.x, rect.origin.y, rect.size.width, rect.size.height)) { return NO; }
        [GLController requestRender];
        [self viewDidInvalidateSceneSnapshot];
        return YES;
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

- (Core3DMeshVertexEditSnapshot *)prepareMeshVertexEditForEntityIdentifier:(NSString *)entityIdentifier
                                                         expected:(Core3DSceneSnapshot *)expected {
    return [self prepareMeshElementEditForEntityIdentifier:entityIdentifier kind:Core3DMeshElementKindVertex expected:expected];
}

- (Core3DMeshVertexEditSnapshot *)prepareMeshElementEditForEntityIdentifier:(NSString *)entityIdentifier
    kind:(Core3DMeshElementKind)kind expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr || expected == nil
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128) { return nil; }
    switch(kind) {
        case Core3DMeshElementKindVertex:
        case Core3DMeshElementKindEdge:
        case Core3DMeshElementKindTriangle:break;
        default:return nil;
    }
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
        const auto result=GLController.viewer->prepareMeshElementEdit(identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)),
            static_cast<core3d::meshedit::ElementKind>(kind));
        return result ? [[Core3DMeshVertexEditSnapshot alloc] initWithNativeSnapshot:*result] : nil;
    } catch(...) {return nil;}
}

- (void)cancelMeshVertexEdit:(Core3DMeshVertexEditSnapshot *)expected {
    if(![NSThread isMainThread] || !_isSetuped || GLController==nil || GLController.viewer==nullptr
        || expected==nil || expected.sessionIdentifier.length==0 || expected.sessionIdentifier.length>128)return;
    const char *identifier=expected.sessionIdentifier.UTF8String;
    if(identifier!=nullptr)GLController.viewer->cancelMeshVertexEdit(identifier);
}

- (Core3DMeshVertexEditResult)commitMeshVertexEdit:(Core3DMeshVertexEditSnapshot *)expected
    vertexIndices:(NSArray<NSNumber *> *)vertexIndices deltaX:(double)deltaX deltaY:(double)deltaY deltaZ:(double)deltaZ {
    if(![NSThread isMainThread] || expected==nil || expected.elementKind!=Core3DMeshElementKindVertex)
        return Core3DMeshVertexEditResultRejected;
    return [self commitMeshElementEdit:expected elementIndices:vertexIndices deltaX:deltaX deltaY:deltaY deltaZ:deltaZ];
}

- (Core3DMeshVertexEditResult)commitMeshElementEdit:(Core3DMeshVertexEditSnapshot *)expected
    elementIndices:(NSArray<NSNumber *> *)vertexIndices deltaX:(double)deltaX deltaY:(double)deltaY deltaZ:(double)deltaZ {
    if(![NSThread isMainThread] || !_isSetuped || GLController==nil || GLController.viewer==nullptr
        || expected==nil || expected.sessionIdentifier.length==0 || expected.sessionIdentifier.length>128
        || vertexIndices.count==0 || vertexIndices.count>64 || !std::isfinite(deltaX)
        || !std::isfinite(deltaY) || !std::isfinite(deltaZ))return Core3DMeshVertexEditResultRejected;
    switch(expected.elementKind) {
        case Core3DMeshElementKindVertex:
        case Core3DMeshElementKindEdge:
        case Core3DMeshElementKindTriangle:break;
        default:return Core3DMeshVertexEditResultRejected;
    }
    try {
        std::vector<std::uint32_t> vertices;vertices.reserve(vertexIndices.count);
        for(NSNumber *value in vertexIndices) {
            if(![value isKindOfClass:[NSNumber class]] || CFGetTypeID((__bridge CFTypeRef)value)==CFBooleanGetTypeID())
                return Core3DMeshVertexEditResultRejected;
            const double number=value.doubleValue;
            if(!std::isfinite(number) || number<0 || number>=12288 || std::floor(number)!=number)
                return Core3DMeshVertexEditResultRejected;
            vertices.push_back(static_cast<std::uint32_t>(number));
        }
        const char *identifier=expected.sessionIdentifier.UTF8String;
        if(identifier==nullptr)return Core3DMeshVertexEditResultRejected;
        const auto result=GLController.viewer->commitMeshElementEdit(identifier,
            static_cast<core3d::meshedit::ElementKind>(expected.elementKind),vertices,gp_Vec(deltaX,deltaY,deltaZ));
        switch(result) {
            case core3d::OrdinaryEditResult::NoChange:return Core3DMeshVertexEditResultUnchanged;
            case core3d::OrdinaryEditResult::Committed:
                [GLController refreshSelectionState];[GLController requestRender];[self viewDidInvalidateSceneSnapshot];
                [self sendNotifyUIState:UIStateChangingHistory];return Core3DMeshVertexEditResultCommitted;
            case core3d::OrdinaryEditResult::Busy:return Core3DMeshVertexEditResultBusy;
            case core3d::OrdinaryEditResult::Invalid:return Core3DMeshVertexEditResultRejected;
            case core3d::OrdinaryEditResult::OutcomeUnknown:return Core3DMeshVertexEditResultRecoveryRequired;
            case core3d::OrdinaryEditResult::RetryableFailure:return Core3DMeshVertexEditResultFailed;
        }
    } catch(...) {}
    return Core3DMeshVertexEditResultRejected;
}

- (Core3DMeshCopyResult)createSourceRetainedMeshCopyForEntityIdentifier:(NSString *)entityIdentifier
                                                         expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr || expected == nil
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128) { return Core3DMeshCopyResultRejected; }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) { return Core3DMeshCopyResultRejected; }
    const char* entity = entityIdentifier.UTF8String;
    const char* publication = expected.publicationSourceIdentifier.UTF8String;
    if (entity == nullptr || publication == nullptr) { return Core3DMeshCopyResultRejected; }
    try {
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entity, [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto result = GLController.viewer->createSourceRetainedMeshCopy(identity,
            static_cast<std::uint32_t>(std::llround(size.width)),
            static_cast<std::uint32_t>(std::llround(size.height)));
        switch (result) {
            case core3d::OrdinaryEditResult::NoChange: return Core3DMeshCopyResultUnchanged;
            case core3d::OrdinaryEditResult::Committed:
                [GLController refreshSelectionState];
                [GLController requestRender];
                [self viewDidInvalidateSceneSnapshot];
                [self sendNotifyUIState:UIStateChangingHistory];
                return Core3DMeshCopyResultCommitted;
            case core3d::OrdinaryEditResult::Busy: return Core3DMeshCopyResultBusy;
            case core3d::OrdinaryEditResult::Invalid: return Core3DMeshCopyResultRejected;
            case core3d::OrdinaryEditResult::OutcomeUnknown: return Core3DMeshCopyResultRecoveryRequired;
            case core3d::OrdinaryEditResult::RetryableFailure: return Core3DMeshCopyResultFailed;
        }
    } catch (...) {}
    return Core3DMeshCopyResultRejected;
}

- (core3d::meshcheck::ContactSourceStatus)core3d_captureMeshContactSource:
    (const core3d::meshcheck::ContactSourceIdentity&)identity
    cancelled:(const std::atomic_bool&)cancelled
    output:(core3d::meshcheck::ContactSourceCapture&)output {
    using Status=core3d::meshcheck::ContactSourceStatus;
    output={};
    if(cancelled.load(std::memory_order_acquire)) return Status::Cancelled;
    if(!NSThread.isMainThread) return Status::InternalFailure;
    if(!_isSetuped || _isLoading.load() || _nativeSolidWork || _objectAlignmentWork
        || GLController==nil || !GLController.viewer) return Status::StaleSource;
    const CGSize size=GLController.drawableSize;
    if(!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width<1 || size.height<1
        || size.width>std::numeric_limits<std::uint32_t>::max()
        || size.height>std::numeric_limits<std::uint32_t>::max()) return Status::StaleSource;
    return GLController.viewer->captureNativeMeshContacts(identity,
        static_cast<std::uint32_t>(std::llround(size.width)),
        static_cast<std::uint32_t>(std::llround(size.height)),cancelled,output);
}

#if DEBUG
- (void)debugSetNextMeshContactDeliveryGate:(void (^)(void (^resume)(void)))gate {
    NSAssert(NSThread.isMainThread,@"Contact delivery gate requires the owner thread");
    _debugMeshContactDeliveryGate=[gate copy];
}
- (void)debugSetNextMeshContactAfterCaptureHook:(void (^)(void))afterCapture
                           beforeDeliveryHook:(void (^)(void))beforeDelivery {
    if(!NSThread.isMainThread) return;
    _debugMeshContactAfterCapture=[afterCapture copy];
    _debugMeshContactBeforeDelivery=[beforeDelivery copy];
}
#endif

- (Core3DMeshContactOperation *)checkMeshContactsForEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected completion:(Core3DMeshContactCompletion)completion {
    using namespace core3d::meshcheck;
    ContactSourceIdentity identity;
    const bool ownerThread=NSThread.isMainThread;
    // The DTO supplies bounded references, never geometry authority. The native
    // owner resolves the actual occurrence and validates these revision domains.
    if(ownerThread && expected!=nil && expected.renderItems.count<=50000
        && expected.meshes.count<=50000) {
        auto copyIdentifier=[](NSString *value,std::string& result) {
            const NSUInteger length=[value lengthOfBytesUsingEncoding:NSUTF8StringEncoding];
            const char *bytes=value.UTF8String;
            if(length==0 || length>128 || bytes==nullptr) return false;
            result.assign(bytes,length);
            return result.find('\0')==std::string::npos;
        };
        try {
            Core3DSceneRenderItemSnapshot *target=nil;
            NSUInteger matches=0;
            for(Core3DSceneRenderItemSnapshot *item in expected.renderItems) {
                if(item.renderRole==Core3DSceneRenderRoleModel
                    && [item.entityIdentifier isEqualToString:entityIdentifier]) {
                    target=item;++matches;
                }
            }
            if(matches==1 && target.meshIndex<expected.meshes.count) {
                Core3DSceneMeshSnapshot *mesh=expected.meshes[target.meshIndex];
                if(copyIdentifier(entityIdentifier,identity.entityIdentifier)
                    && copyIdentifier(mesh.definitionIdentifier,identity.definitionIdentifier)
                    && copyIdentifier(expected.publicationSourceIdentifier,identity.publicationSourceIdentifier)) {
                    identity.documentGeneration=expected.revisions.documentGeneration;
                    identity.modelRevision=expected.revisions.modelRevision;
                    identity.geometryRevision=mesh.geometryRevision;
                } else identity={};
            }
        } catch(...) { identity={}; }
    }
    NSObject *token=[[NSObject alloc] init];
    if(ownerThread) {
        [_meshContactOperation cancel];
        _meshContactOperation=nil;_meshContactSource.reset();
        _issuedMeshContactReport=nil;_meshContactToken=token;
    }
    __weak Core3DViewController *weakOwner=self;
    Core3DMeshContactOperation *operation=[[Core3DMeshContactOperation alloc]
        initWithIdentity:identity ownerToken:token
        prepare:^ContactSourceStatus(const std::atomic_bool& cancelled,
            const std::shared_ptr<void>& reservation,
            std::shared_ptr<const ContactSourceCapture>& output) {
            if(!ownerThread) return ContactSourceStatus::InternalFailure;
            Core3DViewController *owner=weakOwner;
            if(owner==nil || owner->_meshContactToken!=token) return ContactSourceStatus::StaleSource;
            ContactSourceCapture captured;
            const auto status=[owner core3d_captureMeshContactSource:identity
                cancelled:cancelled output:captured];
            if(status==ContactSourceStatus::Ready) {
                // The alias shares an owner containing both the exact numeric
                // source and its budget reservation. No job/source cycle and
                // no OCCT handle can enter this ownership graph.
                struct ReservedSource {
                    std::shared_ptr<void> reservation;
                    ContactSourceCapture capture;
                };
                auto reserved=std::make_shared<ReservedSource>(
                    ReservedSource{reservation,std::move(captured)});
                output=std::shared_ptr<const ContactSourceCapture>(reserved,&reserved->capture);
                owner->_meshContactSource=output;
            }
            return status;
        }
        validate:^ContactSourceStatus(const ContactSourceCapture& original,
            const std::atomic_bool& cancelled) {
            Core3DViewController *owner=weakOwner;
            if(owner==nil || owner->_meshContactToken!=token
                || owner->_meshContactSource.get()!=&original) return ContactSourceStatus::StaleSource;
            ContactSourceCapture current;
            const auto status=[owner core3d_captureMeshContactSource:original.identity
                cancelled:cancelled output:current];
            if(status!=ContactSourceStatus::Ready) return status;
            return SameContactSource(original,current)
                ? ContactSourceStatus::Ready : ContactSourceStatus::StaleSource;
        }
        completion:^(Core3DMeshContactReport *report) {
            Core3DViewController *owner=weakOwner;
            if(owner!=nil && owner->_meshContactToken==token) {
                owner->_meshContactOperation=nil;
                if(report.state==Core3DMeshContactStateComplete) {
                    owner->_issuedMeshContactReport=report;
                    [report core3d_setReleaseHandler:^{
                        dispatch_async(dispatch_get_main_queue(), ^{
                            Core3DViewController *currentOwner=weakOwner;
                            if(currentOwner!=nil && currentOwner->_meshContactToken==token
                                && currentOwner->_issuedMeshContactReport==nil) {
                                currentOwner->_meshContactSource.reset();
                                currentOwner->_meshContactToken=nil;
                            }
                        });
                    }];
                }
                else { owner->_issuedMeshContactReport=nil;owner->_meshContactSource.reset(); }
            }
            // Publish owner state before invoking user code; the callback may
            // start a replacement job without the old delivery clearing it.
            if(completion) completion(report);
        }];
    if(ownerThread) {
#if DEBUG
        [operation core3d_setDeliveryGate:_debugMeshContactDeliveryGate];
        _debugMeshContactDeliveryGate=nil;
        [operation core3d_setAfterCaptureHook:_debugMeshContactAfterCapture
            beforeDeliveryHook:_debugMeshContactBeforeDelivery];
        _debugMeshContactAfterCapture=nil;_debugMeshContactBeforeDelivery=nil;
#endif
        _meshContactOperation=operation;
    }
    return operation;
}

- (void)discardMeshContactReport:(Core3DMeshContactReport *)report {
    if(!NSThread.isMainThread || report==nil || report!=_issuedMeshContactReport
        || report.ownerToken!=_meshContactToken) return;
    _issuedMeshContactReport=nil;_meshContactSource.reset();_meshContactToken=nil;
}

- (BOOL)isMeshContactReportCurrent:(Core3DMeshContactReport *)report {
    using namespace core3d::meshcheck;
    if(!NSThread.isMainThread || report==nil || report.class!=Core3DMeshContactReport.class
        || report!=_issuedMeshContactReport || report.ownerToken!=_meshContactToken
        || report.state!=Core3DMeshContactStateComplete || !_meshContactSource) return NO;
    const auto& original=*_meshContactSource;
    const auto& identity=original.identity;
    // Public report metadata cannot replace the retained owner-issued capture.
    if(![report.entityIdentifier isEqualToString:@(identity.entityIdentifier.c_str())]
        || ![report.definitionIdentifier isEqualToString:@(identity.definitionIdentifier.c_str())]
        || ![report.publicationSourceIdentifier isEqualToString:@(identity.publicationSourceIdentifier.c_str())]
        || report.documentGeneration!=identity.documentGeneration
        || report.modelRevision!=identity.modelRevision
        || report.geometryRevision!=identity.geometryRevision) return NO;
    std::atomic_bool cancelled{false};ContactSourceCapture current;
    return [self core3d_captureMeshContactSource:identity cancelled:cancelled output:current]
        ==ContactSourceStatus::Ready && SameContactSource(original,current);
}

- (Core3DMeshContactInspection *)makeMeshContactInspectionForReport:
    (Core3DMeshContactReport *)report pairIndex:(NSInteger)pairIndex {
    if (![self isMeshContactReportCurrent:report] || pairIndex<0
        || (NSUInteger)pairIndex>=report.unexpectedPairs.count) return nil;
    Core3DMeshContactPair *pair=report.unexpectedPairs[(NSUInteger)pairIndex];
    return [[Core3DMeshContactInspection alloc] initWithSource:*_meshContactSource
        firstTriangle:pair.firstTriangle secondTriangle:pair.secondTriangle];
}

- (Core3DMeshWindingRepairResult)repairMeshWindingForEntityIdentifier:(NSString *)entityIdentifier
                                                         expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr || expected == nil
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || expected.publicationSourceIdentifier.length == 0
        || expected.publicationSourceIdentifier.length > 128) { return Core3DMeshWindingRepairResultRejected; }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) { return Core3DMeshWindingRepairResultRejected; }
    const char* entity = entityIdentifier.UTF8String;
    const char* publication = expected.publicationSourceIdentifier.UTF8String;
    if (entity == nullptr || publication == nullptr) { return Core3DMeshWindingRepairResultRejected; }
    try {
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entity, [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto result = GLController.viewer->repairMeshWinding(identity, expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),
            static_cast<std::uint32_t>(std::llround(size.height)));
        switch (result) {
            case core3d::OrdinaryEditResult::NoChange: return Core3DMeshWindingRepairResultUnchanged;
            case core3d::OrdinaryEditResult::Committed:
                [GLController refreshSelectionState];
                [GLController requestRender];
                [self viewDidInvalidateSceneSnapshot];
                [self sendNotifyUIState:UIStateChangingHistory];
                return Core3DMeshWindingRepairResultCommitted;
            case core3d::OrdinaryEditResult::Busy: return Core3DMeshWindingRepairResultBusy;
            case core3d::OrdinaryEditResult::Invalid: return Core3DMeshWindingRepairResultRejected;
            case core3d::OrdinaryEditResult::OutcomeUnknown: return Core3DMeshWindingRepairResultRecoveryRequired;
            case core3d::OrdinaryEditResult::RetryableFailure: return Core3DMeshWindingRepairResultFailed;
        }
    } catch (...) {}
    return Core3DMeshWindingRepairResultRejected;
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
    [_savedCutSourceJob cancel];
    _nativeSolidCancelled = YES;
    core3d::Core3DViewer::cancelNativeSolid(_nativeSolidWork);
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
    if (_nativeSolidWork || _isLoading.load()) { completion(Core3DProfileConstructionResultBusy); return; }
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
        [self runNativeSolidWork:work completion:completion];
    } catch (...) { completion(Core3DProfileConstructionResultRejected); }
}

- (void)createEnclosureWithDefinition:(Core3DEnclosureDefinition *)definition
    expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) return;
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ completion(Core3DProfileConstructionResultRejected); });
        return;
    }
    if (_nativeSolidWork || _isLoading.load()) { completion(Core3DProfileConstructionResultBusy); return; }
    if (!_isSetuped || GLController == nil || GLController.viewer == nullptr
        || ![definition isKindOfClass:[Core3DEnclosureDefinition class]]
        || ![expected isKindOfClass:[Core3DSceneSnapshot class]] || expected.selectionMode != Core3DSceneElementKindObject
        || definition.metersPerUnit != expected.metersPerUnit
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
        const char *publication = expected.publicationSourceIdentifier.UTF8String;
        if (!publication) { completion(Core3DProfileConstructionResultRejected); return; }
        core3d::ObjectFrameIdentity identity;
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto parameters = [definition nativeParameters];
        const auto work = viewer->prepareEnclosureSolid(parameters,identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        if (!work) { completion(Core3DProfileConstructionResultRejected); return; }
        [self runNativeSolidWork:work completion:completion];
    } catch (...) { completion(Core3DProfileConstructionResultRejected); }
}

- (Core3DStoredEnclosureSnapshot *)storedEnclosureWithEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || _nativeSolidWork || _isLoading.load() || !_isSetuped
        || GLController == nil || GLController.viewer == nullptr
        || ![entityIdentifier isKindOfClass:[NSString class]] || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || ![expected isKindOfClass:[Core3DSceneSnapshot class]] || expected.selectionMode != Core3DSceneElementKindObject
        || expected.publicationSourceIdentifier.length == 0 || expected.publicationSourceIdentifier.length > 128) return nil;
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height) || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) return nil;
    try {
        if (!entityIdentifier.UTF8String || !expected.publicationSourceIdentifier.UTF8String) return nil;
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entityIdentifier.UTF8String,
            [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(expected.publicationSourceIdentifier.UTF8String,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto result = GLController.viewer->storedEnclosureDefinition(identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        return result ? [[Core3DStoredEnclosureSnapshot alloc] initWithNativeSnapshot:*result] : nil;
    } catch (...) { return nil; }
}

- (void)rebuildStoredEnclosure:(Core3DStoredEnclosureSnapshot *)original
    definition:(Core3DEnclosureDefinition *)definition expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) return;
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ completion(Core3DProfileConstructionResultRejected); }); return;
    }
    if (_nativeSolidWork || _isLoading.load()) { completion(Core3DProfileConstructionResultBusy); return; }
    if (![original isKindOfClass:[Core3DStoredEnclosureSnapshot class]]
        || ![definition isKindOfClass:[Core3DEnclosureDefinition class]]) {
        completion(Core3DProfileConstructionResultRejected); return;
    }
    const auto live = [self storedEnclosureWithEntityIdentifier:original.entityIdentifier expected:expected];
    if (!live) { completion(Core3DProfileConstructionResultRejected); return; }
    try {
        const CGSize size = GLController.drawableSize;
        const auto originalNative = [original nativeSnapshot];
        auto requested = [definition nativeParameters];
        const auto work = GLController.viewer->prepareStoredEnclosureRebuild(requested,
            originalNative,[live nativeSnapshot].identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        if (!work) { completion(Core3DProfileConstructionResultRejected); return; }
        [self runNativeSolidWork:work completion:completion];
    } catch (...) { completion(Core3DProfileConstructionResultRejected); }
}


- (BOOL)core3d_canCaptureModelingContext {
    return [self core3d_canCaptureModelingContextForReservation:nil];
}
- (BOOL)core3d_canCaptureModelingContextForReservation:(Core3DModelingPlanningContext *)context {
    if(!NSThread.isMainThread)return NO;
    if(_pendingModelingReservation&&(!context||_pendingModelingReservation->_requestContext!=context))return NO;
    return [NSThread isMainThread] && _isSetuped && !_isPreviewMode
        && [GLController canIssueModelingPlanningContext]
        && ![self core3d_hasCompetingLoadOrControllerWork]
        && [self core3d_canBeginCommittedEdit]
        && [GLController getSelectionType] == PrimitiveSelectionTypeShape
        && [GLController getGizmoType] == PrimitiveGizmoTypeNone
        && !GLController.viewer->getObjectInteractor()->isManipulatorGestureActive();
}

- (BOOL)prepareForModelingPlanning {
    if (!NSThread.isMainThread || _pendingModelingReservation) return NO;
    if (![NSThread isMainThread] || !_isSetuped || _isPreviewMode
        || ![GLController canIssueModelingPlanningContext]
        || [self core3d_hasCompetingLoadOrControllerWork]
        || ![self core3d_canBeginCommittedEdit]
        || _currentSelectionType != PrimitiveSelectionTypeShape
        || [GLController getSelectionType] != PrimitiveSelectionTypeShape) return NO;
    const PrimitiveGizmoType gizmo = [GLController getGizmoType];
    if (_currentGizmoType != gizmo || (gizmo != PrimitiveGizmoTypeNone
        && gizmo != PrimitiveGizmoTypeMoveRotate && gizmo != PrimitiveGizmoTypeScale)) return NO;
    const auto viewer = GLController.viewer;
    const auto interactor = viewer->getObjectInteractor();
    if (!interactor || interactor->isManipulatorGestureActive()) return NO;
    try {
        Core3DSceneSnapshot *before = [self captureSceneSnapshot];
        Core3DScenePresentationOverlaySnapshot *overlay = [self captureScenePresentationOverlay];
        if (!before || !overlay || before.selectionMode != Core3DSceneElementKindObject
            || overlay.suppressedEntityIdentifiers.count != 0) return NO;
        // Ordinary passive gizmo geometry is the only removable overlay.
        // Never interpret an active modeling preview as a harmless tool icon.
        const BOOL passiveOverlay = overlay.kind == Core3DScenePresentationOverlayKindNone
            || (gizmo == PrimitiveGizmoTypeMoveRotate && overlay.kind == Core3DScenePresentationOverlayKindMoveRotateGizmo)
            || (gizmo == PrimitiveGizmoTypeScale && overlay.kind == Core3DScenePresentationOverlayKindScaleGizmo);
        if (!passiveOverlay) return NO;
        for (Core3DSceneRenderItemSnapshot *item in overlay.renderItems)
            if (item.renderRole != Core3DSceneRenderRoleGizmo) return NO;
        const auto document = viewer->getDocument();
        if (document.IsNull()) return NO;
        const auto stamp = document->CaptureNativePlanningStamp(viewer->canBeginCommittedEdit());
        if (!stamp) return NO;
        // All admission checks precede the existing native transition. In
        // particular, a pending gesture/worker/recovery is never cancelled here.
        if (gizmo != PrimitiveGizmoTypeNone) [self setGizmoType:PrimitiveGizmoTypeNone];
        if (![self core3d_canCaptureModelingContext]) return NO;
        Core3DSceneSnapshot *after = [self captureSceneSnapshot];
        Core3DScenePresentationOverlaySnapshot *empty = [self captureScenePresentationOverlay];
        const auto finalStamp = document->CaptureNativePlanningStamp(viewer->canBeginCommittedEdit());
        if (!after || !empty || empty.kind != Core3DScenePresentationOverlayKindNone || !finalStamp
            || finalStamp->instanceNonce != stamp->instanceNonce || finalStamp->opening != stamp->opening
            || finalStamp->edit != stamp->edit
            || ![after.publicationSourceIdentifier isEqualToString:before.publicationSourceIdentifier]
            || after.revisions.documentGeneration != before.revisions.documentGeneration
            || after.revisions.modelRevision != before.revisions.modelRevision
            || after.selectionMode != before.selectionMode || after.metersPerUnit != before.metersPerUnit
            || after.selection.selectedElements.count != before.selection.selectedElements.count) return NO;
        for (NSUInteger i = 0; i < before.selection.selectedElements.count; ++i) {
            Core3DSceneElementIdentifier *a = before.selection.selectedElements[i];
            Core3DSceneElementIdentifier *b = after.selection.selectedElements[i];
            if (![a.entityIdentifier isEqualToString:b.entityIdentifier] || a.kind != b.kind
                || a.topologyIndex != b.topologyIndex || a.geometryRevision != b.geometryRevision) return NO;
        }
        return YES;
    } catch (...) { return NO; }
}

// Internal reservation slot excludes competing AI admission only. It is not an
// OCAF/viewer lock: manual touch may proceed and invalidate the original lease.
- (std::optional<core3d::request::ReservationDispatch>)core3d_registerReservation:
    (Core3DModelingPreparedRequest *)request completion:(Core3DReservationCompletion)completion {
    if(!NSThread.isMainThread||!completion||_pendingModelingReservation||_reservedModelingCapability
        ||![self isModelingPreparedRequestCurrent:request]
        ||request->_requestAdmitted||Core3DReservationEntries.count>=32)return std::nullopt;
    try {
        const auto operation=request->_requestDescriptor.operation;
        if(operation==core3d::request::Operation::CreateEnclosure||operation==core3d::request::Operation::CreateAssembly
            ||operation==core3d::request::Operation::RebuildEnclosure||operation==core3d::request::Operation::RebuildProfile
            ||operation==core3d::request::Operation::RebuildLoftStation||operation==core3d::request::Operation::SetPlacement){
            // Fail known catalog conflicts before any permanent reservation.
            if(!GLController||!GLController.viewer)return std::nullopt;
            const auto owner=GLController.viewer->getDocument();
            if(owner.IsNull()||owner->Document().IsNull()||owner->Document()->HasOpenCommand())return std::nullopt;
            if ((operation==core3d::request::Operation::RebuildEnclosure||operation==core3d::request::Operation::RebuildProfile
                ||operation==core3d::request::Operation::RebuildLoftStation)
                && !Core3DRebuildSourceMatches(request,owner)) return std::nullopt;
            if(operation==core3d::request::Operation::SetPlacement&&!Core3DPlacementSourceMatches(request,owner))return std::nullopt;
            core3d::receipt::Catalog prior;const auto status=core3d::receipt::Read(owner->Document(),prior);
            if((status!=core3d::receipt::ReadStatus::Absent&&status!=core3d::receipt::ReadStatus::Valid)
                ||!prior.supportsAppend()
                ||!prior.canAppend())return std::nullopt;
            if(prior.contains(request->_requestKey.request))return std::nullopt;
            request->_requestCreationCatalog=std::move(prior);
        }
        Core3DModelingReservationEntry *entry=[Core3DModelingReservationEntry new];
        entry->_owner=self;entry->_prepared=request;entry->_completion=[completion copy];
        entry->_dispatch.key=request->_requestKey;
        if(!Core3DRequestUUID(NSUUID.UUID,entry->_dispatch.deliveryID))return std::nullopt;
        entry->_admission.emplace(entry->_dispatch.key);
        if(!entry->_admission->begin())return std::nullopt;
        if(!Core3DReservationEntries)Core3DReservationEntries=[NSMutableDictionary dictionary];
        NSUUID *identifier=Core3DReservationIdentifier(entry->_dispatch.deliveryID);
        if(Core3DReservationEntries[identifier])return std::nullopt;
        Core3DReservationEntries[identifier]=entry;
        // All checks and callback allocation precede single main-thread consume.
        // There is no transaction or document command open across the wait.
        request->_requestAdmitted=YES;request->_requestContext->_planningConsumed=YES;
        request->_requestReservationDeliveryID=entry->_dispatch.deliveryID;
        _pendingModelingReservation=request;
        const auto deliveryID=entry->_dispatch.deliveryID;
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW,30*NSEC_PER_SEC),dispatch_get_main_queue(),^{
            Core3DDeadlineReservation(deliveryID);
        });
        return entry->_dispatch;
    }catch(...){return std::nullopt;}
}
- (BOOL)core3d_reservationMatches:(Core3DModelingPreparedRequest *)request {
    return NSThread.isMainThread&&request&&_pendingModelingReservation==request
        &&request->_requestOwner==self&&request->_requestAdmitted&&!request->_requestRetired
        &&[_issuedModelingPreparedRequests containsObject:request]
        &&Core3DHostSessionCurrent(request->_requestSession)
        &&[self core3d_modelingContext:request->_requestContext matchesAllowingConsumed:YES];
}
- (void)core3d_releaseReservation:(Core3DModelingPreparedRequest *)request {
    if(!NSThread.isMainThread)return;
    if(_pendingModelingReservation==request)_pendingModelingReservation=nil;
    if(_reservedModelingCapability&&_reservedModelingCapability->_entry->_prepared==request)
        _reservedModelingCapability=nil;
}
- (BOOL)core3d_acceptReservedCapability:(Core3DModelingReservedCapability *)capability {
    if(!NSThread.isMainThread||!capability||_reservedModelingCapability||capability->_taken
        ||capability->_delivery.disposition!=core3d::tombstone::Reservation::Reserved
        ||capability->_entry->_owner!=self
        ||!(capability->_entry->_dispatch.key==capability->_delivery.key)
        ||capability->_entry->_admission->phase()!=core3d::request::Phase::Reserved
        ||![self core3d_reservationMatches:capability->_entry->_prepared])return NO;
    _reservedModelingCapability=capability;return YES;
}
- (void)core3d_retireReservationContext:(Core3DModelingPlanningContext *)context {
    if(!NSThread.isMainThread||!_pendingModelingReservation
        ||_pendingModelingReservation->_requestContext!=context)return;
    Core3DModelingPreparedRequest *request=_pendingModelingReservation;
    Core3DModelingReservationEntry *entry=Core3DReservationEntries[
        Core3DReservationIdentifier(request->_requestReservationDeliveryID)];
    if(entry&&entry->_owner==self&&entry->_prepared==request) {
        entry->_stopped=YES;entry->_admission->stop();
    }
    if(_reservedModelingCapability&&_reservedModelingCapability->_entry->_prepared==request)
        _reservedModelingCapability->_entry->_admission->stop();
    request->_requestRetired=YES;context->_planningRetired=YES;
    if(request->_requestEpoch)request->_requestEpoch->retired=true;
    [self core3d_releaseReservation:request];
}

- (Core3DModelingPreparedRequest *)core3d_prepareRequest:(const core3d::request::Descriptor&)descriptor
    context:(Core3DModelingPlanningContext *)context session:(Core3DModelingHostSession *)session
    requestID:(NSUUID *)requestID featureIDs:(const std::vector<core3d::request::UUID>&)featureIDs
    coverage:(Core3DModelingEvidenceCoverage)coverage {
    namespace r=core3d::request;
    if (![self isModelingPlanningContextCurrent:context]||!Core3DHostSessionCurrent(session)) return nil;
    if (!_issuedModelingPreparedRequests) _issuedModelingPreparedRequests=[NSHashTable weakObjectsHashTable];
    if (_issuedModelingPreparedRequests.allObjects.count>=32) return nil;
    try {
        r::Key key;key.accountScope=session->_hostScopeDigest;
        if (!Core3DRequestUUID(requestID,key.request)
            || !Core3DRequestUUID([[NSUUID alloc] initWithUUIDString:context.documentIdentifier],key.document)
            || !r::CommandHash(descriptor,key.command)||featureIDs.size()!=descriptor.parts.size()) return nil;
        r::Writer authority;
        authority.raw(session->_hostIdentity.data(),session->_hostIdentity.size());
        authority.raw(session->_hostGenerationDigest.data(),session->_hostGenerationDigest.size());
        authority.raw(context->_planningStamp.instanceNonce.data(),context->_planningStamp.instanceNonce.size());
        authority.integer(context->_planningStamp.opening);authority.integer(context->_planningStamp.edit);
        authority.integer(context->_planningStamp.selection);authority.integer(context->_planningOverlayRevision);
        std::string publication;if (!Core3DRequestString(context.scene.publicationSourceIdentifier,128,publication)) return nil;
        authority.text(publication);authority.integer(context.scene.revisions.documentGeneration);
        authority.integer(context.scene.revisions.modelRevision);authority.integer(context.scene.revisions.presentationRevision);
        authority.scalar(context.scene.metersPerUnit);authority.integer(context.scene.selection.selectedElements.count,4);
        for (Core3DSceneElementIdentifier *element in context.scene.selection.selectedElements) {
            std::string entity;if (!Core3DRequestString(element.entityIdentifier,128,entity)) return nil;
            authority.text(entity);authority.integer(element.kind);authority.integer(element.topologyIndex);authority.integer(element.geometryRevision);
        }
        // Target state comes from native persisted object authority. Renderer
        // Float buffers never supply recipe units, transforms or stable IDs.
        NSString *target=nil;std::vector<double> originalValues;
        std::optional<core3d::receipt::Effect> expectedSource;
        if (descriptor.operation==r::Operation::RebuildProfile) {
            if (!context.selectedProfile) return nil;
            const auto original=[context.selectedProfile nativeSnapshot];
            if (!core3d::profile::Encode(original.parameters,originalValues)) return nil;
            target=context.selectedProfile.entityIdentifier;authority.scalar(original.dimensionMetersPerUnit);
            authority.text(original.featureIdentifier);
        } else if (descriptor.operation==r::Operation::RebuildEnclosure) {
            if (!context.selectedEnclosure) return nil;
            const auto original=[context.selectedEnclosure nativeSnapshot];
            if (!core3d::enclosure::Encode(original.parameters,originalValues)) return nil;
            target=context.selectedEnclosure.entityIdentifier;authority.scalar(original.dimensionMetersPerUnit);
            authority.text(original.featureIdentifier);
        }
        if (descriptor.operation==r::Operation::RebuildLoftStation) {
            if (!context.selectedLoft) return nil;
            const auto original=[context.selectedLoft nativeSnapshot];
            if (!core3d::loft_persistence::Encode(original.definition,originalValues)) return nil;
            target=context.selectedLoft.entityIdentifier;authority.scalar(original.effectiveDimensionMetersPerUnit);
            authority.text(original.featureIdentifier);
        }
        authority.integer(originalValues.size(),4);for(double value:originalValues) authority.scalar(value);
        if (target) {
            const auto viewer=context->_planningViewer.lock();if (!viewer) return nil;
            const auto document=viewer->getDocument();if (document.IsNull()||document->Document().IsNull()
                ||!XCAFDoc_DocumentTool::CheckShapeTool(document->Document()->Main())) return nil;
            TDF_LabelSequence roots;XCAFDoc_DocumentTool::ShapeTool(document->Document()->Main())->GetFreeShapes(roots);
            if (roots.Length()>50000) return nil;
            OcctObjectTransformState state;bool found=false;
            for (int i=1;i<=roots.Length();++i) {
                if (document->EntityIdentifierForLabel(roots.Value(i))!=target.UTF8String) continue;
                if (found||!document->CaptureObjectTransformStateForLabel(roots.Value(i),state)) return nil;found=true;
                core3d::receipt::Effect effect;
                if (!core3d::receipt::CaptureEffect(document,roots.Value(i),effect)) return nil;
                expectedSource=effect;
            }
            if (!found) return nil;
            authority.text(state.entityIdentifier);authority.text(state.definitionIdentifier);
            for (std::size_t i=0;i<state.scalars.size();++i) {authority.integer(state.present[i]?1:0,1);authority.scalar(state.scalars[i]);}
        }
        if (target) {
            if (!expectedSource) return nil;
            // Append a tagged rebuild-only authority extension. Creation bytes
            // and native command descriptors retain their existing encodings.
            if(descriptor.operation==r::Operation::RebuildLoftStation){
                if(expectedSource->policy!=core3d::receipt::ExactLoftPolicy4097
                    ||expectedSource->feature!=core3d::receipt::Feature::RectangularLoft)return nil;
                authority.text("loft-rebuild-source-effect/v1");
                authority.integer(expectedSource->policy,2);
                authority.integer(core3d::loft_persistence::Schema,4);
            }else authority.text("rebuild-source-effect/v1");
            authority.integer(std::uint8_t(expectedSource->feature),1);
            authority.raw(expectedSource->entity.data(),expectedSource->entity.size());
            authority.raw(expectedSource->definition.data(),expectedSource->definition.size());
            authority.raw(expectedSource->featureID.data(),expectedSource->featureID.size());
            authority.raw(expectedSource->geometry.data(),expectedSource->geometry.size());
            authority.raw(expectedSource->state.data(),expectedSource->state.size());
        }
        if(descriptor.operation==r::Operation::SetPlacement){
            if(!context->_planningPlacementEvidence||!context->_planningPlacementSnapshot||!descriptor.placement||!featureIDs.empty())return nil;
            const auto& evidence=*context->_planningPlacementEvidence;
            authority.text("selected-placement-source/v1");
            authority.integer(core3d::receipt::ExactPlacementPolicy8193,2);
            authority.integer(context->_planningPlacementSnapshot.positionEditGeneration);
            authority.integer(context->_planningPlacementSnapshot.documentEditGeneration);
            authority.integer(context->_planningPlacementSnapshot.geometryEditGeneration);
            authority.integer(evidence.stateBytes.size(),4);authority.raw(evidence.stateBytes.data(),evidence.stateBytes.size());
            expectedSource=core3d::placement::Effect(evidence);
        }
        authority.integer(featureIDs.size(),4);
        for (const auto&identifier:featureIDs) {if (!r::Nonzero(identifier)) return nil;authority.raw(identifier.data(),identifier.size());}
        if (!authority.valid||!r::ExecutionHash(key.command,key.accountScope,key.document,key.request,authority.bytes,key.execution)
            || ![self isModelingPlanningContextCurrent:context]||!Core3DHostSessionCurrent(session)) return nil;
        Core3DModelingPreparedRequest *request=[[Core3DModelingPreparedRequest alloc]
            initPrivateWithRequest:requestID document:context.documentIdentifier key:key coverage:coverage];
        request->_requestOwner=self;request->_requestContext=context;request->_requestSession=session;
        request->_requestEpoch=std::make_shared<core3d::NativeModelingEpoch>();
        request->_requestDescriptor=descriptor;request->_requestFeatureIDs=featureIDs;
        request->_requestSourceEffect=expectedSource;
        [_issuedModelingPreparedRequests addObject:request];return request;
    } catch (...) {return nil;}
}

- (Core3DModelingPreparedRequest *)prepareEnclosureRequest:(Core3DEnclosureDefinition *)definition
    rebuild:(BOOL)rebuild context:(Core3DModelingPlanningContext *)context
    session:(Core3DModelingHostSession *)session requestID:(NSUUID *)requestID {
    if (![self isModelingPlanningContextCurrent:context]||!Core3DHostSessionCurrent(session)
        || ![definition isKindOfClass:Core3DEnclosureDefinition.class]) return nil;
    try {
        auto parameters=[definition nativeParameters];core3d::request::UUID feature;
        if (rebuild) {
            if (!context.selectedEnclosure) return nil;const auto original=[context.selectedEnclosure nativeSnapshot];
            if (!original.current||parameters.metersPerUnit!=original.parameters.metersPerUnit
                || parameters.definition.constructionFrame!=original.parameters.definition.constructionFrame
                || !Core3DRequestUUID([[NSUUID alloc] initWithUUIDString:context.selectedEnclosure.featureIdentifier],feature)) return nil;
        } else if (parameters.metersPerUnit!=context.scene.metersPerUnit||!Core3DRequestUUID(NSUUID.UUID,feature)) return nil;
        core3d::request::Part part;part.recipe=core3d::request::Recipe::Enclosure;
        part.schema=parameters.definition.constructionFrame?core3d::enclosure::FramedSchemaVersion:core3d::enclosure::SchemaVersion;
        if (!core3d::enclosure::Encode(parameters,part.values)) return nil;
        core3d::request::Descriptor descriptor;descriptor.operation=rebuild?core3d::request::Operation::RebuildEnclosure:core3d::request::Operation::CreateEnclosure;descriptor.parts={part};
        Core3DModelingPreparedRequest *request=[self core3d_prepareRequest:descriptor context:context session:session requestID:requestID featureIDs:(std::vector<core3d::request::UUID>{feature}) coverage:Core3DModelingEvidenceCoverageCanonicalEffect];
        if (request) request->_requestEnclosure=parameters;return request;
    } catch (...) {return nil;}
}

- (Core3DModelingPreparedRequest *)prepareProfileRequest:(Core3DProfileDefinition *)definition
    rebuild:(BOOL)rebuild context:(Core3DModelingPlanningContext *)context
    session:(Core3DModelingHostSession *)session requestID:(NSUUID *)requestID {
    if (![self isModelingPlanningContextCurrent:context]||!Core3DHostSessionCurrent(session)
        || ![definition isKindOfClass:Core3DProfileDefinition.class]) return nil;
    try {
        auto parameters=[definition nativeParameters];core3d::request::UUID feature;
        if (rebuild) {
            if (!context.selectedProfile) return nil;const auto original=[context.selectedProfile nativeSnapshot];
            const auto kind=Core3DCanonicalModelingProfileKind(original.parameters);
            if (!original.current||kind==0||Core3DCanonicalModelingProfileKind(parameters)!=kind
                || parameters.definition.plane!=original.parameters.definition.plane
                || parameters.metersPerUnit!=original.parameters.metersPerUnit
                || !Core3DRequestUUID([[NSUUID alloc] initWithUUIDString:context.selectedProfile.featureIdentifier],feature)) return nil;
            // Match the existing rebuild adapter: null/unchanged fields already
            // preserve exact raw values, and the native frame remains opening.
            parameters.constructionFrame=original.parameters.constructionFrame;
        } else if (parameters.metersPerUnit!=context.scene.metersPerUnit||!Core3DRequestUUID(NSUUID.UUID,feature)) return nil;
        core3d::request::Part part;part.recipe=core3d::request::Recipe::Profile;
        part.schema=core3d::profile::SchemaFor(parameters);
        if (!core3d::profile::Encode(parameters,part.values)) return nil;
        core3d::request::Descriptor descriptor;descriptor.operation=rebuild?core3d::request::Operation::RebuildProfile:core3d::request::Operation::CreateProfile;descriptor.parts={part};
        // Receipt schema has no CreateProfile operation yet. Never classify an
        // arbitrary/new profile as covered just because its outline is simple.
        const auto coverage=rebuild?Core3DModelingEvidenceCoverageCanonicalEffect:Core3DModelingEvidenceCoverageUnverifiedProfile;
        Core3DModelingPreparedRequest *request=[self core3d_prepareRequest:descriptor context:context session:session requestID:requestID featureIDs:(std::vector<core3d::request::UUID>{feature}) coverage:coverage];
        if (request) request->_requestProfile=parameters;return request;
    } catch (...) {return nil;}
}

- (Core3DModelingPreparedRequest *)prepareLoftStationRequest:(Core3DRectangularLoftStationEdit *)edit
    context:(Core3DModelingPlanningContext *)context session:(Core3DModelingHostSession *)session
    requestID:(NSUUID *)requestID {
    if(![self isModelingPlanningContextCurrent:context]||!Core3DHostSessionCurrent(session)
        ||![edit isKindOfClass:Core3DRectangularLoftStationEdit.class]||!context.selectedLoft)return nil;
    try {
        const auto original=[context.selectedLoft nativeSnapshot];
        const auto numeric=[edit nativeEdit];core3d::rectangular_loft::Definition candidate;
        core3d::request::Descriptor descriptor;core3d::receipt::UUID feature;
        if(!original.current||!core3d::loft_rebuild::Apply(original.definition,numeric,candidate)
            ||!Core3DModelingLoftSupported(candidate,original.effectiveDimensionMetersPerUnit)
            ||!core3d::receipt::LoftStationDescriptor(original.definition,numeric,descriptor)
            ||!core3d::receipt::ParseUUID(original.featureIdentifier,feature))return nil;
        Core3DModelingPreparedRequest *request=[self core3d_prepareRequest:descriptor context:context session:session
            requestID:requestID featureIDs:(std::vector<core3d::request::UUID>{feature})
            coverage:Core3DModelingEvidenceCoverageExactLoftEffect];
        if(request)request->_requestLoftEdit=numeric;return request;
    }catch(...){return nil;}
}

- (Core3DModelingPreparedRequest *)prepareAssemblyRequest:(NSArray<Core3DAssemblyPartDefinition *> *)parts
    context:(Core3DModelingPlanningContext *)context session:(Core3DModelingHostSession *)session requestID:(NSUUID *)requestID {
    if (![self isModelingPlanningContextCurrent:context]||!Core3DHostSessionCurrent(session)
        || ![parts isKindOfClass:NSArray.class]||parts.count<1||parts.count>16) return nil;
    try {
        core3d::request::Descriptor descriptor;descriptor.operation=core3d::request::Operation::CreateAssembly;
        std::vector<core3d::AssemblyPartDefinition> nativeParts;std::vector<core3d::request::UUID> featureIDs;
        NSMutableSet *names=[NSMutableSet set],*identifiers=[NSMutableSet set];
        for (Core3DAssemblyPartDefinition *part in [parts copy]) {
            if (![part isKindOfClass:Core3DAssemblyPartDefinition.class]||[names containsObject:part.name]
                ||[identifiers containsObject:part.partIdentifier]) return nil;
            const auto native=[part nativePartForMetersPerUnit:context.scene.metersPerUnit];core3d::request::UUID feature;
            core3d::request::Part encoded;encoded.recipe=core3d::request::Recipe::Profile;
            if (!native||!Core3DRequestUUID([[NSUUID alloc] initWithUUIDString:part.partIdentifier],feature)
                ||!Core3DRequestString(part.name,256,encoded.name)) return nil;
            encoded.schema=core3d::profile::SchemaFor(native->parameters);
            if (!core3d::profile::Encode(native->parameters,encoded.values)) return nil;
            descriptor.parts.push_back(std::move(encoded));nativeParts.push_back(*native);featureIDs.push_back(feature);
            [names addObject:part.name];[identifiers addObject:part.partIdentifier];
        }
        Core3DModelingPreparedRequest *request=[self core3d_prepareRequest:descriptor context:context session:session requestID:requestID featureIDs:featureIDs coverage:Core3DModelingEvidenceCoverageCanonicalEffect];
        if (request) request->_requestAssembly=std::move(nativeParts);return request;
    } catch (...) {return nil;}
}

- (BOOL)isModelingPreparedRequestCurrent:(Core3DModelingPreparedRequest *)request {
    return [NSThread isMainThread]&&[request isKindOfClass:Core3DModelingPreparedRequest.class]
        && request->_requestOwner==self && !request->_requestRetired && !request->_requestAdmitted
        && [_issuedModelingPreparedRequests containsObject:request]
        && Core3DHostSessionCurrent(request->_requestSession)
        && [self isModelingPlanningContextCurrent:request->_requestContext];
}
- (void)stopModelingPreparedRequest:(Core3DModelingPreparedRequest *)request {
    if (![NSThread isMainThread]||![request isKindOfClass:Core3DModelingPreparedRequest.class]
        ||request->_requestOwner!=self||![_issuedModelingPreparedRequests containsObject:request]) return;
    request->_requestRetired=YES;
#if DEBUG
    request->_requestAsyncGate=nil;request->_requestAsyncAfterStart=nil;request->_requestAsyncBeforeCompletion=nil;request->_requestAsyncGeometryDeliveryGate=nil;
#endif
    if(request->_requestEpoch)request->_requestEpoch->retired=true;
    [self retireModelingPlanningContext:request->_requestContext];
}
- (Core3DModelingRequestExecutionResult)executeModelingPreparedRequest:(Core3DModelingPreparedRequest *)request {
    // Public execution remains disabled until Stage C. This entry performs
    // no Store operation, geometry, callback, consume or receipt query.
    return Core3DModelingRequestExecutionResultUnavailable;
}


- (void)executeModelingPreparedRequest:(Core3DModelingPreparedRequest *)request
    completion:(void (^)(Core3DModelingAsyncOutcome *))completion {
    if(!completion)return;
    if(!NSThread.isMainThread){
        // Off-main invocation is rejected without touching native ownership or
        // reading supplied request fields. No controller enters this block.
        dispatch_async(dispatch_get_main_queue(),^{
            completion([[Core3DModelingAsyncOutcome alloc] initWithDisposition:Core3DModelingAsyncDispositionRejected
                storage:Core3DModelingStorageObservationNotAttempted request:nil document:nil entities:@[]]);
        });return;
    }
    Core3DModelingAsyncCompletion *box=[Core3DModelingAsyncCompletion new];
    box->_completion=[completion copy];box->_storage=Core3DModelingStorageObservationNotAttempted;
    if(![request isKindOfClass:Core3DModelingPreparedRequest.class]
        ||![self isModelingPreparedRequestCurrent:request]){
        Core3DFinishAsyncModeling(box,Core3DModelingAsyncDispositionRejected);return;
    }
    box->_owner=self;box->_prepared=request;box->_requestID=request.requestIdentifier;
    box->_documentID=request.documentIdentifier;box->_key=request->_requestKey;
    const auto operation=request->_requestDescriptor.operation;
    const bool loft=operation==core3d::request::Operation::RebuildLoftStation
        &&request.evidenceCoverage==Core3DModelingEvidenceCoverageExactLoftEffect;
    const bool creation=request.evidenceCoverage==Core3DModelingEvidenceCoverageCanonicalEffect
        &&(operation==core3d::request::Operation::CreateEnclosure||operation==core3d::request::Operation::CreateAssembly);
    const bool placement=operation==core3d::request::Operation::SetPlacement
        &&request.evidenceCoverage==Core3DModelingEvidenceCoverageExactPlacementEffect;
    if(!creation&&!loft&&!placement){
        Core3DFinishAsyncModeling(box,Core3DModelingAsyncDispositionUnsupported);return;
    }
#if DEBUG
    box->_beforeCompletion=request->_requestAsyncBeforeCompletion;request->_requestAsyncBeforeCompletion=nil;
    Core3DReservationTestConfig configuration;
    const BOOL privateStore=request->_requestAsyncConfigured&&request->_requestAsyncScenario>=0;
    if(privateStore){
        configuration.scenario=unsigned(request->_requestAsyncScenario);
        NSString *pattern=[NSTemporaryDirectory() stringByAppendingPathComponent:@"native-async-reservation-XXXXXX"];
        std::string path;if(!Core3DRequestString(pattern,configuration.parentPattern.size()-1,path)){
            Core3DFinishAsyncModeling(box,Core3DModelingAsyncDispositionRejected);return;
        }
        std::copy(path.begin(),path.end(),configuration.parentPattern.begin());
    }
#endif
    const auto work=[self core3d_registerReservation:request completion:
        ^(Core3DReservationOutcome outcome,const core3d::request::ReservationDelivery& delivery,BOOL known){
            box->_storage=Core3DAsyncStorage(delivery,known);
#if DEBUG
            Core3DModelingPreparedRequest *observed=box->_prepared;
            if(observed)observed->_requestAsyncStorageSnapshot=Core3DReservationDebugResult(outcome,delivery,known);
#endif
            if(outcome!=Core3DReservationOutcomeReservedForCoupling){
                Core3DFinishAsyncModeling(box,Core3DAsyncReservationDisposition(outcome));return;
            }
            Core3DViewController *owner=box->_owner;Core3DModelingPreparedRequest *prepared=box->_prepared;
            if(!owner||!prepared){Core3DFinishAsyncModeling(box,Core3DModelingAsyncDispositionRejected);return;}
#if DEBUG
            const BOOL failReceipt=prepared->_requestAsyncReceiptFailure;
            void (^afterStart)(void)=prepared->_requestAsyncAfterStart;prepared->_requestAsyncAfterStart=nil;
#endif
            if(placement){
#if DEBUG
                prepared->_requestAsyncAfterStart=afterStart;
#endif
                // The private shared route binds this exact resolution/entity
                // before any ordinary notification or synchronous completion.
                const auto result=[owner core3d_executeReservedPlacement:prepared
                    receiptFailure:
#if DEBUG
                    failReceipt
#else
                    NO
#endif
                    asyncCompletion:box];
                Core3DModelingAsyncDisposition disposition=Core3DModelingAsyncDispositionRejected;
                switch(result){
                    case Core3DTransformInspectorPositionCommitResultCommitted:disposition=Core3DModelingAsyncDispositionCommitted;break;
                    case Core3DTransformInspectorPositionCommitResultUnchanged:disposition=Core3DModelingAsyncDispositionUnchanged;break;
                    case Core3DTransformInspectorPositionCommitResultBusy:disposition=Core3DModelingAsyncDispositionBusy;break;
                    case Core3DTransformInspectorPositionCommitResultUnsupported:disposition=Core3DModelingAsyncDispositionUnsupported;break;
                    case Core3DTransformInspectorPositionCommitResultInternalFailure:disposition=Core3DModelingAsyncDispositionUncertain;break;
                    default:break;
                }
                Core3DFinishAsyncModeling(box,disposition);return;
            }
            void (^nativeCompletion)(Core3DProfileConstructionResult)=^(Core3DProfileConstructionResult result){
                Core3DFinishAsyncModeling(box,Core3DAsyncConstructionDisposition(result));
            };
            const BOOL started=loft
                ?[owner core3d_startReservedRebuild:prepared asyncCompletion:box completion:nativeCompletion]
                :[owner core3d_startReservedCreation:prepared completion:nativeCompletion];
            if(!loft&&!box->_finished&&prepared->_requestCommitPermit)
                box->_resolution=prepared->_requestCommitPermit->resolution();
            if(!started){
                [owner stopModelingPreparedRequest:prepared];
                Core3DFinishAsyncModeling(box,Core3DModelingAsyncDispositionRejected);return;
            }
#if DEBUG
            if(!box->_finished&&failReceipt)core3d::NativeModelingPermitIssuer::failReceiptStage(prepared);
            if(!box->_finished&&afterStart)afterStart();
#endif
        }];
    if(!work){Core3DFinishAsyncModeling(box,Core3DModelingAsyncDispositionBusy);return;}
#if DEBUG
    Core3DModelingReservationEntry *entry=Core3DReservationEntries[Core3DReservationIdentifier(work->deliveryID)];
    entry->_gate=request->_requestAsyncGate;request->_requestAsyncGate=nil;
    if(privateStore){
        const auto copied=*work;
        dispatch_async(dispatch_get_global_queue(QOS_CLASS_UTILITY,0),^{
            const auto delivery=Core3DRunPrivateReservation(copied,configuration);
            dispatch_async(dispatch_get_main_queue(),^{Core3DRouteReservationDelivery(delivery);});
        });return;
    }
#endif
    Core3DRunReservation(*work);
}

- (BOOL)core3d_startReservedCreation:(Core3DModelingPreparedRequest *)request
    completion:(void (^)(Core3DProfileConstructionResult))completion {
    if(!NSThread.isMainThread||!completion||!request
        ||!_reservedModelingCapability||_reservedModelingCapability->_taken
        ||_reservedModelingCapability->_entry->_prepared!=request
        ||request->_requestCommitPermit||_nativeSolidWork)return NO;
    if(![self core3d_reservationMatches:request]||!GLController||!GLController.viewer){
        [self stopModelingPreparedRequest:request];completion(Core3DProfileConstructionResultRejected);return YES;
    }
    try {
        const auto viewer=GLController.viewer;const auto document=viewer->getDocument();
        auto permit=core3d::NativeModelingPermitIssuer::take(_reservedModelingCapability,document);
        if(!permit){[self stopModelingPreparedRequest:request];completion(Core3DProfileConstructionResultRejected);return YES;}
        request->_requestCommitPermit=permit;
        _reservedModelingCapability=nil; // Moved authority cannot be reconstructed from the key.
        const auto context=request->_requestContext;
        const auto size=GLController.view.bounds.size;
        std::shared_ptr<core3d::NativeSolidWork> work;
        if(std::isfinite(size.width)&&std::isfinite(size.height)&&size.width>=1&&size.height>=1&&size.width<=16384&&size.height<=16384){
            core3d::ObjectFrameIdentity identity;
            identity.publicationSourceIdentifier=context.scene.publicationSourceIdentifier.UTF8String;
            identity.documentGeneration=context.scene.revisions.documentGeneration;
            identity.modelRevision=context.scene.revisions.modelRevision;
            if(request->_requestDescriptor.operation==core3d::request::Operation::CreateEnclosure&&request->_requestEnclosure)
                work=viewer->prepareEnclosureSolid(*request->_requestEnclosure,identity,context.scene.revisions.presentationRevision,
                    std::uint32_t(std::llround(size.width)),std::uint32_t(std::llround(size.height)));
            else if(request->_requestDescriptor.operation==core3d::request::Operation::CreateAssembly)
                work=viewer->prepareAssemblySolid(request->_requestAssembly,identity,context.scene.revisions.presentationRevision,
                    std::uint32_t(std::llround(size.width)),std::uint32_t(std::llround(size.height)));
        }
        // Preparation may touch native selection bookkeeping. Never replace the
        // original lease with a freshly captured scene after it does so.
        if(!work||![self core3d_reservationMatches:request]
            ||!core3d::Core3DViewer::attachModelingCreationPermit(work,permit)){
            [self stopModelingPreparedRequest:request];completion(Core3DProfileConstructionResultRejected);return YES;
        }
        _modelingConstructionContext=context;
        __weak Core3DViewController *weakOwner=self;
        __weak Core3DModelingPreparedRequest *weakRequest=request;
        __weak Core3DModelingPlanningContext *weakContext=context;
        // runNativeSolidWork stores this callback in its main-only registry.
        // Its utility block still captures geometry and weak native owners only.
#if DEBUG
        auto geometryDeliveryGate=request->_requestAsyncGeometryDeliveryGate;
        request->_requestAsyncGeometryDeliveryGate=nil;
        [self runNativeSolidWork:work debugGeometryDeliveryGate:geometryDeliveryGate completion:^(Core3DProfileConstructionResult result){
#else
        [self runNativeSolidWork:work completion:^(Core3DProfileConstructionResult result){
#endif
            Core3DViewController *owner=weakOwner;Core3DModelingPreparedRequest *issued=weakRequest;
            // Stop may release the last request before this late completion.
            // The owner's occupied context slot independently keeps this weak
            // identity alive; clear only that exact slot, never a newer context.
            Core3DModelingPlanningContext *finishedContext=weakContext;
            if(owner&&owner->_modelingConstructionContext==finishedContext)
                owner->_modelingConstructionContext=nil;
            if(owner&&issued){
                [owner core3d_releaseReservation:issued];issued->_requestRetired=YES;
                issued->_requestContext->_planningRetired=YES;
            }
            completion(result);
        }];
        return YES;
    }catch(...){[self stopModelingPreparedRequest:request];return NO;}
}

- (BOOL)core3d_startReservedRebuild:(Core3DModelingPreparedRequest *)request
    completion:(void (^)(Core3DProfileConstructionResult))completion {
    return [self core3d_startReservedRebuild:request asyncCompletion:nil completion:completion];
}
- (BOOL)core3d_startReservedRebuild:(Core3DModelingPreparedRequest *)request
    asyncCompletion:(Core3DModelingAsyncCompletion *)box completion:(void (^)(Core3DProfileConstructionResult))completion {
    if(!NSThread.isMainThread||!completion||!request
        ||!_reservedModelingCapability||_reservedModelingCapability->_taken
        ||_reservedModelingCapability->_entry->_prepared!=request
        ||request->_requestCommitPermit||_nativeSolidWork)return NO;
    if(![self core3d_reservationMatches:request]||!GLController||!GLController.viewer){
        [self stopModelingPreparedRequest:request];completion(Core3DProfileConstructionResultRejected);return YES;
    }
    try {
        const auto viewer=GLController.viewer;const auto document=viewer->getDocument();
        auto permit=core3d::NativeModelingPermitIssuer::takeRebuild(_reservedModelingCapability,document);
        if(!permit){[self stopModelingPreparedRequest:request];completion(Core3DProfileConstructionResultRejected);return YES;}
        request->_requestCommitPermit=permit;
        if(box&&!core3d::NativeModelingPermitIssuer::bindAsyncLoft(box,request,permit)){
            [self stopModelingPreparedRequest:request];completion(Core3DProfileConstructionResultRejected);return YES;
        }
        _reservedModelingCapability=nil; // Moved authority cannot be reconstructed from the key.
        const auto context=request->_requestContext;
        const auto size=GLController.view.bounds.size;
        std::shared_ptr<core3d::NativeSolidWork> work;
        if(std::isfinite(size.width)&&std::isfinite(size.height)&&size.width>=1&&size.height>=1&&size.width<=16384&&size.height<=16384){
            core3d::ObjectFrameIdentity identity;
            identity.publicationSourceIdentifier=context.scene.publicationSourceIdentifier.UTF8String;
            identity.documentGeneration=context.scene.revisions.documentGeneration;
            identity.modelRevision=context.scene.revisions.modelRevision;
            if(request->_requestDescriptor.operation==core3d::request::Operation::RebuildEnclosure
                &&request->_requestEnclosure&&context.selectedEnclosure){
                const auto original=[context.selectedEnclosure nativeSnapshot]; identity.entityIdentifier=original.identity.entityIdentifier;
                work=viewer->prepareStoredEnclosureRebuild(*request->_requestEnclosure,original,identity,
                    context.scene.revisions.presentationRevision,std::uint32_t(std::llround(size.width)),std::uint32_t(std::llround(size.height)));
            }else if(request->_requestDescriptor.operation==core3d::request::Operation::RebuildProfile
                &&request->_requestProfile&&context.selectedProfile){
                const auto original=[context.selectedProfile nativeSnapshot]; identity.entityIdentifier=original.identity.entityIdentifier;
                work=viewer->prepareStoredProfileRebuild(*request->_requestProfile,original,identity,
                    context.scene.revisions.presentationRevision,std::uint32_t(std::llround(size.width)),std::uint32_t(std::llround(size.height)));
            }else if(request->_requestDescriptor.operation==core3d::request::Operation::RebuildLoftStation
                &&request->_requestLoftEdit&&context.selectedLoft){
                const auto original=[context.selectedLoft nativeSnapshot];identity.entityIdentifier=original.identity.entityIdentifier;
                work=viewer->prepareStoredLoftStationRebuild(*request->_requestLoftEdit,original,identity,
                    context.scene.revisions.presentationRevision,std::uint32_t(std::llround(size.width)),std::uint32_t(std::llround(size.height)));
            }
        }
        // Preparation may touch native selection bookkeeping. Never replace the
        // original lease with a freshly captured scene after it does so.
        if(!work||![self core3d_reservationMatches:request]
            ||!core3d::Core3DViewer::attachModelingRebuildPermit(work,permit)){
            [self stopModelingPreparedRequest:request];completion(Core3DProfileConstructionResultRejected);return YES;
        }
        _modelingConstructionContext=context;
        __weak Core3DViewController *weakOwner=self;
        __weak Core3DModelingPreparedRequest *weakRequest=request;
        __weak Core3DModelingPlanningContext *weakContext=context;
        // runNativeSolidWork stores this callback in its main-only registry.
        // Its utility block still captures geometry and weak native owners only.
        [self runNativeSolidWork:work completion:^(Core3DProfileConstructionResult result){
            Core3DViewController *owner=weakOwner;Core3DModelingPreparedRequest *issued=weakRequest;
            // Stop may release the last request before this late completion.
            // The owner's occupied context slot independently keeps this weak
            // identity alive; clear only that exact slot, never a newer context.
            Core3DModelingPlanningContext *finishedContext=weakContext;
            if(owner&&owner->_modelingConstructionContext==finishedContext)
                owner->_modelingConstructionContext=nil;
            if(owner&&issued){
                [owner core3d_releaseReservation:issued];issued->_requestRetired=YES;
                issued->_requestContext->_planningRetired=YES;
            }
            completion(result);
        }];
        return YES;
    }catch(...){[self stopModelingPreparedRequest:request];return NO;}
}

- (Core3DModelingPlanningContext *)captureModelingPlanningContext {
    if (![self core3d_canCaptureModelingContext]) return nil;
    try {
        const auto viewer = GLController.viewer;
        const auto document = viewer->getDocument();
        if (document.IsNull()) return nil;
        const auto stamp = document->CaptureNativePlanningStamp(viewer->canBeginCommittedEdit());
        Core3DSceneSnapshot *scene = [self captureSceneSnapshot];
        Core3DScenePresentationOverlaySnapshot *overlay = [self captureScenePresentationOverlay];
        if (!stamp || !scene || !overlay || scene.selectionMode != Core3DSceneElementKindObject
            || overlay.kind != Core3DScenePresentationOverlayKindNone
            || !std::isfinite(scene.metersPerUnit) || scene.metersPerUnit <= 0) return nil;
        NSString *documentID = [[NSString alloc] initWithUTF8String:document->DocumentIdentifier().c_str()];
        if (documentID.length == 0 || documentID.length > 128) return nil;
        Core3DStoredEnclosureSnapshot *enclosure = nil;
        Core3DStoredProfileSnapshot *profile = nil;
        Core3DStoredProfileSnapshot *recipe = nil;
        Core3DStoredSweepSnapshot *sweep = nil;
        Core3DStoredRectangularLoftSnapshot *loft = nil;
        if (scene.selection.selectedElements.count == 1) {
            Core3DSceneElementIdentifier *selected = scene.selection.selectedElements.firstObject;
            if (selected.kind == Core3DSceneElementKindObject)
                enclosure = [self storedEnclosureWithEntityIdentifier:selected.entityIdentifier expected:scene];
            if (enclosure && !enclosure.current) enclosure = nil;
            if (!enclosure && selected.kind == Core3DSceneElementKindObject) {
                profile = [self storedProfileWithEntityIdentifier:selected.entityIdentifier expected:scene];
                if (profile && (!profile.current || !std::isfinite(profile.dimensionMetersPerUnit)
                    || profile.dimensionMetersPerUnit <= 0)) profile = nil;
                if (profile && Core3DCanonicalModelingProfileKind([profile nativeSnapshot].parameters) == 0) {
                    if (Core3DModelingProfileRecipeSupported([profile nativeSnapshot].parameters,
                        profile.dimensionMetersPerUnit)) recipe = profile;
                    profile = nil;
                }
            }
            if(!enclosure&&!profile&&!recipe&&selected.kind==Core3DSceneElementKindObject){
                sweep=[self storedSweepWithEntityIdentifier:selected.entityIdentifier expected:scene];
                if(sweep&&(!sweep.current||!Core3DModelingSweepSupported([sweep nativeSnapshot].definition,
                    sweep.effectiveDimensionMetersPerUnit)))sweep=nil;
            }
            if(!enclosure&&!profile&&!recipe&&!sweep&&selected.kind==Core3DSceneElementKindObject){
                loft=[self storedRectangularLoftWithEntityIdentifier:selected.entityIdentifier expected:scene];
                if(loft&&(!loft.current||!Core3DModelingLoftSupported([loft nativeSnapshot].definition,
                    loft.effectiveDimensionMetersPerUnit)))loft=nil;
            }
        }
        // Native reads must not acquire or refresh authority during capture.
        const auto after = document->CaptureNativePlanningStamp(viewer->canBeginCommittedEdit());
        if (!after || !(*after == *stamp)) return nil;
        Core3DModelingPlanningContext *context = [[Core3DModelingPlanningContext alloc]
            initWithScene:scene documentIdentifier:documentID enclosure:enclosure profile:profile recipe:recipe sweep:sweep loft:loft];
        context->_planningOwner = self; context->_planningViewer = viewer;
        context->_planningStamp = *stamp; context->_planningOverlayRevision = overlay.overlayRevision;
        Core3DModelingPlanningContext *previous = _issuedModelingPlanningContext;
        if (previous) previous->_planningRetired = YES;
        _issuedModelingPlanningContext = context;
        return context;
    } catch (...) { return nil; }
}

- (Core3DModelingPlanningContext *)captureModelingPlanningContextIncludingSavedCutSource {
    Core3DModelingPlanningContext *context=[self captureModelingPlanningContext];
    if(!context||context.scene.selection.selectedElements.count!=1
        ||context.selectedEnclosure||context.selectedProfile||context.selectedProfileRecipe
        ||context.selectedSweep||context.selectedLoft)return context;
    try {
        Core3DSceneElementIdentifier *selected=context.scene.selection.selectedElements.firstObject;
        if(selected.kind!=Core3DSceneElementKindObject)return context;
        Core3DCylindricalCutSnapshot *source=[self cylindricalCutSourceWithEntityIdentifier:selected.entityIdentifier expected:context.scene];
        Core3DCylindricalCutProgramSnapshot *program=nil;
        Core3DSavedCutSourceValues *values=nil;
        if(source&&source.rebuilding&&(values=source.sourceRecipeMM)){
            const auto original=[source nativeSnapshot];
            if(core3d::saved_cut_bore_clearance::Inspect(original.source.envelope).status
                !=core3d::saved_cut_bore_clearance::Status::ClearRecipeDisk)return context;
        }else{
            source=nil;
            program=[self cylindricalCutProgramWithEntityIdentifier:selected.entityIdentifier expected:context.scene];
            // A program context retains the complete native aggregate. Bounded
            // descriptive bores are projected only after every-bore source
            // support has been verified by sourceRecipeMM.
            if(!program||program.bores.count<2||program.bores.count>4||(values=program.sourceRecipeMM)==nil)return context;
        }
        // All added reads are enclosed by the original stamp and this exact
        // current-context check. No second snapshot replaces its authority.
        if(![self isModelingPlanningContextCurrent:context]){
            [self retireModelingPlanningContext:context];return nil;
        }
        context->_planningSavedCutSource=source;context->_planningSavedCutProgramSource=program;
        context->_planningSavedCutValues=values;
        return context;
    }catch(...){[self retireModelingPlanningContext:context];return nil;}
}

- (Core3DSavedCutSourceOperation *)beginModelingSavedCutSourceEdit:(Core3DSavedCutSourcePatch *)patch
    context:(Core3DModelingPlanningContext *)context completion:(void(^)(Core3DProfileConstructionResult))completion {
    if(!completion)return nil;
    if(!NSThread.isMainThread){dispatch_async(dispatch_get_main_queue(),^{completion(Core3DProfileConstructionResultRejected);});return nil;}
    // Preserve validation-before-dereference for wrong-class, foreign and
    // stale contexts. Only this owner's current native context may expose its
    // mutually exclusive opaque target properties below.
    if(![self isModelingPlanningContextCurrent:context]){
        completion(Core3DProfileConstructionResultRejected);return nil;
    }
    const BOOL legacy=context.selectedSavedCutSource!=nil;
    const BOOL program=context.selectedSavedCutProgramSource!=nil;
    if(legacy==program
        ||!context.selectedSavedCutSourceRecipeMM||context.selectedEnclosure||context.selectedProfile
        ||context.selectedProfileRecipe||context.selectedSweep||context.selectedLoft
        ||![patch isMemberOfClass:Core3DSavedCutSourcePatch.class]){
        completion(Core3DProfileConstructionResultRejected);return nil;
    }
    context->_planningConsumed=YES;
    __weak Core3DModelingPlanningContext *weakContext=context;
    void (^finished)(Core3DProfileConstructionResult)=^(Core3DProfileConstructionResult result){
            Core3DModelingPlanningContext *finished=weakContext;
            if(finished)finished->_planningRetired=YES;
            completion(result);
        };
    return legacy
        ?[self core3d_beginSavedCutSourceEdit:context.selectedSavedCutSource patch:patch expected:context.scene
            planningContext:context completion:finished]
        :[self core3d_beginSavedProgramCutSourceEdit:context.selectedSavedCutProgramSource patch:patch expected:context.scene
            planningContext:context completion:finished];
}

- (Core3DModelingPlanningContext *)captureModelingPlacementContext {
    Core3DModelingPlanningContext *context=[self captureModelingPlanningContext];
    if(!context)return nil;
    try{
        Core3DTransformInspectorSnapshot *snapshot=[self requestTransformInspectorSnapshotWithCompletion:nil];
        const auto document=GLController.viewer->getDocument();TDF_Label label;core3d::placement::Evidence evidence;
        if(context.scene.selection.selectedElements.count!=1||!snapshot.canEditPosition||!snapshot.presentationMatchesDocument
            ||snapshot.representation!=Core3DTransformInspectorRepresentationBRep
            ||!snapshot.positionEditGeneration||!snapshot.documentEditGeneration||!snapshot.geometryEditGeneration
            ||![snapshot.entityIdentifier isEqualToString:context.scene.selection.selectedElements.firstObject.entityIdentifier]
            ||!snapshot.entityIdentifier.UTF8String||!snapshot.definitionIdentifier.UTF8String
            ||!core3d::placement::Find(document,snapshot.entityIdentifier.UTF8String,label)
            ||!core3d::placement::Capture(document,label,evidence)
            ||document->DefinitionIdentifierForLabel(label)!=snapshot.definitionIdentifier.UTF8String
            ||core3d::sweep_persistence::Bits(snapshot.metersPerUnit)!=core3d::sweep_persistence::Bits(context.scene.metersPerUnit)
            ||![self isModelingPlanningContextCurrent:context]){
            [self retireModelingPlanningContext:context];return nil;
        }
        context->_planningPlacementSnapshot=snapshot;context->_planningPlacementEvidence=std::move(evidence);return context;
    }catch(...){[self retireModelingPlanningContext:context];return nil;}
}
- (Core3DModelingPreparedRequest *)preparePlacementRequestWithValue:(double)value
    kind:(Core3DRigidPlacementKind)kind axis:(Core3DTransformInspectorAxis)axis
    context:(Core3DModelingPlanningContext *)context session:(Core3DModelingHostSession *)session requestID:(NSUUID *)requestID {
    if(![self isModelingPlanningContextCurrent:context]||!Core3DHostSessionCurrent(session)
        ||!context->_planningPlacementSnapshot||!context->_planningPlacementEvidence)return nil;
    try{
        auto prepared=[self prepareRigidPlacementValue:value kind:kind axis:axis expected:context->_planningPlacementSnapshot];
        if(!prepared||!(prepared->_placementEvidence==*context->_planningPlacementEvidence)
            ||prepared->_placementViewer.lock()!=context->_planningViewer.lock()
            ||!(prepared->_placementStamp==context->_planningStamp))return nil;
        core3d::request::Descriptor descriptor;
        const auto nativeAxis=axis==Core3DTransformInspectorAxisX?0:axis==Core3DTransformInspectorAxisY?1:2;
        if(!core3d::placement::Descriptor(prepared->_placementEvidence,context.scene.metersPerUnit,
            kind==Core3DRigidPlacementKindRotation?1:0,nativeAxis,prepared->_placementRawValue,descriptor))return nil;
        auto request=[self core3d_prepareRequest:descriptor context:context session:session requestID:requestID
            featureIDs:(std::vector<core3d::request::UUID>{}) coverage:Core3DModelingEvidenceCoverageExactPlacementEffect];
        if(request)request->_requestPlacement=prepared;return request;
    }catch(...){return nil;}
}
- (Core3DTransformInspectorPositionCommitResult)core3d_executeReservedPlacement:(Core3DModelingPreparedRequest *)request
    receiptFailure:(BOOL)receiptFailure {
    return [self core3d_executeReservedPlacement:request receiptFailure:receiptFailure asyncCompletion:nil];
}
- (Core3DTransformInspectorPositionCommitResult)core3d_executeReservedPlacement:(Core3DModelingPreparedRequest *)request
    receiptFailure:(BOOL)receiptFailure asyncCompletion:(Core3DModelingAsyncCompletion *)box {
    if(!NSThread.isMainThread||![request isKindOfClass:Core3DModelingPreparedRequest.class]
        ||request->_requestOwner!=self||!_reservedModelingCapability||_reservedModelingCapability->_taken
        ||_reservedModelingCapability->_entry->_prepared!=request||request->_requestCommitPermit
        ||!request->_requestPlacement||!GLController||!GLController.viewer)return Core3DTransformInspectorPositionCommitResultStale;
    try{
        if(![self core3d_reservationMatches:request]){[self stopModelingPreparedRequest:request];return Core3DTransformInspectorPositionCommitResultStale;}
        if(box&&(box->_finished||box->_owner!=self||box->_prepared!=request
            ||box->_storage!=Core3DModelingStorageObservationReserved||!(box->_key.accountScope==request->_requestKey.accountScope&&box->_key.document==request->_requestKey.document
                &&box->_key.request==request->_requestKey.request&&box->_key.command==request->_requestKey.command
                &&box->_key.execution==request->_requestKey.execution))){
            [self stopModelingPreparedRequest:request];return Core3DTransformInspectorPositionCommitResultStale;
        }
        auto permit=core3d::NativeModelingPermitIssuer::takePlacement(_reservedModelingCapability,GLController.viewer->getDocument());
        if(!permit){[self stopModelingPreparedRequest:request];return Core3DTransformInspectorPositionCommitResultStale;}
        request->_requestCommitPermit=permit;_reservedModelingCapability=nil;
        if(box){box->_resolution=permit->resolution();box->_placementResolutionBound=YES;
            box->_placementEntity=request->_requestDescriptor.placement->entity;}
#if DEBUG
        if(receiptFailure)core3d::NativeModelingPermitIssuer::failReceiptStage(request);
        if(box){auto afterStart=request->_requestAsyncAfterStart;request->_requestAsyncAfterStart=nil;if(afterStart)afterStart();}
#endif
        auto prepared=request->_requestPlacement;auto snapshot=prepared->_placementSnapshot;
        prepared->_placementConsumed=YES;prepared->_placementSnapshot=nil;
        // Synchronous main-only shared inspector path. The original request is
        // retained on this stack through publication; resolution seals before callbacks.
        const auto result=[self commitTransformInspectorValue:prepared->_placementRawValue axis:prepared->_placementAxis
            expectedSnapshot:snapshot kind:prepared->_placementKind==Core3DRigidPlacementKindPosition
                ?core3d::TransformInspectorEditKind::Position:core3d::TransformInspectorEditKind::Rotation
            placementPermit:std::move(permit)];
        [self core3d_releaseReservation:request];request->_requestRetired=YES;request->_requestContext->_planningRetired=YES;
        // Epoch lifetime stays in the unresolved ledger when recovery owns it.
        return result;
    }catch(...){[self stopModelingPreparedRequest:request];return Core3DTransformInspectorPositionCommitResultInternalFailure;}
}
- (BOOL)core3d_modelingContext:(Core3DModelingPlanningContext *)context
    matchesAllowingConsumed:(BOOL)allowConsumed {
    if (![NSThread isMainThread] || ![context isKindOfClass:Core3DModelingPlanningContext.class]
        || context != _issuedModelingPlanningContext || context->_planningOwner != self
        || context->_planningRetired || (!allowConsumed && context->_planningConsumed)
        || ![self core3d_canCaptureModelingContextForReservation:allowConsumed?context:nil]) return NO;
    try {
        const auto viewer = context->_planningViewer.lock();
        if (!viewer || viewer != GLController.viewer) return NO;
        const auto document = viewer->getDocument();
        if (document.IsNull()) return NO;
        const auto stamp = document->CaptureNativePlanningStamp(viewer->canBeginCommittedEdit());
        if (!stamp || !(*stamp == context->_planningStamp)) return NO;
        NSString *documentID = [[NSString alloc] initWithUTF8String:document->DocumentIdentifier().c_str()];
        if (![documentID isEqualToString:context.documentIdentifier]) return NO;
        Core3DSceneSnapshot *live = [self captureSceneSnapshot];
        Core3DScenePresentationOverlaySnapshot *overlay = [self captureScenePresentationOverlay];
        Core3DSceneSnapshot *expected = context.scene;
        if (!live || !overlay || overlay.kind != Core3DScenePresentationOverlayKindNone
            || overlay.overlayRevision != context->_planningOverlayRevision
            || live.selectionMode != expected.selectionMode
            || ![live.publicationSourceIdentifier isEqualToString:expected.publicationSourceIdentifier]
            || live.revisions.documentGeneration != expected.revisions.documentGeneration
            || live.revisions.modelRevision != expected.revisions.modelRevision
            || live.revisions.presentationRevision != expected.revisions.presentationRevision
            || live.metersPerUnit != expected.metersPerUnit
            || live.selection.selectedElements.count != expected.selection.selectedElements.count) return NO;
        for (NSUInteger i = 0; i < live.selection.selectedElements.count; ++i) {
            Core3DSceneElementIdentifier *a = live.selection.selectedElements[i];
            Core3DSceneElementIdentifier *b = expected.selection.selectedElements[i];
            if (![a.entityIdentifier isEqualToString:b.entityIdentifier] || a.kind != b.kind
                || a.topologyIndex != b.topologyIndex || a.geometryRevision != b.geometryRevision) return NO;
        }
        const auto after = document->CaptureNativePlanningStamp(viewer->canBeginCommittedEdit());
        return after && *after == context->_planningStamp;
    } catch (...) { return NO; }
}

- (BOOL)isModelingPlanningContextCurrent:(Core3DModelingPlanningContext *)context {
    return [self core3d_modelingContext:context matchesAllowingConsumed:NO];
}

- (void)retireModelingPlanningContext:(Core3DModelingPlanningContext *)context {
    if (![NSThread isMainThread] || ![context isKindOfClass:Core3DModelingPlanningContext.class]
        || context->_planningOwner != self) return;
    [self core3d_retireReservationContext:context];
    Core3DSavedCutSourceJob *sourceJob=context->_planningSavedCutJob;
    if(sourceJob&&sourceJob->_planningContext==context)[sourceJob cancel];
    context->_planningRetired = YES;
    if (_modelingConstructionContext == context && _nativeSolidWork)
        [self cancelNativeConstruction];
}

- (void)core3d_executeEnclosure:(Core3DEnclosureDefinition *)definition
    context:(Core3DModelingPlanningContext *)context rebuild:(BOOL)rebuild
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) return;
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ completion(Core3DProfileConstructionResultRejected); });
        return;
    }
    if (![self isModelingPlanningContextCurrent:context]
        || ![definition isKindOfClass:Core3DEnclosureDefinition.class]
        || (rebuild && !context.selectedEnclosure)) {
        completion(Core3DProfileConstructionResultRejected); return;
    }
    // Consume before invoking native admission. A failed attempt cannot be
    // silently replayed with refreshed permission; request a new plan/context.
    context->_planningConsumed = YES;
    _modelingConstructionContext = context;
    __weak Core3DViewController *weakSelf = self;
    __weak Core3DModelingPlanningContext *weakContext = context;
    void (^finish)(Core3DProfileConstructionResult) = ^(Core3DProfileConstructionResult result) {
        Core3DViewController *owner = weakSelf;
        Core3DModelingPlanningContext *finishedContext = weakContext;
        if (finishedContext) finishedContext->_planningRetired = YES;
        if (owner && owner->_modelingConstructionContext == finishedContext)
            owner->_modelingConstructionContext = nil;
        completion(result);
    };
    if (rebuild) [self rebuildStoredEnclosure:context.selectedEnclosure definition:definition
        expected:context.scene completion:finish];
    else [self createEnclosureWithDefinition:definition expected:context.scene completion:finish];
}

- (void)createEnclosureWithDefinition:(Core3DEnclosureDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    [self core3d_executeEnclosure:definition context:context rebuild:NO completion:completion];
}

- (void)createProfileWithDefinition:(Core3DProfileDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) return;
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{completion(Core3DProfileConstructionResultRejected);});
        return;
    }
    if (![self isModelingPlanningContextCurrent:context]
        || ![definition isKindOfClass:Core3DProfileDefinition.class]
        || definition.metersPerUnit != context.scene.metersPerUnit) {
        completion(Core3DProfileConstructionResultRejected); return;
    }
    // The existing typed creator copies validated numeric profile/curve values
    // into its private worker. Consume the original lease before that admission;
    // failure never reconstructs authority from a serialized scene or plan.
    context->_planningConsumed = YES;
    _modelingConstructionContext = context;
    __weak Core3DViewController *weakSelf = self;
    __weak Core3DModelingPlanningContext *weakContext = context;
    [self createProfileWithDefinition:definition expected:context.scene
        completion:^(Core3DProfileConstructionResult result) {
            Core3DViewController *owner = weakSelf;
            Core3DModelingPlanningContext *finished = weakContext;
            if (finished) finished->_planningRetired = YES;
            if (owner && owner->_modelingConstructionContext == finished)
                owner->_modelingConstructionContext = nil;
            completion(result);
        }];
}

- (void)rebuildEnclosureWithDefinition:(Core3DEnclosureDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    [self core3d_executeEnclosure:definition context:context rebuild:YES completion:completion];
}

- (void)rebuildProfileWithDefinition:(Core3DProfileDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) return;
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ completion(Core3DProfileConstructionResultRejected); }); return;
    }
    if (![self isModelingPlanningContextCurrent:context] || !context.selectedProfile
        || ![definition isKindOfClass:Core3DProfileDefinition.class]) {
        completion(Core3DProfileConstructionResultRejected); return;
    }
    try {
        const auto original=[context.selectedProfile nativeSnapshot];
        const auto requested=[definition nativeParameters];
        const auto kind=Core3DCanonicalModelingProfileKind(original.parameters);
        if (!original.current || original.dimensionMetersPerUnit <= 0 || kind == 0
            || Core3DCanonicalModelingProfileKind(requested) != kind
            || requested.definition.plane != original.parameters.definition.plane
            || requested.metersPerUnit != original.parameters.metersPerUnit) {
            completion(Core3DProfileConstructionResultRejected); return;
        }
        context->_planningConsumed=YES; _modelingConstructionContext=context;
        __weak Core3DViewController *weakSelf=self;
        __weak Core3DModelingPlanningContext *weakContext=context;
        // Ordinary rebuild copies the exact opening construction frame; its
        // native transform ledger retains authored placement and stable IDs.
        [self rebuildStoredProfile:context.selectedProfile definition:definition expected:context.scene
            completion:^(Core3DProfileConstructionResult result) {
                Core3DViewController *owner=weakSelf;
                Core3DModelingPlanningContext *finished=weakContext;
                if (finished) finished->_planningRetired=YES;
                if (owner && owner->_modelingConstructionContext==finished) owner->_modelingConstructionContext=nil;
                completion(result);
            }];
    } catch (...) { completion(Core3DProfileConstructionResultRejected); }
}

- (void)createSweepWithDefinition:(Core3DSweepDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if(!completion)return;
    if(!NSThread.isMainThread){dispatch_async(dispatch_get_main_queue(),^{completion(Core3DProfileConstructionResultRejected);});return;}
    if(![self isModelingPlanningContextCurrent:context]||![definition isMemberOfClass:Core3DSweepDefinition.class]
        ||core3d::sweep_persistence::Bits(definition.metersPerUnit)!=core3d::sweep_persistence::Bits(context.scene.metersPerUnit)){
        completion(Core3DProfileConstructionResultRejected);return;
    }
    context->_planningConsumed=YES;_modelingConstructionContext=context;
    __weak Core3DViewController *weakSelf=self;
    __weak Core3DModelingPlanningContext *weakContext=context;
    [self createSweepWithDefinition:definition expected:context.scene completion:^(Core3DProfileConstructionResult result){
        Core3DViewController *owner=weakSelf;Core3DModelingPlanningContext *finished=weakContext;
        if(finished)finished->_planningRetired=YES;
        if(owner&&owner->_modelingConstructionContext==finished)owner->_modelingConstructionContext=nil;
        completion(result);
    }];
}

- (Core3DRectangularLoftStationEdit *)modelingLoftStationEdit:(uint32_t)stationIdentifier
    widthMM:(NSNumber *)width depthMM:(NSNumber *)depth context:(Core3DModelingPlanningContext *)context {
    if(![self isModelingPlanningContextCurrent:context]||!context.selectedLoft||!stationIdentifier
        ||context.selectedEnclosure||context.selectedProfile||context.selectedProfileRecipe||context.selectedSweep
        ||(!width&&!depth))return nil;
    try {
        const auto source=[context.selectedLoft nativeSnapshot];
        if(!source.current||!Core3DModelingLoftSupported(source.definition,source.effectiveDimensionMetersPerUnit))return nil;
        const core3d::rectangular_loft::Station *station=nullptr;
        for(const auto& value:source.definition.stations)if(value.identifier==stationIdentifier){
            if(station)return nil;station=&value;
        }
        if(!station)return nil;
        const double factor=source.effectiveDimensionMetersPerUnit*1000;
        auto convert=[&](NSNumber *value,double original,NSNumber *__strong *result){
            *result=nil;if(!value)return true;
            if(![value isKindOfClass:NSNumber.class]||CFGetTypeID((__bridge CFTypeRef)value)==CFBooleanGetTypeID())return false;
            const double physical=value.doubleValue;
            if(!std::isfinite(physical)||physical<0.001||physical>1e6)return false;
            // Preserve exact native bits when the requested physical Double is
            // the same as the advertised value; never round-trip a no-op.
            const double raw=physical==original*factor?original:physical/factor;
            if(!std::isfinite(raw)||raw<=0)return false;
            *result=@(raw);return true;
        };
        NSNumber *nativeWidth=nil,*nativeDepth=nil;
        if(!convert(width,station->width,&nativeWidth)||!convert(depth,station->depth,&nativeDepth))return nil;
        Core3DRectangularLoftStationEdit *edit=[[Core3DRectangularLoftStationEdit alloc]
            initWithStationIdentifier:stationIdentifier width:nativeWidth depth:nativeDepth];
        if(!edit)return nil;
        core3d::rectangular_loft::Definition candidate;
        if(!core3d::loft_rebuild::Apply(source.definition,[edit nativeEdit],candidate)
            ||!Core3DModelingLoftSupported(candidate,source.effectiveDimensionMetersPerUnit)
            ||![self isModelingPlanningContextCurrent:context])return nil;
        // Numeric preparation never consumes or replaces the original lease.
        return edit;
    }catch(...){return nil;}
}

- (void)rebuildSweepRadiusWithDefinition:(Core3DSweepDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if(!completion)return;
    if(!NSThread.isMainThread){dispatch_async(dispatch_get_main_queue(),^{completion(Core3DProfileConstructionResultRejected);});return;}
    if(![self isModelingPlanningContextCurrent:context]||!context.selectedSweep||context.selectedEnclosure
        ||context.selectedProfile||context.selectedProfileRecipe||![definition isMemberOfClass:Core3DSweepDefinition.class]){
        completion(Core3DProfileConstructionResultRejected);return;
    }
    try {
        const auto original=[context.selectedSweep nativeSnapshot];auto requested=[definition nativeDefinition];
        if(!original.current||!Core3DModelingSweepSupported(original.definition,original.effectiveDimensionMetersPerUnit)
            ||!Core3DModelingSweepSupported(requested,original.effectiveDimensionMetersPerUnit)){
            completion(Core3DProfileConstructionResultRejected);return;
        }
        // Radius is the only admitted delta, including signed-zero and optional
        // presence bits elsewhere. Neither JSON nor a new scene can refresh it.
        auto expected=original.definition;expected.radius=requested.radius;
        std::vector<double> expectedValues,requestedValues;
        if(!core3d::sweep_persistence::Encode(expected,expectedValues)
            ||!core3d::sweep_persistence::Encode(requested,requestedValues)
            ||!core3d::sweep_persistence::SameBits(expectedValues,requestedValues)){
            completion(Core3DProfileConstructionResultRejected);return;
        }
        const double factor=original.effectiveDimensionMetersPerUnit*1000;
        // If both raw radii advertise the exact same finite physical Double,
        // retain original raw bits before ordinary semantic no-change admission.
        Core3DSweepDefinition *frozen=definition;
        if(requested.radius*factor==original.definition.radius*factor){
            frozen=[context.selectedSweep.definition changingRadius:original.definition.radius];
            if(!frozen){completion(Core3DProfileConstructionResultRejected);return;}
        }
        context->_planningConsumed=YES;_modelingConstructionContext=context;
        __weak Core3DViewController *weakSelf=self;
        __weak Core3DModelingPlanningContext *weakContext=context;
        [self rebuildStoredSweep:context.selectedSweep definition:frozen expected:context.scene
            completion:^(Core3DProfileConstructionResult result){
                Core3DViewController *owner=weakSelf;Core3DModelingPlanningContext *finished=weakContext;
                if(finished)finished->_planningRetired=YES;
                if(owner&&owner->_modelingConstructionContext==finished)owner->_modelingConstructionContext=nil;
                completion(result);
            }];
    }catch(...){completion(Core3DProfileConstructionResultRejected);}
}

- (void)rebuildProfileRecipeWithDefinition:(Core3DProfileDefinition *)definition
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) return;
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{completion(Core3DProfileConstructionResultRejected);});return;
    }
    if (![self isModelingPlanningContextCurrent:context] || !context.selectedProfileRecipe
        || context.selectedProfile || context.selectedEnclosure
        || ![definition isMemberOfClass:Core3DProfileDefinition.class]) {
        completion(Core3DProfileConstructionResultRejected);return;
    }
    try {
        const auto original=[context.selectedProfileRecipe nativeSnapshot];
        const auto requested=[definition nativeParameters];
        if (!original.current || !Core3DModelingProfileRecipeSupported(original.parameters,original.dimensionMetersPerUnit)
            || !Core3DModelingProfileRecipeSupported(requested,original.dimensionMetersPerUnit)) {
            completion(Core3DProfileConstructionResultRejected);return;
        }
        // Encode the original with just the requested parameter, then compare
        // every scalar bit. This also binds optional frame, unit, all IDs and
        // ordered contour/hole values; +/-zero cannot hide any other change.
        auto expected=original.parameters;expected.definition.depth=requested.definition.depth;
        std::vector<double> expectedValues,requestedValues;
        if (!core3d::profile::Encode(expected,expectedValues)
            || !core3d::profile::Encode(requested,requestedValues)
            || expectedValues.size()!=requestedValues.size()
            || std::memcmp(expectedValues.data(),requestedValues.data(),expectedValues.size()*sizeof(double))!=0) {
            completion(Core3DProfileConstructionResultRejected);return;
        }
        context->_planningConsumed=YES;_modelingConstructionContext=context;
        __weak Core3DViewController *weakSelf=self;
        __weak Core3DModelingPlanningContext *weakContext=context;
        [self rebuildStoredProfile:context.selectedProfileRecipe definition:definition expected:context.scene
            completion:^(Core3DProfileConstructionResult result) {
                Core3DViewController *owner=weakSelf;
                Core3DModelingPlanningContext *finished=weakContext;
                if (finished) finished->_planningRetired=YES;
                if (owner && owner->_modelingConstructionContext==finished) owner->_modelingConstructionContext=nil;
                completion(result);
            }];
    } catch (...) {completion(Core3DProfileConstructionResultRejected);}
}

- (void)createAssemblyWithParts:(NSArray<Core3DAssemblyPartDefinition *> *)parts
    context:(Core3DModelingPlanningContext *)context
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) return;
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{completion(Core3DProfileConstructionResultRejected);});return;
    }
    if (![self isModelingPlanningContextCurrent:context] || ![parts isKindOfClass:NSArray.class]
        || parts.count==0 || parts.count>16) {completion(Core3DProfileConstructionResultRejected);return;}
    try {
        NSArray<Core3DAssemblyPartDefinition *> *frozen=[parts copy];
        NSMutableSet<NSString *> *names=[NSMutableSet set],*identifiers=[NSMutableSet set];
        std::vector<core3d::AssemblyPartDefinition> nativeParts;
        for (Core3DAssemblyPartDefinition *part in frozen) {
            if (![part isKindOfClass:Core3DAssemblyPartDefinition.class] || [names containsObject:part.name]
                || [identifiers containsObject:part.partIdentifier]) {completion(Core3DProfileConstructionResultRejected);return;}
            const auto native=[part nativePartForMetersPerUnit:context.scene.metersPerUnit];
            if (!native) {completion(Core3DProfileConstructionResultRejected);return;}
            [names addObject:part.name];[identifiers addObject:part.partIdentifier];nativeParts.push_back(*native);
        }
        const CGSize size=GLController.drawableSize;
        if (!std::isfinite(size.width)||!std::isfinite(size.height)||size.width<1||size.height<1
            ||size.width>UINT32_MAX||size.height>UINT32_MAX) {completion(Core3DProfileConstructionResultRejected);return;}
        core3d::ObjectFrameIdentity identity;
        identity.publicationSourceIdentifier=context.scene.publicationSourceIdentifier.UTF8String;
        identity.documentGeneration=context.scene.revisions.documentGeneration;
        identity.modelRevision=context.scene.revisions.modelRevision;
        const auto work=GLController.viewer->prepareAssemblySolid(nativeParts,identity,context.scene.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        if (!work) {completion(Core3DProfileConstructionResultRejected);return;}
        context->_planningConsumed=YES;_modelingConstructionContext=context;
        __weak Core3DViewController *weakSelf=self;
        __weak Core3DModelingPlanningContext *weakContext=context;
        [self runNativeSolidWork:work completion:^(Core3DProfileConstructionResult result) {
            Core3DViewController *owner=weakSelf;
            Core3DModelingPlanningContext *finished=weakContext;
            if (finished) finished->_planningRetired=YES;
            if (owner && owner->_modelingConstructionContext==finished) owner->_modelingConstructionContext=nil;
            completion(result);
        }];
    } catch (...) {completion(Core3DProfileConstructionResultRejected);}
}

- (void)cancelNativeConstruction { [self cancelProfileConstruction]; }

- (Core3DStoredSweepSnapshot *)storedSweepWithEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || _nativeSolidWork || _isLoading.load() || !_isSetuped || _isPreviewMode
        || GLController == nil || GLController.viewer == nullptr
        || ![entityIdentifier isKindOfClass:[NSString class]] || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || ![expected isKindOfClass:[Core3DSceneSnapshot class]] || expected.selectionMode != Core3DSceneElementKindObject
        || expected.publicationSourceIdentifier.length == 0 || expected.publicationSourceIdentifier.length > 128) return nil;
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height) || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) return nil;
    try {
        if (!entityIdentifier.UTF8String || !expected.publicationSourceIdentifier.UTF8String) return nil;
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entityIdentifier.UTF8String,
            [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(expected.publicationSourceIdentifier.UTF8String,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto result = GLController.viewer->storedSweepDefinition(identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        return result ? [[Core3DStoredSweepSnapshot alloc] initWithNativeSnapshot:*result] : nil;
    } catch (...) { return nil; }
}

- (void)rebuildStoredSweep:(Core3DStoredSweepSnapshot *)original
    definition:(Core3DSweepDefinition *)definition expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) return;
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ completion(Core3DProfileConstructionResultRejected); }); return;
    }
    if (_nativeSolidWork || _isLoading.load()) { completion(Core3DProfileConstructionResultBusy); return; }
    if (![original isKindOfClass:[Core3DStoredSweepSnapshot class]]
        || ![definition isKindOfClass:[Core3DSweepDefinition class]]) {
        completion(Core3DProfileConstructionResultRejected); return;
    }
    const auto live = [self storedSweepWithEntityIdentifier:original.entityIdentifier expected:expected];
    if (!live) { completion(Core3DProfileConstructionResultRejected); return; }
    try {
        const CGSize size = GLController.drawableSize;
        const auto originalNative = [original nativeSnapshot];
        auto requested = [definition nativeDefinition];
        const auto work = GLController.viewer->prepareStoredSweepRebuild(requested,
            originalNative,[live nativeSnapshot].identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        if (!work) { completion(Core3DProfileConstructionResultRejected); return; }
        [self runNativeSolidWork:work completion:completion];
    } catch (...) { completion(Core3DProfileConstructionResultRejected); }
}


- (Core3DCylindricalCutSnapshot *)cylindricalCutSourceWithEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || _nativeSolidWork || _isLoading.load() || !_isSetuped || _isPreviewMode
        || GLController == nil || GLController.viewer == nullptr
        || ![entityIdentifier isKindOfClass:[NSString class]] || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || ![expected isKindOfClass:[Core3DSceneSnapshot class]] || expected.selectionMode != Core3DSceneElementKindObject
        || expected.publicationSourceIdentifier.length == 0 || expected.publicationSourceIdentifier.length > 128) return nil;
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height) || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) return nil;
    try {
        if (!entityIdentifier.UTF8String || !expected.publicationSourceIdentifier.UTF8String) return nil;
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entityIdentifier.UTF8String,
            [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(expected.publicationSourceIdentifier.UTF8String,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto result = GLController.viewer->cylindricalCutSource(identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        return result ? [[Core3DCylindricalCutSnapshot alloc] initWithNative:*result owner:self viewer:GLController.viewer] : nil;
    } catch (...) { return nil; }
}

- (Core3DSavedCutSourceOperation *)beginSavedCutSourceEdit:(Core3DCylindricalCutSnapshot *)original
    patch:(Core3DSavedCutSourcePatch *)patch expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    return [self core3d_beginSavedCutSourceEdit:original patch:patch expected:expected planningContext:nil completion:completion];
}

- (Core3DSavedCutSourceOperation *)core3d_beginSavedCutSourceEdit:(Core3DCylindricalCutSnapshot *)original
    patch:(Core3DSavedCutSourcePatch *)patch expected:(Core3DSceneSnapshot *)expected
    planningContext:(Core3DModelingPlanningContext *)planningContext
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if(!completion)return nil;
    if(!NSThread.isMainThread){dispatch_async(dispatch_get_main_queue(),^{completion(Core3DProfileConstructionResultRejected);});return nil;}
    if(_savedCutSourceJob||_nativeSolidWork||_objectAlignmentWork||_isLoading.load()){
        completion(Core3DProfileConstructionResultBusy);return nil;
    }
    if(!_isSetuped||_isPreviewMode||!GLController||!GLController.viewer
        ||![original isKindOfClass:Core3DCylindricalCutSnapshot.class]
        ||![patch isKindOfClass:Core3DSavedCutSourcePatch.class]
        ||![original matchesOwner:self viewer:GLController.viewer]){
        completion(Core3DProfileConstructionResultRejected);return nil;
    }
    const auto live=[self cylindricalCutSourceWithEntityIdentifier:original.entityIdentifier expected:expected];
    if(!live){completion(Core3DProfileConstructionResultRejected);return nil;}
    Core3DSavedCutSourceJob *job=nil;NSUUID *completionToken=nil;
    bool immediateDelivered=false;
    const auto immediate=[&](Core3DProfileConstructionResult result){
        if(immediateDelivered)return;immediateDelivered=true;completion(result);
    };
    try {
        const auto before=[original nativeSnapshot];
        if(!before.source.rebuilding||!before.source.original.retained.value){immediate(Core3DProfileConstructionResultRejected);return nil;}
        // Convert this typed request once using the ORIGINAL source units.
        // Live capture supplies only current admission identity, never a new edit target.
        const auto converted=[patch nativePatchFor:before.source.envelope];
        if(!converted){immediate(Core3DProfileConstructionResultRejected);return nil;}
        const auto viewer=GLController.viewer;
        const auto lease=core3d::Core3DViewer::makeSavedCutSourceEditWork();
        const auto cancellation=core3d::Core3DViewer::savedCutSourceEditCancellation(lease);
        if(!lease||!cancellation){immediate(Core3DProfileConstructionResultRejected);return nil;}
        job=[Core3DSavedCutSourceJob new];if(!job){immediate(Core3DProfileConstructionResultRejected);return nil;}
        job->_lease=lease;job->_cancellation=cancellation;job->_viewer=viewer;
        completionToken=Core3DRegisterNativeSolidCompletion(completion);
        if(!completionToken){[job finish:Core3DProfileConstructionResultBusy];immediate(Core3DProfileConstructionResultBusy);return nil;}
        job->_completionToken=completionToken;
        if(planningContext){
            // The sole internal caller already consumed this exact issued
            // context. Install reciprocal weak ownership before preparation.
            job->_planningContext=planningContext;planningContext->_planningSavedCutJob=job;
        }
        _savedCutSourceJob=job; // Own main lease/token BEFORE synchronous preparation.
        Core3DSavedCutSourceOperation *operation=[[Core3DSavedCutSourceOperation alloc] initWithJob:job];
        const CGSize size=GLController.drawableSize;
        if(!operation||!viewer->prepareSavedCutSourceEdit(lease,before,*converted,[live nativeSnapshot].identity,
            expected.revisions.presentationRevision,static_cast<std::uint32_t>(std::llround(size.width)),
            static_cast<std::uint32_t>(std::llround(size.height)))){
            _savedCutSourceJob=nil;[job finish:Core3DProfileConstructionResultRejected];return nil;
        }
        const auto geometry=core3d::Core3DViewer::savedCutSourceEditGeometry(lease);
        if(!geometry){_savedCutSourceJob=nil;[job finish:Core3DProfileConstructionResultRejected];return nil;}
#if DEBUG
        if(_debugSavedCutSourceDeliveryGate){
            Core3DNativeSolidGeometryDeliveryGates()[completionToken]=[_debugSavedCutSourceDeliveryGate copy];
            _debugSavedCutSourceDeliveryGate=nil;
        }
#endif
        __weak Core3DViewController *weakSelf=self;
        __weak Core3DSavedCutSourceJob *weakJob=job;
        // No original snapshot, patch, job, lease, viewer or callback is strongly
        // captured by utility work. Returned geometry likewise has no live scene.
        dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED,0),^{
            const auto built=core3d::Core3DViewer::buildSavedCutSourceDetached(geometry);
            dispatch_async(dispatch_get_main_queue(),^{
                dispatch_block_t delivery=^{
                    Core3DViewController *controller=weakSelf;
                    Core3DSavedCutSourceJob *pending=weakJob;
                    if(!controller||!pending){Core3DDeliverNativeSolidCompletion(completionToken,Core3DProfileConstructionResultRejected);return;}
                    if(controller->_savedCutSourceJob!=pending){[pending finish:Core3DProfileConstructionResultRejected];return;}
                    Core3DProfileConstructionResult result=Core3DProfileConstructionResultRejected;
                    const auto currentViewer=pending->_viewer.lock();
                    if(pending->_cancelled)result=Core3DProfileConstructionResultCancelled;
                    else if(currentViewer
                        &&std::holds_alternative<std::shared_ptr<core3d::SavedCutSourceEditWork>>(pending->_lease)
                        &&!controller->_nativeSolidWork&&!controller->_objectAlignmentWork
                        &&!controller->_isLoading.load()&&controller->_isSetuped&&!controller->_isPreviewMode
                        &&controller.glController&&((GLViewController *)controller.glController).viewer==currentViewer){
                        if(!built)result=Core3DProfileConstructionResultFailed;
                        else {
                            const auto actual=currentViewer->commitSavedCutSourceEdit(
                                std::get<std::shared_ptr<core3d::SavedCutSourceEditWork>>(pending->_lease),built);
                            switch(actual){
                                case core3d::OrdinaryEditResult::Committed:result=Core3DProfileConstructionResultCommitted;break;
                                case core3d::OrdinaryEditResult::NoChange:result=Core3DProfileConstructionResultUnchanged;break;
                                case core3d::OrdinaryEditResult::Busy:result=Core3DProfileConstructionResultBusy;break;
                                case core3d::OrdinaryEditResult::Invalid:result=Core3DProfileConstructionResultRejected;break;
                                case core3d::OrdinaryEditResult::OutcomeUnknown:result=Core3DProfileConstructionResultRecoveryRequired;break;
                                case core3d::OrdinaryEditResult::RetryableFailure:result=Core3DProfileConstructionResultFailed;break;
                            }
                        }
                    }
                    controller->_savedCutSourceJob=nil; // Exact old job only; before callbacks/new admission.
                    if(result==Core3DProfileConstructionResultCommitted){
                        [((GLViewController *)controller.glController) refreshSelectionState];
                        [controller viewDidChangeViewportPresentationState];
                        [controller sendNotifyUIState:UIStateChangingSelection|UIStateChangingGizmo|UIStateChangingDelete
                            |UIStateChangingDuplicate|UIStateChangingApply|UIStateChangingApplyMaterial|UIStateChangingHistory];
                    }
                    [pending finish:result];
                };
#if DEBUG
                Core3DGateNativeSolidGeometryDelivery(completionToken,delivery);
#else
                delivery();
#endif
            });
        });
        return operation;
    }catch(...){
        if(job){
            const BOOL registeredOrFinished=job->_completionToken!=nil||job->_finished;
            if(_savedCutSourceJob==job)_savedCutSourceJob=nil;
            [job finish:Core3DProfileConstructionResultRejected];
            if(!registeredOrFinished)immediate(Core3DProfileConstructionResultRejected);
        }
        else if(completionToken)Core3DDeliverNativeSolidCompletion(completionToken,Core3DProfileConstructionResultRejected);
        else immediate(Core3DProfileConstructionResultRejected);
        return nil;
    }
}

- (Core3DSavedCutSourceOperation *)beginSavedProgramCutSourceEdit:(Core3DCylindricalCutProgramSnapshot *)original
    patch:(Core3DSavedCutSourcePatch *)patch expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    return [self core3d_beginSavedProgramCutSourceEdit:original patch:patch expected:expected planningContext:nil completion:completion];
}

- (Core3DSavedCutSourceOperation *)core3d_beginSavedProgramCutSourceEdit:(Core3DCylindricalCutProgramSnapshot *)original
    patch:(Core3DSavedCutSourcePatch *)patch expected:(Core3DSceneSnapshot *)expected
    planningContext:(Core3DModelingPlanningContext *)planningContext
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if(!completion)return nil;
    if(!NSThread.isMainThread){dispatch_async(dispatch_get_main_queue(),^{completion(Core3DProfileConstructionResultRejected);});return nil;}
    if(_savedCutSourceJob||_nativeSolidWork||_objectAlignmentWork||_isLoading.load()){
        completion(Core3DProfileConstructionResultBusy);return nil;
    }
    // Runtime ObjC class and exact owner/exclusivity validation BEFORE any
    // snapshot or patch selector is read; wrong-class or foreign inputs reject
    // here, never inside untrusted definition/patch selectors.
    if(!_isSetuped||_isPreviewMode||!GLController||!GLController.viewer
        ||![original isKindOfClass:Core3DCylindricalCutProgramSnapshot.class]
        ||![patch isKindOfClass:Core3DSavedCutSourcePatch.class]
        ||![original matchesOwner:self viewer:GLController.viewer]){
        completion(Core3DProfileConstructionResultRejected);return nil;
    }
    // Live capture supplies only current admission identity, never a new edit target.
    const auto live=[self cylindricalCutProgramWithEntityIdentifier:original.entityIdentifier expected:expected];
    if(!live){completion(Core3DProfileConstructionResultRejected);return nil;}
    Core3DSavedCutSourceJob *job=nil;NSUUID *completionToken=nil;
    bool immediateDelivered=false;
    const auto immediate=[&](Core3DProfileConstructionResult result){
        if(immediateDelivered)return;immediateDelivered=true;completion(result);
    };
    try {
        const auto before=[original nativeSnapshot];
        const auto* program=std::get_if<core3d::retained_boolean::Program>(&before.source.recipe);
        // Whole-program source edits require the complete v2 program payload;
        // a legacy one-bore payload keeps its exact existing legacy API.
        if(!before.source.original.retained.value||!program){immediate(Core3DProfileConstructionResultRejected);return nil;}
        // Convert this typed request once using the ORIGINAL complete source.
        const auto converted=[patch nativePatchForProgram:*program];
        if(!converted){immediate(Core3DProfileConstructionResultRejected);return nil;}
        const auto viewer=GLController.viewer;
        const auto lease=core3d::Core3DViewer::makeSavedProgramSourceEditWork();
        const auto cancellation=core3d::Core3DViewer::savedProgramSourceEditCancellation(lease);
        if(!lease||!cancellation){immediate(Core3DProfileConstructionResultRejected);return nil;}
        job=[Core3DSavedCutSourceJob new];if(!job){immediate(Core3DProfileConstructionResultRejected);return nil;}
        job->_lease=lease;job->_cancellation=cancellation;job->_viewer=viewer;
        completionToken=Core3DRegisterNativeSolidCompletion(completion);
        if(!completionToken){[job finish:Core3DProfileConstructionResultBusy];immediate(Core3DProfileConstructionResultBusy);return nil;}
        job->_completionToken=completionToken;
        if(planningContext){
            // Match the legacy modeling route: the consumed exact context and
            // this job retain reciprocal weak cancellation ownership until the
            // admitted numeric worker actually drains.
            job->_planningContext=planningContext;planningContext->_planningSavedCutJob=job;
        }
        _savedCutSourceJob=job; // Own main lease/token BEFORE synchronous preparation.
        Core3DSavedCutSourceOperation *operation=[[Core3DSavedCutSourceOperation alloc] initWithJob:job];
        const CGSize size=GLController.drawableSize;
        if(!operation||!viewer->prepareSavedProgramSourceEdit(lease,before,*converted,[live nativeSnapshot].identity,
            expected.revisions.presentationRevision,static_cast<std::uint32_t>(std::llround(size.width)),
            static_cast<std::uint32_t>(std::llround(size.height)))){
            _savedCutSourceJob=nil;[job finish:Core3DProfileConstructionResultRejected];return nil;
        }
        const auto geometry=core3d::Core3DViewer::savedProgramSourceEditGeometry(lease);
        if(!geometry){_savedCutSourceJob=nil;[job finish:Core3DProfileConstructionResultRejected];return nil;}
#if DEBUG
        if(_debugSavedCutSourceDeliveryGate){
            Core3DNativeSolidGeometryDeliveryGates()[completionToken]=[_debugSavedCutSourceDeliveryGate copy];
            _debugSavedCutSourceDeliveryGate=nil;
        }
#endif
        __weak Core3DViewController *weakSelf=self;
        __weak Core3DSavedCutSourceJob *weakJob=job;
        // No original snapshot, patch, job, lease, viewer or callback is strongly
        // captured by utility work. Returned geometry likewise has no live scene.
        dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED,0),^{
            const auto built=core3d::Core3DViewer::buildSavedProgramSourceDetached(geometry);
            dispatch_async(dispatch_get_main_queue(),^{
                dispatch_block_t delivery=^{
                    Core3DViewController *controller=weakSelf;
                    Core3DSavedCutSourceJob *pending=weakJob;
                    if(!controller||!pending){Core3DDeliverNativeSolidCompletion(completionToken,Core3DProfileConstructionResultRejected);return;}
                    if(controller->_savedCutSourceJob!=pending){[pending finish:Core3DProfileConstructionResultRejected];return;}
                    Core3DProfileConstructionResult result=Core3DProfileConstructionResultRejected;
                    const auto currentViewer=pending->_viewer.lock();
                    if(pending->_cancelled)result=Core3DProfileConstructionResultCancelled;
                    else if(currentViewer
                        &&std::holds_alternative<std::shared_ptr<core3d::SavedProgramSourceEditWork>>(pending->_lease)
                        &&!controller->_nativeSolidWork&&!controller->_objectAlignmentWork
                        &&!controller->_isLoading.load()&&controller->_isSetuped&&!controller->_isPreviewMode
                        &&controller.glController&&((GLViewController *)controller.glController).viewer==currentViewer){
                        if(!built)result=Core3DProfileConstructionResultFailed;
                        else {
                            const auto actual=currentViewer->commitSavedProgramSourceEdit(
                                std::get<std::shared_ptr<core3d::SavedProgramSourceEditWork>>(pending->_lease),built);
                            switch(actual){
                                case core3d::OrdinaryEditResult::Committed:result=Core3DProfileConstructionResultCommitted;break;
                                case core3d::OrdinaryEditResult::NoChange:result=Core3DProfileConstructionResultUnchanged;break;
                                case core3d::OrdinaryEditResult::Busy:result=Core3DProfileConstructionResultBusy;break;
                                case core3d::OrdinaryEditResult::Invalid:result=Core3DProfileConstructionResultRejected;break;
                                case core3d::OrdinaryEditResult::OutcomeUnknown:result=Core3DProfileConstructionResultRecoveryRequired;break;
                                case core3d::OrdinaryEditResult::RetryableFailure:result=Core3DProfileConstructionResultFailed;break;
                            }
                        }
                    }
                    controller->_savedCutSourceJob=nil; // Exact old job only; before callbacks/new admission.
                    if(result==Core3DProfileConstructionResultCommitted){
                        [((GLViewController *)controller.glController) refreshSelectionState];
                        [controller viewDidChangeViewportPresentationState];
                        [controller sendNotifyUIState:UIStateChangingSelection|UIStateChangingGizmo|UIStateChangingDelete
                            |UIStateChangingDuplicate|UIStateChangingApply|UIStateChangingApplyMaterial|UIStateChangingHistory];
                    }
                    [pending finish:result];
                };
#if DEBUG
                Core3DGateNativeSolidGeometryDelivery(completionToken,delivery);
#else
                delivery();
#endif
            });
        });
        return operation;
    }catch(...){
        if(job){
            const BOOL registeredOrFinished=job->_completionToken!=nil||job->_finished;
            if(_savedCutSourceJob==job)_savedCutSourceJob=nil;
            [job finish:Core3DProfileConstructionResultRejected];
            if(!registeredOrFinished)immediate(Core3DProfileConstructionResultRejected);
        }
        else if(completionToken)Core3DDeliverNativeSolidCompletion(completionToken,Core3DProfileConstructionResultRejected);
        else immediate(Core3DProfileConstructionResultRejected);
        return nil;
    }
}
#if DEBUG
- (void)debugSetSavedCutSourceDeliveryGate:(void (^)(void (^)(void)))gate {
    if(NSThread.isMainThread)_debugSavedCutSourceDeliveryGate=[gate copy];
}
+ (NSDictionary<NSString *,NSNumber *> *)debugSavedCutSourceMMConversionProbe {
    const double tiny=std::numeric_limits<double>::denorm_min();
    const auto check=[](auto fn){double out=0;return fn(out);};
    const double maximum=std::numeric_limits<double>::max();
    return @{
        @"positive-underflow-refused":@(check([&](double& out){return !Core3DSourceMMToRecipe(tiny,0,1000,out);})),
        @"negative-underflow-refused":@(check([&](double& out){return !Core3DSourceMMToRecipe(-tiny,0,1000,out);})),
        @"descriptor-underflow-refused":@(check([&](double& out){return !Core3DSourceRecipeToMM(tiny,0.001,out);})),
        @"old-underflow-refused-before-equality":@(check([&](double& out){return !Core3DSourceMMToRecipe(0,tiny,0.001,out);})),
        @"converted-overflow-refused":@(check([&](double& out){return !Core3DSourceMMToRecipe(1,0,tiny,out);})),
        @"descriptor-overflow-refused":@(check([&](double& out){return !Core3DSourceRecipeToMM(maximum,1000,out);})),
        @"nan-refused":@(check([&](double& out){return !Core3DSourceMMToRecipe(std::numeric_limits<double>::quiet_NaN(),0,1,out);})),
        @"zero-factor-refused":@(check([&](double& out){return !Core3DSourceMMToRecipe(1,0,0,out);})),
        @"negative-factor-refused":@(check([&](double& out){return !Core3DSourceRecipeToMM(1,-1,out);})),
        @"subnormal-representable-preserved":@(check([&](double& out){return Core3DSourceMMToRecipe(tiny,0,1,out)&&out==tiny;})),
        @"negative-zero-original-preserved":@(check([&](double& out){return Core3DSourceMMToRecipe(0,-0.0,1000,out)&&std::signbit(out);})),
        @"metre-conversion":@(check([&](double& out){return Core3DSourceMMToRecipe(120,0.1,1000,out)&&out==0.12;})),
        @"unchanged-metre-bits":@(check([&](double& out){return Core3DSourceMMToRecipe(60,0.06,1000,out)&&core3d::retained_solid::Bits(out)==core3d::retained_solid::Bits(0.06);}))
    };
}
#endif

- (BOOL)core3d_cancelCutWork:(const std::shared_ptr<core3d::NativeSolidWork>&)work {
    if(!NSThread.isMainThread||!work||_nativeSolidWork!=work)return NO;[self cancelNativeConstruction];return YES;
}
- (Core3DCylindricalCutOperation *)core3d_beginCut:(Core3DCylindricalCutSnapshot *)original
    definition:(Core3DCylindricalCutDefinition *)definition radius:(std::optional<double>)radius
    expected:(Core3DSceneSnapshot *)expected completion:(void(^)(Core3DProfileConstructionResult))completion {
    if(!completion)return nil;
    if(!NSThread.isMainThread){dispatch_async(dispatch_get_main_queue(),^{completion(Core3DProfileConstructionResultRejected);});return nil;}
    if(_nativeSolidWork||_isLoading.load()){completion(Core3DProfileConstructionResultBusy);return nil;}
    if(![original isKindOfClass:Core3DCylindricalCutSnapshot.class]||!GLController||!GLController.viewer
        ||![original matchesOwner:self viewer:GLController.viewer]
        ||bool(radius)==bool(definition)||(definition&&![definition isKindOfClass:Core3DCylindricalCutDefinition.class])){
        completion(Core3DProfileConstructionResultRejected);return nil;
    }
    const auto live=[self cylindricalCutSourceWithEntityIdentifier:original.entityIdentifier expected:expected];
    if(!live){completion(Core3DProfileConstructionResultRejected);return nil;}
    try {
        const auto before=[original nativeSnapshot];const auto identity=[live nativeSnapshot].identity;
        std::optional<core3d::cylindrical_cut::CreateEdit> create;std::optional<core3d::cylindrical_cut::RadiusEdit> rebuild;
        if(radius)rebuild=core3d::cylindrical_cut::RadiusEdit{*radius};
        else create=core3d::cylindrical_cut::CreateEdit{static_cast<core3d::analytic_boolean::Axis>(definition.axis),
            {definition.localX,definition.localY,definition.localZ},definition.worldRadiusMM};
        const CGSize size=GLController.drawableSize;
        const auto work=GLController.viewer->prepareCylindricalCut(before,create,rebuild,identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        if(!work){completion(Core3DProfileConstructionResultRejected);return nil;}
        __block BOOL delivered=NO;
        [self runNativeSolidWork:work completion:^(Core3DProfileConstructionResult result){delivered=YES;completion(result);}];
        // No yield: only this exact admitted work receives cancellation rights.
        if(delivered||_nativeSolidWork!=work)return nil;
        return [[Core3DCylindricalCutOperation alloc] initWithOwner:self work:work];
    }catch(...){completion(Core3DProfileConstructionResultRejected);return nil;}
}
- (Core3DCylindricalCutOperation *)beginCylindricalCut:(Core3DCylindricalCutSnapshot *)original
    definition:(Core3DCylindricalCutDefinition *)definition expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if(!definition){if(completion){if(NSThread.isMainThread)completion(Core3DProfileConstructionResultRejected);
        else dispatch_async(dispatch_get_main_queue(),^{completion(Core3DProfileConstructionResultRejected);});}return nil;}
    return [self core3d_beginCut:original definition:definition radius:std::nullopt expected:expected completion:completion];
}
- (Core3DCylindricalCutOperation *)beginCylindricalCutRadius:(Core3DCylindricalCutSnapshot *)original
    worldRadiusMM:(double)radius expected:(Core3DSceneSnapshot *)expected completion:(void(^)(Core3DProfileConstructionResult))completion {
    return [self core3d_beginCut:original definition:nil radius:std::optional<double>(radius) expected:expected completion:completion];
}

- (Core3DCylindricalCutProgramSnapshot *)cylindricalCutProgramWithEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || _nativeSolidWork || _isLoading.load() || !_isSetuped || _isPreviewMode
        || GLController == nil || GLController.viewer == nullptr
        || ![entityIdentifier isKindOfClass:[NSString class]] || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || ![expected isKindOfClass:[Core3DSceneSnapshot class]] || expected.selectionMode != Core3DSceneElementKindObject
        || expected.publicationSourceIdentifier.length == 0 || expected.publicationSourceIdentifier.length > 128) return nil;
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height) || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) return nil;
    try {
        if (!entityIdentifier.UTF8String || !expected.publicationSourceIdentifier.UTF8String) return nil;
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entityIdentifier.UTF8String,
            [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(expected.publicationSourceIdentifier.UTF8String,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto result = GLController.viewer->cylindricalCutProgramSource(identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        return result ? [[Core3DCylindricalCutProgramSnapshot alloc] initWithNative:*result owner:self viewer:GLController.viewer] : nil;
    } catch (...) { return nil; }
}
- (Core3DCylindricalCutOperation *)core3d_beginProgramEdit:(Core3DCylindricalCutProgramSnapshot *)original
    definition:(Core3DCylindricalCutDefinition *)definition
    radius:(std::optional<core3d::retained_boolean::SetBoreRadius>)radius
    expected:(Core3DSceneSnapshot *)expected completion:(void(^)(Core3DProfileConstructionResult))completion {
    if(!completion)return nil;
    if(!NSThread.isMainThread){dispatch_async(dispatch_get_main_queue(),^{completion(Core3DProfileConstructionResultRejected);});return nil;}
    if(_nativeSolidWork||_isLoading.load()){completion(Core3DProfileConstructionResultBusy);return nil;}
    // Runtime class and exclusivity validation on the correct main-thread
    // completion path BEFORE any definition selector is read, exactly like
    // core3d_beginCut; a wrong-class input rejects here, never at the callers.
    if(![original isKindOfClass:Core3DCylindricalCutProgramSnapshot.class]||!GLController||!GLController.viewer
        ||![original matchesOwner:self viewer:GLController.viewer]
        ||bool(radius)==bool(definition)||(definition&&![definition isKindOfClass:Core3DCylindricalCutDefinition.class])){
        completion(Core3DProfileConstructionResultRejected);return nil;
    }
    // Live capture supplies only current admission identity, never a new edit target.
    const auto live=[self cylindricalCutProgramWithEntityIdentifier:original.entityIdentifier expected:expected];
    if(!live){completion(Core3DProfileConstructionResultRejected);return nil;}
    try {
        const auto before=[original nativeSnapshot];const auto identity=[live nativeSnapshot].identity;
        core3d::retained_boolean::ProgramEdit edit;
        if(radius)edit=*radius;
        else edit=core3d::retained_boolean::AppendBore{
            core3d::cylindrical_cut::CreateEdit{static_cast<core3d::analytic_boolean::Axis>(definition.axis),
                {definition.localX,definition.localY,definition.localZ},definition.worldRadiusMM}};
        const CGSize size=GLController.drawableSize;
        const auto work=GLController.viewer->prepareCylindricalCutProgramEdit(before,edit,identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        if(!work){completion(Core3DProfileConstructionResultRejected);return nil;}
        __block BOOL delivered=NO;
        [self runNativeSolidWork:work completion:^(Core3DProfileConstructionResult result){delivered=YES;completion(result);}];
        // No yield: only this exact admitted work receives cancellation rights.
        if(delivered||_nativeSolidWork!=work)return nil;
        return [[Core3DCylindricalCutOperation alloc] initWithOwner:self work:work];
    }catch(...){completion(Core3DProfileConstructionResultRejected);return nil;}
}
- (Core3DCylindricalCutOperation *)beginCylindricalCutAppendBore:(Core3DCylindricalCutProgramSnapshot *)original
    definition:(Core3DCylindricalCutDefinition *)definition expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if(!definition){if(completion){if(NSThread.isMainThread)completion(Core3DProfileConstructionResultRejected);
        else dispatch_async(dispatch_get_main_queue(),^{completion(Core3DProfileConstructionResultRejected);});}return nil;}
    return [self core3d_beginProgramEdit:original definition:definition radius:std::nullopt expected:expected completion:completion];
}
- (Core3DCylindricalCutOperation *)beginCylindricalCutBoreRadius:(Core3DCylindricalCutProgramSnapshot *)original
    operandIdentifier:(uint32_t)operandIdentifier worldRadiusMM:(double)radius expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    return [self core3d_beginProgramEdit:original definition:nil
        radius:std::optional<core3d::retained_boolean::SetBoreRadius>(core3d::retained_boolean::SetBoreRadius{operandIdentifier,radius})
        expected:expected completion:completion];
}

- (Core3DStoredRectangularLoftSnapshot *)storedRectangularLoftWithEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || _nativeSolidWork || _isLoading.load() || !_isSetuped || _isPreviewMode
        || GLController == nil || GLController.viewer == nullptr
        || ![entityIdentifier isKindOfClass:[NSString class]] || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || ![expected isKindOfClass:[Core3DSceneSnapshot class]] || expected.selectionMode != Core3DSceneElementKindObject
        || expected.publicationSourceIdentifier.length == 0 || expected.publicationSourceIdentifier.length > 128) return nil;
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height) || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) return nil;
    try {
        if (!entityIdentifier.UTF8String || !expected.publicationSourceIdentifier.UTF8String) return nil;
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entityIdentifier.UTF8String,
            [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(expected.publicationSourceIdentifier.UTF8String,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto result = GLController.viewer->storedRectangularLoftDefinition(identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        return result ? [[Core3DStoredRectangularLoftSnapshot alloc] initWithNativeSnapshot:*result] : nil;
    } catch (...) { return nil; }
}

- (void)rebuildStoredRectangularLoft:(Core3DStoredRectangularLoftSnapshot *)original
    edit:(Core3DRectangularLoftStationEdit *)edit expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) return;
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ completion(Core3DProfileConstructionResultRejected); }); return;
    }
    if (_nativeSolidWork || _isLoading.load()) { completion(Core3DProfileConstructionResultBusy); return; }
    if (![original isKindOfClass:[Core3DStoredRectangularLoftSnapshot class]]
        || ![edit isKindOfClass:[Core3DRectangularLoftStationEdit class]]) {
        completion(Core3DProfileConstructionResultRejected); return;
    }
    const auto live = [self storedRectangularLoftWithEntityIdentifier:original.entityIdentifier expected:expected];
    if (!live) { completion(Core3DProfileConstructionResultRejected); return; }
    try {
        const CGSize size = GLController.drawableSize;
        const auto originalNative = [original nativeSnapshot];
        const auto requested = [edit nativeEdit];
        const auto work = GLController.viewer->prepareStoredLoftStationRebuild(requested,
            originalNative,[live nativeSnapshot].identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        if (!work) { completion(Core3DProfileConstructionResultRejected); return; }
        [self runNativeSolidWork:work completion:completion];
    } catch (...) { completion(Core3DProfileConstructionResultRejected); }
}


- (BOOL)core3d_cancelStoredLoftWork:(const std::shared_ptr<core3d::NativeSolidWork>&)work {
    if(![NSThread isMainThread] || !work || _nativeSolidWork!=work)return NO;
    [self cancelNativeConstruction];return YES;
}

- (Core3DStoredLoftEditOperation *)beginStoredRectangularLoftEdit:(Core3DStoredRectangularLoftSnapshot *)original
    edit:(Core3DRectangularLoftStationEdit *)edit expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if(!completion)return nil;
    if(![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{completion(Core3DProfileConstructionResultRejected);});return nil;
    }
    // The existing entry performs all authority/validation and owns the work.
    // Its synchronous refusal callback may itself start a different job: mark
    // delivery BEFORE calling user code so that job can never be captured here.
    __block BOOL delivered=NO;
    [self rebuildStoredRectangularLoft:original edit:edit expected:expected completion:^(Core3DProfileConstructionResult result){
        delivered=YES;completion(result);
    }];
    // Main has not yielded since the entry returned; a genuine admitted job
    // cannot deliver its queued main completion before this exact slot capture.
    if(delivered || !_nativeSolidWork)return nil;
    return [[Core3DStoredLoftEditOperation alloc] initWithOwner:self work:_nativeSolidWork];
}

- (void)createRectangularLoftWithDefinition:(Core3DRectangularLoftDefinition *)definition
    expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) return;
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ completion(Core3DProfileConstructionResultRejected); });
        return;
    }
    if (_nativeSolidWork || _isLoading.load()) { completion(Core3DProfileConstructionResultBusy); return; }
    if (!_isSetuped || GLController == nil || GLController.viewer == nullptr
        || ![definition isKindOfClass:[Core3DRectangularLoftDefinition class]]
        || expected == nil || expected.selectionMode != Core3DSceneElementKindObject
        || definition.metersPerUnit != expected.metersPerUnit
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
        const char *publication = expected.publicationSourceIdentifier.UTF8String;
        if (!publication) { completion(Core3DProfileConstructionResultRejected); return; }
        core3d::ObjectFrameIdentity identity;
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto parameters = [definition nativeDefinition];
        const auto work = viewer->prepareLoftSolid(parameters,identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        if (!work) { completion(Core3DProfileConstructionResultRejected); return; }
        [self runNativeSolidWork:work completion:completion];
    } catch (...) { completion(Core3DProfileConstructionResultRejected); }
}
- (void)createSweepWithDefinition:(Core3DSweepDefinition *)definition
    expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) return;
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ completion(Core3DProfileConstructionResultRejected); });
        return;
    }
    if (_nativeSolidWork || _isLoading.load()) { completion(Core3DProfileConstructionResultBusy); return; }
    if (!_isSetuped || GLController == nil || GLController.viewer == nullptr
        || ![definition isKindOfClass:[Core3DSweepDefinition class]]
        || expected == nil || expected.selectionMode != Core3DSceneElementKindObject
        || definition.metersPerUnit != expected.metersPerUnit
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
        const char *publication = expected.publicationSourceIdentifier.UTF8String;
        if (!publication) { completion(Core3DProfileConstructionResultRejected); return; }
        core3d::ObjectFrameIdentity identity;
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto parameters = [definition nativeDefinition];
        const auto work = viewer->prepareSweepSolid(parameters,identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        if (!work) { completion(Core3DProfileConstructionResultRejected); return; }
        [self runNativeSolidWork:work completion:completion];
    } catch (...) { completion(Core3DProfileConstructionResultRejected); }
}

- (void)createProfileWithDefinition:(Core3DProfileDefinition *)definition
    expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) return;
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ completion(Core3DProfileConstructionResultRejected); });
        return;
    }
    if (_nativeSolidWork || _isLoading.load()) { completion(Core3DProfileConstructionResultBusy); return; }
    if (!_isSetuped || GLController == nil || GLController.viewer == nullptr
        || ![definition isKindOfClass:[Core3DProfileDefinition class]]
        || expected == nil || expected.selectionMode != Core3DSceneElementKindObject
        || definition.metersPerUnit != expected.metersPerUnit
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
        const char *publication = expected.publicationSourceIdentifier.UTF8String;
        if (!publication) { completion(Core3DProfileConstructionResultRejected); return; }
        core3d::ObjectFrameIdentity identity;
        identity.publicationSourceIdentifier.assign(publication,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto parameters = [definition nativeParameters];
        const auto work = viewer->prepareProfileSolid(parameters,identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        if (!work) { completion(Core3DProfileConstructionResultRejected); return; }
        [self runNativeSolidWork:work completion:completion];
    } catch (...) { completion(Core3DProfileConstructionResultRejected); }
}

- (void)runNativeSolidWork:(const std::shared_ptr<core3d::NativeSolidWork>&)work
                completion:(void(^)(Core3DProfileConstructionResult))completion {
#if DEBUG
        [self runNativeSolidWork:work debugGeometryDeliveryGate:nil completion:completion];
}
- (void)runNativeSolidWork:(const std::shared_ptr<core3d::NativeSolidWork>&)work
    debugGeometryDeliveryGate:(void (^)(void (^)(void)))gate
    completion:(void(^)(Core3DProfileConstructionResult))completion {
#endif
        NSUUID *completionToken = Core3DRegisterNativeSolidCompletion(completion);
        if (!completionToken) { completion(Core3DProfileConstructionResultBusy); return; }
#if DEBUG
        if(gate)Core3DNativeSolidGeometryDeliveryGates()[completionToken]=[gate copy];
#endif
        _nativeSolidWork = work; _nativeSolidCancelled = NO;
        __weak Core3DModelingPlanningContext *weakPlanningContext = _modelingConstructionContext;
        const auto geometry = core3d::Core3DViewer::nativeSolidGeometry(work);
        const std::weak_ptr<core3d::Core3DViewer> expectedViewer = GLController.viewer;
        __weak Core3DViewController* weakSelf = self;
        dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^{
            const bool built = core3d::Core3DViewer::buildNativeSolidGeometry(geometry);
            dispatch_async(dispatch_get_main_queue(), ^{
#if DEBUG
                dispatch_block_t delivery=^{
#endif
                Core3DViewController* controller = weakSelf;
                if (!controller) { Core3DDeliverNativeSolidCompletion(completionToken, Core3DProfileConstructionResultRejected); return; }
                Core3DModelingPlanningContext *planningContext = weakPlanningContext;
                const BOOL cancelled = controller->_nativeSolidCancelled;
                // Main-thread work owns live scene authority. The worker's only
                // strong payload contains new private geometry and scalar values.
                const auto pendingWork = std::move(controller->_nativeSolidWork);
                const auto currentViewer = expectedViewer.lock();
                if (cancelled) { Core3DDeliverNativeSolidCompletion(completionToken, Core3DProfileConstructionResultCancelled); return; }
                if (!pendingWork || !currentViewer || controller->_isLoading.load() || !controller->_isSetuped
                    || ((GLViewController *)controller.glController) == nil
                    || ((GLViewController *)controller.glController).viewer != currentViewer) {
                    Core3DDeliverNativeSolidCompletion(completionToken, Core3DProfileConstructionResultRejected); return;
                }
                // Recheck the originally issued lease after private geometry
                // work, before opening the single ordinary OCAF transaction.
                // The work slot was moved out above, so its own job is not
                // mistaken for competing native work by readiness admission.
                if (planningContext && ![controller core3d_modelingContext:planningContext
                        matchesAllowingConsumed:YES]) {
                    Core3DDeliverNativeSolidCompletion(completionToken, Core3DProfileConstructionResultRejected); return;
                }
                Core3DModelingPreparedRequest *reserved=controller->_pendingModelingReservation;
                if(reserved&&reserved->_requestContext==planningContext&&reserved->_requestCommitPermit
                    &&(![controller core3d_reservationMatches:reserved]||!reserved->_requestCommitPermit->current())){
                    Core3DDeliverNativeSolidCompletion(completionToken,Core3DProfileConstructionResultRejected);return;
                }
                if (!built) { Core3DDeliverNativeSolidCompletion(completionToken, Core3DProfileConstructionResultFailed); return; }
                const auto native = currentViewer->commitNativeSolid(pendingWork);
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
                    case core3d::OrdinaryEditResult::NoChange: result = Core3DProfileConstructionResultUnchanged; break;
                    case core3d::OrdinaryEditResult::Invalid: result = Core3DProfileConstructionResultRejected; break;
                    case core3d::OrdinaryEditResult::OutcomeUnknown: result = Core3DProfileConstructionResultRecoveryRequired; break;
                    case core3d::OrdinaryEditResult::RetryableFailure: result = Core3DProfileConstructionResultFailed; break;
                }
                Core3DDeliverNativeSolidCompletion(completionToken, result);
#if DEBUG
                };
                Core3DGateNativeSolidGeometryDelivery(completionToken,delivery);
#endif
            });
        });
}

- (void)rebuildStoredProfileWithEntityIdentifier:(NSString *)entityIdentifier
                                     parameter:(double)parameter
                                      expected:(Core3DSceneSnapshot *)expected
                                    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) return;
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ completion(Core3DProfileConstructionResultRejected); });
        return;
    }
    if (_nativeSolidWork || _isLoading.load()) { completion(Core3DProfileConstructionResultBusy); return; }
    if (!_isSetuped || GLController == nil || GLController.viewer == nullptr
        || ![entityIdentifier isKindOfClass:[NSString class]] || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || expected == nil || expected.selectionMode != Core3DSceneElementKindObject
        || expected.publicationSourceIdentifier.length == 0 || expected.publicationSourceIdentifier.length > 128
        || !std::isfinite(parameter)) { completion(Core3DProfileConstructionResultRejected); return; }
    const auto viewer = GLController.viewer;
    if (!viewer->canBeginCommittedEdit()) { completion(Core3DProfileConstructionResultBusy); return; }
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height) || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) {
        completion(Core3DProfileConstructionResultRejected); return;
    }
    try {
        if (!entityIdentifier.UTF8String || !expected.publicationSourceIdentifier.UTF8String) {
            completion(Core3DProfileConstructionResultRejected); return;
        }
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entityIdentifier.UTF8String,
            [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(expected.publicationSourceIdentifier.UTF8String,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto work = viewer->prepareStoredProfileRebuild(parameter, identity, expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)), static_cast<std::uint32_t>(std::llround(size.height)));
        if (!work) { completion(Core3DProfileConstructionResultRejected); return; }
        [self runNativeSolidWork:work completion:completion];
    } catch (...) { completion(Core3DProfileConstructionResultRejected); }
}

- (Core3DStoredProfileSnapshot *)storedProfileWithEntityIdentifier:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || _nativeSolidWork || _isLoading.load() || !_isSetuped
        || GLController == nil || GLController.viewer == nullptr
        || ![entityIdentifier isKindOfClass:[NSString class]] || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || ![expected isKindOfClass:[Core3DSceneSnapshot class]] || expected.selectionMode != Core3DSceneElementKindObject
        || expected.publicationSourceIdentifier.length == 0 || expected.publicationSourceIdentifier.length > 128) return nil;
    const CGSize size = GLController.drawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height) || size.width < 1 || size.height < 1
        || size.width > std::numeric_limits<std::uint32_t>::max()
        || size.height > std::numeric_limits<std::uint32_t>::max()) return nil;
    try {
        if (!entityIdentifier.UTF8String || !expected.publicationSourceIdentifier.UTF8String) return nil;
        core3d::ObjectFrameIdentity identity;
        identity.entityIdentifier.assign(entityIdentifier.UTF8String,
            [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.publicationSourceIdentifier.assign(expected.publicationSourceIdentifier.UTF8String,
            [expected.publicationSourceIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        identity.documentGeneration = expected.revisions.documentGeneration;
        identity.modelRevision = expected.revisions.modelRevision;
        const auto result = GLController.viewer->storedProfileDefinition(identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        return result ? [[Core3DStoredProfileSnapshot alloc] initWithNativeSnapshot:*result] : nil;
    } catch (...) { return nil; }
}

- (void)rebuildStoredProfile:(Core3DStoredProfileSnapshot *)original
    definition:(Core3DProfileDefinition *)definition expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DProfileConstructionResult))completion {
    if (!completion) return;
    if (![NSThread isMainThread]) {
        dispatch_async(dispatch_get_main_queue(), ^{ completion(Core3DProfileConstructionResultRejected); }); return;
    }
    if (_nativeSolidWork || _isLoading.load()) { completion(Core3DProfileConstructionResultBusy); return; }
    if (![original isKindOfClass:[Core3DStoredProfileSnapshot class]]
        || ![definition isKindOfClass:[Core3DProfileDefinition class]]) {
        completion(Core3DProfileConstructionResultRejected); return;
    }
    const auto live = [self storedProfileWithEntityIdentifier:original.entityIdentifier expected:expected];
    if (!live) { completion(Core3DProfileConstructionResultRejected); return; }
    try {
        const CGSize size = GLController.drawableSize;
        const auto originalNative = [original nativeSnapshot];
        auto requested = [definition nativeParameters];
        // The outline editor owns profile dimensions, not the construction
        // frame. Retain that frame from the exact opening authority; native
        // admission rejects stale originals or attempts to change it.
        requested.constructionFrame = originalNative.parameters.constructionFrame;
        const auto work = GLController.viewer->prepareStoredProfileRebuild(requested,
            originalNative,[live nativeSnapshot].identity,expected.revisions.presentationRevision,
            static_cast<std::uint32_t>(std::llround(size.width)),static_cast<std::uint32_t>(std::llround(size.height)));
        if (!work) { completion(Core3DProfileConstructionResultRejected); return; }
        [self runNativeSolidWork:work completion:completion];
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
        || anchor < Core3DObjectAlignmentAnchorMinimum || anchor > Core3DObjectAlignmentAnchorCenterGround) {
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

- (Core3DRigidPlacementPreparation *)prepareRigidPlacementValue:(double)value
    kind:(Core3DRigidPlacementKind)kind axis:(Core3DTransformInspectorAxis)axis
    expected:(Core3DTransformInspectorSnapshot *)snapshot {
    if(![NSThread isMainThread]||!_isSetuped||!GLController||!GLController.viewer
        ||![self core3d_canBeginCommittedEdit]
        ||![snapshot isKindOfClass:Core3DTransformInspectorSnapshot.class]
        ||!snapshot.canEditPosition||!snapshot.presentationMatchesDocument
        ||snapshot.positionEditGeneration==0||snapshot.documentEditGeneration==0||snapshot.geometryEditGeneration==0
        ||snapshot.representation!=Core3DTransformInspectorRepresentationBRep
        ||!std::isfinite(value)||std::abs(value)>1e6
        ||!std::isfinite(snapshot.metersPerUnit)||snapshot.metersPerUnit<=0
        ||(kind!=Core3DRigidPlacementKindPosition&&kind!=Core3DRigidPlacementKindRotation)
        ||(axis!=Core3DTransformInspectorAxisX&&axis!=Core3DTransformInspectorAxisY&&axis!=Core3DTransformInspectorAxisZ))return nil;
    try {
        double factor=0;
        if(!core3d::placement::MillimetersPerUnit(snapshot.metersPerUnit,factor))return nil;
        const double original=axis==Core3DTransformInspectorAxisX?snapshot.position.x
            :axis==Core3DTransformInspectorAxisY?snapshot.position.y:snapshot.position.z;
        const double raw=kind==Core3DRigidPlacementKindPosition
            ?(value==original*factor?original:value/factor):value;
        if(!std::isfinite(raw)||std::abs(raw)>Core3DTransformInspectorMaximumPositionMagnitude)return nil;
        const auto viewer=GLController.viewer;
        const auto doc=viewer->getDocument();
        if(doc.IsNull())return nil;
        const auto stamp=doc->CaptureNativePlanningStamp(viewer->canBeginCommittedEdit());
        if(!stamp)return nil;
        TDF_Label label;core3d::placement::Evidence evidence;
        if(!snapshot.entityIdentifier.UTF8String||!snapshot.definitionIdentifier.UTF8String
            ||!core3d::placement::Find(doc,snapshot.entityIdentifier.UTF8String,label)
            ||!core3d::placement::Capture(doc,label,evidence)
            ||doc->DefinitionIdentifierForLabel(label)!=snapshot.definitionIdentifier.UTF8String)return nil;
        double unit=0;
        if(!XCAFDoc_DocumentTool::GetLengthUnit(doc->Document(),unit)
            ||core3d::sweep_persistence::Bits(unit)!=core3d::sweep_persistence::Bits(snapshot.metersPerUnit))return nil;
        const auto after=doc->CaptureNativePlanningStamp(viewer->canBeginCommittedEdit());
        if(!after||!(*after==*stamp))return nil;
        auto prepared=[[Core3DRigidPlacementPreparation alloc] initPrivate];
        if(!prepared)return nil;
        prepared->_placementOwner=self;prepared->_placementSnapshot=snapshot;
        prepared->_placementViewer=viewer;prepared->_placementStamp=*stamp;
        prepared->_placementEvidence=std::move(evidence);prepared->_placementRawValue=raw;
        prepared->_placementKind=kind;prepared->_placementAxis=axis;
        return prepared;
    }catch(...){return nil;}
}
- (void)cancelRigidPlacement:(Core3DRigidPlacementPreparation *)prepared {
    if(![NSThread isMainThread]||![prepared isKindOfClass:Core3DRigidPlacementPreparation.class]
        ||prepared->_placementOwner!=self)return;
    // This exact preparation owns no right to cancel a later inspector capture.
    prepared->_placementConsumed=YES;prepared->_placementSnapshot=nil;
}
- (Core3DTransformInspectorPositionCommitResult)executeRigidPlacement:(Core3DRigidPlacementPreparation *)prepared {
    if(![NSThread isMainThread]||![prepared isKindOfClass:Core3DRigidPlacementPreparation.class]
        ||prepared->_placementOwner!=self||prepared->_placementConsumed)return Core3DTransformInspectorPositionCommitResultStale;
    // Detach/consume before any native publication or reentrant callback.
    auto snapshot=prepared->_placementSnapshot;
    prepared->_placementConsumed=YES;prepared->_placementSnapshot=nil;
    if(!_isSetuped||!GLController||!GLController.viewer||![self core3d_canBeginCommittedEdit])
        return Core3DTransformInspectorPositionCommitResultBusy;
    try {
        const auto viewer=prepared->_placementViewer.lock();
        if(!viewer||viewer!=GLController.viewer)return Core3DTransformInspectorPositionCommitResultStale;
        const auto doc=viewer->getDocument();
        if(doc.IsNull())return Core3DTransformInspectorPositionCommitResultStale;
        const auto stamp=doc->CaptureNativePlanningStamp(viewer->canBeginCommittedEdit());
        if(!stamp||!(*stamp==prepared->_placementStamp))return Core3DTransformInspectorPositionCommitResultStale;
        TDF_Label label;core3d::placement::Evidence live;
        if(!snapshot||!snapshot.entityIdentifier.UTF8String
            ||!core3d::placement::Find(doc,snapshot.entityIdentifier.UTF8String,label)
            ||!core3d::placement::Capture(doc,label,live)||!(live==prepared->_placementEvidence))
            return Core3DTransformInspectorPositionCommitResultStale;
        const auto after=doc->CaptureNativePlanningStamp(viewer->canBeginCommittedEdit());
        if(!after||!(*after==prepared->_placementStamp))return Core3DTransformInspectorPositionCommitResultStale;
        // Native opening/edit/selection epochs above reject ABA even when the
        // restored geometry and transform are byte-identical to preparation.
        // Shared touch engine performs the final exact native lease, selection,
        // presentation/unit/tool checks and owns the one ordinary transaction.
        // Never infer or override its outcome from a later diagnostic capture.
        return [self commitTransformInspectorValue:prepared->_placementRawValue
            axis:prepared->_placementAxis expectedSnapshot:snapshot
            kind:prepared->_placementKind==Core3DRigidPlacementKindPosition
                ?core3d::TransformInspectorEditKind::Position:core3d::TransformInspectorEditKind::Rotation];
    }catch(...){return Core3DTransformInspectorPositionCommitResultInternalFailure;}
}
#if DEBUG
- (NSDictionary *)debugRigidPlacementEvidence:(NSString *)entity {
    if(![NSThread isMainThread]||!GLController||!GLController.viewer||!entity.UTF8String)return nil;
    try {
        auto doc=GLController.viewer->getDocument();TDF_Label label;core3d::placement::Evidence e;
        if(!core3d::placement::Find(doc,entity.UTF8String,label)||!core3d::placement::Capture(doc,label,e))return nil;
        OcctObjectTransformState state;std::vector<std::uint8_t> geometryBytes;core3d::receipt::Digest digest;
        if(!doc->CaptureObjectTransformStateForLabel(label,state)
            ||!core3d::receipt::GeometryDigestForPolicy(state.shape,digest,false,&geometryBytes)||digest!=e.geometry)return nil;
        auto hex=[](const auto& a){return [NSString stringWithUTF8String:core3d::receipt::Hex(a).c_str()];};
        return @{@"version":@1,@"family":@(unsigned(e.family)),@"schema":@(e.schema),
            @"document":hex(e.document),@"entity":hex(e.entity),@"definition":hex(e.definition),@"feature":hex(e.feature),
            @"recipeSHA":hex(e.recipe),@"geometrySHA":hex(e.geometry),@"stateSHA":hex(e.state),
            @"geometryBytes":[NSData dataWithBytes:geometryBytes.data() length:geometryBytes.size()],
            @"transformBytes":[NSData dataWithBytes:e.transformBytes.data() length:e.transformBytes.size()],
            @"recipeBytes":[NSData dataWithBytes:e.recipeBytes.data() length:e.recipeBytes.size()],
            @"stateBytes":[NSData dataWithBytes:e.stateBytes.data() length:e.stateBytes.size()]};
    }catch(...){return nil;}
}
// Isolated real OCAF metadata probe; never changes the viewer document.
- (NSDictionary *)debugRigidPlacementAdmissionProbe {
    if(!NSThread.isMainThread)return nil;
    NSMutableDictionary *result=[NSMutableDictionary dictionary];
    try {
        double factor=123;
        result[@"overflowUnitRefused"]=@(!core3d::placement::MillimetersPerUnit(std::numeric_limits<double>::max(),factor)&&factor==0);
        result[@"zeroUnitRefused"]=@(!core3d::placement::MillimetersPerUnit(0,factor));
        result[@"negativeUnitRefused"]=@(!core3d::placement::MillimetersPerUnit(-1,factor));
        result[@"nanUnitRefused"]=@(!core3d::placement::MillimetersPerUnit(std::numeric_limits<double>::quiet_NaN(),factor));
        result[@"mmFactorExact"]=@(core3d::placement::MillimetersPerUnit(.001,factor)&&factor==1);
        result[@"metreFactorExact"]=@(core3d::placement::MillimetersPerUnit(1,factor)&&factor==1000);
        for(int kind=0;kind<4;++kind) {
            struct Scope {
                Handle(OcctDocument) owner;
                ~Scope()noexcept {try{if(!owner.IsNull()&&!owner->Document().IsNull()){
                    auto doc=owner->Document();if(doc->HasOpenCommand())doc->AbortCommand();
                    const auto app=Handle(TDocStd_Application)::DownCast(doc->Application());if(!app.IsNull())app->Close(doc);
                }}catch(...) {}}
            } scope;
            scope.owner=new OcctDocument();scope.owner->InitDoc();auto doc=scope.owner->Document();
            if(doc.IsNull())return nil;
            XCAFDoc_DocumentTool::SetLengthUnit(doc,.001);
            doc->NewCommand();
            Handle(AIS_Shape) shape=new AIS_Shape(BRepPrimAPI_MakeBox(100,80,40).Shape());
            const auto label=scope.owner->AddShape(shape,OcctGeometryRepresentation::BRep);
            if(label.IsNull()||!doc->CommitCommand())return nil;
            core3d::placement::Evidence before;
            if(!core3d::placement::Capture(scope.owner,label,before)
                ||before.family!=core3d::placement::Family::UnparametrizedSolid||before.schema!=0)return nil;
            if(kind==0){result[@"actualBareSolidAccepted"]=@YES;continue;}
            doc->NewCommand();
            if(kind==1)TDataStd_Integer::Set(label.FindChild(13),core3d::profile::SchemaID(),1);
            if(kind==2)TDataStd_Integer::Set(label.FindChild(13),core3d::sweep_persistence::SchemaID(),1);
            if(kind==3) {
                core3d::enclosure::Parameters enclosure;enclosure.metersPerUnit=.001;
                enclosure.definition.dimensions={100,80,40,3,3,5};
                if(!core3d::enclosure::Stage(doc,label,enclosure,NSUUID.UUID.UUIDString.UTF8String))return nil;
                core3d::profile::Parameters profile;profile.metersPerUnit=.001;
                profile.definition.points={{0,0},{100,0},{100,80},{0,80}};profile.definition.depth=40;
                if(!core3d::profile::Stage(doc,label,profile,NSUUID.UUID.UUIDString.UTF8String))return nil;
                // Deliberately corrupt cross-family metadata on this private
                // owner. Strict enclosure Read refuses the conflicting profile.
            }
            if(!doc->CommitCommand())return nil;
            core3d::placement::Evidence rejected;OcctObjectTransformState state;
            const bool strictRefusal=!scope.owner->CaptureObjectTransformStateForLabel(label,state);
            const bool evidenceRefusal=!core3d::placement::Capture(scope.owner,label,rejected);
            NSString *key=kind==1?@"partialProfileRefused":kind==2?@"partialSweepRefused":@"ambiguousFamiliesRefused";
            result[key]=@(strictRefusal&&evidenceRefusal);
        }
        return result;
    }catch(...){return nil;}
}

#endif

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
    return [self commitTransformInspectorValue:value axis:axis expectedSnapshot:snapshot kind:kind placementPermit:std::shared_ptr<core3d::NativeModelingCommitPermit>{}];
}
- (Core3DTransformInspectorPositionCommitResult)commitTransformInspectorValue:(double)value
    axis:(Core3DTransformInspectorAxis)axis expectedSnapshot:(Core3DTransformInspectorSnapshot *)snapshot
    kind:(core3d::TransformInspectorEditKind)kind placementPermit:(std::shared_ptr<core3d::NativeModelingCommitPermit>)permit {
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
            ->commitTransformInspectorPosition(request,std::move(permit));
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
    if (axis < 0 || axis > 2) return NO;
    return [self previewMirrorAxis:static_cast<Core3DMirrorAxis>(axis)
                        backward:backward];
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

- (void)debugSetLinearArrayProfileCopyFault:(NSInteger)mode {
    [GLController debugSetLinearArrayProfileCopyFault:mode];
}

- (void)debugSetMirrorProfileCopyFault:(NSInteger)mode {
    [GLController debugSetMirrorProfileCopyFault:mode];
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

- (void)debugSetRadialArrayProfileCopyFault:(NSInteger)mode {
    [GLController debugSetRadialArrayProfileCopyFault:mode];
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

- (BOOL)debugClearReadyMirrorSourceSelection {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) { return NO; }
    const auto viewer = GLController.viewer;
    if (!viewer || viewer->AisContext().IsNull() || !viewer->getObjectInteractor()) { return NO; }
    const auto interactor = viewer->getObjectInteractor();
    if (!interactor->canFrameMirrorPreview(interactor->mirrorPreviewGeneration())) { return NO; }
    try {
        viewer->AisContext()->ClearSelected(Standard_False);
        return viewer->AisContext()->NbSelected() == 0;
    } catch (...) { return NO; }
}

- (BOOL)debugMutateFirstMirrorSourcePersistedTransform {
    return [GLController debugMutateFirstMirrorSourcePersistedTransform];
}

// Insert within existing DEBUG methods, with NativeTriangleContacts.hpp included
// under DEBUG. Numeric probes have no document, viewer or selection access.
- (NSData *)debugTriangleContactPairDecisions:(NSData *)coordinates {
    constexpr NSUInteger stride=18*sizeof(double);
    if(coordinates.length==0 || coordinates.length%stride!=0 || coordinates.length/stride>20000)return nil;
    try {
        const auto* bytes=static_cast<const std::uint8_t*>(coordinates.bytes);
        std::vector<std::uint8_t> result;result.reserve(coordinates.length/stride);
        for(NSUInteger offset=0;offset<coordinates.length;offset+=stride) {
            std::array<core3d::meshcheck::Triangle,2> pair;NSUInteger index=offset;
            for(auto& t:pair)for(auto& p:t)for(auto& value:p) {
                std::uint64_t bits=0;for(unsigned k=0;k<8;++k)bits|=std::uint64_t(bytes[index++])<<(8*k);
                std::memcpy(&value,&bits,sizeof(value));
            }
            // Expected decisions never enter this method. Every result is
            // independently calculated from the18 supplied coordinates.
            try {result.push_back(core3d::meshcheck::exact::unexpected(pair[0],pair[1])?1:0);}
            catch(const std::invalid_argument&){result.push_back(2);}
            catch(const std::length_error&){result.push_back(3);}
            catch(...){result.push_back(4);}
        }
        return [NSData dataWithBytes:result.data() length:result.size()];
    } catch(...) {return nil;}
}

- (NSDictionary<NSString *, id> *)debugTriangleContactMesh:(NSData *)coordinates mode:(NSInteger)mode {
    constexpr NSUInteger stride=9*sizeof(double);
    if(coordinates.length%stride!=0 || coordinates.length/stride>20001 || mode<0 || mode>8)return nil;
    try {
        const auto* bytes=static_cast<const std::uint8_t*>(coordinates.bytes);
        std::vector<core3d::meshcheck::Triangle> triangles(coordinates.length/stride);NSUInteger index=0;
        for(auto& t:triangles)for(auto& p:t)for(auto& value:p) {
            std::uint64_t bits=0;for(unsigned k=0;k<8;++k)bits|=std::uint64_t(bytes[index++])<<(8*k);
            std::memcpy(&value,&bits,sizeof(value));
        }
        const auto original=coordinates.copy;
        std::atomic_bool cancelled{mode==1};core3d::meshcheck::ContactLimits limits;
        if(mode==2)limits.maximumPairs=0;
        if(mode==3)limits.maximumContacts=0;
        if(mode==4)limits.duration=std::chrono::milliseconds(0);
        if(mode==5)limits.maximumTriangles=1;
        if(mode==6)limits.maximumTriangles=20001;
        if(mode==7)limits.duration=std::chrono::milliseconds::max();
        if(mode==8)limits.duration=std::chrono::milliseconds(1);
        core3d::meshcheck::ContactReport report;
        // Seed stale data to prove every non-ready path clears the output.
        report.triangleCount=999;report.candidatePairs=999;report.unexpectedPairs={{99,100}};
        const auto started=std::chrono::steady_clock::now();
        const auto status=core3d::meshcheck::AnalyzeTriangleContacts(triangles,report,cancelled,limits);
        NSString* state=nil;
        switch(status) {
            case core3d::meshcheck::ContactStatus::Ready:state=@"ready";break;
            case core3d::meshcheck::ContactStatus::Invalid:state=@"invalid";break;
            case core3d::meshcheck::ContactStatus::TooLarge:state=@"tooLarge";break;
            case core3d::meshcheck::ContactStatus::Cancelled:state=@"cancelled";break;
            case core3d::meshcheck::ContactStatus::TimedOut:state=@"timedOut";break;
        }
        NSMutableArray* pairs=[NSMutableArray arrayWithCapacity:report.unexpectedPairs.size()];
        for(const auto& pair:report.unexpectedPairs)[pairs addObject:@[@(pair.first),@(pair.second)]];
        // Re-encode every input value after scanning: byte equality, including
        // NaN payloads/signed zero, is stronger than floating-point equality.
        NSMutableData* after=[NSMutableData dataWithLength:coordinates.length];auto* output=static_cast<std::uint8_t*>(after.mutableBytes);index=0;
        for(const auto& t:triangles)for(const auto& p:t)for(const auto value:p) {
            std::uint64_t bits;std::memcpy(&bits,&value,sizeof(bits));
            for(unsigned k=0;k<8;++k)output[index++]=std::uint8_t(bits>>(8*k));
        }
        const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
        return @{ @"state":state,@"triangles":@(report.triangleCount),@"candidatePairs":@(report.candidatePairs),
                  @"contacts":pairs,@"inputUnchanged":@([original isEqualToData:after]),@"elapsedSeconds":@(elapsed) };
    } catch(...) {return nil;}
}

- (NSDictionary<NSString *, NSArray<NSNumber *> *> *)debugWindingGeometryProbe {
    if (![NSThread isMainThread]) return nil;
    try {
        const auto plan = core3d::debug::RunNativeWindingPlanProbe();
        const auto candidate = core3d::debug::RunNativeWindingCandidateProbe();
        NSMutableArray<NSNumber *> *planValues = [NSMutableArray arrayWithCapacity:plan.size()];
        NSMutableArray<NSNumber *> *candidateValues = [NSMutableArray arrayWithCapacity:candidate.size()];
        for (const bool value : plan) [planValues addObject:@(value)];
        for (const bool value : candidate) [candidateValues addObject:@(value)];
        return @{ @"plan": planValues, @"candidate": candidateValues };
    } catch (...) { return nil; }
}

- (BOOL)debugBeginEmptyBooleanWithGizmoType:(PrimitiveGizmoType)gizmoType {
    core3d::BooleanAction action = core3d::BooleanAction::BooleanSubtract;
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || !Core3DTryBooleanActionForGizmo(gizmoType, action)) return NO;
    [self setGizmoType:gizmoType];
    if (_currentGizmoType != gizmoType || [GLController getGizmoType] != gizmoType
        || GLController.viewer == nullptr) return NO;
    const BOOL result = GLController.viewer->debugCycleBooleanSelection(action, {}, true);
    self.can_apply = [GLController canApplyBoolean];
    [GLController debugRequestRender];
    [self viewDidChangeViewportPresentationState];
    [self sendNotifyUIState:UIStateChangingApply];
    return result;
}

- (BOOL)debugCycleBooleanSelectionForEntityIdentifier:(NSString *)entityIdentifier {
    core3d::BooleanAction action = core3d::BooleanAction::BooleanSubtract;
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil
        || GLController.viewer == nullptr
        || !Core3DTryBooleanActionForGizmo(_currentGizmoType, action)
        || [GLController getGizmoType] != _currentGizmoType
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || entityIdentifier.UTF8String == nullptr) return NO;
    try {
        const std::string entity(entityIdentifier.UTF8String,
            [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        const BOOL result = GLController.viewer->debugCycleBooleanSelection(action, entity, false);
        self.can_apply = [GLController canApplyBoolean];
        [GLController debugRequestRender];
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply];
        return result;
    } catch (...) { return NO; }
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

- (void)debugSetBooleanMetadataFailurePhase:(NSUInteger)phase {
    [GLController debugSetBooleanMetadataFailurePhase:phase];
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

- (void)debugSimulateMirrorMemoryWarning {
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
        if (!aStatus.active) {
            // Memory pressure retires an ordinary preview through cancelMirror.
            // Keep the public tool synchronized with its native inactive owner.
            [self completeOperationInteraction];
        } else {
            // Failed cleanup and unknown commits retain their recovery controls.
            self.can_apply = aStatus.canApply;
            [self viewDidChangeViewportPresentationState];
            [self sendNotifyUIState:UIStateChangingApply];
        }
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
        if (nativeGizmoType == PrimitiveGizmoTypeNone
            && (_currentGizmoType == PrimitiveGizmoTypeMoveRotate
                || _currentGizmoType == PrimitiveGizmoTypeScale)) {
            // Entering an object gizmo can attach a browser-selected target.
            // Derive action availability after the public tool is final.
            [GLController refreshSelectionState];
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
        if (_queuedAssetRequest != nil) {
            const Core3DAssetLoadResult result = [self core3d_startQueuedRequest:_queuedAssetRequest];
            if (result != Core3DAssetLoadResultSuccess) [self viewDidFailToLoadFromBundle:result];
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

// Deferred setup starts the exact retained request directly. Core teardown
// abandons only its own native load owner before releasing the rendering controller.

- (BOOL)core3d_canReserveQueuedInput:(Core3DQueuedAssetInput *)input
                  fromGLController:(GLViewController *)controller {
    if (![NSThread isMainThread] || controller == nil || _glController != controller
        || _nativeSolidWork || _objectAlignmentWork || !_isSetuped) return NO;
    if (_queuedAssetRequest == nil) return !_isLoading.load();
    return _queuedRequestGL == controller && _queuedAssetRequest.input == input
        && [_queuedAssetRequestSlot ownsRequest:_queuedAssetRequest];
}

- (BOOL)core3d_hasCompetingLoadOrControllerWork {
    return ![NSThread isMainThread] || _queuedAssetRequest != nil
        || _isLoading.load() || _nativeSolidWork || _objectAlignmentWork;
}

- (Core3DAssetLoadResult)tryLoadFromBundle:(NSURL *)bundleURL {
    // Admission rejection cannot cancel a worker, alter UI, or invoke the
    // accepted request's lifecycle callback. Check before freezing inputs.
    if ([self core3d_hasCompetingLoadOrControllerWork])
        return Core3DAssetLoadResultBusy;
    Core3DQueuedAssetInput *input = [Core3DQueuedAssetInput freezeBundleURL:bundleURL];
    if (input == nil) return Core3DAssetLoadResultInvalidData;
    return [self core3d_acceptQueuedInput:input];
}

- (Core3DAssetLoadResult)tryLoadFromAssetFile:(NSURL *)fileURL
                          expectedByteCount:(unsigned long long)byteCount
                              expectedSHA256:(NSString *)sha256 {
    if ([self core3d_hasCompetingLoadOrControllerWork])
        return Core3DAssetLoadResultBusy;
    Core3DQueuedAssetInput *input = [Core3DQueuedAssetInput freezeVerifiedFile:fileURL
        expectedByteCount:byteCount expectedSHA256:sha256];
    if (input == nil) return Core3DAssetLoadResultInvalidData;
    return [self core3d_acceptQueuedInput:input];
}

- (void)loadFromBundle:(NSURL *)bundleURL {
    const Core3DAssetLoadResult admission = [self tryLoadFromBundle:bundleURL];
    if (admission != Core3DAssetLoadResultSuccess && [NSThread isMainThread]
        && _queuedAssetRequest == nil && !_isLoading.load()) {
        [self reconcilePublicStateAfterDocumentLifecycleActivatingPassiveTool:NO];
        [self viewDidFailToLoadFromBundle:admission];
    }
}

- (void)loadFromAssetFile:(NSURL *)fileURL
        expectedByteCount:(unsigned long long)byteCount
            expectedSHA256:(NSString *)sha256 {
    const Core3DAssetLoadResult admission = [self tryLoadFromAssetFile:fileURL
        expectedByteCount:byteCount expectedSHA256:sha256];
    if (admission != Core3DAssetLoadResultSuccess && [NSThread isMainThread]
        && _queuedAssetRequest == nil && !_isLoading.load()) {
        [self reconcilePublicStateAfterDocumentLifecycleActivatingPassiveTool:NO];
        [self viewDidFailToLoadFromBundle:admission];
    }
}

- (Core3DAssetLoadResult)core3d_acceptQueuedInput:(Core3DQueuedAssetInput *)input {
    if ([self core3d_hasCompetingLoadOrControllerWork])
        return Core3DAssetLoadResultBusy;
    if (_queuedAssetRequestSlot == nil)
        _queuedAssetRequestSlot = [[Core3DQueuedAssetRequestSlot alloc] init];
    Core3DQueuedAssetRequest *request = [_queuedAssetRequestSlot acceptInput:input];
    if (request == nil) return Core3DAssetLoadResultInternalFailure;
    _queuedAssetRequest = request;
    _queuedRequestGL = [_glController isKindOfClass:GLViewController.class] ? GLController : nil;
    _isLoading = true;
    // There is no initialized native document before setup. The Core slot owns
    // this deferred request and suppresses competing controller work. Reserve
    // the actual document in GL before any private work is submitted.
    if (!_isSetuped) return Core3DAssetLoadResultSuccess;
    return [self core3d_startQueuedRequest:request];
}

- (Core3DAssetLoadResult)core3d_startQueuedRequest:(Core3DQueuedAssetRequest *)request {
    if (![NSThread isMainThread] || !_isSetuped
        || _queuedAssetRequest != request
        || ![_queuedAssetRequestSlot ownsRequest:request]
        || ![_queuedAssetRequestSlot claimStartForRequest:request])
        return Core3DAssetLoadResultBusy;
    // Recheck the real slots on deferred setup. A cancellation flag alone is
    // not native settlement, and Ready-only admission never cancels them.
    if (_nativeSolidWork || _objectAlignmentWork || _queuedRequestGL == nil
        || _glController != _queuedRequestGL) {
        [_queuedAssetRequestSlot finishRequest:request];
        _queuedAssetRequest = nil;
        _isLoading = false;
        return Core3DAssetLoadResultBusy;
    }
    __weak typeof(self) weakSelf = self;
    __weak GLViewController *originGL = _queuedRequestGL;
    Core3DQueuedAssetLoadOwner *owner = nil;
    const Core3DAssetLoadResult admission = [GLController
        acceptQueuedAssetInput:request.input owner:&owner
        progress:^(BOOL cleanupPending) {
            __strong typeof(weakSelf) controller = weakSelf;
            if (controller != nil && controller->_queuedAssetRequest == request
                && controller.glController == originGL)
                [controller viewDidChangeAssetLoadCleanupPending:cleanupPending];
        }
        completion:^(Core3DAssetLoadResult result) {
            __strong typeof(weakSelf) controller = weakSelf;
            if (controller == nil || ![NSThread isMainThread]
                || controller->_queuedAssetRequest != request
                || ![controller->_queuedAssetRequestSlot ownsRequest:request]) return;
            // Private cleanup and native settlement are complete. Keep Core's
            // accepted slot through reconciliation and progress feedback so a
            // reentrant load cannot overtake the prior document lifecycle.
            if (originGL != nil && controller.glController == originGL) {
                if (result == Core3DAssetLoadResultSuccess) {
                    [originGL fitAll];
                    [controller activatePassiveStateAfterDocumentReplacement];
                } else {
                    [controller reconcilePublicStateAfterDocumentLifecycleActivatingPassiveTool:NO];
                }
                if (controller.glController == originGL)
                    [controller viewDidChangeAssetLoadCleanupPending:NO];
            }
            if (![controller->_queuedAssetRequestSlot finishRequest:request]) return;
            controller->_queuedNativeLoadOwner = nil;
            controller->_queuedAssetRequest = nil;
            controller->_isLoading = false;
            // A replacement GL is a different editor lifecycle. Release this
            // request but never publish the old terminal event into that editor.
            if (originGL == nil || controller.glController != originGL) return;
            // Terminal callbacks may immediately begin a subsequent load. No
            // prior native reconciliation or progress follows that callback.
            if (result == Core3DAssetLoadResultSuccess) [controller viewDidLoadFromBundle];
            else [controller viewDidFailToLoadFromBundle:result];
        }];
    if (admission == Core3DAssetLoadResultSuccess) {
        // GL guarantees asynchronous completion after a successful admission.
        _queuedNativeLoadOwner = owner;
    } else {
        [_queuedAssetRequestSlot finishRequest:request];
        _queuedAssetRequest = nil;
        _isLoading = false;
    }
    return admission;
}

- (void)viewDidChangeAssetLoadCleanupPending:(BOOL)pending {}
- (void)retryAssetLoadCleanup {
    if ([NSThread isMainThread] && _queuedNativeLoadOwner != nil && _queuedRequestGL != nil)
        [_queuedRequestGL retryQueuedAssetCleanup];
}

#ifdef DEBUG
- (void)debugPauseQueuedAssetAdoption:(BOOL)paused {
    if ([NSThread isMainThread] && [_glController isKindOfClass:GLViewController.class])
        [GLController debugPauseQueuedAssetAdoption:paused];
}
- (void)debugFailQueuedAssetCleanup:(NSUInteger)count {
    if ([NSThread isMainThread] && [_glController isKindOfClass:GLViewController.class])
        [GLController debugFailQueuedAssetCleanup:count];
}
- (NSDictionary<NSString *, id> *)debugQueuedAssetLoadState {
    if (![NSThread isMainThread] || ![_glController isKindOfClass:GLViewController.class]) return @{};
    return [GLController debugQueuedAssetLoadState];
}
#endif

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
