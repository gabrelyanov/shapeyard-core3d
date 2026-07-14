//
//  Core3DViewController.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 28.03.2024.
//

#import "GLViewController.h"
#import "Core3DViewController.h"
#import "Core3DViewController+PrimitiveManager.h"
#import "Core3DViewController+GLViewControllerProtocol.h"
#import <Core3D/AssetBundle.h>
#import "../Viewport/Core3DSceneSnapshotFactory.hpp"
#import "Core3DTransformInspectorSnapshotFactory.hpp"

#include "GLViewController+Trick.h"
#include "BooleanOperationController.hpp"
#include "TransformInspectorMeasurementController.hpp"
#include "../OCCTKit/OcctDocument.h"
#include "../Common/dispatch_cancelable_block.h"
#include "XCAFDoc_DocumentTool.hxx"
#include "XCAFDoc_ColorTool.hxx"
#include "XCAFDoc_LayerTool.hxx"
#include "XCAFDoc_VisMaterial.hxx"
#include "XCAFDoc_VisMaterialTool.hxx"
#include "Image_Texture.hxx"
#include "BRep_Tool.hxx"
#include "BRepTools.hxx"
#include "BRepPrimAPI_MakeBox.hxx"
#include "BRepPrimAPI_MakePrism.hxx"
#include "BRepBuilderAPI_MakePolygon.hxx"
#include "BRepBuilderAPI_MakeFace.hxx"
#include "BRep_Builder.hxx"
#include "TopoDS_Compound.hxx"
#include "TopoDS_CompSolid.hxx"
#include "TopoDS.hxx"
#include "TopoDS_Face.hxx"
#include "Poly_Triangle.hxx"
#include "Poly_Triangulation.hxx"
#include "NCollection_Buffer.hxx"
#include "TDataStd_Integer.hxx"
#include "TDataStd_Real.hxx"
#include "TDF_LabelSequence.hxx"
#include "Standard_GUID.hxx"
#include "gp_Ax2.hxx"
#include "gp_Dir.hxx"
#include "gp_Pnt2d.hxx"
#include "gp_Trsf.hxx"
#include "gp_Vec.hxx"
#include "TopLoc_Location.hxx"
#include "TopExp_Explorer.hxx"
#include "XCAFPrs_DocumentExplorer.hxx"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <string>
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
    const BOOL supportsEmissiveTextureEditing) {
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
supportsEmissiveTextureEditing:supportsEmissiveTextureEditing];
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

@interface Core3DViewController () {
    BOOL _isSetuped;
    NSURL *_shouldLoadBundleUrl;
    NSURL *_shouldLoadAssetFileURL;
    unsigned long long _shouldLoadAssetByteCount;
    NSString *_shouldLoadAssetSHA256;
    std::atomic_bool _isLoading;
#ifdef DEBUG
    NSUInteger _debugMaximumTextureAuthoringObjects;
#endif
}

@end

@implementation Core3DViewController {
    __weak dispatch_cancelable_block_t _uiStateChangingBlock;
    UIStateChanging _sendingState;

    __weak dispatch_cancelable_block_t _modifiedAssetBlock;
}

- (void)dealloc {
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

-(void) updateSelectionWithMaterial:(Core3DMaterial*)material color:(Core3DColor*)color {
    if ((self.selectedModelCapabilities
            & Core3DModelCapabilityMaterial) == 0) {
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
	            editable, YES, YES, YES);
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
			& Core3DModelCapabilityMaterial) == 0) {
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
        if (!nativeMaterial.MetallicRoughnessTexture.IsNull()
            || !nativeMaterial.OcclusionTexture.IsNull()
            || !nativeMaterial.NormalTexture.IsNull()
            || !Core3DApplyNativePBRScalars(
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
                doc->SupportsEmissiveTextureEditingForLabel(style.label));
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
    if (transaction.IsNull() || transaction->HasOpenCommand()) {
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
            doc->SupportsEmissiveTextureEditingForLabel(style.label));
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
    if (transaction.IsNull() || transaction->HasOpenCommand()) {
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
            doc->SupportsEmissiveTextureEditingForLabel(style.label));
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
    if (transaction.IsNull() || transaction->HasOpenCommand()) {
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
            || !doc->SupportsEmissiveTextureEditingForLabel(label)) {
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
        if (!nativeMaterial.EmissiveTexture.IsNull()) {
            const std::string existingIdentifier(
                nativeMaterial.EmissiveTexture->TextureId().ToCString());
            const auto cached = identicalExistingTextureByIdentifier.find(
                existingIdentifier);
            if (cached != identicalExistingTextureByIdentifier.end()) {
                hasIdenticalTexture = cached->second;
            } else {
                hasIdenticalTexture = Core3DTexturesMatch(
                    nativeMaterial.EmissiveTexture, authoredTexture);
                identicalExistingTextureByIdentifier.emplace(
                    existingIdentifier, hasIdenticalTexture);
            }
        }
        const bool isOwnedIdenticalTexture = hasIdenticalTexture
            && doc->SupportsScalarPBRMaterialEditingForLabel(label);
        const bool isFirstEmissiveTexture =
            nativeMaterial.EmissiveTexture.IsNull();
        bool autoPromotedEmissiveFactor =
            doc->IsEmissiveTextureFactorAutoPromotedForLabel(label);
        if (isFirstEmissiveTexture
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
        nativeMaterial.EmissiveTexture = authoredTexture;
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
                    style.prevalidatedBaseColorTexture, authoredTexture});
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
            doc->SupportsEmissiveTextureEditingForLabel(style.label));
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

-(BOOL)clearSelectionEmissiveTextureWithError:(NSError* _Nullable * _Nullable)error {
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
    if (transaction.IsNull() || transaction->HasOpenCommand()) {
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
            || !doc->SupportsEmissiveTextureEditingForLabel(label)) {
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
        const bool changed = !nativeMaterial.EmissiveTexture.IsNull();
        const bool autoPromotedEmissiveFactor =
            doc->IsEmissiveTextureFactorAutoPromotedForLabel(label);
        nativeMaterial.EmissiveTexture.Nullify();
        if (autoPromotedEmissiveFactor) {
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
                    style.label, Standard_False)) {
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
            doc->SupportsEmissiveTextureEditingForLabel(style.label));
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
            [states addObject:state];
        }
    } catch (...) {
        return @[];
    }
    return states;
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

- (NSNumber *_Nullable)debugDocumentMetersPerUnit {
    auto document = GLController.viewer->getDocument()->ChangeDocument();
    Standard_Real metersPerUnit = 0.0;
    if (document.IsNull()
        || !XCAFDoc_DocumentTool::GetLengthUnit(document, metersPerUnit)) {
        return nil;
    }
    return @(metersPerUnit);
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

- (NSDictionary<NSString *, NSNumber *> *)debugFramebufferStatistics {
    return [GLController debugFramebufferStatistics];
}

- (NSInteger)debugSelectedShapeCount {
    return [GLController debugSelectedShapeCount];
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
        // invalidation callback. Publish the Metal snapshot/UI state here
        // without requesting a duplicate renderer-neutral invalidation.
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingGizmo
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
        || mode < 0 || mode > 2) {
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

- (void)setSelectionType:(PrimitiveSelectionType)type {
	if (_currentSelectionType != type) {
		[GLController setSelectionType:type];
		if ([GLController getSelectionType] != type) {
			if (_currentGizmoType == PrimitiveGizmoTypeMirror
				|| _currentGizmoType
					== PrimitiveGizmoTypeLinearArray
				|| _currentGizmoType == PrimitiveGizmoTypeShell) {
				const Core3DModelingPreviewStatus aStatus =
					[self modelingPreviewStatusForGizmoType:
						_currentGizmoType];
				self.can_apply = aStatus.canApply;
				[self viewDidChangeViewportPresentationState];
				[self sendNotifyUIState:UIStateChangingSelection
									 | UIStateChangingGizmo
									 | UIStateChangingApply];
			}
			return;
		}
		_currentSelectionType = type;

        _availableGizmoTypes = @[];
        switch (_currentSelectionType) {
            case PrimitiveSelectionTypeShape:
                if ([GLController isSelected]) {
                    _availableGizmoTypes = @[@(PrimitiveGizmoTypeMoveRotate),
                                             @(PrimitiveGizmoTypeScale),
                                             @(PrimitiveGizmoTypeChamfer),
                                             @(PrimitiveGizmoTypeMirror),
                                             @(PrimitiveGizmoTypeLinearArray),
                                             @(PrimitiveGizmoTypeSubtract),
                                             @(PrimitiveGizmoTypeUnion),
                                             @(PrimitiveGizmoTypeIntersect),
                                             @(PrimitiveGizmoTypeMaterial)];
                } else {
                    [self setGizmoType:PrimitiveGizmoTypeMoveRotate];
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
                [GLController deselectAll];
            }
                _can_delete = NO;
                _can_duplicate = NO;
                [self setGizmoType:PrimitiveGizmoTypeNone];
                // ^^ in this case setGizmoType may not be called, because gizmoType doesn't change
                //    when the selection type is changed [PrimitiveGizmoTypeChamfer -> PrimitiveGizmoTypeChamfer],
                //    so we notify ui state manually
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
                [GLController deselectAll];
            }
                _can_delete = NO;
                _can_duplicate = NO;
                [self setGizmoType:PrimitiveGizmoTypeNone];
                break;

            default:
                break;
        }

        // GLController emits render invalidations while this method is still
        // reconciling the public selection/gizmo state. Observe once more only
        // after that state is authoritative so alternate renderers cannot stay
        // gated by an intermediate mode until the debounced UI refresh.
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingSelection
                                 | UIStateChangingGizmo
                                 | UIStateChangingDelete
                                 | UIStateChangingDuplicate];
    }
}

- (void)setGizmoType:(PrimitiveGizmoType)type {
    if (_currentGizmoType != type) {
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

- (void)loadFromBundle:(NSURL *)bundleUrl {
    if(bundleUrl == nil) {
        [self viewDidFailToLoadFromBundle:Core3DAssetLoadResultInternalFailure];
        return;
    }
    
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
                [strongSelf viewDidFailToLoadFromBundle:result];
                return;
            }
            [(GLViewController *)strongSelf.glController fitAll];
            if ([strongSelf currentGizmoType] == PrimitiveGizmoTypeNone) {
                [strongSelf setGizmoType:PrimitiveGizmoTypeMoveRotate];
            }
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
            [strongSelf viewDidFailToLoadFromBundle:result];
            return;
        }
        [(GLViewController *)strongSelf.glController fitAll];
        if ([strongSelf currentGizmoType] == PrimitiveGizmoTypeNone) {
            [strongSelf setGizmoType:PrimitiveGizmoTypeMoveRotate];
        }
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
