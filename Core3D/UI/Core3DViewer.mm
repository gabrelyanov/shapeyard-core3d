//
//  Core3DViewer.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 11.04.2024.
//

#include "Core3DViewer.h"

#include <BRepFilletAPI_MakeFillet.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_DisplayMode.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Aspect_NeutralWindow.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Image_AlienPixMap.hxx>
#include <V3d_View.hxx>
#include <V3d_DirectionalLight.hxx>
#include <V3d_AmbientLight.hxx>

#include "BRepPrimAPI_MakeCylinder.hxx"
#include "BRepPrimAPI_MakeBox.hxx"
#include "BRepPrimAPI_MakeSphere.hxx"
#include "BRepPrimAPI_MakeCone.hxx"
#include "BRepPrimAPI_MakeTorus.hxx"
#include "BRepAlgoAPI_Cut.hxx"
#include "BRepAlgoAPI_Fuse.hxx"
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterial.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepBndLib.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_MapOfShape.hxx>


#include <BRepBuilderAPI_GTransform.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <new>
#include <string>
#include <sys/stat.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#import <UIKit/UIKit.h>
#import <CoreGraphics/CoreGraphics.h>
#import <CoreImage/CIFilter.h>

namespace core3d {

namespace {

#ifdef DEBUG
std::atomic<Standard_Size> gBoundedProjectTopologyValidationCount{0};
std::atomic<Standard_Size> gGeometricBRepValidationCount{0};
#endif

bool RunGeometricBRepValidation(const TopoDS_Shape& shape) {
#ifdef DEBUG
    gGeometricBRepValidationCount.fetch_add(1, std::memory_order_relaxed);
#endif
    BRepCheck_Analyzer analyzer(shape, Standard_True);
    return analyzer.IsValid();
}

bool IsTopologicallyValid(const TopoDS_Shape& shape) {
    if (shape.IsNull()) {
        return false;
    }
    try {
        return RunGeometricBRepValidation(shape);
    } catch (...) {
        return false;
    }
}

bool TryCountDisplayedModelShapes(
    const Handle(Core3DContext)& context,
    Standard_Size& count) noexcept {
    count = 0;
    if (context.IsNull()) {
        return false;
    }

    try {
        OCC_CATCH_SIGNALS
        AIS_ListOfInteractive displayedShapes;
        context->DisplayedObjects(AIS_KOI_Shape, -1, displayedShapes);
        for (AIS_ListIteratorOfListOfInteractive displayed(displayedShapes);
             displayed.More(); displayed.Next()) {
            const Handle(AIS_Shape) modelShape =
                Handle(AIS_Shape)::DownCast(displayed.Value());
            if (!modelShape.IsNull() && !modelShape->Shape().IsNull()) {
                ++count;
            }
        }
        return true;
    } catch (...) {
        // If the graphics context cannot provide a trustworthy count, preserve
        // the user's camera instead of risking an unexpected reframe.
        count = 0;
        return false;
    }
}

constexpr NSUInteger kMaximumPrimitiveCount = 1024;
constexpr std::uint64_t kMaximumProjectDocumentBytes =
    256ull * 1024ull * 1024ull;
constexpr Standard_Integer kMaximumVisualMaterialDefinitions = 2048;
constexpr Standard_Size kMaximumAggregateTextureBytes =
    128ull * 1024ull * 1024ull;
constexpr Standard_Real kMaximumEmissionFactor = 65504.0;
constexpr Standard_Size kMaximumAssemblyTraversalDepth = 128;
constexpr Standard_Size kMaximumAssemblyTraversalNodes = 32768;
constexpr Standard_Size kMaximumShapeDefinitions = 4096;
constexpr Standard_Size kMaximumSubshapesPerDefinition = 250000;
constexpr Standard_Size kMaximumDocumentLabels = 100000;
constexpr Standard_Size kMaximumDecodedTextureBytes =
    128ull * 1024ull * 1024ull;
constexpr double kMinimumPrimitiveScale = 1.0e-4;
constexpr double kMaximumPrimitiveScale = 1.0e4;
constexpr double kMinimumPrimitiveDimension = 1.0e-3;
constexpr double kMaximumPrimitiveDimension = 1.0e6;
constexpr double kMaximumPositionMagnitude = 1.0e6;
constexpr double kMaximumRotationMagnitude = 360000.0;

struct PreparedPrimitive {
    Handle(AIS_Shape) presentation;
    Graphic3d_NameOfMaterial material;
    Quantity_NameOfColor color;
};

bool ReadFiniteJSONNumber(id value, double& result) {
    if (![value isKindOfClass:[NSNumber class]]) {
        return false;
    }

    NSNumber* number = static_cast<NSNumber*>(value);
    if (CFGetTypeID((__bridge CFTypeRef)number) == CFBooleanGetTypeID()) {
        return false;
    }

    result = number.doubleValue;
    return std::isfinite(result);
}

bool ReadFiniteVector3(id value, std::array<double, 3>& result) {
    if (![value isKindOfClass:[NSArray class]]) {
        return false;
    }

    NSArray* values = static_cast<NSArray*>(value);
    if (values.count != result.size()) {
        return false;
    }

    for (NSUInteger index = 0; index < values.count; ++index) {
        if (!ReadFiniteJSONNumber(values[index], result[index])) {
            return false;
        }
    }
    return true;
}

bool IsBoundedMagnitude(const std::array<double, 3>& values, double maximum) {
    for (double value : values) {
        if (std::abs(value) > maximum) {
            return false;
        }
    }
    return true;
}

bool IsValidScale(const std::array<double, 3>& scale) {
    for (double value : scale) {
        if (value < kMinimumPrimitiveScale || value > kMaximumPrimitiveScale) {
            return false;
        }
    }
    return true;
}

bool IsValidDimension(double value) {
    return std::isfinite(value)
        && value >= kMinimumPrimitiveDimension
        && value <= kMaximumPrimitiveDimension;
}

void AbortCommandNoThrow(const Handle(TDocStd_Document)& document) noexcept {
    if (document.IsNull()) {
        return;
    }
    try {
        if (document->HasOpenCommand()) {
            document->AbortCommand();
        }
    } catch (...) {
        // Preserve the original transaction failure.
    }
}

bool StagePreparedPrimitives(
    const Handle(OcctDocument)& document,
    const std::vector<PreparedPrimitive>& primitives) {
    if (document.IsNull() || primitives.empty()) {
        return false;
    }

    Handle(TDocStd_Document) transaction = document->ChangeDocument();
    if (transaction.IsNull() || transaction->HasOpenCommand()) {
        return false;
    }

    try {
        OCC_CATCH_SIGNALS
        transaction->NewCommand();
        if (!transaction->HasOpenCommand()) {
            return false;
        }

        for (const PreparedPrimitive& primitive : primitives) {
            if (primitive.presentation.IsNull()
                || primitive.presentation->Shape().IsNull()) {
                AbortCommandNoThrow(transaction);
                return false;
            }

            const TDF_Label label = document->AddShape(primitive.presentation);
            if (label.IsNull()) {
                AbortCommandNoThrow(transaction);
                return false;
            }
            document->SaveObjectMaterial(label, primitive.material);
            document->SaveObjectColor(label, primitive.color);
        }

        return true;
    } catch (const Standard_Failure&) {
        AbortCommandNoThrow(transaction);
        return false;
    } catch (const std::exception&) {
        AbortCommandNoThrow(transaction);
        return false;
    } catch (...) {
        AbortCommandNoThrow(transaction);
        return false;
    }
}

bool PublishPreparedPrimitives(
    const Handle(OcctDocument)& document,
	const Handle(Core3DContext)& context,
	const std::vector<PreparedPrimitive>& primitives,
	Standard_Integer selectionMode) {
	if (context.IsNull() || !StagePreparedPrimitives(document, primitives)) {
		return false;
	}

	Standard_Boolean displayedAll = Standard_False;
	try {
		OCC_CATCH_SIGNALS
		for (const PreparedPrimitive& primitive : primitives) {
			context->Display(
				primitive.presentation,
				AIS_Shaded,
				selectionMode,
				Standard_False);
		}
		displayedAll = Standard_True;
	} catch (...) {
		displayedAll = Standard_False;
	}

	Handle(TDocStd_Document) transaction = document->ChangeDocument();
	if (!displayedAll) {
		for (const PreparedPrimitive& primitive : primitives) {
			try {
				context->Remove(primitive.presentation, Standard_False);
			} catch (...) {
			}
		}
		AbortCommandNoThrow(transaction);
		return false;
	}

	Standard_Boolean committed = Standard_False;
	try {
		committed = !transaction.IsNull()
			&& transaction->HasOpenCommand()
			&& transaction->CommitCommand();
	} catch (...) {
		committed = Standard_False;
	}
	if (!committed) {
		for (const PreparedPrimitive& primitive : primitives) {
			try {
				context->Remove(primitive.presentation, Standard_False);
			} catch (...) {
			}
		}
		AbortCommandNoThrow(transaction);
		return false;
	}

	document->NotifyChanges();
	return true;
}

AssetImportResult ImportResultForReaderStatus(PCDM_ReaderStatus status) {
    switch (status) {
        case PCDM_RS_OK:
            return AssetImportResult::Success;
        case PCDM_RS_NoDocument:
        case PCDM_RS_FormatFailure:
        case PCDM_RS_UnrecognizedFileFormat:
        case PCDM_RS_NoModel:
            return AssetImportResult::InvalidData;
        case PCDM_RS_OpenError:
        case PCDM_RS_WrongStreamMode:
        case PCDM_RS_PermissionDenied:
        case PCDM_RS_UnknownDocument:
            return AssetImportResult::TemporaryFileFailure;
        case PCDM_RS_AlreadyRetrievedAndModified:
        case PCDM_RS_AlreadyRetrieved:
            return AssetImportResult::Busy;
        case PCDM_RS_UnknownFileDriver:
        case PCDM_RS_NoVersion:
        case PCDM_RS_TypeFailure:
        case PCDM_RS_TypeNotFoundInSchema:
        case PCDM_RS_NoDriver:
        case PCDM_RS_NoSchema:
            return AssetImportResult::UnsupportedVersion;
        case PCDM_RS_MakeFailure:
        case PCDM_RS_ReaderException:
        case PCDM_RS_ExtensionFailure:
        case PCDM_RS_DriverFailure:
        case PCDM_RS_WrongResource:
        case PCDM_RS_UserBreak:
            return AssetImportResult::InternalFailure;
    }
}

void CloseDocumentNoThrow(const Handle(TDocStd_Application)& app,
                          Handle(TDocStd_Document)& document) noexcept {
    if (app.IsNull() || document.IsNull()) {
        return;
    }

    try {
        if (document->HasOpenCommand()) {
            document->AbortCommand();
        }
        app->Close(document);
    } catch (...) {
        // A failed cleanup must not replace the original load result or crash rollback.
    }
    document.Nullify();
}

bool IsValidTopologyOrientation(
    const TopAbs_Orientation orientation) noexcept {
    switch (orientation) {
        case TopAbs_FORWARD:
        case TopAbs_REVERSED:
        case TopAbs_INTERNAL:
        case TopAbs_EXTERNAL:
            return true;
    }
    return false;
}

bool IsAllowedTopologyChild(
    const TopAbs_ShapeEnum parent,
    const TopAbs_ShapeEnum child) noexcept {
    switch (parent) {
        case TopAbs_COMPOUND:
            return child >= TopAbs_COMPOUND && child < TopAbs_SHAPE;
        case TopAbs_COMPSOLID:
            return child == TopAbs_SOLID;
        case TopAbs_SOLID:
            return child == TopAbs_SHELL;
        case TopAbs_SHELL:
            return child == TopAbs_FACE;
        case TopAbs_FACE:
            return child == TopAbs_WIRE;
        case TopAbs_WIRE:
            return child == TopAbs_EDGE;
        case TopAbs_EDGE:
            return child == TopAbs_VERTEX;
        case TopAbs_VERTEX:
        case TopAbs_SHAPE:
            return false;
    }
    return false;
}

bool ValidateBoundedTopologicalShape(
    const TopoDS_Shape& shape,
    Standard_Size& aggregateSubshapes,
    const Standard_Size maximumAggregateSubshapes) {
    if (shape.IsNull() || maximumAggregateSubshapes == 0) {
        return false;
    }
    try {
        OCC_CATCH_SIGNALS
#ifdef DEBUG
        gBoundedProjectTopologyValidationCount.fetch_add(
            1, std::memory_order_relaxed);
#endif
        struct TopologyFrame {
            TopoDS_Shape shape;
            Standard_Size depth = 0;
            bool leaving = false;
        };
        const Standard_Size maximumSubshapesPerDefinition =
            std::min(kMaximumSubshapesPerDefinition,
                     maximumAggregateSubshapes);
        const std::size_t maximumStackFrames =
            static_cast<std::size_t>(maximumSubshapesPerDefinition) * 2U;
        std::vector<TopologyFrame> stack = {{shape, 0, false}};
        TopTools_MapOfShape visited;
        TopTools_MapOfShape activePath;
        Standard_Size count = 0;
        while (!stack.empty()) {
            const TopologyFrame frame = stack.back();
            stack.pop_back();
            if (frame.shape.IsNull()) {
                return false;
            }
            if (frame.leaving) {
                activePath.Remove(frame.shape);
                visited.Add(frame.shape);
                continue;
            }
            if (visited.Contains(frame.shape)) {
                continue;
            }
            if (activePath.Contains(frame.shape)
                || frame.shape.ShapeType() == TopAbs_SHAPE
                || !IsValidTopologyOrientation(frame.shape.Orientation())
                || frame.depth > kMaximumAssemblyTraversalDepth
                || count >= maximumSubshapesPerDefinition
                || aggregateSubshapes >= maximumAggregateSubshapes) {
                return false;
            }
            activePath.Add(frame.shape);
            ++count;
            if (count > maximumAggregateSubshapes - aggregateSubshapes
                || stack.size() >= maximumStackFrames) {
                return false;
            }
            stack.push_back({frame.shape, frame.depth, true});
            const TopAbs_ShapeEnum parentType = frame.shape.ShapeType();
            for (TopoDS_Iterator child(
                     frame.shape, Standard_True, Standard_True);
                 child.More(); child.Next()) {
                const TopoDS_Shape& childShape = child.Value();
                if (childShape.IsNull()
                    || !IsAllowedTopologyChild(
                        parentType, childShape.ShapeType())
                    || !IsValidTopologyOrientation(
                        childShape.Orientation())
                    || stack.size() >= maximumStackFrames) {
                    return false;
                }
                stack.push_back({childShape, frame.depth + 1, false});
            }
        }
        aggregateSubshapes += count;
        // Do not call BRepCheck_Analyzer here. It evaluates attacker-controlled
        // curves and surfaces without a progress/cancellation hook. Safe binary
        // retrieval plus this canonical, cycle-aware, allocation-bounded walk
        // is the fail-closed project-load gate; ordinary modeling operations
        // retain geometric validity checks through IsTopologicallyValid.
        return true;
    } catch (...) {
        return false;
    }
}

struct ShapeTraversalFrame {
    TDF_Label label;
    Standard_Size depth = 0;
    bool leaving = false;
};

bool HasValidOptionalLengthUnit(
    const Handle(TDocStd_Document)& document) {
    if (document.IsNull()) {
        return false;
    }

    Standard_Real metersPerUnit = 0.0;
    const bool hasLengthUnit =
        XCAFDoc_DocumentTool::GetLengthUnit(document, metersPerUnit);
    return !hasLengthUnit
        || (std::isfinite(metersPerUnit) && metersPerUnit > 0.0);
}

bool ValidateShapeTree(
    const Handle(TDocStd_Document)& document,
    TDF_LabelMap& activeDefinitionLabels,
    const Standard_Size maximumAggregateSubshapes) {
    activeDefinitionLabels.Clear();
    if (document.IsNull() || maximumAggregateSubshapes == 0) {
        return false;
    }

    const Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(document->Main());
    if (shapeTool.IsNull()) {
        return false;
    }

    std::vector<ShapeTraversalFrame> stack;
    stack.reserve(64);
    for (TDF_ChildIterator root(shapeTool->Label(), Standard_False);
         root.More(); root.Next()) {
        const TDF_Label& label = root.Value();
        if (!label.IsNull() && XCAFDoc_ShapeTool::IsShape(label)) {
            if (stack.size()
                >= static_cast<std::size_t>(kMaximumShapeDefinitions)) {
                return false;
            }
            stack.push_back({label, 0, false});
        }
    }

    TDF_LabelMap activePath;
    TDF_LabelMap visitedLabels;
    Standard_Size traversalNodes = 0;
    Standard_Size definitionCount = 0;
    Standard_Size aggregateSubshapes = 0;
    while (!stack.empty()) {
        const ShapeTraversalFrame frame = stack.back();
        stack.pop_back();
        if (frame.leaving) {
            activePath.Remove(frame.label);
            visitedLabels.Add(frame.label);
            continue;
        }
        if (visitedLabels.Contains(frame.label)) {
            continue;
        }
        if (frame.label.IsNull()
            || frame.depth > kMaximumAssemblyTraversalDepth
            || activePath.Contains(frame.label)
            || ++traversalNodes > kMaximumAssemblyTraversalNodes) {
            return false;
        }

        activePath.Add(frame.label);
        stack.push_back({frame.label, frame.depth, true});

        if (XCAFDoc_ShapeTool::IsReference(frame.label)
            || XCAFDoc_ShapeTool::IsComponent(frame.label)) {
            TDF_Label referredLabel;
            if (!XCAFDoc_ShapeTool::GetReferredShape(
                    frame.label, referredLabel)
                || referredLabel.IsNull()
                || referredLabel.Data() != frame.label.Data()
                || !XCAFDoc_ShapeTool::IsShape(referredLabel)) {
                return false;
            }
            stack.push_back({referredLabel, frame.depth + 1, false});
            continue;
        }

        if (!XCAFDoc_ShapeTool::IsShape(frame.label)
            || ++definitionCount > kMaximumShapeDefinitions) {
            return false;
        }
        activeDefinitionLabels.Add(frame.label);
        const TopoDS_Shape definitionShape =
            XCAFDoc_ShapeTool::GetShape(frame.label);
        if (!ValidateBoundedTopologicalShape(
                definitionShape,
                aggregateSubshapes,
                maximumAggregateSubshapes)) {
            return false;
        }

        if (!XCAFDoc_ShapeTool::IsAssembly(frame.label)) {
            continue;
        }

        std::vector<TDF_Label> components;
        for (TDF_ChildIterator child(frame.label, Standard_False);
             child.More(); child.Next()) {
            const TDF_Label& component = child.Value();
            if (XCAFDoc_ShapeTool::IsComponent(component)) {
                const Standard_Size componentCount =
                    static_cast<Standard_Size>(components.size()) + 1;
                if (traversalNodes > kMaximumAssemblyTraversalNodes
                    || componentCount
                        > kMaximumAssemblyTraversalNodes - traversalNodes
                    || stack.size()
                        > static_cast<std::size_t>(
                            kMaximumAssemblyTraversalNodes)
                    || componentCount
                        > kMaximumAssemblyTraversalNodes
                            - static_cast<Standard_Size>(stack.size())) {
                    return false;
                }
                components.push_back(component);
            }
        }
        for (auto component = components.rbegin();
             component != components.rend(); ++component) {
            stack.push_back({*component, frame.depth + 1, false});
        }
    }

    // An empty document is valid: a user can intentionally save a blank scene.
    return true;
}

const Standard_GUID& LocalPBRMaterialValidationAttributeID() {
    static const Standard_GUID id(
        "248A5203-4A22-4F2B-85C4-BE0BA89A5E4D");
    return id;
}

const Standard_GUID& AutoPromotedEmissiveFactorValidationAttributeID() {
    static const Standard_GUID id(
        "1DA4580F-1B19-46DA-ABD4-BBCE9FBADE44");
    return id;
}

bool IsFiniteUnit(const Standard_Real value) {
    return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}

bool ValidateColor(const Quantity_Color& color) {
    return IsFiniteUnit(color.Red())
        && IsFiniteUnit(color.Green())
        && IsFiniteUnit(color.Blue());
}

using TextureValidationState = Core3DEmbeddedTextureBudgetState;

bool ValidateEmbeddedTexture(const Handle(Image_Texture)& texture,
                             TextureValidationState& state) {
    return Core3DAccumulateEmbeddedTextureBudget(
        texture, state,
        kMaximumAggregateTextureBytes,
        kMaximumDecodedTextureBytes);
}

bool ValidatePBRMaterial(const XCAFDoc_VisMaterialPBR& material,
                         TextureValidationState& textures) {
    const Quantity_Color& baseColor = material.BaseColor.GetRGB();
    return material.IsDefined
        && ValidateColor(baseColor)
        && IsFiniteUnit(material.BaseColor.Alpha())
        && IsFiniteUnit(material.Metallic)
        && IsFiniteUnit(material.Roughness)
        && std::isfinite(material.RefractionIndex)
        && material.RefractionIndex >= 1.0f
        && material.RefractionIndex <= 3.0f
        && std::isfinite(material.EmissiveFactor.x())
        && std::isfinite(material.EmissiveFactor.y())
        && std::isfinite(material.EmissiveFactor.z())
        && material.EmissiveFactor.x() >= 0.0f
        && material.EmissiveFactor.y() >= 0.0f
        && material.EmissiveFactor.z() >= 0.0f
        && material.EmissiveFactor.x() <= kMaximumEmissionFactor
        && material.EmissiveFactor.y() <= kMaximumEmissionFactor
        && material.EmissiveFactor.z() <= kMaximumEmissionFactor
        && ValidateEmbeddedTexture(material.BaseColorTexture, textures)
        && ValidateEmbeddedTexture(
            material.MetallicRoughnessTexture, textures)
        && ValidateEmbeddedTexture(material.EmissiveTexture, textures)
        && ValidateEmbeddedTexture(material.OcclusionTexture, textures)
        && ValidateEmbeddedTexture(material.NormalTexture, textures);
}

bool ValidateCommonMaterial(const XCAFDoc_VisMaterialCommon& material,
                            TextureValidationState& textures) {
    return material.IsDefined
        && ValidateColor(material.AmbientColor)
        && ValidateColor(material.DiffuseColor)
        && ValidateColor(material.SpecularColor)
        && ValidateColor(material.EmissiveColor)
        && IsFiniteUnit(material.Shininess)
        && IsFiniteUnit(material.Transparency)
        && ValidateEmbeddedTexture(material.DiffuseTexture, textures);
}

bool ValidateVisualMaterials(
    const Handle(TDocStd_Document)& document,
    const TDF_LabelMap& activeDefinitionLabels) {
    if (document.IsNull()) {
        return false;
    }

    Handle(XCAFDoc_VisMaterialTool) materialTool;
    if (XCAFDoc_DocumentTool::CheckVisMaterialTool(document->Main())) {
        materialTool = XCAFDoc_DocumentTool::VisMaterialTool(
            document->Main());
        if (materialTool.IsNull()) {
            return false;
        }
    }

    TextureValidationState textureState;
    TDF_LabelMap tableMaterialLabels;
    if (!materialTool.IsNull()) {
        TDF_LabelSequence materialLabels;
        materialTool->GetMaterials(materialLabels);
        if (materialLabels.Length()
            > kMaximumVisualMaterialDefinitions) {
            return false;
        }
        for (TDF_LabelSequence::Iterator iterator(materialLabels);
             iterator.More(); iterator.Next()) {
            tableMaterialLabels.Add(iterator.Value());
            const Handle(XCAFDoc_VisMaterial) material =
                XCAFDoc_VisMaterialTool::GetMaterial(iterator.Value());
            if (material.IsNull()
                || (!material->HasPbrMaterial()
                    && !material->HasCommonMaterial())) {
                return false;
            }
            if (material->HasPbrMaterial()
                && !ValidatePBRMaterial(
                    material->PbrMaterial(), textureState)) {
                return false;
            }
            if (material->HasCommonMaterial()
                && !ValidateCommonMaterial(
                    material->CommonMaterial(), textureState)) {
                return false;
            }
            switch (material->AlphaMode()) {
                case Graphic3d_AlphaMode_BlendAuto:
                case Graphic3d_AlphaMode_Opaque:
                case Graphic3d_AlphaMode_Mask:
                case Graphic3d_AlphaMode_Blend:
                    break;
                case Graphic3d_AlphaMode_MaskBlend:
                    // The renderer-neutral contract currently has distinct
                    // mask and blend modes, not a combined two-stage mode.
                    // Reject instead of silently discarding the cutoff.
                    return false;
                default:
                    return false;
            }
            if (!IsFiniteUnit(material->AlphaCutOff())) {
                return false;
            }
            switch (material->FaceCulling()) {
                case Graphic3d_TypeOfBackfacingModel_Auto:
                case Graphic3d_TypeOfBackfacingModel_DoubleSided:
                case Graphic3d_TypeOfBackfacingModel_BackCulled:
                case Graphic3d_TypeOfBackfacingModel_FrontCulled:
                    break;
                default:
                    return false;
            }
        }
    }

    TDF_LabelMap validatedLocalMaterialLabels;
    std::unordered_set<std::string> validatedCanonicalTextureIDs;
    const auto hasUnregisteredDirectMaterial =
        [&](const TDF_Label& label) {
            Handle(XCAFDoc_VisMaterial) directMaterial;
            return !label.IsNull()
                && label.FindAttribute(
                    XCAFDoc_VisMaterial::GetID(), directMaterial)
                && (directMaterial.IsNull()
                    || !tableMaterialLabels.Contains(label));
        };
    const Handle(TDF_Data)& documentData = document->GetData();
    if (documentData.IsNull()
        || hasUnregisteredDirectMaterial(documentData->Root())
        || hasUnregisteredDirectMaterial(document->Main())) {
        return false;
    }
    Standard_Size labelCount = 0;
    for (TDF_ChildIterator iterator(
             documentData->Root(), Standard_True);
         iterator.More(); iterator.Next()) {
        if (++labelCount > kMaximumDocumentLabels) {
            return false;
        }
        const TDF_Label& label = iterator.Value();
        Handle(XCAFDoc_VisMaterial) directMaterial;
        if (label.FindAttribute(
                XCAFDoc_VisMaterial::GetID(), directMaterial)
            && (directMaterial.IsNull() || materialTool.IsNull()
                || !tableMaterialLabels.Contains(label))) {
            // Raw BinXCAF preflight charges every material attribute. Keeping
            // orphans outside the material-tool table would let a writer scan
            // omit serialized texture occurrences and exceed the reopen cap.
            return false;
        }
        TDF_Label assignedMaterialLabel;
        if (XCAFDoc_VisMaterialTool::GetShapeMaterial(
                label, assignedMaterialLabel)
            && (materialTool.IsNull()
                || assignedMaterialLabel.IsNull()
                || !materialTool->IsMaterial(assignedMaterialLabel))) {
            return false;
        }

        Handle(TDataStd_Integer) marker;
        Handle(TDataStd_Integer) autoPromotedEmissiveFactor;
        const bool hasAutoPromotedEmissiveFactor = label.FindAttribute(
            AutoPromotedEmissiveFactorValidationAttributeID(),
            autoPromotedEmissiveFactor);
        if (!label.FindAttribute(
                LocalPBRMaterialValidationAttributeID(), marker)) {
            if (hasAutoPromotedEmissiveFactor) {
                return false;
            }
            continue;
        }
        if (marker.IsNull() || marker->Get() != 1
            || !activeDefinitionLabels.Contains(label)
            || materialTool.IsNull()
            || assignedMaterialLabel.IsNull()) {
            return false;
        }
        if (hasAutoPromotedEmissiveFactor) {
            const Handle(XCAFDoc_VisMaterial) material =
                XCAFDoc_VisMaterialTool::GetMaterial(
                    assignedMaterialLabel);
            if (autoPromotedEmissiveFactor.IsNull()
                || autoPromotedEmissiveFactor->Get() != 1
                || material.IsNull() || !material->HasPbrMaterial()) {
                return false;
            }
            const XCAFDoc_VisMaterialPBR& pbr =
                material->PbrMaterial();
            if (pbr.EmissiveTexture.IsNull()
                || pbr.EmissiveFactor.x() != 1.0f
                || pbr.EmissiveFactor.y() != 1.0f
                || pbr.EmissiveFactor.z() != 1.0f) {
                return false;
            }
        }
        if (!validatedLocalMaterialLabels.Contains(
                assignedMaterialLabel)) {
            const Handle(XCAFDoc_VisMaterial) material =
                XCAFDoc_VisMaterialTool::GetMaterial(
                    assignedMaterialLabel);
            if (material.IsNull() || !material->HasPbrMaterial()) {
                return false;
            }
            const XCAFDoc_VisMaterialPBR& pbr = material->PbrMaterial();
            if (!pbr.MetallicRoughnessTexture.IsNull()
                || !pbr.OcclusionTexture.IsNull()
                || !pbr.NormalTexture.IsNull()) {
                return false;
            }
            const Handle(Image_Texture)& pbrBase = pbr.BaseColorTexture;
            const Handle(Image_Texture) commonBase =
                material->HasCommonMaterial()
                    ? material->CommonMaterial().DiffuseTexture
                    : Handle(Image_Texture)();
            if (pbrBase.IsNull() != commonBase.IsNull()) {
                return false;
            }
            if (!pbrBase.IsNull()) {
                if (!Core3DTexturesMatch(pbrBase, commonBase)) {
                    // Exact same-ID byte equality has already been established
                    // by TextureValidationState while walking the bounded
                    // material table above.
                    return false;
                }
            }
            for (const Handle(Image_Texture)& texture : {
                     pbrBase, pbr.EmissiveTexture}) {
                if (texture.IsNull()) {
                    continue;
                }
                const std::string textureID(
                    texture->TextureId().ToCString());
                if (textureID.empty()
                    || (validatedCanonicalTextureIDs.insert(textureID).second
                        && !Core3DValidateAuthoredTexture(texture))) {
                    return false;
                }
            }
            validatedLocalMaterialLabels.Add(assignedMaterialLabel);
        }
        for (const Standard_Integer tag : {11, 12}) {
            const TDF_Label legacy = label.FindChild(tag, Standard_False);
            Handle(TDataStd_Integer) legacyValue;
            if (!legacy.IsNull()
                && legacy.FindAttribute(
                    TDataStd_Integer::GetID(), legacyValue)) {
                return false;
            }
        }
    }
    return true;
}

bool IsProjectFileWithinSizeLimit(const std::string& path) {
    struct stat fileInfo = {};
    return !path.empty()
        && ::stat(path.c_str(), &fileInfo) == 0
        && fileInfo.st_size >= 0
        && static_cast<std::uint64_t>(fileInfo.st_size)
            <= kMaximumProjectDocumentBytes;
}

} // namespace

void Core3DViewer::release() noexcept {
    // Interactors retain the view, context, document, and manipulator graphics.
    // GLViewController calls this while the viewport EAGL context is current,
    // so release them before the base handles and before that context is
    // restored. Repeated calls are intentionally harmless.
    _interactiveCallback = {};
    _booleanPreviewStateChangedCallback = {};
    _shapeInteractor.reset();
    _objectInteractor.reset();
    OcctViewer::release();
}

NSString* Core3DViewer::addTestPrimitives() {

    gp_Pnt lowerLeftCornerOfBox(-50.0,-50.0,0.0);
    BRepPrimAPI_MakeBox boxMaker(lowerLeftCornerOfBox,100,100,50);
    TopoDS_Shape box = boxMaker.Shape();

    //Create a cylinder with a radius 25.0 and height 50.0, centered at the origin
    BRepPrimAPI_MakeCylinder cylinderMaker(25.0,50.0);
    TopoDS_Shape cylinder = cylinderMaker.Shape();

    //Cut the cylinder out from the box
    BRepAlgoAPI_Cut cutMaker(box,cylinder);
    //TopoDS_Shape boxWithHole = cutMaker.Shape();

    // fillets
    BRepFilletAPI_MakeFillet  MF(cutMaker);
    TopExp_Explorer  ex(cutMaker,TopAbs_EDGE);
    while (ex.More()) {
        MF.Add(5,TopoDS::Edge(ex.Current()));
        ex.Next();
    }

    gp_Ax2 cc1;
    cc1.SetLocation(gp_Pnt(50.0, 50.0, 10.0));

    BRepPrimAPI_MakeCylinder cylinderMaker1(cc1, 25.0,50.0);
    TopoDS_Shape cylinder1 = cylinderMaker1.Shape();

    BRepAlgoAPI_Fuse fuseMaker(cylinder1, MF.Shape());

    Handle(AIS_InteractiveObject) aShapePrs = new AIS_Shape (fuseMaker.Shape());
    myContext->Display (aShapePrs, AIS_Shaded, 0, false);

    return @"";

    //    NSString *cacheDirectory = NSSearchPathForDirectoriesInDomains(NSCachesDirectory, NSUserDomainMask, YES).firstObject;
    //
    //    __auto_type stlFilename = [cacheDirectory stringByAppendingPathComponent:@"test.stl"];
    //
    //    StlAPI_Writer aStlWriter;
    //    aStlWriter.Write(fuseMaker.Shape(), stlFilename.UTF8String);
    //
    //    return stlFilename;
}

bool Core3DViewer::InitViewer (UIView* theWin) {
    bool result = OcctViewer::InitViewer(theWin);
    if(result) {
        if(_objectInteractor == nullptr) {
            const float scale = [[UIScreen mainScreen] scale];
            const float ppm = scale * (([[UIDevice currentDevice] userInterfaceIdiom] == UIUserInterfaceIdiomPad) ? 132 : 163) / 25.4;
            const float device_independent_side = 15 * ppm;
            _objectInteractor = std::make_shared<ObjectInteractor>(myContext, myView, myDoc, device_independent_side);
            _objectInteractor->setBooleanPreviewStateChangedCallback(
                _booleanPreviewStateChangedCallback);
        }
        if(_shapeInteractor == nullptr) {
            _shapeInteractor = std::make_shared<ShapeInteractor>(myContext, myView, myDoc);
        }
    }
    return result;
}

void Core3DViewer::recreateInteractors(PrimitiveManipulatorType theManipulatorType,
                                       ShapeSelectionMode theSelectionMode) {
    const float scale = [[UIScreen mainScreen] scale];
    const float pointsPerMillimeter = scale
        * (([[UIDevice currentDevice] userInterfaceIdiom] == UIUserInterfaceIdiomPad) ? 132 : 163)
        / 25.4;
    const float manipulatorSide = 15 * pointsPerMillimeter;

    _objectInteractor = std::make_shared<ObjectInteractor>(myContext, myView, myDoc, manipulatorSide);
    _objectInteractor->setBooleanPreviewStateChangedCallback(
        _booleanPreviewStateChangedCallback);
    _shapeInteractor = std::make_shared<ShapeInteractor>(myContext, myView, myDoc);

    if (theSelectionMode != ShapeSelectionMode::WholeShape) {
        _shapeInteractor->setSelectionMode(theSelectionMode);
    }
    if (theManipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeNone) {
        _objectInteractor->setManipulatorType(theManipulatorType);
        if (theManipulatorType
            == PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract) {
            (void)_objectInteractor->beginBoolean(
                BooleanAction::BooleanSubtract);
        } else if (theManipulatorType
                   == PrimitiveManipulatorType::PrimitiveGizmoTypeUnion) {
            (void)_objectInteractor->beginBoolean(
                BooleanAction::BooleanUnion);
        }
    }
}

void Core3DViewer::setBooleanPreviewStateChangedCallback(
    std::function<void()> theCallback)
{
    _booleanPreviewStateChangedCallback = std::move(theCallback);
    if (_objectInteractor != nullptr) {
        _objectInteractor->setBooleanPreviewStateChangedCallback(
            _booleanPreviewStateChangedCallback);
    }
}

std::shared_ptr<ObjectInteractor> Core3DViewer::getObjectInteractor() {
    return _objectInteractor;
}

std::shared_ptr<ShapeInteractor> Core3DViewer::getShapeInteractor() {
    return _shapeInteractor;
}

Handle(OcctDocument) Core3DViewer::getDocument() {
    return myDoc;
}

static Quantity_NameOfColor colorNameFromString(NSString* colorStr) {
    static NSDictionary<NSString*, NSNumber*>* colorMap = @{
        @"white":   @(Quantity_NOC_WHITE),
        @"black":   @(Quantity_NOC_BLACK),
        @"red":     @(Quantity_NOC_RED),
        @"green":   @(Quantity_NOC_GREEN),
        @"blue":    @(Quantity_NOC_BLUE1),
        @"yellow":  @(Quantity_NOC_YELLOW),
        @"orange":  @(Quantity_NOC_ORANGE),
        @"brown":   @(Quantity_NOC_SADDLEBROWN),
        @"gray":    @(Quantity_NOC_GRAY80),
        @"grey":    @(Quantity_NOC_GRAY80),
        @"pink":    @(Quantity_NOC_PINK),
        @"purple":  @(Quantity_NOC_PURPLE),
        @"cyan":    @(Quantity_NOC_CYAN1),
    };
    NSNumber* val = colorMap[colorStr.lowercaseString];
    return val ? (Quantity_NameOfColor)val.intValue : Quantity_NOC_GRAY50;
}

void Core3DViewer::addPrimitivesFromJSON(NSString* json) {
    if (![json isKindOfClass:[NSString class]]
        || myDoc.IsNull()
        || myContext.IsNull()
        || _shapeInteractor == nullptr) {
        return;
    }

    NSData* data = [json dataUsingEncoding:NSUTF8StringEncoding];
    if (!data) {
        return;
    }

    NSError* error = nil;
    id rootValue = [NSJSONSerialization JSONObjectWithData:data options:0 error:&error];
    if (error || ![rootValue isKindOfClass:[NSDictionary class]]) {
        return;
    }

    NSDictionary* root = static_cast<NSDictionary*>(rootValue);
    id primitivesValue = root[@"primitives"];
    if (![primitivesValue isKindOfClass:[NSArray class]]) {
        return;
    }

    NSArray* primitives = static_cast<NSArray*>(primitivesValue);
    if (primitives.count == 0 || primitives.count > kMaximumPrimitiveCount) {
        return;
    }

    std::vector<PreparedPrimitive> preparedPrimitives;
    preparedPrimitives.reserve(primitives.count);

    try {
        OCC_CATCH_SIGNALS
        for (id primitiveValue in primitives) {
            if (![primitiveValue isKindOfClass:[NSDictionary class]]) {
                return;
            }

            NSDictionary* primitive = static_cast<NSDictionary*>(primitiveValue);
            id typeValue = primitive[@"type"];
            id positionValue = primitive[@"position"];
            id scaleValue = primitive[@"scale"];
            id rotationValue = primitive[@"rotation"];
            id colorValue = primitive[@"color"];

            if (![typeValue isKindOfClass:[NSString class]]
                || (colorValue != nil && ![colorValue isKindOfClass:[NSString class]])) {
                return;
            }

            NSString* type = static_cast<NSString*>(typeValue);
            NSString* color = colorValue == nil
                ? nil
                : static_cast<NSString*>(colorValue);

            std::array<double, 3> position;
            std::array<double, 3> scale;
            std::array<double, 3> rotation = {0.0, 0.0, 0.0};
            if (!ReadFiniteVector3(positionValue, position)
                || !ReadFiniteVector3(scaleValue, scale)
                || (rotationValue != nil && !ReadFiniteVector3(rotationValue, rotation))
                || !IsBoundedMagnitude(position, kMaximumPositionMagnitude)
                || !IsValidScale(scale)
                || !IsBoundedMagnitude(rotation, kMaximumRotationMagnitude)) {
                return;
            }

            const double px = position[0];
            const double py = position[1];
            const double pz = position[2];
            const double sx = scale[0];
            const double sy = scale[1];
            const double sz = scale[2];
            const double rx = rotation[0];
            const double ry = rotation[1];
            const double rz = rotation[2];

            // No coordinate conversion — JSON uses Z-up matching OCCT natively.
            // Create shapes with correct dimensions directly (no GTransform).
            TopoDS_Shape shape;
            if ([type isEqualToString:@"cube"]) {
                const double width = 50.0 * sx;
                const double depth = 50.0 * sy;
                const double height = 50.0 * sz;
                if (!IsValidDimension(width)
                    || !IsValidDimension(depth)
                    || !IsValidDimension(height)) {
                    return;
                }
                gp_Pnt corner(-25.0 * sx, -25.0 * sy, 0.0);
                BRepPrimAPI_MakeBox maker(corner, width, depth, height);
                shape = maker.Shape();
            } else if ([type isEqualToString:@"sphere"]) {
                const double radiusX = 25.0 * sx;
                const double radiusY = 25.0 * sy;
                const double radiusZ = 25.0 * sz;
                if (!IsValidDimension(radiusX)
                    || !IsValidDimension(radiusY)
                    || !IsValidDimension(radiusZ)) {
                    return;
                }

                // Average scale for radius, then apply non-uniform via GTransform only if needed.
                const double averageScale = (sx + sy + sz) / 3.0;
                if (std::abs(sx - sy) < 0.01 && std::abs(sy - sz) < 0.01) {
                    BRepPrimAPI_MakeSphere maker(25.0 * averageScale);
                    shape = maker.Shape();
                } else {
                    BRepPrimAPI_MakeSphere maker(25.0);
                    gp_GTrsf scaleTransform;
                    scaleTransform.SetValue(1, 1, sx);
                    scaleTransform.SetValue(2, 2, sy);
                    scaleTransform.SetValue(3, 3, sz);
                    BRepBuilderAPI_GTransform scaler(maker.Shape(), scaleTransform, true);
                    shape = scaler.Shape();
                }
            } else if ([type isEqualToString:@"cylinder"]
                       || [type isEqualToString:@"cone"]
                       || [type isEqualToString:@"torus"]) {
                const double radialScale = (sx + sy) / 2.0;
                const double radius = 25.0 * radialScale;
                const double height = 50.0 * sz;
                if (!IsValidDimension(radius) || !IsValidDimension(height)) {
                    return;
                }

                if ([type isEqualToString:@"cylinder"]) {
                    BRepPrimAPI_MakeCylinder maker(radius, height);
                    shape = maker.Shape();
                } else if ([type isEqualToString:@"cone"]) {
                    gp_Ax2 axis;
                    axis.SetLocation(gp_Pnt(0.0, 0.0, 0.0));
                    BRepPrimAPI_MakeCone maker(axis, radius, 0.0, height);
                    shape = maker.Shape();
                } else {
                    const double minorRadius = 10.0 * sz;
                    if (!IsValidDimension(minorRadius)) {
                        return;
                    }
                    gp_Ax2 axis;
                    axis.SetLocation(gp_Pnt(0.0, 0.0, 0.0));
                    BRepPrimAPI_MakeTorus maker(axis, radius, minorRadius);
                    shape = maker.Shape();
                }
            } else {
                return;
            }

            if (!IsTopologicallyValid(shape)) {
                return;
            }

            // Keep rotation/translation as the editable presentation transform, the
            // same representation produced by the on-screen gizmo. Geometry remains
            // stable in OCAF and the transform is persisted on its label.
            gp_Trsf rotationTransform;
            if (rx != 0.0 || ry != 0.0 || rz != 0.0) {
                gp_Trsf xRotation;
                gp_Trsf yRotation;
                gp_Trsf zRotation;
                if (rx != 0.0) {
                    xRotation.SetRotation(
                        gp_Ax1(gp::Origin(), gp::DX()),
                        rx * M_PI / 180.0);
                }
                if (ry != 0.0) {
                    yRotation.SetRotation(
                        gp_Ax1(gp::Origin(), gp::DY()),
                        ry * M_PI / 180.0);
                }
                if (rz != 0.0) {
                    zRotation.SetRotation(
                        gp_Ax1(gp::Origin(), gp::DZ()),
                        rz * M_PI / 180.0);
                }
                rotationTransform = zRotation * yRotation * xRotation;
            }

            gp_Trsf translationTransform;
            translationTransform.SetTranslation(gp_Vec(px, py, pz));
            const gp_Trsf objectTransform = translationTransform * rotationTransform;

            Handle(AIS_Shape) presentation = new AIS_Shape(shape);
            presentation->SetLocalTransformation(objectTransform);
            myContext->ApplyDefaultMaterial(presentation);
            Quantity_NameOfColor shapeColor = Quantity_NOC_GRAY80;
            if (color != nil) {
                shapeColor = colorNameFromString(color);
                presentation->SetColor(shapeColor);
            }

            if (presentation.IsNull()
                || presentation->Shape().IsNull()
                || !IsTopologicallyValid(presentation->Shape())) {
                return;
            }

            preparedPrimitives.push_back({
                presentation,
                Graphic3d_NameOfMaterial_ShinyPlastified,
                shapeColor,
            });
        }
    } catch (const Standard_Failure&) {
        return;
    } catch (const std::exception&) {
        return;
    } catch (...) {
        return;
    }

	// Clear outgoing selection/tool visuals before opening the batch command;
	// deselection can itself finalize an explicitly pending chamfer.
	deselectAll();

	// Keep one command open through presentation staging so either side can be
	// rolled back without consuming undo/redo history.
	const Standard_Integer selectionMode =
		static_cast<Standard_Integer>(_shapeInteractor->getSelectionMode());
	if (!PublishPreparedPrimitives(
		myDoc,
		myContext,
		preparedPrimitives,
		selectionMode)) {
        return;
    }
	try {
		if (!myView.IsNull()) {
			myView->FitAll();
			myView->Redraw();
		}
	} catch (...) {
		// Framing failure does not invalidate the fully displayed, durable batch.
    }
}

void Core3DViewer::addPrimitive(PrimitiveType primitiveType) {

    TopoDS_Shape shape;
	Standard_Size displayedModelShapeCount = 0;
	const bool shouldFrameFirstPrimitive =
		TryCountDisplayedModelShapes(myContext, displayedModelShapeCount)
		&& displayedModelShapeCount == 0;

    switch (primitiveType) {
        case PrimitiveTypeCube:
        {
            gp_Pnt lowerLeftCornerOfBox(-25.0, -25.0, 0.0);
            BRepPrimAPI_MakeBox boxMaker(lowerLeftCornerOfBox,50, 50, 50);
            shape = boxMaker.Shape();
        }
            break;
        case PrimitiveTypeSphere:
        {
            gp_Ax2 anAxis;
            anAxis.SetLocation(gp_Pnt(0.0, 0.0, 0.0));
            BRepPrimAPI_MakeSphere sphereMaker(anAxis, 25.0);
            shape = sphereMaker.Shape();
        }
            break;
        case PrimitiveTypeCylinder:
        {
            BRepPrimAPI_MakeCylinder cylinderMaker(25.0, 50.0);
            shape = cylinderMaker.Shape();
        }
            break;
        case PrimitiveTypeCone:
        {
            gp_Ax2 anAxis;
            anAxis.SetLocation(gp_Pnt(0.0, 0.0, 0.0));
            BRepPrimAPI_MakeCone coneMaker(anAxis, 25.0, 0.0, 50.0);
            shape = coneMaker.Shape();
        }
            break;
        case PrimitiveTypeTorus:
        {
            gp_Ax2 anAxis;
            anAxis.SetLocation(gp_Pnt(0.0, 0.0, 0.0));
            BRepPrimAPI_MakeTorus torusMaker(anAxis, 25.0, 10.0);
            shape = torusMaker.Shape();
        }
            break;
        default:
            break;
    }

    Handle(AIS_Shape) aShapePrs = new AIS_Shape (shape);
    myContext->ApplyDefaultMaterial(aShapePrs);
    Quantity_Color qc;
    aShapePrs->Color(qc);
	if (!IsTopologicallyValid(shape)) {
		return;
	}
	deselectAll();
	const std::vector<PreparedPrimitive> primitive = {{
		aShapePrs,
		Graphic3d_NameOfMaterial_ShinyPlastified,
		qc.Name(),
	}};
	if (!PublishPreparedPrimitives(
		myDoc,
		myContext,
		primitive,
		static_cast<Standard_Integer>(_shapeInteractor->getSelectionMode()))) {
        return;
    }
	if (shouldFrameFirstPrimitive && !myView.IsNull()) {
		// Frame only the first real model object. The V3d construction grid is not
		// an AIS shape, and the manipulator does not downcast to AIS_Shape, so
		// neither affects the pre-insert count. Later inserts must preserve the
		// camera the user established while modeling.
		Bnd_Box aFrameBox;
		BRepBndLib::Add(shape, aFrameBox);
		if (!aFrameBox.IsVoid()) {
			myView->FitAll(aFrameBox, 0.2, Standard_False);
			myView->ZFitAll();
		}
	}
	_objectInteractor->attachManipulatorToSelection();
    getObjectInteractor()->SelectAndAttachManipulator(aShapePrs);
}

bool Core3DViewer::traverseLabel (const Handle(TDocStd_Document)& theDoc,
                                  const TDF_Label& theLabel,
                                  const TCollection_AsciiString& theNamePrefix,
                                  const TopLoc_Location& theLoc,
                                  MapOfPrsForShapes& theMapOfShapes)
{
    if (theDoc.IsNull() || theLabel.IsNull()) {
        return true;
    }
    (void)theNamePrefix;
    (void)theLoc;
    (void)theMapOfShapes;
    XCAFPrs_Style aDefStyle;
    aDefStyle.SetColorSurf(Quantity_NOC_GRAY80);
    aDefStyle.SetColorCurv(Quantity_NOC_GRAY80);
    return !displayWithChildren(theDoc, theLabel, aDefStyle);
}

bool Core3DViewer::traverseDocument (const Handle(TDocStd_Document)& theDoc)
{
    TDF_LabelSequence aLabels;
    XCAFDoc_DocumentTool::ShapeTool (theDoc->Main())->GetFreeShapes (aLabels);
    if (aLabels.IsEmpty()) {
        return false;
    }
    XCAFPrs_Style aDefStyle;
    aDefStyle.SetColorSurf(Quantity_NOC_GRAY80);
    aDefStyle.SetColorCurv(Quantity_NOC_GRAY80);
    return !displayWithChildren(theDoc, aLabels, aDefStyle);
}

#ifdef DEBUG
void Core3DViewer::DebugResetProjectTopologyValidationCounters() const
{
    gBoundedProjectTopologyValidationCount.store(
        0, std::memory_order_relaxed);
    gGeometricBRepValidationCount.store(0, std::memory_order_relaxed);
}

Standard_Size Core3DViewer::DebugBoundedProjectTopologyValidationCount() const
{
    return gBoundedProjectTopologyValidationCount.load(
        std::memory_order_relaxed);
}

Standard_Size Core3DViewer::DebugGeometricBRepValidationCount() const
{
    return gGeometricBRepValidationCount.load(std::memory_order_relaxed);
}

void Core3DViewer::DebugSetSceneSnapshotTriangulationFailure(
    const scene::OcctSceneSnapshotBuilder::DebugTriangulationFailure
        theFailure) noexcept
{
    _sceneSnapshotBuilder.DebugSetTriangulationFailure(theFailure);
}

void Core3DViewer::DebugResetSceneSnapshotMesherInvocationCount() noexcept
{
    _sceneSnapshotBuilder.DebugResetMesherInvocationCount();
}

std::uint64_t
Core3DViewer::DebugSceneSnapshotMesherInvocationCount() const noexcept
{
    return _sceneSnapshotBuilder.DebugMesherInvocationCount();
}
#endif

AssetImportResult Core3DViewer::ImportCbf(const std::string &theFilename) {
    assert(!myContext.IsNull());

    Handle(TDocStd_Document) previous = myDoc->Document();
    if (previous.IsNull()) {
        return AssetImportResult::InternalFailure;
    }
    if (previous->HasOpenCommand()) {
        return AssetImportResult::Busy;
    }

    const AssetImportResult validationResult = ValidateCbf(theFilename);
    if (validationResult != AssetImportResult::Success) {
        return validationResult;
    }

    auto app = Handle(TDocStd_Application)::DownCast(previous->Application());
    if (app.IsNull()) {
        return AssetImportResult::InternalFailure;
    }

    const PrimitiveManipulatorType previousManipulatorType = _objectInteractor == nullptr
        ? PrimitiveManipulatorType::PrimitiveGizmoTypeNone
        : _objectInteractor->getManipulatorType();
    const ShapeSelectionMode previousSelectionMode = _shapeInteractor == nullptr
        ? ShapeSelectionMode::WholeShape
        : _shapeInteractor->getSelectionMode();
	AIS_ListOfInteractive previousPresentations;
	myContext->DisplayedObjects(AIS_KOI_Shape, -1, previousPresentations);
	auto restorePreviousState = [&]() noexcept {
		myDoc->ChangeDocument() = previous;
		try {
			clearContext();
			traverseDocument(previous);
			recreateInteractors(previousManipulatorType, previousSelectionMode);
			myContext->UpdateCurrentViewer();
			return;
		} catch (...) {
			// Reuse the exact pre-load AIS handles if rebuilding presentations
			// from OCAF fails. RemoveAll() does not destroy these retained handles.
		}

		try {
			clearContext();
			for (AIS_ListIteratorOfListOfInteractive presentation(previousPresentations);
				 presentation.More(); presentation.Next()) {
				const Handle(AIS_InteractiveObject)& object = presentation.Value();
				if (!Handle(AIS_Shape)::DownCast(object).IsNull()) {
					myContext->Display(object, Standard_False);
				}
			}
			recreateInteractors(previousManipulatorType, previousSelectionMode);
			myContext->UpdateCurrentViewer();
		} catch (...) {
			// The persistent previous document remains authoritative even if the
			// graphics driver itself can no longer restore a presentation.
		}
	};

    Handle(TDocStd_Document) candidate;
    try {
        OCC_CATCH_SIGNALS
        Core3DBeginSafeBinaryRead();
        PCDM_ReaderStatus status = app->Open(theFilename.c_str(), candidate);
        const Standard_Boolean wasRejected =
            Core3DSafeBinaryReadWasRejected();
        if (wasRejected || status != PCDM_RS_OK || candidate.IsNull()) {
            if (wasRejected) {
                CloseDocumentNoThrow(app, candidate);
                return AssetImportResult::InvalidData;
            }
            AssetImportResult result = status == PCDM_RS_OK
                ? AssetImportResult::InvalidData
                : ImportResultForReaderStatus(status);
            // Isolated validation already proved these bytes readable. A
            // conflicting result from the live application is an engine/state
            // failure, never evidence that the committed revision is corrupt.
            if (result == AssetImportResult::InvalidData) {
                result = AssetImportResult::InternalFailure;
            }
            CloseDocumentNoThrow(app, candidate);
            return result;
        }
    } catch (const Standard_Failure& failure) {
        std::cout << "Load CBF failure: " << failure.GetMessageString() << std::endl;
        CloseDocumentNoThrow(app, candidate);
        return AssetImportResult::InternalFailure;
    } catch (const std::exception& exception) {
        std::cout << "Load CBF exception: " << exception.what() << std::endl;
        CloseDocumentNoThrow(app, candidate);
        return AssetImportResult::InternalFailure;
    } catch (...) {
        CloseDocumentNoThrow(app, candidate);
        return AssetImportResult::InternalFailure;
    }

	try {
		OCC_CATCH_SIGNALS
		TDF_LabelMap candidateDefinitions;
		if (!HasValidOptionalLengthUnit(candidate)
			|| !ValidateShapeTree(
                candidate,
                candidateDefinitions,
                myMaximumProjectTopologyValidationNodes)
			|| !ValidateVisualMaterials(candidate, candidateDefinitions)) {
			CloseDocumentNoThrow(app, candidate);
			return AssetImportResult::InvalidData;
		}
		// CBF files created before persistent scene identity need a one-time
		// migration. Do this while the candidate is isolated, before it becomes
		// the editable document and before normal undo history is enabled.
		if (!myDoc->MigrateLegacyIdentifiers(candidate)) {
			CloseDocumentNoThrow(app, candidate);
			return AssetImportResult::InternalFailure;
		}
		candidate->SetUndoLimit(40);

        // Keep the previous OCAF document alive until the candidate has been
        // fully traversed and displayed. Only the presentation is temporary.
        clearContext();
        if (traverseDocument(candidate)) {
			restorePreviousState();
			CloseDocumentNoThrow(app, candidate);
			return AssetImportResult::InvalidData;
		}
        myContext->UpdateCurrentViewer();

        myDoc->ChangeDocument() = candidate;
        recreateInteractors(previousManipulatorType, previousSelectionMode);
        myContext->UpdateCurrentViewer();
    } catch (const Standard_Failure& failure) {
        std::cout << "Display CBF failure: " << failure.GetMessageString() << std::endl;
		restorePreviousState();
        CloseDocumentNoThrow(app, candidate);
        return AssetImportResult::InternalFailure;
    } catch (...) {
		restorePreviousState();
        CloseDocumentNoThrow(app, candidate);
        return AssetImportResult::InternalFailure;
    }

    CloseDocumentNoThrow(app, previous);
    return AssetImportResult::Success;
}

AssetImportResult Core3DViewer::ValidateCbf(const std::string &theFilename) const {
    if (!IsProjectFileWithinSizeLimit(theFilename)) {
        return AssetImportResult::InvalidData;
    }

    Handle(TDocStd_Application) validationApplication;
    Handle(TDocStd_Document) candidate;
    try {
        OCC_CATCH_SIGNALS
        validationApplication = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(validationApplication);

        Core3DBeginSafeBinaryRead();
        const PCDM_ReaderStatus status =
            validationApplication->Open(theFilename.c_str(), candidate);
        const Standard_Boolean wasRejected =
            Core3DSafeBinaryReadWasRejected();
        if (wasRejected || status != PCDM_RS_OK || candidate.IsNull()) {
            if (wasRejected) {
                CloseDocumentNoThrow(validationApplication, candidate);
                return AssetImportResult::InvalidData;
            }
            const AssetImportResult result = status == PCDM_RS_OK
                ? AssetImportResult::InvalidData
                : ImportResultForReaderStatus(status);
            CloseDocumentNoThrow(validationApplication, candidate);
            return result;
        }

        TDF_LabelMap activeDefinitionLabels;
        const bool isValid = HasValidOptionalLengthUnit(candidate)
            && ValidateShapeTree(
                candidate,
                activeDefinitionLabels,
                myMaximumProjectTopologyValidationNodes)
            && ValidateVisualMaterials(
                candidate, activeDefinitionLabels);
        CloseDocumentNoThrow(validationApplication, candidate);
        return isValid
            ? AssetImportResult::Success
            : AssetImportResult::InvalidData;
    } catch (const Standard_Failure& failure) {
        std::cout << "Validate CBF failure: " << failure.GetMessageString() << std::endl;
        CloseDocumentNoThrow(validationApplication, candidate);
        return AssetImportResult::InternalFailure;
    } catch (const std::bad_alloc&) {
        CloseDocumentNoThrow(validationApplication, candidate);
        return AssetImportResult::InternalFailure;
    } catch (const std::exception& exception) {
        std::cout << "Validate CBF exception: " << exception.what() << std::endl;
        CloseDocumentNoThrow(validationApplication, candidate);
        return AssetImportResult::InternalFailure;
    } catch (...) {
        CloseDocumentNoThrow(validationApplication, candidate);
        return AssetImportResult::InternalFailure;
    }
}

void Core3DViewer::redrawDocument() {
	const PrimitiveManipulatorType manipulatorType = _objectInteractor == nullptr
		? PrimitiveManipulatorType::PrimitiveGizmoTypeNone
		: _objectInteractor->getManipulatorType();
	const ShapeSelectionMode selectionMode = _shapeInteractor == nullptr
		? ShapeSelectionMode::WholeShape
		: _shapeInteractor->getSelectionMode();
    clearContext();
    traverseDocument(myDoc->ChangeDocument());
	recreateInteractors(manipulatorType, selectionMode);
    myContext->UpdateCurrentViewer();
}

void Core3DViewer::setPreviewMode() {
    myView->TriedronErase();
}

void Core3DViewer::showGrid(bool show) {
    if (show) {
        myViewer->ActivateGrid(Aspect_GridType::Aspect_GT_Rectangular, Aspect_GridDrawMode::Aspect_GDM_Lines);
    } else {
        myViewer->DeactivateGrid();
    }
    myView->Redraw();
}

void Core3DViewer::setOrthoProjection(const OrthoProjectionType orthoType) {

        V3d_TypeOfOrientation orientation = V3d_Yneg;

        switch (orthoType) {
            case OrthoProjectionTypeFront:
                orientation = V3d_Xpos;
                break;
            case OrthoProjectionTypeBack:
                orientation = V3d_Xneg;
                break;
            case OrthoProjectionTypeTop:
                orientation = V3d_Zpos;
                break;
            case OrthoProjectionTypeBottom:
                orientation = V3d_Zneg;
                break;
            case OrthoProjectionTypeLeft:
                orientation = V3d_Yneg;
                break;
            case OrthoProjectionTypeRight:
                orientation = V3d_Ypos;
                break;
            default:
                break;
        }

        myView->SetProj(orientation);

        if (myView->Camera()->ProjectionType() != Graphic3d_Camera::Projection_Orthographic) {
            myView->Camera()->SetProjectionType(Graphic3d_Camera::Projection_Orthographic);
			if (orthoType == OrthoProjectionTypeTop)
				myView->Camera()->SetUp(gp::DX());
			else if (orthoType == OrthoProjectionTypeBottom)
				myView->Camera()->SetUp(-gp::DX());
        }

        myView->FitAll(0.2, Standard_False);
        myView->RedrawImmediate();
    }

void Core3DViewer::StartRotation(int theX, int theY) {
    if(_objectInteractor == nullptr) {
        return;
    }
    if(!_objectInteractor->startTransformManipulator(theX, theY)) {
        OcctViewer::StartRotation(theX, theY);
    }
}

scene::OcctSceneSnapshotBuilder::SnapshotPointer
Core3DViewer::captureSceneSnapshot(
    const std::uint32_t viewportWidth,
    const std::uint32_t viewportHeight) noexcept {
    return _sceneSnapshotBuilder.Build(
        myDoc,
        myContext,
        myView,
        scene::UInt2{viewportWidth, viewportHeight});
}

std::optional<scene::FrameSnapshot>
Core3DViewer::captureSceneFrameSnapshot(
    const std::uint32_t viewportWidth,
    const std::uint32_t viewportHeight) noexcept {
    return _sceneSnapshotBuilder.CaptureFrame(
        myDoc,
        myView,
        scene::UInt2{viewportWidth, viewportHeight});
}

scene::OcctSceneSnapshotBuilder::OverlayPointer
Core3DViewer::captureScenePresentationOverlay() noexcept {
    if (_objectInteractor == nullptr) {
        return {};
    }
    scene::PresentationOverlayContent aContent;
    std::vector<Handle(AIS_Shape)> aMirrorPreviewObjects;
    BooleanPreviewCapture aBooleanPreview;
    if (_objectInteractor->captureIdlePresentationOverlay(
            aContent,
            aMirrorPreviewObjects,
            aBooleanPreview)
        != PresentationOverlayCaptureStatus::Available) {
        return {};
    }
    if (!aMirrorPreviewObjects.empty()) {
        return _sceneSnapshotBuilder.PublishMirrorPreviewOverlay(
            myDoc,
            std::move(aContent),
            aMirrorPreviewObjects);
    }
    if (!aBooleanPreview.actors.empty()
        || !aBooleanPreview.results.empty()) {
        return _sceneSnapshotBuilder.PublishBooleanPreviewOverlay(
            myDoc,
            aBooleanPreview.action == BooleanAction::BooleanSubtract
                ? scene::PresentationOverlayKind::BooleanSubtractPreview
                : scene::PresentationOverlayKind::BooleanUnionPreview,
            aBooleanPreview.actors,
            aBooleanPreview.results,
            aBooleanPreview.suppressedSourceLabels);
    }
    return _sceneSnapshotBuilder.PublishPresentationOverlay(
        myDoc,
        std::move(aContent));
}

#ifdef DEBUG
Standard_Boolean Core3DViewer::debugBeginBooleanSelection(
    const BooleanAction theAction,
    const std::vector<std::string>& theActorEntityIdentifiers,
    const std::vector<std::string>& theSubjectEntityIdentifiers) noexcept {
    if (_objectInteractor == nullptr || myContext.IsNull() || myDoc.IsNull()
        || (theAction == BooleanAction::BooleanUnion
            && !theActorEntityIdentifiers.empty())) {
        return Standard_False;
    }
    const PrimitiveManipulatorType anExpectedManipulator =
        theAction == BooleanAction::BooleanSubtract
            ? PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract
            : PrimitiveManipulatorType::PrimitiveGizmoTypeUnion;
    if (_objectInteractor->getManipulatorType()
        != anExpectedManipulator) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        _objectInteractor->cancelActiveBoolean();

        std::unordered_set<std::string> aRequested;
        for (const std::string& anIdentifier : theActorEntityIdentifiers) {
            if (anIdentifier.empty()
                || !aRequested.insert(anIdentifier).second) {
                return Standard_False;
            }
        }
        for (const std::string& anIdentifier : theSubjectEntityIdentifiers) {
            if (anIdentifier.empty()
                || !aRequested.insert(anIdentifier).second) {
                return Standard_False;
            }
        }

        std::unordered_map<std::string, Handle(AIS_InteractiveObject)>
            aCommittedPresentations;
        AIS_ListOfInteractive aDisplayed;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, aDisplayed);
        for (AIS_ListIteratorOfListOfInteractive anObject(aDisplayed);
             anObject.More(); anObject.Next()) {
            const Handle(AIS_InteractiveObject)& aPresentation =
                anObject.Value();
            const TDF_Label aLabel = myDoc->ShapeLabel(aPresentation);
            const std::string anIdentifier =
                myDoc->EntityIdentifierForLabel(aLabel);
            if (aLabel.IsNull() || anIdentifier.empty()
                || aRequested.find(anIdentifier) == aRequested.end()) {
                continue;
            }
            if (!aCommittedPresentations.emplace(
                    anIdentifier,
                    aPresentation).second) {
                return Standard_False;
            }
        }
        if (aCommittedPresentations.size() != aRequested.size()) {
            return Standard_False;
        }

        std::vector<Handle(AIS_InteractiveObject)> anActors;
        std::vector<Handle(AIS_InteractiveObject)> aSubjects;
        anActors.reserve(theActorEntityIdentifiers.size());
        aSubjects.reserve(theSubjectEntityIdentifiers.size());
        for (const std::string& anIdentifier : theActorEntityIdentifiers) {
            anActors.push_back(aCommittedPresentations.at(anIdentifier));
        }
        for (const std::string& anIdentifier : theSubjectEntityIdentifiers) {
            aSubjects.push_back(aCommittedPresentations.at(anIdentifier));
        }
        return _objectInteractor->debugBeginBooleanSelection(
            anActors,
            aSubjects,
            theAction);
    } catch (...) {
        _objectInteractor->cancelActiveBoolean();
        return Standard_False;
    }
}

BooleanPreviewDebugState
Core3DViewer::DebugBooleanPreviewState() const noexcept
{
    return _objectInteractor == nullptr
        ? BooleanPreviewDebugState()
        : _objectInteractor->debugBooleanPreviewState();
}

void Core3DViewer::DebugSetBooleanPreviewWorkerBlocked(
    const Standard_Boolean theBlocked) noexcept
{
    if (_objectInteractor != nullptr) {
        _objectInteractor->debugSetBooleanPreviewWorkerBlocked(
            theBlocked);
    }
}

void Core3DViewer::DebugSetMaximumBooleanCaptureTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    if (_objectInteractor != nullptr) {
        _objectInteractor->debugSetMaximumBooleanCaptureTopologyNodes(
            theLimit);
    }
}

void Core3DViewer::DebugSetMaximumBooleanResultTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    if (_objectInteractor != nullptr) {
        _objectInteractor->debugSetMaximumBooleanResultTopologyNodes(
            theLimit);
    }
}

void Core3DViewer::DebugSetMaximumBooleanResultSolids(
    const Standard_Size theLimit) noexcept
{
    if (_objectInteractor != nullptr) {
        _objectInteractor->debugSetMaximumBooleanResultSolids(theLimit);
    }
}

void Core3DViewer::DebugSetBooleanTransactionFailureCount(
    const Standard_Size theCount) noexcept
{
    if (_objectInteractor != nullptr) {
        _objectInteractor->debugSetBooleanTransactionFailureCount(
            theCount);
    }
}

void Core3DViewer::DebugSetBooleanAbortFailureCount(
    const Standard_Size theCount) noexcept
{
    if (_objectInteractor != nullptr) {
        _objectInteractor->debugSetBooleanAbortFailureCount(theCount);
    }
}

Standard_Boolean Core3DViewer::debugBeginExtrusionSelection(
    const std::string& theEntityIdentifier,
    const Standard_Size theFaceTopologyIndex) noexcept {
    if (_shapeInteractor == nullptr || myContext.IsNull() || myDoc.IsNull()
        || theEntityIdentifier.empty()) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        if (!_shapeInteractor->cancelExtrusion()) {
            return Standard_False;
        }
        Handle(AIS_Shape) aPresentation;
        AIS_ListOfInteractive aDisplayed;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, aDisplayed);
        for (AIS_ListIteratorOfListOfInteractive anObject(aDisplayed);
             anObject.More(); anObject.Next()) {
            const Handle(AIS_InteractiveObject)& anInteractive =
                anObject.Value();
            const TDF_Label aLabel = myDoc->ShapeLabel(anInteractive);
            if (aLabel.IsNull()
                || myDoc->EntityIdentifierForLabel(aLabel)
                    != theEntityIdentifier) {
                continue;
            }
            const Handle(AIS_Shape) aCandidate =
                Handle(AIS_Shape)::DownCast(anInteractive);
            if (!aPresentation.IsNull() || aCandidate.IsNull()
                || !myDoc->IsPresentationEditable(aCandidate)) {
                return Standard_False;
            }
            aPresentation = aCandidate;
        }
        if (aPresentation.IsNull() || aPresentation->Shape().IsNull()) {
            return Standard_False;
        }
        Standard_Size aFaceIndex = 0;
        for (TopExp_Explorer aFace(
                 aPresentation->Shape(), TopAbs_FACE);
             aFace.More(); aFace.Next(), ++aFaceIndex) {
            if (aFaceIndex == theFaceTopologyIndex) {
                return _shapeInteractor->debugBeginExtrusionSelection(
                    aPresentation,
                    TopoDS::Face(aFace.Current()));
            }
        }
        return Standard_False;
    } catch (...) {
        _shapeInteractor->cancelExtrusion();
        return Standard_False;
    }
}
#endif

void Core3DViewer::Rotation(int theX, int theY) {
    if(_objectInteractor == nullptr) {
        return;
    }
    if(!_objectInteractor->transformManipulator(theX, theY)){
        OcctViewer::Rotation(theX, theY);
        myContext->UpdateCurrentViewer();
    }
    
    if(_interactiveCallback != nullptr) {
        _interactiveCallback(theX, theY);
    }
}

void Core3DViewer::FinishInteraction(int theX, int theY) {
    if(_objectInteractor != nullptr) {
        _objectInteractor->finishInteraction();
    }
}

void Core3DViewer::CancelInteraction(int theX, int theY) {
    if(_objectInteractor != nullptr) {
		_objectInteractor->cancelInteraction();
    }
}

const int Core3DViewer::selectedCount() const {
    int count = 0;
    for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
        ++count;
    }
    return count;
}

void Core3DViewer::Select(int theX, int theY) {
    if (_objectInteractor == nullptr || _shapeInteractor == nullptr) {
        printf("ERROR with Select\n");
        return;
    }
    if (_objectInteractor->isBooleanSelectionFrozen()) {
        return;
    }
    if (_shapeInteractor->hasActiveExtrusion()) {
        return;
    }

    switch(_objectInteractor->getManipulatorType()) {
        case PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract:
        case PrimitiveManipulatorType::PrimitiveGizmoTypeUnion:
            if (!hitTest(theX, theY)) {
                return;
            }
            break;
        default:
            break;
    }

    int oldCount = selectedCount();
    // cancel chamfer when empty tapped
    //		if (!hitTest(theX, theY) && _objectInteractor->getManipulatorType() == PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer) {
    //            _shapeInteractor->setChamferValueForSelection(0);
    //            _shapeInteractor->resetWireframeTemplateShape();
    //		}

    bool isBoleanOp = _objectInteractor->getManipulatorType() == PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract;
//	bool isMirrorOp = _objectInteractor->getManipulatorType() == PrimitiveManipulatorType::PrimitiveGizmoTypeMirror;
//	OcctViewer::Select(theX, theY, isMirrorOp ? AIS_SelectionScheme::AIS_SelectionScheme_Replace : AIS_SelectionScheme::AIS_SelectionScheme_XOR);
    OcctViewer::Select(theX, theY, AIS_SelectionScheme::AIS_SelectionScheme_XOR);
    //        OcctViewer::Select(theX, theY, isBoleanOp ? AIS_SelectionScheme::AIS_SelectionScheme_Add : AIS_SelectionScheme::AIS_SelectionScheme_XOR );

    int newCount = selectedCount();
    std::cout << "count before =" << oldCount << ", count after=" << newCount << ", isBoleanOp=" << isBoleanOp << std::endl;

    _objectInteractor->attachManipulatorToSelection(newCount < oldCount);
    
    switch (_objectInteractor->getManipulatorType()) {
        case PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer:
            if (PREVENT_RECHAMFER && _shapeInteractor->saveSelectionEdges() == 0) {
                myContext->ClearSelected(Standard_False);
            }
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract:
            _objectInteractor->updateDetectedState(true, BooleanAction::BooleanSubtract);
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeUnion:
            _objectInteractor->updateDetectedState(false, BooleanAction::BooleanUnion);
            break;
        default:
            break;
    }

    redraw();
}

	void Core3DViewer::deselectAll() {
		if (myContext.IsNull()) { return; }
		if (_shapeInteractor != nullptr
			&& !_shapeInteractor->cancelExtrusion()) {
			return;
		}
		if (_objectInteractor != nullptr) {
			_objectInteractor->cancelInteraction();
		}
        myContext->SelectDetected(AIS_SelectionScheme::AIS_SelectionScheme_Remove);
        myContext->ClearSelected(Standard_True);
        if (_objectInteractor == nullptr) { return; }
        if (_objectInteractor->getManipulatorType() == PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer
            && _shapeInteractor != nullptr) {
            _shapeInteractor->resetWireframeTemplateShape();
        }
        redraw();
    }

    void Core3DViewer::redraw() {
        myView->Redraw();
    }

    const bool Core3DViewer::hitTest(const int x, const int y) const {
        return myContext->MoveTo(x, y, myView, Standard_False) != AIS_SOD_Nothing;
    }

    bool Core3DViewer::dumpOfDisplayedColoredObjects(const Standard_Integer width,
                                                     const Standard_Integer height,
                                                     const TCollection_AsciiString &fileName) {

        // prepare viewer
        Handle(Aspect_DisplayConnection) displayConnection = new Aspect_DisplayConnection();
        Handle(OpenGl_GraphicDriver) graphicDriver = new OpenGl_GraphicDriver(displayConnection);
        Handle(V3d_Viewer) viewer = new V3d_Viewer(graphicDriver);
        viewer->SetDefaultTypeOfView(V3d_PERSPECTIVE);
        Handle(V3d_DirectionalLight) lightDir = new V3d_DirectionalLight(V3d_Zneg, Quantity_Color(Quantity_NOC_WHITE), Standard_True);
        Handle(V3d_AmbientLight)     lightAmb = new V3d_AmbientLight();
        lightDir->SetDirection(1.0, -2.0, -10.0);
        viewer->AddLight(lightDir);
        viewer->AddLight(lightAmb);
        viewer->SetLightOn(lightDir);
        viewer->SetLightOn(lightAmb);

        // prepare context
        Handle(AIS_InteractiveContext) context = new AIS_InteractiveContext(viewer);
        // prepare off-screen view
        Handle(V3d_View) view = viewer->CreateView();
        Handle(Aspect_NeutralWindow) wnd = new Aspect_NeutralWindow();
        EAGLContext* aRendCtx = [EAGLContext currentContext];
        wnd->SetSize(width, height);
        wnd->SetVirtual(true);
        view->SetWindow(wnd, aRendCtx);
        view->SetBackgroundColor(Quantity_Color(Quantity_NOC_BLACK));
        view->MustBeResized();
		//			Graphic3d_RenderingParams& aParams = view->ChangeRenderingParams();
		//			aParams.FrustumCullingState = Graphic3d_RenderingParams::FrustumCulling::FrustumCulling_On;
		//			aParams.Method = Graphic3d_RenderingMode::Graphic3d_RM_RASTERIZATION;
		//			aParams.Exposure = 2;
		//			aParams.RaytracingDepth = 3;
		//			aParams.SamplesPerPixel = 2;
		//			aParams.CoherentPathTracingMode = true;
		//			aParams.IsReflectionEnabled = false;
		//			aParams.IsTransparentShadowEnabled = true;
		//			aParams.TwoSidedBsdfModels = true;
		//			aParams.ToneMappingMethod = Graphic3d_ToneMappingMethod_Filmic;
		//			aParams.ToEnableDepthPrepass = true;
		//			aParams.TransparencyMethod = Graphic3d_RenderTransparentMethod::Graphic3d_RTM_DEPTH_PEELING_OIT;
		//			aParams.RebuildRayTracingShaders = true;
		//			aParams.IsAntialiasingEnabled = false;
		//			aParams.NbMsaaSamples = 1;
		//			aParams.ShadingModel = Graphic3d_TypeOfShadingModel::Graphic3d_TOSM_FRAGMENT;
		//			aParams.UseEnvironmentMapBackground = false;
		//			aParams.IsGlobalIlluminationEnabled = false;

        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        AIS_ListIteratorOfListOfInteractive iobject(objects);

        while (iobject.More()) {
            TopoDS_Shape shape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(iobject.Value())->Shape(),
                                                          iobject.Value()->LocalTransformation());

            // prepare presentation filled by shape
            auto color = Quantity_Color();
            Handle(AIS_Shape)::DownCast(iobject.Value())->Color(color);
            Handle(AIS_Shape) presentation = new AIS_Shape(shape);
			presentation->UnsetColor();
			auto mat = myDoc->MaterialNameForShape(Handle(AIS_Shape)::DownCast(iobject.Value()));
			presentation->SetMaterial(mat);
			presentation->SetColor(color.Name());
            context->Display(presentation, Standard_False);
            context->SetDisplayMode(presentation, AIS_Shaded, Standard_False);

            iobject.Next();
        }

        view->FitAll();
        view->Redraw();
        // prepare pixmap image
        Image_AlienPixMap img;
        if (!view->ToPixMap(img, width, height))
            return false;

        CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
        CGBitmapInfo bitmapInfo = kCGImageByteOrderDefault;

        const auto sz = sizeof(unsigned char)*img.SizeBytes();

        CGDataProviderRef provider = CGDataProviderCreateWithData(nil, img.Data(), sz, nil);
        CGImageRef cgImageRef = CGImageCreate(width, height, 8, 24, img.SizeRowBytes(), colorSpace, bitmapInfo, provider, nil, false, (CGColorRenderingIntent)kCGRenderingIntentDefault);

        UIGraphicsBeginImageContext(CGSizeMake(width, height));
        [[UIImage imageWithCGImage:cgImageRef scale:1.0 orientation:UIImageOrientationDownMirrored] drawInRect:CGRectMake(0,0,width ,height)];
        UIImage* newImage = UIGraphicsGetImageFromCurrentImageContext();
        UIGraphicsEndImageContext();

		CIImage *ciImage = [CIImage imageWithCGImage:newImage.CGImage];
		CIFilter* filterGA = [CIFilter filterWithName:@"CIGammaAdjust"];
		[filterGA setValue:ciImage forKey:@"inputImage"];
		[filterGA setValue: [NSNumber numberWithFloat:2.0f] forKey: @"inputPower"];
		CIImage *resultImage = [filterGA valueForKey: @"outputImage"];
		UIImage* newImageGamma = [UIImage imageWithCIImage:resultImage];
		NSData* pngDataRep = UIImagePNGRepresentation(newImageGamma);

        CFRelease(provider);
        CFRelease(colorSpace);
        CFRelease(cgImageRef);

        return [pngDataRep writeToFile:[NSString stringWithCString:fileName.ToCString() encoding:NSASCIIStringEncoding] atomically:YES];
    }

    bool Core3DViewer::dumpOfDisplayedObjects(const Standard_Integer width, 
                                              const Standard_Integer height,
                                              const TCollection_AsciiString &fileName) {
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        AIS_ListIteratorOfListOfInteractive iobject(objects);

        TopoDS_Compound resultShape;
        BRep_Builder builder;
        builder.MakeCompound(resultShape);

        while (iobject.More()) {
            TopoDS_Shape shape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(iobject.Value())->Shape(),
                                                          iobject.Value()->LocalTransformation());
			if (!shape.IsNull())
				builder.Add(resultShape, shape);
            iobject.Next();
        }

        return dumpShape(resultShape, width, height, fileName);
    }

    bool Core3DViewer::dumpShape(const TopoDS_Shape& shape,
                                 const Standard_Integer width,
                                 const Standard_Integer height,
                                 const TCollection_AsciiString& fileName) {
        // prepare viewer
        Handle(Aspect_DisplayConnection) displayConnection = new Aspect_DisplayConnection();
        Handle(OpenGl_GraphicDriver) graphicDriver = new OpenGl_GraphicDriver(displayConnection);
        Handle(V3d_Viewer) viewer = new V3d_Viewer(graphicDriver);
		viewer->SetDefaultTypeOfView(V3d_PERSPECTIVE);
        Handle(V3d_DirectionalLight) lightDir = new V3d_DirectionalLight(V3d_Zneg, Quantity_Color(Quantity_NOC_WHITE), Standard_True);
        Handle(V3d_AmbientLight)     lightAmb = new V3d_AmbientLight();
        lightDir->SetDirection(1.0, -2.0, -10.0);
        viewer->AddLight(lightDir);
        viewer->AddLight(lightAmb);
        viewer->SetLightOn(lightDir);
        viewer->SetLightOn(lightAmb);

        // prepare context
        Handle(AIS_InteractiveContext) context = new AIS_InteractiveContext(viewer);
        // prepare off-screen view
        Handle(V3d_View) view = viewer->CreateView();
        Handle(Aspect_NeutralWindow) wnd = new Aspect_NeutralWindow();
        EAGLContext* aRendCtx = [EAGLContext currentContext];
        wnd->SetSize(width, height);
        wnd->SetVirtual(true);
        view->SetWindow(wnd, aRendCtx);
        view->SetBackgroundColor(Quantity_Color(Quantity_NOC_BLACK));
        view->MustBeResized();
        // prepare presentation filled by shape
        Handle(AIS_Shape) presentation = new AIS_Shape(shape);
        presentation->SetMaterial(Graphic3d_NameOfMaterial_ShinyPlastified);
        presentation->SetColor(Quantity_NOC_GRAY50);
        context->Display(presentation, Standard_False);
        context->SetDisplayMode(presentation, AIS_Shaded, Standard_False);
        view->FitAll();
        view->Redraw();
        // prepare pixmap image
        Image_AlienPixMap img;
        if (!view->ToPixMap(img, width, height))
            return false;

        CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
        CGBitmapInfo bitmapInfo = kCGImageByteOrderDefault;

        const auto sz = sizeof(unsigned char)*img.SizeBytes();

        CGDataProviderRef provider = CGDataProviderCreateWithData(nil, img.Data(), sz, nil);
        CGImageRef cgImageRef = CGImageCreate(width, height, 8, 24, img.SizeRowBytes(), colorSpace, bitmapInfo, provider, nil, false, (CGColorRenderingIntent)kCGRenderingIntentDefault);

        UIGraphicsBeginImageContext(CGSizeMake(width, height));
        [[UIImage imageWithCGImage:cgImageRef scale:1.0 orientation:UIImageOrientationDownMirrored] drawInRect:CGRectMake(0,0,width ,height)];
        UIImage* newImage = UIGraphicsGetImageFromCurrentImageContext();
        UIGraphicsEndImageContext();

        NSData* pngDataRep = UIImagePNGRepresentation(newImage);

        CFRelease(provider);
        CFRelease(colorSpace);
        CFRelease(cgImageRef);

        return [pngDataRep writeToFile:[NSString stringWithCString:fileName.ToCString() encoding:NSASCIIStringEncoding] atomically:YES];

//        // save image into a file
//        return img.Save(fileName);
    }

    bool Core3DViewer::saveSnapshot(const TCollection_AsciiString& thePath,
                                    int theWidth,
                                    int theHeight) {
        showGrid(false);
        myView->TriedronErase();
        myView->FitAll();
        const auto result = myView->Dump(thePath.ToCString());
        showGrid(true);
        myView->TriedronDisplay (Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.20, V3d_ZBUFFER); //FIXME: tmp solution!
        return result;

// FIXME: commented non-working code
//        if (myContext.IsNull() || thePath.IsEmpty()) {
//            Message::DefaultMessenger()->Send ("Image dump failed - view is unavailable", Message_Fail);
//            return false;
//        }
//
//        if (theWidth  < 1 || theHeight < 1) {
//            myView->Window()->Size(theWidth, theHeight);
//        }
//        if (theWidth  < 1 || theHeight < 1) {
//            Message::DefaultMessenger()->Send ("Image dump failed - view is unavailable", Message_Fail);
//            return false;
//        }
//
//        Image_AlienPixMap anAlienImage;
//        if (!anAlienImage.InitTrash(Image_Format::Image_Format_BGRA, theWidth, theHeight)) {
//            Message::DefaultMessenger()->Send (TCollection_AsciiString() + "RGBA image " + theWidth + "x" + theHeight + " allocation failed", Message_Fail);
//            return false;
//        }
//
//        // OpenGL ES does not support fetching data in BGRA format
//        // while FreeImage does not support RGBA format.
//        Image_PixMap anImage;
//        anImage.InitWrapper (Image_Format::Image_Format_BGRA,
//                             anAlienImage.ChangeData(),
//                             anAlienImage.SizeX(),
//                             anAlienImage.SizeY(),
//                             anAlienImage.SizeRowBytes());
//        if (!myView->ToPixMap (anImage, theWidth, theHeight, Graphic3d_BT_RGBA)) {
//            Message::DefaultMessenger()->Send (TCollection_AsciiString() + "View dump to the image " + theWidth + "x" + theHeight + " failed", Message_Fail);
//        }
//
//        for (Standard_Size aRow = 0; aRow < anAlienImage.SizeY(); ++aRow) {
//            for (Standard_Size aCol = 0; aCol < anAlienImage.SizeX(); ++aCol) {
//                Image_ColorRGBA& aPixel = anAlienImage.ChangeValue<Image_ColorRGBA> (aRow, aCol);
//                std::swap (aPixel.r(), aPixel.b());
//                //aPixel.a() = 1.0;
//            }
//        }
//
//        if (!anAlienImage.Save (thePath)) {
//            Message::DefaultMessenger()->Send (TCollection_AsciiString() + "Image saving to path '" + thePath + "' failed", Message_Fail);
//            return false;
//        }
//        Message::DefaultMessenger()->Send (TCollection_AsciiString() + "View " + theWidth + "x" + theHeight + " dumped to image '" + thePath + "'", Message_Info);
//        return true;
    }
}
