//
//  RadialArrayOperationController.cpp
//  Core3D
//

#include "RadialArrayOperationController.hpp"

#include "../Common/Core3DMobileResourceLimits.h"

#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRep_Tool.hxx>
#include <Poly_ListOfTriangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_TriangulationParameters.hxx>
#include <Precision.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <TColStd_ListOfInteger.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_Real.hxx>
#include <TDF_Attribute.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterial.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <utility>

namespace core3d {
namespace {

constexpr Standard_Real kPi =
    3.141592653589793238462643383279502884;
constexpr Standard_Real kLegacyMetersPerUnit = 0.001;

// Authority leases must survive controller/interactor recreation without ABA.
// Exhaustion is fail-closed: UINT64_MAX is never wrapped or reissued.
std::atomic<std::uint64_t> gNextReferenceAuthorityToken{0};

Standard_Boolean TryAcquireReferenceAuthorityToken(
    std::uint64_t& theToken) noexcept
{
    std::uint64_t aCurrent =
        gNextReferenceAuthorityToken.load(std::memory_order_relaxed);
    for (;;) {
        if (aCurrent == std::numeric_limits<std::uint64_t>::max()) {
            theToken = 0;
            return Standard_False;
        }
        const std::uint64_t aNext = aCurrent + 1U;
        if (gNextReferenceAuthorityToken.compare_exchange_weak(
                aCurrent,
                aNext,
                std::memory_order_relaxed,
                std::memory_order_relaxed)) {
            theToken = aNext;
            return Standard_True;
        }
    }
}

OcctGeometryRepresentation DestinationRepresentation(
    const OcctGeometryRepresentation theRepresentation) noexcept
{
    return theRepresentation == OcctGeometryRepresentation::LegacyUnknown
        ? OcctGeometryRepresentation::BRep
        : theRepresentation;
}

Standard_Boolean IsSupportedRepresentation(
    const OcctGeometryRepresentation theRepresentation) noexcept
{
    return theRepresentation == OcctGeometryRepresentation::LegacyUnknown
        || theRepresentation == OcctGeometryRepresentation::BRep
        || theRepresentation == OcctGeometryRepresentation::TriangleMesh;
}

Standard_Boolean HasExactPresentationStyle(
    const Handle(AIS_Shape)& thePresentation,
    const Graphic3d_NameOfMaterial theMaterialName,
    const Quantity_Color& theColor,
    const Standard_Real theTransparency)
{
    if (thePresentation.IsNull()
        || !thePresentation->HasColor()
        || !thePresentation->HasMaterial()
        || theMaterialName < Graphic3d_NameOfMaterial_Brass
        || theMaterialName > Graphic3d_NameOfMaterial_Transparent
        || !std::isfinite(theTransparency)
        || theTransparency < 0.0 || theTransparency > 1.0
        || thePresentation->Material() != theMaterialName
        || thePresentation->Transparency() != theTransparency) {
        return Standard_False;
    }
    Quantity_Color aColor;
    thePresentation->Color(aColor);
    return aColor.Red() == theColor.Red()
        && aColor.Green() == theColor.Green()
        && aColor.Blue() == theColor.Blue();
}

Standard_Boolean TransformDiffers(
    const gp_Trsf& theLeft,
    const gp_Trsf& theRight) noexcept
{
    for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
        for (Standard_Integer aColumn = 1; aColumn <= 4; ++aColumn) {
            if (std::abs(theLeft.Value(aRow, aColumn)
                    - theRight.Value(aRow, aColumn))
                > Precision::Confusion()) {
                return Standard_True;
            }
        }
    }
    return Standard_False;
}

Standard_Boolean ReferenceAxisDiffers(
    const OcctReferenceAxis& theLeft,
    const OcctReferenceAxis& theRight) noexcept
{
    constexpr Standard_Real aTolerance = 1.0e-12;
    try {
        return theLeft.pivotSpace != theRight.pivotSpace
            || theLeft.directionSpace != theRight.directionSpace
            || !theLeft.pivot.IsEqual(theRight.pivot, aTolerance)
            || !theLeft.direction.IsEqual(
                theRight.direction, aTolerance);
    } catch (...) {
        return Standard_True;
    }
}

Standard_Boolean WorldAxisDiffers(
    const gp_Ax1& theLeft,
    const gp_Ax1& theRight) noexcept
{
    constexpr Standard_Real aTolerance = 1.0e-10;
    try {
        return !theLeft.Location().IsEqual(
                theRight.Location(), aTolerance)
            || !theLeft.Direction().IsEqual(
                theRight.Direction(), aTolerance);
    } catch (...) {
        return Standard_True;
    }
}

Standard_Real StepRadians(
    const Standard_Real theSweepDegrees,
    const Standard_Integer theCount) noexcept
{
    if (!std::isfinite(theSweepDegrees) || theCount <= 0) {
        return std::numeric_limits<Standard_Real>::quiet_NaN();
    }
    return theSweepDegrees * kPi
        / (180.0 * static_cast<Standard_Real>(theCount));
}

Standard_Boolean IsNearZeroStep(
    const Standard_Real theSweepDegrees,
    const Standard_Integer theCount) noexcept
{
    const Standard_Real aStep = StepRadians(theSweepDegrees, theCount);
    return !std::isfinite(aStep)
        || std::abs(aStep) <= Precision::Angular();
}

Standard_Boolean TryReadMetersPerUnit(
    const Handle(TDocStd_Document)& theDocument,
    Standard_Real& theMetersPerUnit) noexcept
{
    theMetersPerUnit = kLegacyMetersPerUnit;
    if (theDocument.IsNull()) {
        return Standard_False;
    }
    try {
        const Standard_Boolean hasLengthUnit =
            XCAFDoc_DocumentTool::GetLengthUnit(
                theDocument, theMetersPerUnit);
        return (hasLengthUnit && std::isfinite(theMetersPerUnit)
                && theMetersPerUnit > 0.0)
            || (!hasLengthUnit
                && theMetersPerUnit == kLegacyMetersPerUnit);
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean IsTopologicallyValid(const TopoDS_Shape& theShape) noexcept
{
    if (theShape.IsNull()) {
        return Standard_False;
    }
    try {
        BRepCheck_Analyzer anAnalyzer(theShape, Standard_True);
        return anAnalyzer.IsValid();
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean CountBoundedTopology(
    const TopoDS_Shape& theShape,
    const Standard_Size theMaximum,
    Standard_Size& theCount) noexcept
{
    theCount = 0;
    if (theShape.IsNull() || theMaximum == 0) {
        return Standard_False;
    }
    try {
        std::vector<TopoDS_Shape> aPending{theShape};
        while (!aPending.empty()) {
            const TopoDS_Shape aCurrent = aPending.back();
            aPending.pop_back();
            if (aCurrent.IsNull() || ++theCount > theMaximum) {
                theCount = 0;
                return Standard_False;
            }
            std::vector<TopoDS_Shape> aChildren;
            for (TopoDS_Iterator aChild(
                     aCurrent, Standard_True, Standard_True);
                 aChild.More(); aChild.Next()) {
                const Standard_Size aRemaining = theMaximum - theCount;
                if (aPending.size() + aChildren.size() + 1U
                    > static_cast<std::size_t>(aRemaining)) {
                    theCount = 0;
                    return Standard_False;
                }
                aChildren.push_back(aChild.Value());
            }
            for (auto aChild = aChildren.rbegin();
                 aChild != aChildren.rend(); ++aChild) {
                aPending.push_back(*aChild);
            }
        }
        return theCount > 0;
    } catch (...) {
        theCount = 0;
        return Standard_False;
    }
}

Standard_Boolean RestoreTriangleMeshCopyParameters(
    const TopoDS_Shape& theSource,
    BRepBuilderAPI_Copy& theCopy) noexcept
{
    try {
        Standard_Size aFaceCount = 0;
        for (TopExp_Explorer aFaceExplorer(theSource, TopAbs_FACE);
             aFaceExplorer.More(); aFaceExplorer.Next()) {
            const TopoDS_Face aSourceFace =
                TopoDS::Face(aFaceExplorer.Current());
            const TopoDS_Shape aCopiedShape =
                theCopy.ModifiedShape(aSourceFace);
            if (aCopiedShape.IsNull()
                || aCopiedShape.ShapeType() != TopAbs_FACE) {
                return Standard_False;
            }
            const TopoDS_Face aCopiedFace = TopoDS::Face(aCopiedShape);
            TopLoc_Location aSourceLocation;
            TopLoc_Location aCopiedLocation;
            const Poly_ListOfTriangulation& aSourceMeshes =
                BRep_Tool::Triangulations(aSourceFace, aSourceLocation);
            const Poly_ListOfTriangulation& aCopiedMeshes =
                BRep_Tool::Triangulations(aCopiedFace, aCopiedLocation);
            if (aSourceMeshes.Size() != 1 || aCopiedMeshes.Size() != 1) {
                return Standard_False;
            }
            const Handle(Poly_Triangulation)& aSourceMesh =
                BRep_Tool::Triangulation(aSourceFace, aSourceLocation);
            const Handle(Poly_Triangulation)& aCopiedMesh =
                BRep_Tool::Triangulation(aCopiedFace, aCopiedLocation);
            if (aSourceMesh.IsNull() || aCopiedMesh.IsNull()
                || aCopiedFace.IsPartner(aSourceFace)
                || aCopiedFace.Orientation() != aSourceFace.Orientation()
                || !aCopiedLocation.IsEqual(aSourceLocation)
                || aSourceMesh == aCopiedMesh
                || aSourceMesh->Deflection() != aCopiedMesh->Deflection()
                || aSourceMesh->NbNodes() != aCopiedMesh->NbNodes()
                || aSourceMesh->NbTriangles()
                    != aCopiedMesh->NbTriangles()
                || aSourceMesh->HasUVNodes() != aCopiedMesh->HasUVNodes()
                || aSourceMesh->HasNormals() != aCopiedMesh->HasNormals()
                || aSourceMesh->MeshPurpose()
                    != aCopiedMesh->MeshPurpose()) {
                return Standard_False;
            }
            const Handle(Poly_TriangulationParameters)& aParameters =
                aSourceMesh->Parameters();
            if (!aParameters.IsNull()) {
                aCopiedMesh->Parameters(new Poly_TriangulationParameters(
                    aParameters->Deflection(),
                    aParameters->Angle(),
                    aParameters->MinSize()));
                const Handle(Poly_TriangulationParameters)& aCopiedParameters =
                    aCopiedMesh->Parameters();
                if (aCopiedParameters.IsNull()
                    || aCopiedParameters->Deflection()
                        != aParameters->Deflection()
                    || aCopiedParameters->Angle() != aParameters->Angle()
                    || aCopiedParameters->MinSize()
                        != aParameters->MinSize()) {
                    return Standard_False;
                }
            } else if (!aCopiedMesh->Parameters().IsNull()) {
                return Standard_False;
            }
            ++aFaceCount;
        }
        return aFaceCount > 0;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean HasStyledSubshape(
    const Handle(TDocStd_Document)& theDocument,
    const TDF_Label& theDefinition,
    const Standard_Size theMaximumLabels) noexcept
{
    if (theDocument.IsNull() || theDefinition.IsNull()
        || theDefinition.Data() != theDocument->GetData()
        || theMaximumLabels == 0) {
        return Standard_True;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(XCAFDoc_ColorTool) aColorTool =
            XCAFDoc_DocumentTool::CheckColorTool(theDocument->Main())
            ? XCAFDoc_DocumentTool::ColorTool(theDocument->Main())
            : Handle(XCAFDoc_ColorTool)();
        const Handle(XCAFDoc_LayerTool) aLayerTool =
            XCAFDoc_DocumentTool::CheckLayerTool(theDocument->Main())
            ? XCAFDoc_DocumentTool::LayerTool(theDocument->Main())
            : Handle(XCAFDoc_LayerTool)();
        Standard_Size aLabelCount = 0;
        for (TDF_ChildIterator anItem(theDefinition, Standard_False);
             anItem.More(); anItem.Next()) {
            if (++aLabelCount > theMaximumLabels) {
                return Standard_True;
            }
            const TDF_Label& aLabel = anItem.Value();
            if (!XCAFDoc_ShapeTool::IsSubShape(aLabel)) {
                continue;
            }
            TDF_LabelSequence aLayers;
            if (aLabel.IsNull()
                || (!aColorTool.IsNull()
                    && (aColorTool->IsSet(aLabel, XCAFDoc_ColorGen)
                        || aColorTool->IsSet(aLabel, XCAFDoc_ColorSurf)
                        || aColorTool->IsSet(aLabel, XCAFDoc_ColorCurv)
                        || !XCAFDoc_ColorTool::IsVisible(aLabel)))
                || !XCAFDoc_VisMaterialTool::GetShapeMaterial(aLabel).IsNull()
                || (!aLayerTool.IsNull()
                    && aLayerTool->GetLayers(aLabel, aLayers)
                    && !aLayers.IsEmpty())) {
                return Standard_True;
            }
        }
        return Standard_False;
    } catch (...) {
        return Standard_True;
    }
}

Standard_Boolean HasUnsupportedOverlayTexture(
    const TDF_Label& theLabel) noexcept
{
    try {
        const Handle(XCAFDoc_VisMaterial) aMaterial =
            XCAFDoc_VisMaterialTool::GetShapeMaterial(theLabel);
        if (aMaterial.IsNull()) {
            return Standard_False;
        }
        if (aMaterial->HasPbrMaterial()) {
            const XCAFDoc_VisMaterialPBR& aPbr = aMaterial->PbrMaterial();
            if (!aPbr.BaseColorTexture.IsNull()
                || !aPbr.EmissiveTexture.IsNull()
                || !aPbr.MetallicRoughnessTexture.IsNull()
                || !aPbr.OcclusionTexture.IsNull()
                || !aPbr.NormalTexture.IsNull()) {
                return Standard_True;
            }
        }
        return aMaterial->HasCommonMaterial()
            && !aMaterial->CommonMaterial().DiffuseTexture.IsNull();
    } catch (...) {
        return Standard_True;
    }
}

Standard_Boolean WorldBounds(
    const TopoDS_Shape& theShape,
    const gp_Trsf& theTransform,
    Standard_Real (&theMinimum)[3],
    Standard_Real (&theMaximum)[3]) noexcept
{
    if (theShape.IsNull()) {
        return Standard_False;
    }
    try {
        Bnd_Box aBox;
        BRepBndLib::Add(theShape, aBox, Standard_True);
        if (aBox.IsVoid() || aBox.IsOpen()) {
            return Standard_False;
        }
        aBox = aBox.Transformed(theTransform);
        aBox.Get(
            theMinimum[0], theMinimum[1], theMinimum[2],
            theMaximum[0], theMaximum[1], theMaximum[2]);
        for (Standard_Integer anAxis = 0; anAxis < 3; ++anAxis) {
            if (!std::isfinite(theMinimum[anAxis])
                || !std::isfinite(theMaximum[anAxis])
                || std::abs(theMinimum[anAxis])
                    > limits::kMaximumModelCoordinateMagnitude
                || std::abs(theMaximum[anAxis])
                    > limits::kMaximumModelCoordinateMagnitude) {
                return Standard_False;
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean HasBoundedPivotRadius(
    const Standard_Real (&theMinimum)[3],
    const Standard_Real (&theMaximum)[3],
    const gp_Ax1& theAxis) noexcept
{
    try {
        const gp_Pnt& aPivot = theAxis.Location();
        Standard_Real aMaximumRadius = 0.0;
        for (Standard_Integer aMask = 0; aMask < 8; ++aMask) {
            const gp_Pnt aCorner(
                (aMask & 1) ? theMaximum[0] : theMinimum[0],
                (aMask & 2) ? theMaximum[1] : theMinimum[1],
                (aMask & 4) ? theMaximum[2] : theMinimum[2]);
            const Standard_Real aRadius = aPivot.Distance(aCorner);
            if (!std::isfinite(aRadius)) {
                return Standard_False;
            }
            aMaximumRadius = std::max(aMaximumRadius, aRadius);
        }
        return aMaximumRadius
            <= RadialArrayOperationController::kMaximumPivotRadius;
    } catch (...) {
        return Standard_False;
    }
}

gp_Trsf ArrayTransform(
    const gp_Trsf& theSourceTransform,
    const gp_Ax1& theWorldAxis,
    const Standard_Integer theOrdinal,
    const Standard_Real theSweepDegrees,
    const Standard_Integer theCount)
{
    gp_Trsf aWorldRotation;
    aWorldRotation.SetRotation(
        theWorldAxis,
        static_cast<Standard_Real>(theOrdinal)
            * StepRadians(theSweepDegrees, theCount));
    // World-space rotation is deliberately pre-multiplied. This preserves
    // Ti = R(P,A,theta_i) * T for translated/rotated/scaled source objects.
    return aWorldRotation.Multiplied(theSourceTransform);
}

} // namespace

RadialArrayOperationController::RadialArrayOperationController(
    Handle(AIS_InteractiveContext) theContext,
    Handle(OcctDocument) theDocument)
    : _context(std::move(theContext)),
      _document(std::move(theDocument))
{
}

RadialArrayOperationController::~RadialArrayOperationController() noexcept
{
    _previewStateChangedCallback = {};
    for (Standard_Integer anAttempt = 0;
         anAttempt < 3 && _ownsDocumentCommand; ++anAttempt) {
        (void)abortOwnedCommand();
    }
    (void)clearPreviewObjects();
}

void RadialArrayOperationController::notifyStateChanged() noexcept
{
    try {
        if (_previewStateChangedCallback) {
            _previewStateChangedCallback();
        }
    } catch (...) {
    }
}

void RadialArrayOperationController::setPreviewStateChangedCallback(
    std::function<void()> theCallback)
{
    _previewStateChangedCallback = std::move(theCallback);
}

Standard_Boolean
RadialArrayOperationController::advanceReferenceAuthorityToken() noexcept
{
    std::uint64_t aToken = 0;
    if (!TryAcquireReferenceAuthorityToken(aToken)) {
        _referenceAuthorityToken = 0;
        return Standard_False;
    }
    _referenceAuthorityToken = aToken;
    return Standard_True;
}

Standard_Boolean RadialArrayOperationController::captureSelectedSource(
    SourceSnapshot& theSource) const noexcept
{
    if (_context.IsNull() || _document.IsNull()) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(TDocStd_Document) aDocument = _document->Document();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()
            || aDocument->GetData().IsNull()) {
            return Standard_False;
        }
        _context->InitSelected();
        if (!_context->MoreSelected()) {
            return Standard_False;
        }
        const Handle(AIS_InteractiveObject) anInteractive =
            _context->SelectedInteractive();
        const Handle(AIS_Shape) aPresentation =
            Handle(AIS_Shape)::DownCast(anInteractive);
        const Handle(StdSelect_BRepOwner) anOwner =
            Handle(StdSelect_BRepOwner)::DownCast(
                _context->SelectedOwner());
        _context->NextSelected();
        if (_context->MoreSelected() || aPresentation.IsNull()
            || aPresentation->Shape().IsNull() || anOwner.IsNull()
            || !anOwner->HasShape()
            || !anOwner->Shape().IsEqual(aPresentation->Shape())
            || !_context->IsDisplayed(aPresentation)
            || !_document->IsPresentationEditable(aPresentation)) {
            return Standard_False;
        }
        const TDF_Label aLabel = _document->ShapeLabel(aPresentation);
        const OcctGeometryRepresentation aRepresentation =
            _document->GeometryRepresentationForLabel(aLabel);
        const OcctGeometryRepresentation aDestinationRepresentation =
            DestinationRepresentation(aRepresentation);
        const TopoDS_Shape aStored = aLabel.IsNull()
            ? TopoDS_Shape()
            : XCAFDoc_ShapeTool::GetShape(aLabel);
        const gp_Trsf aTransform = aPresentation->LocalTransformation();
        Quantity_Color aColor;
        Graphic3d_NameOfMaterial aMaterialName =
            Graphic3d_NameOfMaterial_ShinyPlastified;
        Standard_Real aTransparency = 0.0;
        Standard_Boolean hasStrictOverlayStyle = Standard_False;
        if (aPresentation->HasColor()
            && aPresentation->HasMaterial()) {
            aPresentation->Color(aColor);
            aMaterialName = aPresentation->Material();
            aTransparency = aPresentation->Transparency();
            hasStrictOverlayStyle = HasExactPresentationStyle(
                aPresentation,
                aMaterialName,
                aColor,
                aTransparency);
        }
        const std::string anEntityIdentifier =
            _document->EntityIdentifierForLabel(aLabel);
        const std::string aDefinitionIdentifier =
            _document->DefinitionIdentifierForLabel(aLabel);
        Standard_Size aTopologyNodeCount = 0;
        Standard_Real aMetersPerUnit = kLegacyMetersPerUnit;
        OcctReferenceAxis aReferenceAxis;
        const OcctReferenceAxisReadState aReferenceAxisState =
            _document->ReadReferenceAxisForLabel(aLabel, aReferenceAxis);
        gp_Ax1 aWorldAxis;
        Standard_Real aMinimum[3] = {0.0, 0.0, 0.0};
        Standard_Real aMaximum[3] = {0.0, 0.0, 0.0};
        if (aLabel.IsNull()
            || aLabel.Data() != aDocument->GetData()
            || !_document->IsEditableFreeSimpleDefinitionLabel(aLabel)
            || !IsSupportedRepresentation(aRepresentation)
            || (aDestinationRepresentation
                    != OcctGeometryRepresentation::BRep
                && aDestinationRepresentation
                    != OcctGeometryRepresentation::TriangleMesh)
            || aStored.IsNull()
            || !aStored.IsEqual(aPresentation->Shape())
            || TransformDiffers(aPresentation->Transformation(), aTransform)
            || TransformDiffers(
                _document->ObjectTransformForLabel(aLabel), aTransform)
            || anEntityIdentifier.empty()
            || aDefinitionIdentifier.empty()
            || aReferenceAxisState
                == OcctReferenceAxisReadState::Invalid
            || !_document->ResolveReferenceAxisInWorld(
                aLabel, TopLoc_Location(), aWorldAxis)
            || !TryReadMetersPerUnit(aDocument, aMetersPerUnit)
            || HasStyledSubshape(
                aDocument, aLabel, kMaximumSourceTopologyNodes)
            || !CountBoundedTopology(
                aStored,
                kMaximumSourceTopologyNodes,
                aTopologyNodeCount)
            || (aDestinationRepresentation
                    == OcctGeometryRepresentation::BRep
                && !IsTopologicallyValid(aStored))
            || !WorldBounds(aStored, aTransform, aMinimum, aMaximum)
            || !HasBoundedPivotRadius(aMinimum, aMaximum, aWorldAxis)) {
            return Standard_False;
        }

        theSource = {};
        theSource.document = aDocument;
        theSource.presentation = aPresentation;
        theSource.label = aLabel;
        theSource.entityIdentifier = anEntityIdentifier;
        theSource.definitionIdentifier = aDefinitionIdentifier;
        theSource.representation = aRepresentation;
        theSource.destinationRepresentation = aDestinationRepresentation;
        theSource.storedShape = aStored;
        theSource.transform = aTransform;
        theSource.hasStrictOverlayStyle = hasStrictOverlayStyle;
        theSource.materialName = aMaterialName;
        theSource.color = aColor;
        theSource.transparency = aTransparency;
        theSource.referenceAxisState = aReferenceAxisState;
        theSource.referenceAxis = aReferenceAxis;
        theSource.worldAxis = aWorldAxis;
        theSource.documentTime = aDocument->GetData()->Time();
        theSource.topologyNodeCount = aTopologyNodeCount;
        theSource.metersPerUnit = aMetersPerUnit;
        for (Standard_Integer anAxis = 0; anAxis < 3; ++anAxis) {
            theSource.worldMinimum[anAxis] = aMinimum[anAxis];
            theSource.worldMaximum[anAxis] = aMaximum[anAxis];
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean RadialArrayOperationController::sourceIsCurrent(
    const Standard_Boolean theRequireOriginalDocumentTime) const noexcept
{
    return _source.has_value()
        && sourceIsCurrent(*_source, theRequireOriginalDocumentTime);
}

Standard_Boolean RadialArrayOperationController::sourceIsCurrent(
    const SourceSnapshot& theSource,
    const Standard_Boolean theRequireOriginalDocumentTime) const noexcept
{
    if (_context.IsNull() || _document.IsNull()
        || theSource.document.IsNull()
        || theSource.presentation.IsNull()
        || theSource.label.IsNull()) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(TDocStd_Document) aDocument = _document->Document();
        if (aDocument.IsNull() || aDocument != theSource.document
            || aDocument->HasOpenCommand() || aDocument->GetData().IsNull()
            || theSource.label.Data() != aDocument->GetData()
            || (theRequireOriginalDocumentTime
                && aDocument->GetData()->Time() != theSource.documentTime)) {
            return Standard_False;
        }
        const TopoDS_Shape aStored =
            XCAFDoc_ShapeTool::GetShape(theSource.label);
        Standard_Size aTopologyNodeCount = 0;
        Standard_Real aMetersPerUnit = kLegacyMetersPerUnit;
        OcctReferenceAxis aReferenceAxis;
        const OcctReferenceAxisReadState aReferenceAxisState =
            _document->ReadReferenceAxisForLabel(
                theSource.label, aReferenceAxis);
        gp_Ax1 aWorldAxis;
        Standard_Real aMinimum[3] = {0.0, 0.0, 0.0};
        Standard_Real aMaximum[3] = {0.0, 0.0, 0.0};
        if (aStored.IsNull() || theSource.storedShape.IsNull()
            || !aStored.IsEqual(theSource.storedShape)
            || theSource.presentation->Shape().IsNull()
            || !theSource.presentation->Shape().IsEqual(
                theSource.storedShape)
            || (theSource.hasStrictOverlayStyle
                && !HasExactPresentationStyle(
                    theSource.presentation,
                    theSource.materialName,
                    theSource.color,
                    theSource.transparency))
            || !_context->IsDisplayed(theSource.presentation)
            || !_document->IsPresentationEditable(theSource.presentation)
            || !_document->ShapeLabel(theSource.presentation).IsEqual(
                theSource.label)
            || !_document->IsEditableFreeSimpleDefinitionLabel(
                theSource.label)
            || _document->EntityIdentifierForLabel(theSource.label)
                != theSource.entityIdentifier
            || _document->DefinitionIdentifierForLabel(theSource.label)
                != theSource.definitionIdentifier
            || _document->GeometryRepresentationForLabel(theSource.label)
                != theSource.representation
            || DestinationRepresentation(theSource.representation)
                != theSource.destinationRepresentation
            || aReferenceAxisState != theSource.referenceAxisState
            || aReferenceAxisState
                == OcctReferenceAxisReadState::Invalid
            || ReferenceAxisDiffers(aReferenceAxis, theSource.referenceAxis)
            || !_document->ResolveReferenceAxisInWorld(
                theSource.label, TopLoc_Location(), aWorldAxis)
            || WorldAxisDiffers(aWorldAxis, theSource.worldAxis)
            || !TryReadMetersPerUnit(aDocument, aMetersPerUnit)
            || aMetersPerUnit != theSource.metersPerUnit
            || TransformDiffers(
                theSource.presentation->LocalTransformation(),
                theSource.transform)
            || TransformDiffers(
                theSource.presentation->Transformation(),
                theSource.transform)
            || TransformDiffers(
                _document->ObjectTransformForLabel(theSource.label),
                theSource.transform)
            || HasStyledSubshape(
                aDocument,
                theSource.label,
                kMaximumSourceTopologyNodes)
            || !CountBoundedTopology(
                aStored,
                kMaximumSourceTopologyNodes,
                aTopologyNodeCount)
            || aTopologyNodeCount != theSource.topologyNodeCount
            || (theSource.destinationRepresentation
                    == OcctGeometryRepresentation::BRep
                && !IsTopologicallyValid(aStored))
            || !WorldBounds(
                aStored, theSource.transform, aMinimum, aMaximum)
            || !HasBoundedPivotRadius(aMinimum, aMaximum, aWorldAxis)) {
            return Standard_False;
        }
        for (Standard_Integer anAxis = 0; anAxis < 3; ++anAxis) {
            if (std::abs(aMinimum[anAxis]
                    - theSource.worldMinimum[anAxis])
                    > Precision::Confusion()
                || std::abs(aMaximum[anAxis]
                    - theSource.worldMaximum[anAxis])
                    > Precision::Confusion()) {
                return Standard_False;
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean RadialArrayOperationController::canAdmitCount(
    const SourceSnapshot& theSource,
    const Standard_Integer theCount) const noexcept
{
    if (_document.IsNull()
        || theCount < kMinimumCount || theCount > kMaximumCount
        || theSource.label.IsNull() || theSource.topologyNodeCount == 0) {
        return Standard_False;
    }
    const Standard_Size aCount = static_cast<Standard_Size>(theCount);
#ifdef DEBUG
    const Standard_Size anAggregateLimit = std::max<Standard_Size>(
        1,
        std::min(
            _debugMaximumTopologyNodes,
            kMaximumAggregateTopologyNodes));
#else
    const Standard_Size anAggregateLimit =
        kMaximumAggregateTopologyNodes;
#endif
    if (theSource.topologyNodeCount
        > anAggregateLimit / aCount) {
        return Standard_False;
    }
    try {
        const std::vector<OcctGeometryDuplicationRequest> aRequests = {{
            theSource.label,
            static_cast<Standard_Size>(theCount - 1),
        }};
        return _document->CanDuplicateGeometryDefinitions(aRequests);
    } catch (...) {
        return Standard_False;
    }
}

Standard_Integer RadialArrayOperationController::maximumAdmittedCount(
    const SourceSnapshot& theSource) const noexcept
{
    Standard_Integer aLow = kMinimumCount;
    Standard_Integer aHigh = kMaximumCount;
    Standard_Integer aMaximum = kMinimumCount - 1;
    while (aLow <= aHigh) {
        const Standard_Integer aCandidate = aLow + (aHigh - aLow) / 2;
        if (canAdmitCount(theSource, aCandidate)) {
            aMaximum = aCandidate;
            aLow = aCandidate + 1;
        } else {
            aHigh = aCandidate - 1;
        }
    }
    return aMaximum;
}

Standard_Boolean RadialArrayOperationController::begin() noexcept
{
    if (hasActiveOperation() && !cancel()) {
        return Standard_False;
    }
    SourceSnapshot aSource;
    if (!captureSelectedSource(aSource)) {
        return Standard_False;
    }
    const Standard_Integer aMaximum = maximumAdmittedCount(aSource);
    if (aMaximum < kMinimumCount) {
        return Standard_False;
    }
    if (!advanceReferenceAuthorityToken()) {
        return Standard_False;
    }

    _source = std::move(aSource);
    _count = aMaximum >= kDefaultCount ? kDefaultCount : kMinimumCount;
    _maximumAdmittedCount = aMaximum;
    _sweepDegrees = kDefaultSweepDegrees;
    _previewValid = Standard_False;
    clearCommandOwnership();
    _pendingResults.clear();
    _pendingReferenceEdit.reset();
    _state = RadialArrayPreviewState::Selecting;
    ++_generation;
    if (!rebuildPreview()) {
        _state = RadialArrayPreviewState::Failed;
        ++_generation;
        notifyStateChanged();
        return Standard_False;
    }
    _state = RadialArrayPreviewState::Ready;
    ++_generation;
    notifyStateChanged();
    return Standard_True;
}

Standard_Boolean RadialArrayOperationController::rebuildPreview() noexcept
{
    if (!_source.has_value()
        || IsNearZeroStep(_sweepDegrees, _count)
        || _count < kMinimumCount || _count > _maximumAdmittedCount
        || !sourceIsCurrent(Standard_True)
        || !canAdmitCount(*_source, _count)) {
        return Standard_False;
    }
    if (!_previewObjects.empty() && !_previewValid
        && !clearPreviewObjects()) {
        return Standard_False;
    }

    std::vector<Handle(AIS_Shape)> aReplacement;
    try {
        OCC_CATCH_SIGNALS
        aReplacement.reserve(static_cast<std::size_t>(_count - 1));
        for (Standard_Integer anOrdinal = 1;
             anOrdinal < _count; ++anOrdinal) {
            Handle(AIS_Shape) aPreview =
                new AIS_Shape(_source->storedShape);
            if (aPreview.IsNull() || aPreview->Shape().IsNull()
                || !aPreview->Shape().IsPartner(_source->storedShape)
                || !aPreview->Shape().IsEqual(_source->storedShape)) {
                return Standard_False;
            }
            const gp_Trsf aTransform = ArrayTransform(
                _source->transform,
                _source->worldAxis,
                anOrdinal,
                _sweepDegrees,
                _count);
            aPreview->SetLocalTransformation(aTransform);
            Standard_Real aMinimum[3] = {0.0, 0.0, 0.0};
            Standard_Real aMaximum[3] = {0.0, 0.0, 0.0};
            if (TransformDiffers(
                    aPreview->Transformation(), aTransform)
                || !WorldBounds(
                    aPreview->Shape(),
                    aPreview->Transformation(),
                    aMinimum,
                    aMaximum)) {
                return Standard_False;
            }
            _document->LoadObjectMeterial(_source->label, aPreview);
            if (_source->hasStrictOverlayStyle) {
                // Plain AIS_Shape previews do not necessarily inherit
                // CafShapePrs' effective appearance. Copy it only when it is
                // representable by the strict Metal overlay. Other authored
                // appearances remain valid through the OCCT fallback.
                aPreview->SetMaterial(_source->materialName);
                aPreview->SetColor(_source->color);
                aPreview->SetTransparency(_source->transparency);
                if (!HasExactPresentationStyle(
                        aPreview,
                        _source->materialName,
                        _source->color,
                        _source->transparency)) {
                    return Standard_False;
                }
            }
            aReplacement.push_back(aPreview);
        }
    } catch (...) {
        return Standard_False;
    }
    if (aReplacement.empty()
        || aReplacement.size() > kMaximumPreviewBodies) {
        return Standard_False;
    }

    const std::vector<Handle(AIS_Shape)> aPrevious = _previewObjects;
    std::vector<Handle(AIS_Shape)> aTransition = aPrevious;
    try {
        aTransition.insert(
            aTransition.end(), aReplacement.begin(), aReplacement.end());
    } catch (...) {
        return Standard_False;
    }
    // Claim every old/new presentation before mutating the context. A partial
    // replacement therefore remains fully recoverable by clearPreviewObjects.
    _previewObjects = std::move(aTransition);
    _previewValid = Standard_False;
    try {
        for (const Handle(AIS_Shape)& aPreview : aReplacement) {
            _context->Display(aPreview, AIS_Shaded, 0, Standard_False);
            _context->Deactivate(aPreview);
            TColStd_ListOfInteger anActiveModes;
            _context->ActivatedModes(aPreview, anActiveModes);
            if (!_context->IsDisplayed(aPreview)
                || !anActiveModes.IsEmpty()) {
                return Standard_False;
            }
        }
        for (const Handle(AIS_Shape)& aPreview : aPrevious) {
#ifdef DEBUG
            if (_debugEraseFailureCount > 0) {
                --_debugEraseFailureCount;
            } else
#endif
            {
            _context->Erase(aPreview, Standard_False);
            }
            if (_context->IsDisplayed(aPreview)) {
                return Standard_False;
            }
        }
        _context->UpdateCurrentViewer();
    } catch (...) {
        return Standard_False;
    }
    _previewObjects = std::move(aReplacement);
    _previewValid = Standard_True;
    return Standard_True;
}

Standard_Boolean
RadialArrayOperationController::clearPreviewObjects() noexcept
{
    _previewValid = Standard_False;
    std::vector<Handle(AIS_Shape)> anUnresolved;
    try {
        anUnresolved.reserve(_previewObjects.size());
    } catch (...) {
        _state = RadialArrayPreviewState::Failed;
        return Standard_False;
    }
    for (const Handle(AIS_Shape)& aPreview : _previewObjects) {
        try {
#ifdef DEBUG
            if (_debugEraseFailureCount > 0) {
                --_debugEraseFailureCount;
                anUnresolved.push_back(aPreview);
                continue;
            }
#endif
            _context->Erase(aPreview, Standard_False);
            if (_context->IsDisplayed(aPreview)) {
                anUnresolved.push_back(aPreview);
            }
        } catch (...) {
            anUnresolved.push_back(aPreview);
        }
    }
    _previewObjects = std::move(anUnresolved);
    try {
        if (!_context.IsNull()) {
            _context->UpdateCurrentViewer();
        }
    } catch (...) {
    }
    if (!_previewObjects.empty()) {
        _state = RadialArrayPreviewState::Failed;
        ++_generation;
        return Standard_False;
    }
    return Standard_True;
}

Standard_Boolean RadialArrayOperationController::setCount(
    const Standard_Integer theCount) noexcept
{
    if (!_source.has_value()
        || theCount < kMinimumCount
        || theCount > _maximumAdmittedCount
        || _state == RadialArrayPreviewState::Committing
        || _state == RadialArrayPreviewState::OutcomeUnknown
        || _ownsDocumentCommand || !_pendingResults.empty()
        || _pendingReferenceEdit.has_value()
        || !sourceIsCurrent(Standard_True)
        || !canAdmitCount(*_source, theCount)) {
        return Standard_False;
    }
    if (_count == theCount
        && (_state == RadialArrayPreviewState::Ready
            || (_state == RadialArrayPreviewState::Selecting
                && IsNearZeroStep(_sweepDegrees, _count)))) {
        return Standard_True;
    }
    _count = theCount;
    if (IsNearZeroStep(_sweepDegrees, _count)) {
        if (!clearPreviewObjects()) {
            _state = RadialArrayPreviewState::Failed;
            notifyStateChanged();
            return Standard_False;
        }
        _state = RadialArrayPreviewState::Selecting;
        ++_generation;
        notifyStateChanged();
        return Standard_True;
    }
    if (!rebuildPreview()) {
        _state = RadialArrayPreviewState::Failed;
        ++_generation;
        notifyStateChanged();
        return Standard_False;
    }
    _state = RadialArrayPreviewState::Ready;
    ++_generation;
    notifyStateChanged();
    return Standard_True;
}

Standard_Boolean RadialArrayOperationController::setSweepDegrees(
    const Standard_Real theSweepDegrees) noexcept
{
    if (!_source.has_value() || !std::isfinite(theSweepDegrees)
        || theSweepDegrees < kMinimumSweepDegrees
        || theSweepDegrees > kMaximumSweepDegrees
        || _state == RadialArrayPreviewState::Committing
        || _state == RadialArrayPreviewState::OutcomeUnknown
        || _ownsDocumentCommand || !_pendingResults.empty()
        || _pendingReferenceEdit.has_value()
        || !sourceIsCurrent(Standard_True)) {
        return Standard_False;
    }
    const Standard_Boolean isSame =
        std::abs(_sweepDegrees - theSweepDegrees)
            <= Precision::Angular() * 180.0 / kPi;
    if (isSame
        && (_state == RadialArrayPreviewState::Ready
            || (_state == RadialArrayPreviewState::Selecting
                && IsNearZeroStep(_sweepDegrees, _count)))) {
        // Preserve the exact parameter used to build the retained preview.
        // A tolerance-equivalent UI resend must not silently desynchronize
        // preview capture from Apply's transform math.
        return Standard_True;
    }
    _sweepDegrees = theSweepDegrees;
    if (IsNearZeroStep(_sweepDegrees, _count)) {
        if (!clearPreviewObjects()) {
            _state = RadialArrayPreviewState::Failed;
            notifyStateChanged();
            return Standard_False;
        }
        _state = RadialArrayPreviewState::Selecting;
        ++_generation;
        notifyStateChanged();
        return Standard_True;
    }
    if (!rebuildPreview()) {
        _state = RadialArrayPreviewState::Failed;
        ++_generation;
        notifyStateChanged();
        return Standard_False;
    }
    _state = RadialArrayPreviewState::Ready;
    ++_generation;
    notifyStateChanged();
    return Standard_True;
}

Standard_Integer RadialArrayOperationController::count() const noexcept
{
    return _count;
}

Standard_Real RadialArrayOperationController::sweepDegrees() const noexcept
{
    return _sweepDegrees;
}

Standard_Real RadialArrayOperationController::metersPerUnit() const noexcept
{
    return _source.has_value() ? _source->metersPerUnit : 0.0;
}

std::pair<Standard_Integer, Standard_Integer>
RadialArrayOperationController::countRange() const noexcept
{
    return {
        kMinimumCount,
        std::max(kMinimumCount, _maximumAdmittedCount),
    };
}

std::pair<Standard_Real, Standard_Real>
RadialArrayOperationController::sweepDegreesRange() const noexcept
{
    return {kMinimumSweepDegrees, kMaximumSweepDegrees};
}

OcctReferenceAxisReadState RadialArrayOperationController::referenceAxis(
    OcctReferenceAxis& theAxis) const noexcept
{
    if (!_source.has_value()
        || _referenceAuthorityToken == 0
        || _pendingReferenceEdit.has_value()
        || !sourceIsCurrent(Standard_True)) {
        return OcctReferenceAxisReadState::Invalid;
    }
    theAxis = _source->referenceAxis;
    return _source->referenceAxisState;
}

std::uint64_t
RadialArrayOperationController::referenceAuthorityToken() const noexcept
{
    return _source.has_value() && !_pendingReferenceEdit.has_value()
        && sourceIsCurrent(Standard_True)
        ? _referenceAuthorityToken
        : 0;
}

Standard_Boolean
RadialArrayOperationController::convertReferenceAxisSpaces(
    const OcctReferenceSpace thePivotSpace,
    const OcctReferenceSpace theDirectionSpace,
    const std::uint64_t theExpectedAuthorityToken,
    OcctReferenceAxis& theAxis) const noexcept
{
    if (!_source.has_value()
        || (thePivotSpace != OcctReferenceSpace::Object
            && thePivotSpace != OcctReferenceSpace::World)
        || (theDirectionSpace != OcctReferenceSpace::Object
            && theDirectionSpace != OcctReferenceSpace::World)
        || theExpectedAuthorityToken == 0
        || theExpectedAuthorityToken != _referenceAuthorityToken
        || _pendingReferenceEdit.has_value()
        || !sourceIsCurrent(Standard_True)) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        gp_Trsf anInverse;
        const Standard_Boolean needsInverse =
            (thePivotSpace == OcctReferenceSpace::Object
                && thePivotSpace
                    != _source->referenceAxis.pivotSpace)
            || (theDirectionSpace == OcctReferenceSpace::Object
                && theDirectionSpace
                    != _source->referenceAxis.directionSpace);
        if (needsInverse) {
            anInverse = _source->transform.Inverted();
        }
        OcctReferenceAxis aConverted;
        aConverted.pivotSpace = thePivotSpace;
        aConverted.directionSpace = theDirectionSpace;
        aConverted.pivot =
            thePivotSpace == _source->referenceAxis.pivotSpace
            ? _source->referenceAxis.pivot
            : thePivotSpace == OcctReferenceSpace::World
                ? _source->worldAxis.Location()
                : _source->worldAxis.Location().Transformed(anInverse);
        aConverted.direction =
            theDirectionSpace == _source->referenceAxis.directionSpace
            ? _source->referenceAxis.direction
            : theDirectionSpace == OcctReferenceSpace::World
                ? _source->worldAxis.Direction()
                : _source->worldAxis.Direction().Transformed(anInverse);
        const std::array<Standard_Real, 3> aPivot = {{
            aConverted.pivot.X(),
            aConverted.pivot.Y(),
            aConverted.pivot.Z(),
        }};
        for (const Standard_Real aValue : aPivot) {
            if (!std::isfinite(aValue)
                || std::abs(aValue)
                    > limits::kMaximumModelCoordinateMagnitude) {
                return Standard_False;
            }
        }
        // gp_Dir::Transformed normalizes after the inverse uniform scale.
        const std::array<Standard_Real, 3> aDirection = {{
            aConverted.direction.X(),
            aConverted.direction.Y(),
            aConverted.direction.Z(),
        }};
        for (const Standard_Real aValue : aDirection) {
            if (!std::isfinite(aValue)) {
                return Standard_False;
            }
        }
        theAxis = aConverted;
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

RadialArrayPreviewState
RadialArrayOperationController::previewState() const noexcept
{
    return _state;
}

std::uint64_t
RadialArrayOperationController::previewGeneration() const noexcept
{
    return _generation;
}

Standard_Boolean
RadialArrayOperationController::hasUnresolvedState() const noexcept
{
    return !_previewObjects.empty() || !_pendingResults.empty()
        || _pendingReferenceEdit.has_value() || _ownsDocumentCommand;
}

Standard_Boolean
RadialArrayOperationController::hasActiveOperation() const noexcept
{
    return _state != RadialArrayPreviewState::Unavailable
        || _source.has_value() || hasUnresolvedState();
}

Standard_Boolean RadialArrayOperationController::canApply() const noexcept
{
    return (_state == RadialArrayPreviewState::Ready
            && _previewValid && _source.has_value()
            && !_pendingReferenceEdit.has_value()
            && !IsNearZeroStep(_sweepDegrees, _count)
            && _previewObjects.size()
                == static_cast<std::size_t>(_count - 1))
        || (_state == RadialArrayPreviewState::OutcomeUnknown
            && !_pendingReferenceEdit.has_value()
            && (_ownsDocumentCommand || !_pendingResults.empty()));
}

Standard_Boolean RadialArrayOperationController::capturePreview(
    RadialArrayPreviewCapture& theCapture) const noexcept
{
    theCapture = {};
    if (_state != RadialArrayPreviewState::Ready || !_previewValid
        || !_source.has_value() || !sourceIsCurrent(Standard_True)
        || _referenceAuthorityToken == 0
        || _pendingReferenceEdit.has_value()
        || HasUnsupportedOverlayTexture(_source->label)
        || _previewObjects.empty()
        || _previewObjects.size()
            != static_cast<std::size_t>(_count - 1)
        || _previewObjects.size() > kMaximumPreviewBodies) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        if (_source->presentation.IsNull()
            || !_context->IsDisplayed(_source->presentation)
            || TransformDiffers(
                _source->presentation->Transformation(),
                _source->transform)) {
            return Standard_False;
        }
        for (std::size_t anIndex = 0;
             anIndex < _previewObjects.size(); ++anIndex) {
            const Handle(AIS_Shape)& aPreview = _previewObjects[anIndex];
            TColStd_ListOfInteger anActiveModes;
            if (aPreview.IsNull() || aPreview->Shape().IsNull()
                || !aPreview->Shape().IsEqual(_source->storedShape)
                || !_context->IsDisplayed(aPreview)
                || TransformDiffers(
                    aPreview->Transformation(),
                    ArrayTransform(
                        _source->transform,
                        _source->worldAxis,
                        static_cast<Standard_Integer>(anIndex + 1U),
                        _sweepDegrees,
                        _count))) {
                return Standard_False;
            }
            _context->ActivatedModes(aPreview, anActiveModes);
            if (!anActiveModes.IsEmpty()) {
                return Standard_False;
            }
        }
        theCapture.sourcePresentation = _source->presentation;
        theCapture.previewObjects = _previewObjects;
        return Standard_True;
    } catch (...) {
        theCapture = {};
        return Standard_False;
    }
}

Standard_Boolean
RadialArrayOperationController::canPublishEmptyPreview() const noexcept
{
    return _state == RadialArrayPreviewState::Selecting
        && !_previewValid
        && _source.has_value()
        && IsNearZeroStep(_sweepDegrees, _count)
        && _previewObjects.empty()
        && _pendingResults.empty()
        && !_pendingReferenceEdit.has_value()
        && !_ownsDocumentCommand
        && sourceIsCurrent(Standard_True)
        && !HasUnsupportedOverlayTexture(_source->label);
}

void RadialArrayOperationController::clearCommandOwnership() noexcept
{
    _ownsDocumentCommand = Standard_False;
    _ownedTransaction = -1;
    _ownedDocumentTime = -1;
    _ownedMarkerValue = 0;
}

Standard_Boolean RadialArrayOperationController::ownedCommandIsCurrent(
    const Handle(TDocStd_Document)& theDocument) const noexcept
{
    try {
        const Handle(TDF_Data) aData = theDocument.IsNull()
            ? Handle(TDF_Data)() : theDocument->GetData();
        Handle(TDataStd_Integer) aMarker;
        return _ownsDocumentCommand
            && !theDocument.IsNull()
            && theDocument->HasOpenCommand()
            && !aData.IsNull()
            && _ownedTransaction > 0
            && aData->Transaction() == _ownedTransaction
            && aData->Time() == _ownedDocumentTime
            && theDocument->Main().FindAttribute(
                Core3DRadialArrayCommandOwnerAttributeID(), aMarker)
            && !aMarker.IsNull()
            && aMarker->Get() == _ownedMarkerValue
            ? Standard_True : Standard_False;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean RadialArrayOperationController::beginOwnedCommand(
    const Handle(TDocStd_Document)& theDocument) noexcept
{
    if (theDocument.IsNull() || theDocument->HasOpenCommand()
        || _ownsDocumentCommand) {
        return Standard_False;
    }
    Standard_Boolean anOpenedHere = Standard_False;
    const auto abortUnprovenCommand = [&]() noexcept {
        if (anOpenedHere) {
            try {
                if (theDocument->HasOpenCommand()) {
                    // This is the narrow synchronous interval after our own
                    // NewCommand and before a sentinel could be proven. No
                    // other caller can have acquired this command.
                    theDocument->AbortCommand();
                }
            } catch (...) {
            }
        }
        clearCommandOwnership();
    };
    try {
        OCC_CATCH_SIGNALS
        Handle(TDF_Attribute) aPriorAttribute;
        const Standard_Boolean hasPrior =
            theDocument->Main().FindAttribute(
                Core3DRadialArrayCommandOwnerAttributeID(),
                aPriorAttribute);
        const Handle(TDataStd_Integer) aPriorMarker =
            Handle(TDataStd_Integer)::DownCast(aPriorAttribute);
        if (hasPrior && aPriorMarker.IsNull()) {
            return Standard_False;
        }
        const Standard_Integer aPriorValue =
            aPriorMarker.IsNull() ? 0 : aPriorMarker->Get();
        const Standard_Integer aMarkerValue =
            aPriorValue == std::numeric_limits<Standard_Integer>::max()
            ? std::numeric_limits<Standard_Integer>::min()
            : aPriorValue + 1;

        theDocument->NewCommand();
        if (!theDocument->HasOpenCommand()) {
            clearCommandOwnership();
            return Standard_False;
        }
        anOpenedHere = Standard_True;
        const Handle(TDF_Data) aData = theDocument->GetData();
        if (aData.IsNull() || aData->Transaction() <= 0) {
            abortUnprovenCommand();
            return Standard_False;
        }
        _ownedTransaction = aData->Transaction();
        _ownedDocumentTime = aData->Time();
        _ownedMarkerValue = aMarkerValue;
        TDataStd_Integer::Set(
            theDocument->Main(),
            Core3DRadialArrayCommandOwnerAttributeID(),
            aMarkerValue);
        _ownsDocumentCommand = Standard_True;
#ifdef DEBUG
        if (_debugBeginOwnedCommandMismatchCount > 0) {
            --_debugBeginOwnedCommandMismatchCount;
            ++_ownedTransaction;
        }
#endif
        if (!ownedCommandIsCurrent(theDocument)) {
            // Ownership was never proven. Direct abort is still limited to
            // the synchronous command opened above.
            _ownsDocumentCommand = Standard_False;
            abortUnprovenCommand();
            return Standard_False;
        }
        return Standard_True;
    } catch (...) {
        if (_ownsDocumentCommand
            && ownedCommandIsCurrent(theDocument)) {
            (void)abortOwnedCommand();
        } else {
            abortUnprovenCommand();
        }
        return Standard_False;
    }
}

Standard_Boolean
RadialArrayOperationController::abortOwnedCommand() noexcept
{
    if (!_ownsDocumentCommand || _document.IsNull()) {
        return Standard_False;
    }
    try {
        const Handle(TDocStd_Document) aDocument =
            _document->ChangeDocument();
        if (aDocument.IsNull()
            || !ownedCommandIsCurrent(aDocument)) {
            return Standard_False;
        }
#ifdef DEBUG
        if (_debugAbortFailureCount > 0) {
            --_debugAbortFailureCount;
            return Standard_False;
        }
#endif
        aDocument->AbortCommand();
        if (aDocument->HasOpenCommand()) {
            return Standard_False;
        }
        clearCommandOwnership();
        return Standard_True;
    } catch (...) {
        // A throwing OCCT close may still have succeeded. Drop ownership as
        // soon as closure is observed so a future command is never at risk.
        try {
            const Handle(TDocStd_Document) aDocument =
                _document.IsNull()
                ? Handle(TDocStd_Document)()
                : _document->ChangeDocument();
            if (!aDocument.IsNull() && !aDocument->HasOpenCommand()) {
                clearCommandOwnership();
                return Standard_True;
            }
        } catch (...) {
        }
        return Standard_False;
    }
}

RadialArrayOperationController::DocumentState
RadialArrayOperationController::inspectPendingResults() noexcept
{
#ifdef DEBUG
    if (_debugPostCommitInspectMode != 0) {
        const Standard_Integer aMode = _debugPostCommitInspectMode;
        _debugPostCommitInspectMode = 0;
        return aMode == 2
            ? DocumentState::PartialOrMismatched
            : DocumentState::Unavailable;
    }
#endif
    if (_document.IsNull()) {
        return DocumentState::Unavailable;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(TDocStd_Document) aDocument = _document->Document();
        if (aDocument.IsNull() || aDocument->GetData().IsNull()) {
            return DocumentState::Unavailable;
        }
        if (aDocument->HasOpenCommand()) {
            if (ownedCommandIsCurrent(aDocument)) {
                return DocumentState::OpenCommand;
            }
            clearCommandOwnership();
            return DocumentState::Unavailable;
        }
        clearCommandOwnership();
        if (_pendingResults.empty()) {
            return DocumentState::None;
        }
        Standard_Size aMissingCount = 0;
        Standard_Size aCommittedCount = 0;
        for (const PendingResult& aResult : _pendingResults) {
            if (aResult.label.IsNull()
                || aResult.label.Data() != aDocument->GetData()) {
                return DocumentState::PartialOrMismatched;
            }
            const TopoDS_Shape aStored =
                XCAFDoc_ShapeTool::GetShape(aResult.label);
            if (aStored.IsNull()) {
                ++aMissingCount;
                continue;
            }
            OcctReferenceAxis aReferenceAxis;
            const OcctReferenceAxisReadState aReferenceAxisState =
                _document->ReadReferenceAxisForLabel(
                    aResult.label, aReferenceAxis);
            if (aResult.expectedShape.IsNull()
                || !aStored.IsEqual(aResult.expectedShape)
                || aResult.entityIdentifier.empty()
                || aResult.definitionIdentifier.empty()
                || _document->EntityIdentifierForLabel(aResult.label)
                    != aResult.entityIdentifier
                || _document->DefinitionIdentifierForLabel(aResult.label)
                    != aResult.definitionIdentifier
                || _document->GeometryRepresentationForLabel(aResult.label)
                    != aResult.representation
                || !_document->IsEditableFreeSimpleDefinitionLabel(
                    aResult.label)
                || TransformDiffers(
                    _document->ObjectTransformForLabel(aResult.label),
                    aResult.expectedTransform)
                || aReferenceAxisState
                    != aResult.expectedReferenceAxisState
                || aReferenceAxisState
                    == OcctReferenceAxisReadState::Invalid
                || ReferenceAxisDiffers(
                    aReferenceAxis,
                    aResult.expectedReferenceAxis)) {
                return DocumentState::PartialOrMismatched;
            }
            ++aCommittedCount;
        }
        if (aCommittedCount == _pendingResults.size()) {
            return DocumentState::AllCommitted;
        }
        if (aMissingCount == _pendingResults.size()) {
            return DocumentState::None;
        }
        return DocumentState::PartialOrMismatched;
    } catch (...) {
        return DocumentState::Unavailable;
    }
}

RadialArrayOperationController::ReferenceDocumentState
RadialArrayOperationController::inspectPendingReferenceEdit() noexcept
{
    if (_document.IsNull()) {
        return ReferenceDocumentState::Unavailable;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(TDocStd_Document) aDocument = _document->Document();
        if (aDocument.IsNull() || aDocument->GetData().IsNull()) {
            return ReferenceDocumentState::Unavailable;
        }
        if (aDocument->HasOpenCommand()) {
            if (ownedCommandIsCurrent(aDocument)) {
                return ReferenceDocumentState::OpenCommand;
            }
            clearCommandOwnership();
            return ReferenceDocumentState::Unavailable;
        }
        clearCommandOwnership();
        if (!_pendingReferenceEdit.has_value()) {
            return ReferenceDocumentState::None;
        }
        if (!_source.has_value() || _source->label.IsNull()
            || _source->label.Data() != aDocument->GetData()) {
            return ReferenceDocumentState::Mismatched;
        }
        OcctReferenceAxis anAxis;
        const OcctReferenceAxisReadState aState =
            _document->ReadReferenceAxisForLabel(
                _source->label, anAxis);
        if (aState == OcctReferenceAxisReadState::Invalid) {
            return ReferenceDocumentState::Mismatched;
        }
        if (aState == _pendingReferenceEdit->expectedState
            && !ReferenceAxisDiffers(
                anAxis, _pendingReferenceEdit->expectedAxis)) {
            return ReferenceDocumentState::Expected;
        }
        if (aState == _pendingReferenceEdit->previousState
            && !ReferenceAxisDiffers(
                anAxis, _pendingReferenceEdit->previousAxis)) {
            return ReferenceDocumentState::Previous;
        }
        return ReferenceDocumentState::Mismatched;
    } catch (...) {
        return ReferenceDocumentState::Unavailable;
    }
}

Standard_Boolean
RadialArrayOperationController::refreshSourceAfterReferenceEdit() noexcept
{
    if (!_source.has_value()) {
        return Standard_False;
    }
    const SourceSnapshot aPrevious = *_source;
    SourceSnapshot aReplacement;
    if (!captureSelectedSource(aReplacement)
        || aReplacement.document != aPrevious.document
        || aReplacement.presentation != aPrevious.presentation
        || !aReplacement.label.IsEqual(aPrevious.label)
        || aReplacement.entityIdentifier != aPrevious.entityIdentifier
        || aReplacement.definitionIdentifier
            != aPrevious.definitionIdentifier
        || aReplacement.representation != aPrevious.representation
        || aReplacement.destinationRepresentation
            != aPrevious.destinationRepresentation
        || aReplacement.storedShape.IsNull()
        || !aReplacement.storedShape.IsEqual(aPrevious.storedShape)
        || TransformDiffers(aReplacement.transform, aPrevious.transform)
        || aReplacement.topologyNodeCount
            != aPrevious.topologyNodeCount
        || aReplacement.metersPerUnit != aPrevious.metersPerUnit) {
        return Standard_False;
    }
    const Standard_Integer aMaximum = maximumAdmittedCount(aReplacement);
    if (aMaximum < _count || !canAdmitCount(aReplacement, _count)) {
        return Standard_False;
    }
    _source = std::move(aReplacement);
    _maximumAdmittedCount = aMaximum;
    if (!advanceReferenceAuthorityToken()) {
        // The edit/rollback outcome is known, but no globally unique lease can
        // be issued. Invalidate mutation/capture authority and fail closed
        // without retaining an OutcomeUnknown document ledger forever.
        (void)clearPreviewObjects();
        _previewValid = Standard_False;
        _state = RadialArrayPreviewState::Failed;
        ++_generation;
        notifyStateChanged();
        return Standard_True;
    }
    if (IsNearZeroStep(_sweepDegrees, _count)) {
        if (!clearPreviewObjects()) {
            _state = RadialArrayPreviewState::Failed;
        } else {
            _state = RadialArrayPreviewState::Selecting;
        }
    } else if (!rebuildPreview()) {
        _state = RadialArrayPreviewState::Failed;
    } else {
        _state = RadialArrayPreviewState::Ready;
    }
    ++_generation;
    notifyStateChanged();
    return Standard_True;
}

Standard_Boolean
RadialArrayOperationController::axisAdmitsCurrentPreview(
    const gp_Ax1& theWorldAxis) const noexcept
{
    if (!_source.has_value()
        || _count < kMinimumCount || _count > _maximumAdmittedCount
        || !HasBoundedPivotRadius(
            _source->worldMinimum,
            _source->worldMaximum,
            theWorldAxis)) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        for (Standard_Integer anOrdinal = 1;
             anOrdinal < _count; ++anOrdinal) {
            const gp_Trsf aTransform = ArrayTransform(
                _source->transform,
                theWorldAxis,
                anOrdinal,
                _sweepDegrees,
                _count);
            Standard_Real aMinimum[3] = {0.0, 0.0, 0.0};
            Standard_Real aMaximum[3] = {0.0, 0.0, 0.0};
            if (!WorldBounds(
                    _source->storedShape,
                    aTransform,
                    aMinimum,
                    aMaximum)) {
                return Standard_False;
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

RadialArrayReferenceEditResult
RadialArrayOperationController::reconcileReferenceEdit() noexcept
{
    ReferenceDocumentState aState = inspectPendingReferenceEdit();
    if (aState == ReferenceDocumentState::OpenCommand
        && abortOwnedCommand()) {
        aState = inspectPendingReferenceEdit();
    }
    if (aState == ReferenceDocumentState::Expected) {
        const PendingReferenceEdit anEdit = *_pendingReferenceEdit;
        _pendingReferenceEdit.reset();
        if (!refreshSourceAfterReferenceEdit()) {
            _pendingReferenceEdit = anEdit;
            _state = RadialArrayPreviewState::OutcomeUnknown;
            ++_generation;
            notifyStateChanged();
            return RadialArrayReferenceEditResult::OutcomeUnknown;
        }
        try {
            _document->NotifyChanges();
        } catch (...) {
        }
        return RadialArrayReferenceEditResult::Applied;
    }
    if (aState == ReferenceDocumentState::Previous
        || aState == ReferenceDocumentState::None) {
        if (aState == ReferenceDocumentState::Previous) {
            const PendingReferenceEdit anEdit = *_pendingReferenceEdit;
            _pendingReferenceEdit.reset();
            if (!refreshSourceAfterReferenceEdit()) {
                _pendingReferenceEdit = anEdit;
                _state = RadialArrayPreviewState::OutcomeUnknown;
                ++_generation;
                notifyStateChanged();
                return RadialArrayReferenceEditResult::OutcomeUnknown;
            }
        }
        _pendingReferenceEdit.reset();
        if (_state == RadialArrayPreviewState::Committing
            || _state == RadialArrayPreviewState::OutcomeUnknown) {
            _state = _previewValid
                ? RadialArrayPreviewState::Ready
                : RadialArrayPreviewState::Failed;
            ++_generation;
            notifyStateChanged();
        }
        return RadialArrayReferenceEditResult::RetryableFailure;
    }
    _state = RadialArrayPreviewState::OutcomeUnknown;
    ++_generation;
    notifyStateChanged();
    return RadialArrayReferenceEditResult::OutcomeUnknown;
}

RadialArrayReferenceEditResult
RadialArrayOperationController::editReferenceAxis(
    const std::optional<OcctReferenceAxis>& theAxis,
    const std::uint64_t theExpectedAuthorityToken) noexcept
{
    if (_pendingReferenceEdit.has_value()) {
        return reconcileReferenceEdit();
    }
    if (!_source.has_value()
        || theExpectedAuthorityToken == 0
        || theExpectedAuthorityToken != _referenceAuthorityToken
        || _state == RadialArrayPreviewState::Committing
        || _state == RadialArrayPreviewState::OutcomeUnknown
        || _ownsDocumentCommand || !_pendingResults.empty()
        || !sourceIsCurrent(Standard_True)) {
        return RadialArrayReferenceEditResult::RetryableFailure;
    }
    if (theAxis.has_value()
        && _source->referenceAxisState
            == OcctReferenceAxisReadState::Authored
        && !ReferenceAxisDiffers(*theAxis, _source->referenceAxis)) {
        return RadialArrayReferenceEditResult::NoChange;
    }
    if (!theAxis.has_value()
        && _source->referenceAxisState
            == OcctReferenceAxisReadState::ImplicitDefault) {
        return RadialArrayReferenceEditResult::NoChange;
    }

    PendingReferenceEdit anEdit;
    anEdit.previousState = _source->referenceAxisState;
    anEdit.previousAxis = _source->referenceAxis;
    anEdit.expectedState = theAxis.has_value()
        ? OcctReferenceAxisReadState::Authored
        : OcctReferenceAxisReadState::ImplicitDefault;
    anEdit.expectedAxis = theAxis.has_value()
        ? *theAxis : OcctReferenceAxis{};
    _pendingReferenceEdit = anEdit;
    _state = RadialArrayPreviewState::Committing;
    ++_generation;
    notifyStateChanged();

    Handle(TDocStd_Document) aDocument;
    try {
        aDocument = _document->ChangeDocument();
    } catch (...) {
    }
    if (aDocument.IsNull() || aDocument != _source->document
        || !beginOwnedCommand(aDocument)) {
        return reconcileReferenceEdit();
    }
    try {
        OCC_CATCH_SIGNALS
        const Standard_Boolean didWrite = theAxis.has_value()
            ? _document->SetReferenceAxisForLabel(
                _source->label, *theAxis)
            : _document->ResetReferenceAxisForLabel(_source->label);
        OcctReferenceAxis aStoredAxis;
        const OcctReferenceAxisReadState aStoredState =
            _document->ReadReferenceAxisForLabel(
                _source->label, aStoredAxis);
        gp_Ax1 aStoredWorldAxis;
        if (!didWrite
            || aStoredState != anEdit.expectedState
            || ReferenceAxisDiffers(aStoredAxis, anEdit.expectedAxis)
            || !_document->ResolveReferenceAxisInWorld(
                _source->label,
                TopLoc_Location(),
                aStoredWorldAxis)
            || !axisAdmitsCurrentPreview(aStoredWorldAxis)
            || !_document->ValidateGeometryRepresentations()
            || !_document->ValidateReferenceAxes()) {
            (void)abortOwnedCommand();
            return reconcileReferenceEdit();
        }
        try {
            Standard_Boolean aCommitReported =
                aDocument->CommitCommand();
#ifdef DEBUG
            const Standard_Integer aCommitMode =
                _debugReferenceEditCommitMode;
            _debugReferenceEditCommitMode = 0;
            if (aCommitMode == 1) {
                aCommitReported = Standard_False;
            } else if (aCommitMode == 2) {
                throw Standard_Failure(
                    "Injected Radial reference commit exception after close");
            }
#endif
            (void)aCommitReported;
        } catch (...) {
            // Readback below is authoritative for false/throw-after-close.
        }
    } catch (...) {
        (void)abortOwnedCommand();
    }
    return reconcileReferenceEdit();
}

RadialArrayReferenceEditResult
RadialArrayOperationController::setReferenceAxis(
    const OcctReferenceAxis& theAxis,
    const std::uint64_t theExpectedAuthorityToken) noexcept
{
    return editReferenceAxis(theAxis, theExpectedAuthorityToken);
}

RadialArrayReferenceEditResult
RadialArrayOperationController::resetReferenceAxis(
    const std::uint64_t theExpectedAuthorityToken) noexcept
{
    return editReferenceAxis(std::nullopt, theExpectedAuthorityToken);
}

RadialArrayApplyResult
RadialArrayOperationController::finishCommittedOperation() noexcept
{
    if (inspectPendingResults() != DocumentState::AllCommitted) {
        return RadialArrayApplyResult::NoChange;
    }
    // Pending labels are the exactly-once proof until every owned preview has
    // been erased. A display failure must not permit a duplicate re-apply.
    if (!clearPreviewObjects()) {
        clearCommandOwnership();
        _state = RadialArrayPreviewState::OutcomeUnknown;
        ++_generation;
        notifyStateChanged();
        return RadialArrayApplyResult::NoChange;
    }
    try {
        _document->NotifyChanges();
    } catch (...) {
    }
    clearInactiveState();
    notifyStateChanged();
    return RadialArrayApplyResult::AppliedNeedsDocumentRedraw;
}

RadialArrayApplyResult RadialArrayOperationController::apply() noexcept
{
    if (!hasActiveOperation() || _pendingReferenceEdit.has_value()) {
        // Reference metadata has its own command and recovery ledger. Radial
        // Apply never folds it into geometry persistence or reconciles it.
        return RadialArrayApplyResult::NoChange;
    }
    if (_state == RadialArrayPreviewState::OutcomeUnknown
        || _ownsDocumentCommand || !_pendingResults.empty()) {
        DocumentState aState = inspectPendingResults();
        if (aState == DocumentState::OpenCommand
            && abortOwnedCommand()) {
            aState = inspectPendingResults();
        }
        if (aState == DocumentState::AllCommitted) {
            return finishCommittedOperation();
        }
        if (aState == DocumentState::None) {
            _pendingResults.clear();
            clearCommandOwnership();
            _state = _previewValid
                ? RadialArrayPreviewState::Ready
                : RadialArrayPreviewState::Failed;
            ++_generation;
            notifyStateChanged();
            return RadialArrayApplyResult::NoChange;
        }
        _state = RadialArrayPreviewState::OutcomeUnknown;
        ++_generation;
        notifyStateChanged();
        return RadialArrayApplyResult::NoChange;
    }
    if (_state != RadialArrayPreviewState::Ready || !canApply()
        || !sourceIsCurrent(Standard_True)
        || !canAdmitCount(*_source, _count)
        || IsNearZeroStep(_sweepDegrees, _count)) {
        _state = RadialArrayPreviewState::Failed;
        ++_generation;
        notifyStateChanged();
        return RadialArrayApplyResult::NoChange;
    }

    struct PreparedResult {
        Handle(AIS_Shape) presentation;
        gp_Trsf transform;
    };
    std::vector<PreparedResult> aPrepared;
    try {
        OCC_CATCH_SIGNALS
        aPrepared.reserve(static_cast<std::size_t>(_count - 1));
#ifdef DEBUG
        const Standard_Size anAggregateLimit = std::max<Standard_Size>(
            1,
            std::min(
                _debugMaximumTopologyNodes,
                kMaximumAggregateTopologyNodes));
#else
        const Standard_Size anAggregateLimit =
            kMaximumAggregateTopologyNodes;
#endif
        Standard_Size anAggregateTopology = _source->topologyNodeCount;
        for (Standard_Integer anOrdinal = 1;
             anOrdinal < _count; ++anOrdinal) {
            BRepBuilderAPI_Copy aCopy;
            aCopy.Perform(
                _source->storedShape,
                Standard_True,
                _source->destinationRepresentation
                    == OcctGeometryRepresentation::TriangleMesh
                    ? Standard_True : Standard_False);
            Standard_Size aResultTopology = 0;
            if (!aCopy.IsDone() || aCopy.Shape().IsNull()
                || aCopy.Shape().IsPartner(_source->storedShape)
                || (_source->destinationRepresentation
                        == OcctGeometryRepresentation::BRep
                    && !IsTopologicallyValid(aCopy.Shape()))
                || (_source->destinationRepresentation
                        == OcctGeometryRepresentation::TriangleMesh
                    && !RestoreTriangleMeshCopyParameters(
                        _source->storedShape, aCopy))
                || anAggregateTopology >= anAggregateLimit
                || !CountBoundedTopology(
                    aCopy.Shape(),
                    anAggregateLimit - anAggregateTopology,
                    aResultTopology)
                || aResultTopology
                    > anAggregateLimit
                        - anAggregateTopology) {
                return RadialArrayApplyResult::NoChange;
            }
            for (const PreparedResult& anExisting : aPrepared) {
                if (anExisting.presentation.IsNull()
                    || aCopy.Shape().IsPartner(
                        anExisting.presentation->Shape())) {
                    return RadialArrayApplyResult::NoChange;
                }
            }
            anAggregateTopology += aResultTopology;
            const gp_Trsf aTransform = ArrayTransform(
                _source->transform,
                _source->worldAxis,
                anOrdinal,
                _sweepDegrees,
                _count);
            Standard_Real aMinimum[3] = {0.0, 0.0, 0.0};
            Standard_Real aMaximum[3] = {0.0, 0.0, 0.0};
            if (!WorldBounds(
                    aCopy.Shape(), aTransform, aMinimum, aMaximum)) {
                return RadialArrayApplyResult::NoChange;
            }
            Handle(AIS_Shape) aPresentation =
                new AIS_Shape(aCopy.Shape());
            if (aPresentation.IsNull()) {
                return RadialArrayApplyResult::NoChange;
            }
            aPresentation->SetLocalTransformation(aTransform);
            if (TransformDiffers(
                    aPresentation->Transformation(), aTransform)) {
                return RadialArrayApplyResult::NoChange;
            }
            _document->LoadObjectMeterial(
                _source->label, aPresentation);
            aPrepared.push_back({aPresentation, aTransform});
        }
    } catch (...) {
        return RadialArrayApplyResult::NoChange;
    }
    if (aPrepared.size() != static_cast<std::size_t>(_count - 1)
        || !sourceIsCurrent(Standard_True)
        || !canAdmitCount(*_source, _count)) {
        return RadialArrayApplyResult::NoChange;
    }

    Handle(TDocStd_Document) aDocument;
    try {
        aDocument = _document->ChangeDocument();
    } catch (...) {
    }
    if (aDocument.IsNull() || aDocument != _source->document
        || aDocument->HasOpenCommand()) {
        return RadialArrayApplyResult::NoChange;
    }
    const auto retainRetryableOrUnknown = [this]() noexcept {
        if (_ownsDocumentCommand) {
            (void)abortOwnedCommand();
        }
        const DocumentState aState = inspectPendingResults();
        if (aState == DocumentState::None) {
            _pendingResults.clear();
            clearCommandOwnership();
            _state = _previewValid
                ? RadialArrayPreviewState::Ready
                : RadialArrayPreviewState::Failed;
        } else {
            _state = RadialArrayPreviewState::OutcomeUnknown;
        }
        ++_generation;
        notifyStateChanged();
        return RadialArrayApplyResult::NoChange;
    };

    _state = RadialArrayPreviewState::Committing;
    _pendingResults.clear();
    ++_generation;
    notifyStateChanged();
    if (!beginOwnedCommand(aDocument)) {
        _state = _previewValid
            ? RadialArrayPreviewState::Ready
            : RadialArrayPreviewState::Failed;
        ++_generation;
        notifyStateChanged();
        return RadialArrayApplyResult::NoChange;
    }
#ifdef DEBUG
    if (_debugTransactionFailureCount > 0) {
        --_debugTransactionFailureCount;
        return retainRetryableOrUnknown();
    }
#endif
    try {
        OCC_CATCH_SIGNALS
        _pendingResults.reserve(aPrepared.size());
        for (PreparedResult& aResult : aPrepared) {
            const TDF_Label aLabel = _document->AddShape(
                aResult.presentation,
                _source->destinationRepresentation);
            if (aLabel.IsNull()) {
                return retainRetryableOrUnknown();
            }
            _pendingResults.push_back({
                aLabel,
                _document->EntityIdentifierForLabel(aLabel),
                _document->DefinitionIdentifierForLabel(aLabel),
                _source->destinationRepresentation,
                aResult.presentation->Shape(),
                aResult.transform,
                _source->referenceAxisState,
                _source->referenceAxis,
            });
            PendingResult& aPending = _pendingResults.back();
            if (aPending.entityIdentifier.empty()
                || aPending.definitionIdentifier.empty()
                || _document->GeometryRepresentationForLabel(aLabel)
                    != _source->destinationRepresentation
                || TransformDiffers(
                    _document->ObjectTransformForLabel(aLabel),
                    aResult.transform)
                || !_document->CopyGeometryRepresentation(
                    _source->label, aLabel)
                || !_document->CopyObjectAppearance(
                    _source->label, aLabel)
                || !_document->CopyReferenceAxis(
                    _source->label, aLabel)) {
                return retainRetryableOrUnknown();
            }
            OcctReferenceAxis aStoredReferenceAxis;
            const OcctReferenceAxisReadState aStoredReferenceAxisState =
                _document->ReadReferenceAxisForLabel(
                    aLabel, aStoredReferenceAxis);
            if (aStoredReferenceAxisState
                    != aPending.expectedReferenceAxisState
                || aStoredReferenceAxisState
                    == OcctReferenceAxisReadState::Invalid
                || ReferenceAxisDiffers(
                    aStoredReferenceAxis,
                    aPending.expectedReferenceAxis)) {
                return retainRetryableOrUnknown();
            }
            _document->LoadObjectMeterial(
                aLabel, aResult.presentation);
        }
        if (!_document->ValidateGeometryRepresentations()
            || !_document->ValidateReferenceAxes()) {
            return retainRetryableOrUnknown();
        }
        try {
            Standard_Boolean aCommitReported =
                aDocument->CommitCommand();
#ifdef DEBUG
            const Standard_Integer aCommitMode =
                _debugApplyCommitMode;
            _debugApplyCommitMode = 0;
            if (aCommitMode == 1) {
                aCommitReported = Standard_False;
            } else if (aCommitMode == 2) {
                throw Standard_Failure(
                    "Injected Radial Array commit exception after close");
            }
#endif
            (void)aCommitReported;
        } catch (...) {
            // The command state and pending-label ledger are authoritative.
        }
    } catch (...) {
        return retainRetryableOrUnknown();
    }

    DocumentState aState = inspectPendingResults();
    if (aState == DocumentState::OpenCommand
        && abortOwnedCommand()) {
        aState = inspectPendingResults();
    }
    if (aState == DocumentState::AllCommitted) {
        return finishCommittedOperation();
    }
    if (aState == DocumentState::None) {
        _pendingResults.clear();
        clearCommandOwnership();
        _state = _previewValid
            ? RadialArrayPreviewState::Ready
            : RadialArrayPreviewState::Failed;
        ++_generation;
        notifyStateChanged();
        return RadialArrayApplyResult::NoChange;
    }
    _state = RadialArrayPreviewState::OutcomeUnknown;
    ++_generation;
    notifyStateChanged();
    return RadialArrayApplyResult::NoChange;
}

Standard_Boolean RadialArrayOperationController::cancel() noexcept
{
    if (!hasActiveOperation()) {
        return Standard_True;
    }
    if (_state == RadialArrayPreviewState::Committing
        || _state == RadialArrayPreviewState::OutcomeUnknown
        || _ownsDocumentCommand || !_pendingResults.empty()
        || _pendingReferenceEdit.has_value()) {
        _state = RadialArrayPreviewState::OutcomeUnknown;
        ++_generation;
        notifyStateChanged();
        return Standard_False;
    }
    if (!clearPreviewObjects()) {
        _state = RadialArrayPreviewState::Failed;
        notifyStateChanged();
        return Standard_False;
    }
    clearInactiveState();
    notifyStateChanged();
    return Standard_True;
}

void RadialArrayOperationController::clearInactiveState() noexcept
{
    _previewObjects.clear();
    _pendingResults.clear();
    _pendingReferenceEdit.reset();
    _source.reset();
    _count = kDefaultCount;
    _maximumAdmittedCount = kMaximumCount;
    _sweepDegrees = kDefaultSweepDegrees;
    _previewValid = Standard_False;
    clearCommandOwnership();
    _state = RadialArrayPreviewState::Unavailable;
    ++_generation;
}

#ifdef DEBUG
RadialArrayPreviewDebugState
RadialArrayOperationController::debugPreviewState() const noexcept
{
    RadialArrayPreviewDebugState aState;
    aState.state = _state;
    aState.generation = _generation;
    aState.referenceAuthorityToken = _referenceAuthorityToken;
    aState.previewBodyCount = _previewObjects.size();
    aState.pendingResultCount = _pendingResults.size();
    aState.hasPendingReferenceEdit =
        _pendingReferenceEdit.has_value();
    aState.activeOperation = hasActiveOperation();
    aState.previewValid = _previewValid;
    aState.canApply = canApply();
    aState.ownsDocumentCommand = _ownsDocumentCommand;
    aState.count = _count;
    aState.sweepDegrees = _sweepDegrees;
    try {
        const Handle(TDocStd_Document) aDocument = _document.IsNull()
            ? Handle(TDocStd_Document)()
            : _document->Document();
        aState.documentCommandOpen = !aDocument.IsNull()
            && aDocument->HasOpenCommand();
    } catch (...) {
        aState.documentCommandOpen = Standard_True;
    }
    return aState;
}

void RadialArrayOperationController::
debugSetBeginOwnedCommandMismatchCount(
    const Standard_Size theCount) noexcept
{
    _debugBeginOwnedCommandMismatchCount = theCount;
}

void RadialArrayOperationController::debugSetTransactionFailureCount(
    const Standard_Size theCount) noexcept
{
    _debugTransactionFailureCount = theCount;
}

void RadialArrayOperationController::debugSetAbortFailureCount(
    const Standard_Size theCount) noexcept
{
    _debugAbortFailureCount = theCount;
}

void RadialArrayOperationController::debugSetEraseFailureCount(
    const Standard_Size theCount) noexcept
{
    _debugEraseFailureCount = theCount;
}

void RadialArrayOperationController::debugSetApplyCommitMode(
    const Standard_Integer theMode) noexcept
{
    _debugApplyCommitMode = theMode >= 0 && theMode <= 2
        ? theMode : 0;
}

void RadialArrayOperationController::debugSetPostCommitInspectMode(
    const Standard_Integer theMode) noexcept
{
    _debugPostCommitInspectMode = theMode >= 0 && theMode <= 2
        ? theMode : 0;
}

void RadialArrayOperationController::debugSetMaximumTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    _debugMaximumTopologyNodes = std::max<Standard_Size>(
        1,
        std::min(theLimit, kMaximumAggregateTopologyNodes));
}

void RadialArrayOperationController::debugSetReferenceEditCommitMode(
    const Standard_Integer theMode) noexcept
{
    _debugReferenceEditCommitMode = theMode >= 0 && theMode <= 2
        ? theMode : 0;
}

Standard_Boolean RadialArrayOperationController::
debugMutateSourcePersistedTransform() noexcept
{
    if (!_source.has_value() || !sourceIsCurrent(Standard_True)
        || _document.IsNull()) {
        return Standard_False;
    }
    Handle(TDocStd_Document) aDocument;
    try {
        OCC_CATCH_SIGNALS
        aDocument = _document->ChangeDocument();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()
            || _source->label.IsNull()
            || _source->label.Data() != aDocument->GetData()) {
            return Standard_False;
        }
        const Standard_Real anOriginalX =
            _document->ObjectTransformForLabel(_source->label)
                .TranslationPart().X();
        if (!std::isfinite(anOriginalX)) {
            return Standard_False;
        }
        aDocument->NewCommand();
        if (!aDocument->HasOpenCommand()) {
            return Standard_False;
        }
        TDataStd_Real::Set(
            _source->label.FindChild(1), anOriginalX + 1.0);
        try {
            (void)aDocument->CommitCommand();
        } catch (...) {
        }
        if (aDocument->HasOpenCommand()) {
            aDocument->AbortCommand();
            return Standard_False;
        }
        return TransformDiffers(
                _document->ObjectTransformForLabel(_source->label),
                _source->transform)
            && !TransformDiffers(
                _source->presentation->LocalTransformation(),
                _source->transform);
    } catch (...) {
        try {
            if (!aDocument.IsNull() && aDocument->HasOpenCommand()) {
                aDocument->AbortCommand();
            }
        } catch (...) {
        }
        return Standard_False;
    }
}
#endif

} // namespace core3d
