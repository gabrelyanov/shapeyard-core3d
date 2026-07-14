//
//  LinearArrayOperationController.cpp
//  Core3D
//

#include "LinearArrayOperationController.hpp"

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
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Real.hxx>
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
#include <cmath>
#include <limits>
#include <utility>

namespace core3d {
namespace {

Standard_Integer AxisIndex(const LinearArrayAxis theAxis) noexcept
{
    switch (theAxis) {
        case LinearArrayAxis::X:
            return 0;
        case LinearArrayAxis::Y:
            return 1;
        case LinearArrayAxis::Z:
            return 2;
    }
    return -1;
}

OcctGeometryRepresentation DestinationRepresentation(
    const OcctGeometryRepresentation theRepresentation) noexcept
{
    if (theRepresentation == OcctGeometryRepresentation::LegacyUnknown) {
        return OcctGeometryRepresentation::BRep;
    }
    return theRepresentation;
}

Standard_Boolean IsSupportedRepresentation(
    const OcctGeometryRepresentation theRepresentation) noexcept
{
    return theRepresentation == OcctGeometryRepresentation::LegacyUnknown
        || theRepresentation == OcctGeometryRepresentation::BRep
        || theRepresentation == OcctGeometryRepresentation::TriangleMesh;
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

Standard_Boolean IsNearZero(const Standard_Real theValue) noexcept
{
    return !std::isfinite(theValue)
        || std::abs(theValue) <= Precision::Confusion();
}

constexpr Standard_Real kLegacyMetersPerUnit = 0.001;

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

//! Count topology occurrences, not only unique TShapes. This bounds the real
//! synchronous copy/persistence work even for an adversarial shared DAG.
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
        // TriangleMesh definitions may have triangulation-only faces. This
        // overload consumes an existing cache but does not invoke a mesher.
        BRepBndLib::Add(theShape, aBox, Standard_True);
        if (aBox.IsVoid() || aBox.IsOpen()) {
            return Standard_False;
        }
        aBox = aBox.Transformed(theTransform);
        Standard_Real aXMin = 0.0;
        Standard_Real aYMin = 0.0;
        Standard_Real aZMin = 0.0;
        Standard_Real aXMax = 0.0;
        Standard_Real aYMax = 0.0;
        Standard_Real aZMax = 0.0;
        aBox.Get(aXMin, aYMin, aZMin, aXMax, aYMax, aZMax);
        const std::array<Standard_Real, 6> aValues = {
            aXMin, aYMin, aZMin, aXMax, aYMax, aZMax};
        for (const Standard_Real aValue : aValues) {
            if (!std::isfinite(aValue)
                || std::abs(aValue)
                    > limits::kMaximumModelCoordinateMagnitude) {
                return Standard_False;
            }
        }
        theMinimum[0] = aXMin;
        theMinimum[1] = aYMin;
        theMinimum[2] = aZMin;
        theMaximum[0] = aXMax;
        theMaximum[1] = aYMax;
        theMaximum[2] = aZMax;
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

gp_Trsf ArrayTransform(
    const gp_Trsf& theSourceTransform,
    const LinearArrayAxis theAxis,
    const Standard_Integer theOrdinal,
    const Standard_Real theSpacing) noexcept
{
    gp_Vec aTranslation(0.0, 0.0, 0.0);
    const Standard_Real aDistance =
        static_cast<Standard_Real>(theOrdinal) * theSpacing;
    switch (theAxis) {
        case LinearArrayAxis::X:
            aTranslation.SetX(aDistance);
            break;
        case LinearArrayAxis::Y:
            aTranslation.SetY(aDistance);
            break;
        case LinearArrayAxis::Z:
            aTranslation.SetZ(aDistance);
            break;
    }
    gp_Trsf aWorldTranslation;
    aWorldTranslation.SetTranslation(aTranslation);
    // World-space consecutive step is deliberately pre-multiplied. Reversing
    // this order rotates the requested world axis with the source object.
    return aWorldTranslation.Multiplied(theSourceTransform);
}

} // namespace

LinearArrayOperationController::LinearArrayOperationController(
    Handle(AIS_InteractiveContext) theContext,
    Handle(OcctDocument) theDocument)
    : _context(std::move(theContext)),
      _document(std::move(theDocument))
{
}

LinearArrayOperationController::~LinearArrayOperationController() noexcept
{
    _previewStateChangedCallback = {};
    for (Standard_Integer anAttempt = 0;
         anAttempt < 3 && _ownsDocumentCommand; ++anAttempt) {
        (void)abortOwnedCommand();
    }
    // AIS ownership is independent of OCAF outcome. Erasing a transient
    // preview is always safe, even when committed labels still need a redraw.
    (void)clearPreviewObjects();
}

void LinearArrayOperationController::notifyStateChanged() noexcept
{
    try {
        if (_previewStateChangedCallback) {
            _previewStateChangedCallback();
        }
    } catch (...) {
    }
}

void LinearArrayOperationController::setPreviewStateChangedCallback(
    std::function<void()> theCallback)
{
    _previewStateChangedCallback = std::move(theCallback);
}

Standard_Boolean LinearArrayOperationController::captureSelectedSource(
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
        const std::string anEntityIdentifier =
            _document->EntityIdentifierForLabel(aLabel);
        const std::string aDefinitionIdentifier =
            _document->DefinitionIdentifierForLabel(aLabel);
        Standard_Size aTopologyNodeCount = 0;
        Standard_Real aMetersPerUnit = kLegacyMetersPerUnit;
        OcctReferenceAxis aReferenceAxis;
        const OcctReferenceAxisReadState aReferenceAxisState =
            _document->ReadReferenceAxisForLabel(
                aLabel, aReferenceAxis);
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
            || TransformDiffers(
                _document->ObjectTransformForLabel(aLabel), aTransform)
            || anEntityIdentifier.empty()
            || aDefinitionIdentifier.empty()
            || aReferenceAxisState
                == OcctReferenceAxisReadState::Invalid
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
            || !WorldBounds(aStored, aTransform, aMinimum, aMaximum)) {
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
        theSource.referenceAxisState = aReferenceAxisState;
        theSource.referenceAxis = aReferenceAxis;
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

Standard_Boolean LinearArrayOperationController::sourceIsCurrent(
    const Standard_Boolean theRequireOriginalDocumentTime) const noexcept
{
    return _source.has_value()
        && sourceIsCurrent(*_source, theRequireOriginalDocumentTime);
}

Standard_Boolean LinearArrayOperationController::sourceIsCurrent(
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
        Standard_Real aMinimum[3] = {0.0, 0.0, 0.0};
        Standard_Real aMaximum[3] = {0.0, 0.0, 0.0};
        if (aStored.IsNull() || theSource.storedShape.IsNull()
            || !aStored.IsEqual(theSource.storedShape)
            || theSource.presentation->Shape().IsNull()
            || !theSource.presentation->Shape().IsEqual(
                theSource.storedShape)
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
            || ReferenceAxisDiffers(
                aReferenceAxis, theSource.referenceAxis)
            || !TryReadMetersPerUnit(aDocument, aMetersPerUnit)
            || aMetersPerUnit != theSource.metersPerUnit
            || TransformDiffers(
                theSource.presentation->LocalTransformation(),
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
                aStored, theSource.transform, aMinimum, aMaximum)) {
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

Standard_Boolean LinearArrayOperationController::canAdmitCount(
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

Standard_Integer LinearArrayOperationController::maximumAdmittedCount(
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

std::pair<Standard_Real, Standard_Real>
LinearArrayOperationController::spacingRange(
    const SourceSnapshot& theSource,
    const LinearArrayAxis theAxis,
    const Standard_Integer theCount) const noexcept
{
    const Standard_Integer anAxis = AxisIndex(theAxis);
    const Standard_Integer aLastOrdinal = theCount - 1;
    if (anAxis < 0 || aLastOrdinal <= 0
        || !std::isfinite(theSource.worldMinimum[anAxis])
        || !std::isfinite(theSource.worldMaximum[anAxis])) {
        return {0.0, 0.0};
    }
    const Standard_Real aCoordinateLimit =
        limits::kMaximumModelCoordinateMagnitude;
    const Standard_Real anOrdinal =
        static_cast<Standard_Real>(aLastOrdinal);
    Standard_Real aMinimum =
        (-aCoordinateLimit - theSource.worldMinimum[anAxis]) / anOrdinal;
    Standard_Real aMaximum =
        (aCoordinateLimit - theSource.worldMaximum[anAxis]) / anOrdinal;
    const Standard_Real aSpanLimit = kMaximumArraySpan / anOrdinal;
    aMinimum = std::max(aMinimum, -aSpanLimit);
    aMaximum = std::min(aMaximum, aSpanLimit);
    if (!std::isfinite(aMinimum) || !std::isfinite(aMaximum)
        || aMinimum > aMaximum) {
        return {0.0, 0.0};
    }
    return {aMinimum, aMaximum};
}

Standard_Boolean LinearArrayOperationController::begin() noexcept
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
    const Standard_Integer anInitialCount =
        aMaximum >= kDefaultCount ? kDefaultCount : kMinimumCount;
    const auto aRange = spacingRange(
        aSource, LinearArrayAxis::X, anInitialCount);
    const Standard_Real anExtent =
        aSource.worldMaximum[0] - aSource.worldMinimum[0];
    const Standard_Real aDesired =
        std::isfinite(anExtent) && anExtent > Precision::Confusion()
        ? anExtent
        : 10.0;
    Standard_Real anInitialSpacing = 0.0;
    if (aRange.second > Precision::Confusion()) {
        anInitialSpacing = std::min(aDesired, aRange.second);
    } else if (aRange.first < -Precision::Confusion()) {
        anInitialSpacing = std::max(-aDesired, aRange.first);
    } else {
        return Standard_False;
    }

    _source = std::move(aSource);
    _axis = LinearArrayAxis::X;
    _count = anInitialCount;
    _maximumAdmittedCount = aMaximum;
    _spacing = anInitialSpacing;
    _previewValid = Standard_False;
    _ownsDocumentCommand = Standard_False;
    _pendingResults.clear();
    _state = LinearArrayPreviewState::Selecting;
    ++_generation;
    if (!rebuildPreview()) {
        _state = LinearArrayPreviewState::Failed;
        ++_generation;
        notifyStateChanged();
        return Standard_False;
    }
    _state = LinearArrayPreviewState::Ready;
    ++_generation;
    notifyStateChanged();
    return Standard_True;
}

Standard_Boolean LinearArrayOperationController::rebuildPreview() noexcept
{
    if (!_source.has_value() || IsNearZero(_spacing)
        || _count < kMinimumCount || _count > _maximumAdmittedCount
        || !sourceIsCurrent(Standard_True)
        || !canAdmitCount(*_source, _count)) {
        return Standard_False;
    }
    const auto aRange = spacingRange(*_source, _axis, _count);
    if (_spacing < aRange.first || _spacing > aRange.second) {
        return Standard_False;
    }
    if (!_previewObjects.empty() && !_previewValid) {
        if (!clearPreviewObjects()) {
            return Standard_False;
        }
    }

    std::vector<Handle(AIS_Shape)> aReplacement;
    try {
        aReplacement.reserve(static_cast<std::size_t>(_count - 1));
        for (Standard_Integer anOrdinal = 1;
             anOrdinal < _count; ++anOrdinal) {
            Handle(AIS_Shape) aPreview =
                new AIS_Shape(_source->storedShape);
            if (aPreview.IsNull() || aPreview->Shape().IsNull()
                || !aPreview->Shape().IsPartner(_source->storedShape)) {
                return Standard_False;
            }
            aPreview->SetLocalTransformation(ArrayTransform(
                _source->transform, _axis, anOrdinal, _spacing));
            Standard_Real aMinimum[3] = {0.0, 0.0, 0.0};
            Standard_Real aMaximum[3] = {0.0, 0.0, 0.0};
            if (!WorldBounds(
                    aPreview->Shape(),
                    aPreview->LocalTransformation(),
                    aMinimum,
                    aMaximum)) {
                return Standard_False;
            }
            _document->LoadObjectMeterial(_source->label, aPreview);
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
    // Claim every old/new handle before the first Display/Erase. A partial
    // replacement can therefore never orphan an AIS presentation.
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

Standard_Boolean LinearArrayOperationController::clearPreviewObjects() noexcept
{
    _previewValid = Standard_False;
    std::vector<Handle(AIS_Shape)> anUnresolved;
    try {
        anUnresolved.reserve(_previewObjects.size());
    } catch (...) {
        _state = LinearArrayPreviewState::Failed;
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
        _state = LinearArrayPreviewState::Failed;
        ++_generation;
        return Standard_False;
    }
    return Standard_True;
}

Standard_Boolean LinearArrayOperationController::setAxis(
    const LinearArrayAxis theAxis) noexcept
{
    if (!_source.has_value() || AxisIndex(theAxis) < 0
        || _state == LinearArrayPreviewState::Committing
        || _state == LinearArrayPreviewState::OutcomeUnknown
        || _ownsDocumentCommand || !_pendingResults.empty()
        || !sourceIsCurrent(Standard_True)) {
        return Standard_False;
    }
    const auto aRange = spacingRange(*_source, theAxis, _count);
    if (!IsNearZero(_spacing)
        && (_spacing < aRange.first || _spacing > aRange.second)) {
        return Standard_False;
    }
    if (_axis == theAxis && _state == LinearArrayPreviewState::Ready) {
        return Standard_True;
    }
    _axis = theAxis;
    if (IsNearZero(_spacing)) {
        if (!clearPreviewObjects()) {
            _state = LinearArrayPreviewState::Failed;
            notifyStateChanged();
            return Standard_False;
        }
        _state = LinearArrayPreviewState::Selecting;
        ++_generation;
        notifyStateChanged();
        return Standard_True;
    }
    if (!rebuildPreview()) {
        _state = LinearArrayPreviewState::Failed;
        ++_generation;
        notifyStateChanged();
        return Standard_False;
    }
    _state = LinearArrayPreviewState::Ready;
    ++_generation;
    notifyStateChanged();
    return Standard_True;
}

Standard_Boolean LinearArrayOperationController::setCount(
    const Standard_Integer theCount) noexcept
{
    if (!_source.has_value()
        || theCount < kMinimumCount
        || theCount > _maximumAdmittedCount
        || _state == LinearArrayPreviewState::Committing
        || _state == LinearArrayPreviewState::OutcomeUnknown
        || _ownsDocumentCommand || !_pendingResults.empty()
        || !sourceIsCurrent(Standard_True)
        || !canAdmitCount(*_source, theCount)) {
        return Standard_False;
    }
    const auto aRange = spacingRange(*_source, _axis, theCount);
    if (!IsNearZero(_spacing)
        && (_spacing < aRange.first || _spacing > aRange.second)) {
        return Standard_False;
    }
    if (_count == theCount && _state == LinearArrayPreviewState::Ready) {
        return Standard_True;
    }
    _count = theCount;
    if (IsNearZero(_spacing)) {
        if (!clearPreviewObjects()) {
            _state = LinearArrayPreviewState::Failed;
            notifyStateChanged();
            return Standard_False;
        }
        _state = LinearArrayPreviewState::Selecting;
        ++_generation;
        notifyStateChanged();
        return Standard_True;
    }
    if (!rebuildPreview()) {
        _state = LinearArrayPreviewState::Failed;
        ++_generation;
        notifyStateChanged();
        return Standard_False;
    }
    _state = LinearArrayPreviewState::Ready;
    ++_generation;
    notifyStateChanged();
    return Standard_True;
}

Standard_Boolean LinearArrayOperationController::setSpacing(
    const Standard_Real theSpacing) noexcept
{
    if (!_source.has_value() || !std::isfinite(theSpacing)
        || _state == LinearArrayPreviewState::Committing
        || _state == LinearArrayPreviewState::OutcomeUnknown
        || _ownsDocumentCommand || !_pendingResults.empty()
        || !sourceIsCurrent(Standard_True)) {
        return Standard_False;
    }
    const auto aRange = spacingRange(*_source, _axis, _count);
    if (!IsNearZero(theSpacing)
        && (theSpacing < aRange.first || theSpacing > aRange.second)) {
        return Standard_False;
    }
    const Standard_Boolean isSame =
        std::abs(_spacing - theSpacing) <= Precision::Confusion();
    _spacing = theSpacing;
    if (IsNearZero(theSpacing)) {
        // Zero Step is valid input but not a valid array preview. Remove all
        // prior bodies so stale geometry cannot imply an applicable result.
        if (!clearPreviewObjects()) {
            _state = LinearArrayPreviewState::Failed;
            notifyStateChanged();
            return Standard_False;
        }
        _state = LinearArrayPreviewState::Selecting;
        ++_generation;
        notifyStateChanged();
        return Standard_True;
    }
    if (isSame && _state == LinearArrayPreviewState::Ready) {
        return Standard_True;
    }
    // Same-value retries are deliberate while Failed; UIKit controls often
    // resend their current value to recover a partial AIS replacement.
    if (!rebuildPreview()) {
        _state = LinearArrayPreviewState::Failed;
        ++_generation;
        notifyStateChanged();
        return Standard_False;
    }
    _state = LinearArrayPreviewState::Ready;
    ++_generation;
    notifyStateChanged();
    return Standard_True;
}

LinearArrayAxis LinearArrayOperationController::axis() const noexcept
{
    return _axis;
}

Standard_Integer LinearArrayOperationController::count() const noexcept
{
    return _count;
}

Standard_Real LinearArrayOperationController::spacing() const noexcept
{
    return _spacing;
}

Standard_Real LinearArrayOperationController::metersPerUnit() const noexcept
{
    return _source.has_value() ? _source->metersPerUnit : 0.0;
}

std::pair<Standard_Integer, Standard_Integer>
LinearArrayOperationController::countRange() const noexcept
{
    return {
        kMinimumCount,
        std::max(kMinimumCount, _maximumAdmittedCount),
    };
}

std::pair<Standard_Real, Standard_Real>
LinearArrayOperationController::spacingRange() const noexcept
{
    return _source.has_value()
        ? spacingRange(*_source, _axis, _count)
        : std::pair<Standard_Real, Standard_Real>{0.0, 0.0};
}

LinearArrayPreviewState
LinearArrayOperationController::previewState() const noexcept
{
    return _state;
}

std::uint64_t
LinearArrayOperationController::previewGeneration() const noexcept
{
    return _generation;
}

Standard_Boolean LinearArrayOperationController::hasUnresolvedState() const
    noexcept
{
    return !_previewObjects.empty() || !_pendingResults.empty()
        || _ownsDocumentCommand;
}

Standard_Boolean LinearArrayOperationController::hasActiveOperation() const
    noexcept
{
    return _state != LinearArrayPreviewState::Unavailable
        || _source.has_value() || hasUnresolvedState();
}

Standard_Boolean LinearArrayOperationController::canApply() const noexcept
{
    return (_state == LinearArrayPreviewState::Ready
            && _previewValid && _source.has_value()
            && !IsNearZero(_spacing)
            && _previewObjects.size()
                == static_cast<std::size_t>(_count - 1))
        || (_state == LinearArrayPreviewState::OutcomeUnknown
            && (_ownsDocumentCommand || !_pendingResults.empty()));
}

Standard_Boolean LinearArrayOperationController::capturePreview(
    std::vector<Handle(AIS_Shape)>& thePreviewObjects) const noexcept
{
    thePreviewObjects.clear();
    if (_state != LinearArrayPreviewState::Ready || !_previewValid
        || !_source.has_value() || !sourceIsCurrent(Standard_True)
        || HasUnsupportedOverlayTexture(_source->label)
        || _previewObjects.empty()
        || _previewObjects.size()
            != static_cast<std::size_t>(_count - 1)
        || _previewObjects.size() > kMaximumPreviewBodies) {
        return Standard_False;
    }
    try {
        for (const Handle(AIS_Shape)& aPreview : _previewObjects) {
            TColStd_ListOfInteger anActiveModes;
            if (aPreview.IsNull() || aPreview->Shape().IsNull()
                || !_context->IsDisplayed(aPreview)) {
                return Standard_False;
            }
            _context->ActivatedModes(aPreview, anActiveModes);
            if (!anActiveModes.IsEmpty()) {
                return Standard_False;
            }
        }
        thePreviewObjects = _previewObjects;
        return Standard_True;
    } catch (...) {
        thePreviewObjects.clear();
        return Standard_False;
    }
}

Standard_Boolean
LinearArrayOperationController::canPublishEmptyPreview() const noexcept
{
    return _state == LinearArrayPreviewState::Selecting
        && !_previewValid
        && _source.has_value()
        && IsNearZero(_spacing)
        && _previewObjects.empty()
        && _pendingResults.empty()
        && !_ownsDocumentCommand
        && sourceIsCurrent(Standard_True)
        && !HasUnsupportedOverlayTexture(_source->label);
}

LinearArrayOperationController::DocumentState
LinearArrayOperationController::inspectPendingResults() const noexcept
{
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
            return _ownsDocumentCommand
                ? DocumentState::OpenCommand
                : DocumentState::Unavailable;
        }
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

Standard_Boolean LinearArrayOperationController::abortOwnedCommand() noexcept
{
    if (!_ownsDocumentCommand || _document.IsNull()) {
        return Standard_False;
    }
    try {
        const Handle(TDocStd_Document) aDocument = _document->ChangeDocument();
        if (aDocument.IsNull()) {
            return Standard_False;
        }
        if (aDocument->HasOpenCommand()) {
#ifdef DEBUG
            if (_debugAbortFailureCount > 0) {
                --_debugAbortFailureCount;
                return Standard_False;
            }
#endif
            aDocument->AbortCommand();
        }
        if (aDocument->HasOpenCommand()) {
            return Standard_False;
        }
        _ownsDocumentCommand = Standard_False;
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

LinearArrayApplyResult
LinearArrayOperationController::finishCommittedOperation() noexcept
{
    // The labels prove that this exact array already committed. Keep them as
    // the exactly-once recovery token until every transient preview is erased.
    if (!clearPreviewObjects()) {
        _ownsDocumentCommand = Standard_False;
        _state = LinearArrayPreviewState::OutcomeUnknown;
        ++_generation;
        notifyStateChanged();
        return LinearArrayApplyResult::NoChange;
    }
    try {
        _document->NotifyChanges();
    } catch (...) {
    }
    clearInactiveState();
    notifyStateChanged();
    return LinearArrayApplyResult::AppliedNeedsDocumentRedraw;
}

LinearArrayApplyResult LinearArrayOperationController::apply() noexcept
{
    if (!hasActiveOperation()) {
        return LinearArrayApplyResult::NoChange;
    }
    const auto inspectAfterCommit = [this]() noexcept {
#ifdef DEBUG
        if (_debugPostCommitInspectFailureCount > 0) {
            --_debugPostCommitInspectFailureCount;
            return DocumentState::Unavailable;
        }
#endif
        return inspectPendingResults();
    };
    if (_state == LinearArrayPreviewState::OutcomeUnknown
        || _ownsDocumentCommand || !_pendingResults.empty()) {
        DocumentState aState = inspectAfterCommit();
        if (aState == DocumentState::OpenCommand
            && abortOwnedCommand()) {
            aState = inspectAfterCommit();
        }
        if (aState == DocumentState::AllCommitted) {
            return finishCommittedOperation();
        }
        if (aState == DocumentState::None) {
            _pendingResults.clear();
            _ownsDocumentCommand = Standard_False;
            _state = _previewValid
                ? LinearArrayPreviewState::Ready
                : LinearArrayPreviewState::Failed;
            ++_generation;
            notifyStateChanged();
            return LinearArrayApplyResult::NoChange;
        }
        _state = LinearArrayPreviewState::OutcomeUnknown;
        ++_generation;
        notifyStateChanged();
        return LinearArrayApplyResult::NoChange;
    }
    if (_state != LinearArrayPreviewState::Ready || !canApply()
        || !sourceIsCurrent(Standard_True)
        || !canAdmitCount(*_source, _count)) {
        _state = LinearArrayPreviewState::Failed;
        ++_generation;
        notifyStateChanged();
        return LinearArrayApplyResult::NoChange;
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
                    ? Standard_True
                    : Standard_False);
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
                    > anAggregateLimit - anAggregateTopology) {
                return LinearArrayApplyResult::NoChange;
            }
            for (const PreparedResult& anExisting : aPrepared) {
                if (anExisting.presentation.IsNull()
                    || aCopy.Shape().IsPartner(
                        anExisting.presentation->Shape())) {
                    return LinearArrayApplyResult::NoChange;
                }
            }
            anAggregateTopology += aResultTopology;
            const gp_Trsf aTransform = ArrayTransform(
                _source->transform, _axis, anOrdinal, _spacing);
            Standard_Real aMinimum[3] = {0.0, 0.0, 0.0};
            Standard_Real aMaximum[3] = {0.0, 0.0, 0.0};
            if (!WorldBounds(
                    aCopy.Shape(), aTransform, aMinimum, aMaximum)) {
                return LinearArrayApplyResult::NoChange;
            }
            Handle(AIS_Shape) aPresentation =
                new AIS_Shape(aCopy.Shape());
            aPresentation->SetLocalTransformation(aTransform);
            _document->LoadObjectMeterial(
                _source->label, aPresentation);
            aPrepared.push_back({aPresentation, aTransform});
        }
    } catch (...) {
        return LinearArrayApplyResult::NoChange;
    }
    if (aPrepared.size() != static_cast<std::size_t>(_count - 1)
        || !sourceIsCurrent(Standard_True)
        || !canAdmitCount(*_source, _count)) {
        return LinearArrayApplyResult::NoChange;
    }

    Handle(TDocStd_Document) aDocument;
    try {
        aDocument = _document->ChangeDocument();
        if (aDocument.IsNull() || aDocument != _source->document
            || aDocument->HasOpenCommand()) {
            return LinearArrayApplyResult::NoChange;
        }
    } catch (...) {
        return LinearArrayApplyResult::NoChange;
    }
    const auto retainRetryableOrUnknown = [this]() noexcept {
        if (abortOwnedCommand()
            && inspectPendingResults() == DocumentState::None) {
            _pendingResults.clear();
            _state = _previewValid
                ? LinearArrayPreviewState::Ready
                : LinearArrayPreviewState::Failed;
        } else {
            _state = LinearArrayPreviewState::OutcomeUnknown;
        }
        ++_generation;
        notifyStateChanged();
        return LinearArrayApplyResult::NoChange;
    };

    _state = LinearArrayPreviewState::Committing;
    _pendingResults.clear();
    ++_generation;
    notifyStateChanged();
    try {
        OCC_CATCH_SIGNALS
        _ownsDocumentCommand = Standard_True;
        aDocument->NewCommand();
        if (!aDocument->HasOpenCommand()) {
            _ownsDocumentCommand = Standard_False;
            _state = LinearArrayPreviewState::Ready;
            ++_generation;
            notifyStateChanged();
            return LinearArrayApplyResult::NoChange;
        }
#ifdef DEBUG
        if (_debugTransactionFailureCount > 0) {
            --_debugTransactionFailureCount;
            return retainRetryableOrUnknown();
        }
#endif
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
        if (!_document->ValidateGeometryRepresentations()) {
            return retainRetryableOrUnknown();
        }
        try {
            Standard_Boolean aCommitReported = aDocument->CommitCommand();
#ifdef DEBUG
            const Standard_Integer aCommitMode = _debugCommitMode;
            _debugCommitMode = 0;
            if (aCommitMode == 1) {
                aCommitReported = Standard_False;
            } else if (aCommitMode == 2) {
                throw Standard_Failure(
                    "Injected Linear Array commit exception after close");
            }
#endif
            (void)aCommitReported;
        } catch (...) {
            // The command state and pending labels below are authoritative.
        }
    } catch (...) {
        return retainRetryableOrUnknown();
    }

    DocumentState aState = inspectAfterCommit();
    if (aState == DocumentState::OpenCommand && abortOwnedCommand()) {
        aState = inspectAfterCommit();
    }
    if (aState == DocumentState::AllCommitted) {
        return finishCommittedOperation();
    }
    if (aState == DocumentState::None) {
        _pendingResults.clear();
        _ownsDocumentCommand = Standard_False;
        _state = _previewValid
            ? LinearArrayPreviewState::Ready
            : LinearArrayPreviewState::Failed;
        ++_generation;
        notifyStateChanged();
        return LinearArrayApplyResult::NoChange;
    }
    _state = LinearArrayPreviewState::OutcomeUnknown;
    ++_generation;
    notifyStateChanged();
    return LinearArrayApplyResult::NoChange;
}

Standard_Boolean LinearArrayOperationController::cancel() noexcept
{
    if (!hasActiveOperation()) {
        return Standard_True;
    }
    if (_state == LinearArrayPreviewState::Committing
        || _state == LinearArrayPreviewState::OutcomeUnknown
        || _ownsDocumentCommand || !_pendingResults.empty()) {
        _state = LinearArrayPreviewState::OutcomeUnknown;
        ++_generation;
        notifyStateChanged();
        return Standard_False;
    }
    if (!clearPreviewObjects()) {
        _state = LinearArrayPreviewState::Failed;
        notifyStateChanged();
        return Standard_False;
    }
    clearInactiveState();
    notifyStateChanged();
    return Standard_True;
}

void LinearArrayOperationController::clearInactiveState() noexcept
{
    _previewObjects.clear();
    _pendingResults.clear();
    _source.reset();
    _axis = LinearArrayAxis::X;
    _count = kDefaultCount;
    _maximumAdmittedCount = kMaximumCount;
    _spacing = 0.0;
    _previewValid = Standard_False;
    _ownsDocumentCommand = Standard_False;
    _state = LinearArrayPreviewState::Unavailable;
    ++_generation;
}

#ifdef DEBUG
LinearArrayPreviewDebugState
LinearArrayOperationController::debugPreviewState() const noexcept
{
    LinearArrayPreviewDebugState aState;
    aState.state = _state;
    aState.generation = _generation;
    aState.previewBodyCount = _previewObjects.size();
    aState.pendingResultCount = _pendingResults.size();
    aState.activeOperation = hasActiveOperation();
    aState.previewValid = _previewValid;
    aState.canApply = canApply();
    aState.ownsDocumentCommand = _ownsDocumentCommand;
    aState.axis = _axis;
    aState.count = _count;
    aState.spacing = _spacing;
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

void LinearArrayOperationController::debugSetTransactionFailureCount(
    const Standard_Size theCount) noexcept
{
    _debugTransactionFailureCount = theCount;
}

void LinearArrayOperationController::debugSetAbortFailureCount(
    const Standard_Size theCount) noexcept
{
    _debugAbortFailureCount = theCount;
}

void LinearArrayOperationController::debugSetEraseFailureCount(
    const Standard_Size theCount) noexcept
{
    _debugEraseFailureCount = theCount;
}

void LinearArrayOperationController::debugSetCommitMode(
    const Standard_Integer theMode) noexcept
{
    _debugCommitMode = theMode >= 0 && theMode <= 2 ? theMode : 0;
}

void LinearArrayOperationController::debugSetPostCommitInspectFailureCount(
    const Standard_Size theCount) noexcept
{
    _debugPostCommitInspectFailureCount = theCount;
}

void LinearArrayOperationController::debugSetMaximumTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    _debugMaximumTopologyNodes = std::max<Standard_Size>(
        1,
        std::min(theLimit, kMaximumAggregateTopologyNodes));
}

Standard_Boolean
LinearArrayOperationController::debugMutateFirstSourcePersistedTransform()
    noexcept
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
