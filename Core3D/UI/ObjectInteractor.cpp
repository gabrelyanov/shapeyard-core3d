//
//  ObjectInteractor.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 19.04.2024.
//

#include "ObjectInteractor.hpp"
#include "../Scene/SceneSnapshot.hpp"
#include "../Common/Core3DMobileResourceLimits.h"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRep_Tool.hxx>
#include <Poly_ListOfTriangulation.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_TriangulationParameters.hxx>
#include <Precision.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <TColStd_ListIteratorOfListOfInteger.hxx>
#include <TColStd_ListOfInteger.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <V3d_View.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Real.hxx>
#include <GP_Quaternion.hxx>
#include <Graphic3d_ZLayerId.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_LineAspect.hxx>
#include <AIS_Shape.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace core3d {
	namespace {
		bool IsBRepModelingRepresentation(
			const OcctGeometryRepresentation theRepresentation) noexcept {
			return theRepresentation
					== OcctGeometryRepresentation::LegacyUnknown
				|| theRepresentation == OcctGeometryRepresentation::BRep;
		}

		bool IsObjectModelingRepresentation(
			const OcctGeometryRepresentation theRepresentation) noexcept {
			return IsBRepModelingRepresentation(theRepresentation)
				|| theRepresentation
					== OcctGeometryRepresentation::TriangleMesh;
		}

		bool RestoreTriangleMeshCopyParameters(
			const TopoDS_Shape& theSource,
			BRepBuilderAPI_Copy& theCopy) noexcept {
			try {
				Standard_Size aFaceCount = 0;
				for (TopExp_Explorer aFaceExplorer(
						theSource, TopAbs_FACE);
					 aFaceExplorer.More(); aFaceExplorer.Next()) {
					const TopoDS_Face aSourceFace =
						TopoDS::Face(aFaceExplorer.Current());
					const TopoDS_Shape aCopiedShape =
						theCopy.ModifiedShape(aSourceFace);
					if (aCopiedShape.IsNull()
						|| aCopiedShape.ShapeType() != TopAbs_FACE) {
						return false;
					}
					const TopoDS_Face aCopiedFace =
						TopoDS::Face(aCopiedShape);
					TopLoc_Location aSourceLocation;
					TopLoc_Location aCopiedLocation;
					const Poly_ListOfTriangulation& aSourceMeshes =
						BRep_Tool::Triangulations(
							aSourceFace, aSourceLocation);
					const Poly_ListOfTriangulation& aCopiedMeshes =
						BRep_Tool::Triangulations(
							aCopiedFace, aCopiedLocation);
					// OCCT 7.8 copies only the active triangulation. Reject a
					// source carrying hidden alternates instead of silently
					// producing a lossy definition.
					if (aSourceMeshes.Size() != 1
						|| aCopiedMeshes.Size() != 1) {
						return false;
					}
					const Handle(Poly_Triangulation)& aSourceMesh =
						BRep_Tool::Triangulation(
							aSourceFace, aSourceLocation);
					const Handle(Poly_Triangulation)& aCopiedMesh =
						BRep_Tool::Triangulation(
							aCopiedFace, aCopiedLocation);
					if (aSourceMesh.IsNull() || aCopiedMesh.IsNull()
						|| aCopiedFace.IsPartner(aSourceFace)
						|| aCopiedFace.Orientation()
							!= aSourceFace.Orientation()
						|| !aCopiedLocation.IsEqual(aSourceLocation)
						|| aSourceMesh == aCopiedMesh
						|| aSourceMesh->Deflection()
							!= aCopiedMesh->Deflection()
						|| aSourceMesh->NbNodes()
							!= aCopiedMesh->NbNodes()
						|| aSourceMesh->NbTriangles()
							!= aCopiedMesh->NbTriangles()
						|| aSourceMesh->HasUVNodes()
							!= aCopiedMesh->HasUVNodes()
						|| aSourceMesh->HasNormals()
							!= aCopiedMesh->HasNormals()
						|| aSourceMesh->MeshPurpose()
							!= aCopiedMesh->MeshPurpose()) {
						return false;
					}

					// OCCT 7.8 deep-copies the active triangulation but omits
					// its immutable generation parameters. Restore an
					// independent value object so snapshot compatibility and
					// persistence metadata remain faithful.
					const Handle(Poly_TriangulationParameters)&
						aSourceParameters = aSourceMesh->Parameters();
					if (!aSourceParameters.IsNull()) {
						aCopiedMesh->Parameters(
							new Poly_TriangulationParameters(
								aSourceParameters->Deflection(),
								aSourceParameters->Angle(),
								aSourceParameters->MinSize()));
						const Handle(Poly_TriangulationParameters)&
							aCopiedParameters = aCopiedMesh->Parameters();
						if (aCopiedParameters.IsNull()
							|| aCopiedParameters->Deflection()
								!= aSourceParameters->Deflection()
							|| aCopiedParameters->Angle()
								!= aSourceParameters->Angle()
							|| aCopiedParameters->MinSize()
								!= aSourceParameters->MinSize()) {
							return false;
						}
					} else if (!aCopiedMesh->Parameters().IsNull()) {
						return false;
					}
					++aFaceCount;
				}
				return aFaceCount > 0;
			} catch (...) {
				return false;
			}
		}

		bool SelectionSupportsBRepModeling(
			const Handle(Core3DContext)& theContext,
			const Handle(OcctDocument)& theDocument) noexcept {
			if (theContext.IsNull() || theDocument.IsNull()) {
				return false;
			}
			try {
				bool hasSelection = false;
				for (theContext->InitSelected(); theContext->MoreSelected();
					 theContext->NextSelected()) {
					const Handle(AIS_InteractiveObject) aSelected =
						theContext->SelectedInteractive();
					const TDF_Label aLabel =
						theDocument->ShapeLabel(aSelected);
					if (aSelected.IsNull()
						|| !theDocument->IsPresentationEditable(aSelected)
						|| aLabel.IsNull()
						|| !theDocument
							->IsEditableFreeSimpleDefinitionLabel(aLabel)
						|| !IsBRepModelingRepresentation(
							theDocument->GeometryRepresentationForLabel(
								aLabel))) {
						return false;
					}
					hasSelection = true;
				}
				return hasSelection;
			} catch (...) {
				return false;
			}
		}

		bool ManipulatorRequiresBRepModeling(
			const PrimitiveManipulatorType theType) noexcept {
			switch (theType) {
				case PrimitiveManipulatorType::PrimitiveGizmoTypeScale:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeUnion:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeIntersect:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeMirror:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeExtrude:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeShell:
					return true;
				case PrimitiveManipulatorType::PrimitiveGizmoTypeNone:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeMaterial:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray:
					return false;
			}
			return true;
		}

		bool ManipulatorAllowsEmptySelection(
			const PrimitiveManipulatorType theType) noexcept {
			switch (theType) {
				case PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeUnion:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeIntersect:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeExtrude:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeShell:
					// These modes acquire and validate their BRep source after
					// the tool is entered.  Empty selection is therefore a valid
					// idle state, while an existing mesh/unsafe selection must
					// still fail closed.
					return true;
				case PrimitiveManipulatorType::PrimitiveGizmoTypeNone:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeScale:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeMirror:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeMaterial:
				case PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray:
					return false;
			}
			return false;
		}

		bool SelectionIsEmpty(
			const Handle(Core3DContext)& theContext) noexcept {
			if (theContext.IsNull()) {
				return false;
			}
			try {
				theContext->InitSelected();
				return !theContext->MoreSelected();
			} catch (...) {
				return false;
			}
		}

		bool ManipulatorObjectsSupportBRepModeling(
			const Handle(Core3DManipulator)& theManipulator,
			const Handle(OcctDocument)& theDocument,
			const PrimitiveManipulatorType theManipulatorType,
			const std::unordered_map<
				const AIS_InteractiveObject*, TDF_Label>&
				theSourceLabels) noexcept {
			if (theManipulator.IsNull() || theDocument.IsNull()
				|| !theManipulator->IsAttached()) {
				return false;
			}
			try {
				const Handle(TDocStd_Document) aDocument =
					theDocument->Document();
				const Handle(Core3DManipulatorObjectSequence) anObjects =
					theManipulator->Objects();
				const auto& aCachedShapes =
					theManipulator->cachedShapes();
				if (aDocument.IsNull()
					|| anObjects.IsNull() || anObjects->Size() == 0) {
					return false;
				}
				for (Core3DManipulatorObjectSequence::Iterator anObject(
						*anObjects);
					 anObject.More(); anObject.Next()) {
					const Handle(AIS_InteractiveObject)& aPresentation =
						anObject.Value();
					const auto aCached = aCachedShapes.find(aPresentation);
					const auto aSourceLabel = theSourceLabels.find(
						aPresentation.get());
					const TDF_Label aLabel =
						aSourceLabel == theSourceLabels.end()
						? TDF_Label()
						: aSourceLabel->second;
					const TopoDS_Shape aStoredShape =
						aLabel.IsNull()
						? TopoDS_Shape()
						: XCAFDoc_ShapeTool::GetShape(aLabel);
					const bool hasCachedShape =
						aCached != aCachedShapes.end()
						&& !aCached->second.IsNull();
					const Handle(AIS_Shape) aPresentationShape =
						Handle(AIS_Shape)::DownCast(aPresentation);
					const bool isSourceShapeCurrent = hasCachedShape
						&& !aPresentationShape.IsNull()
						&& !aPresentationShape->Shape().IsNull()
						&& aPresentationShape->Shape().IsEqual(
							aCached->second);
					const bool mayOwnScaledPreview =
						theManipulatorType
							== PrimitiveManipulatorType::PrimitiveGizmoTypeScale
						&& theManipulator->HasActiveTransformation();
					const TDF_Label aCurrentLabel =
						theDocument->ShapeLabel(aPresentation);
					if (aPresentation.IsNull()
						|| !theDocument->IsPresentationEditable(aPresentation)
						|| aCached == aCachedShapes.end()
						|| aCached->second.IsNull()
						|| aLabel.IsNull()
						|| aLabel.Data() != aDocument->GetData()
						|| aStoredShape.IsNull()
						|| !aStoredShape.IsEqual(aCached->second)
						|| (!isSourceShapeCurrent && !mayOwnScaledPreview)
						|| (!aCurrentLabel.IsNull()
							&& !aCurrentLabel.IsEqual(aLabel))
						|| (aCurrentLabel.IsNull() && isSourceShapeCurrent)
						|| !theDocument
							->IsEditableFreeSimpleDefinitionLabel(aLabel)
						|| !IsBRepModelingRepresentation(
							theDocument->GeometryRepresentationForLabel(
								aLabel))) {
						return false;
					}
				}
				return true;
			} catch (...) {
				return false;
			}
		}

		Standard_Boolean IsTopologicallyValid(const TopoDS_Shape& shape) {
			if (shape.IsNull()) {
				return Standard_False;
			}
			try {
				BRepCheck_Analyzer analyzer(shape, Standard_True);
				return analyzer.IsValid();
			} catch (...) {
				return Standard_False;
			}
		}

		Standard_Boolean CountBoundedMirrorTopology(
			const TopoDS_Shape& theShape,
			const Standard_Size theMaximum,
			Standard_Size& theCount) noexcept {
			theCount = 0;
			if (theShape.IsNull() || theMaximum == 0) {
				return Standard_False;
			}
			try {
				std::vector<TopoDS_Shape> aPending{theShape};
				Standard_Size aTraversalCount = 0;
				while (!aPending.empty()) {
					const TopoDS_Shape aCurrent = aPending.back();
					aPending.pop_back();
					if (aCurrent.IsNull()
						|| ++aTraversalCount > theMaximum) {
						return Standard_False;
					}
					std::vector<TopoDS_Shape> aChildren;
					for (TopoDS_Iterator aChild(
							aCurrent, Standard_True, Standard_True);
						 aChild.More(); aChild.Next()) {
						const Standard_Size aRemaining =
							theMaximum - aTraversalCount;
						if (aPending.size() + aChildren.size()
								>= static_cast<std::size_t>(aRemaining)) {
							return Standard_False;
						}
						aChildren.push_back(aChild.Value());
					}
					for (auto aChild = aChildren.rbegin();
						 aChild != aChildren.rend(); ++aChild) {
						aPending.push_back(*aChild);
					}
				}
				// Count occurrences, not only unique TShapes. This is the actual
				// synchronous work budget consumed by shared-DAG topology and makes
				// the aggregate source/result ceiling authoritative.
				theCount = aTraversalCount;
				return theCount > 0;
			} catch (...) {
				theCount = 0;
				return Standard_False;
			}
		}

		//! Build the same deterministic, first-occurrence face ordering as
		//! TopExp::MapShapes using an explicit depth-first walk. Every topology
		//! occurrence and every queued child is charged before traversal, so a
		//! shared DAG cannot hide exponential work behind a small unique-node map.
		//! Descendants of faces are also visited and charged; this makes the output
		//! count a conservative whole-BRep work budget. An empty map is a valid
		//! face-less BRep; false always means the shape exceeded a safety budget
		//! or could not be inspected safely.
		Standard_Boolean MapBoundedMirrorReferenceFaces(
			const TopoDS_Shape& theShape,
			const Standard_Size theMaximumTopologyNodes,
			const Standard_Size theMaximumFaces,
			Standard_Size& theTopologyNodeCount,
			TopTools_IndexedMapOfShape& theFaces) noexcept {
			theTopologyNodeCount = 0;
			theFaces.Clear();
			if (theShape.IsNull() || theMaximumTopologyNodes == 0
				|| theMaximumFaces == 0) {
				return Standard_False;
			}
			try {
				std::vector<TopoDS_Shape> aPending{theShape};
				while (!aPending.empty()) {
					const TopoDS_Shape aCurrent = aPending.back();
					aPending.pop_back();
					if (aCurrent.IsNull()
						|| ++theTopologyNodeCount
							> theMaximumTopologyNodes) {
						theFaces.Clear();
						return Standard_False;
					}
					if (aCurrent.ShapeType() == TopAbs_FACE
						&& !theFaces.Contains(aCurrent)) {
						if (static_cast<Standard_Size>(theFaces.Extent())
							>= theMaximumFaces) {
							theFaces.Clear();
							return Standard_False;
						}
						theFaces.Add(aCurrent);
					}

					std::vector<TopoDS_Shape> aChildren;
					for (TopoDS_Iterator aChild(
							aCurrent, Standard_True, Standard_True);
						 aChild.More(); aChild.Next()) {
						const Standard_Size aRemaining =
							theMaximumTopologyNodes
								- theTopologyNodeCount;
						if (aPending.size() + aChildren.size()
								>= static_cast<std::size_t>(aRemaining)) {
							theFaces.Clear();
							return Standard_False;
						}
						aChildren.push_back(aChild.Value());
					}
					for (auto aChild = aChildren.rbegin();
						 aChild != aChildren.rend(); ++aChild) {
						aPending.push_back(*aChild);
					}
				}
				return Standard_True;
			} catch (...) {
				theTopologyNodeCount = 0;
				theFaces.Clear();
				return Standard_False;
			}
		}

		// Root appearance can be copied directly, but per-face/per-edge XCAF
		// assignments target source topology labels. Until Mirror remaps the
		// transform history into destination subshape labels, accepting one would
		// silently strip authored appearance. The direct-child scan is bounded
		// because XCAF subshape labels belong to their simple definition.
		Standard_Boolean HasStyledMirrorSubshape(
			const Handle(TDocStd_Document)& theDocument,
			const TDF_Label& theDefinition,
			const Standard_Size theMaximumLabels) noexcept {
			if (theDocument.IsNull() || theDefinition.IsNull()
				|| theDefinition.Data() != theDocument->GetData()
				|| theMaximumLabels == 0) {
				return Standard_True;
			}
			try {
				OCC_CATCH_SIGNALS
				const Handle(XCAFDoc_ColorTool) aColorTool =
					XCAFDoc_DocumentTool::CheckColorTool(
						theDocument->Main())
					? XCAFDoc_DocumentTool::ColorTool(theDocument->Main())
					: Handle(XCAFDoc_ColorTool)();
				const Handle(XCAFDoc_LayerTool) aLayerTool =
					XCAFDoc_DocumentTool::CheckLayerTool(
						theDocument->Main())
					? XCAFDoc_DocumentTool::LayerTool(theDocument->Main())
					: Handle(XCAFDoc_LayerTool)();
				Standard_Size aLabelCount = 0;
				for (TDF_ChildIterator anItem(
						theDefinition, Standard_False);
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
							&& (aColorTool->IsSet(
									aLabel, XCAFDoc_ColorGen)
								|| aColorTool->IsSet(
									aLabel, XCAFDoc_ColorSurf)
								|| aColorTool->IsSet(
									aLabel, XCAFDoc_ColorCurv)
								|| !XCAFDoc_ColorTool::IsVisible(aLabel)))
						|| !XCAFDoc_VisMaterialTool::GetShapeMaterial(
							aLabel).IsNull()
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

		bool TransformDiffers(const gp_Trsf& theLeft,
		                      const gp_Trsf& theRight) {
			for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
				for (Standard_Integer aColumn = 1; aColumn <= 4; ++aColumn) {
					if (std::abs(theLeft.Value(aRow, aColumn)
					             - theRight.Value(aRow, aColumn))
					    > Precision::Confusion()) {
						return true;
					}
				}
			}
			return false;
		}

		bool IsFiniteBoundedCoordinate(const Standard_Real theValue) noexcept {
			return std::isfinite(theValue)
				&& std::abs(theValue)
					<= limits::kMaximumModelCoordinateMagnitude;
		}

		bool IsFiniteBoundedPoint(const gp_Pnt& thePoint) noexcept {
			return IsFiniteBoundedCoordinate(thePoint.X())
				&& IsFiniteBoundedCoordinate(thePoint.Y())
				&& IsFiniteBoundedCoordinate(thePoint.Z());
		}

		bool IsFiniteDirection(const gp_Dir& theDirection) noexcept {
			return std::isfinite(theDirection.X())
				&& std::isfinite(theDirection.Y())
				&& std::isfinite(theDirection.Z());
		}

		bool IsShapeWithinModelCoordinates(
			const TopoDS_Shape& theShape,
			const gp_Trsf& theTransform = gp_Trsf()) noexcept {
			if (theShape.IsNull()) {
				return false;
			}
			try {
				Bnd_Box aBox;
				BRepBndLib::Add(theShape, aBox, Standard_False);
				if (aBox.IsVoid() || aBox.IsOpen()) {
					return false;
				}
				aBox = aBox.Transformed(theTransform);
				return IsFiniteBoundedPoint(aBox.CornerMin())
					&& IsFiniteBoundedPoint(aBox.CornerMax());
			} catch (...) {
				return false;
			}
		}

		bool BuildOrientedWorldPlane(
			const TopoDS_Face& theFace,
			const gp_Trsf& thePresentationTransform,
			gp_Pnt& theWorldOrigin,
			gp_Dir& theWorldNormal,
			gp_Dir& theWorldXDirection) noexcept {
			if (theFace.IsNull()
				|| (theFace.Orientation() != TopAbs_FORWARD
					&& theFace.Orientation() != TopAbs_REVERSED)) {
				return false;
			}
			try {
				BRepAdaptor_Surface aSurface(theFace, Standard_True);
				if (aSurface.GetType() != GeomAbs_Plane) {
					return false;
				}
				const gp_Pln aPlane = aSurface.Plane();
				theWorldOrigin = aPlane.Location();
				theWorldNormal = aPlane.Axis().Direction();
				if (theFace.Orientation() == TopAbs_REVERSED) {
					theWorldNormal.Reverse();
				}
				theWorldOrigin.Transform(thePresentationTransform);
				theWorldNormal.Transform(thePresentationTransform);
				if (!IsFiniteBoundedPoint(theWorldOrigin)
					|| !IsFiniteDirection(theWorldNormal)) {
					return false;
				}

				// Choose the least parallel positive world axis, then project it
				// into the plane. Equal components retain X/Y/Z order, making the
				// in-plane frame deterministic across reloads and tessellation.
				const std::array<gp_Dir, 3> aWorldAxes = {
					gp::DX(), gp::DY(), gp::DZ()};
				Standard_Integer aBestAxis = 0;
				Standard_Real aBestAlignment = std::abs(
					theWorldNormal.Dot(aWorldAxes[0]));
				for (Standard_Integer anAxis = 1; anAxis < 3; ++anAxis) {
					const Standard_Real anAlignment = std::abs(
						theWorldNormal.Dot(aWorldAxes[anAxis]));
					if (anAlignment < aBestAlignment) {
						aBestAlignment = anAlignment;
						aBestAxis = anAxis;
					}
				}
				gp_Vec anX(aWorldAxes[aBestAxis]);
				const gp_Vec aNormal(theWorldNormal);
				anX -= aNormal * anX.Dot(aNormal);
				if (!std::isfinite(anX.SquareMagnitude())
					|| anX.SquareMagnitude() <= gp::Resolution()) {
					return false;
				}
				theWorldXDirection = gp_Dir(anX);
				return IsFiniteDirection(theWorldXDirection)
					&& IsShapeWithinModelCoordinates(
						theFace, thePresentationTransform);
			} catch (...) {
				return false;
			}
		}

		bool SelectionModesMatch(
			const Handle(Core3DContext)& theContext,
			const Handle(AIS_InteractiveObject)& thePresentation,
			const std::vector<Standard_Integer>& theExpected) {
			if (theContext.IsNull() || thePresentation.IsNull()) {
				return false;
			}
			TColStd_ListOfInteger anActive;
			theContext->ActivatedModes(thePresentation, anActive);
			std::vector<Standard_Integer> anActual;
			anActual.reserve(static_cast<std::size_t>(anActive.Extent()));
			for (TColStd_ListIteratorOfListOfInteger anIterator(anActive);
				 anIterator.More(); anIterator.Next()) {
				anActual.push_back(anIterator.Value());
			}
			auto aSortedExpected = theExpected;
			std::sort(aSortedExpected.begin(), aSortedExpected.end());
			std::sort(anActual.begin(), anActual.end());
			return anActual == aSortedExpected;
		}
	}

    ObjectInteractor::ObjectInteractor(Handle(Core3DContext) context
                                       , Handle(Core3DView) view
                                       , Handle(OcctDocument) doc
                                       , Standard_ShortReal manipulatorSide)
        : Interactor(context, view, doc)
        , _manipulatorSide(manipulatorSide)
        , _booleanOpController(
            std::make_shared<BooleanOperationController>(context, doc))
        , _linearArrayController(
            std::make_shared<LinearArrayOperationController>(context, doc)) {
    }

    void ObjectInteractor::selectLastObject() {
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        Handle(AIS_InteractiveObject) object;
        for (AIS_ListIteratorOfListOfInteractive item(objects);
             item.More(); item.Next()) {
            if (myDoc->IsPresentationEditable(item.Value())
                && !myDoc->ShapeLabel(item.Value()).IsNull()) {
                object = item.Value();
            }
        }
        if (object.IsNull()) {
            return;
        }
        myContext->SetSelected(object, Standard_True);
        attachManipulator(object);
    }

    void ObjectInteractor::createManipulatorIfNeeded() {
        if(_manipulator.IsNull()) {
            _manipulator = new Core3DManipulator();
            _manipulator->SetSize(_manipulatorSide);
            _manipulator->SetZoomPersistence(Standard_True);
            _manipulator->SetModeActivationOnDetection(Standard_True);
        }
    }

    void ObjectInteractor::attachManipulator(Handle(AIS_InteractiveObject) toObject) {
		if (_manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray
			|| _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeShell) {
			// Parameter-panel tools capture immutable source leases in their
			// operation controllers and own no draggable viewport gizmo.
			return;
		}
        const TDF_Label aLabel = myDoc->ShapeLabel(toObject);
		const OcctGeometryRepresentation aRepresentation =
			myDoc->GeometryRepresentationForLabel(aLabel);
        if (toObject.IsNull()
            || !myDoc->IsPresentationEditable(toObject)
			|| aLabel.IsNull()
			|| !myDoc->IsEditableFreeSimpleDefinitionLabel(aLabel)
			|| !IsObjectModelingRepresentation(aRepresentation)
			|| (ManipulatorRequiresBRepModeling(_manipulatorType)
				&& !IsBRepModelingRepresentation(
					aRepresentation))) {
            return;
        }
        createManipulatorIfNeeded();
        _manipulator->Attach(toObject);
		_manipulatorSourceLabels[toObject.get()] = aLabel;
        myContext->UpdateCurrentViewer();
    }

	void ObjectInteractor::detachManipulator(Handle(AIS_InteractiveObject) fromObject) {
		if (!_manipulator.IsNull()) {
			if (_manipulatorGestureActive
				|| _manipulator->HasActiveTransformation()) {
				cancelInteraction();
			}
			_manipulator->Detach(fromObject);
			_manipulatorSourceLabels.erase(fromObject.get());
			myContext->UpdateCurrentViewer();
		}
	}

	void ObjectInteractor::detachManipulator(bool updateViewer) {
		if (!_manipulator.IsNull()) {
			if (_manipulatorGestureActive
				|| _manipulator->HasActiveTransformation()) {
				cancelInteraction();
			}
			_manipulator->Detach();
			_manipulatorSourceLabels.clear();
			if (updateViewer)
				myContext->UpdateCurrentViewer();
		}
	}

	void ObjectInteractor::	selectAll() {
		if (!_manipulator.IsNull()
			&& (_manipulatorGestureActive
				|| _manipulator->HasActiveTransformation())) {
			cancelInteraction();
		}
		AIS_ListOfInteractive objects;
		myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
		std::vector<Handle(AIS_InteractiveObject)> editableObjects;
		for (AIS_ListIteratorOfListOfInteractive iobject(objects);
		     iobject.More(); iobject.Next()) {
			const Handle(AIS_InteractiveObject)& object = iobject.Value();
			if (myDoc->IsPresentationEditable(object)
				&& !myDoc->ShapeLabel(object).IsNull()) {
				editableObjects.push_back(object);
			}
		}
		myContext->ClearSelected(Standard_False);
		if (editableObjects.empty()) {
			detachManipulator(false);
			myContext->UpdateCurrentViewer();
			return;
		}
		std::vector<std::pair<Handle(AIS_InteractiveObject), TDF_Label>>
			attachableObjects;
		attachableObjects.reserve(editableObjects.size());
		bool canAttachAll = true;
		for (const Handle(AIS_InteractiveObject)& object : editableObjects) {
			myContext->AddSelect(object);
			myContext->HilightSelected(Standard_False);
			const TDF_Label label = myDoc->ShapeLabel(object);
			const OcctGeometryRepresentation representation =
				myDoc->GeometryRepresentationForLabel(label);
			if (label.IsNull()
				|| !myDoc->IsEditableFreeSimpleDefinitionLabel(label)
				|| representation == OcctGeometryRepresentation::Invalid
				|| (ManipulatorRequiresBRepModeling(_manipulatorType)
					&& !IsBRepModelingRepresentation(representation))) {
				canAttachAll = false;
				continue;
			}
			attachableObjects.push_back({object, label});
		}
		if (!canAttachAll
			|| attachableObjects.size() != editableObjects.size()) {
			detachManipulator(false);
			myContext->UpdateCurrentViewer();
			return;
		}
		createManipulatorIfNeeded();
		Handle(Core3DManipulatorObjectSequence) sequence =
			new Core3DManipulatorObjectSequence();
		for (const auto& object : attachableObjects) {
			sequence->Append(object.first);
		}
		_manipulator->Attach(sequence);
		_manipulatorSourceLabels.clear();
		for (const auto& object : attachableObjects) {
			_manipulatorSourceLabels.emplace(
				object.first.get(), object.second);
		}
		myContext->UpdateCurrentViewer();
	}

	Handle(TopLoc_Datum3D) ObjectInteractor::manipulatorTransform() {
		if (!_manipulator.IsNull())
			return _manipulator->Transform();
		else
			return nullptr;
	}

	gp_XYZ ObjectInteractor::manipulatorPosition() {
		if (!_manipulator.IsNull())
			return _manipulator->Position().Location().Coord();
		else
			return gp_XYZ();
	}

    PresentationOverlayCaptureStatus
    ObjectInteractor::captureIdlePresentationOverlay(
        scene::PresentationOverlayContent& theContent,
        std::vector<Handle(AIS_Shape)>& theMirrorPreviewObjects,
        BooleanPreviewCapture& theBooleanPreview) const noexcept {
        try {
            theContent = {};
            theMirrorPreviewObjects.clear();
            theBooleanPreview = {};
            const bool hasMirrorPreview = !_trialMirrorObjects.empty();
			// The current renderer-neutral MirrorPreview contract has exactly
			// six gizmo slots followed by N mirrored result bodies, all with the
			// MirrorPreview role. A picked reference face is a distinct semantic
			// object and cannot be appended truthfully without a schema change.
			// Retain OCCT while picking/custom preview is visible instead of
			// publishing a mislabeled Metal payload.
			if (_mirrorPlanePicking || _customMirrorPlane.has_value()
				|| !_mirrorReferencePresentations.empty()
				|| _trialMirrorUsesCustomPlane) {
				return PresentationOverlayCaptureStatus::Unsafe;
			}
            // Cleanup residue is deliberately not publishable. Retaining OCCT
            // prevents an unresolved presentation from being omitted.
            if (hasMirrorPreview
                && (!_trialMirrorObjectsValid
					|| _mirrorPreviewState != MirrorPreviewState::Ready
                    || _trialMirrorObjects.size() > kMaxMirrorPreviewBodies)) {
                return PresentationOverlayCaptureStatus::Unsafe;
            }
			if (_mirrorOwnsDocumentCommand
				|| !_pendingMirrorResults.empty()
				|| _mirrorPreviewState == MirrorPreviewState::Committing
				|| _mirrorPreviewState == MirrorPreviewState::OutcomeUnknown
				|| (_mirrorPreviewState == MirrorPreviewState::Failed
					&& hasUnresolvedMirrorObjects())) {
				return PresentationOverlayCaptureStatus::Unsafe;
			}
            if (_booleanOpController->hasUnresolvedState()) {
                return PresentationOverlayCaptureStatus::Unsafe;
            }
            const bool isSubtract = _manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract;
            const bool isUnion = _manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeUnion;
            const bool isIntersect = _manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeIntersect;
            if (isSubtract || isUnion || isIntersect) {
                if (hasMirrorPreview) {
                    return PresentationOverlayCaptureStatus::Unsafe;
                }
                if (!_booleanOpController->hasSelectionState()) {
                    return PresentationOverlayCaptureStatus::Available;
                }
                if (!_booleanOpController->capturePreview(theBooleanPreview)
                    || (isSubtract
                        && theBooleanPreview.action
                            != BooleanAction::BooleanSubtract)
                    || (isUnion
                        && theBooleanPreview.action
                            != BooleanAction::BooleanUnion)
                    || (isIntersect
                        && theBooleanPreview.action
                            != BooleanAction::BooleanIntersect)) {
                    theBooleanPreview = {};
                    return PresentationOverlayCaptureStatus::Unsafe;
                }
                return PresentationOverlayCaptureStatus::Available;
            }
            if (_booleanOpController->hasActiveOperation()
                || _booleanOpController->hasSelectionState()) {
                return PresentationOverlayCaptureStatus::Unsafe;
            }
            if (_manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeNone) {
                return hasMirrorPreview
                    ? PresentationOverlayCaptureStatus::Unsafe
                    : PresentationOverlayCaptureStatus::Available;
            }
            const bool isMoveRotate = _manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate;
            const bool isScale = _manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeScale;
            const bool isMirror = _manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeMirror;
            if (!isMoveRotate && !isScale && !isMirror) {
                return PresentationOverlayCaptureStatus::Unsafe;
            }
            if (_manipulator.IsNull() || !_manipulator->IsAttached()) {
                return hasMirrorPreview
                    ? PresentationOverlayCaptureStatus::Unsafe
                    : PresentationOverlayCaptureStatus::Available;
            }
            if (myContext.IsNull()
                || !_manipulator->IsInstance(
                    STANDARD_TYPE(Core3DManipulator))
                || !myContext->IsDisplayed(_manipulator)
                || !_manipulator->ZoomPersistence()
                || _manipulator->HasActiveMode()
                || _manipulator->HasActiveTransformation()) {
                theContent = {};
                return PresentationOverlayCaptureStatus::Unsafe;
            }
            const Standard_Boolean didCapture = isMoveRotate
                ? _manipulator->CaptureIdleMoveRotateOverlay(theContent)
                : isScale
                    ? _manipulator->CaptureIdleScaleOverlay(theContent)
                    : _manipulator->CaptureIdleMirrorOverlay(theContent);
            if (!didCapture) {
                theContent = {};
                return PresentationOverlayCaptureStatus::Unsafe;
            }
            if (hasMirrorPreview) {
                if (!isMirror) {
                    theContent = {};
                    return PresentationOverlayCaptureStatus::Unsafe;
                }
                for (const Handle(AIS_Shape)& aShape : _trialMirrorObjects) {
                    if (aShape.IsNull() || aShape->Shape().IsNull()
                        || !myContext->IsDisplayed(aShape)) {
                        theContent = {};
                        return PresentationOverlayCaptureStatus::Unsafe;
                    }
                }
                theContent.kind =
                    scene::PresentationOverlayKind::MirrorPreview;
                theMirrorPreviewObjects = _trialMirrorObjects;
            }
            return PresentationOverlayCaptureStatus::Available;
        } catch (...) {
            theContent = {};
            theMirrorPreviewObjects.clear();
            theBooleanPreview = {};
            return PresentationOverlayCaptureStatus::Unsafe;
        }
    }

    void ObjectInteractor::deleteSelected() {
        if (_manipulator.IsNull() || !_manipulator->IsAttached()) { return; }
		auto doc = myDoc->ChangeDocument();
		if (doc.IsNull() || doc->HasOpenCommand()) { return; }

        Handle(Core3DManipulatorObjectSequence) objects = _manipulator->Objects();
		std::vector<std::pair<Handle(AIS_InteractiveObject), TDF_Label>> removals;
        for (Core3DManipulatorObjectSequence::Iterator it(*objects); it.More(); it.Next()) {
			const TDF_Label label = myDoc->ShapeLabel(it.Value());
			if (label.IsNull()
				|| !myDoc->IsPresentationEditable(it.Value())
				|| !myDoc->IsEditableFreeSimpleDefinitionLabel(label)
				|| myDoc->GeometryRepresentationForLabel(label)
					== OcctGeometryRepresentation::Invalid) {
				return;
			}
			for (const auto& removal : removals) {
				if (removal.second.IsEqual(label)) { return; }
			}
			removals.push_back({it.Value(), label});
        }
		if (removals.empty()) { return; }

		try {
			doc->NewCommand();
			for (const auto& removal : removals) {
				if (!myDoc->RemoveShape(removal.second)) {
					doc->AbortCommand();
					return;
				}
			}
			if (!doc->CommitCommand()) {
				if (doc->HasOpenCommand()) { doc->AbortCommand(); }
				return;
			}
		} catch (...) {
			if (doc->HasOpenCommand()) { doc->AbortCommand(); }
			return;
		}

		detachManipulator(false);
		for (const auto& removal : removals) {
			myContext->Remove(removal.first, Standard_False);
		}
		myDoc->NotifyChanges();
        myContext->UpdateCurrentViewer();
    }

    void ObjectInteractor::duplicateSelected() {
        auto doc = myDoc->ChangeDocument();
		if (doc.IsNull() || doc->HasOpenCommand()) { return; }
        
        struct DuplicateSource {
            Handle(AIS_Shape) presentation;
            TDF_Label label;
			OcctGeometryRepresentation representation;
        };
        std::vector<DuplicateSource> sources;
        for (myContext->InitSelected(); myContext->MoreSelected();
             myContext->NextSelected()) {
            const Handle(AIS_InteractiveObject) selected =
                myContext->SelectedInteractive();
            const Handle(AIS_Shape) shape =
                Handle(AIS_Shape)::DownCast(selected);
            const TDF_Label label = myDoc->ShapeLabel(selected);
            const OcctGeometryRepresentation representation =
                myDoc->GeometryRepresentationForLabel(label);
			const OcctGeometryRepresentation destinationRepresentation =
				representation
					== OcctGeometryRepresentation::LegacyUnknown
				? OcctGeometryRepresentation::BRep
				: representation;
            const TopoDS_Shape storedShape = label.IsNull()
                ? TopoDS_Shape()
                : XCAFDoc_ShapeTool::GetShape(label);
            if (selected.IsNull() || shape.IsNull()
                || shape->Shape().IsNull()
                || !myDoc->IsPresentationEditable(selected)
                || label.IsNull()
                || !myDoc->IsEditableFreeSimpleDefinitionLabel(label)
				|| !IsObjectModelingRepresentation(representation)
				|| (destinationRepresentation
						!= OcctGeometryRepresentation::BRep
					&& destinationRepresentation
						!= OcctGeometryRepresentation::TriangleMesh)
                || storedShape.IsNull()
                || !storedShape.IsEqual(shape->Shape())) {
                return;
            }
            bool isDuplicateLabel = false;
            for (const DuplicateSource& source : sources) {
                if (source.label.IsEqual(label)) {
                    isDuplicateLabel = true;
                    break;
                }
            }
            if (!isDuplicateLabel) {
				sources.push_back({
					shape, label, destinationRepresentation});
            }
        }
        if (sources.empty()) { return; }
		std::vector<TDF_Label> sourceLabels;
		sourceLabels.reserve(sources.size());
		for (const DuplicateSource& source : sources) {
			sourceLabels.push_back(source.label);
		}
		if (!myDoc->CanDuplicateGeometryDefinitions(sourceLabels)) {
			return;
		}

        Bnd_Box overallBox;
        for (const DuplicateSource& source : sources) {
            Bnd_Box b;
            source.presentation->BoundingBox(b);
            if (b.IsVoid()) { return; }
            if(overallBox.IsVoid()) {
                overallBox = b;
            } else {
                overallBox.Add(b);
            }
        }

		if (overallBox.IsVoid()) { return; }
		
        Standard_Real xmin, xmax, ymin, ymax, zmin, zmax;
        overallBox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        const Standard_Real w = xmax - xmin;
        const Standard_Real d = ymax - ymin;
		const Standard_Real h = zmax - zmin;
		if (!std::isfinite(w) || !std::isfinite(d) || !std::isfinite(h)
			|| w < 0.0 || d < 0.0 || h < 0.0) {
			return;
		}
        
        gp_Trsf minAxisDisplacement;
		const Standard_Real aMinimumExtent = Precision::Confusion();
		gp_Vec aDisplacement;
		if (w > aMinimumExtent && d > aMinimumExtent) {
			aDisplacement = w > d
				? gp_Vec(0.0, d, 0.0)
				: gp_Vec(w, 0.0, 0.0);
		} else if (w > aMinimumExtent) {
			aDisplacement = gp_Vec(w, 0.0, 0.0);
		} else if (d > aMinimumExtent) {
			aDisplacement = gp_Vec(0.0, d, 0.0);
		} else if (h > aMinimumExtent) {
			aDisplacement = gp_Vec(0.0, 0.0, h);
		} else {
			// Keep a point-sized legacy BRep visibly separate without risking
			// the document coordinate bound.
			aDisplacement = gp_Vec(10.0, 0.0, 0.0);
		}
		minAxisDisplacement.SetTranslation(aDisplacement);
        
		struct DuplicateRecord {
			Handle(AIS_Shape) presentation;
			TDF_Label sourceLabel;
			TDF_Label resultLabel;
			OcctGeometryRepresentation representation;
		};
		std::vector<DuplicateRecord> duplicates;
		try {
			for (const DuplicateSource& source : sources) {
				BRepBuilderAPI_Copy shapeCopy;
				shapeCopy.Perform(
					source.presentation->Shape(),
					Standard_True,
					source.representation
						== OcctGeometryRepresentation::TriangleMesh
					? Standard_True
					: Standard_False);
				if (!shapeCopy.IsDone() || shapeCopy.Shape().IsNull()
					|| shapeCopy.Shape().IsPartner(
						source.presentation->Shape())
					|| (source.representation
							== OcctGeometryRepresentation::BRep
						&& !IsTopologicallyValid(shapeCopy.Shape()))
					|| (source.representation
							== OcctGeometryRepresentation::TriangleMesh
						&& !RestoreTriangleMeshCopyParameters(
							source.presentation->Shape(), shapeCopy))) {
					return;
				}
				Handle(AIS_Shape) copy = new AIS_Shape(shapeCopy.Shape());
				copy->SetLocalTransformation(
					source.presentation->LocalTransformation().Multiplied(
						minAxisDisplacement));
				myDoc->LoadObjectMeterial(source.label, copy);
				duplicates.push_back({
					copy,
					source.label,
					TDF_Label(),
					source.representation});
			}
		} catch (...) {
			return;
		}
		if (duplicates.empty()) { return; }

		try {
			doc->NewCommand();
			if (!doc->HasOpenCommand()) { return; }
			for (auto& duplicate : duplicates) {
				const TDF_Label label = myDoc->AddShape(
					duplicate.presentation,
					duplicate.representation);
				if (label.IsNull()) {
					doc->AbortCommand();
					return;
				}
				if (!myDoc->CopyGeometryRepresentation(
						duplicate.sourceLabel, label)
					|| !myDoc->CopyObjectAppearance(
						duplicate.sourceLabel, label)) {
					doc->AbortCommand();
					return;
				}
				duplicate.resultLabel = label;
				myDoc->LoadObjectMeterial(label, duplicate.presentation);
			}
			if (!myDoc->ValidateGeometryRepresentations()) {
				doc->AbortCommand();
				return;
			}
			if (!doc->CommitCommand()) {
				if (doc->HasOpenCommand()) { doc->AbortCommand(); }
				return;
			}
		} catch (...) {
			if (doc->HasOpenCommand()) { doc->AbortCommand(); }
			return;
		}

		detachManipulator(false);
		myContext->ClearSelected(Standard_False);
		for (const auto& duplicate : duplicates) {
			myContext->Display(duplicate.presentation, AIS_Shaded, 0, Standard_False);
			myContext->AddSelect(duplicate.presentation);
			_manipulator->Attach(duplicate.presentation);
			_manipulatorSourceLabels[duplicate.presentation.get()] =
				duplicate.resultLabel;
		}
		myContext->HilightSelected(Standard_True);
		myDoc->NotifyChanges();
		_manipulator->Redisplay();
		myContext->UpdateCurrentViewer();
    }

    const bool ObjectInteractor::isSelected() const {
//        printf(">>> ObjectInteractor::isSelected - %d\n", !myContext->FirstSelectedObject().IsNull());
        return !myContext->FirstSelectedObject().IsNull();
    }

    void ObjectInteractor::attachManipulatorToSelection(bool detach) {
        if (_manipulatorType == PrimitiveManipulatorType::PrimitiveGizmoTypeNone) { return; }
		if (_manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray
			|| _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeShell) {
			detachManipulator(false);
			return;
		}
		if (ManipulatorRequiresBRepModeling(_manipulatorType)
			&& !SelectionSupportsBRepModeling(myContext, myDoc)) {
			detachManipulator(false);
			return;
		}

		if (detach) {
			myContext->InitDetected();
			if (myContext->MoreDetected()) {
				auto detected = myContext->DetectedInteractive();
                if (!_manipulator.IsNull() && !_manipulator->Objects().IsNull()) {
                    if (_manipulator->Objects()->Size() > 1) {
                        detachManipulator(detected);
                        return;
                    }
                }
			}
		}
		
		Handle(AIS_InteractiveObject) selected;
		bool hasUnsafeSelection = false;
        for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected())
        {
            selected = myContext->SelectedInteractive();
			if (selected.IsNull()
				|| !myDoc->IsPresentationEditable(selected)
				|| myDoc->ShapeLabel(selected).IsNull()) {
				hasUnsafeSelection = true;
				break;
			}
        }
		if (hasUnsafeSelection) {
			myContext->ClearSelected(Standard_False);
			detachManipulator(false);
			myContext->UpdateCurrentViewer();
			return;
		}
        
        if(selected.IsNull()) {
            if(!_manipulator.IsNull()) {
                detachManipulator(false);
                myContext->UpdateCurrentViewer();
            }
            return;
        }
        
        attachManipulator(selected);
		return;
    }

	void ObjectInteractor::setObjectTransparent(Handle(AIS_InteractiveObject) selected, const bool on) {
		
		Quantity_Color color = Quantity_Color(on ? Quantity_NameOfColor::Quantity_NOC_BLUE : Quantity_NameOfColor::Quantity_NOC_GRAY80);
		
		Graphic3d_MaterialAspect mat = selected->Material();
		mat.SetAlpha(on ? 0.4 : 1.0);
		selected->SetMaterial(mat);
		selected->SetColor(color);
		myContext->SetMaterial(selected, mat, Standard_True);
	}

	void ObjectInteractor::setManipulatorType(PrimitiveManipulatorType type) {
		const PrimitiveManipulatorType aPreviousType = _manipulatorType;
		if (type
				!= PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray
			&& hasActiveLinearArray()
			&& !cancelLinearArray()) {
			// A closed/unknown OCAF outcome is an exactly-once recovery token.
			// Never hide it by allowing another tool to replace the retained UI.
			return;
		}
		if (type != PrimitiveManipulatorType::PrimitiveGizmoTypeMirror
			&& hasActiveMirror()
			&& !cancelMirror()) {
			// Unresolved Mirror ownership is a hard transition barrier. Keeping
			// the old manipulator type makes the existing GL caller's post-set
			// equality check fail closed instead of hiding recovery controls.
			return;
		}
		if (!_manipulator.IsNull()
			&& (_manipulatorGestureActive
				|| _manipulator->HasActiveTransformation())) {
			cancelInteraction();
		}
		if (ManipulatorRequiresBRepModeling(type)
			&& !SelectionSupportsBRepModeling(myContext, myDoc)
			&& (!ManipulatorAllowsEmptySelection(type)
				|| !SelectionIsEmpty(myContext))) {
			type = PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
		}
		_manipulatorType = type;
		if (_manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray
			&& (aPreviousType
					!= PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray
					|| !hasActiveLinearArray())
				&& !beginLinearArray()) {
				// Admission failures leave no retained operation and may safely
				// fall back to None. A preview/display failure can retain owned AIS
				// presentations for recovery; keep Array selected so its recovery
				// controls and transition barrier remain visible.
				if (!hasActiveLinearArray()) {
					_manipulatorType =
						PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
					type = PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
				}
			}
		if (_manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeMirror
			&& aPreviousType
				!= PrimitiveManipulatorType::PrimitiveGizmoTypeMirror) {
			_mirrorPreviewState = MirrorPreviewState::Selecting;
			++_mirrorPreviewGeneration;
		} else if (_manipulatorType
				!= PrimitiveManipulatorType::PrimitiveGizmoTypeMirror
			&& !hasUnresolvedMirrorObjects()
			&& _pendingMirrorResults.empty()) {
			_mirrorPreviewState = MirrorPreviewState::Unavailable;
		}
        createManipulatorIfNeeded();
        bool scale = type == PrimitiveManipulatorType::PrimitiveGizmoTypeScale;
        bool movRot = type == PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate;
        bool mirror = type == PrimitiveManipulatorType::PrimitiveGizmoTypeMirror;
        for(auto axis = 0; axis < 3; ++axis) {
            _manipulator->SetPart (axis, AIS_ManipulatorMode::AIS_MM_Scaling, scale);
			_manipulator->SetPart (axis, AIS_ManipulatorMode::AIS_MM_ScalingUniform, scale);
            _manipulator->SetPart (axis, AIS_ManipulatorMode::AIS_MM_Translation, movRot && !mirror);
            _manipulator->SetPart (axis, AIS_ManipulatorMode::AIS_MM_Rotation, movRot && !mirror);
            _manipulator->SetPart (axis, AIS_ManipulatorMode::AIS_MM_TranslationPlane, Standard_False);
			_manipulator->SetPart (axis, AIS_ManipulatorMode::AIS_MM_MirroringPlanePos, mirror);
			_manipulator->SetPart (axis, AIS_ManipulatorMode::AIS_MM_MirroringPlaneNeg, mirror);
        }
        
        auto objects = _manipulator->Objects();
		if(!objects.IsNull() && objects->Size() > 0) {
			std::vector<std::pair<
				Handle(AIS_InteractiveObject), TDF_Label>> sources;
			sources.reserve(static_cast<std::size_t>(objects->Size()));
			bool canReattach = true;
			const auto& cachedShapes = _manipulator->cachedShapes();
			for (Core3DManipulatorObjectSequence::Iterator object(*objects);
				 object.More(); object.Next()) {
				const Handle(AIS_InteractiveObject)& presentation =
					object.Value();
				const TDF_Label label = myDoc->ShapeLabel(presentation);
				const auto cached = cachedShapes.find(presentation);
				const TopoDS_Shape stored = label.IsNull()
					? TopoDS_Shape()
					: XCAFDoc_ShapeTool::GetShape(label);
				const OcctGeometryRepresentation representation =
					myDoc->GeometryRepresentationForLabel(label);
				if (presentation.IsNull()
					|| !myDoc->IsPresentationEditable(presentation)
					|| label.IsNull()
					|| !myDoc->IsEditableFreeSimpleDefinitionLabel(label)
					|| cached == cachedShapes.end()
					|| cached->second.IsNull()
					|| stored.IsNull()
					|| !stored.IsEqual(cached->second)
					|| representation
						== OcctGeometryRepresentation::Invalid
					|| (ManipulatorRequiresBRepModeling(_manipulatorType)
						&& !IsBRepModelingRepresentation(
							representation))) {
					canReattach = false;
					break;
				}
				sources.push_back({presentation, label});
			}
            if (_manipulator->IsAttached()) {
                _manipulator->Detach();
            }
			_manipulatorSourceLabels.clear();
			if (_manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer
				&& _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeExtrude
				&& _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray
				&& _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeShell
				&& _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeNone
				&& canReattach
				&& sources.size()
					== static_cast<std::size_t>(objects->Size())) {
				_manipulator->Attach(objects);
				for (const auto& source : sources) {
					_manipulatorSourceLabels.emplace(
						source.first.get(), source.second);
				}
            }
        }

		_manipulator->Redisplay();

        if (type == PrimitiveManipulatorType::PrimitiveGizmoTypeNone
			|| type
				== PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray
			|| type
				== PrimitiveManipulatorType::PrimitiveGizmoTypeShell) { // remove gizmo when the operation owns only AIS previews
            _manipulator->DeactivateCurrentMode();
            myContext->Remove(_manipulator, Standard_True);
        }

        myContext->UpdateCurrentViewer();
    }

    bool ObjectInteractor::transformManipulator(const int theX, const int theY) {
		if (ManipulatorRequiresBRepModeling(_manipulatorType)
			&& !ManipulatorObjectsSupportBRepModeling(
				_manipulator,
				myDoc,
				_manipulatorType,
				_manipulatorSourceLabels)) {
			cancelInteraction();
			return false;
		}
        if(!_manipulator.IsNull() && _manipulator->IsAttached()) {
            if(_manipulator->HasActiveMode()) {
                _manipulator->Transform(theX, theY, myView, myContext);
                myContext->UpdateCurrentViewer();
                return true;
            }
        }
        return false;
    }

    bool ObjectInteractor::startTransformManipulator(const int theX, const int theY) {
		if (ManipulatorRequiresBRepModeling(_manipulatorType)
			&& !ManipulatorObjectsSupportBRepModeling(
				_manipulator,
				myDoc,
				_manipulatorType,
				_manipulatorSourceLabels)) {
			cancelInteraction();
			return false;
		}
        if(!_manipulator.IsNull() && _manipulator->IsAttached()) {
            myContext->MoveTo(theX, theY, myView, Standard_False);
            myContext->UpdateCurrentViewer();
            if(_manipulator->HasActiveMode()) {
                _manipulator->StartTransform(theX, theY, myView, myContext);
				_manipulatorGestureActive = true;
                return true;
            }
        }
        return false;
    }

    void ObjectInteractor::finishInteraction() {
		struct GestureStateReset final {
			bool& state;
			~GestureStateReset() noexcept { state = false; }
		} aGestureStateReset{_manipulatorGestureActive};
		if (ManipulatorRequiresBRepModeling(_manipulatorType)
			&& !ManipulatorObjectsSupportBRepModeling(
				_manipulator,
				myDoc,
				_manipulatorType,
				_manipulatorSourceLabels)) {
			cancelInteraction();
			return;
		}
        if(!_manipulator.IsNull() && _manipulator->IsAttached() && _manipulator->HasActiveMode()) {
			const AIS_ManipulatorMode activeMode = _manipulator->ActiveMode();
			const bool isMirrorPlane = _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeMirror
				&& (activeMode
						== AIS_ManipulatorMode::AIS_MM_MirroringPlaneNeg
					|| activeMode
						== AIS_ManipulatorMode::AIS_MM_MirroringPlanePos);
			if (isMirrorPlane) {
				// Mirror handles select an operation plane; they intentionally do
				// not mutate the source presentation. Resolve the transient preview
				// before the transform-only no-op guard below.
				_manipulator->StopTransform(Standard_False);
				tryMirror(
					_manipulator->ActiveAxisIndex(),
					activeMode
						== AIS_ManipulatorMode::AIS_MM_MirroringPlaneNeg);
				_manipulator->DeactivateCurrentMode();
				myContext->ClearDetected(Standard_False);
				_manipulator->Redisplay();
				myContext->UpdateCurrentViewer();
				return;
			}

			if (!_manipulator->HasActiveTransformation()) {
				cancelInteraction();
				return;
			}
			const auto &cachedShapes = _manipulator->cachedShapes();
			Handle(Core3DManipulatorObjectSequence) objects =
				_manipulator->Objects();
			if (objects.IsNull()) {
				cancelInteraction();
				return;
			}
			bool didChange = false;
			Standard_Integer objectIndex = 1;
			for (Core3DManipulatorObjectSequence::Iterator object(*objects);
			     object.More(); object.Next(), ++objectIndex) {
				const Handle(AIS_InteractiveObject)& current = object.Value();
				const auto cached = cachedShapes.find(current);
				const Handle(AIS_Shape) presentation =
					Handle(AIS_Shape)::DownCast(current);
				if (cached == cachedShapes.end()
				    || cached->second.IsNull()
				    || presentation.IsNull()
				    || presentation->Shape().IsNull()) {
					cancelInteraction();
					return;
				}
				didChange = didChange
					|| TransformDiffers(
						presentation->LocalTransformation(),
						_manipulator->StartTransformation(objectIndex))
					|| !presentation->Shape().IsEqual(cached->second);
			}
			if (!didChange) {
				cancelInteraction();
				return;
			}

            auto doc = myDoc->ChangeDocument();
			if (doc.IsNull() || doc->HasOpenCommand()) {
				cancelInteraction();
				return;
			}

			std::vector<std::pair<Handle(AIS_Shape), TDF_Label>> changes;
			for (const auto& cachedShape : cachedShapes) {
				Handle(AIS_Shape) presentation = Handle(AIS_Shape)::DownCast(cachedShape.first);
				const auto sourceLabel = _manipulatorSourceLabels.find(
					cachedShape.first.get());
				const TDF_Label label =
					sourceLabel == _manipulatorSourceLabels.end()
					? TDF_Label()
					: sourceLabel->second;
				const TopoDS_Shape storedShape = label.IsNull()
					? TopoDS_Shape()
					: XCAFDoc_ShapeTool::GetShape(label);
				const OcctGeometryRepresentation representation =
					myDoc->GeometryRepresentationForLabel(label);
				const bool isSourceShapeCurrent =
					!presentation.IsNull()
					&& !presentation->Shape().IsNull()
					&& presentation->Shape().IsEqual(cachedShape.second);
				const TDF_Label currentLabel =
					myDoc->ShapeLabel(presentation);
				if (presentation.IsNull()
					|| presentation->Shape().IsNull()
					|| !myDoc->IsPresentationEditable(presentation)
					|| label.IsNull()
					|| label.Data() != doc->GetData()
					|| storedShape.IsNull()
					|| !storedShape.IsEqual(cachedShape.second)
					|| (!isSourceShapeCurrent
						&& _manipulatorType
							!= PrimitiveManipulatorType::PrimitiveGizmoTypeScale)
					|| (!currentLabel.IsNull()
						&& !currentLabel.IsEqual(label))
					|| (currentLabel.IsNull() && isSourceShapeCurrent)
					|| !myDoc->IsEditableFreeSimpleDefinitionLabel(label)
					|| !IsObjectModelingRepresentation(representation)
					|| (_manipulatorType
							== PrimitiveManipulatorType::PrimitiveGizmoTypeScale
						&& !IsBRepModelingRepresentation(
							representation))
					|| (_manipulatorType == PrimitiveManipulatorType::PrimitiveGizmoTypeScale
						&& !IsTopologicallyValid(presentation->Shape()))) {
					cancelInteraction();
					return;
				}
				changes.push_back({presentation, label});
			}
			if (changes.empty()) {
				cancelInteraction();
				return;
			}

			try {
				doc->NewCommand();
				for (const auto& change : changes) {
					if (_manipulatorType == PrimitiveManipulatorType::PrimitiveGizmoTypeScale) {
						if (!myDoc->ReplaceShape(change.second, change.first)) {
							throw Standard_Failure(
								"Unable to replace scaled geometry");
						}
					}
					myDoc->SaveObjectTransform(change.second, change.first);
				}
				if (!doc->CommitCommand()) {
					if (doc->HasOpenCommand()) { doc->AbortCommand(); }
					cancelInteraction();
					return;
				}
			} catch (...) {
				if (doc->HasOpenCommand()) { doc->AbortCommand(); }
				cancelInteraction();
				return;
			}

			_manipulator->StopTransform(Standard_True);
			myDoc->NotifyChanges();

			_manipulator->UpdateCachedShapes();
            
            _manipulator->DeactivateCurrentMode();
            // A touch release has no continuing pointer hover. OCCT otherwise
            // retains the last detected handle and can immediately reactivate
            // its mode during redisplay, leaving the idle presentation in an
            // unsafe pseudo-active state after a successful transform.
            myContext->ClearDetected(Standard_False);
            _manipulator->Redisplay();
            myContext->UpdateCurrentViewer();
        }
    }

    void ObjectInteractor::cancelInteraction() {
		_manipulatorGestureActive = false;
        if(!_manipulator.IsNull() && _manipulator->IsAttached()) {
			_manipulator->StopTransform(Standard_False);
			for (const auto& cachedShape : _manipulator->cachedShapes()) {
				Handle(AIS_Shape) presentation = Handle(AIS_Shape)::DownCast(cachedShape.first);
				if (!presentation.IsNull() && !cachedShape.second.IsNull()) {
					presentation->SetShape(cachedShape.second);
					myContext->Redisplay(presentation, Standard_False);
				}
            }
            _manipulator->DeactivateCurrentMode();
			myContext->ClearDetected(Standard_False);
			_manipulator->UpdateCachedShapes();
            _manipulator->Redisplay();
            myContext->UpdateCurrentViewer();
        }
    }

    const bool ObjectInteractor::isManipulatorAttached() const {
        return !_manipulator.IsNull() && _manipulator->IsAttached();
    }

    const bool ObjectInteractor::isManipulatorGestureActive() const {
        return _manipulatorGestureActive;
    }

    const bool ObjectInteractor::isManipulatorInteractionActive() const {
		return !_mirrorPlanePicking
			&& !_manipulator.IsNull()
            && _manipulator->IsAttached()
            && _manipulator->HasActiveMode();
    }

    void ObjectInteractor::SelectAndAttachManipulator(Handle(AIS_InteractiveObject) toObject) {
        if (toObject.IsNull()
            || !myDoc->IsPresentationEditable(toObject)
            || myDoc->ShapeLabel(toObject).IsNull()) {
            return;
        }
        myContext->SetSelected(toObject, Standard_True);
        attachManipulatorToSelection();
    }

    const PrimitiveManipulatorType ObjectInteractor::getManipulatorType() const {
        return _manipulatorType;
    }

    bool ObjectInteractor::publishCommittedInspectorTransform(
        const Handle(AIS_Shape)& thePresentation,
        const gp_Trsf& theTransform) noexcept
    {
        try {
            OCC_CATCH_SIGNALS
            if (thePresentation.IsNull()
                || thePresentation->Shape().IsNull()
                || myContext.IsNull() || myDoc.IsNull()
                || !myDoc->IsPresentationEditable(thePresentation)) {
                return false;
            }
            const TDF_Label aLabel = myDoc->ShapeLabel(thePresentation);
            const TopoDS_Shape aStoredShape = aLabel.IsNull()
                ? TopoDS_Shape()
                : XCAFDoc_ShapeTool::GetShape(aLabel);
            const bool wasManipulatorAttached =
                !_manipulator.IsNull() && _manipulator->IsAttached();
            if (wasManipulatorAttached) {
                const auto objects = _manipulator->Objects();
                if (objects.IsNull() || objects->Size() != 1
                    || objects->First() != thePresentation) {
                    return false;
                }
            }
            if (aLabel.IsNull()
                || !myDoc->IsEditableFreeSimpleDefinitionLabel(aLabel)
                || aStoredShape.IsNull()
                || !aStoredShape.IsEqual(thePresentation->Shape())
                || _manipulatorGestureActive
                || (!_manipulator.IsNull()
                    && _manipulator->HasActiveTransformation())) {
                return false;
            }

            thePresentation->SetLocalTransformation(theTransform);
            myContext->Redisplay(thePresentation, Standard_False);
            if (!_manipulator.IsNull() && _manipulator->IsAttached()) {
                // Reusing the existing type follows the normal detach/reattach
                // path, which recomputes the gizmo origin and refreshes cached
                // source shapes without appending a duplicate owner.
                setManipulatorType(_manipulatorType);
            }
            if (wasManipulatorAttached) {
                if (_manipulator.IsNull()
                    || !_manipulator->IsAttached()) {
                    return false;
                }
                const auto objects = _manipulator->Objects();
                if (objects.IsNull() || objects->Size() != 1
                    || objects->First() != thePresentation) {
                    return false;
                }
            }
            if (TransformDiffers(
                    thePresentation->LocalTransformation(),
                    theTransform)) {
                return false;
            }
            myContext->UpdateCurrentViewer();
            return true;
        } catch (...) {
            return false;
        }
    }

	void ObjectInteractor::fillSelectedState(Standard_Boolean forceActor, BooleanAction action) {
		const auto failClosed = [this]() noexcept {
			_booleanOpController->cancelActive();
			try {
				if (!myContext.IsNull()) {
					myContext->ClearSelected(Standard_True);
				}
			} catch (...) {
			}
		};

		try {
			std::array<Handle(AIS_InteractiveObject),
				BooleanOperationController::kMaxSourceOperands> selectedObjects;
			std::size_t selectedCount = 0;
			for (myContext->InitSelected();
				 myContext->MoreSelected();
				 myContext->NextSelected()) {
				if (selectedCount >= selectedObjects.size()) {
					failClosed();
					return;
				}
				selectedObjects[selectedCount++] =
					myContext->SelectedInteractive();
			}
			for (std::size_t index = 0; index < selectedCount; ++index) {
				_booleanOpController->updateDetectedState(
					selectedObjects[index],
					Handle(SelectMgr_EntityOwner)(),
					forceActor,
					action);
				if (forceActor) {
					forceActor = false;
				}
			}
			(void)_booleanOpController->visualApply(action);
		} catch (...) {
			failClosed();
		}
	}

	void ObjectInteractor::updateDetectedState(Standard_Boolean forceActor, BooleanAction action) {
		if (_booleanOpController->isSelectionFrozen()) {
			return;
		}
		std::size_t selectedCount = 0;
		for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
			++selectedCount;
		}
		if (selectedCount > BooleanOperationController::kMaxSourceOperands) {
			_booleanOpController->cancelActive();
			myContext->ClearSelected(Standard_True);
			return;
		}
		if (myContext->HasDetected()) {
			_booleanOpController->updateDetectedState(myContext->DetectedInteractive(), myContext->DetectedOwner(), forceActor, action);
			(void)_booleanOpController->visualApply(action);
		}
	}

	Standard_Boolean ObjectInteractor::beginBoolean(BooleanAction action) noexcept {
		return _booleanOpController->begin(action);
	}

	BooleanApplyResult ObjectInteractor::applyBoolean(BooleanAction action) noexcept {
		return _booleanOpController->apply(action);
	}

	void ObjectInteractor::cancelBoolean(BooleanAction action) noexcept {
		_booleanOpController->cancel(action);
		try {
			attachManipulatorToSelection();
		} catch (...) {
		}
	}

	void ObjectInteractor::cancelActiveBoolean() noexcept {
		_booleanOpController->cancelActive();
		try {
			attachManipulatorToSelection();
		} catch (...) {
		}
	}

    const bool ObjectInteractor::canApplyBoolean() const {
        return _booleanOpController->canApply();
    }

	const bool ObjectInteractor::hasActiveBoolean() const {
		return _booleanOpController->hasActiveOperation();
	}

	const bool ObjectInteractor::hasActiveBoolean(
		const BooleanAction action) const {
		return _booleanOpController->hasActiveOperation(action);
	}

	const bool ObjectInteractor::hasUnresolvedBoolean() const {
		return _booleanOpController->hasUnresolvedState();
	}

	const bool ObjectInteractor::isBooleanSelectionFrozen() const {
		return _booleanOpController->isSelectionFrozen();
	}

	BooleanPreviewState ObjectInteractor::booleanPreviewState() const noexcept {
		return _booleanOpController == nullptr
			? BooleanPreviewState::Selecting
			: _booleanOpController->previewState();
	}

	std::uint64_t ObjectInteractor::booleanPreviewGeneration() const noexcept {
		return _booleanOpController == nullptr
			? 0
			: _booleanOpController->previewGeneration();
	}

	void ObjectInteractor::setBooleanPreviewStateChangedCallback(
		std::function<void()> callback) {
		_booleanOpController->setPreviewStateChangedCallback(
			std::move(callback));
	}

#ifdef DEBUG
	Standard_Boolean ObjectInteractor::debugBeginBooleanSelection(
		const std::vector<Handle(AIS_InteractiveObject)>& actors,
		const std::vector<Handle(AIS_InteractiveObject)>& subjects,
		BooleanAction action) noexcept {
		try {
			const bool isSingleResult =
				action == BooleanAction::BooleanUnion
				|| action == BooleanAction::BooleanIntersect;
			if ((action != BooleanAction::BooleanSubtract
					&& !isSingleResult)
				|| (isSingleResult && !actors.empty())
				|| actors.size() + subjects.size()
					> BooleanOperationController::kMaxSourceOperands
				|| (action == BooleanAction::BooleanSubtract
					&& (actors.empty() || subjects.empty()))
				|| (isSingleResult
					&& subjects.size() < 2)) {
				return Standard_False;
			}
			_booleanOpController->cancelActive();
			if (!_booleanOpController->begin(action)) {
				return Standard_False;
			}
			for (const Handle(AIS_InteractiveObject)& actor : actors) {
				if (!_booleanOpController->setSelectionState(
						actor,
						BooleanSelectionType::Actor,
						action)) {
					_booleanOpController->cancelActive();
					return Standard_False;
				}
			}
			for (const Handle(AIS_InteractiveObject)& subject : subjects) {
				if (!_booleanOpController->setSelectionState(
						subject,
						BooleanSelectionType::Subject,
						action)) {
					_booleanOpController->cancelActive();
					return Standard_False;
				}
			}
			return _booleanOpController->visualApply(action);
		} catch (...) {
			_booleanOpController->cancelActive();
			return Standard_False;
		}
	}

	Standard_Boolean ObjectInteractor::debugRecomputeBooleanPreview(
		BooleanAction action) noexcept {
		try {
			if (!_booleanOpController->hasActiveOperation()) {
				return Standard_False;
			}
			return _booleanOpController->visualApply(action);
		} catch (...) {
			_booleanOpController->cancelActive();
			return Standard_False;
		}
	}

	BooleanPreviewDebugState
	ObjectInteractor::debugBooleanPreviewState() const noexcept {
		return _booleanOpController->debugPreviewState();
	}

	void ObjectInteractor::debugSetBooleanPreviewWorkerBlocked(
		const Standard_Boolean blocked) noexcept {
		_booleanOpController->debugSetWorkerBlocked(blocked);
	}

	void ObjectInteractor::debugSetMaximumBooleanCaptureTopologyNodes(
		const Standard_Size limit) noexcept {
		_booleanOpController->debugSetMaximumCaptureTopologyNodes(limit);
	}

	void ObjectInteractor::debugSetMaximumBooleanResultTopologyNodes(
		const Standard_Size limit) noexcept {
		_booleanOpController->debugSetMaximumResultTopologyNodes(limit);
	}

	void ObjectInteractor::debugSetMaximumBooleanResultSolids(
		const Standard_Size limit) noexcept {
		_booleanOpController->debugSetMaximumResultSolids(limit);
	}

	void ObjectInteractor::debugSetBooleanTransactionFailureCount(
		const Standard_Size count) noexcept {
		_booleanOpController->debugSetTransactionFailureCount(count);
	}

	void ObjectInteractor::debugSetBooleanAbortFailureCount(
		const Standard_Size count) noexcept {
		_booleanOpController->debugSetAbortFailureCount(count);
	}
#endif

	Standard_Boolean ObjectInteractor::beginLinearArray() noexcept {
		if (_linearArrayController == nullptr) {
			return Standard_False;
		}
		const Standard_Boolean didBegin =
			_linearArrayController->begin();
		if (!didBegin
			&& !_linearArrayController->hasActiveOperation()
			&& _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray) {
			_manipulatorType =
				PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
		}
		return didBegin;
	}

	LinearArrayApplyResult ObjectInteractor::applyLinearArray() noexcept {
		return _linearArrayController == nullptr
			? LinearArrayApplyResult::NoChange
			: _linearArrayController->apply();
	}

	Standard_Boolean ObjectInteractor::cancelLinearArray() noexcept {
		return _linearArrayController == nullptr
			|| _linearArrayController->cancel();
	}

	Standard_Boolean ObjectInteractor::setLinearArrayAxis(
		const LinearArrayAxis axis) noexcept {
		return _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray
			&& _linearArrayController != nullptr
			&& _linearArrayController->setAxis(axis);
	}

	Standard_Boolean ObjectInteractor::setLinearArrayCount(
		const Standard_Integer count) noexcept {
		return _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray
			&& _linearArrayController != nullptr
			&& _linearArrayController->setCount(count);
	}

	Standard_Boolean ObjectInteractor::setLinearArraySpacing(
		const Standard_Real spacing) noexcept {
		return _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray
			&& _linearArrayController != nullptr
			&& _linearArrayController->setSpacing(spacing);
	}

	LinearArrayAxis ObjectInteractor::linearArrayAxis() const noexcept {
		return _linearArrayController == nullptr
			? LinearArrayAxis::X
			: _linearArrayController->axis();
	}

	Standard_Integer ObjectInteractor::linearArrayCount() const noexcept {
		return _linearArrayController == nullptr
			? LinearArrayOperationController::kDefaultCount
			: _linearArrayController->count();
	}

	Standard_Real ObjectInteractor::linearArraySpacing() const noexcept {
		return _linearArrayController == nullptr
			? 0.0
			: _linearArrayController->spacing();
	}

	Standard_Real ObjectInteractor::linearArrayMetersPerUnit() const noexcept {
		return _linearArrayController == nullptr
			? 0.0
			: _linearArrayController->metersPerUnit();
	}

	std::pair<Standard_Integer, Standard_Integer>
	ObjectInteractor::linearArrayCountRange() const noexcept {
		return _linearArrayController == nullptr
			? std::pair<Standard_Integer, Standard_Integer>{
				LinearArrayOperationController::kMinimumCount,
				LinearArrayOperationController::kMaximumCount}
			: _linearArrayController->countRange();
	}

	std::pair<Standard_Real, Standard_Real>
	ObjectInteractor::linearArraySpacingRange() const noexcept {
		return _linearArrayController == nullptr
			? std::pair<Standard_Real, Standard_Real>{0.0, 0.0}
			: _linearArrayController->spacingRange();
	}

	Standard_Boolean ObjectInteractor::canApplyLinearArray() const noexcept {
		return _linearArrayController != nullptr
			&& _linearArrayController->canApply();
	}

	Standard_Boolean ObjectInteractor::hasActiveLinearArray() const noexcept {
		return _linearArrayController != nullptr
			&& _linearArrayController->hasActiveOperation();
	}

	Standard_Boolean ObjectInteractor::hasUnresolvedLinearArray() const noexcept {
		return _linearArrayController != nullptr
			&& _linearArrayController->hasUnresolvedState();
	}

	LinearArrayPreviewState
	ObjectInteractor::linearArrayPreviewState() const noexcept {
		return _linearArrayController == nullptr
			? LinearArrayPreviewState::Unavailable
			: _linearArrayController->previewState();
	}

	std::uint64_t ObjectInteractor::linearArrayPreviewGeneration() const noexcept {
		return _linearArrayController == nullptr
			? 0
			: _linearArrayController->previewGeneration();
	}

	Standard_Boolean ObjectInteractor::captureLinearArrayPreview(
		std::vector<Handle(AIS_Shape)>& previewObjects) const noexcept {
		previewObjects.clear();
		return _linearArrayController != nullptr
			&& _linearArrayController->capturePreview(previewObjects);
	}

	Standard_Boolean
	ObjectInteractor::canPublishEmptyLinearArrayPreview() const noexcept {
		return _linearArrayController != nullptr
			&& _linearArrayController->canPublishEmptyPreview();
	}

	void ObjectInteractor::setLinearArrayPreviewStateChangedCallback(
		std::function<void()> callback) {
		if (_linearArrayController != nullptr) {
			_linearArrayController->setPreviewStateChangedCallback(
				std::move(callback));
		}
	}

#ifdef DEBUG
	LinearArrayPreviewDebugState
	ObjectInteractor::debugLinearArrayPreviewState() const noexcept {
		return _linearArrayController == nullptr
			? LinearArrayPreviewDebugState{}
			: _linearArrayController->debugPreviewState();
	}

	void ObjectInteractor::debugSetLinearArrayTransactionFailureCount(
		const Standard_Size count) noexcept {
		if (_linearArrayController != nullptr) {
			_linearArrayController->debugSetTransactionFailureCount(count);
		}
	}

	void ObjectInteractor::debugSetLinearArrayAbortFailureCount(
		const Standard_Size count) noexcept {
		if (_linearArrayController != nullptr) {
			_linearArrayController->debugSetAbortFailureCount(count);
		}
	}

	void ObjectInteractor::debugSetLinearArrayEraseFailureCount(
		const Standard_Size count) noexcept {
		if (_linearArrayController != nullptr) {
			_linearArrayController->debugSetEraseFailureCount(count);
		}
	}

	void ObjectInteractor::debugSetLinearArrayCommitMode(
		const Standard_Integer mode) noexcept {
		if (_linearArrayController != nullptr) {
			_linearArrayController->debugSetCommitMode(mode);
		}
	}

	void ObjectInteractor::debugSetLinearArrayPostCommitInspectFailureCount(
		const Standard_Size count) noexcept {
		if (_linearArrayController != nullptr) {
			_linearArrayController
				->debugSetPostCommitInspectFailureCount(count);
		}
	}

	void ObjectInteractor::debugSetMaximumLinearArrayTopologyNodes(
		const Standard_Size limit) noexcept {
		if (_linearArrayController != nullptr) {
			_linearArrayController->debugSetMaximumTopologyNodes(limit);
		}
	}

	Standard_Boolean ObjectInteractor::
	debugMutateFirstLinearArraySourcePersistedTransform() noexcept {
		return _linearArrayController != nullptr
			&& _linearArrayController
				->debugMutateFirstSourcePersistedTransform();
	}
#endif

	Standard_Boolean ObjectInteractor::tryMirror(
		Standard_Integer axisIndex,
		bool backward) noexcept {
		if (_manipulatorType
				!= PrimitiveManipulatorType::PrimitiveGizmoTypeMirror
			|| _mirrorPreviewState == MirrorPreviewState::Committing
			|| _mirrorPreviewState == MirrorPreviewState::OutcomeUnknown) {
			return Standard_False;
		}
		if (_mirrorPlanePicking && !cancelMirrorPlanePicking()) {
			return Standard_False;
		}
		++_mirrorPreviewGeneration;
		try {
			if (!tryMirrorImpl(axisIndex, backward)) {
				_mirrorPreviewState = MirrorPreviewState::Failed;
				return Standard_False;
			}
			if (!clearCustomMirrorPlaneState()) {
				_mirrorPreviewState = MirrorPreviewState::Failed;
				return Standard_False;
			}
			_trialMirrorUsesCustomPlane = false;
			_mirrorPreviewState = MirrorPreviewState::Ready;
			return Standard_True;
		} catch (...) {
			(void)clearTrialMirrorObjects();
			_mirrorPreviewState = MirrorPreviewState::Failed;
			return Standard_False;
		}
	}

	Standard_Boolean ObjectInteractor::tryMirrorImpl(
		Standard_Integer axisIndex,
		bool backward) {
		if (_manipulator.IsNull()
			|| !_manipulator->IsAttached()
			|| axisIndex < 0
			|| axisIndex > 2) {
			return Standard_False;
		}
		const Handle(Core3DManipulatorObjectSequence) anObjects =
			_manipulator->Objects();
		if (anObjects.IsNull() || anObjects->Size() == 0
			|| static_cast<std::size_t>(anObjects->Size())
				> kMaxMirrorPreviewBodies) {
			return Standard_False;
		}

		Bnd_Box aBox, aBoxSum;
		for (const Handle(AIS_InteractiveObject)& anObject : *anObjects) {
			anObject->BoundingBox(aBox);
			aBoxSum.Add(aBox);
		}
		if (aBoxSum.IsVoid() || aBoxSum.IsOpen()) {
			return Standard_False;
		}
		const gp_Pnt aMaximum = aBoxSum.CornerMax();
		const gp_Pnt aMinimum = aBoxSum.CornerMin();
		gp_Pnt anOrigin = _manipulator->Position().Location();
		gp_Dir aNormal;
		gp_Dir anXDirection;
		const Standard_ShortReal aSign = backward ? -1.0f : 1.0f;
		switch (axisIndex) {
			case 0:
				aNormal = gp_Dir(1.0, 0.0, 0.0);
				anXDirection = gp_Dir(0.0, 1.0, 0.0);
				anOrigin.Translate(gp_Vec(
					abs(aMaximum.X() - aMinimum.X()) * 0.5f * aSign,
					0.0, 0.0));
				break;
			case 1:
				aNormal = gp_Dir(0.0, 1.0, 0.0);
				anXDirection = gp_Dir(0.0, 0.0, 1.0);
				anOrigin.Translate(gp_Vec(
					0.0,
					abs(aMaximum.Y() - aMinimum.Y()) * 0.5f * aSign,
					0.0));
				break;
			case 2:
				aNormal = gp_Dir(0.0, 0.0, 1.0);
				anXDirection = gp_Dir(1.0, 0.0, 0.0);
				anOrigin.Translate(gp_Vec(
					0.0, 0.0,
					abs(aMaximum.Z() - aMinimum.Z()) * 0.5f * aSign));
				break;
			default:
				return Standard_False;
		}
		return tryMirrorWorldPlaneImpl(
			gp_Ax2(anOrigin, aNormal, anXDirection));
	}

	Standard_Boolean ObjectInteractor::tryMirrorWorldPlaneImpl(
		const gp_Ax2& theWorldPlane) {
		if (!_trialMirrorObjects.empty() && !_trialMirrorObjectsValid) {
			(void)clearTrialMirrorObjects();
			if (!_trialMirrorObjects.empty()) {
				return Standard_False;
			}
		}
		if (_manipulator.IsNull() || !_manipulator->IsAttached()
			|| !IsFiniteBoundedPoint(theWorldPlane.Location())
			|| !IsFiniteDirection(theWorldPlane.Direction())
			|| !IsFiniteDirection(theWorldPlane.XDirection())) {
			return Standard_False;
		}
		const Handle(TDocStd_Document) aSourceDocument =
			myDoc.IsNull()
			? Handle(TDocStd_Document)()
			: myDoc->ChangeDocument();
		if (aSourceDocument.IsNull()
			|| aSourceDocument->HasOpenCommand()) {
			return Standard_False;
		}

		Handle(Core3DManipulatorObjectSequence) anObjects = _manipulator->Objects();
		if (anObjects.IsNull() || anObjects->Size() == 0
			|| static_cast<std::size_t>(anObjects->Size())
				> kMaxMirrorPreviewBodies) {
			return Standard_False;
		}
		Core3DManipulatorObjectSequence::Iterator anObjIter (*anObjects);
		std::vector<Handle(AIS_Shape)> replacementObjects;
		replacementObjects.reserve(
			static_cast<std::size_t>(anObjects->Size()));
		std::vector<MirrorSourceSnapshot> replacementSources;
		replacementSources.reserve(
			static_cast<std::size_t>(anObjects->Size()));
		
		if (!ManipulatorObjectsSupportBRepModeling(
				_manipulator,
				myDoc,
				PrimitiveManipulatorType::PrimitiveGizmoTypeMirror,
				_manipulatorSourceLabels)) {
			return Standard_False;
		}

#ifdef DEBUG
		const Standard_Size anAggregateLimit =
			std::max<Standard_Size>(
				1,
				std::min(
					_debugMaximumMirrorTopologyNodes,
					kMaxMirrorTopologyNodes));
#else
		const Standard_Size anAggregateLimit = kMaxMirrorTopologyNodes;
#endif
		const Standard_Size aPerSourceLimit =
			std::min(kMaxMirrorSourceTopologyNodes, anAggregateLimit);
		Standard_Size anAggregateTopologyNodes = 0;

		for (; anObjIter.More(); anObjIter.Next()) {
			Handle(AIS_InteractiveObject) selected = anObjIter.Value();

			Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(selected);
			if (shape.IsNull() || shape->Shape().IsNull()) {
				return Standard_False;
			}
			const auto source = _manipulatorSourceLabels.find(
				selected.get());
			const TDF_Label sourceLabel =
				source == _manipulatorSourceLabels.end()
				? TDF_Label()
				: source->second;
			const OcctGeometryRepresentation sourceRepresentation =
				myDoc->GeometryRepresentationForLabel(sourceLabel);
			if (sourceLabel.IsNull()
				|| (sourceRepresentation
					!= OcctGeometryRepresentation::LegacyUnknown
					&& sourceRepresentation
						!= OcctGeometryRepresentation::BRep)) {
				// The current negative-transform path deliberately drops mesh
				// data, so triangle-only definitions cannot enter a preview.
				return Standard_False;
			}
			const TopoDS_Shape sourceStoredShape =
				XCAFDoc_ShapeTool::GetShape(sourceLabel);
			const std::string sourceEntityIdentifier =
				myDoc->EntityIdentifierForLabel(sourceLabel);
			const std::string sourceDefinitionIdentifier =
				myDoc->DefinitionIdentifierForLabel(sourceLabel);
			gp_Trsf aTrsfSelected = selected->Transformation();
			Standard_Size sourceNodeCount = 0;
			if (sourceStoredShape.IsNull()
				|| !sourceStoredShape.IsEqual(shape->Shape())
				|| sourceEntityIdentifier.empty()
				|| sourceDefinitionIdentifier.empty()
				|| TransformDiffers(
					myDoc->ObjectTransformForLabel(sourceLabel),
					aTrsfSelected)
				|| HasStyledMirrorSubshape(
					aSourceDocument,
					sourceLabel,
					kMaxMirrorSourceTopologyNodes)
				|| !CountBoundedMirrorTopology(
					sourceStoredShape,
					aPerSourceLimit,
					sourceNodeCount)
				|| sourceNodeCount
					> anAggregateLimit - anAggregateTopologyNodes
				|| !IsTopologicallyValid(sourceStoredShape)) {
				return Standard_False;
			}
			anAggregateTopologyNodes += sourceNodeCount;

			gp_Pnt offset = theWorldPlane.Location();
			gp_Dir mirrorAxis = theWorldPlane.Direction();
			gp_Dir mirrorPln = theWorldPlane.XDirection();
			
			mirrorAxis.Transform(aTrsfSelected.Inverted());
			mirrorPln.Transform(aTrsfSelected.Inverted());
			offset.Transform(aTrsfSelected.Inverted());

			gp_Trsf aTrsfMirror;
			aTrsfMirror.SetMirror(gp_Ax2(offset, mirrorAxis, mirrorPln));
			
			// OCCT's negative-transform mesh-copy path corrupts allocator state
			// when mirror previews are replaced repeatedly. Keep mesh copying
			// disabled; AIS Display below triangulates the owned trial before
			// renderer-neutral capture reads that cache.
			BRepBuilderAPI_Transform aBRepTrsf(
				shape->Shape(),
				aTrsfSelected * aTrsfMirror,
				Standard_False,
				Standard_False);
			Handle(AIS_Shape) aShapePrs = new AIS_Shape (aBRepTrsf.Shape());
			Standard_Size resultNodeCount = 0;
			if (aShapePrs.IsNull() || aShapePrs->Shape().IsNull()
				|| !IsShapeWithinModelCoordinates(aShapePrs->Shape())
				|| !CountBoundedMirrorTopology(
					aShapePrs->Shape(),
					anAggregateLimit - anAggregateTopologyNodes,
					resultNodeCount)
				|| resultNodeCount == 0
				|| !IsTopologicallyValid(aShapePrs->Shape())) {
				return Standard_False;
			}
			anAggregateTopologyNodes += resultNodeCount;
			myDoc->LoadObjectMeterial(sourceLabel, aShapePrs);
			replacementObjects.push_back(aShapePrs);
			replacementSources.push_back({
				shape,
				sourceLabel,
				sourceEntityIdentifier,
				sourceDefinitionIdentifier,
				sourceStoredShape,
				aTrsfSelected,
			});
		}
		std::vector<TDF_Label> aSourceLabels;
		aSourceLabels.reserve(replacementSources.size());
		for (const MirrorSourceSnapshot& aSource : replacementSources) {
			aSourceLabels.push_back(aSource.label);
		}
		if (!myDoc->CanDuplicateGeometryDefinitions(aSourceLabels)) {
			return Standard_False;
		}

		// A plane is a choice for the current mirror operation, not an
		// additional operation. Take ownership of every handle that might be
		// displayed before mutating OCCT. Until the full swap succeeds the set
		// is deliberately non-applicable, and cleanup retains any handle whose
		// erase operation fails so no presentation can become orphaned.
		const std::vector<Handle(AIS_Shape)> previousObjects =
			_trialMirrorObjects;
		std::vector<Handle(AIS_Shape)> transitionObjects =
			previousObjects;
		transitionObjects.insert(
			transitionObjects.end(),
			replacementObjects.begin(),
			replacementObjects.end());
		_trialMirrorObjects = std::move(transitionObjects);
		_trialMirrorObjectsValid = false;

		for (const Handle(AIS_Shape)& aShapePrs : replacementObjects) {
			myContext->Display(
				aShapePrs,
				AIS_Shaded,
				(Standard_Integer)0,
				Standard_False);
			// Mirror trials are explicit presentation only. Keep the OCCT
			// fallback behavior aligned with the renderer-neutral nonselectable
			// contract while the operation is pending.
			myContext->Deactivate(aShapePrs);
			if (!myContext->IsDisplayed(aShapePrs)) {
				return Standard_False;
			}
		}
		for (const Handle(AIS_Shape)& aShapePrs
			 : previousObjects) {
			bool shouldErase = true;
#ifdef DEBUG
			if (_debugMirrorEraseFailureCount > 0) {
				--_debugMirrorEraseFailureCount;
				shouldErase = false;
			}
#endif
			if (shouldErase) {
				myContext->Erase(aShapePrs, Standard_False);
			}
			if (myContext->IsDisplayed(aShapePrs)) {
				// _trialMirrorObjects still owns the combined old/new set.
				// A non-throwing no-op erase must not orphan the old preview.
				return Standard_False;
			}
		}
		_trialMirrorObjects = std::move(replacementObjects);
		_trialMirrorSources = std::move(replacementSources);
		_pendingMirrorResults.clear();
		_mirrorOwnsDocumentCommand = false;
		_trialMirrorObjectsValid = true;
		_mirrorPreviewState = MirrorPreviewState::Ready;
		myContext->UpdateCurrentViewer();
		return Standard_True;
	}

	Standard_Boolean ObjectInteractor::beginMirrorPlanePicking() noexcept {
		if (_mirrorPlanePicking) {
			return Standard_True;
		}
		if (_manipulatorType
				!= PrimitiveManipulatorType::PrimitiveGizmoTypeMirror
			|| _mirrorPreviewState == MirrorPreviewState::Committing
			|| _mirrorPreviewState == MirrorPreviewState::OutcomeUnknown
			|| myContext.IsNull() || myDoc.IsNull()
			|| _manipulator.IsNull() || !_manipulator->IsAttached()) {
			return Standard_False;
		}

		try {
			OCC_CATCH_SIGNALS
			std::vector<MirrorPlanePickSelectionModes> aSnapshots;
			AIS_ListOfInteractive aDisplayed;
			myContext->DisplayedObjects(aDisplayed);
			constexpr std::size_t kMaximumPickPresentations =
				limits::kMaximumLeafPresentations + 1;
			aSnapshots.reserve(std::min<std::size_t>(
				static_cast<std::size_t>(aDisplayed.Size()) + 1,
				kMaximumPickPresentations));

			auto captureModes = [&](const Handle(AIS_InteractiveObject)& theObject) {
				MirrorPlanePickSelectionModes aSnapshot;
				aSnapshot.presentation = theObject;
				TColStd_ListOfInteger anActive;
				myContext->ActivatedModes(theObject, anActive);
				aSnapshot.modes.reserve(
					static_cast<std::size_t>(anActive.Extent()));
				for (TColStd_ListIteratorOfListOfInteger anIterator(anActive);
					 anIterator.More(); anIterator.Next()) {
					aSnapshot.modes.push_back(anIterator.Value());
				}
				aSnapshots.push_back(std::move(aSnapshot));
			};

			const Standard_Integer aFaceMode =
				AIS_Shape::SelectionMode(TopAbs_FACE);
#ifdef DEBUG
			const Standard_Size aMaximumReferenceTopologyNodes =
				std::max<Standard_Size>(
					1,
					std::min(
						_debugMaximumMirrorReferenceTopologyNodes,
						kMaxMirrorReferenceTopologyNodes));
			const Standard_Size aMaximumReferenceFaces =
				std::max<Standard_Size>(
					1,
					std::min(
						_debugMaximumMirrorReferenceFaces,
						kMaxMirrorReferenceFaces));
#else
			const Standard_Size aMaximumReferenceTopologyNodes =
				kMaxMirrorReferenceTopologyNodes;
			const Standard_Size aMaximumReferenceFaces =
				kMaxMirrorReferenceFaces;
#endif
			Standard_Size anAggregateReferenceTopologyNodes = 0;
			Standard_Size anAggregateReferenceFaces = 0;
			for (AIS_ListIteratorOfListOfInteractive anIterator(aDisplayed);
				 anIterator.More(); anIterator.Next()) {
				const Handle(AIS_InteractiveObject)& anObject =
					anIterator.Value();
				const Handle(AIS_Shape) aShape =
					Handle(AIS_Shape)::DownCast(anObject);
				const TDF_Label aLabel = myDoc->ShapeLabel(anObject);
				const OcctGeometryRepresentation aRepresentation =
					myDoc->GeometryRepresentationForLabel(aLabel);
				const TopoDS_Shape aStoredShape = aLabel.IsNull()
					? TopoDS_Shape()
					: XCAFDoc_ShapeTool::GetShape(aLabel);
				if (aShape.IsNull() || aShape->Shape().IsNull()
					|| !myDoc->IsPresentationEditable(anObject)
					|| aLabel.IsNull()
					|| !myDoc->IsEditableFreeSimpleDefinitionLabel(aLabel)
					|| !IsBRepModelingRepresentation(aRepresentation)
					|| aStoredShape.IsNull()
					|| !aStoredShape.IsEqual(aShape->Shape())) {
					continue;
				}
				if (anAggregateReferenceTopologyNodes
						>= aMaximumReferenceTopologyNodes
					|| anAggregateReferenceFaces
						>= aMaximumReferenceFaces) {
					return Standard_False;
				}
				const Standard_Size aPresentationTopologyLimit =
					std::min(
						kMaxMirrorReferenceTopologyNodesPerPresentation,
						aMaximumReferenceTopologyNodes
							- anAggregateReferenceTopologyNodes);
				const Standard_Size aPresentationFaceLimit =
					std::min(
						kMaxMirrorReferenceFacesPerPresentation,
						aMaximumReferenceFaces
							- anAggregateReferenceFaces);
				Standard_Size aTopologyNodeCount = 0;
				TopTools_IndexedMapOfShape aFaces;
				if (!MapBoundedMirrorReferenceFaces(
						aStoredShape,
						aPresentationTopologyLimit,
						aPresentationFaceLimit,
						aTopologyNodeCount,
						aFaces)) {
					return Standard_False;
				}
				anAggregateReferenceTopologyNodes += aTopologyNodeCount;
				if (aFaces.Extent() == 0) {
					continue;
				}
				anAggregateReferenceFaces +=
					static_cast<Standard_Size>(aFaces.Extent());
				if (aSnapshots.size() >= kMaximumPickPresentations) {
					return Standard_False;
				}
				captureModes(anObject);
			}
			if (aSnapshots.empty()
				|| aSnapshots.size() >= kMaximumPickPresentations) {
				return Standard_False;
			}

			captureModes(_manipulator);
			_mirrorPlanePickModes = std::move(aSnapshots);
			_mirrorPlanePicking = true;
			for (std::size_t anIndex = 0;
				 anIndex + 1 < _mirrorPlanePickModes.size(); ++anIndex) {
				myContext->SetSelectionModeActive(
					_mirrorPlanePickModes[anIndex].presentation,
					aFaceMode,
					Standard_True,
					AIS_SelectionModesConcurrency_Multiple,
					Standard_True);
			}
			_manipulator->DeactivateCurrentMode();
			myContext->Deactivate(_manipulator);
			myContext->ClearDetected(Standard_False);

			for (std::size_t anIndex = 0;
				 anIndex + 1 < _mirrorPlanePickModes.size(); ++anIndex) {
				auto anExpected = _mirrorPlanePickModes[anIndex].modes;
				if (std::find(anExpected.begin(), anExpected.end(), aFaceMode)
					== anExpected.end()) {
					anExpected.push_back(aFaceMode);
				}
				if (!SelectionModesMatch(
						myContext,
						_mirrorPlanePickModes[anIndex].presentation,
						anExpected)) {
					if (restoreMirrorPlanePickingModes()) {
						return Standard_False;
					}
					_mirrorPreviewState = MirrorPreviewState::Failed;
					return Standard_False;
				}
			}
			if (!SelectionModesMatch(myContext, _manipulator, {})) {
				if (restoreMirrorPlanePickingModes()) {
					return Standard_False;
				}
				_mirrorPreviewState = MirrorPreviewState::Failed;
				return Standard_False;
			}
			_manipulator->Redisplay();
			myContext->UpdateCurrentViewer();
			return Standard_True;
		} catch (...) {
			if (_mirrorPlanePicking
				&& !restoreMirrorPlanePickingModes()) {
				_mirrorPreviewState = MirrorPreviewState::Failed;
			}
			return Standard_False;
		}
	}

	Standard_Boolean
	ObjectInteractor::restoreMirrorPlanePickingModes() noexcept {
		if (!_mirrorPlanePicking) {
			return Standard_True;
		}
		if (myContext.IsNull() || _mirrorPlanePickModes.empty()) {
			return Standard_False;
		}
		try {
			OCC_CATCH_SIGNALS
			const Standard_Integer aFaceMode =
				AIS_Shape::SelectionMode(TopAbs_FACE);
			for (std::size_t anIndex = 0;
				 anIndex < _mirrorPlanePickModes.size(); ++anIndex) {
				const MirrorPlanePickSelectionModes& aSnapshot =
					_mirrorPlanePickModes[anIndex];
				if (aSnapshot.presentation.IsNull()) {
					return Standard_False;
				}
				const bool isManipulator =
					anIndex + 1 == _mirrorPlanePickModes.size();
				if (isManipulator) {
					myContext->Deactivate(aSnapshot.presentation);
					for (const Standard_Integer aMode : aSnapshot.modes) {
						myContext->SetSelectionModeActive(
							aSnapshot.presentation,
							aMode,
							Standard_True,
							AIS_SelectionModesConcurrency_Multiple,
							Standard_True);
					}
				} else if (std::find(
						aSnapshot.modes.begin(),
						aSnapshot.modes.end(),
						aFaceMode) == aSnapshot.modes.end()) {
					// Remove only the additive face detector. Deactivating the
					// source's original mode can invalidate its selected owner.
					myContext->SetSelectionModeActive(
						aSnapshot.presentation,
						aFaceMode,
						Standard_False,
						AIS_SelectionModesConcurrency_Multiple,
						Standard_True);
				}
			}
			for (const MirrorPlanePickSelectionModes& aSnapshot :
				 _mirrorPlanePickModes) {
				if (!SelectionModesMatch(
						myContext,
						aSnapshot.presentation,
						aSnapshot.modes)) {
					return Standard_False;
				}
			}
			_mirrorPlanePickModes.clear();
			_mirrorPlanePicking = false;
			if (!_manipulator.IsNull()) {
				_manipulator->Redisplay();
			}
			myContext->ClearDetected(Standard_False);
			myContext->UpdateCurrentViewer();
			return Standard_True;
		} catch (...) {
			return Standard_False;
		}
	}

	Standard_Boolean
	ObjectInteractor::cancelMirrorPlanePicking() noexcept {
		return restoreMirrorPlanePickingModes();
	}

	const bool ObjectInteractor::isPickingMirrorPlane() const noexcept {
		return _mirrorPlanePicking;
	}

	const bool ObjectInteractor::hasCustomMirrorPlane() const noexcept {
		return _customMirrorPlane.has_value()
			&& _mirrorReferencePresentations.size() == 1
			&& !_mirrorReferencePresentations.front().IsNull()
			&& _trialMirrorUsesCustomPlane;
	}

	const bool ObjectInteractor::hasCustomMirrorPlaneState() const noexcept {
		return _customMirrorPlane.has_value()
			|| !_mirrorReferencePresentations.empty()
			|| _trialMirrorUsesCustomPlane;
	}

	Standard_Real ObjectInteractor::mirrorPlaneOffset() const noexcept {
		return _mirrorPlaneOffset;
	}

	std::pair<Standard_Real, Standard_Real>
	ObjectInteractor::mirrorPlaneOffsetRange() const noexcept {
		const std::pair<Standard_Real, Standard_Real> anUnavailable{0.0, 0.0};
		if (!_customMirrorPlane.has_value() || !_trialMirrorUsesCustomPlane
			|| !_trialMirrorObjectsValid || _trialMirrorObjects.empty()) {
			return anUnavailable;
		}
		try {
			OCC_CATCH_SIGNALS
			gp_Ax2 aCurrentPlane;
			if (!customMirrorPlaneIsCurrent(
					*_customMirrorPlane,
					aCurrentPlane,
					_mirrorPlaneOffset)) {
				return anUnavailable;
			}

			const Standard_Real aLimit =
				limits::kMaximumModelCoordinateMagnitude
				- 16.0 * Precision::Confusion();
			Standard_Real aMinimum = -aLimit;
			Standard_Real aMaximum = aLimit;
			auto intersectCoordinate = [&](
					const Standard_Real theValue,
					const Standard_Real theCoefficient,
					const Standard_Real theParameterOrigin) {
				if (!std::isfinite(theValue)
					|| !std::isfinite(theCoefficient)
					|| !std::isfinite(theParameterOrigin)) {
					return false;
				}
				// A direction component is dimensionless. Precision::Confusion()
				// is a length tolerance and cannot classify it as zero: even a
				// very small nonzero component can cross the coordinate boundary
				// over the advertised offset span. Solve every representable
				// nonzero coefficient so every returned endpoint remains valid.
				if (theCoefficient == 0.0) {
					return std::abs(theValue) <= aLimit;
				}
				Standard_Real aFirst = theParameterOrigin
					+ (-aLimit - theValue) / theCoefficient;
				Standard_Real aLast = theParameterOrigin
					+ (aLimit - theValue) / theCoefficient;
				if (aFirst > aLast) {
					std::swap(aFirst, aLast);
				}
				aMinimum = std::max(aMinimum, aFirst);
				aMaximum = std::min(aMaximum, aLast);
				return aMinimum < aMaximum;
			};
			auto intersectBox = [&](
					const Bnd_Box& theBox,
					const gp_Dir& theNormal,
					const Standard_Real theMotionMultiplier,
					const Standard_Real theParameterOrigin) {
				if (theBox.IsVoid() || theBox.IsOpen()) {
					return false;
				}
				const gp_Pnt aLow = theBox.CornerMin();
				const gp_Pnt aHigh = theBox.CornerMax();
				const std::array<Standard_Real, 3> aCoefficients = {
					theMotionMultiplier * theNormal.X(),
					theMotionMultiplier * theNormal.Y(),
					theMotionMultiplier * theNormal.Z(),
				};
				const std::array<Standard_Real, 3> aLows = {
					aLow.X(), aLow.Y(), aLow.Z()};
				const std::array<Standard_Real, 3> aHighs = {
					aHigh.X(), aHigh.Y(), aHigh.Z()};
				for (std::size_t anAxis = 0; anAxis < 3; ++anAxis) {
					if (!intersectCoordinate(
							aLows[anAxis],
							aCoefficients[anAxis],
							theParameterOrigin)
						|| !intersectCoordinate(
							aHighs[anAxis],
							aCoefficients[anAxis],
							theParameterOrigin)) {
						return false;
					}
				}
				return true;
			};

			for (const Handle(AIS_Shape)& aPreview : _trialMirrorObjects) {
				if (aPreview.IsNull() || aPreview->Shape().IsNull()) {
					return anUnavailable;
				}
				Bnd_Box aPreviewBox;
				BRepBndLib::Add(
					aPreview->Shape(), aPreviewBox, Standard_False);
				// Moving a reflection plane by t moves the reflected result by
				// 2*t along its normal. The current preview supplies the exact
				// reference AABB at _mirrorPlaneOffset.
				if (!intersectBox(
						aPreviewBox,
						aCurrentPlane.Direction(),
						2.0,
						_mirrorPlaneOffset)) {
					return anUnavailable;
				}
			}

			Bnd_Box aReferenceBox;
			BRepBndLib::Add(
				_customMirrorPlane->face, aReferenceBox, Standard_False);
			aReferenceBox = aReferenceBox.Transformed(
				_customMirrorPlane->presentationTransform);
			if (!intersectBox(
					aReferenceBox,
					aCurrentPlane.Direction(),
					1.0,
					0.0)) {
				return anUnavailable;
			}
			const gp_Pnt aStoredPlaneOrigin =
				_customMirrorPlane->worldOrigin;
			const std::array<Standard_Real, 3> aPlaneOriginValues = {
				aStoredPlaneOrigin.X(),
				aStoredPlaneOrigin.Y(),
				aStoredPlaneOrigin.Z(),
			};
			const std::array<Standard_Real, 3> aPlaneOriginMotion = {
				aCurrentPlane.Direction().X(),
				aCurrentPlane.Direction().Y(),
				aCurrentPlane.Direction().Z(),
			};
			for (std::size_t anAxis = 0; anAxis < 3; ++anAxis) {
				if (!intersectCoordinate(
						aPlaneOriginValues[anAxis],
						aPlaneOriginMotion[anAxis],
						0.0)) {
					return anUnavailable;
				}
			}

			Bnd_Box anErgonomicBox = aReferenceBox;
			for (const MirrorSourceSnapshot& aSource : _trialMirrorSources) {
				Bnd_Box aSourceBox;
				BRepBndLib::Add(
					aSource.storedShape, aSourceBox, Standard_False);
				if (aSourceBox.IsVoid() || aSourceBox.IsOpen()) {
					return anUnavailable;
				}
				anErgonomicBox.Add(aSourceBox.Transformed(aSource.transform));
			}
			if (anErgonomicBox.IsVoid() || anErgonomicBox.IsOpen()) {
				return anUnavailable;
			}
			const gp_Pnt anErgonomicLow = anErgonomicBox.CornerMin();
			const gp_Pnt anErgonomicHigh = anErgonomicBox.CornerMax();
			const Standard_Real aDiagonal =
				anErgonomicLow.Distance(anErgonomicHigh);
			if (!std::isfinite(aDiagonal)) {
				return anUnavailable;
			}
			const Standard_Real anErgonomicSpan = std::min(
				10'000.0,
				std::max(
					2.0 * aDiagonal,
					128.0 * Precision::Confusion()));
			aMinimum = std::max(aMinimum, -anErgonomicSpan);
			aMaximum = std::min(aMaximum, anErgonomicSpan);
			const Standard_Real aZeroTolerance =
				16.0 * Precision::Confusion();
			if (!std::isfinite(aMinimum) || !std::isfinite(aMaximum)
				|| aMinimum >= -aZeroTolerance
				|| aMaximum <= aZeroTolerance
				|| _mirrorPlaneOffset < aMinimum
				|| _mirrorPlaneOffset > aMaximum) {
				return anUnavailable;
			}
			return {aMinimum, aMaximum};
		} catch (...) {
			return anUnavailable;
		}
	}

	Standard_Boolean ObjectInteractor::captureMirrorPlaneReference(
		const Handle(AIS_Shape)& thePresentation,
		const TopoDS_Face& theFace,
		MirrorPlaneReferenceSnapshot& theSnapshot) const {
		if (thePresentation.IsNull() || thePresentation->Shape().IsNull()
			|| theFace.IsNull() || myDoc.IsNull()) {
			return Standard_False;
		}
		const Handle(TDocStd_Document) aDocument = myDoc->ChangeDocument();
		const TDF_Label aLabel = myDoc->ShapeLabel(thePresentation);
		const TopoDS_Shape aStoredShape = aLabel.IsNull()
			? TopoDS_Shape()
			: XCAFDoc_ShapeTool::GetShape(aLabel);
		const OcctGeometryRepresentation aRepresentation =
			myDoc->GeometryRepresentationForLabel(aLabel);
		if (aDocument.IsNull() || aDocument->HasOpenCommand()
			|| !myDoc->IsPresentationEditable(thePresentation)
			|| aLabel.IsNull()
			|| aLabel.Data() != aDocument->GetData()
			|| !myDoc->IsEditableFreeSimpleDefinitionLabel(aLabel)
			|| !IsBRepModelingRepresentation(aRepresentation)
			|| aStoredShape.IsNull()
			|| !aStoredShape.IsEqual(thePresentation->Shape())) {
			return Standard_False;
		}

		Standard_Size aMaximumTopologyNodes =
			kMaxMirrorReferenceTopologyNodesPerPresentation;
		Standard_Size aMaximumFaces =
			kMaxMirrorReferenceFacesPerPresentation;
#ifdef DEBUG
		aMaximumTopologyNodes = std::min(
			aMaximumTopologyNodes,
			_debugMaximumMirrorReferenceTopologyNodes);
		aMaximumFaces = std::min(
			aMaximumFaces,
			_debugMaximumMirrorReferenceFaces);
#endif
		if (aMaximumTopologyNodes == 0 || aMaximumFaces == 0) {
			return Standard_False;
		}
		Standard_Size aTopologyNodeCount = 0;
		TopTools_IndexedMapOfShape aFaces;
		if (!MapBoundedMirrorReferenceFaces(
				aStoredShape,
				aMaximumTopologyNodes,
				aMaximumFaces,
				aTopologyNodeCount,
				aFaces)) {
			return Standard_False;
		}
		const Standard_Integer aFaceIndex = aFaces.FindIndex(theFace);
		if (aFaceIndex <= 0) {
			return Standard_False;
		}
		gp_Pnt aWorldOrigin;
		gp_Dir aWorldNormal;
		gp_Dir aWorldXDirection;
		const gp_Trsf aPresentationTransform =
			thePresentation->Transformation();
		if (!BuildOrientedWorldPlane(
				theFace,
				aPresentationTransform,
				aWorldOrigin,
				aWorldNormal,
				aWorldXDirection)) {
			return Standard_False;
		}
		const std::string anEntityIdentifier =
			myDoc->EntityIdentifierForLabel(aLabel);
		const std::string aDefinitionIdentifier =
			myDoc->DefinitionIdentifierForLabel(aLabel);
		if (anEntityIdentifier.empty() || aDefinitionIdentifier.empty()
			|| TransformDiffers(
				myDoc->ObjectTransformForLabel(aLabel),
				aPresentationTransform)) {
			return Standard_False;
		}

		theSnapshot = {
			aDocument,
			thePresentation,
			aLabel,
			anEntityIdentifier,
			aDefinitionIdentifier,
			aRepresentation,
			aStoredShape,
			theFace,
			aFaceIndex - 1,
			aTopologyNodeCount,
			static_cast<Standard_Size>(aFaces.Extent()),
			aPresentationTransform,
			aWorldOrigin,
			aWorldNormal,
			aWorldXDirection,
		};
		return Standard_True;
	}

	Standard_Boolean ObjectInteractor::customMirrorPlaneIsCurrent(
		const MirrorPlaneReferenceSnapshot& theSnapshot,
		gp_Ax2& theWorldPlane,
		const Standard_Real theOffset) const {
		if (!std::isfinite(theOffset)
			|| std::abs(theOffset)
				> limits::kMaximumModelCoordinateMagnitude
			|| myDoc.IsNull() || theSnapshot.document.IsNull()
			|| theSnapshot.presentation.IsNull()
			|| theSnapshot.face.IsNull()
			|| theSnapshot.faceTopologyIndex < 0) {
			return Standard_False;
		}
		const Handle(TDocStd_Document) aDocument = myDoc->ChangeDocument();
		const TDF_Label aCurrentLabel =
			myDoc->ShapeLabel(theSnapshot.presentation);
		const TopoDS_Shape aCurrentShape = aCurrentLabel.IsNull()
			? TopoDS_Shape()
			: XCAFDoc_ShapeTool::GetShape(aCurrentLabel);
		if (aDocument.IsNull() || aDocument != theSnapshot.document
			|| aDocument->HasOpenCommand()
			|| !myDoc->IsPresentationEditable(theSnapshot.presentation)
			|| aCurrentLabel.IsNull()
			|| !aCurrentLabel.IsEqual(theSnapshot.label)
			|| aCurrentLabel.Data() != aDocument->GetData()
			|| !myDoc->IsEditableFreeSimpleDefinitionLabel(aCurrentLabel)
			|| myDoc->GeometryRepresentationForLabel(aCurrentLabel)
				!= theSnapshot.representation
			|| !IsBRepModelingRepresentation(theSnapshot.representation)
			|| myDoc->EntityIdentifierForLabel(aCurrentLabel)
				!= theSnapshot.entityIdentifier
			|| myDoc->DefinitionIdentifierForLabel(aCurrentLabel)
				!= theSnapshot.definitionIdentifier
			|| aCurrentShape.IsNull()
			|| !aCurrentShape.IsEqual(theSnapshot.storedShape)
			|| !theSnapshot.presentation->Shape().IsEqual(aCurrentShape)
			|| TransformDiffers(
				theSnapshot.presentation->Transformation(),
				theSnapshot.presentationTransform)
			|| TransformDiffers(
				myDoc->ObjectTransformForLabel(aCurrentLabel),
				theSnapshot.presentationTransform)) {
			return Standard_False;
		}

		Standard_Size aMaximumTopologyNodes =
			kMaxMirrorReferenceTopologyNodesPerPresentation;
		Standard_Size aMaximumFaces =
			kMaxMirrorReferenceFacesPerPresentation;
#ifdef DEBUG
		aMaximumTopologyNodes = std::min(
			aMaximumTopologyNodes,
			_debugMaximumMirrorReferenceTopologyNodes);
		aMaximumFaces = std::min(
			aMaximumFaces,
			_debugMaximumMirrorReferenceFaces);
#endif
		Standard_Size aTopologyNodeCount = 0;
		TopTools_IndexedMapOfShape aFaces;
		if (aMaximumTopologyNodes == 0 || aMaximumFaces == 0
			|| !MapBoundedMirrorReferenceFaces(
				aCurrentShape,
				aMaximumTopologyNodes,
				aMaximumFaces,
				aTopologyNodeCount,
				aFaces)
			|| aTopologyNodeCount != theSnapshot.topologyNodeCount
			|| static_cast<Standard_Size>(aFaces.Extent())
				!= theSnapshot.faceCount) {
			return Standard_False;
		}
		const Standard_Integer aOneBasedIndex =
			theSnapshot.faceTopologyIndex + 1;
		if (aOneBasedIndex <= 0 || aOneBasedIndex > aFaces.Extent()
			|| !aFaces(aOneBasedIndex).IsEqual(theSnapshot.face)) {
			return Standard_False;
		}
		gp_Pnt aWorldOrigin;
		gp_Dir aWorldNormal;
		gp_Dir aWorldXDirection;
		if (!BuildOrientedWorldPlane(
				TopoDS::Face(aFaces(aOneBasedIndex)),
				theSnapshot.presentationTransform,
				aWorldOrigin,
				aWorldNormal,
				aWorldXDirection)
			|| !aWorldOrigin.IsEqual(
				theSnapshot.worldOrigin, Precision::Confusion())
			|| !aWorldNormal.IsEqual(
				theSnapshot.worldNormal, Precision::Angular())
			|| !aWorldXDirection.IsEqual(
				theSnapshot.worldXDirection, Precision::Angular())) {
			return Standard_False;
		}
		aWorldOrigin.Translate(gp_Vec(aWorldNormal) * theOffset);
		if (!IsFiniteBoundedPoint(aWorldOrigin)) {
			return Standard_False;
		}
		gp_Trsf anOffsetTransform;
		anOffsetTransform.SetTranslation(gp_Vec(aWorldNormal) * theOffset);
		if (!IsShapeWithinModelCoordinates(
				theSnapshot.face,
				anOffsetTransform * theSnapshot.presentationTransform)) {
			return Standard_False;
		}
		theWorldPlane = gp_Ax2(
			aWorldOrigin, aWorldNormal, aWorldXDirection);
		return Standard_True;
	}

	Standard_Boolean ObjectInteractor::clearMirrorReferencePresentation()
		noexcept {
		std::vector<Handle(AIS_Shape)> anUnresolved;
		try {
			anUnresolved.reserve(_mirrorReferencePresentations.size());
			for (const Handle(AIS_Shape)& aPresentation :
				 _mirrorReferencePresentations) {
				if (aPresentation.IsNull()) {
					continue;
				}
				bool shouldErase = true;
#ifdef DEBUG
				if (_debugMirrorReferenceEraseFailureCount > 0) {
					--_debugMirrorReferenceEraseFailureCount;
					shouldErase = false;
				}
#endif
				if (shouldErase) {
					myContext->Erase(aPresentation, Standard_False);
				}
				if (myContext->IsDisplayed(aPresentation)) {
					anUnresolved.push_back(aPresentation);
				}
			}
		} catch (...) {
			// Preserve every still-owned handle; a later cleanup can retry.
			anUnresolved = _mirrorReferencePresentations;
		}
		_mirrorReferencePresentations = std::move(anUnresolved);
		return _mirrorReferencePresentations.empty();
	}

	Standard_Boolean ObjectInteractor::clearCustomMirrorPlaneState() noexcept {
		if (!clearMirrorReferencePresentation()) {
			return Standard_False;
		}
		_customMirrorPlane.reset();
		_mirrorPlaneOffset = 0.0;
		return Standard_True;
	}

	Standard_Boolean ObjectInteractor::replaceMirrorReferencePresentation(
		const MirrorPlaneReferenceSnapshot& theSnapshot,
		const Standard_Real theOffset) noexcept {
		try {
			OCC_CATCH_SIGNALS
			gp_Ax2 aWorldPlane;
			if (!customMirrorPlaneIsCurrent(
					theSnapshot, aWorldPlane, theOffset)) {
				return Standard_False;
			}
			gp_Trsf anOffsetTransform;
			anOffsetTransform.SetTranslation(
				gp_Vec(aWorldPlane.Direction()) * theOffset);
			BRepBuilderAPI_Transform aWorldFace(
				theSnapshot.face,
				anOffsetTransform * theSnapshot.presentationTransform,
				Standard_False,
				Standard_False);
			if (!aWorldFace.IsDone() || aWorldFace.Shape().IsNull()) {
				return Standard_False;
			}
			Handle(AIS_Shape) aReplacement =
				new AIS_Shape(aWorldFace.Shape());
			aReplacement->SetColor(Quantity_NOC_YELLOW);
			aReplacement->SetTransparency(0.35);
			aReplacement->SetZLayer(Graphic3d_ZLayerId_Topmost);
			const Handle(Prs3d_Drawer)& aDrawer =
				aReplacement->Attributes();
			if (aDrawer.IsNull()) {
				return Standard_False;
			}
			aDrawer->SetFaceBoundaryDraw(Standard_True);
			(void)aDrawer->SetupOwnFaceBoundaryAspect();
			const Handle(Prs3d_LineAspect)& aBoundary =
				aDrawer->FaceBoundaryAspect();
			if (aBoundary.IsNull()) {
				return Standard_False;
			}
			aBoundary->SetColor(Quantity_Color(Quantity_NOC_ORANGE));
			aBoundary->SetWidth(3.0);
			const std::vector<Handle(AIS_Shape)> aPrevious =
				_mirrorReferencePresentations;
			_mirrorReferencePresentations = aPrevious;
			_mirrorReferencePresentations.push_back(aReplacement);
			myContext->Display(
				aReplacement, AIS_Shaded, 0, Standard_False);
			myContext->Deactivate(aReplacement);
			if (!myContext->IsDisplayed(aReplacement)) {
				return Standard_False;
			}
			for (const Handle(AIS_Shape)& anOld : aPrevious) {
				bool shouldErase = true;
#ifdef DEBUG
				if (_debugMirrorReferenceEraseFailureCount > 0) {
					--_debugMirrorReferenceEraseFailureCount;
					shouldErase = false;
				}
#endif
				if (shouldErase) {
					myContext->Erase(anOld, Standard_False);
				}
				if (myContext->IsDisplayed(anOld)) {
					// Keep both old and replacement handles owned. Reset/Cancel can
					// retry cleanup; Apply remains disabled while the swap is partial.
					return Standard_False;
				}
			}
			_mirrorReferencePresentations = {aReplacement};
			return Standard_True;
		} catch (...) {
			return Standard_False;
		}
	}

	Standard_Boolean ObjectInteractor::completeMirrorPlanePick(
		MirrorPlaneReferenceSnapshot&& theSnapshot,
		const Standard_Real theOffset) {
		gp_Ax2 aWorldPlane;
		if (!customMirrorPlaneIsCurrent(
				theSnapshot, aWorldPlane, theOffset)) {
			return Standard_False;
		}
		if (_mirrorPlanePicking && !restoreMirrorPlanePickingModes()) {
			_mirrorPreviewState = MirrorPreviewState::Failed;
			return Standard_False;
		}
		++_mirrorPreviewGeneration;
		// Claim custom provenance before the shared builder can publish a
		// combined old/new ownership set. Even a partial display/erase failure
		// must route Reset and lifecycle cleanup through clearTrialMirrorObjects.
		_trialMirrorUsesCustomPlane = true;
		if (!tryMirrorWorldPlaneImpl(aWorldPlane)) {
			_mirrorPreviewState = MirrorPreviewState::Failed;
			return Standard_False;
		}
		if (!replaceMirrorReferencePresentation(theSnapshot, theOffset)) {
			_mirrorPreviewState = MirrorPreviewState::Failed;
			return Standard_False;
		}
		_customMirrorPlane = std::move(theSnapshot);
		_mirrorPlaneOffset = theOffset;
		_mirrorPreviewState = MirrorPreviewState::Ready;
		myContext->UpdateCurrentViewer();
		return Standard_True;
	}

	Standard_Boolean ObjectInteractor::pickMirrorPlaneAt(
		const Standard_Integer theX,
		const Standard_Integer theY) noexcept {
		if (!_mirrorPlanePicking || myContext.IsNull() || myView.IsNull()) {
			return Standard_False;
		}
		try {
			OCC_CATCH_SIGNALS
			if (myContext->MoveTo(theX, theY, myView, Standard_False)
				== AIS_SOD_Nothing) {
				return Standard_False;
			}
			myContext->InitDetected();
			for (; myContext->MoreDetected(); myContext->NextDetected()) {
				const Handle(StdSelect_BRepOwner) anOwner =
					Handle(StdSelect_BRepOwner)::DownCast(
						myContext->DetectedCurrentOwner());
				if (anOwner.IsNull() || !anOwner->HasShape()
					|| anOwner->Shape().ShapeType() != TopAbs_FACE) {
					continue;
				}
				const Handle(AIS_Shape) aPresentation =
					Handle(AIS_Shape)::DownCast(anOwner->Selectable());
				MirrorPlaneReferenceSnapshot aSnapshot;
				if (!captureMirrorPlaneReference(
						aPresentation,
						TopoDS::Face(anOwner->Shape()),
						aSnapshot)) {
					continue;
				}
				return completeMirrorPlanePick(std::move(aSnapshot), 0.0);
			}
			return Standard_False;
		} catch (...) {
			return Standard_False;
		}
	}

	Standard_Boolean ObjectInteractor::setMirrorPlaneOffset(
		const Standard_Real theOffset) noexcept {
		if (_mirrorPlanePicking || !_customMirrorPlane.has_value()
			|| !_trialMirrorUsesCustomPlane
			|| _mirrorPreviewState == MirrorPreviewState::Committing
			|| _mirrorPreviewState == MirrorPreviewState::OutcomeUnknown) {
			return Standard_False;
		}
		try {
			const auto anOffsetRange = mirrorPlaneOffsetRange();
			if (anOffsetRange.first >= anOffsetRange.second
				|| theOffset < anOffsetRange.first
				|| theOffset > anOffsetRange.second) {
				return Standard_False;
			}
			gp_Ax2 aWorldPlane;
			if (!customMirrorPlaneIsCurrent(
					*_customMirrorPlane, aWorldPlane, theOffset)) {
				return Standard_False;
			}
			++_mirrorPreviewGeneration;
			if (!tryMirrorWorldPlaneImpl(aWorldPlane)
				|| !replaceMirrorReferencePresentation(
					*_customMirrorPlane, theOffset)) {
				_mirrorPreviewState = MirrorPreviewState::Failed;
				return Standard_False;
			}
			_mirrorPlaneOffset = theOffset;
			_trialMirrorUsesCustomPlane = true;
			_mirrorPreviewState = MirrorPreviewState::Ready;
			myContext->UpdateCurrentViewer();
			return Standard_True;
		} catch (...) {
			_mirrorPreviewState = MirrorPreviewState::Failed;
			return Standard_False;
		}
	}

	Standard_Boolean ObjectInteractor::resetMirrorPlane() noexcept {
		if (_manipulatorType
				!= PrimitiveManipulatorType::PrimitiveGizmoTypeMirror
			|| _mirrorPreviewState == MirrorPreviewState::Committing
			|| _mirrorPreviewState == MirrorPreviewState::OutcomeUnknown) {
			return Standard_False;
		}
		if (_mirrorPlanePicking && !cancelMirrorPlanePicking()) {
			_mirrorPreviewState = MirrorPreviewState::Failed;
			return Standard_False;
		}
		if (_trialMirrorUsesCustomPlane && !clearTrialMirrorObjects()) {
			return Standard_False;
		}
		if (!clearCustomMirrorPlaneState()) {
			_mirrorPreviewState = MirrorPreviewState::Failed;
			return Standard_False;
		}
		_trialMirrorUsesCustomPlane = false;
		_mirrorPreviewState = MirrorPreviewState::Selecting;
		++_mirrorPreviewGeneration;
		try {
			myContext->UpdateCurrentViewer();
		} catch (...) {
			return Standard_False;
		}
		return Standard_True;
	}

	Standard_Boolean ObjectInteractor::clearTrialMirrorObjects() noexcept {
		if (_mirrorPreviewState == MirrorPreviewState::Committing
			|| _mirrorPreviewState == MirrorPreviewState::OutcomeUnknown
			|| _mirrorOwnsDocumentCommand
			|| !_pendingMirrorResults.empty()) {
			_mirrorPreviewState = MirrorPreviewState::OutcomeUnknown;
			return Standard_False;
		}
		_trialMirrorObjectsValid = false;
		std::vector<Handle(AIS_Shape)> unresolvedObjects;
		try {
			unresolvedObjects.reserve(_trialMirrorObjects.size());
		} catch (...) {
			_mirrorPreviewState = MirrorPreviewState::Failed;
			return Standard_False;
		}
		for (const Handle(AIS_Shape)& aShapePrs
			 : _trialMirrorObjects) {
			try {
#ifdef DEBUG
				if (_debugMirrorEraseFailureCount > 0) {
					--_debugMirrorEraseFailureCount;
					unresolvedObjects.push_back(aShapePrs);
					continue;
				}
#endif
				myContext->Erase(aShapePrs, Standard_False);
				if (myContext->IsDisplayed(aShapePrs)) {
					unresolvedObjects.push_back(aShapePrs);
				}
			} catch (...) {
				unresolvedObjects.push_back(aShapePrs);
			}
		}
		_trialMirrorObjects = std::move(unresolvedObjects);
		try {
			myContext->UpdateCurrentViewer();
		} catch (...) {
			// Cleanup state remains authoritative and can be retried later.
		}
		if (!_trialMirrorObjects.empty()) {
			_mirrorPreviewState = MirrorPreviewState::Failed;
			++_mirrorPreviewGeneration;
			return Standard_False;
		}
		_trialMirrorSources.clear();
		_pendingMirrorResults.clear();
		_mirrorOwnsDocumentCommand = false;
		_mirrorPreviewState = _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeMirror
			? MirrorPreviewState::Selecting
			: MirrorPreviewState::Unavailable;
		++_mirrorPreviewGeneration;
		return Standard_True;
	}

	const bool ObjectInteractor::hasTrialMirrorObjects() const {
		return _trialMirrorObjectsValid && !_trialMirrorObjects.empty();
	}

	const bool ObjectInteractor::hasUnresolvedMirrorObjects() const {
		return !_trialMirrorObjects.empty()
			|| !_pendingMirrorResults.empty()
			|| !_mirrorReferencePresentations.empty()
			|| hasCustomMirrorPlaneState()
			|| _mirrorOwnsDocumentCommand
			|| _mirrorPlanePicking;
	}

	const bool ObjectInteractor::hasActiveMirror() const noexcept {
		return _mirrorPreviewState != MirrorPreviewState::Unavailable
			|| hasUnresolvedMirrorObjects();
	}

	const bool ObjectInteractor::canApplyMirror() const noexcept {
		return !_mirrorPlanePicking
			&& ((_mirrorPreviewState == MirrorPreviewState::Ready
					&& _trialMirrorObjectsValid
					&& !_trialMirrorObjects.empty()
					&& _trialMirrorObjects.size()
						== _trialMirrorSources.size()
					&& (_trialMirrorUsesCustomPlane
						? _customMirrorPlane.has_value()
							&& _mirrorReferencePresentations.size() == 1
						: !_customMirrorPlane.has_value()
							&& _mirrorReferencePresentations.empty()))
				|| (_mirrorPreviewState
						== MirrorPreviewState::OutcomeUnknown
					&& (_mirrorOwnsDocumentCommand
						|| !_pendingMirrorResults.empty())));
	}

	MirrorPreviewState ObjectInteractor::mirrorPreviewState() const noexcept {
		return _mirrorPlanePicking
			? MirrorPreviewState::Selecting
			: _mirrorPreviewState;
	}

	std::uint64_t ObjectInteractor::mirrorPreviewGeneration() const noexcept {
		return _mirrorPreviewGeneration;
	}

	Standard_Boolean ObjectInteractor::mirrorSourcesAreCurrent() const noexcept {
		if (!_trialMirrorObjectsValid || _trialMirrorObjects.empty()
			|| _trialMirrorObjects.size() != _trialMirrorSources.size()
			|| _trialMirrorObjects.size() > kMaxMirrorPreviewBodies
			|| myDoc.IsNull() || myContext.IsNull()) {
			return Standard_False;
		}
		try {
			const Handle(TDocStd_Document) aDocument = myDoc->ChangeDocument();
			if (aDocument.IsNull()) {
				return Standard_False;
			}
			if (_trialMirrorUsesCustomPlane) {
				gp_Ax2 aCustomPlane;
				if (!_customMirrorPlane.has_value()
					|| _mirrorReferencePresentations.size() != 1
					|| _mirrorReferencePresentations.front().IsNull()
					|| !myContext->IsDisplayed(
						_mirrorReferencePresentations.front())
					|| !customMirrorPlaneIsCurrent(
						*_customMirrorPlane,
						aCustomPlane,
						_mirrorPlaneOffset)) {
					return Standard_False;
				}
			} else if (_customMirrorPlane.has_value()
				|| !_mirrorReferencePresentations.empty()) {
				return Standard_False;
			}
#ifdef DEBUG
			const Standard_Size anAggregateLimit =
				std::max<Standard_Size>(
					1,
					std::min(
						_debugMaximumMirrorTopologyNodes,
						kMaxMirrorTopologyNodes));
#else
			const Standard_Size anAggregateLimit = kMaxMirrorTopologyNodes;
#endif
			const Standard_Size aPerSourceLimit =
				std::min(kMaxMirrorSourceTopologyNodes, anAggregateLimit);
			Standard_Size anAggregateNodes = 0;
			for (std::size_t anIndex = 0;
				 anIndex < _trialMirrorSources.size(); ++anIndex) {
				const MirrorSourceSnapshot& aSource =
					_trialMirrorSources[anIndex];
				const Handle(AIS_Shape)& aResult =
					_trialMirrorObjects[anIndex];
				const TopoDS_Shape aStored = aSource.label.IsNull()
					? TopoDS_Shape()
					: XCAFDoc_ShapeTool::GetShape(aSource.label);
				Standard_Size aSourceNodes = 0;
				Standard_Size aResultNodes = 0;
				if (aSource.presentation.IsNull()
					|| aSource.label.IsNull()
					|| aSource.label.Data() != aDocument->GetData()
					|| aSource.storedShape.IsNull()
					|| aStored.IsNull()
					|| !aStored.IsEqual(aSource.storedShape)
					|| aSource.presentation->Shape().IsNull()
					|| !aSource.presentation->Shape().IsEqual(
						aSource.storedShape)
					|| TransformDiffers(
						aSource.presentation->Transformation(),
						aSource.transform)
					|| TransformDiffers(
						myDoc->ObjectTransformForLabel(aSource.label),
						aSource.transform)
					|| HasStyledMirrorSubshape(
						aDocument,
						aSource.label,
						kMaxMirrorSourceTopologyNodes)
					|| !myDoc->ShapeLabel(aSource.presentation).IsEqual(
						aSource.label)
					|| !myDoc->IsPresentationEditable(aSource.presentation)
					|| !myDoc->IsEditableFreeSimpleDefinitionLabel(
						aSource.label)
					|| !IsBRepModelingRepresentation(
						myDoc->GeometryRepresentationForLabel(
							aSource.label))
					|| myDoc->EntityIdentifierForLabel(aSource.label)
						!= aSource.entityIdentifier
					|| myDoc->DefinitionIdentifierForLabel(aSource.label)
						!= aSource.definitionIdentifier
					|| !myContext->IsDisplayed(aSource.presentation)
					|| aResult.IsNull() || aResult->Shape().IsNull()
					|| !CountBoundedMirrorTopology(
						aStored, aPerSourceLimit, aSourceNodes)
					|| aSourceNodes
						> anAggregateLimit - anAggregateNodes) {
					return Standard_False;
				}
				anAggregateNodes += aSourceNodes;
				if (!myContext->IsDisplayed(aResult)
					|| !CountBoundedMirrorTopology(
						aResult->Shape(),
						anAggregateLimit - anAggregateNodes,
						aResultNodes)
					|| !IsTopologicallyValid(aStored)
					|| !IsTopologicallyValid(aResult->Shape())) {
					return Standard_False;
				}
				anAggregateNodes += aResultNodes;
			}
			return Standard_True;
		} catch (...) {
			return Standard_False;
		}
	}

	ObjectInteractor::MirrorDocumentState
	ObjectInteractor::inspectPendingMirrorResults() const noexcept {
		try {
			if (myDoc.IsNull()) {
				return MirrorDocumentState::Unavailable;
			}
			const Handle(TDocStd_Document) aDocument = myDoc->ChangeDocument();
			if (aDocument.IsNull()) {
				return MirrorDocumentState::Unavailable;
			}
			if (aDocument->HasOpenCommand()) {
				return _mirrorOwnsDocumentCommand
					? MirrorDocumentState::OpenCommand
					: MirrorDocumentState::Unavailable;
			}
			if (_pendingMirrorResults.empty()) {
				return MirrorDocumentState::None;
			}
			Standard_Size aMissingCount = 0;
			Standard_Size aCommittedCount = 0;
			for (const MirrorPendingResult& aResult
				 : _pendingMirrorResults) {
				if (aResult.label.IsNull()
					|| aResult.label.Data() != aDocument->GetData()) {
					return MirrorDocumentState::PartialOrMismatched;
				}
				const TopoDS_Shape aStored =
					XCAFDoc_ShapeTool::GetShape(aResult.label);
				if (aStored.IsNull()) {
					++aMissingCount;
					continue;
				}
				if (aResult.expectedShape.IsNull()
					|| !aStored.IsEqual(aResult.expectedShape)
					|| aResult.entityIdentifier.empty()
					|| aResult.definitionIdentifier.empty()
					|| myDoc->EntityIdentifierForLabel(aResult.label)
						!= aResult.entityIdentifier
					|| myDoc->DefinitionIdentifierForLabel(aResult.label)
						!= aResult.definitionIdentifier
					|| myDoc->GeometryRepresentationForLabel(aResult.label)
						!= OcctGeometryRepresentation::BRep
					|| !myDoc->IsEditableFreeSimpleDefinitionLabel(
						aResult.label)) {
					return MirrorDocumentState::PartialOrMismatched;
				}
				++aCommittedCount;
			}
			if (aCommittedCount == _pendingMirrorResults.size()) {
				return MirrorDocumentState::AllCommitted;
			}
			if (aMissingCount == _pendingMirrorResults.size()) {
				return MirrorDocumentState::None;
			}
			return MirrorDocumentState::PartialOrMismatched;
		} catch (...) {
			return MirrorDocumentState::Unavailable;
		}
	}

	Standard_Boolean ObjectInteractor::abortOwnedMirrorCommand() noexcept {
		if (!_mirrorOwnsDocumentCommand || myDoc.IsNull()) {
			return Standard_False;
		}
		try {
			const Handle(TDocStd_Document) aDocument = myDoc->ChangeDocument();
			if (aDocument.IsNull()) {
				return Standard_False;
			}
			if (aDocument->HasOpenCommand()) {
#ifdef DEBUG
				if (_debugMirrorAbortFailureCount > 0) {
					--_debugMirrorAbortFailureCount;
					return Standard_False;
				}
#endif
				aDocument->AbortCommand();
			}
			if (aDocument->HasOpenCommand()) {
				return Standard_False;
			}
			_mirrorOwnsDocumentCommand = false;
			return Standard_True;
		} catch (...) {
			return Standard_False;
		}
	}

	MirrorApplyResult ObjectInteractor::finishCommittedMirror() noexcept {
		// The result labels are the only durable evidence needed to reconcile a
		// commit whose transient custom reference could not be erased. Retain
		// them and enter Apply-only recovery until cleanup is proven; a later
		// Apply re-inspects the same labels and retries without opening a command.
		if (!clearCustomMirrorPlaneState()) {
			_mirrorOwnsDocumentCommand = false;
			_mirrorPreviewState = MirrorPreviewState::OutcomeUnknown;
			return MirrorApplyResult::NoChange;
		}
		try {
			myDoc->NotifyChanges();
		} catch (...) {
		}
		// The same AIS handles are now backed by the committed document labels.
		// Release transient ownership without erasing document-owned geometry.
		_trialMirrorObjects.clear();
		_trialMirrorSources.clear();
		_pendingMirrorResults.clear();
		_trialMirrorObjectsValid = false;
		_mirrorOwnsDocumentCommand = false;
		_trialMirrorUsesCustomPlane = false;
		_mirrorPreviewState = MirrorPreviewState::Unavailable;
		++_mirrorPreviewGeneration;
		return MirrorApplyResult::AppliedNeedsDocumentRedraw;
	}

	MirrorApplyResult ObjectInteractor::applyMirror() noexcept {
		if (!hasActiveMirror()) {
			return MirrorApplyResult::NoChange;
		}
		if (_mirrorPlanePicking && !cancelMirrorPlanePicking()) {
			_mirrorPreviewState = MirrorPreviewState::Failed;
			return MirrorApplyResult::NoChange;
		}

		auto inspectAfterCommit = [&]() noexcept {
#ifdef DEBUG
			if (_debugMirrorPostCommitInspectFailureCount > 0) {
				--_debugMirrorPostCommitInspectFailureCount;
				return MirrorDocumentState::Unavailable;
			}
#endif
			return inspectPendingMirrorResults();
		};

		if (_mirrorPreviewState == MirrorPreviewState::OutcomeUnknown
			|| _mirrorOwnsDocumentCommand
			|| !_pendingMirrorResults.empty()) {
			MirrorDocumentState aState = inspectAfterCommit();
			if (aState == MirrorDocumentState::OpenCommand
				&& abortOwnedMirrorCommand()) {
				aState = inspectAfterCommit();
			}
			if (aState == MirrorDocumentState::AllCommitted) {
				return finishCommittedMirror();
			}
			if (aState == MirrorDocumentState::None) {
				_pendingMirrorResults.clear();
				_mirrorOwnsDocumentCommand = false;
				_mirrorPreviewState = hasTrialMirrorObjects()
					? MirrorPreviewState::Ready
					: MirrorPreviewState::Failed;
				return MirrorApplyResult::NoChange;
			}
			_mirrorPreviewState = MirrorPreviewState::OutcomeUnknown;
			return MirrorApplyResult::NoChange;
		}

		if (_mirrorPreviewState != MirrorPreviewState::Ready
			|| !canApplyMirror()) {
			return MirrorApplyResult::NoChange;
		}
		if (!mirrorSourcesAreCurrent()) {
			if (!cancelMirror()) {
				_mirrorPreviewState = MirrorPreviewState::Failed;
			}
			return MirrorApplyResult::NoChange;
		}

		Handle(TDocStd_Document) aDocument;
		try {
			aDocument = myDoc->ChangeDocument();
			if (aDocument.IsNull() || aDocument->HasOpenCommand()) {
				return MirrorApplyResult::NoChange;
			}
		} catch (...) {
			return MirrorApplyResult::NoChange;
		}
		std::vector<TDF_Label> aSourceLabels;
		try {
			aSourceLabels.reserve(_trialMirrorSources.size());
			for (const MirrorSourceSnapshot& aSource : _trialMirrorSources) {
				aSourceLabels.push_back(aSource.label);
			}
		} catch (...) {
			return MirrorApplyResult::NoChange;
		}
		if (!myDoc->CanDuplicateGeometryDefinitions(aSourceLabels)) {
			if (!cancelMirror()) {
				_mirrorPreviewState = MirrorPreviewState::Failed;
			}
			return MirrorApplyResult::NoChange;
		}

		const auto retainRetryableOrUnknown = [&]() noexcept {
			if (abortOwnedMirrorCommand()
				&& inspectPendingMirrorResults()
					== MirrorDocumentState::None) {
				_pendingMirrorResults.clear();
				_mirrorPreviewState = MirrorPreviewState::Ready;
			} else {
				_mirrorPreviewState = MirrorPreviewState::OutcomeUnknown;
			}
			return MirrorApplyResult::NoChange;
		};

		_mirrorPreviewState = MirrorPreviewState::Committing;
		_pendingMirrorResults.clear();
		try {
			OCC_CATCH_SIGNALS
			// No command was open at admission. Claim this synchronous command
			// attempt before calling into OCAF so an exception thrown after it
			// opens can still be aborted deterministically. abortOwnedMirrorCommand
			// also clears this intent safely when NewCommand opened nothing.
			_mirrorOwnsDocumentCommand = true;
			aDocument->NewCommand();
			if (!aDocument->HasOpenCommand()) {
				_mirrorOwnsDocumentCommand = false;
				_mirrorPreviewState = MirrorPreviewState::Ready;
				return MirrorApplyResult::NoChange;
			}
#ifdef DEBUG
			if (_debugMirrorTransactionFailureCount > 0) {
				--_debugMirrorTransactionFailureCount;
				return retainRetryableOrUnknown();
			}
#endif
			_pendingMirrorResults.reserve(_trialMirrorObjects.size());
			for (std::size_t anIndex = 0;
				 anIndex < _trialMirrorObjects.size(); ++anIndex) {
				const Handle(AIS_Shape)& aShape =
					_trialMirrorObjects[anIndex];
				const MirrorSourceSnapshot& aSource =
					_trialMirrorSources[anIndex];
				const TDF_Label aLabel = myDoc->AddShape(
					aShape, OcctGeometryRepresentation::BRep);
				if (aLabel.IsNull()) {
					return retainRetryableOrUnknown();
				}
				_pendingMirrorResults.push_back({
					aLabel,
					myDoc->EntityIdentifierForLabel(aLabel),
					myDoc->DefinitionIdentifierForLabel(aLabel),
					aShape->Shape(),
				});
				if (_pendingMirrorResults.back().entityIdentifier.empty()
					|| _pendingMirrorResults.back()
						.definitionIdentifier.empty()
					|| myDoc->GeometryRepresentationForLabel(aLabel)
						!= OcctGeometryRepresentation::BRep
					|| !myDoc->CopyGeometryRepresentation(
						aSource.label, aLabel)
					|| !myDoc->CopyObjectAppearance(
						aSource.label, aLabel)) {
					return retainRetryableOrUnknown();
				}
				myDoc->LoadObjectMeterial(aLabel, aShape);
			}
			if (!myDoc->ValidateGeometryRepresentations()) {
				return retainRetryableOrUnknown();
			}

			try {
				Standard_Boolean aCommitReported =
					aDocument->CommitCommand();
#ifdef DEBUG
				const Standard_Integer aCommitMode =
					_debugMirrorCommitMode;
				_debugMirrorCommitMode = 0;
				if (aCommitMode == 1) {
					aCommitReported = Standard_False;
				} else if (aCommitMode == 2) {
					throw Standard_Failure(
						"Injected Mirror commit exception after close");
				}
#endif
				(void)aCommitReported;
			} catch (...) {
				// Authoritative OCAF inspection below distinguishes a closed commit
				// from an open command. Never infer document truth from this throw.
			}
		} catch (...) {
			return retainRetryableOrUnknown();
		}

		MirrorDocumentState aState = inspectAfterCommit();
		if (aState == MirrorDocumentState::OpenCommand
			&& abortOwnedMirrorCommand()) {
			aState = inspectAfterCommit();
		}
		if (aState == MirrorDocumentState::AllCommitted) {
			return finishCommittedMirror();
		}
		if (aState == MirrorDocumentState::None) {
			_pendingMirrorResults.clear();
			_mirrorOwnsDocumentCommand = false;
			_mirrorPreviewState = MirrorPreviewState::Ready;
			return MirrorApplyResult::NoChange;
		}
		_mirrorPreviewState = MirrorPreviewState::OutcomeUnknown;
		return MirrorApplyResult::NoChange;
	}

	Standard_Boolean ObjectInteractor::cancelMirror() noexcept {
		if (!hasActiveMirror()) {
			return Standard_True;
		}
		if (_mirrorPlanePicking && !cancelMirrorPlanePicking()) {
			_mirrorPreviewState = MirrorPreviewState::Failed;
			return Standard_False;
		}
		if (_mirrorPreviewState == MirrorPreviewState::Committing
			|| _mirrorPreviewState == MirrorPreviewState::OutcomeUnknown
			|| _mirrorOwnsDocumentCommand
			|| !_pendingMirrorResults.empty()) {
			_mirrorPreviewState = MirrorPreviewState::OutcomeUnknown;
			return Standard_False;
		}
		if (!clearTrialMirrorObjects()) {
			return Standard_False;
		}
		if (!clearCustomMirrorPlaneState()) {
			_mirrorPreviewState = MirrorPreviewState::Failed;
			return Standard_False;
		}
		_trialMirrorUsesCustomPlane = false;
		_mirrorPreviewState = MirrorPreviewState::Unavailable;
		++_mirrorPreviewGeneration;
		return Standard_True;
	}

#ifdef DEBUG
	MirrorPreviewDebugState
	ObjectInteractor::debugMirrorPreviewState() const noexcept {
		MirrorPreviewDebugState aState;
		aState.state = mirrorPreviewState();
		aState.generation = _mirrorPreviewGeneration;
		aState.previewBodyCount = _trialMirrorObjects.size();
		aState.pendingResultCount = _pendingMirrorResults.size();
		aState.activeOperation = hasActiveMirror();
		aState.previewValid = _trialMirrorObjectsValid;
		aState.canApply = canApplyMirror();
		aState.ownsDocumentCommand = _mirrorOwnsDocumentCommand;
		aState.pickingCustomPlane = _mirrorPlanePicking;
		aState.hasCustomPlane = hasCustomMirrorPlane();
		aState.previewUsesCustomPlane = _trialMirrorUsesCustomPlane;
		aState.manipulatorAttached = isManipulatorAttached();
		aState.referencePresentationCount =
			_mirrorReferencePresentations.size();
		aState.customPlaneOffset = _mirrorPlaneOffset;
		const auto anOffsetRange = mirrorPlaneOffsetRange();
		aState.customPlaneMinimumOffset = anOffsetRange.first;
		aState.customPlaneMaximumOffset = anOffsetRange.second;
		try {
			const Handle(TDocStd_Document) aDocument = myDoc.IsNull()
				? Handle(TDocStd_Document)()
				: myDoc->ChangeDocument();
			aState.documentCommandOpen = !aDocument.IsNull()
				&& aDocument->HasOpenCommand();
		} catch (...) {
			aState.documentCommandOpen = Standard_True;
		}
		return aState;
	}

	void ObjectInteractor::debugSetMirrorTransactionFailureCount(
		const Standard_Size count) noexcept {
		_debugMirrorTransactionFailureCount = count;
	}

	void ObjectInteractor::debugSetMirrorAbortFailureCount(
		const Standard_Size count) noexcept {
		_debugMirrorAbortFailureCount = count;
	}

	void ObjectInteractor::debugSetMirrorEraseFailureCount(
		const Standard_Size count) noexcept {
		_debugMirrorEraseFailureCount = count;
	}

	void ObjectInteractor::debugSetMirrorReferenceEraseFailureCount(
		const Standard_Size count) noexcept {
		_debugMirrorReferenceEraseFailureCount = count;
	}

	void ObjectInteractor::debugSetMirrorCommitMode(
		const Standard_Integer mode) noexcept {
		_debugMirrorCommitMode = mode >= 0 && mode <= 2 ? mode : 0;
	}

	void ObjectInteractor::debugSetMirrorPostCommitInspectFailureCount(
		const Standard_Size count) noexcept {
		_debugMirrorPostCommitInspectFailureCount = count;
	}

	void ObjectInteractor::debugSetMaximumMirrorTopologyNodes(
		const Standard_Size limit) noexcept {
		_debugMaximumMirrorTopologyNodes = std::max<Standard_Size>(
			1, std::min(limit, kMaxMirrorTopologyNodes));
	}

	void ObjectInteractor::debugSetMaximumMirrorReferenceTopologyNodes(
		const Standard_Size limit) noexcept {
		_debugMaximumMirrorReferenceTopologyNodes =
			std::max<Standard_Size>(
				1,
				std::min(
					limit,
					kMaxMirrorReferenceTopologyNodes));
	}

	void ObjectInteractor::debugSetMaximumMirrorReferenceFaces(
		const Standard_Size limit) noexcept {
		_debugMaximumMirrorReferenceFaces = std::max<Standard_Size>(
			1, std::min(limit, kMaxMirrorReferenceFaces));
	}

	Standard_Boolean
	ObjectInteractor::debugMutateFirstMirrorSourcePersistedTransform() noexcept {
		if (!mirrorSourcesAreCurrent() || _trialMirrorSources.empty()
			|| myDoc.IsNull()) {
			return Standard_False;
		}
		Handle(TDocStd_Document) aDocument;
		try {
			OCC_CATCH_SIGNALS
			aDocument = myDoc->ChangeDocument();
			const MirrorSourceSnapshot& aSource =
				_trialMirrorSources.front();
			if (aDocument.IsNull() || aDocument->HasOpenCommand()
				|| aSource.label.IsNull()
				|| aSource.label.Data() != aDocument->GetData()) {
				return Standard_False;
			}
			const Standard_Real anOriginalX =
				myDoc->ObjectTransformForLabel(aSource.label)
					.TranslationPart().X();
			if (!std::isfinite(anOriginalX)) {
				return Standard_False;
			}
			aDocument->NewCommand();
			if (!aDocument->HasOpenCommand()) {
				return Standard_False;
			}
			TDataStd_Real::Set(
				aSource.label.FindChild(1),
				anOriginalX + 1.0);
			try {
				(void)aDocument->CommitCommand();
			} catch (...) {
			}
			if (aDocument->HasOpenCommand()) {
				aDocument->AbortCommand();
				return Standard_False;
			}
			return TransformDiffers(
					myDoc->ObjectTransformForLabel(aSource.label),
					aSource.transform)
				&& !TransformDiffers(
					aSource.presentation->Transformation(),
					aSource.transform);
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

	Standard_Boolean ObjectInteractor::debugTryMirrorPlane(
		const std::string& theEntityIdentifier,
		const Standard_Integer theFaceTopologyIndex,
		const Standard_Real theOffset) noexcept {
		if (theEntityIdentifier.empty() || theFaceTopologyIndex < 0
			|| _manipulatorType
				!= PrimitiveManipulatorType::PrimitiveGizmoTypeMirror
			|| !std::isfinite(theOffset)) {
			return Standard_False;
		}
		if (!_mirrorPlanePicking && !beginMirrorPlanePicking()) {
			return Standard_False;
		}
		try {
			OCC_CATCH_SIGNALS
			AIS_ListOfInteractive aDisplayed;
			myContext->DisplayedObjects(aDisplayed);
			for (AIS_ListIteratorOfListOfInteractive anIterator(aDisplayed);
				 anIterator.More(); anIterator.Next()) {
				const Handle(AIS_Shape) aPresentation =
					Handle(AIS_Shape)::DownCast(anIterator.Value());
				const TDF_Label aLabel =
					myDoc->ShapeLabel(aPresentation);
				if (aPresentation.IsNull() || aLabel.IsNull()
					|| myDoc->EntityIdentifierForLabel(aLabel)
						!= theEntityIdentifier) {
					continue;
				}
				const Standard_Size aMaximumTopologyNodes = std::min(
					kMaxMirrorReferenceTopologyNodesPerPresentation,
					_debugMaximumMirrorReferenceTopologyNodes);
				const Standard_Size aMaximumFaces = std::min(
					kMaxMirrorReferenceFacesPerPresentation,
					_debugMaximumMirrorReferenceFaces);
				Standard_Size aTopologyNodeCount = 0;
				TopTools_IndexedMapOfShape aFaces;
				if (aMaximumTopologyNodes == 0 || aMaximumFaces == 0
					|| !MapBoundedMirrorReferenceFaces(
						aPresentation->Shape(),
						aMaximumTopologyNodes,
						aMaximumFaces,
						aTopologyNodeCount,
						aFaces)) {
					return Standard_False;
				}
				const Standard_Integer aFaceIndex =
					theFaceTopologyIndex + 1;
				if (aFaceIndex <= 0 || aFaceIndex > aFaces.Extent()) {
					return Standard_False;
				}
				MirrorPlaneReferenceSnapshot aSnapshot;
				if (!captureMirrorPlaneReference(
						aPresentation,
						TopoDS::Face(aFaces(aFaceIndex)),
						aSnapshot)) {
					return Standard_False;
				}
				return completeMirrorPlanePick(
					std::move(aSnapshot), theOffset);
			}
			return Standard_False;
		} catch (...) {
			return Standard_False;
		}
	}
#endif
}
