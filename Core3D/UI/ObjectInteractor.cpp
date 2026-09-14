//
//  ObjectInteractor.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 19.04.2024.
//

#include "ObjectInteractor.hpp"
#include "OrdinaryEditController.hpp"
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
#include <TDF_Attribute.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_Real.hxx>
#include <TDataStd_Name.hxx>
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
#include <unordered_set>
#include <utility>

namespace core3d {
	namespace {
        // Preserve absent legacy units as distinct authority. The established
        // legacy interpretation is millimetres; reading must not add metadata.
        bool ReadMirrorDocumentUnits(const Handle(TDocStd_Document)& document,
                                     double& metersPerUnit, bool& present) noexcept {
            metersPerUnit = 0.001; present = false;
            if (document.IsNull()) return false;
            try {
                present = XCAFDoc_DocumentTool::GetLengthUnit(document, metersPerUnit);
                return present ? std::isfinite(metersPerUnit) && metersPerUnit > 0
                               : metersPerUnit == 0.001;
            } catch (...) { return false; }
        }

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
				case PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray:
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
				case PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray:
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
                // BRepCheck_Edge::InContext in OCCT 7.8 dereferences the face
                // surface. A malformed mixed BRep/triangulation candidate
                // must fail admission before reaching that unchecked access.
                for (TopExp_Explorer faces(shape, TopAbs_FACE); faces.More(); faces.Next()) {
                    TopLoc_Location location;
                    if (BRep_Tool::Surface(TopoDS::Face(faces.Current()), location).IsNull()) {
                        return Standard_False;
                    }
                }
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

        bool IsAdmittedGestureTransform(const gp_Trsf& transform) noexcept {
            for (Standard_Integer row = 1; row <= 3; ++row) {
                for (Standard_Integer column = 1; column <= 4; ++column) {
                    const Standard_Real value = transform.Value(row, column);
                    if (!std::isfinite(value)
                        || (column == 4 && std::abs(value)
                            > limits::kMaximumModelCoordinateMagnitude)) {
                        return false;
                    }
                }
            }
            return std::isfinite(transform.ScaleFactor())
                && std::abs(transform.ScaleFactor())
                    > std::numeric_limits<Standard_Real>::epsilon();
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

		bool ReferenceAxisDiffers(
			const OcctReferenceAxis& theLeft,
			const OcctReferenceAxis& theRight) noexcept {
			constexpr Standard_Real aTolerance = 1.0e-12;
			try {
				return theLeft.pivotSpace != theRight.pivotSpace
					|| theLeft.directionSpace != theRight.directionSpace
					|| !theLeft.pivot.IsEqual(theRight.pivot, aTolerance)
					|| !theLeft.direction.IsEqual(
						theRight.direction, aTolerance);
			} catch (...) {
				return true;
			}
		}

		bool TryExpectedBakedReferenceAxis(
			const OcctReferenceAxisReadState theSourceState,
			const OcctReferenceAxis& theSource,
			const gp_Trsf& theBakeTransform,
			OcctReferenceAxisReadState& theExpectedState,
			OcctReferenceAxis& theExpected) noexcept {
			theExpectedState = OcctReferenceAxisReadState::Invalid;
			theExpected = theSource;
			if (theSourceState == OcctReferenceAxisReadState::Invalid) {
				return false;
			}
			try {
				for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
					for (Standard_Integer aColumn = 1; aColumn <= 4;
						 aColumn++) {
						if (!std::isfinite(
								theBakeTransform.Value(aRow, aColumn))) {
							return false;
						}
					}
				}
				if (theExpected.pivotSpace == OcctReferenceSpace::Object) {
					theExpected.pivot.Transform(theBakeTransform);
				}
				if (!std::isfinite(theExpected.pivot.X())
					|| !std::isfinite(theExpected.pivot.Y())
					|| !std::isfinite(theExpected.pivot.Z())
					|| std::abs(theExpected.pivot.X())
						> limits::kMaximumModelCoordinateMagnitude
					|| std::abs(theExpected.pivot.Y())
						> limits::kMaximumModelCoordinateMagnitude
					|| std::abs(theExpected.pivot.Z())
						> limits::kMaximumModelCoordinateMagnitude) {
					return false;
				}
				if (theExpected.directionSpace
						== OcctReferenceSpace::Object) {
					gp_Vec aDirection(theExpected.direction);
					aDirection.Transform(theBakeTransform);
					const Standard_Real aSquaredMagnitude =
						aDirection.SquareMagnitude();
					if (!std::isfinite(aDirection.X())
						|| !std::isfinite(aDirection.Y())
						|| !std::isfinite(aDirection.Z())
						|| !std::isfinite(aSquaredMagnitude)
						|| aSquaredMagnitude
							<= std::numeric_limits<Standard_Real>::epsilon()) {
						return false;
					}
					theExpected.direction = gp_Dir(aDirection);
				}
				if (!std::isfinite(theExpected.direction.X())
					|| !std::isfinite(theExpected.direction.Y())
					|| !std::isfinite(theExpected.direction.Z())) {
					return false;
				}

				const OcctReferenceAxis aDefault;
				theExpectedState =
					theSourceState
							== OcctReferenceAxisReadState::ImplicitDefault
						&& !ReferenceAxisDiffers(theExpected, aDefault)
					? OcctReferenceAxisReadState::ImplicitDefault
					: OcctReferenceAxisReadState::Authored;
				return true;
			} catch (...) {
				theExpectedState = OcctReferenceAxisReadState::Invalid;
				return false;
			}
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
				// A baked reflection can leave the plane's Ax3 indirect. Its
				// stored Z axis then opposes the surface U x V normal. Apply
				// face orientation to the parametric normal before transforming
				// it by the authored presentation placement.
				theWorldNormal = aPlane.XAxis().Direction().Crossed(
					aPlane.YAxis().Direction());
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
            std::make_shared<LinearArrayOperationController>(context, doc))
        , _radialArrayController(
            std::make_shared<RadialArrayOperationController>(context, doc)) {
    }

    void ObjectInteractor::selectLastObject() {
        if (hasUnresolvedEdit()) { return; }
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
				== PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray
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
		if (hasUnresolvedEdit()) { return; }
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
		if (positionManipulatorAtExactSavedGroupOrigin()
			== SavedGroupPivotResult::Failed) {
			// Select All retains its exact selection, but malformed saved-group
			// authority must never expose a gizmo at a guessed pivot.
			_manipulator->Detach();
			_manipulatorSourceLabels.clear();
		}
		myContext->UpdateCurrentViewer();
	}

    ObjectInteractor::SavedGroupPivotResult ObjectInteractor::positionManipulatorAtExactSavedGroupOrigin() noexcept {
        try {
            if (_manipulator.IsNull() || !_manipulator->IsAttached() || _manipulatorSourceLabels.size()<2)
                return SavedGroupPivotResult::NotApplicable;
            std::unordered_set<std::string> selected;
            for(const auto& pair:_manipulatorSourceLabels) {
                const auto id=myDoc->EntityIdentifierForLabel(pair.second);
                if(id.empty()||!selected.insert(id).second)return SavedGroupPivotResult::Failed;
            }
            OcctSavedGroupState groups;if(!myDoc->CaptureSavedGroups(groups))return SavedGroupPivotResult::Failed;
            for(const auto& group:groups.groups) {
                if(group.members.size()!=selected.size())continue;
                bool exact=true;for(const auto& member:group.members)exact=exact&&selected.count(myDoc->EntityIdentifierForLabel(member));
                if(!exact)continue;
#ifdef DEBUG
                if(_debugSavedGroupPivotFailures>0){--_debugSavedGroupPivotFailures;return SavedGroupPivotResult::Failed;}
#endif
                if(group.originPresent) {
                    if(!OcctDocument::IsAdmittedSavedGroupOrigin(group.origin))return SavedGroupPivotResult::Failed;
                    auto position=_manipulator->Position();position.SetLocation(group.origin);
                    _manipulator->SetPosition(position);
                    return _manipulator->Position().Location().IsEqual(group.origin,0.0)
                        ?SavedGroupPivotResult::Positioned:SavedGroupPivotResult::Failed;
                }
                const auto attached=_manipulator->Objects();
                if(attached.IsNull()||static_cast<std::size_t>(attached->Size())!=selected.size())return SavedGroupPivotResult::Failed;
                Handle(Core3DManipulatorObjectSequence) copy=new Core3DManipulatorObjectSequence();
                for(Core3DManipulatorObjectSequence::Iterator it(*attached);it.More();it.Next())copy->Append(it.Value());
                _manipulator->Detach();_manipulator->Attach(copy);
                const auto rebound=_manipulator->Objects();
                return _manipulator->IsAttached()&&!rebound.IsNull()
                    &&static_cast<std::size_t>(rebound->Size())==selected.size()
                    &&OcctDocument::IsAdmittedSavedGroupOrigin(_manipulator->Position().Location())
                    ?SavedGroupPivotResult::Positioned:SavedGroupPivotResult::Failed;
            }
            return SavedGroupPivotResult::NotApplicable;
        } catch (...) { return SavedGroupPivotResult::Failed; }
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
            if (hasUnresolvedEdit()) {
                return PresentationOverlayCaptureStatus::Unsafe;
            }
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

    Standard_Boolean
    ObjectInteractor::captureSelectionModeSuspendedPresentations(
        std::vector<Handle(AIS_Shape)>& thePresentations) const noexcept
    {
        thePresentations.clear();
        if (myContext.IsNull()) {
            return Standard_False;
        }
        if (_manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray
            && _linearArrayController != nullptr) {
            return _linearArrayController
                ->captureSelectionModeSuspendedPresentations(
                    thePresentations);
        }
        if (_manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray
            && _radialArrayController != nullptr) {
            return _radialArrayController
                ->captureSelectionModeSuspendedPresentations(
                    thePresentations);
        }
        if (_booleanOpController != nullptr) {
            if (_manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeUnion) {
                return _booleanOpController
                    ->captureSelectionModeSuspendedPresentations(
                        BooleanAction::BooleanUnion,
                        thePresentations);
            }
            if (_manipulatorType
                == PrimitiveManipulatorType::PrimitiveGizmoTypeIntersect) {
                return _booleanOpController
                    ->captureSelectionModeSuspendedPresentations(
                        BooleanAction::BooleanIntersect,
                        thePresentations);
            }
        }
        if (_manipulatorType
                != PrimitiveManipulatorType::PrimitiveGizmoTypeMirror
            || _mirrorPlanePicking || !hasActiveMirror()
            || (_trialMirrorObjects.empty()
                && _mirrorReferencePresentations.empty())
            || _trialMirrorObjects.size() > kMaxMirrorPreviewBodies
            || _mirrorReferencePresentations.size() > 1) {
            return Standard_False;
        }
        try {
            OCC_CATCH_SIGNALS
            thePresentations.reserve(
                _trialMirrorObjects.size()
                + _mirrorReferencePresentations.size());
            const auto appendSuspended = [&](const Handle(AIS_Shape)& aShape) {
                TColStd_ListOfInteger activeModes;
                if (aShape.IsNull() || aShape->Shape().IsNull()
                    || !myContext->IsDisplayed(aShape)) {
                    return Standard_False;
                }
                myContext->ActivatedModes(aShape, activeModes);
                if (!activeModes.IsEmpty()) {
                    return Standard_False;
                }
                thePresentations.push_back(aShape);
                return Standard_True;
            };
            for (const Handle(AIS_Shape)& aShape : _trialMirrorObjects) {
                if (!appendSuspended(aShape)) {
                    thePresentations.clear();
                    return Standard_False;
                }
            }
            for (const Handle(AIS_Shape)& aShape :
                 _mirrorReferencePresentations) {
                if (!appendSuspended(aShape)) {
                    thePresentations.clear();
                    return Standard_False;
                }
            }
            return !thePresentations.empty();
        } catch (...) {
            thePresentations.clear();
            return Standard_False;
        }
    }

    void ObjectInteractor::deleteSelected() {
        if (hasUnresolvedEdit()) { return; }
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

	ObjectInteractor::DuplicateDocumentState
	ObjectInteractor::inspectPendingDuplicateResults() noexcept {
		try {
			if (myDoc.IsNull()) {
				return DuplicateDocumentState::Unavailable;
			}
			const Handle(TDocStd_Document) document =
				myDoc->ChangeDocument();
			if (document.IsNull()) {
				return DuplicateDocumentState::Unavailable;
			}
			if (document->HasOpenCommand()) {
				if (duplicateOwnedCommandIsCurrent(document)) {
					return DuplicateDocumentState::OpenCommand;
				}
				_duplicateOwnsDocumentCommand = false;
				_duplicateOwnedTransaction = -1;
				_duplicateOwnedDocumentTime = -1;
				_duplicateOwnedMarkerValue = 0;
				return DuplicateDocumentState::Unavailable;
			}
			// Ownership cannot survive an observed close. Retaining the result
			// ledger is safe, but retaining this bit could later make Duplicate
			// abort a different tool's newly opened transaction.
			_duplicateOwnsDocumentCommand = false;
			_duplicateOwnedTransaction = -1;
			_duplicateOwnedDocumentTime = -1;
			_duplicateOwnedMarkerValue = 0;
			if (_pendingDuplicateResults.empty()) {
				return DuplicateDocumentState::None;
			}
			Standard_Size missingCount = 0;
			Standard_Size committedCount = 0;
			for (const DuplicatePendingResult& duplicate :
				 _pendingDuplicateResults) {
                profile::Record sourceProfile;
                OcctObjectNameState sourceName;
                if (duplicate.sourceLabel.IsNull()
                    || duplicate.sourceLabel.Data() != document->GetData()
                    || duplicate.originalOwnerShape.IsNull()
                    || !duplicate.originalOwnerShape.IsEqual(XCAFDoc_ShapeTool::GetShape(duplicate.sourceLabel))
                    || !profile::Read(document, duplicate.sourceLabel, sourceProfile)
                    || !sourceProfile.IsEqual(duplicate.originalProfile)
                    || !myDoc->CaptureObjectNameStateForLabel(duplicate.sourceLabel, sourceName)
                    || !sourceName.IsEqual(duplicate.originalName)
                    || sourceProfile.IsCurrent(document, duplicate.sourceLabel) != duplicate.originalProfileCurrent
                    || sourceName.object.enclosure.IsCurrent(document, duplicate.sourceLabel)
                        != duplicate.originalEnclosureCurrent) {
                    return DuplicateDocumentState::PartialOrMismatched;
                }
				if (duplicate.resultLabel.IsNull()) {
					++missingCount;
					continue;
				}
				if (duplicate.resultLabel.Data() != document->GetData()) {
					return DuplicateDocumentState::PartialOrMismatched;
				}
				const TopoDS_Shape stored = XCAFDoc_ShapeTool::GetShape(
					duplicate.resultLabel);
                profile::Record actualProfile;
                enclosure::Record actualEnclosure;
                if (!profile::Read(document, duplicate.resultLabel, actualProfile)
                    || !enclosure::Read(document, duplicate.resultLabel, actualEnclosure)) {
                    return DuplicateDocumentState::PartialOrMismatched;
                }
				if (stored.IsNull()) {
                    if (!actualProfile.label.IsNull() || !actualEnclosure.label.IsNull()) { return DuplicateDocumentState::PartialOrMismatched; }
					++missingCount;
					continue;
				}
				OcctReferenceAxis storedReferenceAxis;
				const OcctReferenceAxisReadState storedReferenceAxisState =
					myDoc->ReadReferenceAxisForLabel(
						duplicate.resultLabel,
						storedReferenceAxis);
                OcctObjectNameState actualName;
				if (!duplicate.profileCandidateSealed
                    || !duplicate.enclosureCandidateSealed
                    || !actualEnclosure.IsEqual(duplicate.candidateEnclosure)
                    || actualEnclosure.IsCurrent(document, duplicate.resultLabel) != duplicate.originalEnclosureCurrent
                    || !myDoc->CaptureObjectNameStateForLabel(duplicate.resultLabel, actualName)
                    || actualName.namePresent != duplicate.originalName.namePresent
                    || (actualName.namePresent && !actualName.name.IsEqual(duplicate.originalName.name))
                    || !actualProfile.IsEqual(duplicate.candidateProfile)
                    || actualProfile.IsCurrent(document, duplicate.resultLabel) != duplicate.originalProfileCurrent
                    || duplicate.expectedShape.IsNull()
					|| !stored.IsEqual(duplicate.expectedShape)
					|| duplicate.entityIdentifier.empty()
					|| duplicate.definitionIdentifier.empty()
					|| myDoc->EntityIdentifierForLabel(
						duplicate.resultLabel)
						!= duplicate.entityIdentifier
					|| myDoc->DefinitionIdentifierForLabel(
						duplicate.resultLabel)
						!= duplicate.definitionIdentifier
					|| myDoc->GeometryRepresentationForLabel(
						duplicate.resultLabel)
						!= duplicate.representation
					|| !myDoc->IsEditableFreeSimpleDefinitionLabel(
						duplicate.resultLabel)
					|| TransformDiffers(
						myDoc->ObjectTransformForLabel(
							duplicate.resultLabel),
						duplicate.expectedTransform)
					|| storedReferenceAxisState
						!= duplicate.expectedReferenceAxisState
					|| storedReferenceAxisState
						== OcctReferenceAxisReadState::Invalid
					|| ReferenceAxisDiffers(
						storedReferenceAxis,
						duplicate.expectedReferenceAxis)) {
					return DuplicateDocumentState::PartialOrMismatched;
				}
				++committedCount;
			}
            const auto& groups = _pendingDuplicateResults.front().groups;
            OcctSavedGroupState actualGroups;
            if (!groups || !myDoc->CaptureSavedGroups(actualGroups)) { return DuplicateDocumentState::Unavailable; }
			if (committedCount == _pendingDuplicateResults.size()) {
                return groups->candidateSealed && groups->candidate.IsEqual(actualGroups)
                    ? DuplicateDocumentState::AllCommitted : DuplicateDocumentState::PartialOrMismatched;
			}
			if (missingCount == _pendingDuplicateResults.size()) {
                return groups->previous.IsEqual(actualGroups)
                    ? DuplicateDocumentState::None : DuplicateDocumentState::PartialOrMismatched;
			}
			return DuplicateDocumentState::PartialOrMismatched;
		} catch (...) {
			return DuplicateDocumentState::Unavailable;
		}
	}

	Standard_Boolean ObjectInteractor::duplicateOwnedCommandIsCurrent(
		const Handle(TDocStd_Document)& document) const noexcept {
		try {
			const Handle(TDF_Data) data = document.IsNull()
				? Handle(TDF_Data)() : document->GetData();
			Handle(TDataStd_Integer) marker;
			return _duplicateOwnsDocumentCommand
				&& !document.IsNull()
				&& document->HasOpenCommand()
				&& !data.IsNull()
				&& _duplicateOwnedTransaction > 0
				&& data->Transaction() == _duplicateOwnedTransaction
				&& data->Time() == _duplicateOwnedDocumentTime
				&& document->Main().FindAttribute(
					Core3DDuplicateCommandOwnerAttributeID(), marker)
				&& !marker.IsNull()
				&& marker->Get() == _duplicateOwnedMarkerValue
				? Standard_True : Standard_False;
		} catch (...) {
			return Standard_False;
		}
	}

	Standard_Boolean
	ObjectInteractor::abortOwnedDuplicateCommand() noexcept {
		if (!_duplicateOwnsDocumentCommand || myDoc.IsNull()) {
			return Standard_False;
		}
		try {
			const Handle(TDocStd_Document) document =
				myDoc->ChangeDocument();
			if (document.IsNull()
				|| !duplicateOwnedCommandIsCurrent(document)) {
				return Standard_False;
			}
			if (document->HasOpenCommand()) {
				document->AbortCommand();
			}
			if (document->HasOpenCommand()) {
				return Standard_False;
			}
			_duplicateOwnsDocumentCommand = false;
			_duplicateOwnedTransaction = -1;
			_duplicateOwnedDocumentTime = -1;
			_duplicateOwnedMarkerValue = 0;
			return Standard_True;
		} catch (...) {
			// OCCT may report an exception after it has already closed the
			// command. Re-observe immediately so a closed owner token can never
			// survive into another tool's future transaction.
			try {
				const Handle(TDocStd_Document) document =
					myDoc.IsNull()
					? Handle(TDocStd_Document)()
					: myDoc->ChangeDocument();
				if (!document.IsNull() && !document->HasOpenCommand()) {
					_duplicateOwnsDocumentCommand = false;
					_duplicateOwnedTransaction = -1;
					_duplicateOwnedDocumentTime = -1;
					_duplicateOwnedMarkerValue = 0;
					return Standard_True;
				}
			} catch (...) {
			}
			return Standard_False;
		}
	}

	Standard_Boolean
	ObjectInteractor::finishCommittedDuplicate() noexcept {
		if (inspectPendingDuplicateResults()
				!= DuplicateDocumentState::AllCommitted
			|| myContext.IsNull() || myDoc.IsNull()) {
			return Standard_False;
		}
		_duplicateOwnsDocumentCommand = false;
		_duplicateOwnedTransaction = -1;
		_duplicateOwnedDocumentTime = -1;
		_duplicateOwnedMarkerValue = 0;
		try {
			detachManipulator(false);
			createManipulatorIfNeeded();
			myContext->ClearSelected(Standard_False);
#ifdef DEBUG
			if (_debugDuplicatePresentationRepairFailureCount > 0) {
				--_debugDuplicatePresentationRepairFailureCount;
				// Model a failure after committed-result repair has cleared the old
				// owners but before the first duplicate is selected. The pending
				// ledger remains the sole recovery authority.
				try { myDoc->NotifyChanges(); } catch (...) {}
				return Standard_False;
			}
#endif
			for (const DuplicatePendingResult& duplicate :
				 _pendingDuplicateResults) {
				if (duplicate.presentation.IsNull()
					|| duplicate.resultLabel.IsNull()) {
					return Standard_False;
				}
				myContext->Display(
					duplicate.presentation,
					AIS_Shaded,
					0,
					Standard_False);
				myContext->AddSelect(duplicate.presentation);
				_manipulator->Attach(duplicate.presentation);
				_manipulatorSourceLabels[duplicate.presentation.get()] =
					duplicate.resultLabel;
			}
			if (positionManipulatorAtExactSavedGroupOrigin()
				== SavedGroupPivotResult::Failed) {
				// The document is committed, but its exact duplicate-group pivot
				// is part of presentation repair. Retain the result ledger and let
				// the existing retry path finish without duplicating again.
				try { myDoc->NotifyChanges(); } catch (...) {}
				return Standard_False;
			}
			myContext->HilightSelected(Standard_True);
			myDoc->NotifyChanges();
			_manipulator->Redisplay();
			myContext->UpdateCurrentViewer();
			_pendingDuplicateResults.clear();
			return Standard_True;
		} catch (...) {
			// The document is already authoritative. Retain the result ledger so
			// the next Duplicate action can retry transient presentation repair.
			try { myDoc->NotifyChanges(); } catch (...) {}
			return Standard_False;
		}
	}

    void ObjectInteractor::duplicateSelected() {
        if (blocksForOrdinaryEdit()) { return; }
        auto doc = myDoc->ChangeDocument();
		if (doc.IsNull()) { return; }
		if (_duplicateOwnsDocumentCommand
			|| !_pendingDuplicateResults.empty()) {
			DuplicateDocumentState state =
				inspectPendingDuplicateResults();
			if (state == DuplicateDocumentState::OpenCommand
				&& abortOwnedDuplicateCommand()) {
				state = inspectPendingDuplicateResults();
			}
			if (state == DuplicateDocumentState::AllCommitted) {
				(void)finishCommittedDuplicate();
			} else if (state == DuplicateDocumentState::None) {
				_pendingDuplicateResults.clear();
				_duplicateOwnsDocumentCommand = false;
				_duplicateOwnedTransaction = -1;
				_duplicateOwnedDocumentTime = -1;
				_duplicateOwnedMarkerValue = 0;
			} else if (state != DuplicateDocumentState::OpenCommand
				&& !doc->HasOpenCommand()) {
				try { myDoc->NotifyChanges(); } catch (...) {}
			}
			return;
		}
		if (doc->HasOpenCommand()) { return; }
        
        struct DuplicateSource {
            Handle(AIS_Shape) presentation;
            TDF_Label label;
			OcctGeometryRepresentation representation;
			OcctReferenceAxisReadState referenceAxisState;
			OcctReferenceAxis referenceAxis;
            profile::Record savedProfile;
            OcctObjectNameState name;
            TopoDS_Shape ownerShape;
            bool profileCurrent = false;
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
			OcctReferenceAxis referenceAxis;
			const OcctReferenceAxisReadState referenceAxisState =
				myDoc->ReadReferenceAxisForLabel(label, referenceAxis);
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
                || !storedShape.IsEqual(shape->Shape())
				|| referenceAxisState
					== OcctReferenceAxisReadState::Invalid) {
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
                profile::Record savedProfile;
                OcctObjectNameState name;
                if (!profile::Read(doc, label, savedProfile)
                    || !myDoc->CaptureObjectNameStateForLabel(label, name)) { return; }
				sources.push_back({shape, label, destinationRepresentation,
                    referenceAxisState, referenceAxis, savedProfile, name, storedShape,
                    savedProfile.IsCurrent(doc, label)});
            }
        }
        if (sources.empty()) { return; }
		std::vector<TDF_Label> sourceLabels;
		sourceLabels.reserve(sources.size());
		for (const DuplicateSource& source : sources) {
			sourceLabels.push_back(source.label);
		}
        std::vector<OcctGeometryDuplicationRequest> duplicationRequests;
        duplicationRequests.reserve(sourceLabels.size());
        for (const auto& label : sourceLabels) duplicationRequests.push_back({label, 1U, false, true});
		if (!myDoc->CanDuplicateGeometryDefinitions(duplicationRequests)) {
			return;
		}
        OcctSavedGroupState groupBefore;
        if (!myDoc->CaptureSavedGroups(groupBefore)) { return; }
        const auto fullySelected = [&](const OcctSavedGroup& group) {
            return !group.members.empty() && std::all_of(group.members.begin(), group.members.end(), [&](const auto& label) {
                return std::any_of(sourceLabels.begin(), sourceLabels.end(), [&](const auto& source) { return source.IsEqual(label); });
            });
        };
        const auto belongsToFullySelectedSavedGroup = [&](const TDF_Label& label) {
            return std::any_of(groupBefore.groups.begin(), groupBefore.groups.end(), [&](const auto& group) {
                return fullySelected(group) && std::any_of(group.members.begin(), group.members.end(),
                    [&](const auto& member) { return member.IsEqual(label); });
            });
        };
        const auto activeGroups = std::count_if(groupBefore.groups.begin(), groupBefore.groups.end(), [](const auto& g) { return !g.members.empty(); });
        const auto copiedGroups = std::count_if(groupBefore.groups.begin(), groupBefore.groups.end(), fullySelected);
        if (activeGroups + copiedGroups > 128) { return; }

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
        
		std::vector<DuplicatePendingResult> duplicates;
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
                const gp_Trsf sourceTransform = source.presentation->LocalTransformation();
                // A complete saved group is one assembly operation: apply the
                // admitted bounds displacement in document/world space so
                // heterogeneous member transforms preserve their layout.
                // Partial and ungrouped duplication retain existing behavior.
                copy->SetLocalTransformation(belongsToFullySelectedSavedGroup(source.label)
                    ? minAxisDisplacement.Multiplied(sourceTransform)
                    : sourceTransform.Multiplied(minAxisDisplacement));
				myDoc->LoadObjectMeterial(source.label, copy);
                DuplicatePendingResult duplicate;
                duplicate.presentation = copy;
                duplicate.sourceLabel = source.label;
                duplicate.representation = source.representation;
                duplicate.expectedShape = copy->Shape();
                duplicate.expectedTransform = copy->LocalTransformation();
                duplicate.expectedReferenceAxisState = source.referenceAxisState;
                duplicate.expectedReferenceAxis = source.referenceAxis;
                duplicate.originalProfile = source.savedProfile;
                duplicate.originalName = source.name;
                duplicate.originalOwnerShape = source.ownerShape;
                duplicate.originalProfileCurrent = source.profileCurrent;
                if (!source.savedProfile.label.IsNull()) {
                    duplicate.preparedProfileIdentifier = OcctDocument::NewProfileIdentifier();
                    if (!profile::IsIdentifier(duplicate.preparedProfileIdentifier)
                        || duplicate.preparedProfileIdentifier == source.savedProfile.identifier) { return; }
                    if (source.savedProfile.boundShape.IsEqual(source.ownerShape)) {
                        duplicate.preparedProfileBinding = copy->Shape();
                    } else {
                        // Preserve the original stale recipe binding as its own
                        // independent solid. Never rebind it to the later root.
                        BRepBuilderAPI_Copy retainedCopy;
                        retainedCopy.Perform(source.savedProfile.boundShape, Standard_True, Standard_False);
                        if (!retainedCopy.IsDone() || retainedCopy.Shape().IsNull()
                            || retainedCopy.Shape().IsPartner(source.savedProfile.boundShape)
                            || !IsTopologicallyValid(retainedCopy.Shape())) { return; }
                        duplicate.preparedProfileBinding = retainedCopy.Shape();
                    }
                }
                const auto& savedEnclosure = source.name.object.enclosure;
                duplicate.originalEnclosureCurrent = savedEnclosure.IsCurrent(doc, source.label);
                if (!savedEnclosure.label.IsNull()) {
                    duplicate.preparedEnclosureIdentifier = OcctDocument::NewProfileIdentifier();
                    if (!profile::IsIdentifier(duplicate.preparedEnclosureIdentifier)
                        || duplicate.preparedEnclosureIdentifier == savedEnclosure.identifier) return;
                    if (savedEnclosure.boundShape.IsEqual(source.ownerShape)) {
                        duplicate.preparedEnclosureBinding = copy->Shape();
                    } else {
                        BRepBuilderAPI_Copy retainedCopy;
                        retainedCopy.Perform(savedEnclosure.boundShape, Standard_True, Standard_False);
                        if (!retainedCopy.IsDone() || retainedCopy.Shape().IsNull()
                            || retainedCopy.Shape().IsPartner(savedEnclosure.boundShape)
                            || !IsTopologicallyValid(retainedCopy.Shape())) return;
                        duplicate.preparedEnclosureBinding = retainedCopy.Shape();
                    }
                }
                duplicates.push_back(std::move(duplicate));
			}
		} catch (...) {
			return;
		}
		if (duplicates.empty()) { return; }
        duplicates.front().groups.emplace();
        duplicates.front().groups->previous = groupBefore;
		_pendingDuplicateResults = std::move(duplicates);

		const auto retainRetryableOrUnknown = [&]() noexcept {
			if (abortOwnedDuplicateCommand()
				&& inspectPendingDuplicateResults()
					== DuplicateDocumentState::None) {
				_pendingDuplicateResults.clear();
				_duplicateOwnsDocumentCommand = false;
				_duplicateOwnedTransaction = -1;
				_duplicateOwnedDocumentTime = -1;
				_duplicateOwnedMarkerValue = 0;
			}
		};

		Standard_Integer duplicateMarkerValue = 0;
		try {
			Handle(TDF_Attribute) priorMarkerAttribute;
			const Standard_Boolean hasPriorMarker =
				doc->Main().FindAttribute(
					Core3DDuplicateCommandOwnerAttributeID(),
					priorMarkerAttribute);
			const Handle(TDataStd_Integer) priorMarker =
				Handle(TDataStd_Integer)::DownCast(priorMarkerAttribute);
			if (hasPriorMarker && priorMarker.IsNull()) {
				_pendingDuplicateResults.clear();
				return;
			}
			const Standard_Integer priorMarkerValue =
				priorMarker.IsNull() ? 0 : priorMarker->Get();
			duplicateMarkerValue =
				priorMarkerValue
					== std::numeric_limits<Standard_Integer>::max()
				? std::numeric_limits<Standard_Integer>::min()
				: priorMarkerValue + 1;
			_duplicateOwnsDocumentCommand = true;
			doc->NewCommand();
			if (!doc->HasOpenCommand()) {
				_duplicateOwnsDocumentCommand = false;
				_duplicateOwnedTransaction = -1;
				_duplicateOwnedDocumentTime = -1;
				_duplicateOwnedMarkerValue = 0;
				_pendingDuplicateResults.clear();
				return;
			}
			const Handle(TDF_Data) transactionData = doc->GetData();
			if (transactionData.IsNull()
				|| transactionData->Transaction() <= 0) {
				try { doc->AbortCommand(); } catch (...) {}
				_duplicateOwnsDocumentCommand = false;
				_duplicateOwnedTransaction = -1;
				_duplicateOwnedDocumentTime = -1;
				_duplicateOwnedMarkerValue = 0;
				_pendingDuplicateResults.clear();
				return;
			}
			_duplicateOwnedTransaction = transactionData->Transaction();
			_duplicateOwnedDocumentTime = transactionData->Time();
			_duplicateOwnedMarkerValue = duplicateMarkerValue;
			TDataStd_Integer::Set(
				doc->Main(),
				Core3DDuplicateCommandOwnerAttributeID(),
				duplicateMarkerValue);
			if (!duplicateOwnedCommandIsCurrent(doc)) {
				try { doc->AbortCommand(); } catch (...) {}
				_duplicateOwnsDocumentCommand = false;
				_duplicateOwnedTransaction = -1;
				_duplicateOwnedDocumentTime = -1;
				_duplicateOwnedMarkerValue = 0;
				_pendingDuplicateResults.clear();
				return;
			}
			for (auto& duplicate : _pendingDuplicateResults) {
				const TDF_Label label = myDoc->AddShape(
					duplicate.presentation,
					duplicate.representation);
                duplicate.resultLabel = label;
				if (label.IsNull()) {
					retainRetryableOrUnknown();
					return;
				}
				if (!myDoc->CopyGeometryRepresentation(
						duplicate.sourceLabel, label)
					|| !myDoc->CopyGeometryOwnedMeshMetadata(
						duplicate.sourceLabel, label)
					|| !myDoc->CopyObjectAppearance(
						duplicate.sourceLabel, label)
					|| !myDoc->CopyReferenceAxis(
						duplicate.sourceLabel, label)) {
					retainRetryableOrUnknown();
					return;
				}
				duplicate.entityIdentifier =
					myDoc->EntityIdentifierForLabel(label);
				duplicate.definitionIdentifier =
					myDoc->DefinitionIdentifierForLabel(label);
				OcctReferenceAxis storedReferenceAxis;
				const OcctReferenceAxisReadState storedReferenceAxisState =
					myDoc->ReadReferenceAxisForLabel(
						label, storedReferenceAxis);
				if (duplicate.entityIdentifier.empty()
					|| duplicate.definitionIdentifier.empty()
					|| myDoc->GeometryRepresentationForLabel(label)
						!= duplicate.representation
					|| TransformDiffers(
						myDoc->ObjectTransformForLabel(label),
						duplicate.expectedTransform)
					|| storedReferenceAxisState
						!= duplicate.expectedReferenceAxisState
					|| storedReferenceAxisState
						== OcctReferenceAxisReadState::Invalid
					|| ReferenceAxisDiffers(
						storedReferenceAxis,
						duplicate.expectedReferenceAxis)) {
					retainRetryableOrUnknown();
					return;
				}
                profile::Record stageAuthority = duplicate.originalProfile;
#ifdef DEBUG
                const Standard_Integer profileFault = _debugDuplicateProfileFault;
                if (!duplicate.originalProfile.label.IsNull()) {
                    _debugDuplicateProfileFault = 0;
                    if (profileFault == 4) stageAuthority.values[1] += 1;
                }
#endif
                if (!profile::StageDuplicate(doc, duplicate.sourceLabel,
                        stageAuthority, duplicate.originalOwnerShape,
                        label, duplicate.preparedProfileBinding,
                        duplicate.preparedProfileIdentifier, duplicate.candidateProfile)) {
                    retainRetryableOrUnknown(); return;
                }
#ifdef DEBUG
                if (profileFault == 5 && !duplicate.candidateProfile.label.IsNull()) {
                    // A real malformed stored scalar count must abort the owned
                    // command, including the already allocated duplicate root.
                    TDataStd_Integer::Set(duplicate.candidateProfile.label, profile::CountID(),
                        static_cast<int>(duplicate.candidateProfile.values.size()) + 1);
                    profile::Record readback;
                    if (!profile::Read(doc, label, readback)) { retainRetryableOrUnknown(); return; }
                    throw Standard_Failure("Injected invalid profile unexpectedly read back");
                }
                if ((profileFault == 6 || profileFault == 7) && !duplicate.candidateProfile.label.IsNull()) {
                    // Valid but unexpected identity survives ordinary commit;
                    // exact source/candidate reconciliation must refuse it.
                    const auto target = profileFault == 6 ? duplicate.candidateProfile.label : duplicate.originalProfile.label;
                    TDataStd_AsciiString::Set(target, profile::IdentityID(),
                        TCollection_AsciiString(OcctDocument::NewProfileIdentifier().c_str()));
                }
#endif
                duplicate.profileCandidateSealed = true;
                enclosure::Record enclosureAuthority = duplicate.originalName.object.enclosure;
#ifdef DEBUG
                const Standard_Integer enclosureFault = _debugDuplicateEnclosureFault;
                if (!enclosureAuthority.label.IsNull()) {
                    _debugDuplicateEnclosureFault = 0;
                    if (enclosureFault == 8) enclosureAuthority.values[2] += 1;
                }
#endif
                if (!enclosure::StageDuplicate(doc, duplicate.sourceLabel,
                        enclosureAuthority, duplicate.originalOwnerShape,
                        label, duplicate.preparedEnclosureBinding,
                        duplicate.preparedEnclosureIdentifier, duplicate.candidateEnclosure)) {
                    retainRetryableOrUnknown(); return;
                }
#ifdef DEBUG
                if (enclosureFault == 9 && !duplicate.candidateEnclosure.label.IsNull()) {
                    TDataStd_Integer::Set(duplicate.candidateEnclosure.label, enclosure::CountID(),
                        static_cast<int>(duplicate.candidateEnclosure.values.size()) + 1);
                    enclosure::Record readback;
                    if (!enclosure::Read(doc, label, readback)) { retainRetryableOrUnknown(); return; }
                    throw Standard_Failure("Injected invalid enclosure unexpectedly read back");
                }
                if ((enclosureFault == 10 || enclosureFault == 11) && !duplicate.candidateEnclosure.label.IsNull()) {
                    const auto target = enclosureFault == 10 ? duplicate.candidateEnclosure.label
                        : duplicate.originalName.object.enclosure.label;
                    TDataStd_AsciiString::Set(target, enclosure::IdentityID(),
                        TCollection_AsciiString(OcctDocument::NewProfileIdentifier().c_str()));
                }
#endif
                duplicate.enclosureCandidateSealed = true;
                // Preserve authored names and their absence on each independent
                // part. Closed-commit recovery compares this exact source state.
                if (duplicate.originalName.namePresent) {
                    TDataStd_Name::Set(label, duplicate.originalName.name);
                } else {
                    label.ForgetAttribute(TDataStd_Name::GetID());
                }
				myDoc->LoadObjectMeterial(label, duplicate.presentation);
			}
            auto& groupAuthority = *_pendingDuplicateResults.front().groups;
            if (copiedGroups > 0) {
                auto requested = groupAuthority.previous.groups;
                requested.erase(std::remove_if(requested.begin(), requested.end(), [](const auto& g) { return g.members.empty(); }), requested.end());
                for (const auto& original : groupAuthority.previous.groups) {
                    if (!fullySelected(original)) { continue; }
                    OcctSavedGroup copy;
                    copy.identifier = OcctDocument::NewSavedGroupIdentifier();
                    copy.name = original.name;
                    if (copy.name.Length() <= 251) { copy.name += TCollection_ExtendedString(" copy"); }
                    for (const auto& label : original.members) {
                        const auto result = std::find_if(_pendingDuplicateResults.begin(), _pendingDuplicateResults.end(), [&](const auto& r) { return r.sourceLabel.IsEqual(label); });
                        if (result == _pendingDuplicateResults.end() || result->resultLabel.IsNull()) { retainRetryableOrUnknown(); return; }
                        copy.members.push_back(result->resultLabel);
                    }
                    // Complete saved-group duplication uses the same admitted
                    // document-space translation for every member and its
                    // optional authored point. Absence remains absence.
                    if(original.originPresent){
                        copy.origin=original.origin.Translated(aDisplacement);
                        if(!OcctDocument::IsAdmittedSavedGroupOrigin(copy.origin)){retainRetryableOrUnknown();return;}
                        copy.originPresent=Standard_True;
                    }
                    requested.push_back(std::move(copy));
                }
                if (!myDoc->StageSavedGroups(requested)) { retainRetryableOrUnknown(); return; }
            }
            if (!myDoc->CaptureSavedGroups(groupAuthority.candidate)) { retainRetryableOrUnknown(); return; }
            groupAuthority.candidateSealed = true;
			if (!myDoc->ValidateGeometryRepresentations()) {
				retainRetryableOrUnknown();
				return;
			}
			try {
				Standard_Boolean commitReported = doc->CommitCommand();
#ifdef DEBUG
				const Standard_Integer commitMode =
					_debugDuplicateCommitMode;
				_debugDuplicateCommitMode = 0;
				if (commitMode == 1) {
					commitReported = Standard_False;
				} else if (commitMode == 2) {
					throw Standard_Failure(
						"Injected Duplicate commit exception after close");
				}
#endif
				(void)commitReported;
			} catch (...) {
				// Reconcile against OCAF below. A throw can happen before or
				// after CommitCommand closes the transaction.
			}
		} catch (...) {
			retainRetryableOrUnknown();
			return;
		}

		DuplicateDocumentState documentState =
			inspectPendingDuplicateResults();
		if (documentState == DuplicateDocumentState::OpenCommand) {
			if (abortOwnedDuplicateCommand()) {
				documentState = inspectPendingDuplicateResults();
			}
		}
		if (documentState == DuplicateDocumentState::AllCommitted) {
			(void)finishCommittedDuplicate();
			return;
		}
		if (documentState == DuplicateDocumentState::None) {
			_pendingDuplicateResults.clear();
			_duplicateOwnsDocumentCommand = false;
			_duplicateOwnedTransaction = -1;
			_duplicateOwnedDocumentTime = -1;
			_duplicateOwnedMarkerValue = 0;
			return;
		}
		if (documentState != DuplicateDocumentState::OpenCommand
			&& !doc->HasOpenCommand()) {
			// If inspection itself is unavailable or detects an impossible
			// partial state, publish document invalidation so renderer clients
			// rebuild from OCAF instead of trusting stale transient UI state.
			try { myDoc->NotifyChanges(); } catch (...) {}
		}
	    }

	bool ObjectInteractor::hasUnresolvedDuplicate() const noexcept {
		return _duplicateOwnsDocumentCommand
			|| !_pendingDuplicateResults.empty();
	}

	    const bool ObjectInteractor::isSelected() const {
//        printf(">>> ObjectInteractor::isSelected - %d\n", !myContext->FirstSelectedObject().IsNull());
        return !myContext->FirstSelectedObject().IsNull();
    }

    void ObjectInteractor::attachManipulatorToSelection(bool detach) {
        if (hasUnresolvedEdit()) { return; }
        if (_manipulatorType == PrimitiveManipulatorType::PrimitiveGizmoTypeNone) { return; }
		if (_manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray
			|| _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray
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
		if (hasUnresolvedEdit()) { return; }
		
		Quantity_Color color = Quantity_Color(on ? Quantity_NameOfColor::Quantity_NOC_BLUE : Quantity_NameOfColor::Quantity_NOC_GRAY80);
		
		Graphic3d_MaterialAspect mat = selected->Material();
		mat.SetAlpha(on ? 0.4 : 1.0);
		selected->SetMaterial(mat);
		selected->SetColor(color);
		myContext->SetMaterial(selected, mat, Standard_True);
	}

	void ObjectInteractor::setManipulatorType(PrimitiveManipulatorType type) {
		if (hasUnresolvedEdit()) { return; }
		const PrimitiveManipulatorType aPreviousType = _manipulatorType;
		if (type
				!= PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray
			&& hasActiveRadialArray()
			&& !cancelRadialArray()) {
			// Retain the radial recovery ledger and its visible controls until
			// the command outcome can be proven exactly once.
			return;
		}
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
				== PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray
			&& (aPreviousType
					!= PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray
					|| !hasActiveRadialArray())
			&& !beginRadialArray()) {
				if (!hasActiveRadialArray()) {
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
				&& _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray
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
				if (positionManipulatorAtExactSavedGroupOrigin()
					== SavedGroupPivotResult::Failed) {
					// Changing gizmo type has no surrounding transaction to
					// restore an authored group pivot. Keep the selection, but
					// never expose a gizmo at the attachment's default center.
					_manipulator->Detach();
					_manipulatorSourceLabels.clear();
				}
            }
        } else if (((mirror && aPreviousType != PrimitiveManipulatorType::PrimitiveGizmoTypeMirror)
                    || ((movRot || scale)
                        && aPreviousType == PrimitiveManipulatorType::PrimitiveGizmoTypeNone))
                   && _manipulatorSourceLabels.empty()
                   && _trialMirrorObjects.empty() && _trialMirrorSources.empty()
                   && _pendingMirrorResults.empty() && !_mirrorOwnsDocumentCommand) {
            // Browser selection and construction can preserve an exact selected
            // part while None owns no gizmo. Entering an object gizmo must
            // attach that selection; never replace a stale nonempty cache.
            std::vector<std::pair<Handle(AIS_InteractiveObject), TDF_Label>> sources;
            std::unordered_set<const AIS_InteractiveObject*> unique;
            bool exact = !myContext.IsNull() && !myDoc.IsNull();
            if (exact) {
                for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
                    const auto object = myContext->SelectedInteractive();
                    const Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(object);
                    const TDF_Label label = myDoc->ShapeLabel(object);
                    gp_Trsf transform;
                    const auto representation = myDoc->GeometryRepresentationForLabel(label);
                    const std::size_t maximumSources = mirror ? kMaxMirrorPreviewBodies : 1024U;
                    if (sources.size() >= maximumSources || shape.IsNull()
                        || !myContext->IsDisplayed(object) || !myDoc->IsPresentationEditable(object)
                        || label.IsNull() || !myDoc->IsEditableFreeSimpleDefinitionLabel(label)
                        || !IsObjectModelingRepresentation(representation)
                        || ((mirror || scale) && !IsBRepModelingRepresentation(representation))
                        || myContext->SelectedOwner().IsNull()
                        || myContext->SelectedOwner() != object->GlobalSelOwner()
                        || shape->Shape().IsNull()
                        || !XCAFDoc_ShapeTool::GetShape(label).IsEqual(shape->Shape())
                        || !myDoc->TryObjectTransformForLabel(label, transform)
                        || TransformDiffers(transform, object->LocalTransformation())
                        || !unique.insert(object.get()).second) {
                        exact = false; break;
                    }
                    sources.emplace_back(object, label);
                }
            }
            if (exact && !sources.empty()) {
                Handle(Core3DManipulatorObjectSequence) selected = new Core3DManipulatorObjectSequence();
                for (const auto& source : sources) selected->Append(source.first);
                _manipulator->Attach(selected);
                for (const auto& source : sources)
                    _manipulatorSourceLabels.emplace(source.first.get(), source.second);
                if (positionManipulatorAtExactSavedGroupOrigin()
                    == SavedGroupPivotResult::Failed) {
                    // This attachment path has no surrounding selection
                    // transaction to roll back. Keep the authoritative
                    // selection, but never expose a gizmo at the wrong pivot.
                    _manipulator->Detach();
                    _manipulatorSourceLabels.clear();
                }
            }
        }

		_manipulator->Redisplay();

        if (type == PrimitiveManipulatorType::PrimitiveGizmoTypeNone
			|| type
				== PrimitiveManipulatorType::PrimitiveGizmoTypeLinearArray
			|| type
				== PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray
			|| type
				== PrimitiveManipulatorType::PrimitiveGizmoTypeShell) { // remove gizmo when the operation owns only AIS previews
            _manipulator->DeactivateCurrentMode();
            myContext->Remove(_manipulator, Standard_True);
        }

        myContext->UpdateCurrentViewer();
    }

    bool ObjectInteractor::transformManipulator(const int theX, const int theY) {
		if (hasUnresolvedEdit()) { return false; }
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
		if (hasUnresolvedEdit()) { return false; }
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

#ifdef DEBUG
    bool ObjectInteractor::debugReplayGesture(
        const Standard_Integer mode, const Standard_Integer axis,
        const std::vector<Standard_Real>& values,
        std::vector<std::array<Standard_Real, 2>>& samples) noexcept {
        samples.clear();
        if (mode < 0 || mode > 3 || axis < 0 || axis > 2
            || values.size() < 2 || values.size() > 32
            || _manipulator.IsNull() || !_manipulator->IsAttached()
            || myView.IsNull() || myContext.IsNull()
            || isManipulatorGestureActive() || hasUnresolvedEdit()) {
            return false;
        }
        try {
            OCC_CATCH_SIGNALS
            const AIS_ManipulatorMode modes[] = {
                AIS_MM_Translation, AIS_MM_Rotation,
                AIS_MM_ScalingUniform, AIS_MM_Scaling};
            const gp_Ax2 position = _manipulator->Position();
            const gp_Dir directions[] = {
                position.XDirection(), position.YDirection(), position.Direction()};
            // Fixed orthographic projection avoids device-pixel-dependent hit
            // detection. Every gesture sample still runs ConvertWithProj and
            // the production manipulator math, snapping, and commit path.
            myView->Camera()->SetProjectionType(
                Graphic3d_Camera::Projection_Orthographic);
            // SetAt/SetCenter changes direction relative to the previous eye;
            // setting it after SetProj makes the first replay differ from one
            // after Undo. Define the complete orientation together so each
            // replay projects identical world samples onto identical pixels.
            myView->Camera()->SetEyeAndCenter(
                position.Location().Translated(gp_Vec(400.0, 400.0, 400.0)),
                position.Location());
            myView->Camera()->SetUp(gp_Dir(-1.0, -1.0, 2.0));
            myView->Camera()->SetScale(400.0);
            myView->Redraw();
            std::vector<std::array<Standard_Integer, 2>> points;
            for (const Standard_Real value : values) {
                if (!std::isfinite(value) || std::abs(value) > 1.0e6
                    || (mode >= 2 && value <= 0.0)) {
                    return false;
                }
                gp_Pnt point = position.Location();
                if (mode == 1) {
                    point.Translate(gp_Vec(directions[(axis + 1) % 3]) * 100.0);
                    gp_Trsf rotation;
                    rotation.SetRotation(gp_Ax1(position.Location(), directions[axis]),
                        value * M_PI / 180.0);
                    point.Transform(rotation);
                } else {
                    point.Translate(gp_Vec(directions[axis])
                        * (mode >= 2 ? 100.0 * value : 100.0 + value));
                }
                Standard_Integer x = 0, y = 0;
                myView->Convert(point.X(), point.Y(), point.Z(), x, y);
                points.push_back({x, y});
            }
            if (!_manipulator->DebugActivateGesture(modes[mode], axis)) {
                return false;
            }
            _manipulator->StartTransform(
                points.front()[0], points.front()[1], myView, myContext);
            _manipulatorGestureActive = _manipulator->HasActiveTransformation();
            if (!_manipulatorGestureActive) {
                cancelInteraction();
                return false;
            }
            for (std::size_t index = 1; index < points.size(); ++index) {
                if (!transformManipulator(points[index][0], points[index][1])) {
                    cancelInteraction();
                    samples.clear();
                    return false;
                }
                std::array<Standard_Real, 2> sample = {0.0, 0.0};
                const auto objects = _manipulator->Objects();
                Standard_Integer objectIndex = 1;
                for (Core3DManipulatorObjectSequence::Iterator object(*objects);
                     object.More(); object.Next(), ++objectIndex) {
                    const gp_Trsf current = object.Value()->LocalTransformation();
                    const gp_Trsf initial = _manipulator->StartTransformation(objectIndex);
                    for (Standard_Integer row = 1; row <= 3; ++row) {
                        for (Standard_Integer column = 1; column <= 4; ++column) {
                            sample[0] = std::max(sample[0], std::abs(
                                current.Value(row, column) - initial.Value(row, column)));
                        }
                    }
                    const auto source = _manipulator->cachedShapes().find(object.Value());
                    const Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(object.Value());
                    if (source != _manipulator->cachedShapes().end()
                        && !shape.IsNull() && !shape->Shape().IsEqual(source->second)) {
                        sample[1] += 1.0;
                    }
                }
                samples.push_back(sample);
            }
            finishInteraction();
            return true;
        } catch (...) {
            try { cancelInteraction(); } catch (...) {}
            samples.clear();
            return false;
        }
    }
#endif

    void ObjectInteractor::finishInteraction() {
		if (hasUnresolvedEdit()) {
			_manipulatorGestureActive = false;
			return;
		}
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
					|| !IsAdmittedGestureTransform(
						presentation->LocalTransformation())
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

            // The viewer-owned typed controller owns staging, reconciliation,
            // selection/gizmo repair and exactly-once publication.
            const auto controller = _ordinaryEditController.lock();
            if (!controller) { cancelInteraction(); return; }
            OrdinaryTransformOperation operation;
            switch (activeMode) {
                case AIS_MM_Translation:
                case AIS_MM_TranslationPlane:
                    operation = OrdinaryTransformOperation::Translate;
                    break;
                case AIS_MM_Rotation:
                    operation = OrdinaryTransformOperation::Rotate;
                    break;
                case AIS_MM_Scaling:
                case AIS_MM_ScalingUniform:
                    operation = OrdinaryTransformOperation::Scale;
                    break;
                default:
                    cancelInteraction(); return;
            }
            std::vector<OrdinaryTransformChange> requests;
            requests.reserve(changes.size());
            try {
                for (const auto& change : changes) {
                    OcctObjectTransformState before;
                    if (!myDoc->CaptureObjectTransformStateForLabel(change.second, before)) {
                        cancelInteraction(); return;
                    }
                    // Editable free definitions have no transformed parent.
                    // A parent-relative occurrence needs a different command
                    // contract; never interpret its local matrix as world space.
                    const auto parent = change.first->CombinedParentTransformation();
                    if (!parent.IsNull() && parent->Form() != gp_Identity) {
                        cancelInteraction(); return;
                    }
                    OrdinaryTransformChange request;
                    request.label = change.second;
                    request.presentation = change.first;
                    request.shape = change.first->Shape();
                    request.operation = operation;
                    request.collectiveWorldDelta = _manipulator->GestureTransformation();
                    request.transform = change.first->LocalTransformation();
                    if (operation == OrdinaryTransformOperation::Translate) {
                        // Preserve the exact persisted orientation/scale rather
                        // than introducing a quaternion round-trip into Move.
                        request.transform = before.transform;
                        request.transform.SetTranslationPart(change.first->LocalTransformation().TranslationPart());
                    } else if (operation == OrdinaryTransformOperation::Rotate) {
                        OrdinaryRotationAroundPivot rotation;
                        rotation.pivot = _manipulator->StartPosition().Location();
                        rotation.delta = _manipulator->GestureTransformation();
                        request.rotationAroundPivot = rotation;
                        request.transform = rotation.delta * before.transform;
                    }
                    requests.push_back(std::move(request));
                }
                OrdinaryEditResult failure = OrdinaryEditResult::Invalid;
                auto lease = controller->beginTransform(requests, &failure);
                if (!lease) {
                    // An uncertain begin retains authority; only a definitive
                    // rejection/no-op permits cancelling the transient preview.
                    if (!controller->blocksNormalWork()) { cancelInteraction(); }
                    return;
                }
                (void)lease.stageAndCommit();
                // Do not roll back from a commit Boolean/exception or publish
                // here. The retained controller resolves the durable outcome.
            } catch (...) {
                if (!controller->blocksNormalWork()) { cancelInteraction(); }
            }
        }
    }

    void ObjectInteractor::cancelInteraction() {
		if (hasUnresolvedEdit()) {
			_manipulatorGestureActive = false;
			return;
		}
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

    bool ObjectInteractor::replaceSelectedObjectForBrowser(
        const Handle(AIS_InteractiveObject)& target, bool& selectionWasTouched) noexcept {
        return replaceSelectedObjectsForBrowser({target}, selectionWasTouched);
    }

    bool ObjectInteractor::replaceSelectedObjectsForBrowser(
        const std::vector<Handle(AIS_InteractiveObject)>& targets,
        bool& selectionWasTouched) noexcept
    {
        selectionWasTouched = false;

#ifdef DEBUG
        Standard_Integer failureMode = 0;
#endif
        std::vector<Handle(SelectMgr_EntityOwner)> previousOwners;
        std::unordered_set<const SelectMgr_EntityOwner*> previousOwnerSet;
        Handle(Core3DManipulatorObjectSequence) previousObjects;
        std::unordered_map<const AIS_InteractiveObject*, TDF_Label> previousLabels;
        try {
            OCC_CATCH_SIGNALS
            if (myDoc.IsNull() || myContext.IsNull() || targets.empty() || targets.size() > 32
                || hasUnresolvedEdit() || _manipulatorGestureActive
                || (!_manipulator.IsNull()
                    && _manipulator->HasActiveTransformation())
                || (_manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeNone
                    && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate
                    && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeScale)) {
                return false;
            }
            const auto document = myDoc->Document();
            if (document.IsNull() || document->HasOpenCommand()) { return false; }
            const auto isCommittedPresentation = [this](
                const Handle(AIS_InteractiveObject)& object) {
                const Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(object);
                if (shape.IsNull() || !myContext->IsDisplayed(object)
                    || !myDoc->IsPresentationEditable(object)) { return false; }
                const TDF_Label label = myDoc->ShapeLabel(object);
                gp_Trsf transform;
                return !label.IsNull()
                    && myDoc->IsEditableFreeSimpleDefinitionLabel(label)
                    && IsObjectModelingRepresentation(myDoc->GeometryRepresentationForLabel(label))
                    && !shape->Shape().IsNull()
                    && XCAFDoc_ShapeTool::GetShape(label).IsEqual(shape->Shape())
                    && myDoc->TryObjectTransformForLabel(label, transform)
                    && !TransformDiffers(transform, shape->LocalTransformation());
            };
            std::unordered_set<const AIS_InteractiveObject*> targetSet;
            for (const auto& target : targets) {
                if (!isCommittedPresentation(target) || !targetSet.insert(target.get()).second
                    || target->GlobalSelOwner().IsNull()) { return false; }
            }
            std::unordered_set<const AIS_InteractiveObject*> previousPresentations;
            for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
                const auto owner = myContext->SelectedOwner();
                const auto object = myContext->SelectedInteractive();
                if (previousOwners.size() >= 50000 || owner.IsNull()
                    || !isCommittedPresentation(object)
                    || !previousOwnerSet.insert(owner.get()).second
                    || !previousPresentations.insert(object.get()).second) {
                    return false;
                }
                previousOwners.push_back(owner);
            }
            previousLabels = _manipulatorSourceLabels;
            if (!_manipulator.IsNull() && _manipulator->IsAttached()) {
                const auto attached = _manipulator->Objects();
                bool previousGizmoIsExact = !attached.IsNull() && attached->Size() > 0
                    && static_cast<std::size_t>(attached->Size()) == previousPresentations.size()
                    && previousLabels.size() == previousPresentations.size();
                std::unordered_set<const AIS_InteractiveObject*> uniqueAttached;
                if (previousGizmoIsExact) {
                    for (Core3DManipulatorObjectSequence::Iterator item(*attached); item.More(); item.Next()) {
                        const auto object = item.Value();
                        const auto label = previousLabels.find(object.get());
                        if (!previousPresentations.count(object.get()) || !uniqueAttached.insert(object.get()).second
                            || label == previousLabels.end() || label->second != myDoc->ShapeLabel(object)) {
                            previousGizmoIsExact = false;
                            break;
                        }
                    }
                }
                if (previousGizmoIsExact) {
                    previousObjects = new Core3DManipulatorObjectSequence();
                    for (Core3DManipulatorObjectSequence::Iterator item(*attached); item.More(); item.Next()) {
                        previousObjects->Append(item.Value());
                    }
                } else {
                    // An explicit new selection may retire an inactive stale
                    // gizmo. Rollback derives its attachment from independently
                    // verified current owners, never from the mismatched cache.
                    // Active gestures and unresolved edits were rejected above.
                    previousLabels.clear();
                    bool canRestoreGizmo = !previousOwners.empty() && previousOwners.size() <= 1024;
                    for (const auto& owner : previousOwners) {
                        const auto object = Handle(AIS_InteractiveObject)::DownCast(owner->Selectable());
                        if (ManipulatorRequiresBRepModeling(_manipulatorType)
                            && !IsBRepModelingRepresentation(myDoc->GeometryRepresentationForLabel(myDoc->ShapeLabel(object)))) {
                            canRestoreGizmo = false;
                        }
                    }
                    if (canRestoreGizmo) {
                        previousObjects = new Core3DManipulatorObjectSequence();
                        for (const auto& owner : previousOwners) {
                            const auto object = Handle(AIS_InteractiveObject)::DownCast(owner->Selectable());
                            previousObjects->Append(object);
                            previousLabels.emplace(object.get(), myDoc->ShapeLabel(object));
                        }
                    }
                }
            } else if (!previousLabels.empty()) {
                // No live attachment can authorize stale source-label entries.
                // Retire them when adopting the new explicit selection.
                previousLabels.clear();
            }
            const bool shouldAttach = _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeNone
                && (!ManipulatorRequiresBRepModeling(_manipulatorType)
                    || std::all_of(targets.begin(), targets.end(), [&](const auto& target) { return IsBRepModelingRepresentation(myDoc->GeometryRepresentationForLabel(myDoc->ShapeLabel(target))); }));
#ifdef DEBUG
            failureMode = std::exchange(_debugBrowserSelectionFailureMode, 0);
#endif
            selectionWasTouched = true;
            myContext->ClearDetected(Standard_False);
            detachManipulator(false);
            // A null gizmo may still have stale cached source labels. The new
            // explicit selection owns its attachment map, including no gizmo.
            _manipulatorSourceLabels.clear();
            myContext->ClearSelected(Standard_False);
            for (const auto& target : targets) { myContext->AddOrRemoveSelected(target->GlobalSelOwner(), Standard_False); }
#ifdef DEBUG
            if (failureMode == 1) { throw Standard_Failure("Injected browser selection failure"); }
#endif
            std::unordered_set<const AIS_InteractiveObject*> actual;
            for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
                const auto object = myContext->SelectedInteractive();
                if (!targetSet.count(object.get()) || !actual.insert(object.get()).second) {
                    throw Standard_Failure("Browser group selection mismatch");
                }
            }
            if (actual.size() != targets.size()) { throw Standard_Failure("Browser group selection incomplete"); }
            // Attach replaces the previous group because it was detached above.
            // The legacy single-object Attach overload otherwise appends objects.
            if (shouldAttach && _manipulator.IsNull()) {
                setManipulatorType(_manipulatorType);
            }
            if (shouldAttach) {
                if (_manipulator.IsNull()) { throw Standard_Failure("Browser group has no gizmo"); }
                // The legacy helper attaches only its last selected object.
                // Bind the complete, already admitted group in one operation.
                _manipulator->Detach();
                Handle(Core3DManipulatorObjectSequence) objects = new Core3DManipulatorObjectSequence();
                _manipulatorSourceLabels.clear();
                for (const auto& target : targets) {
                    objects->Append(target);
                    _manipulatorSourceLabels.emplace(target.get(), myDoc->ShapeLabel(target));
                }
                _manipulator->Attach(objects);
                if (positionManipulatorAtExactSavedGroupOrigin()
                    == SavedGroupPivotResult::Failed) {
                    // The catch path below restores the previous selection and
                    // exact attachment. Do not report browser selection success
                    // when an authored group pivot could not be installed.
                    throw Standard_Failure("Browser group pivot mismatch");
                }
            }
            if (isManipulatorAttached() != shouldAttach) {
                throw Standard_Failure("Browser selection gizmo availability mismatch");
            }
            if (shouldAttach) {
                const auto attached = _manipulator->Objects();
                if (attached.IsNull() || static_cast<std::size_t>(attached->Size()) != targets.size()
                    || _manipulatorSourceLabels.size() != targets.size()) {
                    throw Standard_Failure("Browser group gizmo size mismatch");
                }
                std::unordered_set<const AIS_InteractiveObject*> attachedSet;
                for (Core3DManipulatorObjectSequence::Iterator it(*attached); it.More(); it.Next()) {
                    const auto object = it.Value();
                    if (!targetSet.count(object.get()) || !attachedSet.insert(object.get()).second
                        || _manipulatorSourceLabels.at(object.get()) != myDoc->ShapeLabel(object)) {
                        throw Standard_Failure("Browser group gizmo target mismatch");
                    }
                }
            }
#ifdef DEBUG
            if (failureMode >= 2) { throw Standard_Failure("Injected browser gizmo failure"); }
#endif
            myContext->UpdateCurrentViewer();
            return true;
        } catch (...) {
            if (!selectionWasTouched) { return false; }
        }
        try {
            OCC_CATCH_SIGNALS
#ifdef DEBUG
            if (failureMode == 3) { throw Standard_Failure("Injected browser selection rollback failure"); }
#endif
            detachManipulator(false);
            myContext->ClearSelected(Standard_False);
            for (const auto& owner : previousOwners) {
                myContext->AddOrRemoveSelected(owner, Standard_False);
            }
            std::size_t restoredCount = 0;
            for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
                if (previousOwnerSet.find(myContext->SelectedOwner().get()) == previousOwnerSet.end()) {
                    throw Standard_Failure("Browser selection rollback owner mismatch");
                }
                ++restoredCount;
            }
            if (restoredCount != previousOwners.size()) {
                throw Standard_Failure("Browser selection rollback was incomplete");
            }
            if (!previousObjects.IsNull()) {
                createManipulatorIfNeeded();
                _manipulator->Attach(previousObjects);
                if (!_manipulator->IsAttached() || _manipulator->Objects() != previousObjects) {
                    throw Standard_Failure("Browser selection rollback gizmo mismatch");
                }
            }
            _manipulatorSourceLabels.swap(previousLabels);
            if (!previousObjects.IsNull()
                && positionManipulatorAtExactSavedGroupOrigin()
                    == SavedGroupPivotResult::Failed) {
                // Rollback restored the exact selection. Keep that authority,
                // but do not leave a draggable gizmo at a guessed pivot.
                _manipulator->Detach();
                _manipulatorSourceLabels.clear();
            }
            myContext->UpdateCurrentViewer();
        } catch (...) {
            // A failed presentation repair must not leave a gizmo capable of
            // changing a different object from the visible selection.
            const Handle(Core3DManipulator) abandoned = _manipulator;
            _manipulator.Nullify();
            _manipulatorSourceLabels.clear();
            _manipulatorGestureActive = false;
            // Raw gesture entry points can only use _manipulator. Sever that
            // reference even if OCCT cannot remove an abandoned presentation.
            try {
                if (!abandoned.IsNull()) { myContext->Remove(abandoned, Standard_False); }
            } catch (...) {}
            try { myContext->ClearSelected(Standard_False); } catch (...) {}
            try { myContext->UpdateCurrentViewer(); } catch (...) {}
        }
        return false;
    }

    const PrimitiveManipulatorType ObjectInteractor::getManipulatorType() const {
        return _manipulatorType;
    }

    bool ObjectInteractor::hasUnresolvedEdit() const noexcept {
        return hasUnresolvedDuplicate() || blocksForOrdinaryEdit();
    }

    bool ObjectInteractor::blocksForOrdinaryEdit() const noexcept {
        const auto controller = _ordinaryEditController.lock();
        return controller && controller->blocksNormalWork();
    }

bool ObjectInteractor::repairCommittedMeshCopyPresentation(const OrdinaryCreationLedger& ledger) noexcept {
    try {
        const auto controller=_ordinaryEditController.lock();
        if (!controller || controller->state()!=OrdinaryEditState::RepairPending
            || !ledger.meshCopy || ledger.records.size()!=1 || myDoc.IsNull() || myContext.IsNull()
            || _manipulatorGestureActive || hasUnresolvedDuplicate()
            || _manipulatorType!=ledger.authority.manipulatorType
            || (!_manipulator.IsNull() && _manipulator->HasActiveTransformation())) return false;
        const auto& source=*ledger.meshCopy;const auto& record=ledger.records.front();
        const auto target=record.requested.presentation;
        const auto sameTransform=[](const gp_Trsf& a,const gp_Trsf& b) {
            for(int r=1;r<=3;++r)for(int c=1;c<=4;++c)if(a.Value(r,c)!=b.Value(r,c))return false;
            return true;
        };
        if (source.presentation.IsNull() || target.IsNull() || source.presentation==target
            || !source.presentation->Shape().IsEqual(source.previous.object.object.shape)
            || !sameTransform(source.presentation->LocalTransformation(),source.previous.object.object.transform)
            || !target->Shape().IsEqual(record.shape) || !sameTransform(target->LocalTransformation(),record.transform)
            || (source.presentation->HasInteractiveContext() && source.presentation->InteractiveContext()!=myContext.get())
            || (target->HasInteractiveContext() && target->InteractiveContext()!=myContext.get())) return false;
        OcctObjectVisibilityState original;
        OcctObjectNameState created;
        if (!myDoc->CaptureObjectVisibilityStateForLabel(source.previous.object.object.label,original)
            || !source.candidate.IsEqual(original) || original.IsEffectivelyVisible()
            || !myDoc->CaptureObjectNameStateForLabel(record.candidate.object.label,created)
            || !record.candidate.IsEqual(created)
            || !myDoc->ShapeLabel(target).IsEqual(record.candidate.object.label)
            || !myDoc->IsPresentationEditable(target)) return false;
        // Retry accepts only the original owner, the created object's owner,
        // or the empty intermediate state produced while repairing this edit.
        std::size_t selectedCount=0;
        for(myContext->InitSelected();myContext->MoreSelected();myContext->NextSelected()) {
            if (++selectedCount>1) return false;
            const auto object=myContext->SelectedInteractive();const auto owner=myContext->SelectedOwner();
            if (object==source.presentation) {
                if (ledger.authority.selectionOwners.size()!=1 || owner!=ledger.authority.selectionOwners.front()) return false;
            } else if (object!=target || owner!=target->GlobalSelOwner()) return false;
        }
        if (_manipulator!=ledger.authority.manipulatorPresentation) return false;
        if (!_manipulator.IsNull() && _manipulator->IsAttached()) {
            const auto objects=_manipulator->Objects();
            if (objects.IsNull() || objects->Size()!=1) return false;
            for(Core3DManipulatorObjectSequence::Iterator it(*objects);it.More();it.Next())
                if (it.Value()!=source.presentation && it.Value()!=target) return false;
        }
        AIS_ListOfInteractive displayed;myContext->DisplayedObjects(AIS_KOI_Shape,-1,displayed);
        if (displayed.Extent()>50000) return false;
        for(AIS_ListIteratorOfListOfInteractive it(displayed);it.More();it.Next()) {
            const auto shape=Handle(AIS_Shape)::DownCast(it.Value());
            if (shape.IsNull())continue;
            const auto label=myDoc->ShapeLabel(shape);
            if ((label.IsEqual(source.previous.object.object.label) && shape!=source.presentation)
                || (label.IsEqual(record.candidate.object.label) && shape!=target)) return false;
        }
        if (!_manipulator.IsNull()) {_manipulator->DeactivateCurrentMode();_manipulator->Detach();}
        _manipulatorSourceLabels.clear();
        myContext->ClearDetected(Standard_False);myContext->ClearSelected(Standard_False);
        if (source.presentation->HasInteractiveContext()) myContext->Remove(source.presentation,Standard_False);
        myDoc->LoadObjectMeterial(record.candidate.object.label,target);
        if (!myContext->IsDisplayed(target)) myContext->Display(target,AIS_Shaded,0,Standard_False);
        myContext->SetSelected(target,Standard_False);
        const bool attach=ledger.authority.hadManipulator
            && _manipulatorType!=PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
        if (!_manipulator.IsNull()) {
        const bool scale=_manipulatorType==PrimitiveManipulatorType::PrimitiveGizmoTypeScale;
        const bool moveRotate=_manipulatorType==PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate;
        for(int axis=0;axis<3;++axis) {
            _manipulator->SetPart(axis,AIS_MM_Scaling,scale);
            _manipulator->SetPart(axis,AIS_MM_ScalingUniform,scale);
            _manipulator->SetPart(axis,AIS_MM_Translation,moveRotate);
            _manipulator->SetPart(axis,AIS_MM_Rotation,moveRotate);
            _manipulator->SetPart(axis,AIS_MM_TranslationPlane,Standard_False);
            _manipulator->SetPart(axis,AIS_MM_MirroringPlaneNeg,Standard_False);
            _manipulator->SetPart(axis,AIS_MM_MirroringPlanePos,Standard_False);
        }
        if (attach) {
            _manipulator->Attach(target);_manipulatorSourceLabels.emplace(target.get(),record.candidate.object.label);
            _manipulator->UpdateCachedShapes();_manipulator->Redisplay();
        }
        _manipulator->DeactivateCurrentMode();
        }
        _manipulatorGestureActive=false;
        OrdinaryNameLedger actual;
        if (!captureOrdinaryNameAuthority(actual) || actual.selectedPresentations.size()!=1
            || actual.selectedPresentations.front().presentation!=target || actual.selectionOwners.size()!=1
            || actual.selectionOwners.front()!=target->GlobalSelOwner()
            || actual.manipulatorType!=ledger.authority.manipulatorType || actual.hadManipulator!=attach
            || myContext->IsDisplayed(source.presentation) || source.presentation->HasInteractiveContext()) return false;
        if (attach && (actual.manipulatorObjects.size()!=1 || actual.manipulatorObjects.front()!=target
            || actual.manipulatorSourceLabels.size()!=1 || !actual.manipulatorSourceLabels.front().IsEqual(record.candidate.object.label)
            || actual.manipulatorCachedShapes.size()!=1 || !actual.manipulatorCachedShapes.front().IsEqual(record.shape))) return false;
        TColStd_ListOfInteger modes;myContext->ActivatedModes(target,modes);
        if (!myContext->IsDisplayed(target) || modes.Extent()!=1 || modes.First()!=0) return false;
        myContext->UpdateCurrentViewer();return true;
    } catch (...) {return false;}
}

    bool ObjectInteractor::captureOrdinaryVisibilityAuthority(OrdinaryVisibilityLedger& ledger) const noexcept {
        try {
            if (ledger.records.empty() || ledger.records.size() > 1024
                || !captureOrdinaryNameAuthority(ledger.authority)) { return false; }
            ledger.selectedObjects.clear();
            for (const auto& selected : ledger.authority.selectedPresentations) {
                OcctObjectNameState state;
                if (!myDoc->CaptureObjectNameStateForLabel(myDoc->ShapeLabel(selected.presentation), state)
                    || !myDoc->IsPresentationEditable(selected.presentation)
                    || !selected.shape.IsEqual(state.object.shape)) { return false; }
                for (int row = 1; row <= 3; ++row) {
                    for (int column = 1; column <= 4; ++column) {
                        if (selected.transform.Value(row, column) != state.object.transform.Value(row, column)) { return false; }
                    }
                }
                ledger.selectedObjects.push_back(state);
            }
            AIS_ListOfInteractive displayed;
            myContext->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
            if (displayed.Extent() > 50000) { return false; }
            ledger.targetPresentations.clear();
            for (const auto& record : ledger.records) {
                Handle(AIS_Shape) found;
                for (AIS_ListIteratorOfListOfInteractive it(displayed); it.More(); it.Next()) {
                    const auto presentation = Handle(AIS_Shape)::DownCast(it.Value());
                    if (presentation.IsNull()
                        || !myDoc->ShapeLabel(presentation).IsEqual(record.previous.object.object.label)) { continue; }
                    if (!found.IsNull() || !myDoc->IsPresentationEditable(presentation)
                        || !presentation->Shape().IsEqual(record.previous.object.object.shape)
                        || TransformDiffers(presentation->LocalTransformation(), record.previous.object.object.transform)) {
                        return false;
                    }
                    found = presentation;
                }
                if (record.previous.IsEffectivelyVisible() != !found.IsNull()) { return false; }
                ledger.targetPresentations.push_back(found);
            }
            return true;
        } catch (...) { return false; }
    }

    bool ObjectInteractor::repairOrdinaryVisibilityPresentation(
        const OrdinaryVisibilityLedger& ledger, bool committed) noexcept {
        try {
            if (myDoc.IsNull() || myContext.IsNull() || ledger.records.empty()
                || ledger.selectedObjects.size() != ledger.authority.selectedPresentations.size()) { return false; }
            const auto sameTransform = [](const gp_Trsf& a, const gp_Trsf& b) {
                for (int row = 1; row <= 3; ++row) {
                    for (int col = 1; col <= 4; ++col) {
                        if (a.Value(row, col) != b.Value(row, col)) { return false; }
                    }
                }
                return true;
            };
            const auto isHiddenTarget = [&](const Handle(AIS_InteractiveObject)& object) {
                const auto presentation = Handle(AIS_Shape)::DownCast(object);
                if (presentation.IsNull()) { return false; }
                const auto label = myDoc->ShapeLabel(presentation);
                for (const auto& record : ledger.records) {
                    const auto& saved = committed ? record.candidate : record.previous;
                    if (label.IsEqual(saved.object.object.label)) { return !saved.IsEffectivelyVisible(); }
                }
                return false;
            };
            // Verify the surviving source objects before any selection repair.
            for (std::size_t i = 0; i < ledger.selectedObjects.size(); ++i) {
                const auto& expected = ledger.selectedObjects[i];
                const auto& presentation = ledger.authority.selectedPresentations[i];
                OcctObjectNameState actual;
                if (!myDoc->CaptureObjectNameStateForLabel(expected.object.label, actual)
                    || !actual.IsEqual(expected)
                    || !presentation.presentation->Shape().IsEqual(presentation.shape)
                    || !sameTransform(presentation.presentation->LocalTransformation(), presentation.transform)) {
                    return false;
                }
            }
            std::vector<Handle(SelectMgr_EntityOwner)> survivors;
            for (const auto& owner : ledger.authority.selectionOwners) {
                if (owner.IsNull()) { return false; }
                const auto object = Handle(AIS_InteractiveObject)::DownCast(owner->Selectable());
                if (object.IsNull()) { return false; }
                if (!isHiddenTarget(object)) { survivors.push_back(owner); }
            }
            const bool changesSelection = survivors.size() != ledger.authority.selectionOwners.size();
            if (changesSelection && !_manipulator.IsNull()) {
                _manipulator->DeactivateCurrentMode();
                _manipulator->Detach();
                _manipulatorSourceLabels.clear();
            }
            AIS_ListOfInteractive displayed;
            myContext->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
            if (displayed.Extent() > 50000) { return false; }
            for (const auto& record : ledger.records) {
                const auto& expected = committed ? record.candidate : record.previous;
                OcctObjectVisibilityState actual;
                if (!myDoc->CaptureObjectVisibilityStateForLabel(expected.object.object.label, actual)
                    || !actual.IsEqual(expected)) { return false; }
                int matches = 0;
                for (AIS_ListIteratorOfListOfInteractive it(displayed); it.More(); it.Next()) {
                    const auto presentation = Handle(AIS_Shape)::DownCast(it.Value());
                    if (presentation.IsNull()
                        || !myDoc->ShapeLabel(presentation).IsEqual(expected.object.object.label)) { continue; }
                    if (++matches > 1 || !myDoc->IsPresentationEditable(presentation)
                        || !presentation->Shape().IsEqual(expected.object.object.shape)
                        || !sameTransform(presentation->LocalTransformation(), expected.object.object.transform)) { return false; }
                    if (!expected.IsEffectivelyVisible()) { myContext->Remove(presentation, Standard_False); }
                }
                if (expected.IsEffectivelyVisible() && matches != 1) { return false; }
            }
            if (!changesSelection) {
                // Showing never selects a new object or moves an existing gizmo.
                if (!verifyOrdinaryNameAuthority(ledger.authority)) { return false; }
            } else {
                myContext->ClearDetected(Standard_False);
                myContext->ClearSelected(Standard_False);
                Handle(Core3DManipulatorObjectSequence) group = new Core3DManipulatorObjectSequence();
                for (const auto& owner : survivors) {
                    const auto object = Handle(AIS_InteractiveObject)::DownCast(owner->Selectable());
                    if (object.IsNull() || !myContext->IsDisplayed(object)) { return false; }
                    myContext->AddOrRemoveSelected(owner, Standard_False);
                    group->Append(object);
                }
                _manipulatorType = ledger.authority.manipulatorType;
                createManipulatorIfNeeded();
                const bool scale = _manipulatorType == PrimitiveManipulatorType::PrimitiveGizmoTypeScale;
                const bool moveRotate = _manipulatorType == PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate;
                for (int axis = 0; axis < 3; ++axis) {
                    _manipulator->SetPart(axis, AIS_MM_Scaling, scale);
                    _manipulator->SetPart(axis, AIS_MM_ScalingUniform, scale);
                    _manipulator->SetPart(axis, AIS_MM_Translation, moveRotate);
                    _manipulator->SetPart(axis, AIS_MM_Rotation, moveRotate);
                    _manipulator->SetPart(axis, AIS_MM_TranslationPlane, Standard_False);
                    _manipulator->SetPart(axis, AIS_MM_MirroringPlaneNeg, Standard_False);
                    _manipulator->SetPart(axis, AIS_MM_MirroringPlanePos, Standard_False);
                }
                const bool attach = ledger.authority.hadManipulator && !survivors.empty()
                    && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
                if (attach) {
                    _manipulator->Attach(group);
                    for (Core3DManipulatorObjectSequence::Iterator it(*group); it.More(); it.Next()) {
                        const auto presentation = Handle(AIS_Shape)::DownCast(it.Value());
                        _manipulatorSourceLabels.emplace(presentation.get(), myDoc->ShapeLabel(presentation));
                    }
                    _manipulator->UpdateCachedShapes();
                    _manipulator->Redisplay();
                }
                _manipulator->DeactivateCurrentMode();
                OrdinaryNameLedger repaired;
                if (!captureOrdinaryNameAuthority(repaired)
                    || repaired.selectionOwners != survivors
                    || repaired.manipulatorType != ledger.authority.manipulatorType
                    || repaired.hadManipulator != attach) { return false; }
                if (attach) {
                    if (repaired.manipulatorObjects.size() != survivors.size()) { return false; }
                    for (std::size_t i = 0; i < repaired.manipulatorObjects.size(); ++i) {
                        const auto presentation = Handle(AIS_Shape)::DownCast(repaired.manipulatorObjects[i]);
                        if (presentation.IsNull() || !myDoc->ShapeLabel(presentation).IsEqual(repaired.manipulatorSourceLabels[i])
                            || !presentation->Shape().IsEqual(repaired.manipulatorCachedShapes[i])) { return false; }
                    }
                }
            }
            // Removed AIS objects must also be absent from the active selector.
            AIS_ListOfInteractive finalDisplay;
            myContext->DisplayedObjects(AIS_KOI_Shape, -1, finalDisplay);
            for (AIS_ListIteratorOfListOfInteractive it(finalDisplay); it.More(); it.Next()) {
                if (isHiddenTarget(it.Value())) { return false; }
            }
            _manipulatorGestureActive = false;
            myContext->ClearDetected(Standard_False);
            myContext->UpdateCurrentViewer();
            return true;
        } catch (...) { return false; }
    }

    bool ObjectInteractor::captureOrdinaryNameAuthority(OrdinaryNameLedger& ledger) const noexcept {
        try {
            if (myDoc.IsNull() || myContext.IsNull() || _manipulatorGestureActive
                || hasUnresolvedDuplicate()
                || (_manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeNone
                    && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate
                    && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeScale
                    && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeMaterial)
                || (!_manipulator.IsNull() && _manipulator->HasActiveTransformation())) { return false; }
            ledger.selectionOwners.clear();
            ledger.selectedPresentations.clear();
            std::unordered_set<const AIS_InteractiveObject*> unique;
            for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
                if (ledger.selectionOwners.size() >= 1024) { return false; }
                const auto owner = myContext->SelectedOwner();
                const auto presentation = Handle(AIS_Shape)::DownCast(myContext->SelectedInteractive());
                if (owner.IsNull() || presentation.IsNull() || owner->Selectable() != presentation
                    || !myContext->IsDisplayed(presentation)) { return false; }
                ledger.selectionOwners.push_back(owner);
                if (unique.insert(presentation.get()).second) {
                    ledger.selectedPresentations.push_back({presentation, presentation->Shape(),
                                                           presentation->LocalTransformation()});
                }
            }
            ledger.manipulatorType = _manipulatorType;
            ledger.manipulatorPresentation = _manipulator;
            ledger.hadManipulator = !_manipulator.IsNull() && _manipulator->IsAttached();
            ledger.manipulatorObjects.clear();
            ledger.manipulatorSourceLabels.clear();
            ledger.manipulatorCachedShapes.clear();
            if (!_manipulator.IsNull()) {
                ledger.manipulatorTransform = _manipulator->LocalTransformation();
                ledger.manipulatorPosition = _manipulator->Position();
            }
            if (ledger.hadManipulator) {
                const auto attached = _manipulator->Objects();
                if (attached.IsNull() || attached->Size() > 1024
                    || static_cast<std::size_t>(attached->Size()) != _manipulatorSourceLabels.size()
                    || static_cast<std::size_t>(attached->Size()) != _manipulator->cachedShapes().size()) { return false; }
                for (Core3DManipulatorObjectSequence::Iterator it(*attached); it.More(); it.Next()) {
                    const auto label = _manipulatorSourceLabels.find(it.Value().get());
                    const auto cached = _manipulator->cachedShapes().find(it.Value());
                    if (it.Value().IsNull() || label == _manipulatorSourceLabels.end()
                        || cached == _manipulator->cachedShapes().end()) { return false; }
                    ledger.manipulatorObjects.push_back(it.Value());
                    ledger.manipulatorSourceLabels.push_back(label->second);
                    ledger.manipulatorCachedShapes.push_back(cached->second);
                }
            }
            return true;
        } catch (...) { return false; }
    }

    bool ObjectInteractor::verifyOrdinaryNameAuthority(const OrdinaryNameLedger& ledger) const noexcept {
        try {
            OrdinaryNameLedger actual;
            if (!captureOrdinaryNameAuthority(actual)
                || actual.selectionOwners != ledger.selectionOwners
                || actual.manipulatorType != ledger.manipulatorType
                || actual.manipulatorPresentation != ledger.manipulatorPresentation
                || actual.hadManipulator != ledger.hadManipulator
                || actual.manipulatorObjects != ledger.manipulatorObjects
                || actual.manipulatorSourceLabels.size() != ledger.manipulatorSourceLabels.size()
                || actual.manipulatorCachedShapes.size() != ledger.manipulatorCachedShapes.size()
                || actual.selectedPresentations.size() != ledger.selectedPresentations.size()) { return false; }
            const auto sameTransform = [](const gp_Trsf& a, const gp_Trsf& b) {
                for (int row = 1; row <= 3; ++row) {
                    for (int column = 1; column <= 4; ++column) {
                        if (a.Value(row, column) != b.Value(row, column)) { return false; }
                    }
                }
                return true;
            };
            if (!ledger.manipulatorPresentation.IsNull()) {
                if (!sameTransform(actual.manipulatorTransform, ledger.manipulatorTransform)) { return false; }
                const auto& a = actual.manipulatorPosition;
                const auto& b = ledger.manipulatorPosition;
                for (int axis = 1; axis <= 3; ++axis) {
                    if (a.Location().Coord(axis) != b.Location().Coord(axis)
                        || a.Direction().Coord(axis) != b.Direction().Coord(axis)
                        || a.XDirection().Coord(axis) != b.XDirection().Coord(axis)) { return false; }
                }
            }
            for (std::size_t i = 0; i < actual.selectedPresentations.size(); ++i) {
                const auto& a = actual.selectedPresentations[i];
                const auto& b = ledger.selectedPresentations[i];
                if (a.presentation != b.presentation || !a.shape.IsEqual(b.shape)
                    || !sameTransform(a.transform, b.transform)) { return false; }
            }
            for (std::size_t i = 0; i < actual.manipulatorObjects.size(); ++i) {
                if (!actual.manipulatorSourceLabels[i].IsEqual(ledger.manipulatorSourceLabels[i])
                    || !actual.manipulatorCachedShapes[i].IsEqual(ledger.manipulatorCachedShapes[i])) { return false; }
            }
            return true;
        } catch (...) { return false; }
    }

    bool ObjectInteractor::captureOrdinaryTransformAuthority(OrdinaryTransformLedger& ledger) const noexcept {
        try {
            if (myDoc.IsNull() || myContext.IsNull() || ledger.records.empty()
                || hasUnresolvedDuplicate()
                || (_manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeNone
                    && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate
                    && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeScale
                    && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeMaterial)) { return false; }
            std::unordered_map<const AIS_InteractiveObject*, const OrdinaryTransformRecord*> expected;
            for (const auto& record : ledger.records) {
                const auto& presentation = record.requested.presentation;
                if (presentation.IsNull() || !myContext->IsDisplayed(presentation)
                    || !myDoc->IsPresentationEditable(presentation)
                    || (!presentation->Shape().IsEqual(record.previous.shape)
                        && !presentation->Shape().IsEqual(record.requested.shape))
                    || (TransformDiffers(presentation->LocalTransformation(), record.previous.transform)
                        && TransformDiffers(presentation->LocalTransformation(), record.requested.transform))
                    || (record.requested.operation == OrdinaryTransformOperation::Scale
                        && (record.previous.resolvedRepresentation == OcctGeometryRepresentation::TriangleMesh
                            ? !record.previous.shape.IsEqual(record.requested.shape)
                            : !IsTopologicallyValid(record.requested.shape)))
                    || !expected.emplace(presentation.get(), &record).second) { return false; }
            }
            ledger.selectionOwners.clear();
            std::unordered_set<const AIS_InteractiveObject*> selected;
            for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
                const auto presentation = myContext->SelectedInteractive();
                const auto owner = myContext->SelectedOwner();
                const auto found = expected.find(presentation.get());
                const auto brepOwner = Handle(StdSelect_BRepOwner)::DownCast(owner);
                if (found == expected.end() || owner.IsNull() || owner->Selectable() != presentation
                    || brepOwner.IsNull() || !brepOwner->HasShape()
                    || !brepOwner->Shape().IsEqual(found->second->previous.shape)
                    || !selected.insert(presentation.get()).second) { return false; }
                ledger.selectionOwners.push_back(owner);
            }
            if (selected.size() != expected.size()) { return false; }
            ledger.manipulatorType = _manipulatorType;
            ledger.hadManipulator = !_manipulator.IsNull() && _manipulator->IsAttached();
            if (ledger.hadManipulator) {
                const auto attached = _manipulator->Objects();
                if (attached.IsNull() || static_cast<std::size_t>(attached->Size()) != expected.size()
                    || _manipulatorSourceLabels.size() != expected.size()) { return false; }
                std::unordered_set<const AIS_InteractiveObject*> unique;
                int index = 1;
                if (_manipulator->HasActiveTransformation()) {
                    for (const auto& record : ledger.records) {
                        if (record.requested.rotationAroundPivot
                            && (_manipulator->ActiveMode() != AIS_ManipulatorMode::AIS_MM_Rotation
                                || !record.requested.rotationAroundPivot->pivot.IsEqual(
                                    _manipulator->StartPosition().Location(), Precision::Confusion()))) { return false; }
                    }
                }
                for (Core3DManipulatorObjectSequence::Iterator it(*attached); it.More(); it.Next(), ++index) {
                    const auto found = expected.find(it.Value().get());
                    const auto cached = _manipulator->cachedShapes().find(it.Value());
                    const auto label = _manipulatorSourceLabels.find(it.Value().get());
                    if (found == expected.end() || !unique.insert(it.Value().get()).second
                        || cached == _manipulator->cachedShapes().end()
                        || !cached->second.IsEqual(found->second->previous.shape)
                        || label == _manipulatorSourceLabels.end()
                        || !label->second.IsEqual(found->second->previous.label)
                        || (_manipulator->HasActiveTransformation()
                            && TransformDiffers(_manipulator->StartTransformation(index), found->second->previous.transform))) {
                        return false;
                    }
                }
            }
            return true;
        } catch (...) { return false; }
    }

    bool ObjectInteractor::repairOrdinaryTransformPresentation(
        const OrdinaryTransformLedger& ledger, bool committed) noexcept {
        try {
            if (myContext.IsNull() || myDoc.IsNull() || ledger.records.empty()
                || ledger.selectionOwners.size() != ledger.records.size()) { return false; }
            // Retire the preview without guessing its durable outcome. Every
            // presentation is assigned from an independently proven OCAF state.
            if (!_manipulator.IsNull()) {
                if (_manipulator->HasActiveTransformation()) { _manipulator->StopTransform(Standard_True); }
                _manipulator->DeactivateCurrentMode();
                _manipulator->Detach();
            }
            _manipulatorGestureActive = false;
            _manipulatorSourceLabels.clear();
            myContext->ClearDetected(Standard_False);
            myContext->ClearSelected(Standard_False);
            Handle(Core3DManipulatorObjectSequence) group = new Core3DManipulatorObjectSequence();
            std::unordered_map<const AIS_InteractiveObject*, const OcctObjectTransformState*> expected;
            for (const auto& record : ledger.records) {
                const auto& saved = committed ? record.candidate : record.previous;
                OcctObjectTransformState actual;
                const auto& presentation = record.requested.presentation;
                if (!myDoc->CaptureObjectTransformStateForLabel(saved.label, actual)
                    || !actual.IsEqual(saved) || presentation.IsNull()
                    || !myContext->IsDisplayed(presentation)) { return false; }
                presentation->SetShape(saved.shape);
                presentation->SetLocalTransformation(saved.transform);
                myContext->Redisplay(presentation, Standard_False);
                // A Scale result may replace topology. Recompute the global
                // owner for both geometry changes and persisted transforms.
                myContext->RecomputeSelectionOnly(presentation);
                const auto owner = presentation->GlobalSelOwner();
                if (owner.IsNull() || owner->Selectable() != presentation) { return false; }
                myContext->AddOrRemoveSelected(owner, Standard_False);
                group->Append(presentation);
                expected.emplace(presentation.get(), &saved);
            }
            _manipulatorType = ledger.manipulatorType;
            if (_manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeNone
                && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate
                && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeScale
                    && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeMaterial) { return false; }
            createManipulatorIfNeeded();
            const bool scale = _manipulatorType == PrimitiveManipulatorType::PrimitiveGizmoTypeScale;
            const bool moveRotate = _manipulatorType == PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate;
            for (int axis = 0; axis < 3; ++axis) {
                _manipulator->SetPart(axis, AIS_ManipulatorMode::AIS_MM_Scaling, scale);
                _manipulator->SetPart(axis, AIS_ManipulatorMode::AIS_MM_ScalingUniform, scale);
                _manipulator->SetPart(axis, AIS_ManipulatorMode::AIS_MM_Translation, moveRotate);
                _manipulator->SetPart(axis, AIS_ManipulatorMode::AIS_MM_Rotation, moveRotate);
                _manipulator->SetPart(axis, AIS_ManipulatorMode::AIS_MM_TranslationPlane, Standard_False);
                _manipulator->SetPart(axis, AIS_ManipulatorMode::AIS_MM_MirroringPlaneNeg, Standard_False);
                _manipulator->SetPart(axis, AIS_ManipulatorMode::AIS_MM_MirroringPlanePos, Standard_False);
            }
            if (ledger.hadManipulator && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeNone) {
                _manipulator->Attach(group);
                for (const auto& record : ledger.records) {
                    _manipulatorSourceLabels.emplace(record.requested.presentation.get(), record.previous.label);
                }
                _manipulator->UpdateCachedShapes();
                _manipulator->Redisplay();
            }
            _manipulator->DeactivateCurrentMode();
            myContext->ClearDetected(Standard_False);
            std::unordered_set<const AIS_InteractiveObject*> selected;
            for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
                const auto object = myContext->SelectedInteractive();
                const auto found = expected.find(object.get());
                const auto owner = Handle(StdSelect_BRepOwner)::DownCast(myContext->SelectedOwner());
                if (found == expected.end() || !selected.insert(object.get()).second
                    || owner.IsNull() || !owner->HasShape() || !owner->Shape().IsEqual(found->second->shape)) { return false; }
            }
            if (selected.size() != expected.size() || _manipulatorGestureActive
                || _manipulator->HasActiveTransformation() || _manipulator->HasActiveMode()) { return false; }
            if (ledger.hadManipulator && _manipulatorType != PrimitiveManipulatorType::PrimitiveGizmoTypeNone) {
                if (!_manipulator->IsAttached() || _manipulator->Objects().IsNull()
                    || static_cast<std::size_t>(_manipulator->Objects()->Size()) != expected.size()
                    || _manipulatorSourceLabels.size() != expected.size()) { return false; }
                for (const auto& record : ledger.records) {
                    const auto cached = _manipulator->cachedShapes().find(record.requested.presentation);
                    const auto& saved = committed ? record.candidate : record.previous;
                    if (cached == _manipulator->cachedShapes().end() || !cached->second.IsEqual(saved.shape)) { return false; }
                }
            } else if (_manipulator->IsAttached()) { return false; }
            myContext->UpdateCurrentViewer();
            return true;
        } catch (...) { return false; }
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
    Standard_Boolean ObjectInteractor::debugCycleBooleanSelection(BooleanAction action,
        const std::string& entity, bool beginEmpty) noexcept {
        if (_booleanOpController == nullptr) return Standard_False;
        if (beginEmpty) return entity.empty()
            ? _booleanOpController->debugBeginEmptySelection(action) : Standard_False;
        return _booleanOpController->hasActiveOperation(action)
            ? _booleanOpController->debugCycleSelection(entity) : Standard_False;
    }

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

	void ObjectInteractor::debugSetBooleanMetadataFailurePhase(
		const Standard_Size phase) noexcept {
		_booleanOpController->debugSetMetadataFailurePhase(phase);
	}

		void ObjectInteractor::debugSetBooleanAbortFailureCount(
			const Standard_Size count) noexcept {
			_booleanOpController->debugSetAbortFailureCount(count);
		}

		void ObjectInteractor::debugSetBooleanPostCommitInspectFailureCount(
			const Standard_Size count) noexcept {
			_booleanOpController->debugSetPostCommitInspectFailureCount(count);
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

    void ObjectInteractor::debugSetLinearArrayProfileCopyFault(
        const Standard_Integer mode) noexcept {
        if (_linearArrayController != nullptr) {
            _linearArrayController->debugSetProfileCopyFault(mode);
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

	Standard_Boolean ObjectInteractor::beginRadialArray() noexcept {
		if (_radialArrayController == nullptr) {
			return Standard_False;
		}
		const Standard_Boolean didBegin =
			_radialArrayController->begin();
		if (!didBegin
			&& !_radialArrayController->hasActiveOperation()
			&& _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray) {
			_manipulatorType =
				PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
		}
		return didBegin;
	}

	RadialArrayApplyResult ObjectInteractor::applyRadialArray() noexcept {
		return _radialArrayController == nullptr
			? RadialArrayApplyResult::NoChange
			: _radialArrayController->apply();
	}

	Standard_Boolean ObjectInteractor::cancelRadialArray() noexcept {
		return _radialArrayController == nullptr
			|| _radialArrayController->cancel();
	}

	Standard_Boolean ObjectInteractor::setRadialArrayCount(
		const Standard_Integer count) noexcept {
		return _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray
			&& _radialArrayController != nullptr
			&& _radialArrayController->setCount(count);
	}

	Standard_Boolean ObjectInteractor::setRadialArraySweepDegrees(
		const Standard_Real sweepDegrees) noexcept {
		return _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray
			&& _radialArrayController != nullptr
			&& _radialArrayController->setSweepDegrees(sweepDegrees);
	}

	Standard_Integer ObjectInteractor::radialArrayCount() const noexcept {
		return _radialArrayController == nullptr
			? RadialArrayOperationController::kDefaultCount
			: _radialArrayController->count();
	}

	Standard_Real
	ObjectInteractor::radialArraySweepDegrees() const noexcept {
		return _radialArrayController == nullptr
			? RadialArrayOperationController::kDefaultSweepDegrees
			: _radialArrayController->sweepDegrees();
	}

	Standard_Real ObjectInteractor::radialArrayMetersPerUnit() const noexcept {
		return _radialArrayController == nullptr
			? 0.0 : _radialArrayController->metersPerUnit();
	}

	std::pair<Standard_Integer, Standard_Integer>
	ObjectInteractor::radialArrayCountRange() const noexcept {
		return _radialArrayController == nullptr
			? std::pair<Standard_Integer, Standard_Integer>{
				RadialArrayOperationController::kMinimumCount,
				RadialArrayOperationController::kMaximumCount}
			: _radialArrayController->countRange();
	}

	std::pair<Standard_Real, Standard_Real>
	ObjectInteractor::radialArraySweepDegreesRange() const noexcept {
		return _radialArrayController == nullptr
			? std::pair<Standard_Real, Standard_Real>{
				RadialArrayOperationController::kMinimumSweepDegrees,
				RadialArrayOperationController::kMaximumSweepDegrees}
			: _radialArrayController->sweepDegreesRange();
	}

	OcctReferenceAxisReadState ObjectInteractor::radialArrayReferenceAxis(
		OcctReferenceAxis& axis) const noexcept {
		return _radialArrayController == nullptr
			? OcctReferenceAxisReadState::Invalid
			: _radialArrayController->referenceAxis(axis);
	}

	std::uint64_t
	ObjectInteractor::radialArrayReferenceAuthorityToken() const noexcept {
		return _radialArrayController == nullptr
			? 0 : _radialArrayController->referenceAuthorityToken();
	}

	Standard_Boolean
	ObjectInteractor::convertRadialArrayReferenceAxisSpaces(
		const OcctReferenceSpace pivotSpace,
		const OcctReferenceSpace directionSpace,
		const std::uint64_t expectedAuthorityToken,
		OcctReferenceAxis& axis) const noexcept {
		return _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray
			&& _radialArrayController != nullptr
			&& _radialArrayController->convertReferenceAxisSpaces(
				pivotSpace,
				directionSpace,
				expectedAuthorityToken,
				axis);
	}

	RadialArrayReferenceEditResult
	ObjectInteractor::setRadialArrayReferenceAxis(
		const OcctReferenceAxis& axis,
		const std::uint64_t expectedAuthorityToken) noexcept {
		return _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray
			&& _radialArrayController != nullptr
			? _radialArrayController->setReferenceAxis(
				axis, expectedAuthorityToken)
			: RadialArrayReferenceEditResult::RetryableFailure;
	}

	RadialArrayReferenceEditResult
	ObjectInteractor::resetRadialArrayReferenceAxis(
		const std::uint64_t expectedAuthorityToken) noexcept {
		return _manipulatorType
				== PrimitiveManipulatorType::PrimitiveGizmoTypeRadialArray
			&& _radialArrayController != nullptr
			? _radialArrayController->resetReferenceAxis(
				expectedAuthorityToken)
			: RadialArrayReferenceEditResult::RetryableFailure;
	}

	Standard_Boolean ObjectInteractor::canApplyRadialArray() const noexcept {
		return _radialArrayController != nullptr
			&& _radialArrayController->canApply();
	}

	Standard_Boolean ObjectInteractor::hasActiveRadialArray() const noexcept {
		return _radialArrayController != nullptr
			&& _radialArrayController->hasActiveOperation();
	}

	Standard_Boolean
	ObjectInteractor::hasUnresolvedRadialArray() const noexcept {
		return _radialArrayController != nullptr
			&& _radialArrayController->hasUnresolvedState();
	}

	RadialArrayPreviewState
	ObjectInteractor::radialArrayPreviewState() const noexcept {
		return _radialArrayController == nullptr
			? RadialArrayPreviewState::Unavailable
			: _radialArrayController->previewState();
	}

	std::uint64_t
	ObjectInteractor::radialArrayPreviewGeneration() const noexcept {
		return _radialArrayController == nullptr
			? 0 : _radialArrayController->previewGeneration();
	}

	Standard_Boolean ObjectInteractor::captureRadialArrayPreview(
		RadialArrayPreviewCapture& capture) const noexcept {
		capture = {};
		return _radialArrayController != nullptr
			&& _radialArrayController->capturePreview(capture);
	}

	Standard_Boolean
	ObjectInteractor::canPublishEmptyRadialArrayPreview() const noexcept {
		return _radialArrayController != nullptr
			&& _radialArrayController->canPublishEmptyPreview();
	}

	void ObjectInteractor::setRadialArrayPreviewStateChangedCallback(
		std::function<void()> callback) {
		if (_radialArrayController != nullptr) {
			_radialArrayController->setPreviewStateChangedCallback(
				std::move(callback));
		}
	}

#ifdef DEBUG
	RadialArrayPreviewDebugState
	ObjectInteractor::debugRadialArrayPreviewState() const noexcept {
		return _radialArrayController == nullptr
			? RadialArrayPreviewDebugState{}
			: _radialArrayController->debugPreviewState();
	}

	void ObjectInteractor::
	debugSetRadialArrayBeginOwnedCommandMismatchCount(
		const Standard_Size count) noexcept {
		if (_radialArrayController != nullptr) {
			_radialArrayController
				->debugSetBeginOwnedCommandMismatchCount(count);
		}
	}

	void ObjectInteractor::debugSetRadialArrayTransactionFailureCount(
		const Standard_Size count) noexcept {
		if (_radialArrayController != nullptr) {
			_radialArrayController->debugSetTransactionFailureCount(count);
		}
	}

	void ObjectInteractor::debugSetRadialArrayAbortFailureCount(
		const Standard_Size count) noexcept {
		if (_radialArrayController != nullptr) {
			_radialArrayController->debugSetAbortFailureCount(count);
		}
	}

	void ObjectInteractor::debugSetRadialArrayEraseFailureCount(
		const Standard_Size count) noexcept {
		if (_radialArrayController != nullptr) {
			_radialArrayController->debugSetEraseFailureCount(count);
		}
	}

	void ObjectInteractor::debugSetRadialArrayApplyCommitMode(
		const Standard_Integer mode) noexcept {
		if (_radialArrayController != nullptr) {
			_radialArrayController->debugSetApplyCommitMode(mode);
		}
	}

	void ObjectInteractor::debugSetRadialArrayPostCommitInspectMode(
		const Standard_Integer mode) noexcept {
		if (_radialArrayController != nullptr) {
			_radialArrayController->debugSetPostCommitInspectMode(mode);
		}
	}

    void ObjectInteractor::debugSetRadialArrayProfileCopyFault(
        const Standard_Integer mode) noexcept {
        if (_radialArrayController != nullptr) {
            _radialArrayController->debugSetProfileCopyFault(mode);
        }
    }

	void ObjectInteractor::debugSetMaximumRadialArrayTopologyNodes(
		const Standard_Size limit) noexcept {
		if (_radialArrayController != nullptr) {
			_radialArrayController->debugSetMaximumTopologyNodes(limit);
		}
	}

	Standard_Boolean ObjectInteractor::
	debugMutateRadialArraySourcePersistedTransform() noexcept {
		return _radialArrayController != nullptr
			&& _radialArrayController
				->debugMutateSourcePersistedTransform();
	}

	void ObjectInteractor::debugSetRadialArrayReferenceEditCommitMode(
		const Standard_Integer mode) noexcept {
		if (_radialArrayController != nullptr) {
			_radialArrayController
				->debugSetReferenceEditCommitMode(mode);
		}
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
        std::vector<OcctGeometryDuplicationRequest> profileRequests;
        Standard_Size projectedTopology = 0;
        for (Core3DManipulatorObjectSequence::Iterator it(*anObjects); it.More(); it.Next()) {
            const auto source = _manipulatorSourceLabels.find(it.Value().get());
            if (source == _manipulatorSourceLabels.end()) return Standard_False;
            OcctObjectNameState owner;
            if (!myDoc->CaptureObjectNameStateForLabel(source->second, owner)) return Standard_False;
            const auto& profile = owner.object.profile;
            const auto& enclosure = owner.object.enclosure;
            Standard_Size currentNodes = 0, retainedNodes = 0, retainedEnclosureNodes = 0;
            if (!CountBoundedMirrorTopology(owner.object.shape, aPerSourceLimit, currentNodes)) return Standard_False;
            if (!profile.label.IsNull() && !profile.boundShape.IsEqual(owner.object.shape)
                && (!CountBoundedMirrorTopology(profile.boundShape, aPerSourceLimit, retainedNodes)
                    || !IsTopologicallyValid(profile.boundShape))) return Standard_False;
            if (!enclosure.label.IsNull() && !enclosure.boundShape.IsEqual(owner.object.shape)
                && (!CountBoundedMirrorTopology(enclosure.boundShape, aPerSourceLimit, retainedEnclosureNodes)
                    || !IsTopologicallyValid(enclosure.boundShape))) return Standard_False;
            for (const Standard_Size cost : {currentNodes, retainedNodes, retainedEnclosureNodes,
                                            currentNodes, retainedNodes, retainedEnclosureNodes}) {
                if (cost > anAggregateLimit - projectedTopology) return Standard_False;
                projectedTopology += cost;
            }
            profileRequests.push_back({source->second, 1, !profile.label.IsNull(),
                                       !enclosure.label.IsNull(), !enclosure.label.IsNull()});
        }
        if (!myDoc->CanDuplicateGeometryDefinitions(profileRequests)) return Standard_False;

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
			OcctReferenceAxis aSourceReferenceAxis;
			const OcctReferenceAxisReadState aSourceReferenceAxisState =
				myDoc->ReadReferenceAxisForLabel(
					sourceLabel, aSourceReferenceAxis);
			Standard_Size sourceNodeCount = 0;
			if (sourceStoredShape.IsNull()
				|| !sourceStoredShape.IsEqual(shape->Shape())
				|| sourceEntityIdentifier.empty()
				|| sourceDefinitionIdentifier.empty()
				|| aSourceReferenceAxisState
					== OcctReferenceAxisReadState::Invalid
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
            OcctObjectNameState profileOwner;
            if (!myDoc->CaptureObjectNameStateForLabel(sourceLabel, profileOwner)) return Standard_False;
            const auto& originalProfile = profileOwner.object.profile;
            const auto& originalEnclosure = profileOwner.object.enclosure;
            double documentMetersPerUnit = 0;
            bool documentLengthUnitPresent = false;
            if (!ReadMirrorDocumentUnits(aSourceDocument, documentMetersPerUnit,
                                         documentLengthUnitPresent)) return Standard_False;
            Standard_Size retainedProfileNodes = 0;
            if (!originalProfile.label.IsNull() && !originalProfile.boundShape.IsEqual(sourceStoredShape)) {
                if (!CountBoundedMirrorTopology(originalProfile.boundShape, aPerSourceLimit, retainedProfileNodes)
                    || retainedProfileNodes > anAggregateLimit - anAggregateTopologyNodes
                    || !IsTopologicallyValid(originalProfile.boundShape)) return Standard_False;
                anAggregateTopologyNodes += retainedProfileNodes;
            }
            Standard_Size retainedEnclosureNodes = 0;
            if (!originalEnclosure.label.IsNull() && !originalEnclosure.boundShape.IsEqual(sourceStoredShape)) {
                if (!CountBoundedMirrorTopology(originalEnclosure.boundShape, aPerSourceLimit, retainedEnclosureNodes)
                    || retainedEnclosureNodes > anAggregateLimit - anAggregateTopologyNodes
                    || !IsTopologicallyValid(originalEnclosure.boundShape)) return Standard_False;
                anAggregateTopologyNodes += retainedEnclosureNodes;
            }

			gp_Pnt offset = theWorldPlane.Location();
			gp_Dir mirrorAxis = theWorldPlane.Direction();
			gp_Dir mirrorPln = theWorldPlane.XDirection();
			
			mirrorAxis.Transform(aTrsfSelected.Inverted());
			mirrorPln.Transform(aTrsfSelected.Inverted());
			offset.Transform(aTrsfSelected.Inverted());

			gp_Trsf aTrsfMirror;
			aTrsfMirror.SetMirror(gp_Ax2(offset, mirrorAxis, mirrorPln));
			const gp_Trsf aBakedTransform =
				aTrsfSelected * aTrsfMirror;
            if (!originalProfile.label.IsNull()) {
                gp_Trsf originalFrame;
                if (originalProfile.parameters.constructionFrame
                    && !originalProfile.parameters.constructionFrame->Transform(originalFrame)) return Standard_False;
                profile::ConstructionFrame composed;
                if (!profile::ConstructionFrame::Capture(aBakedTransform * originalFrame, composed)) return Standard_False;
            }
			
            if (!originalEnclosure.label.IsNull()) {
                gp_Trsf originalFrame;
                if (originalEnclosure.parameters.definition.constructionFrame
                    && !originalEnclosure.parameters.definition.constructionFrame->Transform(originalFrame)) return Standard_False;
                profile::ConstructionFrame composed;
                if (!profile::ConstructionFrame::Capture(aBakedTransform * originalFrame, composed)) return Standard_False;
            }
			// OCCT's negative-transform mesh-copy path corrupts allocator state
			// when mirror previews are replaced repeatedly. Keep mesh copying
			// disabled; AIS Display below triangulates the owned trial before
			// renderer-neutral capture reads that cache.
			BRepBuilderAPI_Transform aBRepTrsf(
				shape->Shape(),
				aBakedTransform,
                originalProfile.label.IsNull() && originalEnclosure.label.IsNull() ? Standard_False : Standard_True,
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
            TopoDS_Shape preparedProfileBinding;
            std::string profileIdentifier;
            if (!originalProfile.label.IsNull()) {
                profileIdentifier = OcctDocument::NewProfileIdentifier();
                if (!profile::IsIdentifier(profileIdentifier) || profileIdentifier == originalProfile.identifier
                    || aShapePrs->Shape().IsPartner(sourceStoredShape)) return Standard_False;
                if (originalProfile.boundShape.IsEqual(sourceStoredShape)) {
                    preparedProfileBinding = aShapePrs->Shape();
                } else {
                    BRepBuilderAPI_Transform retained(originalProfile.boundShape, aBakedTransform,
                                                       Standard_True, Standard_False);
                    Standard_Size nodes = 0;
                    if (!retained.IsDone() || retained.Shape().IsNull()
                        || retained.Shape().IsPartner(originalProfile.boundShape)
                        || retained.Shape().IsPartner(aShapePrs->Shape())
                        || !IsShapeWithinModelCoordinates(retained.Shape())
                        || !IsTopologicallyValid(retained.Shape())
                        || !CountBoundedMirrorTopology(retained.Shape(), anAggregateLimit - anAggregateTopologyNodes, nodes)
                        || nodes != retainedProfileNodes) return Standard_False;
                    anAggregateTopologyNodes += nodes;
                    preparedProfileBinding = retained.Shape();
                }
            }
            TopoDS_Shape preparedEnclosureBinding;
            std::string enclosureIdentifier;
            if (!originalEnclosure.label.IsNull()) {
                enclosureIdentifier = OcctDocument::NewProfileIdentifier();
                if (!profile::IsIdentifier(enclosureIdentifier) || enclosureIdentifier == originalEnclosure.identifier
                    || aShapePrs->Shape().IsPartner(sourceStoredShape)) return Standard_False;
                if (originalEnclosure.boundShape.IsEqual(sourceStoredShape)) {
                    preparedEnclosureBinding = aShapePrs->Shape();
                } else {
                    BRepBuilderAPI_Transform retained(originalEnclosure.boundShape, aBakedTransform,
                                                       Standard_True, Standard_False);
                    Standard_Size nodes = 0;
                    if (!retained.IsDone() || retained.Shape().IsNull()
                        || retained.Shape().IsPartner(originalEnclosure.boundShape)
                        || retained.Shape().IsPartner(aShapePrs->Shape())
                        || !IsShapeWithinModelCoordinates(retained.Shape())
                        || !IsTopologicallyValid(retained.Shape())
                        || !CountBoundedMirrorTopology(retained.Shape(), anAggregateLimit - anAggregateTopologyNodes, nodes)
                        || nodes != retainedEnclosureNodes) return Standard_False;
                    anAggregateTopologyNodes += nodes;
                    preparedEnclosureBinding = retained.Shape();
                }
                for (const auto& previous : replacementSources) {
                    if (enclosureIdentifier == previous.enclosureIdentifier
                        || (!previous.preparedEnclosureBinding.IsNull()
                            && preparedEnclosureBinding.IsPartner(previous.preparedEnclosureBinding))) return Standard_False;
                }
            }
			myDoc->LoadObjectMeterial(sourceLabel, aShapePrs);
			replacementObjects.push_back(aShapePrs);
			replacementSources.push_back({
				shape,
				sourceLabel,
				sourceEntityIdentifier,
				sourceDefinitionIdentifier,
				sourceStoredShape,
				aTrsfSelected,
				aBakedTransform,
				aSourceReferenceAxisState,
				aSourceReferenceAxis,
                profileOwner,
                originalProfile.IsCurrent(aSourceDocument, sourceLabel),
                retainedProfileNodes,
                documentMetersPerUnit,
                documentLengthUnitPresent,
                preparedProfileBinding,
                profileIdentifier,
                originalEnclosure.IsCurrent(aSourceDocument, sourceLabel),
                retainedEnclosureNodes,
                preparedEnclosureBinding,
                enclosureIdentifier,
			});
		}
		std::vector<OcctGeometryDuplicationRequest> aSourceLabels;
		aSourceLabels.reserve(replacementSources.size());
		for (const MirrorSourceSnapshot& aSource : replacementSources) {
			aSourceLabels.push_back({aSource.label, 1, !aSource.profileOwner.object.profile.label.IsNull(),
                                        !aSource.profileOwner.object.enclosure.label.IsNull(),
                                        !aSource.profileOwner.object.enclosure.label.IsNull()});
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

	Standard_Boolean
	ObjectInteractor::mirrorPlanePickingAuthorityMatches() const noexcept {
		constexpr std::size_t kMaximumPickPresentations =
			limits::kMaximumLeafPresentations + 1;
		if (!_mirrorPlanePicking
			|| _manipulatorType
				!= PrimitiveManipulatorType::PrimitiveGizmoTypeMirror
			|| myContext.IsNull() || myDoc.IsNull()
			|| myDoc->Document().IsNull()
			|| _manipulator.IsNull() || !_manipulator->IsAttached()
			|| _mirrorPlanePickModes.size() < 2
			|| _mirrorPlanePickModes.size() > kMaximumPickPresentations) {
			return Standard_False;
		}

		try {
			OCC_CATCH_SIGNALS
			const Standard_Integer aShapeMode =
				AIS_Shape::SelectionMode(TopAbs_SHAPE);
			const Standard_Integer aFaceMode =
				AIS_Shape::SelectionMode(TopAbs_FACE);
			const MirrorPlanePickSelectionModes& aManipulatorSnapshot =
				_mirrorPlanePickModes.back();
			if (aManipulatorSnapshot.presentation != _manipulator
				|| !myContext->IsDisplayed(_manipulator)
				|| _manipulator->HasActiveMode()
				|| _manipulator->HasActiveTransformation()
				|| !SelectionModesMatch(myContext, _manipulator, {})) {
				return Standard_False;
			}

			std::unordered_set<const AIS_InteractiveObject*>
				aTrackedPresentations;
			aTrackedPresentations.reserve(
				_mirrorPlanePickModes.size() - 1);
			for (std::size_t anIndex = 0;
				 anIndex + 1 < _mirrorPlanePickModes.size(); ++anIndex) {
				const MirrorPlanePickSelectionModes& aSnapshot =
					_mirrorPlanePickModes[anIndex];
				if (aSnapshot.presentation.IsNull()
					|| aSnapshot.presentation == _manipulator) {
					return Standard_False;
				}
				const Handle(AIS_Shape) aShape =
					Handle(AIS_Shape)::DownCast(aSnapshot.presentation);
				const TDF_Label aLabel =
					myDoc->ShapeLabel(aSnapshot.presentation);
				const OcctGeometryRepresentation aRepresentation =
					myDoc->GeometryRepresentationForLabel(aLabel);
				const TopoDS_Shape aStoredShape = aLabel.IsNull()
					? TopoDS_Shape()
					: XCAFDoc_ShapeTool::GetShape(aLabel);
				if (aSnapshot.modes.size() != 1
					|| aSnapshot.modes.front() != aShapeMode
					|| aShape.IsNull() || aShape->Shape().IsNull()
					|| !myContext->IsDisplayed(aSnapshot.presentation)
					|| !myDoc->IsPresentationEditable(
						aSnapshot.presentation)
					|| aLabel.IsNull()
					|| !myDoc->IsEditableFreeSimpleDefinitionLabel(aLabel)
					|| !IsBRepModelingRepresentation(aRepresentation)
					|| aStoredShape.IsNull()
					|| !aStoredShape.IsEqual(aShape->Shape())
					|| !aTrackedPresentations.insert(
						aSnapshot.presentation.get()).second
					|| !SelectionModesMatch(
						myContext,
						aSnapshot.presentation,
						{aShapeMode, aFaceMode})) {
					return Standard_False;
				}
			}

			std::unordered_set<const AIS_InteractiveObject*>
				aSeenTrackedPresentations;
			aSeenTrackedPresentations.reserve(
				aTrackedPresentations.size());
			AIS_ListOfInteractive aDisplayed;
			myContext->DisplayedObjects(aDisplayed);
			for (AIS_ListIteratorOfListOfInteractive anIterator(aDisplayed);
				 anIterator.More(); anIterator.Next()) {
				const Handle(AIS_InteractiveObject)& aPresentation =
					anIterator.Value();
				if (aPresentation == _manipulator) {
					continue;
				}
				if (aTrackedPresentations.find(aPresentation.get())
						!= aTrackedPresentations.end()) {
					aSeenTrackedPresentations.insert(aPresentation.get());
					continue;
				}

				const TDF_Label aLabel = myDoc->ShapeLabel(aPresentation);
				const OcctGeometryRepresentation aRepresentation =
					myDoc->GeometryRepresentationForLabel(aLabel);
				if (aRepresentation == OcctGeometryRepresentation::Invalid) {
					// Transient operation presentations do not participate in
					// committed topology authority.
					continue;
				}
				std::vector<Standard_Integer> anExpectedModes;
				if (myDoc->IsPresentationEditable(aPresentation)) {
					if (aRepresentation
							== OcctGeometryRepresentation::TriangleMesh
						|| IsBRepModelingRepresentation(aRepresentation)) {
						anExpectedModes.push_back(aShapeMode);
					} else {
						return Standard_False;
					}
				}
				if (!SelectionModesMatch(
						myContext, aPresentation, anExpectedModes)) {
					return Standard_False;
				}
			}
			if (aSeenTrackedPresentations.size()
				!= aTrackedPresentations.size()) {
				return Standard_False;
			}

			const Handle(StdSelect_ViewerSelector3d)& aSelector =
				myContext->MainSelector();
			return !aSelector.IsNull()
				&& aSelector->CustomPixelTolerance() < 0;
		} catch (...) {
			return Standard_False;
		}
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

    bool ObjectInteractor::canFrameMirrorPreview(
        const std::uint64_t expectedGeneration) const noexcept {
        try {
            if (_mirrorPreviewState != MirrorPreviewState::Ready
                || _manipulatorGestureActive
                || _mirrorPreviewGeneration != expectedGeneration
                || _mirrorPlanePicking || _trialMirrorUsesCustomPlane
                || _customMirrorPlane.has_value()
                || !_mirrorReferencePresentations.empty()
                || _mirrorOwnsDocumentCommand || !_pendingMirrorResults.empty()
                || myContext.IsNull() || _trialMirrorSources.empty()
                || _trialMirrorSources.size() > kMaxMirrorPreviewBodies
                || !canApplyMirror() || !mirrorSourcesAreCurrent()) { return false; }
            // The native fit uses selected Model bounds. Prove those objects
            // are exactly the sources captured by this preview, not a later
            // selection that merely shares the same document/model revision.
            if (myContext->NbSelected()
                != static_cast<Standard_Integer>(_trialMirrorSources.size())) { return false; }
            for (const auto& source : _trialMirrorSources) {
                if (source.presentation.IsNull()
                    || !myContext->IsSelected(source.presentation)) { return false; }
            }
            return true;
        } catch (...) { return false; }
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
				OcctReferenceAxis aReferenceAxis;
				const OcctReferenceAxisReadState aReferenceAxisState =
					myDoc->ReadReferenceAxisForLabel(
						aSource.label, aReferenceAxis);
				Standard_Size aSourceNodes = 0;
				Standard_Size aResultNodes = 0;
                OcctObjectNameState profileOwner;
                double documentMetersPerUnit = 0;
                bool documentLengthUnitPresent = false;
                if (!ReadMirrorDocumentUnits(aDocument, documentMetersPerUnit, documentLengthUnitPresent)
                    || documentLengthUnitPresent != aSource.documentLengthUnitPresent
                    || documentMetersPerUnit != aSource.documentMetersPerUnit
                    || !myDoc->CaptureObjectNameStateForLabel(aSource.label, profileOwner)
                    || !profileOwner.IsEqual(aSource.profileOwner)
                    || profileOwner.object.profile.IsCurrent(aDocument, aSource.label) != aSource.profileCurrent
                    || profileOwner.object.enclosure.IsCurrent(aDocument, aSource.label) != aSource.enclosureCurrent)
                    return Standard_False;
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
					|| aReferenceAxisState != aSource.referenceAxisState
					|| aReferenceAxisState
						== OcctReferenceAxisReadState::Invalid
					|| ReferenceAxisDiffers(
						aReferenceAxis, aSource.referenceAxis)
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
                const auto& originalProfile = profileOwner.object.profile;
                if (!originalProfile.label.IsNull()) {
                    if (aSource.preparedProfileBinding.IsNull()
                        || aSource.preparedProfileBinding.IsPartner(originalProfile.boundShape)
                        || (aSource.preparedProfileBinding.IsEqual(aResult->Shape())
                            != originalProfile.boundShape.IsEqual(aStored))) return Standard_False;
                    if (!originalProfile.boundShape.IsEqual(aStored)) {
                        Standard_Size originalNodes = 0, candidateNodes = 0;
                        if (!CountBoundedMirrorTopology(originalProfile.boundShape, aPerSourceLimit, originalNodes)
                            || originalNodes != aSource.retainedProfileTopologyNodes
                            || originalNodes > anAggregateLimit - anAggregateNodes) return Standard_False;
                        anAggregateNodes += originalNodes;
                        if (!CountBoundedMirrorTopology(aSource.preparedProfileBinding,
                                anAggregateLimit - anAggregateNodes, candidateNodes)
                            || candidateNodes != originalNodes
                            || !IsTopologicallyValid(aSource.preparedProfileBinding)) return Standard_False;
                        anAggregateNodes += candidateNodes;
                    }
                } else if (!aSource.preparedProfileBinding.IsNull() || !aSource.profileIdentifier.empty()) return Standard_False;
                const auto& originalEnclosure = profileOwner.object.enclosure;
                if (!originalEnclosure.label.IsNull()) {
                    if (aSource.preparedEnclosureBinding.IsNull()
                        || aSource.preparedEnclosureBinding.IsPartner(originalEnclosure.boundShape)
                        || (aSource.preparedEnclosureBinding.IsEqual(aResult->Shape())
                            != originalEnclosure.boundShape.IsEqual(aStored))) return Standard_False;
                    if (!originalEnclosure.boundShape.IsEqual(aStored)) {
                        Standard_Size originalNodes = 0, candidateNodes = 0;
                        if (!CountBoundedMirrorTopology(originalEnclosure.boundShape, aPerSourceLimit, originalNodes)
                            || originalNodes != aSource.retainedEnclosureTopologyNodes
                            || originalNodes > anAggregateLimit - anAggregateNodes) return Standard_False;
                        anAggregateNodes += originalNodes;
                        if (!CountBoundedMirrorTopology(aSource.preparedEnclosureBinding,
                                anAggregateLimit - anAggregateNodes, candidateNodes)
                            || candidateNodes != originalNodes
                            || !IsTopologicallyValid(aSource.preparedEnclosureBinding)) return Standard_False;
                        anAggregateNodes += candidateNodes;
                    }
                } else if (!aSource.preparedEnclosureBinding.IsNull() || !aSource.enclosureIdentifier.empty()) return Standard_False;
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
            if (_trialMirrorSources.empty()) return MirrorDocumentState::Unavailable;
            double documentMetersPerUnit = 0;
            bool documentLengthUnitPresent = false;
            if (!ReadMirrorDocumentUnits(aDocument, documentMetersPerUnit, documentLengthUnitPresent))
                return MirrorDocumentState::Unavailable;
            for (const auto& source : _trialMirrorSources) {
                OcctObjectNameState current;
                if (documentLengthUnitPresent != source.documentLengthUnitPresent
                    || documentMetersPerUnit != source.documentMetersPerUnit
                    || !myDoc->CaptureObjectNameStateForLabel(source.label, current)
                    || !current.IsEqual(source.profileOwner)
                    || current.object.profile.IsCurrent(aDocument, source.label) != source.profileCurrent
                    || current.object.enclosure.IsCurrent(aDocument, source.label) != source.enclosureCurrent)
                    return MirrorDocumentState::PartialOrMismatched;
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
                profile::Record storedProfile;
                enclosure::Record storedEnclosure;
                if (!profile::Read(aDocument, aResult.label, storedProfile)
                    || !enclosure::Read(aDocument, aResult.label, storedEnclosure)) return MirrorDocumentState::PartialOrMismatched;
				if (aStored.IsNull()) {
                    if (!storedProfile.label.IsNull() || !storedEnclosure.label.IsNull()) return MirrorDocumentState::PartialOrMismatched;
					++aMissingCount;
					continue;
				}
				OcctReferenceAxis aReferenceAxis;
				const OcctReferenceAxisReadState aReferenceAxisState =
					myDoc->ReadReferenceAxisForLabel(
						aResult.label, aReferenceAxis);
                OcctObjectNameState candidateOwner;
                if (!aResult.profileCandidateSealed
                    || !myDoc->CaptureObjectNameStateForLabel(aResult.label, candidateOwner)
                    || !candidateOwner.IsEqual(aResult.expectedProfileOwner)
                    || storedProfile.IsCurrent(aDocument, aResult.label) != aResult.expectedProfileCurrent
                    || storedEnclosure.IsCurrent(aDocument, aResult.label) != aResult.expectedEnclosureCurrent
                    || aResult.expectedShape.IsNull()
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
						aResult.label)
					|| TransformDiffers(
						myDoc->ObjectTransformForLabel(aResult.label),
						aResult.expectedTransform)
					|| aReferenceAxisState
						!= aResult.expectedReferenceAxisState
					|| aReferenceAxisState
						== OcctReferenceAxisReadState::Invalid
					|| ReferenceAxisDiffers(
						aReferenceAxis,
						aResult.expectedReferenceAxis)) {
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
		std::vector<OcctGeometryDuplicationRequest> aSourceLabels;
		try {
			aSourceLabels.reserve(_trialMirrorSources.size());
			for (const MirrorSourceSnapshot& aSource : _trialMirrorSources) {
				aSourceLabels.push_back({aSource.label, 1, !aSource.profileOwner.object.profile.label.IsNull(),
                                        !aSource.profileOwner.object.enclosure.label.IsNull(),
                                        !aSource.profileOwner.object.enclosure.label.IsNull()});
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
				OcctReferenceAxisReadState anExpectedReferenceAxisState =
					OcctReferenceAxisReadState::Invalid;
				OcctReferenceAxis anExpectedReferenceAxis;
				if (!TryExpectedBakedReferenceAxis(
						aSource.referenceAxisState,
						aSource.referenceAxis,
						aSource.bakedTransform,
						anExpectedReferenceAxisState,
						anExpectedReferenceAxis)) {
					return retainRetryableOrUnknown();
				}
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
					aShape->LocalTransformation(),
					anExpectedReferenceAxisState,
					anExpectedReferenceAxis,
				});
				if (_pendingMirrorResults.back().entityIdentifier.empty()
					|| _pendingMirrorResults.back()
						.definitionIdentifier.empty()
					|| myDoc->GeometryRepresentationForLabel(aLabel)
						!= OcctGeometryRepresentation::BRep
					|| TransformDiffers(
						myDoc->ObjectTransformForLabel(aLabel),
						_pendingMirrorResults.back().expectedTransform)
					|| !myDoc->CopyGeometryRepresentation(
						aSource.label, aLabel)
					|| !myDoc->CopyObjectAppearance(
						aSource.label, aLabel)
					|| !myDoc->CopyReferenceAxisThroughBakedTransform(
						aSource.label,
						aLabel,
						aSource.bakedTransform)) {
					return retainRetryableOrUnknown();
				}
				MirrorPendingResult& aPending =
					_pendingMirrorResults.back();
				OcctReferenceAxis aStoredReferenceAxis;
				const OcctReferenceAxisReadState aStoredReferenceAxisState =
					myDoc->ReadReferenceAxisForLabel(
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
                profile::Record candidateProfile;
                auto stageAuthority = aSource.profileOwner.object.profile;
#ifdef DEBUG
                Standard_Integer profileFault = 0;
                if (!stageAuthority.label.IsNull() && _debugMirrorProfileCopyFault >= 1 && _debugMirrorProfileCopyFault <= 2) {
                    profileFault = _debugMirrorProfileCopyFault; _debugMirrorProfileCopyFault = 0;
                    if (profileFault == 1) stageAuthority.values[1] += 1;
                }
#endif
                if (!profile::StageTransformedCopy(aDocument, aSource.label,
                        stageAuthority, aSource.storedShape,
                        aLabel, aSource.preparedProfileBinding, aSource.profileIdentifier,
                        aSource.bakedTransform, candidateProfile)) return retainRetryableOrUnknown();
#ifdef DEBUG
                if (profileFault == 2) {
                    TDataStd_Integer::Set(candidateProfile.label, profile::CountID(), static_cast<int>(candidateProfile.values.size()) + 1);
                    profile::Record readback;
                    if (!profile::Read(aDocument, aLabel, readback)) return retainRetryableOrUnknown();
                    throw Standard_Failure("Injected invalid mirror profile unexpectedly read back");
                }
#endif
                enclosure::Record candidateEnclosure;
                auto enclosureAuthority = aSource.profileOwner.object.enclosure;
#ifdef DEBUG
                Standard_Integer enclosureFault = 0;
                if (!enclosureAuthority.label.IsNull()
                    && _debugMirrorProfileCopyFault >= 12 && _debugMirrorProfileCopyFault <= 13) {
                    enclosureFault = _debugMirrorProfileCopyFault; _debugMirrorProfileCopyFault = 0;
                    if (enclosureFault == 12) enclosureAuthority.values[2] += 1;
                }
#endif
                if (!enclosure::StageTransformedCopy(aDocument, aSource.label,
                        enclosureAuthority, aSource.storedShape, aLabel,
                        aSource.preparedEnclosureBinding, aSource.enclosureIdentifier,
                        aSource.bakedTransform, candidateEnclosure)) return retainRetryableOrUnknown();
#ifdef DEBUG
                if (enclosureFault == 13) {
                    TDataStd_Integer::Set(candidateEnclosure.label, enclosure::CountID(),
                                         static_cast<int>(candidateEnclosure.values.size()) + 1);
                    enclosure::Record readback;
                    if (!enclosure::Read(aDocument, aLabel, readback)) return retainRetryableOrUnknown();
                    throw Standard_Failure("Injected invalid mirror enclosure unexpectedly read back");
                }
#endif
                if (aSource.profileOwner.namePresent) TDataStd_Name::Set(aLabel, aSource.profileOwner.name);
                else aLabel.ForgetAttribute(TDataStd_Name::GetID());
                if (!myDoc->CaptureObjectNameStateForLabel(aLabel, aPending.expectedProfileOwner)
                    || !aPending.expectedProfileOwner.object.profile.IsEqual(candidateProfile)
                    || !aPending.expectedProfileOwner.object.enclosure.IsEqual(candidateEnclosure)
                    || aPending.expectedProfileOwner.namePresent != aSource.profileOwner.namePresent
                    || (aSource.profileOwner.namePresent
                        && !aPending.expectedProfileOwner.name.IsEqual(aSource.profileOwner.name))) return retainRetryableOrUnknown();
                aPending.expectedProfileCurrent = candidateProfile.IsCurrent(aDocument, aLabel);
                if (aPending.expectedProfileCurrent != aSource.profileCurrent) return retainRetryableOrUnknown();
                aPending.expectedEnclosureCurrent = candidateEnclosure.IsCurrent(aDocument, aLabel);
                if (aPending.expectedEnclosureCurrent != aSource.enclosureCurrent) return retainRetryableOrUnknown();
                aPending.profileCandidateSealed = Standard_True;
				myDoc->LoadObjectMeterial(aLabel, aShape);
			}
			if (!myDoc->ValidateGeometryRepresentations()) {
				return retainRetryableOrUnknown();
			}
#ifdef DEBUG
            // Inject authority drift after sealing ordinary (including legacy
            // recipe-less) candidates, inside the owned mirror transaction.
            // Mode 8 changes presence alone for an absent-unit legacy file.
            if (!_trialMirrorSources.empty() && !_pendingMirrorResults.empty()
                && !_trialMirrorSources.front().profileOwner.object.enclosure.label.IsNull()
                && _debugMirrorProfileCopyFault >= 14 && _debugMirrorProfileCopyFault <= 18) {
                const auto fault = _debugMirrorProfileCopyFault; _debugMirrorProfileCopyFault = 0;
                const auto& original = _trialMirrorSources.front().profileOwner.object.enclosure;
                const auto& candidate = _pendingMirrorResults.front().expectedProfileOwner.object.enclosure;
                if (fault == 14 || fault == 15) {
                    TDataStd_AsciiString::Set(fault == 14 ? candidate.label : original.label,
                        enclosure::IdentityID(), TCollection_AsciiString(OcctDocument::NewProfileIdentifier().c_str()));
                } else if (fault == 16 || fault == 17) {
                    TDataStd_Name::Set(fault == 16 ? _pendingMirrorResults.front().label : _trialMirrorSources.front().label,
                                      TCollection_ExtendedString("Unexpected mirrored enclosure name"));
                } else {
                    const int firstFrameScalar = static_cast<int>(candidate.values.size()) - 8 + 1;
                    TDataStd_Real::Set(candidate.label.FindChild(firstFrameScalar, Standard_False),
                                      candidate.parameters.definition.constructionFrame->values[0] + 1);
                }
            }
            if (_debugMirrorProfileCopyFault == 8 || _debugMirrorProfileCopyFault == 9) {
                const auto fault = _debugMirrorProfileCopyFault; _debugMirrorProfileCopyFault = 0;
                XCAFDoc_DocumentTool::SetLengthUnit(aDocument, fault == 8 ? 0.001 : 1.0);
            }
            if (!_trialMirrorSources.empty() && !_pendingMirrorResults.empty()
                && !_trialMirrorSources.front().profileOwner.object.profile.label.IsNull()
                && _debugMirrorProfileCopyFault >= 3 && _debugMirrorProfileCopyFault <= 7) {
                const auto fault = _debugMirrorProfileCopyFault; _debugMirrorProfileCopyFault = 0;
                const auto& original = _trialMirrorSources.front().profileOwner.object.profile;
                const auto& candidate = _pendingMirrorResults.front().expectedProfileOwner.object.profile;
                if (fault == 3 || fault == 4) {
                    TDataStd_AsciiString::Set(fault == 3 ? candidate.label : original.label,
                        profile::IdentityID(), TCollection_AsciiString(OcctDocument::NewProfileIdentifier().c_str()));
                } else if (fault == 5 || fault == 6) {
                    TDataStd_Name::Set(fault == 5 ? _pendingMirrorResults.front().label : _trialMirrorSources.front().label,
                                      TCollection_ExtendedString("Unexpected mirror name"));
                } else {
                    const int firstFrameScalar = static_cast<int>(candidate.values.size()) - 8 + 1;
                    TDataStd_Real::Set(candidate.label.FindChild(firstFrameScalar, Standard_False),
                                      candidate.parameters.constructionFrame->values[0] + 1);
                }
            }
#endif

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
	void ObjectInteractor::debugSetDuplicateCommitMode(
		const Standard_Integer mode) noexcept {
        _debugDuplicateProfileFault = mode >= 4 && mode <= 7 ? mode : 0;
        _debugDuplicateEnclosureFault = mode >= 8 && mode <= 11 ? mode : 0;
		if (mode == 3) {
			_debugDuplicateCommitMode = 0;
			_debugDuplicatePresentationRepairFailureCount = 1;
			return;
		}
		_debugDuplicateCommitMode = mode >= 0 && mode <= 2 ? mode : 0;
		_debugDuplicatePresentationRepairFailureCount = 0;
	}

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

    void ObjectInteractor::debugSetMirrorProfileCopyFault(const Standard_Integer mode) noexcept {
        _debugMirrorProfileCopyFault = ((mode >= 0 && mode <= 9) || (mode >= 12 && mode <= 18)) ? mode : 0;
        // Modes 10/11 simulate a separate committed unit edit between preview
        // capture and Apply. They own only the new command opened here.
        if (mode != 10 && mode != 11) return;
        Handle(TDocStd_Document) document;
        bool ownsCommand = false;
        try {
            if (_mirrorPreviewState != MirrorPreviewState::Ready || !canApplyMirror()
                || _mirrorOwnsDocumentCommand || !_pendingMirrorResults.empty() || myDoc.IsNull()) return;
            document = myDoc->ChangeDocument();
            if (document.IsNull() || document->HasOpenCommand()) return;
            ownsCommand = true;
            document->NewCommand();
            if (!document->HasOpenCommand()) { ownsCommand = false; return; }
            XCAFDoc_DocumentTool::SetLengthUnit(document, mode == 10 ? 0.001 : 1.0);
            (void)document->CommitCommand();
            ownsCommand = document->HasOpenCommand();
        } catch (...) {}
        if (ownsCommand && !document.IsNull()) {
            try { if (document->HasOpenCommand()) document->AbortCommand(); } catch (...) {}
        }
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
