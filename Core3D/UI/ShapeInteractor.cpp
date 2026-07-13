//
//  ShapeInteractor.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 19.04.2024.
//

#include "ShapeInteractor.hpp"
#include "../OCCTKit/Core3DSTEPExchangeLock.h"
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <BRep_Tool.hxx>
#include <StlAPI_Writer.hxx>
#include <RWObj_CafWriter.hxx>
#include <RWGltf_CafWriter.hxx>
#include <STEPControl_Writer.hxx>
#include <Interface_Static.hxx>
#include <TCollection_AsciiString.hxx>
#include <TopoDS_Compound.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <BRepFeat_MakePrism.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>
#include <TDF_ChildIterator.hxx>
#include <BRepTools.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Prs3d_Drawer.hxx>
#include <StdPrs_ToolTriangulatedShape.hxx>
#include <Standard_Failure.hxx>
#include <Standard_ErrorHandler.hxx>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <set>
#include <vector>

#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Iterator.hxx>

#include "../Export/obj/vectornd.h"
#include "../Export/obj/geometry.h"
#include "../Export/obj/importstl.h"
#include "../Export/obj/exportobj.h"

namespace core3d {
	namespace {
		// Synchronous v1 admission is intentionally far below the absolute
		// hostile-input requirements (8192/256). This keeps a local feature on
		// the UI thread bounded while worker/generation-token support is built.
		constexpr Standard_Size kMaximumExtrusionSourceSubshapes = 1024;
		constexpr Standard_Size kMaximumExtrusionProfileEdges = 64;
		constexpr Standard_Real kMaximumExtrusionDistance = 100.0;

		Standard_Boolean IsBRepModelingLabel(
			const Handle(OcctDocument)& document,
			const TDF_Label& label) noexcept {
			if (document.IsNull() || label.IsNull()) {
				return Standard_False;
			}
			if (!document->IsEditableFreeSimpleDefinitionLabel(label)) {
				return Standard_False;
			}
			const OcctGeometryRepresentation representation =
				document->GeometryRepresentationForLabel(label);
			return representation
					== OcctGeometryRepresentation::LegacyUnknown
				|| representation == OcctGeometryRepresentation::BRep;
		}

		enum class ExtrusionDocumentState {
			Unavailable,
			OpenCommand,
			OriginalShape,
			CandidateShape,
			OtherClosedShape,
		};

		ExtrusionDocumentState InspectExtrusionDocumentState(
			const Handle(OcctDocument)& wrapper,
			const TDF_Label& label,
			const TopoDS_Shape& original,
			const TopoDS_Shape& candidate) noexcept {
			try {
				OCC_CATCH_SIGNALS
				const Handle(TDocStd_Document) document = wrapper.IsNull()
					? Handle(TDocStd_Document)()
					: wrapper->Document();
				if (document.IsNull() || label.IsNull()
					|| label.Data() != document->GetData()) {
					return ExtrusionDocumentState::Unavailable;
				}
				if (document->HasOpenCommand()) {
					return ExtrusionDocumentState::OpenCommand;
				}
				const TopoDS_Shape stored =
					XCAFDoc_ShapeTool::GetShape(label);
				if (stored.IsNull()) {
					return ExtrusionDocumentState::OtherClosedShape;
				}
				if (!candidate.IsNull() && stored.IsEqual(candidate)) {
					return ExtrusionDocumentState::CandidateShape;
				}
				if (!original.IsNull() && stored.IsEqual(original)) {
					return ExtrusionDocumentState::OriginalShape;
				}
				return ExtrusionDocumentState::OtherClosedShape;
			} catch (...) {
				return ExtrusionDocumentState::Unavailable;
			}
		}

		Standard_Boolean CountBoundedTopology(
			const TopoDS_Shape& shape,
			const Standard_Size maximum,
			Standard_Size& subshapeCount,
			Standard_Size& solidCount) {
			subshapeCount = 0;
			solidCount = 0;
			if (shape.IsNull() || maximum == 0) {
				return Standard_False;
			}
			TopTools_IndexedMapOfShape visited;
			std::vector<TopoDS_Shape> pending;
			pending.push_back(shape);
			while (!pending.empty()) {
				const TopoDS_Shape current = pending.back();
				pending.pop_back();
				if (current.IsNull() || visited.Contains(current)) {
					continue;
				}
				visited.Add(current);
				if (static_cast<Standard_Size>(visited.Extent()) > maximum) {
					return Standard_False;
				}
				if (current.ShapeType() == TopAbs_SOLID) {
					++solidCount;
				}
				for (TopoDS_Iterator child(current, Standard_True, Standard_True);
					 child.More(); child.Next()) {
					if (pending.size()
						>= static_cast<std::size_t>(maximum)) {
						return Standard_False;
					}
					pending.push_back(child.Value());
				}
			}
			subshapeCount = static_cast<Standard_Size>(visited.Extent());
			return Standard_True;
		}

		Standard_Boolean CountBoundedProfileEdges(
			const TopoDS_Face& face,
			Standard_Size& edgeCount) {
			edgeCount = 0;
			if (face.IsNull()) {
				return Standard_False;
			}
			TopTools_IndexedMapOfShape edges;
			for (TopExp_Explorer edge(face, TopAbs_EDGE);
				 edge.More(); edge.Next()) {
				edges.Add(edge.Current());
				if (static_cast<Standard_Size>(edges.Extent())
					> kMaximumExtrusionProfileEdges) {
					return Standard_False;
				}
			}
			edgeCount = static_cast<Standard_Size>(edges.Extent());
			return edgeCount > 0;
		}

		Standard_Boolean HasStyledXCAFSubshape(
			const Handle(TDocStd_Document)& document,
			const TDF_Label& definition) {
			if (document.IsNull() || definition.IsNull()) {
				return Standard_True;
			}
			const Handle(XCAFDoc_ColorTool) colorTool =
				XCAFDoc_DocumentTool::CheckColorTool(document->Main())
					? XCAFDoc_DocumentTool::ColorTool(document->Main())
					: Handle(XCAFDoc_ColorTool)();
			const Handle(XCAFDoc_LayerTool) layerTool =
				XCAFDoc_DocumentTool::CheckLayerTool(document->Main())
					? XCAFDoc_DocumentTool::LayerTool(document->Main())
					: Handle(XCAFDoc_LayerTool)();
			Standard_Size childCount = 0;
			for (TDF_ChildIterator item(definition, Standard_False);
				 item.More(); item.Next()) {
				if (++childCount > kMaximumExtrusionSourceSubshapes) {
					return Standard_True;
				}
				const TDF_Label& label = item.Value();
				if (!XCAFDoc_ShapeTool::IsSubShape(label)) {
					continue;
				}
				TDF_LabelSequence layers;
				if (label.IsNull()
					|| (!colorTool.IsNull()
						&& (colorTool->IsSet(label, XCAFDoc_ColorGen)
							|| colorTool->IsSet(label, XCAFDoc_ColorSurf)
							|| colorTool->IsSet(label, XCAFDoc_ColorCurv)
							|| !XCAFDoc_ColorTool::IsVisible(label)))
					|| !XCAFDoc_VisMaterialTool::GetShapeMaterial(
						label).IsNull()
					|| (!layerTool.IsNull()
						&& layerTool->GetLayers(label, layers)
						&& !layers.IsEmpty())) {
					return Standard_True;
				}
			}
			return Standard_False;
		}

		Standard_Boolean FaceBelongsToShape(
			const TopoDS_Shape& shape,
			const TopoDS_Face& face) {
			for (TopExp_Explorer item(shape, TopAbs_FACE);
				 item.More(); item.Next()) {
				if (item.Current().IsSame(face)) {
					return Standard_True;
				}
			}
			return Standard_False;
		}

		Standard_Boolean IsEditableFreeSolidDefinition(
			const Handle(OcctDocument)& document,
			const Handle(AIS_Shape)& presentation,
			const TDF_Label& label,
			const TopoDS_Face& face,
			Standard_Size& sourceSubshapeCount,
			Standard_Size& profileEdgeCount) {
			if (document.IsNull() || presentation.IsNull()
				|| presentation->Shape().IsNull() || label.IsNull()
				|| !IsBRepModelingLabel(document, label)
				|| document->Document().IsNull()
				|| label.Data() != document->Document()->GetData()
				|| !document->IsPresentationEditable(presentation)
				|| !document->ShapeLabel(presentation).IsEqual(label)) {
				return Standard_False;
			}
			const Handle(XCAFDoc_ShapeTool) shapeTool =
				XCAFDoc_DocumentTool::ShapeTool(
					document->Document()->Main());
			if (shapeTool.IsNull() || !shapeTool->IsShape(label)
				|| !XCAFDoc_ShapeTool::IsFree(label)
				|| !XCAFDoc_ShapeTool::IsSimpleShape(label)
				|| XCAFDoc_ShapeTool::IsReference(label)
				|| XCAFDoc_ShapeTool::IsComponent(label)
				|| XCAFDoc_ShapeTool::IsAssembly(label)
				|| XCAFDoc_ShapeTool::IsSubShape(label)
				|| presentation->Shape().ShapeType() != TopAbs_SOLID
				|| face.IsNull()) {
				return Standard_False;
			}
			Standard_Size solidCount = 0;
			if (!CountBoundedTopology(
					presentation->Shape(),
					kMaximumExtrusionSourceSubshapes,
					sourceSubshapeCount,
					solidCount)
				|| solidCount != 1) {
				return Standard_False;
			}
			if (HasStyledXCAFSubshape(document->Document(), label)) {
				return Standard_False;
			}
			return FaceBelongsToShape(presentation->Shape(), face)
				&& CountBoundedProfileEdges(face, profileEdgeCount);
		}

		Standard_Boolean BuildExtrusionCandidate(
			const TopoDS_Shape& source,
			const TopoDS_Face& face,
			const Standard_Real distance,
			TopoDS_Shape& candidate,
			Standard_Size& candidateSubshapeCount,
			Standard_Size& candidateSolidCount,
			Standard_Integer& failureStage,
			Standard_Integer& featureStatus,
			Standard_Integer& candidateShapeType,
			Standard_Size& rawDirectChildCount,
			Standard_Integer& rawDirectChildShapeType,
			Standard_Boolean& historyHasModified,
			Standard_Boolean& historyHasGenerated,
			Standard_Boolean& historyFaceDeleted) {
			candidate.Nullify();
			candidateSubshapeCount = 0;
			candidateSolidCount = 0;
			failureStage = 0;
			featureStatus = static_cast<Standard_Integer>(BRepFeat_OK);
			candidateShapeType = -1;
			rawDirectChildCount = 0;
			rawDirectChildShapeType = -1;
			historyHasModified = Standard_False;
			historyHasGenerated = Standard_False;
			historyFaceDeleted = Standard_False;
			const Standard_Real magnitude = std::abs(distance);
			if (source.IsNull() || face.IsNull()
				|| !std::isfinite(distance)
				|| magnitude <= Precision::Confusion()
				|| magnitude > kMaximumExtrusionDistance) {
				failureStage = 1;
				return Standard_False;
			}
			try {
				OCC_CATCH_SIGNALS
				const BRepAdaptor_Surface surface(face, Standard_True);
				if (surface.GetType() != GeomAbs_Plane
					|| (face.Orientation() != TopAbs_FORWARD
						&& face.Orientation() != TopAbs_REVERSED)) {
					failureStage = 2;
					return Standard_False;
				}
				gp_Dir outward = surface.Plane().Axis().Direction();
				if (face.Orientation() == TopAbs_REVERSED) {
					outward.Reverse();
				}
				const Standard_Boolean isProtrusion = distance > 0.0;
				if (!isProtrusion) {
					outward.Reverse();
				}
				BRepFeat_MakePrism feature(
					source,
					face,
					face,
					outward,
					isProtrusion ? 1 : 0,
					Standard_True);
				feature.Perform(magnitude);
				#ifdef DEBUG
				try {
					OCC_CATCH_SIGNALS
					featureStatus = static_cast<Standard_Integer>(
						feature.CurrentStatusError());
				} catch (...) {
					featureStatus = -1;
				}
				#endif
				if (!feature.IsDone()) {
					failureStage = 3;
					return Standard_False;
				}
				const TopoDS_Shape rawCandidate = feature.Shape();
				if (rawCandidate.IsNull()) {
					failureStage = 4;
					return Standard_False;
				}
				candidateShapeType = static_cast<Standard_Integer>(
					rawCandidate.ShapeType());
				Standard_Size rawSubshapeCount = 0;
				Standard_Size rawSolidCount = 0;
				if (!CountBoundedTopology(
						rawCandidate,
						kMaximumExtrusionSourceSubshapes,
						rawSubshapeCount,
						rawSolidCount)) {
					failureStage = 6;
					return Standard_False;
				}
				if (rawCandidate.ShapeType() == TopAbs_SOLID) {
					candidate = rawCandidate;
				} else if (rawCandidate.ShapeType() == TopAbs_COMPOUND
					|| rawCandidate.ShapeType() == TopAbs_COMPSOLID) {
					for (TopoDS_Iterator child(
							rawCandidate, Standard_True, Standard_True);
						 child.More(); child.Next()) {
						++rawDirectChildCount;
						if (rawDirectChildCount == 1) {
							rawDirectChildShapeType =
								static_cast<Standard_Integer>(
									child.Value().ShapeType());
							candidate = child.Value();
						}
						if (rawDirectChildCount > 1) {
							break;
						}
					}
					if (rawDirectChildCount != 1 || candidate.IsNull()
						|| candidate.ShapeType() != TopAbs_SOLID) {
						failureStage = 5;
						return Standard_False;
					}
				} else {
					failureStage = 5;
					return Standard_False;
				}
				if (!CountBoundedTopology(
						candidate,
						kMaximumExtrusionSourceSubshapes,
						candidateSubshapeCount,
						candidateSolidCount)) {
					failureStage = 6;
					return Standard_False;
				}
				if (candidateSolidCount != 1) {
					failureStage = 7;
					return Standard_False;
				}
				// BRepFeat_Form reuses one mutable scratch list for both history
				// accessors. Snapshot each answer before invoking the next one.
				historyHasModified =
					!feature.Modified(face).IsEmpty();
				historyHasGenerated =
					!feature.Generated(face).IsEmpty();
				historyFaceDeleted = feature.IsDeleted(face);
				if (candidate.IsSame(source) || candidate.IsEqual(source)
					|| (!historyHasModified && !historyHasGenerated
						&& !historyFaceDeleted)) {
					failureStage = 8;
					return Standard_False;
				}
				// The OCCT contract explicitly defines GeomControls=false as a
				// topological-only check. It runs only after the candidate is bounded.
				const BRepCheck_Analyzer topologyOnly(
					candidate, Standard_False);
				if (!topologyOnly.IsValid()) {
					failureStage = 9;
					return Standard_False;
				}
				return Standard_True;
			} catch (...) {
				candidate.Nullify();
				candidateSubshapeCount = 0;
				candidateSolidCount = 0;
				failureStage = 10;
				return Standard_False;
			}
		}

			bool CanExportCommittedDocument(
				const Handle(OcctDocument)& document,
				const std::string& filename,
				const OcctGeometryExportFormat format,
				const bool hasActiveTransientOperation) {
				const Handle(TDocStd_Document) transaction = document.IsNull()
					? Handle(TDocStd_Document)()
					: document->ChangeDocument();
				if (filename.empty() || hasActiveTransientOperation
					|| transaction.IsNull()
					|| transaction->HasOpenCommand()
					|| !document->CanExportGeometry(format)) {
				if (!filename.empty()) {
					std::remove(filename.c_str());
				}
				return false;
			}
			return true;
		}

		void AbortOpenCommandNoThrow(
			const Handle(TDocStd_Document)& document) noexcept {
			try {
				if (!document.IsNull() && document->HasOpenCommand()) {
					document->AbortCommand();
				}
			} catch (...) {
			}
		}
	}

	    ShapeInteractor::ShapeInteractor(Handle(Core3DContext) context, Handle(Core3DView) view, Handle(OcctDocument) doc)
	    : Interactor(context, view, doc),
	      _bevelController(std::make_shared<BevelOperationController>(
	          context, doc)) {
	    }

	ShapeInteractor::~ShapeInteractor() noexcept {
		if (!cancelExtrusion()) {
			(void)cancelExtrusion();
		}
		try {
			cancelChamfer();
		} catch (...) {
		}
	}

	Standard_Boolean ShapeInteractor::beginExtrusionSelection() noexcept {
		if (myContext.IsNull()) {
			return Standard_False;
		}
		try {
			OCC_CATCH_SIGNALS
			Handle(AIS_Shape) presentation;
			TopoDS_Face face;
			Standard_Size selectedCount = 0;
			for (myContext->InitSelected(); myContext->MoreSelected();
				 myContext->NextSelected()) {
				if (++selectedCount != 1) {
					cancelExtrusion();
					return Standard_False;
				}
				const Handle(SelectMgr_EntityOwner) owner =
					myContext->SelectedOwner();
				const Handle(StdSelect_BRepOwner) brepOwner =
					Handle(StdSelect_BRepOwner)::DownCast(owner);
				presentation = Handle(AIS_Shape)::DownCast(
					myContext->SelectedInteractive());
				if (owner.IsNull() || !owner->HasSelectable()
					|| brepOwner.IsNull() || !brepOwner->HasShape()
					|| brepOwner->Shape().ShapeType() != TopAbs_FACE
					|| presentation.IsNull()
					|| owner->Selectable() != presentation) {
					cancelExtrusion();
					return Standard_False;
				}
				face = TopoDS::Face(brepOwner->Shape());
			}
			if (selectedCount != 1) {
				cancelExtrusion();
				return Standard_False;
			}
			return beginExtrusionSelectionImpl(presentation, face);
		} catch (...) {
			cancelExtrusion();
			return Standard_False;
		}
	}

	Standard_Boolean ShapeInteractor::beginExtrusionSelectionImpl(
		const Handle(AIS_Shape)& presentation,
		const TopoDS_Face& face) noexcept {
		if (!cancelExtrusion() || _extrusion.IsReady()
			|| _extrusion.ownsCommand) {
			return Standard_False;
		}
#ifdef DEBUG
		_lastExtrusionApplySucceeded = Standard_False;
		_lastExtrusionResultSubshapeCount = 0;
		_lastExtrusionResultSolidCount = 0;
		_lastExtrusionFailureStage = 0;
		_lastExtrusionFeatureStatus = 0;
		_lastExtrusionCandidateShapeType = -1;
		_lastExtrusionRawDirectChildCount = 0;
		_lastExtrusionRawDirectChildShapeType = -1;
		_lastExtrusionHistoryHasModified = Standard_False;
		_lastExtrusionHistoryHasGenerated = Standard_False;
		_lastExtrusionHistoryFaceDeleted = Standard_False;
		_lastExtrusionCandidateVolume = 0.0;
		for (Standard_Real& value : _lastExtrusionCandidateBounds) {
			value = 0.0;
		}
#endif
		try {
			OCC_CATCH_SIGNALS
			const Handle(TDocStd_Document) document = myDoc.IsNull()
				? Handle(TDocStd_Document)()
				: myDoc->ChangeDocument();
			if (document.IsNull() || document->HasOpenCommand()
				|| document->GetUndoLimit() == 0) {
				return Standard_False;
			}
			const TDF_Label label = myDoc->ShapeLabel(presentation);
			if (!IsBRepModelingLabel(myDoc, label)) {
				return Standard_False;
			}
			Standard_Size sourceSubshapeCount = 0;
			Standard_Size profileEdgeCount = 0;
			if (!IsEditableFreeSolidDefinition(
					myDoc,
					presentation,
					label,
					face,
					sourceSubshapeCount,
					profileEdgeCount)) {
				return Standard_False;
			}
			const BRepAdaptor_Surface surface(face, Standard_True);
			if (surface.GetType() != GeomAbs_Plane
				|| (face.Orientation() != TopAbs_FORWARD
					&& face.Orientation() != TopAbs_REVERSED)) {
				return Standard_False;
			}

			_extrusion.label = label;
			_extrusion.originalPresentation = presentation;
			_extrusion.originalShape = presentation->Shape();
			_extrusion.selectedFace = face;
			_extrusion.transform = presentation->LocalTransformation();
			_extrusion.sourceSubshapeCount = sourceSubshapeCount;
			_extrusion.profileEdgeCount = profileEdgeCount;
			return Standard_True;
		} catch (...) {
			clearExtrusionSelection();
			return Standard_False;
		}
	}

	void ShapeInteractor::restoreExtrusionOriginalPresentation() noexcept {
		try {
			if (myContext.IsNull()) {
				return;
			}
			if (!_extrusion.candidatePresentation.IsNull()) {
				myContext->Remove(
					_extrusion.candidatePresentation, Standard_False);
			}
			if (!_extrusion.originalPresentation.IsNull()
				&& !_extrusion.originalShape.IsNull()) {
				_extrusion.originalPresentation->SetShape(
					_extrusion.originalShape);
				_extrusion.originalPresentation->SetLocalTransformation(
					_extrusion.transform);
				myContext->Display(
					_extrusion.originalPresentation,
					AIS_Shaded,
					_topAbsSelMode,
					Standard_False);
				setInteractiveObjectSelectionMode(
					_extrusion.originalPresentation);
			}
		} catch (...) {
		}
	}

	Standard_Boolean ShapeInteractor::discardExtrusionPreview(
		const Standard_Boolean updateViewer) noexcept {
		Standard_Boolean commandResolved = !_extrusion.ownsCommand;
		if (_extrusion.ownsCommand) {
			try {
				OCC_CATCH_SIGNALS
				const TopoDS_Shape candidate =
					_extrusion.candidatePresentation.IsNull()
						? TopoDS_Shape()
						: _extrusion.candidatePresentation->Shape();
				ExtrusionDocumentState state =
					InspectExtrusionDocumentState(
						myDoc,
						_extrusion.label,
						_extrusion.originalShape,
						candidate);
				if (state == ExtrusionDocumentState::OpenCommand) {
					const Handle(TDocStd_Document) document = myDoc.IsNull()
						? Handle(TDocStd_Document)()
						: myDoc->ChangeDocument();
					if (document.IsNull()) {
						throw Standard_Failure(
							"Unable to access extrusion transaction");
					}
#ifdef DEBUG
					if (_debugExtrusionAbortFailureCount > 0) {
						--_debugExtrusionAbortFailureCount;
						throw Standard_Failure(
							"Injected extrusion abort failure");
					}
#endif
					document->AbortCommand();
					state = InspectExtrusionDocumentState(
						myDoc,
						_extrusion.label,
						_extrusion.originalShape,
						candidate);
				}
				commandResolved =
					state == ExtrusionDocumentState::OriginalShape;
			} catch (...) {
				commandResolved = Standard_False;
			}
		}
		if (!commandResolved) {
			if (!_extrusion.commitOutcomeUnknown) {
				_extrusion.rollbackFailed = Standard_True;
			}
			return Standard_False;
		}
		_extrusion.ownsCommand = Standard_False;
		_extrusion.rollbackFailed = Standard_False;
		_extrusion.commitOutcomeUnknown = Standard_False;
		restoreExtrusionOriginalPresentation();
		_extrusion.candidatePresentation.Nullify();
		_extrusion.distance = 0.0;
		_extrusion.candidateSubshapeCount = 0;
		_extrusion.candidateSolidCount = 0;
		if (updateViewer) {
			try {
				if (!myContext.IsNull()) {
					myContext->UpdateCurrentViewer();
				}
			} catch (...) {
			}
		}
		return Standard_True;
	}

	void ShapeInteractor::clearExtrusionSelection() noexcept {
		_extrusion = ExtrusionSelection();
	}

	Standard_Boolean ShapeInteractor::setExtrusionValueForSelection(
		const Standard_Real distance) noexcept {
		try {
			OCC_CATCH_SIGNALS
			const Standard_Real magnitude = std::abs(distance);
			if (!std::isfinite(distance)
				|| magnitude > kMaximumExtrusionDistance) {
				(void)discardExtrusionPreview(Standard_True);
				return Standard_False;
			}
			if (magnitude <= Precision::Confusion()) {
				return discardExtrusionPreview(Standard_True);
			}
			if (!_extrusion.IsReady()) {
				return Standard_False;
			}

			// Every recompute starts from the exact committed source. The prior
			// candidate command is aborted before any new feature work begins.
			if (!discardExtrusionPreview(Standard_False)) {
				return Standard_False;
			}
			const Handle(TDocStd_Document) document = myDoc.IsNull()
				? Handle(TDocStd_Document)()
				: myDoc->ChangeDocument();
			const TDF_Label currentLabel = myDoc->ShapeLabel(
				_extrusion.originalPresentation);
			if (document.IsNull() || document->HasOpenCommand()
				|| document->GetUndoLimit() == 0
				|| _extrusion.label.Data() != document->GetData()
				|| !IsBRepModelingLabel(myDoc, _extrusion.label)
				|| currentLabel.IsNull()
				|| !currentLabel.IsEqual(_extrusion.label)
				|| !XCAFDoc_ShapeTool::GetShape(_extrusion.label)
					.IsEqual(_extrusion.originalShape)) {
				cancelExtrusion();
				return Standard_False;
			}

			TopoDS_Shape candidateShape;
			Standard_Size candidateSubshapeCount = 0;
			Standard_Size candidateSolidCount = 0;
			Standard_Integer failureStage = 0;
			Standard_Integer featureStatus = 0;
			Standard_Integer candidateShapeType = -1;
			Standard_Size rawDirectChildCount = 0;
			Standard_Integer rawDirectChildShapeType = -1;
			Standard_Boolean historyHasModified = Standard_False;
			Standard_Boolean historyHasGenerated = Standard_False;
			Standard_Boolean historyFaceDeleted = Standard_False;
			if (!BuildExtrusionCandidate(
					_extrusion.originalShape,
					_extrusion.selectedFace,
					distance,
					candidateShape,
					candidateSubshapeCount,
					candidateSolidCount,
					failureStage,
					featureStatus,
					candidateShapeType,
					rawDirectChildCount,
					rawDirectChildShapeType,
					historyHasModified,
					historyHasGenerated,
					historyFaceDeleted)) {
#ifdef DEBUG
				_lastExtrusionFailureStage = failureStage;
				_lastExtrusionFeatureStatus = featureStatus;
				_lastExtrusionCandidateShapeType = candidateShapeType;
				_lastExtrusionRawDirectChildCount =
					rawDirectChildCount;
				_lastExtrusionRawDirectChildShapeType =
					rawDirectChildShapeType;
				_lastExtrusionHistoryHasModified = historyHasModified;
				_lastExtrusionHistoryHasGenerated = historyHasGenerated;
				_lastExtrusionHistoryFaceDeleted = historyFaceDeleted;
#endif
				restoreExtrusionOriginalPresentation();
				myContext->UpdateCurrentViewer();
				return Standard_False;
			}

			Handle(AIS_Shape) candidate = new AIS_Shape(candidateShape);
			candidate->SetLocalTransformation(_extrusion.transform);
			myDoc->LoadObjectMeterial(_extrusion.label, candidate);
			// Mark command ownership intent before NewCommand so an exception
			// cannot strand an open command behind a false ownership bit.
			_extrusion.ownsCommand = Standard_True;
			document->NewCommand();
			_extrusion.ownsCommand = document->HasOpenCommand();
			if (!_extrusion.ownsCommand) {
				return Standard_False;
			}
			_extrusion.candidatePresentation = candidate;
			if (!myDoc->ReplaceShape(_extrusion.label, candidate)) {
				throw Standard_Failure(
					"Unable to replace extrusion preview geometry");
			}
			if (myDoc->GeometryRepresentationForLabel(_extrusion.label)
				!= OcctGeometryRepresentation::BRep) {
				throw Standard_Failure(
					"Extrusion result is not persisted as BRep");
			}

			myContext->ClearSelected(Standard_False);
			myContext->Display(
				_extrusion.originalPresentation,
				AIS_WireFrame,
				_topAbsSelMode,
				Standard_False);
			myContext->Deactivate(_extrusion.originalPresentation);
			myContext->Display(
				candidate,
				AIS_Shaded,
				_topAbsSelMode,
				Standard_False);
			myContext->Deactivate(candidate);
			_extrusion.distance = distance;
			_extrusion.candidateSubshapeCount = candidateSubshapeCount;
			_extrusion.candidateSolidCount = candidateSolidCount;
#ifdef DEBUG
			_lastExtrusionFailureStage = 0;
			_lastExtrusionFeatureStatus = featureStatus;
			_lastExtrusionCandidateShapeType = candidateShapeType;
			_lastExtrusionRawDirectChildCount = rawDirectChildCount;
			_lastExtrusionRawDirectChildShapeType =
				rawDirectChildShapeType;
			_lastExtrusionHistoryHasModified = historyHasModified;
			_lastExtrusionHistoryHasGenerated = historyHasGenerated;
			_lastExtrusionHistoryFaceDeleted = historyFaceDeleted;
			GProp_GProps volume;
			BRepGProp::VolumeProperties(candidateShape, volume);
			_lastExtrusionCandidateVolume = volume.Mass();
			Bnd_Box bounds;
			BRepBndLib::Add(candidateShape, bounds, Standard_False);
			if (!bounds.IsVoid()) {
				bounds.Get(
					_lastExtrusionCandidateBounds[0],
					_lastExtrusionCandidateBounds[1],
					_lastExtrusionCandidateBounds[2],
					_lastExtrusionCandidateBounds[3],
					_lastExtrusionCandidateBounds[4],
					_lastExtrusionCandidateBounds[5]);
			}
#endif
			myContext->UpdateCurrentViewer();
			return Standard_True;
		} catch (...) {
			(void)discardExtrusionPreview(Standard_True);
			return Standard_False;
		}
	}

	Standard_Boolean ShapeInteractor::canApplyExtrusion() const noexcept {
		try {
			const Handle(TDocStd_Document) document = myDoc.IsNull()
				? Handle(TDocStd_Document)()
				: myDoc->Document();
			return _extrusion.IsReady()
				&& _extrusion.ownsCommand
				&& !_extrusion.rollbackFailed
				&& !_extrusion.candidatePresentation.IsNull()
				&& !_extrusion.candidatePresentation->Shape().IsNull()
				&& std::isfinite(_extrusion.distance)
				&& std::abs(_extrusion.distance) > Precision::Confusion()
				&& _extrusion.candidateSolidCount == 1
				&& !document.IsNull()
				&& document->GetUndoLimit() != 0
				&& document->HasOpenCommand()
				&& _extrusion.label.Data() == document->GetData()
				&& myDoc->GeometryRepresentationForLabel(
					_extrusion.label) == OcctGeometryRepresentation::BRep
				&& myDoc->IsPresentationEditable(
					_extrusion.candidatePresentation)
				&& myDoc->ShapeLabel(_extrusion.candidatePresentation)
					.IsEqual(_extrusion.label)
				&& XCAFDoc_ShapeTool::GetShape(_extrusion.label).IsEqual(
					_extrusion.candidatePresentation->Shape());
		} catch (...) {
			return Standard_False;
		}
	}

	Standard_Boolean
	ShapeInteractor::canRetryExtrusionResolution() const noexcept {
		try {
			return _extrusion.IsReady()
				&& _extrusion.ownsCommand
				&& _extrusion.commitOutcomeUnknown
				&& !_extrusion.candidatePresentation.IsNull()
				&& !_extrusion.candidatePresentation->Shape().IsNull();
		} catch (...) {
			return Standard_False;
		}
	}

	Standard_Boolean ShapeInteractor::hasActiveExtrusion() const noexcept {
		return _extrusion.IsReady();
	}

	Standard_Boolean ShapeInteractor::hasExtrusionPreview() const noexcept {
		return canApplyExtrusion();
	}

	ExtrusionPreviewState
	ShapeInteractor::extrusionPreviewState() const noexcept {
		if (!_extrusion.IsReady()) {
			return ExtrusionPreviewState::Unavailable;
		}
		if (_extrusion.rollbackFailed) {
			return ExtrusionPreviewState::Failed;
		}
		if (_extrusion.commitOutcomeUnknown) {
			return ExtrusionPreviewState::OutcomeUnknown;
		}
		if (canApplyExtrusion()) {
			return ExtrusionPreviewState::Ready;
		}
		if (!_extrusion.candidatePresentation.IsNull()
			|| _extrusion.ownsCommand) {
			return ExtrusionPreviewState::Failed;
		}
		return ExtrusionPreviewState::Selecting;
	}

	Standard_Boolean ShapeInteractor::applyExtrusion() noexcept {
		const Standard_Size resultSubshapeCount =
			_extrusion.candidateSubshapeCount;
		const Standard_Size resultSolidCount =
			_extrusion.candidateSolidCount;
		try {
			OCC_CATCH_SIGNALS
			const Handle(AIS_Shape) original =
				_extrusion.originalPresentation;
			const Handle(AIS_Shape) candidate =
				_extrusion.candidatePresentation;
            const TopoDS_Shape candidateShape = candidate.IsNull()
                ? TopoDS_Shape()
                : candidate->Shape();
			const auto finishCommitted = [&]() noexcept {
				_extrusion.ownsCommand = Standard_False;
				_extrusion.rollbackFailed = Standard_False;
				_extrusion.commitOutcomeUnknown = Standard_False;
#ifdef DEBUG
				_lastExtrusionApplySucceeded = Standard_True;
				_lastExtrusionResultSubshapeCount = resultSubshapeCount;
				_lastExtrusionResultSolidCount = resultSolidCount;
#endif
				// OCAF is authoritative after the command closes. Incremental AIS
				// publication remains best-effort; the bridge redraws from OCAF.
				try {
					myContext->ClearSelected(Standard_False);
					if (!original.IsNull()) {
						myContext->Remove(original, Standard_False);
					}
					if (!candidate.IsNull()) {
						myContext->Display(
							candidate,
							AIS_Shaded,
							_topAbsSelMode,
							Standard_False);
						setInteractiveObjectSelectionMode(candidate);
					}
					myContext->UpdateCurrentViewer();
				} catch (...) {
				}
				try {
					myDoc->NotifyChanges();
				} catch (...) {
				}
				clearExtrusionSelection();
			};

			ExtrusionDocumentState state =
				InspectExtrusionDocumentState(
					myDoc,
					_extrusion.label,
					_extrusion.originalShape,
					candidateShape);
			// CommitCommand's Boolean only reports whether an undo delta was
			// appended. A prior call may have closed successfully and then thrown
			// or reported false, so reconcile the authoritative label first.
			if (_extrusion.ownsCommand
				&& state == ExtrusionDocumentState::CandidateShape
				&& myDoc->GeometryRepresentationForLabel(
					_extrusion.label) == OcctGeometryRepresentation::BRep) {
				finishCommitted();
				return Standard_True;
			}
			if (_extrusion.commitOutcomeUnknown
				&& (state == ExtrusionDocumentState::Unavailable
					|| state == ExtrusionDocumentState::OtherClosedShape)) {
				return Standard_False;
			}
			if (!canApplyExtrusion()
				|| state != ExtrusionDocumentState::OpenCommand) {
				(void)discardExtrusionPreview(Standard_True);
				return Standard_False;
			}
			const Handle(TDocStd_Document) document =
				myDoc->ChangeDocument();
			try {
				Standard_Boolean commitReported =
					document->CommitCommand();
#ifdef DEBUG
				const Standard_Integer commitMode =
					_debugExtrusionCommitMode;
				_debugExtrusionCommitMode = 0;
				if (commitMode == 1) {
					commitReported = Standard_False;
				} else if (commitMode == 2) {
					throw Standard_Failure(
						"Injected extrusion commit exception after close");
				}
#endif
				(void)commitReported;
			} catch (...) {
				// State reconciliation below distinguishes open/rolled-back from
				// a command that closed before the exception escaped.
			}
#ifdef DEBUG
			if (_debugExtrusionPostCommitInspectFailureCount > 0) {
				--_debugExtrusionPostCommitInspectFailureCount;
				state = ExtrusionDocumentState::Unavailable;
			} else
#endif
			{
				state = InspectExtrusionDocumentState(
					myDoc,
					_extrusion.label,
					_extrusion.originalShape,
					candidateShape);
			}
			if (state == ExtrusionDocumentState::CandidateShape
				&& myDoc->GeometryRepresentationForLabel(
					_extrusion.label) == OcctGeometryRepresentation::BRep) {
				finishCommitted();
				return Standard_True;
			}
			if (state == ExtrusionDocumentState::OpenCommand
				|| state == ExtrusionDocumentState::OriginalShape) {
				(void)discardExtrusionPreview(Standard_True);
				return Standard_False;
			}
			// Unknown state must not be visually rewritten as the old shape or
			// committed a second time while closed. Keep Apply enabled solely as
			// a retryable state-inspection action.
			_extrusion.rollbackFailed = Standard_False;
			_extrusion.commitOutcomeUnknown = Standard_True;
			return Standard_False;
		} catch (...) {
			(void)discardExtrusionPreview(Standard_True);
			return Standard_False;
		}
	}

	Standard_Boolean ShapeInteractor::cancelExtrusion() noexcept {
		if (!discardExtrusionPreview(Standard_True)) {
			return Standard_False;
		}
		clearExtrusionSelection();
		return Standard_True;
	}

#ifdef DEBUG
	Standard_Boolean ShapeInteractor::debugBeginExtrusionSelection(
		const Handle(AIS_Shape)& presentation,
		const TopoDS_Face& face) noexcept {
		return beginExtrusionSelectionImpl(presentation, face);
	}

		ExtrusionDebugState ShapeInteractor::debugExtrusionState() const noexcept {
		ExtrusionDebugState state;
		try {
			OCC_CATCH_SIGNALS
			state.selectionReady = _extrusion.IsReady();
			state.previewActive = hasExtrusionPreview();
			state.canApply = canApplyExtrusion();
			state.resolutionRetryable =
				canRetryExtrusionResolution();
			const Handle(TDocStd_Document) aDocument = myDoc.IsNull()
				? Handle(TDocStd_Document)()
				: myDoc->Document();
			state.commandOpen = !aDocument.IsNull()
				&& aDocument->HasOpenCommand();
			state.lastApplySucceeded = _lastExtrusionApplySucceeded;
			state.distance = _extrusion.distance;
			state.sourceSubshapeCount = _extrusion.sourceSubshapeCount;
			state.profileEdgeCount = _extrusion.profileEdgeCount;
			state.candidateSubshapeCount = _extrusion.IsReady()
				? _extrusion.candidateSubshapeCount
				: _lastExtrusionResultSubshapeCount;
			state.candidateSolidCount = _extrusion.IsReady()
				? _extrusion.candidateSolidCount
				: _lastExtrusionResultSolidCount;
			state.lastFailureStage = _lastExtrusionFailureStage;
			state.lastFeatureStatus = _lastExtrusionFeatureStatus;
			state.lastCandidateShapeType =
				_lastExtrusionCandidateShapeType;
			state.rawDirectChildCount =
				_lastExtrusionRawDirectChildCount;
			state.rawDirectChildShapeType =
				_lastExtrusionRawDirectChildShapeType;
			state.historyHasModified =
				_lastExtrusionHistoryHasModified;
			state.historyHasGenerated =
				_lastExtrusionHistoryHasGenerated;
			state.historyFaceDeleted =
				_lastExtrusionHistoryFaceDeleted;
			state.candidateVolume = _lastExtrusionCandidateVolume;
			state.candidateMinX = _lastExtrusionCandidateBounds[0];
			state.candidateMinY = _lastExtrusionCandidateBounds[1];
			state.candidateMinZ = _lastExtrusionCandidateBounds[2];
			state.candidateMaxX = _lastExtrusionCandidateBounds[3];
			state.candidateMaxY = _lastExtrusionCandidateBounds[4];
			state.candidateMaxZ = _lastExtrusionCandidateBounds[5];
		} catch (...) {
			// DEBUG telemetry is strictly observational and must never terminate
			// a test host if OCCT reports a signal or document access failure.
		}
			return state;
		}

		Standard_Boolean ShapeInteractor::debugBeginBevelSelection(
			const Handle(AIS_Shape)& presentation,
			const std::vector<Standard_Size>& edgeTopologyIndices) noexcept {
			return debugBeginBevelSelection(
				std::vector<Handle(AIS_Shape)>{presentation},
				std::vector<std::vector<Standard_Size>>{
					edgeTopologyIndices});
		}

		Standard_Boolean ShapeInteractor::debugBeginBevelSelection(
			const std::vector<Handle(AIS_Shape)>& presentations,
			const std::vector<std::vector<Standard_Size>>&
				edgeTopologyIndices) noexcept {
			if (_bevelController == nullptr || presentations.empty()
				|| presentations.size() != edgeTopologyIndices.size()
				|| presentations.size()
					> BevelOperationController::kMaxSourceBodies) {
				return Standard_False;
			}
			try {
				OCC_CATCH_SIGNALS
				std::vector<BevelSourceSelection> aSources;
				aSources.reserve(presentations.size());
				Standard_Size anAggregateEdgeCount = 0;
				for (Standard_Size aSourceIndex = 0;
					 aSourceIndex < presentations.size(); ++aSourceIndex) {
					const Handle(AIS_Shape)& aPresentation =
						presentations[aSourceIndex];
					const std::vector<Standard_Size>& aTopologyIndices =
						edgeTopologyIndices[aSourceIndex];
					if (aPresentation.IsNull()
						|| aPresentation->Shape().IsNull()
						|| aTopologyIndices.empty()
						|| aTopologyIndices.size()
							> BevelOperationController::kMaxSelectedEdges
						|| anAggregateEdgeCount
							> BevelOperationController::kMaxSelectedEdges
								- aTopologyIndices.size()) {
						return Standard_False;
					}
					anAggregateEdgeCount += aTopologyIndices.size();
					TopTools_IndexedMapOfShape anEdges;
					TopExp::MapShapes(
						aPresentation->Shape(), TopAbs_EDGE, anEdges);
					std::vector<TopoDS_Edge> aCaptured;
					aCaptured.reserve(aTopologyIndices.size());
					std::set<Standard_Size> aUniqueIndices;
					for (const Standard_Size anIndex : aTopologyIndices) {
						if (anIndex
								>= static_cast<Standard_Size>(anEdges.Extent())
							|| !aUniqueIndices.insert(anIndex).second) {
							return Standard_False;
						}
						aCaptured.push_back(TopoDS::Edge(
							anEdges(static_cast<Standard_Integer>(anIndex + 1))));
					}
					BevelSourceSelection aSource;
					aSource.original = aPresentation;
					aSource.documentLabel = myDoc->ShapeLabel(aPresentation);
					aSource.edges = std::move(aCaptured);
					aSource.edgeTopologyIndices = aTopologyIndices;
					aSource.selectionMode =
						AIS_Shape::SelectionMode(_topAbsSelMode);
					aSources.push_back(std::move(aSource));
				}
				return _bevelController->begin(aSources);
			} catch (...) {
				(void)_bevelController->cancel();
				return Standard_False;
			}
		}

		BevelPreviewDebugState ShapeInteractor::debugBevelState() const noexcept {
			return _bevelController == nullptr
				? BevelPreviewDebugState()
				: _bevelController->debugPreviewState();
		}

		void ShapeInteractor::debugSetBevelWorkerBlocked(
			const Standard_Boolean blocked) noexcept {
			if (_bevelController != nullptr) {
				_bevelController->debugSetWorkerBlocked(blocked);
			}
		}

		void ShapeInteractor::debugSetMaximumBevelCaptureTopologyNodes(
			const Standard_Size limit) noexcept {
			if (_bevelController != nullptr) {
				_bevelController->debugSetMaximumCaptureTopologyNodes(limit);
			}
		}

		void ShapeInteractor::debugSetMaximumBevelResultTopologyNodes(
			const Standard_Size limit) noexcept {
			if (_bevelController != nullptr) {
				_bevelController->debugSetMaximumResultTopologyNodes(limit);
			}
		}

		void ShapeInteractor::debugSetMaximumBevelResultSolids(
			const Standard_Size limit) noexcept {
			if (_bevelController != nullptr) {
				_bevelController->debugSetMaximumResultSolids(limit);
			}
		}

		void ShapeInteractor::debugSetBevelTransactionFailureCount(
			const Standard_Size count) noexcept {
			if (_bevelController != nullptr) {
				_bevelController->debugSetTransactionFailureCount(count);
			}
		}

		void ShapeInteractor::debugSetBevelCancelDiscardFailureCount(
			const Standard_Size count) noexcept {
			if (_bevelController != nullptr) {
				_bevelController->debugSetCancelDiscardFailureCount(count);
			}
		}

		Standard_Boolean
		ShapeInteractor::debugMutateFirstBevelSourcePersistedTransform() noexcept {
			return _bevelController != nullptr
				&& _bevelController
					->debugMutateFirstSourcePersistedTransform();
		}
#endif

	Standard_Boolean ShapeInteractor::beginBevelSelectionFromDetectedEdges() noexcept {
		if (_bevelController == nullptr || _detectedEdges.empty()) {
			return Standard_False;
		}
		try {
			std::vector<BevelSourceSelection> aSelection;
			aSelection.reserve(_detectedEdges.size());
			for (const EdgesSelection& anEdges : _detectedEdges) {
				if (anEdges.detectedOwner.IsNull()
					|| !anEdges.detectedOwner->HasSelectable()
					|| anEdges.edges.empty()) {
					return Standard_False;
				}
				const Handle(AIS_Shape) anOriginal =
					Handle(AIS_Shape)::DownCast(
						anEdges.detectedOwner->Selectable());
				if (anOriginal.IsNull()) {
					return Standard_False;
				}
				BevelSourceSelection aSource;
				aSource.original = anOriginal;
				aSource.documentLabel = anEdges.documentLabel;
				aSource.edges = anEdges.edges;
				aSource.selectionMode =
					AIS_Shape::SelectionMode(_topAbsSelMode);
				aSelection.push_back(std::move(aSource));
			}
			return _bevelController->begin(aSelection);
		} catch (...) {
			return Standard_False;
		}
	}

	Standard_Boolean ShapeInteractor::setChamferValueForSelection(
		const Standard_Real value) {
		return _bevelController != nullptr
			&& _bevelController->setValue(value / 100.0);
	}

	BevelApplyResult ShapeInteractor::applyBevel() noexcept {
		const BevelApplyResult aResult = _bevelController == nullptr
			? BevelApplyResult::NoChange
			: _bevelController->apply();
		if (aResult != BevelApplyResult::NoChange) {
			_detectedEdges.clear();
		}
		return aResult;
	}

	Standard_Boolean ShapeInteractor::canApplyBevel() const noexcept {
		return _bevelController != nullptr && _bevelController->canApply();
	}

	Standard_Boolean ShapeInteractor::hasActiveBevel() const noexcept {
		return _bevelController != nullptr
			&& _bevelController->hasActiveOperation();
	}

	Standard_Boolean ShapeInteractor::isBevelSelectionFrozen() const noexcept {
		return _bevelController != nullptr
			&& _bevelController->isSelectionFrozen();
	}

	BevelPreviewState ShapeInteractor::bevelPreviewState() const noexcept {
		return _bevelController == nullptr
			? BevelPreviewState::Selecting
			: _bevelController->previewState();
	}

	std::uint64_t ShapeInteractor::bevelPreviewGeneration() const noexcept {
		return _bevelController == nullptr
			? 0
			: _bevelController->previewGeneration();
	}

	Standard_Boolean ShapeInteractor::captureBevelPreview(
		BevelPreviewCapture& capture) const noexcept {
		return _bevelController != nullptr
			&& _bevelController->capturePreview(capture);
	}

	void ShapeInteractor::setBevelPreviewStateChangedCallback(
		std::function<void()> callback) {
		if (_bevelController != nullptr) {
			_bevelController->setPreviewStateChangedCallback(
				std::move(callback));
		}
	}

	Standard_Boolean ShapeInteractor::cancelChamfer() noexcept {
		const Standard_Boolean didCancel = _bevelController == nullptr
			|| _bevelController->cancel();
		if (didCancel) {
			_detectedEdges.clear();
		}
		return didCancel;
	}

	Standard_Boolean ShapeInteractor::resetWireframeTemplateShape() noexcept {
		// Legacy call sites use this when deselecting or changing tools. Those
		// lifecycle edges are Cancel, never an implicit geometry commit.
		return cancelChamfer();
	}

    Standard_Size ShapeInteractor::saveSelectionEdges(bool preventRechamfer) {
	// checkFilleted:
	//      - false - enable the chamfer functionality anyway
	//      - true - disable the chamfer functionality if it has already been used
        
		if (!resetWireframeTemplateShape()) {
			return 0;
		}

        for (myContext->InitSelected(); myContext->MoreSelected(); myContext->NextSelected()) {
            const Handle(SelectMgr_EntityOwner) detectedOwner = myContext->SelectedOwner();
			if (detectedOwner.IsNull() || !detectedOwner->HasSelectable()) { continue; }
            Handle(AIS_Shape) ownerShape = Handle(AIS_Shape)::DownCast(detectedOwner->Selectable());
			if (ownerShape.IsNull() || ownerShape->Shape().IsNull()) { continue; }
            const Handle(StdSelect_BRepOwner) &aBRepOwnerOfSelection = Handle(StdSelect_BRepOwner)::DownCast(detectedOwner);
			if (aBRepOwnerOfSelection.IsNull()) { continue; }

			Standard_Integer found = -1;
			for (Standard_Size index = 0; index < _detectedEdges.size(); ++index) {
				if (_detectedEdges[index].detectedOwner.IsNull()
					|| !_detectedEdges[index].detectedOwner->HasSelectable()) { continue; }
				Handle(AIS_Shape) existing = Handle(AIS_Shape)::DownCast(
					_detectedEdges[index].detectedOwner->Selectable());
				if (!existing.IsNull() && existing->Shape().IsSame(ownerShape->Shape())) {
					found = static_cast<Standard_Integer>(index);
					break;
				}
			}
			const Standard_Boolean isNewSelection = found < 0;
			if (isNewSelection) {
				EdgesSelection sel;
				sel.documentLabel = myDoc->ShapeLabel(ownerShape);
				if (sel.documentLabel.IsNull()
					|| !IsBRepModelingLabel(
						myDoc, sel.documentLabel)) {
					continue;
				}
				sel.transform = ownerShape->LocalTransformation();
				sel.materialName = myDoc->MaterialNameForLabel(sel.documentLabel);
				sel.colorName = myDoc->ColorNameForLabel(sel.documentLabel);
				sel.detectedOwner = detectedOwner;
				_detectedEdges.push_back(sel);
				found = static_cast<Standard_Integer>(_detectedEdges.size() - 1);
			}

			EdgesSelection& selection = _detectedEdges[found];
			const Standard_Size originalEdgeCount = selection.edges.size();
            auto selectedType = aBRepOwnerOfSelection->Shape().ShapeType();
            if (aBRepOwnerOfSelection->IsSelected() && (selectedType == _topAbsSelMode || (selectedType == TopAbs_SOLID && _topAbsSelMode == TopAbs_SHAPE))) {
				auto addEdge = [&](const TopoDS_Edge& edge) {
					Standard_Real first = 0;
					Standard_Real last = 0;
					Handle_Geom_Curve curve = BRep_Tool::Curve(edge, first, last);
					if (curve.IsNull()) { return; }
					GeomAdaptor_Curve adaptor(curve);
					const GeomAbs_CurveType curveType = adaptor.GetType();
					const Standard_Boolean isChamferable = curveType == GeomAbs_Line
						|| curveType == GeomAbs_BSplineCurve || adaptor.IsClosed();
					if (!isChamferable && preventRechamfer) { return; }
					for (const TopoDS_Edge& existing : selection.edges) {
						if (existing.IsSame(edge)) { return; }
					}
					selection.edges.push_back(edge);
				};
				const TopoDS_Shape selectedShape = aBRepOwnerOfSelection->Shape();
				if (selectedShape.ShapeType() == TopAbs_EDGE) {
					addEdge(TopoDS::Edge(selectedShape));
				} else {
					for (TopExp_Explorer exp(selectedShape, TopAbs_EDGE); exp.More(); exp.Next()) {
						addEdge(TopoDS::Edge(exp.Current()));
					}
				}
            }
			if (selection.edges.size() > 64) {
				selection.edges.resize(originalEdgeCount);
			}
			if (isNewSelection && selection.edges.empty()) {
				_detectedEdges.erase(_detectedEdges.begin() + found);
			}
        }
		if (_detectedEdges.empty()
			|| !beginBevelSelectionFromDetectedEdges()) {
			_detectedEdges.clear();
			return 0;
		}
	        return _detectedEdges.size();
	    }

    const size_t ShapeInteractor::getNumberOfDetectedEdges() const {
        return _detectedEdges.size();
    }

    void ShapeInteractor::setSelectionMode(ShapeSelectionMode mode) {
        switch (mode) {
            case ShapeSelectionMode::Edge:
            case ShapeSelectionMode::Vertex:
                myContext->SetPixelTolerance(32);
                break;
            default:
                myContext->SetPixelTolerance();
                break;
        }

		if (_previousSelectionMode != mode) {
			if (!resetWireframeTemplateShape()) {
				return;
			}
			myContext->ClearSelected(Standard_True);
		}
        else {
            return;
        }

		TopAbs_ShapeEnum selMode;
		switch (mode) {
			case ShapeSelectionMode::WholeShape:
				selMode = TopAbs_ShapeEnum::TopAbs_SHAPE;
				break;
			case ShapeSelectionMode::Face:
				selMode = TopAbs_ShapeEnum::TopAbs_FACE;
				break;
			case ShapeSelectionMode::Edge:
				selMode = TopAbs_ShapeEnum::TopAbs_EDGE;
				break;
			case ShapeSelectionMode::Vertex:
				selMode = TopAbs_ShapeEnum::TopAbs_VERTEX;
				break;
			default:
				selMode = TopAbs_ShapeEnum::TopAbs_SHAPE;
				break;
		}

		AIS_ListOfInteractive objects;
		myContext->DisplayedObjects(objects);
		AIS_ListIteratorOfListOfInteractive iobject(objects);
		_previousSelectionMode = mode;
		_topAbsSelMode = selMode;

		while (iobject.More()) {
			setInteractiveObjectSelectionMode(iobject.Value());
			iobject.Next();
		}

		std::cout << "set selection mode=" << selMode << std::endl;
    }

	void ShapeInteractor::setInteractiveObjectSelectionMode(const Handle(AIS_InteractiveObject) aio) {
		if (aio.IsNull() || !myDoc->IsPresentationEditable(aio)) {
			if (!aio.IsNull()) {
				myContext->Deactivate(aio);
			}
			return;
		}
		const TDF_Label aLabel = myDoc->ShapeLabel(aio);
		const OcctGeometryRepresentation aRepresentation =
			myDoc->GeometryRepresentationForLabel(aLabel);
		if (aRepresentation == OcctGeometryRepresentation::Invalid) {
			myContext->Deactivate(aio);
			return;
		}
		if (aRepresentation == OcctGeometryRepresentation::TriangleMesh) {
			// Triangle-only geometry has no editable BRep topology. Preserve the
			// global mode for BRep objects in mixed documents, but expose this
			// presentation only as one object-level selection target.
			myContext->Deactivate(aio);
			myContext->Activate(
				aio,
				AIS_Shape::SelectionMode(TopAbs_SHAPE),
				Standard_True);
			return;
		}
		myContext->Deactivate(aio, aio->GlobalSelectionMode());//(Standard_Integer)_previousSelectionMode);
		myContext->Activate(aio, (Standard_Integer)_previousSelectionMode,  Standard_True);
		myContext->SetSelectionModeActive (aio, AIS_Shape::SelectionMode (_topAbsSelMode), true, AIS_SelectionModesConcurrency::AIS_SelectionModesConcurrency_Single);
	}

    const ShapeSelectionMode ShapeInteractor::getSelectionMode() const {
        return _previousSelectionMode;
    }

    const Standard_Boolean ShapeInteractor::isEmptyOfDisplayedObjects() const {
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(objects);
        return objects.IsEmpty();
    }

    const Standard_Size ShapeInteractor::getNumberOfDisplayedShapes() const {
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        return objects.Size();
    }

    void ShapeInteractor::exportShapes() {
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        AIS_ListIteratorOfListOfInteractive iobject(objects);

        while (iobject.More()) {
            printf(">>> Export shape\n");

            std::vector<TopoDS_Face>   faces;
            std::vector<TopoDS_Edge>   edges;
            std::vector<TopoDS_Vertex> vertices;
            TopoDS_Shape myShape = Handle(AIS_Shape)::DownCast(iobject.Value())->Shape();
            extractGeometryShapes(myShape, faces, edges, vertices);

            iobject.Next();
        }
    }

    void ShapeInteractor::extractGeometryShapes(const TopoDS_Shape &shape,
                                                std::vector<TopoDS_Face> &faces,
                                                std::vector<TopoDS_Edge> &edges,
                                                std::vector<TopoDS_Vertex> &vertices) {
        faces.resize(0);
        edges.resize(0);
        vertices.resize(0);

        TopExp_Explorer exp;
        for (exp.Init(shape, TopAbs_FACE); exp.More(); exp.Next()) {
            faces.push_back(TopoDS::Face(exp.Current()));
        }
        for (exp.Init(shape, TopAbs_EDGE); exp.More(); exp.Next()) {
            edges.push_back(TopoDS::Edge(exp.Current()));
        }
        for (exp.Init(shape, TopAbs_VERTEX); exp.More(); exp.Next()) {
            vertices.push_back(TopoDS::Vertex(exp.Current()));
            gp_Pnt pnt = BRep_Tool::Pnt(TopoDS::Vertex(exp.Current()));
            printf(">>> Vertex: %f, %f, %f\n", pnt.X(), pnt.Y(), pnt.Z());
        }
    }

    void ShapeInteractor::exportToStl(const std::string &filename, const Standard_Boolean isASCII/* = Standard_True*/) {
        if (!CanExportCommittedDocument(
					myDoc,
					filename,
					OcctGeometryExportFormat::Stl,
					hasActiveBevel() || hasActiveExtrusion())) {
            return;
        }
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        AIS_ListIteratorOfListOfInteractive iobject(objects);

        TopoDS_Compound resultShape;
        BRep_Builder builder;
        builder.MakeCompound(resultShape);

        while (iobject.More()) {
            TopoDS_Shape shape = BRepBuilderAPI_Transform(Handle(AIS_Shape)::DownCast(iobject.Value())->Shape(),
                                                          iobject.Value()->LocalTransformation());
            builder.Add(resultShape, shape);
            iobject.Next();
        }

        StlAPI_Writer aStlWriter;
        aStlWriter.ASCIIMode() = isASCII;
        if (!aStlWriter.Write(resultShape, filename.c_str())) {
            printf("Error creating STL file: %s\n", filename.c_str());
        }
    }

    void ShapeInteractor::exportToObj(const std::string &filename) {
        if (!CanExportCommittedDocument(
					myDoc,
					filename,
					OcctGeometryExportFormat::Obj,
					hasActiveBevel() || hasActiveExtrusion())) {
            return;
        }
//#define converter2obj
#ifdef converter2obj
        const auto tmp_stl = filename + ".tmp";
        exportToStl(tmp_stl, Standard_False);

        //  create a geometry tesselation object
        Geometry tessel;
        //  fill up the tesselation object with STL data (load STL)
        tessel.visit (ImportSTL (tmp_stl));
        //  write down the tesselation object into OBJ file (save OBJ)
        tessel.visit (ExportOBJ (filename));
        printf("OBJ file: %s\n", filename.c_str());
#else
        // OBJ export writes the mesh stored in the XCAF document. RWObj_CafWriter does
        // NOT tessellate — its header states "Triangulation data should be precomputed
        // within shapes!" — so any untriangulated face makes it raise a Standard_Failure.
        // Shapes authored or edited in-session are only tessellated for on-screen display,
        // never in the document, and because the write below used to be unguarded that
        // exception unwound across the Obj-C++/Swift boundary into std::terminate, instantly
        // quitting the app. (STL is unaffected: it reads the already-tessellated display
        // shapes and StlAPI_Writer meshes internally.) Mirror the model-load / glTF paths:
        // triangulate the free shapes first, and guard the whole operation so a failure
        // yields an empty file (handled as nil upstream) instead of crashing.
        myDoc->ChangeDocument()->NewCommand();
        bool exportSucceeded = false;

        try {
            myDoc->ApplyTransforms();

            Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool(myDoc->Document()->Main());
            TDF_LabelSequence labels;
            shapeTool->GetFreeShapes(labels);

            if (labels.IsEmpty()) {
                // Nothing to export. RWObj_CafWriter creates the output file at
                // construction, so an empty document would otherwise leave a
                // header-only .obj that passes the non-empty guard upstream. Bail
                // out before the writer is constructed so no file is produced.
                printf("Error creating OBJ file (no shapes to export): %s\n", filename.c_str());
            } else {
                TopoDS_Compound compound;
                BRep_Builder builder;
                builder.MakeCompound(compound);
                for (Standard_Integer i = 1; i <= labels.Length(); ++i) {
                    TopoDS_Shape shape = shapeTool->GetShape(labels.Value(i));
                    if (!shape.IsNull()) {
                        builder.Add(compound, shape);
                    }
                }

                // Use the display-relative deflection (as the load path does) instead of a
                // fixed absolute value: on a large, mm-scale scene an absolute deflection
                // explodes the triangle count and can OOM the export on iPad.
                Handle(Prs3d_Drawer) drawer = myContext->DefaultDrawer();
                Standard_Real deflection = StdPrs_ToolTriangulatedShape::GetDeflection(compound, drawer);
                if (!BRepTools::Triangulation(compound, deflection)) {
                    BRepMesh_IncrementalMesh mesher;
                    mesher.ChangeParameters().Deflection = deflection;
                    mesher.ChangeParameters().Angle = drawer->DeviationAngle();
                    mesher.ChangeParameters().InParallel = Standard_True;
                    mesher.SetShape(compound);
                    mesher.Perform();
                }

                TColStd_IndexedDataMapOfStringString aFileInfo;
                aFileInfo.Add("Author", "Shapeyard 3D");

                auto writer = RWObj_CafWriter(TCollection_AsciiString(filename.c_str()));
                exportSucceeded = writer.Perform(myDoc->Document(), aFileInfo, Message_ProgressRange());
                if (!exportSucceeded) {
                    printf("Error creating OBJ file (writer reported failure): %s\n", filename.c_str());
                }
            }
        } catch (const Standard_Failure &theFailure) {
            printf("Error creating OBJ file (exception): %s [%s]\n", filename.c_str(), theFailure.GetMessageString());
        } catch (...) {
            printf("Error creating OBJ file (exception): %s\n", filename.c_str());
        }

        // Always roll back the transient NewCommand()/ApplyTransforms() so the live
        // document is left untouched for continued editing — even if the export above
        // threw. Leaving the command open would let a later NewCommand() commit stale
        // (double-transformed) geometry.
        AbortOpenCommandNoThrow(myDoc->ChangeDocument());

        // A failed or aborted write can leave a partial (non-empty) file behind, which
        // would pass the size check upstream and share a corrupt OBJ — remove it so
        // failure reliably surfaces as nil (no share sheet) instead.
        if (!exportSucceeded) {
            std::remove(filename.c_str());
        }
#endif
    }

    void ShapeInteractor::exportToGltf(const std::string &filename) {
        if (!CanExportCommittedDocument(
					myDoc,
					filename,
					OcctGeometryExportFormat::Gltf,
					hasActiveBevel() || hasActiveExtrusion())) {
            return;
        }
        bool exportSucceeded = false;
        try {
            // Bake presentation transforms only inside a transient command;
            // every exit below aborts it so export cannot mutate the editor.
            myDoc->ChangeDocument()->NewCommand();
            myDoc->ApplyTransforms();

            Handle(XCAFDoc_ShapeTool) shapeTool =
                XCAFDoc_DocumentTool::ShapeTool(
                    myDoc->Document()->Main());
            TDF_LabelSequence labels;
            if (!shapeTool.IsNull()) {
                shapeTool->GetFreeShapes(labels);
            }
            if (!shapeTool.IsNull() && !labels.IsEmpty()) {
                TopoDS_Compound compound;
                BRep_Builder builder;
                builder.MakeCompound(compound);
                for (Standard_Integer i = 1; i <= labels.Length(); ++i) {
                    const TopoDS_Shape shape =
                        shapeTool->GetShape(labels.Value(i));
                    if (!shape.IsNull()) {
                        builder.Add(compound, shape);
                    }
                }
                Handle(Prs3d_Drawer) drawer = myContext->DefaultDrawer();
                const Standard_Real deflection =
                    StdPrs_ToolTriangulatedShape::GetDeflection(
                        compound, drawer);
                if (!BRepTools::Triangulation(compound, deflection)) {
                    BRepMesh_IncrementalMesh mesher;
                    mesher.ChangeParameters().Deflection = deflection;
                    mesher.ChangeParameters().Angle = drawer->DeviationAngle();
                    mesher.ChangeParameters().InParallel = Standard_True;
                    mesher.SetShape(compound);
                    mesher.Perform();
                }

                TColStd_IndexedDataMapOfStringString fileInfo;
                fileInfo.Add("Author", "Shapeyard 3D");
                RWGltf_CafWriter writer(
                    TCollection_AsciiString(filename.c_str()),
                    Standard_True);
                exportSucceeded = writer.Perform(
                    myDoc->Document(),
                    fileInfo,
                    Message_ProgressRange());
            }
        } catch (const Standard_Failure& failure) {
            printf("Error creating glTF file (exception): %s [%s]\n",
                   filename.c_str(), failure.GetMessageString());
        } catch (...) {
            printf("Error creating glTF file (exception): %s\n", filename.c_str());
        }

        AbortOpenCommandNoThrow(myDoc->ChangeDocument());
        if (!exportSucceeded) {
            std::remove(filename.c_str());
        }
    }

    void ShapeInteractor::exportToStep(const std::string &filename) {
        if (!CanExportCommittedDocument(
					myDoc,
					filename,
					OcctGeometryExportFormat::Step,
					hasActiveBevel() || hasActiveExtrusion())) {
            return;
        }
        AIS_ListOfInteractive objects;
        myContext->DisplayedObjects(AIS_KOI_Shape, -1, objects);
        AIS_ListIteratorOfListOfInteractive iobject(objects);

        TopoDS_Compound resultShape;
        BRep_Builder builder;
        builder.MakeCompound(resultShape);

        while (iobject.More()) {
            TopoDS_Shape shape = BRepBuilderAPI_Transform(
                Handle(AIS_Shape)::DownCast(iobject.Value())->Shape(),
                iobject.Value()->LocalTransformation());
            builder.Add(resultShape, shape);
            iobject.Next();
        }

        {
            std::lock_guard<std::mutex> stepExchangeLock(
                Core3DSTEPExchangeMutex());
            Interface_Static::SetCVal("write.step.schema", "AP214IS");
            STEPControl_Writer stepWriter;
            stepWriter.Transfer(resultShape, STEPControl_AsIs);
            if (stepWriter.Write(filename.c_str()) != IFSelect_RetDone) {
                printf("Error creating STEP file: %s\n", filename.c_str());
            }
        }
    }
}
