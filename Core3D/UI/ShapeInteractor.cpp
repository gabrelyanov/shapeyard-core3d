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
#include <BRepTools.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Prs3d_Drawer.hxx>
#include <StdPrs_ToolTriangulatedShape.hxx>
#include <TColStd_ListIteratorOfListOfInteger.hxx>
#include <TColStd_ListOfInteger.hxx>
#include <Standard_Failure.hxx>
#include <Standard_ErrorHandler.hxx>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <set>
#include <unordered_set>
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
		constexpr Standard_Integer kAdaptivePixelTolerance = -1;
		constexpr Standard_Integer kExpandedTopologyPixelTolerance = 32;

		//! Consume the Bevel admission budget by occurrence, not unique TShape.
		//! This must run before any source-wide map, curve inspection, or BRep
		//! validation so shared/adversarial topology cannot amplify UI-thread work.
		Standard_Boolean ConsumeBevelSourceTopologyBudget(
			const TopoDS_Shape& shape,
			const Standard_Size perSourceLimit,
			const Standard_Size aggregateLimit,
			Standard_Size& aggregateCount,
			Standard_Size& sourceCount) noexcept {
			sourceCount = 0;
			if (shape.IsNull() || perSourceLimit == 0
				|| aggregateLimit == 0
				|| aggregateCount > aggregateLimit) {
				return Standard_False;
			}
			try {
				OCC_CATCH_SIGNALS
				std::vector<TopoDS_Shape> pending{shape};
				while (!pending.empty()) {
					const TopoDS_Shape current = pending.back();
					pending.pop_back();
					if (current.IsNull()
						|| ++sourceCount
							> perSourceLimit
						|| ++aggregateCount
							> aggregateLimit) {
						sourceCount = 0;
						return Standard_False;
					}
					for (TopoDS_Iterator child(
							 current, Standard_True, Standard_True);
						 child.More(); child.Next()) {
						const Standard_Size sourceRemaining =
							perSourceLimit - sourceCount;
						const Standard_Size aggregateRemaining =
							aggregateLimit - aggregateCount;
						if (pending.size() + 1U
								> static_cast<std::size_t>(sourceRemaining)
							|| pending.size() + 1U
								> static_cast<std::size_t>(aggregateRemaining)) {
							sourceCount = 0;
							return Standard_False;
						}
						pending.push_back(child.Value());
					}
				}
				return sourceCount > 0;
			} catch (...) {
				sourceCount = 0;
				return Standard_False;
			}
		}

		Standard_Boolean UsesExpandedTopologyPixelTolerance(
			const ShapeSelectionMode mode) noexcept {
			return mode == ShapeSelectionMode::Edge
				|| mode == ShapeSelectionMode::Vertex;
		}

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
	          context, doc)),
	      _shellController(std::make_shared<ShellOperationController>(
	          context, doc)) {
	    }

	ShapeInteractor::~ShapeInteractor() noexcept {
		if (!cancelShell()) {
			(void)cancelShell();
		}
		if (!cancelExtrusion()) {
			(void)cancelExtrusion();
		}
		try {
			cancelChamfer();
		} catch (...) {
		}
	}

	Standard_Boolean
	ShapeInteractor::tryCaptureExactlyOneSelectedPlanarFace(
		FaceOperationSourceProof& theProof) const noexcept {
		theProof = FaceOperationSourceProof();
		if (myContext.IsNull()
			|| getSelectionMode() != ShapeSelectionMode::Face
			|| _topAbsSelMode != TopAbs_FACE
			|| !selectionModeAuthorityIsExact()) {
			return Standard_False;
		}
		try {
			OCC_CATCH_SIGNALS
			Standard_Size aRawSelectedOwnerCount = 0;
			Handle(AIS_Shape) aPresentation;
			TopoDS_Face aFace;
			for (myContext->InitSelected(); myContext->MoreSelected();
				 myContext->NextSelected()) {
				if (++aRawSelectedOwnerCount != 1) {
					return Standard_False;
				}
				const Handle(SelectMgr_EntityOwner) anOwner =
					myContext->SelectedOwner();
				const Handle(AIS_InteractiveObject) aSelectedInteractive =
					myContext->SelectedInteractive();
				const Handle(StdSelect_BRepOwner) aBRepOwner =
					Handle(StdSelect_BRepOwner)::DownCast(anOwner);
				aPresentation =
					Handle(AIS_Shape)::DownCast(aSelectedInteractive);
				if (anOwner.IsNull() || !anOwner->HasSelectable()
					|| aBRepOwner.IsNull() || !aBRepOwner->IsSelected()
					|| !aBRepOwner->HasShape()
					|| aBRepOwner->Shape().ShapeType() != TopAbs_FACE
					|| aSelectedInteractive.IsNull()
					|| aPresentation.IsNull()
					|| aPresentation->Shape().IsNull()
					|| anOwner->Selectable() != aSelectedInteractive
					|| !myContext->IsDisplayed(aPresentation)) {
					return Standard_False;
				}
				aFace = TopoDS::Face(aBRepOwner->Shape());
			}
			if (aRawSelectedOwnerCount != 1 || aPresentation.IsNull()
				|| aFace.IsNull()) {
				return Standard_False;
			}

			return TryPrepareFaceOperationSource(
				myContext,
				myDoc,
				aPresentation,
				aFace,
				ShellOperationController::kMaximumSourceTopologyNodes,
				ShellOperationController::
					kMaximumStyledSubshapeLabels,
				theProof);
		} catch (...) {
			theProof = FaceOperationSourceProof();
			return Standard_False;
		}
	}

	Standard_Boolean ShapeInteractor::canBeginShellSelection() const noexcept {
		if (_shellController == nullptr
			|| _shellController->hasActiveOperation()) {
			return Standard_False;
		}
		FaceOperationSourceProof aProof;
		if (!tryCaptureExactlyOneSelectedPlanarFace(aProof)) {
			return Standard_False;
		}
		try {
			OCC_CATCH_SIGNALS
			ShellSourceSelection aSelection;
			aSelection.proof = aProof;
			aSelection.selectionMode =
				AIS_Shape::SelectionMode(_topAbsSelMode);
			return _shellController->canBegin(aSelection);
		} catch (...) {
			return Standard_False;
		}
	}

	Standard_Boolean ShapeInteractor::queryFaceOperationAdmission(
		Standard_Boolean& theCanBeginExtrusion,
		Standard_Boolean& theCanBeginShell) const noexcept {
		theCanBeginExtrusion = Standard_False;
		theCanBeginShell = Standard_False;
		FaceOperationSourceProof aProof;
		if (!tryCaptureExactlyOneSelectedPlanarFace(aProof)) {
			return Standard_False;
		}
		try {
			OCC_CATCH_SIGNALS
			if (!hasActiveExtrusion()) {
				TDF_Label aLabel;
				Standard_Size aSourceSubshapeCount = 0;
				Standard_Size aProfileEdgeCount = 0;
				theCanBeginExtrusion = tryPrepareExtrusionSelection(
					aProof,
					aLabel,
					aSourceSubshapeCount,
					aProfileEdgeCount);
			}
			if (_shellController != nullptr
				&& !_shellController->hasActiveOperation()) {
				ShellSourceSelection aSelection;
				aSelection.proof = aProof;
				aSelection.selectionMode =
					AIS_Shape::SelectionMode(_topAbsSelMode);
				theCanBeginShell =
					_shellController->canBegin(aSelection);
			}
			return Standard_True;
		} catch (...) {
			theCanBeginExtrusion = Standard_False;
			theCanBeginShell = Standard_False;
			return Standard_False;
		}
	}

	Standard_Boolean ShapeInteractor::beginShellSelection() noexcept {
		if (myContext.IsNull() || _shellController == nullptr) {
			return Standard_False;
		}
		try {
			OCC_CATCH_SIGNALS
			FaceOperationSourceProof proof;
			if (!tryCaptureExactlyOneSelectedPlanarFace(proof)) {
				(void)cancelShell();
				return Standard_False;
			}
			return beginShellSelectionImpl(proof);
		} catch (...) {
			(void)cancelShell();
			return Standard_False;
		}
	}

	Standard_Boolean ShapeInteractor::beginShellSelectionImpl(
		const FaceOperationSourceProof& proof) noexcept {
		if (_shellController == nullptr || myDoc.IsNull()
			|| proof.original.IsNull() || proof.shape.IsNull()
			|| proof.openingFace.IsNull()) {
			return Standard_False;
		}
		try {
			OCC_CATCH_SIGNALS
			ShellSourceSelection selection;
			selection.proof = proof;
			selection.selectionMode =
				AIS_Shape::SelectionMode(_topAbsSelMode);
			return _shellController->begin(selection);
		} catch (...) {
			return Standard_False;
		}
	}

	Standard_Boolean ShapeInteractor::setShellThickness(
		const Standard_Real thickness) noexcept {
		return _shellController != nullptr
			&& _shellController->setThickness(thickness);
	}

	ShellApplyResult ShapeInteractor::applyShell() noexcept {
		return _shellController == nullptr
			? ShellApplyResult::NoChange
			: _shellController->apply();
	}

	Standard_Boolean ShapeInteractor::cancelShell() noexcept {
		return _shellController == nullptr || _shellController->cancel();
	}

	Standard_Boolean ShapeInteractor::canApplyShell() const noexcept {
		return _shellController != nullptr && _shellController->canApply();
	}

	Standard_Boolean ShapeInteractor::hasActiveShell() const noexcept {
		return _shellController != nullptr
			&& _shellController->hasActiveOperation();
	}

	Standard_Boolean ShapeInteractor::hasUnresolvedShell() const noexcept {
		return _shellController != nullptr
			&& _shellController->hasUnresolvedState();
	}

	Standard_Boolean ShapeInteractor::isShellSelectionFrozen() const noexcept {
		return _shellController != nullptr
			&& _shellController->isSelectionFrozen();
	}

	ShellPreviewState ShapeInteractor::shellPreviewState() const noexcept {
		return _shellController == nullptr
			? ShellPreviewState::Unavailable
			: _shellController->previewState();
	}

	std::uint64_t ShapeInteractor::shellPreviewGeneration() const noexcept {
		return _shellController == nullptr
			? 0
			: _shellController->previewGeneration();
	}

	Standard_Real ShapeInteractor::shellThickness() const noexcept {
		return _shellController == nullptr
			? 0.0
			: _shellController->thickness();
	}

	Standard_Real ShapeInteractor::shellDefaultThickness() const noexcept {
		return _shellController == nullptr
			? 0.0
			: _shellController->defaultThickness();
	}

	Standard_Real ShapeInteractor::shellMetersPerUnit() const noexcept {
		return _shellController == nullptr
			? 0.0
			: _shellController->metersPerUnit();
	}

	std::pair<Standard_Real, Standard_Real>
	ShapeInteractor::shellThicknessRange() const noexcept {
		return _shellController == nullptr
			? std::make_pair(0.0, 0.0)
			: _shellController->thicknessRange();
	}

	Standard_Boolean ShapeInteractor::captureShellPreview(
		ShellPreviewCapture& capture) const noexcept {
		return _shellController != nullptr
			&& _shellController->capturePreview(capture);
	}

	void ShapeInteractor::setShellPreviewStateChangedCallback(
		std::function<void()> callback) {
		if (_shellController != nullptr) {
			_shellController->setPreviewStateChangedCallback(
				std::move(callback));
		}
	}

	Standard_Boolean
	ShapeInteractor::canBeginExtrusionSelection() const noexcept {
		if (hasActiveExtrusion()) {
			return Standard_False;
		}
		FaceOperationSourceProof aProof;
		if (!tryCaptureExactlyOneSelectedPlanarFace(aProof)) {
			return Standard_False;
		}
		TDF_Label aLabel;
		Standard_Size aSourceSubshapeCount = 0;
		Standard_Size aProfileEdgeCount = 0;
		return tryPrepareExtrusionSelection(
			aProof,
			aLabel,
			aSourceSubshapeCount,
			aProfileEdgeCount);
	}

	Standard_Boolean ShapeInteractor::beginExtrusionSelection() noexcept {
		if (myContext.IsNull()) {
			return Standard_False;
		}
		try {
			OCC_CATCH_SIGNALS
			FaceOperationSourceProof proof;
			if (!tryCaptureExactlyOneSelectedPlanarFace(proof)) {
				cancelExtrusion();
				return Standard_False;
			}
			return beginExtrusionSelectionImpl(proof);
		} catch (...) {
			cancelExtrusion();
			return Standard_False;
		}
	}

	Standard_Boolean ShapeInteractor::tryPrepareExtrusionSelection(
		const FaceOperationSourceProof& theProof,
		TDF_Label& theLabel,
		Standard_Size& theSourceSubshapeCount,
		Standard_Size& theProfileEdgeCount) const noexcept {
		theLabel = TDF_Label();
		theSourceSubshapeCount = 0;
		theProfileEdgeCount = 0;
		try {
			OCC_CATCH_SIGNALS
			if (!FaceOperationSourceProofIsCurrent(
					myContext,
					myDoc,
					theProof,
					ShellOperationController::
						kMaximumStyledSubshapeLabels)
				|| theProof.topologyNodeCount
					> ShellOperationController::
						kMaximumSourceTopologyNodes) {
				return Standard_False;
			}
			Standard_Size aProfileEdgeCount = 0;
			if (!CountBoundedProfileEdges(
					theProof.openingFace, aProfileEdgeCount)) {
				return Standard_False;
			}
			theLabel = theProof.documentLabel;
			theSourceSubshapeCount = theProof.topologyNodeCount;
			theProfileEdgeCount = aProfileEdgeCount;
			return Standard_True;
		} catch (...) {
			theLabel = TDF_Label();
			theSourceSubshapeCount = 0;
			theProfileEdgeCount = 0;
			return Standard_False;
		}
	}

	Standard_Boolean ShapeInteractor::beginExtrusionSelectionImpl(
		const FaceOperationSourceProof& proof) noexcept {
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
			TDF_Label label;
			Standard_Size sourceSubshapeCount = 0;
			Standard_Size profileEdgeCount = 0;
			if (!tryPrepareExtrusionSelection(
					proof,
					label,
					sourceSubshapeCount,
					profileEdgeCount)) {
				return Standard_False;
			}
			_extrusion.label = label;
			_extrusion.originalPresentation = proof.original;
			_extrusion.originalShape = proof.shape;
			_extrusion.selectedFace = proof.openingFace;
			_extrusion.transform = proof.transform;
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

    Standard_Real ShapeInteractor::extrusionMetersPerUnit() const noexcept {
        try {
            if (!_extrusion.IsReady() || myDoc.IsNull()) { return 0.0; }
            const Handle(TDocStd_Document) document = myDoc->ChangeDocument();
            if (document.IsNull()
                || _extrusion.label.Data() != document->GetData()) { return 0.0; }
            Standard_Real documentMeters = 0.001;
            const Standard_Boolean hasUnit =
                XCAFDoc_DocumentTool::GetLengthUnit(document, documentMeters);
            if ((!hasUnit && documentMeters != 0.001)
                || !std::isfinite(documentMeters) || documentMeters <= 0.0) {
                return 0.0;
            }
            const Standard_Real effective = documentMeters
                * std::abs(_extrusion.transform.ScaleFactor());
            return std::isfinite(effective) && effective > 0.0 ? effective : 0.0;
        } catch (...) {
            return 0.0;
        }
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
	Standard_Boolean ShapeInteractor::debugBeginShellSelection(
		const Handle(AIS_Shape)& presentation,
		const TopoDS_Face& face) noexcept {
		FaceOperationSourceProof proof;
		return TryPrepareFaceOperationSource(
			myContext,
			myDoc,
			presentation,
			face,
			ShellOperationController::kMaximumSourceTopologyNodes,
			ShellOperationController::kMaximumStyledSubshapeLabels,
			proof)
			&& beginShellSelectionImpl(proof);
	}

	ShellPreviewDebugState ShapeInteractor::debugShellState() const noexcept {
		return _shellController == nullptr
			? ShellPreviewDebugState()
			: _shellController->debugPreviewState();
	}

	void ShapeInteractor::debugSetShellWorkerBlocked(
		const Standard_Boolean blocked) noexcept {
		if (_shellController != nullptr) {
			_shellController->debugSetWorkerBlocked(blocked);
		}
	}

	void ShapeInteractor::debugSetMaximumShellCaptureTopologyNodes(
		const Standard_Size limit) noexcept {
		if (_shellController != nullptr) {
			_shellController->debugSetMaximumCaptureTopologyNodes(limit);
		}
	}

	void ShapeInteractor::debugSetMaximumShellResultTopologyNodes(
		const Standard_Size limit) noexcept {
		if (_shellController != nullptr) {
			_shellController->debugSetMaximumResultTopologyNodes(limit);
		}
	}

	void ShapeInteractor::debugSetShellTransactionFailureCount(
		const Standard_Size count) noexcept {
		if (_shellController != nullptr) {
			_shellController->debugSetTransactionFailureCount(count);
		}
	}

	void ShapeInteractor::debugSetShellAbortFailureCount(
		const Standard_Size count) noexcept {
		if (_shellController != nullptr) {
			_shellController->debugSetAbortFailureCount(count);
		}
	}

	void ShapeInteractor::debugSetShellPreviewEraseFailureCount(
		const Standard_Size count) noexcept {
		if (_shellController != nullptr) {
			_shellController->debugSetPreviewEraseFailureCount(count);
		}
	}

	void ShapeInteractor::debugSetShellCommitMode(
		const Standard_Integer mode) noexcept {
		if (_shellController != nullptr) {
			_shellController->debugSetCommitMode(mode);
		}
	}

	void ShapeInteractor::debugSetShellPostCommitInspectFailureCount(
		const Standard_Size count) noexcept {
		if (_shellController != nullptr) {
			_shellController->debugSetPostCommitInspectFailureCount(count);
		}
	}

	Standard_Boolean
	ShapeInteractor::debugMutateShellSourcePersistedTransform() noexcept {
		return _shellController != nullptr
			&& _shellController->debugMutateSourcePersistedTransform();
	}

	Standard_Boolean
	ShapeInteractor::debugMutateShellSourcePersistedShape() noexcept {
		return _shellController != nullptr
			&& _shellController->debugMutateSourcePersistedShape();
	}

	Standard_Boolean ShapeInteractor::debugBeginExtrusionSelection(
		const Handle(AIS_Shape)& presentation,
		const TopoDS_Face& face) noexcept {
		FaceOperationSourceProof proof;
		return TryPrepareFaceOperationSource(
			myContext,
			myDoc,
			presentation,
			face,
			ShellOperationController::kMaximumSourceTopologyNodes,
			ShellOperationController::kMaximumStyledSubshapeLabels,
			proof)
			&& beginExtrusionSelectionImpl(proof);
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
				Standard_Size anAggregateTopologyCount = 0;
				const Standard_Size anAggregateTopologyLimit =
					_bevelController->maximumCaptureTopologyNodes();
				const Standard_Size aPerSourceTopologyLimit = std::min(
					BevelOperationController::kMaxSourceTopologyNodes,
					anAggregateTopologyLimit);
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
					Standard_Size aSourceTopologyCount = 0;
					if (!ConsumeBevelSourceTopologyBudget(
							aPresentation->Shape(),
							aPerSourceTopologyLimit,
							anAggregateTopologyLimit,
							anAggregateTopologyCount,
							aSourceTopologyCount)) {
						return Standard_False;
					}
					// The occurrence proof above intentionally precedes this map.
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

	Standard_Boolean ShapeInteractor::tryCaptureBevelSelection(
		std::vector<BevelSourceSelection>& theSelection) const noexcept {
		theSelection.clear();
		if (myContext.IsNull() || myDoc.IsNull()
			|| _bevelController == nullptr
			|| !selectionModeAuthorityIsExact()) {
			return Standard_False;
		}

		TopAbs_ShapeEnum anExpectedOwnerType = TopAbs_SHAPE;
		switch (getSelectionMode()) {
			case ShapeSelectionMode::WholeShape:
				anExpectedOwnerType = TopAbs_SHAPE;
				break;
			case ShapeSelectionMode::Face:
				anExpectedOwnerType = TopAbs_FACE;
				break;
			case ShapeSelectionMode::Edge:
				anExpectedOwnerType = TopAbs_EDGE;
				break;
			case ShapeSelectionMode::Vertex:
			case ShapeSelectionMode::Wire:
				return Standard_False;
		}
		if (_topAbsSelMode != anExpectedOwnerType) {
			return Standard_False;
		}

		struct CapturedSource {
			Handle(AIS_Shape) presentation;
			TopoDS_Shape shape;
			BevelSourceSelection selection;
			TopTools_IndexedMapOfShape edges;
			TopTools_IndexedMapOfShape faces;
			TopTools_IndexedMapOfShape selectedTopology;
		};

		try {
			OCC_CATCH_SIGNALS
			const Handle(TDocStd_Document) aDocument = myDoc->Document();
			if (aDocument.IsNull() || aDocument->HasOpenCommand()
				|| aDocument->GetUndoLimit() == 0) {
				return Standard_False;
			}
			std::vector<std::unique_ptr<CapturedSource>> aSources;
			aSources.reserve(BevelOperationController::kMaxSourceBodies);
			Standard_Size aRawOwnerCount = 0;
			Standard_Size anAggregateTopologyCount = 0;
			Standard_Size anAggregateEdgeCount = 0;
			const Standard_Size anAggregateTopologyLimit =
				_bevelController->maximumCaptureTopologyNodes();
			const Standard_Size aPerSourceTopologyLimit = std::min(
				BevelOperationController::kMaxSourceTopologyNodes,
				anAggregateTopologyLimit);

			for (myContext->InitSelected(); myContext->MoreSelected();
				 myContext->NextSelected()) {
				if (++aRawOwnerCount
					> BevelOperationController::kMaxSelectedEdges) {
					return Standard_False;
				}
				const Handle(SelectMgr_EntityOwner) anOwner =
					myContext->SelectedOwner();
				const Handle(AIS_InteractiveObject) aSelectedInteractive =
					myContext->SelectedInteractive();
				const Handle(StdSelect_BRepOwner) aBRepOwner =
					Handle(StdSelect_BRepOwner)::DownCast(anOwner);
				const Handle(AIS_Shape) aPresentation =
					Handle(AIS_Shape)::DownCast(aSelectedInteractive);
				if (anOwner.IsNull() || !anOwner->HasSelectable()
					|| aSelectedInteractive.IsNull()
					|| aBRepOwner.IsNull() || !aBRepOwner->IsSelected()
					|| !aBRepOwner->HasShape()
					|| aPresentation.IsNull()
					|| aPresentation->Shape().IsNull()
					|| anOwner->Selectable() != aSelectedInteractive
					|| !myContext->IsDisplayed(aPresentation)) {
					return Standard_False;
				}

				CapturedSource* aSource = nullptr;
				for (const std::unique_ptr<CapturedSource>& aCandidate
					 : aSources) {
					if (aCandidate->presentation == aPresentation) {
						aSource = aCandidate.get();
						break;
					}
				}
				if (aSource == nullptr) {
					if (aSources.size()
						>= BevelOperationController::kMaxSourceBodies) {
						return Standard_False;
					}
					const TopoDS_Shape aShape = aPresentation->Shape();
					const TDF_Label aLabel = myDoc->ShapeLabel(aPresentation);
					const TopoDS_Shape aStoredShape =
						XCAFDoc_ShapeTool::GetShape(aLabel);
					Standard_Size aSourceTopologyCount = 0;
					if (aShape.ShapeType() != TopAbs_SOLID
						|| aLabel.IsNull()
						|| aLabel.Data() != aDocument->GetData()
						|| !IsBRepModelingLabel(myDoc, aLabel)
						|| !myDoc->IsPresentationEditable(aPresentation)
						|| aStoredShape.IsNull()
						|| !aStoredShape.IsEqual(aShape)
						|| !ConsumeBevelSourceTopologyBudget(
							aShape,
							aPerSourceTopologyLimit,
							anAggregateTopologyLimit,
							anAggregateTopologyCount,
							aSourceTopologyCount)) {
						return Standard_False;
					}

					auto aCaptured = std::make_unique<CapturedSource>();
					aCaptured->presentation = aPresentation;
					aCaptured->shape = aShape;
					aCaptured->selection.original = aPresentation;
					aCaptured->selection.documentLabel = aLabel;
					aCaptured->selection.selectionMode =
						AIS_Shape::SelectionMode(_topAbsSelMode);
					// The occurrence cap above deliberately precedes these maps.
					TopExp::MapShapes(aShape, TopAbs_EDGE, aCaptured->edges);
					if (aCaptured->edges.IsEmpty()) {
						return Standard_False;
					}
					if (_topAbsSelMode == TopAbs_FACE) {
						TopExp::MapShapes(aShape, TopAbs_FACE, aCaptured->faces);
						if (aCaptured->faces.IsEmpty()) {
							return Standard_False;
						}
					}
					aSource = aCaptured.get();
					aSources.push_back(std::move(aCaptured));
				}

				const TopoDS_Shape anOwnedShape = aBRepOwner->Shape();
				TopoDS_Shape aCanonicalSelectedShape;
				if (_topAbsSelMode == TopAbs_SHAPE) {
					if (anOwnedShape.ShapeType() != TopAbs_SOLID
						|| !anOwnedShape.IsEqual(aSource->shape)) {
						return Standard_False;
					}
					aCanonicalSelectedShape = aSource->shape;
				} else if (_topAbsSelMode == TopAbs_FACE) {
					if (anOwnedShape.ShapeType() != TopAbs_FACE) {
						return Standard_False;
					}
					const Standard_Integer anIndex =
						aSource->faces.FindIndex(anOwnedShape);
					if (anIndex <= 0) {
						return Standard_False;
					}
					aCanonicalSelectedShape =
						aSource->faces.FindKey(anIndex);
					// Face-derived edges may be normalized only after proving the
					// selected Face owner's canonical orientation exactly.
					if (!aCanonicalSelectedShape.IsEqual(anOwnedShape)) {
						return Standard_False;
					}
				} else {
					if (anOwnedShape.ShapeType() != TopAbs_EDGE) {
						return Standard_False;
					}
					const Standard_Integer anIndex =
						aSource->edges.FindIndex(anOwnedShape);
					if (anIndex <= 0) {
						return Standard_False;
					}
					aCanonicalSelectedShape =
						aSource->edges.FindKey(anIndex);
					// Direct Edge selection preserves canonical orientation; unlike
					// Face/Object-derived edges it is never silently reversed.
					if (!aCanonicalSelectedShape.IsEqual(anOwnedShape)) {
						return Standard_False;
					}
				}
				if (aCanonicalSelectedShape.IsNull()
					|| aSource->selectedTopology.Contains(
						aCanonicalSelectedShape)) {
					return Standard_False;
				}
				aSource->selectedTopology.Add(aCanonicalSelectedShape);

				Standard_Size anOwnerEligibleEdgeCount = 0;
				const auto captureEdge = [&](const TopoDS_Edge& anEdge) {
					const Standard_Integer anIndex =
						anEdge.IsNull() ? 0 : aSource->edges.FindIndex(anEdge);
					if (anIndex <= 0) {
						return Standard_False;
					}
					const TopoDS_Edge aCanonicalEdge = TopoDS::Edge(
						aSource->edges.FindKey(anIndex));
					if (aCanonicalEdge.IsNull()) {
						return Standard_False;
					}
					if (!IsBevelEdgeEligible(aCanonicalEdge)) {
						return Standard_True;
					}
					++anOwnerEligibleEdgeCount;
					for (const TopoDS_Edge& anExisting
						 : aSource->selection.edges) {
						if (anExisting.IsSame(aCanonicalEdge)) {
							return Standard_True;
						}
					}
					if (anAggregateEdgeCount
						>= BevelOperationController::kMaxSelectedEdges) {
						return Standard_False;
					}
					++anAggregateEdgeCount;
					aSource->selection.edges.push_back(aCanonicalEdge);
					return Standard_True;
				};

				if (_topAbsSelMode == TopAbs_EDGE) {
					if (!captureEdge(TopoDS::Edge(aCanonicalSelectedShape))) {
						return Standard_False;
					}
				} else {
					Standard_Size anEdgeOccurrenceCount = 0;
					for (TopExp_Explorer anEdge(
							 aCanonicalSelectedShape, TopAbs_EDGE);
						 anEdge.More(); anEdge.Next()) {
						if (++anEdgeOccurrenceCount
								> BevelOperationController::
									kMaxSourceTopologyNodes
							|| !captureEdge(TopoDS::Edge(anEdge.Current()))) {
							return Standard_False;
						}
					}
				}
				if (anOwnerEligibleEdgeCount == 0) {
					return Standard_False;
				}
			}

			if (aRawOwnerCount == 0 || aSources.empty()
				|| anAggregateEdgeCount == 0) {
				return Standard_False;
			}
			std::vector<BevelSourceSelection> aResult;
			aResult.reserve(aSources.size());
			for (const std::unique_ptr<CapturedSource>& aSource : aSources) {
				if (aSource->selection.edges.empty()) {
					return Standard_False;
				}
				aResult.push_back(aSource->selection);
			}
			theSelection = std::move(aResult);
			return Standard_True;
		} catch (...) {
			theSelection.clear();
			return Standard_False;
		}
	}

	Standard_Boolean ShapeInteractor::canBeginBevelSelection() const noexcept {
		if (_bevelController == nullptr) {
			return Standard_False;
		}
		std::vector<BevelSourceSelection> aSelection;
		return tryCaptureBevelSelection(aSelection)
			&& _bevelController->canBegin(aSelection);
	}

	Standard_Boolean
	ShapeInteractor::beginBevelSelectionFromCurrentSelection() noexcept {
		if (_bevelController == nullptr) {
			return Standard_False;
		}
		std::vector<BevelSourceSelection> aSelection;
		if (!tryCaptureBevelSelection(aSelection)
			|| !_bevelController->begin(aSelection)) {
			// Core3DViewer calls this after every in-tool selection toggle. Once
			// the live selection is invalid/empty, a pristine Selecting ledger is
			// no longer authoritative. Retained preview states are never retired.
			if (_bevelController->retireIdleSelectingSelection()) {
				_detectedEdges.clear();
			}
			return Standard_False;
		}
		_detectedEdges.clear();
		_detectedEdges.resize(aSelection.size());
		return Standard_True;
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
		// The frozen v1 contract always rejects already-ineligible topology.
		// Keep the legacy parameter ABI, but do not let it weaken admission.
		(void)preventRechamfer;
		return beginBevelSelectionFromCurrentSelection()
			? static_cast<Standard_Size>(_detectedEdges.size())
			: 0;
	}

	    const size_t ShapeInteractor::getNumberOfDetectedEdges() const {
	        return _detectedEdges.size();
	    }

#ifdef DEBUG
    TopologySelectionDebugState
    ShapeInteractor::debugTopologySelectionState() const noexcept
    {
        TopologySelectionDebugState state;
        state.acceptedMode = _previousSelectionMode;
        if (myContext.IsNull() || myDoc.IsNull()
            || myDoc->Document().IsNull()) {
            return state;
        }

        const auto kindMatchesMode = [](const ShapeSelectionMode mode,
                                        const TopologySelectionDebugElementKind
                                            kind) noexcept {
            switch (mode) {
                case ShapeSelectionMode::WholeShape:
                    return kind
                        == TopologySelectionDebugElementKind::Object;
                case ShapeSelectionMode::Face:
                    return kind == TopologySelectionDebugElementKind::Face;
                case ShapeSelectionMode::Edge:
                    return kind == TopologySelectionDebugElementKind::Edge;
                case ShapeSelectionMode::Vertex:
                    return kind
                        == TopologySelectionDebugElementKind::Vertex;
                case ShapeSelectionMode::Wire:
                    return false;
            }
            return false;
        };
        const auto clearSingleSelection = [&state]() noexcept {
            state.selectedKind =
                TopologySelectionDebugElementKind::None;
            state.topologyIndex = -1;
            state.representation = OcctGeometryRepresentation::Invalid;
            state.entityIdentifier.clear();
            state.singleSelectionExact = Standard_False;
        };

        try {
            OCC_CATCH_SIGNALS
            state.hasDetected = myContext->HasDetected();
            if (!selectionModeAuthorityIsExact()) {
                return state;
            }
            Standard_Boolean allSelectionsMatchMode = Standard_True;
            for (myContext->InitSelected(); myContext->MoreSelected();
                 myContext->NextSelected()) {
                ++state.rawSelectedOwnerCount;
                const Handle(SelectMgr_EntityOwner) owner =
                    myContext->SelectedOwner();
                const Handle(AIS_Shape) presentation =
                    Handle(AIS_Shape)::DownCast(
                        myContext->SelectedInteractive());
                if (owner.IsNull() || !owner->HasSelectable()
                    || presentation.IsNull()
                    || presentation->Shape().IsNull()
                    || owner->Selectable() != presentation) {
                    ++state.invalidSelectedOwnerCount;
                    allSelectionsMatchMode = Standard_False;
                    continue;
                }

                const TDF_Label label = myDoc->ShapeLabel(presentation);
                if (label.IsNull()) {
                    ++state.invalidSelectedOwnerCount;
                    allSelectionsMatchMode = Standard_False;
                    continue;
                }
                const OcctGeometryRepresentation representation =
                    myDoc->GeometryRepresentationForLabel(label);
                const std::string entityIdentifier =
                    myDoc->EntityIdentifierForLabel(label);
                if (representation
                        == OcctGeometryRepresentation::Invalid
                    || entityIdentifier.empty()) {
                    ++state.invalidSelectedOwnerCount;
                    allSelectionsMatchMode = Standard_False;
                    continue;
                }

                TopologySelectionDebugElementKind kind =
                    TopologySelectionDebugElementKind::None;
                Standard_Integer topologyIndex = -1;
                Standard_Boolean exact = Standard_False;
                if (_previousSelectionMode
                    == ShapeSelectionMode::WholeShape) {
                    // Object identity is the accepted selection authority, not
                    // the root TopoDS type. A valid BRep definition may itself
                    // be a Face, Wire, Edge, or Vertex and still represents one
                    // document object in WholeShape mode.
                    kind = TopologySelectionDebugElementKind::Object;
                    topologyIndex = 0;
                    exact = Standard_True;
                } else if (representation
                    != OcctGeometryRepresentation::TriangleMesh) {
                    const Handle(StdSelect_BRepOwner) brepOwner =
                        Handle(StdSelect_BRepOwner)::DownCast(owner);
                    if (!brepOwner.IsNull() && brepOwner->HasShape()) {
                        const TopoDS_Shape selectedShape = brepOwner->Shape();
                        TopAbs_ShapeEnum topologyType = TopAbs_SHAPE;
                        switch (_previousSelectionMode) {
                            case ShapeSelectionMode::Face:
                                kind =
                                    TopologySelectionDebugElementKind::Face;
                                topologyType = TopAbs_FACE;
                                break;
                            case ShapeSelectionMode::Edge:
                                kind =
                                    TopologySelectionDebugElementKind::Edge;
                                topologyType = TopAbs_EDGE;
                                break;
                            case ShapeSelectionMode::Vertex:
                                kind =
                                    TopologySelectionDebugElementKind::Vertex;
                                topologyType = TopAbs_VERTEX;
                                break;
                            case ShapeSelectionMode::WholeShape:
                            case ShapeSelectionMode::Wire:
                                break;
                        }
                        if (topologyType != TopAbs_SHAPE
                            && selectedShape.ShapeType() == topologyType) {
                            TopTools_IndexedMapOfShape topology;
                            Standard_Size visitedOccurrences = 0;
                            for (TopExp_Explorer item(
                                     presentation->Shape(), topologyType);
                                 item.More(); item.Next()) {
                                if (++visitedOccurrences
                                        > ShellOperationController::
                                            kMaximumSourceTopologyNodes) {
                                    break;
                                }
                                topology.Add(item.Current());
                                if (!item.Current().IsSame(selectedShape)) {
                                    continue;
                                }
                                const Standard_Integer oneBasedIndex =
                                    topology.FindIndex(selectedShape);
                                if (oneBasedIndex > 0) {
                                    topologyIndex = oneBasedIndex - 1;
                                    exact = topology.FindKey(oneBasedIndex)
                                        .IsEqual(selectedShape);
                                }
                                break;
                            }
                        }
                    }
                }

                ++state.selectedCount;
                allSelectionsMatchMode = allSelectionsMatchMode
                    && kindMatchesMode(_previousSelectionMode, kind);
                if (state.selectedCount == 1) {
                    state.selectedKind = kind;
                    state.topologyIndex = topologyIndex;
                    state.representation = representation;
                    state.entityIdentifier = entityIdentifier;
                    state.singleSelectionExact = exact;
                }
            }

            if (state.rawSelectedOwnerCount != 1
                || state.selectedCount != 1
                || state.invalidSelectedOwnerCount != 0) {
                clearSingleSelection();
            }
            state.selectionMatchesMode = allSelectionsMatchMode;
            state.ready = Standard_True;
            return state;
        } catch (...) {
            TopologySelectionDebugState failed;
            failed.acceptedMode = _previousSelectionMode;
            return failed;
        }
    }
#endif

    ShapeSelectionModeChangeResult ShapeInteractor::setSelectionMode(
        const ShapeSelectionMode mode) noexcept
    {
        if (myContext.IsNull() || myDoc.IsNull()
            || myDoc->Document().IsNull()) {
            return ShapeSelectionModeChangeResult::NotReady;
        }

        TopAbs_ShapeEnum selectionShapeType = TopAbs_SHAPE;
        switch (mode) {
            case ShapeSelectionMode::WholeShape:
                selectionShapeType = TopAbs_SHAPE;
                break;
            case ShapeSelectionMode::Face:
                selectionShapeType = TopAbs_FACE;
                break;
            case ShapeSelectionMode::Edge:
                selectionShapeType = TopAbs_EDGE;
                break;
            case ShapeSelectionMode::Vertex:
                selectionShapeType = TopAbs_VERTEX;
                break;
            case ShapeSelectionMode::Wire:
                selectionShapeType = TopAbs_WIRE;
                break;
            default:
                return ShapeSelectionModeChangeResult::Unsupported;
        }
        if (_previousSelectionMode == mode
            && selectionModeAuthorityIsExact()) {
            return ShapeSelectionModeChangeResult::Succeeded;
        }

        // Resolve the retained Bevel presentation before touching any active
        // selection mode. The committed originals can be redisplayed by this
        // reconciliation, so the transactional presentation set is captured
        // only after it succeeds.
        if (!resetWireframeTemplateShape()) {
            return ShapeSelectionModeChangeResult::PresentationFailure;
        }

        struct PresentationSelectionModes {
            Handle(AIS_InteractiveObject) presentation;
            std::vector<Standard_Integer> modes;
        };
        AIS_ListOfInteractive displayedObjects;
        std::vector<PresentationSelectionModes> previousPresentations;
        Standard_Integer previousPixelTolerance =
            kAdaptivePixelTolerance;
        try {
            OCC_CATCH_SIGNALS
            myContext->DisplayedObjects(displayedObjects);
            previousPresentations.reserve(
                static_cast<std::size_t>(displayedObjects.Size()));
            for (AIS_ListIteratorOfListOfInteractive anIterator(
                     displayedObjects);
                 anIterator.More(); anIterator.Next()) {
                PresentationSelectionModes snapshot;
                snapshot.presentation = anIterator.Value();
                TColStd_ListOfInteger activeModes;
                myContext->ActivatedModes(
                    snapshot.presentation, activeModes);
                snapshot.modes.reserve(
                    static_cast<std::size_t>(activeModes.Extent()));
                for (TColStd_ListIteratorOfListOfInteger modeIterator(
                         activeModes);
                     modeIterator.More(); modeIterator.Next()) {
                    snapshot.modes.push_back(modeIterator.Value());
                }
                previousPresentations.push_back(std::move(snapshot));
            }
            const Handle(StdSelect_ViewerSelector3d)& selector =
                myContext->MainSelector();
            if (selector.IsNull()) {
                return ShapeSelectionModeChangeResult::PresentationFailure;
            }
            const Standard_Integer rawPixelTolerance =
                selector->CustomPixelTolerance();
            previousPixelTolerance = rawPixelTolerance < 0
                ? kAdaptivePixelTolerance : rawPixelTolerance;
        } catch (...) {
            return ShapeSelectionModeChangeResult::PresentationFailure;
        }

        const auto modesMatch = [&](const PresentationSelectionModes& snapshot)
            -> Standard_Boolean {
            TColStd_ListOfInteger activeModes;
            myContext->ActivatedModes(snapshot.presentation, activeModes);
            std::vector<Standard_Integer> actualModes;
            actualModes.reserve(
                static_cast<std::size_t>(activeModes.Extent()));
            for (TColStd_ListIteratorOfListOfInteger modeIterator(activeModes);
                 modeIterator.More(); modeIterator.Next()) {
                actualModes.push_back(modeIterator.Value());
            }
            std::sort(actualModes.begin(), actualModes.end());
            std::vector<Standard_Integer> expectedModes = snapshot.modes;
            std::sort(expectedModes.begin(), expectedModes.end());
            return actualModes == expectedModes;
        };
        const auto restorePreviousPresentation = [&]() noexcept {
            Standard_Boolean restoredEveryPresentation = Standard_True;
            try {
                OCC_CATCH_SIGNALS
                for (const PresentationSelectionModes& snapshot :
                     previousPresentations) {
                    myContext->Deactivate(snapshot.presentation);
                    for (const Standard_Integer activeMode : snapshot.modes) {
                        myContext->SetSelectionModeActive(
                            snapshot.presentation,
                            activeMode,
                            Standard_True,
                            AIS_SelectionModesConcurrency_Multiple,
                            Standard_True);
                    }
                    restoredEveryPresentation =
                        modesMatch(snapshot) && restoredEveryPresentation;
                }
                myContext->SetPixelTolerance(previousPixelTolerance);
                const Handle(StdSelect_ViewerSelector3d)& selector =
                    myContext->MainSelector();
                restoredEveryPresentation = restoredEveryPresentation
                    && !selector.IsNull()
                    && selector->CustomPixelTolerance()
                        == previousPixelTolerance;
            } catch (...) {
                restoredEveryPresentation = Standard_False;
            }
            if (!restoredEveryPresentation) {
                // The retained enum is no longer publishable when the exact
                // presentation snapshot cannot be restored. Remove owners so
                // neither selected nor hover identity can leak across the
                // fail-closed poisoned state; the same mode can later repair.
                try {
                    OCC_CATCH_SIGNALS
                    myContext->ClearDetected(Standard_False);
                    myContext->ClearSelected(Standard_True);
                } catch (...) {
                }
            }
            return restoredEveryPresentation;
        };

        for (AIS_ListIteratorOfListOfInteractive anIterator(displayedObjects);
             anIterator.More(); anIterator.Next()) {
            if (!trySetInteractiveObjectSelectionMode(
                    anIterator.Value(), mode, selectionShapeType)) {
                (void)restorePreviousPresentation();
                return ShapeSelectionModeChangeResult::PresentationFailure;
            }
        }

        try {
            OCC_CATCH_SIGNALS
            // Whole-shape/face selection uses OCCT's adaptive default. Its raw
            // CustomPixelTolerance() value is negative even though the fresh
            // effective tolerance is 2. Passing 2 here can be a no-op in OCCT
            // and must not be mistaken for an explicit custom value.
            myContext->SetPixelTolerance(
                UsesExpandedTopologyPixelTolerance(mode)
                    ? kExpandedTopologyPixelTolerance
                    : kAdaptivePixelTolerance);
#ifdef DEBUG
            if (_debugSelectionModeVerificationFailureCount > 0) {
                --_debugSelectionModeVerificationFailureCount;
                throw Standard_Failure(
                    "Injected selection-mode verification failure");
            }
#endif
            if (!selectionModeAuthorityMatches(
                    mode, selectionShapeType)) {
                throw Standard_Failure(
                    "Selection presentation authority mismatch");
            }

            // Do not explicitly clear owners until every requested presentation
            // and picker tolerance has been proven. OCCT may already invalidate
            // transient detection while reconfiguring selection modes; rollback
            // restores modes and tolerance but cannot fabricate that hover owner.
            myContext->ClearDetected(Standard_False);
            myContext->ClearSelected(Standard_True);
            if (!selectionModeAuthorityMatches(
                    mode, selectionShapeType)) {
                throw Standard_Failure(
                    "Selection authority changed while clearing owners");
            }
        } catch (...) {
            (void)restorePreviousPresentation();
            return ShapeSelectionModeChangeResult::PresentationFailure;
        }

        _previousSelectionMode = mode;
        _topAbsSelMode = selectionShapeType;
        return ShapeSelectionModeChangeResult::Succeeded;
    }

	void ShapeInteractor::setInteractiveObjectSelectionMode(
        const Handle(AIS_InteractiveObject) aio)
    {
        (void)trySetInteractiveObjectSelectionMode(
            aio, _previousSelectionMode, _topAbsSelMode);
	}

    Standard_Boolean ShapeInteractor::trySetInteractiveObjectSelectionMode(
        const Handle(AIS_InteractiveObject)& aio,
        const ShapeSelectionMode mode,
        const TopAbs_ShapeEnum topAbsMode) noexcept
    {
        if (myContext.IsNull() || myDoc.IsNull()
            || myDoc->Document().IsNull() || aio.IsNull()) {
            return Standard_False;
        }
        try {
            OCC_CATCH_SIGNALS
            if (!myDoc->IsPresentationEditable(aio)) {
                myContext->Deactivate(aio);
                return interactiveObjectSelectionModeMatches(
                    aio, mode, topAbsMode);
            }
            const TDF_Label aLabel = myDoc->ShapeLabel(aio);
            const OcctGeometryRepresentation aRepresentation =
                myDoc->GeometryRepresentationForLabel(aLabel);
            if (aRepresentation == OcctGeometryRepresentation::Invalid) {
                myContext->Deactivate(aio);
                return interactiveObjectSelectionModeMatches(
                    aio, mode, topAbsMode);
            }

            myContext->Deactivate(aio);
            if (aRepresentation
                == OcctGeometryRepresentation::TriangleMesh) {
                // Mesh imports expose no editable BRep subshape topology.
                // In every subshape mode they are fully deactivated, while a
                // mixed document may continue selecting its BRep contents.
                if (mode == ShapeSelectionMode::WholeShape) {
                    myContext->SetSelectionModeActive(
                        aio,
                        AIS_Shape::SelectionMode(TopAbs_SHAPE),
                        Standard_True,
                        AIS_SelectionModesConcurrency_Single,
                        Standard_True);
                }
                return interactiveObjectSelectionModeMatches(
                    aio, mode, topAbsMode);
            }

            myContext->SetSelectionModeActive(
                aio,
                AIS_Shape::SelectionMode(topAbsMode),
                Standard_True,
                AIS_SelectionModesConcurrency_Single,
                Standard_True);
            return interactiveObjectSelectionModeMatches(
                aio, mode, topAbsMode);
        } catch (...) {
            // Never leave the presentation active in a possibly half-applied
            // requested mode. The transition owner subsequently attempts to
            // restore the prior mode across the complete displayed set.
            try {
                OCC_CATCH_SIGNALS
                myContext->Deactivate(aio);
            } catch (...) {
            }
            return Standard_False;
        }
    }

    Standard_Boolean ShapeInteractor::interactiveObjectSelectionModeMatches(
        const Handle(AIS_InteractiveObject)& aio,
        const ShapeSelectionMode mode,
        const TopAbs_ShapeEnum topAbsMode) const noexcept
    {
        if (myContext.IsNull() || myDoc.IsNull()
            || myDoc->Document().IsNull() || aio.IsNull()) {
            return Standard_False;
        }
        try {
            OCC_CATCH_SIGNALS
            const TDF_Label label = myDoc->ShapeLabel(aio);
            const OcctGeometryRepresentation representation =
                myDoc->GeometryRepresentationForLabel(label);
            // Transient operation and gizmo presentations are outside the
            // committed document topology authority.
            if (representation == OcctGeometryRepresentation::Invalid) {
                return Standard_True;
            }

            std::vector<Standard_Integer> expectedModes;
            if (myDoc->IsPresentationEditable(aio)) {
                if (representation
                    == OcctGeometryRepresentation::TriangleMesh) {
                    if (mode == ShapeSelectionMode::WholeShape) {
                        expectedModes.push_back(
                            AIS_Shape::SelectionMode(TopAbs_SHAPE));
                    }
                } else if (representation
                        == OcctGeometryRepresentation::BRep
                    || representation
                        == OcctGeometryRepresentation::LegacyUnknown) {
                    expectedModes.push_back(
                        AIS_Shape::SelectionMode(topAbsMode));
                } else {
                    return Standard_False;
                }
            }

            TColStd_ListOfInteger activeModes;
            myContext->ActivatedModes(aio, activeModes);
            std::vector<Standard_Integer> actualModes;
            actualModes.reserve(
                static_cast<std::size_t>(activeModes.Extent()));
            for (TColStd_ListIteratorOfListOfInteger anIterator(activeModes);
                 anIterator.More(); anIterator.Next()) {
                actualModes.push_back(anIterator.Value());
            }
            std::sort(expectedModes.begin(), expectedModes.end());
            std::sort(actualModes.begin(), actualModes.end());
            return actualModes == expectedModes;
        } catch (...) {
            return Standard_False;
        }
    }

    Standard_Boolean ShapeInteractor::selectionModeAuthorityMatches(
        const ShapeSelectionMode mode,
        const TopAbs_ShapeEnum topAbsMode) const noexcept
    {
        if (myContext.IsNull() || myDoc.IsNull()
            || myDoc->Document().IsNull()) {
            return Standard_False;
        }
        try {
            OCC_CATCH_SIGNALS
            AIS_ListOfInteractive displayedObjects;
            myContext->DisplayedObjects(displayedObjects);
            for (AIS_ListIteratorOfListOfInteractive anIterator(
                     displayedObjects);
                 anIterator.More(); anIterator.Next()) {
                if (!interactiveObjectSelectionModeMatches(
                        anIterator.Value(), mode, topAbsMode)) {
                    return Standard_False;
                }
            }
            const Handle(StdSelect_ViewerSelector3d)& selector =
                myContext->MainSelector();
            if (selector.IsNull()) {
                return Standard_False;
            }
            const Standard_Integer rawPixelTolerance =
                selector->CustomPixelTolerance();
            return UsesExpandedTopologyPixelTolerance(mode)
                ? rawPixelTolerance == kExpandedTopologyPixelTolerance
                : rawPixelTolerance < 0;
        } catch (...) {
            return Standard_False;
        }
    }

    Standard_Boolean
    ShapeInteractor::selectionModeAuthorityIsExact() const noexcept
    {
        return selectionModeAuthorityMatches(
            _previousSelectionMode, _topAbsSelMode);
    }

    Standard_Boolean
    ShapeInteractor::selectionModeAuthorityIsExactIgnoring(
        const std::vector<Handle(AIS_Shape)>& theSuspendedPresentations)
        const noexcept
    {
        if (theSuspendedPresentations.empty() || myContext.IsNull()
            || myDoc.IsNull() || myDoc->Document().IsNull()) {
            return Standard_False;
        }
        try {
            OCC_CATCH_SIGNALS
            std::unordered_set<const AIS_InteractiveObject*> expected;
            expected.reserve(theSuspendedPresentations.size());
            for (const Handle(AIS_Shape)& aPresentation :
                 theSuspendedPresentations) {
                if (aPresentation.IsNull()
                    || !expected.insert(aPresentation.get()).second) {
                    return Standard_False;
                }
            }

            std::unordered_set<const AIS_InteractiveObject*> observed;
            observed.reserve(expected.size());
            AIS_ListOfInteractive displayedObjects;
            myContext->DisplayedObjects(displayedObjects);
            for (AIS_ListIteratorOfListOfInteractive anIterator(
                     displayedObjects);
                 anIterator.More(); anIterator.Next()) {
                const Handle(AIS_InteractiveObject)& aPresentation =
                    anIterator.Value();
                if (!aPresentation.IsNull()
                    && expected.find(aPresentation.get()) != expected.end()) {
                    TColStd_ListOfInteger activeModes;
                    myContext->ActivatedModes(aPresentation, activeModes);
                    if (!activeModes.IsEmpty()
                        || !observed.insert(aPresentation.get()).second) {
                        return Standard_False;
                    }
                    continue;
                }
                if (!interactiveObjectSelectionModeMatches(
                        aPresentation,
                        _previousSelectionMode,
                        _topAbsSelMode)) {
                    return Standard_False;
                }
            }
            if (observed.size() != expected.size()) {
                return Standard_False;
            }

            const Handle(StdSelect_ViewerSelector3d)& selector =
                myContext->MainSelector();
            if (selector.IsNull()) {
                return Standard_False;
            }
            const Standard_Integer rawPixelTolerance =
                selector->CustomPixelTolerance();
            return UsesExpandedTopologyPixelTolerance(
                       _previousSelectionMode)
                ? rawPixelTolerance == kExpandedTopologyPixelTolerance
                : rawPixelTolerance < 0;
        } catch (...) {
            return Standard_False;
        }
    }

    Standard_Boolean
    ShapeInteractor::captureExtrusionSelectionModeSuspendedPresentations(
        std::vector<Handle(AIS_Shape)>& thePresentations) const noexcept
    {
        thePresentations.clear();
        const ExtrusionPreviewState aState = extrusionPreviewState();
        const Standard_Boolean ownsRetainedState =
            (aState == ExtrusionPreviewState::Ready
                && canApplyExtrusion())
            || (aState == ExtrusionPreviewState::OutcomeUnknown
                && canRetryExtrusionResolution());
        if (!ownsRetainedState || myContext.IsNull()
            || _extrusion.originalPresentation.IsNull()
            || _extrusion.candidatePresentation.IsNull()
            || _extrusion.originalPresentation
                == _extrusion.candidatePresentation) {
            return Standard_False;
        }
        try {
            OCC_CATCH_SIGNALS
            const auto appendSuspended =
                [&](const Handle(AIS_Shape)& aPresentation) {
                    TColStd_ListOfInteger activeModes;
                    if (aPresentation.IsNull()
                        || aPresentation->Shape().IsNull()
                        || !myContext->IsDisplayed(aPresentation)) {
                        return Standard_False;
                    }
                    myContext->ActivatedModes(aPresentation, activeModes);
                    if (!activeModes.IsEmpty()) {
                        return Standard_False;
                    }
                    thePresentations.push_back(aPresentation);
                    return Standard_True;
                };
            if (!appendSuspended(_extrusion.originalPresentation)
                || !appendSuspended(_extrusion.candidatePresentation)) {
                thePresentations.clear();
                return Standard_False;
            }
            return thePresentations.size() == 2;
        } catch (...) {
            thePresentations.clear();
            return Standard_False;
        }
    }

    Standard_Boolean
    ShapeInteractor::captureShellSelectionModeSuspendedPresentations(
        std::vector<Handle(AIS_Shape)>& thePresentations) const noexcept
    {
        thePresentations.clear();
        return _shellController != nullptr
            && _shellController
                ->captureSelectionModeSuspendedPresentations(
                    thePresentations);
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
					hasActiveBevel() || hasActiveExtrusion()
						|| hasActiveShell())) {
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
					hasActiveBevel() || hasActiveExtrusion()
						|| hasActiveShell())) {
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
					hasActiveBevel() || hasActiveExtrusion()
						|| hasActiveShell())) {
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
					hasActiveBevel() || hasActiveExtrusion()
						|| hasActiveShell())) {
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
