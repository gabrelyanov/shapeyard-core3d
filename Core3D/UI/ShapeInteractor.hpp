//
//  ShapeInteractor.hpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 19.04.2024.
//

#ifndef ShapeInteractor_hpp
#define ShapeInteractor_hpp

#include "Interactor.hpp"
#include "BevelOperationController.hpp"
#include <AIS_Shape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TDocStd_Document.hxx>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace core3d {

	//flag for don't chamfer or fillet for already filleted shape
	#define PREVENT_RECHAMFER true

    enum struct ShapeSelectionMode {
        WholeShape = 0,
        Vertex,
        Edge,
        Wire,
        Face
    };

    enum class ExtrusionPreviewState : std::uint8_t {
        Unavailable = 0,
        Selecting,
        Ready,
        OutcomeUnknown,
        Failed,
    };

	struct EdgesSelection {
		Handle(SelectMgr_EntityOwner) detectedOwner;
		TDF_Label documentLabel;
		std::vector<TopoDS_Edge> edges;
		Handle(AIS_Shape) tempFilletShapePrs;
		Handle(AIS_Shape) filletShapePrs;
		gp_Trsf transform;
		Graphic3d_NameOfMaterial materialName = Graphic3d_NameOfMaterial_ShinyPlastified;
		Quantity_NameOfColor colorName = Quantity_NOC_GRAY80;
	};

#ifdef DEBUG
    struct ExtrusionDebugState {
        Standard_Boolean selectionReady = Standard_False;
        Standard_Boolean previewActive = Standard_False;
        Standard_Boolean canApply = Standard_False;
        Standard_Boolean resolutionRetryable = Standard_False;
        Standard_Boolean commandOpen = Standard_False;
        Standard_Boolean lastApplySucceeded = Standard_False;
        Standard_Real distance = 0.0;
        Standard_Size sourceSubshapeCount = 0;
        Standard_Size profileEdgeCount = 0;
        Standard_Size candidateSubshapeCount = 0;
        Standard_Size candidateSolidCount = 0;
        Standard_Integer lastFailureStage = 0;
        Standard_Integer lastFeatureStatus = 0;
        Standard_Integer lastCandidateShapeType = -1;
        Standard_Size rawDirectChildCount = 0;
        Standard_Integer rawDirectChildShapeType = -1;
        Standard_Boolean historyHasModified = Standard_False;
        Standard_Boolean historyHasGenerated = Standard_False;
        Standard_Boolean historyFaceDeleted = Standard_False;
        Standard_Real candidateVolume = 0.0;
        Standard_Real candidateMinX = 0.0;
        Standard_Real candidateMinY = 0.0;
        Standard_Real candidateMinZ = 0.0;
        Standard_Real candidateMaxX = 0.0;
        Standard_Real candidateMaxY = 0.0;
        Standard_Real candidateMaxZ = 0.0;
    };
#endif

    class ShapeInteractor : public Interactor {
        ShapeSelectionMode _previousSelectionMode = ShapeSelectionMode::WholeShape; //whole shape
		TopAbs_ShapeEnum _topAbsSelMode = TopAbs_ShapeEnum::TopAbs_SHAPE;
		Handle(AIS_Shape) _temporalChamferShapePrs;
		Handle(AIS_InteractiveObject) _subtractorObjectPrs;
			std::vector<EdgesSelection> _detectedEdges;
			std::shared_ptr<BevelOperationController> _bevelController;

        struct ExtrusionSelection {
            TDF_Label label;
            Handle(AIS_Shape) originalPresentation;
            Handle(AIS_Shape) candidatePresentation;
            TopoDS_Shape originalShape;
            TopoDS_Face selectedFace;
            gp_Trsf transform;
            Standard_Real distance = 0.0;
            Standard_Size sourceSubshapeCount = 0;
            Standard_Size profileEdgeCount = 0;
            Standard_Size candidateSubshapeCount = 0;
            Standard_Size candidateSolidCount = 0;
            Standard_Boolean ownsCommand = Standard_False;
            Standard_Boolean rollbackFailed = Standard_False;
            Standard_Boolean commitOutcomeUnknown = Standard_False;

            Standard_Boolean IsReady() const {
                return !label.IsNull()
                    && !originalPresentation.IsNull()
                    && !originalShape.IsNull()
                    && !selectedFace.IsNull();
            }
        };
        ExtrusionSelection _extrusion;
#ifdef DEBUG
        Standard_Boolean _lastExtrusionApplySucceeded = Standard_False;
        Standard_Size _lastExtrusionResultSubshapeCount = 0;
        Standard_Size _lastExtrusionResultSolidCount = 0;
        Standard_Integer _lastExtrusionFailureStage = 0;
        Standard_Integer _lastExtrusionFeatureStatus = 0;
        Standard_Integer _lastExtrusionCandidateShapeType = -1;
        Standard_Size _lastExtrusionRawDirectChildCount = 0;
        Standard_Integer _lastExtrusionRawDirectChildShapeType = -1;
        Standard_Boolean _lastExtrusionHistoryHasModified = Standard_False;
        Standard_Boolean _lastExtrusionHistoryHasGenerated = Standard_False;
        Standard_Boolean _lastExtrusionHistoryFaceDeleted = Standard_False;
        Standard_Real _lastExtrusionCandidateVolume = 0.0;
        Standard_Real _lastExtrusionCandidateBounds[6] = {};
        Standard_Integer _debugExtrusionCommitMode = 0;
        Standard_Size _debugExtrusionAbortFailureCount = 0;
        Standard_Size _debugExtrusionPostCommitInspectFailureCount = 0;
#endif

        void extractGeometryShapes(const TopoDS_Shape &shape,
                                   std::vector<TopoDS_Face> &faces,
                                   std::vector<TopoDS_Edge> &edges,
                                   std::vector<TopoDS_Vertex> &vertices);
    public:
        ShapeInteractor() = delete;
        ShapeInteractor(Handle(Core3DContext), Handle(Core3DView), Handle(OcctDocument) doc);
        ~ShapeInteractor() noexcept;
        
        const size_t getNumberOfDetectedEdges() const;
        void setSelectionMode(const ShapeSelectionMode mode);
        const ShapeSelectionMode getSelectionMode() const;
			Standard_Boolean setChamferValueForSelection(const Standard_Real value);
			BevelApplyResult applyBevel() noexcept;
			Standard_Boolean canApplyBevel() const noexcept;
			Standard_Boolean hasActiveBevel() const noexcept;
			Standard_Boolean isBevelSelectionFrozen() const noexcept;
			BevelPreviewState bevelPreviewState() const noexcept;
			std::uint64_t bevelPreviewGeneration() const noexcept;
			Standard_Boolean captureBevelPreview(
				BevelPreviewCapture& capture) const noexcept;
			void setBevelPreviewStateChangedCallback(
				std::function<void()> callback);
				Standard_Boolean resetWireframeTemplateShape() noexcept;
			Standard_Boolean cancelChamfer() noexcept;

        //! Capture one selected planar face on one editable free solid.
        //! No document command is opened until a nonzero preview succeeds.
        Standard_Boolean beginExtrusionSelection() noexcept;
        //! Recompute a local BRepFeat prism preview in model units. Zero
        //! discards the open preview while retaining the captured face.
        Standard_Boolean setExtrusionValueForSelection(
            Standard_Real distance) noexcept;
        Standard_Boolean canApplyExtrusion() const noexcept;
        //! A prior CommitCommand closed or threw, but the authoritative label
        //! could not yet be classified. Apply remains available solely to
        //! re-inspect; no second commit occurs while the document is closed.
        Standard_Boolean canRetryExtrusionResolution() const noexcept;
        Standard_Boolean hasActiveExtrusion() const noexcept;
        Standard_Boolean hasExtrusionPreview() const noexcept;
        ExtrusionPreviewState extrusionPreviewState() const noexcept;
        Standard_Boolean applyExtrusion() noexcept;
        //! Return false only when an owned preview command could not be
        //! confirmed aborted; state remains intact so cancellation can retry.
        Standard_Boolean cancelExtrusion() noexcept;
#ifdef DEBUG
        Standard_Boolean debugBeginExtrusionSelection(
            const Handle(AIS_Shape)& presentation,
            const TopoDS_Face& face) noexcept;
	        ExtrusionDebugState debugExtrusionState() const noexcept;
			Standard_Boolean debugBeginBevelSelection(
				const Handle(AIS_Shape)& presentation,
				const std::vector<Standard_Size>& edgeTopologyIndices) noexcept;
			Standard_Boolean debugBeginBevelSelection(
				const std::vector<Handle(AIS_Shape)>& presentations,
				const std::vector<std::vector<Standard_Size>>&
					edgeTopologyIndices) noexcept;
			BevelPreviewDebugState debugBevelState() const noexcept;
			void debugSetBevelWorkerBlocked(Standard_Boolean blocked) noexcept;
			void debugSetMaximumBevelCaptureTopologyNodes(
				Standard_Size limit) noexcept;
			void debugSetMaximumBevelResultTopologyNodes(
				Standard_Size limit) noexcept;
			void debugSetMaximumBevelResultSolids(Standard_Size limit) noexcept;
			void debugSetBevelTransactionFailureCount(
				Standard_Size count) noexcept;
			void debugSetBevelCancelDiscardFailureCount(
				Standard_Size count) noexcept;
			Standard_Boolean
				debugMutateFirstBevelSourcePersistedTransform() noexcept;
        void debugSetExtrusionCommitMode(Standard_Integer mode) noexcept {
            _debugExtrusionCommitMode = mode >= 0 && mode <= 2 ? mode : 0;
        }
        void debugSetExtrusionAbortFailureCount(Standard_Size count) noexcept {
            _debugExtrusionAbortFailureCount = count;
        }
        void debugSetExtrusionPostCommitInspectFailureCount(
            Standard_Size count) noexcept {
            _debugExtrusionPostCommitInspectFailureCount = count;
        }
#endif
		Standard_Size saveSelectionEdges(bool preventRechamfer = PREVENT_RECHAMFER);
		const Standard_Boolean isEmptyOfDisplayedObjects() const;
		const Standard_Size getNumberOfDisplayedShapes() const;
        void exportShapes();
        void exportToStl(const std::string &filename, const Standard_Boolean isASCII = Standard_True);
        void exportToObj(const std::string &filename);
        void exportToGltf(const std::string &filename);
        void exportToStep(const std::string &filename);

	private:
		void setInteractiveObjectSelectionMode(const Handle(AIS_InteractiveObject) aio);
			Standard_Boolean beginBevelSelectionFromDetectedEdges() noexcept;
        Standard_Boolean beginExtrusionSelectionImpl(
            const Handle(AIS_Shape)& presentation,
            const TopoDS_Face& face) noexcept;
        Standard_Boolean discardExtrusionPreview(
            Standard_Boolean updateViewer) noexcept;
        void restoreExtrusionOriginalPresentation() noexcept;
        void clearExtrusionSelection() noexcept;

    };
}


#endif /* ShapeInteractor_hpp */
