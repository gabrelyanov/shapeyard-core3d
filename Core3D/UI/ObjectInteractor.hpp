//
//  ObjectInteractor.hpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 19.04.2024.
//

#ifndef ObjectInteractor_hpp
#define ObjectInteractor_hpp

#include "Interactor.hpp"
#include "Core3DManipulator.hpp"
#include <AIS_Shape.hxx>
#include "BooleanOperationController.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>
#include <TopoDS_Face.hxx>

namespace core3d {

    enum struct PrimitiveManipulatorType {
        PrimitiveGizmoTypeNone = 0,
        PrimitiveGizmoTypeMoveRotate,
        PrimitiveGizmoTypeScale,
        PrimitiveGizmoTypeChamfer,
        PrimitiveGizmoTypeSubtract,
        PrimitiveGizmoTypeUnion,
        PrimitiveGizmoTypeMirror,
        PrimitiveGizmoTypeMaterial,
        PrimitiveGizmoTypeExtrude,
        PrimitiveGizmoTypeIntersect,
    };

    enum class PresentationOverlayCaptureStatus : std::uint8_t {
        Available = 0,
        Unsafe,
    };

    enum class MirrorApplyResult : std::uint8_t {
        NoChange = 0,
        Applied,
        AppliedNeedsDocumentRedraw,
    };

    enum class MirrorPreviewState : std::uint8_t {
        Unavailable = 0,
        Selecting,
        Ready,
        Committing,
        OutcomeUnknown,
        Failed,
    };

#ifdef DEBUG
    struct MirrorPreviewDebugState {
        MirrorPreviewState state = MirrorPreviewState::Unavailable;
        std::uint64_t generation = 0;
        Standard_Size previewBodyCount = 0;
        Standard_Size pendingResultCount = 0;
        Standard_Boolean activeOperation = Standard_False;
        Standard_Boolean previewValid = Standard_False;
        Standard_Boolean canApply = Standard_False;
        Standard_Boolean ownsDocumentCommand = Standard_False;
        Standard_Boolean documentCommandOpen = Standard_False;
		Standard_Boolean pickingCustomPlane = Standard_False;
		Standard_Boolean hasCustomPlane = Standard_False;
		Standard_Boolean previewUsesCustomPlane = Standard_False;
		Standard_Boolean manipulatorAttached = Standard_False;
		Standard_Size referencePresentationCount = 0;
		Standard_Real customPlaneOffset = 0.0;
		Standard_Real customPlaneMinimumOffset = 0.0;
		Standard_Real customPlaneMaximumOffset = 0.0;
    };
#endif

    class ObjectInteractor : public Interactor {
        
		PrimitiveManipulatorType _manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
        std::shared_ptr<BooleanOperationController> _booleanOpController;
        Handle(Core3DManipulator) _manipulator;
//        std::vector<TopoDS_Shape> _beforeTransformObjects;
    public:
        static constexpr Standard_ShortReal kManipulatorGap = 100;
        static constexpr std::size_t kMaxMirrorPreviewBodies = 8;
		static constexpr Standard_Size kMaxMirrorSourceTopologyNodes = 1'024;
		static constexpr Standard_Size kMaxMirrorTopologyNodes = 8'192;
		//! Face picking has a separate, conservative budget. It must never ask
		//! OCCT to materialize selection owners for the project's full topology.
		static constexpr Standard_Size
			kMaxMirrorReferenceTopologyNodesPerPresentation = 1'024;
		static constexpr Standard_Size
			kMaxMirrorReferenceFacesPerPresentation = 256;
		static constexpr Standard_Size
			kMaxMirrorReferenceTopologyNodes = 8'192;
		static constexpr Standard_Size kMaxMirrorReferenceFaces = 1'024;
        
        ObjectInteractor() = delete;
        ObjectInteractor(Handle(Core3DContext), Handle(Core3DView), Handle(OcctDocument) doc, Standard_ShortReal manipulatorSide = 300);
        
        void selectLastObject();
		void selectAll();
        void deleteSelected();
        void duplicateSelected();
		void attachManipulatorToSelection(bool detach = false);
        
        bool transformManipulator(int theX, int theY);
        bool startTransformManipulator(int theX, int theY);
        void finishInteraction();
        void cancelInteraction();
        void setManipulatorType(PrimitiveManipulatorType type);
		Handle(TopLoc_Datum3D) manipulatorTransform();
		gp_XYZ manipulatorPosition();
        
        void SelectAndAttachManipulator(Handle(AIS_InteractiveObject) toObject);
        const bool isManipulatorAttached() const;
        //! True only from a successful raw touch start through finish/cancel.
        //! Unlike isManipulatorInteractionActive(), hover detection alone does
        //! not activate this state; mirror-plane gestures are included.
        const bool isManipulatorGestureActive() const;
        //! Legacy detected-mode state used by GL touch/tap suppression.
        const bool isManipulatorInteractionActive() const;
        const PrimitiveManipulatorType getManipulatorType() const;

        //! Publish an already-committed authoritative transform to the
        //! selected AIS object and resynchronize any attached manipulator.
        //! Returns false so the viewer can rebuild from OCAF as a fail-safe.
        bool publishCommittedInspectorTransform(
            const Handle(AIS_Shape)& presentation,
            const gp_Trsf& transform) noexcept;

        //! Capture only supported idle presentation. A valid mirror trial set
        //! is returned as explicit owned AIS_Shape handles so the snapshot
        //! builder never enumerates the interactive context. Available with
        //! empty content is an explicit clear; Unsafe means a renderer must
        //! retain OCCT.
        PresentationOverlayCaptureStatus captureIdlePresentationOverlay(
            scene::PresentationOverlayContent& theContent,
            std::vector<Handle(AIS_Shape)>& theMirrorPreviewObjects,
            BooleanPreviewCapture& theBooleanPreview) const noexcept;

        const bool isSelected() const;
		
		void fillSelectedState(Standard_Boolean forceActor, BooleanAction action);
		void updateDetectedState(Standard_Boolean forceActor, BooleanAction action);
		Standard_Boolean beginBoolean(BooleanAction action) noexcept;
		BooleanApplyResult applyBoolean(BooleanAction action) noexcept;
		void cancelBoolean(BooleanAction action) noexcept;
		void cancelActiveBoolean() noexcept;
		const bool canApplyBoolean() const;
		const bool hasActiveBoolean() const;
		const bool hasActiveBoolean(BooleanAction action) const;
		const bool hasUnresolvedBoolean() const;
		const bool isBooleanSelectionFrozen() const;
		BooleanPreviewState booleanPreviewState() const noexcept;
		std::uint64_t booleanPreviewGeneration() const noexcept;
		void setBooleanPreviewStateChangedCallback(
			std::function<void()> callback);
#ifdef DEBUG
		Standard_Boolean debugBeginBooleanSelection(
			const std::vector<Handle(AIS_InteractiveObject)>& actors,
			const std::vector<Handle(AIS_InteractiveObject)>& subjects,
			BooleanAction action) noexcept;
		Standard_Boolean debugRecomputeBooleanPreview(
			BooleanAction action) noexcept;
		BooleanPreviewDebugState debugBooleanPreviewState() const noexcept;
		void debugSetBooleanPreviewWorkerBlocked(
			Standard_Boolean blocked) noexcept;
		void debugSetMaximumBooleanCaptureTopologyNodes(
			Standard_Size limit) noexcept;
		void debugSetMaximumBooleanResultTopologyNodes(
			Standard_Size limit) noexcept;
		void debugSetMaximumBooleanResultSolids(
			Standard_Size limit) noexcept;
		void debugSetBooleanTransactionFailureCount(
			Standard_Size count) noexcept;
		void debugSetBooleanAbortFailureCount(
			Standard_Size count) noexcept;
#endif
        MirrorApplyResult applyMirror() noexcept;
		Standard_Boolean cancelMirror() noexcept;
		Standard_Boolean tryMirror(
			Standard_Integer axisIndex,
			bool backward) noexcept;
		//! Enter a one-tap, non-mutating planar-face detector for Mirror.
		Standard_Boolean beginMirrorPlanePicking() noexcept;
		//! Exit face picking and restore the exact selection modes captured on entry.
		Standard_Boolean cancelMirrorPlanePicking() noexcept;
		//! Consume one viewport tap while face picking is active.
		Standard_Boolean pickMirrorPlaneAt(
			Standard_Integer theX,
			Standard_Integer theY) noexcept;
		const bool isPickingMirrorPlane() const noexcept;
		const bool hasCustomMirrorPlane() const noexcept;
		const bool hasCustomMirrorPlaneState() const noexcept;
		Standard_Boolean setMirrorPlaneOffset(
			Standard_Real theOffset) noexcept;
		Standard_Real mirrorPlaneOffset() const noexcept;
		std::pair<Standard_Real, Standard_Real>
			mirrorPlaneOffsetRange() const noexcept;
		//! Remove only the custom reference/preview and retain Mirror selection mode.
		Standard_Boolean resetMirrorPlane() noexcept;
		//! Discard only transient preview geometry while retaining Mirror mode.
		//! False means owned state remains and every caller must stop transitioning.
		Standard_Boolean clearTrialMirrorObjects() noexcept;
		const bool hasTrialMirrorObjects() const;
		const bool hasUnresolvedMirrorObjects() const;
		const bool hasActiveMirror() const noexcept;
		const bool canApplyMirror() const noexcept;
		MirrorPreviewState mirrorPreviewState() const noexcept;
		std::uint64_t mirrorPreviewGeneration() const noexcept;
#ifdef DEBUG
		MirrorPreviewDebugState debugMirrorPreviewState() const noexcept;
		void debugSetMirrorTransactionFailureCount(
			Standard_Size count) noexcept;
		void debugSetMirrorAbortFailureCount(Standard_Size count) noexcept;
		void debugSetMirrorEraseFailureCount(Standard_Size count) noexcept;
		void debugSetMirrorReferenceEraseFailureCount(
			Standard_Size count) noexcept;
		void debugSetMirrorCommitMode(Standard_Integer mode) noexcept;
		void debugSetMirrorPostCommitInspectFailureCount(
			Standard_Size count) noexcept;
		void debugSetMaximumMirrorTopologyNodes(
			Standard_Size limit) noexcept;
		void debugSetMaximumMirrorReferenceTopologyNodes(
			Standard_Size limit) noexcept;
		void debugSetMaximumMirrorReferenceFaces(
			Standard_Size limit) noexcept;
		Standard_Boolean
			debugMutateFirstMirrorSourcePersistedTransform() noexcept;
		//! Test the same custom-plane admission path without viewport projection.
		Standard_Boolean debugTryMirrorPlane(
			const std::string& theEntityIdentifier,
			Standard_Integer theFaceTopologyIndex,
			Standard_Real theOffset) noexcept;
#endif
		
		void setManipulator(Handle(Core3DManipulator) manipulator) {
			_manipulator = manipulator;
			_manipulatorGestureActive = false;
			_manipulatorSourceLabels.clear();
		}
		void setObjectTransparent(Handle(AIS_InteractiveObject) selected, const bool on);
		void detachManipulator(bool updateViewer);

    private:
        void createManipulatorIfNeeded();
        void attachManipulator(Handle(AIS_InteractiveObject) toObject);
		void detachManipulator(Handle(AIS_InteractiveObject) fromObject);
		Standard_Boolean tryMirrorImpl(
			Standard_Integer axisIndex,
			bool backward);
		Standard_Boolean tryMirrorWorldPlaneImpl(const gp_Ax2& theWorldPlane);
		struct MirrorPlaneReferenceSnapshot;
		Standard_Boolean captureMirrorPlaneReference(
			const Handle(AIS_Shape)& thePresentation,
			const TopoDS_Face& theFace,
			MirrorPlaneReferenceSnapshot& theSnapshot) const;
		Standard_Boolean completeMirrorPlanePick(
			MirrorPlaneReferenceSnapshot&& theSnapshot,
			Standard_Real theOffset);
		Standard_Boolean customMirrorPlaneIsCurrent(
			const MirrorPlaneReferenceSnapshot& theSnapshot,
			gp_Ax2& theWorldPlane,
			Standard_Real theOffset) const;
		Standard_Boolean restoreMirrorPlanePickingModes() noexcept;
		Standard_Boolean replaceMirrorReferencePresentation(
			const MirrorPlaneReferenceSnapshot& theSnapshot,
			Standard_Real theOffset) noexcept;
		Standard_Boolean clearMirrorReferencePresentation() noexcept;
		Standard_Boolean clearCustomMirrorPlaneState() noexcept;
		Standard_Boolean mirrorSourcesAreCurrent() const noexcept;
		Standard_Boolean abortOwnedMirrorCommand() noexcept;
		MirrorApplyResult finishCommittedMirror() noexcept;
		enum class MirrorDocumentState : std::uint8_t {
			None = 0,
			AllCommitted,
			OpenCommand,
			PartialOrMismatched,
			Unavailable,
		};
		MirrorDocumentState inspectPendingMirrorResults() const noexcept;
		
		void setSelectionTransparent(Handle(AIS_InteractiveObject) selected, const bool on);
		
    private:
		Standard_ShortReal _manipulatorSide;
		std::unordered_map<const AIS_InteractiveObject*, TDF_Label>
			_manipulatorSourceLabels;
		std::vector<Handle(AIS_Shape)> _trialMirrorObjects;
		struct MirrorSourceSnapshot {
			Handle(AIS_Shape) presentation;
			TDF_Label label;
			std::string entityIdentifier;
			std::string definitionIdentifier;
			TopoDS_Shape storedShape;
			gp_Trsf transform;
		};
		struct MirrorPendingResult {
			TDF_Label label;
			std::string entityIdentifier;
			std::string definitionIdentifier;
			TopoDS_Shape expectedShape;
		};
		struct MirrorPlaneReferenceSnapshot {
			Handle(TDocStd_Document) document;
			Handle(AIS_Shape) presentation;
			TDF_Label label;
			std::string entityIdentifier;
			std::string definitionIdentifier;
			OcctGeometryRepresentation representation =
				OcctGeometryRepresentation::Invalid;
			TopoDS_Shape storedShape;
			TopoDS_Face face;
			Standard_Integer faceTopologyIndex = -1;
			Standard_Size topologyNodeCount = 0;
			Standard_Size faceCount = 0;
			gp_Trsf presentationTransform;
			gp_Pnt worldOrigin;
			gp_Dir worldNormal;
			gp_Dir worldXDirection;
		};
		struct MirrorPlanePickSelectionModes {
			Handle(AIS_InteractiveObject) presentation;
			std::vector<Standard_Integer> modes;
		};
		std::vector<MirrorSourceSnapshot> _trialMirrorSources;
		std::vector<MirrorPendingResult> _pendingMirrorResults;
		std::optional<MirrorPlaneReferenceSnapshot> _customMirrorPlane;
		std::vector<Handle(AIS_Shape)> _mirrorReferencePresentations;
		std::vector<MirrorPlanePickSelectionModes> _mirrorPlanePickModes;
		Standard_Real _mirrorPlaneOffset = 0.0;
		bool _mirrorPlanePicking = false;
		bool _trialMirrorUsesCustomPlane = false;
		bool _trialMirrorObjectsValid = false;
		bool _mirrorOwnsDocumentCommand = false;
		MirrorPreviewState _mirrorPreviewState =
			MirrorPreviewState::Unavailable;
		std::uint64_t _mirrorPreviewGeneration = 0;
		bool _manipulatorGestureActive = false;
#ifdef DEBUG
		Standard_Size _debugMirrorTransactionFailureCount = 0;
		Standard_Size _debugMirrorAbortFailureCount = 0;
		Standard_Size _debugMirrorEraseFailureCount = 0;
		Standard_Size _debugMirrorReferenceEraseFailureCount = 0;
		Standard_Integer _debugMirrorCommitMode = 0;
		Standard_Size _debugMirrorPostCommitInspectFailureCount = 0;
		Standard_Size _debugMaximumMirrorTopologyNodes = 8'192;
		Standard_Size _debugMaximumMirrorReferenceTopologyNodes = 8'192;
		Standard_Size _debugMaximumMirrorReferenceFaces = 1'024;
#endif
    };
}
#endif /* Core3dObjectInteractor_hpp */
