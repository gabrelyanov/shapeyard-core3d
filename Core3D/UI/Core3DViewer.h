//
//  Core3DViewer.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 11.04.2024.
//

#ifndef Core3DViewer_H
#define Core3DViewer_H

#include "OcctViewer.h"
#include "AIS_Manipulator.hxx"

#import <Core3D/PrimitiveType.h>

#include "ObjectInteractor.hpp"
#include "ShapeInteractor.hpp"
#include "TransformInspectorMeasurementController.hpp"
#include "OrdinaryEditController.hpp"

#include "OrthoProjectionType.h"
#include "../Scene/OcctSceneSnapshotBuilder.hpp"

namespace core3d {
    typedef unsigned char selection_t;

    enum class AssetImportResult {
        Success = 0,
        InvalidData,
        TemporaryFileFailure,
        Busy,
        UnsupportedVersion,
        InternalFailure,
    };

    struct ObjectFrameIdentity {
        std::string entityIdentifier;
        std::string publicationSourceIdentifier;
        std::uint64_t documentGeneration = 0;
        std::uint64_t modelRevision = 0;
    };

    struct ObjectAlignmentWork;
    struct ObjectAlignmentMeasurement;
    enum class ObjectAlignmentAnchor { Minimum, Center, Maximum, Ground };

    class Core3DViewer: public OcctViewer, private OrdinaryEditPresentationHost {
    public:
        static constexpr selection_t kSelectionTypeNone = 0;
        static constexpr selection_t kSelectionTypeManipulator = 1 << 0;
        static constexpr selection_t kSelectionTypeObject = 1 << 1;

        //! Release derived interactors before the base OCCT graphics handles.
        Standard_EXPORT void release() noexcept;

        Standard_EXPORT bool InitViewer (UIView* theWin);
        
        Standard_EXPORT NSString* addTestPrimitives();
        void addPrimitive(PrimitiveType primitiveType);
        void addPrimitivesFromJSON(NSString* json);
        
        void showGrid(bool show);
        
        void redraw();
        
        /***
         Interactions
         */
        void Select(int theX, int theY);
        void StartRotation(int theX, int theY);
        void Rotation(int theX, int theY);
        void FinishInteraction(int theX, int theY);
        void CancelInteraction(int theX, int theY);
        
        const bool hitTest(const int x, const int y) const;

        void deselectAll();
		const int selectedCount() const;

        std::shared_ptr<ObjectInteractor> getObjectInteractor();
        std::shared_ptr<ShapeInteractor> getShapeInteractor();
        Handle(OcctDocument) getDocument();
        //! True only while no typed operation owns a command, preview, or
        //! exactly-once recovery ledger and the OCAF document is writable.
        bool canBeginCommittedEdit() const noexcept;
        //! Duplicate presentation repair is intentionally exposed separately:
        //! committed snapshots and project serialization must fail closed while
        //! its result ledger remains the sole recovery authority.
        bool hasUnresolvedDuplicate() const noexcept;
        bool hasUnresolvedOrdinaryEdit() const noexcept;
        bool hasUnresolvedEdit() const noexcept;
        OrdinaryEditLease beginOrdinaryTransform(const std::vector<OrdinaryTransformChange>& changes,
                                                 OrdinaryEditResult* failure = nullptr) noexcept;
        OrdinaryEditResult reconcileOrdinaryEdit() noexcept;
        //! Main-thread capture/commit; measurement accesses only private copies.
        std::shared_ptr<ObjectAlignmentWork> prepareObjectAlignment(
            int axis, ObjectAlignmentAnchor anchor, const ObjectFrameIdentity& identity,
            std::uint64_t presentationRevision, std::uint32_t width, std::uint32_t height) noexcept;
        static std::shared_ptr<ObjectAlignmentMeasurement> objectAlignmentMeasurement(
            const std::shared_ptr<ObjectAlignmentWork>& work) noexcept;
        static bool measureObjectAlignment(const std::shared_ptr<ObjectAlignmentMeasurement>& measurement) noexcept;
        static void cancelObjectAlignment(const std::shared_ptr<ObjectAlignmentWork>& work) noexcept;
        OrdinaryEditResult commitObjectAlignment(const std::shared_ptr<ObjectAlignmentWork>& work) noexcept;
        OrdinaryEditResult setObjectVisibilityFromBrowser(const ObjectFrameIdentity& identity,
            bool visible, std::uint64_t presentationRevision, std::uint32_t viewportWidth,
            std::uint32_t viewportHeight, bool* blockedByLayer = nullptr) noexcept;
#ifdef DEBUG
        void debugSetOrdinaryVisibilityAfterRepairFailures(int count) noexcept {
            _debugOrdinaryVisibilityAfterRepairFailures = count > 0 ? count : 0;
        }
#endif
        //! Operations: 0 create, 1 rename, 2 ungroup, 3 hide, 4 show.
        OrdinaryEditResult editSavedGroup(int operation, const std::string& groupIdentifier,
            const std::vector<std::string>& entities, const TCollection_ExtendedString& name,
            const ObjectFrameIdentity& expected, std::uint64_t presentationRevision,
            std::uint32_t width, std::uint32_t height, bool* blockedByLayer = nullptr) noexcept;
        bool selectSavedGroup(const ObjectFrameIdentity& expected, std::uint64_t presentationRevision,
            std::uint32_t width, std::uint32_t height, bool& selectionWasTouched) noexcept;
        OrdinaryEditResult renameObjectFromBrowser(const ObjectFrameIdentity& identity,
            const TCollection_ExtendedString& name, std::uint32_t viewportWidth,
            std::uint32_t viewportHeight) noexcept;
#ifdef DEBUG
        void debugSetOrdinaryRepairFailures(int incremental, int redraw) noexcept {
            _debugOrdinaryRepairFailures = incremental;
            _debugOrdinaryRedrawFailures = redraw;
            _debugOrdinaryRedrawAttempts = 0;
        }
        int debugOrdinaryRedrawAttempts() const noexcept { return _debugOrdinaryRedrawAttempts; }
        std::shared_ptr<OrdinaryEditController> debugOrdinaryEditController() const noexcept {
            return _ordinaryEditController;
        }
#endif
        bool dumpOfDisplayedColoredObjects(const Standard_Integer width,
                                           const Standard_Integer height,
                                           const TCollection_AsciiString& fileName);
        bool dumpOfDisplayedObjects(const Standard_Integer width,
                                    const Standard_Integer height,
                                    const TCollection_AsciiString& fileName);
        bool dumpShape(const TopoDS_Shape& shape,
                         const Standard_Integer width,
                         const Standard_Integer height,
                         const TCollection_AsciiString& fileName);
        bool saveSnapshot(const TCollection_AsciiString& thePath,
                            int theWidth,
                            int theHeight);
        
        AssetImportResult ImportCbf(const std::string &theFilename);
        AssetImportResult ValidateCbf(const std::string &theFilename) const;
        //! Rebuild presentations from authoritative OCAF. If traversal or
        //! interactor recreation fails after clearing the context, restore the
        //! retained pre-redraw AIS handles instead of leaving a blank/partial
        //! viewport. Returns true only for a complete OCAF rebuild.
        bool redrawDocument() noexcept;

        void setPreviewMode();
        inline void setInteractiveCallback(const std::function<void(int,int)> cb) {
            _interactiveCallback = cb;
        }
        void setBooleanPreviewStateChangedCallback(
            std::function<void()> callback);
        void setLinearArrayPreviewStateChangedCallback(
            std::function<void()> callback);
        void setRadialArrayPreviewStateChangedCallback(
            std::function<void()> callback);
        void setBevelPreviewStateChangedCallback(
            std::function<void()> callback);
        void setShellPreviewStateChangedCallback(
            std::function<void()> callback);
        //! Synchronously mirrors the actual native tool after interactor
        //! recreation, before a caller can publish a selection callback.
        void setInteractorRecreatedCallback(
            std::function<void(PrimitiveManipulatorType)> callback);

        //! Capture the document-authoritative single-selection transform and
        //! hybrid exact local bounds. A BRep cache miss returns Measuring and
        //! may deliver one terminal main-thread completion.
        TransformInspectorMeasurement captureTransformInspectorMeasurement(
            TransformInspectorMeasurementCompletion completion = {}) noexcept;
        TransformInspectorPositionCommitResult
            commitTransformInspectorPosition(
                const TransformInspectorPositionCommitRequest& request)
                noexcept;
        //! Suppress any pending transform-inspector completion. Exact worker
        //! work already inside OCCT may still populate its bounded cache.
        void cancelTransformInspectorMeasurement() noexcept;
        TransformInspectorMeasurementPerformanceState
            transformInspectorMeasurementPerformanceState() const noexcept;

        void setOrthoProjection(const OrthoProjectionType orthoType);
        //! Change projection while retaining target, direction and apparent
        //! scale at the target plane. Never opens an OCAF command.
        bool setCameraOrthographic(bool orthographic) noexcept;
        //! Camera-only framing from validated committed scene bounds. Selected
        //! framing requires Object mode. No meshing, OCAF command, or selection
        //! mutation is performed. Failure preserves the previous camera.
        bool frameModel(bool selectedObjectsOnly, std::uint32_t viewportWidth,
                        std::uint32_t viewportHeight,
                        double targetX = 0.0, double targetY = 0.0,
                        double targetWidth = 1.0, double targetHeight = 1.0,
                        const ObjectFrameIdentity* objectIdentity = nullptr) noexcept;

        //! Select one exact, visible, editable object from a current browser
        //! lease. Keeps camera and model history; updates the selection gizmo.
        bool selectObjectFromBrowser(const ObjectFrameIdentity& identity,
                                     std::uint32_t viewportWidth,
                                     std::uint32_t viewportHeight,
                                     bool& selectionWasTouched) noexcept;

        //! Capture committed OCAF geometry and semantic camera state into
        //! immutable renderer-neutral values. Main-thread only.
        scene::OcctSceneSnapshotBuilder::SnapshotPointer captureSceneSnapshot(
            std::uint32_t viewportWidth,
            std::uint32_t viewportHeight) noexcept;
        //! True only when the active typed operation owns every deviation from
        //! the retained selection mode. Ordinary GL selection authority stays
        //! strict and OutcomeUnknown scene publication remains blocked.
        bool selectionModeAuthorityAllowsRetainedOperation()
            const noexcept;

        //! Capture only the current semantic camera and established revision
        //! vector. Main-thread only and constant with respect to mesh size.
        std::optional<scene::FrameSnapshot> captureSceneFrameSnapshot(
            std::uint32_t viewportWidth,
            std::uint32_t viewportHeight) noexcept;

        //! Capture an immutable idle tool presentation paired with the most
        //! recent full scene. Ready Bevel and Shell previews replace their
        //! committed sources; empty generic content is a valid explicit clear.
        scene::OcctSceneSnapshotBuilder::OverlayPointer
        captureScenePresentationOverlay() noexcept;
#ifdef DEBUG
        //! Select exactly one presentation whose modes are deliberately
        //! suspended by the active typed operation. This is a test-only seam
        //! for exercising the production selection callback; it never admits
        //! an unrelated displayed shape.
        Standard_Boolean
            DebugSelectRetainedOperationPresentation() noexcept;
		//! Publish one deterministic DEBUG-only detected owner without running
		//! view-space picking. The owner is consumed by the production snapshot
		//! builder exactly as an AIS MoveTo result would be.
		Standard_Boolean DebugSetDetectedOwner(
			const Handle(SelectMgr_EntityOwner)& owner) noexcept;
        Standard_Boolean debugBeginBooleanSelection(
            BooleanAction action,
            const std::vector<std::string>& actorEntityIdentifiers,
            const std::vector<std::string>& subjectEntityIdentifiers) noexcept;
        BooleanPreviewDebugState DebugBooleanPreviewState() const noexcept;
        void DebugSetBooleanPreviewWorkerBlocked(
            Standard_Boolean blocked) noexcept;
        void DebugSetMaximumBooleanCaptureTopologyNodes(
            Standard_Size limit) noexcept;
        void DebugSetMaximumBooleanResultTopologyNodes(
            Standard_Size limit) noexcept;
        void DebugSetMaximumBooleanResultSolids(
            Standard_Size limit) noexcept;
        void DebugSetBooleanTransactionFailureCount(
            Standard_Size count) noexcept;
        void DebugSetBooleanAbortFailureCount(
            Standard_Size count) noexcept;
        void DebugSetBooleanPostCommitInspectFailureCount(
            Standard_Size count) noexcept;
        Standard_Boolean debugBeginExtrusionSelection(
            const std::string& entityIdentifier,
            Standard_Size faceTopologyIndex) noexcept;
        Standard_Boolean debugBeginShellSelection(
            const std::string& entityIdentifier,
            Standard_Size faceTopologyIndex) noexcept;
        ShellPreviewDebugState DebugShellPreviewState() const noexcept;
        void DebugSetShellPreviewWorkerBlocked(
            Standard_Boolean blocked) noexcept;
        void DebugSetMaximumShellCaptureTopologyNodes(
            Standard_Size limit) noexcept;
        void DebugSetMaximumShellResultTopologyNodes(
            Standard_Size limit) noexcept;
        void DebugSetShellTransactionFailureCount(
            Standard_Size count) noexcept;
        void DebugSetShellAbortFailureCount(
            Standard_Size count) noexcept;
        void DebugSetShellPreviewEraseFailureCount(
            Standard_Size count) noexcept;
        void DebugSetShellCommitMode(Standard_Integer mode) noexcept;
        void DebugSetShellPostCommitInspectFailureCount(
            Standard_Size count) noexcept;
        Standard_Boolean
            DebugMutateShellSourcePersistedTransform() noexcept;
        Standard_Boolean
            DebugMutateShellSourcePersistedShape() noexcept;
        Standard_Boolean debugBeginBevelSelection(
            const std::string& entityIdentifier,
            const std::vector<Standard_Size>& edgeTopologyIndices) noexcept;
        Standard_Boolean debugBeginBevelSelection(
            const std::vector<std::string>& entityIdentifiers,
            const std::vector<std::vector<Standard_Size>>&
                edgeTopologyIndices) noexcept;
        BevelPreviewDebugState DebugBevelPreviewState() const noexcept;
        void DebugSetBevelPreviewWorkerBlocked(
            Standard_Boolean blocked) noexcept;
        void DebugSetMaximumBevelCaptureTopologyNodes(
            Standard_Size limit) noexcept;
        void DebugSetMaximumBevelResultTopologyNodes(
            Standard_Size limit) noexcept;
        void DebugSetMaximumBevelResultSolids(
            Standard_Size limit) noexcept;
        void DebugSetBevelTransactionFailureCount(
            Standard_Size count) noexcept;
        void DebugSetBevelCancelDiscardFailureCount(
            Standard_Size count) noexcept;
        Standard_Boolean
            DebugMutateFirstBevelSourcePersistedTransform() noexcept;
        //! Test-only admission ceiling for the bounded project-load topology
        //! walk. Production uses the fixed mobile-safe aggregate ceiling.
        void SetDebugMaximumProjectTopologyValidationNodes(
            const Standard_Size limit)
        {
            myMaximumProjectTopologyValidationNodes = limit > 0 ? limit : 1;
        }
        //! Runtime proof that project publication used the bounded structural
        //! walk and did not enter Core3DViewer's geometric BRep checker.
        void DebugResetProjectTopologyValidationCounters() const;
        Standard_Size DebugBoundedProjectTopologyValidationCount() const;
        Standard_Size DebugGeometricBRepValidationCount() const;
        void DebugSetSceneSnapshotTriangulationFailure(
            scene::OcctSceneSnapshotBuilder::DebugTriangulationFailure
                theFailure) noexcept;
        void DebugResetSceneSnapshotMesherInvocationCount() noexcept;
        std::uint64_t DebugSceneSnapshotMesherInvocationCount() const noexcept;
        std::uint64_t DebugPublishedDocumentGeneration() const noexcept;
        std::uint64_t DebugPublishedModelRevision() const noexcept;
        TransformInspectorMeasurementDebugState
            DebugTransformInspectorMeasurementState() const noexcept;
        void DebugSetTransformInspectorWorkerBlocked(
            Standard_Boolean blocked) noexcept;
        void DebugSetTransformInspectorForcedMeasurementFailure(
            Standard_Boolean failure) noexcept;
        void DebugSetMaximumTransformInspectorBRepTopologyNodes(
            Standard_Size limit) noexcept;
        void DebugSetMaximumTransformInspectorTriangleMeshSweepNodes(
            Standard_Size limit) noexcept;
        void DebugSetTransformInspectorMeshSweepWatchdog(
            Standard_Real deadlineMilliseconds,
            Standard_Size pollNodes) noexcept;
        void DebugSetTransformInspectorPositionCommitMode(
            Standard_Integer mode) noexcept;
        //! One-shot publication fallback: 0 normal, 1 forces incremental
        //! publication failure with a successful OCAF redraw, 2 also forces
        //! redraw traversal failure to exercise retained AIS restore, and 3
        //! forces the post-redraw exact-owner restoration to miss.
        void DebugSetTransformInspectorPositionPublicationFallbackMode(
            Standard_Integer mode) noexcept;
#endif
    private:
        // document traversal
        bool traverseDocument (const Handle(TDocStd_Document)& theDoc);
        bool traverseLabel (const Handle(TDocStd_Document)& theDoc,
                            const TDF_Label& theLabel,
                                            const TCollection_AsciiString& theNamePrefix,
                                            const TopLoc_Location& theLoc,
                                            MapOfPrsForShapes& theMapOfShapes);
        bool recreateInteractors(PrimitiveManipulatorType theManipulatorType,
                                 ShapeSelectionMode theSelectionMode);
        bool recreateFreshInteractorsForDocumentReplacement();
        bool publishRecreatedInteractorState() noexcept;
        Standard_Boolean captureSelectionModeSuspendedPresentations(
            std::vector<Handle(AIS_Shape)>& presentations) const noexcept;

    private:
        std::shared_ptr<OrdinaryEditController> _ordinaryEditController;
#ifdef DEBUG
        int _debugOrdinaryRepairFailures = 0;
        int _debugOrdinaryVisibilityAfterRepairFailures = 0;
        int _debugOrdinaryRedrawFailures = 0;
        int _debugOrdinaryOwnerResolutionFailures = 0;
        int _debugOrdinaryRedrawAttempts = 0;
#endif
        bool admitTransform(OrdinaryTransformLedger& ledger) noexcept override;
        bool admitVisibility(OrdinaryVisibilityLedger& ledger) noexcept override;
        bool repairVisibility(const OrdinaryVisibilityLedger& ledger, bool committed) noexcept override;
        bool admitGrouping(OrdinaryGroupingLedger& ledger) noexcept override;
        bool repairGrouping(const OrdinaryGroupingLedger& ledger, bool committed) noexcept override;
        bool admitNames(OrdinaryNameLedger& ledger) noexcept override;
        bool repairNames(const OrdinaryNameLedger& ledger, bool committed) noexcept override;
        bool repairTransform(const OrdinaryTransformLedger& ledger, bool committed) noexcept override;
        bool rebuildTransform(const OrdinaryTransformLedger& ledger, bool committed,
                              std::vector<Handle(AIS_Shape)>& replacements) noexcept override;
        std::shared_ptr<ObjectInteractor> _objectInteractor;
        std::shared_ptr<ShapeInteractor> _shapeInteractor;
        std::shared_ptr<TransformInspectorMeasurementController>
            _transformInspectorMeasurementController;
        
        std::function<void(int,int)> _interactiveCallback;
        std::function<void()> _booleanPreviewStateChangedCallback;
        std::function<void()> _linearArrayPreviewStateChangedCallback;
        std::function<void()> _radialArrayPreviewStateChangedCallback;
        std::function<void()> _bevelPreviewStateChangedCallback;
        std::function<void()> _shellPreviewStateChangedCallback;
        std::function<void(PrimitiveManipulatorType)>
            _interactorRecreatedCallback;
        scene::OcctSceneSnapshotBuilder _sceneSnapshotBuilder;
#ifdef DEBUG
        Standard_Integer
            _debugTransformInspectorPositionPublicationFallbackMode = 0;
        Standard_Boolean
            _debugForceNextTransformInspectorRedrawFailure = Standard_False;
#endif
        Standard_Size myMaximumProjectTopologyValidationNodes = 2'000'000;
    };
}

#endif // Core3DViewer_H
