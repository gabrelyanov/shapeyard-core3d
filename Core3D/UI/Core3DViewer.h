#include "../OCCTKit/SavedCutSourceDetachedWork.hxx"
#include "../OCCTKit/ProfileDefinition.hxx"
#include "../OCCTKit/PlanarSweepDefinition.hxx"
#include "../OCCTKit/RectangularLoftDefinition.hxx"
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
#include <gp_Pnt2d.hxx>
#include <optional>
#if DEBUG
#include <map>
#endif
#include <variant>
#include "../OCCTKit/NativeMeshElementSelection.hpp"

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

    //! Identity of a presentation-only Ready mirror framing request.
    //! Snapshot serials advance on capture; compare stable revision domains.
    struct MirrorPreviewFrameIdentity {
        std::string publicationSourceIdentifier;
        scene::RevisionVector revisions;
        std::uint64_t previewGeneration = 0;
    };

    struct NativePBRScalarWork;
    struct NativeSolidWork;
    class SavedCutSourceEditWork;
    class SavedCutSourceEditCancellation;
    class NativeModelingCommitPermit;
    struct ProfileSolidGeometry;
    struct EnclosureSolidGeometry;
    struct AssemblySolidGeometry;
    struct SweepSolidGeometry;
    struct LoftSolidGeometry;
    struct CutSolidGeometry;
    struct AssemblyPartDefinition {
        profile::Parameters parameters;
        TCollection_ExtendedString name;
        std::string identifier;
    };
    // Detached typed payload only; monostate is an invalid/unprepared request.
    using NativeSolidGeometryPayload = std::variant<std::monostate,
        std::shared_ptr<ProfileSolidGeometry>, std::shared_ptr<EnclosureSolidGeometry>,
        std::shared_ptr<AssemblySolidGeometry>,std::shared_ptr<SweepSolidGeometry>,std::shared_ptr<LoftSolidGeometry>,std::shared_ptr<CutSolidGeometry>>;
    struct CylindricalCutSnapshot {
        OcctCylindricalCutSource source;
        ObjectFrameIdentity identity;
        authority::Stamp authorityStamp;
        std::shared_ptr<const OcctSavedCutSceneState> guard;
    };
    struct StoredSweepSnapshot {
        planar_sweep::Definition definition;
        ObjectFrameIdentity identity;
        std::string definitionIdentifier,featureIdentifier;
        OcctObjectTransformState sourceState;
        authority::Stamp authorityStamp;
        std::shared_ptr<const SweepRebuildGuard> sourceGuard;
        double effectiveDimensionMetersPerUnit=0;
        bool current=false;
    };
    struct StoredRectangularLoftSnapshot {
        rectangular_loft::Definition definition;
        ObjectFrameIdentity identity;
        std::string definitionIdentifier,featureIdentifier;
        OcctObjectTransformState sourceState;
        authority::Stamp authorityStamp;
        std::shared_ptr<const SweepRebuildGuard> sourceGuard;
        double effectiveDimensionMetersPerUnit=0;
        bool current=false;
    };
    struct StoredProfileSnapshot {
        profile::Parameters parameters;
        ObjectFrameIdentity identity;
        std::string definitionIdentifier;
        std::string featureIdentifier;
        double dimensionMetersPerUnit = 0;
        bool current = false;
    };
    struct StoredEnclosureSnapshot {
        enclosure::Parameters parameters;
        ObjectFrameIdentity identity;
        std::string definitionIdentifier;
        std::string featureIdentifier;
        bool current = false;
        // Physical metres per recipe length, including both signed uniform scales.
        double dimensionMetersPerUnit = 0;
        // Main-thread-only native opening authority, never a detached worker or
        // provider payload. Retains exact root/binding, identity and transform
        // through the existing shared native object-state validation contract.
        OcctObjectTransformState sourceState;
    };
    struct MeshVertexEditWork;
    struct MeshVertexEditSnapshot {
        std::string sessionIdentifier;
        std::string entityIdentifier;
        std::vector<std::array<double,3>> worldVertices;
        meshedit::ElementKind elementKind = meshedit::ElementKind::Vertex;
        std::vector<std::array<std::uint32_t,2>> edgeVertices;
        std::vector<std::array<std::uint32_t,3>> triangleVertices;
    };
    struct DocumentReplacementWork;
    struct QueuedAssetLoadWork;
    struct ObjectAlignmentWork;
    struct ObjectAlignmentMeasurement;
    enum class ObjectAlignmentAnchor { Minimum, Center, Maximum, Ground, EqualCenters, EqualGaps };

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
        //! Typed creation requires exact document units and no caller-supplied
        //! construction frame. Stored rebuild has separate frame authority.
        std::shared_ptr<NativePBRScalarWork> preparePBRScalar(const OcctPBRScalarPatch&,
            const ObjectFrameIdentity&,std::uint64_t presentation,std::uint32_t width,std::uint32_t height) noexcept;
        OrdinaryEditResult executePBRScalar(const std::shared_ptr<NativePBRScalarWork>&) noexcept;
        void cancelPBRScalar(const std::shared_ptr<NativePBRScalarWork>&) noexcept;
        std::shared_ptr<NativeSolidWork> prepareProfileSolid(
            const profile::Parameters& parameters, const ObjectFrameIdentity& identity,
            std::uint64_t presentationRevision, std::uint32_t width, std::uint32_t height) noexcept;
        //! Polygon or exact circular-section construction: owned private worker geometry.
        std::shared_ptr<NativeSolidWork> prepareProfileSolid(
            const ProfileDefinition& definition, const ObjectFrameIdentity& identity,
            std::uint64_t presentationRevision, std::uint32_t width, std::uint32_t height) noexcept;
        std::shared_ptr<NativeSolidWork> prepareProfileSolid(
            const std::vector<gp_Pnt2d>& points, int plane, double depth,
            const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
            std::uint32_t width, std::uint32_t height, bool revolve = false,
            const std::optional<ProfileCircularSection>& circle = std::nullopt,
            const std::vector<ProfileCircularHole>& holes = {}) noexcept;
        //! Rebuild the selected current saved profile, retaining its entity and placement.
        std::shared_ptr<NativeSolidWork> prepareStoredProfileRebuild(
            double parameter, const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
            std::uint32_t width, std::uint32_t height) noexcept;
        std::optional<StoredProfileSnapshot> storedProfileDefinition(
            const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
            std::uint32_t width, std::uint32_t height) noexcept;
        std::shared_ptr<NativeSolidWork> prepareStoredProfileRebuild(
            const profile::Parameters& parameters, const StoredProfileSnapshot& original,
            const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
            std::uint32_t width, std::uint32_t height) noexcept;
        std::shared_ptr<NativeSolidWork> prepareNativeSolidWork(
            const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
            std::uint32_t width, std::uint32_t height) noexcept;
        std::shared_ptr<NativeSolidWork> prepareEnclosureSolid(
            const enclosure::Parameters& parameters, const ObjectFrameIdentity& identity,
            std::uint64_t presentationRevision, std::uint32_t width, std::uint32_t height) noexcept;
        std::optional<StoredEnclosureSnapshot> storedEnclosureDefinition(
            const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
            std::uint32_t width, std::uint32_t height) noexcept;
        std::shared_ptr<NativeSolidWork> prepareStoredEnclosureRebuild(
            const enclosure::Parameters& parameters, const StoredEnclosureSnapshot& original,
            const ObjectFrameIdentity& identity, std::uint64_t presentationRevision,
            std::uint32_t width, std::uint32_t height) noexcept;
        std::shared_ptr<NativeSolidWork> prepareAssemblySolid(
            const std::vector<AssemblyPartDefinition>& parts, const ObjectFrameIdentity& identity,
            std::uint64_t presentationRevision, std::uint32_t width, std::uint32_t height) noexcept;
        std::shared_ptr<NativeSolidWork> prepareLoftSolid(
            const rectangular_loft::Definition& definition,const ObjectFrameIdentity& identity,
            std::uint64_t presentationRevision,std::uint32_t width,std::uint32_t height) noexcept;
        std::shared_ptr<NativeSolidWork> prepareSweepSolid(
            const planar_sweep::Definition& definition,const ObjectFrameIdentity& identity,
            std::uint64_t presentationRevision,std::uint32_t width,std::uint32_t height) noexcept;
        std::optional<StoredSweepSnapshot> storedSweepDefinition(
            const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
            std::uint32_t width,std::uint32_t height) noexcept;
        std::shared_ptr<NativeSolidWork> prepareStoredSweepRebuild(
            const planar_sweep::Definition& definition,const StoredSweepSnapshot& original,
            const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
            std::uint32_t width,std::uint32_t height) noexcept;
        std::optional<CylindricalCutSnapshot> cylindricalCutSource(const ObjectFrameIdentity&,
        std::uint64_t,std::uint32_t,std::uint32_t) noexcept;
#if DEBUG
        std::map<std::string,bool> debugSavedCutSourceViewerQualification(const CylindricalCutSnapshot&,
            const ObjectFrameIdentity&,std::uint64_t,std::uint32_t,std::uint32_t);
    bool debugSetCutDisplayCoefficient(double coefficient,bool pending) noexcept;
#endif
    std::shared_ptr<NativeSolidWork> prepareCylindricalCut(const CylindricalCutSnapshot&,
        const std::optional<cylindrical_cut::CreateEdit>&,const std::optional<cylindrical_cut::RadiusEdit>&,
        const ObjectFrameIdentity&,std::uint64_t,std::uint32_t,std::uint32_t) noexcept;
    // Separate native-only source-edit lease. Main-thread authority never goes
    // to the geometry worker; no Objective-C or provider route is activated.
    static std::shared_ptr<SavedCutSourceEditWork> makeSavedCutSourceEditWork() noexcept;
    bool prepareSavedCutSourceEdit(const std::shared_ptr<SavedCutSourceEditWork>&,
        const CylindricalCutSnapshot&,const saved_cut_source_edit::Patch&,
        const ObjectFrameIdentity&,std::uint64_t,std::uint32_t,std::uint32_t) noexcept;
    static std::shared_ptr<SavedCutSourceDetachedWork> savedCutSourceEditGeometry(
        const std::shared_ptr<SavedCutSourceEditWork>&) noexcept;
    // Obtain before synchronous prepare. Only this token may cross threads.
    static std::shared_ptr<SavedCutSourceEditCancellation> savedCutSourceEditCancellation(
        const std::shared_ptr<SavedCutSourceEditWork>&) noexcept;
    // Thread-safe signal: token owns no document, AIS or main lease handles.
    static bool cancelSavedCutSourceEdit(const std::shared_ptr<SavedCutSourceEditCancellation>&) noexcept;
    bool discardSavedCutSourceEdit(const std::shared_ptr<SavedCutSourceEditWork>&) noexcept;
    OrdinaryEditResult commitSavedCutSourceEdit(const std::shared_ptr<SavedCutSourceEditWork>&,
        const std::shared_ptr<const SavedCutSourceDetachedResult>&) noexcept;
    // Detached-only source rebuild. Lifecycle owns work before synchronous prep.
    static std::shared_ptr<SavedCutSourceDetachedWork> makeSavedCutSourceDetachedWork() noexcept;
    bool prepareSavedCutSourceDetached(const std::shared_ptr<SavedCutSourceDetachedWork>&,
        const CylindricalCutSnapshot&,const saved_cut_source_edit::Patch&,
        const ObjectFrameIdentity&,std::uint64_t,std::uint32_t,std::uint32_t) noexcept;
    static std::shared_ptr<const SavedCutSourceDetachedResult> buildSavedCutSourceDetached(
        const std::shared_ptr<SavedCutSourceDetachedWork>&) noexcept;
    static void cancelSavedCutSourceDetached(const std::shared_ptr<SavedCutSourceDetachedWork>&) noexcept;
    std::optional<StoredRectangularLoftSnapshot> storedRectangularLoftDefinition(
            const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
            std::uint32_t width,std::uint32_t height) noexcept;
        std::shared_ptr<NativeSolidWork> prepareStoredLoftStationRebuild(
            const rectangular_loft::StationDimensionEdit& edit,const StoredRectangularLoftSnapshot& original,
            const ObjectFrameIdentity& identity,std::uint64_t presentationRevision,
            std::uint32_t width,std::uint32_t height) noexcept;
        static NativeSolidGeometryPayload nativeSolidGeometry(const std::shared_ptr<NativeSolidWork>& work) noexcept;
        static bool buildNativeSolidGeometry(const NativeSolidGeometryPayload& payload) noexcept;
        static std::shared_ptr<ProfileSolidGeometry> profileSolidGeometry(
            const std::shared_ptr<NativeSolidWork>& work) noexcept;
        static bool buildProfileSolidGeometry(const std::shared_ptr<ProfileSolidGeometry>& geometry) noexcept;
        static void cancelNativeSolid(const std::shared_ptr<NativeSolidWork>& work) noexcept;
        static bool attachModelingRebuildPermit(const std::shared_ptr<NativeSolidWork>& work,
            std::shared_ptr<NativeModelingCommitPermit> permit) noexcept;
        static bool attachModelingCreationPermit(const std::shared_ptr<NativeSolidWork>& work,
            std::shared_ptr<NativeModelingCommitPermit> permit) noexcept;
        OrdinaryEditResult commitNativeSolid(const std::shared_ptr<NativeSolidWork>& work) noexcept;
        
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
        //! Conservative semantic transition fence, including failed/no-op
        //! selection and tool attempts. Camera-only publication does not call it.
        void observeNativePlanningInteraction() noexcept;
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
        void debugSetOrdinaryCreationAfterRepairFailures(int count) noexcept {
            _debugOrdinaryCreationAfterRepairFailures = count > 0 ? count : 0;
        }
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
        std::optional<MeshVertexEditSnapshot> prepareMeshVertexEdit(const ObjectFrameIdentity& identity,
            std::uint64_t presentationRevision,std::uint32_t width,std::uint32_t height) noexcept;
        OrdinaryEditResult commitMeshVertexEdit(const std::string& sessionIdentifier,
            const std::vector<std::uint32_t>& vertices,const gp_Vec& worldDelta) noexcept;
        std::optional<MeshVertexEditSnapshot> prepareMeshElementEdit(const ObjectFrameIdentity& identity,
            std::uint64_t presentationRevision,std::uint32_t width,std::uint32_t height,
            meshedit::ElementKind kind) noexcept;
        OrdinaryEditResult commitMeshElementEdit(const std::string& sessionIdentifier,
            meshedit::ElementKind kind,const std::vector<std::uint32_t>& elements,
            const gp_Vec& worldDelta) noexcept;
        void cancelMeshVertexEdit(const std::string& sessionIdentifier) noexcept;
        OrdinaryEditResult createSourceRetainedMeshCopy(const ObjectFrameIdentity& identity,
            std::uint32_t width, std::uint32_t height) noexcept;
        std::optional<OcctMeshUVAtlasPreview> previewCoherentUVAtlas(const ObjectFrameIdentity& identity,
            std::uint32_t width, std::uint32_t height,
            const std::optional<OcctMeshUVAtlasOptions>& options) noexcept;
        OrdinaryEditResult repairMeshWinding(const ObjectFrameIdentity& identity,
            std::uint64_t presentationRevision, std::uint32_t width, std::uint32_t height) noexcept;
        OrdinaryEditResult generateTriangleUVAtlas(const ObjectFrameIdentity& identity,
            std::uint32_t width, std::uint32_t height, const OcctMeshUVAtlasOptions& options = {}) noexcept;
        OrdinaryEditResult renameObjectFromBrowser(const ObjectFrameIdentity& identity,
            const TCollection_ExtendedString& name, std::uint32_t viewportWidth,
            std::uint32_t viewportHeight) noexcept;
#ifdef DEBUG
        void debugSetOrdinaryRepairFailures(int incremental, int redraw) noexcept {
            _debugOrdinaryRepairFailures = incremental;
            _debugOrdinaryRedrawFailures = redraw;
            _debugOrdinaryRedrawAttempts = 0;
        }
        bool debugProbeMeshVertexStorageChange(int mode) noexcept;
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
        // Internal accepted-request owner, held through staging cleanup on main.
        std::shared_ptr<QueuedAssetLoadWork> beginQueuedAssetLoad() noexcept;
        bool ownsQueuedAssetLoad(const std::shared_ptr<QueuedAssetLoadWork>& work) const noexcept;
        bool canAdoptQueuedAssetLoad(const std::shared_ptr<QueuedAssetLoadWork>& work) noexcept;
        bool finishQueuedAssetLoadPrivateWork(const std::shared_ptr<QueuedAssetLoadWork>& work,
                                             bool privateWorkSettled) noexcept;
        AssetImportResult ImportCbf(const std::string& theFilename,
                                    const std::shared_ptr<QueuedAssetLoadWork>& work);
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
                const TransformInspectorPositionCommitRequest& request,
                std::shared_ptr<NativeModelingCommitPermit> placementPermit = {})
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
        //! Fit selected committed sources plus an exact Ready axis preview.
        //! Never changes the document, selection, operation or model history.
        bool frameMirrorPreview(const MirrorPreviewFrameIdentity& expected,
                        std::uint32_t viewportWidth, std::uint32_t viewportHeight,
                        double targetX, double targetY,
                        double targetWidth, double targetHeight) noexcept;

        //! Select one exact, visible, editable object from a current browser
        //! lease. Keeps camera and model history; updates the selection gizmo.
        bool selectObjectFromBrowser(const ObjectFrameIdentity& identity,
                                     std::uint32_t viewportWidth,
                                     std::uint32_t viewportHeight,
                                     bool& selectionWasTouched) noexcept;

        //! Capture committed OCAF geometry and semantic camera state into
        //! immutable renderer-neutral values. Main-thread only.
        //! Read-only native mesh capture for one explicit occurrence. The
        //! committed-scene barriers apply; selection is never retargeted.
        meshcheck::ContactSourceStatus captureNativeMeshContacts(
            const meshcheck::ContactSourceIdentity& expected,
            std::uint32_t width, std::uint32_t height,
            const std::atomic_bool& cancelled,
            meshcheck::ContactSourceCapture& output) noexcept;
        meshcheck::ContactSourceStatus validateNativeMeshContacts(
            const meshcheck::ContactSourceCapture& original,
            std::uint32_t width, std::uint32_t height,
            const std::atomic_bool& cancelled) noexcept;

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
        Standard_Boolean debugCycleBooleanSelection(BooleanAction action,
            const std::string& entity, bool beginEmpty) noexcept;
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
        void DebugSetBooleanMetadataFailurePhase(
            Standard_Size phase) noexcept;
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
        //! Fail once after provisional document assignment, before adoption.
        void DebugFailNextDocumentAdoption() noexcept { _debugFailNextDocumentAdoption = true; }
        void DebugSetDocumentReplacementFaults(bool preparation, int restorationAttempts) noexcept {
            _debugFailNextDocumentPreparation = preparation;
            _debugDocumentRestorationFailures = restorationAttempts < 0 ? 0 : restorationAttempts > 100 ? 100 : restorationAttempts;
        }

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
        // Weak slot cannot keep a dropped main-thread lease or scene alive.
        std::weak_ptr<SavedCutSourceEditWork> _savedCutSourceEditWork;
        std::shared_ptr<MeshVertexEditWork> _meshVertexEditWork;
        std::shared_ptr<DocumentReplacementWork> _documentReplacementWork;
        std::shared_ptr<QueuedAssetLoadWork> _queuedAssetLoadWork;
        bool hasUnresolvedOrdinaryEditExcludingQueuedLoad() const noexcept;
        bool restoreDocumentReplacement(bool afterImportFailure = false) noexcept;
#ifdef DEBUG
        bool _debugFailNextDocumentAdoption = false;
        bool _debugFailNextDocumentPreparation = false;
        int _debugDocumentRestorationFailures = 0;
        int _debugOrdinaryRepairFailures = 0;
        int _debugOrdinaryCreationAfterRepairFailures = 0;
        int _debugOrdinaryVisibilityAfterRepairFailures = 0;
        int _debugOrdinaryRedrawFailures = 0;
        int _debugOrdinaryOwnerResolutionFailures = 0;
        int _debugOrdinaryRedrawAttempts = 0;
#endif
        bool admitTransform(OrdinaryTransformLedger& ledger) noexcept override;
        bool admitVisibility(OrdinaryVisibilityLedger& ledger) noexcept override;
        bool repairVisibility(const OrdinaryVisibilityLedger& ledger, bool committed) noexcept override;
        OrdinaryEditResult publishCreatedPrimitives(const std::vector<OrdinaryCreationRequest>& requests,
            std::shared_ptr<NativeModelingCommitPermit> permit = {}) noexcept;
        bool admitCreation(OrdinaryCreationLedger& ledger) noexcept override;
        bool admitMeshCopy(OrdinaryCreationLedger& ledger) noexcept override;
        bool repairMeshCopy(const OrdinaryCreationLedger& ledger, bool committed) noexcept override;
        bool repairCreation(const OrdinaryCreationLedger& ledger, bool committed) noexcept override;
        bool admitAppearance(OrdinaryAppearanceLedger&) noexcept override;
        bool repairAppearance(const OrdinaryAppearanceLedger&,bool) noexcept override;
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
