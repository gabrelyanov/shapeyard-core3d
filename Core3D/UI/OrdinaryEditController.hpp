#ifndef OrdinaryEditController_hpp
#define OrdinaryEditController_hpp
#include <gp_Ax2.hxx>

#include "OrdinaryEditCommand.hpp"
#include "../OCCTKit/NativeModelingReceipt.hxx"
#include "../OCCTKit/RectangularLoftRebuild.hxx"
#include "NativeModelingRequest.hxx"
#include "../OCCTKit/NativeRigidPlacementEvidence.hxx"
#include "../OCCTKit/SavedCutSourceDetachedWork.hxx"
#include "../OCCTKit/SavedProgramSourceDetachedWork.hxx"
#include "../OCCTKit/RetainedBooleanEditValues.hxx"
#include <SelectMgr_EntityOwner.hxx>
#include <memory>
#include <map>
#include <Graphic3d_NameOfMaterial.hxx>
#include <Quantity_NameOfColor.hxx>
#include <optional>
#include <variant>
#include <vector>

namespace core3d {
enum class PrimitiveManipulatorType;
enum class ShapeSelectionMode;

enum class OrdinaryEditKind : std::uint8_t { Transform, Add, Remove, Appearance, Name, Visibility, Grouping };
enum class OrdinaryEditState : std::uint8_t { Idle, OpenOwned, OutcomeUnknown, RepairPending, Publishing };
enum class OrdinaryEditResult : std::uint8_t { NoChange, Committed, RetryableFailure, OutcomeUnknown, Busy, Invalid };
enum class OrdinaryTransformOperation : std::uint8_t { Translate, Rotate, Scale, MeshUVAtlas, MeshVertexMove, MeshWindingRepair, ProfileRebuild, EnclosureRebuild, SweepRebuild, LoftStationRebuild, CylindricalCut, CylindricalCutSourceRebuild, CylindricalCutProgramSourceRebuild };

// Main-only proof lifetime. No callbacks, app objects or worker-captured handles.
// Only the ordinary controller can seal this result; an unresolved ledger retains it.
class NativeModelingReceiptResolution final {
public:
    enum class State { Pending, Committed, Aborted, Unchanged };
    State state() const noexcept { return state_; }
    const receipt::Record& record() const noexcept { return record_; }
private:
    friend class OrdinaryEditController;
    State state_=State::Pending;
    receipt::Record record_;
};
struct NativeModelingEpoch final { bool retired=false; }; // read/write main only
struct NativeModelingPermitIssuer; // Defined solely at the owner reservation boundary.
class NativeModelingCommitPermit final {
public:
    NativeModelingCommitPermit(const NativeModelingCommitPermit&)=delete;
    NativeModelingCommitPermit& operator=(const NativeModelingCommitPermit&)=delete;
    bool current() const noexcept {
        return session_&&request_&&!session_->retired&&!request_->retired;
    }
    const std::shared_ptr<NativeModelingReceiptResolution>& resolution() const noexcept { return resolution_; }
private:
    friend struct NativeModelingPermitIssuer;
    friend class Core3DViewer;
    friend class OrdinaryEditController;
    NativeModelingCommitPermit()=default;
    receipt::Key key_;
    receipt::Operation operation_=receipt::Operation::CreateEnclosure;
    request::Descriptor descriptor_;
    std::vector<receipt::UUID> featureIDs_;
    Handle(TDocStd_Document) document_;
    receipt::Catalog previous_;
    std::optional<placement::Evidence> expectedPlacement_;
    std::optional<receipt::Effect> expectedSource_; // Rebuild-only, main-owned native proof.
    std::shared_ptr<NativeModelingEpoch> session_,request_;
    std::shared_ptr<NativeModelingReceiptResolution> resolution_;
    std::optional<request::Admission> admission_;
    bool attached_=false,admitted_=false;
#if DEBUG
    bool debugStageFailure_=false;
    bool debugBeforeReleaseFailure_=false;
#endif
};
struct OrdinaryModelingReceiptLedger {
    std::shared_ptr<NativeModelingCommitPermit> permit;
    receipt::Catalog candidate;
    receipt::Record record;
    bool staged=false;
};

//! Rotation of a selection around an explicit world-space point, in model
//! units. delta must be a unit-scale rigid transform that fixes the pivot.
struct OrdinaryRotationAroundPivot {
    gp_Pnt pivot;
    gp_Trsf delta;
};

//! Session-local logical vertices and a world-space vector in document mm.
struct OrdinaryMeshVertexMove {
    std::vector<std::uint32_t> vertices;
    gp_Vec worldDelta;
};

// Minted only after the inspector validates its exact original lease and
// computes the candidate through the shared touch calculation. Ordinary
// admission compares the descriptor, baseline and matrix before opening OCAF.
class NativePlacementContinuation final {
    friend class TransformInspectorMeasurementController;
    friend class OrdinaryEditController;
    NativePlacementContinuation()=default;
    request::Descriptor descriptor_;
    gp_Trsf candidate_;
    TopoDS_Shape shape_;
    TDF_Label label_;
};
struct SweepRebuildGuard; // Opaque immutable source catalog, main only.
struct OrdinaryTransformChange {
    TDF_Label label;
    Handle(AIS_Shape) presentation;
    TopoDS_Shape shape;
    gp_Trsf transform;
    OrdinaryTransformOperation operation = OrdinaryTransformOperation::Translate;
    std::optional<OrdinaryRotationAroundPivot> rotationAroundPivot;
    OcctMeshUVAtlasOptions meshUVAtlasOptions;
    std::optional<OrdinaryMeshVertexMove> meshVertexMove;
    std::shared_ptr<const NativePlacementContinuation> placementContinuation;
    std::optional<profile::Parameters> profileRebuild;
    std::optional<enclosure::Parameters> enclosureRebuild;
    std::optional<planar_sweep::Definition> sweepRebuild;
    std::optional<rectangular_loft::Definition> loftRebuild;
    std::optional<rectangular_loft::StationDimensionEdit> loftStationEdit;
    std::shared_ptr<const SweepRebuildGuard> sweepSource;
    std::shared_ptr<const retained_solid::Payload> cut;
    std::shared_ptr<const OcctSavedCutSceneState> cutSource;
    // Whole-program typed append/identified-radius input. Set exactly when the
    // cut carrier holds a v2 Program; the ordinary owner and the document stage
    // independently recompute the legal transition from the captured original.
    std::optional<retained_boolean::ProgramEdit> cutProgramEdit;
    // Separate native source/base/result path. No radius payload or AI permit.
    std::optional<saved_cut_source_edit::Patch> cutSourcePatch;
    std::shared_ptr<const SavedCutSourceDetachedResult> cutSourceRebuild;
    // Explicit whole-program source path: the complete typed recipe transition
    // with the paired native-constructed detached result. Set exactly for
    // CylindricalCutProgramSourceRebuild; never combined with the legacy
    // one-bore fields, a radius/append carrier or an AI permit above.
    std::optional<saved_cut_source_edit::Patch> cutProgramSourcePatch;
    std::shared_ptr<const SavedProgramSourceDetachedResult> cutProgramSourceRebuild;
};

struct OrdinaryTransformRecord {
    OcctObjectTransformState previous;
    OcctObjectTransformState candidate;
    OrdinaryTransformChange requested;
};

struct SweepRebuildGuard; // Main-owned bounded catalog; never dispatched to geometry workers.
struct OrdinaryTransformLedger {
    std::shared_ptr<SweepRebuildGuard> sweepGuard;
    std::shared_ptr<const OcctSavedCutSceneState> cutPrevious,cutCandidate;
    // Minted by the paired document stage, never supplied as a request payload.
    std::shared_ptr<const retained_solid::Payload> cutSourcePayload;
    std::optional<OrdinaryModelingReceiptLedger> modelingReceipt;
    std::vector<OrdinaryTransformRecord> records;
    bool candidateSealed = false;
    std::vector<Handle(SelectMgr_EntityOwner)> selectionOwners;
    PrimitiveManipulatorType manipulatorType = static_cast<PrimitiveManipulatorType>(0);
    bool hadManipulator = false;
};

struct OrdinaryNameChange {
    TDF_Label label;
    TCollection_ExtendedString name;
};

struct OrdinaryNameRecord {
    OcctObjectNameState previous;
    OcctObjectNameState candidate;
    OrdinaryNameChange requested;
};

//! Names do not replace presentations or selection. Retain exact authority
//! across uncertain transaction outcomes before publishing the new metadata.
struct OrdinaryNamePresentation {
    Handle(AIS_Shape) presentation;
    TopoDS_Shape shape;
    gp_Trsf transform;
};

struct OrdinaryNameLedger {
    std::vector<OrdinaryNameRecord> records;
    bool candidateSealed = false;
    std::vector<Handle(SelectMgr_EntityOwner)> selectionOwners;
    PrimitiveManipulatorType manipulatorType = static_cast<PrimitiveManipulatorType>(0);
    ShapeSelectionMode selectionMode = static_cast<ShapeSelectionMode>(0);
    bool hadManipulator = false;
    gp_Trsf manipulatorTransform;
    std::vector<Handle(AIS_InteractiveObject)> manipulatorObjects;
    std::vector<TDF_Label> manipulatorSourceLabels;
    std::vector<TopoDS_Shape> manipulatorCachedShapes;
    std::vector<OrdinaryNamePresentation> selectedPresentations;
    Handle(AIS_InteractiveObject) manipulatorPresentation;
    gp_Ax2 manipulatorPosition;
};

struct OrdinaryVisibilityChange {
    TDF_Label label;
    bool visible = true;
};

struct OrdinaryVisibilityRecord {
    OcctObjectVisibilityState previous;
    OcctObjectVisibilityState candidate;
    OrdinaryVisibilityChange requested;
};

struct OrdinaryVisibilityLedger {
    std::vector<OrdinaryVisibilityRecord> records;
    bool candidateSealed = false;
    std::vector<Handle(SelectMgr_EntityOwner)> selectionOwners;
    std::vector<OcctObjectNameState> selectedObjects;
    OrdinaryNameLedger authority;
    PrimitiveManipulatorType manipulatorType = static_cast<PrimitiveManipulatorType>(0);
    ShapeSelectionMode selectionMode = static_cast<ShapeSelectionMode>(0);
    bool hadManipulator = false;
    std::vector<Handle(AIS_Shape)> targetPresentations;
};

struct OrdinaryAppearanceLedger {
    std::shared_ptr<const OcctPBRScalarPreparation> prepared;
    std::shared_ptr<const OcctPBRScalarState> previous,candidate;
    OrdinaryNameLedger authority;
    bool candidateSealed=false;
};

//! Organization is a distinct durable family; names of objects, geometry,
//! transforms and existing presentation/selection authority are preserved.
struct OrdinaryGroupingLedger {
    OcctSavedGroupState previous;
    OcctSavedGroupState candidate;
    std::vector<OcctSavedGroup> requested;
    std::vector<OcctObjectNameState> objects;
    OrdinaryNameLedger authority;
    bool candidateSealed = false;
};


//! Private, prepared geometry. Ordinary creation never borrows a live source
//! presentation and never publishes it until the durable result is proven.
struct OrdinaryCreationRequest {
    Handle(AIS_Shape) presentation;
    Graphic3d_NameOfMaterial material = Graphic3d_NameOfMaterial_ShinyPlastified;
    Quantity_NameOfColor color = Quantity_NOC_WHITE;
    OcctGeometryRepresentation representation = OcctGeometryRepresentation::BRep;
    std::optional<profile::Parameters> profile;
    std::string profileIdentifier;
    std::optional<enclosure::Parameters> enclosure;
    std::string enclosureIdentifier;
    std::optional<planar_sweep::Definition> sweep;
    std::string sweepIdentifier;
    std::optional<rectangular_loft::Definition> loft;
    std::string loftIdentifier;
    std::optional<TCollection_ExtendedString> name; // Staged in this same creation command.
};
struct OrdinaryCreationRecord {
    OrdinaryCreationRequest requested;
    TopoDS_Shape shape;
    gp_Trsf transform;
    OcctObjectNameState candidate;
};
//! Existing roots may be assemblies or read-only imported occurrences. They
//! are preserved, not subjected to the new object's editable-root admission.
struct OrdinaryCreationRoot {
    TDF_Label label;
    TopoDS_Shape shape;
    std::string entityIdentifier;
    std::string definitionIdentifier;
    OcctGeometryRepresentation representation = OcctGeometryRepresentation::Invalid;
    profile::Record profile;
    enclosure::Record enclosure;
    sweep_persistence::Record sweep;
    loft_persistence::Record loft;
};
using OrdinaryCreationCatalog = std::map<std::string, OrdinaryCreationRoot>;
//! Exact retained source and intended derived-copy metadata.
struct OrdinaryMeshCopySource {
    Handle(AIS_Shape) presentation;
    OcctObjectVisibilityState previous;
    OcctObjectVisibilityState candidate;
    OcctScalarAppearanceState appearance;
    OcctReferenceAxisReadState axisState = OcctReferenceAxisReadState::Invalid;
    OcctReferenceAxis axis;
    TCollection_ExtendedString destinationName;
};

struct OrdinaryCreationLedger {
    std::optional<OrdinaryModelingReceiptLedger> modelingReceipt;
    std::optional<OrdinaryMeshCopySource> meshCopy;
    OrdinaryCreationCatalog previousRoots;
    OcctSavedGroupState groups;
    std::vector<OrdinaryCreationRecord> records;
    OrdinaryNameLedger authority;
    bool candidateSealed = false;
};

//! Typed presentation boundary. The viewer implements admission/repair for
//! exact selection, tool, manipulator and renderer identity. Neither method
//! may mutate OCAF or call NotifyChanges. No captured callbacks in a ledger.
class OrdinaryEditPresentationHost {
public:
    virtual ~OrdinaryEditPresentationHost() = default;
    virtual bool admitTransform(OrdinaryTransformLedger& ledger) noexcept = 0;
    virtual bool admitCreation(OrdinaryCreationLedger&) noexcept { return false; }
    virtual bool admitMeshCopy(OrdinaryCreationLedger&) noexcept { return false; }
    virtual bool repairMeshCopy(const OrdinaryCreationLedger&, bool) noexcept { return false; }
    virtual bool repairCreation(const OrdinaryCreationLedger&, bool) noexcept { return false; }
    virtual bool admitAppearance(OrdinaryAppearanceLedger&) noexcept { return false; }
    virtual bool repairAppearance(const OrdinaryAppearanceLedger&, bool) noexcept { return false; }
    virtual bool admitGrouping(OrdinaryGroupingLedger&) noexcept { return false; }
    virtual bool repairGrouping(const OrdinaryGroupingLedger&, bool) noexcept { return false; }
    virtual bool admitNames(OrdinaryNameLedger&) noexcept { return false; }
    virtual bool admitVisibility(OrdinaryVisibilityLedger&) noexcept { return false; }
    virtual bool repairVisibility(const OrdinaryVisibilityLedger&, bool) noexcept { return false; }
    virtual bool repairNames(const OrdinaryNameLedger&, bool) noexcept { return false; }
    virtual bool repairTransform(const OrdinaryTransformLedger& ledger, bool committed) noexcept = 0;
    //! One bounded full redraw per reconciliation attempt. Only presentation
    //! handles may be replaced; the controller validates them against its
    //! immutable durable snapshots before adopting them or publishing.
    virtual bool rebuildTransform(const OrdinaryTransformLedger& ledger, bool committed,
                                  std::vector<Handle(AIS_Shape)>& replacements) noexcept {
        replacements.clear();
        return false;
    }
};

class OrdinaryEditController;

//! Move-only authority for one synchronous stage/close interval. Destruction
//! requests owned abort/reconciliation; unknown truth remains in the controller.
class Standard_EXPORT OrdinaryEditLease final {
public:
    OrdinaryEditLease() = default;
    OrdinaryEditLease(OrdinaryEditLease&& other) noexcept;
    OrdinaryEditLease& operator=(OrdinaryEditLease&& other) noexcept;
    ~OrdinaryEditLease() noexcept;
    OrdinaryEditLease(const OrdinaryEditLease&) = delete;
    OrdinaryEditLease& operator=(const OrdinaryEditLease&) = delete;
    explicit operator bool() const noexcept { return _token != 0 && !_controller.expired(); }
    OrdinaryEditResult stageAndCommit() noexcept;
    OrdinaryEditResult cancel() noexcept;
private:
    friend class OrdinaryEditController;
    OrdinaryEditLease(std::weak_ptr<OrdinaryEditController> controller, std::uint64_t token,
                      std::shared_ptr<const std::uint8_t> lifetime) noexcept;
    std::weak_ptr<OrdinaryEditController> _controller;
    std::shared_ptr<const std::uint8_t> _lifetime;
    std::uint64_t _token = 0;
};

//! Shared durable state machine with typed transform, creation and metadata
//! families. Remove and Appearance still need their exact snapshot contracts.
//! The owning viewer must retain this controller and gate normal work with
//! blocksNormalWork(), including the synchronous Publishing interval.
class Standard_EXPORT OrdinaryEditController final
    : public std::enable_shared_from_this<OrdinaryEditController> {
public:
    OrdinaryEditController(Handle(OcctDocument) document, OrdinaryEditPresentationHost& host);
    OrdinaryEditController(const OrdinaryEditController&) = delete;
    OrdinaryEditController& operator=(const OrdinaryEditController&) = delete;
    OrdinaryEditLease beginTransform(const std::vector<OrdinaryTransformChange>& changes,
                                     OrdinaryEditResult* failure = nullptr) noexcept;
    OrdinaryEditLease beginMeshCopy(const Handle(AIS_Shape)& source,
        const TCollection_ExtendedString& name, OrdinaryEditResult* failure = nullptr) noexcept;
    OrdinaryEditLease beginCreation(const std::vector<OrdinaryCreationRequest>& requests,
                                   OrdinaryEditResult* failure = nullptr) noexcept;
    OrdinaryEditLease beginModelingCreation(const std::vector<OrdinaryCreationRequest>& requests,
        std::shared_ptr<NativeModelingCommitPermit> permit, OrdinaryEditResult* failure) noexcept;
    OrdinaryEditLease beginModelingRebuild(const OrdinaryTransformChange& change,
        std::shared_ptr<NativeModelingCommitPermit> permit, OrdinaryEditResult* failure) noexcept;
    OrdinaryEditLease beginModelingPlacement(const OrdinaryTransformChange& change,
        std::shared_ptr<NativeModelingCommitPermit> permit, OrdinaryEditResult* failure) noexcept;
    OrdinaryEditLease beginAppearance(const std::shared_ptr<const OcctPBRScalarPreparation>&,
        OrdinaryEditResult* failure = nullptr) noexcept;
    OrdinaryEditLease beginGrouping(const std::vector<OcctSavedGroup>& groups,
                                   OrdinaryEditResult* failure = nullptr) noexcept;
    OrdinaryEditLease beginNames(const std::vector<OrdinaryNameChange>& changes,
                                OrdinaryEditResult* failure = nullptr) noexcept;
    OrdinaryEditLease beginVisibility(const std::vector<OrdinaryVisibilityChange>& changes,
                                     OrdinaryEditResult* failure = nullptr) noexcept;
    OrdinaryEditResult reconcile() noexcept;
    bool blocksNormalWork() const noexcept;
    std::shared_ptr<const SweepRebuildGuard> captureSavedSweepRebuildSource() const noexcept;
    bool savedSweepRebuildSourceIsCurrent(const std::shared_ptr<const SweepRebuildGuard>& source) const noexcept;
    OrdinaryEditState state() const noexcept { return _state; }
#ifdef DEBUG
    OrdinaryEditCommandStamp& debugCommandStamp() noexcept { return _command; }
    void debugSetStageFailureIndex(int index) noexcept { _stageFailureIndex = index; }
    void debugSetTruthUnavailableCount(int count) noexcept { _truthUnavailableCount = count; }
#endif
private:
    friend class OrdinaryEditLease;
    using PendingEdit = std::variant<OrdinaryTransformLedger, OrdinaryNameLedger, OrdinaryVisibilityLedger, OrdinaryGroupingLedger, OrdinaryCreationLedger, OrdinaryAppearanceLedger>;
    OrdinaryEditResult stageAndCommit(std::uint64_t token) noexcept;
    OrdinaryEditResult cancel(std::uint64_t token) noexcept;
    OrdinaryEditResult reconcileImpl() noexcept;
    OrdinaryEditResult stageCreationAndCommit(std::uint64_t token) noexcept;
    OrdinaryEditResult reconcileCreationImpl() noexcept;
    OrdinaryEditLease beginTransformImpl(const std::vector<OrdinaryTransformChange>& changes,
        OrdinaryEditResult* failure, std::shared_ptr<NativeModelingCommitPermit> permit) noexcept;
    bool bindPlacementReceipt(OrdinaryTransformLedger& ledger,const OrdinaryTransformChange& change,
        const OcctObjectTransformState& previous,std::shared_ptr<NativeModelingCommitPermit> permit) noexcept;
    bool capturePlacementReceipt(const OrdinaryTransformLedger& ledger,placement::Evidence& out) noexcept;
    bool bindRebuildReceipt(OrdinaryTransformLedger& ledger, const OrdinaryTransformChange& change,
        const OcctObjectTransformState& previous, std::shared_ptr<NativeModelingCommitPermit> permit) noexcept;
    bool rebuildReceiptMatches(const OrdinaryTransformLedger& ledger, bool candidate) noexcept;
    bool stageRebuildReceipt(OrdinaryTransformLedger& ledger) noexcept;
    bool stageCreationReceipt(OrdinaryCreationLedger& ledger) noexcept;
    bool creationReceiptMatches(const OrdinaryCreationLedger& ledger, bool candidate) const noexcept;
    bool creationMatches(const OrdinaryCreationLedger& ledger, bool candidate) const noexcept;
    OrdinaryEditLease beginCreationImpl(const std::vector<OrdinaryCreationRequest>& requests,
        std::optional<OrdinaryMeshCopySource> meshCopy, OrdinaryEditResult* failure,
        std::shared_ptr<NativeModelingCommitPermit> permit = {}) noexcept;
    bool meshCopySourceMatches(const OrdinaryMeshCopySource& source, bool candidate) const noexcept;
    OrdinaryEditResult stageAppearanceAndCommit(std::uint64_t token) noexcept;
    OrdinaryEditResult reconcileAppearanceImpl() noexcept;
    OrdinaryEditResult stageGroupingAndCommit(std::uint64_t token) noexcept;
    OrdinaryEditResult reconcileGroupingImpl() noexcept;
    bool groupingMatches(const OrdinaryGroupingLedger& ledger, bool candidate) const noexcept;
    OrdinaryEditResult stageNamesAndCommit(std::uint64_t token) noexcept;
    OrdinaryEditResult reconcileNamesImpl() noexcept;
    OrdinaryEditResult stageVisibilityAndCommit(std::uint64_t token) noexcept;
    OrdinaryEditResult reconcileVisibilityImpl() noexcept;
    bool captureMatches(const OcctObjectVisibilityState& expected) const noexcept;
    bool captureMatches(const OcctObjectNameState& expected) const noexcept;
    bool captureMatches(const OcctObjectTransformState& expected) const noexcept;
    bool savedCutSourceChangeMatches(const OrdinaryTransformChange& request,
        const OcctObjectTransformState& previous) const noexcept;
    bool savedProgramSourceChangeMatches(const OrdinaryTransformChange& request,
        const OcctObjectTransformState& previous) const noexcept;
    bool presentationMatches(const OrdinaryTransformLedger& ledger, bool committed) const noexcept;
    void clearResolved() noexcept;

    Handle(OcctDocument) _document;
    OrdinaryEditPresentationHost& _host;
    OrdinaryEditCommandStamp _command;
    std::optional<PendingEdit> _pending;
    OrdinaryEditState _state = OrdinaryEditState::Idle;
    std::uint64_t _nextToken = 0;
    std::uint64_t _activeToken = 0;
    std::weak_ptr<const std::uint8_t> _leaseLifetime;
    bool _entering = false;
    bool _reconciling = false;
    bool _committed = false;
    bool _didPublish = false;
#ifdef DEBUG
    int _stageFailureIndex = -1;
    int _truthUnavailableCount = 0;
#endif
};

} // namespace core3d
#endif
