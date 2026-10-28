//
//  OcctSceneSnapshotBuilder.hpp
//  Core3D
//
//  Main-thread adapter from the mutable OCCT/OCAF model to immutable,
//  renderer-neutral scene values.
//

#ifndef Core3D_OcctSceneSnapshotBuilder_hpp
#define Core3D_OcctSceneSnapshotBuilder_hpp

#include "SceneSnapshot.hpp"
#include "../OCCTKit/NativeContactSource.hpp"
#include "../OCCTKit/PaintedAtlasBake.hxx"

#include <Standard_Handle.hxx>
#include <TDF_Label.hxx>

#include <memory>
#include <functional>
#include <TopoDS_Shape.hxx>
#include <array>
#include <optional>
#include <string>
#include <cstdint>
#include <vector>

class AIS_InteractiveContext;
class AIS_Shape;
class OcctDocument;
class V3d_View;

namespace core3d::scene {

//! Exact mapping carried by a private export from saved authority through a
//! disposable face to the primitive emitted by the final representation.
struct PrivateExportFaceCorrespondence final {
    std::string ownerDefinitionIdentifier;
    std::uint32_t originalFaceIndex = 0;
    std::uint32_t disposableFaceIndex = 0;
    std::uint32_t emittedPrimitiveIndex = 0;
    std::uint32_t materialIndex = 0;
};

//! Value-only writer-facing final mesh descriptor. MeshSnapshot owns the
//! exact numeric vertex/index/corner/tangent streams; InstanceSnapshot owns
//! occurrence transform, winding and primitive/material bindings.
struct PrivateExportFinalMesh final {
    std::string ownerDefinitionIdentifier;
    MeshSnapshot mesh;
    InstanceSnapshot instance;
};

//! Immutable operation-owned value produced from the saved all-member layout
//! path before any raster bake. Every vector is retained by the returned
//! private-export owner and is readable only through its exact scene alias.
// Shared native fine-profile mesher used by the layout-only producer and the
// existing DEBUG final-result oracle. It never bakes or publishes an atlas.
bool MeshSavedGroupLayoutPrivateSurfaces(const TopoDS_Shape& shape) noexcept;

struct SavedGroupAllMemberLayout final {
    std::vector<std::string> memberDefinitions;
    asset_atlas::Definition atlas;
    std::vector<asset_atlas::MemberUVAssignment> assignments;
    asset_atlas::build::LayoutEvidence layout;
    painted_atlas_bake::kernel::WorkMeasurement work;
};

struct SavedGroupAllMemberLayoutRecord final {
    decal_layer::bake::accounting::Ticket recordStorage;
    painted_atlas_bake::owner::AggregateBakePlan aggregate;
    std::vector<std::shared_ptr<
        const painted_atlas_bake::owner::ExportBakeStorage>> planStorage;
    std::vector<SavedGroupAllMemberLayout> groups;
    std::uint64_t layoutNanoseconds = 0;
    std::uint64_t preparationNanoseconds = 0;
    std::size_t retainedBytes = 0;
};

enum class SavedGroupAllMemberLayoutReason : std::uint8_t {
    Present = 0,
    Unavailable = 1,
    Overflow = 2,
    Malformed = 3,
    StaleSource = 4,
    StaleBinding = 5,
    UnsupportedSurface = 6,
    Refused = 7,
};

struct SavedGroupAllMemberLayoutRead final {
    std::shared_ptr<const SavedGroupAllMemberLayoutRecord> record;
    SavedGroupAllMemberLayoutReason reason =
        SavedGroupAllMemberLayoutReason::Unavailable;
};

//! Immutable passive evidence from the exact production operation. Typed
//! source authority is borrowed from capture values owned by the same
//! returned-scene owner; bounded final witnesses are copied once under that
//! operation's retained accounting. No field grants bake or publication
//! authority.
struct PrivateExportRawEvidenceRecord final {
    decal_layer::bake::accounting::Ticket recordStorage;
    std::vector<const painted_atlas_bake::owner::ExportCapture*>
        savedCaptures;
    std::vector<const std::vector<
        painted_atlas_bake::owner::CapturedSource>*> transientSources;
    std::shared_ptr<const SavedGroupAllMemberLayoutRecord> savedGroupLayout;
    std::vector<PrivateExportFaceCorrespondence> correspondence;
    std::vector<std::string> inputKeys;
    std::vector<std::string> bakeSeals;
    std::vector<std::string> outputRoles;
    std::vector<std::uint64_t> outputRoleBytes;
    struct RoleContentWitness final {
        std::string ownerDefinitionIdentifier;
        face_image::Role role = face_image::Role::BaseColor;
        face_image::Digest contentDigest{};
        std::int32_t returnedTextureIndex = -1;
    };
    std::vector<RoleContentWitness> roleContentWitnesses;
    const SceneSnapshot* returnedSceneAlias = nullptr;
    std::uint64_t memberCount = 0;
    std::uint64_t layoutCount = 0;
    std::uint64_t finalTriangleCount = 0;
    std::uint64_t finalCornerCount = 0;
    std::uint64_t layoutResolution = 0;
    std::uint64_t layoutChartCount = 0;
    bool complete = false;
};

struct PrivateExportRawEvidenceRead final {
    std::shared_ptr<const PrivateExportRawEvidenceRecord> record;
};

struct PrivateExportDecalArtifact final {
    std::shared_ptr<const SceneSnapshot> scene;
    std::vector<PrivateExportFaceCorrespondence> correspondence;
    std::vector<std::string> inputKeys;
    std::vector<std::string> bakeSeals;
    std::shared_ptr<const SavedGroupAllMemberLayoutRecord> savedGroupLayout;
    SavedGroupAllMemberLayoutReason savedGroupLayoutReason =
        SavedGroupAllMemberLayoutReason::Unavailable;
    std::shared_ptr<const PrivateExportRawEvidenceRecord> rawEvidence;
};

//! Opaque move-only authority captured before any disposable export command
//! or remeshing. Its native shapes/resources and aggregate budget never leave
//! the implementation file.
class PrivateDecalCapture final {
public:
    PrivateDecalCapture() noexcept;
    ~PrivateDecalCapture();
    PrivateDecalCapture(PrivateDecalCapture&&) noexcept;
    PrivateDecalCapture& operator=(PrivateDecalCapture&&) noexcept;
    PrivateDecalCapture(const PrivateDecalCapture&) = delete;
    PrivateDecalCapture& operator=(const PrivateDecalCapture&) = delete;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    friend class OcctSceneSnapshotBuilder;
};

using PrivateDecalCapturePointer = std::unique_ptr<PrivateDecalCapture>;

#ifdef DEBUG
enum class DebugPrivateExportDecalFault : std::uint8_t {
    None = 0,
    Capture = 1,
    FinalProduction = 2,
    Publication = 3,
    AggregateBudget = 4,
    SavedAtlasCaptureCheckpoint = 5,
    SavedAtlasCaptureDenyRead = 6,
    SavedAtlasCaptureMissingBake = 7,
    SavedAtlasCaptureMalformedBake = 8,
    SavedAtlasCaptureStaleBinding = 9,
    SavedAtlasCaptureMissingMember = 10,
    SavedAtlasCaptureForeignMember = 11,
    SavedAtlasCaptureDuplicateMember = 12,
    SavedAtlasCaptureForeignRawSlot = 13,
    SavedAtlasCaptureMalformedResource = 14,
};

enum class DebugPrivateExportPhase : std::uint8_t {
    Capture = 1,
    Finalization = 2,
};

//! Bounded values observed from the executed private GLB derivative path.
//! Digests and numeric streams are copied; labels, handles and authority
//! tokens remain native and never escape this seam.
struct DebugPrivateExportDecalObservation final {
    std::string savedGroupLayoutRecordState = "absent";
    std::string savedGroupLayoutRecordReason = "unavailable";
    std::vector<std::string> savedGroupLayoutMemberDefinitions;
    std::vector<std::uint64_t> savedGroupLayoutAssignmentCornerCounts;
    std::uint64_t savedGroupLayoutMemberCount = 0;
    std::uint64_t savedGroupLayoutAssignmentCount = 0;
    std::uint64_t savedGroupLayoutResolution = 0;
    std::uint64_t savedGroupLayoutGutter = 0;
    std::uint64_t savedGroupLayoutChartCount = 0;
    std::uint64_t savedGroupLayoutNanoseconds = 0;
    std::uint64_t savedGroupLayoutPreparationNanoseconds = 0;
    std::uint64_t savedGroupLayoutRetainedBytes = 0;
    std::uint64_t savedGroupLayoutOperationBytes = 0;
    std::vector<std::uint64_t> savedGroupLayoutStoragePeakBytes;
    std::vector<std::uint64_t> savedGroupLayoutStorageLimitBytes;
    bool savedGroupLayoutBakeJobsUnsealed = false;
    std::uint64_t ownerCount = 0;
    std::uint64_t requiredReceiverCount = 0;
    std::uint64_t producedReceiverCount = 0;
    std::uint64_t savedTriangleCount = 0;
    std::uint64_t savedCornerCount = 0;
    std::uint64_t finalTriangleCount = 0;
    std::uint64_t finalCornerCount = 0;
    std::uint64_t operationWork = 0;
    std::uint64_t operationResidentBytes = 0;
    std::uint64_t aggregateBeforeBytes = 0;
    std::uint64_t aggregatePressureBytes = 0;
    std::uint64_t aggregateChargedBytes = 0;
    std::uint64_t aggregateLimitBytes = 0;
    std::uint64_t aggregateRetainedBytes = 0;
    std::uint64_t aggregatePeakBytes = 0;
    std::uint64_t aggregateDeniedRequestBytes = 0;
    std::uint64_t aggregateDeniedAvailableBytes = 0;
    std::uint64_t allocationsAfterAggregateDenial = 0;
    std::uint64_t terminalResidentBytes = 0;
    std::uint64_t terminalDestructionCount = 0;
    std::uint64_t terminalDestroyedBytes = 0;
    std::vector<std::string> canonicalReadDigests;
    std::vector<std::string> sourceReadDigests;
    std::string occluderDigest;
    std::vector<PrivateExportFaceCorrespondence> correspondence;
    std::vector<std::string> inputKeys;
    std::vector<std::string> bakeSeals;
    std::vector<std::string> outputRoles;
    std::vector<std::uint64_t> outputRoleBytes;
    std::vector<std::vector<std::uint8_t>>
        rawEvidenceCanonicalBindingBytes;
    std::vector<std::string> savedMemberDefinitions;
    std::vector<std::string> savedMemberOwners;
    std::vector<std::string> savedSourceOwners;
    std::vector<std::string> savedSourceResources;
    std::vector<std::uint64_t> savedSourceMemberIndices;
    std::vector<std::uint64_t> savedSourceFaceIndices;
    std::vector<std::uint64_t> savedSourceOrigins;
    std::vector<std::uint64_t> savedCanonicalLengths;
    std::uint64_t savedGroupCount = 0;
    std::uint64_t savedMemberCount = 0;
    std::uint64_t savedSourceCount = 0;
    std::uint64_t savedRawSlotCount = 0;
    std::uint64_t savedMeasuredCopyCount = 0;
    std::uint64_t savedCorrespondenceCount = 0;
    std::uint64_t savedOriginalOnlyCount = 0;
    std::uint64_t savedNonMemberCount = 0;
    std::uint64_t savedSelectedMemberCount = 0;
    std::uint64_t savedAtlasReadEntries = 0;
    std::uint64_t savedReceiverMeasuredCopyCount = 0;
    std::uint64_t savedBorrowedOwnerCount = 0;
    std::uint64_t savedBorrowedReceiverCount = 0;
    std::uint64_t savedBorrowedReceiverCopyMatches = 0;
    std::string captureRefusalCause;
    std::string captureRefusalBoundary;
    std::string captureRefusalField;
    std::string aggregateFirstDenialSite;
    std::string aggregateFirstDenialReason;
    std::string terminalFirstFailureSite;
    std::uint64_t layoutResolution = 0;
    std::uint64_t layoutChartCount = 0;
    std::string failureStage;
    bool captureReached = false;
    bool finalProductionReached = false;
    bool publicationReached = false;
    bool finalizationReached = false;
    bool capturePauseReached = false;
    bool finalizationPauseReached = false;
    bool aggregatePressureInjected = false;
    bool terminalSettlementObserved = false;
    bool terminalSettlementOverflow = false;
    bool workerReturned = false;
    bool privateFilesClean = false;
    bool returnedScene = false;
    bool complete = false;
    bool savedMeasuredStageComplete = false;
    bool savedDetachedIdentitiesDistinct = false;
    // Independently re-derived at the checkpoint: the committed document's
    // export closure carries no SYDL record, so an empty borrowed
    // owner/receiver census is the expected case rather than a missing one.
    bool savedReceiverExpectationEmpty = false;
    bool savedArtifactPresent = false;
};

enum class DebugPersistedAtlasCaptureMode : std::uint8_t {
    Capture = 0,
    CancelBeforeCopy = 1,
    DenyFirstCopy = 2,
};

//! Value-only observation from the production saved-atlas capture/copy/bake
//! helpers. Native shapes, labels, operation views and persisted bytes remain
//! private to the builder.
struct DebugPersistedAtlasCaptureLineageObservation final {
    std::vector<std::string> memberDefinitions;
    std::vector<std::string> sourceDefinitions;
    std::string failureStage;
    std::uint64_t atlasCount = 0;
    std::uint64_t memberCount = 0;
    std::uint64_t measuredCopyCount = 0;
    std::uint64_t sourceCount = 0;
    std::uint64_t correspondenceCount = 0;
    std::uint64_t originalOnlyCount = 0;
    std::uint64_t nonMemberCount = 0;
    std::uint64_t assignmentCount = 0;
    std::uint64_t roleCount = 0;
    std::uint64_t resolution = 0;
    std::uint64_t gutter = 0;
    bool inheritedOperation = false;
    bool stableMemberOrder = false;
    bool detachedAuthority = false;
    bool canonicalUnchanged = false;
    bool selectedMemberPresent = false;
    bool allMemberBake = false;
    bool oneSharedBake = false;
    bool captureMovePreserved = false;
    bool outputSurvivedCaptureReset = false;
    bool cancelledBeforeCopy = false;
    bool deniedBeforeCopy = false;
};

//! Value-only evidence from one saved-group transaction over native-remeshed
//! detached members. No document handles, labels or operation views escape.
struct DebugSavedGroupFinalObservation final {
    std::vector<std::string> memberDefinitions;
    std::vector<std::uint64_t> savedTriangleCounts;
    std::vector<std::uint64_t> finalTriangleCounts;
    std::vector<std::uint64_t> finalCornerCounts;
    std::vector<std::uint64_t> assignmentCornerCounts;
    std::vector<std::uint8_t> sourceOrigins;
    std::vector<std::vector<std::uint8_t>> outputPNGs;
    std::string strictKey;
    std::string failureStage;
    std::uint64_t measuredCopyCount = 0;
    std::uint64_t occurrenceCount = 0;
    std::uint64_t assignmentCount = 0;
    std::uint64_t layoutChartCount = 0;
    std::uint64_t outputCount = 0;
    std::uint64_t resolution = 0;
    std::uint64_t gutter = 0;
    bool selectedObjectsOnly = false;
    bool fullSavedMembership = false;
    bool exactFaceCorrespondence = false;
    bool planeInput = false;
    bool cylinderInput = false;
    bool changedCylinderInput = false;
    bool retainedLayout = false;
    bool sharedResultIdentity = false;
    bool complete = false;
};

//! Nb's passive, separately owned test result. No P1/P2b ABI extension.
struct DebugSavedOriginalPaintObservation final {
    DebugSavedGroupFinalObservation group;
    std::string failureStage;
    std::vector<std::uint8_t> receiverPNG;
    std::uint64_t protectedCornerCount = 0;
    std::uint64_t receiverFace = 0;
    bool receiverCountRefused = false;
    bool completeOriginalPaintBinding = false;
    struct Source final {
        std::size_t member = 0;
        std::string origin, workingContent;
        std::vector<std::uint8_t> png;
        bool effective = false, ownerMatched = false;
        bool faceMatched = false, fenceMatched = false;
    };
    std::vector<Source> sources;
    std::vector<std::array<double, 19>> corners;
    std::vector<std::array<double, 18>> charts;
    std::array<double, 4> baseTint{};
    std::array<std::uint64_t, 4> accounting{};
    std::array<std::uint64_t, 2> frozenSizes{};
    std::string fault;
    bool usedReleaseAdapter = false, canonicalUnchanged = false;
    bool proofsUnchanged = false, oneBuild = false, sameOperation = false;
    bool lifetimeVerified = false, mutatedDetachedInput = false;
    bool productPresent = false, stickyFailure = false;
    std::uint64_t allocationsAfterRefusal = 0, retainedBytesAfterRelease = 0;

};

struct DebugSavedProducerBatchObservation final {
    std::vector<std::string> strictKeys;
    std::vector<std::uint64_t> memberCounts;
    std::vector<std::uint64_t> assignmentCounts;
    std::vector<std::uint64_t> outputCounts;
    std::vector<std::uint64_t> work;
    std::vector<std::vector<std::uint8_t>> firstGroupPNGs;
    std::string failureStage;
    std::uint64_t rawSlotCount = 0;
    std::uint64_t presentSlotCount = 0;
    std::uint64_t absentSlotCount = 0;
    std::uint64_t shadowedByteCount = 0;
    std::uint64_t measuredCopyCount = 0;
    std::uint64_t layoutCount = 0;
    bool disjointMembership = false;
    bool rawAuthorityRetained = false;
    bool descriptorDifferenceDetected = false;
    bool oneAggregateSeal = false;
    bool distinctGroupResults = false;
    bool sharedWithinGroups = false;
    bool complete = false;
};

//! Value-only observation of the bounded-curve input used by one successful
//! full-scene publication. This test seam is armed explicitly for one
//! synchronous DEBUG capture and never retains labels, document handles or
//! edit authority.
struct DebugBoundedCurvePublicationObservation final {
    std::string entityIdentifier;
    std::string definitionIdentifier;
    std::string publicationSourceIdentifier;
    std::vector<std::uint8_t> canonicalDefinitionBytes;
    std::vector<std::uint8_t> canonicalOwnerBytes;
    std::array<std::uint8_t, 32> canonicalDefinitionDigest{};
    std::uint64_t modelRevision = 0;
    std::uint32_t edgeCount = 0;
    std::uint32_t vertexCount = 0;
};

//! Arm/reset, consume, or cancel the one-shot publication observation. Only a
//! fully validated and committed Build can publish a consumable value.
void DebugBeginBoundedCurvePublicationObservation() noexcept;
bool DebugTakeBoundedCurvePublicationObservation(
    DebugBoundedCurvePublicationObservation& theObservation) noexcept;
void DebugCancelBoundedCurvePublicationObservation() noexcept;
#endif

//! Failure from this synchronous capture only. The optional bake status is
//! populated solely by the committed ordinary receiver's ProduceFace call.
//! Stage points to a static string; no document or DEBUG state escapes capture.
struct SceneSnapshotCaptureFailure final {
    std::optional<decal_layer::bake::Status> decalCoverageStatus;
    const char* stage = nullptr;
};

//! Failure from one private export derivative invocation. The optional status
//! is populated only by the ordinary receiver's actual ProduceFace result and
//! is reset on every entry. Stage points to a static string.
struct PrivateExportDerivativeFailure final {
    std::optional<decal_layer::bake::Status> decalCoverageStatus;
    const char* stage = nullptr;
};

class OcctSceneSnapshotBuilder final {
public:
    using SnapshotPointer = std::shared_ptr<const SceneSnapshot>;
    using OverlayPointer = std::shared_ptr<const PresentationOverlaySnapshot>;

    //! Passive lookup sealed to the exact returned-scene alias/control block.
    //! A copied scene is unavailable. The native wrapper also checks the
    //! originating operation identity before exposing this value.
    static SavedGroupAllMemberLayoutRead SavedGroupAllMemberLayoutForReturnedScene(
        const SnapshotPointer& returnedScene) noexcept;
    static PrivateExportRawEvidenceRead
        OperationOwnedRawEvidenceForReturnedScene(
            const SnapshotPointer& returnedScene) noexcept;

#ifdef DEBUG
    // Passive aggregate of the retained executed saved results, never a bake.
    static std::array<double, 4> DebugSavedFinalQualityForReturnedScene(
        const SnapshotPointer& returnedScene) noexcept;
#endif

    OcctSceneSnapshotBuilder();
    ~OcctSceneSnapshotBuilder();

    OcctSceneSnapshotBuilder(const OcctSceneSnapshotBuilder&) = delete;
    OcctSceneSnapshotBuilder& operator=(const OcctSceneSnapshotBuilder&) = delete;

    //! Build a committed-scene snapshot. This method is deliberately
    //! main-thread-only because OCAF documents, AIS state, and face
    //! triangulations are mutable even though their handles are ref-counted.
    //!
    //! A null result means validation or extraction failed. Failed builds do
    //! not consume revision numbers or alter the builder's last-known state.
    //! The accepted Object/Face/Edge mode is copied into the publication and
    //! controls both semantic topology identity and GPU pick admission.
    SnapshotPointer Build(
        const Handle(OcctDocument)& theDocument,
        const Handle(AIS_InteractiveContext)& theContext,
        const Handle(V3d_View)& theView,
        const UInt2& theViewportPixels,
        ElementKind theAcceptedSelectionKind) noexcept;

    //! Same capture with an explicit, reset-on-entry production failure value.
    SnapshotPointer Build(
        const Handle(OcctDocument)& theDocument,
        const Handle(AIS_InteractiveContext)& theContext,
        const Handle(V3d_View)& theView,
        const UInt2& theViewportPixels,
        ElementKind theAcceptedSelectionKind,
        SceneSnapshotCaptureFailure* failureOut) noexcept;

    //! Owner-thread read of a named occurrence from the last committed full
    //! publication. Does not require or modify selection. Caller must first
    //! obtain a fresh committed snapshot through the viewer's edit barriers.
    //! Only copied numeric values leave this adapter, never labels or shapes.
    meshcheck::ContactSourceStatus CaptureNativeMeshContacts(
        const Handle(OcctDocument)& document,
        const meshcheck::ContactSourceIdentity& expected,
        const std::atomic_bool& cancelled,
        meshcheck::ContactSourceCapture& output) noexcept;

    //! Export-only derivative over an exclusively owned deserialized document.
    //! Never call with the live document. The callback meshes only the supplied
    //! private definitions. Frozen appearance/provenance and authored mesh
    //! buffers are retained; the returned values must never be published to a
    //! renderer as a new committed scene. No AIS/view handles are consulted.
    static SnapshotPointer BuildPrivateExportDerivative(
        const Handle(OcctDocument)& document,
        const SceneSnapshot& source,
        bool selectedObjectsOnly,
        const std::function<void(const TopoDS_Shape&)>& meshPrivateSurfaces,
        const std::function<bool()>& cancelled,
        PrivateExportDerivativeFailure* failureOut = nullptr
#ifdef DEBUG
        , DebugPrivateExportDecalFault debugFault =
            DebugPrivateExportDecalFault::None
        , DebugPrivateExportDecalObservation* debugObservation = nullptr
        , const std::function<void(DebugPrivateExportPhase)>&
            debugPhaseObserver = {}
#endif
        ) noexcept;

    static bool CapturePrivateExportDecals(
        const Handle(OcctDocument)& privateDocument,
        const SceneSnapshot& wholeCommittedScene,
        bool selectedObjectsOnly,
        const std::function<bool()>& cancelled,
        PrivateDecalCapturePointer& capture
#ifdef DEBUG
        , DebugPrivateExportDecalFault debugFault =
            DebugPrivateExportDecalFault::None
        , DebugPrivateExportDecalObservation* debugObservation = nullptr
        , const std::function<void(DebugPrivateExportPhase)>&
            debugPhaseObserver = {}
#endif
        ) noexcept;

    static bool FinalizePrivateExportDecals(
        PrivateDecalCapture& capture,
        const SceneSnapshot& finalScene,
        const std::vector<PrivateExportFaceCorrespondence>& correspondence,
        const std::vector<PrivateExportFinalMesh>& emittedMeshes,
        const std::vector<std::string>& inputKeys,
        const std::vector<std::string>& bakeSeals,
        const std::function<bool()>& cancelled,
        PrivateExportDecalArtifact& artifact
#ifdef DEBUG
        , DebugPrivateExportDecalObservation* debugObservation = nullptr
        , const std::function<void(DebugPrivateExportPhase)>&
            debugPhaseObserver = {}
#endif
        ) noexcept;

#ifdef DEBUG
    static bool DebugCapturePersistedAtlasLineage(
        const Handle(OcctDocument)& privateDocument,
        const std::string& selectedDefinitionIdentifier,
        DebugPersistedAtlasCaptureMode mode,
        DebugPersistedAtlasCaptureLineageObservation& observation) noexcept;

    static bool DebugProduceSavedGroupFinalResult(
        const Handle(OcctDocument)& exclusivelyOwnedPrivateDocument,
        const SceneSnapshot& wholeCommittedScene,
        bool selectedObjectsOnly,
        const std::function<void(const TopoDS_Shape&)>& meshPrivateSurfaces,
        const std::function<bool()>& cancelled,
        DebugSavedGroupFinalObservation& observation) noexcept;
    static bool DebugProduceSavedOriginalPaint(
        const Handle(OcctDocument)& exclusivelyOwnedPrivateDocument,
        const SceneSnapshot& wholeCommittedScene,
        bool selectedObjectsOnly,
        const std::function<void(const TopoDS_Shape&)>& meshPrivateSurfaces,
        const std::function<bool()>& cancelled,
        const std::string& fault,
        DebugSavedOriginalPaintObservation& observation) noexcept;
    static bool DebugProduceSavedProducerBatch(
        const Handle(OcctDocument)& exclusivelyOwnedPrivateDocument,
        const SceneSnapshot& wholeCommittedScene,
        const std::function<void(const TopoDS_Shape&)>& meshPrivateSurfaces,
        const std::function<bool()>& cancelled,
        DebugSavedProducerBatchObservation& observation) noexcept;
#endif

    //! Capture only the semantic camera and revisions. Document, model, and
    //! presentation revisions come from the most recent full snapshot; camera
    //! and snapshot revisions advance when camera semantics change. Returns no
    //! value until this builder has published the active document once.
    //! Main-thread only.
    std::optional<FrameSnapshot> CaptureFrame(
        const Handle(OcctDocument)& theDocument,
        const Handle(V3d_View)& theView,
        const UInt2& theViewportPixels) noexcept;

    //! Publish already-extracted transient presentation data against the most
    //! recent full scene. This never traverses the document and has an
    //! independent revision domain. Empty content is an explicit clear; a
    //! null result means the base scene or content is unsafe. Main-thread only.
    OverlayPointer PublishPresentationOverlay(
        const Handle(OcctDocument)& theDocument,
        PresentationOverlayContent&& theContent) noexcept;

    //! Append renderer-neutral world-space geometry from one explicitly owned
    //! valid mirror trial set, then publish it with the exact six-plane mirror
    //! gizmo prefix. This reads only existing face triangulations and never
    //! enumerates AIS context or remeshes either transient or committed shapes.
    OverlayPointer PublishMirrorPreviewOverlay(
        const Handle(OcctDocument)& theDocument,
        PresentationOverlayContent&& theMirrorGizmoContent,
        const std::vector<Handle(AIS_Shape)>& thePreviewShapes) noexcept;

    //! Publish one explicitly owned, already-displayed Boolean preview. Source
    //! labels are resolved only through the retained mapping from the last full
    //! scene; no AIS enumeration, OCAF traversal, or remeshing is performed.
    OverlayPointer PublishBooleanPreviewOverlay(
        const Handle(OcctDocument)& theDocument,
        PresentationOverlayKind theKind,
        const std::vector<Handle(AIS_Shape)>& theActorShapes,
        const std::vector<Handle(AIS_Shape)>& theResultShapes,
        const std::vector<TDF_Label>& theSuppressedSourceLabels) noexcept;

    //! Publish one bounded, explicitly owned Bevel result per committed source.
    //! Source labels are resolved only through the retained last-full-scene map;
    //! existing face triangulations are copied without AIS discovery or meshing.
    OverlayPointer PublishChamferPreviewOverlay(
        const Handle(OcctDocument)& theDocument,
        const std::vector<Handle(AIS_Shape)>& theResultShapes,
        const std::vector<TDF_Label>& theSuppressedSourceLabels) noexcept;

    //! Publish exactly one explicitly owned Shell result replacing exactly one
    //! committed source occurrence. Textured or subshape-styled definitions
    //! fail closed so the OCCT viewport remains appearance-authoritative.
    //! Main-thread only.
    OverlayPointer PublishShellPreviewOverlay(
        const Handle(OcctDocument)& theDocument,
        const Handle(AIS_Shape)& theResultShape,
        const TDF_Label& theSuppressedSourceLabel) noexcept;

    //! Publish one source mesh/material with one world-space instance per
    //! explicitly owned Linear Array preview body. Geometry and appearance are
    //! copied once from the first handle's existing cache; the committed source
    //! remains unchanged and is never suppressed. Main-thread only.
    OverlayPointer PublishLinearArrayPreviewOverlay(
        const Handle(OcctDocument)& theDocument,
        const std::vector<Handle(AIS_Shape)>& thePreviewShapes) noexcept;

    //! Publish the exact-empty semantic Linear Array overlay used when a
    //! retained operation has valid zero spacing. This is intentionally
    //! separate from the generic empty overlay so renderers can distinguish a
    //! deliberate zero-copy preview from a missing selection gizmo.
    //! Main-thread only.
    OverlayPointer PublishEmptyLinearArrayPreviewOverlay(
        const Handle(OcctDocument)& theDocument) noexcept;

    //! Publish one source mesh/material with one proper-rigid world-space
    //! instance per explicitly owned Radial Array preview body. Geometry and
    //! appearance are copied once from the committed source presentation's
    //! existing cache. Main-thread only; no AIS discovery or meshing occurs.
    OverlayPointer PublishRadialArrayPreviewOverlay(
        const Handle(OcctDocument)& theDocument,
        const Handle(AIS_Shape)& theSourceShape,
        const std::vector<Handle(AIS_Shape)>& thePreviewShapes) noexcept;

    //! Publish the exact-empty semantic Radial Array overlay used for a stable
    //! near-zero angular step. Unsafe or unresolved states must return null
    //! instead, leaving the OCCT viewport authoritative.
    OverlayPointer PublishEmptyRadialArrayPreviewOverlay(
        const Handle(OcctDocument)& theDocument) noexcept;

#ifdef DEBUG
    enum class DebugTriangulationFailure : std::uint8_t {
        None = 0,
        Missing = 1,
        Incompatible = 2,
    };

    //! Test-only admission override. It exercises the same cached-mesh
    //! validation used by Build without changing live OCAF/AIS state.
    void DebugSetTriangulationFailure(
        DebugTriangulationFailure theFailure) noexcept;

    //! Counter seam for the forbidden snapshot-owned mesher path. Build must
    //! leave this at zero; OpenGL presentation meshing is outside this adapter.
    void DebugResetMesherInvocationCount() noexcept;
    std::uint64_t DebugMesherInvocationCount() const noexcept;

    //! Observe the last published document identity even during an owned
    //! preview. Does not capture a frame or admit uncommitted geometry.
    std::uint64_t DebugPublishedDocumentGeneration(
        const Handle(OcctDocument)& theDocument) const noexcept;
    //! Revision of that same committed publication, not the working OCAF state.
    std::uint64_t DebugPublishedModelRevision(
        const Handle(OcctDocument)& theDocument) const noexcept;
#endif

private:
    OverlayPointer PublishPresentationOverlayImpl(
        const Handle(OcctDocument)& theDocument,
        PresentationOverlayContent&& theContent,
        bool theAllowsMirrorPreview,
        bool theAllowsBooleanPreview,
        bool theAllowsChamferPreview,
        bool theAllowsLinearArrayPreview,
        bool theAllowsShellPreview,
        bool theAllowsRadialArrayPreview) noexcept;

    struct State;
    std::unique_ptr<State> myState;
};

} // namespace core3d::scene

#endif // Core3D_OcctSceneSnapshotBuilder_hpp
