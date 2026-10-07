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

struct PrivateExportDecalArtifact final {
    std::shared_ptr<const SceneSnapshot> scene;
    std::vector<PrivateExportFaceCorrespondence> correspondence;
    std::vector<std::string> inputKeys;
    std::vector<std::string> bakeSeals;
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
};

//! Bounded values observed from the executed private GLB derivative path.
//! Digests and numeric streams are copied; labels, handles and authority
//! tokens remain native and never escape this seam.
struct DebugPrivateExportDecalObservation final {
    std::uint64_t ownerCount = 0;
    std::uint64_t requiredReceiverCount = 0;
    std::uint64_t producedReceiverCount = 0;
    std::uint64_t savedTriangleCount = 0;
    std::uint64_t savedCornerCount = 0;
    std::uint64_t finalTriangleCount = 0;
    std::uint64_t finalCornerCount = 0;
    std::uint64_t operationWork = 0;
    std::uint64_t operationResidentBytes = 0;
    std::vector<std::string> canonicalReadDigests;
    std::vector<std::string> sourceReadDigests;
    std::string occluderDigest;
    std::vector<PrivateExportFaceCorrespondence> correspondence;
    std::vector<std::string> inputKeys;
    std::vector<std::string> bakeSeals;
    std::vector<std::string> outputRoles;
    std::vector<std::uint64_t> outputRoleBytes;
    std::uint64_t layoutResolution = 0;
    std::uint64_t layoutChartCount = 0;
    std::string failureStage;
    bool captureReached = false;
    bool finalProductionReached = false;
    bool publicationReached = false;
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

class OcctSceneSnapshotBuilder final {
public:
    using SnapshotPointer = std::shared_ptr<const SceneSnapshot>;
    using OverlayPointer = std::shared_ptr<const PresentationOverlaySnapshot>;

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
        const std::function<bool()>& cancelled
#ifdef DEBUG
        , DebugPrivateExportDecalFault debugFault =
            DebugPrivateExportDecalFault::None
        , DebugPrivateExportDecalObservation* debugObservation = nullptr
#endif
        ) noexcept;

    static bool CapturePrivateExportDecals(
        const Handle(OcctDocument)& privateDocument,
        const SceneSnapshot& wholeCommittedScene,
        bool selectedObjectsOnly,
        const std::function<bool()>& cancelled,
        PrivateDecalCapturePointer& capture) noexcept;

    static bool FinalizePrivateExportDecals(
        PrivateDecalCapture& capture,
        const SceneSnapshot& finalScene,
        const std::vector<PrivateExportFaceCorrespondence>& correspondence,
        const std::vector<PrivateExportFinalMesh>& emittedMeshes,
        const std::function<bool()>& cancelled,
        PrivateExportDecalArtifact& artifact) noexcept;

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
