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

#include <Standard_Handle.hxx>
#include <TDF_Label.hxx>

#include <memory>
#include <optional>
#include <cstdint>
#include <vector>

class AIS_InteractiveContext;
class AIS_Shape;
class OcctDocument;
class V3d_View;

namespace core3d::scene {

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
    SnapshotPointer Build(
        const Handle(OcctDocument)& theDocument,
        const Handle(AIS_InteractiveContext)& theContext,
        const Handle(V3d_View)& theView,
        const UInt2& theViewportPixels) noexcept;

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
#endif

private:
    OverlayPointer PublishPresentationOverlayImpl(
        const Handle(OcctDocument)& theDocument,
        PresentationOverlayContent&& theContent,
        bool theAllowsMirrorPreview,
        bool theAllowsBooleanPreview,
        bool theAllowsChamferPreview,
        bool theAllowsLinearArrayPreview,
        bool theAllowsShellPreview) noexcept;

    struct State;
    std::unique_ptr<State> myState;
};

} // namespace core3d::scene

#endif // Core3D_OcctSceneSnapshotBuilder_hpp
