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

#include <memory>
#include <optional>

class AIS_InteractiveContext;
class OcctDocument;
class V3d_View;

namespace core3d::scene {

class OcctSceneSnapshotBuilder final {
public:
    using SnapshotPointer = std::shared_ptr<const SceneSnapshot>;

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

private:
    struct State;
    std::unique_ptr<State> myState;
};

} // namespace core3d::scene

#endif // Core3D_OcctSceneSnapshotBuilder_hpp
