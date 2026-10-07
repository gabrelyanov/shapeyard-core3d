#pragma once

#include "NativeEditAuthority.hpp"
#include "../Scene/SceneSnapshot.hpp"

#include <Standard_Handle.hxx>
#include <TopoDS_Shape.hxx>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class TDF_Data;
class TDocStd_Document;

namespace core3d {
class Core3DViewer;
class OrdinaryEditCommandStamp;

namespace native_opening {
namespace detail { struct State; }

//! One persistent object whose real AIS presentation must be reconciled to
//! the committed OCAF state. The entity identifier is authority supplied by
//! the owning stager's validated plan, never caller text; the optional
//! previous shape only recognizes a stale presentation whose label is gone.
struct CommittedEditItem final {
    std::string entityIdentifier;
    TopoDS_Shape previousShape;
};

//! The affected labels/identities of one known committed native edit, split
//! by plan role. The viewer proves the actual document state for each item;
//! a stale scene value can never mint authority here.
struct CommittedEditPublication final {
    std::vector<CommittedEditItem> created;
    std::vector<CommittedEditItem> replaced;
    std::vector<CommittedEditItem> removed;
};

//! Immutable evidence from the renderer's actual committed-scene publication.
//! Construction is viewer-only: serialized fields and callers cannot mint it.
class Fence final {
public:
    const authority::Stamp& stamp() const noexcept { return stamp_; }
    const std::string& publicationSourceIdentifier() const noexcept { return publicationSourceIdentifier_; }
    std::uint64_t documentGeneration() const noexcept { return documentGeneration_; }
    std::uint64_t modelRevision() const noexcept { return modelRevision_; }
    scene::ElementKind selectionMode() const noexcept { return selectionMode_; }
    const std::vector<scene::ElementIdentifier>& selection() const noexcept { return selection_; }
    const std::optional<scene::ElementIdentifier>& hovered() const noexcept { return hovered_; }
    const Handle(TDocStd_Document)& document() const noexcept { return document_; }
    const Handle(TDF_Data)& data() const noexcept { return data_; }
    double metersPerUnit() const noexcept { return metersPerUnit_; }
    const std::vector<std::string>& sourceReceipts() const noexcept { return sourceReceipts_; }

private:
    friend class core3d::Core3DViewer;
    friend class Context;
    Fence() = default;
    authority::Stamp stamp_;
    std::string publicationSourceIdentifier_;
    std::uint64_t documentGeneration_ = 0;
    std::uint64_t modelRevision_ = 0;
    scene::ElementKind selectionMode_ = scene::ElementKind::None;
    std::vector<scene::ElementIdentifier> selection_;
    std::optional<scene::ElementIdentifier> hovered_;
    Handle(TDocStd_Document) document_;
    Handle(TDF_Data) data_;
    double metersPerUnit_ = 0.0;
    std::vector<std::string> sourceReceipts_;
};

class Context;

//! Exact ownership of one OCAF command. It is neither serializable nor
//! constructible outside the issuing Context. An unsettled lease retains the
//! recovery fence and therefore blocks every other opening.
class CommandLease final {
public:
    ~CommandLease();
    CommandLease(const CommandLease&) = delete;
    CommandLease& operator=(const CommandLease&) = delete;
    const Handle(TDF_Data)& data() const noexcept { return data_; }
    int transaction() const noexcept { return transaction_; }
    bool ownsOpenCommand() const noexcept;
    bool commit() noexcept;
    bool abort() noexcept;

private:
    friend class Context;
    CommandLease(std::shared_ptr<Context>, const Handle(TDocStd_Document)&,
                 const Handle(TDF_Data)&, int, std::uint64_t,
                 bool borrowed = false) noexcept;
    void retainRecovery() noexcept;
    std::shared_ptr<Context> context_;
    Handle(TDocStd_Document) document_;
    Handle(TDF_Data) data_;
    int transaction_ = 0;
    std::uint64_t marker_ = 0;
    bool settled_ = false;
    bool borrowed_ = false;
    OrdinaryEditCommandStamp* ordinary_ = nullptr;
};

//! One viewer-owned native opening. The weak owner link deliberately prevents
//! an editor from prolonging an adopted viewer/document lifetime.
class Context final : public std::enable_shared_from_this<Context> {
public:
    ~Context();
    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;
    const Fence& openingFence() const noexcept { return openingFence_; }
    std::shared_ptr<const Fence> recapture(
        std::uint32_t viewportWidth, std::uint32_t viewportHeight) const noexcept;
    bool isCurrent(std::uint32_t viewportWidth, std::uint32_t viewportHeight) const noexcept;
    //! Private D3 selected-path handoff.  The viewer verifies that every
    //! opening/document fence component is unchanged except the native
    //! selection component, and returns a newly registered issuer.  It never
    //! updates this context or makes an old token current again.
    std::shared_ptr<Context> captureSelectionReplacement(
        std::uint32_t viewportWidth, std::uint32_t viewportHeight) const noexcept;
    std::shared_ptr<CommandLease> beginCommandLease(
        const Fence& expected, std::uint32_t viewportWidth,
        std::uint32_t viewportHeight) noexcept;
    //! Borrows the exact command already opened by the ordinary coordinator.
    //! The ordinary stamp remains the sole close and recovery authority.
    std::shared_ptr<CommandLease> borrowCommandLease(
        OrdinaryEditCommandStamp&) noexcept;
    bool reconcileRecovery(bool exactStateKnown) noexcept;
    //! Post-commit publication of a known committed native result, after
    //! native readback and for the same live document/data and this issuing
    //! context. Reconciles the owning viewer's real AIS presentations through
    //! the production presentation path; runs no OCAF command, no second
    //! geometry edit and no extra undo step. On failure the shared recovery
    //! fence is armed and the same plan is retained for exact recovery.
    bool publishCommittedEdit(const CommittedEditPublication& publication) noexcept;
    //! Retains the prepared plan of an edit whose close is unknown so exact
    //! recovery can reconcile the actual committed/aborted document state.
    //! No fence state changes here; the unknown close already armed it.
    void retainUnprovenEdit(const CommittedEditPublication& publication) noexcept;
#if DEBUG
    //! Row-265 lifecycle probe seam: the next real close at a lease issued by
    //! this context runs the real CommitCommand and is then reported unproven
    //! at the close/lease result boundary, arming the real shared recovery
    //! fence. DEBUG-only; Release contains neither flag nor branch.
    void debugReportNextCloseUnproven() noexcept;
#endif

private:
    friend class core3d::Core3DViewer;
    friend class CommandLease;
    Context(std::weak_ptr<detail::State>, std::uint64_t, Fence) noexcept;
    bool ownsMarker(std::uint64_t marker) const noexcept;
    void finishMarker(std::uint64_t marker, bool recovery,
                      bool retainForPublication = false) noexcept;
    std::weak_ptr<detail::State> state_;
    const std::uint64_t identifier_;
    const Fence openingFence_;
};

bool SameFence(const Fence&, const Fence&) noexcept;

} // namespace native_opening
} // namespace core3d
