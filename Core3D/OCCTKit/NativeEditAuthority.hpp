#pragma once
// Native observation and reservation policy. No public AI token is issued by the app.
// Integration must cover actual adoption, transaction/history and all semantic
// selection invalidations, and gate capture on the native command/preview fence.
#include <array>
#include <atomic>
#include <cstdint>
#include <limits>
#include <optional>
#include <thread>

namespace core3d::authority {
struct Stamp {
    std::array<std::uint8_t,16> instanceNonce{}; // Generated natively once per owner.
    std::uint64_t opening=0,edit=0,selection=0;
    bool operator==(const Stamp& b) const noexcept {
        return instanceNonce==b.instanceNonce && opening==b.opening && edit==b.edit && selection==b.selection;
    }
};
// Native-issued value. Bridge/UI code must retain this value in an opaque
// owner object; it must never reconstruct a reservation from wire fields.
class ManualIntentReservation final {
    friend class NativeEditAuthority;
    ManualIntentReservation(const std::array<std::uint8_t,16>& nonce,
                            std::uint64_t opening,std::uint64_t serial) noexcept
        : nonce_(nonce),opening_(opening),serial_(serial) {}
    std::array<std::uint8_t,16> nonce_;
    std::uint64_t opening_,serial_;
};
enum class ManualIntentEnd : std::uint8_t { Settled, Pending, NotCurrent, Unavailable };
// Retained only by the native replacement owner; never serialized.
class ReplacementReservation final {
    friend class NativeEditAuthority;
    ReplacementReservation(const std::array<std::uint8_t,16>& nonce,
                           std::uint64_t opening, std::uint64_t serial) noexcept
        : nonce_(nonce), opening_(opening), serial_(serial) {}
    std::array<std::uint8_t,16> nonce_;
    std::uint64_t opening_, serial_;
};
enum class ReplacementEnd : std::uint8_t { Adopted, Restored, Pending, NotCurrent, Unavailable };

// Internal admission must be derived from the actual native tool owner. A
// caller cannot classify arbitrary pending work as cancellable. Legacy tools
// have no ManualIntentReservation yet; migrated tools must supply their exact
// reservation. Unknown recovery or replacement work must use Reject.
enum class QueuedLoadAdmission : std::uint8_t {
    Reject, Ready, KnownCancellableLegacyTool, KnownCancellableManualIntent
};
class QueuedLoadReservation final {
    friend class NativeEditAuthority;
    QueuedLoadReservation(const std::array<std::uint8_t,16>& nonce,
                          std::uint64_t opening, std::uint64_t serial) noexcept
        : nonce_(nonce), opening_(opening), serial_(serial) {}
    std::array<std::uint8_t,16> nonce_;
    std::uint64_t opening_, serial_;
};
enum class QueuedLoadEnd : std::uint8_t {
    PrivateWorkSettled, Pending, NotCurrent, Unavailable
};

class NativeEditAuthority final {
public:
    explicit NativeEditAuthority(const std::array<std::uint8_t,16>& nativeNonce) noexcept {
        stamp_.instanceNonce=nativeNonce;
        bool any=false;for(auto byte:nativeNonce)any|=byte!=0;
        valid_=any;
    }
    NativeEditAuthority(const NativeEditAuthority&)=delete;
    NativeEditAuthority& operator=(const NativeEditAuthority&)=delete;
    // Internal native pointer equality only, never part of the public stamp.
    // Call exclusively after successful adoption, not provisional assignment.
    bool Adopt(const void* nativeDocument) noexcept {
        if(!OnOwner() || !Valid())return false;
        if(loadSerial_!=0 || replacementSerial_!=0 || nativeDocument==nullptr || nativeDocument==active_)return Fail();
        if(!Advance(stamp_.opening) || !Advance(stamp_.edit) || !Advance(stamp_.selection))return false;
        active_=nativeDocument;manualSerial_=0;return true;
    }
    void Detach() noexcept {
        if(!OnOwner())return;
        active_=nullptr;manualSerial_=0;replacementSerial_=0;loadSerial_=0;
    }
    // Conservatively invalidate on opening/commit/abort attempts. A transaction
    // rolled back to identical data must still invalidate an older AI proposal.
    // Inactive private candidate-document callbacks do not affect active work.
    void TransactionBoundary(const void* nativeDocument) noexcept {
        if(!OnOwner() || !Valid())return;
        if(nativeDocument==nullptr) {Fail();return;}
        if(nativeDocument==active_)Advance(stamp_.edit);
    }
    // History completion and attempts need explicit boundaries: bundled OCAF
    // callbacks do not provide all undo/redo events. Mutation unknown outcomes
    // remain fenced by the owner even if its public geometry looks unchanged.
    void HistoryBoundary(const void* nativeDocument) noexcept {
        TransactionBoundary(nativeDocument);
    }
    // Invoke at the semantic transition boundary, including no-op attempts and
    // failed repair/selection-mode changes. Invalidate before touching AIS; no
    // capture may be issued between an operation's preview/repair stages.
    void SelectionBoundary(const void* nativeDocument) noexcept {
        if(!OnOwner() || !Valid())return;
        if(nativeDocument==nullptr) {Fail();return;}
        // A replacement keeps active_ on the old document until candidate
        // presentation/interactor installation succeeds. Those same native
        // selection hooks also run against the provisionally assigned candidate.
        // The already-owned replacement fences all Capture/Matches calls; its
        // exact reservation alone may adopt or restore the settled document.
        // Keep these interactions invalidating without poisoning that owner.
        if(nativeDocument!=active_ && replacementSerial_==0) {Fail();return;}
        Advance(stamp_.selection);
    }
    // Private native policy only. Begin before accepting a touch tool request,
    // including before asynchronous camera/presentation preparation. A typed AI
    // command must never refresh stale permission through this manual API.
    std::optional<ManualIntentReservation> BeginManualIntent(
        const void* nativeDocument,bool nativeEditReady,bool nativeCommandOpen) noexcept {
        if (!Capture(nativeDocument,nativeEditReady,nativeCommandOpen)) return std::nullopt;
        if (!Advance(manualSequence_) || !Advance(stamp_.selection)) return std::nullopt;
        manualSerial_=manualSequence_;
        return ManualIntentReservation(stamp_.instanceNonce,stamp_.opening,manualSerial_);
    }
    // Picker-to-coordinate handoff carries the same reservation. It does not
    // End/Begin, and no new AI capture is available in the handoff gap.
    bool OwnsManualIntent(const ManualIntentReservation& reservation) noexcept {
        if (!OnOwner() || !Valid() || active_==nullptr || manualSerial_==0) return false;
        return reservation.nonce_==stamp_.instanceNonce
            && reservation.opening_==stamp_.opening && reservation.serial_==manualSerial_;
    }
    // nativeWorkSettled must be derived from real worker/session/recovery state,
    // excluding this reservation itself; modal disappearance is not proof.
    // A delayed/duplicate completion is harmless and cannot release newer work.
    ManualIntentEnd EndManualIntent(const ManualIntentReservation& reservation,
                                   bool nativeWorkSettled,bool nativeCommandOpen) noexcept {
        if (!OnOwner() || !Valid()) return ManualIntentEnd::Unavailable;
        if (!OwnsManualIntent(reservation)) return ManualIntentEnd::NotCurrent;
        if (!nativeWorkSettled || nativeCommandOpen) return ManualIntentEnd::Pending;
        if (!Advance(stamp_.selection)) return ManualIntentEnd::Unavailable;
        manualSerial_=0;
        return ManualIntentEnd::Settled;
    }
    // Begin before the first live context change. Private file validation alone
    // does not require a fence. The old document remains authoritative while a
    // candidate is provisionally installed in the viewer.
    std::optional<ReplacementReservation> BeginReplacement(
        const void* nativeDocument, bool nativeEditReady, bool nativeCommandOpen) noexcept {
        if (!Capture(nativeDocument, nativeEditReady, nativeCommandOpen)) return std::nullopt;
        if (!Advance(replacementSequence_) || !Advance(stamp_.edit)
            || !Advance(stamp_.selection)) return std::nullopt;
        replacementSerial_ = replacementSequence_;
        return ReplacementReservation(stamp_.instanceNonce, stamp_.opening, replacementSerial_);
    }
    // Atomically transfer an accepted queued manual request into the live
    // replacement window. No End/Begin gap and no refreshed AI permission.
    // Readiness excludes this matching reservation; actual worker/command or
    // recovery ownership must already be settled on the native main thread.
    std::optional<ReplacementReservation> PromoteManualIntentToReplacement(
        const ManualIntentReservation& intent, const void* nativeDocument,
        bool nativeEditReady, bool nativeCommandOpen) noexcept {
        if (!OwnsManualIntent(intent) || nativeDocument != active_
            || replacementSerial_ != 0 || loadSerial_ != 0 || !nativeEditReady || nativeCommandOpen)
            return std::nullopt;
        if (!Advance(replacementSequence_) || !Advance(stamp_.edit)
            || !Advance(stamp_.selection)) return std::nullopt;
        replacementSerial_ = replacementSequence_;
        manualSerial_ = 0;
        return ReplacementReservation(stamp_.instanceNonce, stamp_.opening, replacementSerial_);
    }
    // Acquire before public-controller cancellation or any queued load work.
    // This reservation fences captures without claiming the older tool settled.
    // All methods remain native-owner-thread only. Immutable input and request
    // identity must be retained by the integration owner, never by wire fields.
    std::optional<QueuedLoadReservation> BeginQueuedLoad(
        const void* nativeDocument, QueuedLoadAdmission admission,
        bool nativeEditReady, bool nativeCommandOpen,
        const ManualIntentReservation* cancellableIntent = nullptr) noexcept {
        if (!OnOwner() || !Valid() || active_ == nullptr || nativeDocument != active_
            || loadSerial_ != 0 || replacementSerial_ != 0) return std::nullopt;
        switch (admission) {
            case QueuedLoadAdmission::Ready:
                if (!nativeEditReady || nativeCommandOpen || manualSerial_ != 0
                    || cancellableIntent != nullptr) return std::nullopt;
                break;
            case QueuedLoadAdmission::KnownCancellableLegacyTool:
                if (manualSerial_ != 0 || cancellableIntent != nullptr) return std::nullopt;
                break;
            case QueuedLoadAdmission::KnownCancellableManualIntent:
                if (cancellableIntent == nullptr || !OwnsManualIntent(*cancellableIntent))
                    return std::nullopt;
                break;
            default:
                return std::nullopt;
        }
        if (!Advance(loadSequence_) || !Advance(stamp_.selection)) return std::nullopt;
        loadSerial_ = loadSequence_;
        return QueuedLoadReservation(stamp_.instanceNonce, stamp_.opening, loadSerial_);
    }
    bool OwnsQueuedLoad(const QueuedLoadReservation& reservation) noexcept {
        if (!OnOwner() || !Valid() || active_ == nullptr || loadSerial_ == 0) return false;
        return reservation.nonce_ == stamp_.instanceNonce
            && reservation.opening_ == stamp_.opening && reservation.serial_ == loadSerial_;
    }
    // ONLY private validation/rejection/cancellation before promotion. Settled
    // means this request's worker and owned temporary artifacts are finished.
    // No assertion is made about a retained Bevel or ordinary recovery ledger;
    // its owner still controls nativeEditReady. Do not issue a fresh AI context.
    // After promotion this exact reservation is no longer current, preventing
    // this path from releasing the retained replacement restoration fence.
    QueuedLoadEnd EndQueuedLoadPrivateWork(const QueuedLoadReservation& reservation,
                                         bool privateWorkSettled) noexcept {
        if (!OnOwner() || !Valid()) return QueuedLoadEnd::Unavailable;
        if (!OwnsQueuedLoad(reservation)) return QueuedLoadEnd::NotCurrent;
        if (!privateWorkSettled) return QueuedLoadEnd::Pending;
        if (!Advance(stamp_.selection)) return QueuedLoadEnd::Unavailable;
        loadSerial_ = 0;
        return QueuedLoadEnd::PrivateWorkSettled;
    }
    // Called once on the native owner immediately before any live replacement.
    // Real old-tool/worker/command recovery must be settled; readiness excludes
    // only this matching load reservation. Do not release then re-acquire.
    std::optional<ReplacementReservation> PromoteQueuedLoadToReplacement(
        const QueuedLoadReservation& reservation, const void* nativeDocument,
        bool nativeEditReady, bool nativeCommandOpen) noexcept {
        if (!OwnsQueuedLoad(reservation) || nativeDocument != active_
            || manualSerial_ != 0 || replacementSerial_ != 0
            || !nativeEditReady || nativeCommandOpen) return std::nullopt;
        if (!Advance(replacementSequence_) || !Advance(stamp_.edit)
            || !Advance(stamp_.selection)) return std::nullopt;
        replacementSerial_ = replacementSequence_;
        loadSerial_ = 0;
        return ReplacementReservation(stamp_.instanceNonce, stamp_.opening, replacementSerial_);
    }
    bool OwnsReplacement(const ReplacementReservation& reservation) noexcept {
        if (!OnOwner() || !Valid() || active_ == nullptr || replacementSerial_ == 0) return false;
        return reservation.nonce_ == stamp_.instanceNonce
            && reservation.opening_ == stamp_.opening && reservation.serial_ == replacementSerial_;
    }
    // settledDocument comes from the actual viewer after successful interactor
    // installation or confirmed restoration. Restoring just its document pointer
    // is insufficient. Unknown presentation/worker outcomes keep the fence.
    ReplacementEnd EndReplacement(const ReplacementReservation& reservation,
        const void* settledDocument, bool candidateAccepted, bool nativeWorkSettled,
        bool nativeCommandOpen) noexcept {
        if (!OnOwner() || !Valid()) return ReplacementEnd::Unavailable;
        if (!OwnsReplacement(reservation)) return ReplacementEnd::NotCurrent;
        if (!nativeWorkSettled || nativeCommandOpen) return ReplacementEnd::Pending;
        if (settledDocument == nullptr
            || (candidateAccepted ? settledDocument == active_ : settledDocument != active_)) {
            Fail(); return ReplacementEnd::Unavailable;
        }
        if (candidateAccepted && !Advance(stamp_.opening)) return ReplacementEnd::Unavailable;
        if (!Advance(stamp_.edit) || !Advance(stamp_.selection)) return ReplacementEnd::Unavailable;
        if (candidateAccepted) active_ = settledDocument;
        replacementSerial_ = 0;
        return candidateAccepted ? ReplacementEnd::Adopted : ReplacementEnd::Restored;
    }
    std::optional<Stamp> Capture(const void* nativeDocument,bool nativeEditReady,
                               bool nativeCommandOpen) noexcept {
        if(!OnOwner() || !Valid() || active_==nullptr || nativeDocument!=active_
            || manualSerial_!=0 || replacementSerial_!=0 || loadSerial_!=0 || !nativeEditReady || nativeCommandOpen)return std::nullopt;
        return stamp_;
    }
    bool Matches(const Stamp& expected,const void* nativeDocument,
                 bool nativeEditReady,bool nativeCommandOpen) noexcept {
        const auto actual=Capture(nativeDocument,nativeEditReady,nativeCommandOpen);
        return actual && *actual==expected;
    }
    bool Valid() const noexcept {
        return owner_==std::this_thread::get_id()
            && !foreignObserved_.load(std::memory_order_relaxed) && valid_;
    }
#if DEBUG
    // Monotonic saturation only, for isolated overflow fixtures. No reset/wrap.
    bool DebugExhaustCounter(unsigned counter) noexcept {
        if (!OnOwner() || !Valid() || active_ == nullptr || counter > 5) return false;
        auto& value = counter == 0 ? stamp_.opening : counter == 1 ? stamp_.edit
            : counter == 2 ? stamp_.selection : counter == 3 ? manualSequence_
            : counter == 4 ? replacementSequence_ : loadSequence_;
        value = std::numeric_limits<std::uint64_t>::max();
        return true;
    }
#endif
private:
    bool OnOwner() noexcept {
        if(owner_==std::this_thread::get_id())return true;
        foreignObserved_.store(true,std::memory_order_relaxed);return false;
    }
    bool Fail() noexcept {valid_=false;active_=nullptr;manualSerial_=0;replacementSerial_=0;loadSerial_=0;return false;}
    bool Advance(std::uint64_t& generation) noexcept {
        if(generation==std::numeric_limits<std::uint64_t>::max())return Fail();
        ++generation;return true;
    }
    const std::thread::id owner_=std::this_thread::get_id();
    std::atomic_bool foreignObserved_{false};
    const void* active_=nullptr;
    // Sequence never resets across detach/adoption. Zero means no live intent.
    std::uint64_t manualSequence_=0,manualSerial_=0;
    std::uint64_t replacementSequence_=0,replacementSerial_=0;
    std::uint64_t loadSequence_=0,loadSerial_=0;
    Stamp stamp_;
    bool valid_=false;
};
} // namespace core3d::authority
