#pragma once
// These isolated policy outcomes do not qualify live loading, tool cancellation,
// UI or public AI authority.
#if DEBUG
#include "NativeEditAuthority.hpp"
#include <array>
#include <thread>
namespace core3d::debug {
inline std::array<bool,16> RunNativeQueuedLoadPolicyProbe() {
    using namespace authority;
    const std::array<std::uint8_t,16> nonce{1}, foreignNonce{2};
    int oldDocument=0, candidate=0;
    std::array<bool,16> result{};
    // A private rejection invalidates old observations without adopting a file.
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto before=a.Capture(&oldDocument,true,false);
        const auto load=a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false);
        if (before && load) {
            const bool fenced=!a.Capture(&oldDocument,true,false)
                && !a.BeginManualIntent(&oldDocument,true,false)
                && !a.BeginReplacement(&oldDocument,true,false)
                && !a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false);
            const bool pending=a.EndQueuedLoadPrivateWork(*load,false)==QueuedLoadEnd::Pending
                && a.OwnsQueuedLoad(*load);
            const auto end=a.EndQueuedLoadPrivateWork(*load,true);
            const auto after=a.Capture(&oldDocument,true,false);
            result[0]=fenced && pending && end==QueuedLoadEnd::PrivateWorkSettled
                && after && after->opening==before->opening && after->edit==before->edit
                && after->selection>before->selection && !a.Matches(*before,&oldDocument,true,false);
        }
    }
    // A rejected preparation never declares unresolved legacy recovery ready.
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto load=a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::KnownCancellableLegacyTool,false,true);
        result[1]=load && !a.PromoteQueuedLoadToReplacement(*load,&oldDocument,false,true)
            && a.EndQueuedLoadPrivateWork(*load,true)==QueuedLoadEnd::PrivateWorkSettled
            && !a.Capture(&oldDocument,false,true) && !a.Capture(&oldDocument,false,false)
            && a.Capture(&oldDocument,true,false).has_value(); // Only AFTER external recovery settles.
    }
    // A migrated tool keeps its exact reservation when the load is rejected.
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto tool=a.BeginManualIntent(&oldDocument,true,false);
        if (tool) {
            const auto load=a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::KnownCancellableManualIntent,false,true,&*tool);
            result[2]=load && a.OwnsManualIntent(*tool)
                && a.EndManualIntent(*tool,false,true)==ManualIntentEnd::Pending
                && a.EndQueuedLoadPrivateWork(*load,true)==QueuedLoadEnd::PrivateWorkSettled
                && a.OwnsManualIntent(*tool) && !a.Capture(&oldDocument,true,false)
                && a.EndManualIntent(*tool,true,false)==ManualIntentEnd::Settled
                && a.Capture(&oldDocument,true,false).has_value();
        }
    }
    // Settling the tool leaves the queued load fenced until atomic promotion.
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto tool=a.BeginManualIntent(&oldDocument,true,false);
        if (tool) {
            const auto load=a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::KnownCancellableManualIntent,false,false,&*tool);
            if (load) {
                const bool blocked=!a.PromoteManualIntentToReplacement(*tool,&oldDocument,true,false)
                    && !a.PromoteQueuedLoadToReplacement(*load,&oldDocument,true,false);
                const bool settled=a.EndManualIntent(*tool,true,false)==ManualIntentEnd::Settled
                    && a.OwnsQueuedLoad(*load) && !a.Capture(&oldDocument,true,false);
                const auto replacement=a.PromoteQueuedLoadToReplacement(*load,&oldDocument,true,false);
                result[3]=blocked && settled && replacement && a.OwnsReplacement(*replacement)
                    && !a.OwnsQueuedLoad(*load)
                    && a.EndQueuedLoadPrivateWork(*load,true)==QueuedLoadEnd::NotCurrent
                    && a.EndReplacement(*replacement,&candidate,true,true,false)==ReplacementEnd::Adopted
                    && a.Capture(&candidate,true,false).has_value();
            }
        }
    }
    // Unknown work, wrong document and invalid Ready classifications are rejected.
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto before=a.Capture(&oldDocument,true,false);
        result[4]=before
            && !a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Reject,false,true)
            && !a.BeginQueuedLoad(&candidate,QueuedLoadAdmission::Ready,true,false)
            && !a.BeginQueuedLoad(nullptr,QueuedLoadAdmission::Ready,true,false)
            && !a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,false,false)
            && !a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,true)
            && a.Matches(*before,&oldDocument,true,false);
    }
    // Legacy classification cannot bypass a migrated manual owner.
    {
        NativeEditAuthority a(nonce), b(foreignNonce); a.Adopt(&oldDocument); b.Adopt(&oldDocument);
        const auto tool=a.BeginManualIntent(&oldDocument,true,false);
        const auto foreign=b.BeginManualIntent(&oldDocument,true,false);
        result[5]=tool && foreign
            && !a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false)
            && !a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::KnownCancellableLegacyTool,false,false)
            && !a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::KnownCancellableManualIntent,false,false)
            && !a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::KnownCancellableManualIntent,false,false,&*foreign)
            && a.OwnsManualIntent(*tool) && a.Valid();
    }
    // Foreign load completion/promotion cannot release another owner.
    {
        NativeEditAuthority a(nonce), b(foreignNonce); a.Adopt(&oldDocument); b.Adopt(&oldDocument);
        const auto load=a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false);
        const auto foreign=b.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false);
        result[6]=load && foreign
            && a.EndQueuedLoadPrivateWork(*foreign,true)==QueuedLoadEnd::NotCurrent
            && !a.PromoteQueuedLoadToReplacement(*foreign,&oldDocument,true,false)
            && a.OwnsQueuedLoad(*load) && a.Valid();
    }
    // A delayed first completion cannot clear the second request.
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto first=a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false);
        if (first && a.EndQueuedLoadPrivateWork(*first,true)==QueuedLoadEnd::PrivateWorkSettled) {
            const auto second=a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false);
            result[7]=second && a.EndQueuedLoadPrivateWork(*first,true)==QueuedLoadEnd::NotCurrent
                && !a.PromoteQueuedLoadToReplacement(*first,&oldDocument,true,false)
                && a.OwnsQueuedLoad(*second) && !a.Capture(&oldDocument,true,false);
        }
    }
    // Promotion remains blocked by actual worker/command readiness.
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto load=a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false);
        result[8]=load && !a.PromoteQueuedLoadToReplacement(*load,&candidate,true,false)
            && !a.PromoteQueuedLoadToReplacement(*load,&oldDocument,false,false)
            && !a.PromoteQueuedLoadToReplacement(*load,&oldDocument,true,true)
            && a.OwnsQueuedLoad(*load) && a.Valid();
    }
    // Unknown restoration retains replacement ownership after load promotion.
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto before=a.Capture(&oldDocument,true,false);
        const auto load=a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false);
        if (before && load) {
            const auto replacement=a.PromoteQueuedLoadToReplacement(*load,&oldDocument,true,false);
            if (replacement) {
                const bool pending=a.EndReplacement(*replacement,&oldDocument,false,false,false)==ReplacementEnd::Pending
                    && a.EndQueuedLoadPrivateWork(*load,true)==QueuedLoadEnd::NotCurrent
                    && !a.Capture(&oldDocument,true,false) && a.OwnsReplacement(*replacement);
                const auto restored=a.EndReplacement(*replacement,&oldDocument,false,true,false);
                const auto after=a.Capture(&oldDocument,true,false);
                result[9]=pending && restored==ReplacementEnd::Restored && after
                    && after->opening==before->opening && after->edit>before->edit
                    && after->selection>before->selection;
            }
        }
    }
    // Adoption invalidates the earlier opening and every delayed load callback.
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto load=a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false);
        if (load) {
            const auto replacement=a.PromoteQueuedLoadToReplacement(*load,&oldDocument,true,false);
            if (replacement && a.EndReplacement(*replacement,&candidate,true,true,false)==ReplacementEnd::Adopted) {
                const auto next=a.BeginQueuedLoad(&candidate,QueuedLoadAdmission::Ready,true,false);
                result[10]=next && a.EndQueuedLoadPrivateWork(*load,true)==QueuedLoadEnd::NotCurrent
                    && !a.PromoteQueuedLoadToReplacement(*load,&candidate,true,false)
                    && a.OwnsQueuedLoad(*next);
            }
        }
    }
    // Native teardown makes old completions inert; serials do not restart.
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto load=a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false);
        a.Detach();
        const bool adopted=a.Adopt(&candidate);
        const auto next=a.BeginQueuedLoad(&candidate,QueuedLoadAdmission::Ready,true,false);
        result[11]=load && adopted && next
            && a.EndQueuedLoadPrivateWork(*load,true)==QueuedLoadEnd::NotCurrent
            && a.OwnsQueuedLoad(*next);
    }
    // Direct adoption cannot bypass an outstanding queued load.
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto load=a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false);
        result[12]=load && !a.Adopt(&candidate) && !a.Valid()
            && !a.Capture(&candidate,true,false);
    }
    // Sequence saturation poisons rather than wrapping a request identity.
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        result[13]=a.DebugExhaustCounter(5)
            && !a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false)
            && !a.Valid();
    }
    // Replacement counter saturation cannot expose a gap during promotion.
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto load=a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false);
        result[14]=load && a.DebugExhaustCounter(4)
            && !a.PromoteQueuedLoadToReplacement(*load,&oldDocument,true,false)
            && !a.Valid() && !a.Capture(&oldDocument,true,false);
    }
    // Foreign-thread delivery fails closed even with an exact copied reservation.
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto load=a.BeginQueuedLoad(&oldDocument,QueuedLoadAdmission::Ready,true,false);
        if (load) {
            auto end=QueuedLoadEnd::PrivateWorkSettled;
            std::thread worker([&] { end=a.EndQueuedLoadPrivateWork(*load,true); }); worker.join();
            result[15]=end==QueuedLoadEnd::Unavailable && !a.Valid()
                && !a.Capture(&oldDocument,true,false);
        }
    }
    return result;
}
}
#endif
