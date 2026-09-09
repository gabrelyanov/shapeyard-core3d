#pragma once
// EXTERNAL AND UNCOMPILED. Wire into a DEBUG XCTest bridge only at a later
// guarded checkpoint. These policy cases do not prove real UIKit ownership.
#if DEBUG
#include "NativeEditAuthority.hpp"
#include <array>
#include <thread>

namespace core3d::debug {
inline std::array<bool,12> RunNativeManualIntentPolicyProbe() {
    using namespace authority;
    const std::array<std::uint8_t,16> nonce{1},otherNonce{2};
    int document=0,replacement=0;
    std::array<bool,12> outcomes{};
    // Tool open, immutable handoff and cancellation never restore old context.
    {
        NativeEditAuthority a(nonce);a.Adopt(&document);
        const auto before=a.Capture(&document,true,false);
        const auto first=a.BeginManualIntent(&document,true,false);
        if (before && first) {
            const auto handoff=*first;
            const bool blocked=!a.Capture(&document,true,false)
                && !a.BeginManualIntent(&document,true,false) && a.OwnsManualIntent(handoff);
            const auto ended=a.EndManualIntent(handoff,true,false);
            const auto after=a.Capture(&document,true,false);
            outcomes[0]=blocked && ended==ManualIntentEnd::Settled && after
                && !(*after==*before) && !a.Matches(*before,&document,true,false);
        }
    }
    // A delayed dismissal is not permission to release a newer tool.
    {
        NativeEditAuthority a(nonce);a.Adopt(&document);
        const auto old=a.BeginManualIntent(&document,true,false);
        if (old && a.EndManualIntent(*old,true,false)==ManualIntentEnd::Settled) {
            const auto current=a.BeginManualIntent(&document,true,false);
            outcomes[1]=current && a.EndManualIntent(*old,true,false)==ManualIntentEnd::NotCurrent
                && a.OwnsManualIntent(*current) && !a.Capture(&document,true,false);
        }
    }
    // Dismissal during worker/command/recovery retains the exact reservation.
    {
        NativeEditAuthority a(nonce);a.Adopt(&document);
        const auto work=a.BeginManualIntent(&document,true,false);
        outcomes[2]=work && a.EndManualIntent(*work,false,false)==ManualIntentEnd::Pending
            && a.EndManualIntent(*work,true,true)==ManualIntentEnd::Pending
            && a.OwnsManualIntent(*work) && !a.Capture(&document,true,false)
            && a.EndManualIntent(*work,true,false)==ManualIntentEnd::Settled;
    }
    // Successful adoption invalidates callbacks from the previous opening.
    {
        NativeEditAuthority a(nonce);a.Adopt(&document);
        const auto old=a.BeginManualIntent(&document,true,false);
        if (old && a.Adopt(&replacement)) {
            const auto current=a.BeginManualIntent(&replacement,true,false);
            outcomes[3]=current && a.EndManualIntent(*old,true,false)==ManualIntentEnd::NotCurrent
                && a.OwnsManualIntent(*current) && a.Valid();
        }
    }
    // Different native wrappers never accept each other's callback value.
    {
        NativeEditAuthority a(nonce),b(otherNonce);a.Adopt(&document);b.Adopt(&document);
        const auto ar=a.BeginManualIntent(&document,true,false),br=b.BeginManualIntent(&document,true,false);
        outcomes[4]=ar && br && a.EndManualIntent(*br,true,false)==ManualIntentEnd::NotCurrent
            && a.OwnsManualIntent(*ar) && a.Valid() && b.OwnsManualIntent(*br);
    }
    // Native pointer reuse after detachment still has a distinct opening.
    {
        NativeEditAuthority a(nonce);a.Adopt(&document);
        const auto before=a.Capture(&document,true,false);
        const auto old=a.BeginManualIntent(&document,true,false);
        a.Detach();const bool detached=!a.Capture(&document,true,false);
        if (old && before && a.Adopt(&document)) {
            const auto current=a.BeginManualIntent(&document,true,false);
            outcomes[5]=detached && current && a.EndManualIntent(*old,true,false)==ManualIntentEnd::NotCurrent
                && a.OwnsManualIntent(*current) && !a.Matches(*before,&document,true,false);
        }
    }
    // Native readiness rejects an initial tool; rejection does not manufacture
    // a reservation or change an otherwise valid issued context.
    {
        NativeEditAuthority a(nonce);a.Adopt(&document);
        const auto before=a.Capture(&document,true,false);
        outcomes[6]=before && !a.BeginManualIntent(&document,false,false)
            && !a.BeginManualIntent(&document,true,true)
            && !a.BeginManualIntent(&replacement,true,false)
            && a.Matches(*before,&document,true,false);
    }
    // Native changes during the tool remain stale after it settles.
    {
        NativeEditAuthority a(nonce);a.Adopt(&document);
        const auto before=a.Capture(&document,true,false);
        const auto work=a.BeginManualIntent(&document,true,false);
        a.TransactionBoundary(&document);a.SelectionBoundary(&document);
        outcomes[7]=before && work && a.OwnsManualIntent(*work)
            && a.EndManualIntent(*work,true,false)==ManualIntentEnd::Settled
            && !a.Matches(*before,&document,true,false) && a.Capture(&document,true,false).has_value();
    }
    // Intent serial never wraps and never reuses an older callback identity.
    {
        NativeEditAuthority a(nonce);a.Adopt(&document);
        const bool exhausted=a.DebugExhaustCounter(3);
        outcomes[8]=exhausted && !a.BeginManualIntent(&document,true,false)
            && !a.Valid() && !a.Capture(&document,true,false);
    }
    // Semantic-context overflow cannot issue a partially admitted reservation.
    {
        NativeEditAuthority a(nonce);a.Adopt(&document);
        const bool exhausted=a.DebugExhaustCounter(2);
        outcomes[9]=exhausted && !a.BeginManualIntent(&document,true,false) && !a.Valid();
    }
    // Overflow while ending reports unavailable, not a successful settlement.
    {
        NativeEditAuthority a(nonce);a.Adopt(&document);
        const auto work=a.BeginManualIntent(&document,true,false);
        outcomes[10]=work && a.DebugExhaustCounter(2)
            && a.EndManualIntent(*work,true,false)==ManualIntentEnd::Unavailable
            && !a.Valid() && !a.Capture(&document,true,false);
    }
    // A foreign callback poisons authority atomically without touching native
    // owner fields from that thread; inspect only after the join.
    {
        NativeEditAuthority a(nonce);a.Adopt(&document);
        const auto work=a.BeginManualIntent(&document,true,false);
        if (work) {
            ManualIntentEnd foreign=ManualIntentEnd::Settled;
            std::thread thread([&] { foreign=a.EndManualIntent(*work,true,false); });thread.join();
            outcomes[11]=foreign==ManualIntentEnd::Unavailable && !a.Valid()
                && !a.Capture(&document,true,false);
        }
    }
    return outcomes;
}
} // namespace core3d::debug
#endif
