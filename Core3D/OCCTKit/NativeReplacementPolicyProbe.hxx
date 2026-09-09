#pragma once
// External, uncompiled acceptance preparation. Requires guarded native XCTest.
#if DEBUG
#include "NativeEditAuthority.hpp"
#include <array>
#include <thread>

namespace core3d::debug {
inline std::array<bool,8> RunNativeReplacementPolicyProbe() {
    using namespace authority;
    const std::array<std::uint8_t,16> nonce{1}, otherNonce{2};
    int oldDocument=0, candidate=0;
    std::array<bool,8> result{};
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto before=a.Capture(&oldDocument,true,false);
        const auto lease=a.BeginReplacement(&oldDocument,true,false);
        if (before && lease) {
            const bool fenced=!a.Capture(&oldDocument,true,false)
                && !a.Capture(&candidate,true,false)
                && !a.BeginManualIntent(&oldDocument,true,false)
                && !a.BeginReplacement(&oldDocument,true,false);
            a.TransactionBoundary(&candidate); // Isolated candidate is not adoption.
            const auto end=a.EndReplacement(*lease,&oldDocument,false,true,false);
            const auto after=a.Capture(&oldDocument,true,false);
            result[0]=fenced && end==ReplacementEnd::Restored && after
                && after->opening==before->opening && !(*after==*before);
        }
    }
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto before=a.Capture(&oldDocument,true,false);
        const auto lease=a.BeginReplacement(&oldDocument,true,false);
        result[1]=before && lease
            && a.EndReplacement(*lease,&candidate,true,true,false)==ReplacementEnd::Adopted
            && !a.Capture(&oldDocument,true,false)
            && !a.Matches(*before,&candidate,true,false)
            && a.Capture(&candidate,true,false).has_value();
    }
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto lease=a.BeginReplacement(&oldDocument,true,false);
        result[2]=lease
            && a.EndReplacement(*lease,&oldDocument,false,false,false)==ReplacementEnd::Pending
            && a.EndReplacement(*lease,&oldDocument,false,true,true)==ReplacementEnd::Pending
            && a.OwnsReplacement(*lease) && !a.Capture(&oldDocument,true,false)
            && a.EndReplacement(*lease,&oldDocument,false,true,false)==ReplacementEnd::Restored;
    }
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto old=a.BeginReplacement(&oldDocument,true,false);
        if (old && a.EndReplacement(*old,&oldDocument,false,true,false)==ReplacementEnd::Restored) {
            const auto current=a.BeginReplacement(&oldDocument,true,false);
            result[3]=current
                && a.EndReplacement(*old,&oldDocument,false,true,false)==ReplacementEnd::NotCurrent
                && a.OwnsReplacement(*current) && !a.Capture(&oldDocument,true,false);
        }
    }
    {
        NativeEditAuthority a(nonce), b(otherNonce); a.Adopt(&oldDocument); b.Adopt(&oldDocument);
        const auto al=a.BeginReplacement(&oldDocument,true,false);
        const auto bl=b.BeginReplacement(&oldDocument,true,false);
        result[4]=al && bl
            && a.EndReplacement(*bl,&oldDocument,false,true,false)==ReplacementEnd::NotCurrent
            && a.OwnsReplacement(*al) && a.Valid();
    }
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto lease=a.BeginReplacement(&oldDocument,true,false);
        a.Detach(); a.Adopt(&oldDocument); // Same address, different opening.
        const auto current=a.BeginReplacement(&oldDocument,true,false);
        result[5]=lease && current
            && a.EndReplacement(*lease,&oldDocument,false,true,false)==ReplacementEnd::NotCurrent
            && a.OwnsReplacement(*current);
    }
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto lease=a.BeginReplacement(&oldDocument,true,false);
        const bool badRestore=lease
            && a.EndReplacement(*lease,&candidate,false,true,false)==ReplacementEnd::Unavailable
            && !a.Valid();
        NativeEditAuthority b(otherNonce); b.Adopt(&oldDocument);
        result[6]=badRestore && b.DebugExhaustCounter(4)
            && !b.BeginReplacement(&oldDocument,true,false) && !b.Valid();
    }
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto lease=a.BeginReplacement(&oldDocument,true,false);
        if (lease) {
            ReplacementEnd end=ReplacementEnd::Restored;
            std::thread thread([&] {
                end=a.EndReplacement(*lease,&oldDocument,false,true,false);
            });
            thread.join();
            result[7]=end==ReplacementEnd::Unavailable && !a.Valid()
                && !a.Capture(&oldDocument,true,false);
        }
    }
    return result;
}
} // namespace core3d::debug
#endif
