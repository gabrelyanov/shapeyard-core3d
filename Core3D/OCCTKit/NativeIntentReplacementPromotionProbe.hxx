#pragma once
// External uncompiled native policy outcomes; no runtime authority qualification.
#if DEBUG
#include "NativeEditAuthority.hpp"
#include <array>
#include <thread>
namespace core3d::debug {
inline std::array<bool,7> RunNativeIntentReplacementPromotionProbe() {
    using namespace authority;
    const std::array<std::uint8_t,16> nonce{1}, otherNonce{2};
    int oldDocument=0,candidate=0;
    std::array<bool,7> result{};
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto before=a.Capture(&oldDocument,true,false);
        const auto intent=a.BeginManualIntent(&oldDocument,true,false);
        if (before && intent) {
            const bool initiallyFenced=!a.BeginReplacement(&oldDocument,true,false)
                && !a.Capture(&oldDocument,true,false);
            const auto replacement=a.PromoteManualIntentToReplacement(*intent,&oldDocument,true,false);
            result[0]=initiallyFenced && replacement && !a.OwnsManualIntent(*intent)
                && a.OwnsReplacement(*replacement) && !a.Capture(&oldDocument,true,false)
                && a.EndManualIntent(*intent,true,false)==ManualIntentEnd::NotCurrent
                && a.EndReplacement(*replacement,&oldDocument,false,true,false)==ReplacementEnd::Restored
                && !a.Matches(*before,&oldDocument,true,false)
                && a.Capture(&oldDocument,true,false).has_value();
        }
    }
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto intent=a.BeginManualIntent(&oldDocument,true,false);
        if (intent) {
            const auto replacement=a.PromoteManualIntentToReplacement(*intent,&oldDocument,true,false);
            result[1]=replacement
                && a.EndReplacement(*replacement,&candidate,true,true,false)==ReplacementEnd::Adopted
                && a.EndManualIntent(*intent,true,false)==ManualIntentEnd::NotCurrent
                && !a.Capture(&oldDocument,true,false) && a.Capture(&candidate,true,false).has_value();
        }
    }
    {
        NativeEditAuthority a(nonce); a.Adopt(&oldDocument);
        const auto intent=a.BeginManualIntent(&oldDocument,true,false);
        result[2]=intent
            && !a.PromoteManualIntentToReplacement(*intent,&oldDocument,false,false)
            && !a.PromoteManualIntentToReplacement(*intent,&oldDocument,true,true)
            && !a.PromoteManualIntentToReplacement(*intent,&candidate,true,false)
            && a.OwnsManualIntent(*intent) && !a.Capture(&oldDocument,true,false)
            && a.PromoteManualIntentToReplacement(*intent,&oldDocument,true,false).has_value();
    }
    {
        NativeEditAuthority a(nonce),b(otherNonce);a.Adopt(&oldDocument);b.Adopt(&oldDocument);
        const auto current=a.BeginManualIntent(&oldDocument,true,false),foreign=b.BeginManualIntent(&oldDocument,true,false);
        result[3]=current && foreign
            && !a.PromoteManualIntentToReplacement(*foreign,&oldDocument,true,false)
            && a.OwnsManualIntent(*current) && a.Valid();
    }
    {
        NativeEditAuthority a(nonce);a.Adopt(&oldDocument);
        const auto stale=a.BeginManualIntent(&oldDocument,true,false);
        if (stale && a.EndManualIntent(*stale,true,false)==ManualIntentEnd::Settled) {
            const auto current=a.BeginManualIntent(&oldDocument,true,false);
            const bool rejectsStale=current
                && !a.PromoteManualIntentToReplacement(*stale,&oldDocument,true,false)
                && a.OwnsManualIntent(*current);
            const auto replacement=current?a.PromoteManualIntentToReplacement(*current,&oldDocument,true,false):std::nullopt;
            result[4]=rejectsStale && replacement
                && !a.PromoteManualIntentToReplacement(*current,&oldDocument,true,false)
                && a.OwnsReplacement(*replacement)
                && a.EndReplacement(*replacement,&oldDocument,false,false,false)==ReplacementEnd::Pending
                && !a.Capture(&oldDocument,true,false);
        }
    }
    {
        NativeEditAuthority a(nonce);a.Adopt(&oldDocument);
        const auto intent=a.BeginManualIntent(&oldDocument,true,false);
        result[5]=intent && a.DebugExhaustCounter(4)
            && !a.PromoteManualIntentToReplacement(*intent,&oldDocument,true,false)
            && !a.Valid() && !a.Capture(&oldDocument,true,false);
    }
    {
        NativeEditAuthority a(nonce);a.Adopt(&oldDocument);
        const auto intent=a.BeginManualIntent(&oldDocument,true,false);
        if (intent) {
            bool issued=true;
            std::thread thread([&]{issued=a.PromoteManualIntentToReplacement(*intent,&oldDocument,true,false).has_value();});thread.join();
            result[6]=!issued && !a.Valid() && !a.Capture(&oldDocument,true,false);
        }
    }
    return result;
}
}
#endif
