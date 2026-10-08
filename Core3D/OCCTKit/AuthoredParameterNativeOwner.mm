#include "AuthoredParameterNativeOwner.hxx"
#include "OcctDocument.h"

#import <Foundation/Foundation.h>

#include <utility>

namespace core3d::authored_parameter {

struct Owner::State final {
    Handle(OcctDocument) document;
    std::shared_ptr<native_opening::Context> context;
    std::shared_ptr<const Capture> capture;
    std::uint32_t width = 0, height = 0;
    bool cancelled = false;
};

Owner::Owner(std::unique_ptr<State> state) noexcept : state_(std::move(state)) {}
Owner::~Owner() = default;

std::shared_ptr<Owner> Owner::Open(
    const Handle(OcctDocument)& document,
    std::shared_ptr<native_opening::Context> context,
    std::shared_ptr<const Capture> capture,
    std::uint32_t width, std::uint32_t height) noexcept {
    try {
        if (![NSThread isMainThread] || document.IsNull() || !context || !capture
            || !width || !height || context->openingFence().document() != capture->document_
            || context->openingFence().data() != capture->data_) return {};
        auto state = std::make_unique<State>();
        state->document = document;
        state->context = std::move(context);
        state->capture = std::move(capture);
        state->width = width;
        state->height = height;
        auto owner = std::shared_ptr<Owner>(new Owner(std::move(state)));
        return owner->currentness() == Currentness::Current ? owner : std::shared_ptr<Owner>();
    } catch (...) { return {}; }
}

const Capture& Owner::capture() const noexcept { return *state_->capture; }

bool Owner::belongsTo(const Handle(OcctDocument)& document) const noexcept {
    return state_ && !document.IsNull() && !state_->document.IsNull()
        && document.get() == state_->document.get();
}

Currentness Owner::currentness() const noexcept {
    if (!state_ || state_->cancelled || !state_->context) return Currentness::Cancelled;
    if (![NSThread isMainThread] || state_->document.IsNull()) return Currentness::Stale;
    if (state_->document->HasUnresolvedTreatmentHistoryCompanion())
        return Currentness::Recovery;
    if (!state_->context->isCurrent(state_->width, state_->height)
        || !state_->document->ReadAuthoredParameterAuthority(*state_->capture))
        return Currentness::Stale;
    return Currentness::Current;
}

Refusal Owner::prepare(const Mutation& mutation) noexcept {
    if (![NSThread isMainThread]) return Refusal::WrongThread;
    switch (currentness()) {
        case Currentness::Cancelled: return Refusal::Cancelled;
        case Currentness::Recovery: return Refusal::RecoveryRequired;
        case Currentness::Stale: return Refusal::StaleOpening;
        case Currentness::Current: break;
    }
    // No adapter is registered in N0. A malformed raw capability is distinct
    // from a well-formed but not-yet-installed closed request arm.
    const auto raw = static_cast<std::uint8_t>(mutation.capability);
    if (raw > static_cast<std::uint8_t>(Capability::ProfileShellValues)
        || mutation.requestedFieldMask != 0 || !mutation.sealedValues.empty())
        return Refusal::MalformedMutation;
    return Refusal::UnsupportedCapability;
}

bool Owner::cancel() noexcept {
    if (![NSThread isMainThread] || !state_) return false;
    if (state_->cancelled) return true;
    state_->cancelled = true;
    state_->context.reset();
    return true;
}

const char* RefusalCode(Refusal refusal) noexcept {
    switch (refusal) {
        case Refusal::None: return "authored.none";
        case Refusal::WrongThread: return "authored.wrong-thread";
        case Refusal::MalformedMutation: return "authored.malformed-mutation";
        case Refusal::UnsupportedCapability: return "authored.unsupported-capability";
        case Refusal::ForeignOpening: return "authored.foreign-opening";
        case Refusal::StaleOpening: return "authored.stale-opening";
        case Refusal::Cancelled: return "authored.cancelled";
        case Refusal::RecoveryRequired: return "authored.recovery-required";
    }
}

} // namespace core3d::authored_parameter
