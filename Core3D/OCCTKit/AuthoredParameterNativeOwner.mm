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
    bool prepared = false;
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

authored_loft::Status Owner::prepareLoft(
    std::uint32_t stationIdentifier, authored_loft::Field field,
    const rectangular_loft::Definition& requested,
    retained_edge_treatment::r2::Edit& output) noexcept {
    if (![NSThread isMainThread] || !state_ || state_->prepared)
        return authored_loft::Status::Malformed;
    if (currentness() != Currentness::Current || !state_->capture->snapshot_)
        return authored_loft::Status::Malformed;
    const auto status = authored_loft::Prepare(*state_->capture->snapshot_,
        stationIdentifier, field, requested, output);
    if (status == authored_loft::Status::Prepared
        || status == authored_loft::Status::Unchanged) state_->prepared = true;
    return status;
}

std::shared_ptr<const retained_edge_treatment::r2::Snapshot>
Owner::retainedSnapshot() const noexcept {
    return state_ && state_->capture ? state_->capture->snapshot_ : nullptr;
}

bool Owner::describeBoolean(part_boolean::AnalyticDefinition& output) const noexcept {
    output = {};
    return state_ && state_->capture && state_->capture->snapshot_
        && authored_boolean::Describe(*state_->capture->snapshot_, output)
            == authored_boolean::Status::Prepared;
}

ApplyResult Owner::applyBoolean(Capability capability, std::size_t inputIndex,
                               const part_boolean::AnalyticDefinition& requested) noexcept {
    ApplyResult result;
    result.measuredUndoDelta = 0;
    const auto refuse = [&](const char *code) {
        result.outcome = ApplyOutcome::Refused;
        result.code = code;
        return result;
    };
    try {
        if (![NSThread isMainThread] || !state_ || state_->prepared)
            return refuse("authored.boolean.malformed");
        switch (currentness()) {
            case Currentness::Cancelled:
                result.outcome = ApplyOutcome::Cancelled;
                result.code = "authored.cancelled";
                return result;
            case Currentness::Recovery:
                return refuse("authored.recovery-required");
            case Currentness::Stale:
                return refuse("authored.stale-opening");
            case Currentness::Current: break;
        }
        authored_boolean::Kind kind;
        if (capability == Capability::BooleanAnalyticInput)
            kind = authored_boolean::Kind::AnalyticInput;
        else if (capability == Capability::BooleanOperation)
            kind = authored_boolean::Kind::Operation;
        else if (capability == Capability::BooleanPlacement)
            kind = authored_boolean::Kind::Placement;
        else return refuse("authored.boolean.malformed");

        const std::atomic_bool neverCancelled(false);
        authored_boolean::Candidate candidate;
        const auto status = authored_boolean::Prepare(*state_->capture->snapshot_,
            kind, inputIndex, requested, state_->capture->untreatedBase_,
            neverCancelled, candidate);
        state_->prepared = true;
        using Status = authored_boolean::Status;
        if (status == Status::Unchanged) {
            result.outcome = ApplyOutcome::Unchanged;
            result.code = "authored.unchanged";
            return result;
        }
        if (status != Status::Prepared) {
            switch (status) {
                case Status::Malformed: result.code = "authored.boolean.malformed"; break;
                case Status::UnsupportedSource: result.code = "authored.boolean.unsupported-source"; break;
                case Status::IdentityChanged: result.code = "authored.boolean.identity-changed"; break;
                case Status::FieldChanged: result.code = "authored.boolean.field-changed"; break;
                case Status::InvalidDefinition: result.code = "authored.boolean.invalid-definition"; break;
                case Status::Budget: result.code = "b1.Budget"; break;
                case Status::Cancelled: result.code = "authored.cancelled"; break;
                case Status::BuildFailed: result.code = "authored.boolean.build-failed"; break;
                case Status::Prepared:
                case Status::Unchanged: break;
            }
            result.outcome = status == Status::Cancelled
                ? ApplyOutcome::Cancelled : ApplyOutcome::Refused;
            return result;
        }

        std::shared_ptr<native_opening::CommandLease> lease;
        if (!state_->document->BeginAuthoredParameterTransaction(*state_->capture,
                *state_->context, state_->width, state_->height, lease)
            || !lease || !lease->ownsOpenCommand())
            return refuse("authored.stale-opening");
        native_opening::CommittedEditPublication publication;
        const std::string entity = state_->document->EntityIdentifierForLabel(
            state_->capture->ownerLabel_);
        if (entity.empty()) {
            lease->abort();
            return refuse("authored.boolean.identity-changed");
        }
        publication.replaced.push_back({entity, TopoDS_Shape()});
        if (!state_->document->StageAuthoredBooleanPair(*state_->capture, candidate)) {
            const bool aborted = lease->abort();
            const bool settled = aborted
                && state_->document->SettleTreatmentHistoryCompanionOnPrior();
            if (!settled) {
                state_->context->retainUnprovenEdit(publication);
                result.outcome = ApplyOutcome::OutcomeUnknown;
                result.code = "authored.outcome-unknown";
                result.measuredUndoDelta.reset();
                return result;
            }
            return refuse("authored.boolean.stage-failed");
        }
        if (!lease->commit()) {
            state_->context->retainUnprovenEdit(publication);
            result.outcome = ApplyOutcome::OutcomeUnknown;
            result.code = "authored.outcome-unknown";
            result.measuredUndoDelta.reset();
            return result;
        }
        if (!state_->document->FinalizeTreatmentHistoryCompanion()
            || state_->document->Document()->GetAvailableUndos()
                != state_->capture->undoDepth_ + 1
            || !state_->context->publishCommittedEdit(publication)) {
            state_->context->retainUnprovenEdit(publication);
            result.outcome = ApplyOutcome::OutcomeUnknown;
            result.code = "authored.outcome-unknown";
            result.measuredUndoDelta.reset();
            return result;
        }
        result.outcome = ApplyOutcome::Committed;
        result.code = "authored.none";
        result.measuredUndoDelta = 1;
        return result;
    } catch (...) { return refuse("authored.boolean.exception"); }
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
