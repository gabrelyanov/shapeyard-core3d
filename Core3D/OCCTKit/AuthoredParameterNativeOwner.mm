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
    // A2-N2 terminal recovery receipt: once the document-owned record is
    // settled and the issuing context's publication has cleared the fence,
    // repeated recovery calls return this cached receipt without publishing,
    // building, finalizing or consuming a later operation again.
    bool recoverySettled = false;
    BooleanRecoveryOutcome recoveryReceiptOutcome =
        BooleanRecoveryOutcome::RecoveryRequired;
    int recoveryReceiptDelta = 0;
#if DEBUG
    bool debugCloseUnprovenArmed = false;
    std::shared_ptr<debug::AuthoredBooleanStageFault> debugStageFault;
    bool debugAbortSettlementUnprovenArmed = false;
    bool debugRecoveryObservationFailureArmed = false;
    std::uint32_t debugRecoveryObservationFaultHits = 0;
    std::uint32_t debugRecoveryCalls = 0;
    std::uint32_t debugRecoveryObservationCalls = 0;
    std::uint32_t debugRecoveryPublicationCalls = 0;
#endif
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

        // A2-N2: resolve the publication identity and retain the
        // document-owned recovery record (immutable prior capture, issuing
        // context and sealed candidate byte witnesses) before any mutation;
        // a retention failure refuses with the baseline unchanged.
        native_opening::CommittedEditPublication publication;
        const std::string entity = state_->document->EntityIdentifierForLabel(
            state_->capture->ownerLabel_);
        if (entity.empty())
            return refuse("authored.boolean.identity-changed");
        publication.replaced.push_back({entity, TopoDS_Shape()});
        std::vector<std::uint8_t> candidateCompositeBytes;
        if (candidate.composite)
            candidateCompositeBytes = candidate.composite->bytes;
        if (!state_->document->RetainAuthoredBooleanRecovery(
                state_->capture, state_->context,
                std::move(candidateCompositeBytes), candidate.treatmentBytes))
            return refuse("authored.boolean.recovery-record");
        std::shared_ptr<native_opening::CommandLease> lease;
        if (!state_->document->BeginAuthoredParameterTransaction(*state_->capture,
                *state_->context, state_->width, state_->height, lease)
            || !lease || !lease->ownsOpenCommand()) {
            state_->document->RetireAuthoredBooleanRecovery();
            return refuse("authored.stale-opening");
        }
#if DEBUG
        if (!state_->document->StageAuthoredBooleanPair(*state_->capture, candidate,
                state_->debugStageFault.get())) {
#else
        if (!state_->document->StageAuthoredBooleanPair(*state_->capture, candidate)) {
#endif
            const bool aborted = lease->abort();
            bool deferSettlement = false;
#if DEBUG
            if (aborted && state_->debugAbortSettlementUnprovenArmed) {
                state_->debugAbortSettlementUnprovenArmed = false;
                deferSettlement = true;
            }
#endif
            const bool settled = aborted && !deferSettlement
                && state_->document->SettleTreatmentHistoryCompanionOnPrior();
            if (!settled) {
                // A genuinely closed real abort without proven prior
                // settlement keeps an owned fence: the same issuing context
                // stays the sole settlement authority until the native
                // recovery record decides.
                if (aborted) state_->context->armRecoveryAfterAbortedClose();
                state_->context->retainUnprovenEdit(publication);
                result.outcome = ApplyOutcome::OutcomeUnknown;
                result.code = "authored.outcome-unknown";
                result.measuredUndoDelta.reset();
                return result;
            }
            state_->document->RetireAuthoredBooleanRecovery();
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
        state_->document->RetireAuthoredBooleanRecovery();
        result.outcome = ApplyOutcome::Committed;
        result.code = "authored.none";
        result.measuredUndoDelta = 1;
        return result;
    } catch (...) {
        // A2-N2: an exception after the native transaction may have run is
        // genuine uncertainty with retained evidence, never a clean
        // pre-command rejection.
        if (state_ && !state_->document.IsNull()
            && state_->document->HasUnresolvedTreatmentHistoryCompanion()) {
            result.outcome = ApplyOutcome::OutcomeUnknown;
            result.code = "authored.outcome-unknown";
            result.measuredUndoDelta.reset();
            return result;
        }
        return refuse("authored.boolean.exception");
    }
}

BooleanRecoveryResult Owner::reconcileBoolean() noexcept {
    BooleanRecoveryResult result;
    try {
        if (![NSThread isMainThread] || !state_ || state_->document.IsNull()
            || !state_->context) {
            result.outcome = BooleanRecoveryOutcome::NotOwner;
            result.code = "authored.recovery-not-owner";
            return result;
        }
#if DEBUG
        ++state_->debugRecoveryCalls;
        result.reconcileCalls = state_->debugRecoveryCalls;
        result.observationFaultHits = state_->debugRecoveryObservationFaultHits;
#endif
        if (!state_->recoverySettled && !state_->cancelled) {
            // Issuer binding: only the context that issued the outstanding
            // transaction observes its document-owned record; a foreign or
            // reused authority keeps the record and the lock untouched.
            const auto phase =
                state_->document->AuthoredBooleanRecoveryPhase(state_->context);
            if (phase == core3d::authored_recovery::Phase::None)
                return result;
            core3d::authored_recovery::Class classified =
                core3d::authored_recovery::Class::RecoveryRequired;
            Standard_Integer measuredDelta = 0;
            bool reconciled = false;
            if (phase == core3d::authored_recovery::Phase::Pending) {
#if DEBUG
                ++state_->debugRecoveryObservationCalls;
                result.observationCalls = state_->debugRecoveryObservationCalls;
                const bool observationFaultArmed =
                    state_->debugRecoveryObservationFailureArmed;
#endif
                reconciled = state_->document->ReconcileAuthoredBooleanRecovery(
                    classified, measuredDelta
#if DEBUG
                    , &state_->debugRecoveryObservationFailureArmed
#endif
                    );
#if DEBUG
                if (observationFaultArmed
                    && !state_->debugRecoveryObservationFailureArmed) {
                    ++state_->debugRecoveryObservationFaultHits;
                    result.observationFaultHits =
                        state_->debugRecoveryObservationFaultHits;
                }
#endif
            } else {
                // The native classification/companion decision is already
                // proven; this returns the stored receipt without observing,
                // finalizing or settling the companion a second time.
                reconciled = state_->document->ReconcileAuthoredBooleanRecovery(
                    classified, measuredDelta
#if DEBUG
                    , nullptr
#endif
                    );
            }
            // Failed readback, ambiguous prior/candidate, wrong issuer,
            // missing history or an unreadable state keeps the record and
            // the lock and exposes no measured delta.
            if (!reconciled) return result;
            // Terminal ordering: only after native classification/companion
            // settlement may the issuing context reconcile the retained
            // publication and clear both fences. A failed publication keeps
            // the record at CompanionSettled for an exact later retry.
            if (!state_->context->reconcileRecovery(true)) return result;
            state_->document->RetireAuthoredBooleanRecovery();
            state_->recoverySettled = true;
            state_->recoveryReceiptOutcome =
                classified == core3d::authored_recovery::Class::Committed
                    ? BooleanRecoveryOutcome::Committed
                    : BooleanRecoveryOutcome::PriorRestored;
            state_->recoveryReceiptDelta = static_cast<int>(measuredDelta);
#if DEBUG
            ++state_->debugRecoveryPublicationCalls;
#endif
        }
        if (state_->recoverySettled) {
            result.outcome = state_->recoveryReceiptOutcome;
            result.code = "authored.none";
            result.measuredUndoDelta = state_->recoveryReceiptDelta;
        }
#if DEBUG
        result.observationCalls = state_->debugRecoveryObservationCalls;
        result.observationFaultHits = state_->debugRecoveryObservationFaultHits;
        result.publicationCalls = state_->debugRecoveryPublicationCalls;
        // Recovery never opens a command; the counter exists only so the
        // transport can prove it.
        result.recoveryCommandCalls = 0;
#endif
        return result;
    } catch (...) { return result; }
}

bool Owner::cancel() noexcept {
    if (![NSThread isMainThread] || !state_) return false;
    if (state_->cancelled) return true;
    state_->cancelled = true;
#if DEBUG
    state_->debugStageFault.reset();
#endif
    state_->context.reset();
    return true;
}

#if DEBUG
bool Owner::debugReportNextCloseUnproven() noexcept {
    if (![NSThread isMainThread] || !state_ || state_->prepared
        || state_->debugCloseUnprovenArmed
        || currentness() != Currentness::Current || !state_->context) return false;
    state_->context->debugReportNextCloseUnproven();
    state_->debugCloseUnprovenArmed = true;
    return true;
}

bool Owner::debugArmAuthoredBooleanStageFault(
    debug::AuthoredBooleanFaultSite site,
    debug::AuthoredBooleanFaultDelivery delivery) noexcept {
    if (![NSThread isMainThread] || !state_ || state_->prepared
        || state_->debugStageFault
        || currentness() != Currentness::Current || !state_->context) return false;
    if (static_cast<std::uint8_t>(site) > static_cast<std::uint8_t>(
            debug::AuthoredBooleanFaultSite::AfterCaptureBeforePublish)
        || static_cast<std::uint8_t>(delivery) > static_cast<std::uint8_t>(
            debug::AuthoredBooleanFaultDelivery::ThrowStandardFailure)) return false;
    state_->debugStageFault =
        std::make_shared<debug::AuthoredBooleanStageFault>(site, delivery);
    return state_->debugStageFault != nullptr;
}

bool Owner::debugAuthoredBooleanStageFaultReceipt(
    debug::AuthoredBooleanFaultReceipt& receipt) const noexcept {
    if (![NSThread isMainThread] || !state_ || !state_->debugStageFault) return false;
    receipt = state_->debugStageFault->receipt();
    return true;
}

bool Owner::debugArmNextAbortSettlementUnproven() noexcept {
    if (![NSThread isMainThread] || !state_ || state_->prepared
        || state_->debugAbortSettlementUnprovenArmed
        || currentness() != Currentness::Current || !state_->context) return false;
    state_->debugAbortSettlementUnprovenArmed = true;
    return true;
}

bool Owner::debugArmNextRecoveryObservationFailure() noexcept {
    if (![NSThread isMainThread] || !state_
        || state_->debugRecoveryObservationFailureArmed
        || currentness() != Currentness::Recovery || !state_->context) return false;
    state_->debugRecoveryObservationFailureArmed = true;
    return true;
}
#endif

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
