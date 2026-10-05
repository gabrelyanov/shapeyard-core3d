#pragma once

// A1-OWNER-budget (R4 / D390 / D392): counted detached analytic operand replay.
//
// One header-only analytic adapter for the detached replay route. It threads
// the caller's already-charged operation continuation — one shared
// retained_topology_budget::Counter plus its cancellation flag — through both
// analytic producers, the source/result serializations, the persistence
// materializations, the Boolean build, the correspondence proof and the
// independent two-pass fixed-point replay. Every underlying pass is charged
// before its work runs (no wrapper-only charge around an uncounted multi-pass
// helper), and a denied charge is sticky on the shared counter: Budget and
// Cancelled propagate to the caller without reclassification.
//
// This is a detached replay behavior only: it owns no label, document,
// command, OCAF history or AIS object, installs no owner, performs no P/E/L
// admission stage and holds no source-edit authority. Shared low-level
// arithmetic, the proof/admission domains and every frozen limit stay
// unchanged; the uncounted production entry points keep their exact behavior.
//
// This header is deliberately not a framework-exported header and has no PBX
// entry; it is included explicitly by the DEBUG qualification probe.
#include "../OCCTKit/PartBooleanCorrespondence.hxx"

#include <atomic>
#include <cstdint>

namespace core3d::a1_analytic_input_adapter {

namespace tb = core3d::retained_topology_budget;
namespace rpb = core3d::retained_part_boolean;
namespace build = core3d::part_boolean::build;
namespace correspondence = core3d::part_boolean::correspondence;
namespace rebuild = core3d::part_boolean::rebuild;

enum class ReplayStatus : std::uint8_t {
    Completed = 0,  // every pass charged and run; inspect refusal/proof/fixed point
    Budget = 1,     // sticky: the shared counter is exhausted
    Cancelled = 2,  // cancellation observed before a pass; outranks budget
    Failed = 3,     // infrastructure failure, never a budget/cancel reclassification
};

struct DetachedReplay final {
    build::AnalyticBuild production;
    correspondence::AnalyticProof proof;
    rebuild::AnalyticFixedPointEvidence fixedPoint;
    ReplayStatus status = ReplayStatus::Completed;
    // DEBUG accounting: Counter::chargeEvents at the moment a Budget/Cancelled
    // stop ended the replay. Equality with the final chargeEvents proves no
    // further charge succeeded after the stop (zero post-denial work).
    std::uint64_t chargeEventsAtStop = 0;
    // True only when the whole replay completed and every stage proved.
    bool proven() const noexcept {
        return status == ReplayStatus::Completed && production.complete
            && proof.proven() && fixedPoint.fixedPoint();
    }
};

inline ReplayStatus StatusFromWalk(tb::WalkStatus walk) noexcept {
    switch (walk) {
        case tb::WalkStatus::Completed: return ReplayStatus::Completed;
        case tb::WalkStatus::Cancelled: return ReplayStatus::Cancelled;
        case tb::WalkStatus::BudgetDenied: return ReplayStatus::Budget;
        case tb::WalkStatus::Failed: return ReplayStatus::Failed;
    }
    return ReplayStatus::Failed;
}

// One counted detached replay of the analytic operand pair: production
// (both producers, both source serializations, the Boolean build and the
// envelope extraction), persistence materialization of both sources and the
// result, the correspondence proof, and the independent two-pass fixed point.
// All of it draws on the continuation the caller passes in; nothing here
// creates, resets or refunds a budget.
inline DetachedReplay ReplayCounted(
    const core3d::part_boolean::AnalyticDefinition& definition,
    double carrierMetersPerUnit,
    const rpb::OperandReadSet& captured,
    const rpb::OperandReadSet& current,
    tb::Counter& budget, const std::atomic_bool& cancelled) noexcept {
    DetachedReplay replay;
    const auto recordStop = [&]() {
#if DEBUG
        replay.chargeEventsAtStop = budget.chargeEvents;
#endif
    };
    try {
        // Cancellation is observed before the first charge and outranks any
        // armed budget debt: a cancelled replay never records a denial.
        if (cancelled.load()) {
            replay.status = ReplayStatus::Cancelled;
            replay.production.candidate.refusal = rpb::Refusal::Cancelled;
            recordStop();
            return replay;
        }
        tb::WalkStatus walk = build::BuildAnalyticCounted(definition,
            carrierMetersPerUnit, captured, current, budget, cancelled,
            replay.production);
        if (walk != tb::WalkStatus::Completed) {
            replay.status = StatusFromWalk(walk);
            recordStop();
            return replay;
        }
        // An ordinary admitted/refused production result is fully charged and
        // is not a budget or cancellation outcome.
        if (!replay.production.complete) return replay;
        walk = correspondence::ProveAnalyticCounted(definition,
            replay.production, carrierMetersPerUnit, budget, cancelled,
            replay.proof);
        if (walk != tb::WalkStatus::Completed) {
            replay.status = StatusFromWalk(walk);
            recordStop();
            return replay;
        }
        walk = rebuild::CheckAnalyticCounted(definition, carrierMetersPerUnit,
            captured, budget, cancelled, replay.fixedPoint);
        if (walk != tb::WalkStatus::Completed) {
            replay.status = StatusFromWalk(walk);
            recordStop();
            return replay;
        }
        return replay;
    } catch (...) {
        replay = {};
        replay.status = ReplayStatus::Failed;
        return replay;
    }
}

} // namespace core3d::a1_analytic_input_adapter
