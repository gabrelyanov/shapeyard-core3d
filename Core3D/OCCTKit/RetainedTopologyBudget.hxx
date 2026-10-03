#pragma once
// R4 / T-B-B2 topology budget — P1 native accounting core.
//
// Private shared accounting for the retained B1/R2 editing lanes. This header
// is deliberately NOT a framework-exported header and has no PBX entry: it is
// included explicitly (same-directory quoted include, matching every
// neighbouring OCCTKit header) by the retained-editing headers only.
//
// Contents:
//   * the frozen limits — 4,096 faces plus edges combined per stage census,
//     65,536 aggregate topology visits per operation, 64 build/work stages
//     per operation, 64 discovery candidates per use;
//   * Counter — the common per-operation topology counter that the B1 and R2
//     ReplayBudget types publicly derive from, keeping their existing
//     buildStages/topologyVisits members and chargeStage(visits) semantics;
//   * bounded census/traversal walkers that charge every root/child
//     occurrence while walking with an explicit iterator-frame stack, enforce
//     the combined face+edge stage census before map growth, and observe
//     cancellation.
//
// Accounting semantics (design review
// r4-tb-b2-topology-budget-design-review-astra, CONTRACT.md "Frozen limits
// and accounting semantics"):
//   * arithmetic validates the current counters first, then compares
//     delta <= limit - used; a corrupt continuation is rejected before any
//     subtraction or wraparound can occur;
//   * any denied admission is sticky on the operation: the counter is marked
//     exhausted, admitted counters stay within limits, and no later charge on
//     the same operation succeeds;
//   * reservations debit their measured pass up front; consuming a
//     reservation later is not a new allowance and never a refund;
//   * DEBUG builds record the first denial (site, dimension, requested delta,
//     admitted counters) and a fixed per-site visit/stage table for the P2
//     owner probe; no allocation happens per visit.

#include <TopAbs_ShapeEnum.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shape.hxx>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#if DEBUG
#include <string>
#include <vector>
#endif

namespace core3d::retained_topology_budget {

inline constexpr std::size_t MaximumStageFaceEdgeCensus = 4096;
inline constexpr std::size_t MaximumTopologyVisits = 65536;
inline constexpr std::size_t MaximumBuildStages = 64;
inline constexpr std::size_t MaximumDiscoveryUses = 64;

// Charge-site identifiers matching the CONTRACT.md charge-site inventory.
// C09/C10/C27/C28 live in the P2 operation-lifetime files; they are named
// here so the whole operation shares one trace vocabulary.
enum class Site : std::uint8_t {
    None = 0,
    C01StageCensus, C02FacePasses, C03AncestorMap, C04WireExplorers,
    C05OwnerScan, C06DirectCensus, C07CurveClassification, C08ReceiptVerify,
    C09ViewerIssuance, C10ReceiptProjection,
    C11GeometryCensus, C12Detachment, C13CommitVerify, C14AnchorResolve,
    C15ReplaySteps, C16KernelBuild, C17SourceRebindOld, C18SourceRebindNew,
    C19R2Prefix, C20R2EditedPrefix, C21LegacyPrefix, C22FilletAdmission,
    C23FilletRound, C24FilletBuild, C25AnalyticInput, C26AnalyticBoolean,
    C27CapturedAnchor, C28CompanionCapture,
    Count
};

enum class Dimension : std::uint8_t {
    Visit = 0, BuildStage = 1, StageCensus = 2, DiscoveryUse = 3
};

struct Counter;

#if DEBUG
namespace debug {
// F10 debt hook (DEBUG only; armed exclusively by the P2 observation bridge).
// The hook never writes counters directly: at the requested deterministic
// site occurrence it adds prior debt through the real checked Counter
// visit/beginStage admissions so that exactly the requested allowance remains
// when the tagged work charge runs. A debt that cannot fit denies through the
// same checked API. Bounds are never lowered; the hook fires at most once and
// is cleared on observation settlement/teardown.
inline void DebtHookBefore(Counter& counter, Site site) noexcept;
}
#endif

struct Counter {
    std::size_t buildStages = 0;
    std::size_t topologyVisits = 0;
    // Sticky exhaustion: once any admission is denied, the operation may not
    // spend further credit. Admitted counters always stay within limits.
    bool exhausted = false;
#if DEBUG
    std::uint64_t chargeEvents = 0;
    std::size_t visitsBySite[static_cast<std::size_t>(Site::Count)] = {};
    std::size_t stagesBySite[static_cast<std::size_t>(Site::Count)] = {};
    Site firstDeniedSite = Site::None;
    Dimension firstDeniedDimension = Dimension::Visit;
    std::size_t firstDeniedRequested = 0;
    std::size_t firstDeniedStagesAdmitted = 0;
    std::size_t firstDeniedVisitsAdmitted = 0;
#endif
    // The current counters must be inside their limits before any delta is
    // compared; a corrupt continuation is never converted into fresh
    // allowance and no subtraction happens on an invalid value (contract §7).
    bool valid() const noexcept {
        return !exhausted && buildStages <= MaximumBuildStages
            && topologyVisits <= MaximumTopologyVisits;
    }
    // Record a denied admission; sticky. Returns false for charge-expression
    // use. Never consumes the denied delta.
    bool deny(Site site, Dimension dimension, std::size_t requested) noexcept {
        exhausted = true;
#if DEBUG
        if (firstDeniedSite == Site::None) {
            firstDeniedSite = site;
            firstDeniedDimension = dimension;
            firstDeniedRequested = requested;
            firstDeniedStagesAdmitted = buildStages;
            firstDeniedVisitsAdmitted = topologyVisits;
        }
#endif
        return false;
    }
    // Charge aggregate topology occurrences (no stage debit). Zero deltas are
    // legal; an exact landing on the limit succeeds and the next visit fails.
    bool visit(std::size_t delta, Site site = Site::None) noexcept {
#if DEBUG
        debug::DebtHookBefore(*this, site);
#endif
        if (!valid() || delta > MaximumTopologyVisits - topologyVisits)
            return deny(site, Dimension::Visit, delta);
        topologyVisits += delta;
#if DEBUG
        ++chargeEvents;
        visitsBySite[static_cast<std::size_t>(site)] += delta;
#endif
        return true;
    }
    // One build/work stage debit without visits.
    bool beginStage(Site site = Site::None) noexcept {
#if DEBUG
        debug::DebtHookBefore(*this, site);
#endif
        if (!valid() || buildStages >= MaximumBuildStages)
            return deny(site, Dimension::BuildStage, 1);
        ++buildStages;
#if DEBUG
        ++chargeEvents;
        ++stagesBySite[static_cast<std::size_t>(site)];
#endif
        return true;
    }
    // Legacy combined debit: one stage plus its visits. Semantics identical to
    // the previous inline ReplayBudget::chargeStage, plus validation-first
    // arithmetic and sticky refusal.
    bool chargeStage(std::size_t visits, Site site = Site::None) noexcept {
#if DEBUG
        debug::DebtHookBefore(*this, site);
#endif
        if (!valid() || buildStages >= MaximumBuildStages)
            return deny(site, Dimension::BuildStage, 1);
        if (visits > MaximumTopologyVisits - topologyVisits)
            return deny(site, Dimension::Visit, visits);
        ++buildStages;
        topologyVisits += visits;
#if DEBUG
        ++chargeEvents;
        ++stagesBySite[static_cast<std::size_t>(site)];
        visitsBySite[static_cast<std::size_t>(site)] += visits;
#endif
        return true;
    }
};

#if DEBUG
namespace debug {
struct DebtHook {
    bool armed = false;
    bool firing = false;
    Site site = Site::None;
    std::size_t occurrence = 0;   // 1-based charge-event ordinal at the site
    std::size_t seen = 0;
    std::size_t remainingVisits = 0;  // allowance to leave when the hook fires
    std::size_t remainingStages = 0;
};
inline DebtHook& DebtHookState() noexcept { static DebtHook state; return state; }
inline void ArmDebtHook(Site site, std::size_t occurrence,
    std::size_t remainingVisits, std::size_t remainingStages) noexcept {
    DebtHook& hook = DebtHookState();
    hook.armed = true; hook.firing = false; hook.site = site;
    hook.occurrence = occurrence ? occurrence : 1; hook.seen = 0;
    hook.remainingVisits = remainingVisits; hook.remainingStages = remainingStages;
}
inline void ClearDebtHook() noexcept { DebtHookState() = DebtHook{}; }
// Runs at the top of every checked charge. At the requested site occurrence it
// tops the operation up to (limit - remaining) through the real checked
// admissions, so the tagged charge itself observes exactly the requested
// allowance and denies BEFORE its work when one unit fewer was requested.
inline void DebtHookBefore(Counter& counter, Site site) noexcept {
    DebtHook& hook = DebtHookState();
    if (!hook.armed || hook.firing || site != hook.site) return;
    ++hook.seen;
    if (hook.seen < hook.occurrence) return;
    hook.armed = false; hook.firing = true;
    const std::size_t visits = counter.topologyVisits;
    const std::size_t stages = counter.buildStages;
    if (hook.remainingVisits > MaximumTopologyVisits - visits) {
        // The operation already spent past the requested allowance: deny via
        // the real checked API rather than manufacturing room.
        counter.visit(MaximumTopologyVisits, Site::None);
    } else if (const std::size_t debt = MaximumTopologyVisits - visits - hook.remainingVisits) {
        counter.visit(debt, Site::None);
    }
    if (hook.remainingStages > MaximumBuildStages - stages) {
        counter.beginStage(Site::None);
    } else {
        for (std::size_t debt = MaximumBuildStages - stages - hook.remainingStages;
            debt; --debt)
            if (!counter.beginStage(Site::None)) break;
    }
    hook.firing = false;
}

// F10 observation session (DEBUG only). One active session at a time, owned
// by the P2 bridge's begin/end pair. Recording is bounded: a fixed-capacity
// phase ring; overflow marks the trace incomplete instead of silently
// truncating into passing evidence. The last observed counter copy carries
// the DEBUG per-site aggregates and the first-denial record.
struct PhaseRecord {
    std::string name;
    std::size_t entryVisits = 0, entryStages = 0, exitVisits = 0, exitStages = 0;
};
struct SessionTrace {
    static constexpr std::size_t PhaseCapacity = 64;
    std::vector<PhaseRecord> phases;
    bool overflow = false;
    bool hasBudget = false;
    Counter budget;
    bool protectedWorkStarted = false;
    bool partialOutputEscaped = false;
    std::size_t completionCount = 0;
    bool commitSeen = false;
    int commitOutcome = -1;
    std::string commitRefusal;
    std::string operationRefusal;
};
inline SessionTrace*& ActiveSession() noexcept {
    static SessionTrace* session = nullptr; return session;
}
inline void RecordPhase(const char* name, const Counter& entry,
    const Counter& exitBudget) noexcept {
    SessionTrace* session = ActiveSession();
    if (!session) return;
    try {
        if (session->phases.size() >= SessionTrace::PhaseCapacity) {
            session->overflow = true; return;
        }
        session->phases.push_back(PhaseRecord{name ? name : "phase",
            entry.topologyVisits, entry.buildStages,
            exitBudget.topologyVisits, exitBudget.buildStages});
        session->budget = exitBudget;
        session->hasBudget = true;
    } catch (...) { session->overflow = true; }
}
// The first protected mutation of the operation (owner shape/carrier write).
inline void RecordMutation() noexcept {
    SessionTrace* session = ActiveSession();
    if (session) session->protectedWorkStarted = true;
}
inline void RecordCommit(int outcome, const char* refusalCode,
    bool partialEscaped) noexcept {
    SessionTrace* session = ActiveSession();
    if (!session) return;
    try {
        session->commitSeen = true;
        session->commitOutcome = outcome;
        session->commitRefusal = refusalCode ? refusalCode : "b1.StageFailed";
        ++session->completionCount;
        if (partialEscaped) session->partialOutputEscaped = true;
    } catch (...) { session->overflow = true; }
}
// Terminal refusal of a query/prepare/detach seam that never reaches commit;
// nil alone is never budget evidence, so the typed cause is recorded.
inline void RecordRefusal(const char* refusalCode) noexcept {
    SessionTrace* session = ActiveSession();
    if (!session || !refusalCode) return;
    try {
        if (!session->commitSeen && session->operationRefusal.empty())
            session->operationRefusal = refusalCode;
    } catch (...) { session->overflow = true; }
}
} // namespace debug
#endif


enum class WalkStatus : std::uint8_t { Completed, Cancelled, BudgetDenied, Failed };

// Result of a bounded stage census (contract §1/§4). faceEdge holds the
// combined distinct face/edge identities using native shape identity including
// location (the same identity TopExp::MapShapes accumulates); faces/edges are
// the same identities split per type for the existing consumer loops.
// occurrences counts every root/child occurrence visited, including repeated,
// shared, vertex, wire and container occurrences. edgeUsesUnderFaces counts
// edge occurrences with a face ancestor — the exact relation-insertion count
// of an edge→face ancestor map over the same shape.
struct Census {
    TopTools_IndexedMapOfShape faceEdge;
    TopTools_IndexedMapOfShape faces;
    TopTools_IndexedMapOfShape edges;
    std::size_t occurrences = 0;
    std::size_t edgeUsesUnderFaces = 0;
};

namespace detail {
// Bounded occurrence walk on an explicit stack of TopoDS_Iterator frames; no
// recursion. Every root/child occurrence is charged BEFORE it is processed or
// its subtree is pushed, so a huge fan-out, deep compound, repeated child,
// empty wire or vertex-only subtree cannot hide behind a small final map.
// Cancellation is observed while walking. admit(shape, faceAncestor) runs per
// charged occurrence; returning false aborts the walk as BudgetDenied (the
// callback must deny through the budget itself so the denial stays sticky).
template <typename Admit>
inline WalkStatus WalkOccurrences(const TopoDS_Shape& root, Counter& budget,
    const std::atomic_bool& cancelled, Site site, Admit& admit) noexcept {
    try {
        if (root.IsNull()) return WalkStatus::Completed;
        struct Frame { TopoDS_Iterator iterator; bool faceAncestor; };
        if (!budget.visit(1, site)) return WalkStatus::BudgetDenied;
        if (!admit(root, false)) return WalkStatus::BudgetDenied;
        std::deque<Frame> stack;
        stack.push_back(Frame{TopoDS_Iterator(root), root.ShapeType() == TopAbs_FACE});
        while (!stack.empty()) {
            if (cancelled.load()) return WalkStatus::Cancelled;
            Frame& frame = stack.back();
            if (!frame.iterator.More()) { stack.pop_back(); continue; }
            const TopoDS_Shape child = frame.iterator.Value();
            frame.iterator.Next();
            if (!budget.visit(1, site)) return WalkStatus::BudgetDenied;
            if (!admit(child, frame.faceAncestor)) return WalkStatus::BudgetDenied;
            stack.push_back(Frame{TopoDS_Iterator(child),
                frame.faceAncestor || child.ShapeType() == TopAbs_FACE});
        }
        return WalkStatus::Completed;
    } catch (...) { return WalkStatus::Failed; }
}
} // namespace detail

// Bounded raw stage census. With debitStage the stage's work debit happens
// before the walk (contract §3: begin a stage before its work). The combined
// face+edge census rejects the distinct admission that would exceed the limit
// before the map grows past it (contract §4).
inline WalkStatus CensusTopology(const TopoDS_Shape& root, Counter& budget,
    const std::atomic_bool& cancelled, Census& census, Site site,
    bool debitStage) noexcept {
    try {
        census = Census{};
        if (debitStage && !budget.beginStage(site)) return WalkStatus::BudgetDenied;
        auto admit = [&](const TopoDS_Shape& shape, bool faceAncestor) -> bool {
            ++census.occurrences;
            const TopAbs_ShapeEnum type = shape.ShapeType();
            if (type == TopAbs_FACE || type == TopAbs_EDGE) {
                if (type == TopAbs_EDGE && faceAncestor) ++census.edgeUsesUnderFaces;
                if (!census.faceEdge.Contains(shape)
                    && census.faceEdge.Extent() >= static_cast<int>(MaximumStageFaceEdgeCensus))
                    return budget.deny(site, Dimension::StageCensus, 1);
                census.faceEdge.Add(shape);
                if (type == TopAbs_FACE) census.faces.Add(shape);
                else census.edges.Add(shape);
            }
            return true;
        };
        return detail::WalkOccurrences(root, budget, cancelled, site, admit);
    } catch (...) { return WalkStatus::Failed; }
}

// Visits-only bounded traversal: one full occurrence pass over the shape with
// no census map. Used to debit a measured serialization/copy/validity/volume/
// kernel input pass (contract §6), optionally reporting the measured
// occurrence count so the caller can reserve a further pass of the same shape.
inline WalkStatus ChargeTraversal(const TopoDS_Shape& shape, Counter& budget,
    const std::atomic_bool& cancelled, Site site,
    std::size_t* occurrences = nullptr) noexcept {
    std::size_t count = 0;
    auto admit = [&](const TopoDS_Shape&, bool) -> bool { ++count; return true; };
    const WalkStatus status = detail::WalkOccurrences(shape, budget, cancelled, site, admit);
    if (occurrences && status == WalkStatus::Completed) *occurrences = count;
    return status;
}

// Reserve one additional full pass over an already censused shape (contract
// §5/§6 reservation formula: one admitted occurrence census per bulk API
// pass). Consuming the reservation later is not a new allowance or a refund;
// the reserved pass is not charged again at its work site.
inline bool ReserveTraversal(const Census& census, Counter& budget, Site site) noexcept {
    return budget.visit(census.occurrences, site);
}

#if DEBUG
inline const char* SiteName(Site site) noexcept {
    static const char* values[] = {
        "none",
        "C01", "C02", "C03", "C04", "C05", "C06", "C07", "C08", "C09", "C10",
        "C11", "C12", "C13", "C14", "C15", "C16", "C17", "C18", "C19", "C20",
        "C21", "C22", "C23", "C24", "C25", "C26", "C27", "C28"
    };
    const auto index = static_cast<std::size_t>(site);
    return index < static_cast<std::size_t>(Site::Count) ? values[index] : "unknown";
}
inline const char* DimensionName(Dimension dimension) noexcept {
    static const char* values[] = {"visit", "build-stage", "stage-census", "discovery-use"};
    const auto index = static_cast<std::size_t>(dimension);
    return index <= static_cast<std::size_t>(Dimension::DiscoveryUse) ? values[index] : "unknown";
}
#endif

} // namespace core3d::retained_topology_budget
