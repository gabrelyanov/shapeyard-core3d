#pragma once

// N1 native-only retained Boolean transaction owner. The document constructs
// this service at adoption boundaries. Callers select a visible carrier and
// supply typed edits; they never supply inspection, build, proof, staging, or
// final-fence functions.
#include "PartBooleanDefinition.hxx"
#include "CompositeRecipeAttribute.hxx"
#include "RetainedRecipeSnapshot.hxx"
#include "NativeEditAuthority.hpp"
#include <TDF_Label.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS_Shape.hxx>
#include <array>
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <stdexcept>
#include <vector>

class OcctDocument;

namespace core3d::part_boolean::owner {

enum class FaultPoint : std::uint8_t {
    None, F0Capture, F1DetachedDependency, F2FinalFence, F3Stop,
    F4AfterOpen, F5AfterShape, F6AfterInputs, F7BeforeClose,
    F8AfterClose, F9Reconcile, F10Presentation,
};

enum class Outcome : std::uint8_t {
    Rejected, Cancelled, Unchanged, Committed, RecoveryRequired,
};

struct HistoryWitness final {
    Standard_Integer transaction = 0;
    Standard_Integer undoCount = 0;
    Standard_Integer redoCount = 0;
    Standard_Integer undoLimit = 0;
    Standard_Integer dataTime = 0;
    std::uint64_t observationSerial = 0;
    std::vector<std::array<std::uint64_t, 2>> undoTimes;
    std::vector<std::array<std::uint64_t, 2>> redoTimes;
    bool operator==(const HistoryWitness& value) const noexcept {
        return transaction == value.transaction && undoCount == value.undoCount
            && redoCount == value.redoCount && undoLimit == value.undoLimit
            && dataTime == value.dataTime && undoTimes == value.undoTimes
            && redoTimes == value.redoTimes;
    }
};

struct DocumentSnapshot final {
    // Versioned exact scopes: graph, payload, source/result BRep, identities,
    // transforms/units, materials, groups/name/visibility, consumption,
    // issuance, dependency census, and unrelated free-object manifest.
    std::vector<std::vector<std::uint8_t>> scopes;
    HistoryWitness history;
    std::uint64_t graphCensus = 0;
    std::uint64_t modelRevision = 0;
    bool operator==(const DocumentSnapshot& value) const noexcept {
        return scopes == value.scopes && history == value.history
            && graphCensus == value.graphCensus
            && modelRevision == value.modelRevision;
    }
    bool semanticEquals(const DocumentSnapshot& value) const noexcept {
        return scopes == value.scopes && graphCensus == value.graphCensus;
    }
};

struct Receipt final {
    Outcome outcome = Outcome::Rejected;
    std::string reason;
    Standard_Integer measuredHistoryDelta = 0;
    std::uint64_t session = 0;
    bool commandClosed = true;
    bool blocksOtherWork = false;
};

class Capture final {
    friend class PartBooleanOwner;
    friend class InternalBooleanOperationSession;
#if DEBUG
public:
    const AnalyticDefinition& debugAnalytic() const noexcept { return analytic; }
    const retained_recipe::OwnerSnapshot& debugSnapshot() const noexcept { return snapshot; }
    const DocumentSnapshot& debugBaseline() const noexcept { return baseline; }
private:
#endif
public:
    //! Descriptive copy source for the production editor. The capability,
    //! labels and currentness stamp remain private and cannot be reconstructed.
    const AnalyticDefinition& editorAnalytic() const noexcept { return analytic; }
    std::uint64_t editorSession() const noexcept { return session; }
private:
    std::uint64_t ownerNonce = 0, session = 0, request = 0;
    const void* documentIdentity = nullptr;
    authority::Stamp opening;
    TDF_Label carrier;
    TDF_Label record;
    retained_recipe::OwnerSnapshot snapshot;
    composite_recipe::Definition graph;
    AnalyticDefinition analytic;
    std::vector<TopoDS_Shape> sourceShapes;
    DocumentSnapshot baseline;
    retained_recipe::NativeCurrentnessFacts currentness;
};

class PreparedBooleanDocumentChange final {
    friend class PartBooleanOwner;
    std::shared_ptr<const Capture> capture;
    std::uint64_t request = 0;
    AnalyticDefinition analytic;
    composite_recipe::Definition graph;
    std::vector<TopoDS_Shape> sources;
    TopoDS_Shape result;
    DocumentSnapshot expected;
    retained_recipe::NativeCurrentnessFacts currentness;
    FaultPoint fault = FaultPoint::None;
};

struct CaptureOutcome final { Receipt receipt; std::shared_ptr<const Capture> handle; };
struct PrepareOutcome final { Receipt receipt; std::shared_ptr<const PreparedBooleanDocumentChange> handle; };

#if DEBUG
struct ColdLifecycleEvidence final {
    bool firstColdOpen = false;
    bool independentReplay = false;
    bool laterEdit = false;
    bool undoRedo = false;
    bool secondColdOpen = false;
    bool thirdColdOpen = false;
    bool completeInputs = false;
    bool discardedOldHandles = false;
};
#endif

class PartBooleanOwner final {
public:
    explicit PartBooleanOwner(OcctDocument&) noexcept;
    ~PartBooleanOwner();
    PartBooleanOwner(const PartBooleanOwner&) = delete;
    PartBooleanOwner& operator=(const PartBooleanOwner&) = delete;

    CaptureOutcome capture(const TDF_Label& visibleCarrier) noexcept;
    bool describe(const TDF_Label& visibleCarrier, AnalyticDefinition& output) const noexcept;
    PrepareOutcome prepare(const std::shared_ptr<const Capture>&,
                           const AnalyticDefinition&, FaultPoint = FaultPoint::None) noexcept;
    Receipt apply(const std::shared_ptr<const PreparedBooleanDocumentChange>&) noexcept;
    Receipt cancel(std::uint64_t session) noexcept;
    Receipt reconcile(std::uint64_t session) noexcept;

    bool blocksOtherWork() const noexcept { return activeSession_ != 0 || recoverySession_ != 0; }
    bool matchingContinuation(std::uint64_t session) const noexcept {
        return session != 0 && (session == activeSession_ || session == recoverySession_);
    }
    bool boundTo(const Handle(TDocStd_Document)&) const noexcept;
    void retireForDocumentReplacement() noexcept;
#if DEBUG
    bool installEvidenceFixture(Operation, double, TDF_Label&) noexcept;
    ColdLifecycleEvidence debugColdLifecycle(const TDF_Label&) noexcept;
#endif

private:
    friend class ::OcctDocument;
    Receipt result(Outcome, const char*, std::uint64_t, Standard_Integer = 0) const noexcept;
    Receipt refuse(const char*) const noexcept;
    Receipt retire(Outcome, const char*, Standard_Integer = 0) noexcept;
    bool captureNativeState(const TDF_Label&, DocumentSnapshot&,
                            composite_recipe::Record* = nullptr) const noexcept;
    bool resolveAnalytic(const TDF_Label&, composite_recipe::Record&,
                         retained_recipe::OwnerSnapshot&, AnalyticDefinition&,
                         retained_recipe::NativeCurrentnessFacts&) const noexcept;
    bool buildPrepared(const Capture&, const AnalyticDefinition&,
                       PreparedBooleanDocumentChange&) const noexcept;
    bool stagePrepared(const PreparedBooleanDocumentChange&) noexcept;
    bool exactlyOneOwnedDelta(const HistoryWitness&, HistoryWitness&) const noexcept;
    bool abortRestored(const PreparedBooleanDocumentChange&) const noexcept;
#if DEBUG
    friend std::map<std::string, bool> RunNativeOwnerEvidence(int scenario);
#endif

    OcctDocument& owner_;
    Handle(TDocStd_Document) document_;
    const void* documentIdentity_ = nullptr;
    std::uint64_t nonce_ = 0, nextSession_ = 0, nextRequest_ = 0;
    std::uint64_t activeSession_ = 0, recoverySession_ = 0;
    std::uint64_t commandObservationSerial_ = 0;
    DocumentSnapshot recoveryBaseline_, recoveryExpected_;
    HistoryWitness recoveryHistoryBefore_;
};

class InternalBooleanOperationSession final {
public:
    explicit InternalBooleanOperationSession(PartBooleanOwner& owner) noexcept : owner_(owner) {}
    CaptureOutcome capture(const TDF_Label& carrier) noexcept;
    PrepareOutcome prepare(const AnalyticDefinition&, FaultPoint = FaultPoint::None) noexcept;
    Receipt apply() noexcept;
    Receipt cancel() noexcept;
    Receipt reconcile() noexcept;
private:
    PartBooleanOwner& owner_;
    std::shared_ptr<const Capture> capture_;
    std::shared_ptr<const PreparedBooleanDocumentChange> prepared_;
    std::uint64_t recoverySession_ = 0;
    Receipt terminal_;
    bool terminalSet_ = false;
};

#if DEBUG
std::map<std::string, bool> RunNativeOwnerEvidence(int scenario);
#endif
} // namespace core3d::part_boolean::owner
