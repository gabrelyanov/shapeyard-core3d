#pragma once

#include "AuthoredLoftParameterAdapter.hxx"
#include "AuthoredBooleanParameterAdapter.hxx"
#include "NativeOpeningContext.hxx"
#include "RetainedEdgeTreatmentR2Snapshot.hxx"

#include <Standard_Handle.hxx>
#include <Standard_Integer.hxx>
#include <TDF_Label.hxx>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

class OcctDocument;
class TDF_Data;
class TDocStd_Document;

namespace core3d::authored_parameter {

//! N0 deliberately installs no positive builder. These are the closed request
//! arms reserved for the separately reviewed L0/A0/C0 adapters.
enum class Capability : std::uint8_t {
    Unregistered = 0,
    BooleanRecipeInput = 1,
    BooleanAnalyticInput = 2,
    BooleanOperation = 3,
    BooleanPlacement = 4,
    LoftStationValues = 5,
    LoftFrame = 6,
    ProfileShellValues = 7,
};

enum class Refusal : std::uint8_t {
    None = 0,
    WrongThread,
    MalformedMutation,
    UnsupportedCapability,
    ForeignOpening,
    StaleOpening,
    Cancelled,
    RecoveryRequired,
};

enum class Currentness : std::uint8_t { Current = 0, Stale, Cancelled, Recovery };

#if DEBUG
namespace debug {

//! SAB-N1 fixed post-write fault sites inside
//! OcctDocument::StageAuthoredBooleanPair. The serialized values are the test
//! contract: never renumber or reuse them.
enum class AuthoredBooleanFaultSite : std::uint8_t {
    AfterOwnerShapeWrite = 0,      // W01/X01: after the owner SetShape write
    AfterPayloadWrite = 1,         // W02/X02: after the real payload write
    AfterPayloadBoundary = 2,      // W03: payload written, before first binding
    AfterFirstSourceBinding = 3,   // W04/X03: after the first source Select
    AfterTreatmentAttribute = 4,   // W05/X04: after the R2 attribute write
    AfterSecondSourceBinding = 5,  // W06/X05: after the second source Select
    GraphReader = 6,               // R01
    GraphValuePresence = 7,        // R02
    GraphBytes = 8,                // R03
    GraphSourceCount = 9,          // R04
    TreatmentReader = 10,          // R05
    TreatmentPresence = 11,        // R06
    TreatmentBytes = 12,           // R07
    TreatmentBase = 13,            // R08
    TreatmentCurrentBinding = 14,  // R09
    WholeR2Validation = 15,        // R10
    OperandEqualityFirst = 16,     // R11
    OperandEqualitySecond = 17,    // R12
    AfterSideCapture = 18,         // R13: captured, before publication
    AfterCaptureBeforePublish = 19 // X06: afterCaptured, before publish
};

enum class AuthoredBooleanFaultDelivery : std::uint8_t {
    ReturnFalse = 0,
    ThrowStandardFailure = 1,
};

struct AuthoredBooleanFaultReceipt final {
    AuthoredBooleanFaultSite site = AuthoredBooleanFaultSite::AfterOwnerShapeWrite;
    AuthoredBooleanFaultDelivery delivery = AuthoredBooleanFaultDelivery::ReturnFalse;
    std::uint32_t hits = 0;
};

//! One-shot, test-only stage fault request bound to one armed owner. The hit
//! receipt is recorded exactly once, only when the armed site is reached after
//! its preceding real write or validation completed.
class AuthoredBooleanStageFault final {
public:
    AuthoredBooleanStageFault(AuthoredBooleanFaultSite site,
        AuthoredBooleanFaultDelivery delivery) noexcept
        : site_(site), delivery_(delivery) {}
    AuthoredBooleanFaultSite site() const noexcept { return site_; }
    AuthoredBooleanFaultDelivery delivery() const noexcept { return delivery_; }
    bool consumed() const noexcept { return consumed_; }
    const AuthoredBooleanFaultReceipt& receipt() const noexcept { return receipt_; }
    bool consumeAt(AuthoredBooleanFaultSite at) noexcept {
        if (consumed_ || at != site_) return false;
        consumed_ = true;
        receipt_.site = site_;
        receipt_.delivery = delivery_;
        receipt_.hits = 1;
        return true;
    }
private:
    AuthoredBooleanFaultSite site_;
    AuthoredBooleanFaultDelivery delivery_;
    bool consumed_ = false;
    AuthoredBooleanFaultReceipt receipt_;
};

struct AuthoredBooleanNamingRow final {
    std::string labelEntry;
    Standard_Integer version = 0;
};

//! Read-only document observation for the fault proof. Every naming row is
//! read from the live TNaming attributes; never a companion's saved values.
struct AuthoredBooleanObservation final {
    std::vector<AuthoredBooleanNamingRow> naming;
    Standard_Integer availableUndos = -1;
    Standard_Integer availableRedos = -1;
    bool openCommand = false;
    bool pendingCompanion = false;
    bool pendingAfterCaptured = false;
    std::size_t finalizedCompanions = 0;
};

} // namespace debug
#endif

enum class ApplyOutcome : std::uint8_t {
    Committed = 0, Unchanged, Refused, Cancelled, OutcomeUnknown,
};

struct ApplyResult final {
    ApplyOutcome outcome = ApplyOutcome::Refused;
    std::string code;
    std::optional<int> measuredUndoDelta;
};

struct Mutation final {
    Capability capability = Capability::Unregistered;
    std::uint64_t requestedFieldMask = 0;
    std::vector<std::uint8_t> sealedValues;
};

//! Document-issued immutable capture. It binds the actual native R2 source and
//! suffix, OCAF document/data, exact owner label, unit bits, scene fence,
//! history depths, and treatment-companion census. Callers cannot construct it.
class Capture final {
public:
    const retained_recipe::OwnerKey& owner() const noexcept { return owner_; }
    const std::vector<std::uint8_t>& sourceBytes() const noexcept { return sourceBytes_; }
    const std::vector<std::uint8_t>& suffixBytes() const noexcept { return suffixBytes_; }
    double metersPerUnit() const noexcept { return metersPerUnit_; }

private:
    friend class ::OcctDocument;
    friend class Owner;
    Capture() = default;
    Handle(TDocStd_Document) document_;
    Handle(TDF_Data) data_;
    TDF_Label ownerLabel_;
    retained_recipe::OwnerKey owner_;
    retained_recipe::RevisionFence revision_;
    std::shared_ptr<const retained_edge_treatment::r2::Snapshot> snapshot_;
    TopoDS_Shape untreatedBase_;
    std::vector<std::uint8_t> sourceBytes_, suffixBytes_;
    double metersPerUnit_ = 0;
    std::uint64_t metersPerUnitBits_ = 0;
    int undoDepth_ = 0, redoDepth_ = 0;
    std::size_t companionCount_ = 0;
};

//! Shared fail-closed owner used by every later authored-parameter adapter.
//! N0 can capture and validate authority, but prepare refuses every capability.
class Owner final {
public:
    static std::shared_ptr<Owner> Open(
        const Handle(OcctDocument)& document,
        std::shared_ptr<native_opening::Context> context,
        std::shared_ptr<const Capture> capture,
        std::uint32_t viewportWidth, std::uint32_t viewportHeight) noexcept;
    ~Owner();
    Owner(const Owner&) = delete;
    Owner& operator=(const Owner&) = delete;

    const Capture& capture() const noexcept;
    bool belongsTo(const Handle(OcctDocument)& document) const noexcept;
    Currentness currentness() const noexcept;
    Refusal prepare(const Mutation& mutation) noexcept;
    authored_loft::Status prepareLoft(
        std::uint32_t stationIdentifier, authored_loft::Field field,
        const rectangular_loft::Definition& requested,
        retained_edge_treatment::r2::Edit& output) noexcept;
    bool describeBoolean(part_boolean::AnalyticDefinition& output) const noexcept;
    ApplyResult applyBoolean(Capability capability, std::size_t inputIndex,
        const part_boolean::AnalyticDefinition& requested) noexcept;
    std::shared_ptr<const retained_edge_treatment::r2::Snapshot>
    retainedSnapshot() const noexcept;
    bool cancel() noexcept;
#if DEBUG
    //! Arms the existing native-opening close probe only for this exact current,
    //! unused issuer. Repeated, stale, cancelled and recovery calls fail closed.
    bool debugReportNextCloseUnproven() noexcept;
    //! SAB-N1: arms the one-shot authored-Boolean post-write stage fault for
    //! this exact current, unprepared issuer. Repeated, stale, cancelled,
    //! recovery and invalid-site requests fail closed. The armed state is
    //! bound to this opening and cleared on consumption, cancel or disposal.
    bool debugArmAuthoredBooleanStageFault(debug::AuthoredBooleanFaultSite site,
        debug::AuthoredBooleanFaultDelivery delivery) noexcept;
    //! Read-only hit receipt of the fault armed on this issuer, if any.
    bool debugAuthoredBooleanStageFaultReceipt(
        debug::AuthoredBooleanFaultReceipt& receipt) const noexcept;
#endif

private:
    struct State;
    explicit Owner(std::unique_ptr<State>) noexcept;
    std::unique_ptr<State> state_;
};

const char* RefusalCode(Refusal) noexcept;

} // namespace core3d::authored_parameter
