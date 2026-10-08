#pragma once

#include "NativeOpeningContext.hxx"
#include "RetainedEdgeTreatmentR2Snapshot.hxx"

#include <Standard_Handle.hxx>
#include <TDF_Label.hxx>

#include <cstdint>
#include <memory>
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
    bool cancel() noexcept;

private:
    struct State;
    explicit Owner(std::unique_ptr<State>) noexcept;
    std::unique_ptr<State> state_;
};

const char* RefusalCode(Refusal) noexcept;

} // namespace core3d::authored_parameter
