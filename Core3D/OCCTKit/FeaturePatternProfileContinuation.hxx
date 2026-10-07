#pragma once

// Truthful native continuation for a Profile baseline followed by one D4
// cut-feature pattern. SYFB recipe bytes remain opaque corroboration: Profile
// is the only decoder/builder and the complete D4 result is always rebuilt.

#include "FeaturePatternOwnerBridge.hxx"
#include "NativeOpeningDependentReplay.hxx"
#include "ProfilePersistence.hxx"

#include <atomic>
#include <memory>
#include <set>
#include <string>
#include <vector>

class OcctDocument;

namespace core3d::profile_d4 {

enum class CaptureStatus : std::uint8_t { Current, Absent, Refused };
enum class CreationOutcome : std::uint8_t { Refused, Committed, OutcomeUnknown };

struct CreationEdit final {
    std::uint64_t sourceCutStepID = 0;
    pattern::Kind kind = pattern::Kind::Linear;
    pattern::Axis rowAxis = pattern::Axis::Y;
    pattern::Axis columnAxis = pattern::Axis::X;
    std::uint32_t rows = 1;
    std::uint32_t columns = 3;
    double rowSpacing = 0;
    double columnSpacing = 0;
    double sweepRadians = pattern::TwoPi;
    std::array<double, 3> radialPivotLocal{{0, 0, 0}};
    std::set<pattern::Coordinate> suppressed;
};

//! Immutable projection of the measurements and charged budget retained by
//! the one real detached creation build. It carries no shape or stage authority.
struct PreparedReview final {
    std::size_t attributedChildCount = 0;
    std::uint32_t sectionCount = 0;
    std::vector<double> positiveRemovedVolumes;
    double measuredPairwiseLigamentMM = 0;
    feature_pattern::Projection chargedProjection;
};

struct Observation final {
    bool current = false;
    std::string hostEntity, sourceEntity, profileFeature, patternFeature;
    std::uint64_t sourceCutStepID = 0;
    std::size_t memberCount = 0, childCount = 0;
    std::size_t profileScalarCount = 0, sourceProgramBytes = 0;
    double metersPerUnit = 0;
    double baselineVolume = 0, resultVolume = 0, sourceVolume = 0;
};

class CreationCapture final {
    friend CaptureStatus CaptureCreationHost(OcctDocument&, const std::string&,
        const std::string&, const std::shared_ptr<native_opening::Context>&,
        std::uint32_t, std::uint32_t,
        std::shared_ptr<const CreationCapture>&) noexcept;
    friend std::shared_ptr<const class PreparedCreation> PrepareCreation(
        const std::shared_ptr<const CreationCapture>&, const CreationEdit&,
        const std::atomic_bool&) noexcept;
    friend CreationOutcome StageCreation(OcctDocument&,
        const std::shared_ptr<const class PreparedCreation>&) noexcept;
    friend bool CreationReadback(OcctDocument&, const class PreparedCreation&,
        const TDF_Label&) noexcept;
    CreationCapture() = default;
    pattern_owner::LabelReceipt host_, source_;
    feature_pattern::UUID document_{};
    profile::Record profile_;
    std::vector<std::uint8_t> scalarRecipe_;
    OcctCylindricalCutProgramSource sourceProgram_;
    std::shared_ptr<native_opening::Context> context_;
    Standard_Integer documentTime_ = -1;
    std::uint32_t width_ = 0, height_ = 0;
    std::size_t freeLabelCount_ = 0, featurePatternBytes_ = 0;
};

class PreparedCreation final {
    friend std::shared_ptr<const PreparedCreation> PrepareCreation(
        const std::shared_ptr<const CreationCapture>&, const CreationEdit&,
        const std::atomic_bool&) noexcept;
    friend CreationOutcome StageCreation(OcctDocument&,
        const std::shared_ptr<const PreparedCreation>&) noexcept;
    friend bool CreationReadback(OcctDocument&, const PreparedCreation&,
        const TDF_Label&) noexcept;
    friend bool ReviewPreparedCreation(const PreparedCreation&,
        PreparedReview&) noexcept;
    PreparedCreation() = default;
    std::shared_ptr<const CreationCapture> capture_;
    feature_pattern::Definition definition_;
    feature_pattern_native::HostBaseline baseline_;
    feature_pattern_native::AttributedBuild built_;
    mutable std::atomic_bool consumed_{false};
};

//! Captures two independent current objects: a Profile host and a distinct
//! same-document full retained Program source. Existing, partial, corrupt or
//! ambiguous D4 relationships refuse; they are never treated as absence.
CaptureStatus CaptureCreationHost(OcctDocument&, const std::string& hostEntity,
    const std::string& sourceEntity,
    const std::shared_ptr<native_opening::Context>&,
    std::uint32_t viewportWidth, std::uint32_t viewportHeight,
    std::shared_ptr<const CreationCapture>&) noexcept;

//! Issues every new durable identity natively and performs the detached,
//! cancellable attributed build. The result is single-use sealed authority.
std::shared_ptr<const PreparedCreation> PrepareCreation(
    const std::shared_ptr<const CreationCapture>&, const CreationEdit&,
    const std::atomic_bool& stop) noexcept;

//! Reads only retained values from the sealed prepared capability. It neither
//! mutates, stages, rebuilds, reconciles nor consumes the capability.
bool ReviewPreparedCreation(const PreparedCreation&,
    PreparedReview&) noexcept;

//! Exact recapture, one lease, baseline/pair/Profile stage, full readback, one
//! close and owning-viewer publication. Unknown close/publication is retained.
CreationOutcome StageCreation(OcctDocument&,
    const std::shared_ptr<const PreparedCreation>&) noexcept;

#if DEBUG
//! Default-disarmed, one-shot test fault consumed only by StageCreation.
//! Every fault exits through StageCreation's real command-abort path.
enum class CreationFault : std::uint8_t {
    None = 0,
    AfterLastChild = 1,
    CreationReadback = 2,
};
void DebugArmCreationFault(CreationFault) noexcept;
#endif

class CurrentCapture final {
    friend CaptureStatus CaptureCurrent(OcctDocument&, const std::string&,
        const std::shared_ptr<native_opening::Context>&,
        std::shared_ptr<const CurrentCapture>&) noexcept;
    friend std::shared_ptr<const class PreparedHostEdit> PrepareHostEdit(
        OcctDocument&, const std::shared_ptr<const CurrentCapture>&,
        const profile::Parameters&, const TopoDS_Shape&,
        const std::atomic_bool&) noexcept;
    friend bool OpeningMatches(const class PreparedHostEdit&,
        const std::shared_ptr<native_opening::Context>&) noexcept;
    friend dependent_replay::Refusal PrepareHostReplay(OcctDocument&,
        const std::shared_ptr<const class PreparedHostEdit>&,
        const dependent_replay::Limits&,
        std::shared_ptr<const dependent_replay::PreparedReplay>&) noexcept;
    friend class ProfileHostPreparer;
    friend bool ObserveCurrent(OcctDocument&, const std::string&,
        const std::shared_ptr<native_opening::Context>&, Observation&) noexcept;
    CurrentCapture() = default;
    feature_pattern_owner::Snapshot d4_;
    profile::Record profile_;
    std::vector<std::uint8_t> scalarRecipe_;
    // The snapshot keeps immutable viewer-issued opening evidence, not the
    // viewer's one live opening slot. Ordinary editing is sequential: the
    // read context retires before a later commit context is acquired.
    std::shared_ptr<const native_opening::Fence> openingFence_;
};

class PreparedHostEdit final {
    friend std::shared_ptr<const PreparedHostEdit> PrepareHostEdit(
        OcctDocument&, const std::shared_ptr<const CurrentCapture>&,
        const profile::Parameters&, const TopoDS_Shape&,
        const std::atomic_bool&) noexcept;
    friend class ProfileHostPreparer;
    friend const TopoDS_Shape& Baseline(const PreparedHostEdit&) noexcept;
    friend const TopoDS_Shape& Result(const PreparedHostEdit&) noexcept;
    friend bool OpeningMatches(const PreparedHostEdit&,
        const std::shared_ptr<native_opening::Context>&) noexcept;
    friend dependent_replay::Refusal PrepareHostReplay(OcctDocument&,
        const std::shared_ptr<const PreparedHostEdit>&,
        const dependent_replay::Limits&,
        std::shared_ptr<const dependent_replay::PreparedReplay>&) noexcept;
    PreparedHostEdit() = default;
    std::shared_ptr<const CurrentCapture> capture_;
    profile::Parameters requested_;
    std::vector<std::uint8_t> scalarRecipe_;
    feature_pattern_owner::PreparedEdit prepared_;
};

CaptureStatus CaptureCurrent(OcctDocument&, const std::string& hostEntity,
    const std::shared_ptr<native_opening::Context>&,
    std::shared_ptr<const CurrentCapture>&) noexcept;

std::shared_ptr<const PreparedHostEdit> PrepareHostEdit(OcctDocument&,
    const std::shared_ptr<const CurrentCapture>&,
    const profile::Parameters&, const TopoDS_Shape& rebuiltProfileBaseline,
    const std::atomic_bool& stop) noexcept;

const TopoDS_Shape& Baseline(const PreparedHostEdit&) noexcept;
const TopoDS_Shape& Result(const PreparedHostEdit&) noexcept;
//! Proves that the sequential commit opening is the exact captured opening;
//! a later fresh capture can never launder stale Profile/D4 authority.
bool OpeningMatches(const PreparedHostEdit&,
    const std::shared_ptr<native_opening::Context>&) noexcept;
//! Prepares the one proved Profile-host D4 edge without asking the generic
//! free-label adapter to reinterpret D4 metadata as a simple source label.
dependent_replay::Refusal PrepareHostReplay(OcctDocument&,
    const std::shared_ptr<const PreparedHostEdit>&,
    const dependent_replay::Limits&,
    std::shared_ptr<const dependent_replay::PreparedReplay>&) noexcept;
bool ObserveCurrent(OcctDocument&, const std::string& hostEntity,
    const std::shared_ptr<native_opening::Context>&, Observation&) noexcept;

class ProfileHostPreparer final : public dependent_replay::Preparer {
public:
    explicit ProfileHostPreparer(
        std::shared_ptr<const PreparedHostEdit> prepared) noexcept
        : prepared_(std::move(prepared)) {}
    dependent_replay::Refusal prepare(OcctDocument&,
        const dependent_replay::Dependency&, dependent_replay::Mutation,
        std::shared_ptr<const dependent_replay::PreparedReplay>&) noexcept override;
private:
    std::shared_ptr<const PreparedHostEdit> prepared_;
};

} // namespace core3d::profile_d4
