#pragma once

#include "RetainedEdgeTreatmentR2Values.hxx"
#include "RetainedEdgeTreatmentSnapshot.hxx"
#include "RetainedFaceSelector.hxx"
#include <AIS_Shape.hxx>
#include <TDF_Label.hxx>
#include <TopoDS_Shape.hxx>
#include <atomic>
#include <memory>
#include <optional>

class OcctDocument;
namespace core3d { class Core3DViewer; }

namespace core3d::retained_edge_treatment::r2 {
using Edit = std::variant<et::SetAmount, et::Remove, RebuildBooleanInput,
                          RebuildAnalyticTool, SetBooleanOperation, SetInputPlacement>;
struct ReplayBudget {
    std::size_t buildStages = 0, topologyVisits = 0, envelopeBytes = 0, featureIdentities = 0;
    bool chargeStage(std::size_t visits) noexcept {
        if (buildStages >= 64 || visits > 65'536 - topologyVisits) return false;
        ++buildStages; topologyVisits += visits; return true;
    }
};
struct LegacyStepSelectorBinding { std::uint64_t oldStepID = 0; fs::SelectorIntent intent; };

class Snapshot final {
    friend class ::OcctDocument; friend class core3d::Core3DViewer;
    TDF_Label ownerLabel_, sourceLabel_;
    rr::OwnerSnapshot owner_;
    BaseRecipe source_;
    Definition definition_;
    std::vector<std::uint8_t> sourceBytes_, definitionBytes_;
    // The proven pre-Boolean source base. An admitted prefix source edit
    // rebuilds the complete prefix from this shape, never from the treated
    // current shape.
    TopoDS_Shape sourceBase_;
    TopoDS_Shape base_, current_;
    ReplayBudget chargedBudget_;
    std::uint64_t nonce_ = 0, presentationRevision_ = 0;
public:
    const rr::OwnerSnapshot& owner() const noexcept { return owner_; }
    const BaseRecipe& source() const noexcept { return source_; }
    const Definition& definition() const noexcept { return definition_; }
    const std::vector<std::uint8_t>& canonicalBytes() const noexcept { return definitionBytes_; }
    bool current() const noexcept { return owner_.status == rr::OwnerStatus::CurrentEditable && !current_.IsNull(); }
    double dimensionMetersPerUnit() const noexcept {
        return std::visit([](const auto& base) { return base.metersPerLocalUnit; }, definition_.base);
    }
};

class MigrationCapture final {
    friend class ::OcctDocument; friend class core3d::Core3DViewer;
    TDF_Label ownerLabel_, sourceLabel_;
    rr::OwnerSnapshot owner_;
    retained_boolean::Recipe original_;
    std::vector<std::uint8_t> originalBytes_;
    TopoDS_Shape originalBase_, originalCurrent_;
    ReplayBudget chargedBudget_;
    std::uint64_t nonce_ = 0, presentationRevision_ = 0;
public:
    const rr::OwnerSnapshot& owner() const noexcept { return owner_; }
    const retained_boolean::Recipe& source() const noexcept { return original_; }
    const std::vector<std::uint8_t>& canonicalOriginalBytes() const noexcept { return originalBytes_; }
};
class MigrationReview final {
    friend class core3d::Core3DViewer;
    std::shared_ptr<const MigrationCapture> original_;
    Digest requestDigest_{};
    MigrationM3 request_;
    ReplayBudget chargedBudget_;
    std::vector<std::shared_ptr<const fs::FaceMembershipProof>> proofs_;
};
// A1 first enrollment: the read-only capture of an A1-composite RetainedBoolean
// owner that holds no R2 record yet, and the reviewed first append (selector
// proof resolved against the exact untreated carrier shape). The composite
// carrier is the enrollment authority and is never restaged by this lane.
class EnrollmentCapture final {
    friend class ::OcctDocument; friend class core3d::Core3DViewer;
    TDF_Label ownerLabel_, sourceLabel_;
    rr::OwnerSnapshot owner_;
    composite_recipe::Definition original_;
    std::vector<std::uint8_t> originalBytes_;
    TopoDS_Shape originalCurrent_;
    ReplayBudget chargedBudget_;
    std::uint64_t nonce_ = 0, presentationRevision_ = 0;
public:
    const rr::OwnerSnapshot& owner() const noexcept { return owner_; }
    const composite_recipe::Definition& source() const noexcept { return original_; }
    const std::vector<std::uint8_t>& canonicalOriginalBytes() const noexcept { return originalBytes_; }
};
class EnrollmentReview final {
    friend class core3d::Core3DViewer;
    std::shared_ptr<const EnrollmentCapture> original_;
    Digest requestDigest_{};
    SelectorAppendIntent request_;
    std::shared_ptr<const fs::FaceMembershipProof> proof_;
    ReplayBudget chargedBudget_;
};
class SelectorTargetCapture final {
    friend class core3d::Core3DViewer;
    std::shared_ptr<const Snapshot> original_;
    std::shared_ptr<const fs::FaceMembershipProof> proof_;
    ReplayBudget chargedBudget_;
};

class DetachedInput final {
    friend class core3d::Core3DViewer;
    std::uint64_t nonce_ = 0;
    Definition candidate_;
    BaseRecipe source_;
    // The admitted edit and the proven pre-Boolean source base travel across
    // the detached boundary as values/shapes only; no document authority.
    std::optional<Edit> edit_;
    TopoDS_Shape sourceBase_;
    TopoDS_Shape base_, originalCurrent_;
    ReplayBudget chargedBudget_;
    std::shared_ptr<std::atomic_bool> cancelled_;
};
class DetachedResult final {
    friend class ::OcctDocument; friend class core3d::Core3DViewer;
    std::uint64_t nonce_ = 0;
    Definition definition_;
    BaseRecipe source_;
    std::vector<std::uint8_t> definitionBytes_, prefixBytes_;
    // Set only when an admitted source edit rebuilt the prefix: the restaged
    // pre-Boolean source base and a marker the staging path requires before
    // it may touch the retained source carrier.
    TopoDS_Shape editedSourceBase_;
    bool sourceChanged_ = false;
    TopoDS_Shape base_, result_;
    ReplayBudget budget_;
public:
    const Definition& definition() const noexcept { return definition_; }
    const TopoDS_Shape& result() const noexcept { return result_; }
    const std::vector<std::uint8_t>& canonicalPrefixBytes() const noexcept { return prefixBytes_; }
    const TopoDS_Shape& editedSourceBase() const noexcept { return editedSourceBase_; }
    bool sourceChanged() const noexcept { return sourceChanged_; }
};
class Work final {
    friend class core3d::Core3DViewer;
public: enum class State { Fresh, Prepared, Building, Ready, Committing, Settled };
private:
    std::shared_ptr<const Snapshot> snapshot_;
    std::shared_ptr<const MigrationCapture> migration_;
    std::shared_ptr<const MigrationReview> review_;
    std::shared_ptr<const EnrollmentCapture> enrollment_;
    std::variant<Edit, MigrationM3> mutation_;
    Definition candidate_;
    BaseRecipe source_;
    TopoDS_Shape sourceBase_;
    std::vector<std::uint8_t> prefixBytes_;
    TopoDS_Shape base_, originalCurrent_;
    ReplayBudget chargedBudget_;
    std::uint64_t nonce_ = 0;
    TDF_Label label_; Handle(AIS_Shape) presentation_;
    std::shared_ptr<std::atomic_bool> cancelled_ = std::make_shared<std::atomic_bool>(false);
    State state_ = State::Fresh;
};
struct Record {
    TDF_Label label, owner;
    std::shared_ptr<const Definition> definition;
    std::vector<std::uint8_t> bytes;
    // The exact retained source-carrier bytes staged by the same transaction;
    // empty only when the carrier was not touched.
    std::vector<std::uint8_t> sourceBytes;
    TopoDS_Shape base, current;
};
} // namespace core3d::retained_edge_treatment::r2
