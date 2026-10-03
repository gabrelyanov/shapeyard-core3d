#pragma once
#include "RetainedEdgeTreatmentAttribute.hxx"
#include "RetainedRecipeSnapshot.hxx"
#include "RetainedTopologyBudget.hxx"
#include "ProfilePersistence.hxx"
#include "EnclosurePersistence.hxx"
#include "RectangularLoftDefinition.hxx"
#include <AIS_Shape.hxx>
#include <atomic>
#include <variant>

class OcctDocument;
namespace core3d { class Core3DViewer; }
namespace core3d::retained_face_selector { class FaceMembershipProof; }

namespace core3d::retained_edge_treatment {
using BaseRecipe=std::variant<profile::Parameters,enclosure::Parameters,rectangular_loft::Definition>;
struct Append {Kind kind;double amountMM;std::vector<Anchor> anchors;};
struct SetAmount {UUID feature;double amountMM;};
struct ReplaceTargets {UUID feature;std::vector<Anchor> anchors;};
struct Remove {UUID feature;};
struct RebuildSource {BaseRecipe requested;};
using Edit=std::variant<Append,SetAmount,ReplaceTargets,Remove,RebuildSource>;
// B1 replay accounting is the common per-operation topology counter (frozen
// B2 limits: 4,096 faces+edges per stage census, 65,536 visits, 64 stages).
// The public buildStages/topologyVisits members and chargeStage(visits)
// semantics are inherited unchanged; the base adds validation-first
// arithmetic, sticky refusal and the DEBUG per-site trace.
struct ReplayBudget : retained_topology_budget::Counter {};
class Snapshot final {friend class ::OcctDocument;friend class core3d::Core3DViewer;TDF_Label ownerLabel_,sourceLabel_;std::string sourceIdentifier_;retained_recipe::OwnerSnapshot owner_;BaseRecipe source_;Definition seed_;std::optional<Definition> definition_;std::vector<std::uint8_t> sourceBytes_,definitionBytes_;TopoDS_Shape base_,current_;std::uint64_t nonce_=0,presentationRevision_=0;ReplayBudget chargedBudget_;public:const retained_recipe::OwnerSnapshot& owner()const noexcept{return owner_;}const BaseRecipe& source()const noexcept{return source_;}const Definition& effectiveDefinition()const noexcept{return definition_?*definition_:seed_;}const std::optional<Definition>& definition()const noexcept{return definition_;}const std::vector<std::uint8_t>& canonicalBytes()const noexcept{return definitionBytes_;}bool current()const noexcept{return owner_.status==retained_recipe::OwnerStatus::CurrentEditable&&!current_.IsNull();}double dimensionMetersPerUnit()const noexcept{return seed_.base.metersPerLocalUnit;}};
class SelectorTargetCapture final {
    friend class core3d::Core3DViewer;
    std::shared_ptr<const Snapshot> original_;
    std::shared_ptr<const retained_face_selector::FaceMembershipProof> proof_;
    ReplayBudget chargedBudget_;
    SelectorTargetCapture(std::shared_ptr<const Snapshot> original,
        std::shared_ptr<const retained_face_selector::FaceMembershipProof> proof,
        ReplayBudget budget) : original_(std::move(original)), proof_(std::move(proof)), chargedBudget_(budget) {}
public:
    const retained_face_selector::FaceMembershipProof& proof() const noexcept { return *proof_; }
};
class SelectorAppendValues final {
    friend class core3d::Core3DViewer; friend class ::OcctDocument;
    std::uint64_t captureNonce_=0;
    retained_recipe::RevisionFence fence_;
    BaseRecipe source_;
    BaseBinding base_;
    Append edit_;
    Step issuedStep_;
    ReplayBudget chargedBudget_;
    SelectorAppendValues()=default;
};
class DetachedInput final {friend class core3d::Core3DViewer;std::shared_ptr<const Snapshot> snapshot_;std::shared_ptr<const SelectorAppendValues> selectorAppend_;Edit edit_;Definition candidate_;BaseRecipe source_;std::uint64_t nonce_=0;TopoDS_Shape base_;ReplayBudget chargedBudget_;std::shared_ptr<std::atomic_bool> cancelled_;};
struct StepProof {UUID feature{};Digest input{},output{};double inputVolumeMM3=0,outputVolumeMM3=0,expectedRemovedMM3=0;std::vector<UUID> consumedKeys;};
class DetachedResult final {friend class core3d::Core3DViewer;friend class ::OcctDocument;std::uint64_t nonce_=0;Digest editDigest_{};BaseRecipe source_;Definition definition_;std::vector<std::uint8_t> definitionBytes_;TopoDS_Shape base_,result_;std::vector<StepProof> proofs_;ReplayBudget budget_;std::shared_ptr<const SelectorAppendValues> selectorAppend_;public:const TopoDS_Shape& result()const noexcept{return result_;}const Definition& definition()const noexcept{return definition_;}};
class Work final {friend class core3d::Core3DViewer;public:enum class State{Fresh,Prepared,Building,Ready,Committing,Settled};private:std::shared_ptr<const Snapshot> snapshot_;std::shared_ptr<const SelectorAppendValues> selectorAppend_;Edit edit_;Definition candidate_;TDF_Label label_;Handle(AIS_Shape)presentation_;std::shared_ptr<std::atomic_bool> cancelled_=std::make_shared<std::atomic_bool>(false);State state_=State::Fresh;};
enum class CommitOutcome {Committed,Unchanged,Refused,Cancelled,Busy,OutcomeUnknown};
struct CommitResult {CommitOutcome outcome=CommitOutcome::Refused;Refusal refusal=Refusal::StageFailed;std::optional<std::int64_t> measuredUndoDelta=0;std::vector<UUID> replayedFeatures;};
} // namespace core3d::retained_edge_treatment
