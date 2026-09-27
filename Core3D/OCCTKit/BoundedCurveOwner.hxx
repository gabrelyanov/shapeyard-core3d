#pragma once

// C1-N production owner contract. Implemented by BoundedCurveOwner.mm against
// OcctDocument; callers never manufacture authority from topology or values.
#include "BoundedCurveAttribute.hxx"
#include "BoundedCurveEdit.hxx"
#include <TDF_Label.hxx>
#include <TopoDS_Wire.hxx>
#include <cstdint>
#include <memory>
#include <string>

class OcctDocument;

namespace core3d::bounded_curve::owner {
enum class Outcome : std::uint8_t {
    captured, prepared, committed, cancelled, unchanged, refused, busy,
    staleGeneration, staleModelRevision, staleOwner, staleDefinition,
    staleFrame, staleDigest, tombstoned, unknownControlPoint,
    unsupportedDependent, outcomeUnknown, recoveryRequired
};

struct SceneFence final {
    std::uint64_t documentGeneration = 0;
    std::uint64_t modelRevision = 0;
    double metersPerUnit = 0;
};

struct Opening final {
    SceneFence scene;
    TDF_Label ownerLabel;
    RetainedState retained;
    TopoDS_Wire wire;
    std::uint64_t session = 0;
};

// Fresh read-only receipt for later D3. It contains authority only: no poles,
// knots, weights or topology index, and is never a path-array mutation.
struct PathReceipt final {
    SceneFence scene;
    retained_recipe::OwnerKey owner;
    UUID feature{};
    std::uint64_t definitionRevision = 0;
    UUID frame{};
    std::uint64_t frameRevision = 0;
    std::uint64_t nextLocalID = 0;
    Digest canonicalDefinitionDigest{};
    std::uint64_t issuance = 0;
};

struct CreateRequest final {
    SceneFence expectedScene;
    PersistedValue persisted;
    TopoDS_Wire detachedWire;
    std::string requestedName;
};

struct Candidate final {
    EditProposal proposal;
    // Complete values are mandatory even for a pole/handle gesture. The
    // native owner compares them to PrepareEdit's candidate; no reduced DTO.
    Definition completeDefinition;
};

struct Prepared final {
    Opening opening;
    PreparedEdit values;
    PersistedValue persisted;
    TopoDS_Wire detachedWire;
    std::uint64_t preparation = 0;
};

struct Receipt final {
    Outcome outcome = Outcome::refused;
    std::string reason;
    SceneFence scene;
    Authority authority;
    std::uint64_t session = 0;
    std::uint64_t preparation = 0;
    int historyDelta = 0;
};

class OcafOwner final {
public:
    explicit OcafOwner(OcctDocument&) noexcept;
    ~OcafOwner();
    OcafOwner(const OcafOwner&) = delete;
    OcafOwner& operator=(const OcafOwner&) = delete;

    bool boundToCurrentDocument() const noexcept;
    bool blocksOtherWork() const noexcept;
    void retireForDocumentReplacement() noexcept;

    // One real OCAF command: owner UUIDs + immutable SYCV/SYCO attribute +
    // exact forward WIRE binding + presentation. Failure aborts the whole set.
    Receipt create(const CreateRequest&) noexcept;

    // entityIdentifier is resolved through current native object authority.
    // No topology index, pole ordinal, copied descriptor or nearest match.
    std::shared_ptr<const Opening> capture(
        const std::string& entityIdentifier,
        const SceneFence& expectedScene) noexcept;

    // Detached geometry and all admitted dependent rebuilds finish here,
    // before a document command. Unsupported dependents refuse atomically.
    std::shared_ptr<const Prepared> prepare(
        const std::shared_ptr<const Opening>&,
        const Candidate&,
        Receipt&) noexcept;

    // Re-read generation/model/owner/definition/frame/digest/issuance fences,
    // then stage, replace, replay, verify and close exactly one command.
    Receipt apply(const std::shared_ptr<const Prepared>&) noexcept;

    // Retires only this owner's detached preparation; zero document history.
    Receipt cancel(std::uint64_t session) noexcept;

    // Always re-resolves and re-reads current native authority. Sketch2D,
    // stale, ambiguous, closed or invalid records return no receipt.
    std::shared_ptr<const PathReceipt> pickCurrentPath3D(
        const std::string& entityIdentifier,
        const SceneFence& expectedScene) const noexcept;

private:
    struct State;
    std::unique_ptr<State> state_;
};

inline bool ValidPathReceipt(const PathReceipt& value) noexcept {
    return value.scene.documentGeneration != 0
        && value.scene.modelRevision != 0
        && Finite(value.scene.metersPerUnit) && value.scene.metersPerUnit > 0
        && retained_recipe::Valid(value.owner) && Nonzero(value.feature)
        && value.definitionRevision != 0 && Nonzero(value.frame)
        && value.frameRevision != 0 && value.nextLocalID != 0
        && retained_recipe::Nonzero(value.canonicalDefinitionDigest)
        && value.issuance != 0;
}
} // namespace core3d::bounded_curve::owner
