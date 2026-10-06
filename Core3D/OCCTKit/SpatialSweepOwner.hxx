#pragma once

// Production owner for one native C2 spatial-sweep creation. Candidate
// values are descriptive; this owner mints every durable identity, prepares
// all geometry detached, and owns exactly one OCAF command at publication.
#include "BoundedCurveDefinition.hxx"
#include "NativeOpeningContext.hxx"
#include "SpatialSweepDefinition.hxx"

#include <cstdint>
#include <memory>
#include <string>

class OcctDocument;

namespace core3d::spatial_sweep::owner {

enum class Outcome : std::uint8_t {
    committed = 0, refused, busy, cancelled, outcomeUnknown, recoveryRequired
};

struct SceneFence final {
    std::uint64_t documentGeneration = 0;
    std::uint64_t modelRevision = 0;
    double metersPerUnit = 0;
};

struct CreateRequest final {
    SceneFence expectedScene;
    bounded_curve::Definition curve;
    Definition sweep;
    std::string requestedName;
};

struct Receipt final {
    Outcome outcome = Outcome::refused;
    std::string reason;
    std::string entityIdentifier;
    int historyDelta = 0;
};

class OcafOwner final {
public:
    explicit OcafOwner(OcctDocument&,
                       std::shared_ptr<native_opening::Context>) noexcept;
    ~OcafOwner();
    OcafOwner(const OcafOwner&) = delete;
    OcafOwner& operator=(const OcafOwner&) = delete;

    bool boundToCurrentDocument() const noexcept;
    bool blocksOtherWork() const noexcept;
    void retireForDocumentReplacement() noexcept;
    Receipt create(const CreateRequest&) noexcept;
    Receipt cancel(std::uint64_t session) noexcept;

private:
    struct State;
    std::unique_ptr<State> state_;
};

} // namespace core3d::spatial_sweep::owner
