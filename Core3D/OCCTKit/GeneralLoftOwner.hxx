#pragma once

// Production C3-N owner. Descriptive dictionaries never recreate this object.
#include "GeneralLoftPersistence.hxx"
#include "NativeOpeningContext.hxx"
#include <TDF_Label.hxx>
#include <cstdint>
#include <memory>
#include <string>

class OcctDocument;
struct OcctGeneralLoftCapture;

namespace core3d::general_loft::owner {
enum class Outcome : std::uint8_t {
    captured, prepared, committed, cancelled, unchanged, refused, busy,
    staleScene, staleOwner, staleDefinition, unsupportedStructure,
    unsupportedDependent, outcomeUnknown, recoveryRequired
};

struct SceneFence final {
    std::uint64_t documentGeneration = 0;
    std::uint64_t modelRevision = 0;
    double metersPerUnit = 0;
};

struct Opening final {
    SceneFence scene;
    Definition definition;
    std::uint64_t session = 0;
    bool creating = false;
    std::string requestedName;
};

struct CreationStation final {
    bounded_curve::Frame frame;
    double orderParameter = 0;
    double twistFromPreviousRadians = 0;
    std::vector<std::array<double, 2>> junctions;
};

struct CreateRequest final {
    SceneFence expectedScene;
    std::vector<CreationStation> stations;
    std::array<double, 3> orderAxis{{0, 0, 1}};
    std::string requestedName;
};

struct Candidate final {
    // Must carry the exact opening values except for one station's numeric
    // frame/curve values. recipeDigest is ignored and replaced natively.
    Definition descriptive;
    UUID editedStation{};
};

struct Prepared final {
    Opening opening;
    Definition candidate;
    AdmittedSolid admitted;
    std::uint64_t preparation = 0;
};

struct Receipt final {
    Outcome outcome = Outcome::refused;
    std::string reason;
    SceneFence scene;
    Definition definition;
    std::uint64_t session = 0, preparation = 0;
    int historyDelta = 0;
};

class OcafOwner final {
public:
    explicit OcafOwner(OcctDocument&, std::shared_ptr<native_opening::Context>) noexcept;
    ~OcafOwner();
    OcafOwner(const OcafOwner&) = delete;
    OcafOwner& operator=(const OcafOwner&) = delete;

    std::shared_ptr<const Opening> beginCreation(const CreateRequest&, Receipt&) noexcept;
    std::shared_ptr<const Opening> capture(const std::string& entityIdentifier,
                                           const SceneFence&) noexcept;
    std::shared_ptr<const Prepared> prepare(const std::shared_ptr<const Opening>&,
                                            const Candidate&, Receipt&) noexcept;
    Receipt apply(const std::shared_ptr<const Prepared>&) noexcept;
    Receipt cancel(std::uint64_t session) noexcept;
private:
    struct State;
    std::unique_ptr<State> state_;
};
} // namespace core3d::general_loft::owner
