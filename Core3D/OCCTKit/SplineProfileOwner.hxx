#pragma once
#include "NativeOpeningContext.hxx"
#include "SplineProfilePersistence.hxx"
#include <memory>
#include <string>

class OcctDocument;
struct OcctSplineProfileCapture;

namespace core3d::spline_profile::owner {
enum class Outcome : std::uint8_t {
    captured, prepared, committed, cancelled, unchanged, refused, busy,
    staleScene, staleDefinition, unsupportedDescendant, recoveryRequired,
    outcomeUnknown
};

struct Opening final {
    Definition definition;
    std::vector<std::uint8_t> canonical;
    retained_recipe::Digest digest{};
    std::uint64_t session = 0;
};

struct Prepared final {
    Opening opening;
    Definition replacement;
    DetachedSolid detached;
    std::uint64_t preparation = 0;
};

struct Receipt final {
    Outcome outcome = Outcome::refused;
    std::string reason;
    std::uint64_t session = 0;
    std::uint64_t preparation = 0;
    int historyDelta = 0;
};

class OcafOwner final {
public:
    OcafOwner(OcctDocument&, std::shared_ptr<native_opening::Context>) noexcept;
    ~OcafOwner();
    std::shared_ptr<const Opening> beginCreate(const Definition&) noexcept;
    std::shared_ptr<const Opening> capture(const std::string&) noexcept;
    std::shared_ptr<const Prepared> prepare(const std::shared_ptr<const Opening>&,
        const Definition&, Receipt&) noexcept;
    Receipt apply(const std::shared_ptr<const Prepared>&,
                  const std::string& requestedName = "Spline Profile") noexcept;
    Receipt cancel(std::uint64_t) noexcept;
private:
    struct State;
    std::unique_ptr<State> state_;
};
} // namespace core3d::spline_profile::owner
