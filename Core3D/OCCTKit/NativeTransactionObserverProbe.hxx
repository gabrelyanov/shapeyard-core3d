#pragma once
// DEBUG qualification seam only. Never used to issue production edit authority.
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

namespace core3d::debug {
struct TransactionObservation {
    enum class Kind : std::uint8_t { Open, Commit, Abort } kind;
    bool adopted;
    bool commandOpen;
    int undos;
    int redos;
};
struct TransactionProbeState {
    // Non-owning identity, explicitly cleared before detach/close. A production
    // opening identity must not be an address; this experiment records dispatch.
    const TDocStd_Document* adopted = nullptr;
    std::array<TransactionObservation, 128> events{};
    std::size_t count = 0;
    bool valid = true;
};
class ObservedApplication final : public TDocStd_Application {
public:
    explicit ObservedApplication(std::weak_ptr<TransactionProbeState> state) : state_(std::move(state)) {}
    void OnOpenTransaction(const Handle(TDocStd_Document)& doc) override { record(TransactionObservation::Kind::Open, doc); }
    void OnCommitTransaction(const Handle(TDocStd_Document)& doc) override { record(TransactionObservation::Kind::Commit, doc); }
    void OnAbortTransaction(const Handle(TDocStd_Document)& doc) override { record(TransactionObservation::Kind::Abort, doc); }
private:
    void record(TransactionObservation::Kind kind, const Handle(TDocStd_Document)& doc) noexcept {
        const auto state = state_.lock();
        if (!state || !state->valid) return;
        try {
            if (doc.IsNull() || state->count == state->events.size()) { state->valid = false; return; }
            state->events[state->count++] = {kind, doc.get() == state->adopted,
                bool(doc->HasOpenCommand()), doc->GetAvailableUndos(), doc->GetAvailableRedos()};
        } catch (...) { state->valid = false; }
    }
    std::weak_ptr<TransactionProbeState> state_;
};
}
