#pragma once
// DEBUG-only bounded diagnostic. Observes actual application callbacks and explicit
// adoption/history boundaries; never grants AI execution authority.
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <utility>

namespace core3d::debug {
struct LiveTransactionObservation {
    enum class Kind : std::uint8_t { Attached, Open, Commit, Abort, UndoCompleted,
        RedoCompleted, Adopted, Detached } kind;
    std::uint64_t sequence=0;
    // Diagnostic opening ordinal only; not a persisted/public authority token.
    std::uint64_t opening=0;
    bool active=false,commandOpen=false;
    int undos=0,redos=0;
};
struct LiveTransactionProbeState {
    const TDocStd_Document* active=nullptr;
    std::thread::id thread=std::this_thread::get_id();
    std::array<LiveTransactionObservation,256> events{};
    std::size_t count=0;
    std::uint64_t opening=0;
    // Only the owning thread touches ordinary fields. A foreign callback must
    // not race a query by writing valid/count/active from another thread.
    bool valid=true;
    std::atomic_bool foreignThreadObserved{false};
    bool OnOwningThread() noexcept {
        if (thread==std::this_thread::get_id()) return true;
        foreignThreadObserved.store(true,std::memory_order_relaxed);return false;
    }
    bool IsValid() const noexcept {
        return thread==std::this_thread::get_id()
            && !foreignThreadObserved.load(std::memory_order_relaxed) && valid;
    }
    void Record(LiveTransactionObservation::Kind kind,const Handle(TDocStd_Document)& doc) noexcept {
        if (!OnOwningThread() || !IsValid()) return;
        try {
            if (doc.IsNull() || count==events.size()) {
                valid=false;return;
            }
            const auto sequence=count+1;
            events[count++]={kind,sequence,opening,doc.get()==active,bool(doc->HasOpenCommand()),
                doc->GetAvailableUndos(),doc->GetAvailableRedos()};
        } catch (...) { valid=false; }
    }
    // Call only after a real successful replacement, not ChangeDocument()'s
    // provisional candidate assignment. Failed/reverted candidates stay false.
    void Adopt(const Handle(TDocStd_Document)& doc,bool initial=false) noexcept {
        if (!OnOwningThread() || !IsValid()) return;
        try {
            if (doc.IsNull()
                || doc->HasOpenCommand() || doc.get()==active || count==events.size()) {
                valid=false;return;
            }
            active=doc.get();++opening;
            Record(initial?LiveTransactionObservation::Kind::Attached:LiveTransactionObservation::Kind::Adopted,doc);
        } catch (...) { valid=false; }
    }
    void Detach(const Handle(TDocStd_Document)& doc) noexcept {
        if (!OnOwningThread()) return;
        if (!doc.IsNull()) Record(LiveTransactionObservation::Kind::Detached,doc);
        active=nullptr;
    }
};
class LiveObservedApplication final : public TDocStd_Application {
public:
    // The document wrapper owns the state. The application and callbacks never
    // prolong it; normal instances remain inert until explicitly attached.
    bool Observe(std::weak_ptr<LiveTransactionProbeState> state) noexcept {
        if (thread_!=std::this_thread::get_id()) {
            foreignCallback_.store(true,std::memory_order_relaxed);return false;
        }
        state_=std::move(state);return !foreignCallback_.load(std::memory_order_relaxed);
    }
    bool ThreadContractValid() const noexcept {
        return thread_==std::this_thread::get_id()
            && !foreignCallback_.load(std::memory_order_relaxed);
    }
    void OnOpenTransaction(const Handle(TDocStd_Document)& doc) override { Record(LiveTransactionObservation::Kind::Open,doc); }
    void OnCommitTransaction(const Handle(TDocStd_Document)& doc) override { Record(LiveTransactionObservation::Kind::Commit,doc); }
    void OnAbortTransaction(const Handle(TDocStd_Document)& doc) override { Record(LiveTransactionObservation::Kind::Abort,doc); }
private:
    void Record(LiveTransactionObservation::Kind kind,const Handle(TDocStd_Document)& doc) noexcept {
        // Do not even access the shared weak_ptr or foreign OCAF document from
        // another thread. The application exposes the violation atomically.
        if (thread_!=std::this_thread::get_id()) {
            foreignCallback_.store(true,std::memory_order_relaxed);return;
        }
        if (auto state=state_.lock()) state->Record(kind,doc);
    }
    const std::thread::id thread_=std::this_thread::get_id();
    std::atomic_bool foreignCallback_{false};
    std::weak_ptr<LiveTransactionProbeState> state_;
};
} // namespace core3d::debug
