#pragma once
// Production transaction observation. Do not issue public tokens until every
// native adoption/history/selection boundary and recovery fence is integrated.
#include "NativeEditAuthority.hpp"
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>
#include <atomic>
#include <memory>
#include <thread>
#include <utility>

namespace core3d::authority {
class NativeObservedApplication : public TDocStd_Application {
public:
    bool ObserveAuthority(std::weak_ptr<NativeEditAuthority> state) noexcept {
        if(!OnOwner())return false;
        state_=std::move(state);return AuthorityThreadContractValid();
    }
    bool AuthorityThreadContractValid() const noexcept {
        return thread_==std::this_thread::get_id()
            && !foreignCallback_.load(std::memory_order_relaxed);
    }
    void OnOpenTransaction(const Handle(TDocStd_Document)& doc) override { Boundary(doc); }
    void OnCommitTransaction(const Handle(TDocStd_Document)& doc) override { Boundary(doc); }
    void OnAbortTransaction(const Handle(TDocStd_Document)& doc) override { Boundary(doc); }
private:
    bool OnOwner() noexcept {
        if(thread_==std::this_thread::get_id())return true;
        foreignCallback_.store(true,std::memory_order_relaxed);return false;
    }
    void Boundary(const Handle(TDocStd_Document)& doc) noexcept {
        // Avoid touching a weak_ptr or any native document from a foreign
        // callback. The document wrapper must include this class's atomic
        // contract check in every authority capture/admission.
        if(!OnOwner())return;
        if(auto state=state_.lock())state->TransactionBoundary(doc.get());
    }
    const std::thread::id thread_=std::this_thread::get_id();
    std::atomic_bool foreignCallback_{false};
    std::weak_ptr<NativeEditAuthority> state_;
};
} // namespace core3d::authority
