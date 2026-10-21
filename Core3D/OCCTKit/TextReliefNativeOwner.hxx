#pragma once

#include "TextReliefDefinition.hxx"

#include <functional>
#include <map>
#include <optional>
#include <utility>

namespace core3d::text_relief::owner {

enum class Mutation : std::uint8_t { Create = 1, Replace = 2, Remove = 3 };
enum class Fault : std::uint8_t {
    None = 0, Preparation, CurveStage, ComponentStage, RecordStage,
    Readback, CloseUnknown, Publication
};
enum class Outcome : std::uint8_t {
    Applied = 0, Refused, Preserved, UnknownRecovery, Blocked
};

struct Snapshot final {
    AuthoredRecord record;
    std::vector<std::vector<std::uint8_t>> canonicalCurves;
    std::vector<std::vector<std::uint8_t>> componentGeometry;
    std::vector<std::uint8_t> canonicalRecord;
};

inline bool Complete(Snapshot& value) noexcept {
    try {
        std::vector<std::uint8_t> exact;
        if (!EncodeAuthored(value.record, exact)
            || value.canonicalCurves.size() != value.record.curves.size()
            || value.componentGeometry.size() != value.record.components.size())
            return false;
        for (std::size_t index = 0; index < value.canonicalCurves.size(); ++index) {
            Digest digest{};
            if (!bounded_curve::Hash(value.canonicalCurves[index],
                                     bounded_curve::MaximumDefinitionBytes, digest)
                || digest != value.record.curves[index].digest) return false;
        }
        for (const auto& geometry : value.componentGeometry)
            if (geometry.empty() || geometry.size() > 16 * 1024 * 1024) return false;
        if (!value.canonicalRecord.empty() && value.canonicalRecord != exact) return false;
        value.canonicalRecord = std::move(exact); return true;
    } catch (...) { return false; }
}

inline bool SameIdentity(const Snapshot& left, const Snapshot& right) noexcept {
    return left.record.owner == right.record.owner
        && left.record.feature == right.record.feature
        && left.record.constructionSeed == right.record.constructionSeed
        && left.record.rootOutput == right.record.rootOutput;
}

inline bool SurvivingComponentsStable(const Snapshot& prior,
                                      const Snapshot& next) noexcept {
    try {
        std::map<std::uint32_t, UUID> oldByKey;
        for (const auto& component : prior.record.components)
            oldByKey.emplace(component.key, component.identifier);
        for (const auto& component : next.record.components) {
            const auto found = oldByKey.find(component.key);
            if (found != oldByKey.end() && found->second != component.identifier)
                return false;
        }
        return true;
    } catch (...) { return false; }
}

//! Production dependent-dispatch admission shared with OcctDocument. S2 can
//! identify and capture the exact E5 descendant; S3-R2 supplies the detached
//! geometry reattachment before replacement replay becomes admissible.
inline bool ValidateReplayDependency(const std::vector<std::uint8_t>& bytes,
                                     const UUID& feature,
                                     const UUID& resultEntity) noexcept {
    AuthoredRecord value;
    return DecodeAuthored(bytes, value) && value.feature == feature
        && value.owner.entity == resultEntity;
}

struct Plan final {
    Mutation mutation = Mutation::Create;
    std::optional<Snapshot> prior;
    std::optional<Snapshot> next;
    std::uint64_t issuer = 0;
};

class Repository final {
public:
    using Stage = std::function<bool(const Plan&, Fault)>;

    bool prepare(Mutation mutation, std::optional<Snapshot> next,
                 Plan& output) const noexcept {
        output = {};
        try {
            if (recovery_ || issuer_ == UINT64_MAX) return false;
            if ((mutation == Mutation::Create && (current_ || !next))
                || (mutation == Mutation::Replace && (!current_ || !next))
                || (mutation == Mutation::Remove && (!current_ || next))) return false;
            if (next && !Complete(*next)) return false;
            if (mutation == Mutation::Replace
                && (!SameIdentity(*current_, *next)
                    || !SurvivingComponentsStable(*current_, *next)
                    || next->record.revision != current_->record.revision + 1))
                return false;
            Plan plan; plan.mutation = mutation; plan.prior = current_;
            plan.next = std::move(next); plan.issuer = issuer_ + 1;
            output = std::move(plan); return true;
        } catch (...) { output = {}; return false; }
    }

    Outcome execute(Plan plan, Fault fault = Fault::None,
                    const Stage& stage = {}) noexcept {
        try {
            if (recovery_) return Outcome::Blocked;
            if (plan.issuer != issuer_ + 1 || plan.prior.has_value() != current_.has_value()
                || (plan.prior && current_
                    && plan.prior->canonicalRecord != current_->canonicalRecord))
                return Outcome::Refused;
            if (fault == Fault::Preparation) return Outcome::Preserved;
            const Fault phases[] = {Fault::CurveStage, Fault::ComponentStage,
                                    Fault::RecordStage, Fault::Readback};
            for (Fault phase : phases) {
                if (fault == phase || (stage && !stage(plan, phase)))
                    return Outcome::Preserved;
            }
            if (fault == Fault::CloseUnknown) {
                recovery_ = Recovery{plan, true, false}; issuer_ = plan.issuer;
                current_ = plan.next; return Outcome::UnknownRecovery;
            }
            current_ = plan.next; issuer_ = plan.issuer; ++historyDepth_;
            undo_.push_back(plan.prior); redo_.clear();
            if (fault == Fault::Publication) {
                recovery_ = Recovery{plan, true, false};
                return Outcome::UnknownRecovery;
            }
            return Outcome::Applied;
        } catch (...) { return Outcome::Preserved; }
    }

    Outcome reconcile(bool commandCommitted) noexcept {
        try {
            if (!recovery_ || recovery_->reconciled) return Outcome::Refused;
            recovery_->reconciled = true;
            current_ = commandCommitted ? recovery_->plan.next : recovery_->plan.prior;
            if (commandCommitted && !recovery_->counted) {
                undo_.push_back(recovery_->plan.prior); redo_.clear();
                ++historyDepth_; recovery_->counted = true;
            }
            recovery_.reset(); return Outcome::Applied;
        } catch (...) { return Outcome::UnknownRecovery; }
    }

    bool undo() noexcept {
        if (recovery_ || undo_.empty()) return false;
        redo_.push_back(current_); current_ = undo_.back(); undo_.pop_back();
        if (historyDepth_) --historyDepth_; return true;
    }
    bool redo() noexcept {
        if (recovery_ || redo_.empty()) return false;
        undo_.push_back(current_); current_ = redo_.back(); redo_.pop_back();
        ++historyDepth_; return true;
    }
    const std::optional<Snapshot>& current() const noexcept { return current_; }
    std::uint64_t historyDepth() const noexcept { return historyDepth_; }
    bool recoveryPending() const noexcept { return recovery_.has_value(); }
    std::uint64_t issuer() const noexcept { return issuer_; }

private:
    struct Recovery final { Plan plan; bool actualCommitted, counted, reconciled = false; };
    std::optional<Snapshot> current_;
    std::vector<std::optional<Snapshot>> undo_, redo_;
    std::optional<Recovery> recovery_;
    std::uint64_t issuer_ = 0, historyDepth_ = 0;
};

} // namespace core3d::text_relief::owner
