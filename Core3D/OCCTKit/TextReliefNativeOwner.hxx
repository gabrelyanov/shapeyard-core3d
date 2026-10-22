#pragma once

#include "TextReliefDefinition.hxx"
#include "BoundedCurveBuild.hxx"
#include "NativeOpeningContext.hxx"
#include "NativeOpeningDependentReplay.hxx"

#include <functional>
#include <map>
#include <optional>
#include <set>
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

//! Detached, immutable installation material for one identity-preserving
//! host replay. All geometry, canonical bytes and C1 payload allocations are
//! completed before the source command. The original capture remains the
//! expected receipt; it is never recaptured after mutation.
struct PreparedHostPair final {
    OcctPreparedLabelClone root;
    AuthoredRecord record;
    std::vector<std::uint8_t> canonicalRecord;
    std::vector<bounded_curve::DetachedWire> wires;
    std::vector<std::shared_ptr<const bounded_curve::Payload>> curves;
};

inline bool PrepareHostPair(const OcctTextReliefCapture& opening,
    const std::vector<bounded_curve::PersistedValue>& curves,
    std::vector<bounded_curve::DetachedWire> wires,
    AuthoredRecord record, const TopoDS_Shape& result,
    PreparedHostPair& output) noexcept {
    output = {};
    try {
        if (!opening.record.payload || result.IsNull()
            || curves.empty() || curves.size() > MaximumCurves
            || curves.size() != opening.curves.size()
            || wires.size() != curves.size()
            || record.curves.size() != curves.size()) return false;
        const auto& prior = opening.record.payload->value;
        if (prior.revision == UINT64_MAX || record.revision != prior.revision + 1
            || record.owner != prior.owner || record.feature != prior.feature
            || record.rootOutput != prior.rootOutput
            || record.constructionSeed != prior.constructionSeed
            || record.components.size() != prior.components.size()) return false;
        for (std::size_t index = 0; index < record.components.size(); ++index)
            if (record.components[index].key != prior.components[index].key
                || record.components[index].identifier
                    != prior.components[index].identifier) return false;
        PreparedHostPair pair;
        if (!EncodeAuthored(record, pair.canonicalRecord)) return false;
        std::set<UUID> features, entities;
        for (std::size_t index = 0; index < curves.size(); ++index) {
            const auto& next = curves[index];
            const auto& old = opening.curves[index].persisted;
            std::vector<std::uint8_t> definition, owner;
            if (old.ownerState.definitionRevision == UINT64_MAX
                || next.ownerState.definitionRevision
                    != old.ownerState.definitionRevision + 1
                || next.ownerState.owner != old.ownerState.owner
                || next.ownerState.owner.document != record.owner.document
                || next.value.feature != old.value.feature
                || next.value.feature != prior.curves[index].feature
                || next.value.feature != record.curves[index].feature
                || !features.insert(next.value.feature).second
                || !entities.insert(next.ownerState.owner.entity).second
                || !bounded_curve::ValidatePersisted(next, &definition, &owner)
                || !bounded_curve::MatchesPersistedValue(wires[index], next)
                || wires[index].canonicalDefinitionBytes != definition
                || wires[index].canonicalOwnerBytes != owner
                || wires[index].definitionDigest != record.curves[index].digest)
                return false;
            auto payload = std::make_shared<bounded_curve::Payload>();
            payload->persisted = next;
            payload->definitionBytes = std::move(definition);
            payload->ownerBytes = std::move(owner);
            pair.curves.push_back(std::move(payload));
        }
        pair.root.source = opening.ownerReceipt;
        pair.root.detachedShape = result;
        pair.root.representation = OcctGeometryRepresentation::BRep;
        pair.record = std::move(record);
        pair.wires = std::move(wires);
        output = std::move(pair);
        return true;
    } catch (...) { output = {}; return false; }
}

inline bool ReadHostPair(const PreparedHostPair& pair,
    const OcctTextReliefCapture& opening,
    const OcctTextReliefCapture& current) noexcept {
    try {
        if (!current.record.payload || !opening.record.payload
            || current.record.payload->bytes != pair.canonicalRecord
            || !current.record.owner.IsEqual(opening.record.owner)
            || !current.record.label.IsEqual(opening.record.label)
            || !current.record.shape.IsEqual(pair.root.detachedShape)
            || current.curves.size() != pair.curves.size()) return false;
        for (std::size_t index = 0; index < current.curves.size(); ++index) {
            const auto& curve = current.curves[index];
            if (!curve.record.value
                || !curve.record.owner.IsEqual(opening.curves[index].record.owner)
                || !curve.record.label.IsEqual(opening.curves[index].record.label)
                || curve.record.value->definitionBytes != pair.curves[index]->definitionBytes
                || curve.record.value->ownerBytes != pair.curves[index]->ownerBytes
                || !curve.wire.IsEqual(pair.wires[index].wire)) return false;
        }
        return true;
    } catch (...) { return false; }
}

class PreparedHostReplay final : public dependent_replay::PreparedReplay {
public:
    PreparedHostReplay(OcctTextReliefCapture capture, PreparedHostPair pair,
        std::shared_ptr<native_opening::Context> context) noexcept
        : capture_(std::move(capture)), pair_(std::move(pair)),
          context_(std::move(context)) {}
    dependent_replay::Family family() const noexcept override {
        return dependent_replay::Family::TextReliefE5;
    }
    UUID feature() const noexcept override { return pair_.record.feature; }
    UUID resultEntity() const noexcept override { return pair_.record.owner.entity; }
    std::size_t documentBytes() const noexcept override {
        std::size_t bytes = pair_.canonicalRecord.size();
        for (const auto& curve : pair_.curves)
            bytes += curve->definitionBytes.size() + curve->ownerBytes.size();
        return bytes;
    }
    std::size_t memoryBytes() const noexcept override {
        std::size_t bytes = documentBytes() + capture_.record.payload->bytes.size();
        for (const auto& curve : capture_.curves)
            bytes += curve.record.value->definitionBytes.size()
                + curve.record.value->ownerBytes.size();
        for (const auto& wire : pair_.wires)
            bytes += wire.canonicalDefinitionBytes.size() + wire.canonicalOwnerBytes.size();
        return bytes;
    }
    std::size_t topologyNodes() const noexcept override {
        try {
            TopTools_IndexedMapOfShape map;
            TopExp::MapShapes(pair_.root.detachedShape, map);
            for (const auto& wire : pair_.wires) TopExp::MapShapes(wire.wire, map);
            return std::size_t(map.Extent());
        } catch (...) { return SIZE_MAX; }
    }
    TopoDS_Shape prospectiveResultShape() const noexcept override {
        return pair_.root.detachedShape;
    }
    bool appendPublicationMembers(std::vector<UUID>& members) const noexcept override {
        try {
            // Component and C1 feature identities describe the closure, but
            // the viewer resolves free-owner entities. Prepare the complete
            // affected owner set once for both commit and settled recovery.
            // Pair preparation proves these owners survive with the same IDs.
            if (!capture_.record.payload
                || pair_.curves.size() != capture_.curves.size()
                || pair_.record.owner != capture_.record.payload->value.owner)
                return false;
            std::set<UUID> entities(members.begin(), members.end());
            entities.insert(pair_.record.owner.entity);
            for (std::size_t index = 0; index < pair_.curves.size(); ++index) {
                const auto& curve = pair_.curves[index];
                if (!curve || curve->persisted.ownerState.owner
                        != capture_.curves[index].persisted.ownerState.owner)
                    return false;
                entities.insert(curve->persisted.ownerState.owner.entity);
            }
            members.assign(entities.begin(), entities.end());
            return true;
        } catch (...) { return false; }
    }
    const std::shared_ptr<native_opening::Context>& sourceContext() const noexcept {
        return context_;
    }
    bool openingCurrent(OcctDocument& document) const noexcept override {
        OcctTextReliefCapture current;
        return context_ && context_->openingFence().document() == document.Document()
            && document.ReadTextReliefExact(feature(), current)
            && capture_.IsEqual(current);
    }
    bool stage(OcctDocument& document,
        native_opening::CommandLease& lease) const noexcept override {
        OcctTextReliefCapture staged;
        return document.StageTextReliefHostReplay(lease, capture_, pair_, staged);
    }
    bool read(OcctDocument& document) const noexcept override {
        OcctTextReliefCapture current;
        return document.ReadTextReliefExact(feature(), current)
            && ReadHostPair(pair_, capture_, current);
    }
private:
    OcctTextReliefCapture capture_;
    PreparedHostPair pair_;
    std::shared_ptr<native_opening::Context> context_;
};

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
