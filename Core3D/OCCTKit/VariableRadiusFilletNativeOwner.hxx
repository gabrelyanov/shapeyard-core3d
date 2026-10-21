#pragma once

#include "NativeOpeningContext.hxx"
#include "NativeOpeningDependentReplay.hxx"
#include "OcctDocument.h"
#include "ReceiptRecord.hxx"
#include "VariableRadiusFilletBuild.hxx"

#include <BRep_Builder.hxx>
#include <BRepTools.hxx>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <locale>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace core3d::variable_radius_fillet::owner {

using UUID = retained_recipe::UUID;

inline constexpr char PropertyKey[] = "shapeyard.variable-radius-fillet.owner.v1";
inline constexpr std::size_t MaximumOwnerBytes = 8 * 1024 * 1024;

enum class Outcome : std::uint8_t {
    Refused = 0, Cancelled, Committed, OutcomeUnknown, RecoveryRequired
};

enum class FaultPoint : std::uint8_t {
    None = 0, Capture, Prepare, Stage, Readback, Close, Publication
};

enum class EditIntent : std::uint8_t { Ordinary = 0, ReverseDirection };

struct Capture final {
    OcctExactLabelReceipt exact;
    bool present = false;
    UUID feature{};
    std::uint32_t topologyIndex = 0;
    std::vector<std::uint8_t> ownerBytes;
    std::vector<std::uint8_t> lawBytes;
    std::vector<std::uint8_t> baseBytes;
    std::vector<std::uint8_t> currentBytes;
    TopoDS_Shape base;
    TopoDS_Shape current;

    bool valid() const noexcept {
        return !exact.documentData.IsNull() && !current.IsNull()
            && !currentBytes.empty()
            && (!present || (retained_recipe::Nonzero(feature)
                && !ownerBytes.empty() && !lawBytes.empty()
                && !baseBytes.empty() && !base.IsNull()));
    }
};

inline void AppendU32(std::vector<std::uint8_t>& bytes,
                      std::uint32_t value) {
    for (unsigned index = 0; index < 4; ++index)
        bytes.push_back(std::uint8_t(value >> (8 * index)));
}

inline bool ReadU32(const std::vector<std::uint8_t>& bytes,
                    std::size_t& cursor, std::uint32_t& value) noexcept {
    value = 0;
    if (cursor > bytes.size() || bytes.size() - cursor < 4) return false;
    for (unsigned index = 0; index < 4; ++index)
        value |= std::uint32_t(bytes[cursor++]) << (8 * index);
    return true;
}

inline bool ExactShapeBytes(const TopoDS_Shape& shape,
                            std::vector<std::uint8_t>& bytes) noexcept {
    bytes.clear();
    try {
        if (shape.IsNull()) return false;
        std::ostringstream output;
        output.imbue(std::locale::classic());
        // Triangulation is derived presentation state, never document
        // authority: a displayed (meshed) label shape must compare byte-exact
        // to the same geometry before display, so the exact bytes exclude it.
        BRepTools::Write(shape, output, Standard_False, Standard_False,
                         TopTools_FormatVersion_CURRENT);
        if (!output.good()) return false;
        const std::string value = output.str();
        if (value.empty() || value.size() > MaximumOwnerBytes) return false;
        bytes.assign(value.begin(), value.end());
        return true;
    } catch (...) { bytes.clear(); return false; }
}

inline bool ReadShape(const std::vector<std::uint8_t>& bytes,
                      TopoDS_Shape& shape) noexcept {
    shape.Nullify();
    try {
        if (bytes.empty() || bytes.size() > MaximumOwnerBytes) return false;
        std::istringstream input(std::string(bytes.begin(), bytes.end()));
        input.imbue(std::locale::classic());
        BRep_Builder builder;
        BRepTools::Read(shape, input, builder);
        return !shape.IsNull();
    } catch (...) { shape.Nullify(); return false; }
}

inline bool CanonicalLaw(const std::vector<std::uint8_t>& bytes) noexcept {
    try {
        Definition linear;
        std::vector<std::uint8_t> exact;
        if (Decode(bytes, linear)) return Encode(linear, exact) && exact == bytes;
        MultiStationDefinition stations;
        return Decode(bytes, stations) && Encode(stations, exact) && exact == bytes;
    } catch (...) { return false; }
}

inline bool EncodeOwner(const UUID& feature, std::uint32_t topologyIndex,
                        const std::vector<std::uint8_t>& law,
                        const std::vector<std::uint8_t>& base,
                        std::vector<std::uint8_t>& bytes) noexcept {
    bytes.clear();
    try {
        if (!retained_recipe::Nonzero(feature) || !CanonicalLaw(law)
            || base.empty() || law.size() > UINT32_MAX || base.size() > UINT32_MAX
            || law.size() + base.size() > MaximumOwnerBytes - 36) return false;
        bytes.insert(bytes.end(), {'S','Y','V','O',1,0,0,0});
        bytes.insert(bytes.end(), feature.begin(), feature.end());
        AppendU32(bytes, topologyIndex);
        AppendU32(bytes, std::uint32_t(law.size()));
        AppendU32(bytes, std::uint32_t(base.size()));
        bytes.insert(bytes.end(), law.begin(), law.end());
        bytes.insert(bytes.end(), base.begin(), base.end());
        return bytes.size() <= MaximumOwnerBytes;
    } catch (...) { bytes.clear(); return false; }
}

inline bool DecodeOwner(const std::vector<std::uint8_t>& bytes,
                        UUID& feature, std::uint32_t& topologyIndex,
                        std::vector<std::uint8_t>& law,
                        std::vector<std::uint8_t>& base) noexcept {
    feature = {}; topologyIndex = 0; law.clear(); base.clear();
    try {
        if (bytes.size() < 36 || bytes.size() > MaximumOwnerBytes
            || std::memcmp(bytes.data(), "SYVO\1\0\0\0", 8) != 0)
            return false;
        std::size_t cursor = 8;
        std::copy_n(bytes.begin() + cursor, feature.size(), feature.begin());
        cursor += feature.size();
        std::uint32_t lawSize = 0, baseSize = 0;
        if (!ReadU32(bytes, cursor, topologyIndex)
            || !ReadU32(bytes, cursor, lawSize)
            || !ReadU32(bytes, cursor, baseSize)
            || lawSize == 0 || baseSize == 0
            || cursor > bytes.size()
            || std::size_t(lawSize) > bytes.size() - cursor
            || std::size_t(baseSize) != bytes.size() - cursor - lawSize)
            return false;
        law.assign(bytes.begin() + cursor, bytes.begin() + cursor + lawSize);
        base.assign(bytes.begin() + cursor + lawSize, bytes.end());
        TopoDS_Shape decoded;
        return retained_recipe::Nonzero(feature) && CanonicalLaw(law)
            && ReadShape(base, decoded);
    } catch (...) {
        feature = {}; topologyIndex = 0; law.clear(); base.clear(); return false;
    }
}

inline bool NormalizeIdentity(const Capture& before,
                              const std::vector<std::uint8_t>& requested,
                              std::vector<std::uint8_t>& normalized,
                              EditIntent intent = EditIntent::Ordinary) noexcept {
    normalized.clear();
    try {
        if (!before.present) { normalized = requested; return CanonicalLaw(normalized); }
        Definition oldLinear, nextLinear;
        if (Decode(before.lawBytes, oldLinear) && Decode(requested, nextLinear)) {
            if (oldLinear.edges.size() != 1 || nextLinear.edges.size() != 1)
                return false;
            if (intent == EditIntent::ReverseDirection) {
                Definition expected = oldLinear;
                auto& edge = expected.edges.front();
                std::swap(edge.startLocal, edge.endLocal);
                edge.orientation = edge.orientation == Orientation::Forward
                    ? Orientation::Reversed : Orientation::Forward;
                for (std::size_t index = 0; index < expected.stations.size(); ++index) {
                    const auto& physical =
                        oldLinear.stations[expected.stations.size() - 1 - index];
                    expected.stations[index] = physical;
                    expected.stations[index].parameter = 1.0 - physical.parameter;
                    if (nextLinear.stations[index].parameter
                            != expected.stations[index].parameter
                        || nextLinear.stations[index].radiusLocal
                            != expected.stations[index].radiusLocal)
                        return false;
                }
                if (!(nextLinear.edges.front() == edge)) return false;
                return Encode(expected, normalized);
            }
            if (!(nextLinear.edges.front() == oldLinear.edges.front())) return false;
            nextLinear.edges = oldLinear.edges;
            nextLinear.stations[0].identifier = oldLinear.stations[0].identifier;
            nextLinear.stations[1].identifier = oldLinear.stations[1].identifier;
            return Encode(nextLinear, normalized);
        }
        MultiStationDefinition oldStations, nextStations;
        if (!Decode(before.lawBytes, oldStations)
            || !Decode(requested, nextStations)
            || oldStations.edges.size() != 1 || nextStations.edges.size() != 1
            || oldStations.stations.size() != nextStations.stations.size()) return false;
        if (intent == EditIntent::ReverseDirection) {
            MultiStationDefinition expected = oldStations;
            auto& edge = expected.edges.front();
            std::swap(edge.startLocal, edge.endLocal);
            edge.orientation = edge.orientation == Orientation::Forward
                ? Orientation::Reversed : Orientation::Forward;
            for (std::size_t index = 0; index < expected.stations.size(); ++index) {
                const auto& physical =
                    oldStations.stations[expected.stations.size() - 1 - index];
                expected.stations[index] = physical;
                expected.stations[index].parameter = 1.0 - physical.parameter;
                if (nextStations.stations[index].parameter
                        != expected.stations[index].parameter
                    || nextStations.stations[index].radiusLocal
                        != expected.stations[index].radiusLocal)
                    return false;
            }
            if (!(nextStations.edges.front() == edge)) return false;
            return Encode(expected, normalized);
        }
        if (!(nextStations.edges.front() == oldStations.edges.front())) return false;
        bool requestedUsesRetainedIdentities = true;
        std::set<UUID> retainedIdentities;
        for (const auto& station : oldStations.stations)
            retainedIdentities.insert(station.identifier);
        for (const auto& station : nextStations.stations)
            requestedUsesRetainedIdentities = requestedUsesRetainedIdentities
                && retainedIdentities.count(station.identifier) == 1;
        if (requestedUsesRetainedIdentities)
            for (std::size_t index = 0; index < nextStations.stations.size(); ++index)
                if (nextStations.stations[index].identifier
                    != oldStations.stations[index].identifier) return false;
        nextStations.edges = oldStations.edges;
        for (std::size_t index = 0; index < nextStations.stations.size(); ++index)
            nextStations.stations[index].identifier = oldStations.stations[index].identifier;
        return Encode(nextStations, normalized);
    } catch (...) { normalized.clear(); return false; }
}

struct Prepared final {
    Capture before;
    Capture expected;
    TopoDS_Shape candidate;
    std::vector<std::uint8_t> ownerBytes;
    std::vector<std::uint8_t> lawBytes;
    std::shared_ptr<const dependent_replay::Plan> replay;
    native_opening::CommittedEditPublication publication;
    std::shared_ptr<native_opening::Context> context;
    std::uint32_t viewportWidth = 0, viewportHeight = 0;
    FaultPoint fault = FaultPoint::None;
};

struct Result final {
    Outcome outcome = Outcome::Refused;
    dependent_replay::Refusal replayRefusal = dependent_replay::Refusal::None;
    int measuredHistoryDelta = 0;
    bool commandClosed = true;
    bool blocksOtherWork = false;
};

inline bool SameCapture(const Capture& first, const Capture& second) noexcept {
    try {
        return first.present == second.present
            && first.feature == second.feature
            && first.topologyIndex == second.topologyIndex
            && first.ownerBytes == second.ownerBytes
            && first.lawBytes == second.lawBytes
            && first.baseBytes == second.baseBytes
            && first.currentBytes == second.currentBytes
            && first.exact.IsEqual(second.exact);
    } catch (...) { return false; }
}

inline bool MintFeature(UUID& feature) noexcept {
    feature = {};
    try {
        for (unsigned attempt = 0; attempt < 16; ++attempt) {
            if (receipt::ParseUUID(OcctDocument::NewProfileIdentifier(), feature)
                && retained_recipe::Nonzero(feature)) return true;
        }
    } catch (...) {}
    feature = {}; return false;
}

inline bool Prepare(OcctDocument& document, const TDF_Label& label,
                    const std::shared_ptr<native_opening::Context>& context,
                    std::uint32_t viewportWidth, std::uint32_t viewportHeight,
                    std::uint32_t topologyIndex,
                    const std::vector<std::uint8_t>& candidateShapeBytes,
                    const std::vector<std::uint8_t>& requestedLaw,
                    Prepared& prepared,
                    FaultPoint fault = FaultPoint::None,
                    EditIntent intent = EditIntent::Ordinary) noexcept {
    prepared = {};
    try {
        if (!context || fault == FaultPoint::Capture
            || !document.CaptureVariableRadiusFilletOwner(label, prepared.before)
            || !prepared.before.valid()
            || !ReadShape(candidateShapeBytes, prepared.candidate)
            || fault == FaultPoint::Prepare
            || !NormalizeIdentity(prepared.before, requestedLaw, prepared.lawBytes, intent))
            return false;
        UUID feature = prepared.before.feature;
        std::vector<std::uint8_t> base = prepared.before.baseBytes;
        if (!prepared.before.present) {
            if (!MintFeature(feature)) return false;
            base = prepared.before.currentBytes;
        } else {
            topologyIndex = prepared.before.topologyIndex;
        }
        if (!EncodeOwner(feature, topologyIndex, prepared.lawBytes,
                         base, prepared.ownerBytes)) return false;

        dependent_replay::Candidate replayCandidate;
        replayCandidate.targetEntityIdentifier =
            prepared.before.exact.visibility.object.object.entityIdentifier;
        replayCandidate.resultShape = prepared.candidate;
        dependent_replay::ProductionPreparer replayPreparer(
            std::move(replayCandidate), context);
        const dependent_replay::Refusal replay = document.PrepareDependentReplayPlan(
            prepared.before.exact, dependent_replay::Mutation::Replace,
            dependent_replay::Limits{}, replayPreparer, prepared.candidate,
            prepared.replay);
        if (replay != dependent_replay::Refusal::None || !prepared.replay) {
#if DEBUG
            std::fprintf(stderr, "B3_OWNER_PREP replay=%d plan=%d\n",
                int(replay), prepared.replay ? 1 : 0);
#endif
            return false;
        }

        std::set<std::string> entities;
        entities.insert(prepared.before.exact.visibility.object.object.entityIdentifier);
        for (const auto& dependency : prepared.replay->dependencies()) {
            for (const auto& member : dependency.memberIdentities)
                entities.insert(retained_solid::UUIDText(member));
            entities.insert(retained_solid::UUIDText(dependency.resultEntity));
        }
        for (const auto& entity : entities) if (!entity.empty())
            prepared.publication.replaced.push_back({entity,
                entity == prepared.before.exact.visibility.object.object.entityIdentifier
                    ? prepared.before.current : TopoDS_Shape()});
        prepared.context = context;
        prepared.viewportWidth = viewportWidth;
        prepared.viewportHeight = viewportHeight;
        prepared.fault = fault;
        return !prepared.publication.replaced.empty();
    } catch (...) { prepared = {}; return false; }
}

class SourceStage final : public dependent_replay::SourceMutation {
public:
    explicit SourceStage(Prepared& prepared) noexcept : prepared_(prepared) {}
    bool stage(OcctDocument& owner,
               native_opening::CommandLease& lease) noexcept override {
        if (prepared_.fault == FaultPoint::Stage) return false;
        return owner.StageVariableRadiusFilletOwner(lease, prepared_.before,
            prepared_.candidate, prepared_.ownerBytes, prepared_.expected);
    }
private:
    Prepared& prepared_;
};

inline Result Apply(OcctDocument& document, Prepared& prepared,
                    const std::atomic_bool& cancelled) noexcept {
    Result result;
    try {
        if (!prepared.context || !prepared.replay || cancelled.load()) {
            result.outcome = cancelled.load() ? Outcome::Cancelled : Outcome::Refused;
            return result;
        }
        const Handle(TDocStd_Document) native = document.Document();
        const int undoBefore = native.IsNull() ? -1 : native->GetAvailableUndos();
        auto lease = prepared.context->beginCommandLease(
            prepared.context->openingFence(), prepared.viewportWidth,
            prepared.viewportHeight);
        if (!lease || !lease->ownsOpenCommand()) return result;
        SourceStage source(prepared);
        result.replayRefusal = document.StageDependentReplayPlan(
            *lease, *prepared.replay, source);
        Capture inside;
        const bool staged = result.replayRefusal == dependent_replay::Refusal::None
            && prepared.expected.valid()
            && prepared.fault != FaultPoint::Readback
            && document.ReadVariableRadiusFilletOwner(prepared.expected, inside)
            && document.ReadDependentReplayPlan(*prepared.replay)
                == dependent_replay::Refusal::None;
        if (!staged || cancelled.load()) {
            const bool stopped = cancelled.load();
            const bool aborted = lease->abort();
            Capture prior;
            const bool proved = aborted
                && document.ReadVariableRadiusFilletOwner(prepared.before, prior)
                && document.SettleVariableRadiusFilletHistoryOnPrior();
            result.outcome = proved
                ? (stopped ? Outcome::Cancelled : Outcome::Refused)
                : Outcome::RecoveryRequired;
            result.commandClosed = aborted;
            result.blocksOtherWork = !proved;
            return result;
        }
#if DEBUG
        if (prepared.fault == FaultPoint::Close)
            prepared.context->debugReportNextCloseUnproven();
#endif
        if (!lease->commit()) {
            prepared.context->retainUnprovenEdit(prepared.publication);
            result.outcome = Outcome::OutcomeUnknown;
            result.commandClosed = false;
            result.blocksOtherWork = true;
            return result;
        }
        Capture after;
        const bool exact = document.ReadVariableRadiusFilletOwner(
                prepared.expected, after)
            && document.ReadDependentReplayPlan(*prepared.replay)
                == dependent_replay::Refusal::None
            && document.FinalizeVariableRadiusFilletHistory();
        if (!exact) {
            prepared.context->retainUnprovenEdit(prepared.publication);
            result.outcome = Outcome::RecoveryRequired;
            result.blocksOtherWork = true;
            return result;
        }
        result.measuredHistoryDelta = native->GetAvailableUndos() - undoBefore;
#if DEBUG
        const bool published = prepared.fault != FaultPoint::Publication
            && prepared.context->publishCommittedEdit(prepared.publication);
#else
        const bool published = prepared.context->publishCommittedEdit(
            prepared.publication);
#endif
        if (!published) {
            prepared.context->retainUnprovenEdit(prepared.publication);
            result.outcome = Outcome::OutcomeUnknown;
            result.blocksOtherWork = true;
            return result;
        }
        result.outcome = result.measuredHistoryDelta == 1
            ? Outcome::Committed : Outcome::RecoveryRequired;
        result.blocksOtherWork = result.outcome != Outcome::Committed;
        return result;
    } catch (...) {
        prepared.context->retainUnprovenEdit(prepared.publication);
        result.outcome = Outcome::RecoveryRequired;
        result.commandClosed = false;
        result.blocksOtherWork = true;
        return result;
    }
}

inline Outcome Reconcile(OcctDocument& document, Prepared& prepared) noexcept {
    try {
        Capture actual;
        if (!document.CaptureVariableRadiusFilletOwner(
                prepared.before.exact.visibility.object.object.label, actual))
            return Outcome::RecoveryRequired;
        const Outcome observed = SameCapture(actual, prepared.before)
            ? Outcome::Refused
            : SameCapture(actual, prepared.expected)
                ? Outcome::Committed : Outcome::RecoveryRequired;
        const bool historySettled = observed == Outcome::Committed
            ? document.FinalizeVariableRadiusFilletHistory()
            : observed == Outcome::Refused
                && document.SettleVariableRadiusFilletHistoryOnPrior();
        if (observed == Outcome::RecoveryRequired || !historySettled
            || !prepared.context->reconcileRecovery(true))
            return Outcome::RecoveryRequired;
        return observed;
    } catch (...) { return Outcome::RecoveryRequired; }
}

} // namespace core3d::variable_radius_fillet::owner
