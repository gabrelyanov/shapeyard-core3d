#pragma once

#include "AttachmentRigMetadataPersistence.hxx"
#include "NativeOpeningContext.hxx"
#include "OcctDocument.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace core3d::attachment_rig::owner {

enum class Outcome : std::uint8_t {
    Refused = 0,
    Cancelled,
    Committed,
    Created,
    Removed,
    NoChange,
    OutcomeUnknown,
    RecoveryRequired,
};

enum class FaultPoint : std::uint8_t {
    None = 0,
    Prepare,
    BeforeStage,
    Stage,
    Readback,
    BeforeCommit,
    Close,
    Publication,
};

struct Result final {
    Outcome outcome = Outcome::Refused;
    std::vector<std::uint8_t> readback;
    int measuredHistoryDelta = 0;
    bool commandClosed = true;
    bool blocksOtherWork = false;
};

using Reobserve = std::function<bool()>;
using PrepareCandidate = std::function<bool(
    const StoredRecord&, std::vector<std::uint8_t>&)>;

inline bool SameStoredRecord(const StoredRecord& first,
                             const StoredRecord& second) noexcept {
    return first.state == second.state && first.bytes == second.bytes;
}

inline bool SameIdentityDomains(const std::vector<std::uint8_t>& before,
                                const std::vector<std::uint8_t>& after) noexcept {
    try {
        Metadata oldValue, newValue;
        std::string refusal;
        if (!DecodeABR1(before, oldValue, refusal)
            || !DecodeABR1(after, newValue, refusal)) return false;
        std::set<std::string> oldAttachments, newAttachments;
        std::set<std::string> oldJoints, newJoints;
        std::multiset<std::string> oldOwners, newOwners;
        for (const auto& value : oldValue.attachments) {
            oldAttachments.insert(value.attachmentID);
            oldOwners.insert(value.rigid ? value.rigid->owner : value.surface->owner);
        }
        for (const auto& value : newValue.attachments) {
            newAttachments.insert(value.attachmentID);
            newOwners.insert(value.rigid ? value.rigid->owner : value.surface->owner);
        }
        for (const auto& value : oldValue.joints) oldJoints.insert(value.jointID);
        for (const auto& value : newValue.joints) newJoints.insert(value.jointID);
        return oldAttachments == newAttachments && oldJoints == newJoints
            && oldOwners == newOwners;
    } catch (...) { return false; }
}

// Creation is append-only. Existing identities and their owner multiplicity
// must remain represented in the candidate; Validate/EncodeABR1 independently
// reject collisions in the appended records.
inline bool PreservesIdentityDomains(const std::vector<std::uint8_t>& before,
                                     const std::vector<std::uint8_t>& after) noexcept {
    try {
        Metadata oldValue, newValue;
        std::string refusal;
        if (!DecodeABR1(before, oldValue, refusal)
            || !DecodeABR1(after, newValue, refusal)) return false;
        std::set<std::string> oldAttachments, newAttachments;
        std::set<std::string> oldJoints, newJoints;
        std::multiset<std::string> oldOwners, newOwners;
        for (const auto& value : oldValue.attachments) {
            oldAttachments.insert(value.attachmentID);
            oldOwners.insert(value.rigid ? value.rigid->owner : value.surface->owner);
        }
        for (const auto& value : newValue.attachments) {
            newAttachments.insert(value.attachmentID);
            newOwners.insert(value.rigid ? value.rigid->owner : value.surface->owner);
        }
        for (const auto& value : oldValue.joints) oldJoints.insert(value.jointID);
        for (const auto& value : newValue.joints) newJoints.insert(value.jointID);
        return std::includes(newAttachments.begin(), newAttachments.end(),
                             oldAttachments.begin(), oldAttachments.end())
            && std::includes(newJoints.begin(), newJoints.end(),
                             oldJoints.begin(), oldJoints.end())
            && std::includes(newOwners.begin(), newOwners.end(),
                             oldOwners.begin(), oldOwners.end());
    } catch (...) { return false; }
}

inline bool CanonicalStoredCandidate(const std::vector<std::uint8_t>& bytes) noexcept {
    try {
        if (bytes.empty() || bytes.size() > kMaxStoredABR1Bytes) return false;
        Metadata value;
        std::string refusal;
        std::vector<std::uint8_t> canonical;
        return DecodeABR1(bytes, value, refusal)
            && EncodeABR1(value, canonical, refusal) && canonical == bytes;
    } catch (...) { return false; }
}

inline Result Apply(OcctDocument& document,
                    const std::shared_ptr<native_opening::Context>& context,
                    const std::vector<std::uint8_t>& candidate,
                    bool remove,
                    const Reobserve& reobserve,
                    const native_opening::CommittedEditPublication& publication,
                    const std::atomic_bool& cancelled,
                    FaultPoint fault = FaultPoint::None,
                    const PrepareCandidate& prepareCandidate = {}) noexcept {
    Result result;
    try {
        StoredRecord before;
        if (!context || !reobserve || cancelled.load()
            || fault == FaultPoint::Prepare || !reobserve()
            || !document.ReadAttachmentRigMetadata(before) || !before.readable()
            || (remove && before.state != StoredReadState::Present)
            || (!remove && !prepareCandidate
                && !CanonicalStoredCandidate(candidate))) {
            result.outcome = cancelled.load() || fault == FaultPoint::Prepare
                ? Outcome::Cancelled : Outcome::Refused;
            return result;
        }
        if (!remove && !prepareCandidate
            && before.state == StoredReadState::Present) {
            if (before.bytes == candidate) {
                result.outcome = Outcome::NoChange;
                result.readback = before.bytes;
                return result;
            }
            if (!SameIdentityDomains(before.bytes, candidate)) return result;
        }

        const Handle(TDocStd_Document) native = document.Document();
        const int undoBefore = native.IsNull() ? -1 : native->GetAvailableUndos();
        auto lease = context->beginCommandLease(context->openingFence(), 64, 64);
        if (!lease || !lease->ownsOpenCommand()) return result;
        if (cancelled.load() || fault == FaultPoint::BeforeStage || !reobserve()) {
            const bool stopped = cancelled.load()
                || fault == FaultPoint::BeforeStage;
            const bool aborted = lease->abort();
            result.outcome = aborted
                ? (stopped ? Outcome::Cancelled : Outcome::Refused)
                : Outcome::RecoveryRequired;
            result.commandClosed = aborted;
            result.blocksOtherWork = !aborted;
            return result;
        }

        std::vector<std::uint8_t> preparedCandidate = candidate;
        if (!remove && prepareCandidate
            && (!prepareCandidate(before, preparedCandidate)
                || !CanonicalStoredCandidate(preparedCandidate)
                || (before.state == StoredReadState::Present
                    && (before.bytes == preparedCandidate
                        || !PreservesIdentityDomains(
                            before.bytes, preparedCandidate))))) {
            const bool aborted = lease->abort();
            StoredRecord restored;
            const bool exact = aborted
                && document.ReadAttachmentRigMetadata(restored)
                && SameStoredRecord(before, restored);
            result.outcome = exact ? Outcome::Refused : Outcome::RecoveryRequired;
            result.commandClosed = aborted;
            result.blocksOtherWork = !exact;
            return result;
        }

        StoredRecord staged;
        const bool stagedOK = fault != FaultPoint::Stage
            && document.StageAttachmentRigMetadata(
                *lease, before, preparedCandidate, remove, staged);
        StoredRecord independent;
        const bool readbackOK = stagedOK && fault != FaultPoint::Readback
            && document.ReadAttachmentRigMetadata(independent)
            && SameStoredRecord(staged, independent)
            && (remove || (independent.state == StoredReadState::Present
                && independent.bytes == preparedCandidate));
        if (!readbackOK || cancelled.load() || fault == FaultPoint::BeforeCommit
            || !reobserve()) {
            const bool stopped = cancelled.load()
                || fault == FaultPoint::BeforeCommit;
            const bool aborted = lease->abort();
            StoredRecord restored;
            const bool exact = aborted
                && document.ReadAttachmentRigMetadata(restored)
                && SameStoredRecord(before, restored);
            result.outcome = exact
                ? (stopped ? Outcome::Cancelled : Outcome::Refused)
                : Outcome::RecoveryRequired;
            result.commandClosed = aborted;
            result.blocksOtherWork = !exact;
            return result;
        }
#if DEBUG
        if (fault == FaultPoint::Close) context->debugReportNextCloseUnproven();
#endif
        if (!lease->commit()) {
            context->retainUnprovenEdit(publication);
            result.outcome = Outcome::OutcomeUnknown;
            result.commandClosed = false;
            result.blocksOtherWork = true;
            return result;
        }
        StoredRecord committed;
        if (!document.ReadAttachmentRigMetadata(committed)
            || !SameStoredRecord(staged, committed)) {
            context->retainUnprovenEdit(publication);
            result.outcome = Outcome::RecoveryRequired;
            result.blocksOtherWork = true;
            return result;
        }
        result.measuredHistoryDelta = native->GetAvailableUndos() - undoBefore;
        if (result.measuredHistoryDelta != 1
            || !document.FinalizeAttachmentRigHistory()) {
            context->retainUnprovenEdit(publication);
            result.outcome = Outcome::RecoveryRequired;
            result.blocksOtherWork = true;
            return result;
        }
#if DEBUG
        const bool published = fault != FaultPoint::Publication
            && context->publishCommittedEdit(publication);
#else
        const bool published = context->publishCommittedEdit(publication);
#endif
        if (!published) {
            context->retainUnprovenEdit(publication);
            result.outcome = Outcome::OutcomeUnknown;
            result.blocksOtherWork = true;
            return result;
        }
        result.readback = committed.bytes;
        result.outcome = remove ? Outcome::Removed
            : before.state == StoredReadState::Absent
                ? Outcome::Created : Outcome::Committed;
        return result;
    } catch (...) {
        if (context) context->retainUnprovenEdit(publication);
        result.outcome = Outcome::RecoveryRequired;
        result.commandClosed = false;
        result.blocksOtherWork = true;
        return result;
    }
}

} // namespace core3d::attachment_rig::owner
