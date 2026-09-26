#pragma once
#include "BoundedCurveEdit.hxx"
#include <map>
#include <string>

namespace core3d::bounded_curve::debug {
inline UUID ID(std::uint8_t value) {
    UUID result{};
    result.front() = value;
    return result;
}
inline Digest Hash(std::uint8_t value) {
    Digest result{};
    result.front() = value;
    return result;
}

inline RetainedState Fixture() {
    RetainedState state;
    state.authority.owner = {ID(1), ID(2), ID(3)};
    state.authority.feature = ID(4);
    state.authority.definitionRevision = 7;
    state.authority.frame = ID(5);
    state.authority.frameRevision = 2;
    state.authority.recipeDigest = Hash(6);
    state.nextLocalID = 4;
    state.definition.domain = Domain::Sketch2D;
    state.definition.frame.identifier = ID(5);
    state.definition.frame.revision = 2;
    state.definition.degree = 2;
    state.definition.controlPoints = {
        {ID(10), {{0, 0, 0}}}, {ID(11), {{2, 3, 0}}}, {ID(12), {{5, 0, 0}}}
    };
    state.definition.knots = {{0, 3}, {1, 3}};
    return state;
}

struct Owner {
    RetainedState state;
    RetainedState before;
    std::vector<RetainedState> history;
    std::size_t cursor = 0;
    bool open = false;
    bool foreignOpen = false;
    bool failStage = false;
    bool failReplace = false;
    bool failVerify = false;
    CloseResult closeResult = CloseResult::Committed;
    std::uint64_t token = 0;
    unsigned begins = 0, commits = 0, aborts = 0, unrelated = 0;

    explicit Owner(const RetainedState& initial) : state(initial), history{initial} {}
    bool closed() const noexcept { return !open && !foreignOpen; }
    bool read(RetainedState& output) const noexcept { output = state; return true; }
    std::uint64_t begin() noexcept {
        if (!closed()) return 0;
        before = state; open = true; token = 0xC1B; ++begins; return token;
    }
    bool owns(std::uint64_t value) const noexcept { return open && value == token; }
    bool stage(std::uint64_t value, const RetainedState& candidate) noexcept {
        if (!owns(value) || failStage) return false;
        state = candidate; return true;
    }
    bool replace(std::uint64_t value, const RetainedState&) noexcept {
        return owns(value) && !failReplace;
    }
    bool verify(std::uint64_t value, const RetainedState& candidate) noexcept {
        return owns(value) && !failVerify && SameState(state, candidate);
    }
    CloseResult commit(std::uint64_t value) noexcept {
        if (!owns(value)) return CloseResult::StillOpen;
        ++commits;
        if (closeResult == CloseResult::StillOpen) return closeResult;
        open = false; token = 0;
        if (closeResult == CloseResult::Committed) {
            history.resize(cursor + 1);
            history.push_back(state);
            ++cursor;
        }
        return closeResult;
    }
    bool abort(std::uint64_t value) noexcept {
        if (!owns(value)) return false;
        state = before; open = false; token = 0; ++aborts; return true;
    }
    bool undo() noexcept {
        if (!closed() || cursor == 0) return false;
        state = history[--cursor]; return true;
    }
    bool redo() noexcept {
        if (!closed() || cursor + 1 >= history.size()) return false;
        state = history[++cursor]; return true;
    }
    void unrelatedLaterStep() noexcept { ++unrelated; }
};

inline EditProposal Move(const RetainedState& state, EditKind kind, UUID target,
                         std::array<double, 3> local, std::uint8_t digest) {
    EditProposal proposal;
    proposal.kind = kind;
    proposal.expected = CurrentAuthority(state);
    proposal.controlPoint = target;
    proposal.replacementLocal = local;
    proposal.replacementRecipeDigest = Hash(digest);
    return proposal;
}

inline std::map<std::string, bool> Probe() {
    std::map<std::string, bool> checks;
    const auto detached = [](const RetainedState& candidate) {
        return ValidState(candidate);
    };
    const RetainedState original = Fixture();
    checks["fixture-valid"] = ValidState(original);

    Owner owner(original);
    const EditProposal pole = Move(owner.state, EditKind::MovePole, ID(11), {{2, 4, 0}}, 20);
    checks["pole-commit"] = CommitEdit(owner, pole, detached) == CommitResult::Committed;
    checks["pole-targeted-by-uuid"] = owner.state.definition.controlPoints[1].identifier == ID(11)
        && owner.state.definition.controlPoints[1].local == std::array<double, 3>{{2, 4, 0}};
    checks["other-uuid-values-survive"] = owner.state.definition.controlPoints[0].identifier == ID(10)
        && owner.state.definition.controlPoints[0].local == original.definition.controlPoints[0].local
        && owner.state.definition.controlPoints[2].identifier == ID(12)
        && owner.state.definition.controlPoints[2].local == original.definition.controlPoints[2].local;
    checks["one-owned-command"] = owner.begins == 1 && owner.commits == 1
        && owner.aborts == 0 && owner.history.size() == 2;
    const RetainedState moved = owner.state;
    checks["undo-exact"] = owner.undo() && SameState(owner.state, original);
    checks["redo-exact"] = owner.redo() && SameState(owner.state, moved);

    const EditProposal handle = Move(owner.state, EditKind::MoveOutgoingHandle,
        ID(12), {{6, 1, 0}}, 21);
    checks["handle-commit"] = CommitEdit(owner, handle, detached) == CommitResult::Committed;
    checks["handle-resolves-pole-uuid"] = owner.state.definition.controlPoints[2].identifier == ID(12)
        && owner.state.definition.controlPoints[2].local == std::array<double, 3>{{6, 1, 0}};
    checks["frame-byte-exact"] = SameFrame(owner.state.definition.frame, original.definition.frame)
        && owner.state.authority.frame == original.authority.frame
        && owner.state.authority.frameRevision == original.authority.frameRevision;
    const RetainedState afterHuman = owner.state;
    owner.unrelatedLaterStep();
    checks["unrelated-later-step-keeps-human-value"] = owner.unrelated == 1
        && SameState(owner.state, afterHuman);

    RetainedState retired = original;
    retired.tombstones.push_back(ID(42));
    // The tombstoned ID(42) was issued, so the issuance fence must exceed it:
    // ValidState requires live (3) + tombstones (1) < nextLocalID.
    retired.nextLocalID = 43;
    Owner tombstoneOwner(retired);
    const EditProposal deleted = Move(retired, EditKind::MovePole, ID(42), {{1, 1, 0}}, 22);
    const PreparedEdit deletedPrepared = PrepareEdit(retired, deleted);
    const RetainedState deletedBefore = tombstoneOwner.state;
    checks["tombstone-refused"] = deletedPrepared.refusal == EditRefusal::ControlPointDeleted
        && CommitEdit(tombstoneOwner, deleted, detached) == CommitResult::Refused;
    checks["tombstone-zero-mutation"] = SameState(tombstoneOwner.state, deletedBefore)
        && tombstoneOwner.begins == 0 && tombstoneOwner.history.size() == 1;

    Owner staleOwner(original);
    EditProposal stale = Move(original, EditKind::MoveIncomingHandle, ID(10), {{1, 0, 0}}, 23);
    --stale.expected.definitionRevision;
    checks["stale-revision-refused"] = CommitEdit(staleOwner, stale, detached)
        == CommitResult::Refused;
    checks["stale-zero-mutation"] = SameState(staleOwner.state, original)
        && staleOwner.begins == 0;

    auto failure = [&](int mode) {
        Owner failed(original);
        failed.failStage = mode == 0;
        failed.failReplace = mode == 1;
        failed.failVerify = mode == 2;
        const EditProposal request = Move(original, EditKind::MovePole, ID(10), {{1, 0, 0}},
            std::uint8_t(30 + mode));
        return CommitEdit(failed, request, detached) == CommitResult::Refused
            && failed.begins == 1 && failed.commits == 0 && failed.aborts == 1
            && failed.history.size() == 1 && SameState(failed.state, original) && failed.closed();
    };
    checks["stage-failure-aborts"] = failure(0);
    checks["replace-failure-aborts"] = failure(1);
    checks["verify-failure-aborts"] = failure(2);

    Owner foreign(original);
    foreign.foreignOpen = true;
    const EditProposal blocked = Move(original, EditKind::MovePole, ID(10), {{1, 0, 0}}, 40);
    checks["foreign-command-refused"] = CommitEdit(foreign, blocked, detached)
        == CommitResult::Busy && foreign.begins == 0 && SameState(foreign.state, original);
    return checks;
}
} // namespace core3d::bounded_curve::debug

inline std::map<std::string, bool> Core3DDebugBoundedCurveEditProbe() {
    return core3d::bounded_curve::debug::Probe();
}
