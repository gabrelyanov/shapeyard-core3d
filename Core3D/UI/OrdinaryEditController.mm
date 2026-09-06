#include "OrdinaryEditController.hpp"
#include "../Common/Core3DMobileResourceLimits.h"
#import <Foundation/Foundation.h>
#include <gp_Quaternion.hxx>
#include <Standard_Failure.hxx>
#include <Standard_GUID.hxx>
#include <cmath>
#include <algorithm>
#include <limits>
#include <unordered_set>
#include <utility>

namespace core3d {
namespace {
bool MatricesEqual(const gp_Trsf& a, const gp_Trsf& b) {
    for (int row = 1; row <= 3; ++row) {
        for (int column = 1; column <= 4; ++column) {
            if (!std::isfinite(a.Value(row, column))
                || a.Value(row, column) != b.Value(row, column)) { return false; }
        }
    }
    return true;
}
// Admission accounts only for bounded floating-point arithmetic in composing
// a rotation with a persisted transform. Candidate sealing/readback below
// still compares exact persisted values, never this tolerance.
bool RotationArithmeticEqual(double a, double b, double arithmeticScale) {
    return std::isfinite(a) && std::isfinite(b) && std::isfinite(arithmeticScale)
        && std::abs(a - b) <= 64 * std::numeric_limits<double>::epsilon()
            * std::max({1.0, std::abs(a), std::abs(b), arithmeticScale});
}
bool IsRotationAroundPivot(const OrdinaryRotationAroundPivot& rotation) {
    if (rotation.delta.ScaleFactor() != 1.0) { return false; }
    for (int index = 1; index <= 3; ++index) {
        if (!std::isfinite(rotation.pivot.Coord(index))
            || std::abs(rotation.pivot.Coord(index)) > limits::kMaximumModelCoordinateMagnitude) { return false; }
        for (int column = 1; column <= 4; ++column) {
            if (!std::isfinite(rotation.delta.Value(index, column))) { return false; }
        }
    }
    const gp_Pnt fixed = rotation.pivot.Transformed(rotation.delta);
    for (int index = 1; index <= 3; ++index) {
        // Near-zero coordinates can result from subtracting large terms.
        const double scale = std::max({1.0, std::abs(rotation.pivot.X()),
                                       std::abs(rotation.pivot.Y()), std::abs(rotation.pivot.Z())});
        if (!std::isfinite(fixed.Coord(index))
            || std::abs(fixed.Coord(index) - rotation.pivot.Coord(index))
                > 64 * std::numeric_limits<double>::epsilon() * scale) { return false; }
    }
    return true;
}
std::array<Standard_Real, 8> EncodedTransform(const gp_Trsf& transform) {
    const gp_Quaternion q = transform.GetRotation();
    return {{transform.TranslationPart().X(), transform.TranslationPart().Y(),
             transform.TranslationPart().Z(), q.X(), q.Y(), q.Z(), q.W(), transform.ScaleFactor()}};
}
bool CandidateIsFinite(const gp_Trsf& transform) {
    const auto values = EncodedTransform(transform);
    for (double value : values) { if (!std::isfinite(value)) { return false; } }
    for (int index = 0; index < 3; ++index) {
        if (std::abs(values[index]) > limits::kMaximumModelCoordinateMagnitude) { return false; }
    }
    return std::abs(values[7]) > std::numeric_limits<Standard_Real>::epsilon();
}
}

OrdinaryEditLease::OrdinaryEditLease(std::weak_ptr<OrdinaryEditController> controller,
                                   std::uint64_t token,
                                   std::shared_ptr<const std::uint8_t> lifetime) noexcept
    : _controller(std::move(controller)), _lifetime(std::move(lifetime)), _token(token) {}
OrdinaryEditLease::OrdinaryEditLease(OrdinaryEditLease&& other) noexcept
    : _controller(std::move(other._controller)), _lifetime(std::move(other._lifetime)),
      _token(std::exchange(other._token, 0)) {}
OrdinaryEditLease& OrdinaryEditLease::operator=(OrdinaryEditLease&& other) noexcept {
    if (this != &other) {
        (void)cancel();
        _controller = std::move(other._controller);
        _lifetime = std::move(other._lifetime);
        _token = std::exchange(other._token, 0);
    }
    return *this;
}
OrdinaryEditLease::~OrdinaryEditLease() noexcept { (void)cancel(); }
OrdinaryEditResult OrdinaryEditLease::stageAndCommit() noexcept {
    const auto controller = _controller.lock();
    if (!controller || _token == 0) { return OrdinaryEditResult::Invalid; }
    // Wrong-thread rejection must not consume the main-thread lease.
    if (![NSThread isMainThread]) { return OrdinaryEditResult::Invalid; }
    const auto token = std::exchange(_token, 0);
    const auto result = controller->stageAndCommit(token);
    _lifetime.reset();
    return result;
}
OrdinaryEditResult OrdinaryEditLease::cancel() noexcept {
    const auto controller = _controller.lock();
    if (!controller || _token == 0) { return OrdinaryEditResult::NoChange; }
    if (![NSThread isMainThread]) { return OrdinaryEditResult::Invalid; }
    const auto result = controller->cancel(std::exchange(_token, 0));
    _lifetime.reset();
    return result;
}

OrdinaryEditController::OrdinaryEditController(Handle(OcctDocument) document,
                                             OrdinaryEditPresentationHost& host)
    : _document(std::move(document)), _host(host) {}

bool OrdinaryEditController::blocksNormalWork() const noexcept {
    return ![NSThread isMainThread] || _entering || _reconciling
        || _state != OrdinaryEditState::Idle || _pending.has_value() || _command.isRetained();
}

OrdinaryEditLease OrdinaryEditController::beginTransform(
    const std::vector<OrdinaryTransformChange>& changes, OrdinaryEditResult* failure) noexcept
{
    const auto reject = [&](OrdinaryEditResult reason) {
        if (failure) { *failure = reason; }
        return OrdinaryEditLease();
    };
    if (![NSThread isMainThread]) { return reject(OrdinaryEditResult::Invalid); }
    if (blocksNormalWork()) { return reject(OrdinaryEditResult::Busy); }
    if (_document.IsNull() || changes.empty() || changes.size() > 1024
        || _nextToken == std::numeric_limits<std::uint64_t>::max()) {
        return reject(OrdinaryEditResult::Invalid);
    }
    _entering = true;
    struct EnterReset { bool& flag; ~EnterReset() { flag = false; } } reset{_entering};
    try {
        const auto self = shared_from_this();
        const auto lifetime = std::make_shared<const std::uint8_t>(0);
        const auto document = _document->Document();
        if (document.IsNull() || document->HasOpenCommand()) { return reject(OrdinaryEditResult::Busy); }
        OrdinaryTransformLedger ledger;
        ledger.records.reserve(changes.size());
        std::unordered_set<std::string> entities;
        std::unordered_set<const AIS_Shape*> presentations;
        bool changed = false;
        for (const auto& request : changes) {
            OrdinaryTransformRecord record;
            if (request.operation != OrdinaryTransformOperation::Translate
                && request.operation != OrdinaryTransformOperation::Rotate
                && request.operation != OrdinaryTransformOperation::Scale) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.presentation.IsNull() || request.shape.IsNull()
                || !CandidateIsFinite(request.transform)
                || !_document->CaptureObjectTransformStateForLabel(request.label, record.previous)
                || !entities.insert(record.previous.entityIdentifier).second
                || !presentations.insert(request.presentation.get()).second) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.rotationAroundPivot.has_value() != changes.front().rotationAroundPivot.has_value()) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.rotationAroundPivot) {
                const auto& rotation = *request.rotationAroundPivot;
                const auto& common = *changes.front().rotationAroundPivot;
                if (request.operation != OrdinaryTransformOperation::Rotate || !IsRotationAroundPivot(rotation)
                    || !rotation.pivot.IsEqual(common.pivot, 0.0)
                    || !MatricesEqual(rotation.delta, common.delta)) { return reject(OrdinaryEditResult::Invalid); }
            }
            if (request.operation == OrdinaryTransformOperation::Translate) {
                for (int row = 1; row <= 3; ++row) {
                    for (int column = 1; column <= 3; ++column) {
                        if (record.previous.transform.Value(row, column) != request.transform.Value(row, column)) {
                            return reject(OrdinaryEditResult::Invalid);
                        }
                    }
                }
            } else if (request.operation == OrdinaryTransformOperation::Rotate) {
                if (record.previous.transform.ScaleFactor() != request.transform.ScaleFactor()) {
                    return reject(OrdinaryEditResult::Invalid);
                }
                if (request.rotationAroundPivot) {
                    const gp_Trsf expected = request.rotationAroundPivot->delta * record.previous.transform;
                    for (int row = 1; row <= 3; ++row) {
                        for (int column = 1; column <= 4; ++column) {
                            // Bound cancellation by the actual products being summed,
                            // including the pivot translation for the position column.
                            const gp_Trsf& delta = request.rotationAroundPivot->delta;
                            double arithmeticScale = column == 4 ? std::abs(delta.Value(row, 4)) : 0.0;
                            for (int term = 1; term <= 3; ++term) {
                                arithmeticScale += std::abs(delta.Value(row, term)
                                    * record.previous.transform.Value(term, column));
                            }
                            if (!RotationArithmeticEqual(expected.Value(row, column),
                                                         request.transform.Value(row, column), arithmeticScale)) {
                                return reject(OrdinaryEditResult::Invalid);
                            }
                        }
                    }
                } else {
                    for (int row = 1; row <= 3; ++row) {
                        if (record.previous.transform.Value(row, 4) != request.transform.Value(row, 4)) {
                            return reject(OrdinaryEditResult::Invalid);
                        }
                    }
                }
            }
            const bool geometryChanges = !record.previous.shape.IsEqual(request.shape);
            if (geometryChanges && request.operation != OrdinaryTransformOperation::Scale) {
                return reject(OrdinaryEditResult::Invalid);
            }
            const auto representation = record.previous.resolvedRepresentation;
            if ((representation != OcctGeometryRepresentation::BRep
                    && representation != OcctGeometryRepresentation::LegacyUnknown
                    && representation != OcctGeometryRepresentation::TriangleMesh)
                || (request.operation == OrdinaryTransformOperation::Scale
                    && representation == OcctGeometryRepresentation::TriangleMesh)) {
                return reject(OrdinaryEditResult::Invalid);
            }
            changed = changed || geometryChanges || !MatricesEqual(record.previous.transform, request.transform);
            record.requested = request;
            ledger.records.push_back(std::move(record));
        }
        if (!changed) { return reject(OrdinaryEditResult::NoChange); }
        if (!_host.admitTransform(ledger)) { return reject(OrdinaryEditResult::Invalid); }
        // Admission is synchronous, but re-read all authority after the host
        // boundary rather than assuming that a successful callback kept it.
        for (const auto& record : ledger.records) {
            if (!captureMatches(record.previous)) { return reject(OrdinaryEditResult::Invalid); }
        }
        _pending.emplace(std::move(ledger));
        _leaseLifetime = lifetime;
        _activeToken = ++_nextToken;
        const auto began = _command.begin(_document);
        if (began != OrdinaryCommandBeginResult::Started) {
            if (began == OrdinaryCommandBeginResult::OutcomeUnknown) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return reject(OrdinaryEditResult::OutcomeUnknown);
            }
            _pending.reset();
            _activeToken = 0;
            return reject(began == OrdinaryCommandBeginResult::Busy
                ? OrdinaryEditResult::Busy : began == OrdinaryCommandBeginResult::Invalid
                ? OrdinaryEditResult::Invalid : OrdinaryEditResult::RetryableFailure);
        }
        _state = OrdinaryEditState::OpenOwned;
        return OrdinaryEditLease(self, _activeToken, lifetime);
    } catch (...) {
        if (_command.isRetained()) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return reject(OrdinaryEditResult::OutcomeUnknown);
        }
        _pending.reset();
        _activeToken = 0;
        return reject(OrdinaryEditResult::Invalid);
    }
}

OrdinaryEditLease OrdinaryEditController::beginGrouping(
    const std::vector<OcctSavedGroup>& groups, OrdinaryEditResult* failure) noexcept {
    const auto reject = [&](OrdinaryEditResult result) {
        if (failure) { *failure = result; }
        return OrdinaryEditLease();
    };
    if (![NSThread isMainThread]) { return reject(OrdinaryEditResult::Invalid); }
    if (blocksNormalWork()) { return reject(OrdinaryEditResult::Busy); }
    if (_document.IsNull() || groups.size() > 128
        || _nextToken == std::numeric_limits<std::uint64_t>::max()) {
        return reject(OrdinaryEditResult::Invalid);
    }
    _entering = true;
    struct EnterReset { bool& flag; ~EnterReset() { flag = false; } } reset{_entering};
    try {
        const auto self = shared_from_this();
        const auto lifetime = std::make_shared<const std::uint8_t>(0);
        const auto document = _document->Document();
        if (document.IsNull() || document->HasOpenCommand()) { return reject(OrdinaryEditResult::Busy); }
        OrdinaryGroupingLedger ledger;
        if (!_document->CaptureSavedGroups(ledger.previous)) { return reject(OrdinaryEditResult::Invalid); }
        ledger.requested = groups;
        std::unordered_set<std::string> groupIDs, entities;
        for (const auto& group : groups) {
            if (group.identifier.size() != 36 || !Standard_GUID::CheckGUIDFormat(group.identifier.c_str())
                || !groupIDs.insert(group.identifier).second || !OcctObjectNameIsValid(group.name)
                || group.members.size() > 32) { return reject(OrdinaryEditResult::Invalid); }
            for (const auto& label : group.members) {
                OcctObjectNameState object;
                if (!_document->CaptureObjectNameStateForLabel(label, object)
                    || !entities.insert(object.object.entityIdentifier).second) { return reject(OrdinaryEditResult::Invalid); }
                ledger.objects.push_back(std::move(object));
            }
        }
        bool changed = groups.size() != ledger.previous.groups.size();
        for (const auto& group : ledger.previous.groups) {
            const auto match = std::find_if(groups.begin(), groups.end(), [&](const auto& value) { return value.identifier == group.identifier; });
            changed = changed || match == groups.end();
            if (match != groups.end()) {
                changed = changed || !match->name.IsEqual(group.name) || match->members.size() != group.members.size();
                for (const auto& label : group.members) {
                    changed = changed || std::none_of(match->members.begin(), match->members.end(), [&](const auto& l) { return l.IsEqual(label); });
                }
            }
            for (const auto& label : group.members) {
                OcctObjectNameState object;
                if (!_document->CaptureObjectNameStateForLabel(label, object)) { return reject(OrdinaryEditResult::Invalid); }
                if (entities.insert(object.object.entityIdentifier).second) { ledger.objects.push_back(std::move(object)); }
            }
        }
        if (!changed) { return reject(OrdinaryEditResult::NoChange); }
        if (!_host.admitGrouping(ledger) || !groupingMatches(ledger, false)) { return reject(OrdinaryEditResult::Invalid); }
        _pending.emplace(std::move(ledger));
        _leaseLifetime = lifetime;
        _activeToken = ++_nextToken;
        const auto began = _command.begin(_document);
        if (began != OrdinaryCommandBeginResult::Started) {
            if (began == OrdinaryCommandBeginResult::OutcomeUnknown) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return reject(OrdinaryEditResult::OutcomeUnknown);
            }
            _pending.reset();
            _activeToken = 0;
            return reject(began == OrdinaryCommandBeginResult::Busy
                ? OrdinaryEditResult::Busy : began == OrdinaryCommandBeginResult::Invalid
                ? OrdinaryEditResult::Invalid : OrdinaryEditResult::RetryableFailure);
        }
        _state = OrdinaryEditState::OpenOwned;
        return OrdinaryEditLease(self, _activeToken, lifetime);
    } catch (...) {
        if (_command.isRetained()) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return reject(OrdinaryEditResult::OutcomeUnknown);
        }
        _pending.reset();
        _activeToken = 0;
        return reject(OrdinaryEditResult::Invalid);
    }
}

bool OrdinaryEditController::groupingMatches(const OrdinaryGroupingLedger& ledger, bool candidate) const noexcept {
    OcctSavedGroupState state;
    if (_document.IsNull() || !_document->CaptureSavedGroups(state)
        || !(candidate ? ledger.candidate : ledger.previous).IsEqual(state)) { return false; }
    for (const auto& object : ledger.objects) { if (!captureMatches(object)) { return false; } }
    return true;
}
OrdinaryEditResult OrdinaryEditController::stageGroupingAndCommit(std::uint64_t token) noexcept {
    try {
        auto& ledger = std::get<OrdinaryGroupingLedger>(*_pending);
        if (!groupingMatches(ledger, false)) { return cancel(token); }
        if (_command.observe() != OrdinaryCommandObservation::OpenOwned) {
            _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown;
        }
#ifdef DEBUG
        if (_stageFailureIndex == 0) { _stageFailureIndex = -1; throw Standard_Failure("Grouping staging fault"); }
#endif
        if (!_document->StageSavedGroups(ledger.requested) || !_document->CaptureSavedGroups(ledger.candidate)) {
            throw Standard_Failure("Grouping readback failed");
        }
        for (const auto& object : ledger.objects) {
            if (!captureMatches(object)) { throw Standard_Failure("Grouping changed object authority"); }
        }
#ifdef DEBUG
        if (_stageFailureIndex == 1) { _stageFailureIndex = -1; throw Standard_Failure("Grouping staged abort fault"); }
#endif
        ledger.candidateSealed = true;
        if (_command.commitAndObserve() == OrdinaryCommandObservation::Unavailable) {
            _state = OrdinaryEditState::OutcomeUnknown; _activeToken = 0; return OrdinaryEditResult::OutcomeUnknown;
        }
    } catch (...) {}
    _state = OrdinaryEditState::OutcomeUnknown; _activeToken = 0; return reconcile();
}
OrdinaryEditResult OrdinaryEditController::reconcileGroupingImpl() noexcept {
    try {
        auto& ledger = std::get<OrdinaryGroupingLedger>(*_pending);
#ifdef DEBUG
        if (_truthUnavailableCount > 0) {
            --_truthUnavailableCount; _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown;
        }
#endif
        auto observation = _command.observe();
        if (observation == OrdinaryCommandObservation::OpenOwned) { observation = _command.abortAndObserve(); }
        const bool candidate = observation == OrdinaryCommandObservation::ClosedWithCandidateMarker;
        const bool previous = observation == OrdinaryCommandObservation::ClosedWithPriorMarker;
        if ((!candidate && !previous) || (candidate && !ledger.candidateSealed) || !groupingMatches(ledger, candidate)) {
            _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown;
        }
        _committed = candidate; _state = OrdinaryEditState::RepairPending;
        if (!_host.repairGrouping(ledger, candidate)) { return OrdinaryEditResult::OutcomeUnknown; }
        if (!groupingMatches(ledger, candidate)) { _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown; }
        _state = OrdinaryEditState::Publishing;
        if (!_command.releaseClosed()) { _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown; }
        if (candidate && !_didPublish) { _didPublish = true; try { _document->NotifyChanges(); } catch (...) {} }
        clearResolved();
        return candidate ? OrdinaryEditResult::Committed : OrdinaryEditResult::RetryableFailure;
    } catch (...) { _state = OrdinaryEditState::OutcomeUnknown; return OrdinaryEditResult::OutcomeUnknown; }
}

OrdinaryEditLease OrdinaryEditController::beginNames(
    const std::vector<OrdinaryNameChange>& changes, OrdinaryEditResult* failure) noexcept {
    const auto reject = [&](OrdinaryEditResult result) {
        if (failure) { *failure = result; }
        return OrdinaryEditLease();
    };
    if (![NSThread isMainThread]) { return reject(OrdinaryEditResult::Invalid); }
    if (blocksNormalWork()) { return reject(OrdinaryEditResult::Busy); }
    if (_document.IsNull() || changes.empty() || changes.size() > 1024
        || _nextToken == std::numeric_limits<std::uint64_t>::max()) {
        return reject(OrdinaryEditResult::Invalid);
    }
    _entering = true;
    struct EnterReset { bool& flag; ~EnterReset() { flag = false; } } reset{_entering};
    try {
        const auto self = shared_from_this();
        const auto lifetime = std::make_shared<const std::uint8_t>(0);
        const auto document = _document->Document();
        if (document.IsNull() || document->HasOpenCommand()) { return reject(OrdinaryEditResult::Busy); }
        OrdinaryNameLedger ledger;
        ledger.records.reserve(changes.size());
        std::unordered_set<std::string> entities;
        bool changed = false;
        for (const auto& request : changes) {
            OrdinaryNameRecord record;
            if (!OcctObjectNameIsValid(request.name)
                || !_document->CaptureObjectNameStateForLabel(request.label, record.previous)
                || !entities.insert(record.previous.object.entityIdentifier).second) {
                return reject(OrdinaryEditResult::Invalid);
            }
            changed = changed || !record.previous.namePresent || !record.previous.name.IsEqual(request.name);
            record.requested = request;
            ledger.records.push_back(std::move(record));
        }
        if (!changed) { return reject(OrdinaryEditResult::NoChange); }
        if (!_host.admitNames(ledger)) { return reject(OrdinaryEditResult::Invalid); }
        for (const auto& record : ledger.records) {
            if (!captureMatches(record.previous)) { return reject(OrdinaryEditResult::Invalid); }
        }
        _pending.emplace(std::move(ledger));
        _leaseLifetime = lifetime;
        _activeToken = ++_nextToken;
        const auto began = _command.begin(_document);
        if (began != OrdinaryCommandBeginResult::Started) {
            if (began == OrdinaryCommandBeginResult::OutcomeUnknown) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return reject(OrdinaryEditResult::OutcomeUnknown);
            }
            _pending.reset();
            _activeToken = 0;
            return reject(began == OrdinaryCommandBeginResult::Busy
                ? OrdinaryEditResult::Busy : began == OrdinaryCommandBeginResult::Invalid
                ? OrdinaryEditResult::Invalid : OrdinaryEditResult::RetryableFailure);
        }
        _state = OrdinaryEditState::OpenOwned;
        return OrdinaryEditLease(self, _activeToken, lifetime);
    } catch (...) {
        if (_command.isRetained()) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return reject(OrdinaryEditResult::OutcomeUnknown);
        }
        _pending.reset();
        _activeToken = 0;
        return reject(OrdinaryEditResult::Invalid);
    }
}

bool OrdinaryEditController::captureMatches(const OcctObjectNameState& expected) const noexcept {
    OcctObjectNameState actual;
    return !_document.IsNull()
        && _document->CaptureObjectNameStateForLabel(expected.object.label, actual)
        && expected.IsEqual(actual);
}

OrdinaryEditResult OrdinaryEditController::stageNamesAndCommit(std::uint64_t token) noexcept {
    // stageAndCommit has already checked thread, state and lease ownership.
    try {
        auto& ledger = std::get<OrdinaryNameLedger>(*_pending);
        for (const auto& record : ledger.records) {
            if (!captureMatches(record.previous)) { return cancel(token); }
        }
        if (_command.observe() != OrdinaryCommandObservation::OpenOwned) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        std::size_t index = 0;
        for (auto& record : ledger.records) {
#ifdef DEBUG
            if (_stageFailureIndex == static_cast<int>(index)) {
                _stageFailureIndex = -1;
                throw Standard_Failure("Ordinary name staging fault");
            }
#endif
            ++index;
            if (!_document->SetObjectNameForLabel(record.requested.label, record.requested.name)
                || !_document->CaptureObjectNameStateForLabel(record.requested.label, record.candidate)
                || !record.candidate.object.IsEqual(record.previous.object)
                || !record.candidate.namePresent
                || !record.candidate.name.IsEqual(record.requested.name)) {
                throw Standard_Failure("Ordinary name candidate readback failed");
            }
        }
        ledger.candidateSealed = true;
        if (_command.commitAndObserve() == OrdinaryCommandObservation::Unavailable) {
            _state = OrdinaryEditState::OutcomeUnknown;
            _activeToken = 0;
            return OrdinaryEditResult::OutcomeUnknown;
        }
    } catch (...) {
        // The owned command and exact metadata ledger determine the outcome.
    }
    _state = OrdinaryEditState::OutcomeUnknown;
    _activeToken = 0;
    return reconcile();
}

OrdinaryEditResult OrdinaryEditController::reconcileNamesImpl() noexcept {
    try {
        auto& ledger = std::get<OrdinaryNameLedger>(*_pending);
#ifdef DEBUG
        if (_truthUnavailableCount > 0) {
            --_truthUnavailableCount;
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
#endif
        auto observation = _command.observe();
        if (observation == OrdinaryCommandObservation::OpenOwned) { observation = _command.abortAndObserve(); }
        const bool candidate = observation == OrdinaryCommandObservation::ClosedWithCandidateMarker;
        const bool previous = observation == OrdinaryCommandObservation::ClosedWithPriorMarker;
        if ((!candidate && !previous) || (candidate && !ledger.candidateSealed)) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        for (const auto& record : ledger.records) {
            if (!captureMatches(candidate ? record.candidate : record.previous)) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return OrdinaryEditResult::OutcomeUnknown;
            }
        }
        _committed = candidate;
        _state = OrdinaryEditState::RepairPending;
        if (!_host.repairNames(ledger, candidate)) { return OrdinaryEditResult::OutcomeUnknown; }
        for (const auto& record : ledger.records) {
            if (!captureMatches(candidate ? record.candidate : record.previous)) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return OrdinaryEditResult::OutcomeUnknown;
            }
        }
        _state = OrdinaryEditState::Publishing;
        if (!_command.releaseClosed()) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        if (candidate && !_didPublish) {
            _didPublish = true;
            try { _document->NotifyChanges(); } catch (...) {}
        }
        clearResolved();
        return candidate ? OrdinaryEditResult::Committed : OrdinaryEditResult::RetryableFailure;
    } catch (...) {
        _state = OrdinaryEditState::OutcomeUnknown;
        return OrdinaryEditResult::OutcomeUnknown;
    }
}

OrdinaryEditLease OrdinaryEditController::beginVisibility(
    const std::vector<OrdinaryVisibilityChange>& changes, OrdinaryEditResult* failure) noexcept {
    const auto reject = [&](OrdinaryEditResult result) {
        if (failure) { *failure = result; }
        return OrdinaryEditLease();
    };
    if (![NSThread isMainThread]) { return reject(OrdinaryEditResult::Invalid); }
    if (blocksNormalWork()) { return reject(OrdinaryEditResult::Busy); }
    if (_document.IsNull() || changes.empty() || changes.size() > 1024
        || _nextToken == std::numeric_limits<std::uint64_t>::max()) {
        return reject(OrdinaryEditResult::Invalid);
    }
    _entering = true;
    struct EnterReset { bool& flag; ~EnterReset() { flag = false; } } reset{_entering};
    try {
        const auto self = shared_from_this();
        const auto lifetime = std::make_shared<const std::uint8_t>(0);
        const auto document = _document->Document();
        if (document.IsNull() || document->HasOpenCommand()) { return reject(OrdinaryEditResult::Busy); }
        OrdinaryVisibilityLedger ledger;
        ledger.records.reserve(changes.size());
        std::unordered_set<std::string> entities;
        bool changed = false;
        for (const auto& request : changes) {
            OrdinaryVisibilityRecord record;
            if (!_document->CaptureObjectVisibilityStateForLabel(request.label, record.previous)
                || !entities.insert(record.previous.object.object.entityIdentifier).second) {
                return reject(OrdinaryEditResult::Invalid);
            }
            if (request.visible) {
                for (bool hidden : record.previous.layerInvisibleAttributePresent) {
                    if (hidden) { return reject(OrdinaryEditResult::Invalid); }
                }
            }
            changed = changed || record.previous.invisibleAttributePresent == request.visible;
            record.requested = request;
            ledger.records.push_back(std::move(record));
        }
        if (!changed) { return reject(OrdinaryEditResult::NoChange); }
        if (!_host.admitVisibility(ledger)) { return reject(OrdinaryEditResult::Invalid); }
        for (const auto& record : ledger.records) {
            if (!captureMatches(record.previous)) { return reject(OrdinaryEditResult::Invalid); }
        }
        _pending.emplace(std::move(ledger));
        _leaseLifetime = lifetime;
        _activeToken = ++_nextToken;
        const auto began = _command.begin(_document);
        if (began != OrdinaryCommandBeginResult::Started) {
            if (began == OrdinaryCommandBeginResult::OutcomeUnknown) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return reject(OrdinaryEditResult::OutcomeUnknown);
            }
            _pending.reset();
            _activeToken = 0;
            return reject(began == OrdinaryCommandBeginResult::Busy
                ? OrdinaryEditResult::Busy : began == OrdinaryCommandBeginResult::Invalid
                ? OrdinaryEditResult::Invalid : OrdinaryEditResult::RetryableFailure);
        }
        _state = OrdinaryEditState::OpenOwned;
        return OrdinaryEditLease(self, _activeToken, lifetime);
    } catch (...) {
        if (_command.isRetained()) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return reject(OrdinaryEditResult::OutcomeUnknown);
        }
        _pending.reset();
        _activeToken = 0;
        return reject(OrdinaryEditResult::Invalid);
    }
}

bool OrdinaryEditController::captureMatches(const OcctObjectVisibilityState& expected) const noexcept {
    OcctObjectVisibilityState actual;
    return !_document.IsNull()
        && _document->CaptureObjectVisibilityStateForLabel(expected.object.object.label, actual)
        && expected.IsEqual(actual);
}

OrdinaryEditResult OrdinaryEditController::stageVisibilityAndCommit(std::uint64_t token) noexcept {
    // stageAndCommit has already checked thread, state and lease ownership.
    try {
        auto& ledger = std::get<OrdinaryVisibilityLedger>(*_pending);
        for (const auto& record : ledger.records) {
            if (!captureMatches(record.previous)) { return cancel(token); }
        }
        if (_command.observe() != OrdinaryCommandObservation::OpenOwned) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        std::size_t index = 0;
        for (auto& record : ledger.records) {
#ifdef DEBUG
            if (_stageFailureIndex == static_cast<int>(index)) {
                _stageFailureIndex = -1;
                throw Standard_Failure("Ordinary visibility staging fault");
            }
#endif
            ++index;
            if (!_document->SetObjectVisibilityForLabel(record.requested.label, record.requested.visible)
                || !_document->CaptureObjectVisibilityStateForLabel(record.requested.label, record.candidate)
                || !record.candidate.HasSameObjectAndLayers(record.previous)
                || record.candidate.invisibleAttributePresent == record.requested.visible) {
                throw Standard_Failure("Ordinary visibility candidate readback failed");
            }
        }
        ledger.candidateSealed = true;
        if (_command.commitAndObserve() == OrdinaryCommandObservation::Unavailable) {
            _state = OrdinaryEditState::OutcomeUnknown;
            _activeToken = 0;
            return OrdinaryEditResult::OutcomeUnknown;
        }
    } catch (...) {
        // The owned command and exact metadata ledger determine the outcome.
    }
    _state = OrdinaryEditState::OutcomeUnknown;
    _activeToken = 0;
    return reconcile();
}

OrdinaryEditResult OrdinaryEditController::reconcileVisibilityImpl() noexcept {
    try {
        auto& ledger = std::get<OrdinaryVisibilityLedger>(*_pending);
#ifdef DEBUG
        if (_truthUnavailableCount > 0) {
            --_truthUnavailableCount;
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
#endif
        auto observation = _command.observe();
        if (observation == OrdinaryCommandObservation::OpenOwned) { observation = _command.abortAndObserve(); }
        const bool candidate = observation == OrdinaryCommandObservation::ClosedWithCandidateMarker;
        const bool previous = observation == OrdinaryCommandObservation::ClosedWithPriorMarker;
        if ((!candidate && !previous) || (candidate && !ledger.candidateSealed)) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        for (const auto& record : ledger.records) {
            if (!captureMatches(candidate ? record.candidate : record.previous)) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return OrdinaryEditResult::OutcomeUnknown;
            }
        }
        _committed = candidate;
        _state = OrdinaryEditState::RepairPending;
        if (!_host.repairVisibility(ledger, candidate)) { return OrdinaryEditResult::OutcomeUnknown; }
        for (const auto& record : ledger.records) {
            if (!captureMatches(candidate ? record.candidate : record.previous)) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return OrdinaryEditResult::OutcomeUnknown;
            }
        }
        _state = OrdinaryEditState::Publishing;
        if (!_command.releaseClosed()) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        if (candidate && !_didPublish) {
            _didPublish = true;
            try { _document->NotifyChanges(); } catch (...) {}
        }
        clearResolved();
        return candidate ? OrdinaryEditResult::Committed : OrdinaryEditResult::RetryableFailure;
    } catch (...) {
        _state = OrdinaryEditState::OutcomeUnknown;
        return OrdinaryEditResult::OutcomeUnknown;
    }
}

bool OrdinaryEditController::captureMatches(const OcctObjectTransformState& expected) const noexcept {
    OcctObjectTransformState actual;
    return !_document.IsNull()
        && _document->CaptureObjectTransformStateForLabel(expected.label, actual)
        && expected.IsEqual(actual);
}

OrdinaryEditResult OrdinaryEditController::stageAndCommit(std::uint64_t token) noexcept {
    if (![NSThread isMainThread]) { return OrdinaryEditResult::Invalid; }
    if (_entering || _reconciling || _state != OrdinaryEditState::OpenOwned
        || !_pending || token == 0 || token != _activeToken) { return OrdinaryEditResult::Busy; }
    if (std::holds_alternative<OrdinaryGroupingLedger>(*_pending)) { return stageGroupingAndCommit(token); }
    if (std::holds_alternative<OrdinaryVisibilityLedger>(*_pending)) { return stageVisibilityAndCommit(token); }
    if (std::holds_alternative<OrdinaryNameLedger>(*_pending)) { return stageNamesAndCommit(token); }
    try {
        auto& ledger = std::get<OrdinaryTransformLedger>(*_pending);
        // Reject stale baseline before the first write; ownership alone is
        // insufficient admission for a batch mutation.
        for (const auto& record : ledger.records) {
            if (!captureMatches(record.previous)) { return cancel(token); }
        }
        if (_command.observe() != OrdinaryCommandObservation::OpenOwned) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        std::size_t index = 0;
        for (auto& record : ledger.records) {
#ifdef DEBUG
            if (_stageFailureIndex == static_cast<int>(index)) {
                _stageFailureIndex = -1;
                throw Standard_Failure("Ordinary transform staging fault");
            }
#endif
            ++index;
            Handle(AIS_Shape) candidate = new AIS_Shape(record.requested.shape);
            candidate->SetLocalTransformation(record.requested.transform);
            if ((!record.previous.shape.IsEqual(record.requested.shape)
                    && !_document->ReplaceShape(record.previous.label, candidate))
                || !_document->SaveObjectTransform(record.previous.label, candidate)
                || !_document->CaptureObjectTransformStateForLabel(record.previous.label, record.candidate)
                || !record.candidate.shape.IsEqual(record.requested.shape)
                || record.candidate.entityIdentifier != record.previous.entityIdentifier
                || record.candidate.definitionIdentifier != record.previous.definitionIdentifier
                || record.candidate.scalars != EncodedTransform(record.requested.transform)) {
                throw Standard_Failure("Ordinary transform candidate readback failed");
            }
            gp_Ax1 referenceAxis;
            if (!_document->ResolveReferenceAxisInWorld(record.candidate.label,
                    record.candidate.shape.Location(), referenceAxis)) {
                throw Standard_Failure("Ordinary transform exceeds persisted reference-axis limits");
            }
            for (bool present : record.candidate.present) {
                if (!present) { throw Standard_Failure("Incomplete ordinary transform candidate"); }
            }
        }
        ledger.candidateSealed = true;
        if (_command.commitAndObserve() == OrdinaryCommandObservation::Unavailable) {
            _state = OrdinaryEditState::OutcomeUnknown;
            _activeToken = 0;
            return OrdinaryEditResult::OutcomeUnknown;
        }
    } catch (...) {
        // The retained before/candidate ledger decides the outcome. No blind
        // rollback or success claim based on a Commit Boolean/exception.
    }
    _state = OrdinaryEditState::OutcomeUnknown;
    _activeToken = 0;
    return reconcile();
}

OrdinaryEditResult OrdinaryEditController::cancel(std::uint64_t token) noexcept {
    if (![NSThread isMainThread]) { return OrdinaryEditResult::Invalid; }
    if (_entering || _reconciling || token == 0 || token != _activeToken || !_pending) {
        return OrdinaryEditResult::Busy;
    }
    const auto observation = _command.abortAndObserve();
    _activeToken = 0;
    _state = OrdinaryEditState::OutcomeUnknown;
    if (observation == OrdinaryCommandObservation::Unavailable) { return OrdinaryEditResult::OutcomeUnknown; }
    return reconcile();
}

bool OrdinaryEditController::presentationMatches(
    const OrdinaryTransformLedger& ledger, bool committed) const noexcept {
    try {
        for (const auto& record : ledger.records) {
            const auto& expected = committed ? record.candidate : record.previous;
            const auto& presentation = record.requested.presentation;
            if (!captureMatches(expected) || presentation.IsNull()
                || presentation->Shape().IsNull() || !presentation->Shape().IsEqual(expected.shape)
                || !MatricesEqual(presentation->LocalTransformation(), expected.transform)
                || !_document->ShapeLabel(presentation).IsEqual(expected.label)
                || !_document->IsPresentationEditable(presentation)) { return false; }
        }
        return true;
    } catch (...) { return false; }
}

void OrdinaryEditController::clearResolved() noexcept {
    _pending.reset();
    _leaseLifetime.reset();
    _activeToken = 0;
    _committed = false;
    _didPublish = false;
    _state = OrdinaryEditState::Idle;
}

OrdinaryEditResult OrdinaryEditController::reconcile() noexcept {
    if (![NSThread isMainThread]) { return OrdinaryEditResult::Invalid; }
    if (_entering || _reconciling || _state == OrdinaryEditState::Publishing) { return OrdinaryEditResult::Busy; }
    if (!_pending) { return _command.isRetained() ? OrdinaryEditResult::OutcomeUnknown : OrdinaryEditResult::NoChange; }
    // An active lease is the only authority allowed to stage or cancel.
    if (_state == OrdinaryEditState::OpenOwned && _activeToken != 0) {
        if (!_leaseLifetime.expired()) { return OrdinaryEditResult::Busy; }
        // A lease can be destroyed on the wrong thread without touching OCAF.
        // Its expired lifetime lets this main-thread retry recover ownership.
        _activeToken = 0;
        _state = OrdinaryEditState::OutcomeUnknown;
    }
    _reconciling = true;
    struct ReconcileReset { bool& flag; ~ReconcileReset() { flag = false; } } reset{_reconciling};
    return reconcileImpl();
}

OrdinaryEditResult OrdinaryEditController::reconcileImpl() noexcept {
    if (std::holds_alternative<OrdinaryGroupingLedger>(*_pending)) { return reconcileGroupingImpl(); }
    if (std::holds_alternative<OrdinaryVisibilityLedger>(*_pending)) { return reconcileVisibilityImpl(); }
    if (std::holds_alternative<OrdinaryNameLedger>(*_pending)) { return reconcileNamesImpl(); }
    try {
        auto& ledger = std::get<OrdinaryTransformLedger>(*_pending);
#ifdef DEBUG
        if (_truthUnavailableCount > 0) {
            --_truthUnavailableCount;
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
#endif
        auto observation = _command.observe();
        if (observation == OrdinaryCommandObservation::OpenOwned) { observation = _command.abortAndObserve(); }
        const bool candidate = observation == OrdinaryCommandObservation::ClosedWithCandidateMarker;
        const bool previous = observation == OrdinaryCommandObservation::ClosedWithPriorMarker;
        if ((!candidate && !previous) || (candidate && !ledger.candidateSealed)) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        for (const auto& record : ledger.records) {
            if (!captureMatches(candidate ? record.candidate : record.previous)) {
                _state = OrdinaryEditState::OutcomeUnknown;
                return OrdinaryEditResult::OutcomeUnknown;
            }
        }
        _committed = candidate;
        _state = OrdinaryEditState::RepairPending;
        if (!_host.repairTransform(ledger, candidate) || !presentationMatches(ledger, candidate)) {
            std::vector<Handle(AIS_Shape)> replacements;
            if (!_host.rebuildTransform(ledger, candidate, replacements)
                || replacements.size() != ledger.records.size()) { return OrdinaryEditResult::OutcomeUnknown; }
            auto repaired = ledger;
            std::unordered_set<const AIS_Shape*> unique;
            for (std::size_t index = 0; index < replacements.size(); ++index) {
                if (replacements[index].IsNull() || !unique.insert(replacements[index].get()).second) {
                    return OrdinaryEditResult::OutcomeUnknown;
                }
                repaired.records[index].requested.presentation = replacements[index];
            }
            if (!presentationMatches(repaired, candidate)) { return OrdinaryEditResult::OutcomeUnknown; }
            for (std::size_t index = 0; index < replacements.size(); ++index) {
                ledger.records[index].requested.presentation = replacements[index];
            }
        }
        // Retain Publishing through synchronous observers. Host/observer
        // reentrancy remains Busy until this entire method has returned.
        _state = OrdinaryEditState::Publishing;
        if (!_command.releaseClosed()) {
            _state = OrdinaryEditState::OutcomeUnknown;
            return OrdinaryEditResult::OutcomeUnknown;
        }
        if (candidate && !_didPublish) {
            _didPublish = true;
            try { _document->NotifyChanges(); } catch (...) {}
        }
        clearResolved();
        return candidate ? OrdinaryEditResult::Committed : OrdinaryEditResult::RetryableFailure;
    } catch (...) {
        _state = OrdinaryEditState::OutcomeUnknown;
        return OrdinaryEditResult::OutcomeUnknown;
    }
}

} // namespace core3d
