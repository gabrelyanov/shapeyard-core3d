#include "OrdinaryEditCommand.hpp"

#import <Foundation/Foundation.h>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TDataStd_Integer.hxx>
#include <TDF_Attribute.hxx>
#include <limits>
#include <utility>

namespace core3d {
namespace {
// No callbacks or suspension inside begin. This also rejects a nested begin
// through another stamp before the first command has established its sentinel.
bool gOrdinaryCommandBeginActive = false;
}

bool OrdinaryEditCommandStamp::identityIsCurrent() const
{
    return _retained && !_owner.IsNull() && !_document.IsNull()
        && !_data.IsNull() && !_documentIdentifier.empty()
        && _owner->Document() == _document
        && _document->GetData() == _data
        && _owner->DocumentIdentifier() == _documentIdentifier;
}

bool OrdinaryEditCommandStamp::readMarker(
    bool& present, Standard_Integer& value) const
{
    Handle(TDF_Attribute) attribute;
    present = _document->Main().FindAttribute(
        Core3DOrdinaryEditCommandOwnerAttributeID(), attribute);
    value = 0;
    if (!present) { return true; }
    const Handle(TDataStd_Integer) marker =
        Handle(TDataStd_Integer)::DownCast(attribute);
    if (marker.IsNull()) { return false; }
    value = marker->Get();
    return true;
}

bool OrdinaryEditCommandStamp::ownedCommandIsCurrent() const
{
    bool present = false;
    Standard_Integer value = 0;
    return _proven && identityIsCurrent() && _document->HasOpenCommand()
        && _transaction > 0 && _data->Transaction() == _transaction
        && _data->Time() == _time && readMarker(present, value)
        && present && value == _candidateMarker;
}

void OrdinaryEditCommandStamp::clear() noexcept
{
    _proven = false;
    _retained = false;
    _owner.Nullify();
    _document.Nullify();
    _data.Nullify();
    _documentIdentifier.clear();
    _transaction = -1;
    _time = -1;
    _priorMarker = 0;
    _candidateMarker = 0;
    _priorPresent = false;
}

void OrdinaryEditCommandStamp::abortExactDocument()
{
#ifdef DEBUG
    const int mode = std::exchange(_abortMode, 0);
    if (mode == 1) { throw Standard_Failure("Ordinary abort before close"); }
#endif
    _document->AbortCommand();
#ifdef DEBUG
    if (mode == 2) { throw Standard_Failure("Ordinary abort after close"); }
#endif
}

OrdinaryCommandBeginResult OrdinaryEditCommandStamp::cleanUpUnprovenBegin() noexcept
{
    // Called only inside our synchronous, non-reentrant begin interval. Arm
    // this path before NewCommand: that call can throw AFTER opening. Never
    // put a fallible HasOpenCommand check before this direct cleanup attempt.
    try { abortExactDocument(); } catch (...) {}
    _proven = false;
    if (observe() == OrdinaryCommandObservation::ClosedWithPriorMarker) {
        clear();
        return OrdinaryCommandBeginResult::RetryableFailure;
    }
    return OrdinaryCommandBeginResult::OutcomeUnknown;
}

OrdinaryCommandBeginResult OrdinaryEditCommandStamp::begin(
    const Handle(OcctDocument)& document) noexcept
{
    if (![NSThread isMainThread]) { return OrdinaryCommandBeginResult::Invalid; }
    if (_retained || gOrdinaryCommandBeginActive) {
        return OrdinaryCommandBeginResult::Busy;
    }
    try {
        if (document.IsNull() || document->Document().IsNull()) {
            return OrdinaryCommandBeginResult::Invalid;
        }
        const Handle(TDocStd_Document) ocaf = document->Document();
        if (ocaf->HasOpenCommand()) { return OrdinaryCommandBeginResult::Busy; }
        const Handle(TDF_Data) data = ocaf->GetData();
        const std::string identifier = document->DocumentIdentifier();
        if (data.IsNull() || identifier.empty()) {
            return OrdinaryCommandBeginResult::Invalid;
        }
        _owner = document;
        _document = ocaf;
        _data = data;
        _documentIdentifier = identifier;
        _retained = true;
        if (!readMarker(_priorPresent, _priorMarker)) {
            clear();
            return OrdinaryCommandBeginResult::Invalid;
        }
        _candidateMarker = _priorMarker == std::numeric_limits<Standard_Integer>::max()
            ? std::numeric_limits<Standard_Integer>::min() : _priorMarker + 1;
    } catch (...) {
        // No NewCommand has run, so there is no abort authority or mutation.
        clear();
        return OrdinaryCommandBeginResult::Invalid;
    }

    struct BeginInterval {
        BeginInterval() { gOrdinaryCommandBeginActive = true; }
        ~BeginInterval() { gOrdinaryCommandBeginActive = false; }
    } interval;
    try {
        OCC_CATCH_SIGNALS
#ifdef DEBUG
        const int mode = std::exchange(_newMode, 0);
        if (mode == 2) { throw Standard_Failure("Ordinary begin before open"); }
        if (mode != 1) { _document->NewCommand(); }
        if (mode == 3) { throw Standard_Failure("Ordinary begin after open"); }
#else
        _document->NewCommand();
#endif
        if (!_document->HasOpenCommand() || !identityIsCurrent()
            || _data->Transaction() <= 0) {
            return cleanUpUnprovenBegin();
        }
        _transaction = _data->Transaction();
        _time = _data->Time();
        TDataStd_Integer::Set(_document->Main(),
            Core3DOrdinaryEditCommandOwnerAttributeID(), _candidateMarker);
        _proven = true;
        if (!ownedCommandIsCurrent()) {
            _proven = false;
            return cleanUpUnprovenBegin();
        }
        return OrdinaryCommandBeginResult::Started;
    } catch (...) {
        // No callback ran and ownership was not handed to the caller. Direct
        // cleanup is limited to this interval, including throw-after-open.
        return cleanUpUnprovenBegin();
    }
}

OrdinaryCommandObservation OrdinaryEditCommandStamp::observe() noexcept
{
    if (![NSThread isMainThread]) { return OrdinaryCommandObservation::Unavailable; }
    try {
        OCC_CATCH_SIGNALS
#ifdef DEBUG
        if (_inspectionFailures > 0) {
            --_inspectionFailures;
            return OrdinaryCommandObservation::Unavailable;
        }
#endif
        if (!identityIsCurrent()) {
            _proven = false;
            return OrdinaryCommandObservation::Unavailable;
        }
        if (_document->HasOpenCommand()) {
            if (ownedCommandIsCurrent()) { return OrdinaryCommandObservation::OpenOwned; }
            // A proven mismatch revokes authority permanently. Restoring a
            // marker later cannot re-authorize abort of a foreign command.
            _proven = false;
            return OrdinaryCommandObservation::Unavailable;
        }
        // Closure destroys abort authority even if reading the marker fails.
        _proven = false;
        bool present = false;
        Standard_Integer value = 0;
        if (!readMarker(present, value)) { return OrdinaryCommandObservation::Unavailable; }
        if (present == _priorPresent && (!present || value == _priorMarker)) {
            return OrdinaryCommandObservation::ClosedWithPriorMarker;
        }
        if (present && value == _candidateMarker) {
            return OrdinaryCommandObservation::ClosedWithCandidateMarker;
        }
    } catch (...) {}
    return OrdinaryCommandObservation::Unavailable;
}

OrdinaryCommandObservation OrdinaryEditCommandStamp::commitAndObserve() noexcept
{
    const auto before = observe();
    if (before != OrdinaryCommandObservation::OpenOwned) { return before; }
    try {
        OCC_CATCH_SIGNALS
#ifdef DEBUG
        const int mode = std::exchange(_commitMode, 0);
        if (mode == 1) { return observe(); }
        if (mode == 2) { throw Standard_Failure("Ordinary commit before close"); }
#endif
        (void)_document->CommitCommand();
#ifdef DEBUG
        if (mode == 4) { throw Standard_Failure("Ordinary commit after close"); }
        // Mode3 represents false after a real close. The Boolean is never
        // used as an outcome, so the same authoritative observation follows.
#endif
    } catch (...) {}
#ifdef DEBUG
    _inspectionFailures += std::exchange(_postCommitInspectionFailures, 0);
#endif
    return observe();
}

OrdinaryCommandObservation OrdinaryEditCommandStamp::abortAndObserve() noexcept
{
    const auto before = observe();
    if (before != OrdinaryCommandObservation::OpenOwned) { return before; }
    try { abortExactDocument(); } catch (...) {}
    return observe();
}

bool OrdinaryEditCommandStamp::releaseClosed() noexcept
{
    const auto result = observe();
    if (result != OrdinaryCommandObservation::ClosedWithPriorMarker
        && result != OrdinaryCommandObservation::ClosedWithCandidateMarker) {
        return false;
    }
    clear();
    return true;
}

} // namespace core3d
