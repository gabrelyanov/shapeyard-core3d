#ifndef OrdinaryEditCommand_hpp
#define OrdinaryEditCommand_hpp

#include "OcctDocument.h"
#include <cstdint>

namespace core3d {

//! These observations prove command ownership/closure only. A typed edit
//! ledger must separately prove all previous/candidate geometry and metadata.
enum class OrdinaryCommandObservation : std::uint8_t {
    Unavailable = 0,
    OpenOwned,
    ClosedWithPriorMarker,
    ClosedWithCandidateMarker,
};

enum class OrdinaryCommandBeginResult : std::uint8_t {
    Started = 0,
    RetryableFailure,
    OutcomeUnknown,
    Busy,
    Invalid,
};

//! Main-thread command stamp retained by the ordinary-edit coordinator. No
//! callbacks, implicit abort on destruction, or geometry mutation. In
//! particular, an unresolved stamp must remain owned by the typed ledger.
//! Copy/move are disabled so command authority cannot be duplicated or lost.
class Standard_EXPORT OrdinaryEditCommandStamp final {
public:
    OrdinaryEditCommandStamp() = default;
    OrdinaryEditCommandStamp(const OrdinaryEditCommandStamp&) = delete;
    OrdinaryEditCommandStamp& operator=(const OrdinaryEditCommandStamp&) = delete;
    OrdinaryEditCommandStamp(OrdinaryEditCommandStamp&&) = delete;
    OrdinaryEditCommandStamp& operator=(OrdinaryEditCommandStamp&&) = delete;

    OrdinaryCommandBeginResult begin(const Handle(OcctDocument)& document) noexcept;
    OrdinaryCommandObservation observe() noexcept;
    OrdinaryCommandObservation commitAndObserve() noexcept;
    OrdinaryCommandObservation abortAndObserve() noexcept;
    //! Only after the owning typed ledger has reconciled its durable outcome.
    //! Refuses to drop an open, foreign, or unavailable command stamp.
    bool releaseClosed() noexcept;
    bool isRetained() const noexcept { return _retained; }

#ifdef DEBUG
    //! One-shot faults: New 1 no open, 2 throw before, 3 throw after open;
    //! Commit 1 false/open, 2 throw/open, 3 false/closed, 4 throw/closed;
    //! Abort 1 fail/open, 2 throw/closed. Inspection failures never grant abort.
    void debugSetNewCommandMode(int mode) noexcept { _newMode = mode; }
    void debugSetCommitMode(int mode) noexcept { _commitMode = mode; }
    void debugSetAbortMode(int mode) noexcept { _abortMode = mode; }
    void debugSetInspectionFailureCount(int count) noexcept { _inspectionFailures = count; }
    void debugSetPostCommitInspectionFailureCount(int count) noexcept { _postCommitInspectionFailures = count; }
#endif

private:
    bool identityIsCurrent() const;
    bool ownedCommandIsCurrent() const;
    bool readMarker(bool& present, Standard_Integer& value) const;
    void abortExactDocument();
    OrdinaryCommandBeginResult cleanUpUnprovenBegin() noexcept;
    void clear() noexcept;

    Handle(OcctDocument) _owner;
    Handle(TDocStd_Document) _document;
    Handle(TDF_Data) _data;
    std::string _documentIdentifier;
    Standard_Integer _transaction = -1;
    Standard_Integer _time = -1;
    Standard_Integer _priorMarker = 0;
    Standard_Integer _candidateMarker = 0;
    bool _priorPresent = false;
    bool _retained = false;
    bool _proven = false;
#ifdef DEBUG
    int _newMode = 0;
    int _commitMode = 0;
    int _abortMode = 0;
    int _inspectionFailures = 0;
    int _postCommitInspectionFailures = 0;
#endif
};

} // namespace core3d
#endif
