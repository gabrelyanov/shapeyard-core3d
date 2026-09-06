#ifndef OrdinaryEditController_hpp
#define OrdinaryEditController_hpp

#include "OrdinaryEditCommand.hpp"
#include <memory>
#include <optional>
#include <variant>
#include <vector>

namespace core3d {

enum class OrdinaryEditKind : std::uint8_t { Transform, Add, Remove, Appearance };
enum class OrdinaryEditState : std::uint8_t { Idle, OpenOwned, OutcomeUnknown, RepairPending, Publishing };
enum class OrdinaryEditResult : std::uint8_t { NoChange, Committed, RetryableFailure, OutcomeUnknown, Busy, Invalid };
enum class OrdinaryTransformOperation : std::uint8_t { Translate, Rotate, Scale };

struct OrdinaryTransformChange {
    TDF_Label label;
    Handle(AIS_Shape) presentation;
    TopoDS_Shape shape;
    gp_Trsf transform;
    OrdinaryTransformOperation operation = OrdinaryTransformOperation::Translate;
};

struct OrdinaryTransformRecord {
    OcctObjectTransformState previous;
    OcctObjectTransformState candidate;
    OrdinaryTransformChange requested;
};

struct OrdinaryTransformLedger {
    std::vector<OrdinaryTransformRecord> records;
    bool candidateSealed = false;
};

//! Typed presentation boundary. The viewer implements admission/repair for
//! exact selection, tool, manipulator and renderer identity. Neither method
//! may mutate OCAF or call NotifyChanges. No captured callbacks in a ledger.
class OrdinaryEditPresentationHost {
public:
    virtual ~OrdinaryEditPresentationHost() = default;
    virtual bool admitTransform(const OrdinaryTransformLedger& ledger) noexcept = 0;
    virtual bool repairTransform(const OrdinaryTransformLedger& ledger, bool committed) noexcept = 0;
};

class OrdinaryEditController;

//! Move-only authority for one synchronous stage/close interval. Destruction
//! requests owned abort/reconciliation; unknown truth remains in the controller.
class Standard_EXPORT OrdinaryEditLease final {
public:
    OrdinaryEditLease() = default;
    OrdinaryEditLease(OrdinaryEditLease&& other) noexcept;
    OrdinaryEditLease& operator=(OrdinaryEditLease&& other) noexcept;
    ~OrdinaryEditLease() noexcept;
    OrdinaryEditLease(const OrdinaryEditLease&) = delete;
    OrdinaryEditLease& operator=(const OrdinaryEditLease&) = delete;
    explicit operator bool() const noexcept { return _token != 0 && !_controller.expired(); }
    OrdinaryEditResult stageAndCommit() noexcept;
    OrdinaryEditResult cancel() noexcept;
private:
    friend class OrdinaryEditController;
    OrdinaryEditLease(std::weak_ptr<OrdinaryEditController> controller, std::uint64_t token,
                      std::shared_ptr<const std::uint8_t> lifetime) noexcept;
    std::weak_ptr<OrdinaryEditController> _controller;
    std::shared_ptr<const std::uint8_t> _lifetime;
    std::uint64_t _token = 0;
};

//! Shared durable state machine. Transform is the first typed family; Add,
//! Remove and Appearance extend PendingEdit when their exact snapshots exist.
//! The owning viewer must retain this controller and gate normal work with
//! blocksNormalWork(), including the synchronous Publishing interval.
class Standard_EXPORT OrdinaryEditController final
    : public std::enable_shared_from_this<OrdinaryEditController> {
public:
    OrdinaryEditController(Handle(OcctDocument) document, OrdinaryEditPresentationHost& host);
    OrdinaryEditController(const OrdinaryEditController&) = delete;
    OrdinaryEditController& operator=(const OrdinaryEditController&) = delete;
    OrdinaryEditLease beginTransform(const std::vector<OrdinaryTransformChange>& changes,
                                     OrdinaryEditResult* failure = nullptr) noexcept;
    OrdinaryEditResult reconcile() noexcept;
    bool blocksNormalWork() const noexcept;
    OrdinaryEditState state() const noexcept { return _state; }
#ifdef DEBUG
    OrdinaryEditCommandStamp& debugCommandStamp() noexcept { return _command; }
    void debugSetStageFailureIndex(int index) noexcept { _stageFailureIndex = index; }
    void debugSetTruthUnavailableCount(int count) noexcept { _truthUnavailableCount = count; }
#endif
private:
    friend class OrdinaryEditLease;
    using PendingEdit = std::variant<OrdinaryTransformLedger>;
    OrdinaryEditResult stageAndCommit(std::uint64_t token) noexcept;
    OrdinaryEditResult cancel(std::uint64_t token) noexcept;
    OrdinaryEditResult reconcileImpl() noexcept;
    bool captureMatches(const OcctObjectTransformState& expected) const noexcept;
    bool presentationMatches(const OrdinaryTransformLedger& ledger, bool committed) const noexcept;
    void clearResolved() noexcept;

    Handle(OcctDocument) _document;
    OrdinaryEditPresentationHost& _host;
    OrdinaryEditCommandStamp _command;
    std::optional<PendingEdit> _pending;
    OrdinaryEditState _state = OrdinaryEditState::Idle;
    std::uint64_t _nextToken = 0;
    std::uint64_t _activeToken = 0;
    std::weak_ptr<const std::uint8_t> _leaseLifetime;
    bool _entering = false;
    bool _reconciling = false;
    bool _committed = false;
    bool _didPublish = false;
#ifdef DEBUG
    int _stageFailureIndex = -1;
    int _truthUnavailableCount = 0;
#endif
};

} // namespace core3d
#endif
