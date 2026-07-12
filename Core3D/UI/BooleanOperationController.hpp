//
//  BooleanOperationController.hpp
//  Core3D
//

#ifndef BooleanOperationController_hpp
#define BooleanOperationController_hpp

#include "AIS_InteractiveContext.hxx"
#include "OcctDocument.h"

#include <AIS_Shape.hxx>

#include <map>
#include <optional>
#include <vector>

namespace core3d {

enum BooleanAction {
    BooleanSubtract = 0,
    BooleanUnion,
};

enum BooleanSelectionType {
    Undefined = -1,
    Actor = 0,
    Subject,
};

enum class BooleanApplyResult {
    NoChange = 0,
    Applied,
    AppliedNeedsDocumentRedraw,
};

struct TemporalBooleanObject {
    Handle(AIS_InteractiveObject) original;
    BooleanSelectionType selectionType = BooleanSelectionType::Undefined;
    TDF_Label documentLabel;
    Graphic3d_NameOfMaterial materialName =
        Graphic3d_NameOfMaterial_ShinyPlastified;
    Quantity_NameOfColor colorName = Quantity_NOC_GRAY80;
};

//! Explicit, controller-owned publication input. The snapshot builder receives
//! only these handles and source labels; it never enumerates AIS context.
struct BooleanPreviewCapture {
    BooleanAction action = BooleanAction::BooleanSubtract;
    std::vector<Handle(AIS_Shape)> actors;
    std::vector<Handle(AIS_Shape)> results;
    std::vector<TDF_Label> suppressedSourceLabels;
};

class BooleanOperationController {
public:
    static constexpr std::size_t kMaxSourceOperands = 8;

    BooleanOperationController() = delete;
    BooleanOperationController(
        Handle(AIS_InteractiveContext),
        Handle(OcctDocument) doc);

    Standard_Boolean begin(BooleanAction action) noexcept;
    void updateDetectedState(
        Handle(AIS_InteractiveObject) detected,
        Handle(SelectMgr_EntityOwner) detectedOwner,
        Standard_Boolean forceActor,
        BooleanAction action) noexcept;
    Standard_Boolean setSelectionState(
        const Handle(AIS_InteractiveObject)& object,
        BooleanSelectionType type,
        BooleanAction action) noexcept;
    void visualApply(BooleanAction action) noexcept;
    BooleanApplyResult apply(BooleanAction action) noexcept;
    void cancel(BooleanAction action) noexcept;
    void cancelActive() noexcept;

    Standard_Boolean canApply() const noexcept;
    Standard_Boolean hasActiveOperation() const noexcept;
    Standard_Boolean hasSelectionState() const noexcept;
    Standard_Boolean hasUnresolvedState() const noexcept;
    Standard_Boolean isSelectionFrozen() const noexcept;
    Standard_Boolean capturePreview(
        BooleanPreviewCapture& capture) const noexcept;

private:
    Standard_Boolean actionMatches(BooleanAction action) const noexcept;
    Standard_Boolean resetCachedSelection() noexcept;
    Standard_Boolean buildSubtractPreview(
        const std::vector<Handle(AIS_InteractiveObject)>& actors,
        const std::vector<Handle(AIS_InteractiveObject)>& subjects) noexcept;
    Standard_Boolean buildUnionPreview(
        const std::vector<Handle(AIS_InteractiveObject)>& subjects) noexcept;
    Standard_Boolean cancelImpl() noexcept;
    void clearOperationState() noexcept;
    void markInvalid() noexcept;
    void rememberOwned(
        const Handle(AIS_InteractiveObject)& presentation) noexcept;
    Standard_Boolean cleanupOwnedPresentations() noexcept;
    Standard_Boolean pruneOwnedPresentations() noexcept;
    void rollbackFailedTransaction(
        const Handle(TDocStd_Document)& document) noexcept;

    void showInteractiveByType(
        const Handle(AIS_InteractiveObject)& shape,
        BooleanSelectionType type);
    void rememberSubjectSelection(const TDF_Label& label);
    void forgetSubjectSelection(const TDF_Label& label);
    std::vector<Handle(AIS_InteractiveObject)> orderedSubjectPresentations(
        Standard_Boolean& isComplete) const;
    std::vector<TDF_Label> orderedSourceLabels(
        Standard_Boolean& isComplete) const;
    void applyStyle(
        Handle(AIS_InteractiveObject)& object,
        const TemporalBooleanObject& style);
    void persistStyle(
        const TDF_Label& label,
        const TemporalBooleanObject& style);
    Handle(AIS_InteractiveObject) ioCopyWithStyle(
        const Handle(AIS_InteractiveObject)& original,
        const TemporalBooleanObject& style);

private:
    std::vector<Handle(AIS_InteractiveObject)> _actedIOArray;
    std::vector<Handle(AIS_InteractiveObject)> _actorIOArray;
    std::vector<TDF_Label> _subjectSelectionOrder;
    Handle(AIS_Shape) _unionTrialResult;
    std::vector<Handle(AIS_InteractiveObject)> _ownedPresentations;
    std::optional<BooleanAction> _activeAction;
    Standard_Boolean _canApply = Standard_False;
    Standard_Boolean _stateValid = Standard_True;
    Standard_Boolean _selectionFrozen = Standard_False;
    Standard_Boolean _documentCommandUnresolved = Standard_False;

    Handle(AIS_InteractiveContext) myContext;
    Handle(OcctDocument) myDoc;
    std::map<Handle(AIS_InteractiveObject), TemporalBooleanObject> _selectionMap;
};

} // namespace core3d

#endif // BooleanOperationController_hpp
