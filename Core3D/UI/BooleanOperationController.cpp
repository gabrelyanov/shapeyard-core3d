//
//  BooleanOperationController.cpp
//  Core3D
//

#include "BooleanOperationController.hpp"

#include <AIS_Shape.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>

#include <algorithm>
#include <iostream>
#include <string>
#include <unordered_set>
#include <utility>

namespace core3d {
namespace {

constexpr Standard_Real kFuzzyValue = 1.0e-3;

Standard_Boolean IsValidSolidBooleanResult(const TopoDS_Shape& theShape)
{
    if (theShape.IsNull()) {
        return Standard_False;
    }
    try {
        BRepCheck_Analyzer anAnalyzer(theShape, Standard_True);
        return anAnalyzer.IsValid()
            && TopExp_Explorer(theShape, TopAbs_SOLID).More();
    } catch (...) {
        return Standard_False;
    }
}

bool ContainsLabel(
    const std::vector<TDF_Label>& theLabels,
    const TDF_Label& theCandidate)
{
    return std::any_of(
        theLabels.begin(),
        theLabels.end(),
        [&](const TDF_Label& theLabel) {
            return theLabel.IsEqual(theCandidate);
        });
}

Standard_Boolean AbortCommandNoThrow(
    const Handle(TDocStd_Document)& theDocument) noexcept
{
    if (theDocument.IsNull()) {
        return Standard_False;
    }
    try {
        if (theDocument->HasOpenCommand()) {
            theDocument->AbortCommand();
        }
        return !theDocument->HasOpenCommand();
    } catch (...) {
        // AbortCommand may throw after closing the command. Re-check before
        // declaring the document unresolved so a recoverable failure does not
        // permanently poison the Boolean controller.
        try {
            return !theDocument->HasOpenCommand();
        } catch (...) {
            return Standard_False;
        }
    }
}

} // namespace

BooleanOperationController::BooleanOperationController(
    Handle(AIS_InteractiveContext) theContext,
    Handle(OcctDocument) theDocument)
: myContext(std::move(theContext)),
  myDoc(std::move(theDocument))
{
    try {
        _ownedPresentations.reserve(kMaxSourceOperands * 3);
        _subjectSelectionOrder.reserve(kMaxSourceOperands);
    } catch (...) {
        _stateValid = Standard_False;
    }
}

Standard_Boolean BooleanOperationController::begin(
    const BooleanAction theAction) noexcept
{
    try {
        if (myContext.IsNull() || myDoc.IsNull()) {
            return Standard_False;
        }
        if (_activeAction.has_value()) {
            if (*_activeAction == theAction
                && _selectionMap.empty() && _ownedPresentations.empty()
                && _stateValid) {
                return Standard_True;
            }
            if (!cancelImpl()) {
                return Standard_False;
            }
        } else if (!_selectionMap.empty() || !_ownedPresentations.empty()
                   || !_stateValid) {
            if (!cancelImpl()) {
                return Standard_False;
            }
        }
        _activeAction = theAction;
        _stateValid = Standard_True;
        _selectionFrozen = Standard_False;
        _canApply = Standard_False;
        return Standard_True;
    } catch (...) {
        markInvalid();
        return Standard_False;
    }
}

Standard_Boolean BooleanOperationController::actionMatches(
    const BooleanAction theAction) const noexcept
{
    return _activeAction.has_value() && *_activeAction == theAction;
}

Standard_Boolean BooleanOperationController::canApply() const noexcept
{
    return _canApply && _stateValid && _activeAction.has_value();
}

Standard_Boolean BooleanOperationController::hasActiveOperation() const noexcept
{
    return _activeAction.has_value();
}

Standard_Boolean BooleanOperationController::hasSelectionState() const noexcept
{
    return !_selectionMap.empty() || !_unionTrialResult.IsNull();
}

Standard_Boolean BooleanOperationController::hasUnresolvedState() const noexcept
{
    return !_stateValid || _documentCommandUnresolved;
}

Standard_Boolean BooleanOperationController::isSelectionFrozen() const noexcept
{
    return _selectionFrozen && canApply()
        && actionMatches(BooleanAction::BooleanUnion);
}

void BooleanOperationController::markInvalid() noexcept
{
    _stateValid = Standard_False;
    _canApply = Standard_False;
    _selectionFrozen = Standard_True;
}

void BooleanOperationController::rememberOwned(
    const Handle(AIS_InteractiveObject)& thePresentation) noexcept
{
    if (thePresentation.IsNull()) {
        markInvalid();
        return;
    }
    for (const Handle(AIS_InteractiveObject)& anOwned :
         _ownedPresentations) {
        if (anOwned == thePresentation) {
            return;
        }
    }
    // A Subtract recompute temporarily owns the current N previews, N fresh
    // source copies, and as many as N - 1 replacement results before the old
    // presentations can be pruned. Keep that whole fail-closed transaction
    // inside the capacity reserved by the constructor.
    if (_ownedPresentations.size() >= kMaxSourceOperands * 3) {
        markInvalid();
        return;
    }
    try {
        _ownedPresentations.push_back(thePresentation);
    } catch (...) {
        markInvalid();
    }
}

Standard_Boolean BooleanOperationController::cleanupOwnedPresentations() noexcept
{
    std::vector<Handle(AIS_InteractiveObject)> anUnresolved;
    try {
        anUnresolved.reserve(_ownedPresentations.size());
    } catch (...) {
        markInvalid();
        return Standard_False;
    }
    for (const Handle(AIS_InteractiveObject)& anOwned :
         _ownedPresentations) {
        if (anOwned.IsNull()) {
            continue;
        }
        try {
            myContext->Remove(anOwned, Standard_False);
        } catch (...) {
            anUnresolved.push_back(anOwned);
        }
    }
    _ownedPresentations = std::move(anUnresolved);
    return _ownedPresentations.empty();
}

Standard_Boolean BooleanOperationController::pruneOwnedPresentations() noexcept
{
    std::vector<Handle(AIS_InteractiveObject)> aRetained;
    try {
        // A failed Remove must also fit in this vector. Reserving only the live
        // count could make the catch-path allocation throw from this noexcept
        // function and terminate the process.
        aRetained.reserve(_ownedPresentations.size());
    } catch (...) {
        markInvalid();
        return Standard_False;
    }
    bool didPruneAll = true;
    for (const Handle(AIS_InteractiveObject)& anOwned :
         _ownedPresentations) {
        bool isLive = !anOwned.IsNull()
            && (!_unionTrialResult.IsNull()
                && anOwned == _unionTrialResult);
        if (!isLive) {
            isLive = _selectionMap.find(anOwned) != _selectionMap.end();
        }
        if (isLive) {
            aRetained.push_back(anOwned);
            continue;
        }
        try {
            if (!anOwned.IsNull()) {
                myContext->Remove(anOwned, Standard_False);
            }
        } catch (...) {
            aRetained.push_back(anOwned);
            didPruneAll = false;
        }
    }
    _ownedPresentations = std::move(aRetained);
    if (!didPruneAll) {
        markInvalid();
    }
    return didPruneAll;
}

void BooleanOperationController::rollbackFailedTransaction(
    const Handle(TDocStd_Document)& theDocument) noexcept
{
    const Standard_Boolean didAbort = AbortCommandNoThrow(theDocument);
    cancelImpl();
    if (!didAbort) {
        // Visual state can still be restored, but keep the controller
        // unresolved until a later cancel retries and confirms that OCAF no
        // longer has the Boolean command open.
        _documentCommandUnresolved = Standard_True;
        markInvalid();
    }
}

void BooleanOperationController::updateDetectedState(
    Handle(AIS_InteractiveObject) theDetected,
    Handle(SelectMgr_EntityOwner) theDetectedOwner,
    const Standard_Boolean theForceActor,
    const BooleanAction theAction) noexcept
{
    if (!actionMatches(theAction) || !_stateValid || _selectionFrozen
        || theDetected.IsNull()) {
        return;
    }

    try {
        TemporalBooleanObject aState;
        auto anExisting = _selectionMap.find(theDetected);
        if (anExisting == _selectionMap.end()) {
            if (_selectionMap.size() >= kMaxSourceOperands) {
                _canApply = Standard_False;
                return;
            }
            const Handle(AIS_Shape) aShape =
                Handle(AIS_Shape)::DownCast(theDetected);
            if (aShape.IsNull() || aShape->Shape().IsNull()) {
                return;
            }
            aState.documentLabel = myDoc->ShapeLabel(theDetected);
            if (aState.documentLabel.IsNull()) {
                return;
            }
            for (const auto& aSelection : _selectionMap) {
                if (aSelection.second.documentLabel.IsEqual(
                        aState.documentLabel)) {
                    return;
                }
            }
            aState.original = theDetected;
            aState.materialName =
                myDoc->MaterialNameForLabel(aState.documentLabel);
            aState.colorName =
                myDoc->ColorNameForLabel(aState.documentLabel);
        } else {
            aState = anExisting->second;
        }

        const Standard_Integer aDisplayMode = theDetected->DisplayMode();
        if (theAction == BooleanAction::BooleanSubtract) {
            if (aDisplayMode == -1 && theForceActor) {
                aState.selectionType = BooleanSelectionType::Actor;
            } else if (aDisplayMode == AIS_WireFrame || !theForceActor) {
                aState.selectionType = BooleanSelectionType::Subject;
            } else {
                aState.selectionType = BooleanSelectionType::Undefined;
            }
        } else {
            aState.selectionType = aDisplayMode == -1
                ? BooleanSelectionType::Subject
                : BooleanSelectionType::Undefined;
        }

        if (aState.selectionType == BooleanSelectionType::Undefined) {
            if (anExisting != _selectionMap.end()) {
                const Handle(AIS_InteractiveObject) anOriginal =
                    anExisting->second.original;
                forgetSubjectSelection(anExisting->second.documentLabel);
                myContext->Remove(theDetected, Standard_False);
                showInteractiveByType(
                    anOriginal,
                    BooleanSelectionType::Undefined);
                _selectionMap.erase(anExisting);
            }
        } else {
            _selectionMap[theDetected] = aState;
            if (aState.selectionType == BooleanSelectionType::Subject) {
                rememberSubjectSelection(aState.documentLabel);
            } else {
                forgetSubjectSelection(aState.documentLabel);
            }
            showInteractiveByType(theDetected, aState.selectionType);
            if (!theDetectedOwner.IsNull()
                && !theDetectedOwner->IsSelected()) {
                myContext->SetSelectedState(
                    theDetectedOwner,
                    Standard_False);
            }
        }
    } catch (...) {
        markInvalid();
    }
}

Standard_Boolean BooleanOperationController::setSelectionState(
    const Handle(AIS_InteractiveObject)& theObject,
    const BooleanSelectionType theType,
    const BooleanAction theAction) noexcept
{
    if (!actionMatches(theAction) || !_stateValid || _selectionFrozen
        || theObject.IsNull()
        || (theType != BooleanSelectionType::Actor
            && theType != BooleanSelectionType::Subject)
        || (theAction == BooleanAction::BooleanUnion
            && theType != BooleanSelectionType::Subject)
        || _selectionMap.size() >= kMaxSourceOperands) {
        return Standard_False;
    }
    try {
        const Handle(AIS_Shape) aShape =
            Handle(AIS_Shape)::DownCast(theObject);
        const TDF_Label aLabel = myDoc->ShapeLabel(theObject);
        if (aShape.IsNull() || aShape->Shape().IsNull()
            || aLabel.IsNull()) {
            return Standard_False;
        }
        for (const auto& aSelection : _selectionMap) {
            if (aSelection.second.documentLabel.IsEqual(aLabel)) {
                return Standard_False;
            }
        }
        TemporalBooleanObject aState;
        aState.original = theObject;
        aState.documentLabel = aLabel;
        aState.selectionType = theType;
        aState.materialName = myDoc->MaterialNameForLabel(aLabel);
        aState.colorName = myDoc->ColorNameForLabel(aLabel);
        _selectionMap.emplace(theObject, aState);
        if (theType == BooleanSelectionType::Subject) {
            rememberSubjectSelection(aLabel);
        }
        showInteractiveByType(theObject, theType);
        return Standard_True;
    } catch (...) {
        markInvalid();
        return Standard_False;
    }
}

Standard_Boolean BooleanOperationController::resetCachedSelection() noexcept
{
    if (!_stateValid || _selectionMap.empty()) {
        return _stateValid;
    }

    try {
        std::map<Handle(AIS_InteractiveObject), TemporalBooleanObject>
            aReplacement;
        std::vector<std::pair<Handle(AIS_InteractiveObject),
                              Handle(AIS_InteractiveObject)>> aSwaps;
        aSwaps.reserve(_selectionMap.size());
        for (const auto& aSelection : _selectionMap) {
            Handle(AIS_InteractiveObject) aCopy = ioCopyWithStyle(
                aSelection.second.original,
                aSelection.second);
            if (aCopy.IsNull()) {
                return Standard_False;
            }
            rememberOwned(aCopy);
            if (!_stateValid) {
                return Standard_False;
            }
            aReplacement.emplace(aCopy, aSelection.second);
            aSwaps.push_back({aSelection.first, aCopy});
        }

        for (const auto& aSwap : aSwaps) {
            const auto aReplacementState = aReplacement.find(aSwap.second);
            if (aReplacementState == aReplacement.end()) {
                markInvalid();
                return Standard_False;
            }
            showInteractiveByType(
                aSwap.second,
                aReplacementState->second.selectionType);
            myContext->Remove(aSwap.first, Standard_False);
        }
        _selectionMap.swap(aReplacement);
        return pruneOwnedPresentations();
    } catch (...) {
        markInvalid();
        return Standard_False;
    }
}

void BooleanOperationController::rememberSubjectSelection(
    const TDF_Label& theLabel)
{
    if (!theLabel.IsNull()
        && !ContainsLabel(_subjectSelectionOrder, theLabel)) {
        _subjectSelectionOrder.push_back(theLabel);
    }
}

void BooleanOperationController::forgetSubjectSelection(
    const TDF_Label& theLabel)
{
    const auto aFound = std::find_if(
        _subjectSelectionOrder.begin(),
        _subjectSelectionOrder.end(),
        [&](const TDF_Label& aLabel) {
            return aLabel.IsEqual(theLabel);
        });
    if (aFound != _subjectSelectionOrder.end()) {
        _subjectSelectionOrder.erase(aFound);
    }
}

std::vector<Handle(AIS_InteractiveObject)>
BooleanOperationController::orderedSubjectPresentations(
    Standard_Boolean& theIsComplete) const
{
    std::vector<Handle(AIS_InteractiveObject)> anOrdered;
    std::size_t aSubjectCount = 0;
    for (const auto& aSelection : _selectionMap) {
        if (aSelection.second.selectionType
            == BooleanSelectionType::Subject) {
            ++aSubjectCount;
        }
    }
    anOrdered.reserve(aSubjectCount);
    theIsComplete = Standard_True;
    for (const TDF_Label& aLabel : _subjectSelectionOrder) {
        Handle(AIS_InteractiveObject) aMatch;
        for (const auto& aSelection : _selectionMap) {
            if (aSelection.second.selectionType
                    != BooleanSelectionType::Subject
                || !aSelection.second.documentLabel.IsEqual(aLabel)) {
                continue;
            }
            if (!aMatch.IsNull()) {
                theIsComplete = Standard_False;
                return {};
            }
            aMatch = aSelection.first;
        }
        if (aMatch.IsNull()) {
            theIsComplete = Standard_False;
            return {};
        }
        anOrdered.push_back(aMatch);
    }
    if (anOrdered.size() != aSubjectCount) {
        theIsComplete = Standard_False;
        return {};
    }
    return anOrdered;
}

std::vector<TDF_Label> BooleanOperationController::orderedSourceLabels(
    Standard_Boolean& theIsComplete) const
{
    std::vector<std::pair<std::string, TDF_Label>> anActors;
    std::vector<TDF_Label> aSubjects;
    std::unordered_set<std::string> aSourceIdentifiers;
    theIsComplete = Standard_True;
    try {
        for (const auto& aSelection : _selectionMap) {
            const TDF_Label& aLabel = aSelection.second.documentLabel;
            const std::string anIdentifier =
                myDoc->EntityIdentifierForLabel(aLabel);
            if (aLabel.IsNull() || anIdentifier.empty()) {
                theIsComplete = Standard_False;
                return {};
            }
            if (!aSourceIdentifiers.insert(anIdentifier).second) {
                theIsComplete = Standard_False;
                return {};
            }
            if (aSelection.second.selectionType
                == BooleanSelectionType::Actor) {
                anActors.push_back({anIdentifier, aLabel});
            }
        }
        std::sort(
            anActors.begin(),
            anActors.end(),
            [](const auto& theLeft, const auto& theRight) {
                return theLeft.first < theRight.first;
            });
        for (const TDF_Label& aLabel : _subjectSelectionOrder) {
            bool hasMatch = false;
            for (const auto& aSelection : _selectionMap) {
                if (aSelection.second.selectionType
                        == BooleanSelectionType::Subject
                    && aSelection.second.documentLabel.IsEqual(aLabel)) {
                    if (hasMatch) {
                        theIsComplete = Standard_False;
                        return {};
                    }
                    hasMatch = true;
                }
            }
            if (!hasMatch) {
                theIsComplete = Standard_False;
                return {};
            }
            aSubjects.push_back(aLabel);
        }
        std::vector<TDF_Label> aResult;
        aResult.reserve(anActors.size() + aSubjects.size());
        for (const auto& anActor : anActors) {
            aResult.push_back(anActor.second);
        }
        aResult.insert(aResult.end(), aSubjects.begin(), aSubjects.end());
        if (aResult.size() != _selectionMap.size()) {
            theIsComplete = Standard_False;
            return {};
        }
        return aResult;
    } catch (...) {
        theIsComplete = Standard_False;
        return {};
    }
}

void BooleanOperationController::visualApply(
    const BooleanAction theAction) noexcept
{
    if (!actionMatches(theAction) || !_stateValid) {
        _canApply = Standard_False;
        return;
    }
    if (_selectionFrozen) {
        return;
    }
    _canApply = Standard_False;
    if (_selectionMap.empty()
        || _selectionMap.size() > kMaxSourceOperands
        || !resetCachedSelection()) {
        return;
    }

    try {
        _actedIOArray.clear();
        _actorIOArray.clear();
        for (const auto& aSelection : _selectionMap) {
            if (aSelection.second.documentLabel.IsNull()) {
                return;
            }
            if (aSelection.second.selectionType
                == BooleanSelectionType::Actor) {
                _actorIOArray.push_back(aSelection.first);
            }
        }
        Standard_Boolean hasCompleteSubjects = Standard_False;
        _actedIOArray = orderedSubjectPresentations(hasCompleteSubjects);
        if (!hasCompleteSubjects) {
            return;
        }

        if (theAction == BooleanAction::BooleanSubtract) {
            if (_actorIOArray.empty() || _actedIOArray.empty()
                || _actorIOArray.size() + _actedIOArray.size()
                    > kMaxSourceOperands) {
                return;
            }
            _canApply = buildSubtractPreview(
                _actorIOArray,
                _actedIOArray);
        } else {
            if (!_actorIOArray.empty() || _actedIOArray.size() < 2
                || _actedIOArray.size() > kMaxSourceOperands) {
                return;
            }
            _canApply = buildUnionPreview(_actedIOArray);
        }
        if (_stateValid && !pruneOwnedPresentations()) {
            _canApply = Standard_False;
        }
    } catch (...) {
        markInvalid();
    }
}

Standard_Boolean BooleanOperationController::buildSubtractPreview(
    const std::vector<Handle(AIS_InteractiveObject)>& theActors,
    const std::vector<Handle(AIS_InteractiveObject)>& theSubjects) noexcept
{
    struct Preview {
        Handle(AIS_InteractiveObject) source;
        Handle(AIS_InteractiveObject) result;
        TemporalBooleanObject state;
    };

    try {
        TopTools_ListOfShape aTools;
        for (const Handle(AIS_InteractiveObject)& anActorIO : theActors) {
            const Handle(AIS_Shape) anActor =
                Handle(AIS_Shape)::DownCast(anActorIO);
            if (anActor.IsNull() || anActor->Shape().IsNull()) {
                return Standard_False;
            }
            aTools.Append(BRepBuilderAPI_Transform(
                anActor->Shape(),
                anActorIO->LocalTransformation()).Shape());
        }

        std::vector<Preview> aPreviews;
        aPreviews.reserve(theSubjects.size());
        for (const Handle(AIS_InteractiveObject)& aSubjectIO : theSubjects) {
            const auto aSelection = _selectionMap.find(aSubjectIO);
            const Handle(AIS_Shape) aSubject =
                Handle(AIS_Shape)::DownCast(aSubjectIO);
            if (aSelection == _selectionMap.end()
                || aSelection->second.documentLabel.IsNull()
                || aSubject.IsNull() || aSubject->Shape().IsNull()) {
                return Standard_False;
            }

            TopTools_ListOfShape anArguments;
            anArguments.Append(BRepBuilderAPI_Transform(
                aSubject->Shape(),
                aSubjectIO->LocalTransformation()).Shape());
            BRepAlgoAPI_Cut aCut;
            aCut.SetNonDestructive(Standard_True);
            aCut.SetArguments(anArguments);
            aCut.SetTools(aTools);
            aCut.SetFuzzyValue(kFuzzyValue);
            aCut.SetUseOBB(Standard_True);
            aCut.SetCheckInverted(Standard_True);
            aCut.Build();
            if (!aCut.IsDone()) {
                return Standard_False;
            }
            aCut.SimplifyResult();
            if (!IsValidSolidBooleanResult(aCut.Shape())) {
                return Standard_False;
            }

            TemporalBooleanObject aState = aSelection->second;
            aState.selectionType = BooleanSelectionType::Subject;
            Handle(AIS_InteractiveObject) aResult =
                new AIS_Shape(aCut.Shape());
            applyStyle(aResult, aState);
            rememberOwned(aResult);
            if (!_stateValid) {
                return Standard_False;
            }
            aPreviews.push_back({aSubjectIO, aResult, aState});
        }

        std::map<Handle(AIS_InteractiveObject), TemporalBooleanObject>
            aReplacement = _selectionMap;
        for (const Preview& aPreview : aPreviews) {
            if (aReplacement.erase(aPreview.source) != 1
                || !aReplacement.emplace(
                    aPreview.result,
                    aPreview.state).second) {
                return Standard_False;
            }
        }
        for (const Preview& aPreview : aPreviews) {
            showInteractiveByType(
                aPreview.result,
                BooleanSelectionType::Subject);
            myContext->SetSelected(aPreview.result, Standard_False);
            myContext->Remove(aPreview.source, Standard_False);
        }
        _selectionMap.swap(aReplacement);
        myContext->UpdateCurrentViewer();
        return Standard_True;
    } catch (const Standard_Failure& aFailure) {
        std::cout << "Boolean subtract preview failure: "
                  << aFailure.GetMessageString() << std::endl;
        markInvalid();
        return Standard_False;
    } catch (...) {
        markInvalid();
        return Standard_False;
    }
}

Standard_Boolean BooleanOperationController::buildUnionPreview(
    const std::vector<Handle(AIS_InteractiveObject)>& theSubjects) noexcept
{
    try {
        if (theSubjects.size() < 2
            || theSubjects.size() > kMaxSourceOperands) {
            return Standard_False;
        }
        TopTools_ListOfShape anArguments;
        TopTools_ListOfShape aTools;
        for (std::size_t anIndex = 0;
             anIndex < theSubjects.size(); ++anIndex) {
            const Handle(AIS_Shape) aShape =
                Handle(AIS_Shape)::DownCast(theSubjects[anIndex]);
            if (aShape.IsNull() || aShape->Shape().IsNull()
                || _selectionMap.find(theSubjects[anIndex])
                    == _selectionMap.end()) {
                return Standard_False;
            }
            const TopoDS_Shape aWorldShape = BRepBuilderAPI_Transform(
                aShape->Shape(),
                theSubjects[anIndex]->LocalTransformation()).Shape();
            (anIndex == 0 ? anArguments : aTools).Append(aWorldShape);
        }

        BRepAlgoAPI_Fuse aFuse;
        aFuse.SetNonDestructive(Standard_True);
        aFuse.SetArguments(anArguments);
        aFuse.SetTools(aTools);
        aFuse.SetFuzzyValue(kFuzzyValue);
        aFuse.SetUseOBB(Standard_True);
        aFuse.SetCheckInverted(Standard_True);
        aFuse.Build();
        if (!aFuse.IsDone()) {
            return Standard_False;
        }
        aFuse.SimplifyResult();
        if (!IsValidSolidBooleanResult(aFuse.Shape())) {
            return Standard_False;
        }

        const TemporalBooleanObject aResultStyle =
            _selectionMap.find(theSubjects.front())->second;
        Handle(AIS_InteractiveObject) aResult =
            new AIS_Shape(aFuse.Shape());
        applyStyle(aResult, aResultStyle);
        rememberOwned(aResult);
        if (!_stateValid) {
            return Standard_False;
        }

        showInteractiveByType(aResult, BooleanSelectionType::Subject);
        myContext->ClearSelected(Standard_False);
        myContext->SetSelected(aResult, Standard_False);
        myContext->Deactivate(aResult);
        for (const Handle(AIS_InteractiveObject)& aSubject : theSubjects) {
            myContext->Remove(aSubject, Standard_False);
        }
        _unionTrialResult = Handle(AIS_Shape)::DownCast(aResult);
        if (_unionTrialResult.IsNull()) {
            markInvalid();
            return Standard_False;
        }
        _selectionFrozen = Standard_True;
        myContext->UpdateCurrentViewer();
        return Standard_True;
    } catch (const Standard_Failure& aFailure) {
        std::cout << "Boolean union preview failure: "
                  << aFailure.GetMessageString() << std::endl;
        markInvalid();
        return Standard_False;
    } catch (...) {
        markInvalid();
        return Standard_False;
    }
}

Standard_Boolean BooleanOperationController::capturePreview(
    BooleanPreviewCapture& theCapture) const noexcept
{
    theCapture = {};
    if (!canApply() || !_activeAction.has_value()
        || myContext.IsNull()) {
        return Standard_False;
    }
    try {
        Standard_Boolean hasCompleteLabels = Standard_False;
        std::vector<TDF_Label> aLabels =
            orderedSourceLabels(hasCompleteLabels);
        if (!hasCompleteLabels || aLabels.size() < 2
            || aLabels.size() > kMaxSourceOperands) {
            return Standard_False;
        }

        BooleanPreviewCapture aCapture;
        aCapture.action = *_activeAction;
        aCapture.suppressedSourceLabels = std::move(aLabels);
        if (*_activeAction == BooleanAction::BooleanUnion) {
            if (!_selectionFrozen || _unionTrialResult.IsNull()
                || !myContext->IsDisplayed(_unionTrialResult)) {
                return Standard_False;
            }
            aCapture.results.push_back(_unionTrialResult);
        } else {
            std::vector<std::pair<std::string, Handle(AIS_Shape)>> anActors;
            for (const auto& aSelection : _selectionMap) {
                const Handle(AIS_Shape) aShape =
                    Handle(AIS_Shape)::DownCast(aSelection.first);
                if (aShape.IsNull() || aShape->Shape().IsNull()
                    || !myContext->IsDisplayed(aShape)) {
                    return Standard_False;
                }
                if (aSelection.second.selectionType
                    == BooleanSelectionType::Actor) {
                    const std::string anIdentifier =
                        myDoc->EntityIdentifierForLabel(
                            aSelection.second.documentLabel);
                    if (anIdentifier.empty()) {
                        return Standard_False;
                    }
                    anActors.push_back({anIdentifier, aShape});
                }
            }
            std::sort(
                anActors.begin(),
                anActors.end(),
                [](const auto& theLeft, const auto& theRight) {
                    return theLeft.first < theRight.first;
                });
            for (const auto& anActor : anActors) {
                aCapture.actors.push_back(anActor.second);
            }
            Standard_Boolean hasCompleteSubjects = Standard_False;
            const auto aSubjects =
                orderedSubjectPresentations(hasCompleteSubjects);
            if (!hasCompleteSubjects) {
                return Standard_False;
            }
            for (const Handle(AIS_InteractiveObject)& aSubject : aSubjects) {
                const Handle(AIS_Shape) aShape =
                    Handle(AIS_Shape)::DownCast(aSubject);
                if (aShape.IsNull() || !myContext->IsDisplayed(aShape)) {
                    return Standard_False;
                }
                aCapture.results.push_back(aShape);
            }
            if (aCapture.actors.empty() || aCapture.results.empty()
                || aCapture.actors.size() + aCapture.results.size()
                    != aCapture.suppressedSourceLabels.size()) {
                return Standard_False;
            }
        }
        theCapture = std::move(aCapture);
        return Standard_True;
    } catch (...) {
        theCapture = {};
        return Standard_False;
    }
}

void BooleanOperationController::applyStyle(
    Handle(AIS_InteractiveObject)& theObject,
    const TemporalBooleanObject& theStyle)
{
    Handle(AIS_Shape) aShape = Handle(AIS_Shape)::DownCast(theObject);
    if (aShape.IsNull()) {
        throw Standard_Failure("Boolean presentation is not an AIS_Shape");
    }
    aShape->UnsetColor();
    if (!theStyle.documentLabel.IsNull()) {
        myDoc->LoadObjectMeterial(theStyle.documentLabel, aShape);
    } else {
        aShape->SetMaterial(theStyle.materialName);
        aShape->SetColor(theStyle.colorName);
    }
}

void BooleanOperationController::persistStyle(
    const TDF_Label& theLabel,
    const TemporalBooleanObject& theStyle)
{
    if (theLabel.IsNull() || theStyle.documentLabel.IsNull()) {
        throw Standard_Failure("Boolean appearance label is null");
    }
    if (!myDoc->CopyObjectAppearance(
            theStyle.documentLabel, theLabel)) {
        throw Standard_Failure("Unable to preserve Boolean appearance");
    }
}

Handle(AIS_InteractiveObject)
BooleanOperationController::ioCopyWithStyle(
    const Handle(AIS_InteractiveObject)& theOriginal,
    const TemporalBooleanObject& theStyle)
{
    const Handle(AIS_Shape) anOriginalShape =
        Handle(AIS_Shape)::DownCast(theOriginal);
    if (anOriginalShape.IsNull() || anOriginalShape->Shape().IsNull()) {
        return {};
    }
    const TopoDS_Shape aWorldShape = BRepBuilderAPI_Transform(
        anOriginalShape->Shape(),
        theOriginal->LocalTransformation()).Shape();
    if (aWorldShape.IsNull()) {
        return {};
    }
    Handle(AIS_InteractiveObject) aCopy = new AIS_Shape(aWorldShape);
    applyStyle(aCopy, theStyle);
    return aCopy;
}

void BooleanOperationController::showInteractiveByType(
    const Handle(AIS_InteractiveObject)& theShape,
    const BooleanSelectionType theType)
{
    if (theShape.IsNull()) {
        throw Standard_Failure("Boolean presentation is null");
    }
    Quantity_Color aColor(Quantity_NOC_GRAY);
    Standard_Boolean isWireframe = Standard_False;
    if (theType == BooleanSelectionType::Actor) {
        isWireframe = Standard_True;
        aColor = Quantity_Color(Quantity_NOC_ORANGE);
    } else if (theType == BooleanSelectionType::Subject) {
        aColor = Quantity_Color(Quantity_NOC_LIGHTSKYBLUE);
    }

    Handle(Prs3d_Drawer) aHighlight = new Prs3d_Drawer();
    aHighlight->SetColor(aColor);
    theShape->SetHilightAttributes(aHighlight);
    theShape->SetDisplayMode(isWireframe ? AIS_WireFrame : AIS_Shaded);
    if (theType == BooleanSelectionType::Undefined) {
        theShape->UnsetDisplayMode();
    }
    myContext->Display(
        theShape,
        isWireframe ? AIS_WireFrame : AIS_Shaded,
        AIS_Shape::SelectionMode(TopAbs_SHAPE),
        Standard_False);
    if (theType == BooleanSelectionType::Undefined) {
        myContext->Unhilight(theShape, Standard_False);
        myContext->Activate(
            theShape,
            AIS_Shape::SelectionMode(TopAbs_SHAPE),
            Standard_False);
    } else {
        myContext->HilightWithColor(
            theShape,
            aHighlight,
            Standard_True);
    }
}

BooleanApplyResult BooleanOperationController::apply(
    const BooleanAction theAction) noexcept
{
    if (!actionMatches(theAction)) {
        return BooleanApplyResult::NoChange;
    }
    if (!canApply() || _selectionMap.empty()) {
        cancelImpl();
        return BooleanApplyResult::NoChange;
    }

    Standard_Boolean hasCompleteLabels = Standard_False;
    const std::vector<TDF_Label> aSourceLabels =
        orderedSourceLabels(hasCompleteLabels);
    if (!hasCompleteLabels || aSourceLabels.size() < 2
        || aSourceLabels.size() > kMaxSourceOperands) {
        cancelImpl();
        return BooleanApplyResult::NoChange;
    }

    Handle(TDocStd_Document) aDocument;
    try {
        aDocument = myDoc->ChangeDocument();
    } catch (...) {
        cancelImpl();
        return BooleanApplyResult::NoChange;
    }
    try {
        if (aDocument.IsNull() || aDocument->HasOpenCommand()) {
            cancelImpl();
            return BooleanApplyResult::NoChange;
        }
    } catch (...) {
        cancelImpl();
        return BooleanApplyResult::NoChange;
    }

    std::vector<std::pair<Handle(AIS_InteractiveObject),
                          TemporalBooleanObject>> aResults;
    try {
        if (theAction == BooleanAction::BooleanUnion) {
            Standard_Boolean hasCompleteSubjects = Standard_False;
            const auto aSubjects =
                orderedSubjectPresentations(hasCompleteSubjects);
            if (!hasCompleteSubjects || aSubjects.size() < 2
                || _unionTrialResult.IsNull()) {
                cancelImpl();
                return BooleanApplyResult::NoChange;
            }
            aResults.push_back({
                _unionTrialResult,
                _selectionMap.find(aSubjects.front())->second,
            });
        } else {
            for (const auto& aSelection : _selectionMap) {
                if (aSelection.second.selectionType
                    == BooleanSelectionType::Subject) {
                    aResults.push_back(aSelection);
                }
            }
            if (aResults.empty()) {
                cancelImpl();
                return BooleanApplyResult::NoChange;
            }
        }

        aDocument->NewCommand();
        if (!aDocument->HasOpenCommand()) {
            cancelImpl();
            return BooleanApplyResult::NoChange;
        }
        // Persist result appearance while each source label is still present;
        // removing a source first clears the XCAF material relationship that
        // the result must inherit.
        for (const auto& aResult : aResults) {
            const TDF_Label aResultLabel = myDoc->AddShape(aResult.first);
            if (aResultLabel.IsNull()) {
                rollbackFailedTransaction(aDocument);
                return BooleanApplyResult::NoChange;
            }
            persistStyle(aResultLabel, aResult.second);
        }
        for (const TDF_Label& aLabel : aSourceLabels) {
            if (!myDoc->RemoveShape(aLabel)) {
                rollbackFailedTransaction(aDocument);
                return BooleanApplyResult::NoChange;
            }
        }
        if (!aDocument->CommitCommand()) {
            rollbackFailedTransaction(aDocument);
            return BooleanApplyResult::NoChange;
        }
    } catch (const Standard_Failure& aFailure) {
        std::cout << "Boolean transaction failure: "
                  << aFailure.GetMessageString() << std::endl;
        rollbackFailedTransaction(aDocument);
        return BooleanApplyResult::NoChange;
    } catch (...) {
        rollbackFailedTransaction(aDocument);
        return BooleanApplyResult::NoChange;
    }

    bool needsRedraw = false;
    try {
        if (theAction == BooleanAction::BooleanSubtract) {
            for (const auto& aSelection : _selectionMap) {
                if (aSelection.second.selectionType
                    == BooleanSelectionType::Actor) {
                    myContext->Remove(aSelection.first, Standard_False);
                } else if (aSelection.second.selectionType
                           == BooleanSelectionType::Subject) {
                    showInteractiveByType(
                        aSelection.first,
                        BooleanSelectionType::Undefined);
                }
            }
        } else {
            showInteractiveByType(
                _unionTrialResult,
                BooleanSelectionType::Undefined);
        }
        myContext->ClearSelected(Standard_False);
        myContext->UpdateCurrentViewer();
        myDoc->NotifyChanges();
    } catch (...) {
        needsRedraw = true;
        try {
            myDoc->NotifyChanges();
        } catch (...) {
        }
    }

    // Every result is now document-owned. Do not remove it with transient
    // cleanup; releasing these handles is safe because AIS/OCAF retain them.
    clearOperationState();
    return needsRedraw
        ? BooleanApplyResult::AppliedNeedsDocumentRedraw
        : BooleanApplyResult::Applied;
}

Standard_Boolean BooleanOperationController::cancelImpl() noexcept
{
    _canApply = Standard_False;
    _selectionFrozen = Standard_True;
    bool didRestoreAll = true;

    if (_documentCommandUnresolved) {
        Handle(TDocStd_Document) aDocument;
        try {
            if (!myDoc.IsNull()) {
                aDocument = myDoc->ChangeDocument();
            }
        } catch (...) {
        }
        if (AbortCommandNoThrow(aDocument)) {
            _documentCommandUnresolved = Standard_False;
        } else {
            didRestoreAll = false;
        }
    }
    std::vector<Handle(AIS_InteractiveObject)> aRestoredOriginals;
    try {
        aRestoredOriginals.reserve(_selectionMap.size());
    } catch (...) {
        markInvalid();
        return Standard_False;
    }

    for (const auto& aSelection : _selectionMap) {
        const Handle(AIS_InteractiveObject)& anOriginal =
            aSelection.second.original;
        if (anOriginal.IsNull()
            || std::find(aRestoredOriginals.begin(),
                         aRestoredOriginals.end(),
                         anOriginal) != aRestoredOriginals.end()) {
            continue;
        }
        try {
            showInteractiveByType(
                anOriginal,
                BooleanSelectionType::Undefined);
            aRestoredOriginals.push_back(anOriginal);
        } catch (...) {
            didRestoreAll = false;
        }
    }
    if (!cleanupOwnedPresentations()) {
        didRestoreAll = false;
    }
    try {
        // Boolean cancellation intentionally restores committed presentations
        // but drops operand selection. Explicit Cancel immediately completes the
        // tool, and lifecycle cancellation must not leave an actionable stale
        // selection/Apply state while the app is inactive.
        myContext->ClearSelected(Standard_False);
        myContext->UpdateCurrentViewer();
    } catch (...) {
        didRestoreAll = false;
    }

    if (didRestoreAll) {
        clearOperationState();
        return Standard_True;
    }
    markInvalid();
    return Standard_False;
}

void BooleanOperationController::cancel(
    const BooleanAction theAction) noexcept
{
    if (actionMatches(theAction)) {
        cancelImpl();
    }
}

void BooleanOperationController::cancelActive() noexcept
{
    if (_activeAction.has_value() || !_selectionMap.empty()
        || !_ownedPresentations.empty() || !_stateValid) {
        cancelImpl();
    }
}

void BooleanOperationController::clearOperationState() noexcept
{
    _selectionMap.clear();
    _actedIOArray.clear();
    _actorIOArray.clear();
    _subjectSelectionOrder.clear();
    _ownedPresentations.clear();
    _unionTrialResult.Nullify();
    _activeAction.reset();
    _canApply = Standard_False;
    _stateValid = Standard_True;
    _selectionFrozen = Standard_False;
    _documentCommandUnresolved = Standard_False;
}

} // namespace core3d
