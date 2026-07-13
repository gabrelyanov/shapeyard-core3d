//
//  BooleanOperationController.cpp
//  Core3D
//

#include "BooleanOperationController.hpp"

#include <AIS_Shape.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_IndexedMapOfShape.hxx>

#include <dispatch/dispatch.h>
#include <pthread.h>

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_set>
#include <utility>

namespace core3d {

enum class BooleanPreviewWorkerOutcome : std::uint8_t {
    Success = 0,
    Cancelled,
    Failed,
};

struct BooleanPreviewWorkerRequest {
    BooleanAction action = BooleanAction::BooleanSubtract;
    std::uint64_t generation = 0;
    std::string fingerprint;
    std::vector<TopoDS_Shape> actors;
    std::vector<TopoDS_Shape> subjects;
    std::shared_ptr<std::atomic_bool> cancellation;
    Standard_Size maximumResultTopologyNodes =
        BooleanOperationController::kMaxResultTopologyNodes;
    Standard_Size maximumResultSolids =
        BooleanOperationController::kMaxResultSolids;
};

struct BooleanPreviewWorkerResult {
    BooleanAction action = BooleanAction::BooleanSubtract;
    std::uint64_t generation = 0;
    std::string fingerprint;
    BooleanPreviewWorkerOutcome outcome =
        BooleanPreviewWorkerOutcome::Failed;
    std::vector<TopoDS_Shape> results;
    Standard_Boolean computeWasMainThread = Standard_False;
};

namespace {

constexpr Standard_Real kFuzzyValue = 1.0e-3;
std::atomic<std::uint64_t> gBooleanControllerEpoch{0};

Standard_Boolean IsSupportedBooleanAction(
    const BooleanAction theAction) noexcept
{
    return theAction == BooleanAction::BooleanSubtract
        || theAction == BooleanAction::BooleanUnion
        || theAction == BooleanAction::BooleanIntersect;
}

Standard_Boolean IsSingleResultBooleanAction(
    const BooleanAction theAction) noexcept
{
    return theAction == BooleanAction::BooleanUnion
        || theAction == BooleanAction::BooleanIntersect;
}

Standard_Boolean IsBRepModelingLabel(
    const Handle(OcctDocument)& theDocument,
    const TDF_Label& theLabel) noexcept
{
    if (theDocument.IsNull() || theLabel.IsNull()) {
        return Standard_False;
    }
	if (!theDocument->IsEditableFreeSimpleDefinitionLabel(theLabel)) {
		return Standard_False;
	}
    const OcctGeometryRepresentation aRepresentation =
        theDocument->GeometryRepresentationForLabel(theLabel);
    return aRepresentation == OcctGeometryRepresentation::LegacyUnknown
        || aRepresentation == OcctGeometryRepresentation::BRep;
}

Standard_Boolean IsCurrentBRepSelection(
    const Handle(OcctDocument)& theDocument,
    const TemporalBooleanObject& theSelection) noexcept
{
    if (!IsBRepModelingLabel(
            theDocument, theSelection.documentLabel)
        || theSelection.original.IsNull()
        || !theDocument->IsPresentationEditable(
            theSelection.original)) {
        return Standard_False;
    }
    try {
        const Handle(AIS_Shape) aShape =
            Handle(AIS_Shape)::DownCast(theSelection.original);
        return !aShape.IsNull() && !aShape->Shape().IsNull()
            && theDocument->ShapeLabel(theSelection.original)
                .IsEqual(theSelection.documentLabel);
    } catch (...) {
        return Standard_False;
    }
}

dispatch_queue_t BooleanWorkerQueue()
{
    static dispatch_queue_t aQueue = []() {
        dispatch_queue_attr_t anAttribute =
            dispatch_queue_attr_make_with_qos_class(
                DISPATCH_QUEUE_SERIAL,
                QOS_CLASS_USER_INITIATED,
                0);
        return dispatch_queue_create(
            "com.shapeyard.core3d.boolean-preview",
            anAttribute);
    }();
    return aQueue;
}

Standard_Boolean AccumulateBoundedTopology(
    const TopoDS_Shape& theShape,
    const Standard_Size thePerShapeLimit,
    const Standard_Size theAggregateLimit,
    Standard_Size& theAggregateCount,
    Standard_Size* theSolidCount = nullptr,
    const Standard_Size theMaximumSolidCount =
        BooleanOperationController::kMaxResultSolids)
{
    if (theShape.IsNull() || thePerShapeLimit == 0
        || theAggregateLimit == 0) {
        return Standard_False;
    }
    std::vector<TopoDS_Shape> aStack;
    aStack.reserve(std::min<Standard_Size>(thePerShapeLimit, 1'024));
    aStack.push_back(theShape);
    TopTools_IndexedMapOfShape aVisited;
    Standard_Size aShapeCount = 0;
    while (!aStack.empty()) {
        const TopoDS_Shape aCurrent = aStack.back();
        aStack.pop_back();
        if (aVisited.Contains(aCurrent)) {
            continue;
        }
        aVisited.Add(aCurrent);
        if (++aShapeCount > thePerShapeLimit
            || ++theAggregateCount > theAggregateLimit) {
            return Standard_False;
        }
        if (theSolidCount != nullptr
            && aCurrent.ShapeType() == TopAbs_SOLID
            && ++*theSolidCount
                > theMaximumSolidCount) {
            return Standard_False;
        }
        for (TopoDS_Iterator aChild(aCurrent, Standard_True, Standard_True);
             aChild.More(); aChild.Next()) {
            if (!aVisited.Contains(aChild.Value())) {
                // The pending frontier is independently bounded, so a broad
                // malformed compound cannot allocate past admission limits
                // before its nodes are counted.
                if (aStack.size() >= thePerShapeLimit
                    || aStack.size() >= theAggregateLimit) {
                    return Standard_False;
                }
                aStack.push_back(aChild.Value());
            }
        }
    }
    return Standard_True;
}

Standard_Boolean HasOnlySolidBooleanResultBranches(
    const TopoDS_Shape& theShape,
    const Standard_Size theMaximumTopologyNodes)
{
    if (theShape.IsNull() || theMaximumTopologyNodes == 0) {
        return Standard_False;
    }

    // Boolean modeling commits volumes. A compound that merely contains one
    // valid solid is not sufficient: OCCT can also return loose shells, faces,
    // wires, or edges alongside it. Traverse only the result's container
    // hierarchy and stop at solids so their normal boundary topology remains
    // valid, while any lower-dimensional sibling branch fails closed.
    std::vector<TopoDS_Shape> aStack;
    aStack.push_back(theShape);
    TopTools_IndexedMapOfShape aVisited;
    Standard_Size aShapeCount = 0;
    Standard_Boolean hasSolid = Standard_False;
    while (!aStack.empty()) {
        const TopoDS_Shape aCurrent = aStack.back();
        aStack.pop_back();
        if (aVisited.Contains(aCurrent)) {
            continue;
        }
        aVisited.Add(aCurrent);
        if (++aShapeCount > theMaximumTopologyNodes) {
            return Standard_False;
        }

        switch (aCurrent.ShapeType()) {
            case TopAbs_SOLID:
                hasSolid = Standard_True;
                break;
            case TopAbs_COMPOUND: {
                Standard_Boolean hasChild = Standard_False;
                for (TopoDS_Iterator aChild(
                         aCurrent, Standard_True, Standard_True);
                     aChild.More(); aChild.Next()) {
                    hasChild = Standard_True;
                    if (!aVisited.Contains(aChild.Value())) {
                        if (aStack.size() >= theMaximumTopologyNodes) {
                            return Standard_False;
                        }
                        aStack.push_back(aChild.Value());
                    }
                }
                if (!hasChild) {
                    return Standard_False;
                }
                break;
            }
            case TopAbs_COMPSOLID: {
                Standard_Boolean hasChild = Standard_False;
                for (TopoDS_Iterator aChild(
                         aCurrent, Standard_True, Standard_True);
                     aChild.More(); aChild.Next()) {
                    hasChild = Standard_True;
                    if (aChild.Value().ShapeType() != TopAbs_SOLID) {
                        return Standard_False;
                    }
                    if (!aVisited.Contains(aChild.Value())) {
                        if (aStack.size() >= theMaximumTopologyNodes) {
                            return Standard_False;
                        }
                        aStack.push_back(aChild.Value());
                    }
                }
                if (!hasChild) {
                    return Standard_False;
                }
                break;
            }
            default:
                return Standard_False;
        }
    }
    return hasSolid;
}

Standard_Boolean IsValidSolidBooleanResult(
    const TopoDS_Shape& theShape,
    const Standard_Size theMaximumTopologyNodes,
    const Standard_Size theMaximumSolidCount,
    Standard_Size& theAggregateTopologyNodes,
    Standard_Size& theAggregateSolidCount)
{
    if (theShape.IsNull()) {
        return Standard_False;
    }
    try {
        const Standard_Size aSolidCountBefore = theAggregateSolidCount;
        if (!AccumulateBoundedTopology(
                theShape,
                theMaximumTopologyNodes,
                theMaximumTopologyNodes,
                theAggregateTopologyNodes,
                &theAggregateSolidCount,
                theMaximumSolidCount)
            || theAggregateSolidCount == aSolidCountBefore
            || !HasOnlySolidBooleanResultBranches(
                theShape, theMaximumTopologyNodes)) {
            return Standard_False;
        }
        BRepCheck_Analyzer anAnalyzer(theShape, Standard_True);
        return anAnalyzer.IsValid();
    } catch (...) {
        return Standard_False;
    }
}

class BooleanCancellationIndicator final
    : public Message_ProgressIndicator {
    DEFINE_STANDARD_RTTI_INLINE(
        BooleanCancellationIndicator,
        Message_ProgressIndicator)
public:
    explicit BooleanCancellationIndicator(
        std::shared_ptr<std::atomic_bool> theCancellation)
    : myCancellation(std::move(theCancellation)) {}

protected:
    Standard_Boolean UserBreak() override
    {
        return myCancellation != nullptr
            && myCancellation->load(std::memory_order_relaxed);
    }

    void Show(
        const Message_ProgressScope&,
        const Standard_Boolean) override {}

private:
    std::shared_ptr<std::atomic_bool> myCancellation;
};

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

class BooleanPreviewWorker final
    : public std::enable_shared_from_this<BooleanPreviewWorker> {
public:
    explicit BooleanPreviewWorker(
        std::weak_ptr<BooleanOperationController> theOwner)
    : myOwner(std::move(theOwner)) {}

    ~BooleanPreviewWorker() noexcept
    {
        shutdown();
    }

    Standard_Boolean enqueue(BooleanPreviewWorkerRequest theRequest) noexcept
    {
        bool shouldSchedule = false;
        try {
            {
                std::lock_guard<std::mutex> aLock(myMutex);
                if (myStopped || theRequest.cancellation == nullptr) {
                    return Standard_False;
                }
                if (myActiveCancellation != nullptr) {
                    myActiveCancellation->store(
                        true, std::memory_order_relaxed);
                }
                if (myPending.has_value()) {
                    myPending->cancellation->store(
                        true, std::memory_order_relaxed);
                    ++myPendingReplacementCount;
                }
                myPending = std::move(theRequest);
                ++mySubmittedCount;
                if (!myDrainScheduled) {
                    myDrainScheduled = true;
                    shouldSchedule = true;
                }
            }
            myCondition.notify_all();
            if (shouldSchedule) {
                auto* aContext = new std::shared_ptr<BooleanPreviewWorker>(
                    shared_from_this());
                dispatch_async_f(
                    BooleanWorkerQueue(),
                    aContext,
                    &BooleanPreviewWorker::DrainTrampoline);
            }
            return Standard_True;
        } catch (...) {
            cancelAll();
            std::lock_guard<std::mutex> aLock(myMutex);
            myDrainScheduled = false;
            return Standard_False;
        }
    }

    void cancelAll() noexcept
    {
        {
            std::lock_guard<std::mutex> aLock(myMutex);
            if (myActiveCancellation != nullptr) {
                myActiveCancellation->store(
                    true, std::memory_order_relaxed);
            }
            if (myPending.has_value()) {
                myPending->cancellation->store(
                    true, std::memory_order_relaxed);
                myPending.reset();
            }
        }
        myCondition.notify_all();
    }

    void cancelActiveForSupersede() noexcept
    {
        {
            std::lock_guard<std::mutex> aLock(myMutex);
            if (myActiveCancellation != nullptr) {
                myActiveCancellation->store(
                    true, std::memory_order_relaxed);
            }
        }
        myCondition.notify_all();
    }

    void shutdown() noexcept
    {
        {
            std::lock_guard<std::mutex> aLock(myMutex);
            if (myStopped) {
                return;
            }
            myStopped = true;
            if (myActiveCancellation != nullptr) {
                myActiveCancellation->store(
                    true, std::memory_order_relaxed);
            }
            if (myPending.has_value()) {
                myPending->cancellation->store(
                    true, std::memory_order_relaxed);
                myPending.reset();
            }
#ifdef DEBUG
            myDebugBlocked = false;
#endif
        }
        myCondition.notify_all();
    }

#ifdef DEBUG
    struct DebugSnapshot {
        std::uint64_t submittedCount = 0;
        std::uint64_t startedCount = 0;
        std::uint64_t completedCount = 0;
        std::uint64_t cancelledCount = 0;
        std::uint64_t pendingReplacementCount = 0;
        Standard_Boolean active = Standard_False;
        Standard_Boolean pending = Standard_False;
        Standard_Boolean lastComputeWasMainThread = Standard_False;
    };

    DebugSnapshot debugSnapshot() const noexcept
    {
        std::lock_guard<std::mutex> aLock(myMutex);
        return {
            mySubmittedCount,
            myStartedCount,
            myCompletedCount,
            myCancelledCount,
            myPendingReplacementCount,
            myActiveCancellation != nullptr,
            myPending.has_value(),
            myLastComputeWasMainThread,
        };
    }

    void debugSetBlocked(const Standard_Boolean theBlocked) noexcept
    {
        {
            std::lock_guard<std::mutex> aLock(myMutex);
            myDebugBlocked = theBlocked;
        }
        myCondition.notify_all();
    }
#endif

private:
    struct MainCompletion {
        std::weak_ptr<BooleanOperationController> owner;
        BooleanPreviewWorkerResult result;
    };

    static void DrainTrampoline(void* theContext)
    {
        std::unique_ptr<std::shared_ptr<BooleanPreviewWorker>> aContext(
            static_cast<std::shared_ptr<BooleanPreviewWorker>*>(
                theContext));
        (*aContext)->drain();
    }

    static void DeliverTrampoline(void* theContext)
    {
        std::unique_ptr<MainCompletion> aCompletion(
            static_cast<MainCompletion*>(theContext));
        if (const auto anOwner = aCompletion->owner.lock()) {
            anOwner->acceptWorkerResult(
                std::move(aCompletion->result));
        }
    }

    void drain() noexcept
    {
        for (;;) {
            BooleanPreviewWorkerRequest aRequest;
            {
                std::unique_lock<std::mutex> aLock(myMutex);
                if (myStopped || !myPending.has_value()) {
                    myDrainScheduled = false;
                    myActiveCancellation.reset();
                    return;
                }
                aRequest = std::move(*myPending);
                myPending.reset();
                myActiveCancellation = aRequest.cancellation;
                ++myStartedCount;
#ifdef DEBUG
                myCondition.wait(aLock, [&]() {
                    return myStopped || !myDebugBlocked;
                });
#endif
            }

            BooleanPreviewWorkerResult aResult = compute(aRequest);
            {
                std::lock_guard<std::mutex> aLock(myMutex);
                myActiveCancellation.reset();
                ++myCompletedCount;
                myLastComputeWasMainThread =
                    aResult.computeWasMainThread;
                if (aResult.outcome
                    == BooleanPreviewWorkerOutcome::Cancelled) {
                    ++myCancelledCount;
                }
            }

            try {
                auto* aCompletion = new MainCompletion{
                    myOwner,
                    std::move(aResult),
                };
                dispatch_async_f(
                    dispatch_get_main_queue(),
                    aCompletion,
                    &BooleanPreviewWorker::DeliverTrampoline);
            } catch (...) {
            }
        }
    }

    static BooleanPreviewWorkerResult compute(
        const BooleanPreviewWorkerRequest& theRequest) noexcept
    {
        BooleanPreviewWorkerResult aResult;
        aResult.action = theRequest.action;
        aResult.generation = theRequest.generation;
        aResult.fingerprint = theRequest.fingerprint;
        aResult.computeWasMainThread = pthread_main_np() != 0;
        const auto wasCancelled = [&]() {
            return theRequest.cancellation == nullptr
                || theRequest.cancellation->load(
                    std::memory_order_relaxed);
        };
        if (wasCancelled()) {
            aResult.outcome = BooleanPreviewWorkerOutcome::Cancelled;
            return aResult;
        }

        try {
            OCC_CATCH_SIGNALS
            Handle(BooleanCancellationIndicator) aProgress =
                new BooleanCancellationIndicator(
                    theRequest.cancellation);
            Standard_Size anAggregateResultNodes = 0;
            Standard_Size anAggregateResultSolids = 0;
            if (theRequest.action == BooleanAction::BooleanSubtract) {
                if (theRequest.actors.empty()
                    || theRequest.subjects.empty()) {
                    return aResult;
                }
                TopTools_ListOfShape aTools;
                for (const TopoDS_Shape& anActor : theRequest.actors) {
                    aTools.Append(anActor);
                }
                aResult.results.reserve(theRequest.subjects.size());
                for (const TopoDS_Shape& aSubject : theRequest.subjects) {
                    if (wasCancelled()) {
                        aResult.outcome =
                            BooleanPreviewWorkerOutcome::Cancelled;
                        aResult.results.clear();
                        return aResult;
                    }
                    TopTools_ListOfShape anArguments;
                    anArguments.Append(aSubject);
                    BRepAlgoAPI_Cut aCut;
                    aCut.SetRunParallel(Standard_False);
                    aCut.SetNonDestructive(Standard_True);
                    aCut.SetArguments(anArguments);
                    aCut.SetTools(aTools);
                    aCut.SetFuzzyValue(kFuzzyValue);
                    aCut.SetUseOBB(Standard_True);
                    aCut.SetCheckInverted(Standard_True);
                    aCut.Build(aProgress->Start());
                    if (wasCancelled()) {
                        aResult.outcome =
                            BooleanPreviewWorkerOutcome::Cancelled;
                        aResult.results.clear();
                        return aResult;
                    }
                    if (!aCut.IsDone()) {
                        return aResult;
                    }
                    TopoDS_Shape aRawResult = aCut.Shape();
                    Standard_Size aRawNodes = 0;
                    if (!AccumulateBoundedTopology(
                            aRawResult,
                            theRequest.maximumResultTopologyNodes,
                            theRequest.maximumResultTopologyNodes,
                            aRawNodes)) {
                        return aResult;
                    }
                    if (wasCancelled()) {
                        aResult.outcome =
                            BooleanPreviewWorkerOutcome::Cancelled;
                        aResult.results.clear();
                        return aResult;
                    }
                    aCut.SimplifyResult();
                    if (wasCancelled()) {
                        aResult.outcome =
                            BooleanPreviewWorkerOutcome::Cancelled;
                        aResult.results.clear();
                        return aResult;
                    }
                    if (!IsValidSolidBooleanResult(
                            aCut.Shape(),
                            theRequest.maximumResultTopologyNodes,
                            theRequest.maximumResultSolids,
                            anAggregateResultNodes,
                            anAggregateResultSolids)) {
                        return aResult;
                    }
                    aResult.results.push_back(aCut.Shape());
                }
            } else if (theRequest.action == BooleanAction::BooleanUnion) {
                if (!theRequest.actors.empty()
                    || theRequest.subjects.size() < 2) {
                    return aResult;
                }
                TopTools_ListOfShape anArguments;
                TopTools_ListOfShape aTools;
                for (std::size_t anIndex = 0;
                     anIndex < theRequest.subjects.size(); ++anIndex) {
                    (anIndex == 0 ? anArguments : aTools).Append(
                        theRequest.subjects[anIndex]);
                }
                BRepAlgoAPI_Fuse aFuse;
                aFuse.SetRunParallel(Standard_False);
                aFuse.SetNonDestructive(Standard_True);
                aFuse.SetArguments(anArguments);
                aFuse.SetTools(aTools);
                aFuse.SetFuzzyValue(kFuzzyValue);
                aFuse.SetUseOBB(Standard_True);
                aFuse.SetCheckInverted(Standard_True);
                aFuse.Build(aProgress->Start());
                if (wasCancelled()) {
                    aResult.outcome =
                        BooleanPreviewWorkerOutcome::Cancelled;
                    return aResult;
                }
                if (!aFuse.IsDone()) {
                    return aResult;
                }
                Standard_Size aRawNodes = 0;
                if (!AccumulateBoundedTopology(
                        aFuse.Shape(),
                        theRequest.maximumResultTopologyNodes,
                        theRequest.maximumResultTopologyNodes,
                        aRawNodes)) {
                    return aResult;
                }
                if (wasCancelled()) {
                    aResult.outcome =
                        BooleanPreviewWorkerOutcome::Cancelled;
                    return aResult;
                }
                aFuse.SimplifyResult();
                if (wasCancelled()) {
                    aResult.outcome =
                        BooleanPreviewWorkerOutcome::Cancelled;
                    return aResult;
                }
                if (!IsValidSolidBooleanResult(
                        aFuse.Shape(),
                        theRequest.maximumResultTopologyNodes,
                        theRequest.maximumResultSolids,
                        anAggregateResultNodes,
                        anAggregateResultSolids)) {
                    return aResult;
                }
                aResult.results.push_back(aFuse.Shape());
            } else if (theRequest.action
                       == BooleanAction::BooleanIntersect) {
                if (!theRequest.actors.empty()
                    || theRequest.subjects.size() < 2) {
                    return aResult;
                }

                // OCCT Common is defined between argument and tool groups.
                // Fold deterministically so three or more subjects mean a
                // true N-way intersection rather than an intersection of two
                // unions. Each intermediate is bounded and validated before
                // it can become the next operand.
                TopoDS_Shape aCurrent = theRequest.subjects.front();
                for (std::size_t anIndex = 1;
                     anIndex < theRequest.subjects.size(); ++anIndex) {
                    if (wasCancelled()) {
                        aResult.outcome =
                            BooleanPreviewWorkerOutcome::Cancelled;
                        aResult.results.clear();
                        return aResult;
                    }
                    TopTools_ListOfShape anArguments;
                    TopTools_ListOfShape aTools;
                    anArguments.Append(aCurrent);
                    aTools.Append(theRequest.subjects[anIndex]);
                    BRepAlgoAPI_Common aCommon;
                    aCommon.SetRunParallel(Standard_False);
                    aCommon.SetNonDestructive(Standard_True);
                    aCommon.SetArguments(anArguments);
                    aCommon.SetTools(aTools);
                    aCommon.SetFuzzyValue(kFuzzyValue);
                    aCommon.SetUseOBB(Standard_True);
                    aCommon.SetCheckInverted(Standard_True);
                    aCommon.Build(aProgress->Start());
                    if (wasCancelled()) {
                        aResult.outcome =
                            BooleanPreviewWorkerOutcome::Cancelled;
                        aResult.results.clear();
                        return aResult;
                    }
                    if (!aCommon.IsDone()) {
                        return aResult;
                    }
                    Standard_Size aRawNodes = 0;
                    if (!AccumulateBoundedTopology(
                            aCommon.Shape(),
                            theRequest.maximumResultTopologyNodes,
                            theRequest.maximumResultTopologyNodes,
                            aRawNodes)) {
                        return aResult;
                    }
                    if (wasCancelled()) {
                        aResult.outcome =
                            BooleanPreviewWorkerOutcome::Cancelled;
                        aResult.results.clear();
                        return aResult;
                    }
                    aCommon.SimplifyResult();
                    if (wasCancelled()) {
                        aResult.outcome =
                            BooleanPreviewWorkerOutcome::Cancelled;
                        aResult.results.clear();
                        return aResult;
                    }
                    Standard_Size aStepResultNodes = 0;
                    Standard_Size aStepResultSolids = 0;
                    if (!IsValidSolidBooleanResult(
                            aCommon.Shape(),
                            theRequest.maximumResultTopologyNodes,
                            theRequest.maximumResultSolids,
                            aStepResultNodes,
                            aStepResultSolids)) {
                        return aResult;
                    }
                    aCurrent = aCommon.Shape();
                }
                aResult.results.push_back(aCurrent);
            } else {
                return aResult;
            }
            if (wasCancelled()) {
                aResult.outcome = BooleanPreviewWorkerOutcome::Cancelled;
                aResult.results.clear();
                return aResult;
            }
            aResult.outcome = BooleanPreviewWorkerOutcome::Success;
            return aResult;
        } catch (...) {
            if (wasCancelled()) {
                aResult.outcome = BooleanPreviewWorkerOutcome::Cancelled;
            }
            aResult.results.clear();
            return aResult;
        }
    }

private:
    std::weak_ptr<BooleanOperationController> myOwner;
    mutable std::mutex myMutex;
    std::condition_variable myCondition;
    std::optional<BooleanPreviewWorkerRequest> myPending;
    std::shared_ptr<std::atomic_bool> myActiveCancellation;
    bool myDrainScheduled = false;
    bool myStopped = false;
    std::uint64_t mySubmittedCount = 0;
    std::uint64_t myStartedCount = 0;
    std::uint64_t myCompletedCount = 0;
    std::uint64_t myCancelledCount = 0;
    std::uint64_t myPendingReplacementCount = 0;
    Standard_Boolean myLastComputeWasMainThread = Standard_False;
#ifdef DEBUG
    bool myDebugBlocked = false;
#endif
};

BooleanOperationController::BooleanOperationController(
    Handle(AIS_InteractiveContext) theContext,
    Handle(OcctDocument) theDocument)
: myContext(std::move(theContext)),
  myDoc(std::move(theDocument))
{
    _controllerEpoch = gBooleanControllerEpoch.fetch_add(
        1, std::memory_order_relaxed) + 1;
    try {
        _ownedPresentations.reserve(kMaxSourceOperands * 3);
        _subjectSelectionOrder.reserve(kMaxSourceOperands);
    } catch (...) {
        _stateValid = Standard_False;
    }
}

BooleanOperationController::~BooleanOperationController() noexcept
{
    ++_previewGeneration;
    if (_previewWorker != nullptr) {
        _previewWorker->shutdown();
        _previewWorker.reset();
    }
    _previewStateChangedCallback = {};
}

Standard_Boolean BooleanOperationController::begin(
    const BooleanAction theAction) noexcept
{
    try {
        if (!IsSupportedBooleanAction(theAction)
            || myContext.IsNull() || myDoc.IsNull()) {
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
        _previewState = BooleanPreviewState::Selecting;
        _requestedFingerprint.clear();
        notifyPreviewStateChanged();
        return Standard_True;
    } catch (...) {
        markInvalid();
        return Standard_False;
    }
}

Standard_Boolean BooleanOperationController::actionMatches(
    const BooleanAction theAction) const noexcept
{
    return IsSupportedBooleanAction(theAction)
        && _activeAction.has_value() && *_activeAction == theAction;
}

Standard_Boolean BooleanOperationController::canApply() const noexcept
{
    return _canApply && _stateValid && _activeAction.has_value()
        && _previewState == BooleanPreviewState::Ready;
}

BooleanPreviewState BooleanOperationController::previewState() const noexcept
{
    return _previewState;
}

void BooleanOperationController::setPreviewStateChangedCallback(
    std::function<void()> theCallback)
{
    _previewStateChangedCallback = std::move(theCallback);
}

void BooleanOperationController::notifyPreviewStateChanged() noexcept
{
    try {
        if (_previewStateChangedCallback) {
            _previewStateChangedCallback();
        }
    } catch (...) {
    }
}

#ifdef DEBUG
Standard_Boolean BooleanOperationController::debugValidateSolidResult(
    const TopoDS_Shape& theShape) noexcept
{
    Standard_Size anAggregateTopologyNodes = 0;
    Standard_Size anAggregateSolidCount = 0;
    return IsValidSolidBooleanResult(
        theShape,
        kMaxResultTopologyNodes,
        kMaxResultSolids,
        anAggregateTopologyNodes,
        anAggregateSolidCount);
}

BooleanPreviewDebugState
BooleanOperationController::debugPreviewState() const noexcept
{
    BooleanPreviewDebugState aState;
    aState.state = _previewState;
    aState.generation = _previewGeneration;
    aState.activeOperation = _activeAction.has_value();
    aState.canApply = canApply();
    aState.documentCommandUnresolved = _documentCommandUnresolved;
    try {
        if (!myDoc.IsNull()) {
            const Handle(TDocStd_Document) aDocument =
                myDoc->ChangeDocument();
            aState.documentCommandOpen = !aDocument.IsNull()
                && aDocument->HasOpenCommand();
        }
    } catch (...) {
        aState.documentCommandOpen = Standard_True;
    }
    aState.acceptedCount = _debugAcceptedCount;
    aState.staleSuppressionCount = _debugStaleSuppressionCount;
    if (_previewWorker != nullptr) {
        const BooleanPreviewWorker::DebugSnapshot aWorker =
            _previewWorker->debugSnapshot();
        aState.submittedCount = aWorker.submittedCount;
        aState.startedCount = aWorker.startedCount;
        aState.completedCount = aWorker.completedCount;
        aState.cancelledCount = aWorker.cancelledCount;
        aState.pendingReplacementCount =
            aWorker.pendingReplacementCount;
        aState.workerActive = aWorker.active;
        aState.workerPending = aWorker.pending;
        aState.lastComputeWasMainThread =
            aWorker.lastComputeWasMainThread;
    }
    return aState;
}

void BooleanOperationController::debugSetWorkerBlocked(
    const Standard_Boolean theBlocked) noexcept
{
    _debugWorkerBlocked = theBlocked;
    if (_previewWorker != nullptr) {
        _previewWorker->debugSetBlocked(theBlocked);
    }
}

void BooleanOperationController::debugSetMaximumCaptureTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    _debugMaximumCaptureTopologyNodes = std::max<Standard_Size>(
        1, std::min(theLimit, kMaxCaptureTopologyNodes));
}

void BooleanOperationController::debugSetMaximumResultTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    _debugMaximumResultTopologyNodes = std::max<Standard_Size>(
        1, std::min(theLimit, kMaxResultTopologyNodes));
}

void BooleanOperationController::debugSetMaximumResultSolids(
    const Standard_Size theLimit) noexcept
{
    _debugMaximumResultSolids = std::max<Standard_Size>(
        1, std::min(theLimit, kMaxResultSolids));
}

void BooleanOperationController::debugSetTransactionFailureCount(
    const Standard_Size theCount) noexcept
{
    _debugTransactionFailureCount = theCount;
}

void BooleanOperationController::debugSetAbortFailureCount(
    const Standard_Size theCount) noexcept
{
    _debugAbortFailureCount = theCount;
}
#endif

Standard_Boolean BooleanOperationController::hasActiveOperation() const noexcept
{
    return _activeAction.has_value();
}

Standard_Boolean BooleanOperationController::hasActiveOperation(
    const BooleanAction theAction) const noexcept
{
    return actionMatches(theAction);
}

Standard_Boolean BooleanOperationController::hasSelectionState() const noexcept
{
    return !_selectionMap.empty() || !_singleTrialResult.IsNull();
}

Standard_Boolean BooleanOperationController::hasUnresolvedState() const noexcept
{
    return !_stateValid || _documentCommandUnresolved;
}

Standard_Boolean BooleanOperationController::isSelectionFrozen() const noexcept
{
    return _selectionFrozen && canApply()
        && _activeAction.has_value()
        && IsSingleResultBooleanAction(*_activeAction);
}

void BooleanOperationController::markInvalid() noexcept
{
    cancelWorkerRequests();
    _stateValid = Standard_False;
    _canApply = Standard_False;
    _selectionFrozen = Standard_True;
    _previewState = BooleanPreviewState::Failed;
    notifyPreviewStateChanged();
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
            && (!_singleTrialResult.IsNull()
                && anOwned == _singleTrialResult);
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
    const std::optional<BooleanAction> anAction = _activeAction;
    const Standard_Boolean didAbort =
        abortDocumentCommandNoThrow(theDocument);
    cancelImpl();
    if (!didAbort) {
        // Visual state can still be restored, but keep the controller
        // unresolved until a later cancel retries and confirms that OCAF no
        // longer has the Boolean command open.
        _activeAction = anAction;
        _documentCommandUnresolved = Standard_True;
        markInvalid();
    }
}

Standard_Boolean BooleanOperationController::abortDocumentCommandNoThrow(
    const Handle(TDocStd_Document)& theDocument) noexcept
{
#ifdef DEBUG
    if (_debugAbortFailureCount > 0) {
        --_debugAbortFailureCount;
        return Standard_False;
    }
#endif
    return AbortCommandNoThrow(theDocument);
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
            const OcctGeometryRepresentation aRepresentation =
                myDoc->GeometryRepresentationForLabel(
                    aState.documentLabel);
            if (aState.documentLabel.IsNull()
                || (aRepresentation
                        != OcctGeometryRepresentation::LegacyUnknown
                    && aRepresentation
                        != OcctGeometryRepresentation::BRep)) {
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
        || (IsSingleResultBooleanAction(theAction)
            && theType != BooleanSelectionType::Subject)
        || _selectionMap.size() >= kMaxSourceOperands) {
        return Standard_False;
    }
    try {
        const Handle(AIS_Shape) aShape =
            Handle(AIS_Shape)::DownCast(theObject);
        const TDF_Label aLabel = myDoc->ShapeLabel(theObject);
        const OcctGeometryRepresentation aRepresentation =
            myDoc->GeometryRepresentationForLabel(aLabel);
        if (aShape.IsNull() || aShape->Shape().IsNull()
            || aLabel.IsNull()
            || (aRepresentation
                    != OcctGeometryRepresentation::LegacyUnknown
                && aRepresentation != OcctGeometryRepresentation::BRep)) {
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

std::string BooleanOperationController::currentSelectionFingerprint(
    const BooleanAction theAction,
    Standard_Boolean& theIsComplete) const
{
    theIsComplete = Standard_False;
    try {
        struct Entry {
            std::string identifier;
            BooleanSelectionType role = BooleanSelectionType::Undefined;
            Handle(AIS_InteractiveObject) presentation;
        };
        std::vector<Entry> anActors;
        std::vector<Entry> aSubjects;
        for (const auto& aSelection : _selectionMap) {
            const std::string anIdentifier =
                myDoc->EntityIdentifierForLabel(
                    aSelection.second.documentLabel);
            if (anIdentifier.empty()
                || aSelection.first.IsNull()
                || Handle(AIS_Shape)::DownCast(
                    aSelection.first).IsNull()) {
                return {};
            }
            if (aSelection.second.selectionType
                == BooleanSelectionType::Actor) {
                anActors.push_back({
                    anIdentifier,
                    BooleanSelectionType::Actor,
                    aSelection.second.original,
                });
            }
        }
        std::sort(
            anActors.begin(), anActors.end(),
            [](const Entry& theLeft, const Entry& theRight) {
                return theLeft.identifier < theRight.identifier;
            });
        for (const TDF_Label& aSubjectLabel : _subjectSelectionOrder) {
            Entry aMatch;
            bool hasMatch = false;
            for (const auto& aSelection : _selectionMap) {
                if (aSelection.second.selectionType
                        != BooleanSelectionType::Subject
                    || !aSelection.second.documentLabel.IsEqual(
                        aSubjectLabel)) {
                    continue;
                }
                if (hasMatch) {
                    return {};
                }
                aMatch.identifier =
                    myDoc->EntityIdentifierForLabel(aSubjectLabel);
                aMatch.role = BooleanSelectionType::Subject;
                aMatch.presentation = aSelection.second.original;
                hasMatch = true;
            }
            if (!hasMatch || aMatch.identifier.empty()) {
                return {};
            }
            aSubjects.push_back(std::move(aMatch));
        }
        if (anActors.size() + aSubjects.size()
                != _selectionMap.size()
            || (theAction == BooleanAction::BooleanSubtract
                && (anActors.empty() || aSubjects.empty()))
            || (IsSingleResultBooleanAction(theAction)
                && (!anActors.empty() || aSubjects.size() < 2))) {
            return {};
        }

        std::ostringstream aFingerprint;
        aFingerprint << static_cast<int>(theAction)
            << ':' << _controllerEpoch
            << ':' << reinterpret_cast<std::uintptr_t>(
                myDoc->Document().get());
        const auto append = [&](const Entry& theEntry) {
            const Handle(AIS_Shape) aShape =
                Handle(AIS_Shape)::DownCast(theEntry.presentation);
            if (aShape.IsNull() || aShape->Shape().IsNull()) {
                return false;
            }
            aFingerprint << '|' << static_cast<int>(theEntry.role)
                << ':' << theEntry.identifier
                << ':' << reinterpret_cast<std::uintptr_t>(
                    aShape->Shape().TShape().get());
            const gp_Trsf aTransform =
                theEntry.presentation->LocalTransformation();
            aFingerprint << std::setprecision(17);
            for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
                for (Standard_Integer aColumn = 1;
                     aColumn <= 4; ++aColumn) {
                    aFingerprint << ':'
                        << aTransform.Value(aRow, aColumn);
                }
            }
            return true;
        };
        for (const Entry& anActor : anActors) {
            if (!append(anActor)) {
                return {};
            }
        }
        for (const Entry& aSubject : aSubjects) {
            if (!append(aSubject)) {
                return {};
            }
        }
        theIsComplete = Standard_True;
        return aFingerprint.str();
    } catch (...) {
        return {};
    }
}

void BooleanOperationController::cancelWorkerRequests() noexcept
{
    if (_previewGeneration
        != std::numeric_limits<std::uint64_t>::max()) {
        ++_previewGeneration;
    }
    if (_previewWorker != nullptr) {
        _previewWorker->cancelAll();
    }
    _requestedFingerprint.clear();
    _canApply = Standard_False;
}

void BooleanOperationController::supersedeWorkerRequests() noexcept
{
    if (_previewGeneration
        != std::numeric_limits<std::uint64_t>::max()) {
        ++_previewGeneration;
    }
    if (_previewWorker != nullptr) {
        // Preserve one already-pending request until enqueue atomically
        // replaces it. If admission fails, visualApply's retirement guard
        // clears that obsolete pending request before returning.
        _previewWorker->cancelActiveForSupersede();
    }
    _requestedFingerprint.clear();
    _canApply = Standard_False;
}

Standard_Boolean BooleanOperationController::enqueuePreviewRequest(
    const BooleanAction theAction) noexcept
{
    try {
        Standard_Boolean hasCompleteFingerprint = Standard_False;
        const std::string aFingerprint = currentSelectionFingerprint(
            theAction, hasCompleteFingerprint);
        if (!hasCompleteFingerprint || aFingerprint.empty()
            || _actorIOArray.size() + _actedIOArray.size()
                > kMaxSourceOperands) {
            return Standard_False;
        }

        Standard_Size anAggregateTopologyNodes = 0;
#ifdef DEBUG
        const Standard_Size anAggregateLimit =
            _debugMaximumCaptureTopologyNodes;
        const Standard_Size aPerSourceLimit = std::min(
            kMaxSourceTopologyNodes,
            _debugMaximumCaptureTopologyNodes);
#else
        const Standard_Size anAggregateLimit =
            kMaxCaptureTopologyNodes;
        const Standard_Size aPerSourceLimit =
            kMaxSourceTopologyNodes;
#endif
        const auto countSource = [&](
            const Handle(AIS_InteractiveObject)& thePresentation) {
            const Handle(AIS_Shape) aShape =
                Handle(AIS_Shape)::DownCast(thePresentation);
            return !aShape.IsNull() && !aShape->Shape().IsNull()
                && AccumulateBoundedTopology(
                    aShape->Shape(),
                    aPerSourceLimit,
                    anAggregateLimit,
                    anAggregateTopologyNodes);
        };
        for (const Handle(AIS_InteractiveObject)& anActor :
             _actorIOArray) {
            if (!countSource(anActor)) {
                return Standard_False;
            }
        }
        for (const Handle(AIS_InteractiveObject)& aSubject :
             _actedIOArray) {
            if (!countSource(aSubject)) {
                return Standard_False;
            }
        }

        BooleanPreviewWorkerRequest aRequest;
        aRequest.action = theAction;
        if (_previewGeneration
            == std::numeric_limits<std::uint64_t>::max()) {
            return Standard_False;
        }
        aRequest.generation = ++_previewGeneration;
        aRequest.fingerprint = aFingerprint;
        aRequest.cancellation =
            std::make_shared<std::atomic_bool>(false);
#ifdef DEBUG
        aRequest.maximumResultTopologyNodes =
            _debugMaximumResultTopologyNodes;
        aRequest.maximumResultSolids = _debugMaximumResultSolids;
#endif
        const auto deepCopyWorldShape = [](
            const Handle(AIS_InteractiveObject)& thePresentation) {
            const Handle(AIS_Shape) aShape =
                Handle(AIS_Shape)::DownCast(thePresentation);
            if (aShape.IsNull() || aShape->Shape().IsNull()) {
                return TopoDS_Shape();
            }
            BRepBuilderAPI_Transform aTransform(
                aShape->Shape(),
                thePresentation->LocalTransformation(),
                Standard_True,
                Standard_False);
            return aTransform.Shape();
        };
        aRequest.actors.reserve(_actorIOArray.size());
        aRequest.subjects.reserve(_actedIOArray.size());
        for (const Handle(AIS_InteractiveObject)& anActor :
             _actorIOArray) {
            TopoDS_Shape aCopy = deepCopyWorldShape(anActor);
            if (aCopy.IsNull()) {
                return Standard_False;
            }
            aRequest.actors.push_back(std::move(aCopy));
        }
        for (const Handle(AIS_InteractiveObject)& aSubject :
             _actedIOArray) {
            TopoDS_Shape aCopy = deepCopyWorldShape(aSubject);
            if (aCopy.IsNull()) {
                return Standard_False;
            }
            aRequest.subjects.push_back(std::move(aCopy));
        }

        if (_previewWorker == nullptr) {
            _previewWorker = std::make_shared<BooleanPreviewWorker>(
                weak_from_this());
#ifdef DEBUG
            _previewWorker->debugSetBlocked(_debugWorkerBlocked);
#endif
        }
        _requestedFingerprint = aFingerprint;
        _previewState = BooleanPreviewState::Computing;
        _selectionFrozen = Standard_False;
        _canApply = Standard_False;
        if (!_previewWorker->enqueue(std::move(aRequest))) {
            _previewState = BooleanPreviewState::Failed;
            _requestedFingerprint.clear();
            notifyPreviewStateChanged();
            return Standard_False;
        }
        notifyPreviewStateChanged();
        return Standard_True;
    } catch (...) {
        _previewState = BooleanPreviewState::Failed;
        _canApply = Standard_False;
        notifyPreviewStateChanged();
        return Standard_False;
    }
}

Standard_Boolean BooleanOperationController::visualApply(
    const BooleanAction theAction) noexcept
{
    if (!actionMatches(theAction) || !_stateValid) {
        _canApply = Standard_False;
        return Standard_False;
    }
    if (_selectionFrozen) {
        return Standard_False;
    }
    // Retire the active generation before any admission branch. Preserve at
    // most one pending request so a successfully admitted request can replace
    // it atomically in enqueue(); a failed admission clears it via the guard.
    supersedeWorkerRequests();
    const std::shared_ptr<BooleanPreviewWorker> aPriorWorker = _previewWorker;
    bool didScheduleReplacement = false;
    struct PendingRetirementGuard {
        std::shared_ptr<BooleanPreviewWorker> worker;
        bool& didSchedule;
        ~PendingRetirementGuard() noexcept
        {
            if (!didSchedule && worker != nullptr) {
                worker->cancelAll();
            }
        }
    } aPendingRetirement{aPriorWorker, didScheduleReplacement};
    _previewState = BooleanPreviewState::Selecting;
    if (_selectionMap.empty()
        || _selectionMap.size() > kMaxSourceOperands) {
        _previewState = BooleanPreviewState::Failed;
        notifyPreviewStateChanged();
        return Standard_False;
    }

    // Admission precedes resetCachedSelection(), whose world-space AIS copies
    // duplicate geometry. Count the immutable committed sources first so a
    // huge input cannot make capture itself an unbounded main-thread task.
    try {
        Standard_Size anAggregateTopologyNodes = 0;
#ifdef DEBUG
        const Standard_Size anAggregateLimit =
            _debugMaximumCaptureTopologyNodes;
        const Standard_Size aPerSourceLimit = std::min(
            kMaxSourceTopologyNodes,
            _debugMaximumCaptureTopologyNodes);
#else
        const Standard_Size anAggregateLimit =
            kMaxCaptureTopologyNodes;
        const Standard_Size aPerSourceLimit =
            kMaxSourceTopologyNodes;
#endif
        for (const auto& aSelection : _selectionMap) {
            const Handle(AIS_Shape) aSource =
                Handle(AIS_Shape)::DownCast(
                    aSelection.second.original);
            if (aSource.IsNull() || aSource->Shape().IsNull()
                || !IsCurrentBRepSelection(
                    myDoc, aSelection.second)
                || !AccumulateBoundedTopology(
                    aSource->Shape(),
                    aPerSourceLimit,
                    anAggregateLimit,
                    anAggregateTopologyNodes)) {
                _previewState = BooleanPreviewState::Failed;
                notifyPreviewStateChanged();
                return Standard_False;
            }
        }
    } catch (...) {
        _previewState = BooleanPreviewState::Failed;
        notifyPreviewStateChanged();
        return Standard_False;
    }
    if (!resetCachedSelection()) {
        _previewState = BooleanPreviewState::Failed;
        notifyPreviewStateChanged();
        return Standard_False;
    }

    try {
        _actedIOArray.clear();
        _actorIOArray.clear();
        for (const auto& aSelection : _selectionMap) {
            if (aSelection.second.documentLabel.IsNull()) {
                _previewState = BooleanPreviewState::Failed;
                notifyPreviewStateChanged();
                return Standard_False;
            }
            if (aSelection.second.selectionType
                == BooleanSelectionType::Actor) {
                _actorIOArray.push_back(aSelection.first);
            }
        }
        std::sort(
            _actorIOArray.begin(), _actorIOArray.end(),
            [&](const Handle(AIS_InteractiveObject)& theLeft,
                const Handle(AIS_InteractiveObject)& theRight) {
                return myDoc->EntityIdentifierForLabel(
                    _selectionMap.find(theLeft)->second.documentLabel)
                    < myDoc->EntityIdentifierForLabel(
                        _selectionMap.find(theRight)->second.documentLabel);
            });
        Standard_Boolean hasCompleteSubjects = Standard_False;
        _actedIOArray = orderedSubjectPresentations(hasCompleteSubjects);
        if (!hasCompleteSubjects
            || (theAction == BooleanAction::BooleanSubtract
                && (_actorIOArray.empty() || _actedIOArray.empty()))
            || (IsSingleResultBooleanAction(theAction)
                && (!_actorIOArray.empty() || _actedIOArray.size() < 2))
            || _actorIOArray.size() + _actedIOArray.size()
                > kMaxSourceOperands) {
            _previewState = BooleanPreviewState::Failed;
            notifyPreviewStateChanged();
            return Standard_False;
        }
        const Standard_Boolean didSchedule =
            enqueuePreviewRequest(theAction);
        didScheduleReplacement = didSchedule != Standard_False;
        if (!didSchedule) {
            _previewState = BooleanPreviewState::Failed;
            notifyPreviewStateChanged();
        }
        return didSchedule;
    } catch (...) {
        markInvalid();
        return Standard_False;
    }
}

void BooleanOperationController::acceptWorkerResult(
    BooleanPreviewWorkerResult theResult) noexcept
{
    Standard_Boolean hasCompleteFingerprint = Standard_False;
    const std::string aCurrentFingerprint = currentSelectionFingerprint(
        theResult.action, hasCompleteFingerprint);
    bool hasCurrentBRepSources = !_selectionMap.empty();
    for (const auto& aSelection : _selectionMap) {
        if (!IsCurrentBRepSelection(myDoc, aSelection.second)) {
            hasCurrentBRepSources = false;
            break;
        }
    }
    if (pthread_main_np() == 0
        || !_activeAction.has_value()
        || *_activeAction != theResult.action
        || _previewState != BooleanPreviewState::Computing
        || theResult.generation != _previewGeneration
        || theResult.fingerprint != _requestedFingerprint
        || !hasCompleteFingerprint
        || aCurrentFingerprint != theResult.fingerprint
        || !hasCurrentBRepSources) {
#ifdef DEBUG
        ++_debugStaleSuppressionCount;
#endif
        return;
    }
    if (theResult.outcome != BooleanPreviewWorkerOutcome::Success) {
        _canApply = Standard_False;
        _selectionFrozen = Standard_False;
        _previewState = theResult.outcome
                == BooleanPreviewWorkerOutcome::Cancelled
            ? BooleanPreviewState::Selecting
            : BooleanPreviewState::Failed;
        notifyPreviewStateChanged();
        return;
    }

    Standard_Boolean didInstall = Standard_False;
    if (theResult.action == BooleanAction::BooleanSubtract) {
        didInstall = installSubtractPreview(theResult.results);
    } else if (IsSingleResultBooleanAction(theResult.action)
               && theResult.results.size() == 1) {
        didInstall = installSingleResultPreview(
            theResult.results.front());
    }
    if (!didInstall || !_stateValid || !pruneOwnedPresentations()) {
        _canApply = Standard_False;
        _previewState = BooleanPreviewState::Failed;
        notifyPreviewStateChanged();
        return;
    }
    _canApply = Standard_True;
    _previewState = BooleanPreviewState::Ready;
#ifdef DEBUG
    ++_debugAcceptedCount;
#endif
    notifyPreviewStateChanged();
}

Standard_Boolean BooleanOperationController::installSubtractPreview(
    const std::vector<TopoDS_Shape>& theResults) noexcept
{
    struct Preview {
        Handle(AIS_InteractiveObject) source;
        Handle(AIS_InteractiveObject) result;
        TemporalBooleanObject state;
    };
    std::vector<Preview> aPreviews;
    const auto forgetOwnedResult = [&](
        const Handle(AIS_InteractiveObject)& theResult) noexcept {
        _ownedPresentations.erase(
            std::remove(
                _ownedPresentations.begin(),
                _ownedPresentations.end(),
                theResult),
            _ownedPresentations.end());
    };
    const auto rollbackPresentation = [&]() noexcept {
        bool didRollbackAll = true;
        for (const Preview& aPreview : aPreviews) {
            bool didRemove = aPreview.result.IsNull();
            if (!didRemove) {
                try {
                    myContext->Remove(
                        aPreview.result, Standard_False);
                    didRemove = true;
                } catch (...) {
                    didRollbackAll = false;
                }
            }
            if (didRemove) {
                forgetOwnedResult(aPreview.result);
            }
            // Every result is registered before display, so a failed Remove
            // remains owned and explicit Cancel can retry it.
        }
        for (const Handle(AIS_InteractiveObject)& aSubject :
             _actedIOArray) {
            try {
                showInteractiveByType(
                    aSubject, BooleanSelectionType::Subject);
            } catch (...) {
                didRollbackAll = false;
            }
        }
        try {
            myContext->UpdateCurrentViewer();
        } catch (...) {
            didRollbackAll = false;
        }
        if (!didRollbackAll) {
            markInvalid();
        }
        return didRollbackAll;
    };

    try {
        if (theResults.size() != _actedIOArray.size()
            || theResults.empty()) {
            return Standard_False;
        }
        aPreviews.reserve(_actedIOArray.size());
        for (std::size_t anIndex = 0;
             anIndex < _actedIOArray.size(); ++anIndex) {
            const Handle(AIS_InteractiveObject)& aSubjectIO =
                _actedIOArray[anIndex];
            const auto aSelection = _selectionMap.find(aSubjectIO);
            if (aSelection == _selectionMap.end()
                || aSelection->second.documentLabel.IsNull()
                || theResults[anIndex].IsNull()) {
                return Standard_False;
            }
            TemporalBooleanObject aState = aSelection->second;
            aState.selectionType = BooleanSelectionType::Subject;
            Handle(AIS_InteractiveObject) aResult =
                new AIS_Shape(theResults[anIndex]);
            applyStyle(aResult, aState);
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
        // Register before the first AIS mutation. A display/remove exception
        // can therefore never leave a visible preview outside cancellation's
        // ownership set.
        for (const Preview& aPreview : aPreviews) {
            rememberOwned(aPreview.result);
            if (!_stateValid) {
                rollbackPresentation();
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
        myContext->UpdateCurrentViewer();
        // Publish controller ownership only after every fallible AIS step has
        // completed. If UpdateCurrentViewer throws, rollback still sees the
        // source-keyed map that matches the presentations it restores.
        _selectionMap.swap(aReplacement);
        return Standard_True;
    } catch (const Standard_Failure& aFailure) {
        std::cout << "Boolean subtract preview failure: "
                  << aFailure.GetMessageString() << std::endl;
        rollbackPresentation();
        return Standard_False;
    } catch (...) {
        rollbackPresentation();
        return Standard_False;
    }
}

Standard_Boolean BooleanOperationController::installSingleResultPreview(
    const TopoDS_Shape& theResult) noexcept
{
    Handle(AIS_InteractiveObject) aResult;
    const auto rollbackPresentation = [&]() noexcept {
        bool didRollbackAll = true;
        bool didRemoveResult = aResult.IsNull();
        if (!didRemoveResult) {
            try {
                myContext->Remove(aResult, Standard_False);
                didRemoveResult = true;
            } catch (...) {
                didRollbackAll = false;
            }
        }
        if (didRemoveResult) {
            _ownedPresentations.erase(
                std::remove(
                    _ownedPresentations.begin(),
                    _ownedPresentations.end(),
                    aResult),
                _ownedPresentations.end());
            _singleTrialResult.Nullify();
        }
        // A failed result removal deliberately leaves the handle owned. The
        // next Cancel retries it instead of losing track of a ghost preview.
        for (const Handle(AIS_InteractiveObject)& aSubject :
             _actedIOArray) {
            try {
                showInteractiveByType(
                    aSubject, BooleanSelectionType::Subject);
            } catch (...) {
                didRollbackAll = false;
            }
        }
        try {
            myContext->UpdateCurrentViewer();
        } catch (...) {
            didRollbackAll = false;
        }
        _selectionFrozen = Standard_False;
        if (!didRollbackAll) {
            markInvalid();
        }
        return didRollbackAll;
    };
    try {
        if (theResult.IsNull() || _actedIOArray.size() < 2
            || _actedIOArray.size() > kMaxSourceOperands) {
            return Standard_False;
        }
        for (const Handle(AIS_InteractiveObject)& aSubject :
             _actedIOArray) {
            if (_selectionMap.find(aSubject) == _selectionMap.end()) {
                return Standard_False;
            }
        }
        const TemporalBooleanObject aResultStyle =
            _selectionMap.find(_actedIOArray.front())->second;
        aResult = new AIS_Shape(theResult);
        applyStyle(aResult, aResultStyle);
        rememberOwned(aResult);
        if (!_stateValid) {
            rollbackPresentation();
            return Standard_False;
        }

        showInteractiveByType(aResult, BooleanSelectionType::Subject);
        myContext->ClearSelected(Standard_False);
        myContext->SetSelected(aResult, Standard_False);
        myContext->Deactivate(aResult);
        for (const Handle(AIS_InteractiveObject)& aSubject :
             _actedIOArray) {
            myContext->Remove(aSubject, Standard_False);
        }
        _singleTrialResult = Handle(AIS_Shape)::DownCast(aResult);
        if (_singleTrialResult.IsNull()) {
            rollbackPresentation();
            return Standard_False;
        }
        _selectionFrozen = Standard_True;
        myContext->UpdateCurrentViewer();
        return Standard_True;
    } catch (const Standard_Failure& aFailure) {
        std::cout << "Boolean single-result preview failure: "
                  << aFailure.GetMessageString() << std::endl;
        rollbackPresentation();
        return Standard_False;
    } catch (...) {
        rollbackPresentation();
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
		for (const auto& aSelection : _selectionMap) {
			if (!IsCurrentBRepSelection(myDoc, aSelection.second)) {
				return Standard_False;
			}
		}

        BooleanPreviewCapture aCapture;
        aCapture.action = *_activeAction;
        aCapture.suppressedSourceLabels = std::move(aLabels);
        if (IsSingleResultBooleanAction(*_activeAction)) {
            if (!_selectionFrozen || _singleTrialResult.IsNull()
                || !myContext->IsDisplayed(_singleTrialResult)) {
                return Standard_False;
            }
            aCapture.results.push_back(_singleTrialResult);
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
        theOriginal->LocalTransformation(),
        Standard_True,
        Standard_False).Shape();
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
    // Computing Apply is an intentional no-op. The worker and tool stay live;
    // only a Ready preview may enter the document transaction.
    if (_previewState != BooleanPreviewState::Ready
        || !canApply() || _selectionMap.empty()) {
        return BooleanApplyResult::NoChange;
    }

    const auto failWithoutMutation = [&]() {
        _canApply = Standard_False;
        _previewState = BooleanPreviewState::Failed;
        notifyPreviewStateChanged();
        return BooleanApplyResult::NoChange;
    };
    Standard_Boolean hasCurrentFingerprint = Standard_False;
    if (currentSelectionFingerprint(
            theAction, hasCurrentFingerprint)
            != _requestedFingerprint
        || !hasCurrentFingerprint) {
        return failWithoutMutation();
    }

    Standard_Boolean hasCompleteLabels = Standard_False;
    const std::vector<TDF_Label> aSourceLabels =
        orderedSourceLabels(hasCompleteLabels);
    if (!hasCompleteLabels || aSourceLabels.size() < 2
        || aSourceLabels.size() > kMaxSourceOperands) {
        return failWithoutMutation();
    }
	for (const auto& aSelection : _selectionMap) {
		if (!IsCurrentBRepSelection(myDoc, aSelection.second)) {
			return failWithoutMutation();
		}
	}

    Handle(TDocStd_Document) aDocument;
    try {
        aDocument = myDoc->ChangeDocument();
    } catch (...) {
        return failWithoutMutation();
    }
    try {
        if (aDocument.IsNull() || aDocument->HasOpenCommand()) {
            return failWithoutMutation();
        }
    } catch (...) {
        return failWithoutMutation();
    }

    _canApply = Standard_False;
    _previewState = BooleanPreviewState::Committing;
    notifyPreviewStateChanged();

    std::vector<std::pair<Handle(AIS_InteractiveObject),
                          TemporalBooleanObject>> aResults;
    try {
        if (IsSingleResultBooleanAction(theAction)) {
            Standard_Boolean hasCompleteSubjects = Standard_False;
            const auto aSubjects =
                orderedSubjectPresentations(hasCompleteSubjects);
            if (!hasCompleteSubjects || aSubjects.size() < 2
                || _singleTrialResult.IsNull()) {
                return failWithoutMutation();
            }
            aResults.push_back({
                _singleTrialResult,
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
                return failWithoutMutation();
            }
        }

        aDocument->NewCommand();
        if (!aDocument->HasOpenCommand()) {
            return failWithoutMutation();
        }
#ifdef DEBUG
        if (_debugTransactionFailureCount > 0) {
            --_debugTransactionFailureCount;
            rollbackFailedTransaction(aDocument);
            return BooleanApplyResult::NoChange;
        }
#endif
        // Persist result appearance while each source label is still present;
        // removing a source first clears the XCAF material relationship that
        // the result must inherit.
        for (const auto& aResult : aResults) {
            const TDF_Label aResultLabel = myDoc->AddShape(
                aResult.first, OcctGeometryRepresentation::BRep);
            if (aResultLabel.IsNull()) {
                rollbackFailedTransaction(aDocument);
                return BooleanApplyResult::NoChange;
            }
			if (myDoc->GeometryRepresentationForLabel(aResultLabel)
				!= OcctGeometryRepresentation::BRep) {
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
                _singleTrialResult,
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
    cancelWorkerRequests();
    _previewState = BooleanPreviewState::Selecting;
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
        if (abortDocumentCommandNoThrow(aDocument)) {
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
        notifyPreviewStateChanged();
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
        || !_ownedPresentations.empty() || !_stateValid
        || _previewState != BooleanPreviewState::Selecting) {
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
    _singleTrialResult.Nullify();
    _activeAction.reset();
    _canApply = Standard_False;
    _stateValid = Standard_True;
    _selectionFrozen = Standard_False;
    _documentCommandUnresolved = Standard_False;
    _previewState = BooleanPreviewState::Selecting;
    _requestedFingerprint.clear();
}

} // namespace core3d
