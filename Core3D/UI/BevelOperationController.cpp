//
//  BevelOperationController.cpp
//  Core3D
//

#include "BevelOperationController.hpp"

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Precision.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TDataStd_Real.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>

#include <dispatch/dispatch.h>
#include <pthread.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <utility>

namespace core3d {

enum class BevelPreviewWorkerOutcome : std::uint8_t {
    Success = 0,
    Cancelled,
    Failed,
};

struct BevelPreviewWorkerSource {
    TopoDS_Shape shape;
    std::vector<TopoDS_Edge> edges;
};

struct BevelPreviewWorkerRequest {
    std::uint64_t generation = 0;
    Standard_Real value = 0.0;
    std::string fingerprint;
    std::vector<BevelPreviewWorkerSource> sources;
    std::shared_ptr<std::atomic_bool> cancellation;
    Standard_Size maximumResultTopologyNodes =
        BevelOperationController::kMaxResultTopologyNodes;
    Standard_Size maximumResultSolids =
        BevelOperationController::kMaxResultSolids;
};

struct BevelPreviewWorkerResult {
    std::uint64_t generation = 0;
    Standard_Real value = 0.0;
    std::string fingerprint;
    BevelPreviewWorkerOutcome outcome = BevelPreviewWorkerOutcome::Failed;
    std::vector<TopoDS_Shape> results;
    Standard_Boolean computeWasMainThread = Standard_False;
};

namespace {

dispatch_queue_t BevelWorkerQueue()
{
    static dispatch_queue_t aQueue = []() {
        dispatch_queue_attr_t anAttribute =
            dispatch_queue_attr_make_with_qos_class(
                DISPATCH_QUEUE_SERIAL,
                QOS_CLASS_USER_INITIATED,
                0);
        return dispatch_queue_create(
            "com.shapeyard.core3d.bevel-preview",
            anAttribute);
    }();
    return aQueue;
}

Standard_Boolean IsBRepModelingLabel(
    const Handle(OcctDocument)& theDocument,
    const TDF_Label& theLabel) noexcept
{
    if (theDocument.IsNull() || theLabel.IsNull()
        || !theDocument->IsEditableFreeSimpleDefinitionLabel(theLabel)) {
        return Standard_False;
    }
    const OcctGeometryRepresentation aRepresentation =
        theDocument->GeometryRepresentationForLabel(theLabel);
    return aRepresentation == OcctGeometryRepresentation::LegacyUnknown
        || aRepresentation == OcctGeometryRepresentation::BRep;
}

Standard_Boolean TransformsMatch(
    const gp_Trsf& theLeft,
    const gp_Trsf& theRight) noexcept
{
    try {
        const Standard_Real aTolerance = Precision::Confusion();
        for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
            for (Standard_Integer aColumn = 1; aColumn <= 4; ++aColumn) {
                const Standard_Real aLeft = theLeft.Value(aRow, aColumn);
                const Standard_Real aRight = theRight.Value(aRow, aColumn);
                if (!std::isfinite(aLeft) || !std::isfinite(aRight)
                    || std::abs(aLeft - aRight) > aTolerance) {
                    return Standard_False;
                }
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

//! Replacing a definition shape cannot preserve style assignments whose labels
//! target the old topology. Refuse such definitions until topology-history
//! remapping is implemented. The walk is deliberately direct because XCAF
//! subshape labels are children of their owning simple-shape definition.
Standard_Boolean HasStyledXCAFSubshape(
    const Handle(TDocStd_Document)& theDocument,
    const TDF_Label& theDefinition,
    const Standard_Size theMaximumLabels) noexcept
{
    if (theDocument.IsNull() || theDefinition.IsNull()
        || theDefinition.Data() != theDocument->GetData()
        || theMaximumLabels == 0) {
        return Standard_True;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(XCAFDoc_ColorTool) aColorTool =
            XCAFDoc_DocumentTool::CheckColorTool(theDocument->Main())
                ? XCAFDoc_DocumentTool::ColorTool(theDocument->Main())
                : Handle(XCAFDoc_ColorTool)();
        const Handle(XCAFDoc_LayerTool) aLayerTool =
            XCAFDoc_DocumentTool::CheckLayerTool(theDocument->Main())
                ? XCAFDoc_DocumentTool::LayerTool(theDocument->Main())
                : Handle(XCAFDoc_LayerTool)();
        Standard_Size aLabelCount = 0;
        for (TDF_ChildIterator anItem(theDefinition, Standard_False);
             anItem.More(); anItem.Next()) {
            if (++aLabelCount > theMaximumLabels) {
                return Standard_True;
            }
            const TDF_Label& aLabel = anItem.Value();
            if (!XCAFDoc_ShapeTool::IsSubShape(aLabel)) {
                continue;
            }
            TDF_LabelSequence aLayers;
            if (aLabel.IsNull()
                || (!aColorTool.IsNull()
                    && (aColorTool->IsSet(aLabel, XCAFDoc_ColorGen)
                        || aColorTool->IsSet(aLabel, XCAFDoc_ColorSurf)
                        || aColorTool->IsSet(aLabel, XCAFDoc_ColorCurv)
                        || !XCAFDoc_ColorTool::IsVisible(aLabel)))
                || !XCAFDoc_VisMaterialTool::GetShapeMaterial(aLabel).IsNull()
                || (!aLayerTool.IsNull()
                    && aLayerTool->GetLayers(aLabel, aLayers)
                    && !aLayers.IsEmpty())) {
                return Standard_True;
            }
        }
        return Standard_False;
    } catch (...) {
        return Standard_True;
    }
}

Standard_Boolean AccumulateBoundedTopology(
    const TopoDS_Shape& theShape,
    const Standard_Size thePerShapeLimit,
    const Standard_Size theAggregateLimit,
    Standard_Size& theAggregateCount) noexcept
{
    if (theShape.IsNull() || thePerShapeLimit == 0
        || theAggregateLimit == 0) {
        return Standard_False;
    }
    try {
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
            for (TopoDS_Iterator aChild(
                     aCurrent, Standard_True, Standard_True);
                 aChild.More(); aChild.Next()) {
                if (!aVisited.Contains(aChild.Value())) {
                    if (aStack.size() >= thePerShapeLimit
                        || aStack.size() >= theAggregateLimit) {
                        return Standard_False;
                    }
                    aStack.push_back(aChild.Value());
                }
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean HasOnlySolidBranches(
    const TopoDS_Shape& theShape,
    const Standard_Size theMaximumNodes,
    const Standard_Size theMaximumSolids,
    Standard_Size& theSolidCount) noexcept
{
    theSolidCount = 0;
    if (theShape.IsNull() || theMaximumNodes == 0
        || theMaximumSolids == 0) {
        return Standard_False;
    }
    try {
        std::vector<TopoDS_Shape> aStack{theShape};
        TopTools_IndexedMapOfShape aVisited;
        Standard_Size aNodeCount = 0;
        while (!aStack.empty()) {
            const TopoDS_Shape aCurrent = aStack.back();
            aStack.pop_back();
            if (aVisited.Contains(aCurrent)) {
                continue;
            }
            aVisited.Add(aCurrent);
            if (++aNodeCount > theMaximumNodes) {
                return Standard_False;
            }
            switch (aCurrent.ShapeType()) {
                case TopAbs_SOLID:
                    if (++theSolidCount > theMaximumSolids) {
                        return Standard_False;
                    }
                    break;
                case TopAbs_COMPOUND:
                case TopAbs_COMPSOLID: {
                    Standard_Boolean hasChild = Standard_False;
                    for (TopoDS_Iterator aChild(
                             aCurrent, Standard_True, Standard_True);
                         aChild.More(); aChild.Next()) {
                        hasChild = Standard_True;
                        if (aStack.size() >= theMaximumNodes) {
                            return Standard_False;
                        }
                        aStack.push_back(aChild.Value());
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
        return theSolidCount > 0;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean IsValidSolidResult(
    const TopoDS_Shape& theShape,
    const Standard_Size theMaximumNodes,
    const Standard_Size theMaximumSolids,
    Standard_Size& theAggregateNodeCount,
    Standard_Size& theAggregateSolidCount) noexcept
{
    try {
        Standard_Size aSolidCount = 0;
        if (!AccumulateBoundedTopology(
                theShape,
                theMaximumNodes,
                theMaximumNodes,
                theAggregateNodeCount)
            || !HasOnlySolidBranches(
                theShape,
                theMaximumNodes,
                theMaximumSolids,
                aSolidCount)
            || theAggregateSolidCount > theMaximumSolids
            || aSolidCount
                > theMaximumSolids - theAggregateSolidCount) {
            return Standard_False;
        }
        BRepCheck_Analyzer anAnalyzer(theShape, Standard_True);
        if (!anAnalyzer.IsValid()) {
            return Standard_False;
        }
        theAggregateSolidCount += aSolidCount;
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

class BevelCancellationIndicator final
    : public Message_ProgressIndicator {
    DEFINE_STANDARD_RTTI_INLINE(
        BevelCancellationIndicator,
        Message_ProgressIndicator)
public:
    explicit BevelCancellationIndicator(
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

} // namespace

class BevelPreviewWorker final
    : public std::enable_shared_from_this<BevelPreviewWorker> {
public:
    explicit BevelPreviewWorker(
        std::weak_ptr<BevelOperationController> theOwner)
    : myOwner(std::move(theOwner)) {}

    ~BevelPreviewWorker() noexcept
    {
        shutdown();
    }

    Standard_Boolean enqueue(BevelPreviewWorkerRequest theRequest) noexcept
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
                auto* aContext = new std::shared_ptr<BevelPreviewWorker>(
                    shared_from_this());
                dispatch_async_f(
                    BevelWorkerQueue(),
                    aContext,
                    &BevelPreviewWorker::DrainTrampoline);
            }
            return Standard_True;
        } catch (...) {
            cancelAll();
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
        std::weak_ptr<BevelOperationController> owner;
        BevelPreviewWorkerResult result;
    };

    static void DrainTrampoline(void* theContext)
    {
        std::unique_ptr<std::shared_ptr<BevelPreviewWorker>> aContext(
            static_cast<std::shared_ptr<BevelPreviewWorker>*>(theContext));
        (*aContext)->drain();
    }

    static void DeliverTrampoline(void* theContext)
    {
        std::unique_ptr<MainCompletion> aCompletion(
            static_cast<MainCompletion*>(theContext));
        if (const auto anOwner = aCompletion->owner.lock()) {
            anOwner->acceptWorkerResult(std::move(aCompletion->result));
        }
    }

    static BevelPreviewWorkerResult compute(
        const BevelPreviewWorkerRequest& theRequest) noexcept
    {
        BevelPreviewWorkerResult aResult;
        aResult.generation = theRequest.generation;
        aResult.value = theRequest.value;
        aResult.fingerprint = theRequest.fingerprint;
        aResult.computeWasMainThread = pthread_main_np() != 0;
        const auto wasCancelled = [&]() {
            return theRequest.cancellation == nullptr
                || theRequest.cancellation->load(
                    std::memory_order_relaxed);
        };
        if (wasCancelled()) {
            aResult.outcome = BevelPreviewWorkerOutcome::Cancelled;
            return aResult;
        }

        try {
            OCC_CATCH_SIGNALS
            Handle(BevelCancellationIndicator) aProgress =
                new BevelCancellationIndicator(theRequest.cancellation);
            aResult.results.reserve(theRequest.sources.size());
            const Standard_Boolean isChamfer = theRequest.value < 0.0;
            const Standard_Real aDistance = std::abs(theRequest.value);
            Standard_Size anAggregateResultTopologyNodes = 0;
            Standard_Size anAggregateResultSolids = 0;
            for (const BevelPreviewWorkerSource& aSource :
                 theRequest.sources) {
                if (wasCancelled() || aSource.shape.IsNull()
                    || aSource.edges.empty()) {
                    aResult.outcome = BevelPreviewWorkerOutcome::Cancelled;
                    aResult.results.clear();
                    return aResult;
                }
                TopoDS_Shape aCandidate;
                if (isChamfer) {
                    BRepFilletAPI_MakeChamfer aBuilder(aSource.shape);
                    for (const TopoDS_Edge& anEdge : aSource.edges) {
                        aBuilder.Add(aDistance, anEdge);
                    }
                    aBuilder.Build(aProgress->Start());
                    if (!aBuilder.IsDone()) {
                        return aResult;
                    }
                    aCandidate = aBuilder.Shape();
                } else {
                    BRepFilletAPI_MakeFillet aBuilder(aSource.shape);
                    for (const TopoDS_Edge& anEdge : aSource.edges) {
                        aBuilder.Add(aDistance, anEdge);
                    }
                    aBuilder.Build(aProgress->Start());
                    if (!aBuilder.IsDone()) {
                        return aResult;
                    }
                    aCandidate = aBuilder.Shape();
                }
                if (wasCancelled()) {
                    aResult.outcome = BevelPreviewWorkerOutcome::Cancelled;
                    aResult.results.clear();
                    return aResult;
                }
                if (aCandidate.IsNull()
                    || aCandidate.IsSame(aSource.shape)
                    || !IsValidSolidResult(
                        aCandidate,
                        theRequest.maximumResultTopologyNodes,
                        theRequest.maximumResultSolids,
                        anAggregateResultTopologyNodes,
                        anAggregateResultSolids)) {
                    return aResult;
                }
                aResult.results.push_back(std::move(aCandidate));
            }
            if (aResult.results.size() != theRequest.sources.size()) {
                aResult.results.clear();
                return aResult;
            }
            aResult.outcome = BevelPreviewWorkerOutcome::Success;
            return aResult;
        } catch (...) {
            aResult.results.clear();
            return aResult;
        }
    }

    void drain() noexcept
    {
        for (;;) {
            BevelPreviewWorkerRequest aRequest;
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

            BevelPreviewWorkerResult aResult = compute(aRequest);
            {
                std::lock_guard<std::mutex> aLock(myMutex);
                myActiveCancellation.reset();
                ++myCompletedCount;
                myLastComputeWasMainThread = aResult.computeWasMainThread;
                if (aResult.outcome
                    == BevelPreviewWorkerOutcome::Cancelled) {
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
                    &BevelPreviewWorker::DeliverTrampoline);
            } catch (...) {
            }
        }
    }

private:
    std::weak_ptr<BevelOperationController> myOwner;
    mutable std::mutex myMutex;
    std::condition_variable myCondition;
    std::optional<BevelPreviewWorkerRequest> myPending;
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

BevelOperationController::BevelOperationController(
    Handle(AIS_InteractiveContext) theContext,
    Handle(OcctDocument) theDocument)
: myContext(std::move(theContext)),
  myDoc(std::move(theDocument))
{
    try {
        mySources.reserve(kMaxSourceBodies);
        myPreviewResults.reserve(kMaxSourceBodies);
    } catch (...) {
        myStateValid = Standard_False;
    }
}

BevelOperationController::~BevelOperationController() noexcept
{
    if (myGeneration != std::numeric_limits<std::uint64_t>::max()) {
        ++myGeneration;
    }
    if (myWorker != nullptr) {
        myWorker->shutdown();
        myWorker.reset();
    }
    myPreviewStateChangedCallback = {};
}

void BevelOperationController::setPreviewStateChangedCallback(
    std::function<void()> theCallback)
{
    myPreviewStateChangedCallback = std::move(theCallback);
}

void BevelOperationController::notifyPreviewStateChanged() noexcept
{
    try {
        if (myPreviewStateChangedCallback) {
            myPreviewStateChangedCallback();
        }
    } catch (...) {
    }
}

Standard_Boolean BevelOperationController::begin(
    const std::vector<BevelSourceSelection>& theSelection) noexcept
{
    if (myContext.IsNull() || myDoc.IsNull() || theSelection.empty()
        || theSelection.size() > kMaxSourceBodies) {
        return Standard_False;
    }
    if (hasActiveOperation() && !cancel()) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(TDocStd_Document) aDocument = myDoc->ChangeDocument();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()) {
            return Standard_False;
        }

        std::vector<Source> aSources;
        aSources.reserve(theSelection.size());
        std::set<std::string> anIdentifiers;
        Standard_Size anAggregateNodes = 0;
        Standard_Size anEdgeCount = 0;
#ifdef DEBUG
        const Standard_Size anAggregateLimit =
            myDebugMaximumCaptureTopologyNodes;
        const Standard_Size aPerSourceLimit = std::min(
            kMaxSourceTopologyNodes,
            myDebugMaximumCaptureTopologyNodes);
#else
        const Standard_Size anAggregateLimit = kMaxCaptureTopologyNodes;
        const Standard_Size aPerSourceLimit = kMaxSourceTopologyNodes;
#endif
        for (const BevelSourceSelection& aSelection : theSelection) {
            if (aSelection.original.IsNull()
                || aSelection.original->Shape().IsNull()
                || aSelection.documentLabel.IsNull()
                || aSelection.edges.empty()
                || aSelection.edges.size() > kMaxSelectedEdges
                || !IsBRepModelingLabel(myDoc, aSelection.documentLabel)
                || !myDoc->IsPresentationEditable(aSelection.original)
                || !myDoc->ShapeLabel(aSelection.original).IsEqual(
                    aSelection.documentLabel)
                || !myContext->IsDisplayed(aSelection.original)) {
                return Standard_False;
            }
            const TopoDS_Shape aShape = aSelection.original->Shape();
            const TopoDS_Shape aStoredShape =
                XCAFDoc_ShapeTool::GetShape(aSelection.documentLabel);
            const gp_Trsf aCapturedTransform =
                aSelection.original->LocalTransformation();
            Standard_Size aSourceSolidCount = 0;
            if (aStoredShape.IsNull() || !aStoredShape.IsEqual(aShape)
                || !TransformsMatch(
                    myDoc->ObjectTransformForLabel(
                        aSelection.documentLabel),
                    aCapturedTransform)
                || HasStyledXCAFSubshape(
                    aDocument,
                    aSelection.documentLabel,
                    kMaxStyledSubshapeLabels)
                || aShape.ShapeType() != TopAbs_SOLID
                || !AccumulateBoundedTopology(
                    aShape,
                    aPerSourceLimit,
                    anAggregateLimit,
                    anAggregateNodes)
                || !HasOnlySolidBranches(
                    aShape,
                    aPerSourceLimit,
                    1,
                    aSourceSolidCount)
                || aSourceSolidCount != 1
                || !BRepCheck_Analyzer(
                    aShape, Standard_False).IsValid()) {
                return Standard_False;
            }
            TopTools_IndexedMapOfShape anEdges;
            TopExp::MapShapes(aShape, TopAbs_EDGE, anEdges);
            TopTools_IndexedMapOfShape aUniqueSelectedEdges;
            for (const TopoDS_Edge& anEdge : aSelection.edges) {
                if (anEdge.IsNull() || anEdges.FindIndex(anEdge) == 0
                    || aUniqueSelectedEdges.Contains(anEdge)) {
                    return Standard_False;
                }
                aUniqueSelectedEdges.Add(anEdge);
            }
            anEdgeCount += aSelection.edges.size();
            if (anEdgeCount > kMaxSelectedEdges) {
                return Standard_False;
            }
            const std::string anIdentifier =
                myDoc->EntityIdentifierForLabel(aSelection.documentLabel);
            if (anIdentifier.empty()
                || !anIdentifiers.insert(anIdentifier).second) {
                return Standard_False;
            }
            Source aSource;
            aSource.original = aSelection.original;
            aSource.label = aSelection.documentLabel;
            aSource.shape = aShape;
            aSource.edges = aSelection.edges;
            aSource.edgeTopologyIndices.reserve(aSelection.edges.size());
            for (const TopoDS_Edge& anEdge : aSelection.edges) {
                const Standard_Integer anIndex = anEdges.FindIndex(anEdge);
                if (anIndex <= 0) {
                    return Standard_False;
                }
                aSource.edgeTopologyIndices.push_back(
                    static_cast<Standard_Size>(anIndex - 1));
            }
            if (!aSelection.edgeTopologyIndices.empty()
                && aSelection.edgeTopologyIndices
                    != aSource.edgeTopologyIndices) {
                return Standard_False;
            }
            aSource.transform = aCapturedTransform;
            aSource.selectionMode = aSelection.selectionMode;
            aSource.entityIdentifier = anIdentifier;
            aSources.push_back(std::move(aSource));
        }
        mySources = std::move(aSources);
        myValue = 0.0;
        myCanApply = Standard_False;
        mySelectionFrozen = Standard_False;
        myStateValid = Standard_True;
        myState = BevelPreviewState::Selecting;
        myRequestedFingerprint.clear();
        notifyPreviewStateChanged();
        return Standard_True;
    } catch (...) {
        clearState();
        return Standard_False;
    }
}

Standard_Boolean BevelOperationController::sourcesAreCurrent() const noexcept
{
    if (myDoc.IsNull() || mySources.empty()) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        for (const Source& aSource : mySources) {
            const TopoDS_Shape aStoredShape =
                XCAFDoc_ShapeTool::GetShape(aSource.label);
            if (aSource.original.IsNull() || aSource.shape.IsNull()
                || aStoredShape.IsNull()
                || !IsBRepModelingLabel(myDoc, aSource.label)
                || !myDoc->IsPresentationEditable(aSource.original)
                || !myDoc->ShapeLabel(aSource.original).IsEqual(aSource.label)
                || !aStoredShape.IsEqual(aSource.shape)
                || !aSource.original->Shape().IsEqual(aSource.shape)
                || !TransformsMatch(
                    aSource.original->LocalTransformation(),
                    aSource.transform)
                || !TransformsMatch(
                    myDoc->ObjectTransformForLabel(aSource.label),
                    aSource.transform)
                || HasStyledXCAFSubshape(
                    myDoc->Document(),
                    aSource.label,
                    kMaxStyledSubshapeLabels)
                || myDoc->EntityIdentifierForLabel(aSource.label)
                    != aSource.entityIdentifier) {
                return Standard_False;
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

std::string BevelOperationController::selectionFingerprint(
    Standard_Boolean& theComplete) const
{
    theComplete = Standard_False;
    try {
        if (!sourcesAreCurrent()) {
            return {};
        }
        std::ostringstream aFingerprint;
        aFingerprint << std::setprecision(17) << myValue;
        for (const Source& aSource : mySources) {
            aFingerprint << '|' << aSource.entityIdentifier << ':'
                << reinterpret_cast<std::uintptr_t>(
                    aSource.shape.TShape().get());
            const gp_Trsf aTransform =
                aSource.original->LocalTransformation();
            for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
                for (Standard_Integer aColumn = 1;
                     aColumn <= 4; ++aColumn) {
                    aFingerprint << ':'
                        << aTransform.Value(aRow, aColumn);
                }
            }
            for (const TopoDS_Edge& anEdge : aSource.edges) {
                aFingerprint << ':'
                    << reinterpret_cast<std::uintptr_t>(
                        anEdge.TShape().get());
            }
        }
        theComplete = Standard_True;
        return aFingerprint.str();
    } catch (...) {
        return {};
    }
}

void BevelOperationController::cancelWorkerRequests() noexcept
{
    if (myGeneration != std::numeric_limits<std::uint64_t>::max()) {
        ++myGeneration;
    }
    if (myWorker != nullptr) {
        myWorker->cancelAll();
    }
    myCanApply = Standard_False;
    myRequestedFingerprint.clear();
}

Standard_Boolean BevelOperationController::enqueuePreviewRequest() noexcept
{
    try {
        Standard_Boolean hasFingerprint = Standard_False;
        const std::string aFingerprint =
            selectionFingerprint(hasFingerprint);
        if (!hasFingerprint || aFingerprint.empty()
            || myGeneration == std::numeric_limits<std::uint64_t>::max()) {
            return Standard_False;
        }
        BevelPreviewWorkerRequest aRequest;
        aRequest.generation = ++myGeneration;
        aRequest.value = myValue;
        aRequest.fingerprint = aFingerprint;
        aRequest.cancellation = std::make_shared<std::atomic_bool>(false);
#ifdef DEBUG
        aRequest.maximumResultTopologyNodes =
            myDebugMaximumResultTopologyNodes;
        aRequest.maximumResultSolids = myDebugMaximumResultSolids;
#endif
        aRequest.sources.reserve(mySources.size());
        for (const Source& aSource : mySources) {
            BRepBuilderAPI_Copy aCopy(
                aSource.shape,
                Standard_True,
                Standard_False);
            const TopoDS_Shape aCopiedShape = aCopy.Shape();
            if (aCopiedShape.IsNull()) {
                return Standard_False;
            }
            BevelPreviewWorkerSource aWorkerSource;
            aWorkerSource.shape = aCopiedShape;
            aWorkerSource.edges.reserve(aSource.edges.size());
            for (const TopoDS_Edge& anEdge : aSource.edges) {
                const TopoDS_Shape aCopiedEdge = aCopy.ModifiedShape(anEdge);
                if (aCopiedEdge.IsNull()
                    || aCopiedEdge.ShapeType() != TopAbs_EDGE) {
                    return Standard_False;
                }
                aWorkerSource.edges.push_back(TopoDS::Edge(aCopiedEdge));
            }
            aRequest.sources.push_back(std::move(aWorkerSource));
        }
        if (myWorker == nullptr) {
            myWorker = std::make_shared<BevelPreviewWorker>(
                weak_from_this());
#ifdef DEBUG
            myWorker->debugSetBlocked(myDebugWorkerBlocked);
#endif
        }
        myRequestedFingerprint = aFingerprint;
        if (!myWorker->enqueue(std::move(aRequest))) {
            myRequestedFingerprint.clear();
            return Standard_False;
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean BevelOperationController::setValue(
    const Standard_Real theSignedDistance) noexcept
{
    if (!hasActiveOperation() || !std::isfinite(theSignedDistance)) {
        return Standard_False;
    }
    const Standard_Real aValue = std::max(
        -kMaxDistance,
        std::min(theSignedDistance, kMaxDistance));
    if (std::abs(aValue - myValue) < 1.0e-9
        && (myState == BevelPreviewState::Computing
            || myState == BevelPreviewState::Ready)) {
        return Standard_True;
    }

    cancelWorkerRequests();
    const Standard_Boolean needsPreviewDiscard =
        !myPreviewResults.empty() || !myStateValid;
    if (needsPreviewDiscard && !discardPreview(Standard_True)) {
        myState = BevelPreviewState::Failed;
        myStateValid = Standard_False;
        notifyPreviewStateChanged();
        return Standard_False;
    }
    // A prior discard failure may have invalidated the transient presentation
    // state. Once a complete retry has removed that preview and restored every
    // source, this operation is coherent again and may produce an applicable
    // replacement.
    myStateValid = Standard_True;
    myValue = aValue;
    if (std::abs(myValue) <= std::numeric_limits<Standard_Real>::epsilon()) {
        myState = BevelPreviewState::Selecting;
        mySelectionFrozen = Standard_False;
        notifyPreviewStateChanged();
        return Standard_True;
    }
    myState = BevelPreviewState::Computing;
    mySelectionFrozen = Standard_True;
    if (!enqueuePreviewRequest()) {
        myState = BevelPreviewState::Failed;
        notifyPreviewStateChanged();
        return Standard_False;
    }
    notifyPreviewStateChanged();
    return Standard_True;
}

void BevelOperationController::acceptWorkerResult(
    BevelPreviewWorkerResult theResult) noexcept
{
    if (theResult.generation != myGeneration
        || theResult.fingerprint != myRequestedFingerprint
        || std::abs(theResult.value - myValue) > 1.0e-9
        || !hasActiveOperation()) {
#ifdef DEBUG
        ++myDebugStaleSuppressionCount;
#endif
        return;
    }
    if (theResult.outcome != BevelPreviewWorkerOutcome::Success
        || theResult.results.size() != mySources.size()
        || !sourcesAreCurrent()
        || !installPreview(theResult.results)) {
        myCanApply = Standard_False;
        myState = BevelPreviewState::Failed;
        notifyPreviewStateChanged();
        return;
    }
    myCanApply = Standard_True;
    myState = BevelPreviewState::Ready;
    mySelectionFrozen = Standard_True;
#ifdef DEBUG
    ++myDebugAcceptedCount;
#endif
    notifyPreviewStateChanged();
}

Standard_Boolean BevelOperationController::installPreview(
    const std::vector<TopoDS_Shape>& theResults) noexcept
{
    if (theResults.size() != mySources.size() || myContext.IsNull()) {
        return Standard_False;
    }
    std::vector<Handle(AIS_Shape)> aPresentations;
    try {
        OCC_CATCH_SIGNALS
        aPresentations.reserve(theResults.size());
        for (Standard_Size anIndex = 0;
             anIndex < theResults.size(); ++anIndex) {
            if (theResults[anIndex].IsNull()) {
                return Standard_False;
            }
            Handle(AIS_Shape) aPresentation =
                new AIS_Shape(theResults[anIndex]);
            aPresentation->SetLocalTransformation(
                mySources[anIndex].transform);
            myDoc->LoadObjectMeterial(
                mySources[anIndex].label,
                aPresentation);
            aPresentations.push_back(std::move(aPresentation));
        }
        if (!myPreviewResults.empty()
            && !discardPreview(Standard_True)) {
            return Standard_False;
        }
        myPreviewResults = aPresentations;
        for (Standard_Size anIndex = 0;
             anIndex < myPreviewResults.size(); ++anIndex) {
            myContext->Display(
                myPreviewResults[anIndex],
                AIS_Shaded,
                mySources[anIndex].selectionMode,
                Standard_False);
            myContext->Deactivate(myPreviewResults[anIndex]);
        }
        for (const Source& aSource : mySources) {
            myContext->Remove(aSource.original, Standard_False);
        }
        myContext->ClearSelected(Standard_False);
        myContext->UpdateCurrentViewer();
        return Standard_True;
    } catch (...) {
        for (const Handle(AIS_Shape)& aPresentation : myPreviewResults) {
            try {
                myContext->Remove(aPresentation, Standard_False);
            } catch (...) {
            }
        }
        myPreviewResults.clear();
        (void)discardPreview(Standard_True);
        return Standard_False;
    }
}

Standard_Boolean BevelOperationController::discardPreview(
    const Standard_Boolean theRestoreSources) noexcept
{
#ifdef DEBUG
    if (myDebugCancelDiscardFailureCount > 0) {
        --myDebugCancelDiscardFailureCount;
        return Standard_False;
    }
#endif
    bool didResolveAll = true;
    if (!myContext.IsNull()) {
        for (const Handle(AIS_Shape)& aPreview : myPreviewResults) {
            if (aPreview.IsNull()) {
                continue;
            }
            try {
                myContext->Remove(aPreview, Standard_False);
            } catch (...) {
                didResolveAll = false;
            }
        }
        if (theRestoreSources) {
            for (const Source& aSource : mySources) {
                if (aSource.original.IsNull()) {
                    didResolveAll = false;
                    continue;
                }
                try {
                    myContext->Display(
                        aSource.original,
                        AIS_Shaded,
                        aSource.selectionMode,
                        Standard_False);
                    myContext->Activate(
                        aSource.original,
                        aSource.selectionMode,
                        Standard_False);
                } catch (...) {
                    didResolveAll = false;
                }
            }
        }
        try {
            myContext->UpdateCurrentViewer();
        } catch (...) {
            didResolveAll = false;
        }
    }
    if (didResolveAll) {
        myPreviewResults.clear();
    }
    return didResolveAll;
}

Standard_Boolean BevelOperationController::canApply() const noexcept
{
    return myCanApply && myStateValid
        && myState == BevelPreviewState::Ready
        && !myPreviewResults.empty()
        && myPreviewResults.size() == mySources.size();
}

Standard_Boolean BevelOperationController::hasActiveOperation() const noexcept
{
    return !mySources.empty();
}

Standard_Boolean BevelOperationController::isSelectionFrozen() const noexcept
{
    return mySelectionFrozen;
}

BevelPreviewState BevelOperationController::previewState() const noexcept
{
    return myState;
}

std::uint64_t BevelOperationController::previewGeneration() const noexcept
{
    return myGeneration;
}

Standard_Boolean BevelOperationController::capturePreview(
    BevelPreviewCapture& theCapture) const noexcept
{
    theCapture = {};
    if (!canApply() || !sourcesAreCurrent()) {
        return Standard_False;
    }
    try {
        BevelPreviewCapture aCapture;
        aCapture.results = myPreviewResults;
        aCapture.suppressedSourceLabels.reserve(mySources.size());
        for (Standard_Size anIndex = 0;
             anIndex < mySources.size(); ++anIndex) {
            if (mySources[anIndex].label.IsNull()
                || myPreviewResults[anIndex].IsNull()
                || !myContext->IsDisplayed(myPreviewResults[anIndex])) {
                return Standard_False;
            }
            aCapture.suppressedSourceLabels.push_back(
                mySources[anIndex].label);
        }
        theCapture = std::move(aCapture);
        return Standard_True;
    } catch (...) {
        theCapture = {};
        return Standard_False;
    }
}

Standard_Boolean BevelOperationController::abortOpenCommand(
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
        try {
            return !theDocument->HasOpenCommand();
        } catch (...) {
            return Standard_False;
        }
    }
}

BevelApplyResult BevelOperationController::apply() noexcept
{
    if (!canApply()) {
        return BevelApplyResult::NoChange;
    }
    if (!sourcesAreCurrent()) {
        myCanApply = Standard_False;
        myState = BevelPreviewState::Failed;
        notifyPreviewStateChanged();
        return BevelApplyResult::NoChange;
    }
    Standard_Boolean hasFingerprint = Standard_False;
    if (selectionFingerprint(hasFingerprint) != myRequestedFingerprint
        || !hasFingerprint) {
        myCanApply = Standard_False;
        myState = BevelPreviewState::Failed;
        notifyPreviewStateChanged();
        return BevelApplyResult::NoChange;
    }
    Handle(TDocStd_Document) aDocument;
    try {
        aDocument = myDoc->ChangeDocument();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()) {
            return BevelApplyResult::NoChange;
        }
    } catch (...) {
        return BevelApplyResult::NoChange;
    }

    myCanApply = Standard_False;
    myState = BevelPreviewState::Committing;
    notifyPreviewStateChanged();
    try {
        OCC_CATCH_SIGNALS
        aDocument->NewCommand();
        if (!aDocument->HasOpenCommand()) {
            myState = BevelPreviewState::Failed;
            notifyPreviewStateChanged();
            return BevelApplyResult::NoChange;
        }
#ifdef DEBUG
        if (myDebugTransactionFailureCount > 0) {
            --myDebugTransactionFailureCount;
            (void)abortOpenCommand(aDocument);
            myState = BevelPreviewState::Failed;
            notifyPreviewStateChanged();
            return BevelApplyResult::NoChange;
        }
#endif
        for (Standard_Size anIndex = 0;
             anIndex < mySources.size(); ++anIndex) {
            if (!myDoc->ReplaceShape(
                    mySources[anIndex].label,
                    myPreviewResults[anIndex])) {
                (void)abortOpenCommand(aDocument);
                myState = BevelPreviewState::Failed;
                notifyPreviewStateChanged();
                return BevelApplyResult::NoChange;
            }
        }
        Standard_Boolean commitReported = Standard_False;
        Standard_Boolean commitThrew = Standard_False;
        try {
            commitReported = aDocument->CommitCommand();
        } catch (...) {
            commitThrew = Standard_True;
        }
        const auto committedCandidatesAreAuthoritative = [&]() noexcept {
            try {
                if (aDocument->HasOpenCommand()) {
                    return Standard_False;
                }
                for (Standard_Size anIndex = 0;
                     anIndex < mySources.size(); ++anIndex) {
                    const TopoDS_Shape aStored =
                        XCAFDoc_ShapeTool::GetShape(
                            mySources[anIndex].label);
                    if (aStored.IsNull()
                        || myPreviewResults[anIndex].IsNull()
                        || !aStored.IsSame(
                            myPreviewResults[anIndex]->Shape())) {
                        return Standard_False;
                    }
                }
                return Standard_True;
            } catch (...) {
                return Standard_False;
            }
        };
        const Standard_Boolean committedAuthoritatively =
            committedCandidatesAreAuthoritative();
        (void)commitReported;
        (void)commitThrew;
        if (!committedAuthoritatively) {
            (void)abortOpenCommand(aDocument);
            myState = BevelPreviewState::Failed;
            notifyPreviewStateChanged();
            return BevelApplyResult::NoChange;
        }
    } catch (...) {
        (void)abortOpenCommand(aDocument);
        myState = BevelPreviewState::Failed;
        notifyPreviewStateChanged();
        return BevelApplyResult::NoChange;
    }

    bool needsRedraw = false;
    try {
        for (Standard_Size anIndex = 0;
             anIndex < myPreviewResults.size(); ++anIndex) {
            myContext->Display(
                myPreviewResults[anIndex],
                AIS_Shaded,
                mySources[anIndex].selectionMode,
                Standard_False);
            myContext->Activate(
                myPreviewResults[anIndex],
                mySources[anIndex].selectionMode,
                Standard_False);
            myContext->Remove(
                mySources[anIndex].original,
                Standard_False);
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
    // The preview presentations are now the committed document presentations;
    // release controller ownership without removing them.
    clearState();
    notifyPreviewStateChanged();
    return needsRedraw
        ? BevelApplyResult::AppliedNeedsDocumentRedraw
        : BevelApplyResult::Applied;
}

Standard_Boolean BevelOperationController::cancel() noexcept
{
    if (!hasActiveOperation() && myPreviewResults.empty()) {
        clearState();
        return Standard_True;
    }
    cancelWorkerRequests();
    const Standard_Boolean didDiscard = discardPreview(Standard_True);
    if (!didDiscard) {
        myStateValid = Standard_False;
        myState = BevelPreviewState::Failed;
        notifyPreviewStateChanged();
        return Standard_False;
    }
    clearState();
    notifyPreviewStateChanged();
    return Standard_True;
}

void BevelOperationController::clearState() noexcept
{
    mySources.clear();
    myPreviewResults.clear();
    myRequestedFingerprint.clear();
    myValue = 0.0;
    myCanApply = Standard_False;
    mySelectionFrozen = Standard_False;
    myStateValid = Standard_True;
    myState = BevelPreviewState::Selecting;
}

#ifdef DEBUG
BevelPreviewDebugState
BevelOperationController::debugPreviewState() const noexcept
{
    BevelPreviewDebugState aState;
    aState.state = myState;
    aState.generation = myGeneration;
    aState.activeOperation = hasActiveOperation();
    aState.canApply = canApply();
    aState.selectionFrozen = mySelectionFrozen;
    aState.value = myValue;
    aState.sourceCount = mySources.size();
    for (const Source& aSource : mySources) {
        aState.edgeCount += aSource.edges.size();
    }
    try {
        OCC_CATCH_SIGNALS
        if (!mySources.empty()) {
            TopTools_IndexedMapOfShape aSourceEdges;
            TopExp::MapShapes(
                mySources.front().shape,
                TopAbs_EDGE,
                aSourceEdges);
            aState.sourceEdgeCount =
                static_cast<Standard_Size>(aSourceEdges.Extent());
            if (!mySources.front().edgeTopologyIndices.empty()) {
                aState.capturedEdgeTopologyIndex =
                    static_cast<Standard_Integer>(
                        mySources.front().edgeTopologyIndices.front());
            }
            if (!mySources.front().edges.empty()) {
                GProp_GProps anEdgeProperties;
                BRepGProp::LinearProperties(
                    mySources.front().edges.front(),
                    anEdgeProperties);
                aState.capturedEdgeLength = anEdgeProperties.Mass();
            }
        }
        for (const Handle(AIS_Shape)& aPreview : myPreviewResults) {
            if (aPreview.IsNull() || aPreview->Shape().IsNull()) {
                continue;
            }
            if (!AccumulateBoundedTopology(
                    aPreview->Shape(),
                    kMaxResultTopologyNodes,
                    kMaxResultTopologyNodes,
                    aState.candidateTopologyNodeCount)) {
                aState.candidateTopologyNodeCount = 0;
            }
            for (TopExp_Explorer aSolid(
                     aPreview->Shape(), TopAbs_SOLID);
                 aSolid.More(); aSolid.Next()) {
                ++aState.candidateSolidCount;
            }
            if (aPreview->Shape().ShapeType() == TopAbs_SOLID) {
                aState.candidateSolidCount = std::max<Standard_Size>(
                    aState.candidateSolidCount,
                    1);
            }
            GProp_GProps aVolumeProperties;
            BRepGProp::VolumeProperties(
                aPreview->Shape(),
                aVolumeProperties);
            aState.candidateVolume += aVolumeProperties.Mass();
            Bnd_Box aBounds;
            // Debug telemetry describes the exact BRep candidate, not its
            // presentation triangulation enlarged by deflection/tolerances.
            BRepBndLib::AddOptimal(
                aPreview->Shape(),
                aBounds,
                Standard_False,
                Standard_False);
            if (!aBounds.IsVoid()) {
                aBounds.Get(
                    aState.candidateBounds[0],
                    aState.candidateBounds[1],
                    aState.candidateBounds[2],
                    aState.candidateBounds[3],
                    aState.candidateBounds[4],
                    aState.candidateBounds[5]);
            }
        }
    } catch (...) {
    }
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
    aState.acceptedCount = myDebugAcceptedCount;
    aState.staleSuppressionCount = myDebugStaleSuppressionCount;
    if (myWorker != nullptr) {
        const BevelPreviewWorker::DebugSnapshot aWorker =
            myWorker->debugSnapshot();
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

void BevelOperationController::debugSetWorkerBlocked(
    const Standard_Boolean theBlocked) noexcept
{
    myDebugWorkerBlocked = theBlocked;
    if (myWorker != nullptr) {
        myWorker->debugSetBlocked(theBlocked);
    }
}

void BevelOperationController::debugSetMaximumCaptureTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    myDebugMaximumCaptureTopologyNodes = std::max<Standard_Size>(
        1, std::min(theLimit, kMaxCaptureTopologyNodes));
}

void BevelOperationController::debugSetMaximumResultTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    myDebugMaximumResultTopologyNodes = std::max<Standard_Size>(
        1, std::min(theLimit, kMaxResultTopologyNodes));
}

void BevelOperationController::debugSetMaximumResultSolids(
    const Standard_Size theLimit) noexcept
{
    myDebugMaximumResultSolids = std::max<Standard_Size>(
        1, std::min(theLimit, kMaxResultSolids));
}

void BevelOperationController::debugSetTransactionFailureCount(
    const Standard_Size theCount) noexcept
{
    myDebugTransactionFailureCount = theCount;
}

void BevelOperationController::debugSetCancelDiscardFailureCount(
    const Standard_Size theCount) noexcept
{
    myDebugCancelDiscardFailureCount = theCount;
}

Standard_Boolean
BevelOperationController::debugMutateFirstSourcePersistedTransform() noexcept
{
    if (!sourcesAreCurrent() || mySources.empty() || myDoc.IsNull()) {
        return Standard_False;
    }
    Handle(TDocStd_Document) aDocument;
    try {
        OCC_CATCH_SIGNALS
        aDocument = myDoc->ChangeDocument();
        const Source& aSource = mySources.front();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()
            || aSource.label.IsNull()
            || aSource.label.Data() != aDocument->GetData()) {
            return Standard_False;
        }
        const Standard_Real anOriginalX =
            myDoc->ObjectTransformForLabel(aSource.label)
                .TranslationPart().X();
        if (!std::isfinite(anOriginalX)) {
            return Standard_False;
        }
        aDocument->NewCommand();
        if (!aDocument->HasOpenCommand()) {
            return Standard_False;
        }
        TDataStd_Real::Set(
            aSource.label.FindChild(1),
            anOriginalX + 1.0);
        try {
            (void)aDocument->CommitCommand();
        } catch (...) {
        }
        if (aDocument->HasOpenCommand()) {
            (void)abortOpenCommand(aDocument);
            return Standard_False;
        }
        return !TransformsMatch(
                myDoc->ObjectTransformForLabel(aSource.label),
                aSource.transform)
            && TransformsMatch(
                aSource.original->LocalTransformation(),
                aSource.transform);
    } catch (...) {
        (void)abortOpenCommand(aDocument);
        return Standard_False;
    }
}
#endif

} // namespace core3d
