//
//  ShellOperationController.cpp
//  Core3D
//

#include "ShellOperationController.hpp"

#include "../Common/Core3DMobileResourceLimits.h"

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepOffset_Error.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Message_ProgressIndicator.hxx>
#include <Precision.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <TColStd_ListOfInteger.hxx>
#include <TDataStd_Real.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>

#include <dispatch/dispatch.h>
#include <pthread.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace core3d {

enum class ShellPreviewWorkerOutcome : std::uint8_t {
    Success = 0,
    Cancelled,
    Failed,
};

struct ShellPreviewWorkerRequest {
    std::uint64_t generation = 0;
    Standard_Real thickness = 0.0;
    Standard_Real tolerance = 0.0;
    Standard_Real sourceVolume = 0.0;
    Standard_Real sourceBounds[6] = {};
    std::string fingerprint;
    TopoDS_Shape source;
    TopoDS_Face openingFace;
    std::shared_ptr<std::atomic_bool> cancellation;
    Standard_Size maximumResultTopologyNodes =
        ShellOperationController::kMaximumResultTopologyNodes;
};

struct ShellPreviewWorkerResult {
    std::uint64_t generation = 0;
    Standard_Real thickness = 0.0;
    std::string fingerprint;
    ShellPreviewWorkerOutcome outcome =
        ShellPreviewWorkerOutcome::Failed;
    TopoDS_Shape result;
    Standard_Boolean computeWasMainThread = Standard_False;
};

namespace {

constexpr Standard_Real kLegacyMetersPerUnit = 0.001;
constexpr Standard_Real kPhysicalKernelToleranceMeters = 1.0e-6;

dispatch_queue_t ShellWorkerQueue()
{
    static dispatch_queue_t aQueue = []() {
        dispatch_queue_attr_t anAttribute =
            dispatch_queue_attr_make_with_qos_class(
                DISPATCH_QUEUE_SERIAL,
                QOS_CLASS_USER_INITIATED,
                0);
        return dispatch_queue_create(
            "com.shapeyard.core3d.shell-preview",
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
            for (Standard_Integer aColumn = 1; aColumn <= 4;
                 ++aColumn) {
                const Standard_Real aLeft =
                    theLeft.Value(aRow, aColumn);
                const Standard_Real aRight =
                    theRight.Value(aRow, aColumn);
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

Standard_Boolean TryReadMetersPerUnit(
    const Handle(TDocStd_Document)& theDocument,
    Standard_Real& theMetersPerUnit) noexcept
{
    theMetersPerUnit = kLegacyMetersPerUnit;
    if (theDocument.IsNull()) {
        return Standard_False;
    }
    try {
        const Standard_Boolean hasLengthUnit =
            XCAFDoc_DocumentTool::GetLengthUnit(
                theDocument, theMetersPerUnit);
        return (hasLengthUnit && std::isfinite(theMetersPerUnit)
                && theMetersPerUnit > 0.0)
            || (!hasLengthUnit
                && theMetersPerUnit == kLegacyMetersPerUnit);
    } catch (...) {
        return Standard_False;
    }
}

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
                || !XCAFDoc_VisMaterialTool::GetShapeMaterial(
                        aLabel).IsNull()
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

//! Counts occurrences rather than unique TShapes. This bounds work for a
//! shared or adversarial topology DAG before any expensive kernel operation.
Standard_Boolean CountBoundedTopology(
    const TopoDS_Shape& theShape,
    const Standard_Size theMaximum,
    Standard_Size& theCount) noexcept
{
    theCount = 0;
    if (theShape.IsNull() || theMaximum == 0) {
        return Standard_False;
    }
    try {
        std::vector<TopoDS_Shape> aPending{theShape};
        while (!aPending.empty()) {
            const TopoDS_Shape aCurrent = aPending.back();
            aPending.pop_back();
            if (aCurrent.IsNull() || ++theCount > theMaximum) {
                theCount = 0;
                return Standard_False;
            }
            std::vector<TopoDS_Shape> aChildren;
            for (TopoDS_Iterator aChild(
                     aCurrent, Standard_True, Standard_True);
                 aChild.More(); aChild.Next()) {
                const Standard_Size aRemaining = theMaximum - theCount;
                if (aPending.size() + aChildren.size() + 1U
                    > static_cast<std::size_t>(aRemaining)) {
                    theCount = 0;
                    return Standard_False;
                }
                aChildren.push_back(aChild.Value());
            }
            for (auto aChild = aChildren.rbegin();
                 aChild != aChildren.rend(); ++aChild) {
                aPending.push_back(*aChild);
            }
        }
        return theCount > 0;
    } catch (...) {
        theCount = 0;
        return Standard_False;
    }
}

Standard_Boolean IsSinglePlanarOpening(
    const TopoDS_Face& theFace) noexcept
{
    if (theFace.IsNull()) {
        return Standard_False;
    }
    try {
        BRepAdaptor_Surface aSurface(theFace, Standard_True);
        if (aSurface.GetType() != GeomAbs_Plane) {
            return Standard_False;
        }
        TopTools_IndexedMapOfShape aWires;
        TopExp::MapShapes(theFace, TopAbs_WIRE, aWires);
        return aWires.Extent() == 1;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean ShapeBounds(
    const TopoDS_Shape& theShape,
    Standard_Real (&theBounds)[6]) noexcept
{
    if (theShape.IsNull()) {
        return Standard_False;
    }
    try {
        Bnd_Box aBox;
        BRepBndLib::AddOptimal(
            theShape, aBox, Standard_False, Standard_False);
        if (aBox.IsVoid() || aBox.IsOpen()) {
            return Standard_False;
        }
        aBox.Get(
            theBounds[0], theBounds[1], theBounds[2],
            theBounds[3], theBounds[4], theBounds[5]);
        for (const Standard_Real aValue : theBounds) {
            if (!std::isfinite(aValue)
                || std::abs(aValue)
                    > limits::kMaximumModelCoordinateMagnitude) {
                return Standard_False;
            }
        }
        return theBounds[3] > theBounds[0]
            && theBounds[4] > theBounds[1]
            && theBounds[5] > theBounds[2];
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean ShapeVolume(
    const TopoDS_Shape& theShape,
    Standard_Real& theVolume) noexcept
{
    theVolume = 0.0;
    if (theShape.IsNull()) {
        return Standard_False;
    }
    try {
        GProp_GProps aProperties;
        BRepGProp::VolumeProperties(theShape, aProperties);
        theVolume = aProperties.Mass();
        return std::isfinite(theVolume) && theVolume > 0.0;
    } catch (...) {
        theVolume = 0.0;
        return Standard_False;
    }
}

Standard_Boolean MaximumVertexTolerance(
    const TopoDS_Shape& theShape,
    Standard_Real& theTolerance) noexcept
{
    theTolerance = Precision::Confusion();
    if (theShape.IsNull()) {
        return Standard_False;
    }
    try {
        Standard_Size aVertexCount = 0;
        for (TopExp_Explorer aVertex(theShape, TopAbs_VERTEX);
             aVertex.More(); aVertex.Next()) {
            if (++aVertexCount
                    > ShellOperationController::
                        kMaximumSourceTopologyNodes) {
                return Standard_False;
            }
            const Standard_Real aTolerance = BRep_Tool::Tolerance(
                TopoDS::Vertex(aVertex.Current()));
            if (!std::isfinite(aTolerance) || aTolerance < 0.0) {
                return Standard_False;
            }
            theTolerance = std::max(theTolerance, aTolerance);
        }
        return aVertexCount > 0 && std::isfinite(theTolerance);
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean ComputeThicknessRange(
    const Standard_Real (&theBounds)[6],
    const Standard_Real theMetersPerUnit,
    const Standard_Real theMaximumVertexTolerance,
    Standard_Real& theMinimum,
    Standard_Real& theMaximum,
    Standard_Real& theKernelTolerance) noexcept
{
    if (!std::isfinite(theMetersPerUnit) || theMetersPerUnit <= 0.0
        || !std::isfinite(theMaximumVertexTolerance)
        || theMaximumVertexTolerance < 0.0) {
        return Standard_False;
    }
    const std::array<Standard_Real, 3> aSpans = {
        theBounds[3] - theBounds[0],
        theBounds[4] - theBounds[1],
        theBounds[5] - theBounds[2],
    };
    const Standard_Real aMinimumSpan = *std::min_element(
        aSpans.begin(), aSpans.end());
    theKernelTolerance = std::max({
        Precision::Confusion(),
        kPhysicalKernelToleranceMeters / theMetersPerUnit,
        theMaximumVertexTolerance,
    });
    theMinimum = 10.0 * theKernelTolerance;
    theMaximum = std::min(
        ShellOperationController::kMaximumPhysicalThicknessMeters
            / theMetersPerUnit,
        ShellOperationController::kMaximumThicknessFraction
            * aMinimumSpan);
    return std::isfinite(aMinimumSpan) && aMinimumSpan > 0.0
        && std::isfinite(theMinimum) && theMinimum > 0.0
        && std::isfinite(theMaximum) && theMaximum > theMinimum;
}

Standard_Boolean IsValidInwardResult(
    const TopoDS_Shape& theCandidate,
    const TopoDS_Shape& theSource,
    const Standard_Real theSourceVolume,
    const Standard_Real (&theSourceBounds)[6],
    const Standard_Real theTolerance,
    const Standard_Size theMaximumTopologyNodes,
    const std::shared_ptr<std::atomic_bool>& theCancellation) noexcept
{
    const auto wasCancelled = [&]() {
        return theCancellation == nullptr
            || theCancellation->load(std::memory_order_relaxed);
    };
    if (wasCancelled() || theCandidate.IsNull()
        || theSource.IsNull()
        || theCandidate.ShapeType() != TopAbs_SOLID
        || theCandidate.IsSame(theSource)
        || !std::isfinite(theSourceVolume) || theSourceVolume <= 0.0) {
        return Standard_False;
    }
    Standard_Size aTopologyCount = 0;
    if (!CountBoundedTopology(
            theCandidate,
            theMaximumTopologyNodes,
            aTopologyCount)
        || wasCancelled()) {
        return Standard_False;
    }
    try {
        BRepCheck_Analyzer anAnalyzer(theCandidate, Standard_True);
        if (!anAnalyzer.IsValid() || wasCancelled()) {
            return Standard_False;
        }
    } catch (...) {
        return Standard_False;
    }
    Standard_Real aCandidateVolume = 0.0;
    if (!ShapeVolume(theCandidate, aCandidateVolume)
        || wasCancelled()
        || !(aCandidateVolume < theSourceVolume)) {
        return Standard_False;
    }
    Standard_Real aCandidateBounds[6] = {};
    if (!ShapeBounds(theCandidate, aCandidateBounds)
        || wasCancelled()) {
        return Standard_False;
    }
    const Standard_Real aScale = std::max({
        std::abs(theSourceBounds[0]),
        std::abs(theSourceBounds[1]),
        std::abs(theSourceBounds[2]),
        std::abs(theSourceBounds[3]),
        std::abs(theSourceBounds[4]),
        std::abs(theSourceBounds[5]),
        1.0,
    });
    const Standard_Real aContainmentTolerance = std::max(
        10.0 * theTolerance,
        64.0 * std::numeric_limits<Standard_Real>::epsilon()
            * aScale);
    for (Standard_Integer anAxis = 0; anAxis < 3; ++anAxis) {
        if (aCandidateBounds[anAxis]
                < theSourceBounds[anAxis] - aContainmentTolerance
            || aCandidateBounds[anAxis + 3]
                > theSourceBounds[anAxis + 3]
                    + aContainmentTolerance) {
            return Standard_False;
        }
    }
    return !wasCancelled();
}

class ShellCancellationIndicator final
    : public Message_ProgressIndicator {
    DEFINE_STANDARD_RTTI_INLINE(
        ShellCancellationIndicator,
        Message_ProgressIndicator)
public:
    explicit ShellCancellationIndicator(
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

class ShellPreviewWorker final
    : public std::enable_shared_from_this<ShellPreviewWorker> {
public:
    explicit ShellPreviewWorker(
        std::weak_ptr<ShellOperationController> theOwner)
    : myOwner(std::move(theOwner)) {}

    ~ShellPreviewWorker() noexcept
    {
        shutdown();
    }

    Standard_Boolean enqueue(ShellPreviewWorkerRequest theRequest) noexcept
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
                auto* aContext =
                    new std::shared_ptr<ShellPreviewWorker>(
                        shared_from_this());
                dispatch_async_f(
                    ShellWorkerQueue(),
                    aContext,
                    &ShellPreviewWorker::DrainTrampoline);
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
        std::weak_ptr<ShellOperationController> owner;
        ShellPreviewWorkerResult result;
    };

    static void DrainTrampoline(void* theContext)
    {
        std::unique_ptr<std::shared_ptr<ShellPreviewWorker>> aContext(
            static_cast<std::shared_ptr<ShellPreviewWorker>*>(
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

    static ShellPreviewWorkerResult compute(
        const ShellPreviewWorkerRequest& theRequest) noexcept
    {
        ShellPreviewWorkerResult aResult;
        aResult.generation = theRequest.generation;
        aResult.thickness = theRequest.thickness;
        aResult.fingerprint = theRequest.fingerprint;
        aResult.computeWasMainThread = pthread_main_np() != 0;
        const auto wasCancelled = [&]() {
            return theRequest.cancellation == nullptr
                || theRequest.cancellation->load(
                    std::memory_order_relaxed);
        };
        if (wasCancelled()) {
            aResult.outcome = ShellPreviewWorkerOutcome::Cancelled;
            return aResult;
        }
        try {
            OCC_CATCH_SIGNALS
            Handle(ShellCancellationIndicator) aProgress =
                new ShellCancellationIndicator(
                    theRequest.cancellation);
            TopTools_ListOfShape anOpenings;
            anOpenings.Append(theRequest.openingFace);
            BRepOffsetAPI_MakeThickSolid aBuilder;
            aBuilder.MakeThickSolidByJoin(
                theRequest.source,
                anOpenings,
                -theRequest.thickness,
                theRequest.tolerance,
                BRepOffset_Skin,
                Standard_False,
                Standard_False,
                GeomAbs_Arc,
                Standard_False,
                aProgress->Start());
            if (wasCancelled()
                || (!aBuilder.IsDone()
                    && aBuilder.MakeOffset().Error()
                        == BRepOffset_UserBreak)) {
                aResult.outcome = ShellPreviewWorkerOutcome::Cancelled;
                return aResult;
            }
            if (!aBuilder.IsDone()
                || aBuilder.MakeOffset().Error()
                    != BRepOffset_NoError) {
                return aResult;
            }
            const TopoDS_Shape aCandidate = aBuilder.Shape();
            if (!IsValidInwardResult(
                    aCandidate,
                    theRequest.source,
                    theRequest.sourceVolume,
                    theRequest.sourceBounds,
                    theRequest.tolerance,
                    theRequest.maximumResultTopologyNodes,
                    theRequest.cancellation)) {
                if (wasCancelled()) {
                    aResult.outcome =
                        ShellPreviewWorkerOutcome::Cancelled;
                }
                return aResult;
            }
            aResult.result = aCandidate;
            aResult.outcome = ShellPreviewWorkerOutcome::Success;
            return aResult;
        } catch (...) {
            if (wasCancelled()) {
                aResult.outcome = ShellPreviewWorkerOutcome::Cancelled;
            }
            aResult.result.Nullify();
            return aResult;
        }
    }

    void drain() noexcept
    {
        for (;;) {
            ShellPreviewWorkerRequest aRequest;
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
            ShellPreviewWorkerResult aResult = compute(aRequest);
            {
                std::lock_guard<std::mutex> aLock(myMutex);
                myActiveCancellation.reset();
                ++myCompletedCount;
                myLastComputeWasMainThread =
                    aResult.computeWasMainThread;
                if (aResult.outcome
                    == ShellPreviewWorkerOutcome::Cancelled) {
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
                    &ShellPreviewWorker::DeliverTrampoline);
            } catch (...) {
            }
        }
    }

private:
    std::weak_ptr<ShellOperationController> myOwner;
    mutable std::mutex myMutex;
    std::condition_variable myCondition;
    std::optional<ShellPreviewWorkerRequest> myPending;
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

ShellOperationController::ShellOperationController(
    Handle(AIS_InteractiveContext) theContext,
    Handle(OcctDocument) theDocument)
: myContext(std::move(theContext)),
  myDoc(std::move(theDocument))
{
}

ShellOperationController::~ShellOperationController() noexcept
{
    myPreviewStateChangedCallback = {};
    cancelWorkerRequests();
    if (myWorker != nullptr) {
        myWorker->shutdown();
        myWorker.reset();
    }
    DocumentState aDocumentState = inspectDocumentState();
    if (aDocumentState == DocumentState::OpenCommand
        && abortOwnedCommand()) {
        aDocumentState = inspectDocumentState();
    }
    if (aDocumentState == DocumentState::Candidate) {
        (void)discardPreview(Standard_False);
    } else if (aDocumentState == DocumentState::Original
               || aDocumentState == DocumentState::Unavailable) {
        (void)discardPreview(Standard_True);
    }
}

void ShellOperationController::setPreviewStateChangedCallback(
    std::function<void()> theCallback)
{
    myPreviewStateChangedCallback = std::move(theCallback);
}

void ShellOperationController::notifyPreviewStateChanged() noexcept
{
    try {
        if (myPreviewStateChangedCallback) {
            myPreviewStateChangedCallback();
        }
    } catch (...) {
    }
}

Standard_Boolean ShellOperationController::begin(
    const ShellSourceSelection& theSelection) noexcept
{
    if (myContext.IsNull() || myDoc.IsNull()
        || theSelection.original.IsNull()
        || theSelection.openingFace.IsNull()) {
        return Standard_False;
    }
    if (hasActiveOperation() && !cancel()) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(TDocStd_Document) aDocument =
            myDoc->ChangeDocument();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()
            || aDocument->GetUndoLimit() == 0
            || theSelection.documentLabel.IsNull()
            || theSelection.documentLabel.Data()
                != aDocument->GetData()
            || !IsBRepModelingLabel(
                myDoc, theSelection.documentLabel)
            || !myDoc->IsPresentationEditable(
                theSelection.original)
            || !myDoc->ShapeLabel(theSelection.original).IsEqual(
                theSelection.documentLabel)
            || !myContext->IsDisplayed(theSelection.original)) {
            return Standard_False;
        }

        const TopoDS_Shape aShape = theSelection.original->Shape();
        const TopoDS_Shape aStoredShape =
            XCAFDoc_ShapeTool::GetShape(
                theSelection.documentLabel);
        const gp_Trsf aTransform =
            theSelection.original->LocalTransformation();
        if (aShape.IsNull() || aStoredShape.IsNull()
            || !aStoredShape.IsEqual(aShape)
            || aShape.ShapeType() != TopAbs_SOLID
            || !TransformsMatch(
                myDoc->ObjectTransformForLabel(
                    theSelection.documentLabel),
                aTransform)
            || HasStyledXCAFSubshape(
                aDocument,
                theSelection.documentLabel,
                kMaximumStyledSubshapeLabels)) {
            return Standard_False;
        }

#ifdef DEBUG
        const Standard_Size aCaptureLimit =
            std::max<Standard_Size>(
                1,
                std::min(
                    myDebugMaximumCaptureTopologyNodes,
                    kMaximumSourceTopologyNodes));
#else
        const Standard_Size aCaptureLimit =
            kMaximumSourceTopologyNodes;
#endif
        Standard_Size aTopologyNodeCount = 0;
        if (!CountBoundedTopology(
                aShape, aCaptureLimit, aTopologyNodeCount)
            || !BRepCheck_Analyzer(
                aShape, Standard_True).IsValid()) {
            return Standard_False;
        }

        TopTools_IndexedMapOfShape aFaces;
        TopExp::MapShapes(aShape, TopAbs_FACE, aFaces);
        const Standard_Integer aFaceIndex =
            aFaces.FindIndex(theSelection.openingFace);
        if (aFaceIndex <= 0
            || static_cast<Standard_Size>(aFaceIndex - 1)
                != theSelection.faceTopologyIndex
            || !IsSinglePlanarOpening(
                theSelection.openingFace)) {
            return Standard_False;
        }

        Standard_Real aMetersPerUnit = 0.0;
        Standard_Real aSourceBounds[6] = {};
        Standard_Real aSourceVolume = 0.0;
        Standard_Real aMaximumVertexTolerance = 0.0;
        if (!TryReadMetersPerUnit(
                aDocument, aMetersPerUnit)
            || !ShapeBounds(aShape, aSourceBounds)
            || !ShapeVolume(aShape, aSourceVolume)
            || !MaximumVertexTolerance(
                aShape, aMaximumVertexTolerance)) {
            return Standard_False;
        }
        Standard_Real aMinimumThickness = 0.0;
        Standard_Real aMaximumThickness = 0.0;
        Standard_Real aKernelTolerance = 0.0;
        if (!ComputeThicknessRange(
                aSourceBounds,
                aMetersPerUnit,
                aMaximumVertexTolerance,
                aMinimumThickness,
                aMaximumThickness,
                aKernelTolerance)) {
            return Standard_False;
        }
        (void)aKernelTolerance;

        const std::string anEntityIdentifier =
            myDoc->EntityIdentifierForLabel(
                theSelection.documentLabel);
        const std::string aDefinitionIdentifier =
            myDoc->DefinitionIdentifierForLabel(
                theSelection.documentLabel);
        if (anEntityIdentifier.empty()
            || aDefinitionIdentifier.empty()) {
            return Standard_False;
        }

        std::unique_ptr<Source> aSource =
            std::make_unique<Source>();
        aSource->original = theSelection.original;
        aSource->label = theSelection.documentLabel;
        aSource->shape = aShape;
        aSource->openingFace = theSelection.openingFace;
        aSource->transform = aTransform;
        aSource->selectionMode = theSelection.selectionMode;
        aSource->faceTopologyIndex =
            static_cast<Standard_Size>(aFaceIndex - 1);
        aSource->topologyNodeCount = aTopologyNodeCount;
        aSource->metersPerUnit = aMetersPerUnit;
        aSource->sourceVolume = aSourceVolume;
        aSource->minimumThickness = aMinimumThickness;
        aSource->maximumThickness = aMaximumThickness;
        aSource->entityIdentifier = anEntityIdentifier;
        aSource->definitionIdentifier = aDefinitionIdentifier;

        mySource = std::move(aSource);
        myThickness = 0.0;
        myRequestedFingerprint.clear();
        myPendingCandidate.Nullify();
        myCanApply = Standard_False;
        mySelectionFrozen = Standard_False;
        myStateValid = Standard_True;
        myOwnsDocumentCommand = Standard_False;
        myState = ShellPreviewState::Selecting;
        notifyPreviewStateChanged();
        return Standard_True;
    } catch (...) {
        clearState();
        return Standard_False;
    }
}

Standard_Boolean ShellOperationController::sourceIsCurrent() const noexcept
{
    if (mySource == nullptr || myDoc.IsNull()) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(TDocStd_Document) aDocument = myDoc->Document();
        const TopoDS_Shape aStoredShape =
            XCAFDoc_ShapeTool::GetShape(mySource->label);
        if (aDocument.IsNull() || aDocument->HasOpenCommand()
            || mySource->label.IsNull()
            || mySource->label.Data() != aDocument->GetData()
            || mySource->original.IsNull()
            || mySource->shape.IsNull()
            || aStoredShape.IsNull()
            || !IsBRepModelingLabel(myDoc, mySource->label)
            || !myDoc->IsPresentationEditable(mySource->original)
            || !myDoc->ShapeLabel(mySource->original).IsEqual(
                mySource->label)
            || !aStoredShape.IsEqual(mySource->shape)
            || !mySource->original->Shape().IsEqual(
                mySource->shape)
            || !TransformsMatch(
                mySource->original->LocalTransformation(),
                mySource->transform)
            || !TransformsMatch(
                myDoc->ObjectTransformForLabel(mySource->label),
                mySource->transform)
            || HasStyledXCAFSubshape(
                aDocument,
                mySource->label,
                kMaximumStyledSubshapeLabels)
            || myDoc->EntityIdentifierForLabel(mySource->label)
                != mySource->entityIdentifier
            || myDoc->DefinitionIdentifierForLabel(mySource->label)
                != mySource->definitionIdentifier) {
            return Standard_False;
        }
        TopTools_IndexedMapOfShape aFaces;
        TopExp::MapShapes(mySource->shape, TopAbs_FACE, aFaces);
        const Standard_Integer aFaceIndex =
            aFaces.FindIndex(mySource->openingFace);
        return aFaceIndex > 0
            && static_cast<Standard_Size>(aFaceIndex - 1)
                == mySource->faceTopologyIndex
            && IsSinglePlanarOpening(mySource->openingFace);
    } catch (...) {
        return Standard_False;
    }
}

std::string ShellOperationController::selectionFingerprint(
    Standard_Boolean& theComplete) const
{
    theComplete = Standard_False;
    try {
        if (!sourceIsCurrent() || mySource == nullptr) {
            return {};
        }
        std::ostringstream aFingerprint;
        aFingerprint << std::setprecision(17)
            << myThickness << '|'
            << mySource->entityIdentifier << '|'
            << mySource->definitionIdentifier << ':'
            << reinterpret_cast<std::uintptr_t>(
                mySource->shape.TShape().get()) << ':'
            << reinterpret_cast<std::uintptr_t>(
                mySource->openingFace.TShape().get()) << ':'
            << mySource->faceTopologyIndex;
        for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
            for (Standard_Integer aColumn = 1; aColumn <= 4;
                 ++aColumn) {
                aFingerprint << ':'
                    << mySource->transform.Value(
                        aRow, aColumn);
            }
        }
        theComplete = Standard_True;
        return aFingerprint.str();
    } catch (...) {
        return {};
    }
}

void ShellOperationController::cancelWorkerRequests() noexcept
{
    if (myGeneration
        != std::numeric_limits<std::uint64_t>::max()) {
        ++myGeneration;
    }
    if (myWorker != nullptr) {
        myWorker->cancelAll();
    }
    myCanApply = Standard_False;
    myRequestedFingerprint.clear();
}

Standard_Boolean
ShellOperationController::enqueuePreviewRequest() noexcept
{
    if (mySource == nullptr) {
        return Standard_False;
    }
    try {
        Standard_Boolean hasFingerprint = Standard_False;
        const std::string aFingerprint =
            selectionFingerprint(hasFingerprint);
        if (!hasFingerprint || aFingerprint.empty()
            || myGeneration
                == std::numeric_limits<std::uint64_t>::max()) {
            return Standard_False;
        }

        BRepBuilderAPI_Copy aCopy(
            mySource->shape,
            Standard_True,
            Standard_False);
        const TopoDS_Shape aCopiedShape = aCopy.Shape();
        const TopoDS_Shape aCopiedFaceShape =
            aCopy.ModifiedShape(mySource->openingFace);
        if (aCopiedShape.IsNull()
            || aCopiedShape.ShapeType() != TopAbs_SOLID
            || aCopiedFaceShape.IsNull()
            || aCopiedFaceShape.ShapeType() != TopAbs_FACE) {
            return Standard_False;
        }

        Standard_Real aCopiedBounds[6] = {};
        Standard_Real aCopiedVolume = 0.0;
        Standard_Real aMaximumVertexTolerance = 0.0;
        if (!ShapeBounds(aCopiedShape, aCopiedBounds)
            || !ShapeVolume(aCopiedShape, aCopiedVolume)
            || !MaximumVertexTolerance(
                aCopiedShape, aMaximumVertexTolerance)) {
            return Standard_False;
        }
        Standard_Real aMinimumThickness = 0.0;
        Standard_Real aMaximumThickness = 0.0;
        Standard_Real aKernelTolerance = 0.0;
        if (!ComputeThicknessRange(
                aCopiedBounds,
                mySource->metersPerUnit,
                aMaximumVertexTolerance,
                aMinimumThickness,
                aMaximumThickness,
                aKernelTolerance)
            || myThickness < aMinimumThickness
            || myThickness > aMaximumThickness) {
            return Standard_False;
        }

        ShellPreviewWorkerRequest aRequest;
        aRequest.generation = ++myGeneration;
        aRequest.thickness = myThickness;
        aRequest.tolerance = aKernelTolerance;
        aRequest.sourceVolume = aCopiedVolume;
        std::copy(
            std::begin(aCopiedBounds),
            std::end(aCopiedBounds),
            std::begin(aRequest.sourceBounds));
        aRequest.fingerprint = aFingerprint;
        aRequest.source = aCopiedShape;
        aRequest.openingFace =
            TopoDS::Face(aCopiedFaceShape);
        aRequest.cancellation =
            std::make_shared<std::atomic_bool>(false);
#ifdef DEBUG
        aRequest.maximumResultTopologyNodes =
            myDebugMaximumResultTopologyNodes;
#endif
        if (myWorker == nullptr) {
            myWorker = std::make_shared<ShellPreviewWorker>(
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

Standard_Boolean ShellOperationController::setThickness(
    const Standard_Real theThickness) noexcept
{
    if (mySource == nullptr || !std::isfinite(theThickness)
        || myState == ShellPreviewState::Committing
        || myState == ShellPreviewState::OutcomeUnknown
        || myOwnsDocumentCommand
        || !myPendingCandidate.IsNull()
        || !sourceIsCurrent()
        || theThickness < mySource->minimumThickness
        || theThickness > mySource->maximumThickness) {
        return Standard_False;
    }
    const Standard_Boolean isSame =
        std::abs(myThickness - theThickness)
            <= Precision::Confusion();
    if (isSame && myState == ShellPreviewState::Ready) {
        return Standard_True;
    }

    cancelWorkerRequests();
    if ((!myPreviewResult.IsNull() || !myStateValid)
        && !discardPreview(Standard_True)) {
        myStateValid = Standard_False;
        myState = ShellPreviewState::Failed;
        notifyPreviewStateChanged();
        return Standard_False;
    }
    myStateValid = Standard_True;
    myThickness = theThickness;
    myState = ShellPreviewState::Computing;
    mySelectionFrozen = Standard_True;
    if (!enqueuePreviewRequest()) {
        myState = ShellPreviewState::Failed;
        notifyPreviewStateChanged();
        return Standard_False;
    }
    notifyPreviewStateChanged();
    return Standard_True;
}

void ShellOperationController::acceptWorkerResult(
    ShellPreviewWorkerResult theResult) noexcept
{
    if (theResult.generation != myGeneration
        || theResult.fingerprint != myRequestedFingerprint
        || std::abs(theResult.thickness - myThickness)
            > Precision::Confusion()
        || myState != ShellPreviewState::Computing
        || mySource == nullptr) {
#ifdef DEBUG
        ++myDebugStaleSuppressionCount;
#endif
        return;
    }
    if (theResult.outcome != ShellPreviewWorkerOutcome::Success
        || theResult.result.IsNull()
        || !sourceIsCurrent()
        || !installPreview(theResult.result)) {
        myCanApply = Standard_False;
        myState = ShellPreviewState::Failed;
        notifyPreviewStateChanged();
        return;
    }
    myCanApply = Standard_True;
    myStateValid = Standard_True;
    myState = ShellPreviewState::Ready;
    mySelectionFrozen = Standard_True;
#ifdef DEBUG
    ++myDebugAcceptedCount;
#endif
    notifyPreviewStateChanged();
}

Standard_Boolean ShellOperationController::installPreview(
    const TopoDS_Shape& theResult) noexcept
{
    if (mySource == nullptr || myContext.IsNull()
        || theResult.IsNull()
        || theResult.ShapeType() != TopAbs_SOLID) {
        return Standard_False;
    }
    Handle(AIS_Shape) aPresentation;
    try {
        OCC_CATCH_SIGNALS
        aPresentation = new AIS_Shape(theResult);
        aPresentation->SetLocalTransformation(
            mySource->transform);
        myDoc->LoadObjectMeterial(
            mySource->label, aPresentation);
        if (!myPreviewResult.IsNull()
            && !discardPreview(Standard_True)) {
            return Standard_False;
        }
        myPreviewResult = aPresentation;
        myContext->Display(
            myPreviewResult,
            AIS_Shaded,
            mySource->selectionMode,
            Standard_False);
        myContext->Deactivate(myPreviewResult);
        myContext->Remove(
            mySource->original, Standard_False);
        myContext->ClearSelected(Standard_False);
        myContext->UpdateCurrentViewer();
        return Standard_True;
    } catch (...) {
        if (!aPresentation.IsNull()) {
            try {
                myContext->Remove(
                    aPresentation, Standard_False);
            } catch (...) {
            }
        }
        if (myPreviewResult == aPresentation) {
            myPreviewResult.Nullify();
        }
        (void)discardPreview(Standard_True);
        return Standard_False;
    }
}

Standard_Boolean ShellOperationController::discardPreview(
    const Standard_Boolean theRestoreSource) noexcept
{
#ifdef DEBUG
    if (myDebugPreviewEraseFailureCount > 0) {
        --myDebugPreviewEraseFailureCount;
        return Standard_False;
    }
#endif
    bool didResolveAll = true;
    if (!myContext.IsNull()) {
        if (!myPreviewResult.IsNull()) {
            try {
                myContext->Remove(
                    myPreviewResult, Standard_False);
            } catch (...) {
                didResolveAll = false;
            }
        }
        if (theRestoreSource && mySource != nullptr) {
            if (mySource->original.IsNull()) {
                didResolveAll = false;
            } else {
                try {
                    myContext->Display(
                        mySource->original,
                        AIS_Shaded,
                        mySource->selectionMode,
                        Standard_False);
                    myContext->Activate(
                        mySource->original,
                        mySource->selectionMode,
                        Standard_False);
                } catch (...) {
                    didResolveAll = false;
                }
            }
        }
        try {
            myContext->ClearSelected(Standard_False);
            myContext->UpdateCurrentViewer();
        } catch (...) {
            didResolveAll = false;
        }
    } else if (!myPreviewResult.IsNull() || theRestoreSource) {
        didResolveAll = false;
    }
    if (didResolveAll) {
        myPreviewResult.Nullify();
    }
    return didResolveAll;
}

Standard_Real ShellOperationController::thickness() const noexcept
{
    return myThickness;
}

Standard_Real ShellOperationController::defaultThickness() const noexcept
{
    if (mySource == nullptr
        || !std::isfinite(mySource->metersPerUnit)
        || mySource->metersPerUnit <= 0.0) {
        return 0.0;
    }
    const Standard_Real aPhysicalDefault =
        kDefaultPhysicalThicknessMeters
        / mySource->metersPerUnit;
    const Standard_Real aFractionDefault =
        mySource->maximumThickness
        * kDefaultThicknessFraction
        / kMaximumThicknessFraction;
    return std::max(
        mySource->minimumThickness,
        std::min({
            aPhysicalDefault,
            aFractionDefault,
            mySource->maximumThickness,
        }));
}

Standard_Real ShellOperationController::metersPerUnit() const noexcept
{
    return mySource != nullptr ? mySource->metersPerUnit : 0.0;
}

std::pair<Standard_Real, Standard_Real>
ShellOperationController::thicknessRange() const noexcept
{
    return mySource != nullptr
        ? std::pair<Standard_Real, Standard_Real>{
            mySource->minimumThickness,
            mySource->maximumThickness}
        : std::pair<Standard_Real, Standard_Real>{0.0, 0.0};
}

Standard_Boolean ShellOperationController::canApply() const noexcept
{
    const Standard_Boolean hasReadyPreview =
        myState == ShellPreviewState::Ready
        && myCanApply && myStateValid
        && mySource != nullptr
        && !myPreviewResult.IsNull()
        && !myPreviewResult->Shape().IsNull()
        && myPreviewResult->Shape().ShapeType() == TopAbs_SOLID
        && myPendingCandidate.IsNull()
        && !myOwnsDocumentCommand;
    const Standard_Boolean hasRetryableOutcome =
        myState == ShellPreviewState::OutcomeUnknown
        && (myOwnsDocumentCommand
            || !myPendingCandidate.IsNull());
    return hasReadyPreview || hasRetryableOutcome;
}

Standard_Boolean ShellOperationController::hasUnresolvedState() const
    noexcept
{
    return !myPreviewResult.IsNull()
        || !myPendingCandidate.IsNull()
        || myOwnsDocumentCommand;
}

Standard_Boolean ShellOperationController::hasActiveOperation() const
    noexcept
{
    return mySource != nullptr || hasUnresolvedState()
        || myState != ShellPreviewState::Unavailable;
}

Standard_Boolean ShellOperationController::isSelectionFrozen() const
    noexcept
{
    return mySelectionFrozen;
}

ShellPreviewState ShellOperationController::previewState() const noexcept
{
    return myState;
}

std::uint64_t ShellOperationController::previewGeneration() const noexcept
{
    return myGeneration;
}

Standard_Boolean ShellOperationController::capturePreview(
    ShellPreviewCapture& theCapture) const noexcept
{
    theCapture = {};
    if (myState != ShellPreviewState::Ready
        || !canApply() || !sourceIsCurrent()
        || mySource == nullptr || myContext.IsNull()) {
        return Standard_False;
    }
    try {
        TColStd_ListOfInteger anActiveModes;
        myContext->ActivatedModes(
            myPreviewResult, anActiveModes);
        if (!myContext->IsDisplayed(myPreviewResult)
            || !anActiveModes.IsEmpty()
            || mySource->label.IsNull()) {
            return Standard_False;
        }
        theCapture.result = myPreviewResult;
        theCapture.suppressedSourceLabel = mySource->label;
        return Standard_True;
    } catch (...) {
        theCapture = {};
        return Standard_False;
    }
}

ShellOperationController::DocumentState
ShellOperationController::inspectDocumentState() const noexcept
{
    if (mySource == nullptr || myDoc.IsNull()) {
        return DocumentState::Unavailable;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(TDocStd_Document) aDocument =
            myDoc->Document();
        if (aDocument.IsNull() || aDocument->GetData().IsNull()
            || mySource->label.IsNull()
            || mySource->label.Data() != aDocument->GetData()) {
            return DocumentState::Unavailable;
        }
        if (aDocument->HasOpenCommand()) {
            return myOwnsDocumentCommand
                ? DocumentState::OpenCommand
                : DocumentState::Unavailable;
        }
        const TopoDS_Shape aStored =
            XCAFDoc_ShapeTool::GetShape(mySource->label);
        if (aStored.IsNull()
            || mySource->entityIdentifier.empty()
            || mySource->definitionIdentifier.empty()
            || myDoc->EntityIdentifierForLabel(mySource->label)
                != mySource->entityIdentifier
            || myDoc->DefinitionIdentifierForLabel(mySource->label)
                != mySource->definitionIdentifier
            || !TransformsMatch(
                myDoc->ObjectTransformForLabel(mySource->label),
                mySource->transform)) {
            return DocumentState::Other;
        }
        if (!myPendingCandidate.IsNull()
            && aStored.IsEqual(myPendingCandidate)) {
            return myDoc->GeometryRepresentationForLabel(
                    mySource->label)
                    == OcctGeometryRepresentation::BRep
                && myDoc->IsEditableFreeSimpleDefinitionLabel(
                    mySource->label)
                ? DocumentState::Candidate
                : DocumentState::Other;
        }
        if (!mySource->shape.IsNull()
            && aStored.IsEqual(mySource->shape)) {
            return IsBRepModelingLabel(myDoc, mySource->label)
                ? DocumentState::Original
                : DocumentState::Other;
        }
        return DocumentState::Other;
    } catch (...) {
        return DocumentState::Unavailable;
    }
}

Standard_Boolean ShellOperationController::abortOwnedCommand() noexcept
{
    if (!myOwnsDocumentCommand || myDoc.IsNull()) {
        return Standard_False;
    }
    try {
        const Handle(TDocStd_Document) aDocument =
            myDoc->ChangeDocument();
        if (aDocument.IsNull()) {
            return Standard_False;
        }
        if (aDocument->HasOpenCommand()) {
#ifdef DEBUG
            if (myDebugAbortFailureCount > 0) {
                --myDebugAbortFailureCount;
                return Standard_False;
            }
#endif
            aDocument->AbortCommand();
        }
        if (aDocument->HasOpenCommand()) {
            return Standard_False;
        }
        myOwnsDocumentCommand = Standard_False;
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

ShellApplyResult
ShellOperationController::finishCommittedOperation() noexcept
{
    myOwnsDocumentCommand = Standard_False;
    if (!discardPreview(Standard_False)) {
        myState = ShellPreviewState::OutcomeUnknown;
        myCanApply = Standard_True;
        notifyPreviewStateChanged();
        return ShellApplyResult::NoChange;
    }
    try {
        myDoc->NotifyChanges();
    } catch (...) {
    }
    clearState();
    notifyPreviewStateChanged();
    return ShellApplyResult::AppliedNeedsDocumentRedraw;
}

ShellApplyResult ShellOperationController::apply() noexcept
{
    if (!hasActiveOperation() || mySource == nullptr) {
        return ShellApplyResult::NoChange;
    }
    const auto inspectAfterCommit = [this]() noexcept {
#ifdef DEBUG
        if (myDebugPostCommitInspectFailureCount > 0) {
            --myDebugPostCommitInspectFailureCount;
            return DocumentState::Unavailable;
        }
#endif
        return inspectDocumentState();
    };
    const auto resolveAfterMutation = [this, &inspectAfterCommit]()
        noexcept {
        DocumentState aState = inspectAfterCommit();
        if (aState == DocumentState::OpenCommand
            && abortOwnedCommand()) {
            aState = inspectAfterCommit();
        }
        if (aState == DocumentState::Candidate) {
            return finishCommittedOperation();
        }
        if (aState == DocumentState::Original) {
            myPendingCandidate.Nullify();
            myOwnsDocumentCommand = Standard_False;
            myCanApply = !myPreviewResult.IsNull()
                && myStateValid && sourceIsCurrent();
            myState = myCanApply
                ? ShellPreviewState::Ready
                : ShellPreviewState::Failed;
            notifyPreviewStateChanged();
            return ShellApplyResult::NoChange;
        }
        myState = ShellPreviewState::OutcomeUnknown;
        myCanApply = Standard_True;
        notifyPreviewStateChanged();
        return ShellApplyResult::NoChange;
    };

    if (myState == ShellPreviewState::OutcomeUnknown
        || myOwnsDocumentCommand
        || !myPendingCandidate.IsNull()) {
        return resolveAfterMutation();
    }
    if (myState != ShellPreviewState::Ready || !canApply()
        || !sourceIsCurrent()) {
        myCanApply = Standard_False;
        myState = ShellPreviewState::Failed;
        notifyPreviewStateChanged();
        return ShellApplyResult::NoChange;
    }
    Standard_Boolean hasFingerprint = Standard_False;
    if (selectionFingerprint(hasFingerprint)
            != myRequestedFingerprint
        || !hasFingerprint) {
        myCanApply = Standard_False;
        myState = ShellPreviewState::Failed;
        notifyPreviewStateChanged();
        return ShellApplyResult::NoChange;
    }

    Handle(TDocStd_Document) aDocument;
    try {
        OCC_CATCH_SIGNALS
        aDocument = myDoc->ChangeDocument();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()
            || aDocument->GetUndoLimit() == 0
            || myPreviewResult.IsNull()
            || myPreviewResult->Shape().IsNull()) {
            return ShellApplyResult::NoChange;
        }
        myPendingCandidate = myPreviewResult->Shape();
        myState = ShellPreviewState::Committing;
        myCanApply = Standard_False;
        mySelectionFrozen = Standard_True;
        notifyPreviewStateChanged();

        // Record ownership intent before NewCommand so an exception cannot
        // strand an open document command behind a false ownership bit.
        myOwnsDocumentCommand = Standard_True;
        aDocument->NewCommand();
        if (!aDocument->HasOpenCommand()) {
            myOwnsDocumentCommand = Standard_False;
            myPendingCandidate.Nullify();
            myCanApply = Standard_True;
            myState = ShellPreviewState::Ready;
            notifyPreviewStateChanged();
            return ShellApplyResult::NoChange;
        }
#ifdef DEBUG
        if (myDebugTransactionFailureCount > 0) {
            --myDebugTransactionFailureCount;
            return resolveAfterMutation();
        }
#endif
        if (!myDoc->ReplaceShape(
                mySource->label, myPreviewResult)) {
            return resolveAfterMutation();
        }
        const TopoDS_Shape aStored =
            XCAFDoc_ShapeTool::GetShape(mySource->label);
        if (aStored.IsNull()
            || !aStored.IsEqual(myPendingCandidate)
            || myDoc->EntityIdentifierForLabel(mySource->label)
                != mySource->entityIdentifier
            || myDoc->DefinitionIdentifierForLabel(mySource->label)
                != mySource->definitionIdentifier
            || myDoc->GeometryRepresentationForLabel(
                    mySource->label)
                != OcctGeometryRepresentation::BRep
            || !TransformsMatch(
                myDoc->ObjectTransformForLabel(mySource->label),
                mySource->transform)) {
            return resolveAfterMutation();
        }
        try {
            Standard_Boolean aCommitReported =
                aDocument->CommitCommand();
#ifdef DEBUG
            const Standard_Integer aCommitMode =
                myDebugCommitMode;
            myDebugCommitMode = 0;
            if (aCommitMode == 1) {
                aCommitReported = Standard_False;
            } else if (aCommitMode == 2) {
                throw Standard_Failure(
                    "Injected Shell commit exception after close");
            }
#endif
            (void)aCommitReported;
        } catch (...) {
            // The stable label and document command state below are
            // authoritative; CommitCommand's Boolean is not an outcome token.
        }
    } catch (...) {
        return resolveAfterMutation();
    }
    return resolveAfterMutation();
}

Standard_Boolean ShellOperationController::cancel() noexcept
{
    if (!hasActiveOperation()) {
        clearState();
        return Standard_True;
    }
    if (myState == ShellPreviewState::Committing
        || myState == ShellPreviewState::OutcomeUnknown
        || myOwnsDocumentCommand
        || !myPendingCandidate.IsNull()) {
        myState = ShellPreviewState::OutcomeUnknown;
        myCanApply = Standard_True;
        notifyPreviewStateChanged();
        return Standard_False;
    }
    cancelWorkerRequests();
    if (!discardPreview(Standard_True)) {
        myStateValid = Standard_False;
        myState = ShellPreviewState::Failed;
        notifyPreviewStateChanged();
        return Standard_False;
    }
    clearState();
    notifyPreviewStateChanged();
    return Standard_True;
}

void ShellOperationController::clearState() noexcept
{
    mySource.reset();
    myPreviewResult.Nullify();
    myPendingCandidate.Nullify();
    myRequestedFingerprint.clear();
    myThickness = 0.0;
    myCanApply = Standard_False;
    mySelectionFrozen = Standard_False;
    myStateValid = Standard_True;
    myOwnsDocumentCommand = Standard_False;
    myState = ShellPreviewState::Unavailable;
    if (myGeneration
        != std::numeric_limits<std::uint64_t>::max()) {
        ++myGeneration;
    }
}

#ifdef DEBUG
ShellPreviewDebugState
ShellOperationController::debugPreviewState() const noexcept
{
    ShellPreviewDebugState aState;
    aState.state = myState;
    aState.generation = myGeneration;
    aState.activeOperation = hasActiveOperation();
    aState.canApply = canApply();
    aState.selectionFrozen = mySelectionFrozen;
    aState.ownsDocumentCommand = myOwnsDocumentCommand;
    aState.thickness = myThickness;
    if (mySource != nullptr) {
        aState.minimumThickness =
            mySource->minimumThickness;
        aState.maximumThickness =
            mySource->maximumThickness;
        aState.metersPerUnit = mySource->metersPerUnit;
        aState.capturedFaceTopologyIndex =
            static_cast<Standard_Integer>(
                mySource->faceTopologyIndex);
        aState.sourceTopologyNodeCount =
            mySource->topologyNodeCount;
        aState.sourceVolume = mySource->sourceVolume;
    }
    try {
        OCC_CATCH_SIGNALS
        const TopoDS_Shape aCandidate =
            !myPreviewResult.IsNull()
            ? myPreviewResult->Shape()
            : myPendingCandidate;
        if (!aCandidate.IsNull()) {
            Standard_Size aTopologyCount = 0;
            if (CountBoundedTopology(
                    aCandidate,
                    kMaximumResultTopologyNodes,
                    aTopologyCount)) {
                aState.candidateTopologyNodeCount =
                    aTopologyCount;
            }
            aState.candidateSolidCount =
                aCandidate.ShapeType() == TopAbs_SOLID ? 1 : 0;
            (void)ShapeVolume(
                aCandidate, aState.candidateVolume);
            (void)ShapeBounds(
                aCandidate, aState.candidateBounds);
        }
    } catch (...) {
    }
    try {
        const Handle(TDocStd_Document) aDocument =
            myDoc.IsNull()
            ? Handle(TDocStd_Document)()
            : myDoc->Document();
        aState.documentCommandOpen = !aDocument.IsNull()
            && aDocument->HasOpenCommand();
    } catch (...) {
        aState.documentCommandOpen = Standard_True;
    }
    aState.acceptedCount = myDebugAcceptedCount;
    aState.staleSuppressionCount =
        myDebugStaleSuppressionCount;
    if (myWorker != nullptr) {
        const ShellPreviewWorker::DebugSnapshot aWorker =
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

void ShellOperationController::debugSetWorkerBlocked(
    const Standard_Boolean theBlocked) noexcept
{
    myDebugWorkerBlocked = theBlocked;
    if (myWorker != nullptr) {
        myWorker->debugSetBlocked(theBlocked);
    }
}

void ShellOperationController::debugSetMaximumCaptureTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    myDebugMaximumCaptureTopologyNodes =
        std::max<Standard_Size>(
            1,
            std::min(
                theLimit,
                kMaximumSourceTopologyNodes));
}

void ShellOperationController::debugSetMaximumResultTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    myDebugMaximumResultTopologyNodes =
        std::max<Standard_Size>(
            1,
            std::min(
                theLimit,
                kMaximumResultTopologyNodes));
}

void ShellOperationController::debugSetTransactionFailureCount(
    const Standard_Size theCount) noexcept
{
    myDebugTransactionFailureCount = theCount;
}

void ShellOperationController::debugSetAbortFailureCount(
    const Standard_Size theCount) noexcept
{
    myDebugAbortFailureCount = theCount;
}

void ShellOperationController::debugSetPreviewEraseFailureCount(
    const Standard_Size theCount) noexcept
{
    myDebugPreviewEraseFailureCount = theCount;
}

void ShellOperationController::debugSetCommitMode(
    const Standard_Integer theMode) noexcept
{
    myDebugCommitMode =
        theMode >= 0 && theMode <= 2 ? theMode : 0;
}

void ShellOperationController::debugSetPostCommitInspectFailureCount(
    const Standard_Size theCount) noexcept
{
    myDebugPostCommitInspectFailureCount = theCount;
}

Standard_Boolean
ShellOperationController::debugMutateSourcePersistedTransform() noexcept
{
    if (mySource == nullptr || !sourceIsCurrent()
        || myDoc.IsNull()) {
        return Standard_False;
    }
    Handle(TDocStd_Document) aDocument;
    try {
        OCC_CATCH_SIGNALS
        aDocument = myDoc->ChangeDocument();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()
            || mySource->label.IsNull()
            || mySource->label.Data()
                != aDocument->GetData()) {
            return Standard_False;
        }
        const Standard_Real anOriginalX =
            myDoc->ObjectTransformForLabel(mySource->label)
                .TranslationPart().X();
        if (!std::isfinite(anOriginalX)) {
            return Standard_False;
        }
        aDocument->NewCommand();
        if (!aDocument->HasOpenCommand()) {
            return Standard_False;
        }
        TDataStd_Real::Set(
            mySource->label.FindChild(1),
            anOriginalX + 1.0);
        try {
            (void)aDocument->CommitCommand();
        } catch (...) {
        }
        if (aDocument->HasOpenCommand()) {
            aDocument->AbortCommand();
            return Standard_False;
        }
        return !TransformsMatch(
                myDoc->ObjectTransformForLabel(
                    mySource->label),
                mySource->transform)
            && TransformsMatch(
                mySource->original->LocalTransformation(),
                mySource->transform);
    } catch (...) {
        try {
            if (!aDocument.IsNull()
                && aDocument->HasOpenCommand()) {
                aDocument->AbortCommand();
            }
        } catch (...) {
        }
        return Standard_False;
    }
}
#endif

} // namespace core3d
