//
//  BevelOperationController.cpp
//  Core3D
//

#include "BevelOperationController.hpp"
#include "../OCCTKit/FilletFailureClassifier.hxx"

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBndLib.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <Geom_Surface.hxx>
#include <gp_Pln.hxx>
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
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_MapOfShape.hxx>
#include <BRepTools_History.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>

#include <dispatch/dispatch.h>
#include <pthread.h>

#include <algorithm>
#include <atomic>
#include <chrono>
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
    std::shared_ptr<const retained_edge_treatment::DetachedInput>
        retainedInput;
};

struct BevelPreviewWorkerRequest {
    std::uint64_t generation = 0;
    Standard_Real value = 0.0;
    std::string fingerprint;
    std::vector<BevelPreviewWorkerSource> sources;
    RetainedBevelAdapter retainedAdapter;
    std::shared_ptr<std::atomic_bool> cancellation;
    Standard_Size maximumResultTopologyNodes =
        BevelOperationController::kMaxResultTopologyNodes;
    Standard_Size maximumResultSolids =
        BevelOperationController::kMaxResultSolids;
    //! K11 (CTO 19 Nov 18:39): 0.1 mm in stored units, the step the HintLine
    //! shows a named size in; 0 when the part's size is unknown.
    Standard_Real namedSizeStep = 0.0;
};

struct BevelPreviewWorkerResult {
    std::uint64_t generation = 0;
    Standard_Real value = 0.0;
    std::string fingerprint;
    BevelPreviewWorkerOutcome outcome = BevelPreviewWorkerOutcome::Failed;
    BevelFailureKind failureKind = BevelFailureKind::Other;
    //! K11: largest round (stored units) that builds after a size refusal; 0 = unknown.
    Standard_Real largestFittingValue = 0.0;
    std::vector<TopoDS_Shape> results;
    std::vector<std::shared_ptr<const retained_edge_treatment::DetachedResult>>
        retainedResults;
    Standard_Boolean computeWasMainThread = Standard_False;
};

namespace {

Standard_Boolean RetainedBevelCoverageMatches(
    const OcctRetainedRecipeCoverage theCoverage,
    const std::shared_ptr<const RetainedBevelAdapter::Capture>& theCapture) noexcept
{
    if (!theCapture || !theCapture->snapshot) return Standard_False;
    switch (theCapture->snapshot->effectiveDefinition().base.family) {
        case retained_edge_treatment::SourceFamily::Profile:
            return theCoverage == OcctRetainedRecipeCoverage::CurrentProfile;
        case retained_edge_treatment::SourceFamily::Enclosure:
            return theCoverage
                == OcctRetainedRecipeCoverage::PresentOutsideP4Coverage;
        default:
            return Standard_False;
    }
}

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
    const TDF_Label& theLabel,
    const Standard_Boolean allowSavedCut = Standard_False) noexcept
{
    if (theDocument.IsNull() || theLabel.IsNull()
        || !theDocument->IsEditableFreeSimpleDefinitionLabel(theLabel)
        || (!theDocument->HasNoSavedSweepForTopology(theLabel)
            && !(allowSavedCut
                && theDocument->CanBakeSavedCutForFaceEdit(theLabel)))) {
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
    const Standard_Size theMaximumLabels,
    const Standard_Boolean theAllowFaceMaterials = Standard_False) noexcept
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
                || (!XCAFDoc_VisMaterialTool::GetShapeMaterial(aLabel).IsNull()
                    // A colour put on one face (a material only) is carried by Bevel.
                    && !(theAllowFaceMaterials
                        && !XCAFDoc_ShapeTool::GetShape(aLabel).IsNull()
                        && XCAFDoc_ShapeTool::GetShape(aLabel).ShapeType() == TopAbs_FACE))
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

//! Counts occurrences, not unique TShapes. Shared/adversarial topology must
//! consume the synchronous source budget before any full map or BRepCheck.
Standard_Boolean AccumulateBoundedTopologyOccurrences(
    const TopoDS_Shape& theShape,
    const Standard_Size thePerShapeLimit,
    const Standard_Size theAggregateLimit,
    Standard_Size& theAggregateCount,
    Standard_Size& theShapeCount) noexcept
{
    theShapeCount = 0;
    if (theShape.IsNull() || thePerShapeLimit == 0
        || theAggregateLimit == 0
        || theAggregateCount > theAggregateLimit) {
        return Standard_False;
    }
    try {
        std::vector<TopoDS_Shape> aPending{theShape};
        while (!aPending.empty()) {
            const TopoDS_Shape aCurrent = aPending.back();
            aPending.pop_back();
            if (aCurrent.IsNull()
                || ++theShapeCount > thePerShapeLimit
                || ++theAggregateCount > theAggregateLimit) {
                theShapeCount = 0;
                return Standard_False;
            }
            std::vector<TopoDS_Shape> aChildren;
            for (TopoDS_Iterator aChild(
                     aCurrent, Standard_True, Standard_True);
                 aChild.More(); aChild.Next()) {
                const Standard_Size aPerShapeRemaining =
                    thePerShapeLimit - theShapeCount;
                const Standard_Size anAggregateRemaining =
                    theAggregateLimit - theAggregateCount;
                if (aPending.size() + aChildren.size() + 1U
                        > static_cast<std::size_t>(aPerShapeRemaining)
                    || aPending.size() + aChildren.size() + 1U
                        > static_cast<std::size_t>(anAggregateRemaining)) {
                    theShapeCount = 0;
                    return Standard_False;
                }
                aChildren.push_back(aChild.Value());
            }
            for (auto aChild = aChildren.rbegin();
                 aChild != aChildren.rend(); ++aChild) {
                aPending.push_back(*aChild);
            }
        }
        return theShapeCount > 0;
    } catch (...) {
        theShapeCount = 0;
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

//! Proves the one canonical solid of a plain-profile bevel candidate (D81).
//! The fillet/chamfer builders legitimately return a forward COMPOUND that
//! wraps the single result solid, while the strict document replacement
//! proof requires the replacement root itself to be a SOLID. Accept a valid
//! forward SOLID unchanged. For a wrapper accept only a forward COMPOUND
//! whose entire direct child list holds exactly one occurrence and that
//! child is a valid forward solid, examined with cumulative orientation and
//! location so the kept child preserves its located geometry. Refuse null
//! or empty wrappers, multiple children (including aliases of one TShape),
//! mixed contents, reversed or invalid solids, nested or unsupported
//! wrappers and any budget overflow. The candidate must pass the bounded
//! topology walk, full BRep validity and a finite positive volume. No
//! rebuilding, sewing, healing or orientation repair is performed: removing
//! the exactly proved redundant wrapper keeps every bit of candidate
//! geometry. Read-only precedent: retained_part_boolean::AnalyticResultSolid
//! in PartBooleanBuild.hxx.
Standard_Boolean CanonicalPlainProfileBevelCandidate(
    const TopoDS_Shape& theResult,
    const Standard_Size theMaximumNodes,
    TopoDS_Shape& theCandidate) noexcept
{
    theCandidate.Nullify();
    if (theResult.IsNull() || theMaximumNodes == 0) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        TopoDS_Shape aCandidate;
        if (theResult.ShapeType() == TopAbs_SOLID) {
            if (theResult.Orientation() != TopAbs_FORWARD) {
                return Standard_False;
            }
            aCandidate = theResult;
        } else if (theResult.ShapeType() == TopAbs_COMPOUND) {
            if (theResult.Orientation() != TopAbs_FORWARD) {
                return Standard_False;
            }
            Standard_Size aChildCount = 0;
            for (TopoDS_Iterator aChild(
                     theResult, Standard_True, Standard_True);
                 aChild.More(); aChild.Next()) {
                if (++aChildCount > 1) {
                    return Standard_False;
                }
                aCandidate = aChild.Value();
            }
            if (aChildCount != 1 || aCandidate.IsNull()
                || aCandidate.ShapeType() != TopAbs_SOLID
                || aCandidate.Orientation() != TopAbs_FORWARD) {
                return Standard_False;
            }
        } else {
            return Standard_False;
        }
        Standard_Size aNodeCount = 0;
        if (!AccumulateBoundedTopology(
                aCandidate,
                theMaximumNodes,
                theMaximumNodes,
                aNodeCount)
            || !BRepCheck_Analyzer(
                aCandidate, Standard_True).IsValid()) {
            return Standard_False;
        }
        GProp_GProps aVolumeProperties;
        BRepGProp::VolumeProperties(aCandidate, aVolumeProperties);
        const Standard_Real aVolume = aVolumeProperties.Mass();
        if (!std::isfinite(aVolume) || aVolume <= 0.0) {
            return Standard_False;
        }
        theCandidate = aCandidate;
        return Standard_True;
    } catch (...) {
        theCandidate.Nullify();
        return Standard_False;
    }
}

//! The plain chamfer and fillet builders return a forward COMPOUND that wraps
//! the one result solid. Stored like that, the part's root is no longer a
//! single SOLID and the face tools (Push-pull, Shell, Split) refuse the part
//! after one Bevel. Take the one forward solid out of its wrapper, examined
//! with cumulative orientation and location as in
//! CanonicalPlainProfileBevelCandidate, so the kept child keeps its located
//! geometry; nothing is rebuilt. Anything else is returned unchanged and is
//! judged by the usual result checks.
#ifdef DEBUG
std::atomic_bool gDebugPlainResultKeepsBuilderWrapper{false};
#endif

TopoDS_Shape PlainBevelResultWithoutWrapper(const TopoDS_Shape& theResult) noexcept
{
    try {
        OCC_CATCH_SIGNALS
#ifdef DEBUG
        if (gDebugPlainResultKeepsBuilderWrapper.load()) {
            return theResult;
        }
#endif
        if (theResult.IsNull() || theResult.ShapeType() != TopAbs_COMPOUND
            || theResult.Orientation() != TopAbs_FORWARD) {
            return theResult;
        }
        TopoDS_Shape aSolid;
        Standard_Size aChildCount = 0;
        for (TopoDS_Iterator aChild(theResult, Standard_True, Standard_True);
             aChild.More(); aChild.Next()) {
            if (++aChildCount > 1) {
                return theResult;
            }
            aSolid = aChild.Value();
        }
        if (aChildCount != 1 || aSolid.IsNull()
            || aSolid.ShapeType() != TopAbs_SOLID
            || aSolid.Orientation() != TopAbs_FORWARD) {
            return theResult;
        }
        return aSolid;
    } catch (...) {
        return theResult;
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

//! CTO 20 Nov 12:23: a flat Bevel skips an edge that lies between two faces
//! in one plane. A flush Join leaves such seams (a side split in two flat
//! pieces); there is nothing to bevel there, and OCCT's chamfer builder
//! refused the whole Bevel on them, while the round one copes. Only the
//! chamfer is built from this list; the selection, the stored edge indices
//! and the round are unchanged. When every edge is such a seam the list is
//! returned unchanged, so the Bevel refuses exactly as before.
std::vector<TopoDS_Edge> ChamferEdgesWithoutFlatSeams(
    const TopoDS_Shape& theShape,
    const std::vector<TopoDS_Edge>& theEdges) noexcept
{
    try {
        OCC_CATCH_SIGNALS
        TopTools_IndexedDataMapOfShapeListOfShape aFacesOfEdge;
        TopExp::MapShapesAndUniqueAncestors(
            theShape, TopAbs_EDGE, TopAbs_FACE, aFacesOfEdge);
        std::vector<TopoDS_Edge> aKept;
        aKept.reserve(theEdges.size());
        for (const TopoDS_Edge& anEdge : theEdges) {
            Standard_Boolean isFlatSeam = Standard_False;
            const Standard_Integer anIndex = anEdge.IsNull()
                ? 0 : aFacesOfEdge.FindIndex(anEdge);
            if (anIndex > 0 && aFacesOfEdge(anIndex).Extent() == 2) {
                const TopoDS_Face aFirst =
                    TopoDS::Face(aFacesOfEdge(anIndex).First());
                const TopoDS_Face aSecond =
                    TopoDS::Face(aFacesOfEdge(anIndex).Last());
                const Handle(Geom_Surface) aFirstSurface =
                    BRep_Tool::Surface(aFirst);
                const Handle(Geom_Surface) aSecondSurface =
                    BRep_Tool::Surface(aSecond);
                if (!aFirst.IsSame(aSecond) && !aFirstSurface.IsNull()
                    && !aSecondSurface.IsNull()) {
                    const GeomAdaptor_Surface aFirstAdaptor(aFirstSurface);
                    const GeomAdaptor_Surface aSecondAdaptor(aSecondSurface);
                    if (aFirstAdaptor.GetType() == GeomAbs_Plane
                        && aSecondAdaptor.GetType() == GeomAbs_Plane) {
                        const gp_Pln aFirstPlane = aFirstAdaptor.Plane();
                        const gp_Pln aSecondPlane = aSecondAdaptor.Plane();
                        const Standard_Real aReach = 10.0 * std::max(
                            BRep_Tool::Tolerance(anEdge),
                            Precision::Confusion());
                        isFlatSeam = aFirstPlane.Axis().Direction().IsParallel(
                                aSecondPlane.Axis().Direction(),
                                1.0e-6)
                            && aFirstPlane.Distance(aSecondPlane.Location())
                                <= aReach;
                    }
                }
            }
            if (!isFlatSeam) {
                aKept.push_back(anEdge);
            }
        }
        return aKept.empty() ? theEdges : aKept;
    } catch (...) {
        return theEdges;
    }
}

//! The shape and edges a flat Bevel is built from. When the selection holds a
//! seam between two faces in one plane (a flush Join), skipping the seam alone
//! still leaves OCCT's chamfer builder a vertex where the seam meets bevelled
//! edges, and it refuses (20 Nov run push-pull-20261120-125311-69082). Then
//! the flat pieces are merged first (ShapeUpgrade_UnifySameDomain) and the
//! selected edges are followed into the merged solid; the seams are gone and
//! split collinear edges become one. With no seam selected, or if the merge
//! fails or loses every edge, the input is used as before.
struct ChamferInput {
    TopoDS_Shape shape;
    std::vector<TopoDS_Edge> edges;
};

ChamferInput ChamferInputWithoutFlatSeams(
    const TopoDS_Shape& theShape,
    const std::vector<TopoDS_Edge>& theEdges) noexcept
{
    ChamferInput anInput;
    anInput.shape = theShape;
    anInput.edges = ChamferEdgesWithoutFlatSeams(theShape, theEdges);
    if (anInput.edges.size() == theEdges.size()) {
        return anInput;
    }
    try {
        OCC_CATCH_SIGNALS
        ShapeUpgrade_UnifySameDomain aUnify(
            theShape, Standard_True, Standard_True, Standard_False);
        aUnify.AllowInternalEdges(Standard_False);
        aUnify.Build();
        const TopoDS_Shape aMerged = aUnify.Shape();
        if (aMerged.IsNull()
            || !BRepCheck_Analyzer(aMerged, Standard_True).IsValid()) {
            return anInput;
        }
        TopTools_IndexedMapOfShape aMergedEdges;
        TopExp::MapShapes(aMerged, TopAbs_EDGE, aMergedEdges);
        const Handle(BRepTools_History) aHistory = aUnify.History();
        TopTools_MapOfShape aSeen;
        std::vector<TopoDS_Edge> aMapped;
        for (const TopoDS_Edge& anEdge : anInput.edges) {
            if (!aHistory.IsNull() && aHistory->IsRemoved(anEdge)) {
                continue;
            }
            TopTools_ListOfShape aNow;
            if (!aHistory.IsNull()) {
                aNow = aHistory->Modified(anEdge);
            }
            if (aNow.IsEmpty()) {
                aNow.Append(anEdge);
            }
            for (TopTools_ListIteratorOfListOfShape anIt(aNow);
                 anIt.More(); anIt.Next()) {
                if (anIt.Value().IsNull()
                    || anIt.Value().ShapeType() != TopAbs_EDGE) {
                    continue;
                }
                const Standard_Integer anIndex =
                    aMergedEdges.FindIndex(anIt.Value());
                if (anIndex > 0 && aSeen.Add(aMergedEdges(anIndex))) {
                    aMapped.push_back(TopoDS::Edge(aMergedEdges(anIndex)));
                }
            }
        }
        if (aMapped.empty()) {
            return anInput;
        }
        ChamferInput aMergedInput;
        aMergedInput.shape = aMerged;
        aMergedInput.edges = std::move(aMapped);
        return aMergedInput;
    } catch (...) {
        return anInput;
    }
}

//! True when the same edges built at theValue (distance for a chamfer,
//! radius for a round) give one valid solid, exactly as a preview checks it.
//! theNodesBefore / theSolidsBefore are the counts of the parts the preview
//! already built before this one, so the probe counts topology across all
//! parts like the real attempt (Fable review of 202b233c, CTO 19 Nov 12:55):
//! a part refused for the shared node limit is never called "too large".
Standard_Boolean PlainBevelBuildsAt(
    const BevelPreviewWorkerSource& theSource,
    const Standard_Real theValue,
    const Standard_Boolean theIsChamfer,
    const std::shared_ptr<std::atomic_bool>& theCancellation,
    const Standard_Size theNodesBefore,
    const Standard_Size theSolidsBefore,
    const Standard_Size theMaximumNodes,
    const Standard_Size theMaximumSolids) noexcept
{
    try {
        OCC_CATCH_SIGNALS
        if (theSource.shape.IsNull() || theSource.edges.empty()
            || !(theValue > Precision::Confusion())
            || theCancellation == nullptr
            || theCancellation->load(std::memory_order_relaxed)) {
            return Standard_False;
        }
        Handle(BevelCancellationIndicator) aProgress =
            new BevelCancellationIndicator(theCancellation);
        TopoDS_Shape aShape;
        if (theIsChamfer) {
            const ChamferInput aChamferInput = ChamferInputWithoutFlatSeams(
                theSource.shape, theSource.edges);
            BRepFilletAPI_MakeChamfer aBuilder(aChamferInput.shape);
            for (const TopoDS_Edge& anEdge : aChamferInput.edges) {
                aBuilder.Add(theValue, anEdge);
            }
            aBuilder.Build(aProgress->Start());
            if (!aBuilder.IsDone()
                || theCancellation->load(std::memory_order_relaxed)) {
                return Standard_False;
            }
            aShape = PlainBevelResultWithoutWrapper(aBuilder.Shape());
        } else {
            BRepFilletAPI_MakeFillet aBuilder(theSource.shape);
            for (const TopoDS_Edge& anEdge : theSource.edges) {
                aBuilder.Add(theValue, anEdge);
            }
            aBuilder.Build(aProgress->Start());
            if (!aBuilder.IsDone()
                || theCancellation->load(std::memory_order_relaxed)) {
                return Standard_False;
            }
            aShape = PlainBevelResultWithoutWrapper(aBuilder.Shape());
        }
        Standard_Size aNodes = theNodesBefore;
        Standard_Size aSolids = theSolidsBefore;
        return !aShape.IsNull() && !aShape.IsSame(theSource.shape)
            && IsValidSolidResult(aShape, theMaximumNodes, theMaximumSolids,
                   aNodes, aSolids);
    } catch (...) {
        return Standard_False;
    }
}

//! OCCT's chamfer builder, unlike the fillet builder, exposes no faulty
//! contour status. A plain chamfer that failed at theDistance is classified
//! as refused for its size only when the same edges chamfered at a quarter
//! of that distance build one valid solid: then a smaller value works and
//! the person can be told so. Anything else stays an ordinary failure.
//! K11 (build 7): the same quarter-size probe for a plain round, so a round
//! that is too large for a short step says so instead of "Preview failed".
//! K11 (CTO 19 Nov 12:35): the sentence names the size that fits. After the
//! quarter probe builds, six halvings between it (builds) and theDistance
//! (refused) find the largest tested value that builds; it is returned in
//! *theLargestFitting (stored units; 0 when none was found). The search runs
//! only when theLargestFitting is given (a round on one part), only after a
//! refusal, never on a preview that works, and is cancellable like a preview.
//! CTO 19 Nov 18:39 (Fable read of 5783a2a9): (b) the halvings stop once
//! about a second has passed since the refusal and keep the best size found
//! so far; (a) the size the HintLine shows is that size floored to
//! theNamedSizeStep (0.1 mm), so that floored size is built once and named
//! only if it builds; else one step down is tried (if time is left); else no
//! size is named. With no step known the tested size is returned as before.
BevelFailureKind ClassifyFailedPlainBevel(
    const BevelPreviewWorkerSource& theSource,
    const Standard_Real theDistance,
    const Standard_Boolean theIsChamfer,
    const std::shared_ptr<std::atomic_bool>& theCancellation,
    const Standard_Size theNodesBefore,
    const Standard_Size theSolidsBefore,
    const Standard_Size theMaximumNodes,
    const Standard_Size theMaximumSolids,
    Standard_Real* theLargestFitting,
    const Standard_Real theNamedSizeStep = 0.0) noexcept
{
    if (theLargestFitting != nullptr) {
        *theLargestFitting = 0.0;
    }
    const auto aDeadline = std::chrono::steady_clock::now()
        + std::chrono::milliseconds(1000);
    const auto aTimeLeft = [&aDeadline]() {
        return std::chrono::steady_clock::now() < aDeadline;
    };
    try {
        const Standard_Real aProbe = theDistance * 0.25;
        if (!PlainBevelBuildsAt(theSource, aProbe, theIsChamfer,
                theCancellation, theNodesBefore, theSolidsBefore,
                theMaximumNodes, theMaximumSolids)) {
            return BevelFailureKind::Other;
        }
        if (theLargestFitting == nullptr) {
            return BevelFailureKind::RadiusRefused;
        }
        Standard_Real aFits = aProbe;
        Standard_Real aRefused = theDistance;
        for (int aStep = 0; aStep < 6; ++aStep) {
            if (theCancellation == nullptr
                || theCancellation->load(std::memory_order_relaxed)
                || !aTimeLeft()) {
                break;
            }
            const Standard_Real aMiddle = 0.5 * (aFits + aRefused);
            if (PlainBevelBuildsAt(theSource, aMiddle, theIsChamfer,
                    theCancellation, theNodesBefore, theSolidsBefore,
                theMaximumNodes, theMaximumSolids)) {
                aFits = aMiddle;
            } else {
                aRefused = aMiddle;
            }
        }
        if (!(theNamedSizeStep > Precision::Confusion())
            || !std::isfinite(theNamedSizeStep)) {
            *theLargestFitting = aFits;
            return BevelFailureKind::RadiusRefused;
        }
        // The size as the HintLine shows it: aFits floored to whole steps.
        Standard_Real aShown =
            std::floor(aFits / theNamedSizeStep + 1.0e-6) * theNamedSizeStep;
        for (int aTry = 0; aTry < 2; ++aTry) {
            if (aShown < theNamedSizeStep * (1.0 - 1.0e-6)
                || theCancellation == nullptr
                || theCancellation->load(std::memory_order_relaxed)
                || (aTry > 0 && !aTimeLeft())) {
                break;
            }
            if (std::abs(aShown - aFits)
                    <= 1.0e-9 * std::max(Standard_Real(1.0), aFits)
                || PlainBevelBuildsAt(theSource, aShown, theIsChamfer,
                    theCancellation, theNodesBefore, theSolidsBefore,
                    theMaximumNodes, theMaximumSolids)) {
                *theLargestFitting = aShown;
                return BevelFailureKind::RadiusRefused;
            }
            aShown -= theNamedSizeStep;
        }
        return BevelFailureKind::RadiusRefused;
    } catch (...) {
        if (theLargestFitting != nullptr) {
            *theLargestFitting = 0.0;
        }
        return BevelFailureKind::Other;
    }
}

BevelFailureKind ClassifyFailedPlainChamfer(
    const BevelPreviewWorkerSource& theSource,
    const Standard_Real theDistance,
    const std::shared_ptr<std::atomic_bool>& theCancellation,
    const Standard_Size theNodesBefore,
    const Standard_Size theSolidsBefore,
    const Standard_Size theMaximumNodes,
    const Standard_Size theMaximumSolids) noexcept
{
    return ClassifyFailedPlainBevel(theSource, theDistance, Standard_True,
        theCancellation, theNodesBefore, theSolidsBefore, theMaximumNodes,
        theMaximumSolids, nullptr);
}

} // namespace

Standard_Boolean IsBevelEdgeEligible(
    const TopoDS_Edge& theEdge) noexcept
{
    if (theEdge.IsNull()) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        Standard_Real aFirst = 0.0;
        Standard_Real aLast = 0.0;
        const Handle(Geom_Curve) aCurve =
            BRep_Tool::Curve(theEdge, aFirst, aLast);
        if (aCurve.IsNull() || !std::isfinite(aFirst)
            || !std::isfinite(aLast)) {
            return Standard_False;
        }
        const GeomAdaptor_Curve anAdaptor(aCurve);
        const GeomAbs_CurveType aType = anAdaptor.GetType();
        return aType == GeomAbs_Line
            || aType == GeomAbs_BSplineCurve
            || anAdaptor.IsClosed();
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean TryResolveCanonicalBevelEdgeTopologyIndexBounded(
	const TopoDS_Shape& theShape,
	const Standard_Size theZeroBasedIndex,
	const Standard_Size theMaximumTopologyOccurrences,
	TopoDS_Edge& theEdge) noexcept
{
	theEdge.Nullify();
	if (theShape.IsNull() || theMaximumTopologyOccurrences == 0
		|| theZeroBasedIndex
			>= theMaximumTopologyOccurrences) {
		return Standard_False;
	}
	try {
		OCC_CATCH_SIGNALS
		Standard_Size anAggregateCount = 0;
		Standard_Size aShapeCount = 0;
		if (!AccumulateBoundedTopologyOccurrences(
				theShape,
				theMaximumTopologyOccurrences,
				theMaximumTopologyOccurrences,
				anAggregateCount,
				aShapeCount)) {
			return Standard_False;
		}
		TopTools_IndexedMapOfShape anEdges;
		TopExp::MapShapes(theShape, TopAbs_EDGE, anEdges);
		if (theZeroBasedIndex
			>= static_cast<Standard_Size>(anEdges.Extent())) {
			return Standard_False;
		}
		theEdge = TopoDS::Edge(anEdges.FindKey(
			static_cast<Standard_Integer>(theZeroBasedIndex + 1)));
		return !theEdge.IsNull();
	} catch (...) {
		theEdge.Nullify();
		return Standard_False;
	}
}

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

    Standard_Boolean isIdle() const noexcept
    {
        try {
            std::lock_guard<std::mutex> aLock(myMutex);
            return myActiveCancellation == nullptr
                && !myPending.has_value();
        } catch (...) {
            return Standard_False;
        }
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
            aResult.failureKind = BevelFailureKind::None;
            return aResult;
        }

        try {
            OCC_CATCH_SIGNALS
            Handle(BevelCancellationIndicator) aProgress =
                new BevelCancellationIndicator(theRequest.cancellation);
            aResult.results.reserve(theRequest.sources.size());
            aResult.retainedResults.reserve(theRequest.sources.size());
            const Standard_Boolean isChamfer = theRequest.value < 0.0;
            const Standard_Real aDistance = std::abs(theRequest.value);
            Standard_Size anAggregateResultTopologyNodes = 0;
            Standard_Size anAggregateResultSolids = 0;
            for (const BevelPreviewWorkerSource& aSource :
                 theRequest.sources) {
                if (wasCancelled() || aSource.shape.IsNull()
                    || aSource.edges.empty()) {
                    aResult.outcome = BevelPreviewWorkerOutcome::Cancelled;
                    aResult.failureKind = BevelFailureKind::None;
                    aResult.results.clear();
                    return aResult;
                }
                // The counts of the parts already built, before this one:
                // a size probe for this part starts from them (CTO 12:55).
                const Standard_Size aNodesBefore =
                    anAggregateResultTopologyNodes;
                const Standard_Size aSolidsBefore = anAggregateResultSolids;
                TopoDS_Shape aCandidate;
                std::shared_ptr<const retained_edge_treatment::DetachedResult>
                    aRetainedResult;
                if (aSource.retainedInput) {
                    retained_edge_treatment::Refusal aRefusal;
                    aRetainedResult = theRequest.retainedAdapter
                        .BuildRetainedBevelPreview(
                            aSource.retainedInput, aRefusal);
                    if (!aRetainedResult) {
                        aResult.failureKind = aRefusal
                                == retained_edge_treatment::Refusal::Clearance
                            ? BevelFailureKind::RadiusRefused
                            : BevelFailureKind::Other;
                        return aResult;
                    }
                    aCandidate = aRetainedResult->result();
                } else if (isChamfer) {
                    const ChamferInput aChamferInput =
                        ChamferInputWithoutFlatSeams(
                            aSource.shape, aSource.edges);
                    BRepFilletAPI_MakeChamfer aBuilder(aChamferInput.shape);
                    for (const TopoDS_Edge& anEdge : aChamferInput.edges) {
                        aBuilder.Add(aDistance, anEdge);
                    }
                    aBuilder.Build(aProgress->Start());
                    if (!aBuilder.IsDone()) {
                        aResult.failureKind = ClassifyFailedPlainChamfer(
                            aSource, aDistance, theRequest.cancellation,
                            aNodesBefore, aSolidsBefore,
                            theRequest.maximumResultTopologyNodes,
                            theRequest.maximumResultSolids);
                        return aResult;
                    }
                    aCandidate = PlainBevelResultWithoutWrapper(aBuilder.Shape());
                } else {
                    BRepFilletAPI_MakeFillet aBuilder(aSource.shape);
                    for (const TopoDS_Edge& anEdge : aSource.edges) {
                        aBuilder.Add(aDistance, anEdge);
                    }
                    aBuilder.Build(aProgress->Start());
                    if (!aBuilder.IsDone()) {
                        // K11 (CTO 19 Nov 12:35): a round on one part also
                        // finds the largest size that builds, for the hint.
                        Standard_Real* aLargest =
                            theRequest.sources.size() == 1
                                ? &aResult.largestFittingValue
                                : nullptr;
                        if (fillet_failure::IsClearanceRefusal(aBuilder)) {
                            if (aLargest != nullptr) {
                                ClassifyFailedPlainBevel(
                                    aSource, aDistance, Standard_False,
                                    theRequest.cancellation,
                                    aNodesBefore, aSolidsBefore,
                                    theRequest.maximumResultTopologyNodes,
                                    theRequest.maximumResultSolids,
                                    aLargest, theRequest.namedSizeStep);
                            }
                            aResult.failureKind =
                                BevelFailureKind::RadiusRefused;
                        } else {
                            aResult.failureKind = ClassifyFailedPlainBevel(
                                aSource, aDistance, Standard_False,
                                theRequest.cancellation,
                                aNodesBefore, aSolidsBefore,
                                theRequest.maximumResultTopologyNodes,
                                theRequest.maximumResultSolids,
                                aLargest, theRequest.namedSizeStep);
                        }
                        return aResult;
                    }
                    aCandidate = PlainBevelResultWithoutWrapper(aBuilder.Shape());
                }
                if (wasCancelled()) {
                    aResult.outcome = BevelPreviewWorkerOutcome::Cancelled;
                    aResult.failureKind = BevelFailureKind::None;
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
                    // A plain chamfer may "build" a broken solid when the
                    // distance runs past a neighbouring edge (phone stand:
                    // Bevel 2 on a 2 mm Push-pull step).
                    if (!aSource.retainedInput
                        && !aCandidate.IsNull()
                        && !aCandidate.IsSame(aSource.shape)) {
                        aResult.failureKind = ClassifyFailedPlainBevel(
                            aSource, aDistance, isChamfer,
                            theRequest.cancellation,
                            aNodesBefore, aSolidsBefore,
                            theRequest.maximumResultTopologyNodes,
                            theRequest.maximumResultSolids,
                            !isChamfer && theRequest.sources.size() == 1
                                ? &aResult.largestFittingValue
                                : nullptr,
                            theRequest.namedSizeStep);
                    }
                    return aResult;
                }
                aResult.results.push_back(std::move(aCandidate));
                aResult.retainedResults.push_back(
                    std::move(aRetainedResult));
            }
            if (aResult.results.size() != theRequest.sources.size()) {
                aResult.results.clear();
                aResult.retainedResults.clear();
                return aResult;
            }
            aResult.outcome = BevelPreviewWorkerOutcome::Success;
            aResult.failureKind = BevelFailureKind::None;
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

void BevelOperationController::setRetainedAdapter(
    RetainedBevelAdapter theAdapter) noexcept
{
    if (!hasActiveOperation()) {
        myRetainedAdapter = std::move(theAdapter);
    }
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

Standard_Boolean BevelOperationController::canBegin(
    const std::vector<BevelSourceSelection>& theSelection) const noexcept
{
    if (hasActiveOperation()
        && !canAtomicallyReplaceSelectingSources()) {
        return Standard_False;
    }
    std::vector<Source> aSources;
    return tryPrepareSources(theSelection, aSources);
}

Standard_Boolean BevelOperationController::canBeginRound(
    const std::vector<BevelSourceSelection>& theSelection) const noexcept
{
    if (theSelection.size() != 1) {
        return Standard_False;
    }
    std::vector<Source> aSources;
    return (!hasActiveOperation() || canAtomicallyReplaceSelectingSources())
        && tryPrepareSources(theSelection, aSources)
        && aSources.size() == 1;
}

Standard_Boolean BevelOperationController::tryPrepareSources(
    const std::vector<BevelSourceSelection>& theSelection,
    std::vector<Source>& theSources) const noexcept
{
    theSources.clear();
    if (myContext.IsNull() || myDoc.IsNull() || theSelection.empty()
        || theSelection.size() > kMaxSourceBodies) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(TDocStd_Document) aDocument = myDoc->Document();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()
            || aDocument->GetUndoLimit() == 0) {
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
            const Standard_Integer anObjectSelectionMode =
                AIS_Shape::SelectionMode(TopAbs_SHAPE);
            const Standard_Integer aFaceSelectionMode =
                AIS_Shape::SelectionMode(TopAbs_FACE);
            const Standard_Integer anEdgeSelectionMode =
                AIS_Shape::SelectionMode(TopAbs_EDGE);
            if (aSelection.original.IsNull()
                || aSelection.original->Shape().IsNull()
                || aSelection.documentLabel.IsNull()
                || aSelection.documentLabel.Data()
                    != aDocument->GetData()
                || aSelection.edges.empty()
                || aSelection.edges.size() > kMaxSelectedEdges
                || (!aSelection.edgeTopologyIndices.empty()
                    && aSelection.edgeTopologyIndices.size()
                        != aSelection.edges.size())
                || (aSelection.selectionMode != anObjectSelectionMode
                    && aSelection.selectionMode != aFaceSelectionMode
                    && aSelection.selectionMode != anEdgeSelectionMode)
                || !IsBRepModelingLabel(
                    myDoc, aSelection.documentLabel, Standard_True)
                || !myDoc->IsPresentationEditable(aSelection.original)
                || !myDoc->ShapeLabel(aSelection.original).IsEqual(
                    aSelection.documentLabel)
                || !myContext->IsDisplayed(aSelection.original)) {
                return Standard_False;
            }
            const TopoDS_Shape aPresentedShape = aSelection.original->Shape();
            // App Store 5.34.1 stored a Union/Subtract result as a compound
            // around its one solid: the Bevel is built on that solid.
            TopoDS_Shape aShape = aPresentedShape;
            while (!aShape.IsNull() && aShape.ShapeType() == TopAbs_COMPOUND) {
                TopoDS_Iterator anOnly(aShape);
                if (!anOnly.More()) break;
                const TopoDS_Shape aChild = anOnly.Value();
                anOnly.Next();
                if (anOnly.More()) break;
                aShape = aChild;
            }
            const TopoDS_Shape aStoredShape =
                XCAFDoc_ShapeTool::GetShape(aSelection.documentLabel);
            const gp_Trsf aCapturedTransform =
                aSelection.original->LocalTransformation();
            gp_Trsf aPersistedTransform;
            Standard_Size aSourceSolidCount = 0;
            Standard_Size aSourceTopologyNodeCount = 0;
            if (aStoredShape.IsNull() || !aStoredShape.IsEqual(aPresentedShape)
                || aShape.ShapeType() != TopAbs_SOLID
                || !myDoc->TryObjectTransformForLabel(
                    aSelection.documentLabel, aPersistedTransform)
                || !TransformsMatch(
                    aPersistedTransform, aCapturedTransform)
                || !AccumulateBoundedTopologyOccurrences(
                    aShape,
                    aPerSourceLimit,
                    anAggregateLimit,
                    anAggregateNodes,
                    aSourceTopologyNodeCount)
                || HasStyledXCAFSubshape(
                    aDocument,
                    aSelection.documentLabel,
                    kMaxStyledSubshapeLabels, Standard_True)
                || !HasOnlySolidBranches(
                    aShape,
                    aPerSourceLimit,
                    1,
                    aSourceSolidCount)
                || aSourceSolidCount != 1
                || !BRepCheck_Analyzer(
                    aShape, Standard_True).IsValid()) {
                return Standard_False;
            }
            TopTools_IndexedMapOfShape anEdges;
            TopExp::MapShapes(aShape, TopAbs_EDGE, anEdges);
            TopTools_IndexedMapOfShape aUniqueSelectedEdges;
            std::vector<TopoDS_Edge> aCanonicalEdges;
            std::vector<Standard_Size> aCanonicalEdgeIndices;
            aCanonicalEdges.reserve(aSelection.edges.size());
            aCanonicalEdgeIndices.reserve(aSelection.edges.size());
            const Standard_Boolean requireExactSelectedEdgeOrientation =
                aSelection.selectionMode == anEdgeSelectionMode;
            for (const TopoDS_Edge& anEdge : aSelection.edges) {
                const Standard_Integer anIndex =
                    anEdge.IsNull() ? 0 : anEdges.FindIndex(anEdge);
                if (anIndex <= 0) {
                    return Standard_False;
                }
                const TopoDS_Edge aCanonicalEdge =
                    TopoDS::Edge(anEdges.FindKey(anIndex));
                if (aCanonicalEdge.IsNull()
                    || (requireExactSelectedEdgeOrientation
                        && !aCanonicalEdge.IsEqual(anEdge))
                    || aUniqueSelectedEdges.Contains(aCanonicalEdge)
                    || !IsBevelEdgeEligible(aCanonicalEdge)) {
                    return Standard_False;
                }
                aUniqueSelectedEdges.Add(aCanonicalEdge);
                aCanonicalEdges.push_back(aCanonicalEdge);
                aCanonicalEdgeIndices.push_back(
                    static_cast<Standard_Size>(anIndex - 1));
            }
            if (anEdgeCount > kMaxSelectedEdges
                    - aCanonicalEdges.size()) {
                return Standard_False;
            }
            anEdgeCount += aCanonicalEdges.size();
            const std::string anIdentifier =
                myDoc->EntityIdentifierForLabel(aSelection.documentLabel);
            const std::string aDefinitionIdentifier =
                myDoc->DefinitionIdentifierForLabel(
                    aSelection.documentLabel);
            if (anIdentifier.empty() || aDefinitionIdentifier.empty()
                || !anIdentifiers.insert(anIdentifier).second) {
                return Standard_False;
            }
            Source aSource;
            aSource.original = aSelection.original;
            aSource.label = aSelection.documentLabel;
            aSource.shape = aShape;
            aSource.stored = aPresentedShape;
            aSource.edges = std::move(aCanonicalEdges);
            aSource.edgeTopologyIndices =
                std::move(aCanonicalEdgeIndices);
            const bool hasRetainedCapture =
                bool(aSelection.retainedCapture)
                && bool(aSelection.retainedCapture->snapshot);
            const auto coverage=myDoc->RetainedRecipeCoverageForLabel(aSelection.documentLabel);
            // A fresh Join/Cut result (its only record is the Boolean originals record) is
            // bevelled as the solid it is; the record goes in the Bevel command (editor lane
            // 15 Nov 01:35: no Bevel after Union).
            TDF_Label aBakedBooleanRecord;
            const bool bakesBooleanResult = !hasRetainedCapture
                && myDoc->BooleanResultRecordForBake(aSelection.documentLabel, aBakedBooleanRecord);
            // A standalone saved Hole is baked inside this tool's own Apply
            // command (PrepareBooleanResultForFaceEdit), exactly as Push-pull
            // does; Undo restores the resizable hole record.
            const Standard_Boolean bakesSavedCut = !hasRetainedCapture
                && !bakesBooleanResult
                && coverage == OcctRetainedRecipeCoverage::PresentOutsideP4Coverage
                && myDoc->CanBakeRecordForFaceEdit(aSelection.documentLabel);
            if ((hasRetainedCapture
                    && (!RetainedBevelCoverageMatches(
                            coverage, aSelection.retainedCapture)
                        || theSelection.size() != 1
                        || !myRetainedAdapter))
                || (!hasRetainedCapture && !bakesBooleanResult && !bakesSavedCut
                    && coverage != OcctRetainedRecipeCoverage::Absent)) {
                return Standard_False;
            }
            aSource.bakedBooleanRecord = aBakedBooleanRecord;
            aSource.retainedCapture=aSelection.retainedCapture;
            aSource.bakesSavedCut=bakesSavedCut;
            if (!aSelection.edgeTopologyIndices.empty()
                && aSelection.edgeTopologyIndices
                    != aSource.edgeTopologyIndices) {
                return Standard_False;
            }
            aSource.transform = aCapturedTransform;
            aSource.selectionMode = aSelection.selectionMode;
            aSource.entityIdentifier = anIdentifier;
            aSource.definitionIdentifier = aDefinitionIdentifier;
            aSource.topologyNodeCount = aSourceTopologyNodeCount;
            aSource.sourceEdgeCount =
                static_cast<Standard_Size>(anEdges.Extent());
            aSources.push_back(std::move(aSource));
        }
        theSources = std::move(aSources);
        return Standard_True;
    } catch (...) {
        theSources.clear();
        return Standard_False;
    }
}

Standard_Boolean BevelOperationController::preparedSourcesAreCurrent(
    const std::vector<Source>& theSources,
    const Standard_Boolean theRequireDisplayed) const noexcept
{
    if (myContext.IsNull() || myDoc.IsNull() || theSources.empty()
        || theSources.size() > kMaxSourceBodies) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(TDocStd_Document) aDocument = myDoc->Document();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()
            || aDocument->GetUndoLimit() == 0) {
            return Standard_False;
        }
        for (const Source& aSource : theSources) {
            const TopoDS_Shape aStoredShape =
                XCAFDoc_ShapeTool::GetShape(aSource.label);
            gp_Trsf aPersistedTransform;
            if (aSource.original.IsNull() || aSource.label.IsNull()
                || aSource.label.Data() != aDocument->GetData()
                || aSource.shape.IsNull() || aSource.edges.empty()
                || aSource.topologyNodeCount == 0
                || aStoredShape.IsNull()
                || !IsBRepModelingLabel(
                    myDoc, aSource.label, aSource.bakesSavedCut)
                || (aSource.bakesSavedCut
                    && !myDoc->CanBakeRecordForFaceEdit(aSource.label))
                || !myDoc->IsPresentationEditable(aSource.original)
                || !myDoc->ShapeLabel(aSource.original).IsEqual(
                    aSource.label)
                || (theRequireDisplayed
                    && !myContext->IsDisplayed(aSource.original))
                || !aStoredShape.IsEqual(aSource.stored)
                || !aSource.original->Shape().IsEqual(aSource.stored)
                || !myDoc->TryObjectTransformForLabel(
                    aSource.label, aPersistedTransform)
                || !TransformsMatch(
                    aSource.original->LocalTransformation(),
                    aSource.transform)
                || !TransformsMatch(
                    aPersistedTransform, aSource.transform)
                || HasStyledXCAFSubshape(
                    aDocument,
                    aSource.label,
                    kMaxStyledSubshapeLabels, Standard_True)
                || myDoc->EntityIdentifierForLabel(aSource.label)
                    != aSource.entityIdentifier
                || myDoc->DefinitionIdentifierForLabel(aSource.label)
                    != aSource.definitionIdentifier) {
                return Standard_False;
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean BevelOperationController::begin(
    const std::vector<BevelSourceSelection>& theSelection) noexcept
{
	const Standard_Boolean isReplacement = hasActiveOperation();
    if (isReplacement && !canAtomicallyReplaceSelectingSources()) {
        return Standard_False;
    }
    std::vector<Source> aSources;
    if (!tryPrepareSources(theSelection, aSources)) {
        return Standard_False;
    }
    if (!preparedSourcesAreCurrent(aSources, Standard_True)) {
        return Standard_False;
    }
	if (isReplacement) {
		// A selection tap may replace only the pristine zero-value Selecting
		// ledger. Revalidate after the potentially expensive source proof, then
		// swap atomically without cancelling, hiding, or redisplaying anything.
		if (!canAtomicallyReplaceSelectingSources()) {
			return Standard_False;
		}
		mySources.swap(aSources);
		notifyPreviewStateChanged();
		return Standard_True;
	}
    try {
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

Standard_Boolean BevelOperationController::beginRound(
    const std::vector<BevelSourceSelection>& theSelection) noexcept
{
    // begin() repeats the full bounded source proof immediately before it
    // creates or replaces the ledger. The explicit count keeps this entry
    // point from ever inheriting legacy Bevel's multi-body authority.
    if (theSelection.size() != 1 || !begin(theSelection)) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        if (mySources.size() != 1 || myDoc.IsNull()) {
            (void)cancel();
            return Standard_False;
        }
        const Handle(TDocStd_Document) aDocument = myDoc->Document();
        Standard_Real aDocumentMetersPerUnit = 0.001;
        const Standard_Boolean hasUnit = !aDocument.IsNull()
            && XCAFDoc_DocumentTool::GetLengthUnit(
                aDocument, aDocumentMetersPerUnit);
        const Standard_Real aScale =
            std::abs(mySources.front().transform.ScaleFactor());
        const Standard_Real anEffectiveMetersPerUnit =
            aDocumentMetersPerUnit * aScale;
        Standard_Real aMaximumRadiusMM = 20.0;
        if (mySources.front().retainedCapture) {
            aMaximumRadiusMM = std::min<Standard_Real>(
                aMaximumRadiusMM, 20.0 * aScale);
        }
        if ((!hasUnit && aDocumentMetersPerUnit != 0.001)
            || !std::isfinite(aDocumentMetersPerUnit)
            || aDocumentMetersPerUnit <= 0.0
            || !std::isfinite(aScale) || aScale <= 0.0
            || !std::isfinite(anEffectiveMetersPerUnit)
            || anEffectiveMetersPerUnit <= 0.0
            || !std::isfinite(aMaximumRadiusMM)
            || aMaximumRadiusMM <= 0.0) {
            (void)cancel();
            return Standard_False;
        }
        myRoundOwned = Standard_True;
        myRoundMetersPerUnit = anEffectiveMetersPerUnit;
        myRoundMaximumRadiusMM = aMaximumRadiusMM;
        return Standard_True;
    } catch (...) {
        (void)cancel();
        return Standard_False;
    }
}

Standard_Boolean BevelOperationController::sourcesAreCurrent() const noexcept
{
    return preparedSourcesAreCurrent(mySources, Standard_False);
}

Standard_Boolean
BevelOperationController::canAtomicallyReplaceSelectingSources() const noexcept
{
	return hasPristineSelectingLedger() && sourcesAreCurrent();
}

Standard_Boolean
BevelOperationController::hasPristineSelectingLedger() const noexcept
{
	return hasActiveOperation()
		&& myStateValid
		&& myState == BevelPreviewState::Selecting
		&& std::abs(myValue)
			<= std::numeric_limits<Standard_Real>::epsilon()
		&& !myCanApply
		&& !mySelectionFrozen
		&& myPreviewResults.empty()
		&& myRequestedFingerprint.empty()
		&& (myWorker == nullptr || myWorker->isIdle());
}

Standard_Boolean
BevelOperationController::retireIdleSelectingSelection() noexcept
{
	// No preview or worker owns presentation state in this structural ledger,
	// so stale source identity is safe to forget after a failed live recapture.
	if (!hasPristineSelectingLedger()) {
		return Standard_False;
	}
	clearState();
	notifyPreviewStateChanged();
	return Standard_True;
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
    myFailureObservation = {myGeneration, BevelFailureKind::None};
    if (myWorker != nullptr) {
        myWorker->cancelAll();
    }
    for (Source& aSource : mySources) {
        if (aSource.retainedPrepared) {
            myRetainedAdapter.CancelRetainedBevelPreview(
                aSource.retainedPrepared->work);
        }
        aSource.retainedPrepared.reset();
        aSource.retainedEdit.reset();
        aSource.retainedResult.reset();
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
        myFailureObservation = {myGeneration, BevelFailureKind::None};
        aRequest.value = myValue;
        aRequest.fingerprint = aFingerprint;
        aRequest.retainedAdapter = myRetainedAdapter;
        aRequest.cancellation = std::make_shared<std::atomic_bool>(false);
        {
            // K11 (CTO 19 Nov 18:39): 0.1 mm as the person sees the part.
            const Standard_Real aMetersPerUnit = bevelMetersPerUnit();
            const Standard_Real aStep = std::isfinite(aMetersPerUnit)
                    && aMetersPerUnit > 0.0
                ? 0.0001 / aMetersPerUnit : 0.0;
            aRequest.namedSizeStep = std::isfinite(aStep) && aStep > 0.0
                ? aStep : 0.0;
        }
#ifdef DEBUG
        aRequest.maximumResultTopologyNodes =
            myDebugMaximumResultTopologyNodes;
        aRequest.maximumResultSolids = myDebugMaximumResultSolids;
#endif
        aRequest.sources.reserve(mySources.size());
        for (Source& aSource : mySources) {
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
            if (aSource.retainedCapture) {
                retained_edge_treatment::Refusal aRefusal;
                aSource.retainedPrepared = myRetainedAdapter
                    .PrepareRetainedBevelPreview(
                        aSource.retainedCapture, myValue, aRefusal);
                if (!aSource.retainedPrepared) {
                    return Standard_False;
                }
                aSource.retainedEdit =
                    aSource.retainedPrepared->edit;
                aSource.retainedResult.reset();
                aWorkerSource.retainedInput =
                    aSource.retainedPrepared->input;
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
    return setValueImpl(theSignedDistance, Standard_False);
}

Standard_Boolean BevelOperationController::setRoundRadius(
    const Standard_Real theRadius) noexcept
{
    if (!myRoundOwned || !hasActiveOperation()
        || !std::isfinite(theRadius) || theRadius <= 0.0
        || !std::isfinite(myRoundMetersPerUnit)
        || myRoundMetersPerUnit <= 0.0
        || !std::isfinite(myRoundMaximumRadiusMM)
        || myRoundMaximumRadiusMM <= 0.0) {
        return Standard_False;
    }
    const Standard_Real aRadiusMM =
        theRadius * myRoundMetersPerUnit * 1000.0;
    if (!std::isfinite(aRadiusMM) || aRadiusMM <= 0.0
        || aRadiusMM > myRoundMaximumRadiusMM) {
        return Standard_False;
    }
    return setValueImpl(theRadius, Standard_True);
}

Standard_Real BevelOperationController::roundMetersPerUnit() const noexcept
{
    return myRoundOwned && hasActiveOperation()
        ? myRoundMetersPerUnit : 0.0;
}

Standard_Real BevelOperationController::roundMaximumRadiusMM() const noexcept
{
    return myRoundOwned && hasActiveOperation()
        ? myRoundMaximumRadiusMM : 0.0;
}

Standard_Real BevelOperationController::bevelMetersPerUnit() const noexcept
{
    try {
        if (!hasActiveOperation() || mySources.empty() || myDoc.IsNull()) {
            return 0.0;
        }
        const Handle(TDocStd_Document) aDocument = myDoc->Document();
        Standard_Real aDocumentMetersPerUnit = 0.001;
        const Standard_Boolean hasUnit = !aDocument.IsNull()
            && XCAFDoc_DocumentTool::GetLengthUnit(
                aDocument, aDocumentMetersPerUnit);
        if ((!hasUnit && aDocumentMetersPerUnit != 0.001)
            || !std::isfinite(aDocumentMetersPerUnit)
            || aDocumentMetersPerUnit <= 0.0) {
            return 0.0;
        }
        const Standard_Real aScale =
            std::abs(mySources.front().transform.ScaleFactor());
        if (!std::isfinite(aScale) || aScale <= 0.0) {
            return 0.0;
        }
        for (const Source& aSource : mySources) {
            const Standard_Real anOtherScale =
                std::abs(aSource.transform.ScaleFactor());
            if (!std::isfinite(anOtherScale)
                || std::abs(anOtherScale - aScale) > 1.0e-9 * aScale) {
                return 0.0;
            }
        }
        const Standard_Real aMetersPerUnit = aDocumentMetersPerUnit * aScale;
        return std::isfinite(aMetersPerUnit) && aMetersPerUnit > 0.0
            ? aMetersPerUnit : 0.0;
    } catch (...) {
        return 0.0;
    }
}

Standard_Boolean BevelOperationController::setValueImpl(
    const Standard_Real theSignedDistance,
    const Standard_Boolean theRefuseOutOfRange) noexcept
{
    if (!hasActiveOperation() || !std::isfinite(theSignedDistance)) {
        return Standard_False;
    }
    if (theRefuseOutOfRange
        && theSignedDistance <= 0.0) {
        return Standard_False;
    }
    // The cap is in the part's own units. On a part made smaller (scale 0.64)
    // 20 units are only 12.8 mm, so a typed 15 mm Bevel was silently cut down
    // (desk lamp K14: a 6.4 mm top instead of 2 mm). Never cap below the 15 mm
    // the editor offers as the user sees the part (getChamferBoundaries).
    Standard_Real aLimit = kMaxDistance;
    const Standard_Real aMetersPerUnit = bevelMetersPerUnit();
    if (std::isfinite(aMetersPerUnit) && aMetersPerUnit > 0.0) {
        const Standard_Real aFifteenMillimetres = 0.015 / aMetersPerUnit;
        if (std::isfinite(aFifteenMillimetres))
            aLimit = std::max(aLimit, aFifteenMillimetres);
    }
    const Standard_Real aValue = theRefuseOutOfRange
        ? theSignedDistance
        : std::max(-aLimit, std::min(theSignedDistance, aLimit));
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
        recordFailure(BevelFailureKind::Other);
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
        recordFailure(BevelFailureKind::Other);
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
    if (theResult.outcome != BevelPreviewWorkerOutcome::Success) {
        myCanApply = Standard_False;
        myState = BevelPreviewState::Failed;
        recordFailure(theResult.failureKind,
            theResult.failureKind == BevelFailureKind::RadiusRefused
                ? theResult.largestFittingValue
                : 0.0);
        notifyPreviewStateChanged();
        return;
    }
    if (theResult.results.size() != mySources.size()
        || theResult.retainedResults.size() != mySources.size()
        || !sourcesAreCurrent()
        || !installPreview(theResult.results)) {
        myCanApply = Standard_False;
        myState = BevelPreviewState::Failed;
        recordFailure(BevelFailureKind::Other);
        notifyPreviewStateChanged();
        return;
    }
    for (Standard_Size anIndex = 0;
         anIndex < mySources.size(); ++anIndex) {
        Source& aSource = mySources[anIndex];
        const auto& aRetainedResult =
            theResult.retainedResults[anIndex];
        if (aSource.retainedCapture) {
            if (!aRetainedResult
                || !aRetainedResult->result().IsEqual(
                    theResult.results[anIndex])) {
                (void)discardPreview(Standard_True);
                myCanApply = Standard_False;
                myState = BevelPreviewState::Failed;
                recordFailure(BevelFailureKind::Other);
                notifyPreviewStateChanged();
                return;
            }
            aSource.retainedResult = aRetainedResult;
        } else if (aRetainedResult) {
            (void)discardPreview(Standard_True);
            myCanApply = Standard_False;
            myState = BevelPreviewState::Failed;
            recordFailure(BevelFailureKind::Other);
            notifyPreviewStateChanged();
            return;
        }
    }
    myCanApply = Standard_True;
    myState = BevelPreviewState::Ready;
    mySelectionFrozen = Standard_True;
    myFailureObservation = {myGeneration, BevelFailureKind::None};
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

Standard_Size
BevelOperationController::maximumCaptureTopologyNodes() const noexcept
{
#ifdef DEBUG
	return myDebugMaximumCaptureTopologyNodes;
#else
	return kMaxCaptureTopologyNodes;
#endif
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

BevelFailureObservation
BevelOperationController::failureObservation() const noexcept
{
    // Round owns the full classification. A plain Bevel reports only the
    // bounded size refusal (ClassifyFailedPlainChamfer) so the screen can say
    // the amount is too large; every other Bevel failure stays unclassified.
    if (myRoundOwned
        || myFailureObservation.kind == BevelFailureKind::RadiusRefused) {
        return myFailureObservation;
    }
    return BevelFailureObservation{myGeneration, BevelFailureKind::None};
}

void BevelOperationController::recordFailure(
    const BevelFailureKind theKind,
    const Standard_Real theLargestFittingValue) noexcept
{
    myFailureObservation = {
        myGeneration,
        theKind,
        theLargestFittingValue > 0.0 ? theLargestFittingValue : 0.0,
    };
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
	// This is the final read-only gate before NewCommand. A legacy BRep-only
	// source (Absent) continues on the unchanged path. A well-formed, current
	// plain profile enters the bounded compatibility operation: the
	// document-side capture proves the exact recipe record, identities,
	// placement, units, and an independent rebuild's correspondence to the
	// captured root. Every other retained coverage keeps the P4 refusal, and
	// malformed metadata still refuses rather than being treated as a
	// BRep-only legacy source.
	std::vector<OcctPlainProfileOperationCapture> aPlainProfileCaptures;
	try {
		aPlainProfileCaptures.reserve(mySources.size());
		for (const Source& source : mySources) {
			const OcctRetainedRecipeCoverage aCoverage =
				myDoc->RetainedRecipeCoverageForLabel(source.label);
			if (source.retainedCapture) {
				if (!RetainedBevelCoverageMatches(
						aCoverage, source.retainedCapture)
					|| !source.retainedCapture->snapshot
					|| !source.retainedEdit
					|| !source.retainedResult) {
					myCanApply = Standard_False;
					myState = BevelPreviewState::Failed;
					notifyPreviewStateChanged();
					return BevelApplyResult::NoChange;
				}
				continue;
			}
			if (aCoverage == OcctRetainedRecipeCoverage::Absent) {
				continue;
			}
			// A fresh Join/Cut result admitted at begin: its record is baked in the command.
			TDF_Label aCurrentRecord;
			if ((!source.bakedBooleanRecord.IsNull()
					&& myDoc->BooleanResultRecordForBake(source.label, aCurrentRecord)
					&& aCurrentRecord.IsEqual(source.bakedBooleanRecord))
				|| (source.bakesSavedCut
					&& aCoverage
						== OcctRetainedRecipeCoverage::PresentOutsideP4Coverage
					&& myDoc->CanBakeRecordForFaceEdit(source.label))) {
				continue;
			}
			OcctPlainProfileOperationCapture aCapture;
			if (aCoverage != OcctRetainedRecipeCoverage::CurrentProfile
				|| !myDoc->CapturePlainProfileOperationSource(
					source.label, aCapture)) {
				myCanApply = Standard_False;
				myState = BevelPreviewState::Failed;
				notifyPreviewStateChanged();
				return BevelApplyResult::NoChange;
			}
			aPlainProfileCaptures.push_back(std::move(aCapture));
		}
	} catch (...) {
		myCanApply = Standard_False;
		myState = BevelPreviewState::Failed;
		notifyPreviewStateChanged();
		return BevelApplyResult::NoChange;
	}
	// D81 (R4, cloud8182): the builders may return the measured result as a
	// forward COMPOUND wrapping its sole solid, while
	// VerifyPlainProfileOperationReplacement requires the replacement root
	// itself to be a SOLID. For every captured plain-profile source, prove the
	// one canonical solid now — before any preview shape is touched and before
	// the final recapture and the single NewCommand — and adopt that same
	// operation-owned shape as the preview/replacement consumed by
	// ReplaceShape, the unchanged document verifier, the post-close comparison
	// and the successful presentation adoption. Absent (legacy BRep-only)
	// sources keep their candidate unchanged. A failed proof or adoption
	// produces no command and no history and does not clear the operation
	// ledger.
	if (!aPlainProfileCaptures.empty()) {
#ifdef DEBUG
		const Standard_Size aCandidateNodeBudget =
			myDebugMaximumResultTopologyNodes;
#else
		const Standard_Size aCandidateNodeBudget =
			kMaxResultTopologyNodes;
#endif
		// Phase 1: pure proof, no preview or document side effects.
		std::vector<TopoDS_Shape> aCanonicalCandidates;
		std::vector<Standard_Boolean> aNeedsCanonicalReplacement;
		Standard_Boolean aCandidatesProved = Standard_True;
		try {
			OCC_CATCH_SIGNALS
			aCanonicalCandidates.resize(mySources.size());
			aNeedsCanonicalReplacement.assign(
				mySources.size(), Standard_False);
			for (const OcctPlainProfileOperationCapture& aCapture :
				 aPlainProfileCaptures) {
				Standard_Size anIndex = mySources.size();
				for (Standard_Size aSourceIndex = 0;
					 aSourceIndex < mySources.size(); ++aSourceIndex) {
					if (mySources[aSourceIndex].label.IsEqual(
							aCapture.label)) {
						anIndex = aSourceIndex;
						break;
					}
				}
				if (anIndex == mySources.size()
					|| myPreviewResults[anIndex].IsNull()
					|| !CanonicalPlainProfileBevelCandidate(
						myPreviewResults[anIndex]->Shape(),
						aCandidateNodeBudget,
						aCanonicalCandidates[anIndex])) {
					aCandidatesProved = Standard_False;
					break;
				}
				aNeedsCanonicalReplacement[anIndex] =
					!aCanonicalCandidates[anIndex].IsSame(
						myPreviewResults[anIndex]->Shape());
			}
		} catch (...) {
			aCandidatesProved = Standard_False;
		}
		if (!aCandidatesProved) {
			myCanApply = Standard_False;
			myState = BevelPreviewState::Failed;
			notifyPreviewStateChanged();
			return BevelApplyResult::NoChange;
		}
		// Phase 2: build every replacement presentation before changing any
		// preview shape; each keeps the source's saved transform and material.
		std::vector<Handle(AIS_Shape)> aCanonicalPresentations;
		Standard_Boolean aPresentationsPrepared = Standard_True;
		try {
			OCC_CATCH_SIGNALS
			aCanonicalPresentations.resize(mySources.size());
			for (Standard_Size anIndex = 0;
				 anIndex < mySources.size(); ++anIndex) {
				if (!aNeedsCanonicalReplacement[anIndex]) {
					continue;
				}
				Handle(AIS_Shape) aPresentation =
					new AIS_Shape(aCanonicalCandidates[anIndex]);
				aPresentation->SetLocalTransformation(
					mySources[anIndex].transform);
				myDoc->LoadObjectMeterial(
					mySources[anIndex].label,
					aPresentation);
				aCanonicalPresentations[anIndex] =
					std::move(aPresentation);
			}
		} catch (...) {
			aPresentationsPrepared = Standard_False;
		}
		if (!aPresentationsPrepared) {
			myCanApply = Standard_False;
			myState = BevelPreviewState::Failed;
			notifyPreviewStateChanged();
			return BevelApplyResult::NoChange;
		}
		// Phase 3: adopt the canonical solids as this operation's preview
		// shapes, replacing the old presentations in the context.
		Standard_Boolean aCanonicalAdopted = Standard_True;
		try {
			OCC_CATCH_SIGNALS
			for (Standard_Size anIndex = 0;
				 anIndex < mySources.size(); ++anIndex) {
				if (!aNeedsCanonicalReplacement[anIndex]) {
					continue;
				}
				myContext->Remove(
					myPreviewResults[anIndex], Standard_False);
				myPreviewResults[anIndex] =
					aCanonicalPresentations[anIndex];
				myContext->Display(
					myPreviewResults[anIndex],
					AIS_Shaded,
					mySources[anIndex].selectionMode,
					Standard_False);
				myContext->Deactivate(myPreviewResults[anIndex]);
			}
			myContext->UpdateCurrentViewer();
		} catch (...) {
			aCanonicalAdopted = Standard_False;
		}
		if (!aCanonicalAdopted) {
			// Mirror installPreview recovery: drop every preview and restore
			// the source presentations; the operation ledger stays intact.
			for (const Handle(AIS_Shape)& aPresentation :
				 myPreviewResults) {
				try {
					myContext->Remove(
						aPresentation, Standard_False);
				} catch (...) {
				}
			}
			myPreviewResults.clear();
			(void)discardPreview(Standard_True);
			myCanApply = Standard_False;
			myState = BevelPreviewState::Failed;
			notifyPreviewStateChanged();
			return BevelApplyResult::NoChange;
		}
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
		// Recapture immediately before staging: no serialized or transient
		// state may mint authority between the read-only gate and this one
		// measured command.
		for (const OcctPlainProfileOperationCapture& aCapture :
			 aPlainProfileCaptures) {
			if (!myDoc->PlainProfileOperationSourceCurrent(aCapture)) {
				myState = BevelPreviewState::Failed;
				notifyPreviewStateChanged();
				return BevelApplyResult::NoChange;
			}
		}
        // Establish creator ownership before opening the command. Because this
        // method is main-thread serialized, any companion staged after this
        // clean boundary is evidence from this operation, not merely a record
        // observed at a coincident Undo depth.
        if (myDoc->HasUnresolvedTreatmentHistoryCompanion()) {
            myStateValid = Standard_False;
            myState = BevelPreviewState::Failed;
            notifyPreviewStateChanged();
            return BevelApplyResult::NoChange;
        }
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
            const auto& source=mySources[anIndex];
            Standard_Boolean staged=Standard_False;
            if(source.retainedCapture){
                core3d::retained_edge_treatment::Record readback;
                core3d::retained_edge_treatment::Refusal refusal;
                staged=source.retainedEdit&&source.retainedResult
                    &&source.retainedResult->result().IsEqual(myPreviewResults[anIndex]->Shape())
                    &&source.retainedCapture->snapshot
                    &&myDoc->StageRetainedEdgeTreatment(*source.retainedCapture->snapshot,
                        *source.retainedEdit,*source.retainedResult,readback,refusal);
            }else{
                TDF_Label record=source.bakedBooleanRecord;
                if(!record.IsNull()) record.ForgetAllAttributes(Standard_True);
                // Colours put on single faces stay on the faces they were on (founder test 14 Nov).
                // A standalone saved Hole is baked first, in this same command (push-pull lane).
                std::vector<TDF_Label> faceLabels;
                std::vector<std::pair<TopoDS_Face, XCAFDoc_VisMaterialPBR>> colouredFaces;
                staged=(!source.bakesSavedCut
                        ||myDoc->PrepareBooleanResultForFaceEdit(source.label))
                    &&myDoc->CaptureFaceColours(source.label,1.0,faceLabels,colouredFaces)
                    &&myDoc->ReplaceShape(source.label,myPreviewResults[anIndex])
                    &&myDoc->CarryFaceColours(source.label,myPreviewResults[anIndex]->Shape(),
                        faceLabels,colouredFaces);
            }
            if (!staged) {
                // Discharge this command's companion only after verified
                // prior-state settlement. A failed proof remains recovery
                // ownership in OcctDocument and invalidates this ledger.
                if (!abortOpenCommand(aDocument)
                    || !myDoc->SettleTreatmentHistoryCompanionOnPrior())
                    myStateValid = Standard_False;
                myState = BevelPreviewState::Failed;
                notifyPreviewStateChanged();
                return BevelApplyResult::NoChange;
            }
			const OcctPlainProfileOperationCapture* aCapture = nullptr;
			for (const OcctPlainProfileOperationCapture& aCandidate :
				 aPlainProfileCaptures) {
				if (aCandidate.label.IsEqual(mySources[anIndex].label)) {
					aCapture = &aCandidate;
					break;
				}
			}
			// Transactional retention proof, still inside the open command:
			// the recipe record, identifier, scalar bytes and bound shape
			// must survive the root replacement as a stale record. A failed
			// proof aborts without history.
			if (aCapture != nullptr
				&& !myDoc->VerifyPlainProfileOperationReplacement(
					*aCapture, myPreviewResults[anIndex]->Shape())) {
				if (!abortOpenCommand(aDocument)
					|| !myDoc->SettleTreatmentHistoryCompanionOnPrior())
					myStateValid = Standard_False;
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
            if (!abortOpenCommand(aDocument)
                || !myDoc->SettleTreatmentHistoryCompanionOnPrior())
                myStateValid = Standard_False;
            myState = BevelPreviewState::Failed;
            notifyPreviewStateChanged();
            return BevelApplyResult::NoChange;
        }
    } catch (...) {
        if (!abortOpenCommand(aDocument)
            || !myDoc->SettleTreatmentHistoryCompanionOnPrior())
            myStateValid = Standard_False;
        myState = BevelPreviewState::Failed;
        notifyPreviewStateChanged();
        return BevelApplyResult::NoChange;
    }

    // D253/CLOUD-8242: the retained staging captured a measured history
    // companion for this command; bind it to the actual closed delta before
    // publishing success or releasing controller state. A failed binding
    // retains recovery ownership: the companion stays unresolved, the viewer
    // barrier keeps blocking normal edits and snapshots, and no successful
    // Apply is reported over unmeasured history.
    if (!myDoc->FinalizeTreatmentHistoryCompanion()) {
        myStateValid = Standard_False;
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
        recordFailure(BevelFailureKind::Other);
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
    myRoundOwned = Standard_False;
    myRoundMetersPerUnit = 0.0;
    myRoundMaximumRadiusMM = 0.0;
    myFailureObservation = {myGeneration, BevelFailureKind::None};
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
            aState.sourceEdgeCount = mySources.front().sourceEdgeCount;
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

void BevelOperationController::debugSetPlainResultKeepsBuilderWrapper(
    const Standard_Boolean keeps) noexcept
{
    gDebugPlainResultKeepsBuilderWrapper.store(keeps == Standard_True);
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
