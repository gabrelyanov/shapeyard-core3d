#pragma once

// M14-0 (25 Nov 2026, build 12): Bevel 2 mm on a cube with a 2 mm Shell closed
// the app. OCCT's fillet builder was handed a round that reaches the far
// border of a face next to a selected edge (the 2 mm rim), and it read through
// a null edge (ChFi3d_Builder::StartSol > BRep_Tool::CurveOnSurface, address
// 0x60). That is a segmentation fault, not an exception: no try / catch and no
// OCC_CATCH_SIGNALS in this project can stop it (OCC_CONVERT_SIGNALS is not
// defined and OSD::SetSignal is never called), so the amount is refused here,
// before the builder is called.
//
// The rule is the Ramp tool's K19 rule (variable_radius_fillet::
// supportFaceClearance / continuousClearance), without its limit to flat faces
// and straight edges: a blend of size r on an edge leaves each of its two
// faces r * tan(turn / 2) away from the edge (turn = the angle between the two
// faces' normals at the edge; a box edge: r itself; a flat Bevel: its distance
// itself). The face must be wider than that: wider than the distance from the
// edge to every other sharp border of the same face that does not touch it,
// less what that border's own blend takes when it is selected too.
//
// (a2, 25 Nov, engine review 24 follow-up 1a) On a flat face the distance is
// measured from the strip the blend covers, not from the edge: from the edge
// to where the blend LANDS, the edge moved by its reach into the face (a
// concentric circle for a circle), ending where it meets the landing line of
// a bevelled neighbour, or the neighbour itself when that is not bevelled, or
// square to the edge where the outline carries on tangentially. Corner to
// corner was the wrong width: it refused Object Bevel 4 on a cube whose
// upright edges are rounded 5 (and hexagon, star and flat-bevelled outlines)
// although the kernel builds it. A landing that is used up (a rounded
// corner's arc, a short edge between two bevelled ones) refuses, and so does
// a far border inside the strip. Where the strip cannot be worked out exactly
// (a face that is not flat, a curve that is not a line or a circle, a curved
// neighbour at a sharp corner) the f842bf14 measure from the edge is kept.
// The edges OCCT adds to the selection by itself (every sharp edge that
// carries a selected one on tangentially) are measured too.
//
// Deliberately narrow (the few shapes a2 refuses that f842bf14 let through are
// listed in FABLE-BEVEL-SHELL-CRASH-20261125.md, section "(a2) corners"):
//  - only borders that do not touch the edge are measured (a neighbour at a
//    corner bounds the ends of a blend, not its width; (a2) it sets where the
//    strip ends);
//  - a border where the surface runs on smoothly into another flat face (a
//    flat seam a Join left) is not a border: the builder may roll over it;
//    (a2) a smooth border into a round is (kRoundBeyondASmoothBorderIsAFarBorder);
//  - anything that cannot be measured is left to the builder, as before.
//    (a2) Except the guard's own limits: past the measuring budget, a distance
//    OCCT cannot give, or an exception, the amount is refused.
// It does not prove that a blend builds; the builder and every check after it
// still judge the candidate.

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <ElCLib.hxx>
#include <GeomLib_IsPlanarSurface.hxx>
#include <Geom_Surface.hxx>
#include <Geom2d_Curve.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Circ.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace core3d::fillet_reach {

constexpr double kPi = 3.14159265358979323846;
//! Half a degree, as IsSmoothBevelEdge: below it two faces meet smoothly.
constexpr double kSmoothTurn = 0.5 * kPi / 180.0;
//! A blend must leave at least this share of the face's width standing, so a
//! size probe never asks the builder for a sliver either.
constexpr double kMargin = 0.01;
//! Exact edge-to-edge distances measured per call; (a2) past it the amount is
//! refused (it was left to the builder).
constexpr int kMaximumMeasuredPairs = 1024;
//! (a2) Edges measured after adding the tangent chains; past it, refused.
constexpr int kMaximumChainEdges = 1024;
//! (a2, review 24 open point ii) A smooth border into a round (the side of an
//! earlier Bevel) is measured like a far sharp border that is not bevelled: a
//! round on the outer rim of a cup whose inner rim is already round cannot run
//! on over the round that falls away beside it, the dead end of M14-0. false =
//! f842bf14: such a border is never measured.
constexpr bool kRoundBeyondASmoothBorderIsAFarBorder = true;

namespace detail {

//! The two different faces that meet at theEdge. False for a seam that closes
//! one face on itself and for an edge the map does not hold.
inline bool FacesAt(
    const TopTools_IndexedDataMapOfShapeListOfShape& theFacesOfEdge,
    const TopoDS_Edge& theEdge,
    TopoDS_Face& theFirst,
    TopoDS_Face& theSecond)
{
    const Standard_Integer anIndex = theFacesOfEdge.FindIndex(theEdge);
    if (anIndex <= 0) {
        return false;
    }
    const TopTools_ListOfShape& aFaces = theFacesOfEdge.FindFromIndex(anIndex);
    if (aFaces.Extent() != 2 || aFaces.First().IsSame(aFaces.Last())) {
        return false;
    }
    theFirst = TopoDS::Face(aFaces.First());
    theSecond = TopoDS::Face(aFaces.Last());
    return true;
}

//! The face's outward normal at the middle of theEdge.
inline bool NormalAt(
    const TopoDS_Edge& theEdge,
    const TopoDS_Face& theFace,
    gp_Vec& theNormal)
{
    Standard_Real aFirst = 0.0;
    Standard_Real aLast = 0.0;
    const Handle(Geom2d_Curve) aCurve =
        BRep_Tool::CurveOnSurface(theEdge, theFace, aFirst, aLast);
    if (aCurve.IsNull() || !std::isfinite(aFirst) || !std::isfinite(aLast)
        || !(aLast > aFirst)) {
        return false;
    }
    const gp_Pnt2d aUV = aCurve->Value(0.5 * (aFirst + aLast));
    const BRepAdaptor_Surface aSurface(theFace, Standard_False);
    gp_Pnt aPoint;
    gp_Vec aDU;
    gp_Vec aDV;
    aSurface.D1(aUV.X(), aUV.Y(), aPoint, aDU, aDV);
    theNormal = aDU.Crossed(aDV);
    if (!(theNormal.Magnitude() > 1.0e-12)) {
        return false;
    }
    if (theFace.Orientation() == TopAbs_REVERSED) {
        theNormal.Reverse();
    }
    return true;
}

//! The angle the surface turns by across theEdge: 0 where two faces run on
//! smoothly, 90 degrees on a box edge, inside or outside. False when unknown.
inline bool TurnAt(
    const TopTools_IndexedDataMapOfShapeListOfShape& theFacesOfEdge,
    const TopoDS_Edge& theEdge,
    double& theTurn)
{
    TopoDS_Face aFirst;
    TopoDS_Face aSecond;
    gp_Vec aFirstNormal;
    gp_Vec aSecondNormal;
    if (!FacesAt(theFacesOfEdge, theEdge, aFirst, aSecond)
        || !NormalAt(theEdge, aFirst, aFirstNormal)
        || !NormalAt(theEdge, aSecond, aSecondNormal)) {
        return false;
    }
    theTurn = aFirstNormal.Angle(aSecondNormal);
    return std::isfinite(theTurn);
}

//! How far from its edge a blend of theAmount leaves each of the two faces.
inline double Reach(
    const double theAmount,
    const double theTurn,
    const bool theIsChamfer) noexcept
{
    return theIsChamfer ? theAmount : theAmount * std::tan(0.5 * theTurn);
}

//! (a2) Unit tangent of theEdge at its end theVertex, pointing from the vertex
//! into the edge. False for a closed edge (its one vertex is both ends).
inline bool TangentFrom(
    const TopoDS_Edge& theEdge,
    const TopoDS_Vertex& theVertex,
    gp_Vec& theTangent) noexcept
{
    try {
        if (TopExp::FirstVertex(theEdge).IsSame(TopExp::LastVertex(theEdge))) {
            return false;
        }
        const BRepAdaptor_Curve aCurve(theEdge);
        const Standard_Real aParameter = BRep_Tool::Parameter(theVertex, theEdge);
        gp_Pnt aPoint;
        aCurve.D1(aParameter, aPoint, theTangent);
        if (!(theTangent.Magnitude() > 1.0e-12)) {
            return false;
        }
        theTangent.Normalize();
        if (std::abs(aParameter - aCurve.LastParameter())
            < std::abs(aParameter - aCurve.FirstParameter())) {
            theTangent.Reverse();
        }
        return true;
    } catch (...) {
        return false;
    }
}

//! (a2) OCCT's fillet and chamfer builders blend every sharp edge that carries
//! a selected one on tangentially as one contour (BRepFilletAPI_MakeFillet::Add
//! takes the whole chain), so those edges join theBlend: the corner arcs of a
//! rounded box are measured with the straight edge picked beside them. False
//! past kMaximumChainEdges.
template <typename SharpTurnOf>
inline bool AddTangentChains(
    const TopoDS_Shape& theShape,
    const SharpTurnOf& theSharpTurnOf,
    TopTools_IndexedMapOfShape& theBlend)
{
    TopTools_IndexedDataMapOfShapeListOfShape anEdgesOfVertex;
    TopExp::MapShapesAndUniqueAncestors(
        theShape, TopAbs_VERTEX, TopAbs_EDGE, anEdgesOfVertex);
    for (Standard_Integer anIndex = 1; anIndex <= theBlend.Extent(); ++anIndex) {
        if (theBlend.Extent() > kMaximumChainEdges) {
            return false;
        }
        const TopoDS_Edge anEdge = TopoDS::Edge(theBlend(anIndex));
        for (TopExp_Explorer anEnd(anEdge, TopAbs_VERTEX); anEnd.More(); anEnd.Next()) {
            const TopoDS_Vertex aVertex = TopoDS::Vertex(anEnd.Current());
            gp_Vec anInto;
            if (!anEdgesOfVertex.Contains(aVertex)
                || !TangentFrom(anEdge, aVertex, anInto)) {
                continue;
            }
            for (TopTools_ListIteratorOfListOfShape aNext(
                     anEdgesOfVertex.FindFromKey(aVertex));
                 aNext.More(); aNext.Next()) {
                const TopoDS_Edge aCandidate = TopoDS::Edge(aNext.Value());
                double aTurn = 0.0;
                gp_Vec aNextInto;
                if (theBlend.Contains(aCandidate)
                    || BRep_Tool::Degenerated(aCandidate)
                    || !theSharpTurnOf(aCandidate, aTurn)
                    || !TangentFrom(aCandidate, aVertex, aNextInto)) {
                    continue;
                }
                // It carries on where this one arrives: opposite directions.
                if (anInto.Angle(aNextInto) > kPi - kSmoothTurn) {
                    theBlend.Add(aCandidate);
                }
            }
        }
    }
    return theBlend.Extent() <= kMaximumChainEdges;
}

//! (a2, ii) theBorder is a smooth edge of theFace into a face that is not flat:
//! the side of an earlier round.
inline bool LeadsIntoARound(
    const TopTools_IndexedDataMapOfShapeListOfShape& theFacesOfEdge,
    const TopoDS_Edge& theBorder,
    const TopoDS_Face& theFace)
{
    TopoDS_Face aFirst;
    TopoDS_Face aSecond;
    if (!FacesAt(theFacesOfEdge, theBorder, aFirst, aSecond)) {
        return false;
    }
    const TopoDS_Face& aBeyond = aFirst.IsSame(theFace) ? aSecond : aFirst;
    const Handle(Geom_Surface) aSurface = BRep_Tool::Surface(aBeyond);
    return aSurface.IsNull()
        || !GeomLib_IsPlanarSurface(aSurface, Precision::Confusion()).IsPlanar();
}


//! (a2) The strip a blend covers on one flat face: from the edge to where the
//! blend lands, the edge moved theReach into the face (a concentric circle for
//! a circle). The landing ends where it meets the landing line of a bevelled
//! neighbour, or the neighbour itself when that is not bevelled, or square to
//! the edge where the outline carries on tangentially (a rounded corner).
struct Strip {
    bool isLine = true;
    bool isRing = false;
    double reach = 0.0;
    // A line: from the edge's start, its direction, into the face, its length,
    // and how far the landing's ends lie along it from square to the edge's
    // ends (positive towards the edge's end).
    gp_Pnt start;
    gp_Vec along;
    gp_Vec into;
    double length = 0.0;
    double startShift = 0.0;
    double endShift = 0.0;
    // A circle: the edge's circle, the landing's radius, the edge's range.
    gp_Circ circle;
    double landingRadius = 0.0;
    double first = 0.0;
    double last = 0.0;
    //! The strip's outline: the edge, its two ends' connections and the
    //! landing (temporary edges, never added to any shape).
    TopoDS_Compound outline;

    //! True when thePoint (in the face's plane) lies in the strip.
    bool Contains(const gp_Pnt& thePoint) const
    {
        const double aTolerance = Precision::Confusion();
        if (isLine) {
            const gp_Vec aWay(start, thePoint);
            const double anIn = aWay.Dot(into);
            if (anIn < -aTolerance || anIn > reach + aTolerance) {
                return false;
            }
            const double aShare = anIn / reach;
            const double anAlong = aWay.Dot(along);
            return anAlong >= startShift * aShare - aTolerance
                && anAlong <= length + endShift * aShare + aTolerance;
        }
        const double aRadius = circle.Location().Distance(thePoint);
        if (aRadius < std::min(circle.Radius(), landingRadius) - aTolerance
            || aRadius > std::max(circle.Radius(), landingRadius) + aTolerance) {
            return false;
        }
        if (isRing) {
            return true;
        }
        double anAngle = ElCLib::Parameter(circle, thePoint);
        while (anAngle < first - 1.0e-9) {
            anAngle += 2.0 * kPi;
        }
        while (anAngle > first + 2.0 * kPi) {
            anAngle -= 2.0 * kPi;
        }
        return anAngle <= last + 1.0e-9;
    }
};

//! Built: theStrip holds the strip. UsedUp: the landing has no length left
//! (a rounded corner's arc, or the edge between two bevelled corners, is used
//! up). Unknown: not worked out (a face that is not flat, a curve that is not a
//! line or a circle, a curved neighbour at a sharp corner, an outline that is
//! not clear); the caller then measures from the edge itself, as f842bf14 does.
enum class Landing { Built, UsedUp, Unknown };

template <typename SharpTurnOf>
inline Landing StripOn(
    const TopoDS_Face& theFace,
    const TopoDS_Edge& theEdge,
    const double theReach,
    const double theAmount,
    const bool theIsChamfer,
    const TopTools_IndexedMapOfShape& theBlend,
    const SharpTurnOf& theSharpTurnOf,
    Strip& theStrip)
{
    try {
        const BRepAdaptor_Surface aSurface(theFace, Standard_False);
        const BRepAdaptor_Curve aCurve(theEdge);
        const bool isLine = aCurve.GetType() == GeomAbs_Line;
        if (aSurface.GetType() != GeomAbs_Plane
            || (!isLine && aCurve.GetType() != GeomAbs_Circle)) {
            return Landing::Unknown;
        }
        gp_Vec aNormal(aSurface.Plane().Axis().Direction());
        if (theFace.Orientation() == TopAbs_REVERSED) {
            aNormal.Reverse();
        }
        // The edge as the face's outline runs along it (the face on its
        // left), and the face's other outline edges.
        TopoDS_Edge aRun;
        std::vector<TopoDS_Edge> anOthers;
        for (TopExp_Explorer anOutline(theFace, TopAbs_EDGE); anOutline.More(); anOutline.Next()) {
            const TopoDS_Edge anOutlineEdge = TopoDS::Edge(anOutline.Current());
            if (!anOutlineEdge.IsSame(theEdge)) {
                if (!BRep_Tool::Degenerated(anOutlineEdge)) {
                    anOthers.push_back(anOutlineEdge);
                }
            } else if (!aRun.IsNull()) {
                return Landing::Unknown; // runs both ways: a seam
            } else {
                aRun = anOutlineEdge;
            }
        }
        if (aRun.IsNull() || (aRun.Orientation() != TopAbs_FORWARD
                              && aRun.Orientation() != TopAbs_REVERSED)) {
            return Landing::Unknown;
        }
        const double aWay = aRun.Orientation() == TopAbs_REVERSED ? -1.0 : 1.0;
        const auto runAt = [&](const double theParameter, gp_Pnt& thePoint,
                               gp_Vec& theAlong, gp_Vec& theInto) {
            aCurve.D1(theParameter, thePoint, theAlong);
            if (!(theAlong.Magnitude() > 1.0e-12)) {
                return false;
            }
            theAlong.Normalize();
            theAlong *= aWay;
            theInto = aNormal.Crossed(theAlong);
            return true;
        };
        const double aFirst = aCurve.FirstParameter();
        const double aLast = aCurve.LastParameter();
        gp_Pnt aMiddle;
        gp_Vec anAlong;
        gp_Vec anInto;
        if (!runAt(0.5 * (aFirst + aLast), aMiddle, anAlong, anInto)) {
            return Landing::Unknown;
        }
        // The side comes from the outline's direction; checked once: a point
        // just inside the middle of the edge must be in the face.
        const double aLength = isLine
            ? aCurve.Value(aFirst).Distance(aCurve.Value(aLast))
            : aCurve.Circle().Radius() * std::abs(aLast - aFirst);
        const double aStep = 1.0e-3 * std::min(aLength, theReach);
        if (!(aStep > Precision::Confusion())
            || BRepClass_FaceClassifier(theFace,
                   aMiddle.Translated(aStep * anInto), Precision::Confusion())
                       .State() != TopAbs_IN) {
            return Landing::Unknown;
        }
        Strip aStrip;
        aStrip.isLine = isLine;
        aStrip.reach = theReach;
        aStrip.isRing = TopExp::FirstVertex(theEdge).IsSame(TopExp::LastVertex(theEdge));
        if (isLine && aStrip.isRing) {
            return Landing::Unknown;
        }
        if (!isLine) {
            // Smaller when the face lies towards the centre (used up at the
            // radius: a rounded corner's arc, a disc), larger when outside.
            aStrip.circle = aCurve.Circle();
            const bool isInside =
                gp_Vec(aMiddle, aStrip.circle.Location()).Dot(anInto) > 0.0;
            aStrip.landingRadius =
                aStrip.circle.Radius() + (isInside ? -theReach : theReach);
            aStrip.first = std::min(aFirst, aLast);
            aStrip.last = std::max(aFirst, aLast);
            if (!(aStrip.landingRadius > kMargin * aStrip.circle.Radius())) {
                return Landing::UsedUp;
            }
        }
        BRep_Builder aBuilder;
        aBuilder.MakeCompound(aStrip.outline);
        aBuilder.Add(aStrip.outline, theEdge);
        gp_Pnt anEnd[2];
        gp_Pnt aCorner[2];
        if (!aStrip.isRing) {
            TopoDS_Vertex aVertices[2];
            TopExp::Vertices(aRun, aVertices[0], aVertices[1], Standard_True);
            if (aVertices[0].IsNull() || aVertices[1].IsNull()) {
                return Landing::Unknown;
            }
            for (int aSide = 0; aSide < 2; ++aSide) {
                const TopoDS_Vertex& aVertex = aVertices[aSide];
                gp_Vec aRunAlong;
                gp_Vec aRunInto;
                if (!runAt(BRep_Tool::Parameter(aVertex, theEdge), anEnd[aSide],
                           aRunAlong, aRunInto)) {
                    return Landing::Unknown;
                }
                const gp_Vec anOut = aSide == 0 ? -aRunAlong : aRunAlong;
                // The outline's next edge at this end: exactly one.
                TopoDS_Edge aNeighbour;
                for (const TopoDS_Edge& anOther : anOthers) {
                    for (TopExp_Explorer anOtherEnd(anOther, TopAbs_VERTEX);
                         anOtherEnd.More(); anOtherEnd.Next()) {
                        if (!anOtherEnd.Current().IsSame(aVertex)) {
                            continue;
                        }
                        if (!aNeighbour.IsNull() && !aNeighbour.IsSame(anOther)) {
                            return Landing::Unknown;
                        }
                        aNeighbour = anOther;
                    }
                }
                gp_Vec anAway;
                if (aNeighbour.IsNull() || !TangentFrom(aNeighbour, aVertex, anAway)) {
                    return Landing::Unknown;
                }
                const GeomAbs_CurveType aNeighbourType =
                    BRepAdaptor_Curve(aNeighbour).GetType();
                const bool isBlended = theBlend.Contains(aNeighbour);
                double aNeighbourTurn = 0.0;
                if (isBlended && !theSharpTurnOf(aNeighbour, aNeighbourTurn)) {
                    return Landing::Unknown;
                }
                double aShift = 0.0; // along anOut, from square to the edge
                if (anOut.Angle(anAway) < kSmoothTurn) {
                    // Tangent: the chain's next edge takes the landing over.
                    if (!isBlended || (aNeighbourType != GeomAbs_Line
                                       && aNeighbourType != GeomAbs_Circle)) {
                        return Landing::Unknown;
                    }
                } else {
                    // A sharp corner: on the neighbour's own landing line
                    // (bevelled) or on the neighbour (not bevelled).
                    if (!isLine || aNeighbourType != GeomAbs_Line) {
                        return Landing::Unknown;
                    }
                    const gp_Vec aNeighbourInto =
                        aNormal.Crossed(aSide == 0 ? -anAway : anAway);
                    const double aCross = anOut.Dot(aNeighbourInto);
                    if (!(std::abs(aCross) > 1.0e-9)) {
                        return Landing::Unknown;
                    }
                    const double aTarget = isBlended
                        ? Reach(theAmount, aNeighbourTurn, theIsChamfer) : 0.0;
                    aShift = (aTarget - theReach * aRunInto.Dot(aNeighbourInto)) / aCross;
                    // Not bevelled, at an inside corner: never shorter than square.
                    if (!isBlended && anAway.Dot(aRunInto) < 0.0) {
                        aShift = std::max(aShift, 0.0);
                    }
                }
                aCorner[aSide] =
                    anEnd[aSide].Translated(theReach * aRunInto + aShift * anOut);
                (aSide == 0 ? aStrip.startShift : aStrip.endShift) =
                    aSide == 0 ? -aShift : aShift;
                if (anEnd[aSide].Distance(aCorner[aSide]) > Precision::Confusion()) {
                    aBuilder.Add(aStrip.outline,
                        BRepBuilderAPI_MakeEdge(anEnd[aSide], aCorner[aSide]).Edge());
                }
            }
        }
        if (isLine) {
            aStrip.start = anEnd[0];
            aStrip.along = anAlong;
            aStrip.into = anInto;
            aStrip.length = aLength;
            if (!(gp_Vec(aCorner[0], aCorner[1]).Dot(anAlong) > kMargin * aLength)) {
                return Landing::UsedUp; // the landing's two ends have met
            }
            aBuilder.Add(aStrip.outline,
                BRepBuilderAPI_MakeEdge(aCorner[0], aCorner[1]).Edge());
        } else {
            const gp_Circ aLanding(aStrip.circle.Position(), aStrip.landingRadius);
            aBuilder.Add(aStrip.outline, aStrip.isRing
                ? BRepBuilderAPI_MakeEdge(aLanding).Edge()
                : BRepBuilderAPI_MakeEdge(aLanding, aFirst, aLast).Edge());
        }
        theStrip = aStrip;
        return Landing::Built;
    } catch (...) {
        return Landing::Unknown;
    }
}

} // namespace detail

//! True when a round of radius theAmount (or, with theIsChamfer, a flat Bevel
//! of distance theAmount) on theEdges of theShape would reach the far sharp
//! border of a face beside one of those edges. Such an amount is refused as
//! too large for the part and is never handed to OCCT's fillet or chamfer
//! builder. False when every measured face is wide enough and whenever the
//! part's own geometry cannot be measured; (a2) true past the guard's own
//! limits (measuring budget, a distance OCCT cannot give, an exception).
//! theShape must not be in use on another thread (the Bevel worker passes its
//! own copy).
inline bool ReachesFarBorder(
    const TopoDS_Shape& theShape,
    const std::vector<TopoDS_Edge>& theEdges,
    const double theAmount,
    const bool theIsChamfer) noexcept
{
    try {
        if (theShape.IsNull() || theEdges.empty()
            || !std::isfinite(theAmount)
            || !(theAmount > Precision::Confusion())) {
            return false;
        }
        TopTools_IndexedDataMapOfShapeListOfShape aFacesOfEdge;
        TopExp::MapShapesAndUniqueAncestors(
            theShape, TopAbs_EDGE, TopAbs_FACE, aFacesOfEdge);
        // Each edge's turn is worked out once: 0 = not yet, -1 = unknown.
        std::vector<double> aTurns(
            static_cast<std::size_t>(aFacesOfEdge.Extent()) + 1, 0.0);
        const auto aSharpTurnOf = [&](const TopoDS_Edge& theEdge,
                                      double& theTurn) {
            const Standard_Integer anIndex = aFacesOfEdge.FindIndex(theEdge);
            if (anIndex <= 0) {
                return false;
            }
            double& aKnown = aTurns[static_cast<std::size_t>(anIndex)];
            if (aKnown == 0.0) {
                double aTurn = 0.0;
                aKnown = detail::TurnAt(aFacesOfEdge, theEdge, aTurn)
                        && aTurn >= kSmoothTurn
                    ? aTurn : -1.0;
            }
            theTurn = aKnown;
            return aKnown > 0.0;
        };
        // (a2) The selection and the edges OCCT blends with it.
        TopTools_IndexedMapOfShape aBlend;
        for (const TopoDS_Edge& anEdge : theEdges) {
            if (!anEdge.IsNull()) {
                aBlend.Add(anEdge);
            }
        }
        if (!detail::AddTangentChains(theShape, aSharpTurnOf, aBlend)) {
            return true;
        }
        int aMeasuredPairs = 0;
        for (Standard_Integer anEdgeIndex = 1; anEdgeIndex <= aBlend.Extent(); ++anEdgeIndex) {
            const TopoDS_Edge anEdge = TopoDS::Edge(aBlend(anEdgeIndex));
            double aTurn = 0.0;
            TopoDS_Face aSides[2];
            if (BRep_Tool::Degenerated(anEdge)
                || !detail::FacesAt(aFacesOfEdge, anEdge, aSides[0], aSides[1])
                || !aSharpTurnOf(anEdge, aTurn)) {
                continue;
            }
            const double aReach =
                detail::Reach(theAmount, aTurn, theIsChamfer);
            if (!std::isfinite(aReach) || !(aReach > 0.0)) {
                continue;
            }
            TopTools_IndexedMapOfShape anEnds;
            TopExp::MapShapes(anEdge, TopAbs_VERTEX, anEnds);
            for (const TopoDS_Face& aFace : aSides) {
                // (a2) On a flat face, measured from the strip the blend
                // covers: the width is its reach plus the gap beyond the
                // strip. Otherwise from the edge itself (f842bf14).
                detail::Strip aStrip;
                const detail::Landing aKind = detail::StripOn(
                    aFace, anEdge, aReach, theAmount, theIsChamfer, aBlend,
                    aSharpTurnOf, aStrip);
                if (aKind == detail::Landing::UsedUp) {
                    return true;
                }
                const bool fromStrip = aKind == detail::Landing::Built;
                const TopoDS_Shape aFrom = fromStrip
                    ? TopoDS_Shape(aStrip.outline) : TopoDS_Shape(anEdge);
                const double aBase = fromStrip ? aReach : 0.0;
                Bnd_Box aFromBox;
                BRepBndLib::Add(aFrom, aFromBox, Standard_False);
                if (aFromBox.IsVoid()) {
                    continue;
                }
                TopTools_IndexedMapOfShape aBorders;
                TopExp::MapShapes(aFace, TopAbs_EDGE, aBorders);
                for (Standard_Integer anIndex = 1;
                     anIndex <= aBorders.Extent(); ++anIndex) {
                    const TopoDS_Edge aBorder =
                        TopoDS::Edge(aBorders.FindKey(anIndex));
                    if (aBorder.IsSame(anEdge)
                        || BRep_Tool::Degenerated(aBorder)) {
                        continue;
                    }
                    bool isTouching = false;
                    for (TopExp_Explorer aVertex(aBorder, TopAbs_VERTEX);
                         aVertex.More() && !isTouching; aVertex.Next()) {
                        isTouching = anEnds.Contains(aVertex.Current());
                    }
                    if (isTouching) {
                        continue;
                    }
                    double aBorderTurn = 0.0;
                    const bool isSharp = aSharpTurnOf(aBorder, aBorderTurn);
                    if (!isSharp
                        && !(kRoundBeyondASmoothBorderIsAFarBorder
                             && detail::LeadsIntoARound(aFacesOfEdge, aBorder, aFace))) {
                        continue;
                    }
                    double aNeeded = aReach;
                    if (isSharp && aBlend.Contains(aBorder)) {
                        const double aBorderReach = detail::Reach(
                            theAmount, aBorderTurn, theIsChamfer);
                        if (!std::isfinite(aBorderReach)) {
                            continue;
                        }
                        aNeeded += aBorderReach;
                    }
                    // The boxes are never farther apart than the edges: a
                    // pair whose boxes are far enough apart needs no measuring.
                    Bnd_Box aBorderBox;
                    BRepBndLib::Add(aBorder, aBorderBox, Standard_False);
                    if (aBorderBox.IsVoid()
                        || (aBase + aFromBox.Distance(aBorderBox)) * (1.0 - kMargin)
                            > aNeeded) {
                        continue;
                    }
                    // (a2) A border lying inside the strip (a small hole next
                    // to the edge) is crossed by the blend.
                    if (fromStrip && aStrip.Contains(
                            BRep_Tool::Pnt(TopExp::FirstVertex(aBorder)))) {
                        return true;
                    }
                    if (++aMeasuredPairs > kMaximumMeasuredPairs) {
                        return true;
                    }
                    BRepExtrema_DistShapeShape aDistance(aFrom, aBorder);
                    if (!aDistance.IsDone() || aDistance.NbSolution() == 0
                        || !std::isfinite(aDistance.Value())) {
                        return true;
                    }
                    const double aGap = aDistance.Value();
                    if (!fromStrip && !(aGap > Precision::Confusion())) {
                        continue;
                    }
                    if (aNeeded >= (aBase + aGap) * (1.0 - kMargin)) {
                        return true;
                    }
                }
            }
        }
        return false;
    } catch (...) {
        return true;
    }
}

} // namespace core3d::fillet_reach
