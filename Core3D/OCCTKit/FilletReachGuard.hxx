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
// Deliberately narrow, so that nothing that builds today is refused:
//  - only borders that do not touch the edge are measured (a neighbour at a
//    corner bounds the ends of a blend, not its width);
//  - a border where the surface runs on smoothly (a flat seam a Join left, the
//    side of an earlier round) is not a border: the builder may roll over it;
//  - anything that cannot be measured is left to the builder, as before.
// It does not prove that a blend builds; the builder and every check after it
// still judge the candidate.

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <Geom2d_Curve.hxx>
#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_MapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <cstddef>
#include <vector>

namespace core3d::fillet_reach {

//! Half a degree, as IsSmoothBevelEdge: below it two faces meet smoothly.
constexpr double kSmoothTurn = 0.5 * 3.14159265358979323846 / 180.0;
//! A blend must leave at least this share of the face's width standing, so a
//! size probe never asks the builder for a sliver either.
constexpr double kMargin = 0.01;
//! Exact edge-to-edge distances measured per call; past it the builder decides.
constexpr int kMaximumMeasuredPairs = 1024;

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

} // namespace detail

//! True when a round of radius theAmount (or, with theIsChamfer, a flat Bevel
//! of distance theAmount) on theEdges of theShape would reach the far sharp
//! border of a face beside one of those edges. Such an amount is refused as
//! too large for the part and is never handed to OCCT's fillet or chamfer
//! builder. False when every measured face is wide enough and whenever
//! something cannot be measured. theShape must not be in use on another
//! thread (the Bevel worker passes its own copy).
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
        TopTools_MapOfShape aSelected;
        for (const TopoDS_Edge& anEdge : theEdges) {
            if (!anEdge.IsNull()) {
                aSelected.Add(anEdge);
            }
        }
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
        int aMeasuredPairs = 0;
        for (const TopoDS_Edge& anEdge : theEdges) {
            double aTurn = 0.0;
            TopoDS_Face aSides[2];
            if (anEdge.IsNull() || BRep_Tool::Degenerated(anEdge)
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
            Bnd_Box anEdgeBox;
            BRepBndLib::Add(anEdge, anEdgeBox, Standard_False);
            if (anEdgeBox.IsVoid()) {
                continue;
            }
            for (const TopoDS_Face& aFace : aSides) {
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
                    double aBorderTurn = 0.0;
                    if (isTouching || !aSharpTurnOf(aBorder, aBorderTurn)) {
                        continue;
                    }
                    double aNeeded = aReach;
                    if (aSelected.Contains(aBorder)) {
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
                        || anEdgeBox.Distance(aBorderBox) * (1.0 - kMargin)
                            > aNeeded) {
                        continue;
                    }
                    if (++aMeasuredPairs > kMaximumMeasuredPairs) {
                        return false;
                    }
                    BRepExtrema_DistShapeShape aDistance(anEdge, aBorder);
                    if (!aDistance.IsDone() || aDistance.NbSolution() == 0) {
                        continue;
                    }
                    const double aWidth = aDistance.Value();
                    if (!std::isfinite(aWidth)
                        || !(aWidth > Precision::Confusion())) {
                        continue;
                    }
                    if (aNeeded >= aWidth * (1.0 - kMargin)) {
                        return true;
                    }
                }
            }
        }
        return false;
    } catch (...) {
        return false;
    }
}

} // namespace core3d::fillet_reach
