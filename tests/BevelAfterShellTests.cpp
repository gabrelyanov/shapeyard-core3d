// Retained host OCCT regression: Bevel on a part that was shelled first (M14-0, build 12,
// 25 Nov 2026: cube 30, Shell 2 from the top, Object mode, Bevel 2 closed the app inside
// BRepFilletAPI_MakeFillet::Build, 2 of 2).
// One case per process, so that a kernel fault in one case cannot hide the others and can
// never take a test runner down (exit 139 = the kernel faulted):
//   BevelAfterShellTests <case 1..11>     (no argument: cases 1 to 11 in this process)
//   BevelAfterShellTests <90..103>        the kernel with no guard at all, one size per process:
//                                         says whether these host libraries fault (exit 139) or
//                                         what the builder answers. A map of the fault, never
//                                         part of the gated run.
// Before FilletReachGuard.hxx exists this file still compiles and runs the builder as the
// Bevel worker did on 25 Nov: cases 1 to 5 are then red (a fault, or no refusal). With the
// f842bf14 guard, cases 7 to 11 are red (part (a2): corners measured corner to corner, and a
// round inner rim not measured); with (a2) all are green.
#include "../Core3D/UI/ShellOperationController.hpp"
#if __has_include("../Core3D/OCCTKit/FilletReachGuard.hxx")
#include "../Core3D/OCCTKit/FilletReachGuard.hxx"
#define BEVEL_AFTER_SHELL_HAS_GUARD 1
#else
#define BEVEL_AFTER_SHELL_HAS_GUARD 0
#endif
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <gp_Vec.hxx>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace core3d {
// The controller's check seam is defined in ShellOperationController.cpp, which this
// host check does not link.
static int gHeldBack = 0;
Standard_Boolean ShellOffsetGuardSkipped() noexcept { return Standard_False; }
void DebugSetShellOffsetGuardSkipped(Standard_Boolean) noexcept {}
void NoteShellOffsetGuardRefusal() noexcept { ++gHeldBack; }
Standard_Integer ShellOffsetGuardRefusalCount() noexcept { return gHeldBack; }
}

static void require(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}
static TopoDS_Shape oneSolid(const TopoDS_Shape& shape) {
    TopTools_IndexedMapOfShape solids;
    TopExp::MapShapes(shape, TopAbs_SOLID, solids);
    require(solids.Extent() == 1, "the part is not one solid");
    return solids(1);
}
// The flat face at the part's highest Z.
static TopoDS_Face flatTop(const TopoDS_Shape& solid) {
    TopoDS_Face best;
    double level = 0.0;
    for (TopExp_Explorer it(solid, TopAbs_FACE); it.More(); it.Next()) {
        const TopoDS_Face face = TopoDS::Face(it.Current());
        gp_Pln plane;
        if (!core3d::ShellFacePlane(face, plane)
            || std::abs(std::abs(plane.Axis().Direction().Z()) - 1.0) > 1.0e-9) continue;
        const double z = plane.Location().Z();
        if (best.IsNull() || z > level) { best = face; level = z; }
    }
    require(!best.IsNull(), "no flat top");
    return best;
}
// What the Shell tool stores: the builder's solid, or the one handed back in `made`.
static TopoDS_Shape shelled(const TopoDS_Shape& source, const double wall) {
    BRepOffsetAPI_MakeThickSolid builder;
    TopoDS_Shape made;
    const bool given = core3d::MakeInwardShell(
        builder, source, {flatTop(source)}, wall, 0.001, Message_ProgressRange(), nullptr, &made);
    TopoDS_Shape result;
    if (!made.IsNull()) {
        result = made;
    } else if (given && builder.IsDone() && builder.MakeOffset().Error() == BRepOffset_NoError) {
        result = builder.Shape();
    }
    require(!result.IsNull(), "Shell " + std::to_string(wall) + " from the top was not made");
    return oneSolid(result);
}
static constexpr double kPi = 3.14159265358979323846;
// (a2) Two faces that meet within half a degree: the side of an earlier round. Object mode
// leaves such an edge out (ShapeInteractor: already round), and so does this check.
static bool smooth(const TopoDS_Edge& edge, const TopTools_ListOfShape& faces) {
    std::vector<gp_Vec> normals;
    for (TopTools_ListIteratorOfListOfShape it(faces); it.More(); it.Next()) {
        const TopoDS_Face face = TopoDS::Face(it.Value());
        double first = 0.0, last = 0.0;
        const Handle(Geom2d_Curve) curve = BRep_Tool::CurveOnSurface(edge, face, first, last);
        if (curve.IsNull()) return false;
        const gp_Pnt2d uv = curve->Value(0.5 * (first + last));
        gp_Pnt point;
        gp_Vec du, dv;
        BRepAdaptor_Surface(face, Standard_False).D1(uv.X(), uv.Y(), point, du, dv);
        gp_Vec normal = du.Crossed(dv);
        if (normal.Magnitude() < 1.0e-12) return false;
        if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
        normals.push_back(normal);
    }
    return normals.size() == 2 && normals[0].Angle(normals[1]) < 0.5 * kPi / 180.0;
}
// The edges Object mode hands to Bevel on these parts: every edge where two different faces
// meet at a corner (a seam that closes one face on itself, and (a2) the side of an earlier
// round, are left out).
static std::vector<TopoDS_Edge> objectEdges(const TopoDS_Shape& solid) {
    TopTools_IndexedDataMapOfShapeListOfShape facesOfEdge;
    TopExp::MapShapesAndUniqueAncestors(solid, TopAbs_EDGE, TopAbs_FACE, facesOfEdge);
    std::vector<TopoDS_Edge> edges;
    for (int index = 1; index <= facesOfEdge.Extent(); ++index) {
        const TopoDS_Edge edge = TopoDS::Edge(facesOfEdge.FindKey(index));
        if (BRep_Tool::Degenerated(edge) || facesOfEdge.FindFromIndex(index).Extent() != 2
            || smooth(edge, facesOfEdge.FindFromIndex(index))) continue;
        edges.push_back(edge);
    }
    return edges;
}
// (a2) The Object-mode edges whose two ends pass `keep` (Edge mode picks).
template <typename Keep>
static std::vector<TopoDS_Edge> edgesWhere(const TopoDS_Shape& solid, const Keep& keep) {
    std::vector<TopoDS_Edge> edges;
    for (const TopoDS_Edge& edge : objectEdges(solid)) {
        if (keep(BRep_Tool::Pnt(TopExp::FirstVertex(edge)), BRep_Tool::Pnt(TopExp::LastVertex(edge)))) {
            edges.push_back(edge);
        }
    }
    return edges;
}
static bool same(const double a, const double b) { return std::abs(a - b) < 1.0e-6; }
// A box's upright edges: both ends at the same x and y.
static std::vector<TopoDS_Edge> uprights(const TopoDS_Shape& solid) {
    return edgesWhere(solid, [](const gp_Pnt& a, const gp_Pnt& b) {
        return same(a.X(), b.X()) && same(a.Y(), b.Y()) && !same(a.Z(), b.Z());
    });
}
// The top edge along y = 0 at height z (cube 30: the outer rim edge of the shelled cup).
static std::vector<TopoDS_Edge> frontTopEdge(const TopoDS_Shape& solid, const double z) {
    const std::vector<TopoDS_Edge> edges = edgesWhere(solid, [z](const gp_Pnt& a, const gp_Pnt& b) {
        return same(a.Z(), z) && same(b.Z(), z) && same(a.Y(), 0.0) && same(b.Y(), 0.0);
    });
    require(edges.size() == 1, "one top edge along y = 0");
    return edges;
}
// Edge mode, tick: the named edges rounded (or flat-bevelled) by the kernel, no guard.
static TopoDS_Shape treated(const TopoDS_Shape& solid, const std::vector<TopoDS_Edge>& edges,
                            const double amount, const bool flat) {
    require(!edges.empty(), "no edge to treat");
    TopoDS_Shape result;
    if (flat) {
        BRepFilletAPI_MakeChamfer builder(solid);
        for (const TopoDS_Edge& edge : edges) builder.Add(amount, edge);
        builder.Build();
        require(builder.IsDone(), "the first flat Bevel was not made");
        result = builder.Shape();
    } else {
        BRepFilletAPI_MakeFillet builder(solid);
        for (const TopoDS_Edge& edge : edges) builder.Add(amount, edge);
        builder.Build();
        require(builder.IsDone(), "the first round was not made");
        result = builder.Shape();
    }
    return oneSolid(result);
}
// Cube 30 with its four upright edges rounded (or flat-bevelled).
static TopoDS_Shape cubeWithTreatedUprights(const double amount, const bool flat, const bool onlyOne = false) {
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(30.0, 30.0, 30.0).Shape();
    std::vector<TopoDS_Edge> edges = uprights(box);
    if (onlyOne) edges.resize(1);
    return treated(box, edges, amount, flat);
}
// A hexagonal prism standing on z = 0 (side 10 = its corner radius).
static TopoDS_Shape hexagonPrism(const double side, const double height) {
    BRepBuilderAPI_MakePolygon outline;
    for (int corner = 0; corner < 6; ++corner) {
        outline.Add(gp_Pnt(side * std::cos(corner * kPi / 3.0), side * std::sin(corner * kPi / 3.0), 0.0));
    }
    outline.Close();
    const TopoDS_Face base = BRepBuilderAPI_MakeFace(outline.Wire(), Standard_True).Face();
    return oneSolid(BRepPrimAPI_MakePrism(base, gp_Vec(0.0, 0.0, height)).Shape());
}
// Cube 30, Shell 2 from the top, then the four inner rim edges rounded 0.5 (Edge mode, tick).
static TopoDS_Shape cupWithRoundInnerRim() {
    const TopoDS_Shape cup = shelled(BRepPrimAPI_MakeBox(30.0, 30.0, 30.0).Shape(), 2.0);
    const std::vector<TopoDS_Edge> inner = edgesWhere(cup, [](const gp_Pnt& a, const gp_Pnt& b) {
        return same(a.Z(), 30.0) && same(b.Z(), 30.0)
            && std::min({a.X(), a.Y(), b.X(), b.Y()}) > 1.0 && std::max({a.X(), a.Y(), b.X(), b.Y()}) < 29.0;
    });
    require(inner.size() == 4, "four inner rim edges");
    return treated(cup, inner, 0.5, false);
}

enum class Outcome { Refused, Built, Failed };
static const char* name(const Outcome outcome) {
    return outcome == Outcome::Refused ? "refused before the builder"
        : outcome == Outcome::Built ? "built one valid solid" : "refused by the builder";
}
// What the Bevel preview worker does with one part: the guard, then the builder, then the
// result check. flat: a flat Bevel (chamfer) instead of a round. (a2) bevelOn: the named
// edges (Edge mode); bevel: every edge (Object mode).
static Outcome bevelOn(const TopoDS_Shape& solid, const std::vector<TopoDS_Edge>& edges,
                       const double amount, const bool flat, const bool guarded = true) {
    require(!edges.empty(), "no edge to bevel");
#if BEVEL_AFTER_SHELL_HAS_GUARD
    if (guarded && core3d::fillet_reach::ReachesFarBorder(solid, edges, amount, flat)) {
        return Outcome::Refused;
    }
#else
    (void)guarded;
#endif
    TopoDS_Shape result;
    if (flat) {
        BRepFilletAPI_MakeChamfer builder(solid);
        for (const TopoDS_Edge& edge : edges) builder.Add(amount, edge);
        builder.Build();
        if (!builder.IsDone()) return Outcome::Failed;
        result = builder.Shape();
    } else {
        BRepFilletAPI_MakeFillet builder(solid);
        for (const TopoDS_Edge& edge : edges) builder.Add(amount, edge);
        builder.Build();
        if (!builder.IsDone()) return Outcome::Failed;
        result = builder.Shape();
    }
    TopTools_IndexedMapOfShape solids;
    TopExp::MapShapes(result, TopAbs_SOLID, solids);
    return solids.Extent() == 1 && BRepCheck_Analyzer(solids(1), Standard_True).IsValid()
        ? Outcome::Built : Outcome::Failed;
}
static Outcome bevel(const TopoDS_Shape& solid, const double amount, const bool flat,
                     const bool guarded = true) {
    return bevelOn(solid, objectEdges(solid), amount, flat, guarded);
}
static void expect(const std::string& what, const Outcome got, const Outcome wanted) {
    require(got == wanted, what + ": " + name(got) + ", expected: " + name(wanted));
    std::cout << "PASS " << what << ": " << name(got) << "\n";
}
// A size the search may ask the builder for: it must come back (no fault) and not be
// refused by the guard; whether it builds is the builder's own answer and is printed.
static void expectAsked(const std::string& what, const Outcome got) {
    require(got != Outcome::Refused, what + ": refused before the builder, but it fits");
    std::cout << "PASS " << what << ": " << name(got) << "\n";
}

static void runCase(const int number) {
    switch (number) {
    case 90: case 91: case 92: case 93: case 94: case 95: case 96: {
        // 90 the recipe (the wall), 91 between half the wall and the wall, 92 past the wall,
        // 93 the recipe as a flat Bevel, 94 the mug at its wall, 95 the mug past its wall,
        // 96 the mug between half its wall and its wall.
        const bool isMug = number >= 94;
        const bool flat = number == 93;
        const double amount = number == 90 || number == 93 ? 2.0 : number == 91 ? 1.5
            : number == 92 ? 3.0 : number == 94 ? 3.0 : number == 95 ? 5.0 : 2.0;
        const TopoDS_Shape part = isMug
            ? shelled(BRepPrimAPI_MakeCylinder(35.0, 85.0).Shape(), 3.0)
            : shelled(BRepPrimAPI_MakeBox(30.0, 30.0, 30.0).Shape(), 2.0);
        std::cout << "KERNEL " << number << (isMug ? " mug 70 x 85, Shell 3, " : " cube 30, Shell 2, ")
                  << (flat ? "flat Bevel " : "Bevel ") << amount << ", no guard: " << std::flush;
        std::cout << name(bevel(part, amount, flat, false)) << "\n";
        break;
    }
    case 1: {
        // M14-0 as typed: cube 30, Shell 2 from the top, Bevel 2 on every edge.
        const TopoDS_Shape cup = shelled(BRepPrimAPI_MakeBox(30.0, 30.0, 30.0).Shape(), 2.0);
        expect("1 cube 30, Shell 2, Bevel 2 (the wall)", bevel(cup, 2.0, false), Outcome::Refused);
        break;
    }
    case 2: {
        // The same part if By numbers keeps the 50 cube and scales it by 0.6: every size is
        // stored divided by 0.6.
        const double scale = 30.0 / 50.0;
        const TopoDS_Shape cup = shelled(BRepPrimAPI_MakeBox(50.0, 50.0, 50.0).Shape(), 2.0 / scale);
        expect("2 cube 50 shown as 30, Shell 2, Bevel 2", bevel(cup, 2.0 / scale, false), Outcome::Refused);
        break;
    }
    case 3: {
        // The sizes around it. Larger ones (typed, dragged, or a quarter-size probe: Bevel 8
        // probes 2) are refused; the sizes the size search asks for after a refused 2 come
        // back from the builder, and the size the HintLine would name (0.9) builds.
        const TopoDS_Shape cup = shelled(BRepPrimAPI_MakeBox(30.0, 30.0, 30.0).Shape(), 2.0);
        for (const double amount : {8.0, 3.0, 1.25, 1.0}) {
            expect("3 Bevel " + std::to_string(amount), bevel(cup, amount, false), Outcome::Refused);
        }
        for (const double amount : {0.875, 0.96875}) {
            expectAsked("3 Bevel " + std::to_string(amount), bevel(cup, amount, false));
        }
        for (const double amount : {0.5, 0.9}) {
            expect("3 Bevel " + std::to_string(amount), bevel(cup, amount, false), Outcome::Built);
        }
        break;
    }
    case 4: {
        // The flat Bevel walks the same kernel code: the wall is refused, a small one builds.
        const TopoDS_Shape cup = shelled(BRepPrimAPI_MakeBox(30.0, 30.0, 30.0).Shape(), 2.0);
        expect("4 flat Bevel 2", bevel(cup, 2.0, true), Outcome::Refused);
        expect("4 flat Bevel 0.5", bevel(cup, 0.5, true), Outcome::Built);
        break;
    }
    case 5: {
        // The lesson's mug: cylinder 70 x 85, Shell 3 from the top. Two roundings of 1.5 cannot
        // share the 3 mm rim; 1 builds.
        const TopoDS_Shape mug = shelled(BRepPrimAPI_MakeCylinder(35.0, 85.0).Shape(), 3.0);
        expect("5 mug, Bevel 3 (the wall)", bevel(mug, 3.0, false), Outcome::Refused);
        expect("5 mug, Bevel 1.5", bevel(mug, 1.5, false), Outcome::Refused);
        expect("5 mug, Bevel 1", bevel(mug, 1.0, false), Outcome::Built);
        break;
    }
    case 6: {
        // Nothing that builds today is refused: the rounded cube (K01), the lamp switch before
        // its Push-pull (K11), an 8 mm plate with Bevel 3, and the same plate flat.
        expect("6 cube 50, Bevel 3",
               bevel(BRepPrimAPI_MakeBox(50.0, 50.0, 50.0).Shape(), 3.0, false), Outcome::Built);
        expect("6 cylinder 12 x 8, Bevel 1",
               bevel(BRepPrimAPI_MakeCylinder(6.0, 8.0).Shape(), 1.0, false), Outcome::Built);
        expect("6 plate 50 x 50 x 8, Bevel 3",
               bevel(BRepPrimAPI_MakeBox(50.0, 50.0, 8.0).Shape(), 3.0, false), Outcome::Built);
        expect("6 plate 50 x 50 x 8, flat Bevel 3",
               bevel(BRepPrimAPI_MakeBox(50.0, 50.0, 8.0).Shape(), 3.0, true), Outcome::Built);
        break;
    }
    case 7: {
        // (a2) Cube 30, the four upright edges rounded 5, then Object mode. The corner arcs are
        // used up at 5; f842bf14 measured corner to corner (7.07 mm) and refused from 3.5.
        // Edge mode on one top edge: OCCT rounds the whole tangent outline with it.
        const TopoDS_Shape part = cubeWithTreatedUprights(5.0, false);
        expect("7 uprights rounded 5, Object Bevel 4", bevel(part, 4.0, false), Outcome::Built);
        expect("7 uprights rounded 5, Object Bevel 3.4", bevel(part, 3.4, false), Outcome::Built);
        expectAsked("7 uprights rounded 5, Object Bevel 4.8", bevel(part, 4.8, false));
        expect("7 uprights rounded 5, Object Bevel 5.2", bevel(part, 5.2, false), Outcome::Refused);
        const std::vector<TopoDS_Edge> top = frontTopEdge(part, 30.0);
        expect("7 uprights rounded 5, one top edge Bevel 4", bevelOn(part, top, 4.0, false), Outcome::Built);
        expect("7 uprights rounded 5, one top edge Bevel 5.2", bevelOn(part, top, 5.2, false), Outcome::Refused);
        break;
    }
    case 8: {
        // (a2) Cube 30, one upright edge flat-bevelled 2, then an Object-mode round: f842bf14
        // refused from 1.4 (corner to corner across the slanted edge, 2.83 mm); the slanted
        // face itself (2.83 mm wide) is used up from about 3.4.
        const TopoDS_Shape part = cubeWithTreatedUprights(2.0, true, true);
        expect("8 one upright flat 2, Object round 1.5", bevel(part, 1.5, false), Outcome::Built);
        expect("8 one upright flat 2, Object round 3.5", bevel(part, 3.5, false), Outcome::Refused);
        break;
    }
    case 9: {
        // (a2) A hexagonal prism, side 10, height 20. Object mode: f842bf14 refused from 4.95
        // (two edges apart, 10 mm corner to corner); the hexagon's inner circle (8.66) is the
        // real end. One top edge alone, its neighbours not bevelled: from about 8.6 the round's
        // end runs past the neighbour onto the next corner (f842bf14 let it through to 9.9).
        const TopoDS_Shape prism = hexagonPrism(10.0, 20.0);
        expect("9 hexagon, Object Bevel 5", bevel(prism, 5.0, false), Outcome::Built);
        expectAsked("9 hexagon, Object Bevel 6", bevel(prism, 6.0, false));
        expect("9 hexagon, Object Bevel 8.7", bevel(prism, 8.7, false), Outcome::Refused);
        const std::vector<TopoDS_Edge> top = edgesWhere(prism, [](const gp_Pnt& a, const gp_Pnt& b) {
            return same(a.Z(), 20.0) && same(b.Z(), 20.0);
        });
        require(top.size() == 6, "six top edges");
        expect("9 hexagon, one top edge Bevel 5", bevelOn(prism, {top.front()}, 5.0, false), Outcome::Built);
        expect("9 hexagon, one top edge Bevel 9", bevelOn(prism, {top.front()}, 9.0, false), Outcome::Refused);
        break;
    }
    case 10: {
        // (a2) Cube 30, uprights rounded 3, Shell 2 from the top: the inner corners are 1 mm
        // arcs. f842bf14 refused from 0.7 (corner to corner across them); the rim still refuses
        // from half the wall, and just below the wall (the crash's neighbours).
        const TopoDS_Shape cup = shelled(cubeWithTreatedUprights(3.0, false), 2.0);
        expect("10 round-cornered cup, Object Bevel 0.9", bevel(cup, 0.9, false), Outcome::Built);
        expect("10 round-cornered cup, Object Bevel 0.5", bevel(cup, 0.5, false), Outcome::Built);
        for (const double amount : {1.0, 1.9, 2.0}) {
            expect("10 round-cornered cup, Object Bevel " + std::to_string(amount),
                   bevel(cup, amount, false), Outcome::Refused);
        }
        break;
    }
    case 11: {
        // (a2, open point ii) Cube 30, Shell 2, the inner rim rounded 0.5: 1.5 mm of flat rim
        // is left before the round falls away. A round on the outer rim larger than that has
        // nowhere to land (M14-0's dead end); f842bf14 never measured the smooth edge.
        const TopoDS_Shape cup = cupWithRoundInnerRim();
        const std::vector<TopoDS_Edge> rim = frontTopEdge(cup, 30.0);
        expect("11 round inner rim, one outer rim edge Bevel 2", bevelOn(cup, rim, 2.0, false), Outcome::Refused);
        expect("11 round inner rim, Object Bevel 2", bevel(cup, 2.0, false), Outcome::Refused);
        expectAsked("11 round inner rim, one outer rim edge Bevel 1", bevelOn(cup, rim, 1.0, false));
        break;
    }
    case 97: case 98: case 99: {
        // One outer rim edge of cube 30, Shell 2 (Edge mode), no guard: 97 at 95 % of the wall,
        // 98 at 99 % (the largest size the guard lets through), 99 at the wall.
        const double amount = number == 97 ? 1.9 : number == 98 ? 1.98 : 2.0;
        const TopoDS_Shape cup = shelled(BRepPrimAPI_MakeBox(30.0, 30.0, 30.0).Shape(), 2.0);
        std::cout << "KERNEL " << number << " cube 30, Shell 2, one outer rim edge, Bevel " << amount
                  << ", no guard: " << std::flush;
        std::cout << name(bevelOn(cup, frontTopEdge(cup, 30.0), amount, false, false)) << "\n";
        break;
    }
    case 100: case 101: {
        // The inner rim rounded 0.5 first, then Bevel 2, no guard: 100 one outer rim edge,
        // 101 every edge (Object mode).
        const TopoDS_Shape cup = cupWithRoundInnerRim();
        std::cout << "KERNEL " << number << " cube 30, Shell 2, inner rim round 0.5, "
                  << (number == 100 ? "one outer rim edge" : "every edge") << ", Bevel 2, no guard: " << std::flush;
        std::cout << name(number == 100 ? bevelOn(cup, frontTopEdge(cup, 30.0), 2.0, false, false)
                                        : bevel(cup, 2.0, false, false)) << "\n";
        break;
    }
    case 102: {
        // A hexagonal prism (side 10), one top edge Bevel 9, no guard: the size (a2) newly refuses.
        const TopoDS_Shape prism = hexagonPrism(10.0, 20.0);
        const std::vector<TopoDS_Edge> top = edgesWhere(prism, [](const gp_Pnt& a, const gp_Pnt& b) {
            return same(a.Z(), 20.0) && same(b.Z(), 20.0);
        });
        require(!top.empty(), "a top edge");
        std::cout << "KERNEL 102 hexagon side 10, one top edge, Bevel 9, no guard: " << std::flush;
        std::cout << name(bevelOn(prism, {top.front()}, 9.0, false, false)) << "\n";
        break;
    }
    case 103: {
        // Cube 30, uprights rounded 5, Object Bevel 5.2 (past the corner arcs), no guard.
        std::cout << "KERNEL 103 cube 30, uprights rounded 5, Object Bevel 5.2, no guard: " << std::flush;
        std::cout << name(bevel(cubeWithTreatedUprights(5.0, false), 5.2, false, false)) << "\n";
        break;
    }
    default:
        throw std::runtime_error("unknown case " + std::to_string(number));
    }
}

int main(int argc, char** argv) {
    try {
        if (argc > 1) {
            runCase(std::atoi(argv[1]));
        } else {
            for (int number = 1; number <= 11; ++number) runCase(number);
        }
        return 0;
    } catch (const std::exception& failure) {
        std::cout << "FAIL " << failure.what() << "\n";
        return 1;
    }
}
