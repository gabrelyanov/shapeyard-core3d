// Retained host OCCT regression: Bevel on a part that was shelled first (M14-0, build 12,
// 25 Nov 2026: cube 30, Shell 2 from the top, Object mode, Bevel 2 closed the app inside
// BRepFilletAPI_MakeFillet::Build, 2 of 2).
// One case per process, so that a kernel fault in one case cannot hide the others and can
// never take a test runner down (exit 139 = the kernel faulted):
//   BevelAfterShellTests <case 1..6>      (no argument: cases 1 to 6 in this process)
//   BevelAfterShellTests <90..96>         the kernel with no guard at all, one size per process:
//                                         says whether these host libraries fault (exit 139) or
//                                         what the builder answers. A map of the fault, never
//                                         part of the gated run.
// Before FilletReachGuard.hxx exists this file still compiles and runs the builder as the
// Bevel worker did on 25 Nov: cases 1 to 5 are then red (a fault, or no refusal).
#include "../Core3D/UI/ShellOperationController.hpp"
#if __has_include("../Core3D/OCCTKit/FilletReachGuard.hxx")
#include "../Core3D/OCCTKit/FilletReachGuard.hxx"
#define BEVEL_AFTER_SHELL_HAS_GUARD 1
#else
#define BEVEL_AFTER_SHELL_HAS_GUARD 0
#endif
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRep_Tool.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
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
// The edges Object mode hands to Bevel on these parts: every edge where two different faces
// meet (a seam that closes one face on itself is left out).
static std::vector<TopoDS_Edge> objectEdges(const TopoDS_Shape& solid) {
    TopTools_IndexedDataMapOfShapeListOfShape facesOfEdge;
    TopExp::MapShapesAndUniqueAncestors(solid, TopAbs_EDGE, TopAbs_FACE, facesOfEdge);
    std::vector<TopoDS_Edge> edges;
    for (int index = 1; index <= facesOfEdge.Extent(); ++index) {
        const TopoDS_Edge edge = TopoDS::Edge(facesOfEdge.FindKey(index));
        if (BRep_Tool::Degenerated(edge) || facesOfEdge.FindFromIndex(index).Extent() != 2) continue;
        edges.push_back(edge);
    }
    return edges;
}

enum class Outcome { Refused, Built, Failed };
static const char* name(const Outcome outcome) {
    return outcome == Outcome::Refused ? "refused before the builder"
        : outcome == Outcome::Built ? "built one valid solid" : "refused by the builder";
}
// What the Bevel preview worker does with one part: the guard, then the builder, then the
// result check. flat: a flat Bevel (chamfer) instead of a round.
static Outcome bevel(const TopoDS_Shape& solid, const double amount, const bool flat,
                     const bool guarded = true) {
    const std::vector<TopoDS_Edge> edges = objectEdges(solid);
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
    default:
        throw std::runtime_error("unknown case " + std::to_string(number));
    }
}

int main(int argc, char** argv) {
    try {
        if (argc > 1) {
            runCase(std::atoi(argv[1]));
        } else {
            for (int number = 1; number <= 6; ++number) runCase(number);
        }
        return 0;
    } catch (const std::exception& failure) {
        std::cout << "FAIL " << failure.what() << "\n";
        return 1;
    }
}
