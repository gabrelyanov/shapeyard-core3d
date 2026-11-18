// Retained host OCCT regression: Shell on a part that was bevelled or rounded first
// (AFTER-BEVEL, desk lamp build 6: K01 rounded cube, K02 bevelled lamp shade).
// One case per process, so that a kernel fault in one case cannot hide the others:
//   ShellAfterBevelTests <case 1..5>      (no argument: every case in this process)
#include "../Core3D/UI/ShellOperationController.hpp"
#include <BRepAdaptor_Curve.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

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
static double volume(const TopoDS_Shape& shape) {
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties);
    return properties.Mass();
}
static TopoDS_Shape oneSolid(const TopoDS_Shape& shape) {
    TopTools_IndexedMapOfShape solids;
    TopExp::MapShapes(shape, TopAbs_SOLID, solids);
    require(solids.Extent() == 1, "the part is not one solid");
    return solids(1);
}
// The flat face at the part's lowest (or highest) Z.
static TopoDS_Face flatEnd(const TopoDS_Shape& solid, const bool top) {
    TopoDS_Face best;
    double level = 0.0;
    for (TopExp_Explorer it(solid, TopAbs_FACE); it.More(); it.Next()) {
        const TopoDS_Face face = TopoDS::Face(it.Current());
        gp_Pln plane;
        if (!core3d::ShellFacePlane(face, plane)
            || std::abs(std::abs(plane.Axis().Direction().Z()) - 1.0) > 1.0e-9) continue;
        const double z = plane.Location().Z();
        if (best.IsNull() || (top ? z > level : z < level)) { best = face; level = z; }
    }
    require(!best.IsNull(), "no flat end");
    return best;
}
// Cube 50, Bevel 3 on every edge (K01).
static TopoDS_Shape roundedCube() {
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(50.0, 50.0, 50.0).Shape();
    BRepFilletAPI_MakeFillet fillet(box);
    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(box, TopAbs_EDGE, edges);
    for (int i = 1; i <= edges.Extent(); ++i) fillet.Add(3.0, TopoDS::Edge(edges(i)));
    fillet.Build();
    require(fillet.IsDone(), "Bevel 3 on the cube");
    return oneSolid(fillet.Shape());
}
// The lamp shade (K02): a cone 120 wide and 110 high cut flat at half its height, Bevel 0.8
// on both rims. Here with exact cone faces; the app's resized cone is stored as splines.
static TopoDS_Shape lampShade() {
    const TopoDS_Shape cone = BRepPrimAPI_MakeCone(60.0, 30.0, 55.0).Shape();
    BRepFilletAPI_MakeFillet fillet(cone);
    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(cone, TopAbs_EDGE, edges);
    int rims = 0;
    for (int i = 1; i <= edges.Extent(); ++i) {
        const TopoDS_Edge edge = TopoDS::Edge(edges(i));
        if (BRep_Tool::Degenerated(edge) || BRepAdaptor_Curve(edge).GetType() != GeomAbs_Circle) continue;
        fillet.Add(0.8, edge);
        ++rims;
    }
    require(rims == 2, "the shade has two rims");
    fillet.Build();
    require(fillet.IsDone(), "Bevel 0.8 on the shade");
    return oneSolid(fillet.Shape());
}

struct Outcome {
    TopoDS_Shape shape;   // null: no shell was made
    bool viaBuilder = false;
    int heldBack = 0;
};
// What the Shell tool's worker does: the builder, or the solid handed back in `made`.
static Outcome shell(const TopoDS_Shape& source, const TopoDS_Face& opening, const double wall) {
    BRepOffsetAPI_MakeThickSolid builder;
    TopoDS_Shape made;
    const int before = core3d::ShellOffsetGuardRefusalCount();
    const bool given = core3d::MakeInwardShell(
        builder, source, {opening}, wall, 0.001, Message_ProgressRange(), nullptr, &made);
    Outcome outcome;
    outcome.heldBack = core3d::ShellOffsetGuardRefusalCount() - before;
    if (!made.IsNull()) {
        outcome.shape = made;
    } else if (given && builder.IsDone() && builder.MakeOffset().Error() == BRepOffset_NoError) {
        outcome.shape = builder.Shape();
        outcome.viaBuilder = true;
    }
    return outcome;
}
// A valid solid with one skin (the hollow is open), the part's outside untouched, and a
// hollow of the expected volume (0.2 %).
static void requireShell(const std::string& name, const TopoDS_Shape& source,
                         const Outcome& outcome, const double expectedHollow) {
    require(!outcome.shape.IsNull(), name + ": no shell was made");
    require(outcome.heldBack == 0, name + ": a made shell was counted as a refusal");
    require(outcome.shape.ShapeType() == TopAbs_SOLID, name + ": the shell is not one solid");
    require(BRepCheck_Analyzer(outcome.shape, Standard_True).IsValid(), name + ": the shell is not valid");
    TopTools_IndexedMapOfShape skins;
    TopExp::MapShapes(outcome.shape, TopAbs_SHELL, skins);
    require(skins.Extent() == 1, name + ": the hollow is not open");
    const double hollow = volume(source) - volume(outcome.shape);
    require(std::abs(hollow - expectedHollow) <= 0.002 * expectedHollow,
            name + ": hollow " + std::to_string(hollow) + " mm3, expected " + std::to_string(expectedHollow));
    std::cout << "PASS " << name << ": hollow " << hollow << " mm3, shell " << volume(outcome.shape) << " mm3\n";
}

static void runCase(const int number) {
    const double pi = std::acos(-1.0);
    switch (number) {
    case 1: {
        // K01: Cube, Bevel 3, Shell 2 from the top. The wall (2) is thinner than the rounding (3).
        // Hollow: the cube moved in by 2 (a 46 cube with 1 mm roundings) plus the 44 x 44 x 2
        // column under the flat top.
        const TopoDS_Shape cube = roundedCube();
        const double moved = 44.0 * 44.0 * 44.0 + 6.0 * 44.0 * 44.0 + 3.0 * pi * 44.0 + 4.0 / 3.0 * pi;
        requireShell("1 rounded cube, Shell 2 from the top", cube,
                     shell(cube, flatEnd(cube, true), 2.0), moved + 44.0 * 44.0 * 2.0);
        break;
    }
    case 2: {
        // The same cube, Shell 5 from the bottom: the wall is thicker than the rounding, so the
        // hollow is the one of the plain cube: 40 x 40 x 45.
        const TopoDS_Shape cube = roundedCube();
        requireShell("2 rounded cube, Shell 5 from the bottom", cube,
                     shell(cube, flatEnd(cube, false), 5.0), 40.0 * 40.0 * 45.0);
        break;
    }
    case 3: {
        // K02: the lamp shade, Shell 3 and Shell 2 from the wide bottom. Hollow: a cone frustum
        // from the bottom to (55 - wall), its radius the shade's less wall / cos(half angle).
        const TopoDS_Shape shade = lampShade();
        const double slope = 60.0 / 110.0, stretch = std::sqrt(1.0 + slope * slope);
        for (const double wall : {3.0, 2.0}) {
            const double height = 55.0 - wall;
            const double wide = 60.0 - wall * stretch;
            const double narrow = 60.0 - height * slope - wall * stretch;
            const double hollow = pi * height / 3.0 * (wide * wide + wide * narrow + narrow * narrow);
            requireShell("3 lamp shade with Bevel 0.8, Shell " + std::to_string(static_cast<int>(wall))
                         + " from the bottom", shade, shell(shade, flatEnd(shade, false), wall), hollow);
        }
        break;
    }
    case 4: {
        // The refusal's number: the rounded edge the wall has to stay under; 0 when none does.
        const TopoDS_Shape cube = roundedCube();
        const TopoDS_Shape shade = lampShade();
        const TopoDS_Shape box = BRepPrimAPI_MakeBox(50.0, 50.0, 50.0).Shape();
        require(std::abs(core3d::ShellRoundedEdgeLimit(cube, {flatEnd(cube, false)}, 5.0, 0.001) - 3.0) <= 1.0e-6,
                "4: Shell 5 on the rounded cube names its 3 mm rounding");
        require(core3d::ShellRoundedEdgeLimit(cube, {flatEnd(cube, false)}, 2.0, 0.001) == 0.0,
                "4: Shell 2 on the rounded cube is not limited by its 3 mm rounding");
        require(std::abs(core3d::ShellRoundedEdgeLimit(shade, {flatEnd(shade, false)}, 3.0, 0.001) - 0.8) <= 1.0e-6,
                "4: Shell 3 on the lamp shade names its 0.8 mm rounding");
        require(core3d::ShellRoundedEdgeLimit(box, {flatEnd(box, false)}, 5.0, 0.001) == 0.0,
                "4: a plain cube has no rounding");
        std::cout << "PASS 4 the rounded edge a refusal names\n";
        break;
    }
    case 5: {
        // Nothing that worked before changes: a plain cube still goes through the builder, and
        // a caller that reads only the builder (a saved profile's replay) is refused the
        // rounded cube exactly as before, counted once.
        const TopoDS_Shape box = BRepPrimAPI_MakeBox(50.0, 50.0, 50.0).Shape();
        const Outcome plain = shell(box, flatEnd(box, true), 1.0);
        require(plain.viaBuilder, "5: the plain cube no longer goes through the builder");
        requireShell("5 plain cube, Shell 1 from the top", box, plain, 48.0 * 48.0 * 49.0);
        const TopoDS_Shape cube = roundedCube();
        BRepOffsetAPI_MakeThickSolid builder;
        const int before = core3d::ShellOffsetGuardRefusalCount();
        require(!core3d::MakeInwardShell(builder, cube, {flatEnd(cube, true)}, 2.0, 0.001),
                "5: the builder-only call was given the rounded cube");
        require(core3d::ShellOffsetGuardRefusalCount() - before == 1, "5: the refusal is counted once");
        std::cout << "PASS 5 builder-only callers unchanged\n";
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
            for (int number = 1; number <= 5; ++number) runCase(number);
        }
        return 0;
    } catch (const std::exception& failure) {
        std::cout << "FAIL " << failure.what() << "\n";
        return 1;
    }
}
