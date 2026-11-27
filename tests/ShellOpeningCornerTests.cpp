// Retained host OCCT regression: Shell on a part whose curved face touches the opening at one point
// (27 Nov 2026; the real App Store 5.34.3 project "M5-03": a cube with a ball-shaped pocket that touches two
// sides of the cube at one point each; Shell from such a side closed the app: SIGSEGV in
// BRepOffset_Tool::TryProject under BRepOffset_Inter3d::ContextIntByArc).
// One case per process, so that a kernel fault in one case cannot hide the others:
//   ShellOpeningCornerTests <case 1..6>      (no argument: every case in this process)
// Without the guard (ShellOpeningCornersAreSafe) cases 1 to 4 end with signal 11.
#include "../Core3D/UI/ShellOperationController.hpp"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
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
// The cube "+" gives: 50 wide, standing on Z = 0, its middle on the Z axis.
static TopoDS_Shape cube() { return BRepPrimAPI_MakeBox(gp_Pnt(-25, -25, 0), gp_Pnt(25, 25, 50)).Shape(); }
static TopoDS_Shape ball(const gp_Pnt& centre) { return BRepPrimAPI_MakeSphere(centre, 25).Shape(); }
static TopoDS_Shape cut(const TopoDS_Shape& a, const TopoDS_Shape& b) {
    BRepAlgoAPI_Cut made(a, b);
    require(made.IsDone() && !made.HasErrors(), "Subtract");
    return oneSolid(made.Shape());
}
static TopoDS_Shape join(const TopoDS_Shape& a, const TopoDS_Shape& b) {
    BRepAlgoAPI_Fuse made(a, b);
    require(made.IsDone() && !made.HasErrors(), "Join");
    return oneSolid(made.Shape());
}
// The one whole flat side of the part in the plane `coordinate` = `value` (X: 0, Y: 1, Z: 2).
static TopoDS_Face side(const TopoDS_Shape& solid, const int coordinate, const double value) {
    TopoDS_Face found;
    for (TopExp_Explorer it(solid, TopAbs_FACE); it.More(); it.Next()) {
        const TopoDS_Face face = TopoDS::Face(it.Current());
        gp_Pln plane;
        if (!core3d::ShellFaceIsSinglePlanarOpening(face) || !core3d::ShellFacePlane(face, plane)) continue;
        const gp_Dir normal = plane.Axis().Direction();
        if (std::abs(std::abs(normal.Coord(coordinate + 1)) - 1.0) > 1.0e-9
            || std::abs(plane.Location().Coord(coordinate + 1) - value) > 1.0e-6) continue;
        GProp_GProps area;
        BRepGProp::SurfaceProperties(face, area);
        if (area.Mass() < 2000.0) continue;
        require(found.IsNull(), "two sides in one plane");
        found = face;
    }
    require(!found.IsNull(), "no such side");
    return found;
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
// Held back from the kernel and nothing made: an ordinary refused preview.
static void requireRefusedByTheGuard(const std::string& name, const TopoDS_Shape& source,
                                     const TopoDS_Face& opening, const double wall) {
    require(BRepCheck_Analyzer(source, Standard_True).IsValid(), name + ": the part is valid");
    require(!core3d::ShellOpeningCornersAreSafe(source, {opening}), name + ": the guard lets it through");
    require(!core3d::ShellOffsetIsSafe(source, {opening}), name + ": the kernel would be given it");
    const Outcome outcome = shell(source, opening, wall);
    require(outcome.shape.IsNull(), name + ": a shell was made");
    require(outcome.heldBack == 1, name + ": the refusal is not counted once");
    BRepOffsetAPI_MakeThickSolid builder;
    require(!core3d::MakeInwardShell(builder, source, {opening}, wall, 0.001),
            name + ": a builder-only caller (a saved profile's replay) was given it");
    std::cout << "PASS " << name << ": held back from the kernel, refused\n";
}

static void runCase(const int number) {
    switch (number) {
    case 1: {
        // M5-03: the cube moved 22.4 along X, minus the ball "+" gives (its middle on the floor).
        const TopoDS_Shape part = cut(BRepPrimAPI_MakeBox(gp_Pnt(-2.6, -25, 0), gp_Pnt(47.4, 25, 50)).Shape(),
                                      ball(gp_Pnt(0, 0, 0)));
        for (const double wall : {0.5, 2.0, 5.0}) {
            requireRefusedByTheGuard("1 cube minus ball as in M5-03, side Y+ wall " + std::to_string(wall),
                                     part, side(part, 1, 25), wall);
            requireRefusedByTheGuard("1 cube minus ball as in M5-03, side Y- wall " + std::to_string(wall),
                                     part, side(part, 1, -25), wall);
        }
        break;
    }
    case 2: {
        // "+" cube, "+" ball, Subtract, nothing moved: the ball touches all four sides.
        const TopoDS_Shape part = cut(cube(), ball(gp_Pnt(0, 0, 0)));
        requireRefusedByTheGuard("2 cube minus ball, nothing moved, side X+", part, side(part, 0, 25), 2.0);
        requireRefusedByTheGuard("2 cube minus ball, nothing moved, side Y-", part, side(part, 1, -25), 2.0);
        break;
    }
    case 3: {
        // A pointed cone joined onto a cube of the same width: its tip is a degenerated edge too.
        const TopoDS_Shape part = join(cube(), BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0, 0, 50), gp_Dir(0, 0, 1)), 25, 0, 40).Shape());
        requireRefusedByTheGuard("3 cone on a cube, side X+", part, side(part, 0, 25), 2.0);
        break;
    }
    case 4: {
        // A ball joined onto a cube of the same width (a dome): held back from the kernel, and made by the
        // route for held-back openings: a valid shell, 2 mm walls.
        const TopoDS_Shape part = join(cube(), ball(gp_Pnt(0, 0, 50)));
        const TopoDS_Face opening = side(part, 1, 25);
        require(!core3d::ShellOffsetIsSafe(part, {opening}), "4: the kernel would be given the dome's side");
        const Outcome outcome = shell(part, opening, 2.0);
        require(!outcome.shape.IsNull() && !outcome.viaBuilder, "4: the dome's side made no shell");
        require(outcome.heldBack == 0, "4: a made shell was counted as a refusal");
        require(outcome.shape.ShapeType() == TopAbs_SOLID
                && BRepCheck_Analyzer(outcome.shape, Standard_True).IsValid(), "4: the shell is not a valid solid");
        const double shellVolume = volume(outcome.shape);
        require(shellVolume > 26000.0 && shellVolume < 28500.0, "4: shell volume " + std::to_string(shellVolume));
        std::cout << "PASS 4 dome on a cube, side Y+: shell " << shellVolume << " mm3 of " << volume(part) << "\n";
        break;
    }
    case 5: {
        // Nothing that worked before changes: a curved face with a pole or a tip that SHARES an edge with the
        // opening still goes to the kernel (a cone from its base, half a ball from its flat side, a ball
        // pocket through a corner), and so does a plain cube.
        struct Kept { std::string name; TopoDS_Shape part; int coordinate; double value; double expected; };
        const TopoDS_Shape cone = BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), 25, 0, 50).Shape();
        const TopoDS_Shape half = cut(ball(gp_Pnt(0, 0, 0)), BRepPrimAPI_MakeBox(gp_Pnt(-30, -30, -30), gp_Pnt(30, 30, 0)).Shape());
        const TopoDS_Shape corner = cut(cube(), BRepPrimAPI_MakeSphere(gp_Pnt(25, 25, 50), 20).Shape());
        const Kept kept[] = {
            {"5 plain cube from the top", cube(), 2, 50, 23432.0},
            {"5 cone from its base", cone, 2, 0, 8019.0363},
            {"5 half a ball from its flat side", half, 2, 0, 7242.4183},
            {"5 cube with a ball pocket through a corner, from the bottom", corner, 2, 0, 22799.2478},
        };
        for (const Kept& k : kept) {
            const TopoDS_Shape solid = oneSolid(k.part);
            TopoDS_Face opening;
            for (TopExp_Explorer it(solid, TopAbs_FACE); it.More() && opening.IsNull(); it.Next()) {
                gp_Pln plane;
                const TopoDS_Face face = TopoDS::Face(it.Current());
                if (core3d::ShellFaceIsSinglePlanarOpening(face) && core3d::ShellFacePlane(face, plane)
                    && std::abs(std::abs(plane.Axis().Direction().Coord(k.coordinate + 1)) - 1.0) < 1.0e-9
                    && std::abs(plane.Location().Coord(k.coordinate + 1) - k.value) < 1.0e-6) opening = face;
            }
            require(!opening.IsNull(), k.name + ": no such face");
            require(core3d::ShellOpeningCornersAreSafe(solid, {opening}), k.name + ": held back by the new guard");
            const Outcome outcome = shell(solid, opening, 2.0);
            require(outcome.viaBuilder && outcome.heldBack == 0, k.name + ": no longer made by the kernel");
            require(BRepCheck_Analyzer(outcome.shape, Standard_True).IsValid(), k.name + ": not valid");
            const double made = volume(outcome.shape);
            require(std::abs(made - k.expected) <= 1.0e-4 * k.expected,
                    k.name + ": shell " + std::to_string(made) + " mm3, was " + std::to_string(k.expected));
            std::cout << "PASS " << k.name << ": shell " << made << " mm3, as before\n";
        }
        break;
    }
    case 6: {
        // Doubt is "not safe": an opening that is not a face of the part, and no opening at all.
        const TopoDS_Shape part = cube();
        const TopoDS_Shape other = BRepPrimAPI_MakeBox(gp_Pnt(100, 100, 100), gp_Pnt(160, 160, 160)).Shape();
        require(!core3d::ShellOpeningCornersAreSafe(part, {side(other, 2, 160)}), "6: a foreign opening is judged safe");
        require(!core3d::ShellOpeningCornersAreSafe(part, {}), "6: no opening is judged safe");
        require(!core3d::ShellOpeningCornersAreSafe(TopoDS_Shape(), {side(part, 2, 50)}), "6: no part is judged safe");
        std::cout << "PASS 6 doubt is not safe\n";
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
    } catch (const Standard_Failure& failure) {
        std::cout << "FAIL kernel exception " << failure.GetMessageString() << "\n";
        return 1;
    }
}
