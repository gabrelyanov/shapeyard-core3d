// Retained host OCCT regression for the production Shell admission/kernel boundary.
#include "../Core3D/UI/ShellOperationController.hpp"
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS.hxx>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

static void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
static double volume(const TopoDS_Shape& shape) {
    GProp_GProps properties;
    BRepGProp::VolumeProperties(shape, properties);
    return properties.Mass();
}
static std::string bytes(const TopoDS_Shape& shape) {
    std::ostringstream stream;
    BRepTools::Write(shape, stream);
    return stream.str();
}
static TopoDS_Shape shell(const TopoDS_Shape& source,
                          const std::vector<TopoDS_Face>& faces) {
    // Exercise the same deep-copy mapping as the background request.
    BRepBuilderAPI_Copy copy(source, Standard_True, Standard_False);
    std::vector<TopoDS_Face> copied;
    for (const auto& face : faces) copied.push_back(TopoDS::Face(copy.ModifiedShape(face)));
    BRepOffsetAPI_MakeThickSolid builder;
    core3d::MakeInwardShell(builder, copy.Shape(), copied, 1.0, 0.001);
    require(builder.IsDone(), "shell build failed");
    require(BRepCheck_Analyzer(builder.Shape()).IsValid(), "invalid shell");
    require(builder.Shape().ShapeType() == TopAbs_SOLID, "shell is not one solid");
    return builder.Shape();
}
int main() {
    const auto cube = BRepPrimAPI_MakeBox(50, 50, 50).Shape();
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(cube, TopAbs_FACE, faces);
    const auto first = TopoDS::Face(faces(1));
    const auto opposite = TopoDS::Face(faces(2));
    const auto adjacent = TopoDS::Face(faces(3));
    require(core3d::ShellOpeningSetIsValid(cube, {first, opposite}, {0, 1}), "opposite faces rejected");
    require(!core3d::ShellOpeningSetIsValid(cube, {first, adjacent}, {0, 2}), "adjacent faces admitted");
    require(!core3d::ShellOpeningSetIsValid(cube, {first, first}, {0, 0}), "duplicate admitted");
    require(!core3d::ShellOpeningSetIsValid(cube, {opposite, first}, {1, 0}), "unsorted proof admitted");
    require(!core3d::ShellOpeningSetIsValid(cube, {}, {}), "empty admitted");
    require(!core3d::ShellOpeningSetIsValid(cube, std::vector<TopoDS_Face>(9, first),
            std::vector<Standard_Size>(9, 0)), "over-cap admitted");
    TopoDS_Face reversed = first;
    reversed.Reverse();
    require(!core3d::ShellOpeningSetIsValid(cube, {reversed}, {0}), "reversed admitted");
    const auto foreign = BRepPrimAPI_MakeBox(50, 50, 50).Shape();
    TopTools_IndexedMapOfShape foreignFaces;
    TopExp::MapShapes(foreign, TopAbs_FACE, foreignFaces);
    require(!core3d::ShellOpeningSetIsValid(cube, {first, TopoDS::Face(foreignFaces(2))}, {0, 1}),
            "foreign face admitted");
    const double tube = volume(shell(cube, {first, opposite}));
    const double expected = 50.0 * 50.0 * 50.0 - 48.0 * 48.0 * 50.0;
    require(std::abs(tube - expected) <= expected * 1e-6, "square sleeve volume");

    // B05 cylindrical grip: planar caps removed, curved side retained.
    const auto cylinder = BRepPrimAPI_MakeCylinder(15, 120).Shape();
    TopTools_IndexedMapOfShape cylinderFaces;
    TopExp::MapShapes(cylinder, TopAbs_FACE, cylinderFaces);
    std::vector<TopoDS_Face> caps;
    std::vector<Standard_Size> capIndices;
    for (int i = 1; i <= cylinderFaces.Extent(); ++i) {
        const auto face = TopoDS::Face(cylinderFaces(i));
        if (core3d::ShellFaceIsSinglePlanarOpening(face)) {
            caps.push_back(face);
            capIndices.push_back(static_cast<Standard_Size>(i - 1));
        }
    }
    require(caps.size() == 2 && core3d::ShellOpeningSetIsValid(cylinder, caps, capIndices), "grip caps");
    const double grip = volume(shell(cylinder, caps));
    const double expectedGrip = std::acos(-1.0) * (15.0 * 15.0 - 14.0 * 14.0) * 120.0;
    require(std::abs(grip - expectedGrip) <= expectedGrip * 1e-6, "B05 grip exact tube volume");

    // Old one-opening invocation and new helper must serialize identically.
    TopTools_ListOfShape legacyFaces;
    legacyFaces.Append(first);
    BRepOffsetAPI_MakeThickSolid legacy, current;
    legacy.MakeThickSolidByJoin(cube, legacyFaces, -1.0, 0.001, BRepOffset_Skin,
        Standard_False, Standard_False, GeomAbs_Arc, Standard_False);
    core3d::MakeInwardShell(current, cube, {first}, 1.0, 0.001);
    require(legacy.IsDone() && current.IsDone(), "single-opening failed");
    require(bytes(legacy.Shape()) == bytes(current.Shape()), "single-opening BRep bytes changed");
    const double singleExpected = 125000.0 - 48.0 * 48.0 * 49.0;
    require(std::abs(volume(current.Shape()) - singleExpected) <= singleExpected * 1e-6,
            "single-opening volume changed");
    std::cout << std::setprecision(17) << "PASS ShellOperationTests: square sleeve=" << tube
              << ", B05 grip=" << grip
              << ", grip relative error=" << std::abs(grip - expectedGrip) / expectedGrip
              << ", admission negatives, single-opening byte identity\n";
}
