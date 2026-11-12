#pragma once

#include "ProfileDefinition.hxx"
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <GProp_GProps.hxx>
#include <Message_ProgressIndicator.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pln.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Dir.hxx>
#include <atomic>
#include <optional>
#include <vector>

namespace core3d {
enum class FaceSketchMode { Add, Cut };
struct FaceSketchResult {
    TopoDS_Shape solid;
    double volume = 0;
    gp_Dir outwardNormal;
};

namespace face_sketch_detail {
class Progress final : public Message_ProgressIndicator {
    DEFINE_STANDARD_RTTI_INLINE(Progress,Message_ProgressIndicator)
public:
    explicit Progress(const std::atomic_bool& value) : cancelled(value) {}
protected:
    Standard_Boolean UserBreak() override { return cancelled.load(); }
    void Show(const Message_ProgressScope&,const Standard_Boolean) override {}
private:
    const std::atomic_bool& cancelled;
};
}

// Detached engine operation; source and face must be the same located, valid
// solid's topology. The planar polygon uses that coordinate system and must
// explicitly close (within 1e-7 source units), with 3..64 distinct corners.
// Depth follows the existing profile limits (1e-3..1e6 source units).
// Add follows the source's oriented outward normal; Cut goes inward. The
// complete footprint must lie within the trimmed face, including its holes.
// Returns one valid solid or no result; never modifies source, selection,
// document or history. Publication/history belongs to the calling command.
// Optional refusal receives a static diagnostic string, or nullptr on success.
inline std::optional<FaceSketchResult> BuildFaceSketch(
    const TopoDS_Shape& source, const TopoDS_Face& face,
    const std::vector<gp_Pnt>& closedProfile, double depth,
    FaceSketchMode mode, const std::atomic_bool& cancelled,
    const char** refusal = nullptr) noexcept {
    if (refusal) *refusal = nullptr;
    const auto reject = [&](const char* reason) -> std::optional<FaceSketchResult> {
        if (refusal) *refusal = reason;
        return std::nullopt;
    };
    constexpr double planeTolerance = 1e-7;
    if (cancelled.load() || source.IsNull() || source.ShapeType() != TopAbs_SOLID
        || face.IsNull() || closedProfile.size() < 4 || closedProfile.size() > 65
        || !std::isfinite(depth) || depth < 1e-3 || depth > 1e6
        || (mode != FaceSketchMode::Add && mode != FaceSketchMode::Cut)) return reject("invalid-input");
    try {
        for (const auto& point : closedProfile) {
            if (!std::isfinite(point.X()) || !std::isfinite(point.Y()) || !std::isfinite(point.Z())
                || std::abs(point.X()) > 1e6 || std::abs(point.Y()) > 1e6 || std::abs(point.Z()) > 1e6)
                return reject("invalid-coordinate");
        }
        if (closedProfile.front().Distance(closedProfile.back()) > planeTolerance
            || !BRepCheck_Analyzer(source).IsValid()) return reject("invalid-source-or-open-profile");
        TopTools_IndexedMapOfShape faces; TopExp::MapShapes(source,TopAbs_FACE,faces);
        const int index = faces.FindIndex(face);
        if (index == 0) return reject("foreign-face");
        // Membership ignores orientation. Resolve the canonical source face so
        // a reversed caller handle cannot reverse Add/Cut's physical direction.
        const auto member = TopoDS::Face(faces(index));
        if (member.Orientation() != TopAbs_FORWARD && member.Orientation() != TopAbs_REVERSED)
            return reject("face-orientation");
        BRepAdaptor_Surface surface(member,Standard_True);
        if (surface.GetType() != GeomAbs_Plane) return reject("nonplanar-face");
        const auto plane = surface.Plane();
        auto normal = plane.Axis().Direction();
        if (member.Orientation() == TopAbs_REVERSED) normal.Reverse();
        std::vector<gp_Pnt2d> uv;
        for (std::size_t i = 0; i + 1 < closedProfile.size(); ++i) {
            const auto& point = closedProfile[i];
            if (plane.Distance(point) > planeTolerance) return reject("off-plane");
            const gp_Vec fromOrigin(plane.Location(),point);
            uv.emplace_back(fromOrigin.Dot(gp_Vec(plane.XAxis().Direction())),
                            fromOrigin.Dot(gp_Vec(plane.YAxis().Direction())));
        }
        double signedArea = 0;
        if (!ValidateProfileOutline(uv,0,depth,signedArea) || cancelled.load()) return reject("outline");
        BRepBuilderAPI_MakePolygon polygon;
        for (const auto& point : uv) {
            polygon.Add(plane.Location().Translated(
                gp_Vec(plane.XAxis().Direction()) * point.X()
                + gp_Vec(plane.YAxis().Direction()) * point.Y()));
        }
        polygon.Close();
        if (!polygon.IsDone()) return reject("polygon");
        auto wire = polygon.Wire();
        if (signedArea < 0) wire.Reverse();
        BRepBuilderAPI_MakeFace faceBuilder(plane,wire,Standard_True);
        if (!faceBuilder.IsDone() || !BRepCheck_Analyzer(faceBuilder.Face()).IsValid()) return reject("profile-face");
        const auto profileFace = faceBuilder.Face();
        const auto boolean = [&](BRepAlgoAPI_BooleanOperation& operation,
                                 const TopoDS_Shape& argument, const TopoDS_Shape& tool)
            -> std::optional<TopoDS_Shape> {
            if (cancelled.load()) return std::nullopt;
            TopTools_ListOfShape arguments,tools; arguments.Append(argument); tools.Append(tool);
            operation.SetArguments(arguments); operation.SetTools(tools);
            operation.SetNonDestructive(Standard_True); operation.SetRunParallel(Standard_False);
            operation.SetCheckInverted(Standard_True);
            Handle(face_sketch_detail::Progress) progress = new face_sketch_detail::Progress(cancelled);
            operation.Build(progress->Start());
            if (cancelled.load() || !operation.IsDone() || operation.HasErrors() || operation.Shape().IsNull())
                return std::nullopt;
            return operation.Shape();
        };
        // Vertices alone do not establish containment: a loop can cross a
        // concavity or enclose a face hole while every corner is on the face.
        BRepAlgoAPI_Common common;
        const auto footprint = boolean(common,profileFace,member);
        if (!footprint) return reject("footprint-boolean");
        GProp_GProps area; BRepGProp::SurfaceProperties(*footprint,area);
        const double expectedArea = std::abs(signedArea);
        if (!std::isfinite(area.Mass())
            || std::abs(area.Mass()-expectedArea) > std::max(1e-10,expectedArea*1e-8)) return reject("outside-face");
        BRepPrimAPI_MakePrism prism(profileFace,
            gp_Vec(normal) * (mode == FaceSketchMode::Cut ? -depth : depth),Standard_True,Standard_True);
        if (cancelled.load() || !prism.IsDone() || prism.Shape().ShapeType() != TopAbs_SOLID)
            return reject("prism");
        auto tool = TopoDS::Solid(prism.Shape());
        if (!BRepLib::OrientClosedSolid(tool) || !BRepCheck_Analyzer(tool).IsValid()) return reject("invalid-prism");
        BRepAlgoAPI_Fuse fuse; BRepAlgoAPI_Cut cut;
        const auto built = mode == FaceSketchMode::Add ? boolean(fuse,source,tool) : boolean(cut,source,tool);
        if (!built) return reject("result-boolean");
        auto result = *built;
        if (result.ShapeType() == TopAbs_COMPOUND) {
            TopoDS_Iterator child(result);
            if (!child.More()) return reject("empty-result");
            const auto only = child.Value(); child.Next();
            if (child.More()) return reject("multiple-results");
            result = only;
        }
        if (result.ShapeType() != TopAbs_SOLID || !BRepCheck_Analyzer(result).IsValid()) return reject("invalid-result");
        GProp_GProps previous,after;
        BRepGProp::VolumeProperties(source,previous); BRepGProp::VolumeProperties(result,after);
        const double volume = after.Mass(), before = previous.Mass();
        const double changed = mode == FaceSketchMode::Add ? volume-before : before-volume;
        if (!std::isfinite(before) || before <= 0 || !std::isfinite(volume) || volume <= 0
            || changed <= std::max(1e-12,before*1e-12) || cancelled.load()) return reject("volume");
        return FaceSketchResult{result,volume,normal};
    } catch (...) { return reject("kernel-exception"); }
}
}
