#pragma once

// Detached P1b builder.  It reuses ShellOperationController's retained replay
// seam and A1's bounded Boolean helper.  It owns no label or document command.
#include "RetainedPartBoolean.hxx"
#include "../UI/ShellOperationController.hpp"
#include <BinTools.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <algorithm>
#include <atomic>
#include <array>
#include <cstdint>
#include <istream>
#include <map>
#include <ostream>
#include <set>
#include <streambuf>
#include <string>
#include <tuple>
#include <vector>

namespace core3d::part_boolean::build {

enum class ShellFixture : std::uint8_t {
    S = 1,
    S2 = 2,
    UpsideDown = 3,
    Thin = 4,
};

struct ShellBuild final {
    profile::Parameters recipe;
    std::vector<double> canonicalRecipe;
    TopoDS_Shape base;
    TopoDS_Shape shell;
    bool replayed = false;
};

struct BooleanBuild final {
    retained_part_boolean::Candidate candidate;
    retained_part_boolean::CandidateEvidence evidence;
};

struct AnalyticBuild final {
    std::array<TopoDS_Shape, 2> sources;
    retained_part_boolean::Candidate candidate;
    retained_part_boolean::CandidateEvidence evidence;
    std::array<std::string, 2> sourceBytes;
    bool complete = false;
};

//! Production P1 build input. Every value is decoded from the captured
//! SYCR/2 graph and SYPB/2 feature; no test scenario selects this branch.
struct ShellCompositeRequest final {
    Definition definition;
    std::array<profile::Parameters, 2> profiles;
    std::array<composite_recipe::InputPlacement, 2> placements;
};

struct SourceFaceClass final {
    TopoDS_Face face;
    RegionKind kind = RegionKind::OuterWall;
};

struct FaceHistoryRelation final {
    std::string faceKey;
    std::uint8_t sourceInput = 0;
    RegionKind kind = RegionKind::OuterWall;
    bool generated = false;
};

struct ShellCompositeBuild final {
    std::array<TopoDS_Shape, 2> sources;
    retained_part_boolean::Candidate candidate;
    retained_part_boolean::CandidateEvidence evidence;
    std::array<std::string, 2> sourceBytes;
    std::vector<FaceHistoryRelation> faceHistory;
    bool historyResultExact = false;
    bool complete = false;
};

// Analytic prism pairs are deliberately small.  Keep their persistence
// materialization detached and bounded rather than involving an OCAF shape set.
class PersistenceBuffer final : public std::streambuf {
    static constexpr std::size_t MaximumBytes = 8U * 1024U * 1024U;
    std::vector<char> bytes_;
    std::size_t extent_ = 0;
public:
    PersistenceBuffer() : bytes_(MaximumBytes) {
        setp(bytes_.data(), bytes_.data() + bytes_.size());
    }
    std::size_t size() const noexcept {
        return std::max(extent_, std::size_t(pptr() - pbase()));
    }
    void beginRead() noexcept {
        extent_ = size();
        setg(bytes_.data(), bytes_.data(), bytes_.data() + extent_);
    }
    bool exhausted() const noexcept { return gptr() == egptr(); }
protected:
    int_type overflow(int_type) override { return traits_type::eof(); }
    pos_type seekoff(off_type offset, std::ios_base::seekdir direction,
                     std::ios_base::openmode mode) override {
        extent_ = size();
        const bool input = bool(mode & std::ios_base::in);
        const bool output = bool(mode & std::ios_base::out);
        if (input == output) return pos_type(off_type(-1));
        const off_type current = input ? off_type(gptr() - eback())
                                       : off_type(pptr() - pbase());
        const off_type origin = direction == std::ios_base::beg ? 0
            : direction == std::ios_base::cur ? current : off_type(extent_);
        const off_type position = origin + offset;
        if (position < 0 || position > off_type(input ? extent_ : bytes_.size()))
            return pos_type(off_type(-1));
        if (input) setg(bytes_.data(), bytes_.data() + position, bytes_.data() + extent_);
        else {
            setp(bytes_.data(), bytes_.data() + bytes_.size());
            pbump(int(position));
        }
        return pos_type(position);
    }
    pos_type seekpos(pos_type position, std::ios_base::openmode mode) override {
        return seekoff(off_type(position), std::ios_base::beg, mode);
    }
};

inline bool SameTopologyCensus(const TopoDS_Shape& before,
                               const TopoDS_Shape& after) {
    if (before.IsNull() || after.IsNull()
        || before.ShapeType() != after.ShapeType()
        || before.Orientation() != after.Orientation()) return false;
    static constexpr std::array<TopAbs_ShapeEnum, 8> kinds = {
        TopAbs_COMPOUND, TopAbs_COMPSOLID, TopAbs_SOLID, TopAbs_SHELL,
        TopAbs_FACE, TopAbs_WIRE, TopAbs_EDGE, TopAbs_VERTEX,
    };
    for (const TopAbs_ShapeEnum kind : kinds) {
        TopTools_IndexedMapOfShape oldShapes, newShapes;
        TopExp::MapShapes(before, kind, oldShapes);
        TopExp::MapShapes(after, kind, newShapes);
        if (oldShapes.Extent() != newShapes.Extent()) return false;
    }
    return true;
}

inline TopoDS_Shape PersistenceMaterializeDetached(
    const TopoDS_Shape& shape) noexcept {
    try {
        if (!retained_part_boolean::IsOneValidForwardSolid(shape)) return {};
        PersistenceBuffer buffer;
        std::ostream writer(&buffer);
        BinTools::Write(shape, writer, Standard_False, Standard_False,
                        BinTools_FormatVersion_VERSION_4);
        if (!writer.good() || buffer.size() == 0) return {};
        buffer.beginRead();
        std::istream reader(&buffer);
        TopoDS_Shape restored;
        BinTools::Read(restored, reader);
        if (!reader.good() || !buffer.exhausted()
            || !retained_part_boolean::IsOneValidForwardSolid(restored)
            || !SameTopologyCensus(shape, restored)) return {};
        return restored;
    } catch (...) { return {}; }
}

inline retained_part_boolean::Operation NativeOperation(Operation operation) noexcept {
    switch (operation) {
        case Operation::Union: return retained_part_boolean::Operation::Union;
        case Operation::Subtract: return retained_part_boolean::Operation::Subtract;
        case Operation::Intersect: return retained_part_boolean::Operation::Intersect;
    }
    return retained_part_boolean::Operation::Union;
}

inline TopoDS_Shape BuildAnalyticPrism(const AnalyticPrismInput& input,
                                       double carrierMetersPerUnit) noexcept {
    try {
        if (!Valid(input) || !std::isfinite(carrierMetersPerUnit)
            || carrierMetersPerUnit <= 0) return {};
        const double scale = input.metersPerUnit / carrierMetersPerUnit;
        const gp_Quaternion rotation(input.rotationXYZW[0], input.rotationXYZW[1],
                                     input.rotationXYZW[2], input.rotationXYZW[3]);
        gp_Trsf placement;
        placement.SetRotation(rotation);
        placement.SetTranslationPart(gp_Vec(input.translation[0] * scale,
                                            input.translation[1] * scale,
                                            input.translation[2] * scale));
        TopoDS_Shape source = BRepPrimAPI_MakeBox(
            input.dimensions[0] * scale,
            input.dimensions[1] * scale,
            input.dimensions[2] * scale).Shape();
        if (source.IsNull()) return {};
        source = BRepBuilderAPI_Transform(source, placement, Standard_True).Shape();
        return retained_part_boolean::IsOneValidForwardSolid(source) ? source : TopoDS_Shape();
    } catch (...) { return {}; }
}

inline TopoDS_Shape AnalyticResultSolid(const TopoDS_Shape& value) noexcept {
    try {
        if (retained_part_boolean::IsOneValidForwardSolid(value)) return value;
        if (value.IsNull() || value.ShapeType() != TopAbs_COMPOUND
            || value.Orientation() != TopAbs_FORWARD) return {};
        TopoDS_Iterator child(value, Standard_True, Standard_True);
        if (!child.More()) return {};
        TopoDS_Shape solid = child.Value();
        child.Next();
        if (child.More()
            || !retained_part_boolean::IsOneValidForwardSolid(solid)) return {};
        return solid;
    } catch (...) { return {}; }
}

// Production analytic-prism-pair builder. Read-set values come from the owner;
// unlike FixtureReads(), no identifier or commitment is synthesized here.
inline AnalyticBuild BuildAnalytic(const AnalyticDefinition& definition,
                                   double carrierMetersPerUnit,
                                   const retained_part_boolean::OperandReadSet& captured,
                                   const retained_part_boolean::OperandReadSet& current) noexcept {
    AnalyticBuild result;
    try {
        if (!Valid(definition)) return result;
        for (std::size_t index = 0; index < 2; ++index) {
            result.sources[index] = BuildAnalyticPrism(
                definition.inputs[index], carrierMetersPerUnit);
            if (result.sources[index].IsNull()
                || !retained_part_boolean::ExactShapeBytes(
                    result.sources[index], result.sourceBytes[index])) return {};
        }
        result.candidate = retained_part_boolean::BuildDetachedCandidate(
            NativeOperation(definition.operation), result.sources[0], result.sources[1],
            Precision::Confusion(), captured, current, &result.evidence);
        if (result.candidate.admitted()) {
            const TopoDS_Shape solid = AnalyticResultSolid(result.candidate.solid);
            if (solid.IsNull()) {
                result.candidate.solid.Nullify();
                result.candidate.refusal = retained_part_boolean::Refusal::BuildFailed;
                return result;
            }
            result.candidate.solid = solid;
        }
        result.complete = result.candidate.admitted()
            && result.evidence.explicitDeterministicOptions
            && result.evidence.inputBytesUnchanged
            && result.evidence.historyObserved;
        if (!result.complete) return result;
        for (std::size_t index = 0; index < result.sources.size(); ++index) {
            result.sources[index] = PersistenceMaterializeDetached(result.sources[index]);
            if (result.sources[index].IsNull()
                || !retained_part_boolean::ExactShapeBytes(
                    result.sources[index], result.sourceBytes[index])) return {};
        }
        result.candidate.solid = PersistenceMaterializeDetached(result.candidate.solid);
        if (result.candidate.solid.IsNull()) return {};
        return result;
    } catch (...) { return {}; }
}

inline profile::Parameters ShellRecipe(ShellFixture fixture) {
    profile::Parameters value;
    value.metersPerUnit = 0.001;
    value.definition.plane = 0;
    value.definition.depth = 30;
    value.definition.revolve = false;
    value.definition.points = {
        gp_Pnt2d(0, 0), gp_Pnt2d(40, 0),
        gp_Pnt2d(40, 30), gp_Pnt2d(0, 30),
    };
    profile::ShellStep shell;
    shell.thickness = fixture == ShellFixture::Thin ? 1 : 3;
    shell.metersPerLocalUnit = 0.001;
    if (fixture == ShellFixture::S2) shell.openings = {4, 5};
    else if (fixture == ShellFixture::UpsideDown) shell.openings = {4};
    else shell.openings = {5};
    value.shells.push_back(std::move(shell));
    return value;
}

inline bool IsRectangularP1ShellRecipe(
    const profile::Parameters& value) noexcept {
    try {
        if (profile::SchemaFor(value) != 5 || value.shells.size() != 1
            || value.definition.revolve || value.definition.plane != 0
            || value.definition.circle || value.definition.curves
            || !value.definition.holes.empty() || value.definition.points.size() != 4
            || value.shells.front().thickness <= 0
            || value.shells.front().thickness >= 15
            || value.shells.front().metersPerLocalUnit != value.metersPerUnit) return false;
        const std::vector<int>& openings = value.shells.front().openings;
        return openings == std::vector<int>{5}
            || openings == std::vector<int>({4, 5});
    } catch (...) {
        return false;
    }
}

inline bool IsRectangularP1BaseRecipe(const profile::Parameters& value,
                                      bool requireShell) noexcept {
    try {
        if (value.definition.revolve || value.definition.plane != 0
            || value.definition.circle || value.definition.curves
            || !value.definition.holes.empty() || value.definition.points.size() != 4
            || !std::isfinite(value.definition.depth) || value.definition.depth <= 0)
            return false;
        const auto& p = value.definition.points;
        const bool orderedRectangle = p[0].Y() == p[1].Y()
            && p[1].X() == p[2].X() && p[2].Y() == p[3].Y()
            && p[3].X() == p[0].X() && p[1].X() > p[0].X()
            && p[2].Y() > p[1].Y();
        if (!orderedRectangle) return false;
        if (requireShell) return IsRectangularP1ShellRecipe(value);
        return value.shells.empty() && profile::SchemaFor(value) >= 1
            && profile::SchemaFor(value) <= 4;
    } catch (...) { return false; }
}

inline bool DecodeShellComposite(const composite_recipe::Definition& graph,
                                 ShellCompositeRequest& output) noexcept {
    output = {};
    try {
        if (graph.schemaVersion != 2 || graph.nodes.size() != 3) return false;
        const auto* feature = std::get_if<composite_recipe::FeatureNode>(
            &graph.nodes.back().value);
        if (!feature || feature->kind != composite_recipe::PartBooleanFeatureKind
            || feature->codecVersion != composite_recipe::PartBooleanShellFeatureCodec
            || feature->inputs.size() != 2
            || !Decode(feature->parameters, output.definition)
            || output.definition.inputs[0].family != InputFamily::ShellProfile
            || output.definition.inputs[1].family != InputFamily::AnalyticRectangularPrism)
            return false;
        for (std::size_t index = 0; index < 2; ++index) {
            const auto* source = std::get_if<composite_recipe::SourceNode>(
                &graph.nodes[index].value);
            std::vector<double> scalars;
            if (!source || source->node != feature->inputs[index]
                || source->node != output.definition.inputs[index].rootNode
                || source->original.sourceFeature
                    != output.definition.inputs[index].originalSourceFeature
                || source->recipe.kind != composite_recipe::RecipeKind::Profile
                || !composite_recipe::DecodeScalarRecipe(source->recipe, scalars)
                || !profile::Decode(scalars, output.profiles[index])) return false;
            output.placements[index] = source->inputToCarrier;
        }
        return IsRectangularP1BaseRecipe(output.profiles[0], true)
            && IsRectangularP1BaseRecipe(output.profiles[1], false);
    } catch (...) { output = {}; return false; }
}

inline TopoDS_Shape BuildProfileSource(const profile::Parameters& recipe,
                                       bool shell) noexcept {
    try {
        if (!IsRectangularP1BaseRecipe(recipe, shell)) return {};
        const auto& p = recipe.definition.points;
        TopoDS_Shape result = BRepPrimAPI_MakeBox(
            gp_Pnt(p[0].X(), p[0].Y(), 0),
            p[1].X() - p[0].X(), p[3].Y() - p[0].Y(),
            recipe.definition.depth).Shape();
        if (result.IsNull()) return {};
        if (recipe.constructionFrame) {
            gp_Trsf frame;
            if (!recipe.constructionFrame->Transform(frame)) return {};
            result = BRepBuilderAPI_Transform(result, frame, Standard_True).Shape();
        }
        if (shell) {
            std::atomic_bool cancelled{false};
            TopoDS_Shape shelled;
            if (!ReplayProfileShells(result, recipe, cancelled, shelled)) return {};
            result = shelled;
        }
        return retained_part_boolean::IsOneValidForwardSolid(result)
            ? result : TopoDS_Shape();
    } catch (...) { return {}; }
}

inline TopoDS_Shape ApplyInputPlacement(
    const TopoDS_Shape& source,
    const composite_recipe::InputPlacement& placement) noexcept {
    try {
        if (source.IsNull() || !composite_recipe::ValidPlacement(placement)) return {};
        const auto& m = placement.matrix;
        gp_Trsf transform;
        transform.SetValues(m[0], m[1], m[2], m[3],
                            m[4], m[5], m[6], m[7],
                            m[8], m[9], m[10], m[11]);
        TopoDS_Shape result = BRepBuilderAPI_Transform(
            source, transform, Standard_True).Shape();
        return retained_part_boolean::IsOneValidForwardSolid(result)
            ? result : TopoDS_Shape();
    } catch (...) { return {}; }
}

inline bool ClassifySourceFaces(const TopoDS_Shape& source,
                                const profile::Parameters& recipe,
                                bool shell,
                                std::vector<SourceFaceClass>& output) noexcept {
    output.clear();
    try {
        if (!retained_part_boolean::IsOneValidForwardSolid(source)) return false;
        GProp_GProps volume;
        BRepGProp::VolumeProperties(source, volume);
        const gp_Pnt centre = volume.CentreOfMass();
        gp_Dir extrusion(0, 0, 1);
        if (recipe.constructionFrame) {
            gp_Trsf frame;
            if (!recipe.constructionFrame->Transform(frame)) return false;
            extrusion.Transform(frame);
        }
        for (TopExp_Explorer it(source, TopAbs_FACE); it.More(); it.Next()) {
            const TopoDS_Face face = TopoDS::Face(it.Current());
            BRepAdaptor_Surface surface(face, Standard_True);
            if (surface.GetType() != GeomAbs_Plane) return false;
            GProp_GProps area;
            BRepGProp::SurfaceProperties(face, area);
            if (!std::isfinite(area.Mass()) || area.Mass() <= 0) return false;
            SourceFaceClass classified;
            classified.face = face;
            if (!shell) {
                classified.kind = RegionKind::ToolBoundary;
            } else {
                gp_Dir normal = surface.Plane().Axis().Direction();
                if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
                const gp_Vec fromCentre(centre, area.CentreOfMass());
                const double axial = normal.Dot(extrusion);
                if (std::abs(axial) >= 1.0 - 1e-9) {
                    classified.kind = fromCentre.Dot(gp_Vec(extrusion)) > 0
                        ? RegionKind::Rim : RegionKind::Floor;
                } else {
                    const gp_Vec radial = fromCentre
                        - gp_Vec(extrusion) * fromCentre.Dot(gp_Vec(extrusion));
                    classified.kind = normal.Dot(radial) >= 0
                        ? RegionKind::OuterWall : RegionKind::InnerWall;
                }
            }
            output.push_back(std::move(classified));
        }
        return !output.empty();
    } catch (...) { output.clear(); return false; }
}

inline bool ApplyInputPlacement(
    const TopoDS_Shape& source,
    const std::vector<SourceFaceClass>& sourceFaces,
    const composite_recipe::InputPlacement& placement,
    TopoDS_Shape& result,
    std::vector<SourceFaceClass>& resultFaces) noexcept {
    result.Nullify(); resultFaces.clear();
    try {
        if (source.IsNull() || !composite_recipe::ValidPlacement(placement)) return false;
        const auto& m = placement.matrix;
        gp_Trsf transform;
        transform.SetValues(m[0], m[1], m[2], m[3],
                            m[4], m[5], m[6], m[7],
                            m[8], m[9], m[10], m[11]);
        BRepBuilderAPI_Transform builder(source, transform, Standard_True);
        result = builder.Shape();
        if (!retained_part_boolean::IsOneValidForwardSolid(result)) return false;
        for (const SourceFaceClass& value : sourceFaces) {
            const TopoDS_Shape mapped = builder.ModifiedShape(value.face);
            if (mapped.IsNull() || mapped.ShapeType() != TopAbs_FACE) return false;
            resultFaces.push_back({TopoDS::Face(mapped), value.kind});
        }
        return resultFaces.size() == sourceFaces.size();
    } catch (...) { result.Nullify(); resultFaces.clear(); return false; }
}

template <class Builder>
inline bool CollectShellBooleanHistory(
    Builder& boolean,
    const std::array<TopoDS_Shape, 2>& sources,
    const std::array<std::vector<SourceFaceClass>, 2>& sourceFaces,
    const TopoDS_Shape& candidate,
    std::vector<FaceHistoryRelation>& output) noexcept {
    output.clear();
    try {
        TopTools_ListOfShape arguments, tools;
        arguments.Append(sources[0]); tools.Append(sources[1]);
        boolean.SetArguments(arguments); boolean.SetTools(tools);
        boolean.SetRunParallel(Standard_False);
        boolean.SetNonDestructive(Standard_True);
        boolean.SetFuzzyValue(Precision::Confusion());
        boolean.SetUseOBB(Standard_True);
        boolean.SetCheckInverted(Standard_True);
        boolean.Build();
        if (!boolean.IsDone() || boolean.HasErrors()) return false;
        std::string candidateBytes, independentBytes;
        if (!retained_part_boolean::ExactShapeBytes(candidate, candidateBytes)
            || !retained_part_boolean::ExactShapeBytes(boolean.Shape(), independentBytes)
            || candidateBytes != independentBytes) return false;

        std::map<std::string, TopoDS_Face> finalFaces;
        for (TopExp_Explorer it(candidate, TopAbs_FACE); it.More(); it.Next()) {
            std::string key;
            const TopoDS_Face face = TopoDS::Face(it.Current());
            if (!retained_part_boolean::ExactShapeBytes(face, key)
                || !finalFaces.emplace(key, face).second) return false;
        }
        std::set<std::tuple<std::string, std::uint8_t, unsigned char, bool>> seen;
        const auto append = [&](const TopoDS_Shape& shape, std::uint8_t input,
                                RegionKind kind, bool generated) {
            if (shape.IsNull() || shape.ShapeType() != TopAbs_FACE) return true;
            std::string key;
            if (!retained_part_boolean::ExactShapeBytes(shape, key)) return false;
            if (!finalFaces.count(key)) {
                // OCCT history returns a face without necessarily carrying its
                // final result orientation. Resolve it in this builder's result
                // before using the exact oriented result bytes as its key.
                bool found = false;
                for (TopExp_Explorer it(boolean.Shape(), TopAbs_FACE); it.More(); it.Next()) {
                    if (!it.Current().IsSame(shape)) continue;
                    if (found) return false;
                    found = true;
                    if (!retained_part_boolean::ExactShapeBytes(it.Current(), key)
                        || !finalFaces.count(key)) return false;
                }
                if (!found) return true;
            }
            const auto token = std::make_tuple(key, input,
                static_cast<unsigned char>(kind), generated);
            if (seen.insert(token).second)
                output.push_back({std::move(key), input, kind, generated});
            return true;
        };
        for (std::uint8_t input = 0; input < 2; ++input) {
            for (const SourceFaceClass& source : sourceFaces[input]) {
                if (!append(source.face, input, source.kind, false)) return false;
                for (TopTools_ListIteratorOfListOfShape it(boolean.Modified(source.face));
                     it.More(); it.Next())
                    if (!append(it.Value(), input, source.kind, false)) return false;
                for (TopTools_ListIteratorOfListOfShape it(boolean.Generated(source.face));
                     it.More(); it.Next())
                    if (!append(it.Value(), input, source.kind, true)) return false;
            }
        }
        std::set<std::string> related;
        for (const FaceHistoryRelation& value : output) related.insert(value.faceKey);
        return !finalFaces.empty() && related.size() == finalFaces.size();
    } catch (...) { output.clear(); return false; }
}

inline ShellCompositeBuild BuildShellComposite(
    const ShellCompositeRequest& request,
    const retained_part_boolean::OperandReadSet& captured,
    const retained_part_boolean::OperandReadSet& current) noexcept {
    ShellCompositeBuild result;
    try {
        if (!Valid(request.definition)
            || request.definition.inputs[0].family != InputFamily::ShellProfile
            || request.definition.inputs[1].family != InputFamily::AnalyticRectangularPrism)
            return result;
        std::array<std::vector<SourceFaceClass>, 2> sourceFaces;
        for (std::size_t index = 0; index < 2; ++index) {
            const TopoDS_Shape local = BuildProfileSource(
                request.profiles[index], index == 0);
            std::vector<SourceFaceClass> localFaces;
            if (!ClassifySourceFaces(local, request.profiles[index], index == 0, localFaces)
                || !ApplyInputPlacement(local, localFaces, request.placements[index],
                                        result.sources[index], sourceFaces[index])) return {};
            if (result.sources[index].IsNull()
                || !retained_part_boolean::ExactShapeBytes(
                    result.sources[index], result.sourceBytes[index])) return {};
        }
        result.candidate = retained_part_boolean::BuildDetachedCandidate(
            NativeOperation(request.definition.operation), result.sources[0],
            result.sources[1], Precision::Confusion(), captured, current,
            &result.evidence);
        if (result.candidate.admitted()) {
            if (request.definition.operation == Operation::Union) {
                BRepAlgoAPI_Fuse boolean;
                result.historyResultExact = CollectShellBooleanHistory(
                    boolean, result.sources, sourceFaces, result.candidate.solid,
                    result.faceHistory);
            } else if (request.definition.operation == Operation::Subtract) {
                BRepAlgoAPI_Cut boolean;
                result.historyResultExact = CollectShellBooleanHistory(
                    boolean, result.sources, sourceFaces, result.candidate.solid,
                    result.faceHistory);
            } else {
                BRepAlgoAPI_Common boolean;
                result.historyResultExact = CollectShellBooleanHistory(
                    boolean, result.sources, sourceFaces, result.candidate.solid,
                    result.faceHistory);
            }
        }
        result.complete = result.candidate.admitted()
            && result.evidence.explicitDeterministicOptions
            && result.evidence.inputBytesUnchanged
            && result.evidence.historyObserved
            && result.historyResultExact;
        if (!result.complete) return result;
        result.candidate.solid = AnalyticResultSolid(result.candidate.solid);
        if (result.candidate.solid.IsNull()) return {};
        const TopoDS_Shape persisted = PersistenceMaterializeDetached(result.candidate.solid);
        if (persisted.IsNull()) return {};
        std::map<std::string, std::string> persistedKeys;
        std::set<std::string> uniquePersistedKeys;
        TopExp_Explorer originalFace(result.candidate.solid, TopAbs_FACE);
        TopExp_Explorer persistedFace(persisted, TopAbs_FACE);
        for (; originalFace.More() && persistedFace.More(); originalFace.Next(), persistedFace.Next()) {
            std::string originalKey, persistedKey;
            if (originalFace.Current().Orientation() != persistedFace.Current().Orientation()
                || !retained_part_boolean::ExactShapeBytes(originalFace.Current(), originalKey)
                || !retained_part_boolean::ExactShapeBytes(persistedFace.Current(), persistedKey)
                || !persistedKeys.emplace(originalKey, persistedKey).second
                || !uniquePersistedKeys.insert(persistedKey).second) return {};
        }
        if (originalFace.More() || persistedFace.More() || persistedKeys.empty()) return {};
        for (auto& relation : result.faceHistory) {
            const auto key = persistedKeys.find(relation.faceKey);
            if (key == persistedKeys.end()) return {};
            relation.faceKey = key->second;
        }
        result.candidate.solid = persisted;
        for (std::size_t index = 0; index < result.sources.size(); ++index) {
            result.sources[index] = PersistenceMaterializeDetached(result.sources[index]);
            if (result.sources[index].IsNull()
                || !retained_part_boolean::ExactShapeBytes(result.sources[index], result.sourceBytes[index])) return {};
        }
        return result;
    } catch (...) { return {}; }
}

inline ShellBuild BuildShell(ShellFixture fixture) noexcept {
    ShellBuild result;
    try {
        result.recipe = ShellRecipe(fixture);
        if (!profile::Encode(result.recipe, result.canonicalRecipe)) return result;
        profile::Parameters decoded;
        std::vector<double> reencoded;
        if (!profile::Decode(result.canonicalRecipe, decoded)
            || !profile::Encode(decoded, reencoded)
            || reencoded != result.canonicalRecipe) return result;
        result.base = BRepPrimAPI_MakeBox(gp_Pnt(0, 0, 0), 40, 30, 30).Shape();
        std::atomic_bool cancelled{false};
        result.replayed = ReplayProfileShells(
            result.base, decoded, cancelled, result.shell)
            && !result.shell.IsNull();
        return result;
    } catch (...) {
        return {};
    }
}

inline retained_recipe::UUID Identifier(std::uint8_t seed) noexcept {
    retained_recipe::UUID value{};
    value.fill(seed);
    return value;
}

inline retained_recipe::Digest Digest(std::uint8_t seed) noexcept {
    retained_recipe::Digest value{};
    value.fill(seed);
    return value;
}

inline retained_recipe::DependencyRead Read(std::uint8_t seed) noexcept {
    retained_recipe::DependencyRead value;
    value.locator.owner = {Identifier(seed), Identifier(seed + 1), Identifier(seed + 2)};
    value.locator.node = Identifier(seed + 3);
    value.locator.sourceFeature = Identifier(seed + 4);
    value.geometry = Digest(seed + 5);
    value.recipe = Digest(seed + 6);
    value.placement = Digest(seed + 7);
    value.material = Digest(seed + 8);
    value.groups = Digest(seed + 9);
    return value;
}

inline retained_part_boolean::OperandReadSet FixtureReads() noexcept {
    return {Read(1), Read(21)};
}

inline TopoDS_Shape Box(const std::array<double, 6>& bounds) {
    return BRepPrimAPI_MakeBox(
        gp_Pnt(bounds[0], bounds[1], bounds[2]),
        bounds[3] - bounds[0], bounds[4] - bounds[1],
        bounds[5] - bounds[2]).Shape();
}

inline BooleanBuild BuildBoolean(
    retained_part_boolean::Operation operation,
    const TopoDS_Shape& left,
    const TopoDS_Shape& right) noexcept {
    BooleanBuild result;
    const auto reads = FixtureReads();
    result.candidate = retained_part_boolean::BuildDetachedCandidate(
        operation, left, right, 1e-7, reads, reads, &result.evidence);
    return result;
}

} // namespace core3d::part_boolean::build
