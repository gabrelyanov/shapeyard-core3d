#pragma once

// Detached P1b builder.  It reuses ShellOperationController's retained replay
// seam and A1's bounded Boolean helper.  It owns no label or document command.
#include "RetainedPartBoolean.hxx"
#include "../UI/ShellOperationController.hpp"
#include <BinTools.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <TopExp.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <gp_Pnt.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <algorithm>
#include <atomic>
#include <array>
#include <cstdint>
#include <istream>
#include <ostream>
#include <streambuf>
#include <string>
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
