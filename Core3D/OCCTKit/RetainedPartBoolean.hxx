#pragma once

// A1a detached geometry gate. It has no OCAF label, command, AIS dependency,
// or mutation authority. A later owner must recheck the full semantic read set
// before it can use an admitted shape in one atomic retained-owner command.
// It never routes or changes the existing general UI Boolean geometry worker.
// Agentic routing remains closed until that retained owner is implemented.
#include "RetainedRecipeAdmission.hxx"
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Shape.hxx>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace core3d::retained_part_boolean {

enum class Operation : std::uint8_t {
    Union = 1,
    Subtract = 2,
    Intersect = 3,
};

enum class Refusal : std::uint8_t {
    None = 0,
    Admission,
    InvalidInput,
    StaleLeftSource,
    StaleRightSource,
    BuildFailed,
    EmptyResult,
    TangentOnly,
    Disconnected,
    NoInteriorOverlap,
};

struct OperandReadSet {
    retained_recipe::DependencyRead leftSource;
    retained_recipe::DependencyRead rightSource;
};

struct Candidate {
    Refusal refusal = Refusal::Admission;
    TopoDS_Shape solid;

    bool admitted() const noexcept { return refusal == Refusal::None; }
};

inline bool IsOneValidForwardSolid(const TopoDS_Shape& value) noexcept {
    try {
        return !value.IsNull() && value.ShapeType() == TopAbs_SOLID
            && value.Orientation() == TopAbs_FORWARD
            && BRepCheck_Analyzer(value).IsValid();
    } catch (...) {
        return false;
    }
}

inline std::size_t SolidCount(const TopoDS_Shape& value) noexcept {
    try {
        std::size_t count = 0;
        for (TopExp_Explorer it(value, TopAbs_SOLID); it.More(); it.Next()) {
            if (++count > 1) break;
        }
        return count;
    } catch (...) {
        return 0;
    }
}

inline double Volume(const TopoDS_Shape& value) noexcept {
    try {
        GProp_GProps properties;
        BRepGProp::VolumeProperties(value, properties);
        const double volume = properties.Mass();
        return std::isfinite(volume) && volume >= 0 ? volume : -1;
    } catch (...) {
        return -1;
    }
}

inline bool ValidOperandRead(
    const retained_recipe::DependencyRead& value) noexcept {
    return retained_recipe::Valid(value.locator)
        && retained_recipe::Nonzero(value.geometry)
        && retained_recipe::Nonzero(value.recipe)
        && retained_recipe::Nonzero(value.placement)
        && retained_recipe::Nonzero(value.material)
        && retained_recipe::Nonzero(value.groups);
}

inline bool SameOperandRead(
    const retained_recipe::DependencyRead& left,
    const retained_recipe::DependencyRead& right) noexcept {
    return left.locator == right.locator
        && left.geometry == right.geometry
        && left.recipe == right.recipe
        && left.placement == right.placement
        && left.material == right.material
        && left.groups == right.groups;
}

// The first A1 slice accepts exactly two analytic source recipes. Recursive
// retained-Boolean sources stay excluded until the atomic retained owner exists.
inline bool HasExactlyTwoAnalyticSourceRecipes(
    const retained_recipe::OwnerSnapshot& snapshot) noexcept {
    if (snapshot.sources.size() != 2) return false;
    for (const auto& source : snapshot.sources) {
        using Kind = composite_recipe::RecipeKind;
        if (source.recipe.kind != Kind::Profile
            && source.recipe.kind != Kind::Enclosure
            && source.recipe.kind != Kind::RectangularLoft) return false;
    }
    return true;
}

// Geometry-only phase. Operand identity is semantic (left/right source) and
// UUID/digest based; no array position or topology ordinal is retained.
inline Candidate BuildDetachedCandidate(
    Operation operation,
    const TopoDS_Shape& leftSourceShape,
    const TopoDS_Shape& rightSourceShape,
    double tolerance,
    const OperandReadSet& capturedReads,
    const OperandReadSet& currentReads) noexcept {
    Candidate result;
    try {
        if (!ValidOperandRead(capturedReads.leftSource)
            || !ValidOperandRead(capturedReads.rightSource)
            || !ValidOperandRead(currentReads.leftSource)
            || !ValidOperandRead(currentReads.rightSource)) {
            result.refusal = Refusal::Admission;
            return result;
        }
        if (!SameOperandRead(capturedReads.leftSource,
                             currentReads.leftSource)) {
            result.refusal = Refusal::StaleLeftSource;
            return result;
        }
        if (!SameOperandRead(capturedReads.rightSource,
                             currentReads.rightSource)) {
            result.refusal = Refusal::StaleRightSource;
            return result;
        }
        if (!std::isfinite(tolerance) || tolerance <= 0
            || !IsOneValidForwardSolid(leftSourceShape)
            || !IsOneValidForwardSolid(rightSourceShape)) {
            result.refusal = Refusal::InvalidInput;
            return result;
        }

        const double volumeTolerance = tolerance * tolerance * tolerance;
        BRepAlgoAPI_Common common(leftSourceShape, rightSourceShape);
        if (!common.IsDone()) {
            result.refusal = Refusal::BuildFailed;
            return result;
        }
        const double commonVolume = common.Shape().IsNull()
            ? 0 : Volume(common.Shape());
        if (commonVolume < 0) {
            result.refusal = Refusal::BuildFailed;
            return result;
        }
        if (commonVolume <= volumeTolerance) {
            BRepExtrema_DistShapeShape distance(
                leftSourceShape, rightSourceShape);
            distance.Perform();
            if (!distance.IsDone()) {
                result.refusal = Refusal::BuildFailed;
                return result;
            }
            if (distance.Value() <= tolerance) {
                result.refusal = Refusal::TangentOnly;
                return result;
            }
            result.refusal = operation == Operation::Intersect
                ? Refusal::EmptyResult
                : operation == Operation::Union
                    ? Refusal::Disconnected
                    : Refusal::NoInteriorOverlap;
            return result;
        }

        TopoDS_Shape built;
        if (operation == Operation::Union) {
            BRepAlgoAPI_Fuse boolean(
                leftSourceShape, rightSourceShape);
            if (!boolean.IsDone()) {
                result.refusal = Refusal::BuildFailed;
                return result;
            }
            built = boolean.Shape();
        } else if (operation == Operation::Subtract) {
            BRepAlgoAPI_Cut boolean(
                leftSourceShape, rightSourceShape);
            if (!boolean.IsDone()) {
                result.refusal = Refusal::BuildFailed;
                return result;
            }
            built = boolean.Shape();
        } else {
            built = common.Shape();
        }
        if (built.IsNull() || Volume(built) <= volumeTolerance) {
            result.refusal = Refusal::EmptyResult;
            return result;
        }
        if (!BRepCheck_Analyzer(built).IsValid()) {
            result.refusal = Refusal::BuildFailed;
            return result;
        }
        if (SolidCount(built) != 1) {
            result.refusal = Refusal::Disconnected;
            return result;
        }
        result.refusal = Refusal::None;
        result.solid = built;
        return result;
    } catch (...) {
        result = {};
        result.refusal = Refusal::BuildFailed;
        return result;
    }
}

} // namespace core3d::retained_part_boolean
