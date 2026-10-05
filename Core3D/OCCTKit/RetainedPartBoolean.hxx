#pragma once

// A1a detached geometry gate. It has no OCAF label, command, AIS dependency,
// or mutation authority. A later owner must recheck the full semantic read set
// before it can use an admitted shape in one atomic retained-owner command.
// It never routes or changes the existing general UI Boolean geometry worker.
// Agentic routing remains closed until that retained owner is implemented.
#include "RetainedRecipeAdmission.hxx"
#include "RetainedTopologyBudget.hxx"
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <GProp_GProps.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_FormatVersion.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS_Shape.hxx>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <locale>
#include <sstream>
#include <string>

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
    // A1-OWNER-budget counted route only: sticky budget exhaustion or caller
    // cancellation observed before a charged pass. Never produced by the
    // uncounted entry points and never reclassified into another refusal.
    Budget,
    Cancelled,
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

// Optional, detached evidence only.  It exposes no face identity and grants no
// persistence or mutation authority.  Counts merely prove that native history
// was queried for both inputs before independent correspondence validation.
struct CandidateEvidence {
    bool explicitDeterministicOptions = false;
    bool inputBytesUnchanged = false;
    bool historyObserved = false;
    std::size_t leftHistoryRelations = 0;
    std::size_t rightHistoryRelations = 0;
};

inline bool ExactShapeBytes(const TopoDS_Shape& shape,
                            std::string& bytes) noexcept {
    bytes.clear();
    try {
        std::ostringstream stream;
        stream.imbue(std::locale::classic());
        BRepTools::Write(shape, stream, Standard_False, Standard_False,
                         TopTools_FormatVersion_VERSION_3);
        if (!stream.good()) return false;
        bytes = stream.str();
        return !bytes.empty();
    } catch (...) {
        bytes.clear();
        return false;
    }
}

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
    const OperandReadSet& currentReads,
    CandidateEvidence* evidence = nullptr) noexcept {
    Candidate result;
    if (evidence != nullptr) *evidence = {};
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

        std::string leftBefore, rightBefore;
        if (evidence != nullptr
            && (!ExactShapeBytes(leftSourceShape, leftBefore)
                || !ExactShapeBytes(rightSourceShape, rightBefore))) {
            result.refusal = Refusal::BuildFailed;
            return result;
        }
        TopTools_ListOfShape arguments, tools;
        arguments.Append(leftSourceShape);
        tools.Append(rightSourceShape);
        const auto configureAndBuild = [&](auto& builder) {
            builder.SetArguments(arguments);
            builder.SetTools(tools);
            builder.SetRunParallel(Standard_False);
            builder.SetNonDestructive(Standard_True);
            builder.SetFuzzyValue(tolerance);
            builder.SetUseOBB(Standard_True);
            builder.SetCheckInverted(Standard_True);
            builder.Build();
            if (evidence != nullptr)
                evidence->explicitDeterministicOptions = true;
            return builder.IsDone() && !builder.HasErrors();
        };
        const auto observeHistory = [&](auto& builder) {
            if (evidence == nullptr) return;
            const auto charge = [&](const TopoDS_Shape& source) {
                std::size_t count = 0;
                for (TopExp_Explorer it(source, TopAbs_FACE); it.More(); it.Next()) {
                    count += std::size_t(builder.Modified(it.Current()).Extent());
                    count += std::size_t(builder.Generated(it.Current()).Extent());
                    if (builder.IsDeleted(it.Current())) ++count;
                }
                return count;
            };
            evidence->leftHistoryRelations = charge(leftSourceShape);
            evidence->rightHistoryRelations = charge(rightSourceShape);
            evidence->historyObserved = true;
        };

        const double volumeTolerance = tolerance * tolerance * tolerance;
        BRepAlgoAPI_Common common;
        if (!configureAndBuild(common)) {
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
            BRepAlgoAPI_Fuse boolean;
            if (!configureAndBuild(boolean)) {
                result.refusal = Refusal::BuildFailed;
                return result;
            }
            built = boolean.Shape();
            observeHistory(boolean);
        } else if (operation == Operation::Subtract) {
            BRepAlgoAPI_Cut boolean;
            if (!configureAndBuild(boolean)) {
                result.refusal = Refusal::BuildFailed;
                return result;
            }
            built = boolean.Shape();
            observeHistory(boolean);
        } else {
            built = common.Shape();
            observeHistory(common);
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
        if (evidence != nullptr) {
            std::string leftAfter, rightAfter;
            evidence->inputBytesUnchanged =
                ExactShapeBytes(leftSourceShape, leftAfter)
                && ExactShapeBytes(rightSourceShape, rightAfter)
                && leftBefore == leftAfter && rightBefore == rightAfter;
        }
        return result;
    } catch (...) {
        result = {};
        result.refusal = Refusal::BuildFailed;
        return result;
    }
}

// A1-OWNER-budget (R4 / D390 / D392): counted detached analytic replay route.
// The overloads below thread the caller's already-charged operation
// continuation (one shared retained_topology_budget::Counter plus its
// cancellation flag) through every pass of the analytic operand route. Each
// traversal/copy/serialization pass is charged BEFORE its work runs; a denied
// charge is sticky on the counter and surfaces as Refusal::Budget, and an
// observed cancellation surfaces as Refusal::Cancelled — neither is ever
// reclassified into another refusal. The uncounted entry points above keep
// their exact legacy behavior; no proof or admission domain changes here.
namespace tb = core3d::retained_topology_budget;

// One measured occurrence pass over a shape, charged before the pass's work
// (serialization, validity, volume, census or kernel reservation) runs.
inline tb::WalkStatus ChargeAnalyticPass(const TopoDS_Shape& shape,
    tb::Counter& budget, const std::atomic_bool& cancelled,
    tb::Site site) noexcept {
    if (cancelled.load()) return tb::WalkStatus::Cancelled;
    return tb::ChargeTraversal(shape, budget, cancelled, site);
}

inline tb::WalkStatus ExactShapeBytesCounted(const TopoDS_Shape& shape,
    std::string& bytes, tb::Counter& budget, const std::atomic_bool& cancelled,
    tb::Site site) noexcept {
    bytes.clear();
    const tb::WalkStatus walk = ChargeAnalyticPass(shape, budget, cancelled, site);
    if (walk != tb::WalkStatus::Completed) return walk;
    return ExactShapeBytes(shape, bytes) ? tb::WalkStatus::Completed
        : tb::WalkStatus::Failed;
}

inline tb::WalkStatus VolumeCounted(const TopoDS_Shape& shape, double& volume,
    tb::Counter& budget, const std::atomic_bool& cancelled,
    tb::Site site) noexcept {
    volume = -1;
    const tb::WalkStatus walk = ChargeAnalyticPass(shape, budget, cancelled, site);
    if (walk != tb::WalkStatus::Completed) return walk;
    volume = Volume(shape);
    return tb::WalkStatus::Completed;
}

inline tb::WalkStatus ValidSolidCounted(const TopoDS_Shape& shape, bool& valid,
    tb::Counter& budget, const std::atomic_bool& cancelled,
    tb::Site site) noexcept {
    valid = false;
    const tb::WalkStatus walk = ChargeAnalyticPass(shape, budget, cancelled, site);
    if (walk != tb::WalkStatus::Completed) return walk;
    valid = IsOneValidForwardSolid(shape);
    return tb::WalkStatus::Completed;
}

// Counted analytic Boolean build. Identical geometry, options and refusal
// domains to BuildDetachedCandidate; the only additions are the per-pass
// charges on the shared continuation and the sticky Budget/Cancelled
// propagation. Cancellation is observed before each pass and outranks budget.
inline tb::WalkStatus BuildDetachedCandidateCounted(
    Operation operation,
    const TopoDS_Shape& leftSourceShape,
    const TopoDS_Shape& rightSourceShape,
    double tolerance,
    const OperandReadSet& capturedReads,
    const OperandReadSet& currentReads,
    tb::Counter& budget,
    const std::atomic_bool& cancelled,
    Candidate& result,
    CandidateEvidence* evidence = nullptr) noexcept {
    result = {};
    if (evidence != nullptr) *evidence = {};
    const auto classify = [&result](tb::WalkStatus walk) {
        result.refusal = walk == tb::WalkStatus::Cancelled ? Refusal::Cancelled
            : walk == tb::WalkStatus::BudgetDenied ? Refusal::Budget
            : Refusal::BuildFailed;
        return walk;
    };
    try {
        if (cancelled.load()) {
            result.refusal = Refusal::Cancelled;
            return tb::WalkStatus::Cancelled;
        }
        if (!ValidOperandRead(capturedReads.leftSource)
            || !ValidOperandRead(capturedReads.rightSource)
            || !ValidOperandRead(currentReads.leftSource)
            || !ValidOperandRead(currentReads.rightSource)) {
            result.refusal = Refusal::Admission;
            return tb::WalkStatus::Completed;
        }
        if (!SameOperandRead(capturedReads.leftSource,
                             currentReads.leftSource)) {
            result.refusal = Refusal::StaleLeftSource;
            return tb::WalkStatus::Completed;
        }
        if (!SameOperandRead(capturedReads.rightSource,
                             currentReads.rightSource)) {
            result.refusal = Refusal::StaleRightSource;
            return tb::WalkStatus::Completed;
        }
        if (!std::isfinite(tolerance) || tolerance <= 0) {
            result.refusal = Refusal::InvalidInput;
            return tb::WalkStatus::Completed;
        }
        // Both operand validity passes are charged before they run.
        bool leftValid = false, rightValid = false;
        tb::WalkStatus walk = ValidSolidCounted(leftSourceShape, leftValid,
            budget, cancelled, tb::Site::C25AnalyticInput);
        if (walk != tb::WalkStatus::Completed) return classify(walk);
        walk = ValidSolidCounted(rightSourceShape, rightValid,
            budget, cancelled, tb::Site::C25AnalyticInput);
        if (walk != tb::WalkStatus::Completed) return classify(walk);
        if (!leftValid || !rightValid) {
            result.refusal = Refusal::InvalidInput;
            return tb::WalkStatus::Completed;
        }

        std::string leftBefore, rightBefore;
        if (evidence != nullptr) {
            walk = ExactShapeBytesCounted(leftSourceShape, leftBefore,
                budget, cancelled, tb::Site::C25AnalyticInput);
            if (walk == tb::WalkStatus::Cancelled
                || walk == tb::WalkStatus::BudgetDenied) return classify(walk);
            const bool leftSerial = walk == tb::WalkStatus::Completed
                && !leftBefore.empty();
            walk = ExactShapeBytesCounted(rightSourceShape, rightBefore,
                budget, cancelled, tb::Site::C25AnalyticInput);
            if (walk == tb::WalkStatus::Cancelled
                || walk == tb::WalkStatus::BudgetDenied) return classify(walk);
            const bool rightSerial = walk == tb::WalkStatus::Completed
                && !rightBefore.empty();
            if (!leftSerial || !rightSerial) {
                result.refusal = Refusal::BuildFailed;
                return tb::WalkStatus::Completed;
            }
        }
        TopTools_ListOfShape arguments, tools;
        arguments.Append(leftSourceShape);
        tools.Append(rightSourceShape);
        const auto configureAndBuild = [&](auto& builder) {
            builder.SetArguments(arguments);
            builder.SetTools(tools);
            builder.SetRunParallel(Standard_False);
            builder.SetNonDestructive(Standard_True);
            builder.SetFuzzyValue(tolerance);
            builder.SetUseOBB(Standard_True);
            builder.SetCheckInverted(Standard_True);
            builder.Build();
            if (evidence != nullptr)
                evidence->explicitDeterministicOptions = true;
            return builder.IsDone() && !builder.HasErrors();
        };
        const auto observeHistory = [&](auto& builder) {
            if (evidence == nullptr) return;
            const auto charge = [&](const TopoDS_Shape& source) {
                std::size_t count = 0;
                for (TopExp_Explorer it(source, TopAbs_FACE); it.More(); it.Next()) {
                    count += std::size_t(builder.Modified(it.Current()).Extent());
                    count += std::size_t(builder.Generated(it.Current()).Extent());
                    if (builder.IsDeleted(it.Current())) ++count;
                }
                return count;
            };
            evidence->leftHistoryRelations = charge(leftSourceShape);
            evidence->rightHistoryRelations = charge(rightSourceShape);
            evidence->historyObserved = true;
        };
        // The history scan walks both operand face sets; reserve each pass
        // before the relation scan runs.
        const auto observeHistoryCounted = [&](auto& builder) -> tb::WalkStatus {
            if (evidence == nullptr) return tb::WalkStatus::Completed;
            tb::WalkStatus walked = ChargeAnalyticPass(leftSourceShape,
                budget, cancelled, tb::Site::C26AnalyticBoolean);
            if (walked != tb::WalkStatus::Completed) return walked;
            walked = ChargeAnalyticPass(rightSourceShape,
                budget, cancelled, tb::Site::C26AnalyticBoolean);
            if (walked != tb::WalkStatus::Completed) return walked;
            observeHistory(builder);
            return tb::WalkStatus::Completed;
        };

        const double volumeTolerance = tolerance * tolerance * tolerance;
        // The overlap pass is one build stage; both kernel input passes are
        // reserved before the kernel runs.
        if (cancelled.load()) {
            result.refusal = Refusal::Cancelled;
            return tb::WalkStatus::Cancelled;
        }
        if (!budget.beginStage(tb::Site::C26AnalyticBoolean)) {
            result.refusal = Refusal::Budget;
            return tb::WalkStatus::BudgetDenied;
        }
        walk = ChargeAnalyticPass(leftSourceShape, budget, cancelled,
            tb::Site::C26AnalyticBoolean);
        if (walk != tb::WalkStatus::Completed) return classify(walk);
        walk = ChargeAnalyticPass(rightSourceShape, budget, cancelled,
            tb::Site::C26AnalyticBoolean);
        if (walk != tb::WalkStatus::Completed) return classify(walk);
        BRepAlgoAPI_Common common;
        if (!configureAndBuild(common)) {
            result.refusal = Refusal::BuildFailed;
            return tb::WalkStatus::Completed;
        }
        double commonVolume = 0;
        if (!common.Shape().IsNull()) {
            walk = VolumeCounted(common.Shape(), commonVolume, budget, cancelled,
                tb::Site::C26AnalyticBoolean);
            if (walk != tb::WalkStatus::Completed) return classify(walk);
        }
        if (commonVolume < 0) {
            result.refusal = Refusal::BuildFailed;
            return tb::WalkStatus::Completed;
        }
        if (commonVolume <= volumeTolerance) {
            walk = ChargeAnalyticPass(leftSourceShape, budget, cancelled,
                tb::Site::C26AnalyticBoolean);
            if (walk != tb::WalkStatus::Completed) return classify(walk);
            walk = ChargeAnalyticPass(rightSourceShape, budget, cancelled,
                tb::Site::C26AnalyticBoolean);
            if (walk != tb::WalkStatus::Completed) return classify(walk);
            BRepExtrema_DistShapeShape distance(
                leftSourceShape, rightSourceShape);
            distance.Perform();
            if (!distance.IsDone()) {
                result.refusal = Refusal::BuildFailed;
                return tb::WalkStatus::Completed;
            }
            if (distance.Value() <= tolerance) {
                result.refusal = Refusal::TangentOnly;
                return tb::WalkStatus::Completed;
            }
            result.refusal = operation == Operation::Intersect
                ? Refusal::EmptyResult
                : operation == Operation::Union
                    ? Refusal::Disconnected
                    : Refusal::NoInteriorOverlap;
            return tb::WalkStatus::Completed;
        }

        TopoDS_Shape built;
        if (operation == Operation::Union) {
            if (cancelled.load()) {
                result.refusal = Refusal::Cancelled;
                return tb::WalkStatus::Cancelled;
            }
            if (!budget.beginStage(tb::Site::C26AnalyticBoolean)) {
                result.refusal = Refusal::Budget;
                return tb::WalkStatus::BudgetDenied;
            }
            walk = ChargeAnalyticPass(leftSourceShape, budget, cancelled,
                tb::Site::C26AnalyticBoolean);
            if (walk != tb::WalkStatus::Completed) return classify(walk);
            walk = ChargeAnalyticPass(rightSourceShape, budget, cancelled,
                tb::Site::C26AnalyticBoolean);
            if (walk != tb::WalkStatus::Completed) return classify(walk);
            BRepAlgoAPI_Fuse boolean;
            if (!configureAndBuild(boolean)) {
                result.refusal = Refusal::BuildFailed;
                return tb::WalkStatus::Completed;
            }
            built = boolean.Shape();
            walk = observeHistoryCounted(boolean);
            if (walk != tb::WalkStatus::Completed) return classify(walk);
        } else if (operation == Operation::Subtract) {
            if (cancelled.load()) {
                result.refusal = Refusal::Cancelled;
                return tb::WalkStatus::Cancelled;
            }
            if (!budget.beginStage(tb::Site::C26AnalyticBoolean)) {
                result.refusal = Refusal::Budget;
                return tb::WalkStatus::BudgetDenied;
            }
            walk = ChargeAnalyticPass(leftSourceShape, budget, cancelled,
                tb::Site::C26AnalyticBoolean);
            if (walk != tb::WalkStatus::Completed) return classify(walk);
            walk = ChargeAnalyticPass(rightSourceShape, budget, cancelled,
                tb::Site::C26AnalyticBoolean);
            if (walk != tb::WalkStatus::Completed) return classify(walk);
            BRepAlgoAPI_Cut boolean;
            if (!configureAndBuild(boolean)) {
                result.refusal = Refusal::BuildFailed;
                return tb::WalkStatus::Completed;
            }
            built = boolean.Shape();
            walk = observeHistoryCounted(boolean);
            if (walk != tb::WalkStatus::Completed) return classify(walk);
        } else {
            built = common.Shape();
            walk = observeHistoryCounted(common);
            if (walk != tb::WalkStatus::Completed) return classify(walk);
        }
        double builtVolume = -1;
        walk = VolumeCounted(built, builtVolume, budget, cancelled,
            tb::Site::C26AnalyticBoolean);
        if (walk != tb::WalkStatus::Completed) return classify(walk);
        if (built.IsNull() || builtVolume <= volumeTolerance) {
            result.refusal = Refusal::EmptyResult;
            return tb::WalkStatus::Completed;
        }
        // The result validity and solidity passes are charged before they run.
        walk = ChargeAnalyticPass(built, budget, cancelled,
            tb::Site::C26AnalyticBoolean);
        if (walk != tb::WalkStatus::Completed) return classify(walk);
        if (!BRepCheck_Analyzer(built).IsValid()) {
            result.refusal = Refusal::BuildFailed;
            return tb::WalkStatus::Completed;
        }
        walk = ChargeAnalyticPass(built, budget, cancelled,
            tb::Site::C26AnalyticBoolean);
        if (walk != tb::WalkStatus::Completed) return classify(walk);
        if (SolidCount(built) != 1) {
            result.refusal = Refusal::Disconnected;
            return tb::WalkStatus::Completed;
        }
        result.refusal = Refusal::None;
        result.solid = built;
        if (evidence != nullptr) {
            std::string leftAfter, rightAfter;
            walk = ExactShapeBytesCounted(leftSourceShape, leftAfter,
                budget, cancelled, tb::Site::C25AnalyticInput);
            if (walk == tb::WalkStatus::Cancelled
                || walk == tb::WalkStatus::BudgetDenied) return classify(walk);
            const bool leftSerial = walk == tb::WalkStatus::Completed
                && !leftAfter.empty();
            walk = ExactShapeBytesCounted(rightSourceShape, rightAfter,
                budget, cancelled, tb::Site::C25AnalyticInput);
            if (walk == tb::WalkStatus::Cancelled
                || walk == tb::WalkStatus::BudgetDenied) return classify(walk);
            const bool rightSerial = walk == tb::WalkStatus::Completed
                && !rightAfter.empty();
            evidence->inputBytesUnchanged = leftSerial && rightSerial
                && leftBefore == leftAfter && rightBefore == rightAfter;
        }
        return tb::WalkStatus::Completed;
    } catch (...) {
        result = {};
        result.refusal = Refusal::BuildFailed;
        return tb::WalkStatus::Failed;
    }
}

} // namespace core3d::retained_part_boolean
