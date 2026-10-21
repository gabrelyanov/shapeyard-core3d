#pragma once

#include "RetainedEdgeTreatmentSnapshot.hxx"
#include "RetainedFaceSelectorValues.hxx"
#include "RetainedTopologyBudget.hxx"
#include "ProfileSourceBoundaryExpectation.hxx"

#include <BRepAdaptor_Curve.hxx>
#include <ElCLib.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Dir.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <atomic>
#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <set>

namespace core3d::retained_face_selector {
namespace et = core3d::retained_edge_treatment;
namespace tb = core3d::retained_topology_budget;

enum class Refusal : std::uint8_t {
    None = 0, InvalidIntent, FaceMissing, FaceAmbiguous, IncompleteBoundary,
    InvalidFaceUse, OwnershipMismatch, ExactCount, PartialCircle, StaleSource,
    UnsupportedTransform, SplitEdge, TruncatedDiscovery, SplineIdentityUnavailable,
    ReplayMismatch, UnsupportedCarrier, AdmissionUnavailable, Budget, Cancelled,
    Busy, NativeFailure, ReadOnlyInvariant
};

inline const char* RefusalCode(Refusal refusal) noexcept {
    static const char* values[] = {
        "b2.None", "b2.InvalidIntent", "b2.FaceMissing", "b2.FaceAmbiguous",
        "b2.IncompleteBoundary", "b2.InvalidFaceUse", "b2.OwnershipMismatch",
        "b2.ExactCount", "b2.PartialCircle", "b2.StaleSource", "b2.UnsupportedTransform",
        "b2.SplitEdge", "b2.TruncatedDiscovery", "b2.SplineIdentityUnavailable",
        "b2.ReplayMismatch", "b2.UnsupportedCarrier", "b2.AdmissionUnavailable",
        "b2.Budget", "b2.Cancelled", "b2.Busy", "b2.NativeFailure", "b2.ReadOnlyInvariant"
    };
    return values[std::size_t(refusal)];
}
inline const char* RefusalMessage(Refusal refusal) noexcept {
    static const char* values[] = {
        "",
        "The geometric selector is invalid. No model changes were made.",
        "The selector does not match a current planar face. No model changes were made.",
        "The selector matches more than one face. No model changes were made.",
        "The selector does not prove every boundary edge of one face. No model changes were made.",
        "A boundary edge has a seam, repeated use, or unsupported orientation. No model changes were made.",
        "A boundary edge does not have the required face ownership. No model changes were made.",
        "The proved selection does not have the required exact edge count. No model changes were made.",
        "A circle rim must be one complete closed circle. No model changes were made.",
        "The source changed after selector capture. Refresh the selection and try again. No model changes were made.",
        "This source transform is not supported for geometric selectors. No model changes were made.",
        "The selector matches split or ambiguous edges. No model changes were made.",
        "Native edge discovery is incomplete. No model changes were made.",
        "Authored spline edge identity is not available. No model changes were made.",
        "The saved selector no longer proves the same face and edges. No model changes were made.",
        "This source has no supported retained selector carrier. No model changes were made.",
        "Retained selector admission is not available for this source. No model changes were made.",
        "Geometric selector proof exceeded its bounded work limit. No model changes were made.",
        "Geometric selector proof was cancelled. No model changes were made.",
        "The model is busy. Try the selector again when the current operation finishes. No model changes were made.",
        "Native geometric selector proof failed. No model changes were made.",
        "Selector inspection failed its read-only integrity check."
    };
    return values[std::size_t(refusal)];
}

struct BoundaryUse {
    TopoDS_Wire wire;
    TopoDS_Edge edge;
    TopAbs_Orientation rawFaceUse = TopAbs_EXTERNAL;
    FaceUseDirection direction = FaceUseDirection::Forward;
    std::vector<TopoDS_Face> ownerFaces;
    bool selected = false;
};

struct Resolution;
class SourceStageProofIssuer;
class ClosedRimRoleProof;

namespace detail {
struct Affine final {
    // Row-major linear transform plus a millimetre translation. This is a
    // derived value only: persisted frame and BRep values are never rewritten.
    std::array<double, 9> linear{{1, 0, 0, 0, 1, 0, 0, 0, 1}};
    std::array<double, 3> translationMM{};
};

inline double Determinant(const Affine& value) noexcept {
    const auto& a = value.linear;
    return a[0] * (a[4] * a[8] - a[5] * a[7])
        - a[1] * (a[3] * a[8] - a[5] * a[6])
        + a[2] * (a[3] * a[7] - a[4] * a[6]);
}
inline bool Similarity(const Affine& value, double& absoluteScale) noexcept {
    for (double component : value.linear) if (!std::isfinite(component)) return false;
    for (double component : value.translationMM) if (!std::isfinite(component)) return false;
    const auto& a = value.linear;
    const auto dotColumn = [&](int first, int second) {
        return a[first] * a[second] + a[3 + first] * a[3 + second]
            + a[6 + first] * a[6 + second];
    };
    const double squared = dotColumn(0, 0);
    if (!std::isfinite(squared) || squared < 1e-12) return false;
    const double tolerance = 1e-10 * std::max(1.0, squared);
    if (std::abs(dotColumn(1, 1) - squared) > tolerance
        || std::abs(dotColumn(2, 2) - squared) > tolerance
        || std::abs(dotColumn(0, 1)) > tolerance
        || std::abs(dotColumn(0, 2)) > tolerance
        || std::abs(dotColumn(1, 2)) > tolerance
        || std::abs(Determinant(value)) < 1e-12) return false;
    absoluteScale = std::sqrt(squared);
    return std::isfinite(absoluteScale) && absoluteScale > 0;
}
inline bool Inverse(const Affine& value, Affine& output) noexcept {
    double ignored = 0;
    if (!Similarity(value, ignored)) return false;
    const auto& a = value.linear;
    const double determinant = Determinant(value);
    Affine result;
    result.linear = {{
        (a[4] * a[8] - a[5] * a[7]) / determinant,
        (a[2] * a[7] - a[1] * a[8]) / determinant,
        (a[1] * a[5] - a[2] * a[4]) / determinant,
        (a[5] * a[6] - a[3] * a[8]) / determinant,
        (a[0] * a[8] - a[2] * a[6]) / determinant,
        (a[2] * a[3] - a[0] * a[5]) / determinant,
        (a[3] * a[7] - a[4] * a[6]) / determinant,
        (a[1] * a[6] - a[0] * a[7]) / determinant,
        (a[0] * a[4] - a[1] * a[3]) / determinant,
    }};
    for (int row = 0; row < 3; ++row)
        result.translationMM[row] = -(result.linear[3 * row] * value.translationMM[0]
            + result.linear[3 * row + 1] * value.translationMM[1]
            + result.linear[3 * row + 2] * value.translationMM[2]);
    output = result;
    return true;
}
inline Affine Compose(const Affine& first, const Affine& second) noexcept {
    Affine result;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            result.linear[3 * row + column] = 0;
            for (int inner = 0; inner < 3; ++inner)
                result.linear[3 * row + column] += first.linear[3 * row + inner]
                    * second.linear[3 * inner + column];
        }
        result.translationMM[row] = first.translationMM[row];
        for (int inner = 0; inner < 3; ++inner)
            result.translationMM[row] += first.linear[3 * row + inner]
                * second.translationMM[inner];
    }
    return result;
}
inline std::array<double, 3> Point(const Affine& value,
    const std::array<double, 3>& point) noexcept {
    std::array<double, 3> result{};
    for (int row = 0; row < 3; ++row) {
        result[row] = value.translationMM[row];
        for (int column = 0; column < 3; ++column)
            result[row] += value.linear[3 * row + column] * point[column];
    }
    return result;
}
inline bool UnitVector(const std::array<double, 3>& value,
    std::array<double, 3>& output) noexcept {
    const double length = std::hypot(std::hypot(value[0], value[1]), value[2]);
    if (!std::isfinite(length) || length < 1e-12) return false;
    output = {value[0] / length, value[1] / length, value[2] / length};
    return true;
}
inline bool Tangent(const Affine& value, const std::array<double, 3>& tangent,
    std::array<double, 3>& output) noexcept {
    std::array<double, 3> raw{};
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            raw[row] += value.linear[3 * row + column] * tangent[column];
    return UnitVector(raw, output);
}
inline bool Normal(const Affine& value, const std::array<double, 3>& normal,
    std::array<double, 3>& output) noexcept {
    Affine inverse;
    if (!Inverse(value, inverse)) return false;
    std::array<double, 3> raw{};
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 3; ++column)
            raw[row] += inverse.linear[3 * column + row] * normal[column];
    return UnitVector(raw, output);
}
inline bool ProfileAffine(const profile::Parameters& parameters, Affine& output) noexcept {
    if (!std::isfinite(parameters.metersPerUnit) || parameters.metersPerUnit <= 0
        || parameters.definition.plane < 0 || parameters.definition.plane > 2) return false;
    try {
        Affine plane;
        if (parameters.definition.plane == 1)
            plane.linear = {{1, 0, 0, 0, 0, 1, 0, 1, 0}};
        else if (parameters.definition.plane == 2)
            plane.linear = {{0, 0, 1, 1, 0, 0, 0, 1, 0}};
        Affine frame;
        if (parameters.constructionFrame) {
            gp_Trsf transform;
            if (!parameters.constructionFrame->Transform(transform)) return false;
            for (int row = 0; row < 3; ++row)
                for (int column = 0; column < 3; ++column)
                    frame.linear[3 * row + column] = transform.Value(row + 1, column + 1);
            const double millimetresPerUnit = 1000.0 * parameters.metersPerUnit;
            for (int row = 0; row < 3; ++row)
                frame.translationMM[row] = transform.Value(row + 1, 4) * millimetresPerUnit;
        }
        const Affine result = Compose(frame, plane);
        double scale = 0;
        if (!Similarity(result, scale)) return false;
        output = result;
        return true;
    } catch (...) { return false; }
}
} // namespace detail

class SourceStageProof final {
public:
#if DEBUG
    // Geometry fixtures can exercise the kernel, but this adapter is absent
    // from Release and cannot issue persisted or production authority.
    static std::optional<SourceStageProof> DebugFixture(const SourceRelativeV1& binding,
        const std::vector<double>& currentProfileValues) noexcept {
        try {
            if (!ValidCaptureProfile(binding)) return std::nullopt;
            std::vector<double> captureValues(binding.captureProfileScalarBits.size());
            for (std::size_t index = 0; index < captureValues.size(); ++index)
                std::memcpy(&captureValues[index], &binding.captureProfileScalarBits[index], sizeof(double));
            profile::Parameters capture, current;
            std::vector<double> canonical;
            detail::Affine captureTransform, currentTransform;
            if (!profile::Decode(captureValues, capture)
                || !profile::Decode(currentProfileValues, current)
                || current.definition.revolve || !current.shells.empty()
                || !profile::Encode(current, canonical) || canonical != currentProfileValues
                || capture.metersPerUnit != current.metersPerUnit
                || !detail::ProfileAffine(capture, captureTransform)
                || !detail::ProfileAffine(current, currentTransform)) return std::nullopt;
            return SourceStageProof(binding, captureTransform, currentTransform);
        } catch (...) { return std::nullopt; }
    }
#endif
private:
    SourceRelativeV1 binding_;
    detail::Affine capture_, current_;
    SourceStageProof(const SourceRelativeV1& binding, const detail::Affine& capture,
        const detail::Affine& current) : binding_(binding), capture_(capture), current_(current) {}
    friend class SourceStageProofIssuer;
    friend Refusal ResolveImpl(const TopoDS_Shape&, const SelectorIntent&,
        const SourceStageProof*, const ClosedRimRoleProof*, double,
        et::ReplayBudget&, const std::atomic_bool&, Resolution&) noexcept;
};

class ClosedRimRoleProof final {
public:
#if DEBUG
    // This fixture adapter can only prove values for a native geometry test.
    // Production source authority remains exclusively issuer-owned.
    static std::optional<ClosedRimRoleProof> DebugFixture(
        const SourceOwnedClosedRimV1& binding,
        const std::vector<double>& currentProfileValues,
        et::ReplayBudget& budget, const std::atomic_bool& cancelled) noexcept {
        try {
            if (!ValidSourceOwnedClosedRim(binding)) return std::nullopt;
            profile::Parameters current;
            std::vector<double> canonical;
            complete_profile_source::ExtrusionExpectation expectation;
            ProfileProducerAccounting accounting(budget, cancelled);
            detail::Affine transform;
            if (!profile::Decode(currentProfileValues, current)
                || current.definition.revolve || !current.shells.empty()
                || !current.definition.circle
                || !profile::Encode(current, canonical)
                || canonical != currentProfileValues
                || !complete_profile_source::ConstructExtrusionExpectation(
                    current, accounting, expectation)
                || !detail::ProfileAffine(current, transform)
                || expectation.loops.empty()
                || expectation.loops.front().representation
                    != complete_profile_source::LoopRepresentation::PeriodicCircle
                || expectation.loops.front().inner) return std::nullopt;
            const double millimetresPerUnit = 1000.0 * current.metersPerUnit;
            const auto& circle = *current.definition.circle;
            const std::array<double, 3> lowerChart{{circle.center.X() * millimetresPerUnit,
                circle.center.Y() * millimetresPerUnit, 0}};
            const std::array<double, 3> upperChart{{circle.center.X() * millimetresPerUnit,
                circle.center.Y() * millimetresPerUnit,
                current.definition.depth * millimetresPerUnit}};
            std::array<double, 3> lowerNormal{}, upperNormal{};
            double scale = 0;
            if (!detail::Normal(transform, {0, 0, -1}, lowerNormal)
                || !detail::Normal(transform, {0, 0, 1}, upperNormal)
                || !detail::Similarity(transform, scale)) return std::nullopt;
            return ClosedRimRoleProof(binding, std::move(expectation),
                detail::Point(transform, lowerChart), detail::Point(transform, upperChart),
                lowerNormal, upperNormal, circle.outerRadius * millimetresPerUnit * scale);
        } catch (...) { return std::nullopt; }
    }
#endif
private:
    SourceOwnedClosedRimV1 binding_;
    complete_profile_source::ExtrusionExpectation expectation_;
    std::array<double, 3> lowerCenterMM_{}, upperCenterMM_{};
    std::array<double, 3> lowerNormal_{}, upperNormal_{};
    double outerRadiusMM_ = 0;
    ClosedRimRoleProof(const SourceOwnedClosedRimV1& binding,
        complete_profile_source::ExtrusionExpectation expectation,
        std::array<double, 3> lowerCenter, std::array<double, 3> upperCenter,
        std::array<double, 3> lowerNormal, std::array<double, 3> upperNormal,
        double radius)
        : binding_(binding), expectation_(std::move(expectation)),
          lowerCenterMM_(lowerCenter), upperCenterMM_(upperCenter),
          lowerNormal_(lowerNormal), upperNormal_(upperNormal), outerRadiusMM_(radius) {}
    friend class SourceStageProofIssuer;
    friend class ClosedRimRoleReview;
    friend Refusal ResolveImpl(const TopoDS_Shape&, const SelectorIntent&,
        const SourceStageProof*, const ClosedRimRoleProof*, double,
        et::ReplayBudget&, const std::atomic_bool&, Resolution&) noexcept;
};

class ClosedRimRoleReview final {
    std::shared_ptr<const ClosedRimRoleProof> proof_;
    explicit ClosedRimRoleReview(std::shared_ptr<const ClosedRimRoleProof> proof)
        : proof_(std::move(proof)) {}
    friend class SourceStageProofIssuer;
};

class FaceMembershipProof final {
public:
    const SelectorIntent& intent() const noexcept { return intent_; }
    const TopoDS_Face& face() const noexcept { return face_; }
    const PlaneWitness& plane() const noexcept { return plane_; }
    const std::vector<BoundaryUse>& boundaryUses() const noexcept { return boundaryUses_; }
    std::uint32_t matchedFaceCount() const noexcept { return 1; }
    std::uint32_t wireCount() const noexcept { return wireCount_; }
    std::uint32_t uniqueEdgeCount() const noexcept { return uniqueEdgeCount_; }
    std::uint32_t selectedEdgeCount() const noexcept { return selectedEdgeCount_; }
    bool discoveryComplete() const noexcept { return true; }
    Coverage coverage() const noexcept { return coverage_; }
#if DEBUG
    const LegacyIntent& debugEvaluatedIntent() const noexcept { return evaluatedIntent_; }
    const std::array<double, 2>& debugScopeBoundsMM() const noexcept { return scopeBoundsMM_; }
#endif
private:
    FaceMembershipProof() = default;
    SelectorIntent intent_;
    TopoDS_Face face_;
    PlaneWitness plane_;
    std::vector<BoundaryUse> boundaryUses_;
    std::uint32_t wireCount_ = 0, uniqueEdgeCount_ = 0, selectedEdgeCount_ = 0;
    Coverage coverage_ = Coverage::EntireBoundary;
    LegacyIntent evaluatedIntent_;
    std::array<double, 2> scopeBoundsMM_{};
    friend Refusal ResolveImpl(const TopoDS_Shape&, const SelectorIntent&,
        const SourceStageProof*, const ClosedRimRoleProof*, double,
        et::ReplayBudget&, const std::atomic_bool&, Resolution&) noexcept;
};

struct Resolution {
    Refusal refusal = Refusal::NativeFailure;
    std::shared_ptr<const FaceMembershipProof> proof;
};

namespace detail {
inline double Component(const gp_Pnt& point, Axis axis) noexcept {
    if (axis == Axis::X) return point.X();
    if (axis == Axis::Y) return point.Y();
    return point.Z();
}
inline std::array<double, 3> Components(const gp_Dir& direction) noexcept {
    return {direction.X(), direction.Y(), direction.Z()};
}
inline gp_Dir RequestedNormal(const PlanarFaceScope& scope) {
    const double sign = scope.side == Side::Max ? 1.0 : -1.0;
    if (scope.axis == Axis::X) return gp_Dir(sign, 0, 0);
    if (scope.axis == Axis::Y) return gp_Dir(0, sign, 0);
    return gp_Dir(0, 0, sign);
}
inline bool SamePoint(const gp_Pnt& point, const std::array<double, 3>& expected, double localTolerance) {
    return point.Distance(gp_Pnt(expected[0], expected[1], expected[2])) <= localTolerance;
}
inline bool Parallel(const gp_Dir& first, const std::array<double, 3>& second, double tolerance = 1e-8) {
    return first.IsParallel(gp_Dir(second[0], second[1], second[2]), tolerance);
}
inline bool ScopeOf(const SelectorIntent& intent, PlanarFaceScope& output) {
    if (const auto* planar = std::get_if<PlanarFaceBoundary>(&intent)) { output = planar->face; return true; }
    if (const auto* line = std::get_if<Line>(&intent)) { output = line->face; return true; }
    return false;
}
struct OrientedScope final {
    Side side = Side::Min;
    std::array<double, 3> originMM{};
    std::array<double, 3> axisDirection{};
    std::array<double, 3> outwardNormal{};
};
inline std::array<double, 3> AxisVector(Axis axis, double sign = 1) noexcept {
    if (axis == Axis::X) return {sign, 0, 0};
    if (axis == Axis::Y) return {0, sign, 0};
    return {0, 0, sign};
}
inline bool TransformRelative(const SourceRelativeV1& source,
    const Affine& capture, const Affine& current, LegacyIntent& output,
    std::optional<OrientedScope>& scope, Affine& effective) noexcept {
    try {
        if (source.coordinateSpace == CoordinateSpace::SourceChart) effective = current;
        else {
            Affine inverseCapture;
            if (!Inverse(capture, inverseCapture)) return false;
            effective = Compose(current, inverseCapture);
        }
        double radiusScale = 0;
        if (!Similarity(effective, radiusScale)) return false;
        return std::visit([&](const auto& inner) {
            using T = std::decay_t<decltype(inner)>;
            if constexpr (std::is_same_v<T, CircleRim>) {
                CircleRim transformed = inner;
                transformed.centerMM = Point(effective, inner.centerMM);
                if (!Normal(effective, inner.normal, transformed.normal)) return false;
                transformed.radiusMM *= radiusScale;
                if (!std::isfinite(transformed.radiusMM)) return false;
                output = transformed;
                return true;
            } else {
                OrientedScope oriented;
                oriented.side = inner.face.side;
                oriented.originMM = Point(effective, {0, 0, 0});
                const auto axis = AxisVector(inner.face.axis);
                const double sideSign = inner.face.side == Side::Max ? 1.0 : -1.0;
                if (!Tangent(effective, axis, oriented.axisDirection)
                    || !Normal(effective, AxisVector(inner.face.axis, sideSign),
                        oriented.outwardNormal)) return false;
                scope = oriented;
                if constexpr (std::is_same_v<T, Line>) {
                    Line transformed = inner;
                    transformed.locationMM = Point(effective, inner.locationMM);
                    if (!Tangent(effective, inner.direction, transformed.direction)) return false;
                    output = transformed;
                } else output = inner;
                return true;
            }
        }, source.inner);
    } catch (...) { return false; }
}
inline Refusal MapWalk(tb::WalkStatus status) noexcept {
    // A cancelled walk reports cancellation; only a genuine denial or a
    // failed traversal maps to the budget/native refusals.
    if (status == tb::WalkStatus::Cancelled) return Refusal::Cancelled;
    if (status == tb::WalkStatus::BudgetDenied) return Refusal::Budget;
    return Refusal::NativeFailure;
}
inline bool FacePlane(const TopoDS_Face& face, gp_Pln& plane) {
    BRepAdaptor_Surface surface(face, true);
    if (surface.GetType() != GeomAbs_Plane) return false;
    plane = surface.Plane();
    if (face.Orientation() == TopAbs_REVERSED) plane.SetAxis(plane.Axis().Reversed());
    return true;
}
} // namespace detail

inline Refusal ResolveImpl(const TopoDS_Shape& inputStage, const SelectorIntent& intent,
    const SourceStageProof* sourceProof, const ClosedRimRoleProof* closedRimProof,
    double metersPerLocalUnit,
    et::ReplayBudget& budget, const std::atomic_bool& cancelled, Resolution& output) noexcept {
    output = {};
    try {
        if (!ValidIntent(intent, true) || !std::isfinite(metersPerLocalUnit) || metersPerLocalUnit <= 0) {
            output.refusal = Refusal::InvalidIntent; return output.refusal;
        }
        if (std::holds_alternative<ReservedSplineEdge>(intent)) {
            output.refusal = Refusal::SplineIdentityUnavailable; return output.refusal;
        }
        // A source-relative predicate is meaningful only after the R2 caller
        // proves and supplies its source/stage context. The legacy entry point
        // must fail closed; it never unwraps or interprets the inner intent in
        // the capture frame.
        LegacyIntent evaluated;
        std::optional<detail::OrientedScope> orientedScope;
        detail::Affine effective;
        const auto* closedRim = std::get_if<SourceOwnedClosedRimV1>(&intent);
        if (const auto* relative = std::get_if<SourceRelativeV1>(&intent)) {
            if (!sourceProof) {
                output.refusal = Refusal::UnsupportedCarrier; return output.refusal;
            }
            if (!(sourceProof->binding_ == *relative)) {
                output.refusal = Refusal::StaleSource; return output.refusal;
            }
            if (!detail::TransformRelative(*relative, sourceProof->capture_, sourceProof->current_,
                    evaluated, orientedScope, effective)) {
                output.refusal = Refusal::UnsupportedTransform; return output.refusal;
            }
        } else if (closedRim) {
            if (!closedRimProof) {
                output.refusal = Refusal::UnsupportedCarrier; return output.refusal;
            }
            if (!(closedRimProof->binding_ == *closedRim)
                || closedRimProof->expectation_.loops.empty()
                || closedRimProof->expectation_.loops.front().representation
                    != complete_profile_source::LoopRepresentation::PeriodicCircle
                || !std::isfinite(closedRimProof->outerRadiusMM_)
                || closedRimProof->outerRadiusMM_ <= 0) {
                output.refusal = Refusal::StaleSource; return output.refusal;
            }
            CircleRim role;
            role.centerMM = closedRim->capRole == CapRole::Upper
                ? closedRimProof->upperCenterMM_ : closedRimProof->lowerCenterMM_;
            role.normal = closedRim->capRole == CapRole::Upper
                ? closedRimProof->upperNormal_ : closedRimProof->lowerNormal_;
            role.radiusMM = closedRimProof->outerRadiusMM_;
            role.expectedCount = 1;
            evaluated = role;
        } else if (const auto* planar = std::get_if<PlanarFaceBoundary>(&intent)) evaluated = *planar;
        else if (const auto* line = std::get_if<Line>(&intent)) evaluated = *line;
        else if (const auto* circle = std::get_if<CircleRim>(&intent)) evaluated = *circle;
        else { output.refusal = Refusal::SplineIdentityUnavailable; return output.refusal; }
        const SelectorIntent evaluatedIntent = std::visit([](const auto& value) -> SelectorIntent {
            return value;
        }, evaluated);
        if (inputStage.IsNull()) { output.refusal = Refusal::FaceMissing; return output.refusal; }
        if (cancelled.load()) { output.refusal = Refusal::Cancelled; return output.refusal; }
        const double localTolerance = 1e-4 / (1000.0 * metersPerLocalUnit);

        // C01: bounded raw stage census. Every root/child occurrence is
        // charged while walking, before the face map grows; the combined
        // face+edge census refuses the 4,097th distinct admission instead of
        // inspecting a completed oversized map. The stage debit is preserved.
        tb::Census census;
        const tb::WalkStatus censusStatus = tb::CensusTopology(inputStage, budget,
            cancelled, census, tb::Site::C01StageCensus, true);
        if (censusStatus != tb::WalkStatus::Completed) {
            output.refusal = detail::MapWalk(censusStatus); return output.refusal;
        }
        const TopTools_IndexedMapOfShape& faceMap = census.faces;
        std::vector<TopoDS_Face> candidates;
        PlanarFaceScope scope;
        std::array<double, 2> projectionBoundsMM{};
        if (orientedScope) {
            const double millimetresPerLocalUnit = 1000.0 * metersPerLocalUnit;
            const gp_Pnt origin(orientedScope->originMM[0] / millimetresPerLocalUnit,
                orientedScope->originMM[1] / millimetresPerLocalUnit,
                orientedScope->originMM[2] / millimetresPerLocalUnit);
            const gp_Dir axis(orientedScope->axisDirection[0], orientedScope->axisDirection[1],
                orientedScope->axisDirection[2]);
            const gp_Dir requested(orientedScope->outwardNormal[0], orientedScope->outwardNormal[1],
                orientedScope->outwardNormal[2]);
            double minimum = std::numeric_limits<double>::infinity();
            double maximum = -std::numeric_limits<double>::infinity();
            for (int index = 1; index <= faceMap.Extent(); ++index) {
                if (!budget.visit(1, tb::Site::C02FacePasses)) {
                    output.refusal = Refusal::Budget; return output.refusal;
                }
                gp_Pln plane;
                if (!detail::FacePlane(TopoDS::Face(faceMap(index)), plane)
                    || !plane.Axis().Direction().IsParallel(requested, 1e-8)) continue;
                const double projection = gp_Vec(origin, plane.Location()).Dot(gp_Vec(axis));
                minimum = std::min(minimum, projection);
                maximum = std::max(maximum, projection);
            }
            if (!std::isfinite(minimum) || !std::isfinite(maximum)) {
                output.refusal = Refusal::FaceMissing; return output.refusal;
            }
            const double extreme = orientedScope->side == Side::Max ? maximum : minimum;
            projectionBoundsMM = {minimum * millimetresPerLocalUnit,
                                  maximum * millimetresPerLocalUnit};
            for (int index = 1; index <= faceMap.Extent(); ++index) {
                if (!budget.visit(1, tb::Site::C02FacePasses)) {
                    output.refusal = Refusal::Budget; return output.refusal;
                }
                gp_Pln plane; const TopoDS_Face face = TopoDS::Face(faceMap(index));
                if (!detail::FacePlane(face, plane)
                    || !plane.Axis().Direction().IsParallel(requested, 1e-8)) continue;
                const double projection = gp_Vec(origin, plane.Location()).Dot(gp_Vec(axis));
                if (std::abs(projection - extreme) <= localTolerance) candidates.push_back(face);
            }
        } else if (detail::ScopeOf(evaluatedIntent, scope)) {
            double extreme = scope.side == Side::Max
                ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
            for (int index = 1; index <= faceMap.Extent(); ++index) {
                // C02: every extrema-pass face inspection is charged, even
                // rejected or duplicate candidates.
                if (!budget.visit(1, tb::Site::C02FacePasses)) {
                    output.refusal = Refusal::Budget; return output.refusal;
                }
                gp_Pln plane; if (!detail::FacePlane(TopoDS::Face(faceMap(index)), plane)) continue;
                if (!plane.Axis().Direction().IsParallel(detail::RequestedNormal(scope), 1e-8)) continue;
                const double coordinate = detail::Component(plane.Location(), scope.axis);
                extreme = scope.side == Side::Max ? std::max(extreme, coordinate) : std::min(extreme, coordinate);
            }
            if (!std::isfinite(extreme)) { output.refusal = Refusal::FaceMissing; return output.refusal; }
            for (int index = 1; index <= faceMap.Extent(); ++index) {
                // C02: collection pass, same per-inspection charge.
                if (!budget.visit(1, tb::Site::C02FacePasses)) {
                    output.refusal = Refusal::Budget; return output.refusal;
                }
                gp_Pln plane; const TopoDS_Face face = TopoDS::Face(faceMap(index));
                if (!detail::FacePlane(face, plane)
                    || !plane.Axis().Direction().IsParallel(detail::RequestedNormal(scope), 1e-8)) continue;
                if (std::abs(detail::Component(plane.Location(), scope.axis) - extreme) <= localTolerance)
                    candidates.push_back(face);
            }
        } else {
            const auto& circle = std::get<CircleRim>(evaluatedIntent);
            const gp_Pnt center(circle.centerMM[0] / (1000.0 * metersPerLocalUnit),
                circle.centerMM[1] / (1000.0 * metersPerLocalUnit),
                circle.centerMM[2] / (1000.0 * metersPerLocalUnit));
            for (int index = 1; index <= faceMap.Extent(); ++index) {
                // C02: circle-plane pass, same per-inspection charge.
                if (!budget.visit(1, tb::Site::C02FacePasses)) {
                    output.refusal = Refusal::Budget; return output.refusal;
                }
                gp_Pln plane; const TopoDS_Face face = TopoDS::Face(faceMap(index));
                if (!detail::FacePlane(face, plane) || !detail::Parallel(plane.Axis().Direction(), circle.normal)
                    || plane.Distance(center) > localTolerance) continue;
                candidates.push_back(face);
            }
        }
        if (candidates.empty()) { output.refusal = Refusal::FaceMissing; return output.refusal; }
        if (candidates.size() > 1) { output.refusal = Refusal::FaceAmbiguous; return output.refusal; }

        auto proof = std::shared_ptr<FaceMembershipProof>(new FaceMembershipProof);
        proof->intent_ = intent; proof->evaluatedIntent_ = evaluated;
        proof->scopeBoundsMM_ = projectionBoundsMM; proof->face_ = candidates.front();
        // C02: the selected-face plane read is charged like every other face
        // inspection.
        if (!budget.visit(1, tb::Site::C02FacePasses)) {
            output.refusal = Refusal::Budget; return output.refusal;
        }
        gp_Pln selectedPlane; detail::FacePlane(proof->face_, selectedPlane);
        proof->plane_.outwardNormal = detail::Components(selectedPlane.Axis().Direction());
        proof->plane_.offsetMM = selectedPlane.Axis().Direction().XYZ().Dot(selectedPlane.Location().XYZ())
            * 1000.0 * metersPerLocalUnit;

        // C03: the edge→face ancestor map over the whole input stage is a bulk
        // OCCT pass. Its full traversal plus every edge-face relation
        // insertion (measured on the C01 census: one per edge occurrence under
        // a face, selected and nonselected) is reserved before the call; the
        // reserved pass is not charged again afterwards.
        if (!budget.visit(census.occurrences + census.edgeUsesUnderFaces, tb::Site::C03AncestorMap)) {
            output.refusal = Refusal::Budget; return output.refusal;
        }
        TopTools_IndexedDataMapOfShapeListOfShape owners;
        TopExp::MapShapesAndAncestors(inputStage, TopAbs_EDGE, TopAbs_FACE, owners);
        TopTools_IndexedMapOfShape uniqueEdges;
        std::set<int> usesOnFace;
        for (TopExp_Explorer wireExplorer(proof->face_, TopAbs_WIRE); wireExplorer.More(); wireExplorer.Next()) {
            if (cancelled.load()) { output.refusal = Refusal::Cancelled; return output.refusal; }
            const TopoDS_Wire wire = TopoDS::Wire(wireExplorer.Current());
            // C04: every inspected wire occurrence is charged.
            if (!budget.visit(1, tb::Site::C04WireExplorers)) {
                output.refusal = Refusal::Budget; return output.refusal;
            }
            ++proof->wireCount_;
            // C06: the independent direct-use census runs before the ordered
            // walk and measures the wire-explorer initialization reservation.
            // A budget denial here takes precedence over any later ownership
            // or malformed-wire diagnosis.
            std::size_t direct = 0;
            TopTools_IndexedMapOfShape directEdges;
            for (TopExp_Explorer directExplorer(wire, TopAbs_EDGE); directExplorer.More(); directExplorer.Next()) {
                if (!budget.visit(1, tb::Site::C06DirectCensus)) {
                    output.refusal = Refusal::Budget; return output.refusal;
                }
                ++direct;
                directEdges.Add(directExplorer.Current());
            }
            // C04: BRepTools_WireExplorer traverses during construction;
            // reserve that pass (measured: the wire's edge-use count) before
            // constructing it.
            if (!budget.visit(direct, tb::Site::C04WireExplorers)) {
                output.refusal = Refusal::Budget; return output.refusal;
            }
            std::size_t explored = 0;
            TopTools_IndexedMapOfShape orderedEdges;
            for (BRepTools_WireExplorer edgeExplorer(wire, proof->face_); edgeExplorer.More(); edgeExplorer.Next()) {
                // C04: every raw ordered edge use is charged, preserving raw
                // orientations and holes exactly.
                if (!budget.visit(1, tb::Site::C04WireExplorers)) {
                    output.refusal = Refusal::Budget; return output.refusal;
                }
                ++explored;
                const TopoDS_Edge edge = edgeExplorer.Current();
                orderedEdges.Add(edge);
                const TopAbs_Orientation orientation = edge.Orientation();
                if (orientation != TopAbs_FORWARD && orientation != TopAbs_REVERSED) {
                    output.refusal = Refusal::InvalidFaceUse; return output.refusal;
                }
                // C05: the owner lookup and the per-wire deduplication are
                // charged ancestry/comparison work.
                if (!budget.visit(2, tb::Site::C05OwnerScan)) {
                    output.refusal = Refusal::Budget; return output.refusal;
                }
                const int edgeIndex = owners.FindIndex(edge);
                if (edgeIndex <= 0 || !usesOnFace.insert(edgeIndex).second || BRep_Tool::Degenerated(edge)) {
                    output.refusal = Refusal::InvalidFaceUse; return output.refusal;
                }
                BoundaryUse use; use.wire = wire; use.edge = edge; use.rawFaceUse = orientation;
                use.direction = orientation == TopAbs_FORWARD ? FaceUseDirection::Forward : FaceUseDirection::Reversed;
                const TopTools_ListOfShape& ancestors = owners.FindFromIndex(edgeIndex);
                bool repeatedOwnerOccurrence = false;
                for (TopTools_ListIteratorOfListOfShape iterator(ancestors); iterator.More(); iterator.Next()) {
                    // C05: every ancestry entry is charged before access, and
                    // every dedup comparison against a prior owner is charged,
                    // not just the two surviving owners.
                    if (!budget.visit(1, tb::Site::C05OwnerScan)) {
                        output.refusal = Refusal::Budget; return output.refusal;
                    }
                    const TopoDS_Face owner = TopoDS::Face(iterator.Value());
                    bool known = false;
                    for (const TopoDS_Face& prior : use.ownerFaces) {
                        if (!budget.visit(1, tb::Site::C05OwnerScan)) {
                            output.refusal = Refusal::Budget; return output.refusal;
                        }
                        if (prior.IsSame(owner)) { known = true; repeatedOwnerOccurrence = true; break; }
                    }
                    if (!known) use.ownerFaces.push_back(owner);
                }
                // C05: the selected-owner comparison is charged per owner.
                bool selectedOwner = false;
                for (const TopoDS_Face& owner : use.ownerFaces) {
                    if (!budget.visit(1, tb::Site::C05OwnerScan)) {
                        output.refusal = Refusal::Budget; return output.refusal;
                    }
                    if (owner.IsSame(proof->face_)) { selectedOwner = true; break; }
                }
                // F05: a repeated occurrence of the same owner is still an
                // invalid ownership relation. It is charged before
                // deduplication above, but deduplication must not turn the
                // deliberately invalid diagnostic compound into mutation
                // authority.
                if (repeatedOwnerOccurrence || use.ownerFaces.size() != 2 || !selectedOwner) {
                    output.refusal = Refusal::OwnershipMismatch; return output.refusal;
                }
                if (closedRim) {
                    Standard_Real first = 0, last = 0;
                    const Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(
                        edge, proof->face_, first, last);
                    BRepAdaptor_Curve curve(edge);
                    if (pcurve.IsNull() || !std::isfinite(first) || !std::isfinite(last)
                        || !std::isfinite(curve.FirstParameter())
                        || !std::isfinite(curve.LastParameter())) {
                        output.refusal = Refusal::IncompleteBoundary; return output.refusal;
                    }
                }
                // C05: the unique-edge/use insertion is charged.
                if (!budget.visit(1, tb::Site::C05OwnerScan)) {
                    output.refusal = Refusal::Budget; return output.refusal;
                }
                uniqueEdges.Add(edge); proof->boundaryUses_.push_back(std::move(use));
            }
            // C06: the direct-use census was charged before the ordered walk;
            // the completeness comparison itself is one charged unit.
            if (!budget.visit(1, tb::Site::C06DirectCensus)) {
                output.refusal = Refusal::Budget; return output.refusal;
            }
            bool sameMembership = directEdges.Extent() == orderedEdges.Extent();
            for (int index = 1; closedRim && sameMembership
                && index <= directEdges.Extent(); ++index)
                sameMembership = orderedEdges.Contains(directEdges(index));
            if (explored == 0 || explored != direct || (closedRim && !sameMembership)) {
                output.refusal = Refusal::IncompleteBoundary; return output.refusal;
            }
        }
        if (proof->wireCount_ == 0 || proof->wireCount_ > 64 || proof->boundaryUses_.size() > 64) {
            output.refusal = proof->boundaryUses_.size() > 64 ? Refusal::TruncatedDiscovery : Refusal::IncompleteBoundary;
            return output.refusal;
        }
        proof->uniqueEdgeCount_ = std::uint32_t(uniqueEdges.Extent());
        // C06: the uniqueness comparison over the census result is charged.
        if (!budget.visit(1, tb::Site::C06DirectCensus)) {
            output.refusal = Refusal::Budget; return output.refusal;
        }
        if (proof->uniqueEdgeCount_ != proof->boundaryUses_.size()) {
            output.refusal = Refusal::InvalidFaceUse; return output.refusal;
        }

        if (const auto* line = std::get_if<Line>(&evaluatedIntent)) {
            proof->coverage_ = Coverage::SelectedLine;
            const gp_Pnt witness(line->locationMM[0] / (1000.0 * metersPerLocalUnit),
                line->locationMM[1] / (1000.0 * metersPerLocalUnit),
                line->locationMM[2] / (1000.0 * metersPerLocalUnit));
            for (auto& use : proof->boundaryUses_) {
                // C07: every inspected use pays for its line classification.
                if (!budget.visit(1, tb::Site::C07CurveClassification)) {
                    output.refusal = Refusal::Budget; return output.refusal;
                }
                BRepAdaptor_Curve curve(use.edge);
                if (curve.GetType() != GeomAbs_Line || !detail::Parallel(curve.Line().Direction(), line->direction)) continue;
                const double parameter = ElCLib::Parameter(curve.Line(), witness);
                if (parameter < curve.FirstParameter() - localTolerance || parameter > curve.LastParameter() + localTolerance
                    || curve.Line().Distance(witness) > localTolerance) continue;
                use.selected = true; ++proof->selectedEdgeCount_;
            }
            if (proof->selectedEdgeCount_ == 0) { output.refusal = Refusal::FaceMissing; return output.refusal; }
            if (proof->selectedEdgeCount_ > 1) { output.refusal = Refusal::SplitEdge; return output.refusal; }
        } else if (closedRim) {
            proof->coverage_ = Coverage::SelectedClosedRim;
            const auto& circle = std::get<CircleRim>(evaluatedIntent);
            const double millimetresPerLocalUnit = 1000.0 * metersPerLocalUnit;
            const gp_Pnt expectedCenter(circle.centerMM[0] / millimetresPerLocalUnit,
                circle.centerMM[1] / millimetresPerLocalUnit,
                circle.centerMM[2] / millimetresPerLocalUnit);
            const gp_Dir expectedAxis(circle.normal[0], circle.normal[1], circle.normal[2]);
            const TopoDS_Wire outerWire = BRepTools::OuterWire(proof->face_);
            std::size_t partial = 0, matching = 0;
            for (auto& use : proof->boundaryUses_) {
                if (!budget.visit(1, tb::Site::C07CurveClassification)) {
                    output.refusal = Refusal::Budget; return output.refusal;
                }
                BRepAdaptor_Curve curve(use.edge);
                if (curve.GetType() != GeomAbs_Circle) continue;
                const bool sameSupport = curve.Circle().Location().Distance(expectedCenter)
                        <= localTolerance
                    && curve.Circle().Axis().Direction().IsParallel(expectedAxis, 1e-8)
                    && std::abs(curve.Circle().Radius() * millimetresPerLocalUnit
                        - circle.radiusMM) <= 1e-4;
                if (!sameSupport) continue;
                const bool complete = curve.IsClosed()
                    && std::abs((curve.LastParameter() - curve.FirstParameter())
                        - 2.0 * M_PI) <= 1e-8;
                if (!complete) { ++partial; continue; }
                if (outerWire.IsNull() || !use.wire.IsSame(outerWire)) continue;
                bool outerWall = false;
                for (const auto& owner : use.ownerFaces) {
                    if (owner.IsSame(proof->face_)) continue;
                    BRepAdaptor_Surface surface(owner, true);
                    if (surface.GetType() != GeomAbs_Cylinder) continue;
                    const auto cylinder = surface.Cylinder();
                    const gp_Lin axis(cylinder.Axis());
                    if (cylinder.Axis().Direction().IsParallel(expectedAxis, 1e-8)
                        && axis.Distance(expectedCenter) <= localTolerance
                        && std::abs(cylinder.Radius() * millimetresPerLocalUnit
                            - circle.radiusMM) <= 1e-4) outerWall = true;
                }
                if (!outerWall) continue;
                use.selected = true; ++matching;
            }
            if (matching > 1) {
                output.refusal = Refusal::FaceAmbiguous; return output.refusal;
            }
            if (matching == 0) {
                output.refusal = partial > 1 ? Refusal::SplitEdge
                    : partial == 1 ? Refusal::PartialCircle : Refusal::FaceMissing;
                return output.refusal;
            }
            proof->selectedEdgeCount_ = 1;
        } else if (const auto* circle = std::get_if<CircleRim>(&evaluatedIntent)) {
            proof->coverage_ = Coverage::EntireBoundary;
            if (proof->wireCount_ != 1 || proof->boundaryUses_.size() != 1) {
                output.refusal = Refusal::PartialCircle; return output.refusal;
            }
            auto& use = proof->boundaryUses_.front();
            // C07: the circle/domain and full-boundary inspection is charged.
            if (!budget.visit(1, tb::Site::C07CurveClassification)) {
                output.refusal = Refusal::Budget; return output.refusal;
            }
            BRepAdaptor_Curve curve(use.edge);
            if (curve.GetType() != GeomAbs_Circle || !curve.IsClosed()
                || std::abs((curve.LastParameter() - curve.FirstParameter()) - 2.0 * M_PI) > 1e-8
                || curve.Circle().Location().Distance(gp_Pnt(circle->centerMM[0] / (1000.0 * metersPerLocalUnit),
                    circle->centerMM[1] / (1000.0 * metersPerLocalUnit),
                    circle->centerMM[2] / (1000.0 * metersPerLocalUnit))) > localTolerance
                || std::abs(curve.Circle().Radius() * 1000.0 * metersPerLocalUnit - circle->radiusMM) > 1e-4) {
                output.refusal = Refusal::PartialCircle; return output.refusal;
            }
            use.selected = true; proof->selectedEdgeCount_ = 1;
        } else {
            proof->coverage_ = Coverage::EntireBoundary;
            for (auto& use : proof->boundaryUses_) {
                // C07: every full-boundary use inspection is charged.
                if (!budget.visit(1, tb::Site::C07CurveClassification)) {
                    output.refusal = Refusal::Budget; return output.refusal;
                }
                if (BRepAdaptor_Curve(use.edge).GetType() != GeomAbs_Line) {
                    output.refusal = Refusal::IncompleteBoundary; return output.refusal;
                }
                use.selected = true; ++proof->selectedEdgeCount_;
            }
        }
        if (proof->selectedEdgeCount_ != ExpectedCount(intent)) {
            output.refusal = Refusal::ExactCount; return output.refusal;
        }
        output.proof = std::move(proof); output.refusal = Refusal::None; return output.refusal;
    } catch (...) { output = {}; output.refusal = Refusal::NativeFailure; return output.refusal; }
}

inline Refusal Resolve(const TopoDS_Shape& inputStage, const SelectorIntent& intent,
    double metersPerLocalUnit, et::ReplayBudget& budget, const std::atomic_bool& cancelled,
    Resolution& output) noexcept {
    return ResolveImpl(inputStage, intent, nullptr, nullptr, metersPerLocalUnit,
        budget, cancelled, output);
}

inline Refusal Resolve(const TopoDS_Shape& inputStage, const SelectorIntent& intent,
    const SourceStageProof& sourceProof, double metersPerLocalUnit, et::ReplayBudget& budget,
    const std::atomic_bool& cancelled, Resolution& output) noexcept {
    return ResolveImpl(inputStage, intent, &sourceProof, nullptr, metersPerLocalUnit,
        budget, cancelled, output);
}

inline Refusal Resolve(const TopoDS_Shape& inputStage, const SelectorIntent& intent,
    const ClosedRimRoleProof& roleProof, double metersPerLocalUnit,
    et::ReplayBudget& budget, const std::atomic_bool& cancelled,
    Resolution& output) noexcept {
    return ResolveImpl(inputStage, intent, nullptr, &roleProof, metersPerLocalUnit,
        budget, cancelled, output);
}

inline Refusal VerifyReceipt(const FaceMembershipProof& proof, const et::Step& step,
    double, et::ReplayBudget& budget) noexcept {
    if (!step.selector || !ValidReceipt(*step.selector)) return Refusal::ReplayMismatch;
    const auto& receipt = *step.selector;
    // C08: the receipt/witness header comparison block and every key-entry
    // comparison charge the caller's operation budget; no fresh budget is
    // created and receipt equality is unchanged.
    if (!budget.visit(1, tb::Site::C08ReceiptVerify)) return Refusal::Budget;
    if (!(receipt.intent == proof.intent()) || !(receipt.face == proof.plane())
        || receipt.coverage != proof.coverage() || receipt.wireCount != proof.wireCount()
        || receipt.boundaryUseCount != proof.boundaryUses().size()
        || receipt.boundaryUniqueEdgeCount != proof.uniqueEdgeCount()
        || receipt.entries.size() != step.anchors.size()) return Refusal::ReplayMismatch;
    for (std::size_t index = 0; index < step.anchors.size(); ++index) {
        if (!budget.visit(1, tb::Site::C08ReceiptVerify)) return Refusal::Budget;
        if (receipt.entries[index].anchorKey != step.anchors[index].key) return Refusal::ReplayMismatch;
    }
    if (std::holds_alternative<SourceOwnedClosedRimV1>(receipt.intent)) {
        const auto selected = std::find_if(proof.boundaryUses().begin(),
            proof.boundaryUses().end(), [](const BoundaryUse& use) { return use.selected; });
        if (selected == proof.boundaryUses().end() || receipt.entries.size() != 1
            || receipt.entries.front().direction != selected->direction)
            return Refusal::ReplayMismatch;
    }
    return Refusal::None;
}

inline et::Refusal MapToB1(Refusal refusal) noexcept {
    using B1 = et::Refusal;
    switch (refusal) {
    case Refusal::None: return B1::None;
    case Refusal::FaceMissing: case Refusal::IncompleteBoundary: case Refusal::ExactCount: return B1::AnchorMissing;
    case Refusal::FaceAmbiguous: case Refusal::SplitEdge: return B1::AnchorAmbiguous;
    case Refusal::InvalidFaceUse: case Refusal::OwnershipMismatch: case Refusal::PartialCircle:
    case Refusal::SplineIdentityUnavailable: return B1::UnsupportedEdge;
    case Refusal::ReplayMismatch: return B1::ReplayMismatch;
    case Refusal::InvalidIntent: return B1::MalformedCarrier;
    case Refusal::UnsupportedCarrier: case Refusal::AdmissionUnavailable: return B1::UnsupportedBase;
    case Refusal::StaleSource: return B1::StaleSnapshot;
    case Refusal::UnsupportedTransform: return B1::UnsupportedTransform;
    case Refusal::Budget: case Refusal::TruncatedDiscovery: return B1::Budget;
    case Refusal::Cancelled: return B1::Cancelled;
    case Refusal::Busy: return B1::Busy;
    default: return B1::BuildFailed;
    }
}
} // namespace core3d::retained_face_selector
