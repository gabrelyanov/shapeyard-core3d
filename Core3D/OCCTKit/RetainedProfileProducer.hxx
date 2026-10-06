#pragma once

#include <Core3D/RetainedEdgeTreatmentBuild.hxx>

#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS_Shape.hxx>

#include <atomic>
#include <cstdint>

namespace core3d::retained_profile_producer {
namespace et = core3d::retained_edge_treatment;
namespace tb = core3d::retained_topology_budget;

// This is the existing source-producer domain, factored without widening it.
// A caller must still prove that the shape is a newly produced private Profile
// product; this predicate grants no authority over captured or shared shapes.
inline bool EligiblePrivateProfileSource(bool revolve, bool spline, bool circle,
    bool curves, bool constructionFrame) noexcept {
    return !revolve && !spline && (circle || curves || constructionFrame);
}

enum class FixedPointStatus : std::uint8_t {
    KeptOriginal = 0,
    Adopted,
    Cancelled,
    BudgetExceeded
};

namespace detail {
inline FixedPointStatus ReserveReadReconstruction(const TopoDS_Shape& input,
    const std::atomic_bool& stop, et::ReplayBudget& budget) noexcept {
    tb::Census census;
    const tb::WalkStatus walked = tb::CensusTopology(input, budget, stop, census,
        tb::Site::C12Detachment, false);
    if (walked == tb::WalkStatus::Cancelled || stop.load()) {
        return FixedPointStatus::Cancelled;
    }
    if (walked == tb::WalkStatus::BudgetDenied || budget.exhausted) {
        return FixedPointStatus::BudgetExceeded;
    }
    if (walked != tb::WalkStatus::Completed) {
        return FixedPointStatus::KeptOriginal;
    }
    if (!tb::ReserveTraversal(census, budget, tb::Site::C12Detachment)) {
        return FixedPointStatus::BudgetExceeded;
    }
    return FixedPointStatus::Adopted;
}
} // namespace detail

// Finite producer materialization: y1=R(privateProduct), y2=R(y1), then
// D(y1)==D(y2). There is no retry, geometry fitting, equality relaxation, or
// normalization of an observed/captured owner. A failed finite attempt leaves
// output bound to privateProduct so the unchanged downstream equality gate can
// decide. Every caller must use the enclosing operation's budget and stop token.
inline FixedPointStatus MaterializePrivateProfileProducerFixedPoint(
    const TopoDS_Shape& privateProduct, const std::atomic_bool& stop,
    et::ReplayBudget& budget, TopoDS_Shape& output) noexcept {
    output = privateProduct;
    if (stop.load()) return FixedPointStatus::Cancelled;
    if (budget.exhausted) return FixedPointStatus::BudgetExceeded;
    if (privateProduct.IsNull()) return FixedPointStatus::KeptOriginal;

    // ReadbackGeometry charges its write/source traversal, reopened census and
    // analyzer reservation. B8 separately charges the binary reconstruction
    // pass before each added readback, using a census of that exact input.
    FixedPointStatus reserved =
        detail::ReserveReadReconstruction(privateProduct, stop, budget);
    if (reserved != FixedPointStatus::Adopted) return reserved;

    TopoDS_Shape y1;
    if (!et::detail::ReadbackGeometry(privateProduct, budget, y1)) {
        if (stop.load()) return FixedPointStatus::Cancelled;
        if (budget.exhausted) return FixedPointStatus::BudgetExceeded;
        return FixedPointStatus::KeptOriginal;
    }
    if (stop.load()) return FixedPointStatus::Cancelled;
    if (budget.exhausted) return FixedPointStatus::BudgetExceeded;
    if (y1.IsNull() || y1.ShapeType() != TopAbs_SOLID) {
        return FixedPointStatus::KeptOriginal;
    }

    reserved = detail::ReserveReadReconstruction(y1, stop, budget);
    if (reserved != FixedPointStatus::Adopted) return reserved;

    TopoDS_Shape y2;
    if (!et::detail::ReadbackGeometry(y1, budget, y2)) {
        if (stop.load()) return FixedPointStatus::Cancelled;
        if (budget.exhausted) return FixedPointStatus::BudgetExceeded;
        return FixedPointStatus::KeptOriginal;
    }
    if (stop.load()) return FixedPointStatus::Cancelled;
    if (budget.exhausted) return FixedPointStatus::BudgetExceeded;

    et::Digest y1Digest{}, y2Digest{};
    if (!et::detail::CommitGeometry(y1, budget, y1Digest)) {
        if (stop.load()) return FixedPointStatus::Cancelled;
        if (budget.exhausted) return FixedPointStatus::BudgetExceeded;
        return FixedPointStatus::KeptOriginal;
    }
    if (stop.load()) return FixedPointStatus::Cancelled;
    if (budget.exhausted) return FixedPointStatus::BudgetExceeded;
    if (!et::detail::CommitGeometry(y2, budget, y2Digest)) {
        if (stop.load()) return FixedPointStatus::Cancelled;
        if (budget.exhausted) return FixedPointStatus::BudgetExceeded;
        return FixedPointStatus::KeptOriginal;
    }
    if (stop.load()) return FixedPointStatus::Cancelled;
    if (budget.exhausted) return FixedPointStatus::BudgetExceeded;
    if (y1Digest != y2Digest) return FixedPointStatus::KeptOriginal;

    output = y1;
    return FixedPointStatus::Adopted;
}

} // namespace core3d::retained_profile_producer
