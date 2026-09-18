#pragma once
#include "AnalyticBooleanOperand.hxx"
#include <algorithm>
#include <cfenv>
#include <limits>
#include <Precision.hxx>

namespace core3d::analytic_boolean_ring {
inline constexpr std::uint32_t kMinimumCount=3;
inline constexpr std::uint32_t kMaximumCount=16;
inline constexpr std::uint32_t kMaximumExpandedDisks=32;
struct Ring {
    std::uint32_t identifier=1;
    analytic_boolean::Axis axis=analytic_boolean::Axis::Z;
    std::array<double,3> center{};
    double boltCircleRadius=0,worldHoleRadiusMM=0,hostRadiusRatio=0;
    std::uint32_t count=0;
};
enum class Status : std::uint8_t { Clear, InvalidCount, InvalidRadii, OffAxis,
    OverlappingHoles, OutsideOrInsufficientLigament };
inline Ring FromOperand(const analytic_boolean::Operand& t,double metersPerUnit){
    return {t.identifier,t.axis,t.point,t.boltCircleRadius,t.radius*metersPerUnit*1000,t.hostRadiusRatio,t.count};
}
inline Status Inspect(const Ring& r,double metersPerUnit) noexcept {
    if(r.count<kMinimumCount||r.count>kMaximumCount)return Status::InvalidCount;
    if(unsigned(r.axis)>2)return Status::OffAxis;
    const double mm=metersPerUnit*1000;
    if(!std::isfinite(mm)||mm<=0||!std::isfinite(r.worldHoleRadiusMM)
        ||r.worldHoleRadiusMM<.001||r.worldHoleRadiusMM>1e6)return Status::InvalidRadii;
    analytic_boolean::Recipe recipe;recipe.metersPerUnit=metersPerUnit;
    auto& t=recipe.tool;t.identifier=r.identifier;t.kind=analytic_boolean::OperandKind::CylinderRing;
    t.axis=r.axis;t.point=r.center;t.radius=r.worldHoleRadiusMM/mm;
    t.boltCircleRadius=r.boltCircleRadius;t.hostRadiusRatio=r.hostRadiusRatio;t.count=r.count;
    if(!analytic_boolean::Inspect(recipe)||std::fegetround()!=FE_TONEAREST)return Status::InvalidRadii;
    const double extent=r.boltCircleRadius*mm;
    if(!std::isfinite(extent)||extent>1e6)return Status::InvalidRadii;
    // Conservative error allowance here supplements the interval all-pairs
    // clearance proof on the expanded disk set at program admission.
    const double gap=2*extent*std::sin(std::acos(-1.0)/r.count)-2*r.worldHoleRadiusMM;
    const double error=64*std::numeric_limits<double>::epsilon()*(extent+r.worldHoleRadiusMM);
    const double separation=std::max(.002,std::max(Precision::Confusion()*32*mm,1e-5));
    if(!std::isfinite(gap)||gap-error<=separation)return Status::OverlappingHoles;
    return Status::Clear;
}
inline analytic_boolean::Operand Expand(const Ring& r,std::uint32_t ordinal,double metersPerUnit) noexcept {
    analytic_boolean::Operand out;out.identifier=0;
    if(ordinal>=r.count||Inspect(r,metersPerUnit)!=Status::Clear)return out;
    out.identifier=r.identifier;out.axis=r.axis;out.point=r.center;
    out.radius=r.worldHoleRadiusMM/(metersPerUnit*1000);
    // X: Y/Z; Y: X/Z; Z: X/Y. Hole zero is on +local X whenever X is radial.
    const unsigned u=r.axis==analytic_boolean::Axis::X?1:0;
    const unsigned v=r.axis==analytic_boolean::Axis::Z?1:2;
    const double angle=2*std::acos(-1.0)*ordinal/r.count;
    out.point[u]+=r.boltCircleRadius*std::cos(angle);
    out.point[v]+=r.boltCircleRadius*std::sin(angle);
    analytic_boolean::Recipe check;check.metersPerUnit=metersPerUnit;check.tool=out;
    if(!analytic_boolean::Inspect(check))out.identifier=0;
    return out;
}
} // namespace core3d::analytic_boolean_ring
