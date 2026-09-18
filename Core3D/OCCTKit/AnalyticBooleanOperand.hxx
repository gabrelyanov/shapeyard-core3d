#pragma once
#include <array>
#include <cmath>
#include <cstdint>

namespace core3d::analytic_boolean {
// Closed typed recipe fragment, NOT a native permission/owner binding. The
// future saved feature envelope owns its feature UUID and original base recipe.
enum class Operation : std::uint8_t { Difference = 1 };
enum class OperandKind : std::uint8_t { Cylinder = 1, CylinderRing = 2 };
enum class Extent : std::uint8_t { ThroughAll = 1 };
enum class Axis : std::uint8_t { X = 0, Y = 1, Z = 2 };
struct Operand {
    std::uint32_t identifier = 1; // Local stable operand ID, not a feature UUID.
    OperandKind kind = OperandKind::Cylinder;
    Extent extent = Extent::ThroughAll;
    Axis axis = Axis::Z;
    // Authored point and radius in the original document unit/frame. The axial
    // point component is retained, although a through-all line is independent
    // of its axial origin. No reader or builder rewrites omitted/raw fields.
    std::array<double,3> point{0,0,0};
    double radius = 0;
    double boltCircleRadius = 0;
    std::uint32_t count = 0;
    double hostRadiusRatio = 0;
};
struct Recipe {
    std::uint32_t schema = 1;
    Operation operation = Operation::Difference;
    double metersPerUnit = 0;
    Operand tool;
};
inline bool Inspect(const Recipe& r) noexcept {
    if (r.schema != 1 || r.operation != Operation::Difference || r.tool.identifier == 0
        || (r.tool.kind != OperandKind::Cylinder && r.tool.kind != OperandKind::CylinderRing)
        || r.tool.extent != Extent::ThroughAll
        || static_cast<unsigned>(r.tool.axis) > 2 || !std::isfinite(r.metersPerUnit)
        || r.metersPerUnit <= 0) return false;
    const double mm = r.metersPerUnit * 1000;
    if (!std::isfinite(mm) || mm <= 0 || !std::isfinite(r.tool.radius)
        || !std::isfinite(r.tool.radius * mm) || r.tool.radius * mm < .001
        || r.tool.radius * mm > 1e6) return false;
    for (double p : r.tool.point)
        if (!std::isfinite(p) || !std::isfinite(p * mm) || std::abs(p * mm) > 1e6) return false;
    if(r.tool.kind==OperandKind::CylinderRing){
        if(!std::isfinite(r.tool.boltCircleRadius)||r.tool.boltCircleRadius<=0||r.tool.boltCircleRadius>1e6
            ||!std::isfinite(r.tool.hostRadiusRatio)||r.tool.hostRadiusRatio<=0||r.tool.hostRadiusRatio>1
            ||r.tool.count<3||r.tool.count>16)return false;
    }else if(r.tool.boltCircleRadius!=0||std::signbit(r.tool.boltCircleRadius)
        ||r.tool.count!=0||r.tool.hostRadiusRatio!=0||std::signbit(r.tool.hostRadiusRatio))return false;
    return true;
}
} // namespace core3d::analytic_boolean
