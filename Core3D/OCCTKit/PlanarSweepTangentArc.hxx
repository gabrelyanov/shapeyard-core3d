#pragma once

#include <gp_Pnt2d.hxx>
#include <algorithm>
#include <cmath>
#include <optional>

namespace core3d::planar_sweep {
struct TangentArcResult {
    gp_Pnt2d trimA, trimB, center;
    double radius = 0, startDegrees = 0, sweepDegrees = 0;
};

// Minor circular fillet traversed a -> corner -> b. No document or solid
// authority: callers must still submit the completed path to Inspect.
inline std::optional<TangentArcResult> TangentArcBetween(
    const gp_Pnt2d& a, const gp_Pnt2d& corner, const gp_Pnt2d& b,
    double radius) noexcept {
    const auto finite = [](const gp_Pnt2d& p) {
        return std::isfinite(p.X()) && std::isfinite(p.Y());
    };
    if (!finite(a) || !finite(corner) || !finite(b)
        || !std::isfinite(radius) || radius <= 0) return std::nullopt;
    double ux = corner.X() - a.X(), uy = corner.Y() - a.Y();
    double vx = b.X() - corner.X(), vy = b.Y() - corner.Y();
    const double lengthA = std::hypot(ux, uy), lengthB = std::hypot(vx, vy);
    if (!std::isfinite(lengthA) || !std::isfinite(lengthB)
        || lengthA == 0 || lengthB == 0) return std::nullopt;
    ux /= lengthA; uy /= lengthA; vx /= lengthB; vy /= lengthB;
    const double cross = ux * vy - uy * vx;
    const double dot = std::clamp(ux * vx + uy * vy, -1.0, 1.0);
    // Both straight and reversing parallel lines have no unique minor fillet.
    if (std::abs(cross) <= 1e-12) return std::nullopt;
    const double turn = std::atan2(cross, dot);
    const double trim = radius * std::tan(std::abs(turn) / 2);
    if (!std::isfinite(trim) || trim <= 0 || trim > lengthA || trim > lengthB)
        return std::nullopt;
    TangentArcResult result;
    result.trimA = gp_Pnt2d(corner.X() - ux * trim, corner.Y() - uy * trim);
    result.trimB = gp_Pnt2d(corner.X() + vx * trim, corner.Y() + vy * trim);
    const double side = cross > 0 ? 1 : -1;
    result.center = gp_Pnt2d(result.trimA.X() - side * uy * radius,
                            result.trimA.Y() + side * ux * radius);
    const double degrees = 180 / std::acos(-1.0);
    result.radius = radius;
    result.startDegrees = std::atan2(-side * ux, side * uy) * degrees;
    result.sweepDegrees = turn * degrees;
    if (!finite(result.trimA) || !finite(result.trimB) || !finite(result.center))
        return std::nullopt;
    return result;
}
} // namespace core3d::planar_sweep
