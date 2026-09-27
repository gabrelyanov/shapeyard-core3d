#pragma once
#include "PatternDefinition.hxx"

#include <array>
#include <cmath>
#include <vector>

namespace core3d::pattern {
using Matrix = std::array<double, 16>;

struct Placement {
    UUID identity{};
    std::uint64_t localID = 0;
    Coordinate coordinate;
    Matrix worldFrame{};
};

inline Matrix IdentityMatrix() noexcept {
    return {{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}};
}

inline Matrix Multiply(const Matrix& left, const Matrix& right) noexcept {
    Matrix result{};
    for (unsigned row = 0; row < 4; ++row) for (unsigned column = 0; column < 4; ++column)
        for (unsigned index = 0; index < 4; ++index)
            result[row * 4 + column] += left[row * 4 + index] * right[index * 4 + column];
    return result;
}

inline Matrix Translation(double x, double y, double z) noexcept {
    Matrix result = IdentityMatrix(); result[3] = x; result[7] = y; result[11] = z; return result;
}

inline Matrix AxisRotation(Axis axis, double angle) noexcept {
    Matrix result = IdentityMatrix(); const double c = std::cos(angle), s = std::sin(angle);
    if (axis == Axis::X) { result[5] = c; result[6] = -s; result[9] = s; result[10] = c; }
    else if (axis == Axis::Y) { result[0] = c; result[2] = s; result[8] = -s; result[10] = c; }
    else { result[0] = c; result[1] = -s; result[4] = s; result[5] = c; }
    return result;
}

inline std::array<double, 3> AxisVector(Axis axis, double magnitude) noexcept {
    std::array<double, 3> result{{0, 0, 0}}; result[unsigned(axis)] = magnitude; return result;
}

inline double RadialAngle(const Definition& value, std::uint32_t ordinal) noexcept {
    const bool closed = std::abs(std::abs(value.sweepRadians) - TwoPi) <= 1e-12;
    const double divisor = closed ? double(value.columnCount) : double(value.columnCount - 1);
    return value.sweepRadians * double(ordinal) / divisor;
}

inline bool BuildPlacements(const Definition& value, std::vector<Placement>& output) noexcept {
    output.clear();
    try {
        if (!Valid(value)) return false;
        std::vector<Placement> placements; placements.reserve(value.members.size());
        for (const Member& member : value.members) {
            if (member.state == MemberState::Suppressed) continue;
            Matrix local = IdentityMatrix();
            if (value.kind == Kind::Linear) {
                const auto offset = AxisVector(value.columnAxis,
                    value.columnSpacing * double(member.coordinate.column));
                local = Translation(offset[0], offset[1], offset[2]);
            } else if (value.kind == Kind::Grid) {
                const auto column = AxisVector(value.columnAxis,
                    value.columnSpacing * double(member.coordinate.column));
                const auto row = AxisVector(value.rowAxis,
                    value.rowSpacing * double(member.coordinate.row));
                local = Translation(column[0] + row[0], column[1] + row[1], column[2] + row[2]);
            } else {
                const auto& pivot = value.radialPivotLocal;
                local = Multiply(Translation(pivot[0], pivot[1], pivot[2]),
                    Multiply(AxisRotation(value.columnAxis,
                        RadialAngle(value, member.coordinate.column)),
                        Translation(-pivot[0], -pivot[1], -pivot[2])));
            }
            const Matrix world = Multiply(value.sourceFrame, local);
            for (double scalar : world) if (!std::isfinite(scalar)) return false;
            placements.push_back({member.identity, member.localID, member.coordinate, world});
        }
        output = std::move(placements); return true;
    } catch (...) { output.clear(); return false; }
}
} // namespace core3d::pattern
