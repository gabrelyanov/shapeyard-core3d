#include "../Core3D/UI/Core3DManipulator.hpp"
#include <cassert>
#include <iomanip>
#include <iostream>
#include <limits>

namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double turn = 2.0 * pi;
void close(const double actual, const double expected) {
    assert(std::abs(actual - expected) < 1e-12);
}
double legacy(const double raw, const double previous) {
    return raw * previous < 0 && std::abs(raw) < pi / 2
        ? (previous > 0 ? -1.0 : 1.0) * (turn - raw) : raw;
}
}

int main() {
    using core3d::UnwrapManipulatorAngle;
    const double epsilon = std::numeric_limits<double>::epsilon();
    // Either sign of zero/roundoff must leave the first +/-60-degree drag
    // untouched. Also covers reversing through the gesture origin.
    for (const double seed : {0.0, -0.0, epsilon, -epsilon, 1e-12, -1e-12}) {
        for (const double raw : {-pi / 3, pi / 3, -1e-8, 1e-8}) {
            assert(UnwrapManipulatorAngle(raw, seed) == raw);
        }
    }
    close(UnwrapManipulatorAngle(-0.1, 0.1), -0.1);
    close(UnwrapManipulatorAngle(0.1, -0.1), 0.1);
    // Cross the branch cut in both directions; no sign reflection.
    close(UnwrapManipulatorAngle(-179 * pi / 180, 179 * pi / 180), 181 * pi / 180);
    close(UnwrapManipulatorAngle(179 * pi / 180, -179 * pi / 180), -181 * pi / 180);
    assert(UnwrapManipulatorAngle(pi, 0) == pi);
    assert(UnwrapManipulatorAngle(-pi, 0) == -pi);
    // Multiple turns, including a sample exactly at the starting point, then
    // reverse direction. Preserve continuous angles and the raw rotation matrix.
    for (const double direction : {-1.0, 1.0}) {
        double previous = 0.0;
        for (int step = 1; step <= 216; ++step) {
            const double expected = direction * step * pi / 36;
            const double raw = std::remainder(expected, turn);
            const double actual = UnwrapManipulatorAngle(raw, previous);
            close(actual, expected);
            close(std::sin(actual), std::sin(raw));
            close(std::cos(actual), std::cos(raw));
            previous = actual;
        }
        for (int step = 215; step >= -72; --step) {
            const double expected = direction * step * pi / 36;
            const double raw = std::remainder(expected, turn);
            const double actual = UnwrapManipulatorAngle(raw, previous);
            close(actual, expected);
            previous = actual;
        }
    }
    // Deterministic discrimination of the old product defect (synthetic inputs,
    // not a claim that the device's seed has been measured in this process).
    const double raw = pi / 3;
    const double seed = -epsilon;
    const double oldAngle = legacy(raw, seed);
    const double corrected = UnwrapManipulatorAngle(raw, seed);
    assert(std::sin(oldAngle) < 0);
    assert(std::sin(corrected) > 0);
    std::cout << std::setprecision(17)
        << "seed=" << seed << " raw=" << raw
        << " legacy=" << oldAngle << " corrected=" << corrected
        << " legacy_sine=" << std::sin(oldAngle)
        << " corrected_sine=" << std::sin(corrected) << '\n';
    std::cout << "PASS manipulator_angle_wrap_tests\n";
}
