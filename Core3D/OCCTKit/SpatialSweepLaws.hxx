#pragma once
#include "SpatialSweepDefinition.hxx"
#include <algorithm>
#include <cmath>
#include <vector>

namespace core3d::spatial_sweep {
struct ArcLengthStation {
    double parameter = 0;
    double length = 0;
};

inline bool ValidArcLengthTable(const std::vector<ArcLengthStation>& table) noexcept {
    if (table.size() < 2 || !Finite(table.front().parameter)
        || !Finite(table.front().length) || table.front().length != 0) return false;
    for (std::size_t index = 1; index < table.size(); ++index)
        if (!Finite(table[index].parameter) || !Finite(table[index].length)
            || !(table[index - 1].parameter < table[index].parameter)
            || !(table[index - 1].length < table[index].length)) return false;
    return true;
}

inline bool ArcLengthAt(const std::vector<ArcLengthStation>& table, double parameter,
                        double& length) noexcept {
    length = 0;
    if (!ValidArcLengthTable(table) || !Finite(parameter)
        || parameter < table.front().parameter || parameter > table.back().parameter) return false;
    const auto upper = std::upper_bound(table.begin(), table.end(), parameter,
        [](double value, const ArcLengthStation& station) { return value < station.parameter; });
    if (upper == table.begin()) return false;
    if (upper == table.end()) { length = table.back().length; return true; }
    const auto& a = *(upper - 1); const auto& b = *upper;
    const double q = (parameter - a.parameter) / (b.parameter - a.parameter);
    length = a.length + q * (b.length - a.length);
    return Finite(length);
}

struct LawEvaluation {
    double radius = 0;
    double firstDerivative = 0;
    double secondDerivative = 0;
};

// The table supplies deterministic dense-output arc length. Derivatives use
// the exact chain rule values from the authored curve adapter, never u/L.
inline bool EvaluateRadiusLaw(const RadiusLaw& law, double arcLength, double totalLength,
                              double speed, double speedDerivative,
                              LawEvaluation& output) noexcept {
    output = {};
    if (!Finite(arcLength) || !Finite(totalLength) || totalLength <= 0
        || arcLength < 0 || arcLength > totalLength || !Finite(speed) || speed <= 0
        || !Finite(speedDerivative)) return false;
    if (law.kind == RadiusLawKind::Constant) {
        if (!Finite(law.startRadius) || law.startRadius <= 0
            || law.startRadius != law.endRadius) return false;
        output.radius = law.startRadius;
        return true;
    }
    if (law.kind != RadiusLawKind::LinearArcLength || !Finite(law.startRadius)
        || !Finite(law.endRadius) || law.startRadius <= 0 || law.endRadius <= 0) return false;
    const double delta = law.endRadius - law.startRadius;
    output.radius = law.startRadius + delta * arcLength / totalLength;
    output.firstDerivative = delta * speed / totalLength;
    output.secondDerivative = delta * speedDerivative / totalLength;
    return Finite(output.radius) && Finite(output.firstDerivative)
        && Finite(output.secondDerivative);
}
} // namespace core3d::spatial_sweep
