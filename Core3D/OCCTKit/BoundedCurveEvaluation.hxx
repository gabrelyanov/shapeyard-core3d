#pragma once

// Deterministic evaluation shared by retained consumers of C1's canonical
// SYCV value. It does not manufacture or rewrite curve identity.
#include "BoundedCurveCodec.hxx"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace core3d::bounded_curve {
using Vector3 = std::array<double, 3>;

struct Evaluation {
    Vector3 point{};
    Vector3 derivative{};
    double parameter = 0;
};

enum class EvaluationRefusal : std::uint8_t {
    None = 0, InvalidCurve, InvalidDomain, NonFinite, ZeroTangent,
    Cusp, IntegrationDidNotConverge, InversionDidNotConverge
};

inline Vector3 Add(const Vector3& a, const Vector3& b) noexcept {
    return {{a[0] + b[0], a[1] + b[1], a[2] + b[2]}};
}
inline Vector3 Subtract(const Vector3& a, const Vector3& b) noexcept {
    return {{a[0] - b[0], a[1] - b[1], a[2] - b[2]}};
}
inline Vector3 Scale(const Vector3& value, double factor) noexcept {
    return {{value[0] * factor, value[1] * factor, value[2] * factor}};
}
inline double Norm(const Vector3& value) noexcept {
    return std::sqrt(Dot(value, value));
}
inline bool Normalize(const Vector3& value, double minimum, Vector3& output) noexcept {
    const double length = Norm(value);
    if (!Finite(length) || length <= minimum) { output = {}; return false; }
    output = Scale(value, 1.0 / length); return true;
}

inline bool ExpandedKnots(const Definition& definition,
                          std::vector<double>& output) noexcept {
    output.clear();
    try {
        if (Validate(definition) != Refusal::None) return false;
        for (const Knot& knot : definition.knots)
            output.insert(output.end(), knot.multiplicity, knot.value);
        return output.size() == definition.controlPoints.size()
            + std::size_t(definition.degree) + 1;
    } catch (...) { output.clear(); return false; }
}

inline bool ParameterDomain(const Definition& definition, double& first,
                            double& last) noexcept {
    std::vector<double> knots;
    if (!ExpandedKnots(definition, knots)) return false;
    const std::size_t degree = definition.degree;
    first = knots[degree]; last = knots[definition.controlPoints.size()];
    return Finite(first) && Finite(last) && first < last;
}

inline double Basis(const std::vector<double>& knots, std::size_t index,
                    unsigned degree, double parameter) noexcept {
    if (index + degree + 1 >= knots.size()) return 0;
    if (parameter == knots.back())
        return index + degree + 1 == knots.size() - 1 ? 1 : 0;
    if (degree == 0) {
        if (knots[index] <= parameter && parameter < knots[index + 1]) return 1;
        return 0;
    }
    double left = 0, right = 0;
    const double leftDenominator = knots[index + degree] - knots[index];
    const double rightDenominator = knots[index + degree + 1] - knots[index + 1];
    if (leftDenominator > 0)
        left = (parameter - knots[index]) / leftDenominator
            * Basis(knots, index, degree - 1, parameter);
    if (rightDenominator > 0)
        right = (knots[index + degree + 1] - parameter) / rightDenominator
            * Basis(knots, index + 1, degree - 1, parameter);
    return left + right;
}

inline double BasisDerivative(const std::vector<double>& knots, std::size_t index,
                              unsigned degree, double parameter) noexcept {
    if (degree == 0 || index + degree + 1 >= knots.size()) return 0;
    double left = 0, right = 0;
    const double leftDenominator = knots[index + degree] - knots[index];
    const double rightDenominator = knots[index + degree + 1] - knots[index + 1];
    if (leftDenominator > 0)
        left = double(degree) / leftDenominator
            * Basis(knots, index, degree - 1, parameter);
    if (rightDenominator > 0)
        right = double(degree) / rightDenominator
            * Basis(knots, index + 1, degree - 1, parameter);
    return left - right;
}

inline Vector3 ToWorldPoint(const Frame& frame, const Vector3& local) noexcept {
    return Add(frame.origin, Add(Scale(frame.xAxis, local[0]),
        Add(Scale(frame.yAxis, local[1]), Scale(frame.zAxis, local[2]))));
}
inline Vector3 ToWorldVector(const Frame& frame, const Vector3& local) noexcept {
    return Add(Scale(frame.xAxis, local[0]),
        Add(Scale(frame.yAxis, local[1]), Scale(frame.zAxis, local[2])));
}

inline EvaluationRefusal Evaluate(const Definition& definition, double parameter,
                                  Evaluation& output) noexcept {
    output = {};
    try {
        if (Validate(definition) != Refusal::None || definition.domain != Domain::Path3D)
            return EvaluationRefusal::InvalidCurve;
        std::vector<double> knots;
        if (!ExpandedKnots(definition, knots)) return EvaluationRefusal::InvalidCurve;
        const double first = knots[definition.degree];
        const double last = knots[definition.controlPoints.size()];
        if (!Finite(parameter) || parameter < first || parameter > last)
            return EvaluationRefusal::InvalidDomain;
        // B-spline derivatives at a clamped maximum use the left limit. The
        // canonical domain still reports the exact saved endpoint parameter.
        const double evaluationParameter = parameter == last
            ? std::nextafter(last, first) : parameter;
        Vector3 numerator{}, derivativeNumerator{};
        double denominator = 0, derivativeDenominator = 0;
        for (std::size_t index = 0; index < definition.controlPoints.size(); ++index) {
            const double weight = definition.weights.empty() ? 1 : definition.weights[index];
            const double basis = Basis(knots, index, definition.degree, parameter);
            const double derivative = BasisDerivative(knots, index, definition.degree,
                                                       evaluationParameter);
            numerator = Add(numerator, Scale(definition.controlPoints[index].local, basis * weight));
            derivativeNumerator = Add(derivativeNumerator,
                Scale(definition.controlPoints[index].local, derivative * weight));
            denominator += basis * weight; derivativeDenominator += derivative * weight;
        }
        if (!Finite(denominator) || denominator <= 0) return EvaluationRefusal::NonFinite;
        const Vector3 localPoint = Scale(numerator, 1.0 / denominator);
        const Vector3 localDerivative = Scale(Subtract(
            Scale(derivativeNumerator, denominator),
            Scale(numerator, derivativeDenominator)), 1.0 / (denominator * denominator));
        output.point = ToWorldPoint(definition.frame, localPoint);
        output.derivative = ToWorldVector(definition.frame, localDerivative);
        output.parameter = parameter;
        for (double value : output.point) if (!Finite(value)) return EvaluationRefusal::NonFinite;
        for (double value : output.derivative) if (!Finite(value)) return EvaluationRefusal::NonFinite;
        return EvaluationRefusal::None;
    } catch (...) { output = {}; return EvaluationRefusal::NonFinite; }
}

inline EvaluationRefusal AuditTangents(const Definition& definition,
                                       double minimumTangent,
                                       double maximumCornerRadians) noexcept {
    if (!Finite(minimumTangent) || minimumTangent <= 0
        || !Finite(maximumCornerRadians) || maximumCornerRadians <= 0
        || maximumCornerRadians >= 3.14159265358979323846)
        return EvaluationRefusal::InvalidDomain;
    std::vector<double> knots;
    if (!ExpandedKnots(definition, knots) || definition.domain != Domain::Path3D)
        return EvaluationRefusal::InvalidCurve;
    const double first = knots[definition.degree];
    const double last = knots[definition.controlPoints.size()];
    Vector3 previous{}; bool havePrevious = false;
    for (std::size_t span = definition.degree;
         span < definition.controlPoints.size(); ++span) {
        const double low = knots[span], high = knots[span + 1];
        if (!(low < high) || high < first || low > last) continue;
        for (unsigned step = 0; step <= 8; ++step) {
            double parameter = low + (high - low) * double(step) / 8.0;
            if (step == 8 && high < last)
                parameter = std::nextafter(high, low);
            Evaluation value;
            const auto status = Evaluate(definition, parameter, value);
            Vector3 tangent{};
            if (status != EvaluationRefusal::None
                || !Normalize(value.derivative, minimumTangent, tangent))
                return EvaluationRefusal::ZeroTangent;
            if (havePrevious) {
                const double cosine = std::clamp(Dot(previous, tangent), -1.0, 1.0);
                if (std::acos(cosine) > maximumCornerRadians) return EvaluationRefusal::Cusp;
            }
            previous = tangent; havePrevious = true;
        }
    }
    return havePrevious ? EvaluationRefusal::None : EvaluationRefusal::InvalidCurve;
}

namespace detail {
inline bool Speed(const Definition& definition, double parameter,
                  double minimumTangent, double& output) noexcept {
    Evaluation value;
    if (Evaluate(definition, parameter, value) != EvaluationRefusal::None) return false;
    output = Norm(value.derivative);
    return Finite(output) && output > minimumTangent;
}

inline bool AdaptiveSimpson(const Definition& definition, double left, double right,
                            double leftSpeed, double middleSpeed, double rightSpeed,
                            double whole, double tolerance, double minimumTangent,
                            unsigned depth, double& output) noexcept {
    const double middle = 0.5 * (left + right);
    const double leftMiddle = 0.5 * (left + middle);
    const double rightMiddle = 0.5 * (middle + right);
    double leftMiddleSpeed = 0, rightMiddleSpeed = 0;
    if (!Speed(definition, leftMiddle, minimumTangent, leftMiddleSpeed)
        || !Speed(definition, rightMiddle, minimumTangent, rightMiddleSpeed)) return false;
    const double leftEstimate = (middle - left)
        * (leftSpeed + 4 * leftMiddleSpeed + middleSpeed) / 6.0;
    const double rightEstimate = (right - middle)
        * (middleSpeed + 4 * rightMiddleSpeed + rightSpeed) / 6.0;
    const double delta = leftEstimate + rightEstimate - whole;
    if (!Finite(delta)) return false;
    if (std::abs(delta) <= 15 * tolerance) {
        output = leftEstimate + rightEstimate + delta / 15.0;
        return Finite(output) && output >= 0;
    }
    if (depth == 0) return false;
    double leftResult = 0, rightResult = 0;
    if (!AdaptiveSimpson(definition, left, middle, leftSpeed, leftMiddleSpeed,
            middleSpeed, leftEstimate, tolerance * 0.5, minimumTangent,
            depth - 1, leftResult)
        || !AdaptiveSimpson(definition, middle, right, middleSpeed, rightMiddleSpeed,
            rightSpeed, rightEstimate, tolerance * 0.5, minimumTangent,
            depth - 1, rightResult)) return false;
    output = leftResult + rightResult; return Finite(output);
}
} // namespace detail

inline EvaluationRefusal ArcLength(const Definition& definition, double first,
                                   double last, double tolerance,
                                   double minimumTangent, double& output) noexcept {
    output = 0;
    if (!Finite(first) || !Finite(last) || !Finite(tolerance) || tolerance <= 0
        || !Finite(minimumTangent) || minimumTangent <= 0 || first > last)
        return EvaluationRefusal::InvalidDomain;
    if (first == last) return EvaluationRefusal::None;
    std::vector<double> expanded;
    double domainFirst = 0, domainLast = 0;
    if (!ExpandedKnots(definition, expanded)
        || !ParameterDomain(definition, domainFirst, domainLast)
        || first < domainFirst || last > domainLast) return EvaluationRefusal::InvalidDomain;
    std::vector<std::pair<double, double>> spans;
    for (std::size_t index = definition.degree;
         index < definition.controlPoints.size(); ++index) {
        const double low = std::max(first, expanded[index]);
        const double high = std::min(last, expanded[index + 1]);
        if (low < high) spans.push_back({low, high});
    }
    if (spans.empty()) return EvaluationRefusal::InvalidDomain;
    const double spanTolerance = tolerance / double(spans.size());
    for (const auto& span : spans) {
        const double middle = 0.5 * (span.first + span.second);
        const double rightProbe = span.second < domainLast
            ? std::nextafter(span.second, span.first) : span.second;
        double a = 0, b = 0, m = 0;
        if (!detail::Speed(definition, span.first, minimumTangent, a)
            || !detail::Speed(definition, middle, minimumTangent, m)
            || !detail::Speed(definition, rightProbe, minimumTangent, b))
            return EvaluationRefusal::ZeroTangent;
        const double whole = (span.second - span.first) * (a + 4 * m + b) / 6.0;
        double part = 0;
        if (!detail::AdaptiveSimpson(definition, span.first, span.second, a, m, b,
                whole, spanTolerance, minimumTangent, 20, part))
            return EvaluationRefusal::IntegrationDidNotConverge;
        output += part;
    }
    return Finite(output) && output > 0
        ? EvaluationRefusal::None : EvaluationRefusal::ZeroTangent;
}

inline EvaluationRefusal InvertArcLength(const Definition& definition,
                                         double target, double total,
                                         double tolerance, double minimumTangent,
                                         double& parameter,
                                         double& measuredLength) noexcept {
    parameter = 0; measuredLength = 0;
    double first = 0, last = 0;
    if (!ParameterDomain(definition, first, last) || !Finite(target)
        || !Finite(total) || total <= 0 || target < 0 || target > total)
        return EvaluationRefusal::InvalidDomain;
    if (target == 0) { parameter = first; return EvaluationRefusal::None; }
    if (target == total) { parameter = last; measuredLength = total; return EvaluationRefusal::None; }
    double low = first, high = last;
    for (unsigned iteration = 0; iteration < 72; ++iteration) {
        const double middle = 0.5 * (low + high);
        double length = 0;
        const auto status = ArcLength(definition, first, middle,
            tolerance * 0.125, minimumTangent, length);
        if (status != EvaluationRefusal::None) return status;
        if (std::abs(length - target) <= tolerance * 0.5) {
            parameter = middle; measuredLength = length; return EvaluationRefusal::None;
        }
        if (length < target) low = middle; else high = middle;
        if (std::nextafter(low, high) >= high) break;
    }
    return EvaluationRefusal::InversionDidNotConverge;
}
} // namespace core3d::bounded_curve
