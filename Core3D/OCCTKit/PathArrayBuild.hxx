#pragma once

#include "BoundedCurveEvaluation.hxx"
#include "PathArrayDefinition.hxx"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace core3d::path_array {
using Matrix = std::array<double, 16>;
using Vector3 = bounded_curve::Vector3;

enum class BuildRefusal : std::uint8_t {
    None = 0, InvalidDefinition, StalePath, ClosedStateMismatch,
    ArcLengthFailure, TooFewInstances, TooManyInstances, ZeroTangent,
    Cusp, FrameFlip, InvalidUpVector, InversionFailure
};

struct Placement {
    UUID identity{};
    std::uint64_t localID = 0;
    std::uint32_t ordinal = 0;
    double requestedArcLength = 0;
    double measuredArcLength = 0;
    double parameter = 0;
    Matrix occurrenceFrame{};
};

struct BuildReceipt {
    double totalArcLength = 0;
    double maximumMeasuredArcError = 0;
    std::uint32_t requestedInstances = 0;
    std::uint32_t emittedInstances = 0;
    bool closedSeamCanonicalized = false;
};

inline Vector3 MatrixAxis(const Matrix& matrix, unsigned column) noexcept {
    return {{matrix[column], matrix[4 + column], matrix[8 + column]}};
}

inline Matrix Frame(const Vector3& origin, const Vector3& x,
                    const Vector3& y, const Vector3& z) noexcept {
    return {{x[0],y[0],z[0],origin[0], x[1],y[1],z[1],origin[1],
             x[2],y[2],z[2],origin[2], 0,0,0,1}};
}

inline Vector3 RotateAroundAxis(const Vector3& value, const Vector3& axis,
                                double angle) noexcept {
    const double cosine = std::cos(angle), sine = std::sin(angle);
    return bounded_curve::Add(bounded_curve::Scale(value, cosine),
        bounded_curve::Add(bounded_curve::Scale(bounded_curve::Cross(axis, value), sine),
            bounded_curve::Scale(axis, bounded_curve::Dot(axis, value) * (1 - cosine))));
}

inline bool ProjectNormal(const Vector3& candidate, const Vector3& tangent,
                          double minimum, Vector3& normal) noexcept {
    return bounded_curve::Normalize(bounded_curve::Subtract(candidate,
        bounded_curve::Scale(tangent, bounded_curve::Dot(candidate, tangent))),
        minimum, normal);
}

inline bool DetectClosed(const bounded_curve::Definition& path, double tolerance,
                         double minimumTangent, bool& closed) noexcept {
    closed = false; double first = 0, last = 0;
    if (!bounded_curve::ParameterDomain(path, first, last)) return false;
    bounded_curve::Evaluation a, b;
    if (bounded_curve::Evaluate(path, first, a) != bounded_curve::EvaluationRefusal::None
        || bounded_curve::Evaluate(path, last, b) != bounded_curve::EvaluationRefusal::None) return false;
    const double gap = bounded_curve::Norm(bounded_curve::Subtract(a.point, b.point));
    if (!std::isfinite(gap)) return false;
    if (gap > tolerance) return true;
    Vector3 ta{}, tb{};
    if (!bounded_curve::Normalize(a.derivative, minimumTangent, ta)
        || !bounded_curve::Normalize(b.derivative, minimumTangent, tb)) return false;
    closed = bounded_curve::Dot(ta, tb) > -0.999999;
    return true;
}

inline bool SampleDistances(const DistributionLaw& law, double total, bool closed,
                            double tolerance, std::vector<double>& output) noexcept {
    output.clear();
    try {
        if (!ValidLaw(law) || !std::isfinite(total) || total <= tolerance) return false;
        if (law.mode == DistributionMode::Count) {
            output.reserve(law.count);
            if (closed) {
                if (law.includeStart) {
                    for (std::uint32_t index = 0; index < law.count; ++index)
                        output.push_back(total * double(index) / double(law.count));
                } else if (law.includeEnd) {
                    for (std::uint32_t index = 1; index <= law.count; ++index)
                        output.push_back(total * double(index) / double(law.count));
                } else {
                    for (std::uint32_t index = 0; index < law.count; ++index)
                        output.push_back(total * (double(index) + 0.5) / double(law.count));
                }
            } else {
                const unsigned endpoints = unsigned(law.includeStart) + unsigned(law.includeEnd);
                const double divisor = double(law.count + 1 - endpoints);
                const double first = law.includeStart ? 0 : total / divisor;
                for (std::uint32_t index = 0; index < law.count; ++index)
                    output.push_back(first + total * double(index) / divisor);
            }
        } else if (closed) {
            if (law.includeStart) output.push_back(0);
            const double first = law.includeStart || law.includeEnd
                ? law.distance : law.distance * 0.5;
            for (double distance = first; distance < total - tolerance;
                 distance += law.distance) {
                if (output.size() >= MaximumInstances) return false;
                output.push_back(distance);
            }
            if (!law.includeStart && law.includeEnd) output.push_back(total);
        } else {
            if (law.includeStart) output.push_back(0);
            for (double distance = law.distance; distance < total - tolerance;
                 distance += law.distance) {
                if (output.size() >= MaximumInstances) return false;
                output.push_back(distance);
            }
            if (law.includeEnd
                && (output.empty() || std::abs(output.back() - total) > tolerance))
                output.push_back(total);
        }
        if (output.size() < 2 || output.size() > MaximumInstances) return false;
        for (std::size_t index = 1; index < output.size(); ++index)
            if (!(output[index - 1] < output[index])) return false;
        return output.front() >= 0 && output.back() <= total;
    } catch (...) { output.clear(); return false; }
}

inline BuildRefusal Translate(bounded_curve::EvaluationRefusal refusal) noexcept {
    switch (refusal) {
        case bounded_curve::EvaluationRefusal::ZeroTangent: return BuildRefusal::ZeroTangent;
        case bounded_curve::EvaluationRefusal::Cusp: return BuildRefusal::Cusp;
        case bounded_curve::EvaluationRefusal::InversionDidNotConverge:
            return BuildRefusal::InversionFailure;
        default: return BuildRefusal::ArcLengthFailure;
    }
}

inline BuildRefusal RequiredInstanceCount(const Definition& definition,
        const bounded_curve::PersistedValue& path, std::uint32_t& count,
        double& totalLength) noexcept {
    count = 0; totalLength = 0;
    if (!ValidLaw(definition.distribution) || !ValidOrientation(definition.orientation)
        || !ValidCurveReference(definition.path)) return BuildRefusal::InvalidDefinition;
    if (!Matches(definition.path, path)) return BuildRefusal::StalePath;
    const auto audit = bounded_curve::AuditTangents(path.value.definition,
        definition.minimumTangent, definition.orientation.maximumFrameStepRadians);
    if (audit != bounded_curve::EvaluationRefusal::None) return Translate(audit);
    bool closed = false;
    if (!DetectClosed(path.value.definition, definition.arcLengthTolerance,
            definition.minimumTangent, closed)) return BuildRefusal::ArcLengthFailure;
    if (closed != definition.closedPath) return BuildRefusal::ClosedStateMismatch;
    double first = 0, last = 0;
    if (!bounded_curve::ParameterDomain(path.value.definition, first, last))
        return BuildRefusal::ArcLengthFailure;
    if (closed) {
        bounded_curve::Evaluation start, end; Vector3 startTangent{}, endTangent{};
        if (bounded_curve::Evaluate(path.value.definition, first, start)
                != bounded_curve::EvaluationRefusal::None
            || bounded_curve::Evaluate(path.value.definition, last, end)
                != bounded_curve::EvaluationRefusal::None
            || !bounded_curve::Normalize(start.derivative, definition.minimumTangent, startTangent)
            || !bounded_curve::Normalize(end.derivative, definition.minimumTangent, endTangent))
            return BuildRefusal::ZeroTangent;
        const double seamAngle = std::acos(std::clamp(
            bounded_curve::Dot(startTangent, endTangent), -1.0, 1.0));
        if (seamAngle > definition.orientation.maximumFrameStepRadians)
            return BuildRefusal::FrameFlip;
    }
    const auto lengthStatus = bounded_curve::ArcLength(path.value.definition, first, last,
        definition.arcLengthTolerance * 0.25, definition.minimumTangent, totalLength);
    if (lengthStatus != bounded_curve::EvaluationRefusal::None) return Translate(lengthStatus);
    std::vector<double> distances;
    if (!SampleDistances(definition.distribution, totalLength, closed,
            definition.arcLengthTolerance, distances))
        return definition.distribution.mode == DistributionMode::Distance
            ? BuildRefusal::TooFewInstances : BuildRefusal::InvalidDefinition;
    count = std::uint32_t(distances.size());
    return count <= MaximumInstances ? BuildRefusal::None : BuildRefusal::TooManyInstances;
}

inline BuildRefusal BuildPlacements(const Definition& definition,
        const bounded_curve::PersistedValue& path, std::vector<Placement>& output,
        BuildReceipt& receipt) noexcept {
    output.clear(); receipt = {};
    try {
        if (!Valid(definition)) return BuildRefusal::InvalidDefinition;
        std::uint32_t required = 0; double total = 0;
        const auto requiredStatus = RequiredInstanceCount(definition, path, required, total);
        if (requiredStatus != BuildRefusal::None) return requiredStatus;
        if (required != definition.members.size()) return BuildRefusal::InvalidDefinition;
        std::vector<double> distances;
        if (!SampleDistances(definition.distribution, total, definition.closedPath,
                definition.arcLengthTolerance, distances)) return BuildRefusal::InvalidDefinition;
        double domainFirst = 0, domainLast = 0;
        if (!bounded_curve::ParameterDomain(path.value.definition, domainFirst, domainLast))
            return BuildRefusal::ArcLengthFailure;
        std::vector<Placement> placements; placements.reserve(required);
        Vector3 previousTangent{}, previousNormal{}; bool haveFrame = false;
        const Vector3 sourceY = MatrixAxis(definition.sourceFrame, 1);
        const Vector3 sourceZ = MatrixAxis(definition.sourceFrame, 2);
        for (std::uint32_t ordinal = 0; ordinal < required; ++ordinal) {
            const double requested = distances[ordinal];
            double parameter = 0, measured = 0;
            const auto inversion = bounded_curve::InvertArcLength(path.value.definition,
                requested, total, definition.arcLengthTolerance,
                definition.minimumTangent, parameter, measured);
            if (inversion != bounded_curve::EvaluationRefusal::None) return Translate(inversion);
            bounded_curve::Evaluation evaluated;
            if (bounded_curve::Evaluate(path.value.definition, parameter, evaluated)
                != bounded_curve::EvaluationRefusal::None) return BuildRefusal::ArcLengthFailure;
            Vector3 tangent{};
            if (!bounded_curve::Normalize(evaluated.derivative,
                    definition.minimumTangent, tangent)) return BuildRefusal::ZeroTangent;
            Vector3 x{}, y{}, z{};
            if (definition.orientation.policy == OrientationPolicy::Fixed) {
                x = MatrixAxis(definition.sourceFrame, 0); y = sourceY; z = sourceZ;
            } else {
                x = tangent;
                if (definition.orientation.policy == OrientationPolicy::Bishop && haveFrame) {
                    const Vector3 axis = bounded_curve::Cross(previousTangent, tangent);
                    const double sine = bounded_curve::Norm(axis);
                    const double cosine = std::clamp(
                        bounded_curve::Dot(previousTangent, tangent), -1.0, 1.0);
                    Vector3 transported = previousNormal;
                    if (sine > definition.minimumTangent) {
                        Vector3 unitAxis{};
                        if (!bounded_curve::Normalize(axis, definition.minimumTangent, unitAxis))
                            return BuildRefusal::FrameFlip;
                        transported = RotateAroundAxis(previousNormal, unitAxis,
                                                       std::atan2(sine, cosine));
                    }
                    if (!ProjectNormal(transported, tangent,
                            definition.minimumTangent, y)) return BuildRefusal::FrameFlip;
                } else {
                    const Vector3 requestedUp = definition.orientation.hasUpVector
                        ? definition.orientation.upVector : sourceY;
                    if (!ProjectNormal(requestedUp, tangent,
                            definition.minimumTangent, y)
                        && !ProjectNormal(sourceZ, tangent,
                            definition.minimumTangent, y)) return BuildRefusal::InvalidUpVector;
                }
                // Seed the Bishop frame with roll once. Parallel transport of
                // that rolled normal preserves the same roll without drift.
                if (definition.orientation.policy == OrientationPolicy::Bishop
                    && !haveFrame && definition.orientation.rollRadians != 0) {
                    y = RotateAroundAxis(y, tangent, definition.orientation.rollRadians);
                    if (!bounded_curve::Normalize(y, definition.minimumTangent, y))
                        return BuildRefusal::InvalidUpVector;
                }
                z = bounded_curve::Cross(x, y);
                if (!bounded_curve::Normalize(z, definition.minimumTangent, z))
                    return BuildRefusal::InvalidUpVector;
                y = bounded_curve::Cross(z, x);
                if (!bounded_curve::Normalize(y, definition.minimumTangent, y))
                    return BuildRefusal::InvalidUpVector;
            }
            if (haveFrame && definition.orientation.policy != OrientationPolicy::Fixed) {
                const double xAngle = std::acos(std::clamp(
                    bounded_curve::Dot(previousTangent, x), -1.0, 1.0));
                const double yAngle = std::acos(std::clamp(
                    bounded_curve::Dot(previousNormal, y), -1.0, 1.0));
                if (std::max(xAngle, yAngle)
                    > definition.orientation.maximumFrameStepRadians) return BuildRefusal::FrameFlip;
            }
            previousTangent = x; previousNormal = y; haveFrame = true;
            receipt.maximumMeasuredArcError = std::max(receipt.maximumMeasuredArcError,
                std::abs(measured - requested));
            const Member& member = definition.members[ordinal];
            if (member.state != MemberState::Suppressed)
                placements.push_back({member.identity, member.localID, ordinal,
                    requested, measured, parameter, Frame(evaluated.point, x, y, z)});
        }
        receipt.totalArcLength = total; receipt.requestedInstances = required;
        receipt.emittedInstances = std::uint32_t(placements.size());
        receipt.closedSeamCanonicalized = definition.closedPath
            && definition.distribution.includeStart && definition.distribution.includeEnd;
        if (receipt.maximumMeasuredArcError > definition.arcLengthTolerance)
            return BuildRefusal::InversionFailure;
        output = std::move(placements); return BuildRefusal::None;
    } catch (...) { output.clear(); receipt = {}; return BuildRefusal::ArcLengthFailure; }
}
} // namespace core3d::path_array
