#pragma once

// Pure detached C1 construction. This header consumes only the canonical
// retained value: UI units, document labels and occurrence transforms are not
// inputs and therefore cannot be applied a second time here.
#include "BoundedCurveEvaluation.hxx"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRep_Tool.hxx>
#include <Geom_BSplineCurve.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace core3d::bounded_curve {

enum class BuildRefusal : std::uint8_t {
    None = 0,
    InvalidPersistedValue,
    InvalidDefinition,
    KernelFailure,
    UnsupportedTopology,
    InvalidParameterDomain,
    EvaluatorMismatch,
    DegenerateGeometry,
    CommitmentFailure
};

// Geometry evidence is bounded to five canonical samples. The wire is always
// detached and forward. Canonical bytes and digests let the later document
// owner prove that a caller did not substitute topology from an older edit.
struct DetachedWire final {
    std::vector<std::uint8_t> canonicalDefinitionBytes;
    std::vector<std::uint8_t> canonicalOwnerBytes;
    TopoDS_Wire wire;
    Digest definitionDigest{};
    Digest geometryCommitment{};
    double firstParameter = 0;
    double lastParameter = 0;
    std::array<Vector3, 5> canonicalSamples{};
};

namespace build_detail {

inline bool Near(double a, double b, double tolerance) noexcept {
    return Finite(a) && Finite(b) && Finite(tolerance)
        && tolerance > 0 && std::abs(a - b) <= tolerance;
}

inline bool SamePoint(const gp_Pnt& actual, const Vector3& expected,
                      double tolerance) noexcept {
    return Near(actual.X(), expected[0], tolerance)
        && Near(actual.Y(), expected[1], tolerance)
        && Near(actual.Z(), expected[2], tolerance);
}

inline Definition EvaluationDefinition(const Definition& source) noexcept {
    Definition result = source;
    // The landed evaluator is intentionally Path3D-only. A Sketch2D wire uses
    // the identical retained poles/frame with its already-validated local Z=0;
    // changing only this discriminator enables the same canonical evaluator.
    result.domain = Domain::Path3D;
    return result;
}

inline bool CanonicalSamples(const Definition& definition, double first,
                             double last,
                             std::array<Vector3, 5>& samples) noexcept {
    const Definition evaluated = EvaluationDefinition(definition);
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const double fraction = double(index) / double(samples.size() - 1);
        const double parameter = index + 1 == samples.size()
            ? last : first + (last - first) * fraction;
        Evaluation value;
        if (Evaluate(evaluated, parameter, value) != EvaluationRefusal::None)
            return false;
        samples[index] = value.point;
    }
    return true;
}

inline bool GeometryDigest(double first, double last,
                           const std::array<Vector3, 5>& samples,
                           Digest& output) noexcept {
    try {
        Writer writer(2 + 2 * sizeof(double) + samples.size() * 3 * sizeof(double));
        writer.raw(reinterpret_cast<const std::uint8_t*>("C1"), 2);
        writer.scalar(first); writer.scalar(last);
        for (const Vector3& point : samples)
            for (double coordinate : point) writer.scalar(coordinate);
        return writer.valid && Hash(writer.bytes, MaximumDefinitionBytes, output);
    } catch (...) { output.fill(0); return false; }
}

inline BuildRefusal AuditWire(const Definition& definition,
                              const TopoDS_Wire& wire,
                              double expectedFirst, double expectedLast,
                              std::array<Vector3, 5>* observedSamples = nullptr) noexcept {
    try {
        if (wire.IsNull() || wire.ShapeType() != TopAbs_WIRE
            || wire.Orientation() != TopAbs_FORWARD
            || !BRepCheck_Analyzer(wire, Standard_True).IsValid())
            return BuildRefusal::UnsupportedTopology;
        TopExp_Explorer edges(wire, TopAbs_EDGE);
        if (!edges.More()) return BuildRefusal::UnsupportedTopology;
        const TopoDS_Edge edge = TopoDS::Edge(edges.Current());
        edges.Next();
        if (edges.More() || edge.IsNull() || edge.Orientation() != TopAbs_FORWARD)
            return BuildRefusal::UnsupportedTopology;
        TopLoc_Location location;
        Standard_Real first = 0, last = 0;
        const Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, location, first, last);
        if (curve.IsNull() || !(first < last)) return BuildRefusal::KernelFailure;
        const double parameterTolerance = 64 * std::numeric_limits<double>::epsilon()
            * std::max({1.0, std::abs(expectedFirst), std::abs(expectedLast)});
        if (!Near(first, expectedFirst, parameterTolerance)
            || !Near(last, expectedLast, parameterTolerance))
            return BuildRefusal::InvalidParameterDomain;

        std::array<Vector3, 5> expected{};
        if (!CanonicalSamples(definition, expectedFirst, expectedLast, expected))
            return BuildRefusal::EvaluatorMismatch;
        double extent = 1;
        for (const Vector3& point : expected)
            for (double coordinate : point) extent = std::max(extent, std::abs(coordinate));
        const double tolerance = 1e-9 * extent;
        double chordSum = 0;
        gp_Pnt previous;
        for (std::size_t index = 0; index < expected.size(); ++index) {
            const double fraction = double(index) / double(expected.size() - 1);
            const double parameter = index + 1 == expected.size()
                ? expectedLast : expectedFirst + (expectedLast - expectedFirst) * fraction;
            gp_Pnt point = curve->Value(parameter);
            point.Transform(location.Transformation());
            if (!SamePoint(point, expected[index], tolerance))
                return BuildRefusal::EvaluatorMismatch;
            if (index != 0) chordSum += point.Distance(previous);
            previous = point;
        }
        if (!Finite(chordSum) || chordSum <= tolerance)
            return BuildRefusal::DegenerateGeometry;
        if (observedSamples) *observedSamples = expected;
        return BuildRefusal::None;
    } catch (...) { return BuildRefusal::KernelFailure; }
}

} // namespace build_detail

inline BuildRefusal BuildWire(const PersistedValue& persisted,
                              DetachedWire& output) noexcept {
    output = {};
    try {
        std::vector<std::uint8_t> definitionBytes, ownerBytes;
        if (!ValidatePersisted(persisted, &definitionBytes, &ownerBytes))
            return BuildRefusal::InvalidPersistedValue;
        const Definition& definition = persisted.value.definition;
        if (Validate(definition) != Refusal::None)
            return BuildRefusal::InvalidDefinition;
        double first = 0, last = 0;
        if (!ParameterDomain(definition, first, last))
            return BuildRefusal::InvalidParameterDomain;

        const Standard_Integer poleCount = Standard_Integer(definition.controlPoints.size());
        const Standard_Integer knotCount = Standard_Integer(definition.knots.size());
        TColgp_Array1OfPnt poles(1, poleCount);
        TColStd_Array1OfReal knots(1, knotCount);
        TColStd_Array1OfInteger multiplicities(1, knotCount);
        for (Standard_Integer index = 1; index <= poleCount; ++index) {
            const Vector3 world = ToWorldPoint(
                definition.frame, definition.controlPoints[std::size_t(index - 1)].local);
            poles.SetValue(index, gp_Pnt(world[0], world[1], world[2]));
        }
        for (Standard_Integer index = 1; index <= knotCount; ++index) {
            const Knot& knot = definition.knots[std::size_t(index - 1)];
            knots.SetValue(index, knot.value);
            multiplicities.SetValue(index, Standard_Integer(knot.multiplicity));
        }

        Handle(Geom_BSplineCurve) curve;
        if (definition.weights.empty()) {
            curve = new Geom_BSplineCurve(poles, knots, multiplicities,
                Standard_Integer(definition.degree), Standard_False);
        } else {
            TColStd_Array1OfReal weights(1, poleCount);
            for (Standard_Integer index = 1; index <= poleCount; ++index)
                weights.SetValue(index, definition.weights[std::size_t(index - 1)]);
            curve = new Geom_BSplineCurve(poles, weights, knots, multiplicities,
                Standard_Integer(definition.degree), Standard_False, Standard_True);
        }
        if (curve.IsNull() || curve->IsPeriodic()) return BuildRefusal::KernelFailure;
        BRepBuilderAPI_MakeEdge edgeBuilder(curve, first, last);
        if (!edgeBuilder.IsDone()) return BuildRefusal::KernelFailure;
        TopoDS_Edge edge = edgeBuilder.Edge();
        edge.Orientation(TopAbs_FORWARD);
        BRepBuilderAPI_MakeWire wireBuilder(edge);
        if (!wireBuilder.IsDone()) return BuildRefusal::KernelFailure;
        TopoDS_Wire wire = wireBuilder.Wire();
        wire.Orientation(TopAbs_FORWARD);

        std::array<Vector3, 5> samples{};
        const BuildRefusal audit = build_detail::AuditWire(
            definition, wire, first, last, &samples);
        if (audit != BuildRefusal::None) return audit;
        Digest geometry{};
        if (!build_detail::GeometryDigest(first, last, samples, geometry))
            return BuildRefusal::CommitmentFailure;

        output.canonicalDefinitionBytes = std::move(definitionBytes);
        output.canonicalOwnerBytes = std::move(ownerBytes);
        output.wire = std::move(wire);
        output.definitionDigest = persisted.ownerState.canonicalDefinitionDigest;
        output.geometryCommitment = geometry;
        output.firstParameter = first;
        output.lastParameter = last;
        output.canonicalSamples = samples;
        return BuildRefusal::None;
    } catch (...) { output = {}; return BuildRefusal::KernelFailure; }
}

inline bool WireMatchesDefinition(const PersistedValue& persisted,
                                  const TopoDS_Wire& wire) noexcept {
    if (!ValidatePersisted(persisted)) return false;
    double first = 0, last = 0;
    return ParameterDomain(persisted.value.definition, first, last)
        && build_detail::AuditWire(persisted.value.definition, wire, first, last)
            == BuildRefusal::None;
}

inline bool MatchesPersistedValue(const DetachedWire& detached,
                                  const PersistedValue& persisted) noexcept {
    std::vector<std::uint8_t> definitionBytes, ownerBytes;
    Digest geometry{};
    return ValidatePersisted(persisted, &definitionBytes, &ownerBytes)
        && detached.definitionDigest == persisted.ownerState.canonicalDefinitionDigest
        && detached.canonicalDefinitionBytes == definitionBytes
        && detached.canonicalOwnerBytes == ownerBytes
        && build_detail::GeometryDigest(detached.firstParameter,
            detached.lastParameter, detached.canonicalSamples, geometry)
        && geometry == detached.geometryCommitment
        && WireMatchesDefinition(persisted, detached.wire);
}

} // namespace core3d::bounded_curve
