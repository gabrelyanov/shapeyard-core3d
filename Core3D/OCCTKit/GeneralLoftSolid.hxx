#pragma once

// Detached C3 ruled-v1 kernel construction. This file has no OCAF, command,
// catalog, registry, presentation, or editor dependency. A KernelBuild is not
// an admitted result: GeneralLoftProof.hxx must independently prove it.
#include "GeneralLoftDefinition.hxx"
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepCheck_Wire.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Precision.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Wire.hxx>
#include <atomic>

namespace core3d::general_loft {

enum class KernelBuildStatus : std::uint8_t {
    BuiltUnproven = 0, InvalidDefinition, Cancelled, CurveRefused,
    WireRefused, KernelFailure, TopologyRefused
};

struct KernelBuild {
    KernelBuildStatus status = KernelBuildStatus::InvalidDefinition;
    TopoDS_Solid solid;
    std::vector<TopoDS_Wire> stationWires;
    std::vector<std::vector<TopoDS_Edge>> stationEdges;
    std::vector<std::vector<TopoDS_Face>> generatedSides;
    TopoDS_Face firstCap;
    TopoDS_Face lastCap;
    double positionalTolerance = 0;
};

namespace solid_detail {
inline gp_Pnt WorldPoint(const bounded_curve::Frame& frame,
                         const std::array<double, 3>& local) {
    return gp_Pnt(
        frame.origin[0] + frame.xAxis[0] * local[0] + frame.yAxis[0] * local[1]
            + frame.zAxis[0] * local[2],
        frame.origin[1] + frame.xAxis[1] * local[0] + frame.yAxis[1] * local[1]
            + frame.zAxis[1] * local[2],
        frame.origin[2] + frame.xAxis[2] * local[0] + frame.yAxis[2] * local[1]
            + frame.zAxis[2] * local[2]);
}

inline Handle(Geom_BSplineCurve) ExactCurve(const bounded_curve::Definition& value) {
    const Standard_Integer poleCount = Standard_Integer(value.controlPoints.size());
    const Standard_Integer knotCount = Standard_Integer(value.knots.size());
    TColgp_Array1OfPnt poles(1, poleCount);
    TColStd_Array1OfReal knots(1, knotCount), weights(1, poleCount);
    TColStd_Array1OfInteger multiplicities(1, knotCount);
    for (Standard_Integer i = 1; i <= poleCount; ++i) {
        poles(i) = WorldPoint(value.frame, value.controlPoints[std::size_t(i - 1)].local);
        weights(i) = value.weights.empty() ? 1.0 : value.weights[std::size_t(i - 1)];
    }
    for (Standard_Integer i = 1; i <= knotCount; ++i) {
        knots(i) = value.knots[std::size_t(i - 1)].value;
        multiplicities(i) = value.knots[std::size_t(i - 1)].multiplicity;
    }
    if (value.weights.empty())
        return new Geom_BSplineCurve(poles, knots, multiplicities, value.degree, Standard_False);
    return new Geom_BSplineCurve(
        poles, weights, knots, multiplicities, value.degree, Standard_False);
}

inline double PinnedTolerance(const Definition& value) noexcept {
    // One nanometre expressed in document units, never below OCCT confusion.
    return std::max(Precision::Confusion(), 1.0e-9 / value.dimensionMetersPerUnit);
}
} // namespace solid_detail

inline KernelBuild BuildDetached(const Definition& definition,
                                 const std::atomic_bool& cancelled) noexcept {
    KernelBuild output;
    if (cancelled.load(std::memory_order_relaxed)) {
        output.status = KernelBuildStatus::Cancelled;
        return output;
    }
    if (Validate(definition) != Admission::Accepted) return output;
    try {
        output.positionalTolerance = solid_detail::PinnedTolerance(definition);
        output.stationWires.reserve(definition.stations.size());
        output.stationEdges.reserve(definition.stations.size());
        for (const Station& station : definition.stations) {
            if (cancelled.load(std::memory_order_relaxed)) {
                output = {}; output.status = KernelBuildStatus::Cancelled; return output;
            }
            BRepBuilderAPI_MakeWire wireMaker;
            std::vector<TopoDS_Edge> edges;
            edges.reserve(station.segments.size());
            for (const Segment& segment : station.segments) {
                const Handle(Geom_BSplineCurve) curve =
                    solid_detail::ExactCurve(segment.curveState.definition);
                if (curve.IsNull()) {
                    output = {}; output.status = KernelBuildStatus::CurveRefused; return output;
                }
                BRepBuilderAPI_MakeEdge edgeMaker(curve, curve->FirstParameter(), curve->LastParameter());
                if (!edgeMaker.IsDone() || edgeMaker.Edge().IsNull()) {
                    output = {}; output.status = KernelBuildStatus::CurveRefused; return output;
                }
                wireMaker.Add(edgeMaker.Edge());
                if (!wireMaker.IsDone() || wireMaker.Edge().IsNull()) {
                    output = {}; output.status = KernelBuildStatus::WireRefused; return output;
                }
                // MakeWire may copy the input edge when sharing coincident
                // vertices. Preserve authored order using the edge it installed.
                edges.push_back(wireMaker.Edge());
            }
            const TopoDS_Wire wire = wireMaker.Wire();
            if (wire.IsNull() || BRepCheck_Wire(wire).Closed() != BRepCheck_NoError
                || !BRepCheck_Analyzer(wire, Standard_True).IsValid()) {
                output = {}; output.status = KernelBuildStatus::WireRefused; return output;
            }
            output.stationEdges.push_back(std::move(edges));
            output.stationWires.push_back(wire);
        }

        BRepOffsetAPI_ThruSections maker(Standard_True, Standard_True,
                                         output.positionalTolerance);
        maker.CheckCompatibility(Standard_False); // authored origins/order are authority
        maker.SetMutableInput(Standard_False);     // station wires are immutable inputs
        for (const TopoDS_Wire& wire : output.stationWires) maker.AddWire(wire);
        maker.Build();
        if (cancelled.load(std::memory_order_relaxed)) {
            output = {}; output.status = KernelBuildStatus::Cancelled; return output;
        }
        if (!maker.IsDone() || maker.Shape().IsNull()
            || maker.Shape().ShapeType() != TopAbs_SOLID) {
            output = {}; output.status = KernelBuildStatus::KernelFailure; return output;
        }
        output.solid = TopoDS::Solid(maker.Shape());
        const TopoDS_Shape first = maker.FirstShape(), last = maker.LastShape();
        if (first.IsNull() || last.IsNull() || first.ShapeType() != TopAbs_FACE
            || last.ShapeType() != TopAbs_FACE) {
            output = {}; output.status = KernelBuildStatus::TopologyRefused; return output;
        }
        output.firstCap = TopoDS::Face(first);
        output.lastCap = TopoDS::Face(last);
        output.generatedSides.resize(definition.stations.size() - 1);
        for (std::size_t station = 0; station + 1 < definition.stations.size(); ++station) {
            auto& interval = output.generatedSides[station];
            interval.reserve(definition.stations[station].segments.size());
            for (const TopoDS_Edge& edge : output.stationEdges[station]) {
                const TopoDS_Shape generated = maker.GeneratedFace(edge);
                if (generated.IsNull() || generated.ShapeType() != TopAbs_FACE) {
                    output = {}; output.status = KernelBuildStatus::TopologyRefused; return output;
                }
                interval.push_back(TopoDS::Face(generated));
            }
        }
        output.status = KernelBuildStatus::BuiltUnproven;
        return output;
    } catch (...) {
        output = {}; output.status = KernelBuildStatus::KernelFailure; return output;
    }
}
} // namespace core3d::general_loft
