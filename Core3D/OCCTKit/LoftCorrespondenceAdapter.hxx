#pragma once

// Detached SYLC/1 geometry values and exact bilinear construction. This file
// deliberately has no OCAF, history, persistence, editor, or registration
// dependency. Admission and result verification live in the independent
// LoftCorrespondenceProof.hxx component.
#include "RectangularLoftDefinition.hxx"
#include "RetainedTopologyBudget.hxx"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom2d_BezierCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Precision.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pln.hxx>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <vector>

namespace core3d::loft_correspondence {
using ElementID = rectangular_loft::ElementID;
namespace budget = retained_topology_budget;

struct StationMapping {
    ElementID station = 0;
    std::array<ElementID, 4> targetsByLane{};
};

struct Definition {
    rectangular_loft::Definition base;
    std::vector<StationMapping> mappings;
    // An implicit equal-index view is buildable without enrolling a SYLC
    // record. An explicit all-identity request is a value-level no-op.
    bool implicitEqualIndex = false;
};

enum class Admission : std::uint8_t {
    Accepted = 0,
    Unchanged,
    InvalidBase,
    InvalidCount,
    InvalidStation,
    InvalidTarget,
    NonCyclic,
    HalfTurn,
    ContinuousClearance,
    Cancelled,
    BudgetDenied,
    NumericFailure
};

enum class BuildStatus : std::uint8_t {
    BuiltUnproven = 0,
    InvalidDefinition,
    Cancelled,
    BudgetDenied,
    KernelFailure,
    InvalidTopology
};

struct MappingInspection {
    std::vector<std::uint8_t> phases;
    bool changed = false;
};

struct BuildResult {
    BuildStatus status = BuildStatus::InvalidDefinition;
    TopoDS_Solid solid;
    std::uint32_t diagnosticStage = 0;
};

namespace detail {
inline bool SamePoint(const gp_Pnt& a, const gp_Pnt& b, double tolerance) noexcept {
    return a.SquareDistance(b) <= tolerance * tolerance;
}

inline bool TransformPoint(const Definition& definition, gp_Pnt& point) noexcept {
    try {
        if (!definition.base.constructionFrame) return true;
        gp_Trsf transform;
        if (!definition.base.constructionFrame->Transform(transform)) return false;
        point.Transform(transform);
        return std::isfinite(point.X()) && std::isfinite(point.Y())
            && std::isfinite(point.Z());
    } catch (...) { return false; }
}

inline std::array<gp_Pnt, 4> AuthoredCorners(const Definition& definition,
                                             std::size_t station,
                                             bool placed = false) {
    auto result = rectangular_loft::detail::Corners(definition.base.stations.at(station));
    if (placed) for (gp_Pnt& point : result) {
        if (!TransformPoint(definition, point)) throw Standard_Failure("invalid frame");
    }
    return result;
}

inline int CornerIndex(const rectangular_loft::Station& station,
                       ElementID target) noexcept {
    for (int index = 0; index < 4; ++index)
        if (station.cornerIdentifiers[std::size_t(index)] == target) return index;
    return -1;
}

inline std::array<gp_Pnt, 4> LaneCorners(const Definition& definition,
                                         std::size_t station,
                                         bool placed = false) {
    const auto geometric = AuthoredCorners(definition, station, placed);
    std::array<gp_Pnt, 4> result;
    for (std::size_t lane = 0; lane < 4; ++lane) {
        const int index = CornerIndex(definition.base.stations.at(station),
                                      definition.mappings.at(station).targetsByLane[lane]);
        if (index < 0) throw Standard_Failure("unknown target corner");
        result[lane] = geometric[std::size_t(index)];
    }
    return result;
}

inline Admission ValidateMapping(const Definition& definition,
                                 MappingInspection& output) noexcept {
    output = {};
    try {
        rectangular_loft::Inspection ignored;
        if (rectangular_loft::Inspect(definition.base, ignored)
            != rectangular_loft::Admission::Accepted) return Admission::InvalidBase;
        const std::size_t count = definition.base.stations.size();
        if (definition.mappings.size() != count) return Admission::InvalidCount;
        output.phases.reserve(count);
        bool changed = false;
        for (std::size_t stationIndex = 0; stationIndex < count; ++stationIndex) {
            const auto& station = definition.base.stations[stationIndex];
            const auto& mapping = definition.mappings[stationIndex];
            if (!mapping.station || mapping.station != station.identifier)
                return Admission::InvalidStation;
            std::set<ElementID> targets;
            for (ElementID target : mapping.targetsByLane)
                if (!target || CornerIndex(station, target) < 0 || !targets.insert(target).second)
                    return Admission::InvalidTarget;
            if (targets.size() != 4) return Admission::InvalidTarget;
            int phase = CornerIndex(station, mapping.targetsByLane[0]);
            if (phase < 0) return Admission::InvalidTarget;
            for (int lane = 0; lane < 4; ++lane) {
                const ElementID expected = station.cornerIdentifiers[std::size_t((lane + phase) % 4)];
                if (mapping.targetsByLane[std::size_t(lane)] != expected)
                    return Admission::NonCyclic;
            }
            if (stationIndex == 0 && phase != 0) return Admission::NonCyclic;
            if (phase != 0) changed = true;
            output.phases.push_back(std::uint8_t(phase));
            if (stationIndex) {
                const int delta = (phase - int(output.phases[stationIndex - 1]) + 4) % 4;
                if (delta == 2) return Admission::HalfTurn;
            }
        }
        output.changed = changed;
        if (!definition.implicitEqualIndex && !changed) return Admission::Unchanged;
        return Admission::Accepted;
    } catch (...) { output = {}; return Admission::NumericFailure; }
}

inline Handle(Geom_BSplineSurface) BilinearSurface(const gp_Pnt& q0,
                                                   const gp_Pnt& q1,
                                                   const gp_Pnt& r0,
                                                   const gp_Pnt& r1) {
    TColgp_Array2OfPnt poles(1, 2, 1, 2);
    poles(1, 1) = q0; poles(2, 1) = q1;
    poles(1, 2) = r0; poles(2, 2) = r1;
    TColStd_Array1OfReal knots(1, 2); knots(1) = 0; knots(2) = 1;
    TColStd_Array1OfInteger multiplicities(1, 2);
    multiplicities(1) = 2; multiplicities(2) = 2;
    return new Geom_BSplineSurface(poles, knots, knots, multiplicities,
                                   multiplicities, 1, 1);
}

inline Handle(Geom_BezierCurve) SegmentCurve(const gp_Pnt& first,
                                             const gp_Pnt& last) {
    TColgp_Array1OfPnt poles(1, 2); poles(1) = first; poles(2) = last;
    return new Geom_BezierCurve(poles);
}

inline Handle(Geom2d_BezierCurve) SegmentPCurve(double u0, double v0,
                                                double u1, double v1) {
    TColgp_Array1OfPnt2d poles(1, 2);
    poles(1) = gp_Pnt2d(u0, v0); poles(2) = gp_Pnt2d(u1, v1);
    return new Geom2d_BezierCurve(poles);
}

inline TopoDS_Edge Reversed(const TopoDS_Edge& edge) {
    return TopoDS::Edge(edge.Reversed());
}
} // namespace detail

// Construction consumes the same caller-owned sticky counter used by proof.
// Each station/span/cap begins a stage before allocating its native topology;
// every authored native vertex/edge/face is also charged as a visit.
inline BuildResult BuildUnproven(const Definition& definition,
                                 const std::atomic_bool& cancelled,
                                 budget::Counter& counter) noexcept {
    BuildResult output;
    if (cancelled.load(std::memory_order_relaxed)) {
        output.status = BuildStatus::Cancelled; return output;
    }
    MappingInspection mapping;
    const Admission admission = detail::ValidateMapping(definition, mapping);
    if (admission != Admission::Accepted) return output;
    try {
        OCC_CATCH_SIGNALS
        const std::size_t count = definition.base.stations.size();
        std::vector<std::array<gp_Pnt, 4>> geometric(count), lanes(count);
        std::vector<std::array<TopoDS_Vertex, 4>> vertices(count);
        std::vector<std::array<TopoDS_Edge, 4>> rings(count), rails(count - 1);
        for (std::size_t station = 0; station < count; ++station) {
            output.diagnosticStage = 10;
            if (cancelled.load(std::memory_order_relaxed)) {
                output.status = BuildStatus::Cancelled; return output;
            }
            if (!counter.beginStage(budget::Site::C16KernelBuild)
                || !counter.visit(8, budget::Site::C16KernelBuild)) {
                output.status = BuildStatus::BudgetDenied; return output;
            }
            geometric[station] = detail::AuthoredCorners(definition, station, false);
            lanes[station] = detail::LaneCorners(definition, station, false);
            for (std::size_t corner = 0; corner < 4; ++corner) {
                output.diagnosticStage = 11;
                BRepBuilderAPI_MakeVertex maker(geometric[station][corner]);
                if (!maker.IsDone()) { output.status = BuildStatus::KernelFailure; return output; }
                vertices[station][corner] = maker.Vertex();
            }
            for (std::size_t corner = 0; corner < 4; ++corner) {
                output.diagnosticStage = 12;
                BRepBuilderAPI_MakeEdge maker(detail::SegmentCurve(geometric[station][corner],
                    geometric[station][(corner + 1) % 4]), vertices[station][corner],
                    vertices[station][(corner + 1) % 4]);
                if (!maker.IsDone()) { output.status = BuildStatus::KernelFailure; return output; }
                rings[station][corner] = maker.Edge();
            }
            if (station) {
                output.diagnosticStage = 13;
                if (!counter.visit(4, budget::Site::C16KernelBuild)) {
                    output.status = BuildStatus::BudgetDenied; return output;
                }
                for (std::size_t lane = 0; lane < 4; ++lane) {
                    const int lower = detail::CornerIndex(definition.base.stations[station - 1],
                        definition.mappings[station - 1].targetsByLane[lane]);
                    const int upper = detail::CornerIndex(definition.base.stations[station],
                        definition.mappings[station].targetsByLane[lane]);
                    BRepBuilderAPI_MakeEdge maker(detail::SegmentCurve(lanes[station - 1][lane],
                        lanes[station][lane]), vertices[station - 1][std::size_t(lower)],
                        vertices[station][std::size_t(upper)]);
                    if (!maker.IsDone()) { output.status = BuildStatus::KernelFailure; return output; }
                    rails[station - 1][lane] = maker.Edge();
                }
            }
        }

        BRep_Builder builder;
        TopoDS_Shell shell;
        builder.MakeShell(shell);
        for (std::size_t station = 0; station + 1 < count; ++station) {
            output.diagnosticStage = 20;
            if (!counter.beginStage(budget::Site::C16KernelBuild)
                || !counter.visit(4, budget::Site::C16KernelBuild)) {
                output.status = BuildStatus::BudgetDenied; return output;
            }
            for (std::size_t lane = 0; lane < 4; ++lane) {
                if (cancelled.load(std::memory_order_relaxed)) {
                    output.status = BuildStatus::Cancelled; return output;
                }
                const std::size_t next = (lane + 1) % 4;
                const std::size_t lowerCorner = std::size_t(detail::CornerIndex(
                    definition.base.stations[station],
                    definition.mappings[station].targetsByLane[lane]));
                const std::size_t upperCorner = std::size_t(detail::CornerIndex(
                    definition.base.stations[station + 1],
                    definition.mappings[station + 1].targetsByLane[lane]));
                const Handle(Geom_BSplineSurface) surface = detail::BilinearSurface(
                    lanes[station][lane], lanes[station][next],
                    lanes[station + 1][lane], lanes[station + 1][next]);
                BRepBuilderAPI_MakeWire wire;
                output.diagnosticStage = 21;
                wire.Add(rings[station][lowerCorner]);
                wire.Add(rails[station][next]);
                wire.Add(detail::Reversed(rings[station + 1][upperCorner]));
                wire.Add(detail::Reversed(rails[station][lane]));
                if (!wire.IsDone() || !wire.Wire().Closed()) {
                    output.status = BuildStatus::KernelFailure; return output;
                }
                output.diagnosticStage = 22;
                TopoDS_Face face;
                builder.MakeFace(face, surface, Precision::Confusion());
                const std::array<TopoDS_Edge, 4> faceEdges = {
                    rings[station][lowerCorner], rails[station][next],
                    rings[station + 1][upperCorner], rails[station][lane]};
                const std::array<Handle(Geom2d_Curve), 4> pcurves = {
                    detail::SegmentPCurve(0, 0, 1, 0), detail::SegmentPCurve(1, 0, 1, 1),
                    detail::SegmentPCurve(0, 1, 1, 1), detail::SegmentPCurve(0, 0, 0, 1)};
                for (std::size_t edgeIndex = 0; edgeIndex < 4; ++edgeIndex) {
                    builder.UpdateEdge(faceEdges[edgeIndex], pcurves[edgeIndex], face,
                                       Precision::Confusion());
                    builder.Range(faceEdges[edgeIndex], face, 0, 1);
                }
                builder.Add(face, wire.Wire());
                builder.Add(shell, face);
            }
        }

        for (std::size_t station : {std::size_t(0), count - 1}) {
            output.diagnosticStage = 30;
            if (!counter.beginStage(budget::Site::C16KernelBuild)
                || !counter.visit(1, budget::Site::C16KernelBuild)) {
                output.status = BuildStatus::BudgetDenied; return output;
            }
            BRepBuilderAPI_MakeWire wire;
            for (const TopoDS_Edge& edge : rings[station]) wire.Add(edge);
            if (!wire.IsDone() || !wire.Wire().Closed()) {
                output.status = BuildStatus::KernelFailure; return output;
            }
            BRepBuilderAPI_MakeFace maker(gp_Pln(geometric[station][0], gp::DZ()),
                                          wire.Wire(), Standard_False);
            output.diagnosticStage = 31;
            if (!maker.IsDone()) { output.status = BuildStatus::KernelFailure; return output; }
            TopoDS_Face face = maker.Face();
            if (station == 0) face.Reverse();
            builder.Add(shell, face);
        }
        output.diagnosticStage = 32;
        if (!BRep_Tool::IsClosed(shell)) {
            output.status = BuildStatus::InvalidTopology; return output;
        }
        shell.Closed(Standard_True);
        TopoDS_Solid solid;
        builder.MakeSolid(solid);
        builder.Add(solid, shell);
        TopoDS_Shape candidate = solid;
        if (definition.base.constructionFrame) {
            output.diagnosticStage = 40;
            gp_Trsf transform;
            if (!definition.base.constructionFrame->Transform(transform)) return output;
            BRepBuilderAPI_Transform placed(candidate, transform, Standard_True, Standard_False);
            if (!placed.IsDone()) { output.status = BuildStatus::KernelFailure; return output; }
            candidate = placed.Shape();
        }
        if (candidate.IsNull() || candidate.ShapeType() != TopAbs_SOLID) {
            output.status = BuildStatus::InvalidTopology; return output;
        }
        output.solid = TopoDS::Solid(candidate);
        output.status = BuildStatus::BuiltUnproven;
        output.diagnosticStage = 100;
        return output;
    } catch (...) {
        output.status = cancelled.load(std::memory_order_relaxed)
            ? BuildStatus::Cancelled : BuildStatus::KernelFailure;
        return output;
    }
}

} // namespace core3d::loft_correspondence
