//
//  Snapping.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 25.11.2024.
//


#include "Snapping.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <IntCurvesFace_Intersector.hxx>
#include <gp_Lin.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <TopExp.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

// Per target, filled lazily while points are queried. A distance tool keeps
// its edge or face (shape 2) and the boxes OCCT computed for it; only the
// probe (shape 1) is loaded again. A face intersector is built once and
// performed for each new line. A midpoint is measured once per edge.
struct core3d::GeometrySnapCache {
    enum class Midpoint : unsigned char { Unknown, None, Known, Failed };
    std::vector<std::unique_ptr<BRepExtrema_DistShapeShape>> edgeDistances, faceDistances;
    std::vector<Handle(IntCurvesFace_Intersector)> faceIntersectors;
    std::vector<Midpoint> midpointStates;
    std::vector<gp_Pnt> midpoints;
};

std::optional<core3d::GeometrySnapTarget> core3d::PrepareGeometrySnapTarget(
    const TopoDS_Shape& target) noexcept {
    if (target.IsNull()) return std::nullopt;
    try {
        GeometrySnapTarget prepared;
        prepared.shape = target;
        TopExp::MapShapes(target,TopAbs_VERTEX,prepared.vertices);
        TopExp::MapShapes(target,TopAbs_EDGE,prepared.edges);
        TopExp::MapShapes(target,TopAbs_FACE,prepared.faces);
        // Boxes from the exact geometry (not a display mesh), which contain
        // the edge or face with its tolerance. Any doubt leaves a box void,
        // and a void box never skips a measurement.
        const auto boxes = [](const TopTools_IndexedMapOfShape& map, std::vector<Bnd_Box>& out) {
            out.resize(static_cast<std::size_t>(map.Extent()));
            for (int index = 1; index <= map.Extent(); ++index) {
                Bnd_Box box;
                try { BRepBndLib::Add(map(index),box,Standard_False); } catch (...) { box.SetVoid(); }
                if (box.IsOpen()) box.SetVoid();
                out[static_cast<std::size_t>(index - 1)] = box;
            }
        };
        boxes(prepared.edges,prepared.edgeBoxes);
        boxes(prepared.faces,prepared.faceBoxes);
        auto cache = std::make_shared<GeometrySnapCache>();
        const auto edgeCount = static_cast<std::size_t>(prepared.edges.Extent());
        const auto faceCount = static_cast<std::size_t>(prepared.faces.Extent());
        cache->edgeDistances.resize(edgeCount);
        cache->faceDistances.resize(faceCount);
        cache->faceIntersectors.resize(faceCount);
        cache->midpointStates.assign(edgeCount,GeometrySnapCache::Midpoint::Unknown);
        cache->midpoints.resize(edgeCount);
        prepared.cache = std::move(cache);
        return prepared;
    } catch (...) { return std::nullopt; }
}

std::optional<core3d::GeometrySnapResult> core3d::SnapPointToGeometry(
    const TopoDS_Shape& target, const gp_Pnt& point, double tolerance,
    unsigned kinds, const std::optional<gp_Dir>& axis) noexcept {
    if (target.IsNull()) return std::nullopt;
    const auto prepared = PrepareGeometrySnapTarget(target);
    if (!prepared) return std::nullopt;
    return SnapPointToGeometry(*prepared,point,tolerance,kinds,axis);
}

std::optional<core3d::GeometrySnapResult> core3d::SnapPointToGeometry(
    const GeometrySnapTarget& prepared, const gp_Pnt& point, double tolerance,
    unsigned kinds, const std::optional<gp_Dir>& axis) noexcept {
    const TopoDS_Shape& target = prepared.shape;
    const auto finitePoint = [](const gp_Pnt& p) {
        return std::isfinite(p.X()) && std::isfinite(p.Y()) && std::isfinite(p.Z());
    };
    if (target.IsNull() || !finitePoint(point) || !std::isfinite(tolerance)
        || tolerance < 0 || kinds == 0 || (kinds & ~AllGeometrySnapKinds) != 0
        || (axis && (!std::isfinite(axis->X()) || !std::isfinite(axis->Y()) || !std::isfinite(axis->Z()))))
        return std::nullopt;
    try {
        std::optional<GeometrySnapResult> best;
        const auto enabled = [kinds](GeometrySnapKind kind) {
            return (kinds & static_cast<unsigned>(kind)) != 0;
        };
        const auto consider = [&](const gp_Pnt& candidate, GeometrySnapKind kind,
                                  int index, const TopoDS_Shape& topology) {
            if (!finitePoint(candidate)) return;
            if (axis) {
                // OCCT's modelling tolerance may admit a nearby corner that
                // the constrained Move cannot actually reach. Allow only
                // coordinate-sized floating-point roundoff; otherwise the
                // feedback point would disagree with the projected movement.
                const double roundoff = 64 * std::numeric_limits<double>::epsilon()
                    * std::max({1.0,std::abs(point.X()),std::abs(point.Y()),std::abs(point.Z()),
                                std::abs(candidate.X()),std::abs(candidate.Y()),std::abs(candidate.Z())});
                if (gp_Lin(point,*axis).Distance(candidate) > roundoff) return;
            }
            const double distance = point.Distance(candidate);
            if (!std::isfinite(distance) || distance > tolerance) return;
            if (best) {
                // Only roundoff-sized ties: feature priority must never pull
                // a point to a materially farther corner instead of a face.
                const double epsilon = 64 * std::numeric_limits<double>::epsilon()
                    * std::max({1.0, distance, best->distance});
                if (distance > best->distance + epsilon) return;
                if (std::abs(distance - best->distance) <= epsilon) {
                    if (static_cast<unsigned>(kind) > static_cast<unsigned>(best->kind)) return;
                    if (kind == best->kind && index >= best->topologyIndex) return;
                }
            }
            best = GeometrySnapResult{candidate,distance,kind,index,topology};
        };
        // Every candidate of an edge or face lies on it, and consider() admits
        // only candidates within the tolerance: a piece whose box, widened by
        // the tolerance, misses the point cannot answer and is not measured.
        const auto outOfReach = [&](const std::vector<Bnd_Box>& boxes, int index) {
            if (index < 1 || static_cast<std::size_t>(index) > boxes.size()) return false;
            Bnd_Box box = boxes[static_cast<std::size_t>(index - 1)];
            if (box.IsVoid()) return false;
            box.Enlarge(tolerance + Precision::Confusion());
            return box.IsOut(point) == Standard_True;
        };
        const auto& vertices = prepared.vertices;
        const auto& edges = prepared.edges;
        const auto& faces = prepared.faces;
        if (enabled(GeometrySnapKind::Vertex)) {
            for (int index = 1; index <= vertices.Extent(); ++index)
                consider(BRep_Tool::Pnt(TopoDS::Vertex(vertices(index))),
                         GeometrySnapKind::Vertex,index,vertices(index));
        }
        const auto probe = BRepBuilderAPI_MakeVertex(point).Vertex();
        GeometrySnapCache* const cache = prepared.cache.get();
        const auto slot = [](auto& tools, int index) -> decltype(tools.data()) {
            if (index < 1 || static_cast<std::size_t>(index) > tools.size()) return nullptr;
            return &tools[static_cast<std::size_t>(index - 1)];
        };
        // The distance from shape 1 to an edge or face of this target. A kept
        // tool holds exactly the edge or face of the target map at that index
        // (callers pass that shape, or index 0 for a fresh tool, as before).
        const auto measure = [&](std::unique_ptr<BRepExtrema_DistShapeShape>& scratch,
                                 const TopoDS_Shape& from, const TopoDS_Shape& topology,
                                 GeometrySnapKind kind, int index) -> BRepExtrema_DistShapeShape& {
            std::unique_ptr<BRepExtrema_DistShapeShape>* kept = nullptr;
            if (cache) kept = kind == GeometrySnapKind::Face
                ? slot(cache->faceDistances,index) : slot(cache->edgeDistances,index);
            if (!kept) kept = &scratch;
            if (!*kept) {
                *kept = std::make_unique<BRepExtrema_DistShapeShape>();
                (*kept)->LoadS2(topology);
            }
            (*kept)->LoadS1(from);
            (*kept)->Perform();
            return **kept;
        };
        std::optional<TopoDS_Edge> segmentEdge;
        std::optional<Bnd_Box> segmentBox;
        const auto closest = [&](const TopoDS_Shape& topology, GeometrySnapKind kind, int index) {
            // Query individual trimmed edges/faces rather than the whole solid:
            // a point inside a solid must not be returned as an interior zero.
            std::unique_ptr<BRepExtrema_DistShapeShape> scratch;
            auto& distance = measure(scratch,probe,topology,kind,index);
            if (!distance.IsDone()) throw Standard_Failure("Geometry snap distance failed");
            for (int solution = 1; solution <= distance.NbSolution(); ++solution)
                consider(distance.PointOnShape2(solution),kind,index,topology);
            if (!axis || tolerance == 0 || distance.Value() > tolerance) return;
            const gp_Lin line(point,*axis);
            // A search segment shorter than OCCT's modelling precision can
            // be degenerate. Final candidate admission still uses the exact
            // caller tolerance, including tiny positive values.
            const double span = std::max(tolerance,2 * Precision::Confusion());
            if (!segmentEdge) {
                segmentEdge = BRepBuilderAPI_MakeEdge(line,-span,span).Edge();
                Bnd_Box box;
                box.Add(gp_Pnt(point.XYZ() - axis->XYZ() * span));
                box.Add(gp_Pnt(point.XYZ() + axis->XYZ() * span));
                box.Enlarge(2 * Precision::Confusion());
                segmentBox = box;
            }
            const TopoDS_Edge& segment = *segmentEdge;
            const auto intersectEdge = [&](const TopoDS_Shape& edge) {
                // A crossing is a point of the segment within Confusion of
                // the edge: an edge whose box misses the segment's box has none.
                const int edgeIndex = prepared.edges.FindIndex(edge);
                if (edgeIndex > 0 && static_cast<std::size_t>(edgeIndex) <= prepared.edgeBoxes.size()) {
                    const Bnd_Box& edgeBox = prepared.edgeBoxes[static_cast<std::size_t>(edgeIndex - 1)];
                    if (!edgeBox.IsVoid() && edgeBox.IsOut(*segmentBox)) return;
                }
                // A face's boundary edge may carry another orientation than
                // the map's edge: it then gets a fresh tool, as before.
                const bool mapped = edgeIndex > 0 && edge.IsEqual(prepared.edges(edgeIndex));
                std::unique_ptr<BRepExtrema_DistShapeShape> fresh;
                auto& crossings = measure(fresh,segment,edge,GeometrySnapKind::Edge,mapped ? edgeIndex : 0);
                if (!crossings.IsDone()) throw Standard_Failure("Geometry snap edge intersection failed");
                if (crossings.Value() <= Precision::Confusion()) {
                    for (int hit = 1; hit <= crossings.NbSolution(); ++hit)
                        consider(crossings.PointOnShape2(hit),kind,index,topology);
                }
            };
            if (kind == GeometrySnapKind::Face) {
                // A normal projection can be off the drag axis, especially
                // on sloping and curved faces. Find all reachable crossings.
                // Do not inflate a trim boundary by its stored edge tolerance:
                // a line just outside the face (or inside a hole) cannot land
                // on the face's infinite supporting surface.
                Handle(IntCurvesFace_Intersector) fresh;
                Handle(IntCurvesFace_Intersector)* kept =
                    cache ? slot(cache->faceIntersectors,index) : nullptr;
                if (!kept) kept = &fresh;
                if (kept->IsNull()) *kept = new IntCurvesFace_Intersector(TopoDS::Face(topology),
                    Precision::Confusion(),Standard_True,Standard_False);
                IntCurvesFace_Intersector& intersections = **kept;
                intersections.Perform(line,-tolerance,tolerance);
                if (!intersections.IsDone()) throw Standard_Failure("Geometry snap intersection failed");
                for (int hit = 1; hit <= intersections.NbPnt(); ++hit)
                    consider(intersections.Pnt(hit),kind,index,topology);
                if (intersections.NbPnt() == 0) {
                    // A line in a planar face has no isolated surface
                    // intersection. Its trim boundary, including hole wires,
                    // supplies the nearest reachable entry into that face.
                    TopTools_IndexedMapOfShape boundary;
                    TopExp::MapShapes(topology,TopAbs_EDGE,boundary);
                    for (int edge = 1; edge <= boundary.Extent(); ++edge) {
                        if (!BRep_Tool::Degenerated(TopoDS::Edge(boundary(edge))))
                            intersectEdge(boundary(edge));
                    }
                }
            } else {
                intersectEdge(topology);
            }
        };
        if (enabled(GeometrySnapKind::EdgeMidpoint) || enabled(GeometrySnapKind::Edge)) {
            for (int index = 1; index <= edges.Extent(); ++index) {
                const auto edge = TopoDS::Edge(edges(index));
                if (BRep_Tool::Degenerated(edge) || outOfReach(prepared.edgeBoxes,index)) continue;
                if (enabled(GeometrySnapKind::EdgeMidpoint)) {
                    using Midpoint = GeometrySnapCache::Midpoint;
                    Midpoint state = Midpoint::Unknown;
                    gp_Pnt middlePoint;
                    Midpoint* keptState = cache ? slot(cache->midpointStates,index) : nullptr;
                    if (keptState) state = *keptState;
                    if (state == Midpoint::Known) middlePoint = cache->midpoints[static_cast<std::size_t>(index - 1)];
                    if (state == Midpoint::Unknown) {
                        // A failure is remembered, so every later point fails
                        // on this edge too, exactly as a fresh measurement would.
                        if (keptState) *keptState = Midpoint::Failed;
                        state = Midpoint::None;
                        BRepAdaptor_Curve curve(edge);
                        const double first = curve.FirstParameter(), last = curve.LastParameter();
                        if (std::isfinite(first) && std::isfinite(last)
                            && !Precision::IsInfinite(first) && !Precision::IsInfinite(last) && first < last) {
                            const double length = GCPnts_AbscissaPoint::Length(curve,first,last,1e-9);
                            if (std::isfinite(length) && length > 0) {
                                GCPnts_AbscissaPoint middle(1e-9,curve,length/2,first);
                                if (!middle.IsDone()) throw Standard_Failure("Geometry snap midpoint failed");
                                middlePoint = curve.Value(middle.Parameter());
                                state = Midpoint::Known;
                            }
                        }
                        if (keptState) {
                            *keptState = state;
                            if (state == Midpoint::Known) cache->midpoints[static_cast<std::size_t>(index - 1)] = middlePoint;
                        }
                    }
                    if (state == Midpoint::Failed) throw Standard_Failure("Geometry snap midpoint failed");
                    if (state == Midpoint::Known)
                        consider(middlePoint,GeometrySnapKind::EdgeMidpoint,index,edge);
                }
                if (enabled(GeometrySnapKind::Edge)) closest(edge,GeometrySnapKind::Edge,index);
            }
        }
        if (enabled(GeometrySnapKind::Face)) {
            for (int index = 1; index <= faces.Extent(); ++index)
                if (!outOfReach(prepared.faceBoxes,index))
                    closest(faces(index),GeometrySnapKind::Face,index);
        }
        return best;
    } catch (...) {
        // Do not publish a partial nearest answer after a geometry failure.
        return std::nullopt;
    }
}

struct SnappingHolder {
    double linear = 0;
    double angular = 0;
    double scaling = 0;
    gp_Pnt currentPosition{};
};

Snapping::Snapping()
: _impl(std::make_shared<SnappingHolder>()) {
    
}

void Snapping::set(const double linear, const double angular, const double scaling) {
    _impl->linear = linear;
    _impl->angular = angular;
    _impl->scaling = scaling;
}

std::optional<double> Snapping::getLinear() const {
    if(abs(_impl->linear) > std::numeric_limits<double>::epsilon()) {
        return _impl->linear;
    }
    return std::nullopt;
}

void Snapping::setLinear(const double val) {
    _impl->linear = val;
}

std::optional<double> Snapping::getAngular() const {
    if(abs(_impl->angular) > std::numeric_limits<double>::epsilon()) {
        return _impl->angular;
    }
    return std::nullopt;
}

void Snapping::setAngular(const double val) {
    _impl->angular = val;
}

std::optional<double> Snapping::getScaling() const {
    if(abs(_impl->scaling) > std::numeric_limits<double>::epsilon()) {
        return _impl->scaling;
    }
    return std::nullopt;
}

void Snapping::setScaling(const double val) {
    _impl->scaling = val;
}

void Snapping::setCurrentPosition(const gp_Pnt& val) {
    _impl->currentPosition = val;
}

gp_Pnt Snapping::getCurrentPosition() const {
    return _impl->currentPosition;
}

