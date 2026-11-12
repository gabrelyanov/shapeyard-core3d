//
//  Snapping.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 25.11.2024.
//


#include "Snapping.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
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

std::optional<core3d::GeometrySnapResult> core3d::SnapPointToGeometry(
    const TopoDS_Shape& target, const gp_Pnt& point, double tolerance,
    unsigned kinds, const std::optional<gp_Dir>& axis) noexcept {
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
            if (axis && gp_Lin(point,*axis).Distance(candidate) > Precision::Confusion()) return;
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
        TopTools_IndexedMapOfShape vertices, edges, faces;
        if (enabled(GeometrySnapKind::Vertex)) {
            TopExp::MapShapes(target,TopAbs_VERTEX,vertices);
            for (int index = 1; index <= vertices.Extent(); ++index)
                consider(BRep_Tool::Pnt(TopoDS::Vertex(vertices(index))),
                         GeometrySnapKind::Vertex,index,vertices(index));
        }
        const auto probe = BRepBuilderAPI_MakeVertex(point).Vertex();
        const auto closest = [&](const TopoDS_Shape& topology, GeometrySnapKind kind, int index) {
            // Query individual trimmed edges/faces rather than the whole solid:
            // a point inside a solid must not be returned as an interior zero.
            BRepExtrema_DistShapeShape distance(probe,topology);
            if (!distance.IsDone()) throw Standard_Failure("Geometry snap distance failed");
            for (int solution = 1; solution <= distance.NbSolution(); ++solution)
                consider(distance.PointOnShape2(solution),kind,index,topology);
            if (!axis || tolerance == 0) return;
            const gp_Lin line(point,*axis);
            if (kind == GeometrySnapKind::Face) {
                // A normal projection can be off the drag axis, especially
                // on sloping and curved faces. Find all reachable crossings.
                IntCurvesFace_ShapeIntersector intersections;
                intersections.Load(topology,Precision::Confusion());
                intersections.Perform(line,-tolerance,tolerance);
                if (!intersections.IsDone()) throw Standard_Failure("Geometry snap intersection failed");
                for (int hit = 1; hit <= intersections.NbPnt(); ++hit)
                    consider(intersections.Pnt(hit),kind,index,topology);
            } else {
                const auto segment = BRepBuilderAPI_MakeEdge(line,-tolerance,tolerance).Edge();
                BRepExtrema_DistShapeShape crossings(segment,topology);
                if (!crossings.IsDone()) throw Standard_Failure("Geometry snap edge intersection failed");
                if (crossings.Value() <= Precision::Confusion()) {
                    for (int hit = 1; hit <= crossings.NbSolution(); ++hit)
                        consider(crossings.PointOnShape2(hit),kind,index,topology);
                }
            }
        };
        if (enabled(GeometrySnapKind::EdgeMidpoint) || enabled(GeometrySnapKind::Edge)) {
            TopExp::MapShapes(target,TopAbs_EDGE,edges);
            for (int index = 1; index <= edges.Extent(); ++index) {
                const auto edge = TopoDS::Edge(edges(index));
                if (BRep_Tool::Degenerated(edge)) continue;
                if (enabled(GeometrySnapKind::EdgeMidpoint)) {
                    BRepAdaptor_Curve curve(edge);
                    const double first = curve.FirstParameter(), last = curve.LastParameter();
                    if (std::isfinite(first) && std::isfinite(last)
                        && !Precision::IsInfinite(first) && !Precision::IsInfinite(last) && first < last) {
                        const double length = GCPnts_AbscissaPoint::Length(curve,first,last,1e-9);
                        if (std::isfinite(length) && length > 0) {
                            GCPnts_AbscissaPoint middle(1e-9,curve,length/2,first);
                            if (!middle.IsDone()) throw Standard_Failure("Geometry snap midpoint failed");
                            consider(curve.Value(middle.Parameter()),GeometrySnapKind::EdgeMidpoint,index,edge);
                        }
                    }
                }
                if (enabled(GeometrySnapKind::Edge)) closest(edge,GeometrySnapKind::Edge,index);
            }
        }
        if (enabled(GeometrySnapKind::Face)) {
            TopExp::MapShapes(target,TopAbs_FACE,faces);
            for (int index = 1; index <= faces.Extent(); ++index)
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

