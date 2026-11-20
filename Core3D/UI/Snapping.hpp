//
//  Snapping.hpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 25.11.2024.
//

#ifndef SNAPPING_HPP
#define SNAPPING_HPP

#include "SimpleSingleton.hpp"
#include <memory>
#include <optional>
#include "gp_Pnt.hxx"
#include "gp_Dir.hxx"
#include "TopoDS_Shape.hxx"
#include "Bnd_Box.hxx"
#include "TopTools_IndexedMapOfShape.hxx"
#include <memory>
#include <vector>

namespace core3d {
enum class GeometrySnapKind : unsigned { Vertex = 1, EdgeMidpoint = 2, Edge = 4, Face = 8 };
constexpr unsigned AllGeometrySnapKinds = 15;

struct GeometrySnapResult {
    gp_Pnt point;
    double distance = 0;
    GeometrySnapKind kind = GeometrySnapKind::Vertex;
    // One-based TopExp::MapShapes index in this exact target, for the kind's
    // topology (midpoints use EDGE). Not a durable ID across geometry edits.
    int topologyIndex = 0;
    TopoDS_Shape topology;
};

// Read-only query in the target's coordinate system, including its Location.
// Tolerance uses the same units as point/target. Finds the nearest enabled
// feature; ties prefer vertex, midpoint, edge, face, then topology order.
// Midpoint means half arc length of a trimmed edge. Faces respect trim holes.
// Optional axis restricts hits to the line through point, preserving a Move
// handle constraint. Intersections use the actual trimmed topology, not its
// infinite supporting surface. No hit, invalid input or geometry failure returns nullopt. Does not move a
// model, change grid snapping, selection, camera, document or history.
std::optional<GeometrySnapResult> SnapPointToGeometry(
    const TopoDS_Shape& target, const gp_Pnt& point, double tolerance,
    unsigned kinds = AllGeometrySnapKinds,
    const std::optional<gp_Dir>& axis = std::nullopt) noexcept;

// One target's corners, edges and faces (TopExp::MapShapes order) and a box
// around each edge and face, built once and reused for many points, e.g. every
// moving corner of one Move drag step. The box lets a query skip an edge or a
// face that lies farther than the tolerance from the point without measuring
// it; the answer is the same as SnapPointToGeometry on the shape.
// The distance and crossing tools of each edge and face are kept in the
// cache after their first use, so later points reuse them (same answers).
struct GeometrySnapCache;
struct GeometrySnapTarget {
    TopoDS_Shape shape;
    TopTools_IndexedMapOfShape vertices, edges, faces;
    std::vector<Bnd_Box> edgeBoxes, faceBoxes; // index - 1; void = always measured
    std::shared_ptr<GeometrySnapCache> cache;
};
std::optional<GeometrySnapTarget> PrepareGeometrySnapTarget(const TopoDS_Shape& target) noexcept;
std::optional<GeometrySnapResult> SnapPointToGeometry(
    const GeometrySnapTarget& target, const gp_Pnt& point, double tolerance,
    unsigned kinds = AllGeometrySnapKinds,
    const std::optional<gp_Dir>& axis = std::nullopt) noexcept;
}

class SnappingHolder;

class Snapping  :  public SimpleSingleton<Snapping> {
public:
    Snapping();
    
    ~Snapping() = default;
    
    void set(const double linear, const double angular, const double scaling);
    
    std::optional<double> getLinear() const;
    
    void setLinear(const double val);
    
    std::optional<double> getAngular() const;

    void setAngular(const double val);

    std::optional<double> getScaling() const;

    void setScaling(const double val);
    
    void setCurrentPosition(const gp_Pnt& val);
    
    gp_Pnt getCurrentPosition() const;
private:
    std::shared_ptr<SnappingHolder> _impl;
};

#endif //SNAPPING_HPP
