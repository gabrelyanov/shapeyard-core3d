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
#include "TopoDS_Shape.hxx"

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
// No hit, invalid input or geometry failure returns nullopt. Does not move a
// model, change grid snapping, selection, camera, document or history.
std::optional<GeometrySnapResult> SnapPointToGeometry(
    const TopoDS_Shape& target, const gp_Pnt& point, double tolerance,
    unsigned kinds = AllGeometrySnapKinds) noexcept;
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
