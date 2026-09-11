#pragma once

// Shared immutable dimensional input
// for a bounded outer-extrude / translated-inner-extrude / cut dependency.
#include "ProfileDefinition.hxx"
#include "ProfileCurvePresets.hxx"
#include <array>
#include <optional>

namespace core3d {
struct EnclosureDimensions {
    double width = 100, depth = 60, height = 30;
    double wall = 2, floor = 2, cornerRadius = 4;
    bool IsEqual(const EnclosureDimensions& other) const noexcept {
        return width == other.width && depth == other.depth && height == other.height
            && wall == other.wall && floor == other.floor && cornerRadius == other.cornerRadius;
    }
};
enum class EnclosureDimension { Width, Depth, Height, Wall, Floor, CornerRadius };
struct EnclosureDefinition {
    EnclosureDimensions dimensions;
    int plane = 0;
};
struct EnclosureDerivedDimensions {
    double innerWidth = 0, innerDepth = 0, innerHeight = 0, innerRadius = 0;
    double expectedVolume = 0;
};

inline bool InspectEnclosureDefinition(const EnclosureDefinition& definition,
    EnclosureDerivedDimensions& output) noexcept {
    output = {};
    const auto& d = definition.dimensions;
    constexpr double minimum = 1e-3, maximum = 1e6;
    if (definition.plane < 0 || definition.plane > 2) return false;
    for (double value : {d.width,d.depth,d.height,d.wall,d.floor,d.cornerRadius})
        if (!std::isfinite(value) || value < minimum || value > maximum) return false;
    // Keep both rounded loops nondegenerate, positive floor/walls, and an open
    // cavity. General offset corner joins and zero-radius boxes are separate.
    if (d.cornerRadius-d.wall < minimum || d.height-d.floor < minimum
        || d.width-2*d.cornerRadius < minimum || d.depth-2*d.cornerRadius < minimum) return false;
    EnclosureDerivedDimensions result;
    result.innerWidth=d.width-2*d.wall; result.innerDepth=d.depth-2*d.wall;
    result.innerHeight=d.height-d.floor; result.innerRadius=d.cornerRadius-d.wall;
    const double cornerLoss=4-std::acos(-1.0);
    const double outerArea=d.width*d.depth-cornerLoss*d.cornerRadius*d.cornerRadius;
    // Algebraic area difference avoids subtracting two nearly equal large areas.
    const double wallArea=2*d.wall*(d.width+d.depth)-4*d.wall*d.wall
        -cornerLoss*d.wall*(2*d.cornerRadius-d.wall);
    result.expectedVolume=outerArea*d.floor+wallArea*result.innerHeight;
    if (!std::isfinite(result.expectedVolume) || result.expectedVolume <= 1e-8
        || !std::isfinite(wallArea) || wallArea <= 0) return false;
    output=result;return true;
}

inline std::optional<EnclosureDefinition> UpdatedEnclosureDimension(
    const EnclosureDefinition& original, EnclosureDimension field, double value) noexcept {
    EnclosureDerivedDimensions inspection;
    if (!InspectEnclosureDefinition(original,inspection)) return {};
    auto candidate=original;
    switch(field) {
        case EnclosureDimension::Width: candidate.dimensions.width=value;break;
        case EnclosureDimension::Depth: candidate.dimensions.depth=value;break;
        case EnclosureDimension::Height: candidate.dimensions.height=value;break;
        case EnclosureDimension::Wall: candidate.dimensions.wall=value;break;
        case EnclosureDimension::Floor: candidate.dimensions.floor=value;break;
        case EnclosureDimension::CornerRadius: candidate.dimensions.cornerRadius=value;break;
        default:return {};
    }
    return InspectEnclosureDefinition(candidate,inspection) ? std::optional<EnclosureDefinition>(candidate) : std::nullopt;
}

// Stable local IDs are scoped to the owner feature. The fixed dependency graph
// cannot carry external references or cycles. Its two profiles are private
// construction inputs, not independently committed document objects.
struct EnclosureProfileDependency {
    ProfileDefinition outer;
    ProfileDefinition cavity;
    double cavityOffset = 0;
    double expectedVolume = 0;
};
inline bool DeriveEnclosureProfiles(const EnclosureDefinition& definition,
    EnclosureProfileDependency& output) noexcept {
    output={};
    try {
        EnclosureDerivedDimensions inspection;
        if (!InspectEnclosureDefinition(definition,inspection)) return false;
        const auto& d=definition.dimensions;
        ProfileCurvePresetIDs<8> outerIDs,innerIDs;
        outerIDs.loop=1;innerIDs.loop=18;
        for (std::size_t i=0;i<8;++i) {
            outerIDs.vertices[i]=ProfileCurveID(2+i);outerIDs.segments[i]=ProfileCurveID(10+i);
            innerIDs.vertices[i]=ProfileCurveID(19+i);innerIDs.segments[i]=ProfileCurveID(27+i);
        }
        ProfileCurveSection outer,inner;
        if (!BuildRoundedRectangleProfileLoop(gp_Pnt2d(0,0),gp_Pnt2d(d.width,d.depth),
                d.cornerRadius,outerIDs,outer.outer)
            || !BuildRoundedRectangleProfileLoop(gp_Pnt2d(d.wall,d.wall),
                gp_Pnt2d(d.width-d.wall,d.depth-d.wall),inspection.innerRadius,innerIDs,inner.outer)) return false;
        EnclosureProfileDependency result;
        result.outer.curves=std::move(outer);result.outer.plane=definition.plane;result.outer.depth=d.height;
        result.cavity.curves=std::move(inner);result.cavity.plane=definition.plane;result.cavity.depth=inspection.innerHeight;
        result.cavityOffset=d.floor;result.expectedVolume=inspection.expectedVolume;
        double area=0,volume=0;
        if (!ProfileDefinitionExpectedVolume(result.outer,area,volume)
            || !ProfileDefinitionExpectedVolume(result.cavity,area,volume)) return false;
        output=std::move(result);return true;
    } catch (...) {output={};return false;}
}
} // namespace core3d
