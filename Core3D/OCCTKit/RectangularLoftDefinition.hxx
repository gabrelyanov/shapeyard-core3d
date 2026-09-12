#pragma once
// Detached numeric recipe. Authored IDs are correspondence, never OCAF authority.
#include "ProfileConstructionFrame.hxx"
#include <Precision.hxx>
#include <gp_Pnt.hxx>
#include <array>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <vector>

namespace core3d::rectangular_loft {
using ElementID=std::uint32_t;
struct Station {
    ElementID identifier=0;
    std::array<ElementID,4> cornerIdentifiers{};
    // Fixed CCW order: (-x,-y), (+x,-y), (+x,+y), (-x,+y).
    std::array<ElementID,4> correspondence{};
    double z=0,centerX=0,centerY=0,width=0,depth=0;
};
struct Definition {
    ElementID loftIdentifier=0;
    std::array<ElementID,4> correspondence{};
    std::vector<Station> stations; // Parallel XY planes, strictly increasing z.
    double dimensionMetersPerUnit=0;
    std::optional<profile::ConstructionFrame> constructionFrame;
};
enum class Admission { Accepted, InvalidCount, InvalidIdentifier, InvalidCorrespondence,
    InvalidNumber, InvalidFrame, InvalidOrder, Degenerate };
struct Inspection {
    double millimetersPerUnit=0,expectedVolume=0;
    std::array<double,6> expectedBounds{}; // Placed native units.
};
namespace detail {
constexpr double maximumNative=1e6,maximumPhysical=1e6,minimumPhysical=0.001;
inline std::array<gp_Pnt,4> Corners(const Station& s) {
    const double x0=s.centerX-s.width/2,x1=s.centerX+s.width/2;
    const double y0=s.centerY-s.depth/2,y1=s.centerY+s.depth/2;
    return {gp_Pnt(x0,y0,s.z),gp_Pnt(x1,y0,s.z),gp_Pnt(x1,y1,s.z),gp_Pnt(x0,y1,s.z)};
}
}
inline Admission Inspect(const Definition& d,Inspection& output) noexcept {
    output={};
    try {
        if (d.stations.size()<2 || d.stations.size()>8) return Admission::InvalidCount;
        if (!std::isfinite(d.dimensionMetersPerUnit) || d.dimensionMetersPerUnit<=0
            || d.dimensionMetersPerUnit>1e6) return Admission::InvalidNumber;
        const double mm=d.dimensionMetersPerUnit*1000;
        const auto finite=[&](double x) {return std::isfinite(x) && std::abs(x)<=detail::maximumNative
            && std::isfinite(x*mm) && std::abs(x*mm)<=detail::maximumPhysical;};
        const double scale=d.constructionFrame ? std::abs(d.constructionFrame->values[7]) : 1;
        gp_Trsf placement;
        if (d.constructionFrame && !d.constructionFrame->Transform(placement)) return Admission::InvalidFrame;
        const auto positive=[&](double x) {return finite(x) && x>=32*Precision::Confusion()
            && x*mm>=detail::minimumPhysical && std::isfinite(x*scale)
            && x*scale>=32*Precision::Confusion() && x*scale*mm>=detail::minimumPhysical;};
        std::set<ElementID> ids;
        const auto claim=[&](ElementID id) {return id && ids.insert(id).second;};
        if (!claim(d.loftIdentifier)) return Admission::InvalidIdentifier;
        for (auto id:d.correspondence) if (!claim(id)) return Admission::InvalidIdentifier;
        Inspection result;result.millimetersPerUnit=mm;
        result.expectedBounds={INFINITY,INFINITY,INFINITY,-INFINITY,-INFINITY,-INFINITY};
        double expected=0;
        for (std::size_t i=0;i<d.stations.size();++i) {
            const auto& s=d.stations[i];
            if (!claim(s.identifier)) return Admission::InvalidIdentifier;
            for (auto id:s.cornerIdentifiers) if (!claim(id)) return Admission::InvalidIdentifier;
            if (s.correspondence!=d.correspondence) return Admission::InvalidCorrespondence;
            if (!finite(s.z) || !finite(s.centerX) || !finite(s.centerY)
                || !finite(s.width) || !finite(s.depth)) return Admission::InvalidNumber;
            if (!positive(s.width) || !positive(s.depth)) return Admission::Degenerate;
            const auto corners=detail::Corners(s);
            if (corners[0].X()>=corners[1].X() || corners[0].Y()>=corners[3].Y()) return Admission::Degenerate;
            for (auto p:corners) {
                for (int axis=1;axis<=3;++axis) if (!finite(p.Coord(axis))) return Admission::InvalidNumber;
                if (d.constructionFrame) p.Transform(placement);
                for (int axis=0;axis<3;++axis) {
                    const double value=p.Coord(axis+1);
                    if (!finite(value)) return Admission::InvalidFrame;
                    result.expectedBounds[axis]=std::min(result.expectedBounds[axis],value);
                    result.expectedBounds[axis+3]=std::max(result.expectedBounds[axis+3],value);
                }
            }
            if (i) {
                const auto& previous=d.stations[i-1];const double length=s.z-previous.z;
                if (!std::isfinite(length) || length<=0) return Admission::InvalidOrder;
                if (!positive(length)) return Admission::Degenerate;
                // Linear interpolation of width and depth gives a quadratic
                // cross-sectional area. Center offsets do not change its integral.
                expected+=length/6*(2*previous.width*previous.depth+previous.width*s.depth
                    +s.width*previous.depth+2*s.width*s.depth);
            }
        }
        result.expectedVolume=expected*scale*scale*scale;
        if (!std::isfinite(result.expectedVolume) || result.expectedVolume<=0) return Admission::InvalidNumber;
        output=result;return Admission::Accepted;
    } catch (...) {output={};return Admission::InvalidNumber;}
}
class Prepared final {
public:
    const Definition definition;
    const Inspection inspection;
private:
    Prepared(Definition d,Inspection i):definition(std::move(d)),inspection(i) {}
    friend std::shared_ptr<const Prepared> Prepare(Definition,Admission&) noexcept;
};
inline std::shared_ptr<const Prepared> Prepare(Definition d,Admission& status) noexcept {
    try {
        Inspection i;status=Inspect(d,i);
        if (status!=Admission::Accepted) return {};
        return std::shared_ptr<const Prepared>(new Prepared(std::move(d),i));
    } catch (...) {status=Admission::InvalidNumber;return {};}
}
} // namespace core3d::rectangular_loft
