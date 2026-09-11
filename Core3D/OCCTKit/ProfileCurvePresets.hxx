#pragma once

// Presets create explicit editable
// curves; they do not infer a preset from arbitrary saved geometry. Callers
// own feature/element identity allocation. Structural success still requires
// shared section/face/solid admission before any document mutation.
#include "ProfileCurveStructure.hxx"
#include <utility>

namespace core3d {
template<std::size_t N> struct ProfileCurvePresetIDs {
    ProfileCurveID loop = 0;
    std::array<ProfileCurveID,N> vertices{};
    std::array<ProfileCurveID,N> segments{};
};

inline bool BuildRoundedRectangleProfileLoop(const gp_Pnt2d& minimum,
    const gp_Pnt2d& maximum, double radius,
    const ProfileCurvePresetIDs<8>& ids, ProfileCurveLoop& output) noexcept {
    output = {};
    try {
        const double x = minimum.X(), y = minimum.Y();
        const double right = maximum.X(), top = maximum.Y();
        for (double value : {x,y,right,top,radius})
            if (!std::isfinite(value) || std::abs(value) > 1e6) return false;
        if (radius < .001 || right-x-2*radius < .001 || top-y-2*radius < .001) return false;
        ProfileCurveLoop loop; loop.identifier = ids.loop;
        const std::array<gp_Pnt2d,8> points = {gp_Pnt2d(x+radius,y),gp_Pnt2d(right-radius,y),
            gp_Pnt2d(right,y+radius),gp_Pnt2d(right,top-radius),gp_Pnt2d(right-radius,top),
            gp_Pnt2d(x+radius,top),gp_Pnt2d(x,top-radius),gp_Pnt2d(x,y+radius)};
        const std::array<gp_Pnt2d,4> centers = {gp_Pnt2d(right-radius,y+radius),
            gp_Pnt2d(right-radius,top-radius),gp_Pnt2d(x+radius,top-radius),gp_Pnt2d(x+radius,y+radius)};
        for (std::size_t i=0;i<8;++i) {
            loop.vertices.push_back({ids.vertices[i],points[i]});
            ProfileCurveSegment segment; segment.identifier=ids.segments[i];
            segment.startVertex=ids.vertices[i];segment.endVertex=ids.vertices[(i+1)%8];
            if (i%2==1) {
                const std::size_t corner=i/2;
                segment.kind=ProfileCurveKind::CircularArc;segment.center=centers[corner];
                segment.radius=radius;segment.startDegrees=-90+90*double(corner);segment.sweepDegrees=90;
            }
            loop.segments.push_back(segment);
        }
        std::set<ProfileCurveID> claimed;std::size_t vertices=0,segments=0;ProfileCurveLoopInspection inspection;
        if (!InspectProfileCurveLoopStructure(loop,minimum,claimed,vertices,segments,inspection)) return false;
        output=std::move(loop);return true;
    } catch (...) { output={};return false; }
}

inline bool BuildCapsuleProfileLoop(const gp_Pnt2d& center, double length,
    double diameter, double rotationDegrees, const ProfileCurvePresetIDs<4>& ids,
    ProfileCurveLoop& output) noexcept {
    output={};
    try {
        for (double value : {center.X(),center.Y(),length,diameter,rotationDegrees})
            if (!std::isfinite(value) || std::abs(value)>1e6) return false;
        if (diameter<.002 || length-diameter<.001 || std::abs(rotationDegrees)>360) return false;
        const double radius=diameter/2, halfStraight=(length-diameter)/2;
        const double radians=rotationDegrees*std::acos(-1.0)/180.0;
        const double c=std::cos(radians),s=std::sin(radians);
        const auto point=[&](double u,double v) { return gp_Pnt2d(center.X()+c*u-s*v,center.Y()+s*u+c*v); };
        const std::array<gp_Pnt2d,4> points={point(-halfStraight,-radius),point(-halfStraight,radius),
            point(halfStraight,radius),point(halfStraight,-radius)};
        ProfileCurveLoop loop;loop.identifier=ids.loop;
        for (std::size_t i=0;i<4;++i) {
            loop.vertices.push_back({ids.vertices[i],points[i]});
            ProfileCurveSegment segment;segment.identifier=ids.segments[i];
            segment.startVertex=ids.vertices[i];segment.endVertex=ids.vertices[(i+1)%4];
            if (i%2==0) {
                segment.kind=ProfileCurveKind::CircularArc;
                segment.center=point(i==2?halfStraight:-halfStraight,0);
                segment.radius=radius;segment.startDegrees=std::remainder((i==2?90.0:-90.0)+rotationDegrees,360.0);
                segment.sweepDegrees=-180;
            }
            loop.segments.push_back(segment);
        }
        std::set<ProfileCurveID> claimed;std::size_t vertices=0,segments=0;ProfileCurveLoopInspection inspection;
        if (!InspectProfileCurveLoopStructure(loop,center,claimed,vertices,segments,inspection)) return false;
        output=std::move(loop);return true;
    } catch (...) { output={};return false; }
}
} // namespace core3d
