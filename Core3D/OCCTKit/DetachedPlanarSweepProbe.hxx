#pragma once
#if DEBUG
// Fixed diagnostic inputs only; this file is not a production command adapter.
#include "PlanarSweepSolid.hxx"
#include <cstring>
#include <limits>
#include <string>

namespace core3d::planar_sweep::probe {
inline Definition Fixture(int fixture,double metersPerUnit,int plane=1,int frame=0) {
    Definition d;d.pathIdentifier=1;d.plane=plane;d.dimensionMetersPerUnit=metersPerUnit;
    const double unit=0.001/metersPerUnit;
    auto vertex=[&](double u,double v) { d.vertices.push_back({ElementID(100+d.vertices.size()),gp_Pnt2d(u*unit,v*unit)}); };
    auto line=[&] { const auto i=d.segments.size();d.segments.push_back({ElementID(200+i),ElementID(100+i),ElementID(101+i),ProfileCurveKind::Line,{},0,0,0}); };
    auto arc=[&](double u,double v,double radius,double start,double angle) {
        const auto i=d.segments.size();d.segments.push_back({ElementID(200+i),ElementID(100+i),ElementID(101+i),
            ProfileCurveKind::CircularArc,gp_Pnt2d(u*unit,v*unit),radius*unit,start,angle});
    };
    if (fixture==0 || fixture==2) {
        d.radius=6*unit;vertex(0,0);vertex(0,300);vertex(80,380);vertex(fixture==2 ? 280 : 220,380);
        line();arc(80,300,80,180,-90);line();
    } else if (fixture==1 || fixture==3) {
        const double bend=fixture==3 ? 80 : 60;
        d.radius=8*unit;vertex(0,0);vertex(0,200);vertex(2*bend,200);vertex(2*bend,0);
        line();arc(bend,200,bend,180,-180);line();
    } else {
        d.radius=6*unit;vertex(0,0);vertex(100,0);line();
    }
    if (frame!=0) {
        profile::ConstructionFrame f;
        if (frame==1) f.values={20*unit,-10*unit,30*unit,0,0,0,1,1};
        if (frame==2) f.values={0,0,0,0,0,std::sqrt(0.5),std::sqrt(0.5),1};
        if (frame==3) f.values={0,0,0,0,0,0,1,-2};
        d.constructionFrame=f;
    }
    return d;
}
inline std::vector<std::uint64_t> Snapshot(const Definition& d) {
    std::vector<std::uint64_t> values;
    const auto scalar=[&](double value) { std::uint64_t bits;std::memcpy(&bits,&value,sizeof bits);values.push_back(bits); };
    values.push_back(d.pathIdentifier);values.push_back(d.plane);scalar(d.radius);scalar(d.dimensionMetersPerUnit);
    values.push_back(d.vertices.size());
    for (const auto& v:d.vertices) {values.push_back(v.identifier);scalar(v.point.X());scalar(v.point.Y());}
    values.push_back(d.segments.size());
    for (const auto& e:d.segments) {
        values.push_back(e.identifier);values.push_back(e.startVertex);values.push_back(e.endVertex);values.push_back(std::uint64_t(e.kind));
        scalar(e.center.X());scalar(e.center.Y());scalar(e.radius);scalar(e.startDegrees);scalar(e.sweepDegrees);
    }
    values.push_back(d.constructionFrame.has_value());
    if (d.constructionFrame) for (double value:d.constructionFrame->values) scalar(value);
    return values;
}
inline const char* Name(Admission a) {
    switch (a) {
        case Admission::Accepted:return "accepted";case Admission::InvalidCount:return "count";
        case Admission::InvalidIdentifier:return "identifier";case Admission::InvalidNumber:return "number";
        case Admission::InvalidFrame:return "frame";case Admission::InvalidConnectivity:return "connectivity";
        case Admission::Degenerate:return "degenerate";case Admission::InvalidArc:return "arc";
        case Admission::NonTangent:return "tangent";case Admission::TightBend:return "bend";
        case Admission::NonlocalContact:return "nonlocal";
    }
    return "invalid-enum";
}
inline const char* Name(BuildStatus s) {
    switch(s) {
        case BuildStatus::Built:return "built";case BuildStatus::InvalidDefinition:return "invalid";
        case BuildStatus::Cancelled:return "cancelled";case BuildStatus::KernelFailure:return "kernel";
        case BuildStatus::InvalidSolid:return "solid";case BuildStatus::SelfInterference:return "interference";
        case BuildStatus::VerificationFailed:return "verification";
    }
    return "invalid-enum";
}
struct Rejection { std::string name;Definition definition;Admission expected; };
inline std::vector<Rejection> Rejections() {
    std::vector<Rejection> cases;
    const auto add=[&](const char* name,Admission expected,const auto& change) {
        auto d=Fixture(0,0.001);change(d);cases.push_back({name,std::move(d),expected});
    };
    add("empty",Admission::InvalidCount,[](auto& d){d.segments.clear();});
    add("too-many",Admission::InvalidCount,[](auto& d){d.segments.resize(33);d.vertices.resize(34);});
    add("duplicate-id",Admission::InvalidIdentifier,[](auto& d){d.vertices[1].identifier=d.pathIdentifier;});
    add("zero-id",Admission::InvalidIdentifier,[](auto& d){d.segments[0].identifier=0;});
    add("foreign-endpoint",Admission::InvalidConnectivity,[](auto& d){d.segments[0].endVertex=555;});
    add("reordered-edges",Admission::InvalidConnectivity,[](auto& d){std::swap(d.segments[0],d.segments[1]);});
    add("zero-length",Admission::Degenerate,[](auto& d){d.vertices[1].point=d.vertices[0].point;});
    add("nan-point",Admission::InvalidNumber,[](auto& d){d.vertices[1].point.SetX(std::numeric_limits<double>::quiet_NaN());});
    add("infinite-radius",Admission::InvalidNumber,[](auto& d){d.radius=std::numeric_limits<double>::infinity();});
    add("zero-units",Admission::InvalidNumber,[](auto& d){d.dimensionMetersPerUnit=0;});
    add("overflow-coordinate",Admission::InvalidNumber,[](auto& d){d.vertices[0].point.SetX(1e6+1);});
    add("physical-coordinate-limit",Admission::InvalidNumber,[](auto& d){d.dimensionMetersPerUnit=1;d.vertices[0].point.SetX(1001);});
    add("hidden-line-fields",Admission::InvalidArc,[](auto& d){d.segments[0].radius=1;});
    add("unknown-segment-kind",Admission::InvalidArc,[](auto& d){d.segments[0].kind=ProfileCurveKind(77);});
    add("arc-endpoint-mismatch",Admission::InvalidArc,[](auto& d){d.vertices[2].point.SetY(381);});
    add("arc-over-half-turn",Admission::InvalidArc,[](auto& d){d.segments[1].sweepDegrees=-181;});
    add("arc-zero-angle",Admission::InvalidArc,[](auto& d){d.segments[1].sweepDegrees=0;});
    add("arc-start-out-of-range",Admission::InvalidArc,[](auto& d){d.segments[1].startDegrees=361;});
    add("tight-bend",Admission::TightBend,[](auto& d){d.radius=21;});
    add("non-tangent-junction",Admission::NonTangent,[](auto& d){d.vertices[3].point.SetY(382);});
    add("zero-frame-scale",Admission::InvalidFrame,[](auto& d){d.constructionFrame=profile::ConstructionFrame{};d.constructionFrame->values[7]=0;});
    add("invalid-quaternion",Admission::InvalidFrame,[](auto& d){d.constructionFrame=profile::ConstructionFrame{};d.constructionFrame->values[6]=0;});
    // Two individually valid semicircles with a G1 common join return to the
    // same position at their far ends. An unconditional adjacent-edge skip fails.
    auto loop=Fixture(1,0.001);loop.vertices={{100,{0,0}},{101,{120,0}},{102,{0,0}}};
    loop.segments={{200,100,101,ProfileCurveKind::CircularArc,{60,0},60,180,-180},
                   {201,101,102,ProfileCurveKind::CircularArc,{60,0},60,0,-180}};
    cases.push_back({"adjacent-far-contact",std::move(loop),Admission::NonlocalContact});
    // Conservative documented limit: tiny collinear subdivisions have disjoint
    // nonadjacent centerlines closer than the tube diameter. No silent merge.
    auto shortSegments=Fixture(4,0.001);shortSegments.vertices={{100,{0,0}},{101,{20,0}},{102,{21,0}},{103,{40,0}}};
    shortSegments.segments={{200,100,101,ProfileCurveKind::Line,{},0,0,0},
        {201,101,102,ProfileCurveKind::Line,{},0,0,0},{202,102,103,ProfileCurveKind::Line,{},0,0,0}};
    cases.push_back({"conservative-close-subdivision",std::move(shortSegments),Admission::NonlocalContact});
    return cases;
}
} // namespace core3d::planar_sweep::probe
#endif
