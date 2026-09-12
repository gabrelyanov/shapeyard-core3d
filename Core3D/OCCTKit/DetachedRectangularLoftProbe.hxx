#pragma once
#if DEBUG
#include "RectangularLoftSolid.hxx"
#include <cstring>
#include <string>
namespace core3d::rectangular_loft::probe {
inline Definition Fixture(int fixture,double units,int frame=0) {
    Definition d;d.loftIdentifier=1;d.correspondence={10,11,12,13};d.dimensionMetersPerUnit=units;
    const double k=0.001/units;
    auto add=[&](double z,double x,double width,double depth) {
        const auto i=d.stations.size();Station s;s.identifier=ElementID(100+i);
        for(std::size_t j=0;j<4;++j)s.cornerIdentifiers[j]=ElementID(1000+10*i+j);
        s.correspondence=d.correspondence;s.z=z*k;s.centerX=x*k;s.centerY=0;
        s.width=width*k;s.depth=depth*k;d.stations.push_back(s);
    };
    if (fixture==0 || fixture==1) {add(0,0,20,12);add(60,6,30,fixture==1 ? 24 : 20);add(120,0,18,12);}
    else if (fixture==2) {add(0,0,20,12);add(120,0,20,12);}
    else for(int i=0;i<8;++i)add(i*20,0,20,12);
    if (frame) {
        profile::ConstructionFrame f;
        if(frame==1)f.values={20*k,-10*k,30*k,0,0,0,1,1};
        if(frame==2)f.values={0,0,0,0,0,std::sqrt(0.5),std::sqrt(0.5),1};
        if(frame==3)f.values={0,0,0,0,0,0,1,-2};
        d.constructionFrame=f;
    }
    return d;
}
inline std::vector<std::uint64_t> Snapshot(const Definition& d) {
    std::vector<std::uint64_t> words;
    const auto scalar=[&](double x) {std::uint64_t bits;std::memcpy(&bits,&x,sizeof bits);words.push_back(bits);};
    words.push_back(d.loftIdentifier);for(auto id:d.correspondence)words.push_back(id);
    scalar(d.dimensionMetersPerUnit);words.push_back(d.stations.size());
    for(const auto& s:d.stations) {
        words.push_back(s.identifier);for(auto id:s.cornerIdentifiers)words.push_back(id);
        for(auto id:s.correspondence)words.push_back(id);
        scalar(s.z);scalar(s.centerX);scalar(s.centerY);scalar(s.width);scalar(s.depth);
    }
    words.push_back(d.constructionFrame.has_value());
    if(d.constructionFrame)for(auto x:d.constructionFrame->values)scalar(x);
    return words;
}
inline const char* Name(Admission a) {
    switch(a) {
        case Admission::Accepted:return "accepted";case Admission::InvalidCount:return "count";
        case Admission::InvalidIdentifier:return "identifier";case Admission::InvalidCorrespondence:return "correspondence";
        case Admission::InvalidNumber:return "number";case Admission::InvalidFrame:return "frame";
        case Admission::InvalidOrder:return "order";case Admission::Degenerate:return "degenerate";
    }
    return "unknown";
}
inline const char* Name(BuildStatus s) {
    switch(s) {
        case BuildStatus::Built:return "built";case BuildStatus::Cancelled:return "cancelled";
        case BuildStatus::InvalidDefinition:return "invalid";case BuildStatus::KernelFailure:return "kernel";
        case BuildStatus::InvalidSolid:return "solid";case BuildStatus::SelfInterference:return "interference";
        case BuildStatus::VerificationFailed:return "verification";
    }
    return "unknown";
}
struct Rejection {std::string name;Definition definition;Admission expected;};
inline std::vector<Rejection> Rejections() {
    std::vector<Rejection> result;
    const auto add=[&](const char* name,Admission expected,const auto& mutation) {
        auto d=Fixture(0,0.001);mutation(d);result.push_back({name,std::move(d),expected});
    };
    add("one-station",Admission::InvalidCount,[](auto& d){d.stations.resize(1);});
    add("nine-stations",Admission::InvalidCount,[](auto& d){d.stations.resize(9);});
    add("zero-loft-id",Admission::InvalidIdentifier,[](auto& d){d.loftIdentifier=0;});
    add("duplicate-correspondence-id",Admission::InvalidIdentifier,[](auto& d){d.correspondence[1]=d.correspondence[0];});
    add("duplicate-station-id",Admission::InvalidIdentifier,[](auto& d){d.stations[1].identifier=d.stations[0].identifier;});
    add("duplicate-corner-id",Admission::InvalidIdentifier,[](auto& d){d.stations[1].cornerIdentifiers[0]=d.stations[0].cornerIdentifiers[0];});
    add("missing-corner-id",Admission::InvalidIdentifier,[](auto& d){d.stations[0].cornerIdentifiers[0]=0;});
    add("reversed-correspondence",Admission::InvalidCorrespondence,[](auto& d){std::reverse(d.stations[1].correspondence.begin(),d.stations[1].correspondence.end());});
    add("cyclic-correspondence",Admission::InvalidCorrespondence,[](auto& d){std::rotate(d.stations[1].correspondence.begin(),d.stations[1].correspondence.begin()+1,d.stations[1].correspondence.end());});
    add("foreign-correspondence",Admission::InvalidCorrespondence,[](auto& d){d.stations[1].correspondence[0]=999;});
    add("duplicate-plane",Admission::InvalidOrder,[](auto& d){d.stations[1].z=d.stations[0].z;});
    add("reversed-planes",Admission::InvalidOrder,[](auto& d){d.stations[1].z=-10;});
    add("too-close-planes",Admission::Degenerate,[](auto& d){d.stations[1].z=0.0001;});
    add("zero-width",Admission::Degenerate,[](auto& d){d.stations[1].width=0;});
    add("negative-depth",Admission::Degenerate,[](auto& d){d.stations[1].depth=-1;});
    add("subminimum-depth",Admission::Degenerate,[](auto& d){d.stations[1].depth=0.0001;});
    add("nan-z",Admission::InvalidNumber,[](auto& d){d.stations[1].z=std::numeric_limits<double>::quiet_NaN();});
    add("infinite-width",Admission::InvalidNumber,[](auto& d){d.stations[1].width=INFINITY;});
    add("zero-units",Admission::InvalidNumber,[](auto& d){d.dimensionMetersPerUnit=0;});
    add("nan-units",Admission::InvalidNumber,[](auto& d){d.dimensionMetersPerUnit=std::numeric_limits<double>::quiet_NaN();});
    add("physical-coordinate-limit",Admission::InvalidNumber,[](auto& d){d.dimensionMetersPerUnit=1;d.stations[1].centerX=1001;});
    add("native-corner-limit",Admission::InvalidNumber,[](auto& d){d.stations[1].centerX=1e6;});
    add("zero-frame-scale",Admission::InvalidFrame,[](auto& d){d.constructionFrame=profile::ConstructionFrame{};d.constructionFrame->values[7]=0;});
    add("invalid-quaternion",Admission::InvalidFrame,[](auto& d){d.constructionFrame=profile::ConstructionFrame{};d.constructionFrame->values[6]=0;});
    add("placed-coordinate-limit",Admission::InvalidFrame,[](auto& d){d.constructionFrame=profile::ConstructionFrame{};d.constructionFrame->values[0]=1e6;});
    add("placed-kernel-minimum",Admission::Degenerate,[](auto& d){d.stations[0].width=.001;d.constructionFrame=profile::ConstructionFrame{};d.constructionFrame->values[7]=1e-6;});
    return result;
}
} // namespace core3d::rectangular_loft::probe
#endif
