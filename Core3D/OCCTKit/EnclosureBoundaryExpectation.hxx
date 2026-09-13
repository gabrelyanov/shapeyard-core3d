#pragma once
// External source candidate. Fixed complete boundary, not native authority.
#include "EnclosureDefinition.hxx"
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <array>
#include <algorithm>
#include <vector>

namespace core3d::enclosure_correspondence {
struct EdgeUse { unsigned edge=0; bool forward=true; };
struct ExpectedEdge { unsigned start=0,end=0; bool circle=false; gp_Pnt center; double radius=0; };
struct ExpectedFace {
    bool cylinder=false;gp_Pnt origin;gp_Vec normalOrAxis;double radius=0;
    int radialSign=0;std::vector<std::vector<EdgeUse>> wires;
};
struct ExpectedBoundary {
    std::array<gp_Pnt,32> vertices;
    std::array<ExpectedEdge,48> edges;
    std::array<ExpectedFace,19> faces;
};
inline gp_Pnt PlanePoint(double u,double v,double w,int plane){
    return plane==0?gp_Pnt(u,v,w):plane==1?gp_Pnt(u,w,v):gp_Pnt(w,u,v);
}
inline gp_Vec PlaneVector(double u,double v,double w,int plane){
    const auto p=PlanePoint(u,v,w,plane);return gp_Vec(p.X(),p.Y(),p.Z());
}
inline std::vector<EdgeUse> Ring(unsigned start,bool forward){
    std::vector<EdgeUse> r;for(unsigned i=0;i<8;++i)r.push_back({start+i,true});
    if(!forward){std::reverse(r.begin(),r.end());for(auto& e:r)e.forward=false;}return r;
}
inline bool BuildExpectedBoundary(const EnclosureDefinition& d,ExpectedBoundary& out)noexcept{
    out={};try{
        EnclosureDerivedDimensions checked;if(!InspectEnclosureDefinition(d,checked))return false;
        gp_Trsf frame;
        if(d.constructionFrame&&(!d.constructionFrame->IsValid()||d.constructionFrame->values[7]<=0||!d.constructionFrame->Transform(frame)))return false;
        const auto& p=d.dimensions;const double s=frame.ScaleFactor();if(!std::isfinite(s)||s<=0)return false;
        const std::array<std::array<double,2>,4> centers{{{p.width-p.cornerRadius,p.cornerRadius},{p.width-p.cornerRadius,p.depth-p.cornerRadius},{p.cornerRadius,p.depth-p.cornerRadius},{p.cornerRadius,p.cornerRadius}}};
        const auto point=[&](double u,double v,double w){return PlanePoint(u,v,w,d.plane).Transformed(frame);};
        const auto vector=[&](double u,double v,double w){auto x=PlaneVector(u,v,w,d.plane);x.Transform(frame);return x;};
        for(unsigned role=0;role<2;++role){
            const double lo=role?p.wall:0,hiU=p.width-lo,hiV=p.depth-lo,r=p.cornerRadius-lo,z=role?p.floor:0;
            const std::array<std::array<double,2>,8> xy{{{lo+r,lo},{hiU-r,lo},{hiU,lo+r},{hiU,hiV-r},{hiU-r,hiV},{lo+r,hiV},{lo,hiV-r},{lo,lo+r}}};
            const unsigned vbase=role*16,ebase=role*24;
            for(unsigned i=0;i<8;++i){
                const unsigned j=(i+1)%8;out.vertices[vbase+i]=point(xy[i][0],xy[i][1],z);out.vertices[vbase+8+i]=point(xy[i][0],xy[i][1],p.height);
                for(unsigned level=0;level<2;++level){auto& e=out.edges[ebase+level*8+i];e.start=vbase+level*8+i;e.end=vbase+level*8+j;e.circle=i%2;
                    if(e.circle){e.center=point(centers[i/2][0],centers[i/2][1],level?p.height:z);e.radius=r*s;}}
                auto& e=out.edges[ebase+16+i];e.start=vbase+i;e.end=vbase+8+i;
                auto& f=out.faces[role*8+i];f.cylinder=i%2;
                if(f.cylinder){f.origin=point(centers[i/2][0],centers[i/2][1],0);f.normalOrAxis=vector(0,0,1);f.radius=r*s;f.radialSign=role?-1:1;}
                else {f.origin=out.vertices[vbase+i];const std::array<std::array<double,2>,4> normals{{{0,-1},{1,0},{0,1},{-1,0}}};const auto n=normals[i/2];f.normalOrAxis=vector(n[0]*(role?-1:1),n[1]*(role?-1:1),0);}
                std::vector<EdgeUse> wire{{ebase+i,true},{ebase+16+j,true},{ebase+8+i,false},{ebase+16+i,false}};
                if(role){std::reverse(wire.begin(),wire.end());for(auto& use:wire)use.forward=!use.forward;}
                f.wires.push_back(std::move(wire));
            }
        }
        out.faces[16].origin=point(0,0,0);out.faces[16].normalOrAxis=vector(0,0,-1);out.faces[16].wires={Ring(0,false)};
        out.faces[17].origin=point(0,0,p.floor);out.faces[17].normalOrAxis=vector(0,0,1);out.faces[17].wires={Ring(24,true)};
        out.faces[18].origin=point(0,0,p.height);out.faces[18].normalOrAxis=vector(0,0,1);out.faces[18].wires={Ring(8,true),Ring(32,false)};
        // XZ coordinates have negative chart handedness; reverse coedges, not F.
        if(d.plane==1)for(auto& f:out.faces)for(auto& wire:f.wires){std::reverse(wire.begin(),wire.end());for(auto& use:wire)use.forward=!use.forward;}
        for(const auto& v:out.vertices)if(!std::isfinite(v.X())||!std::isfinite(v.Y())||!std::isfinite(v.Z()))return false;
        return true;
    }catch(...){out={};return false;}
}
}
