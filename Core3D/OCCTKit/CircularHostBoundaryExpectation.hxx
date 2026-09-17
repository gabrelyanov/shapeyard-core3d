#pragma once
#include "EnclosureBoundaryExpectation.hxx"
#include "ProfilePersistence.hxx"

namespace core3d::saved_cut_circular_host {
struct Expectation {
    std::vector<gp_Pnt> vertices;
    std::vector<enclosure_correspondence::ExpectedEdge> edges;
    std::vector<enclosure_correspondence::ExpectedFace> faces;
    std::array<unsigned,2> caps{0,1};
    unsigned hostGenus=0;
};
// Independent recipe cells. Meridian is the authored +U circle basis, never
// inferred from a kernel seam or from a retained result.
inline bool BuildExpectedBoundary(const ProfileDefinition& definition,
    const std::optional<profile::ConstructionFrame>& constructionFrame,Expectation& out) noexcept {
    out={};try {
        double area=0,volume=0;
        if(!definition.circle||definition.revolve||definition.curves||!definition.points.empty()
            ||!definition.holes.empty()||!ProfileDefinitionExpectedVolume(definition,area,volume))return false;
        gp_Trsf frame;
        if(constructionFrame&&(!constructionFrame->IsValid()||constructionFrame->values[7]<=0
            ||!constructionFrame->Transform(frame)))return false;
        const double scale=frame.ScaleFactor();if(!std::isfinite(scale)||scale<=0)return false;
        const auto& c=*definition.circle;out.hostGenus=c.innerRadius>0?1:0;
        const auto point=[&](double u,double v,double w){return enclosure_correspondence::PlanePoint(u,v,w,definition.plane).Transformed(frame);};
        const auto vector=[&](double u,double v,double w){auto v3=enclosure_correspondence::PlaneVector(u,v,w,definition.plane);v3.Transform(frame);return v3;};
        out.faces.resize(3+out.hostGenus);
        out.faces[0].origin=point(c.center.X(),c.center.Y(),0);out.faces[0].normalOrAxis=vector(0,0,-1);
        out.faces[1].origin=point(c.center.X(),c.center.Y(),definition.depth);out.faces[1].normalOrAxis=vector(0,0,1);
        for(unsigned ring=0;ring<=out.hostGenus;++ring){
            const double radius=ring?c.innerRadius:c.outerRadius;const unsigned v=2*ring,e=3*ring;
            out.vertices.push_back(point(c.center.X()+radius,c.center.Y(),0));
            out.vertices.push_back(point(c.center.X()+radius,c.center.Y(),definition.depth));
            for(unsigned level=0;level<2;++level)out.edges.push_back({v+level,v+level,true,
                point(c.center.X(),c.center.Y(),level?definition.depth:0),radius*scale,true});
            out.edges.push_back({v,v+1,false,{},0,false});
            out.faces[0].wires.push_back({{e,ring!=0}});out.faces[1].wires.push_back({{e+1,ring==0}});
            auto& side=out.faces[2+ring];side.cylinder=true;side.origin=out.faces[0].origin;
            side.normalOrAxis=vector(0,0,1);side.radius=radius*scale;side.radialSign=ring?-1:1;
            side.wires={{{e,true},{e+2,true},{e+1,false},{e+2,false}}};
            if(ring)for(auto& wire:side.wires){std::reverse(wire.begin(),wire.end());for(auto& use:wire)use.forward=!use.forward;}
        }
        if(definition.plane==1)for(auto& face:out.faces)for(auto& wire:face.wires){std::reverse(wire.begin(),wire.end());for(auto& use:wire)use.forward=!use.forward;}
        for(const auto& p:out.vertices)if(!std::isfinite(p.X())||!std::isfinite(p.Y())||!std::isfinite(p.Z()))return false;
        return true;
    }catch(...){out={};return false;}
}
} // namespace core3d::saved_cut_circular_host
