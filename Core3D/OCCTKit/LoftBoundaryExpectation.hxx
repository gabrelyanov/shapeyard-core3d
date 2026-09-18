#pragma once
#include "RectangularLoftDefinition.hxx"
#include "EnclosureBoundaryExpectation.hxx"
#include "PrismBoundaryExpectation.hxx"
#include <map>

namespace core3d::saved_cut_loft {
// Authored XY station corners are CCW. The source frame is a positive
// similarity, as for the other cut families; occurrence transforms are absent.
struct Expectation {
    std::vector<gp_Pnt> vertices;
    std::vector<enclosure_correspondence::ExpectedEdge> edges;
    std::vector<enclosure_correspondence::ExpectedFace> faces;
    std::array<unsigned,2> caps{0,1};
    saved_cut_prism_prototype::Expectation polyhedron;
};
inline bool BuildExpectedBoundary(const rectangular_loft::Definition& recipe,Expectation& out) noexcept {
    out={};try {
        rectangular_loft::Inspection checked;
        if(rectangular_loft::Inspect(recipe,checked)!=rectangular_loft::Admission::Accepted)return false;
        gp_Trsf frame;
        if(recipe.constructionFrame&&(!recipe.constructionFrame->IsValid()
            ||recipe.constructionFrame->values[7]<=0||!recipe.constructionFrame->Transform(frame)))return false;
        Expectation x;auto& p=x.polyhedron;const unsigned n=unsigned(recipe.stations.size());
        for(const auto& s:recipe.stations)for(const auto& v:rectangular_loft::detail::Corners(s))p.vertices.push_back(v);
        const auto face=[&](std::vector<std::size_t> v){saved_cut_prism_prototype::FaceBoundary f;f.vertices=std::move(v);p.faces.push_back(std::move(f));};
        face({3,2,1,0});const std::size_t top=4*(n-1);face({top,top+1,top+2,top+3});
        for(unsigned s=0;s+1<n;++s)for(unsigned c=0;c<4;++c){
            const std::size_t a=4*s+c,b=4*s+(c+1)%4;face({a,b,b+4,a+4});}
        std::map<std::pair<std::size_t,std::size_t>,std::size_t> shared;
        std::vector<std::array<unsigned,2>> counts;
        for(auto& f:p.faces)for(std::size_t i=0;i<f.vertices.size();++i){
            const auto a=f.vertices[i],b=f.vertices[(i+1)%f.vertices.size()];const auto key=std::minmax(a,b);
            auto inserted=shared.emplace(key,p.edges.size());const auto index=inserted.first->second;
            if(inserted.second){p.edges.push_back({key.first,key.second});counts.push_back({0,0});}
            const bool forward=a==key.first;f.edges.push_back({index,forward});++counts[index][forward?0:1];}
        if(p.vertices.size()!=4*n||p.edges.size()!=4*n+4*(n-1)||p.faces.size()!=4*(n-1)+2)return false;
        for(const auto& uses:counts)if(uses[0]!=1||uses[1]!=1)return false;
        for(const auto& v:p.vertices)x.vertices.push_back(v.Transformed(frame));
        for(const auto& e:p.edges){enclosure_correspondence::ExpectedEdge edge;edge.start=unsigned(e[0]);edge.end=unsigned(e[1]);x.edges.push_back(edge);}
        for(const auto& f:p.faces){enclosure_correspondence::ExpectedFace target;target.origin=x.vertices[f.vertices[0]];
            gp_Vec normal=gp_Vec(p.vertices[f.vertices[0]],p.vertices[f.vertices[1]]).Crossed(gp_Vec(p.vertices[f.vertices[0]],p.vertices[f.vertices[2]]));
            normal.Transform(frame);if(!std::isfinite(normal.SquareMagnitude())||normal.SquareMagnitude()<=0)return false;
            target.normalOrAxis=normal;std::vector<enclosure_correspondence::EdgeUse> wire;
            for(const auto& use:f.edges)wire.push_back({unsigned(use.edge),use.forward});target.wires.push_back(std::move(wire));x.faces.push_back(std::move(target));}
        out=std::move(x);return true;
    }catch(...){return false;}
}
}
