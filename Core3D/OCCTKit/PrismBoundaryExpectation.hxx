#pragma once
// External prototype: expected boundary only. Does not inspect a retained shape,
// prove correspondence, or grant native edit authority. No production inclusion.
#include "ProfileDefinition.hxx"
#include <gp_Vec.hxx>
#include <array>
#include <cstddef>
#include <map>
#include <utility>
#include <vector>

namespace core3d::saved_cut_prism_prototype {
struct EdgeUse { std::size_t edge=0; bool forward=false; };
struct FaceBoundary {
    // One complete oriented wire; each index names an expected shared vertex.
    // Face0 bottom, face1 top; remaining faces follow the outward side winding.
    std::vector<std::size_t> vertices;
    std::vector<EdgeUse> edges;
};
struct Expectation {
    // Vertex i and i+n retain original recipe ordinal i, independent of winding.
    // Coordinates remain in untransformed recipe space. Extraction must apply
    // inverse validated source frame, never occurrence, to observed geometry.
    std::vector<gp_Pnt> vertices;
    std::vector<std::array<std::size_t,2>> edges;
    std::vector<FaceBoundary> faces;
    gp_Vec extrusion;
};
inline bool BuildExpectedBoundary(const ProfileDefinition& recipe,Expectation& output) noexcept {
    output={};
    try {
        double area=0;
        if(recipe.revolve||recipe.curves||recipe.circle||!recipe.holes.empty()
            ||!ValidateProfileOutline(recipe.points,recipe.plane,recipe.depth,area))return false;
        const std::size_t n=recipe.points.size();Expectation result;
        result.extrusion=recipe.plane==0?gp_Vec(0,0,recipe.depth)
            :recipe.plane==1?gp_Vec(0,recipe.depth,0):gp_Vec(recipe.depth,0,0);
        result.vertices.reserve(2*n);
        for(const auto& point:recipe.points)result.vertices.push_back(ProfilePointInPlane(point,recipe.plane));
        for(std::size_t i=0;i<n;++i)result.vertices.push_back(result.vertices[i].Translated(result.extrusion));
        // XZ has U cross V = -Y, opposite its positive extrusion. Determine
        // outward cap/side winding explicitly instead of assuming XY semantics.
        const bool originalFacesExtrusion=area*(recipe.plane==1?-1.0:1.0)>0;
        std::vector<std::size_t> order;order.reserve(n);
        for(std::size_t i=0;i<n;++i)order.push_back(originalFacesExtrusion?i:n-1-i);
        FaceBoundary bottom,top;
        bottom.vertices.assign(order.rbegin(),order.rend());
        for(auto i:order)top.vertices.push_back(i+n);
        result.faces.push_back(std::move(bottom));result.faces.push_back(std::move(top));
        for(std::size_t i=0;i<n;++i){
            const auto a=order[i],b=order[(i+1)%n];
            FaceBoundary side;side.vertices={a,b,b+n,a+n};result.faces.push_back(std::move(side));
        }
        std::map<std::pair<std::size_t,std::size_t>,std::size_t> shared;
        std::vector<std::array<unsigned,2>> counts;
        for(auto& face:result.faces){
            for(std::size_t i=0;i<face.vertices.size();++i){
                const auto a=face.vertices[i],b=face.vertices[(i+1)%face.vertices.size()];
                if(a==b)return false;
                const auto key=std::make_pair(std::min(a,b),std::max(a,b));
                auto found=shared.find(key);std::size_t index;
                if(found==shared.end()){
                    index=result.edges.size();shared.emplace(key,index);
                    result.edges.push_back({key.first,key.second});counts.push_back({0,0});
                }else index=found->second;
                const bool forward=a==key.first;face.edges.push_back({index,forward});
                ++counts[index][forward?0:1];
            }
        }
        if(result.edges.size()!=3*n||result.faces.size()!=n+2)return false;
        for(const auto& uses:counts)if(uses[0]!=1||uses[1]!=1)return false;
        output=std::move(result);return true;
    }catch(...){return false;}
}
} // namespace core3d::saved_cut_prism_prototype
