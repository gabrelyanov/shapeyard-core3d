#pragma once
#include "SavedCutBoreResultObservation.hxx" // requires vertex-sharing review2
#include "PrismBoundaryExpectation.hxx"
namespace core3d::saved_cut_whole_result {
struct Expected {
    std::vector<gp_Pnt> vertices;
    std::vector<enclosure_correspondence::ExpectedEdge> edges;
    std::vector<enclosure_correspondence::ExpectedFace> faces;
    std::array<unsigned,2> caps{};
};
// Full original boundary; the result matcher adds ONLY the explicit bore patch.
inline bool ExpectedSource(const retained_solid::Envelope& e,Expected& out){
    out={};if(!retained_solid::Valid(e))return false;
    if(e.sourceFamily==2){enclosure::Parameters p;enclosure_correspondence::ExpectedBoundary x;
        if(!enclosure::Decode(int(e.sourceSchema),e.sourceValues,p)||!enclosure_correspondence::BuildExpectedBoundary(p.definition,x))return false;
        out.vertices.assign(x.vertices.begin(),x.vertices.end());out.edges.assign(x.edges.begin(),x.edges.end());out.faces.assign(x.faces.begin(),x.faces.end());out.caps={16,17};return true;}
    profile::Parameters p;saved_cut_prism_prototype::Expectation x;
    if(e.sourceFamily!=1||!profile::Decode(e.sourceValues,p)||!saved_cut_prism_prototype::BuildExpectedBoundary(p.definition,x))return false;
    gp_Trsf frame;if(p.constructionFrame&&(!p.constructionFrame->IsValid()||p.constructionFrame->values[7]<=0||!p.constructionFrame->Transform(frame)))return false;
    for(const auto& v:x.vertices)out.vertices.push_back(v.Transformed(frame));
    for(const auto& edge:x.edges){enclosure_correspondence::ExpectedEdge r;r.start=unsigned(edge[0]);r.end=unsigned(edge[1]);out.edges.push_back(r);}
    const gp_Vec w=p.definition.plane==0?gp_Vec(0,0,1):p.definition.plane==1?gp_Vec(0,1,0):gp_Vec(1,0,0);
    for(unsigned faceID=0;faceID<x.faces.size();++faceID){const auto& face=x.faces[faceID];enclosure_correspondence::ExpectedFace r;
        if(face.vertices.size()<3)return false;r.origin=out.vertices[face.vertices[0]];
        // Expected cap direction is independent of collinear polygon triples.
        // Side direction uses the already outward expected lower edge and unit
        // extrusion axis; do not multiply by depth or accumulate polygon area.
        gp_Vec normal=faceID==0?-w:faceID==1?w:gp_Vec(x.vertices[face.vertices[0]],x.vertices[face.vertices[1]]).Crossed(w);
        normal.Transform(frame);
        if(!enclosure_correspondence::detail::Finite(normal)||normal.SquareMagnitude()<=0)return false;r.normalOrAxis=normal;
        std::vector<enclosure_correspondence::EdgeUse> wire;for(const auto& use:face.edges)wire.push_back({unsigned(use.edge),use.forward});r.wires.push_back(std::move(wire));out.faces.push_back(std::move(r));
    }out.caps={0,1};return true;
}
}
