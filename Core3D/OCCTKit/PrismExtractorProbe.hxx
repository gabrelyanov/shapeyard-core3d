#pragma once
// Authored detached OCCT regression probe; not registered, compiled or run.
#include "SavedCutPrismExtractor.hxx"
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRep_Builder.hxx>
#include <BRepLib.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Solid.hxx>
#include <string>

namespace core3d::saved_cut_prism_prototype::probe {
inline TopoDS_Shape Build(const ProfileDefinition& recipe,const profile::ConstructionFrame& frame){
    // Actual detached primitive construction independent of the expectation graph.
    BRepBuilderAPI_MakePolygon polygon;
    for(const auto& p:recipe.points)polygon.Add(ProfilePointInPlane(p,recipe.plane));polygon.Close();
    if(!polygon.IsDone())return {};
    BRepBuilderAPI_MakeFace face(polygon.Wire(),Standard_True);if(!face.IsDone())return {};
    const auto direction=recipe.plane==0?gp_Vec(0,0,recipe.depth)
        :recipe.plane==1?gp_Vec(0,recipe.depth,0):gp_Vec(recipe.depth,0,0);
    BRepPrimAPI_MakePrism prism(face.Face(),direction,Standard_True,Standard_True);if(!prism.IsDone())return {};
    gp_Trsf transform;if(!frame.Transform(transform))return {};
    BRepBuilderAPI_Transform placed(prism.Shape(),transform,Standard_True,Standard_False);
    if(!placed.IsDone()||placed.Shape().ShapeType()!=TopAbs_SOLID)return {};
    auto result=TopoDS::Solid(placed.Shape());if(!BRepLib::OrientClosedSolid(result))return {};return result;
}
inline ProfileDefinition L(double factor,int plane,bool reverse){
    ProfileDefinition p;p.plane=plane;p.depth=8/factor;
    for(const auto& uv:std::vector<std::array<double,2>>{{0,0},{60,0},{60,8},{8,8},{8,50},{0,50}})
        p.points.emplace_back(uv[0]/factor,uv[1]/factor);
    if(reverse)std::reverse(p.points.begin(),p.points.end());return p;
}
inline std::map<std::string,bool> Run(){
    std::map<std::string,bool> out;std::atomic_bool stop{false};unsigned fixture=0;
    for(double unit:{0.001,1.0})for(int plane=0;plane<3;++plane)for(bool reversed:{false,true})for(double scale:{0.25,1.0,2.0}){
        const double factor=unit*1000;auto recipe=L(factor,plane,reversed);
        profile::ConstructionFrame frame;frame.values={23/factor,-17/factor,11/factor,0,0,std::sin(0.2),std::cos(0.2),scale};
        const auto shape=Build(recipe,frame);Inspection report;
        const auto key=std::string("direct.")+std::to_string(fixture++);
        out[key]=!shape.IsNull()&&InspectPrism(shape,recipe,frame,unit,stop,report)==Classification::MatchedBoundary
            &&report.vertices==12&&report.edges==18&&report.faces==8;
        GProp_GProps mass;if(!shape.IsNull())BRepGProp::VolumeProperties(shape,mass);
        const double volumeMM=mass.Mass()*factor*factor*factor;
        out[key+".independent-volume"]=std::isfinite(volumeMM)&&std::abs(volumeMM-6528*scale*scale*scale)<1e-5;
        stop=true;out[key+".cancelled"]=InspectPrism(shape,recipe,frame,unit,stop,report)==Classification::Cancelled;stop=false;
        auto wrong=recipe;wrong.depth=9/factor;
        out[key+".wrong-depth"]=InspectPrism(shape,wrong,frame,unit,stop,report)==Classification::Refused;
        auto reflected=frame;reflected.values[7]=-scale;
        out[key+".reflected-frame"]=InspectPrism(shape,recipe,reflected,unit,stop,report)==Classification::Refused;
    }
    const auto recipe=L(1,1,false);profile::ConstructionFrame identity;const auto original=Build(recipe,identity);
    auto other=recipe;other.points={{0,0},{60,0},{60,50},{52,50},{52,8},{0,8}};
    const auto opposite=Build(other,identity);Inspection report;
    GProp_GProps a,b;if(!original.IsNull())BRepGProp::VolumeProperties(original,a);if(!opposite.IsNull())BRepGProp::VolumeProperties(opposite,b);
    out["same-volume-wrong-boundary"]=!opposite.IsNull()&&std::abs(a.Mass()-b.Mass())<1e-8
        &&InspectPrism(opposite,recipe,identity,0.001,stop,report)==Classification::Refused;
    if(!original.IsNull()){
        out["reversed-solid"]=InspectPrism(original.Reversed(),recipe,identity,0.001,stop,report)==Classification::Refused;
        BRepBuilderAPI_Copy copy(original,Standard_True,Standard_False);const auto bad=copy.Shape();
        TopExp_Explorer vertices(bad,TopAbs_VERTEX);
        if(vertices.More()){
            BRep_Builder builder;builder.UpdateVertex(TopoDS::Vertex(vertices.Current()),0.002);
            out["over-kernel-cap"]=InspectPrism(bad,recipe,identity,0.001,stop,report)==Classification::Refused;
        }else out["over-kernel-cap"]=false;
    }else {out["reversed-solid"]=false;out["over-kernel-cap"]=false;}
    out["location.identity"]=locations::RawLocation(TopLoc_Location());
    TopLoc_Location chain;
    for(unsigned i=0;i<8;++i){gp_Trsf step;step.SetTranslation(gp_Vec(1,0,0));chain=chain*TopLoc_Location(step);}
    auto translated=identity;translated.values[0]=8;
    out["location.eight-datums"]=locations::RawLocation(chain)
        &&InspectPrism(original.Moved(chain),recipe,translated,0.001,stop,report)==Classification::MatchedBoundary;
    gp_Trsf next;next.SetTranslation(gp_Vec(1,0,0));const TopLoc_Location ninth(next);
    out["location.ninth-refused"]=!locations::RawLocation(chain*ninth);
    out["location.raw-power-two-refused"]=!locations::RawLocation(ninth.Powered(2));
    gp_Trsf far,back;far.SetTranslation(gp_Vec(1e15,0,0));back.SetTranslation(gp_Vec(-1e15,0,0));
    out["location.cancelling-conditioning-refused"]=InspectPrism(original.Moved(TopLoc_Location(far)*TopLoc_Location(back)),
        recipe,identity,0.001,stop,report)==Classification::Refused;
    return out;
}
} // namespace core3d::saved_cut_prism_prototype::probe
