#pragma once
#if DEBUG
#include "OcctDocument.h"
#include "SavedCutSourceEdit.hxx"
#include <Precision.hxx>
#include "SavedCutWholeResultCorrespondence.hxx"
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <map>

namespace core3d::saved_cut_source_changed_probe {
// Independent exact-fixture measurements, complementary to full correspondence.
// Coordinates below are source recipe millimetres. Plane maps recipe (u,v,w)
// into object coordinates; persisted F is applied once, O is deliberately absent.
inline gp_Pnt RecipePoint(double u,double v,double w,double mm,int plane,const gp_Trsf& frame){
    gp_Pnt p=plane==0?gp_Pnt(u/mm,v/mm,w/mm):plane==1?gp_Pnt(u/mm,w/mm,v/mm):gp_Pnt(w/mm,u/mm,v/mm);
    p.Transform(frame);return p;
}
inline std::map<std::string,bool> Enclosure(const OcctCylindricalCutSource& source,double widthMM) {
    std::map<std::string,bool> out;
    enclosure::Parameters p;
    if(!source.rebuilding||source.envelope.sourceFamily!=2
        ||!enclosure::Decode(int(source.envelope.sourceSchema),source.envelope.sourceValues,p))return {{"enclosure-fixture",false}};
    gp_Trsf frame;
    if(p.definition.constructionFrame&&(!p.definition.constructionFrame->Transform(frame)||frame.ScaleFactor()<=0))
        return {{"enclosure-fixture",false}};
    const double mm=source.envelope.metersPerUnit*1000;
    if(!std::isfinite(mm)||mm<=0)return {{"enclosure-fixture",false}};
    const auto& d=p.definition.dimensions;
    const auto near=[](double a,double b){return std::isfinite(a)&&std::abs(a-b)<=1e-8;};
    const double frameScale=frame.ScaleFactor();
    const int axis=p.definition.plane==0?2:p.definition.plane==1?1:0;
    const gp_Pnt bore=RecipePoint(50,30,0,mm,p.definition.plane,frame);
    const gp_Pnt end=RecipePoint(50,30,1,mm,p.definition.plane,frame);
    bool fixedAxis=source.envelope.axis==axis;
    for(int i=0;i<3;++i)if(i!=axis)fixedAxis=fixedAxis&&near(source.envelope.point[i]*mm,bore.Coord(i+1)*mm)
        &&near(end.Coord(i+1)*mm,bore.Coord(i+1)*mm);
    out["enclosure-fixture"]=std::isfinite(mm)&&mm>0&&near(d.width*mm,widthMM)
        &&near(d.depth*mm,60)&&near(d.height*mm,30)&&near(d.wall*mm,2)
        &&near(d.floor*mm,2)&&near(d.cornerRadius*mm,4)
        &&fixedAxis&&near(source.envelope.radius*mm,3*frameScale);
    if(!out["enclosure-fixture"])return out;
    const std::atomic_bool stop(false);
    out["independent-source-boundary"]=saved_cut_source_edit::InspectBase(source.base,source.envelope,stop);
    out["complete-result-boundary"]=saved_cut_whole_result::Inspect(source.original.shape,source.envelope,source.envelope,stop).classification
        ==saved_cut_whole_result::Classification::MatchedOrientedBoundary;
    const double pi=std::acos(-1.0);
    // Independent integrated rounded outer/cavity volumes and 2mm floor bore.
    const double scale3=frameScale*frameScale*frameScale;
    const double expectedBase=((widthMM*60-(4-pi)*16)*30-((widthMM-4)*56-(4-pi)*4)*28)*scale3;
    GProp_GProps base,result;BRepGProp::VolumeProperties(source.base,base);BRepGProp::VolumeProperties(source.original.shape,result);
    out["analytic-base-volume"]=std::abs(base.Mass()*mm*mm*mm-expectedBase)<=std::max(1e-5,expectedBase*1e-8);
    out["analytic-cut-volume"]=std::abs(result.Mass()*mm*mm*mm-(expectedBase-18*pi*scale3))<=std::max(1e-5,expectedBase*1e-8);
    BRepClass3d_SolidClassifier classifier(source.original.shape);
    const double tolerance=std::max(Precision::Confusion(),1e-5/mm);
    const auto classify=[&](const char* name,double x,double y,double z,TopAbs_State expected){
        classifier.Perform(RecipePoint(x,y,z,mm,p.definition.plane,frame),tolerance);out[name]=classifier.State()==expected;
    };
    classify("left-wall-material",1,30,15,TopAbs_IN);classify("left-wall-inner-void",3,30,15,TopAbs_OUT);
    classify("right-wall-material",widthMM-1,30,15,TopAbs_IN);classify("right-wall-inner-void",widthMM-3,30,15,TopAbs_OUT);
    classify("right-exterior-void",widthMM+1,30,15,TopAbs_OUT);
    classify("front-wall-material",25,1,15,TopAbs_IN);classify("front-wall-inner-void",25,3,15,TopAbs_OUT);
    classify("back-wall-material",25,59,15,TopAbs_IN);classify("back-wall-inner-void",25,57,15,TopAbs_OUT);
    classify("floor-material",20,20,1,TopAbs_IN);classify("above-floor-void",20,20,3,TopAbs_OUT);
    classify("below-floor-void",20,20,-1,TopAbs_OUT);classify("bore-interior-void",50,30,1,TopAbs_OUT);
    for(int i=0;i<8;++i){
        const double a=2*pi*i/8;
        const std::string inside="bore-inner-"+std::to_string(i),outside="bore-outer-"+std::to_string(i);
        classify(inside.c_str(),50+2.75*std::cos(a),30+2.75*std::sin(a),1,TopAbs_OUT);
        classify(outside.c_str(),50+3.25*std::cos(a),30+3.25*std::sin(a),1,TopAbs_IN);
    }
    return out;
}
inline std::map<std::string,bool> Circle(const OcctCylindricalCutSource& source,double outerMM,double innerMM,double depthMM) {
    profile::Parameters p;std::map<std::string,bool> out;
    if(!source.rebuilding||source.envelope.sourceFamily!=1||!profile::Decode(source.envelope.sourceValues,p)
        ||!p.definition.circle)return {{"circle-fixture",false}};
    const auto& d=p.definition;const auto& circle=*d.circle;const double mm=p.metersPerUnit*1000;
    const auto near=[](double a,double b){return std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=1e-8;};
    out["circle-fixture"]=!d.revolve&&!d.curves&&d.points.empty()&&d.holes.empty()
        &&near(circle.center.X()*mm,0)&&near(circle.center.Y()*mm,0)
        &&near(circle.outerRadius*mm,outerMM)&&near(circle.innerRadius*mm,innerMM)&&near(d.depth*mm,depthMM);
    if(!out["circle-fixture"])return out;
    const std::atomic_bool stop(false);
    out["independent-source-boundary"]=saved_cut_source_edit::InspectBase(source.base,source.envelope,stop);
    const auto proof=saved_cut_whole_result::Inspect(source.original.shape,source.envelope,source.envelope,stop);
    out["complete-result-boundary"]=proof.classification==saved_cut_whole_result::Classification::MatchedOrientedBoundary;
    out["all-vertex-links"]=proof.vertexLinks==proof.vertices&&proof.vertices==(innerMM>0?6u:4u);
    gp_Trsf frame;if(p.constructionFrame&&!p.constructionFrame->Transform(frame))return {{"circle-frame",false}};
    const double scale=frame.ScaleFactor(),pi=std::acos(-1.0);
    const double expectedBase=pi*(outerMM*outerMM-innerMM*innerMM)*depthMM*scale*scale*scale;
    const double bore=source.envelope.radius*mm,expectedCut=expectedBase-pi*bore*bore*depthMM*scale;
    GProp_GProps base,result;BRepGProp::VolumeProperties(source.base,base);BRepGProp::VolumeProperties(source.original.shape,result);
    out["analytic-base-volume"]=std::abs(base.Mass()*mm*mm*mm-expectedBase)<=expectedBase*1e-6;
    out["analytic-cut-volume"]=std::abs(result.Mass()*mm*mm*mm-expectedCut)<=expectedCut*1e-6;
    return out;
}
inline std::map<std::string,bool> Bracket(const OcctCylindricalCutSource& source,double lengthMM,double depthMM) {
    std::map<std::string,bool> out;profile::Parameters p;
    if(!source.rebuilding||source.envelope.sourceFamily!=1
        ||!profile::Decode(source.envelope.sourceValues,p)||profile::SchemaFor(p)!=int(source.envelope.sourceSchema))return {{"bracket-fixture",false}};
    const double mm=source.envelope.metersPerUnit*1000;const auto& d=p.definition;
    if(!std::isfinite(mm)||mm<=0)return {{"bracket-fixture",false}};
    gp_Trsf frame;
    if(p.constructionFrame&&(!p.constructionFrame->Transform(frame)||frame.ScaleFactor()<=0))return {{"bracket-fixture",false}};
    const auto near=[](double a,double b){return std::isfinite(a)&&std::abs(a-b)<=1e-8;};
    const std::array<std::array<double,2>,6> points={{{0,0},{lengthMM,0},{lengthMM,8},{8,8},{8,50},{0,50}}};
    bool outline=d.points.size()==points.size()&&!d.curves&&!d.circle&&d.holes.empty()&&!d.revolve;
    if(outline)for(std::size_t i=0;i<points.size();++i)outline=outline&&near(d.points[i].X()*mm,points[i][0])&&near(d.points[i].Y()*mm,points[i][1]);
    const int axis=d.plane==0?2:d.plane==1?1:0;
    const gp_Pnt bore=RecipePoint(30,4,0,mm,d.plane,frame),end=RecipePoint(30,4,1,mm,d.plane,frame);
    bool fixedAxis=source.envelope.axis==axis;
    for(int i=0;i<3;++i)if(i!=axis)fixedAxis=fixedAxis&&near(source.envelope.point[i]*mm,bore.Coord(i+1)*mm)
        &&near(end.Coord(i+1)*mm,bore.Coord(i+1)*mm);
    const double scale=frame.ScaleFactor(),scale3=scale*scale*scale;
    out["bracket-fixture"]=std::isfinite(mm)&&mm>0&&outline&&near(d.depth*mm,depthMM)&&fixedAxis&&near(source.envelope.radius*mm,2*scale);
    if(!out["bracket-fixture"])return out;
    const std::atomic_bool stop(false);
    out["independent-source-boundary"]=saved_cut_source_edit::InspectBase(source.base,source.envelope,stop);
    out["complete-result-boundary"]=saved_cut_whole_result::Inspect(source.original.shape,source.envelope,source.envelope,stop).classification
        ==saved_cut_whole_result::Classification::MatchedOrientedBoundary;
    // Disjoint rectangles: horizontal foot length*8 and vertical arm8*(50-8).
    const double pi=std::acos(-1.0),expectedBase=(lengthMM*8+8*42)*depthMM*scale3;
    GProp_GProps base,result;BRepGProp::VolumeProperties(source.base,base);BRepGProp::VolumeProperties(source.original.shape,result);
    out["analytic-base-volume"]=std::abs(base.Mass()*mm*mm*mm-expectedBase)<=std::max(1e-5,expectedBase*1e-8);
    out["analytic-cut-volume"]=std::abs(result.Mass()*mm*mm*mm-(expectedBase-4*pi*depthMM*scale3))<=std::max(1e-5,expectedBase*1e-8);
    BRepClass3d_SolidClassifier classifier(source.original.shape);const double tolerance=std::max(Precision::Confusion(),1e-5/mm);
    const auto classify=[&](const char* name,double u,double v,double w,TopAbs_State expected){
        classifier.Perform(RecipePoint(u,v,w,mm,d.plane,frame),tolerance);out[name]=classifier.State()==expected;
    };
    classify("foot-end-material",lengthMM-1,4,depthMM/2,TopAbs_IN);classify("foot-end-exterior",lengthMM+1,4,depthMM/2,TopAbs_OUT);
    classify("upright-material",4,49,depthMM/2,TopAbs_IN);classify("notch-void",9,9,depthMM/2,TopAbs_OUT);
    classify("near-cap-material",20,4,0.5,TopAbs_IN);classify("far-cap-material",20,4,depthMM-0.5,TopAbs_IN);
    classify("near-cap-exterior",20,4,-0.5,TopAbs_OUT);classify("far-cap-exterior",20,4,depthMM+0.5,TopAbs_OUT);
    classify("bore-near-opening",30,4,0.25,TopAbs_OUT);classify("bore-far-opening",30,4,depthMM-0.25,TopAbs_OUT);
    for(int i=0;i<8;++i){const double a=2*pi*i/8;
        const std::string inner="bore-inner-"+std::to_string(i),outer="bore-outer-"+std::to_string(i);
        classify(inner.c_str(),30+1.75*std::cos(a),4+1.75*std::sin(a),depthMM/2,TopAbs_OUT);
        classify(outer.c_str(),30+2.25*std::cos(a),4+2.25*std::sin(a),depthMM/2,TopAbs_IN);
    }
    return out;
}

}
#endif
