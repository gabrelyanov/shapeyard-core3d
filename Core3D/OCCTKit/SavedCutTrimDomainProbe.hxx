#pragma once
#if DEBUG
#include "SavedCutWholeResultCorrespondence.hxx"
#include "EnclosureGeometry.hxx"
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <TopExp_Explorer.hxx>
#include <sstream>
#include <map>
#include <stdexcept>
namespace core3d::saved_cut_trim_domain::probe {
namespace d=enclosure_correspondence::detail;
inline std::string Bytes(const TopoDS_Shape& shape){
    if(shape.IsNull())throw std::invalid_argument("trim fixture null");std::ostringstream out;out.imbue(std::locale::classic());
    BRepTools::Write(shape,out,Standard_True,Standard_True,TopTools_FormatVersion_VERSION_3);
    const auto bytes=out.str();if(!out.good()||bytes.empty()||bytes.size()>4U*1024U*1024U)throw std::invalid_argument("trim fixture stream");return bytes;
}
inline TopoDS_Shape Read(const std::string& bytes){
    std::istringstream in(bytes);in.imbue(std::locale::classic());TopoDS_Shape shape;BRep_Builder builder;BRepTools::Read(shape,in,builder);
    if(shape.IsNull()||!in)throw std::invalid_argument("trim fixture read");return shape;
}
// Actual private-BRep edge mutation. Range and support location stay unchanged;
// only the analytic wrapper's domain is narrowed. No output repair or sampling.
inline bool Narrow(const TopoDS_Shape& shape,bool pcurve,bool circle,double mm,bool tiny){
    unsigned faces=0;
    for(TopExp_Explorer fi(shape,TopAbs_FACE);fi.More();fi.Next()){
        if(++faces>130)return false;const auto face=TopoDS::Face(fi.Current());TopLoc_Location sl;const auto surface=BRep_Tool::Surface(face,sl);
        if(pcurve&&Handle(Geom_CylindricalSurface)::DownCast(surface).IsNull())continue;
        unsigned edges=0;for(TopExp_Explorer ei(face,TopAbs_EDGE);ei.More();ei.Next()){
            if(++edges>128)return false;const auto edge=TopoDS::Edge(ei.Current().Oriented(TopAbs_FORWARD));TopLoc_Location cl;double f=0,l=0;
            const auto curve=BRep_Tool::Curve(edge,cl,f,l);if(curve.IsNull()||!d::Range(f,l))continue;
            const bool isCircle=!Handle(Geom_Circle)::DownCast(curve).IsNull();const bool isLine=!Handle(Geom_Line)::DownCast(curve).IsNull();
            if((circle&&!isCircle)||(!circle&&!isLine))continue;
            const double delta=circle?(tiny?1e-13:.01):(tiny?1e-12:.01)/mm;
            const double lower=f+delta,upper=l-delta;if(!(f<lower&&lower<upper&&upper<l))return false;
            const auto data=Handle(BRep_TEdge)::DownCast(edge.TShape());if(data.IsNull())return false;
            if(!pcurve){
                const Handle(Geom_Curve) wrapped=new Geom_TrimmedCurve(curve,lower,upper,Standard_True,Standard_False);
                for(BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());it.More();it.Next())if(it.Value()->IsCurve3D()){
                    const auto gc=Handle(BRep_GCurve)::DownCast(it.Value());double first=0,last=0;if(gc.IsNull())return false;gc->Range(first,last);
                    if(first!=f||last!=l)return false;it.Value()->Curve3D(wrapped);gc->Range(first,last);
                    return it.Value()->Curve3D()==wrapped&&first==f&&last==l;
                }return false;
            }
            const auto relative=sl.Predivided(edge.Location());
            Handle(Geom2d_Curve) basis;Handle(BRep_CurveRepresentation) stored;
            for(BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());it.More();it.Next())
                if(it.Value()->IsCurveOnSurface(surface,relative)&&!it.Value()->IsCurveOnClosedSurface()){stored=it.Value();basis=stored->PCurve();break;}
            if(basis.IsNull())continue;
            for(unsigned n=0;n<d::MaximumWrappers;++n){const auto trim=Handle(Geom2d_TrimmedCurve)::DownCast(basis);if(trim.IsNull())break;basis=trim->BasisCurve();}
            if(Handle(Geom2d_Line)::DownCast(basis).IsNull())return false;
            const Handle(Geom2d_Curve) wrapped=new Geom2d_TrimmedCurve(basis,lower,upper,Standard_True,Standard_False);
            const auto gc=Handle(BRep_GCurve)::DownCast(stored);double first=0,last=0;if(gc.IsNull())return false;gc->Range(first,last);
            if(first!=f||last!=l)return false;stored->PCurve(wrapped);gc->Range(first,last);
            return stored->PCurve()==wrapped&&first==f&&last==l;
        }
    }return false;
}
inline std::map<std::string,bool> Run(){
    std::map<std::string,bool> out;try {
        const double error=1e-9;double charge=0,value=0;
        const auto add=[&](const char* key,bool result){out[std::string("arithmetic.")+key]=result;};
        d::Curve line;line.first=0;line.last=1;line.a=gp_Vec(1,0,0);
        d::Surface plane;plane.x=gp_Vec(1,0,0);plane.y=gp_Vec(0,1,0);plane.z=gp_Vec(0,0,1);
        d::PCurve pc;pc.first=0;pc.last=1;pc.a=gp_Vec2d(1,0);
        add("unchanged-untrimmed-exact",d::TrimResidualWithin(line,pc,plane,1,0,error));
        add("tiny-line-wrapper-admitted",d::RetainTrim(line.trims,0,1,1e-12,1-1e-12)&&d::TrimResidualWithin(line,pc,plane,1,0,error));
        add("charge-is-positive-and-outward",d::CurveTrimCharge(line,1,charge)&&charge>1e-12&&charge<error);
        add("coefficient-plus-domain-over-budget-refuses",!d::TrimResidualWithin(line,pc,plane,1,error-charge/2,error));
        auto both=pc;add("both-domain-charges-share-budget",d::RetainTrim(both.trims,0,1,1e-12,1-1e-12)&&!d::TrimResidualWithin(line,both,plane,1,error-charge*1.5,error));
        auto clipped=line;clipped.trims={};add("material-line-clip-refuses",d::RetainTrim(clipped.trims,0,1,.01,.99)&&!d::TrimResidualWithin(clipped,pc,plane,1,0,error));
        d::TrimDomains common;add("first-partial-domain-valid",d::RetainTrim(common,0,1,0,.5));
        add("point-only-common-domain-refuses",!d::RetainTrim(common,0,1,.5,1));
        add("disjoint-common-domain-refuses",!d::RetainTrim(common,0,1,.6,1));
        d::TrimDomains invalid;add("nan-domain-refuses",!d::RetainTrim(invalid,0,1,std::numeric_limits<double>::quiet_NaN(),1));
        add("infinite-domain-refuses",!d::RetainTrim(invalid,0,1,0,std::numeric_limits<double>::infinity()));
        add("empty-domain-refuses",!d::RetainTrim(invalid,0,1,.5,.5));
        d::TrimDomains nested;bool eight=true;for(unsigned n=0;n<d::MaximumWrappers;++n)eight=eight&&d::RetainTrim(nested,0,1,0,1);
        add("eight-domain-storage-bound",eight&&nested.size()==8);add("ninth-domain-refuses",!d::RetainTrim(nested,0,1,0,1));
        add("positive-add-overflow-refuses",!d::TrimUpperAdd(std::numeric_limits<double>::max(),std::numeric_limits<double>::max(),value));
        add("positive-product-overflow-refuses",!d::TrimUpperMultiply(std::numeric_limits<double>::max(),2,value));
        add("nan-residual-refuses",!d::TrimResidualWithin(line,pc,plane,1,std::numeric_limits<double>::quiet_NaN(),error));
        add("infinite-scale-refuses",!d::CurveTrimCharge(line,std::numeric_limits<double>::infinity(),charge));
        d::Curve circle;circle.circle=true;circle.first=0;circle.last=2*std::acos(-1.0);circle.a=gp_Vec(5,0,0);circle.b=gp_Vec(0,5,0);
        d::PCurve full;full.circle=true;full.first=circle.first;full.last=circle.last;full.a=gp_Vec2d(5,0);full.b=gp_Vec2d(0,5);
        add("full-planar-circle-tiny-trim-admitted",d::RetainTrim(full.trims,full.first,full.last,1e-13,full.last-1e-13)
            &&saved_cut_whole_result::detail::FullPlanarCircleIdentity(circle,full,plane,1,error));
        full.trims={};add("full-planar-circle-clipped-refuses",d::RetainTrim(full.trims,full.first,full.last,.01,full.last-.01)
            &&!saved_cut_whole_result::detail::FullPlanarCircleIdentity(circle,full,plane,1,error));
        d::Surface cylinder;cylinder.cylinder=true;cylinder.x=gp_Vec(5,0,0);cylinder.y=gp_Vec(0,5,0);cylinder.z=gp_Vec(0,0,1);
        d::PCurve cylindrical;cylindrical.first=circle.first;cylindrical.last=circle.last;cylindrical.a=gp_Vec2d(1,0);
        add("cylinder-angular-tiny-trim-admitted",d::RetainTrim(cylindrical.trims,circle.first,circle.last,1e-13,circle.last-1e-13)&&d::PCurveIdentity(circle,cylindrical,cylinder,1,error));
        cylindrical.trims={};add("cylinder-angular-clipped-refuses",d::RetainTrim(cylindrical.trims,circle.first,circle.last,.01,circle.last-.01)&&!d::PCurveIdentity(circle,cylindrical,cylinder,1,error));
        d::TrimDomains overflow;add("finite-overrun-subtraction-overflow-refuses",d::RetainTrim(overflow,-std::numeric_limits<double>::max(),std::numeric_limits<double>::max(),std::numeric_limits<double>::max()/2,std::numeric_limits<double>::max())&&!d::TrimSpan(overflow,-std::numeric_limits<double>::max(),std::numeric_limits<double>::max(),charge));
        // Actual generated and separately read shapes. All mutation inputs are
        // private reads; original generated and archived streams remain exact.
        for(double unit:{.001,1.0})for(int planeIndex=0;planeIndex<3;++planeIndex){
            const double mm=unit*1000,k=1/mm;enclosure::Parameters parameters;parameters.metersPerUnit=unit;parameters.definition.plane=planeIndex;
            parameters.definition.dimensions={100*k,60*k,30*k,2*k,2*k,4*k};profile::ConstructionFrame frame;const double a=std::acos(-1.0)/12;
            frame.values={13*k,-7*k,5*k,0,0,std::sin(a),std::cos(a),1.25};parameters.definition.constructionFrame=frame;
            auto stop=std::make_shared<std::atomic_bool>(false);EnclosureSolidResult generated;
            const std::string prefix=std::string(unit==.001?"mm":"metre")+".plane"+std::to_string(planeIndex)+".";
            const bool built=BuildEnclosureSolidGeometry(parameters.definition,stop,generated);out[prefix+"actual-generated"]=built;
            if(!built)continue;const auto original=Bytes(generated.solid);const auto reopened=Read(original);const auto fresh=Bytes(reopened);enclosure_correspondence::Inspection observed;
            out[prefix+"actual-ascii-reopen-matches"]=enclosure_correspondence::InspectEnclosure(reopened,parameters,*stop,observed)==enclosure_correspondence::Classification::MatchedBoundary;
            out[prefix+"reopened-inspection-preserves-stream"]=Bytes(reopened)==fresh;
            for(bool pcurve:{false,true})for(bool arc:{false,true})for(bool tiny:{true,false}){
                const auto input=Read(original);const auto untouched=Bytes(input);const bool changed=Narrow(input,pcurve,arc,mm,tiny);const auto before=Bytes(input);enclosure_correspondence::Inspection result;
                const auto status=enclosure_correspondence::InspectEnclosure(input,parameters,*stop,result);
                const auto name=std::string(pcurve?"cylinder-pcurve-":"curve3d-")+(arc?"arc-":"line-")+(tiny?"tiny-admitted":"clipped-refused");
                out[prefix+name]=changed&&before!=untouched&&status==(tiny?enclosure_correspondence::Classification::MatchedBoundary:enclosure_correspondence::Classification::Refused)
                    &&Bytes(input)==before&&Bytes(generated.solid)==original;
            }
            out[prefix+"original-stream-preserved"]=Bytes(generated.solid)==original;
        }
    }catch(...){out["exception"]=false;}return out;
}
}
#endif
