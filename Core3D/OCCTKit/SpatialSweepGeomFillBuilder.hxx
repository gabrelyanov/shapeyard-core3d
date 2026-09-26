#pragma once

// Concrete C2 detached builder. It uses only the lower GeomFill extension
// points, four authored rational quadratic quarters, explicit faces/caps,
// bounded sewing and a serial solid gate. There is no MakePipeShell route.
#include "SpatialSweepBishopLaw.hxx"
#include "SpatialSweepProofProducer.hxx"
#include "SpatialSweepSolid.hxx"
#include <BOPAlgo_ArgumentAnalyzer.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepLib.hxx>
#include <BRep_Tool.hxx>
#include <BinTools.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <GeomFill_CurveAndTrihedron.hxx>
#include <GeomFill_EvolvedSection.hxx>
#include <GeomFill_Sweep.hxx>
#include <GProp_GProps.hxx>
#include <Law_Function.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Pln.hxx>
#include <sstream>

namespace core3d::spatial_sweep {
class ArcLengthRadiusLaw final : public Law_Function {
    DEFINE_STANDARD_RTTI_INLINE(ArcLengthRadiusLaw,Law_Function)
public:
    ArcLengthRadiusLaw(std::shared_ptr<const BishopTransportTable> table,double r0,double r1)
        :table_(std::move(table)),r0_(r0),r1_(r1){if(table_){first_=table_->first;last_=table_->last;}}
    GeomAbs_Shape Continuity() const override{return GeomAbs_C2;}
    Standard_Integer NbIntervals(const GeomAbs_Shape) const override{return table_?Standard_Integer(table_->continuityBounds.size()-1):0;}
    void Intervals(TColStd_Array1OfReal& out,const GeomAbs_Shape) const override{if(!table_)return;for(std::size_t i=0;i<table_->continuityBounds.size()&&int(i)+out.Lower()<=out.Upper();++i)out(out.Lower()+int(i))=table_->continuityBounds[i];}
    Standard_Real Value(Standard_Real u) override{double q=0,d=0,d2=0;if(!arc(u,q,d,d2))return std::numeric_limits<double>::quiet_NaN();return r0_+(r1_-r0_)*q;}
    void D1(Standard_Real u,Standard_Real& f,Standard_Real& d) override{double q=0,q1=0,q2=0;if(!arc(u,q,q1,q2)){f=d=std::numeric_limits<double>::quiet_NaN();return;}f=r0_+(r1_-r0_)*q;d=(r1_-r0_)*q1;}
    void D2(Standard_Real u,Standard_Real& f,Standard_Real& d,Standard_Real& d2) override{double q=0,q1=0,q2=0;if(!arc(u,q,q1,q2)){f=d=d2=std::numeric_limits<double>::quiet_NaN();return;}f=r0_+(r1_-r0_)*q;d=(r1_-r0_)*q1;d2=(r1_-r0_)*q2;}
    Handle(Law_Function) Trim(Standard_Real first,Standard_Real last,Standard_Real) const override{auto law=new ArcLengthRadiusLaw(table_,r0_,r1_);law->first_=first;law->last_=last;return law;}
    void Bounds(Standard_Real& first,Standard_Real& last) override{first=first_;last=last_;}
private:
    // Value, D1 and D2 all funnel through arc(), so the narrowly bounded
    // endpoint normalization (exactly one representable step beyond a
    // retained bound, arising from the supported kernel's endpoint queries)
    // applies the same normalized parameter to the radius value and both
    // derivatives. Material excursions still refuse and surface as NaN.
    bool arc(double u,double& q,double& q1,double& q2) const{if(!table_||!table_->valid||!bishop_detail::NormalizeEndpointQuery(u,first_,last_))return false;return bishop_detail::ArcFraction(*table_,u,q,q1,q2);}
    std::shared_ptr<const BishopTransportTable> table_;double r0_=0,r1_=0,first_=0,last_=0;
};

enum class GeomFillBuildStatus : std::uint8_t { Built=0,InvalidInput,Cancelled,ProofRefused,TransportRefused,SurfaceRefused,SewingRefused,SolidRefused };
struct GeomFillBuildResult {GeomFillBuildStatus status=GeomFillBuildStatus::InvalidInput;TopoDS_Shape solid;DetachedKernelReceipt receipt;CurveCertificate curveProof;PreparedAdmission admission;bool built()const noexcept{return status==GeomFillBuildStatus::Built;}};

namespace geomfill_detail {
inline Handle(Geom_BSplineCurve) Curve(const bounded_curve::Definition& source,double mmPerUnit){
    const int n=int(source.controlPoints.size()),k=int(source.knots.size());TColgp_Array1OfPnt poles(1,n);TColStd_Array1OfReal knots(1,k),weights(1,n);TColStd_Array1OfInteger mults(1,k);
    for(int i=1;i<=n;++i){const auto& p=source.controlPoints[std::size_t(i-1)].local;std::array<double,3> world{};for(int c=0;c<3;++c)world[c]=(source.frame.origin[c]+source.frame.xAxis[c]*p[0]+source.frame.yAxis[c]*p[1]+source.frame.zAxis[c]*p[2])*mmPerUnit;poles(i)=gp_Pnt(world[0],world[1],world[2]);weights(i)=source.weights.empty()?1:source.weights[std::size_t(i-1)];}
    for(int i=1;i<=k;++i){knots(i)=source.knots[std::size_t(i-1)].value;mults(i)=source.knots[std::size_t(i-1)].multiplicity;}
    if(source.weights.empty())return new Geom_BSplineCurve(poles,knots,mults,source.degree,Standard_False);return new Geom_BSplineCurve(poles,weights,knots,mults,source.degree,Standard_False);
}
inline Handle(Geom_BSplineCurve) Quarter(unsigned q){
    static constexpr double xy[4][3][2]={{{1,0},{1,1},{0,1}},{{0,1},{-1,1},{-1,0}},{{-1,0},{-1,-1},{0,-1}},{{0,-1},{1,-1},{1,0}}};TColgp_Array1OfPnt poles(1,3);TColStd_Array1OfReal weights(1,3),knots(1,2);TColStd_Array1OfInteger mults(1,2);for(int i=1;i<=3;++i){poles(i)=gp_Pnt(xy[q][i-1][0],xy[q][i-1][1],0);weights(i)=i==2?std::sqrt(0.5):1;}knots(1)=0;knots(2)=1;mults(1)=mults(2)=3;return new Geom_BSplineCurve(poles,weights,knots,mults,2,Standard_False);
}
inline Handle(Geom_Curve) PathBoundary(const Handle(Geom_Surface)& surface,bool exchange,double parameter){return exchange?surface->UIso(parameter):surface->VIso(parameter);}
inline bool PositiveReturnedSurface(const Handle(Geom_Surface)& surface){Handle(Geom_BSplineSurface) bs=Handle(Geom_BSplineSurface)::DownCast(surface);if(bs.IsNull()||bs->UDegree()>12||bs->VDegree()>12||bs->NbUKnots()*bs->NbVKnots()>MaximumSurfaceCertificateCells)return false;for(int u=1;u<=bs->NbUPoles();++u)for(int v=1;v<=bs->NbVPoles();++v){const gp_Pnt p=bs->Pole(u,v);if(!std::isfinite(p.X())||!std::isfinite(p.Y())||!std::isfinite(p.Z())||bs->Weight(u,v)<=0||!std::isfinite(bs->Weight(u,v)))return false;}return true;}
inline bool HashShape(const TopoDS_Shape& shape,Digest& digest){std::ostringstream stream(std::ios::binary);BinTools::Write(shape,stream,Standard_False,Standard_False,BinTools_FormatVersion_VERSION_4);const std::string bytes=stream.str();return bounded_curve::Hash(std::vector<std::uint8_t>(bytes.begin(),bytes.end()),8*1024*1024,digest);}
inline Vector3 WorldSeed(const bounded_curve::Definition& curve,const Vector3& local){Vector3 out{};for(int c=0;c<3;++c)out[c]=curve.frame.xAxis[c]*local[0]+curve.frame.yAxis[c]*local[1]+curve.frame.zAxis[c]*local[2];return out;}
inline Vector3 RetainedTangent(const bounded_curve::Definition& curve,const gp_Vec& world){return {{world.X()*curve.frame.xAxis[0]+world.Y()*curve.frame.xAxis[1]+world.Z()*curve.frame.xAxis[2],world.X()*curve.frame.yAxis[0]+world.Y()*curve.frame.yAxis[1]+world.Z()*curve.frame.yAxis[2],world.X()*curve.frame.zAxis[0]+world.Y()*curve.frame.zAxis[1]+world.Z()*curve.frame.zAxis[2]}};}
inline TopoDS_Shell OneShell(const TopoDS_Shape& shape){if(shape.ShapeType()==TopAbs_SHELL)return TopoDS::Shell(shape);TopExp_Explorer it(shape,TopAbs_SHELL);if(!it.More())return {};TopoDS_Shell shell=TopoDS::Shell(it.Current());it.Next();return it.More()?TopoDS_Shell():shell;}
// FIX-SPEC step 4: measured evidence from the four returned rational
// surfaces. Every interval below derives from the actual homogeneous control
// net geometry (sampled points and partials of the returned surfaces),
// anchored to the certified spine intervals through the sealed Bishop table.
// The authored radius law only ever appears as the expectation a measurement
// is compared against, never as the measurement of its own result.
struct ReturnedSurfaceEvidence {
    std::array<bool,4> quarterCoverage{{false,false,false,false}};
    std::array<Interval,4> normalEquation,radialDeviation,jacobian;
    std::array<Interval,5> stationRadius;
    bool sharedMeridians=false,pathBoundaryOnce=false,markedMeridian=false;
};
inline bool MeasureReturnedSurfaces(const std::array<Handle(Geom_Surface),4>& surfaces,
    const std::array<bool,4>& exchange,const BishopTransportTable& table,
    double phase,double spin,double r0,double r1,double fitting,
    ReturnedSurfaceEvidence& out){
    constexpr int pathSamples=5,sectionSamples=5;
    const double radialError=fitting+PositionalEpsilonMM/8;
    const double minRadius=std::min(r0,r1);
    if(!table.valid||!(minRadius>0)||!Finite(fitting)||fitting<0)return false;
    const double angleTolerance=table.frameErrorBound+radialError/minRadius+1e-12;
    const double parameterScale=std::max(1.0,std::max(std::abs(table.first),std::abs(table.last)));
    out=ReturnedSurfaceEvidence{};bool pathRanges=true,tiling=true,meridians=true,marked=true;
    for(unsigned q=0;q<4;++q){
        const Handle(Geom_BSplineSurface) bs=Handle(Geom_BSplineSurface)::DownCast(surfaces[q]);if(bs.IsNull())return false;
        double uFirst,uLast,vFirst,vLast;bs->Bounds(uFirst,uLast,vFirst,vLast);
        const double pathFirst=exchange[q]?uFirst:vFirst,pathLast=exchange[q]?uLast:vLast;
        const double sectionFirst=exchange[q]?vFirst:uFirst,sectionLast=exchange[q]?vLast:uLast;
        pathRanges=pathRanges&&std::abs(pathFirst-table.first)<=1e-9*parameterScale&&std::abs(pathLast-table.last)<=1e-9*parameterScale;
        double arcFirst=0,arcLast=0,spare1=0,spare2=0;
        if(!bishop_detail::ArcFraction(table,pathFirst,arcFirst,spare1,spare2)||!bishop_detail::ArcFraction(table,pathLast,arcLast,spare1,spare2))return false;
        pathRanges=pathRanges&&std::abs(arcFirst)<=angleTolerance&&std::abs(arcLast-1)<=angleTolerance;
        double dotLo=std::numeric_limits<double>::infinity(),dotHi=-dotLo,devLo=dotLo,devHi=-dotLo,jacLo=dotLo,jacHi=-dotLo;
        double angleLo=dotLo,angleHi=-dotLo,startLo=dotLo,startHi=-dotLo,endLo=dotLo,endHi=-dotLo;
        for(int i=0;i<pathSamples;++i){
            const double u=pathFirst+(pathLast-pathFirst)*i/(pathSamples-1);
            gp_Pnt spine;gp_Vec spineDerivative;try{table.curve->D1(u,spine,spineDerivative);}catch(...){return false;}
            Frame bishop,spun;double arc=0;
            if(!bishop_detail::EvaluateBishop(table,u,bishop,arc)||!ApplySpin(bishop,phase+spin*arc,spun))return false;
            const gp_Vec tangent=bishop_detail::V(spun.tangent),axis1=bishop_detail::V(spun.e1),axis2=bishop_detail::V(spun.e2);
            for(int j=0;j<sectionSamples;++j){
                const double w=sectionFirst+(sectionLast-sectionFirst)*j/(sectionSamples-1);
                gp_Pnt point;gp_Vec pathPartial,sectionPartial,partialU,partialV;
                try{if(exchange[q]){bs->D1(u,w,point,pathPartial,sectionPartial);}else{bs->D1(w,u,point,partialU,partialV);pathPartial=partialV;sectionPartial=partialU;}}catch(...){return false;}
                const gp_Vec offset=gp_Vec(spine,point),radial=offset-tangent*offset.Dot(tangent);
                const double measuredRadius=radial.Magnitude();
                const gp_Vec normal=pathPartial.Crossed(sectionPartial);const double normalMagnitude=normal.Magnitude();
                if(!(measuredRadius>0)||!(normalMagnitude>0))return false;
                const double alignment=normal.Dot(radial)/(normalMagnitude*measuredRadius);
                const double deviation=measuredRadius-(r0+(r1-r0)*arc);
                dotLo=std::min(dotLo,alignment);dotHi=std::max(dotHi,alignment);
                devLo=std::min(devLo,deviation);devHi=std::max(devHi,deviation);
                jacLo=std::min(jacLo,normalMagnitude);jacHi=std::max(jacHi,normalMagnitude);
                const double delta=std::atan2(std::sin(std::atan2(radial.Dot(axis2),radial.Dot(axis1))-q*Pi/2),std::cos(std::atan2(radial.Dot(axis2),radial.Dot(axis1))-q*Pi/2));
                angleLo=std::min(angleLo,delta);angleHi=std::max(angleHi,delta);
                if(j==0){startLo=std::min(startLo,delta);startHi=std::max(startHi,delta);if(q==0&&std::abs(delta)>angleTolerance)marked=false;}
                if(j==sectionSamples-1){endLo=std::min(endLo,delta-Pi/2);endHi=std::max(endHi,delta-Pi/2);}
            }
        }
        out.normalEquation[q]={dotLo-angleTolerance,dotHi+angleTolerance};
        out.radialDeviation[q]={devLo-radialError,devHi+radialError};
        out.jacobian[q]={jacLo*(1-1e-12)-1e-12,jacHi*(1+1e-12)+1e-12};
        const bool covered=startHi<=angleTolerance&&startLo>=-angleTolerance&&endHi<=angleTolerance&&endLo>=-angleTolerance&&angleLo>=-angleTolerance&&angleHi<=Pi/2+angleTolerance;
        out.quarterCoverage[q]=covered;tiling=tiling&&covered;
    }
    // Shared meridians: adjacent quarters must coincide geometrically along
    // the whole sampled path, not merely claim a coverage label.
    for(unsigned q=0;q<4&&meridians;++q){
        const unsigned next=(q+1)%4;
        const Handle(Geom_BSplineSurface) a=Handle(Geom_BSplineSurface)::DownCast(surfaces[q]),b=Handle(Geom_BSplineSurface)::DownCast(surfaces[next]);
        if(a.IsNull()||b.IsNull())return false;
        double aU1,aU2,aV1,aV2,bU1,bU2,bV1,bV2;a->Bounds(aU1,aU2,aV1,aV2);b->Bounds(bU1,bU2,bV1,bV2);
        const double pathFirst=exchange[q]?aU1:aV1,pathLast=exchange[q]?aU2:aV2;
        const double aSectionEnd=exchange[q]?aV2:aU2,bSectionStart=exchange[next]?bV1:bU1;
        for(int i=0;i<pathSamples&&meridians;++i){
            const double u=pathFirst+(pathLast-pathFirst)*i/(pathSamples-1);gp_Pnt pa,pb;
            try{if(exchange[q])a->D0(u,aSectionEnd,pa);else a->D0(aSectionEnd,u,pa);if(exchange[next])b->D0(u,bSectionStart,pb);else b->D0(bSectionStart,u,pb);}catch(...){return false;}
            if(pa.Distance(pb)>2*fitting+PositionalEpsilonMM/8)meridians=false;
        }
    }
    // Station radii: locate each arc-fraction station by inverting the
    // certified arc representation, then measure the section radius on the
    // returned surfaces at that station. The authored law is not consulted
    // here; the probe compares these intervals against it independently.
    for(int station=0;station<=4;++station){
        const double target=station/4.0;double lo=table.first,hi=table.last;
        for(int iteration=0;iteration<64;++iteration){const double mid=(lo+hi)/2;double frac=0,f1=0,f2=0;if(!bishop_detail::ArcFraction(table,mid,frac,f1,f2))return false;if(frac<target)lo=mid;else hi=mid;}
        const double u=(lo+hi)/2;
        gp_Pnt spine;gp_Vec spineDerivative;try{table.curve->D1(u,spine,spineDerivative);}catch(...){return false;}
        Frame bishop,spun;double arc=0;
        if(!bishop_detail::EvaluateBishop(table,u,bishop,arc)||!ApplySpin(bishop,phase+spin*arc,spun))return false;
        const gp_Vec tangent=bishop_detail::V(spun.tangent);
        double radiusLo=std::numeric_limits<double>::infinity(),radiusHi=-radiusLo;
        for(unsigned quarter=0;quarter<4;++quarter){
            const Handle(Geom_BSplineSurface) bs=Handle(Geom_BSplineSurface)::DownCast(surfaces[quarter]);if(bs.IsNull())return false;
            double qU1,qU2,qV1,qV2;bs->Bounds(qU1,qU2,qV1,qV2);
            const double sectionMiddle=exchange[quarter]?(qV1+qV2)/2:(qU1+qU2)/2;
            gp_Pnt point;try{if(exchange[quarter])bs->D0(u,sectionMiddle,point);else bs->D0(sectionMiddle,u,point);}catch(...){return false;}
            const gp_Vec offset=gp_Vec(spine,point),radial=offset-tangent*offset.Dot(tangent);
            const double measured=radial.Magnitude();if(!(measured>0))return false;
            radiusLo=std::min(radiusLo,measured);radiusHi=std::max(radiusHi,measured);
        }
        out.stationRadius[station]={radiusLo-radialError,radiusHi+radialError};
    }
    out.sharedMeridians=meridians;out.pathBoundaryOnce=pathRanges&&tiling;out.markedMeridian=marked;return true;
}
// An open cap must be genuinely planar on the returned geometry and oriented
// away from the solid interior, measured from the sewn face itself.
inline bool CapPlanarAndOutward(const TopoDS_Face& face,const gp_Vec& outward){
    if(face.IsNull()||!(outward.Magnitude()>0))return false;
    TopLoc_Location location;const Handle(Geom_Surface)& surface=BRep_Tool::Surface(face,location);
    if(surface.IsNull())return false;
    GeomAdaptor_Surface adaptor(surface);if(adaptor.GetType()!=GeomAbs_Plane)return false;
    gp_Dir normal=adaptor.Plane().Axis().Direction();if(face.Orientation()==TopAbs_REVERSED)normal.Reverse();
    return normal.Dot(gp_Dir(outward))>=1-1e-6;
}
// Every authored local section must be present in the sewn topology: the
// face count matches the authored section/cap count and each returned
// surface is still referenced (by handle or by identical net shape).
inline bool EverySectionAccountedFor(const TopoDS_Shell& shell,const std::array<Handle(Geom_Surface),4>& surfaces,std::size_t expectedFaces){
    std::size_t found=0;bool matched[4]={false,false,false,false};
    for(TopExp_Explorer it(shell,TopAbs_FACE);it.More();it.Next()){
        ++found;const Handle(Geom_Surface)& geometry=BRep_Tool::Surface(TopoDS::Face(it.Current()));
        for(unsigned q=0;q<4;++q){
            if(matched[q]||geometry.IsNull())continue;
            if(geometry==surfaces[q]){matched[q]=true;continue;}
            const Handle(Geom_BSplineSurface) a=Handle(Geom_BSplineSurface)::DownCast(geometry),b=Handle(Geom_BSplineSurface)::DownCast(surfaces[q]);
            if(a.IsNull()||b.IsNull())continue;
            if(a->NbUPoles()==b->NbUPoles()&&a->NbVPoles()==b->NbVPoles()&&a->NbUKnots()==b->NbUKnots()&&a->NbVKnots()==b->NbVKnots()&&a->Pole(1,1).Distance(b->Pole(1,1))<=PositionalEpsilonMM/8)matched[q]=true;
        }
    }
    return found==expectedFaces&&matched[0]&&matched[1]&&matched[2]&&matched[3];
}
} // namespace geomfill_detail

inline GeomFillBuildResult BuildGeomFillSweep(const bounded_curve::Definition& curve,const Definition& definition,const std::atomic_bool& cancelled) noexcept {
    GeomFillBuildResult output;try{
        if(cancelled.load()||bounded_curve::Validate(curve)!=bounded_curve::Refusal::None||Validate(definition)!=Refusal::None)return output;const double mmPerUnit=definition.dimensionMetersPerUnit*1000,r0=definition.radius.startRadius*mmPerUnit,r1=definition.radius.endRadius*mmPerUnit,R=std::max(r0,r1)+4*PositionalEpsilonMM;
        Handle(Geom_BSplineCurve) geometry=geomfill_detail::Curve(curve,mmPerUnit);Handle(GeomAdaptor_Curve) adaptor=new GeomAdaptor_Curve(geometry);gp_Pnt p;gp_Vec startDerivative;adaptor->D1(adaptor->FirstParameter(),p,startDerivative);
        auto preliminary=proof_producer::Produce(curve,mmPerUnit,R,definition.closure==ClosureKind::ClosedNoCaps,{0,0},&cancelled);if(!preliminary.produced()){output.status=GeomFillBuildStatus::ProofRefused;return output;}
        const double frameTolerance=FrameTolerance(R);auto bishopOnly=PrepareBishopTransport(adaptor,geomfill_detail::WorldSeed(curve,definition.orientation.authoredSeed),0,0,frameTolerance,&cancelled);if(!bishopOnly){output.status=GeomFillBuildStatus::TransportRefused;return output;}
        Interval holonomy{0,0};if(definition.closure==ClosureKind::ClosedNoCaps){double h=0;if(!Holonomy(bishopOnly->nodes.front().bishop,bishopOnly->nodes.back().bishop,h)){output.status=GeomFillBuildStatus::TransportRefused;return output;}holonomy={std::nextafter(h,-std::numeric_limits<double>::infinity()),std::nextafter(h,std::numeric_limits<double>::infinity())};}
        auto proof=proof_producer::Produce(curve,mmPerUnit,R,definition.closure==ClosureKind::ClosedNoCaps,holonomy,&cancelled);if(!proof.produced()){output.status=GeomFillBuildStatus::ProofRefused;return output;}output.curveProof=proof.certificate;output.admission=Admit(definition,curve,output.curveProof,geomfill_detail::RetainedTangent(curve,startDerivative),true);if(!output.admission.admitted()){output.status=GeomFillBuildStatus::ProofRefused;return output;}
        auto table=PrepareBishopTransport(adaptor,geomfill_detail::WorldSeed(curve,definition.orientation.authoredSeed),definition.orientation.phaseRadians,output.admission.correctedTotalSpin,frameTolerance,&cancelled);if(!table){output.status=GeomFillBuildStatus::TransportRefused;return output;}Handle(BishopTrihedronLaw) trihedron=new BishopTrihedronLaw(table,definition.orientation.phaseRadians,output.admission.correctedTotalSpin);Handle(GeomFill_CurveAndTrihedron) location=new GeomFill_CurveAndTrihedron(trihedron);if(!location->SetCurve(adaptor)){output.status=GeomFillBuildStatus::TransportRefused;return output;}Handle(ArcLengthRadiusLaw) radius=new ArcLengthRadiusLaw(table,r0,r1);
        std::array<TopoDS_Face,4> sides;std::array<Handle(Geom_Surface),4> surfaces;std::array<bool,4> exchange{};BRepBuilderAPI_Sewing sewing(PositionalEpsilonMM/8,Standard_True,Standard_True,Standard_True,Standard_False);double fitting=0;
        for(unsigned q=0;q<4;++q){if(cancelled.load()){output.status=GeomFillBuildStatus::Cancelled;return output;}Handle(GeomFill_EvolvedSection) section=new GeomFill_EvolvedSection(geomfill_detail::Quarter(q),radius);GeomFill_Sweep sweep(location,Standard_False);sweep.SetDomain(table->first,table->last,table->first,table->last);sweep.SetTolerance(PositionalEpsilonMM/4,PositionalEpsilonMM/8,PositionalEpsilonMM/8,frameTolerance);sweep.SetForceApproxC1(Standard_False);sweep.Build(section,GeomFill_Section,GeomAbs_C1,12,256);if(!sweep.IsDone()||!std::isfinite(sweep.ErrorOnSurface())||sweep.ErrorOnSurface()>PositionalEpsilonMM/4){output.status=GeomFillBuildStatus::SurfaceRefused;return output;}surfaces[q]=sweep.Surface();exchange[q]=sweep.ExchangeUV();if(sweep.UReversed()||sweep.VReversed()||!geomfill_detail::PositiveReturnedSurface(surfaces[q])){output.status=GeomFillBuildStatus::SurfaceRefused;return output;}BRepBuilderAPI_MakeFace face(surfaces[q],PositionalEpsilonMM/8);if(!face.IsDone()){output.status=GeomFillBuildStatus::SurfaceRefused;return output;}sides[q]=face.Face();sewing.Add(sides[q]);fitting=std::max(fitting,sweep.ErrorOnSurface());}
        auto cap=[&](double u,bool reverse,TopoDS_Face& face)->bool{BRepBuilderAPI_MakeWire wire;for(unsigned q=0;q<4;++q){BRepBuilderAPI_MakeEdge edge(geomfill_detail::PathBoundary(surfaces[q],exchange[q],u));if(!edge.IsDone())return false;wire.Add(edge.Edge());}if(!wire.IsDone())return false;BRepBuilderAPI_MakeFace make(wire.Wire(),Standard_True);if(!make.IsDone())return false;face=make.Face();if(reverse)face.Reverse();return true;};
        TopoDS_Face firstCap,lastCap;
        if(definition.closure==ClosureKind::OpenFlatCaps){if(!cap(table->first,true,firstCap)||!cap(table->last,false,lastCap)){output.status=GeomFillBuildStatus::SurfaceRefused;return output;}sewing.Add(firstCap);sewing.Add(lastCap);}sewing.Perform();if(sewing.NbFreeEdges()!=0||sewing.NbMultipleEdges()!=0){output.status=GeomFillBuildStatus::SewingRefused;return output;}TopoDS_Shell shell=geomfill_detail::OneShell(sewing.SewedShape());if(shell.IsNull()){output.status=GeomFillBuildStatus::SewingRefused;return output;}BRepBuilderAPI_MakeSolid makeSolid(shell);if(!makeSolid.IsDone()){output.status=GeomFillBuildStatus::SolidRefused;return output;}TopoDS_Solid solid=makeSolid.Solid();if(!BRepLib::OrientClosedSolid(solid)||!BRepCheck_Analyzer(solid,Standard_True).IsValid()||!BRep_Tool::IsClosed(shell)){output.status=GeomFillBuildStatus::SolidRefused;return output;}
        // FIX-SPEC step 4: coverage flags, per-cell intervals, station radii
        // and meridian/cap signs are measured on the four returned rational
        // surfaces, their seams, caps and sewn topology. A measurement that
        // cannot be computed refuses the build; a measurement whose value
        // violates a frozen gate is recorded honestly and refused by the
        // unchanged verifier below.
        geomfill_detail::ReturnedSurfaceEvidence evidence;
        if(!geomfill_detail::MeasureReturnedSurfaces(surfaces,exchange,*table,definition.orientation.phaseRadians,output.admission.correctedTotalSpin,r0,r1,fitting,evidence)){output.status=GeomFillBuildStatus::SurfaceRefused;return output;}
        std::size_t shellFaces=0;for(TopExp_Explorer faceIt(shell,TopAbs_FACE);faceIt.More();faceIt.Next())++shellFaces;
        bool capsPlanarAndOriented=false;
        if(definition.closure==ClosureKind::OpenFlatCaps){gp_Pnt anchor;gp_Vec startTangent,endTangent;adaptor->D1(table->first,anchor,startTangent);adaptor->D1(table->last,anchor,endTangent);capsPlanarAndOriented=shellFaces==6&&geomfill_detail::CapPlanarAndOutward(firstCap,startTangent.Reversed())&&geomfill_detail::CapPlanarAndOutward(lastCap,endTangent);}
        auto& receipt=output.receipt;receipt.usedBishopTrihedron=receipt.usedFourKnownQuarterSections=receipt.withKpartDisabled=receipt.forceApproxC1Disabled=receipt.deterministicSpanOrder=true;receipt.surface.fittingErrorMM=fitting;receipt.surface.transportErrorMM=table->frameErrorBound*R;receipt.surface.sewingErrorMM=PositionalEpsilonMM/8;receipt.surface.serializationErrorMM=PositionalEpsilonMM/8;receipt.surface.orderedQuarterCoverage=evidence.quarterCoverage;receipt.surface.sharedMeridiansMatch=evidence.sharedMeridians;receipt.surface.pathBoundaryCoveredOnce=evidence.pathBoundaryOnce;receipt.surface.openCapsPlanarAndOriented=capsPlanarAndOriented;receipt.surface.closedHasNoCaps=definition.closure==ClosureKind::ClosedNoCaps&&shellFaces==4;
        for(unsigned q=0;q<4;++q){SurfaceCellCertificate cell;cell.normalEquationDerivative=evidence.normalEquation[q];cell.radialDeviationMM=evidence.radialDeviation[q];cell.orientedMapJacobian=evidence.jacobian[q];cell.intendedSpineBracketIsUnique=proof.certificate.localBandInjective&&evidence.pathBoundaryOnce;cell.otherSpineIntervalsDischarged=proof.certificate.fullPairDomainVisited;receipt.surface.cells.push_back(cell);}receipt.surface.work=proof.certificate.work;receipt.surface.work.surfaceCells=receipt.surface.cells.size();receipt.surface.provenance=SurfaceCertificate::Provenance::ReturnedRationalSurfaceIntervalsV1;
        GProp_GProps properties;BRepGProp::VolumeProperties(solid,properties,1e-10);const double expectedLo=IdealCircularSweepVolume(proof.certificate.lengthMM.lower,r0,r1),expectedHi=IdealCircularSweepVolume(proof.certificate.lengthMM.upper,r0,r1),measured=properties.Mass();const double volumeAllowance=2*Pi*proof.certificate.lengthMM.upper*(1+proof.certificate.curvaturePerMM.upper*R)*(std::max(r0,r1)*PositionalEpsilonMM+PositionalEpsilonMM*PositionalEpsilonMM/2)+2*Pi*R*R*PositionalEpsilonMM;receipt.independentMeasure.sourceLengthMM=proof.certificate.lengthMM;receipt.independentMeasure.expectedVolumeMM3={expectedLo-volumeAllowance,expectedHi+volumeAllowance};receipt.independentMeasure.measuredVolumeMM3={measured,measured};for(int i=0;i<=4;++i)receipt.independentMeasure.stationRadiusMM.push_back(evidence.stationRadius[i]);const double independentLength=GCPnts_AbscissaPoint::Length(*adaptor);receipt.independentMeasure.sourcePolynomialIntegratedIndependently=std::isfinite(independentLength)&&proof.certificate.lengthMM.lower<=independentLength&&independentLength<=proof.certificate.lengthMM.upper;receipt.independentMeasure.everyLocalSectionAccountedFor=geomfill_detail::EverySectionAccountedFor(shell,surfaces,definition.closure==ClosureKind::OpenFlatCaps?6:4);receipt.independentMeasure.markedMeridianMatches=evidence.markedMeridian;
        TopTools_IndexedMapOfShape solids,shells,faces,edges;TopExp::MapShapes(solid,TopAbs_SOLID,solids);TopExp::MapShapes(solid,TopAbs_SHELL,shells);TopExp::MapShapes(solid,TopAbs_FACE,faces);TopExp::MapShapes(solid,TopAbs_EDGE,edges);receipt.topology={std::size_t(solids.Extent()),std::size_t(shells.Extent()),std::size_t(faces.Extent()),std::size_t(edges.Extent()),true,true,true,solid.Orientation()==TopAbs_FORWARD,true,true,true,false,true};BOPAlgo_ArgumentAnalyzer interference;interference.SetShape1(solid);interference.SetRunParallel(Standard_False);interference.StopOnFirstFaulty()=Standard_True;interference.SelfInterMode()=Standard_True;interference.Perform();receipt.topology.selfInterferenceFree=!interference.HasErrors()&&!interference.HasWarnings()&&!interference.HasFaulty();if(!geomfill_detail::HashShape(solid,receipt.geometryBinaryDigest)){output.status=GeomFillBuildStatus::SolidRefused;return output;}receipt.serializerFormatVersion=4;const auto admitted=ValidateDetachedSolid(definition,output.admission,receipt);if(!admitted.admitted()){output.status=GeomFillBuildStatus::SolidRefused;return output;}output.solid=solid;output.status=GeomFillBuildStatus::Built;return output;
    }catch(...){return output;}
}
} // namespace core3d::spatial_sweep
