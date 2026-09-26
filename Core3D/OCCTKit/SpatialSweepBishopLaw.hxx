#pragma once

// Concrete query-order-independent OCCT trihedron. Preparation walks the
// complete path once, left-to-right, and seals an immutable dense table.
// Copies and trimmed laws share that table; SetInterval never reseeds or
// restarts transport. There is intentionally no Frenet/discrete fallback.
#include "SpatialSweepTransport.hxx"
#include <Adaptor3d_Curve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomFill_TrihedronLaw.hxx>
#include <GeomAbs_Shape.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <atomic>
#include <memory>

namespace core3d::spatial_sweep {
struct BishopTableNode {
    double parameter=0;
    double arcLengthMM=0;
    Frame bishop;
    double accumulatedAngularError=0;
    // Curve speed and its parameter rate at this node, captured once while the
    // table is sealed. They anchor the quintic Hermite arc representation so
    // value, first and second derivative queries stay mutually consistent.
    double speed=0;
    double speedRate=0;
};

struct BishopTransportTable {
    Handle(Adaptor3d_Curve) curve;
    std::vector<BishopTableNode> nodes;
    std::vector<double> continuityBounds;
    PreparedTransport normalized;
    double first=0,last=0,totalLengthMM=0,frameErrorBound=0;
    bool valid=false;
};

namespace bishop_detail {
inline Vector3 V(const gp_Vec& v){return {{v.X(),v.Y(),v.Z()}};}
inline gp_Vec V(const Vector3& v){return gp_Vec(v[0],v[1],v[2]);}
inline double Angle(const Vector3& a,const Vector3& b){return std::atan2(Norm(Cross(a,b)),Dot(a,b));}
inline bool CurveData(const Handle(Adaptor3d_Curve)& curve,double u,gp_Vec& d1,gp_Vec* d2=nullptr,gp_Vec* d3=nullptr){
    try{gp_Pnt p;if(d3){gp_Vec a,b;curve->D3(u,p,d1,a,b);*d2=a;*d3=b;}else if(d2)curve->D2(u,p,d1,*d2);else curve->D1(u,p,d1);return d1.SquareMagnitude()>0&&std::isfinite(d1.SquareMagnitude());}catch(...){return false;}
}
struct PreparedSegment{Frame finish;double length=0,error=0;};
inline bool Step(const Handle(Adaptor3d_Curve)& curve,double a,double b,const Frame& start,PreparedSegment& out){
    gp_Vec va,vm,vb;if(!CurveData(curve,a,va)||!CurveData(curve,(a+b)/2,vm)||!CurveData(curve,b,vb))return false;
    Frame middle{},finish{},coarse{};if(!BishopStep(start,V(vm),middle)||!BishopStep(middle,V(vb),finish)||!BishopStep(start,V(vb),coarse))return false;
    const double h=b-a;const double fineLength=h*(va.Magnitude()+4*vm.Magnitude()+vb.Magnitude())/6;const double coarseLength=h*(va.Magnitude()+vb.Magnitude())/2;
    out.finish=finish;out.length=fineLength;out.error=Angle(finish.e1,coarse.e1)+std::abs(fineLength-coarseLength)/std::max(1.0,fineLength);return std::isfinite(out.error)&&fineLength>0;
}
inline bool Append(const Handle(Adaptor3d_Curve)& curve,double a,double b,const Frame& start,
                   double tolerance,double range,std::uint32_t depth,std::vector<BishopTableNode>& nodes,
                   double& length,double& accumulated,std::size_t& leaves,const std::atomic_bool* cancelled){
    if(cancelled&&cancelled->load())return false;PreparedSegment whole;if(!Step(curve,a,b,start,whole))return false;
    const double mid=(a+b)/2;PreparedSegment left,right;if(!Step(curve,a,mid,start,left)||!Step(curve,mid,b,left.finish,right))return false;
    const double defect=Angle(whole.finish.e1,right.finish.e1)+std::abs(whole.length-left.length-right.length)/std::max(1.0,left.length+right.length);
    // One global budget for the whole path: each cell owns a deterministic
    // parameter-width share of it, and a cell whose defect would exceed the
    // remaining global allowance is refined locally instead of consuming it.
    // The depth/leaf budgets and the final accumulated check are unchanged.
    if(defect>tolerance*(b-a)/range/4||accumulated+defect>tolerance){if(depth>=MaximumBisectionDepth||leaves+2>MaximumProofLeaves)return false;if(!Append(curve,a,mid,start,tolerance,range,depth+1,nodes,length,accumulated,leaves,cancelled))return false;return Append(curve,mid,b,nodes.back().bishop,tolerance,range,depth+1,nodes,length,accumulated,leaves,cancelled);}
    length+=left.length+right.length;accumulated+=defect;nodes.push_back({b,length,right.finish,accumulated});++leaves;return accumulated<=tolerance;
}
inline bool TangentDerivatives(const Handle(Adaptor3d_Curve)& curve,double u,Vector3& t,Vector3& tu,Vector3& tuu,double& speed,double& speedU){
    gp_Vec v,a,j;if(!CurveData(curve,u,v,&a,&j))return false;const Vector3 Vv=V(v),A=V(a),J=V(j);speed=Norm(Vv);if(!(speed>0))return false;const double d=Dot(Vv,A),dp=Dot(A,A)+Dot(Vv,J);t=Scale(Vv,1/speed);tu=Subtract(Scale(A,1/speed),Scale(Vv,d/(speed*speed*speed)));tuu=Add(Subtract(Scale(J,1/speed),Scale(A,2*d/(speed*speed*speed))),Add(Scale(Vv,3*d*d/std::pow(speed,5)),Scale(Vv,-dp/std::pow(speed,3))));speedU=d/speed;return true;
}
// The pinned OCCT 7.8.0 kernel issues endpoint queries at the representable
// double adjacent to a retained bound (exactly nextafter beyond it). Such a
// request is numerically equivalent to the retained endpoint and is
// normalized to it; any wider excursion is a genuine out-of-domain request
// and refuses. This is intentionally not a general clamp.
inline bool NormalizeEndpointQuery(double& u,double first,double last){
    if(!std::isfinite(u))return false;
    if(u<first){if(u!=std::nextafter(first,-std::numeric_limits<double>::infinity()))return false;u=first;}
    else if(u>last){if(u!=std::nextafter(last,std::numeric_limits<double>::infinity()))return false;u=last;}
    return true;
}
// Normalized arc fraction and its first two parameter derivatives from one
// bounded representation: a quintic Hermite interpolant over the sealed table
// matching q, dq/du and d2q/du2 at both bracketing nodes. Value, D1 and D2
// queries therefore all derive from the same representation instead of mixing
// a piecewise-linear arc value with unrelated analytic arc derivatives.
inline bool ArcFraction(const BishopTransportTable& table,double u,double& q,double& q1,double& q2){
    if(!table.valid||u<table.first||u>table.last||table.nodes.size()<2||!(table.totalLengthMM>0))return false;
    const auto upper=std::upper_bound(table.nodes.begin(),table.nodes.end(),u,[](double x,const BishopTableNode& n){return x<n.parameter;});
    const std::size_t right=upper==table.nodes.begin()?1:upper==table.nodes.end()?table.nodes.size()-1:std::size_t(upper-table.nodes.begin());
    const BishopTableNode& a=table.nodes[right-1],&b=table.nodes[right];
    const double h=b.parameter-a.parameter;if(!(h>0))return false;
    const double t=(u-a.parameter)/h,L=table.totalLengthMM;
    const double p0=a.arcLengthMM/L,p1=b.arcLengthMM/L,m0=a.speed/L*h,m1=b.speed/L*h,s0=a.speedRate/L*h*h,s1=b.speedRate/L*h*h;
    const double A=p1-p0-m0-s0/2,B=m1-m0-s0,C=s1-s0;
    const double c5=(C+12*A-6*B)/2,c4=(B-3*A)-2*c5,c3=A-c4-c5;
    q=p0+m0*t+0.5*s0*t*t+((c5*t+c4)*t+c3)*t*t*t;
    q1=(m0+s0*t+((5*c5*t+4*c4)*t+3*c3)*t*t)/h;
    q2=(s0+((20*c5*t+12*c4)*t+6*c3)*t)/(h*h);
    return std::isfinite(q)&&std::isfinite(q1)&&std::isfinite(q2);
}
inline bool EvaluateBishop(const BishopTransportTable& table,double u,Frame& bishop,double& q,double* q1=nullptr,double* q2=nullptr){
    if(!table.valid||u<table.first||u>table.last)return false;auto upper=std::upper_bound(table.nodes.begin(),table.nodes.end(),u,[](double x,const BishopTableNode& n){return x<n.parameter;});const BishopTableNode& base=upper==table.nodes.begin()?table.nodes.front():*(upper-1);gp_Vec derivative;if(!CurveData(table.curve,u,derivative)||!BishopStep(base.bishop,V(derivative),bishop))return false;
    double arc=0,arc1=0,arc2=0;if(!ArcFraction(table,u,arc,arc1,arc2))return false;q=arc;if(q1)*q1=arc1;if(q2)*q2=arc2;return true;
}
} // namespace bishop_detail

inline std::shared_ptr<const BishopTransportTable> PrepareBishopTransport(
    const Handle(Adaptor3d_Curve)& curve,const Vector3& authoredSeed,
    double phase,double totalSpin,double frameTolerance,
    const std::atomic_bool* cancelled=nullptr) noexcept {
    try{
        if(curve.IsNull()||!Finite(phase)||!Finite(totalSpin)||!Finite(frameTolerance)||frameTolerance<=0)return {};
        auto table=std::make_shared<BishopTransportTable>();table->curve=curve;table->first=curve->FirstParameter();table->last=curve->LastParameter();if(!(table->first<table->last))return {};
        gp_Vec initialDerivative;if(!bishop_detail::CurveData(curve,table->first,initialDerivative))return {};Frame initial;if(!SeedFrame(bishop_detail::V(initialDerivative),authoredSeed,initial))return {};
        table->nodes.push_back({table->first,0,initial,0});const int count=curve->NbIntervals(GeomAbs_C2);TColStd_Array1OfReal intervals(1,count+1);curve->Intervals(intervals,GeomAbs_C2);table->continuityBounds.reserve(count+1);for(int i=intervals.Lower();i<=intervals.Upper();++i)table->continuityBounds.push_back(intervals(i));
        const double range=table->last-table->first;double length=0,error=0;std::size_t leaves=0;for(std::size_t i=0;i+1<table->continuityBounds.size();++i)if(!bishop_detail::Append(curve,table->continuityBounds[i],table->continuityBounds[i+1],table->nodes.back().bishop,frameTolerance,range,0,table->nodes,length,error,leaves,cancelled))return {};
        if(!(length>0)||error>frameTolerance)return {};
        for(auto& node:table->nodes){gp_Vec v,a;if(!bishop_detail::CurveData(curve,node.parameter,v,&a))return {};const double speed=v.Magnitude();if(!(speed>0)||!std::isfinite(speed))return {};node.speed=speed;node.speedRate=v.Dot(a)/speed;if(!std::isfinite(node.speedRate))return {};}table->totalLengthMM=length;table->frameErrorBound=error;table->normalized.phase=phase;table->normalized.totalSpin=totalSpin;for(const auto& n:table->nodes)table->normalized.stations.push_back({n.arcLengthMM/length,n.bishop});table->normalized.stations.front().normalizedArcLength=0;table->normalized.stations.back().normalizedArcLength=1;table->valid=Valid(table->normalized);return table;
    }catch(...){return {};}
}

class BishopTrihedronLaw final : public GeomFill_TrihedronLaw {
    DEFINE_STANDARD_RTTI_INLINE(BishopTrihedronLaw,GeomFill_TrihedronLaw)
public:
    // The constructor initializes the OCCT base curve state through the base
    // SetCurve API, so a copied law can have SetInterval call the base
    // trimming implementation immediately; transport is never reseeded.
    BishopTrihedronLaw(std::shared_ptr<const BishopTransportTable> table,double phase,double totalSpin)
        :table_(std::move(table)),phase_(phase),spin_(totalSpin){if(table_){first_=table_->first;last_=table_->last;SetCurve(table_->curve);}}
    Handle(GeomFill_TrihedronLaw) Copy() const override {auto copy=new BishopTrihedronLaw(table_,phase_,spin_);copy->first_=first_;copy->last_=last_;return copy;}
    // Coincident parameter bounds alone cannot pair this law with an
    // unrelated precomputed table: a different adaptor is accepted only when
    // it is the retained instance or wraps the identical underlying geometry.
    Standard_Boolean SetCurve(const Handle(Adaptor3d_Curve)& curve) override {if(!table_||!table_->valid||curve.IsNull()||curve->FirstParameter()!=table_->first||curve->LastParameter()!=table_->last)return Standard_False;if(curve!=table_->curve){const Handle(GeomAdaptor_Curve) offered=Handle(GeomAdaptor_Curve)::DownCast(curve),retained=Handle(GeomAdaptor_Curve)::DownCast(table_->curve);if(offered.IsNull()||retained.IsNull()||offered->Curve()!=retained->Curve())return Standard_False;}myCurve=curve;return Standard_True;}
    Standard_Boolean D0(Standard_Real u,gp_Vec& tangent,gp_Vec& normal,gp_Vec& binormal) override {Frame f;if(!frame(u,f,nullptr))return Standard_False;tangent=bishop_detail::V(f.tangent);normal=bishop_detail::V(f.e1);binormal=bishop_detail::V(f.e2);return Standard_True;}
    Standard_Boolean D1(Standard_Real u,gp_Vec& tangent,gp_Vec& dt,gp_Vec& normal,gp_Vec& dn,gp_Vec& binormal,gp_Vec& db) override {
        Frame f;double q=0,q1=0,q2=0;if(!frame(u,f,&q,&q1,&q2))return Standard_False;Vector3 t,tu,tuu;double speed=0,speedU=0;if(!bishop_detail::TangentDerivatives(table_->curve,u,t,tu,tuu,speed,speedU))return Standard_False;const double beta=spin_*q1;const Vector3 e1u=Add(Scale(t,-Dot(tu,f.e1)),Scale(f.e2,beta)),e2u=Add(Scale(t,-Dot(tu,f.e2)),Scale(f.e1,-beta));tangent=bishop_detail::V(t);dt=bishop_detail::V(tu);normal=bishop_detail::V(f.e1);dn=bishop_detail::V(e1u);binormal=bishop_detail::V(f.e2);db=bishop_detail::V(e2u);return Standard_True;
    }
    Standard_Boolean D2(Standard_Real u,gp_Vec& tangent,gp_Vec& dt,gp_Vec& d2t,gp_Vec& normal,gp_Vec& dn,gp_Vec& d2n,gp_Vec& binormal,gp_Vec& db,gp_Vec& d2b) override {
        Frame f;double q=0,q1=0,q2=0;if(!frame(u,f,&q,&q1,&q2))return Standard_False;Vector3 t,tu,tuu;double speed=0,speedU=0;if(!bishop_detail::TangentDerivatives(table_->curve,u,t,tu,tuu,speed,speedU))return Standard_False;const double beta=spin_*q1,beta2=spin_*q2;const Vector3 e1u=Add(Scale(t,-Dot(tu,f.e1)),Scale(f.e2,beta)),e2u=Add(Scale(t,-Dot(tu,f.e2)),Scale(f.e1,-beta));const Vector3 e1uu=Add(Add(Scale(t,-Dot(tuu,f.e1)-Dot(tu,e1u)),Scale(tu,-Dot(tu,f.e1))),Add(Scale(f.e2,beta2),Scale(e2u,beta)));const Vector3 e2uu=Add(Add(Scale(t,-Dot(tuu,f.e2)-Dot(tu,e2u)),Scale(tu,-Dot(tu,f.e2))),Add(Scale(f.e1,-beta2),Scale(e1u,-beta)));tangent=bishop_detail::V(t);dt=bishop_detail::V(tu);d2t=bishop_detail::V(tuu);normal=bishop_detail::V(f.e1);dn=bishop_detail::V(e1u);d2n=bishop_detail::V(e1uu);binormal=bishop_detail::V(f.e2);db=bishop_detail::V(e2u);d2b=bishop_detail::V(e2uu);return Standard_True;
    }
    // Clipped interval count derived from the actual interior breaks plus
    // endpoints, so it always agrees with the values Intervals() emits,
    // including trims that span partial end cells across several intervals.
    Standard_Integer NbIntervals(const GeomAbs_Shape) const override {if(!table_)return 0;std::size_t interior=0;for(double bound:table_->continuityBounds)if(bound>first_&&bound<last_)++interior;return Standard_Integer(interior+1);}
    void Intervals(TColStd_Array1OfReal& out,const GeomAbs_Shape) const override {std::vector<double> values{first_};if(table_)for(double u:table_->continuityBounds)if(u>first_&&u<last_)values.push_back(u);values.push_back(last_);for(std::size_t i=0;i<values.size()&&int(i)+out.Lower()<=out.Upper();++i)out(out.Lower()+int(i))=values[i];}
    void SetInterval(Standard_Real first,Standard_Real last) override {if(table_&&first>=table_->first&&last<=table_->last&&first<last){first_=first;last_=last;if(!myCurve.IsNull())GeomFill_TrihedronLaw::SetInterval(first,last);}}
    void GetAverageLaw(gp_Vec& t,gp_Vec& n,gp_Vec& b) override {D0((first_+last_)/2,t,n,b);}
    Standard_Boolean IsConstant() const override {return Standard_False;}
    Standard_Boolean IsOnlyBy3dCurve() const override {return Standard_False;}
private:
    // The normalized parameter is used consistently for the frame, the arc
    // fraction and every downstream curve derivative query.
    bool frame(double& u,Frame& output,double* q,double* q1=nullptr,double* q2=nullptr) const {if(!table_||!bishop_detail::NormalizeEndpointQuery(u,first_,last_))return false;Frame bishop;double arc=0;if(!bishop_detail::EvaluateBishop(*table_,u,bishop,arc,q1,q2)||!ApplySpin(bishop,phase_+spin_*arc,output))return false;if(q)*q=arc;return true;}
    std::shared_ptr<const BishopTransportTable> table_;double phase_=0,spin_=0,first_=0,last_=0;
};
} // namespace core3d::spatial_sweep
