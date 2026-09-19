#pragma once
// Read-only recipe clearance and detached transverse-result verification.
// No occurrence, document owner, command, history, or permission is issued here.
#include "CylindricalCutDefinition.hxx"
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepTools.hxx>
#include <sstream>
#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cfenv>
#include <limits>
#include <optional>

#if defined(__FAST_MATH__)
#error Saved-cut interval clearance must not be compiled with fast-math.
#endif
namespace core3d::saved_cut_bore_clearance {
static_assert(std::numeric_limits<double>::is_iec559 && std::numeric_limits<double>::digits==53,
    "Saved-cut interval clearance requires IEEE binary64");
inline constexpr double KernelToleranceCapMM=.001;
inline constexpr double KernelSeparationMM=2*KernelToleranceCapMM;
inline constexpr double MaximumNumericUncertaintyMM=1e-6;
enum class Status { RefusedValues, UnsupportedFamily, UnsupportedFrame,
    Nonparallel, NumericUncertain, OutsideOrInsufficientLigament, ClearRecipeDisk, ClearRecipeTransverse };
struct Report {
    Status status=Status::RefusedValues;
    int sourcePlane=-1;
    int transverseAxis=-1; // Recipe X/Y; -1 retains the existing disk path.
    // Original object-local physical mm, BEFORE occurrence scale. The owner
    // layer may convert lengths once using its validated occurrence scale.
    double sourceScale=0, recipeUnitToOriginalMM=0;
    double radiusOriginalMM=0, boundaryDistanceLowerMM=0, boundaryDistanceUpperMM=0;
    double ligamentLowerMM=0, numericUncertaintyMM=0, angularSweepAllowanceMM=0;
    double kernelSeparationMM=KernelSeparationMM;
    std::array<double,2> centerRecipe{};
    std::array<double,2> wallIntervalRecipe{};
};
namespace detail {
// Outward binary64 interval operations. No fast-math/reassociation permitted.
// sqrt needs the platform's correctly-rounded IEEE binary64 sqrt; native
// boundary tests must qualify the target implementation before integration.
struct I { double lo=0,hi=0; explicit I(double x=0):lo(x),hi(x){} I(double a,double b):lo(a),hi(b){} };
inline double down(double x){return std::nextafter(x,-std::numeric_limits<double>::infinity());}
inline double up(double x){return std::nextafter(x,std::numeric_limits<double>::infinity());}
inline bool good(I a){return std::isfinite(a.lo)&&std::isfinite(a.hi)&&a.lo<=a.hi;}
// Invalid or nonfinite intervals are absorbing. Check before min/max so NaN
// ordering cannot hide a failed bound, and check outward results for overflow.
inline I invalid(){return {1,0};}
inline I checked(I a){return good(a)?a:invalid();}
inline I add(I a,I b){if(!good(a)||!good(b))return invalid();return checked({down(a.lo+b.lo),up(a.hi+b.hi)});}
inline I neg(I a){if(!good(a))return invalid();return checked({-a.hi,-a.lo});}
inline I sub(I a,I b){return add(a,neg(b));}
inline I mul(I a,I b){
    if(!good(a)||!good(b))return invalid();
    const std::array<double,4> v{a.lo*b.lo,a.lo*b.hi,a.hi*b.lo,a.hi*b.hi};
    for(double x:v)if(!std::isfinite(x))return invalid();
    return checked({down(*std::min_element(v.begin(),v.end())),up(*std::max_element(v.begin(),v.end()))});
}
inline I div(I a,I b){if(!good(a)||!good(b)||(b.lo<=0&&b.hi>=0))return invalid();return mul(a,checked({down(1/b.hi),up(1/b.lo)}));}
inline I sq(I a){
    if(!good(a))return invalid();
    if(a.lo<0&&a.hi<=0)return sq(neg(a));
    const double lo2=a.lo*a.lo,hi2=a.hi*a.hi;
    if(!std::isfinite(lo2)||!std::isfinite(hi2))return invalid();
    if(a.lo>=0)return checked({std::max(0.0,down(lo2)),up(hi2)});
    return checked({0,up(std::max(lo2,hi2))});
}
// Preserve the existing nonnegative-domain intersection for a straddling input.
inline I root(I a){if(!good(a)||a.hi<0)return invalid();return checked({std::max(0.0,down(std::sqrt(std::max(0.0,a.lo)))),up(std::sqrt(a.hi))});}
inline I abs(I a){if(!good(a))return invalid();if(a.lo>=0)return a;if(a.hi<=0)return neg(a);return checked({0,std::max(-a.lo,a.hi)});}
inline I minimum(I a,I b){if(!good(a)||!good(b))return invalid();return checked({std::min(a.lo,b.lo),std::min(a.hi,b.hi)});}
inline I maximum(I a,I b){if(!good(a)||!good(b))return invalid();return checked({std::max(a.lo,b.lo),std::max(a.hi,b.hi)});}
inline I norm(I a,I b){return root(add(sq(a),sq(b)));}
inline double midpoint(I a){if(!good(a))return std::numeric_limits<double>::quiet_NaN();const double result=a.lo+(a.hi-a.lo)*.5;return std::isfinite(result)?result:std::numeric_limits<double>::quiet_NaN();}
inline double mag(I a){if(!good(a))return std::numeric_limits<double>::quiet_NaN();return std::max(std::abs(a.lo),std::abs(a.hi));}
inline I widen(I a,double error){if(!good(a)||!std::isfinite(error)||error<0)return invalid();return checked({down(a.lo-error),up(a.hi+error)});}
inline I segmentDistance(I x,I y,const gp_Pnt2d& a,const gp_Pnt2d& b){
    const I dx=sub(I(b.X()),I(a.X())),dy=sub(I(b.Y()),I(a.Y()));
    const I length2=add(sq(dx),sq(dy));if(!good(length2)||length2.lo<=0)return {1,0};
    I t=div(add(mul(sub(x,I(a.X())),dx),mul(sub(y,I(a.Y())),dy)),length2);
    if(!good(t))return {1,0};t=minimum(I(1),maximum(I(0),t));
    return norm(sub(x,add(I(a.X()),mul(t,dx))),sub(y,add(I(a.Y()),mul(t,dy))));
}
inline bool inside(const std::vector<gp_Pnt2d>& points,double x,double y){
    // Half-open winding of the exact nominal point; every determinant sign is
    // interval-proven. Distance below independently covers its uncertainty disk.
    int winding=0;
    for(std::size_t n=0;n<points.size();++n){const auto&a=points[n];const auto&b=points[(n+1)%points.size()];
        if(!(a.Y()<=y&&b.Y()>y)&&!(a.Y()>y&&b.Y()<=y))continue;
        I cross=sub(mul(sub(I(b.X()),I(a.X())),sub(I(y),I(a.Y()))),mul(sub(I(b.Y()),I(a.Y())),sub(I(x),I(a.X()))));
        if(!good(cross)||(cross.lo<=0&&cross.hi>=0))return false;
        if(a.Y()<=y&&cross.lo>0)++winding;
        if(a.Y()>y&&cross.hi<0)--winding;
    }return winding!=0;
}
inline I roundedCavityDistance(I x,I y,const EnclosureDimensions& d){
    // Exact rounded-rectangle signed-distance formula includes all straight
    // sides and complete circular quadrants; not a centre/bounding-box test.
    I r=sub(I(d.cornerRadius),I(d.wall));
    I hx=sub(div(I(d.width),I(2)),I(d.wall)),hy=sub(div(I(d.depth),I(2)),I(d.wall));
    I qx=sub(abs(sub(x,div(I(d.width),I(2)))),sub(hx,r));
    I qy=sub(abs(sub(y,div(I(d.depth),I(2)))),sub(hy,r));
    return neg(sub(add(norm(maximum(qx,I(0)),maximum(qy,I(0))),minimum(maximum(qx,qy),I(0))),r));
}
}
inline Report Inspect(const retained_solid::Envelope& envelope) noexcept {
    using namespace detail; Report out;
    try {
        if(!retained_solid::Valid(envelope))return out;
        if(std::fegetround()!=FE_TONEAREST){out.status=Status::NumericUncertain;return out;}
        std::optional<profile::ConstructionFrame> frame;ProfileDefinition polygon;EnclosureDefinition enclosureDefinition;
        rectangular_loft::Definition loft;
        double bottom=0,thickness=0;int plane=-1;
        if(envelope.sourceFamily==1){
            profile::Parameters p;
            if(envelope.sourceSchema>2||!profile::Decode(envelope.sourceValues,p))return out;
            polygon=p.definition;frame=p.constructionFrame;plane=polygon.plane;thickness=polygon.depth;
            if(polygon.revolve||polygon.curves||!polygon.holes.empty()){
                out.status=Status::UnsupportedFamily;return out;}
        }else if(envelope.sourceFamily==2){
            enclosure::Parameters p;
            if(!enclosure::Decode(int(envelope.sourceSchema),envelope.sourceValues,p))return out;
            enclosureDefinition=p.definition;frame=enclosureDefinition.constructionFrame;plane=enclosureDefinition.plane;thickness=enclosureDefinition.dimensions.floor;
        }else if(envelope.sourceFamily==3){
            if(!loft_persistence::Decode(envelope.sourceValues,loft))return out;
            frame=loft.constructionFrame;plane=0;bottom=loft.stations.front().z;thickness=loft.stations.back().z;
        }else {out.status=Status::UnsupportedFamily;return out;}
        out.status=Status::UnsupportedFrame;
        gp_Trsf forward;
        if(frame&&(!frame->IsValid()||frame->values[7]<=0||!frame->Transform(forward)))return out;
        std::array<double,12>A{};for(int i=0;i<3;++i)for(int j=0;j<4;++j)A[i*4+j]=forward.Value(i+1,j+1);
        double measured=0;
        if(!cylindrical_cut::PositiveUniformSimilarity(A,measured)||forward.ScaleFactor()<=0)return out;
        const double scale=forward.ScaleFactor();out.sourceScale=scale;
        double forwardGramResidual=0;
        for(int i=0;i<3;++i){I row(0);for(int j=0;j<3;++j){I gram(0);
            for(int k=0;k<3;++k)gram=add(gram,mul(I(A[k*4+i]),I(A[k*4+j])));
            row=add(row,abs(sub(gram,i==j?sq(I(scale)):I(0))));}
            if(!good(row))return out;forwardGramResidual=std::max(forwardGramResidual,row.hi);}
        I scaleBounds=root(add(sq(I(scale)),I(-forwardGramResidual,forwardGramResidual)));
        I factor=mul(mul(I(envelope.metersPerUnit),I(1000)),scaleBounds);
        if(!good(factor)||factor.lo<=0)return out;
        // Exactly one inverse construction. No occurrence participates. Treat
        // returned inverse coefficients as rounded values and bound their
        // residual against the forward matrix before using them geometrically.
        const gp_Trsf inverse=forward.Inverted();double B[3][3]{};
        for(int i=0;i<3;++i)for(int j=0;j<3;++j){B[i][j]=inverse.Value(i+1,j+1);if(!std::isfinite(B[i][j]))return out;}
        double residual=0,bnorm=0,anorm=0;
        for(int i=0;i<3;++i){I row(0),bn(0),an(0);
            for(int j=0;j<3;++j){I product(0);for(int k=0;k<3;++k)product=add(product,mul(I(A[i*4+k]),I(B[k][j])));
                row=add(row,abs(sub(product,I(i==j?1:0))));bn=add(bn,I(std::abs(B[i][j])));an=add(an,I(std::abs(A[i*4+j])));}
            if(!good(row)||!good(bn)||!good(an))return out;
            residual=std::max(residual,row.hi);bnorm=std::max(bnorm,bn.hi);anorm=std::max(anorm,an.hi);
        }
        out.status=Status::NumericUncertain;
        // Neumann bound: ||A^-1-B|| <= ||B|| ||I-AB||/(1-||I-AB||).
        if(!(residual>=0&&residual<.5))return out;
        I inverseError=div(mul(I(bnorm),I(residual)),sub(I(1),I(residual)));
        if(!good(inverseError))return out;
        I delta[3];double maxDelta=0;
        for(int i=0;i<3;++i){delta[i]=sub(I(envelope.point[i]),I(A[i*4+3]));maxDelta=std::max(maxDelta,mag(delta[i]));}
        const I pointError=mul(I(inverseError.hi),I(maxDelta));if(!good(pointError))return out;
        I p[3];for(int i=0;i<3;++i){p[i]=I(0);for(int j=0;j<3;++j)p[i]=add(p[i],mul(I(B[i][j]),delta[j]));p[i]=widen(p[i],pointError.hi);if(!good(p[i]))return out;}
        std::array<int,3> chart=plane==0?std::array<int,3>{0,1,2}:plane==1?std::array<int,3>{0,2,1}:std::array<int,3>{1,2,0};
        I direction[3];for(int i=0;i<3;++i)direction[i]=widen(I(B[i][envelope.axis]),inverseError.hi);
        const double parallelLimit=up(512*std::numeric_limits<double>::epsilon()*std::max(1.0,up(anorm*bnorm)));
        // A transverse tool exits the two faces along its axis. Its strip must
        // retain material on the other station sides and both loft end caps.
        // Classify in the RECIPE frame, never using occurrence placement.
        if(envelope.sourceFamily==3)for(int axis=0;axis<2;++axis){
            const I along=abs(direction[axis]);
            const I across=norm(direction[1-axis],direction[2]);
            if(good(along)&&good(across)&&along.lo>0){
                const I ratio=div(across,along);
                if(good(ratio)&&ratio.hi<=parallelLimit){
                    out.transverseAxis=axis;chart={1-axis,2,axis};break;
                }
            }
        }
        I axial=abs(direction[chart[2]]),perp=norm(direction[chart[0]],direction[chart[1]]);
        if(!good(axial)||!good(perp)||axial.lo<=0){out.status=Status::Nonparallel;return out;}
        I slope=div(perp,axial);
        if(!good(slope)||slope.hi>parallelLimit){out.status=Status::Nonparallel;return out;}
        if(out.transverseAxis>=0){
            bottom=std::numeric_limits<double>::max();thickness=-bottom;
            for(const auto& station:loft.stations){
                const double center=out.transverseAxis==0?station.centerX:station.centerY;
                const double width=out.transverseAxis==0?station.width:station.depth;
                const I ends=add(I(center),mul(I(-.5,.5),I(width)));
                if(!good(ends))return out;
                bottom=std::min(bottom,ends.lo);thickness=std::max(thickness,ends.hi);
            }
        }
        // Numerically parallel rotations (e.g. a quaternion quarter-turn) have
        // a bounded nonzero slope. Cover the complete source-normal interval,
        // not just the tool point. Also cover the projected ellipse radius.
        I span=maximum(abs(sub(I(bottom),p[chart[2]])),abs(sub(I(thickness),p[chart[2]])));
        I radius=div(I(envelope.radius),I(scale));
        // Rounded gp_Trsf coefficients need not be algebraically orthogonal.
        // Bound the largest true inverse stretch using the Gram residual and
        // the inverse-entry error above, rather than assuming radius/s exact.
        I inverseScale2=div(I(1),sq(I(scale)));double gramResidual=0;
        for(int i=0;i<3;++i){I row(0);for(int j=0;j<3;++j){I gram(0);
            for(int k=0;k<3;++k)gram=add(gram,mul(widen(I(B[k][i]),inverseError.hi),widen(I(B[k][j]),inverseError.hi)));
            row=add(row,abs(sub(gram,i==j?inverseScale2:I(0))));}
            if(!good(row))return out;gramResidual=std::max(gramResidual,row.hi);}
        I radiusBound=mul(I(envelope.radius),root(add(inverseScale2,I(gramResidual))));
        I inflated=mul(radiusBound,root(add(I(1),sq(slope))));
        I angular=add(mul(span,slope),maximum(I(0),sub(inflated,radius)));
        if(!good(radius)||!good(angular)||radius.lo<=0)return out;
        I distance;
        if(out.transverseAxis>=0){
            // Conservative full-width strip over the entire circle's Z span,
            // including interpolated section boundaries BETWEEN stations.
            // Checking only authored stations misses a bore between stations.
            const I reach=add(radius,angular);
            const I low=sub(p[2],reach),high=add(p[2],reach);
            if(!good(low)||!good(high))return out;
            distance=minimum(sub(p[2],I(loft.stations.front().z)),sub(I(loft.stations.back().z),p[2]));
            for(std::size_t n=1;n<loft.stations.size();++n){
                const auto& a=loft.stations[n-1];const auto& b=loft.stations[n];
                const double lo=std::max(a.z,low.lo),hi=std::min(b.z,high.hi);
                if(lo>hi)continue;
                const int lateral=1-out.transverseAxis;
                const double ca=lateral==0?a.centerX:a.centerY,cb=lateral==0?b.centerX:b.centerY;
                const double wa=lateral==0?a.width:a.depth,wb=lateral==0?b.width:b.depth;
                for(double z:{lo,hi}){
                    const I t=div(sub(I(z),I(a.z)),sub(I(b.z),I(a.z)));
                    const I center=add(I(ca),mul(t,sub(I(cb),I(ca))));
                    const I half=div(add(I(wa),mul(t,sub(I(wb),I(wa)))),I(2));
                    distance=minimum(distance,sub(half,abs(sub(p[lateral],center))));
                }
            }
        }else if(envelope.sourceFamily==1&&polygon.circle){
            const auto& c=*polygon.circle;
            const I radial=norm(sub(p[chart[0]],I(c.center.X())),sub(p[chart[1]],I(c.center.Y())));
            if(!good(radial))return out;
            if(radial.hi>=c.outerRadius||(c.innerRadius>0&&radial.lo<=c.innerRadius)){
                out.status=Status::OutsideOrInsufficientLigament;return out;}
            distance=sub(I(c.outerRadius),radial);
            if(c.innerRadius>0)distance=minimum(distance,sub(radial,I(c.innerRadius)));
        }else if(envelope.sourceFamily==1){
            if(!inside(polygon.points,midpoint(p[chart[0]]),midpoint(p[chart[1]]))){out.status=Status::OutsideOrInsufficientLigament;return out;}
            distance=I(std::numeric_limits<double>::max());
            for(std::size_t n=0;n<polygon.points.size();++n){I d=segmentDistance(p[chart[0]],p[chart[1]],polygon.points[n],polygon.points[(n+1)%polygon.points.size()]);if(!good(d))return out;distance=minimum(distance,d);}
        }else if(envelope.sourceFamily==3){
            distance=I(std::numeric_limits<double>::max());
            for(const auto& s:loft.stations){
                const I dx=sub(div(I(s.width),I(2)),abs(sub(p[0],I(s.centerX))));
                const I dy=sub(div(I(s.depth),I(2)),abs(sub(p[1],I(s.centerY))));
                distance=minimum(distance,minimum(dx,dy));
            }
        }else distance=roundedCavityDistance(p[chart[0]],p[chart[1]],enclosureDefinition.dimensions);
        I physicalDistance=mul(distance,factor),physicalRadius=mul(I(envelope.radius),mul(I(envelope.metersPerUnit),I(1000)));
        I angularMM=mul(angular,factor),ligament=sub(sub(physicalDistance,physicalRadius),angularMM);
        if(!good(physicalDistance)||!good(physicalRadius)||!good(angularMM)||!good(ligament))return out;
        I uncertaintyBound=add(add(sub(I(physicalDistance.hi),I(physicalDistance.lo)),sub(I(physicalRadius.hi),I(physicalRadius.lo))),I(angularMM.hi));
        if(!good(uncertaintyBound))return out;const double uncertainty=uncertaintyBound.hi;
        out.sourcePlane=plane;out.recipeUnitToOriginalMM=midpoint(factor);out.radiusOriginalMM=midpoint(physicalRadius);
        out.centerRecipe={midpoint(p[chart[0]]),midpoint(p[chart[1]])};out.wallIntervalRecipe={bottom,thickness};
        out.boundaryDistanceLowerMM=physicalDistance.lo;out.boundaryDistanceUpperMM=physicalDistance.hi;
        out.ligamentLowerMM=ligament.lo;out.numericUncertaintyMM=uncertainty;out.angularSweepAllowanceMM=angularMM.hi;
        if(!std::isfinite(uncertainty)||uncertainty>MaximumNumericUncertaintyMM)return out;
        out.status=ligament.lo>KernelSeparationMM
            ?(out.transverseAxis>=0?Status::ClearRecipeTransverse:Status::ClearRecipeDisk)
            :Status::OutsideOrInsufficientLigament;
        return out;
    }catch(...){return Report{};}
}
// Independent integral of the removed material. At each Z the circle has a
// chord 2*sqrt(r*r-(z-cz)^2) and the ruled loft's axial width is affine.
// Admission above proves that the full chord is inside the other two walls.
inline bool TransverseRemovedVolume(const retained_solid::Envelope& envelope,double& volume) noexcept {
    volume=0;
    try {
        const auto clearance=Inspect(envelope);
        if(clearance.status!=Status::ClearRecipeTransverse)return false;
        rectangular_loft::Definition loft;
        if(!loft_persistence::Decode(envelope.sourceValues,loft))return false;
        gp_Trsf transform;
        if(loft.constructionFrame&&!loft.constructionFrame->Transform(transform))return false;
        const gp_Pnt center=gp_Pnt(envelope.point[0],envelope.point[1],envelope.point[2]).Transformed(transform.Inverted());
        const double scale=transform.ScaleFactor(),r=envelope.radius/scale,cz=center.Z();
        const auto area=[&](double t){
            t=std::clamp(t,-r,r);
            return t*std::sqrt(std::max(0.,r*r-t*t))+r*r*std::asin(t/r);
        };
        const auto moment=[&](double t){return -2./3.*std::pow(std::max(0.,r*r-t*t),1.5);};
        for(std::size_t n=1;n<loft.stations.size();++n){
            const auto& a=loft.stations[n-1];const auto& b=loft.stations[n];
            const double low=std::max(a.z,cz-r),high=std::min(b.z,cz+r);
            if(low>=high)continue;
            const double wa=clearance.transverseAxis==0?a.width:a.depth;
            const double wb=clearance.transverseAxis==0?b.width:b.depth;
            const double slope=(wb-wa)/(b.z-a.z),atCenter=wa+slope*(cz-a.z);
            volume+=atCenter*(area(high-cz)-area(low-cz))+slope*(moment(high-cz)-moment(low-cz));
        }
        volume*=scale*scale*scale;
        return std::isfinite(volume)&&volume>0;
    }catch(...){volume=0;return false;}
}
// Transverse openings intersect sloped station faces, so the disk observer's
// two circular cap loops are not applicable. Verify an exact detached replay
// from the retained base/tool, plus the independent ruled-width integral.
// Mesh caches are omitted from the byte comparison; geometry, topology,
// orientations, tolerances and all analytic representations remain included.
inline bool VerifyTransverseResult(const TopoDS_Shape& base,const TopoDS_Shape& result,
    const retained_solid::Envelope& envelope,const std::atomic_bool& stop,
    const TopoDS_Shape& replay) noexcept {
    try {
        double removed=0;
        if(stop.load()||!TransverseRemovedVolume(envelope,removed)
            ||base.IsNull()||result.IsNull()||replay.IsNull()
            ||result.ShapeType()!=TopAbs_SOLID||result.Orientation()!=TopAbs_FORWARD
            ||!BRepCheck_Analyzer(result,Standard_True).IsValid())return false;
        GProp_GProps before,after;
        BRepGProp::VolumeProperties(base,before,1e-12,Standard_True);
        BRepGProp::VolumeProperties(result,after,1e-12,Standard_True);
        if(!std::isfinite(before.Mass())||!std::isfinite(after.Mass())
            ||std::abs((before.Mass()-after.Mass())-removed)>removed*1e-9)return false;
        std::ostringstream actual,expected;
        actual.imbue(std::locale::classic());expected.imbue(std::locale::classic());
        BRepTools::Write(result,actual,Standard_False,Standard_False,TopTools_FormatVersion_VERSION_3);
        BRepTools::Write(replay,expected,Standard_False,Standard_False,TopTools_FormatVersion_VERSION_3);
        return !stop.load()&&actual.good()&&expected.good()&&actual.str()==expected.str();
    }catch(...){return false;}
}

}
