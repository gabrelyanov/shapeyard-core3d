#pragma once
#include "RetainedSolidAttribute.hxx"
#include "AnalyticBooleanOperand.hxx"
#include <limits>
#include <gp_Trsf.hxx>

#if DEBUG
#include <atomic>
#include <memory>
// One record per bridge operation, carried explicitly across the detached
// worker/delivery boundary. The TLS binding never owns document or UI state.
namespace core3d::cylindrical_cut {
struct DebugRefusalRecord {
    std::atomic<const char*> reason{nullptr};
    std::atomic<int> detail{-1};
};
inline thread_local std::shared_ptr<DebugRefusalRecord> debugRefusalRecord;
struct DebugRefusalScope {
    std::shared_ptr<DebugRefusalRecord> previous;
    explicit DebugRefusalScope(const std::shared_ptr<DebugRefusalRecord>& record)
        :previous(debugRefusalRecord) { debugRefusalRecord=record; }
    ~DebugRefusalScope() { debugRefusalRecord=previous; }
};
inline void DebugRefuse(const char* reason,int detail=-1) noexcept {
    if(debugRefusalRecord){
        const char* empty=nullptr;
        if(debugRefusalRecord->reason.compare_exchange_strong(empty,reason))
            debugRefusalRecord->detail.store(detail);
    }
}
}
#define CORE3D_CUT_NOTE(reason) ::core3d::cylindrical_cut::DebugRefuse(reason)
#define CORE3D_CUT_DETAIL(reason, detail) ::core3d::cylindrical_cut::DebugRefuse(reason, int(detail))
#else
#define CORE3D_CUT_DETAIL(reason, detail) ((void)0)
#define CORE3D_CUT_NOTE(reason) ((void)0)
#endif
#define CORE3D_CUT_STRINGIFY_IMPL(value) #value
#define CORE3D_CUT_STRINGIFY(value) CORE3D_CUT_STRINGIFY_IMPL(value)
#define CORE3D_CUT_REFUSE(reason, ...) do { CORE3D_CUT_NOTE(reason); return __VA_ARGS__; } while(false)

namespace core3d::cylindrical_cut {
// Native value input; identity/ownership is issued separately by the viewer.
// Center is explicitly in ORIGINAL object-local document units. Radius is a
// world-space physical millimetre dimension after the occurrence similarity.
struct CreateEdit {
    analytic_boolean::Axis axis=analytic_boolean::Axis::Z;
    std::array<double,3> localCenter{};
    double worldRadiusMM=0;
};
struct RadiusEdit { double worldRadiusMM=0; };
struct RingCreateEdit {
    analytic_boolean::Axis axis=analytic_boolean::Axis::Z;
    std::array<double,3> localCenter{};
    double boltCircleRadius=0,worldHoleRadiusMM=0;
    std::uint32_t count=0;
};
struct RingRadiusEdit {std::uint32_t identifier=0;double worldHoleRadiusMM=0;};
inline bool PositiveUniformSimilarity(const std::array<double,12>& matrix,double& scale)noexcept{
    scale=0;
    for(double value:matrix)if(!std::isfinite(value))return false;
    std::array<double,3> norms{};std::array<std::array<double,3>,3> columns{};
    for(unsigned c=0;c<3;++c){
        norms[c]=std::hypot(matrix[c],matrix[4+c],matrix[8+c]);
        if(!std::isfinite(norms[c])||norms[c]<=0)return false;
        for(unsigned r=0;r<3;++r)columns[c][r]=matrix[r*4+c]/norms[c];
    }
    constexpr double tolerance=256*std::numeric_limits<double>::epsilon();
    for(unsigned c=1;c<3;++c)if(std::abs(norms[c]/norms[0]-1)>tolerance)return false;
    for(unsigned a=0;a<3;++a)for(unsigned b=a+1;b<3;++b){
        double dot=0;for(unsigned r=0;r<3;++r)dot+=columns[a][r]*columns[b][r];
        if(std::abs(dot)>tolerance)return false;
    }
    const auto& a=columns[0];const auto& b=columns[1];const auto& c=columns[2];
    const double determinant=a[0]*(b[1]*c[2]-b[2]*c[1])-b[0]*(a[1]*c[2]-a[2]*c[1])+c[0]*(a[1]*b[2]-a[2]*b[1]);
    if(!std::isfinite(determinant)||determinant<=0||std::abs(determinant-1)>tolerance*4)return false;
    scale=norms[0];return true;
}
inline bool EffectiveMM(const gp_Trsf& transform,double units,double& scale,double& factor)noexcept{
    std::array<double,12> values{};for(int r=1;r<=3;++r)for(int c=1;c<=4;++c)values[(r-1)*4+c-1]=transform.Value(r,c);
    factor=0;const double mm=units*1000;
    if(!std::isfinite(units)||units<=0||!std::isfinite(mm)||mm<=0||!PositiveUniformSimilarity(values,scale))return false;
    factor=mm*scale;return std::isfinite(factor)&&factor>0;
}
// Occurrence-only placement scales the existing bore with the part. It does
// not rewrite the original-local tool or retained base. Keep its exposed
// physical radius inside the same supported admission domain as later edits.
inline bool OccurrenceRadius(const retained_solid::Envelope& envelope,const gp_Trsf& transform,double& worldMM)noexcept{
    worldMM=0;double scale=0,factor=0;
    if(!EffectiveMM(transform,envelope.metersPerUnit,scale,factor))return false;
    const double radius=envelope.radius*factor;
    if(!std::isfinite(radius)||radius<.001||radius>1e6)return false;
    worldMM=radius;return true;
}
// Stable public capability bit layout; only remove unsupported derived-owner
// construction routes. This does not grant AI prepared/receipt authority.
inline constexpr std::uint64_t UnsupportedOccurrenceCapabilities=
    (1ull<<1)|(1ull<<5)|(1ull<<7)|(1ull<<8)|(1ull<<9)|(1ull<<10)|(1ull<<11)
    |(1ull<<17)|(1ull<<18)|(1ull<<19);
inline bool Radius(double worldMM,double factor,double& local)noexcept{
    local=0;if(!std::isfinite(worldMM)||worldMM<.001||worldMM>1e6||!std::isfinite(factor)||factor<=0)return false;
    local=worldMM/factor;return std::isfinite(local)&&local>0;
}
inline analytic_boolean::Recipe Recipe(const retained_solid::Envelope& e){
    analytic_boolean::Recipe result;result.metersPerUnit=e.metersPerUnit;
    result.tool.identifier=e.operandID;result.tool.axis=analytic_boolean::Axis(e.axis);
    result.tool.point=e.point;result.tool.radius=e.radius;return result;
}
inline bool Rebuild(const retained_solid::Envelope& original,const RadiusEdit& edit,double factor,
                    retained_solid::Envelope& out)noexcept{
    out={};try{
        double local=0;if(!Radius(edit.worldRadiusMM,factor,local))return false;
        auto candidate=original;
        const double currentWorld=original.radius*factor;
        // The exact exposed current physical value is a no-op; preserve the
        // stored original radius instead of re-dividing a rounded product.
        if(edit.worldRadiusMM!=currentWorld)candidate.radius=local;
        std::vector<std::uint8_t> bytes;
        if(!retained_solid::Encode(candidate,bytes)||!analytic_boolean::Inspect(Recipe(candidate)))return false;
        out=std::move(candidate);return true;
    }catch(...){out={};return false;}
}
inline bool SameFixedEnvelope(const retained_solid::Envelope& a,const retained_solid::Envelope& b)noexcept{
    try {auto expected=b;expected.radius=a.radius;std::vector<std::uint8_t>x,y;
        return retained_solid::Encode(a,x)&&retained_solid::Encode(expected,y)&&x==y;
    }catch(...){return false;}
}
}

namespace core3d::wedge_cut {
// Positions stay object-local. Dimensions are physical mm after the selected
// occurrence's positive similarity, matching the cylindrical edit contract.
struct CreateEdit {
    analytic_boolean::Axis axis=analytic_boolean::Axis::Z;
    std::array<double,3> localApex{};
    double directionAngle=0,worldHalfWidthApexMM=0,worldHalfWidthMouthMM=0,worldLengthMM=0;
};
struct WidthsEdit {double worldHalfWidthApexMM=0,worldHalfWidthMouthMM=0;};
struct LengthEdit {double worldLengthMM=0;};
inline bool Dimension(double world,double factor,double minimum,double& local) noexcept {
    if(!std::isfinite(world)||world<minimum||world>1e6||!std::isfinite(factor)||factor<=0)return false;
    local=world/factor;return std::isfinite(local)&&local>0;
}
inline bool Widths(double apex,double mouth,double factor,double& a,double& b) noexcept {
    return Dimension(apex,factor,.05,a)&&Dimension(mouth,factor,.05,b);
}
}
