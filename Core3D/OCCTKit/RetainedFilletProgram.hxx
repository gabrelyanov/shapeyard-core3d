#pragma once
// Value-only minor-4 tail. Radius/points are original-local document units;
// world millimetres enter only through typed edit admission.
#include "RetainedSolidEnvelope.hxx"
#include <set>
namespace core3d::retained_fillet {
enum class Outcome { Built, DeclinedRadiusAdmission, DeclinedAnchorNoMatch, DeclinedAnchorAmbiguous, DeclinedOcctFailure, Cancelled, DeclinedBudget, Generic, DeclinedUnsupportedEdge, DeclinedNonRemoving, DeclinedReplayIdentity };
enum class CurveKind:std::uint8_t { Line=1, Circle=2 };
struct EdgeAnchor {
    std::uint64_t identifier=0;
    CurveKind curveKind=CurveKind::Line;
    std::array<double,3> anchorPoint{},axis{{1,0,0}};
    double circleRadius=0;
};
// Discovery records are values only; no live geometry escapes the query.
inline constexpr std::size_t MaximumCandidates=64;
enum class CandidateStatus { Available, Unsupported, Stale, Budget, Failed };
struct Candidate { EdgeAnchor anchor; double lengthLocal=0; };
struct Candidates {
    CandidateStatus status=CandidateStatus::Unsupported;
    bool truncated=false;
    std::vector<Candidate> values;
};
struct Step {
    std::uint64_t stepIdentifier=0;
    double radiusLocal=0;
    std::vector<EdgeAnchor> anchors;
};
struct Program {
    std::uint64_t nextFilletStepID=1,nextFilletEdgeID=1;
    std::vector<Step> filletSteps;
};
inline constexpr std::size_t MaximumSteps=8,MaximumAnchors=16;
inline bool Dimension(double value,double mm){return std::isfinite(value)&&std::isfinite(value*mm)&&value*mm>=.001&&value*mm<=1e6;}
inline bool ValidAnchor(const EdgeAnchor& a,double mm){
    if(a.curveKind!=CurveKind::Line&&a.curveKind!=CurveKind::Circle)return false;
    for(double x:a.anchorPoint)if(!std::isfinite(x)||!std::isfinite(x*mm)||std::abs(x*mm)>1e6)return false;
    double norm=0;for(double x:a.axis){if(!std::isfinite(x))return false;norm+=x*x;}
    if(std::abs(norm-1)>1e-12)return false;
    return a.curveKind==CurveKind::Line?retained_solid::Bits(a.circleRadius)==0:Dimension(a.circleRadius,mm);
}
inline bool Valid(const Program& p,double mm) noexcept {
    try {
        if(!std::isfinite(mm)||mm<=0||p.filletSteps.size()>MaximumSteps
            ||!p.nextFilletStepID||!p.nextFilletEdgeID)return false;
        std::set<std::uint64_t> steps,edges;
        for(const auto& s:p.filletSteps){
            if(!s.stepIdentifier||s.stepIdentifier>=p.nextFilletStepID||!steps.insert(s.stepIdentifier).second
                ||!Dimension(s.radiusLocal,mm)||s.anchors.empty()||s.anchors.size()>MaximumAnchors)return false;
            for(const auto& a:s.anchors)if(!a.identifier||a.identifier>=p.nextFilletEdgeID
                ||!edges.insert(a.identifier).second||edges.size()>MaximumAnchors||!ValidAnchor(a,mm))return false;
        }return true;
    }catch(...){return false;}
}
inline std::size_t EncodedSize(const Program& p){
    std::size_t bytes=24;for(const auto& s:p.filletSteps)bytes+=24+65*s.anchors.size();return bytes;
}
// Persist both high-waters after count, even for an empty tail. Removal must
// never cause issuance reuse. Boolean bytes and all older minor layouts stay fixed.
inline bool Encode(const Program& p,double mm,std::vector<std::uint8_t>& out) noexcept {
    out.clear();try {
        if(!Valid(p,mm)||EncodedSize(p)>retained_solid::MaximumEnvelopeBytes)return false;
        using retained_solid::U64;using retained_solid::Bits;
        U64(out,p.filletSteps.size());U64(out,p.nextFilletStepID);U64(out,p.nextFilletEdgeID);
        for(const auto& s:p.filletSteps){U64(out,s.stepIdentifier);U64(out,Bits(s.radiusLocal));U64(out,s.anchors.size());
            for(const auto& a:s.anchors){U64(out,a.identifier);out.push_back(std::uint8_t(a.curveKind));
                for(double x:a.anchorPoint)U64(out,Bits(x));for(double x:a.axis)U64(out,Bits(x));U64(out,Bits(a.circleRadius));}}
        return true;
    }catch(...){out.clear();return false;}
}
inline bool Decode(const std::vector<std::uint8_t>& bytes,double mm,Program& out) noexcept {
    out={};try {
        if(bytes.size()<24||bytes.size()>retained_solid::MaximumEnvelopeBytes)return false;
        Program p;std::size_t at=0,total=0;
        const auto integer=[&](){std::uint64_t v=0;for(unsigned i=0;i<8;++i)v|=std::uint64_t(bytes.at(at++))<<(8*i);return v;};
        const auto scalar=[&](){const auto bits=integer();double x;std::memcpy(&x,&bits,8);return x;};
        const auto count=integer();p.nextFilletStepID=integer();p.nextFilletEdgeID=integer();if(count>MaximumSteps)return false;
        for(std::uint64_t i=0;i<count;++i){Step s;s.stepIdentifier=integer();s.radiusLocal=scalar();const auto n=integer();
            if(!n||n>MaximumAnchors-total||n>(bytes.size()-at)/65)return false;total+=std::size_t(n);
            for(std::uint64_t j=0;j<n;++j){EdgeAnchor a;a.identifier=integer();a.curveKind=CurveKind(bytes.at(at++));
                for(double& x:a.anchorPoint)x=scalar();for(double& x:a.axis)x=scalar();a.circleRadius=scalar();s.anchors.push_back(a);}
            p.filletSteps.push_back(std::move(s));}
        std::vector<std::uint8_t> exact;if(at!=bytes.size()||!Encode(p,mm,exact)||exact!=bytes)return false;
        out=std::move(p);return true;
    }catch(...){out={};return false;}
}
}
