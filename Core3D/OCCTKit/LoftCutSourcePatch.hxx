#pragma once
#include "RectangularLoftRebuild.hxx"
namespace core3d::saved_cut_source_values {
struct LoftPatch {
    rectangular_loft::ElementID stationIdentifier=0;
    std::optional<double> width,depth;
};
inline bool Valid(const LoftPatch& p) noexcept {
    return p.stationIdentifier&&(p.width||p.depth)
        &&(!p.width||(std::isfinite(*p.width)&&*p.width>0))
        &&(!p.depth||(std::isfinite(*p.depth)&&*p.depth>0));
}
// Preserve the exact encoded record, including all unaddressed signed bits.
inline bool Apply(const std::vector<double>& original,const LoftPatch& patch,std::vector<double>& out) noexcept {
    out.clear();try {
        rectangular_loft::Definition d;
        if(!Valid(patch)||!loft_persistence::Decode(original,d)
            ||(d.constructionFrame&&d.constructionFrame->values[7]<=0))return false;
        for(std::size_t i=0;i<d.stations.size();++i)if(d.stations[i].identifier==patch.stationIdentifier){
            auto candidate=original;const auto offset=8+14*i;
            if(patch.width&&*patch.width!=candidate[offset+12])candidate[offset+12]=*patch.width;
            if(patch.depth&&*patch.depth!=candidate[offset+13])candidate[offset+13]=*patch.depth;
            if(!loft_persistence::Decode(candidate,d))return false;out=std::move(candidate);return true;
        }return false;
    }catch(...){return false;}
}
}
