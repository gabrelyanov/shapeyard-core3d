#pragma once
#include "RectangularLoftPersistence.hxx"

namespace core3d::rectangular_loft {
// Raw feature units, immutable at the ObjC boundary. IDs address existing recipe
// stations, never OCAF labels or receipt authority. At least one dimension is set.
struct StationDimensionEdit {
    ElementID stationIdentifier=0;
    std::optional<double> width,depth;
};
}
namespace core3d::loft_rebuild {
inline bool Apply(const rectangular_loft::Definition& source,
                  const rectangular_loft::StationDimensionEdit& edit,
                  rectangular_loft::Definition& output) noexcept {
    output={};
    try {
        if (!edit.stationIdentifier || (!edit.width && !edit.depth)) return false;
        std::vector<double> original;
        if (!loft_persistence::Encode(source,original)) return false;
        auto candidate=source;bool found=false;
        for (auto& station:candidate.stations) if(station.identifier==edit.stationIdentifier) {
            if(found)return false;found=true;
            if(edit.width) {if(!std::isfinite(*edit.width)||*edit.width<=0)return false;
                if(*edit.width!=station.width)station.width=*edit.width;}
            if(edit.depth) {if(!std::isfinite(*edit.depth)||*edit.depth<=0)return false;
                if(*edit.depth!=station.depth)station.depth=*edit.depth;}
        }
        std::vector<double> encoded;
        if(!found || !loft_persistence::Encode(candidate,encoded))return false;
        output=std::move(candidate);return true;
    }catch(...){output={};return false;}
}
inline bool Matches(const rectangular_loft::Definition& source,
                    const rectangular_loft::StationDimensionEdit& edit,
                    const rectangular_loft::Definition& candidate) noexcept {
    rectangular_loft::Definition exact;std::vector<double> a,b;
    return Apply(source,edit,exact) && loft_persistence::Encode(exact,a)
        && loft_persistence::Encode(candidate,b) && loft_persistence::SameBits(a,b);
}
inline bool HasOnlyMetadataSubshapes(const Handle(TDocStd_Document)& document,
                                    const TDF_Label& owner) noexcept {
    try {
        loft_persistence::Record record;
        if(!loft_persistence::Read(document,owner,record)||record.label.IsNull())return false;
        TDF_LabelSequence children;XCAFDoc_ShapeTool::GetSubShapes(owner,children);
        if(children.Length()>profile::MaximumLabels)return false;
        for(int i=1;i<=children.Length();++i)if(!children.Value(i).IsEqual(record.label))return false;
        return true;
    }catch(...){return false;}
}
}
