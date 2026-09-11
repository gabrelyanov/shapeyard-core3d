#pragma once

// Enclosure codec is distinct from profile schemas.
// Construction-frame/copy transforms require a deliberate successor codec and
// ownership implementation; schema1 never silently discards their data.
#include "EnclosureDefinition.hxx"
#include <vector>
namespace core3d::enclosure {
inline constexpr int SchemaVersion=1;
inline constexpr std::size_t ScalarCount=9;
struct Parameters {
    EnclosureDefinition definition;
    double metersPerUnit=0;
};
inline bool Encode(const Parameters& p,std::vector<double>& output) {
    output.clear();EnclosureDerivedDimensions inspected;
    if (!std::isfinite(p.metersPerUnit) || p.metersPerUnit<=0
        || !InspectEnclosureDefinition(p.definition,inspected)) return false;
    const auto& d=p.definition.dimensions;
    output={1.0,double(p.definition.plane),d.width,d.depth,d.height,d.wall,d.floor,d.cornerRadius,p.metersPerUnit};
    return true;
}
inline bool Decode(int schema,const std::vector<double>& values,Parameters& output) {
    output={};
    if (schema!=SchemaVersion || values.size()!=ScalarCount) return false;
    for (double value:values) if (!std::isfinite(value)) return false;
    if (values[0]!=1 || values[1]<0 || values[1]>2 || values[1]!=std::floor(values[1])) return false;
    Parameters candidate;candidate.definition.plane=int(values[1]);
    candidate.definition.dimensions={values[2],values[3],values[4],values[5],values[6],values[7]};
    candidate.metersPerUnit=values[8];std::vector<double> encoded;
    if (!Encode(candidate,encoded) || encoded!=values) return false;
    output=std::move(candidate);return true;
}
} // namespace core3d::enclosure
