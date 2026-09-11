#pragma once

// Enclosure records preserve legacy unframed values exactly. An explicitly
// stored frame, including identity, uses schema2 and is never erased on read.
#include "EnclosureDefinition.hxx"
#include <vector>
namespace core3d::enclosure {
inline constexpr int SchemaVersion=1;
inline constexpr int FramedSchemaVersion=2;
inline constexpr std::size_t ScalarCount=9;
inline constexpr std::size_t FramedScalarCount=17;
inline std::size_t ScalarCountForSchema(int schema) noexcept {
    return schema==SchemaVersion ? ScalarCount
        : schema==FramedSchemaVersion ? FramedScalarCount : 0;
}
struct Parameters {
    EnclosureDefinition definition;
    double metersPerUnit=0;
};
inline bool Encode(const Parameters& p,std::vector<double>& output) {
    output.clear();EnclosureDerivedDimensions inspected;
    if (!std::isfinite(p.metersPerUnit) || p.metersPerUnit<=0
        || !InspectEnclosureDefinition(p.definition,inspected)) return false;
    const auto& d=p.definition.dimensions;
    const int schema=p.definition.constructionFrame ? FramedSchemaVersion : SchemaVersion;
    output={double(schema),double(p.definition.plane),d.width,d.depth,d.height,d.wall,d.floor,d.cornerRadius,p.metersPerUnit};
    if (p.definition.constructionFrame) {
        const auto& frame=p.definition.constructionFrame->values;
        output.insert(output.end(),frame.begin(),frame.end());
    }
    return true;
}
inline bool Decode(int schema,const std::vector<double>& values,Parameters& output) {
    output={};
    const auto count=ScalarCountForSchema(schema);
    if (count==0 || values.size()!=count) return false;
    for (double value:values) if (!std::isfinite(value)) return false;
    if (values[0]!=double(schema) || values[1]<0 || values[1]>2 || values[1]!=std::floor(values[1])) return false;
    Parameters candidate;candidate.definition.plane=int(values[1]);
    candidate.definition.dimensions={values[2],values[3],values[4],values[5],values[6],values[7]};
    candidate.metersPerUnit=values[8];
    if (schema==FramedSchemaVersion) {
        candidate.definition.constructionFrame.emplace();
        std::copy(values.begin()+ScalarCount,values.end(),candidate.definition.constructionFrame->values.begin());
    }
    std::vector<double> encoded;
    if (!Encode(candidate,encoded) || encoded!=values) return false;
    output=std::move(candidate);return true;
}
} // namespace core3d::enclosure
