#pragma once
#include "SavedCutEnclosureExtractor.hxx"
#include "RetainedSolidAttribute.hxx"
namespace core3d::enclosure_correspondence {
// Call only with a record obtained through the existing document owner reader.
// This does not replace that reader's document/label/namespace/receipt fences.
inline Classification InspectRetainedEnclosureBase(const retained_solid::Payload& payload,
    const std::atomic_bool& stop,Inspection& report)noexcept{
    report={};try{
        if(stop.load())return Classification::Cancelled;
        std::vector<std::uint8_t> encoded;
        if(!retained_solid::Encode(payload.envelope,encoded)||encoded!=payload.bytes
            ||payload.envelope.sourceFamily!=2)return Classification::Refused;
        enclosure::Parameters parameters;
        if(!enclosure::Decode(int(payload.envelope.sourceSchema),payload.envelope.sourceValues,parameters)
            ||retained_solid::Bits(parameters.metersPerUnit)!=retained_solid::Bits(payload.envelope.metersPerUnit))return Classification::Refused;
        return InspectEnclosure(payload.base,parameters,stop,report);
    }catch(...){return stop.load()?Classification::Cancelled:Classification::Refused;}
}
}
