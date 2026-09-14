#pragma once
// One native retained solid, encoded through the DOCUMENT'S existing shape set.
// This foundation grants no ordinary edit, prepared-request or AI authority.
#include "RectangularLoftPersistence.hxx"
#include <TDF_Attribute.hxx>
#include <TDF_RelocationTable.hxx>
#include <CommonCrypto/CommonDigest.h>
#include <memory>
#include <array>
#include <cstring>
#include <vector>
#include <cstdint>
#include <TDF_LabelMap.hxx>
#include <Standard_Failure.hxx>

class OcctDocument;
namespace core3d::retained_solid {
class BinaryDriver;
#if DEBUG
struct Probe;
#endif
inline constexpr std::size_t MaximumEnvelopeBytes=64*1024;
inline constexpr std::size_t MaximumAggregateEnvelopeBytes=8*1024*1024;
inline constexpr int MinimumRecordTag=13;
inline const Standard_GUID& AttributeID(){
    static const Standard_GUID id("72997251-32AF-4771-B094-8D097B41B7D2");return id;
}
using UUID=std::array<std::uint8_t,16>;
using Digest=std::array<std::uint8_t,32>;
inline bool Nonzero(const UUID& id){return std::any_of(id.begin(),id.end(),[](auto v){return v!=0;});}
inline std::uint64_t Bits(double value){std::uint64_t result;std::memcpy(&result,&value,8);return result;}
struct Envelope {
    UUID document{},entity{},definition{},sourceFeature{},derivedFeature{};
    // First schema retains a profile/enclosure base and one cylindrical
    // through-all Difference operand, expressed on object-local X/Y/Z.
    std::uint8_t sourceFamily=0,axis=2;
    std::uint32_t sourceSchema=0,operandID=1;
    double metersPerUnit=0,radius=0;
    std::array<double,3> point{};
    std::vector<double> sourceValues;
};
inline bool Valid(const Envelope& e){
    if(!Nonzero(e.document)||!Nonzero(e.entity)||!Nonzero(e.definition)
        ||!Nonzero(e.sourceFeature)||!Nonzero(e.derivedFeature)||e.sourceFeature==e.derivedFeature
        ||e.axis>2||!e.operandID||!std::isfinite(e.metersPerUnit)||e.metersPerUnit<=0
        ||e.sourceValues.empty()||e.sourceValues.size()>std::size_t(profile::MaximumScalars))return false;
    const double mm=e.metersPerUnit*1000;
    if(!std::isfinite(mm)||mm<=0||!std::isfinite(e.radius)||e.radius<=0
        ||!std::isfinite(e.radius*mm)||e.radius*mm<.001||e.radius*mm>1e6)return false;
    for(double p:e.point)if(!std::isfinite(p)||!std::isfinite(p*mm)||std::abs(p*mm)>1e6)return false;
    for(double v:e.sourceValues)if(!std::isfinite(v))return false;
    if(e.sourceFamily==1){
        if(e.sourceSchema<1||e.sourceSchema>4)return false;
        profile::Parameters p;
        return profile::Decode(e.sourceValues,p)&&profile::SchemaFor(p)==int(e.sourceSchema)&&Bits(p.metersPerUnit)==Bits(e.metersPerUnit);
    }
    if(e.sourceFamily==2){
        if(e.sourceSchema<1||e.sourceSchema>2)return false;
        enclosure::Parameters p;
        return enclosure::Decode(int(e.sourceSchema),e.sourceValues,p)&&Bits(p.metersPerUnit)==Bits(e.metersPerUnit);
    }
    return false;
}
inline void U64(std::vector<std::uint8_t>& out,std::uint64_t value){for(unsigned i=0;i<8;++i)out.push_back(std::uint8_t(value>>(8*i)));}
inline bool Hash(const std::vector<std::uint8_t>& bytes,Digest& out){
    return bytes.size()<=MaximumEnvelopeBytes&&CC_SHA256(bytes.data(),CC_LONG(bytes.size()),out.data())!=nullptr;
}
inline bool Encode(const Envelope& e,std::vector<std::uint8_t>& out){
    out.clear();try{
        if(!Valid(e))return false;
        std::vector<std::uint8_t> b{'S','Y','R','S',1,1,1,1,e.sourceFamily,e.axis};
        for(const auto& id:{e.document,e.entity,e.definition,e.sourceFeature,e.derivedFeature})b.insert(b.end(),id.begin(),id.end());
        U64(b,e.sourceSchema);U64(b,e.operandID);U64(b,Bits(e.metersPerUnit));
        for(double p:e.point)U64(b,Bits(p));U64(b,Bits(e.radius));U64(b,e.sourceValues.size());
        for(double v:e.sourceValues)U64(b,Bits(v));
        if(b.size()>MaximumEnvelopeBytes-32)return false;Digest digest;if(!Hash(b,digest))return false;
        b.insert(b.end(),digest.begin(),digest.end());out=std::move(b);return true;
    }catch(...){out.clear();return false;}
}
inline bool Decode(const std::vector<std::uint8_t>& bytes,Envelope& out){
    out={};try{
        constexpr std::size_t prefix=10+80+8*8;
        if(bytes.size()<prefix+8+32||bytes.size()>MaximumEnvelopeBytes
            ||std::memcmp(bytes.data(),"SYRS\1\1\1\1",8)!=0)return false;
        std::size_t at=10;Envelope e;e.sourceFamily=bytes[8];e.axis=bytes[9];
        for(auto* id:{&e.document,&e.entity,&e.definition,&e.sourceFeature,&e.derivedFeature}){
            std::copy_n(bytes.begin()+at,16,id->begin());at+=16;
        }
        auto integer=[&](){std::uint64_t v=0;for(unsigned i=0;i<8;++i)v|=std::uint64_t(bytes[at++])<<(8*i);return v;};
        auto scalar=[&](){auto bits=integer();double v;std::memcpy(&v,&bits,8);return v;};
        const auto schema=integer(),operand=integer();
        if(schema>UINT32_MAX||operand>UINT32_MAX)return false;e.sourceSchema=std::uint32_t(schema);e.operandID=std::uint32_t(operand);
        e.metersPerUnit=scalar();for(double& p:e.point)p=scalar();e.radius=scalar();const auto count=integer();
        if(count>std::size_t(profile::MaximumScalars)||count>(bytes.size()-at-32)/8||bytes.size()!=at+8*count+32)return false;
        e.sourceValues.reserve(std::size_t(count));for(std::uint64_t i=0;i<count;++i)e.sourceValues.push_back(scalar());
        std::vector<std::uint8_t> expected;if(!Encode(e,expected)||expected!=bytes)return false;
        out=std::move(e);return true;
    }catch(...){out={};return false;}
}
} // namespace core3d::retained_solid
