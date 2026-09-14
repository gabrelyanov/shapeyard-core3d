#pragma once
// Versioned values only. No owner, command, geometry or prepared AI authority.
// Legacy Encode/Decode remain the sole v1 codec and are not rewritten here.
#include "RetainedSolidEnvelope.hxx"
#include "AnalyticBooleanOperand.hxx"
#include <variant>
#include <optional>
#include <limits>

namespace core3d::retained_boolean {
using Legacy=retained_solid::Envelope;
using UUID=retained_solid::UUID;
inline constexpr std::size_t MaximumOperands=2; // Initial complete correspondence scope; not a tuple wire format.
struct Source {
    UUID document{},entity{},definition{},sourceFeature{},derivedFeature{};
    std::uint8_t family=0;
    std::uint32_t schema=0;
    double metersPerUnit=0;
    std::vector<double> values;
};
struct Step {
    analytic_boolean::Operation operation=analytic_boolean::Operation::Difference;
    analytic_boolean::Operand operand;
};
struct Program {
    Source source;
    // One-past high-water; UINT32_MAX+1 represents exhausted issuance, not wrap.
    std::uint64_t nextOperandID=1;
    std::vector<Step> steps;
};
using Recipe=std::variant<Legacy,Program>;
inline bool ValidSource(const Source& s) noexcept {
    try {
        using retained_solid::Nonzero;using retained_solid::Bits;
        if(!Nonzero(s.document)||!Nonzero(s.entity)||!Nonzero(s.definition)
            ||!Nonzero(s.sourceFeature)||!Nonzero(s.derivedFeature)||s.sourceFeature==s.derivedFeature
            ||!std::isfinite(s.metersPerUnit)||s.metersPerUnit<=0
            ||s.values.empty()||s.values.size()>std::size_t(profile::MaximumScalars))return false;
        const double mm=s.metersPerUnit*1000;if(!std::isfinite(mm)||mm<=0)return false;
        for(double value:s.values)if(!std::isfinite(value))return false;
        if(s.family==1){
            profile::Parameters p;return s.schema>=1&&s.schema<=4&&profile::Decode(s.values,p)
                &&profile::SchemaFor(p)==int(s.schema)&&Bits(p.metersPerUnit)==Bits(s.metersPerUnit);
        }
        if(s.family==2){
            enclosure::Parameters p;return s.schema>=1&&s.schema<=2&&enclosure::Decode(int(s.schema),s.values,p)
                &&Bits(p.metersPerUnit)==Bits(s.metersPerUnit);
        }
        return false;
    }catch(...){return false;}
}
inline bool Valid(const Program& p) noexcept {
    try {
        if(!ValidSource(p.source)||p.steps.empty()||p.steps.size()>MaximumOperands
            ||p.nextOperandID<2||p.nextOperandID>std::uint64_t(UINT32_MAX)+1)return false;
        for(std::size_t i=0;i<p.steps.size();++i){
            const auto& step=p.steps[i];analytic_boolean::Recipe r;
            r.metersPerUnit=p.source.metersPerUnit;r.operation=step.operation;r.tool=step.operand;
            if(!analytic_boolean::Inspect(r)||step.operand.identifier>=p.nextOperandID)return false;
            for(std::size_t j=0;j<i;++j)if(p.steps[j].operand.identifier==step.operand.identifier)return false;
        }
        // Checked complete size before Encode allocates, including digest.
        constexpr std::size_t fixed=10+80+5*8+32,stepBytes=4+8+4*8;
        return fixed<=retained_solid::MaximumEnvelopeBytes
            &&p.source.values.size()<=(retained_solid::MaximumEnvelopeBytes-fixed)/8
            &&p.steps.size()<=(retained_solid::MaximumEnvelopeBytes-fixed-p.source.values.size()*8)/stepBytes;
    }catch(...){return false;}
}
inline bool Encode(const Program& p,std::vector<std::uint8_t>& out) noexcept {
    out.clear();try {
        if(!Valid(p))return false;
        std::vector<std::uint8_t> bytes{'S','Y','R','S',2,1,1,1,p.source.family,0};
        bytes.reserve(10+80+5*8+p.source.values.size()*8+p.steps.size()*44+32);
        for(const auto& id:{p.source.document,p.source.entity,p.source.definition,p.source.sourceFeature,p.source.derivedFeature})
            bytes.insert(bytes.end(),id.begin(),id.end());
        using retained_solid::U64;using retained_solid::Bits;
        U64(bytes,p.source.schema);U64(bytes,Bits(p.source.metersPerUnit));U64(bytes,p.source.values.size());
        U64(bytes,p.nextOperandID);U64(bytes,p.steps.size());
        for(double value:p.source.values)U64(bytes,Bits(value));
        for(const auto& step:p.steps){
            bytes.push_back(std::uint8_t(step.operation));bytes.push_back(std::uint8_t(step.operand.kind));
            bytes.push_back(std::uint8_t(step.operand.extent));bytes.push_back(std::uint8_t(step.operand.axis));
            U64(bytes,step.operand.identifier);for(double value:step.operand.point)U64(bytes,Bits(value));
            U64(bytes,Bits(step.operand.radius));
        }
        retained_solid::Digest digest;if(!retained_solid::Hash(bytes,digest))return false;
        bytes.insert(bytes.end(),digest.begin(),digest.end());out=std::move(bytes);return true;
    }catch(...){out.clear();return false;}
}
inline bool Decode(const std::vector<std::uint8_t>& bytes,Program& out) noexcept {
    out={};try {
        constexpr std::size_t fixed=10+80+5*8,stepBytes=44;
        if(bytes.size()<fixed+8+stepBytes+32||bytes.size()>retained_solid::MaximumEnvelopeBytes
            ||std::memcmp(bytes.data(),"SYRS\2\1\1\1",8)!=0||bytes[9]!=0)return false;
        Program p;p.source.family=bytes[8];std::size_t at=10;
        for(auto* id:{&p.source.document,&p.source.entity,&p.source.definition,&p.source.sourceFeature,&p.source.derivedFeature}){
            std::copy_n(bytes.begin()+at,16,id->begin());at+=16;
        }
        auto integer=[&](){std::uint64_t v=0;for(unsigned i=0;i<8;++i)v|=std::uint64_t(bytes[at++])<<(8*i);return v;};
        auto scalar=[&](){auto bits=integer();double value;std::memcpy(&value,&bits,8);return value;};
        const auto schema=integer();if(schema>UINT32_MAX)return false;p.source.schema=std::uint32_t(schema);
        p.source.metersPerUnit=scalar();const auto count=integer();p.nextOperandID=integer();const auto steps=integer();
        if(!count||count>std::size_t(profile::MaximumScalars)||!steps||steps>MaximumOperands
            ||count>(bytes.size()-at-32)/8)return false;
        const std::size_t afterValues=at+std::size_t(count)*8;
        if(steps>(bytes.size()-afterValues-32)/stepBytes||bytes.size()!=afterValues+std::size_t(steps)*stepBytes+32)return false;
        p.source.values.reserve(std::size_t(count));for(std::uint64_t i=0;i<count;++i)p.source.values.push_back(scalar());
        p.steps.reserve(std::size_t(steps));for(std::uint64_t i=0;i<steps;++i){
            Step step;step.operation=analytic_boolean::Operation(bytes[at++]);step.operand.kind=analytic_boolean::OperandKind(bytes[at++]);
            step.operand.extent=analytic_boolean::Extent(bytes[at++]);step.operand.axis=analytic_boolean::Axis(bytes[at++]);
            const auto id=integer();if(id>UINT32_MAX)return false;step.operand.identifier=std::uint32_t(id);
            for(double& value:step.operand.point)value=scalar();step.operand.radius=scalar();p.steps.push_back(step);
        }
        std::vector<std::uint8_t> exact;if(!Encode(p,exact)||exact!=bytes)return false;
        out=std::move(p);return true;
    }catch(...){out={};return false;}
}
inline bool Encode(const Recipe& recipe,std::vector<std::uint8_t>& out) noexcept {
    try {
        if(const auto* legacy=std::get_if<Legacy>(&recipe))return retained_solid::Encode(*legacy,out);
        return Encode(std::get<Program>(recipe),out);
    }catch(...){out.clear();return false;}
}
inline bool Decode(const std::vector<std::uint8_t>& bytes,Recipe& out) noexcept {
    out=Legacy{};try {
        if(bytes.size()<8)return false;
        if(bytes[4]==1){Legacy legacy;if(!retained_solid::Decode(bytes,legacy))return false;out=std::move(legacy);return true;}
        if(bytes[4]==2){Program program;if(!Decode(bytes,program))return false;out=std::move(program);return true;}
        return false;
    }catch(...){out=Legacy{};return false;}
}
inline bool Promote(const Legacy& original,Program& out) noexcept {
    out={};try {
        if(!retained_solid::Valid(original))return false;
        Program p;p.source={original.document,original.entity,original.definition,original.sourceFeature,
            original.derivedFeature,original.sourceFamily,original.sourceSchema,original.metersPerUnit,original.sourceValues};
        Step first;first.operand.identifier=original.operandID;first.operand.axis=analytic_boolean::Axis(original.axis);
        first.operand.point=original.point;first.operand.radius=original.radius;p.steps.push_back(first);
        p.nextOperandID=std::uint64_t(original.operandID)+1;
        if(!Valid(p))return false;out=std::move(p);return true;
    }catch(...){out={};return false;}
}
// Common identity access contains no operand projection or geometry authority.
struct Identity {UUID document{},entity{},definition{},sourceFeature{},derivedFeature{};double metersPerUnit=0;};
inline Identity Identities(const Recipe& recipe){
    if(const auto* e=std::get_if<Legacy>(&recipe))return {e->document,e->entity,e->definition,e->sourceFeature,e->derivedFeature,e->metersPerUnit};
    const auto& e=std::get<Program>(recipe).source;return {e.document,e.entity,e.definition,e.sourceFeature,e.derivedFeature,e.metersPerUnit};
}
} // namespace core3d::retained_boolean
