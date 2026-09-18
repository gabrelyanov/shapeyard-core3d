#pragma once
// Versioned values only. No owner, command, geometry or prepared AI authority.
// Legacy Encode/Decode remain the sole v1 codec and are not rewritten here.
#include "RetainedSolidEnvelope.hxx"
#include "AnalyticBooleanRingOperand.hxx"
#include <variant>
#include <optional>
#include <limits>

namespace core3d::retained_boolean {
using Legacy=retained_solid::Envelope;
using UUID=retained_solid::UUID;
inline constexpr std::size_t MaximumOperands=4; // Retained operands; expanded disks have their own independent bound.
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
    std::uint8_t codecMinor=1; // Preserve v1 program bytes until a ring is appended.
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
        if(s.family==3){
            rectangular_loft::Definition d;
            return s.schema==loft_persistence::Schema&&loft_persistence::Decode(s.values,d)
                &&Bits(d.dimensionMetersPerUnit)==Bits(s.metersPerUnit);
        }
        return false;
    }catch(...){return false;}
}
inline bool Valid(const Program& p) noexcept {
    try {
        if((p.codecMinor!=1&&p.codecMinor!=2)||!ValidSource(p.source)||p.steps.empty()||p.steps.size()>MaximumOperands
            ||p.nextOperandID<2||p.nextOperandID>std::uint64_t(UINT32_MAX)+1)return false;
        std::size_t disks=0;
        for(std::size_t i=0;i<p.steps.size();++i){
            const auto& step=p.steps[i];analytic_boolean::Recipe r;
            r.metersPerUnit=p.source.metersPerUnit;r.operation=step.operation;r.tool=step.operand;
            if(!analytic_boolean::Inspect(r)||step.operand.identifier>=p.nextOperandID)return false;
            if(step.operand.kind==analytic_boolean::OperandKind::CylinderRing){
                if(p.codecMinor!=2||analytic_boolean_ring::Inspect(analytic_boolean_ring::FromOperand(step.operand,p.source.metersPerUnit),
                    p.source.metersPerUnit)!=analytic_boolean_ring::Status::Clear)return false;
                disks+=step.operand.count;
            }else ++disks;
            if(disks>analytic_boolean_ring::kMaximumExpandedDisks)return false;
            for(std::size_t j=0;j<i;++j)if(p.steps[j].operand.identifier==step.operand.identifier)return false;
        }
        // Checked complete size before Encode allocates, including digest.
        constexpr std::size_t fixed=10+80+5*8+32;const std::size_t stepBytes=p.codecMinor==1?44:64;
        return fixed<=retained_solid::MaximumEnvelopeBytes
            &&p.source.values.size()<=(retained_solid::MaximumEnvelopeBytes-fixed)/8
            &&p.steps.size()<=(retained_solid::MaximumEnvelopeBytes-fixed-p.source.values.size()*8)/stepBytes;
    }catch(...){return false;}
}
struct Disk {analytic_boolean::Operand operand;std::size_t step=0;std::uint32_t ordinal=0;};
inline bool ExpandedDisks(const Program& p,std::vector<Disk>& out) noexcept {
    out.clear();try {
        if(!Valid(p))return false;
        for(std::size_t i=0;i<p.steps.size();++i){const auto& t=p.steps[i].operand;
            if(t.kind==analytic_boolean::OperandKind::Cylinder){out.push_back({t,i,0});continue;}
            const auto ring=analytic_boolean_ring::FromOperand(t,p.source.metersPerUnit);
            for(std::uint32_t k=0;k<t.count;++k){auto disk=analytic_boolean_ring::Expand(ring,k,p.source.metersPerUnit);
                if(!disk.identifier){out.clear();return false;}disk.radius=t.radius;out.push_back({disk,i,k});}
        }
        return !out.empty()&&out.size()<=analytic_boolean_ring::kMaximumExpandedDisks;
    }catch(...){out.clear();return false;}
}
inline bool Encode(const Program& p,std::vector<std::uint8_t>& out) noexcept {
    out.clear();try {
        if(!Valid(p))return false;
        std::vector<std::uint8_t> bytes{'S','Y','R','S',2,1,1,p.codecMinor,p.source.family,0};
        bytes.reserve(10+80+5*8+p.source.values.size()*8+p.steps.size()*(p.codecMinor==1?44:64)+32);
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
            if(p.codecMinor==2){U64(bytes,Bits(step.operand.boltCircleRadius));
                for(unsigned k=0;k<4;++k)bytes.push_back(std::uint8_t(step.operand.count>>(8*k)));
                U64(bytes,Bits(step.operand.hostRadiusRatio));}
        }
        retained_solid::Digest digest;if(!retained_solid::Hash(bytes,digest))return false;
        bytes.insert(bytes.end(),digest.begin(),digest.end());out=std::move(bytes);return true;
    }catch(...){out.clear();return false;}
}
inline bool Decode(const std::vector<std::uint8_t>& bytes,Program& out) noexcept {
    out={};try {
        constexpr std::size_t fixed=10+80+5*8;const std::size_t stepBytes=bytes.size()>7&&bytes[7]==2?64:44;
        if(bytes.size()<fixed+8+stepBytes+32||bytes.size()>retained_solid::MaximumEnvelopeBytes
            ||std::memcmp(bytes.data(),"SYRS\2\1\1",7)!=0||(bytes[7]!=1&&bytes[7]!=2)||bytes[9]!=0)return false;
        Program p;p.codecMinor=bytes[7];p.source.family=bytes[8];std::size_t at=10;
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
            for(double& value:step.operand.point)value=scalar();step.operand.radius=scalar();
            if(p.codecMinor==2){step.operand.boltCircleRadius=scalar();
                for(unsigned k=0;k<4;++k)step.operand.count|=std::uint32_t(bytes[at++])<<(8*k);
                step.operand.hostRadiusRatio=scalar();}
            p.steps.push_back(step);
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
