#pragma once
// Stage A: closed numeric data/state only. No filesystem, Foundation, native
// handles, callbacks, geometry execution, positive query or reservation proof.
#include <CommonCrypto/CommonDigest.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <algorithm>
#include <string>
#include <vector>
#include <optional>

namespace core3d::request {
using UUID=std::array<std::uint8_t,16>;
using Digest=std::array<std::uint8_t,32>;
constexpr std::size_t MaximumBytes=65536,MaximumParts=16,MaximumValues=8192;
enum class Operation:std::uint8_t {CreateEnclosure=1,RebuildEnclosure=2,CreateAssembly=3,RebuildProfile=4,CreateProfile=5,RebuildLoftStation=6,SetPlacement=7};
enum class Recipe:std::uint8_t {Profile=1,Enclosure=2,RectangularLoft=3};
struct Part {Recipe recipe=Recipe::Profile;std::uint32_t schema=0;std::string name;std::vector<double> values;};
// V2 extension is numeric intent only, never executable native authority.
struct LoftStationEdit {std::uint32_t identifier=0;std::optional<double> width,depth;};
// V3 selected-object identity is distinct from an authored Part. Family5
// alone carries an absent feature UUID. All source digests are native-owned.
struct PlacementIntent {
    UUID entity{},definition{},feature{};
    Digest geometry{},recipe{},state{};
    std::uint8_t family=0,kind=0,axis=0;
    std::uint32_t schema=0;
    double metersPerUnit=0,value=0; // native-frozen model-unit position or degrees
};
struct Descriptor {Operation operation=Operation::CreateProfile;std::vector<Part> parts;std::optional<LoftStationEdit> loftStationEdit;std::optional<PlacementIntent> placement;};
struct Key {
    Digest accountScope{},command{},execution{};UUID document{},request{};
    bool operator==(const Key&b)const noexcept{return accountScope==b.accountScope&&command==b.command&&execution==b.execution&&document==b.document&&request==b.request;}
};
inline bool Nonzero(const auto&x){return std::any_of(x.begin(),x.end(),[](auto b){return b!=0;});}
inline bool ValidKey(const Key&k){return Nonzero(k.accountScope)&&Nonzero(k.command)&&Nonzero(k.execution)&&Nonzero(k.document)&&Nonzero(k.request);}
inline bool UTF8(const std::string&s,std::size_t limit,bool empty=false)noexcept{
    if(s.size()>limit||(!empty&&s.empty()))return false;
    for(std::size_t i=0;i<s.size();){const auto a=std::uint8_t(s[i++]);
        if(a<0x80){if(a==0||a<0x20||a==0x7f)return false;continue;}
        unsigned n=0;std::uint32_t value=0,minimum=0;
        if(a>=0xc2&&a<=0xdf){n=1;value=a&31;minimum=0x80;}
        else if(a>=0xe0&&a<=0xef){n=2;value=a&15;minimum=0x800;}
        else if(a>=0xf0&&a<=0xf4){n=3;value=a&7;minimum=0x10000;}else return false;
        if(n>s.size()-i)return false;while(n--){const auto b=std::uint8_t(s[i++]);if((b&0xc0)!=0x80)return false;value=(value<<6)|(b&63);}
        if(value<minimum||value>0x10ffff||(value>=0xd800&&value<=0xdfff)
            ||(value>=0x80&&value<=0x9f)||value==0x2028||value==0x2029)return false;
    }return true;
}
class Writer {
public:
    std::vector<std::uint8_t> bytes;bool valid=true;
    void raw(const std::uint8_t*p,std::size_t n){if(n==0)return;if(!valid||n>MaximumBytes-bytes.size()){valid=false;return;}bytes.insert(bytes.end(),p,p+n);}
    void integer(std::uint64_t value,unsigned n=8){if(n==0||n>8){valid=false;return;}std::uint8_t b[8];for(unsigned i=0;i<n;++i)b[i]=std::uint8_t(value>>(8*i));raw(b,n);}
    void text(const std::string&s){integer(s.size(),4);raw(reinterpret_cast<const std::uint8_t*>(s.data()),s.size());}
    void scalar(double value){if(!std::isfinite(value)){valid=false;return;}std::uint64_t b;std::memcpy(&b,&value,8);integer(b);}
};
inline bool Encode(const Descriptor&d,std::vector<std::uint8_t>&out)noexcept{
    out.clear();try{
        if(d.operation==Operation::SetPlacement){
            if(!d.parts.empty()||d.loftStationEdit||!d.placement)return false;
            const auto&p=*d.placement;
            if(!Nonzero(p.entity)||!Nonzero(p.definition)||!Nonzero(p.geometry)||!Nonzero(p.recipe)||!Nonzero(p.state)
                ||p.family<1||p.family>5||(p.family==5?Nonzero(p.feature)||p.schema!=0:!Nonzero(p.feature)||p.schema<1)
                ||(p.family==1&&p.schema>4)||(p.family==2&&p.schema>2)
                ||((p.family==3||p.family==4)&&p.schema!=1)
                ||p.kind>1||p.axis>2||!std::isfinite(p.metersPerUnit)||p.metersPerUnit<=0
                ||!std::isfinite(p.metersPerUnit*1000)||p.metersPerUnit*1000<=0
                ||!std::isfinite(p.value)||std::abs(p.value)>1e6)return false;
            Writer w;w.raw(reinterpret_cast<const std::uint8_t*>("SYMD"),4);
            w.integer(3,1);w.integer(7,1);w.integer(0,1);w.integer(0,1);
            w.raw(reinterpret_cast<const std::uint8_t*>("PLOB"),4);w.integer(1,1);
            for(const auto&id:{p.entity,p.definition,p.feature})w.raw(id.data(),id.size());
            for(const auto&hash:{p.geometry,p.recipe,p.state})w.raw(hash.data(),hash.size());
            w.integer(p.family,1);w.integer(p.schema,4);w.integer(p.kind,1);w.integer(p.axis,1);
            w.scalar(p.metersPerUnit);w.scalar(p.value);
            if(!w.valid)return false;out=std::move(w.bytes);return true;
        }
        if(d.placement)return false;
        const bool loft=d.operation==Operation::RebuildLoftStation;
        if(d.operation<Operation::CreateEnclosure||d.operation>Operation::RebuildLoftStation||d.parts.empty()||d.parts.size()>MaximumParts
            ||(d.operation!=Operation::CreateAssembly&&d.parts.size()!=1))return false;
        if(loft!=d.loftStationEdit.has_value())return false;
        if(loft){const auto&e=*d.loftStationEdit;
            if(!e.identifier||(!e.width&&!e.depth))return false;
            for(const auto&v:{e.width,e.depth})if(v&&(!std::isfinite(*v)||*v<=0||*v>1e6))return false;
        }
        Writer w;w.raw(reinterpret_cast<const std::uint8_t*>("SYMD"),4);w.integer(loft?2:1,1);w.integer(std::uint8_t(d.operation),1);w.integer(d.parts.size(),1);w.integer(0,1);
        for(const auto&p:d.parts){const auto expected=(d.operation==Operation::CreateEnclosure||d.operation==Operation::RebuildEnclosure)?Recipe::Enclosure:(loft?Recipe::RectangularLoft:Recipe::Profile);
            if(p.recipe!=expected||p.schema<1||p.schema>(p.recipe==Recipe::Profile?4u:(loft?1u:2u))||!UTF8(p.name,256,true)||p.values.empty()||p.values.size()>MaximumValues
                ||(d.operation!=Operation::CreateAssembly&&!p.name.empty()))return false;
            w.integer(std::uint8_t(p.recipe),1);w.integer(p.schema,4);w.text(p.name);w.integer(p.values.size(),4);for(double x:p.values)w.scalar(x);
        }
        if(loft){const auto&e=*d.loftStationEdit;
            if(d.parts.front().values.size()<36||d.parts.front().values.size()>128)return false;
            w.raw(reinterpret_cast<const std::uint8_t*>("LSED"),4);w.integer(1,1);w.integer(e.identifier,4);
            w.integer((e.width?1:0)|(e.depth?2:0),1);
            if(e.width)w.scalar(*e.width);if(e.depth)w.scalar(*e.depth);
        }
        if(!w.valid)return false;out=std::move(w.bytes);return true;
    }catch(...){out.clear();return false;}
}
inline bool Decode(const std::vector<std::uint8_t>&bytes,Descriptor&out)noexcept{
    out={};try{
        if(bytes.size()<8||bytes.size()>MaximumBytes||std::memcmp(bytes.data(),"SYMD",4)||(bytes[4]!=1&&bytes[4]!=2&&bytes[4]!=3)||bytes[7]!=0)return false;
        std::size_t cursor=8;auto integer=[&](unsigned n,std::uint64_t&x){x=0;if(n>bytes.size()-cursor)return false;for(unsigned i=0;i<n;++i)x|=std::uint64_t(bytes[cursor++])<<(8*i);return true;};
        Descriptor d;d.operation=Operation(bytes[5]);
        if(bytes[4]==3){
            if(bytes[5]!=7||bytes[6]!=0||bytes.size()!=180||std::memcmp(bytes.data()+8,"PLOB",4)||bytes[12]!=1)return false;
            cursor=13;PlacementIntent p;
            auto raw=[&](auto&v){if(v.size()>bytes.size()-cursor)return false;std::copy_n(bytes.begin()+cursor,v.size(),v.begin());cursor+=v.size();return true;};
            if(!raw(p.entity)||!raw(p.definition)||!raw(p.feature)||!raw(p.geometry)||!raw(p.recipe)||!raw(p.state))return false;
            std::uint64_t n;if(!integer(1,n))return false;p.family=std::uint8_t(n);
            if(!integer(4,n))return false;p.schema=std::uint32_t(n);
            if(!integer(1,n))return false;p.kind=std::uint8_t(n);
            if(!integer(1,n))return false;p.axis=std::uint8_t(n);
            if(!integer(8,n))return false;std::memcpy(&p.metersPerUnit,&n,8);
            if(!integer(8,n))return false;std::memcpy(&p.value,&n,8);d.placement=p;
            std::vector<std::uint8_t> canonical;if(cursor!=bytes.size()||!Encode(d,canonical)||canonical!=bytes)return false;
            out=std::move(d);return true;
        }
        if((bytes[4]==2)!=(d.operation==Operation::RebuildLoftStation))return false;
        const auto count=bytes[6];if(!count||count>MaximumParts)return false;
        for(unsigned i=0;i<count;++i){Part p;std::uint64_t n;
            if(!integer(1,n))return false;p.recipe=Recipe(n);if(!integer(4,n))return false;p.schema=std::uint32_t(n);
            if(!integer(4,n)||n>256||n>bytes.size()-cursor)return false;p.name.assign(reinterpret_cast<const char*>(bytes.data()+cursor),std::size_t(n));cursor+=n;
            if(!integer(4,n)||!n||n>MaximumValues||n>(bytes.size()-cursor)/8)return false;
            p.values.reserve(std::size_t(n));for(std::uint64_t j=0;j<n;++j){std::uint64_t b;if(!integer(8,b))return false;double x;std::memcpy(&x,&b,8);p.values.push_back(x);}d.parts.push_back(std::move(p));
        }
        if(bytes[4]==2){
            if(bytes.size()-cursor<10||std::memcmp(bytes.data()+cursor,"LSED",4)||bytes[cursor+4]!=1)return false;
            cursor+=5;std::uint64_t id,mask;if(!integer(4,id)||!integer(1,mask)||!id||mask<1||mask>3)return false;
            LoftStationEdit e;e.identifier=std::uint32_t(id);
            for(unsigned flag:{1u,2u})if(mask&flag){std::uint64_t bits;if(!integer(8,bits))return false;double value;std::memcpy(&value,&bits,8);
                if(flag==1)e.width=value;else e.depth=value;}
            d.loftStationEdit=e;
        }
        std::vector<std::uint8_t> canonical;if(cursor!=bytes.size()||!Encode(d,canonical)||canonical!=bytes)return false;out=std::move(d);return true;
    }catch(...){out={};return false;}
}
inline bool Hash(const char*domain,const std::vector<std::uint8_t>&bytes,Digest&out)noexcept{
    out.fill(0);if(bytes.empty()||bytes.size()>MaximumBytes)return false;
    CC_SHA256_CTX context;if(CC_SHA256_Init(&context)!=1||CC_SHA256_Update(&context,domain,CC_LONG(std::strlen(domain)))!=1
        ||CC_SHA256_Update(&context,bytes.data(),CC_LONG(bytes.size()))!=1||CC_SHA256_Final(out.data(),&context)!=1){out.fill(0);return false;}return true;
}
inline bool ScopeHash(const std::string&scope,Digest&out)noexcept{
    out.fill(0);try{if(!UTF8(scope,512))return false;
        return Hash("Shapeyard/host-namespace/v1",std::vector<std::uint8_t>(scope.begin(),scope.end()),out);
    }catch(...){return false;}
}
inline bool CommandHash(const Descriptor&d,Digest&out)noexcept{std::vector<std::uint8_t> bytes;out.fill(0);return Encode(d,bytes)&&Hash("Shapeyard/native-command/v1",bytes,out);}
inline bool ExecutionHash(const Digest&command,const Digest&account,const UUID&document,const UUID&request,
    const std::vector<std::uint8_t>&nativeAuthority,Digest&out)noexcept{
    out.fill(0);try{if(!Nonzero(command)||!Nonzero(account)||!Nonzero(document)||!Nonzero(request)||nativeAuthority.empty())return false;
        Writer w;w.raw(command.data(),command.size());w.raw(account.data(),account.size());w.raw(document.data(),document.size());w.raw(request.data(),request.size());w.integer(nativeAuthority.size(),4);w.raw(nativeAuthority.data(),nativeAuthority.size());
        return w.valid&&Hash("Shapeyard/native-execution/v1",w.bytes,out);
    }catch(...){return false;}
}
// Numeric continuation policy only. A State or Key is NOT executable native
// authority. Stage A has no caller that reserves, builds or commits from this.
enum class Phase {Prepared,Reserving,Reserved,Building,ReadyForOrdinary,Terminal};
enum class ReserveReply {FirstReserved,PreviouslySeen,Conflict,Capacity,Busy,Uncertain};
enum class Terminal {None,Cancelled,Rejected,PreviouslySeen,Conflict,Capacity,Busy,Uncertain,HandedToOrdinary};
class Admission final {
public:
    explicit Admission(Key key):key_(key){}
    bool begin()noexcept{if(phase_!=Phase::Prepared||!ValidKey(key_))return false;phase_=Phase::Reserving;return true;}
    bool reservation(const Key&key,ReserveReply reply)noexcept{
        if(phase_!=Phase::Reserving||!(key==key_))return false;
        switch(reply){case ReserveReply::FirstReserved:phase_=Phase::Reserved;return true;
            case ReserveReply::PreviouslySeen:return finish(Terminal::PreviouslySeen);
            case ReserveReply::Conflict:return finish(Terminal::Conflict);case ReserveReply::Capacity:return finish(Terminal::Capacity);
            case ReserveReply::Busy:return finish(Terminal::Busy);case ReserveReply::Uncertain:return finish(Terminal::Uncertain);}
        return finish(Terminal::Rejected);
    }
    bool beginGeometry(bool exactNativeFences)noexcept{if(phase_!=Phase::Reserved)return false;if(!exactNativeFences)return finish(Terminal::Rejected);phase_=Phase::Building;return true;}
    bool geometryReady(bool built,bool exactNativeFences)noexcept{if(phase_!=Phase::Building)return false;if(!built||!exactNativeFences)return finish(Terminal::Rejected);phase_=Phase::ReadyForOrdinary;return true;}
    bool takeForOrdinary(bool exactNativeFences)noexcept{if(phase_!=Phase::ReadyForOrdinary)return false;return finish(exactNativeFences?Terminal::HandedToOrdinary:Terminal::Rejected);}
    bool stop()noexcept{if(phase_==Phase::Terminal)return false;finish(Terminal::Cancelled);return true;}
    Phase phase()const noexcept{return phase_;}Terminal terminal()const noexcept{return terminal_;}
private:
    bool finish(Terminal result)noexcept{phase_=Phase::Terminal;terminal_=result;return result==Terminal::HandedToOrdinary;}
    Key key_;Phase phase_=Phase::Prepared;Terminal terminal_=Terminal::None;
};
}
