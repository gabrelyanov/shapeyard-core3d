#pragma once
// Exact existing receipt grammar, separated to share it with the binary driver.
#include <Standard_GUID.hxx>
#include <CommonCrypto/CommonDigest.h>
#include <array>
#include <algorithm>
#include <cstring>
#include <cstdint>
#include <iterator>
#include <set>
#include <string>
#include <vector>
namespace core3d::receipt {
inline const Standard_GUID& ScalableSchemaID(){static const Standard_GUID id("B8DA7821-DAB0-46EB-84F2-914A523AEF63");return id;}
using Digest=std::array<std::uint8_t,32>;
using UUID=std::array<std::uint8_t,16>;
constexpr std::size_t MaximumRecords=128, MaximumEffects=16, MaximumBytes=256*1024;
constexpr std::size_t ChunkBytes=256, MaximumChunks=MaximumBytes*2/ChunkBytes;
inline const Standard_GUID& SchemaID(){ static const Standard_GUID id("9FB0D315-83AB-4F68-AE29-C8C85BA670C1");return id; }
inline const Standard_GUID& CountID(){ static const Standard_GUID id("9FB0D315-83AB-4F68-AE29-C8C85BA670C2");return id; }
// Policy1 freezes the exact qualified417 OCCT7.8/V3 analytic-zero procedure.
// Never retarget an assigned policy. Zero denotes unversioned legacy only.
constexpr std::uint16_t AnalyticZeroPolicy1=1;
// Separate policy namespace for exact OCCT7.8/V3 loft representation. No
// numeric zero rewriting. Initial/reopen phase compatibility is not presumed.
constexpr std::uint16_t ExactLoftPolicy4097=4097;
// Placement state v1 + exact OCCT V3. Separate from both prior policies.
constexpr std::uint16_t ExactPlacementPolicy8193=8193;
// Exact independently reconstructed initial/native-readback V3 fillet geometry.
// No observed-shape rounding, no retargeting of an older policy.
constexpr std::uint16_t ExactRetainedFilletPolicy12289=12289;
inline const Standard_GUID& VersionedSchemaID(){static const Standard_GUID id("42AB4BB7-6421-445F-A2B0-2DC682AB3001");return id;}
inline const Standard_GUID& VersionedCountID(){static const Standard_GUID id("42AB4BB7-6421-445F-A2B0-2DC682AB3002");return id;}
enum class EffectEvidenceStatus { Current, Mismatch, LegacyUnversioned, UnsupportedPolicy, Unavailable };
enum class Operation:std::uint8_t { CreateEnclosure=1, RebuildEnclosure=2, CreateAssembly=3, RebuildProfile=4, RebuildLoftStation=6, SetPlacement=7, RebuildRetainedFillet=8 };
enum class Feature:std::uint8_t { Profile=1, Enclosure=2, RectangularLoft=3, PlacementObject=4, PlacementBareSolid=5, RetainedSolid=6 };
inline bool SupportedPolicy(Operation operation,std::uint16_t policy)noexcept {
    return operation==Operation::RebuildRetainedFillet?policy==ExactRetainedFilletPolicy12289:operation==Operation::SetPlacement?policy==ExactPlacementPolicy8193:operation==Operation::RebuildLoftStation?policy==ExactLoftPolicy4097:policy==AnalyticZeroPolicy1;
}
enum class ReadStatus { Absent, Valid, Unsupported, Malformed, Unavailable };
enum class DocumentPresence { Absent, Present, Conflict, Unavailable };
enum class VerifiedQueryStatus { Unavailable };
// Intentionally no BOOL that can turn a file record into native authority.
inline VerifiedQueryStatus QueryVerifiedReceipt() noexcept { return VerifiedQueryStatus::Unavailable; }
struct Key {
    Digest accountScope{}, command{}, execution{};
    UUID document{}, request{};
    bool operator==(const Key& b) const {return accountScope==b.accountScope&&command==b.command&&execution==b.execution&&document==b.document&&request==b.request;}
};
struct Effect {
    Feature feature=Feature::Profile;
    UUID entity{}, definition{}, featureID{};
    Digest geometry{}, state{};
    std::uint16_t policy=AnalyticZeroPolicy1;
    bool operator==(const Effect& b)const{return policy==b.policy&&feature==b.feature&&entity==b.entity&&definition==b.definition&&featureID==b.featureID&&geometry==b.geometry&&state==b.state;}
};
struct Record {Key key;Operation operation=Operation::CreateAssembly;std::vector<Effect> effects;std::uint16_t policy=AnalyticZeroPolicy1;};
struct Inspection {DocumentPresence presence=DocumentPresence::Unavailable;bool effectsCurrent=false;std::vector<Effect> effects;EffectEvidenceStatus evidence=EffectEvidenceStatus::Unavailable;};
inline bool Nonzero(const auto& a){return std::any_of(a.begin(),a.end(),[](auto b){return b!=0;});}
inline int Nibble(char c){return c>='0'&&c<='9'?c-'0':c>='A'&&c<='F'?c-'A'+10:c>='a'&&c<='f'?c-'a'+10:-1;}
inline bool ParseUUID(const std::string& text,UUID& out){
    out.fill(0);if(text.size()!=36||!Standard_GUID::CheckGUIDFormat(text.c_str()))return false;
    std::size_t n=0;int high=-1;
    for(char c:text){if(c=='-')continue;const int x=Nibble(c);if(x<0)return false;if(high<0)high=x;else{if(n>=out.size())return false;out[n++]=std::uint8_t(high*16+x);high=-1;}}
    return n==16&&high<0&&Nonzero(out);
}
inline std::string Hex(const auto& bytes){static const char chars[]="0123456789abcdef";std::string s;s.reserve(bytes.size()*2);for(auto b:bytes){s+=chars[b>>4];s+=chars[b&15];}return s;}
inline bool Hash(const std::uint8_t* p,std::size_t n,Digest& out){out.fill(0);return n<=MaximumBytes&&CC_SHA256(p,static_cast<CC_LONG>(n),out.data())!=nullptr;}
inline bool Valid(const Record& r){
    if(!Nonzero(r.key.accountScope)||!Nonzero(r.key.command)||!Nonzero(r.key.execution)||!Nonzero(r.key.document)||!Nonzero(r.key.request)
        ||r.effects.empty()||r.effects.size()>MaximumEffects)return false;
    const bool placement=r.operation==Operation::SetPlacement;
    const bool fillet=r.operation==Operation::RebuildRetainedFillet;
    const bool loft=r.operation==Operation::RebuildLoftStation;
    if((!loft&&!placement&&!fillet&&(r.operation<Operation::CreateEnclosure||r.operation>Operation::RebuildProfile))
        ||(r.operation!=Operation::CreateAssembly&&r.effects.size()!=1)
        ||(fillet&&(r.policy==0||r.policy==AnalyticZeroPolicy1||r.policy==ExactLoftPolicy4097||r.policy==ExactPlacementPolicy8193))
        ||(loft&&(r.policy==0||r.policy==AnalyticZeroPolicy1))
        ||(placement&&(r.policy==0||r.policy==AnalyticZeroPolicy1||r.policy==ExactLoftPolicy4097)))return false;
    // An assigned loft policy cannot retroactively invalidate unknown-policy
    // records on older operations. SupportedPolicy keeps them unresolved and
    // nonappendable without changing their structural bytes.
    std::set<UUID> entities,definitions,features;
    for(const auto& e:r.effects){
        const auto expected=(r.operation==Operation::CreateEnclosure||r.operation==Operation::RebuildEnclosure)?Feature::Enclosure:(fillet?Feature::RetainedSolid:loft?Feature::RectangularLoft:Feature::Profile);
        const bool bare=placement&&e.feature==Feature::PlacementBareSolid;
        if(e.policy!=r.policy||(placement?(e.feature!=Feature::PlacementObject&&!bare):e.feature!=expected)
            ||!Nonzero(e.entity)||!Nonzero(e.definition)||(bare?Nonzero(e.featureID):!Nonzero(e.featureID))||!Nonzero(e.geometry)||!Nonzero(e.state)
            ||!entities.insert(e.entity).second||!definitions.insert(e.definition).second||!features.insert(e.featureID).second)return false;
    }
    return true;
}
inline bool Encode(std::vector<Record> records,std::vector<std::uint8_t>& output) noexcept {
    output.clear();std::vector<std::uint8_t> out;try{
        if(records.empty()||records.size()>MaximumRecords)return false;
        std::sort(records.begin(),records.end(),[](const auto&a,const auto&b){return a.key.request<b.key.request;});
        out={'S','Y','R','C',2,0,std::uint8_t(records.size()),std::uint8_t(records.size()>>8)};
        auto append=[&](const auto& a){out.insert(out.end(),a.begin(),a.end());};
        UUID previous{};
        for(auto&r:records){
            if(!Valid(r)||r.policy==0||r.key.request==previous)return false;previous=r.key.request;
            append(r.key.accountScope);append(r.key.document);append(r.key.request);append(r.key.command);append(r.key.execution);
            out.push_back(std::uint8_t(r.operation));out.push_back(std::uint8_t(r.effects.size()));
            out.push_back(std::uint8_t(r.policy));out.push_back(std::uint8_t(r.policy>>8));
            std::sort(r.effects.begin(),r.effects.end(),[](const auto&a,const auto&b){return a.entity<b.entity;});
            for(const auto&e:r.effects){out.push_back(std::uint8_t(e.feature));append(e.entity);append(e.definition);append(e.featureID);append(e.geometry);append(e.state);}
            if(out.size()>MaximumBytes-32)return false;
        }
        Digest digest;if(!Hash(out.data(),out.size(),digest))return false;append(digest);output=std::move(out);return true;
    }catch(...){out.clear();return false;}
}
inline ReadStatus Decode(const std::vector<std::uint8_t>& bytes,std::vector<Record>& output) noexcept {
    output.clear();std::vector<Record> out;try{
        if(bytes.size()<40||bytes.size()>MaximumBytes||std::memcmp(bytes.data(),"SYRC",4))return ReadStatus::Malformed;
        if(bytes[4]!=1&&bytes[4]!=2)return ReadStatus::Unsupported;
        const bool legacy=bytes[4]==1;
        if(bytes[5]!=0)return ReadStatus::Malformed;
        Digest actual,expected;std::copy(bytes.end()-32,bytes.end(),expected.begin());
        if(!Hash(bytes.data(),bytes.size()-32,actual)||actual!=expected)return ReadStatus::Malformed;
        const auto count=std::size_t(bytes[6])+(std::size_t(bytes[7])<<8);
        if(!count||count>MaximumRecords)return ReadStatus::Malformed;
        std::size_t offset=8;auto take=[&](auto& a){if(a.size()>bytes.size()-32-offset)return false;std::copy_n(bytes.begin()+offset,a.size(),a.begin());offset+=a.size();return true;};
        UUID previous{};
        for(std::size_t i=0;i<count;++i){
            Record r;if(!take(r.key.accountScope)||!take(r.key.document)||!take(r.key.request)||!take(r.key.command)||!take(r.key.execution)||offset+2>bytes.size()-32)return ReadStatus::Malformed;
            r.operation=Operation(bytes[offset++]);const auto n=bytes[offset++];if(n<1||n>MaximumEffects)return ReadStatus::Malformed;
            r.policy=0;
            if(!legacy){if(offset+2>bytes.size()-32)return ReadStatus::Malformed;
                r.policy=std::uint16_t(bytes[offset])|(std::uint16_t(bytes[offset+1])<<8);offset+=2;
                if(r.policy==0)return ReadStatus::Malformed;}
            UUID last{};
            for(unsigned j=0;j<n;++j){if(offset>=bytes.size()-32)return ReadStatus::Malformed;Effect e;e.policy=r.policy;e.feature=Feature(bytes[offset++]);
                if(!take(e.entity)||!take(e.definition)||!take(e.featureID)||!take(e.geometry)||!take(e.state)||!(last<e.entity))return ReadStatus::Malformed;
                last=e.entity;r.effects.push_back(e);
            }
            if(!Valid(r)||!(previous<r.key.request))return ReadStatus::Malformed;previous=r.key.request;out.push_back(std::move(r));
        }
        if(offset!=bytes.size()-32)return ReadStatus::Malformed;output=std::move(out);return ReadStatus::Valid;
    }catch(...){out.clear();return ReadStatus::Malformed;}
}

} // namespace core3d::receipt
