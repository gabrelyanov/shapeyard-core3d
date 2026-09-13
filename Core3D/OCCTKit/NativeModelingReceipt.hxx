#pragma once
// Receipt data/evidence component only. No request admission or retry permission.
// Every mutation requires the caller's existing OCAF command. Verified queries
// remain unavailable until native permanent tombstones and ordinary integration
// exist; a saved document record by itself is not a trusted execution receipt.
#include "OcctDocument.h"
#include "ProfilePersistence.hxx"
#include "EnclosurePersistence.hxx"
#include "RectangularLoftRebuild.hxx"
#include "NativeModelingRequest.hxx"
#include <CommonCrypto/CommonDigest.h>
#include <BRepTools.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <TopoDS_Iterator.hxx>
#include <TDF_TagSource.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_AttributeIterator.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TDataStd_Integer.hxx>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <string>
#include <utility>
#include <locale>
#include <limits>
#include <iterator>
#include <ostream>
#include <set>
#include <streambuf>
#include <sstream>
#include <vector>

namespace core3d::receipt {
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
inline const Standard_GUID& VersionedSchemaID(){static const Standard_GUID id("42AB4BB7-6421-445F-A2B0-2DC682AB3001");return id;}
inline const Standard_GUID& VersionedCountID(){static const Standard_GUID id("42AB4BB7-6421-445F-A2B0-2DC682AB3002");return id;}
enum class EffectEvidenceStatus { Current, Mismatch, LegacyUnversioned, UnsupportedPolicy, Unavailable };
enum class Operation:std::uint8_t { CreateEnclosure=1, RebuildEnclosure=2, CreateAssembly=3, RebuildProfile=4, RebuildLoftStation=6, SetPlacement=7 };
enum class Feature:std::uint8_t { Profile=1, Enclosure=2, RectangularLoft=3, PlacementObject=4, PlacementBareSolid=5 };
inline bool SupportedPolicy(Operation operation,std::uint16_t policy)noexcept {
    return operation==Operation::SetPlacement?policy==ExactPlacementPolicy8193:operation==Operation::RebuildLoftStation?policy==ExactLoftPolicy4097:policy==AnalyticZeroPolicy1;
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
// label/bytes describe only the versioned component. Legacy storage is never
// rewritten. All authority fences compare both exact components through matches.
struct Catalog {
    TDF_Label label,legacyLabel;
    std::vector<Record> records; // Combined sorted request inventory, including legacy.
    std::vector<std::uint8_t> bytes,legacyBytes;
    bool matches(const Catalog& b)const noexcept {
        return label==b.label&&legacyLabel==b.legacyLabel&&bytes==b.bytes&&legacyBytes==b.legacyBytes;
    }
    bool supportsAppend()const noexcept {
        return std::all_of(records.begin(),records.end(),[](const Record&r){return r.policy==0||SupportedPolicy(r.operation,r.policy);});
    }
};
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
    const bool loft=r.operation==Operation::RebuildLoftStation;
    if((!loft&&!placement&&(r.operation<Operation::CreateEnclosure||r.operation>Operation::RebuildProfile))
        ||(r.operation!=Operation::CreateAssembly&&r.effects.size()!=1)
        ||(loft&&(r.policy==0||r.policy==AnalyticZeroPolicy1))
        ||(placement&&(r.policy==0||r.policy==AnalyticZeroPolicy1||r.policy==ExactLoftPolicy4097)))return false;
    // An assigned loft policy cannot retroactively invalidate unknown-policy
    // records on older operations. SupportedPolicy keeps them unresolved and
    // nonappendable without changing their structural bytes.
    std::set<UUID> entities,definitions,features;
    for(const auto& e:r.effects){
        const auto expected=(r.operation==Operation::CreateEnclosure||r.operation==Operation::RebuildEnclosure)?Feature::Enclosure:(loft?Feature::RectangularLoft:Feature::Profile);
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
inline bool HasCatalogAttribute(const TDF_Label& label){
    return label.IsAttribute(SchemaID())||label.IsAttribute(CountID())
        ||label.IsAttribute(VersionedSchemaID())||label.IsAttribute(VersionedCountID());
}
inline ReadStatus Read(const Handle(TDocStd_Document)& doc,Catalog& out) noexcept {
    out={};try{
        if(doc.IsNull()||doc->GetData().IsNull())return ReadStatus::Unavailable;
        const auto root=doc->GetData()->Root();
        if(HasCatalogAttribute(root)||HasCatalogAttribute(doc->Main()))return ReadStatus::Malformed;
        Catalog found;std::size_t visited=0;
        // Exactly one traversal pays the shared label budget, including chunk
        // descendants. No per-catalog reset can double this ceiling.
        for(TDF_ChildIterator it(root,Standard_True);it.More();it.Next()){
            if(++visited>100000)return ReadStatus::Malformed;
            const auto label=it.Value();if(!HasCatalogAttribute(label))continue;
            if(label==doc->Main()||label.Father()!=root)return ReadStatus::Malformed;
            const bool legacy=label.IsAttribute(SchemaID())||label.IsAttribute(CountID());
            const bool versioned=label.IsAttribute(VersionedSchemaID())||label.IsAttribute(VersionedCountID());
            if(legacy==versioned)return ReadStatus::Malformed;
            auto& slot=legacy?found.legacyLabel:found.label;
            if(!slot.IsNull())return ReadStatus::Malformed;slot=label;
        }
        if(found.label.IsNull()&&found.legacyLabel.IsNull())return ReadStatus::Absent;
        std::size_t totalBytes=0,totalChunks=0;
        for(bool legacy:{true,false}){
            const auto label=legacy?found.legacyLabel:found.label;if(label.IsNull())continue;
            const auto& schemaID=legacy?SchemaID():VersionedSchemaID();
            const auto& countID=legacy?CountID():VersionedCountID();
            Handle(TDataStd_Integer) schema,count;
            if(!label.FindAttribute(schemaID,schema)||!label.FindAttribute(countID,count))return ReadStatus::Malformed;
            if(schema->Get()!=(legacy?1:2))return ReadStatus::Unsupported;
            if(count->Get()<1||std::size_t(count->Get())>MaximumChunks-totalChunks)return ReadStatus::Malformed;
            totalChunks+=std::size_t(count->Get());
            for(TDF_AttributeIterator a(label);a.More();a.Next())if(a.Value()->ID()!=schemaID&&a.Value()->ID()!=countID&&a.Value()->ID()!=TDF_TagSource::GetID())return ReadStatus::Malformed;
            std::vector<std::string> chunks(std::size_t(count->Get()));std::size_t populated=0;
            for(TDF_ChildIterator it(label,Standard_False);it.More();it.Next()){
                const auto child=it.Value();
                for(TDF_ChildIterator nested(child,Standard_True);nested.More();nested.Next())if(nested.Value().HasAttribute())return ReadStatus::Malformed;
                if(!child.HasAttribute())continue;
                if(child.Tag()<1||child.Tag()>count->Get())return ReadStatus::Malformed;
                Handle(TDataStd_AsciiString) value;if(!child.FindAttribute(TDataStd_AsciiString::GetID(),value))return ReadStatus::Malformed;
                for(TDF_AttributeIterator a(child);a.More();a.Next())if(a.Value()->ID()!=TDataStd_AsciiString::GetID())return ReadStatus::Malformed;
                const auto length=value->Get().Length();if(length<1||length>int(ChunkBytes)||(child.Tag()<count->Get()&&length!=int(ChunkBytes)))return ReadStatus::Malformed;
                chunks[std::size_t(child.Tag()-1)]=value->Get().ToCString();++populated;
            }
            if(populated!=chunks.size())return ReadStatus::Malformed;
            auto& bytes=legacy?found.legacyBytes:found.bytes;int high=-1;
            for(const auto& chunk:chunks)for(char c:chunk){const int x=Nibble(c);if(x<0||(c>='A'&&c<='F'))return ReadStatus::Malformed;
                if(high<0)high=x;else{if(totalBytes>=MaximumBytes)return ReadStatus::Malformed;bytes.push_back(std::uint8_t(high*16+x));high=-1;++totalBytes;}}
            if(high>=0||bytes.size()<8||bytes[4]!=(legacy?1:2))return ReadStatus::Malformed;
            std::vector<Record> decoded;const auto status=Decode(bytes,decoded);if(status!=ReadStatus::Valid)return status;
            if(decoded.size()>MaximumRecords-found.records.size())return ReadStatus::Malformed;
            found.records.insert(found.records.end(),std::make_move_iterator(decoded.begin()),std::make_move_iterator(decoded.end()));
        }
        std::sort(found.records.begin(),found.records.end(),[](const auto&a,const auto&b){return a.key.request<b.key.request;});
        UUID previous{};for(const auto& record:found.records){if(!(previous<record.key.request))return ReadStatus::Malformed;previous=record.key.request;}
        out=std::move(found);return ReadStatus::Valid;
    }catch(...){out={};return ReadStatus::Malformed;}
}
// Only the native ordinary owner may use this data staging seam inside its
// already-owned OCAF command. Legacy component attributes/chunks are untouched.
inline bool Stage(const Handle(OcctDocument)& owner,const Record& record,const Catalog& expected) noexcept {
    try{
        if(owner.IsNull()||owner->Document().IsNull()||!owner->Document()->HasOpenCommand()
            ||!Valid(record)||!SupportedPolicy(record.operation,record.policy))return false;
        UUID document;if(!ParseUUID(owner->DocumentIdentifier(),document)||record.key.document!=document)return false;
        Catalog live;const auto status=Read(owner->Document(),live);
        if((status!=ReadStatus::Absent&&status!=ReadStatus::Valid)||!live.matches(expected)||!live.supportsAppend()
            ||live.records.size()>=MaximumRecords)return false;
        std::vector<Record> versioned;
        for(const auto&r:live.records){if(r.key.request==record.key.request)return false;if(r.policy!=0)versioned.push_back(r);}
        versioned.push_back(record);std::vector<std::uint8_t> encoded;
        if(!Encode(versioned,encoded)||encoded.size()>MaximumBytes-live.legacyBytes.size())return false;
        const auto chunkCount=(encoded.size()*2+ChunkBytes-1)/ChunkBytes;
        const auto legacyChunks=(live.legacyBytes.size()*2+ChunkBytes-1)/ChunkBytes;
        if(chunkCount>MaximumChunks-legacyChunks)return false;
        auto label=live.label;
        if(label.IsNull()){
            const auto root=owner->Document()->GetData()->Root();Standard_Integer maximumTag=0;std::size_t visited=0;
            for(TDF_ChildIterator it(root,Standard_False);it.More();it.Next()){
                if(++visited>100000)return false;maximumTag=std::max(maximumTag,it.Value().Tag());}
            if(maximumTag==std::numeric_limits<Standard_Integer>::max())return false;
            label=root.FindChild(maximumTag+1,Standard_True);
        }
        const auto hex=Hex(encoded);TDataStd_Integer::Set(label,VersionedSchemaID(),2);TDataStd_Integer::Set(label,VersionedCountID(),Standard_Integer(chunkCount));
        for(std::size_t i=0;i<chunkCount;++i){const auto chunk=hex.substr(i*ChunkBytes,ChunkBytes);TDataStd_AsciiString::Set(label.FindChild(Standard_Integer(i+1)),TCollection_AsciiString(chunk.c_str()));}
        Catalog readback;return Read(owner->Document(),readback)==ReadStatus::Valid&&readback.label==label&&readback.bytes==encoded
            &&readback.legacyLabel==expected.legacyLabel&&readback.legacyBytes==expected.legacyBytes;
    }catch(...){return false;}
}

#if DEBUG
// Optional bounded evidence sinks expose the exact bytes already hashed. They
// never substitute a digest or normalize additional geometry/state values.
struct DebugEffectCapture {
    std::vector<std::uint8_t> geometryBytes, stateBytes;
    const char *stage = "entry";
};
#endif
// OCCT7.8 compact geometry grammar, limited to the measured analytic records.
// Only literal -0 numeric tokens are canonicalized:2D line coordinates and
// directions;3D circle and plane/cylinder directions. All other spellings,
// nonzero values,3D origins/radii,trim parameters and topology bytes survive.
// Unknown records abandon parsing for the remainder; never seek a later header
// inside an unrecognized multiline record. No native shape/state is modified.
class AnalyticGeometryZeroFilter final {
public:
    template<class Sink> bool write(const char *bytes,std::size_t count,Sink&&sink) {
        for(std::size_t i=0;i<count;++i) {
            if(phase==Phase::Passthrough)return sink(bytes+i,count-i);
            if(line.size()==4096){phase=Phase::Passthrough;if(!sink(line.data(),line.size()))return false;
                line.clear();return sink(bytes+i,count-i);}
            line.push_back(bytes[i]);
            if(bytes[i]=='\n'){if(!emitLine(sink))return false;line.clear();}
        }return true;
    }
    template<class Sink> bool finish(Sink&&sink) {
        // Incomplete final lines have no proven record boundary.
        const bool okay=line.empty()||sink(line.data(),line.size());line.clear();return okay;
    }
private:
    enum class Phase {Version,Prelude,Between,Curve2ds,Curves,Surfaces,Passthrough};
    Phase phase=Phase::Version;std::string line;std::size_t remaining=0,trimDepth=0;
    unsigned lastSection=0;bool sawLocations=false;
    struct Token {std::size_t begin=0,size=0;};
    static bool whitespace(char c){return c==' '||c=='\t'||c=='\r'||c=='\n';}
    static bool finiteNumber(const std::string& token) {
        if(token.empty()||token.size()>128)return false;
        std::istringstream input(token);input.imbue(std::locale::classic());double value=0;
        return bool(input>>value)&&input.eof()&&std::isfinite(value);
    }
    static bool boundedCount(const std::string& token,std::size_t& value) {
        value=0;if(token.empty()||token.size()>4)return false;
        for(char c:token){if(c<'0'||c>'9')return false;const std::size_t digit=std::size_t(c-'0');
            if(value>(8192-digit)/10)return false;value=value*10+digit;}
        return true;
    }
    template<class Sink> bool emitLine(Sink&&sink) {
        auto passthrough=[&](){phase=Phase::Passthrough;return sink(line.data(),line.size());};
        if(phase==Phase::Version) {
            if(line=="\n")return sink(line.data(),line.size());
            phase=line=="CASCADE Topology V3, (c) Open Cascade\n"?Phase::Prelude:Phase::Passthrough;
            return sink(line.data(),line.size());
        }
        std::array<Token,16> tokens{};std::size_t count=0,cursor=0;
        while(cursor<line.size()) {
            while(cursor<line.size()&&whitespace(line[cursor]))++cursor;
            if(cursor==line.size())break;
            const auto first=cursor;while(cursor<line.size()&&!whitespace(line[cursor]))++cursor;
            if(count==tokens.size())return passthrough();
            tokens[count++]={first,cursor-first};
        }
        auto text=[&](std::size_t i){return line.substr(tokens[i].begin,tokens[i].size);};
        if(phase==Phase::Prelude||phase==Phase::Between) {
            if(count!=2)return passthrough();
            const auto name=text(0);std::size_t records=0;
            if(!boundedCount(text(1),records))return passthrough();
            if(name=="Locations") {
                // Nonempty location tables have a different grammar. Preserve
                // them and everything after them rather than infer boundaries.
                if(phase!=Phase::Prelude||sawLocations||lastSection||records)return passthrough();
                sawLocations=true;return sink(line.data(),line.size());
            }
            const unsigned section=name=="Curve2ds"?1:name=="Curves"?2:name=="Polygon3D"?3:
                name=="PolygonOnTriangulations"?4:name=="Surfaces"?5:0;
            if(!section||section<=lastSection)return passthrough();
            lastSection=section;remaining=records;trimDepth=0;
            if(section==3||section==4) {
                if(records)return passthrough();phase=Phase::Between;
            }else if(!records)phase=section==5?Phase::Passthrough:Phase::Between;
            else phase=section==1?Phase::Curve2ds:section==2?Phase::Curves:Phase::Surfaces;
            return sink(line.data(),line.size());
        }
        if(!count)return passthrough();
        const auto type=text(0);std::size_t firstZero=0,lastZero=0;bool trimmed=false;
        if(phase==Phase::Curve2ds) {
            if(type=="1"&&count==5){firstZero=1;lastZero=4;}
            else if(type=="2"&&count==8){} // Circle basis: preserve every scalar.
            else if(type=="8"&&count==3&&trimDepth<64)trimmed=true;
            else return passthrough();
        }else if(phase==Phase::Curves) {
            if(type=="1"&&count==7){} //3D line fields are outside measured policy.
            else if(type=="2"&&count==14){firstZero=4;lastZero=12;}
            else return passthrough();
        }else if(phase==Phase::Surfaces) {
            if((type=="1"&&count==13)||(type=="2"&&count==14)){firstZero=4;lastZero=12;}
            else return passthrough();
        }else return passthrough();
        for(std::size_t i=1;i<count;++i)if(!finiteNumber(text(i)))return passthrough();
        // A trimmed2D record owns exactly one recursively written basis.
        // Only a terminal line/circle consumes a counted top-level record.
        if(trimmed){++trimDepth;return sink(line.data(),line.size());}
        if(!remaining)return passthrough();
        std::size_t emitted=0;
        if(firstZero)for(std::size_t i=firstZero;i<=lastZero;++i)if(text(i)=="-0") {
            if(!sink(line.data()+emitted,tokens[i].begin-emitted))return false;
            emitted=tokens[i].begin+1;
        }
        trimDepth=0;
        if(--remaining==0)phase=phase==Phase::Surfaces?Phase::Passthrough:Phase::Between;
        return sink(line.data()+emitted,line.size()-emitted);
    }
};

class GeometryStream final:public std::streambuf {
public:
#if DEBUG
    std::vector<std::uint8_t> *debugBytes = nullptr;
#endif
    explicit GeometryStream(bool analytic=true):analytic(analytic){valid=CC_SHA256_Init(&context)==1;}
    bool finish(Digest& digest){
        auto sink=[&](const char*p,std::size_t n){return emit(p,n);};
        return valid&&written>0&&(!analytic||filter.finish(sink))&&CC_SHA256_Final(digest.data(),&context)==1;
    }
protected:
    std::streamsize xsputn(const char* p,std::streamsize n)override{
        if(!valid||n<0||std::size_t(n)>8*1024*1024-written){valid=false;return 0;}
        auto sink=[&](const char*bytes,std::size_t count){return emit(bytes,count);};
        if(!(analytic?filter.write(p,std::size_t(n),sink):emit(p,std::size_t(n)))){valid=false;return 0;}written+=std::size_t(n);return n;
    }
    int_type overflow(int_type c)override{if(traits_type::eq_int_type(c,traits_type::eof()))return traits_type::not_eof(c);const char x=traits_type::to_char_type(c);return xsputn(&x,1)==1?c:traits_type::eof();}
private:
    bool emit(const char*p,std::size_t n){
        if(!valid||CC_SHA256_Update(&context,p,CC_LONG(n))!=1){valid=false;return false;}
#if DEBUG
        // Diagnostics expose exactly the canonical bytes fed to SHA256.
        if(debugBytes)debugBytes->insert(debugBytes->end(),p,p+n);
#endif
        return true;
    }
    const bool analytic;
    AnalyticGeometryZeroFilter filter;
    CC_SHA256_CTX context{};std::size_t written=0;bool valid=false;
};
inline bool GeometryDigestForPolicy(const TopoDS_Shape& shape,Digest& out,bool analytic
#if DEBUG
    , std::vector<std::uint8_t> *debugBytes=nullptr
#endif
) noexcept {
    out.fill(0);try{
        if(shape.IsNull()||shape.ShapeType()!=TopAbs_SOLID)return false;
        std::vector<std::pair<TopoDS_Shape,unsigned>> pending{{shape,0}};std::size_t nodes=0;
        while(!pending.empty()){auto current=std::move(pending.back());pending.pop_back();if(++nodes>8192||current.second>64)return false;
            for(TopoDS_Iterator it(current.first);it.More();it.Next()){if(pending.size()>=8192)return false;pending.emplace_back(it.Value(),current.second+1);}}
        BRepBuilderAPI_Copy copied(shape,Standard_True,Standard_False);
        if(!copied.IsDone()||copied.Shape().IsNull())return false;
        auto detached=copied.Shape();
        if(!BRepCheck_Analyzer(detached,Standard_True).IsValid())return false;
        // Normalize only mutable bookkeeping flags on this private deep copy.
        // Mesh caches and observer/checker activity must not change evidence.
        pending={{detached,0}};nodes=0;
        while(!pending.empty()){auto current=std::move(pending.back());pending.pop_back();if(++nodes>8192||current.second>64)return false;
            current.first.Free(Standard_True);current.first.Modified(Standard_False);current.first.Checked(Standard_False);
            for(TopoDS_Iterator it(current.first);it.More();it.Next()){if(pending.size()>=8192)return false;pending.emplace_back(it.Value(),current.second+1);}}
        GeometryStream buffer(analytic);
#if DEBUG
        if(debugBytes){debugBytes->clear();buffer.debugBytes=debugBytes;}
#endif
        std::ostream stream(&buffer);stream.imbue(std::locale::classic());
        // Fixed OCCT format, no cached tessellation/normals. This is exact
        // representation identity except the proven typed analytic zero fields.
        // The typed stream filter never changes the detached/native shape.
        BRepTools::Write(detached,stream,Standard_False,Standard_False,TopTools_FormatVersion_VERSION_3);
        return stream.good()&&buffer.finish(out);
    }catch(...){out.fill(0);return false;}
}
// Existing callers keep the exact original analytic policy and byte stream.
inline bool GeometryDigest(const TopoDS_Shape& shape,Digest& out
#if DEBUG
    , std::vector<std::uint8_t> *debugBytes=nullptr
#endif
) noexcept {
    return GeometryDigestForPolicy(shape,out,true
#if DEBUG
        ,debugBytes
#endif
    );
}
// Only validated native recipe data can make the reusable numeric descriptor.
// This helper does not create a request, source lease, reservation or receipt.
inline bool LoftStationDescriptor(const rectangular_loft::Definition& source,
    const rectangular_loft::StationDimensionEdit& edit,request::Descriptor& output)noexcept {
    output={};try{
        rectangular_loft::Definition candidate;
        if(!loft_rebuild::Apply(source,edit,candidate)||!loft_rebuild::Matches(source,edit,candidate))return false;
        request::Part part;part.recipe=request::Recipe::RectangularLoft;part.schema=loft_persistence::Schema;
        if(!loft_persistence::Encode(candidate,part.values))return false;
        request::Descriptor d;d.operation=request::Operation::RebuildLoftStation;d.parts={std::move(part)};
        d.loftStationEdit=request::LoftStationEdit{edit.stationIdentifier,edit.width,edit.depth};
        std::vector<std::uint8_t> encoded;if(!request::Encode(d,encoded))return false;
        output=std::move(d);return true;
    }catch(...){output={};return false;}
}
inline bool SupportedProfile(const profile::Parameters& p){
    const auto&d=p.definition;if(d.revolve||d.curves||!d.holes.empty())return false;
    if(d.circle)return d.points.empty()&&d.circle->center.X()==0&&d.circle->center.Y()==0&&d.circle->innerRadius==0;
    if(d.points.size()!=4)return false;const auto&a=d.points;
    return a[0].X()==0&&a[0].Y()==0&&a[1].X()>0&&a[1].Y()==0&&a[2].X()==a[1].X()&&a[2].Y()>0&&a[3].X()==0&&a[3].Y()==a[2].Y();
}
inline bool CaptureEffect(const Handle(OcctDocument)& owner,const TDF_Label& label,Effect& out
#if DEBUG
    , DebugEffectCapture *debug=nullptr
#endif
) noexcept {
    out={};try{
        if(owner.IsNull()||owner->Document().IsNull())return false;
#if DEBUG
        if(debug){*debug={};debug->stage="object-state";}
#endif
        OcctObjectNameState named;if(!owner->CaptureObjectNameStateForLabel(label,named))return false;
        const auto&s=named.object;Effect e;
        if(s.resolvedRepresentation!=OcctGeometryRepresentation::BRep||!ParseUUID(s.entityIdentifier,e.entity)||!ParseUUID(s.definitionIdentifier,e.definition))return false;
#if DEBUG
        if(debug)debug->stage="recipe";
#endif
        std::vector<double> values;int schema=0;
        if(!s.loft.label.IsNull()){
            if(!s.enclosure.label.IsNull()||!s.profile.label.IsNull()||!s.sweep.label.IsNull()
                ||!s.loft.IsCurrent(owner->Document(),label)||!loft_persistence::Encode(s.loft.definition,values))return false;
            e.feature=Feature::RectangularLoft;e.policy=ExactLoftPolicy4097;schema=loft_persistence::Schema;
            if(!ParseUUID(s.loft.identifier,e.featureID))return false;
        }else if(!s.enclosure.label.IsNull()){
            if(!s.profile.label.IsNull()||!s.enclosure.IsCurrent(owner->Document(),label)||!enclosure::Encode(s.enclosure.parameters,values))return false;
            e.feature=Feature::Enclosure;schema=s.enclosure.parameters.definition.constructionFrame?enclosure::FramedSchemaVersion:enclosure::SchemaVersion;if(!ParseUUID(s.enclosure.identifier,e.featureID))return false;
        }else{
            if(s.profile.label.IsNull()||!s.profile.IsCurrent(owner->Document(),label)||!SupportedProfile(s.profile.parameters)||!profile::Encode(s.profile.parameters,values))return false;
            e.feature=Feature::Profile;schema=profile::SchemaFor(s.profile.parameters);if(!ParseUUID(s.profile.identifier,e.featureID))return false;
        }
#if DEBUG
        if(debug)debug->stage="geometry";
        if(!GeometryDigestForPolicy(s.shape,e.geometry,e.policy==AnalyticZeroPolicy1,debug?&debug->geometryBytes:nullptr)||named.name.Length()>256)return false;
        if(debug)debug->stage="state";
#else
        if(!GeometryDigestForPolicy(s.shape,e.geometry,e.policy==AnalyticZeroPolicy1)||named.name.Length()>256)return false;
#endif
        std::vector<std::uint8_t> bytes{'S','Y','E','F',2,std::uint8_t(e.policy),std::uint8_t(e.policy>>8),std::uint8_t(e.feature),std::uint8_t(schema)};
        auto append=[&](const auto&a){bytes.insert(bytes.end(),a.begin(),a.end());};
        auto integer=[&](std::uint64_t n){for(unsigned i=0;i<8;++i)bytes.push_back(std::uint8_t(n>>(8*i)));};
        auto scalar=[&](double d){if(!std::isfinite(d))throw Standard_Failure("Nonfinite receipt effect");std::uint64_t bits;std::memcpy(&bits,&d,8);integer(bits);};
        append(e.entity);append(e.definition);append(e.featureID);append(e.geometry);integer(values.size());for(double v:values)scalar(v);
        for(std::size_t i=0;i<s.scalars.size();++i){bytes.push_back(s.present[i]?1:0);scalar(s.scalars[i]);}
        bytes.push_back(named.namePresent?1:0);integer(std::size_t(named.name.Length()));for(int i=1;i<=named.name.Length();++i)integer(std::uint16_t(named.name.Value(i)));
#if DEBUG
        if(debug)debug->stateBytes=bytes;
#endif
        if(!Hash(bytes.data(),bytes.size(),e.state))return false;out=e;
#if DEBUG
        if(debug)debug->stage="complete";
#endif
        return true;
    }catch(...){out={};return false;}
}
inline Inspection InspectDocument(const Handle(OcctDocument)& owner,const Key& key) noexcept {
    Inspection result;try{
        if(owner.IsNull()||owner->Document().IsNull()||owner->Document()->HasOpenCommand())return result;
        UUID document;if(!ParseUUID(owner->DocumentIdentifier(),document)||document!=key.document){result.presence=DocumentPresence::Conflict;return result;}
        Catalog catalog;const auto status=Read(owner->Document(),catalog);
        if(status==ReadStatus::Absent){result.presence=DocumentPresence::Absent;return result;}
        if(status!=ReadStatus::Valid){result.presence=status==ReadStatus::Malformed?DocumentPresence::Conflict:DocumentPresence::Unavailable;return result;}
        const Record* match=nullptr;for(const auto&r:catalog.records)if(r.key.request==key.request){if(!(r.key==key)){result.presence=DocumentPresence::Conflict;return result;}match=&r;}
        if(!match){result.presence=DocumentPresence::Absent;return result;}
        result.presence=DocumentPresence::Present;result.effects=match->effects;
        if(match->policy==0){result.evidence=EffectEvidenceStatus::LegacyUnversioned;return result;}
        if(!SupportedPolicy(match->operation,match->policy)){result.evidence=EffectEvidenceStatus::UnsupportedPolicy;return result;}
        if(match->operation==Operation::SetPlacement){result.evidence=EffectEvidenceStatus::Unavailable;return result;}
        result.effectsCurrent=true;result.evidence=EffectEvidenceStatus::Current;
        TDF_LabelSequence roots;XCAFDoc_DocumentTool::ShapeTool(owner->Document()->Main())->GetFreeShapes(roots);
        if(roots.Length()>50000){result.presence=DocumentPresence::Unavailable;result.effectsCurrent=false;result.evidence=EffectEvidenceStatus::Unavailable;return result;}
        for(const auto&expected:match->effects){TDF_Label found;
            for(int i=1;i<=roots.Length();++i){UUID id;if(ParseUUID(owner->EntityIdentifierForLabel(roots.Value(i)),id)&&id==expected.entity){if(!found.IsNull()){result.presence=DocumentPresence::Conflict;result.effectsCurrent=false;return result;}found=roots.Value(i);}}
            if(found.IsNull()){result.effectsCurrent=false;if(result.evidence!=EffectEvidenceStatus::Unavailable)result.evidence=EffectEvidenceStatus::Mismatch;continue;}
            Effect live;if(!CaptureEffect(owner,found,live)){result.effectsCurrent=false;result.evidence=EffectEvidenceStatus::Unavailable;continue;}
            if(!(live==expected)){result.effectsCurrent=false;if(result.evidence!=EffectEvidenceStatus::Unavailable)result.evidence=EffectEvidenceStatus::Mismatch;}
        }
        return result;
    }catch(...){return {};}
}
} // namespace core3d::receipt
