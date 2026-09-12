#pragma once
// Receipt data/evidence component only. No request admission or retry permission.
// Every mutation requires the caller's existing OCAF command. Verified queries
// remain unavailable until native permanent tombstones and ordinary integration
// exist; a saved document record by itself is not a trusted execution receipt.
#include "OcctDocument.h"
#include "ProfilePersistence.hxx"
#include "EnclosurePersistence.hxx"
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
#include <ostream>
#include <set>
#include <streambuf>
#include <vector>

namespace core3d::receipt {
using Digest=std::array<std::uint8_t,32>;
using UUID=std::array<std::uint8_t,16>;
constexpr std::size_t MaximumRecords=128, MaximumEffects=16, MaximumBytes=256*1024;
constexpr std::size_t ChunkBytes=256, MaximumChunks=MaximumBytes*2/ChunkBytes;
inline const Standard_GUID& SchemaID(){ static const Standard_GUID id("9FB0D315-83AB-4F68-AE29-C8C85BA670C1");return id; }
inline const Standard_GUID& CountID(){ static const Standard_GUID id("9FB0D315-83AB-4F68-AE29-C8C85BA670C2");return id; }
enum class Operation:std::uint8_t { CreateEnclosure=1, RebuildEnclosure=2, CreateAssembly=3, RebuildProfile=4 };
enum class Feature:std::uint8_t { Profile=1, Enclosure=2 };
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
    bool operator==(const Effect& b)const{return feature==b.feature&&entity==b.entity&&definition==b.definition&&featureID==b.featureID&&geometry==b.geometry&&state==b.state;}
};
struct Record {Key key;Operation operation=Operation::CreateAssembly;std::vector<Effect> effects;};
struct Catalog {TDF_Label label;std::vector<Record> records;std::vector<std::uint8_t> bytes;};
struct Inspection {DocumentPresence presence=DocumentPresence::Unavailable;bool effectsCurrent=false;std::vector<Effect> effects;};
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
    if(r.operation<Operation::CreateEnclosure||r.operation>Operation::RebuildProfile
        ||(r.operation!=Operation::CreateAssembly&&r.effects.size()!=1))return false;
    std::set<UUID> entities,definitions,features;
    for(const auto& e:r.effects){
        const auto expected=(r.operation==Operation::CreateEnclosure||r.operation==Operation::RebuildEnclosure)?Feature::Enclosure:Feature::Profile;
        if(e.feature!=expected||!Nonzero(e.entity)||!Nonzero(e.definition)||!Nonzero(e.featureID)||!Nonzero(e.geometry)||!Nonzero(e.state)
            ||!entities.insert(e.entity).second||!definitions.insert(e.definition).second||!features.insert(e.featureID).second)return false;
    }
    return true;
}
inline bool Encode(std::vector<Record> records,std::vector<std::uint8_t>& output) noexcept {
    output.clear();std::vector<std::uint8_t> out;try{
        if(records.empty()||records.size()>MaximumRecords)return false;
        std::sort(records.begin(),records.end(),[](const auto&a,const auto&b){return a.key.request<b.key.request;});
        out={'S','Y','R','C',1,0,std::uint8_t(records.size()),std::uint8_t(records.size()>>8)};
        auto append=[&](const auto& a){out.insert(out.end(),a.begin(),a.end());};
        UUID previous{};
        for(auto&r:records){
            if(!Valid(r)||r.key.request==previous)return false;previous=r.key.request;
            append(r.key.accountScope);append(r.key.document);append(r.key.request);append(r.key.command);append(r.key.execution);
            out.push_back(std::uint8_t(r.operation));out.push_back(std::uint8_t(r.effects.size()));
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
        if(bytes[4]!=1)return ReadStatus::Unsupported;
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
            UUID last{};
            for(unsigned j=0;j<n;++j){if(offset>=bytes.size()-32)return ReadStatus::Malformed;Effect e;e.feature=Feature(bytes[offset++]);
                if(!take(e.entity)||!take(e.definition)||!take(e.featureID)||!take(e.geometry)||!take(e.state)||!(last<e.entity))return ReadStatus::Malformed;
                last=e.entity;r.effects.push_back(e);
            }
            if(!Valid(r)||!(previous<r.key.request))return ReadStatus::Malformed;previous=r.key.request;out.push_back(std::move(r));
        }
        if(offset!=bytes.size()-32)return ReadStatus::Malformed;output=std::move(out);return ReadStatus::Valid;
    }catch(...){out.clear();return ReadStatus::Malformed;}
}
inline bool HasCatalogAttribute(const TDF_Label& label){return label.IsAttribute(SchemaID())||label.IsAttribute(CountID());}
inline ReadStatus Read(const Handle(TDocStd_Document)& doc,Catalog& out) noexcept {
    out={};try{
        if(doc.IsNull()||doc->GetData().IsNull())return ReadStatus::Unavailable;
        const auto root=doc->GetData()->Root();
        if(HasCatalogAttribute(root)||HasCatalogAttribute(doc->Main()))return ReadStatus::Malformed;
        Catalog found;std::size_t visited=0;
        for(TDF_ChildIterator it(root,Standard_True);it.More();it.Next()){
            if(++visited>100000)return ReadStatus::Malformed;
            if(!HasCatalogAttribute(it.Value()))continue;
            // Reserved GUIDs anywhere else are malformed, including Main,
            // XCAF tool subtrees and nested/duplicate catalog locations.
            if(it.Value()==doc->Main()||it.Value().Father()!=root)return ReadStatus::Malformed;
            if(!found.label.IsNull())return ReadStatus::Malformed;found.label=it.Value();
        }
        if(found.label.IsNull())return ReadStatus::Absent;
        Handle(TDataStd_Integer) schema,count;
        if(!found.label.FindAttribute(SchemaID(),schema)||!found.label.FindAttribute(CountID(),count))return ReadStatus::Malformed;
        if(schema->Get()!=1)return ReadStatus::Unsupported;
        if(count->Get()<1||count->Get()>int(MaximumChunks))return ReadStatus::Malformed;
        for(TDF_AttributeIterator a(found.label);a.More();a.Next())if(a.Value()->ID()!=SchemaID()&&a.Value()->ID()!=CountID()&&a.Value()->ID()!=TDF_TagSource::GetID())return ReadStatus::Malformed;
        std::vector<std::string> chunks(std::size_t(count->Get()));std::size_t populated=0;
        for(TDF_ChildIterator it(found.label,Standard_False);it.More();it.Next()){
            if(++visited>100000)return ReadStatus::Malformed;const auto child=it.Value();
            for(TDF_ChildIterator nested(child,Standard_True);nested.More();nested.Next())if(++visited>100000||nested.Value().HasAttribute())return ReadStatus::Malformed;
            if(!child.HasAttribute())continue;
            if(child.Tag()<1||child.Tag()>count->Get())return ReadStatus::Malformed;
            Handle(TDataStd_AsciiString) value;if(!child.FindAttribute(TDataStd_AsciiString::GetID(),value))return ReadStatus::Malformed;
            for(TDF_AttributeIterator a(child);a.More();a.Next())if(a.Value()->ID()!=TDataStd_AsciiString::GetID())return ReadStatus::Malformed;
            const auto length=value->Get().Length();if(length<1||length>int(ChunkBytes)||(child.Tag()<count->Get()&&length!=int(ChunkBytes)))return ReadStatus::Malformed;
            chunks[std::size_t(child.Tag()-1)]=value->Get().ToCString();++populated;
        }
        if(populated!=chunks.size())return ReadStatus::Malformed;
        int high=-1;for(const auto&s:chunks)for(char c:s){const int x=Nibble(c);if(x<0||(c>='A'&&c<='F'))return ReadStatus::Malformed;
            if(high<0)high=x;else{if(found.bytes.size()>=MaximumBytes)return ReadStatus::Malformed;found.bytes.push_back(std::uint8_t(high*16+x));high=-1;}}
        if(high>=0)return ReadStatus::Malformed;
        const auto result=Decode(found.bytes,found.records);if(result==ReadStatus::Valid)out=std::move(found);return result;
    }catch(...){out={};return ReadStatus::Malformed;}
}
// Component-level data staging seam, not an authority/admission API. No public
// Apply calls this yet. Caller must supply native-captured effects and bind them
// to its command. The future ordinary ledger must retain exact previous bytes,
// revalidate candidate effects and require permanent native intent.
inline bool Stage(const Handle(OcctDocument)& owner,const Record& record,const Catalog& expected) noexcept {
    try{
        if(owner.IsNull()||owner->Document().IsNull()||!owner->Document()->HasOpenCommand()||!Valid(record))return false;
        UUID document;if(!ParseUUID(owner->DocumentIdentifier(),document)||record.key.document!=document)return false;
        Catalog live;const auto status=Read(owner->Document(),live);
        if((status!=ReadStatus::Absent&&status!=ReadStatus::Valid)||live.bytes!=expected.bytes||live.label!=expected.label)return false;
        for(const auto&r:live.records)if(r.key.request==record.key.request)return false;
        live.records.push_back(record);std::vector<std::uint8_t> encoded;if(!Encode(live.records,encoded))return false;
        auto label=live.label;
        if(label.IsNull()){
            // Match saved-group storage: Main owns XCAF fixed tag children.
            // A fresh root sibling cannot overwrite a lazily initialized tool;
            // do not rely on a TagSource counter reflecting existing labels.
            const auto root=owner->Document()->GetData()->Root();
            Standard_Integer maximumTag=0;std::size_t visited=0;
            for(TDF_ChildIterator it(root,Standard_False);it.More();it.Next()){
                if(++visited>100000)return false;
                maximumTag=std::max(maximumTag,it.Value().Tag());
            }
            if(maximumTag==std::numeric_limits<Standard_Integer>::max())return false;
            label=root.FindChild(maximumTag+1,Standard_True);
        }
        const auto hex=Hex(encoded);const auto count=(hex.size()+ChunkBytes-1)/ChunkBytes;
        TDataStd_Integer::Set(label,SchemaID(),1);TDataStd_Integer::Set(label,CountID(),Standard_Integer(count));
        for(std::size_t i=0;i<count;++i){const auto s=hex.substr(i*ChunkBytes,ChunkBytes);TDataStd_AsciiString::Set(label.FindChild(Standard_Integer(i+1)),TCollection_AsciiString(s.c_str()));}
        Catalog readback;return Read(owner->Document(),readback)==ReadStatus::Valid&&readback.bytes==encoded;
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
class GeometryStream final:public std::streambuf {
public:
#if DEBUG
    std::vector<std::uint8_t> *debugBytes = nullptr;
#endif
    GeometryStream(){valid=CC_SHA256_Init(&context)==1;}
    bool finish(Digest& digest){return valid&&written>0&&CC_SHA256_Final(digest.data(),&context)==1;}
protected:
    std::streamsize xsputn(const char* p,std::streamsize n)override{
        if(!valid||n<0||std::size_t(n)>8*1024*1024-written){valid=false;return 0;}
#if DEBUG
        if(debugBytes)debugBytes->insert(debugBytes->end(),p,p+n);
#endif
        if(CC_SHA256_Update(&context,p,CC_LONG(n))!=1){valid=false;return 0;}written+=std::size_t(n);return n;
    }
    int_type overflow(int_type c)override{if(traits_type::eq_int_type(c,traits_type::eof()))return traits_type::not_eof(c);const char x=traits_type::to_char_type(c);return xsputn(&x,1)==1?c:traits_type::eof();}
private:CC_SHA256_CTX context{};std::size_t written=0;bool valid=false;
};
inline bool GeometryDigest(const TopoDS_Shape& shape,Digest& out
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
        GeometryStream buffer;
#if DEBUG
        if(debugBytes){debugBytes->clear();buffer.debugBytes=debugBytes;}
#endif
        std::ostream stream(&buffer);stream.imbue(std::locale::classic());
        // Fixed OCCT format, no cached tessellation/normals. This is exact
        // serialized representation identity, not geometric equivalence.
        BRepTools::Write(detached,stream,Standard_False,Standard_False,TopTools_FormatVersion_VERSION_3);
        return stream.good()&&buffer.finish(out);
    }catch(...){out.fill(0);return false;}
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
        if(!s.enclosure.label.IsNull()){
            if(!s.profile.label.IsNull()||!s.enclosure.IsCurrent(owner->Document(),label)||!enclosure::Encode(s.enclosure.parameters,values))return false;
            e.feature=Feature::Enclosure;schema=s.enclosure.parameters.definition.constructionFrame?enclosure::FramedSchemaVersion:enclosure::SchemaVersion;if(!ParseUUID(s.enclosure.identifier,e.featureID))return false;
        }else{
            if(s.profile.label.IsNull()||!s.profile.IsCurrent(owner->Document(),label)||!SupportedProfile(s.profile.parameters)||!profile::Encode(s.profile.parameters,values))return false;
            e.feature=Feature::Profile;schema=profile::SchemaFor(s.profile.parameters);if(!ParseUUID(s.profile.identifier,e.featureID))return false;
        }
#if DEBUG
        if(debug)debug->stage="geometry";
        if(!GeometryDigest(s.shape,e.geometry,debug?&debug->geometryBytes:nullptr)||named.name.Length()>256)return false;
        if(debug)debug->stage="state";
#else
        if(!GeometryDigest(s.shape,e.geometry)||named.name.Length()>256)return false;
#endif
        std::vector<std::uint8_t> bytes{'S','Y','E','F',1,std::uint8_t(e.feature),std::uint8_t(schema)};
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
        result.presence=DocumentPresence::Present;result.effects=match->effects;result.effectsCurrent=true;
        TDF_LabelSequence roots;XCAFDoc_DocumentTool::ShapeTool(owner->Document()->Main())->GetFreeShapes(roots);
        if(roots.Length()>50000){result.presence=DocumentPresence::Unavailable;result.effectsCurrent=false;return result;}
        for(const auto&expected:match->effects){TDF_Label found;
            for(int i=1;i<=roots.Length();++i){UUID id;if(ParseUUID(owner->EntityIdentifierForLabel(roots.Value(i)),id)&&id==expected.entity){if(!found.IsNull()){result.presence=DocumentPresence::Conflict;result.effectsCurrent=false;return result;}found=roots.Value(i);}}
            Effect live;if(found.IsNull()||!CaptureEffect(owner,found,live)||!(live==expected))result.effectsCurrent=false;
        }
        return result;
    }catch(...){return {};}
}
} // namespace core3d::receipt
