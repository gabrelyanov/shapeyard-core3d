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

#include "ReceiptCatalogAttribute.hxx"

namespace core3d::receipt {
// V3 Catalog/Read are supplied by ReceiptCatalogAttribute.hxx.
// Only the native ordinary owner may use this data staging seam inside its
// already-owned OCAF command. Legacy component attributes/chunks are untouched.
inline bool StageLegacy(const Handle(OcctDocument)& owner,const Record& record,const Catalog& expected) noexcept {
    try{
        if(owner.IsNull()||owner->Document().IsNull()||!owner->Document()->HasOpenCommand()
            ||!Valid(record)||!SupportedPolicy(record.operation,record.policy))return false;
        UUID document;if(!ParseUUID(owner->DocumentIdentifier(),document)||record.key.document!=document)return false;
        Catalog live;const auto status=Read(owner->Document(),live);
        if((status!=ReadStatus::Absent&&status!=ReadStatus::Valid)||!live.matches(expected)||!live.supportsAppend()
            ||live.tree||live.records.size()>=MaximumRecords)return false;
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

// Called only from the existing ordinary transaction owner. The legacy path is
// retained until its existing representation cannot admit this exact append.
inline bool Stage(const Handle(OcctDocument)& owner,const Record& record,const Catalog& expected) noexcept {
    try{
        if(owner.IsNull()||owner->Document().IsNull()||!owner->Document()->HasOpenCommand()
            ||!Valid(record)||!SupportedPolicy(record.operation,record.policy))return false;
        UUID document;if(!ParseUUID(owner->DocumentIdentifier(),document)||record.key.document!=document)return false;
        Catalog live;const auto status=Read(owner->Document(),live);
        if((status!=ReadStatus::Absent&&status!=ReadStatus::Valid)||!live.matches(expected)
            ||!live.supportsAppend()||live.contains(record.key.request)||!live.canAppend())return false;
        if(!live.tree){
            std::vector<Record> versioned;for(const auto&r:live.records)if(r.policy!=0)versioned.push_back(r);
            versioned.push_back(record);std::vector<std::uint8_t> encoded;
            if(live.records.size()<MaximumRecords&&Encode(versioned,encoded)
                &&encoded.size()<=MaximumBytes-live.legacyBytes.size()
                &&(encoded.size()*2+ChunkBytes-1)/ChunkBytes<=MaximumChunks-(live.legacyBytes.size()*2+ChunkBytes-1)/ChunkBytes)
                return StageLegacy(owner,record,expected);
        }
        auto previous=live.tree;
        if(!previous){
            Digest legacy,versioned;if(!v3::ArchiveHash(live.legacyBytes,legacy)||!v3::ArchiveHash(live.bytes,versioned))return false;
            previous=v3::Tree::Empty(document,legacy,versioned);
            previous=v3::ImportArchive(previous,live.legacyBytes);previous=v3::ImportArchive(previous,live.bytes);
        }
        auto candidate=previous->append(record); // All fallible tree work before TDF mutation.
        auto label=live.v3Label;
        Handle(Core3D_ModelingReceiptCatalog) attribute;
        if(label.IsNull()){
            const auto root=owner->Document()->GetData()->Root();Standard_Integer last=0;std::size_t labels=0;
            for(TDF_ChildIterator it(root,Standard_False);it.More();it.Next()){
                if(++labels>100000)return false;last=std::max(last,it.Value().Tag());}
            if(last==std::numeric_limits<Standard_Integer>::max())return false;
            attribute=new Core3D_ModelingReceiptCatalog();label=root.FindChild(last+1,Standard_True);label.AddAttribute(attribute);
        }else if(!label.FindAttribute(ScalableSchemaID(),attribute)||attribute.IsNull()||attribute->root_!=live.tree)return false;
        attribute->Backup();attribute->root_=candidate;
        Catalog readback;return Read(owner->Document(),readback)==ReadStatus::Valid&&readback.tree==candidate
            &&readback.v3Label==label&&readback.label==live.label&&readback.legacyLabel==live.legacyLabel
            &&readback.bytes==live.bytes&&readback.legacyBytes==live.legacyBytes;
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
        Record matched;
        if(!catalog.lookup(key.request,matched)){result.presence=DocumentPresence::Absent;return result;}
        if(!(matched.key==key)){result.presence=DocumentPresence::Conflict;return result;}
        const Record* match=&matched;
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
