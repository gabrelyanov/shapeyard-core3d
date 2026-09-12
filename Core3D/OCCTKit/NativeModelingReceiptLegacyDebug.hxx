#pragma once
#if DEBUG
// Historical generators are fixture-only; production inspection never tries old policies.
namespace core3d::receipt::legacy_debug {
namespace raw {
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
}
namespace cylinder {
class CylinderDirectionZeroFilter final {
public:
    template<class Sink> bool write(const char *bytes,std::size_t count,Sink&&sink) {
        for(std::size_t i=0;i<count;++i) {
            if(phase==Phase::Passthrough)return sink(bytes+i,count-i);
            // Bound lookahead independently of the existing 8 MiB stream limit.
            // An unsupported long record is preserved, never partly tokenized.
            if(line.size()==4096){phase=Phase::Passthrough;if(!sink(line.data(),line.size()))return false;
                line.clear();return sink(bytes+i,count-i);}
            line.push_back(bytes[i]);
            if(bytes[i]=='\n'){if(!emitLine(sink))return false;line.clear();}
        }return true;
    }
    template<class Sink> bool finish(Sink&&sink) {
        // A partial final record is not a proven typed surface record.
        const bool okay=line.empty()||sink(line.data(),line.size());line.clear();return okay;
    }
private:
    enum class Phase {Version,Prelude,Surfaces,Passthrough};
    Phase phase=Phase::Version;std::string line;std::size_t remaining=0;
    struct Token {std::size_t begin=0,size=0;};
    static bool whitespace(char c){return c==' '||c=='\t'||c=='\r'||c=='\n';}
    static bool finiteNumber(const std::string& token) {
        if(token.empty()||token.size()>128)return false;
        std::istringstream input(token);input.imbue(std::locale::classic());double value=0;
        return bool(input>>value)&&input.eof()&&std::isfinite(value);
    }
    template<class Sink> bool emitLine(Sink&&sink) {
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
            if(count==tokens.size()) {if(phase==Phase::Surfaces)phase=Phase::Passthrough;return sink(line.data(),line.size());}
            tokens[count++]={first,cursor-first};
        }
        auto text=[&](std::size_t i){return line.substr(tokens[i].begin,tokens[i].size);};
        if(phase==Phase::Prelude) {
            if(count&&text(0)=="Surfaces") {
                bool valid=count==2;remaining=0;
                if(valid)for(char c:text(1)){if(c<'0'||c>'9'||remaining>8192){valid=false;break;}remaining=remaining*10+std::size_t(c-'0');}
                phase=valid&&remaining>0&&remaining<=8192?Phase::Surfaces:Phase::Passthrough;
            }
            return sink(line.data(),line.size());
        }
        // Only these single-line record boundaries are recognized. Another
        // surface type may contain nested/multiline data, so preserve it and
        // the entire remainder verbatim instead of guessing later boundaries.
        const bool cylinder=count==14&&text(0)=="2";
        const bool plane=count==13&&text(0)=="1";
        if(!cylinder&&!plane){phase=Phase::Passthrough;return sink(line.data(),line.size());}
        for(std::size_t i=1;i<count;++i)if(!finiteNumber(text(i))){phase=Phase::Passthrough;return sink(line.data(),line.size());}
        std::size_t emitted=0;
        if(cylinder)for(std::size_t i=4;i<=12;++i)if(text(i)=="-0") {
            // Delete exactly this token's minus byte; all whitespace and every
            // other byte are emitted unchanged, regardless of write chunking.
            if(!sink(line.data()+emitted,tokens[i].begin-emitted))return false;
            emitted=tokens[i].begin+1;
        }
        if(--remaining==0)phase=Phase::Passthrough;
        return sink(line.data()+emitted,line.size()-emitted);
    }
};

class GeometryStream final:public std::streambuf {
public:
#if DEBUG
    std::vector<std::uint8_t> *debugBytes = nullptr;
#endif
    GeometryStream(){valid=CC_SHA256_Init(&context)==1;}
    bool finish(Digest& digest){
        auto sink=[&](const char*p,std::size_t n){return emit(p,n);};
        return valid&&written>0&&filter.finish(sink)&&CC_SHA256_Final(digest.data(),&context)==1;
    }
protected:
    std::streamsize xsputn(const char* p,std::streamsize n)override{
        if(!valid||n<0||std::size_t(n)>8*1024*1024-written){valid=false;return 0;}
        auto sink=[&](const char*bytes,std::size_t count){return emit(bytes,count);};
        if(!filter.write(p,std::size_t(n),sink)){valid=false;return 0;}written+=std::size_t(n);return n;
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
    CylinderDirectionZeroFilter filter;
    CC_SHA256_CTX context{};std::size_t written=0;bool valid=false;
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
        // representation identity except proven signed-zero cylinder directions.
        // The typed stream filter never changes the detached/native shape.
        BRepTools::Write(detached,stream,Standard_False,Standard_False,TopTools_FormatVersion_VERSION_3);
        return stream.good()&&buffer.finish(out);
    }catch(...){out.fill(0);return false;}
}
}
inline bool EncodeLegacy(std::vector<Record> records,std::vector<std::uint8_t>& output) noexcept {
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

inline bool Capture(const Handle(OcctDocument)& owner,const TDF_Label& label,int policy,Effect& effect,DebugEffectCapture& capture) {
    if(policy<0||policy>2||!CaptureEffect(owner,label,effect,&capture)||capture.stateBytes.size()<89)return false;
    OcctObjectNameState named;if(!owner->CaptureObjectNameStateForLabel(label,named))return false;
    if(policy==0&&!raw::GeometryDigest(named.object.shape,effect.geometry,&capture.geometryBytes))return false;
    if(policy==1&&!cylinder::GeometryDigest(named.object.shape,effect.geometry,&capture.geometryBytes))return false;
    // Exact historical SYEF1 layout, including untouched raw scalar/name bytes.
    capture.stateBytes.erase(capture.stateBytes.begin()+5,capture.stateBytes.begin()+7);
    capture.stateBytes[4]=1;
    std::copy(effect.geometry.begin(),effect.geometry.end(),capture.stateBytes.begin()+55);
    effect.policy=0;
    return Hash(capture.stateBytes.data(),capture.stateBytes.size(),effect.state);
}
// Caller-owned DEBUG fixture transaction only. No production installation API.
inline bool Write(const Handle(TDocStd_Document)& doc,const std::vector<std::uint8_t>& bytes,bool legacy,TDF_Label label={}) {
    if(doc.IsNull()||!doc->HasOpenCommand()||bytes.empty()||bytes.size()>MaximumBytes)return false;
    if(label.IsNull()) {
        const auto root=doc->GetData()->Root();Standard_Integer tag=0;
        for(TDF_ChildIterator it(root,Standard_False);it.More();it.Next())tag=std::max(tag,it.Value().Tag());
        if(tag==std::numeric_limits<Standard_Integer>::max())return false;label=root.FindChild(tag+1,Standard_True);
    }
    const auto hex=Hex(bytes);const auto count=(hex.size()+ChunkBytes-1)/ChunkBytes;
    TDataStd_Integer::Set(label,legacy?SchemaID():VersionedSchemaID(),legacy?1:2);
    TDataStd_Integer::Set(label,legacy?CountID():VersionedCountID(),int(count));
    for(std::size_t i=0;i<count;++i)TDataStd_AsciiString::Set(label.FindChild(int(i+1),Standard_True),TCollection_AsciiString(hex.substr(i*ChunkBytes,ChunkBytes).c_str()));
    return true;
}
inline bool StageLegacy(const Handle(OcctDocument)& owner,const Record& record,const Catalog& expected) {
    Catalog live;const auto status=Read(owner->Document(),live);
    if((status!=ReadStatus::Absent&&status!=ReadStatus::Valid)||!live.matches(expected)||record.policy!=0)return false;
    auto records=live.records;records.push_back(record);std::vector<std::uint8_t> bytes;
    if(!EncodeLegacy(records,bytes)||!Write(owner->Document(),bytes,true,live.legacyLabel))return false;
    Catalog after;return Read(owner->Document(),after)==ReadStatus::Valid&&after.legacyBytes==bytes;
}
} // fixture namespace
#endif
