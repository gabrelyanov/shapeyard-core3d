#pragma once
// Detached source-rebuild values and content proof only. No document authority.
#include "SavedCutSourceValuePatch.hxx"
#include "SavedCutSourceBoreClearance.hxx"
#include "SavedCutPrismExtractor.hxx"
#include "RetainedEnclosureCorrespondence.hxx"
#include <BRepTools.hxx>
#include <TopTools_FormatVersion.hxx>
#include <ostream>
#include <streambuf>
#include <locale>

namespace core3d::saved_cut_source_edit {
using Patch = saved_cut_source_values::Patch;
struct ShapeCommitment {
    retained_solid::Digest sha256{};
    std::size_t bytes=0;
    bool operator==(const ShapeCommitment& other)const noexcept {
        return bytes==other.bytes && sha256==other.sha256;
    }
};
// Same exact stream representation as the existing saved-cut/PBR guard.
// A digest is not a correspondence proof and never substitutes for inspection.
inline constexpr std::size_t MaximumShapeBytes=8U*1024U*1024U;
inline constexpr std::size_t MaximumWorkStreamBytes=128U*1024U*1024U;
class CommitmentStream final:public std::streambuf {
    CC_SHA256_CTX context{};
    const std::atomic_bool& stop;
    std::size_t& aggregate;
    std::size_t bytes=0;
    bool good=false,finished=false;
public:
    CommitmentStream(const std::atomic_bool& cancellation,std::size_t& total)
      :stop(cancellation),aggregate(total),good(CC_SHA256_Init(&context)==1){}
    bool finish(ShapeCommitment& out) {
        if(finished||!good||stop.load()||!bytes)return false;
        finished=true;ShapeCommitment value;value.bytes=bytes;
        if(CC_SHA256_Final(value.sha256.data(),&context)!=1)return false;
        out=value;return true;
    }
protected:
    std::streamsize xsputn(const char* p,std::streamsize n)override {
        if(!good||finished||stop.load()||n<0||bytes>MaximumShapeBytes
            ||aggregate>MaximumWorkStreamBytes
            ||std::size_t(n)>MaximumShapeBytes-bytes
            ||std::size_t(n)>MaximumWorkStreamBytes-aggregate){good=false;return 0;}
        good=CC_SHA256_Update(&context,p,static_cast<CC_LONG>(n))==1;
        if(!good)return 0;bytes+=std::size_t(n);aggregate+=std::size_t(n);return n;
    }
    int overflow(int c)override {
        if(c==traits_type::eof())return traits_type::not_eof(c);
        const char b=char(c);return xsputn(&b,1)==1?c:traits_type::eof();
    }
};
inline bool Commit(const TopoDS_Shape& shape,const std::atomic_bool& stop,
    std::size_t& aggregate,ShapeCommitment& out)noexcept {
    out={};try {
        if(stop.load()||shape.IsNull())return false;
        CommitmentStream sink(stop,aggregate);std::ostream stream(&sink);
        stream.imbue(std::locale::classic());
        BRepTools::Write(shape,stream,Standard_True,Standard_True,TopTools_FormatVersion_VERSION_3);
        return stream.good()&&sink.finish(out)&&!stop.load();
    }catch(...){out={};return false;}
}
struct Values {
    retained_solid::Envelope oldEnvelope,newEnvelope;
    std::vector<std::uint8_t> oldBytes,newBytes;
    saved_cut_bore_clearance::Report oldClearance,newClearance;
    bool changed=false;
};
inline bool PrepareValues(const retained_solid::Payload& retained,const Patch& patch,
    const std::atomic_bool& stop,Values& out)noexcept {
    out={};try {
        // Validate lengths and the original exact envelope before copying.
        if(stop.load()||retained.bytes.size()>retained_solid::MaximumEnvelopeBytes)return false;
        const auto* legacy=std::get_if<retained_solid::Envelope>(&retained.envelope);if(!legacy)return false;
        std::vector<std::uint8_t> oldBytes;
        if(!retained_boolean::Encode(retained.envelope,oldBytes)||oldBytes!=retained.bytes)return false;
        const auto applied=saved_cut_source_values::Apply(*legacy,patch);
        if(stop.load()||!applied)return false;
        Values v;v.oldEnvelope=*legacy;v.newEnvelope=applied->envelope;
        v.oldBytes=std::move(oldBytes);v.changed=applied->changed;
        if(!retained_solid::Encode(v.newEnvelope,v.newBytes))return false;
        // Exact fixed envelope check independent of any regenerated DTO.
        auto fixed=v.newEnvelope;fixed.sourceValues=v.oldEnvelope.sourceValues;
        std::vector<std::uint8_t> fixedBytes;
        if(!retained_solid::Encode(fixed,fixedBytes)||fixedBytes!=v.oldBytes)return false;
        if(stop.load())return false;
        v.oldClearance=saved_cut_bore_clearance::Inspect(v.oldEnvelope);
        if(stop.load())return false;
        v.newClearance=saved_cut_bore_clearance::Inspect(v.newEnvelope);
        if(v.oldClearance.status!=saved_cut_bore_clearance::Status::ClearRecipeDisk
            ||v.newClearance.status!=saved_cut_bore_clearance::Status::ClearRecipeDisk)return false;
        if(stop.load())return false;
        out=std::move(v);return true;
    }catch(...){out={};return false;}
}
inline bool InspectBase(const TopoDS_Shape& base,const retained_solid::Envelope& e,
    const std::atomic_bool& stop)noexcept {
    try {
        if(stop.load()||!retained_solid::Valid(e))return false;
        if(e.sourceFamily==1){
            profile::Parameters p;saved_cut_prism_prototype::Inspection report;
            return profile::Decode(e.sourceValues,p)
                &&saved_cut_prism_prototype::InspectPrism(base,p.definition,p.constructionFrame,
                    e.metersPerUnit,stop,report)==saved_cut_prism_prototype::Classification::MatchedBoundary;
        }
        if(e.sourceFamily==2){
            enclosure::Parameters p;enclosure_correspondence::Inspection report;
            return enclosure::Decode(int(e.sourceSchema),e.sourceValues,p)
                &&enclosure_correspondence::InspectEnclosure(base,p,stop,report)
                    ==enclosure_correspondence::Classification::MatchedBoundary;
        }
        return false;
    }catch(...){return false;}
}
} // namespace core3d::saved_cut_source_edit
