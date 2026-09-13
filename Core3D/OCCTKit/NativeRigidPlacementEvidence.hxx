#pragma once
// Placement evidence v1 only: no catalog, durable receipt, admission or retry
// authority. Numeric request/coupled receipt integration remains separate.
#include "NativeModelingReceipt.hxx"
#include <pthread.h>

namespace core3d::placement {
using Digest = receipt::Digest;
using UUID = receipt::UUID;
enum class Family : std::uint8_t { Profile=1, Enclosure=2, Sweep=3, RectangularLoft=4, UnparametrizedSolid=5 };
struct Evidence {
    UUID document{}, entity{}, definition{}, feature{};
    Family family=Family::Profile;
    int schema=0;
    Digest geometry{}, recipe{}, state{};
    std::vector<std::uint8_t> recipeBytes, stateBytes, transformBytes;
    bool operator==(const Evidence& b)const noexcept {
        return document==b.document && entity==b.entity && definition==b.definition
            && feature==b.feature && family==b.family && schema==b.schema
            && geometry==b.geometry && recipe==b.recipe && state==b.state
            && recipeBytes==b.recipeBytes && stateBytes==b.stateBytes;
    }
};
// Raw IEEE754 values, including signed zero, attribute presence and actual
// saved schema/ordered recipe scalars. No reconstruction from display values.
inline void Integer(std::vector<std::uint8_t>& b,std::uint64_t n) {
    for(unsigned i=0;i<8;++i)b.push_back(std::uint8_t(n>>(8*i)));
}
inline void Scalar(std::vector<std::uint8_t>& b,double v) {
    std::uint64_t n;static_assert(sizeof(n)==sizeof(v));std::memcpy(&n,&v,sizeof(n));Integer(b,n);
}
// A finite document unit alone does not imply a finite physical factor.
inline bool MillimetersPerUnit(double unit,double& factor)noexcept {
    factor=0;
    if(!std::isfinite(unit)||unit<=0)return false;
    const double candidate=unit*1000;
    if(!std::isfinite(candidate)||candidate<=0)return false;
    factor=candidate;return true;
}
inline bool Capture(const Handle(OcctDocument)& owner,const TDF_Label& label,Evidence& out)noexcept {
    out={};try{
        if(!pthread_main_np()||owner.IsNull()||owner->Document().IsNull()
            ||owner->Document()->HasOpenCommand()
            ||!XCAFDoc_DocumentTool::CheckShapeTool(owner->Document()->Main())||label.IsNull()
            ||!owner->IsEditableFreeSimpleDefinitionLabel(label))return false;
        OcctObjectNameState named;
        if(!owner->CaptureObjectNameStateForLabel(label,named))return false;
        const auto& s=named.object;Evidence e;
        if(s.resolvedRepresentation!=OcctGeometryRepresentation::BRep
            ||!receipt::ParseUUID(owner->DocumentIdentifier(),e.document)
            ||!receipt::ParseUUID(s.entityIdentifier,e.entity)
            ||!receipt::ParseUUID(s.definitionIdentifier,e.definition))return false;
        const unsigned families=unsigned(!s.profile.label.IsNull())+unsigned(!s.enclosure.label.IsNull())
            +unsigned(!s.sweep.label.IsNull())+unsigned(!s.loft.label.IsNull());
        // CaptureObjectNameStateForLabel already strictly reads every recipe
        // family: malformed/partial/ambiguous metadata never reaches absence.
        if(families>1)return false;
        const std::vector<double> absentValues;
        const std::vector<double>* values=nullptr;std::string feature;
        if(!s.profile.label.IsNull()) {
            if(!s.profile.IsCurrent(owner->Document(),label))return false;
            e.family=Family::Profile;e.schema=profile::SchemaFor(s.profile.parameters);
            values=&s.profile.values;feature=s.profile.identifier;
        }else if(!s.enclosure.label.IsNull()) {
            if(!s.enclosure.IsCurrent(owner->Document(),label))return false;
            e.family=Family::Enclosure;e.schema=s.enclosure.parameters.definition.constructionFrame
                ?enclosure::FramedSchemaVersion:enclosure::SchemaVersion;
            values=&s.enclosure.values;feature=s.enclosure.identifier;
        }else if(!s.sweep.label.IsNull()) {
            if(!s.sweep.IsCurrent(owner->Document(),label))return false;
            e.family=Family::Sweep;e.schema=sweep_persistence::Schema;
            values=&s.sweep.values;feature=s.sweep.identifier;
        }else if(!s.loft.label.IsNull()) {
            if(!s.loft.IsCurrent(owner->Document(),label))return false;
            e.family=Family::RectangularLoft;e.schema=loft_persistence::Schema;
            values=&s.loft.values;feature=s.loft.identifier;
        }else {
            // Explicit absence representation, never a synthesized recipe or
            // feature UUID. Entity/definition UUIDs and exact solid geometry
            // still bind the object. Zero feature bytes mean absence only in
            // this family/schema0 domain.
            e.family=Family::UnparametrizedSolid;e.schema=0;values=&absentValues;
        }
        double unit=0;
        if((families!=0&&!receipt::ParseUUID(feature,e.feature))||!values
            ||(families!=0&&values->empty())||values->size()>std::size_t(profile::MaximumScalars)
            ||!XCAFDoc_DocumentTool::GetLengthUnit(owner->Document(),unit)
            ||!std::isfinite(unit)||unit<=0||named.name.Length()>256)return false;
        e.recipeBytes={'S','Y','P','L','R',1,std::uint8_t(e.family)};
        Integer(e.recipeBytes,std::uint64_t(e.schema));Scalar(e.recipeBytes,unit);
        Integer(e.recipeBytes,values->size());
        for(double v:*values){if(!std::isfinite(v))return false;Scalar(e.recipeBytes,v);}
        if(!receipt::Hash(e.recipeBytes.data(),e.recipeBytes.size(),e.recipe))return false;
        // Reuse the existing exact (analytic=false) bounded OCCT7.8/V3
        // geometry procedure, including its private-copy bookkeeping policy.
        // No new numeric normalization and no promised reopen equality.
        if(!receipt::GeometryDigestForPolicy(s.shape,e.geometry,false))return false;
        e.stateBytes={'S','Y','P','L','S',1};
        auto append=[&](const auto& a){e.stateBytes.insert(e.stateBytes.end(),a.begin(),a.end());};
        append(e.document);append(e.entity);append(e.definition);append(e.feature);
        append(e.recipe);append(e.geometry);
        for(std::size_t i=0;i<s.scalars.size();++i){
            if(!std::isfinite(s.scalars[i]))return false;
            e.transformBytes.push_back(s.present[i]?1:0);Scalar(e.transformBytes,s.scalars[i]);
        }
        append(e.transformBytes);
        e.stateBytes.push_back(named.namePresent?1:0);Integer(e.stateBytes,named.name.Length());
        for(int i=1;i<=named.name.Length();++i)Integer(e.stateBytes,std::uint16_t(named.name.Value(i)));
        if(!receipt::Hash(e.stateBytes.data(),e.stateBytes.size(),e.state))return false;
        out=std::move(e);return true;
    }catch(...){out={};return false;}
}
// Unique current free definition. No entity prefix/mesh suffix guessing.
inline bool Find(const Handle(OcctDocument)& owner,const std::string& entity,TDF_Label& out)noexcept {
    out.Nullify();try{
        if(!pthread_main_np()||owner.IsNull()||owner->Document().IsNull()||entity.empty()
            ||!XCAFDoc_DocumentTool::CheckShapeTool(owner->Document()->Main()))return false;
        TDF_LabelSequence roots;XCAFDoc_DocumentTool::ShapeTool(owner->Document()->Main())->GetFreeShapes(roots);
        if(roots.Length()>50000)return false;
        unsigned matches=0;
        for(int i=1;i<=roots.Length();++i)if(owner->EntityIdentifierForLabel(roots.Value(i))==entity){out=roots.Value(i);++matches;}
        if(matches!=1){out.Nullify();return false;}return true;
    }catch(...){out.Nullify();return false;}
}
}
