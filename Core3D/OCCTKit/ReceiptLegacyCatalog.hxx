#pragma once
#include "ReceiptRecord.hxx"
#include <TDocStd_Document.hxx>
#include <TDF_Data.hxx>
#include <TDF_TagSource.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_AttributeIterator.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TDataStd_Integer.hxx>
namespace core3d::receipt {
// label/bytes describe only the versioned component. Legacy storage is never
// rewritten. All authority fences compare both exact components through matches.
struct LegacyCatalog {
    TDF_Label label,legacyLabel,v3Label;
    std::vector<Record> records; // Combined sorted request inventory, including legacy.
    std::vector<std::uint8_t> bytes,legacyBytes;
    bool matches(const LegacyCatalog& b)const noexcept {
        return label==b.label&&legacyLabel==b.legacyLabel&&v3Label==b.v3Label&&bytes==b.bytes&&legacyBytes==b.legacyBytes;
    }
    bool supportsAppend()const noexcept {
        return std::all_of(records.begin(),records.end(),[](const Record&r){return r.policy==0||SupportedPolicy(r.operation,r.policy);});
    }
};
inline bool HasCatalogAttribute(const TDF_Label& label){
    return label.IsAttribute(ScalableSchemaID())||label.IsAttribute(SchemaID())||label.IsAttribute(CountID())
        ||label.IsAttribute(VersionedSchemaID())||label.IsAttribute(VersionedCountID());
}
inline ReadStatus ReadLegacy(const Handle(TDocStd_Document)& doc,LegacyCatalog& out) noexcept {
    out={};try{
        if(doc.IsNull()||doc->GetData().IsNull())return ReadStatus::Unavailable;
        const auto root=doc->GetData()->Root();
        if(HasCatalogAttribute(root)||HasCatalogAttribute(doc->Main()))return ReadStatus::Malformed;
        LegacyCatalog found;std::size_t visited=0;
        // Exactly one traversal pays the shared label budget, including chunk
        // descendants. No per-catalog reset can double this ceiling.
        for(TDF_ChildIterator it(root,Standard_True);it.More();it.Next()){
            if(++visited>100000)return ReadStatus::Malformed;
            const auto label=it.Value();if(!HasCatalogAttribute(label))continue;
            if(label==doc->Main()||label.Father()!=root)return ReadStatus::Malformed;
            if(label.IsAttribute(ScalableSchemaID())){
                if(!found.v3Label.IsNull())return ReadStatus::Malformed;found.v3Label=label;
                for(TDF_AttributeIterator a(label);a.More();a.Next())if(a.Value()->ID()!=ScalableSchemaID()&&a.Value()->ID()!=TDF_TagSource::GetID())return ReadStatus::Malformed;
                for(TDF_ChildIterator child(label,Standard_True);child.More();child.Next()){
                    if(++visited>100000||child.Value().HasAttribute())return ReadStatus::Malformed;
                }
                continue;
            }
            const bool legacy=label.IsAttribute(SchemaID())||label.IsAttribute(CountID());
            const bool versioned=label.IsAttribute(VersionedSchemaID())||label.IsAttribute(VersionedCountID());
            if(legacy==versioned)return ReadStatus::Malformed;
            auto& slot=legacy?found.legacyLabel:found.label;
            if(!slot.IsNull())return ReadStatus::Malformed;slot=label;
        }
        if(found.label.IsNull()&&found.legacyLabel.IsNull()&&found.v3Label.IsNull())return ReadStatus::Absent;
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

}
