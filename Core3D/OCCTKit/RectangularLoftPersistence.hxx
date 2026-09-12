#pragma once

#include "RectangularLoftDefinition.hxx"
#include "SweepPersistence.hxx"
#include "EnclosurePersistence.hxx"
#include <cstdint>
#include <cstring>

// Exact bounded rectangular-loft recipe and current owner binding. Callers own
// transactions; SavedFeatureRecords supplies the shared four-family admission.
namespace core3d::loft_persistence {
using Definition = rectangular_loft::Definition;
inline constexpr int Schema = 1, MinimumScalars = 36, MaximumScalars = 128;
inline constexpr int MinimumRecordTag = 13, MaximumLabels = 100000;
inline const Standard_GUID& SchemaID() {
    static const Standard_GUID id("1CD95086-A4C9-4867-84F4-D63A5379B45D"); return id;
}
inline const Standard_GUID& IdentityID() {
    static const Standard_GUID id("F27C4997-3371-4CA3-B716-3F0DF10BDA41"); return id;
}
inline const Standard_GUID& CountID() {
    static const Standard_GUID id("6041AA30-5099-4624-B9C0-3CE47D2F835B"); return id;
}
// Share the already reviewed exact binary64 comparisons; no normalization.
using sweep_persistence::Bits;
using sweep_persistence::SameBits;
inline bool Encode(const Definition& d,std::vector<double>& output) noexcept {
    output.clear();
    try {
        rectangular_loft::Inspection inspection;
        if (rectangular_loft::Inspect(d,inspection)!=rectangular_loft::Admission::Accepted) return false;
        std::vector<double> values{double(d.loftIdentifier),d.dimensionMetersPerUnit,
            double(d.stations.size()),d.constructionFrame?1.0:0.0};
        for (auto id:d.correspondence) values.push_back(double(id));
        for (const auto& station:d.stations) {
            values.push_back(double(station.identifier));
            for (auto id:station.cornerIdentifiers) values.push_back(double(id));
            for (auto id:station.correspondence) values.push_back(double(id));
            values.insert(values.end(),{station.z,station.centerX,station.centerY,station.width,station.depth});
        }
        if (d.constructionFrame) values.insert(values.end(),d.constructionFrame->values.begin(),d.constructionFrame->values.end());
        if (values.size()<MinimumScalars || values.size()>MaximumScalars) return false;
        output=std::move(values);return true;
    } catch (...) {output.clear();return false;}
}
inline bool Decode(const std::vector<double>& values,Definition& output) noexcept {
    output={};
    try {
        if (values.size()<MinimumScalars || values.size()>MaximumScalars) return false;
        for (double v:values) if (!std::isfinite(v)) return false;
        const auto integer=[](double v,double low,double high) {return v>=low&&v<=high&&v==std::floor(v);};
        const auto identity=[&](double v) {return integer(v,1,double(std::numeric_limits<rectangular_loft::ElementID>::max()));};
        if (!identity(values[0])||!integer(values[2],2,8)||!integer(values[3],0,1)) return false;
        const auto count=std::size_t(values[2]);const bool framed=values[3]==1;
        if (values.size()!=8+14*count+(framed?8:0))return false;
        Definition d;d.loftIdentifier=rectangular_loft::ElementID(values[0]);d.dimensionMetersPerUnit=values[1];
        for (std::size_t i=0;i<4;++i) {if(!identity(values[4+i]))return false;d.correspondence[i]=rectangular_loft::ElementID(values[4+i]);}
        d.stations.reserve(count);std::size_t offset=8;
        for (std::size_t i=0;i<count;++i,offset+=14) {
            for (std::size_t j=0;j<9;++j)if(!identity(values[offset+j]))return false;
            rectangular_loft::Station station;station.identifier=rectangular_loft::ElementID(values[offset]);
            for (std::size_t j=0;j<4;++j) {
                station.cornerIdentifiers[j]=rectangular_loft::ElementID(values[offset+1+j]);
                station.correspondence[j]=rectangular_loft::ElementID(values[offset+5+j]);
            }
            station.z=values[offset+9];station.centerX=values[offset+10];station.centerY=values[offset+11];
            station.width=values[offset+12];station.depth=values[offset+13];d.stations.push_back(station);
        }
        if(framed) {profile::ConstructionFrame frame;std::copy_n(values.begin()+offset,8,frame.values.begin());d.constructionFrame=frame;}
        std::vector<double> encoded;if(!Encode(d,encoded)||!SameBits(values,encoded))return false;
        output=std::move(d);return true;
    } catch (...) {output={};return false;}
}
inline bool HasAttribute(const TDF_Label& label) {
    return label.IsAttribute(SchemaID()) || label.IsAttribute(IdentityID()) || label.IsAttribute(CountID());
}
struct Record {
    TDF_Label label;
    std::string identifier;
    Definition definition;
    std::vector<double> values;
    TopoDS_Shape boundShape;
    bool IsEqual(const Record& other) const {
        if (label.IsNull() || other.label.IsNull()) return label.IsNull() && other.label.IsNull();
        return label.IsEqual(other.label) && label.Data()==other.label.Data()
            && identifier==other.identifier && SameBits(values,other.values)
            && !boundShape.IsNull() && !other.boundShape.IsNull() && boundShape.IsEqual(other.boundShape);
    }
    bool IsCurrent(const Handle(TDocStd_Document)& document,const TDF_Label& owner) const {
        double unit=0;
        return !document.IsNull() && !owner.IsNull() && !label.IsNull()
            && owner.Data()==document->GetData() && label.Data()==document->GetData()
            && label.Father().IsEqual(owner) && !boundShape.IsNull()
            && boundShape.IsEqual(XCAFDoc_ShapeTool::GetShape(owner))
            && XCAFDoc_DocumentTool::GetLengthUnit(document,unit)
            && Bits(unit)==Bits(definition.dimensionMetersPerUnit);
    }
};
// true + null label means absent. Partial/malformed-present returns false.
inline bool Read(const Handle(TDocStd_Document)& document,const TDF_Label& owner,Record& output) noexcept {
    output={};
    try {
        if (document.IsNull() || owner.IsNull() || owner.Data()!=document->GetData()) return false;
        Record r; int visited=0;
        for (TDF_ChildIterator it(owner,Standard_False);it.More();it.Next()) {
            if (++visited>MaximumLabels) return false;
            if (!HasAttribute(it.Value())) continue;
            if (!r.label.IsNull() || it.Value().Tag()<MinimumRecordTag) return false;
            r.label=it.Value();
        }
        if (r.label.IsNull()) return true;
        if (!XCAFDoc_DocumentTool::CheckShapeTool(document->Main())
            || !XCAFDoc_ShapeTool::IsSimpleShape(owner) || !XCAFDoc_ShapeTool::IsFree(owner)) return false;
        profile::Record p; enclosure::Record e; sweep_persistence::Record sweep;
        if (!profile::Read(document,owner,p) || !p.label.IsNull()
            || !enclosure::Read(document,owner,e) || !e.label.IsNull()
            || !sweep_persistence::Read(document,owner,sweep) || !sweep.label.IsNull()) return false;
        Handle(TDataStd_Integer) schema,count; Handle(TDataStd_AsciiString) identity;
        Handle(TNaming_NamedShape) binding;
        if (!r.label.FindAttribute(SchemaID(),schema) || schema->Get()!=Schema
            || !r.label.FindAttribute(CountID(),count) || count->Get()<MinimumScalars || count->Get()>MaximumScalars
            || !r.label.FindAttribute(IdentityID(),identity)
            || !r.label.FindAttribute(TNaming_NamedShape::GetID(),binding)) return false;
        if (identity->Get().Length()!=36) return false;
        r.identifier=identity->Get().ToCString(); if (!profile::IsIdentifier(r.identifier)) return false;
        r.boundShape=binding->Get();
        if (r.boundShape.IsNull() || r.boundShape.ShapeType()!=TopAbs_SOLID) return false;
        for (TDF_AttributeIterator it(r.label);it.More();it.Next()) {
            const auto& id=it.Value()->ID();
            if (id!=SchemaID() && id!=CountID() && id!=IdentityID() && id!=TNaming_NamedShape::GetID()) return false;
        }
        r.values.resize(count->Get()); int present=0;
        for (TDF_ChildIterator it(r.label,Standard_False);it.More();it.Next()) {
            const auto child=it.Value(); if (++visited>MaximumLabels || child.Tag()<1) return false;
            for (TDF_ChildIterator nested(child,Standard_True);nested.More();nested.Next())
                if (++visited>MaximumLabels || nested.Value().HasAttribute()) return false;
            if (!child.HasAttribute()) continue;
            if (child.Tag()>count->Get()) return false;
            Handle(TDataStd_Real) scalar;
            if (!child.FindAttribute(TDataStd_Real::GetID(),scalar) || !std::isfinite(scalar->Get())) return false;
            for (TDF_AttributeIterator attr(child);attr.More();attr.Next())
                if (attr.Value()->ID()!=TDataStd_Real::GetID()) return false;
            r.values[child.Tag()-1]=scalar->Get(); ++present;
        }
        if (present!=count->Get() || !Decode(r.values,r.definition) || !r.IsCurrent(document,owner)) return false;
        output=std::move(r); return true;
    } catch (...) { output={}; return false; }
}
// Caller owns the command and must abort/retain it on failure. No implicit transaction.
inline bool Stage(const Handle(TDocStd_Document)& document,const TDF_Label& owner,
                  const Definition& definition,const std::string& identifier) noexcept {
    try {
        Record previous; profile::Record p; enclosure::Record e; sweep_persistence::Record sweep; std::vector<double> values; double unit=0;
        if (document.IsNull() || !document->HasOpenCommand() || !profile::IsIdentifier(identifier)
            || !Encode(definition,values) || !Read(document,owner,previous)
            || !profile::Read(document,owner,p) || !p.label.IsNull()
            || !enclosure::Read(document,owner,e) || !e.label.IsNull()
            || !sweep_persistence::Read(document,owner,sweep) || !sweep.label.IsNull()
            || !XCAFDoc_ShapeTool::IsSimpleShape(owner) || !XCAFDoc_ShapeTool::IsFree(owner)
            || !XCAFDoc_DocumentTool::GetLengthUnit(document,unit) || Bits(unit)!=Bits(definition.dimensionMetersPerUnit)
            || (!previous.label.IsNull() && previous.identifier!=identifier)) return false;
        auto shape=XCAFDoc_ShapeTool::GetShape(owner);
        if (shape.IsNull() || shape.ShapeType()!=TopAbs_SOLID) return false;
        auto label=previous.label;
        if (label.IsNull()) {
            int maximumTag=MinimumRecordTag-1,visited=0;
            for (TDF_ChildIterator it(owner,Standard_False);it.More();it.Next()) {
                if (++visited>MaximumLabels) return false;
                maximumTag=std::max(maximumTag,it.Value().Tag());
            }
            if (maximumTag==std::numeric_limits<int>::max()) return false;
            label=owner.FindChild(maximumTag+1,Standard_True);
        }
        TDataStd_Integer::Set(label,SchemaID(),Schema);
        TDataStd_Integer::Set(label,CountID(),int(values.size()));
        TDataStd_AsciiString::Set(label,IdentityID(),TCollection_AsciiString(identifier.c_str()));
        TNaming_Builder(label).Select(shape,shape);
        for (TDF_ChildIterator it(label,Standard_False);it.More();it.Next()) it.Value().ForgetAllAttributes(Standard_True);
        for (std::size_t i=0;i<values.size();++i) TDataStd_Real::Set(label.FindChild(int(i)+1,Standard_True),values[i]);
        Record stored;
        return Read(document,owner,stored) && stored.identifier==identifier && SameBits(stored.values,values);
    } catch (...) { return false; }
}
} // namespace core3d::loft_persistence
