#pragma once

#include "PlanarSweepDefinition.hxx"
#include "EnclosurePersistence.hxx"
#include <cstdint>
#include <cstring>

// Component only: callers own transactions and document-wide admission.
// No loader, ordinary command, copy or request capability is installed here.
namespace core3d::sweep_persistence {
using Definition = planar_sweep::Definition;
inline constexpr int Schema = 2, MinimumScalars = 24, MaximumScalars = 405;
inline constexpr int MinimumRecordTag = 13, MaximumLabels = 100000;
inline const Standard_GUID& SchemaID() {
    static const Standard_GUID id("FE068D01-49F0-4694-8E80-B6B30AAF45E7"); return id;
}
inline const Standard_GUID& IdentityID() {
    static const Standard_GUID id("E599B067-47D5-44FB-B77F-9F17EEFF97DE"); return id;
}
inline const Standard_GUID& CountID() {
    static const Standard_GUID id("78BD2966-2904-4CF5-A3E2-551DF2B251CB"); return id;
}
inline std::uint64_t Bits(double value) noexcept {
    static_assert(sizeof(double)==sizeof(std::uint64_t));
    std::uint64_t bits; std::memcpy(&bits,&value,sizeof(bits)); return bits;
}
inline bool SameBits(const std::vector<double>& a,const std::vector<double>& b) noexcept {
    if (a.size()!=b.size()) return false;
    for (std::size_t i=0;i<a.size();++i) if (Bits(a[i])!=Bits(b[i])) return false;
    return true;
}
inline bool Encode(const Definition& d,std::vector<double>& output) noexcept {
    output.clear();
    try {
        planar_sweep::Inspection inspection;
        if (planar_sweep::Inspect(d,inspection)!=planar_sweep::Admission::Accepted) return false;
        std::vector<double> values{double(d.plane),0,0,double(d.pathIdentifier),d.radius,d.EndRadius(),
            d.dimensionMetersPerUnit,double(d.vertices.size()),double(d.segments.size()),
            d.constructionFrame?1.0:0.0};
        for (const auto& v:d.vertices)
            values.insert(values.end(),{double(v.identifier),v.point.X(),v.point.Y()});
        for (const auto& s:d.segments)
            values.insert(values.end(),{double(s.identifier),double(static_cast<int>(s.kind)),
                double(s.startVertex),double(s.endVertex),s.center.X(),s.center.Y(),
                s.radius,s.startDegrees,s.sweepDegrees});
        if (d.constructionFrame) values.insert(values.end(),d.constructionFrame->values.begin(),d.constructionFrame->values.end());
        if (values.size()<MinimumScalars || values.size()>MaximumScalars) return false;
        output=std::move(values); return true;
    } catch (...) { output.clear(); return false; }
}
inline bool Decode(const std::vector<double>& values,Definition& output,int schema) noexcept {
    output={};
    try {
        if ((schema!=1 && schema!=Schema) || values.size()<MinimumScalars || values.size()>MaximumScalars) return false;
        const std::size_t shift=schema==1 ? 0 : 1;
        for (double v:values) if (!std::isfinite(v)) return false;
        const auto integer=[](double v,double low,double high) { return v>=low && v<=high && v==std::floor(v); };
        const auto identity=[&](double v) { return integer(v,1,double(std::numeric_limits<ProfileCurveID>::max())); };
        if (!integer(values[0],0,2) || values[1]!=0 || values[2]!=0 || !identity(values[3])
            || !integer(values[6+shift],2,33) || !integer(values[7+shift],1,32)
            || values[6+shift]!=values[7+shift]+1 || !integer(values[8+shift],0,1)) return false;
        const std::size_t vertices=std::size_t(values[6+shift]),segments=std::size_t(values[7+shift]);
        const bool framed=values[8+shift]==1;
        if (values.size()!=9+shift+3*vertices+9*segments+(framed?8:0)) return false;
        Definition d; d.plane=int(values[0]); d.pathIdentifier=ProfileCurveID(values[3]);
        d.radius=values[4]; d.endRadius=schema==1 ? d.radius : values[5]; d.dimensionMetersPerUnit=values[5+shift];
        d.vertices.reserve(vertices); d.segments.reserve(segments);
        std::size_t offset=9+shift;
        for (std::size_t i=0;i<vertices;++i,offset+=3) {
            if (!identity(values[offset])) return false;
            d.vertices.push_back({ProfileCurveID(values[offset]),gp_Pnt2d(values[offset+1],values[offset+2])});
        }
        for (std::size_t i=0;i<segments;++i,offset+=9) {
            if (!identity(values[offset]) || !integer(values[offset+1],0,1)
                || !identity(values[offset+2]) || !identity(values[offset+3])) return false;
            ProfileCurveSegment s; s.identifier=ProfileCurveID(values[offset]);
            s.kind=ProfileCurveKind(int(values[offset+1])); s.startVertex=ProfileCurveID(values[offset+2]);
            s.endVertex=ProfileCurveID(values[offset+3]); s.center=gp_Pnt2d(values[offset+4],values[offset+5]);
            s.radius=values[offset+6]; s.startDegrees=values[offset+7]; s.sweepDegrees=values[offset+8];
            d.segments.push_back(s);
        }
        if (framed) {
            profile::ConstructionFrame frame;
            std::copy_n(values.begin()+offset,8,frame.values.begin()); d.constructionFrame=frame;
        }
        std::vector<double> encoded;
        // Scalar tags/IDs are canonical integers; authored geometry/frame bits survive exactly.
        if (!Encode(d,encoded)) return false;
        if (schema==1) encoded.erase(encoded.begin()+5);
        if (!SameBits(values,encoded)) return false;
        output=std::move(d); return true;
    } catch (...) { output={}; return false; }
}
// Bare numeric callers may reopen either historical layout. Their lengths are
// disjoint modulo 12 (v1: 0/8, v2: 1/9); persisted records always pass their schema explicitly.
inline bool Decode(const std::vector<double>& values,Definition& output) noexcept {
    const auto remainder=values.size()%12;
    return Decode(values,output,(remainder==0 || remainder==8) ? 1 : Schema);
}
inline bool HasAttribute(const TDF_Label& label) {
    return label.IsAttribute(SchemaID()) || label.IsAttribute(IdentityID()) || label.IsAttribute(CountID());
}
struct Record {
    int schema = Schema;
    TDF_Label label;
    std::string identifier;
    Definition definition;
    std::vector<double> values;
    TopoDS_Shape boundShape;
    bool IsEqual(const Record& other) const {
        if (label.IsNull() || other.label.IsNull()) return label.IsNull() && other.label.IsNull();
        return label.IsEqual(other.label) && label.Data()==other.label.Data()
            && schema==other.schema && identifier==other.identifier && SameBits(values,other.values)
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
        profile::Record p; enclosure::Record e;
        if (!profile::Read(document,owner,p) || !p.label.IsNull()
            || !enclosure::Read(document,owner,e) || !e.label.IsNull()) return false;
        Handle(TDataStd_Integer) schema,count; Handle(TDataStd_AsciiString) identity;
        Handle(TNaming_NamedShape) binding;
        if (!r.label.FindAttribute(SchemaID(),schema) || (schema->Get()!=1 && schema->Get()!=Schema)
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
        r.schema=schema->Get();
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
        if (present!=count->Get() || !Decode(r.values,r.definition,r.schema) || !r.IsCurrent(document,owner)) return false;
        output=std::move(r); return true;
    } catch (...) { output={}; return false; }
}
// Caller owns the command and must abort/retain it on failure. No implicit transaction.
inline bool Stage(const Handle(TDocStd_Document)& document,const TDF_Label& owner,
                  const Definition& definition,const std::string& identifier) noexcept {
    try {
        Record previous; profile::Record p; enclosure::Record e; std::vector<double> values; double unit=0;
        if (document.IsNull() || !document->HasOpenCommand() || !profile::IsIdentifier(identifier)
            || !Encode(definition,values) || !Read(document,owner,previous)
            || !profile::Read(document,owner,p) || !p.label.IsNull()
            || !enclosure::Read(document,owner,e) || !e.label.IsNull()
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
} // namespace core3d::sweep_persistence
