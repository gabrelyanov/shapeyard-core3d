#pragma once
#include "SweepPersistence.hxx"

// Read-only admission shared by capture and the private ordinary staging path.
// No alternate codec mode and no authority to rebind a stale record.
namespace core3d::sweep_rebuild {
inline bool FixedStructure(const planar_sweep::Definition& a,
                           const planar_sweep::Definition& b) noexcept {
    if (a.pathIdentifier!=b.pathIdentifier || a.plane!=b.plane
        || sweep_persistence::Bits(a.dimensionMetersPerUnit)!=sweep_persistence::Bits(b.dimensionMetersPerUnit)
        || a.vertices.size()!=b.vertices.size() || a.segments.size()!=b.segments.size()
        || bool(a.constructionFrame)!=bool(b.constructionFrame)) return false;
    if (a.constructionFrame) for (std::size_t i=0;i<8;++i)
        if (sweep_persistence::Bits(a.constructionFrame->values[i])
            !=sweep_persistence::Bits(b.constructionFrame->values[i])) return false;
    for (std::size_t i=0;i<a.vertices.size();++i)
        if (a.vertices[i].identifier!=b.vertices[i].identifier) return false;
    for (std::size_t i=0;i<a.segments.size();++i) {
        const auto& x=a.segments[i];const auto& y=b.segments[i];
        if (x.identifier!=y.identifier || x.kind!=y.kind
            || x.startVertex!=y.startVertex || x.endVertex!=y.endVertex) return false;
    }
    return true;
}
inline bool HasOnlyMetadataSubshapes(const Handle(TDocStd_Document)& document,
                                    const TDF_Label& owner) noexcept {
    try {
        sweep_persistence::Record record;
        if (!sweep_persistence::Read(document,owner,record) || record.label.IsNull()) return false;
        TDF_LabelSequence children;XCAFDoc_ShapeTool::GetSubShapes(owner,children);
        if (children.Length()>profile::MaximumLabels) return false;
        for (int i=1;i<=children.Length();++i)
            if (!children.Value(i).IsEqual(record.label)) return false;
        return true;
    } catch (...) {return false;}
}
template<class A,class B> inline bool SameRawScalars(const A& a,const B& b) noexcept {
    if (a.size()!=b.size()) return false;
    for (std::size_t i=0;i<a.size();++i)
        if (sweep_persistence::Bits(a[i])!=sweep_persistence::Bits(b[i])) return false;
    return true;
}
}
