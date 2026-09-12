#pragma once
#include "RectangularLoftPersistence.hxx"

namespace core3d::saved_features {
// One traversal/aggregate budget and feature-identity namespace for all families.
// Existing profile/enclosure codec semantics are retained, including legacy records.
inline bool Validate(const Handle(TDocStd_Document)& document,
    std::vector<profile::Record>& profiles,std::vector<enclosure::Record>& enclosures,
    std::vector<sweep_persistence::Record>& sweeps,std::vector<loft_persistence::Record>& lofts) noexcept {
    profiles.clear();enclosures.clear();sweeps.clear();lofts.clear();
    const auto clear=[&] {profiles.clear();enclosures.clear();sweeps.clear();lofts.clear();return false;};
    try {
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const auto root=document->GetData()->Root();
        const auto marked=[](const TDF_Label& l) {return profile::HasAttribute(l)||enclosure::HasAttribute(l)||sweep_persistence::HasAttribute(l)||loft_persistence::HasAttribute(l);};
        if (marked(root)) return false;
        int visited=0;std::set<std::string> identities;
        for (TDF_ChildIterator it(root,Standard_True);it.More();it.Next()) {
            if (++visited>profile::MaximumLabels) return clear();
            const auto label=it.Value();if (!marked(label)) continue;
            if (identities.size()>=profile::MaximumRecords) return clear();
            profile::Record p;enclosure::Record e;sweep_persistence::Record s;loft_persistence::Record loft;
            if (!profile::Read(document,label.Father(),p)||!enclosure::Read(document,label.Father(),e)
                ||!sweep_persistence::Read(document,label.Father(),s)||!loft_persistence::Read(document,label.Father(),loft)) return clear();
            const int families=int(!p.label.IsNull())+int(!e.label.IsNull())+int(!s.label.IsNull())+int(!loft.label.IsNull());
            if (families!=1) return clear();
            if (!p.label.IsNull()) {
                if (!p.label.IsEqual(label)||!identities.insert(p.identifier).second) return clear();
                profiles.push_back(std::move(p));
            } else if (!e.label.IsNull()) {
                if (!e.label.IsEqual(label)||!identities.insert(e.identifier).second) return clear();
                enclosures.push_back(std::move(e));
            } else if (!s.label.IsNull()) {
                if (!s.label.IsEqual(label)||!identities.insert(s.identifier).second) return clear();
                sweeps.push_back(std::move(s));
            } else {
                if (!loft.label.IsEqual(label)||!identities.insert(loft.identifier).second) return clear();
                lofts.push_back(std::move(loft));
            }
        }
        return true;
    } catch (...) {return clear();}
}
} // namespace core3d::saved_features
