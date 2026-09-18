#pragma once
#if DEBUG
#include "SavedCutSourceEdit.hxx"
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <map>
#include <string>

namespace core3d::detached_loft_cut_probe {
// Detached checks also exposed to XCTest, because the exact loft scalar codec
// depends on OCCT headers/libraries and cannot be a library-free prototype.
inline std::map<std::string,bool> Run() noexcept {
    std::map<std::string,bool> out;
    try {
        rectangular_loft::Definition d;d.loftIdentifier=1;d.correspondence={10,11,12,13};d.dimensionMetersPerUnit=.001;
        for(unsigned i=0;i<3;++i){rectangular_loft::Station s;s.identifier=100+i;s.cornerIdentifiers={200+4*i,201+4*i,202+4*i,203+4*i};s.correspondence=d.correspondence;
            s.z=20+60*i;s.centerX=i==1?6:-0.0;s.width=i==1?100:80;s.depth=i==1?60:50;d.stations.push_back(s);}
        retained_solid::Envelope e;for(auto* id:{&e.document,&e.entity,&e.definition,&e.sourceFeature,&e.derivedFeature})(*id)[0]=1;
        e.derivedFeature[0]=2;e.sourceFamily=3;e.sourceSchema=1;e.metersPerUnit=.001;e.radius=12;
        if(!loft_persistence::Encode(d,e.sourceValues))return {{"fixture",false}};
        const auto patched=saved_cut_source_values::Apply(e,saved_cut_source_values::LoftPatch{101,110,64});
        bool exact=patched&&patched->changed;
        if(patched)for(std::size_t i=0;i<e.sourceValues.size();++i)
            exact=exact&&retained_solid::Bits(patched->envelope.sourceValues[i])==retained_solid::Bits(i==34?110:i==35?64:e.sourceValues[i]);
        out["LoftPatchAppliesOnlyAddressedStationScalars"]=exact;
        bool invalid=true;
        for(double value:{0.,-1.,1e20,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()})
            invalid=invalid&&!saved_cut_source_values::Apply(e,saved_cut_source_values::LoftPatch{101,value,{}});
        auto foreign=e;foreign.sourceValues[22]=foreign.sourceValues[8];
        invalid=invalid&&!saved_cut_source_values::Apply(foreign,saved_cut_source_values::LoftPatch{101,110,{}})
            &&!saved_cut_source_values::Apply(e,saved_cut_source_values::LoftPatch{999,110,{}});
        out["LoftPatchRejectsForeignBitsAndOutOfDomainWidths"]=invalid;
        std::vector<std::uint8_t> bytes,again;retained_solid::Envelope decoded;
        out["FamilyThreeEnvelopeValidatesSchemaOne"]=retained_solid::Encode(e,bytes)&&retained_solid::Decode(bytes,decoded)
            &&retained_solid::Encode(decoded,again)&&bytes==again;
        bytes.back()^=1;out["stale-envelope-bytes-refused"]=!retained_solid::Decode(bytes,decoded);
        // Reproduce the historical stored spline shape, and prove that the
        // detached first-cut conversion can replace it with the exact recipe.
        BRepOffsetAPI_ThruSections legacy(Standard_True,Standard_True);legacy.CheckCompatibility(Standard_False);
        for(const auto& station:d.stations){BRepBuilderAPI_MakePolygon wire;
            for(const auto& corner:rectangular_loft::detail::Corners(station))wire.Add(corner);wire.Close();legacy.AddWire(wire.Wire());}
        legacy.Build();const std::atomic_bool stop(false);TopoDS_Shape planar;
        out["legacy-spline-conversion"]=legacy.IsDone()
            &&!saved_cut_source_edit::InspectBase(legacy.Shape(),e,stop)
            &&saved_cut_source_edit::RebuildLoftBase(e,stop,planar)==saved_cut_source_edit::LoftBaseStatus::Built
            &&saved_cut_source_edit::FirstCutBaseMatches(legacy.Shape(),planar,false,e)
            &&!saved_cut_source_edit::FirstCutBaseMatches(legacy.Shape(),planar,true,e);
        const std::atomic_bool cancelled(true);TopoDS_Shape empty;
        out["cancelled-conversion-empty"]=saved_cut_source_edit::RebuildLoftBase(e,cancelled,empty)==saved_cut_source_edit::LoftBaseStatus::Cancelled&&empty.IsNull();
    }catch(...){out["exception"]=false;}
    return out;
}
}
#endif
