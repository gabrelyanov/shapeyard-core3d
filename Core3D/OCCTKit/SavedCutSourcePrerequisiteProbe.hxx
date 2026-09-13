#pragma once
#if DEBUG
#include "SavedCutSourceValuePatch.hxx"
#include "PrismExtractorProbe.hxx"
#include "RetainedSolidProbe.hxx"
#include "EnclosureDefinition.hxx"
#include <BRepTools.hxx>
#include <Geom2d_Line.hxx>
#include <gp_Lin2d.hxx>
#include <sstream>
#include <map>
#include <limits>

namespace core3d::saved_cut_source_prerequisite_probe {
using Checks=std::map<std::string,bool>;
using Envelope=retained_solid::Envelope;
using namespace saved_cut_source_values;
inline Envelope Identity(double unit){
    Envelope e;e.document.fill(1);e.entity.fill(2);e.definition.fill(3);e.sourceFeature.fill(4);e.derivedFeature.fill(5);
    e.metersPerUnit=unit;e.radius=.001/unit;e.point={-0.0,.03/unit,0};return e;
}
inline profile::ConstructionFrame Frame(double k){
    profile::ConstructionFrame f;f.values={13*k,-0.0,7*k,0,0,std::sin(.2),std::cos(.2),1.25};return f;
}
inline std::vector<std::uint8_t> Bytes(const Envelope& e){
    std::vector<std::uint8_t> b;if(!retained_solid::Encode(e,b))throw std::invalid_argument("probe envelope");return b;
}
inline bool Preserved(const Envelope& original,const Envelope& result,const std::vector<std::size_t>& allowed){
    if(original.sourceValues.size()!=result.sourceValues.size())return false;auto restored=result;
    for(auto i:allowed){if(i>=restored.sourceValues.size())return false;restored.sourceValues[i]=original.sourceValues[i];}
    return Bytes(restored)==Bytes(original); // complete wire incl IDs/tool/frame/raw omitted values
}
inline Checks EnclosureValues(){
    Checks out;
    for(double unit:{.001,1.0})for(bool framed:{false,true}){
        const std::string prefix=std::string(unit==.001?"mm":"metre")+(framed?".schema2.":".schema1.");
        const double k=.001/unit;enclosure::Parameters p;p.metersPerUnit=unit;
        p.definition.dimensions={100*k,60*k,30*k,2*k,2*k,20*k};if(framed)p.definition.constructionFrame=Frame(k);
        auto e=Identity(unit);e.sourceFamily=2;e.sourceSchema=framed?2:1;
        if(!enclosure::Encode(p,e.sourceValues))throw std::invalid_argument("enclosure encode");e.sourceValues[1]=-0.0;
        const auto before=Bytes(e);EnclosurePatch onlyWidth;onlyWidth.dimensions[0]=10*k;
        out[prefix+"intermediateWidthRefuses"]=!Apply(e,onlyWidth);
        EnclosurePatch atomic;atomic.dimensions[0]=10*k;atomic.dimensions[5]=4*k;const auto changed=Apply(e,atomic);
        out[prefix+"atomicWidthRadiusAdmitted"]=changed&&changed->changed
            &&retained_solid::Bits(changed->envelope.sourceValues[2])==retained_solid::Bits(10*k)
            &&retained_solid::Bits(changed->envelope.sourceValues[7])==retained_solid::Bits(4*k);
        out[prefix+"allOmittedWireBitsExact"]=changed&&Preserved(e,changed->envelope,{2,7});
        enclosure::Parameters decoded;out[prefix+"decodedFinalDimensions"]=changed
            &&enclosure::Decode(int(e.sourceSchema),changed->envelope.sourceValues,decoded)
            &&decoded.definition.dimensions.width==10*k&&decoded.definition.dimensions.cornerRadius==4*k
            &&decoded.definition.dimensions.wall==2*k&&decoded.definition.dimensions.floor==2*k;
        EnclosurePatch noChange;for(unsigned i=0;i<6;++i)noChange.dimensions[i]=e.sourceValues[2+i];
        const auto same=Apply(e,noChange);out[prefix+"numericNoOpExactWire"]=same&&!same->changed&&Bytes(same->envelope)==before;
        EnclosurePatch invalid;invalid.dimensions[3]=25*k;out[prefix+"invalidFinalRefuses"]=!Apply(e,invalid);
        invalid={};invalid.dimensions[0]=std::numeric_limits<double>::quiet_NaN();out[prefix+"nonfiniteRefuses"]=!Apply(e,invalid);
        out[prefix+"wrongPatchFamilyRefuses"]=!Apply(e,PolygonPatch{});
        auto mismatched=e;mismatched.metersPerUnit=unit==1?.001:1;out[prefix+"unitMismatchRefuses"]=!Apply(mismatched,atomic);
        auto negative=p;negative.definition.constructionFrame=Frame(k);negative.definition.constructionFrame->values[7]=-1.25;
        auto reflected=e;reflected.sourceSchema=2;const bool negativeEncoded=enclosure::Encode(negative,reflected.sourceValues);
        out[prefix+"negativeSourceFrameRefuses"]=negativeEncoded&&retained_solid::Valid(reflected)&&!Apply(reflected,atomic);
        out[prefix+"originalStillExact"]=Bytes(e)==before;
    }return out;
}
inline Checks PolygonValues(){
    Checks out;
    for(double unit:{.001,1.0})for(bool framed:{false,true}){
        const std::string prefix=std::string(unit==.001?"mm":"metre")+(framed?".schema2.":".schema1.");
        const double k=.001/unit;profile::Parameters p;p.metersPerUnit=unit;
        p.definition=saved_cut_prism_prototype::probe::L(unit*1000,1,false);p.definition.points[0].SetX(-0.0);
        if(framed)p.constructionFrame=Frame(k);auto e=Identity(unit);e.sourceFamily=1;e.sourceSchema=profile::SchemaFor(p);
        if(!profile::Encode(p,e.sourceValues))throw std::invalid_argument("polygon encode");const auto before=Bytes(e);
        PolygonPatch patch;patch.depth=9*k;patch.coordinates={{1,Component::U,62*k},{2,Component::U,62*k},{4,Component::V,51*k},{5,Component::V,51*k}};
        const auto changed=Apply(e,patch);out[prefix+"depthOrdinalComponentsAdmitted"]=changed&&changed->changed;
        out[prefix+"allOmittedWireBitsExact"]=changed&&Preserved(e,changed->envelope,{1,9,11,16,18});
        profile::Parameters decoded;out[prefix+"decodedExactPatchedValues"]=changed&&profile::Decode(changed->envelope.sourceValues,decoded)
            &&retained_solid::Bits(decoded.definition.depth)==retained_solid::Bits(9*k)
            &&retained_solid::Bits(decoded.definition.points[1].X())==retained_solid::Bits(62*k)
            &&retained_solid::Bits(decoded.definition.points[2].X())==retained_solid::Bits(62*k)
            &&retained_solid::Bits(decoded.definition.points[4].Y())==retained_solid::Bits(51*k)
            &&retained_solid::Bits(decoded.definition.points[5].Y())==retained_solid::Bits(51*k);
        PolygonPatch sameValue;sameValue.depth=p.definition.depth;sameValue.coordinates={{0,Component::U,0.0}};
        const auto same=Apply(e,sameValue);out[prefix+"signedZeroNoOpExactWire"]=same&&!same->changed&&Bytes(same->envelope)==before
            &&retained_solid::Bits(same->envelope.sourceValues[7])==retained_solid::Bits(-0.0);
        auto duplicate=patch;duplicate.coordinates.push_back(patch.coordinates[0]);out[prefix+"duplicateCoordinateRefuses"]=!Apply(e,duplicate);
        PolygonPatch invalid;invalid.coordinates={{6,Component::U,1}};out[prefix+"invalidOrdinalRefuses"]=!Apply(e,invalid);
        invalid.coordinates={{0,static_cast<Component>(2),1}};out[prefix+"invalidComponentRefuses"]=!Apply(e,invalid);
        invalid.coordinates={{0,Component::U,std::numeric_limits<double>::infinity()}};out[prefix+"nonfiniteRefuses"]=!Apply(e,invalid);
        invalid={};invalid.depth=0;out[prefix+"invalidDepthRefuses"]=!Apply(e,invalid);
        out[prefix+"wrongPatchFamilyRefuses"]=!Apply(e,EnclosurePatch{});
        auto reflected=p;reflected.constructionFrame=Frame(k);reflected.constructionFrame->values[7]=-1.25;auto negative=e;negative.sourceSchema=2;
        const bool encoded=profile::Encode(reflected,negative.sourceValues);out[prefix+"negativeSourceFrameRefuses"]=encoded&&retained_solid::Valid(negative)&&!Apply(negative,patch);
        EnclosureProfileDependency curved;EnclosureDefinition enclosure;
        if(!DeriveEnclosureProfiles(enclosure,curved))throw std::invalid_argument("curved source");
        auto curves=p;curves.definition=curved.outer;auto unsupported=e;unsupported.sourceSchema=profile::SchemaFor(curves);
        const bool curveEncoded=profile::Encode(curves,unsupported.sourceValues);
        out[prefix+"actualCurveSchemaRefuses"]=curveEncoded&&retained_solid::Valid(unsupported)&&unsupported.sourceSchema==(framed?4:3)&&!Apply(unsupported,patch);
        out[prefix+"originalStillExact"]=Bytes(e)==before;
    }return out;
}
inline std::string FullBRep(const TopoDS_Shape& shape){
    if(shape.IsNull())throw std::invalid_argument("null probe shape");std::ostringstream out;out.imbue(std::locale::classic());
    BRepTools::Write(shape,out,Standard_True,Standard_True,TopTools_FormatVersion_VERSION_3);
    if(!out.good()||out.str().empty()||out.str().size()>4*1024*1024)throw std::invalid_argument("probe brep");return out.str();
}
inline Checks Reopened(){
    using namespace saved_cut_prism_prototype;Checks out;std::atomic_bool stop{false};
    for(double unit:{.001,1.0})for(int plane=0;plane<3;++plane)for(bool reversed:{false,true}){
        const auto prefix=std::string(unit==.001?"mm":"metre")+".plane"+std::to_string(plane)+(reversed?".reverse.":".forward.");
        const auto recipe=probe::L(unit*1000,plane,reversed);const auto frame=Frame(.001/unit);const auto shape=probe::Build(recipe,frame);
        const auto original=FullBRep(shape);TopoDS_Shape reopened;BRep_Builder builder;
        std::istringstream input(original);input.imbue(std::locale::classic());BRepTools::Read(reopened,input,builder);Inspection report;
        out[prefix+"freshBRepMatches"]=!reopened.IsNull()&&InspectPrism(reopened,recipe,frame,unit,stop,report)==Classification::MatchedBoundary;
        out[prefix+"nativeBoundaryCounts"]=report.vertices==12&&report.edges==18&&report.faces==8;
        auto wrong=recipe;wrong.depth=9/(unit*1000);out[prefix+"freshWrongDepthRefuses"]=InspectPrism(reopened,wrong,frame,unit,stop,report)==Classification::Refused;
        const auto reopenedBytes=FullBRep(reopened);BRepBuilderAPI_Copy copy(reopened,Standard_True,Standard_False);const auto corrupted=copy.Shape();
        TopExp_Explorer faces(corrupted,TopAbs_FACE);bool injected=false;
        if(faces.More()){const auto face=TopoDS::Face(faces.Current());TopExp_Explorer edges(face,TopAbs_EDGE);
            if(edges.More()){const auto edge=TopoDS::Edge(edges.Current());double first,last;const auto pc=BRep_Tool::CurveOnSurface(edge,face,first,last);
                if(!pc.IsNull()&&std::isfinite(first)&&std::isfinite(last)&&first<last){
                    const auto a=pc->Value(first),b=pc->Value(last);gp_Vec2d direction(a,b);
                    if(direction.SquareMagnitude()>0){Handle(Geom2d_Line) changed=new Geom2d_Line(gp_Lin2d(gp_Pnt2d(a.X()+.1,a.Y()+.2),gp_Dir2d(direction)));
                        builder.UpdateEdge(edge,changed,face,BRep_Tool::Tolerance(edge));injected=true;}
                }
            }
        }
        out[prefix+"actualPCurveMismatchRefuses"]=injected&&InspectPrism(corrupted,recipe,frame,unit,stop,report)==Classification::Refused;
        out[prefix+"originalsRemainByteExact"]=FullBRep(shape)==original&&FullBRep(reopened)==reopenedBytes;
    }
    for(bool xcaf:{false,true})for(int version:{10,11,12})for(double unit:{.001,1.0}){
        const auto prefix=std::string(xcaf?"xcaf":"ocaf")+"."+std::to_string(version)+(unit==.001?".mm.":".metre.");
        retained_solid::Probe::App app;const auto fixture=retained_solid::Probe::New(app,xcaf,version,unit);
        profile::Parameters p;if(!profile::Decode(fixture.envelope.sourceValues,p))throw std::invalid_argument("carrier source");
        profile::ConstructionFrame frame;Inspection report;const auto original=FullBRep(fixture.base);
        out[prefix+"originalCarrierMatches"]=InspectPrism(fixture.base,p.definition,frame,unit,stop,report)==Classification::MatchedBoundary;
        const auto bytes=retained_solid::Probe::Save(app);std::vector<retained_solid::Record> records;double loadedUnit=0;
        const bool opened=retained_solid::Probe::Open(bytes,false,&records,retained_solid::MaximumAggregateEnvelopeBytes,false,0,nullptr,&loadedUnit);
        out[prefix+"nativeFreshCarrierMatches"]=opened&&records.size()==1&&records[0].value&&records[0].value->bytes==fixture.bytes
            &&retained_solid::Bits(loadedUnit)==retained_solid::Bits(unit)
            &&InspectPrism(records[0].value->base,p.definition,frame,unit,stop,report)==Classification::MatchedBoundary;
        auto wrong=p.definition;wrong.depth*=1.1;
        out[prefix+"nativeFreshWrongRecipeRefuses"]=opened&&records.size()==1&&records[0].value
            &&InspectPrism(records[0].value->base,wrong,frame,unit,stop,report)==Classification::Refused;
        bool selected=false;out[prefix+"oldReaderRefuses"]=!retained_solid::Probe::Open(bytes,true,nullptr,retained_solid::MaximumAggregateEnvelopeBytes,false,0,&selected);
        out[prefix+"oldReaderActuallySelected"]=selected;
        out[prefix+"sourceStillByteExact"]=FullBRep(fixture.base)==original;
    }
    const auto recipe=probe::L(1,0,false);profile::ConstructionFrame identity;const auto base=probe::Build(recipe,identity);
    gp_Trsf step;step.SetTranslation(gp_Vec(7,11,13));const auto inverse=TopLoc_Location(step).Inverted();auto expected=identity;expected.values[0]=-7;expected.values[1]=-11;expected.values[2]=-13;
    Inspection report;out["negativePowerActualRootMatches"]=locations::RawLocation(inverse)
        &&InspectPrism(base.Moved(inverse),recipe,expected,.001,stop,report)==Classification::MatchedBoundary;
    return out;
}
inline Checks Run(int scenario){
    try{switch(scenario){case 0:return EnclosureValues();case 1:return PolygonValues();
        case 2:return saved_cut_prism_prototype::probe::Run();case 3:return Reopened();default:return {{"invalidScenario",false}};}
    }catch(...){return {{"setupException",false}};}
}
}
#endif
