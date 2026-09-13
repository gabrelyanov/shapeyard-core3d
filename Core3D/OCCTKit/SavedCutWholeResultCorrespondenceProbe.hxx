#pragma once
#if DEBUG
#include "SavedCutWholeResultCorrespondence.hxx"
#include "SavedCutBoreResultObservationProbe.hxx"
namespace core3d::saved_cut_whole_result::probe {
namespace fixture=core3d::saved_cut_bore_result::probe;
struct Evidence {std::map<std::string,bool> checks;std::map<std::string,std::string> phases;};
inline TopoDS_Shape Read(const std::string& bytes){
    if(bytes.empty()||bytes.size()>4*1024*1024)return {};std::istringstream in(bytes);in.imbue(std::locale::classic());TopoDS_Shape shape;BRep_Builder builder;BRepTools::Read(shape,in,builder);return shape;
}
inline bool Frame(retained_solid::Envelope& e){
    const double mm=e.metersPerUnit*1000;profile::ConstructionFrame f;
    f.values={11/mm,-7/mm,9/mm,0,0,0,std::cos(.2),1.25};f.values[3+e.axis]=std::sin(.2);
    gp_Trsf t;if(!f.Transform(t))return false;gp_Pnt p(e.point[0],e.point[1],e.point[2]);p.Transform(t);e.point={p.X(),p.Y(),p.Z()};
    if(e.sourceFamily==1){profile::Parameters source;if(!profile::Decode(e.sourceValues,source))return false;source.constructionFrame=f;e.sourceSchema=2;return profile::Encode(source,e.sourceValues);}
    enclosure::Parameters source;if(!enclosure::Decode(int(e.sourceSchema),e.sourceValues,source))return false;
    source.definition.constructionFrame=f;e.sourceSchema=2;return enclosure::Encode(source,e.sourceValues);
}
// Actual stored malformed cap pcurve; the 3D edge and cylinder stay unchanged.
// If the generated cap originally had no stored pcurve, BRep_Tool obtains its
// actual planar projection and the test then INSTALLS a shifted stored circle.
inline bool ShiftCap(const TopoDS_Shape& shape,double radius,double mm){
    Budget budget;unsigned faceCount=0;
    for(TopExp_Explorer fi(shape,TopAbs_FACE);fi.More();fi.Next()){
        if(++faceCount>130)return false;const auto face=TopoDS::Face(fi.Current());d::Surface surface;
        if(!d::ReadSurface(face,mm,budget,surface)||surface.cylinder)continue;
        unsigned edgeCount=0;for(TopExp_Explorer ei(face,TopAbs_EDGE);ei.More();ei.Next()){
            if(++edgeCount>128)return false;const auto edge=TopoDS::Edge(ei.Current().Oriented(TopAbs_FORWARD));d::Curve curve;
            if(!od::Curve(edge,mm,budget,curve)||!curve.circle||std::abs(d::Norm(curve.a)-radius)*mm>1e-9||curve.last-curve.first<std::acos(-1.0)*1.5)continue;
            double first=0,last=0;const auto pc=BRep_Tool::CurveOnSurface(edge,face,first,last);
            if(pc.IsNull()||first!=curve.first||last!=curve.last)continue;
            auto changed=Handle(Geom2d_Curve)::DownCast(pc->Copy());const auto circle=Handle(Geom2d_Circle)::DownCast(changed);if(circle.IsNull())continue;
            const auto location=circle->Location();const bool sameParameter=BRep_Tool::SameParameter(edge),sameRange=BRep_Tool::SameRange(edge);const double tol=BRep_Tool::Tolerance(edge);
            changed->Translate(gp_Vec2d(.125,0));BRep_Builder builder;builder.UpdateEdge(edge,changed,face,tol);builder.Range(edge,face,first,last);
            d::PCurve observed;d::Curve after;
            if(!od::Curve(edge,mm,budget,after)||!d::ReadPCurve(edge,surface,after,observed,budget)||!observed.stored||!observed.circle)return false;
            return observed.c.X()==location.X()+.125&&observed.c.Y()==location.Y()
                &&sameParameter==BRep_Tool::SameParameter(edge)&&sameRange==BRep_Tool::SameRange(edge)&&tol==BRep_Tool::Tolerance(edge)
                &&od::Bits(curve.first,after.first)&&od::Bits(curve.last,after.last);
        }
    }return false;
}
// Install a real native point record on an actual incident vertex. Mode0 is
// valid owned curve metadata; mode1 has a copied foreign support at identical
// coordinates; mode2 has an owned face but NaN second surface parameter.
// Modes3/4 install actual incident pcurve/surface records with valid parameters.
inline bool AddPointRecord(const TopoDS_Shape& shape,unsigned mode){
    if(mode>4)return false;unsigned faces=0;
    for(TopExp_Explorer fi(shape,TopAbs_FACE);fi.More();fi.Next()){
        if(++faces>130)return false;const auto face=TopoDS::Face(fi.Current());TopLoc_Location sl;const auto surface=BRep_Tool::Surface(face,sl);
        unsigned edges=0;for(TopExp_Explorer ei(face,TopAbs_EDGE);ei.More();ei.Next()){
            if(++edges>128)return false;const auto edge=TopoDS::Edge(ei.Current().Oriented(TopAbs_FORWARD));
            TopLoc_Location cl;double first=0,last=0;const auto curve=BRep_Tool::Curve(edge,cl,first,last);if(curve.IsNull())continue;
            for(TopoDS_Iterator vi(edge);vi.More();vi.Next()){
                if(vi.Value().ShapeType()!=TopAbs_VERTEX||vi.Value().Orientation()!=TopAbs_FORWARD)continue;
                const auto vertex=TopoDS::Vertex(vi.Value());const auto data=Handle(BRep_TVertex)::DownCast(vertex.TShape());if(data.IsNull())return false;
                Handle(BRep_PointRepresentation) record;
                if(mode<2){const auto support=mode?Handle(Geom_Curve)::DownCast(curve->Copy()):curve;if(support.IsNull()||(mode&&support==curve))return false;
                    const auto relative=cl.Predivided(vertex.Location());record=new BRep_PointOnCurve(first,support,relative);
                    if(record->IsPointOnCurve(curve,relative)!=(mode==0))return false;
                }else {if(surface.IsNull())return false;const auto relative=sl.Predivided(vertex.Location());
                    if(mode==2){record=new BRep_PointOnSurface(0,std::numeric_limits<double>::quiet_NaN(),surface,relative);
                        if(!record->IsPointOnSurface(surface,relative)||std::isfinite(record->Parameter2()))return false;
                    }else if(mode==3){
                        const auto edgeData=Handle(BRep_TEdge)::DownCast(edge.TShape());const auto er=sl.Predivided(edge.Location());
                        for(BRep_ListIteratorOfListOfCurveRepresentation it(edgeData->Curves());it.More();it.Next())if(it.Value()->IsCurveOnSurface(surface,er)){
                            record=new BRep_PointOnCurveOnSurface(first,it.Value()->PCurve(),surface,relative);break;}
                        if(record.IsNull())continue;
                    }else {double f=0,l=0;const auto pc=BRep_Tool::CurveOnSurface(edge,face,f,l);if(pc.IsNull()||f!=first||l!=last)continue;
                        const auto uv=pc->Value(first);if(!std::isfinite(uv.X())||!std::isfinite(uv.Y()))return false;
                        record=new BRep_PointOnSurface(uv.X(),uv.Y(),surface,relative);
                    }
                }
                const auto before=data->Points().Extent();data->ChangePoints().Append(record);
                return data->Points().Extent()==before+1&&data->Points().Last()==record;
            }
        }
    }return false;
}
inline Evidence Run(){Evidence evidence;try {
    for(double unit:{.001,1.0})for(int plane=0;plane<3;++plane)for(bool enclosure:{false,true})for(bool framed:{false,true}){
        const auto key=std::string(unit==.001?"mm":"metre")+".plane"+std::to_string(plane)+(enclosure?".enclosure.":".bracket.")+(framed?"framed.":"plain.");
        const auto add=[&](const char* name,bool value){evidence.checks[key+name]=value;};const double mm=unit*1000;
        auto old=enclosure?saved_cut_bore_clearance::probe::box(unit,plane):saved_cut_bore_clearance::probe::bracket(unit,plane);
        if(framed&&!Frame(old))throw std::invalid_argument("whole-result frame");auto next=old;
        if(enclosure){core3d::enclosure::Parameters p;core3d::enclosure::Decode(int(next.sourceSchema),next.sourceValues,p);
            p.definition.dimensions.width=120/mm;p.definition.dimensions.floor=3/mm;core3d::enclosure::Encode(p,next.sourceValues);}
        else {profile::Parameters p;profile::Decode(next.sourceValues,p);p.definition.points[1].SetX(80/mm);p.definition.points[2].SetX(80/mm);
            p.definition.points[4].SetY(70/mm);p.definition.points[5].SetY(70/mm);p.definition.depth=12/mm;profile::Encode(p,next.sourceValues);}
        auto stop=std::make_shared<std::atomic_bool>(false);bool oldMatched=false,newMatched=false;
        const auto oldBase=fixture::Base(old,stop,oldMatched),newBase=fixture::Base(next,stop,newMatched);
        add("independent-old-new-base-correspondence",oldMatched&&newMatched);
        analytic_boolean::Result initial,built;
        const bool initialBuilt=analytic_boolean::Build(oldBase,cylindrical_cut::Recipe(old),*stop,initial)==analytic_boolean::Status::Built;
        const bool nextBuilt=analytic_boolean::Build(newBase,cylindrical_cut::Recipe(next),*stop,built)==analytic_boolean::Status::Built;
        add("actual-old-new-booleans",initialBuilt&&nextBuilt);if(!initialBuilt||!nextBuilt)continue;
        const auto initialBytes=fixture::Bytes(initial.solid),resultBytes=fixture::Bytes(built.solid),oldBytes=fixture::Bytes(oldBase),newBytes=fixture::Bytes(newBase);
        const auto a=Inspect(initial.solid,old,old,*stop),b=Inspect(built.solid,old,next,*stop);evidence.phases[key+"initial"]=a.phase;evidence.phases[key+"new"]=b.phase;
        add("old-whole-oriented-boundary",a.classification==Classification::MatchedOrientedBoundary);
        add("paired-new-whole-oriented-boundary",b.classification==Classification::MatchedOrientedBoundary);
        add("explicit-cells-and-vertex-links",b.vertices==(enclosure?34u:14u)&&b.edges==(enclosure?51u:21u)&&b.faces==(enclosure?20u:9u)&&b.vertexLinks==b.vertices);
        auto fresh=Read(resultBytes);const auto freshBytes=fixture::Bytes(fresh);const auto reopened=Inspect(fresh,old,next,*stop);evidence.phases[key+"reopened"]=reopened.phase;
        add("fresh-private-brep-whole-match",!fresh.IsNull()&&!fresh.IsPartner(built.solid)&&reopened.classification==Classification::MatchedOrientedBoundary);
        add("fresh-reader-inspection-preserves-stream",freshBytes==fixture::Bytes(fresh)&&!freshBytes.empty());
        auto wrong=next;
        if(enclosure){core3d::enclosure::Parameters p;core3d::enclosure::Decode(int(wrong.sourceSchema),wrong.sourceValues,p);p.definition.dimensions.width=140/mm;core3d::enclosure::Encode(p,wrong.sourceValues);}
        else {profile::Parameters p;profile::Decode(wrong.sourceValues,p);p.definition.points[1].SetX(90/mm);p.definition.points[2].SetX(90/mm);profile::Encode(p,wrong.sourceValues);}
        bool alternateMatched=false;const auto alternateBase=fixture::Base(wrong,stop,alternateMatched);analytic_boolean::Result alternate;
        const bool alternateBuilt=analytic_boolean::Build(alternateBase,cylindrical_cut::Recipe(wrong),*stop,alternate)==analytic_boolean::Status::Built;
        const auto partial=alternateBuilt?saved_cut_bore_result::Inspect(alternate.solid,old,next,*stop):saved_cut_bore_result::Report{};
        const auto rejected=Inspect(alternate.solid,old,next,*stop);evidence.phases[key+"wrong-exterior"]=rejected.phase;
        add("same-bore-wrong-exterior-refuses",alternateMatched&&alternateBuilt&&partial.status==saved_cut_bore_result::Status::BoreWallObservedExteriorUnproven&&rejected.classification==Classification::Refused);
        auto badCap=Read(resultBytes);const auto capBefore=fixture::Bytes(badCap);const bool capShifted=ShiftCap(badCap,next.radius,mm);
        const auto capPartial=saved_cut_bore_result::Inspect(badCap,old,next,*stop);const auto capRefusal=Inspect(badCap,old,next,*stop);evidence.phases[key+"cap-corruption"]=capRefusal.phase;
        add("actual-cap-pcurve-coefficient-refusal",capShifted&&!capBefore.empty()&&fixture::Bytes(badCap)!=capBefore
            &&capPartial.status==saved_cut_bore_result::Status::BoreWallObservedExteriorUnproven&&capRefusal.classification==Classification::Refused&&std::string(capRefusal.phase)=="whole-pcurves");
        auto validRecord=Read(resultBytes);const bool ownedAdded=AddPointRecord(validRecord,0);const auto ownedBytes=fixture::Bytes(validRecord);
        const auto ownedResult=Inspect(validRecord,old,next,*stop);evidence.phases[key+"owned-point"]=ownedResult.phase;
        add("actual-owned-point-record-admitted",ownedAdded&&ownedResult.classification==Classification::MatchedOrientedBoundary
            &&!ownedBytes.empty()&&ownedBytes==fixture::Bytes(validRecord));
        for(unsigned mode:{3u,4u}){auto input=Read(resultBytes);const bool installed=AddPointRecord(input,mode);const auto bytes=fixture::Bytes(input);
            const auto observed=Inspect(input,old,next,*stop);evidence.phases[key+(mode==3?"owned-pcurve-point":"owned-surface-point")]=observed.phase;
            add(mode==3?"actual-owned-pcurve-point-admitted":"actual-owned-surface-point-admitted",installed
                &&observed.classification==Classification::MatchedOrientedBoundary&&!bytes.empty()&&fixture::Bytes(input)==bytes);}
        auto foreignRecord=Read(resultBytes);const bool foreignAdded=AddPointRecord(foreignRecord,1);const auto foreignResult=Inspect(foreignRecord,old,next,*stop);evidence.phases[key+"foreign-point"]=foreignResult.phase;
        add("same-geometry-foreign-point-support-refuses",foreignAdded&&foreignResult.classification==Classification::Refused&&std::string(foreignResult.phase)=="whole-point-owners");
        auto nonfiniteRecord=Read(resultBytes);const bool nonfiniteAdded=AddPointRecord(nonfiniteRecord,2);const auto nonfiniteResult=Inspect(nonfiniteRecord,old,next,*stop);evidence.phases[key+"nonfinite-surface-point"]=nonfiniteResult.phase;
        add("owned-point-surface-nonfinite-second-parameter-refuses",nonfiniteAdded&&nonfiniteResult.classification==Classification::Refused&&std::string(nonfiniteResult.phase)=="whole-point-owners");
        const auto filled=Inspect(newBase,old,next,*stop);evidence.phases[key+"filled-base-refuses"]=filled.phase;
        add("filled-base-refuses",filled.classification==Classification::Refused);
        const auto reversed=Inspect(built.solid.Reversed(),old,next,*stop);evidence.phases[key+"reversed-root-refuses"]=reversed.phase;
        add("reversed-root-refuses",reversed.classification==Classification::Refused);
        auto changedTool=next;changedTool.radius=2/mm;const auto toolChanged=Inspect(built.solid,old,changedTool,*stop);evidence.phases[key+"changed-stored-tool-refuses"]=toolChanged.phase;
        add("changed-stored-tool-refuses",toolChanged.classification==Classification::Refused);
        auto changedValues=next;
        if(enclosure){core3d::enclosure::Parameters p;core3d::enclosure::Decode(int(changedValues.sourceSchema),changedValues.sourceValues,p);p.definition.dimensions.floor=4/mm;core3d::enclosure::Encode(p,changedValues.sourceValues);}
        else {profile::Parameters p;profile::Decode(changedValues.sourceValues,p);p.definition.depth=13/mm;profile::Encode(p,changedValues.sourceValues);}
        const auto intervalChanged=Inspect(built.solid,old,changedValues,*stop);evidence.phases[key+"wrong-wall-interval-refuses"]=intervalChanged.phase;
        add("wrong-wall-interval-refuses",intervalChanged.classification==Classification::Refused);
        // Same top-level type, second actual shell; no compound-only shortcut.
        auto extraCopy=Read(resultBytes),extraBase=Read(newBytes);TopoDS_Solid extra;BRep_Builder builder;builder.MakeSolid(extra);
        for(TopoDS_Iterator it(extraCopy);it.More();it.Next())builder.Add(extra,it.Value());
        for(TopoDS_Iterator it(extraBase);it.More();it.Next())builder.Add(extra,it.Value());
        const auto extraShell=Inspect(extra,old,next,*stop);evidence.phases[key+"extra-solid-shell-refuses"]=extraShell.phase;
        add("extra-solid-shell-refuses",extraShell.classification==Classification::Refused);
        stop->store(true);const auto stopped=Inspect(built.solid,old,next,*stop);evidence.phases[key+"stopped"]=stopped.phase;
        add("already-stopped-cancels",stopped.classification==Classification::Cancelled);stop->store(false);
        add("four-source-result-streams-stay-exact",!initialBytes.empty()&&!resultBytes.empty()&&!oldBytes.empty()&&!newBytes.empty()
            &&fixture::Bytes(initial.solid)==initialBytes&&fixture::Bytes(built.solid)==resultBytes&&fixture::Bytes(oldBase)==oldBytes&&fixture::Bytes(newBase)==newBytes);
    }
}catch(...){evidence.checks["exception"]=false;}return evidence;}
}
#endif
