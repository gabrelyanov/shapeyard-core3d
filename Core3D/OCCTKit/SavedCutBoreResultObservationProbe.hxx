#pragma once
#if DEBUG
#include "SavedCutBoreResultObservation.hxx"
#include "SavedCutSourceBoreClearanceProbe.hxx"
#include "AnalyticBooleanSolid.hxx"
#include "PrismExtractorProbe.hxx"
#include "EnclosureGeometry.hxx"
#include <BRepTools.hxx>
#include <sstream>
#include <TopoDS_Compound.hxx>

namespace core3d::saved_cut_bore_result::probe {
struct Evidence {std::map<std::string,bool> checks;std::map<std::string,std::string> phases;};
inline std::string Bytes(const TopoDS_Shape& shape){
    std::ostringstream out;out.imbue(std::locale::classic());
    BRepTools::Write(shape,out,Standard_True,Standard_True,TopTools_FormatVersion_VERSION_3);
    if(!out.good()||out.str().size()>4*1024*1024)return {};return out.str();
}
inline TopoDS_Shape Base(const retained_solid::Envelope& e,const std::shared_ptr<std::atomic_bool>& stop,bool& matches){
    matches=false;
    if(e.sourceFamily==1){profile::Parameters p;if(!profile::Decode(e.sourceValues,p))return {};
        const auto frame=p.constructionFrame.value_or(profile::ConstructionFrame{});
        auto shape=saved_cut_prism_prototype::probe::Build(p.definition,frame);saved_cut_prism_prototype::Inspection inspected;
        matches=saved_cut_prism_prototype::InspectPrism(shape,p.definition,frame,e.metersPerUnit,*stop,inspected)==saved_cut_prism_prototype::Classification::MatchedBoundary;return shape;}
    enclosure::Parameters p;if(!enclosure::Decode(int(e.sourceSchema),e.sourceValues,p))return {};
    EnclosureSolidResult built;if(!BuildEnclosureSolidGeometry(p.definition,stop,built))return {};
    enclosure_correspondence::Inspection inspected;
    matches=enclosure_correspondence::InspectEnclosure(built.solid,p,*stop,inspected)==enclosure_correspondence::Classification::MatchedBoundary;return built.solid;
}
inline bool Observed(const Report& r,double radius,double first,double last){
    return r.status==Status::BoreWallObservedExteriorUnproven&&r.circularOpeningCycles==2&&r.seamUses==2
        &&std::abs(r.radiusMM-radius)<1e-9&&std::abs(r.axisIntervalOriginalMM[0]-first)<1e-8
        &&std::abs(r.axisIntervalOriginalMM[1]-last)<1e-8&&std::string(r.phase)=="exterior-unproved";
}
// Disposable, independently BRep-read shape only. Actual closed seam mutation.
inline bool ShiftSeam(const TopoDS_Shape& shape,double radius,double mm){
    Budget budget;unsigned faces=0;
    for(TopExp_Explorer fi(shape,TopAbs_FACE);fi.More();fi.Next()){
        if(++faces>130)return false;const auto face=TopoDS::Face(fi.Current());d::Surface surface;
        if(!d::ReadSurface(face,mm,budget,surface)||!surface.cylinder||std::abs(d::Norm(surface.x)-radius)*mm>1e-9)continue;
        unsigned edges=0;for(TopExp_Explorer ei(face,TopAbs_EDGE);ei.More();ei.Next()){
            if(++edges>128)return false;const auto edge=TopoDS::Edge(ei.Current().Oriented(TopAbs_FORWARD));
            if(!BRep_Tool::IsClosed(edge,face))continue;
            const auto relative=surface.location.Predivided(edge.Location());
            const auto data=Handle(BRep_TEdge)::DownCast(edge.TShape());
            for(BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());it.More();it.Next()){
                const auto& rep=it.Value();if(!rep->IsCurveOnSurface(surface.handle,relative)||!rep->IsCurveOnClosedSurface())continue;
                auto first=Handle(Geom2d_Curve)::DownCast(rep->PCurve()->Copy());
                auto second=Handle(Geom2d_Curve)::DownCast(rep->PCurve2()->Copy());
                const auto line=Handle(Geom2d_Line)::DownCast(first);if(line.IsNull()||second.IsNull())return false;
                const auto oldOrigin=line->Location();const double tolerance=BRep_Tool::Tolerance(edge);
                const bool sameParameter=BRep_Tool::SameParameter(edge),sameRange=BRep_Tool::SameRange(edge);
                double a=0,b=0;Handle(BRep_GCurve)::DownCast(rep)->Range(a,b);
                first->Translate(gp_Vec2d(.125,0));BRep_Builder builder;builder.UpdateEdge(edge,first,second,face,tolerance);builder.Range(edge,face,a,b);
                // Read the replacement representation back, not the local copy.
                for(BRep_ListIteratorOfListOfCurveRepresentation jt(data->Curves());jt.More();jt.Next()){
                    const auto& observed=jt.Value();if(!observed->IsCurveOnSurface(surface.handle,relative)||!observed->IsCurveOnClosedSurface())continue;
                    const auto changed=Handle(Geom2d_Line)::DownCast(observed->PCurve());double c=0,dv=0;Handle(BRep_GCurve)::DownCast(observed)->Range(c,dv);
                    return !changed.IsNull()&&changed->Location().X()==oldOrigin.X()+.125&&changed->Location().Y()==oldOrigin.Y()
                        &&a==c&&b==dv&&sameParameter==BRep_Tool::SameParameter(edge)&&sameRange==BRep_Tool::SameRange(edge)&&tolerance==BRep_Tool::Tolerance(edge);
                }return false;
            }
        }
    }return false;
}
// Topology-only mutation on a private generated clone: preserve every edge
// representation handle and parameter, replacing ONE endpoint identity at the
// same exact coordinates. Old observer could not distinguish this disconnection.
inline bool DisconnectSeamVertex(const TopoDS_Shape& shape,double radius,double mm,bool last){
    Budget budget;unsigned faceCount=0;
    for(TopExp_Explorer fi(shape,TopAbs_FACE);fi.More();fi.Next()){
        if(++faceCount>130)return false;const auto face=TopoDS::Face(fi.Current());d::Surface surface;
        if(!d::ReadSurface(face,mm,budget,surface)||!surface.cylinder||std::abs(d::Norm(surface.x)-radius)*mm>1e-9)continue;
        unsigned edgeCount=0;for(TopExp_Explorer ei(face,TopAbs_EDGE);ei.More();ei.Next()){
            if(++edgeCount>128)return false;auto edge=TopoDS::Edge(ei.Current().Oriented(TopAbs_FORWARD));
            if(!BRep_Tool::IsClosed(edge,face))continue;
            // Fixture shapes are unlocated. Do not silently reinterpret an
            // arbitrary located vertex as a coordinate-only mutation.
            if(!edge.Location().IsIdentity())return false;
            TopoDS_Vertex prior;unsigned children=0;
            for(TopoDS_Iterator vi(edge,Standard_False,Standard_False);vi.More();vi.Next()){
                if(++children>2||vi.Value().ShapeType()!=TopAbs_VERTEX)return false;
                if(vi.Value().Orientation()==(last?TopAbs_REVERSED:TopAbs_FORWARD)){
                    if(!prior.IsNull())return false;prior=TopoDS::Vertex(vi.Value());
                }
            }
            if(children!=2||prior.IsNull()||!prior.Location().IsIdentity())return false;
            const auto point=BRep_Tool::Pnt(prior);const double tolerance=BRep_Tool::Tolerance(prior);
            const double parameter=BRep_Tool::Parameter(prior,edge),edgeTolerance=BRep_Tool::Tolerance(edge);
            const bool sameParameter=BRep_Tool::SameParameter(edge),sameRange=BRep_Tool::SameRange(edge),wasFree=edge.Free(),wasModified=edge.Modified();
            const auto data=Handle(BRep_TEdge)::DownCast(edge.TShape());std::vector<Handle(BRep_CurveRepresentation)> representations;
            for(BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());it.More();it.Next())representations.push_back(it.Value());
            detail::Edge before;before.shape=edge;if(!detail::Curve(edge,mm,budget,before.curve))return false;
            BRep_Builder builder;TopoDS_Vertex fresh;builder.MakeVertex(fresh,point,tolerance);fresh.Orientation(prior.Orientation());
            edge.Free(Standard_True);builder.Remove(edge,prior);builder.Add(edge,fresh);edge.Free(wasFree);edge.Modified(wasModified);
            const auto observed=BRep_Tool::Pnt(fresh);detail::Edge after;after.shape=edge;
            if(!detail::Curve(edge,mm,budget,after.curve))return false;
            const auto& a=before.curve;const auto& b=after.curve;
            const auto vectorBits=[](const gp_Vec& x,const gp_Vec& y){return detail::Bits(x.X(),y.X())&&detail::Bits(x.Y(),y.Y())&&detail::Bits(x.Z(),y.Z());};
            bool curveExact=a.circle==b.circle&&vectorBits(a.c,b.c)&&vectorBits(a.a,b.a)&&vectorBits(a.b,b.b)
                &&detail::Bits(a.first,b.first)&&detail::Bits(a.last,b.last)&&a.location.IsEqual(b.location);
            unsigned index=0;for(BRep_ListIteratorOfListOfCurveRepresentation it(data->Curves());it.More();it.Next()){
                if(index>=representations.size()||it.Value()!=representations[index++])return false;
            }
            bool foundFresh=false,foundPrior=false;children=0;
            for(TopoDS_Iterator vi(edge,Standard_False,Standard_False);vi.More();vi.Next()){
                ++children;foundFresh|=vi.Value().IsSame(fresh);foundPrior|=vi.Value().IsSame(prior);
            }
            return children==2&&foundFresh&&!foundPrior&&!fresh.IsPartner(prior)&&curveExact&&index==representations.size()
                &&detail::Bits(observed.X(),point.X())&&detail::Bits(observed.Y(),point.Y())&&detail::Bits(observed.Z(),point.Z())
                &&detail::Bits(BRep_Tool::Tolerance(fresh),tolerance)&&detail::Bits(BRep_Tool::Parameter(fresh,edge),parameter)
                &&detail::Bits(BRep_Tool::Tolerance(edge),edgeTolerance)&&BRep_Tool::SameParameter(edge)==sameParameter&&BRep_Tool::SameRange(edge)==sameRange
                &&edge.Free()==wasFree&&edge.Modified()==wasModified;
        }
    }return false;
}
// Proposed real native generator/codec tests; no bridge, test method or selection.
inline Evidence Run(){
    Evidence evidence;try {
        for(double unit:{.001,1.0})for(int plane=0;plane<3;++plane)for(bool isEnclosure:{false,true}){
            const auto key=std::string(unit==.001?"mm":"metre")+".plane"+std::to_string(plane)+(isEnclosure?".enclosure.":".bracket.");
            auto add=[&](const char* suffix,bool value){evidence.checks[key+suffix]=value;};const double mm=unit*1000;
            auto old=isEnclosure?saved_cut_bore_clearance::probe::box(unit,plane):saved_cut_bore_clearance::probe::bracket(unit,plane);
            auto next=old;
            if(isEnclosure){enclosure::Parameters p;enclosure::Decode(int(next.sourceSchema),next.sourceValues,p);
                p.definition.dimensions.width=120/mm;p.definition.dimensions.floor=3/mm;enclosure::Encode(p,next.sourceValues);}
            else {profile::Parameters p;profile::Decode(next.sourceValues,p);
                p.definition.points[1].SetX(80/mm);p.definition.points[2].SetX(80/mm);p.definition.points[4].SetY(70/mm);p.definition.points[5].SetY(70/mm);
                p.definition.depth=12/mm;profile::Encode(p,next.sourceValues);}
            auto stop=std::make_shared<std::atomic_bool>(false);bool oldMatch=false,newMatch=false;
            const auto oldBase=Base(old,stop,oldMatch),newBase=Base(next,stop,newMatch);
            add("old-base-correspondence",oldMatch);add("new-base-correspondence",newMatch);
            analytic_boolean::Result original,built;
            const bool oldBuilt=analytic_boolean::Build(oldBase,cylindrical_cut::Recipe(old),*stop,original)==analytic_boolean::Status::Built;
            const bool newBuilt=analytic_boolean::Build(newBase,cylindrical_cut::Recipe(next),*stop,built)==analytic_boolean::Status::Built;
            add("actual-old-and-new-cut-built",oldBuilt&&newBuilt);if(!oldBuilt||!newBuilt)continue;
            const auto before=Bytes(built.solid),originalBytes=Bytes(original.solid),oldBytes=Bytes(oldBase),newBytes=Bytes(newBase);
            const auto a=Inspect(original.solid,old,old,*stop),b=Inspect(built.solid,old,next,*stop);
            evidence.phases[key+"old"]=a.phase;evidence.phases[key+"new"]=b.phase;
            add("old-wall-observation-only",Observed(a,1,0,isEnclosure?2:8));
            add("paired-new-wall-observation-only",Observed(b,1,0,isEnclosure?3:12));
            add("complete-input-streams-unchanged",!before.empty()&&!originalBytes.empty()&&!oldBytes.empty()&&!newBytes.empty()
                &&Bytes(built.solid)==before&&Bytes(original.solid)==originalBytes&&Bytes(oldBase)==oldBytes&&Bytes(newBase)==newBytes);
            // Independent analytic removal, supplementary only: no gate uses it.
            add("removed-volume-independent",std::abs(built.removedVolume*mm*mm*mm-std::acos(-1.0)*(isEnclosure?3:12))<1e-5);
            TopoDS_Shape reopened;BRep_Builder builder;std::istringstream input(before);input.imbue(std::locale::classic());BRepTools::Read(reopened,input,builder);
            const auto fresh=Inspect(reopened,old,next,*stop);evidence.phases[key+"fresh"]=fresh.phase;
            add("private-fresh-reopen",!reopened.IsNull()&&!reopened.IsPartner(built.solid)&&Observed(fresh,1,0,isEnclosure?3:12));
            const auto freshBytes=Bytes(reopened);const bool shifted=ShiftSeam(reopened,next.radius,mm);
            const auto corrupt=Inspect(reopened,old,next,*stop);evidence.phases[key+"corrupt-seam"]=corrupt.phase;
            add("actual-seam-pcurve-corruption-refuses",shifted&&Bytes(reopened)!=freshBytes&&corrupt.status==Status::Refused
                &&std::string(corrupt.phase)=="bore-trims"&&Bytes(built.solid)==before);
            for(bool endpoint:{false,true}){
                TopoDS_Shape disconnected;BRep_Builder db;std::istringstream serialized(before);serialized.imbue(std::locale::classic());BRepTools::Read(disconnected,serialized,db);
                const bool admitted=!disconnected.IsNull()&&Observed(Inspect(disconnected,old,next,*stop),1,0,isEnclosure?3:12);
                const auto untouched=Bytes(disconnected);const bool replaced=admitted&&DisconnectSeamVertex(disconnected,next.radius,mm,endpoint);
                const auto refusal=Inspect(disconnected,old,next,*stop);
                evidence.phases[key+(endpoint?"disconnected-seam-end":"disconnected-seam-start")]=refusal.phase;
                add(endpoint?"same-position-seam-end-refuses":"same-position-seam-start-refuses",replaced&&Bytes(disconnected)!=untouched
                    &&refusal.status==Status::Refused&&std::string(refusal.phase)=="bore-vertex-sharing"
                    &&Bytes(built.solid)==before&&Bytes(original.solid)==originalBytes&&Bytes(oldBase)==oldBytes&&Bytes(newBase)==newBytes);
            }
            add("filled-base-refuses",Inspect(newBase,old,next,*stop).status==Status::Refused);
            add("reversed-root-refuses",Inspect(built.solid.Reversed(),old,next,*stop).status==Status::Refused);
            auto wrongOld=old,wrongNext=next;wrongOld.radius=wrongNext.radius=2/mm;
            add("wrong-radius-refuses",Inspect(built.solid,wrongOld,wrongNext,*stop).status==Status::Refused);
            wrongNext=next;wrongNext.point[(next.axis+1)%3]+=1/mm;
            add("tool-change-refuses",Inspect(built.solid,old,wrongNext,*stop).status==Status::Refused);
            // TopoDS_Builder::Add freezes its child's shared TShape. Assemble
            // this negative fixture from independent serialized copies only.
            const bool originalsBeforeCompound=Bytes(built.solid)==before&&Bytes(original.solid)==originalBytes
                &&Bytes(oldBase)==oldBytes&&Bytes(newBase)==newBytes;
            TopoDS_Shape compoundCut,compoundBase;
            std::istringstream cutInput(before),baseInput(newBytes);
            cutInput.imbue(std::locale::classic());baseInput.imbue(std::locale::classic());
            BRepTools::Read(compoundCut,cutInput,builder);BRepTools::Read(compoundBase,baseInput,builder);
            const bool privateCompoundInputs=!compoundCut.IsNull()&&!compoundBase.IsNull()
                &&compoundCut.ShapeType()==TopAbs_SOLID&&compoundBase.ShapeType()==TopAbs_SOLID
                &&!compoundCut.IsPartner(built.solid)&&!compoundCut.IsPartner(newBase)
                &&!compoundBase.IsPartner(built.solid)&&!compoundBase.IsPartner(newBase)
                &&!compoundCut.IsPartner(compoundBase);
            TopoDS_Compound extra;builder.MakeCompound(extra);
            if(privateCompoundInputs){builder.Add(extra,compoundCut);builder.Add(extra,compoundBase);}
            const bool originalsAfterCompound=Bytes(built.solid)==before&&Bytes(original.solid)==originalBytes
                &&Bytes(oldBase)==oldBytes&&Bytes(newBase)==newBytes;
            add("compound-extra-solid-refuses",originalsBeforeCompound&&privateCompoundInputs&&extra.NbChildren()==2
                &&originalsAfterCompound&&Inspect(extra,old,next,*stop).status==Status::Refused);
            stop->store(true);add("already-stopped-cancels",Inspect(built.solid,old,next,*stop).status==Status::Cancelled);stop->store(false);
            // Deliberate limitation witness: alter the exterior without changing
            // the independently observed bore. NO result-success authority exists.
            auto different=next;
            if(isEnclosure){enclosure::Parameters p;enclosure::Decode(int(different.sourceSchema),different.sourceValues,p);
                p.definition.dimensions.width=140/mm;enclosure::Encode(p,different.sourceValues);}
            else {profile::Parameters p;profile::Decode(different.sourceValues,p);p.definition.points[1].SetX(90/mm);p.definition.points[2].SetX(90/mm);profile::Encode(p,different.sourceValues);}
            bool differentMatches=false;const auto differentBase=Base(different,stop,differentMatches);analytic_boolean::Result changed;
            const bool changedBuilt=analytic_boolean::Build(differentBase,cylindrical_cut::Recipe(different),*stop,changed)==analytic_boolean::Status::Built;
            add("wrong-exterior-never-qualified",differentMatches&&changedBuilt&&Observed(Inspect(changed.solid,old,next,*stop),1,0,isEnclosure?3:12));
            add("original-stream-still-exact",Bytes(built.solid)==before&&Bytes(original.solid)==originalBytes&&Bytes(oldBase)==oldBytes&&Bytes(newBase)==newBytes);
        }
    }catch(...){evidence.checks["exception"]=false;}return evidence;
}
}
#endif
