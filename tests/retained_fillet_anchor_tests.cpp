#include "SavedBooleanFilletProbe.hxx"
#include <iostream>
#include <iomanip>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <TNaming_Builder.hxx>
#include <TNaming_Tool.hxx>
#include <TDataStd_ByteArray.hxx>
#include "RetainedFilletCandidates.hxx"
static bool CandidateTests(){
    namespace f=core3d::retained_fillet;namespace p=core3d::saved_boolean_fillet_probe;
    bool passed=true;
    const auto check=[&](const char* name,bool value){std::cout<<"candidates "<<name<<" "<<(value?"PASS":"FAIL")<<"\n";passed=passed&&value;};
    const auto program=p::Fixture(0);const auto built=p::wedge::Build(program);
    const auto base=p::wedge::Base(core3d::saved_boolean_result::detail::GeometryView(program,0));
    check("fixture-built",built.status==core3d::saved_boolean_build::Status::Built);
    if(built.solid.IsNull()||base.IsNull())return false;
    const auto before=f::CandidateShapeBytes(built.solid),baseBefore=f::CandidateShapeBytes(base);
    const auto bytes=p::wedge::Bytes(program);
    // Keep actual OCAF-owned geometry and program attributes live while the
    // query runs, so copy isolation also has an undo/redo regression.
    const auto application=XCAFApp_Application::GetApplication();Handle(TDocStd_Document) document;
    application->NewDocument("BinXCAF",document);document->SetUndoLimit(10);document->NewCommand();
    const auto shapeLabel=document->Main().FindChild(1),programLabel=document->Main().FindChild(2);
    TNaming_Builder(shapeLabel).Generated(built.solid);
    const auto attribute=TDataStd_ByteArray::Set(programLabel,1,static_cast<int>(bytes.size()));
    for(std::size_t i=0;i<bytes.size();++i)attribute->SetValue(static_cast<int>(i+1),bytes[i]);
    document->CommitCommand();const int undos=document->GetAvailableUndos(),redos=document->GetAvailableRedos();
    const auto candidates=f::DiscoverCandidates(built.solid,program,base);
    check("complete-35",candidates.status==f::CandidateStatus::Available&&candidates.values.size()==35&&!candidates.truncated);
    unsigned lines=0,circles=0;TopTools_IndexedMapOfShape selected;
    bool canonical=true;
    for(const auto& value:candidates.values){
        const auto& a=value.anchor;TopoDS_Edge edge;
        if(f::Resolve(built.solid,a,1,edge)!=f::Outcome::Built){canonical=false;continue;}
        canonical=canonical&&!selected.Contains(edge);selected.Add(edge);BRepAdaptor_Curve c(edge);
        const auto expected=c.Value(c.GetType()==GeomAbs_Line?(c.FirstParameter()+c.LastParameter())/2:c.FirstParameter()+std::acos(-1.)/2);
        canonical=canonical&&expected.Distance(f::Point(a))<=1e-9;
        for(TopExp_Explorer v(edge,TopAbs_VERTEX);v.More();v.Next())
            canonical=canonical&&f::Point(a).Distance(BRep_Tool::Pnt(TopoDS::Vertex(v.Current())))>1e-4;
        if(a.curveKind==f::CurveKind::Line)++lines;else ++circles;
        std::cout<<std::setprecision(17)<<"candidate "<<int(a.curveKind);
        for(double x:a.anchorPoint)std::cout<<" "<<x;for(double x:a.axis)std::cout<<" "<<x;
        std::cout<<" "<<a.circleRadius<<" "<<value.lengthLocal<<"\n";
    }
    check("33-lines-2-circles",lines==33&&circles==2);check("unique-canonical-interior",canonical);
    auto anchors=p::Rims();const auto caps=p::Caps();anchors.insert(anchors.end(),caps.begin(),caps.end());bool covered=true;
    for(const auto& anchor:anchors){TopoDS_Edge edge;covered=covered&&f::Resolve(built.solid,anchor,1,edge)==f::Outcome::Built&&selected.Contains(edge);}
    check("all-12-hand-anchors-edge-equivalent",covered);
    const auto again=f::DiscoverCandidates(built.solid,program,base);bool same=candidates.values.size()==again.values.size();
    for(std::size_t i=0;same&&i<candidates.values.size();++i){const auto& a=candidates.values[i];const auto& b=again.values[i];
        same=a.anchor.curveKind==b.anchor.curveKind&&a.anchor.anchorPoint==b.anchor.anchorPoint&&a.anchor.axis==b.anchor.axis
            &&a.anchor.circleRadius==b.anchor.circleRadius&&a.lengthLocal==b.lengthLocal;}
    check("deterministic",same);check("original-shape-base-program-bytes-unchanged",before==f::CandidateShapeBytes(built.solid)
        &&baseBefore==f::CandidateShapeBytes(base)&&bytes==p::wedge::Bytes(program));
    Handle(TNaming_NamedShape) named;bool documentSame=shapeLabel.FindAttribute(TNaming_NamedShape::GetID(),named);
    documentSame=documentSame&&before==f::CandidateShapeBytes(TNaming_Tool::GetShape(named));
    for(std::size_t i=0;i<bytes.size();++i)documentSame=documentSame&&attribute->Value(static_cast<int>(i+1))==bytes[i];
    check("document-bytes-and-history-unchanged",documentSame&&!document->HasOpenCommand()
        &&document->GetAvailableUndos()==undos&&document->GetAvailableRedos()==redos);
    check("bare-loft-unsupported",f::DiscoverCandidates(base,core3d::retained_boolean::Program{},{}).status==f::CandidateStatus::Unsupported);
    // A real 24-sided retained prism has 72 convex lines plus its two bore rims.
    auto envelope=p::wedge::Prism();core3d::profile::Parameters polygon;polygon.metersPerUnit=.001;
    polygon.definition.plane=0;polygon.definition.depth=120;
    for(unsigned i=0;i<24;++i){const double angle=2*std::acos(-1.)*i/24;polygon.definition.points.push_back({100*std::cos(angle),100*std::sin(angle)});}
    core3d::retained_boolean::Program many;
    const bool ready=core3d::profile::Encode(polygon,envelope.sourceValues)&&core3d::retained_boolean::Promote(envelope,many);
    check("cap-fixture-values",ready);
    if(ready){many.codecMinor=4;const auto large=p::wedge::Build(many);const auto largeBase=p::wedge::Base(envelope);
        check("cap-fixture-built",large.status==core3d::saved_boolean_build::Status::Built);
        const auto limited=f::DiscoverCandidates(large.solid,many,largeBase);
        check("64-cap-truncated",limited.status==f::CandidateStatus::Available&&limited.values.size()==64&&limited.truncated);}
    auto fillets=p::Fixture(1);std::atomic_bool stop{false};
    const auto budget=f::Build(built.solid,fillets,stop,[](const TopoDS_Shape&){return false;});
    check("budget-outcome",budget.outcome==f::Outcome::DeclinedBudget&&f::IsDeclined(budget.outcome));
    application->Close(document);
    return passed;
}
int main(){bool passed=CandidateTests();
    for(unsigned scenario=0;scenario<4;++scenario){const auto checks=core3d::saved_boolean_fillet_probe::Run(scenario);
        if(checks.empty())passed=false;
        for(const auto& row:checks){std::cout<<scenario<<" "<<row.first<<" "<<(row.second?"PASS":"FAIL")<<"\n";passed=passed&&row.second;}}
    return passed?0:1;
}
