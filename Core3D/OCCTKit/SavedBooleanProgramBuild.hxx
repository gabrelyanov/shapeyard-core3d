#pragma once
// Real detached complete-program geometry. This is not an ordinary/AI permit.
#include "SavedBooleanResultCorrespondence.hxx"
#include "RetainedBooleanEditValues.hxx"
#include "SavedCutSourceEdit.hxx"
#include "AnalyticBooleanSolid.hxx"
#include "CutDisplayPreparation.hxx"
#include "RetainedFilletBuild.hxx"
#include <BinTools.hxx>
namespace core3d::saved_boolean_build {
using Program=retained_boolean::Program;
using Commitment=saved_cut_source_edit::ShapeCommitment;
enum class Status { Refused, Cancelled, Built };
struct Budget {
    std::size_t shapeOccurrences=0,streamBytes=0,booleanSteps=0,filletSteps=0;
    static constexpr std::size_t MaximumOccurrences=65536;
    static constexpr std::size_t MaximumSteps=2*retained_boolean::MaximumOperands; // old verification + changed replay
};
struct Result {
    Status status=Status::Refused;
    const char* phase="input";
    TopoDS_Shape solid;
    Commitment retainedBase,finalResult;
    std::vector<std::uint8_t> exactProgram;
    saved_boolean_result::Inspection correspondence;
    Budget budget;
    retained_fillet::Outcome filletOutcome=retained_fillet::Outcome::Built;
};
inline bool Charge(const TopoDS_Shape& shape,const std::atomic_bool& stop,Budget& budget) {
    if(shape.IsNull()||budget.shapeOccurrences>Budget::MaximumOccurrences)return false;
    struct Entry {TopoDS_Shape shape;unsigned depth;};std::vector<Entry> pending{{shape,0}};
    while(!pending.empty()){
        if(stop.load()||budget.shapeOccurrences>=Budget::MaximumOccurrences)return false;
        ++budget.shapeOccurrences;auto entry=std::move(pending.back());pending.pop_back();if(entry.depth>64)return false;
        for(TopoDS_Iterator it(entry.shape);it.More();it.Next()){
            if(stop.load()||pending.size()>=Budget::MaximumOccurrences-budget.shapeOccurrences)return false;
            pending.push_back({it.Value(),entry.depth+1});
        }
    }return true;
}
// IDs/order/high-water and plain bores stay exact. Rings alone re-anchor their bolt radius.
// Existing value-only patch validation is applied to EVERY operand view; none
// is returned or accepted as a one-bore owner/command.
inline std::optional<retained_boolean::Change> SourcePatch(const Program& original,
    const saved_cut_source_values::Patch& patch) noexcept {
    try {
        if(!retained_boolean::Valid(original))return {};
        retained_boolean::Change result;result.recipe=original;
        if(!retained_boolean::Encode(original,result.oldBytes))return {};
        auto& changed=std::get<Program>(result.recipe);std::optional<std::vector<double>> sourceValues;
        for(std::size_t i=0;i<original.steps.size();++i){
            const auto view=saved_boolean_result::detail::GeometryView(original,i);
            const auto applied=saved_cut_source_values::Apply(view,patch);if(!applied)return {};
            if(sourceValues&&*sourceValues!=applied->envelope.sourceValues)return {};
            // Numeric equality is not sufficient for exact omitted signed bits.
            if(sourceValues)for(std::size_t k=0;k<sourceValues->size();++k)
                if(retained_solid::Bits((*sourceValues)[k])!=retained_solid::Bits(applied->envelope.sourceValues[k]))return {};
            sourceValues=applied->envelope.sourceValues;
        }
        if(!sourceValues)return {};changed.source.values=std::move(*sourceValues);
        for(std::size_t i=0;i<changed.steps.size();++i){auto& t=changed.steps[i].operand;
            if(t.kind!=analytic_boolean::OperandKind::CylinderRing)continue;
            double before=0,after=0;
            if(!saved_boolean_result::detail::HostRadialExtent(original,t,before)
                ||!saved_boolean_result::detail::HostRadialExtent(changed,t,after))return {};
            if(before!=after)t.boltCircleRadius=t.hostRadiusRatio*after;
        }
        if((retained_boolean::HasRing(changed)||retained_boolean::HasWedge(changed))&&(!saved_boolean_result::detail::AdmitSections(original)
            ||!saved_boolean_result::detail::AdmitSections(changed)))return {};
        if(!retained_boolean::Encode(result.recipe,result.newBytes))return {};
        result.changed=result.oldBytes!=result.newBytes;return result;
    }catch(...){return {};}
}
// Ring-specific preparation is shared by viewer, ordinary admission and OCAF
// stage/seal. The legacy source helper remains unchanged. Values is the existing
// saved_program_source_edit::Values DTO, instantiated only after its definition.
template<class Values> inline bool PrepareRingSourceValues(const retained_boolean::Recipe& original,
    const saved_cut_source_values::Patch& patch,const std::atomic_bool& stop,Values& out) noexcept {
    out={};try {
        const auto* p=std::get_if<Program>(&original);
        if(stop.load()||!p||(!retained_boolean::HasRing(*p)&&!retained_boolean::HasWedge(*p)))return false;
        const auto change=SourcePatch(*p,patch);if(!change||stop.load())return false;
        Values v;v.oldProgram=*p;v.newProgram=std::get<Program>(change->recipe);
        v.oldBytes=change->oldBytes;v.newBytes=change->newBytes;v.changed=change->changed;
        auto fixed=v.newProgram;fixed.source.values=p->source.values;
        if(fixed.steps.size()!=p->steps.size())return false;
        for(std::size_t i=0;i<p->steps.size();++i)if(p->steps[i].operand.kind==analytic_boolean::OperandKind::CylinderRing){
            double before=0,after=0;const auto& old=p->steps[i].operand;
            if(!saved_boolean_result::detail::HostRadialExtent(*p,old,before)
                ||!saved_boolean_result::detail::HostRadialExtent(v.newProgram,old,after))return false;
            const double expected=before==after?old.boltCircleRadius:old.hostRadiusRatio*after;
            if(retained_solid::Bits(v.newProgram.steps[i].operand.boltCircleRadius)!=retained_solid::Bits(expected))return false;
            fixed.steps[i].operand.boltCircleRadius=old.boltCircleRadius;
        }
        std::vector<std::uint8_t> bytes;
        if(!retained_boolean::Encode(fixed,bytes)||bytes!=v.oldBytes||stop.load())return false;
        out=std::move(v);return true;
    }catch(...){out={};return false;}
}
template<class Values,class LegacyPrepare> inline bool PrepareSourceValues(const retained_boolean::Recipe& original,
    const saved_cut_source_values::Patch& patch,const std::atomic_bool& stop,Values& out,LegacyPrepare legacy) noexcept {
    const auto* p=std::get_if<Program>(&original);
    return p&&(retained_boolean::HasRing(*p)||retained_boolean::HasWedge(*p))?PrepareRingSourceValues(original,patch,stop,out):legacy(original,patch,stop,out);
}
inline bool SingleFilletCarrier(const Program& p){return p.codecMinor==4&&p.steps.size()==1
    &&p.steps[0].operand.kind==analytic_boolean::OperandKind::Cylinder;}
inline bool AdmitProgram(const Program& p){return SingleFilletCarrier(p)?retained_boolean::Valid(p)
    &&saved_cut_bore_clearance::Inspect(saved_boolean_result::detail::GeometryView(p,0)).status==saved_cut_bore_clearance::Status::ClearRecipeDisk
    :saved_boolean_result::detail::AdmitSections(p);}
inline saved_boolean_result::Inspection PreFilletInspection(const TopoDS_Shape& shape,const Program& p,const std::atomic_bool& stop){
    if(SingleFilletCarrier(p)){const auto view=saved_boolean_result::detail::GeometryView(p,0);
        return saved_cut_whole_result::Inspect(shape,view,view,stop);}
    return saved_boolean_result::Inspect(shape,p,stop);
}
// Aggregate-budget overload: the caller-owned budget accumulates occurrence
// and stream work across old capture, private copies and this rebuild, so the
// whole source-edit job shares one bounded allowance instead of a fresh
// per-substep budget. The original 4-argument form keeps its own local budget.
inline Result Build(const TopoDS_Shape& retainedBase,const Program& program,
    const cut_display::Settings& display,const std::atomic_bool& stop,Budget& budget) noexcept {
    Result out;const auto refuse=[&](){Result empty;empty.status=stop.load()?Status::Cancelled:Status::Refused;empty.budget=budget;empty.phase=out.phase;empty.correspondence=out.correspondence;empty.filletOutcome=out.filletOutcome;return empty;};
    try {
        if((display.type!=Aspect_TOD_RELATIVE&&display.type!=Aspect_TOD_ABSOLUTE)||!display.automatic)return refuse();
        for(double value:display.values)if(!std::isfinite(value))return refuse();
        if(display.values[0]<=0||display.values[1]<=0||display.values[1]>=M_PI||display.values[2]<=0
            ||(display.ownCoefficient&&std::abs(display.values[0]-display.values[3])>Precision::Confusion())
            ||(display.ownAngle&&std::abs(display.values[1]-display.values[4])>Precision::Angular()))return refuse();
        out.phase="source-and-program";
        if(stop.load()||!AdmitProgram(program)
            ||!retained_boolean::Encode(program,out.exactProgram)||!Charge(retainedBase,stop,budget)
            ||!saved_cut_source_edit::Commit(retainedBase,stop,budget.streamBytes,out.retainedBase))return refuse();
        std::vector<retained_boolean::Disk> disks;if(!retained_boolean::ExpandedSections(program,disks))return refuse();
        for(const auto& disk:disks){
            const auto view=saved_boolean_result::detail::GeometryView(program,disk.operand);
            if(stop.load()||!saved_cut_source_edit::InspectBase(retainedBase,view,stop)
                ||(disk.operand.kind!=analytic_boolean::OperandKind::Wedge&&saved_cut_bore_clearance::Inspect(view).status!=saved_cut_bore_clearance::Status::ClearRecipeDisk))return refuse();
        }
        // Each existing primitive makes an independent deep geometry copy.
        // The original base is never replaced by the displayed/current result.
        TopoDS_Shape current=retainedBase;
        if(!program.filletSteps.empty()&&program.source.family==3){
            // Native readback reconstructs gp_Dir components in a stored base.
            // Always replay fillets from the recipe's canonical planar loft,
            // after proving the retained base above. Never replace that base.
            const auto view=saved_boolean_result::detail::GeometryView(program,0);
            if(saved_cut_source_edit::RebuildLoftBase(view,stop,current)!=saved_cut_source_edit::LoftBaseStatus::Built
                ||!Charge(current,stop,budget))return refuse();
        }
        for(const auto& step:program.steps){
            if(stop.load()||budget.booleanSteps>=Budget::MaximumSteps)return refuse();
            ++budget.booleanSteps;out.phase="sequential-boolean";analytic_boolean::Recipe recipe;
            recipe.metersPerUnit=program.source.metersPerUnit;recipe.operation=step.operation;recipe.tool=step.operand;
            analytic_boolean::Result built;double wedgeVolume=0;
            if(step.operand.kind==analytic_boolean::OperandKind::Wedge){
                saved_cut_whole_result::Expected expected;const auto view=saved_boolean_result::detail::GeometryView(program,step.operand);
                if(!saved_cut_whole_result::ExpectedSource(view,expected))return refuse();
                const auto admitted=analytic_boolean_wedge::ExpectedBoundary(view,step.operand,expected);
                if(admitted.status!=analytic_boolean_wedge::Status::Clear)return refuse();wedgeVolume=admitted.removedVolume;
            }
            if(analytic_boolean::Build(current,recipe,stop,built,wedgeVolume)!=analytic_boolean::Status::Built
                ||!Charge(built.solid,stop,budget))return refuse();
            current=std::move(built.solid);
        }
        out.phase="display-preparation";
        if(stop.load()||!cut_display::Prepare(current,display,stop))return refuse();
        out.phase="complete-program-correspondence";out.correspondence=PreFilletInspection(current,program,stop);
        if(out.correspondence.classification!=saved_boolean_result::Classification::MatchedOrientedBoundary)return refuse();
        if(!program.filletSteps.empty()){
            out.phase="retained-fillet";
            if(program.filletSteps.size()>2*retained_fillet::MaximumSteps-budget.filletSteps)return refuse();
            budget.filletSteps+=program.filletSteps.size();
            const auto filleted=retained_fillet::Build(current,program,stop,[&](const TopoDS_Shape& shape){return Charge(shape,stop,budget);});out.filletOutcome=filleted.outcome;
            if(filleted.outcome!=retained_fillet::Outcome::Built)return refuse();
            current=filleted.solid;
            if(!cut_display::Prepare(current,display,stop))return refuse();
        }
        out.phase="base-preservation-and-final-commitment";Commitment afterBase;
        if(!saved_cut_source_edit::Commit(retainedBase,stop,budget.streamBytes,afterBase)
            ||!(afterBase==out.retainedBase)||!saved_cut_source_edit::Commit(current,stop,budget.streamBytes,out.finalResult)
            ||stop.load())return refuse();
        out.budget=budget;out.solid=std::move(current);out.status=Status::Built;out.phase="built";return out;
    }catch(...){return refuse();}
}
inline Result Build(const TopoDS_Shape& retainedBase,const Program& program,
    const cut_display::Settings& display,const std::atomic_bool& stop) noexcept {
    Budget budget;return Build(retainedBase,program,display,stop,budget);
}
// Mesh-independent exact V3 stream is the retained-fillet determinism check.
// This is deliberately separate from the content guard, which also binds meshes.
inline bool GeometryCommit(const TopoDS_Shape& shape,const std::atomic_bool& stop,std::size_t& bytes,Commitment& out){
    saved_cut_source_edit::CommitmentStream sink(stop,bytes);std::ostream stream(&sink);stream.imbue(std::locale::classic());
    BRepTools::Write(shape,stream,Standard_False,Standard_False,TopTools_FormatVersion_VERSION_3);
    return stream.good()&&sink.finish(out)&&!stop.load();
}
// A bounded native binary readback reproduces OCCT's documented persistence
// phase (gp_Dir/gp_Ax re-construction). It is applied ONLY to the independently
// rebuilt expectation, never to an observed candidate, and performs no numeric
// rounding/zero rewriting. Policy12289 accepts exact initial or exact readback
// V3 bytes; all other representations decline.
class FilletReadbackBuffer final:public std::streambuf {
    std::vector<char> bytes;
    std::size_t extent=0;
public:
    FilletReadbackBuffer():bytes(saved_cut_source_edit::MaximumShapeBytes){setp(bytes.data(),bytes.data()+bytes.size());}
    std::size_t size()const{return std::max(extent,std::size_t(pptr()-pbase()));}
    void read(){extent=size();setg(bytes.data(),bytes.data(),bytes.data()+extent);}
protected:
    int_type overflow(int_type)override{return traits_type::eof();}
    pos_type seekoff(off_type offset,std::ios_base::seekdir dir,std::ios_base::openmode mode)override{
        extent=size();
        const bool input=mode==std::ios_base::in,output=mode==std::ios_base::out;
        if(!input&&!output)return pos_type(off_type(-1));
        const off_type current=input?off_type(gptr()-eback()):off_type(pptr()-pbase());
        const off_type origin=dir==std::ios_base::beg?0:dir==std::ios_base::cur?current:off_type(extent);
        const off_type position=origin+offset;
        if(position<0||position>off_type(input?extent:bytes.size()))return pos_type(off_type(-1));
        if(input)setg(bytes.data(),bytes.data()+position,bytes.data()+extent);
        else {setp(bytes.data(),bytes.data()+bytes.size());pbump(int(position));}
        return pos_type(position);
    }
    pos_type seekpos(pos_type position,std::ios_base::openmode mode)override{return seekoff(off_type(position),std::ios_base::beg,mode);}
};
inline bool PersistedGeometry(const TopoDS_Shape& shape,const std::atomic_bool& stop,Budget& budget,Commitment& out){
    FilletReadbackBuffer buffer;std::ostream writer(&buffer);
    BinTools::Write(shape,writer,Standard_False,Standard_False,BinTools_FormatVersion_VERSION_4);
    if(!writer.good()||stop.load()||buffer.size()>saved_cut_source_edit::MaximumWorkStreamBytes-budget.streamBytes)return false;
    budget.streamBytes+=buffer.size();buffer.read();std::istream reader(&buffer);TopoDS_Shape reopened;BinTools::Read(reopened,reader);
    return !reopened.IsNull()&&Charge(reopened,stop,budget)&&BRepCheck_Analyzer(reopened).IsValid()
        &&GeometryCommit(reopened,stop,budget.streamBytes,out);
}
inline bool VerifyCurrent(const TopoDS_Shape& base,const TopoDS_Shape& result,const Program& p,
    const cut_display::Settings& display,const std::atomic_bool& stop,Budget& budget,
    std::array<double,6>* canonicalBounds=nullptr) noexcept {
    if(canonicalBounds)*canonicalBounds={};
    try {
        if(p.filletSteps.empty())return PreFilletInspection(result,p,stop).classification==saved_boolean_result::Classification::MatchedOrientedBoundary
            &&(!canonicalBounds||analytic_boolean::detail::Bounds(result,p.source.metersPerUnit*1000,*canonicalBounds));
        if(!Charge(result,stop,budget)||!BRepCheck_Analyzer(result).IsValid())return false;
        const auto rebuilt=Build(base,p,display,stop,budget);Commitment actual,expected;
        if(rebuilt.status!=Status::Built||!GeometryCommit(result,stop,budget.streamBytes,actual)
            ||!GeometryCommit(rebuilt.solid,stop,budget.streamBytes,expected))return false;
        if(!(actual==expected)&&!(PersistedGeometry(rebuilt.solid,stop,budget,expected)&&actual==expected))return false;
        // Binary readback renormalizes surface axes, which can shift analytic
        // extrema by one ULP. Once the actual carrier is proven above, report
        // its bounds through the same recipe rebuild on commit, redo and reopen.
        // Measure the complete filleted solid, not its unfilleted source box.
        // No rounding or tolerance substitution enters the evidence.
        return !canonicalBounds||analytic_boolean::detail::Bounds(rebuilt.solid,p.source.metersPerUnit*1000,*canonicalBounds);
    }catch(...){return false;}
}
} // namespace core3d::saved_boolean_build
