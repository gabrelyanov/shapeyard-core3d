#pragma once
// Real detached complete-program geometry. This is not an ordinary/AI permit.
#include "SavedBooleanResultCorrespondence.hxx"
#include "RetainedBooleanEditValues.hxx"
#include "SavedCutSourceEdit.hxx"
#include "AnalyticBooleanSolid.hxx"
#include "CutDisplayPreparation.hxx"
namespace core3d::saved_boolean_build {
using Program=retained_boolean::Program;
using Commitment=saved_cut_source_edit::ShapeCommitment;
enum class Status { Refused, Cancelled, Built };
struct Budget {
    std::size_t shapeOccurrences=0,streamBytes=0,booleanSteps=0;
    static constexpr std::size_t MaximumOccurrences=65536;
    static constexpr std::size_t MaximumSteps=retained_boolean::MaximumOperands;
};
struct Result {
    Status status=Status::Refused;
    const char* phase="input";
    TopoDS_Shape solid;
    Commitment retainedBase,finalResult;
    std::vector<std::uint8_t> exactProgram;
    saved_boolean_result::Inspection correspondence;
    Budget budget;
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
        if(retained_boolean::HasRing(changed)&&(!saved_boolean_result::detail::AdmitDisks(original)
            ||!saved_boolean_result::detail::AdmitDisks(changed)))return {};
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
        if(stop.load()||!p||!retained_boolean::HasRing(*p))return false;
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
    return p&&retained_boolean::HasRing(*p)?PrepareRingSourceValues(original,patch,stop,out):legacy(original,patch,stop,out);
}
// Aggregate-budget overload: the caller-owned budget accumulates occurrence
// and stream work across old capture, private copies and this rebuild, so the
// whole source-edit job shares one bounded allowance instead of a fresh
// per-substep budget. The original 4-argument form keeps its own local budget.
inline Result Build(const TopoDS_Shape& retainedBase,const Program& program,
    const cut_display::Settings& display,const std::atomic_bool& stop,Budget& budget) noexcept {
    Result out;const auto refuse=[&](){Result empty;empty.status=stop.load()?Status::Cancelled:Status::Refused;empty.budget=budget;empty.phase=out.phase;empty.correspondence=out.correspondence;return empty;};
    try {
        if((display.type!=Aspect_TOD_RELATIVE&&display.type!=Aspect_TOD_ABSOLUTE)||!display.automatic)return refuse();
        for(double value:display.values)if(!std::isfinite(value))return refuse();
        if(display.values[0]<=0||display.values[1]<=0||display.values[1]>=M_PI||display.values[2]<=0
            ||(display.ownCoefficient&&std::abs(display.values[0]-display.values[3])>Precision::Confusion())
            ||(display.ownAngle&&std::abs(display.values[1]-display.values[4])>Precision::Angular()))return refuse();
        out.phase="source-and-program";
        if(stop.load()||!saved_boolean_result::detail::SeparateDisks(program)
            ||!retained_boolean::Encode(program,out.exactProgram)||!Charge(retainedBase,stop,budget)
            ||!saved_cut_source_edit::Commit(retainedBase,stop,budget.streamBytes,out.retainedBase))return refuse();
        std::vector<retained_boolean::Disk> disks;if(!retained_boolean::ExpandedDisks(program,disks))return refuse();
        for(const auto& disk:disks){
            const auto view=saved_boolean_result::detail::GeometryView(program,disk.operand);
            if(stop.load()||!saved_cut_source_edit::InspectBase(retainedBase,view,stop)
                ||saved_cut_bore_clearance::Inspect(view).status!=saved_cut_bore_clearance::Status::ClearRecipeDisk)return refuse();
        }
        // Each existing primitive makes an independent deep geometry copy.
        // The original base is never replaced by the displayed/current result.
        TopoDS_Shape current=retainedBase;
        for(const auto& step:program.steps){
            if(stop.load()||budget.booleanSteps>=Budget::MaximumSteps)return refuse();
            ++budget.booleanSteps;out.phase="sequential-boolean";analytic_boolean::Recipe recipe;
            recipe.metersPerUnit=program.source.metersPerUnit;recipe.operation=step.operation;recipe.tool=step.operand;
            analytic_boolean::Result built;
            if(analytic_boolean::Build(current,recipe,stop,built)!=analytic_boolean::Status::Built
                ||!Charge(built.solid,stop,budget))return refuse();
            current=std::move(built.solid);
        }
        out.phase="display-preparation";
        if(stop.load()||!cut_display::Prepare(current,display,stop))return refuse();
        out.phase="complete-program-correspondence";out.correspondence=saved_boolean_result::Inspect(current,program,stop);
        if(out.correspondence.classification!=saved_boolean_result::Classification::MatchedOrientedBoundary)return refuse();
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
} // namespace core3d::saved_boolean_build
