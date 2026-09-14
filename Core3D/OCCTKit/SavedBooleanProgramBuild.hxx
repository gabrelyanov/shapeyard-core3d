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
// All tool values/order/high-water remain exact while source fields are edited.
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
        if(!retained_boolean::Encode(result.recipe,result.newBytes))return {};
        result.changed=result.oldBytes!=result.newBytes;return result;
    }catch(...){return {};}
}
inline Result Build(const TopoDS_Shape& retainedBase,const Program& program,
    const cut_display::Settings& display,const std::atomic_bool& stop) noexcept {
    Result out;const auto refuse=[&](){Result empty;empty.status=stop.load()?Status::Cancelled:Status::Refused;empty.budget=out.budget;empty.phase=out.phase;empty.correspondence=out.correspondence;return empty;};
    try {
        if((display.type!=Aspect_TOD_RELATIVE&&display.type!=Aspect_TOD_ABSOLUTE)||!display.automatic)return refuse();
        for(double value:display.values)if(!std::isfinite(value))return refuse();
        if(display.values[0]<=0||display.values[1]<=0||display.values[1]>=M_PI||display.values[2]<=0
            ||(display.ownCoefficient&&std::abs(display.values[0]-display.values[3])>Precision::Confusion())
            ||(display.ownAngle&&std::abs(display.values[1]-display.values[4])>Precision::Angular()))return refuse();
        out.phase="source-and-program";
        if(stop.load()||!saved_boolean_result::detail::SeparateDisks(program)
            ||!retained_boolean::Encode(program,out.exactProgram)||!Charge(retainedBase,stop,out.budget)
            ||!saved_cut_source_edit::Commit(retainedBase,stop,out.budget.streamBytes,out.retainedBase))return refuse();
        for(std::size_t i=0;i<program.steps.size();++i){
            const auto view=saved_boolean_result::detail::GeometryView(program,i);
            if(stop.load()||!saved_cut_source_edit::InspectBase(retainedBase,view,stop)
                ||saved_cut_bore_clearance::Inspect(view).status!=saved_cut_bore_clearance::Status::ClearRecipeDisk)return refuse();
        }
        // Each existing primitive makes an independent deep geometry copy.
        // The original base is never replaced by the displayed/current result.
        TopoDS_Shape current=retainedBase;
        for(const auto& step:program.steps){
            if(stop.load()||out.budget.booleanSteps>=Budget::MaximumSteps)return refuse();
            ++out.budget.booleanSteps;out.phase="sequential-boolean";analytic_boolean::Recipe recipe;
            recipe.metersPerUnit=program.source.metersPerUnit;recipe.operation=step.operation;recipe.tool=step.operand;
            analytic_boolean::Result built;
            if(analytic_boolean::Build(current,recipe,stop,built)!=analytic_boolean::Status::Built
                ||!Charge(built.solid,stop,out.budget))return refuse();
            current=std::move(built.solid);
        }
        out.phase="display-preparation";
        if(stop.load()||!cut_display::Prepare(current,display,stop))return refuse();
        out.phase="complete-program-correspondence";out.correspondence=saved_boolean_result::Inspect(current,program,stop);
        if(out.correspondence.classification!=saved_boolean_result::Classification::MatchedOrientedBoundary)return refuse();
        out.phase="base-preservation-and-final-commitment";Commitment afterBase;
        if(!saved_cut_source_edit::Commit(retainedBase,stop,out.budget.streamBytes,afterBase)
            ||!(afterBase==out.retainedBase)||!saved_cut_source_edit::Commit(current,stop,out.budget.streamBytes,out.finalResult)
            ||stop.load())return refuse();
        out.solid=std::move(current);out.status=Status::Built;out.phase="built";return out;
    }catch(...){return refuse();}
}
} // namespace core3d::saved_boolean_build
