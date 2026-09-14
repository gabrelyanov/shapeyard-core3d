#pragma once
#include "RetainedBooleanProgram.hxx"
#include "CylindricalCutDefinition.hxx"
namespace core3d::retained_boolean {
struct Change {
    Recipe recipe;
    std::vector<std::uint8_t> oldBytes,newBytes;
    std::uint32_t selectedOperandID=0;
    bool changed=false;
};
inline std::optional<Change> Append(const Recipe& original,const cylindrical_cut::CreateEdit& edit,
                                  double effectiveMM) noexcept {
    try {
        Change result;if(!Encode(original,result.oldBytes))return {};
        Program program;if(const auto* legacy=std::get_if<Legacy>(&original)){
            if(!Promote(*legacy,program))return {};
        }else program=std::get<Program>(original);
        if(!Valid(program)||program.steps.size()>=MaximumOperands||program.nextOperandID>UINT32_MAX)return {};
        double radius=0;if(!cylindrical_cut::Radius(edit.worldRadiusMM,effectiveMM,radius))return {};
        Step next;next.operand.identifier=std::uint32_t(program.nextOperandID);
        next.operand.axis=edit.axis;next.operand.point=edit.localCenter;next.operand.radius=radius;
        ++program.nextOperandID;program.steps.push_back(next);
        if(!Valid(program))return {};
        result.selectedOperandID=next.operand.identifier;result.changed=true;result.recipe=std::move(program);
        if(!Encode(result.recipe,result.newBytes))return {};return result;
    }catch(...){return {};}
}
inline std::optional<Change> Radius(const Recipe& original,std::uint32_t operandID,
                                  double worldMM,double effectiveMM) noexcept {
    try {
        Change result;if(!operandID||!Encode(original,result.oldBytes))return {};
        result.recipe=original;result.selectedOperandID=operandID;
        if(auto* legacy=std::get_if<Legacy>(&result.recipe)){
            if(legacy->operandID!=operandID)return {};
            Legacy edited;if(!cylindrical_cut::Rebuild(*legacy,{worldMM},effectiveMM,edited))return {};
            *legacy=std::move(edited); // Remains legacy; even a changed radius does not migrate.
        }else{
            auto& program=std::get<Program>(result.recipe);double local=0;
            if(!cylindrical_cut::Radius(worldMM,effectiveMM,local))return {};
            auto found=std::find_if(program.steps.begin(),program.steps.end(),[&](const auto& step){return step.operand.identifier==operandID;});
            if(found==program.steps.end())return {};
            const double current=found->operand.radius*effectiveMM;
            if(!std::isfinite(current)||current<.001||current>1e6)return {};
            if(worldMM!=current)found->operand.radius=local;
            if(!Valid(program))return {};
        }
        if(!Encode(result.recipe,result.newBytes))return {};
        result.changed=result.oldBytes!=result.newBytes;return result;
    }catch(...){return {};}
}
// Explicit typed whole-program input, never a loose dictionary or an
// overloaded first-cut flag. Append issues one new persisted high-water ID;
// SetBoreRadius resolves the exact stable ID inside the original full recipe.
// Neither variant carries owner, command, geometry or prepared AI authority.
struct AppendBore { cylindrical_cut::CreateEdit edit; };
struct SetBoreRadius { std::uint32_t operandID=0; double worldRadiusMM=0; };
using ProgramEdit=std::variant<AppendBore,SetBoreRadius>;
inline std::optional<Change> Apply(const Recipe& original,const ProgramEdit& edit,double effectiveMM) noexcept {
    try {
        if(const auto* append=std::get_if<AppendBore>(&edit))return Append(original,append->edit,effectiveMM);
        const auto& radius=std::get<SetBoreRadius>(edit);
        return Radius(original,radius.operandID,radius.worldRadiusMM,effectiveMM);
    }catch(...){return {};}
}
// Occurrence-only placement scales every operand with the part; it never
// rewrites a tool, the retained base or the recipe. Each resulting world
// radius must stay inside the same supported admission domain as edits.
inline bool OccurrenceRadiiMM(const Recipe& recipe,const gp_Trsf& transform) noexcept {
    try {
        double scale=0,factor=0;
        if(!cylindrical_cut::EffectiveMM(transform,Identities(recipe).metersPerUnit,scale,factor))return false;
        const auto supported=[&](double radius){const double world=radius*factor;
            return std::isfinite(world)&&world>=.001&&world<=1e6;};
        if(const auto* legacy=std::get_if<Legacy>(&recipe))return supported(legacy->radius);
        for(const auto& step:std::get<Program>(recipe).steps)if(!supported(step.operand.radius))return false;
        return true;
    }catch(...){return false;}
}
} // namespace core3d::retained_boolean
