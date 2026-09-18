#pragma once
#include "RetainedBooleanProgram.hxx"
#include "CylindricalCutDefinition.hxx"
#include "SavedBooleanResultCorrespondence.hxx"
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
            if(found==program.steps.end()||found->operand.kind!=analytic_boolean::OperandKind::Cylinder)return {};
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
struct AppendRing {cylindrical_cut::RingCreateEdit edit;};
struct SetRingRadius {std::uint32_t operandID=0;double worldHoleRadiusMM=0;};
// Physical bolt radius is converted using the current occurrence scale.
struct SetRingBoltRadius {std::uint32_t operandID=0;double worldBoltRadiusMM=0;};
struct SetRingCount {std::uint32_t operandID=0;std::uint32_t count=0;};
struct AppendWedge {wedge_cut::CreateEdit edit;};
struct SetWedgeWidths {std::uint32_t operandID=0;double worldHalfWidthApexMM=0,worldHalfWidthMouthMM=0;};
struct SetWedgeLength {std::uint32_t operandID=0;double worldLengthMM=0;};
struct AppendFilletStep {double radiusMM=0;std::vector<retained_fillet::EdgeAnchor> anchors;};
struct SetFilletRadius {std::uint64_t stepIdentifier=0;double radiusMM=0;};
struct RemoveFilletStep {std::uint64_t stepIdentifier=0;};
using ProgramEdit=std::variant<AppendBore,SetBoreRadius,AppendRing,SetRingRadius,SetRingCount,SetRingBoltRadius,AppendWedge,SetWedgeWidths,SetWedgeLength,AppendFilletStep,SetFilletRadius,RemoveFilletStep>;
inline bool FilletEdit(const ProgramEdit& e){return std::holds_alternative<AppendFilletStep>(e)||std::holds_alternative<SetFilletRadius>(e)||std::holds_alternative<RemoveFilletStep>(e);}
inline bool WedgeEdit(const ProgramEdit& edit){return std::holds_alternative<AppendWedge>(edit)
    ||std::holds_alternative<SetWedgeWidths>(edit)||std::holds_alternative<SetWedgeLength>(edit);}
inline bool RingEdit(const ProgramEdit& edit){return std::holds_alternative<AppendRing>(edit)
    ||std::holds_alternative<SetRingRadius>(edit)||std::holds_alternative<SetRingCount>(edit)
    ||std::holds_alternative<SetRingBoltRadius>(edit);}
inline bool HasRing(const Program& p){return std::any_of(p.steps.begin(),p.steps.end(),[](const auto& s){
    return s.operand.kind==analytic_boolean::OperandKind::CylinderRing;});}
inline std::optional<Change> Apply(const Recipe& original,const ProgramEdit& edit,double effectiveMM) noexcept {
    try {
        if(const auto* append=std::get_if<AppendBore>(&edit))return Append(original,append->edit,effectiveMM);
        if(const auto* radius=std::get_if<SetBoreRadius>(&edit))return Radius(original,radius->operandID,radius->worldRadiusMM,effectiveMM);
        Change result;if(!Encode(original,result.oldBytes))return {};
        Program p;if(const auto* legacy=std::get_if<Legacy>(&original)){if(!Promote(*legacy,p))return {};}
        else p=std::get<Program>(original);
        if(FilletEdit(edit)){
            if(const auto* a=std::get_if<AppendFilletStep>(&edit)){
                if(p.nextFilletStepID==UINT64_MAX||p.nextFilletEdgeID>UINT64_MAX-a->anchors.size()
                    ||a->anchors.empty()||a->anchors.size()>retained_fillet::MaximumAnchors)return {};
                retained_fillet::Step step;step.stepIdentifier=p.nextFilletStepID++;
                if(!cylindrical_cut::Radius(a->radiusMM,effectiveMM,step.radiusLocal))return {};
                step.anchors=a->anchors;for(auto& anchor:step.anchors)anchor.identifier=p.nextFilletEdgeID++;
                p.filletSteps.push_back(std::move(step));p.codecMinor=4;
            }else{
                const auto* radius=std::get_if<SetFilletRadius>(&edit);const auto* remove=std::get_if<RemoveFilletStep>(&edit);
                const auto id=radius?radius->stepIdentifier:remove->stepIdentifier;
                auto it=std::find_if(p.filletSteps.begin(),p.filletSteps.end(),[&](const retained_fillet::Step& step){return step.stepIdentifier==id;});
                if(it==p.filletSteps.end())return {};
                if(radius){double local=0;if(!cylindrical_cut::Radius(radius->radiusMM,effectiveMM,local))return {};
                    if(radius->radiusMM!=it->radiusLocal*effectiveMM)it->radiusLocal=local;}
                else p.filletSteps.erase(it);
            }
            result.selectedOperandID=p.steps.front().operand.identifier;
            result.recipe=std::move(p);if(!Encode(result.recipe,result.newBytes))return {};
            result.changed=result.oldBytes!=result.newBytes;return result;
        }
        if(WedgeEdit(edit)){
            if(const auto* append=std::get_if<AppendWedge>(&edit)){
                if(p.steps.size()>=MaximumOperands||p.nextOperandID>UINT32_MAX)return {};
                const auto& w=append->edit;Step step;auto& t=step.operand;t.identifier=std::uint32_t(p.nextOperandID);
                t.kind=analytic_boolean::OperandKind::Wedge;t.axis=w.axis;t.point=w.localApex;
                if(!std::isfinite(w.directionAngle))CORE3D_CUT_REFUSE("wedge.invalid-angle", {});
                t.directionAngle=std::fmod(w.directionAngle,2*std::acos(-1.));if(t.directionAngle<0)t.directionAngle+=2*std::acos(-1.);
                if(t.directionAngle==0)t.directionAngle=0; // canonical positive zero at capture
                if(!wedge_cut::Widths(w.worldHalfWidthApexMM,w.worldHalfWidthMouthMM,effectiveMM,t.halfWidthApex,t.halfWidthMouth)
                    ||!wedge_cut::Dimension(w.worldLengthMM,effectiveMM,.1,t.length))CORE3D_CUT_REFUSE("wedge.invalid-dimensions", {});
                p.codecMinor=std::max<std::uint8_t>(3,p.codecMinor);result.selectedOperandID=t.identifier;p.steps.push_back(step);++p.nextOperandID;
            }else{
                const auto* widths=std::get_if<SetWedgeWidths>(&edit);const auto* length=std::get_if<SetWedgeLength>(&edit);
                if(!widths&&!length)return {};const auto id=widths?widths->operandID:length->operandID;
                auto it=std::find_if(p.steps.begin(),p.steps.end(),[&](const Step& s){return s.operand.identifier==id;});
                if(it==p.steps.end()||it->operand.kind!=analytic_boolean::OperandKind::Wedge)CORE3D_CUT_REFUSE("wedge.operand-id-or-kind", {});
                auto& t=it->operand;
                if(widths){double a=0,b=0;if(!wedge_cut::Widths(widths->worldHalfWidthApexMM,widths->worldHalfWidthMouthMM,effectiveMM,a,b))return {};
                    if(widths->worldHalfWidthApexMM!=t.halfWidthApex*effectiveMM)t.halfWidthApex=a;
                    if(widths->worldHalfWidthMouthMM!=t.halfWidthMouth*effectiveMM)t.halfWidthMouth=b;
                }else{double local=0;if(!wedge_cut::Dimension(length->worldLengthMM,effectiveMM,.1,local))return {};
                    if(length->worldLengthMM!=t.length*effectiveMM)t.length=local;}
                result.selectedOperandID=id;
            }
            if(!saved_boolean_result::detail::AdmitSections(p))CORE3D_CUT_REFUSE("wedge.section-or-host-admission", {});
            result.recipe=std::move(p);if(!Encode(result.recipe,result.newBytes))return {};
            result.changed=result.oldBytes!=result.newBytes;return result;
        }
        if(const auto* append=std::get_if<AppendRing>(&edit)){
            if(p.steps.size()>=MaximumOperands||p.nextOperandID>UINT32_MAX)return {};
            const auto& r=append->edit;Step step;auto& t=step.operand;
            t.identifier=std::uint32_t(p.nextOperandID);t.axis=r.axis;t.point=r.localCenter;
            if(!cylindrical_cut::Radius(r.worldHoleRadiusMM,effectiveMM,t.radius))return {};
            // Extent validation uses a plain value-only view of the host.
            double extent=0;if(!saved_boolean_result::detail::HostRadialExtent(p,t,extent))return {};
            t.kind=analytic_boolean::OperandKind::CylinderRing;t.boltCircleRadius=r.boltCircleRadius;
            t.hostRadiusRatio=r.boltCircleRadius/extent;t.count=r.count;p.codecMinor=std::max<std::uint8_t>(2,p.codecMinor);
            result.selectedOperandID=t.identifier;p.steps.push_back(step);++p.nextOperandID;
        }else{
            const auto* radius=std::get_if<SetRingRadius>(&edit);const auto* count=std::get_if<SetRingCount>(&edit);
            const auto* bolt=std::get_if<SetRingBoltRadius>(&edit);
            if(!radius&&!count&&!bolt)return {};
            const auto id=radius?radius->operandID:count?count->operandID:bolt->operandID;
            auto it=std::find_if(p.steps.begin(),p.steps.end(),[&](const auto& s){return s.operand.identifier==id;});
            if(it==p.steps.end()||it->operand.kind!=analytic_boolean::OperandKind::CylinderRing)return {};
            if(radius){double local=0;if(!cylindrical_cut::Radius(radius->worldHoleRadiusMM,effectiveMM,local))return {};
                if(radius->worldHoleRadiusMM!=it->operand.radius*effectiveMM)it->operand.radius=local;}
            else if(count)it->operand.count=count->count;
            else {
                double local=0,extent=0;
                if(!cylindrical_cut::Radius(bolt->worldBoltRadiusMM,effectiveMM,local)
                    ||!saved_boolean_result::detail::HostRadialExtent(p,it->operand,extent))return {};
                // Preserve exact bytes on no-op; on change anchor to TODAY'S
                // authored host extent, so later source edits retain the ratio.
                if(bolt->worldBoltRadiusMM!=it->operand.boltCircleRadius*effectiveMM){
                    it->operand.boltCircleRadius=local;it->operand.hostRadiusRatio=local/extent;
                }
            }
            result.selectedOperandID=id;
        }
        if(!saved_boolean_result::detail::AdmitSections(p))return {};
        result.recipe=std::move(p);if(!Encode(result.recipe,result.newBytes))return {};
        result.changed=result.oldBytes!=result.newBytes;return result;
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
        for(const auto& step:std::get<Program>(recipe).steps){const auto& t=step.operand;
            if(t.kind==analytic_boolean::OperandKind::Wedge){double local=0;
                if(!wedge_cut::Dimension(t.halfWidthApex*factor,1,.05,local)||!wedge_cut::Dimension(t.halfWidthMouth*factor,1,.05,local)
                    ||!wedge_cut::Dimension(t.length*factor,1,.1,local))return false;
            }else if(!supported(t.radius))return false;
        }
        for(const auto& step:std::get<Program>(recipe).filletSteps)if(!supported(step.radiusLocal))return false;
        return true;
    }catch(...){return false;}
}
} // namespace core3d::retained_boolean
