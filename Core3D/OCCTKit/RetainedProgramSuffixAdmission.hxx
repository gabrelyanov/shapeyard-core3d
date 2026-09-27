#pragma once
// A3/P2 typed admission for appending a retained program suffix to a composite
// Boolean owner. Value-only evaluation: no document handle, label, command or
// geometry authority. Every refusal is typed and occurs before geometry work;
// while the family-admission switch stays false the decision is always
// AdmissionDisabled AFTER the structural checks, so the refusal ordering is
// deterministic and observable in tests.
#include "RetainedProgramSuffix.hxx"
#include "CompositeRecipeCodec.hxx"
#include "RetainedRecipeSnapshot.hxx"
#include "PartBooleanFamilyAdmission.hxx"

namespace core3d::retained_program_suffix {

enum class Refusal : std::uint8_t {
    None = 0,
    AdmissionDisabled,     // native route switch off (current production posture)
    RuleNotInstalled,      // exception/unknown path
    OwnerMalformed,        // composite graph fails its own codec validity
    OwnerAmbiguous,        // foreign fence owner or foreign dependency owner
    StaleFence,            // fence invalid, missing a source read, or commitment drift
    UnsupportedBaseFamily, // output is not a PartBoolean feature (e.g. already suffixed)
    BaseFeatureMismatch,   // program.baseFeature != the owner output feature node
    ProgramOversized,      // step/section/payload budgets exceeded
    ProgramInvalid,        // SYPS value validation failed
};

struct Request final {
    composite_recipe::Definition graph;  // decoded owner BEFORE the append
    retained_recipe::RevisionFence fence;
    Definition program;                  // proposed suffix values
};

struct Decision final {
    Refusal refusal = Refusal::RuleNotInstalled;
    std::vector<retained_recipe::RecipeLocator> completeReadSet;
    bool admitted() const noexcept { return refusal == Refusal::None; }
};

// The base must be the owner's current output PartBoolean feature in SYCR/3,
// using analytic codec 1 or shell codec 2. An owner whose output is
// already a suffix is an EDIT of that suffix, not an append, and refuses here.
inline bool BaseFeatureIsPartBoolean(const composite_recipe::Definition& graph) noexcept {
    try {
        if (graph.nodes.empty()) return false;
        const auto* feature = std::get_if<composite_recipe::FeatureNode>(&graph.nodes.back().value);
        if (!feature || graph.outputNode != feature->node
            || feature->kind != composite_recipe::PartBooleanFeatureKind) return false;
        return graph.schemaVersion == 3
            && (feature->codecVersion == composite_recipe::PartBooleanFeatureCodec
                || feature->codecVersion == composite_recipe::PartBooleanShellFeatureCodec);
    } catch (...) { return false; }
}

// Capacity classification is intentionally independent of Valid(): Valid()
// enforces both value well-formedness and these limits, so using it as the
// prerequisite would make an otherwise well-formed overflow look malformed.
// The arithmetic below is bounded before every addition/multiplication and
// models the complete SYPS/1 payload, including its trailing digest.
inline bool CapacityExceeded(const Definition& program) noexcept {
    try {
        if (program.steps.size() > MaximumSteps) return true;

        std::size_t sections = 0;
        for (const auto& step : program.steps) {
            const std::size_t added = step.operand.kind
                == analytic_boolean::OperandKind::CylinderRing
                ? std::size_t(step.operand.count) : 1;
            if (added > analytic_boolean_ring::kMaximumExpandedDisks - sections)
                return true;
            sections += added;
        }

        constexpr std::size_t payloadLimit = MaximumPayloadBytes - 32;
        std::size_t payloadBytes = 8 + 32 + 8 + 8 + 12 + 4 + 8 + 32;
        const auto add = [&](std::size_t count, std::size_t width) {
            if (count > (payloadLimit - payloadBytes) / width) return false;
            payloadBytes += count * width;
            return true;
        };
        if (!add(program.steps.size(), 96) || !add(1, 24)) return true;
        for (const auto& fillet : program.filletSteps)
            if (!add(1, 24) || !add(fillet.anchors.size(), 65)) return true;
        return payloadBytes > payloadLimit;
    } catch (...) { return true; }
}

inline Decision Evaluate(const Request& request) noexcept {
    Decision result;
    try {
        const composite_recipe::Definition& graph = request.graph;
        for (const composite_recipe::Node& node : graph.nodes)
            if (const auto* source = std::get_if<composite_recipe::SourceNode>(&node.value))
                result.completeReadSet.push_back(
                    {graph.owner, source->node, source->original.sourceFeature});
        if (!composite_recipe::Valid(graph)) {
            result.refusal = Refusal::OwnerMalformed; return result;
        }
        // ProgramOversized is distinguished from ProgramInvalid so capacity
        // failures are measurable separately from malformed values.
        if (CapacityExceeded(request.program)) {
            result.refusal = Refusal::ProgramOversized; return result;
        }
        if (!Valid(request.program)) { result.refusal = Refusal::ProgramInvalid; return result; }
        if (!BaseFeatureIsPartBoolean(graph)) {
            result.refusal = Refusal::UnsupportedBaseFamily; return result;
        }
        const auto& output = std::get<composite_recipe::FeatureNode>(graph.nodes.back().value);
        if (request.program.baseFeature != output.node) {
            result.refusal = Refusal::BaseFeatureMismatch; return result;
        }
        if (!retained_recipe::Valid(request.fence)) {
            result.refusal = Refusal::StaleFence; return result;
        }
        if (!(request.fence.dependencies.front().locator.owner == graph.owner)) {
            result.refusal = Refusal::OwnerAmbiguous; return result;
        }
        // Every retained source must be read with its exact commitments; a
        // changed second operand is stale even when the carrier is untouched.
        for (const composite_recipe::Node& node : graph.nodes) {
            const auto* source = std::get_if<composite_recipe::SourceNode>(&node.value);
            if (!source) continue;
            const retained_recipe::RecipeLocator locator{
                graph.owner, source->node, source->original.sourceFeature};
            const auto read = std::find_if(request.fence.dependencies.begin(),
                request.fence.dependencies.end(),
                [&](const retained_recipe::DependencyRead& value) {
                    return value.locator == locator;
                });
            if (read == request.fence.dependencies.end()
                || read->geometry != source->commitments.geometry
                || read->recipe != source->commitments.recipe
                || read->placement != source->commitments.placement
                || read->material != source->commitments.material
                || read->groups != source->commitments.groups) {
                result.refusal = Refusal::StaleFence; return result;
            }
        }
        for (const auto& read : request.fence.dependencies)
            if (!(read.locator.owner == graph.owner)) {
                result.refusal = Refusal::OwnerAmbiguous; return result;
            }
        // The compile-time route switch is evaluated LAST: a structurally
        // complete append on a current owner still refuses closed until the
        // guarded qualification receipt promotes the family flag.
        if (!part_boolean::family_admission::RetainedProgramSuffixRouteInstalled) {
            result.refusal = Refusal::AdmissionDisabled; return result;
        }
        result.refusal = Refusal::None;
        return result;
    } catch (...) {
        result.refusal = Refusal::RuleNotInstalled;
        result.completeReadSet.clear();
        return result;
    }
}
} // namespace core3d::retained_program_suffix
