#pragma once
#include "RetainedRecipeSnapshot.hxx"

namespace core3d::retained_recipe {
enum class OperationKind : std::uint8_t {
    PartBoolean = 1, EditInput = 2, EditFeature = 3, SpatialCircleSweep = 4
};
enum class Refusal : std::uint8_t {
    None = 0,
    SnapshotNotCurrent,
    SourceCount,
    SourceFamily,
    FeatureKind,
    SelectorVersion,
    ProofProfile,
    ParameterBounds,
    RuleNotInstalled,
};

struct AdmissionRule {
    OperationKind operation = OperationKind::PartBoolean;
    std::uint32_t featureKind = 0, featureCodecVersion = 0, selectorVersion = 0, proofProfile = 0;
    std::vector<composite_recipe::RecipeKind> orderedSourceKinds;
    Digest parameterBounds{};
    bool nativeBuilderInstalled = false, nativeProofInstalled = false;
    bool nativeOwnerInstalled = false, retainedInputsPersistenceInstalled = false;
};

struct AdmissionDecision {
    Refusal refusal = Refusal::RuleNotInstalled;
    std::vector<RecipeLocator> completeReadSet;
    bool detachedCandidateOnly = true;
    bool admitted() const noexcept { return refusal == Refusal::None; }
};

inline AdmissionDecision Evaluate(const OwnerSnapshot& snapshot,
                                  const AdmissionRule& rule) noexcept {
    AdmissionDecision result;
    try {
        for (const auto& source : snapshot.sources) result.completeReadSet.push_back(source.locator);
        if (snapshot.status != OwnerStatus::CurrentEditable) { result.refusal = Refusal::SnapshotNotCurrent; return result; }
        if (!rule.nativeBuilderInstalled || !rule.nativeProofInstalled
            || !rule.nativeOwnerInstalled || !rule.retainedInputsPersistenceInstalled) return result;
        const bool partBoolean = rule.featureKind == composite_recipe::PartBooleanFeatureKind
            && (rule.featureCodecVersion == composite_recipe::PartBooleanFeatureCodec
                || (rule.featureCodecVersion == composite_recipe::PartBooleanShellFeatureCodec
                    && rule.operation != OperationKind::PartBoolean));
        const bool spatialSweep = rule.operation == OperationKind::SpatialCircleSweep
            && rule.featureKind == composite_recipe::SpatialCircleSweepFeatureKind
            && rule.featureCodecVersion == composite_recipe::SpatialCircleSweepFeatureCodec;
        if (!partBoolean && !spatialSweep) {
            result.refusal = Refusal::FeatureKind; return result;
        }
        if (rule.selectorVersion == 0) { result.refusal = Refusal::SelectorVersion; return result; }
        if (rule.proofProfile == 0) { result.refusal = Refusal::ProofProfile; return result; }
        if (!Nonzero(rule.parameterBounds)) { result.refusal = Refusal::ParameterBounds; return result; }
        if (snapshot.sources.size() != rule.orderedSourceKinds.size()) { result.refusal = Refusal::SourceCount; return result; }
        for (std::size_t index = 0; index < snapshot.sources.size(); ++index)
            if (snapshot.sources[index].recipe.kind != rule.orderedSourceKinds[index]) {
                result.refusal = Refusal::SourceFamily; return result;
            }
        result.refusal = Refusal::None;
        // Only the native owner evaluates a CurrentEditable snapshot carrying
        // its process-local receipt. Public analytic and shell dispatch remain
        // separately disabled; this flag only distinguishes a complete native
        // prepared change from a detached candidate.
        result.detachedCandidateOnly = false;
        return result;
    } catch (...) { result.refusal = Refusal::RuleNotInstalled; result.completeReadSet.clear(); return result; }
}

inline AdmissionDecision P1NoFeatureAdmission(const OwnerSnapshot& snapshot) noexcept {
    AdmissionDecision result;
    for (const auto& source : snapshot.sources) result.completeReadSet.push_back(source.locator);
    result.refusal = snapshot.status == OwnerStatus::CurrentEditable
        ? Refusal::RuleNotInstalled : Refusal::SnapshotNotCurrent;
    return result;
}

enum class V3MutationKind : std::uint8_t { EditSource = 1, EditFeature = 2, AppendFeature = 3, RemoveFeature = 4 };
struct V3MutationRequest final {
    V3MutationKind operation = V3MutationKind::EditFeature;
    RecipeLocator target;
    std::uint32_t featureKind = 0, featureCodecVersion = 0;
    std::vector<std::uint8_t> canonicalParameters;
};

inline AdmissionDecision EvaluateV3(const OwnerSnapshot& snapshot,
                                    const V3MutationRequest& request,
                                    const retained_feature::RegistryView& registry
                                        = retained_feature::ProductionRegistry()) noexcept {
    AdmissionDecision result;
    try {
        for (const auto& source : snapshot.sources) result.completeReadSet.push_back(source.locator);
        for (const auto& feature : snapshot.features) result.completeReadSet.push_back(feature.locator);
        if (snapshot.status != OwnerStatus::CurrentEditable) {
            result.refusal = Refusal::SnapshotNotCurrent; return result;
        }
        const auto* entry = registry.find({request.featureKind, request.featureCodecVersion});
        if (!entry || !entry->execution.installed()) {
            result.refusal = entry ? Refusal::RuleNotInstalled : Refusal::FeatureKind; return result;
        }
        const auto found = std::find_if(snapshot.features.begin(), snapshot.features.end(),
            [&](const FeatureSnapshot& value) { return value.locator == request.target; });
        if ((request.operation == V3MutationKind::EditFeature
                || request.operation == V3MutationKind::RemoveFeature)
            && found == snapshot.features.end()) {
            result.refusal = Refusal::FeatureKind; return result;
        }
        if (request.operation != V3MutationKind::RemoveFeature
            && (request.canonicalParameters.empty()
                || request.canonicalParameters.size() > entry->codec.maximumPayloadBytes)) {
            result.refusal = Refusal::ParameterBounds; return result;
        }
        result.refusal = Refusal::None; result.detachedCandidateOnly = false;
        return result;
    } catch (...) { result.completeReadSet.clear(); result.refusal = Refusal::RuleNotInstalled; return result; }
}
} // namespace core3d::retained_recipe
