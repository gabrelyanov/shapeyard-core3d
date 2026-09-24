#pragma once
#include "RetainedRecipeSnapshot.hxx"

namespace core3d::retained_recipe {
enum class OperationKind : std::uint8_t { PartBoolean = 1, EditInput = 2, EditFeature = 3 };
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
        if (!rule.nativeBuilderInstalled || !rule.nativeProofInstalled) return result;
        if (rule.featureKind != composite_recipe::PartBooleanFeatureKind
            || rule.featureCodecVersion != composite_recipe::PartBooleanFeatureCodec) {
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
        result.refusal = Refusal::None; return result;
    } catch (...) { result.refusal = Refusal::RuleNotInstalled; result.completeReadSet.clear(); return result; }
}

inline AdmissionDecision P1NoFeatureAdmission(const OwnerSnapshot& snapshot) noexcept {
    AdmissionDecision result;
    for (const auto& source : snapshot.sources) result.completeReadSet.push_back(source.locator);
    result.refusal = snapshot.status == OwnerStatus::CurrentEditable
        ? Refusal::RuleNotInstalled : Refusal::SnapshotNotCurrent;
    return result;
}
} // namespace core3d::retained_recipe
