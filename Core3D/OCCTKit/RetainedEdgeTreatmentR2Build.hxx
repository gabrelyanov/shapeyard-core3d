#pragma once

#include "RetainedEdgeTreatmentR2Snapshot.hxx"
#include "RetainedEdgeTreatmentBuild.hxx"
#include "CompositeRecipeCodec.hxx"
#include <BRepCheck_Analyzer.hxx>

namespace core3d::retained_edge_treatment::r2 {
inline bool ValidateCompletePrefix(const RetainedBooleanBase& source,
                                   const BooleanBaseBinding& binding,
                                   et::Refusal& refusal) noexcept {
    try {
        if (!detail::Valid(binding)) { refusal = et::Refusal::MalformedCarrier; return false; }
        const auto exact = std::visit([](const auto& value) -> const std::vector<std::uint8_t>& {
            return value.canonicalPrefixBytes;
        }, source.source);
        if (exact.empty() || exact.size() > et::MaximumEnvelopeBytes) { refusal = et::Refusal::Budget; return false; }
        Digest digest{};
        if (!CC_SHA256(exact.data(), CC_LONG(exact.size()), digest.data()) || digest != binding.sourceRecipeDigest) {
            refusal = et::Refusal::NoncurrentSource; return false;
        }
        if (binding.format == PrefixFormat::SYRS) {
            const auto* legacy = std::get_if<LegacyBooleanBase>(&source.source);
            std::vector<std::uint8_t> encoded;
            if (!legacy || !retained_boolean::Encode(legacy->prefix, encoded) || encoded != exact) {
                refusal = et::Refusal::MalformedCarrier; return false;
            }
        } else if (!std::holds_alternative<CompositeBooleanBase>(source.source)) {
            refusal = et::Refusal::MalformedCarrier; return false;
        }
        refusal = et::Refusal::None; return true;
    } catch (...) { refusal = et::Refusal::MalformedCarrier; return false; }
}

// Geometry builders consume copied shapes and values only.  The document,
// labels, AIS objects and proof handles remain on the main-owned Work object.
inline bool ReplayTreatmentSuffix(const TopoDS_Shape& postBoolean,
                                  const Definition& definition,
                                  TopoDS_Shape& output,
                                  et::ReplayBudget& budget,
                                  et::Refusal& refusal) noexcept {
    if (postBoolean.IsNull() || !BRepCheck_Analyzer(postBoolean).IsValid()) {
        refusal = et::Refusal::UnsupportedBase; return false;
    }
    et::BaseBinding shadow;
    if (const auto* old = std::get_if<et::BaseBinding>(&definition.base)) shadow = *old;
    else {
        const auto& source = std::get<BooleanBaseBinding>(definition.base);
        shadow.source = source.source; shadow.sourceNode = source.sourceNode;
        shadow.sourceSchema = source.sourceSchema; shadow.sourceRecipeDigest = source.sourceRecipeDigest;
        shadow.metersPerLocalUnit = source.metersPerLocalUnit;
    }
    et::Definition suffix{definition.schema, definition.owner, shadow, definition.issuance,
                          definition.outputNode, definition.steps};
    std::vector<et::StepProof> proofs;
    return et::Replay(postBoolean, suffix, output, proofs, budget, refusal);
}
} // namespace core3d::retained_edge_treatment::r2
