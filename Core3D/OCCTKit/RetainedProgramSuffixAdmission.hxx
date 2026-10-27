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
#include "PartBooleanProgramOperand.hxx"

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
// using analytic codec 1, shell codec 2, or readable originals codec 4. An owner whose output is
// already a suffix is an EDIT of that suffix, not an append, and refuses here.
inline bool BaseFeatureIsPartBoolean(const composite_recipe::Definition& graph) noexcept {
    try {
        if (graph.nodes.empty()) return false;
        const auto* feature = std::get_if<composite_recipe::FeatureNode>(&graph.nodes.back().value);
        if (!feature || graph.outputNode != feature->node
            || feature->kind != composite_recipe::PartBooleanFeatureKind) return false;
        return graph.schemaVersion == 3
            && (feature->codecVersion == composite_recipe::PartBooleanFeatureCodec
                || feature->codecVersion == composite_recipe::PartBooleanShellFeatureCodec
                || feature->codecVersion == composite_recipe::PartBooleanOriginalsFeatureCodec);
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

namespace core3d::part_boolean::program_operand {

inline bool SameBits(double left, double right) noexcept {
    return std::memcmp(&left, &right, sizeof(double)) == 0;
}

inline bool IdentityPlacement(const composite_recipe::InputPlacement& value) noexcept {
    static constexpr std::array<double, 16> identity{{
        1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
    }};
    return composite_recipe::ValidPlacement(value)
        && value.matrix == identity
        && SameBits(value.sourceMetersPerUnit, value.carrierMetersPerUnit);
}

inline std::uint8_t SourceFamily(const retained_boolean::Recipe& recipe) noexcept {
    if (const auto* legacy = std::get_if<retained_boolean::Legacy>(&recipe))
        return legacy->sourceFamily;
    return std::get<retained_boolean::Program>(recipe).source.family;
}

inline std::uint32_t SourceSchema(const retained_boolean::Recipe& recipe) noexcept {
    if (const auto* legacy = std::get_if<retained_boolean::Legacy>(&recipe))
        return legacy->sourceSchema;
    return std::get<retained_boolean::Program>(recipe).source.schema;
}

inline BaseFamily FamilyFor(const retained_boolean::Recipe& recipe) noexcept {
    switch (SourceFamily(recipe)) {
    case 1: return BaseFamily::Profile;
    case 2: return BaseFamily::Enclosure;
    case 3: return BaseFamily::RectangularLoft;
    default: return BaseFamily::ProfileSchema5Shell;
    }
}

inline bool ExactRecipe(const std::vector<std::uint8_t>& bytes,
                        const retained_recipe::Digest& digest,
                        const retained_boolean::Recipe& captured) noexcept {
    try {
        retained_recipe::Digest observed;
        if (!composite_recipe::Hash(bytes, observed) || observed != digest) return false;
        retained_boolean::Recipe decoded;
        std::vector<std::uint8_t> decodedBytes, capturedBytes;
        return retained_boolean::Decode(bytes, decoded)
            && retained_boolean::Encode(decoded, decodedBytes) && decodedBytes == bytes
            && retained_boolean::Encode(captured, capturedBytes) && capturedBytes == bytes;
    } catch (...) { return false; }
}

inline Refusal InspectValues(const Value& value) noexcept {
    try {
        const IdentityValues& identity = value.identity;
        if (!retained_recipe::Valid(identity.outerOwner)
            || !retained_recipe::Valid(identity.original)
            || !retained_recipe::Nonzero(identity.derivedFeature)
            || !retained_recipe::Nonzero(identity.sourceNode)
            || !retained_recipe::Nonzero(identity.outputFeature)
            || identity.sourceLocalID == 0 || identity.nextLocalID <= identity.sourceLocalID
            || value.recipe.originalBytes.empty() || value.recipe.effectiveBytes.empty())
            return Refusal::MissingSource;

        const auto oversized = [](const std::vector<std::uint8_t>& bytes) {
            return bytes.size() > retained_solid::MaximumEnvelopeBytes;
        };
        if (oversized(value.recipe.originalBytes) || oversized(value.recipe.effectiveBytes)
            || value.appearance.originalMaterials.size() > part_boolean::MaximumMetadataRows
            || value.appearance.effectiveMaterials.size() > part_boolean::MaximumMetadataRows
            || value.appearance.originalGroups.size() > part_boolean::MaximumMetadataRows
            || value.appearance.effectiveGroups.size() > part_boolean::MaximumMetadataRows
            || value.appearance.originalLayers.size() > part_boolean::MaximumMetadataRows
            || value.appearance.effectiveLayers.size() > part_boolean::MaximumMetadataRows
            || value.appearance.regions.size() > part_boolean::MaximumRegions
            || value.appearance.selectors.size() > part_boolean::MaximumSelectors)
            return Refusal::ProgramOversized;

        const auto versioned = [](const std::vector<std::uint8_t>& bytes) {
            return bytes.size() >= 8 && std::memcmp(bytes.data(), "SYRS", 4) == 0
                && (bytes[4] == 1 || bytes[4] == 2);
        };
        if (!versioned(value.recipe.originalBytes) || !versioned(value.recipe.effectiveBytes))
            return Refusal::ProgramVersionMismatch;

        retained_recipe::Digest originalDigest, effectiveDigest;
        if (!composite_recipe::Hash(value.recipe.originalBytes, originalDigest)
            || !composite_recipe::Hash(value.recipe.effectiveBytes, effectiveDigest)
            || originalDigest != value.recipe.originalDigest
            || effectiveDigest != value.recipe.effectiveDigest)
            return Refusal::ProgramDigestMismatch;
        if (!ExactRecipe(value.recipe.originalBytes, value.recipe.originalDigest,
                value.recipe.originalDecoded)
            || !ExactRecipe(value.recipe.effectiveBytes, value.recipe.effectiveDigest,
                value.recipe.effectiveDecoded))
            return Refusal::ProgramMalformed;

        if (value.carrier == Carrier::CompositeProgramSuffix)
            return Refusal::UnsupportedProgramCarrier;
        if (value.carrier == Carrier::DependentRoundOnly)
            return Refusal::UnsupportedDependentFeature;
        if (value.carrier != Carrier::CanonicalRetainedProgram)
            return Refusal::UnsupportedProgramCarrier;
        if (value.recipe.baseFamily == BaseFamily::ProfileSchema5Shell
            || FamilyFor(value.recipe.originalDecoded) != value.recipe.baseFamily
            || FamilyFor(value.recipe.effectiveDecoded) != value.recipe.baseFamily
            || (SourceFamily(value.recipe.originalDecoded) == 1
                && SourceSchema(value.recipe.originalDecoded) > 4))
            return Refusal::UnsupportedBaseFamily;
        if (!IdentityPlacement(value.sourceToCarrier)
            || !SameBits(value.sourceToCarrier.sourceMetersPerUnit,
                value.observedOwnerMetersPerUnit)
            || !SameBits(value.observedOwnerMetersPerUnit,
                value.observedDocumentMetersPerUnit))
            return Refusal::UnsupportedPlacement;

        const retained_boolean::Identity original =
            retained_boolean::Identities(value.recipe.originalDecoded);
        const retained_boolean::Identity effective =
            retained_boolean::Identities(value.recipe.effectiveDecoded);
        if (original.document != identity.original.document
            || original.entity != identity.original.entity
            || original.definition != identity.original.definition
            || original.sourceFeature != identity.original.sourceFeature
            || original.derivedFeature != identity.derivedFeature
            || effective.document != identity.original.document
            || effective.entity != identity.original.entity
            || effective.definition != identity.original.definition
            || effective.sourceFeature != identity.original.sourceFeature
            || effective.derivedFeature != identity.derivedFeature)
            return Refusal::BaseFeatureMismatch;
        if (!SameBits(original.metersPerUnit, value.sourceToCarrier.sourceMetersPerUnit)
            || !SameBits(effective.metersPerUnit, value.sourceToCarrier.sourceMetersPerUnit))
            return Refusal::UnsupportedPlacement;

        const AppearanceValues& appearance = value.appearance;
        if (appearance.materialPolicy != part_boolean::MaterialPolicy::RejectConflictingMerges
            || appearance.versions.serializer != 1 || appearance.versions.build != 1
            || appearance.versions.proof != 1 || appearance.versions.selector != 1
            || appearance.versions.material != 1
            || !part_boolean::ValidTables(appearance.originalMaterials,
                appearance.originalGroups, appearance.originalLayers)
            || !part_boolean::ValidTables(appearance.effectiveMaterials,
                appearance.effectiveGroups, appearance.effectiveLayers)
            || !part_boolean::Valid(appearance.original,
                appearance.originalMaterials.size(), appearance.originalGroups.size(),
                appearance.originalLayers.size())
            || !part_boolean::Valid(appearance.effective,
                appearance.effectiveMaterials.size(), appearance.effectiveGroups.size(),
                appearance.effectiveLayers.size())
            || !part_boolean::Valid(value.originalCommitments)
            || !part_boolean::Valid(value.effectiveCommitments))
            return Refusal::ProgramMalformed;

        const FenceValues& fence = value.fence;
        if (!retained_recipe::Valid(fence.revision)
            || !retained_recipe::Nonzero(fence.base)
            || !retained_recipe::Nonzero(fence.result)
            || !retained_recipe::Nonzero(fence.units)
            || !retained_recipe::Nonzero(fence.selection)
            || !retained_recipe::Nonzero(fence.issuance)
            || !retained_recipe::Nonzero(fence.history))
            return Refusal::StaleFence;
        const retained_recipe::RecipeLocator locator{
            identity.outerOwner, identity.sourceNode, identity.original.sourceFeature};
        const auto read = std::find_if(fence.revision.dependencies.begin(),
            fence.revision.dependencies.end(), [&](const retained_recipe::DependencyRead& item) {
                return item.locator == locator;
            });
        if (read == fence.revision.dependencies.end()
            || read->geometry != value.effectiveCommitments.geometry
            || read->recipe != value.effectiveCommitments.recipe
            || read->placement != value.effectiveCommitments.placement
            || read->material != value.effectiveCommitments.material
            || read->groups != value.effectiveCommitments.groups
            || fence.base != value.recipe.effectiveBaseCommitment
            || fence.result != value.recipe.effectiveResultCommitment)
            return Refusal::StaleFence;
        return Refusal::None;
    } catch (...) { return Refusal::RuleNotInstalled; }
}

inline Decision EvaluateProgramOperand(const Request& request) noexcept {
    Decision result;
    try {
        if (!retained_recipe::Valid(request.outerOwner)
            || !retained_recipe::Nonzero(request.outputFeature)
            || !retained_recipe::Valid(request.issuance)) {
            result.refusal = Refusal::MissingSource; return result;
        }
        for (const Value& value : request.operands) {
            const Refusal refusal = InspectValues(value);
            if (refusal != Refusal::None) { result.refusal = refusal; return result; }
        }
        const Value& left = request.operands[0];
        const Value& right = request.operands[1];
        if (left.role != Role::Left || right.role != Role::Right
            || left.identity.sourceNode == right.identity.sourceNode
            || left.identity.sourceLocalID == right.identity.sourceLocalID
            || left.identity.shapeSlot == right.identity.shapeSlot
            || left.identity.original == right.identity.original
            || left.identity.derivedFeature == right.identity.derivedFeature
            || request.outputFeature == left.identity.outputFeature
            || request.outputFeature == right.identity.outputFeature)
            { result.refusal = Refusal::DuplicateIdentity; return result; }
        for (const Value& value : request.operands) {
            if (!(value.identity.outerOwner == request.outerOwner)
                || value.identity.original.document != request.outerOwner.document) {
                result.refusal = Refusal::OwnerAmbiguous; return result;
            }
            result.completeReadSet.push_back({value.identity.outerOwner,
                value.identity.sourceNode, value.identity.original.sourceFeature});
        }
        if (!part_boolean::family_admission::ProgramOperandNativeAdmissionEnabled) {
            result.refusal = Refusal::AdmissionDisabled; return result;
        }
        // P1 has no owner-minted execution receipt. This branch remains closed
        // even after a future isolated flag flip until the later owner rule is
        // installed in P9.
        result.refusal = Refusal::RuleNotInstalled;
        result.completeReadSet.clear();
        return result;
    } catch (...) {
        result.refusal = Refusal::RuleNotInstalled;
        result.completeReadSet.clear();
        return result;
    }
}

} // namespace core3d::part_boolean::program_operand
