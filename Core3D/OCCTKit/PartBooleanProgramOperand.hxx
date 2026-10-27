#pragma once
// Value-only capture contract for using a complete retained cut program as a
// PartBoolean operand. These values contain no label, document handle, native
// lease, geometry candidate, command authority, or caller-controlled enable.
#include "PartBooleanDefinition.hxx"
#include "RetainedBooleanProgram.hxx"
#include "RetainedRecipeSnapshot.hxx"
#include <array>
#include <cstdint>
#include <vector>

namespace core3d::part_boolean::program_operand {

enum class Role : std::uint8_t { Left = 1, Right = 2 };
enum class Carrier : std::uint8_t {
    CanonicalRetainedProgram = 1,
    CompositeProgramSuffix = 2,
    DependentRoundOnly = 3,
};
enum class BaseFamily : std::uint8_t {
    Profile = 1,
    Enclosure = 2,
    RectangularLoft = 3,
    ProfileSchema5Shell = 4,
};

// The closed list is deliberately wider than P1. Later replay/owner portions
// refine these same outcomes instead of replacing them with strings or a
// catch-all success/failure result.
enum class Refusal : std::uint8_t {
    None = 0,
    MissingSource,
    ProgramMalformed,
    ProgramOversized,
    ProgramVersionMismatch,
    ProgramDigestMismatch,
    DuplicateIdentity,
    UnsupportedProgramCarrier,
    UnsupportedBaseFamily,
    UnsupportedDependentFeature,
    UnsupportedPlacement,
    OwnerAmbiguous,
    BaseFeatureMismatch,
    StaleFence,
    AdmissionDisabled,
    RuleNotInstalled,
    BaseMismatch,
    ProgramReplayMismatch,
    UnprovenBoundary,
    AmbiguousSelector,
    MaterialConflict,
    MissingMaterial,
    EmptyResult,
    TangentOnly,
    Disconnected,
    NoInteriorOverlap,
    BuildFailed,
    Cancelled,
    Budget,
};

struct IdentityValues final {
    retained_recipe::OwnerKey outerOwner;
    retained_recipe::SourceIdentity original;
    retained_recipe::UUID derivedFeature{};
    retained_recipe::UUID sourceNode{};
    retained_recipe::UUID outputFeature{};
    std::uint64_t sourceLocalID = 0;
    std::uint32_t shapeSlot = 0;
    std::uint64_t nextLocalID = 0;
};

struct RecipeValues final {
    BaseFamily baseFamily = BaseFamily::Profile;
    std::vector<std::uint8_t> originalBytes;
    retained_recipe::Digest originalDigest{};
    retained_boolean::Recipe originalDecoded;
    std::vector<std::uint8_t> effectiveBytes;
    retained_recipe::Digest effectiveDigest{};
    retained_boolean::Recipe effectiveDecoded;
    retained_recipe::Digest originalBaseCommitment{};
    retained_recipe::Digest originalResultCommitment{};
    retained_recipe::Digest effectiveBaseCommitment{};
    retained_recipe::Digest effectiveResultCommitment{};
};

struct AppearanceValues final {
    part_boolean::CompleteMetadata original;
    part_boolean::CompleteMetadata effective;
    std::vector<part_boolean::ScalarMaterialRow> originalMaterials;
    std::vector<part_boolean::ScalarMaterialRow> effectiveMaterials;
    std::vector<part_boolean::GroupRow> originalGroups;
    std::vector<part_boolean::GroupRow> effectiveGroups;
    std::vector<part_boolean::LayerRow> originalLayers;
    std::vector<part_boolean::LayerRow> effectiveLayers;
    std::vector<part_boolean::RegionBinding> regions;
    std::vector<part_boolean::SelectorBinding> selectors;
    part_boolean::MaterialPolicy materialPolicy =
        part_boolean::MaterialPolicy::RejectConflictingMerges;
    part_boolean::CodecVersions versions;
    bool originallyVisible = true;
    bool originallyConsumed = false;
};

struct FenceValues final {
    retained_recipe::RevisionFence revision;
    retained_recipe::Digest base{}, result{}, units{}, selection{}, issuance{}, history{};
};

struct Value final {
    Role role = Role::Left;
    Carrier carrier = Carrier::CanonicalRetainedProgram;
    IdentityValues identity;
    RecipeValues recipe;
    composite_recipe::InputPlacement sourceToCarrier;
    double observedOwnerMetersPerUnit = 0;
    double observedDocumentMetersPerUnit = 0;
    part_boolean::SignatureCommitments originalCommitments;
    part_boolean::SignatureCommitments effectiveCommitments;
    AppearanceValues appearance;
    FenceValues fence;
};

struct Request final {
    part_boolean::Operation operation = part_boolean::Operation::Union;
    retained_recipe::OwnerKey outerOwner;
    retained_recipe::UUID outputFeature{};
    retained_recipe::IssuanceState issuance;
    std::array<Value, 2> operands;
};

struct Decision final {
    Refusal refusal = Refusal::RuleNotInstalled;
    std::vector<retained_recipe::RecipeLocator> completeReadSet;
    bool admitted() const noexcept { return refusal == Refusal::None; }
};

} // namespace core3d::part_boolean::program_operand
