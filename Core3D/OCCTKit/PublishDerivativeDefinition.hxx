#ifndef Core3D_PublishDerivativeDefinition_HeaderFile
#define Core3D_PublishDerivativeDefinition_HeaderFile

// F0's durable, destination-neutral publish authority. This is deliberately
// a descriptor of a parametric source and a separately rebuildable derivative;
// it is never an editable replacement for the source recipe.

#include <cstdint>
#include <string>
#include <vector>

namespace core3d { namespace publish {

enum class SourceSelectionPolicy : std::uint8_t { SelectedOutputs, AllVisibleCaptured };
enum class DerivativeStatus : std::uint8_t { Unbuilt, Current, Stale, Failed };
enum class ValidationOutcome : std::uint8_t { Pass, Fail, Unverified, NotApplicable };
enum class PublishLifecycle : std::uint8_t { Ready, Cancelled, RecoveryPending, Failed };

struct VersionedDigest {
    std::string version;
    std::string sha256;
    bool IsPresent() const noexcept { return !version.empty() && sha256.size() == 64; }
};

struct OwnerSourceCommitment {
    std::string ownerKey; // Native owner identity, never a display name.
    std::string recipeDigest;
    std::string geometryDigest;
    std::string placementDigest;
    std::string materialTextureDigest;
};

struct PublishSourceBinding {
    std::string documentIdentifier;
    std::string generationIdentifier;
    std::string modelRevision;
    SourceSelectionPolicy selectionPolicy = SourceSelectionPolicy::SelectedOutputs;
    std::vector<OwnerSourceCommitment> owners; // Strictly sorted by ownerKey.
    std::string unitConvention;
    std::string frameConvention;
};

struct PublishConfiguration {
    std::string configurationIdentifier;
    std::string configurationVersion;
    SourceSelectionPolicy sourceSelectionPolicy = SourceSelectionPolicy::SelectedOutputs;
    std::string outputFormat;
    VersionedDigest meshing;
    VersionedDigest materialProfile;
    VersionedDigest constraintProfile;
    VersionedDigest derivativeSettings;
    VersionedDigest attachmentSet;
};

struct DerivativeDescriptor {
    std::string derivativeIdentifier;
    PublishSourceBinding source;
    VersionedDigest algorithm;
    std::vector<std::string> resourceSHA256;
    VersionedDigest dependencyRemap;
    std::string nativeArtifactIdentifier;
    DerivativeStatus status = DerivativeStatus::Unbuilt;
    // No admission or upload lease belongs in this persistent descriptor.
};

struct PreparedArtifact {
    std::string immutableResourceHandle;
    std::string sourceDigest;
    std::string configurationDigest;
    std::string derivativeDigest;
    std::string attachmentDigest;
    std::vector<std::string> emittedMembers;
    VersionedDigest formatProfile;
    std::uint64_t byteLength = 0;
    std::string artifactSHA256;
    std::string glbSHA256; // Separate named scopes: empty when not emitted.
    std::string binSHA256;
};

struct ValidationCheck {
    std::string identifier;
    ValidationOutcome outcome = ValidationOutcome::Unverified;
    std::string measuredValue;
    std::string measuredUnit;
    std::string requiredLimit; // Empty means no supplied destination limit.
    VersionedDigest method;
    std::string tolerance;
    std::string evidence;
    bool mandatory = true;
};

struct ValidationReport {
    std::string artifactSHA256;
    std::string sourceDigest;
    std::string settingsDigest;
    std::string profileDigest;
    std::vector<ValidationCheck> checks;
    PublishLifecycle lifecycle = PublishLifecycle::Ready;
    bool historicalObservation = false; // Reopen requires revalidation.

    bool IsPass() const noexcept {
        if (lifecycle != PublishLifecycle::Ready || historicalObservation) return false;
        for (const ValidationCheck& check : checks)
            if (check.mandatory && check.outcome != ValidationOutcome::Pass) return false;
        return !checks.empty();
    }
};

inline bool IsStrictlySortedOwners(const PublishSourceBinding& binding) noexcept {
    if (binding.owners.empty()) return false;
    for (std::size_t index = 0; index < binding.owners.size(); ++index) {
        const OwnerSourceCommitment& owner = binding.owners[index];
        if (owner.ownerKey.empty() || owner.recipeDigest.empty() || owner.geometryDigest.empty()
            || owner.placementDigest.empty() || owner.materialTextureDigest.empty()) return false;
        if (index != 0 && binding.owners[index - 1].ownerKey >= owner.ownerKey) return false;
    }
    return !binding.documentIdentifier.empty() && !binding.generationIdentifier.empty()
        && !binding.modelRevision.empty() && !binding.unitConvention.empty()
        && !binding.frameConvention.empty();
}

}} // namespace core3d::publish

#endif
