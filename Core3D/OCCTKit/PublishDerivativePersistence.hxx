#ifndef Core3D_PublishDerivativePersistence_HeaderFile
#define Core3D_PublishDerivativePersistence_HeaderFile

#include "AttachmentRigDefinition.hxx"
#include "PublishDerivativeDefinition.hxx"

#include <cstdint>
#include <string>
#include <vector>

namespace core3d { namespace publish {

// G0 owns the concrete OCAF attribute/driver. F0 freezes the bounded payload
// and invalidation semantics so later tools cannot turn a derivative into master.
struct PersistedPublishContract {
    std::uint32_t schemaVersion = 1;
    PublishConfiguration configuration;
    DerivativeDescriptor derivative;
    std::vector<AttachmentDefinition> attachments;
    ValidationReport historicalValidation;
};

enum class InvalidationReason : std::uint8_t {
    Recipe, Placement, Units, MaterialImage, SelectedMembers, Attachments,
    Settings, ConstraintProfile, DependencyRemap, SourceDrift
};

inline void MarkDerivativeStale(
    PersistedPublishContract& contract, const InvalidationReason) noexcept {
    contract.derivative.status = DerivativeStatus::Stale;
    contract.historicalValidation.historicalObservation = true;
}

inline bool CanRestoreCachedDerivative(const PersistedPublishContract& contract,
    const std::string& recapturedSourceDigest,
    const std::string& recapturedConfigurationDigest,
    const std::string& recapturedAttachmentDigest) noexcept {
    return contract.derivative.status == DerivativeStatus::Current
        && !contract.historicalValidation.historicalObservation
        && contract.historicalValidation.sourceDigest == recapturedSourceDigest
        && contract.derivative.source.modelRevision == recapturedSourceDigest
        && contract.historicalValidation.settingsDigest == recapturedConfigurationDigest
        && !recapturedAttachmentDigest.empty();
}

}} // namespace core3d::publish

#endif
