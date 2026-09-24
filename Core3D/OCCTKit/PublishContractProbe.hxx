#ifndef Core3D_PublishContractProbe_HeaderFile
#define Core3D_PublishContractProbe_HeaderFile

#include "AttachmentRigDefinition.hxx"
#include "PublishDerivativeDefinition.hxx"

namespace core3d { namespace publish {

inline bool IsPublishContractReady(const PublishSourceBinding& source,
    const PublishConfiguration& configuration,
    const DerivativeDescriptor& derivative,
    const std::vector<AttachmentDefinition>& attachments) noexcept {
    return IsStrictlySortedOwners(source)
        && !configuration.configurationIdentifier.empty()
        && !configuration.configurationVersion.empty()
        && configuration.meshing.IsPresent()
        && configuration.materialProfile.IsPresent()
        && configuration.constraintProfile.IsPresent()
        && configuration.derivativeSettings.IsPresent()
        && configuration.attachmentSet.IsPresent()
        && !derivative.derivativeIdentifier.empty()
        && IsValidRigidAttachmentSet(attachments);
}

inline bool MayInstallPreparedArtifact(const PreparedArtifact& artifact,
    const ValidationReport& validation) noexcept {
    return !artifact.immutableResourceHandle.empty() && artifact.byteLength > 0
        && artifact.artifactSHA256.size() == 64
        && artifact.artifactSHA256 == validation.artifactSHA256
        && validation.IsPass();
}

}} // namespace core3d::publish

#endif
