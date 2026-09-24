#ifndef Core3D_AttachmentRigDefinition_HeaderFile
#define Core3D_AttachmentRigDefinition_HeaderFile

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace core3d { namespace publish {

enum class AttachmentBindingKind : std::uint8_t { RigidOwner, SurfaceFeatureReserved };

struct AttachmentDefinition {
    std::string id;
    std::string ownerLocator;
    std::string roleNamespaceVersion;
    std::string role;
    AttachmentBindingKind bindingKind = AttachmentBindingKind::RigidOwner;
    std::array<double, 3> localOriginMM = {{0, 0, 0}};
    std::array<double, 4> localRotationXYZW = {{0, 0, 0, 1}};
    std::string parentAttachmentID;
    std::string selector; // Must be empty for the initial rigid slice.
    std::string bindRevision;
};

constexpr std::size_t kMaximumAttachmentsPerPublishAsset = 64;
constexpr std::size_t kMaximumAttachmentHierarchyDepth = 16;
constexpr std::size_t kMaximumAttachmentRoleBytes = 128;
constexpr double kRigidFrameTolerance = 1e-9;

inline bool IsFiniteUnitQuaternion(const std::array<double, 4>& q) noexcept {
    double lengthSquared = 0.0;
    for (double component : q) {
        if (!std::isfinite(component)) return false;
        lengthSquared += component * component;
    }
    return std::isfinite(lengthSquared) && std::abs(lengthSquared - 1.0) <= kRigidFrameTolerance;
}

inline bool IsValidRigidAttachmentSet(const std::vector<AttachmentDefinition>& attachments) noexcept {
    if (attachments.size() > kMaximumAttachmentsPerPublishAsset) return false;
    std::unordered_map<std::string, const AttachmentDefinition*> byID;
    for (const AttachmentDefinition& attachment : attachments) {
        if (attachment.id.empty() || attachment.ownerLocator.empty() || attachment.roleNamespaceVersion.empty()
            || attachment.role.empty() || attachment.role.size() > kMaximumAttachmentRoleBytes
            || attachment.bindingKind != AttachmentBindingKind::RigidOwner || !attachment.selector.empty()
            || attachment.bindRevision.empty() || !IsFiniteUnitQuaternion(attachment.localRotationXYZW)
            || !byID.emplace(attachment.id, &attachment).second) return false;
        for (double coordinate : attachment.localOriginMM) if (!std::isfinite(coordinate)) return false;
    }
    for (const AttachmentDefinition& attachment : attachments) {
        std::unordered_set<std::string> ancestors;
        const AttachmentDefinition* current = &attachment;
        for (std::size_t depth = 0; !current->parentAttachmentID.empty(); ++depth) {
            if (depth >= kMaximumAttachmentHierarchyDepth || !ancestors.insert(current->id).second) return false;
            const auto parent = byID.find(current->parentAttachmentID);
            if (parent == byID.end()) return false;
            current = parent->second;
        }
    }
    return true;
}

}} // namespace core3d::publish

#endif
