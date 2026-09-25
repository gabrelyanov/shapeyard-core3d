#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

// F4 isolated value seam.  Persistence and owner mutation are deliberately
// supplied by the serial OcctDocument integrator; this header stores no OCCT
// topology ordinal and contains no skin influence data.
namespace core3d::attachment_rig {

inline constexpr std::uint32_t kSchemaVersion = 1;
inline constexpr std::size_t kMaxAttachments = 64;
inline constexpr std::size_t kMaxHierarchyDepth = 16;
inline constexpr std::size_t kMaxRoleUTF8Bytes = 128;
inline constexpr double kFrameTolerance = 1e-9;

struct FrameMM {
    std::array<double, 3> originMM{};
    std::array<double, 3> x{};
    std::array<double, 3> y{};
    std::array<double, 3> z{};
};

// `owner` is the frozen G0 OwnerKey/RecipeLocator encoding.  It is never a
// display name, TDF entry, or a reusable topology enumeration number.
struct RigidOwnerBinding { std::string owner; };

// `faceSelector` is exactly the B2 persisted proved face-membership selector
// receipt.  It carries semantic feature-local anchors and face-use proof; it
// must not encode TopExp, wire, face, edge, or triangle ordinals.
struct SurfaceBinding { std::string owner; std::string faceSelector; FrameMM localFrameMM; };

struct Attachment {
    std::string attachmentID;
    std::string displayName;
    std::string role;
    FrameMM frameMM;
    std::optional<RigidOwnerBinding> rigid;
    std::optional<SurfaceBinding> surface;
};

struct Joint {
    std::string jointID;
    std::string displayName;
    std::optional<std::string> parentJointID;
    FrameMM bindTransformMM;
};

struct TargetRoleProfile {
    std::uint32_t schemaVersion = kSchemaVersion;
    std::string profileID;
    std::string profileVersion;
    bool externallyVerified = false; // false is required until destination evidence is captured.
    std::vector<std::string> requiredRoles;
};

struct Metadata {
    std::uint32_t schemaVersion = kSchemaVersion;
    std::vector<Attachment> attachments;
    std::vector<Joint> joints;
    TargetRoleProfile targetProfile;
};

enum class ReattachOutcome : std::uint8_t { Current, Reattached, NeedsReview, Refused };
struct ReattachReceipt { std::string attachmentID; ReattachOutcome outcome; std::string reason; };

// Pure admission: finite right-handed orthonormal frames; unique stable IDs;
// exactly one binding; role/profile cardinality; ownership/remap and cycle
// fences.  It never produces vertex weights or silently updates a surface.
bool Validate(const Metadata&, std::string& refusal) noexcept;
std::vector<ReattachReceipt> ReattachSurfaceBindings(const Metadata&, std::string& refusal) noexcept;

} // namespace core3d::attachment_rig
