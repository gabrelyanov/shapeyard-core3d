#pragma once

#include "AttachmentRigMetadata.hxx"

namespace core3d::attachment_rig {

inline constexpr std::size_t kMaxJoints = 64;
inline constexpr std::size_t kMaxRequiredRoles = 64;
inline constexpr std::size_t kMaxDisplayNameUTF8Bytes = 256;
inline constexpr std::size_t kMaxSemanticReceiptBytes = 64U * 1024U;
inline constexpr std::size_t kMaxABR1Bytes = 1024U * 1024U;

// ABR1 persistence is a bounded, length-delimited, checksum-framed value.
// Decode is fail-closed: malformed/unknown schema remains unreadable rather
// than being interpreted as an empty rig.  Caller owns the OCAF transaction.
bool EncodeABR1(const Metadata&, std::vector<std::uint8_t>& bytes, std::string& refusal) noexcept;
bool DecodeABR1(const std::vector<std::uint8_t>& bytes, Metadata&, std::string& refusal) noexcept;

} // namespace core3d::attachment_rig
