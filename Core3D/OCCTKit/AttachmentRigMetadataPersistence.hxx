#pragma once

#include "AttachmentRigMetadata.hxx"

#include <cstdint>
#include <string>
#include <vector>

class Standard_GUID;

namespace core3d::attachment_rig {

inline constexpr std::size_t kMaxJoints = 64;
inline constexpr std::size_t kMaxRequiredRoles = 64;
inline constexpr std::size_t kMaxDisplayNameUTF8Bytes = 256;
inline constexpr std::size_t kMaxSemanticReceiptBytes = 64U * 1024U;
inline constexpr std::size_t kMaxABR1Bytes = 1024U * 1024U;
//! D-F1's authoritative stored-form boundary. The detached S1 codec keeps
//! its ratified one-MiB validation ceiling; only durable adoption tightens it.
inline constexpr std::size_t kMaxStoredABR1Bytes = 65'536U;

//! One document-main-label slot. A present attribute that cannot be decoded
//! is Corrupt, never Absent and never an empty rig.
const Standard_GUID& AttachmentRigAttributeID() noexcept;

enum class StoredReadState : std::uint8_t {
    Absent = 0,
    Present,
    Corrupt,
};

struct StoredRecord final {
    StoredReadState state = StoredReadState::Absent;
    std::vector<std::uint8_t> bytes;
    std::string refusal;

    bool readable() const noexcept {
        return state == StoredReadState::Absent
            || (state == StoredReadState::Present && !bytes.empty());
    }
};

// ABR1 persistence is a bounded, length-delimited, checksum-framed value.
// Decode is fail-closed: malformed/unknown schema remains unreadable rather
// than being interpreted as an empty rig.  Caller owns the OCAF transaction.
bool EncodeABR1(const Metadata&, std::vector<std::uint8_t>& bytes, std::string& refusal) noexcept;
bool DecodeABR1(const std::vector<std::uint8_t>& bytes, Metadata&, std::string& refusal) noexcept;

} // namespace core3d::attachment_rig
