#pragma once

#include "MikkTangentSpace.hpp"
#include <CommonCrypto/CommonDigest.h>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace core3d::scene::authored {

using GeometryIdentity = std::array<std::uint8_t, 32>;
// Private versioned storage. Identity is the digest of the owning geometry,
// supplied by the native owner; this codec alone does not establish ownership.
inline constexpr std::size_t kHeaderBytes = 48;
inline constexpr std::size_t kDigestBytes = 32;
inline constexpr std::size_t kMaximumFrames = kMaximumTangentTriangles * 3;
inline constexpr std::size_t kMaximumArchiveBytes = kHeaderBytes + kMaximumFrames * 16 + kDigestBytes;

enum class ArchiveStatus { Valid, InvalidLayout, InvalidFrame, ResourceLimit, IdentityMismatch, ChecksumMismatch, AllocationFailure };

inline std::uint32_t Read32(const std::uint8_t* p) noexcept {
    return std::uint32_t(p[0]) | (std::uint32_t(p[1]) << 8)
        | (std::uint32_t(p[2]) << 16) | (std::uint32_t(p[3]) << 24);
}
inline void Write32(std::uint8_t* p, std::uint32_t value) noexcept {
    for (unsigned i = 0; i < 4; ++i) p[i] = std::uint8_t(value >> (8 * i));
}
inline void WriteFloat(std::uint8_t* p, float value) noexcept {
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
    std::uint32_t bits; std::memcpy(&bits, &value, 4); Write32(p, bits);
}
inline float ReadFloat(const std::uint8_t* p) noexcept {
    const auto bits = Read32(p); float value; std::memcpy(&value, &bits, 4); return value;
}
inline bool ValidFrame(const Float4& t) noexcept {
    const double lengthSquared = double(t.x)*t.x + double(t.y)*t.y + double(t.z)*t.z;
    return std::isfinite(t.x) && std::isfinite(t.y) && std::isfinite(t.z)
        && std::abs(lengthSquared - 1.0) <= 1.0e-4 && (t.w == -1.0f || t.w == 1.0f);
}
inline bool Digest(const std::uint8_t* bytes, std::size_t size, GeometryIdentity& digest) noexcept {
    return size <= kMaximumArchiveBytes
        && CC_SHA256(bytes, static_cast<CC_LONG>(size), digest.data()) != nullptr;
}

inline ArchiveStatus Encode(const std::vector<Float4>& frames,
                            const GeometryIdentity& geometry,
                            std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    if (frames.empty() || frames.size() % 3) return ArchiveStatus::InvalidLayout;
    if (frames.size() > kMaximumFrames) return ArchiveStatus::ResourceLimit;
    for (std::size_t i = 0; i < frames.size(); ++i)
        if (!ValidFrame(frames[i]) || frames[i].w != frames[i - i % 3].w)
            return ArchiveStatus::InvalidFrame;
    try {
        std::vector<std::uint8_t> bytes(kHeaderBytes + frames.size() * 16 + kDigestBytes, 0);
        // SYTF, version1, corner count, reserved0, 32-byte geometry identity.
        bytes[0] = 'S'; bytes[1] = 'Y'; bytes[2] = 'T'; bytes[3] = 'F';
        Write32(bytes.data() + 4, 1); Write32(bytes.data() + 8, std::uint32_t(frames.size()));
        std::memcpy(bytes.data() + 16, geometry.data(), geometry.size());
        for (std::size_t i = 0; i < frames.size(); ++i) {
            auto* p = bytes.data() + kHeaderBytes + i * 16;
            const auto& t = frames[i];
            WriteFloat(p, t.x); WriteFloat(p + 4, t.y); WriteFloat(p + 8, t.z); WriteFloat(p + 12, t.w);
        }
        GeometryIdentity digest;
        if (!Digest(bytes.data(), bytes.size() - kDigestBytes, digest)) return ArchiveStatus::InvalidLayout;
        std::memcpy(bytes.data() + bytes.size() - kDigestBytes, digest.data(), digest.size());
        output.swap(bytes); return ArchiveStatus::Valid;
    } catch (...) { return ArchiveStatus::AllocationFailure; }
}

inline ArchiveStatus Decode(const std::uint8_t* bytes, std::size_t size,
                            const GeometryIdentity& expectedGeometry,
                            std::vector<Float4>& output) noexcept {
    output.clear();
    if (size > kMaximumArchiveBytes) return ArchiveStatus::ResourceLimit;
    if (!bytes || size < kHeaderBytes + 3 * 16 + kDigestBytes
        || std::memcmp(bytes, "SYTF", 4) || Read32(bytes + 4) != 1 || Read32(bytes + 12) != 0)
        return ArchiveStatus::InvalidLayout;
    const std::size_t count = Read32(bytes + 8);
    if (count > kMaximumFrames) return ArchiveStatus::ResourceLimit;
    if (!count || count % 3 || size != kHeaderBytes + count * 16 + kDigestBytes)
        return ArchiveStatus::InvalidLayout;
    if (std::memcmp(bytes + 16, expectedGeometry.data(), expectedGeometry.size()))
        return ArchiveStatus::IdentityMismatch;
    GeometryIdentity digest;
    if (!Digest(bytes, size - kDigestBytes, digest)
        || std::memcmp(digest.data(), bytes + size - kDigestBytes, kDigestBytes))
        return ArchiveStatus::ChecksumMismatch;
    // Verify frames before allocating their expanded array. This is not a
    // substitute for checking orthogonality against the owning mesh normals.
    for (std::size_t i = 0; i < count; ++i) {
        const auto* p = bytes + kHeaderBytes + i * 16;
        const Float4 t{ReadFloat(p), ReadFloat(p + 4), ReadFloat(p + 8), ReadFloat(p + 12)};
        const float triangleSign = ReadFloat(bytes + kHeaderBytes + (i - i % 3) * 16 + 12);
        if (!ValidFrame(t) || t.w != triangleSign) return ArchiveStatus::InvalidFrame;
    }
    try {
        std::vector<Float4> frames; frames.reserve(count);
        for (std::size_t i = 0; i < count; ++i) {
            const auto* p = bytes + kHeaderBytes + i * 16;
            frames.push_back({ReadFloat(p), ReadFloat(p + 4), ReadFloat(p + 8), ReadFloat(p + 12)});
        }
        output.swap(frames); return ArchiveStatus::Valid;
    } catch (...) { return ArchiveStatus::AllocationFailure; }
}
} // namespace core3d::scene::authored
