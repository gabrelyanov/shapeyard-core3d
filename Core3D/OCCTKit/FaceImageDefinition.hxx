#pragma once

// E3 semantic planar-face image/UV retained definitions (278b portion 1).
// Durable binding/resource/role/colour-space/UV-transform/wrap value types,
// the versioned SYFI/1 binding codec and the versioned SYFR/1 resource
// envelope codec with fail-closed bounded reads. Face identity is always the
// B2 semantic receipt (the FaceIntent identifier plus the current receipt
// proof fenced at capture); a tessellation ordinal is never durable face
// identity. Original imported bytes and normalized working bytes are distinct
// resources with separate hashes, media/dimensions and alpha interpretation;
// neither is derivable from the other here. This header is values and codecs
// only: no document registration, no replay and no UI seam. The SYFI/1 codec
// mirrors the SYEA/1 value seam in AssetAtlasDefinition.hxx and never widens
// it; the frozen FaceIntent v2/16-faces/4096-bytes contract is untouched.
#include "RetainedFinishingRecord.hxx"
#include <CommonCrypto/CommonDigest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace core3d::face_image {
using retained_recipe::Digest;
using retained_recipe::OwnerKey;
using retained_recipe::UUID;

inline constexpr char kSchemaVersion[] = "shapeyard.face-image.v1";
inline constexpr std::size_t kMaximumBytes = 8U * 1024U * 1024U;   // record cap
inline constexpr std::size_t kMaximumBindings = 256;               // per owner
inline constexpr std::size_t kMaximumEncodedImageBytes = 32U * 1024U * 1024U;
inline constexpr int kMaximumImageDimension = 8192;
inline constexpr std::size_t kMaximumImagePixels = 16777216;
inline constexpr std::size_t kMaximumDecodedImageBytes = 128U * 1024U * 1024U;
// One resource envelope is bounded by its two encoded images plus the fixed
// header; the trailing digest is accounted by the codec.
inline constexpr std::size_t kMaximumEnvelopeBytes = 2U * kMaximumEncodedImageBytes + 4096U;
// Aggregate original+working encoded bytes across every envelope of one
// document. Shared use, retained undo copies and serialized occurrences all
// charge against this one budget; there is no per-binding multiplication.
inline constexpr std::size_t kMaximumAggregateResourceBytes = 128U * 1024U * 1024U;
inline constexpr char kResourceSchemaVersion[] = "shapeyard.face-image-resource.v1";

enum class Role : std::uint8_t { BaseColor, Emissive, MetallicRoughness, Occlusion, Normal };
enum class ColorSpace : std::uint8_t { SRGB, Linear };   // SRGB iff role is BaseColor/Emissive
enum class Wrap : std::uint8_t { ClampToEdge, Repeat, MirroredRepeat };
enum class ImageEncoding : std::uint8_t { PNG = 1, JPEG = 2 };
enum class AlphaInterpretation : std::uint8_t { Opaque = 1, Straight = 2 };

struct UVTransform final {   // applied to the admitted planar face's chart UVs
    std::array<double, 2> scale{1.0, 1.0};    // finite, each in (0, 64]
    std::array<double, 2> offset{0.0, 0.0};   // finite, each in [-4096, 4096]
    double rotationDegrees = 0.0;             // finite, normalized to [0, 360)
    Wrap wrapU = Wrap::Repeat, wrapV = Wrap::Repeat;
};

inline bool SameScalarBits(double a, double b) noexcept {
    std::uint64_t x = 0, y = 0;
    static_assert(sizeof x == sizeof a, "Double bit comparison requires 64-bit doubles");
    std::memcpy(&x, &a, sizeof x); std::memcpy(&y, &b, sizeof y);
    return x == y;
}

struct Binding final {
    UUID binding{};            // durable, minted at commit
    UUID face{};               // B2 FaceIntent identifier (never an ordinal)
    Digest selectorProof{};    // current B2 receipt proof, fenced at capture
    UUID resource{};           // document-owned image resource identity
    Role role = Role::BaseColor;
    ColorSpace colorSpace = ColorSpace::SRGB;
    UVTransform transform;
    bool operator==(const Binding& value) const noexcept {
        return binding == value.binding && face == value.face
            && selectorProof == value.selectorProof && resource == value.resource
            && role == value.role && colorSpace == value.colorSpace
            && SameScalarBits(transform.scale[0], value.transform.scale[0])
            && SameScalarBits(transform.scale[1], value.transform.scale[1])
            && SameScalarBits(transform.offset[0], value.transform.offset[0])
            && SameScalarBits(transform.offset[1], value.transform.offset[1])
            && SameScalarBits(transform.rotationDegrees, value.transform.rotationDegrees)
            && transform.wrapU == value.transform.wrapU
            && transform.wrapV == value.transform.wrapV;
    }
};
struct Definition final {
    OwnerKey owner{};          // retained solid owner (row-266 binding)
    std::vector<Binding> bindings;
    Digest bindingProof{};
    bool operator==(const Definition& value) const noexcept {
        return owner == value.owner && bindings == value.bindings
            && bindingProof == value.bindingProof;
    }
};
enum class Refusal : std::uint8_t {
    None, Malformed, Oversized, StaleSource, StaleFace, AmbiguousFaceRemap,
    UnsupportedSurface, MissingResource, StaleResource, ForeignResource,
    UnsupportedDownstream, OwnerMismatch, TransactionBusy, PersistenceFailure
};

// Versioned document-owned image resource envelope. The original imported
// bytes are retained verbatim with their own content identity (the
// "texture-sha256-..." immutable identity); the normalized working bytes are
// a separate resource with their own hash. Provenance is role-independent:
// the same content may serve several roles without sharing interpretation.
struct ResourceEnvelope final {
    UUID resource{};                // document-owned identity; never a table index
    Digest originalContent{};       // sha256 of the verbatim imported bytes
    Digest workingContent{};        // sha256 of the normalized working bytes
    ImageEncoding originalFormat = ImageEncoding::PNG;
    ImageEncoding workingFormat = ImageEncoding::PNG;
    AlphaInterpretation alpha = AlphaInterpretation::Opaque;
    std::uint32_t originalWidthTexels = 0, originalHeightTexels = 0;
    std::uint32_t workingWidthTexels = 0, workingHeightTexels = 0;
    Digest provenance{};            // role-independent import provenance witness
    std::vector<std::uint8_t> originalBytes;  // verbatim import, bounded
    std::vector<std::uint8_t> workingBytes;   // normalized working bytes, bounded
    bool operator==(const ResourceEnvelope& value) const noexcept {
        return resource == value.resource && originalContent == value.originalContent
            && workingContent == value.workingContent
            && originalFormat == value.originalFormat && workingFormat == value.workingFormat
            && alpha == value.alpha
            && originalWidthTexels == value.originalWidthTexels
            && originalHeightTexels == value.originalHeightTexels
            && workingWidthTexels == value.workingWidthTexels
            && workingHeightTexels == value.workingHeightTexels
            && provenance == value.provenance
            && originalBytes == value.originalBytes && workingBytes == value.workingBytes;
    }
};

// Currentness fence captured at Prepare by the existing document authority.
// A face fence carries the current B2 receipt proof of one semantic face; a
// resource fence carries the immutable original-content identity of one
// document-owned resource. This seam never manufactures currentness itself.
struct FaceFence final {
    UUID face{};
    Digest selectorProof{};
    bool operator==(const FaceFence& value) const noexcept {
        return face == value.face && selectorProof == value.selectorProof;
    }
};
struct ResourceFence final {
    UUID resource{};
    Digest content{};
    bool operator==(const ResourceFence& value) const noexcept {
        return resource == value.resource && content == value.content;
    }
};
struct Observed final {
    std::vector<FaceFence> faces;
    std::vector<ResourceFence> resources;
    bool operator==(const Observed& value) const noexcept {
        return faces == value.faces && resources == value.resources;
    }
};

inline bool Nonzero(const Digest& value) noexcept { return retained_recipe::Nonzero(value); }

inline bool Valid(const UVTransform& value) noexcept {
    for (double scalar : value.scale)
        if (!std::isfinite(scalar) || scalar <= 0.0 || scalar > 64.0) return false;
    for (double scalar : value.offset)
        if (!std::isfinite(scalar) || scalar < -4096.0 || scalar > 4096.0) return false;
    if (!std::isfinite(value.rotationDegrees) || value.rotationDegrees < 0.0
        || value.rotationDegrees >= 360.0) return false;
    switch (value.wrapU) {
        case Wrap::ClampToEdge: case Wrap::Repeat: case Wrap::MirroredRepeat: break;
        default: return false;
    }
    switch (value.wrapV) {
        case Wrap::ClampToEdge: case Wrap::Repeat: case Wrap::MirroredRepeat: break;
        default: return false;
    }
    return true;
}

inline bool ValidColorSpaceForRole(Role role, ColorSpace colorSpace) noexcept {
    switch (role) {
        case Role::BaseColor: case Role::Emissive:
            return colorSpace == ColorSpace::SRGB;
        case Role::MetallicRoughness: case Role::Occlusion: case Role::Normal:
            return colorSpace == ColorSpace::Linear;
        default: return false;
    }
}

inline bool Valid(const Binding& value) noexcept {
    return retained_recipe::Nonzero(value.binding) && retained_recipe::Nonzero(value.face)
        && Nonzero(value.selectorProof) && retained_recipe::Nonzero(value.resource)
        && ValidColorSpaceForRole(value.role, value.colorSpace) && Valid(value.transform);
}

inline bool Valid(const Definition& value, Refusal& refusal) noexcept {
    refusal = Refusal::Malformed;
    try {
        if (!retained_recipe::Valid(value.owner) || value.bindings.empty()
            || value.bindings.size() > kMaximumBindings || !Nonzero(value.bindingProof))
            return false;
        for (std::size_t first = 0; first < value.bindings.size(); ++first) {
            const auto& binding = value.bindings[first];
            if (!Valid(binding)) return false;
            for (std::size_t other = first + 1; other < value.bindings.size(); ++other) {
                const auto& next = value.bindings[other];
                // One durable binding identity each; one binding per face/role
                // pair, so a later edit resolves exactly one record row.
                if (next.binding == binding.binding
                    || (next.face == binding.face && next.role == binding.role))
                    return false;
            }
        }
        refusal = Refusal::None;
        return true;
    } catch (...) { return false; }
}

// Currentness of a candidate against the captured fence: every bound face
// must still carry the identical current receipt proof and every bound
// resource must still be present in the fence. A face absent from the fence,
// or whose proof moved, is stale; a resource absent from the fence is
// missing. Resource content drift between Prepare and Commit is caught by
// the Transaction's exact captured-versus-observed comparison.
inline Refusal Current(const Definition& value, const Observed& observed) noexcept {
    try {
        for (const auto& binding : value.bindings) {
            bool faceFound = false;
            for (const auto& fence : observed.faces)
                if (fence.face == binding.face) {
                    faceFound = true;
                    if (!(fence.selectorProof == binding.selectorProof))
                        return Refusal::StaleFace;
                    break;
                }
            if (!faceFound) return Refusal::StaleFace;
            bool resourceFound = false;
            for (const auto& fence : observed.resources)
                if (fence.resource == binding.resource) { resourceFound = true; break; }
            if (!resourceFound) return Refusal::MissingResource;
        }
        return Refusal::None;
    } catch (...) { return Refusal::Malformed; }
}

namespace detail {
class Writer final {
public:
    std::vector<std::uint8_t> bytes;
    bool ok = true;
    void raw(const std::uint8_t* value, std::size_t count) {
        if (!ok || bytes.size() > kMaximumEnvelopeBytes
            || count > kMaximumEnvelopeBytes - bytes.size()) { ok = false; return; }
        bytes.insert(bytes.end(), value, value + count);
    }
    template<std::size_t N> void raw(const std::array<std::uint8_t, N>& value) { raw(value.data(), N); }
    void integer(std::uint64_t value, unsigned width) {
        std::uint8_t encoded[8]{};
        if (width == 0 || width > 8) { ok = false; return; }
        for (unsigned index = 0; index < width; ++index) encoded[index] = std::uint8_t(value >> (8 * index));
        raw(encoded, width);
    }
    void real(double value) {
        std::uint64_t bits = 0;
        static_assert(sizeof bits == sizeof value, "double width");
        std::memcpy(&bits, &value, sizeof bits);
        integer(bits, 8);
    }
};
class Reader final {
public:
    Reader(const std::vector<std::uint8_t>& value, std::size_t end) : bytes(value), limit(end) {}
    bool raw(std::uint8_t* output, std::size_t count) {
        if (!ok || cursor > limit || count > limit - cursor) { ok = false; return false; }
        std::copy_n(bytes.begin() + cursor, count, output); cursor += count; return true;
    }
    template<std::size_t N> bool raw(std::array<std::uint8_t, N>& output) { return raw(output.data(), N); }
    bool integer(unsigned width, std::uint64_t& output) {
        output = 0;
        if (!ok || width == 0 || width > 8 || cursor > limit || width > limit - cursor) { ok = false; return false; }
        for (unsigned index = 0; index < width; ++index) output |= std::uint64_t(bytes[cursor++]) << (8 * index);
        return true;
    }
    bool real(double& output) {
        std::uint64_t bits = 0;
        if (!integer(8, bits)) return false;
        std::memcpy(&output, &bits, sizeof output);
        return true;
    }
    bool complete() const noexcept { return ok && cursor == limit; }
private:
    const std::vector<std::uint8_t>& bytes; std::size_t limit = 0, cursor = 0; bool ok = true;
};

// Canonical SYFI/1 body writer shared by Encode and the binding-proof
// binding. The proof covers every field except bindingProof itself, which
// the caller zeros.
inline void EncodeBody(const Definition& value, Writer& writer) {
    writer.raw(reinterpret_cast<const std::uint8_t*>("SYFI\1\0\0\0"), 8);
    writer.raw(value.owner.document); writer.raw(value.owner.entity);
    writer.raw(value.owner.definition);
    writer.integer(0, 2);
    writer.integer(value.bindings.size(), 2);
    for (const auto& binding : value.bindings) {
        writer.raw(binding.binding); writer.raw(binding.face);
        writer.raw(binding.selectorProof); writer.raw(binding.resource);
        writer.integer(std::uint8_t(binding.role), 1);
        writer.integer(std::uint8_t(binding.colorSpace), 1);
        writer.integer(std::uint8_t(binding.transform.wrapU), 1);
        writer.integer(std::uint8_t(binding.transform.wrapV), 1);
        for (double scalar : binding.transform.scale) writer.real(scalar);
        for (double scalar : binding.transform.offset) writer.real(scalar);
        writer.real(binding.transform.rotationDegrees);
    }
    writer.raw(value.bindingProof);
}
} // namespace detail

// SHA-256 over this module's own bounded byte strings. retained_solid::Hash
// carries the 64 KiB retained-envelope bound and must not be used for
// record/envelope-scale bytes. Null, empty and over-bound input is refused
// with a zeroed digest.
static_assert(kMaximumEnvelopeBytes <= std::numeric_limits<CC_LONG>::max(),
              "face-image byte bound must fit CC_LONG");
inline bool HashFaceImageBytes(const std::uint8_t* bytes, std::size_t size,
                               Digest& output) noexcept {
    output = {};
    if (!bytes || size == 0 || size > kMaximumEnvelopeBytes) return false;
    return CC_SHA256(bytes, CC_LONG(size), output.data()) != nullptr;
}
inline bool HashFaceImageBytes(const std::vector<std::uint8_t>& bytes,
                               Digest& output) noexcept {
    return HashFaceImageBytes(bytes.data(), bytes.size(), output);
}

// Binds the exact committed binding set into the persisted record: the proof
// is SHA-256 over the canonical encoding of every Definition field except
// bindingProof itself.
inline bool BindBindingProof(Definition& value) noexcept {
    try {
        Definition covered = value;
        covered.bindingProof = Digest{};
        detail::Writer writer;
        detail::EncodeBody(covered, writer);
        if (!writer.ok || writer.bytes.size() > kMaximumBytes) return false;
        Digest proof{};
        if (!HashFaceImageBytes(writer.bytes, proof) || !Nonzero(proof)) return false;
        value.bindingProof = proof;
        return true;
    } catch (...) { return false; }
}

inline bool Encode(const Definition& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear(); Refusal refusal;
    try {
        if (!Valid(value, refusal)) return false;
        detail::Writer writer;
        detail::EncodeBody(value, writer);
        if (!writer.ok || writer.bytes.size() > kMaximumBytes - 32) return false;
        Digest digest{}; if (!HashFaceImageBytes(writer.bytes, digest)) return false;
        writer.raw(digest); if (!writer.ok) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Definition& output,
                   Refusal& refusal) noexcept {
    output = {}; refusal = bytes.size() > kMaximumBytes ? Refusal::Oversized : Refusal::Malformed;
    try {
        constexpr std::size_t fixed = 8 + 3 * 16 + 2 + 2 + 32 + 32;
        if (bytes.size() < fixed || bytes.size() > kMaximumBytes
            || std::memcmp(bytes.data(), "SYFI\1\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{}; if (!HashFaceImageBytes(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin()); if (actual != expected) return false;
        detail::Reader reader(bytes, bytes.size() - 32); std::array<std::uint8_t, 8> prefix{};
        Definition value; std::uint64_t reserved = 0, count = 0;
        if (!reader.raw(prefix) || !reader.raw(value.owner.document)
            || !reader.raw(value.owner.entity) || !reader.raw(value.owner.definition)
            || !reader.integer(2, reserved) || reserved != 0
            || !reader.integer(2, count) || count == 0 || count > kMaximumBindings)
            return false;
        value.bindings.resize(std::size_t(count));
        for (auto& binding : value.bindings) {
            std::uint64_t role = 0, colorSpace = 0, wrapU = 0, wrapV = 0;
            if (!reader.raw(binding.binding) || !reader.raw(binding.face)
                || !reader.raw(binding.selectorProof) || !reader.raw(binding.resource)
                || !reader.integer(1, role) || !reader.integer(1, colorSpace)
                || !reader.integer(1, wrapU) || !reader.integer(1, wrapV))
                return false;
            binding.role = Role(role); binding.colorSpace = ColorSpace(colorSpace);
            binding.transform.wrapU = Wrap(wrapU); binding.transform.wrapV = Wrap(wrapV);
            for (double& scalar : binding.transform.scale)
                if (!reader.real(scalar)) return false;
            for (double& scalar : binding.transform.offset)
                if (!reader.real(scalar)) return false;
            if (!reader.real(binding.transform.rotationDegrees)) return false;
        }
        if (!reader.raw(value.bindingProof)) return false;
        std::vector<std::uint8_t> canonical; Refusal validation;
        if (!reader.complete() || !Valid(value, validation) || !Encode(value, canonical)
            || canonical != bytes) return false;
        output = std::move(value); refusal = Refusal::None; return true;
    } catch (...) { output = {}; return false; }
}

inline bool Valid(const ResourceEnvelope& value, Refusal& refusal) noexcept {
    refusal = Refusal::Malformed;
    try {
        if (!retained_recipe::Nonzero(value.resource) || !Nonzero(value.originalContent)
            || !Nonzero(value.workingContent) || !Nonzero(value.provenance))
            return false;
        switch (value.originalFormat) {
            case ImageEncoding::PNG: case ImageEncoding::JPEG: break;
            default: return false;
        }
        switch (value.workingFormat) {
            case ImageEncoding::PNG: case ImageEncoding::JPEG: break;
            default: return false;
        }
        switch (value.alpha) {
            case AlphaInterpretation::Opaque: case AlphaInterpretation::Straight: break;
            default: return false;
        }
        const std::uint64_t dimensions[2][2] = {
            {value.originalWidthTexels, value.originalHeightTexels},
            {value.workingWidthTexels, value.workingHeightTexels}};
        for (const auto& pair : dimensions) {
            if (pair[0] == 0 || pair[1] == 0 || pair[0] > std::uint64_t(kMaximumImageDimension)
                || pair[1] > std::uint64_t(kMaximumImageDimension)
                || pair[0] * pair[1] > std::uint64_t(kMaximumImagePixels)
                || pair[0] * pair[1] * 4 > std::uint64_t(kMaximumDecodedImageBytes))
                return false;
        }
        if (value.originalBytes.empty() || value.originalBytes.size() > kMaximumEncodedImageBytes
            || value.workingBytes.empty() || value.workingBytes.size() > kMaximumEncodedImageBytes)
            { refusal = Refusal::Oversized; return false; }
        // The stored identities must be the actual hashes of the carried
        // bytes; a forged or mislabeled envelope never validates.
        Digest original{}, working{};
        if (!HashFaceImageBytes(value.originalBytes, original)
            || !HashFaceImageBytes(value.workingBytes, working)
            || original != value.originalContent || working != value.workingContent)
            return false;
        refusal = Refusal::None;
        return true;
    } catch (...) { return false; }
}

inline bool Encode(const ResourceEnvelope& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear(); Refusal refusal;
    try {
        if (!Valid(value, refusal)) return false;
        detail::Writer writer;
        writer.raw(reinterpret_cast<const std::uint8_t*>("SYFR\1\0\0\0"), 8);
        writer.raw(value.resource);
        writer.raw(value.originalContent); writer.raw(value.workingContent);
        writer.integer(std::uint8_t(value.originalFormat), 1);
        writer.integer(std::uint8_t(value.workingFormat), 1);
        writer.integer(std::uint8_t(value.alpha), 1);
        writer.integer(0, 1);
        writer.integer(value.originalWidthTexels, 4);
        writer.integer(value.originalHeightTexels, 4);
        writer.integer(value.workingWidthTexels, 4);
        writer.integer(value.workingHeightTexels, 4);
        writer.raw(value.provenance);
        writer.integer(value.originalBytes.size(), 4);
        writer.raw(value.originalBytes.data(), value.originalBytes.size());
        writer.integer(value.workingBytes.size(), 4);
        writer.raw(value.workingBytes.data(), value.workingBytes.size());
        if (!writer.ok || writer.bytes.size() > kMaximumEnvelopeBytes - 32) return false;
        Digest digest{}; if (!HashFaceImageBytes(writer.bytes, digest)) return false;
        writer.raw(digest); if (!writer.ok) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, ResourceEnvelope& output,
                   Refusal& refusal) noexcept {
    output = {};
    refusal = bytes.size() > kMaximumEnvelopeBytes ? Refusal::Oversized : Refusal::Malformed;
    try {
        constexpr std::size_t fixed = 8 + 16 + 2 * 32 + 4 + 4 * 4 + 32 + 2 * 4 + 32;
        if (bytes.size() < fixed || bytes.size() > kMaximumEnvelopeBytes
            || std::memcmp(bytes.data(), "SYFR\1\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{}; if (!HashFaceImageBytes(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin()); if (actual != expected) return false;
        detail::Reader reader(bytes, bytes.size() - 32); std::array<std::uint8_t, 8> prefix{};
        ResourceEnvelope value; std::uint64_t originalFormat = 0, workingFormat = 0, alpha = 0;
        std::uint64_t reserved = 0, originalCount = 0, workingCount = 0;
        std::uint64_t width = 0, height = 0;
        if (!reader.raw(prefix) || !reader.raw(value.resource)
            || !reader.raw(value.originalContent) || !reader.raw(value.workingContent)
            || !reader.integer(1, originalFormat) || !reader.integer(1, workingFormat)
            || !reader.integer(1, alpha) || !reader.integer(1, reserved) || reserved != 0)
            return false;
        value.originalFormat = ImageEncoding(originalFormat);
        value.workingFormat = ImageEncoding(workingFormat);
        value.alpha = AlphaInterpretation(alpha);
        if (!reader.integer(4, width) || width > std::uint64_t(kMaximumImageDimension)
            || !reader.integer(4, height) || height > std::uint64_t(kMaximumImageDimension))
            return false;
        value.originalWidthTexels = std::uint32_t(width);
        value.originalHeightTexels = std::uint32_t(height);
        if (!reader.integer(4, width) || width > std::uint64_t(kMaximumImageDimension)
            || !reader.integer(4, height) || height > std::uint64_t(kMaximumImageDimension))
            return false;
        value.workingWidthTexels = std::uint32_t(width);
        value.workingHeightTexels = std::uint32_t(height);
        if (!reader.raw(value.provenance)
            || !reader.integer(4, originalCount) || originalCount == 0
            || originalCount > kMaximumEncodedImageBytes) return false;
        value.originalBytes.resize(std::size_t(originalCount));
        if (!reader.raw(value.originalBytes.data(), value.originalBytes.size())
            || !reader.integer(4, workingCount) || workingCount == 0
            || workingCount > kMaximumEncodedImageBytes) return false;
        value.workingBytes.resize(std::size_t(workingCount));
        if (!reader.raw(value.workingBytes.data(), value.workingBytes.size()))
            return false;
        std::vector<std::uint8_t> canonical; Refusal validation;
        if (!reader.complete() || !Valid(value, validation) || !Encode(value, canonical)
            || canonical != bytes) return false;
        output = std::move(value); refusal = Refusal::None; return true;
    } catch (...) { output = {}; return false; }
}

// Prepare/Commit/Cancel staging, mirroring the SYEF Transaction. Prepare
// resolves and fences only; Commit re-proves the identical fence and never
// publishes a stale or drifted capture; Cancel discards the staging whole.
struct Transaction final {
    OwnerKey owner;
    Observed captured;
    bool prepared = false;

    Refusal Prepare(const Definition& candidate, const OwnerKey& resolvedOwner,
                    const Observed& observed) noexcept {
        Refusal refusal;
        if (prepared) return Refusal::TransactionBusy;
        if (!Valid(candidate, refusal)) return refusal;
        if (!(candidate.owner == resolvedOwner)) return Refusal::OwnerMismatch;
        refusal = Current(candidate, observed);
        if (refusal != Refusal::None) return refusal;
        owner = resolvedOwner; captured = observed; prepared = true; return Refusal::None;
    }
    Refusal Commit(const Definition& candidate, const OwnerKey& resolvedOwner,
                   const Observed& observed) noexcept {
        if (!prepared) return Refusal::Malformed;
        prepared = false;
        if (!(owner == resolvedOwner) || !(candidate.owner == resolvedOwner))
            return Refusal::OwnerMismatch;
        const Refusal currentness = Current(candidate, observed);
        if (currentness != Refusal::None) return currentness;
        if (!(captured == observed)) return Refusal::StaleSource;
        return Refusal::None;
    }
    void Cancel() noexcept { prepared = false; }
};

} // namespace core3d::face_image
