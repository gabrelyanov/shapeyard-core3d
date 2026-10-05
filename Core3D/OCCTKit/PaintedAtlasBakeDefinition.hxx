#pragma once

// E2b explicit painted-atlas preservation record.  SYEB/1 is independent
// from SYEA/1 and SYFI/1: it fences those records and names regenerable baked
// resources, but never replaces atlas intent, face-image bindings, original
// image bytes, or retained source recipes.
#include "AssetAtlasDefinition.hxx"
#include "FaceImagePersistence.hxx"

#include <TDataStd_AsciiString.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_UAttribute.hxx>
#include <TDF_AttributeIterator.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDocStd_Document.hxx>
#include <CommonCrypto/CommonDigest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

Standard_Boolean Core3DValidatePaintedAtlasBakeDocument(
    const Handle(TDocStd_Document)& document);

namespace core3d::painted_atlas_bake {
using retained_recipe::Digest;
using retained_recipe::OwnerKey;
using retained_recipe::UUID;
using face_image::ColorSpace;
using face_image::Role;
using face_image::UVTransform;
using face_image::Wrap;

inline constexpr char kSchemaVersion[] = "shapeyard.painted-atlas-bake.v1";
inline constexpr std::size_t kMaximumBytes = 8U * 1024U * 1024U;
inline constexpr std::size_t kMaximumBindings = asset_atlas::kMaximumMembers
    * face_image::kMaximumBindings;
inline constexpr std::size_t kMaximumBakedResources = 5;
inline constexpr int kMaximumBakedDimension = 4096;
inline constexpr std::size_t kMaximumEncodedImageBytes = 32U * 1024U * 1024U;
inline constexpr std::size_t kMaximumDecodedImageBytes = 128U * 1024U * 1024U;

struct BindingFence final {
    OwnerKey owner{};
    UUID binding{};
    UUID resource{};
    Digest selectorProof{};
    Digest bindingProof{};
    Digest originalContent{};
    Digest workingContent{};
    Role role = Role::BaseColor;
    ColorSpace colorSpace = ColorSpace::SRGB;
    UVTransform transform;
    bool operator==(const BindingFence& value) const noexcept {
        return owner == value.owner && binding == value.binding
            && resource == value.resource && selectorProof == value.selectorProof
            && bindingProof == value.bindingProof
            && originalContent == value.originalContent
            && workingContent == value.workingContent && role == value.role
            && colorSpace == value.colorSpace
            && face_image::SameScalarBits(transform.scale[0], value.transform.scale[0])
            && face_image::SameScalarBits(transform.scale[1], value.transform.scale[1])
            && face_image::SameScalarBits(transform.offset[0], value.transform.offset[0])
            && face_image::SameScalarBits(transform.offset[1], value.transform.offset[1])
            && face_image::SameScalarBits(transform.rotationDegrees,
                                          value.transform.rotationDegrees)
            && transform.wrapU == value.transform.wrapU
            && transform.wrapV == value.transform.wrapV;
    }
};

struct BakedResource final {
    Role role = Role::BaseColor;
    ColorSpace colorSpace = ColorSpace::SRGB;
    Digest identity{};       // deterministic role-bound resource identity
    Digest content{};        // deterministic PNG bytes
    std::uint32_t widthTexels = 0;
    std::uint32_t heightTexels = 0;
    Wrap wrapU = Wrap::ClampToEdge;
    Wrap wrapV = Wrap::ClampToEdge;
    bool operator==(const BakedResource& value) const noexcept {
        return role == value.role && colorSpace == value.colorSpace
            && identity == value.identity && content == value.content
            && widthTexels == value.widthTexels
            && heightTexels == value.heightTexels
            && wrapU == value.wrapU && wrapV == value.wrapV;
    }
};

// Frozen acceptance values.  Persisting them makes a future reader reject a
// record produced under weaker limits instead of silently adopting it.
struct OracleSnapshot final {
    std::uint8_t maximumInteriorError255 = 2;
    std::uint8_t maximumMeanError255 = 1;
    double normalLengthTolerance = 1.e-5;
    double maximumDensityRatio = 1.00001;
    double minimumCylinderStretch = 0.999;
    double maximumCylinderStretch = 1.001;
    double minimumFixtureOccupancy = 0.50;
    bool operator==(const OracleSnapshot& value) const noexcept {
        return maximumInteriorError255 == value.maximumInteriorError255
            && maximumMeanError255 == value.maximumMeanError255
            && face_image::SameScalarBits(normalLengthTolerance,
                                          value.normalLengthTolerance)
            && face_image::SameScalarBits(maximumDensityRatio,
                                          value.maximumDensityRatio)
            && face_image::SameScalarBits(minimumCylinderStretch,
                                          value.minimumCylinderStretch)
            && face_image::SameScalarBits(maximumCylinderStretch,
                                          value.maximumCylinderStretch)
            && face_image::SameScalarBits(minimumFixtureOccupancy,
                                          value.minimumFixtureOccupancy);
    }
};

struct Definition final {
    asset_atlas::Key key{};
    Digest layoutProof{};
    std::vector<BindingFence> bindings;
    std::vector<BakedResource> resources;
    OracleSnapshot oracle;
    std::uint32_t algorithmVersion = 1;
    std::uint32_t encoderVersion = 1;
    Digest bakeProof{};
    bool operator==(const Definition& value) const noexcept {
        return key == value.key && layoutProof == value.layoutProof
            && bindings == value.bindings && resources == value.resources
            && oracle == value.oracle && algorithmVersion == value.algorithmVersion
            && encoderVersion == value.encoderVersion
            && bakeProof == value.bakeProof;
    }
};

enum class Refusal : std::uint8_t {
    None, Malformed, Oversized, StaleSource, StaleBinding, MissingResource,
    ForeignResource, OverBudget, UnsupportedSurface, OwnerMismatch, Busy,
    PersistenceFailure, Absent
};

inline bool Nonzero(const Digest& value) noexcept {
    return retained_recipe::Nonzero(value);
}

inline bool Frozen(const OracleSnapshot& value) noexcept {
    return value.maximumInteriorError255 == 2
        && value.maximumMeanError255 == 1
        && face_image::SameScalarBits(value.normalLengthTolerance, 1.e-5)
        && face_image::SameScalarBits(value.maximumDensityRatio, 1.00001)
        && face_image::SameScalarBits(value.minimumCylinderStretch, 0.999)
        && face_image::SameScalarBits(value.maximumCylinderStretch, 1.001)
        && face_image::SameScalarBits(value.minimumFixtureOccupancy, 0.50);
}

inline bool Valid(const BindingFence& value) noexcept {
    return retained_recipe::Valid(value.owner)
        && retained_recipe::Nonzero(value.binding)
        && retained_recipe::Nonzero(value.resource)
        && Nonzero(value.selectorProof) && Nonzero(value.bindingProof)
        && Nonzero(value.originalContent) && Nonzero(value.workingContent)
        && face_image::ValidColorSpaceForRole(value.role, value.colorSpace)
        && face_image::Valid(value.transform);
}

inline bool Valid(const BakedResource& value) noexcept {
    return face_image::ValidColorSpaceForRole(value.role, value.colorSpace)
        && Nonzero(value.identity) && Nonzero(value.content)
        && value.widthTexels > 0 && value.heightTexels > 0
        && value.widthTexels <= kMaximumBakedDimension
        && value.heightTexels <= kMaximumBakedDimension
        && std::uint64_t(value.widthTexels) * value.heightTexels * 4
            <= kMaximumDecodedImageBytes
        && value.wrapU == Wrap::ClampToEdge
        && value.wrapV == Wrap::ClampToEdge;
}

inline bool Valid(const Definition& value, Refusal& refusal) noexcept {
    refusal = Refusal::Malformed;
    try {
        if (!retained_recipe::Nonzero(value.key.document)
            || !retained_recipe::Nonzero(value.key.atlas)
            || !Nonzero(value.layoutProof) || !Nonzero(value.bakeProof)
            || value.bindings.empty() || value.bindings.size() > kMaximumBindings
            || value.resources.empty()
            || value.resources.size() > kMaximumBakedResources
            || value.algorithmVersion != 1 || value.encoderVersion != 1
            || !Frozen(value.oracle)) return false;
        for (std::size_t index = 0; index < value.bindings.size(); ++index) {
            if (!Valid(value.bindings[index])) return false;
            for (std::size_t other = index + 1; other < value.bindings.size(); ++other)
                if (value.bindings[index].owner == value.bindings[other].owner
                    && value.bindings[index].binding == value.bindings[other].binding)
                    return false;
        }
        std::uint64_t decoded = 0;
        for (std::size_t index = 0; index < value.resources.size(); ++index) {
            if (!Valid(value.resources[index])) return false;
            decoded += std::uint64_t(value.resources[index].widthTexels)
                * value.resources[index].heightTexels * 4;
            if (decoded > kMaximumDecodedImageBytes) {
                refusal = Refusal::OverBudget; return false;
            }
            for (std::size_t other = index + 1; other < value.resources.size(); ++other)
                if (value.resources[index].role == value.resources[other].role
                    || value.resources[index].identity == value.resources[other].identity)
                    return false;
        }
        refusal = Refusal::None;
        return true;
    } catch (...) { return false; }
}

namespace detail {
class Writer final {
public:
    std::vector<std::uint8_t> bytes;
    bool ok = true;
    void raw(const std::uint8_t* value, std::size_t count) {
        if (!ok || !value || bytes.size() > kMaximumBytes
            || count > kMaximumBytes - bytes.size()) { ok = false; return; }
        bytes.insert(bytes.end(), value, value + count);
    }
    template<std::size_t N> void raw(const std::array<std::uint8_t, N>& value) {
        raw(value.data(), N);
    }
    void integer(std::uint64_t value, unsigned width) {
        std::uint8_t encoded[8]{};
        if (width == 0 || width > 8) { ok = false; return; }
        for (unsigned index = 0; index < width; ++index)
            encoded[index] = std::uint8_t(value >> (8 * index));
        raw(encoded, width);
    }
    void real(double value) {
        std::uint64_t bits = 0; std::memcpy(&bits, &value, sizeof bits); integer(bits, 8);
    }
};

class Reader final {
public:
    Reader(const std::vector<std::uint8_t>& value, std::size_t end)
        : bytes(value), limit(end) {}
    bool raw(std::uint8_t* output, std::size_t count) {
        if (!ok || !output || cursor > limit || count > limit - cursor) {
            ok = false; return false;
        }
        std::copy_n(bytes.begin() + cursor, count, output); cursor += count; return true;
    }
    template<std::size_t N> bool raw(std::array<std::uint8_t, N>& output) {
        return raw(output.data(), N);
    }
    bool integer(unsigned width, std::uint64_t& output) {
        output = 0;
        if (!ok || width == 0 || width > 8 || cursor > limit || width > limit - cursor) {
            ok = false; return false;
        }
        for (unsigned index = 0; index < width; ++index)
            output |= std::uint64_t(bytes[cursor++]) << (8 * index);
        return true;
    }
    bool real(double& output) {
        std::uint64_t bits = 0;
        if (!integer(8, bits)) return false;
        std::memcpy(&output, &bits, sizeof output); return true;
    }
    bool complete() const noexcept { return ok && cursor == limit; }
private:
    const std::vector<std::uint8_t>& bytes;
    std::size_t limit = 0, cursor = 0;
    bool ok = true;
};

inline void EncodeBody(const Definition& value, Writer& writer) {
    writer.raw(reinterpret_cast<const std::uint8_t*>("SYEB\1\0\0\0"), 8);
    writer.raw(value.key.document); writer.raw(value.key.atlas);
    writer.raw(value.layoutProof);
    writer.integer(value.algorithmVersion, 4); writer.integer(value.encoderVersion, 4);
    writer.integer(value.bindings.size(), 4); writer.integer(value.resources.size(), 1);
    writer.integer(value.oracle.maximumInteriorError255, 1);
    writer.integer(value.oracle.maximumMeanError255, 1); writer.integer(0, 1);
    writer.real(value.oracle.normalLengthTolerance);
    writer.real(value.oracle.maximumDensityRatio);
    writer.real(value.oracle.minimumCylinderStretch);
    writer.real(value.oracle.maximumCylinderStretch);
    writer.real(value.oracle.minimumFixtureOccupancy);
    for (const auto& binding : value.bindings) {
        writer.raw(binding.owner.document); writer.raw(binding.owner.entity);
        writer.raw(binding.owner.definition); writer.raw(binding.binding);
        writer.raw(binding.resource); writer.raw(binding.selectorProof);
        writer.raw(binding.bindingProof); writer.raw(binding.originalContent);
        writer.raw(binding.workingContent);
        writer.integer(std::uint8_t(binding.role), 1);
        writer.integer(std::uint8_t(binding.colorSpace), 1);
        writer.integer(std::uint8_t(binding.transform.wrapU), 1);
        writer.integer(std::uint8_t(binding.transform.wrapV), 1);
        for (double scalar : binding.transform.scale) writer.real(scalar);
        for (double scalar : binding.transform.offset) writer.real(scalar);
        writer.real(binding.transform.rotationDegrees);
    }
    for (const auto& resource : value.resources) {
        writer.integer(std::uint8_t(resource.role), 1);
        writer.integer(std::uint8_t(resource.colorSpace), 1);
        writer.integer(std::uint8_t(resource.wrapU), 1);
        writer.integer(std::uint8_t(resource.wrapV), 1);
        writer.integer(resource.widthTexels, 4); writer.integer(resource.heightTexels, 4);
        writer.raw(resource.identity); writer.raw(resource.content);
    }
    writer.raw(value.bakeProof);
}
} // namespace detail

inline bool Hash(const std::uint8_t* bytes, std::size_t size, Digest& output) noexcept {
    output = {};
    return bytes && size > 0 && size <= kMaximumBytes
        && CC_SHA256(bytes, CC_LONG(size), output.data()) != nullptr;
}
inline bool Hash(const std::vector<std::uint8_t>& bytes, Digest& output) noexcept {
    return Hash(bytes.data(), bytes.size(), output);
}

inline bool BindBakeProof(Definition& value) noexcept {
    try {
        Definition covered = value; covered.bakeProof = {};
        detail::Writer writer; detail::EncodeBody(covered, writer);
        Digest proof{};
        if (!writer.ok || !Hash(writer.bytes, proof) || !Nonzero(proof)) return false;
        value.bakeProof = proof; return true;
    } catch (...) { return false; }
}

inline bool Encode(const Definition& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear(); Refusal refusal;
    try {
        if (!Valid(value, refusal)) return false;
        detail::Writer writer; detail::EncodeBody(value, writer);
        if (!writer.ok || writer.bytes.size() > kMaximumBytes - 32) return false;
        Digest digest{}; if (!Hash(writer.bytes, digest)) return false;
        writer.raw(digest); if (!writer.ok) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Definition& output,
                   Refusal& refusal) noexcept {
    output = {};
    refusal = bytes.size() > kMaximumBytes ? Refusal::Oversized : Refusal::Malformed;
    try {
        constexpr std::size_t minimum = 8 + 2 * 16 + 32 + 4 + 4 + 4 + 1 + 4
            + 5 * 8 + 32 + 32;
        if (bytes.size() < minimum || bytes.size() > kMaximumBytes
            || std::memcmp(bytes.data(), "SYEB\1\0\0\0", 8) != 0) return false;
        const std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{};
        if (!Hash(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin());
        if (expected != actual) return false;
        detail::Reader reader(bytes, bytes.size() - 32);
        std::array<std::uint8_t, 8> prefix{};
        Definition value;
        std::uint64_t algorithm = 0, encoder = 0, bindingCount = 0;
        std::uint64_t resourceCount = 0, maxError = 0, meanError = 0, reserved = 0;
        if (!reader.raw(prefix) || !reader.raw(value.key.document)
            || !reader.raw(value.key.atlas) || !reader.raw(value.layoutProof)
            || !reader.integer(4, algorithm) || !reader.integer(4, encoder)
            || !reader.integer(4, bindingCount) || !reader.integer(1, resourceCount)
            || !reader.integer(1, maxError) || !reader.integer(1, meanError)
            || !reader.integer(1, reserved) || reserved != 0
            || bindingCount == 0 || bindingCount > kMaximumBindings
            || resourceCount == 0 || resourceCount > kMaximumBakedResources)
            return false;
        value.algorithmVersion = std::uint32_t(algorithm);
        value.encoderVersion = std::uint32_t(encoder);
        value.oracle.maximumInteriorError255 = std::uint8_t(maxError);
        value.oracle.maximumMeanError255 = std::uint8_t(meanError);
        if (!reader.real(value.oracle.normalLengthTolerance)
            || !reader.real(value.oracle.maximumDensityRatio)
            || !reader.real(value.oracle.minimumCylinderStretch)
            || !reader.real(value.oracle.maximumCylinderStretch)
            || !reader.real(value.oracle.minimumFixtureOccupancy)) return false;
        value.bindings.resize(std::size_t(bindingCount));
        for (auto& binding : value.bindings) {
            std::uint64_t role = 0, color = 0, wrapU = 0, wrapV = 0;
            if (!reader.raw(binding.owner.document) || !reader.raw(binding.owner.entity)
                || !reader.raw(binding.owner.definition) || !reader.raw(binding.binding)
                || !reader.raw(binding.resource) || !reader.raw(binding.selectorProof)
                || !reader.raw(binding.bindingProof) || !reader.raw(binding.originalContent)
                || !reader.raw(binding.workingContent) || !reader.integer(1, role)
                || !reader.integer(1, color) || !reader.integer(1, wrapU)
                || !reader.integer(1, wrapV)) return false;
            binding.role = Role(role); binding.colorSpace = ColorSpace(color);
            binding.transform.wrapU = Wrap(wrapU); binding.transform.wrapV = Wrap(wrapV);
            for (double& scalar : binding.transform.scale) if (!reader.real(scalar)) return false;
            for (double& scalar : binding.transform.offset) if (!reader.real(scalar)) return false;
            if (!reader.real(binding.transform.rotationDegrees)) return false;
        }
        value.resources.resize(std::size_t(resourceCount));
        for (auto& resource : value.resources) {
            std::uint64_t role = 0, color = 0, wrapU = 0, wrapV = 0, width = 0, height = 0;
            if (!reader.integer(1, role) || !reader.integer(1, color)
                || !reader.integer(1, wrapU) || !reader.integer(1, wrapV)
                || !reader.integer(4, width) || !reader.integer(4, height)
                || !reader.raw(resource.identity) || !reader.raw(resource.content)) return false;
            resource.role = Role(role); resource.colorSpace = ColorSpace(color);
            resource.wrapU = Wrap(wrapU); resource.wrapV = Wrap(wrapV);
            resource.widthTexels = std::uint32_t(width);
            resource.heightTexels = std::uint32_t(height);
        }
        if (!reader.raw(value.bakeProof)) return false;
        std::vector<std::uint8_t> canonical; Refusal validation;
        if (!reader.complete() || !Valid(value, validation)
            || !Encode(value, canonical) || canonical != bytes) return false;
        output = std::move(value); refusal = Refusal::None; return true;
    } catch (...) { output = {}; return false; }
}

namespace persistence {
inline const Standard_GUID& MarkerID() {
    static const Standard_GUID id("E278C2B6-2F4A-4D3B-9C4E-7A1F0B5D8E32"); return id;
}
inline const Standard_GUID& VersionID() {
    static const Standard_GUID id("E278C2B6-2F4A-4D3B-9C4E-7A1F0B5D8E33"); return id;
}
inline const Standard_GUID& BindingCountID() {
    static const Standard_GUID id("E278C2B6-2F4A-4D3B-9C4E-7A1F0B5D8E34"); return id;
}
inline const Standard_GUID& ResourceCountID() {
    static const Standard_GUID id("E278C2B6-2F4A-4D3B-9C4E-7A1F0B5D8E35"); return id;
}
inline const Standard_GUID& ChunkCountID() {
    static const Standard_GUID id("E278C2B6-2F4A-4D3B-9C4E-7A1F0B5D8E36"); return id;
}
inline const Standard_GUID& DigestID() {
    static const Standard_GUID id("E278C2B6-2F4A-4D3B-9C4E-7A1F0B5D8E37"); return id;
}
inline constexpr int RootTag = 279;
inline constexpr Standard_Size ChunkCharacters = 256;
inline constexpr Standard_Size MaximumChunks = 65536;

enum class ReadState : int { Absent = 0, Malformed = 1, Present = 2 };

struct Record final {
    TDF_Label label;
    Definition definition;
    std::vector<std::uint8_t> bytes;
};

inline bool HasSchemaAttribute(const TDF_Label& label) noexcept {
    try {
        Handle(TDF_Attribute) value;
        return label.FindAttribute(MarkerID(), value)
            || label.FindAttribute(VersionID(), value)
            || label.FindAttribute(BindingCountID(), value)
            || label.FindAttribute(ResourceCountID(), value)
            || label.FindAttribute(ChunkCountID(), value)
            || label.FindAttribute(DigestID(), value);
    } catch (...) { return true; }
}

inline bool Hex(const std::vector<std::uint8_t>& bytes, std::string& output) noexcept {
    output.clear();
    try {
        static constexpr char digits[] = "0123456789abcdef";
        if (bytes.empty() || bytes.size() > kMaximumBytes) return false;
        output.reserve(2 * bytes.size());
        for (std::uint8_t byte : bytes) {
            output.push_back(digits[byte >> 4]); output.push_back(digits[byte & 15]);
        }
        return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Unhex(const std::string& text, std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (text.empty() || (text.size() & 1) != 0 || text.size() / 2 > kMaximumBytes)
            return false;
        const auto nibble = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            return -1;
        };
        output.reserve(text.size() / 2);
        for (std::size_t index = 0; index < text.size(); index += 2) {
            const int hi = nibble(text[index]), lo = nibble(text[index + 1]);
            if (hi < 0 || lo < 0) { output.clear(); return false; }
            output.push_back(std::uint8_t((hi << 4) | lo));
        }
        return true;
    } catch (...) { output.clear(); return false; }
}

inline std::string DigestText(const std::vector<std::uint8_t>& bytes) {
    Digest digest{}; if (!Hash(bytes, digest)) return {};
    static constexpr char digits[] = "0123456789abcdef";
    std::string text; text.reserve(64);
    for (std::uint8_t byte : digest) {
        text.push_back(digits[byte >> 4]); text.push_back(digits[byte & 15]);
    }
    return text;
}

inline ReadState ReadLabel(const TDF_Label& label, Record& output) noexcept {
    output = {};
    try {
        if (label.IsNull()) return ReadState::Absent;
        Handle(TDataStd_UAttribute) marker;
        if (!label.FindAttribute(MarkerID(), marker) || marker.IsNull())
            return HasSchemaAttribute(label) ? ReadState::Malformed : ReadState::Absent;
        Handle(TDataStd_Integer) version, bindings, resources, chunks;
        Handle(TDataStd_AsciiString) digest;
        if (!label.FindAttribute(VersionID(), version)
            || !label.FindAttribute(BindingCountID(), bindings)
            || !label.FindAttribute(ResourceCountID(), resources)
            || !label.FindAttribute(ChunkCountID(), chunks)
            || !label.FindAttribute(DigestID(), digest)
            || version.IsNull() || bindings.IsNull() || resources.IsNull()
            || chunks.IsNull() || digest.IsNull() || version->Get() != 1
            || bindings->Get() <= 0 || bindings->Get() > Standard_Integer(kMaximumBindings)
            || resources->Get() <= 0
            || resources->Get() > Standard_Integer(kMaximumBakedResources)
            || chunks->Get() <= 0 || chunks->Get() > Standard_Integer(MaximumChunks))
            return ReadState::Malformed;
        int attributes = 0;
        for (TDF_AttributeIterator it(label); it.More(); it.Next()) {
            const auto& id = it.Value()->ID();
            if (id != MarkerID() && id != VersionID() && id != BindingCountID()
                && id != ResourceCountID() && id != ChunkCountID() && id != DigestID())
                return ReadState::Malformed;
            ++attributes;
        }
        const std::string expected = digest->Get().ToCString();
        if (attributes != 6 || expected.size() != 64) return ReadState::Malformed;
        std::string hex;
        for (Standard_Integer index = 1; index <= chunks->Get(); ++index) {
            const TDF_Label child = label.FindChild(index, Standard_False);
            Handle(TDataStd_AsciiString) text;
            int childAttributes = 0;
            if (child.IsNull() || !child.FindAttribute(TDataStd_AsciiString::GetID(), text)
                || text.IsNull()) return ReadState::Malformed;
            for (TDF_AttributeIterator it(child); it.More(); it.Next()) {
                if (it.Value()->ID() != TDataStd_AsciiString::GetID()) return ReadState::Malformed;
                ++childAttributes;
            }
            const std::string value = text->Get().ToCString();
            if (childAttributes != 1 || value.empty()
                || value.size() > std::size_t(ChunkCharacters)
                || (index < chunks->Get()
                    && value.size() != std::size_t(ChunkCharacters))) return ReadState::Malformed;
            hex += value;
        }
        std::vector<std::uint8_t> bytes;
        Definition definition; Refusal refusal;
        if (!Unhex(hex, bytes) || DigestText(bytes) != expected
            || !Decode(bytes, definition, refusal)
            || definition.bindings.size() != std::size_t(bindings->Get())
            || definition.resources.size() != std::size_t(resources->Get()))
            return ReadState::Malformed;
        int children = 0;
        for (TDF_ChildIterator child(label, Standard_False); child.More(); child.Next()) {
            if (++children > chunks->Get() || child.Value().Tag() > chunks->Get())
                return ReadState::Malformed;
            for (TDF_ChildIterator nested(child.Value(), Standard_True);
                 nested.More(); nested.Next())
                if (nested.Value().HasAttribute()) return ReadState::Malformed;
        }
        output = {label, std::move(definition), std::move(bytes)};
        return ReadState::Present;
    } catch (...) { output = {}; return ReadState::Malformed; }
}

inline bool ReadAll(const Handle(TDocStd_Document)& document,
                    std::vector<Record>& output) noexcept {
    output.clear();
    try {
        if (document.IsNull() || document->GetData().IsNull()) return false;
        const TDF_Label root = document->Main().FindChild(RootTag, Standard_False);
        if (root.IsNull()) return true;
        if (root.HasAttribute() || HasSchemaAttribute(document->Main())) return false;
        UUID documentID{};
        if (!retained_solid::ReadUUID(document->Main(),
                Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)) return false;
        std::size_t aggregate = 0;
        for (TDF_ChildIterator child(root, Standard_False); child.More(); child.Next()) {
            Record record;
            const ReadState state = ReadLabel(child.Value(), record);
            if (state == ReadState::Malformed) return false;
            if (state == ReadState::Absent) {
                if (child.Value().HasAttribute()) return false;
                continue;
            }
            if (!(record.definition.key.document == documentID)
                || record.bytes.size() > kMaximumBytes - aggregate) return false;
            aggregate += record.bytes.size();
            for (const auto& prior : output)
                if (prior.definition.key.atlas == record.definition.key.atlas) return false;
            output.push_back(std::move(record));
        }
        return true;
    } catch (...) { output.clear(); return false; }
}

inline ReadState Read(const Handle(TDocStd_Document)& document,
                      const asset_atlas::Key& key, Record& output) noexcept {
    output = {};
    std::vector<Record> records;
    if (!ReadAll(document, records)) return ReadState::Malformed;
    for (auto& record : records)
        if (record.definition.key == key) { output = std::move(record); return ReadState::Present; }
    return ReadState::Absent;
}

inline bool StageCommitted(const Handle(TDocStd_Document)& document,
                           const Definition& candidate) noexcept {
    try {
        if (document.IsNull() || !document->HasOpenCommand()) return false;
        std::vector<std::uint8_t> bytes; std::string hex;
        if (!Encode(candidate, bytes) || !Hex(bytes, hex)) return false;
        const std::size_t chunkCount = (hex.size() + ChunkCharacters - 1) / ChunkCharacters;
        if (chunkCount == 0 || chunkCount > std::size_t(MaximumChunks)) return false;
        const TDF_Label root = document->Main().FindChild(RootTag, Standard_True);
        if (root.IsNull() || root.HasAttribute()) return false;
        TDF_Label target; int highest = 0;
        for (TDF_ChildIterator child(root, Standard_False); child.More(); child.Next()) {
            highest = std::max(highest, child.Value().Tag());
            Record record;
            const auto state = ReadLabel(child.Value(), record);
            if (state == ReadState::Malformed) return false;
            if (state == ReadState::Present
                && record.definition.key.atlas == candidate.key.atlas) target = child.Value();
        }
        if (target.IsNull()) target = root.FindChild(highest + 1, Standard_True);
        if (target.IsNull()) return false;
        target.ForgetAllAttributes(Standard_True);
        TDataStd_UAttribute::Set(target, MarkerID());
        TDataStd_Integer::Set(target, VersionID(), 1);
        TDataStd_Integer::Set(target, BindingCountID(), Standard_Integer(candidate.bindings.size()));
        TDataStd_Integer::Set(target, ResourceCountID(), Standard_Integer(candidate.resources.size()));
        TDataStd_Integer::Set(target, ChunkCountID(), Standard_Integer(chunkCount));
        const std::string digest = DigestText(bytes);
        if (digest.size() != 64) return false;
        TDataStd_AsciiString::Set(target, DigestID(), TCollection_AsciiString(digest.c_str()));
        for (std::size_t index = 0; index < chunkCount; ++index) {
            const std::string chunk = hex.substr(index * ChunkCharacters, ChunkCharacters);
            TDataStd_AsciiString::Set(target.FindChild(Standard_Integer(index + 1), Standard_True),
                TCollection_AsciiString(chunk.c_str()));
        }
        Record readback;
        return Read(document, candidate.key, readback) == ReadState::Present
            && readback.definition == candidate && readback.bytes == bytes;
    } catch (...) { return false; }
}

template<class Base> class StorageDriver : public Base {
public:
    void Write(const Handle(CDM_Document)& document,
               const TCollection_ExtendedString& file,
               const Message_ProgressRange& progress = Message_ProgressRange()) override {
        Prepare(document); Base::Write(document, file, progress);
    }
    void Write(const Handle(CDM_Document)& document, Standard_OStream& stream,
               const Message_ProgressRange& progress = Message_ProgressRange()) override {
        Prepare(document); Base::Write(document, stream, progress);
    }
private:
    static void Prepare(const Handle(CDM_Document)& value) {
        const auto document = Handle(TDocStd_Document)::DownCast(value);
        if (!Core3DValidatePaintedAtlasBakeDocument(document))
            Standard_Failure::Raise("Painted atlas bake writer record");
    }
};
} // namespace persistence
} // namespace core3d::painted_atlas_bake
