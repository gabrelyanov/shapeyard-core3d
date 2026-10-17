#pragma once

// Deterministic single-threaded painted-atlas derivative.  The caller owns
// image decoding and supplies exact straight RGBA8 working pixels.  This
// kernel owns sampling, chart rasterization, gutter dilation, normal-vector
// renormalization and a metadata-free lossless PNG encoder.  IDAT is one
// zlib stream produced by compress2 at a fixed level/strategy, so repeated
// bakes of equal inputs emit byte-identical PNGs on a given build while the
// document resource table stays far inside its aggregate budget.
#include "PaintedAtlasBakeDefinition.hxx"
#include "AssetAtlasBuild.hxx"
#include "DecalLayerBake.hxx"

#include <CommonCrypto/CommonDigest.h>
#include <zlib.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <new>
#include <vector>

namespace core3d::painted_atlas_bake::kernel {

enum class SourceOrigin : std::uint8_t {
    OrderedLayer,
    FaceBinding,
    OwnerMaterialSlot
};

enum class MaterialSlot : std::uint8_t {
    None,
    CommonDiffuse,
    PBRBaseColor,
    PBREmissive,
    PBRMetallicRoughness,
    PBROcclusion,
    PBRNormal
};

#ifdef DEBUG
struct DebugOperationStorage final {
    std::size_t privateLive = 0;
    std::size_t privateRetained = 0;
    std::size_t privatePeak = 0;
    std::size_t cumulativeWork = 0;
    std::size_t encoderAllocationEntries = 0;
    decal_layer::bake::accounting::FailureSite firstFailure =
        decal_layer::bake::accounting::FailureSite::None;
};
struct DebugKernelObservation final {
    bool rolesReservedBeforeAllocation = false;
    bool outputsReservedBeforeAllocation = false;
    bool ownerReservedBeforeAllocation = false;
    bool gutterReservedBeforeAllocation = false;
    bool encoderReservedBeforeAllocation = false;
    bool outputTransferred = false;
    bool gutterOverlap = false;
    bool atlasJobConsumed = false;
    bool sourceJobConsumed = false;
    bool gutterJobConsumed = false;
    bool encoderJobConsumed = false;
    std::size_t roleCount = 0;
    std::size_t outputRGBABytes = 0;
    std::size_t outputPNGBytes = 0;
    std::size_t encoderReservationAttempts = 0;
    std::size_t encoderAllocationEntries = 0;
};
void DebugBeginKernelObservation(bool denyEncoder) noexcept;
DebugKernelObservation DebugTakeKernelObservation() noexcept;
DebugOperationStorage DebugObserveOperationStorage(
    const decal_layer::bake::accounting::View& operation) noexcept;
#endif

struct Image final {
    decal_layer::bake::accounting::Owner operationOwner;
    decal_layer::bake::accounting::Ticket rgbaTicket;
    std::uint32_t width = 0, height = 0;
    std::vector<std::uint8_t> rgba;

    Image() = default;
    Image(std::uint32_t pixelWidth, std::uint32_t pixelHeight,
          const std::vector<std::uint8_t>& pixels)
        : width(pixelWidth), height(pixelHeight), rgba(pixels) {}
    Image(const Image&) = delete;
    Image& operator=(const Image&) = delete;
    Image(Image&& other) noexcept { *this = std::move(other); }
    Image& operator=(Image&& other) noexcept {
        if (this != &other) {
            reset();
            operationOwner = std::move(other.operationOwner);
            width = other.width; height = other.height;
            rgba = std::move(other.rgba);
            rgbaTicket = std::move(other.rgbaTicket);
            other.width = other.height = 0;
        }
        return *this;
    }
    void reset() noexcept {
        std::vector<std::uint8_t>().swap(rgba);
        width = height = 0;
        rgbaTicket.reset();
        operationOwner.reset();
    }
};

// Bounded ImageIO decode to exact straight RGBA8 working pixels.  Defined in
// PaintedAtlasBakeOwner.mm (the one place Foundation/ImageIO is imported);
// declared here so the DEBUG probe measures persisted PNG bytes with the same
// decoder the product capture path uses.
bool DecodeImage(
    const std::vector<std::uint8_t>& bytes,
    Image& output,
    const decal_layer::bake::accounting::View& operation) noexcept;
//! Source-compatible explicit standalone entry.  It acquires one real outer
//! operation owner and retains it in Image; nested production calls use the
//! required inherited-view overload above.
bool DecodeImage(const std::vector<std::uint8_t>& bytes,
                 Image& output) noexcept;

struct Source final {
    BindingFence fence;
    Image image;
    SourceOrigin origin = SourceOrigin::OrderedLayer;
    MaterialSlot materialSlot = MaterialSlot::None;
    std::size_t memberIndex = std::numeric_limits<std::size_t>::max();
    std::size_t faceIndex = std::numeric_limits<std::size_t>::max();
    std::size_t order = 0;
    bool boundedLayer = false;
    double centerU = 0, centerV = 0;
    double width = 0, height = 0;
    double angle = 0, opacity = 1;
    bool maskPresent = false;
    Image mask;
    bool maskUsesLuminance = false;
    bool maskInverted = false;
    UVTransform maskTransform;
};

//! Operation-local source-vector storage. The reservation precedes vector
//! growth and remains live until every contained Image (and its pixel ticket)
//! has been destroyed.
struct SourceStorage final {
    decal_layer::bake::accounting::Ticket vectorTicket;
    std::vector<Source> values;

    SourceStorage() = default;
    SourceStorage(const SourceStorage&) = delete;
    SourceStorage& operator=(const SourceStorage&) = delete;
    SourceStorage(SourceStorage&& other) noexcept { *this = std::move(other); }
    SourceStorage& operator=(SourceStorage&& other) noexcept {
        if (this != &other) {
            reset();
            values = std::move(other.values);
            vectorTicket = std::move(other.vectorTicket);
        }
        return *this;
    }
    void reset() noexcept {
        std::vector<Source>().swap(values);
        vectorTicket.reset();
    }
};

struct Output final {
    // Declared before the vectors so their backing allocations die first.
    decal_layer::bake::accounting::Ticket rgbaTicket;
    decal_layer::bake::accounting::Ticket pngTicket;
    BakedResource descriptor;
    std::vector<std::uint8_t> rgba;
    std::vector<std::uint8_t> png;
};

struct SampleEvidence final {
    bool available = false;
    Role role = Role::BaseColor;
    std::uint32_t x = 0, y = 0;
    // Chart-local source UV BEFORE the frozen binding transform, so the test
    // reapplies the transform with its own independent arithmetic.
    double sourceU = 0, sourceV = 0;
    std::array<std::uint8_t, 4> emitted{};
};

struct Evidence final {
    std::uint64_t coveredTexels = 0;
    std::uint64_t dilatedTexels = 0;
    std::uint64_t overlapTexels = 0;
    std::uint64_t foreignBleedTexels = 0;
    double maximumInteriorError255 = 0;
    double meanInteriorError255 = 0;
    double maximumNormalLengthError = 0;
    bool tangentParity = true;
    SampleEvidence sample;
    // Deterministic first-N covered texels per role (rasterization order).
    // The DEBUG probe exports these so the test can recompute the expected
    // texel independently from the original fixture images.
    std::vector<SampleEvidence> samples;
};

inline bool ImageValid(const Image& value) noexcept {
    return value.width > 0 && value.height > 0
        && value.width <= std::uint32_t(face_image::kMaximumImageDimension)
        && value.height <= std::uint32_t(face_image::kMaximumImageDimension)
        && std::uint64_t(value.width) * value.height * 4 == value.rgba.size();
}

inline double Wrapped(double value, Wrap wrap) noexcept {
    switch (wrap) {
        case Wrap::ClampToEdge: return std::clamp(value, 0.0, 1.0);
        case Wrap::Repeat: {
            double result = value - std::floor(value);
            return result < 0 ? result + 1 : result;
        }
        case Wrap::MirroredRepeat: {
            double period = std::fmod(value, 2.0);
            if (period < 0) period += 2.0;
            return period <= 1.0 ? period : 2.0 - period;
        }
    }
}

inline std::array<double, 2> TransformUV(double u, double v,
                                         const UVTransform& transform) noexcept {
    constexpr double pi = 3.141592653589793238462643383279502884;
    const double radians = transform.rotationDegrees * pi / 180.0;
    const double c = std::cos(radians), s = std::sin(radians);
    const double x = u - 0.5, y = v - 0.5;
    const double rotatedU = c * x - s * y + 0.5;
    const double rotatedV = s * x + c * y + 0.5;
    return {Wrapped(rotatedU * transform.scale[0] + transform.offset[0], transform.wrapU),
            Wrapped(rotatedV * transform.scale[1] + transform.offset[1], transform.wrapV)};
}

inline std::array<double, 4> Bilinear(const Image& image, double u, double v) noexcept {
    std::array<double, 4> output{};
    if (!ImageValid(image)) return output;
    const double x = std::clamp(u, 0.0, 1.0) * double(image.width - 1);
    const double y = std::clamp(v, 0.0, 1.0) * double(image.height - 1);
    const std::uint32_t x0 = std::uint32_t(std::floor(x));
    const std::uint32_t y0 = std::uint32_t(std::floor(y));
    const std::uint32_t x1 = std::min(x0 + 1, image.width - 1);
    const std::uint32_t y1 = std::min(y0 + 1, image.height - 1);
    const double tx = x - x0, ty = y - y0;
    for (unsigned channel = 0; channel < 4; ++channel) {
        const auto at = [&](std::uint32_t px, std::uint32_t py) {
            return double(image.rgba[(std::size_t(py) * image.width + px) * 4 + channel]);
        };
        output[channel] = (1 - ty) * ((1 - tx) * at(x0, y0) + tx * at(x1, y0))
            + ty * ((1 - tx) * at(x0, y1) + tx * at(x1, y1));
    }
    return output;
}

inline std::array<std::uint8_t, 4> Quantize(const std::array<double, 4>& sample,
                                            Role role,
                                            double& normalError,
                                            std::array<double, 4>& aimed) noexcept {
    aimed = sample;
    if (role == Role::Normal) {
        double x = sample[0] / 127.5 - 1.0;
        double y = sample[1] / 127.5 - 1.0;
        double z = sample[2] / 127.5 - 1.0;
        const double length = std::sqrt(x * x + y * y + z * z);
        if (length > 1.e-12 && std::isfinite(length)) {
            x /= length; y /= length; z /= length;
            normalError = std::max(normalError,
                std::abs(std::sqrt(x * x + y * y + z * z) - 1.0));
            aimed[0] = (x + 1.0) * 127.5;
            aimed[1] = (y + 1.0) * 127.5;
            aimed[2] = (z + 1.0) * 127.5;
        }
    }
    std::array<std::uint8_t, 4> result{};
    for (unsigned channel = 0; channel < 4; ++channel)
        result[channel] = std::uint8_t(std::clamp(std::llround(aimed[channel]), 0LL, 255LL));
    return result;
}

inline std::uint32_t CRC32(const std::uint8_t* bytes, std::size_t count) noexcept {
    std::uint32_t crc = 0xffffffffU;
    for (std::size_t index = 0; index < count; ++index) {
        crc ^= bytes[index];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
    return crc ^ 0xffffffffU;
}

inline void Big(std::vector<std::uint8_t>& bytes, std::uint32_t value) {
    bytes.push_back(std::uint8_t(value >> 24)); bytes.push_back(std::uint8_t(value >> 16));
    bytes.push_back(std::uint8_t(value >> 8)); bytes.push_back(std::uint8_t(value));
}

inline void Chunk(std::vector<std::uint8_t>& png, const char type[4],
                  const std::vector<std::uint8_t>& payload) {
    Big(png, std::uint32_t(payload.size()));
    const std::size_t start = png.size();
    png.insert(png.end(), type, type + 4);
    png.insert(png.end(), payload.begin(), payload.end());
    Big(png, CRC32(png.data() + start, 4 + payload.size()));
}

inline bool EncodePNG(
    std::uint32_t width, std::uint32_t height,
    const std::vector<std::uint8_t>& rgba, bool srgb,
    std::vector<std::uint8_t>& png,
    decal_layer::bake::accounting::Ticket& pngTicket,
    const decal_layer::bake::accounting::View& operation) noexcept {
    using namespace decal_layer::bake::accounting;
    png.clear();
    try {
        if (!operation.valid() || width == 0 || height == 0
            || width > std::uint32_t(face_image::kMaximumImageDimension)
            || height > std::uint32_t(face_image::kMaximumImageDimension)
            || std::uint64_t(width) * height * 4U != rgba.size()) return false;
        std::size_t scanlineBytes = 0;
        if (!operation.CheckedProduct(height, 1U + 4U * std::size_t(width),
                1U, scanlineBytes, FailureSite::ExportCanonicalStorage))
            return false;
        const uLong compressedBound = compressBound(uLong(scanlineBytes));
        if (compressedBound == 0
            || compressedBound > std::numeric_limits<std::size_t>::max() - 70U)
            return false;
        constexpr std::size_t zlibWorkspaceBytes = 512U * 1024U;
        Ticket scanlineTicket, zlibTicket, workspaceTicket, formatTicket;
#ifdef DEBUG
        extern bool DebugKernelEncoderReservationAttempt() noexcept;
        if (DebugKernelEncoderReservationAttempt()) {
            (void)operation.Reserve(StorageDimension::PrivateStorage,
                std::numeric_limits<std::size_t>::max(), Retention::Scratch,
                FailureSite::ExportCanonicalStorage);
            return false;
        }
#endif
        if (!scanlineTicket.acquire(operation, StorageDimension::PrivateStorage,
                scanlineBytes, Retention::Scratch,
                FailureSite::ExportCanonicalStorage)
            || !zlibTicket.acquire(operation, StorageDimension::PrivateStorage,
                std::size_t(compressedBound), Retention::Scratch,
                FailureSite::ExportCanonicalStorage)
            || !workspaceTicket.acquire(operation,
                StorageDimension::PrivateStorage, zlibWorkspaceBytes,
                Retention::Scratch,
                FailureSite::ExportCanonicalStorage)
            || !formatTicket.acquire(operation,
                StorageDimension::PrivateStorage, 13U, Retention::Scratch,
                FailureSite::ExportCanonicalStorage)) return false;
#ifdef DEBUG
        extern void DebugKernelEncoderReserved() noexcept;
        DebugKernelEncoderReserved();
#endif
        if (!operation.EnterAllocation(FailureSite::ExportCanonicalStorage))
            return false;
#ifdef DEBUG
        extern void DebugKernelEncoderAllocated() noexcept;
        DebugKernelEncoderAllocated();
#endif
        std::vector<std::uint8_t> scanlines;
        scanlines.reserve(scanlineBytes);
        for (std::uint32_t y = 0; y < height; ++y) {
            scanlines.push_back(0);
            const auto first = rgba.begin() + std::size_t(y) * width * 4;
            scanlines.insert(scanlines.end(), first, first + std::size_t(width) * 4);
        }
        // One bounded zlib stream: fixed level and default strategy/memLevel,
        // so equal inputs deflate to identical bytes on this build.
        uLongf compressedLength = compressedBound;
        std::vector<std::uint8_t> zlib(compressedLength);
        if (compress2(zlib.data(), &compressedLength, scanlines.data(),
                      uLong(scanlines.size()), Z_BEST_COMPRESSION) != Z_OK)
            return false;
        zlib.resize(compressedLength);
        const std::size_t pngCapacity = std::size_t(compressedLength)
            + (srgb ? 70U : 57U);
        if (!pngTicket.acquire(operation, StorageDimension::EncodedTexture,
                pngCapacity, Retention::Retained,
                FailureSite::ExportCanonicalStorage)
            || !operation.EnterAllocation(
                FailureSite::ExportCanonicalStorage)) return false;
        png.reserve(pngCapacity);
        png = {137,80,78,71,13,10,26,10};
        std::vector<std::uint8_t> ihdr;
        ihdr.reserve(13);
        Big(ihdr, width); Big(ihdr, height);
        ihdr.insert(ihdr.end(), {8, 6, 0, 0, 0});
        Chunk(png, "IHDR", ihdr);
        if (srgb) {
            ihdr.assign(1U, 0U);
            Chunk(png, "sRGB", ihdr);
        }
        Chunk(png, "IDAT", zlib); Chunk(png, "IEND", {});
        if (png.capacity() > pngTicket.bytes
            || png.size() > kMaximumEncodedImageBytes) {
            operation.AllocationFailed(FailureSite::ExportCanonicalStorage);
            return false;
        }
        return true;
    } catch (const std::bad_alloc&) {
        operation.AllocationFailed(FailureSite::ExportCanonicalStorage);
        png.clear(); pngTicket.reset(); return false;
    } catch (...) { png.clear(); pngTicket.reset(); return false; }
}

inline bool ImageHash(const std::vector<std::uint8_t>& bytes, Digest& digest) noexcept {
    digest = {};
    return !bytes.empty() && bytes.size() <= kMaximumEncodedImageBytes
        && CC_SHA256(bytes.data(), CC_LONG(bytes.size()), digest.data()) != nullptr;
}

inline bool Bake(const asset_atlas::Definition& atlas,
                 const asset_atlas::build::LayoutEvidence& layout,
                 const std::vector<Source>& sources,
                 std::vector<Output>& outputs,
                 Evidence& evidence,
                 decal_layer::bake::accounting::Ticket& outputVectorTicket,
                 decal_layer::bake::accounting::Ticket& evidenceTicket,
                 const decal_layer::bake::accounting::View& operation,
                 bool atlasJobAlreadySealed = false) noexcept {
    using namespace decal_layer::bake::accounting;
    std::vector<Output>().swap(outputs);
    outputVectorTicket.reset();
    evidenceTicket.reset();
    evidence = {};
    try {
        if (!operation.valid()
            || atlas.resolutionTexels <= 0 || atlas.resolutionTexels > kMaximumBakedDimension
            || atlas.gutterTexels < 1 || atlas.gutterTexels > 8
            || layout.charts.empty() || layout.packed.triangles.empty()
            || layout.chartMembers.size() != layout.charts.size()
            || layout.chartFaces.size() != layout.charts.size()) return false;
        constexpr std::size_t unspecified =
            std::numeric_limits<std::size_t>::max();
        for (const auto& source : sources) {
            bool ownerPresent = false;
            for (const auto& member : atlas.members)
                ownerPresent = ownerPresent || member.owner == source.fence.owner;
            if (!Valid(source.fence) || !ImageValid(source.image)
                || !ownerPresent
                || (source.origin == SourceOrigin::FaceBinding
                    && (source.memberIndex == unspecified
                        || source.faceIndex == unspecified
                        || source.materialSlot != MaterialSlot::None))
                || (source.origin == SourceOrigin::OwnerMaterialSlot
                    && (source.memberIndex == unspecified
                        || source.materialSlot == MaterialSlot::None
                        || source.faceIndex != unspecified))
                || (source.origin == SourceOrigin::OrderedLayer
                    && source.materialSlot != MaterialSlot::None)
                || !std::isfinite(source.opacity)
                || source.opacity < 0.0 || source.opacity > 1.0
                || (source.memberIndex != unspecified
                    && (source.memberIndex >= atlas.members.size()
                        || !(atlas.members[source.memberIndex].owner
                            == source.fence.owner)))
                || (source.boundedLayer
                    && (!std::isfinite(source.centerU)
                        || !std::isfinite(source.centerV)
                        || !std::isfinite(source.width)
                        || !std::isfinite(source.height)
                        || !std::isfinite(source.angle)
                        || !(source.width > 0.0)
                        || !(source.height > 0.0)))
                || (source.maskPresent
                    && (!ImageValid(source.mask)
                        || !face_image::Valid(source.maskTransform))))
                return false;
            if (source.faceIndex != unspecified) {
                bool represented = false;
                for (std::size_t chart = 0;
                     chart < layout.chartFaces.size(); ++chart) {
                    if (layout.chartFaces[chart] != source.faceIndex) continue;
                    const std::size_t member = layout.chartMembers[chart].first;
                    if (member >= atlas.members.size()) return false;
                    if (atlas.members[member].owner == source.fence.owner
                        && (source.memberIndex == unspecified
                            || member == source.memberIndex)) {
                        represented = true;
                        break;
                    }
                }
                if (!represented) return false;
            }
        }
        const std::uint32_t size = std::uint32_t(atlas.resolutionTexels);
        Ticket rolesTicket;
        std::size_t rolesBytes = 0;
        if (!operation.CheckedProduct(sources.size(), sizeof(Role), 1U,
                rolesBytes, FailureSite::ExportObservedStorage)
            || !rolesTicket.acquire(operation, StorageDimension::PrivateStorage,
                rolesBytes, Retention::Scratch,
                FailureSite::ExportObservedStorage)
            || !operation.EnterAllocation(FailureSite::ExportObservedStorage))
            return false;
        std::vector<Role> roles;
        roles.reserve(sources.size());
#ifdef DEBUG
        extern void DebugKernelRolesReserved() noexcept;
        DebugKernelRolesReserved();
#endif
        for (const auto& source : sources)
            if (std::find(roles.begin(), roles.end(), source.fence.role) == roles.end())
                roles.push_back(source.fence.role);
        std::sort(roles.begin(), roles.end(), [](Role a, Role b) {
            return std::uint8_t(a) < std::uint8_t(b);
        });
        if (roles.empty() || roles.capacity() * sizeof(Role) > rolesTicket.bytes)
            return false;

        std::size_t pixels = 0, pixelBytes = 0, rolePixels = 0;
        std::size_t gutterWork = 0, encoderWork = 0;
        std::size_t sourceWork = 0, maskWork = 0, maskCount = 0;
        std::size_t atlasWork = 0, part = 0;
        const auto add = [&](std::size_t value, std::size_t& total) {
            if (value > std::numeric_limits<std::size_t>::max() - total) {
                operation.Refuse(FailureSite::CheckedProduct);
                return false;
            }
            total += value;
            return true;
        };
        if (!operation.CheckedProduct(size, size, 1U, pixels,
                FailureSite::ExportCanonicalStorage)
            || !operation.CheckedProduct(pixels, 4U, 1U, pixelBytes,
                FailureSite::ExportCanonicalStorage)
            || !operation.CheckedProduct(pixels, roles.size(), 1U, rolePixels,
                FailureSite::ExportCanonicalStorage)
            || !operation.CheckedProduct(rolePixels,
                std::size_t(atlas.gutterTexels), 1U, gutterWork,
                FailureSite::ExportCanonicalStorage)
            || !operation.CheckedProduct(pixelBytes, roles.size(), 1U,
                encoderWork, FailureSite::ExportCanonicalStorage)) return false;
        for (const auto& source : sources) {
            if (!operation.CheckedProduct(source.image.width,
                    source.image.height, 1U, part,
                    FailureSite::ExportCanonicalStorage)
                || !add(part, sourceWork)) return false;
            if (source.maskPresent) {
                ++maskCount;
                if (!ImageValid(source.mask)
                    || !operation.CheckedProduct(source.mask.width,
                        source.mask.height, 1U, part,
                        FailureSite::ExportCanonicalStorage)
                || !add(part, maskWork)) return false;
            }
        }
        // The source and mask jobs cover both decoded input texels and all
        // later ordered-compositor visits. Plan the latter from the actual
        // packed rectangles before any output allocation or sampling.
        for (Role role : roles) {
            std::size_t roleSources = 0;
            for (const auto& source : sources)
                if (source.fence.role == role) ++roleSources;
            if (!add(sources.size(), sourceWork)
                || !operation.CheckedProduct(roleSources, roleSources, 1U,
                    part, FailureSite::ExportCanonicalStorage)
                || !add(part, sourceWork)) return false;
            for (const auto& placed : layout.packed.triangles) {
                if (placed.chartIndex < 0
                    || std::size_t(placed.chartIndex)
                        >= layout.chartMembers.size()
                    || std::size_t(placed.chartIndex)
                        >= layout.chartFaces.size()
                    || std::size_t(placed.chartIndex) >= layout.charts.size()
                    || placed.triangleIndex < 0
                    || std::size_t(placed.triangleIndex)
                        >= layout.charts[std::size_t(placed.chartIndex)]
                            .triangles.size())
                    return false;
                const std::size_t chart = std::size_t(placed.chartIndex);
                const std::size_t member = layout.chartMembers[chart].first;
                if (member >= atlas.members.size()) return false;
                const std::size_t face = layout.chartFaces[chart];
                std::size_t candidates = 0, maskedCandidates = 0;
                for (const auto& source : sources) {
                    if (source.fence.role != role
                        || !(source.fence.owner == atlas.members[member].owner)
                        || (source.memberIndex != unspecified
                            && source.memberIndex != member)
                        || (source.faceIndex != unspecified
                            && source.faceIndex != face)) continue;
                    ++candidates;
                    if (source.maskPresent) ++maskedCandidates;
                }
                double x[3], y[3];
                for (int corner = 0; corner < 3; ++corner) {
                    x[corner] = placed.uv[corner].x * size;
                    y[corner] = placed.uv[corner].y * size;
                    if (!std::isfinite(x[corner]) || !std::isfinite(y[corner]))
                        return false;
                }
                const int minX = std::max(0, int(std::ceil(
                    std::min({x[0], x[1], x[2]}) - .5)));
                const int maxX = std::min(int(size) - 1, int(std::floor(
                    std::max({x[0], x[1], x[2]}) - .5)));
                const int minY = std::max(0, int(std::ceil(
                    std::min({y[0], y[1], y[2]}) - .5)));
                const int maxY = std::min(int(size) - 1, int(std::floor(
                    std::max({y[0], y[1], y[2]}) - .5)));
                std::size_t visits = 0;
                if (maxX >= minX && maxY >= minY
                    && (!operation.CheckedProduct(
                            std::size_t(maxX - minX + 1),
                            std::size_t(maxY - minY + 1), 1U, visits,
                            FailureSite::ExportCanonicalStorage)
                        || !add(visits, sourceWork)
                        || !operation.CheckedProduct(visits, candidates, 1U,
                            part, FailureSite::ExportCanonicalStorage)
                        || !add(part, sourceWork)
                        || !operation.CheckedProduct(visits,
                            maskedCandidates, 1U, part,
                            FailureSite::ExportCanonicalStorage)
                        || !add(part, maskWork))) return false;
            }
        }
        atlasWork = rolePixels;
        if (!operation.CheckedProduct(layout.packed.triangles.size(), 3U,
                std::size_t(atlas.gutterTexels), part,
                FailureSite::ExportCanonicalStorage)
            || !add(part, atlasWork)
            || !add(layout.packed.triangles.size(), atlasWork)
            || !add(sources.size(), atlasWork)
            || !add(maskCount, atlasWork)
            || (!atlasJobAlreadySealed
                && !operation.SealJob(Job::AtlasRoles, atlasWork,
                    FailureSite::ExportCanonicalStorage))
            || !operation.ConsumeJob(Job::AtlasRoles, atlasWork,
                FailureSite::ExportCanonicalStorage)
            || !operation.SealJob(Job::SourcePixels, sourceWork,
                FailureSite::ExportCanonicalStorage)
            || !operation.ConsumeJob(Job::SourcePixels, sourceWork,
                FailureSite::ExportCanonicalStorage)
            || !operation.SealJob(Job::MaskPixels, maskWork,
                FailureSite::ExportCanonicalStorage)
            || !operation.ConsumeJob(Job::MaskPixels, maskWork,
                FailureSite::ExportCanonicalStorage)
            || !operation.SealJob(Job::Gutter, gutterWork,
                FailureSite::ExportCanonicalStorage)
            || !operation.ConsumeJob(Job::Gutter, gutterWork,
                FailureSite::ExportCanonicalStorage)
            || !operation.SealJob(Job::Encoder, encoderWork,
                FailureSite::ExportCanonicalStorage)
            || !operation.ConsumeJob(Job::Encoder, encoderWork,
                FailureSite::ExportCanonicalStorage)) return false;
#ifdef DEBUG
        extern void DebugKernelJobsConsumed(std::size_t) noexcept;
        DebugKernelJobsConsumed(roles.size());
#endif
        std::size_t outputVectorBytes = 0;
        constexpr std::size_t maximumEvidenceSamples = 12;
        if (!operation.CheckedProduct(roles.size(), sizeof(Output), 1U,
                outputVectorBytes, FailureSite::ExportBakeStorage)
            || !outputVectorTicket.acquire(operation,
                StorageDimension::PrivateStorage, outputVectorBytes,
                Retention::Retained, FailureSite::ExportBakeStorage)
            || !evidenceTicket.acquireProduct(operation,
                StorageDimension::PrivateStorage, maximumEvidenceSamples,
                sizeof(SampleEvidence), 1U, Retention::Retained,
                FailureSite::ExportObservedStorage)
            || !operation.EnterAllocation(FailureSite::ExportBakeStorage))
            return false;
        outputs.reserve(roles.size());
        evidence.samples.reserve(maximumEvidenceSamples);
#ifdef DEBUG
        extern void DebugKernelOutputsReserved() noexcept;
        DebugKernelOutputsReserved();
#endif
        std::uint64_t aggregate = 0;
        double interiorErrorSum = 0;
        std::uint64_t interiorErrorCount = 0;
        for (Role role : roles) {
            aggregate += std::uint64_t(size) * size * 4;
            if (aggregate > kMaximumDecodedImageBytes) return false;
            Output output;
            output.descriptor.role = role;
            output.descriptor.colorSpace = face_image::ValidColorSpaceForRole(role, ColorSpace::SRGB)
                ? ColorSpace::SRGB : ColorSpace::Linear;
            output.descriptor.widthTexels = size; output.descriptor.heightTexels = size;
            Ticket applicableTicket;
            std::size_t applicableBytes = 0;
            if (!operation.CheckedProduct(sources.size(),
                    sizeof(const Source*), 1U, applicableBytes,
                    FailureSite::ExportObservedStorage)
                || !applicableTicket.acquire(operation,
                    StorageDimension::PrivateStorage, applicableBytes,
                    Retention::Scratch, FailureSite::ExportObservedStorage)
                || !operation.EnterAllocation(
                    FailureSite::ExportObservedStorage)) return false;
            std::vector<const Source*> applicable;
            applicable.reserve(sources.size());
            for (const auto& source : sources)
                if (source.fence.role == role) applicable.push_back(&source);
            if (applicable.empty()) return false;
            // Stable insertion sort has no hidden allocation. Strict `>`
            // retains capture order for equal-order layers.
            for (std::size_t index = 1; index < applicable.size(); ++index) {
                const Source* value = applicable[index];
                std::size_t insertion = index;
                while (insertion > 0
                       && applicable[insertion - 1]->order > value->order) {
                    applicable[insertion] = applicable[insertion - 1];
                    --insertion;
                }
                applicable[insertion] = value;
            }

            Ticket ownerTicket;
            std::size_t ownerBytes = 0;
            if (!operation.CheckedProduct(pixels, sizeof(std::int16_t), 1U,
                    ownerBytes, FailureSite::ExportObservedStorage)
                || !output.rgbaTicket.acquire(operation,
                    StorageDimension::PrivateStorage, pixelBytes,
                    Retention::Retained, FailureSite::ExportBakeStorage)
                || !ownerTicket.acquire(operation,
                    StorageDimension::PrivateStorage, ownerBytes,
                    Retention::Scratch, FailureSite::ExportObservedStorage)
                || !operation.EnterAllocation(FailureSite::ExportBakeStorage))
                return false;
#ifdef DEBUG
            extern void DebugKernelOwnerReserved(std::size_t) noexcept;
            DebugKernelOwnerReserved(pixelBytes);
#endif
            output.rgba.assign(pixelBytes, 0);
            if (role == Role::Normal)
                for (std::size_t i = 0; i < output.rgba.size(); i += 4) {
                    output.rgba[i] = 128; output.rgba[i + 1] = 128;
                    output.rgba[i + 2] = 255; output.rgba[i + 3] = 255;
                }
            // A final atlas has at most 64 charts, so the ownership sentinel
            // and every chart index fit exactly in int16 without changing any
            // comparison or raster arithmetic.
            std::vector<std::int16_t> owner(pixels, -1);
            for (const auto& placed : layout.packed.triangles) {
                if (placed.chartIndex < 0
                    || std::size_t(placed.chartIndex) >= layout.chartMembers.size()
                    || placed.triangleIndex < 0
                    || std::size_t(placed.triangleIndex)
                        >= layout.charts[std::size_t(placed.chartIndex)].triangles.size())
                    return false;
                const std::size_t memberIndex =
                    layout.chartMembers[std::size_t(placed.chartIndex)].first;
                if (memberIndex >= atlas.members.size()) return false;
                const std::size_t faceIndex =
                    layout.chartFaces[std::size_t(placed.chartIndex)];
                bool hasApplicable = false;
                for (const auto* candidate : applicable)
                    if (candidate->fence.owner == atlas.members[memberIndex].owner
                        && (candidate->memberIndex == unspecified
                            || candidate->memberIndex == memberIndex)
                        && (candidate->faceIndex == unspecified
                            || candidate->faceIndex == faceIndex)) {
                        hasApplicable = true;
                        break;
                    }
                const auto& developed =
                    layout.charts[std::size_t(placed.chartIndex)]
                        .triangles[std::size_t(placed.triangleIndex)];
                const auto& chart = layout.charts[std::size_t(placed.chartIndex)];
                const double du = chart.rectMax.x - chart.rectMin.x;
                const double dv = chart.rectMax.y - chart.rectMin.y;
                if (!(du > 0) || !(dv > 0)) return false;
                double x[3], y[3], su[3], sv[3];
                for (int corner = 0; corner < 3; ++corner) {
                    x[corner] = placed.uv[corner].x * size;
                    y[corner] = placed.uv[corner].y * size;
                    su[corner] = (developed.corner[corner].x - chart.rectMin.x) / du;
                    sv[corner] = (developed.corner[corner].y - chart.rectMin.y) / dv;
                }
                const double area = (x[1] - x[0]) * (y[2] - y[0])
                    - (x[2] - x[0]) * (y[1] - y[0]);
                if (std::abs(area) <= 1.e-18) return false;
                int order[3] = {0, 1, 2};
                if (area < 0) std::swap(order[1], order[2]);
                const int minX = std::max(0, int(std::ceil(std::min({x[0],x[1],x[2]}) - .5)));
                const int maxX = std::min(int(size) - 1,
                    int(std::floor(std::max({x[0],x[1],x[2]}) - .5)));
                const int minY = std::max(0, int(std::ceil(std::min({y[0],y[1],y[2]}) - .5)));
                const int maxY = std::min(int(size) - 1,
                    int(std::floor(std::max({y[0],y[1],y[2]}) - .5)));
                for (int py = minY; py <= maxY; ++py) for (int px = minX; px <= maxX; ++px) {
                    const double qx = px + .5, qy = py + .5;
                    // Half-open top-left ownership is shared with the
                    // independent E2a raster census. Shared triangle edges
                    // are covered exactly once, independent of emission
                    // order, before any sampling or dilation occurs.
                    bool inside = true;
                    for (int edge = 0; edge < 3 && inside; ++edge) {
                        const int a = order[edge], b = order[(edge + 1) % 3];
                        const double dx = x[b] - x[a], dy = y[b] - y[a];
                        const double cross = dx * (qy - y[a]) - dy * (qx - x[a]);
                        const double band = 1.e-9 * (dx * dx + dy * dy + 1.0);
                        if (cross > band) continue;
                        if (cross < -band) { inside = false; break; }
                        const bool owns = dy < 0.0 || (dy == 0.0 && dx < 0.0);
                        if (!owns) inside = false;
                    }
                    if (!inside) continue;
                    const double w0 = ((x[1] - qx) * (y[2] - qy)
                        - (x[2] - qx) * (y[1] - qy)) / area;
                    const double w1 = ((x[2] - qx) * (y[0] - qy)
                        - (x[0] - qx) * (y[2] - qy)) / area;
                    const double w2 = 1.0 - w0 - w1;
                    const std::size_t pixel = std::size_t(py) * size + std::size_t(px);
                    if (owner[pixel] >= 0 && owner[pixel] != placed.chartIndex) {
                        ++evidence.overlapTexels; continue;
                    }
                    owner[pixel] = placed.chartIndex;
                    const double localU = w0 * su[0] + w1 * su[1] + w2 * su[2];
                    const double localV = w0 * sv[0] + w1 * sv[1] + w2 * sv[2];
                    if (!hasApplicable) continue;
                    bool sampled = false;
                    double accumulatedAlpha = 0.0;
                    std::array<double, 4> composed{};
                    double evidenceU = localU, evidenceV = localV;
                    const double developedU = chart.rectMin.x + localU * du;
                    const double developedV = chart.rectMin.y + localV * dv;
                    for (const auto* source : applicable) {
                        if (!(source->fence.owner
                                == atlas.members[memberIndex].owner)
                            || (source->memberIndex != unspecified
                                && source->memberIndex != memberIndex)
                            || (source->faceIndex != unspecified
                                && source->faceIndex != faceIndex)) continue;
                        double sourceU = localU, sourceV = localV;
                        if (source->boundedLayer) {
                            constexpr double pi =
                                3.141592653589793238462643383279502884;
                            const double radians = -source->angle * pi / 180.0;
                            const double c = std::cos(radians);
                            const double s = std::sin(radians);
                            const double centeredU = developedU - source->centerU;
                            const double centeredV = developedV - source->centerV;
                            sourceU = (c * centeredU - s * centeredV)
                                / source->width + .5;
                            sourceV = (s * centeredU + c * centeredV)
                                / source->height + .5;
                            if (sourceU < 0.0 || sourceU > 1.0
                                || sourceV < 0.0 || sourceV > 1.0) continue;
                        }
                        const auto uv = TransformUV(
                            sourceU, sourceV, source->fence.transform);
                        const auto sample = Bilinear(
                            source->image, uv[0], uv[1]);
                        double maskScalar = 1.0;
                        if (source->maskPresent) {
                            const auto maskUV = TransformUV(
                                sourceU, sourceV, source->maskTransform);
                            const auto mask = Bilinear(
                                source->mask, maskUV[0], maskUV[1]);
                            maskScalar = source->maskUsesLuminance
                                ? (0.2126 * mask[0] + 0.7152 * mask[1]
                                    + 0.0722 * mask[2]) / 255.0
                                : mask[3] / 255.0;
                            if (source->maskInverted)
                                maskScalar = 1.0 - maskScalar;
                            maskScalar = std::clamp(maskScalar, 0.0, 1.0);
                        }
                        const double sourceAlpha = std::clamp(
                            sample[3] / 255.0 * source->opacity * maskScalar,
                            0.0, 1.0);
                        if (!sampled) {
                            composed = sample;
                            accumulatedAlpha = sourceAlpha;
                            sampled = true;
                            evidenceU = sourceU;
                            evidenceV = sourceV;
                            continue;
                        }
                        const double outputAlpha = sourceAlpha
                            + accumulatedAlpha * (1.0 - sourceAlpha);
                        if (outputAlpha > 0.0) {
                            for (unsigned channel = 0; channel < 3; ++channel)
                                composed[channel] = (sample[channel] * sourceAlpha
                                    + composed[channel] * accumulatedAlpha
                                        * (1.0 - sourceAlpha)) / outputAlpha;
                        }
                        accumulatedAlpha = outputAlpha;
                        composed[3] = outputAlpha * 255.0;
                    }
                    if (!sampled) continue;
                    composed[3] = accumulatedAlpha * 255.0;
                    double normalError = 0; std::array<double, 4> aimed;
                    const auto quantized = Quantize(
                        composed, role, normalError, aimed);
                    evidence.maximumNormalLengthError =
                        std::max(evidence.maximumNormalLengthError, normalError);
                    // Measured interior error: quantized emission against the
                    // exact (post-renormalization for normals) bilinear aim.
                    double worst = 0;
                    for (unsigned channel = 0; channel < 4; ++channel)
                        worst = std::max(worst,
                            std::abs(double(quantized[channel]) - aimed[channel]));
                    evidence.maximumInteriorError255 =
                        std::max(evidence.maximumInteriorError255, worst);
                    interiorErrorSum += worst; ++interiorErrorCount;
                    std::copy(quantized.begin(), quantized.end(), output.rgba.begin() + pixel * 4);
                    ++evidence.coveredTexels;
                    if (!evidence.sample.available && role == Role::BaseColor) {
                        evidence.sample.available = true; evidence.sample.role = role;
                        evidence.sample.x = std::uint32_t(px); evidence.sample.y = std::uint32_t(py);
                        evidence.sample.sourceU = evidenceU;
                        evidence.sample.sourceV = evidenceV;
                        evidence.sample.emitted = quantized;
                    }
                    const std::size_t sampleCap = role == Role::BaseColor ? 8
                        : role == Role::Normal ? 4 : 0;
                    std::size_t roleSamples = 0;
                    for (const auto& existing : evidence.samples)
                        if (existing.role == role) ++roleSamples;
                    if (roleSamples < sampleCap) {
                        SampleEvidence entry;
                        entry.available = true; entry.role = role;
                        entry.x = std::uint32_t(px); entry.y = std::uint32_t(py);
                        entry.sourceU = evidenceU; entry.sourceV = evidenceV;
                        entry.emitted = quantized;
                        evidence.samples.push_back(entry);
                    }
                }
                const double outputDet = (placed.uv[1].x - placed.uv[0].x)
                    * (placed.uv[2].y - placed.uv[0].y)
                    - (placed.uv[2].x - placed.uv[0].x)
                    * (placed.uv[1].y - placed.uv[0].y);
                const double sourceDet = (su[1] - su[0]) * (sv[2] - sv[0])
                    - (su[2] - su[0]) * (sv[1] - sv[0]);
                evidence.tangentParity = evidence.tangentParity
                    && outputDet * sourceDet > 0;
            }
            // Exact N-step chart-local Manhattan dilation.  A texel is copied
            // only from one chart and never across an already owned chart.
            for (int step = 0; step < atlas.gutterTexels; ++step) {
                Ticket gutterTicket;
                std::size_t gutterBytes = ownerBytes;
                if (!add(pixelBytes, gutterBytes)
                    || !gutterTicket.acquire(operation,
                        StorageDimension::PrivateStorage, gutterBytes,
                        Retention::Scratch,
                        FailureSite::ExportObservedStorage)
                    || !operation.EnterAllocation(
                        FailureSite::ExportObservedStorage)) return false;
#ifdef DEBUG
                extern void DebugKernelGutterReserved(std::size_t) noexcept;
                DebugKernelGutterReserved(gutterBytes);
#endif
                auto nextOwner = owner; auto nextRGBA = output.rgba;
                for (std::uint32_t y = 0; y < size; ++y) for (std::uint32_t x = 0; x < size; ++x) {
                    const std::size_t pixel = std::size_t(y) * size + x;
                    if (owner[pixel] >= 0) continue;
                    std::int32_t selected = -1; std::size_t sourcePixel = 0;
                    const std::array<std::pair<int,int>,4> offsets{{{-1,0},{1,0},{0,-1},{0,1}}};
                    for (const auto& offset : offsets) {
                        const int nx = int(x) + offset.first, ny = int(y) + offset.second;
                        if (nx < 0 || ny < 0 || nx >= int(size) || ny >= int(size)) continue;
                        const std::size_t neighbour = std::size_t(ny) * size + std::size_t(nx);
                        if (owner[neighbour] < 0) continue;
                        if (selected >= 0 && selected != owner[neighbour]) continue;
                        selected = owner[neighbour]; sourcePixel = neighbour;
                    }
                    if (selected >= 0) {
                        nextOwner[pixel] = selected;
                        std::copy_n(output.rgba.begin() + sourcePixel * 4, 4,
                                    nextRGBA.begin() + pixel * 4);
                        ++evidence.dilatedTexels;
                    }
                }
                owner.swap(nextOwner); output.rgba.swap(nextRGBA);
            }
            // The owner/applicable scratch has no consumer after dilation.
            // Release it before encoder scanlines/zlib overlap the retained
            // outputs. EncodePNG borrows the immutable RGBA bytes directly.
            std::vector<std::int16_t>().swap(owner);
            ownerTicket.reset();
            std::vector<const Source*>().swap(applicable);
            applicableTicket.reset();
            const bool srgb = role == Role::BaseColor || role == Role::Emissive;
            if (!EncodePNG(size, size, output.rgba, srgb, output.png,
                    output.pngTicket, operation)
                || !ImageHash(output.png, output.descriptor.content)) return false;
            // Resource identity is stable and role-bound, while content stays
            // the digest of the exact PNG bytes.  Two semantic roles may emit
            // equal background-heavy PNGs, but they must remain distinct
            // resources under the fail-closed SYEB validator.
            Ticket identityTicket;
            constexpr std::size_t identityBytesCount = 7U + Digest{}.size();
            if (!identityTicket.acquire(operation,
                    StorageDimension::PrivateStorage, identityBytesCount,
                    Retention::Scratch, FailureSite::ExportCanonicalStorage)
                || !operation.EnterAllocation(
                    FailureSite::ExportCanonicalStorage)) return false;
            std::vector<std::uint8_t> identityBytes;
            identityBytes.reserve(identityBytesCount);
            identityBytes.insert(identityBytes.end(), {
                'S', 'Y', 'E', 'B', '/', '1', std::uint8_t(role)});
            identityBytes.insert(identityBytes.end(), output.descriptor.content.begin(),
                                 output.descriptor.content.end());
            if (!retained_solid::Hash(identityBytes, output.descriptor.identity)) return false;
            outputs.push_back(std::move(output));
#ifdef DEBUG
            extern void DebugKernelOutputTransferred(std::size_t) noexcept;
            DebugKernelOutputTransferred(outputs.back().png.size());
#endif
        }
        if (interiorErrorCount > 0)
            evidence.meanInteriorError255 = interiorErrorSum / double(interiorErrorCount);
        return !outputs.empty() && interiorErrorCount > 0
            && evidence.overlapTexels == 0
            && evidence.foreignBleedTexels == 0
            && evidence.maximumInteriorError255 <= 2
            && evidence.meanInteriorError255 <= 1
            && evidence.maximumNormalLengthError <= 1.e-5
            && evidence.tangentParity;
    } catch (const std::bad_alloc&) {
        operation.AllocationFailed(FailureSite::ExportBakeStorage);
        std::vector<Output>().swap(outputs);
        outputVectorTicket.reset();
        evidenceTicket.reset();
        evidence = {};
        return false;
    } catch (...) {
        std::vector<Output>().swap(outputs);
        outputVectorTicket.reset();
        evidenceTicket.reset();
        evidence = {};
        return false;
    }
}
} // namespace core3d::painted_atlas_bake::kernel

namespace core3d::painted_atlas_bake::owner {
enum class Outcome : std::uint8_t {
    Prepared, Committed, Refused, StaleSource, StaleBinding, MissingResource,
    ForeignResource, OverBudget, UnsupportedSurface, OwnerMismatch, Busy,
    Malformed, PersistenceFailure, Absent
};

//! Admission retained across the strict whole-document SYEA reader. The
//! encoded ticket covers the largest simultaneously-live canonical Encode
//! scratch; local covers returned records, member-owner labels and traversal
//! overlap. Both are acquired before persistence::ReadAll is entered.
struct AtlasReadAdmission final {
    decal_layer::bake::accounting::Ticket encoded;
    decal_layer::bake::accounting::Ticket local;

    void reset() noexcept {
        local.reset();
        encoded.reset();
    }
};

Outcome PreflightAtlasRead(
    const Handle(TDocStd_Document)& document,
    const decal_layer::bake::accounting::View& operation,
    AtlasReadAdmission& admission,
    decal_layer::bake::accounting::FailureSite site =
        decal_layer::bake::accounting::FailureSite::AtlasReadScratch) noexcept;

//! Tickets acquired from immutable document metadata before the strict
//! resource reader materializes its table, canonical encodings, or returned
//! deep envelope. Scratch dies after the read; returned byte tickets move to
//! the value that owns the corresponding vectors.
struct ResourceReadAdmission final {
    decal_layer::bake::accounting::Ticket table;
    decal_layer::bake::accounting::Ticket serialization;
    decal_layer::bake::accounting::Ticket originalBytes;
    decal_layer::bake::accounting::Ticket workingBytes;
    bool selected = false;

    void reset() noexcept {
        workingBytes.reset();
        originalBytes.reset();
        serialization.reset();
        table.reset();
        selected = false;
    }
};

Outcome PreflightResourceRead(
    const Handle(TDocStd_Document)& document,
    const face_image::UUID& resource,
    const decal_layer::bake::accounting::View& operation,
    ResourceReadAdmission& admission) noexcept;

struct CapturedSource final {
    BindingFence fence;
    decal_layer::bake::accounting::Ticket originalBytesTicket;
    decal_layer::bake::accounting::Ticket workingBytesTicket;
    decal_layer::bake::accounting::Ticket maskOriginalBytesTicket;
    decal_layer::bake::accounting::Ticket maskWorkingBytesTicket;
    face_image::ResourceEnvelope envelope;
    kernel::SourceOrigin origin = kernel::SourceOrigin::OrderedLayer;
    kernel::MaterialSlot materialSlot = kernel::MaterialSlot::None;
    face_image::UUID faceIdentifier{};
    std::size_t memberIndex = std::numeric_limits<std::size_t>::max();
    std::size_t faceIndex = std::numeric_limits<std::size_t>::max();
    std::size_t order = 0;
    bool boundedLayer = false;
    double centerU = 0, centerV = 0;
    double width = 0, height = 0;
    double angle = 0, opacity = 1;
    bool maskPresent = false;
    face_image::ResourceEnvelope maskEnvelope;
    bool maskUsesLuminance = false;
    bool maskInverted = false;
    UVTransform maskTransform;

    CapturedSource() = default;
    CapturedSource(const CapturedSource& other) { copyFrom(other); }
    CapturedSource& operator=(const CapturedSource& other) {
        if (this != &other) {
            reset();
            copyFrom(other);
        }
        return *this;
    }
    CapturedSource(CapturedSource&& other) noexcept { *this = std::move(other); }
    CapturedSource& operator=(CapturedSource&& other) noexcept {
        if (this != &other) {
            reset();
            fence = other.fence;
            envelope = std::move(other.envelope);
            origin = other.origin;
            materialSlot = other.materialSlot;
            faceIdentifier = other.faceIdentifier;
            memberIndex = other.memberIndex;
            faceIndex = other.faceIndex;
            order = other.order;
            boundedLayer = other.boundedLayer;
            centerU = other.centerU; centerV = other.centerV;
            width = other.width; height = other.height;
            angle = other.angle; opacity = other.opacity;
            maskPresent = other.maskPresent;
            maskEnvelope = std::move(other.maskEnvelope);
            maskUsesLuminance = other.maskUsesLuminance;
            maskInverted = other.maskInverted;
            maskTransform = other.maskTransform;
            originalBytesTicket = std::move(other.originalBytesTicket);
            workingBytesTicket = std::move(other.workingBytesTicket);
            maskOriginalBytesTicket =
                std::move(other.maskOriginalBytesTicket);
            maskWorkingBytesTicket =
                std::move(other.maskWorkingBytesTicket);
        }
        return *this;
    }
    void reset() noexcept {
        envelope = {};
        maskEnvelope = {};
        originalBytesTicket.reset();
        workingBytesTicket.reset();
        maskOriginalBytesTicket.reset();
        maskWorkingBytesTicket.reset();
        fence = {};
        origin = kernel::SourceOrigin::OrderedLayer;
        materialSlot = kernel::MaterialSlot::None;
        faceIdentifier = {};
        memberIndex = faceIndex = std::numeric_limits<std::size_t>::max();
        order = 0; boundedLayer = false;
        centerU = centerV = width = height = angle = 0;
        opacity = 1; maskPresent = false;
        maskUsesLuminance = maskInverted = false;
        maskTransform = {};
    }

private:
    void copyFrom(const CapturedSource& other) {
        using namespace decal_layer::bake::accounting;
        FailureSite active = FailureSite::SourceOriginalBytes;
        View activeView;
        try {
            if (other.originalBytesTicket.bytes != 0) {
                activeView = other.originalBytesTicket.view;
                if (!originalBytesTicket.acquire(activeView,
                        StorageDimension::EncodedTexture,
                        other.envelope.originalBytes.capacity(),
                        Retention::Retained, FailureSite::SourceOriginalBytes)
                    || !activeView.EnterAllocation(FailureSite::SourceOriginalBytes))
                    throw std::bad_alloc();
            }
            active = FailureSite::SourceWorkingBytes;
            if (other.workingBytesTicket.bytes != 0) {
                activeView = other.workingBytesTicket.view;
                if (!workingBytesTicket.acquire(activeView,
                        StorageDimension::EncodedTexture,
                        other.envelope.workingBytes.capacity(),
                        Retention::Retained, FailureSite::SourceWorkingBytes)
                    || !activeView.EnterAllocation(FailureSite::SourceWorkingBytes))
                    throw std::bad_alloc();
            }
            if (other.maskOriginalBytesTicket.bytes != 0) {
                active = FailureSite::SourceOriginalBytes;
                activeView = other.maskOriginalBytesTicket.view;
                if (!maskOriginalBytesTicket.acquire(activeView,
                        StorageDimension::EncodedTexture,
                        other.maskEnvelope.originalBytes.capacity(),
                        Retention::Retained, FailureSite::SourceOriginalBytes)
                    || !activeView.EnterAllocation(
                        FailureSite::SourceOriginalBytes))
                    throw std::bad_alloc();
            }
            if (other.maskWorkingBytesTicket.bytes != 0) {
                active = FailureSite::SourceWorkingBytes;
                activeView = other.maskWorkingBytesTicket.view;
                if (!maskWorkingBytesTicket.acquire(activeView,
                        StorageDimension::EncodedTexture,
                        other.maskEnvelope.workingBytes.capacity(),
                        Retention::Retained, FailureSite::SourceWorkingBytes)
                    || !activeView.EnterAllocation(
                        FailureSite::SourceWorkingBytes))
                    throw std::bad_alloc();
            }
            fence = other.fence;
            envelope = other.envelope;
            origin = other.origin;
            materialSlot = other.materialSlot;
            faceIdentifier = other.faceIdentifier;
            memberIndex = other.memberIndex;
            faceIndex = other.faceIndex;
            order = other.order;
            boundedLayer = other.boundedLayer;
            centerU = other.centerU; centerV = other.centerV;
            width = other.width; height = other.height;
            angle = other.angle; opacity = other.opacity;
            maskPresent = other.maskPresent;
            maskEnvelope = other.maskEnvelope;
            maskUsesLuminance = other.maskUsesLuminance;
            maskInverted = other.maskInverted;
            maskTransform = other.maskTransform;
        } catch (const std::bad_alloc&) {
            reset();
            activeView.AllocationFailed(active);
            throw;
        } catch (...) {
            reset();
            throw;
        }
    }
};

//! Complete operation-local observation of one concrete Common/PBR slot.
//! Absence is a value, not a missing witness. Present encoded bytes remain
//! owned once by the effective or shadowed CapturedSource.
struct RawSlotWitness final {
    retained_recipe::OwnerKey owner;
    std::size_t memberIndex = std::numeric_limits<std::size_t>::max();
    kernel::MaterialSlot slot = kernel::MaterialSlot::None;
    bool present = false;
    BindingFence fence;
    std::size_t originalBytes = 0;
    std::size_t workingBytes = 0;

    bool operator==(const RawSlotWitness& value) const noexcept {
        return owner == value.owner && memberIndex == value.memberIndex
            && slot == value.slot && present == value.present
            && fence == value.fence && originalBytes == value.originalBytes
            && workingBytes == value.workingBytes;
    }
};

struct CanonicalBindingWitness final {
    decal_layer::bake::accounting::Ticket bytesTicket;
    retained_recipe::OwnerKey owner;
    std::vector<std::uint8_t> bytes;

    bool operator==(const CanonicalBindingWitness& value) const noexcept {
        return owner == value.owner && bytes == value.bytes;
    }
};

//! Owns the admission ticket for the std::vector<CapturedSource> allocation
//! independently of the encoded-byte tickets carried by each source and the
//! later decoded kernel::SourceStorage allocation.  The ticket is declared
//! first so the vector releases its backing before the accounting view does.
struct CapturedSourceStorage final {
    decal_layer::bake::accounting::Ticket vectorTicket;
    decal_layer::bake::accounting::Ticket rawSlotsTicket;
    decal_layer::bake::accounting::Ticket canonicalBindingsTicket;
    std::vector<CapturedSource> values;
    std::vector<RawSlotWitness> rawSlots;
    std::vector<CanonicalBindingWitness> canonicalBindings;

    CapturedSourceStorage() = default;
    CapturedSourceStorage(const CapturedSourceStorage&) = delete;
    CapturedSourceStorage& operator=(const CapturedSourceStorage&) = delete;
    CapturedSourceStorage(CapturedSourceStorage&& other) noexcept {
        *this = std::move(other);
    }
    CapturedSourceStorage& operator=(CapturedSourceStorage&& other) noexcept {
        if (this != &other) {
            reset();
            vectorTicket = std::move(other.vectorTicket);
            rawSlotsTicket = std::move(other.rawSlotsTicket);
            canonicalBindingsTicket = std::move(other.canonicalBindingsTicket);
            values = std::move(other.values);
            rawSlots = std::move(other.rawSlots);
            canonicalBindings = std::move(other.canonicalBindings);
        }
        return *this;
    }
    ~CapturedSourceStorage() { reset(); }
    void reset() noexcept {
        std::vector<CapturedSource>().swap(values);
        std::vector<RawSlotWitness>().swap(rawSlots);
        std::vector<CanonicalBindingWitness>().swap(canonicalBindings);
        canonicalBindingsTicket.reset();
        rawSlotsTicket.reset();
        vectorTicket.reset();
    }
    void clear() noexcept { reset(); }
    bool empty() const noexcept { return values.empty(); }
    std::size_t size() const noexcept { return values.size(); }
    CapturedSource& operator[](std::size_t index) noexcept {
        return values[index];
    }
    const CapturedSource& operator[](std::size_t index) const noexcept {
        return values[index];
    }
    auto begin() noexcept { return values.begin(); }
    auto end() noexcept { return values.end(); }
    auto begin() const noexcept { return values.begin(); }
    auto end() const noexcept { return values.end(); }
};

#ifdef DEBUG
struct DebugSourceStorageEvidence final {
    bool captured = false;
    bool envelopeBytesEqual = false;
    bool identityValid = false;
    bool decoded = false;
    bool copyChargedSeparately = false;
    bool moveTransferred = false;
    bool workingDeniedBeforeEntry = false;
    bool originalDeniedBeforeEntry = false;
    bool vectorDeniedBeforeEntry = false;
    std::size_t originalBytes = 0;
    std::size_t workingBytes = 0;
    std::size_t vectorBytes = 0;
    Digest decodedPixels{};
};
struct DebugCaptureReadEvidence final {
    bool normalCaptured = false;
    bool envelopeBytesPreserved = false;
    bool bindingBytesPreserved = false;
    bool ticketsRetained = false;
    bool capturedVectorPreflighted = false;
    bool capturedVectorTicketRetained = false;
    bool deniedOverBudget = false;
    bool deniedBeforeBindingRead = false;
    bool deniedBeforeResourceRead = false;
};
struct DebugOrdinaryStagingEvidence final {
    bool prepareCommitPrepared = false;
    bool prepareCommitOverlapCharged = false;
    bool normalCommitted = false;
    bool prepareCancelPrepared = false;
    bool prepareCancelOverlapCharged = false;
    bool cancelReleased = false;
    bool denialPrepared = false;
    bool denialOverlapCharged = false;
    bool denialOverBudget = false;
    bool operationStableAtCommit = false;
    bool noAdoptionEntry = false;
    bool noPersistenceEntry = false;
    bool historyUnchanged = false;
};
enum class DebugPersistenceScenario : std::uint8_t {
    Transfer,
    AggregateRefusal,
    Settlement
};
struct DebugPersistenceEvidence final {
    bool prepared = false;
    bool committed = false;
    bool commandCommitted = false;
    bool faceImageTransferred = false;
    bool paintedBakeTransferred = false;
    bool noTransferBeforeSuccess = false;
    bool exactResourceBytes = false;
    bool exactBakeBytes = false;
    bool exactTransientBytes = false;
    bool finalBalanceZero = false;
    bool contextControlDestroyed = false;
    bool persistentReadback = false;
    bool unchangedAddsNoTransfer = false;
    bool undoRestored = false;
    bool redoRestored = false;
    bool aggregateExceeded = false;
    bool aggregateFirstFailure = false;
    bool refusedBeforeWrites = false;
    bool noPartialArtifact = false;
    bool historyUnchanged = false;
    bool manifestUnchanged = false;
    bool denialSitesSticky = false;
    bool transientSettled = false;
    bool abortRestored = false;
    bool cancelSettled = false;
    bool transferredNotDestroyed = false;
    std::size_t aggregateBefore = 0;
    std::size_t candidateDelta = 0;
    std::size_t aggregateLimit = 0;
    std::size_t resourceBytes = 0;
    std::size_t bakeBytes = 0;
    std::size_t transientBytes = 0;
    std::size_t transferEvents = 0;
    std::size_t destructionEvents = 0;
};
DebugCaptureReadEvidence DebugExerciseE4CaptureReadPreflight(
    const Handle(TDocStd_Document)& document,
    const asset_atlas::Key& key) noexcept;
DebugSourceStorageEvidence DebugExerciseE4SourceStorage(
    const Handle(TDocStd_Document)& document,
    const asset_atlas::Key& key) noexcept;
DebugOrdinaryStagingEvidence DebugExerciseE4OrdinaryStaging(
    const Handle(TDocStd_Document)& document,
    const asset_atlas::Key& key) noexcept;
DebugPersistenceEvidence DebugExerciseE4Persistence(
    const Handle(TDocStd_Document)& document,
    const asset_atlas::Key& key,
    DebugPersistenceScenario scenario) noexcept;
void DebugDenyNextSourceVectorReservation() noexcept;
std::array<std::size_t, 3> DebugTakeSourceVectorReservationObservation() noexcept;
void DebugDenyNextCurrentnessReservation(
    decal_layer::bake::accounting::FailureSite site) noexcept;
void DebugClearCurrentnessReservationDenial() noexcept;
#endif

struct Staging final {
    decal_layer::bake::accounting::Owner operation;
    decal_layer::bake::accounting::Ticket observedMembersTicket;
    decal_layer::bake::accounting::Ticket priorBakeResourcesTicket;
    decal_layer::bake::accounting::Ticket priorAtlasBytesTicket;
    asset_atlas::Definition atlas;
    std::vector<asset_atlas::MemberUVAssignment> assignments;
    std::vector<asset_atlas::Member> observedMembers;
    CapturedSourceStorage capturedSources;
    Definition bake;
    decal_layer::bake::accounting::Ticket outputVectorTicket;
    decal_layer::bake::accounting::Ticket evidenceTicket;
    std::vector<kernel::Output> outputs;
    kernel::Evidence evidence;
    std::vector<BakedResource> priorBakeResources;
    std::vector<std::uint8_t> priorAtlasBytes; // persisted SYEA/1 at Prepare
    bool unchanged = false;

    Staging() = default;
    Staging(const Staging&) = delete;
    Staging& operator=(const Staging&) = delete;
    Staging(Staging&& other) noexcept { *this = std::move(other); }
    Staging& operator=(Staging&& other) noexcept {
        if (this != &other) {
            reset();
            operation = std::move(other.operation);
            observedMembersTicket = std::move(other.observedMembersTicket);
            priorBakeResourcesTicket = std::move(other.priorBakeResourcesTicket);
            priorAtlasBytesTicket = std::move(other.priorAtlasBytesTicket);
            atlas = std::move(other.atlas);
            assignments = std::move(other.assignments);
            observedMembers = std::move(other.observedMembers);
            capturedSources = std::move(other.capturedSources);
            bake = std::move(other.bake);
            outputVectorTicket = std::move(other.outputVectorTicket);
            evidenceTicket = std::move(other.evidenceTicket);
            outputs = std::move(other.outputs);
            evidence = std::move(other.evidence);
            priorBakeResources = std::move(other.priorBakeResources);
            priorAtlasBytes = std::move(other.priorAtlasBytes);
            unchanged = other.unchanged;
            other.unchanged = false;
        }
        return *this;
    }
    ~Staging() { reset(); }
    void reset() noexcept {
        capturedSources.reset();
        std::vector<kernel::Output>().swap(outputs);
        outputVectorTicket.reset();
        evidenceTicket.reset();
        priorBakeResources.clear();
        priorAtlasBytes.clear();
        assignments.clear();
        observedMembers.clear();
        bake = {};
        atlas = {};
        evidence = {};
        unchanged = false;
        priorAtlasBytesTicket.reset();
        priorBakeResourcesTicket.reset();
        observedMembersTicket.reset();
        operation.reset();
    }
};

//! Immutable, operation-owned source authority for an export-only rebuild.
//! Unlike Staging this value carries no prior resources to remove and grants
//! no right to adopt resources or stage SYEA/SYEB records.
struct ExportCapture final {
    decal_layer::bake::accounting::Owner operation;
    //! The operation actually used by every capture/read/decode/bake stage.
    //! Ordinary callers retain `operation`; private builder callers borrow a
    //! view whose concrete ledger is dominated by PrivateDecalCapture.
    decal_layer::bake::accounting::View inheritedOperation;
    decal_layer::bake::accounting::Ticket savedAtlasTicket;
    decal_layer::bake::accounting::Ticket membersTicket;
    decal_layer::bake::accounting::Ticket observedMembersTicket;
    decal_layer::bake::accounting::Ticket savedBakeTicket;
    decal_layer::bake::accounting::Ticket canonicalAtlasBytesTicket;
    decal_layer::bake::accounting::Ticket canonicalBakeBytesTicket;
    asset_atlas::Definition savedAtlas;
    asset_atlas::Capture members;
    std::vector<asset_atlas::Member> observedMembers;
    CapturedSourceStorage sources;
    Definition savedBake;
    std::vector<std::uint8_t> canonicalAtlasBytes;
    std::vector<std::uint8_t> canonicalBakeBytes;

    ExportCapture() = default;
    ExportCapture(const ExportCapture&) = delete;
    ExportCapture& operator=(const ExportCapture&) = delete;
    ExportCapture(ExportCapture&& other) noexcept {
        *this = std::move(other);
    }
    ExportCapture& operator=(ExportCapture&& other) noexcept {
        if (this != &other) {
            reset();
            operation = std::move(other.operation);
            inheritedOperation = other.inheritedOperation;
            other.inheritedOperation = {};
            savedAtlasTicket = std::move(other.savedAtlasTicket);
            membersTicket = std::move(other.membersTicket);
            observedMembersTicket = std::move(other.observedMembersTicket);
            savedBakeTicket = std::move(other.savedBakeTicket);
            canonicalAtlasBytesTicket =
                std::move(other.canonicalAtlasBytesTicket);
            canonicalBakeBytesTicket =
                std::move(other.canonicalBakeBytesTicket);
            savedAtlas = std::move(other.savedAtlas);
            members = std::move(other.members);
            observedMembers = std::move(other.observedMembers);
            sources = std::move(other.sources);
            savedBake = std::move(other.savedBake);
            canonicalAtlasBytes = std::move(other.canonicalAtlasBytes);
            canonicalBakeBytes = std::move(other.canonicalBakeBytes);
        }
        return *this;
    }
    ~ExportCapture() { reset(); }
    void reset() noexcept {
        sources.reset();
        std::vector<std::uint8_t>().swap(canonicalAtlasBytes);
        std::vector<std::uint8_t>().swap(canonicalBakeBytes);
        std::vector<asset_atlas::Member>().swap(observedMembers);
        std::vector<BindingFence>().swap(savedBake.bindings);
        std::vector<BakedResource>().swap(savedBake.resources);
        savedBake = {};
        std::vector<asset_atlas::MemberCapture>().swap(members.members);
        members = {};
        std::vector<asset_atlas::Member>().swap(savedAtlas.members);
        std::vector<asset_atlas::Chart>().swap(savedAtlas.charts);
        std::vector<retained_finishing::MaterialResource>().swap(
            savedAtlas.resources);
        savedAtlas = {};
        canonicalAtlasBytesTicket.reset();
        canonicalBakeBytesTicket.reset();
        savedBakeTicket.reset();
        observedMembersTicket.reset();
        membersTicket.reset();
        savedAtlasTicket.reset();
        inheritedOperation = {};
        operation.reset();
    }
    decal_layer::bake::accounting::View operationView() const noexcept {
        return operation.valid() ? operation.view : inheritedOperation;
    }
};

//! Move-only currentness value. The persisted Definition is unchanged; its
//! two vector allocations remain charged to the one ordinary operation until
//! reset, destruction, an admitted clone, or a continuation transfer.
struct CurrentnessResult final {
    decal_layer::bake::accounting::Owner operation;
    decal_layer::bake::accounting::Ticket bindingsTicket;
    decal_layer::bake::accounting::Ticket resourcesTicket;
    Definition value;

    CurrentnessResult() = default;
    CurrentnessResult(const CurrentnessResult&) = delete;
    CurrentnessResult& operator=(const CurrentnessResult&) = delete;
    CurrentnessResult(CurrentnessResult&& other) noexcept {
        *this = std::move(other);
    }
    CurrentnessResult& operator=(CurrentnessResult&& other) noexcept {
        if (this != &other) {
            reset();
            operation = std::move(other.operation);
            bindingsTicket = std::move(other.bindingsTicket);
            resourcesTicket = std::move(other.resourcesTicket);
            value = std::move(other.value);
        }
        return *this;
    }
    ~CurrentnessResult() { reset(); }
    void reset() noexcept {
        std::vector<BindingFence>().swap(value.bindings);
        std::vector<BakedResource>().swap(value.resources);
        value = {};
        resourcesTicket.reset();
        bindingsTicket.reset();
        operation.reset();
    }
    const Definition& definition() const noexcept { return value; }

    bool clone(CurrentnessResult& output) const noexcept;
    bool cloneUsing(const CurrentnessResult& operationSource,
                    CurrentnessResult& output) const noexcept;
    bool admitContinuation(
        std::size_t bytes,
        decal_layer::bake::accounting::Ticket& ticket) const noexcept;
    void continueInto(
        Definition& output,
        decal_layer::bake::accounting::Owner& continuedOperation,
        decal_layer::bake::accounting::Ticket& continuedBindings,
        decal_layer::bake::accounting::Ticket& continuedResources) noexcept;
};

//! Operation-local authority for an owner that has no persisted SYEA/SYEB
//! pair. This is deliberately a separate type: it cannot be mistaken for a
//! saved atlas capture and carries no persistence/adoption capability.
struct TransientExportCapture final {
    decal_layer::bake::accounting::View operation;
    asset_atlas::Key atlas;
    asset_atlas::Member member;
    // Borrow the operation-owned frozen source vector. The builder retains
    // its ticket for the complete call; copying it here would be a second
    // physical allocation and a second set of encoded-byte reservations.
    const std::vector<CapturedSource>* sources = nullptr;
};

//! One shared owner for the layout/result storage emitted by the single
//! all-member BuildFinalAtlas call. ExportBake remains cheaply copyable for
//! per-member lookup while the physical arrays are charged exactly once.
struct ExportBakeStorage final {
    decal_layer::bake::accounting::Owner operation;
    decal_layer::bake::accounting::Ticket controlStorage;
    decal_layer::bake::accounting::Ticket bakeValueStorage;
    decal_layer::bake::accounting::Ticket outputVectorStorage;
    decal_layer::bake::accounting::Ticket evidenceStorage;
    asset_atlas::build::FinalAtlasAccounting finalAtlas;
    // Declared after FinalAtlasAccounting so the retained layout payload is
    // destroyed before the ticket that charges its physical vectors.
    asset_atlas::build::LayoutEvidence layout;
};

//! Complete detached painted result for one final layout. All applicable
//! roles are baked together from ExportCapture::sources and remain values
//! owned by the export operation.
struct ExportBake final {
    std::shared_ptr<ExportBakeStorage> storage;
    asset_atlas::Definition atlas;
    std::vector<asset_atlas::MemberUVAssignment> assignments;
    Definition bake;
    std::vector<kernel::Output> outputs;
    kernel::Evidence evidence;
};

Outcome Prepare(Staging& staging, const Handle(TDocStd_Document)& document,
                const asset_atlas::Key& key) noexcept;
Outcome Commit(Staging& staging, const Handle(TDocStd_Document)& document) noexcept;
void Cancel(Staging& staging) noexcept;
Outcome Currentness(const Handle(TDocStd_Document)& document,
                    const asset_atlas::Key& key,
                    CurrentnessResult* output = nullptr) noexcept;
Outcome Currentness(const Handle(TDocStd_Document)& document,
                    const asset_atlas::Key& key,
                    const decal_layer::bake::accounting::Owner& operation,
                    CurrentnessResult* output) noexcept;
Outcome CaptureForExport(ExportCapture& capture,
                         const Handle(TDocStd_Document)& document,
                         const asset_atlas::Key& key) noexcept;
//! Private capture overload. It never creates an ordinary operation owner;
//! every retained value is charged to and dominated by `operation`.
Outcome CaptureForExport(ExportCapture& capture,
                         const Handle(TDocStd_Document)& document,
                         const asset_atlas::Key& key,
                         const decal_layer::bake::accounting::View& operation)
    noexcept;
Outcome BakeForExport(const ExportCapture& capture,
                      asset_atlas::Definition finalAtlas,
                      std::vector<asset_atlas::MemberUVAssignment> assignments,
                      asset_atlas::build::LayoutEvidence&& layout,
                      std::shared_ptr<ExportBakeStorage> storage,
                      ExportBake& output) noexcept;
Outcome BakeForExport(const ExportCapture& capture,
                      const CapturedSourceStorage& productionSources,
                      asset_atlas::Definition finalAtlas,
                      std::vector<asset_atlas::MemberUVAssignment> assignments,
                      asset_atlas::build::LayoutEvidence&& layout,
                      std::shared_ptr<ExportBakeStorage> storage,
                      ExportBake& output) noexcept;
Outcome BuildAndBakeForExport(
    const ExportCapture& capture,
    const std::vector<asset_atlas::build::FinalMemberInput>& finalMembers,
    const asset_atlas::build::Settings& settings,
    ExportBake& output,
    std::string& diagnosis) noexcept;
Outcome BuildAndBakeForExport(
    const ExportCapture& capture,
    const CapturedSourceStorage& productionSources,
    const std::vector<asset_atlas::build::FinalMemberInput>& finalMembers,
    const asset_atlas::build::Settings& settings,
    ExportBake& output,
    std::string& diagnosis) noexcept;
Outcome BuildAndBakeTransientForExport(
    const TransientExportCapture& capture,
    const asset_atlas::build::FinalMemberInput& finalMember,
    const asset_atlas::build::Settings& settings,
    ExportBake& output,
    std::string& diagnosis) noexcept;
} // namespace core3d::painted_atlas_bake::owner
