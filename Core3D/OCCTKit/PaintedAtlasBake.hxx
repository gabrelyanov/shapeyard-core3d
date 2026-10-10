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
#include <new>
#include <vector>

namespace core3d::painted_atlas_bake::kernel {

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

inline bool EncodePNG(const Image& image, bool srgb,
                      std::vector<std::uint8_t>& png) noexcept {
    png.clear();
    try {
        if (!ImageValid(image)) return false;
        std::vector<std::uint8_t> scanlines;
        scanlines.reserve(std::size_t(image.height) * (1 + 4 * image.width));
        for (std::uint32_t y = 0; y < image.height; ++y) {
            scanlines.push_back(0);
            const auto first = image.rgba.begin() + std::size_t(y) * image.width * 4;
            scanlines.insert(scanlines.end(), first, first + std::size_t(image.width) * 4);
        }
        // One bounded zlib stream: fixed level and default strategy/memLevel,
        // so equal inputs deflate to identical bytes on this build.
        uLongf compressedLength = compressBound(uLong(scanlines.size()));
        std::vector<std::uint8_t> zlib(compressedLength);
        if (compress2(zlib.data(), &compressedLength, scanlines.data(),
                      uLong(scanlines.size()), Z_BEST_COMPRESSION) != Z_OK)
            return false;
        zlib.resize(compressedLength);
        png = {137,80,78,71,13,10,26,10};
        std::vector<std::uint8_t> ihdr;
        Big(ihdr, image.width); Big(ihdr, image.height);
        ihdr.insert(ihdr.end(), {8, 6, 0, 0, 0});
        Chunk(png, "IHDR", ihdr);
        if (srgb) Chunk(png, "sRGB", std::vector<std::uint8_t>{0});
        Chunk(png, "IDAT", zlib); Chunk(png, "IEND", {});
        return png.size() <= kMaximumEncodedImageBytes;
    } catch (...) { png.clear(); return false; }
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
                 Evidence& evidence) noexcept {
    outputs.clear(); evidence = {};
    try {
        if (atlas.resolutionTexels <= 0 || atlas.resolutionTexels > kMaximumBakedDimension
            || atlas.gutterTexels < 1 || atlas.gutterTexels > 8
            || layout.charts.empty() || layout.packed.triangles.empty()
            || layout.chartMembers.size() != layout.charts.size()) return false;
        for (const auto& source : sources) if (!Valid(source.fence) || !ImageValid(source.image))
            return false;
        const std::uint32_t size = std::uint32_t(atlas.resolutionTexels);
        std::vector<Role> roles;
        for (const auto& source : sources)
            if (std::find(roles.begin(), roles.end(), source.fence.role) == roles.end())
                roles.push_back(source.fence.role);
        std::sort(roles.begin(), roles.end(), [](Role a, Role b) {
            return std::uint8_t(a) < std::uint8_t(b);
        });
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
            output.rgba.assign(std::size_t(size) * size * 4, 0);
            if (role == Role::Normal)
                for (std::size_t i = 0; i < output.rgba.size(); i += 4) {
                    output.rgba[i] = 128; output.rgba[i + 1] = 128;
                    output.rgba[i + 2] = 255; output.rgba[i + 3] = 255;
                }
            std::vector<std::int32_t> owner(std::size_t(size) * size, -1);
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
                const Source* source = nullptr;
                for (const auto& candidate : sources)
                    if (candidate.fence.owner == atlas.members[memberIndex].owner
                        && candidate.fence.role == role) { source = &candidate; break; }
                if (!source) continue;
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
                    const auto uv = TransformUV(localU, localV, source->fence.transform);
                    double normalError = 0; std::array<double, 4> aimed;
                    const auto quantized = Quantize(Bilinear(source->image, uv[0], uv[1]),
                                                     role, normalError, aimed);
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
                        evidence.sample.sourceU = localU; evidence.sample.sourceV = localV;
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
                        entry.sourceU = localU; entry.sourceV = localV;
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
            Image encoded{size, size, output.rgba};
            const bool srgb = role == Role::BaseColor || role == Role::Emissive;
            if (!EncodePNG(encoded, srgb, output.png)
                || !ImageHash(output.png, output.descriptor.content)) return false;
            // Resource identity is stable and role-bound, while content stays
            // the digest of the exact PNG bytes.  Two semantic roles may emit
            // equal background-heavy PNGs, but they must remain distinct
            // resources under the fail-closed SYEB validator.
            std::vector<std::uint8_t> identityBytes{
                'S', 'Y', 'E', 'B', '/', '1', std::uint8_t(role)};
            identityBytes.insert(identityBytes.end(), output.descriptor.content.begin(),
                                 output.descriptor.content.end());
            if (!retained_solid::Hash(identityBytes, output.descriptor.identity)) return false;
            outputs.push_back(std::move(output));
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
    } catch (...) { outputs.clear(); evidence = {}; return false; }
}
} // namespace core3d::painted_atlas_bake::kernel

namespace core3d::painted_atlas_bake::owner {
enum class Outcome : std::uint8_t {
    Prepared, Committed, Refused, StaleSource, StaleBinding, MissingResource,
    ForeignResource, OverBudget, UnsupportedSurface, OwnerMismatch, Busy,
    Malformed, PersistenceFailure, Absent
};

struct CapturedSource final {
    BindingFence fence;
    decal_layer::bake::accounting::Ticket originalBytesTicket;
    decal_layer::bake::accounting::Ticket workingBytesTicket;
    face_image::ResourceEnvelope envelope;

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
            originalBytesTicket = std::move(other.originalBytesTicket);
            workingBytesTicket = std::move(other.workingBytesTicket);
        }
        return *this;
    }
    void reset() noexcept {
        envelope = {};
        originalBytesTicket.reset();
        workingBytesTicket.reset();
        fence = {};
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
            fence = other.fence;
            envelope = other.envelope;
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
DebugSourceStorageEvidence DebugExerciseE4SourceStorage(
    const Handle(TDocStd_Document)& document,
    const asset_atlas::Key& key) noexcept;
void DebugDenyNextSourceVectorReservation() noexcept;
std::array<std::size_t, 3> DebugTakeSourceVectorReservationObservation() noexcept;
#endif

struct Staging final {
    decal_layer::bake::accounting::Owner operation;
    asset_atlas::Definition atlas;
    std::vector<asset_atlas::MemberUVAssignment> assignments;
    std::vector<asset_atlas::Member> observedMembers;
    std::vector<CapturedSource> capturedSources;
    Definition bake;
    std::vector<kernel::Output> outputs;
    kernel::Evidence evidence;
    std::vector<BakedResource> priorBakeResources;
    std::vector<std::uint8_t> priorAtlasBytes; // persisted SYEA/1 at Prepare
    bool unchanged = false;
};

//! Immutable, operation-owned source authority for an export-only rebuild.
//! Unlike Staging this value carries no prior resources to remove and grants
//! no right to adopt resources or stage SYEA/SYEB records.
struct ExportCapture final {
    decal_layer::bake::accounting::Owner operation;
    asset_atlas::Definition savedAtlas;
    asset_atlas::Capture members;
    std::vector<asset_atlas::Member> observedMembers;
    std::vector<CapturedSource> sources;
    Definition savedBake;
    std::vector<std::uint8_t> canonicalAtlasBytes;
};

//! Operation-local authority for an owner that has no persisted SYEA/SYEB
//! pair. This is deliberately a separate type: it cannot be mistaken for a
//! saved atlas capture and carries no persistence/adoption capability.
struct TransientExportCapture final {
    decal_layer::bake::accounting::View operation;
    asset_atlas::Key atlas;
    asset_atlas::Member member;
    std::vector<CapturedSource> sources;
};

//! Complete detached painted result for one final layout. All applicable
//! roles are baked together from ExportCapture::sources and remain values
//! owned by the export operation.
struct ExportBake final {
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
                    Definition* output = nullptr) noexcept;
Outcome CaptureForExport(ExportCapture& capture,
                         const Handle(TDocStd_Document)& document,
                         const asset_atlas::Key& key) noexcept;
Outcome BakeForExport(const ExportCapture& capture,
                      const asset_atlas::Definition& finalAtlas,
                      const std::vector<asset_atlas::MemberUVAssignment>& assignments,
                      const asset_atlas::build::LayoutEvidence& layout,
                      ExportBake& output) noexcept;
Outcome BuildAndBakeForExport(
    const ExportCapture& capture,
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
