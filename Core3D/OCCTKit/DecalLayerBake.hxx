#pragma once

// Pure deterministic E4 CPU derivative builder.  Callers must provide fully
// validated/current resource pixels, exact receiver receipts, charged
// occluder visibility, and final emitted geometry/UVs.  This module neither
// resolves nor invents a face receipt and never mutates a document.
#include "DecalLayerPersistence.hxx"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace core3d::decal_layer::bake {

inline constexpr std::uint32_t kAlgorithmVersion = 1;
inline constexpr std::uint32_t kColorVersion = 1;
inline constexpr std::uint32_t kEncoderVersion = 1;
inline constexpr std::size_t kMaximumOutputBytes = 32U * 1024U * 1024U;
inline constexpr std::uint32_t kMaximumDimension = 8192;
inline constexpr std::uint64_t kMaximumPixels = 16'777'216;

//! Small non-owning access to the builder-owned accounting context. The
//! concrete ledger, topology component and DEBUG exercises stay private to
//! OcctSceneSnapshotBuilder.mm. An invalid view always refuses; it is never a
//! permission to allocate without the operation owner.
namespace accounting {
enum class StorageDimension : std::uint8_t {
    SnapshotNumeric,
    EncodedTexture,
    PrivateStorage,
    Count
};
enum class Retention : std::uint8_t { Scratch, Retained };
enum class Job : std::uint8_t {
    AtlasRoles,
    SourcePixels,
    MaskPixels,
    Gutter,
    Coverage,
    Occlusion,
    Encoder,
    Publication,
    Debug,
    Count
};
enum class FailureSite : std::uint8_t {
    None,
    ContextControl,
    ProtectedSceneCopy,
    StorageReserve,
    StorageRelease,
    CheckedProduct,
    JobSeal,
    JobConsume,
    Cancellation,
    DebugExactLimit,
    DebugOneUnit,
    DebugCheckedProduct,
    DebugDuplicateJob,
    DebugJobLimit,
    DebugCopyOne,
    DebugCopyTwo,
    DebugShared,
    DebugMove,
    DebugScratch,
    DebugWork
};

struct View final {
    void* context = nullptr;
    bool (*reserve)(void*, StorageDimension, std::size_t, Retention,
                    FailureSite) noexcept = nullptr;
    void (*release)(void*, StorageDimension, std::size_t,
                    Retention) noexcept = nullptr;
    bool (*checkedProduct)(void*, std::size_t, std::size_t, std::size_t,
                           std::size_t&, FailureSite) noexcept = nullptr;
    bool (*sealJob)(void*, Job, std::size_t, FailureSite) noexcept = nullptr;
    bool (*consumeJob)(void*, Job, std::size_t, FailureSite) noexcept = nullptr;
    bool (*recheck)(void*, FailureSite) noexcept = nullptr;
    void (*cancel)(void*, FailureSite) noexcept = nullptr;

    bool valid() const noexcept {
        return context && reserve && release && checkedProduct && sealJob
            && consumeJob && recheck && cancel;
    }
    bool Reserve(StorageDimension dimension, std::size_t bytes,
                 Retention retention, FailureSite site) const noexcept {
        return valid()
            && reserve(context, dimension, bytes, retention, site);
    }
    void Release(StorageDimension dimension, std::size_t bytes,
                 Retention retention) const noexcept {
        if (valid()) release(context, dimension, bytes, retention);
    }
    bool CheckedProduct(std::size_t first, std::size_t second,
                        std::size_t third, std::size_t& product,
                        FailureSite site) const noexcept {
        product = 0;
        return valid()
            && checkedProduct(context, first, second, third, product, site);
    }
    bool SealJob(Job job, std::size_t admittedUnits,
                 FailureSite site) const noexcept {
        return valid() && sealJob(context, job, admittedUnits, site);
    }
    bool ConsumeJob(Job job, std::size_t units,
                    FailureSite site) const noexcept {
        return valid() && consumeJob(context, job, units, site);
    }
    bool Recheck(FailureSite site) const noexcept {
        return valid() && recheck(context, site);
    }
    void Cancel(FailureSite site) const noexcept {
        if (valid()) cancel(context, site);
    }
};

struct Ticket final {
    View view;
    StorageDimension dimension = StorageDimension::PrivateStorage;
    std::size_t bytes = 0;
    Retention retention = Retention::Scratch;

    Ticket() = default;
    Ticket(const Ticket&) = delete;
    Ticket& operator=(const Ticket&) = delete;
    Ticket(Ticket&& other) noexcept
        : view(other.view), dimension(other.dimension), bytes(other.bytes),
          retention(other.retention) {
        other.view = {};
        other.bytes = 0;
    }
    Ticket& operator=(Ticket&& other) noexcept {
        if (this != &other) {
            reset();
            view = other.view;
            dimension = other.dimension;
            bytes = other.bytes;
            retention = other.retention;
            other.view = {};
            other.bytes = 0;
        }
        return *this;
    }
    ~Ticket() { reset(); }

    bool acquire(const View& candidate, StorageDimension requestedDimension,
                 std::size_t count, Retention requestedRetention,
                 FailureSite site) noexcept {
        reset();
        if (!candidate.Reserve(requestedDimension, count,
                               requestedRetention, site)) return false;
        view = candidate;
        dimension = requestedDimension;
        bytes = count;
        retention = requestedRetention;
        return true;
    }
    bool acquireProduct(const View& candidate,
                        StorageDimension requestedDimension,
                        std::size_t first, std::size_t second,
                        std::size_t third, Retention requestedRetention,
                        FailureSite site) noexcept {
        std::size_t count = 0;
        return candidate.CheckedProduct(
                first, second, third, count, site)
            && acquire(candidate, requestedDimension, count,
                       requestedRetention, site);
    }
    void reset() noexcept {
        if (bytes != 0)
            view.Release(dimension, bytes, retention);
        view = {};
        bytes = 0;
    }
};
} // namespace accounting

struct Raster final {
    std::uint32_t width = 0, height = 0;
    // Canonical top-to-bottom, left-to-right straight RGBA8 samples.
    std::vector<std::uint8_t> rgba;
};
struct UV final { double u = 0.0, v = 0.0; };
struct Point final { double x = 0.0, y = 0.0; };
struct CoverageSample final {
    std::uint32_t x = 0, y = 0;
    ReceiverReceipt receiver;
    // Every affected texel has its own geometric verdict. `conclusive` is
    // false for tied/grazing/over-budget intersections and always refuses.
    bool conclusive = false;
    bool affected = false;
    bool selectedReceiver = false;
    bool frontFacing = false;
    bool insideTrim = false;
    bool nearestHit = false;
};
struct Triangle final {
    std::array<Point, 3> outputPixels;
    std::array<UV, 3> layerUV;
    ReceiverReceipt receiver;
    // Exact row-major proof for every top-left-owned output sample in this
    // triangle. Bake independently enumerates coverage and rejects omissions,
    // extras and duplicates instead of trusting a triangle-wide boolean.
    std::vector<CoverageSample> coverage;
};
struct ResolvedImage final {
    ImageRef reference;
    face_image::ResourceEnvelope envelope;
    Raster pixels;
    // Set only by the Objective-C++ adapter after measuring both carried
    // encodings and decoding the actual working bytes.
    bool originalMeasured = false;
    bool workingMeasured = false;
    bool workingHasAlpha = false;
};
struct ResolvedLayer final {
    Layer intent;
    ResolvedImage image;
    std::optional<ResolvedImage> mask;
    std::vector<Triangle> triangles;
    bool completeOccludersProved = false;
    bool boundaryProved = false;
    bool footprintContained = false;
};
struct Input final {
    Raster baseColor;
    std::vector<ResolvedLayer> layers; // exact persisted order
    std::vector<std::uint8_t> canonicalLayers;
    Digest sourceProof{};
    Digest finalGeometryUVProof{};
    Digest occluderProof{};
    Digest effectiveAppearanceProof{};
    Digest tangentProof{};
    Digest atlasProof{};
    Digest occurrenceProof{};
    Digest inputKey{};
    // Required inherited operation view for private/ordinary production.
    // S6 connects the existing ordinary entry; invalid never means uncharged.
    accounting::View operationLedger;
};
struct Output final {
    Raster pixels;
    std::vector<std::uint8_t> png;
    Digest pngDigest{};
};

//! Bounded ephemeral publication handed from the final-geometry baker to all
//! preview/export consumers. It contains no OCAF label, lease or mutation
//! grant. Receiver receipts are re-resolved by each consumer; face ordinals
//! are intentionally absent from this cache.
struct PublishedFace final {
    ReceiverReceipt receiver;
    std::uint32_t pixelWidth = 0;
    std::uint32_t pixelHeight = 0;
    std::vector<std::uint8_t> png;
    Digest pngDigest{};
};
struct PublishedDerivative final {
    OwnerKey owner{};
    Digest layerProof{};
    Digest sourceProof{};
    std::uint64_t geometryRevision = 0;
    Digest finalGeometryUVProof{};
    Digest occluderProof{};
    Digest inputKey{};
    Digest bakeKey{};
    std::vector<PublishedFace> faces;
};
enum class Status : std::uint8_t {
    Baked, Malformed, Stale, Unsupported, VisibilityInconclusive, Budget
};

namespace publication {
inline constexpr std::size_t kMaximumEntries = 128;
inline constexpr std::size_t kMaximumResidentPNGBytes = 64U * 1024U * 1024U;
struct State final {
    std::mutex mutex;
    std::unordered_map<std::string, PublishedDerivative> byInput;
    std::size_t residentBytes = 0;
};
inline State& SharedState() { static State state; return state; }
inline bool BindBakeKey(PublishedDerivative& value) noexcept {
    value.bakeKey = {};
    try {
        face_image::detail::Writer writer;
        writer.raw(reinterpret_cast<const std::uint8_t*>("SYDB\1\0\0\0"), 8);
        writer.integer(kAlgorithmVersion, 4); writer.integer(kColorVersion, 4);
        writer.integer(kEncoderVersion, 4); writer.integer(0, 4);
        writer.raw(value.owner.document); writer.raw(value.owner.entity);
        writer.raw(value.owner.definition); writer.raw(value.layerProof);
        writer.raw(value.sourceProof); writer.integer(value.geometryRevision, 8);
        writer.raw(value.finalGeometryUVProof); writer.raw(value.occluderProof);
        writer.raw(value.inputKey);
        writer.integer(value.faces.size(), 2); writer.integer(0, 2);
        for (const PublishedFace& face : value.faces) {
            writer.raw(face.receiver.face); writer.raw(face.receiver.selectorProof);
            writer.integer(face.pixelWidth, 4); writer.integer(face.pixelHeight, 4);
            writer.raw(face.pngDigest);
        }
        return writer.ok && writer.bytes.size() <= kMaximumBytes
            && face_image::HashFaceImageBytes(writer.bytes, value.bakeKey)
            && Nonzero(value.bakeKey);
    } catch (...) { value.bakeKey = {}; return false; }
}
inline std::string OwnerKeyText(const OwnerKey& owner) {
    static constexpr char digits[] = "0123456789abcdef";
    std::string value; value.reserve(96);
    for (const UUID* part : {&owner.document, &owner.entity, &owner.definition})
        for (std::uint8_t byte : *part) {
            value.push_back(digits[byte >> 4]); value.push_back(digits[byte & 15]);
        }
    return value;
}
inline std::string CacheKeyText(const OwnerKey& owner, const Digest& inputKey) {
    static constexpr char digits[] = "0123456789abcdef";
    std::string value = OwnerKeyText(owner); value.reserve(160);
    for (std::uint8_t byte : inputKey) {
        value.push_back(digits[byte >> 4]); value.push_back(digits[byte & 15]);
    }
    return value;
}
inline bool Valid(const PublishedDerivative& value) noexcept {
    try {
        if (!retained_recipe::Valid(value.owner) || !Nonzero(value.layerProof)
            || !Nonzero(value.sourceProof) || value.geometryRevision == 0
            || !Nonzero(value.finalGeometryUVProof) || !Nonzero(value.occluderProof)
            || !Nonzero(value.inputKey)
            || !Nonzero(value.bakeKey) || value.faces.empty()
            || value.faces.size() > kMaximumLayers) return false;
        std::size_t bytes = 0;
        for (std::size_t index = 0; index < value.faces.size(); ++index) {
            const PublishedFace& face = value.faces[index]; Digest digest{};
            if (!core3d::decal_layer::Valid(face.receiver) || face.pixelWidth == 0
                || face.pixelHeight == 0 || face.pixelWidth > kMaximumDimension
                || face.pixelHeight > kMaximumDimension
                || std::uint64_t(face.pixelWidth) * face.pixelHeight > kMaximumPixels
                || face.png.empty() || face.png.size() > kMaximumOutputBytes
                || !face_image::HashFaceImageBytes(face.png, digest)
                || digest != face.pngDigest
                || bytes > kMaximumResidentPNGBytes - face.png.size()) return false;
            bytes += face.png.size();
            for (std::size_t other = index + 1; other < value.faces.size(); ++other)
                if (face.receiver.face == value.faces[other].receiver.face
                    && face.receiver.selectorProof
                        == value.faces[other].receiver.selectorProof) return false;
        }
        PublishedDerivative canonical = value;
        const Digest supplied = canonical.bakeKey;
        return BindBakeKey(canonical) && canonical.bakeKey == supplied;
    } catch (...) { return false; }
}

//! Operation-owned destination for the complete set of private derivatives.
//! It owns every retained PNG across all owners and has no path to SharedState.
//! The ordinary resident/entry ceilings apply once to the whole operation:
//! adding another owner never creates a fresh allowance.
struct PrivateDestination final {
    std::vector<PublishedDerivative> values;
    std::size_t residentBytes = 0;

    PrivateDestination() = default;
    PrivateDestination(const PrivateDestination&) = delete;
    PrivateDestination& operator=(const PrivateDestination&) = delete;
    PrivateDestination(PrivateDestination&&) = default;
    PrivateDestination& operator=(PrivateDestination&&) = default;

    void clear() noexcept {
        values.clear();
        residentBytes = 0;
    }
};

inline bool Store(PrivateDestination& destination,
                  const PublishedDerivative& value) noexcept {
    try {
        if (!Valid(value) || destination.values.size() >= kMaximumEntries)
            return false;
        std::size_t incoming = 0;
        for (const PublishedFace& face : value.faces) {
            if (incoming > kMaximumResidentPNGBytes - face.png.size())
                return false;
            incoming += face.png.size();
        }
        if (destination.residentBytes
                > kMaximumResidentPNGBytes - incoming)
            return false;
        for (const PublishedDerivative& existing : destination.values)
            if (existing.inputKey == value.inputKey)
                return false;
        destination.values.push_back(value);
        destination.residentBytes += incoming;
        return true;
    } catch (...) {
        destination.clear();
        return false;
    }
}

inline bool Lookup(const PrivateDestination& destination,
                   const OwnerKey& owner,
                   const Digest& inputKey,
                   PublishedDerivative& output) noexcept {
    output = {};
    try {
        if (!Nonzero(inputKey)) return false;
        const PublishedDerivative* found = nullptr;
        for (const PublishedDerivative& value : destination.values) {
            if (!(value.owner == owner) || value.inputKey != inputKey)
                continue;
            if (found || !Valid(value)) return false;
            found = &value;
        }
        if (!found) return false;
        output = *found;
        return true;
    } catch (...) {
        output = {};
        return false;
    }
}

inline bool Store(const PublishedDerivative& value) noexcept {
    try {
        if (!Valid(value)) return false;
        const std::string key = CacheKeyText(value.owner, value.inputKey);
        std::size_t incoming = 0;
        for (const PublishedFace& face : value.faces) incoming += face.png.size();
        State& state = SharedState(); std::lock_guard<std::mutex> lock(state.mutex);
        const auto found = state.byInput.find(key);
        std::size_t previous = 0;
        if (found != state.byInput.end())
            for (const PublishedFace& face : found->second.faces) previous += face.png.size();
        if (found == state.byInput.end()
            && state.byInput.size() >= kMaximumEntries)
            return false;
        if (state.residentBytes < previous
            || state.residentBytes - previous > kMaximumResidentPNGBytes - incoming)
            return false;
        state.residentBytes = state.residentBytes - previous + incoming;
        state.byInput[key] = value;
        return true;
    } catch (...) { return false; }
}
inline bool Lookup(const OwnerKey& owner, const Digest& inputKey,
                   PublishedDerivative& output) noexcept {
    output = {};
    try {
        State& state = SharedState(); std::lock_guard<std::mutex> lock(state.mutex);
        if (!Nonzero(inputKey)) return false;
        const auto found = state.byInput.find(CacheKeyText(owner, inputKey));
        if (found == state.byInput.end() || !Valid(found->second)
            || found->second.inputKey != inputKey) return false;
        output = found->second; return true;
    } catch (...) { output = {}; return false; }
}
inline void Evict(const OwnerKey& owner) noexcept {
    try {
        State& state = SharedState(); std::lock_guard<std::mutex> lock(state.mutex);
        const std::string prefix = OwnerKeyText(owner);
        for (auto found = state.byInput.begin(); found != state.byInput.end();) {
            if (found->first.compare(0, prefix.size(), prefix) != 0) {
                ++found; continue;
            }
            for (const PublishedFace& face : found->second.faces)
                state.residentBytes -= face.png.size();
            found = state.byInput.erase(found);
        }
    } catch (...) {}
}
#ifdef DEBUG
//! Bounded value-only cache census for native export tests. Cache keys are
//! already derived UUID/digest text; no labels, document handles or mutable
//! publication values cross this boundary.
struct DebugCacheInventory final {
    std::vector<std::string> keys;
    std::size_t residentBytes = 0;
};

inline DebugCacheInventory DebugInventory() noexcept {
    DebugCacheInventory result;
    try {
        State& state = SharedState();
        std::lock_guard<std::mutex> lock(state.mutex);
        result.residentBytes = state.residentBytes;
        result.keys.reserve(state.byInput.size());
        for (const auto& entry : state.byInput)
            result.keys.push_back(entry.first);
        std::sort(result.keys.begin(), result.keys.end());
    } catch (...) {
        result = {};
    }
    return result;
}

inline void DebugEvictAll() noexcept {
    try {
        State& state = SharedState();
        std::lock_guard<std::mutex> lock(state.mutex);
        state.byInput.clear();
        state.residentBytes = 0;
    } catch (...) {}
}
#endif
} // namespace publication

inline bool Valid(const Raster& value) noexcept {
    return value.width != 0 && value.height != 0
        && value.width <= kMaximumDimension && value.height <= kMaximumDimension
        && std::uint64_t(value.width) * value.height <= kMaximumPixels
        && value.rgba.size() == std::size_t(value.width) * value.height * 4;
}
inline bool SameImageRef(const ImageRef& left,
                         const ImageRef& right) noexcept {
    return left.resource == right.resource
        && left.normalizedContent == right.normalizedContent
        && left.originalContent == right.originalContent
        && left.provenance == right.provenance
        && left.producerVersion == right.producerVersion
        && left.mediaType == right.mediaType
        && left.widthTexels == right.widthTexels
        && left.heightTexels == right.heightTexels
        && left.role == right.role && left.colorSpace == right.colorSpace
        && left.alpha == right.alpha;
}
inline bool Valid(const ResolvedImage& value,
                  const ImageRef& expected) noexcept {
    try {
        face_image::Refusal refusal{};
        Digest original{}, working{};
        return SameImageRef(value.reference, expected)
            && image_contract::Supported(value.reference)
            && face_image::Valid(value.envelope, refusal)
            && refusal == face_image::Refusal::None
            && value.envelope.resource == value.reference.resource
            && value.envelope.originalContent
                == value.reference.originalContent
            && value.envelope.workingContent
                == value.reference.normalizedContent
            && value.envelope.provenance == value.reference.provenance
            && value.envelope.workingFormat == value.reference.mediaType
            && value.envelope.workingWidthTexels
                == value.reference.widthTexels
            && value.envelope.workingHeightTexels
                == value.reference.heightTexels
            && value.envelope.alpha == value.reference.alpha
            && value.originalMeasured && value.workingMeasured
            && face_image::HashFaceImageBytes(
                value.envelope.originalBytes, original)
            && face_image::HashFaceImageBytes(
                value.envelope.workingBytes, working)
            && original == value.envelope.originalContent
            && working == value.envelope.workingContent
            && Valid(value.pixels)
            && value.pixels.width == value.reference.widthTexels
            && value.pixels.height == value.reference.heightTexels
            && (value.reference.alpha
                    != face_image::AlphaInterpretation::Straight
                || (value.reference.mediaType
                        == face_image::ImageEncoding::PNG
                    && value.workingHasAlpha));
    } catch (...) { return false; }
}
inline bool RasterDigest(const Raster& value, Digest& output) noexcept {
    output = {};
    return Valid(value)
        && face_image::HashFaceImageBytes(value.rgba, output)
        && Nonzero(output);
}
inline bool ResolvedImageDigest(const ResolvedImage& value,
                                Digest& output) noexcept {
    output = {};
    try {
        std::vector<std::uint8_t> envelope;
        Digest envelopeDigest{}, pixelsDigest{};
        face_image::detail::Writer writer;
        if (!face_image::Encode(value.envelope, envelope)
            || !face_image::HashFaceImageBytes(envelope, envelopeDigest)
            || !RasterDigest(value.pixels, pixelsDigest)) return false;
        writer.raw(reinterpret_cast<const std::uint8_t*>("E4RI"), 4);
        writer.integer(1, 4); writer.raw(envelopeDigest);
        writer.raw(pixelsDigest);
        writer.integer(value.originalMeasured ? 1 : 0, 1);
        writer.integer(value.workingMeasured ? 1 : 0, 1);
        writer.integer(value.workingHasAlpha ? 1 : 0, 1);
        writer.integer(0, 1);
        return writer.ok
            && face_image::HashFaceImageBytes(writer.bytes, output)
            && Nonzero(output);
    } catch (...) { output = {}; return false; }
}
inline bool ComputeInputKey(const Input& input, Digest& output) noexcept {
    output = {};
    try {
        if (!Valid(input.baseColor) || input.layers.empty()
            || input.layers.size() > kMaximumLayers
            || input.canonicalLayers.empty()
            || input.canonicalLayers.size() > kMaximumBytes
            || !Nonzero(input.sourceProof)
            || !Nonzero(input.finalGeometryUVProof)
            || !Nonzero(input.occluderProof)
            || !Nonzero(input.effectiveAppearanceProof)
            || !Nonzero(input.tangentProof) || !Nonzero(input.atlasProof)
            || !Nonzero(input.occurrenceProof)) return false;
        Digest baseDigest{}, layersDigest{};
        if (!RasterDigest(input.baseColor, baseDigest)
            || !face_image::HashFaceImageBytes(
                input.canonicalLayers, layersDigest)
            || !Nonzero(layersDigest)) return false;
        face_image::detail::Writer writer;
        writer.raw(reinterpret_cast<const std::uint8_t*>("E4IK"), 4);
        writer.integer(1, 4); writer.integer(kAlgorithmVersion, 4);
        writer.integer(kColorVersion, 4); writer.integer(kEncoderVersion, 4);
        writer.raw(input.sourceProof); writer.raw(input.finalGeometryUVProof);
        writer.raw(input.occluderProof);
        writer.raw(input.effectiveAppearanceProof); writer.raw(input.tangentProof);
        writer.raw(input.atlasProof); writer.raw(input.occurrenceProof);
        writer.raw(baseDigest); writer.raw(layersDigest);
        writer.integer(input.baseColor.width, 4);
        writer.integer(input.baseColor.height, 4);
        writer.integer(input.layers.size(), 4);
        for (const ResolvedLayer& layer : input.layers) {
            Digest imageDigest{}, maskDigest{};
            if (!Valid(layer.intent) || !Valid(layer.image, layer.intent.image)
                || !ResolvedImageDigest(layer.image, imageDigest)
                || layer.triangles.empty() || !layer.completeOccludersProved
                || !layer.boundaryProved
                || (layer.intent.placement.edgePolicy
                        == EdgePolicy::RejectCrossing
                    && !layer.footprintContained)) return false;
            if (layer.intent.mask.present) {
                if (!layer.mask.has_value()
                    || !Valid(*layer.mask, layer.intent.mask.image)
                    || !ResolvedImageDigest(*layer.mask, maskDigest)) return false;
            } else if (layer.mask.has_value()) return false;
            writer.raw(layer.intent.identifier); writer.raw(imageDigest);
            writer.raw(maskDigest);
            writer.integer(layer.completeOccludersProved ? 1 : 0, 1);
            writer.integer(layer.boundaryProved ? 1 : 0, 1);
            writer.integer(layer.footprintContained ? 1 : 0, 1);
            writer.integer(0, 1); writer.integer(layer.triangles.size(), 4);
            for (const Triangle& triangle : layer.triangles) {
                if (!core3d::decal_layer::Valid(triangle.receiver)
                    || triangle.coverage.empty()) return false;
                writer.raw(triangle.receiver.face);
                writer.raw(triangle.receiver.selectorProof);
                for (const Point& point : triangle.outputPixels) {
                    writer.real(point.x); writer.real(point.y);
                }
                for (const UV& uv : triangle.layerUV) {
                    writer.real(uv.u); writer.real(uv.v);
                }
                writer.integer(triangle.coverage.size(), 8);
                for (const CoverageSample& sample : triangle.coverage) {
                    writer.integer(sample.x, 4); writer.integer(sample.y, 4);
                    writer.raw(sample.receiver.face);
                    writer.raw(sample.receiver.selectorProof);
                    std::uint8_t flags = sample.conclusive ? 1 : 0;
                    flags |= sample.affected ? 2 : 0;
                    flags |= sample.selectedReceiver ? 4 : 0;
                    flags |= sample.frontFacing ? 8 : 0;
                    flags |= sample.insideTrim ? 16 : 0;
                    flags |= sample.nearestHit ? 32 : 0;
                    writer.integer(flags, 1);
                }
            }
        }
        return writer.ok && writer.bytes.size() <= kMaximumBytes
            && face_image::HashFaceImageBytes(writer.bytes, output)
            && Nonzero(output);
    } catch (...) { output = {}; return false; }
}
inline bool BindInputKey(Input& input) noexcept {
    input.inputKey = {};
    return ComputeInputKey(input, input.inputKey);
}
inline bool ValidInputKey(const Input& input) noexcept {
    Digest observed{};
    return Nonzero(input.inputKey) && ComputeInputKey(input, observed)
        && observed == input.inputKey;
}
inline double SRGBToLinear(std::uint8_t sample) noexcept {
    const double value = double(sample) / 255.0;
    return value <= 0.04045 ? value / 12.92
        : std::pow((value + 0.055) / 1.055, 2.4);
}
inline std::uint8_t LinearToSRGB(double value) noexcept {
    value = std::clamp(value, 0.0, 1.0);
    const double encoded = value <= 0.0031308 ? value * 12.92
        : 1.055 * std::pow(value, 1.0 / 2.4) - 0.055;
    return std::uint8_t(std::clamp(std::floor(encoded * 255.0 + 0.5), 0.0, 255.0));
}
inline std::uint8_t LinearByte(double value) noexcept {
    return std::uint8_t(std::clamp(std::floor(std::clamp(value, 0.0, 1.0)
        * 255.0 + 0.5), 0.0, 255.0));
}
inline double Wrap(double coordinate, face_image::Wrap mode) noexcept {
    switch (mode) {
        case face_image::Wrap::ClampToEdge:
            return std::clamp(coordinate, 0.0, 1.0);
        case face_image::Wrap::Repeat:
            return coordinate - std::floor(coordinate);
        case face_image::Wrap::MirroredRepeat: {
            const double whole = std::floor(coordinate);
            const double part = coordinate - whole;
            const auto parity = std::int64_t(whole) & 1;
            return parity == 0 ? part : 1.0 - part;
        }
    }
    return std::numeric_limits<double>::quiet_NaN();
}
inline UV Transform(UV value, const face_image::UVTransform& transform) noexcept {
    const double angle = transform.rotationDegrees * 0.017453292519943295769;
    const double x = value.u * transform.scale[0];
    const double y = value.v * transform.scale[1];
    value.u = std::cos(angle) * x - std::sin(angle) * y + transform.offset[0];
    value.v = std::sin(angle) * x + std::cos(angle) * y + transform.offset[1];
    value.u = Wrap(value.u, transform.wrapU); value.v = Wrap(value.v, transform.wrapV);
    return value;
}
inline std::array<double, 4> Texel(const Raster& raster, std::uint32_t x,
                                   std::uint32_t y, bool srgb,
                                   bool opaque) noexcept {
    const std::size_t offset = (std::size_t(y) * raster.width + x) * 4;
    const double alpha = opaque ? 1.0 : raster.rgba[offset + 3] / 255.0;
    return {
        (srgb ? SRGBToLinear(raster.rgba[offset])
              : raster.rgba[offset] / 255.0) * alpha,
        (srgb ? SRGBToLinear(raster.rgba[offset + 1])
              : raster.rgba[offset + 1] / 255.0) * alpha,
        (srgb ? SRGBToLinear(raster.rgba[offset + 2])
              : raster.rgba[offset + 2] / 255.0) * alpha,
        alpha,
    };
}
inline std::array<double, 4> Sample(const Raster& raster, UV uv, bool srgb,
                                    face_image::Wrap wrapU,
                                    face_image::Wrap wrapV,
                                    bool opaque) noexcept {
    uv.u = Wrap(uv.u, wrapU); uv.v = Wrap(uv.v, wrapV);
    // Persistent UV origin is lower-left; raster storage is top-left.  A
    // texel centre at i+0.5 maps exactly to integer sample position i.
    const double x = uv.u * raster.width - 0.5;
    const double y = (1.0 - uv.v) * raster.height - 0.5;
    const auto coordinate = [](std::int64_t value, std::uint32_t size,
                               face_image::Wrap mode) noexcept {
        if (mode == face_image::Wrap::ClampToEdge)
            return std::uint32_t(std::clamp<std::int64_t>(value, 0, size - 1));
        const std::int64_t period = mode == face_image::Wrap::Repeat
            ? size : std::int64_t(size) * 2;
        std::int64_t mapped = value % period; if (mapped < 0) mapped += period;
        if (mapped >= size) mapped = period - 1 - mapped;
        return std::uint32_t(mapped);
    };
    const std::int64_t x0 = std::int64_t(std::floor(x)), y0 = std::int64_t(std::floor(y));
    const double fx = x - std::floor(x), fy = y - std::floor(y);
    const auto a = Texel(raster, coordinate(x0, raster.width, wrapU),
                         coordinate(y0, raster.height, wrapV), srgb, opaque);
    const auto b = Texel(raster, coordinate(x0 + 1, raster.width, wrapU),
                         coordinate(y0, raster.height, wrapV), srgb, opaque);
    const auto c = Texel(raster, coordinate(x0, raster.width, wrapU),
                         coordinate(y0 + 1, raster.height, wrapV), srgb, opaque);
    const auto d = Texel(raster, coordinate(x0 + 1, raster.width, wrapU),
                         coordinate(y0 + 1, raster.height, wrapV), srgb, opaque);
    std::array<double, 4> result{};
    for (std::size_t channel = 0; channel < 4; ++channel)
        result[channel] = (a[channel] * (1.0 - fx) + b[channel] * fx) * (1.0 - fy)
            + (c[channel] * (1.0 - fx) + d[channel] * fx) * fy;
    if (result[3] > 0.0)
        for (std::size_t channel = 0; channel < 3; ++channel)
            result[channel] /= result[3];
    else result[0] = result[1] = result[2] = 0.0;
    return result;
}

inline double Edge(const Point& a, const Point& b, const Point& p) noexcept {
    return (p.x - a.x) * (b.y - a.y) - (p.y - a.y) * (b.x - a.x);
}
inline bool TopLeft(const Point& a, const Point& b) noexcept {
    return (a.y == b.y && a.x < b.x) || a.y > b.y;
}
inline bool BarycentricTopLeft(const Triangle& triangle, Point sample,
                               std::array<double, 3>& weights) noexcept {
    Point a = triangle.outputPixels[0], b = triangle.outputPixels[1],
          c = triangle.outputPixels[2];
    double area = Edge(a, b, c);
    if (!std::isfinite(area) || area == 0.0) return false;
    if (area < 0.0) { std::swap(b, c); area = -area; }
    const double e0 = Edge(b, c, sample), e1 = Edge(c, a, sample), e2 = Edge(a, b, sample);
    const auto owns = [](double edge, const Point& first, const Point& second) noexcept {
        return edge > 0.0 || (edge == 0.0 && TopLeft(first, second));
    };
    if (!owns(e0, b, c) || !owns(e1, c, a) || !owns(e2, a, b)) return false;
    weights = {e0 / area, e1 / area, e2 / area};
    if (triangle.outputPixels[1].x == b.x && triangle.outputPixels[1].y == b.y)
        return true;
    std::swap(weights[1], weights[2]); return true;
}
inline bool SameReceipt(const ReceiverReceipt& a,
                        const ReceiverReceipt& b) noexcept {
    return a.face == b.face && a.selectorProof == b.selectorProof;
}
inline ReceiverReceipt IntentReceipt(const Layer& layer) noexcept {
    return layer.placement.kind == PlacementKind::Face
        ? layer.placement.face.receiver : layer.placement.projector.receiver;
}

inline std::uint32_t CRC32(const std::uint8_t* bytes, std::size_t count) noexcept {
    std::uint32_t crc = 0xffffffffU;
    for (std::size_t index = 0; index < count; ++index) {
        crc ^= bytes[index];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320U & std::uint32_t(-(std::int32_t(crc & 1))));
    }
    return ~crc;
}
inline void BE32(std::vector<std::uint8_t>& bytes, std::uint32_t value) {
    bytes.push_back(std::uint8_t(value >> 24)); bytes.push_back(std::uint8_t(value >> 16));
    bytes.push_back(std::uint8_t(value >> 8)); bytes.push_back(std::uint8_t(value));
}
inline void Chunk(std::vector<std::uint8_t>& png, const char type[4],
                  const std::vector<std::uint8_t>& payload) {
    BE32(png, std::uint32_t(payload.size()));
    const std::size_t start = png.size();
    png.insert(png.end(), type, type + 4); png.insert(png.end(), payload.begin(), payload.end());
    BE32(png, CRC32(png.data() + start, png.size() - start));
}
inline bool EncodePNG(const Raster& raster, std::vector<std::uint8_t>& output) noexcept {
    output.clear();
    try {
        if (!Valid(raster)) return false;
        std::vector<std::uint8_t> scanlines;
        scanlines.reserve((std::size_t(raster.width) * 4 + 1) * raster.height);
        for (std::uint32_t y = 0; y < raster.height; ++y) {
            scanlines.push_back(0); // fixed PNG filter None
            const auto first = raster.rgba.begin() + std::size_t(y) * raster.width * 4;
            scanlines.insert(scanlines.end(), first, first + std::size_t(raster.width) * 4);
        }
        std::vector<std::uint8_t> zlib{0x78, 0x01}; // deflate, no preset dictionary
        for (std::size_t offset = 0; offset < scanlines.size();) {
            const std::size_t count = std::min<std::size_t>(65'535, scanlines.size() - offset);
            const bool final = offset + count == scanlines.size();
            zlib.push_back(final ? 1 : 0); // byte-aligned uncompressed block
            const std::uint16_t length = std::uint16_t(count), inverse = std::uint16_t(~length);
            zlib.push_back(std::uint8_t(length)); zlib.push_back(std::uint8_t(length >> 8));
            zlib.push_back(std::uint8_t(inverse)); zlib.push_back(std::uint8_t(inverse >> 8));
            zlib.insert(zlib.end(), scanlines.begin() + offset,
                        scanlines.begin() + offset + count); offset += count;
        }
        std::uint32_t s1 = 1, s2 = 0;
        for (std::uint8_t byte : scanlines) { s1 = (s1 + byte) % 65521; s2 = (s2 + s1) % 65521; }
        BE32(zlib, (s2 << 16) | s1);
        static const std::uint8_t signature[8] = {137,80,78,71,13,10,26,10};
        output.assign(signature, signature + 8);
        std::vector<std::uint8_t> ihdr; BE32(ihdr, raster.width); BE32(ihdr, raster.height);
        ihdr.insert(ihdr.end(), {8, 6, 0, 0, 0});
        Chunk(output, "IHDR", ihdr); Chunk(output, "IDAT", zlib); Chunk(output, "IEND", {});
        return output.size() <= kMaximumOutputBytes;
    } catch (...) { output.clear(); return false; }
}

inline Status Bake(const Input& input, Output& output) noexcept {
    output = {};
    try {
        if (!ValidInputKey(input))
            return Status::Malformed;
        output.pixels = input.baseColor;
        const std::size_t pixelCount = std::size_t(output.pixels.width) * output.pixels.height;
        std::vector<std::array<double, 4>> premultiplied(pixelCount);
        for (std::size_t index = 0; index < pixelCount; ++index) {
            const double alpha = output.pixels.rgba[index * 4 + 3] / 255.0;
            premultiplied[index] = {
                SRGBToLinear(output.pixels.rgba[index * 4]) * alpha,
                SRGBToLinear(output.pixels.rgba[index * 4 + 1]) * alpha,
                SRGBToLinear(output.pixels.rgba[index * 4 + 2]) * alpha, alpha};
        }
        std::uint64_t proofSamples = 0;
        for (const ResolvedLayer& resolved : input.layers) {
            if (!Valid(resolved.intent)
                || resolved.intent.image.role != face_image::Role::BaseColor
                || !Valid(resolved.image, resolved.intent.image)
                || resolved.triangles.empty()
                || !resolved.completeOccludersProved
                || !resolved.boundaryProved
                || (resolved.intent.placement.edgePolicy
                        == EdgePolicy::RejectCrossing
                    && !resolved.footprintContained)
                || (resolved.intent.mask.present
                    && (!resolved.mask.has_value()
                        || !Valid(*resolved.mask,
                            resolved.intent.mask.image))))
                return Status::Malformed;
            const ReceiverReceipt expected = IntentReceipt(resolved.intent);
            bool admittedCoverage = false;
            std::vector<std::uint8_t> claimed(pixelCount, 0);
            for (const Triangle& triangle : resolved.triangles) {
                if (!SameReceipt(expected, triangle.receiver)) return Status::Stale;
                double minX = triangle.outputPixels[0].x, maxX = minX;
                double minY = triangle.outputPixels[0].y, maxY = minY;
                for (const Point& point : triangle.outputPixels) {
                    if (!std::isfinite(point.x) || !std::isfinite(point.y)) return Status::Malformed;
                    minX = std::min(minX, point.x); maxX = std::max(maxX, point.x);
                    minY = std::min(minY, point.y); maxY = std::max(maxY, point.y);
                }
                const int firstX = std::max(0, int(std::floor(minX)));
                const int lastX = std::min(int(output.pixels.width) - 1, int(std::ceil(maxX)) - 1);
                const int firstY = std::max(0, int(std::floor(minY)));
                const int lastY = std::min(int(output.pixels.height) - 1, int(std::ceil(maxY)) - 1);
                std::size_t coverageIndex = 0;
                for (int y = firstY; y <= lastY; ++y) for (int x = firstX; x <= lastX; ++x) {
                    std::array<double, 3> weight{};
                    if (!BarycentricTopLeft(triangle, {x + 0.5, y + 0.5}, weight)) continue;
                    if (coverageIndex >= triangle.coverage.size())
                        return Status::VisibilityInconclusive;
                    const CoverageSample& proof =
                        triangle.coverage[coverageIndex++];
                    if (proof.x != std::uint32_t(x)
                        || proof.y != std::uint32_t(y)
                        || !SameReceipt(expected, proof.receiver)
                        || !proof.conclusive)
                        return Status::VisibilityInconclusive;
                    const std::size_t pixel = std::size_t(y)
                        * output.pixels.width + std::size_t(x);
                    if (claimed[pixel] != 0) return Status::Stale;
                    claimed[pixel] = 1;
                    if (++proofSamples > kMaximumPixels)
                        return Status::Budget;
                    if (!proof.affected) continue;
                    if (!proof.selectedReceiver || !proof.frontFacing
                        || !proof.insideTrim || !proof.nearestHit) continue;
                    admittedCoverage = true;
                    UV uv{};
                    for (std::size_t corner = 0; corner < 3; ++corner) {
                        uv.u += weight[corner] * triangle.layerUV[corner].u;
                        uv.v += weight[corner] * triangle.layerUV[corner].v;
                    }
                    const auto source = Sample(resolved.image.pixels, uv,
                        resolved.intent.image.colorSpace
                            == face_image::ColorSpace::SRGB,
                        face_image::Wrap::ClampToEdge,
                        face_image::Wrap::ClampToEdge,
                        resolved.intent.image.alpha
                            == face_image::AlphaInterpretation::Opaque);
                    double mask = 1.0;
                    if (resolved.intent.mask.present) {
                        const UV maskUV = Transform(
                            uv, resolved.intent.mask.transform);
                        const auto sampled = Sample(resolved.mask->pixels, maskUV,
                            resolved.intent.mask.image.colorSpace
                                == face_image::ColorSpace::SRGB,
                            resolved.intent.mask.transform.wrapU,
                            resolved.intent.mask.transform.wrapV,
                            resolved.intent.mask.image.alpha
                                == face_image::AlphaInterpretation::Opaque);
                        mask = resolved.intent.mask.channel == MaskChannel::Alpha
                            ? sampled[3] : 0.2126 * sampled[0] + 0.7152 * sampled[1] + 0.0722 * sampled[2];
                        if (resolved.intent.mask.inverted) mask = 1.0 - mask;
                    }
                    const double sourceAlpha = std::clamp(source[3]
                        * resolved.intent.opacity * mask, 0.0, 1.0);
                    auto& destination = premultiplied[pixel];
                    for (std::size_t channel = 0; channel < 3; ++channel)
                        destination[channel] = source[channel] * sourceAlpha
                            + destination[channel] * (1.0 - sourceAlpha);
                    destination[3] = sourceAlpha + destination[3] * (1.0 - sourceAlpha);
                }
                if (coverageIndex != triangle.coverage.size())
                    return Status::VisibilityInconclusive;
            }
            if (!admittedCoverage) return Status::Unsupported;
        }
        for (std::size_t index = 0; index < pixelCount; ++index) {
            const double alpha = std::clamp(premultiplied[index][3], 0.0, 1.0);
            for (std::size_t channel = 0; channel < 3; ++channel)
                output.pixels.rgba[index * 4 + channel] = LinearToSRGB(
                    alpha == 0.0 ? 0.0 : premultiplied[index][channel] / alpha);
            output.pixels.rgba[index * 4 + 3] = LinearByte(alpha);
        }
        if (!EncodePNG(output.pixels, output.png)
            || !face_image::HashFaceImageBytes(output.png, output.pngDigest)) {
            output = {}; return Status::Budget;
        }
        return Status::Baked;
    } catch (...) { output = {}; return Status::Budget; }
}

struct FaceProduct final {
    PublishedFace face;
    Digest inputKey{};
};

inline Status ProduceFace(Input& input, const ReceiverReceipt& receiver,
                          FaceProduct& product) noexcept {
    product = {};
    try {
        if (!core3d::decal_layer::Valid(receiver)
            || input.layers.empty()) return Status::Malformed;
        for (const ResolvedLayer& layer : input.layers)
            if (!SameReceipt(IntentReceipt(layer.intent), receiver))
                return Status::Stale;
        if (!BindInputKey(input)) return Status::Malformed;
        Output output;
        const Status status = Bake(input, output);
        if (status != Status::Baked) return status;
        product.face.receiver = receiver;
        product.face.pixelWidth = output.pixels.width;
        product.face.pixelHeight = output.pixels.height;
        product.face.png = std::move(output.png);
        product.face.pngDigest = output.pngDigest;
        product.inputKey = input.inputKey;
        return Status::Baked;
    } catch (...) { product = {}; return Status::Budget; }
}

//! Assemble and seal a complete multi-face artifact only after every face has
//! been freshly produced. This is the one validation/seal algorithm for both
//! ordinary shared publication and private operation-owned publication.
inline bool Assemble(const Definition& definition,
                     const Digest& finalGeometryUVProof,
                     const Digest& occluderProof,
                     const std::vector<FaceProduct>& products,
                     PublishedDerivative& published) noexcept {
    published = {};
    try {
        Refusal refusal{};
        Definition canonical = definition;
        const Digest suppliedLayerProof = canonical.layerProof;
        if (!Valid(definition, refusal) || refusal != Refusal::None
            || !BindLayerProof(canonical)
            || canonical.layerProof != suppliedLayerProof
            || !Nonzero(finalGeometryUVProof) || !Nonzero(occluderProof)
            || products.empty() || products.size() > kMaximumLayers)
            return false;
        face_image::detail::Writer input;
        input.raw(reinterpret_cast<const std::uint8_t*>("E4AK"), 4);
        input.integer(1, 4); input.raw(definition.owner.document);
        input.raw(definition.owner.entity); input.raw(definition.owner.definition);
        input.raw(definition.layerProof); input.raw(definition.sourceProof);
        input.integer(definition.geometryRevision, 8);
        input.raw(finalGeometryUVProof); input.raw(occluderProof);
        input.integer(products.size(), 4);
        std::set<std::pair<UUID, Digest>> receivers;
        published.faces.reserve(products.size());
        for (const FaceProduct& product : products) {
            if (!Nonzero(product.inputKey)
                || !core3d::decal_layer::Valid(product.face.receiver)
                || !receivers.emplace(product.face.receiver.face,
                    product.face.receiver.selectorProof).second) return false;
            input.raw(product.face.receiver.face);
            input.raw(product.face.receiver.selectorProof);
            input.raw(product.inputKey);
            published.faces.push_back(product.face);
        }
        if (!input.ok
            || !face_image::HashFaceImageBytes(
                input.bytes, published.inputKey)
            || !Nonzero(published.inputKey)) return false;
        published.owner = definition.owner;
        published.layerProof = definition.layerProof;
        published.sourceProof = definition.sourceProof;
        published.geometryRevision = definition.geometryRevision;
        published.finalGeometryUVProof = finalGeometryUVProof;
        published.occluderProof = occluderProof;
        if (!publication::BindBakeKey(published)
            || !publication::Valid(published)) {
            published = {}; return false;
        }
        return true;
    } catch (...) { published = {}; return false; }
}

//! Atomically publish through the ordinary process-shared cache.
inline bool Publish(const Definition& definition,
                    const Digest& finalGeometryUVProof,
                    const Digest& occluderProof,
                    const std::vector<FaceProduct>& products,
                    PublishedDerivative& published) noexcept {
    if (!Assemble(definition, finalGeometryUVProof, occluderProof,
                  products, published)
        || !publication::Store(published)) {
        published = {};
        return false;
    }
    return true;
}

//! Atomically publish into an explicit operation-owned destination without
//! inserting, replacing, evicting, or looking up a viewport cache entry.
inline bool PublishPrivate(const Definition& definition,
                           const Digest& finalGeometryUVProof,
                           const Digest& occluderProof,
                           const std::vector<FaceProduct>& products,
                           publication::PrivateDestination& destination,
                           PublishedDerivative& published) noexcept {
    if (!Assemble(definition, finalGeometryUVProof, occluderProof,
                  products, published)
        || !publication::Store(destination, published)) {
        destination.clear();
        published = {};
        return false;
    }
    PublishedDerivative reopened;
    if (!publication::Lookup(destination, published.owner,
                             published.inputKey, reopened)
        || reopened.bakeKey != published.bakeKey) {
        destination.clear();
        published = {};
        return false;
    }
    return true;
}

} // namespace core3d::decal_layer::bake
