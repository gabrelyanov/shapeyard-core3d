#pragma once

// E4 retained decal-layer values and the canonical SYDL/1 byte codec.
// This file deliberately contains no document mutation authority.  In
// particular, decoded UUIDs and digests are descriptive values only: native
// owner/resource/currentness checks are required before preview or export.
#include "FaceImageDefinition.hxx"
#include "../Common/Core3DMobileResourceLimits.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

namespace core3d::decal_layer {
using face_image::Digest;
using face_image::UUID;
using face_image::OwnerKey;

inline constexpr char kSchemaVersion[] = "shapeyard.decal-layers.v1";
inline constexpr std::size_t kMaximumBytes = 8U * 1024U * 1024U;
inline constexpr std::size_t kMaximumLayers = 256;
// The approved Portion-2 carrier is the existing single-face receiver.  A
// future retained-region schema must be founder-approved and cannot be
// smuggled into the cardinality field.
inline constexpr std::uint16_t kMaximumReceiverCardinality = 1;

enum class PlacementKind : std::uint8_t { Face = 1, OrthographicProjector = 2 };
enum class EdgePolicy : std::uint8_t { RejectCrossing = 1, ClipToSelectedFaces = 2 };
enum class MaskChannel : std::uint8_t { Alpha = 1, LinearLuminance = 2 };
enum class Refusal : std::uint8_t {
    None, Malformed, Oversized, StaleOwner, StaleSource, StaleReceiver,
    AmbiguousReceiver, MissingResource, StaleResource, ForeignResource,
    UnsupportedSurface, OcclusionInconclusive, Budget, PersistenceFailure
};

struct ImageRef final {
    UUID resource{};
    Digest normalizedContent{};
    Digest originalContent{};
    Digest provenance{};
    std::uint32_t producerVersion = 0;
    face_image::ImageEncoding mediaType = face_image::ImageEncoding::PNG;
    std::uint32_t widthTexels = 0;
    std::uint32_t heightTexels = 0;
    face_image::Role role = face_image::Role::BaseColor;
    face_image::ColorSpace colorSpace = face_image::ColorSpace::SRGB;
    face_image::AlphaInterpretation alpha = face_image::AlphaInterpretation::Straight;
};

struct ReceiverReceipt final {
    UUID face{};
    Digest selectorProof{};
};

struct Frame final {
    std::array<double, 3> origin{};
    // Sign-canonical unit quaternion (x,y,z,w), w >= 0.
    std::array<double, 4> orientation{0.0, 0.0, 0.0, 1.0};
};

struct FacePlacement final {
    ReceiverReceipt receiver;
    std::array<double, 2> anchorMeters{};
    double angleRadians = 0.0;
};

struct ProjectorPlacement final {
    ReceiverReceipt receiver;
    Frame frame;
    double nearDepthMeters = 0.0;
    double farDepthMeters = 0.0;
};

struct Placement final {
    PlacementKind kind = PlacementKind::Face;
    EdgePolicy edgePolicy = EdgePolicy::RejectCrossing;
    std::uint16_t expectedCardinality = 1;
    FacePlacement face;
    ProjectorPlacement projector;
};

struct Mask final {
    bool present = false;
    ImageRef image;
    MaskChannel channel = MaskChannel::Alpha;
    bool inverted = false;
    face_image::UVTransform transform;
};

struct Layer final {
    UUID identifier{};
    ImageRef image;
    Placement placement;
    double widthMeters = 0.0;
    double heightMeters = 0.0;
    double opacity = 1.0;
    Mask mask;
};

struct Definition final {
    OwnerKey owner{};
    UUID sourceRecipe{};
    std::uint64_t sourceRevision = 0;
    std::uint64_t geometryRevision = 0;
    std::uint64_t placementRevision = 0;
    Digest sourceProof{};
    std::vector<Layer> layers; // order is authoritative
    Digest layerProof{};
};

inline bool Nonzero(const Digest& value) noexcept { return face_image::Nonzero(value); }
inline bool Finite(double value) noexcept { return std::isfinite(value); }
inline bool PositiveLength(double value) noexcept {
    return Finite(value) && value > 0.0
        && value <= core3d::limits::kMaximumModelCoordinateMagnitude;
}
inline bool ValidQuaternion(const std::array<double, 4>& value) noexcept {
    double norm = 0.0;
    for (double scalar : value) {
        if (!Finite(scalar)) return false;
        norm += scalar * scalar;
    }
    return value[3] >= 0.0 && std::abs(norm - 1.0) <= 1.0e-12;
}
inline bool Valid(const ImageRef& value) noexcept {
    if (!retained_recipe::Nonzero(value.resource)
        || !Nonzero(value.normalizedContent) || !Nonzero(value.originalContent)
        || !Nonzero(value.provenance) || value.producerVersion == 0
        || value.widthTexels == 0 || value.heightTexels == 0
        || value.widthTexels > std::uint32_t(face_image::kMaximumImageDimension)
        || value.heightTexels > std::uint32_t(face_image::kMaximumImageDimension)
        || std::uint64_t(value.widthTexels) * value.heightTexels
            > face_image::kMaximumImagePixels
        || !face_image::ValidColorSpaceForRole(value.role, value.colorSpace)) return false;
    switch (value.mediaType) {
        case face_image::ImageEncoding::PNG:
        case face_image::ImageEncoding::JPEG: break;
        default: return false;
    }
    switch (value.alpha) {
        case face_image::AlphaInterpretation::Opaque:
        case face_image::AlphaInterpretation::Straight: break;
        default: return false;
    }
    return true;
}
inline bool Valid(const ReceiverReceipt& value) noexcept {
    return retained_recipe::Nonzero(value.face) && Nonzero(value.selectorProof);
}
inline bool Valid(const Placement& value) noexcept {
    if (value.expectedCardinality != 1
        || value.expectedCardinality > kMaximumReceiverCardinality) return false;
    switch (value.edgePolicy) {
        case EdgePolicy::RejectCrossing:
        case EdgePolicy::ClipToSelectedFaces: break;
        default: return false;
    }
    if (value.kind == PlacementKind::Face) {
        return Valid(value.face.receiver)
            && Finite(value.face.anchorMeters[0])
            && Finite(value.face.anchorMeters[1])
            && Finite(value.face.angleRadians)
            && value.face.angleRadians >= -3.14159265358979323846
            && value.face.angleRadians < 3.14159265358979323846;
    }
    if (value.kind == PlacementKind::OrthographicProjector) {
        if (!Valid(value.projector.receiver)
            || !ValidQuaternion(value.projector.frame.orientation)
            || !Finite(value.projector.nearDepthMeters)
            || !Finite(value.projector.farDepthMeters)
            || value.projector.nearDepthMeters < 0.0
            || value.projector.farDepthMeters <= value.projector.nearDepthMeters)
            return false;
        for (double scalar : value.projector.frame.origin)
            if (!Finite(scalar)) return false;
        return true;
    }
    return false;
}
inline bool Valid(const Layer& value) noexcept {
    if (!retained_recipe::Nonzero(value.identifier) || !Valid(value.image)
        || !Valid(value.placement) || !PositiveLength(value.widthMeters)
        || !PositiveLength(value.heightMeters) || !Finite(value.opacity)
        || value.opacity < 0.0 || value.opacity > 1.0) return false;
    if (!value.mask.present) return true;
    if (!Valid(value.mask.image) || !face_image::Valid(value.mask.transform)) return false;
    switch (value.mask.channel) {
        case MaskChannel::Alpha: case MaskChannel::LinearLuminance: return true;
        default: return false;
    }
}
inline bool Valid(const Definition& value, Refusal& refusal) noexcept {
    refusal = Refusal::Malformed;
    try {
        if (!retained_recipe::Valid(value.owner)
            || !retained_recipe::Nonzero(value.sourceRecipe)
            || value.sourceRevision == 0 || value.geometryRevision == 0
            || value.placementRevision == 0 || !Nonzero(value.sourceProof)
            || value.layers.empty() || value.layers.size() > kMaximumLayers
            || !Nonzero(value.layerProof)) return false;
        for (std::size_t index = 0; index < value.layers.size(); ++index) {
            if (!Valid(value.layers[index])) return false;
            for (std::size_t other = index + 1; other < value.layers.size(); ++other)
                if (value.layers[index].identifier == value.layers[other].identifier)
                    return false;
        }
        refusal = Refusal::None;
        return true;
    } catch (...) { return false; }
}

namespace detail {
using Writer = face_image::detail::Writer;
using Reader = face_image::detail::Reader;
inline void WriteImage(const ImageRef& value, Writer& writer) {
    writer.raw(value.resource); writer.raw(value.normalizedContent);
    writer.raw(value.originalContent); writer.raw(value.provenance);
    writer.integer(value.producerVersion, 4);
    writer.integer(std::uint8_t(value.mediaType), 1);
    writer.integer(std::uint8_t(value.role), 1);
    writer.integer(std::uint8_t(value.colorSpace), 1);
    writer.integer(std::uint8_t(value.alpha), 1);
    writer.integer(value.widthTexels, 4); writer.integer(value.heightTexels, 4);
}
inline bool ReadImage(Reader& reader, ImageRef& value) {
    std::uint64_t producer = 0, media = 0, role = 0, color = 0, alpha = 0;
    std::uint64_t width = 0, height = 0;
    if (!reader.raw(value.resource) || !reader.raw(value.normalizedContent)
        || !reader.raw(value.originalContent) || !reader.raw(value.provenance)
        || !reader.integer(4, producer) || !reader.integer(1, media)
        || !reader.integer(1, role) || !reader.integer(1, color)
        || !reader.integer(1, alpha) || !reader.integer(4, width)
        || !reader.integer(4, height) || producer > UINT32_MAX
        || width > UINT32_MAX || height > UINT32_MAX) return false;
    value.producerVersion = std::uint32_t(producer);
    value.mediaType = face_image::ImageEncoding(media);
    value.role = face_image::Role(role); value.colorSpace = face_image::ColorSpace(color);
    value.alpha = face_image::AlphaInterpretation(alpha);
    value.widthTexels = std::uint32_t(width); value.heightTexels = std::uint32_t(height);
    return true;
}
inline void WriteTransform(const face_image::UVTransform& value, Writer& writer) {
    for (double scalar : value.scale) writer.real(scalar);
    for (double scalar : value.offset) writer.real(scalar);
    writer.real(value.rotationDegrees);
    writer.integer(std::uint8_t(value.wrapU), 1);
    writer.integer(std::uint8_t(value.wrapV), 1);
}
inline bool ReadTransform(Reader& reader, face_image::UVTransform& value) {
    std::uint64_t wrapU = 0, wrapV = 0;
    for (double& scalar : value.scale) if (!reader.real(scalar)) return false;
    for (double& scalar : value.offset) if (!reader.real(scalar)) return false;
    if (!reader.real(value.rotationDegrees) || !reader.integer(1, wrapU)
        || !reader.integer(1, wrapV)) return false;
    value.wrapU = face_image::Wrap(wrapU); value.wrapV = face_image::Wrap(wrapV);
    return true;
}
inline void WriteDefinition(const Definition& value, Writer& writer) {
    writer.raw(reinterpret_cast<const std::uint8_t*>("SYDL\1\0\0\0"), 8);
    writer.raw(value.owner.document); writer.raw(value.owner.entity);
    writer.raw(value.owner.definition); writer.raw(value.sourceRecipe);
    writer.integer(value.sourceRevision, 8); writer.integer(value.geometryRevision, 8);
    writer.integer(value.placementRevision, 8); writer.raw(value.sourceProof);
    writer.integer(value.layers.size(), 2); writer.integer(0, 2);
    for (const Layer& layer : value.layers) {
        writer.raw(layer.identifier); WriteImage(layer.image, writer);
        writer.integer(std::uint8_t(layer.placement.kind), 1);
        writer.integer(std::uint8_t(layer.placement.edgePolicy), 1);
        writer.integer(layer.placement.expectedCardinality, 2);
        const ReceiverReceipt& receipt = layer.placement.kind == PlacementKind::Face
            ? layer.placement.face.receiver : layer.placement.projector.receiver;
        writer.raw(receipt.face); writer.raw(receipt.selectorProof);
        if (layer.placement.kind == PlacementKind::Face) {
            writer.real(layer.placement.face.anchorMeters[0]);
            writer.real(layer.placement.face.anchorMeters[1]);
            writer.real(layer.placement.face.angleRadians);
        } else {
            for (double scalar : layer.placement.projector.frame.origin) writer.real(scalar);
            for (double scalar : layer.placement.projector.frame.orientation) writer.real(scalar);
            writer.real(layer.placement.projector.nearDepthMeters);
            writer.real(layer.placement.projector.farDepthMeters);
        }
        writer.real(layer.widthMeters); writer.real(layer.heightMeters);
        writer.real(layer.opacity); writer.integer(layer.mask.present ? 1 : 0, 1);
        if (layer.mask.present) {
            WriteImage(layer.mask.image, writer);
            writer.integer(std::uint8_t(layer.mask.channel), 1);
            writer.integer(layer.mask.inverted ? 1 : 0, 1);
            writer.integer(0, 2); WriteTransform(layer.mask.transform, writer);
        }
    }
    writer.raw(value.layerProof);
}
} // namespace detail

inline bool Hash(const std::vector<std::uint8_t>& bytes, Digest& output) noexcept {
    return !bytes.empty() && bytes.size() <= kMaximumBytes
        && face_image::HashFaceImageBytes(bytes, output);
}
inline bool BindLayerProof(Definition& value) noexcept {
    try {
        Definition covered = value; covered.layerProof = {};
        detail::Writer writer; detail::WriteDefinition(covered, writer);
        return writer.ok && writer.bytes.size() <= kMaximumBytes
            && Hash(writer.bytes, value.layerProof) && Nonzero(value.layerProof);
    } catch (...) { value.layerProof = {}; return false; }
}
inline bool Encode(const Definition& value, std::vector<std::uint8_t>& output) noexcept {
    output.clear(); Refusal refusal;
    try {
        if (!Valid(value, refusal)) return false;
        detail::Writer writer; detail::WriteDefinition(value, writer);
        if (!writer.ok || writer.bytes.size() > kMaximumBytes - 32) return false;
        Digest digest{}; if (!Hash(writer.bytes, digest)) return false;
        writer.raw(digest); if (!writer.ok || writer.bytes.size() > kMaximumBytes) return false;
        output = std::move(writer.bytes); return true;
    } catch (...) { output.clear(); return false; }
}

inline bool Decode(const std::vector<std::uint8_t>& bytes, Definition& output,
                   Refusal& refusal) noexcept {
    output = {}; refusal = bytes.size() > kMaximumBytes ? Refusal::Oversized : Refusal::Malformed;
    try {
        constexpr std::size_t minimum = 8 + 4 * 16 + 3 * 8 + 32 + 4 + 32 + 32;
        if (bytes.size() < minimum || bytes.size() > kMaximumBytes
            || std::memcmp(bytes.data(), "SYDL\1\0\0\0", 8) != 0) return false;
        std::vector<std::uint8_t> body(bytes.begin(), bytes.end() - 32);
        Digest expected{}, actual{};
        if (!Hash(body, expected)) return false;
        std::copy_n(bytes.end() - 32, 32, actual.begin());
        if (actual != expected) return false;
        detail::Reader reader(bytes, bytes.size() - 32);
        std::array<std::uint8_t, 8> prefix{}; Definition value;
        std::uint64_t count = 0, reserved = 0;
        if (!reader.raw(prefix) || !reader.raw(value.owner.document)
            || !reader.raw(value.owner.entity) || !reader.raw(value.owner.definition)
            || !reader.raw(value.sourceRecipe) || !reader.integer(8, value.sourceRevision)
            || !reader.integer(8, value.geometryRevision)
            || !reader.integer(8, value.placementRevision) || !reader.raw(value.sourceProof)
            || !reader.integer(2, count) || count == 0 || count > kMaximumLayers
            || !reader.integer(2, reserved) || reserved != 0) return false;
        value.layers.resize(std::size_t(count));
        for (Layer& layer : value.layers) {
            std::uint64_t kind = 0, edge = 0, cardinality = 0, mask = 0;
            if (!reader.raw(layer.identifier) || !detail::ReadImage(reader, layer.image)
                || !reader.integer(1, kind) || !reader.integer(1, edge)
                || !reader.integer(2, cardinality)) return false;
            layer.placement.kind = PlacementKind(kind);
            layer.placement.edgePolicy = EdgePolicy(edge);
            layer.placement.expectedCardinality = std::uint16_t(cardinality);
            ReceiverReceipt* receipt = kind == std::uint8_t(PlacementKind::Face)
                ? &layer.placement.face.receiver : &layer.placement.projector.receiver;
            if (!reader.raw(receipt->face) || !reader.raw(receipt->selectorProof)) return false;
            if (layer.placement.kind == PlacementKind::Face) {
                if (!reader.real(layer.placement.face.anchorMeters[0])
                    || !reader.real(layer.placement.face.anchorMeters[1])
                    || !reader.real(layer.placement.face.angleRadians)) return false;
            } else if (layer.placement.kind == PlacementKind::OrthographicProjector) {
                for (double& scalar : layer.placement.projector.frame.origin)
                    if (!reader.real(scalar)) return false;
                for (double& scalar : layer.placement.projector.frame.orientation)
                    if (!reader.real(scalar)) return false;
                if (!reader.real(layer.placement.projector.nearDepthMeters)
                    || !reader.real(layer.placement.projector.farDepthMeters)) return false;
            } else return false;
            if (!reader.real(layer.widthMeters) || !reader.real(layer.heightMeters)
                || !reader.real(layer.opacity) || !reader.integer(1, mask) || mask > 1)
                return false;
            layer.mask.present = mask != 0;
            if (layer.mask.present) {
                std::uint64_t channel = 0, inverted = 0;
                if (!detail::ReadImage(reader, layer.mask.image)
                    || !reader.integer(1, channel) || !reader.integer(1, inverted)
                    || inverted > 1 || !reader.integer(2, reserved) || reserved != 0
                    || !detail::ReadTransform(reader, layer.mask.transform)) return false;
                layer.mask.channel = MaskChannel(channel); layer.mask.inverted = inverted != 0;
            }
        }
        if (!reader.raw(value.layerProof) || !reader.complete()) return false;
        std::vector<std::uint8_t> canonical; Refusal validation;
        if (!Valid(value, validation) || !Encode(value, canonical) || canonical != bytes)
            return false;
        output = std::move(value); refusal = Refusal::None; return true;
    } catch (...) { output = {}; return false; }
}

} // namespace core3d::decal_layer
