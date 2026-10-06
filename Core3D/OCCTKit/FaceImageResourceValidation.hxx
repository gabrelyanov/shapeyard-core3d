#pragma once

// Shared private validation for E3 adoption and future E4 command-free
// preparation. This boundary derives every property from the supplied bytes;
// caller digests and decoded manifests are never authority.
#import <Foundation/Foundation.h>

#include "FaceImageDefinition.hxx"
#include "OcctDocument.h"

#include <ImageIO/ImageIO.h>
#include <Image_Texture.hxx>

#include <cstdint>
#include <cstring>
#include <vector>

namespace core3d::face_image::validation {

struct FaceImageRasterInfo final {
    ImageEncoding format = ImageEncoding::PNG;
    std::uint32_t width = 0, height = 0;
    bool hasAlpha = false;
};

// Measure one encoded image from its actual bytes: the shared bounded
// validator first, then ImageIO metadata and a real pixel decode for exact
// dimensions and alpha-channel presence. Nothing is taken from a caller
// manifest.
inline bool MeasureFaceImageRaster(NSData *data, FaceImageRasterInfo& output) {
    output = {};
    @try {
        if (![data isKindOfClass:NSData.class] || data.length == 0
            || data.length > kMaximumEncodedImageBytes) return false;
        const auto *bytes = static_cast<const std::uint8_t*>(data.bytes);
        const std::size_t size = data.length;
        const bool png = size >= 8
            && std::memcmp(bytes, "\x89PNG\r\n\x1a\n", 8) == 0;
        const bool jpeg = size >= 3
            && bytes[0] == 0xFF && bytes[1] == 0xD8 && bytes[2] == 0xFF;
        if (!png && !jpeg) return false;
        Handle(Image_Texture) texture;
        if (!Core3DCreateAuthoredTexture(bytes, size,
                png ? "image/png" : "image/jpeg", texture) || texture.IsNull())
            return false;
        CGImageSourceRef source =
            CGImageSourceCreateWithData((__bridge CFDataRef)data, nullptr);
        if (!source) return false;
        bool ok = false;
        std::int64_t width = 0, height = 0;
        bool hasAlpha = false;
        do {
            if (CGImageSourceGetCount(source) != 1
                || CGImageSourceGetStatusAtIndex(source, 0) != kCGImageStatusComplete)
                break;
            CFDictionaryRef properties =
                CGImageSourceCopyPropertiesAtIndex(source, 0, nullptr);
            if (!properties) break;
            const CFTypeRef widthValue = CFDictionaryGetValue(
                properties, kCGImagePropertyPixelWidth);
            const CFTypeRef heightValue = CFDictionaryGetValue(
                properties, kCGImagePropertyPixelHeight);
            const bool measured = widthValue && heightValue
                && CFGetTypeID(widthValue) == CFNumberGetTypeID()
                && CFGetTypeID(heightValue) == CFNumberGetTypeID()
                && CFNumberGetValue((CFNumberRef)widthValue, kCFNumberSInt64Type, &width)
                && CFNumberGetValue((CFNumberRef)heightValue, kCFNumberSInt64Type, &height);
            CFRelease(properties);
            if (!measured) break;
            CGImageRef image = CGImageSourceCreateImageAtIndex(source, 0, nullptr);
            if (!image) break;
            const CGImageAlphaInfo alphaInfo = CGImageGetAlphaInfo(image);
            hasAlpha = alphaInfo != kCGImageAlphaNone
                && alphaInfo != kCGImageAlphaNoneSkipFirst
                && alphaInfo != kCGImageAlphaNoneSkipLast;
            CGImageRelease(image);
            ok = true;
        } while (false);
        CFRelease(source);
        if (!ok || width < 1 || height < 1
            || width > kMaximumImageDimension || height > kMaximumImageDimension
            || static_cast<std::uint64_t>(width) * static_cast<std::uint64_t>(height)
                > kMaximumImagePixels
            || static_cast<std::uint64_t>(width) * static_cast<std::uint64_t>(height) * 4
                > kMaximumDecodedImageBytes) return false;
        output.format = png ? ImageEncoding::PNG : ImageEncoding::JPEG;
        output.width = static_cast<std::uint32_t>(width);
        output.height = static_cast<std::uint32_t>(height);
        output.hasAlpha = hasAlpha;
        return true;
    } @catch (...) { output = {}; return false; }
}

// Build one validated envelope from trusted local bytes. Both payloads are
// rehashed natively; formats, dimensions and alpha presence are measured from
// the actual images. `straight` requires a real alpha channel in the working
// image; a JPEG can never carry one.
inline bool BuildFaceImageEnvelope(NSData *original, NSData *working,
                                   NSString *alphaInterpretation, NSData *provenance,
                                   const UUID& resource,
                                   ResourceEnvelope& envelope) {
    envelope = {};
    @try {
        FaceImageRasterInfo originalInfo, workingInfo;
        if (!MeasureFaceImageRaster(original, originalInfo)) return false;
        if (!MeasureFaceImageRaster(working, workingInfo)) return false;
        if ([alphaInterpretation isEqualToString:@"opaque"]) {
            envelope.alpha = AlphaInterpretation::Opaque;
        } else if ([alphaInterpretation isEqualToString:@"straight"]) {
            if (!workingInfo.hasAlpha || workingInfo.format == ImageEncoding::JPEG)
                return false;
            envelope.alpha = AlphaInterpretation::Straight;
        } else {
            return false;
        }
        if (![provenance isKindOfClass:NSData.class] || provenance.length == 0
            || provenance.length > 4096) return false;
        if (!retained_recipe::Nonzero(resource)) return false;
        envelope.resource = resource;
        envelope.originalBytes.assign(
            static_cast<const std::uint8_t*>(original.bytes),
            static_cast<const std::uint8_t*>(original.bytes) + original.length);
        envelope.workingBytes.assign(
            static_cast<const std::uint8_t*>(working.bytes),
            static_cast<const std::uint8_t*>(working.bytes) + working.length);
        if (!HashFaceImageBytes(envelope.originalBytes, envelope.originalContent)
            || !HashFaceImageBytes(envelope.workingBytes, envelope.workingContent))
            return false;
        const std::vector<std::uint8_t> provenanceBytes(
            static_cast<const std::uint8_t*>(provenance.bytes),
            static_cast<const std::uint8_t*>(provenance.bytes) + provenance.length);
        if (!HashFaceImageBytes(provenanceBytes, envelope.provenance)) return false;
        envelope.originalFormat = originalInfo.format;
        envelope.workingFormat = workingInfo.format;
        envelope.originalWidthTexels = originalInfo.width;
        envelope.originalHeightTexels = originalInfo.height;
        envelope.workingWidthTexels = workingInfo.width;
        envelope.workingHeightTexels = workingInfo.height;
        Refusal refusal;
        return Valid(envelope, refusal);
    } @catch (...) { envelope = {}; return false; }
}

} // namespace core3d::face_image::validation
