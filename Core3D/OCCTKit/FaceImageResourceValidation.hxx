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

//! Optional inherited admission for this validator's native allocations.
//! The caller owns the tickets and keeps them alive until this function
//! returns. Header parsing is the only work done before admitDecoded.
struct FaceImageRasterOptions final {
    void *context = nullptr;
    bool (*admitSource)(void *, std::size_t) noexcept = nullptr;
    bool (*admitDecoded)(void *, std::size_t, std::size_t) noexcept = nullptr;
    bool (*enterDecode)(void *) noexcept = nullptr;

    bool enabled() const noexcept {
        return context && admitSource && admitDecoded && enterDecode;
    }
};

// Measure one encoded image from its actual bytes: the shared bounded
// validator first, then ImageIO metadata and a real pixel decode for exact
// dimensions and alpha-channel presence. Nothing is taken from a caller
// manifest.
inline bool MeasureFaceImageRaster(NSData *data, FaceImageRasterInfo& output,
                                   const FaceImageRasterOptions& options) {
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
        if (options.context && !options.enabled()) return false;
        if (options.enabled() && !options.admitSource(options.context, size))
            return false;
        const void* optionKeys[] = {
            kCGImageSourceShouldCache,
            kCGImageSourceShouldCacheImmediately,
        };
        const void* optionValues[] = {kCFBooleanFalse, kCFBooleanFalse};
        CFDictionaryRef imageOptions = CFDictionaryCreate(kCFAllocatorDefault,
            optionKeys, optionValues, 2, &kCFTypeDictionaryKeyCallBacks,
            &kCFTypeDictionaryValueCallBacks);
        if (!imageOptions) return false;
        CGImageSourceRef source =
            CGImageSourceCreateWithData((__bridge CFDataRef)data, imageOptions);
        if (!source) {
            CFRelease(imageOptions);
            return false;
        }
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
            const CFTypeRef depthValue = CFDictionaryGetValue(
                properties, kCGImagePropertyDepth);
            const CFTypeRef colorModelValue = CFDictionaryGetValue(
                properties, kCGImagePropertyColorModel);
            const CFTypeRef alphaValue = CFDictionaryGetValue(
                properties, kCGImagePropertyHasAlpha);
            std::int64_t depth = 0;
            const bool measured = widthValue && heightValue && depthValue
                && CFGetTypeID(widthValue) == CFNumberGetTypeID()
                && CFGetTypeID(heightValue) == CFNumberGetTypeID()
                && CFGetTypeID(depthValue) == CFNumberGetTypeID()
                && colorModelValue
                && CFGetTypeID(colorModelValue) == CFStringGetTypeID()
                && alphaValue
                && CFGetTypeID(alphaValue) == CFBooleanGetTypeID()
                && CFNumberGetValue((CFNumberRef)widthValue, kCFNumberSInt64Type, &width)
                && CFNumberGetValue((CFNumberRef)heightValue, kCFNumberSInt64Type, &height)
                && CFNumberGetValue((CFNumberRef)depthValue, kCFNumberSInt64Type, &depth);
            std::size_t components = 0;
            if (measured) {
                const auto colorModel =
                    static_cast<CFStringRef>(colorModelValue);
                if (CFEqual(colorModel, kCGImagePropertyColorModelRGB))
                    components = 3;
                else if (CFEqual(colorModel, kCGImagePropertyColorModelGray))
                    components = 1;
                else if (CFEqual(colorModel, kCGImagePropertyColorModelCMYK))
                    components = 4;
                else if (CFEqual(colorModel, kCGImagePropertyColorModelLab))
                    components = 3;
                if (CFBooleanGetValue(static_cast<CFBooleanRef>(alphaValue)))
                    ++components;
            }
            CFRelease(properties);
            if (!measured || depth < 1 || components == 0
                || width < 1 || height < 1
                || width > kMaximumImageDimension
                || height > kMaximumImageDimension
                || static_cast<std::uint64_t>(width)
                    * static_cast<std::uint64_t>(height) > kMaximumImagePixels
                || static_cast<std::uint64_t>(width)
                    * static_cast<std::uint64_t>(height) * 4
                    > kMaximumDecodedImageBytes)
                break;
            if (options.enabled()
                && !options.admitDecoded(options.context,
                    std::size_t(width), std::size_t(height)))
                break;
            if (options.enabled() && !options.enterDecode(options.context))
                break;
            Handle(Image_Texture) texture;
            if (!Core3DCreateAuthoredTexture(bytes, size,
                    png ? "image/png" : "image/jpeg", texture)
                || texture.IsNull())
                break;
            CGImageRef image =
                CGImageSourceCreateImageAtIndex(source, 0, imageOptions);
            if (!image) break;
            if (CGImageGetWidth(image) != std::size_t(width)
                || CGImageGetHeight(image) != std::size_t(height)) {
                CGImageRelease(image);
                break;
            }
            const CGImageAlphaInfo alphaInfo = CGImageGetAlphaInfo(image);
            hasAlpha = alphaInfo != kCGImageAlphaNone
                && alphaInfo != kCGImageAlphaNoneSkipFirst
                && alphaInfo != kCGImageAlphaNoneSkipLast;
            CGImageRelease(image);
            ok = true;
        } while (false);
        CFRelease(source);
        CFRelease(imageOptions);
        if (!ok) return false;
        output.format = png ? ImageEncoding::PNG : ImageEncoding::JPEG;
        output.width = static_cast<std::uint32_t>(width);
        output.height = static_cast<std::uint32_t>(height);
        output.hasAlpha = hasAlpha;
        return true;
    } @catch (...) { output = {}; return false; }
}

inline bool MeasureFaceImageRaster(NSData *data, FaceImageRasterInfo& output) {
    return MeasureFaceImageRaster(data, output, {});
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
