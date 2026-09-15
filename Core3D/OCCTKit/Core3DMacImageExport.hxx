#pragma once

#import <TargetConditionals.h>
#if TARGET_OS_OSX
// Carbon's legacy Handle callback type must be parsed without OCCT's
// function-like Handle macro. Restore the caller's exact macro afterward.
#pragma push_macro("Handle")
#undef Handle
#import <Foundation/Foundation.h>
#import <CoreImage/CoreImage.h>
#import <ImageIO/ImageIO.h>
#pragma pop_macro("Handle")
#include <Image_PixMap.hxx>

// Synchronous thumbnail encoding. The caller owns the OCCT RGB buffer for
// this entire operation. Normalize row orientation before creating the PNG.
inline NSData* Core3DMacPNGData(const Image_PixMap& image, bool adjustGamma)
{
    if (image.IsEmpty() || image.Format() != Image_Format_RGB) return nil;
    CGColorSpaceRef colorSpace = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    if (colorSpace == nullptr) return nil;
    CGDataProviderRef provider = CGDataProviderCreateWithData(
        nullptr, image.Data(), image.SizeBytes(), nullptr);
    if (provider == nullptr) { CGColorSpaceRelease(colorSpace); return nil; }
    CGImageRef source = CGImageCreate(image.SizeX(), image.SizeY(), 8, 24,
        image.SizeRowBytes(), colorSpace, kCGImageAlphaNone, provider,
        nullptr, false, kCGRenderingIntentDefault);
    if (source == nullptr) {
        CGDataProviderRelease(provider); CGColorSpaceRelease(colorSpace); return nil;
    }
    CIImage* content = [CIImage imageWithCGImage:source];
    if (!image.IsTopDown()) {
        content = [content imageByApplyingCGOrientation:kCGImagePropertyOrientationDownMirrored];
    }
    if (adjustGamma) {
        content = [content imageByApplyingFilter:@"CIGammaAdjust"
                             withInputParameters:@{@"inputPower": @2.0}];
    }
    CIContext* encoder = [CIContext contextWithOptions:nil];
    CGImageRef normalized = content == nil ? nullptr :
        [encoder createCGImage:content fromRect:content.extent
                        format:kCIFormatRGBA8 colorSpace:colorSpace];
    NSMutableData* data = [NSMutableData data];
    CGImageDestinationRef destination = normalized == nullptr ? nullptr :
        CGImageDestinationCreateWithData((__bridge CFMutableDataRef)data,
                                         CFSTR("public.png"), 1, nullptr);
    bool encoded = false;
    if (destination != nullptr) {
        CGImageDestinationAddImage(destination, normalized, nullptr);
        encoded = CGImageDestinationFinalize(destination);
        CFRelease(destination);
    }
    if (normalized != nullptr) CGImageRelease(normalized);
    CGImageRelease(source);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);
    return encoded ? data : nil;
}
#endif
