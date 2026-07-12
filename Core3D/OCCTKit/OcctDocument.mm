// Copyright (c) 2017 OPEN CASCADE SAS
//
// This file is part of the examples of the Open CASCADE Technology software library.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE

#import <Foundation/Foundation.h>
#import <ImageIO/ImageIO.h>
#import <CommonCrypto/CommonDigest.h>

#include "OcctDocument.h"
#include "CafShapePrs.h"

#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Message_ProgressRange.hxx>
#include <Message_ProgressScope.hxx>

#include <TCollection_AsciiString.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TDataStd_Integer.hxx>
#include <BinDrivers_DocumentStorageDriver.hxx>
#include <BinXCAFDrivers_DocumentStorageDriver.hxx>
#include <BinDrivers_DocumentRetrievalDriver.hxx>
#include <BinMDF_ADriverTable.hxx>
#include <BinMDF_TagSourceDriver.hxx>
#include <BinMDataStd_AsciiStringDriver.hxx>
#include <BinMDataStd_GenericEmptyDriver.hxx>
#include <BinMDataStd_GenericExtStringDriver.hxx>
#include <BinMDataStd_IntegerDriver.hxx>
#include <BinMDataStd_RealDriver.hxx>
#include <BinMDataStd_TreeNodeDriver.hxx>
#include <BinMDataStd_UAttributeDriver.hxx>
#include <BinMNaming_NamedShapeDriver.hxx>
#include <BinMXCAFDoc_ColorDriver.hxx>
#include <BinMXCAFDoc_LengthUnitDriver.hxx>
#include <BinMXCAFDoc_LocationDriver.hxx>
#include <BinMXCAFDoc_VisMaterialDriver.hxx>
#include <BinMXCAFDoc_VisMaterialToolDriver.hxx>
#include <BinObjMgt_Persistent.hxx>
#include <Storage_TypeData.hxx>

#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc.hxx>
#include <XCAFDoc_VisMaterial.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>
#include <TDataStd_Real.hxx>
#include <TDataStd_TreeNode.hxx>
#include <TDF_ChildIterator.hxx>
#include <gp_Trsf.hxx>
#include <GP_Quaternion.hxx>
#include <TNaming.hxx>
#include <Standard_GUID.hxx>
#include <TDF_LabelMap.hxx>
#include <XCAFPrs_DocumentExplorer.hxx>
#include <Graphic3d_TextureSet.hxx>
#include <Image_PixMap.hxx>
#include <Image_Texture.hxx>
#include <NCollection_Buffer.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <algorithm>
#include <cmath>
#include <array>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

IMPLEMENT_STANDARD_RTTIEXT(OcctDocument, Standard_Transient)

namespace {

Handle(Graphic3d_AspectFillArea3d) ClearDrawerTextureMapping(
    const Handle(Prs3d_Drawer)& theDrawer)
{
    if (theDrawer.IsNull()) {
        return {};
    }
    theDrawer->SetupOwnShadingAspect();
    const Handle(Prs3d_ShadingAspect)& aShading =
        theDrawer->ShadingAspect();
    if (aShading.IsNull() || aShading->Aspect().IsNull()) {
        return {};
    }
    // XCAFDoc_VisMaterial::FillAspect() deliberately does not clear an
    // existing texture set when the new material has no maps. Reset both the
    // enable bit and the owning handle before every full material transition.
    aShading->Aspect()->SetTextureMapOff();
    aShading->Aspect()->SetTextureSet(
        Handle(Graphic3d_TextureSet)());
    return aShading->Aspect();
}

void ResetDrawerForLegacyMaterial(
    const Handle(Prs3d_Drawer)& theDrawer)
{
    const Handle(Graphic3d_AspectFillArea3d) anAspect =
        ClearDrawerTextureMapping(theDrawer);
    if (anAspect.IsNull()) {
        return;
    }
    // Graphic3d_Aspects and XCAFDoc_VisMaterial define this as the legacy
    // compatibility contract. BlendAuto resolves an opaque preset to opaque
    // and a transparent preset to blending, while Auto culls only closed,
    // opaque groups. This is the same resolution used by the Metal snapshot.
    anAspect->SetAlphaMode(Graphic3d_AlphaMode_BlendAuto, 0.5f);
    anAspect->SetFaceCulling(
        Graphic3d_TypeOfBackfacingModel_Auto);
}

void ApplyVisualMaterialToPlainPresentation(
    const Handle(XCAFDoc_VisMaterial)& theMaterial,
    const Handle(AIS_Shape)& thePresentation)
{
    if (theMaterial.IsNull() || thePresentation.IsNull()) {
        return;
    }
    Graphic3d_MaterialAspect anAspect;
    theMaterial->FillMaterialAspect(anAspect);
    // Keep AIS' own-material and own-color state intact for selection,
    // duplication, and legacy callers, while installing the renderer-facing
    // texture set through OCCT's native Image_Texture bridge.
    thePresentation->SetMaterial(anAspect);
    thePresentation->SetColor(theMaterial->BaseColor().GetRGB());
    const Handle(Graphic3d_AspectFillArea3d) aFillAspect =
        ClearDrawerTextureMapping(thePresentation->Attributes());
    if (aFillAspect.IsNull()) {
        thePresentation->SynchronizeAspects();
        return;
    }
    theMaterial->FillAspect(aFillAspect);
    thePresentation->SynchronizeAspects();
}

constexpr Standard_Integer kMaximumVisualMaterialDefinitions = 2048;
constexpr Standard_Size kMaximumDocumentLabels = 100000;
constexpr Standard_Real kDefaultMetersPerUnit = 0.001;
constexpr Standard_Real kMaximumEmissionFactor = 65504.0;
constexpr Standard_Size kMaximumEmbeddedTextureBytes =
    32ull * 1024ull * 1024ull;
constexpr Standard_Size kMaximumAggregateTextureBytes =
    128ull * 1024ull * 1024ull;
constexpr std::uint64_t kMaximumTextureDimension = 8192;
constexpr std::uint64_t kMaximumTexturePixels = 4096ull * 4096ull;
constexpr Standard_Size kMaximumDecodedTextureBytes =
    128ull * 1024ull * 1024ull;
constexpr Standard_Integer kMaximumPersistentTextureIdentifierBytes = 256;
constexpr Standard_Integer kMaximumPersistentNameCharacters = 4096;
constexpr Standard_Integer kPersistentRecordHeaderBytes =
    3 * static_cast<Standard_Integer>(sizeof(Standard_Integer));
constexpr const char* kBufferTexturePrefix = "texturebuf://";
thread_local bool gSafeBinaryReadRejected = false;

Handle(Image_Texture) MakeRendererDecodableTexture(
    const Handle(NCollection_Buffer)& theBuffer,
    const TCollection_AsciiString& theIdentifier);

void RejectSafeBinaryRead() noexcept
{
    gSafeBinaryReadRejected = true;
}

bool TryPersistentRecordEnd(
    const BinObjMgt_Persistent& theSource,
    Standard_Integer& theRecordEnd)
{
    const Standard_Integer aLength = theSource.Length();
    if (aLength < 0
        || aLength
            > std::numeric_limits<Standard_Integer>::max()
                - kPersistentRecordHeaderBytes) {
        return false;
    }
    theRecordEnd = kPersistentRecordHeaderBytes + aLength;
    return true;
}

bool ReadBoundedPersistentAsciiString(
    const BinObjMgt_Persistent& theSource,
    std::string& theValue)
{
    theValue.clear();
    const Standard_Integer aPosition = theSource.Position();
    Standard_Integer aRecordEnd = 0;
    if (!TryPersistentRecordEnd(theSource, aRecordEnd)
        || aPosition < kPersistentRecordHeaderBytes
        || aPosition > aRecordEnd
        || aPosition
            > std::numeric_limits<Standard_Integer>::max() - 3) {
        return false;
    }
    const Standard_Integer anAlignedPosition =
        (aPosition + 3) & ~Standard_Integer(3);
    if (anAlignedPosition > aRecordEnd
        || !theSource.SetPosition(anAlignedPosition)) {
        return false;
    }

    for (Standard_Integer anIndex = 0;
         anIndex <= kMaximumPersistentTextureIdentifierBytes;
         ++anIndex) {
        if (theSource.Position() >= aRecordEnd) {
            return false;
        }
        Standard_Character aCharacter = '\0';
        if (!theSource.GetCharacter(aCharacter).IsOK()) {
            return false;
        }
        if (aCharacter == '\0') {
            return true;
        }
        if (anIndex == kMaximumPersistentTextureIdentifierBytes) {
            return false;
        }
        theValue.push_back(aCharacter);
    }
    return false;
}

bool StripBufferTexturePrefixes(std::string& theIdentifier)
{
    const std::string aPrefix(kBufferTexturePrefix);
    while (theIdentifier.rfind(aPrefix, 0) == 0) {
        theIdentifier.erase(0, aPrefix.size());
    }
    return !theIdentifier.empty()
        && theIdentifier.size()
            <= static_cast<std::size_t>(
                kMaximumPersistentTextureIdentifierBytes
                - aPrefix.size());
}

bool PreflightBoundedPersistentExtendedString(
    const BinObjMgt_Persistent& theSource)
{
    const Standard_Integer aStart = theSource.Position();
    Standard_Integer aRecordEnd = 0;
    if (!TryPersistentRecordEnd(theSource, aRecordEnd)
        || aStart < kPersistentRecordHeaderBytes
        || aStart > aRecordEnd
        || aStart
            > std::numeric_limits<Standard_Integer>::max() - 3) {
        return false;
    }
    const Standard_Integer anAlignedPosition =
        (aStart + 3) & ~Standard_Integer(3);
    if (anAlignedPosition > aRecordEnd
        || !theSource.SetPosition(anAlignedPosition)) {
        return false;
    }
    for (Standard_Integer anIndex = 0;
         anIndex <= kMaximumPersistentNameCharacters;
         ++anIndex) {
        if (theSource.Position() > aRecordEnd
                - static_cast<Standard_Integer>(
                    sizeof(Standard_ExtCharacter))) {
            return false;
        }
        Standard_ExtCharacter aCharacter = 0;
        if (!theSource.GetExtCharacter(aCharacter).IsOK()) {
            return false;
        }
        if (aCharacter == 0) {
            return theSource.SetPosition(aStart);
        }
        if (anIndex == kMaximumPersistentNameCharacters) {
            return false;
        }
    }
    return false;
}

bool PreflightEmbeddedTexture(
    const BinObjMgt_Persistent& theSource,
    Standard_Size& theAggregateBytes)
{
    std::string anIdentifier;
    if (!ReadBoundedPersistentAsciiString(theSource, anIdentifier)) {
        return false;
    }
    if (anIdentifier.empty()) {
        return true;
    }

    Standard_Boolean usesBuffer = Standard_False;
    if (!theSource.GetBoolean(usesBuffer).IsOK() || !usesBuffer
        || !StripBufferTexturePrefixes(anIdentifier)) {
        // External paths and offsets are deliberately unsupported for
        // self-contained, sandbox-safe projects.
        return false;
    }

    Standard_Integer aLength = 0;
    if (!theSource.GetInteger(aLength).IsOK()
        || aLength <= 0
        || static_cast<Standard_Size>(aLength)
            > kMaximumEmbeddedTextureBytes) {
        return false;
    }
    const Standard_Integer aPosition = theSource.Position();
    Standard_Integer aRecordEnd = 0;
    if (!TryPersistentRecordEnd(theSource, aRecordEnd)
        || aPosition < kPersistentRecordHeaderBytes
        || aPosition > aRecordEnd
        || aLength > aRecordEnd - aPosition
        || theAggregateBytes > kMaximumAggregateTextureBytes
        || static_cast<Standard_Size>(aLength)
            > kMaximumAggregateTextureBytes - theAggregateBytes) {
        return false;
    }
    theAggregateBytes += static_cast<Standard_Size>(aLength);
    return theSource.SetPosition(aPosition + aLength);
}

bool SkipShortReals(
    const BinObjMgt_Persistent& theSource,
    const Standard_Integer theCount)
{
    for (Standard_Integer anIndex = 0; anIndex < theCount; ++anIndex) {
        Standard_ShortReal aValue = 0.0f;
        if (!theSource.GetShortReal(aValue).IsOK()) {
            return false;
        }
    }
    return true;
}

bool NormalizeEmbeddedTexture(Handle(Image_Texture)& theTexture)
{
    if (theTexture.IsNull()) {
        return true;
    }
    const Handle(NCollection_Buffer)& aBuffer = theTexture->DataBuffer();
    if (aBuffer.IsNull()) {
        return false;
    }
    std::string anIdentifier(theTexture->TextureId().ToCString());
    if (!StripBufferTexturePrefixes(anIdentifier)) {
        return false;
    }
    theTexture = MakeRendererDecodableTexture(
        aBuffer, TCollection_AsciiString(anIdentifier.c_str()));
    return true;
}

bool IsPNGSignature(const Standard_Byte* theBytes,
                    const Standard_Size theSize)
{
    return theBytes != nullptr && theSize >= 8
        && std::memcmp(theBytes, "\x89PNG\r\n\x1A\n", 8) == 0;
}

bool IsJPEGSignature(const Standard_Byte* theBytes,
                     const Standard_Size theSize)
{
    return theBytes != nullptr && theSize >= 3
        && theBytes[0] == 0xFF && theBytes[1] == 0xD8
        && theBytes[2] == 0xFF;
}

bool HasSupportedRasterSignature(const Standard_Byte* theBytes,
                                 const Standard_Size theSize)
{
    if (theBytes == nullptr) {
        return false;
    }
    return IsPNGSignature(theBytes, theSize)
        || IsJPEGSignature(theBytes, theSize)
        || (theSize >= 6
            && (std::memcmp(theBytes, "GIF87a", 6) == 0
                || std::memcmp(theBytes, "GIF89a", 6) == 0))
        || (theSize >= 4
            && (std::memcmp(theBytes, "II\x2A\x00", 4) == 0
                || std::memcmp(theBytes, "MM\x00\x2A", 4) == 0))
        || (theSize >= 2 && std::memcmp(theBytes, "BM", 2) == 0)
        || (theSize >= 12
            && std::memcmp(theBytes, "RIFF", 4) == 0
            && std::memcmp(theBytes + 8, "WEBP", 4) == 0);
}

bool TryRendererTextureDimensions(
    const std::uint64_t theSourceWidth,
    const std::uint64_t theSourceHeight,
    std::uint64_t& theRendererWidth,
    std::uint64_t& theRendererHeight)
{
    theRendererWidth = 0;
    theRendererHeight = 0;
    if (theSourceWidth == 0 || theSourceHeight == 0
        || theSourceWidth > kMaximumTextureDimension
        || theSourceHeight > kMaximumTextureDimension
        || theSourceWidth > kMaximumTexturePixels / theSourceHeight) {
        return false;
    }
    const auto previousPowerOfTwo = [](const std::uint64_t theValue) {
        std::uint64_t aResult = 1;
        while (aResult <= theValue / 2) {
            aResult *= 2;
        }
        return aResult;
    };
    const std::uint64_t aPreviousWidth =
        previousPowerOfTwo(theSourceWidth);
    const std::uint64_t aPreviousHeight =
        previousPowerOfTwo(theSourceHeight);
    const std::uint64_t aNextWidth = aPreviousWidth == theSourceWidth
        ? theSourceWidth : aPreviousWidth * 2;
    const std::uint64_t aNextHeight = aPreviousHeight == theSourceHeight
        ? theSourceHeight : aPreviousHeight * 2;
    const std::uint64_t aMaximumDecodedPixels =
        static_cast<std::uint64_t>(kMaximumDecodedTextureBytes) / 4;
    const bool canUseNextPowerOfTwo =
        aNextWidth <= kMaximumTextureDimension
        && aNextHeight <= kMaximumTextureDimension
        && aNextWidth <= kMaximumTexturePixels / aNextHeight
        && aNextWidth <= aMaximumDecodedPixels / aNextHeight;
    theRendererWidth = canUseNextPowerOfTwo
        ? aNextWidth : aPreviousWidth;
    theRendererHeight = canUseNextPowerOfTwo
        ? aNextHeight : aPreviousHeight;
    return theRendererWidth > 0 && theRendererHeight > 0
        && theRendererWidth <= kMaximumTexturePixels / theRendererHeight
        && theRendererWidth <= aMaximumDecodedPixels / theRendererHeight;
}

bool TryTextureDecodedBytes(
    const Handle(NCollection_Buffer)& theBuffer,
    Standard_Size& theDecodedBytes)
{
    theDecodedBytes = 0;
    if (theBuffer.IsNull() || theBuffer->Data() == nullptr
        || theBuffer->Size() == 0
        || !HasSupportedRasterSignature(
            theBuffer->Data(), theBuffer->Size())) {
        return false;
    }
    CFDataRef data = CFDataCreateWithBytesNoCopy(
        kCFAllocatorDefault,
        reinterpret_cast<const UInt8*>(theBuffer->Data()),
        static_cast<CFIndex>(theBuffer->Size()),
        kCFAllocatorNull);
    if (data == nullptr) {
        return false;
    }
    const void* optionKeys[] = {kCGImageSourceShouldCache};
    const void* optionValues[] = {kCFBooleanFalse};
    CFDictionaryRef options = CFDictionaryCreate(
        kCFAllocatorDefault,
        optionKeys,
        optionValues,
        1,
        &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);
    CGImageSourceRef source = CGImageSourceCreateWithData(data, options);
    if (options != nullptr) {
        CFRelease(options);
    }
    CFRelease(data);
    if (source == nullptr || CGImageSourceGetType(source) == nullptr
        || CGImageSourceGetCount(source) != 1
        || CGImageSourceGetStatus(source) != kCGImageStatusComplete
        || CGImageSourceGetStatusAtIndex(source, 0)
            != kCGImageStatusComplete) {
        if (source != nullptr) {
            CFRelease(source);
        }
        return false;
    }

    CFDictionaryRef properties =
        CGImageSourceCopyPropertiesAtIndex(source, 0, nullptr);
    CFRelease(source);
    if (properties == nullptr) {
        return false;
    }
    const CFTypeRef widthValue = CFDictionaryGetValue(
        properties, kCGImagePropertyPixelWidth);
    const CFTypeRef heightValue = CFDictionaryGetValue(
        properties, kCGImagePropertyPixelHeight);
    std::int64_t width = 0;
    std::int64_t height = 0;
    std::uint64_t rendererWidth = 0;
    std::uint64_t rendererHeight = 0;
    const bool isValid = widthValue != nullptr
        && heightValue != nullptr
        && CFGetTypeID(widthValue) == CFNumberGetTypeID()
        && CFGetTypeID(heightValue) == CFNumberGetTypeID()
        && CFNumberGetValue(
            static_cast<CFNumberRef>(widthValue),
            kCFNumberSInt64Type, &width)
        && CFNumberGetValue(
            static_cast<CFNumberRef>(heightValue),
            kCFNumberSInt64Type, &height)
        && width > 0 && height > 0
        && TryRendererTextureDimensions(
            static_cast<std::uint64_t>(width),
            static_cast<std::uint64_t>(height),
            rendererWidth, rendererHeight);
    if (isValid) {
        const std::uint64_t pixels =
            rendererWidth * rendererHeight;
        if (pixels
            <= std::numeric_limits<Standard_Size>::max() / 4) {
            theDecodedBytes =
                static_cast<Standard_Size>(pixels * 4);
        }
    }
    CFRelease(properties);
    return isValid && theDecodedBytes > 0
        && theDecodedBytes <= kMaximumDecodedTextureBytes;
}

Handle(Image_PixMap) DecodeRendererTextureWithImageIO(
    const Handle(NCollection_Buffer)& theBuffer)
{
    Standard_Size aPreflightDecodedBytes = 0;
    if (!TryTextureDecodedBytes(
            theBuffer, aPreflightDecodedBytes)
        || theBuffer.IsNull() || theBuffer->Data() == nullptr) {
        return {};
    }

    CFDataRef aData = CFDataCreateWithBytesNoCopy(
        kCFAllocatorDefault,
        reinterpret_cast<const UInt8*>(theBuffer->Data()),
        static_cast<CFIndex>(theBuffer->Size()),
        kCFAllocatorNull);
    if (aData == nullptr) {
        return {};
    }
    const void* aSourceKeys[] = {
        kCGImageSourceShouldCache,
    };
    const void* aSourceValues[] = {
        kCFBooleanFalse,
    };
    CFDictionaryRef aSourceOptions = CFDictionaryCreate(
        kCFAllocatorDefault,
        aSourceKeys,
        aSourceValues,
        1,
        &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);
    CGImageSourceRef aSource =
        CGImageSourceCreateWithData(aData, aSourceOptions);
    if (aSourceOptions != nullptr) {
        CFRelease(aSourceOptions);
    }
    CFRelease(aData);
    if (aSource == nullptr
        || CGImageSourceGetCount(aSource) != 1
        || CGImageSourceGetStatus(aSource)
            != kCGImageStatusComplete
        || CGImageSourceGetStatusAtIndex(aSource, 0)
            != kCGImageStatusComplete) {
        if (aSource != nullptr) {
            CFRelease(aSource);
        }
        return {};
    }

    // Thumbnail creation at the already-validated maximum dimension is the
    // ImageIO API that applies EXIF orientation without ever allocating an
    // unbounded intermediate. Valid sources remain at their native size.
    std::int64_t aMaximumDimension =
        static_cast<std::int64_t>(kMaximumTextureDimension);
    CFNumberRef aMaximumDimensionNumber = CFNumberCreate(
        kCFAllocatorDefault,
        kCFNumberSInt64Type,
        &aMaximumDimension);
    if (aMaximumDimensionNumber == nullptr) {
        CFRelease(aSource);
        return {};
    }
    const void* aDecodeKeys[] = {
        kCGImageSourceCreateThumbnailFromImageAlways,
        kCGImageSourceCreateThumbnailWithTransform,
        kCGImageSourceThumbnailMaxPixelSize,
        kCGImageSourceShouldCacheImmediately,
    };
    const void* aDecodeValues[] = {
        kCFBooleanTrue,
        kCFBooleanTrue,
        aMaximumDimensionNumber,
        kCFBooleanTrue,
    };
    CFDictionaryRef aDecodeOptions = CFDictionaryCreate(
        kCFAllocatorDefault,
        aDecodeKeys,
        aDecodeValues,
        4,
        &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);
    CGImageRef anImage = aDecodeOptions == nullptr
        ? nullptr
        : CGImageSourceCreateThumbnailAtIndex(
            aSource, 0, aDecodeOptions);
    if (aDecodeOptions != nullptr) {
        CFRelease(aDecodeOptions);
    }
    CFRelease(aMaximumDimensionNumber);
    CFRelease(aSource);
    if (anImage == nullptr) {
        return {};
    }

    const std::uint64_t aSourceWidth = CGImageGetWidth(anImage);
    const std::uint64_t aSourceHeight = CGImageGetHeight(anImage);
    std::uint64_t aWidth = 0;
    std::uint64_t aHeight = 0;
    if (!TryRendererTextureDimensions(
            aSourceWidth, aSourceHeight,
            aWidth, aHeight)) {
        CGImageRelease(anImage);
        return {};
    }

    Handle(Image_PixMap) aPixMap = new Image_PixMap();
    if (aPixMap.IsNull()
        || !aPixMap->InitTrash(
            Image_Format_RGBA,
            static_cast<Standard_Size>(aWidth),
            static_cast<Standard_Size>(aHeight),
            static_cast<Standard_Size>(aWidth * 4))) {
        CGImageRelease(anImage);
        return {};
    }
    // CGBitmapContext writes the first scanline as the visual top row for a
    // CGImage draw. OCCT uses this flag to apply the matching OpenGL V flip.
    aPixMap->SetTopDown(true);

    CGColorSpaceRef aColorSpace =
        CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    const CGBitmapInfo aBitmapInfo = static_cast<CGBitmapInfo>(
        kCGImageAlphaPremultipliedLast
        | kCGBitmapByteOrder32Big);
    CGContextRef aContext = aColorSpace == nullptr
        ? nullptr
        : CGBitmapContextCreate(
            aPixMap->ChangeData(),
            static_cast<size_t>(aWidth),
            static_cast<size_t>(aHeight),
            8,
            aPixMap->SizeRowBytes(),
            aColorSpace,
            aBitmapInfo);
    if (aColorSpace != nullptr) {
        CGColorSpaceRelease(aColorSpace);
    }
    if (aContext == nullptr) {
        CGImageRelease(anImage);
        return {};
    }
    CGContextSetBlendMode(aContext, kCGBlendModeCopy);
    CGContextSetInterpolationQuality(aContext, kCGInterpolationHigh);
    // OCCT's OpenGL ES 2 texture wrapper unconditionally requests mipmaps.
    // Resampling only NPOT axes into a bounded POT backing avoids an
    // incomplete/black GPU texture while keeping the entire source image in
    // the same normalized UV domain. Already-POT dimensions remain unchanged.
    CGContextDrawImage(
        aContext,
        CGRectMake(
            0.0, 0.0,
            static_cast<CGFloat>(aWidth),
            static_cast<CGFloat>(aHeight)),
        anImage);
    CGContextFlush(aContext);
    CGContextRelease(aContext);
    CGImageRelease(anImage);

    // CoreGraphics' supported RGBA bitmap layout is premultiplied. OCCT's
    // base-color texture contract is straight alpha, so undo the premultiply
    // explicitly; otherwise translucent texels are darkened a second time by
    // the PBR shader. Fully transparent RGB is canonicalized to zero.
    for (Standard_Size aRow = 0; aRow < aPixMap->SizeY(); ++aRow) {
        Standard_Byte* aPixel = aPixMap->ChangeRow(aRow);
        for (Standard_Size aColumn = 0;
             aColumn < aPixMap->SizeX();
             ++aColumn, aPixel += 4) {
            const unsigned int anAlpha = aPixel[3];
            if (anAlpha == 0) {
                aPixel[0] = 0;
                aPixel[1] = 0;
                aPixel[2] = 0;
                continue;
            }
            if (anAlpha == 255) {
                continue;
            }
            for (Standard_Integer aChannel = 0;
                 aChannel < 3;
                 ++aChannel) {
                const unsigned int aStraight =
                    (static_cast<unsigned int>(aPixel[aChannel])
                        * 255u + anAlpha / 2u) / anAlpha;
                aPixel[aChannel] = static_cast<Standard_Byte>(
                    std::min(aStraight, 255u));
            }
        }
    }
    return aPixMap;
}

class Core3DImageIOTexture final : public Image_Texture {
    DEFINE_STANDARD_RTTI_INLINE(
        Core3DImageIOTexture,
        Image_Texture)

public:
    Core3DImageIOTexture(
        const Handle(NCollection_Buffer)& theBuffer,
        const TCollection_AsciiString& theIdentifier)
    : Image_Texture(theBuffer, theIdentifier) {
    }

    Handle(Image_PixMap) ReadImage(
        const Handle(Image_SupportedFormats)&) const override
    {
        return DecodeRendererTextureWithImageIO(DataBuffer());
    }
};

Handle(Image_Texture) MakeRendererDecodableTexture(
    const Handle(NCollection_Buffer)& theBuffer,
    const TCollection_AsciiString& theIdentifier)
{
    if (theBuffer.IsNull() || theBuffer->Data() == nullptr
        || theBuffer->Size() == 0 || theIdentifier.IsEmpty()) {
        return {};
    }
    // Both authored textures and safe-loaded/import-artifact textures pass
    // through this factory before their first presentation. A binary update
    // also recreates the GL context, so an old failed GPU-resource cache can
    // never outlive the source upgrade performed here.
    return new Core3DImageIOTexture(theBuffer, theIdentifier);
}

bool ValidateAuthoredRasterBytes(const Standard_Byte* theBytes,
                                 const Standard_Size theSize,
                                 const std::string* theMediaType)
{
    if (theBytes == nullptr || theSize == 0
        || theSize > kMaximumEmbeddedTextureBytes) {
        return false;
    }
    const bool isPNG = IsPNGSignature(theBytes, theSize);
    const bool isJPEG = IsJPEGSignature(theBytes, theSize);
    if (!isPNG && !isJPEG) {
        return false;
    }
    if (theMediaType != nullptr
        && !((*theMediaType == "image/png" && isPNG)
             || (*theMediaType == "image/jpeg" && isJPEG))) {
        return false;
    }

    CFDataRef data = CFDataCreateWithBytesNoCopy(
        kCFAllocatorDefault,
        reinterpret_cast<const UInt8*>(theBytes),
        static_cast<CFIndex>(theSize),
        kCFAllocatorNull);
    if (data == nullptr) {
        return false;
    }
    const void* optionKeys[] = {kCGImageSourceShouldCache};
    const void* optionValues[] = {kCFBooleanFalse};
    CFDictionaryRef options = CFDictionaryCreate(
        kCFAllocatorDefault,
        optionKeys,
        optionValues,
        1,
        &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);
    CGImageSourceRef source = CGImageSourceCreateWithData(data, options);
    if (options != nullptr) {
        CFRelease(options);
    }
    CFRelease(data);
    if (source == nullptr || CGImageSourceGetType(source) == nullptr
        || CGImageSourceGetCount(source) != 1
        || CGImageSourceGetStatus(source) != kCGImageStatusComplete
        || CGImageSourceGetStatusAtIndex(source, 0)
            != kCGImageStatusComplete) {
        if (source != nullptr) {
            CFRelease(source);
        }
        return false;
    }

    CFDictionaryRef properties =
        CGImageSourceCopyPropertiesAtIndex(source, 0, nullptr);
    CFRelease(source);
    if (properties == nullptr) {
        return false;
    }
    const CFTypeRef widthValue = CFDictionaryGetValue(
        properties, kCGImagePropertyPixelWidth);
    const CFTypeRef heightValue = CFDictionaryGetValue(
        properties, kCGImagePropertyPixelHeight);
    const CFTypeRef depthValue = CFDictionaryGetValue(
        properties, kCGImagePropertyDepth);
    std::int64_t width = 0;
    std::int64_t height = 0;
    std::int64_t depth = 8;
    const bool hasValidDepth = depthValue == nullptr
        || (CFGetTypeID(depthValue) == CFNumberGetTypeID()
            && CFNumberGetValue(
                static_cast<CFNumberRef>(depthValue),
                kCFNumberSInt64Type,
                &depth));
    const bool isValid = widthValue != nullptr
        && heightValue != nullptr
        && CFGetTypeID(widthValue) == CFNumberGetTypeID()
        && CFGetTypeID(heightValue) == CFNumberGetTypeID()
        && CFNumberGetValue(
            static_cast<CFNumberRef>(widthValue),
            kCFNumberSInt64Type,
            &width)
        && CFNumberGetValue(
            static_cast<CFNumberRef>(heightValue),
            kCFNumberSInt64Type,
            &height)
        && hasValidDepth && depth > 0 && depth <= 8
        && width > 0 && height > 0
        && static_cast<std::uint64_t>(width)
            <= kMaximumTextureDimension
        && static_cast<std::uint64_t>(height)
            <= kMaximumTextureDimension
        && static_cast<std::uint64_t>(width)
            <= kMaximumTexturePixels
                / static_cast<std::uint64_t>(height);
    bool isWithinDecodedBudget = false;
    if (isValid) {
        const std::uint64_t pixels = static_cast<std::uint64_t>(width)
            * static_cast<std::uint64_t>(height);
        isWithinDecodedBudget = pixels
            <= static_cast<std::uint64_t>(kMaximumDecodedTextureBytes) / 4;
    }
    CFRelease(properties);
    return isValid && isWithinDecodedBudget;
}

std::string AuthoredTextureIdentifier(const Standard_Byte* theBytes,
                                      const Standard_Size theSize)
{
    if (theBytes == nullptr || theSize == 0
        || theSize > std::numeric_limits<CC_LONG>::max()) {
        return {};
    }
    std::array<unsigned char, CC_SHA256_DIGEST_LENGTH> digest = {};
    if (CC_SHA256(theBytes, static_cast<CC_LONG>(theSize), digest.data())
        == nullptr) {
        return {};
    }
    std::ostringstream stream;
    stream << "texture-sha256-" << std::hex << std::setfill('0');
    for (const unsigned char byte : digest) {
        stream << std::setw(2) << static_cast<unsigned int>(byte);
    }
    return stream.str();
}

template <typename Driver>
class Core3DFailClosedDriver final : public Driver
{
public:
    explicit Core3DFailClosedDriver(
        const Handle(Message_Messenger)& theMessageDriver)
    : Driver(theMessageDriver)
    {
    }

    Standard_Boolean Paste(
        const BinObjMgt_Persistent& theSource,
        const Handle(TDF_Attribute)& theTarget,
        BinObjMgt_RRelocationTable& theRelocationTable) const override
    {
        const Standard_Boolean succeeded = Driver::Paste(
            theSource, theTarget, theRelocationTable);
        if (!succeeded) {
            RejectSafeBinaryRead();
        }
        return succeeded;
    }
};

class Core3DBoundedAsciiStringDriver final
    : public BinMDataStd_AsciiStringDriver
{
public:
    explicit Core3DBoundedAsciiStringDriver(
        const Handle(Message_Messenger)& theMessageDriver)
    : BinMDataStd_AsciiStringDriver(theMessageDriver)
    {
    }

    Standard_Boolean Paste(
        const BinObjMgt_Persistent& theSource,
        const Handle(TDF_Attribute)& theTarget,
        BinObjMgt_RRelocationTable& theRelocationTable) const override
    {
        const Standard_Integer aStart = theSource.Position();
        std::string aValue;
        if (!ReadBoundedPersistentAsciiString(theSource, aValue)
            || !theSource.SetPosition(aStart)) {
            RejectSafeBinaryRead();
            return Standard_False;
        }
        return BinMDataStd_AsciiStringDriver::Paste(
            theSource, theTarget, theRelocationTable);
    }
};

class Core3DBoundedExtendedStringDriver final
    : public BinMDataStd_GenericExtStringDriver
{
public:
    explicit Core3DBoundedExtendedStringDriver(
        const Handle(Message_Messenger)& theMessageDriver)
    : BinMDataStd_GenericExtStringDriver(theMessageDriver)
    {
    }

    Standard_Boolean Paste(
        const BinObjMgt_Persistent& theSource,
        const Handle(TDF_Attribute)& theTarget,
        BinObjMgt_RRelocationTable& theRelocationTable) const override
    {
        if (!PreflightBoundedPersistentExtendedString(theSource)) {
            RejectSafeBinaryRead();
            return Standard_False;
        }
        return BinMDataStd_GenericExtStringDriver::Paste(
            theSource, theTarget, theRelocationTable);
    }
};

class Core3DBoundedVisMaterialDriver final : public BinMDF_ADriver
{
public:
    Core3DBoundedVisMaterialDriver(
        const Handle(Message_Messenger)& theMessageDriver,
        const std::shared_ptr<Standard_Size>& theAggregateTextureBytes)
    : BinMDF_ADriver(
          theMessageDriver,
          STANDARD_TYPE(XCAFDoc_VisMaterial)->Name()),
      myDelegate(new BinMXCAFDoc_VisMaterialDriver(theMessageDriver)),
      myAggregateTextureBytes(theAggregateTextureBytes)
    {
    }

    Handle(TDF_Attribute) NewEmpty() const override
    {
        return new XCAFDoc_VisMaterial();
    }

    Standard_Boolean Paste(
        const BinObjMgt_Persistent& theSource,
        const Handle(TDF_Attribute)& theTarget,
        BinObjMgt_RRelocationTable& theRelocationTable) const override
    {
        const Standard_Integer aStart = theSource.Position();
        auto restoreStart = [&]() {
            return theSource.SetPosition(aStart);
        };
        Standard_Byte aMajor = 0;
        Standard_Byte aMinor = 0;
        Standard_Byte aFaceCulling = 0;
        Standard_Byte anAlphaMode = 0;
        Standard_Boolean hasPBR = Standard_False;
        Standard_Boolean hasCommon = Standard_False;
        Standard_Size anAggregate = myAggregateTextureBytes == nullptr
            ? 0
            : *myAggregateTextureBytes;

        bool isSafe = theSource.GetByte(aMajor).IsOK()
            && theSource.GetByte(aMinor).IsOK()
            && aMajor == 1 && aMinor <= 1
            && theSource.GetByte(aFaceCulling).IsOK()
            && theSource.GetByte(anAlphaMode).IsOK()
            && (aFaceCulling == static_cast<Standard_Byte>('0')
                || aFaceCulling == static_cast<Standard_Byte>('B')
                || aFaceCulling == static_cast<Standard_Byte>('F')
                || aFaceCulling == static_cast<Standard_Byte>('1'))
            && (anAlphaMode == static_cast<Standard_Byte>('O')
                || anAlphaMode == static_cast<Standard_Byte>('M')
                || anAlphaMode == static_cast<Standard_Byte>('B')
                || anAlphaMode == static_cast<Standard_Byte>('b')
                || anAlphaMode == static_cast<Standard_Byte>('A'))
            && SkipShortReals(theSource, 1)
            && theSource.GetBoolean(hasPBR).IsOK();
        if (isSafe && hasPBR) {
            isSafe = SkipShortReals(theSource, 4 + 3 + 2)
                && PreflightEmbeddedTexture(theSource, anAggregate)
                && PreflightEmbeddedTexture(theSource, anAggregate)
                && PreflightEmbeddedTexture(theSource, anAggregate)
                && PreflightEmbeddedTexture(theSource, anAggregate)
                && PreflightEmbeddedTexture(theSource, anAggregate);
        }
        if (isSafe) {
            isSafe = theSource.GetBoolean(hasCommon).IsOK();
        }
        if (isSafe && hasCommon) {
            isSafe = SkipShortReals(theSource, 3 * 4 + 2)
                && PreflightEmbeddedTexture(theSource, anAggregate);
        }
        if (isSafe && hasPBR && aMinor >= 1) {
            isSafe = SkipShortReals(theSource, 1);
        }
        if (!isSafe || !theSource.IsOK() || !restoreStart()) {
            restoreStart();
            RejectSafeBinaryRead();
            return Standard_False;
        }

        if (!myDelegate->Paste(
                theSource, theTarget, theRelocationTable)) {
            RejectSafeBinaryRead();
            return Standard_False;
        }
        const Handle(XCAFDoc_VisMaterial) aMaterial =
            Handle(XCAFDoc_VisMaterial)::DownCast(theTarget);
        if (aMaterial.IsNull()) {
            RejectSafeBinaryRead();
            return Standard_False;
        }
        if (aMaterial->HasPbrMaterial()) {
            XCAFDoc_VisMaterialPBR aPBR = aMaterial->PbrMaterial();
            if (!NormalizeEmbeddedTexture(aPBR.BaseColorTexture)
                || !NormalizeEmbeddedTexture(
                    aPBR.MetallicRoughnessTexture)
                || !NormalizeEmbeddedTexture(aPBR.EmissiveTexture)
                || !NormalizeEmbeddedTexture(aPBR.OcclusionTexture)
                || !NormalizeEmbeddedTexture(aPBR.NormalTexture)) {
                RejectSafeBinaryRead();
                return Standard_False;
            }
            aMaterial->SetPbrMaterial(aPBR);
        }
        if (aMaterial->HasCommonMaterial()) {
            XCAFDoc_VisMaterialCommon aCommon =
                aMaterial->CommonMaterial();
            if (!NormalizeEmbeddedTexture(aCommon.DiffuseTexture)) {
                RejectSafeBinaryRead();
                return Standard_False;
            }
            aMaterial->SetCommonMaterial(aCommon);
        }
        if (myAggregateTextureBytes != nullptr) {
            *myAggregateTextureBytes = anAggregate;
        }
        return Standard_True;
    }

    void Paste(
        const Handle(TDF_Attribute)& theSource,
        BinObjMgt_Persistent& theTarget,
        BinObjMgt_SRelocationTable& theRelocationTable) const override
    {
        myDelegate->Paste(theSource, theTarget, theRelocationTable);
    }

private:
    Handle(BinMXCAFDoc_VisMaterialDriver) myDelegate;
    std::shared_ptr<Standard_Size> myAggregateTextureBytes;
};

class Core3DBoundedBinXCAFRetrievalDriver final
    : public BinDrivers_DocumentRetrievalDriver
{
public:
    Core3DBoundedBinXCAFRetrievalDriver()
    : myAggregateTextureBytes(std::make_shared<Standard_Size>(0))
    {
    }

    void Read(
        Standard_IStream& theStream,
        const Handle(Storage_Data)& theStorageData,
        const Handle(CDM_Document)& theDocument,
        const Handle(CDM_Application)& theApplication,
        const Handle(PCDM_ReaderFilter)& theFilter =
            Handle(PCDM_ReaderFilter)(),
        const Message_ProgressRange& theProgress =
            Message_ProgressRange()) override
    {
        ResetAggregateTextureBytes();
        if (theStorageData.IsNull() || theStorageData->TypeData().IsNull()) {
            RejectSafeBinaryRead();
            return;
        }
        const Handle(TColStd_HSequenceOfAsciiString) aPersistentTypes =
            theStorageData->TypeData()->Types();
        if (aPersistentTypes.IsNull()
            || aPersistentTypes->Length() < 0
            || aPersistentTypes->Length() > 128) {
            RejectSafeBinaryRead();
            return;
        }
        TColStd_SequenceOfAsciiString aTypeNames;
        for (Standard_Integer anIndex = 1;
             anIndex <= aPersistentTypes->Length(); ++anIndex) {
            const TCollection_AsciiString& aName =
                aPersistentTypes->Value(anIndex);
            if (aName.IsEmpty() || aName.Length() > 128) {
                RejectSafeBinaryRead();
                return;
            }
            aTypeNames.Append(aName);
        }
        Handle(BinMDF_ADriverTable) aSupportedDrivers =
            AttributeDrivers(Message::DefaultMessenger());
        aSupportedDrivers->AssignIds(aTypeNames);
        for (Standard_Integer anIndex = 1;
             anIndex <= aTypeNames.Length(); ++anIndex) {
            if (aSupportedDrivers->GetDriver(anIndex).IsNull()) {
                RejectSafeBinaryRead();
                return;
            }
        }
        try {
            BinDrivers_DocumentRetrievalDriver::Read(
                theStream,
                theStorageData,
                theDocument,
                theApplication,
                theFilter,
                theProgress);
        } catch (...) {
            ResetAggregateTextureBytes();
            throw;
        }
        ResetAggregateTextureBytes();
    }

    Handle(BinMDF_ADriverTable) AttributeDrivers(
        const Handle(Message_Messenger)& theMessageDriver) override
    {
        // Project files intentionally support a narrow OCAF schema. Starting
        // with BinDrivers::AttributeDrivers() would expose unrelated array,
        // list, named-data, function, note, and geometric-constraint readers
        // that trust attacker-controlled element counts before app validation.
        Handle(BinMDF_ADriverTable) aTable = new BinMDF_ADriverTable();
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDF_TagSourceDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_GenericEmptyDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DBoundedExtendedStringDriver(theMessageDriver));
        aTable->AddDriver(
            new Core3DBoundedAsciiStringDriver(theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_IntegerDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_RealDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_TreeNodeDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMDataStd_UAttributeDriver>(
                theMessageDriver));
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMXCAFDoc_ColorDriver>(
                theMessageDriver));

        const Handle(BinMNaming_NamedShapeDriver) aNamedShapeDriver =
            new Core3DFailClosedDriver<BinMNaming_NamedShapeDriver>(
                theMessageDriver);
        aTable->AddDriver(aNamedShapeDriver);
        Handle(BinMXCAFDoc_LocationDriver) aLocationDriver =
            new Core3DFailClosedDriver<BinMXCAFDoc_LocationDriver>(
                theMessageDriver);
        aLocationDriver->SetNSDriver(aNamedShapeDriver);
        aTable->AddDriver(aLocationDriver);
        aTable->AddDriver(
            new Core3DFailClosedDriver<BinMXCAFDoc_LengthUnitDriver>(
                theMessageDriver));
        aTable->AddDriver(new Core3DBoundedVisMaterialDriver(
            theMessageDriver, myAggregateTextureBytes));
        aTable->AddDriver(
            new Core3DFailClosedDriver<
                BinMXCAFDoc_VisMaterialToolDriver>(theMessageDriver));
        return aTable;
    }

    void Clear() override
    {
        BinDrivers_DocumentRetrievalDriver::Clear();
        ResetAggregateTextureBytes();
    }

private:
    void ResetAggregateTextureBytes() noexcept
    {
        if (myAggregateTextureBytes != nullptr) {
            *myAggregateTextureBytes = 0;
        }
    }

    std::shared_ptr<Standard_Size> myAggregateTextureBytes;
};

// These GUIDs are persistent schema identifiers. They identify the attribute
// role; the UUID string stored in each attribute identifies the document,
// occurrence, or shared shape definition itself.
const Standard_GUID& DocumentIdentifierAttributeID()
{
    static const Standard_GUID anId("74386E4E-F620-498F-8092-E6D883AF33A4");
    return anId;
}

const Standard_GUID& EntityIdentifierAttributeID()
{
    static const Standard_GUID anId("0074F7C2-9EAA-4F89-B2DE-8716E155FF62");
    return anId;
}

const Standard_GUID& DefinitionIdentifierAttributeID()
{
    static const Standard_GUID anId("3611F2B2-C694-4E12-AED8-A2A97A3D283B");
    return anId;
}

//! Marks XCAF visualization material assignments authored by Shapeyard's PBR
//! editor. Imported XCAF styles remain untouched and legacy child-11/12 style
//! overrides retain their historical precedence until the user authors PBR.
const Standard_GUID& LocalPBRMaterialAttributeID()
{
    static const Standard_GUID anId("248A5203-4A22-4F2B-85C4-BE0BA89A5E4D");
    return anId;
}

//! Records that Shapeyard promoted the default black emissive factor to white
//! solely to make the first authored emissive texture visible. This lives on
//! the shape label so it follows OCAF history and appearance-copy operations.
const Standard_GUID& AutoPromotedEmissiveFactorAttributeID()
{
    static const Standard_GUID anId("1DA4580F-1B19-46DA-ABD4-BBCE9FBADE44");
    return anId;
}

//! Marks immutable table entries created by Shapeyard. Imported material
//! libraries must never be garbage-collected by local authoring operations.
const Standard_GUID& OwnedPBRMaterialDefinitionAttributeID()
{
    static const Standard_GUID anId("750690D2-357B-4101-B0EC-70DF20A2EC97");
    return anId;
}

void RemoveUnreferencedOwnedMaterial(
    const Handle(XCAFDoc_VisMaterialTool)& theTool,
    const TDF_Label& theMaterialLabel)
{
    if (theTool.IsNull() || theMaterialLabel.IsNull()) {
        return;
    }
    Handle(TDataStd_Integer) anOwnedMarker;
    if (!theMaterialLabel.FindAttribute(
            OwnedPBRMaterialDefinitionAttributeID(), anOwnedMarker)
        || anOwnedMarker.IsNull() || anOwnedMarker->Get() != 1) {
        return;
    }
    Handle(TDataStd_TreeNode) aReferenceRoot;
    if (theMaterialLabel.FindAttribute(
            XCAFDoc::VisMaterialRefGUID(), aReferenceRoot)
        && !aReferenceRoot.IsNull() && aReferenceRoot->HasFirst()) {
        return;
    }
    theTool->RemoveMaterial(theMaterialLabel);
}

Standard_Boolean IsOwnedMaterialDefinition(
    const TDF_Label& theMaterialLabel)
{
    if (theMaterialLabel.IsNull()) {
        return Standard_False;
    }
    Handle(TDataStd_Integer) anOwnedMarker;
    return theMaterialLabel.FindAttribute(
            OwnedPBRMaterialDefinitionAttributeID(), anOwnedMarker)
        && !anOwnedMarker.IsNull() && anOwnedMarker->Get() == 1;
}

Handle(XCAFDoc_VisMaterial) CreatePersistedPBRMaterial(
    const XCAFDoc_VisMaterialPBR& thePBR,
    const Handle(XCAFDoc_VisMaterial)& thePreviousMaterial)
{
    Graphic3d_AlphaMode anAlphaMode =
        thePBR.BaseColor.Alpha() < 0.999f
            ? Graphic3d_AlphaMode_Blend
            : Graphic3d_AlphaMode_Opaque;
    Standard_ShortReal anAlphaCutoff = 0.5f;
    Graphic3d_TypeOfBackfacingModel aFaceCulling =
        Graphic3d_TypeOfBackfacingModel_Auto;
    if (!thePreviousMaterial.IsNull()) {
        anAlphaMode = thePreviousMaterial->AlphaMode();
        anAlphaCutoff = thePreviousMaterial->AlphaCutOff();
        aFaceCulling = thePreviousMaterial->FaceCulling();
    }

    Handle(XCAFDoc_VisMaterial) aMaterial =
        new XCAFDoc_VisMaterial();
    aMaterial->SetPbrMaterial(thePBR);
    // The ES2/OpenGL Common fallback and authoritative PBR definition are
    // intentionally serialized as two slots. The safe binary reader charges
    // each occurrence, even when both reference identical bytes.
    XCAFDoc_VisMaterialCommon aCommon =
        aMaterial->ConvertToCommonMaterial();
    aCommon.DiffuseTexture = thePBR.BaseColorTexture;
    aMaterial->SetCommonMaterial(aCommon);
    aMaterial->SetAlphaMode(anAlphaMode, anAlphaCutoff);
    aMaterial->SetFaceCulling(aFaceCulling);
    return aMaterial;
}

bool AccumulateSerializedMaterialTextureOccurrences(
    const Handle(XCAFDoc_VisMaterial)& theMaterial,
    Core3DEmbeddedTextureBudgetState& theBudget,
    const Standard_Size theMaximumSerializedBytes,
    const Standard_Size theMaximumDecodedBytes)
{
    if (theMaterial.IsNull()) {
        return false;
    }
    if (theMaterial->HasPbrMaterial()) {
        const XCAFDoc_VisMaterialPBR& aPBR =
            theMaterial->PbrMaterial();
        if (!Core3DAccumulateEmbeddedTextureBudget(
                aPBR.BaseColorTexture,
                theBudget, theMaximumSerializedBytes,
                theMaximumDecodedBytes)
            || !Core3DAccumulateEmbeddedTextureBudget(
                aPBR.MetallicRoughnessTexture,
                theBudget, theMaximumSerializedBytes,
                theMaximumDecodedBytes)
            || !Core3DAccumulateEmbeddedTextureBudget(
                aPBR.EmissiveTexture,
                theBudget, theMaximumSerializedBytes,
                theMaximumDecodedBytes)
            || !Core3DAccumulateEmbeddedTextureBudget(
                aPBR.OcclusionTexture,
                theBudget, theMaximumSerializedBytes,
                theMaximumDecodedBytes)
            || !Core3DAccumulateEmbeddedTextureBudget(
                aPBR.NormalTexture,
                theBudget, theMaximumSerializedBytes,
                theMaximumDecodedBytes)) {
            return false;
        }
    }
    if (theMaterial->HasCommonMaterial()
        && !Core3DAccumulateEmbeddedTextureBudget(
            theMaterial->CommonMaterial().DiffuseTexture,
            theBudget, theMaximumSerializedBytes,
            theMaximumDecodedBytes)) {
        return false;
    }
    return true;
}

std::string ReadIdentifier(const TDF_Label& theLabel,
                           const Standard_GUID& theAttributeID)
{
    if (theLabel.IsNull()) {
        return {};
    }

    Handle(TDataStd_AsciiString) anIdentifier;
    if (!theLabel.FindAttribute(theAttributeID, anIdentifier)
        || anIdentifier.IsNull()) {
        return {};
    }

    const TCollection_AsciiString& aValue = anIdentifier->Get();
    if (aValue.IsEmpty() || !Standard_GUID::CheckGUIDFormat(aValue.ToCString())) {
        return {};
    }
    return aValue.ToCString();
}

std::string NewIdentifier()
{
    NSString* aValue = NSUUID.UUID.UUIDString;
    return aValue == nil ? std::string() : std::string(aValue.UTF8String);
}

Standard_Boolean AssignNewIdentifier(
    const TDF_Label& theLabel,
    const Standard_GUID& theAttributeID)
{
    if (theLabel.IsNull()) {
        return Standard_False;
    }

    const std::string anIdentifier = NewIdentifier();
    if (anIdentifier.empty()) {
        return Standard_False;
    }
    TDataStd_AsciiString::Set(
        theLabel,
        theAttributeID,
        TCollection_AsciiString(anIdentifier.c_str()));
    return Standard_True;
}

Standard_Boolean AssignIdentifierIfMissing(
    const TDF_Label& theLabel,
    const Standard_GUID& theAttributeID)
{
    return !ReadIdentifier(theLabel, theAttributeID).empty()
        || AssignNewIdentifier(theLabel, theAttributeID);
}

void AbortCommandNoThrow(const Handle(TDocStd_Document)& theDocument) noexcept
{
    if (theDocument.IsNull()) {
        return;
    }
    try {
        if (theDocument->HasOpenCommand()) {
            theDocument->AbortCommand();
        }
    } catch (...) {
    }
}

} // namespace

Standard_Boolean Core3DAccumulateEmbeddedTextureBudget(
    const Handle(Image_Texture)& texture,
    Core3DEmbeddedTextureBudgetState& state,
    const Standard_Size maximumSerializedOccurrenceBytes,
    const Standard_Size maximumDecodedResourceBytes)
{
    if (texture.IsNull()) {
        return Standard_True;
    }
    const std::string storedIdentifier(
        texture->TextureId().ToCString());
    std::string canonicalIdentifier = storedIdentifier;
    if (!texture->FilePath().IsEmpty()
        || storedIdentifier.empty()
        || storedIdentifier.size()
            > static_cast<std::size_t>(
                kMaximumPersistentTextureIdentifierBytes)
        || !StripBufferTexturePrefixes(canonicalIdentifier)) {
        return Standard_False;
    }
    const Handle(NCollection_Buffer)& buffer =
        texture->DataBuffer();
    if (buffer.IsNull() || buffer->Data() == nullptr
        || buffer->Size() == 0
        || buffer->Size() > kMaximumEmbeddedTextureBytes) {
        return Standard_False;
    }

    const Standard_Size serializedLimit = std::min(
        maximumSerializedOccurrenceBytes,
        kMaximumAggregateTextureBytes);
    if (state.serializedOccurrenceBytes > serializedLimit
        || buffer->Size()
            > serializedLimit - state.serializedOccurrenceBytes) {
        return Standard_False;
    }
    state.serializedOccurrenceBytes += buffer->Size();

    const auto existing =
        state.resourcesByIdentifier.find(canonicalIdentifier);
    if (existing != state.resourcesByIdentifier.end()) {
        const Handle(NCollection_Buffer)& existingBuffer =
            existing->second;
        return !existingBuffer.IsNull()
            && existingBuffer->Data() != nullptr
            && existingBuffer->Size() == buffer->Size()
            && std::memcmp(existingBuffer->Data(), buffer->Data(),
                           buffer->Size()) == 0;
    }

    Standard_Size decodedBytes = 0;
    const Standard_Size decodedLimit = std::min(
        maximumDecodedResourceBytes,
        kMaximumDecodedTextureBytes);
    if (!TryTextureDecodedBytes(buffer, decodedBytes)
        || state.decodedResourceBytes > decodedLimit
        || decodedBytes
            > decodedLimit - state.decodedResourceBytes) {
        return Standard_False;
    }
    state.decodedResourceBytes += decodedBytes;
    state.resourcesByIdentifier.emplace(
        canonicalIdentifier, buffer);
    return Standard_True;
}

Standard_Boolean Core3DCreateAuthoredTexture(
    const Standard_Byte* bytes,
    const Standard_Size size,
    const std::string& mediaType,
    Handle(Image_Texture)& texture)
{
    texture.Nullify();
    if (!ValidateAuthoredRasterBytes(bytes, size, &mediaType)) {
        return Standard_False;
    }
    const std::string identifier = AuthoredTextureIdentifier(bytes, size);
    if (identifier.size() != 15 + CC_SHA256_DIGEST_LENGTH * 2) {
        return Standard_False;
    }
    Handle(NCollection_Buffer) buffer = new NCollection_Buffer(
        NCollection_BaseAllocator::CommonBaseAllocator(), size);
    if (buffer.IsNull() || buffer->ChangeData() == nullptr
        || buffer->Size() != size) {
        return Standard_False;
    }
    std::memcpy(buffer->ChangeData(), bytes, size);
    texture = MakeRendererDecodableTexture(
        buffer, TCollection_AsciiString(identifier.c_str()));
    if (texture.IsNull()) {
        return Standard_False;
    }
#ifdef DEBUG
    // Keep the constructor's OCCT-added texturebuf:// spelling aligned with
    // the standalone validator used by persistence and scalar edits.
    if (!Core3DValidateAuthoredTexture(texture)) {
        texture.Nullify();
        return Standard_False;
    }
#endif
    return Standard_True;
}

Standard_Boolean Core3DValidateAuthoredTexture(
    const Handle(Image_Texture)& texture)
{
    if (texture.IsNull() || !texture->FilePath().IsEmpty()) {
        return Standard_False;
    }
    const Handle(NCollection_Buffer)& buffer = texture->DataBuffer();
    if (buffer.IsNull() || buffer->Data() == nullptr
        || !ValidateAuthoredRasterBytes(
            buffer->Data(), buffer->Size(), nullptr)) {
        return Standard_False;
    }
    const std::string identifier = AuthoredTextureIdentifier(
        buffer->Data(), buffer->Size());
    std::string storedIdentifier(texture->TextureId().ToCString());
    return !identifier.empty()
        && StripBufferTexturePrefixes(storedIdentifier)
        && storedIdentifier == identifier;
}

Standard_Boolean Core3DTexturesMatch(
    const Handle(Image_Texture)& first,
    const Handle(Image_Texture)& second)
{
    if (first.IsNull() || second.IsNull()
        || !first->FilePath().IsEmpty()
        || !second->FilePath().IsEmpty()
        || !first->TextureId().IsEqual(second->TextureId())) {
        return Standard_False;
    }
    const Handle(NCollection_Buffer)& firstBuffer = first->DataBuffer();
    const Handle(NCollection_Buffer)& secondBuffer = second->DataBuffer();
    return !firstBuffer.IsNull() && !secondBuffer.IsNull()
        && firstBuffer->Data() != nullptr && secondBuffer->Data() != nullptr
        && firstBuffer->Size() == secondBuffer->Size()
        && std::memcmp(firstBuffer->Data(), secondBuffer->Data(),
                       firstBuffer->Size()) == 0;
}

Standard_Boolean Core3DCreateAuthoredBaseColorTexture(
    const Standard_Byte* bytes,
    const Standard_Size size,
    const std::string& mediaType,
    Handle(Image_Texture)& texture)
{
    return Core3DCreateAuthoredTexture(
        bytes, size, mediaType, texture);
}

Standard_Boolean Core3DValidateAuthoredBaseColorTexture(
    const Handle(Image_Texture)& texture)
{
    return Core3DValidateAuthoredTexture(texture);
}

Standard_Boolean Core3DBaseColorTexturesMatch(
    const Handle(Image_Texture)& first,
    const Handle(Image_Texture)& second)
{
    return Core3DTexturesMatch(first, second);
}

void Core3DBeginSafeBinaryRead()
{
    gSafeBinaryReadRejected = false;
}

Standard_Boolean Core3DSafeBinaryReadWasRejected()
{
    return gSafeBinaryReadRejected ? Standard_True : Standard_False;
}

void Core3DDefineSafeBinXCAFFormat(
    const Handle(TDocStd_Application)& application)
{
    if (application.IsNull()) {
        return;
    }
    application->DefineFormat(
        TCollection_AsciiString("BinOcaf"),
        TCollection_AsciiString("Binary OCAF Document"),
        TCollection_AsciiString("cbf"),
        new Core3DBoundedBinXCAFRetrievalDriver(),
        new BinDrivers_DocumentStorageDriver());
    application->DefineFormat(
        TCollection_AsciiString("BinXCAF"),
        TCollection_AsciiString("Binary XCAF Document"),
        TCollection_AsciiString("xbf"),
        new Core3DBoundedBinXCAFRetrievalDriver(),
        new BinXCAFDrivers_DocumentStorageDriver());
}

// =======================================================================
// function : OcctViewer
// purpose  :
// =======================================================================
OcctDocument::OcctDocument()
: myMaximumSerializedTextureOccurrenceBytes(
      kMaximumAggregateTextureBytes),
  myMaximumDecodedTextureResourceBytes(
      kMaximumDecodedTextureBytes),
  myMaximumVisualMaterialDefinitions(
      kMaximumVisualMaterialDefinitions)
{
  try
  {
    OCC_CATCH_SIGNALS
    myApp = new TDocStd_Application();
  }
  catch (const Standard_Failure& theFailure)
  {
    Message::SendFail (TCollection_AsciiString("Error in creating application") + theFailure.GetMessageString());
  }
}

// =======================================================================
// function : ~OcctDocument
// purpose  :
// =======================================================================
OcctDocument::~OcctDocument()
{
    std::cout << "~OcctDocument" << std::endl;
}

// =======================================================================
// function : InitDoc
// purpose  :
// =======================================================================
void OcctDocument::InitDoc()
{
    
    std::cout << "InitDoc()" << std::endl;
  // close old document
  if (!myOcafDoc.IsNull())
  {
    if (myOcafDoc->HasOpenCommand())
    {
      myOcafDoc->AbortCommand();
    }

    myOcafDoc->Main().Root().ForgetAllAttributes(Standard_True);
    myApp->Close(myOcafDoc);
    myOcafDoc.Nullify();
  }


  // Register both readers before creating the document: old projects remain
  // readable while new saves use the XCAF driver required by visual materials.
  Core3DDefineSafeBinXCAFFormat(myApp);
  myApp->NewDocument(TCollection_ExtendedString("BinXCAF"), myOcafDoc);

  // Install document infrastructure and identity before enabling normal undo
  // history. These are schema attributes, not user-authored edits.
  if (!myOcafDoc.IsNull())
  {
	// Create the persistent XCAF tools before the first undoable command. If the
	// first shape command creates these infrastructure attributes, Undo removes
	// them and a viewport redraw recreates them outside history; Redo then fails
	// because the same labels already carry those attributes.
	(void)XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
	(void)XCAFDoc_DocumentTool::ColorTool(myOcafDoc->Main());
	(void)XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
	XCAFDoc_DocumentTool::SetLengthUnit(
	    myOcafDoc, kDefaultMetersPerUnit);
	if (!AssignIdentifierIfMissing(
	        myOcafDoc->Main(), DocumentIdentifierAttributeID())) {
	  Message::SendFail("Unable to assign Core3D document identifier");
	}
	myOcafDoc->ClearUndos();
	myOcafDoc->SetUndoLimit(40);
  }
}

std::string OcctDocument::DocumentIdentifier() const
{
    return myOcafDoc.IsNull()
        ? std::string()
        : ReadIdentifier(myOcafDoc->Main(), DocumentIdentifierAttributeID());
}

std::string OcctDocument::EntityIdentifierForLabel(const TDF_Label& label) const
{
    return ReadIdentifier(label, EntityIdentifierAttributeID());
}

std::string OcctDocument::DefinitionIdentifierForLabel(const TDF_Label& label) const
{
    return ReadIdentifier(label, DefinitionIdentifierAttributeID());
}

Standard_Boolean OcctDocument::MigrateLegacyIdentifiers()
{
    return MigrateLegacyIdentifiers(myOcafDoc);
}

Standard_Boolean OcctDocument::MigrateLegacyIdentifiers(
    const Handle(TDocStd_Document)& document)
{
  try {
    OCC_CATCH_SIGNALS
    if (document.IsNull() || document->HasOpenCommand()) {
        return Standard_False;
    }

    TDF_LabelMap entityLabels;
    TDF_LabelMap definitionLabels;
    try {
        OCC_CATCH_SIGNALS
        XCAFPrs_DocumentExplorer anExplorer(
            document,
            XCAFPrs_DocumentExplorerFlags_None);
        for (; anExplorer.More(); anExplorer.Next()) {
            const XCAFPrs_DocumentNode& aNode = anExplorer.Current();
            if (!aNode.Label.IsNull()) {
                entityLabels.Add(aNode.Label);
            }
            const TDF_Label& aDefinitionLabel = aNode.RefLabel.IsNull()
                ? aNode.Label
                : aNode.RefLabel;
            if (!aDefinitionLabel.IsNull()) {
                definitionLabels.Add(aDefinitionLabel);
            }
        }
    } catch (...) {
        return Standard_False;
    }

    const TCollection_ExtendedString aBinXCAFFormat("BinXCAF");
    const Standard_Boolean needsStorageFormat =
        !document->StorageFormat().IsEqual(aBinXCAFFormat);
    const Standard_Boolean needsDocumentIdentifier =
        ReadIdentifier(document->Main(), DocumentIdentifierAttributeID()).empty();
    const Standard_Boolean needsVisMaterialTool =
        !XCAFDoc_DocumentTool::CheckVisMaterialTool(document->Main());
    std::vector<TDF_Label> entityIdentifiersNeedingAssignment;
    std::vector<TDF_Label> definitionIdentifiersNeedingAssignment;
    std::set<std::string> entityIdentifiers;
    std::set<std::string> definitionIdentifiers;
    for (TDF_MapIteratorOfLabelMap anEntity(entityLabels);
         anEntity.More(); anEntity.Next()) {
        const std::string anIdentifier = ReadIdentifier(
            anEntity.Key(), EntityIdentifierAttributeID());
        if (anIdentifier.empty()
            || !entityIdentifiers.insert(anIdentifier).second) {
            entityIdentifiersNeedingAssignment.push_back(anEntity.Key());
        }
    }
    for (TDF_MapIteratorOfLabelMap aDefinition(definitionLabels);
         aDefinition.More(); aDefinition.Next()) {
        const std::string anIdentifier = ReadIdentifier(
            aDefinition.Key(), DefinitionIdentifierAttributeID());
        if (anIdentifier.empty()
            || !definitionIdentifiers.insert(anIdentifier).second) {
            definitionIdentifiersNeedingAssignment.push_back(aDefinition.Key());
        }
    }

    const Standard_Boolean needsSchemaMigration =
        needsDocumentIdentifier
        || needsVisMaterialTool
        || !entityIdentifiersNeedingAssignment.empty()
        || !definitionIdentifiersNeedingAssignment.empty();
    if (!needsStorageFormat
        && !needsSchemaMigration) {
        return Standard_True;
    }

    // Identity migration is a schema operation and must never erase an active
    // user's history. Callers run it immediately after import/open, before the
    // document is published for editing.
    if (document->GetAvailableUndos() != 0
        || document->GetAvailableRedos() != 0) {
        return Standard_False;
    }

    const Standard_Integer aPreviousUndoLimit = document->GetUndoLimit();
    const TCollection_ExtendedString aPreviousStorageFormat =
        document->StorageFormat();
    try {
        OCC_CATCH_SIGNALS

        if (needsSchemaMigration) {
            document->SetUndoLimit(1);
            document->NewCommand();
            if (!document->HasOpenCommand()) {
                throw Standard_Failure("Unable to start document migration");
            }

            if (needsDocumentIdentifier
                && !AssignNewIdentifier(
                    document->Main(), DocumentIdentifierAttributeID())) {
                throw Standard_Failure("Unable to migrate document identifier");
            }
            if (needsVisMaterialTool
                && XCAFDoc_DocumentTool::VisMaterialTool(
                       document->Main()).IsNull()) {
                throw Standard_Failure("Unable to migrate XCAF material tool");
            }
            for (const TDF_Label& aLabel : entityIdentifiersNeedingAssignment) {
                if (!AssignNewIdentifier(
                        aLabel, EntityIdentifierAttributeID())) {
                    throw Standard_Failure("Unable to migrate entity identifier");
                }
            }
            for (const TDF_Label& aLabel : definitionIdentifiersNeedingAssignment) {
                if (!AssignNewIdentifier(
                        aLabel, DefinitionIdentifierAttributeID())) {
                    throw Standard_Failure("Unable to migrate definition identifier");
                }
            }

            if (!document->CommitCommand()) {
                throw Standard_Failure("Unable to commit document migration");
            }
            document->ClearUndos();
        }

        // Storage format is document metadata, not an OCAF attribute. Promote
        // it only after schema changes are safely committed, and never wrap a
        // format-only upgrade in an empty undo command.
        if (needsStorageFormat) {
            document->ChangeStorageFormat(aBinXCAFFormat);
        }
        document->SetUndoLimit(aPreviousUndoLimit);
        return Standard_True;
    } catch (...) {
        AbortCommandNoThrow(document);
        try {
            document->ClearUndos();
        } catch (...) {
        }
        try {
            document->SetUndoLimit(aPreviousUndoLimit);
        } catch (...) {
        }
        if (needsStorageFormat
            && !document->StorageFormat().IsEqual(
                aPreviousStorageFormat)) {
            try {
                document->ChangeStorageFormat(aPreviousStorageFormat);
            } catch (...) {
                // The candidate document will be rejected by the caller. Do
                // not let a secondary format-restore failure escape C++.
            }
        }
        return Standard_False;
    }
  } catch (...) {
    return Standard_False;
  }
}

void OcctDocument::RemoveShape(Handle(AIS_InteractiveObject) object) {
    RemoveShape(ShapeLabel(object));
}

void OcctDocument::RemoveShape(Handle(AIS_Shape) aisShape) {
    RemoveShape(ShapeLabel(aisShape));
}

void OcctDocument::RemoveShape(TopoDS_Shape object) {
    if (myOcafDoc.IsNull() || object.IsNull()) {
        return;
    }
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_Label label;
    if(shapeTool->FindShape(object, label)
       || shapeTool->FindShape(object, label, Standard_True)) {
        RemoveShape(label);
    }
}

TDF_Label OcctDocument::ShapeLabel(Handle(AIS_InteractiveObject) object) const {
    TDF_Label label;
    if (myOcafDoc.IsNull() || object.IsNull()) {
        return label;
    }

    const Handle(CafShapePrs) aCafPresentation =
        Handle(CafShapePrs)::DownCast(object);
    if (!aCafPresentation.IsNull()) {
        if (!aCafPresentation->IsEditablePresentation()
            || aCafPresentation->GetLabel().IsNull()
            || aCafPresentation->GetLabel().Data()
                != myOcafDoc->GetData()) {
            return label;
        }
        return aCafPresentation->GetLabel();
    }

    Handle(AIS_Shape) aisShape = Handle(AIS_Shape)::DownCast(object);
    if (aisShape.IsNull() || aisShape->Shape().IsNull()) {
        return label;
    }

    Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
    if (shapeTool.IsNull()) {
        return label;
    }
    shapeTool->FindShape(aisShape->Shape(), label)
        || shapeTool->FindShape(aisShape->Shape(), label, Standard_True);
    return label;
}

Standard_Boolean OcctDocument::IsPresentationEditable(
    Handle(AIS_InteractiveObject) object) const {
    if (object.IsNull()) {
        return Standard_False;
    }
    const Handle(CafShapePrs) aCafPresentation =
        Handle(CafShapePrs)::DownCast(object);
    return aCafPresentation.IsNull()
        || aCafPresentation->IsEditablePresentation();
}

Standard_Boolean OcctDocument::IsEditableFreeSimpleDefinitionLabel(
    const TDF_Label& label) const {
    if (myOcafDoc.IsNull() || label.IsNull()
        || label.Data() != myOcafDoc->GetData()) {
        return Standard_False;
    }
    const Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
    return !shapeTool.IsNull() && shapeTool->IsShape(label)
        && XCAFDoc_ShapeTool::IsFree(label)
        && XCAFDoc_ShapeTool::IsSimpleShape(label)
        && !XCAFDoc_ShapeTool::IsReference(label)
        && !XCAFDoc_ShapeTool::IsComponent(label)
        && !XCAFDoc_ShapeTool::IsAssembly(label)
        && !XCAFDoc_ShapeTool::IsSubShape(label);
}

Standard_Boolean OcctDocument::RemoveShape(const TDF_Label& label) {
    if (myOcafDoc.IsNull() || label.IsNull()) {
        return Standard_False;
    }
    Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
    TDF_Label aPreviousMaterialLabel;
    XCAFDoc_VisMaterialTool::GetShapeMaterial(
        label, aPreviousMaterialLabel);
    const Standard_Boolean didRemove =
        !shapeTool.IsNull()
        && shapeTool->RemoveShape(label, Standard_True);
    if (didRemove && myOcafDoc->HasOpenCommand()
        && !aPreviousMaterialLabel.IsNull()
        && XCAFDoc_DocumentTool::CheckVisMaterialTool(
            myOcafDoc->Main())) {
        RemoveUnreferencedOwnedMaterial(
            XCAFDoc_DocumentTool::VisMaterialTool(
                myOcafDoc->Main()),
            aPreviousMaterialLabel);
    }
    return didRemove;
}

TDF_Label OcctDocument::AddShape(Handle(AIS_InteractiveObject) object) {
    Handle(AIS_Shape) aisShape = Handle(AIS_Shape)::DownCast(object);
    return AddShape(aisShape);
}

TDF_Label OcctDocument::AddShape(Handle(AIS_Shape) aisShape) {
	if (myOcafDoc.IsNull()
	    || !myOcafDoc->HasOpenCommand()
	    || aisShape.IsNull()
	    || aisShape->Shape().IsNull()) {
	    return TDF_Label();
	}
	Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
	if (shapeTool.IsNull()) {
	    return TDF_Label();
	}
	TDF_Label label = shapeTool->NewShape();
	shapeTool->SetShape(label, aisShape->Shape());
	if (!AssignNewIdentifier(label, EntityIdentifierAttributeID())
	    || !AssignNewIdentifier(label, DefinitionIdentifierAttributeID())) {
	    return TDF_Label();
	}
	SaveObjectTransform(label, aisShape);
	return label;
}

Standard_Boolean OcctDocument::ReplaceShape(
    const TDF_Label& label,
    Handle(AIS_Shape) aisShape) {
	// Stable entity and definition identifiers belong to the label, so replacing
	// its geometry deliberately leaves both identity attributes untouched.
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || label.IsNull() || label.Data() != myOcafDoc->GetData()
        || aisShape.IsNull() || aisShape->Shape().IsNull()) {
        return Standard_False;
    }
	Handle(XCAFDoc_ShapeTool) shapeTool =
        XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
    if (!IsEditableFreeSimpleDefinitionLabel(label)) {
        return Standard_False;
    }
    const TopoDS_Shape previousShape = XCAFDoc_ShapeTool::GetShape(label);
    if (previousShape.IsNull()) {
        return Standard_False;
    }
    struct SavedRealAttribute {
        Standard_Boolean wasPresent = Standard_False;
        Standard_Real value = 0.0;
    };
    SavedRealAttribute previousTransform[8];
    for (Standard_Integer tag = 1; tag <= 8; ++tag) {
        const TDF_Label child = label.FindChild(tag, Standard_False);
        Handle(TDataStd_Real) attribute;
        if (!child.IsNull()
            && child.FindAttribute(TDataStd_Real::GetID(), attribute)
            && !attribute.IsNull()) {
            previousTransform[tag - 1].wasPresent = Standard_True;
            previousTransform[tag - 1].value = attribute->Get();
        }
    }
    const auto restorePrevious = [&]() noexcept {
        try {
            shapeTool->SetShape(label, previousShape);
            for (Standard_Integer tag = 1; tag <= 8; ++tag) {
                const SavedRealAttribute& saved =
                    previousTransform[tag - 1];
                if (saved.wasPresent) {
                    TDataStd_Real::Set(label.FindChild(tag), saved.value);
                } else {
                    const TDF_Label child =
                        label.FindChild(tag, Standard_False);
                    if (!child.IsNull()) {
                        child.ForgetAttribute(TDataStd_Real::GetID());
                    }
                }
            }
        } catch (...) {
        }
    };
    try {
        shapeTool->SetShape(label, aisShape->Shape());
        const TopoDS_Shape stored = XCAFDoc_ShapeTool::GetShape(label);
        if (stored.IsNull() || !stored.IsEqual(aisShape->Shape())) {
            restorePrevious();
            return Standard_False;
        }
        SaveObjectTransform(label, aisShape);
        return Standard_True;
    } catch (...) {
        restorePrevious();
        return Standard_False;
    }
}

void OcctDocument::SaveObjectTransform(const TDF_Label& label, const Handle(AIS_Shape) anAis) {
    auto t = anAis->LocalTransformation();
    TDataStd_Real::Set(label.FindChild(1), t.TranslationPart().X());
    TDataStd_Real::Set(label.FindChild(2), t.TranslationPart().Y());
    TDataStd_Real::Set(label.FindChild(3), t.TranslationPart().Z());
    TDataStd_Real::Set(label.FindChild(4), t.GetRotation().X());
    TDataStd_Real::Set(label.FindChild(5), t.GetRotation().Y());
    TDataStd_Real::Set(label.FindChild(6), t.GetRotation().Z());
    TDataStd_Real::Set(label.FindChild(7), t.GetRotation().W());
    TDataStd_Real::Set(label.FindChild(8), t.ScaleFactor());
 
}

void OcctDocument::SaveObjectMaterial(Handle(AIS_Shape) object
                                      , const Graphic3d_NameOfMaterial name_of_material) {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_Label label;
    Handle(TDataStd_Integer) aCurrentint;
    if(shapeTool->FindShape(object->Shape(), label)) {
        SaveObjectMaterial(label, name_of_material);
    }

}

void OcctDocument::SaveObjectColor(Handle(AIS_Shape) object
                                   , const Quantity_NameOfColor name_of_color) {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_Label label;
    Handle(TDataStd_Integer) aCurrentint;
    if(shapeTool->FindShape(object->Shape(), label)) {
        SaveObjectColor(label, name_of_color);
    }
}

void OcctDocument::SaveObjectMaterial(const TDF_Label& label, const Graphic3d_NameOfMaterial name_of_material) {
    TDataStd_Integer::Set(label.FindChild(11), name_of_material);
}

void OcctDocument::SaveObjectColor(const TDF_Label& label, const Quantity_NameOfColor name_of_color) {
    TDataStd_Integer::Set(label.FindChild(12), name_of_color);
}

Standard_Boolean OcctDocument::SaveObjectPBRMaterial(
    const TDF_Label& label,
    const XCAFDoc_VisMaterialPBR& material) {
    return SaveObjectPBRMaterials({{
        label, material, Handle(Image_Texture)(), Handle(Image_Texture)()}});
}

Standard_Boolean OcctDocument::SaveObjectPBRMaterial(
    const TDF_Label& label,
    const XCAFDoc_VisMaterialPBR& material,
    const Handle(Image_Texture)& prevalidatedBaseColorTexture) {
    return SaveObjectPBRMaterials({{
        label, material, prevalidatedBaseColorTexture,
        Handle(Image_Texture)()}});
}

Standard_Boolean OcctDocument::SaveObjectPBRMaterial(
    const TDF_Label& label,
    const XCAFDoc_VisMaterialPBR& material,
    const Handle(Image_Texture)& prevalidatedBaseColorTexture,
    const Handle(Image_Texture)& prevalidatedEmissiveTexture) {
    return SaveObjectPBRMaterials({{
        label, material, prevalidatedBaseColorTexture,
        prevalidatedEmissiveTexture}});
}

void OcctDocument::SetMaximumSerializedTextureOccurrenceBytesForTesting(
    const Standard_Size maximumBytes)
{
    myMaximumSerializedTextureOccurrenceBytes = std::min(
        maximumBytes, kMaximumAggregateTextureBytes);
}

void OcctDocument::SetMaximumDecodedTextureResourceBytesForTesting(
    const Standard_Size maximumBytes)
{
    myMaximumDecodedTextureResourceBytes = std::min(
        maximumBytes, kMaximumDecodedTextureBytes);
}

void OcctDocument::SetMaximumVisualMaterialDefinitionsForTesting(
    const Standard_Size maximumDefinitions)
{
    myMaximumVisualMaterialDefinitions = std::min(
        maximumDefinitions,
        static_cast<Standard_Size>(
            kMaximumVisualMaterialDefinitions));
}

Standard_Boolean OcctDocument::CanSaveObjectPBRMaterials(
    const std::vector<OcctPBRMaterialUpdate>& updates,
    std::vector<TDF_Label>* reclaimMaterialLabels) const
{
    if (reclaimMaterialLabels != nullptr) {
        reclaimMaterialLabels->clear();
    }
    if (myOcafDoc.IsNull() || updates.empty()
        || updates.size()
            > static_cast<std::size_t>(
                kMaximumVisualMaterialDefinitions)
        || !XCAFDoc_DocumentTool::CheckVisMaterialTool(
            myOcafDoc->Main())) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterialTool) aTool =
        XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
    if (aTool.IsNull()) {
        return Standard_False;
    }
    const Standard_Size aMaximumDefinitions = std::min(
        myMaximumVisualMaterialDefinitions,
        static_cast<Standard_Size>(
            kMaximumVisualMaterialDefinitions));

    struct ExistingDefinition {
        TDF_Label label;
        Handle(XCAFDoc_VisMaterial) material;
        bool reclaim = false;
    };
    TDF_LabelSequence existingLabels;
    aTool->GetMaterials(existingLabels);
    if (existingLabels.Length() < 0
        || existingLabels.Length()
            > kMaximumVisualMaterialDefinitions) {
        return Standard_False;
    }
    std::vector<ExistingDefinition> existingDefinitions;
    TDF_LabelMap tableMaterialLabels;
    existingDefinitions.reserve(
        static_cast<std::size_t>(existingLabels.Length()));
    for (Standard_Integer index = 1;
         index <= existingLabels.Length(); ++index) {
        const TDF_Label& aLabel = existingLabels.Value(index);
        const Handle(XCAFDoc_VisMaterial) aMaterial =
            XCAFDoc_VisMaterialTool::GetMaterial(aLabel);
        if (aLabel.IsNull() || aMaterial.IsNull()) {
            return Standard_False;
        }
        tableMaterialLabels.Add(aLabel);
        existingDefinitions.push_back({aLabel, aMaterial, false});
    }
    const Handle(TDF_Data)& documentData = myOcafDoc->GetData();
    if (documentData.IsNull()) {
        return Standard_False;
    }
    const auto hasUnregisteredDirectMaterial =
        [&](const TDF_Label& label) {
            Handle(XCAFDoc_VisMaterial) directMaterial;
            return !label.IsNull()
                && label.FindAttribute(
                    XCAFDoc_VisMaterial::GetID(), directMaterial)
                && (directMaterial.IsNull()
                    || !tableMaterialLabels.Contains(label));
        };
    if (hasUnregisteredDirectMaterial(documentData->Root())) {
        return Standard_False;
    }
    Standard_Size labelCount = 0;
    for (TDF_ChildIterator iterator(
             documentData->Root(), Standard_True);
         iterator.More(); iterator.Next()) {
        if (++labelCount > kMaximumDocumentLabels
            || hasUnregisteredDirectMaterial(iterator.Value())) {
            return Standard_False;
        }
    }

    struct ProjectedUpdate {
        TDF_Label label;
        Handle(XCAFDoc_VisMaterial) material;
        Standard_Integer previousExisting = -1;
        Standard_Integer targetExisting = -1;
    };
    std::vector<ProjectedUpdate> projectedUpdates;
    projectedUpdates.reserve(updates.size());
    std::vector<Handle(XCAFDoc_VisMaterial)> newDefinitions;

    const auto existingIndexForLabel = [&](const TDF_Label& label) {
        for (std::size_t index = 0;
             index < existingDefinitions.size(); ++index) {
            if (existingDefinitions[index].label.IsEqual(label)) {
                return static_cast<Standard_Integer>(index);
            }
        }
        return Standard_Integer(-1);
    };
    for (const OcctPBRMaterialUpdate& update : updates) {
        if (update.label.IsNull()) {
            return Standard_False;
        }
        for (const ProjectedUpdate& projected : projectedUpdates) {
            if (projected.label.IsEqual(update.label)) {
                return Standard_False;
            }
        }

        TDF_Label aPreviousLabel;
        const Handle(XCAFDoc_VisMaterial) aPreviousMaterial =
            XCAFDoc_VisMaterialTool::GetShapeMaterial(update.label);
        XCAFDoc_VisMaterialTool::GetShapeMaterial(
            update.label, aPreviousLabel);
        const Handle(XCAFDoc_VisMaterial) aCandidate =
            CreatePersistedPBRMaterial(
                update.material, aPreviousMaterial);
        if (aCandidate.IsNull()) {
            return Standard_False;
        }

        Standard_Integer aTargetExisting = -1;
        for (std::size_t index = 0;
             index < existingDefinitions.size(); ++index) {
            if (existingDefinitions[index].material->IsEqual(
                    aCandidate)) {
                aTargetExisting =
                    static_cast<Standard_Integer>(index);
                break;
            }
        }
        if (aTargetExisting < 0) {
            bool isAlreadyProjected = false;
            for (const Handle(XCAFDoc_VisMaterial)& projected :
                 newDefinitions) {
                if (!projected.IsNull()
                    && projected->IsEqual(aCandidate)) {
                    isAlreadyProjected = true;
                    break;
                }
            }
            if (!isAlreadyProjected) {
                newDefinitions.push_back(aCandidate);
            }
        }
        projectedUpdates.push_back({
            update.label,
            aCandidate,
            existingIndexForLabel(aPreviousLabel),
            aTargetExisting});
    }

    const auto updateForLabel = [&](const TDF_Label& label)
        -> const ProjectedUpdate* {
        for (const ProjectedUpdate& update : projectedUpdates) {
            if (update.label.IsEqual(label)) {
                return &update;
            }
        }
        return nullptr;
    };
    for (std::size_t index = 0;
         index < existingDefinitions.size(); ++index) {
        ExistingDefinition& existing = existingDefinitions[index];
        bool wasPreviousDefinition = false;
        bool hasFinalReference = false;
        for (const ProjectedUpdate& update : projectedUpdates) {
            wasPreviousDefinition = wasPreviousDefinition
                || update.previousExisting
                    == static_cast<Standard_Integer>(index);
            hasFinalReference = hasFinalReference
                || update.targetExisting
                    == static_cast<Standard_Integer>(index);
        }
        if (!wasPreviousDefinition
            || !IsOwnedMaterialDefinition(existing.label)) {
            continue;
        }

        Handle(TDataStd_TreeNode) aReferenceRoot;
        if (existing.label.FindAttribute(
                XCAFDoc::VisMaterialRefGUID(), aReferenceRoot)
            && !aReferenceRoot.IsNull()) {
            for (Handle(TDataStd_TreeNode) aReference =
                     aReferenceRoot->First();
                 !aReference.IsNull();
                 aReference = aReference->Next()) {
                const ProjectedUpdate* update =
                    updateForLabel(aReference->Label());
                if (update == nullptr
                    || update->previousExisting
                        != static_cast<Standard_Integer>(index)
                    || update->targetExisting
                        == static_cast<Standard_Integer>(index)) {
                    hasFinalReference = true;
                    break;
                }
            }
        }
        // Remove from the projected definition set only when every extant
        // reference is part of this batch and moves elsewhere. Imported or
        // orphaned/unrelated definitions remain charged exactly as saved.
        existing.reclaim = !hasFinalReference;
    }

    Standard_Size finalDefinitionCount =
        static_cast<Standard_Size>(newDefinitions.size());
    if (finalDefinitionCount > aMaximumDefinitions) {
        return Standard_False;
    }
    for (const ExistingDefinition& existing : existingDefinitions) {
        if (!existing.reclaim) {
            if (finalDefinitionCount
                >= aMaximumDefinitions) {
                return Standard_False;
            }
            ++finalDefinitionCount;
        }
    }

    const Standard_Size aMaximumSerializedBytes = std::min(
        myMaximumSerializedTextureOccurrenceBytes,
        kMaximumAggregateTextureBytes);
    const Standard_Size aMaximumDecodedBytes = std::min(
        myMaximumDecodedTextureResourceBytes,
        kMaximumDecodedTextureBytes);
    Core3DEmbeddedTextureBudgetState aTextureBudget;
    for (const ExistingDefinition& existing : existingDefinitions) {
        if (!existing.reclaim
            && !AccumulateSerializedMaterialTextureOccurrences(
                existing.material,
                aTextureBudget,
                aMaximumSerializedBytes,
                aMaximumDecodedBytes)) {
            return Standard_False;
        }
    }
    for (const Handle(XCAFDoc_VisMaterial)& material :
         newDefinitions) {
        if (!AccumulateSerializedMaterialTextureOccurrences(
                material,
                aTextureBudget,
                aMaximumSerializedBytes,
                aMaximumDecodedBytes)) {
            return Standard_False;
        }
    }
    if (reclaimMaterialLabels != nullptr) {
        reclaimMaterialLabels->reserve(existingDefinitions.size());
        for (const ExistingDefinition& existing : existingDefinitions) {
            if (existing.reclaim) {
                reclaimMaterialLabels->push_back(existing.label);
            }
        }
    }
    return Standard_True;
}

Standard_Boolean OcctDocument::SaveObjectPBRMaterials(
    const std::vector<OcctPBRMaterialUpdate>& updates)
{
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || updates.empty()) {
        return Standard_False;
    }
    const auto isFiniteUnit = [](const Standard_Real theValue) {
        return std::isfinite(theValue)
            && theValue >= 0.0 && theValue <= 1.0;
    };
    const auto isFiniteNonNegative = [](const Standard_Real theValue) {
        return std::isfinite(theValue) && theValue >= 0.0;
    };
    std::unordered_set<const Image_Texture*> validatedAuthoredTextures;
    const auto validateAuthoredTextureBinding =
        [&validatedAuthoredTextures](
            const Handle(Image_Texture)& texture,
            const Handle(Image_Texture)& prevalidatedTexture) {
            if (texture.IsNull()) {
                return prevalidatedTexture.IsNull();
            }
            if (!prevalidatedTexture.IsNull()
                && texture.get() != prevalidatedTexture.get()) {
                return false;
            }
            return !validatedAuthoredTextures.insert(texture.get()).second
                || Core3DValidateAuthoredTexture(texture);
        };
    for (const OcctPBRMaterialUpdate& update : updates) {
        const XCAFDoc_VisMaterialPBR& material = update.material;
        const Quantity_Color& aBaseColor = material.BaseColor.GetRGB();
        if (update.label.IsNull() || !material.IsDefined
            || !isFiniteUnit(aBaseColor.Red())
            || !isFiniteUnit(aBaseColor.Green())
            || !isFiniteUnit(aBaseColor.Blue())
            || !isFiniteUnit(material.BaseColor.Alpha())
            || !isFiniteUnit(material.Metallic)
            || !isFiniteUnit(material.Roughness)
            || !isFiniteNonNegative(material.EmissiveFactor.x())
            || !isFiniteNonNegative(material.EmissiveFactor.y())
            || !isFiniteNonNegative(material.EmissiveFactor.z())
            || material.EmissiveFactor.x() > kMaximumEmissionFactor
            || material.EmissiveFactor.y() > kMaximumEmissionFactor
            || material.EmissiveFactor.z() > kMaximumEmissionFactor
            || !material.MetallicRoughnessTexture.IsNull()
            || !material.OcclusionTexture.IsNull()
            || !material.NormalTexture.IsNull()
            || !validateAuthoredTextureBinding(
                material.BaseColorTexture,
                update.prevalidatedBaseColorTexture)
            || !validateAuthoredTextureBinding(
                material.EmissiveTexture,
                update.prevalidatedEmissiveTexture)
            || !std::isfinite(material.RefractionIndex)
            || material.RefractionIndex < 1.0f
            || material.RefractionIndex > 3.0f) {
            return Standard_False;
        }
    }
    std::vector<TDF_Label> reclaimMaterialLabels;
    if (!CanSaveObjectPBRMaterials(
            updates, &reclaimMaterialLabels)) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterialTool) aTool =
        XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
    if (aTool.IsNull()) {
        return Standard_False;
    }
    const Standard_Size aMaximumDefinitions = std::min(
        myMaximumVisualMaterialDefinitions,
        static_cast<Standard_Size>(
            kMaximumVisualMaterialDefinitions));

    struct PreparedUpdate {
        TDF_Label label;
        TDF_Label previousMaterialLabel;
        Handle(XCAFDoc_VisMaterial) material;
    };
    std::vector<PreparedUpdate> preparedUpdates;
    preparedUpdates.reserve(updates.size());
    for (const OcctPBRMaterialUpdate& update : updates) {
        TDF_Label aPreviousMaterialLabel;
        const Handle(XCAFDoc_VisMaterial) aPreviousMaterial =
            XCAFDoc_VisMaterialTool::GetShapeMaterial(update.label);
        XCAFDoc_VisMaterialTool::GetShapeMaterial(
            update.label, aPreviousMaterialLabel);
        const Handle(XCAFDoc_VisMaterial) aMaterial =
            CreatePersistedPBRMaterial(
                update.material, aPreviousMaterial);
        if (aMaterial.IsNull()) {
            return Standard_False;
        }
        preparedUpdates.push_back({
            update.label, aPreviousMaterialLabel, aMaterial});
    }

    // Detach the whole batch first. This makes final-state reclaim realizable
    // even when several selected labels share one old definition and the table
    // is already at its cap. Candidate handles above preserve each label's
    // alpha/cutoff/culling before those links are removed.
    for (const PreparedUpdate& update : preparedUpdates) {
        aTool->UnSetShapeMaterial(update.label);
    }
    for (const TDF_Label& reclaimLabel : reclaimMaterialLabels) {
        RemoveUnreferencedOwnedMaterial(aTool, reclaimLabel);
    }

    for (const PreparedUpdate& update : preparedUpdates) {
        TDF_Label aMaterialLabel;
        bool didAddMaterial = false;
        TDF_LabelSequence existingLabels;
        aTool->GetMaterials(existingLabels);
        for (Standard_Integer index = 1;
             index <= existingLabels.Length(); ++index) {
            const Handle(XCAFDoc_VisMaterial) existing =
                XCAFDoc_VisMaterialTool::GetMaterial(
                    existingLabels.Value(index));
            if (!existing.IsNull()
                && existing->IsEqual(update.material)) {
                aMaterialLabel = existingLabels.Value(index);
                break;
            }
        }
        if (aMaterialLabel.IsNull()) {
            if (existingLabels.Length() < 0
                || static_cast<Standard_Size>(
                    existingLabels.Length())
                    >= aMaximumDefinitions) {
                return Standard_False;
            }
            aMaterialLabel = aTool->AddMaterial(
                update.material,
                TCollection_AsciiString("Shapeyard PBR"));
            didAddMaterial = !aMaterialLabel.IsNull();
        }
        if (didAddMaterial) {
            TDataStd_Integer::Set(
                aMaterialLabel,
                OwnedPBRMaterialDefinitionAttributeID(),
                1);
        }
        if (aMaterialLabel.IsNull()) {
            return Standard_False;
        }
        aTool->SetShapeMaterial(update.label, aMaterialLabel);
        TDataStd_Integer::Set(
            update.label, LocalPBRMaterialAttributeID(), 1);

        // Canonical PBR and legacy preset tags must never compete for
        // precedence.
        for (const Standard_Integer aTag : {11, 12}) {
            const TDF_Label aLegacyLabel =
                update.label.FindChild(aTag, Standard_False);
            if (!aLegacyLabel.IsNull()) {
                aLegacyLabel.ForgetAttribute(
                    TDataStd_Integer::GetID());
            }
        }
    }
    for (const PreparedUpdate& update : preparedUpdates) {
        RemoveUnreferencedOwnedMaterial(
            aTool, update.previousMaterialLabel);
    }
    return Standard_True;
}

Standard_Boolean OcctDocument::ClearObjectVisualMaterial(
    const TDF_Label& label) {
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || label.IsNull()) {
        return Standard_False;
    }
    if (XCAFDoc_DocumentTool::CheckVisMaterialTool(myOcafDoc->Main())) {
        Handle(XCAFDoc_VisMaterialTool) aTool =
            XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
        if (!aTool.IsNull()) {
            TDF_Label aPreviousMaterialLabel;
            XCAFDoc_VisMaterialTool::GetShapeMaterial(
                label, aPreviousMaterialLabel);
            aTool->UnSetShapeMaterial(label);
            RemoveUnreferencedOwnedMaterial(
                aTool, aPreviousMaterialLabel);
        }
    }
    label.ForgetAttribute(LocalPBRMaterialAttributeID());
    label.ForgetAttribute(AutoPromotedEmissiveFactorAttributeID());
    return Standard_True;
}

Standard_Boolean OcctDocument::CopyObjectAppearance(
    const TDF_Label& source,
    const TDF_Label& destination) {
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || source.IsNull() || destination.IsNull()) {
        return Standard_False;
    }

    Handle(TDataStd_Integer) aLocalPBRMarker;
    const Standard_Boolean hasLocalPBR =
        source.FindAttribute(
            LocalPBRMaterialAttributeID(), aLocalPBRMarker)
        && !aLocalPBRMarker.IsNull()
        && aLocalPBRMarker->Get() == 1;
    const Standard_Boolean hasAutoPromotedEmissiveFactor =
        IsEmissiveTextureFactorAutoPromotedForLabel(source);
    Graphic3d_NameOfMaterial aLegacyMaterial;
    Quantity_NameOfColor aLegacyColor;
    const Standard_Boolean hasLegacyMaterial =
        TryMaterialNameForLabel(source, aLegacyMaterial);
    const Standard_Boolean hasLegacyColor =
        TryColorNameForLabel(source, aLegacyColor);
    const auto clearDestinationLegacyAppearance = [&destination]() {
        for (const Standard_Integer aTag : {11, 12}) {
            const TDF_Label aLegacyLabel =
                destination.FindChild(aTag, Standard_False);
            if (!aLegacyLabel.IsNull()) {
                aLegacyLabel.ForgetAttribute(TDataStd_Integer::GetID());
            }
        }
    };
    clearDestinationLegacyAppearance();

    TDF_Label aVisualMaterialLabel;
    const Standard_Boolean hasVisualMaterial =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(
            source, aVisualMaterialLabel)
        && !aVisualMaterialLabel.IsNull();
    if (hasVisualMaterial) {
        Handle(XCAFDoc_VisMaterialTool) aTool =
            XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
        if (aTool.IsNull()) {
            return Standard_False;
        }
        TDF_Label aPreviousDestinationMaterialLabel;
        XCAFDoc_VisMaterialTool::GetShapeMaterial(
            destination, aPreviousDestinationMaterialLabel);
        aTool->SetShapeMaterial(destination, aVisualMaterialLabel);
        if (!aPreviousDestinationMaterialLabel.IsNull()
            && !aPreviousDestinationMaterialLabel.IsEqual(
                aVisualMaterialLabel)) {
            RemoveUnreferencedOwnedMaterial(
                aTool, aPreviousDestinationMaterialLabel);
        }
        if (hasLocalPBR) {
            TDataStd_Integer::Set(
                destination, LocalPBRMaterialAttributeID(), 1);
        } else {
            destination.ForgetAttribute(LocalPBRMaterialAttributeID());
        }
        if (hasLocalPBR && hasAutoPromotedEmissiveFactor) {
            TDataStd_Integer::Set(
                destination,
                AutoPromotedEmissiveFactorAttributeID(),
                1);
        } else {
            destination.ForgetAttribute(
                AutoPromotedEmissiveFactorAttributeID());
        }
        if (hasLocalPBR) {
            return Standard_True;
        }
        if (hasLegacyMaterial) {
            SaveObjectMaterial(destination, aLegacyMaterial);
        }
        if (hasLegacyColor) {
            SaveObjectColor(destination, aLegacyColor);
        }
        return Standard_True;
    }

    if (!ClearObjectVisualMaterial(destination)) {
        return Standard_False;
    }
    // Preserve attribute absence as well as attribute values. Filling missing
    // children with defaults can change a preset-only source's effective base
    // color or make future precedence checks treat it as explicitly styled.
    if (hasLegacyMaterial) {
        SaveObjectMaterial(destination, aLegacyMaterial);
    }
    if (hasLegacyColor) {
        SaveObjectColor(destination, aLegacyColor);
    }
    return Standard_True;
}

Graphic3d_NameOfMaterial OcctDocument::MaterialNameForShape(Handle(AIS_Shape) object) {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_Label label;
    if(shapeTool->FindShape(object->Shape(), label)) {
        return MaterialNameForLabel(label);
    }
    return Graphic3d_NameOfMaterial_UserDefined;
}

Graphic3d_NameOfMaterial OcctDocument::MaterialNameForLabel(const TDF_Label& label) const {
    Graphic3d_NameOfMaterial material;
    return TryMaterialNameForLabel(label, material)
        ? material
        : Graphic3d_NameOfMaterial_ShinyPlastified;
}

Standard_Boolean OcctDocument::TryMaterialNameForLabel(
    const TDF_Label& label,
    Graphic3d_NameOfMaterial& material) const {
    Handle(TDataStd_Integer) attribute;
    const TDF_Label materialLabel = label.IsNull()
        ? TDF_Label()
        : label.FindChild(11, Standard_False);
    if (!materialLabel.IsNull()
        && materialLabel.FindAttribute(TDataStd_Integer::GetID(), attribute)
        && !attribute.IsNull()) {
        material = static_cast<Graphic3d_NameOfMaterial>(attribute->Get());
        return Standard_True;
    }
    return Standard_False;
}

Quantity_NameOfColor OcctDocument::ColorNameForLabel(const TDF_Label& label) const {
    Quantity_NameOfColor color;
    return TryColorNameForLabel(label, color)
        ? color
        : Quantity_NOC_GRAY80;
}

Standard_Boolean OcctDocument::TryColorNameForLabel(
    const TDF_Label& label,
    Quantity_NameOfColor& color) const {
    Handle(TDataStd_Integer) attribute;
    const TDF_Label colorLabel = label.IsNull()
        ? TDF_Label()
        : label.FindChild(12, Standard_False);
    if (!colorLabel.IsNull()
        && colorLabel.FindAttribute(TDataStd_Integer::GetID(), attribute)
        && !attribute.IsNull()) {
        color = static_cast<Quantity_NameOfColor>(attribute->Get());
        return Standard_True;
    }
    return Standard_False;
}

Standard_Boolean OcctDocument::TryPBRMaterialForLabel(
    const TDF_Label& label,
    XCAFDoc_VisMaterialPBR& material) const {
    if (label.IsNull()) {
        return Standard_False;
    }
    Handle(TDataStd_Integer) aMarker;
    if (!label.FindAttribute(LocalPBRMaterialAttributeID(), aMarker)
        || aMarker.IsNull() || aMarker->Get() != 1) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterial) aMaterial =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    if (aMaterial.IsNull() || !aMaterial->HasPbrMaterial()) {
        return Standard_False;
    }
    material = aMaterial->PbrMaterial();
    return material.IsDefined;
}

Standard_Boolean OcctDocument::TryEffectivePBRMaterialForLabel(
    const TDF_Label& label,
    XCAFDoc_VisMaterialPBR& material) const {
    if (label.IsNull()) {
        return Standard_False;
    }
    Handle(TDataStd_Integer) aMarker;
    const Standard_Boolean hasLocalPBR =
        label.FindAttribute(LocalPBRMaterialAttributeID(), aMarker)
        && !aMarker.IsNull() && aMarker->Get() == 1;
    Graphic3d_NameOfMaterial aLegacyMaterial;
    Quantity_NameOfColor aLegacyColor;
    const Standard_Boolean hasLegacyMaterial =
        TryMaterialNameForLabel(label, aLegacyMaterial);
    const Standard_Boolean hasLegacyColor =
        TryColorNameForLabel(label, aLegacyColor);
    if (!hasLocalPBR && hasLegacyMaterial) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterial) aVisualMaterial =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    if (aVisualMaterial.IsNull()) {
        return Standard_False;
    }
    if (aVisualMaterial->HasPbrMaterial()) {
        material = aVisualMaterial->PbrMaterial();
    } else if (aVisualMaterial->HasCommonMaterial()) {
        material = aVisualMaterial->ConvertToPbrMaterial();
    } else {
        return Standard_False;
    }
    // Imported assets may place their sole base-color map in the Common
    // compatibility representation while keeping authoritative PBR scalars.
    // Surface that as one effective base texture for selection/replacement;
    // SupportsBaseColorTextureEditingForLabel() separately rejects a genuine
    // PBR/Common conflict when both representations provide different maps.
    if (material.BaseColorTexture.IsNull()
        && aVisualMaterial->HasCommonMaterial()
        && !aVisualMaterial->CommonMaterial().DiffuseTexture.IsNull()) {
        material.BaseColorTexture =
            aVisualMaterial->CommonMaterial().DiffuseTexture;
    }
    if (!hasLocalPBR && hasLegacyColor) {
        material.BaseColor = Quantity_ColorRGBA(
            Quantity_Color(aLegacyColor), material.BaseColor.Alpha());
    }
    return material.IsDefined;
}

Standard_Boolean OcctDocument::SupportsScalarPBRMaterialEditingForLabel(
    const TDF_Label& label) const {
    if (label.IsNull()) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterial) material =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    if (material.IsNull()) {
        return Standard_True;
    }
    Handle(TDataStd_Integer) marker;
    const Standard_Boolean hasLocalPBR =
        label.FindAttribute(LocalPBRMaterialAttributeID(), marker)
        && !marker.IsNull() && marker->Get() == 1;
    if (material->HasPbrMaterial()) {
        const XCAFDoc_VisMaterialPBR& pbr = material->PbrMaterial();
        if (!pbr.MetallicRoughnessTexture.IsNull()
            || !pbr.OcclusionTexture.IsNull()
            || !pbr.NormalTexture.IsNull()) {
            return Standard_False;
        }
        const Handle(Image_Texture)& base = pbr.BaseColorTexture;
        const Handle(Image_Texture) common = material->HasCommonMaterial()
            ? material->CommonMaterial().DiffuseTexture
            : Handle(Image_Texture)();
        if (!base.IsNull() || !pbr.EmissiveTexture.IsNull()) {
            return hasLocalPBR
                && base.IsNull() == common.IsNull()
                && (base.IsNull()
                    || Core3DTexturesMatch(base, common));
        }
        return common.IsNull();
    }
    return !material->HasCommonMaterial()
        || material->CommonMaterial().DiffuseTexture.IsNull();
}

Standard_Boolean OcctDocument::SupportsBaseColorTextureEditingForLabel(
    const TDF_Label& label) const {
    if (label.IsNull()) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterial) material =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    if (material.IsNull()) {
        return Standard_True;
    }
    if (!material->HasPbrMaterial()
        && !material->HasCommonMaterial()) {
        return Standard_False;
    }

    Handle(Image_Texture) pbrBase;
    Handle(Image_Texture) emissive;
    if (material->HasPbrMaterial()) {
        const XCAFDoc_VisMaterialPBR& pbr = material->PbrMaterial();
        if (!pbr.MetallicRoughnessTexture.IsNull()
            || !pbr.OcclusionTexture.IsNull()
            || !pbr.NormalTexture.IsNull()) {
            return Standard_False;
        }
        pbrBase = pbr.BaseColorTexture;
        emissive = pbr.EmissiveTexture;
    }

    const Handle(Image_Texture) commonBase = material->HasCommonMaterial()
        ? material->CommonMaterial().DiffuseTexture
        : Handle(Image_Texture)();
    if (!pbrBase.IsNull() && !commonBase.IsNull()) {
        if (!Core3DTexturesMatch(pbrBase, commonBase)) {
            return Standard_False;
        }
    }
    if (!emissive.IsNull()) {
        Handle(TDataStd_Integer) marker;
        const Standard_Boolean hasLocalPBR =
            label.FindAttribute(LocalPBRMaterialAttributeID(), marker)
            && !marker.IsNull() && marker->Get() == 1;
        if (!hasLocalPBR) {
            // Base-color edits preserve emissive. Never silently take ownership
            // of an imported emissive map as a side effect of editing base.
            return Standard_False;
        }
    }
    return Standard_True;
}

Standard_Boolean OcctDocument::SupportsEmissiveTextureEditingForLabel(
    const TDF_Label& label) const {
    if (label.IsNull()) {
        return Standard_False;
    }
    const Handle(XCAFDoc_VisMaterial) material =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    if (material.IsNull()) {
        return Standard_True;
    }
    if (!material->HasPbrMaterial()
        && !material->HasCommonMaterial()) {
        return Standard_False;
    }

    Handle(Image_Texture) pbrBase;
    if (material->HasPbrMaterial()) {
        const XCAFDoc_VisMaterialPBR& pbr = material->PbrMaterial();
        if (!pbr.MetallicRoughnessTexture.IsNull()
            || !pbr.OcclusionTexture.IsNull()
            || !pbr.NormalTexture.IsNull()) {
            return Standard_False;
        }
        pbrBase = pbr.BaseColorTexture;
    }
    const Handle(Image_Texture) commonBase = material->HasCommonMaterial()
        ? material->CommonMaterial().DiffuseTexture
        : Handle(Image_Texture)();
    if (pbrBase.IsNull() && commonBase.IsNull()) {
        return Standard_True;
    }

    Handle(TDataStd_Integer) marker;
    const Standard_Boolean hasLocalPBR =
        label.FindAttribute(LocalPBRMaterialAttributeID(), marker)
        && !marker.IsNull() && marker->Get() == 1;
    return hasLocalPBR
        && !pbrBase.IsNull() && !commonBase.IsNull()
        && Core3DTexturesMatch(pbrBase, commonBase);
}

Standard_Boolean
OcctDocument::IsEmissiveTextureFactorAutoPromotedForLabel(
    const TDF_Label& label) const {
    if (label.IsNull()) {
        return Standard_False;
    }
    Handle(TDataStd_Integer) marker;
    return label.FindAttribute(
            AutoPromotedEmissiveFactorAttributeID(), marker)
        && !marker.IsNull() && marker->Get() == 1;
}

Standard_Boolean
OcctDocument::SetEmissiveTextureFactorAutoPromotedForLabel(
    const TDF_Label& label,
    const Standard_Boolean isAutoPromoted) {
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || label.IsNull()) {
        return Standard_False;
    }
    if (isAutoPromoted) {
        Handle(TDataStd_Integer) localMarker;
        const Handle(XCAFDoc_VisMaterial) material =
            XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
        if (!label.FindAttribute(
                LocalPBRMaterialAttributeID(), localMarker)
            || localMarker.IsNull() || localMarker->Get() != 1
            || material.IsNull() || !material->HasPbrMaterial()) {
            return Standard_False;
        }
        const XCAFDoc_VisMaterialPBR& pbr = material->PbrMaterial();
        if (pbr.EmissiveTexture.IsNull()
            || pbr.EmissiveFactor.x() != 1.0f
            || pbr.EmissiveFactor.y() != 1.0f
            || pbr.EmissiveFactor.z() != 1.0f) {
            return Standard_False;
        }
        TDataStd_Integer::Set(
            label, AutoPromotedEmissiveFactorAttributeID(), 1);
    } else {
        label.ForgetAttribute(
            AutoPromotedEmissiveFactorAttributeID());
    }
    return Standard_True;
}

void OcctDocument::LoadObjectMeterial(const TDF_Label& label, const Handle(AIS_Shape) anAis) {
    if (label.IsNull() || anAis.IsNull()) {
        return;
    }
    const Handle(XCAFDoc_VisMaterial) aVisualMaterial =
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    Handle(TDataStd_Integer) aLocalPBRMarker;
    const Standard_Boolean hasLocalPBR =
        label.FindAttribute(
            LocalPBRMaterialAttributeID(), aLocalPBRMarker)
        && !aLocalPBRMarker.IsNull()
        && aLocalPBRMarker->Get() == 1;
    Graphic3d_NameOfMaterial aLegacyMaterial;
    Quantity_NameOfColor aLegacyColor;
    const Standard_Boolean hasLegacyMaterial =
        TryMaterialNameForLabel(label, aLegacyMaterial);
    const Standard_Boolean hasLegacyColor =
        TryColorNameForLabel(label, aLegacyColor);
    const Handle(CafShapePrs) aCafPresentation =
        Handle(CafShapePrs)::DownCast(anAis);
    if (!aVisualMaterial.IsNull()) {
        Graphic3d_MaterialAspect anAspect;
        aVisualMaterial->FillMaterialAspect(anAspect);
        if (hasLocalPBR) {
            if (!aCafPresentation.IsNull()) {
                aCafPresentation->ApplyAuthoredVisualMaterial(
                    aVisualMaterial);
            } else {
                ApplyVisualMaterialToPlainPresentation(
                    aVisualMaterial, anAis);
            }
            return;
        }
        if (!aCafPresentation.IsNull()) {
            // CafShapePrs' default-style path already uses FillAspect() and
            // retains imported face/occurrence precedence.
            anAis->SetMaterial(anAspect);
            anAis->SetColor(aVisualMaterial->BaseColor().GetRGB());
        } else {
            ApplyVisualMaterialToPlainPresentation(
                aVisualMaterial, anAis);
        }
    }
    const Graphic3d_MaterialAspect aLegacyAspect = hasLegacyMaterial
        ? Graphic3d_MaterialAspect(aLegacyMaterial)
        : Graphic3d_MaterialAspect();
    const Quantity_Color aLegacyQuantity = hasLegacyColor
        ? Quantity_Color(aLegacyColor)
        : Quantity_Color(Quantity_NOC_GRAY80);
    if (!aCafPresentation.IsNull()
        && (hasLegacyMaterial || hasLegacyColor)) {
        aCafPresentation->ApplyAuthoredLegacyAppearance(
            hasLegacyMaterial,
            aLegacyAspect,
            hasLegacyColor,
            aLegacyQuantity);
    } else {
        if (hasLegacyMaterial) {
            ResetDrawerForLegacyMaterial(anAis->Attributes());
        } else if (aVisualMaterial.IsNull()) {
            ClearDrawerTextureMapping(anAis->Attributes());
        }
        if (hasLegacyMaterial) {
            anAis->SetMaterial(aLegacyAspect);
        }
        if (hasLegacyColor) {
            anAis->SetColor(aLegacyQuantity);
        }
        if (hasLegacyMaterial || hasLegacyColor
            || aVisualMaterial.IsNull()) {
            anAis->SynchronizeAspects();
        }
    }

}

void OcctDocument::LoadObjectAuthoredMaterialOverrides(
    const TDF_Label& label,
    const Handle(AIS_Shape) anAis) {
    if (label.IsNull() || anAis.IsNull()) {
        return;
    }

    Handle(TDataStd_Integer) aLocalPBRMarker;
    const Standard_Boolean hasLocalPBR =
        label.FindAttribute(
            LocalPBRMaterialAttributeID(), aLocalPBRMarker)
        && !aLocalPBRMarker.IsNull()
        && aLocalPBRMarker->Get() == 1;
    if (hasLocalPBR) {
        const Handle(XCAFDoc_VisMaterial) aVisualMaterial =
            XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
        if (!aVisualMaterial.IsNull()) {
            Graphic3d_MaterialAspect anAspect;
            aVisualMaterial->FillMaterialAspect(anAspect);
            const Handle(CafShapePrs) aCafPresentation =
                Handle(CafShapePrs)::DownCast(anAis);
            if (!aCafPresentation.IsNull()) {
                aCafPresentation->ApplyAuthoredVisualMaterial(
                    aVisualMaterial);
            } else {
                ApplyVisualMaterialToPlainPresentation(
                    aVisualMaterial, anAis);
            }
            return;
        }
    }

    Graphic3d_NameOfMaterial aLegacyMaterial;
    const Standard_Boolean hasLegacyMaterial =
        TryMaterialNameForLabel(label, aLegacyMaterial);
    Quantity_NameOfColor aLegacyColor;
    const Standard_Boolean hasLegacyColor =
        TryColorNameForLabel(label, aLegacyColor);
    const Graphic3d_MaterialAspect aLegacyAspect = hasLegacyMaterial
        ? Graphic3d_MaterialAspect(aLegacyMaterial)
        : Graphic3d_MaterialAspect();
    const Quantity_Color aLegacyQuantity = hasLegacyColor
        ? Quantity_Color(aLegacyColor)
        : Quantity_Color(Quantity_NOC_GRAY80);
    const Handle(CafShapePrs) aCafPresentation =
        Handle(CafShapePrs)::DownCast(anAis);
    if (!aCafPresentation.IsNull()
        && (hasLegacyMaterial || hasLegacyColor)) {
        aCafPresentation->ApplyAuthoredLegacyAppearance(
            hasLegacyMaterial,
            aLegacyAspect,
            hasLegacyColor,
            aLegacyQuantity);
    } else {
        if (hasLegacyMaterial) {
            ResetDrawerForLegacyMaterial(anAis->Attributes());
        }
        if (hasLegacyMaterial) {
            anAis->SetMaterial(aLegacyAspect);
        }
        if (hasLegacyColor) {
            anAis->SetColor(aLegacyQuantity);
        }
        if (hasLegacyMaterial || hasLegacyColor) {
            anAis->SynchronizeAspects();
        }
    }
}


Standard_Boolean OcctDocument::OpenPrivateExportSnapshot(
    const std::string& path,
    const Message_ProgressRange& progress) {
    if (path.empty() || myApp.IsNull() || !myOcafDoc.IsNull()) {
        return Standard_False;
    }

    Handle(TDocStd_Document) candidate;
    try {
        OCC_CATCH_SIGNALS
        Core3DDefineSafeBinXCAFFormat(myApp);
        Core3DBeginSafeBinaryRead();
        const PCDM_ReaderStatus status = myApp->Open(
            TCollection_ExtendedString(path.c_str(), Standard_True),
            candidate,
            progress);
        const Standard_Boolean wasRejected =
            Core3DSafeBinaryReadWasRejected();
        if (wasRejected || status != PCDM_RS_OK || candidate.IsNull()) {
            if (!candidate.IsNull()) {
                try {
                    myApp->Close(candidate);
                } catch (...) {
                }
                candidate.Nullify();
            }
            return Standard_False;
        }
        myOcafDoc = candidate;
        return Standard_True;
    } catch (...) {
        if (!candidate.IsNull()) {
            try {
                myApp->Close(candidate);
            } catch (...) {
            }
            candidate.Nullify();
        }
        return Standard_False;
    }
}

void OcctDocument::ClosePrivateExportSnapshot() noexcept {
    if (myApp.IsNull() || myOcafDoc.IsNull()) {
        myOcafDoc.Nullify();
        return;
    }
    try {
        if (myOcafDoc->HasOpenCommand()) {
            myOcafDoc->AbortCommand();
        }
    } catch (...) {
    }
    try {
        myApp->Close(myOcafDoc);
    } catch (...) {
    }
    myOcafDoc.Nullify();
}

void OcctDocument::ApplyTransforms() {
    ApplyTransforms(Message_ProgressRange());
}

Standard_Boolean OcctDocument::ApplyTransforms(
    const Message_ProgressRange& progress) {
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    TDF_LabelSequence aLabels;
    shapeTool->GetFreeShapes (aLabels);
    Message_ProgressScope aScope(
        progress,
        "Apply object transforms",
        aLabels.Length());
    for (Standard_Integer aLabIter = 1;
         aLabIter <= aLabels.Length();
         ++aLabIter)
    {
        if (!aScope.More()) {
            return Standard_False;
        }
        const TDF_Label& aLabel = aLabels.Value (aLabIter);
        const auto t = ObjectTransformForLabel(aLabel);
        TNaming::Displace(aLabel, TopLoc_Location(t));
        aScope.Next();
    }
    return Standard_True;
}

gp_Trsf OcctDocument::ObjectTransformForLabel(const TDF_Label& aRefLabel) const {
    const auto readReal = [&aRefLabel](const Standard_Integer theTag,
                                      const Standard_Real theDefault) {
        if (aRefLabel.IsNull()) {
            return theDefault;
        }
        const TDF_Label aChild = aRefLabel.FindChild(theTag, Standard_False);
        if (aChild.IsNull()) {
            return theDefault;
        }
        Handle(TDataStd_Real) anAttribute;
        return aChild.FindAttribute(TDataStd_Real::GetID(), anAttribute)
            && !anAttribute.IsNull()
            ? anAttribute->Get()
            : theDefault;
    };

    const Standard_Real x = readReal(1, 0.0);
    const Standard_Real y = readReal(2, 0.0);
    const Standard_Real z = readReal(3, 0.0);
    const Standard_Real rx = readReal(4, 0.0);
    const Standard_Real ry = readReal(5, 0.0);
    const Standard_Real rz = readReal(6, 0.0);
    const Standard_Real rw = readReal(7, 1.0);
    const Standard_Real scale = readReal(8, 1.0);
    
    gp_Trsf t = gp_Trsf();
    t.SetTranslation({x, y, z});
    t.SetRotationPart({rx, ry, rz, rw});
    t.SetScaleFactor(scale);
    return t;
}

void OcctDocument::LoadObjectTransform(const TDF_Label& aRefLabel, const Handle(AIS_Shape) anAis) {
    anAis->SetLocalTransformation(ObjectTransformForLabel(aRefLabel));
}

Standard_Boolean OcctDocument::undo() {
    if (!canUndo()) {
		return Standard_False;
    }
    try {
        if (myOcafDoc->Undo()) {
            NotifyChanges();
			return Standard_True;
        }
    } catch (const Standard_Failure& ex) {
        std::cout << ex.GetMessageString() << std::endl;
    }
	return Standard_False;
}
Standard_Boolean OcctDocument::redo() {
    if (!canRedo()) {
		return Standard_False;
    }
    try {
		if (myOcafDoc->Redo()) {
			NotifyChanges();
			return Standard_True;
		}
	} catch(const Standard_Failure& ex) {
        std::cout << ex.GetMessageString() << std::endl;
    }
	return Standard_False;
}

const bool OcctDocument::canUndo() const {
	return !myOcafDoc.IsNull() && !myOcafDoc->HasOpenCommand()
		&& myOcafDoc->GetAvailableUndos() > 0;
}

const bool OcctDocument::canRedo() const {
	return !myOcafDoc.IsNull() && !myOcafDoc->HasOpenCommand()
		&& myOcafDoc->GetAvailableRedos() > 0;
}

std::string OcctDocument::save(const std::string& path) {
    return save(path, Message_ProgressRange());
}

std::string OcctDocument::save(
    const std::string& path,
    const Message_ProgressRange& progress) {
    if (myOcafDoc.IsNull() || myOcafDoc->HasOpenCommand()) {
        return {};
    }

    auto app =  Handle(TDocStd_Application)::DownCast(myOcafDoc->Application());
    if (app.IsNull()) {
        return {};
    }

    try {
        PCDM_StoreStatus status = app->SaveAs(
            myOcafDoc, path.c_str(), progress); // ".cbf"
        if (status != PCDM_SS_OK) {
            std::cout << "Save CBF failed with status " << status << std::endl;
            return {};
        }
        const TCollection_ExtendedString aBinXCAFFormat("BinXCAF");
        return path + (myOcafDoc->StorageFormat().IsEqual(aBinXCAFFormat)
            ? ".xbf"
            : ".cbf");
    } catch (const Standard_Failure& failure) {
        std::cout << "Save CBF failure: " << failure.GetMessageString() << std::endl;
        return {};
    }
}

void OcctDocument::NotifyChanges() {
    [[NSNotificationCenter defaultCenter]
     postNotificationName:@"OcctDocumentChanges"
     object:[NSValue valueWithPointer:this]];
}
