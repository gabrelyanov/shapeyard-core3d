#include "NativeObservedApplication.hxx"
#include "NativeMeshVertexMove.hxx"
#include "NativeMeshWindingCandidate.hxx"
#if DEBUG
#include "NativeLiveTransactionObserverProbe.hxx"
#endif
#include "../Scene/MikkTangentSpace.hpp"
#include <RWMesh_FaceIterator.hxx>
#include "CoherentMeshUVAtlas.hpp"
#include <TDataStd_UAttribute.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_GraphNode.hxx>
#include <TDataStd_Name.hxx>
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
#include "AuthoredFrameAttributeID.hxx"
#include "NativeAuthoredFrameGeometry.hxx"
#include <TDataStd_ByteArray.hxx>
#include <TDF_AttributeIterator.hxx>
#include "Core3DBoundedAuthoredFrameDriver.hxx"
#include "CafShapePrs.h"
#include "../Common/Core3DMobileResourceLimits.h"

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
#include <Storage_HeaderData.hxx>
#include <Storage_Schema.hxx>

#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc.hxx>
#include <XCAFDoc_VisMaterial.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>
#include <TDataStd_Real.hxx>
#include <TDataStd_TreeNode.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRep_Builder.hxx>
#include <gp_Trsf.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <GP_Quaternion.hxx>
#include <TNaming.hxx>
#include <TNaming_NamedShape.hxx>
#include <Standard_GUID.hxx>
#include <TDF_LabelMap.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_MapOfShape.hxx>
#include <XCAFPrs_DocumentExplorer.hxx>
#include <Graphic3d_TextureSet.hxx>
#include "Core3DDataMapShader.hxx"
#include <Graphic3d_TextureParams.hxx>
#include <XCAFPrs_Texture.hxx>
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

const Standard_GUID& Core3DDuplicateCommandOwnerAttributeID()
{
    static const Standard_GUID anId(
        "D7598D08-A879-4E17-8D23-CC92568EAD5C");
    return anId;
}

const Standard_GUID& Core3DOrdinaryEditCommandOwnerAttributeID()
{
    static const Standard_GUID anId("58BBFE2E-F5F9-4F87-A7EA-EC1459362C89");
    return anId;
}

const Standard_GUID& Core3DRadialArrayCommandOwnerAttributeID()
{
    static const Standard_GUID anId(
        "2DF9F8BE-F297-4CFD-8D93-A40346EBEF3A");
    return anId;
}

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
    if (IsCore3DDataMapShader(aShading->Aspect()->ShaderProgram())) {
        aShading->Aspect()->SetShaderProgram({});
    }
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
    Core3DPrepareRendererTextures(aFillAspect);
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

// Numeric PNGs are sampled as stored channel codes. Do not draw through a
// color-managed CGContext: a linear-tagged roughness value of 128 is not an
// sRGB color. The narrow admission matches the mobile numeric importer.
template<class T> struct ScopedNumericCF {
    T value;
    explicit ScopedNumericCF(T v) : value(v) {}
    ~ScopedNumericCF() { if (value != nullptr) CFRelease(value); }
    ScopedNumericCF(const ScopedNumericCF&) = delete;
    ScopedNumericCF& operator=(const ScopedNumericCF&) = delete;
};

Handle(Image_PixMap) DecodeNumericRendererPNG(
    const Handle(NCollection_Buffer)& buffer)
{
    Standard_Size outputBytes = 0;
    if (!TryTextureDecodedBytes(buffer, outputBytes)
        || buffer.IsNull() || buffer->Size() < 33) return {};
    const Standard_Byte* header = buffer->Data();
    static const unsigned char prefix[] = {
        137,80,78,71,13,10,26,10,0,0,0,13,73,72,68,82};
    if (std::memcmp(header, prefix, sizeof(prefix)) != 0
        || header[24] != 8
        || !(header[25] == 0 || header[25] == 2
             || header[25] == 4 || header[25] == 6)) return {};
    ScopedNumericCF<CFDataRef> data(CFDataCreateWithBytesNoCopy(
        kCFAllocatorDefault, header, static_cast<CFIndex>(buffer->Size()),
        kCFAllocatorNull));
    if (data.value == nullptr) return {};
    NSDictionary* options = @{(__bridge NSString*)kCGImageSourceShouldCache: @NO};
    ScopedNumericCF<CGImageSourceRef> source(CGImageSourceCreateWithData(
        data.value, (__bridge CFDictionaryRef)options));
    if (source.value == nullptr || CGImageSourceGetCount(source.value) != 1
        || CGImageSourceGetStatus(source.value) != kCGImageStatusComplete
        || CGImageSourceGetStatusAtIndex(source.value, 0) != kCGImageStatusComplete)
        return {};
    ScopedNumericCF<CFDictionaryRef> properties(CGImageSourceCopyPropertiesAtIndex(
        source.value, 0, (__bridge CFDictionaryRef)options));
    if (properties.value == nullptr) return {};
    CFTypeRef orientation = CFDictionaryGetValue(properties.value,
                                                 kCGImagePropertyOrientation);
    if (orientation != nullptr) {
        double value = 0;
        if (CFGetTypeID(orientation) != CFNumberGetTypeID()
            || !CFNumberGetValue(static_cast<CFNumberRef>(orientation),
                                kCFNumberDoubleType, &value) || value != 1) return {};
    }
    ScopedNumericCF<CGImageRef> image(CGImageSourceCreateImageAtIndex(
        source.value, 0, (__bridge CFDictionaryRef)options));
    if (image.value == nullptr || CGImageGetBitsPerComponent(image.value) != 8
        || (CGImageGetBitmapInfo(image.value) & kCGBitmapFloatComponents)) return {};
    const size_t width = CGImageGetWidth(image.value);
    const size_t height = CGImageGetHeight(image.value);
    std::uint64_t renderWidth = 0, renderHeight = 0;
    if (!TryRendererTextureDimensions(width, height, renderWidth, renderHeight)
        || renderWidth * renderHeight * 4 != outputBytes) return {};
    CGColorSpaceRef space = CGImageGetColorSpace(image.value);
    if (space == nullptr) return {};
    const CGColorSpaceModel model = CGColorSpaceGetModel(space);
    if (model != kCGColorSpaceModelRGB && model != kCGColorSpaceModelMonochrome)
        return {};
    const int channels = model == kCGColorSpaceModelMonochrome ? 1 : 3;
    if (CGColorSpaceGetNumberOfComponents(space) != channels) return {};
    const CGFloat* decode = CGImageGetDecode(image.value);
    for (int c = 0; decode != nullptr && c < channels; ++c)
        if (decode[c * 2] != 0 || decode[c * 2 + 1] != 1) return {};
    const size_t bits = CGImageGetBitsPerPixel(image.value);
    if (bits % 8 != 0) return {};
    const size_t stride = bits / 8;
    int colorStart = 0, alphaIndex = -1;
    const CGImageAlphaInfo alpha = CGImageGetAlphaInfo(image.value);
    switch (alpha) {
        case kCGImageAlphaNone:
            if (stride != channels) return {};
            break;
        case kCGImageAlphaLast: case kCGImageAlphaPremultipliedLast:
        case kCGImageAlphaNoneSkipLast:
            if (stride != channels + 1) return {};
            alphaIndex = alpha == kCGImageAlphaNoneSkipLast ? -1 : channels;
            break;
        case kCGImageAlphaFirst: case kCGImageAlphaPremultipliedFirst:
        case kCGImageAlphaNoneSkipFirst:
            if (stride != channels + 1) return {};
            colorStart = 1;
            alphaIndex = alpha == kCGImageAlphaNoneSkipFirst ? -1 : 0;
            break;
        default: return {};
    }
    const CGBitmapInfo order = CGImageGetBitmapInfo(image.value) & kCGBitmapByteOrderMask;
    const bool reverse = stride == 4 && order == kCGBitmapByteOrder32Little;
    if (!(order == kCGBitmapByteOrderDefault
          || (stride == 4 && order == kCGBitmapByteOrder32Big) || reverse)) return {};
    CGDataProviderRef provider = CGImageGetDataProvider(image.value);
    if (provider == nullptr) return {};
    const size_t rowBytes = CGImageGetBytesPerRow(image.value);
    if (rowBytes < width * stride
        || rowBytes > (kMaximumDecodedTextureBytes - outputBytes) / height) return {};
    ScopedNumericCF<CFDataRef> raw(CGDataProviderCopyData(provider));
    if (raw.value == nullptr || CFDataGetLength(raw.value) < 0) return {};
    const auto rawSize = static_cast<std::uint64_t>(CFDataGetLength(raw.value));
    if (rowBytes < width * stride || rowBytes > rawSize / height
        || rawSize > kMaximumDecodedTextureBytes - outputBytes) return {};
    const UInt8* bytes = CFDataGetBytePtr(raw.value);
    if (bytes == nullptr) return {};
    const auto sample = [&](size_t x, size_t y, int c) -> unsigned int {
        return bytes[y * rowBytes + x * stride + (reverse ? stride - 1 - c : c)];
    };
    // Validate every source sample, including ones down/up-sampling may skip.
    if (alphaIndex >= 0)
        for (size_t y = 0; y < height; ++y)
            for (size_t x = 0; x < width; ++x)
                if (sample(x, y, alphaIndex) != 255) return {};
    Handle(Image_PixMap) result = new Image_PixMap();
    if (!result->InitTrash(Image_Format_RGBA, renderWidth, renderHeight,
                           renderWidth * 4)) return {};
    result->SetTopDown(true);
    // OCCT ES2 needs a POT mipmapped backing. Interpolate numeric codes in
    // linear space only on NPOT axes; POT images preserve each code exactly.
    for (size_t y = 0; y < renderHeight; ++y) {
        const double sy = std::max(0.0, std::min(double(height - 1),
            (double(y) + 0.5) * height / renderHeight - 0.5));
        const size_t y0 = static_cast<size_t>(sy), y1 = std::min(y0 + 1, height - 1);
        const double fy = sy - y0;
        Standard_Byte* out = result->ChangeRow(y);
        for (size_t x = 0; x < renderWidth; ++x, out += 4) {
            const double sx = std::max(0.0, std::min(double(width - 1),
                (double(x) + 0.5) * width / renderWidth - 0.5));
            const size_t x0 = static_cast<size_t>(sx), x1 = std::min(x0 + 1, width - 1);
            const double fx = sx - x0;
            for (int c = 0; c < 3; ++c) {
                const int channel = colorStart + (channels == 1 ? 0 : c);
                const double top = (1 - fx) * sample(x0, y0, channel) + fx * sample(x1, y0, channel);
                const double bottom = (1 - fx) * sample(x0, y1, channel) + fx * sample(x1, y1, channel);
                out[c] = static_cast<Standard_Byte>(std::lround((1 - fy) * top + fy * bottom));
            }
            out[3] = 255;
        }
    }
    return result;
}

// Slot interpretation belongs to the renderer binding, never to Image_Texture
// or its persisted content address. XCAFPrs_Texture otherwise shares the same
// GPU key for identical bytes even when one binding is sRGB and another linear.
class Core3DRoleAwareTexture final : public XCAFPrs_Texture {
    DEFINE_STANDARD_RTTI_INLINE(Core3DRoleAwareTexture, XCAFPrs_Texture)
public:
    explicit Core3DRoleAwareTexture(const Handle(XCAFPrs_Texture)& original)
    : XCAFPrs_Texture(original->GetImageSource(), original->GetParams()->TextureUnit()) {
        myParams = original->GetParams();
        myHasMipmaps = original->HasMipmaps();
        myTexId += IsColorMap() ? "|shapeyard-color-v1" : "|shapeyard-data-v1";
    }
    Handle(Image_CompressedPixMap) GetCompressedImage(
        const Handle(Image_SupportedFormats)&) override { return {}; }
    Handle(Image_PixMap) GetImage(
        const Handle(Image_SupportedFormats)& supported) override {
        if (IsColorMap()) return XCAFPrs_Texture::GetImage(supported);
        Handle(Image_PixMap) result = GetImageSource().IsNull()
            ? Handle(Image_PixMap)()
            : DecodeNumericRendererPNG(GetImageSource()->DataBuffer());
        if (!result.IsNull() && !supported.IsNull()) convertToCompatible(supported, result);
        return result;
    }
};

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
    : myAggregateTextureBytes(std::make_shared<Standard_Size>(0)),
      myFrameBudget(std::make_shared<core3d::persistence::AuthoredFrameReadBudget>())
    {
    }

#if DEBUG
    explicit Core3DBoundedBinXCAFRetrievalDriver(
        std::shared_ptr<core3d::persistence::AuthoredFrameReadBudget> budget)
        : Core3DBoundedBinXCAFRetrievalDriver() { myFrameBudget = std::move(budget); myValidateFrameOwners = false; }
#endif

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
        ResetAggregateReadBudgets();
        const auto rejectTypes = [&]() {
            myReaderStatus = PCDM_RS_TypeFailure;
            RejectSafeBinaryRead();
        };
        if (theStorageData.IsNull() || theStorageData->HeaderData().IsNull()) {
            rejectTypes(); return;
        }
        const auto header = theStorageData->HeaderData();
        if (!header->StorageVersion().IsIntegerValue()) { rejectTypes(); return; }
        const auto version = header->StorageVersion().IntegerValue();
        if (version < TDocStd_FormatVersion_LOWER || version > TDocStd_FormatVersion_CURRENT) {
            // Preserve the caller's UnsupportedVersion result for a different
            // document format; this is distinct from malformed attribute data.
            myReaderStatus = PCDM_RS_NoVersion; return;
        }
        // BinLDrivers resolves attribute IDs from this UserInfo section,
        // not Storage_Data::TypeData (the unrelated storage-object table).
        // Reject unsupported attributes before OCCT can silently skip them.
        const auto& info = header->UserInfo();
        if (info.Length() < 2 || info.Length() > 1024) { rejectTypes(); return; }
        TColStd_SequenceOfAsciiString aTypeNames;
        bool began = false, ended = false;
        for (Standard_Integer i = 1; i <= info.Length(); ++i) {
            const auto& line = info.Value(i);
            if (line == "START_TYPES") {
                if (began || ended) { rejectTypes(); return; }
                began = true; continue;
            }
            if (line == "END_TYPES") {
                if (!began || ended) { rejectTypes(); return; }
                ended = true; continue;
            }
            if (!began || ended) continue;
            if (line.IsEmpty() || line.Length() > 128 || aTypeNames.Length() >= 128) {
                rejectTypes(); return;
            }
            TCollection_AsciiString name = line;
            if (version < TDocStd_FormatVersion_VERSION_8) {
                TCollection_AsciiString migrated;
                if (Storage_Schema::CheckTypeMigration(name, migrated)) name = migrated;
            }
            if (name.IsEmpty() || name.Length() > 128) { rejectTypes(); return; }
            for (Standard_Integer j = 1; j <= aTypeNames.Length(); ++j)
                if (aTypeNames.Value(j) == name) { rejectTypes(); return; }
            aTypeNames.Append(name);
        }
        if (!began || !ended) { rejectTypes(); return; }
        Handle(BinMDF_ADriverTable) aSupportedDrivers =
            AttributeDrivers(Message::DefaultMessenger());
        aSupportedDrivers->AssignIds(aTypeNames);
        for (Standard_Integer anIndex = 1;
             anIndex <= aTypeNames.Length(); ++anIndex) {
            if (aSupportedDrivers->GetDriver(anIndex).IsNull()) {
                rejectTypes();
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
            if (myValidateFrameOwners && myReaderStatus == PCDM_RS_OK) {
                Standard_Size frameBytes = 0;
                if (gSafeBinaryReadRejected || !Core3DValidateAuthoredFrameOwners(
                        Handle(TDocStd_Document)::DownCast(theDocument), frameBytes)) rejectTypes();
            }
        } catch (...) {
            ResetAggregateReadBudgets();
            throw;
        }
        ResetAggregateReadBudgets();
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
        aTable->AddDriver(new core3d::persistence::BoundedAuthoredFrameDriver(
            theMessageDriver, myFrameBudget, RejectSafeBinaryRead));
        return aTable;
    }

    void Clear() override
    {
        try { BinDrivers_DocumentRetrievalDriver::Clear(); }
        catch (...) { ResetAggregateReadBudgets(); throw; }
        ResetAggregateReadBudgets();
    }

private:
    void ResetAggregateReadBudgets() noexcept
    {
        if (myFrameBudget) { myFrameBudget->bytes = 0; myFrameBudget->rejected = false; }
        if (myAggregateTextureBytes != nullptr) {
            *myAggregateTextureBytes = 0;
        }
    }

    std::shared_ptr<Standard_Size> myAggregateTextureBytes;
    std::shared_ptr<core3d::persistence::AuthoredFrameReadBudget> myFrameBudget;
    bool myValidateFrameOwners = true;
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

//! Definition-owned discriminator between editable BRep and imported
//! triangle-only geometry. This GUID and the non-negative enum values in the
//! public header are persistent schema identifiers.
const Standard_GUID& MeshUVAtlasAttributeID() {
    static const Standard_GUID id("9bf75aca-8719-4aca-9a5e-c7b4c3a40ba1");
    return id;
}

const Standard_GUID& MeshUVAtlasSettingsAttributeID(const int index) {
    // Fixed scalar attributes use the existing bounded binary schema. Never
    // broaden the document reader to arbitrary arrays for this three-value record.
    static const Standard_GUID ids[] = {
        Standard_GUID("137327fe-a06c-4be4-b45b-345f5343bda2"),
        Standard_GUID("29dd8d60-6fd2-4139-bbc5-9d8e9b469f9d"),
        Standard_GUID("a46a3ce4-2af3-4a77-a17d-b403e1503f13")
    };
    return ids[index];
}

const Standard_GUID& GeometryRepresentationAttributeID()
{
    static const Standard_GUID anId("67E669F4-00C0-4C45-BC55-9CC5DA22A2B5");
    return anId;
}

//! A reference-axis record is seven custom scalar attributes on one free,
//! simple definition label. These GUIDs and the encoded mode values are
//! permanent serialized schema identifiers: never renumber or reuse them.
const Standard_GUID& ReferenceAxisModeAttributeID()
{
    static const Standard_GUID anId("26128380-D69C-4530-B856-0C2AEEF47E60");
    return anId;
}

const Standard_GUID& ReferenceAxisPivotXAttributeID()
{
    static const Standard_GUID anId("11531A1D-14DB-4F78-AD91-814981846842");
    return anId;
}

const Standard_GUID& ReferenceAxisPivotYAttributeID()
{
    static const Standard_GUID anId("BA2AE804-64BD-480B-8910-B1144DA1AAD3");
    return anId;
}

const Standard_GUID& ReferenceAxisPivotZAttributeID()
{
    static const Standard_GUID anId("7CDD4B6E-5375-48F2-BAA9-AD76ACF6D47A");
    return anId;
}

const Standard_GUID& ReferenceAxisDirectionXAttributeID()
{
    static const Standard_GUID anId("CCEC34C3-8D3A-447A-8C70-EDB35A5B7E1C");
    return anId;
}

const Standard_GUID& ReferenceAxisDirectionYAttributeID()
{
    static const Standard_GUID anId("1DDBE964-693E-460A-9095-E47713BA28C0");
    return anId;
}

const Standard_GUID& ReferenceAxisDirectionZAttributeID()
{
    static const Standard_GUID anId("09CC9F05-9628-4C84-B214-2676C8BED8AA");
    return anId;
}

constexpr Standard_Integer kReferenceAxisSchemaV1 = 0x0100;
constexpr Standard_Integer kReferenceAxisPivotWorldBit = 0x0001;
constexpr Standard_Integer kReferenceAxisDirectionWorldBit = 0x0002;
constexpr Standard_Real kReferenceAxisUnitTolerance = 1.0e-10;

const std::array<const Standard_GUID*, 7>& ReferenceAxisAttributeIDs()
{
    static const std::array<const Standard_GUID*, 7> anIds = {{
        &ReferenceAxisModeAttributeID(),
        &ReferenceAxisPivotXAttributeID(),
        &ReferenceAxisPivotYAttributeID(),
        &ReferenceAxisPivotZAttributeID(),
        &ReferenceAxisDirectionXAttributeID(),
        &ReferenceAxisDirectionYAttributeID(),
        &ReferenceAxisDirectionZAttributeID(),
    }};
    return anIds;
}

constexpr Standard_Size kMaximumGeometryDocumentLabels = 100'000;
static_assert(core3d::profile::MaximumLabels == kMaximumGeometryDocumentLabels);

Standard_Boolean ValidateCommandOwnerSentinelsDocument(
    const Handle(TDocStd_Document)& theDocument)
{
    try {
        OCC_CATCH_SIGNALS
        if (theDocument.IsNull() || theDocument->GetData().IsNull()) {
            return Standard_False;
        }
        const TDF_Label aRoot = theDocument->GetData()->Root();
        const TDF_Label aMain = theDocument->Main();
        if (aRoot.IsNull() || aMain.IsNull()) {
            return Standard_False;
        }
        const auto isValidLabel = [&](const TDF_Label& theLabel) {
            const std::array<const Standard_GUID*, 3> anIds = {{
                &Core3DDuplicateCommandOwnerAttributeID(),
                &Core3DRadialArrayCommandOwnerAttributeID(),
                &Core3DOrdinaryEditCommandOwnerAttributeID(),
            }};
            for (const Standard_GUID* anId : anIds) {
                Handle(TDF_Attribute) anAttribute;
                if (!theLabel.FindAttribute(*anId, anAttribute)) {
                    continue;
                }
                if (!theLabel.IsEqual(aMain)
                    || anAttribute.IsNull()
                    || Handle(TDataStd_Integer)::DownCast(
                        anAttribute).IsNull()) {
                    return false;
                }
            }
            return true;
        };
        if (!isValidLabel(aRoot)) {
            return Standard_False;
        }
        Standard_Size aLabelCount = 0;
        for (TDF_ChildIterator aLabel(aRoot, Standard_True);
             aLabel.More(); aLabel.Next()) {
            if (++aLabelCount > kMaximumGeometryDocumentLabels
                || !isValidLabel(aLabel.Value())) {
                return Standard_False;
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

constexpr Standard_Size kMaximumGeometryDefinitionLabels = 4'096;
constexpr Standard_Size kMaximumSubshapesPerDefinition = 8'192;
constexpr Standard_Size kMaximumSubshapesPerDocument = 131'072;
constexpr Standard_Size kMaximumTopologyDepth = 128;
constexpr Standard_Size kMaximumMeshVerticesPerDefinition = 1'000'000;
constexpr Standard_Size kMaximumMeshVerticesPerDocument = 1'500'000;
constexpr Standard_Size kMaximumMeshIndicesPerDefinition = 3'000'000;
constexpr Standard_Size kMaximumMeshIndicesPerDocument = 4'500'000;
static_assert(
    static_cast<Standard_Integer>(
        OcctGeometryRepresentation::LegacyUnknown) == 0);
static_assert(
    static_cast<Standard_Integer>(
        OcctGeometryRepresentation::BRep) == 1);
static_assert(
    static_cast<Standard_Integer>(
        OcctGeometryRepresentation::TriangleMesh) == 2);

enum class DefinitionGeometryClass
{
    Invalid,
    BRep,
    TriangleMesh,
};

struct GeometryValidationBudget
{
    Standard_Size subshapes = 0;
    Standard_Size meshVertices = 0;
    Standard_Size meshIndices = 0;
};

bool IsFiniteBoundedMeshCoordinate(const Standard_Real theValue) noexcept
{
    return std::isfinite(theValue)
        && std::abs(theValue)
            <= core3d::limits::kMaximumModelCoordinateMagnitude;
}

bool AddWithinLimit(Standard_Size& theAggregate,
                    const Standard_Size theValue,
                    const Standard_Size theMaximum) noexcept
{
    if (theAggregate > theMaximum
        || theValue > theMaximum - theAggregate) {
        return false;
    }
    theAggregate += theValue;
    return true;
}

bool AddMultipliedWithinLimit(Standard_Size& theAggregate,
                              const Standard_Size theValue,
                              const Standard_Size theMultiplier,
                              const Standard_Size theMaximum) noexcept
{
    if (theAggregate > theMaximum) {
        return false;
    }
    const Standard_Size aRemaining = theMaximum - theAggregate;
    if (theValue != 0U && theMultiplier > aRemaining / theValue) {
        return false;
    }
    theAggregate += theValue * theMultiplier;
    return true;
}

bool IsGeometryDefinitionLabel(
    const Handle(TDocStd_Document)& theDocument,
    const Handle(XCAFDoc_ShapeTool)& theShapeTool,
    const TDF_Label& theLabel)
{
    return !theDocument.IsNull() && !theShapeTool.IsNull()
        && !theLabel.IsNull()
        && theLabel.Data() == theDocument->GetData()
        && theShapeTool->IsShape(theLabel)
        && XCAFDoc_ShapeTool::IsSimpleShape(theLabel)
        && !XCAFDoc_ShapeTool::IsAssembly(theLabel)
        && !XCAFDoc_ShapeTool::IsReference(theLabel)
        && !XCAFDoc_ShapeTool::IsComponent(theLabel)
        && !XCAFDoc_ShapeTool::IsSubShape(theLabel);
}

OcctReferenceAxis DefaultReferenceAxis()
{
    OcctReferenceAxis anAxis;
    anAxis.pivotSpace = OcctReferenceSpace::Object;
    anAxis.pivot = gp_Pnt(0.0, 0.0, 0.0);
    anAxis.directionSpace = OcctReferenceSpace::World;
    anAxis.direction = gp_Dir(0.0, 0.0, 1.0);
    return anAxis;
}

bool IsReferenceSpace(const OcctReferenceSpace theSpace) noexcept
{
    return theSpace == OcctReferenceSpace::Object
        || theSpace == OcctReferenceSpace::World;
}

bool IsFiniteBoundedReferencePoint(const gp_Pnt& thePoint) noexcept
{
    return IsFiniteBoundedMeshCoordinate(thePoint.X())
        && IsFiniteBoundedMeshCoordinate(thePoint.Y())
        && IsFiniteBoundedMeshCoordinate(thePoint.Z());
}

bool IsFiniteReferenceDirection(const gp_Dir& theDirection) noexcept
{
    const Standard_Real aSquaredLength =
        theDirection.X() * theDirection.X()
        + theDirection.Y() * theDirection.Y()
        + theDirection.Z() * theDirection.Z();
    return std::isfinite(theDirection.X())
        && std::isfinite(theDirection.Y())
        && std::isfinite(theDirection.Z())
        && std::isfinite(aSquaredLength)
        && std::abs(aSquaredLength - 1.0)
            <= kReferenceAxisUnitTolerance;
}

Standard_Real CanonicalReferenceScalar(const Standard_Real theValue) noexcept
{
    return theValue == 0.0 ? 0.0 : theValue;
}

bool ReferenceAxesMatch(
    const OcctReferenceAxis& theLeft,
    const OcctReferenceAxis& theRight,
    const Standard_Real theTolerance = 1.0e-12) noexcept
{
    return theLeft.pivotSpace == theRight.pivotSpace
        && theLeft.directionSpace == theRight.directionSpace
        && theLeft.pivot.IsEqual(theRight.pivot, theTolerance)
        && theLeft.direction.IsEqual(theRight.direction, theTolerance);
}

Standard_Integer EncodedReferenceAxisMode(
    const OcctReferenceAxis& theAxis) noexcept
{
    return kReferenceAxisSchemaV1
        | (theAxis.pivotSpace == OcctReferenceSpace::World
            ? kReferenceAxisPivotWorldBit : 0)
        | (theAxis.directionSpace == OcctReferenceSpace::World
            ? kReferenceAxisDirectionWorldBit : 0);
}

OcctReferenceAxisReadState ReadReferenceAxisRecord(
    const TDF_Label& theLabel,
    OcctReferenceAxis& theAxis)
{
    theAxis = DefaultReferenceAxis();
    if (theLabel.IsNull()) {
        return OcctReferenceAxisReadState::Invalid;
    }

    const auto& anIds = ReferenceAxisAttributeIDs();
    std::array<Handle(TDF_Attribute), 7> anAttributes;
    Standard_Size aPresentCount = 0;
    for (std::size_t anIndex = 0; anIndex < anIds.size(); ++anIndex) {
        if (theLabel.FindAttribute(*anIds[anIndex], anAttributes[anIndex])) {
            ++aPresentCount;
        }
    }
    if (aPresentCount == 0U) {
        return OcctReferenceAxisReadState::ImplicitDefault;
    }
    if (aPresentCount != anIds.size()) {
        return OcctReferenceAxisReadState::Invalid;
    }

    const Handle(TDataStd_Integer) aMode =
        Handle(TDataStd_Integer)::DownCast(anAttributes[0]);
    if (aMode.IsNull()) {
        return OcctReferenceAxisReadState::Invalid;
    }
    const Standard_Integer aModeValue = aMode->Get();
    if ((aModeValue & ~0x0003) != kReferenceAxisSchemaV1) {
        return OcctReferenceAxisReadState::Invalid;
    }

    Standard_Real aValues[6] = {};
    for (std::size_t anIndex = 0; anIndex < 6U; ++anIndex) {
        const Handle(TDataStd_Real) aValue =
            Handle(TDataStd_Real)::DownCast(anAttributes[anIndex + 1U]);
        if (aValue.IsNull() || !std::isfinite(aValue->Get())) {
            return OcctReferenceAxisReadState::Invalid;
        }
        aValues[anIndex] = aValue->Get();
    }

    const gp_Pnt aPivot(aValues[0], aValues[1], aValues[2]);
    if (!IsFiniteBoundedReferencePoint(aPivot)) {
        return OcctReferenceAxisReadState::Invalid;
    }
    const Standard_Real aDirectionSquaredLength =
        aValues[3] * aValues[3]
        + aValues[4] * aValues[4]
        + aValues[5] * aValues[5];
    if (!std::isfinite(aDirectionSquaredLength)
        || std::abs(aDirectionSquaredLength - 1.0)
            > kReferenceAxisUnitTolerance) {
        return OcctReferenceAxisReadState::Invalid;
    }

    try {
        OCC_CATCH_SIGNALS
        theAxis.pivotSpace =
            (aModeValue & kReferenceAxisPivotWorldBit) != 0
            ? OcctReferenceSpace::World : OcctReferenceSpace::Object;
        theAxis.pivot = aPivot;
        theAxis.directionSpace =
            (aModeValue & kReferenceAxisDirectionWorldBit) != 0
            ? OcctReferenceSpace::World : OcctReferenceSpace::Object;
        theAxis.direction = gp_Dir(aValues[3], aValues[4], aValues[5]);
        return IsFiniteReferenceDirection(theAxis.direction)
            ? OcctReferenceAxisReadState::Authored
            : OcctReferenceAxisReadState::Invalid;
    } catch (...) {
        theAxis = DefaultReferenceAxis();
        return OcctReferenceAxisReadState::Invalid;
    }
}

bool WriteReferenceAxisRecord(
    const TDF_Label& theLabel,
    const OcctReferenceAxis& theAxis)
{
    if (theLabel.IsNull()
        || !IsReferenceSpace(theAxis.pivotSpace)
        || !IsReferenceSpace(theAxis.directionSpace)
        || !IsFiniteBoundedReferencePoint(theAxis.pivot)
        || !IsFiniteReferenceDirection(theAxis.direction)) {
        return false;
    }
    try {
        OCC_CATCH_SIGNALS
        TDataStd_Integer::Set(
            theLabel,
            ReferenceAxisModeAttributeID(),
            EncodedReferenceAxisMode(theAxis));
        TDataStd_Real::Set(
            theLabel,
            ReferenceAxisPivotXAttributeID(),
            CanonicalReferenceScalar(theAxis.pivot.X()));
        TDataStd_Real::Set(
            theLabel,
            ReferenceAxisPivotYAttributeID(),
            CanonicalReferenceScalar(theAxis.pivot.Y()));
        TDataStd_Real::Set(
            theLabel,
            ReferenceAxisPivotZAttributeID(),
            CanonicalReferenceScalar(theAxis.pivot.Z()));
        TDataStd_Real::Set(
            theLabel,
            ReferenceAxisDirectionXAttributeID(),
            CanonicalReferenceScalar(theAxis.direction.X()));
        TDataStd_Real::Set(
            theLabel,
            ReferenceAxisDirectionYAttributeID(),
            CanonicalReferenceScalar(theAxis.direction.Y()));
        TDataStd_Real::Set(
            theLabel,
            ReferenceAxisDirectionZAttributeID(),
            CanonicalReferenceScalar(theAxis.direction.Z()));

        OcctReferenceAxis aStored;
        return ReadReferenceAxisRecord(theLabel, aStored)
                == OcctReferenceAxisReadState::Authored
            && ReferenceAxesMatch(theAxis, aStored);
    } catch (...) {
        return false;
    }
}

bool HasAnyReferenceAxisAttribute(const TDF_Label& theLabel)
{
    if (theLabel.IsNull()) {
        return false;
    }
    for (const Standard_GUID* anId : ReferenceAxisAttributeIDs()) {
        if (anId != nullptr && theLabel.IsAttribute(*anId)) {
            return true;
        }
    }
    return false;
}

bool TryReadObjectTransform(
    const Handle(TDocStd_Document)& theDocument,
    const Handle(XCAFDoc_ShapeTool)& theShapeTool,
    const TDF_Label& theLabel,
    gp_Trsf& theTransform)
{
    if (!IsGeometryDefinitionLabel(theDocument, theShapeTool, theLabel)) {
        return false;
    }

    const Standard_Real aDefaults[8] = {
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
        1.0,
    };
    Standard_Real aValues[8] = {};
    for (Standard_Integer anIndex = 0; anIndex < 8; ++anIndex) {
        aValues[anIndex] = aDefaults[anIndex];
        const TDF_Label aChild =
            theLabel.FindChild(anIndex + 1, Standard_False);
        if (!aChild.IsNull()) {
            Handle(TDataStd_Real) anAttribute;
            if (aChild.FindAttribute(TDataStd_Real::GetID(), anAttribute)) {
                if (anAttribute.IsNull()) {
                    return false;
                }
                aValues[anIndex] = anAttribute->Get();
            }
        }
        if (!std::isfinite(aValues[anIndex])) {
            return false;
        }
    }
    for (Standard_Integer anAxis = 0; anAxis < 3; ++anAxis) {
        if (std::abs(aValues[anAxis])
            > core3d::limits::kMaximumModelCoordinateMagnitude) {
            return false;
        }
    }
    if (std::abs(aValues[7])
        <= std::numeric_limits<Standard_Real>::epsilon()) {
        return false;
    }

    const Standard_Real aMaximumQuaternionComponent = std::max({
        std::abs(aValues[3]), std::abs(aValues[4]),
        std::abs(aValues[5]), std::abs(aValues[6]),
    });
    if (!std::isfinite(aMaximumQuaternionComponent)
        || aMaximumQuaternionComponent
            <= std::numeric_limits<Standard_Real>::min()) {
        return false;
    }
    Standard_Real aQuaternion[4] = {
        aValues[3] / aMaximumQuaternionComponent,
        aValues[4] / aMaximumQuaternionComponent,
        aValues[5] / aMaximumQuaternionComponent,
        aValues[6] / aMaximumQuaternionComponent,
    };
    const Standard_Real aQuaternionNorm = std::sqrt(
        aQuaternion[0] * aQuaternion[0]
        + aQuaternion[1] * aQuaternion[1]
        + aQuaternion[2] * aQuaternion[2]
        + aQuaternion[3] * aQuaternion[3]);
    if (!std::isfinite(aQuaternionNorm)
        || aQuaternionNorm
            <= std::numeric_limits<Standard_Real>::epsilon()) {
        return false;
    }
    for (Standard_Real& aComponent : aQuaternion) {
        aComponent /= aQuaternionNorm;
    }

    try {
        OCC_CATCH_SIGNALS
        gp_Trsf aTransform;
        aTransform.SetRotationPart(gp_Quaternion(
            aQuaternion[0], aQuaternion[1],
            aQuaternion[2], aQuaternion[3]));
        aTransform.SetScaleFactor(aValues[7]);
        aTransform.SetTranslationPart(
            gp_XYZ(aValues[0], aValues[1], aValues[2]));
        for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
            for (Standard_Integer aColumn = 1; aColumn <= 4; ++aColumn) {
                if (!std::isfinite(aTransform.Value(aRow, aColumn))) {
                    return false;
                }
            }
        }
        theTransform = aTransform;
        return true;
    } catch (...) {
        return false;
    }
}

bool TryResolveReferenceAxis(
    const Handle(TDocStd_Document)& theDocument,
    const Handle(XCAFDoc_ShapeTool)& theShapeTool,
    const TDF_Label& theLabel,
    const TopLoc_Location& theOccurrenceLocation,
    gp_Ax1& theWorldAxis)
{
    OcctReferenceAxis aReference;
    if (ReadReferenceAxisRecord(theLabel, aReference)
            == OcctReferenceAxisReadState::Invalid
        || !IsGeometryDefinitionLabel(
            theDocument, theShapeTool, theLabel)) {
        return false;
    }

    gp_Trsf anObjectTransform;
    if (!TryReadObjectTransform(
            theDocument, theShapeTool, theLabel, anObjectTransform)) {
        return false;
    }
    const gp_Trsf anOccurrenceTransform =
        theOccurrenceLocation.Transformation();
    for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
        for (Standard_Integer aColumn = 1; aColumn <= 4; ++aColumn) {
            if (!std::isfinite(
                    anOccurrenceTransform.Value(aRow, aColumn))) {
                return false;
            }
        }
    }
    const gp_Trsf anObjectToWorld =
        anObjectTransform.Multiplied(anOccurrenceTransform);

    gp_Pnt aWorldPivot = aReference.pivot;
    if (aReference.pivotSpace == OcctReferenceSpace::Object) {
        aWorldPivot.Transform(anObjectToWorld);
    }
    if (!IsFiniteBoundedReferencePoint(aWorldPivot)) {
        return false;
    }

    gp_Vec aWorldDirection(aReference.direction);
    if (aReference.directionSpace == OcctReferenceSpace::Object) {
        aWorldDirection.Transform(anObjectToWorld);
    }
    const Standard_Real aSquaredMagnitude = aWorldDirection.SquareMagnitude();
    if (!std::isfinite(aWorldDirection.X())
        || !std::isfinite(aWorldDirection.Y())
        || !std::isfinite(aWorldDirection.Z())
        || !std::isfinite(aSquaredMagnitude)
        || aSquaredMagnitude
            <= std::numeric_limits<Standard_Real>::epsilon()) {
        return false;
    }
    try {
        OCC_CATCH_SIGNALS
        const gp_Dir aWorldDirectionUnit(aWorldDirection);
        if (!IsFiniteReferenceDirection(aWorldDirectionUnit)) {
            return false;
        }
        theWorldAxis = gp_Ax1(aWorldPivot, aWorldDirectionUnit);
        return true;
    } catch (...) {
        return false;
    }
}

Standard_Boolean ValidateReferenceAxisDocument(
    const Handle(TDocStd_Document)& theDocument)
{
    try {
        OCC_CATCH_SIGNALS
        if (theDocument.IsNull() || theDocument->GetData().IsNull()) {
            return Standard_False;
        }
        const TDF_Label aRoot = theDocument->GetData()->Root();
        if (HasAnyReferenceAxisAttribute(aRoot)) {
            return Standard_False;
        }

        const bool hasShapeTool =
            XCAFDoc_DocumentTool::CheckShapeTool(theDocument->Main());
        const Handle(XCAFDoc_ShapeTool) aShapeTool = hasShapeTool
            ? XCAFDoc_DocumentTool::ShapeTool(theDocument->Main())
            : Handle(XCAFDoc_ShapeTool)();
        if (hasShapeTool && aShapeTool.IsNull()) {
            return Standard_False;
        }

        Standard_Size aLabelCount = 0;
        for (TDF_ChildIterator aLabel(aRoot, Standard_True);
             aLabel.More(); aLabel.Next()) {
            if (++aLabelCount > kMaximumGeometryDocumentLabels) {
                return Standard_False;
            }
            const TDF_Label& aValue = aLabel.Value();
            const bool hasReferenceAxis =
                HasAnyReferenceAxisAttribute(aValue);
            const bool isGeometryDefinition = !aShapeTool.IsNull()
                && IsGeometryDefinitionLabel(
                    theDocument, aShapeTool, aValue);
            if (hasReferenceAxis) {
                if (!isGeometryDefinition
                    || !XCAFDoc_ShapeTool::IsFree(aValue)) {
                    return Standard_False;
                }
                OcctReferenceAxis anAxis;
                if (ReadReferenceAxisRecord(aValue, anAxis)
                        != OcctReferenceAxisReadState::Authored) {
                    return Standard_False;
                }
            }
            if (isGeometryDefinition) {
                // The implicit Object-Origin / World-Z axis is authority too.
                // Resolve every definition, not only definitions carrying an
                // authored record, so a malformed persisted object transform
                // cannot pass open/save admission and fail later publication.
                gp_Ax1 aResolved;
                if (!TryResolveReferenceAxis(
                        theDocument,
                        aShapeTool,
                        aValue,
                        TopLoc_Location(),
                        aResolved)) {
                    return Standard_False;
                }
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

bool ReadGeometryRepresentation(
    const TDF_Label& theLabel,
    bool& theHasMarker,
    OcctGeometryRepresentation& theRepresentation)
{
    theHasMarker = false;
    theRepresentation = OcctGeometryRepresentation::LegacyUnknown;
    if (theLabel.IsNull()) {
        theRepresentation = OcctGeometryRepresentation::Invalid;
        return false;
    }
    Handle(TDataStd_Integer) anAttribute;
    if (!theLabel.FindAttribute(
            GeometryRepresentationAttributeID(), anAttribute)) {
        return true;
    }
    theHasMarker = true;
    if (anAttribute.IsNull()) {
        theRepresentation = OcctGeometryRepresentation::Invalid;
        return false;
    }
    switch (anAttribute->Get()) {
        case static_cast<Standard_Integer>(
            OcctGeometryRepresentation::LegacyUnknown):
            theRepresentation =
                OcctGeometryRepresentation::LegacyUnknown;
            return true;
        case static_cast<Standard_Integer>(
            OcctGeometryRepresentation::BRep):
            theRepresentation = OcctGeometryRepresentation::BRep;
            return true;
        case static_cast<Standard_Integer>(
            OcctGeometryRepresentation::TriangleMesh):
            theRepresentation =
                OcctGeometryRepresentation::TriangleMesh;
            return true;
        default:
            theRepresentation = OcctGeometryRepresentation::Invalid;
            return false;
    }
}

bool WriteGeometryRepresentationMarker(
    const TDF_Label& theLabel,
    const OcctGeometryRepresentation theRepresentation)
{
    if (theLabel.IsNull()
        || theRepresentation == OcctGeometryRepresentation::Invalid) {
        return false;
    }
    TDataStd_Integer::Set(
        theLabel,
        GeometryRepresentationAttributeID(),
        static_cast<Standard_Integer>(theRepresentation));
    Handle(TDataStd_Integer) aStoredRepresentation;
    return theLabel.FindAttribute(
               GeometryRepresentationAttributeID(),
               aStoredRepresentation)
        && !aStoredRepresentation.IsNull()
        && aStoredRepresentation->Get()
            == static_cast<Standard_Integer>(theRepresentation);
}

bool IsValidMeshFace(
    const TopoDS_Face& theFace,
    Standard_Size& theDefinitionVertices,
    Standard_Size& theDefinitionIndices)
{
    if (theFace.IsNull()
        || (theFace.Orientation() != TopAbs_FORWARD
            && theFace.Orientation() != TopAbs_REVERSED)) {
        return false;
    }
    TopLoc_Location aLocation;
    const Handle(Poly_Triangulation)& aTriangulation =
        BRep_Tool::Triangulation(theFace, aLocation);
    if (aTriangulation.IsNull()
        || aTriangulation->HasDeferredData()
        || !aTriangulation->HasGeometry()
        || aTriangulation->NbNodes() <= 0
        || aTriangulation->NbTriangles() <= 0) {
        return false;
    }

    const Standard_Size aNodeCount =
        static_cast<Standard_Size>(aTriangulation->NbNodes());
    const Standard_Size aTriangleCount =
        static_cast<Standard_Size>(aTriangulation->NbTriangles());
    if (aTriangleCount
            > kMaximumMeshIndicesPerDefinition / 3U
        || !AddWithinLimit(
            theDefinitionVertices,
            aNodeCount,
            kMaximumMeshVerticesPerDefinition)
        || !AddWithinLimit(
            theDefinitionIndices,
            aTriangleCount * 3U,
            kMaximumMeshIndicesPerDefinition)) {
        return false;
    }

    const gp_Trsf aTransform = aLocation.Transformation();
    for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
        for (Standard_Integer aColumn = 1; aColumn <= 4; ++aColumn) {
            if (!std::isfinite(aTransform.Value(aRow, aColumn))) {
                return false;
            }
        }
    }
    for (Standard_Integer aNode = 1;
         aNode <= aTriangulation->NbNodes(); ++aNode) {
        const gp_Pnt& aStoredPoint = aTriangulation->Node(aNode);
        if (!IsFiniteBoundedMeshCoordinate(aStoredPoint.X())
            || !IsFiniteBoundedMeshCoordinate(aStoredPoint.Y())
            || !IsFiniteBoundedMeshCoordinate(aStoredPoint.Z())) {
            return false;
        }
        gp_Pnt aPoint = aStoredPoint;
        aPoint.Transform(aTransform);
        if (!IsFiniteBoundedMeshCoordinate(aPoint.X())
            || !IsFiniteBoundedMeshCoordinate(aPoint.Y())
            || !IsFiniteBoundedMeshCoordinate(aPoint.Z())) {
            return false;
        }
        if (aTriangulation->HasNormals()) {
            const gp_Dir aNormal = aTriangulation->Normal(aNode);
            const Standard_Real aSquaredLength =
                aNormal.X() * aNormal.X()
                + aNormal.Y() * aNormal.Y()
                + aNormal.Z() * aNormal.Z();
            if (!std::isfinite(aNormal.X())
                || !std::isfinite(aNormal.Y())
                || !std::isfinite(aNormal.Z())
                || !std::isfinite(aSquaredLength)
                || std::abs(aSquaredLength - 1.0) > 1.0e-3) {
                return false;
            }
        }
        if (aTriangulation->HasUVNodes()) {
            const gp_Pnt2d aUV = aTriangulation->UVNode(aNode);
            if (!std::isfinite(aUV.X()) || !std::isfinite(aUV.Y())) {
                return false;
            }
        }
    }

    for (Standard_Integer aTriangle = 1;
         aTriangle <= aTriangulation->NbTriangles(); ++aTriangle) {
        Standard_Integer aNodes[3] = {0, 0, 0};
        aTriangulation->Triangle(aTriangle).Get(
            aNodes[0], aNodes[1], aNodes[2]);
        for (const Standard_Integer aNode : aNodes) {
            if (aNode < 1 || aNode > aTriangulation->NbNodes()) {
                return false;
            }
        }
        if (aNodes[0] == aNodes[1]
            || aNodes[1] == aNodes[2]
            || aNodes[2] == aNodes[0]) {
            return false;
        }
        gp_Pnt aP0 = aTriangulation->Node(aNodes[0]);
        gp_Pnt aP1 = aTriangulation->Node(aNodes[1]);
        gp_Pnt aP2 = aTriangulation->Node(aNodes[2]);
        aP0.Transform(aTransform);
        aP1.Transform(aTransform);
        aP2.Transform(aTransform);
        const Standard_Real aUX = aP1.X() - aP0.X();
        const Standard_Real aUY = aP1.Y() - aP0.Y();
        const Standard_Real aUZ = aP1.Z() - aP0.Z();
        const Standard_Real aVX = aP2.X() - aP0.X();
        const Standard_Real aVY = aP2.Y() - aP0.Y();
        const Standard_Real aVZ = aP2.Z() - aP0.Z();
        const Standard_Real aCrossX = aUY * aVZ - aUZ * aVY;
        const Standard_Real aCrossY = aUZ * aVX - aUX * aVZ;
        const Standard_Real aCrossZ = aUX * aVY - aUY * aVX;
        const Standard_Real aSquaredArea =
            aCrossX * aCrossX
            + aCrossY * aCrossY
            + aCrossZ * aCrossZ;
        if (!std::isfinite(aSquaredArea) || aSquaredArea <= 0.0) {
            return false;
        }
    }
    return true;
}

bool IsValidGeometryTopologyOrientation(
    const TopAbs_Orientation theOrientation) noexcept
{
    switch (theOrientation) {
        case TopAbs_FORWARD:
        case TopAbs_REVERSED:
        case TopAbs_INTERNAL:
        case TopAbs_EXTERNAL:
            return true;
    }
    return false;
}

bool IsAllowedGeometryTopologyChild(
    const TopAbs_ShapeEnum theParent,
    const TopAbs_ShapeEnum theChild) noexcept
{
    switch (theParent) {
        case TopAbs_COMPOUND:
            return theChild >= TopAbs_COMPOUND
                && theChild < TopAbs_SHAPE;
        case TopAbs_COMPSOLID:
            return theChild == TopAbs_SOLID;
        case TopAbs_SOLID:
            return theChild == TopAbs_SHELL;
        case TopAbs_SHELL:
            return theChild == TopAbs_FACE;
        case TopAbs_FACE:
            return theChild == TopAbs_WIRE;
        case TopAbs_WIRE:
            return theChild == TopAbs_EDGE;
        case TopAbs_EDGE:
            return theChild == TopAbs_VERTEX;
        case TopAbs_VERTEX:
        case TopAbs_SHAPE:
            return false;
    }
    return false;
}

DefinitionGeometryClass ClassifyDefinitionGeometry(
    const TopoDS_Shape& theShape,
    GeometryValidationBudget* theBudget)
{
    if (theShape.IsNull()) {
        return DefinitionGeometryClass::Invalid;
    }
    try {
        OCC_CATCH_SIGNALS
        struct TopologyFrame {
            TopoDS_Shape shape;
            Standard_Size depth = 0;
            bool leaving = false;
        };
        std::vector<TopologyFrame> aStack = {{theShape, 0, false}};
        TopTools_MapOfShape aVisited;
        TopTools_MapOfShape anActivePath;
        Standard_Size aSubshapeCount = 0;
        Standard_Size aVertexCount = 0;
        Standard_Size anIndexCount = 0;
        bool hasBRepFace = false;
        bool hasMeshFace = false;

        while (!aStack.empty()) {
            const TopologyFrame aFrame = aStack.back();
            aStack.pop_back();
            if (aFrame.shape.IsNull()) {
                return DefinitionGeometryClass::Invalid;
            }
            if (aFrame.leaving) {
                anActivePath.Remove(aFrame.shape);
                aVisited.Add(aFrame.shape);
                continue;
            }
            if (aVisited.Contains(aFrame.shape)) {
                continue;
            }
            if (anActivePath.Contains(aFrame.shape)
                || aFrame.shape.ShapeType() == TopAbs_SHAPE
                || !IsValidGeometryTopologyOrientation(
                    aFrame.shape.Orientation())
                || aFrame.depth > kMaximumTopologyDepth
                || aSubshapeCount
                    >= kMaximumSubshapesPerDefinition) {
                return DefinitionGeometryClass::Invalid;
            }
            anActivePath.Add(aFrame.shape);
            ++aSubshapeCount;

            if (aFrame.shape.ShapeType() == TopAbs_FACE) {
                const TopoDS_Face aFace =
                    TopoDS::Face(aFrame.shape);
                if (!BRep_Tool::Surface(aFace).IsNull()) {
                    hasBRepFace = true;
                    if (hasMeshFace) {
                        return DefinitionGeometryClass::Invalid;
                    }
                } else {
                    hasMeshFace = true;
                    if (hasBRepFace
                        || !IsValidMeshFace(
                            aFace, aVertexCount, anIndexCount)) {
                        return DefinitionGeometryClass::Invalid;
                    }
                }
            }

            if (aStack.size()
                >= static_cast<std::size_t>(
                    kMaximumSubshapesPerDefinition) * 2U) {
                return DefinitionGeometryClass::Invalid;
            }
            aStack.push_back({
                aFrame.shape, aFrame.depth, true});
            const TopAbs_ShapeEnum aParentType =
                aFrame.shape.ShapeType();
            for (TopoDS_Iterator aChild(
                     aFrame.shape, Standard_True, Standard_True);
                 aChild.More(); aChild.Next()) {
                const TopoDS_Shape& aChildShape = aChild.Value();
                if (aChildShape.IsNull()
                    || !IsAllowedGeometryTopologyChild(
                        aParentType, aChildShape.ShapeType())
                    || !IsValidGeometryTopologyOrientation(
                        aChildShape.Orientation())
                    || aFrame.depth >= kMaximumTopologyDepth
                    || aStack.size()
                        >= static_cast<std::size_t>(
                            kMaximumSubshapesPerDefinition) * 2U) {
                    return DefinitionGeometryClass::Invalid;
                }
                aStack.push_back({
                    aChildShape, aFrame.depth + 1U, false});
            }
        }

        if (theBudget != nullptr
            && (!AddWithinLimit(
                    theBudget->subshapes,
                    aSubshapeCount,
                    kMaximumSubshapesPerDocument)
                || (hasMeshFace
                    && (!AddWithinLimit(
                            theBudget->meshVertices,
                            aVertexCount,
                            kMaximumMeshVerticesPerDocument)
                        || !AddWithinLimit(
                            theBudget->meshIndices,
                            anIndexCount,
                            kMaximumMeshIndicesPerDocument))))) {
            return DefinitionGeometryClass::Invalid;
        }
        if (hasMeshFace) {
            return DefinitionGeometryClass::TriangleMesh;
        }
        // Legacy definitions made solely from edges, wires, vertices, or an
        // empty compound remain BRep-compatible. Every existing face has
        // proved to own a geometric surface in the bounded walk above.
        return DefinitionGeometryClass::BRep;
    } catch (...) {
        return DefinitionGeometryClass::Invalid;
    }
}

bool GeometryClassMatchesRepresentation(
    const DefinitionGeometryClass theGeometryClass,
    const OcctGeometryRepresentation theRepresentation) noexcept
{
    switch (theRepresentation) {
        case OcctGeometryRepresentation::LegacyUnknown:
        case OcctGeometryRepresentation::BRep:
            return theGeometryClass == DefinitionGeometryClass::BRep;
        case OcctGeometryRepresentation::TriangleMesh:
            return theGeometryClass
                == DefinitionGeometryClass::TriangleMesh;
        case OcctGeometryRepresentation::Invalid:
            return false;
    }
    return false;
}

OcctGeometryRepresentation ValidatedGeometryRepresentation(
    const Handle(TDocStd_Document)& theDocument,
    const Handle(XCAFDoc_ShapeTool)& theShapeTool,
    const TDF_Label& theLabel,
    GeometryValidationBudget* theBudget = nullptr)
{
    if (!IsGeometryDefinitionLabel(
            theDocument, theShapeTool, theLabel)) {
        return OcctGeometryRepresentation::Invalid;
    }
    bool hasMarker = false;
    OcctGeometryRepresentation aRepresentation =
        OcctGeometryRepresentation::Invalid;
    if (!ReadGeometryRepresentation(
            theLabel, hasMarker, aRepresentation)) {
        return OcctGeometryRepresentation::Invalid;
    }
    (void)hasMarker;
    const DefinitionGeometryClass aGeometryClass =
        ClassifyDefinitionGeometry(
            XCAFDoc_ShapeTool::GetShape(theLabel), theBudget);
    return GeometryClassMatchesRepresentation(
               aGeometryClass, aRepresentation)
        ? aRepresentation
        : OcctGeometryRepresentation::Invalid;
}

//! Marks XCAF visualization material assignments authored by Shapeyard's PBR
//! editor. Imported XCAF styles remain untouched and legacy child-11/12 style
//! overrides retain their historical precedence until the user authors PBR.
const Standard_GUID& LocalPBRMaterialAttributeID()
{
    static const Standard_GUID anId("248A5203-4A22-4F2B-85C4-BE0BA89A5E4D");
    return anId;
}

//! Per-object recipe, independent of deduplicated image/material definitions.
//! Version1 fixes the pinned Mikk generator and owned canonical UV/normal input.
const Standard_GUID& NormalTextureRecipeAttributeID()
{
    static const Standard_GUID anId("98EAD304-EB49-4F0E-ABFC-AEF94C250161");
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

// Persistent saved-group schema v1. Each string uses an existing bounded
// BinXCAF driver; never serialize the catalog into one large ASCII payload.
const Standard_GUID& SavedGroupContainerID() {
    static const Standard_GUID id("EC7B5F15-218F-47E4-BF6A-61BF42861401"); return id;
}
const Standard_GUID& SavedGroupRecordID() {
    static const Standard_GUID id("EC7B5F15-218F-47E4-BF6A-61BF42861402"); return id;
}
const Standard_GUID& SavedGroupNameID() {
    static const Standard_GUID id("EC7B5F15-218F-47E4-BF6A-61BF42861403"); return id;
}
const Standard_GUID& SavedGroupMembershipID() {
    static const Standard_GUID id("EC7B5F15-218F-47E4-BF6A-61BF42861404"); return id;
}
constexpr std::size_t kMaximumSavedGroups = 128;
constexpr std::size_t kMaximumSavedGroupMembers = 32;
bool IsCanonicalSavedGroupID(const std::string& id) {
    if (id.size() != 36 || !Standard_GUID::CheckGUIDFormat(id.c_str())) { return false; }
    for (char c : id) {
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || c == '-')) { return false; }
    }
    return true;
}
bool HasSavedGroupAttribute(const TDF_Label& label) {
    Handle(TDF_Attribute) attribute;
    return label.FindAttribute(SavedGroupContainerID(), attribute)
        || label.FindAttribute(SavedGroupRecordID(), attribute)
        || label.FindAttribute(SavedGroupNameID(), attribute)
        || label.FindAttribute(SavedGroupMembershipID(), attribute);
}
bool ReadSavedGroups(const Handle(TDocStd_Document)& document, OcctSavedGroupState& output) noexcept {
    output = OcctSavedGroupState();
    try {
        if (document.IsNull() || document->GetData().IsNull()) { return false; }
        OcctSavedGroupState state;
        state.documentData = document->GetData();
        const TDF_Label root = state.documentData->Root();
        if (HasSavedGroupAttribute(root)) { return false; }
        std::vector<TDF_Label> records, members;
        std::size_t count = 0;
        // First collect every schema-bearing label, including wrongly placed
        // attributes. Downcasting an unexpected type must fail admission.
        for (TDF_ChildIterator it(root, Standard_True); it.More(); it.Next()) {
            if (++count > kMaximumGeometryDocumentLabels) { return false; }
            const TDF_Label label = it.Value();
            Handle(TDF_Attribute) marker, record, name, member;
            const bool hasMarker = label.FindAttribute(SavedGroupContainerID(), marker);
            const bool hasRecord = label.FindAttribute(SavedGroupRecordID(), record);
            const bool hasName = label.FindAttribute(SavedGroupNameID(), name);
            const bool hasMember = label.FindAttribute(SavedGroupMembershipID(), member);
            if (hasMarker) {
                const auto typed = Handle(TDataStd_Integer)::DownCast(marker);
                if (typed.IsNull() || typed->Get() != 1 || !label.Father().IsEqual(root)
                    || label.IsEqual(document->Main()) || !state.container.IsNull()
                    || hasRecord || hasName || hasMember) { return false; }
                state.container = label;
            }
            if (hasRecord || hasName) {
                if (!hasRecord || !hasName || hasMember
                    || Handle(TDataStd_AsciiString)::DownCast(record).IsNull()
                    || Handle(TDataStd_Name)::DownCast(name).IsNull()
                    || records.size() >= kMaximumSavedGroups) { return false; }
                records.push_back(label);
            }
            if (hasMember) {
                if (Handle(TDataStd_AsciiString)::DownCast(member).IsNull()
                    || members.size() >= kMaximumSavedGroups * kMaximumSavedGroupMembers) { return false; }
                members.push_back(label);
            }
        }
        if (state.container.IsNull() && (!records.empty() || !members.empty())) { return false; }
        std::unordered_map<std::string, std::size_t> indices;
        for (const auto& label : records) {
            if (!label.Father().IsEqual(state.container)) { return false; }
            OcctSavedGroup group;
            group.recordLabel = label;
            group.identifier = ReadIdentifier(label, SavedGroupRecordID());
            Handle(TDataStd_Name) name;
            if (!IsCanonicalSavedGroupID(group.identifier)
                || !label.FindAttribute(SavedGroupNameID(), name)
                || !OcctObjectNameIsValid(name->Get())
                || !indices.emplace(group.identifier, state.groups.size()).second) { return false; }
            group.name = name->Get();
            state.groups.push_back(std::move(group));
        }
        const auto shapes = XCAFDoc_DocumentTool::CheckShapeTool(document->Main())
            ? XCAFDoc_DocumentTool::ShapeTool(document->Main()) : Handle(XCAFDoc_ShapeTool)();
        for (const auto& label : members) {
            const auto id = ReadIdentifier(label, SavedGroupMembershipID());
            const auto found = indices.find(id);
            if (!IsCanonicalSavedGroupID(id) || found == indices.end() || shapes.IsNull()
                || !IsGeometryDefinitionLabel(document, shapes, label)
                || !XCAFDoc_ShapeTool::IsFree(label)
                || ReadIdentifier(label, EntityIdentifierAttributeID()).empty()
                || ReadIdentifier(label, DefinitionIdentifierAttributeID()).empty()) { return false; }
            auto& group = state.groups[found->second];
            if (group.members.size() >= kMaximumSavedGroupMembers) { return false; }
            group.members.push_back(label);
        }
        output = std::move(state);
        return true;
    } catch (...) { output = OcctSavedGroupState(); return false; }
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

void Core3DPrepareRendererTextures(
    const Handle(Graphic3d_AspectFillArea3d)& aspect)
{
    if (aspect.IsNull()) return;
    const bool ownsShader = IsCore3DDataMapShader(aspect->ShaderProgram());
    if (aspect->TextureSet().IsNull() || aspect->TextureSet()->IsEmpty()) {
        if (ownsShader) aspect->SetShaderProgram({});
        return;
    }
    const Handle(Graphic3d_TextureSet)& original = aspect->TextureSet();
    if (original->IsEmpty()) return;
    Handle(Graphic3d_TextureSet) replacement = new Graphic3d_TextureSet(original->Size());
    bool changed = false;
    for (Standard_Integer i = 0; i < original->Size(); ++i) {
        const Handle(Graphic3d_TextureMap)& current = original->Value(i);
        replacement->SetValue(i, current);
        const Handle(XCAFPrs_Texture) texture = Handle(XCAFPrs_Texture)::DownCast(current);
        if (texture.IsNull() || !Handle(Core3DRoleAwareTexture)::DownCast(texture).IsNull()
            || texture->GetParams().IsNull()) continue;
        Handle(Core3DRoleAwareTexture) prepared = new Core3DRoleAwareTexture(texture);
        replacement->SetValue(i, prepared);
        changed = true;
    }
    if (changed) aspect->SetTextureSet(replacement);
    Standard_Integer bits = 0;
    for (Standard_Integer i = 0; i < replacement->Size(); ++i) {
        const auto& texture = replacement->Value(i);
        if (texture.IsNull() || texture->GetParams().IsNull()) continue;
        const auto unit = texture->GetParams()->TextureUnit();
        if (unit >= Graphic3d_TextureUnit_BaseColor && unit <= Graphic3d_TextureUnit_MetallicRoughness) {
            bits |= (1 << static_cast<int>(unit));
        }
    }
    const bool hasDataMaps = (bits & (Graphic3d_TextureSetBits_MetallicRoughness | Graphic3d_TextureSetBits_Occlusion | Graphic3d_TextureSetBits_Normal)) != 0;
    if (hasDataMaps && (ownsShader || aspect->ShaderProgram().IsNull())) {
        const auto expectedID = TCollection_AsciiString((bits & Graphic3d_TextureSetBits_Normal)
            ? "shapeyard-data-maps-v1-normal-" : "shapeyard-data-maps-v1-") + bits;
        if (!ownsShader || aspect->ShaderProgram()->GetId() != expectedID) {
            aspect->SetShaderProgram(MakeCore3DDataMapShader(bits));
        }
    } else if (ownsShader) {
        aspect->SetShaderProgram({});
    }
}

Handle(Image_Texture)& Core3DMaterialTexture(XCAFDoc_VisMaterialPBR& material,
                                            OcctMaterialTextureSlot slot)
{
    switch (slot) {
        case OcctMaterialTextureSlot::BaseColor: return material.BaseColorTexture;
        case OcctMaterialTextureSlot::Emissive: return material.EmissiveTexture;
        case OcctMaterialTextureSlot::MetallicRoughness: return material.MetallicRoughnessTexture;
        case OcctMaterialTextureSlot::Occlusion: return material.OcclusionTexture;
        case OcctMaterialTextureSlot::Normal: return material.NormalTexture;
    }
    throw Standard_Failure("Invalid material texture slot");
}

Standard_Boolean Core3DValidateNumericTexture(const Handle(Image_Texture)& texture)
{
    if (texture.IsNull() || !Core3DValidateAuthoredTexture(texture)) return Standard_False;
    return !DecodeNumericRendererPNG(texture->DataBuffer()).IsNull();
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

#if DEBUG
void Core3DDebugDefineFrameBinXCAFFormat(
    const Handle(TDocStd_Application)& application,
    const std::shared_ptr<core3d::persistence::AuthoredFrameReadBudget>& budget)
{
    if (application.IsNull() || !budget) return;
    application->DefineFormat(TCollection_AsciiString("BinXCAF"),
        TCollection_AsciiString("Private frame test document"), TCollection_AsciiString("xbf"),
        new Core3DBoundedBinXCAFRetrievalDriver(budget),
        new BinXCAFDrivers_DocumentStorageDriver());
}
#endif

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
#if DEBUG
    myObservedApplication = new core3d::debug::LiveObservedApplication();
    myAuthorityApplication = myObservedApplication;
#else
    myAuthorityApplication = new core3d::authority::NativeObservedApplication();
#endif
    myApp = myAuthorityApplication;
    std::array<std::uint8_t, 16> nonce{};
    [[NSUUID UUID] getUUIDBytes:nonce.data()];
    myNativeAuthority = std::make_shared<core3d::authority::NativeEditAuthority>(nonce);
    if (!myAuthorityApplication->ObserveAuthority(myNativeAuthority)) myNativeAuthority.reset();
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
  if (myNativeAuthority) myNativeAuthority->Detach();
#if DEBUG
  if (myLiveProbe) myLiveProbe->Detach(myOcafDoc);
#endif
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
  ObserveSuccessfulNativeDocumentAdoption();
#if DEBUG
  DebugObserveSuccessfulDocumentAdoption();
#endif
}

void OcctDocument::ObserveSuccessfulNativeDocumentAdoption() noexcept {
    if (!myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()
            || DocumentIdentifier().empty()) {
            myNativeAuthority->Detach(); return;
        }
        myNativeAuthority->Adopt(myOcafDoc.get());
    } catch (...) { myNativeAuthority->Detach(); }
}

std::optional<core3d::authority::QueuedLoadReservation>
OcctDocument::BeginNativeQueuedLoad() noexcept {
    if (!myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return std::nullopt;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()) return std::nullopt;
        // Existing typed preview/recovery ledgers are rejected by the viewer.
        // Profile/alignment private workers are settled by the controller before adoption.
        return myNativeAuthority->BeginQueuedLoad(myOcafDoc.get(),
            core3d::authority::QueuedLoadAdmission::Ready, true, myOcafDoc->HasOpenCommand());
    } catch (...) { return std::nullopt; }
}

bool OcctDocument::OwnsNativeQueuedLoad(
    const core3d::authority::QueuedLoadReservation& reservation) noexcept {
    if (!myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return false;
    try {
        return !myOcafDoc.IsNull() && myOcafDoc->Application().get() == myApp.get()
            && myNativeAuthority->OwnsQueuedLoad(reservation);
    } catch (...) { return false; }
}

core3d::authority::QueuedLoadEnd OcctDocument::EndNativeQueuedLoadPrivateWork(
    const core3d::authority::QueuedLoadReservation& reservation, bool privateWorkSettled) noexcept {
    using core3d::authority::QueuedLoadEnd;
    if (!myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return QueuedLoadEnd::Unavailable;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()) return QueuedLoadEnd::Unavailable;
        return myNativeAuthority->EndQueuedLoadPrivateWork(reservation, privateWorkSettled);
    } catch (...) { return QueuedLoadEnd::Unavailable; }
}

std::optional<core3d::authority::ReplacementReservation> OcctDocument::PromoteNativeQueuedLoad(
    const core3d::authority::QueuedLoadReservation& reservation, bool nativeEditReady) noexcept {
    if (!myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return std::nullopt;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()) return std::nullopt;
        return myNativeAuthority->PromoteQueuedLoadToReplacement(reservation, myOcafDoc.get(),
            nativeEditReady, myOcafDoc->HasOpenCommand());
    } catch (...) { return std::nullopt; }
}

std::optional<core3d::authority::ReplacementReservation> OcctDocument::BeginNativeReplacement() noexcept {
    if (!myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return std::nullopt;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()) return std::nullopt;
        return myNativeAuthority->BeginReplacement(myOcafDoc.get(), true, myOcafDoc->HasOpenCommand());
    } catch (...) { return std::nullopt; }
}

core3d::authority::ReplacementEnd OcctDocument::EndNativeReplacement(
    const core3d::authority::ReplacementReservation& reservation,
    bool accepted, bool restored) noexcept {
    using core3d::authority::ReplacementEnd;
    if (!myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid()) return ReplacementEnd::Unavailable;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()
            || DocumentIdentifier().empty()) return ReplacementEnd::Unavailable;
        return myNativeAuthority->EndReplacement(reservation, myOcafDoc.get(), accepted,
            restored, myOcafDoc->HasOpenCommand());
    } catch (...) { return ReplacementEnd::Unavailable; }
}

#if DEBUG
std::optional<core3d::authority::Stamp> OcctDocument::DebugNativeMutationStamp() noexcept {
    // Diagnostic read only: selection and owner/preview fences are not wired.
    if (!myNativeAuthority || !myAuthorityApplication
        || !myAuthorityApplication->AuthorityThreadContractValid() || myOcafDoc.IsNull())
        return std::nullopt;
    return myNativeAuthority->Capture(myOcafDoc.get(), true, myOcafDoc->HasOpenCommand());
}
bool OcctDocument::DebugStartLiveTransactionProbe() noexcept {
    if (![NSThread isMainThread] || myLiveProbe || myObservedApplication == nullptr
        || myApp.get() != myObservedApplication
        || !myObservedApplication->ThreadContractValid()) return false;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->HasOpenCommand()
            || myOcafDoc->Application().get() != myApp.get()
            || DocumentIdentifier().empty()) return false;
        auto state = std::make_shared<core3d::debug::LiveTransactionProbeState>();
        state->Adopt(myOcafDoc, true);
        if (!state->IsValid() || !myObservedApplication->Observe(state)) return false;
        myLiveProbe = std::move(state);
        return true;
    } catch (...) { return false; }
}
void OcctDocument::DebugStopLiveTransactionProbe() noexcept {
    if (![NSThread isMainThread] || !myLiveProbe || myObservedApplication == nullptr
        || !myLiveProbe->OnOwningThread()) return;
    // Invalid/overflowed observation must still be detachable on its owner.
    myLiveProbe->Detach(myOcafDoc);
    myObservedApplication->Observe({});
    myLiveProbe.reset();
}
std::shared_ptr<const core3d::debug::LiveTransactionProbeState>
OcctDocument::DebugLiveTransactionProbe() const noexcept {
    return [NSThread isMainThread] ? myLiveProbe : nullptr;
}
bool OcctDocument::DebugLiveTransactionProbeValid() const noexcept {
    return [NSThread isMainThread] && myLiveProbe && myLiveProbe->IsValid()
        && myObservedApplication != nullptr && myObservedApplication->ThreadContractValid();
}
void OcctDocument::DebugObserveSuccessfulDocumentAdoption() noexcept {
    if (!myLiveProbe || !myLiveProbe->OnOwningThread()) return;
    try {
        if (myOcafDoc.IsNull() || myOcafDoc->Application().get() != myApp.get()
            || DocumentIdentifier().empty()) { myLiveProbe->valid = false; return; }
        myLiveProbe->Adopt(myOcafDoc);
    } catch (...) { myLiveProbe->valid = false; }
}
#endif

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

OcctGeometryRepresentation OcctDocument::GeometryRepresentationForLabel(
    const TDF_Label& label) const
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return OcctGeometryRepresentation::Invalid;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        return ValidatedGeometryRepresentation(
            myOcafDoc, aShapeTool, label);
    } catch (...) {
        return OcctGeometryRepresentation::Invalid;
    }
}

OcctGeometryRepresentation
OcctDocument::StoredGeometryRepresentationForLabel(
    const TDF_Label& label) const
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return OcctGeometryRepresentation::Invalid;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (!IsGeometryDefinitionLabel(
                myOcafDoc, aShapeTool, label)) {
            return OcctGeometryRepresentation::Invalid;
        }
        bool hasMarker = false;
        OcctGeometryRepresentation aRepresentation =
            OcctGeometryRepresentation::Invalid;
        if (!ReadGeometryRepresentation(
                label, hasMarker, aRepresentation)) {
            return OcctGeometryRepresentation::Invalid;
        }
        (void)hasMarker;
        return aRepresentation;
    } catch (...) {
        return OcctGeometryRepresentation::Invalid;
    }
}

OcctReferenceAxisReadState OcctDocument::ReadReferenceAxisForLabel(
    const TDF_Label& label,
    OcctReferenceAxis& axis) const
{
    axis = DefaultReferenceAxis();
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return OcctReferenceAxisReadState::Invalid;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (!IsGeometryDefinitionLabel(myOcafDoc, aShapeTool, label)) {
            return OcctReferenceAxisReadState::Invalid;
        }
        const OcctReferenceAxisReadState aState =
            ReadReferenceAxisRecord(label, axis);
        if (aState == OcctReferenceAxisReadState::Authored
            && !XCAFDoc_ShapeTool::IsFree(label)) {
            axis = DefaultReferenceAxis();
            return OcctReferenceAxisReadState::Invalid;
        }
        return aState;
    } catch (...) {
        axis = DefaultReferenceAxis();
        return OcctReferenceAxisReadState::Invalid;
    }
}

Standard_Boolean OcctDocument::ResolveReferenceAxisInWorld(
    const TDF_Label& label,
    const TopLoc_Location& occurrenceLocation,
    gp_Ax1& axis) const
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return Standard_False;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        OcctReferenceAxis aReference;
        const OcctReferenceAxisReadState aState =
            ReadReferenceAxisForLabel(label, aReference);
        if (aState == OcctReferenceAxisReadState::Invalid) {
            return Standard_False;
        }
        return TryResolveReferenceAxis(
            myOcafDoc, aShapeTool, label, occurrenceLocation, axis);
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::ValidateGeometryRepresentationForLabel(
    const TDF_Label& label) const
{
    return GeometryRepresentationForLabel(label)
        != OcctGeometryRepresentation::Invalid;
}

Standard_Boolean OcctDocument::ValidateGeometryRepresentations() const
{
    return ValidateGeometryRepresentations(myOcafDoc);
}

namespace {

struct GeometryDocumentUsage
{
    GeometryValidationBudget geometry;
    Standard_Size definitions = 0;
    Standard_Size labels = 0;
    Standard_Size graphVisits = 0;
    Standard_Size leafOccurrences = 0;
};

Standard_Boolean ValidateGeometryDocument(
    const Handle(TDocStd_Document)& document,
    GeometryDocumentUsage* output)
{
    try {
        OCC_CATCH_SIGNALS
        if (document.IsNull() || document->GetData().IsNull()
            || !ValidateCommandOwnerSentinelsDocument(document)
            || !ValidateReferenceAxisDocument(document)
            || !([](const Handle(TDocStd_Document)& doc) {
                OcctSavedGroupState groups; return ReadSavedGroups(doc, groups);
            })(document)) {
            return Standard_False;
        }
        std::vector<core3d::profile::Record> profiles;
        if (!core3d::profile::ValidateDocument(document, profiles)) return Standard_False;
        const TDF_Label aRoot = document->GetData()->Root();
        Handle(TDataStd_Integer) aRootMarker;
        Handle(TNaming_NamedShape) aRootShape;
        if (aRoot.FindAttribute(
                GeometryRepresentationAttributeID(), aRootMarker)
            || aRoot.FindAttribute(
                TNaming_NamedShape::GetID(), aRootShape)) {
            return Standard_False;
        }

        if (!XCAFDoc_DocumentTool::CheckShapeTool(document->Main())) {
            Standard_Size aLabelCount = 0;
            for (TDF_ChildIterator aLabel(aRoot, Standard_True);
                 aLabel.More(); aLabel.Next()) {
                if (++aLabelCount > kMaximumGeometryDocumentLabels) {
                    return Standard_False;
                }
                Handle(TDataStd_Integer) aMarker;
                Handle(TNaming_NamedShape) aNamedShape;
                if (aLabel.Value().FindAttribute(
                        GeometryRepresentationAttributeID(), aMarker)
                    || aLabel.Value().FindAttribute(
                        TNaming_NamedShape::GetID(), aNamedShape)) {
                    return Standard_False;
                }
            }
            if (output != nullptr) {
                GeometryDocumentUsage usage;
                usage.labels = aLabelCount;
                *output = usage;
            }
            return Standard_True;
        }

        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (aShapeTool.IsNull()) {
            return Standard_False;
        }
        TDF_LabelSequence anAllTopLevelLabels;
        aShapeTool->GetShapes(anAllTopLevelLabels);
        if (static_cast<Standard_Size>(
                anAllTopLevelLabels.Length())
            > kMaximumGeometryDocumentLabels) {
            return Standard_False;
        }
        TDF_LabelMap anAllTopLevelLabelSet;
        for (Standard_Integer anIndex = 1;
             anIndex <= anAllTopLevelLabels.Length(); ++anIndex) {
            const TDF_Label& aLabel =
                anAllTopLevelLabels.Value(anIndex);
            if (aLabel.IsNull()
                || aLabel.Data() != document->GetData()
                || !aShapeTool->IsShape(aLabel)
                || !aShapeTool->IsTopLevel(aLabel)
                || XCAFDoc_ShapeTool::IsComponent(aLabel)
                || XCAFDoc_ShapeTool::IsSubShape(aLabel)
                || !anAllTopLevelLabelSet.Add(aLabel)) {
                return Standard_False;
            }
        }
        TDF_LabelSequence aFreeRootLabels;
        aShapeTool->GetFreeShapes(aFreeRootLabels);
        if (static_cast<Standard_Size>(aFreeRootLabels.Length())
            > kMaximumGeometryDocumentLabels) {
            return Standard_False;
        }

        struct AssemblyFrame {
            TDF_Label label;
            Standard_Size depth = 0;
            bool leaving = false;
        };
        TDF_LabelMap aVisitedGraphLabels;
        TDF_LabelMap aDefinitionLabels;
        GeometryValidationBudget aBudget;
        // Current bindings share the already-budgeted root geometry. Stale
        // definitions can retain older solids and must account for them too.
        for (const auto& profile : profiles) {
            if (!profile.boundShape.IsEqual(XCAFDoc_ShapeTool::GetShape(profile.label.Father()))
                && ClassifyDefinitionGeometry(profile.boundShape, &aBudget) != DefinitionGeometryClass::BRep)
                return Standard_False;
        }

        Standard_Size aDefinitionCount = 0;
        Standard_Size anAggregateGraphVisitCount = 0;
        Standard_Size aLeafOccurrenceCount = 0;
        for (Standard_Integer aRootIndex = 1;
             aRootIndex <= aFreeRootLabels.Length(); ++aRootIndex) {
            const TDF_Label& aRootLabel =
                aFreeRootLabels.Value(aRootIndex);
            if (aRootLabel.IsNull()
                || aRootLabel.Data() != document->GetData()
                || !aShapeTool->IsShape(aRootLabel)
                || !aShapeTool->IsTopLevel(aRootLabel)
                || !anAllTopLevelLabelSet.Contains(aRootLabel)
                || XCAFDoc_ShapeTool::IsComponent(aRootLabel)
                || XCAFDoc_ShapeTool::IsSubShape(aRootLabel)) {
                return Standard_False;
            }
            // Depth is path-relative. Revisit shared DAG nodes through every
            // occurrence branch so a shallow first path cannot memoize away
            // an over-depth later path. Definition geometry is still
            // classified once through aDefinitionLabels below.
            TDF_LabelMap anActivePathForRoot;
            std::vector<AssemblyFrame> aStack = {{
                aRootLabel, 0, false}};
            while (!aStack.empty()) {
                const AssemblyFrame aFrame = aStack.back();
                aStack.pop_back();
                const TDF_Label& aLabel = aFrame.label;
                if (aLabel.IsNull()
                    || aLabel.Data() != document->GetData()) {
                    return Standard_False;
                }
                if (aFrame.leaving) {
                    anActivePathForRoot.Remove(aLabel);
                    aVisitedGraphLabels.Add(aLabel);
                    continue;
                }
                if (anActivePathForRoot.Contains(aLabel)
                    || aFrame.depth > kMaximumTopologyDepth
                    || anAggregateGraphVisitCount
                        >= kMaximumGeometryDocumentLabels
                    || !aShapeTool->IsShape(aLabel)) {
                    return Standard_False;
                }
                anActivePathForRoot.Add(aLabel);
                ++anAggregateGraphVisitCount;
                if (aStack.size()
                    >= static_cast<std::size_t>(
                        kMaximumGeometryDocumentLabels)) {
                    return Standard_False;
                }
                aStack.push_back({
                    aLabel, aFrame.depth, true});

                if (XCAFDoc_ShapeTool::IsReference(aLabel)
                    || XCAFDoc_ShapeTool::IsComponent(aLabel)) {
                    TDF_Label aReferredLabel;
                    if (!XCAFDoc_ShapeTool::GetReferredShape(
                            aLabel, aReferredLabel)
                        || aReferredLabel.IsNull()
                        || aReferredLabel.Data()
                            != document->GetData()
                        || aFrame.depth >= kMaximumTopologyDepth
                        || aStack.size()
                            >= static_cast<std::size_t>(
                                kMaximumGeometryDocumentLabels)) {
                        return Standard_False;
                    }
                    aStack.push_back({
                        aReferredLabel, aFrame.depth + 1U, false});
                    continue;
                }
                if (XCAFDoc_ShapeTool::IsAssembly(aLabel)) {
                    TDF_LabelSequence aComponents;
                    if (!XCAFDoc_ShapeTool::GetComponents(
                            aLabel, aComponents, Standard_False)
                        || aComponents.IsEmpty()
                        || static_cast<Standard_Size>(
                               aComponents.Length())
                            > kMaximumGeometryDocumentLabels
                        || aFrame.depth >= kMaximumTopologyDepth) {
                        return Standard_False;
                    }
                    for (Standard_Integer aComponentIndex = 1;
                         aComponentIndex <= aComponents.Length();
                         ++aComponentIndex) {
                        const TDF_Label& aComponent =
                            aComponents.Value(aComponentIndex);
                        if (aComponent.IsNull()
                            || aComponent.Data()
                                != document->GetData()
                            || !XCAFDoc_ShapeTool::IsComponent(
                                aComponent)
                            || aStack.size()
                                >= static_cast<std::size_t>(
                                    kMaximumGeometryDocumentLabels)) {
                            return Standard_False;
                        }
                        aStack.push_back({
                            aComponent,
                            aFrame.depth + 1U,
                            false});
                    }
                    continue;
                }
                if (!IsGeometryDefinitionLabel(
                        document, aShapeTool, aLabel)) {
                    return Standard_False;
                }
                // This path-relative traversal reaches one resolved leaf per
                // occurrence, including repeated references to a shared
                // definition. Geometry itself remains classified once below.
				if (aLeafOccurrenceCount
						>= core3d::limits::kMaximumLeafPresentations) {
					return Standard_False;
				}
                ++aLeafOccurrenceCount;
                if (!aDefinitionLabels.Contains(aLabel)) {
                    if (aDefinitionCount
                            >= kMaximumGeometryDefinitionLabels
                        || !aDefinitionLabels.Add(aLabel)
                        || ValidatedGeometryRepresentation(
                               document, aShapeTool, aLabel, &aBudget)
                            == OcctGeometryRepresentation::Invalid) {
                        return Standard_False;
                    }
                    ++aDefinitionCount;
                }
            }
            if (!anActivePathForRoot.IsEmpty()) {
                return Standard_False;
            }
        }

		// The definition pass above validates authored and implicit axes at an
		// identity occurrence. Schema v6 publishes the resolved world axis for
		// every leaf, so admission must also prove each real cumulative XCAF
		// occurrence location. Use the same explorer/location authority as the
		// snapshot builder and cross-check its count against the bounded graph
		// traversal so neither path can silently omit a leaf.
		Standard_Size aResolvedReferenceOccurrenceCount = 0;
		XCAFPrs_DocumentExplorer anOccurrenceExplorer(
			document,
			XCAFPrs_DocumentExplorerFlags_OnlyLeafNodes,
			XCAFPrs_Style());
		for (; anOccurrenceExplorer.More(); anOccurrenceExplorer.Next()) {
			if (aResolvedReferenceOccurrenceCount
					>= core3d::limits::kMaximumLeafPresentations) {
				return Standard_False;
			}
			const XCAFPrs_DocumentNode& aNode =
				anOccurrenceExplorer.Current();
			const TDF_Label aDefinitionLabel = aNode.RefLabel.IsNull()
				? aNode.Label : aNode.RefLabel;
			gp_Ax1 aResolvedReferenceAxis;
			if (aDefinitionLabel.IsNull()
				|| !aDefinitionLabels.Contains(aDefinitionLabel)
				|| !TryResolveReferenceAxis(
					document,
					aShapeTool,
					aDefinitionLabel,
					aNode.Location,
					aResolvedReferenceAxis)) {
				return Standard_False;
			}
			++aResolvedReferenceOccurrenceCount;
		}
		if (aResolvedReferenceOccurrenceCount != aLeafOccurrenceCount) {
			return Standard_False;
		}
        for (TDF_MapIteratorOfLabelMap aTopLevel(
                 anAllTopLevelLabelSet);
             aTopLevel.More(); aTopLevel.Next()) {
            if (!aVisitedGraphLabels.Contains(aTopLevel.Key())) {
                // Non-free top-level labels must be reachable from some free
                // root. This rejects rootless assembly cycles and orphaned
                // reference islands even if their individual labels look
                // structurally plausible.
                return Standard_False;
            }
        }

        TDF_LabelMap aValidatedSubshapeLabels;
        Standard_Size aSubshapeLabelCount = 0;
        for (TDF_MapIteratorOfLabelMap aDefinition(
                 aDefinitionLabels);
             aDefinition.More(); aDefinition.Next()) {
            TDF_LabelSequence aSubshapes;
            if (!XCAFDoc_ShapeTool::GetSubShapes(
                    aDefinition.Key(), aSubshapes)) {
                continue;
            }
            if (static_cast<Standard_Size>(aSubshapes.Length())
                > kMaximumGeometryDocumentLabels) {
                return Standard_False;
            }
            for (Standard_Integer aSubshapeIndex = 1;
                 aSubshapeIndex <= aSubshapes.Length();
                 ++aSubshapeIndex) {
                const TDF_Label& aSubshape =
                    aSubshapes.Value(aSubshapeIndex);
                if (aSubshape.IsNull()
                    || aSubshape.Data() != document->GetData()
                    || !aShapeTool->IsShape(aSubshape)
                    || !XCAFDoc_ShapeTool::IsSubShape(aSubshape)) {
                    return Standard_False;
                }
                if (!aValidatedSubshapeLabels.Contains(aSubshape)) {
                    if (aSubshapeLabelCount
                            >= kMaximumGeometryDocumentLabels
                        || !aValidatedSubshapeLabels.Add(aSubshape)) {
                        return Standard_False;
                    }
                    ++aSubshapeLabelCount;
                }
            }
        }

        Standard_Size aLabelCount = 0;
        for (TDF_ChildIterator aLabel(aRoot, Standard_True);
             aLabel.More(); aLabel.Next()) {
            if (++aLabelCount > kMaximumGeometryDocumentLabels) {
                return Standard_False;
            }
            Handle(TDataStd_Integer) aMarker;
            Handle(TNaming_NamedShape) aNamedShape;
            if (aLabel.Value().FindAttribute(
                    GeometryRepresentationAttributeID(), aMarker)
                && !aDefinitionLabels.Contains(aLabel.Value())) {
                return Standard_False;
            }
            if (aLabel.Value().FindAttribute(
                    TNaming_NamedShape::GetID(), aNamedShape)
                && !aVisitedGraphLabels.Contains(aLabel.Value())
                && !aValidatedSubshapeLabels.Contains(
                    aLabel.Value())) {
                return Standard_False;
            }
        }
        if (output != nullptr) {
            GeometryDocumentUsage usage;
            usage.geometry = aBudget;
            usage.definitions = aDefinitionCount;
            usage.labels = aLabelCount;
            usage.graphVisits = anAggregateGraphVisitCount;
            usage.leafOccurrences = aLeafOccurrenceCount;
            *output = usage;
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

} // namespace

Standard_Boolean OcctDocument::ValidateGeometryRepresentations(
    const Handle(TDocStd_Document)& document) const
{
    return ValidateGeometryDocument(document, nullptr);
}

Standard_Boolean OcctDocument::ValidateReferenceAxes() const
{
    return ValidateReferenceAxes(myOcafDoc);
}

Standard_Boolean OcctDocument::ValidateReferenceAxes(
    const Handle(TDocStd_Document)& document) const
{
    return ValidateReferenceAxisDocument(document);
}

Standard_Boolean OcctDocument::CanDuplicateGeometryDefinitions(
    const std::vector<TDF_Label>& sourceDefinitionLabels) const
{
    try {
        OCC_CATCH_SIGNALS
        if (sourceDefinitionLabels.empty()
            || sourceDefinitionLabels.size()
                > static_cast<std::size_t>(
                    kMaximumGeometryDefinitionLabels)) {
            return Standard_False;
        }

        std::vector<OcctGeometryDuplicationRequest> requests;
        requests.reserve(sourceDefinitionLabels.size());
        for (const TDF_Label& source : sourceDefinitionLabels) {
            requests.push_back({source, 1U});
        }
        return CanDuplicateGeometryDefinitions(requests);
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::CanDuplicateGeometryDefinitions(
    const std::vector<OcctGeometryDuplicationRequest>& requests) const
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || myOcafDoc->HasOpenCommand()
            || requests.empty()
            || requests.size()
                > static_cast<std::size_t>(
                    kMaximumGeometryDefinitionLabels)
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return Standard_False;
        }

        GeometryDocumentUsage current;
        if (!ValidateGeometryDocument(myOcafDoc, &current)) {
            return Standard_False;
        }
        const Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (shapeTool.IsNull()) {
            return Standard_False;
        }

        Standard_Size projectedNormalBytes = 0;
        if (!Core3DValidateOwnedFrameUsage(myOcafDoc,projectedNormalBytes)) return Standard_False;

        GeometryValidationBudget projectedGeometry = current.geometry;
        Standard_Size projectedDefinitions = current.definitions;
        Standard_Size projectedLabels = current.labels;
        Standard_Size projectedGraphVisits = current.graphVisits;
        Standard_Size projectedLeafOccurrences = current.leafOccurrences;
        TDF_LabelMap uniqueSources;
        for (const OcctGeometryDuplicationRequest& request : requests) {
            const TDF_Label& source = request.sourceDefinition;
            const Standard_Size destinationCount =
                request.destinationCount;
            if (source.IsNull()
                || source.Data() != myOcafDoc->GetData()
                || destinationCount == 0U
                || !IsEditableFreeSimpleDefinitionLabel(source)
                || !uniqueSources.Add(source)) {
                return Standard_False;
            }

            GeometryValidationBudget sourceGeometry;
            if (ValidatedGeometryRepresentation(
                    myOcafDoc, shapeTool, source, &sourceGeometry)
                    == OcctGeometryRepresentation::Invalid) {
                return Standard_False;
            }

            OcctAuthoredFrameRecord sourceFrame;
            if (Core3DReadAuthoredFrameOwner(myOcafDoc, source, sourceFrame) == OcctAuthoredFrameReadState::Invalid
                || !AddMultipliedWithinLimit(projectedNormalBytes, sourceFrame.nativeBytes, destinationCount,
                    64U * 1024U * 1024U)) return Standard_False;

            XCAFDoc_VisMaterialPBR sourceMaterial;
            if (TryPBRMaterialForLabel(source, sourceMaterial) && !sourceMaterial.NormalTexture.IsNull()) {
                Standard_Size bytes = 0;
                if (!Core3DValidateNormalTextureBinding(myOcafDoc, source, &bytes)
                    || !AddMultipliedWithinLimit(projectedNormalBytes, bytes, destinationCount,
                        64U * 1024U * 1024U)) return Standard_False;
            }

            // AddShape creates one definition label and eight transform
            // children. CopyObjectAppearance creates the legacy material and
            // color children only when a local PBR assignment is not
            // authoritative; visual-material/texture definitions are shared.
            Standard_Size destinationLabels = 9U;
            Handle(TDataStd_Integer) localPBRMarker;
            const Standard_Boolean hasLocalPBR =
                source.FindAttribute(
                    LocalPBRMaterialAttributeID(), localPBRMarker)
                && !localPBRMarker.IsNull()
                && localPBRMarker->Get() == 1;
            if (!hasLocalPBR) {
                Graphic3d_NameOfMaterial material;
                Quantity_NameOfColor color;
                if (TryMaterialNameForLabel(source, material)) {
                    ++destinationLabels;
                }
                if (TryColorNameForLabel(source, color)) {
                    ++destinationLabels;
                }
            }
            if (!AddMultipliedWithinLimit(
                    projectedDefinitions,
                    1U,
                    destinationCount,
                    kMaximumGeometryDefinitionLabels)
                || !AddMultipliedWithinLimit(
                    projectedGraphVisits,
                    1U,
                    destinationCount,
                    kMaximumGeometryDocumentLabels)
                || !AddMultipliedWithinLimit(
                    projectedLeafOccurrences,
                    1U,
                    destinationCount,
                    static_cast<Standard_Size>(
                        core3d::limits::kMaximumLeafPresentations))
                || !AddMultipliedWithinLimit(
                    projectedGeometry.subshapes,
                    sourceGeometry.subshapes,
                    destinationCount,
                    kMaximumSubshapesPerDocument)
                || !AddMultipliedWithinLimit(
                    projectedGeometry.meshVertices,
                    sourceGeometry.meshVertices,
                    destinationCount,
                    kMaximumMeshVerticesPerDocument)
                || !AddMultipliedWithinLimit(
                    projectedGeometry.meshIndices,
                    sourceGeometry.meshIndices,
                    destinationCount,
                    kMaximumMeshIndicesPerDocument)
                || !AddMultipliedWithinLimit(
                    projectedLabels,
                    destinationLabels,
                    destinationCount,
                    kMaximumGeometryDocumentLabels)) {
                return Standard_False;
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Integer OcctDocument::SupportedGeometryExportFormats() const
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || myOcafDoc->HasOpenCommand()
            || !ValidateGeometryRepresentations()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return 0;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (aShapeTool.IsNull()) {
            return 0;
        }
        TDF_LabelSequence aLabels;
        aShapeTool->GetShapes(aLabels);
        if (static_cast<Standard_Size>(aLabels.Length())
            > kMaximumGeometryDocumentLabels) {
            return 0;
        }
        Standard_Integer aFormats =
            static_cast<Standard_Integer>(OcctGeometryExportFormat::Obj)
            | static_cast<Standard_Integer>(OcctGeometryExportFormat::Stl)
            | static_cast<Standard_Integer>(OcctGeometryExportFormat::Gltf)
            | static_cast<Standard_Integer>(OcctGeometryExportFormat::Step);
        Standard_Size aDefinitionCount = 0;
        for (Standard_Integer anIndex = 1;
             anIndex <= aLabels.Length(); ++anIndex) {
            const TDF_Label& aLabel = aLabels.Value(anIndex);
            if (aLabel.IsNull()
                || aLabel.Data() != myOcafDoc->GetData()
                || !aShapeTool->IsShape(aLabel)) {
                return 0;
            }
            if (!IsGeometryDefinitionLabel(
                    myOcafDoc, aShapeTool, aLabel)) {
                continue;
            }
            if (aDefinitionCount
                >= kMaximumGeometryDefinitionLabels) {
                return 0;
            }
            ++aDefinitionCount;
            const OcctGeometryRepresentation aRepresentation =
                GeometryRepresentationForLabel(aLabel);
            if (aRepresentation
                == OcctGeometryRepresentation::TriangleMesh) {
                aFormats &= ~static_cast<Standard_Integer>(
                    OcctGeometryExportFormat::Step);
            } else if (aRepresentation
                    != OcctGeometryRepresentation::LegacyUnknown
                && aRepresentation
                    != OcctGeometryRepresentation::BRep) {
                return 0;
            }
        }
        return aDefinitionCount > 0 ? aFormats : 0;
    } catch (...) {
        return 0;
    }
}

Standard_Boolean OcctDocument::CanExportGeometry(
    const OcctGeometryExportFormat format) const
{
    const Standard_Integer aRequested =
        static_cast<Standard_Integer>(format);
    const Standard_Integer aKnown =
        static_cast<Standard_Integer>(OcctGeometryExportFormat::Obj)
        | static_cast<Standard_Integer>(OcctGeometryExportFormat::Stl)
        | static_cast<Standard_Integer>(OcctGeometryExportFormat::Gltf)
        | static_cast<Standard_Integer>(OcctGeometryExportFormat::Step);
    return aRequested != 0 && (aRequested & ~aKnown) == 0
        && (SupportedGeometryExportFormats() & aRequested)
            == aRequested;
}

Standard_Boolean OcctDocument::IsGeometryDocumentEmpty() const
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || myOcafDoc->HasOpenCommand()
            || !ValidateGeometryRepresentations()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return Standard_False;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (aShapeTool.IsNull()) {
            return Standard_False;
        }
        TDF_LabelSequence aLabels;
        aShapeTool->GetShapes(aLabels);
        if (static_cast<Standard_Size>(aLabels.Length())
            > kMaximumGeometryDocumentLabels) {
            return Standard_False;
        }
        for (Standard_Integer anIndex = 1;
             anIndex <= aLabels.Length(); ++anIndex) {
            const TDF_Label& aLabel = aLabels.Value(anIndex);
            if (aLabel.IsNull()
                || aLabel.Data() != myOcafDoc->GetData()
                || !aShapeTool->IsShape(aLabel)) {
                return Standard_False;
            }
            if (IsGeometryDefinitionLabel(
                    myOcafDoc, aShapeTool, aLabel)) {
                return Standard_False;
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::SetGeometryRepresentationForLabel(
    const TDF_Label& label,
    const OcctGeometryRepresentation representation)
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || representation == OcctGeometryRepresentation::Invalid
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return Standard_False;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (!IsGeometryDefinitionLabel(
                myOcafDoc, aShapeTool, label)) {
            return Standard_False;
        }
        bool hasMarker = false;
        OcctGeometryRepresentation anExistingRepresentation =
            OcctGeometryRepresentation::Invalid;
        if (!ReadGeometryRepresentation(
                label, hasMarker, anExistingRepresentation)) {
            return Standard_False;
        }
        const DefinitionGeometryClass aGeometryClass =
            ClassifyDefinitionGeometry(
                XCAFDoc_ShapeTool::GetShape(label), nullptr);
        if ((hasMarker
                && !GeometryClassMatchesRepresentation(
                    aGeometryClass, anExistingRepresentation))
            || !GeometryClassMatchesRepresentation(
                aGeometryClass, representation)) {
            return Standard_False;
        }
        return WriteGeometryRepresentationMarker(
            label, representation);
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean
OcctDocument::EnsureGeometryRepresentationForMutation(
    const TDF_Label& label)
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return Standard_False;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        if (!IsGeometryDefinitionLabel(
                myOcafDoc, aShapeTool, label)) {
            return Standard_False;
        }
        bool hasMarker = false;
        OcctGeometryRepresentation aRepresentation =
            OcctGeometryRepresentation::Invalid;
        if (!ReadGeometryRepresentation(
                label, hasMarker, aRepresentation)) {
            return Standard_False;
        }
        if (hasMarker
            && (aRepresentation == OcctGeometryRepresentation::BRep
                || aRepresentation
                    == OcctGeometryRepresentation::TriangleMesh)) {
            // Candidate documents and every geometry replacement validate the
            // explicit marker once. Ordinary scalar/transform mutations must
            // not rescan up to a million mesh vertices on the main thread.
            return Standard_True;
        }
        if (aRepresentation
                != OcctGeometryRepresentation::LegacyUnknown
            || ClassifyDefinitionGeometry(
                   XCAFDoc_ShapeTool::GetShape(label), nullptr)
                != DefinitionGeometryClass::BRep) {
            return Standard_False;
        }
        return WriteGeometryRepresentationMarker(
            label, OcctGeometryRepresentation::BRep);
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::CopyGeometryRepresentation(
    const TDF_Label& source,
    const TDF_Label& destination)
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return Standard_False;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        OcctGeometryRepresentation aSourceRepresentation =
            ValidatedGeometryRepresentation(
                myOcafDoc, aShapeTool, source);
        if (aSourceRepresentation
            == OcctGeometryRepresentation::LegacyUnknown) {
            aSourceRepresentation = OcctGeometryRepresentation::BRep;
        }
        if (aSourceRepresentation
                == OcctGeometryRepresentation::Invalid
            || !IsGeometryDefinitionLabel(
                myOcafDoc, aShapeTool, destination)) {
            return Standard_False;
        }
        bool hasDestinationMarker = false;
        OcctGeometryRepresentation aDestinationRepresentation =
            OcctGeometryRepresentation::Invalid;
        if (!ReadGeometryRepresentation(
                destination,
                hasDestinationMarker,
                aDestinationRepresentation)) {
            return Standard_False;
        }
        const DefinitionGeometryClass aDestinationClass =
            ClassifyDefinitionGeometry(
                XCAFDoc_ShapeTool::GetShape(destination), nullptr);
        if ((hasDestinationMarker
                && !GeometryClassMatchesRepresentation(
                    aDestinationClass, aDestinationRepresentation))
            || !GeometryClassMatchesRepresentation(
                aDestinationClass, aSourceRepresentation)) {
            return Standard_False;
        }
        return WriteGeometryRepresentationMarker(
            destination, aSourceRepresentation);
    } catch (...) {
        return Standard_False;
    }
}

namespace {

Standard_Boolean MarkImportedDefinitions(
    const Handle(TDocStd_Document)& theDocument,
    const OcctGeometryRepresentation theExpectedRepresentation)
{
    if (theDocument.IsNull() || theDocument->HasOpenCommand()
        || theDocument->GetAvailableUndos() != 0
        || theDocument->GetAvailableRedos() != 0
        || (theExpectedRepresentation != OcctGeometryRepresentation::BRep
            && theExpectedRepresentation
                != OcctGeometryRepresentation::TriangleMesh)
        || !XCAFDoc_DocumentTool::CheckShapeTool(
            theDocument->Main())) {
        return Standard_False;
    }

    const DefinitionGeometryClass anExpectedClass =
        theExpectedRepresentation == OcctGeometryRepresentation::BRep
        ? DefinitionGeometryClass::BRep
        : DefinitionGeometryClass::TriangleMesh;
    const Handle(XCAFDoc_ShapeTool) aShapeTool =
        XCAFDoc_DocumentTool::ShapeTool(theDocument->Main());
    if (aShapeTool.IsNull()) {
        return Standard_False;
    }
    TDF_LabelSequence aShapeLabels;
    aShapeTool->GetShapes(aShapeLabels);
    if (aShapeLabels.IsEmpty()
        || static_cast<Standard_Size>(aShapeLabels.Length())
            > kMaximumGeometryDocumentLabels) {
        return Standard_False;
    }

    std::vector<TDF_Label> aLegacyDefinitions;
    TDF_LabelMap aVisitedDefinitions;
    GeometryValidationBudget aBudget;
    try {
        OCC_CATCH_SIGNALS
        aLegacyDefinitions.reserve(
            static_cast<std::size_t>(aShapeLabels.Length()));
        for (Standard_Integer anIndex = 1;
             anIndex <= aShapeLabels.Length(); ++anIndex) {
            const TDF_Label& aTopLevelLabel =
                aShapeLabels.Value(anIndex);
            if (XCAFDoc_ShapeTool::IsAssembly(aTopLevelLabel)) {
                continue;
            }
            TDF_Label aLabel = aTopLevelLabel;
            if (XCAFDoc_ShapeTool::IsReference(aTopLevelLabel)) {
                if (!XCAFDoc_ShapeTool::GetReferredShape(
                        aTopLevelLabel, aLabel)
                    || aLabel.IsNull()) {
                    return Standard_False;
                }
                if (XCAFDoc_ShapeTool::IsAssembly(aLabel)) {
                    continue;
                }
            }
            if (!IsGeometryDefinitionLabel(
                    theDocument, aShapeTool, aLabel)) {
                return Standard_False;
            }
            if (aVisitedDefinitions.Contains(aLabel)) {
                continue;
            }
            if (!aVisitedDefinitions.Add(aLabel)
                || static_cast<Standard_Size>(
                    aVisitedDefinitions.Extent())
                    > kMaximumGeometryDefinitionLabels
                || ClassifyDefinitionGeometry(
                    XCAFDoc_ShapeTool::GetShape(aLabel), &aBudget)
                    != anExpectedClass) {
                return Standard_False;
            }

            bool hasMarker = false;
            OcctGeometryRepresentation aRepresentation =
                OcctGeometryRepresentation::Invalid;
            if (!ReadGeometryRepresentation(
                    aLabel, hasMarker, aRepresentation)) {
                return Standard_False;
            }
            (void)hasMarker;
            if (aRepresentation
                    == OcctGeometryRepresentation::LegacyUnknown) {
                aLegacyDefinitions.push_back(aLabel);
            } else if (aRepresentation
                    != theExpectedRepresentation) {
                return Standard_False;
            }
        }
        if (aVisitedDefinitions.IsEmpty()) {
            return Standard_False;
        }
        if (aLegacyDefinitions.empty()) {
            return Standard_True;
        }

        const Standard_Integer aPreviousUndoLimit =
            theDocument->GetUndoLimit();
        theDocument->SetUndoLimit(1);
        theDocument->NewCommand();
        if (!theDocument->HasOpenCommand()) {
            theDocument->SetUndoLimit(aPreviousUndoLimit);
            return Standard_False;
        }
        try {
            for (const TDF_Label& aLabel : aLegacyDefinitions) {
                if (!WriteGeometryRepresentationMarker(
                        aLabel, theExpectedRepresentation)) {
                    throw Standard_Failure(
                        "Unable to mark imported geometry definition");
                }
            }
            if (!theDocument->CommitCommand()) {
                throw Standard_Failure(
                    "Unable to commit imported geometry markers");
            }
            theDocument->ClearUndos();
            theDocument->SetUndoLimit(aPreviousUndoLimit);
        } catch (...) {
            AbortCommandNoThrow(theDocument);
            try {
                theDocument->ClearUndos();
            } catch (...) {
            }
            try {
                theDocument->SetUndoLimit(aPreviousUndoLimit);
            } catch (...) {
            }
            return Standard_False;
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

} // namespace

Standard_Boolean OcctDocument::MarkImportedBRepDefinitions()
{
    return MarkImportedDefinitions(
               myOcafDoc, OcctGeometryRepresentation::BRep)
        && ValidateGeometryRepresentations();
}

Standard_Boolean OcctDocument::MarkImportedTriangleMeshDefinitions()
{
    return MarkImportedDefinitions(
               myOcafDoc,
               OcctGeometryRepresentation::TriangleMesh)
        && ValidateGeometryRepresentations();
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

namespace {
bool TriangleAtlasFace(const TopoDS_Shape& shape, TopoDS_Face& face,
                       Handle(Poly_Triangulation)& mesh) {
    if (shape.ShapeType() != TopAbs_FACE) {
        if (shape.ShapeType() != TopAbs_COMPOUND) { return false; }
        TopoDS_Iterator children(shape);
        if (!children.More() || children.Value().ShapeType() != TopAbs_FACE) { return false; }
        children.Next(); if (children.More()) { return false; }
    }
    int faces = 0;
    for (TopExp_Explorer it(shape, TopAbs_FACE); it.More(); it.Next()) {
        face = TopoDS::Face(it.Current());
        if (++faces > 1) { return false; }
    }
    if (faces != 1 || !BRep_Tool::Surface(face).IsNull()) { return false; }
    TopLoc_Location location;
    mesh = BRep_Tool::Triangulation(face, location);
    return !mesh.IsNull() && !mesh->HasDeferredData() && mesh->HasGeometry()
        && mesh->NbTriangles() > 0 && mesh->NbTriangles() <= 4096
        && mesh->NbNodes() > 0 && mesh->NbNodes() <= 24576;
}
}

Standard_Integer Core3DNormalTextureRecipeForLabel(const TDF_Label& label) noexcept
{
    try {
        if (label.IsNull()) return 0;
        Handle(TDF_Attribute) attribute;
        if (!label.FindAttribute(NormalTextureRecipeAttributeID(), attribute)) return 0;
        const auto value = Handle(TDataStd_Integer)::DownCast(attribute);
        return !value.IsNull() && (value->Get() == 1 || value->Get() == 2) ? value->Get() : -1;
    } catch (...) { return -1; }
}

OcctAuthoredFrameReadState Core3DReadAuthoredFrameOwner(
    const Handle(TDocStd_Document)& document, const TDF_Label& label,
    OcctAuthoredFrameRecord& record) noexcept {
    record = {};
    try {
        using namespace core3d::persistence;
        using namespace core3d::scene::authored;
        if (document.IsNull() || document->GetData().IsNull() || label.IsNull()
            || label.Data() != document->GetData()) return OcctAuthoredFrameReadState::Invalid;
        Handle(TDF_Attribute) attribute;
        if (!label.FindAttribute(AuthoredFrameAttributeID(), attribute)) return OcctAuthoredFrameReadState::Absent;
        const auto bytes = Handle(TDataStd_ByteArray)::DownCast(attribute);
        if (bytes.IsNull() || bytes->Lower() != 0 || bytes->Upper() < 127
            || bytes->Upper() >= Standard_Integer(kMaximumArchiveBytes) || bytes->GetDelta()
            || !XCAFDoc_DocumentTool::CheckShapeTool(document->Main())) return OcctAuthoredFrameReadState::Invalid;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (!IsGeometryDefinitionLabel(document, shapes, label) || !shapes->IsTopLevel(label)
            || !XCAFDoc_ShapeTool::IsFree(label)
            || ValidatedGeometryRepresentation(document, shapes, label) != OcctGeometryRepresentation::TriangleMesh)
            return OcctAuthoredFrameReadState::Invalid;
        double unit = 0;
        if (!XCAFDoc_DocumentTool::GetLengthUnit(document, unit) || unit != 0.001)
            return OcctAuthoredFrameReadState::Invalid;
        // Validate the placed definition above, then remove only its outer
        // placement. Child orientation/location remain part of the contract.
        auto local = XCAFDoc_ShapeTool::GetShape(label);
        local.Location(TopLoc_Location());
        TopoDS_Face face;
        if (local.ShapeType() == TopAbs_FACE) face = TopoDS::Face(local);
        else if (local.ShapeType() == TopAbs_COMPOUND) {
            TopoDS_Iterator children(local);
            if (!children.More() || children.Value().ShapeType() != TopAbs_FACE)
                return OcctAuthoredFrameReadState::Invalid;
            face = TopoDS::Face(children.Value());
            children.Next(); if (children.More()) return OcctAuthoredFrameReadState::Invalid;
        } else return OcctAuthoredFrameReadState::Invalid;
        OcctAuthoredFrameRecord result;
        result.archive.resize(Standard_Size(bytes->Upper()) + 1);
        for (Standard_Integer i = 0; i <= bytes->Upper(); ++i) result.archive[Standard_Size(i)] = bytes->Value(i);
        std::vector<core3d::scene::Float4> frames;
        if (!DecodeNativeAuthoredFrames(face, result.archive.data(), result.archive.size(), frames))
            return OcctAuthoredFrameReadState::Invalid;
        result.cornerCount = frames.size();
        result.nativeBytes = result.archive.size() + result.cornerCount * 64U;
        std::copy(result.archive.end() - kDigestBytes, result.archive.end(), result.identity.begin());
        record = std::move(result);
        return OcctAuthoredFrameReadState::Authored;
    } catch (...) { record = {}; return OcctAuthoredFrameReadState::Invalid; }
}

Standard_Boolean Core3DValidateAuthoredFrameOwners(
    const Handle(TDocStd_Document)& document, Standard_Size& nativeBytes,
    Standard_Size maximumBytes) noexcept {
    nativeBytes = 0;
    try {
        if (document.IsNull() || document->GetData().IsNull() || maximumBytes > 64U * 1024U * 1024U)
            return Standard_False;
        Standard_Size total = 0;
        auto validate = [&](const TDF_Label& label) {
            for (TDF_AttributeIterator it(label); it.More(); it.Next()) {
                const auto& attribute = it.Value();
                if (!Handle(TDataStd_ByteArray)::DownCast(attribute).IsNull()
                    && attribute->ID() != core3d::persistence::AuthoredFrameAttributeID()) return false;
            }
            OcctAuthoredFrameRecord record;
            if (Core3DReadAuthoredFrameOwner(document, label, record) == OcctAuthoredFrameReadState::Invalid)
                return false;
            return AddMultipliedWithinLimit(total, record.nativeBytes, 1U, maximumBytes);
        };
        const auto root = document->GetData()->Root();
        if (!validate(root)) return Standard_False;
        Standard_Size count = 0;
        for (TDF_ChildIterator it(root, Standard_True); it.More(); it.Next()) {
            if (++count > kMaximumGeometryDocumentLabels || !validate(it.Value())) return Standard_False;
        }
        nativeBytes = total; return Standard_True;
    } catch (...) { nativeBytes = 0; return Standard_False; }
}

namespace {
bool ValidateNormalTextureShape(const TopoDS_Shape& shape, Standard_Size* requiredNativeBytes) noexcept {
    if(requiredNativeBytes!=nullptr)*requiredNativeBytes=0;
    try {
        TopoDS_Face face; Handle(Poly_Triangulation) mesh;
        if (shape.IsNull() || !TriangleAtlasFace(shape, face, mesh)
            || !mesh->HasUVNodes() || !mesh->HasNormals()) return Standard_False;
        RWMesh_FaceIterator it(shape);
        if (!it.More() || !it.Face().IsSame(face) || !it.HasNormals() || !it.HasTexCoords()
            || it.NbNodes() != mesh->NbNodes() || it.NbTriangles() != mesh->NbTriangles()) return Standard_False;
        using namespace core3d::scene;
        std::vector<Vertex> vertices;
        std::vector<std::uint32_t> indices;
        vertices.reserve(it.NbNodes()); indices.reserve(3 * it.NbTriangles());
        // Use the same centered float geometry as immutable publication.
        gp_XYZ lower, upper;
        bool first = true;
        for (int n = it.NodeLower(); n <= it.NodeUpper(); ++n) {
            const gp_XYZ point = it.NodeTransformed(n).XYZ();
            if (!std::isfinite(point.X()) || !std::isfinite(point.Y()) || !std::isfinite(point.Z())) return Standard_False;
            if (first) { lower = upper = point; first = false; }
            else for (int c = 1; c <= 3; ++c) {
                lower.SetCoord(c, std::min(lower.Coord(c), point.Coord(c)));
                upper.SetCoord(c, std::max(upper.Coord(c), point.Coord(c)));
            }
        }
        const gp_XYZ origin = (lower + upper) * 0.5;
        for (int n = it.NodeLower(); n <= it.NodeUpper(); ++n) {
            const gp_XYZ point = it.NodeTransformed(n).XYZ() - origin;
            const gp_Dir normal = it.NormalTransformed(n);
            const gp_Pnt2d uv = it.NodeTexCoord(n);
            const double maximumFloat = std::numeric_limits<float>::max();
            for (const double value : {point.X(), point.Y(), point.Z(), uv.X(), uv.Y()})
                if (!std::isfinite(value) || std::abs(value) > maximumFloat) return Standard_False;
            vertices.push_back({float(point.X()), float(point.Y()), float(point.Z()),
                float(normal.X()), float(normal.Y()), float(normal.Z()), float(uv.X()), float(uv.Y())});
        }
        for (int t = it.ElemLower(); t <= it.ElemUpper(); ++t) {
            int nodes[3]; it.TriangleOriented(t).Get(nodes[0], nodes[1], nodes[2]);
            for (int n : nodes) {
                if (n < it.NodeLower() || n > it.NodeUpper()) return Standard_False;
                indices.push_back(static_cast<std::uint32_t>(n - it.NodeLower()));
            }
        }
        it.Next(); if (it.More()) return Standard_False;
        std::vector<Float4> frames;
        if (GenerateMikkCornerTangents(vertices, indices, true, frames) != TangentSpaceError::None) return Standard_False;
        if (requiredNativeBytes != nullptr) *requiredNativeBytes = indices.size() * 64;
        return Standard_True;
    } catch (...) { return Standard_False; }
}

} // namespace

Standard_Boolean Core3DValidateNormalTextureGeometry(
    const TDF_Label& label,Standard_Size* requiredNativeBytes) noexcept {
    if(requiredNativeBytes!=nullptr)*requiredNativeBytes=0;
    try {
        if(label.IsNull() || !XCAFDoc_ShapeTool::IsFree(label)
            || !XCAFDoc_ShapeTool::IsSimpleShape(label) || XCAFDoc_ShapeTool::IsReference(label))return Standard_False;
        return ValidateNormalTextureShape(XCAFDoc_ShapeTool::GetShape(label),requiredNativeBytes);
    } catch(...) {return Standard_False;}
}

Standard_Integer Core3DNormalTextureBasisForLabel(
    const Handle(TDocStd_Document)& document, const TDF_Label& label,
    Standard_Size* additionalNativeBytes) noexcept {
    if (additionalNativeBytes != nullptr) *additionalNativeBytes = 0;
    try {
        if (document.IsNull() || label.IsNull() || label.Data() != document->GetData()
            || !XCAFDoc_DocumentTool::CheckShapeTool(document->Main())) return 0;
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (ValidatedGeometryRepresentation(document, shapes, label) != OcctGeometryRepresentation::TriangleMesh)
            return 0;
        OcctAuthoredFrameRecord record;
        const auto state = Core3DReadAuthoredFrameOwner(document, label, record);
        if (state == OcctAuthoredFrameReadState::Invalid) return 0;
        if (state == OcctAuthoredFrameReadState::Authored) return 2;
        return Core3DValidateNormalTextureGeometry(label, additionalNativeBytes) ? 1 : 0;
    } catch (...) { if (additionalNativeBytes != nullptr) *additionalNativeBytes = 0; return 0; }
}

Standard_Boolean Core3DValidateNormalTextureBinding(
    const Handle(TDocStd_Document)& document, const TDF_Label& label,
    Standard_Size* additionalNativeBytes) noexcept {
    if (additionalNativeBytes != nullptr) *additionalNativeBytes = 0;
    Standard_Size bytes = 0;
    const auto basis = Core3DNormalTextureBasisForLabel(document, label, &bytes);
    if (basis == 0 || Core3DNormalTextureRecipeForLabel(label) != basis) return Standard_False;
    if (additionalNativeBytes != nullptr) *additionalNativeBytes = bytes;
    return Standard_True;
}

Standard_Boolean Core3DValidateOwnedFrameUsage(
    const Handle(TDocStd_Document)& document, Standard_Size& nativeBytes,
    Standard_Size maximumBytes) noexcept {
    nativeBytes = 0;
    try {
        Standard_Size total = 0;
        if (!Core3DValidateAuthoredFrameOwners(document,total,maximumBytes)) return Standard_False;
        auto validate = [&](const TDF_Label& label) {
            const auto recipe = Core3DNormalTextureRecipeForLabel(label);
            if (recipe < 0) return false;
            const auto material = XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
            const bool normal = !material.IsNull() && material->HasPbrMaterial()
                && !material->PbrMaterial().NormalTexture.IsNull();
            if (!normal) return recipe == 0;
            Handle(TDataStd_Integer) marker;
            const bool owned = label.FindAttribute(LocalPBRMaterialAttributeID(),marker)
                && !marker.IsNull() && marker->Get() == 1;
            OcctAuthoredFrameRecord frame;
            const auto state = Core3DReadAuthoredFrameOwner(document,label,frame);
            if (state == OcctAuthoredFrameReadState::Invalid) return false;
            // Preserve legacy imported materials without geometry-owned frames.
            // Supplied normal bindings must have explicit native recipe ownership.
            if (!owned && state == OcctAuthoredFrameReadState::Absent) return recipe == 0;
            Standard_Size additional = 0;
            return owned && Core3DValidateNormalTextureBinding(document,label,&additional)
                && AddMultipliedWithinLimit(total,additional,1U,maximumBytes);
        };
        const auto root=document->GetData()->Root();
        if (!validate(root)) return Standard_False;
        Standard_Size count=0;
        for (TDF_ChildIterator it(root,Standard_True);it.More();it.Next()) {
            if (++count>kMaximumGeometryDocumentLabels || !validate(it.Value())) return Standard_False;
        }
        nativeBytes=total;return Standard_True;
    } catch (...) { nativeBytes=0;return Standard_False; }
}

Standard_Boolean OcctDocument::SupportsNormalTextureGeometryForLabel(
    const TDF_Label& label) const noexcept {
    return [NSThread isMainThread] && HasNativeNormalTextureGeometry(label);
}

Standard_Boolean OcctDocument::HasNativeNormalTextureGeometry(
    const TDF_Label& label) const noexcept {
    try {
        return IsEditableFreeSimpleDefinitionLabel(label)
            && GeometryRepresentationForLabel(label) == OcctGeometryRepresentation::TriangleMesh
            && Core3DNormalTextureBasisForLabel(myOcafDoc, label) != 0;
    } catch (...) { return Standard_False; }
}

namespace {
// Copy policy compares local stored payload, not positions rounded for rendering.
// Strip only the outer placement; preserve child placement/orientation and all
// unused nodes. UV-only sources may have no normals and cannot use SYTG hashing.
bool SameStoredMeshCopyPayload(TopoDS_Shape source, TopoDS_Shape destination) {
    source.Location(TopLoc_Location()); destination.Location(TopLoc_Location());
    TopoDS_Face a, b; Handle(Poly_Triangulation) x, y;
    if (source.ShapeType() != destination.ShapeType()
        || source.Orientation() != destination.Orientation()
        || !TriangleAtlasFace(source, a, x) || !TriangleAtlasFace(destination, b, y)
        || a.Orientation() != b.Orientation() || !a.Location().IsEqual(b.Location())
        || x->NbNodes() != y->NbNodes() || x->NbTriangles() != y->NbTriangles()
        || x->HasNormals() != y->HasNormals() || x->HasUVNodes() != y->HasUVNodes()) return false;
    const auto same = [](const auto left, const auto right) {
        return std::memcmp(&left, &right, sizeof(left)) == 0;
    };
    for (int n = 1; n <= x->NbNodes(); ++n) {
        const auto p = x->Node(n), q = y->Node(n);
        for (int c = 1; c <= 3; ++c) if (!same(p.Coord(c), q.Coord(c))) return false;
        if (x->HasUVNodes()) {
            const auto u = x->UVNode(n), v = y->UVNode(n);
            if (!same(u.X(), v.X()) || !same(u.Y(), v.Y())) return false;
        }
        if (x->HasNormals()) {
            gp_Vec3f u, v; x->Normal(n, u); y->Normal(n, v);
            for (int c = 0; c < 3; ++c) if (!same(u[c], v[c])) return false;
        }
    }
    for (int t = 1; t <= x->NbTriangles(); ++t) {
        int u[3], v[3]; x->Triangle(t).Get(u[0], u[1], u[2]); y->Triangle(t).Get(v[0], v[1], v[2]);
        for (int c = 0; c < 3; ++c) if (u[c] != v[c]) return false;
    }
    return true;
}
}

Standard_Boolean OcctDocument::CopyGeometryOwnedMeshMetadata(
    const TDF_Label& source, const TDF_Label& destination) {
    if (![NSThread isMainThread]) return Standard_False;
    try {
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || source.IsNull() || destination.IsNull() || source.IsEqual(destination)) return Standard_False;
        OcctObjectTransformState before, target;
        if (!CaptureObjectTransformStateForLabel(source, before)
            || !CaptureObjectTransformStateForLabel(destination, target)
            || before.resolvedRepresentation != target.resolvedRepresentation) return Standard_False;
        OcctAuthoredFrameRecord frame, previousFrame;
        if (Core3DReadAuthoredFrameOwner(myOcafDoc, source, frame) == OcctAuthoredFrameReadState::Invalid
            || Core3DReadAuthoredFrameOwner(myOcafDoc, destination, previousFrame) == OcctAuthoredFrameReadState::Invalid)
            return Standard_False;
        if (before.meshUVAtlasVersion == 0 && target.meshUVAtlasVersion == 0
            && frame.archive.empty() && previousFrame.archive.empty()) return Standard_True;
        // Clearing a destination's metadata is still an exact-copy operation.
        // An unannotated but different source cannot erase another mesh's basis.
        if (!SameStoredMeshCopyPayload(before.shape, target.shape)) return Standard_False;
        if (before.meshUVAtlasVersion != 0 || !frame.archive.empty()) {
            if (before.meshUVAtlasVersion != 0) {
                TopoDS_Face face; Handle(Poly_Triangulation) mesh;
                if (!TriangleAtlasFace(before.shape, face, mesh) || !mesh->HasUVNodes()) return Standard_False;
                const int originals = mesh->NbNodes() - 3 * mesh->NbTriangles();
                if (originals <= 0 || originals > 12288
                    || (before.meshUVAtlasVersion == 2 && before.meshUVAtlasSettings[2] != originals)) return Standard_False;
                for (int t = 1; t <= mesh->NbTriangles(); ++t) {
                    int ids[3]; mesh->Triangle(t).Get(ids[0], ids[1], ids[2]);
                    for (int c = 0; c < 3; ++c) if (ids[c] != originals + (t - 1) * 3 + c + 1) return Standard_False;
                }
                if (before.meshUVAtlasVersion == 2) {
                    OcctMeshUVAtlasPreview preview;
                    if (!CaptureMeshUVAtlasPreview(source, preview)) return Standard_False;
                }
            }
        }
        // Charge geometry-owned records even when hidden or mapless. Project
        // the replacement owner before writes without changing a bound recipe.
        if (!frame.archive.empty() || !previousFrame.archive.empty()) {
            Standard_Size residentBytes = 0;
            if (!Core3DValidateAuthoredFrameOwners(myOcafDoc, residentBytes)
                || previousFrame.nativeBytes > residentBytes) return Standard_False;
            residentBytes -= previousFrame.nativeBytes;
            if (!AddMultipliedWithinLimit(residentBytes, frame.nativeBytes, 1U, 64U * 1024U * 1024U)) return Standard_False;
            TDF_LabelSequence roots;
            const auto shapes = XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
            shapes->GetFreeShapes(roots);
            if (roots.Length() < 0 || roots.Length() > 50000) return Standard_False;
            for (int i = 1; i <= roots.Length(); ++i) {
                const auto& label = roots.Value(i);
                XCAFDoc_VisMaterialPBR material;
                if (!TryPBRMaterialForLabel(label, material) || material.NormalTexture.IsNull()) continue;
                OcctAuthoredFrameRecord boundFrame;
                const auto state = Core3DReadAuthoredFrameOwner(myOcafDoc, label, boundFrame);
                Standard_Size bytes = 0;
                if (state == OcctAuthoredFrameReadState::Invalid) return Standard_False;
                const bool projectedSupplied = label.IsEqual(destination)
                    ? !frame.archive.empty() : state == OcctAuthoredFrameReadState::Authored;
                if (Core3DNormalTextureRecipeForLabel(label) != (projectedSupplied ? 2 : 1)
                    || (!projectedSupplied && !Core3DValidateNormalTextureGeometry(label, &bytes))
                    || !AddMultipliedWithinLimit(residentBytes, bytes, 1U, 64U * 1024U * 1024U)) return Standard_False;
            }
        }
        // Validation is complete before any writes. Caller owns command/abort.
        if (frame.archive.empty()) destination.ForgetAttribute(core3d::persistence::AuthoredFrameAttributeID());
        else {
            const auto bytes = TDataStd_ByteArray::Set(destination, core3d::persistence::AuthoredFrameAttributeID(),
                0, Standard_Integer(frame.archive.size()) - 1, Standard_False);
            for (Standard_Size i = 0; i < frame.archive.size(); ++i) bytes->SetValue(Standard_Integer(i), frame.archive[i]);
        }
        if (before.meshUVAtlasVersion == 0) destination.ForgetAttribute(MeshUVAtlasAttributeID());
        else TDataStd_Integer::Set(destination, MeshUVAtlasAttributeID(), before.meshUVAtlasVersion);
        for (int i = 0; i < 3; ++i) {
            if (before.meshUVAtlasVersion == 2) TDataStd_Integer::Set(destination, MeshUVAtlasSettingsAttributeID(i), before.meshUVAtlasSettings[i]);
            else destination.ForgetAttribute(MeshUVAtlasSettingsAttributeID(i));
        }
        return Standard_True;
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctDocument::CaptureMeshUVAtlasPreview(
    const TDF_Label& label, OcctMeshUVAtlasPreview& preview) const noexcept {
    preview={};
    if (![NSThread isMainThread]) return Standard_False;
    try {
        OcctObjectTransformState source;
        if (!CaptureObjectTransformStateForLabel(label,source) || source.meshUVAtlasVersion!=2) return Standard_False;
        TopoDS_Face face; Handle(Poly_Triangulation) mesh;
        if (!TriangleAtlasFace(source.shape,face,mesh) || !mesh->HasUVNodes()
            || mesh->NbNodes()-3*mesh->NbTriangles()!=source.meshUVAtlasSettings[2]) return Standard_False;
        const int count=mesh->NbTriangles(), originals=source.meshUVAtlasSettings[2];
        using Corner=std::array<double,5>;
        std::map<std::pair<Corner,Corner>,std::vector<int>> edges;
        std::vector<int> parents(count);std::iota(parents.begin(),parents.end(),0);
        auto root=[&](int i) { while(parents[i]!=i) { parents[i]=parents[parents[i]];i=parents[i]; } return i; };
        OcctMeshUVAtlasPreview result;result.triangleUVs.reserve(count*6);
        for(int triangle=0;triangle<count;++triangle) {
            int ids[3];mesh->Triangle(triangle+1).Get(ids[0],ids[1],ids[2]);
            std::array<Corner,3> corners;
            for(int k=0;k<3;++k) {
                if(ids[k]!=originals+triangle*3+1+k) return Standard_False;
                const auto& point=mesh->Node(ids[k]);const auto& uv=mesh->UVNode(ids[k]);
                corners[k]={point.X(),point.Y(),point.Z(),uv.X(),uv.Y()};
                for(double value:corners[k]) if(!std::isfinite(value)) return Standard_False;
                if(uv.X()<0 || uv.X()>1 || uv.Y()<0 || uv.Y()>1) return Standard_False;
                result.triangleUVs.push_back(uv.X());result.triangleUVs.push_back(uv.Y());
            }
            const auto& a=corners[0];const auto& b=corners[1];const auto& c=corners[2];
            const double area=std::abs((b[3]-a[3])*(c[4]-a[4])-(b[4]-a[4])*(c[3]-a[3]))*0.5;
            if(!std::isfinite(area) || area<=0) return Standard_False;
            result.occupancy+=area;
            for(int k=0;k<3;++k) {
                auto first=corners[k],second=corners[(k+1)%3];if(second<first)std::swap(first,second);
                auto& uses=edges[{first,second}];uses.push_back(triangle);
                if(uses.size()>2) return Standard_False;
                if(uses.size()==2) parents[root(triangle)]=root(uses[0]);
            }
        }
        for(int i=0;i<count;++i) if(root(i)==i) ++result.chartCount;
        if(!std::isfinite(result.occupancy) || result.occupancy>1+1.e-10) return Standard_False;
        result.authoredResolution=source.meshUVAtlasSettings[0];
        result.authoredGutterPixels=source.meshUVAtlasSettings[1];
        preview=std::move(result);return Standard_True;
    } catch(...) { preview={};return Standard_False; }
}

Standard_Boolean OcctDocument::PrepareTriangleUVAtlas(
    const TDF_Label& label, TopoDS_Shape& candidate, const OcctMeshUVAtlasOptions& options, OcctMeshUVAtlasPreview* preview) const noexcept {
    candidate.Nullify();
    if (preview) *preview = {};
    if (![NSThread isMainThread]) { return Standard_False; }
    try {
        const shapeyard::uv::Settings settings{options.resolution, options.gutterPixels};
        if ((options.version != 1 && options.version != 2)
            || (options.version == 1 && (options.resolution != 0 || options.gutterPixels != 0))
            || (options.version == 2 && !settings.valid())) { return Standard_False; }
        OcctObjectTransformState source;
        if (!CaptureObjectTransformStateForLabel(label, source)
            || source.resolvedRepresentation != OcctGeometryRepresentation::TriangleMesh
            || (options.version == 1 && source.meshUVAtlasVersion != 0)) { return Standard_False; }
        if (!core3d::profile::HasOnlyMetadataSubshapes(myOcafDoc, label)) return Standard_False;
        // Replacing UVs under an image would silently change its appearance.
        TDF_Label materialLabel;
        XCAFDoc_VisMaterialTool::GetShapeMaterial(label, materialLabel);
        if (!materialLabel.IsNull()) {
            const auto materials = XCAFDoc_DocumentTool::VisMaterialTool(myOcafDoc->Main());
            const auto material = materials->GetMaterial(materialLabel);
            if (material.IsNull()) { return Standard_False; }
            if (material->HasCommonMaterial() && !material->CommonMaterial().DiffuseTexture.IsNull()) { return Standard_False; }
            if (material->HasPbrMaterial()) {
                const auto& pbr = material->PbrMaterial();
                if (!pbr.BaseColorTexture.IsNull() || !pbr.EmissiveTexture.IsNull()
                    || !pbr.NormalTexture.IsNull() || !pbr.MetallicRoughnessTexture.IsNull()
                    || !pbr.OcclusionTexture.IsNull()) { return Standard_False; }
            }
        }
        TopoDS_Face face; Handle(Poly_Triangulation) mesh;
        if (!TriangleAtlasFace(source.shape, face, mesh)) { return Standard_False; }
        const int count = mesh->NbTriangles();
        int originals = mesh->NbNodes();
        if (source.meshUVAtlasVersion != 0) {
            originals -= 3 * count;
            if (!mesh->HasUVNodes() || originals <= 0
                || (source.meshUVAtlasVersion == 2 && source.meshUVAtlasSettings[2] != originals)) { return Standard_False; }
            // Owned prefix stays fixed through regeneration, including legacy v1.
            for (int i=1;i<=count;++i) {
                int ids[3]; mesh->Triangle(i).Get(ids[0],ids[1],ids[2]);
                for (int k=0;k<3;++k) if (ids[k] != originals+(i-1)*3+1+k) { return Standard_False; }
            }
        }
        if (originals <= 0 || originals > 12288) { return Standard_False; }
        shapeyard::uv::Atlas coherent;
        if (options.version == 2) {
            std::vector<shapeyard::uv::Triangle> triangles(count);
            for (int i=1;i<=count;++i) {
                int ids[3]; mesh->Triangle(i).Get(ids[0],ids[1],ids[2]);
                for(int k=0;k<3;++k) {
                    if(ids[k]<1 || ids[k]>mesh->NbNodes()) return Standard_False;
                    const auto p=mesh->Node(ids[k]);
                    triangles[i-1].points[k]={p.X(),p.Y(),p.Z()};
                }
            }
            if (!shapeyard::uv::generate(triangles, settings, coherent)) { return Standard_False; }
        }
        const int columns = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(count))));
        const double cell = 1.0 / columns, padding = cell * 0.04;
        Handle(Poly_Triangulation) atlas = new Poly_Triangulation(originals + 3 * count, count, true, mesh->HasNormals());
        atlas->Deflection(mesh->Deflection());
        // Preserve even unused stored nodes and their exact normal values.
        for (int node = 1; node <= originals; ++node) {
            const auto point = mesh->Node(node);
            for (int axis = 1; axis <= 3; ++axis) {
                if (!std::isfinite(point.Coord(axis)) || std::abs(point.Coord(axis)) > 1.0e6) { return Standard_False; }
            }
            atlas->SetNode(node, point); atlas->SetUVNode(node, gp_Pnt2d(0, 0));
            if (mesh->HasNormals()) { gp_Vec3f normal; mesh->Normal(node, normal); atlas->SetNormal(node, normal); }
        }
        for (int triangle = 1; triangle <= count; ++triangle) {
            int ids[3]; mesh->Triangle(triangle).Get(ids[0], ids[1], ids[2]);
            for (int id : ids) { if (id < 1 || id > mesh->NbNodes()) { return Standard_False; } }
            const gp_Vec edge(mesh->Node(ids[0]), mesh->Node(ids[1]));
            const gp_Vec other(mesh->Node(ids[0]), mesh->Node(ids[2]));
            const double length = edge.Magnitude();
            if (!std::isfinite(length) || length <= 1.0e-12) { return Standard_False; }
            const double x = edge.Dot(other) / length;
            const double y = edge.Crossed(other).Magnitude() / length;
            const double minX = std::min(0.0, x), maxX = std::max(length, x);
            const double extent = std::max(maxX - minX, y);
            if (!std::isfinite(x) || !std::isfinite(y) || y <= length * 1.0e-12
                || !std::isfinite(extent) || extent <= 0) { return Standard_False; }
            const double scale = (cell - 2 * padding) / extent;
            const double originU = ((triangle - 1) % columns) * cell + padding;
            const double originV = ((triangle - 1) / columns) * cell + padding;
            const double xs[3] = {0, length, x}, ys[3] = {0, 0, y};
            const int first = originals + (triangle - 1) * 3 + 1;
            for (int corner = 0; corner < 3; ++corner) {
                atlas->SetNode(first + corner, mesh->Node(ids[corner]));
                if (options.version == 2) {
                    const auto& uv = coherent.corners[triangle-1][corner];
                    atlas->SetUVNode(first + corner, gp_Pnt2d(uv[0],uv[1]));
                } else {
                    atlas->SetUVNode(first + corner, gp_Pnt2d(originU + (xs[corner] - minX) * scale, originV + ys[corner] * scale));
                }
                if (mesh->HasNormals()) { gp_Vec3f normal; mesh->Normal(ids[corner], normal); atlas->SetNormal(first + corner, normal); }
            }
            atlas->SetTriangle(triangle, Poly_Triangle(first, first + 1, first + 2));
        }
        BRepBuilderAPI_Copy copy(source.shape, Standard_True, Standard_True);
        TopoDS_Shape result = copy.Shape();
        TopoDS_Face copiedFace; Handle(Poly_Triangulation) copiedMesh;
        if (result.IsNull() || result.IsSame(source.shape)
            || !TriangleAtlasFace(result, copiedFace, copiedMesh) || copiedFace.IsSame(face)) { return Standard_False; }
        BRep_Builder builder; builder.UpdateFace(copiedFace, atlas);
        if (!GeometryClassMatchesRepresentation(ClassifyDefinitionGeometry(result, nullptr), OcctGeometryRepresentation::TriangleMesh)) { return Standard_False; }
        if (preview && options.version == 2) {
            OcctMeshUVAtlasPreview value;
            value.triangleUVs.reserve(coherent.corners.size()*6);
            for (const auto& triangle:coherent.corners) for (const auto& uv:triangle) {
                value.triangleUVs.push_back(uv[0]);value.triangleUVs.push_back(uv[1]);
            }
            value.chartCount=coherent.chartCount;value.occupancy=coherent.occupancy;
            *preview=std::move(value);
        }
        candidate = result;
        return Standard_True;
    } catch (...) { candidate.Nullify(); return Standard_False; }
}

namespace {
bool CaptureEditableMeshSource(const OcctDocument& document,const TDF_Label& label,
    OcctObjectTransformState& source,core3d::meshedit::NativeTopologyCapture& captured,
    int& prefix,Standard_Size& resident) noexcept {
    source={};captured={};prefix=0;resident=0;
    if(![NSThread isMainThread])return false;
    try {
        double unit=0;
        if (!document.CaptureObjectTransformStateForLabel(label,source)
            || source.resolvedRepresentation!=OcctGeometryRepresentation::TriangleMesh
            || source.authoredFramesPresent
            || !XCAFDoc_DocumentTool::GetLengthUnit(document.Document(),unit) || unit!=0.001)
            return Standard_False;
        const auto recipe=Core3DNormalTextureRecipeForLabel(label);
        if(recipe<0 || recipe>1)return Standard_False;
        XCAFDoc_VisMaterialPBR material;
        const bool hasNormal=document.TryPBRMaterialForLabel(label,material) && !material.NormalTexture.IsNull();
        if(hasNormal!=(recipe==1))return Standard_False;
        if(!Core3DValidateOwnedFrameUsage(document.Document(),resident))return Standard_False;
        std::atomic_bool cancelled{false};
        if(core3d::meshedit::CaptureNativeTopology(source.shape,captured,cancelled)
            !=core3d::meshedit::TopologyResult::Ready)return Standard_False;
        prefix=0;
        if(source.meshUVAtlasVersion!=0) {
            prefix=captured.sourceMesh->NbNodes()-3*captured.sourceMesh->NbTriangles();
            if(prefix<=0 || (source.meshUVAtlasVersion==2 && prefix!=source.meshUVAtlasSettings[2]))
                return Standard_False;
        } else if(captured.sourceMesh->HasUVNodes()) {
            // Imported arbitrary layouts require an explicit shading contract.
            return Standard_False;
        }
        return core3d::meshedit::HasFlatCornerLayout(captured,prefix);
    } catch(...) {source={};captured={};return false;}
}
} // namespace

Standard_Boolean OcctDocument::CanEditMeshVertices(const TDF_Label& label) const noexcept {
    OcctObjectTransformState source;core3d::meshedit::NativeTopologyCapture captured;
    int prefix=0;Standard_Size resident=0;
    return CaptureEditableMeshSource(*this,label,source,captured,prefix,resident);
}

// All public entry points run on the native owner thread. No writes here.
Standard_Boolean OcctDocument::PrepareMeshVertexMove(const TDF_Label& label,
    const std::vector<std::uint32_t>& vertices, const gp_Vec& worldDelta,
    TopoDS_Shape& candidate) const noexcept {
    candidate.Nullify();
    if (![NSThread isMainThread]) return Standard_False;
    try {
        OcctObjectTransformState source;core3d::meshedit::NativeTopologyCapture captured;
        int prefix=0;Standard_Size resident=0;
        if(!CaptureEditableMeshSource(*this,label,source,captured,prefix,resident))return Standard_False;
        const auto recipe=Core3DNormalTextureRecipeForLabel(label);
        std::atomic_bool cancelled{false};
        const gp_Trsf placed=source.transform*captured.meshLocation.Transformation();
        for(int a=1;a<=3;++a)
            if(!std::isfinite(worldDelta.Coord(a)) || std::abs(worldDelta.Coord(a))>1.e6)return Standard_False;
        const gp_Vec localDelta=worldDelta.Transformed(placed.Inverted());
        TopoDS_Shape result;
        if(core3d::meshedit::PrepareFlatVertexMove(captured,vertices,
            {localDelta.X(),localDelta.Y(),localDelta.Z()},prefix,result,cancelled)
            !=core3d::meshedit::TopologyResult::Ready)return Standard_False;
        core3d::meshedit::NativeTopologyCapture edited;
        if(core3d::meshedit::CaptureNativeTopology(result,edited,cancelled)
            !=core3d::meshedit::TopologyResult::Ready)return Standard_False;
        for(const auto& p:edited.storedNodes) {
            const auto world=gp_Pnt(p[0],p[1],p[2]).Transformed(placed);
            for(int a=1;a<=3;++a)
                if(!std::isfinite(world.Coord(a)) || std::abs(world.Coord(a))>1.e6)return Standard_False;
        }
        if(recipe==1) {
            Standard_Size beforeBytes=0,afterBytes=0;
            if(!Core3DValidateNormalTextureBinding(myOcafDoc,label,&beforeBytes)
                || !ValidateNormalTextureShape(result,&afterBytes) || beforeBytes>resident)
                return Standard_False;
            resident-=beforeBytes;
            if(!AddMultipliedWithinLimit(resident,afterBytes,1U,64U*1024U*1024U))return Standard_False;
        }
        candidate=result;return Standard_True;
    } catch(...) {candidate.Nullify();return Standard_False;}
}

Standard_Boolean OcctDocument::ValidateMeshVertexMove(const TDF_Label& label,
    const std::vector<std::uint32_t>& vertices,const gp_Vec& worldDelta,
    const TopoDS_Shape& candidate) const noexcept {
    try {
        TopoDS_Shape expected;
        if(!PrepareMeshVertexMove(label,vertices,worldDelta,expected))return Standard_False;
        std::atomic_bool cancelled{false};
        core3d::meshedit::NativeTopologyCapture a,b;
        if(core3d::meshedit::CaptureNativeTopology(expected,a,cancelled)!=core3d::meshedit::TopologyResult::Ready
            || core3d::meshedit::CaptureNativeTopology(candidate,b,cancelled)!=core3d::meshedit::TopologyResult::Ready)
            return Standard_False;
        return expected.ShapeType()==candidate.ShapeType() && expected.Orientation()==candidate.Orientation()
            && expected.Location().IsEqual(candidate.Location()) && a.face.Orientation()==b.face.Orientation()
            && a.meshLocation.IsEqual(b.meshLocation) && a.storedNodes==b.storedNodes
            && a.storedUVs==b.storedUVs && a.storedNormals==b.storedNormals && a.deflection==b.deflection
            && a.triangleNodeIDs==b.triangleNodeIDs;
    } catch(...) {return Standard_False;}
}

// Capture exact owner state separately from strict selectable topology.
namespace {
bool CaptureWindingRepairSource(const OcctDocument& document, const TDF_Label& label,
    OcctObjectTransformState& source, core3d::meshedit::NativeMeshStorageCapture& captured,
    int& prefix, Standard_Size& resident) noexcept {
    source={}; captured={}; prefix=0; resident=0;
    if (![NSThread isMainThread]) return false;
    try {
        double unit=0;
        if (!document.CaptureObjectTransformStateForLabel(label,source)
            || source.resolvedRepresentation!=OcctGeometryRepresentation::TriangleMesh
            || source.authoredFramesPresent
            || !XCAFDoc_DocumentTool::GetLengthUnit(document.Document(),unit) || unit!=0.001)
            return false;
        const auto recipe=Core3DNormalTextureRecipeForLabel(label);
        if (recipe<0 || recipe>1) return false;
        XCAFDoc_VisMaterialPBR material;
        const bool normal=document.TryPBRMaterialForLabel(label,material) && !material.NormalTexture.IsNull();
        if (normal!=(recipe==1) || !Core3DValidateOwnedFrameUsage(document.Document(),resident)) return false;
        std::atomic_bool cancelled{false};
        using namespace core3d::meshedit;
        if (CaptureNativeMeshStorage(source.shape,captured,cancelled)!=TopologyResult::Ready) return false;
        if (source.meshUVAtlasVersion!=0) {
            if (source.meshUVAtlasVersion!=1 && source.meshUVAtlasVersion!=2) return false;
            prefix=captured.sourceMesh->NbNodes()-3*captured.sourceMesh->NbTriangles();
            if (prefix<=0 || (source.meshUVAtlasVersion==2 && prefix!=source.meshUVAtlasSettings[2])) return false;
        } else if (captured.sourceMesh->HasUVNodes()) return false;
        // Raw inconsistent storage is never offered as selectable topology.
        // Validate each existing flat corner normal independently, then the
        // winding planner validates cross-triangle orientability separately.
        NativeTopologyCapture flat;
        static_cast<NativeMeshStorageCapture&>(flat)=captured;
        for (const auto& triangle:captured.storedTriangles) {
            Topology isolated;
            if (Analyze({triangle},isolated,cancelled)!=TopologyResult::Ready) return false;
            flat.topology.unitNormals.push_back(isolated.unitNormals.front());
        }
        return HasFlatCornerLayout(flat,prefix);
    } catch (...) { source={}; captured={}; return false; }
}
}

OcctMeshWindingRepairResult OcctDocument::PrepareMeshWindingRepair(
    const TDF_Label& label, TopoDS_Shape& candidate) const noexcept {
    candidate.Nullify();
    if (![NSThread isMainThread]) return OcctMeshWindingRepairResult::Invalid;
    try {
        using namespace core3d::meshedit;
        OcctObjectTransformState source; NativeMeshStorageCapture captured;
        int prefix=0; Standard_Size resident=0;
        if (!CaptureWindingRepairSource(*this,label,source,captured,prefix,resident))
            return OcctMeshWindingRepairResult::Invalid;
        std::atomic_bool cancelled{false}; WindingPlan plan;
        if (PlanConsistentWinding(captured.storedTriangles,plan,cancelled)!=TopologyResult::Ready)
            return OcctMeshWindingRepairResult::Invalid;
        if (plan.reversedTriangles.empty()) return OcctMeshWindingRepairResult::Unchanged;
        TopoDS_Shape result;
        if (PrepareFlatWindingRepair(captured,prefix,result,plan,cancelled)!=TopologyResult::Ready)
            return OcctMeshWindingRepairResult::Invalid;
        NativeTopologyCapture edited;
        if (CaptureNativeTopology(result,edited,cancelled)!=TopologyResult::Ready)
            return OcctMeshWindingRepairResult::Invalid;
        const gp_Trsf placed=source.transform*edited.meshLocation.Transformation();
        for (const auto& point:edited.storedNodes) {
            const auto world=gp_Pnt(point[0],point[1],point[2]).Transformed(placed);
            for (int axis=1;axis<=3;++axis)
                if (!std::isfinite(world.Coord(axis)) || std::abs(world.Coord(axis))>1.e6)
                    return OcctMeshWindingRepairResult::Invalid;
        }
        if (Core3DNormalTextureRecipeForLabel(label)==1) {
            Standard_Size before=0,after=0;
            if (!Core3DValidateNormalTextureBinding(myOcafDoc,label,&before)
                || !ValidateNormalTextureShape(result,&after) || before>resident)
                return OcctMeshWindingRepairResult::Invalid;
            resident-=before;
            if (!AddMultipliedWithinLimit(resident,after,1U,64U*1024U*1024U))
                return OcctMeshWindingRepairResult::Invalid;
        }
        candidate=result;
        return OcctMeshWindingRepairResult::Prepared;
    } catch (...) { candidate.Nullify(); return OcctMeshWindingRepairResult::Invalid; }
}

Standard_Boolean OcctDocument::ValidateMeshWindingRepair(const TDF_Label& label,
    const TopoDS_Shape& candidate) const noexcept {
    if (![NSThread isMainThread]) return Standard_False;
    try {
        TopoDS_Shape expected;
        if (PrepareMeshWindingRepair(label,expected)!=OcctMeshWindingRepairResult::Prepared) return Standard_False;
        std::atomic_bool cancelled{false};
        core3d::meshedit::NativeTopologyCapture a,b;
        if (core3d::meshedit::CaptureNativeTopology(expected,a,cancelled)!=core3d::meshedit::TopologyResult::Ready
            || core3d::meshedit::CaptureNativeTopology(candidate,b,cancelled)!=core3d::meshedit::TopologyResult::Ready)
            return Standard_False;
        return expected.ShapeType()==candidate.ShapeType() && expected.Orientation()==candidate.Orientation()
            && expected.Location().IsEqual(candidate.Location()) && a.face.Orientation()==b.face.Orientation()
            && a.meshLocation.IsEqual(b.meshLocation) && a.storedNodes==b.storedNodes
            && a.storedUVs==b.storedUVs && a.storedNormals==b.storedNormals && a.deflection==b.deflection
            && a.triangleNodeIDs==b.triangleNodeIDs;
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctDocument::ValidateTriangleUVAtlas(
    const TDF_Label& label, const TopoDS_Shape& candidate, const OcctMeshUVAtlasOptions& options) const noexcept {
    try {
        TopoDS_Shape expected;
        if (!PrepareTriangleUVAtlas(label, expected, options)) { return Standard_False; }
        TopoDS_Face a, b; Handle(Poly_Triangulation) x, y;
        if (!TriangleAtlasFace(expected, a, x) || !TriangleAtlasFace(candidate, b, y)
            || candidate.ShapeType() != expected.ShapeType() || candidate.Orientation() != expected.Orientation()
            || !candidate.Location().IsEqual(expected.Location()) || a.Orientation() != b.Orientation()
            || !a.Location().IsEqual(b.Location()) || x->NbNodes() != y->NbNodes()
            || x->NbTriangles() != y->NbTriangles() || !y->HasUVNodes() || x->HasNormals() != y->HasNormals()) { return Standard_False; }
        for (int i = 1; i <= x->NbNodes(); ++i) {
            if (!x->Node(i).IsEqual(y->Node(i), 0.0) || !x->UVNode(i).IsEqual(y->UVNode(i), 0.0)) { return Standard_False; }
            if (x->HasNormals()) { gp_Vec3f nx, ny; x->Normal(i, nx); y->Normal(i, ny); if (nx != ny) { return Standard_False; } }
        }
        for (int i = 1; i <= x->NbTriangles(); ++i) {
            int ix[3], iy[3]; x->Triangle(i).Get(ix[0], ix[1], ix[2]); y->Triangle(i).Get(iy[0], iy[1], iy[2]);
            for (int j = 0; j < 3; ++j) { if (ix[j] != iy[j]) { return Standard_False; } }
        }
        return Standard_True;
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctDocument::MarkTriangleUVAtlas(const TDF_Label& label, const OcctMeshUVAtlasOptions& options) noexcept {
    try {
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || !IsEditableFreeSimpleDefinitionLabel(label)
            || GeometryRepresentationForLabel(label) != OcctGeometryRepresentation::TriangleMesh) { return Standard_False; }
        if (options.version == 2) {
            if (!shapeyard::uv::Settings{options.resolution,options.gutterPixels}.valid()) return Standard_False;
            TopoDS_Face face; Handle(Poly_Triangulation) mesh;
            if (!TriangleAtlasFace(XCAFDoc_ShapeTool::GetShape(label),face,mesh)) return Standard_False;
            const int originals=mesh->NbNodes()-3*mesh->NbTriangles();
            if (originals<=0 || originals>12288) return Standard_False;
            TDataStd_Integer::Set(label,MeshUVAtlasSettingsAttributeID(0),options.resolution);
            TDataStd_Integer::Set(label,MeshUVAtlasSettingsAttributeID(1),options.gutterPixels);
            TDataStd_Integer::Set(label,MeshUVAtlasSettingsAttributeID(2),originals);
        } else if (options.version != 1 || options.resolution != 0 || options.gutterPixels != 0
                   || label.IsAttribute(MeshUVAtlasSettingsAttributeID(0))
                   || label.IsAttribute(MeshUVAtlasSettingsAttributeID(1))
                   || label.IsAttribute(MeshUVAtlasSettingsAttributeID(2))) { return Standard_False; }
        TDataStd_Integer::Set(label, MeshUVAtlasAttributeID(), options.version);
        return Standard_True;
    } catch (...) { return Standard_False; }
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
    return AddShape(
        object, OcctGeometryRepresentation::BRep);
}

TDF_Label OcctDocument::AddShape(
    Handle(AIS_InteractiveObject) object,
    const OcctGeometryRepresentation representation) {
    Handle(AIS_Shape) aisShape = Handle(AIS_Shape)::DownCast(object);
    return AddShape(aisShape, representation);
}

TDF_Label OcctDocument::AddShape(Handle(AIS_Shape) aisShape) {
	return AddShape(
	    aisShape, OcctGeometryRepresentation::BRep);
}

TDF_Label OcctDocument::AddShape(
    Handle(AIS_Shape) aisShape,
    const OcctGeometryRepresentation representation) {
	if (myOcafDoc.IsNull()
	    || !myOcafDoc->HasOpenCommand()
	    || aisShape.IsNull()
	    || aisShape->Shape().IsNull()
	    || (representation != OcctGeometryRepresentation::BRep
	        && representation
	            != OcctGeometryRepresentation::TriangleMesh)) {
	    return TDF_Label();
	}
	Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
	if (shapeTool.IsNull()) {
	    return TDF_Label();
	}
	TDF_Label label = shapeTool->NewShape();
	shapeTool->SetShape(label, aisShape->Shape());
	if (!AssignNewIdentifier(label, EntityIdentifierAttributeID())
	    || !AssignNewIdentifier(label, DefinitionIdentifierAttributeID())
	    || !SetGeometryRepresentationForLabel(
	        label, representation)) {
	    return TDF_Label();
	}
	if (!SaveObjectTransform(label, aisShape)) {
	    return TDF_Label();
	}
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
    const OcctGeometryRepresentation previousRepresentation =
        GeometryRepresentationForLabel(label);
    if (previousRepresentation == OcctGeometryRepresentation::Invalid) {
        return Standard_False;
    }
    const OcctGeometryRepresentation resolvedRepresentation =
        previousRepresentation
            == OcctGeometryRepresentation::LegacyUnknown
        ? OcctGeometryRepresentation::BRep
        : previousRepresentation;
    if (!GeometryClassMatchesRepresentation(
            ClassifyDefinitionGeometry(aisShape->Shape(), nullptr),
            resolvedRepresentation)) {
        return Standard_False;
    }
    bool hadRepresentationMarker = false;
    OcctGeometryRepresentation storedRepresentation =
        OcctGeometryRepresentation::Invalid;
    if (!ReadGeometryRepresentation(
            label, hadRepresentationMarker, storedRepresentation)) {
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
            if (hadRepresentationMarker) {
                TDataStd_Integer::Set(
                    label,
                    GeometryRepresentationAttributeID(),
                    static_cast<Standard_Integer>(storedRepresentation));
            } else {
                label.ForgetAttribute(
                    GeometryRepresentationAttributeID());
            }
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
        if (previousRepresentation
                == OcctGeometryRepresentation::LegacyUnknown
            && !WriteGeometryRepresentationMarker(
                label, OcctGeometryRepresentation::BRep)) {
            restorePrevious();
            return Standard_False;
        }
        if (!SaveObjectTransform(label, aisShape)) {
            restorePrevious();
            return Standard_False;
        }
        return Standard_True;
    } catch (...) {
        restorePrevious();
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::SaveObjectTransform(
    const TDF_Label& label, const Handle(AIS_Shape) anAis)
{
    if (![NSThread isMainThread]) { return Standard_False; }
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
            || label.IsNull() || label.Data() != myOcafDoc->GetData()
            || anAis.IsNull()
            || !IsEditableFreeSimpleDefinitionLabel(label)) {
            return Standard_False;
        }
        const gp_Trsf transform = anAis->LocalTransformation();
        const gp_Quaternion rotation = transform.GetRotation();
        const Standard_Real values[8] = {
            transform.TranslationPart().X(),
            transform.TranslationPart().Y(),
            transform.TranslationPart().Z(),
            rotation.X(), rotation.Y(), rotation.Z(), rotation.W(),
            transform.ScaleFactor(),
        };
        // Validate the whole candidate before changing any attribute. The
        // caller retains transaction ownership if staging later fails.
        for (const Standard_Real value : values) {
            if (!std::isfinite(value)) {
                return Standard_False;
            }
        }
        for (Standard_Integer axis = 0; axis < 3; ++axis) {
            if (std::abs(values[axis])
                > core3d::limits::kMaximumModelCoordinateMagnitude) {
                return Standard_False;
            }
        }
        if (std::abs(values[7])
                <= std::numeric_limits<Standard_Real>::epsilon()
            || !EnsureGeometryRepresentationForMutation(label)) {
            return Standard_False;
        }
        for (Standard_Integer index = 0; index < 8; ++index) {
            TDataStd_Real::Set(label.FindChild(index + 1), values[index]);
        }
        OcctObjectTransformState stored;
        if (!CaptureObjectTransformStateForLabel(label, stored)) {
            return Standard_False;
        }
        for (Standard_Integer index = 0; index < 8; ++index) {
            if (!stored.present[index] || stored.scalars[index] != values[index]) {
                return Standard_False;
            }
        }
        return Standard_True;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::SetObjectPositionComponentForLabel(
    const TDF_Label& label,
    const Standard_Integer axis,
    const Standard_Real value)
{
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || label.IsNull() || label.Data() != myOcafDoc->GetData()
        || axis < 0 || axis > 2 || !std::isfinite(value)
        || std::abs(value)
            > core3d::limits::kMaximumModelCoordinateMagnitude
        || !IsEditableFreeSimpleDefinitionLabel(label)) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        const OcctGeometryRepresentation representation =
            StoredGeometryRepresentationForLabel(label);
        const bool isSupportedRepresentation =
            representation == OcctGeometryRepresentation::BRep
            || representation
                == OcctGeometryRepresentation::TriangleMesh
            || (representation
                    == OcctGeometryRepresentation::LegacyUnknown
                && GeometryRepresentationForLabel(label)
                    == OcctGeometryRepresentation::BRep);
        if (!isSupportedRepresentation) {
            return Standard_False;
        }
        const TDF_Label child = label.FindChild(axis + 1);
        if (child.IsNull()) {
            return Standard_False;
        }
        TDataStd_Real::Set(child, value);
        Handle(TDataStd_Real) stored;
        return child.FindAttribute(TDataStd_Real::GetID(), stored)
            && !stored.IsNull()
            && stored->Get() == value;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::SetReferenceAxisForLabel(
    const TDF_Label& label,
    const OcctReferenceAxis& axis)
{
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || label.IsNull() || label.Data() != myOcafDoc->GetData()
        || !IsEditableFreeSimpleDefinitionLabel(label)
        || GeometryRepresentationForLabel(label)
            == OcctGeometryRepresentation::Invalid) {
        return Standard_False;
    }
    return WriteReferenceAxisRecord(label, axis)
        ? Standard_True : Standard_False;
}

Standard_Boolean OcctDocument::ResetReferenceAxisForLabel(
    const TDF_Label& label)
{
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || label.IsNull() || label.Data() != myOcafDoc->GetData()
        || !IsEditableFreeSimpleDefinitionLabel(label)
        || GeometryRepresentationForLabel(label)
            == OcctGeometryRepresentation::Invalid) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        for (const Standard_GUID* anId : ReferenceAxisAttributeIDs()) {
            if (anId != nullptr) {
                label.ForgetAttribute(*anId);
            }
        }
        OcctReferenceAxis aStored;
        return ReadReferenceAxisRecord(label, aStored)
                == OcctReferenceAxisReadState::ImplicitDefault
            && ReferenceAxesMatch(aStored, DefaultReferenceAxis());
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::CopyReferenceAxis(
    const TDF_Label& source,
    const TDF_Label& destination)
{
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || source.IsNull() || destination.IsNull()
        || source.Data() != myOcafDoc->GetData()
        || destination.Data() != myOcafDoc->GetData()
        || !IsEditableFreeSimpleDefinitionLabel(destination)) {
        return Standard_False;
    }
    OcctReferenceAxis anAxis;
    const OcctReferenceAxisReadState aState =
        ReadReferenceAxisForLabel(source, anAxis);
    if (aState == OcctReferenceAxisReadState::Invalid) {
        return Standard_False;
    }
    return aState == OcctReferenceAxisReadState::ImplicitDefault
        ? ResetReferenceAxisForLabel(destination)
        : SetReferenceAxisForLabel(destination, anAxis);
}

Standard_Boolean OcctDocument::CopyReferenceAxisThroughBakedTransform(
    const TDF_Label& source,
    const TDF_Label& destination,
    const gp_Trsf& sourceLocalToDestinationLocal)
{
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || source.IsNull() || destination.IsNull()
        || source.Data() != myOcafDoc->GetData()
        || destination.Data() != myOcafDoc->GetData()
        || !IsEditableFreeSimpleDefinitionLabel(destination)) {
        return Standard_False;
    }
    for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
        for (Standard_Integer aColumn = 1; aColumn <= 4; ++aColumn) {
            if (!std::isfinite(
                    sourceLocalToDestinationLocal.Value(aRow, aColumn))) {
                return Standard_False;
            }
        }
    }

    OcctReferenceAxis anAxis;
    const OcctReferenceAxisReadState aSourceState =
        ReadReferenceAxisForLabel(source, anAxis);
    if (aSourceState == OcctReferenceAxisReadState::Invalid) {
        return Standard_False;
    }
    try {
        OCC_CATCH_SIGNALS
        if (anAxis.pivotSpace == OcctReferenceSpace::Object) {
            anAxis.pivot.Transform(sourceLocalToDestinationLocal);
        }
        if (!IsFiniteBoundedReferencePoint(anAxis.pivot)) {
            return Standard_False;
        }
        if (anAxis.directionSpace == OcctReferenceSpace::Object) {
            gp_Vec aDirection(anAxis.direction);
            aDirection.Transform(sourceLocalToDestinationLocal);
            const Standard_Real aSquaredMagnitude =
                aDirection.SquareMagnitude();
            if (!std::isfinite(aDirection.X())
                || !std::isfinite(aDirection.Y())
                || !std::isfinite(aDirection.Z())
                || !std::isfinite(aSquaredMagnitude)
                || aSquaredMagnitude
                    <= std::numeric_limits<Standard_Real>::epsilon()) {
                return Standard_False;
            }
            anAxis.direction = gp_Dir(aDirection);
        }
        if (!IsFiniteReferenceDirection(anAxis.direction)) {
            return Standard_False;
        }
        if (aSourceState == OcctReferenceAxisReadState::ImplicitDefault
            && ReferenceAxesMatch(anAxis, DefaultReferenceAxis())) {
            return ResetReferenceAxisForLabel(destination);
        }
        return SetReferenceAxisForLabel(destination, anAxis);
    } catch (...) {
        return Standard_False;
    }
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
    if (!EnsureGeometryRepresentationForMutation(label)) {
        return;
    }
    TDataStd_Integer::Set(label.FindChild(11), name_of_material);
}

void OcctDocument::SaveObjectColor(const TDF_Label& label, const Quantity_NameOfColor name_of_color) {
    if (!EnsureGeometryRepresentationForMutation(label)) {
        return;
    }
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

    // Reserve the native per-corner derivative for the final free-object
    // material bindings, including hidden objects and unchanged normal maps.
    // 64 bytes includes every admitted source field plus a float4 tangent.
    const auto shapeTool = XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
    if (shapeTool.IsNull()) return Standard_False;
    TDF_LabelSequence roots; shapeTool->GetFreeShapes(roots);
    if (roots.Length() < 0 || roots.Length() > 50000) return Standard_False;
    Standard_Size normalBytes = 0;
    if (!Core3DValidateAuthoredFrameOwners(myOcafDoc, normalBytes)) return Standard_False;
    for (int index = 1; index <= roots.Length(); ++index) {
        const auto& root = roots.Value(index);
        const auto update = std::find_if(updates.begin(), updates.end(),
            [&](const auto& value) { return value.label.IsEqual(root); });
        XCAFDoc_VisMaterialPBR material;
        if (update != updates.end()) material = update->material;
        else if (!TryPBRMaterialForLabel(root, material)) continue;
        if (material.NormalTexture.IsNull()) continue;
        Standard_Size bytes = 0;
        const auto basis = Core3DNormalTextureBasisForLabel(myOcafDoc, root, &bytes);
        if (basis == 0 || (update == updates.end() && Core3DNormalTextureRecipeForLabel(root) != basis)
            || !AddMultipliedWithinLimit(normalBytes, bytes, 1U, 64U * 1024U * 1024U)) return Standard_False;
    }

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
            || (!material.NormalTexture.IsNull()
                && (!Core3DValidateNumericTexture(material.NormalTexture)
                    || !HasNativeNormalTextureGeometry(update.label)))
            || (!material.MetallicRoughnessTexture.IsNull()
                && !Core3DValidateNumericTexture(material.MetallicRoughnessTexture))
            || (!material.OcclusionTexture.IsNull()
                && !Core3DValidateNumericTexture(material.OcclusionTexture))
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
    for (const PreparedUpdate& update : preparedUpdates) {
        if (!EnsureGeometryRepresentationForMutation(update.label)) {
            return Standard_False;
        }
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
        if (!update.material->PbrMaterial().NormalTexture.IsNull()) {
            const auto basis = Core3DNormalTextureBasisForLabel(myOcafDoc, update.label);
            if (basis == 0) return Standard_False;
            TDataStd_Integer::Set(update.label, NormalTextureRecipeAttributeID(), basis);
        } else {
            update.label.ForgetAttribute(NormalTextureRecipeAttributeID());
        }

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
        || label.IsNull()
        || !EnsureGeometryRepresentationForMutation(label)) {
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
    label.ForgetAttribute(NormalTextureRecipeAttributeID());
    label.ForgetAttribute(AutoPromotedEmissiveFactorAttributeID());
    return Standard_True;
}

Standard_Boolean OcctDocument::CaptureScalarAppearanceForMeshCopy(
    const TDF_Label& label, OcctScalarAppearanceState& output) const noexcept {
    output={};
    if (![NSThread isMainThread]) return Standard_False;
    try {
        if (myOcafDoc.IsNull() || label.IsNull() || label.Data()!=myOcafDoc->GetData()
            || !IsEditableFreeSimpleDefinitionLabel(label)) return Standard_False;
        // Subshape styling needs a deliberate triangle/material mapping. The
        // first copy rejects these labels rather than flattening their styles.
        if (!core3d::profile::HasOnlyMetadataSubshapes(myOcafDoc, label)) return Standard_False;
        for (auto color:{XCAFDoc_ColorGen,XCAFDoc_ColorSurf,XCAFDoc_ColorCurv})
            if (label.IsAttribute(XCAFDoc::ColorRefGUID(color))) return Standard_False;
        if (label.IsAttribute(NormalTextureRecipeAttributeID())
            || label.IsAttribute(AutoPromotedEmissiveFactorAttributeID())) return Standard_False;
        OcctScalarAppearanceState state;
        for (int i=0;i<2;++i) {
            const auto child=label.FindChild(11+i,Standard_False);
            if (child.IsNull()) continue;
            Handle(TDF_Attribute) attribute;
            if (!child.FindAttribute(TDataStd_Integer::GetID(),attribute)) continue;
            const auto integer=Handle(TDataStd_Integer)::DownCast(attribute);
            if (integer.IsNull()) return Standard_False;
            state.legacyPresent[i]=true;state.legacyValues[i]=integer->Get();
        }
        Handle(TDF_Attribute) marker;
        if (label.FindAttribute(LocalPBRMaterialAttributeID(),marker)) {
            const auto integer=Handle(TDataStd_Integer)::DownCast(marker);
            if (integer.IsNull() || integer->Get()!=1) return Standard_False;
            state.localPBR=true;
        }
        const bool linked=XCAFDoc_VisMaterialTool::GetShapeMaterial(label,state.materialLabel);
        const auto material=XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
        if (linked != !material.IsNull() || linked != !state.materialLabel.IsNull()
            || (state.localPBR && (material.IsNull() || !material->HasPbrMaterial()))) return Standard_False;
        if (!material.IsNull()) {
            if (state.materialLabel.Data()!=myOcafDoc->GetData() || material->IsEmpty()) return Standard_False;
            const auto& p=material->PbrMaterial();const auto& c=material->CommonMaterial();
            // Reject even disabled-model texture handles: the source state is
            // captured completely and no payload aliases enter this contract.
            if (!p.BaseColorTexture.IsNull() || !p.MetallicRoughnessTexture.IsNull()
                || !p.NormalTexture.IsNull() || !p.OcclusionTexture.IsNull()
                || !p.EmissiveTexture.IsNull() || !c.DiffuseTexture.IsNull()) return Standard_False;
            auto& values=state.visualValues;
            values={double(material->FaceCulling()),double(material->AlphaMode()),material->AlphaCutOff(),
                double(p.IsDefined),double(c.IsDefined)};
            const auto rgb=[&values](const Quantity_Color& color) {
                values.push_back(color.Red());values.push_back(color.Green());values.push_back(color.Blue());
            };
            rgb(p.BaseColor.GetRGB());values.push_back(p.BaseColor.Alpha());
            for (int i=0;i<3;++i) values.push_back(p.EmissiveFactor[i]);
            values.push_back(p.Metallic);values.push_back(p.Roughness);values.push_back(p.RefractionIndex);
            rgb(c.AmbientColor);rgb(c.DiffuseColor);rgb(c.SpecularColor);rgb(c.EmissiveColor);
            values.push_back(c.Shininess);values.push_back(c.Transparency);
            for (double value:values) if (!std::isfinite(value)) return Standard_False;
        }
        output=std::move(state);return Standard_True;
    } catch (...) {output={};return Standard_False;}
}

Standard_Boolean OcctDocument::CopyObjectAppearance(
    const TDF_Label& source,
    const TDF_Label& destination) {
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || source.IsNull() || destination.IsNull()
        || GeometryRepresentationForLabel(source)
            == OcctGeometryRepresentation::Invalid
        || !EnsureGeometryRepresentationForMutation(destination)) {
        return Standard_False;
    }

    Handle(TDataStd_Integer) aLocalPBRMarker;
    const Standard_Boolean hasLocalPBR =
        source.FindAttribute(
            LocalPBRMaterialAttributeID(), aLocalPBRMarker)
        && !aLocalPBRMarker.IsNull()
        && aLocalPBRMarker->Get() == 1;
    const auto sourceVisual = XCAFDoc_VisMaterialTool::GetShapeMaterial(source);
    const bool hasOwnedNormal = hasLocalPBR && !sourceVisual.IsNull()
        && sourceVisual->HasPbrMaterial() && !sourceVisual->PbrMaterial().NormalTexture.IsNull();
    const auto normalRecipe = Core3DNormalTextureRecipeForLabel(source);
    if ((hasOwnedNormal && (!Core3DValidateNormalTextureBinding(myOcafDoc, source)
            || Core3DNormalTextureBasisForLabel(myOcafDoc, destination) != normalRecipe))
        || (!hasOwnedNormal && normalRecipe != 0)) return Standard_False;
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
        if (hasOwnedNormal) {
            TDataStd_Integer::Set(destination, NormalTextureRecipeAttributeID(), normalRecipe);
        } else {
            destination.ForgetAttribute(NormalTextureRecipeAttributeID());
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
        if (!pbr.NormalTexture.IsNull() && (!hasLocalPBR
            || !Core3DValidateNormalTextureBinding(myOcafDoc, label))) return Standard_False;
        const Handle(Image_Texture)& base = pbr.BaseColorTexture;
        const Handle(Image_Texture) common = material->HasCommonMaterial()
            ? material->CommonMaterial().DiffuseTexture
            : Handle(Image_Texture)();
        if (!base.IsNull() || !pbr.EmissiveTexture.IsNull()
            || !pbr.MetallicRoughnessTexture.IsNull() || !pbr.OcclusionTexture.IsNull()
            || !pbr.NormalTexture.IsNull()) {
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
    return SupportsMaterialTextureEditingForLabel(label, OcctMaterialTextureSlot::BaseColor);
}

Standard_Boolean OcctDocument::SupportsEmissiveTextureEditingForLabel(
    const TDF_Label& label) const {
    return SupportsMaterialTextureEditingForLabel(label, OcctMaterialTextureSlot::Emissive);
}

Standard_Boolean OcctDocument::SupportsMaterialTextureEditingForLabel(
    const TDF_Label& label, OcctMaterialTextureSlot slot) const {
    if (label.IsNull() || (slot == OcctMaterialTextureSlot::Normal
        && !SupportsNormalTextureGeometryForLabel(label))) return Standard_False;
    const Handle(XCAFDoc_VisMaterial) material = XCAFDoc_VisMaterialTool::GetShapeMaterial(label);
    if (material.IsNull()) return Standard_True;
    if (!material->HasPbrMaterial() && !material->HasCommonMaterial()) return Standard_False;
    XCAFDoc_VisMaterialPBR pbr = material->HasPbrMaterial()
        ? material->PbrMaterial() : material->ConvertToPbrMaterial();
    const Handle(Image_Texture) common = material->HasCommonMaterial()
        ? material->CommonMaterial().DiffuseTexture : Handle(Image_Texture)();
    if (!pbr.BaseColorTexture.IsNull() && !common.IsNull()
        && !Core3DTexturesMatch(pbr.BaseColorTexture, common)) return Standard_False;
    Handle(TDataStd_Integer) marker;
    const bool owned = label.FindAttribute(LocalPBRMaterialAttributeID(), marker)
        && !marker.IsNull() && marker->Get() == 1;
    if (owned && !pbr.NormalTexture.IsNull()
        && !Core3DValidateNormalTextureBinding(myOcafDoc, label)) return Standard_False;
    if (slot != OcctMaterialTextureSlot::BaseColor
        && (!pbr.BaseColorTexture.IsNull() || !common.IsNull())
        && (!owned || pbr.BaseColorTexture.IsNull() || common.IsNull())) return Standard_False;
    for (const auto other : {OcctMaterialTextureSlot::BaseColor, OcctMaterialTextureSlot::Emissive,
                            OcctMaterialTextureSlot::MetallicRoughness, OcctMaterialTextureSlot::Occlusion, OcctMaterialTextureSlot::Normal}) {
        if (other == slot) continue;
        const Handle(Image_Texture)& texture = Core3DMaterialTexture(pbr, other);
        // Replacing one imported map is explicit. Preserving a different
        // imported map must never silently change its ownership.
        if (!texture.IsNull() && !owned) return Standard_False;
    }
    return Standard_True;
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
        || label.IsNull()
        || !EnsureGeometryRepresentationForMutation(label)) {
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
        Standard_Size frameBytes = 0;
        if (wasRejected || status != PCDM_RS_OK || candidate.IsNull()
            || !ValidateGeometryRepresentations(candidate)
            || !Core3DValidateOwnedFrameUsage(candidate,frameBytes)) {
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
    if (myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || !ValidateGeometryRepresentations()) {
        return Standard_False;
    }
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (myOcafDoc->Main());
    if (shapeTool.IsNull()) {
        return Standard_False;
    }
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
        if (XCAFDoc_ShapeTool::IsSimpleShape(aLabel)
            && !XCAFDoc_ShapeTool::IsAssembly(aLabel)
            && !EnsureGeometryRepresentationForMutation(aLabel)) {
            return Standard_False;
        }
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

Standard_Boolean OcctDocument::TryObjectTransformForLabel(
    const TDF_Label& label,
    gp_Trsf& transform) const
{
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull()
            || !XCAFDoc_DocumentTool::CheckShapeTool(
                myOcafDoc->Main())) {
            return Standard_False;
        }
        const Handle(XCAFDoc_ShapeTool) aShapeTool =
            XCAFDoc_DocumentTool::ShapeTool(myOcafDoc->Main());
        return TryReadObjectTransform(
            myOcafDoc, aShapeTool, label, transform)
            ? Standard_True : Standard_False;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctObjectTransformState::IsEqual(
    const OcctObjectTransformState& other) const noexcept
{
    if (![NSThread isMainThread]) { return Standard_False; }
    try {
        for (Standard_Integer row = 1; row <= 3; ++row) {
            for (Standard_Integer column = 1; column <= 4; ++column) {
                const Standard_Real value = transform.Value(row, column);
                if (!std::isfinite(value) || value != other.transform.Value(row, column)) {
                    return Standard_False;
                }
            }
        }
        return !label.IsNull() && !other.label.IsNull()
            && !documentData.IsNull() && documentData == other.documentData
            && label.IsEqual(other.label) && label.Data() == documentData
            && other.label.Data() == other.documentData
            && !shape.IsNull() && !other.shape.IsNull() && shape.IsEqual(other.shape)
            && !entityIdentifier.empty() && entityIdentifier == other.entityIdentifier
            && !definitionIdentifier.empty() && definitionIdentifier == other.definitionIdentifier
            && storedRepresentation != OcctGeometryRepresentation::Invalid
            && storedRepresentation == other.storedRepresentation
            && resolvedRepresentation != OcctGeometryRepresentation::Invalid
            && resolvedRepresentation == other.resolvedRepresentation
            && meshUVAtlasVersion == other.meshUVAtlasVersion
            && meshUVAtlasSettings == other.meshUVAtlasSettings
            && authoredFramesPresent == other.authoredFramesPresent
            && authoredFramesIdentity == other.authoredFramesIdentity
            && profile.IsEqual(other.profile)
            && present == other.present && scalars == other.scalars;
    } catch (...) {
        return Standard_False;
    }
}

Standard_Boolean OcctDocument::CaptureObjectTransformStateForLabel(
    const TDF_Label& label, OcctObjectTransformState& state) const noexcept
{
    state = OcctObjectTransformState();
    if (![NSThread isMainThread]) { return Standard_False; }
    try {
        OCC_CATCH_SIGNALS
        if (myOcafDoc.IsNull()
            || !XCAFDoc_DocumentTool::CheckShapeTool(myOcafDoc->Main())
            || !IsEditableFreeSimpleDefinitionLabel(label)) {
            return Standard_False;
        }
        OcctObjectTransformState captured;
        captured.label = label;
        captured.documentData = myOcafDoc->GetData();
        captured.shape = XCAFDoc_ShapeTool::GetShape(label);
        captured.entityIdentifier = EntityIdentifierForLabel(label);
        captured.definitionIdentifier = DefinitionIdentifierForLabel(label);
        captured.storedRepresentation = StoredGeometryRepresentationForLabel(label);
        captured.resolvedRepresentation = GeometryRepresentationForLabel(label);
        if (!core3d::profile::Read(myOcafDoc, label, captured.profile)) return Standard_False;
        OcctAuthoredFrameRecord frames;
        const auto frameState = Core3DReadAuthoredFrameOwner(myOcafDoc, label, frames);
        if (frameState == OcctAuthoredFrameReadState::Invalid) return Standard_False;
        captured.authoredFramesPresent = frameState == OcctAuthoredFrameReadState::Authored;
        captured.authoredFramesIdentity = frames.identity;
        Handle(TDF_Attribute) atlasAttribute;
        if (label.FindAttribute(MeshUVAtlasAttributeID(), atlasAttribute)) {
            const auto version = Handle(TDataStd_Integer)::DownCast(atlasAttribute);
            if (version.IsNull() || (version->Get() != 1 && version->Get() != 2)
                || captured.resolvedRepresentation != OcctGeometryRepresentation::TriangleMesh) { return Standard_False; }
            captured.meshUVAtlasVersion = version->Get();
        }
        for (int i=0;i<3;++i) {
            Handle(TDF_Attribute) attribute;
            const bool present=label.FindAttribute(MeshUVAtlasSettingsAttributeID(i),attribute);
            if (captured.meshUVAtlasVersion == 2) {
                const auto value=Handle(TDataStd_Integer)::DownCast(attribute);
                if (!present || value.IsNull()) return Standard_False;
                captured.meshUVAtlasSettings[i]=value->Get();
            } else if (present) { return Standard_False; }
        }
        if (captured.meshUVAtlasVersion == 2
            && (!shapeyard::uv::Settings{captured.meshUVAtlasSettings[0],captured.meshUVAtlasSettings[1]}.valid()
                || captured.meshUVAtlasSettings[2]<=0 || captured.meshUVAtlasSettings[2]>12288)) return Standard_False;
        if (captured.documentData.IsNull() || captured.shape.IsNull()
            || captured.entityIdentifier.empty() || captured.definitionIdentifier.empty()
            || captured.storedRepresentation == OcctGeometryRepresentation::Invalid
            || captured.resolvedRepresentation == OcctGeometryRepresentation::Invalid
            || !TryObjectTransformForLabel(label, captured.transform)) {
            return Standard_False;
        }
        for (Standard_Integer index = 0; index < 8; ++index) {
            const TDF_Label child = label.FindChild(index + 1, Standard_False);
            Handle(TDF_Attribute) attribute;
            if (!child.IsNull() && child.FindAttribute(TDataStd_Real::GetID(), attribute)) {
                const Handle(TDataStd_Real) scalar = Handle(TDataStd_Real)::DownCast(attribute);
                if (scalar.IsNull() || !std::isfinite(scalar->Get())) {
                    return Standard_False;
                }
                captured.present[index] = Standard_True;
                captured.scalars[index] = scalar->Get();
            }
        }
        state = std::move(captured);
        return Standard_True;
    } catch (...) {
        state = OcctObjectTransformState();
        return Standard_False;
    }
}

Standard_Boolean OcctObjectVisibilityState::HasSameObjectAndLayers(
    const OcctObjectVisibilityState& other) const noexcept {
    try {
        if (!object.IsEqual(other.object) || layerLinkPresent != other.layerLinkPresent
            || layers.size() != other.layers.size()
            || layerInvisibleAttributePresent != other.layerInvisibleAttributePresent) { return Standard_False; }
        for (std::size_t index = 0; index < layers.size(); ++index) {
            if (!layers[index].IsEqual(other.layers[index])) { return Standard_False; }
        }
        return Standard_True;
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctObjectVisibilityState::IsEqual(const OcctObjectVisibilityState& other) const noexcept {
    return invisibleAttributePresent == other.invisibleAttributePresent && HasSameObjectAndLayers(other);
}

Standard_Boolean OcctObjectVisibilityState::IsEffectivelyVisible() const noexcept {
    if (invisibleAttributePresent) { return Standard_False; }
    for (bool hidden : layerInvisibleAttributePresent) { if (hidden) { return Standard_False; } }
    return Standard_True;
}

Standard_Boolean OcctDocument::CaptureObjectVisibilityStateForLabel(
    const TDF_Label& label, OcctObjectVisibilityState& state) const noexcept {
    state = OcctObjectVisibilityState();
    if (![NSThread isMainThread]) { return Standard_False; }
    try {
        OcctObjectVisibilityState captured;
        if (!CaptureObjectNameStateForLabel(label, captured.object)) { return Standard_False; }
        const auto captureInvisible = [](const TDF_Label& target, bool& present) {
            Handle(TDF_Attribute) attribute;
            present = target.FindAttribute(XCAFDoc::InvisibleGUID(), attribute);
            return !present || !Handle(TDataStd_UAttribute)::DownCast(attribute).IsNull();
        };
        bool hidden = false;
        if (!captureInvisible(label, hidden)) { return Standard_False; }
        captured.invisibleAttributePresent = hidden;
        Handle(TDF_Attribute) association;
        if (label.FindAttribute(XCAFDoc::LayerRefGUID(), association)) {
            const auto graph = Handle(XCAFDoc_GraphNode)::DownCast(association);
            if (graph.IsNull() || graph->NbFathers() < 0 || graph->NbFathers() > 1024) { return Standard_False; }
            captured.layerLinkPresent = Standard_True;
            for (Standard_Integer index = 1; index <= graph->NbFathers(); ++index) {
                const auto father = graph->GetFather(index);
                if (father.IsNull()) { return Standard_False; }
                const auto layer = father->Label();
                if (layer.IsNull() || layer.Data() != captured.object.object.documentData
                    || !captureInvisible(layer, hidden)) { return Standard_False; }
                for (const auto& previous : captured.layers) {
                    if (previous.IsEqual(layer)) { return Standard_False; }
                }
                captured.layers.push_back(layer);
                captured.layerInvisibleAttributePresent.push_back(hidden);
            }
        }
        state = std::move(captured);
        return Standard_True;
    } catch (...) { state = OcctObjectVisibilityState(); return Standard_False; }
}

Standard_Boolean OcctDocument::SetObjectVisibilityForLabel(
    const TDF_Label& label, Standard_Boolean visible) noexcept {
    if (![NSThread isMainThread] || myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || !XCAFDoc_DocumentTool::CheckColorTool(myOcafDoc->Main())) { return Standard_False; }
    try {
        OcctObjectVisibilityState before, after;
        if (!CaptureObjectVisibilityStateForLabel(label, before)) { return Standard_False; }
        if (visible) {
            for (bool hidden : before.layerInvisibleAttributePresent) { if (hidden) { return Standard_False; } }
        }
        if (before.invisibleAttributePresent == !visible) { return Standard_True; }
        const auto colors = XCAFDoc_DocumentTool::ColorTool(myOcafDoc->Main());
        colors->SetVisibility(label, visible);
        return CaptureObjectVisibilityStateForLabel(label, after)
            && before.HasSameObjectAndLayers(after) && after.invisibleAttributePresent == !visible;
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctObjectNameState::IsEqual(const OcctObjectNameState& other) const noexcept {
    try {
        return object.IsEqual(other.object) && namePresent == other.namePresent
            && (!namePresent || name.IsEqual(other.name));
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctSavedGroupState::IsEqual(const OcctSavedGroupState& other) const noexcept {
    try {
        if (documentData.IsNull() || documentData != other.documentData
            || !container.IsEqual(other.container) || groups.size() != other.groups.size()) { return Standard_False; }
        for (std::size_t i = 0; i < groups.size(); ++i) {
            const auto& a = groups[i]; const auto& b = other.groups[i];
            if (!a.recordLabel.IsEqual(b.recordLabel) || a.identifier != b.identifier
                || !a.name.IsEqual(b.name) || a.members.size() != b.members.size()) { return Standard_False; }
            for (std::size_t j = 0; j < a.members.size(); ++j) {
                if (!a.members[j].IsEqual(b.members[j])) { return Standard_False; }
            }
        }
        return Standard_True;
    } catch (...) { return Standard_False; }
}
std::string OcctDocument::NewSavedGroupIdentifier() noexcept {
    if (![NSThread isMainThread]) { return {}; }
    try { return NewIdentifier(); } catch (...) { return {}; }
}
Standard_Boolean OcctDocument::CaptureSavedGroups(OcctSavedGroupState& state) const noexcept {
    state = OcctSavedGroupState();
    return [NSThread isMainThread] && ReadSavedGroups(myOcafDoc, state);
}
Standard_Boolean OcctDocument::StageSavedGroups(const std::vector<OcctSavedGroup>& groups) noexcept {
    if (![NSThread isMainThread] || myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || groups.size() > kMaximumSavedGroups) { return Standard_False; }
    try {
        OcctSavedGroupState before;
        if (!CaptureSavedGroups(before)) { return Standard_False; }
        std::unordered_set<std::string> identifiers, entities;
        std::vector<OcctObjectNameState> objectAuthority;
        for (const auto& group : groups) {
            if (!IsCanonicalSavedGroupID(group.identifier) || !OcctObjectNameIsValid(group.name)
                || !identifiers.insert(group.identifier).second || group.members.size() > kMaximumSavedGroupMembers) { return Standard_False; }
            for (const auto& label : group.members) {
                OcctObjectNameState object;
                if (!CaptureObjectNameStateForLabel(label, object)
                    || !entities.insert(object.object.entityIdentifier).second) { return Standard_False; }
                objectAuthority.push_back(std::move(object));
            }
        }
        // Capture removed members too; catalog replacement never changes parts.
        for (const auto& group : before.groups) {
            for (const auto& label : group.members) {
                OcctObjectNameState object;
                if (!CaptureObjectNameStateForLabel(label, object)) { return Standard_False; }
                objectAuthority.push_back(std::move(object));
            }
        }
        TDF_Label container = before.container;
        if (container.IsNull() && groups.empty()) { return Standard_True; }
        if (container.IsNull()) {
            const auto root = myOcafDoc->GetData()->Root();
            Standard_Integer tag = 0;
            for (TDF_ChildIterator it(root, Standard_False); it.More(); it.Next()) { tag = std::max(tag, it.Value().Tag()); }
            if (tag == std::numeric_limits<Standard_Integer>::max()) { return Standard_False; }
            // A sibling of Main is outside all XCAF document-tool fixed tags.
            container = root.FindChild(tag + 1, Standard_True);
            TDataStd_Integer::Set(container, SavedGroupContainerID(), 1);
        }
        std::unordered_map<std::string, TDF_Label> retained;
        std::vector<TDF_Label> reusable;
        for (const auto& group : before.groups) {
            if (identifiers.count(group.identifier)) { retained.emplace(group.identifier, group.recordLabel); }
            for (const auto& label : group.members) { label.ForgetAttribute(SavedGroupMembershipID()); }
            group.recordLabel.ForgetAttribute(SavedGroupRecordID());
            group.recordLabel.ForgetAttribute(SavedGroupNameID());
        }
        Standard_Integer maximumTag = 0;
        for (TDF_ChildIterator it(container, Standard_False); it.More(); it.Next()) {
            const auto label = it.Value(); maximumTag = std::max(maximumTag, label.Tag());
            bool reserved = false;
            for (const auto& pair : retained) { if (pair.second.IsEqual(label)) { reserved = true; break; } }
            if (!reserved && !label.HasAttribute() && !label.HasChild()) { reusable.push_back(label); }
        }
        for (const auto& group : groups) {
            TDF_Label label;
            const auto found = retained.find(group.identifier);
            if (found != retained.end()) { label = found->second; }
            else if (!reusable.empty()) { label = reusable.back(); reusable.pop_back(); }
            else {
                if (maximumTag == std::numeric_limits<Standard_Integer>::max()) { return Standard_False; }
                label = container.FindChild(++maximumTag, Standard_True);
            }
            TDataStd_AsciiString::Set(label, SavedGroupRecordID(), TCollection_AsciiString(group.identifier.c_str()));
            TDataStd_Name::Set(label, SavedGroupNameID(), group.name);
            for (const auto& member : group.members) {
                TDataStd_AsciiString::Set(member, SavedGroupMembershipID(), TCollection_AsciiString(group.identifier.c_str()));
            }
        }
        OcctSavedGroupState after;
        if (!CaptureSavedGroups(after) || after.groups.size() != groups.size()) { return Standard_False; }
        for (const auto& requested : groups) {
            const auto found = std::find_if(after.groups.begin(), after.groups.end(), [&](const auto& g) { return g.identifier == requested.identifier; });
            if (found == after.groups.end() || !found->name.IsEqual(requested.name)
                || found->members.size() != requested.members.size()) { return Standard_False; }
            for (const auto& member : requested.members) {
                if (std::none_of(found->members.begin(), found->members.end(), [&](const auto& l) { return l.IsEqual(member); })) { return Standard_False; }
            }
        }
        for (const auto& expected : objectAuthority) {
            OcctObjectNameState actual;
            if (!CaptureObjectNameStateForLabel(expected.object.label, actual) || !expected.IsEqual(actual)) { return Standard_False; }
        }
        return Standard_True;
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctObjectNameIsValid(const TCollection_ExtendedString& name) noexcept {
    try {
        if (name.Length() < 1 || name.Length() > 256) { return Standard_False; }
        NSString* value = [[NSString alloc]
            initWithCharacters:reinterpret_cast<const unichar*>(name.ToExtString())
            length:static_cast<NSUInteger>(name.Length())];
        if (value == nil || [value dataUsingEncoding:NSUTF8StringEncoding allowLossyConversion:NO] == nil) {
            return Standard_False;
        }
        for (Standard_Integer index = 1; index <= name.Length(); ++index) {
            const auto character = name.Value(index);
            if (character < 0x20 || (character >= 0x7f && character <= 0x9f)
                || character == 0x2028 || character == 0x2029) { return Standard_False; }
        }
        NSString* trimmed = [value stringByTrimmingCharactersInSet:NSCharacterSet.whitespaceAndNewlineCharacterSet];
        return trimmed.length > 0 && [trimmed isEqualToString:value];
    } catch (...) { return Standard_False; }
}

Standard_Boolean OcctDocument::CaptureObjectNameStateForLabel(
    const TDF_Label& label, OcctObjectNameState& state) const noexcept {
    state = OcctObjectNameState();
    if (![NSThread isMainThread]) { return Standard_False; }
    try {
        OcctObjectNameState captured;
        if (!CaptureObjectTransformStateForLabel(label, captured.object)) { return Standard_False; }
        Handle(TDF_Attribute) attribute;
        if (label.FindAttribute(TDataStd_Name::GetID(), attribute)) {
            const auto authored = Handle(TDataStd_Name)::DownCast(attribute);
            if (authored.IsNull()) { return Standard_False; }
            captured.namePresent = Standard_True;
            captured.name = authored->Get();
        }
        state = std::move(captured);
        return Standard_True;
    } catch (...) { state = OcctObjectNameState(); return Standard_False; }
}

Standard_Boolean OcctDocument::SetObjectNameForLabel(
    const TDF_Label& label, const TCollection_ExtendedString& name) noexcept {
    if (![NSThread isMainThread] || myOcafDoc.IsNull() || !myOcafDoc->HasOpenCommand()
        || !OcctObjectNameIsValid(name)) { return Standard_False; }
    try {
        OcctObjectNameState before, after;
        if (!CaptureObjectNameStateForLabel(label, before)) { return Standard_False; }
        if (before.namePresent && before.name.IsEqual(name)) { return Standard_True; }
        TDataStd_Name::Set(label, name);
        return CaptureObjectNameStateForLabel(label, after)
            && before.object.IsEqual(after.object) && after.namePresent && after.name.IsEqual(name);
    } catch (...) { return Standard_False; }
}

void OcctDocument::LoadObjectTransform(const TDF_Label& aRefLabel, const Handle(AIS_Shape) anAis) {
    anAis->SetLocalTransformation(ObjectTransformForLabel(aRefLabel));
}

Standard_Boolean OcctDocument::undo() {
    if (myNativeAuthority && !myOcafDoc.IsNull()) myNativeAuthority->HistoryBoundary(myOcafDoc.get());
    if (!canUndo()) {
		return Standard_False;
    }
    try {
        if (myOcafDoc->Undo()) {
            if (myNativeAuthority) myNativeAuthority->HistoryBoundary(myOcafDoc.get());
#if DEBUG
            if (myLiveProbe) myLiveProbe->Record(
                core3d::debug::LiveTransactionObservation::Kind::UndoCompleted, myOcafDoc);
#endif
            NotifyChanges();
			return Standard_True;
        }
    } catch (const Standard_Failure& ex) {
        std::cout << ex.GetMessageString() << std::endl;
    }
	return Standard_False;
}
Standard_Boolean OcctDocument::redo() {
    if (myNativeAuthority && !myOcafDoc.IsNull()) myNativeAuthority->HistoryBoundary(myOcafDoc.get());
    if (!canRedo()) {
		return Standard_False;
    }
    try {
		if (myOcafDoc->Redo()) {
            if (myNativeAuthority) myNativeAuthority->HistoryBoundary(myOcafDoc.get());
#if DEBUG
            if (myLiveProbe) myLiveProbe->Record(
                core3d::debug::LiveTransactionObservation::Kind::RedoCompleted, myOcafDoc);
#endif
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
    Standard_Size frameBytes = 0;
    if (myOcafDoc.IsNull() || myOcafDoc->HasOpenCommand()
        || !ValidateGeometryRepresentations()
        || !Core3DValidateOwnedFrameUsage(myOcafDoc,frameBytes)) {
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
