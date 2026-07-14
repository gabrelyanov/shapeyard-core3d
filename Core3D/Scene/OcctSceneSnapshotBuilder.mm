//
//  OcctSceneSnapshotBuilder.mm
//  Core3D
//

#import <Foundation/Foundation.h>
#import <ImageIO/ImageIO.h>

#include "OcctSceneSnapshotBuilder.hpp"

#include "../Common/Core3DMobileResourceLimits.h"
#include "../OCCTKit/OcctDocument.h"

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <BRep_Tool.hxx>
#include <Graphic3d_Camera.hxx>
#include <Graphic3d_AspectFillArea3d.hxx>
#include <Graphic3d_MaterialAspect.hxx>
#include <Graphic3d_PBRMaterial.hxx>
#include <Graphic3d_TextureSet.hxx>
#include <Image_Texture.hxx>
#include <Precision.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_TriangulationParameters.hxx>
#include <Poly_Triangle.hxx>
#include <Prs3d_Drawer.hxx>
#include <Prs3d_ShadingAspect.hxx>
#include <RWMesh_FaceIterator.hxx>
#include <NCollection_Buffer.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <StdPrs_ToolTriangulatedShape.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_Data.hxx>
#include <TDF_LabelSequence.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_VisMaterial.hxx>
#include <XCAFDoc_VisMaterialTool.hxx>
#include <XCAFPrs_DocumentExplorer.hxx>
#include <V3d_View.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <CommonCrypto/CommonDigest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace core3d::scene {
namespace {

constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr std::size_t kMaxFacesPerMesh = 250'000;
constexpr std::size_t kMaxVerticesPerMesh = 1'000'000;
constexpr std::size_t kMaxIndicesPerMesh = 3'000'000;
constexpr std::size_t kMaxMeshNumericBytes = 96ULL * 1024ULL * 1024ULL;
// These publication ceilings account for the peak where immutable C++ values
// and their Objective-C DTO copies coexist. Larger documents remain editable
// in the OCCT viewport and can use a future packed/streamed snapshot path.
constexpr std::size_t kMaxVerticesPerSnapshot = 1'500'000;
constexpr std::size_t kMaxIndicesPerSnapshot = 4'500'000;
constexpr std::size_t kMaxSnapshotNumericBytes = 96ULL * 1024ULL * 1024ULL;
constexpr std::size_t kMaxMaterialsPerSnapshot = 50'000;
constexpr std::size_t kMaxPickElementsPerSnapshot = 250'001;
constexpr std::size_t kMaxPrimitiveBindingsPerSnapshot = 250'000;
constexpr std::size_t kMaxTexturesPerSnapshot = 256;
constexpr std::size_t kMaxEncodedTextureBytes = 32ULL * 1024ULL * 1024ULL;
constexpr std::size_t kMaxAggregateEncodedTextureBytes =
    64ULL * 1024ULL * 1024ULL;
constexpr std::size_t kMaxAggregateDecodedTextureBytes =
    128ULL * 1024ULL * 1024ULL;
constexpr std::uint64_t kMaxTextureDimension = 8192;
constexpr std::uint64_t kMaxTexturePixels = 4096ULL * 4096ULL;
constexpr double kLegacyMetersPerUnit = 0.001;
constexpr std::size_t kMaxOccurrenceDepth = 1'024;
constexpr std::size_t kMaxLabelInstanceMappings = 1'000'000;
constexpr std::size_t kMaxSelectedElements = 50'000;
constexpr std::size_t kMaxRetainedDefinitionRevisions = 250'000;
constexpr std::size_t kMaxOverlayMeshes = 16;
constexpr std::size_t kMaxRetainedOverlayDefinitions = 48;
constexpr std::size_t kMaxOverlayInstances = 16;
constexpr std::size_t kMaxOverlayMaterials = 16;
constexpr std::size_t kMaxOverlayVertices = 100'000;
constexpr std::size_t kMaxOverlayIndices = 300'000;
constexpr std::size_t kMaxOverlayPrimitives = 25'000;
constexpr std::size_t kMaxOverlayPrimitiveBindings = 25'000;
constexpr std::size_t kMaxOverlayNumericBytes = 16ULL * 1024ULL * 1024ULL;
constexpr std::size_t kMaxMirrorPreviewBodies = 8;
constexpr std::size_t kMaxBooleanSourceOperands = 8;
constexpr std::size_t kMaxChamferPreviewBodies = 8;
constexpr std::size_t kMaxLinearArrayPreviewBodies = 15;
constexpr std::size_t kMaxShellStyledSubshapeLabels = 1'024;
constexpr std::array<const char*, 6> kMirrorEntityIdentifiers = {
    "gizmo/mirroring/x/negative",
    "gizmo/mirroring/y/negative",
    "gizmo/mirroring/z/negative",
    "gizmo/mirroring/x/positive",
    "gizmo/mirroring/y/positive",
    "gizmo/mirroring/z/positive",
};
constexpr std::array<const char*, 6> kMirrorMeshIdentifiers = {
    "gizmo/mirroring/x/negative/mesh",
    "gizmo/mirroring/y/negative/mesh",
    "gizmo/mirroring/z/negative/mesh",
    "gizmo/mirroring/x/positive/mesh",
    "gizmo/mirroring/y/positive/mesh",
    "gizmo/mirroring/z/positive/mesh",
};
constexpr std::array<const char*, 6> kMirrorMaterialIdentifiers = {
    "gizmo/material/mirroring/x/negative",
    "gizmo/material/mirroring/y/negative",
    "gizmo/material/mirroring/z/negative",
    "gizmo/material/mirroring/x/positive",
    "gizmo/material/mirroring/y/positive",
    "gizmo/material/mirroring/z/positive",
};
constexpr std::array<const char*, 6> kMirrorNames = {
    "Negative X mirror plane",
    "Negative Y mirror plane",
    "Negative Z mirror plane",
    "Positive X mirror plane",
    "Positive Y mirror plane",
    "Positive Z mirror plane",
};

class Fingerprint {
public:
    void AddByte(const std::uint8_t theValue)
    {
        myValue ^= theValue;
        myValue *= kFnvPrime;
    }

    template <typename Integer>
    void AddInteger(Integer theValue)
    {
        using Unsigned = std::make_unsigned_t<Integer>;
        std::uint64_t aValue = static_cast<Unsigned>(theValue);
        for (std::size_t anIndex = 0; anIndex < sizeof(Unsigned); ++anIndex) {
            AddByte(static_cast<std::uint8_t>(aValue & 0xffU));
            aValue >>= 8U;
        }
    }

    void AddBool(const bool theValue) { AddByte(theValue ? 1U : 0U); }

    void AddFloat(const float theValue)
    {
        std::uint32_t aBits = 0;
        static_assert(sizeof(aBits) == sizeof(theValue));
        std::memcpy(&aBits, &theValue, sizeof(aBits));
        AddInteger(aBits);
    }

    void AddDouble(const double theValue)
    {
        std::uint64_t aBits = 0;
        static_assert(sizeof(aBits) == sizeof(theValue));
        std::memcpy(&aBits, &theValue, sizeof(aBits));
        AddInteger(aBits);
    }

    void AddString(const std::string& theValue)
    {
        AddInteger<std::uint64_t>(theValue.size());
        for (const char aCharacter : theValue) {
            AddByte(static_cast<std::uint8_t>(
                static_cast<unsigned char>(aCharacter)));
        }
    }

    std::uint64_t Value() const { return myValue; }

private:
    std::uint64_t myValue = kFnvOffset;
};

std::string DeriveOccurrenceIdentifier(
    const std::string& theDocumentIdentifier,
    const std::vector<std::string>& theLabelIdentifiers)
{
    if (theLabelIdentifiers.size() == 1) {
        return theLabelIdentifiers.front();
    }
    if (theDocumentIdentifier.empty() || theLabelIdentifiers.empty()) {
        return {};
    }

    const auto hashPath = [&](const char* theDomain) {
        Fingerprint aHash;
        aHash.AddString(theDomain);
        aHash.AddString(theDocumentIdentifier);
        aHash.AddInteger<std::uint64_t>(theLabelIdentifiers.size());
        for (const std::string& anIdentifier : theLabelIdentifiers) {
            aHash.AddString(anIdentifier);
        }
        return aHash.Value();
    };

    const std::uint64_t aHigh = hashPath("shapeyard-occurrence-v8-high");
    const std::uint64_t aLow = hashPath("shapeyard-occurrence-v8-low");
    std::array<std::uint8_t, 16> aBytes;
    for (std::size_t anIndex = 0; anIndex < 8; ++anIndex) {
        const std::size_t aShift = (7U - anIndex) * 8U;
        aBytes[anIndex] = static_cast<std::uint8_t>(aHigh >> aShift);
        aBytes[anIndex + 8U] = static_cast<std::uint8_t>(aLow >> aShift);
    }
    // RFC 9562 UUID variant and application-defined version 8.
    aBytes[6] = static_cast<std::uint8_t>((aBytes[6] & 0x0fU) | 0x80U);
    aBytes[8] = static_cast<std::uint8_t>((aBytes[8] & 0x3fU) | 0x80U);

    std::ostringstream aStream;
    aStream << std::hex << std::setfill('0');
    for (std::size_t anIndex = 0; anIndex < aBytes.size(); ++anIndex) {
        if (anIndex == 4 || anIndex == 6 || anIndex == 8 || anIndex == 10) {
            aStream << '-';
        }
        aStream << std::setw(2)
                << static_cast<unsigned int>(aBytes[anIndex]);
    }
    return aStream.str();
}

bool IsFinite(const double theValue)
{
    return std::isfinite(theValue);
}

bool IsFinite(const float theValue)
{
    return std::isfinite(theValue);
}

bool FitsFloat(const double theValue)
{
    return IsFinite(theValue)
        && std::abs(theValue) <= std::numeric_limits<float>::max();
}

bool FitsUInt32(const std::size_t theValue)
{
    return theValue <= std::numeric_limits<std::uint32_t>::max();
}

bool CheckedMultiply(const std::size_t theLeft,
                     const std::size_t theRight,
                     std::size_t& theResult)
{
    if (theLeft != 0
        && theRight > std::numeric_limits<std::size_t>::max() / theLeft) {
        return false;
    }
    theResult = theLeft * theRight;
    return true;
}

bool CheckedAdd(const std::size_t theLeft,
                const std::size_t theRight,
                std::size_t& theResult)
{
    if (theRight > std::numeric_limits<std::size_t>::max() - theLeft) {
        return false;
    }
    theResult = theLeft + theRight;
    return true;
}

bool IncrementRevision(std::uint64_t& theRevision)
{
    if (theRevision == std::numeric_limits<std::uint64_t>::max()) {
        return false;
    }
    ++theRevision;
    return true;
}

void Extend(Bounds3d& theBounds, const double theX, const double theY, const double theZ)
{
    if (!theBounds.valid) {
        theBounds.minimum = {theX, theY, theZ};
        theBounds.maximum = theBounds.minimum;
        theBounds.valid = true;
        return;
    }
    theBounds.minimum.x = std::min(theBounds.minimum.x, theX);
    theBounds.minimum.y = std::min(theBounds.minimum.y, theY);
    theBounds.minimum.z = std::min(theBounds.minimum.z, theZ);
    theBounds.maximum.x = std::max(theBounds.maximum.x, theX);
    theBounds.maximum.y = std::max(theBounds.maximum.y, theY);
    theBounds.maximum.z = std::max(theBounds.maximum.z, theZ);
}

bool IsValid(const Bounds3d& theBounds)
{
    return theBounds.valid
        && IsFinite(theBounds.minimum.x)
        && IsFinite(theBounds.minimum.y)
        && IsFinite(theBounds.minimum.z)
        && IsFinite(theBounds.maximum.x)
        && IsFinite(theBounds.maximum.y)
        && IsFinite(theBounds.maximum.z)
        && theBounds.minimum.x <= theBounds.maximum.x
        && theBounds.minimum.y <= theBounds.maximum.y
        && theBounds.minimum.z <= theBounds.maximum.z;
}

bool IsValidIdentifier(const std::string& theValue)
{
    if (theValue.empty() || theValue.size() > 128) {
        return false;
    }
    return std::all_of(
        theValue.begin(), theValue.end(), [](const unsigned char theCharacter) {
            return theCharacter >= 0x21U && theCharacter <= 0x7eU;
        });
}

bool IsRigidWorldAnchorTransform(const Matrix4d& theMatrix)
{
    for (const double aValue : theMatrix.values) {
        if (!IsFinite(aValue)) {
            return false;
        }
    }
    constexpr double aTolerance = 1.0e-6;
    if (std::abs(theMatrix.values[3]) > aTolerance
        || std::abs(theMatrix.values[7]) > aTolerance
        || std::abs(theMatrix.values[11]) > aTolerance
        || std::abs(theMatrix.values[15] - 1.0) > aTolerance) {
        return false;
    }
    const Double3 anX = {
        theMatrix.values[0], theMatrix.values[1], theMatrix.values[2]};
    const Double3 aY = {
        theMatrix.values[4], theMatrix.values[5], theMatrix.values[6]};
    const Double3 aZ = {
        theMatrix.values[8], theMatrix.values[9], theMatrix.values[10]};
    const auto dot = [](const Double3& theLeft, const Double3& theRight) {
        return theLeft.x * theRight.x
            + theLeft.y * theRight.y
            + theLeft.z * theRight.z;
    };
    const double aDeterminant =
        anX.x * (aY.y * aZ.z - aY.z * aZ.y)
        - aY.x * (anX.y * aZ.z - anX.z * aZ.y)
        + aZ.x * (anX.y * aY.z - anX.z * aY.y);
    return std::abs(dot(anX, anX) - 1.0) <= aTolerance
        && std::abs(dot(aY, aY) - 1.0) <= aTolerance
        && std::abs(dot(aZ, aZ) - 1.0) <= aTolerance
        && std::abs(dot(anX, aY)) <= aTolerance
        && std::abs(dot(anX, aZ)) <= aTolerance
        && std::abs(dot(aY, aZ)) <= aTolerance
        && std::abs(aDeterminant - 1.0) <= aTolerance;
}

bool IsTranslationOnlyWorldTransform(const Matrix4d& theMatrix)
{
    if (!IsRigidWorldAnchorTransform(theMatrix)) {
        return false;
    }
    constexpr double aTolerance = 1.0e-6;
    const std::array<double, 12> anExpected = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
    };
    for (std::size_t anIndex = 0;
         anIndex < anExpected.size(); ++anIndex) {
        if (std::abs(theMatrix.values[anIndex]
                     - anExpected[anIndex]) > aTolerance) {
            return false;
        }
    }
    return true;
}

bool IsCenteredLocalBounds(const Bounds3d& theBounds)
{
    if (!IsValid(theBounds)) {
        return false;
    }
    const double aScale = std::max({
        1.0,
        std::abs(theBounds.minimum.x),
        std::abs(theBounds.minimum.y),
        std::abs(theBounds.minimum.z),
        std::abs(theBounds.maximum.x),
        std::abs(theBounds.maximum.y),
        std::abs(theBounds.maximum.z),
    });
    const double aTolerance = aScale
        * 32.0 * std::numeric_limits<float>::epsilon();
    return std::abs((theBounds.minimum.x + theBounds.maximum.x) * 0.5)
            <= aTolerance
        && std::abs((theBounds.minimum.y + theBounds.maximum.y) * 0.5)
            <= aTolerance
        && std::abs((theBounds.minimum.z + theBounds.maximum.z) * 0.5)
            <= aTolerance;
}

Double3 Center(const Bounds3d& theBounds)
{
    return {
        theBounds.minimum.x + (theBounds.maximum.x - theBounds.minimum.x) * 0.5,
        theBounds.minimum.y + (theBounds.maximum.y - theBounds.minimum.y) * 0.5,
        theBounds.minimum.z + (theBounds.maximum.z - theBounds.minimum.z) * 0.5,
    };
}

bool MatrixFromTransform(const gp_Trsf& theTransform, Matrix4d& theMatrix)
{
    for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
        for (Standard_Integer aColumn = 1; aColumn <= 4; ++aColumn) {
            const double aValue = theTransform.Value(aRow, aColumn);
            if (!IsFinite(aValue)) {
                return false;
            }
            theMatrix.values[static_cast<std::size_t>(aColumn - 1) * 4
                           + static_cast<std::size_t>(aRow - 1)] = aValue;
        }
    }
    theMatrix.values[3] = 0.0;
    theMatrix.values[7] = 0.0;
    theMatrix.values[11] = 0.0;
    theMatrix.values[15] = 1.0;
    return true;
}

bool TransformPoint(const Matrix4d& theMatrix,
                    const Double3& thePoint,
                    Double3& theResult)
{
    theResult = {
        theMatrix.values[0] * thePoint.x
            + theMatrix.values[4] * thePoint.y
            + theMatrix.values[8] * thePoint.z
            + theMatrix.values[12],
        theMatrix.values[1] * thePoint.x
            + theMatrix.values[5] * thePoint.y
            + theMatrix.values[9] * thePoint.z
            + theMatrix.values[13],
        theMatrix.values[2] * thePoint.x
            + theMatrix.values[6] * thePoint.y
            + theMatrix.values[10] * thePoint.z
            + theMatrix.values[14],
    };
    return IsFinite(theResult.x) && IsFinite(theResult.y) && IsFinite(theResult.z);
}

std::string ReadName(const TDF_Label& theOccurrenceLabel,
                     const TDF_Label& theDefinitionLabel)
{
    Handle(TDataStd_Name) aName;
    if ((theOccurrenceLabel.IsNull()
         || !theOccurrenceLabel.FindAttribute(TDataStd_Name::GetID(), aName)
         || aName.IsNull())
        && (!theDefinitionLabel.IsNull())) {
        aName.Nullify();
        theDefinitionLabel.FindAttribute(TDataStd_Name::GetID(), aName);
    }
    if (aName.IsNull() || aName->Get().IsEmpty()) {
        return {};
    }

    const TCollection_ExtendedString& aValue = aName->Get();
    NSString* aString = [[NSString alloc]
        initWithCharacters:reinterpret_cast<const unichar*>(aValue.ToExtString())
                   length:static_cast<NSUInteger>(aValue.Length())];
    const char* aUtf8 = aString.UTF8String;
    return aUtf8 == nullptr ? std::string() : std::string(aUtf8);
}

bool DetectTextureEncoding(const Standard_Byte* theBytes,
                           const Standard_Size theSize,
                           TextureEncoding& theEncoding)
{
    if (theBytes == nullptr) {
        return false;
    }
    if (theSize >= 8
        && std::memcmp(theBytes, "\x89PNG\r\n\x1A\n", 8) == 0) {
        theEncoding = TextureEncoding::PNG;
        return true;
    }
    if (theSize >= 3 && theBytes[0] == 0xffU
        && theBytes[1] == 0xd8U && theBytes[2] == 0xffU) {
        theEncoding = TextureEncoding::JPEG;
        return true;
    }
    if (theSize >= 6
        && (std::memcmp(theBytes, "GIF87a", 6) == 0
            || std::memcmp(theBytes, "GIF89a", 6) == 0)) {
        theEncoding = TextureEncoding::GIF;
        return true;
    }
    if (theSize >= 4
        && (std::memcmp(theBytes, "II\x2A\x00", 4) == 0
            || std::memcmp(theBytes, "MM\x00\x2A", 4) == 0)) {
        theEncoding = TextureEncoding::TIFF;
        return true;
    }
    if (theSize >= 2 && std::memcmp(theBytes, "BM", 2) == 0) {
        theEncoding = TextureEncoding::BMP;
        return true;
    }
    if (theSize >= 12 && std::memcmp(theBytes, "RIFF", 4) == 0
        && std::memcmp(theBytes + 8, "WEBP", 4) == 0) {
        theEncoding = TextureEncoding::WebP;
        return true;
    }
    return false;
}

bool ReadTextureMetadata(const Handle(NCollection_Buffer)& theBuffer,
                         TextureEncoding& theEncoding,
                         std::uint32_t& thePixelWidth,
                         std::uint32_t& thePixelHeight,
                         std::size_t& theDecodedBytes)
{
    thePixelWidth = 0;
    thePixelHeight = 0;
    theDecodedBytes = 0;
    if (theBuffer.IsNull() || theBuffer->Size() == 0
        || theBuffer->Size() > kMaxEncodedTextureBytes
        || !DetectTextureEncoding(theBuffer->Data(),
                                  theBuffer->Size(),
                                  theEncoding)) {
        return false;
    }

    CFDataRef aData = CFDataCreateWithBytesNoCopy(
        kCFAllocatorDefault,
        reinterpret_cast<const UInt8*>(theBuffer->Data()),
        static_cast<CFIndex>(theBuffer->Size()),
        kCFAllocatorNull);
    if (aData == nullptr) {
        return false;
    }
    const void* anOptionKeys[] = {kCGImageSourceShouldCache};
    const void* anOptionValues[] = {kCFBooleanFalse};
    CFDictionaryRef anOptions = CFDictionaryCreate(
        kCFAllocatorDefault,
        anOptionKeys,
        anOptionValues,
        1,
        &kCFTypeDictionaryKeyCallBacks,
        &kCFTypeDictionaryValueCallBacks);
    CGImageSourceRef aSource = CGImageSourceCreateWithData(aData, anOptions);
    if (anOptions != nullptr) {
        CFRelease(anOptions);
    }
    CFRelease(aData);
    if (aSource == nullptr || CGImageSourceGetType(aSource) == nullptr
        || CGImageSourceGetCount(aSource) != 1
        || CGImageSourceGetStatus(aSource) != kCGImageStatusComplete
        || CGImageSourceGetStatusAtIndex(aSource, 0)
            != kCGImageStatusComplete) {
        if (aSource != nullptr) {
            CFRelease(aSource);
        }
        return false;
    }

    CFDictionaryRef aProperties =
        CGImageSourceCopyPropertiesAtIndex(aSource, 0, nullptr);
    CFRelease(aSource);
    if (aProperties == nullptr) {
        return false;
    }
    const CFTypeRef aWidthValue = CFDictionaryGetValue(
        aProperties, kCGImagePropertyPixelWidth);
    const CFTypeRef aHeightValue = CFDictionaryGetValue(
        aProperties, kCGImagePropertyPixelHeight);
    const CFTypeRef aDepthValue = CFDictionaryGetValue(
        aProperties, kCGImagePropertyDepth);
    std::int64_t aWidth = 0;
    std::int64_t aHeight = 0;
    std::int64_t aDepth = 0;
    const bool isValid = aWidthValue != nullptr && aHeightValue != nullptr
        && aDepthValue != nullptr
        && CFGetTypeID(aWidthValue) == CFNumberGetTypeID()
        && CFGetTypeID(aHeightValue) == CFNumberGetTypeID()
        && CFGetTypeID(aDepthValue) == CFNumberGetTypeID()
        && CFNumberGetValue(static_cast<CFNumberRef>(aWidthValue),
                            kCFNumberSInt64Type,
                            &aWidth)
        && CFNumberGetValue(static_cast<CFNumberRef>(aHeightValue),
                            kCFNumberSInt64Type,
                            &aHeight)
        && CFNumberGetValue(static_cast<CFNumberRef>(aDepthValue),
                            kCFNumberSInt64Type,
                            &aDepth)
        && aWidth > 0 && aHeight > 0
        // The Metal contract is normalized 8-bit sRGB. Higher precision
        // images remain valid OCCT content and deliberately use that renderer.
        && aDepth > 0 && aDepth <= 8
        && static_cast<std::uint64_t>(aWidth) <= kMaxTextureDimension
        && static_cast<std::uint64_t>(aHeight) <= kMaxTextureDimension
        && static_cast<std::uint64_t>(aWidth)
            <= kMaxTexturePixels / static_cast<std::uint64_t>(aHeight);
    if (isValid) {
        const std::uint64_t aPixels = static_cast<std::uint64_t>(aWidth)
            * static_cast<std::uint64_t>(aHeight);
        const std::uint64_t aDecodedBytes = aPixels * 4ULL;
        if (aDecodedBytes <= kMaxAggregateDecodedTextureBytes) {
            thePixelWidth = static_cast<std::uint32_t>(aWidth);
            thePixelHeight = static_cast<std::uint32_t>(aHeight);
            theDecodedBytes = static_cast<std::size_t>(aDecodedBytes);
        }
    }
    CFRelease(aProperties);
    return isValid && thePixelWidth > 0 && thePixelHeight > 0
        && theDecodedBytes > 0;
}

std::string SHA256Identifier(const Standard_Byte* theBytes,
                             const Standard_Size theSize)
{
    std::array<unsigned char, CC_SHA256_DIGEST_LENGTH> aDigest;
    if (CC_SHA256(theBytes, static_cast<CC_LONG>(theSize), aDigest.data())
        == nullptr) {
        return {};
    }
    std::ostringstream aStream;
    aStream << "texture-sha256-" << std::hex << std::setfill('0');
    for (const unsigned char aByte : aDigest) {
        aStream << std::setw(2) << static_cast<unsigned int>(aByte);
    }
    return aStream.str();
}

struct TextureTableState {
    std::size_t aggregateEncodedBytes = 0;
    std::size_t aggregateDecodedBytes = 0;
    std::unordered_map<std::string, std::vector<std::size_t>> resourcesByDigest;
    std::unordered_map<std::string, std::size_t> resourcesBySourceIdentifier;
};

bool AddTextureResource(SceneSnapshot& theScene,
                        TextureTableState& theState,
                        const Handle(Image_Texture)& theTexture,
                        std::int32_t& theTextureIndex)
{
    theTextureIndex = -1;
    if (theTexture.IsNull()) {
        return true;
    }
    if (!theTexture->FilePath().IsEmpty()
        || theTexture->TextureId().IsEmpty()
        || theTexture->TextureId().Length() > 256) {
        return false;
    }
    const Handle(NCollection_Buffer)& aBuffer = theTexture->DataBuffer();
    if (aBuffer.IsNull() || aBuffer->Data() == nullptr
        || aBuffer->Size() == 0
        || aBuffer->Size() > kMaxEncodedTextureBytes) {
        return false;
    }

    const std::string aSourceIdentifier(
        theTexture->TextureId().ToCString(),
        static_cast<std::size_t>(theTexture->TextureId().Length()));
    const auto aSourceFound =
        theState.resourcesBySourceIdentifier.find(aSourceIdentifier);
    if (aSourceFound != theState.resourcesBySourceIdentifier.end()) {
        const std::size_t anExistingIndex = aSourceFound->second;
        if (anExistingIndex >= theScene.textures.size()
            || theScene.textures[anExistingIndex].encodedBytes.size()
                != aBuffer->Size()
            || std::memcmp(theScene.textures[anExistingIndex]
                               .encodedBytes.data(),
                           aBuffer->Data(),
                           aBuffer->Size()) != 0) {
            return false;
        }
        theTextureIndex = static_cast<std::int32_t>(anExistingIndex);
        return true;
    }

    const std::string aDigest = SHA256Identifier(aBuffer->Data(),
                                                 aBuffer->Size());
    if (aDigest.empty()) {
        return false;
    }
    const auto aDigestFound = theState.resourcesByDigest.find(aDigest);
    if (aDigestFound != theState.resourcesByDigest.end()) {
        for (const std::size_t anExistingIndex : aDigestFound->second) {
            const TextureResourceSnapshot& anExisting =
                theScene.textures[anExistingIndex];
            if (anExisting.encodedBytes.size() == aBuffer->Size()
                && std::memcmp(anExisting.encodedBytes.data(),
                               aBuffer->Data(),
                               aBuffer->Size()) == 0) {
                theState.resourcesBySourceIdentifier.emplace(
                    aSourceIdentifier, anExistingIndex);
                theTextureIndex = static_cast<std::int32_t>(anExistingIndex);
                return true;
            }
        }
        // A content identifier is an exact cache and revision invariant. A
        // cryptographic collision cannot be represented safely; fail closed.
        return false;
    }

    TextureEncoding anEncoding = TextureEncoding::PNG;
    std::uint32_t aWidth = 0;
    std::uint32_t aHeight = 0;
    std::size_t aDecodedBytes = 0;
    if (!ReadTextureMetadata(aBuffer,
                             anEncoding,
                             aWidth,
                             aHeight,
                             aDecodedBytes)) {
        return false;
    }

    if (theScene.textures.size() >= kMaxTexturesPerSnapshot
        || theScene.textures.size()
            > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())
        || aBuffer->Size() > kMaxAggregateEncodedTextureBytes
        || theState.aggregateEncodedBytes
            > kMaxAggregateEncodedTextureBytes - aBuffer->Size()
        || aDecodedBytes > kMaxAggregateDecodedTextureBytes
        || theState.aggregateDecodedBytes
            > kMaxAggregateDecodedTextureBytes - aDecodedBytes) {
        return false;
    }

    TextureResourceSnapshot aResource;
    aResource.identifier = aDigest;
    aResource.encoding = anEncoding;
    aResource.pixelWidth = aWidth;
    aResource.pixelHeight = aHeight;
    aResource.encodedBytes.assign(aBuffer->Data(),
                                  aBuffer->Data() + aBuffer->Size());
    const std::size_t aResourceIndex = theScene.textures.size();
    theScene.textures.push_back(std::move(aResource));
    theState.resourcesByDigest[aDigest].push_back(aResourceIndex);
    theState.resourcesBySourceIdentifier.emplace(aSourceIdentifier,
                                                 aResourceIndex);
    theState.aggregateEncodedBytes += aBuffer->Size();
    theState.aggregateDecodedBytes += aDecodedBytes;
    theTextureIndex = static_cast<std::int32_t>(aResourceIndex);
    return true;
}

AlphaMode ConvertAlphaMode(const Graphic3d_AlphaMode theMode,
                           const float theAlpha)
{
    switch (theMode) {
        case Graphic3d_AlphaMode_Mask:
            return AlphaMode::Mask;
        case Graphic3d_AlphaMode_Blend:
        case Graphic3d_AlphaMode_MaskBlend:
            return AlphaMode::Blend;
        case Graphic3d_AlphaMode_BlendAuto:
            return theAlpha < 0.999f ? AlphaMode::Blend : AlphaMode::Opaque;
        case Graphic3d_AlphaMode_Opaque:
        default:
            return AlphaMode::Opaque;
    }
}

void SetColor(MaterialSnapshot& theMaterial, const Quantity_ColorRGBA& theColor)
{
    theMaterial.baseColor = {
        static_cast<float>(theColor.GetRGB().Red()),
        static_cast<float>(theColor.GetRGB().Green()),
        static_cast<float>(theColor.GetRGB().Blue()),
        theColor.Alpha(),
    };
}

void SetColor(MaterialSnapshot& theMaterial,
              const Quantity_Color& theColor,
              const float theAlpha)
{
    theMaterial.baseColor = {
        static_cast<float>(theColor.Red()),
        static_cast<float>(theColor.Green()),
        static_cast<float>(theColor.Blue()),
        theAlpha,
    };
}

bool ApplyPreset(MaterialSnapshot& theMaterial,
                 const Graphic3d_NameOfMaterial theName,
                 const bool theClosed)
{
    if (theName < Graphic3d_NameOfMaterial_Brass
        || theName > Graphic3d_NameOfMaterial_Transparent) {
        return false;
    }
    const Graphic3d_MaterialAspect anAspect(theName);
    const Graphic3d_PBRMaterial& aPbr = anAspect.PBRMaterial();
    SetColor(theMaterial, aPbr.Color());
    const Graphic3d_Vec3 anEmission = aPbr.Emission();
    theMaterial.emission = {anEmission.x(), anEmission.y(), anEmission.z()};
    theMaterial.metallic = aPbr.Metallic();
    theMaterial.roughness = aPbr.NormalizedRoughness();
    theMaterial.indexOfRefraction = aPbr.IOR();
    theMaterial.alphaCutoff = 0.5f;
    theMaterial.alphaMode = aPbr.Alpha() < 0.999f
        ? AlphaMode::Blend
        : AlphaMode::Opaque;
    theMaterial.cullMode = theClosed ? CullMode::Back : CullMode::None;
    return true;
}

MaterialSnapshot DefaultMaterial(const bool theClosed)
{
    MaterialSnapshot aMaterial;
    if (!ApplyPreset(aMaterial,
                     Graphic3d_NameOfMaterial_ShinyPlastified,
                     theClosed)) {
        return aMaterial;
    }
    SetColor(aMaterial, Quantity_Color(Quantity_NOC_GRAY80), aMaterial.baseColor.w);
    return aMaterial;
}

bool ValidateMaterial(MaterialSnapshot& theMaterial)
{
    const auto isUnit = [](const float theValue) {
        return IsFinite(theValue) && theValue >= 0.0f && theValue <= 1.0f;
    };
    if (!isUnit(theMaterial.baseColor.x)
        || !isUnit(theMaterial.baseColor.y)
        || !isUnit(theMaterial.baseColor.z)
        || !isUnit(theMaterial.baseColor.w)
        || !isUnit(theMaterial.metallic)
        || !isUnit(theMaterial.roughness)
        || !IsFinite(theMaterial.indexOfRefraction)
        || theMaterial.indexOfRefraction < 1.0f
        || !isUnit(theMaterial.alphaCutoff)
        || !IsFinite(theMaterial.emission.x)
        || !IsFinite(theMaterial.emission.y)
        || !IsFinite(theMaterial.emission.z)
        || theMaterial.emission.x < 0.0f
        || theMaterial.emission.y < 0.0f
        || theMaterial.emission.z < 0.0f) {
        return false;
    }
    return true;
}

struct WholeObjectPBRMaterial {
    XCAFDoc_VisMaterialPBR pbr;
    Graphic3d_AlphaMode alphaMode = Graphic3d_AlphaMode_Opaque;
    Standard_ShortReal alphaCutoff = 0.5f;
    Graphic3d_TypeOfBackfacingModel faceCulling =
        Graphic3d_TypeOfBackfacingModel_Auto;
};

bool ResolveMaterial(const RWMesh_FaceIterator& theFace,
                     const std::optional<Graphic3d_NameOfMaterial>& theMaterialOverride,
                     const std::optional<Quantity_NameOfColor>& theColorOverride,
                     const std::optional<WholeObjectPBRMaterial>& thePbrOverride,
                     const bool theClosed,
                     MaterialSnapshot& theResult,
                     Handle(Image_Texture)& theBaseColorTexture,
                     Handle(Image_Texture)& theEmissiveTexture)
{
    theResult = DefaultMaterial(theClosed);
    theBaseColorTexture.Nullify();
    theEmissiveTexture.Nullify();
    const XCAFPrs_Style& aStyle = theFace.FaceStyle();
    const Handle(XCAFDoc_VisMaterial)& aVisualMaterial = aStyle.Material();
    if (!aVisualMaterial.IsNull()) {
        if (aVisualMaterial->HasPbrMaterial()) {
            const XCAFDoc_VisMaterialPBR& aPbr = aVisualMaterial->PbrMaterial();
            SetColor(theResult, aPbr.BaseColor);
            theResult.emission = {
                aPbr.EmissiveFactor.x(),
                aPbr.EmissiveFactor.y(),
                aPbr.EmissiveFactor.z(),
            };
            theResult.metallic = aPbr.Metallic;
            theResult.roughness = aPbr.Roughness;
            theResult.indexOfRefraction = aPbr.RefractionIndex;
        } else if (aVisualMaterial->HasCommonMaterial()) {
            const XCAFDoc_VisMaterialCommon& aCommon =
                aVisualMaterial->CommonMaterial();
            SetColor(theResult,
                     aCommon.DiffuseColor,
                     1.0f - aCommon.Transparency);
            theResult.emission = {
                static_cast<float>(aCommon.EmissiveColor.Red()),
                static_cast<float>(aCommon.EmissiveColor.Green()),
                static_cast<float>(aCommon.EmissiveColor.Blue()),
            };
            theResult.metallic = Graphic3d_PBRMaterial::MetallicFromSpecular(
                aCommon.SpecularColor);
            theResult.roughness = Graphic3d_PBRMaterial::RoughnessFromSpecular(
                aCommon.SpecularColor,
                aCommon.Shininess);
        }
        theResult.alphaMode = ConvertAlphaMode(aVisualMaterial->AlphaMode(),
                                                theResult.baseColor.w);
        theResult.alphaCutoff = aVisualMaterial->AlphaCutOff();
        switch (aVisualMaterial->FaceCulling()) {
            case Graphic3d_TypeOfBackfacingModel_DoubleSided:
                theResult.cullMode = CullMode::None;
                break;
            case Graphic3d_TypeOfBackfacingModel_Auto:
                theResult.cullMode = theClosed
                    ? CullMode::Back : CullMode::None;
                break;
            case Graphic3d_TypeOfBackfacingModel_BackCulled:
                theResult.cullMode = CullMode::Back;
                break;
            case Graphic3d_TypeOfBackfacingModel_FrontCulled:
                theResult.cullMode = CullMode::Front;
                break;
        }
    }

    if (theFace.HasFaceColor()) {
        SetColor(theResult, theFace.FaceColor());
    } else if (aStyle.IsSetColorSurf()) {
        SetColor(theResult, aStyle.GetColorSurfRGBA());
    }

    if (thePbrOverride.has_value()) {
        const WholeObjectPBRMaterial& anOverride = *thePbrOverride;
        const XCAFDoc_VisMaterialPBR& aPbr = anOverride.pbr;
        if (!aPbr.IsDefined
            || !aPbr.MetallicRoughnessTexture.IsNull()
            || !aPbr.OcclusionTexture.IsNull()
            || !aPbr.NormalTexture.IsNull()) {
            return false;
        }
        SetColor(theResult, aPbr.BaseColor);
        theResult.emission = {
            aPbr.EmissiveFactor.x(),
            aPbr.EmissiveFactor.y(),
            aPbr.EmissiveFactor.z(),
        };
        theResult.metallic = aPbr.Metallic;
        theResult.roughness = aPbr.Roughness;
        theResult.indexOfRefraction = aPbr.RefractionIndex;
        theResult.alphaMode = ConvertAlphaMode(
            anOverride.alphaMode, aPbr.BaseColor.Alpha());
        theResult.alphaCutoff = anOverride.alphaCutoff;
        switch (anOverride.faceCulling) {
            case Graphic3d_TypeOfBackfacingModel_DoubleSided:
                theResult.cullMode = CullMode::None;
                break;
            case Graphic3d_TypeOfBackfacingModel_Auto:
                theResult.cullMode = theClosed
                    ? CullMode::Back : CullMode::None;
                break;
            case Graphic3d_TypeOfBackfacingModel_BackCulled:
                theResult.cullMode = CullMode::Back;
                break;
            case Graphic3d_TypeOfBackfacingModel_FrontCulled:
                theResult.cullMode = CullMode::Front;
                break;
        }
        theBaseColorTexture = aPbr.BaseColorTexture;
        theEmissiveTexture = aPbr.EmissiveTexture;
    } else {
        if (theMaterialOverride.has_value()
            && !ApplyPreset(theResult, *theMaterialOverride, theClosed)) {
            return false;
        }
        if (theColorOverride.has_value()) {
            if (*theColorOverride < Quantity_NOC_BLACK
                || *theColorOverride > Quantity_NOC_WHITE) {
                return false;
            }
            SetColor(theResult,
                     Quantity_Color(*theColorOverride),
                     theResult.baseColor.w);
        }
    }
    // Authored whole-object materials replace imported appearance, while a
    // color-only override remains a scalar tint over the embedded texture.
    if (!thePbrOverride.has_value() && !theMaterialOverride.has_value()) {
        if (!aVisualMaterial.IsNull()
            && aVisualMaterial->HasPbrMaterial()) {
            const XCAFDoc_VisMaterialPBR& aPbr =
                aVisualMaterial->PbrMaterial();
            // Schema v4 represents base color and emissive. Preserve rendering
            // fidelity by keeping OCCT active whenever another map matters.
            if (!aPbr.MetallicRoughnessTexture.IsNull()
                || !aPbr.OcclusionTexture.IsNull()
                || !aPbr.NormalTexture.IsNull()) {
                return false;
            }
            theEmissiveTexture = aPbr.EmissiveTexture;
        }
        theBaseColorTexture = aStyle.BaseColorTexture();
    }
    return ValidateMaterial(theResult);
}

bool MaterialValuesEqual(const MaterialSnapshot& theLeft,
                         const MaterialSnapshot& theRight)
{
    return theLeft.baseColor.x == theRight.baseColor.x
        && theLeft.baseColor.y == theRight.baseColor.y
        && theLeft.baseColor.z == theRight.baseColor.z
        && theLeft.baseColor.w == theRight.baseColor.w
        && theLeft.emission.x == theRight.emission.x
        && theLeft.emission.y == theRight.emission.y
        && theLeft.emission.z == theRight.emission.z
        && theLeft.metallic == theRight.metallic
        && theLeft.roughness == theRight.roughness
        && theLeft.indexOfRefraction == theRight.indexOfRefraction
        && theLeft.alphaMode == theRight.alphaMode
        && theLeft.alphaCutoff == theRight.alphaCutoff
        && theLeft.cullMode == theRight.cullMode
        && theLeft.baseColorTextureIndex
            == theRight.baseColorTextureIndex
        && theLeft.emissiveTextureIndex
            == theRight.emissiveTextureIndex;
}

void AddMaterialValues(Fingerprint& theHash, const MaterialSnapshot& theMaterial)
{
    theHash.AddFloat(theMaterial.baseColor.x);
    theHash.AddFloat(theMaterial.baseColor.y);
    theHash.AddFloat(theMaterial.baseColor.z);
    theHash.AddFloat(theMaterial.baseColor.w);
    theHash.AddFloat(theMaterial.emission.x);
    theHash.AddFloat(theMaterial.emission.y);
    theHash.AddFloat(theMaterial.emission.z);
    theHash.AddFloat(theMaterial.metallic);
    theHash.AddFloat(theMaterial.roughness);
    theHash.AddFloat(theMaterial.indexOfRefraction);
    theHash.AddInteger(static_cast<std::uint8_t>(theMaterial.alphaMode));
    theHash.AddFloat(theMaterial.alphaCutoff);
    theHash.AddInteger(static_cast<std::uint8_t>(theMaterial.cullMode));
    theHash.AddInteger(theMaterial.baseColorTextureIndex);
    theHash.AddInteger(theMaterial.emissiveTextureIndex);
}

std::string HexIdentifier(const char* thePrefix, const std::uint64_t theValue)
{
    std::ostringstream aStream;
    aStream << thePrefix << std::hex << std::setw(16) << std::setfill('0') << theValue;
    return aStream.str();
}

bool AddMaterial(SceneSnapshot& theScene,
                 MaterialSnapshot theMaterial,
                 std::uint32_t& theIndex)
{
    for (std::size_t anIndex = 0; anIndex < theScene.materials.size(); ++anIndex) {
        if (MaterialValuesEqual(theScene.materials[anIndex], theMaterial)) {
            theIndex = static_cast<std::uint32_t>(anIndex);
            return true;
        }
    }
    if (!FitsUInt32(theScene.materials.size())
        || theScene.materials.size() >= kMaxMaterialsPerSnapshot) {
        return false;
    }
    Fingerprint aHash;
    AddMaterialValues(aHash, theMaterial);
    const std::string aBaseIdentifier = HexIdentifier("material-", aHash.Value());
    theMaterial.identifier = aBaseIdentifier;
    std::uint32_t aCollision = 1;
    const auto identifierExists = [&theScene](const std::string& theIdentifier) {
        return std::any_of(theScene.materials.begin(),
                           theScene.materials.end(),
                           [&theIdentifier](const MaterialSnapshot& theExisting) {
                               return theExisting.identifier == theIdentifier;
                           });
    };
    while (identifierExists(theMaterial.identifier)) {
        if (aCollision == std::numeric_limits<std::uint32_t>::max()) {
            return false;
        }
        theMaterial.identifier = aBaseIdentifier + "-" + std::to_string(aCollision++);
    }
    theIndex = static_cast<std::uint32_t>(theScene.materials.size());
    theScene.materials.push_back(std::move(theMaterial));
    return true;
}

struct SourceVertex {
    Double3 position;
    Double3 normal;
    double textureU = 0.0;
    double textureV = 0.0;
};

struct OccurrenceData {
    TDF_Label occurrenceLabel;
    TDF_Label definitionLabel;
    TopLoc_Location occurrenceLocation;
    XCAFPrs_Style style;
    std::vector<std::string> labelIdentifiers;
    std::string entityIdentifier;
    std::string definitionIdentifier;
    std::string name;
    bool visible = true;
};

struct DefinitionData {
    TDF_Label label;
    TopoDS_Shape shape;
    TopTools_IndexedMapOfShape faces;
    MeshSnapshot mesh;
    Double3 sourceOrigin;
    OcctGeometryRepresentation representation =
        OcctGeometryRepresentation::Invalid;
    std::uint64_t fingerprint = 0;
    bool closed = false;
};

std::uint64_t MeshFingerprint(const MeshSnapshot& theMesh,
                              const double theLinearDeflection,
                              const double theAngularDeflection)
{
    Fingerprint aHash;
    aHash.AddString(theMesh.definitionIdentifier);
    aHash.AddDouble(theLinearDeflection);
    aHash.AddDouble(theAngularDeflection);
    aHash.AddInteger<std::uint64_t>(theMesh.vertices.size());
    for (const Vertex& aVertex : theMesh.vertices) {
        aHash.AddFloat(aVertex.positionX);
        aHash.AddFloat(aVertex.positionY);
        aHash.AddFloat(aVertex.positionZ);
        aHash.AddFloat(aVertex.normalX);
        aHash.AddFloat(aVertex.normalY);
        aHash.AddFloat(aVertex.normalZ);
        aHash.AddFloat(aVertex.textureU);
        aHash.AddFloat(aVertex.textureV);
    }
    aHash.AddInteger<std::uint64_t>(theMesh.indices.size());
    for (const std::uint32_t anIndex : theMesh.indices) {
        aHash.AddInteger(anIndex);
    }
    aHash.AddInteger<std::uint64_t>(theMesh.primitives.size());
    for (const MeshPrimitive& aPrimitive : theMesh.primitives) {
        aHash.AddInteger(aPrimitive.firstIndex);
        aHash.AddInteger(aPrimitive.indexCount);
        aHash.AddInteger(aPrimitive.faceIndex);
        aHash.AddBool(aPrimitive.hasTextureCoordinates);
    }
    return aHash.Value();
}

struct WorldPreviewItem {
    MeshSnapshot mesh;
    InstanceSnapshot instance;
    MaterialSnapshot material;
};

bool HasUnsupportedPresentationTexture(
    const Handle(AIS_Shape)& thePresentation)
{
    if (thePresentation.IsNull()) {
        return true;
    }
    const Handle(Prs3d_Drawer)& aDrawer =
        thePresentation->Attributes();
    if (aDrawer.IsNull() || aDrawer->ShadingAspect().IsNull()
        || aDrawer->ShadingAspect()->Aspect().IsNull()) {
        return true;
    }
    const Handle(Graphic3d_AspectFillArea3d)& anAspect =
        aDrawer->ShadingAspect()->Aspect();
    return anAspect->ToMapTexture()
        || (!anAspect->TextureSet().IsNull()
            && !anAspect->TextureSet()->IsEmpty());
}

bool HasUnsupportedVisualMaterialTexture(const TDF_Label& theLabel) noexcept
{
    if (theLabel.IsNull()) {
        return true;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(XCAFDoc_VisMaterial) aMaterial =
            XCAFDoc_VisMaterialTool::GetShapeMaterial(theLabel);
        if (aMaterial.IsNull()) {
            return false;
        }
        if (aMaterial->HasPbrMaterial()) {
            const XCAFDoc_VisMaterialPBR& aPbr =
                aMaterial->PbrMaterial();
            if (!aPbr.BaseColorTexture.IsNull()
                || !aPbr.EmissiveTexture.IsNull()
                || !aPbr.MetallicRoughnessTexture.IsNull()
                || !aPbr.OcclusionTexture.IsNull()
                || !aPbr.NormalTexture.IsNull()) {
                return true;
            }
        }
        return aMaterial->HasCommonMaterial()
            && !aMaterial->CommonMaterial().DiffuseTexture.IsNull();
    } catch (...) {
        return true;
    }
}

//! Shell replaces topology, so a source subshape style cannot be reproduced
//! by the scalar-only transient overlay. Bound the defensive label walk and
//! fail closed on absent tools, malformed labels, or OCCT failures.
bool HasStyledShellSubshape(
    const Handle(TDocStd_Document)& theDocument,
    const TDF_Label& theDefinition) noexcept
{
    if (theDocument.IsNull() || theDefinition.IsNull()
        || theDefinition.Data() != theDocument->GetData()) {
        return true;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(XCAFDoc_ColorTool) aColorTool =
            XCAFDoc_DocumentTool::CheckColorTool(theDocument->Main())
                ? XCAFDoc_DocumentTool::ColorTool(theDocument->Main())
                : Handle(XCAFDoc_ColorTool)();
        const Handle(XCAFDoc_LayerTool) aLayerTool =
            XCAFDoc_DocumentTool::CheckLayerTool(theDocument->Main())
                ? XCAFDoc_DocumentTool::LayerTool(theDocument->Main())
                : Handle(XCAFDoc_LayerTool)();
        std::size_t aLabelCount = 0;
        for (TDF_ChildIterator anItem(theDefinition, Standard_False);
             anItem.More(); anItem.Next()) {
            if (++aLabelCount > kMaxShellStyledSubshapeLabels) {
                return true;
            }
            const TDF_Label& aLabel = anItem.Value();
            if (!XCAFDoc_ShapeTool::IsSubShape(aLabel)) {
                continue;
            }
            TDF_LabelSequence aLayers;
            if (aLabel.IsNull()
                || (!aColorTool.IsNull()
                    && (aColorTool->IsSet(aLabel, XCAFDoc_ColorGen)
                        || aColorTool->IsSet(aLabel, XCAFDoc_ColorSurf)
                        || aColorTool->IsSet(aLabel, XCAFDoc_ColorCurv)
                        || !XCAFDoc_ColorTool::IsVisible(aLabel)))
                || !XCAFDoc_VisMaterialTool::GetShapeMaterial(
                        aLabel).IsNull()
                || (!aLayerTool.IsNull()
                    && aLayerTool->GetLayers(aLabel, aLayers)
                    && !aLayers.IsEmpty())) {
                return true;
            }
        }
        return false;
    } catch (...) {
        return true;
    }
}

bool HasCompatibleLinearArrayStyle(
    const Handle(AIS_Shape)& theFirst,
    const Handle(AIS_Shape)& theCandidate)
{
    if (theFirst.IsNull() || theCandidate.IsNull()
        || !theFirst->HasColor() || !theCandidate->HasColor()
        || !theFirst->HasMaterial() || !theCandidate->HasMaterial()
        || HasUnsupportedPresentationTexture(theFirst)
        || HasUnsupportedPresentationTexture(theCandidate)
        || theFirst->Material() != theCandidate->Material()) {
        return false;
    }
    Quantity_Color aFirstColor;
    Quantity_Color aCandidateColor;
    theFirst->Color(aFirstColor);
    theCandidate->Color(aCandidateColor);
    const double aFirstTransparency = theFirst->Transparency();
    const double aCandidateTransparency = theCandidate->Transparency();
    return IsFinite(aFirstTransparency)
        && IsFinite(aCandidateTransparency)
        && aFirstTransparency == aCandidateTransparency
        && aFirstColor.Red() == aCandidateColor.Red()
        && aFirstColor.Green() == aCandidateColor.Green()
        && aFirstColor.Blue() == aCandidateColor.Blue();
}

bool HaveCompatibleLinearArrayBasis(
    const gp_Trsf& theFirst,
    const gp_Trsf& theCandidate)
{
    for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
        for (Standard_Integer aColumn = 1; aColumn <= 3; ++aColumn) {
            const double aFirst = theFirst.Value(aRow, aColumn);
            const double aCandidate =
                theCandidate.Value(aRow, aColumn);
            if (!IsFinite(aFirst) || !IsFinite(aCandidate)
                || aFirst != aCandidate) {
                return false;
            }
        }
    }
    return true;
}

//! Copy one explicitly owned AIS preview using only triangulations that
//! already exist on its BRep faces. No mesher is invoked: a cache miss fails
//! closed so OCCT remains the authoritative renderer for that frame.
bool ExtractWorldPreviewItem(
    const Handle(AIS_Shape)& thePresentation,
    const std::string& theIdentifier,
    const std::string& theName,
    const std::uint32_t theMeshAndMaterialIndex,
    const RenderRole theRole,
    const RenderStyle theRenderStyle,
    const std::optional<Quantity_NameOfColor> theColorOverride,
    const std::size_t theMaxVertices,
    const std::size_t theMaxIndices,
    const std::size_t theMaxPrimitives,
    WorldPreviewItem& theItem)
{
    if (thePresentation.IsNull()
        || !IsValidIdentifier(theIdentifier)
        || theName.empty() || theName.size() > 4'096
        || theMeshAndMaterialIndex >= kMaxOverlayMeshes
        || !thePresentation->HasColor()
        || !thePresentation->HasMaterial()) {
        return false;
    }
    const TopoDS_Shape& aShape = thePresentation->Shape();
    if (aShape.IsNull()) {
        return false;
    }

    std::vector<TopoDS_Face> aFaces;
    aFaces.reserve(std::min<std::size_t>(theMaxPrimitives, 256));
    for (TopExp_Explorer aFaceExplorer(aShape, TopAbs_FACE);
         aFaceExplorer.More(); aFaceExplorer.Next()) {
        if (aFaces.size() >= theMaxPrimitives
            || aFaces.size() >= kMaxOverlayPrimitives) {
            return false;
        }
        aFaces.push_back(TopoDS::Face(aFaceExplorer.Current()));
    }
    const std::size_t aFaceCount = aFaces.size();
    if (aFaceCount == 0 || !FitsUInt32(aFaceCount)) {
        return false;
    }

    std::vector<SourceVertex> aSourceVertices;
    std::vector<std::uint32_t> aSourceIndices;
    std::vector<MeshPrimitive> aPrimitives;
    aPrimitives.reserve(aFaceCount);
    Bounds3d aSourceBounds;
    const gp_Trsf aPresentationTransform =
        thePresentation->Transformation();

    for (std::size_t aFaceIndex = 0;
         aFaceIndex < aFaces.size(); ++aFaceIndex) {
        const TopoDS_Face& aFace = aFaces[aFaceIndex];
        if (aFace.Orientation() != TopAbs_FORWARD
            && aFace.Orientation() != TopAbs_REVERSED) {
            return false;
        }
        TopLoc_Location aFaceLocation;
        const Handle(Poly_Triangulation)& aTriangulation =
            BRep_Tool::Triangulation(aFace, aFaceLocation);
        if (aTriangulation.IsNull()
            || !aTriangulation->HasGeometry()
            || aTriangulation->NbNodes() <= 0
            || aTriangulation->NbTriangles() <= 0) {
            return false;
        }
        const std::size_t aNodeCount =
            static_cast<std::size_t>(aTriangulation->NbNodes());
        const std::size_t aTriangleCount =
            static_cast<std::size_t>(aTriangulation->NbTriangles());
        if (aTriangleCount > std::numeric_limits<std::size_t>::max() / 3U
            || aTriangleCount
                > std::numeric_limits<std::uint32_t>::max() / 3U) {
            return false;
        }
        const std::size_t aFaceIndexCount = aTriangleCount * 3U;
        if (aNodeCount > theMaxVertices
            || aSourceVertices.size() > theMaxVertices - aNodeCount
            || aSourceVertices.size()
                > std::numeric_limits<std::uint32_t>::max() - aNodeCount
            || aFaceIndexCount > theMaxIndices
            || aSourceIndices.size() > theMaxIndices - aFaceIndexCount
            || aSourceIndices.size()
                > std::numeric_limits<std::uint32_t>::max()
                    - aFaceIndexCount) {
            return false;
        }

        const std::uint32_t aVertexBase =
            static_cast<std::uint32_t>(aSourceVertices.size());
        const gp_Trsf aFaceTransform = aFaceLocation.Transformation();
        const bool hasNormals = aTriangulation->HasNormals();
        const bool isMirrored =
            static_cast<bool>(aFaceTransform.IsNegative())
            != static_cast<bool>(aPresentationTransform.IsNegative());
        for (Standard_Integer aNode = 1;
             aNode <= aTriangulation->NbNodes(); ++aNode) {
            gp_Pnt aPoint = aTriangulation->Node(aNode);
            aPoint.Transform(aFaceTransform);
            aPoint.Transform(aPresentationTransform);
            if (!IsFinite(aPoint.X()) || !IsFinite(aPoint.Y())
                || !IsFinite(aPoint.Z())) {
                return false;
            }
            SourceVertex aVertex;
            aVertex.position = {aPoint.X(), aPoint.Y(), aPoint.Z()};
            if (hasNormals) {
                gp_Dir aNormal = aTriangulation->Normal(aNode);
                aNormal.Transform(aFaceTransform);
                aNormal.Transform(aPresentationTransform);
                if (aFace.Orientation() == TopAbs_REVERSED) {
                    aNormal.Reverse();
                }
                aVertex.normal = {
                    aNormal.X(), aNormal.Y(), aNormal.Z()};
            }
            if (aTriangulation->HasUVNodes()) {
                const gp_Pnt2d aUV = aTriangulation->UVNode(aNode);
                if (!IsFinite(aUV.X()) || !IsFinite(aUV.Y())) {
                    return false;
                }
                aVertex.textureU = aUV.X();
                aVertex.textureV = aUV.Y();
            }
            aSourceVertices.push_back(aVertex);
            Extend(aSourceBounds, aPoint.X(), aPoint.Y(), aPoint.Z());
        }

        const std::uint32_t aFirstIndex =
            static_cast<std::uint32_t>(aSourceIndices.size());
        const bool reversesFace =
            (aFace.Orientation() == TopAbs_REVERSED) != isMirrored;
        for (Standard_Integer aTriangleIndex = 1;
             aTriangleIndex <= aTriangulation->NbTriangles();
             ++aTriangleIndex) {
            Standard_Integer aNodes[3] = {0, 0, 0};
            aTriangulation->Triangle(aTriangleIndex).Get(
                aNodes[0], aNodes[1], aNodes[2]);
            if (reversesFace) {
                std::swap(aNodes[1], aNodes[2]);
            }
            for (const Standard_Integer aNode : aNodes) {
                if (aNode < 1 || aNode > aTriangulation->NbNodes()) {
                    return false;
                }
            }
            const std::uint32_t aLocal0 =
                static_cast<std::uint32_t>(aNodes[0] - 1);
            const std::uint32_t aLocal1 =
                static_cast<std::uint32_t>(aNodes[1] - 1);
            const std::uint32_t aLocal2 =
                static_cast<std::uint32_t>(aNodes[2] - 1);
            if (aLocal0 == aLocal1 || aLocal1 == aLocal2
                || aLocal2 == aLocal0) {
                return false;
            }
            aSourceIndices.push_back(aVertexBase + aLocal0);
            aSourceIndices.push_back(aVertexBase + aLocal1);
            aSourceIndices.push_back(aVertexBase + aLocal2);

            if (!hasNormals) {
                const Double3& aP0 =
                    aSourceVertices[aVertexBase + aLocal0].position;
                const Double3& aP1 =
                    aSourceVertices[aVertexBase + aLocal1].position;
                const Double3& aP2 =
                    aSourceVertices[aVertexBase + aLocal2].position;
                const Double3 aU = {
                    aP1.x - aP0.x, aP1.y - aP0.y, aP1.z - aP0.z};
                const Double3 aV = {
                    aP2.x - aP0.x, aP2.y - aP0.y, aP2.z - aP0.z};
                const Double3 aCross = {
                    aU.y * aV.z - aU.z * aV.y,
                    aU.z * aV.x - aU.x * aV.z,
                    aU.x * aV.y - aU.y * aV.x,
                };
                const double aLengthSquared = aCross.x * aCross.x
                    + aCross.y * aCross.y + aCross.z * aCross.z;
                if (!IsFinite(aLengthSquared)
                    || aLengthSquared <= 0.0) {
                    return false;
                }
                for (const std::uint32_t aLocal : {
                         aLocal0, aLocal1, aLocal2}) {
                    Double3& aNormal =
                        aSourceVertices[aVertexBase + aLocal].normal;
                    aNormal.x += aCross.x;
                    aNormal.y += aCross.y;
                    aNormal.z += aCross.z;
                }
            }
        }

        if (!hasNormals) {
            for (std::size_t aNode = 0; aNode < aNodeCount; ++aNode) {
                Double3& aNormal =
                    aSourceVertices[aVertexBase + aNode].normal;
                const double aLength = std::sqrt(
                    aNormal.x * aNormal.x
                    + aNormal.y * aNormal.y
                    + aNormal.z * aNormal.z);
                if (!IsFinite(aLength)
                    || aLength <= Precision::Confusion()) {
                    return false;
                }
                aNormal.x /= aLength;
                aNormal.y /= aLength;
                aNormal.z /= aLength;
            }
        }
        aPrimitives.push_back({
            aFirstIndex,
            static_cast<std::uint32_t>(aFaceIndexCount),
            static_cast<std::uint32_t>(aFaceIndex),
            aTriangulation->HasUVNodes(),
        });
    }

    if (!IsValid(aSourceBounds)
        || aSourceVertices.empty() || aSourceIndices.empty()
        || aPrimitives.size() != aFaceCount) {
        return false;
    }

    MeshSnapshot aMesh;
    aMesh.definitionIdentifier = theIdentifier + "/mesh";
    aMesh.indices = std::move(aSourceIndices);
    aMesh.primitives = std::move(aPrimitives);
    const Double3 aSourceOrigin = Center(aSourceBounds);
    aMesh.vertices.reserve(aSourceVertices.size());
    for (const SourceVertex& aSource : aSourceVertices) {
        const double aPositionX = aSource.position.x - aSourceOrigin.x;
        const double aPositionY = aSource.position.y - aSourceOrigin.y;
        const double aPositionZ = aSource.position.z - aSourceOrigin.z;
        if (!FitsFloat(aPositionX) || !FitsFloat(aPositionY)
            || !FitsFloat(aPositionZ) || !FitsFloat(aSource.normal.x)
            || !FitsFloat(aSource.normal.y)
            || !FitsFloat(aSource.normal.z)
            || !FitsFloat(aSource.textureU)
            || !FitsFloat(aSource.textureV)) {
            return false;
        }
        Vertex aVertex;
        aVertex.positionX = static_cast<float>(aPositionX);
        aVertex.positionY = static_cast<float>(aPositionY);
        aVertex.positionZ = static_cast<float>(aPositionZ);
        aVertex.normalX = static_cast<float>(aSource.normal.x);
        aVertex.normalY = static_cast<float>(aSource.normal.y);
        aVertex.normalZ = static_cast<float>(aSource.normal.z);
        aVertex.textureU = static_cast<float>(aSource.textureU);
        aVertex.textureV = static_cast<float>(aSource.textureV);
        aMesh.vertices.push_back(aVertex);
        Extend(aMesh.localBounds,
               aVertex.positionX,
               aVertex.positionY,
               aVertex.positionZ);
    }
    if (!IsValid(aMesh.localBounds)) {
        return false;
    }

    const bool isClosed =
        StdPrs_ToolTriangulatedShape::IsClosed(aShape);
    MaterialSnapshot aMaterial = DefaultMaterial(isClosed);
    if (!ApplyPreset(aMaterial,
                     thePresentation->Material(),
                     isClosed)) {
        return false;
    }
    Quantity_Color aColor;
    if (theColorOverride.has_value()) {
        aColor = Quantity_Color(*theColorOverride);
    } else {
        thePresentation->Color(aColor);
    }
    const double aTransparency = thePresentation->Transparency();
    if (!IsFinite(aTransparency)
        || aTransparency < 0.0 || aTransparency > 1.0
        || aTransparency > 1.0e-6) {
        return false;
    }
    SetColor(aMaterial,
             aColor,
             static_cast<float>(1.0 - aTransparency));
    aMaterial.alphaMode = AlphaMode::Opaque;
    aMaterial.identifier = theIdentifier + "/material";
    if (!ValidateMaterial(aMaterial)) {
        return false;
    }

    InstanceSnapshot anInstance;
    anInstance.entityIdentifier = theIdentifier;
    anInstance.meshIndex = theMeshAndMaterialIndex;
    anInstance.worldFromObject.values[12] = aSourceOrigin.x;
    anInstance.worldFromObject.values[13] = aSourceOrigin.y;
    anInstance.worldFromObject.values[14] = aSourceOrigin.z;
    anInstance.visible = true;
    anInstance.selectable = false;
    anInstance.selected = false;
    anInstance.name = theName;
    anInstance.role = theRole;
    anInstance.coordinateSpace = CoordinateSpace::World;
    anInstance.depthPolicy = DepthPolicy::Scene;
    anInstance.renderStyle = theRenderStyle;
    anInstance.primitiveBindings.reserve(aMesh.primitives.size());
    for (std::size_t aPrimitiveIndex = 0;
         aPrimitiveIndex < aMesh.primitives.size(); ++aPrimitiveIndex) {
        anInstance.primitiveBindings.push_back({
            theMeshAndMaterialIndex,
            0,
            true,
        });
    }

    theItem.mesh = std::move(aMesh);
    theItem.instance = std::move(anInstance);
    theItem.material = std::move(aMaterial);
    return true;
}

bool ExtractDefinitionGeometry(const TDF_Label& theDefinitionLabel,
                               const std::string& theDefinitionIdentifier,
#ifdef DEBUG
                               const std::uint8_t theDebugTriangulationFailure,
#endif
                               DefinitionData& theDefinition)
{
    theDefinition.label = theDefinitionLabel;
    theDefinition.shape = XCAFDoc_ShapeTool::GetShape(theDefinitionLabel);
    if (theDefinition.shape.IsNull()) {
        return false;
    }
    TopExp::MapShapes(theDefinition.shape, TopAbs_FACE, theDefinition.faces);
    if (theDefinition.faces.IsEmpty()
        || !FitsUInt32(static_cast<std::size_t>(theDefinition.faces.Extent()))
        || static_cast<std::size_t>(theDefinition.faces.Extent()) > kMaxFacesPerMesh) {
        return false;
    }
    theDefinition.closed = StdPrs_ToolTriangulatedShape::IsClosed(theDefinition.shape);

    std::vector<SourceVertex> aSourceVertices;
    std::vector<std::uint32_t> aSourceIndices;
    std::vector<MeshPrimitive> aPrimitives;
    std::vector<bool> aVisited(static_cast<std::size_t>(theDefinition.faces.Extent()), false);
    Bounds3d aSourceBounds;

    RWMesh_FaceIterator aFace(theDefinitionLabel, TopLoc_Location(), Standard_False);
    for (; aFace.More(); aFace.Next()) {
        Handle(Poly_Triangulation) aTriangulation = aFace.Triangulation();
#ifdef DEBUG
        // Exercise the exact production admission predicate without mutating
        // live OCAF topology or the OpenGL-owned triangulation cache.
        if (theDebugTriangulationFailure == 1U) {
            aTriangulation.Nullify();
        }
#endif
        if (aTriangulation.IsNull()
            || !aTriangulation->HasGeometry()
            || aTriangulation->NbNodes() <= 0
            || aTriangulation->NbTriangles() <= 0) {
            return false;
        }
        const Handle(Poly_TriangulationParameters)& aParameters =
            aTriangulation->Parameters();
        double aStoredDeflection = aTriangulation->Deflection();
#ifdef DEBUG
        if (theDebugTriangulationFailure == 2U) {
            // A negative stored error is invalid even for legacy caches that
            // predate Poly_TriangulationParameters serialization.
            aStoredDeflection = -1.0;
        }
#endif
        if (!IsFinite(aStoredDeflection)
            || aStoredDeflection < 0.0) {
            return false;
        }
        if (!aParameters.IsNull()) {
            const double aParameterDeflection =
                aParameters->Deflection();
            const double aParameterAngle = aParameters->Angle();
            if (!aParameters->HasDeflection()
                || !aParameters->HasAngle()
                || !IsFinite(aParameterDeflection)
                || aParameterDeflection <= 0.0
                || !IsFinite(aParameterAngle)
                || aParameterAngle <= 0.0) {
                return false;
            }
        }
        if (aFace.NbNodes() <= 0 || aFace.NbTriangles() <= 0) {
            return false;
        }
        const Standard_Integer aFaceMapIndex = theDefinition.faces.FindIndex(aFace.Face());
        if (aFaceMapIndex <= 0 || aFaceMapIndex > theDefinition.faces.Extent()) {
            return false;
        }
        const std::size_t aFaceOffset = static_cast<std::size_t>(aFaceMapIndex - 1);
        if (aVisited[aFaceOffset]) {
            return false;
        }
        aVisited[aFaceOffset] = true;

        const std::size_t aNodeCount = static_cast<std::size_t>(aFace.NbNodes());
        const std::size_t aTriangleCount = static_cast<std::size_t>(aFace.NbTriangles());
        if (aNodeCount > kMaxVerticesPerMesh
            || aTriangleCount > kMaxIndicesPerMesh / 3U
            || aTriangleCount > std::numeric_limits<std::uint32_t>::max() / 3U
            || aSourceVertices.size() > std::numeric_limits<std::uint32_t>::max() - aNodeCount
            || aSourceIndices.size() > std::numeric_limits<std::uint32_t>::max()
                                          - aTriangleCount * 3U
            || aSourceVertices.size() > kMaxVerticesPerMesh - aNodeCount
            || aSourceIndices.size() > kMaxIndicesPerMesh - aTriangleCount * 3U) {
            return false;
        }

        const std::uint32_t aVertexBase =
            static_cast<std::uint32_t>(aSourceVertices.size());
        const bool hasNormals = aFace.HasNormals();
        const bool hasTexCoords = aFace.HasTexCoords();
        for (Standard_Integer aNode = aFace.NodeLower();
             aNode <= aFace.NodeUpper(); ++aNode) {
            const gp_Pnt aPoint = aFace.NodeTransformed(aNode);
            if (!IsFinite(aPoint.X()) || !IsFinite(aPoint.Y()) || !IsFinite(aPoint.Z())) {
                return false;
            }
            SourceVertex aVertex;
            aVertex.position = {aPoint.X(), aPoint.Y(), aPoint.Z()};
            Extend(aSourceBounds, aPoint.X(), aPoint.Y(), aPoint.Z());
            if (hasNormals) {
                const gp_Dir aNormal = aFace.NormalTransformed(aNode);
                aVertex.normal = {aNormal.X(), aNormal.Y(), aNormal.Z()};
            }
            if (hasTexCoords) {
                const gp_Pnt2d aTexCoord = aFace.NodeTexCoord(aNode);
                if (!IsFinite(aTexCoord.X()) || !IsFinite(aTexCoord.Y())) {
                    return false;
                }
                aVertex.textureU = aTexCoord.X();
                aVertex.textureV = aTexCoord.Y();
            }
            aSourceVertices.push_back(aVertex);
        }

        const std::uint32_t aFirstIndex =
            static_cast<std::uint32_t>(aSourceIndices.size());
        for (Standard_Integer aTriangleIndex = aFace.ElemLower();
             aTriangleIndex <= aFace.ElemUpper(); ++aTriangleIndex) {
            const Poly_Triangle aTriangle = aFace.TriangleOriented(aTriangleIndex);
            Standard_Integer aNodes[3] = {0, 0, 0};
            aTriangle.Get(aNodes[0], aNodes[1], aNodes[2]);
            for (const Standard_Integer aNode : aNodes) {
                if (aNode < 1 || aNode > aFace.NbNodes()) {
                    return false;
                }
            }
            const std::uint32_t aLocal0 = static_cast<std::uint32_t>(aNodes[0] - 1);
            const std::uint32_t aLocal1 = static_cast<std::uint32_t>(aNodes[1] - 1);
            const std::uint32_t aLocal2 = static_cast<std::uint32_t>(aNodes[2] - 1);
            if (aLocal0 == aLocal1 || aLocal1 == aLocal2 || aLocal2 == aLocal0) {
                return false;
            }
            aSourceIndices.push_back(aVertexBase + aLocal0);
            aSourceIndices.push_back(aVertexBase + aLocal1);
            aSourceIndices.push_back(aVertexBase + aLocal2);

            if (!hasNormals) {
                const Double3& aP0 = aSourceVertices[aVertexBase + aLocal0].position;
                const Double3& aP1 = aSourceVertices[aVertexBase + aLocal1].position;
                const Double3& aP2 = aSourceVertices[aVertexBase + aLocal2].position;
                const Double3 aU = {aP1.x - aP0.x, aP1.y - aP0.y, aP1.z - aP0.z};
                const Double3 aV = {aP2.x - aP0.x, aP2.y - aP0.y, aP2.z - aP0.z};
                const Double3 aCross = {
                    aU.y * aV.z - aU.z * aV.y,
                    aU.z * aV.x - aU.x * aV.z,
                    aU.x * aV.y - aU.y * aV.x,
                };
                const double aLengthSquared = aCross.x * aCross.x
                    + aCross.y * aCross.y + aCross.z * aCross.z;
                if (!IsFinite(aLengthSquared) || aLengthSquared <= 0.0) {
                    return false;
                }
                for (const std::uint32_t aLocal : {aLocal0, aLocal1, aLocal2}) {
                    Double3& aNormal = aSourceVertices[aVertexBase + aLocal].normal;
                    aNormal.x += aCross.x;
                    aNormal.y += aCross.y;
                    aNormal.z += aCross.z;
                }
            }
        }

        if (!hasNormals) {
            for (std::size_t aNode = 0; aNode < aNodeCount; ++aNode) {
                Double3& aNormal = aSourceVertices[aVertexBase + aNode].normal;
                const double aLength = std::sqrt(aNormal.x * aNormal.x
                    + aNormal.y * aNormal.y + aNormal.z * aNormal.z);
                if (!IsFinite(aLength) || aLength <= Precision::Confusion()) {
                    return false;
                }
                aNormal.x /= aLength;
                aNormal.y /= aLength;
                aNormal.z /= aLength;
            }
        }

        const std::size_t anIndexCount = aSourceIndices.size() - aFirstIndex;
        if (anIndexCount == 0 || !FitsUInt32(anIndexCount)) {
            return false;
        }
        aPrimitives.push_back({
            aFirstIndex,
            static_cast<std::uint32_t>(anIndexCount),
            static_cast<std::uint32_t>(aFaceMapIndex - 1),
            hasTexCoords,
        });
    }

    if (!IsValid(aSourceBounds)
        || std::find(aVisited.begin(), aVisited.end(), false) != aVisited.end()
        || aPrimitives.size() != aVisited.size()
        || aSourceVertices.size() > kMaxVerticesPerMesh
        || aSourceIndices.size() > kMaxIndicesPerMesh) {
        return false;
    }

    std::size_t aVertexBytes = 0;
    std::size_t anIndexBytes = 0;
    std::size_t aPrimitiveBytes = 0;
    std::size_t aMeshBytes = 0;
    if (!CheckedMultiply(aSourceVertices.size(), sizeof(Vertex), aVertexBytes)
        || !CheckedMultiply(aSourceIndices.size(), sizeof(std::uint32_t), anIndexBytes)
        || !CheckedMultiply(aPrimitives.size(), sizeof(MeshPrimitive), aPrimitiveBytes)
        || !CheckedAdd(aVertexBytes, anIndexBytes, aMeshBytes)
        || !CheckedAdd(aMeshBytes, aPrimitiveBytes, aMeshBytes)
        || aMeshBytes > kMaxMeshNumericBytes) {
        return false;
    }

    theDefinition.sourceOrigin = Center(aSourceBounds);
    MeshSnapshot aMesh;
    aMesh.definitionIdentifier = theDefinitionIdentifier;
    aMesh.indices = std::move(aSourceIndices);
    aMesh.primitives = std::move(aPrimitives);
    aMesh.vertices.reserve(aSourceVertices.size());
    for (const SourceVertex& aSource : aSourceVertices) {
        const double aPositionX = aSource.position.x - theDefinition.sourceOrigin.x;
        const double aPositionY = aSource.position.y - theDefinition.sourceOrigin.y;
        const double aPositionZ = aSource.position.z - theDefinition.sourceOrigin.z;
        if (!FitsFloat(aPositionX) || !FitsFloat(aPositionY) || !FitsFloat(aPositionZ)
            || !FitsFloat(aSource.normal.x) || !FitsFloat(aSource.normal.y)
            || !FitsFloat(aSource.normal.z) || !FitsFloat(aSource.textureU)
            || !FitsFloat(aSource.textureV)) {
            return false;
        }
        Vertex aVertex;
        aVertex.positionX = static_cast<float>(aPositionX);
        aVertex.positionY = static_cast<float>(aPositionY);
        aVertex.positionZ = static_cast<float>(aPositionZ);
        aVertex.normalX = static_cast<float>(aSource.normal.x);
        aVertex.normalY = static_cast<float>(aSource.normal.y);
        aVertex.normalZ = static_cast<float>(aSource.normal.z);
        aVertex.textureU = static_cast<float>(aSource.textureU);
        aVertex.textureV = static_cast<float>(aSource.textureV);
        aMesh.vertices.push_back(aVertex);
        Extend(aMesh.localBounds,
               aVertex.positionX,
               aVertex.positionY,
               aVertex.positionZ);
    }
    if (!IsValid(aMesh.localBounds)) {
        return false;
    }
    theDefinition.mesh = std::move(aMesh);
    // Geometry identity is derived from the immutable copied payload itself.
    // No desired quality is recomputed from attacker-controlled BRep geometry
    // on the main thread.
    theDefinition.fingerprint = MeshFingerprint(theDefinition.mesh, 0.0, 0.0);
    return true;
}

bool BuildCamera(const Handle(V3d_View)& theView,
                 const UInt2& theViewportPixels,
                 CameraSnapshot& theCamera)
{
    if (theView.IsNull() || theViewportPixels.x == 0 || theViewportPixels.y == 0) {
        return false;
    }
    const Handle(Graphic3d_Camera)& aCamera = theView->Camera();
    if (aCamera.IsNull()) {
        return false;
    }
    const gp_Pnt anEye = aCamera->Eye();
    const gp_Pnt aCenter = aCamera->Center();
    const gp_Dir anUp = aCamera->OrthogonalizedUp();
    theCamera.eye = {anEye.X(), anEye.Y(), anEye.Z()};
    theCamera.center = {aCenter.X(), aCenter.Y(), aCenter.Z()};
    theCamera.up = {anUp.X(), anUp.Y(), anUp.Z()};
    theCamera.projection = aCamera->IsOrthographic()
        ? Projection::Orthographic
        : Projection::Perspective;
    theCamera.aspect = static_cast<double>(theViewportPixels.x)
        / static_cast<double>(theViewportPixels.y);

    // OCCT defines FOVy()/Scale() against the viewport's minor dimension.
    // Publish renderer-neutral values against the actual vertical dimension
    // so consumers can always use a conventional vertical projection.
    const double aMinorToVertical = theCamera.aspect < 1.0
        ? 1.0 / theCamera.aspect
        : 1.0;
    const double aMinorFovRadians = aCamera->FOVy() * kPi / 180.0;
    theCamera.verticalFovRadians = 2.0 * std::atan(
        std::tan(aMinorFovRadians * 0.5) * aMinorToVertical);
    theCamera.orthographicHeight = aCamera->Scale() * aMinorToVertical;
    theCamera.nearPlane = aCamera->ZNear();
    theCamera.farPlane = aCamera->ZFar();
    theCamera.viewportPixels = theViewportPixels;

    const double aViewDirectionSquared =
        (aCenter.X() - anEye.X()) * (aCenter.X() - anEye.X())
        + (aCenter.Y() - anEye.Y()) * (aCenter.Y() - anEye.Y())
        + (aCenter.Z() - anEye.Z()) * (aCenter.Z() - anEye.Z());
    const bool hasFiniteValues = IsFinite(theCamera.eye.x)
        && IsFinite(theCamera.eye.y) && IsFinite(theCamera.eye.z)
        && IsFinite(theCamera.center.x) && IsFinite(theCamera.center.y)
        && IsFinite(theCamera.center.z) && IsFinite(theCamera.up.x)
        && IsFinite(theCamera.up.y) && IsFinite(theCamera.up.z)
        && IsFinite(aMinorFovRadians) && IsFinite(aMinorToVertical)
        && IsFinite(theCamera.verticalFovRadians)
        && IsFinite(theCamera.orthographicHeight)
        && IsFinite(theCamera.nearPlane) && IsFinite(theCamera.farPlane)
        && IsFinite(theCamera.aspect) && IsFinite(aViewDirectionSquared);
    if (!hasFiniteValues || aViewDirectionSquared <= 0.0
        || theCamera.nearPlane >= theCamera.farPlane
        || theCamera.aspect <= 0.0
        || theCamera.orthographicHeight <= 0.0) {
        return false;
    }
    return theCamera.projection == Projection::Orthographic
        || (theCamera.nearPlane > 0.0
            && aMinorFovRadians > 0.0
            && aMinorFovRadians < kPi
            && theCamera.verticalFovRadians > 0.0
            && theCamera.verticalFovRadians < kPi);
}

std::uint64_t CameraFingerprint(const CameraSnapshot& theCamera)
{
    Fingerprint aHash;
    aHash.AddInteger(static_cast<std::uint8_t>(theCamera.projection));
    for (const double aValue : {
             theCamera.eye.x, theCamera.eye.y, theCamera.eye.z,
             theCamera.center.x, theCamera.center.y, theCamera.center.z,
             theCamera.up.x, theCamera.up.y, theCamera.up.z,
             theCamera.verticalFovRadians, theCamera.orthographicHeight,
             theCamera.nearPlane, theCamera.farPlane, theCamera.aspect}) {
        aHash.AddDouble(aValue);
    }
    aHash.AddInteger(theCamera.viewportPixels.x);
    aHash.AddInteger(theCamera.viewportPixels.y);
    return aHash.Value();
}

std::uint64_t ModelFingerprint(const SceneSnapshot& theScene)
{
    Fingerprint aHash;
    aHash.AddDouble(theScene.metersPerUnit);
    aHash.AddInteger<std::uint64_t>(theScene.meshes.size());
    for (const MeshSnapshot& aMesh : theScene.meshes) {
        aHash.AddString(aMesh.definitionIdentifier);
        aHash.AddInteger(aMesh.geometryRevision);
    }
    aHash.AddInteger<std::uint64_t>(theScene.instances.size());
    for (const InstanceSnapshot& anInstance : theScene.instances) {
        aHash.AddString(anInstance.entityIdentifier);
        aHash.AddInteger(anInstance.meshIndex);
        for (const double aValue : anInstance.worldFromObject.values) {
            aHash.AddDouble(aValue);
        }
        aHash.AddBool(anInstance.reversesWinding);
        aHash.AddString(anInstance.name);
    }
    return aHash.Value();
}

std::uint64_t PresentationFingerprint(const SceneSnapshot& theScene)
{
    Fingerprint aHash;
    aHash.AddInteger<std::uint64_t>(theScene.textures.size());
    for (const TextureResourceSnapshot& aTexture : theScene.textures) {
        // The identifier is a SHA-256 of the exact payload, so hashing it plus
        // validated metadata avoids rescanning large immutable byte vectors.
        aHash.AddString(aTexture.identifier);
        aHash.AddInteger(static_cast<std::uint8_t>(aTexture.encoding));
        aHash.AddInteger(aTexture.pixelWidth);
        aHash.AddInteger(aTexture.pixelHeight);
        aHash.AddInteger<std::uint64_t>(aTexture.encodedBytes.size());
    }
    aHash.AddInteger<std::uint64_t>(theScene.materials.size());
    for (const MaterialSnapshot& aMaterial : theScene.materials) {
        aHash.AddString(aMaterial.identifier);
        AddMaterialValues(aHash, aMaterial);
    }
    aHash.AddInteger<std::uint64_t>(theScene.instances.size());
    for (const InstanceSnapshot& anInstance : theScene.instances) {
        aHash.AddString(anInstance.entityIdentifier);
        aHash.AddBool(anInstance.visible);
        aHash.AddBool(anInstance.selectable);
        aHash.AddBool(anInstance.selected);
        aHash.AddInteger(static_cast<std::uint8_t>(anInstance.role));
        aHash.AddInteger(
            static_cast<std::uint8_t>(anInstance.coordinateSpace));
        aHash.AddInteger(
            static_cast<std::uint8_t>(anInstance.depthPolicy));
        aHash.AddInteger(
            static_cast<std::uint8_t>(anInstance.renderStyle));
        aHash.AddInteger<std::uint64_t>(anInstance.primitiveBindings.size());
        for (const PrimitiveBinding& aBinding : anInstance.primitiveBindings) {
            aHash.AddInteger(aBinding.materialIndex);
            aHash.AddInteger(aBinding.pickToken);
            aHash.AddBool(aBinding.visible);
        }
    }
    aHash.AddInteger<std::uint64_t>(theScene.selection.selected.size());
    for (const ElementIdentifier& anElement : theScene.selection.selected) {
        aHash.AddString(anElement.entityIdentifier);
        aHash.AddInteger(static_cast<std::uint8_t>(anElement.kind));
        aHash.AddInteger(anElement.topologyIndex);
        aHash.AddInteger(anElement.geometryRevision);
    }
    aHash.AddBool(theScene.selection.hovered.has_value());
    if (theScene.selection.hovered.has_value()) {
        const ElementIdentifier& anElement = *theScene.selection.hovered;
        aHash.AddString(anElement.entityIdentifier);
        aHash.AddInteger(static_cast<std::uint8_t>(anElement.kind));
        aHash.AddInteger(anElement.topologyIndex);
        aHash.AddInteger(anElement.geometryRevision);
    }
    return aHash.Value();
}

bool ValidatePresentationOverlayPayload(
    const PresentationOverlayKind theKind,
    const std::vector<MeshSnapshot>& theMeshes,
    const std::vector<InstanceSnapshot>& theInstances,
    const std::vector<MaterialSnapshot>& theMaterials,
    const std::vector<std::string>& theSuppressedEntityIdentifiers,
    const bool theHasPublishedGeometryRevisions)
{
    if (theMeshes.size() > kMaxOverlayMeshes
        || theInstances.size() > kMaxOverlayInstances
        || theMaterials.size() > kMaxOverlayMaterials) {
        return false;
    }
    const bool isEmpty = theMeshes.empty()
        && theInstances.empty() && theMaterials.empty();
    bool hasMirrorPlanePrefix = false;
    std::size_t aMirrorPreviewCount = 0;
    std::size_t aBooleanActorCount = 0;
    switch (theKind) {
        case PresentationOverlayKind::None:
            return isEmpty && theSuppressedEntityIdentifiers.empty();
        case PresentationOverlayKind::MoveRotateGizmo:
            if (isEmpty || theMeshes.size() != 7
                || theInstances.size() != 7
                || theMaterials.size() != 4) {
                return false;
            }
            break;
        case PresentationOverlayKind::ScaleGizmo:
            if (isEmpty || theMeshes.size() != 5
                || theInstances.size() != 5
                || theMaterials.size() != 4) {
                return false;
            }
            break;
        case PresentationOverlayKind::MirrorGizmo:
            if (isEmpty || theMeshes.size() != 6
                || theInstances.size() != 6
                || theMaterials.size() != 6) {
                return false;
            }
            hasMirrorPlanePrefix = true;
            break;
        case PresentationOverlayKind::MirrorPreview:
            if (theMeshes.size() <= 6
                || theMeshes.size() != theInstances.size()
                || theMeshes.size() != theMaterials.size()) {
                return false;
            }
            aMirrorPreviewCount = theMeshes.size() - 6;
            if (aMirrorPreviewCount == 0
                || aMirrorPreviewCount > kMaxMirrorPreviewBodies) {
                return false;
            }
            hasMirrorPlanePrefix = true;
            break;
        case PresentationOverlayKind::BooleanSubtractPreview: {
            const std::size_t anItemCount = theInstances.size();
            if (anItemCount < 2
                || anItemCount > kMaxBooleanSourceOperands
                || theMeshes.size() != anItemCount
                || theMaterials.size() != anItemCount
                || theSuppressedEntityIdentifiers.size() != anItemCount) {
                return false;
            }
            bool hasReachedResults = false;
            for (const InstanceSnapshot& anInstance : theInstances) {
                if (anInstance.role == RenderRole::BooleanActor
                    && !hasReachedResults) {
                    ++aBooleanActorCount;
                } else if (anInstance.role == RenderRole::BooleanSubject) {
                    hasReachedResults = true;
                } else {
                    return false;
                }
            }
            if (aBooleanActorCount == 0
                || aBooleanActorCount == anItemCount) {
                return false;
            }
            break;
        }
        case PresentationOverlayKind::BooleanUnionPreview:
        case PresentationOverlayKind::BooleanIntersectPreview:
            if (theMeshes.size() != 1 || theInstances.size() != 1
                || theMaterials.size() != 1
                || theSuppressedEntityIdentifiers.size() < 2
                || theSuppressedEntityIdentifiers.size()
                    > kMaxBooleanSourceOperands) {
                return false;
            }
            break;
        case PresentationOverlayKind::ChamferPreview: {
            const std::size_t anItemCount = theInstances.size();
            if (anItemCount == 0
                || anItemCount > kMaxChamferPreviewBodies
                || theMeshes.size() != anItemCount
                || theMaterials.size() != anItemCount
                || theSuppressedEntityIdentifiers.size() != anItemCount) {
                return false;
            }
            break;
        }
        case PresentationOverlayKind::LinearArrayPreview:
            if (!isEmpty
                && (theMeshes.size() != 1 || theMaterials.size() != 1
                    || theInstances.empty()
                    || theInstances.size()
                        > kMaxLinearArrayPreviewBodies)) {
                return false;
            }
            break;
        case PresentationOverlayKind::ShellPreview:
            if (theMeshes.size() != 1 || theInstances.size() != 1
                || theMaterials.size() != 1
                || theSuppressedEntityIdentifiers.size() != 1) {
                return false;
            }
            break;
        default:
            return false;
    }

    const bool isBooleanPreview =
        theKind == PresentationOverlayKind::BooleanSubtractPreview
        || theKind == PresentationOverlayKind::BooleanUnionPreview
        || theKind == PresentationOverlayKind::BooleanIntersectPreview;
    const bool isChamferPreview =
        theKind == PresentationOverlayKind::ChamferPreview;
    const bool isLinearArrayPreview =
        theKind == PresentationOverlayKind::LinearArrayPreview;
    const bool isShellPreview =
        theKind == PresentationOverlayKind::ShellPreview;
    if (!isBooleanPreview && !isChamferPreview && !isShellPreview
        && !theSuppressedEntityIdentifiers.empty()) {
        return false;
    }
    std::unordered_set<std::string> aSuppressedIdentifiers;
    aSuppressedIdentifiers.reserve(theSuppressedEntityIdentifiers.size());
    for (const std::string& anIdentifier :
         theSuppressedEntityIdentifiers) {
        if (!IsValidIdentifier(anIdentifier)
            || !aSuppressedIdentifiers.insert(anIdentifier).second) {
            return false;
        }
    }
    const auto aBooleanEntityIdentifier = [&](const std::size_t theIndex) {
        if (theKind == PresentationOverlayKind::BooleanUnionPreview) {
            return std::string("boolean/union/result/0");
        }
        if (theKind == PresentationOverlayKind::BooleanIntersectPreview) {
            return std::string("boolean/intersect/result/0");
        }
        if (theIndex < aBooleanActorCount) {
            return std::string("boolean/subtract/actor/")
                + std::to_string(theIndex);
        }
        return std::string("boolean/subtract/result/")
            + std::to_string(theIndex - aBooleanActorCount);
    };
    const auto aBooleanName = [&](const std::size_t theIndex) {
        if (theKind == PresentationOverlayKind::BooleanUnionPreview) {
            return std::string("Boolean union result 0");
        }
        if (theKind == PresentationOverlayKind::BooleanIntersectPreview) {
            return std::string("Boolean intersect result 0");
        }
        if (theIndex < aBooleanActorCount) {
            return std::string("Boolean subtract actor ")
                + std::to_string(theIndex);
        }
        return std::string("Boolean subtract result ")
            + std::to_string(theIndex - aBooleanActorCount);
    };
    const auto aChamferEntityIdentifier = [](const std::size_t theIndex) {
        return std::string("chamfer/preview/") + std::to_string(theIndex);
    };
    constexpr const char* kShellPreviewIdentifier = "shell/preview/0";

    std::unordered_set<std::string> aMaterialIdentifiers;
    aMaterialIdentifiers.reserve(theMaterials.size());
    for (std::size_t aMaterialIndex = 0;
         aMaterialIndex < theMaterials.size(); ++aMaterialIndex) {
        const MaterialSnapshot& aMaterial = theMaterials[aMaterialIndex];
        const auto isUnit = [](const float theValue) {
            return IsFinite(theValue) && theValue >= 0.0f && theValue <= 1.0f;
        };
        const bool isMirrorPlane =
            hasMirrorPlanePrefix && aMaterialIndex < 6;
        const bool isMirrorPreview =
            theKind == PresentationOverlayKind::MirrorPreview
            && aMaterialIndex >= 6;
        const bool isChamferMaterial = isChamferPreview;
        const bool isLinearArrayMaterial = isLinearArrayPreview;
        const bool isShellMaterial = isShellPreview;
        bool hasExpectedIdentifier = true;
        bool hasExpectedAlpha = true;
        bool hasExpectedColor = true;
        if (isMirrorPlane) {
            hasExpectedIdentifier = aMaterial.identifier
                == kMirrorMaterialIdentifiers[aMaterialIndex];
            const bool expectsBlend = aMaterialIndex >= 3;
            hasExpectedAlpha = expectsBlend
                ? aMaterial.alphaMode == AlphaMode::Blend
                    && std::abs(aMaterial.baseColor.w - 0.75f)
                        <= 1.0e-6f
                : aMaterial.alphaMode == AlphaMode::Opaque
                    && aMaterial.baseColor.w == 1.0f;
        } else if (isMirrorPreview) {
            hasExpectedIdentifier = aMaterial.identifier
                == "mirror/preview/"
                    + std::to_string(aMaterialIndex - 6)
                    + "/material";
            hasExpectedAlpha = aMaterial.baseColor.w == 1.0f
                && (aMaterial.alphaMode == AlphaMode::Opaque
                    || aMaterial.alphaMode == AlphaMode::Mask);
        } else if (isBooleanPreview) {
            hasExpectedIdentifier = aMaterial.identifier
                == aBooleanEntityIdentifier(aMaterialIndex) + "/material";
            hasExpectedAlpha = aMaterial.alphaMode == AlphaMode::Opaque
                && aMaterial.baseColor.w == 1.0f;
            const Quantity_Color anExpectedColor(
                aMaterialIndex < aBooleanActorCount
                    ? Quantity_NOC_ORANGE
                    : Quantity_NOC_LIGHTSKYBLUE);
            hasExpectedColor =
                std::abs(aMaterial.baseColor.x - anExpectedColor.Red())
                        <= 1.0e-6f
                && std::abs(aMaterial.baseColor.y - anExpectedColor.Green())
                        <= 1.0e-6f
                && std::abs(aMaterial.baseColor.z - anExpectedColor.Blue())
                        <= 1.0e-6f;
        } else if (isChamferMaterial) {
            hasExpectedIdentifier = aMaterial.identifier
                == aChamferEntityIdentifier(aMaterialIndex) + "/material";
            hasExpectedAlpha = aMaterial.alphaMode == AlphaMode::Opaque
                && aMaterial.baseColor.w == 1.0f;
        } else if (isLinearArrayMaterial) {
            hasExpectedIdentifier = aMaterialIndex == 0
                && aMaterial.identifier
                    == "linear-array/source/0/material";
            hasExpectedAlpha = aMaterial.alphaMode == AlphaMode::Opaque
                && aMaterial.baseColor.w == 1.0f;
        } else if (isShellMaterial) {
            hasExpectedIdentifier = aMaterialIndex == 0
                && aMaterial.identifier
                    == std::string(kShellPreviewIdentifier) + "/material";
            hasExpectedAlpha = aMaterial.alphaMode == AlphaMode::Opaque
                && aMaterial.baseColor.w == 1.0f;
        } else {
            hasExpectedAlpha = aMaterial.alphaMode == AlphaMode::Opaque
                && aMaterial.baseColor.w == 1.0f;
        }
        if (!hasExpectedIdentifier || !hasExpectedColor
            || !IsValidIdentifier(aMaterial.identifier)
            || !aMaterialIdentifiers.insert(aMaterial.identifier).second
            || !isUnit(aMaterial.baseColor.x)
            || !isUnit(aMaterial.baseColor.y)
            || !isUnit(aMaterial.baseColor.z)
            || !hasExpectedAlpha
            || ((isChamferMaterial || isLinearArrayMaterial
                    || isShellMaterial)
                && (aMaterial.baseColorTextureIndex != -1
                    || aMaterial.emissiveTextureIndex != -1))
            || !IsFinite(aMaterial.emission.x)
            || !IsFinite(aMaterial.emission.y)
            || !IsFinite(aMaterial.emission.z)
            || aMaterial.emission.x < 0.0f
            || aMaterial.emission.y < 0.0f
            || aMaterial.emission.z < 0.0f
            || !isUnit(aMaterial.metallic)
            || !isUnit(aMaterial.roughness)
            || !IsFinite(aMaterial.indexOfRefraction)
            || aMaterial.indexOfRefraction <= 0.0f
            || !isUnit(aMaterial.alphaCutoff)) {
            return false;
        }
    }

    std::size_t aVertexCount = 0;
    std::size_t anIndexCount = 0;
    std::size_t aPrimitiveCount = 0;
    std::size_t aNumericByteCount = 0;
    std::unordered_set<std::string> aDefinitionIdentifiers;
    aDefinitionIdentifiers.reserve(theMeshes.size());
    for (std::size_t aMeshIndex = 0;
         aMeshIndex < theMeshes.size(); ++aMeshIndex) {
        const MeshSnapshot& aMesh = theMeshes[aMeshIndex];
        const bool isMirrorPlane = hasMirrorPlanePrefix && aMeshIndex < 6;
        const bool isMirrorPreview =
            theKind == PresentationOverlayKind::MirrorPreview
            && aMeshIndex >= 6;
        const bool isBooleanMesh = isBooleanPreview;
        const bool isChamferMesh = isChamferPreview;
        const bool isLinearArrayMesh = isLinearArrayPreview;
        const bool isShellMesh = isShellPreview;
        const std::string anExpectedPreviewIdentifier = isMirrorPreview
            ? "mirror/preview/" + std::to_string(aMeshIndex - 6) + "/mesh"
            : std::string();
        if ((isMirrorPlane
                && aMesh.definitionIdentifier
                    != kMirrorMeshIdentifiers[aMeshIndex])
            || (isMirrorPreview
                && aMesh.definitionIdentifier
                    != anExpectedPreviewIdentifier)
            || (isBooleanMesh
                && aMesh.definitionIdentifier
                    != aBooleanEntityIdentifier(aMeshIndex) + "/mesh")
            || (isChamferMesh
                && aMesh.definitionIdentifier
                    != aChamferEntityIdentifier(aMeshIndex) + "/mesh")
            || (isLinearArrayMesh
                && (aMeshIndex != 0
                    || aMesh.definitionIdentifier
                        != "linear-array/source/0/mesh"))
            || (isShellMesh
                && (aMeshIndex != 0
                    || aMesh.definitionIdentifier
                        != std::string(kShellPreviewIdentifier) + "/mesh"))
            || !IsValidIdentifier(aMesh.definitionIdentifier)
            || !aDefinitionIdentifiers.insert(
                aMesh.definitionIdentifier).second
            || (theHasPublishedGeometryRevisions
                    ? aMesh.geometryRevision == 0
                    : aMesh.geometryRevision != 0)
            || !IsValid(aMesh.localBounds)
            || ((isMirrorPreview || isBooleanMesh || isChamferMesh
                    || isLinearArrayMesh || isShellMesh)
                && !IsCenteredLocalBounds(aMesh.localBounds))
            || aMesh.vertices.empty() || aMesh.indices.empty()
            || aMesh.primitives.empty()
            || (!isMirrorPreview && !isBooleanMesh && !isChamferMesh
                    && !isLinearArrayMesh && !isShellMesh
                && aMesh.primitives.size() != 1)
            || !CheckedAdd(aPrimitiveCount,
                           aMesh.primitives.size(),
                           aPrimitiveCount)
            || aPrimitiveCount > kMaxOverlayPrimitives
            || !CheckedAdd(aVertexCount,
                           aMesh.vertices.size(),
                           aVertexCount)
            || aVertexCount > kMaxOverlayVertices
            || !CheckedAdd(anIndexCount,
                           aMesh.indices.size(),
                           anIndexCount)
            || anIndexCount > kMaxOverlayIndices) {
            return false;
        }
        std::size_t anExpectedFirstIndex = 0;
        for (std::size_t aPrimitiveIndex = 0;
             aPrimitiveIndex < aMesh.primitives.size();
             ++aPrimitiveIndex) {
            const MeshPrimitive& aPrimitive =
                aMesh.primitives[aPrimitiveIndex];
            std::size_t anIndexEnd = 0;
            if (aPrimitive.firstIndex != anExpectedFirstIndex
                || aPrimitive.indexCount == 0
                || aPrimitive.indexCount % 3 != 0
                || aPrimitive.faceIndex != aPrimitiveIndex
                || !CheckedAdd(
                    static_cast<std::size_t>(aPrimitive.firstIndex),
                    static_cast<std::size_t>(aPrimitive.indexCount),
                    anIndexEnd)
                || anIndexEnd > aMesh.indices.size()) {
                return false;
            }
            anExpectedFirstIndex = anIndexEnd;
        }
        if (anExpectedFirstIndex != aMesh.indices.size()) {
            return false;
        }
        std::size_t aVertexBytes = 0;
        std::size_t anIndexBytes = 0;
        if (!CheckedMultiply(aMesh.vertices.size(), sizeof(Vertex), aVertexBytes)
            || !CheckedMultiply(aMesh.indices.size(),
                                sizeof(std::uint32_t),
                                anIndexBytes)
            || !CheckedAdd(aNumericByteCount,
                           aVertexBytes,
                           aNumericByteCount)
            || !CheckedAdd(aNumericByteCount,
                           anIndexBytes,
                           aNumericByteCount)
            || aNumericByteCount > kMaxOverlayNumericBytes) {
            return false;
        }
        for (const Vertex& aVertex : aMesh.vertices) {
            const double aNormalSquared =
                static_cast<double>(aVertex.normalX) * aVertex.normalX
                + static_cast<double>(aVertex.normalY) * aVertex.normalY
                + static_cast<double>(aVertex.normalZ) * aVertex.normalZ;
            if (!IsFinite(aVertex.positionX) || !IsFinite(aVertex.positionY)
                || !IsFinite(aVertex.positionZ) || !IsFinite(aVertex.normalX)
                || !IsFinite(aVertex.normalY) || !IsFinite(aVertex.normalZ)
                || !IsFinite(aVertex.textureU) || !IsFinite(aVertex.textureV)
                || !IsFinite(aNormalSquared) || aNormalSquared <= 1.0e-12) {
                return false;
            }
        }
        for (const std::uint32_t anIndex : aMesh.indices) {
            if (anIndex >= aMesh.vertices.size()) {
                return false;
            }
        }
    }

    std::vector<std::uint8_t> aMeshReferences(theMeshes.size(), 0);
    std::vector<std::uint8_t> aMaterialReferences(theMaterials.size(), 0);
    std::unordered_set<std::string> anEntityIdentifiers;
    anEntityIdentifiers.reserve(theInstances.size());
    std::optional<std::array<double, 16>> aMirrorWorldAnchor;
    std::size_t aPrimitiveBindingCount = 0;
    for (std::size_t anInstanceIndex = 0;
         anInstanceIndex < theInstances.size(); ++anInstanceIndex) {
        const InstanceSnapshot& anInstance = theInstances[anInstanceIndex];
        const bool isMirrorPlane =
            hasMirrorPlanePrefix && anInstanceIndex < 6;
        const bool isMirrorPreview =
            theKind == PresentationOverlayKind::MirrorPreview
            && anInstanceIndex >= 6;
        const bool isBooleanItem = isBooleanPreview;
        const bool isChamferItem = isChamferPreview;
        const bool isLinearArrayItem = isLinearArrayPreview;
        const bool isShellItem = isShellPreview;
        const std::size_t aPreviewIndex = isMirrorPreview
            ? anInstanceIndex - 6
            : 0;
        const std::string anExpectedPreviewIdentifier = isMirrorPreview
            ? "mirror/preview/" + std::to_string(aPreviewIndex)
            : std::string();
        const bool hasExpectedIdentity = isMirrorPlane
            ? anInstance.entityIdentifier
                    == kMirrorEntityIdentifiers[anInstanceIndex]
                && anInstance.name == kMirrorNames[anInstanceIndex]
                && anInstance.meshIndex == anInstanceIndex
            : isMirrorPreview
                ? anInstance.entityIdentifier
                        == anExpectedPreviewIdentifier
                    && anInstance.name
                        == "Mirror preview " + std::to_string(aPreviewIndex)
                    && anInstance.meshIndex == anInstanceIndex
                : isBooleanItem
                    ? anInstance.entityIdentifier
                            == aBooleanEntityIdentifier(anInstanceIndex)
                        && anInstance.name == aBooleanName(anInstanceIndex)
                        && anInstance.meshIndex == anInstanceIndex
                    : isChamferItem
                        ? anInstance.entityIdentifier
                                == aChamferEntityIdentifier(anInstanceIndex)
                            && anInstance.name
                                == "Chamfer preview "
                                    + std::to_string(anInstanceIndex)
                            && anInstance.meshIndex == anInstanceIndex
                    : isLinearArrayItem
                        ? anInstance.entityIdentifier
                                == "linear-array/preview/0/"
                                    + std::to_string(anInstanceIndex + 1U)
                            && anInstance.name
                                == "Linear array preview "
                                    + std::to_string(anInstanceIndex + 1U)
                            && anInstance.meshIndex == 0
                    : isShellItem
                        ? anInstance.entityIdentifier
                                == kShellPreviewIdentifier
                            && anInstance.name == "Shell preview 0"
                            && anInstance.meshIndex == 0
                    : true;
        const bool hasExpectedSemantics = isBooleanItem
            ? anInstance.coordinateSpace == CoordinateSpace::World
                && anInstance.depthPolicy == DepthPolicy::Scene
                && (anInstanceIndex < aBooleanActorCount
                    ? anInstance.role == RenderRole::BooleanActor
                        && anInstance.renderStyle == RenderStyle::Wireframe
                    : anInstance.role == RenderRole::BooleanSubject
                        && anInstance.renderStyle == RenderStyle::Shaded)
            : (isMirrorPreview || isChamferItem || isLinearArrayItem
                    || isShellItem)
                ? anInstance.role == (isChamferItem
                        ? RenderRole::ChamferPreview
                        : isLinearArrayItem
                            ? RenderRole::LinearArrayPreview
                            : isShellItem
                                ? RenderRole::ShellPreview
                            : RenderRole::MirrorPreview)
                    && anInstance.coordinateSpace == CoordinateSpace::World
                    && anInstance.depthPolicy == DepthPolicy::Scene
                : anInstance.role == RenderRole::Gizmo
                    && anInstance.coordinateSpace
                        == CoordinateSpace::WorldAnchorPixels
                    && anInstance.depthPolicy == DepthPolicy::Topmost;
        const std::size_t anExpectedBindingCount =
            (isMirrorPreview || isBooleanItem || isChamferItem
                || isLinearArrayItem || isShellItem)
            ? theMeshes[isLinearArrayItem ? 0 : anInstanceIndex]
                .primitives.size()
            : 1;
        if (!hasExpectedIdentity
            || !IsValidIdentifier(anInstance.entityIdentifier)
            || !anEntityIdentifiers.insert(
                anInstance.entityIdentifier).second
            || anInstance.name.size() > 4'096
            || anInstance.meshIndex >= theMeshes.size()
            || anInstance.reversesWinding || !anInstance.visible
            || anInstance.selectable || anInstance.selected
            || !hasExpectedSemantics
            || (!isBooleanItem && !isChamferItem && !isShellItem
                && anInstance.renderStyle != RenderStyle::Shaded)
            || ((isChamferItem || isShellItem)
                && anInstance.renderStyle != RenderStyle::Shaded)
            || ((isMirrorPreview || isBooleanItem || isChamferItem
                    || isLinearArrayItem || isShellItem)
                ? !IsTranslationOnlyWorldTransform(
                    anInstance.worldFromObject)
                : !IsRigidWorldAnchorTransform(
                    anInstance.worldFromObject))
            || anInstance.primitiveBindings.size()
                != anExpectedBindingCount
            || !CheckedAdd(aPrimitiveBindingCount,
                           anInstance.primitiveBindings.size(),
                           aPrimitiveBindingCount)
            || aPrimitiveBindingCount
                > kMaxOverlayPrimitiveBindings) {
            return false;
        }
        if (++aMeshReferences[anInstance.meshIndex] != 1
            && !isLinearArrayItem) {
            return false;
        }
        for (const PrimitiveBinding& aBinding :
             anInstance.primitiveBindings) {
            if (((isMirrorPlane || isMirrorPreview
                    || isBooleanItem || isChamferItem
                    || isLinearArrayItem || isShellItem)
                    && aBinding.materialIndex
                        != (isLinearArrayItem ? 0 : anInstanceIndex))
                || aBinding.materialIndex >= theMaterials.size()
                || aBinding.pickToken != 0 || !aBinding.visible) {
                return false;
            }
            aMaterialReferences[aBinding.materialIndex] = 1;
        }
        if (isMirrorPlane) {
            if (!aMirrorWorldAnchor.has_value()) {
                aMirrorWorldAnchor = anInstance.worldFromObject.values;
            } else if (*aMirrorWorldAnchor
                       != anInstance.worldFromObject.values) {
                return false;
            }
        }
    }
    for (const std::string& aSuppressed :
         theSuppressedEntityIdentifiers) {
        if (anEntityIdentifiers.find(aSuppressed)
            != anEntityIdentifiers.end()) {
            return false;
        }
    }
    if (isLinearArrayPreview) {
        return isEmpty
            || (aMeshReferences.size() == 1
            && aMeshReferences[0] == theInstances.size()
            && aMaterialReferences.size() == 1
            && aMaterialReferences[0] == 1);
    }
    return std::all_of(aMeshReferences.begin(), aMeshReferences.end(),
                       [](const std::uint8_t theCount) {
                           return theCount == 1;
                       })
        && std::all_of(aMaterialReferences.begin(), aMaterialReferences.end(),
                       [](const std::uint8_t theCount) {
                           return theCount == 1;
                       });
}

std::uint64_t PresentationOverlayPayloadFingerprint(
    const PresentationOverlaySnapshot& theOverlay)
{
    Fingerprint aHash;
    // Base compatibility is published and validated separately. This revision
    // identifies only immutable overlay content, so camera-only full captures
    // do not advance it when the gizmo itself is unchanged.
    aHash.AddInteger(static_cast<std::uint8_t>(theOverlay.kind));
    aHash.AddInteger<std::uint64_t>(
        theOverlay.suppressedEntityIdentifiers.size());
    for (const std::string& anIdentifier :
         theOverlay.suppressedEntityIdentifiers) {
        aHash.AddString(anIdentifier);
    }
    aHash.AddInteger<std::uint64_t>(theOverlay.meshes.size());
    for (const MeshSnapshot& aMesh : theOverlay.meshes) {
        aHash.AddInteger(MeshFingerprint(aMesh, 0.0, 0.0));
        aHash.AddInteger(aMesh.geometryRevision);
    }
    aHash.AddInteger<std::uint64_t>(theOverlay.materials.size());
    for (const MaterialSnapshot& aMaterial : theOverlay.materials) {
        aHash.AddString(aMaterial.identifier);
        AddMaterialValues(aHash, aMaterial);
    }
    aHash.AddInteger<std::uint64_t>(theOverlay.instances.size());
    for (const InstanceSnapshot& anInstance : theOverlay.instances) {
        aHash.AddString(anInstance.entityIdentifier);
        aHash.AddInteger(anInstance.meshIndex);
        for (const double aValue : anInstance.worldFromObject.values) {
            aHash.AddDouble(aValue);
        }
        aHash.AddBool(anInstance.reversesWinding);
        aHash.AddBool(anInstance.visible);
        aHash.AddBool(anInstance.selectable);
        aHash.AddBool(anInstance.selected);
        aHash.AddString(anInstance.name);
        aHash.AddInteger(static_cast<std::uint8_t>(anInstance.role));
        aHash.AddInteger(
            static_cast<std::uint8_t>(anInstance.coordinateSpace));
        aHash.AddInteger(
            static_cast<std::uint8_t>(anInstance.depthPolicy));
        aHash.AddInteger(
            static_cast<std::uint8_t>(anInstance.renderStyle));
        for (const PrimitiveBinding& aBinding :
             anInstance.primitiveBindings) {
            aHash.AddInteger(aBinding.materialIndex);
            aHash.AddInteger(aBinding.pickToken);
            aHash.AddBool(aBinding.visible);
        }
    }
    return aHash.Value();
}

} // namespace

struct OcctSceneSnapshotBuilder::State {
    using LabelInstanceMap =
        std::unordered_map<std::string, std::vector<std::size_t>>;
    struct DefinitionRevision {
        std::uint64_t fingerprint = 0;
        std::uint64_t revision = 0;
        std::uint64_t lastSeenSnapshot = 0;
        bool hasFingerprint = false;
    };

    //! Small, independently transactional state for transient presentation.
    //! Live payloads remain bounded by kMaxOverlayMeshes. Retained revisions
    //! cover the stable union of supported gizmos so revisiting a tool cannot
    //! recycle a geometry revision already held in a renderer cache.
    struct OverlayState {
        std::uint64_t revision = 0;
        std::optional<std::uint64_t> fingerprint;
        std::unordered_map<std::string, DefinitionRevision> definitions;

        void Swap(OverlayState& theOther) noexcept
        {
            std::swap(revision, theOther.revision);
            fingerprint.swap(theOther.fingerprint);
            definitions.swap(theOther.definitions);
        }
    };

    Handle(TDocStd_Document) documentObject;
    std::string publicationSourceIdentifier;
    std::string documentIdentifier;
    std::uint64_t snapshotRevision = 0;
    std::uint64_t lastFullSnapshotRevision = 0;
    std::uint64_t documentGeneration = 0;
    std::uint64_t modelRevision = 0;
    std::uint64_t presentationRevision = 0;
    std::uint64_t cameraRevision = 0;
    Standard_Integer lastFullDocumentTime = 0;
    std::optional<std::uint64_t> modelFingerprint;
    std::optional<std::uint64_t> presentationFingerprint;
    std::optional<std::uint64_t> cameraFingerprint;
    std::unordered_map<std::string, DefinitionRevision> definitions;
    //! Shared immutable lookup for the exact last full snapshot. State copies
    //! remain bounded during overlay publication because only these pointers are
    //! copied, never the potentially large committed-scene maps.
    std::shared_ptr<const LabelInstanceMap> lastFullLabelToInstances;
    std::shared_ptr<const std::vector<std::string>> lastFullEntityIdentifiers;
    OverlayState overlay;
#ifdef DEBUG
    DebugTriangulationFailure debugTriangulationFailure =
        DebugTriangulationFailure::None;
    std::uint64_t debugMesherInvocationCount = 0;
#endif
};

OcctSceneSnapshotBuilder::OcctSceneSnapshotBuilder()
: myState(std::make_unique<State>())
{
    NSString *aSourceIdentifier = NSUUID.UUID.UUIDString;
    const char *aUtf8 = aSourceIdentifier.UTF8String;
    if (aUtf8 != nullptr) {
        myState->publicationSourceIdentifier = aUtf8;
    }
}

OcctSceneSnapshotBuilder::~OcctSceneSnapshotBuilder() = default;

#ifdef DEBUG
void OcctSceneSnapshotBuilder::DebugSetTriangulationFailure(
    const DebugTriangulationFailure theFailure) noexcept
{
    if (myState != nullptr) {
        myState->debugTriangulationFailure = theFailure;
    }
}

void OcctSceneSnapshotBuilder::DebugResetMesherInvocationCount() noexcept
{
    if (myState != nullptr) {
        myState->debugMesherInvocationCount = 0;
    }
}

std::uint64_t OcctSceneSnapshotBuilder::DebugMesherInvocationCount() const noexcept
{
    return myState == nullptr ? 0 : myState->debugMesherInvocationCount;
}
#endif

std::optional<FrameSnapshot> OcctSceneSnapshotBuilder::CaptureFrame(
    const Handle(OcctDocument)& theDocument,
    const Handle(V3d_View)& theView,
    const UInt2& theViewportPixels) noexcept
{
    if (![NSThread isMainThread]
        || theDocument.IsNull()
        || theView.IsNull()
        || theViewportPixels.x == 0
        || theViewportPixels.y == 0
        || myState == nullptr
        || myState->publicationSourceIdentifier.empty()
        || myState->documentObject.IsNull()
        || myState->documentGeneration == 0
        || myState->snapshotRevision == 0) {
        return std::nullopt;
    }

    try {
        OCC_CATCH_SIGNALS

        const Handle(TDocStd_Document)& aDocument = theDocument->Document();
        if (aDocument.IsNull()
            || aDocument->HasOpenCommand()
            || aDocument.get() != myState->documentObject.get()
            || theDocument->DocumentIdentifier() != myState->documentIdentifier) {
            return std::nullopt;
        }

        FrameSnapshot aFrame;
        aFrame.publicationSourceIdentifier =
            myState->publicationSourceIdentifier;
        if (!BuildCamera(theView, theViewportPixels, aFrame.camera)) {
            return std::nullopt;
        }

        const std::uint64_t aFingerprint = CameraFingerprint(aFrame.camera);
        std::uint64_t aCameraRevision = myState->cameraRevision;
        std::uint64_t aSnapshotRevision = myState->snapshotRevision;
        const bool isChanged = !myState->cameraFingerprint.has_value()
            || *myState->cameraFingerprint != aFingerprint;
        if (isChanged
            && (!IncrementRevision(aCameraRevision)
                || !IncrementRevision(aSnapshotRevision))) {
            return std::nullopt;
        }

        aFrame.revisions.snapshot = aSnapshotRevision;
        aFrame.revisions.documentGeneration = myState->documentGeneration;
        aFrame.revisions.model = myState->modelRevision;
        aFrame.revisions.presentation = myState->presentationRevision;
        aFrame.revisions.camera = aCameraRevision;

        if (isChanged) {
            myState->cameraFingerprint = aFingerprint;
            myState->cameraRevision = aCameraRevision;
            myState->snapshotRevision = aSnapshotRevision;
        }
        return aFrame;
    } catch (const Standard_Failure&) {
        return std::nullopt;
    } catch (...) {
        return std::nullopt;
    }
}

OcctSceneSnapshotBuilder::OverlayPointer
OcctSceneSnapshotBuilder::PublishPresentationOverlay(
    const Handle(OcctDocument)& theDocument,
    PresentationOverlayContent&& theContent) noexcept
{
    return PublishPresentationOverlayImpl(
        theDocument,
        std::move(theContent),
        false,
        false,
        false,
        false,
        false);
}

OcctSceneSnapshotBuilder::OverlayPointer
OcctSceneSnapshotBuilder::PublishPresentationOverlayImpl(
    const Handle(OcctDocument)& theDocument,
    PresentationOverlayContent&& theContent,
    const bool theAllowsMirrorPreview,
    const bool theAllowsBooleanPreview,
    const bool theAllowsChamferPreview,
    const bool theAllowsLinearArrayPreview,
    const bool theAllowsShellPreview) noexcept
{
    if (![NSThread isMainThread]
        || theDocument.IsNull()
        || (theContent.kind == PresentationOverlayKind::MirrorPreview
            && !theAllowsMirrorPreview)
        || ((theContent.kind
                == PresentationOverlayKind::BooleanSubtractPreview
             || theContent.kind
                == PresentationOverlayKind::BooleanUnionPreview
             || theContent.kind
                == PresentationOverlayKind::BooleanIntersectPreview)
            && !theAllowsBooleanPreview)
        || (theContent.kind == PresentationOverlayKind::ChamferPreview
            && !theAllowsChamferPreview)
        || (theContent.kind
                == PresentationOverlayKind::LinearArrayPreview
            && !theAllowsLinearArrayPreview)
        || (theContent.kind == PresentationOverlayKind::ShellPreview
            && !theAllowsShellPreview)
        || myState == nullptr
        || myState->publicationSourceIdentifier.empty()
        || myState->documentObject.IsNull()
        || myState->lastFullSnapshotRevision == 0
        || myState->documentGeneration == 0
        || myState->modelRevision == 0
        || myState->presentationRevision == 0) {
        return {};
    }

    try {
        OCC_CATCH_SIGNALS

        const Handle(TDocStd_Document)& aDocument = theDocument->Document();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()
            || aDocument.get() != myState->documentObject.get()
            || theDocument->DocumentIdentifier()
                != myState->documentIdentifier) {
            return {};
        }
        const Handle(TDF_Data)& aData = aDocument->GetData();
        if (aData.IsNull()
            || aData->Time() != myState->lastFullDocumentTime
            || !ValidatePresentationOverlayPayload(
                theContent.kind,
                theContent.meshes,
                theContent.instances,
                theContent.materials,
                theContent.suppressedEntityIdentifiers,
                false)) {
            return {};
        }

        // Copy only bounded transient state. The committed definition cache can
        // contain hundreds of thousands of entries and must not be copied for
        // a bounded transform-overlay publication.
        State::OverlayState aNextOverlayState = myState->overlay;
        PresentationOverlaySnapshot anOverlay;
        anOverlay.kind = theContent.kind;
        anOverlay.publicationSourceIdentifier =
            myState->publicationSourceIdentifier;
        anOverlay.baseSnapshotRevision =
            myState->lastFullSnapshotRevision;
        anOverlay.baseDocumentGeneration =
            myState->documentGeneration;
        anOverlay.baseModelRevision = myState->modelRevision;
        anOverlay.basePresentationRevision =
            myState->presentationRevision;
        anOverlay.meshes = std::move(theContent.meshes);
        anOverlay.instances = std::move(theContent.instances);
        anOverlay.materials = std::move(theContent.materials);
        anOverlay.suppressedEntityIdentifiers =
            std::move(theContent.suppressedEntityIdentifiers);

        for (MeshSnapshot& aMesh : anOverlay.meshes) {
            auto aRevisionFound = aNextOverlayState.definitions.find(
                aMesh.definitionIdentifier);
            if (aRevisionFound == aNextOverlayState.definitions.end()) {
                if (aNextOverlayState.definitions.size()
                    >= kMaxRetainedOverlayDefinitions) {
                    return {};
                }
                aRevisionFound = aNextOverlayState.definitions.emplace(
                    aMesh.definitionIdentifier,
                    State::DefinitionRevision()).first;
            }
            State::DefinitionRevision& aRevision = aRevisionFound->second;
            const std::uint64_t aFingerprint =
                MeshFingerprint(aMesh, 0.0, 0.0);
            if (!aRevision.hasFingerprint
                || aRevision.fingerprint != aFingerprint) {
                if (!IncrementRevision(aRevision.revision)) {
                    return {};
                }
                aRevision.fingerprint = aFingerprint;
                aRevision.hasFingerprint = true;
            }
            aRevision.lastSeenSnapshot =
                myState->lastFullSnapshotRevision;
            aMesh.geometryRevision = aRevision.revision;
        }
        if (!ValidatePresentationOverlayPayload(
                anOverlay.kind,
                anOverlay.meshes,
                anOverlay.instances,
                anOverlay.materials,
                anOverlay.suppressedEntityIdentifiers,
                true)) {
            return {};
        }

        const std::uint64_t aFingerprint =
            PresentationOverlayPayloadFingerprint(anOverlay);
        if (!aNextOverlayState.fingerprint.has_value()
            || *aNextOverlayState.fingerprint != aFingerprint) {
            if (!IncrementRevision(aNextOverlayState.revision)) {
                return {};
            }
            aNextOverlayState.fingerprint = aFingerprint;
        }
        if (aNextOverlayState.revision == 0) {
            return {};
        }
        anOverlay.overlayRevision = aNextOverlayState.revision;

        OverlayPointer aSnapshot =
            std::make_shared<const PresentationOverlaySnapshot>(
                std::move(anOverlay));
        // Allocation and validation are complete. This bounded swap is the
        // only mutation and cannot expose a partially advanced revision.
        myState->overlay.Swap(aNextOverlayState);
        return aSnapshot;
    } catch (const Standard_Failure&) {
        return {};
    } catch (...) {
        return {};
    }
}

OcctSceneSnapshotBuilder::OverlayPointer
OcctSceneSnapshotBuilder::PublishMirrorPreviewOverlay(
    const Handle(OcctDocument)& theDocument,
    PresentationOverlayContent&& theMirrorGizmoContent,
    const std::vector<Handle(AIS_Shape)>& thePreviewShapes) noexcept
{
    if (![NSThread isMainThread]
        || theMirrorGizmoContent.kind
            != PresentationOverlayKind::MirrorPreview
        || thePreviewShapes.empty()
        || thePreviewShapes.size() > kMaxMirrorPreviewBodies) {
        return {};
    }

    try {
        OCC_CATCH_SIGNALS

        // The interactor changes only the semantic kind after capturing the
        // manipulator. Validate the six immutable plane slots in isolation
        // before appending any trial geometry.
        theMirrorGizmoContent.kind =
            PresentationOverlayKind::MirrorGizmo;
        if (!ValidatePresentationOverlayPayload(
                theMirrorGizmoContent.kind,
                theMirrorGizmoContent.meshes,
                theMirrorGizmoContent.instances,
                theMirrorGizmoContent.materials,
                theMirrorGizmoContent.suppressedEntityIdentifiers,
                false)) {
            return {};
        }
        theMirrorGizmoContent.kind =
            PresentationOverlayKind::MirrorPreview;

        std::size_t aVertexCount = 0;
        std::size_t anIndexCount = 0;
        std::size_t aPrimitiveCount = 0;
        std::size_t aBindingCount = 0;
        std::size_t aNumericByteCount = 0;
        for (const MeshSnapshot& aMesh : theMirrorGizmoContent.meshes) {
            std::size_t aVertexBytes = 0;
            std::size_t anIndexBytes = 0;
            if (!CheckedAdd(aVertexCount,
                            aMesh.vertices.size(),
                            aVertexCount)
                || !CheckedAdd(anIndexCount,
                               aMesh.indices.size(),
                               anIndexCount)
                || !CheckedAdd(aPrimitiveCount,
                               aMesh.primitives.size(),
                               aPrimitiveCount)
                || !CheckedMultiply(aMesh.vertices.size(),
                                    sizeof(Vertex),
                                    aVertexBytes)
                || !CheckedMultiply(aMesh.indices.size(),
                                    sizeof(std::uint32_t),
                                    anIndexBytes)
                || !CheckedAdd(aNumericByteCount,
                               aVertexBytes,
                               aNumericByteCount)
                || !CheckedAdd(aNumericByteCount,
                               anIndexBytes,
                               aNumericByteCount)) {
                return {};
            }
        }
        for (const InstanceSnapshot& anInstance :
             theMirrorGizmoContent.instances) {
            if (!CheckedAdd(aBindingCount,
                            anInstance.primitiveBindings.size(),
                            aBindingCount)) {
                return {};
            }
        }

        std::unordered_set<const AIS_Shape*> aSeenPresentations;
        aSeenPresentations.reserve(thePreviewShapes.size());
        for (std::size_t aPreviewIndex = 0;
             aPreviewIndex < thePreviewShapes.size();
             ++aPreviewIndex) {
            const Handle(AIS_Shape)& aPresentation =
                thePreviewShapes[aPreviewIndex];
            if (aPresentation.IsNull()
                || !aSeenPresentations.insert(aPresentation.get()).second
                || aVertexCount >= kMaxOverlayVertices
                || anIndexCount >= kMaxOverlayIndices
                || aPrimitiveCount >= kMaxOverlayPrimitives
                || aBindingCount >= kMaxOverlayPrimitiveBindings) {
                return {};
            }

            WorldPreviewItem anItem;
            const std::string anIdentifier =
                "mirror/preview/" + std::to_string(aPreviewIndex);
            if (!ExtractWorldPreviewItem(
                    aPresentation,
                    anIdentifier,
                    "Mirror preview " + std::to_string(aPreviewIndex),
                    static_cast<std::uint32_t>(6 + aPreviewIndex),
                    RenderRole::MirrorPreview,
                    RenderStyle::Shaded,
                    std::nullopt,
                    kMaxOverlayVertices - aVertexCount,
                    kMaxOverlayIndices - anIndexCount,
                    std::min(kMaxOverlayPrimitives - aPrimitiveCount,
                             kMaxOverlayPrimitiveBindings - aBindingCount),
                    anItem)) {
                return {};
            }

            std::size_t aVertexBytes = 0;
            std::size_t anIndexBytes = 0;
            if (!CheckedAdd(aVertexCount,
                            anItem.mesh.vertices.size(),
                            aVertexCount)
                || aVertexCount > kMaxOverlayVertices
                || !CheckedAdd(anIndexCount,
                               anItem.mesh.indices.size(),
                               anIndexCount)
                || anIndexCount > kMaxOverlayIndices
                || !CheckedAdd(aPrimitiveCount,
                               anItem.mesh.primitives.size(),
                               aPrimitiveCount)
                || aPrimitiveCount > kMaxOverlayPrimitives
                || !CheckedAdd(aBindingCount,
                               anItem.instance.primitiveBindings.size(),
                               aBindingCount)
                || aBindingCount > kMaxOverlayPrimitiveBindings
                || !CheckedMultiply(anItem.mesh.vertices.size(),
                                    sizeof(Vertex),
                                    aVertexBytes)
                || !CheckedMultiply(anItem.mesh.indices.size(),
                                    sizeof(std::uint32_t),
                                    anIndexBytes)
                || !CheckedAdd(aNumericByteCount,
                               aVertexBytes,
                               aNumericByteCount)
                || !CheckedAdd(aNumericByteCount,
                               anIndexBytes,
                               aNumericByteCount)
                || aNumericByteCount > kMaxOverlayNumericBytes) {
                return {};
            }
            theMirrorGizmoContent.meshes.push_back(
                std::move(anItem.mesh));
            theMirrorGizmoContent.instances.push_back(
                std::move(anItem.instance));
            theMirrorGizmoContent.materials.push_back(
                std::move(anItem.material));
        }

        if (theMirrorGizmoContent.meshes.size()
                > kMaxOverlayMeshes
            || theMirrorGizmoContent.instances.size()
                > kMaxOverlayInstances
            || theMirrorGizmoContent.materials.size()
                > kMaxOverlayMaterials) {
            return {};
        }
        return PublishPresentationOverlayImpl(
            theDocument,
            std::move(theMirrorGizmoContent),
            true,
            false,
            false,
            false,
            false);
    } catch (const Standard_Failure&) {
        return {};
    } catch (...) {
        return {};
    }
}

OcctSceneSnapshotBuilder::OverlayPointer
OcctSceneSnapshotBuilder::PublishBooleanPreviewOverlay(
    const Handle(OcctDocument)& theDocument,
    const PresentationOverlayKind theKind,
    const std::vector<Handle(AIS_Shape)>& theActorShapes,
    const std::vector<Handle(AIS_Shape)>& theResultShapes,
    const std::vector<TDF_Label>& theSuppressedSourceLabels) noexcept
{
    const bool isSubtract =
        theKind == PresentationOverlayKind::BooleanSubtractPreview;
    const bool isUnion =
        theKind == PresentationOverlayKind::BooleanUnionPreview;
    const bool isIntersect =
        theKind == PresentationOverlayKind::BooleanIntersectPreview;
    const bool isSingleResult = isUnion || isIntersect;
    const std::size_t anItemCount =
        theActorShapes.size() + theResultShapes.size();
    if (![NSThread isMainThread] || theDocument.IsNull()
        || myState == nullptr || (!isSubtract && !isSingleResult)
        || (isSubtract
            && (theActorShapes.empty() || theResultShapes.empty()
                || anItemCount > kMaxBooleanSourceOperands
                || theSuppressedSourceLabels.size() != anItemCount))
        || (isSingleResult
            && (!theActorShapes.empty() || theResultShapes.size() != 1
                || theSuppressedSourceLabels.size() < 2
                || theSuppressedSourceLabels.size()
                    > kMaxBooleanSourceOperands))) {
        return {};
    }

    try {
        OCC_CATCH_SIGNALS

        const Handle(TDocStd_Document)& aDocument = theDocument->Document();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()
            || aDocument.get() != myState->documentObject.get()
            || theDocument->DocumentIdentifier()
                != myState->documentIdentifier
            || myState->lastFullLabelToInstances == nullptr
            || myState->lastFullEntityIdentifiers == nullptr) {
            return {};
        }
        const Handle(TDF_Data)& aData = aDocument->GetData();
        if (aData.IsNull()
            || aData->Time() != myState->lastFullDocumentTime) {
            return {};
        }

        PresentationOverlayContent aContent;
        aContent.kind = theKind;
        aContent.meshes.reserve(anItemCount);
        aContent.instances.reserve(anItemCount);
        aContent.materials.reserve(anItemCount);
        aContent.suppressedEntityIdentifiers.reserve(
            theSuppressedSourceLabels.size());

        std::unordered_set<std::string> aSeenSourceLabels;
        std::unordered_set<std::string> aSeenSuppressedEntities;
        for (const TDF_Label& aLabel : theSuppressedSourceLabels) {
            const std::string aLabelIdentifier =
                theDocument->EntityIdentifierForLabel(aLabel);
            const auto aMapping =
                myState->lastFullLabelToInstances->find(aLabelIdentifier);
            // The Boolean controller operates on one displayed committed body
            // per source. Ambiguous assembly-definition mappings stay on OCCT.
            if (aLabel.IsNull() || aLabelIdentifier.empty()
                || !aSeenSourceLabels.insert(aLabelIdentifier).second
                || aMapping == myState->lastFullLabelToInstances->end()
                || aMapping->second.size() != 1) {
                return {};
            }
            const std::size_t anInstanceIndex = aMapping->second.front();
            if (anInstanceIndex
                    >= myState->lastFullEntityIdentifiers->size()) {
                return {};
            }
            const std::string& anEntityIdentifier =
                (*myState->lastFullEntityIdentifiers)[anInstanceIndex];
            if (!IsValidIdentifier(anEntityIdentifier)
                || !aSeenSuppressedEntities.insert(
                    anEntityIdentifier).second) {
                return {};
            }
            aContent.suppressedEntityIdentifiers.push_back(
                anEntityIdentifier);
        }

        std::size_t aVertexCount = 0;
        std::size_t anIndexCount = 0;
        std::size_t aPrimitiveCount = 0;
        std::size_t aBindingCount = 0;
        std::size_t aNumericByteCount = 0;
        std::unordered_set<const AIS_Shape*> aSeenPresentations;
        aSeenPresentations.reserve(anItemCount);

        const auto appendItem = [&](const Handle(AIS_Shape)& theShape,
                                    const std::string& theIdentifier,
                                    const std::string& theName,
                                    const RenderRole theRole,
                                    const RenderStyle theStyle,
                                    const Quantity_NameOfColor theColor) {
            if (theShape.IsNull()
                || !aSeenPresentations.insert(theShape.get()).second
                || aContent.meshes.size() >= kMaxOverlayMeshes
                || aVertexCount >= kMaxOverlayVertices
                || anIndexCount >= kMaxOverlayIndices
                || aPrimitiveCount >= kMaxOverlayPrimitives
                || aBindingCount >= kMaxOverlayPrimitiveBindings) {
                return false;
            }
            WorldPreviewItem anItem;
            if (!ExtractWorldPreviewItem(
                    theShape,
                    theIdentifier,
                    theName,
                    static_cast<std::uint32_t>(aContent.meshes.size()),
                    theRole,
                    theStyle,
                    theColor,
                    kMaxOverlayVertices - aVertexCount,
                    kMaxOverlayIndices - anIndexCount,
                    std::min(kMaxOverlayPrimitives - aPrimitiveCount,
                             kMaxOverlayPrimitiveBindings - aBindingCount),
                    anItem)) {
                return false;
            }

            std::size_t aVertexBytes = 0;
            std::size_t anIndexBytes = 0;
            if (!CheckedAdd(aVertexCount,
                            anItem.mesh.vertices.size(),
                            aVertexCount)
                || aVertexCount > kMaxOverlayVertices
                || !CheckedAdd(anIndexCount,
                               anItem.mesh.indices.size(),
                               anIndexCount)
                || anIndexCount > kMaxOverlayIndices
                || !CheckedAdd(aPrimitiveCount,
                               anItem.mesh.primitives.size(),
                               aPrimitiveCount)
                || aPrimitiveCount > kMaxOverlayPrimitives
                || !CheckedAdd(aBindingCount,
                               anItem.instance.primitiveBindings.size(),
                               aBindingCount)
                || aBindingCount > kMaxOverlayPrimitiveBindings
                || !CheckedMultiply(anItem.mesh.vertices.size(),
                                    sizeof(Vertex),
                                    aVertexBytes)
                || !CheckedMultiply(anItem.mesh.indices.size(),
                                    sizeof(std::uint32_t),
                                    anIndexBytes)
                || !CheckedAdd(aNumericByteCount,
                               aVertexBytes,
                               aNumericByteCount)
                || !CheckedAdd(aNumericByteCount,
                               anIndexBytes,
                               aNumericByteCount)
                || aNumericByteCount > kMaxOverlayNumericBytes) {
                return false;
            }
            aContent.meshes.push_back(std::move(anItem.mesh));
            aContent.instances.push_back(std::move(anItem.instance));
            aContent.materials.push_back(std::move(anItem.material));
            return true;
        };

        for (std::size_t anActorIndex = 0;
             anActorIndex < theActorShapes.size(); ++anActorIndex) {
            const std::string anIdentifier =
                "boolean/subtract/actor/" + std::to_string(anActorIndex);
            if (!appendItem(
                    theActorShapes[anActorIndex],
                    anIdentifier,
                    "Boolean subtract actor "
                        + std::to_string(anActorIndex),
                    RenderRole::BooleanActor,
                    RenderStyle::Wireframe,
                    Quantity_NOC_ORANGE)) {
                return {};
            }
        }
        for (std::size_t aResultIndex = 0;
             aResultIndex < theResultShapes.size(); ++aResultIndex) {
            const std::string anIdentifier = isUnion
                ? "boolean/union/result/0"
                : isIntersect
                    ? "boolean/intersect/result/0"
                    : "boolean/subtract/result/"
                        + std::to_string(aResultIndex);
            const std::string aName = isUnion
                ? "Boolean union result 0"
                : isIntersect
                    ? "Boolean intersect result 0"
                    : "Boolean subtract result "
                        + std::to_string(aResultIndex);
            if (!appendItem(
                    theResultShapes[aResultIndex],
                    anIdentifier,
                    aName,
                    RenderRole::BooleanSubject,
                    RenderStyle::Shaded,
                    Quantity_NOC_LIGHTSKYBLUE)) {
                return {};
            }
        }

        return PublishPresentationOverlayImpl(
            theDocument,
            std::move(aContent),
            false,
            true,
            false,
            false,
            false);
    } catch (const Standard_Failure&) {
        return {};
    } catch (...) {
        return {};
    }
}

OcctSceneSnapshotBuilder::OverlayPointer
OcctSceneSnapshotBuilder::PublishChamferPreviewOverlay(
    const Handle(OcctDocument)& theDocument,
    const std::vector<Handle(AIS_Shape)>& theResultShapes,
    const std::vector<TDF_Label>& theSuppressedSourceLabels) noexcept
{
    const std::size_t anItemCount = theResultShapes.size();
    if (![NSThread isMainThread] || theDocument.IsNull()
        || myState == nullptr || anItemCount == 0
        || anItemCount > kMaxChamferPreviewBodies
        || theSuppressedSourceLabels.size() != anItemCount) {
        return {};
    }

    try {
        OCC_CATCH_SIGNALS

        const Handle(TDocStd_Document)& aDocument = theDocument->Document();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()
            || aDocument.get() != myState->documentObject.get()
            || theDocument->DocumentIdentifier()
                != myState->documentIdentifier
            || myState->lastFullLabelToInstances == nullptr
            || myState->lastFullEntityIdentifiers == nullptr) {
            return {};
        }
        const Handle(TDF_Data)& aData = aDocument->GetData();
        if (aData.IsNull()
            || aData->Time() != myState->lastFullDocumentTime) {
            return {};
        }

        struct SourceResultPair {
            Handle(AIS_Shape) result;
            std::string labelIdentifier;
        };
        std::vector<SourceResultPair> aPairs;
        aPairs.reserve(anItemCount);
        std::unordered_set<std::string> aSeenSourceLabels;
        std::unordered_set<const AIS_Shape*> aSeenPresentations;
        std::unordered_set<const void*> aSeenResultTopologies;
        for (std::size_t anIndex = 0; anIndex < anItemCount; ++anIndex) {
            const Handle(AIS_Shape)& aResult = theResultShapes[anIndex];
            const TDF_Label& aLabel = theSuppressedSourceLabels[anIndex];
            if (aResult.IsNull() || aResult->Shape().IsNull()
                || aLabel.IsNull()
                || !theDocument->IsEditableFreeSimpleDefinitionLabel(aLabel)
                || !aSeenPresentations.insert(aResult.get()).second
                || !aSeenResultTopologies.insert(
                    aResult->Shape().TShape().get()).second) {
                return {};
            }
            const OcctGeometryRepresentation aRepresentation =
                theDocument->GeometryRepresentationForLabel(aLabel);
            if (aRepresentation != OcctGeometryRepresentation::LegacyUnknown
                && aRepresentation != OcctGeometryRepresentation::BRep) {
                return {};
            }
            // Presentation overlays intentionally carry no texture resource
            // table. Suppressing a textured committed occurrence and replacing
            // it with a scalar-only Bevel material would be a visible fidelity
            // regression, so keep OCCT authoritative for that preview.
            const Handle(XCAFDoc_VisMaterial) aSourceMaterial =
                XCAFDoc_VisMaterialTool::GetShapeMaterial(aLabel);
            if (!aSourceMaterial.IsNull()) {
                if (aSourceMaterial->HasPbrMaterial()) {
                    const XCAFDoc_VisMaterialPBR& aPbr =
                        aSourceMaterial->PbrMaterial();
                    if (!aPbr.BaseColorTexture.IsNull()
                        || !aPbr.EmissiveTexture.IsNull()
                        || !aPbr.MetallicRoughnessTexture.IsNull()
                        || !aPbr.OcclusionTexture.IsNull()
                        || !aPbr.NormalTexture.IsNull()) {
                        return {};
                    }
                }
                if (aSourceMaterial->HasCommonMaterial()
                    && !aSourceMaterial->CommonMaterial()
                            .DiffuseTexture.IsNull()) {
                    return {};
                }
            }
            std::string aLabelIdentifier =
                theDocument->EntityIdentifierForLabel(aLabel);
            if (!IsValidIdentifier(aLabelIdentifier)
                || !aSeenSourceLabels.insert(aLabelIdentifier).second) {
                return {};
            }
            aPairs.push_back({
                aResult,
                std::move(aLabelIdentifier),
            });
        }
        std::sort(
            aPairs.begin(),
            aPairs.end(),
            [](const SourceResultPair& theLeft,
               const SourceResultPair& theRight) {
                return theLeft.labelIdentifier < theRight.labelIdentifier;
            });

        PresentationOverlayContent aContent;
        aContent.kind = PresentationOverlayKind::ChamferPreview;
        aContent.meshes.reserve(anItemCount);
        aContent.instances.reserve(anItemCount);
        aContent.materials.reserve(anItemCount);
        aContent.suppressedEntityIdentifiers.reserve(anItemCount);

        std::unordered_set<std::string> aSeenSuppressedEntities;
        for (const SourceResultPair& aPair : aPairs) {
            const auto aMapping =
                myState->lastFullLabelToInstances->find(
                    aPair.labelIdentifier);
            // A transient result replaces exactly one committed occurrence.
            // Assembly definitions with multiple occurrences stay on OCCT.
            if (aMapping == myState->lastFullLabelToInstances->end()
                || aMapping->second.size() != 1) {
                return {};
            }
            const std::size_t anInstanceIndex = aMapping->second.front();
            if (anInstanceIndex
                    >= myState->lastFullEntityIdentifiers->size()) {
                return {};
            }
            const std::string& anEntityIdentifier =
                (*myState->lastFullEntityIdentifiers)[anInstanceIndex];
            if (!IsValidIdentifier(anEntityIdentifier)
                || !aSeenSuppressedEntities.insert(
                    anEntityIdentifier).second) {
                return {};
            }
            aContent.suppressedEntityIdentifiers.push_back(
                anEntityIdentifier);
        }

        std::size_t aVertexCount = 0;
        std::size_t anIndexCount = 0;
        std::size_t aPrimitiveCount = 0;
        std::size_t aBindingCount = 0;
        std::size_t aNumericByteCount = 0;
        for (std::size_t anIndex = 0; anIndex < aPairs.size(); ++anIndex) {
            if (aContent.meshes.size() >= kMaxOverlayMeshes
                || aVertexCount >= kMaxOverlayVertices
                || anIndexCount >= kMaxOverlayIndices
                || aPrimitiveCount >= kMaxOverlayPrimitives
                || aBindingCount >= kMaxOverlayPrimitiveBindings) {
                return {};
            }
            const std::string anIdentifier =
                "chamfer/preview/" + std::to_string(anIndex);
            WorldPreviewItem anItem;
            if (!ExtractWorldPreviewItem(
                    aPairs[anIndex].result,
                    anIdentifier,
                    "Chamfer preview " + std::to_string(anIndex),
                    static_cast<std::uint32_t>(anIndex),
                    RenderRole::ChamferPreview,
                    RenderStyle::Shaded,
                    std::nullopt,
                    kMaxOverlayVertices - aVertexCount,
                    kMaxOverlayIndices - anIndexCount,
                    std::min(kMaxOverlayPrimitives - aPrimitiveCount,
                             kMaxOverlayPrimitiveBindings - aBindingCount),
                    anItem)) {
                return {};
            }

            std::size_t aVertexBytes = 0;
            std::size_t anIndexBytes = 0;
            if (!CheckedAdd(aVertexCount,
                            anItem.mesh.vertices.size(),
                            aVertexCount)
                || aVertexCount > kMaxOverlayVertices
                || !CheckedAdd(anIndexCount,
                               anItem.mesh.indices.size(),
                               anIndexCount)
                || anIndexCount > kMaxOverlayIndices
                || !CheckedAdd(aPrimitiveCount,
                               anItem.mesh.primitives.size(),
                               aPrimitiveCount)
                || aPrimitiveCount > kMaxOverlayPrimitives
                || !CheckedAdd(aBindingCount,
                               anItem.instance.primitiveBindings.size(),
                               aBindingCount)
                || aBindingCount > kMaxOverlayPrimitiveBindings
                || !CheckedMultiply(anItem.mesh.vertices.size(),
                                    sizeof(Vertex),
                                    aVertexBytes)
                || !CheckedMultiply(anItem.mesh.indices.size(),
                                    sizeof(std::uint32_t),
                                    anIndexBytes)
                || !CheckedAdd(aNumericByteCount,
                               aVertexBytes,
                               aNumericByteCount)
                || !CheckedAdd(aNumericByteCount,
                               anIndexBytes,
                               aNumericByteCount)
                || aNumericByteCount > kMaxOverlayNumericBytes) {
                return {};
            }
            aContent.meshes.push_back(std::move(anItem.mesh));
            aContent.instances.push_back(std::move(anItem.instance));
            aContent.materials.push_back(std::move(anItem.material));
        }

        return PublishPresentationOverlayImpl(
            theDocument,
            std::move(aContent),
            false,
            false,
            true,
            false,
            false);
    } catch (const Standard_Failure&) {
        return {};
    } catch (...) {
        return {};
    }
}

OcctSceneSnapshotBuilder::OverlayPointer
OcctSceneSnapshotBuilder::PublishShellPreviewOverlay(
    const Handle(OcctDocument)& theDocument,
    const Handle(AIS_Shape)& theResultShape,
    const TDF_Label& theSuppressedSourceLabel) noexcept
{
    if (![NSThread isMainThread] || theDocument.IsNull()
        || myState == nullptr || theResultShape.IsNull()
        || theResultShape->Shape().IsNull()
        || theResultShape->Shape().ShapeType() != TopAbs_SOLID
        || theSuppressedSourceLabel.IsNull()) {
        return {};
    }

    try {
        OCC_CATCH_SIGNALS

        const Handle(TDocStd_Document)& aDocument =
            theDocument->Document();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()
            || aDocument.get() != myState->documentObject.get()
            || theDocument->DocumentIdentifier()
                != myState->documentIdentifier
            || myState->lastFullLabelToInstances == nullptr
            || myState->lastFullEntityIdentifiers == nullptr) {
            return {};
        }
        const Handle(TDF_Data)& aData = aDocument->GetData();
        if (aData.IsNull()
            || aData->Time() != myState->lastFullDocumentTime
            || theSuppressedSourceLabel.Data() != aData
            || !theDocument->IsEditableFreeSimpleDefinitionLabel(
                theSuppressedSourceLabel)
            || HasUnsupportedVisualMaterialTexture(
                theSuppressedSourceLabel)
            || HasStyledShellSubshape(
                aDocument, theSuppressedSourceLabel)
            || HasUnsupportedPresentationTexture(theResultShape)) {
            return {};
        }
        const OcctGeometryRepresentation aRepresentation =
            theDocument->GeometryRepresentationForLabel(
                theSuppressedSourceLabel);
        if (aRepresentation != OcctGeometryRepresentation::LegacyUnknown
            && aRepresentation != OcctGeometryRepresentation::BRep) {
            return {};
        }

        const std::string aLabelIdentifier =
            theDocument->EntityIdentifierForLabel(
                theSuppressedSourceLabel);
        if (!IsValidIdentifier(aLabelIdentifier)) {
            return {};
        }
        const auto aMapping =
            myState->lastFullLabelToInstances->find(aLabelIdentifier);
        // A Shell result has one source definition and can replace only one
        // committed occurrence. Shared assembly definitions remain on OCCT.
        if (aMapping == myState->lastFullLabelToInstances->end()
            || aMapping->second.size() != 1) {
            return {};
        }
        const std::size_t anInstanceIndex = aMapping->second.front();
        if (anInstanceIndex
                >= myState->lastFullEntityIdentifiers->size()) {
            return {};
        }
        const std::string& aSuppressedEntityIdentifier =
            (*myState->lastFullEntityIdentifiers)[anInstanceIndex];
        if (!IsValidIdentifier(aSuppressedEntityIdentifier)
            || aSuppressedEntityIdentifier == "shell/preview/0") {
            return {};
        }

        WorldPreviewItem anItem;
        if (!ExtractWorldPreviewItem(
                theResultShape,
                "shell/preview/0",
                "Shell preview 0",
                0,
                RenderRole::ShellPreview,
                RenderStyle::Shaded,
                std::nullopt,
                kMaxOverlayVertices,
                kMaxOverlayIndices,
                std::min(kMaxOverlayPrimitives,
                         kMaxOverlayPrimitiveBindings),
                anItem)
            || anItem.material.baseColorTextureIndex != -1
            || anItem.material.emissiveTextureIndex != -1) {
            return {};
        }

        PresentationOverlayContent aContent;
        aContent.kind = PresentationOverlayKind::ShellPreview;
        aContent.meshes.push_back(std::move(anItem.mesh));
        aContent.instances.push_back(std::move(anItem.instance));
        aContent.materials.push_back(std::move(anItem.material));
        aContent.suppressedEntityIdentifiers.push_back(
            aSuppressedEntityIdentifier);
        return PublishPresentationOverlayImpl(
            theDocument,
            std::move(aContent),
            false,
            false,
            false,
            false,
            true);
    } catch (const Standard_Failure&) {
        return {};
    } catch (...) {
        return {};
    }
}

OcctSceneSnapshotBuilder::OverlayPointer
OcctSceneSnapshotBuilder::PublishEmptyLinearArrayPreviewOverlay(
    const Handle(OcctDocument)& theDocument) noexcept
{
    PresentationOverlayContent aContent;
    aContent.kind = PresentationOverlayKind::LinearArrayPreview;
    return PublishPresentationOverlayImpl(
        theDocument,
        std::move(aContent),
        false,
        false,
        false,
        true,
        false);
}

OcctSceneSnapshotBuilder::OverlayPointer
OcctSceneSnapshotBuilder::PublishLinearArrayPreviewOverlay(
    const Handle(OcctDocument)& theDocument,
    const std::vector<Handle(AIS_Shape)>& thePreviewShapes) noexcept
{
    const std::size_t anInstanceCount = thePreviewShapes.size();
    if (![NSThread isMainThread] || theDocument.IsNull()
        || myState == nullptr || anInstanceCount == 0
        || anInstanceCount > kMaxLinearArrayPreviewBodies) {
        return {};
    }

    try {
        OCC_CATCH_SIGNALS

        const Handle(TDocStd_Document)& aDocument =
            theDocument->Document();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()
            || aDocument.get() != myState->documentObject.get()
            || theDocument->DocumentIdentifier()
                != myState->documentIdentifier) {
            return {};
        }
        const Handle(TDF_Data)& aData = aDocument->GetData();
        if (aData.IsNull()
            || aData->Time() != myState->lastFullDocumentTime) {
            return {};
        }

        const Handle(AIS_Shape)& aFirstPresentation =
            thePreviewShapes.front();
        if (aFirstPresentation.IsNull()
            || aFirstPresentation->Shape().IsNull()
            || HasUnsupportedPresentationTexture(aFirstPresentation)) {
            return {};
        }
        const TopoDS_Shape& aFirstShape =
            aFirstPresentation->Shape();
        const gp_Trsf aFirstTransform =
            aFirstPresentation->Transformation();
        if (!HaveCompatibleLinearArrayBasis(
                aFirstTransform, aFirstTransform)) {
            return {};
        }

        const std::size_t aMaximumPrimitiveCount = std::min(
            kMaxOverlayPrimitives,
            kMaxOverlayPrimitiveBindings / anInstanceCount);
        if (aMaximumPrimitiveCount == 0) {
            return {};
        }
        WorldPreviewItem aSourceItem;
        if (!ExtractWorldPreviewItem(
                aFirstPresentation,
                "linear-array/source/0",
                "Linear array source 0",
                0,
                RenderRole::LinearArrayPreview,
                RenderStyle::Shaded,
                std::nullopt,
                kMaxOverlayVertices,
                kMaxOverlayIndices,
                aMaximumPrimitiveCount,
                aSourceItem)
            || !IsTranslationOnlyWorldTransform(
                aSourceItem.instance.worldFromObject)
            || aSourceItem.material.baseColorTextureIndex != -1
            || aSourceItem.material.emissiveTextureIndex != -1) {
            return {};
        }

        PresentationOverlayContent aContent;
        aContent.kind = PresentationOverlayKind::LinearArrayPreview;
        aContent.meshes.reserve(1);
        aContent.materials.reserve(1);
        aContent.instances.reserve(anInstanceCount);
        std::unordered_set<const AIS_Shape*> aSeenPresentations;
        aSeenPresentations.reserve(anInstanceCount);
        for (std::size_t anIndex = 0;
             anIndex < anInstanceCount; ++anIndex) {
            const Handle(AIS_Shape)& aPresentation =
                thePreviewShapes[anIndex];
            if (aPresentation.IsNull()
                || !aSeenPresentations.insert(
                    aPresentation.get()).second
                || aPresentation->Shape().IsNull()
                || !aPresentation->Shape().IsPartner(aFirstShape)
                || !aPresentation->Shape().IsEqual(aFirstShape)
                || !HasCompatibleLinearArrayStyle(
                    aFirstPresentation, aPresentation)) {
                return {};
            }
            const gp_Trsf aTransform =
                aPresentation->Transformation();
            if (!HaveCompatibleLinearArrayBasis(
                    aFirstTransform, aTransform)) {
                return {};
            }

            InstanceSnapshot anInstance = aSourceItem.instance;
            const std::size_t anOrdinal = anIndex + 1U;
            anInstance.entityIdentifier =
                "linear-array/preview/0/"
                + std::to_string(anOrdinal);
            anInstance.name =
                "Linear array preview " + std::to_string(anOrdinal);
            anInstance.meshIndex = 0;
            anInstance.role = RenderRole::LinearArrayPreview;
            anInstance.coordinateSpace = CoordinateSpace::World;
            anInstance.depthPolicy = DepthPolicy::Scene;
            anInstance.renderStyle = RenderStyle::Shaded;

            // The controller's shallow partners share one exact linear basis.
            // Extraction bakes that basis into the centered source mesh, so
            // each AIS-local translation delta is the exact remaining instance
            // transform and no preview geometry is traversed again.
            for (Standard_Integer anAxis = 1; anAxis <= 3; ++anAxis) {
                const double aTranslation =
                    aSourceItem.instance.worldFromObject.values[
                        static_cast<std::size_t>(11 + anAxis)]
                    + aTransform.Value(anAxis, 4)
                    - aFirstTransform.Value(anAxis, 4);
                if (!IsFinite(aTranslation)) {
                    return {};
                }
                anInstance.worldFromObject.values[
                    static_cast<std::size_t>(11 + anAxis)] =
                        aTranslation;
            }
            if (!IsTranslationOnlyWorldTransform(
                    anInstance.worldFromObject)) {
                return {};
            }
            for (PrimitiveBinding& aBinding :
                 anInstance.primitiveBindings) {
                aBinding.materialIndex = 0;
                aBinding.pickToken = 0;
                aBinding.visible = true;
            }
            aContent.instances.push_back(std::move(anInstance));
        }
        aContent.meshes.push_back(std::move(aSourceItem.mesh));
        aContent.materials.push_back(std::move(aSourceItem.material));

        return PublishPresentationOverlayImpl(
            theDocument,
            std::move(aContent),
            false,
            false,
            false,
            true,
            false);
    } catch (const Standard_Failure&) {
        return {};
    } catch (...) {
        return {};
    }
}

OcctSceneSnapshotBuilder::SnapshotPointer OcctSceneSnapshotBuilder::Build(
    const Handle(OcctDocument)& theDocument,
    const Handle(AIS_InteractiveContext)& theContext,
    const Handle(V3d_View)& theView,
    const UInt2& theViewportPixels) noexcept
{
    if (![NSThread isMainThread]
        || theDocument.IsNull()
        || theContext.IsNull()
        || theView.IsNull()
        || myState == nullptr
        || myState->publicationSourceIdentifier.empty()
        || theViewportPixels.x == 0
        || theViewportPixels.y == 0) {
        return {};
    }

    try {
        OCC_CATCH_SIGNALS

        const Handle(TDocStd_Document)& aDocument = theDocument->Document();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()) {
            return {};
        }
        const std::string aDocumentIdentifier =
            theDocument->DocumentIdentifier();
        if (aDocumentIdentifier.empty()
            || !XCAFDoc_DocumentTool::CheckShapeTool(aDocument->Main())) {
            return {};
        }
        Standard_Real aMetersPerUnit = kLegacyMetersPerUnit;
        const bool hasDocumentLengthUnit =
            XCAFDoc_DocumentTool::GetLengthUnit(aDocument, aMetersPerUnit);
        if ((hasDocumentLengthUnit
                && (!IsFinite(aMetersPerUnit)
                    || aMetersPerUnit <= 0.0))
            || (!hasDocumentLengthUnit
                && aMetersPerUnit != kLegacyMetersPerUnit)) {
            return {};
        }

        std::vector<OccurrenceData> anOccurrences;
        std::unordered_set<std::string> anEntityIdentifiers;
        std::unordered_map<std::string, TDF_Label> aPersistentEntityLabels;
        std::unordered_map<std::string, std::size_t> aDefinitionIndices;
        std::vector<DefinitionData> aDefinitions;
        std::size_t aLabelInstanceMappingCount = 0;

        XCAFPrs_DocumentExplorer anExplorer(
            aDocument,
            XCAFPrs_DocumentExplorerFlags_OnlyLeafNodes,
            XCAFPrs_Style());
        for (; anExplorer.More(); anExplorer.Next()) {
            if (anOccurrences.size()
                >= core3d::limits::kMaximumLeafPresentations) {
                return {};
            }
            const XCAFPrs_DocumentNode& aNode = anExplorer.Current();
            const TDF_Label aDefinitionLabel = aNode.RefLabel.IsNull()
                ? aNode.Label
                : aNode.RefLabel;
            if (aNode.Label.IsNull() || aDefinitionLabel.IsNull()) {
                return {};
            }
            OccurrenceData anOccurrence;
            anOccurrence.occurrenceLabel = aNode.Label;
            anOccurrence.definitionLabel = aDefinitionLabel;
            anOccurrence.occurrenceLocation = aNode.Location;
            anOccurrence.style = aNode.Style;
            anOccurrence.visible = aNode.Style.IsVisible();
            const Standard_Integer aCurrentDepth = anExplorer.CurrentDepth();
            if (aCurrentDepth < 0
                || static_cast<std::size_t>(aCurrentDepth)
                    >= kMaxOccurrenceDepth) {
                return {};
            }
            anOccurrence.labelIdentifiers.reserve(
                static_cast<std::size_t>(aCurrentDepth) + 1U);
            for (Standard_Integer aDepth = 0;
                 aDepth <= aCurrentDepth; ++aDepth) {
                const TDF_Label& aPathLabel = anExplorer.Current(aDepth).Label;
                if (aPathLabel.IsNull()) {
                    return {};
                }
                std::string aLabelIdentifier =
                    theDocument->EntityIdentifierForLabel(aPathLabel);
                if (aLabelIdentifier.empty()) {
                    return {};
                }
                const auto [aKnownIdentifier, wasInserted] =
                    aPersistentEntityLabels.emplace(aLabelIdentifier,
                                                    aPathLabel);
                if (!wasInserted
                    && !aKnownIdentifier->second.IsEqual(aPathLabel)) {
                    return {};
                }
                anOccurrence.labelIdentifiers.push_back(
                    std::move(aLabelIdentifier));
            }
            if (!CheckedAdd(aLabelInstanceMappingCount,
                            anOccurrence.labelIdentifiers.size(),
                            aLabelInstanceMappingCount)
                || aLabelInstanceMappingCount > kMaxLabelInstanceMappings) {
                return {};
            }
            anOccurrence.entityIdentifier = DeriveOccurrenceIdentifier(
                aDocumentIdentifier,
                anOccurrence.labelIdentifiers);
            anOccurrence.definitionIdentifier =
                theDocument->DefinitionIdentifierForLabel(aDefinitionLabel);
            if (anOccurrence.entityIdentifier.empty()
                || anOccurrence.definitionIdentifier.empty()
                || !anEntityIdentifiers.insert(
                        anOccurrence.entityIdentifier).second) {
                return {};
            }
            anOccurrence.name = ReadName(aNode.Label, aDefinitionLabel);
            if (anOccurrence.name.empty()) {
                anOccurrence.name = anOccurrence.entityIdentifier;
            }

            const TopoDS_Shape aShape =
                XCAFDoc_ShapeTool::GetShape(aDefinitionLabel);
            if (aShape.IsNull()) {
                return {};
            }
            const auto aKnownDefinition = aDefinitionIndices.find(
                anOccurrence.definitionIdentifier);
            if (aKnownDefinition == aDefinitionIndices.end()) {
                DefinitionData aDefinition;
                aDefinition.label = aDefinitionLabel;
                aDefinition.shape = aShape;
                aDefinition.representation =
                    theDocument->GeometryRepresentationForLabel(
                        aDefinitionLabel);
                if (aDefinition.representation
                    == OcctGeometryRepresentation::Invalid) {
                    return {};
                }
                aDefinitionIndices.emplace(anOccurrence.definitionIdentifier,
                                           aDefinitions.size());
                aDefinitions.push_back(std::move(aDefinition));
            } else {
                const DefinitionData& aKnown =
                    aDefinitions[aKnownDefinition->second];
                if (!aKnown.label.IsEqual(aDefinitionLabel)
                    || !aKnown.shape.IsSame(aShape)) {
                    // A persistent definition UUID must identify exactly one
                    // product label, even when two labels share a TShape.
                    return {};
                }
            }
            anOccurrences.push_back(std::move(anOccurrence));
        }

        // Identity preflight is complete. Full snapshot publication is a
        // bounded read of triangulations already produced by OpenGL. It must
        // never invoke a mesher here: Build is main-thread-only because it
        // reads live OCAF/AIS/V3d state, and meshing can be unbounded.
        if (!aDefinitions.empty()) {
            for (auto& [aDefinitionIdentifier, aDefinitionIndex] : aDefinitionIndices) {
                if (!ExtractDefinitionGeometry(
                        aDefinitions[aDefinitionIndex].label,
                        aDefinitionIdentifier,
#ifdef DEBUG
                        static_cast<std::uint8_t>(
                            myState->debugTriangulationFailure),
#endif
                        aDefinitions[aDefinitionIndex])) {
                    return {};
                }
            }
        }

        State aNextState = *myState;
        if (aNextState.documentObject.get() != aDocument.get()
            || aNextState.documentIdentifier != aDocumentIdentifier) {
            if (!IncrementRevision(aNextState.documentGeneration)) {
                return {};
            }
            aNextState.documentObject = aDocument;
            aNextState.documentIdentifier = aDocumentIdentifier;
            aNextState.modelFingerprint.reset();
            aNextState.presentationFingerprint.reset();
            aNextState.cameraFingerprint.reset();
            aNextState.overlay.fingerprint.reset();
            aNextState.definitions.clear();
            aNextState.overlay.definitions.clear();
            aNextState.lastFullLabelToInstances.reset();
            aNextState.lastFullEntityIdentifiers.reset();
            aNextState.lastFullSnapshotRevision = 0;
            aNextState.lastFullDocumentTime = 0;
        }

        SceneSnapshot aScene;
        aScene.publicationSourceIdentifier =
            aNextState.publicationSourceIdentifier;
        aScene.metersPerUnit = aMetersPerUnit;
        TextureTableState aTextureTable;
        aScene.meshes.reserve(aDefinitions.size());
        std::vector<std::string> aLiveRevisionKeys;
        aLiveRevisionKeys.reserve(aDefinitions.size());
        std::unordered_set<std::string> aLiveRevisionKeySet;
        aLiveRevisionKeySet.reserve(aDefinitions.size());
        std::size_t aNewRevisionCount = 0;
        for (const DefinitionData& aDefinition : aDefinitions) {
            std::string aRevisionKey = aDocumentIdentifier + "\n"
                + aDefinition.mesh.definitionIdentifier;
            if (!aLiveRevisionKeySet.insert(aRevisionKey).second) {
                return {};
            }
            if (aNextState.definitions.find(aRevisionKey)
                == aNextState.definitions.end()) {
                ++aNewRevisionCount;
            }
            aLiveRevisionKeys.push_back(std::move(aRevisionKey));
        }

        std::size_t aRetainedRevisionCount = 0;
        if (!CheckedAdd(aNextState.definitions.size(),
                        aNewRevisionCount,
                        aRetainedRevisionCount)) {
            return {};
        }
        if (aRetainedRevisionCount > kMaxRetainedDefinitionRevisions) {
            const std::size_t anEvictionCount = aRetainedRevisionCount
                - kMaxRetainedDefinitionRevisions;
            std::vector<std::pair<std::uint64_t, std::string>> aTombstones;
            aTombstones.reserve(aNextState.definitions.size());
            for (const auto& [aRevisionKey, aRevision] :
                 aNextState.definitions) {
                if (aLiveRevisionKeySet.find(aRevisionKey)
                    == aLiveRevisionKeySet.end()) {
                    aTombstones.emplace_back(aRevision.lastSeenSnapshot,
                                             aRevisionKey);
                }
            }
            if (aTombstones.size() < anEvictionCount) {
                return {};
            }
            std::sort(aTombstones.begin(),
                      aTombstones.end(),
                      [](const auto& theLeft, const auto& theRight) {
                          return theLeft.first != theRight.first
                              ? theLeft.first < theRight.first
                              : theLeft.second < theRight.second;
                      });
            for (std::size_t anIndex = 0;
                 anIndex < anEvictionCount; ++anIndex) {
                aNextState.definitions.erase(aTombstones[anIndex].second);
            }
        }

        std::size_t aSnapshotNumericBytes = 0;
        std::size_t aSnapshotVertexCount = 0;
        std::size_t aSnapshotIndexCount = 0;
        for (std::size_t aDefinitionIndex = 0;
             aDefinitionIndex < aDefinitions.size(); ++aDefinitionIndex) {
            DefinitionData& aDefinition = aDefinitions[aDefinitionIndex];
            const std::string& aRevisionKey =
                aLiveRevisionKeys[aDefinitionIndex];
            auto aRevisionFound = aNextState.definitions.find(aRevisionKey);
            if (aRevisionFound == aNextState.definitions.end()) {
                if (aNextState.definitions.size()
                    >= kMaxRetainedDefinitionRevisions) {
                    return {};
                }
                aRevisionFound = aNextState.definitions.emplace(
                    aRevisionKey,
                    State::DefinitionRevision()).first;
            }
            State::DefinitionRevision& aRevision = aRevisionFound->second;
            aRevision.lastSeenSnapshot = aNextState.snapshotRevision;
            if (!aRevision.hasFingerprint
                || aRevision.fingerprint != aDefinition.fingerprint) {
                if (!IncrementRevision(aRevision.revision)) {
                    return {};
                }
                aRevision.fingerprint = aDefinition.fingerprint;
                aRevision.hasFingerprint = true;
            }
            aDefinition.mesh.geometryRevision = aRevision.revision;
            std::size_t aVertexBytes = 0;
            std::size_t anIndexBytes = 0;
            std::size_t aPrimitiveBytes = 0;
            std::size_t aMeshBytes = 0;
            if (!CheckedMultiply(aDefinition.mesh.vertices.size(),
                                 sizeof(Vertex),
                                 aVertexBytes)
                || !CheckedMultiply(aDefinition.mesh.indices.size(),
                                    sizeof(std::uint32_t),
                                    anIndexBytes)
                || !CheckedMultiply(aDefinition.mesh.primitives.size(),
                                    sizeof(MeshPrimitive),
                                    aPrimitiveBytes)
                || !CheckedAdd(aVertexBytes, anIndexBytes, aMeshBytes)
                || !CheckedAdd(aMeshBytes, aPrimitiveBytes, aMeshBytes)
                || !CheckedAdd(aSnapshotNumericBytes,
                               aMeshBytes,
                               aSnapshotNumericBytes)
                || !CheckedAdd(aSnapshotVertexCount,
                               aDefinition.mesh.vertices.size(),
                               aSnapshotVertexCount)
                || !CheckedAdd(aSnapshotIndexCount,
                               aDefinition.mesh.indices.size(),
                               aSnapshotIndexCount)
                || aSnapshotNumericBytes > kMaxSnapshotNumericBytes
                || aSnapshotVertexCount > kMaxVerticesPerSnapshot
                || aSnapshotIndexCount > kMaxIndicesPerSnapshot) {
                return {};
            }
            aScene.meshes.push_back(std::move(aDefinition.mesh));
        }

        std::unordered_map<std::string, std::vector<std::size_t>>
            aLabelToInstances;
        std::unordered_map<std::string, std::vector<std::size_t>>
            aDefinitionToInstances;
        std::size_t aBindingCount = 0;
        aScene.instances.reserve(anOccurrences.size());
        for (const OccurrenceData& anOccurrence : anOccurrences) {
            const auto aDefinitionFound = aDefinitionIndices.find(
                anOccurrence.definitionIdentifier);
            if (aDefinitionFound == aDefinitionIndices.end()
                || !FitsUInt32(aDefinitionFound->second)) {
                return {};
            }
            const std::size_t aDefinitionIndex = aDefinitionFound->second;
            const DefinitionData& aDefinition = aDefinitions[aDefinitionIndex];
            const MeshSnapshot& aMesh = aScene.meshes[aDefinitionIndex];
            if (!CheckedAdd(aBindingCount,
                            aMesh.primitives.size(),
                            aBindingCount)
                || aBindingCount > kMaxPrimitiveBindingsPerSnapshot) {
                return {};
            }

            gp_Trsf aWorldTransform =
                theDocument->ObjectTransformForLabel(anOccurrence.definitionLabel)
                    .Multiplied(anOccurrence.occurrenceLocation.Transformation());
            gp_Trsf aMeshOrigin;
            aMeshOrigin.SetTranslation(gp_Vec(aDefinition.sourceOrigin.x,
                                               aDefinition.sourceOrigin.y,
                                               aDefinition.sourceOrigin.z));
            aWorldTransform.Multiply(aMeshOrigin);

            InstanceSnapshot anInstance;
            anInstance.entityIdentifier = anOccurrence.entityIdentifier;
            anInstance.meshIndex = static_cast<std::uint32_t>(aDefinitionIndex);
            if (!MatrixFromTransform(aWorldTransform, anInstance.worldFromObject)) {
                return {};
            }
            anInstance.reversesWinding = aWorldTransform.IsNegative();
            anInstance.visible = anOccurrence.visible;
            anInstance.selectable = anOccurrence.visible;
            anInstance.name = anOccurrence.name;
            anInstance.role = RenderRole::Model;

            Graphic3d_NameOfMaterial aMaterialName;
            Quantity_NameOfColor aColorName;
            const std::optional<Graphic3d_NameOfMaterial> aMaterialOverride =
                theDocument->TryMaterialNameForLabel(
                    anOccurrence.definitionLabel, aMaterialName)
                ? std::optional<Graphic3d_NameOfMaterial>(aMaterialName)
                : std::nullopt;
            const std::optional<Quantity_NameOfColor> aColorOverride =
                theDocument->TryColorNameForLabel(
                    anOccurrence.definitionLabel, aColorName)
                ? std::optional<Quantity_NameOfColor>(aColorName)
                : std::nullopt;
            XCAFDoc_VisMaterialPBR aPbrMaterial;
            std::optional<WholeObjectPBRMaterial> aPbrOverride;
            if (theDocument->TryPBRMaterialForLabel(
                    anOccurrence.definitionLabel, aPbrMaterial)) {
                const Handle(XCAFDoc_VisMaterial) aVisualMaterial =
                    XCAFDoc_VisMaterialTool::GetShapeMaterial(
                        anOccurrence.definitionLabel);
                if (aVisualMaterial.IsNull()
                    || !aVisualMaterial->HasPbrMaterial()) {
                    return {};
                }
                aPbrOverride = WholeObjectPBRMaterial{
                    aPbrMaterial,
                    aVisualMaterial->AlphaMode(),
                    aVisualMaterial->AlphaCutOff(),
                    aVisualMaterial->FaceCulling(),
                };
            }

            std::vector<std::optional<MaterialSnapshot>> aFaceMaterials(
                aMesh.primitives.size());
            std::vector<bool> aFaceVisibility(aMesh.primitives.size(), true);
            std::unordered_map<std::uint32_t, std::size_t> aPrimitiveByFace;
            for (std::size_t aPrimitiveIndex = 0;
                 aPrimitiveIndex < aMesh.primitives.size(); ++aPrimitiveIndex) {
                aPrimitiveByFace.emplace(aMesh.primitives[aPrimitiveIndex].faceIndex,
                                         aPrimitiveIndex);
            }
            RWMesh_FaceIterator aFace(anOccurrence.definitionLabel,
                                      TopLoc_Location(),
                                      Standard_True,
                                      anOccurrence.style);
            for (; aFace.More(); aFace.Next()) {
                const Standard_Integer aFaceMapIndex =
                    aDefinition.faces.FindIndex(aFace.Face());
                if (aFaceMapIndex <= 0) {
                    return {};
                }
                const auto aPrimitiveFound = aPrimitiveByFace.find(
                    static_cast<std::uint32_t>(aFaceMapIndex - 1));
                if (aPrimitiveFound == aPrimitiveByFace.end()
                    || aFaceMaterials[aPrimitiveFound->second].has_value()) {
                    return {};
                }
                aFaceVisibility[aPrimitiveFound->second] =
                    aFace.FaceStyle().IsVisible();
                MaterialSnapshot aMaterial;
                Handle(Image_Texture) aBaseColorTexture;
                Handle(Image_Texture) anEmissiveTexture;
                if (!ResolveMaterial(aFace,
                                     aMaterialOverride,
                                     aColorOverride,
                                     aPbrOverride,
                                     aDefinition.closed,
                                     aMaterial,
                                     aBaseColorTexture,
                                     anEmissiveTexture)
                    || !AddTextureResource(
                        aScene,
                        aTextureTable,
                        aBaseColorTexture,
                        aMaterial.baseColorTextureIndex)
                    || !AddTextureResource(
                        aScene,
                        aTextureTable,
                        anEmissiveTexture,
                        aMaterial.emissiveTextureIndex)) {
                    return {};
                }
                aFaceMaterials[aPrimitiveFound->second] = std::move(aMaterial);
            }

            anInstance.primitiveBindings.reserve(aMesh.primitives.size());
            std::optional<std::uint32_t> anObjectPickToken;
            for (std::size_t aPrimitiveIndex = 0;
                 aPrimitiveIndex < aMesh.primitives.size(); ++aPrimitiveIndex) {
                if (!aFaceMaterials[aPrimitiveIndex].has_value()) {
                    return {};
                }
                std::uint32_t aMaterialIndex = 0;
                if (!AddMaterial(aScene,
                                 std::move(*aFaceMaterials[aPrimitiveIndex]),
                                 aMaterialIndex)) {
                    return {};
                }
                PrimitiveBinding aBinding;
                aBinding.materialIndex = aMaterialIndex;
                aBinding.visible = aFaceVisibility[aPrimitiveIndex];
                if (anInstance.selectable && aBinding.visible) {
                    const bool isTriangleMesh =
                        aDefinition.representation
                        == OcctGeometryRepresentation::TriangleMesh;
                    if (!isTriangleMesh || !anObjectPickToken.has_value()) {
                        if (!FitsUInt32(aScene.pickTable.size())
                            || aScene.pickTable.size()
                                == std::numeric_limits<std::uint32_t>::max()
                            || aScene.pickTable.size()
                                >= kMaxPickElementsPerSnapshot) {
                            return {};
                        }
                        const std::uint32_t aPickToken =
                            static_cast<std::uint32_t>(
                                aScene.pickTable.size());
                        aScene.pickTable.push_back({
                            anInstance.entityIdentifier,
                            isTriangleMesh
                                ? ElementKind::Object
                                : ElementKind::Face,
                            isTriangleMesh
                                ? 0U
                                : aMesh.primitives[aPrimitiveIndex].faceIndex,
                            aMesh.geometryRevision,
                        });
                        if (isTriangleMesh) {
                            anObjectPickToken = aPickToken;
                        }
                    }
                    aBinding.pickToken = anObjectPickToken.has_value()
                        ? *anObjectPickToken
                        : static_cast<std::uint32_t>(
                            aScene.pickTable.size() - 1U);
                }
                anInstance.primitiveBindings.push_back(aBinding);
            }
            const std::size_t anInstanceIndex = aScene.instances.size();
            std::unordered_set<std::string> aMappedLabelIdentifiers;
            for (const std::string& aLabelIdentifier :
                 anOccurrence.labelIdentifiers) {
                if (aMappedLabelIdentifiers.insert(aLabelIdentifier).second) {
                    aLabelToInstances[aLabelIdentifier].push_back(
                        anInstanceIndex);
                }
            }
            aDefinitionToInstances[anOccurrence.definitionIdentifier]
                .push_back(anInstanceIndex);
            aScene.instances.push_back(std::move(anInstance));
        }

        std::size_t anInstanceBytes = 0;
        std::size_t aBindingBytes = 0;
        std::size_t aMaterialBytes = 0;
        std::size_t aTextureMetadataBytes = 0;
        std::size_t aPickBytes = 0;
        std::size_t anAuxiliaryBytes = 0;
        if (!CheckedMultiply(aScene.instances.size(),
                             sizeof(Matrix4d),
                             anInstanceBytes)
            || !CheckedMultiply(aBindingCount,
                                sizeof(PrimitiveBinding),
                                aBindingBytes)
            || !CheckedMultiply(aScene.materials.size(),
                                sizeof(MaterialSnapshot),
                                aMaterialBytes)
            || !CheckedMultiply(aScene.textures.size(),
                                sizeof(TextureResourceSnapshot),
                                aTextureMetadataBytes)
            || !CheckedMultiply(aScene.pickTable.size(),
                                sizeof(ElementIdentifier),
                                aPickBytes)
            || !CheckedAdd(anInstanceBytes, aBindingBytes, anAuxiliaryBytes)
            || !CheckedAdd(anAuxiliaryBytes, aMaterialBytes, anAuxiliaryBytes)
            || !CheckedAdd(anAuxiliaryBytes,
                           aTextureMetadataBytes,
                           anAuxiliaryBytes)
            || !CheckedAdd(anAuxiliaryBytes, aPickBytes, anAuxiliaryBytes)
            || !CheckedAdd(aSnapshotNumericBytes,
                           anAuxiliaryBytes,
                           aSnapshotNumericBytes)
            || aSnapshotNumericBytes > kMaxSnapshotNumericBytes) {
            return {};
        }

        // Selection is intentionally copied after committed instances exist.
        // Unknown/transient AIS objects are ignored rather than leaked into the
        // committed document snapshot.
        std::unordered_set<std::string> aSelectedKeys;
        const auto elementForInstance = [&](const std::size_t theInstanceIndex,
                                            const TopoDS_Shape& theSubshape) {
            const InstanceSnapshot& anInstance =
                aScene.instances[theInstanceIndex];
            const MeshSnapshot& aMesh = aScene.meshes[anInstance.meshIndex];
            ElementIdentifier anElement;
            anElement.entityIdentifier = anInstance.entityIdentifier;
            anElement.kind = ElementKind::Object;
            anElement.geometryRevision = aMesh.geometryRevision;
            if (aDefinitions[anInstance.meshIndex].representation
                    != OcctGeometryRepresentation::TriangleMesh
                && !theSubshape.IsNull()
                && theSubshape.ShapeType() == TopAbs_FACE) {
                const Standard_Integer aFaceIndex =
                    aDefinitions[anInstance.meshIndex].faces.FindIndex(
                        theSubshape);
                if (aFaceIndex > 0) {
                    anElement.kind = ElementKind::Face;
                    anElement.topologyIndex =
                        static_cast<std::uint32_t>(aFaceIndex - 1);
                }
            }
            return anElement;
        };
        theContext->InitSelected();
        for (; theContext->MoreSelected(); theContext->NextSelected()) {
            const Handle(AIS_InteractiveObject) anInteractive =
                theContext->SelectedInteractive();
            if (anInteractive.IsNull()) {
                continue;
            }
            const TDF_Label aLabel = theDocument->ShapeLabel(anInteractive);
            const std::string aLabelIdentifier =
                theDocument->EntityIdentifierForLabel(aLabel);
            const auto anInstancesFound =
                aLabelToInstances.find(aLabelIdentifier);
            const std::vector<std::size_t>* anInstanceIndices =
                anInstancesFound == aLabelToInstances.end()
                ? nullptr
                : &anInstancesFound->second;
            if (anInstanceIndices == nullptr) {
                const std::string aDefinitionIdentifier =
                    theDocument->DefinitionIdentifierForLabel(aLabel);
                const auto aDefinitionsFound =
                    aDefinitionToInstances.find(aDefinitionIdentifier);
                if (!aDefinitionIdentifier.empty()
                    && aDefinitionsFound != aDefinitionToInstances.end()) {
                    anInstanceIndices = &aDefinitionsFound->second;
                }
            }
            if (anInstanceIndices == nullptr) {
                continue;
            }
            TopoDS_Shape aSelectedSubshape;
            const Handle(StdSelect_BRepOwner) aSelectedOwner =
                Handle(StdSelect_BRepOwner)::DownCast(
                    theContext->SelectedOwner());
            if (!aSelectedOwner.IsNull() && aSelectedOwner->HasShape()) {
                // Keep topology identity in definition-local space. OCCT's
                // SelectedShape() applies the interactive transformation as a
                // TopLoc_Location, which deliberately rejects uniform scale.
                // The immutable instance matrix already carries that transform.
                aSelectedSubshape = aSelectedOwner->Shape();
            }
            for (const std::size_t anInstanceIndex :
                 *anInstanceIndices) {
                if (aScene.selection.selected.size()
                    >= kMaxSelectedElements) {
                    return {};
                }
                InstanceSnapshot& anInstance =
                    aScene.instances[anInstanceIndex];
                const ElementIdentifier anElement = elementForInstance(
                    anInstanceIndex,
                    aSelectedSubshape);
                const std::string aSelectionKey =
                    anElement.entityIdentifier + ":"
                    + std::to_string(
                        static_cast<unsigned int>(anElement.kind)) + ":"
                    + std::to_string(anElement.topologyIndex);
                if (aSelectedKeys.insert(aSelectionKey).second) {
                    aScene.selection.selected.push_back(anElement);
                }
                anInstance.selected = true;
            }
        }
        std::sort(aScene.selection.selected.begin(),
                  aScene.selection.selected.end(),
                  [](const ElementIdentifier& theLeft,
                     const ElementIdentifier& theRight) {
                      if (theLeft.entityIdentifier
                          != theRight.entityIdentifier) {
                          return theLeft.entityIdentifier
                              < theRight.entityIdentifier;
                      }
                      if (theLeft.kind != theRight.kind) {
                          return static_cast<std::uint8_t>(theLeft.kind)
                              < static_cast<std::uint8_t>(theRight.kind);
                      }
                      if (theLeft.topologyIndex
                          != theRight.topologyIndex) {
                          return theLeft.topologyIndex
                              < theRight.topologyIndex;
                      }
                      return theLeft.geometryRevision
                          < theRight.geometryRevision;
                  });

        if (theContext->HasDetected()) {
            const Handle(AIS_InteractiveObject) anInteractive =
                theContext->DetectedInteractive();
            if (!anInteractive.IsNull()) {
                const TDF_Label aLabel = theDocument->ShapeLabel(anInteractive);
                const std::string aLabelIdentifier =
                    theDocument->EntityIdentifierForLabel(aLabel);
                const auto anInstancesFound =
                    aLabelToInstances.find(aLabelIdentifier);
                const std::vector<std::size_t>* anInstanceIndices =
                    anInstancesFound == aLabelToInstances.end()
                    ? nullptr
                    : &anInstancesFound->second;
                if (anInstanceIndices == nullptr) {
                    const std::string aDefinitionIdentifier =
                        theDocument->DefinitionIdentifierForLabel(aLabel);
                    const auto aDefinitionsFound =
                        aDefinitionToInstances.find(aDefinitionIdentifier);
                    if (!aDefinitionIdentifier.empty()
                        && aDefinitionsFound != aDefinitionToInstances.end()) {
                        anInstanceIndices = &aDefinitionsFound->second;
                    }
                }
                if (anInstanceIndices != nullptr) {
                    TopoDS_Shape aDetectedSubshape;
                    const Handle(StdSelect_BRepOwner) anOwner =
                        Handle(StdSelect_BRepOwner)::DownCast(
                            theContext->DetectedOwner());
                    if (!anOwner.IsNull() && anOwner->HasShape()) {
                        aDetectedSubshape = anOwner->Shape();
                    }

                    std::optional<std::size_t> aDetectedInstance;
                    if (anInstanceIndices->size() == 1
                        && aDefinitions[
                            aScene.instances[anInstanceIndices->front()]
                                .meshIndex].representation
                            == OcctGeometryRepresentation::TriangleMesh) {
                        aDetectedInstance = anInstanceIndices->front();
                    } else if (!aDetectedSubshape.IsNull()
                        && aDetectedSubshape.ShapeType() == TopAbs_FACE) {
                        for (const std::size_t anInstanceIndex :
                             *anInstanceIndices) {
                            const InstanceSnapshot& anInstance =
                                aScene.instances[anInstanceIndex];
                            if (aDefinitions[anInstance.meshIndex]
                                    .representation
                                == OcctGeometryRepresentation::TriangleMesh) {
                                continue;
                            }
                            if (aDefinitions[anInstance.meshIndex]
                                    .faces.FindIndex(aDetectedSubshape) > 0) {
                                if (aDetectedInstance.has_value()) {
                                    aDetectedInstance.reset();
                                    break;
                                }
                                aDetectedInstance = anInstanceIndex;
                            }
                        }
                    } else if (anInstanceIndices->size() == 1) {
                        aDetectedInstance = anInstanceIndices->front();
                    }
                    if (aDetectedInstance.has_value()) {
                        aScene.selection.hovered = elementForInstance(
                            *aDetectedInstance,
                            aDetectedSubshape);
                    }
                }
            }
        }

        std::size_t aSelectionBytes = 0;
        for (const ElementIdentifier& anElement :
             aScene.selection.selected) {
            if (!CheckedAdd(aSelectionBytes,
                            sizeof(ElementIdentifier),
                            aSelectionBytes)
                || !CheckedAdd(aSelectionBytes,
                               anElement.entityIdentifier.size(),
                               aSelectionBytes)) {
                return {};
            }
        }
        if (aScene.selection.hovered.has_value()
            && (!CheckedAdd(aSelectionBytes,
                            sizeof(ElementIdentifier),
                            aSelectionBytes)
                || !CheckedAdd(
                    aSelectionBytes,
                    aScene.selection.hovered->entityIdentifier.size(),
                    aSelectionBytes))) {
            return {};
        }
        if (!CheckedAdd(aSnapshotNumericBytes,
                        aSelectionBytes,
                        aSnapshotNumericBytes)
            || aSnapshotNumericBytes > kMaxSnapshotNumericBytes) {
            return {};
        }

        if (!BuildCamera(theView, theViewportPixels, aScene.camera)) {
            return {};
        }

        Bounds3d aWorldBounds;
        for (const InstanceSnapshot& anInstance : aScene.instances) {
            const Bounds3d& aBounds = aScene.meshes[anInstance.meshIndex].localBounds;
            for (const double anX : {aBounds.minimum.x, aBounds.maximum.x}) {
                for (const double aY : {aBounds.minimum.y, aBounds.maximum.y}) {
                    for (const double aZ : {aBounds.minimum.z, aBounds.maximum.z}) {
                        Double3 aWorldPoint;
                        if (!TransformPoint(anInstance.worldFromObject,
                                            {anX, aY, aZ},
                                            aWorldPoint)) {
                            return {};
                        }
                        Extend(aWorldBounds,
                               aWorldPoint.x,
                               aWorldPoint.y,
                               aWorldPoint.z);
                    }
                }
            }
        }
        aScene.renderOrigin = IsValid(aWorldBounds)
            ? Center(aWorldBounds)
            : aScene.camera.center;

        const std::uint64_t aModelFingerprint = ModelFingerprint(aScene);
        const std::uint64_t aPresentationFingerprint =
            PresentationFingerprint(aScene);
        const std::uint64_t aCameraFingerprint = CameraFingerprint(aScene.camera);
        if (!aNextState.modelFingerprint.has_value()
            || *aNextState.modelFingerprint != aModelFingerprint) {
            if (!IncrementRevision(aNextState.modelRevision)) {
                return {};
            }
            aNextState.modelFingerprint = aModelFingerprint;
        }
        if (!aNextState.presentationFingerprint.has_value()
            || *aNextState.presentationFingerprint != aPresentationFingerprint) {
            if (!IncrementRevision(aNextState.presentationRevision)) {
                return {};
            }
            aNextState.presentationFingerprint = aPresentationFingerprint;
        }
        if (!aNextState.cameraFingerprint.has_value()
            || *aNextState.cameraFingerprint != aCameraFingerprint) {
            if (!IncrementRevision(aNextState.cameraRevision)) {
                return {};
            }
            aNextState.cameraFingerprint = aCameraFingerprint;
        }
        if (!IncrementRevision(aNextState.snapshotRevision)) {
            return {};
        }

        aScene.revisions.snapshot = aNextState.snapshotRevision;
        aScene.revisions.documentGeneration = aNextState.documentGeneration;
        aScene.revisions.model = aNextState.modelRevision;
        aScene.revisions.presentation = aNextState.presentationRevision;
        aScene.revisions.camera = aNextState.cameraRevision;
        const Handle(TDF_Data)& aData = aDocument->GetData();
        if (aData.IsNull()) {
            return {};
        }
        aNextState.lastFullSnapshotRevision =
            aNextState.snapshotRevision;
        aNextState.lastFullDocumentTime = aData->Time();
        auto aLastFullEntityIdentifiers =
            std::make_shared<std::vector<std::string>>();
        aLastFullEntityIdentifiers->reserve(aScene.instances.size());
        for (const InstanceSnapshot& anInstance : aScene.instances) {
            aLastFullEntityIdentifiers->push_back(
                anInstance.entityIdentifier);
        }
        aNextState.lastFullEntityIdentifiers =
            std::move(aLastFullEntityIdentifiers);
        aNextState.lastFullLabelToInstances =
            std::make_shared<State::LabelInstanceMap>(
                std::move(aLabelToInstances));

        // Finish every potentially allocating operation before publishing
        // state. A failed allocation must not consume any revision.
        auto aCommittedState = std::make_unique<State>(std::move(aNextState));
        SnapshotPointer aSnapshot =
            std::make_shared<const SceneSnapshot>(std::move(aScene));
        myState.swap(aCommittedState);
        return aSnapshot;
    } catch (const Standard_Failure&) {
        return {};
    } catch (...) {
        return {};
    }
}

} // namespace core3d::scene
