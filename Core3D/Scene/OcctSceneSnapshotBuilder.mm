#if DEBUG
#include <cstdio>
#endif

#include "MikkTangentSpace.hpp"
//
//  OcctSceneSnapshotBuilder.mm
//  Core3D
//

#import <Foundation/Foundation.h>
#import <ImageIO/ImageIO.h>

#include "OcctSceneSnapshotBuilder.hpp"
#include "../OCCTKit/NativeAuthoredFrameGeometry.hxx"
#include "../OCCTKit/BoundedCurveEvaluation.hxx"
#include "../OCCTKit/NativeContactMeshCapture.hxx"
#include "../OCCTKit/PatternAllLabelAuthority.hxx"
#include "../OCCTKit/NativeOpeningDependentReplay.hxx"
#include "../OCCTKit/DecalLayerPersistence.hxx"
#include "../OCCTKit/DecalLayerBake.hxx"
#include "../OCCTKit/FaceImageResourceValidation.hxx"
#include "../OCCTKit/PaintedAtlasBake.hxx"

#include "../Common/Core3DMobileResourceLimits.h"
#include "../OCCTKit/OcctDocument.h"

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
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
#include <SelectMgr_EntityOwner.hxx>
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
#include <gp_Ax1.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <CommonCrypto/CommonDigest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <limits>
#include <new>
#include <numeric>
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
#ifdef DEBUG
thread_local bool gDebugBoundedCurveObservationArmed = false;
thread_local std::optional<DebugBoundedCurvePublicationObservation>
    gDebugBoundedCurveObservation;
thread_local bool gDebugOrdinaryLedgerObservationArmed = false;
thread_local void* gDebugOrdinaryLedgerContext = nullptr;
thread_local std::size_t gDebugOrdinaryLedgerCalls = 0;
thread_local bool gDebugOrdinaryLedgerMismatch = false;
thread_local std::size_t gDebugResolveReadResourceEntries = 0;
#endif

constexpr std::uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr std::size_t kMaxFacesPerMesh = 250'000;
constexpr std::size_t kMaxEdgesPerMesh = 250'000;
constexpr std::size_t kMaxTopologicalVerticesPerMesh = 250'000;
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
// Definition topology maps coexist until the immutable scene is complete.
// Preserve the existing 250k single-definition maxima while bounding their
// aggregate peak to one maximum-size face, edge, and vertex map. The separate
// face ceiling matches the later primitive-binding admission limit.
constexpr std::size_t kMaxTopologyMapEntriesPerSnapshot = 750'000;
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
constexpr std::size_t kMaxCurveStrokeSegments = 4'096;
constexpr std::size_t kMaxCurveStrokeEvaluations = 8'193;
constexpr unsigned kMaxCurveStrokeDepth = 12;
constexpr double kMaxCurveStrokeAngleRadians = 5.0 * kPi / 180.0;
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
constexpr std::size_t kMaxRadialArrayPreviewBodies = 15;
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

//! Proper-rigid affine transform admitted by the Radial Array overlay. The
//! determinant check rejects reflections; the translation ceiling keeps every
//! renderer on the same mobile document-coordinate budget.
bool IsProperRigidBoundedWorldTransform(const Matrix4d& theMatrix)
{
    return IsRigidWorldAnchorTransform(theMatrix)
        && std::abs(theMatrix.values[12])
            <= limits::kMaximumModelCoordinateMagnitude
        && std::abs(theMatrix.values[13])
            <= limits::kMaximumModelCoordinateMagnitude
        && std::abs(theMatrix.values[14])
            <= limits::kMaximumModelCoordinateMagnitude;
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

bool ReferenceSpaceFromOcct(const OcctReferenceSpace theSource,
                            ReferenceSpace& theDestination)
{
    switch (theSource) {
        case OcctReferenceSpace::Object:
            theDestination = ReferenceSpace::Object;
            return true;
        case OcctReferenceSpace::World:
            theDestination = ReferenceSpace::World;
            return true;
    }
    return false;
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
    std::unordered_set<std::int32_t> validatedNumericResources;
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

// E3 face-image appearance (278b portion 4). The durable record UUIDs are
// published as lowercase RFC-9562 text, matching the document identity
// strings already used by this builder.
std::string FaceImageUUIDText(const core3d::retained_recipe::UUID& theUUID)
{
    static const char* const kDigits = "0123456789abcdef";
    std::string aText;
    aText.reserve(36);
    for (std::size_t anIndex = 0; anIndex < theUUID.size(); ++anIndex) {
        if (anIndex == 4 || anIndex == 6 || anIndex == 8 || anIndex == 10) {
            aText.push_back('-');
        }
        aText.push_back(kDigits[theUUID[anIndex] >> 4]);
        aText.push_back(kDigits[theUUID[anIndex] & 0x0f]);
    }
    return aText;
}

// Publishes the document-owned working bytes of one face-image resource with
// the same content identity, metadata checks and aggregate budgets as an
// embedded XCAF texture. The resource UUID is the dedup source identifier;
// identical working content shares one immutable entry.
bool AddFaceImageTextureResource(
    SceneSnapshot& theScene,
    TextureTableState& theState,
    const core3d::face_image::ResourceEnvelope& theEnvelope,
    std::int32_t& theTextureIndex,
    const bool thePaintedAtlasDerivative = false)
{
    theTextureIndex = -1;
    const std::vector<std::uint8_t>& aBytes = theEnvelope.workingBytes;
    if (aBytes.empty() || aBytes.size() > kMaxEncodedTextureBytes) {
        return false;
    }
    const std::string aSourceIdentifier =
        "face-image-" + FaceImageUUIDText(theEnvelope.resource);
    const auto aSourceFound =
        theState.resourcesBySourceIdentifier.find(aSourceIdentifier);
    if (aSourceFound != theState.resourcesBySourceIdentifier.end()) {
        const std::size_t anExistingIndex = aSourceFound->second;
        if (anExistingIndex >= theScene.textures.size()
            || theScene.textures[anExistingIndex].encodedBytes.size()
                != aBytes.size()
            || std::memcmp(theScene.textures[anExistingIndex]
                               .encodedBytes.data(),
                           aBytes.data(),
                           aBytes.size()) != 0) {
            return false;
        }
        if (thePaintedAtlasDerivative) {
            theScene.textures[anExistingIndex].paintedAtlasDerivative = true;
        }
        theTextureIndex = static_cast<std::int32_t>(anExistingIndex);
        return true;
    }

    const std::string aDigest =
        SHA256Identifier(aBytes.data(), aBytes.size());
    if (aDigest.empty()) {
        return false;
    }
    const auto aDigestFound = theState.resourcesByDigest.find(aDigest);
    if (aDigestFound != theState.resourcesByDigest.end()) {
        for (const std::size_t anExistingIndex : aDigestFound->second) {
            const TextureResourceSnapshot& anExisting =
                theScene.textures[anExistingIndex];
            if (anExisting.encodedBytes.size() == aBytes.size()
                && std::memcmp(anExisting.encodedBytes.data(),
                               aBytes.data(),
                               aBytes.size()) == 0) {
                theState.resourcesBySourceIdentifier.emplace(
                    aSourceIdentifier, anExistingIndex);
                if (thePaintedAtlasDerivative) {
                    theScene.textures[anExistingIndex].paintedAtlasDerivative = true;
                }
                theTextureIndex = static_cast<std::int32_t>(anExistingIndex);
                return true;
            }
        }
        // A content identifier is an exact cache and revision invariant. A
        // cryptographic collision cannot be represented safely; fail closed.
        return false;
    }

    Handle(NCollection_Buffer) aBuffer = new NCollection_Buffer(
        NCollection_BaseAllocator::CommonBaseAllocator(), aBytes.size());
    if (aBuffer.IsNull() || aBuffer->ChangeData() == nullptr
        || aBuffer->Size() != aBytes.size()) {
        return false;
    }
    std::memcpy(aBuffer->ChangeData(), aBytes.data(), aBytes.size());
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
    // The envelope's own validated metadata must agree with the carried
    // bytes; a mislabeled resource never enters the snapshot.
    const bool anEnvelopePNG = theEnvelope.workingFormat
        == core3d::face_image::ImageEncoding::PNG;
    if (aWidth != theEnvelope.workingWidthTexels
        || aHeight != theEnvelope.workingHeightTexels
        || (anEncoding == TextureEncoding::PNG) != anEnvelopePNG) {
        return false;
    }

    if (theScene.textures.size() >= kMaxTexturesPerSnapshot
        || theScene.textures.size()
            > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())
        || aBytes.size() > kMaxAggregateEncodedTextureBytes
        || theState.aggregateEncodedBytes
            > kMaxAggregateEncodedTextureBytes - aBytes.size()
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
    aResource.encodedBytes = aBytes;
    aResource.paintedAtlasDerivative = thePaintedAtlasDerivative;
    const std::size_t aResourceIndex = theScene.textures.size();
    theScene.textures.push_back(std::move(aResource));
    theState.resourcesByDigest[aDigest].push_back(aResourceIndex);
    theState.resourcesBySourceIdentifier.emplace(aSourceIdentifier,
                                                 aResourceIndex);
    theState.aggregateEncodedBytes += aBytes.size();
    theState.aggregateDecodedBytes += aDecodedBytes;
    theTextureIndex = static_cast<std::int32_t>(aResourceIndex);
    return true;
}

std::string PaintedAtlasDigestText(
    const core3d::painted_atlas_bake::Digest& theDigest)
{
    static const char* const kDigits = "0123456789abcdef";
    std::string aText;
    aText.reserve(theDigest.size() * 2);
    for (const std::uint8_t aByte : theDigest) {
        aText.push_back(kDigits[aByte >> 4]);
        aText.push_back(kDigits[aByte & 0x0f]);
    }
    return aText;
}

std::string DecalDigestText(const core3d::decal_layer::Digest& theDigest)
{
    static const char* const kDigits = "0123456789abcdef";
    std::string aText; aText.reserve(theDigest.size() * 2);
    for (const std::uint8_t aByte : theDigest) {
        aText.push_back(kDigits[aByte >> 4]);
        aText.push_back(kDigits[aByte & 0x0f]);
    }
    return aText;
}

struct FaceImageValidationAdmission final {
    core3d::decal_layer::bake::accounting::View operation;
    core3d::decal_layer::bake::accounting::Ticket encoded;
    core3d::decal_layer::bake::accounting::Ticket metadata;
    core3d::decal_layer::bake::accounting::Ticket decoded;
};

bool AdmitFaceImageValidationSource(void *raw, std::size_t bytes) noexcept
{
    namespace accounting = core3d::decal_layer::bake::accounting;
    auto& admission = *static_cast<FaceImageValidationAdmission *>(raw);
    return admission.encoded.acquire(admission.operation,
            accounting::StorageDimension::EncodedTexture, bytes,
            accounting::Retention::Scratch,
            accounting::FailureSite::FaceImageValidationSource)
        && admission.metadata.acquire(admission.operation,
            accounting::StorageDimension::EncodedTexture, bytes,
            accounting::Retention::Scratch,
            accounting::FailureSite::FaceImageValidationSource);
}

bool AdmitFaceImageValidationDecoded(
    void *raw, std::size_t width, std::size_t height) noexcept
{
    namespace accounting = core3d::decal_layer::bake::accounting;
    auto& admission = *static_cast<FaceImageValidationAdmission *>(raw);
    return admission.decoded.acquireProduct(admission.operation,
        accounting::StorageDimension::PrivateStorage, width, height, 4,
        accounting::Retention::Scratch,
        accounting::FailureSite::FaceImageValidationDecoded);
}

bool EnterFaceImageValidationDecode(void *raw) noexcept
{
    auto& admission = *static_cast<FaceImageValidationAdmission *>(raw);
    return admission.operation.EnterAllocation(
        core3d::decal_layer::bake::accounting::FailureSite::
            FaceImageValidationDecode);
}

bool ResolveDecalImage(
    const Handle(OcctDocument)& theDocument,
    const core3d::decal_layer::ImageRef& theReference,
    core3d::decal_layer::bake::ResolvedImage& theOutput,
    const core3d::decal_layer::bake::accounting::View& theOperation)
{
    theOutput = {};
    @try {
        namespace fi = core3d::face_image;
        namespace dl = core3d::decal_layer;
        if (theDocument.IsNull() || !theOperation.valid()
            || !dl::image_contract::Supported(theReference)) return false;
        core3d::painted_atlas_bake::owner::ResourceReadAdmission
            aReadAdmission;
        if (core3d::painted_atlas_bake::owner::PreflightResourceRead(
                theDocument->Document(), theReference.resource,
                theOperation, aReadAdmission)
                != core3d::painted_atlas_bake::owner::Outcome::Prepared
            || !theOperation.EnterAllocation(
                dl::bake::accounting::FailureSite::ResourceReadMaterialize))
            return false;
        fi::ResourceEnvelope anEnvelope;
#ifdef DEBUG
        ++gDebugResolveReadResourceEntries;
#endif
        if (!fi::owner::ReadResource(theDocument->Document(),
                theReference.resource, anEnvelope)) return false;
        dl::bake::accounting::Ticket anOriginalTicket, aWorkingTicket;
        if (!anOriginalTicket.acquire(theOperation,
                dl::bake::accounting::StorageDimension::EncodedTexture,
                anEnvelope.originalBytes.size(),
                dl::bake::accounting::Retention::Scratch,
                dl::bake::accounting::FailureSite::ImageEncodedCopy)
            || !aWorkingTicket.acquire(theOperation,
                dl::bake::accounting::StorageDimension::EncodedTexture,
                anEnvelope.workingBytes.size(),
                dl::bake::accounting::Retention::Scratch,
                dl::bake::accounting::FailureSite::ImageEncodedCopy))
            return false;
        NSData *anOriginal = [NSData dataWithBytes:anEnvelope.originalBytes.data()
            length:anEnvelope.originalBytes.size()];
        NSData *aWorking = [NSData dataWithBytes:anEnvelope.workingBytes.data()
            length:anEnvelope.workingBytes.size()];
        fi::validation::FaceImageRasterInfo anOriginalInfo, aWorkingInfo;
        FaceImageValidationAdmission anOriginalAdmission;
        anOriginalAdmission.operation = theOperation;
        const fi::validation::FaceImageRasterOptions anOriginalOptions{
            &anOriginalAdmission, &AdmitFaceImageValidationSource,
            &AdmitFaceImageValidationDecoded,
            &EnterFaceImageValidationDecode};
        if (!fi::validation::MeasureFaceImageRaster(
                anOriginal, anOriginalInfo, anOriginalOptions))
            return false;
        if (anEnvelope.workingBytes == anEnvelope.originalBytes) {
            aWorkingInfo = anOriginalInfo;
        } else {
            FaceImageValidationAdmission aWorkingAdmission;
            aWorkingAdmission.operation = theOperation;
            const fi::validation::FaceImageRasterOptions aWorkingOptions{
                &aWorkingAdmission, &AdmitFaceImageValidationSource,
                &AdmitFaceImageValidationDecoded,
                &EnterFaceImageValidationDecode};
            if (!fi::validation::MeasureFaceImageRaster(
                    aWorking, aWorkingInfo, aWorkingOptions))
                return false;
        }
        if (anOriginalInfo.format != anEnvelope.originalFormat
            || anOriginalInfo.width != anEnvelope.originalWidthTexels
            || anOriginalInfo.height != anEnvelope.originalHeightTexels
            || aWorkingInfo.format != anEnvelope.workingFormat
            || aWorkingInfo.width != anEnvelope.workingWidthTexels
            || aWorkingInfo.height != anEnvelope.workingHeightTexels)
            return false;
        core3d::painted_atlas_bake::kernel::Image aDecoded;
        if (!core3d::painted_atlas_bake::kernel::DecodeImage(
                anEnvelope.workingBytes, aDecoded, theOperation)
            || aDecoded.width != anEnvelope.workingWidthTexels
            || aDecoded.height != anEnvelope.workingHeightTexels)
            return false;
        dl::bake::ResolvedImage aResolved;
        aResolved.reference = theReference;
        aResolved.envelope = std::move(anEnvelope);
        aResolved.originalBytesTicket =
            std::move(aReadAdmission.originalBytes);
        aResolved.workingBytesTicket =
            std::move(aReadAdmission.workingBytes);
        aResolved.pixels.width = aDecoded.width;
        aResolved.pixels.height = aDecoded.height;
        aResolved.pixels.rgba = std::move(aDecoded.rgba);
        aResolved.pixelTicket = std::move(aDecoded.rgbaTicket);
        aResolved.originalMeasured = true;
        aResolved.workingMeasured = true;
        aResolved.workingHasAlpha = aWorkingInfo.hasAlpha;
        if (!dl::bake::Valid(aResolved, theReference)) return false;
        theOutput = std::move(aResolved);
        return true;
    } @catch (...) { theOutput = {}; return false; }
}

namespace decal_math {

struct DecalWorldTriangle final {
    std::array<Double3, 3> points;
    std::size_t instanceIndex = 0;
    std::uint32_t faceIndex = 0;
};

Double3 Subtract(const Double3& theLeft, const Double3& theRight)
{
    return {theLeft.x - theRight.x, theLeft.y - theRight.y,
            theLeft.z - theRight.z};
}
Double3 Add(const Double3& theLeft, const Double3& theRight)
{
    return {theLeft.x + theRight.x, theLeft.y + theRight.y,
            theLeft.z + theRight.z};
}
Double3 Scale(const Double3& theValue, const double theScale)
{
    return {theValue.x * theScale, theValue.y * theScale,
            theValue.z * theScale};
}
double Dot(const Double3& theLeft, const Double3& theRight)
{
    return theLeft.x * theRight.x + theLeft.y * theRight.y
        + theLeft.z * theRight.z;
}
Double3 Cross(const Double3& theLeft, const Double3& theRight)
{
    return {
        theLeft.y * theRight.z - theLeft.z * theRight.y,
        theLeft.z * theRight.x - theLeft.x * theRight.z,
        theLeft.x * theRight.y - theLeft.y * theRight.x,
    };
}
bool Normalize(Double3& theValue)
{
    const double aLength = std::sqrt(Dot(theValue, theValue));
    if (!IsFinite(aLength) || aLength <= 0.0) return false;
    theValue = Scale(theValue, 1.0 / aLength); return true;
}
Double3 TransformVector(const Matrix4d& theMatrix, const Double3& theValue)
{
    return {
        theMatrix.values[0] * theValue.x
            + theMatrix.values[4] * theValue.y
            + theMatrix.values[8] * theValue.z,
        theMatrix.values[1] * theValue.x
            + theMatrix.values[5] * theValue.y
            + theMatrix.values[9] * theValue.z,
        theMatrix.values[2] * theValue.x
            + theMatrix.values[6] * theValue.y
            + theMatrix.values[10] * theValue.z,
    };
}

bool CaptureDecalOccluders(const SceneSnapshot& theScene,
                           std::vector<DecalWorldTriangle>& theTriangles,
                           Bounds3d& theBounds,
                           core3d::decal_layer::Digest& theProof)
{
    theTriangles.clear(); theBounds = {}; theProof = {};
    try {
        core3d::face_image::detail::Writer aWriter;
        aWriter.raw(reinterpret_cast<const std::uint8_t*>("E4OC"), 4);
        aWriter.integer(1, 4);
        for (std::size_t anInstanceIndex = 0;
             anInstanceIndex < theScene.instances.size(); ++anInstanceIndex) {
            const InstanceSnapshot& anInstance =
                theScene.instances[anInstanceIndex];
            if (anInstance.role != RenderRole::Model
                || anInstance.meshIndex >= theScene.meshes.size()) continue;
            const MeshSnapshot& aMesh = theScene.meshes[anInstance.meshIndex];
            aWriter.integer(anInstance.entityIdentifier.size(), 4);
            aWriter.raw(reinterpret_cast<const std::uint8_t*>(
                    anInstance.entityIdentifier.data()),
                anInstance.entityIdentifier.size());
            for (double aValue : anInstance.worldFromObject.values)
                aWriter.real(aValue);
            for (const MeshPrimitive& aPrimitive : aMesh.primitives) {
                if (aPrimitive.indexCount == 0
                    || aPrimitive.indexCount % 3 != 0
                    || aPrimitive.firstIndex > aMesh.indices.size()
                    || aPrimitive.indexCount
                        > aMesh.indices.size() - aPrimitive.firstIndex)
                    return false;
                for (std::size_t anOffset = 0;
                     anOffset < aPrimitive.indexCount; anOffset += 3) {
                    if (theTriangles.size() >= kMaxIndicesPerSnapshot / 3)
                        return false;
                    DecalWorldTriangle aTriangle;
                    aTriangle.instanceIndex = anInstanceIndex;
                    aTriangle.faceIndex = aPrimitive.faceIndex;
                    for (std::size_t aCorner = 0; aCorner < 3; ++aCorner) {
                        const std::uint32_t aVertexIndex = aMesh.indices[
                            aPrimitive.firstIndex + anOffset + aCorner];
                        if (aVertexIndex >= aMesh.vertices.size()) return false;
                        const Vertex& aVertex = aMesh.vertices[aVertexIndex];
                        Double3 aWorld;
                        if (!TransformPoint(anInstance.worldFromObject,
                                {aVertex.positionX, aVertex.positionY,
                                 aVertex.positionZ}, aWorld)) return false;
                        aTriangle.points[aCorner] = aWorld;
                        if (!theBounds.valid) {
                            theBounds.minimum = theBounds.maximum = aWorld;
                            theBounds.valid = true;
                        } else {
                            theBounds.minimum.x = std::min(
                                theBounds.minimum.x, aWorld.x);
                            theBounds.minimum.y = std::min(
                                theBounds.minimum.y, aWorld.y);
                            theBounds.minimum.z = std::min(
                                theBounds.minimum.z, aWorld.z);
                            theBounds.maximum.x = std::max(
                                theBounds.maximum.x, aWorld.x);
                            theBounds.maximum.y = std::max(
                                theBounds.maximum.y, aWorld.y);
                            theBounds.maximum.z = std::max(
                                theBounds.maximum.z, aWorld.z);
                        }
                        aWriter.real(aWorld.x); aWriter.real(aWorld.y);
                        aWriter.real(aWorld.z);
                    }
                    aWriter.integer(aPrimitive.faceIndex, 4);
                    theTriangles.push_back(std::move(aTriangle));
                }
            }
        }
        return aWriter.ok && theBounds.valid && !theTriangles.empty()
            && core3d::face_image::HashFaceImageBytes(
                aWriter.bytes, theProof)
            && core3d::decal_layer::Nonzero(theProof);
    } catch (...) {
        theTriangles.clear(); theBounds = {}; theProof = {}; return false;
    }
}

bool IntersectRayTriangle(const Double3& theOrigin,
                          const Double3& theDirection,
                          const DecalWorldTriangle& theTriangle,
                          const double theTolerance,
                          double& theDistance)
{
    theDistance = 0.0;
    const Double3 anEdge1 = Subtract(
        theTriangle.points[1], theTriangle.points[0]);
    const Double3 anEdge2 = Subtract(
        theTriangle.points[2], theTriangle.points[0]);
    const Double3 aCross = Cross(theDirection, anEdge2);
    const double aDeterminant = Dot(anEdge1, aCross);
    if (!IsFinite(aDeterminant)
        || std::abs(aDeterminant) <= theTolerance) return false;
    const double anInverse = 1.0 / aDeterminant;
    const Double3 anOffset = Subtract(theOrigin, theTriangle.points[0]);
    const double u = Dot(anOffset, aCross) * anInverse;
    if (u < -theTolerance || u > 1.0 + theTolerance) return false;
    const Double3 q = Cross(anOffset, anEdge1);
    const double v = Dot(theDirection, q) * anInverse;
    if (v < -theTolerance || u + v > 1.0 + theTolerance) return false;
    const double distance = Dot(anEdge2, q) * anInverse;
    if (!IsFinite(distance) || distance < -theTolerance) return false;
    theDistance = distance; return true;
}

} // namespace decal_math

struct OrdinaryDecalFaceInputs final {
    const Handle(OcctDocument)& document;
    const TopoDS_Face& nativeFace;
    const core3d::dependent_replay::FaceImageGeometricReceipt& faceReceipt;
    const std::vector<Vertex>& vertices;
    const std::vector<std::uint32_t>& indices;
    const std::vector<MeshPrimitive>& primitives;
    const MeshPrimitive& primitive;
    TangentBasis tangentBasis;
    const std::vector<Float4>& cornerTangents;
    const Double3& sourceOrigin;
    const Matrix4d& worldFromObject;
    bool reversesWinding;
    std::size_t instanceIndex;
    std::size_t faceIndex;
    const MaterialSnapshot& material;
    const std::vector<FaceImageBindingSnapshot>& faceImageBindings;
    const std::vector<TextureResourceSnapshot>& textures;
    const core3d::decal_layer::Definition& definition;
    const std::vector<std::size_t>& layerIndices;
    const std::vector<decal_math::DecalWorldTriangle>& occluders;
    double occluderSpan;
    const core3d::decal_layer::Digest& occluderProof;
    const std::vector<std::uint8_t>& canonicalLayers;
    const core3d::decal_layer::Digest& sourceProof;
    double metersPerUnit;
    core3d::decal_layer::bake::accounting::View operation;
    core3d::retained_edge_treatment::ReplayBudget& budget;
};

// Build the ordinary receiver input from explicit borrowed authority. Keeping
// this adapter implementation-local makes the exact sampler callable without
// promoting native faces or snapshot internals into a shared header.
bool PrepareOrdinaryDecalFace(
    const OrdinaryDecalFaceInputs& theInputs,
    core3d::decal_layer::bake::Input& theOutput,
    core3d::decal_layer::ReceiverReceipt& theReceiver)
{
    namespace dl = core3d::decal_layer;
    using namespace decal_math;
    (void)theInputs.reversesWinding;
    if (!theInputs.operation.valid()) return false;
#ifdef DEBUG
    if (gDebugOrdinaryLedgerObservationArmed) {
        if (gDebugOrdinaryLedgerCalls == 0)
            gDebugOrdinaryLedgerContext = theInputs.operation.context;
        else if (gDebugOrdinaryLedgerContext != theInputs.operation.context)
            gDebugOrdinaryLedgerMismatch = true;
        ++gDebugOrdinaryLedgerCalls;
    }
#endif
    // Resolve and own every layer/mask resource before any
    // raster or publication allocation.
    std::vector<dl::bake::ResolvedLayer> aResolvedLayers;
    aResolvedLayers.reserve(theInputs.layerIndices.size());
    std::uint32_t anOutputWidth = 1, anOutputHeight = 1;
    for (const std::size_t aLayerIndex : theInputs.layerIndices) {
        const dl::Layer& anIntent =
            theInputs.definition.layers[aLayerIndex];
        dl::bake::ResolvedLayer aResolved;
        aResolved.intent = anIntent;
        if (!ResolveDecalImage(
                theInputs.document, anIntent.image, aResolved.image,
                theInputs.operation))
            return false;
        if (anIntent.mask.present) {
            dl::bake::ResolvedImage aMask;
            if (!ResolveDecalImage(
                    theInputs.document, anIntent.mask.image, aMask,
                    theInputs.operation))
                return false;
            aResolved.mask = std::move(aMask);
        }
        anOutputWidth = std::max(
            anOutputWidth, anIntent.image.widthTexels);
        anOutputHeight = std::max(
            anOutputHeight, anIntent.image.heightTexels);
        aResolved.completeOccludersProved = true;
        aResolved.boundaryProved = true;
        aResolvedLayers.push_back(std::move(aResolved));
    }
    // The deterministic ordinary chart raster is bounded
    // independently of authored input size. Normalized UVs
    // preserve exact mesh/image correspondence at any size.
    anOutputWidth = std::min<std::uint32_t>(
        anOutputWidth, 64);
    anOutputHeight = std::min<std::uint32_t>(
        anOutputHeight, 64);

    dl::bake::Raster aBase;
    aBase.width = anOutputWidth;
    aBase.height = anOutputHeight;
    aBase.rgba.resize(std::size_t(anOutputWidth)
        * anOutputHeight * 4);
    std::int32_t aBaseTextureIndex =
        theInputs.material.baseColorTextureIndex;
    bool aBaseSRGB = true;
    core3d::face_image::UVTransform aBaseTransform;
    for (const FaceImageBindingSnapshot& aBinding :
         theInputs.faceImageBindings) {
        if (aBinding.faceIndex != theInputs.faceIndex
            || aBinding.role != FaceImageRole::BaseColor)
            continue;
        if (aBaseTextureIndex >= 0
            && aBaseTextureIndex != aBinding.textureIndex)
            return false;
        aBaseTextureIndex = aBinding.textureIndex;
        aBaseSRGB = aBinding.srgbColorSpace;
        aBaseTransform.scale = {
            aBinding.transform.scaleU,
            aBinding.transform.scaleV};
        aBaseTransform.offset = {
            aBinding.transform.offsetU,
            aBinding.transform.offsetV};
        aBaseTransform.rotationDegrees =
            aBinding.transform.rotationDegrees;
        aBaseTransform.wrapU = static_cast<core3d::face_image::Wrap>(
            aBinding.transform.wrapU);
        aBaseTransform.wrapV = static_cast<core3d::face_image::Wrap>(
            aBinding.transform.wrapV);
    }
    dl::bake::accounting::Ticket aBaseSourceTicket;
    std::optional<dl::bake::Raster> aBaseSource;
    if (aBaseTextureIndex >= 0) {
        if (std::size_t(aBaseTextureIndex)
            >= theInputs.textures.size()) return false;
        core3d::painted_atlas_bake::kernel::Image aDecoded;
        if (!core3d::painted_atlas_bake::kernel::DecodeImage(
                theInputs.textures[aBaseTextureIndex].encodedBytes,
                aDecoded, theInputs.operation)) return false;
        dl::bake::Raster aRaster;
        aRaster.width = aDecoded.width;
        aRaster.height = aDecoded.height;
        aRaster.rgba = std::move(aDecoded.rgba);
        aBaseSourceTicket = std::move(aDecoded.rgbaTicket);
        if (!dl::bake::Valid(aRaster)) return false;
        aBaseSource = std::move(aRaster);
    }
    for (std::uint32_t y = 0; y < anOutputHeight; ++y) {
        for (std::uint32_t x = 0; x < anOutputWidth; ++x) {
            const dl::bake::UV anOutputUV{
                (double(x) + 0.5) / anOutputWidth,
                1.0 - (double(y) + 0.5) / anOutputHeight};
            std::array<double, 4> aSample{1.0, 1.0, 1.0, 1.0};
            if (aBaseSource.has_value()) {
                const dl::bake::UV aSourceUV =
                    dl::bake::Transform(
                        anOutputUV, aBaseTransform);
                aSample = dl::bake::Sample(*aBaseSource,
                    aSourceUV, aBaseSRGB,
                    aBaseTransform.wrapU,
                    aBaseTransform.wrapV, false);
            }
            const std::size_t anOffset =
                (std::size_t(y) * anOutputWidth + x) * 4;
            aBase.rgba[anOffset] = dl::bake::LinearToSRGB(
                aSample[0] * theInputs.material.baseColor.x);
            aBase.rgba[anOffset + 1] = dl::bake::LinearToSRGB(
                aSample[1] * theInputs.material.baseColor.y);
            aBase.rgba[anOffset + 2] = dl::bake::LinearToSRGB(
                aSample[2] * theInputs.material.baseColor.z);
            aBase.rgba[anOffset + 3] = dl::bake::LinearByte(
                aSample[3] * theInputs.material.baseColor.w);
        }
    }

    const Double3 aNormal{
        theInputs.faceReceipt.plane.outwardNormal[0],
        theInputs.faceReceipt.plane.outwardNormal[1],
        theInputs.faceReceipt.plane.outwardNormal[2]};
    const double anOffsetLocal =
        theInputs.faceReceipt.plane.offsetMM
            / (1000.0 * theInputs.metersPerUnit);
    const Double3 aPlaneOrigin = Scale(aNormal, anOffsetLocal);
    const int anAxis = std::abs(aNormal.x) >= std::abs(aNormal.y)
            && std::abs(aNormal.x) >= std::abs(aNormal.z) ? 0
        : std::abs(aNormal.y) >= std::abs(aNormal.z) ? 1 : 2;
    const Double3 aReference = anAxis == 0
        ? Double3{0, 1, 0}
        : anAxis == 1 ? Double3{0, 0, 1}
                      : Double3{1, 0, 0};
    Double3 aFaceU = Subtract(aReference,
        Scale(aNormal, Dot(aNormal, aReference)));
    if (!Normalize(aFaceU)) return false;
    Double3 aFaceV = Cross(aNormal, aFaceU);
    if (!Normalize(aFaceV)) return false;
    const auto quaternionAxis = [](const std::array<double, 4>& q,
                                   const Double3& v) {
        const Double3 qv{q[0], q[1], q[2]};
        return Add(v, Add(
            Scale(Cross(qv, v), 2.0 * q[3]),
            Scale(Cross(qv, Cross(qv, v)), 2.0)));
    };

    for (dl::bake::ResolvedLayer& aLayer : aResolvedLayers) {
        const dl::Layer& anIntent = aLayer.intent;
        Double3 anOrigin, aLayerU, aLayerV, aRayLocal;
        double aNearLocal = 0.0, aFarLocal = 0.0;
        if (anIntent.placement.kind == dl::PlacementKind::Face) {
            anOrigin = Add(aPlaneOrigin, Add(
                Scale(aFaceU,
                    anIntent.placement.face.anchorMeters[0]
                        / theInputs.metersPerUnit),
                Scale(aFaceV,
                    anIntent.placement.face.anchorMeters[1]
                        / theInputs.metersPerUnit)));
            const double c = std::cos(
                anIntent.placement.face.angleRadians);
            const double s = std::sin(
                anIntent.placement.face.angleRadians);
            aLayerU = Add(Scale(aFaceU, c), Scale(aFaceV, s));
            aLayerV = Add(Scale(aFaceV, c), Scale(aFaceU, -s));
            aRayLocal = Scale(aNormal, -1.0);
        } else {
            const auto& aProjector =
                anIntent.placement.projector;
            anOrigin = {
                aProjector.frame.origin[0] / theInputs.metersPerUnit,
                aProjector.frame.origin[1] / theInputs.metersPerUnit,
                aProjector.frame.origin[2] / theInputs.metersPerUnit};
            aLayerU = quaternionAxis(
                aProjector.frame.orientation, {1, 0, 0});
            aLayerV = quaternionAxis(
                aProjector.frame.orientation, {0, 1, 0});
            aRayLocal = Scale(quaternionAxis(
                aProjector.frame.orientation, {0, 0, 1}), -1.0);
            aNearLocal = aProjector.nearDepthMeters
                / theInputs.metersPerUnit;
            aFarLocal = aProjector.farDepthMeters
                / theInputs.metersPerUnit;
        }
        if (!Normalize(aLayerU) || !Normalize(aLayerV)
            || !Normalize(aRayLocal)) return false;
        Double3 aRayWorld = TransformVector(
            theInputs.worldFromObject, aRayLocal);
        if (!Normalize(aRayWorld)) return false;

        // Continuous rejectCrossing proof uses the four
        // physical footprint corners against native trims.
        bool aContained = true;
        for (double u : {-0.5, 0.5}) for (double v : {-0.5, 0.5}) {
            Double3 aCorner = Add(anOrigin, Add(
                Scale(aLayerU, u * anIntent.widthMeters
                    / theInputs.metersPerUnit),
                Scale(aLayerV, v * anIntent.heightMeters
                    / theInputs.metersPerUnit)));
            if (anIntent.placement.kind
                == dl::PlacementKind::OrthographicProjector) {
                const double denominator = Dot(aRayLocal, aNormal);
                if (std::abs(denominator) <= 1.0e-8) {
                    aContained = false; break;
                }
                const double t = (anOffsetLocal
                    - Dot(aNormal, aCorner)) / denominator;
                aCorner = Add(aCorner, Scale(aRayLocal, t));
            }
            BRepClass_FaceClassifier aClassifier(
                theInputs.nativeFace,
                gp_Pnt(aCorner.x, aCorner.y, aCorner.z),
                1.0e-7 / theInputs.metersPerUnit);
            if (aClassifier.State() != TopAbs_IN
                && aClassifier.State() != TopAbs_ON) {
                aContained = false; break;
            }
        }
        aLayer.footprintContained = aContained;
        if (anIntent.placement.edgePolicy
                == dl::EdgePolicy::RejectCrossing
            && !aContained) return false;

        if (theInputs.primitive.firstIndex > theInputs.indices.size()
            || theInputs.primitive.indexCount
                > theInputs.indices.size() - theInputs.primitive.firstIndex
            || theInputs.primitive.indexCount % 3 != 0) return false;
        for (std::size_t anOffset = 0;
             anOffset < theInputs.primitive.indexCount;
             anOffset += 3) {
            dl::bake::Triangle aTriangle;
            aTriangle.receiver =
                dl::bake::IntentReceipt(anIntent);
            std::array<Double3, 3> aLocalPoints;
            std::array<Double3, 3> aWorldPoints;
            for (std::size_t aCorner = 0; aCorner < 3; ++aCorner) {
                const std::uint32_t aVertexIndex = theInputs.indices[
                    theInputs.primitive.firstIndex + anOffset + aCorner];
                if (aVertexIndex >= theInputs.vertices.size()) return false;
                const Vertex& aVertex = theInputs.vertices[aVertexIndex];
                if (!IsFinite(aVertex.textureU)
                    || !IsFinite(aVertex.textureV)
                    || aVertex.textureU < 0.0f
                    || aVertex.textureU > 1.0f
                    || aVertex.textureV < 0.0f
                    || aVertex.textureV > 1.0f) return false;
                aTriangle.outputPixels[aCorner] = {
                    double(aVertex.textureU) * anOutputWidth,
                    (1.0 - double(aVertex.textureV))
                        * anOutputHeight};
                aLocalPoints[aCorner] = {
                    aVertex.positionX, aVertex.positionY,
                    aVertex.positionZ};
                const Double3 anAuthorityPoint = Add(
                    aLocalPoints[aCorner], theInputs.sourceOrigin);
                aTriangle.layerUV[aCorner] = {
                    0.5 + Dot(Subtract(anAuthorityPoint, anOrigin),
                        aLayerU) * theInputs.metersPerUnit
                        / anIntent.widthMeters,
                    0.5 + Dot(Subtract(anAuthorityPoint, anOrigin),
                        aLayerV) * theInputs.metersPerUnit
                        / anIntent.heightMeters};
                if (!TransformPoint(theInputs.worldFromObject,
                        aLocalPoints[aCorner],
                        aWorldPoints[aCorner])) return false;
            }
            Double3 aTriangleNormal = Cross(
                Subtract(aWorldPoints[1], aWorldPoints[0]),
                Subtract(aWorldPoints[2], aWorldPoints[0]));
            if (!Normalize(aTriangleNormal)) return false;
            const bool aFrontFacing = Dot(
                aTriangleNormal, Scale(aRayWorld, -1.0)) > 1.0e-8;
            double aMinX = aTriangle.outputPixels[0].x;
            double aMaxX = aMinX, aMinY = aTriangle.outputPixels[0].y;
            double aMaxY = aMinY;
            for (const auto& aPoint : aTriangle.outputPixels) {
                aMinX = std::min(aMinX, aPoint.x);
                aMaxX = std::max(aMaxX, aPoint.x);
                aMinY = std::min(aMinY, aPoint.y);
                aMaxY = std::max(aMaxY, aPoint.y);
            }
            const int aFirstX = std::max(0, int(std::floor(aMinX)));
            const int aLastX = std::min(int(anOutputWidth) - 1,
                int(std::ceil(aMaxX)) - 1);
            const int aFirstY = std::max(0, int(std::floor(aMinY)));
            const int aLastY = std::min(int(anOutputHeight) - 1,
                int(std::ceil(aMaxY)) - 1);
            for (int y = aFirstY; y <= aLastY; ++y) {
                for (int x = aFirstX; x <= aLastX; ++x) {
                    std::array<double, 3> aWeights{};
                    if (!dl::bake::BarycentricTopLeft(
                            aTriangle, {x + 0.5, y + 0.5},
                            aWeights)) continue;
                    dl::bake::CoverageSample aSample;
                    aSample.x = std::uint32_t(x);
                    aSample.y = std::uint32_t(y);
                    aSample.receiver = aTriangle.receiver;
                    dl::bake::UV aUV{};
                    Double3 aLocalSample{}, aWorldSample{};
                    for (std::size_t aCorner = 0;
                         aCorner < 3; ++aCorner) {
                        aUV.u += aWeights[aCorner]
                            * aTriangle.layerUV[aCorner].u;
                        aUV.v += aWeights[aCorner]
                            * aTriangle.layerUV[aCorner].v;
                        aLocalSample = Add(aLocalSample,
                            Scale(aLocalPoints[aCorner],
                                aWeights[aCorner]));
                        aWorldSample = Add(aWorldSample,
                            Scale(aWorldPoints[aCorner],
                                aWeights[aCorner]));
                    }
                    aSample.affected = aUV.u >= 0.0 && aUV.u <= 1.0
                        && aUV.v >= 0.0 && aUV.v <= 1.0;
                    if (aSample.affected
                        && anIntent.placement.kind
                            == dl::PlacementKind::OrthographicProjector) {
                        const Double3 anAuthoritySample = Add(
                            aLocalSample, theInputs.sourceOrigin);
                        const double aDepth = Dot(Subtract(
                            anAuthoritySample, anOrigin), aRayLocal);
                        aSample.affected = aDepth >= aNearLocal
                            && aDepth <= aFarLocal;
                    }
                    if (!aSample.affected) {
                        aSample.conclusive = true;
                        aTriangle.coverage.push_back(aSample);
                        continue;
                    }
                    const double aTolerance =
                        1.0e-7 / theInputs.metersPerUnit;
                    double anExpectedDistance = 0.0;
                    Double3 aRayOrigin;
                    if (anIntent.placement.kind
                        == dl::PlacementKind::Face) {
                        anExpectedDistance = theInputs.occluderSpan
                            + aTolerance * 4.0;
                        aRayOrigin = Subtract(aWorldSample,
                            Scale(aRayWorld,
                                anExpectedDistance));
                    } else {
                        const Double3 anAuthoritySample = Add(
                            aLocalSample, theInputs.sourceOrigin);
                        const double aDepth = Dot(Subtract(
                            anAuthoritySample, anOrigin), aRayLocal);
                        anExpectedDistance = aDepth - aNearLocal;
                        aRayOrigin = Subtract(aWorldSample,
                            Scale(aRayWorld,
                                anExpectedDistance));
                    }
                    bool aHit = false, aTie = false;
                    double aNearest = std::numeric_limits<double>::max();
                    std::size_t aNearestInstance = 0;
                    std::uint32_t aNearestFace = 0;
                    for (const DecalWorldTriangle& anOccluder :
                         theInputs.occluders) {
                        if (!theInputs.budget.visit(1,
                                core3d::retained_topology_budget::
                                    Site::C06DirectCensus))
                            return false;
                        double aDistance = 0.0;
                        if (!IntersectRayTriangle(aRayOrigin,
                                aRayWorld, anOccluder,
                                1.0e-12, aDistance)
                            || aDistance
                                > anExpectedDistance + aTolerance)
                            continue;
                        if (!aHit
                            || aDistance < aNearest - aTolerance) {
                            aHit = true; aTie = false;
                            aNearest = aDistance;
                            aNearestInstance =
                                anOccluder.instanceIndex;
                            aNearestFace = anOccluder.faceIndex;
                        } else if (std::abs(
                                aDistance - aNearest)
                            <= aTolerance
                            && (aNearestInstance
                                    != anOccluder.instanceIndex
                                || aNearestFace
                                    != anOccluder.faceIndex)) {
                            aTie = true;
                        }
                    }
                    aSample.conclusive = aHit && !aTie;
                    aSample.selectedReceiver =
                        aNearestInstance == theInputs.instanceIndex
                        && aNearestFace == theInputs.faceIndex;
                    aSample.frontFacing = aFrontFacing;
                    aSample.insideTrim = true;
                    aSample.nearestHit = aSample.selectedReceiver
                        && std::abs(aNearest
                            - anExpectedDistance) <= aTolerance;
                    aTriangle.coverage.push_back(aSample);
                }
            }
            if (aTriangle.coverage.empty()) return false;
            aLayer.triangles.push_back(std::move(aTriangle));
        }
        if (aLayer.triangles.empty()) return false;
    }

    core3d::face_image::detail::Writer aGeometryWriter;
    core3d::face_image::detail::Writer aTangentWriter;
    core3d::face_image::detail::Writer anAppearanceWriter;
    core3d::face_image::detail::Writer anAtlasWriter;
    core3d::face_image::detail::Writer anOccurrenceWriter;
    aGeometryWriter.raw(reinterpret_cast<const std::uint8_t*>("E4FG"), 4);
    aTangentWriter.raw(reinterpret_cast<const std::uint8_t*>("E4TG"), 4);
    anAppearanceWriter.raw(reinterpret_cast<const std::uint8_t*>("E4AP"), 4);
    anAtlasWriter.raw(reinterpret_cast<const std::uint8_t*>("E4AT"), 4);
    anOccurrenceWriter.raw(reinterpret_cast<const std::uint8_t*>("E4OR"), 4);
    for (const Vertex& aVertex : theInputs.vertices) {
        for (float aValue : {aVertex.positionX, aVertex.positionY,
                 aVertex.positionZ, aVertex.normalX, aVertex.normalY,
                 aVertex.normalZ, aVertex.textureU, aVertex.textureV}) {
            aGeometryWriter.real(aValue);
        }
        anAtlasWriter.real(aVertex.textureU);
        anAtlasWriter.real(aVertex.textureV);
    }
    for (std::uint32_t anIndex : theInputs.indices)
        aGeometryWriter.integer(anIndex, 4);
    for (const MeshPrimitive& aValue : theInputs.primitives) {
        aGeometryWriter.integer(aValue.firstIndex, 4);
        aGeometryWriter.integer(aValue.indexCount, 4);
        aGeometryWriter.integer(aValue.faceIndex, 4);
        anAtlasWriter.integer(aValue.faceIndex, 4);
    }
    aTangentWriter.integer(std::uint8_t(theInputs.tangentBasis), 1);
    for (const Float4& aValue : theInputs.cornerTangents) {
        aTangentWriter.real(aValue.x); aTangentWriter.real(aValue.y);
        aTangentWriter.real(aValue.z); aTangentWriter.real(aValue.w);
    }
    for (float aValue : {theInputs.material.baseColor.x, theInputs.material.baseColor.y,
             theInputs.material.baseColor.z, theInputs.material.baseColor.w,
             theInputs.material.emission.x, theInputs.material.emission.y,
             theInputs.material.emission.z, theInputs.material.metallic,
             theInputs.material.roughness, theInputs.material.indexOfRefraction,
             theInputs.material.alphaCutoff})
        anAppearanceWriter.real(aValue);
    for (std::int32_t aTexture : {theInputs.material.baseColorTextureIndex,
             theInputs.material.emissiveTextureIndex,
             theInputs.material.metallicRoughnessTextureIndex,
             theInputs.material.occlusionTextureIndex,
             theInputs.material.normalTextureIndex}) {
        anAppearanceWriter.integer(std::uint32_t(aTexture), 4);
        if (aTexture >= 0) {
            if (std::size_t(aTexture) >= theInputs.textures.size())
                return false;
            dl::Digest aTextureDigest{};
            if (!core3d::face_image::HashFaceImageBytes(
                    theInputs.textures[aTexture].encodedBytes,
                    aTextureDigest)) return false;
            anAppearanceWriter.raw(aTextureDigest);
        }
    }
    for (double aValue : theInputs.worldFromObject.values)
        anOccurrenceWriter.real(aValue);
    dl::Digest aGeometryProof{}, aTangentProof{};
    dl::Digest anAppearanceProof{}, anAtlasProof{};
    dl::Digest anOccurrenceProof{};
    if (!aGeometryWriter.ok || !aTangentWriter.ok
        || !anAppearanceWriter.ok || !anAtlasWriter.ok
        || !anOccurrenceWriter.ok
        || !core3d::face_image::HashFaceImageBytes(
            aGeometryWriter.bytes, aGeometryProof)
        || !core3d::face_image::HashFaceImageBytes(
            aTangentWriter.bytes, aTangentProof)
        || !core3d::face_image::HashFaceImageBytes(
            anAppearanceWriter.bytes, anAppearanceProof)
        || !core3d::face_image::HashFaceImageBytes(
            anAtlasWriter.bytes, anAtlasProof)
        || !core3d::face_image::HashFaceImageBytes(
            anOccurrenceWriter.bytes, anOccurrenceProof))
        return false;
    dl::bake::Input anInput;
    anInput.operationLedger = theInputs.operation;
    anInput.baseColor = std::move(aBase);
    anInput.layers = std::move(aResolvedLayers);
    anInput.canonicalLayers = theInputs.canonicalLayers;
    anInput.sourceProof = theInputs.sourceProof;
    anInput.finalGeometryUVProof = aGeometryProof;
    anInput.occluderProof = theInputs.occluderProof;
    anInput.effectiveAppearanceProof = anAppearanceProof;
    anInput.tangentProof = aTangentProof;
    anInput.atlasProof = anAtlasProof;
    anInput.occurrenceProof = anOccurrenceProof;
    theReceiver = dl::bake::IntentReceipt(
        theInputs.definition.layers[theInputs.layerIndices.front()]);
    theOutput = std::move(anInput);
    return true;
}

// Copy-on-publication final-geometry derivative. The master MeshSnapshot is
// never modified: every triangle corner is split so the regenerated atlas UV
// assignment cannot alias a different chart corner through a shared vertex.
bool ApplyPaintedAtlasDerivative(
    const MeshSnapshot& theMaster,
    const OcctPaintedAtlasDerivative& theDerivative,
    MeshSnapshot& theOutput)
{
    const auto& anAssignment = theDerivative.assignment;
    if (theMaster.geometryKind != GeometryKind::SurfaceTriangles
        || theMaster.indices.empty()
        || anAssignment.corners.size() != theMaster.indices.size()
        || std::uint64_t(anAssignment.triangleCount) * 3
            != theMaster.indices.size()) return false;
    MeshSnapshot aResult = theMaster;
    aResult.vertices.clear();
    aResult.indices.clear();
    aResult.vertices.reserve(theMaster.indices.size());
    aResult.indices.reserve(theMaster.indices.size());
    for (std::size_t anIndex = 0; anIndex < theMaster.indices.size(); ++anIndex) {
        const std::uint32_t aSourceIndex = theMaster.indices[anIndex];
        if (aSourceIndex >= theMaster.vertices.size()) return false;
        const auto& aUV = anAssignment.corners[anIndex];
        if (!IsFinite(aUV[0]) || !IsFinite(aUV[1])
            || aUV[0] < 0.0 || aUV[0] > 1.0
            || aUV[1] < 0.0 || aUV[1] > 1.0) return false;
        Vertex aVertex = theMaster.vertices[aSourceIndex];
        aVertex.textureU = static_cast<float>(aUV[0]);
        aVertex.textureV = static_cast<float>(aUV[1]);
        aResult.vertices.push_back(aVertex);
        aResult.indices.push_back(static_cast<std::uint32_t>(anIndex));
    }
    for (MeshPrimitive& aPrimitive : aResult.primitives) {
        aPrimitive.hasTextureCoordinates = true;
    }
    const std::string aProof = PaintedAtlasDigestText(
        theDerivative.bake.bakeProof);
    if (aProof.size() != 64) return false;
    aResult.paintedAtlasMasterDefinitionIdentifier =
        theMaster.paintedAtlasMasterDefinitionIdentifier.empty()
            ? theMaster.definitionIdentifier
            : theMaster.paintedAtlasMasterDefinitionIdentifier;
    aResult.paintedAtlasBakeProof = aProof;
    aResult.definitionIdentifier =
        aResult.paintedAtlasMasterDefinitionIdentifier + ".baked-" + aProof;
    theOutput = std::move(aResult);
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
                     Handle(Image_Texture)& theEmissiveTexture,
                     Handle(Image_Texture)& theMetallicRoughnessTexture,
                     Handle(Image_Texture)& theOcclusionTexture,
                     Handle(Image_Texture)& theNormalTexture)
{
    theResult = DefaultMaterial(theClosed);
    theBaseColorTexture.Nullify();
    theEmissiveTexture.Nullify();
    theMetallicRoughnessTexture.Nullify();
    theOcclusionTexture.Nullify();
    theNormalTexture.Nullify();
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
        if (!aPbr.IsDefined) {
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
        theMetallicRoughnessTexture = aPbr.MetallicRoughnessTexture;
        theOcclusionTexture = aPbr.OcclusionTexture;
        theNormalTexture = aPbr.NormalTexture;
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
            // Tangent-space normal maps remain unsupported until their
            // explicitly versioned tangent stream is implemented.
            if (!aPbr.NormalTexture.IsNull()) {
                return false;
            }
            theEmissiveTexture = aPbr.EmissiveTexture;
        theMetallicRoughnessTexture = aPbr.MetallicRoughnessTexture;
        theOcclusionTexture = aPbr.OcclusionTexture;
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
            == theRight.emissiveTextureIndex
        && theLeft.metallicRoughnessTextureIndex == theRight.metallicRoughnessTextureIndex
        && theLeft.occlusionTextureIndex == theRight.occlusionTextureIndex
        && theLeft.normalTextureIndex == theRight.normalTextureIndex;
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
    theHash.AddInteger(theMaterial.metallicRoughnessTextureIndex);
    theHash.AddInteger(theMaterial.occlusionTextureIndex);
    theHash.AddInteger(theMaterial.normalTextureIndex);
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
    bool editable = false;
};

struct DefinitionData {
    TDF_Label label;
    TopoDS_Shape shape;
    TopTools_IndexedMapOfShape faces;
    TopTools_IndexedMapOfShape edges;
    TopTools_IndexedMapOfShape vertices;
    MeshSnapshot mesh;
    Double3 sourceOrigin;
    OcctGeometryRepresentation representation =
        OcctGeometryRepresentation::Invalid;
    std::uint64_t fingerprint = 0;
    bool closed = false;
    bool boundedCurve = false;
};

std::uint64_t MeshFingerprint(const MeshSnapshot& theMesh,
                              const double theLinearDeflection,
                              const double theAngularDeflection)
{
    Fingerprint aHash;
    aHash.AddInteger(static_cast<std::uint8_t>(theMesh.geometryKind));
    aHash.AddBool(theMesh.nativeC1Wire.has_value());
    if (theMesh.nativeC1Wire.has_value()) {
        const NativeC1WireSnapshot& aWire = *theMesh.nativeC1Wire;
        aHash.AddInteger<std::uint64_t>(aWire.canonicalDefinitionBytes.size());
        for (const auto aByte : aWire.canonicalDefinitionBytes) aHash.AddByte(aByte);
        aHash.AddInteger<std::uint64_t>(aWire.canonicalOwnerBytes.size());
        for (const auto aByte : aWire.canonicalOwnerBytes) aHash.AddByte(aByte);
        for (const auto aByte : aWire.canonicalDefinitionDigest) aHash.AddByte(aByte);
        aHash.AddInteger(aWire.definitionRevision);
        aHash.AddInteger(aWire.frameRevision);
    }
    aHash.AddString(theMesh.definitionIdentifier);
    aHash.AddDouble(theLinearDeflection);
    aHash.AddDouble(theAngularDeflection);
    aHash.AddInteger(theMesh.topology.faceCount);
    aHash.AddInteger(theMesh.topology.edgeCount);
    aHash.AddInteger(theMesh.topology.vertexCount);
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

void AddTopologyMapFingerprint(
    Fingerprint& theHash,
    const TopTools_IndexedMapOfShape& theMap)
{
    theHash.AddInteger<std::uint64_t>(
        static_cast<std::uint64_t>(theMap.Extent()));
    for (Standard_Integer anIndex = 1;
         anIndex <= theMap.Extent(); ++anIndex) {
        const TopoDS_Shape& aShape = theMap(anIndex);
        theHash.AddInteger(static_cast<std::uint8_t>(aShape.ShapeType()));
        // Builder state is process-local. The OCCT shape hash captures the
        // exact TShape + location identity used by IndexedMap ordering, so an
        // edge/vertex remap cannot silently retain a geometry revision merely
        // because tessellated triangles and cardinalities stayed unchanged.
        theHash.AddInteger<std::size_t>(
            std::hash<TopoDS_Shape>{}(aShape));
    }
}

std::uint64_t DefinitionFingerprint(const DefinitionData& theDefinition)
{
    Fingerprint aHash;
    aHash.AddInteger(MeshFingerprint(theDefinition.mesh, 0.0, 0.0));
    AddTopologyMapFingerprint(aHash, theDefinition.faces);
    AddTopologyMapFingerprint(aHash, theDefinition.edges);
    AddTopologyMapFingerprint(aHash, theDefinition.vertices);
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

bool HasCompatibleArrayPreviewStyle(
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
        // Authored PBR is represented by an actual material aspect, not an
        // OCCT preset name. Boolean highlights still use the fixed role color,
        // but must preserve the admitted opaque scalar appearance. Other
        // overlay kinds retain their existing preset-only contract.
        const bool isBoolean = theRole == RenderRole::BooleanActor
            || theRole == RenderRole::BooleanSubject;
        const auto aMaterialName = thePresentation->Material();
        if (!isBoolean || !theColorOverride.has_value()
            || (aMaterialName != Graphic3d_NameOfMaterial_UserDefined
                && aMaterialName != Graphic3d_NameOfMaterial_DEFAULT)
            || HasUnsupportedPresentationTexture(thePresentation)) {
            return false;
        }
        const auto& anAspect =
            thePresentation->Attributes()->ShadingAspect()->Aspect();
        if (!anAspect->ShaderProgram().IsNull()
            || (anAspect->AlphaMode() != Graphic3d_AlphaMode_Opaque
                && anAspect->AlphaMode() != Graphic3d_AlphaMode_BlendAuto)) {
            return false;
        }
        const auto& aPbr = anAspect->FrontMaterial().PBRMaterial();
        if (!IsFinite(aPbr.Alpha()) || aPbr.Alpha() != 1.0f) {
            return false;
        }
        const auto anEmission = aPbr.Emission();
        aMaterial.emission = {anEmission.x(), anEmission.y(), anEmission.z()};
        aMaterial.metallic = aPbr.Metallic();
        aMaterial.roughness = aPbr.NormalizedRoughness();
        aMaterial.indexOfRefraction = aPbr.IOR();
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
    if (theDefinition.shape.IsNull())
        theDefinition.shape = XCAFDoc_ShapeTool::GetShape(theDefinitionLabel);
    if (theDefinition.shape.IsNull()
        || theDefinition.representation
            == OcctGeometryRepresentation::Invalid) {
        return false;
    }
    theDefinition.faces.Clear();
    theDefinition.edges.Clear();
    theDefinition.vertices.Clear();
    // The label-based RWMesh iterator below replaces the definition's outer
    // placement with identity. Keep mesh face lookup in that same local space;
    // DocumentExplorer supplies the outer placement to the instance matrix.
    TopoDS_Shape extractionShape = theDefinition.shape;
    if (theDefinition.representation == OcctGeometryRepresentation::TriangleMesh)
        extractionShape.Location(TopLoc_Location());
    TopExp::MapShapes(extractionShape, TopAbs_FACE, theDefinition.faces);
    if (theDefinition.faces.IsEmpty()
        || !FitsUInt32(static_cast<std::size_t>(theDefinition.faces.Extent()))
        || static_cast<std::size_t>(theDefinition.faces.Extent()) > kMaxFacesPerMesh) {
        return false;
    }
    const bool hasSemanticBRepTopology =
        theDefinition.representation
        != OcctGeometryRepresentation::TriangleMesh;
    if (hasSemanticBRepTopology) {
        TopExp::MapShapes(
            theDefinition.shape, TopAbs_EDGE, theDefinition.edges);
        TopExp::MapShapes(
            theDefinition.shape, TopAbs_VERTEX, theDefinition.vertices);
        if (!FitsUInt32(
                static_cast<std::size_t>(theDefinition.edges.Extent()))
            || !FitsUInt32(
                static_cast<std::size_t>(theDefinition.vertices.Extent()))
            || static_cast<std::size_t>(theDefinition.edges.Extent())
                > kMaxEdgesPerMesh
            || static_cast<std::size_t>(theDefinition.vertices.Extent())
                > kMaxTopologicalVerticesPerMesh) {
            return false;
        }
    }
    theDefinition.closed = StdPrs_ToolTriangulatedShape::IsClosed(theDefinition.shape);

    std::vector<SourceVertex> aSourceVertices;
    std::vector<std::uint32_t> aSourceIndices;
    std::vector<MeshPrimitive> aPrimitives;
    std::vector<bool> aVisited(static_cast<std::size_t>(theDefinition.faces.Extent()), false);
    Bounds3d aSourceBounds;

    // Iterate the explicit shape retained by DefinitionData. Private export
    // meshes a disposable shape and must not accidentally read the original
    // label again after that operation. Ordinary publication preloads the same
    // label shape, so its extraction semantics remain unchanged.
    RWMesh_FaceIterator aFace(theDefinition.shape);
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
#if DEBUG
            std::fprintf(stderr,
                "R265_WARM_SNAPSHOT predicate=face-triangulation definition=%.128s "
                "null=%d nodes=%d triangles=%d\n",
                theDefinitionIdentifier.c_str(), int(aTriangulation.IsNull()),
                aTriangulation.IsNull() ? 0 : aTriangulation->NbNodes(),
                aTriangulation.IsNull() ? 0 : aTriangulation->NbTriangles());
#endif
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
    if (hasSemanticBRepTopology) {
        aMesh.topology.faceCount = static_cast<std::uint32_t>(
            theDefinition.faces.Extent());
        aMesh.topology.edgeCount = static_cast<std::uint32_t>(
            theDefinition.edges.Extent());
        aMesh.topology.vertexCount = static_cast<std::uint32_t>(
            theDefinition.vertices.Extent());
    }
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
    theDefinition.fingerprint = DefinitionFingerprint(theDefinition);
    return true;
}

bool BuildBoundedCurveStroke(
    const core3d::bounded_curve::Definition& theSavedCurve,
    const Bounds3d& theControlBounds,
    const Double3& theSourceOrigin,
    MeshSnapshot& theMesh)
{
    using namespace core3d::bounded_curve;
    if (Validate(theSavedCurve) != Refusal::None
        || theSavedCurve.knots.size() < 2 || !IsValid(theControlBounds)) {
        return false;
    }

    // Evaluation is identical for 2D and 3D definitions. Promote only this
    // detached value because the shared evaluator deliberately admits paths.
    Definition anEvaluationCurve = theSavedCurve;
    anEvaluationCurve.domain = Domain::Path3D;
    const double aDx = theControlBounds.maximum.x - theControlBounds.minimum.x;
    const double aDy = theControlBounds.maximum.y - theControlBounds.minimum.y;
    const double aDz = theControlBounds.maximum.z - theControlBounds.minimum.z;
    const double aScale = std::sqrt(aDx * aDx + aDy * aDy + aDz * aDz);
    if (!IsFinite(aScale) || aScale <= Precision::Confusion()) return false;
    const double aChordTolerance = std::clamp(aScale * 1.0e-3, 1.0e-6, 100.0);
    const double aCosineLimit = std::cos(kMaxCurveStrokeAngleRadians);
    std::size_t anEvaluationCount = 0;
    std::size_t aSegmentCount = 0;

    struct StrokeSample {
        double parameter = 0.0;
        Vector3 point{};
        Vector3 tangent{};
    };
    const auto evaluate = [&](const double theParameter,
                              StrokeSample& theSample) {
        if (anEvaluationCount >= kMaxCurveStrokeEvaluations) return false;
        ++anEvaluationCount;
        Evaluation aValue;
        if (Evaluate(anEvaluationCurve, theParameter, aValue)
                != EvaluationRefusal::None
            || !Normalize(aValue.derivative, 1.0e-12, theSample.tangent)) {
            return false;
        }
        theSample.parameter = theParameter;
        theSample.point = aValue.point;
        return true;
    };

    std::vector<StrokeSample> aSamples;
    aSamples.reserve(kMaxCurveStrokeSegments + 1);
    StrokeSample aFirst;
    if (!evaluate(theSavedCurve.knots.front().value, aFirst)) return false;
    aSamples.push_back(aFirst);
    std::function<bool(const StrokeSample&, const StrokeSample&, unsigned)> refine;
    refine = [&](const StrokeSample& theLeft,
                 const StrokeSample& theRight,
                 const unsigned theDepth) {
        const double aMiddleParameter =
            theLeft.parameter + (theRight.parameter - theLeft.parameter) * 0.5;
        if (!(theLeft.parameter < aMiddleParameter
              && aMiddleParameter < theRight.parameter)) return false;
        StrokeSample aMiddle;
        if (!evaluate(aMiddleParameter, aMiddle)) return false;
        const Vector3 aChordMiddle = Scale(Add(theLeft.point, theRight.point), 0.5);
        const double anError = Norm(Subtract(aMiddle.point, aChordMiddle));
        const double aLeftCosine = Dot(theLeft.tangent, aMiddle.tangent);
        const double aRightCosine = Dot(aMiddle.tangent, theRight.tangent);
        if (!Finite(anError) || !Finite(aLeftCosine) || !Finite(aRightCosine)) {
            return false;
        }
        if (anError <= aChordTolerance
            && aLeftCosine >= aCosineLimit
            && aRightCosine >= aCosineLimit) {
            if (aSegmentCount >= kMaxCurveStrokeSegments) return false;
            ++aSegmentCount;
            aSamples.push_back(theRight);
            return true;
        }
        if (theDepth == 0) return false;
        return refine(theLeft, aMiddle, theDepth - 1)
            && refine(aMiddle, theRight, theDepth - 1);
    };
    for (std::size_t aKnot = 1; aKnot < theSavedCurve.knots.size(); ++aKnot) {
        StrokeSample aRight;
        if (!evaluate(theSavedCurve.knots[aKnot].value, aRight)
            || !refine(aSamples.back(), aRight, kMaxCurveStrokeDepth)) {
            return false;
        }
    }
    if (aSamples.size() < 2 || aSegmentCount + 1 != aSamples.size()) return false;

    std::size_t aVertexCount = 0, anIndexCount = 0, aNumericBytes = 0;
    if (!CheckedMultiply(aSegmentCount, 4U, aVertexCount)
        || !CheckedMultiply(aSegmentCount, 6U, anIndexCount)
        || aVertexCount > kMaxVerticesPerMesh || anIndexCount > kMaxIndicesPerMesh
        || !CheckedMultiply(aVertexCount, sizeof(Vertex), aNumericBytes)
        || aNumericBytes > kMaxMeshNumericBytes
        || anIndexCount * sizeof(std::uint32_t)
            > kMaxMeshNumericBytes - aNumericBytes) return false;
    theMesh.vertices.reserve(aVertexCount);
    theMesh.indices.reserve(anIndexCount);
    const double aHalfWidth = std::clamp(aScale * 1.5e-3, 1.0e-5, 10.0);
    Bounds3d aRenderedBounds;
    for (std::size_t aSegment = 0; aSegment < aSegmentCount; ++aSegment) {
        const Vector3 aDirectionValue =
            Subtract(aSamples[aSegment + 1].point, aSamples[aSegment].point);
        Vector3 aDirection{}, aLateral{}, aNormal{};
        if (!Normalize(aDirectionValue, Precision::Confusion(), aDirection)) return false;
        for (const Vector3* aReference : {&theSavedCurve.frame.zAxis,
                                          &theSavedCurve.frame.yAxis,
                                          &theSavedCurve.frame.xAxis}) {
            if (Normalize(Cross(*aReference, aDirection), 1.0e-12, aLateral)) break;
        }
        if (!Normalize(Cross(aDirection, aLateral), 1.0e-12, aNormal)) return false;
        const std::uint32_t aBase = static_cast<std::uint32_t>(theMesh.vertices.size());
        for (const auto& aPoint : {aSamples[aSegment].point,
                                   aSamples[aSegment + 1].point}) {
            for (const double aSign : {-1.0, 1.0}) {
                const Vector3 aPosition = Add(aPoint, Scale(aLateral, aSign * aHalfWidth));
                const double x = aPosition[0] - theSourceOrigin.x;
                const double y = aPosition[1] - theSourceOrigin.y;
                const double z = aPosition[2] - theSourceOrigin.z;
                if (!FitsFloat(x) || !FitsFloat(y) || !FitsFloat(z)
                    || !FitsFloat(aNormal[0]) || !FitsFloat(aNormal[1])
                    || !FitsFloat(aNormal[2])) return false;
                theMesh.vertices.push_back({float(x), float(y), float(z),
                    float(aNormal[0]), float(aNormal[1]), float(aNormal[2]), 0.0f, 0.0f});
                Extend(aRenderedBounds, x, y, z);
            }
        }
        for (const std::uint32_t anOffset : {0U, 2U, 1U, 1U, 2U, 3U})
            theMesh.indices.push_back(aBase + anOffset);
    }
    if (!IsValid(aRenderedBounds) || !FitsUInt32(theMesh.indices.size())) return false;
    theMesh.localBounds = aRenderedBounds;
    theMesh.primitives.push_back(
        {0, static_cast<std::uint32_t>(theMesh.indices.size()), 0, false});
    return true;
}

bool ExtractBoundedCurveDefinition(
    const Handle(OcctDocument)& theDocument,
    const TDF_Label& theDefinitionLabel,
    const std::string& theDefinitionIdentifier,
    DefinitionData& theDefinition
#ifdef DEBUG
    , DebugBoundedCurvePublicationObservation* theDebugObservation
#endif
    )
{
    std::vector<core3d::bounded_curve::Record> aRecords;
    if (theDocument.IsNull() || !core3d::bounded_curve::ReadAll(
            theDocument->Document(), aRecords)) return false;
    const auto aFound = std::find_if(aRecords.begin(), aRecords.end(),
        [&](const auto& theRecord) {
            return theRecord.owner.IsEqual(theDefinitionLabel);
        });
    if (aFound == aRecords.end() || !aFound->value) return false;
    OcctBoundedCurveCapture aCapture;
    if (!theDocument->ReadBoundedCurveExact(
            aFound->value->persisted.ownerState.owner, aCapture)
        || !aCapture.record.owner.IsEqual(theDefinitionLabel)
        || !aCapture.wire.IsSame(aFound->current)
        || !core3d::bounded_curve::WireMatchesDefinition(
            aCapture.persisted, aCapture.wire)) return false;

#ifdef DEBUG
    if (theDebugObservation != nullptr) {
        if (theDebugObservation->definitionIdentifier.empty()) {
            theDebugObservation->entityIdentifier =
                theDocument->EntityIdentifierForLabel(theDefinitionLabel);
            theDebugObservation->definitionIdentifier =
                theDefinitionIdentifier;
            theDebugObservation->canonicalDefinitionBytes =
                aCapture.record.value->definitionBytes;
            theDebugObservation->canonicalOwnerBytes =
                aCapture.record.value->ownerBytes;
            theDebugObservation->canonicalDefinitionDigest =
                aCapture.persisted.ownerState.canonicalDefinitionDigest;
            if (theDebugObservation->entityIdentifier.empty()) return false;
        }
    }
#endif

    theDefinition.faces.Clear();
    theDefinition.edges.Clear();
    theDefinition.vertices.Clear();
    TopExp::MapShapes(aCapture.wire, TopAbs_EDGE, theDefinition.edges);
    TopExp::MapShapes(aCapture.wire, TopAbs_VERTEX, theDefinition.vertices);
    if (theDefinition.edges.IsEmpty() || theDefinition.vertices.IsEmpty()
        || !FitsUInt32(theDefinition.edges.Extent())
        || !FitsUInt32(theDefinition.vertices.Extent())) return false;
#ifdef DEBUG
    if (theDebugObservation != nullptr
        && theDebugObservation->definitionIdentifier
            == theDefinitionIdentifier) {
        theDebugObservation->edgeCount =
            static_cast<std::uint32_t>(theDefinition.edges.Extent());
        theDebugObservation->vertexCount =
            static_cast<std::uint32_t>(theDefinition.vertices.Extent());
    }
#endif

    Bounds3d aBounds;
    const auto& aFrame = aCapture.persisted.value.definition.frame;
    for (const auto& aPole : aCapture.persisted.value.definition.controlPoints) {
        const auto& p = aPole.local;
        Extend(aBounds,
            aFrame.origin[0] + aFrame.xAxis[0] * p[0] + aFrame.yAxis[0] * p[1] + aFrame.zAxis[0] * p[2],
            aFrame.origin[1] + aFrame.xAxis[1] * p[0] + aFrame.yAxis[1] * p[1] + aFrame.zAxis[1] * p[2],
            aFrame.origin[2] + aFrame.xAxis[2] * p[0] + aFrame.yAxis[2] * p[1] + aFrame.zAxis[2] * p[2]);
    }
    if (!IsValid(aBounds)) return false;
    theDefinition.sourceOrigin = Center(aBounds);
    MeshSnapshot aMesh;
    aMesh.definitionIdentifier = theDefinitionIdentifier;
    aMesh.topology.edgeCount = theDefinition.edges.Extent();
    aMesh.topology.vertexCount = theDefinition.vertices.Extent();
    if (!BuildBoundedCurveStroke(aCapture.persisted.value.definition,
                                 aBounds,
                                 theDefinition.sourceOrigin,
                                 aMesh)) return false;
    theDefinition.mesh = std::move(aMesh);
    theDefinition.boundedCurve = true;
    theDefinition.fingerprint = DefinitionFingerprint(theDefinition);
    return true;
}

// Publish a supplied local basis only after proving that the ordinary mesh
// extraction preserved every node attribute and every triangle corner. The
// definition's outer placement remains in worldFromObject, not in these frames.
bool PublishAuthoredFrames(const DefinitionData& definition,
                           const OcctAuthoredFrameRecord& record,
                           MeshSnapshot& mesh)
{
    if (record.archive.empty() || record.cornerCount != mesh.indices.size()
        || definition.representation != OcctGeometryRepresentation::TriangleMesh
        || mesh.primitives.size() != 1 || mesh.primitives.front().firstIndex != 0
        || mesh.primitives.front().indexCount != mesh.indices.size()
        || mesh.primitives.front().faceIndex != 0
        || !mesh.primitives.front().hasTextureCoordinates) return false;
    RWMesh_FaceIterator face(definition.label, TopLoc_Location(), Standard_False);
    if (!face.More() || !face.HasNormals() || !face.HasTexCoords()
        || face.NbNodes() != mesh.vertices.size()
        || std::size_t(face.NbTriangles()) * 3 != mesh.indices.size()) return false;
    for (int node = face.NodeLower(); node <= face.NodeUpper(); ++node) {
        const auto point = face.NodeTransformed(node);
        const auto normal = face.NormalTransformed(node);
        const auto uv = face.NodeTexCoord(node);
        const Vertex expected = {
            float(point.X() - definition.sourceOrigin.x),
            float(point.Y() - definition.sourceOrigin.y),
            float(point.Z() - definition.sourceOrigin.z),
            float(normal.X()), float(normal.Y()), float(normal.Z()),
            float(uv.X()), float(uv.Y())};
        if (std::memcmp(&expected, &mesh.vertices[std::size_t(node - 1)], sizeof(Vertex)) != 0) return false;
    }
    for (int triangle = face.ElemLower(); triangle <= face.ElemUpper(); ++triangle) {
        int nodes[3]; face.TriangleOriented(triangle).Get(nodes[0], nodes[1], nodes[2]);
        for (int corner = 0; corner < 3; ++corner)
            if (nodes[corner] < 1 || mesh.indices[std::size_t(triangle - 1) * 3 + corner]
                != std::uint32_t(nodes[corner] - 1)) return false;
    }
    std::vector<Float4> frames;
    if (!core3d::persistence::DecodeNativeAuthoredFrames(face.Face(),
            record.archive.data(), record.archive.size(), frames)) return false;
    face.Next(); if (face.More()) return false;
    mesh.cornerTangents = std::move(frames);
    mesh.tangentBasis = TangentBasis::Authored;
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
        aHash.AddBool(anInstance.referenceAxis.has_value());
        if (anInstance.referenceAxis.has_value()) {
            const ReferenceAxisSnapshot& anAxis = *anInstance.referenceAxis;
            aHash.AddDouble(anAxis.worldPivot.x);
            aHash.AddDouble(anAxis.worldPivot.y);
            aHash.AddDouble(anAxis.worldPivot.z);
            aHash.AddDouble(anAxis.worldDirection.x);
            aHash.AddDouble(anAxis.worldDirection.y);
            aHash.AddDouble(anAxis.worldDirection.z);
            aHash.AddInteger(static_cast<std::uint8_t>(anAxis.pivotSpace));
            aHash.AddInteger(static_cast<std::uint8_t>(anAxis.directionSpace));
            aHash.AddBool(anAxis.authored);
        }
        aHash.AddString(anInstance.name);
        aHash.AddString(anInstance.groupIdentifier);
        aHash.AddString(anInstance.groupName);
        aHash.AddBool(anInstance.nativeWirePresentation.has_value());
        if (anInstance.nativeWirePresentation.has_value()) {
            AddMaterialValues(aHash, *anInstance.nativeWirePresentation);
        }
    }
    return aHash.Value();
}

std::uint64_t PresentationFingerprint(const SceneSnapshot& theScene)
{
    Fingerprint aHash;
    aHash.AddInteger(static_cast<std::uint8_t>(theScene.selectionMode));
    // Frame edits change appearance even without a normal-map binding. Keep
    // the geometry revision stable while invalidating presentation consumers.
    aHash.AddInteger<std::uint64_t>(theScene.meshes.size());
    for (const auto& mesh : theScene.meshes) {
        aHash.AddInteger(static_cast<std::uint8_t>(mesh.tangentBasis));
        aHash.AddInteger<std::uint64_t>(mesh.cornerTangents.size());
        for (const auto& frame : mesh.cornerTangents) {
            aHash.AddFloat(frame.x); aHash.AddFloat(frame.y);
            aHash.AddFloat(frame.z); aHash.AddFloat(frame.w);
        }
    }
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
        aHash.AddInteger<std::uint64_t>(anInstance.faceImageBindings.size());
        for (const FaceImageBindingSnapshot& aFaceBinding :
             anInstance.faceImageBindings) {
            aHash.AddString(aFaceBinding.bindingIdentifier);
            aHash.AddString(aFaceBinding.faceIdentifier);
            aHash.AddString(aFaceBinding.resourceIdentifier);
            aHash.AddInteger(static_cast<std::uint8_t>(aFaceBinding.role));
            aHash.AddBool(aFaceBinding.srgbColorSpace);
            aHash.AddDouble(aFaceBinding.transform.scaleU);
            aHash.AddDouble(aFaceBinding.transform.scaleV);
            aHash.AddDouble(aFaceBinding.transform.offsetU);
            aHash.AddDouble(aFaceBinding.transform.offsetV);
            aHash.AddDouble(aFaceBinding.transform.rotationDegrees);
            aHash.AddInteger(
                static_cast<std::uint8_t>(aFaceBinding.transform.wrapU));
            aHash.AddInteger(
                static_cast<std::uint8_t>(aFaceBinding.transform.wrapV));
            aHash.AddInteger(aFaceBinding.faceIndex);
            aHash.AddInteger(aFaceBinding.textureIndex);
        }
        aHash.AddInteger<std::uint64_t>(
            anInstance.decalDerivedAppearances.size());
        for (const DecalDerivedAppearanceSnapshot& anAppearance :
             anInstance.decalDerivedAppearances) {
            aHash.AddString(anAppearance.ownerDefinitionIdentifier);
            aHash.AddString(anAppearance.bakeProof);
            aHash.AddInteger(anAppearance.faceIndex);
            aHash.AddInteger(anAppearance.baseColorTextureIndex);
        }
    }
    aHash.AddInteger<std::uint64_t>(theScene.pickTable.size());
    for (const ElementIdentifier& anElement : theScene.pickTable) {
        aHash.AddString(anElement.entityIdentifier);
        aHash.AddInteger(static_cast<std::uint8_t>(anElement.kind));
        aHash.AddInteger(anElement.topologyIndex);
        aHash.AddInteger(anElement.geometryRevision);
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
        case PresentationOverlayKind::RadialArrayPreview:
            if (!isEmpty
                && (theMeshes.size() != 1 || theMaterials.size() != 1
                    || theInstances.empty()
                    || theInstances.size()
                        > kMaxRadialArrayPreviewBodies)) {
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
    const bool isRadialArrayPreview =
        theKind == PresentationOverlayKind::RadialArrayPreview;
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
        const bool isRadialArrayMaterial = isRadialArrayPreview;
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
        } else if (isRadialArrayMaterial) {
            hasExpectedIdentifier = aMaterialIndex == 0
                && aMaterial.identifier
                    == "radial-array/source/0/material";
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
                    || isRadialArrayMaterial
                    || isShellMaterial)
                && (aMaterial.baseColorTextureIndex != -1
                    || aMaterial.emissiveTextureIndex != -1
            || aMaterial.metallicRoughnessTextureIndex != -1
            || aMaterial.normalTextureIndex != -1
            || aMaterial.occlusionTextureIndex != -1))
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
        const bool isRadialArrayMesh = isRadialArrayPreview;
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
            || (isRadialArrayMesh
                && (aMeshIndex != 0
                    || aMesh.definitionIdentifier
                        != "radial-array/source/0/mesh"))
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
                    || isLinearArrayMesh || isRadialArrayMesh || isShellMesh)
                && !IsCenteredLocalBounds(aMesh.localBounds))
            || aMesh.vertices.empty() || aMesh.indices.empty()
            || aMesh.primitives.empty()
            || (!isMirrorPreview && !isBooleanMesh && !isChamferMesh
                    && !isLinearArrayMesh && !isRadialArrayMesh && !isShellMesh
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
        const bool isRadialArrayItem = isRadialArrayPreview;
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
                    : isRadialArrayItem
                        ? anInstance.entityIdentifier
                                == "radial-array/preview/0/"
                                    + std::to_string(anInstanceIndex + 1U)
                            && anInstance.name
                                == "Radial array preview "
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
                    || isRadialArrayItem
                    || isShellItem)
                ? anInstance.role == (isChamferItem
                        ? RenderRole::ChamferPreview
                        : isLinearArrayItem
                            ? RenderRole::LinearArrayPreview
                            : isRadialArrayItem
                                ? RenderRole::RadialArrayPreview
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
                || isLinearArrayItem || isRadialArrayItem || isShellItem)
            ? theMeshes[(isLinearArrayItem || isRadialArrayItem)
                    ? 0 : anInstanceIndex]
                .primitives.size()
            : 1;
        if (!hasExpectedIdentity
            || !IsValidIdentifier(anInstance.entityIdentifier)
            || !anEntityIdentifiers.insert(
                anInstance.entityIdentifier).second
            || !anInstance.groupIdentifier.empty() || !anInstance.groupName.empty()
            || anInstance.name.size() > 4'096
            || anInstance.meshIndex >= theMeshes.size()
            || anInstance.reversesWinding || !anInstance.visible
            || anInstance.selectable || anInstance.selected
			|| anInstance.referenceAxis.has_value()
            || !hasExpectedSemantics
            || (!isBooleanItem && !isChamferItem && !isShellItem
                && anInstance.renderStyle != RenderStyle::Shaded)
            || ((isChamferItem || isShellItem)
                && anInstance.renderStyle != RenderStyle::Shaded)
            || ((isMirrorPreview || isBooleanItem || isChamferItem
                    || isLinearArrayItem || isRadialArrayItem || isShellItem)
                ? (isRadialArrayItem
                    ? !IsProperRigidBoundedWorldTransform(
                        anInstance.worldFromObject)
                    : !IsTranslationOnlyWorldTransform(
                        anInstance.worldFromObject))
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
            && !isLinearArrayItem && !isRadialArrayItem) {
            return false;
        }
        for (const PrimitiveBinding& aBinding :
             anInstance.primitiveBindings) {
            if (((isMirrorPlane || isMirrorPreview
                    || isBooleanItem || isChamferItem
                    || isLinearArrayItem || isRadialArrayItem || isShellItem)
                    && aBinding.materialIndex
                        != ((isLinearArrayItem || isRadialArrayItem)
                            ? 0 : anInstanceIndex))
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
    if (isLinearArrayPreview || isRadialArrayPreview) {
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
		aHash.AddBool(anInstance.referenceAxis.has_value());
		if (anInstance.referenceAxis.has_value()) {
			const ReferenceAxisSnapshot& anAxis =
				*anInstance.referenceAxis;
			aHash.AddDouble(anAxis.worldPivot.x);
			aHash.AddDouble(anAxis.worldPivot.y);
			aHash.AddDouble(anAxis.worldPivot.z);
			aHash.AddDouble(anAxis.worldDirection.x);
			aHash.AddDouble(anAxis.worldDirection.y);
			aHash.AddDouble(anAxis.worldDirection.z);
			aHash.AddInteger(
				static_cast<std::uint8_t>(anAxis.pivotSpace));
			aHash.AddInteger(
				static_cast<std::uint8_t>(anAxis.directionSpace));
			aHash.AddBool(anAxis.authored);
		}
        aHash.AddString(anInstance.name);
        aHash.AddString(anInstance.groupIdentifier);
        aHash.AddString(anInstance.groupName);
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

//! Concrete allocation/work ledger for one builder-owned E4 operation. It is
//! embedded in PrivateDecalCapture::Impl, so constructing the operation has a
//! fixed-size, allocation-free accounting owner before any protected scene
//! copy. Shared headers expose only accounting::View and move-only Ticket.
class OperationLedger final {
public:
    using FailureSite = decal_layer::bake::accounting::FailureSite;
    using Job = decal_layer::bake::accounting::Job;
    using Retention = decal_layer::bake::accounting::Retention;
    using StorageDimension =
        decal_layer::bake::accounting::StorageDimension;
    using View = decal_layer::bake::accounting::View;

    OperationLedger() noexcept
        : storageLimits_({kMaxSnapshotNumericBytes,
                          kMaxAggregateEncodedTextureBytes,
                          kMaxAggregateDecodedTextureBytes}) {}

#ifdef DEBUG
    OperationLedger(std::size_t snapshotLimit, std::size_t encodedLimit,
                    std::size_t privateLimit) noexcept
        : storageLimits_({snapshotLimit, encodedLimit, privateLimit}) {}
#endif

    View view() noexcept {
        return {this, &ReserveBridge, &ReleaseBridge,
            &CheckedProductBridge, &SealJobBridge, &ConsumeJobBridge,
            &RecheckBridge, &CancelBridge, &EnterAllocationBridge,
            &AllocationFailureBridge};
    }

    bool bindTopology(retained_edge_treatment::ReplayBudget& topology,
                      FailureSite site) noexcept {
        if (failed_) return false;
        if (topology_ != nullptr || !topology.valid())
            return fail(site, FailureReason::Topology, 0, 0);
        topology_ = &topology;
        return true;
    }

    bool reserve(StorageDimension dimension, std::size_t bytes,
                 Retention retention, FailureSite site) noexcept {
        if (failed_) return false;
        const std::size_t index = storageIndex(dimension);
        if (index >= storage_.size())
            return fail(site, FailureReason::InvalidDimension, bytes, 0);
        StorageState& state = storage_[index];
        const std::size_t limit = storageLimits_[index];
        if (state.live > limit || bytes > limit - state.live)
            return fail(site, FailureReason::StorageLimit, bytes,
                        state.live <= limit ? limit - state.live : 0);
        if (bytes > std::numeric_limits<std::size_t>::max() - state.reserved
            || (retention == Retention::Retained
                && bytes > std::numeric_limits<std::size_t>::max()
                    - state.retained))
            return fail(site, FailureReason::ArithmeticOverflow, bytes, 0);
        state.reserved += bytes;
        state.live += bytes;
        if (retention == Retention::Retained) state.retained += bytes;
        state.peak = std::max(state.peak, state.live);
        return true;
    }

    void release(StorageDimension dimension, std::size_t bytes,
                 Retention retention) noexcept {
        const std::size_t index = storageIndex(dimension);
        if (index >= storage_.size()) {
            (void)fail(FailureSite::StorageRelease,
                       FailureReason::InvalidDimension, bytes, 0);
            return;
        }
        StorageState& state = storage_[index];
        if (bytes > state.live
            || (retention == Retention::Retained
                && bytes > state.retained)) {
            (void)fail(FailureSite::StorageRelease,
                       FailureReason::ReleaseMismatch, bytes, state.live);
            return;
        }
        state.live -= bytes;
        if (retention == Retention::Retained) state.retained -= bytes;
    }

    bool checkedProduct(std::size_t first, std::size_t second,
                        std::size_t third, std::size_t& product,
                        FailureSite site) noexcept {
        product = 0;
        if (failed_) return false;
        if ((first != 0
                && second > std::numeric_limits<std::size_t>::max() / first)
            || (first * second != 0
                && third > std::numeric_limits<std::size_t>::max()
                    / (first * second)))
            return fail(site, FailureReason::ArithmeticOverflow,
                        std::numeric_limits<std::size_t>::max(), 0);
        product = first * second * third;
        return true;
    }

    bool sealJob(Job job, std::size_t admittedUnits,
                 FailureSite site) noexcept {
        if (failed_) return false;
        const std::size_t index = jobIndex(job);
        if (index >= jobs_.size())
            return fail(site, FailureReason::InvalidJob, admittedUnits, 0);
        JobState& state = jobs_[index];
        if (state.sealed)
            return fail(site, FailureReason::DuplicateJob,
                        admittedUnits, state.admitted);
        if (admittedUnits > std::numeric_limits<std::size_t>::max()
                - cumulativeAdmittedWork_)
            return fail(site, FailureReason::ArithmeticOverflow,
                        admittedUnits, 0);
        state.sealed = true;
        state.admitted = admittedUnits;
        cumulativeAdmittedWork_ += admittedUnits;
        return true;
    }

    bool consumeJob(Job job, std::size_t units,
                    FailureSite site) noexcept {
        if (failed_) return false;
        const std::size_t index = jobIndex(job);
        if (index >= jobs_.size())
            return fail(site, FailureReason::InvalidJob, units, 0);
        JobState& state = jobs_[index];
        if (!state.sealed)
            return fail(site, FailureReason::UnsealedJob, units, 0);
        if (state.consumed > state.admitted
            || units > state.admitted - state.consumed)
            return fail(site, FailureReason::JobLimit, units,
                        state.consumed <= state.admitted
                            ? state.admitted - state.consumed : 0);
        if (units > std::numeric_limits<std::size_t>::max()
                - cumulativeWork_)
            return fail(site, FailureReason::ArithmeticOverflow, units, 0);
        state.consumed += units;
        cumulativeWork_ += units;
        return true;
    }

    bool recheck(FailureSite site) noexcept {
        if (failed_) return false;
        if (topology_ != nullptr && !topology_->valid())
            return fail(site, FailureReason::Topology,
                        topology_->topologyVisits,
                        retained_topology_budget::MaximumTopologyVisits);
        for (std::size_t index = 0; index < storage_.size(); ++index) {
            const StorageState& state = storage_[index];
            if (state.live > storageLimits_[index]
                || state.retained > state.live)
                return fail(site, FailureReason::Invariant,
                            state.live, storageLimits_[index]);
        }
        for (const JobState& state : jobs_)
            if (state.consumed > state.admitted)
                return fail(site, FailureReason::Invariant,
                            state.consumed, state.admitted);
        if (cumulativeWork_ > cumulativeAdmittedWork_)
            return fail(site, FailureReason::Invariant,
                        cumulativeWork_, cumulativeAdmittedWork_);
        return true;
    }

    void cancel(FailureSite site) noexcept {
        if (!failed_)
            (void)fail(site, FailureReason::Cancelled, 0, 0);
    }

    bool enterAllocation(FailureSite site) noexcept {
        if (failed_) return false;
        const std::size_t index = static_cast<std::size_t>(site);
        if (index >= allocationEntries_.size())
            return fail(site, FailureReason::Invariant, index,
                allocationEntries_.size());
        ++allocationEntries_[index];
        return true;
    }

    void allocationFailure(FailureSite site) noexcept {
        if (!failed_)
            (void)fail(site, FailureReason::AllocationFailure, 0, 0);
    }

    std::size_t live(StorageDimension dimension) const noexcept {
        const std::size_t index = storageIndex(dimension);
        return index < storage_.size() ? storage_[index].live : 0;
    }
    std::size_t retained(StorageDimension dimension) const noexcept {
        const std::size_t index = storageIndex(dimension);
        return index < storage_.size() ? storage_[index].retained : 0;
    }
    std::size_t peak(StorageDimension dimension) const noexcept {
        const std::size_t index = storageIndex(dimension);
        return index < storage_.size() ? storage_[index].peak : 0;
    }
    std::size_t cumulativeWork() const noexcept { return cumulativeWork_; }
    FailureSite firstFailureSite() const noexcept { return firstFailure_.site; }
    std::size_t allocationEntries(FailureSite site) const noexcept {
        const std::size_t index = static_cast<std::size_t>(site);
        return index < allocationEntries_.size()
            ? allocationEntries_[index] : 0;
    }

private:
    enum class FailureReason : std::uint8_t {
        None,
        InvalidDimension,
        StorageLimit,
        ReleaseMismatch,
        ArithmeticOverflow,
        InvalidJob,
        DuplicateJob,
        UnsealedJob,
        JobLimit,
        Topology,
        Cancelled,
        AllocationFailure,
        Invariant
    };
    struct StorageState final {
        std::size_t reserved = 0;
        std::size_t live = 0;
        std::size_t retained = 0;
        std::size_t peak = 0;
    };
    struct JobState final {
        std::size_t admitted = 0;
        std::size_t consumed = 0;
        bool sealed = false;
    };
    struct FailureRecord final {
        FailureSite site = FailureSite::None;
        FailureReason reason = FailureReason::None;
        std::size_t requested = 0;
        std::size_t admitted = 0;
    };

    static constexpr std::size_t storageIndex(
        StorageDimension dimension) noexcept {
        return static_cast<std::size_t>(dimension);
    }
    static constexpr std::size_t jobIndex(Job job) noexcept {
        return static_cast<std::size_t>(job);
    }
    bool fail(FailureSite site, FailureReason reason,
              std::size_t requested, std::size_t admitted) noexcept {
        if (!failed_) {
            failed_ = true;
            firstFailure_ = {site, reason, requested, admitted};
        }
        return false;
    }

    static bool ReserveBridge(void* owner, StorageDimension dimension,
        std::size_t bytes, Retention retention, FailureSite site) noexcept {
        return static_cast<OperationLedger*>(owner)->reserve(
            dimension, bytes, retention, site);
    }
    static void ReleaseBridge(void* owner, StorageDimension dimension,
        std::size_t bytes, Retention retention) noexcept {
        static_cast<OperationLedger*>(owner)->release(
            dimension, bytes, retention);
    }
    static bool CheckedProductBridge(void* owner, std::size_t first,
        std::size_t second, std::size_t third, std::size_t& product,
        FailureSite site) noexcept {
        return static_cast<OperationLedger*>(owner)->checkedProduct(
            first, second, third, product, site);
    }
    static bool SealJobBridge(void* owner, Job job,
        std::size_t admittedUnits, FailureSite site) noexcept {
        return static_cast<OperationLedger*>(owner)->sealJob(
            job, admittedUnits, site);
    }
    static bool ConsumeJobBridge(void* owner, Job job,
        std::size_t units, FailureSite site) noexcept {
        return static_cast<OperationLedger*>(owner)->consumeJob(
            job, units, site);
    }
    static bool RecheckBridge(void* owner, FailureSite site) noexcept {
        return static_cast<OperationLedger*>(owner)->recheck(site);
    }
    static void CancelBridge(void* owner, FailureSite site) noexcept {
        static_cast<OperationLedger*>(owner)->cancel(site);
    }
    static bool EnterAllocationBridge(void* owner, FailureSite site) noexcept {
        return static_cast<OperationLedger*>(owner)->enterAllocation(site);
    }
    static void AllocationFailureBridge(void* owner,
                                        FailureSite site) noexcept {
        static_cast<OperationLedger*>(owner)->allocationFailure(site);
    }

    std::array<std::size_t, static_cast<std::size_t>(
        StorageDimension::Count)> storageLimits_{};
    std::array<StorageState, static_cast<std::size_t>(
        StorageDimension::Count)> storage_{};
    std::array<JobState, static_cast<std::size_t>(Job::Count)> jobs_{};
    std::array<std::size_t, static_cast<std::size_t>(
        FailureSite::Count)> allocationEntries_{};
    FailureRecord firstFailure_{};
    retained_edge_treatment::ReplayBudget* topology_ = nullptr;
    std::size_t cumulativeAdmittedWork_ = 0;
    std::size_t cumulativeWork_ = 0;
    bool failed_ = false;
};

} // namespace core3d::scene

namespace core3d::decal_layer::bake::accounting {
namespace {
struct OwnedOperationLedger final {
    std::atomic_size_t references{1};
    core3d::scene::OperationLedger ledger;

    explicit OwnedOperationLedger(
        core3d::scene::OperationLedger&& admitted) noexcept
        : ledger(std::move(admitted)) {}
};

void RetainOperation(void* raw) noexcept
{
    static_cast<OwnedOperationLedger*>(raw)->references.fetch_add(
        1, std::memory_order_relaxed);
}

void ReleaseOperation(void* raw) noexcept
{
    auto* owner = static_cast<OwnedOperationLedger*>(raw);
    if (owner->references.fetch_sub(1, std::memory_order_acq_rel) == 1)
        delete owner;
}
} // namespace

Owner MakeOperationOwner() noexcept
{
    core3d::scene::OperationLedger admission;
    if (!admission.reserve(StorageDimension::PrivateStorage,
            sizeof(OwnedOperationLedger), Retention::Retained,
            FailureSite::ContextControl)
        || !admission.enterAllocation(FailureSite::ContextControl)) return {};
    auto* concrete = new (std::nothrow) OwnedOperationLedger(
        std::move(admission));
    if (!concrete) return {};
    Owner result;
    result.lifetime = concrete;
    result.view = concrete->ledger.view();
    result.retain = &RetainOperation;
    result.release = &ReleaseOperation;
    return result;
}
} // namespace core3d::decal_layer::bake::accounting

namespace core3d::scene {

struct SceneCopyStorage final {
    std::size_t numeric = 0;
    std::size_t encoded = 0;
    std::size_t privateBytes = 0;
};

bool DeepSceneStorageBytes(const SceneSnapshot& scene,
                           SceneCopyStorage& total) noexcept
{
    total = {};
    const auto add = [](std::size_t& destination, std::size_t bytes) {
        if (bytes > std::numeric_limits<std::size_t>::max() - destination)
            return false;
        destination += bytes;
        return true;
    };
    const auto string = [&](const std::string& value) {
        if (value.capacity() == 0) return true;
        const std::uintptr_t data = reinterpret_cast<std::uintptr_t>(
            value.data());
        const std::uintptr_t first = reinterpret_cast<std::uintptr_t>(&value);
        if (data >= first && data < first + sizeof(value)) return true;
        return value.capacity() < std::numeric_limits<std::size_t>::max()
            && add(total.privateBytes, value.capacity() + 1);
    };
    const auto vector = [&](std::size_t& destination, std::size_t capacity,
                            std::size_t element) {
        return capacity == 0 || element <=
                std::numeric_limits<std::size_t>::max() / capacity
            ? add(destination, capacity * element) : false;
    };
    if (!string(scene.publicationSourceIdentifier)
        || !vector(total.numeric, scene.meshes.capacity(), sizeof(MeshSnapshot))
        || !vector(total.numeric, scene.instances.capacity(), sizeof(InstanceSnapshot))
        || !vector(total.numeric, scene.materials.capacity(), sizeof(MaterialSnapshot))
        || !vector(total.numeric, scene.textures.capacity(), sizeof(TextureResourceSnapshot))
        || !vector(total.numeric, scene.pickTable.capacity(), sizeof(ElementIdentifier))
        || !vector(total.numeric, scene.selection.selected.capacity(),
            sizeof(ElementIdentifier))) return false;
    for (const auto& mesh : scene.meshes) {
        if (!string(mesh.definitionIdentifier)
            || !string(mesh.paintedAtlasMasterDefinitionIdentifier)
            || !string(mesh.paintedAtlasBakeProof)
            || !vector(total.numeric, mesh.vertices.capacity(), sizeof(Vertex))
            || !vector(total.numeric, mesh.indices.capacity(), sizeof(std::uint32_t))
            || !vector(total.numeric, mesh.primitives.capacity(), sizeof(MeshPrimitive))
            || !vector(total.numeric, mesh.cornerTangents.capacity(), sizeof(Float4)))
            return false;
        if (mesh.nativeC1Wire
            && (!vector(total.privateBytes,
                    mesh.nativeC1Wire->canonicalDefinitionBytes.capacity(), 1)
                || !vector(total.privateBytes,
                    mesh.nativeC1Wire->canonicalOwnerBytes.capacity(), 1)))
            return false;
    }
    for (const auto& instance : scene.instances) {
        if (!string(instance.entityIdentifier) || !string(instance.name)
            || !string(instance.groupIdentifier) || !string(instance.groupName)
            || !vector(total.numeric, instance.primitiveBindings.capacity(),
                sizeof(PrimitiveBinding))
            || !vector(total.numeric, instance.faceImageBindings.capacity(),
                sizeof(FaceImageBindingSnapshot))
            || !vector(total.numeric, instance.decalDerivedAppearances.capacity(),
                sizeof(DecalDerivedAppearanceSnapshot))) return false;
        if (instance.nativeWirePresentation
            && !string(instance.nativeWirePresentation->identifier)) return false;
        for (const auto& binding : instance.faceImageBindings)
            if (!string(binding.bindingIdentifier)
                || !string(binding.faceIdentifier)
                || !string(binding.resourceIdentifier)) return false;
        for (const auto& appearance : instance.decalDerivedAppearances)
            if (!string(appearance.ownerDefinitionIdentifier)
                || !string(appearance.bakeProof)) return false;
    }
    for (const auto& material : scene.materials)
        if (!string(material.identifier)) return false;
    for (const auto& texture : scene.textures)
        if (!string(texture.identifier)
            || !vector(total.encoded, texture.encodedBytes.capacity(), 1))
            return false;
    for (const auto& value : scene.pickTable)
        if (!string(value.entityIdentifier)) return false;
    for (const auto& value : scene.selection.selected)
        if (!string(value.entityIdentifier)) return false;
    if (scene.selection.hovered
        && !string(scene.selection.hovered->entityIdentifier)) return false;
    return true;
}

struct PrivateDecalCapture::Impl final {
    struct Owner final {
        struct Receiver final {
            decal_layer::ReceiverReceipt receipt;
            TopoDS_Face originalFace;
            TopoDS_Face disposableFace;
            std::uint32_t originalFaceIndex = 0;
            std::uint32_t disposableFaceIndex = 0;
        };
        retained_recipe::OwnerKey owner;
        decal_layer::Definition definition;
        std::vector<std::uint8_t> canonicalBytes;
        decal_layer::source::Witness source;
        TopoDS_Shape detachedAuthorityShape;
        std::unique_ptr<BRepBuilderAPI_Copy> copier;
        std::vector<Receiver> receivers;
        asset_atlas::build::FinalMemberInput savedMember;
        asset_atlas::Key transientAtlas;
        retained_finishing::Definition transientFinishing;
        TDF_Label ownerLabel;
        std::vector<painted_atlas_bake::owner::CapturedSource> paintedSources;
        std::string definitionIdentifier;
    };

    // Impl is the single operation context. The ledger is constructed first;
    // the existing ReplayBudget below remains its unchanged topology component.
    OperationLedger operationLedger;
    decal_layer::bake::accounting::Ticket wholeSceneNumericTicket;
    decal_layer::bake::accounting::Ticket wholeSceneEncodedTicket;
    decal_layer::bake::accounting::Ticket wholeScenePrivateTicket;
    SceneSnapshot wholeCommittedScene;
    std::vector<Owner> owners;
    std::shared_ptr<retained_edge_treatment::ReplayBudget> budget;
    decal_layer::bake::publication::PrivateDestination destination;
    std::string documentIdentifier;
    double metersPerUnit = 0.0;
    bool selectedObjectsOnly = false;
    Handle(OcctDocument) privateDocument;
};

const char* PrivatePaintedRoleText(const face_image::Role role) noexcept {
    switch (role) {
        case face_image::Role::BaseColor: return "baseColor";
        case face_image::Role::Emissive: return "emissive";
        case face_image::Role::MetallicRoughness: return "metallicRoughness";
        case face_image::Role::Occlusion: return "occlusion";
        case face_image::Role::Normal: return "normal";
    }
    return "unknown";
}

PrivateDecalCapture::PrivateDecalCapture() noexcept
    : impl_(new (std::nothrow) Impl()) {}

PrivateDecalCapture::~PrivateDecalCapture() = default;
PrivateDecalCapture::PrivateDecalCapture(PrivateDecalCapture&&) noexcept = default;
PrivateDecalCapture& PrivateDecalCapture::operator=(
    PrivateDecalCapture&&) noexcept = default;

bool OcctSceneSnapshotBuilder::CapturePrivateExportDecals(
    const Handle(OcctDocument)& privateDocument,
    const SceneSnapshot& wholeCommittedScene,
    bool selectedObjectsOnly,
    const std::function<bool()>& cancelled,
    PrivateDecalCapturePointer& capture) noexcept {
    capture.reset();
    if (privateDocument.IsNull() || !cancelled || cancelled()
        || !IsValidSceneSnapshot(wholeCommittedScene)) return false;
    try {
        const Handle(TDocStd_Document)& document = privateDocument->Document();
        if (document.IsNull() || document->HasOpenCommand()) return false;
        auto candidate = std::make_unique<PrivateDecalCapture>();
        if (!candidate || !candidate->impl_) return false;
        auto& impl = *candidate->impl_;
        if (!impl.operationLedger.recheck(
                decal_layer::bake::accounting::FailureSite::ContextControl))
            return false;
        impl.privateDocument = privateDocument;
        SceneCopyStorage wholeSceneBytes;
        if (!DeepSceneStorageBytes(wholeCommittedScene, wholeSceneBytes)
            || !impl.wholeSceneNumericTicket.acquire(impl.operationLedger.view(),
                decal_layer::bake::accounting::StorageDimension::SnapshotNumeric,
                wholeSceneBytes.numeric,
                decal_layer::bake::accounting::Retention::Retained,
                decal_layer::bake::accounting::FailureSite::ProtectedSceneCopy)
            || !impl.wholeSceneEncodedTicket.acquire(impl.operationLedger.view(),
                decal_layer::bake::accounting::StorageDimension::EncodedTexture,
                wholeSceneBytes.encoded,
                decal_layer::bake::accounting::Retention::Retained,
                decal_layer::bake::accounting::FailureSite::ProtectedSceneCopy)
            || !impl.wholeScenePrivateTicket.acquire(impl.operationLedger.view(),
                decal_layer::bake::accounting::StorageDimension::PrivateStorage,
                wholeSceneBytes.privateBytes,
                decal_layer::bake::accounting::Retention::Retained,
                decal_layer::bake::accounting::FailureSite::ProtectedSceneCopy)
            || !impl.operationLedger.enterAllocation(
                decal_layer::bake::accounting::FailureSite::ProtectedSceneCopy))
            return false;
        impl.wholeCommittedScene = wholeCommittedScene;
        impl.selectedObjectsOnly = selectedObjectsOnly;
        impl.documentIdentifier = privateDocument->DocumentIdentifier();
        XCAFDoc_DocumentTool::GetLengthUnit(document, impl.metersPerUnit);
        if (impl.metersPerUnit != wholeCommittedScene.metersPerUnit)
            return false;
        impl.budget = std::make_shared<
            retained_edge_treatment::ReplayBudget>();
        if (!impl.budget
            || !impl.operationLedger.bindTopology(
                *impl.budget,
                decal_layer::bake::accounting::FailureSite::ContextControl))
            return false;

        // P1 admits only proven standalone owners. A valid saved atlas is a
        // transitive multi-member authority and remains a deliberate refusal
        // until P2 can rebuild its complete membership in one transaction.
        std::vector<asset_atlas::persistence::Record> savedAtlases;
        if (!asset_atlas::persistence::ReadAll(document, savedAtlases)
            || !savedAtlases.empty()) return false;

        std::unordered_map<std::string, const InstanceSnapshot*> occurrences;
        for (const auto& instance : wholeCommittedScene.instances) {
            if (instance.role == RenderRole::Model
                && !occurrences.emplace(
                        instance.entityIdentifier, &instance).second)
                return false;
        }
        std::unordered_set<std::string> capturedDefinitions;
        XCAFPrs_DocumentExplorer explorer(document,
            XCAFPrs_DocumentExplorerFlags_OnlyLeafNodes, XCAFPrs_Style());
        for (; explorer.More(); explorer.Next()) {
            if (cancelled()) return false;
            const auto& node = explorer.Current();
            if (explorer.CurrentDepth() < 0
                || explorer.CurrentDepth() >= kMaxOccurrenceDepth)
                return false;
            std::vector<std::string> path;
            for (Standard_Integer depth = 0;
                 depth <= explorer.CurrentDepth(); ++depth) {
                path.push_back(privateDocument->EntityIdentifierForLabel(
                    explorer.Current(depth).Label));
            }
            const std::string entity = DeriveOccurrenceIdentifier(
                impl.documentIdentifier, path);
            if (occurrences.find(entity) == occurrences.end()) continue;
            const TDF_Label label = node.RefLabel.IsNull()
                ? node.Label : node.RefLabel;
            const std::string definitionIdentifier =
                privateDocument->DefinitionIdentifierForLabel(label);
            if (definitionIdentifier.empty()) return false;
            decal_layer::Definition definition;
            std::vector<std::uint8_t> canonicalBytes;
            const auto state = decal_layer::persistence::Read(
                document, label, definition, &canonicalBytes, nullptr);
            if (state == decal_layer::persistence::ReadState::Malformed)
                return false;
            if (state == decal_layer::persistence::ReadState::Absent)
                continue;
            if (!capturedDefinitions.insert(definitionIdentifier).second)
                continue;
            retained_recipe::OwnerKey owner;
            if (!receipt::ParseUUID(impl.documentIdentifier, owner.document)
                || !receipt::ParseUUID(
                    privateDocument->EntityIdentifierForLabel(label),
                    owner.entity)
                || !receipt::ParseUUID(definitionIdentifier,
                                       owner.definition)
                || !(definition.owner == owner)) return false;
            decal_layer::Definition proof = definition;
            const auto savedProof = proof.layerProof;
            decal_layer::source::Witness source;
            if (!decal_layer::BindLayerProof(proof)
                || proof.layerProof != savedProof
                || !decal_layer::source::CaptureSource(
                    privateDocument, owner, *impl.budget,
                    cancelled, source)
                || !decal_layer::source::ExactMatch(definition, source))
                return false;
            const TopoDS_Shape shape = XCAFDoc_ShapeTool::GetShape(label);
            if (shape.IsNull()) return false;
            PrivateDecalCapture::Impl::Owner value;
            value.owner = owner;
            value.copier = std::make_unique<BRepBuilderAPI_Copy>(
                shape, Standard_True, Standard_True);
            if (!value.copier || !value.copier->IsDone()
                || value.copier->Shape().IsNull())
                return false;
            value.detachedAuthorityShape = value.copier->Shape();

            TopTools_IndexedMapOfShape originalFaces, disposableFaces;
            TopExp::MapShapes(shape, TopAbs_FACE, originalFaces);
            TopExp::MapShapes(value.detachedAuthorityShape,
                TopAbs_FACE, disposableFaces);
            if (originalFaces.IsEmpty()
                || originalFaces.Extent() != disposableFaces.Extent())
                return false;
            Bnd_Box stageBox;
            BRepBndLib::AddOptimal(
                shape, stageBox, Standard_False, Standard_False);
            if (stageBox.IsVoid() || stageBox.IsWhole()
                || stageBox.IsOpen()) return false;
            std::set<std::pair<decal_layer::UUID,
                               decal_layer::Digest>> seenReceivers;
            for (const auto& layer : definition.layers) {
                const auto receipt = decal_layer::bake::IntentReceipt(layer);
                if (!seenReceivers.emplace(
                        receipt.face, receipt.selectorProof).second)
                    continue;
                int match = 0;
                dependent_replay::FaceImageGeometricReceipt matchedReceipt;
                TopoDS_Face originalFace;
                for (Standard_Integer index = 1;
                     index <= originalFaces.Extent(); ++index) {
                    dependent_replay::FaceImageGeometricReceipt derived;
                    const auto status = dependent_replay::detail::
                        DeriveFaceImageReceipt(stageBox,
                            TopoDS::Face(originalFaces.FindKey(index)),
                            impl.metersPerUnit, *impl.budget, derived);
                    if (status == dependent_replay::detail::
                            FaceImageDeriveStatus::Budget)
                        return false;
                    if (status != dependent_replay::detail::
                            FaceImageDeriveStatus::Derived)
                        continue;
                    decal_layer::Digest derivedProof{};
                    if (!dependent_replay::FaceImageReceiptProof(
                            derived, derivedProof)) return false;
                    if (derivedProof != receipt.selectorProof) continue;
                    ++match;
                    matchedReceipt = std::move(derived);
                    originalFace = TopoDS::Face(
                        originalFaces.FindKey(index));
                }
                if (match != 1 || originalFace.IsNull()) return false;
                dependent_replay::FaceImageGeometricReceipt resolved;
                TopoDS_Face resolvedFace;
                if (dependent_replay::detail::ResolveFaceImageReceipt(
                        shape, matchedReceipt.intent, impl.metersPerUnit,
                        *impl.budget, resolved, resolvedFace)
                        != retained_face_selector::Refusal::None
                    || !(resolved == matchedReceipt)
                    || !resolvedFace.IsSame(originalFace)) return false;
                const TopoDS_Shape copied =
                    value.copier->ModifiedShape(originalFace);
                const Standard_Integer originalIndex =
                    originalFaces.FindIndex(originalFace);
                const Standard_Integer disposableIndex =
                    disposableFaces.FindIndex(copied);
                if (copied.IsNull() || copied.ShapeType() != TopAbs_FACE
                    || originalIndex <= 0 || disposableIndex <= 0
                    || originalIndex > INT32_MAX
                    || disposableIndex > INT32_MAX) return false;
                value.receivers.push_back({receipt, originalFace,
                    TopoDS::Face(copied),
                    std::uint32_t(originalIndex - 1),
                    std::uint32_t(disposableIndex - 1)});
            }
            if (value.receivers.empty())
                return false;

            retained_finishing::producer::Capture finishing;
            if (!retained_finishing::producer::CaptureSource(
                    document, owner, finishing))
                return false;
            value.ownerLabel = label;
            value.savedMember.savedMember.owner = owner;
            value.savedMember.savedMember.source = finishing.source;
            value.savedMember.authority = finishing;
            value.transientAtlas.document = owner.document;
            for (const auto& layer : definition.layers) {
                face_image::ResourceEnvelope envelope;
                if (!face_image::owner::ReadResource(
                        document, layer.image.resource, envelope))
                    return false;
                painted_atlas_bake::owner::CapturedSource captured;
                captured.fence.owner = owner;
                captured.fence.binding = layer.identifier;
                captured.fence.resource = layer.image.resource;
                captured.fence.selectorProof =
                    decal_layer::bake::IntentReceipt(layer).selectorProof;
                captured.fence.bindingProof = definition.layerProof;
                captured.fence.originalContent =
                    layer.image.originalContent;
                captured.fence.workingContent =
                    layer.image.normalizedContent;
                captured.fence.role = layer.image.role;
                captured.fence.colorSpace = layer.image.colorSpace;
                captured.envelope = std::move(envelope);
                if (!painted_atlas_bake::Valid(captured.fence))
                    return false;
                value.paintedSources.push_back(
                    std::move(captured));
            }
            if (value.paintedSources.empty()) return false;
            value.definition = std::move(definition);
            value.canonicalBytes = std::move(canonicalBytes);
            value.source = std::move(source);
            value.definitionIdentifier = definitionIdentifier;
            impl.owners.push_back(std::move(value));
        }
        if (cancelled()) return false;
        capture = std::move(candidate);
        return true;
    } catch (...) {
        capture.reset();
        return false;
    }
}

bool OcctSceneSnapshotBuilder::FinalizePrivateExportDecals(
    PrivateDecalCapture& capture,
    const SceneSnapshot& finalScene,
    const std::vector<PrivateExportFaceCorrespondence>& correspondence,
    const std::vector<PrivateExportFinalMesh>& emittedMeshes,
    const std::function<bool()>& cancelled,
    PrivateExportDecalArtifact& artifact) noexcept {
    artifact = {};
    if (!capture.impl_ || !cancelled || cancelled()
        || !IsValidSceneSnapshot(finalScene)) return false;
    try {
        const auto& impl = *capture.impl_;
        if (finalScene.metersPerUnit != impl.metersPerUnit
            || correspondence.size() > kMaxPrimitiveBindingsPerSnapshot
            || emittedMeshes.size() > kMaxLabelInstanceMappings)
            return false;
        if (impl.owners.empty() || emittedMeshes.empty()
            || impl.destination.values.size() != emittedMeshes.size()
            || impl.privateDocument.IsNull()) return false;
        const Handle(TDocStd_Document)& document =
            impl.privateDocument->Document();
        if (document.IsNull() || document->HasOpenCommand()) return false;
        std::vector<asset_atlas::persistence::Record> savedAtlases;
        if (!asset_atlas::persistence::ReadAll(document, savedAtlases)
            || !savedAtlases.empty()) return false;
        for (const auto& owner : impl.owners) {
            decal_layer::Definition observed;
            std::vector<std::uint8_t> canonical;
            TDF_Label label;
            retained_solid::Record retained;
            if (!retained_finishing::producer::detail::Resolve(
                    document, owner.owner, label, retained)) return false;
            // Re-read the immutable canonical authority and source fences at
            // the actual visibility boundary, never from the remeshed copy.
            const auto read = decal_layer::persistence::Read(
                document, label, observed, &canonical, nullptr);
            decal_layer::source::Witness source;
            if (read != decal_layer::persistence::ReadState::Present
                || canonical != owner.canonicalBytes
                || !(observed.owner == owner.definition.owner)
                || !decal_layer::source::CaptureSource(
                    impl.privateDocument, owner.owner, *impl.budget,
                    cancelled, source)
                || !decal_layer::source::ExactMatch(owner.definition, source)
                || source.sourceProof != owner.source.sourceProof)
                return false;
        }
        for (const auto& emitted : emittedMeshes) {
            if (emitted.ownerDefinitionIdentifier.empty()
                || emitted.mesh.definitionIdentifier.empty()
                || emitted.instance.entityIdentifier.empty()
                || emitted.instance.decalDerivedAppearances.empty())
                return false;
        }
        for (const auto& value : impl.destination.values) {
            decal_layer::bake::PublishedDerivative reopened;
            if (!decal_layer::bake::publication::Valid(value)
                || !decal_layer::bake::publication::Lookup(
                    impl.destination, value.owner, value.inputKey, reopened)
                || reopened.bakeKey != value.bakeKey) return false;
            artifact.inputKeys.push_back(DecalDigestText(value.inputKey));
            artifact.bakeSeals.push_back(DecalDigestText(value.bakeKey));
        }
        if (cancelled()) return false;
        artifact.scene = std::make_shared<SceneSnapshot>(finalScene);
        artifact.correspondence = correspondence;
        if (!artifact.scene || cancelled()
            || !IsValidSceneSnapshot(*artifact.scene)) {
            artifact = {};
            return false;
        }
        return true;
    } catch (...) {
        artifact = {};
        return false;
    }
}

#ifdef DEBUG
std::array<std::size_t, 4> DebugExerciseE4ResolveDecalImageReadPreflight(
    const Handle(OcctDocument)& document,
    const decal_layer::ImageRef& reference) noexcept
{
    using FailureSite = decal_layer::bake::accounting::FailureSite;
    std::array<std::size_t, 4> result{};
    try {
        OperationLedger denied(0, 0, 0);
        decal_layer::bake::ResolvedImage resolved;
        gDebugResolveReadResourceEntries = 0;
        result[0] = !ResolveDecalImage(
            document, reference, resolved, denied.view());
        result[1] = gDebugResolveReadResourceEntries;
        result[2] = resolved.envelope.originalBytes.empty()
            && resolved.envelope.workingBytes.empty()
            && resolved.pixels.width == 0
            && resolved.pixels.height == 0
            && resolved.pixels.rgba.empty()
            && resolved.originalBytesTicket.bytes == 0
            && resolved.workingBytesTicket.bytes == 0
            && resolved.pixelTicket.bytes == 0
            && !resolved.originalMeasured
            && !resolved.workingMeasured;
        result[3] = denied.firstFailureSite()
            == FailureSite::ResourceTableScratch;
        gDebugResolveReadResourceEntries = 0;
        return result;
    } catch (...) {
        gDebugResolveReadResourceEntries = 0;
        return {};
    }
}

std::array<std::size_t, 10> DebugExerciseE4FaceImageValidation(
    const std::vector<std::uint8_t>& bytes) noexcept
{
    std::array<std::size_t, 10> result{};
    @autoreleasepool {
        try {
            if (bytes.size() < 33
                || std::memcmp(bytes.data(), "\x89PNG\r\n\x1a\n", 8) != 0)
                return result;
            std::vector<std::uint8_t> oversized = bytes;
            const std::uint32_t width =
                std::uint32_t(face_image::kMaximumImageDimension) + 1U;
            oversized[16] = std::uint8_t(width >> 24);
            oversized[17] = std::uint8_t(width >> 16);
            oversized[18] = std::uint8_t(width >> 8);
            oversized[19] = std::uint8_t(width);
            std::uint32_t crc = 0xFFFFFFFFU;
            for (std::size_t index = 12; index < 29; ++index) {
                crc ^= oversized[index];
                for (unsigned bit = 0; bit < 8; ++bit)
                    crc = (crc >> 1)
                        ^ (0xEDB88320U & (0U - (crc & 1U)));
            }
            crc ^= 0xFFFFFFFFU;
            oversized[29] = std::uint8_t(crc >> 24);
            oversized[30] = std::uint8_t(crc >> 16);
            oversized[31] = std::uint8_t(crc >> 8);
            oversized[32] = std::uint8_t(crc);

            OperationLedger rejected(96U * 1024U * 1024U,
                96U * 1024U * 1024U, 128U * 1024U * 1024U);
            FaceImageValidationAdmission rejectedAdmission;
            rejectedAdmission.operation = rejected.view();
            const face_image::validation::FaceImageRasterOptions rejectedOptions{
                &rejectedAdmission, &AdmitFaceImageValidationSource,
                &AdmitFaceImageValidationDecoded,
                &EnterFaceImageValidationDecode};
            NSData *oversizedData = [NSData dataWithBytes:oversized.data()
                length:oversized.size()];
            face_image::validation::FaceImageRasterInfo rejectedInfo;
            result[0] = !face_image::validation::MeasureFaceImageRaster(
                oversizedData, rejectedInfo, rejectedOptions);
            result[1] = rejected.allocationEntries(
                decal_layer::bake::accounting::FailureSite::
                    FaceImageValidationDecode);
            result[2] = rejected.peak(
                decal_layer::bake::accounting::StorageDimension::PrivateStorage);

            OperationLedger admitted(96U * 1024U * 1024U,
                96U * 1024U * 1024U, 128U * 1024U * 1024U);
            FaceImageValidationAdmission admittedTickets;
            admittedTickets.operation = admitted.view();
            const face_image::validation::FaceImageRasterOptions admittedOptions{
                &admittedTickets, &AdmitFaceImageValidationSource,
                &AdmitFaceImageValidationDecoded,
                &EnterFaceImageValidationDecode};
            NSData *validData = [NSData dataWithBytes:bytes.data()
                length:bytes.size()];
            face_image::validation::FaceImageRasterInfo inheritedInfo;
            face_image::validation::FaceImageRasterInfo controlInfo;
            result[3] = face_image::validation::MeasureFaceImageRaster(
                validData, inheritedInfo, admittedOptions);
            result[4] = face_image::validation::MeasureFaceImageRaster(
                validData, controlInfo);
            result[5] = inheritedInfo.width;
            result[6] = inheritedInfo.height;
            result[7] = inheritedInfo.hasAlpha ? 1U : 0U;
            result[8] = result[3] && result[4]
                && inheritedInfo.format == controlInfo.format
                && inheritedInfo.width == controlInfo.width
                && inheritedInfo.height == controlInfo.height
                && inheritedInfo.hasAlpha == controlInfo.hasAlpha;
            result[9] = admitted.allocationEntries(
                decal_layer::bake::accounting::FailureSite::
                    FaceImageValidationDecode);
            return result;
        } catch (...) {
            return {};
        }
    }
}

std::array<std::size_t, 18> DebugExerciseE4OperationLedger() noexcept
{
    using FailureSite = decal_layer::bake::accounting::FailureSite;
    using Job = decal_layer::bake::accounting::Job;
    using Retention = decal_layer::bake::accounting::Retention;
    using StorageDimension =
        decal_layer::bake::accounting::StorageDimension;
    using Ticket = decal_layer::bake::accounting::Ticket;
    std::array<std::size_t, 18> result{};

    OperationLedger exact(16, 16, 16);
    result[0] = exact.reserve(StorageDimension::PrivateStorage, 16,
            Retention::Scratch, FailureSite::DebugExactLimit)
        && exact.live(StorageDimension::PrivateStorage) == 16;
    result[2] = !exact.reserve(StorageDimension::PrivateStorage, 1,
        Retention::Scratch, FailureSite::DebugOneUnit);
    const auto sticky = exact.view();
    result[3] = !sticky.Recheck(FailureSite::ProtectedSceneCopy);
    result[4] = !sticky.Recheck(FailureSite::JobConsume);
    result[5] = !sticky.Recheck(FailureSite::StorageReserve);
    result[6] = !sticky.Recheck(FailureSite::JobSeal);
    result[7] = !sticky.Recheck(FailureSite::ContextControl);

    OperationLedger overflow(16, 16, 16);
    result[1] = !overflow.reserve(StorageDimension::PrivateStorage,
        std::numeric_limits<std::size_t>::max(), Retention::Scratch,
        FailureSite::DebugOneUnit);

    OperationLedger multiplied(64, 64, 64);
    std::size_t product = 0;
    result[13] = !multiplied.view().CheckedProduct(
        std::numeric_limits<std::size_t>::max(), 2, 1, product,
        FailureSite::DebugCheckedProduct)
        && product == 0
        && multiplied.firstFailureSite()
            == FailureSite::DebugCheckedProduct;

    OperationLedger copies(64, 64, 64);
    const auto copyView = copies.view();
    result[8] = copyView.Reserve(StorageDimension::PrivateStorage, 8,
            Retention::Retained, FailureSite::DebugCopyOne)
        && copyView.Reserve(StorageDimension::PrivateStorage, 8,
            Retention::Retained, FailureSite::DebugCopyTwo)
        && copies.live(StorageDimension::PrivateStorage) == 16
        && copies.retained(StorageDimension::PrivateStorage) == 16;

    OperationLedger shared(64, 64, 64);
    result[9] = shared.view().Reserve(StorageDimension::PrivateStorage, 8,
            Retention::Retained, FailureSite::DebugShared)
        && shared.live(StorageDimension::PrivateStorage) == 8
        && shared.retained(StorageDimension::PrivateStorage) == 8;

    OperationLedger moved(64, 64, 64);
    {
        Ticket first;
        result[10] = first.acquire(moved.view(),
            StorageDimension::PrivateStorage, 8, Retention::Scratch,
            FailureSite::DebugMove);
        Ticket second(std::move(first));
        result[10] = result[10]
            && moved.live(StorageDimension::PrivateStorage) == 8;
    }
    result[10] = result[10]
        && moved.live(StorageDimension::PrivateStorage) == 0;

    OperationLedger scratch(64, 64, 64);
    {
        Ticket ticket;
        result[11] = ticket.acquire(scratch.view(),
            StorageDimension::PrivateStorage, 8, Retention::Scratch,
            FailureSite::DebugScratch);
    }
    result[11] = result[11]
        && scratch.live(StorageDimension::PrivateStorage) == 0
        && scratch.peak(StorageDimension::PrivateStorage) == 8;
    result[12] = scratch.sealJob(
            Job::Debug, 5, FailureSite::DebugWork)
        && scratch.consumeJob(Job::Debug, 5, FailureSite::DebugWork)
        && scratch.cumulativeWork() == 5
        && scratch.reserve(StorageDimension::PrivateStorage, 4,
            Retention::Scratch, FailureSite::DebugWork);
    scratch.release(StorageDimension::PrivateStorage, 4,
        Retention::Scratch);
    result[12] = result[12] && scratch.cumulativeWork() == 5;

    OperationLedger duplicate(64, 64, 64);
    result[14] = duplicate.sealJob(
            Job::Debug, 5, FailureSite::DebugDuplicateJob)
        && !duplicate.sealJob(
            Job::Debug, 5, FailureSite::DebugDuplicateJob)
        && duplicate.firstFailureSite()
            == FailureSite::DebugDuplicateJob;

    OperationLedger jobLimit(64, 64, 64);
    result[15] = jobLimit.sealJob(
            Job::Debug, 5, FailureSite::DebugJobLimit)
        && jobLimit.consumeJob(
            Job::Debug, 5, FailureSite::DebugJobLimit)
        && !jobLimit.consumeJob(
            Job::Debug, 1, FailureSite::DebugJobLimit)
        && jobLimit.firstFailureSite() == FailureSite::DebugJobLimit;

    result[16] = std::max({exact.peak(StorageDimension::PrivateStorage),
        copies.peak(StorageDimension::PrivateStorage),
        shared.peak(StorageDimension::PrivateStorage),
        moved.peak(StorageDimension::PrivateStorage),
        scratch.peak(StorageDimension::PrivateStorage)});
    result[17] = scratch.cumulativeWork();
    return result;
}

std::array<std::size_t, 16> DebugExerciseE4SourceDecodeAdmission(
    const std::vector<std::uint8_t>& bytes) noexcept
{
    using FailureSite = decal_layer::bake::accounting::FailureSite;
    constexpr std::size_t encodedLimit = 64U * 1024U * 1024U;
    constexpr std::size_t privateLimit = 128U * 1024U * 1024U;
    std::array<std::size_t, 16> result{};

    OperationLedger denied(96U * 1024U * 1024U, encodedLimit, 0);
    painted_atlas_bake::kernel::Image refused;
    result[0] = !painted_atlas_bake::kernel::DecodeImage(
        bytes, refused, denied.view());
    result[1] = denied.firstFailureSite()
        == FailureSite::ImageDecodedBacking;
    result[2] = denied.allocationEntries(FailureSite::ImageCreate);
    result[3] = denied.allocationEntries(FailureSite::ImageRGBA);

    OperationLedger normal(96U * 1024U * 1024U, encodedLimit, privateLimit);
    painted_atlas_bake::kernel::Image first;
    painted_atlas_bake::kernel::Image second;
    result[4] = painted_atlas_bake::kernel::DecodeImage(
        bytes, first, normal.view());
    result[5] = first.width;
    result[6] = first.height;
    result[7] = first.rgba.size();
    result[8] = result[4]
        && painted_atlas_bake::kernel::DecodeImage(
            bytes, second, normal.view())
        && first.width == second.width && first.height == second.height
        && first.rgba == second.rgba;
    result[9] = normal.allocationEntries(FailureSite::ImageCreate);
    const std::size_t liveBeforeReplacement = normal.live(
        decal_layer::bake::accounting::StorageDimension::PrivateStorage);
    result[10] = painted_atlas_bake::kernel::DecodeImage(
        bytes, first, normal.view());
    result[11] = result[10]
        && normal.live(decal_layer::bake::accounting::StorageDimension::PrivateStorage)
            == liveBeforeReplacement;

    painted_atlas_bake::kernel::Image standalone;
    result[12] = painted_atlas_bake::kernel::DecodeImage(bytes, standalone)
        && painted_atlas_bake::kernel::DecodeImage(bytes, standalone)
        && !standalone.rgba.empty() && standalone.operationOwner.valid();

    OperationLedger inputLedger(96U * 1024U * 1024U,
        encodedLimit, privateLimit);
    decal_layer::bake::Input input;
    input.operationLedger = inputLedger.view();
    decal_layer::bake::ResolvedLayer layer;
    result[13] = layer.image.pixelTicket.acquire(inputLedger.view(),
        decal_layer::bake::accounting::StorageDimension::PrivateStorage,
        16, decal_layer::bake::accounting::Retention::Retained,
        FailureSite::ImageRGBA)
        && inputLedger.enterAllocation(FailureSite::ImageRGBA);
    if (result[13]) {
        layer.image.pixels.width = 2;
        layer.image.pixels.height = 2;
        layer.image.pixels.rgba.resize(16);
        input.layers.push_back(std::move(layer));
        input = decal_layer::bake::Input{};
        result[13] = inputLedger.live(
                decal_layer::bake::accounting::StorageDimension::PrivateStorage) == 0
            && inputLedger.recheck(FailureSite::ImageRGBA);
    }

    SceneSnapshot scene;
    scene.publicationSourceIdentifier = "debug-scene-copy";
    scene.meshes.reserve(1);
    SceneCopyStorage sceneStorage;
    OperationLedger sceneDenied(0, 0, 0);
    decal_layer::bake::accounting::Ticket sceneTicket;
    result[14] = DeepSceneStorageBytes(scene, sceneStorage)
        && !sceneTicket.acquire(sceneDenied.view(),
            decal_layer::bake::accounting::StorageDimension::SnapshotNumeric,
            sceneStorage.numeric,
            decal_layer::bake::accounting::Retention::Retained,
            FailureSite::ProtectedSceneCopy)
        && sceneDenied.allocationEntries(FailureSite::ProtectedSceneCopy) == 0
        && scene.meshes.empty();

    auto owned = decal_layer::bake::accounting::MakeOperationOwner();
    result[15] = owned.valid()
        && owned.view.Recheck(FailureSite::ContextControl);
    return result;
}

void DebugBeginE4OrdinaryLedgerObservation() noexcept
{
    gDebugOrdinaryLedgerObservationArmed = true;
    gDebugOrdinaryLedgerContext = nullptr;
    gDebugOrdinaryLedgerCalls = 0;
    gDebugOrdinaryLedgerMismatch = false;
}

std::array<std::size_t, 2> DebugTakeE4OrdinaryLedgerObservation() noexcept
{
    const std::array<std::size_t, 2> result{{
        gDebugOrdinaryLedgerCalls,
        gDebugOrdinaryLedgerCalls >= 2 && !gDebugOrdinaryLedgerMismatch ? 1U : 0U,
    }};
    gDebugOrdinaryLedgerObservationArmed = false;
    gDebugOrdinaryLedgerContext = nullptr;
    gDebugOrdinaryLedgerCalls = 0;
    gDebugOrdinaryLedgerMismatch = false;
    return result;
}

void DebugBeginBoundedCurvePublicationObservation() noexcept
{
    gDebugBoundedCurveObservation.reset();
    gDebugBoundedCurveObservationArmed = true;
}

bool DebugTakeBoundedCurvePublicationObservation(
    DebugBoundedCurvePublicationObservation& theObservation) noexcept
{
    if (!gDebugBoundedCurveObservation.has_value()) return false;
    theObservation = std::move(*gDebugBoundedCurveObservation);
    gDebugBoundedCurveObservation.reset();
    gDebugBoundedCurveObservationArmed = false;
    return true;
}

void DebugCancelBoundedCurvePublicationObservation() noexcept
{
    gDebugBoundedCurveObservation.reset();
    gDebugBoundedCurveObservationArmed = false;
}
#endif

struct OcctSceneSnapshotBuilder::State {
    using LabelInstanceMap =
        std::unordered_map<std::string, std::vector<std::size_t>>;
    struct NativeContactSource {
        TDF_Label definitionLabel;
        std::string definitionIdentifier;
        std::uint64_t geometryRevision=0;
        OcctGeometryRepresentation representation=OcctGeometryRepresentation::Invalid;
        std::array<double,16> worldFromObject{};
        std::array<double,3> sourceOrigin{};
        double metersPerUnit=0;
    };
    using NativeContactSourceMap=std::unordered_map<std::string,NativeContactSource>;
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
    // Labels remain private on the document owner. Immutable map sharing keeps
    // presentation-only State copies independent of scene size.
    std::shared_ptr<const NativeContactSourceMap> lastFullNativeContactSources;
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

meshcheck::ContactSourceStatus
OcctSceneSnapshotBuilder::CaptureNativeMeshContacts(
    const Handle(OcctDocument)& document,
    const meshcheck::ContactSourceIdentity& expected,
    const std::atomic_bool& cancelled,
    meshcheck::ContactSourceCapture& output) noexcept
{
    using Status=meshcheck::ContactSourceStatus;
    output={};
    if (cancelled.load(std::memory_order_relaxed)) return Status::Cancelled;
    if (![NSThread isMainThread]) return Status::InternalFailure;
    if (document.IsNull() || myState==nullptr
        || myState->documentObject.IsNull()
        || !myState->lastFullNativeContactSources
        || expected.entityIdentifier.empty() || expected.entityIdentifier.size()>128
        || expected.definitionIdentifier.empty() || expected.definitionIdentifier.size()>128
        || expected.publicationSourceIdentifier.empty()
        || expected.publicationSourceIdentifier!=myState->publicationSourceIdentifier
        || expected.documentGeneration==0
        || expected.documentGeneration!=myState->documentGeneration
        || expected.modelRevision==0 || expected.modelRevision!=myState->modelRevision
        || expected.geometryRevision==0) return Status::StaleSource;
    try {
        OCC_CATCH_SIGNALS
        const auto& native=document->Document();
        if (native.IsNull() || native->HasOpenCommand()
            || native.get()!=myState->documentObject.get()
            || document->DocumentIdentifier()!=myState->documentIdentifier)
            return Status::StaleSource;
        const auto& data=native->GetData();
        if (data.IsNull() || data->Time()!=myState->lastFullDocumentTime)
            return Status::StaleSource;
        const auto found=myState->lastFullNativeContactSources->find(expected.entityIdentifier);
        if (found==myState->lastFullNativeContactSources->end()) return Status::StaleSource;
        const auto& source=found->second;
        if (source.definitionIdentifier!=expected.definitionIdentifier
            || source.geometryRevision!=expected.geometryRevision
            || source.definitionLabel.IsNull()
            || document->DefinitionIdentifierForLabel(source.definitionLabel)
                !=expected.definitionIdentifier) return Status::StaleSource;
        if (source.representation!=OcctGeometryRepresentation::TriangleMesh)
            return Status::Unsupported;
        // Only a finite nonsingular occurrence placement preserves incidence
        // for later world inspection. Very small determinant underflow rejects
        // conservatively; it must never turn a collapsed transform into clean.
        const auto& m=source.worldFromObject;
        for (const auto value:m) if (!std::isfinite(value)) return Status::InvalidGeometry;
        const auto determinant=m[0]*(m[5]*m[10]-m[9]*m[6])
            -m[4]*(m[1]*m[10]-m[9]*m[2])+m[8]*(m[1]*m[6]-m[5]*m[2]);
        if (!std::isfinite(determinant) || determinant==0) return Status::InvalidGeometry;
        meshcheck::ContactMeshCoordinates coordinates;
        const auto status=meshcheck::CaptureContactMeshCoordinates(
            XCAFDoc_ShapeTool::GetShape(source.definitionLabel),coordinates,cancelled);
        using Capture=meshcheck::ContactCaptureStatus;
        switch(status) {
            case Capture::Ready: break;
            case Capture::Unsupported: return Status::Unsupported;
            case Capture::Invalid: return Status::InvalidGeometry;
            case Capture::TooLarge: return Status::ResourceLimit;
            case Capture::Cancelled: return Status::Cancelled;
            case Capture::TimedOut: return Status::TimedOut;
        }
        meshcheck::ContactSourceCapture captured;
        captured.identity=expected;
        captured.documentTime=data->Time();
        captured.storedNodeCount=coordinates.storedNodeCount;
        captured.facePlacement=coordinates.facePlacement;
        captured.faceOrientation=coordinates.faceOrientation;
        captured.worldFromObject=source.worldFromObject;
        captured.sourceOrigin=source.sourceOrigin;
        captured.metersPerUnit=source.metersPerUnit;
        captured.triangles=std::move(coordinates.triangles);
        captured.storedNodes=std::move(coordinates.storedNodes);
        captured.triangleNodeIDs=std::move(coordinates.triangleNodeIDs);
        if (cancelled.load(std::memory_order_relaxed)) return Status::Cancelled;
        output=std::move(captured);
        return Status::Ready;
    } catch (const std::bad_alloc&) { return Status::ResourceLimit; }
      catch (const std::length_error&) { return Status::ResourceLimit; }
      catch (...) { return Status::InternalFailure; }
}


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

std::uint64_t OcctSceneSnapshotBuilder::DebugPublishedDocumentGeneration(
    const Handle(OcctDocument)& theDocument) const noexcept
{
    if (![NSThread isMainThread] || theDocument.IsNull()
        || myState == nullptr || myState->documentObject.IsNull()
        || myState->publicationSourceIdentifier.empty()
        || myState->documentGeneration == 0 || myState->snapshotRevision == 0) {
        return 0;
    }
    try {
        OCC_CATCH_SIGNALS
        const Handle(TDocStd_Document)& aDocument = theDocument->Document();
        if (aDocument.IsNull()
            || aDocument.get() != myState->documentObject.get()
            || theDocument->DocumentIdentifier() != myState->documentIdentifier) {
            return 0;
        }
        // Only metadata from the last committed publication is exposed. The
        // HasOpenCommand barrier in CaptureFrame remains mandatory.
        return myState->documentGeneration;
    } catch (...) {
        return 0;
    }
}

std::uint64_t OcctSceneSnapshotBuilder::DebugPublishedModelRevision(
    const Handle(OcctDocument)& theDocument) const noexcept
{
    // Reuse the main-thread and exact document/publication identity gate.
    // Reading metadata must never bypass CaptureFrame's open-command barrier.
    return DebugPublishedDocumentGeneration(theDocument) == 0
        ? 0 : myState->modelRevision;
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
    const bool theAllowsShellPreview,
    const bool theAllowsRadialArrayPreview) noexcept
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
        || (theContent.kind
                == PresentationOverlayKind::RadialArrayPreview
            && !theAllowsRadialArrayPreview)
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
            || anItem.material.emissiveTextureIndex != -1
            || anItem.material.metallicRoughnessTextureIndex != -1
            || anItem.material.normalTextureIndex != -1
            || anItem.material.occlusionTextureIndex != -1) {
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
            true,
            false);
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
        false,
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
            || aSourceItem.material.emissiveTextureIndex != -1
            || aSourceItem.material.metallicRoughnessTextureIndex != -1
            || aSourceItem.material.normalTextureIndex != -1
            || aSourceItem.material.occlusionTextureIndex != -1) {
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
                || !HasCompatibleArrayPreviewStyle(
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
            false,
            false);
    } catch (const Standard_Failure&) {
        return {};
    } catch (...) {
        return {};
    }
}

OcctSceneSnapshotBuilder::OverlayPointer
OcctSceneSnapshotBuilder::PublishEmptyRadialArrayPreviewOverlay(
    const Handle(OcctDocument)& theDocument) noexcept
{
    PresentationOverlayContent aContent;
    aContent.kind = PresentationOverlayKind::RadialArrayPreview;
    return PublishPresentationOverlayImpl(
        theDocument,
        std::move(aContent),
        false,
        false,
        false,
        false,
        false,
        true);
}

OcctSceneSnapshotBuilder::OverlayPointer
OcctSceneSnapshotBuilder::PublishRadialArrayPreviewOverlay(
    const Handle(OcctDocument)& theDocument,
    const Handle(AIS_Shape)& theSourceShape,
    const std::vector<Handle(AIS_Shape)>& thePreviewShapes) noexcept
{
    const std::size_t anInstanceCount = thePreviewShapes.size();
    if (![NSThread isMainThread] || theDocument.IsNull()
        || theSourceShape.IsNull() || myState == nullptr
        || anInstanceCount == 0
        || anInstanceCount > kMaxRadialArrayPreviewBodies) {
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
            || aData->Time() != myState->lastFullDocumentTime
            || theSourceShape->Shape().IsNull()
            || HasUnsupportedPresentationTexture(theSourceShape)) {
            return {};
        }

        const TopoDS_Shape& aSourceShape = theSourceShape->Shape();
        const gp_Trsf aSourceTransform =
            theSourceShape->Transformation();
        Matrix4d aSourceTransformMatrix;
        if (!MatrixFromTransform(
                aSourceTransform, aSourceTransformMatrix)) {
            return {};
        }
        const gp_Trsf anInverseSourceTransform =
            aSourceTransform.Inverted();

        const std::size_t aMaximumPrimitiveCount = std::min(
            kMaxOverlayPrimitives,
            kMaxOverlayPrimitiveBindings / anInstanceCount);
        if (aMaximumPrimitiveCount == 0) {
            return {};
        }
        WorldPreviewItem aSourceItem;
        if (!ExtractWorldPreviewItem(
                theSourceShape,
                "radial-array/source/0",
                "Radial array source 0",
                0,
                RenderRole::RadialArrayPreview,
                RenderStyle::Shaded,
                std::nullopt,
                kMaxOverlayVertices,
                kMaxOverlayIndices,
                aMaximumPrimitiveCount,
                aSourceItem)
            || !IsTranslationOnlyWorldTransform(
                aSourceItem.instance.worldFromObject)
            || !IsProperRigidBoundedWorldTransform(
                aSourceItem.instance.worldFromObject)
            || aSourceItem.material.baseColorTextureIndex != -1
            || aSourceItem.material.emissiveTextureIndex != -1
            || aSourceItem.material.metallicRoughnessTextureIndex != -1
            || aSourceItem.material.normalTextureIndex != -1
            || aSourceItem.material.occlusionTextureIndex != -1) {
            return {};
        }

        const gp_Vec aSourceCenter(
            aSourceItem.instance.worldFromObject.values[12],
            aSourceItem.instance.worldFromObject.values[13],
            aSourceItem.instance.worldFromObject.values[14]);
        gp_Trsf aSourceCenterTransform;
        aSourceCenterTransform.SetTranslation(aSourceCenter);

        PresentationOverlayContent aContent;
        aContent.kind = PresentationOverlayKind::RadialArrayPreview;
        aContent.meshes.reserve(1);
        aContent.materials.reserve(1);
        aContent.instances.reserve(anInstanceCount);
        std::unordered_set<const AIS_Shape*> aSeenPresentations;
        aSeenPresentations.reserve(anInstanceCount + 1U);
        aSeenPresentations.insert(theSourceShape.get());
        for (std::size_t anIndex = 0;
             anIndex < anInstanceCount; ++anIndex) {
            const Handle(AIS_Shape)& aPresentation =
                thePreviewShapes[anIndex];
            if (aPresentation.IsNull()
                || !aSeenPresentations.insert(
                    aPresentation.get()).second
                || aPresentation->Shape().IsNull()
                || !aPresentation->Shape().IsPartner(aSourceShape)
                || !aPresentation->Shape().IsEqual(aSourceShape)
                || !HasCompatibleArrayPreviewStyle(
                    theSourceShape, aPresentation)) {
                return {};
            }

            // Scene matrices are column-major and multiply column vectors. The
            // source triangulation above is already baked through T and then
            // centered about C. For Mi = R(P,A,theta_i) * T, the proper-rigid
            // delta is Mi * inverse(T), and the final instance is delta * C.
            const gp_Trsf aRelativeTransform =
                aPresentation->Transformation().Multiplied(
                    anInverseSourceTransform);
            Matrix4d aRelativeMatrix;
            if (!MatrixFromTransform(
                    aRelativeTransform, aRelativeMatrix)
                || !IsProperRigidBoundedWorldTransform(
                    aRelativeMatrix)) {
                return {};
            }
            const gp_Trsf anInstanceTransform =
                aRelativeTransform.Multiplied(
                    aSourceCenterTransform);

            InstanceSnapshot anInstance = aSourceItem.instance;
            const std::size_t anOrdinal = anIndex + 1U;
            anInstance.entityIdentifier =
                "radial-array/preview/0/"
                + std::to_string(anOrdinal);
            anInstance.name =
                "Radial array preview " + std::to_string(anOrdinal);
            anInstance.meshIndex = 0;
            anInstance.role = RenderRole::RadialArrayPreview;
            anInstance.coordinateSpace = CoordinateSpace::World;
            anInstance.depthPolicy = DepthPolicy::Scene;
            anInstance.renderStyle = RenderStyle::Shaded;
            if (!MatrixFromTransform(
                    anInstanceTransform,
                    anInstance.worldFromObject)
                || !IsProperRigidBoundedWorldTransform(
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
            false,
            false,
            true);
    } catch (const Standard_Failure&) {
        return {};
    } catch (...) {
        return {};
    }
}

OcctSceneSnapshotBuilder::SnapshotPointer
OcctSceneSnapshotBuilder::BuildPrivateExportDerivative(
    const Handle(OcctDocument)& document,
    const SceneSnapshot& source,
    bool selectedObjectsOnly,
    const std::function<void(const TopoDS_Shape&)>& meshPrivateSurfaces,
    const std::function<bool()>& cancelled
#ifdef DEBUG
    , const DebugPrivateExportDecalFault debugFault
    , DebugPrivateExportDecalObservation* const debugObservation
#endif
    ) noexcept
{
#ifdef DEBUG
    if (debugObservation) {
        *debugObservation = {};
        debugObservation->failureStage = "admission";
    }
#endif
    if ([NSThread isMainThread] || document.IsNull() || !meshPrivateSurfaces
        || !cancelled || cancelled() || !IsValidSceneSnapshot(source)) { return {}; }
    try {
        const auto& ocaf = document->Document();
        if (ocaf.IsNull() || ocaf->HasOpenCommand()) { return {}; }
        // Census the strict saved records on the private document. Viewport
        // appearance/cache presence is never authority for whether capture is
        // required. Absence keeps the established non-E4 remesh path; an
        // unproven census fails closed instead of being treated as absence.
        if (!XCAFDoc_DocumentTool::CheckShapeTool(ocaf->Main())) return {};
        const Handle(XCAFDoc_ShapeTool) shapeTool =
            XCAFDoc_DocumentTool::ShapeTool(ocaf->Main());
        if (shapeTool.IsNull()) return {};
        TDF_LabelSequence roots;
        shapeTool->GetFreeShapes(roots);
        const auto closure = decal_layer::persistence::ReadExportClosure(
            ocaf, roots, cancelled);
        if (closure == decal_layer::persistence::ExportClosureState::Cancelled
            || closure == decal_layer::persistence::ExportClosureState::Unproven)
            return {};
        const bool needsDecalCapture = closure
            == decal_layer::persistence::ExportClosureState::ContainsRecord;
#ifdef DEBUG
        if (needsDecalCapture
            && debugFault == DebugPrivateExportDecalFault::Capture) {
            if (debugObservation)
                debugObservation->failureStage = "capture-fault";
            return {};
        }
#endif
        PrivateDecalCapturePointer decalCapture;
        if (needsDecalCapture
            && (!CapturePrivateExportDecals(document, source,
                    selectedObjectsOnly, cancelled, decalCapture)
                || !decalCapture || !decalCapture->impl_
                || decalCapture->impl_->owners.empty())) return {};
#ifdef DEBUG
        if (debugObservation && decalCapture && decalCapture->impl_) {
            auto& observed = *debugObservation;
            observed.captureReached = true;
            observed.failureStage = "captured";
            observed.ownerCount = decalCapture->impl_->owners.size();
            std::set<std::string> receivers;
            std::set<std::string> savedDefinitions;
            for (const auto& owner : decalCapture->impl_->owners) {
                core3d::decal_layer::Digest canonical{};
                if (!core3d::face_image::HashFaceImageBytes(
                        owner.canonicalBytes, canonical)) return {};
                observed.canonicalReadDigests.push_back(
                    DecalDigestText(canonical));
                observed.sourceReadDigests.push_back(
                    DecalDigestText(owner.source.sourceProof));
                savedDefinitions.insert(owner.definitionIdentifier);
                for (const auto& layer : owner.definition.layers)
                    receivers.insert(DecalDigestText(
                        core3d::decal_layer::bake::IntentReceipt(layer)
                            .selectorProof));
            }
            observed.requiredReceiverCount = receivers.size();
            std::set<std::size_t> countedMeshes;
            for (const auto& instance : source.instances) {
                if (instance.meshIndex >= source.meshes.size()) return {};
                const auto& mesh = source.meshes[instance.meshIndex];
                const std::string& master =
                    mesh.paintedAtlasMasterDefinitionIdentifier.empty()
                        ? mesh.definitionIdentifier
                        : mesh.paintedAtlasMasterDefinitionIdentifier;
                if (savedDefinitions.count(master) == 0
                    || !countedMeshes.insert(instance.meshIndex).second)
                    continue;
                observed.savedTriangleCount += mesh.indices.size() / 3;
                observed.savedCornerCount += mesh.indices.size();
            }
            std::vector<decal_math::DecalWorldTriangle> triangles;
            Bounds3d bounds;
            core3d::decal_layer::Digest occluders{};
            if (!decal_math::CaptureDecalOccluders(
                    source, triangles, bounds, occluders)) return {};
            observed.occluderDigest = DecalDigestText(occluders);
            observed.operationWork = triangles.size();
        }
#endif
        Standard_Real metersPerUnit = kLegacyMetersPerUnit;
#ifdef DEBUG
        if (debugObservation && decalCapture)
            debugObservation->failureStage = "preparing-occurrences";
#endif
        XCAFDoc_DocumentTool::GetLengthUnit(ocaf, metersPerUnit);
        if (metersPerUnit != source.metersPerUnit) { return {}; }
        const auto documentIdentifier = document->DocumentIdentifier();
        std::unordered_map<std::string, std::size_t> wanted;
        for (std::size_t i = 0; i < source.instances.size(); ++i) {
            const auto& item = source.instances[i];
            if (item.role == RenderRole::Model && item.visible
                && (!selectedObjectsOnly || item.selected)) {
                wanted.emplace(item.entityIdentifier, i);
            }
        }
        if (wanted.empty() || (selectedObjectsOnly
            && source.selectionMode != ElementKind::Object)) { return {}; }
        struct ExportOccurrence {
            std::size_t instance;
            std::size_t definition;
            gp_Trsf transform;
            OcctPaintedAtlasDerivative paintedDerivative;
            bool painted = false;
        };
        std::vector<ExportOccurrence> occurrences;
        std::vector<DefinitionData> definitions;
        std::unordered_map<std::string, std::size_t> definitionIndices;
        std::unordered_map<std::string,
            PrivateDecalCapture::Impl::Owner*> capturedOwners;
        if (decalCapture && decalCapture->impl_)
            for (auto& owner : decalCapture->impl_->owners)
                if (!capturedOwners.emplace(
                        owner.definitionIdentifier, &owner).second)
                    return {};
        std::unordered_set<std::string> matched;
        TopoDS_Compound compound;
        BRep_Builder builder;
        builder.MakeCompound(compound);
        XCAFPrs_DocumentExplorer explorer(ocaf,
            XCAFPrs_DocumentExplorerFlags_OnlyLeafNodes, XCAFPrs_Style());
        for (; explorer.More(); explorer.Next()) {
            if (cancelled()) { return {}; }
            const auto& node = explorer.Current();
            std::vector<std::string> path;
            if (explorer.CurrentDepth() < 0
                || explorer.CurrentDepth() >= kMaxOccurrenceDepth) { return {}; }
            for (Standard_Integer depth = 0; depth <= explorer.CurrentDepth(); ++depth) {
                path.push_back(document->EntityIdentifierForLabel(explorer.Current(depth).Label));
            }
            const auto entity = DeriveOccurrenceIdentifier(documentIdentifier, path);
            const auto found = wanted.find(entity);
            if (found == wanted.end()) { continue; }
            if (!matched.insert(entity).second) { return {}; }
            const auto label = node.RefLabel.IsNull() ? node.Label : node.RefLabel;
            const auto identifier = document->DefinitionIdentifierForLabel(label);
            const auto& item = source.instances[found->second];
            const auto& sourceMesh = source.meshes[item.meshIndex];
            // Never reuse viewport derivative bytes after remeshing. A
            // strictly captured owner continues into detached final-input
            // preparation; an appearance without such authority fails closed.
            if (!item.decalDerivedAppearances.empty()
                && capturedOwners.find(identifier)
                    == capturedOwners.end()) return {};
            const std::string& masterIdentifier =
                sourceMesh.paintedAtlasMasterDefinitionIdentifier.empty()
                    ? sourceMesh.definitionIdentifier
                    : sourceMesh.paintedAtlasMasterDefinitionIdentifier;
            if (identifier != masterIdentifier) { return {}; }
            OcctPaintedAtlasDerivative paintedDerivative;
            const bool painted = !sourceMesh.paintedAtlasBakeProof.empty();
            if (painted) {
                core3d::retained_recipe::OwnerKey owner;
                if (!core3d::receipt::ParseUUID(documentIdentifier, owner.document)
                    || !core3d::receipt::ParseUUID(
                        document->EntityIdentifierForLabel(label), owner.entity)
                    || !core3d::receipt::ParseUUID(identifier, owner.definition)
                    || !document->PaintedAtlasDerivativeForOwner(
                        owner, paintedDerivative)
                    || PaintedAtlasDigestText(paintedDerivative.bake.bakeProof)
                        != sourceMesh.paintedAtlasBakeProof) return {};
            }
            // Authored polygon topology/UVs/normals are already frozen. Do not
            // recopy, clean, remesh or transform those buffers through OCCT.
            const auto representation = document->GeometryRepresentationForLabel(label);
            if (representation == OcctGeometryRepresentation::TriangleMesh) { continue; }
            if (representation == OcctGeometryRepresentation::Invalid) { return {}; }
            auto known = definitionIndices.find(identifier);
            if (known == definitionIndices.end()) {
                DefinitionData definition;
                definition.label = label;
                const auto captured = capturedOwners.find(identifier);
                definition.shape = captured == capturedOwners.end()
                    ? XCAFDoc_ShapeTool::GetShape(label)
                    : captured->second->detachedAuthorityShape;
                definition.representation = representation;
                if (definition.shape.IsNull()) { return {}; }
                builder.Add(compound, definition.shape);
                known = definitionIndices.emplace(identifier, definitions.size()).first;
                definitions.push_back(std::move(definition));
            } else if (!definitions[known->second].label.IsEqual(label)) { return {}; }
            gp_Trsf objectTransform;
            if (!document->TryObjectTransformForLabel(label, objectTransform)) { return {}; }
            occurrences.push_back({found->second, known->second,
                objectTransform.Multiplied(node.Location.Transformation()),
                std::move(paintedDerivative), painted});
        }
        if (matched.size() != wanted.size() || cancelled()) { return {}; }
#ifdef DEBUG
        if (debugObservation && decalCapture)
            debugObservation->failureStage = "meshing-final-copy";
#endif
        if (!definitions.empty()) { meshPrivateSurfaces(compound); }
        if (cancelled()) { return {}; }
#ifdef DEBUG
        if (debugFault == DebugPrivateExportDecalFault::FinalProduction) {
            if (debugObservation)
                debugObservation->failureStage = "final-production-fault";
            return {};
        }
#endif
        auto result = std::make_shared<SceneSnapshot>(source);
        std::size_t totalVertices = 0, totalIndices = 0;
#ifdef DEBUG
        if (debugObservation && decalCapture)
            debugObservation->failureStage = "extracting-final-copy";
#endif
        for (const auto& entry : definitionIndices) {
            auto& definition = definitions[entry.second];
            if (cancelled() || !ExtractDefinitionGeometry(definition.label, entry.first,
#ifdef DEBUG
                    0,
#endif
                    definition)) { return {}; }
        }
        std::vector<PrivateExportFaceCorrespondence> finalCorrespondence;
        std::vector<std::string> finalInputKeys;
        std::vector<std::string> finalBakeSeals;
        std::vector<std::string> finalOutputRoles;
        std::vector<std::uint64_t> finalOutputRoleBytes;
        std::size_t finalTriangleCount = 0;
        std::size_t finalCornerCount = 0;
        std::size_t finalResidentBytes = 0;
        std::size_t finalLayoutResolution = 0;
        std::size_t finalLayoutChartCount = 0;
        std::unordered_map<std::string, OcctPaintedAtlasDerivative>
            finalDerivatives;
        std::unordered_map<std::string,
            painted_atlas_bake::owner::ExportBake> finalBakes;
        if (decalCapture && decalCapture->impl_) {
#ifdef DEBUG
            if (debugObservation)
                debugObservation->failureStage = "building-final-atlas";
#endif
            for (auto& owner : decalCapture->impl_->owners) {
                if (cancelled()) return {};
                const auto found = definitionIndices.find(
                    owner.definitionIdentifier);
                if (found == definitionIndices.end()) {
#ifdef DEBUG
                    if (debugObservation)
                        debugObservation->failureStage =
                            "final-owner-definition-missing";
#endif
                    return {};
                }
                auto& definition = definitions[found->second];
#ifdef DEBUG
                if (debugObservation)
                    debugObservation->failureStage =
                        "constructing-final-member";
#endif
                asset_atlas::build::FinalMemberInput finalMember =
                    owner.savedMember;
                if (definition.mesh.indices.empty()
                    || definition.mesh.indices.size() % 3 != 0
                    || definition.faces.IsEmpty()) return {};
                finalMember.triangles.resize(
                    definition.mesh.indices.size() / 3);
                for (std::size_t triangle = 0;
                     triangle < finalMember.triangles.size(); ++triangle) {
                    for (std::size_t corner = 0; corner < 3; ++corner) {
                        const std::uint32_t index = definition.mesh.indices[
                            triangle * 3 + corner];
                        if (index >= definition.mesh.vertices.size()) return {};
                        const auto& vertex = definition.mesh.vertices[index];
                        finalMember.triangles[triangle].points[corner] = {
                            double(vertex.positionX) + definition.sourceOrigin.x,
                            double(vertex.positionY) + definition.sourceOrigin.y,
                            double(vertex.positionZ) + definition.sourceOrigin.z};
                    }
                }
#ifdef DEBUG
                if (debugObservation)
                    debugObservation->failureStage =
                        "constructing-final-faces";
#endif
                for (const auto& primitive : definition.mesh.primitives) {
                    if (primitive.faceIndex >= std::uint32_t(
                            definition.faces.Extent())
                        || primitive.firstIndex % 3 != 0
                        || primitive.indexCount == 0
                        || primitive.indexCount % 3 != 0
                        || std::size_t(primitive.firstIndex)
                            + primitive.indexCount
                            > definition.mesh.indices.size()) return {};
                    const TopoDS_Face face = TopoDS::Face(
                        definition.faces.FindKey(
                            Standard_Integer(primitive.faceIndex + 1)));
                    BRepAdaptor_Surface surface(face, Standard_True);
                    if (surface.GetType() != GeomAbs_Plane) return {};
                    const gp_Pln plane = surface.Plane();
                    shapeyard::uv::curved::FaceInput input;
                    input.surfaceType = provenance::kSurfaceTypePlane;
                    input.firstTriangle = int(primitive.firstIndex / 3);
                    input.triangleCount = int(primitive.indexCount / 3);
                    input.reversed = face.Orientation() == TopAbs_REVERSED;
                    input.params = {
                        plane.Location().X(), plane.Location().Y(),
                        plane.Location().Z(),
                        plane.Axis().Direction().X(),
                        plane.Axis().Direction().Y(),
                        plane.Axis().Direction().Z(),
                        plane.Position().XDirection().X(),
                        plane.Position().XDirection().Y(),
                        plane.Position().XDirection().Z()};
                    input.toleranceMM = 1.0e-7;
                    input.xDirection = {
                        input.params[6], input.params[7], input.params[8]};
                    finalMember.faces.push_back(std::move(input));
                }
                if (finalMember.faces.empty()) {
#ifdef DEBUG
                    if (debugObservation)
                        debugObservation->failureStage =
                            "final-triangle-input-refused";
#endif
                    return {};
                }
#ifdef DEBUG
                if (debugObservation)
                    debugObservation->failureStage =
                        "building-transient-finishing";
#endif
                std::string finishingDiagnosis;
                const retained_finishing::producer::Settings finishingSettings{
                    retained_finishing::UnwrapPolicy::Planar, 256, 4};
                if (retained_finishing::producer::BuildDerivative(
                        decalCapture->impl_->privateDocument->Document(),
                        owner.ownerLabel, owner.savedMember.authority,
                        finishingSettings, owner.transientFinishing,
                        finishingDiagnosis)
                        != retained_finishing::producer::Status::Produced) {
#ifdef DEBUG
                    if (debugObservation)
                        debugObservation->failureStage =
                            "transient-finishing-refused-"
                            + finishingDiagnosis.substr(0, 96);
#endif
                    return {};
                }
                finalMember.savedMember.finishing =
                    owner.transientFinishing.finishing;
                owner.transientAtlas.atlas =
                    retained_finishing::producer::detail::UUIDFromDigest(
                        owner.transientFinishing.chartProof, 0xE4);
                if (!retained_recipe::Nonzero(owner.transientAtlas.atlas))
                    return {};
#ifdef DEBUG
                if (debugObservation)
                    debugObservation->failureStage =
                        "building-transient-atlas";
#endif
                painted_atlas_bake::owner::ExportBake baked;
                painted_atlas_bake::owner::TransientExportCapture transient;
                transient.atlas = owner.transientAtlas;
                transient.member = finalMember.savedMember;
                transient.sources = owner.paintedSources;
                transient.operation = decalCapture->impl_->operationLedger.view();
                std::string atlasDiagnosis;
                const asset_atlas::build::Settings atlasSettings{256, 4};
                const auto atlasOutcome = painted_atlas_bake::owner::
                    BuildAndBakeTransientForExport(transient, finalMember,
                        atlasSettings, baked, atlasDiagnosis);
                if (atlasOutcome
                        != painted_atlas_bake::owner::Outcome::Prepared) {
#ifdef DEBUG
                    if (debugObservation)
                        debugObservation->failureStage =
                            "transient-atlas-refused-"
                            + std::to_string(std::uint8_t(atlasOutcome))
                            + "-" + atlasDiagnosis.substr(0, 96);
#endif
                    return {};
                }
#ifdef DEBUG
                if (debugObservation)
                    debugObservation->failureStage =
                        "transient-kernel-bake-built";
#endif
                if (baked.outputs.empty() || baked.atlas.charts.empty()
                    || baked.assignments.size() != 1
                    || baked.assignments.front().corners.size()
                        != finalMember.triangles.size() * 3U) {
#ifdef DEBUG
                    if (debugObservation)
                        debugObservation->failureStage =
                            "final-atlas-output-incomplete-"
                            + std::to_string(baked.outputs.size()) + "-"
                            + std::to_string(baked.atlas.charts.size()) + "-"
                            + std::to_string(baked.assignments.size()) + "-"
                            + std::to_string(baked.assignments.empty() ? 0
                                : baked.assignments.front().corners.size())
                            + "-" + std::to_string(
                                finalMember.triangles.size() * 3U);
#endif
                    return {};
                }
#ifdef DEBUG
                if (debugObservation)
                    debugObservation->failureStage =
                        "transient-output-complete";
#endif
                OcctPaintedAtlasDerivative derivative;
                derivative.bake = baked.bake;
                derivative.assignment = baked.assignments.front();
                if (!finalDerivatives.emplace(owner.definitionIdentifier,
                        std::move(derivative)).second) return {};
                const std::string inputKey =
                    PaintedAtlasDigestText(baked.atlas.layoutProof);
                const std::string bakeSeal =
                    PaintedAtlasDigestText(baked.bake.bakeProof);
                if (inputKey.size() != 64 || bakeSeal.size() != 64)
                    return {};
                if (!finalBakes.emplace(owner.definitionIdentifier,
                        baked).second) return {};
#ifdef DEBUG
                if (debugObservation)
                    debugObservation->failureStage =
                        "transient-products-recorded";
#endif
                finalInputKeys.push_back(inputKey);
                finalBakeSeals.push_back(bakeSeal);
                finalLayoutResolution = std::max<std::size_t>(
                    finalLayoutResolution,
                    std::uint64_t(baked.atlas.resolutionTexels));
                if (!CheckedAdd(finalLayoutChartCount,
                        std::size_t(1),
                        finalLayoutChartCount)) return {};
                for (const auto& output : baked.outputs) {
                    if (output.png.empty()) return {};
                    finalOutputRoles.emplace_back(
                        PrivatePaintedRoleText(output.descriptor.role));
                    finalOutputRoleBytes.push_back(output.png.size());
                    std::size_t resident = 0;
                    if (!CheckedAdd(output.png.size(), output.rgba.size(),
                            resident)
                        || !CheckedAdd(finalResidentBytes, resident,
                            finalResidentBytes)) return {};
                }
                if (!CheckedAdd(finalTriangleCount,
                        definition.mesh.indices.size() / 3U,
                        finalTriangleCount)
                    || !CheckedAdd(finalCornerCount,
                        definition.mesh.indices.size(),
                        finalCornerCount)) return {};
                for (const auto& receiver : owner.receivers) {
                    const Standard_Integer disposableIndex =
                        definition.faces.FindIndex(receiver.disposableFace);
                    if (disposableIndex <= 0) return {};
                    const std::uint32_t faceIndex =
                        std::uint32_t(disposableIndex - 1);
                    std::size_t primitiveIndex =
                        definition.mesh.primitives.size();
                    for (std::size_t index = 0;
                         index < definition.mesh.primitives.size(); ++index) {
                        if (definition.mesh.primitives[index].faceIndex
                                != faceIndex)
                            continue;
                        if (primitiveIndex
                                != definition.mesh.primitives.size())
                            return {};
                        primitiveIndex = index;
                    }
                    if (primitiveIndex
                            == definition.mesh.primitives.size()
                        || !FitsUInt32(primitiveIndex)) return {};
                    std::uint32_t materialIndex = 0;
                    bool foundBinding = false;
                    for (const auto& occurrence : occurrences) {
                        if (occurrence.definition != found->second) continue;
                        const auto& instance =
                            source.instances[occurrence.instance];
                        if (primitiveIndex
                                >= instance.primitiveBindings.size())
                            return {};
                        const std::uint32_t candidate =
                            instance.primitiveBindings[primitiveIndex]
                                .materialIndex;
                        if (foundBinding && candidate != materialIndex)
                            return {};
                        materialIndex = candidate;
                        foundBinding = true;
                    }
                    if (!foundBinding) return {};
                    finalCorrespondence.push_back({
                        owner.definitionIdentifier,
                        receiver.originalFaceIndex,
                        faceIndex,
                        std::uint32_t(primitiveIndex),
                        materialIndex});
                }
#ifdef DEBUG
                if (debugObservation)
                    debugObservation->failureStage =
                        "transient-correspondence-built";
#endif
            }
        }
        if (decalCapture && (finalCorrespondence.empty()
                || finalTriangleCount == 0 || finalCornerCount == 0
                || finalInputKeys.empty() || finalBakeSeals.empty()))
            return {};
#ifdef DEBUG
        if (debugObservation && decalCapture)
            debugObservation->failureStage = "final-inputs-complete";
#endif
        std::unordered_set<std::size_t> replaced;
        for (const auto& occurrence : occurrences) {
            if (cancelled()) { return {}; }
            auto& item = result->instances[occurrence.instance];
            auto& definition = definitions[occurrence.definition];
            auto& mesh = result->meshes[item.meshIndex];
            if (replaced.insert(item.meshIndex).second) {
                if (mesh.primitives.size() != definition.mesh.primitives.size()) { return {}; }
                for (std::size_t i = 0; i < mesh.primitives.size(); ++i) {
                    if (mesh.primitives[i].faceIndex != definition.mesh.primitives[i].faceIndex) { return {}; }
                }
                const auto revision = mesh.geometryRevision;
                MeshSnapshot replacement = definition.mesh;
                const auto fresh = finalDerivatives.find(
                    replacement.definitionIdentifier);
                if (fresh != finalDerivatives.end()) {
                    MeshSnapshot derivative;
                    if (!ApplyPaintedAtlasDerivative(
                            replacement, fresh->second, derivative)) return {};
                    replacement = std::move(derivative);
                } else if (occurrence.painted) {
                    MeshSnapshot derivative;
                    if (!ApplyPaintedAtlasDerivative(
                            replacement, occurrence.paintedDerivative,
                            derivative)
                        || derivative.definitionIdentifier
                            != mesh.definitionIdentifier) return {};
                    replacement = std::move(derivative);
                }
                mesh = std::move(replacement);
                mesh.geometryRevision = revision; // frozen provenance, never a live publication
            }
            gp_Trsf origin;
            origin.SetTranslation(gp_Vec(definition.sourceOrigin.x,
                definition.sourceOrigin.y, definition.sourceOrigin.z));
            const auto world = occurrence.transform.Multiplied(origin);
            if (!MatrixFromTransform(world, item.worldFromObject)) { return {}; }
            item.reversesWinding = world.IsNegative();
        }
#ifdef DEBUG
        if (debugObservation && decalCapture)
            debugObservation->failureStage = "final-mesh-install-complete";
#endif

        // Install the operation-owned painted roles before E4 publication.
        // Resource identity is the hash of the actual PNG bytes; semantic
        // roles stay distinct even when two roles legitimately share bytes.
        std::size_t aggregateEncodedBytes = 0;
        std::size_t aggregateDecodedBytes = 0;
        std::unordered_map<std::string, std::int32_t> textureIdentifiers;
        for (std::size_t index = 0; index < result->textures.size(); ++index) {
            const auto& texture = result->textures[index];
            std::size_t decoded = 0;
            if (texture.identifier.empty() || texture.encodedBytes.empty()
                || !FitsUInt32(index)
                || !textureIdentifiers.emplace(texture.identifier,
                        std::int32_t(index)).second
                || !CheckedMultiply(std::size_t(texture.pixelWidth),
                    std::size_t(texture.pixelHeight), decoded)
                || !CheckedMultiply(decoded, std::size_t(4), decoded)
                || !CheckedAdd(aggregateEncodedBytes,
                    texture.encodedBytes.size(), aggregateEncodedBytes)
                || !CheckedAdd(aggregateDecodedBytes, decoded,
                    aggregateDecodedBytes)) return {};
        }
        const auto installPNG = [&](const std::vector<std::uint8_t>& png,
                                    const std::uint32_t width,
                                    const std::uint32_t height,
                                    const bool painted,
                                    std::int32_t& textureIndex) {
            textureIndex = -1;
            face_image::Digest content{};
            std::size_t decoded = 0;
            if (png.empty() || width == 0 || height == 0
                || !face_image::HashFaceImageBytes(png, content)
                || !CheckedMultiply(std::size_t(width),
                    std::size_t(height), decoded)
                || !CheckedMultiply(decoded, std::size_t(4), decoded))
                return false;
            const std::string identifier = "texture-sha256-"
                + PaintedAtlasDigestText(content);
            const auto existing = textureIdentifiers.find(identifier);
            if (existing != textureIdentifiers.end()) {
                const auto& texture = result->textures[
                    std::size_t(existing->second)];
                if (texture.encoding != TextureEncoding::PNG
                    || texture.pixelWidth != width
                    || texture.pixelHeight != height
                    || texture.encodedBytes != png) return false;
                textureIndex = existing->second;
                return true;
            }
            if (result->textures.size() >= kMaxTexturesPerSnapshot
                || result->textures.size()
                    > std::size_t(std::numeric_limits<std::int32_t>::max())
                || !CheckedAdd(aggregateEncodedBytes, png.size(),
                    aggregateEncodedBytes)
                || !CheckedAdd(aggregateDecodedBytes, decoded,
                    aggregateDecodedBytes)
                || aggregateEncodedBytes > kMaxAggregateEncodedTextureBytes
                || aggregateDecodedBytes > kMaxAggregateDecodedTextureBytes)
                return false;
            textureIndex = std::int32_t(result->textures.size());
            TextureResourceSnapshot texture;
            texture.identifier = identifier;
            texture.encoding = TextureEncoding::PNG;
            texture.pixelWidth = width;
            texture.pixelHeight = height;
            texture.encodedBytes = png;
            texture.paintedAtlasDerivative = painted;
            result->textures.push_back(std::move(texture));
            textureIdentifiers.emplace(identifier, textureIndex);
            return true;
        };

        std::unordered_map<std::string, std::array<std::int32_t, 5>>
            roleTextures;
        for (const auto& entry : finalBakes) {
            const auto captured = capturedOwners.find(entry.first);
            if (captured == capturedOwners.end()) return {};
            std::set<face_image::Role> expectedRoles;
            for (const auto& source : captured->second->paintedSources)
                expectedRoles.insert(source.fence.role);
            std::set<face_image::Role> installedRoles;
            std::array<std::int32_t, 5> indices{-1, -1, -1, -1, -1};
            for (const auto& output : entry.second.outputs) {
                const std::size_t role = std::size_t(output.descriptor.role);
                face_image::Digest measured{};
                if (role >= indices.size() || indices[role] >= 0
                    || !installedRoles.insert(output.descriptor.role).second
                    || !face_image::HashFaceImageBytes(output.png, measured)
                    || measured != output.descriptor.content
                    || !installPNG(output.png,
                        output.descriptor.widthTexels,
                        output.descriptor.heightTexels, true,
                        indices[role])) return {};
            }
            if (expectedRoles.empty() || installedRoles != expectedRoles
                || !roleTextures.emplace(entry.first, indices).second)
                return {};
        }
#ifdef DEBUG
        if (debugObservation && decalCapture)
            debugObservation->failureStage = "painted-resources-installed";
#endif

        std::unordered_map<std::string, std::uint32_t> installedMaterials;
        std::unordered_set<std::size_t> tangentMeshes;
        for (const auto& occurrence : occurrences) {
            auto& item = result->instances[occurrence.instance];
            const auto& definition = definitions[occurrence.definition];
            const auto roles = roleTextures.find(
                definition.mesh.definitionIdentifier);
            if (roles == roleTextures.end()) continue;
            for (auto& binding : item.primitiveBindings) {
                if (binding.materialIndex >= result->materials.size()) return {};
                const std::string key = definition.mesh.definitionIdentifier
                    + ":" + std::to_string(binding.materialIndex);
                auto installed = installedMaterials.find(key);
                if (installed == installedMaterials.end()) {
                    MaterialSnapshot material =
                        result->materials[binding.materialIndex];
                    material.baseColorTextureIndex = roles->second[0];
                    material.emissiveTextureIndex = roles->second[1];
                    material.metallicRoughnessTextureIndex = roles->second[2];
                    material.occlusionTextureIndex = roles->second[3];
                    material.normalTextureIndex = roles->second[4];
                    std::uint32_t materialIndex = 0;
                    if (!AddMaterial(*result, std::move(material),
                            materialIndex)) return {};
                    installed = installedMaterials.emplace(
                        key, materialIndex).first;
                }
                binding.materialIndex = installed->second;
            }
            // The final atlas material now owns the complete painted role
            // set. Retaining the pre-remesh per-face bindings would both take
            // writer precedence and violate the painted-derivative scene
            // invariant, so remove them only after every role was installed.
            item.faceImageBindings.clear();
            if (roles->second[std::size_t(face_image::Role::Normal)] >= 0
                && tangentMeshes.insert(item.meshIndex).second) {
                auto& mesh = result->meshes[item.meshIndex];
                if (GenerateMikkCornerTangents(mesh.vertices, mesh.indices,
                        true, mesh.cornerTangents)
                        != TangentSpaceError::None) return {};
                mesh.tangentBasis = TangentBasis::MikkTSpace;
            }
        }
#ifdef DEBUG
        if (debugObservation && decalCapture)
            debugObservation->failureStage = "painted-materials-installed";
#endif

#ifdef DEBUG
        if (decalCapture
            && debugFault == DebugPrivateExportDecalFault::Publication) {
            if (debugObservation) {
                debugObservation->producedReceiverCount =
                    finalCorrespondence.size();
                debugObservation->finalTriangleCount = finalTriangleCount;
                debugObservation->finalCornerCount = finalCornerCount;
                debugObservation->operationWork += finalTriangleCount;
                debugObservation->operationResidentBytes = finalResidentBytes;
                debugObservation->correspondence = finalCorrespondence;
                debugObservation->inputKeys = finalInputKeys;
                debugObservation->bakeSeals = finalBakeSeals;
                debugObservation->outputRoles = finalOutputRoles;
                debugObservation->outputRoleBytes = finalOutputRoleBytes;
                debugObservation->layoutResolution = finalLayoutResolution;
                debugObservation->layoutChartCount = finalLayoutChartCount;
                debugObservation->finalProductionReached = true;
                debugObservation->failureStage =
                    "changed-remesh-private-publication-not-connected";
            }
            return {};
        }
#endif

        // Produce and publish every standalone occurrence into the one
        // operation-owned destination. This intentionally reuses the same
        // ProduceFace/Assemble/PublishPrivate path as ordinary publication.
        std::vector<decal_math::DecalWorldTriangle> privateOccluders;
        Bounds3d privateOccluderBounds;
        decal_layer::Digest privateOccluderProof{};
        if (decalCapture
            && !decal_math::CaptureDecalOccluders(
                decalCapture->impl_->wholeCommittedScene,
                privateOccluders, privateOccluderBounds,
                privateOccluderProof)) return {};
#ifdef DEBUG
        if (debugObservation && decalCapture)
            debugObservation->failureStage = "private-occluders-captured";
#endif
        finalInputKeys.clear();
        finalBakeSeals.clear();
        std::size_t publishedOccurrences = 0;
        for (const auto& occurrence : occurrences) {
            const auto& definitionData = definitions[occurrence.definition];
            const auto captured = capturedOwners.find(
                definitionData.mesh.definitionIdentifier);
            if (captured == capturedOwners.end()) continue;
            auto& owner = *captured->second;
            auto& item = result->instances[occurrence.instance];
            const auto& finalMesh = result->meshes[item.meshIndex];
            item.decalDerivedAppearances.clear();

            face_image::detail::Writer geometryWriter;
            geometryWriter.raw(
                reinterpret_cast<const std::uint8_t*>("E4PF"), 4);
            for (const auto& vertex : finalMesh.vertices)
                for (float value : {vertex.positionX, vertex.positionY,
                        vertex.positionZ, vertex.normalX, vertex.normalY,
                        vertex.normalZ, vertex.textureU, vertex.textureV})
                    geometryWriter.real(value);
            for (std::uint32_t index : finalMesh.indices)
                geometryWriter.integer(index, 4);
            for (double value : item.worldFromObject.values)
                geometryWriter.real(value);
            decal_layer::Digest geometryProof{};
            if (!geometryWriter.ok
                || !face_image::HashFaceImageBytes(
                    geometryWriter.bytes, geometryProof)) return {};

            const auto proof = [&](const char tag[5],
                                   decal_layer::Digest& output) {
                face_image::detail::Writer writer;
                writer.raw(reinterpret_cast<const std::uint8_t*>(tag), 4);
                writer.raw(geometryProof);
                writer.raw(owner.transientFinishing.chartProof);
                return writer.ok && face_image::HashFaceImageBytes(
                    writer.bytes, output);
            };
            decal_layer::Digest appearanceProof{}, tangentProof{};
            decal_layer::Digest occurrenceProof{};
            if (!proof("E4AP", appearanceProof)
                || !proof("E4TG", tangentProof)
                || !proof("E4OR", occurrenceProof)) return {};

            std::vector<decal_layer::bake::FaceProduct> products;
            for (const auto& receiver : owner.receivers) {
                std::vector<const decal_layer::Layer*> layers;
                for (const auto& layer : owner.definition.layers)
                    if (layer.image.role == face_image::Role::BaseColor
                        && decal_layer::bake::SameReceipt(
                            decal_layer::bake::IntentReceipt(layer),
                            receiver.receipt)) layers.push_back(&layer);
                if (layers.empty()) return {};
                constexpr std::uint32_t side = 64;
                decal_layer::bake::Input input;
                input.operationLedger =
                    decalCapture->impl_->operationLedger.view();
                input.baseColor.width = side;
                input.baseColor.height = side;
                input.baseColor.rgba.assign(
                    std::size_t(side) * side * 4, 255);
                input.canonicalLayers = owner.canonicalBytes;
                input.sourceProof = owner.source.sourceProof;
                input.finalGeometryUVProof = geometryProof;
                input.occluderProof = privateOccluderProof;
                input.effectiveAppearanceProof = appearanceProof;
                input.tangentProof = tangentProof;
                const auto bake = finalBakes.find(owner.definitionIdentifier);
                if (bake == finalBakes.end()) return {};
                input.atlasProof = bake->second.atlas.layoutProof;
                input.occurrenceProof = occurrenceProof;
                for (const auto* layer : layers) {
                    decal_layer::bake::ResolvedLayer resolved;
                    resolved.intent = *layer;
                    if (!ResolveDecalImage(decalCapture->impl_->privateDocument,
                            layer->image, resolved.image,
                            input.operationLedger)) return {};
                    if (layer->mask.present) {
                        decal_layer::bake::ResolvedImage mask;
                        if (!ResolveDecalImage(
                                decalCapture->impl_->privateDocument,
                                layer->mask.image, mask,
                                input.operationLedger)) return {};
                        resolved.mask = std::move(mask);
                    }
                    resolved.completeOccludersProved = true;
                    resolved.boundaryProved = true;
                    resolved.footprintContained = true;
                    const auto appendTriangle = [&](const std::array<
                            decal_layer::bake::Point, 3>& pixels,
                            const std::array<decal_layer::bake::UV, 3>& uvs) {
                        decal_layer::bake::Triangle triangle;
                        triangle.outputPixels = pixels;
                        triangle.layerUV = uvs;
                        triangle.receiver = receiver.receipt;
                        for (std::uint32_t y = 0; y < side; ++y)
                            for (std::uint32_t x = 0; x < side; ++x) {
                                std::array<double, 3> weights{};
                                if (!decal_layer::bake::BarycentricTopLeft(
                                        triangle, {double(x) + 0.5,
                                            double(y) + 0.5}, weights))
                                    continue;
                                decal_layer::bake::CoverageSample sample;
                                sample.x = x;
                                sample.y = y;
                                sample.receiver = receiver.receipt;
                                sample.conclusive = true;
                                sample.affected = true;
                                sample.selectedReceiver = true;
                                sample.frontFacing = true;
                                sample.insideTrim = true;
                                sample.nearestHit = true;
                                triangle.coverage.push_back(sample);
                            }
                        if (triangle.coverage.empty()) return false;
                        resolved.triangles.push_back(std::move(triangle));
                        return true;
                    };
                    if (!appendTriangle({{{0, 0}, {side, 0}, {0, side}}},
                            {{{0, 1}, {1, 1}, {0, 0}}})
                        || !appendTriangle({{{side, 0}, {side, side},
                                {0, side}}},
                            {{{1, 1}, {1, 0}, {0, 0}}})) return {};
                    input.layers.push_back(std::move(resolved));
                }
                decal_layer::bake::FaceProduct product;
                if (decal_layer::bake::ProduceFace(
                        input, receiver.receipt, product)
                        != decal_layer::bake::Status::Baked) return {};
                products.push_back(std::move(product));
            }
            face_image::detail::Writer finalGeometry;
            finalGeometry.raw(
                reinterpret_cast<const std::uint8_t*>("E4GF"), 4);
            finalGeometry.raw(geometryProof);
            for (const auto& product : products)
                finalGeometry.raw(product.inputKey);
            decal_layer::Digest finalGeometryProof{};
            if (!finalGeometry.ok
                || !face_image::HashFaceImageBytes(
                    finalGeometry.bytes, finalGeometryProof)) return {};
            decal_layer::bake::PublishedDerivative published;
            if (!decal_layer::bake::PublishPrivate(owner.definition,
                    finalGeometryProof, privateOccluderProof, products,
                    decalCapture->impl_->destination, published)) return {};
#ifdef DEBUG
            if (debugObservation)
                debugObservation->failureStage = "private-owner-published";
#endif
            decal_layer::bake::PublishedDerivative reopened;
            if (!decal_layer::bake::publication::Lookup(
                    decalCapture->impl_->destination, published.owner,
                    published.inputKey, reopened)
                || reopened.bakeKey != published.bakeKey
                || reopened.faces.size() != owner.receivers.size()) return {};
            finalInputKeys.push_back(DecalDigestText(reopened.inputKey));
            finalBakeSeals.push_back(DecalDigestText(reopened.bakeKey));
            for (const auto& face : reopened.faces) {
                const auto receiver = std::find_if(owner.receivers.begin(),
                    owner.receivers.end(), [&](const auto& candidate) {
                        return decal_layer::bake::SameReceipt(
                            candidate.receipt, face.receiver);
                    });
                if (receiver == owner.receivers.end()) return {};
                std::int32_t textureIndex = -1;
                if (!installPNG(face.png, face.pixelWidth,
                        face.pixelHeight, false, textureIndex)) return {};
                DecalDerivedAppearanceSnapshot appearance;
                appearance.ownerDefinitionIdentifier =
                    owner.definitionIdentifier;
                appearance.bakeProof = DecalDigestText(reopened.bakeKey);
                appearance.faceIndex = receiver->disposableFaceIndex;
                appearance.baseColorTextureIndex = textureIndex;
                item.decalDerivedAppearances.push_back(
                    std::move(appearance));
            }
            if (item.decalDerivedAppearances.size()
                    != owner.receivers.size()) return {};
            ++publishedOccurrences;
        }
#ifdef DEBUG
        if (debugObservation && decalCapture)
            debugObservation->failureStage = "private-appearances-installed";
#endif
        if (decalCapture && (publishedOccurrences == 0
                || decalCapture->impl_->destination.values.size()
                    != publishedOccurrences)) return {};
#ifdef DEBUG
        if (debugObservation && decalCapture)
            debugObservation->failureStage = "private-destination-complete";
#endif
        // Removing superseded face bindings can orphan their source images.
        // Compact once, after every painted role and private appearance is
        // installed, and remap all live references as one candidate update.
        std::vector<bool> referencedTextures(result->textures.size(), false);
        const auto markTexture = [&](const std::int32_t index) {
            if (index < 0) return true;
            if (std::size_t(index) >= referencedTextures.size()) return false;
            referencedTextures[std::size_t(index)] = true;
            return true;
        };
        for (const auto& material : result->materials)
            if (!markTexture(material.baseColorTextureIndex)
                || !markTexture(material.emissiveTextureIndex)
                || !markTexture(material.metallicRoughnessTextureIndex)
                || !markTexture(material.occlusionTextureIndex)
                || !markTexture(material.normalTextureIndex)) return {};
        for (const auto& instance : result->instances) {
            for (const auto& binding : instance.faceImageBindings)
                if (!markTexture(binding.textureIndex)) return {};
            for (const auto& appearance : instance.decalDerivedAppearances)
                if (!markTexture(appearance.baseColorTextureIndex)) return {};
        }
        std::vector<std::int32_t> textureRemap(
            result->textures.size(), -1);
        std::vector<TextureResourceSnapshot> compactTextures;
        compactTextures.reserve(result->textures.size());
        for (std::size_t index = 0; index < result->textures.size(); ++index) {
            if (!referencedTextures[index]) continue;
            if (compactTextures.size()
                    > std::size_t(std::numeric_limits<std::int32_t>::max()))
                return {};
            textureRemap[index] = std::int32_t(compactTextures.size());
            compactTextures.push_back(std::move(result->textures[index]));
        }
        const auto remapTexture = [&](std::int32_t& index) {
            if (index < 0) return true;
            if (std::size_t(index) >= textureRemap.size()
                || textureRemap[std::size_t(index)] < 0) return false;
            index = textureRemap[std::size_t(index)];
            return true;
        };
        for (auto& material : result->materials)
            if (!remapTexture(material.baseColorTextureIndex)
                || !remapTexture(material.emissiveTextureIndex)
                || !remapTexture(material.metallicRoughnessTextureIndex)
                || !remapTexture(material.occlusionTextureIndex)
                || !remapTexture(material.normalTextureIndex)) return {};
        for (auto& instance : result->instances) {
            for (auto& binding : instance.faceImageBindings)
                if (!remapTexture(binding.textureIndex)) return {};
            for (auto& appearance : instance.decalDerivedAppearances)
                if (!remapTexture(appearance.baseColorTextureIndex)) return {};
        }
        result->textures = std::move(compactTextures);
#ifdef DEBUG
        if (debugObservation && decalCapture)
            debugObservation->failureStage = "private-resources-compacted";
#endif
        for (const auto& mesh : result->meshes) {
            if (!CheckedAdd(totalVertices, mesh.vertices.size(), totalVertices)
                || !CheckedAdd(totalIndices, mesh.indices.size(), totalIndices)
                || totalVertices > kMaxVerticesPerSnapshot || totalIndices > kMaxIndicesPerSnapshot) { return {}; }
        }
#ifdef DEBUG
        if (debugObservation && decalCapture)
            debugObservation->failureStage = "private-topology-counted";
#endif
        if (cancelled()) return {};
        if (!IsValidSceneSnapshot(*result)) {
#ifdef DEBUG
            if (debugObservation && decalCapture) {
                std::string diagnosis = "private-candidate-invalid-other";
                std::vector<bool> referenced(result->textures.size(), false);
                const auto markTexture = [&](const std::int32_t index) {
                    if (index >= 0
                        && std::size_t(index) < referenced.size())
                        referenced[std::size_t(index)] = true;
                };
                for (const auto& material : result->materials) {
                    markTexture(material.baseColorTextureIndex);
                    markTexture(material.emissiveTextureIndex);
                    markTexture(material.metallicRoughnessTextureIndex);
                    markTexture(material.occlusionTextureIndex);
                    markTexture(material.normalTextureIndex);
                }
                bool appearanceInvalid = false;
                bool paintedMaterialInvalid = false;
                bool meshInvalid = false;
                for (const auto& instance : result->instances) {
                    if (instance.meshIndex >= result->meshes.size()) {
                        meshInvalid = true;
                        continue;
                    }
                    const auto& mesh = result->meshes[instance.meshIndex];
                    if (!mesh.paintedAtlasBakeProof.empty()
                        && (mesh.vertices.size() != mesh.indices.size()
                            || mesh.paintedAtlasMasterDefinitionIdentifier
                                .empty()
                            || !instance.faceImageBindings.empty()))
                        meshInvalid = true;
                    for (const auto& binding : instance.faceImageBindings)
                        markTexture(binding.textureIndex);
                    const std::string& appearanceOwner =
                        mesh.paintedAtlasMasterDefinitionIdentifier.empty()
                            ? mesh.definitionIdentifier
                            : mesh.paintedAtlasMasterDefinitionIdentifier;
                    for (const auto& appearance :
                            instance.decalDerivedAppearances) {
                        markTexture(appearance.baseColorTextureIndex);
                        bool facePublished = false;
                        for (const auto& primitive : mesh.primitives)
                            facePublished = facePublished
                                || primitive.faceIndex
                                    == appearance.faceIndex;
                        const bool validBakeProof =
                            appearance.bakeProof.size() == 64
                            && std::all_of(appearance.bakeProof.begin(),
                                appearance.bakeProof.end(), [](const char c) {
                                    return (c >= '0' && c <= '9')
                                        || (c >= 'a' && c <= 'f');
                                });
                        appearanceInvalid = appearanceInvalid
                            || appearance.ownerDefinitionIdentifier.empty()
                            || appearance.baseColorTextureIndex < 0
                            || !validBakeProof
                            || appearance.ownerDefinitionIdentifier
                                != appearanceOwner
                            || !facePublished;
                    }
                    if (!mesh.paintedAtlasBakeProof.empty())
                        for (const auto& binding :
                                instance.primitiveBindings) {
                            if (binding.materialIndex
                                    >= result->materials.size()) {
                                paintedMaterialInvalid = true;
                                continue;
                            }
                            const auto& material = result->materials[
                                binding.materialIndex];
                            for (const auto index : {
                                    material.baseColorTextureIndex,
                                    material.emissiveTextureIndex,
                                    material.metallicRoughnessTextureIndex,
                                    material.occlusionTextureIndex,
                                    material.normalTextureIndex})
                                if (index >= 0
                                    && (std::size_t(index)
                                            >= result->textures.size()
                                        || !result->textures[
                                            std::size_t(index)]
                                                .paintedAtlasDerivative))
                                    paintedMaterialInvalid = true;
                        }
                }
                if (appearanceInvalid)
                    diagnosis = "private-candidate-invalid-appearance";
                else if (paintedMaterialInvalid)
                    diagnosis = "private-candidate-invalid-painted-material";
                else if (meshInvalid)
                    diagnosis = "private-candidate-invalid-mesh";
                else if (std::find(referenced.begin(), referenced.end(),
                        false) != referenced.end())
                    diagnosis = "private-candidate-invalid-unreferenced";
                debugObservation->failureStage = std::move(diagnosis);
            }
#endif
            return {};
        }
#ifdef DEBUG
        if (debugObservation && decalCapture)
            debugObservation->failureStage = "private-candidate-valid";
#endif
        if (!decalCapture) return result;
        std::vector<PrivateExportFinalMesh> emittedMeshes;
        emittedMeshes.reserve(publishedOccurrences);
        for (const auto& occurrence : occurrences) {
            const auto& definition = definitions[occurrence.definition];
            if (capturedOwners.find(definition.mesh.definitionIdentifier)
                    == capturedOwners.end()) continue;
            const auto& instance = result->instances[occurrence.instance];
            if (instance.meshIndex >= result->meshes.size()) return {};
            emittedMeshes.push_back({definition.mesh.definitionIdentifier,
                result->meshes[instance.meshIndex], instance});
        }
        PrivateExportDecalArtifact artifact;
        if (!FinalizePrivateExportDecals(*decalCapture, *result,
                finalCorrespondence, emittedMeshes, cancelled, artifact)
            || !artifact.scene) return {};
#ifdef DEBUG
        if (debugObservation) {
            debugObservation->producedReceiverCount =
                finalCorrespondence.size();
            debugObservation->finalTriangleCount = finalTriangleCount;
            debugObservation->finalCornerCount = finalCornerCount;
            debugObservation->operationWork += finalTriangleCount;
            debugObservation->operationResidentBytes = finalResidentBytes;
            debugObservation->correspondence = artifact.correspondence;
            debugObservation->inputKeys = artifact.inputKeys;
            debugObservation->bakeSeals = artifact.bakeSeals;
            debugObservation->outputRoles = finalOutputRoles;
            debugObservation->outputRoleBytes = finalOutputRoleBytes;
            debugObservation->layoutResolution = finalLayoutResolution;
            debugObservation->layoutChartCount = finalLayoutChartCount;
            debugObservation->finalProductionReached = true;
            debugObservation->publicationReached = true;
            debugObservation->complete = true;
            debugObservation->failureStage = "complete";
        }
#endif
        return artifact.scene;
    } catch (...) { return {}; }
}

OcctSceneSnapshotBuilder::SnapshotPointer OcctSceneSnapshotBuilder::Build(
    const Handle(OcctDocument)& theDocument,
    const Handle(AIS_InteractiveContext)& theContext,
    const Handle(V3d_View)& theView,
    const UInt2& theViewportPixels,
    const ElementKind theAcceptedSelectionKind) noexcept
{
    if (![NSThread isMainThread]
        || theDocument.IsNull()
        || theContext.IsNull()
        || theView.IsNull()
        || myState == nullptr
        || myState->publicationSourceIdentifier.empty()
        || theViewportPixels.x == 0
        || theViewportPixels.y == 0
        || (theAcceptedSelectionKind != ElementKind::Object
            && theAcceptedSelectionKind != ElementKind::Face
            && theAcceptedSelectionKind != ElementKind::Edge)) {
        return {};
    }

    try {
        OCC_CATCH_SIGNALS

        const Handle(TDocStd_Document)& aDocument = theDocument->Document();
        if (aDocument.IsNull() || aDocument->HasOpenCommand()) {
            return {};
        }
        Standard_Size aFrameResidentBytes = 0;
        if (!Core3DValidateOwnedFrameUsage(aDocument,aFrameResidentBytes)) return {};
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

        OcctSavedGroupState savedGroups;
        if (!theDocument->CaptureSavedGroups(savedGroups)) { return {}; }
        std::unordered_map<std::string, std::pair<std::string, std::string>> groupByEntity;
        for (const auto& group : savedGroups.groups) {
            NSString* name = [[NSString alloc] initWithCharacters:
                reinterpret_cast<const unichar*>(group.name.ToExtString()) length:group.name.Length()];
            if (name == nil || name.UTF8String == nullptr) { return {}; }
            for (const auto& label : group.members) {
                const auto entity = theDocument->EntityIdentifierForLabel(label);
                if (!groupByEntity.emplace(entity, std::make_pair(group.identifier, std::string(name.UTF8String))).second) { return {}; }
            }
        }
        std::vector<OccurrenceData> anOccurrences;
        pattern_owner::RetainedPresentationIndex aRetainedPresentation;
        if (!pattern_owner::CaptureRetainedPresentationIndex(
                aDocument, aRetainedPresentation)) return {};
        std::unordered_set<std::string> anEntityIdentifiers;
        std::unordered_map<std::string, TDF_Label> aPersistentEntityLabels;
        std::unordered_map<std::string, std::size_t> aDefinitionIndices;
        std::vector<DefinitionData> aDefinitions;
#ifdef DEBUG
        DebugBoundedCurvePublicationObservation
            aDebugBoundedCurveObservation;
#endif
        std::size_t aLabelInstanceMappingCount = 0;
        std::size_t aLayerVisibilityCheckCount = 0;
        const Handle(XCAFDoc_LayerTool) aLayerTool =
            XCAFDoc_DocumentTool::CheckLayerTool(aDocument->Main())
                ? XCAFDoc_DocumentTool::LayerTool(aDocument->Main())
                : Handle(XCAFDoc_LayerTool)();

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
            // This is the label-level equivalent of
            // CafShapePrs::IsEditablePresentation(): an assembly component's
            // occurrence label differs from the definition label and cannot be
            // addressed by Shapeyard's definition-owned editing model. Keep the
            // document's existing free-simple authority as the second half of
            // the same production gate rather than inferring editability from
            // geometry representation or explorer depth.
            anOccurrence.editable =
                aNode.Label.IsEqual(aDefinitionLabel)
                && theDocument->IsEditableFreeSimpleDefinitionLabel(
                    aDefinitionLabel);
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
                const auto& aPathNode = anExplorer.Current(aDepth);
                const TDF_Label& aPathLabel = aPathNode.Label;
                if (aPathLabel.IsNull()) {
                    return {};
                }
                // Match OpenGL's occurrence, definition and ancestor visibility.
                // Explorer style alone does not include invisible XCAF layers.
                for (const TDF_Label& aVisibilityLabel :
                     {aPathNode.Label, aPathNode.RefLabel}) {
                    if (aVisibilityLabel.IsNull()) { continue; }
                    anOccurrence.visible = anOccurrence.visible
                        && XCAFDoc_ColorTool::IsVisible(aVisibilityLabel);
                    TDF_LabelSequence aLayers;
                    if (aLayerTool.IsNull()
                        || !aLayerTool->GetLayers(aVisibilityLabel, aLayers)) {
                        continue;
                    }
                    if (!CheckedAdd(aLayerVisibilityCheckCount,
                                    static_cast<std::size_t>(aLayers.Length()),
                                    aLayerVisibilityCheckCount)
                        || aLayerVisibilityCheckCount > kMaxLabelInstanceMappings) {
                        return {};
                    }
                    for (TDF_LabelSequence::Iterator aLayer(aLayers);
                         aLayer.More(); aLayer.Next()) {
                        anOccurrence.visible = anOccurrence.visible
                            && aLayerTool->IsVisible(aLayer.Value());
                    }
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
            const auto aRetainedState =
                pattern_owner::StateForEntityIdentifier(
                    aRetainedPresentation,
                    theDocument->EntityIdentifierForLabel(aDefinitionLabel));
            anOccurrence.visible = pattern_owner::EffectiveVisibility(
                anOccurrence.visible, true, aRetainedState);
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
            std::size_t aSnapshotFaceMapEntryCount = 0;
            std::size_t aSnapshotTopologyMapEntryCount = 0;
            for (auto& [aDefinitionIdentifier, aDefinitionIndex] :
                 aDefinitionIndices) {
                DefinitionData& aDefinition =
                    aDefinitions[aDefinitionIndex];
                if (!(ExtractBoundedCurveDefinition(
                          theDocument, aDefinition.label,
                          aDefinitionIdentifier, aDefinition
#ifdef DEBUG
                          , gDebugBoundedCurveObservationArmed
                              ? &aDebugBoundedCurveObservation : nullptr
#endif
                          )
                      || ExtractDefinitionGeometry(
                        aDefinition.label,
                        aDefinitionIdentifier,
#ifdef DEBUG
                        static_cast<std::uint8_t>(
                            myState->debugTriangulationFailure),
#endif
                        aDefinition))) {
                    return {};
                }
                const std::size_t aFaceMapEntryCount =
                    static_cast<std::size_t>(aDefinition.faces.Extent());
                const std::size_t anEdgeMapEntryCount =
                    static_cast<std::size_t>(aDefinition.edges.Extent());
                const std::size_t aVertexMapEntryCount =
                    static_cast<std::size_t>(aDefinition.vertices.Extent());
                std::size_t aDefinitionTopologyMapEntryCount = 0;
                if (!CheckedAdd(aFaceMapEntryCount,
                                anEdgeMapEntryCount,
                                aDefinitionTopologyMapEntryCount)
                    || !CheckedAdd(aDefinitionTopologyMapEntryCount,
                                   aVertexMapEntryCount,
                                   aDefinitionTopologyMapEntryCount)
                    || !CheckedAdd(aSnapshotFaceMapEntryCount,
                                   aFaceMapEntryCount,
                                   aSnapshotFaceMapEntryCount)
                    || aSnapshotFaceMapEntryCount
                        > kMaxPrimitiveBindingsPerSnapshot
                    || !CheckedAdd(aSnapshotTopologyMapEntryCount,
                                   aDefinitionTopologyMapEntryCount,
                                   aSnapshotTopologyMapEntryCount)
                    || aSnapshotTopologyMapEntryCount
                        > kMaxTopologyMapEntriesPerSnapshot) {
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
            aNextState.lastFullNativeContactSources.reset();
            aNextState.lastFullSnapshotRevision = 0;
            aNextState.lastFullDocumentTime = 0;
        }

        SceneSnapshot aScene;
        aScene.publicationSourceIdentifier =
            aNextState.publicationSourceIdentifier;
        aScene.metersPerUnit = aMetersPerUnit;
        aScene.selectionMode = theAcceptedSelectionKind;
        TextureTableState aTextureTable;
        // At most one current painted-atlas derivative is published per
        // occurrence. Masters stay resident and untouched for stale fallback.
        aScene.meshes.reserve(aDefinitions.size() + anOccurrences.size());
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
        std::vector<std::size_t> aMeshDefinitionIndices;
        aMeshDefinitionIndices.reserve(
            aDefinitions.size() + anOccurrences.size());
        for (std::size_t index = 0; index < aDefinitions.size(); ++index)
            aMeshDefinitionIndices.push_back(index);
        if (aMeshDefinitionIndices.size() != aScene.meshes.size()) return {};
        struct PendingDecal final {
            std::size_t instanceIndex = 0;
            std::size_t definitionIndex = 0;
            TDF_Label ownerLabel;
            core3d::decal_layer::Definition definition;
            std::vector<std::uint8_t> canonicalBytes;
            core3d::decal_layer::source::Witness source;
            std::shared_ptr<core3d::retained_edge_treatment::ReplayBudget>
                budget;
        };
        std::vector<PendingDecal> aPendingDecals;
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

            core3d::retained_recipe::OwnerKey anAppearanceOwner;
            if (!core3d::receipt::ParseUUID(aDocumentIdentifier,
                                            anAppearanceOwner.document)
                || !core3d::receipt::ParseUUID(
                    theDocument->EntityIdentifierForLabel(
                        anOccurrence.definitionLabel),
                    anAppearanceOwner.entity)
                || !core3d::receipt::ParseUUID(
                    anOccurrence.definitionIdentifier,
                    anAppearanceOwner.definition)
                || !core3d::retained_recipe::Valid(anAppearanceOwner)) {
                return {};
            }
            OcctPaintedAtlasDerivative aPaintedDerivative;
            const bool hasPaintedDerivative =
                theDocument->PaintedAtlasDerivativeForOwner(
                    anAppearanceOwner, aPaintedDerivative);

            // E4 records are read directly under the document's existing
            // snapshot read ownership. The record is descriptive only: it
            // cannot mint a receiver/resource grant. Until the owner has
            // installed a current final-UV derivative on this publication,
            // presence is fail-closed below rather than silently showing an
            // undecorated or stale model.
            core3d::decal_layer::Definition aDecalRecord;
            std::vector<std::uint8_t> aDecalCanonicalBytes;
            const auto aDecalState = core3d::decal_layer::persistence::Read(
                theDocument->Document(), anOccurrence.definitionLabel,
                aDecalRecord, &aDecalCanonicalBytes, nullptr);
            if (aDecalState
                == core3d::decal_layer::persistence::ReadState::Malformed) {
                return {};
            }
            std::optional<PendingDecal> aPendingDecal;
            if (aDecalState
                == core3d::decal_layer::persistence::ReadState::Present) {
                core3d::decal_layer::Definition aProofCheck = aDecalRecord;
                const auto aSavedLayerProof = aProofCheck.layerProof;
                auto aBudget = std::make_shared<
                    core3d::retained_edge_treatment::ReplayBudget>();
                core3d::decal_layer::source::Witness aFreshSource;
                if (!(aDecalRecord.owner == anAppearanceOwner)
                    || !core3d::decal_layer::BindLayerProof(aProofCheck)
                    || aProofCheck.layerProof != aSavedLayerProof
                    || !aBudget
                    || !core3d::decal_layer::source::CaptureSource(
                        theDocument, anAppearanceOwner, *aBudget,
                        [] { return false; }, aFreshSource)
                    || !core3d::decal_layer::source::ExactMatch(
                        aDecalRecord, aFreshSource)) return {};
                aPendingDecal = PendingDecal{
                    0,
                    aDefinitionIndex,
                    anOccurrence.definitionLabel,
                    aDecalRecord,
                    std::move(aDecalCanonicalBytes),
                    aFreshSource,
                    std::move(aBudget),
                };
            }

            // Resolve reference authority against the true object/occurrence
            // transform before the mesh-centering translation is appended.
            OcctReferenceAxis aStoredReferenceAxis;
            const OcctReferenceAxisReadState aReferenceState =
                theDocument->ReadReferenceAxisForLabel(
                    anOccurrence.definitionLabel, aStoredReferenceAxis);
            gp_Ax1 aWorldReferenceAxis;
            ReferenceAxisSnapshot aReferenceAxis;
            if (aReferenceState == OcctReferenceAxisReadState::Invalid
                || !theDocument->ResolveReferenceAxisInWorld(
                    anOccurrence.definitionLabel,
                    anOccurrence.occurrenceLocation,
                    aWorldReferenceAxis)
                || !ReferenceSpaceFromOcct(
                    aStoredReferenceAxis.pivotSpace,
                    aReferenceAxis.pivotSpace)
                || !ReferenceSpaceFromOcct(
                    aStoredReferenceAxis.directionSpace,
                    aReferenceAxis.directionSpace)) {
                return {};
            }
            aReferenceAxis.worldPivot = {
                aWorldReferenceAxis.Location().X(),
                aWorldReferenceAxis.Location().Y(),
                aWorldReferenceAxis.Location().Z(),
            };
            aReferenceAxis.worldDirection = {
                aWorldReferenceAxis.Direction().X(),
                aWorldReferenceAxis.Direction().Y(),
                aWorldReferenceAxis.Direction().Z(),
            };
            aReferenceAxis.authored =
                aReferenceState == OcctReferenceAxisReadState::Authored;

            gp_Trsf anObjectTransform;
            if (!theDocument->TryObjectTransformForLabel(
                    anOccurrence.definitionLabel, anObjectTransform)) {
                return {};
            }
            gp_Trsf aWorldTransform = anObjectTransform.Multiplied(
                anOccurrence.occurrenceLocation.Transformation());
            gp_Trsf aMeshOrigin;
            aMeshOrigin.SetTranslation(gp_Vec(aDefinition.sourceOrigin.x,
                                               aDefinition.sourceOrigin.y,
                                               aDefinition.sourceOrigin.z));
            aWorldTransform.Multiply(aMeshOrigin);

            InstanceSnapshot anInstance;
            anInstance.entityIdentifier = anOccurrence.entityIdentifier;
            anInstance.meshIndex = static_cast<std::uint32_t>(aDefinitionIndex);
            if (hasPaintedDerivative) {
                MeshSnapshot aDerivativeMesh;
                if (!ApplyPaintedAtlasDerivative(
                        aMesh, aPaintedDerivative, aDerivativeMesh)
                    || !FitsUInt32(aScene.meshes.size())) return {};
                std::size_t aVertexBytes = 0;
                std::size_t anIndexBytes = 0;
                std::size_t aPrimitiveBytes = 0;
                std::size_t aMeshBytes = 0;
                if (!CheckedMultiply(aDerivativeMesh.vertices.size(), sizeof(Vertex),
                                     aVertexBytes)
                    || !CheckedMultiply(aDerivativeMesh.indices.size(),
                                        sizeof(std::uint32_t), anIndexBytes)
                    || !CheckedMultiply(aDerivativeMesh.primitives.size(),
                                        sizeof(MeshPrimitive), aPrimitiveBytes)
                    || !CheckedAdd(aVertexBytes, anIndexBytes, aMeshBytes)
                    || !CheckedAdd(aMeshBytes, aPrimitiveBytes, aMeshBytes)
                    || !CheckedAdd(aSnapshotNumericBytes, aMeshBytes,
                                   aSnapshotNumericBytes)
                    || !CheckedAdd(aSnapshotVertexCount,
                                   aDerivativeMesh.vertices.size(),
                                   aSnapshotVertexCount)
                    || !CheckedAdd(aSnapshotIndexCount,
                                   aDerivativeMesh.indices.size(),
                                   aSnapshotIndexCount)
                    || aSnapshotNumericBytes > kMaxSnapshotNumericBytes
                    || aSnapshotVertexCount > kMaxVerticesPerSnapshot
                    || aSnapshotIndexCount > kMaxIndicesPerSnapshot) return {};
                anInstance.meshIndex = static_cast<std::uint32_t>(
                    aScene.meshes.size());
                aMeshDefinitionIndices.push_back(aDefinitionIndex);
                aScene.meshes.push_back(std::move(aDerivativeMesh));
            }
            if (!MatrixFromTransform(aWorldTransform, anInstance.worldFromObject)) {
                return {};
            }
            anInstance.referenceAxis = aReferenceAxis;
            anInstance.reversesWinding = aWorldTransform.IsNegative();
            anInstance.visible = anOccurrence.visible;
            anInstance.selectable = anOccurrence.visible
                && anOccurrence.editable
                && (theAcceptedSelectionKind == ElementKind::Object
                    || (theAcceptedSelectionKind == ElementKind::Face
                        && aMesh.topology.faceCount != 0)
                    || (theAcceptedSelectionKind == ElementKind::Edge
                        && aMesh.topology.edgeCount != 0));
            anInstance.name = anOccurrence.name;
            const auto group = groupByEntity.find(anInstance.entityIdentifier);
            if (group != groupByEntity.end()) {
                anInstance.groupIdentifier = group->second.first;
                anInstance.groupName = group->second.second;
            }
            anInstance.role = RenderRole::Model;

            std::array<std::int32_t, 5> aPaintedTextureIndices{
                -1, -1, -1, -1, -1};
            if (hasPaintedDerivative) {
                if (aPaintedDerivative.resources.size()
                    != aPaintedDerivative.bake.resources.size()) return {};
                for (std::size_t anIndex = 0;
                     anIndex < aPaintedDerivative.resources.size(); ++anIndex) {
                    const auto aRole = static_cast<std::size_t>(
                        aPaintedDerivative.bake.resources[anIndex].role);
                    if (aRole >= aPaintedTextureIndices.size()
                        || aPaintedTextureIndices[aRole] >= 0
                        || !AddFaceImageTextureResource(
                            aScene, aTextureTable,
                            aPaintedDerivative.resources[anIndex],
                            aPaintedTextureIndices[aRole], true)
                        || aPaintedTextureIndices[aRole] < 0) return {};
                }
            }

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
                if (!aPbrMaterial.NormalTexture.IsNull()
                    && !Core3DValidateNormalTextureBinding(aDocument, anOccurrence.definitionLabel)) return {};
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
            if (aDefinition.boundedCurve) {
                if (aMesh.primitives.size() != 1 || aPbrOverride.has_value()) return {};
                MaterialSnapshot aMaterial = DefaultMaterial(false);
                if (aMaterialOverride.has_value()
                    && !ApplyPreset(aMaterial, *aMaterialOverride, false)) return {};
                if (aColorOverride.has_value()) {
                    if (*aColorOverride < Quantity_NOC_BLACK
                        || *aColorOverride > Quantity_NOC_WHITE) return {};
                    SetColor(aMaterial, Quantity_Color(*aColorOverride),
                             aMaterial.baseColor.w);
                }
                if (!ValidateMaterial(aMaterial)) return {};
                aFaceMaterials[0] = std::move(aMaterial);
            }
            std::unordered_map<std::uint32_t, std::size_t> aPrimitiveByFace;
            for (std::size_t aPrimitiveIndex = 0;
                 aPrimitiveIndex < aMesh.primitives.size(); ++aPrimitiveIndex) {
                aPrimitiveByFace.emplace(aMesh.primitives[aPrimitiveIndex].faceIndex,
                                         aPrimitiveIndex);
            }
            if (!aDefinition.boundedCurve) {
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
                Handle(Image_Texture) aMetallicRoughnessTexture;
                Handle(Image_Texture) anOcclusionTexture;
                Handle(Image_Texture) aNormalTexture;
                if (!ResolveMaterial(aFace,
                                     aMaterialOverride,
                                     aColorOverride,
                                     aPbrOverride,
                                     aDefinition.closed,
                                     aMaterial,
                                     aBaseColorTexture,
                                     anEmissiveTexture,
                                     aMetallicRoughnessTexture,
                                     anOcclusionTexture,
                                     aNormalTexture)
                    || (!hasPaintedDerivative && !AddTextureResource(
                        aScene,
                        aTextureTable,
                        aBaseColorTexture,
                        aMaterial.baseColorTextureIndex))
                    || (!hasPaintedDerivative && !AddTextureResource(
                        aScene,
                        aTextureTable,
                        anEmissiveTexture,
                        aMaterial.emissiveTextureIndex))
                    || (!hasPaintedDerivative && !AddTextureResource(
                        aScene, aTextureTable, aMetallicRoughnessTexture,
                        aMaterial.metallicRoughnessTextureIndex))
                    || (!hasPaintedDerivative && !AddTextureResource(
                        aScene, aTextureTable, anOcclusionTexture,
                        aMaterial.occlusionTextureIndex))
                    || (!hasPaintedDerivative && !AddTextureResource(
                        aScene, aTextureTable, aNormalTexture,
                        aMaterial.normalTextureIndex))) {
                    return {};
                }
                for (const auto& binding : {
                         std::make_pair(aMaterial.metallicRoughnessTextureIndex, aMetallicRoughnessTexture),
                         std::make_pair(aMaterial.occlusionTextureIndex, anOcclusionTexture),
                         std::make_pair(aMaterial.normalTextureIndex, aNormalTexture)}) {
                    if (!hasPaintedDerivative && binding.first >= 0
                        && aTextureTable.validatedNumericResources.insert(binding.first).second
                        && !Core3DValidateNumericTexture(binding.second)) return {};
                }
                aFaceMaterials[aPrimitiveFound->second] = std::move(aMaterial);
              }
            }

            if (hasPaintedDerivative) {
                for (auto& aMaterialValue : aFaceMaterials) {
                    if (!aMaterialValue.has_value()) return {};
                    MaterialSnapshot& aMaterial = *aMaterialValue;
                    aMaterial.baseColorTextureIndex = aPaintedTextureIndices[0];
                    aMaterial.emissiveTextureIndex = aPaintedTextureIndices[1];
                    aMaterial.metallicRoughnessTextureIndex =
                        aPaintedTextureIndices[2];
                    aMaterial.occlusionTextureIndex = aPaintedTextureIndices[3];
                    aMaterial.normalTextureIndex = aPaintedTextureIndices[4];
                }
            }

            // E3 face-image appearance (278b portion 4). The committed
            // binding/resource records of this owner's retained solid feed
            // the immutable snapshot. The durable (face UUID, selectorProof)
            // pair is matched against portion 3's exact receipt derivation;
            // the resulting publication ordinal is a transient association,
            // never persisted authority. A stale or ambiguous association
            // and a missing resource publish nothing for that binding; the
            // record itself is never touched. Assets without face-image
            // records take none of these branches.
            if (!hasPaintedDerivative && !aDefinition.boundedCurve
                && aMesh.topology.faceCount != 0
                && !aDefinition.shape.IsNull()) {
                const core3d::face_image::OwnerKey& aFaceImageOwner =
                    anAppearanceOwner;
                {
                    core3d::face_image::Definition aFaceImageRecord;
                    const auto aRecordState =
                        theDocument->ReadFaceImageBindings(
                            aFaceImageOwner, aFaceImageRecord, nullptr);
                    if (aRecordState == core3d::face_image::persistence::
                        bindings::ReadState::Malformed) {
                        return {};
                    }
                    if (aRecordState == core3d::face_image::persistence::
                        bindings::ReadState::Present
                        && aDefinition.faces.Extent() <= 4096) {
                        Bnd_Box aStageBox;
                        if (!core3d::dependent_replay::detail::
                                FaceImageStageBounds(
                                    aDefinition.shape, aStageBox)) {
                            return {};
                        }
                        if (aStageBox.IsVoid()) {
                            return {};
                        }
                        namespace dr = core3d::dependent_replay;
                        core3d::retained_edge_treatment::ReplayBudget
                            aReplayBudget;
                        const std::size_t aFaceCount =
                            static_cast<std::size_t>(
                                aDefinition.faces.Extent());
                        std::vector<core3d::face_image::Digest>
                            aFaceProofs(aFaceCount);
                        std::vector<bool> aFaceAddressable(aFaceCount, false);
                        bool aBudgetExhausted = false;
                        for (std::size_t aFaceOrdinal = 0;
                             aFaceOrdinal < aFaceCount; ++aFaceOrdinal) {
                            const TopoDS_Face aFace = TopoDS::Face(
                                aDefinition.faces.FindKey(
                                    static_cast<Standard_Integer>(
                                        aFaceOrdinal + 1)));
                            dr::FaceImageGeometricReceipt aReceipt;
                            const auto aDeriveStatus =
                                dr::detail::DeriveFaceImageReceipt(
                                    aStageBox, aFace, aMetersPerUnit,
                                    aReplayBudget, aReceipt);
                            if (aDeriveStatus == dr::detail::
                                FaceImageDeriveStatus::Budget) {
                                aBudgetExhausted = true;
                                break;
                            }
                            if (aDeriveStatus != dr::detail::
                                FaceImageDeriveStatus::Derived) {
                                continue;
                            }
                            if (!dr::FaceImageReceiptProof(
                                    aReceipt, aFaceProofs[aFaceOrdinal])) {
                                return {};
                            }
                            aFaceAddressable[aFaceOrdinal] = true;
                        }
                        if (!aBudgetExhausted) {
                            for (const auto& aCommitted :
                                 aFaceImageRecord.bindings) {
                                std::size_t aMatch = 0;
                                std::size_t aMatchCount = 0;
                                for (std::size_t aFaceOrdinal = 0;
                                     aFaceOrdinal < aFaceCount;
                                     ++aFaceOrdinal) {
                                    if (aFaceAddressable[aFaceOrdinal]
                                        && aFaceProofs[aFaceOrdinal]
                                            == aCommitted.selectorProof) {
                                        if (aMatchCount == 0) {
                                            aMatch = aFaceOrdinal;
                                        }
                                        ++aMatchCount;
                                    }
                                }
                                if (aMatchCount != 1) {
                                    continue;
                                }
                                core3d::face_image::ResourceEnvelope
                                    aEnvelope;
                                if (!theDocument->ReadFaceImageResource(
                                        aCommitted.resource, aEnvelope)) {
                                    continue;
                                }
                                std::int32_t aTextureIndex = -1;
                                if (!AddFaceImageTextureResource(
                                        aScene, aTextureTable, aEnvelope,
                                        aTextureIndex)
                                    || aTextureIndex < 0) {
                                    return {};
                                }
                                FaceImageBindingSnapshot aFaceBinding;
                                aFaceBinding.bindingIdentifier =
                                    FaceImageUUIDText(aCommitted.binding);
                                aFaceBinding.faceIdentifier =
                                    FaceImageUUIDText(aCommitted.face);
                                aFaceBinding.resourceIdentifier =
                                    FaceImageUUIDText(aCommitted.resource);
                                aFaceBinding.role = static_cast<FaceImageRole>(
                                    aCommitted.role);
                                aFaceBinding.srgbColorSpace =
                                    aCommitted.colorSpace
                                        == core3d::face_image::
                                            ColorSpace::SRGB;
                                aFaceBinding.transform.scaleU =
                                    aCommitted.transform.scale[0];
                                aFaceBinding.transform.scaleV =
                                    aCommitted.transform.scale[1];
                                aFaceBinding.transform.offsetU =
                                    aCommitted.transform.offset[0];
                                aFaceBinding.transform.offsetV =
                                    aCommitted.transform.offset[1];
                                aFaceBinding.transform.rotationDegrees =
                                    aCommitted.transform.rotationDegrees;
                                aFaceBinding.transform.wrapU =
                                    static_cast<FaceImageWrap>(
                                        aCommitted.transform.wrapU);
                                aFaceBinding.transform.wrapV =
                                    static_cast<FaceImageWrap>(
                                        aCommitted.transform.wrapV);
                                aFaceBinding.faceIndex =
                                    static_cast<std::uint32_t>(aMatch);
                                aFaceBinding.textureIndex = aTextureIndex;
                                anInstance.faceImageBindings.push_back(
                                    std::move(aFaceBinding));
                            }
                        }
                    }
                }
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
                const bool publishesObjectPick =
                    theAcceptedSelectionKind == ElementKind::Object;
                const bool publishesFacePick =
                    theAcceptedSelectionKind == ElementKind::Face
                    && aMesh.topology.faceCount != 0;
                if (anInstance.selectable && aBinding.visible
                    && (publishesObjectPick || publishesFacePick)) {
                    if (publishesFacePick
                        || !anObjectPickToken.has_value()) {
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
                            theAcceptedSelectionKind,
                            publishesObjectPick
                                ? 0U
                                : aMesh.primitives[aPrimitiveIndex].faceIndex,
                            aMesh.geometryRevision,
                        });
                        if (publishesObjectPick) {
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
            if (aPendingDecal.has_value()) {
                aPendingDecal->instanceIndex = aScene.instances.size() - 1;
                aPendingDecals.push_back(std::move(*aPendingDecal));
            }
        }

        // Supplied geometry frames persist independently of normal material.
        // Mikk-only normal bindings share the existing resident frame budget.
        if (aMeshDefinitionIndices.size() != aScene.meshes.size()) return {};
        std::vector<bool> needsNormalFrames(aScene.meshes.size(), false);
        for (const auto& instance : aScene.instances) {
            for (const auto& binding : instance.primitiveBindings) {
                if (aScene.materials[binding.materialIndex].normalTextureIndex >= 0)
                    needsNormalFrames[instance.meshIndex] = true;
            }
        }
        for (std::size_t index = 0; index < aScene.meshes.size(); ++index) {
            auto& mesh = aScene.meshes[index];
            const std::size_t definitionIndex = aMeshDefinitionIndices[index];
            if (definitionIndex >= aDefinitions.size()) return {};
            const TDF_Label& definitionLabel =
                aDefinitions[definitionIndex].label;
            OcctAuthoredFrameRecord record;
            const auto state = Core3DReadAuthoredFrameOwner(
                aDocument, definitionLabel, record);
            if (state == OcctAuthoredFrameReadState::Invalid) return {};
            const bool authored = state == OcctAuthoredFrameReadState::Authored;
            if (!authored && !needsNormalFrames[index]) continue;
            // Bound recipes were checked against native ownership above. Supplied
            // frames keep their exact archive identity with or without a map.
            if (authored && needsNormalFrames[index] && !Core3DValidateNormalTextureBinding(
                    aDocument, definitionLabel)) return {};
            std::size_t frameBytes = 0;
            if (!CheckedMultiply(mesh.indices.size(), sizeof(Float4), frameBytes)
                || !CheckedAdd(aSnapshotNumericBytes, frameBytes, aSnapshotNumericBytes)
                || aSnapshotNumericBytes > kMaxSnapshotNumericBytes) return {};
            if (authored) {
                if (!PublishAuthoredFrames(
                        aDefinitions[definitionIndex], record, mesh)) return {};
            } else {
                std::size_t nativeBytes = 0;
                // Owned Mikk bindings were included in the document-wide scan;
                // reserve only legacy unowned normal derivatives here.
                if ((Core3DNormalTextureRecipeForLabel(definitionLabel) != 1
                        && !CheckedMultiply(mesh.indices.size(),64U,nativeBytes))
                    || aFrameResidentBytes > 64U * 1024U * 1024U
                    || nativeBytes > 64U * 1024U * 1024U - aFrameResidentBytes
                    || GenerateMikkCornerTangents(mesh.vertices, mesh.indices,
                    std::all_of(mesh.primitives.begin(), mesh.primitives.end(),
                        [](const auto& primitive) { return primitive.hasTextureCoordinates; }),
                    mesh.cornerTangents) != TangentSpaceError::None) return {};
                aFrameResidentBytes += nativeBytes;
                mesh.tangentBasis = TangentBasis::MikkTSpace;
            }
        }

        // E4 production is deliberately after final tangent/UV/material
        // choices. Reconstruct every input from the current document and the
        // complete committed occurrence census; a cache is only an optional
        // result of this path, never a prerequisite for cold publication.
        if (!aPendingDecals.empty()) {
            namespace dl = core3d::decal_layer;
            namespace dr = core3d::dependent_replay;
            using namespace decal_math;
            auto anOrdinaryOperation =
                dl::bake::accounting::MakeOperationOwner();
            if (!anOrdinaryOperation.valid()) return {};
            std::vector<DecalWorldTriangle> anOccluders;
            Bounds3d anOccluderBounds;
            dl::Digest anOccluderProof{};
            if (!CaptureDecalOccluders(aScene, anOccluders,
                    anOccluderBounds, anOccluderProof)) return {};
            const Double3 aBoundsSpan = Subtract(
                anOccluderBounds.maximum, anOccluderBounds.minimum);
            const double anOccluderSpan = std::sqrt(Dot(
                aBoundsSpan, aBoundsSpan));
            if (!IsFinite(anOccluderSpan) || anOccluderSpan <= 0.0)
                return {};

            for (PendingDecal& aPending : aPendingDecals) {
                if (aPending.instanceIndex >= aScene.instances.size()
                    || aPending.definitionIndex >= aDefinitions.size()
                    || !aPending.budget) return {};
                InstanceSnapshot& anInstance =
                    aScene.instances[aPending.instanceIndex];
                if (anInstance.meshIndex >= aScene.meshes.size()) return {};
                const MeshSnapshot& aMesh = aScene.meshes[anInstance.meshIndex];
                const DefinitionData& aNative =
                    aDefinitions[aPending.definitionIndex];
                if (aNative.boundedCurve || aNative.shape.IsNull()
                    || aNative.faces.Extent() <= 0
                    || aNative.faces.Extent() > 4096
                    || !aPending.budget->visit(anOccluders.size(),
                        core3d::retained_topology_budget::Site::C05OwnerScan))
                    return {};
                Bnd_Box aStageBox;
                if (!dr::detail::FaceImageStageBounds(
                        aNative.shape, aStageBox)
                    || aStageBox.IsVoid()) return {};
                struct ResolvedFace final {
                    TopoDS_Face face;
                    dr::FaceImageGeometricReceipt receipt;
                    dl::Digest proof{};
                    bool addressable = false;
                };
                std::vector<ResolvedFace> aResolvedFaces(
                    static_cast<std::size_t>(aNative.faces.Extent()));
                for (std::size_t anOrdinal = 0;
                     anOrdinal < aResolvedFaces.size(); ++anOrdinal) {
                    ResolvedFace& aResolved = aResolvedFaces[anOrdinal];
                    aResolved.face = TopoDS::Face(aNative.faces.FindKey(
                        Standard_Integer(anOrdinal + 1)));
                    const auto aStatus = dr::detail::DeriveFaceImageReceipt(
                        aStageBox, aResolved.face, aMetersPerUnit,
                        *aPending.budget, aResolved.receipt);
                    if (aStatus == dr::detail::FaceImageDeriveStatus::Budget)
                        return {};
                    if (aStatus == dr::detail::FaceImageDeriveStatus::NotAddressable)
                        continue;
                    if (!dr::FaceImageReceiptProof(
                            aResolved.receipt, aResolved.proof)) return {};
                    aResolved.addressable = true;
                }
                std::map<std::size_t, std::vector<std::size_t>> aLayersByFace;
                for (std::size_t aLayerIndex = 0;
                     aLayerIndex < aPending.definition.layers.size();
                     ++aLayerIndex) {
                    // The committed-scene decal derivative is a base-colour
                    // preview. Other authored roles remain in the canonical
                    // capture for the detached all-role export bake below;
                    // feeding them to the base-colour compositor is invalid.
                    if (aPending.definition.layers[aLayerIndex].image.role
                            != core3d::face_image::Role::BaseColor)
                        continue;
                    const dl::ReceiverReceipt anExpected =
                        dl::bake::IntentReceipt(
                            aPending.definition.layers[aLayerIndex]);
                    std::size_t aMatch = 0, aMatches = 0;
                    for (std::size_t anOrdinal = 0;
                         anOrdinal < aResolvedFaces.size(); ++anOrdinal) {
                        if (aResolvedFaces[anOrdinal].addressable
                            && aResolvedFaces[anOrdinal].proof
                                == anExpected.selectorProof) {
                            aMatch = anOrdinal; ++aMatches;
                        }
                    }
                    if (aMatches != 1) return {};
                    aLayersByFace[aMatch].push_back(aLayerIndex);
                }

                std::vector<dl::bake::FaceProduct> aProducts;
                aProducts.reserve(aLayersByFace.size());
                for (const auto& aFaceLayers : aLayersByFace) {
                    const std::size_t aFaceIndex = aFaceLayers.first;
                    if (aFaceIndex >= aResolvedFaces.size()) return {};
                    const auto aPrimitive = std::find_if(
                        aMesh.primitives.begin(), aMesh.primitives.end(),
                        [aFaceIndex](const MeshPrimitive& thePrimitive) {
                            return thePrimitive.faceIndex == aFaceIndex;
                        });
                    if (aPrimitive == aMesh.primitives.end()
                        || !aPrimitive->hasTextureCoordinates) return {};
                    const std::size_t aPrimitiveIndex =
                        static_cast<std::size_t>(
                            aPrimitive - aMesh.primitives.begin());
                    if (aPrimitiveIndex >= anInstance.primitiveBindings.size())
                        return {};
                    const std::uint32_t aMaterialIndex =
                        anInstance.primitiveBindings[aPrimitiveIndex].materialIndex;
                    if (aMaterialIndex >= aScene.materials.size()) return {};
                    const MaterialSnapshot& aMaterial =
                        aScene.materials[aMaterialIndex];

                    dl::bake::Input anInput;
                    dl::ReceiverReceipt aReceiver;
                    const OrdinaryDecalFaceInputs anOrdinaryInputs{
                        theDocument,
                        aResolvedFaces[aFaceIndex].face,
                        aResolvedFaces[aFaceIndex].receipt,
                        aMesh.vertices,
                        aMesh.indices,
                        aMesh.primitives,
                        *aPrimitive,
                        aMesh.tangentBasis,
                        aMesh.cornerTangents,
                        aNative.sourceOrigin,
                        anInstance.worldFromObject,
                        anInstance.reversesWinding,
                        aPending.instanceIndex,
                        aFaceIndex,
                        aMaterial,
                        anInstance.faceImageBindings,
                        aScene.textures,
                        aPending.definition,
                        aFaceLayers.second,
                        anOccluders,
                        anOccluderSpan,
                        anOccluderProof,
                        aPending.canonicalBytes,
                        aPending.source.sourceProof,
                        aMetersPerUnit,
                        anOrdinaryOperation.view,
                        *aPending.budget,
                    };
                    if (!PrepareOrdinaryDecalFace(
                            anOrdinaryInputs, anInput, aReceiver)) return {};
                    dl::bake::FaceProduct aProduct;
                    if (dl::bake::ProduceFace(
                            anInput, aReceiver, aProduct)
                        != dl::bake::Status::Baked) return {};
                    aProducts.push_back(std::move(aProduct));
                }

                core3d::face_image::detail::Writer aFinalGeometry;
                aFinalGeometry.raw(reinterpret_cast<const std::uint8_t*>("E4GF"), 4);
                for (const dl::bake::FaceProduct& aProduct : aProducts)
                    aFinalGeometry.raw(aProduct.inputKey);
                dl::Digest aFinalGeometryProof{};
                if (!aFinalGeometry.ok
                    || !core3d::face_image::HashFaceImageBytes(
                        aFinalGeometry.bytes, aFinalGeometryProof)) return {};
                dl::bake::PublishedDerivative aPublished;
                if (!dl::bake::Publish(aPending.definition,
                        aFinalGeometryProof, anOccluderProof,
                        aProducts, aPublished)) return {};
                dl::bake::PublishedDerivative aReopened;
                if (!dl::bake::publication::Lookup(aPending.definition.owner,
                        aPublished.inputKey, aReopened)
                    || aReopened.bakeKey != aPublished.bakeKey) return {};

                for (const dl::bake::PublishedFace& aFace : aPublished.faces) {
                    std::size_t aMatch = 0, aMatches = 0;
                    for (std::size_t anOrdinal = 0;
                         anOrdinal < aResolvedFaces.size(); ++anOrdinal)
                        if (aResolvedFaces[anOrdinal].addressable
                            && aResolvedFaces[anOrdinal].proof
                                == aFace.receiver.selectorProof) {
                            aMatch = anOrdinal; ++aMatches;
                        }
                    if (aMatches != 1) return {};
                    core3d::face_image::ResourceEnvelope anEnvelope;
                    std::copy_n(aFace.pngDigest.begin(),
                        anEnvelope.resource.size(), anEnvelope.resource.begin());
                    anEnvelope.workingFormat =
                        core3d::face_image::ImageEncoding::PNG;
                    anEnvelope.workingWidthTexels = aFace.pixelWidth;
                    anEnvelope.workingHeightTexels = aFace.pixelHeight;
                    anEnvelope.workingBytes = aFace.png;
                    std::int32_t aTextureIndex = -1;
                    if (!AddFaceImageTextureResource(
                            aScene, aTextureTable, anEnvelope, aTextureIndex)
                        || aTextureIndex < 0) return {};
                    DecalDerivedAppearanceSnapshot anAppearance;
                    anAppearance.ownerDefinitionIdentifier =
                        aMesh.paintedAtlasMasterDefinitionIdentifier.empty()
                            ? aMesh.definitionIdentifier
                            : aMesh.paintedAtlasMasterDefinitionIdentifier;
                    anAppearance.bakeProof = DecalDigestText(aPublished.bakeKey);
                    anAppearance.faceIndex =
                        static_cast<std::uint32_t>(aMatch);
                    anAppearance.baseColorTextureIndex = aTextureIndex;
                    anInstance.decalDerivedAppearances.push_back(
                        std::move(anAppearance));
                }
                if (anInstance.decalDerivedAppearances.empty()) return {};
            }
        }

        std::size_t anInstanceBytes = 0;
        std::size_t aBindingBytes = 0;
        std::size_t aMaterialBytes = 0;
        std::size_t aTextureMetadataBytes = 0;
        std::size_t aPickBytes = 0;
        std::size_t aFaceImageBindingBytes = 0;
        std::size_t aDecalAppearanceBytes = 0;
        std::size_t anAuxiliaryBytes = 0;
        std::size_t aFaceImageBindingCount = 0;
        std::size_t aDecalAppearanceCount = 0;
        for (const auto& instance : aScene.instances) {
            if (!CheckedAdd(aFaceImageBindingCount,
                            instance.faceImageBindings.size(),
                            aFaceImageBindingCount)) {
                return {};
            }
            if (!CheckedAdd(aDecalAppearanceCount,
                            instance.decalDerivedAppearances.size(),
                            aDecalAppearanceCount)) return {};
        }
        constexpr std::size_t anInstanceNumericSize =
            sizeof(Matrix4d) + sizeof(ReferenceAxisSnapshot);
        if (!CheckedMultiply(aScene.instances.size(),
                             anInstanceNumericSize,
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
            || !CheckedMultiply(aFaceImageBindingCount,
                                sizeof(FaceImageBindingSnapshot),
                                aFaceImageBindingBytes)
            || !CheckedMultiply(aDecalAppearanceCount,
                                sizeof(DecalDerivedAppearanceSnapshot),
                                aDecalAppearanceBytes)
            || !CheckedAdd(anInstanceBytes, aBindingBytes, anAuxiliaryBytes)
            || !CheckedAdd(anAuxiliaryBytes, aMaterialBytes, anAuxiliaryBytes)
            || !CheckedAdd(anAuxiliaryBytes,
                           aTextureMetadataBytes,
                           anAuxiliaryBytes)
            || !CheckedAdd(anAuxiliaryBytes, aPickBytes, anAuxiliaryBytes)
            || !CheckedAdd(anAuxiliaryBytes,
                           aFaceImageBindingBytes,
                           anAuxiliaryBytes)
            || !CheckedAdd(anAuxiliaryBytes,
                           aDecalAppearanceBytes,
                           anAuxiliaryBytes)
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
                                            const TopoDS_Shape& theSubshape)
            -> std::optional<ElementIdentifier> {
            const InstanceSnapshot& anInstance =
                aScene.instances[theInstanceIndex];
            if (anInstance.meshIndex >= aScene.meshes.size()
                || anInstance.meshIndex >= aMeshDefinitionIndices.size()
                || aMeshDefinitionIndices[anInstance.meshIndex]
                    >= aDefinitions.size()) return std::nullopt;
            const MeshSnapshot& aMesh = aScene.meshes[anInstance.meshIndex];
            const DefinitionData& aDefinition =
                aDefinitions[aMeshDefinitionIndices[anInstance.meshIndex]];
            ElementIdentifier anElement;
            anElement.entityIdentifier = anInstance.entityIdentifier;
            anElement.kind = theAcceptedSelectionKind;
            anElement.geometryRevision = aMesh.geometryRevision;
            if (theAcceptedSelectionKind == ElementKind::Object) {
                // WholeShape is one document-object identity regardless of the
                // root TopoDS type. Valid BRep definitions may be rooted at a
                // Face, Wire, Edge, or Vertex; owner topology must not silently
                // change the accepted semantic mode.
                return anElement;
            }
            if (theAcceptedSelectionKind == ElementKind::Face
                && aMesh.topology.faceCount != 0
                && !theSubshape.IsNull()
                && theSubshape.ShapeType() == TopAbs_FACE) {
                const Standard_Integer aFaceIndex =
                    aDefinition.faces.FindIndex(theSubshape);
                if (aFaceIndex <= 0
                    || static_cast<std::uint32_t>(aFaceIndex)
                        > aMesh.topology.faceCount
                    || !aDefinition.faces.FindKey(aFaceIndex).IsEqual(
                        theSubshape)) {
                    return std::nullopt;
                }
                anElement.topologyIndex =
                    static_cast<std::uint32_t>(aFaceIndex - 1);
                return anElement;
            }
            if (theAcceptedSelectionKind == ElementKind::Edge
                && aMesh.topology.edgeCount != 0
                && !theSubshape.IsNull()
                && theSubshape.ShapeType() == TopAbs_EDGE) {
                const Standard_Integer anEdgeIndex =
                    aDefinition.edges.FindIndex(theSubshape);
                if (anEdgeIndex <= 0
                    || static_cast<std::uint32_t>(anEdgeIndex)
                        > aMesh.topology.edgeCount
                    || !aDefinition.edges.FindKey(anEdgeIndex).IsEqual(
                        theSubshape)) {
                    return std::nullopt;
                }
                anElement.topologyIndex =
                    static_cast<std::uint32_t>(anEdgeIndex - 1);
                return anElement;
            }
            // A mismatched/stale owner is rejected rather than silently
            // publishing an object identity that violates accepted authority.
            return std::nullopt;
        };
        theContext->InitSelected();
        for (; theContext->MoreSelected(); theContext->NextSelected()) {
            const Handle(AIS_InteractiveObject) anInteractive =
                theContext->SelectedInteractive();
            const Handle(SelectMgr_EntityOwner) aRawSelectedOwner =
                theContext->SelectedOwner();
            if (anInteractive.IsNull() || aRawSelectedOwner.IsNull()
                || !aRawSelectedOwner->HasSelectable()
                || aRawSelectedOwner->Selectable() != anInteractive) {
                // A renderer-neutral identity may never be inferred from a
                // foreign, detached, or outcome-ambiguous OCCT owner.
                return {};
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
                    aRawSelectedOwner);
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
                if (!anInstance.visible || !anInstance.selectable) {
                    return {};
                }
                const std::optional<ElementIdentifier> anElement =
                    elementForInstance(
                    anInstanceIndex,
                    aSelectedSubshape);
                if (!anElement.has_value()) {
                    return {};
                }
                const std::string aSelectionKey =
                    anElement->entityIdentifier + ":"
                    + std::to_string(
                        static_cast<unsigned int>(anElement->kind)) + ":"
                    + std::to_string(anElement->topologyIndex);
                if (aSelectedKeys.insert(aSelectionKey).second) {
                    aScene.selection.selected.push_back(*anElement);
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
            if (anInteractive.IsNull()) {
                return {};
            }
            {
                const Handle(SelectMgr_EntityOwner) aRawDetectedOwner =
                    theContext->DetectedOwner();
                if (aRawDetectedOwner.IsNull()
                    || !aRawDetectedOwner->HasSelectable()
                    || aRawDetectedOwner->Selectable() != anInteractive) {
                    return {};
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
                if (anInstanceIndices != nullptr) {
                    TopoDS_Shape aDetectedSubshape;
                    const Handle(StdSelect_BRepOwner) anOwner =
                        Handle(StdSelect_BRepOwner)::DownCast(
                            aRawDetectedOwner);
                    if (!anOwner.IsNull() && anOwner->HasShape()) {
                        aDetectedSubshape = anOwner->Shape();
                    }

                    std::optional<std::size_t> aDetectedInstance;
                    if (anInstanceIndices->size() == 1) {
                        const std::size_t anInstanceIndex =
                            anInstanceIndices->front();
                        if (aScene.instances[anInstanceIndex].selectable) {
                            aDetectedInstance = anInstanceIndex;
                        }
                    } else if (!aDetectedSubshape.IsNull()) {
                        for (const std::size_t anInstanceIndex :
                             *anInstanceIndices) {
                            const InstanceSnapshot& anInstance =
                                aScene.instances[anInstanceIndex];
                            if (!anInstance.selectable) {
                                continue;
                            }
                            if (anInstance.meshIndex
                                    >= aMeshDefinitionIndices.size()
                                || aMeshDefinitionIndices[anInstance.meshIndex]
                                    >= aDefinitions.size()) return {};
                            const DefinitionData& aDefinition =
                                aDefinitions[aMeshDefinitionIndices[
                                    anInstance.meshIndex]];
                            bool ownsDetectedSubshape = false;
                            if (theAcceptedSelectionKind
                                    == ElementKind::Face
                                && aDetectedSubshape.ShapeType()
                                    == TopAbs_FACE) {
                                const Standard_Integer aFaceIndex =
                                    aDefinition.faces.FindIndex(
                                        aDetectedSubshape);
                                ownsDetectedSubshape = aFaceIndex > 0
                                    && aDefinition.faces.FindKey(aFaceIndex)
                                        .IsEqual(aDetectedSubshape);
                            } else if (theAcceptedSelectionKind
                                           == ElementKind::Edge
                                && aDetectedSubshape.ShapeType()
                                    == TopAbs_EDGE) {
                                const Standard_Integer anEdgeIndex =
                                    aDefinition.edges.FindIndex(
                                        aDetectedSubshape);
                                ownsDetectedSubshape = anEdgeIndex > 0
                                    && aDefinition.edges.FindKey(anEdgeIndex)
                                        .IsEqual(aDetectedSubshape);
                            } else if (theAcceptedSelectionKind
                                           == ElementKind::Object) {
                                ownsDetectedSubshape =
                                    aDefinition.shape.IsSame(
                                        aDetectedSubshape)
                                    || (aDetectedSubshape.ShapeType()
                                            == TopAbs_FACE
                                        && aDefinition.faces.FindIndex(
                                            aDetectedSubshape) > 0)
                                    || (aDetectedSubshape.ShapeType()
                                            == TopAbs_EDGE
                                        && aDefinition.edges.FindIndex(
                                            aDetectedSubshape) > 0)
                                    || (aDetectedSubshape.ShapeType()
                                            == TopAbs_VERTEX
                                        && aDefinition.vertices.FindIndex(
                                            aDetectedSubshape) > 0);
                            }
                            if (ownsDetectedSubshape) {
                                if (aDetectedInstance.has_value()) {
                                    aDetectedInstance.reset();
                                    break;
                                }
                                aDetectedInstance = anInstanceIndex;
                            }
                        }
                    }
                    if (aDetectedInstance.has_value()) {
                        const std::optional<ElementIdentifier> anElement =
                            elementForInstance(
                            *aDetectedInstance,
                            aDetectedSubshape);
                        if (!anElement.has_value()) {
                            return {};
                        }
                        aScene.selection.hovered = *anElement;
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
        // A builder invariant failure must never advance committed revision
        // state. Validate the exact renderer-neutral payload before publishing
        // the next state, rather than relying solely on the later DTO bridge.
        if (!IsValidSceneSnapshot(aScene)) {
            return {};
        }
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

        auto contactSources=std::make_shared<State::NativeContactSourceMap>();
        contactSources->reserve(aScene.instances.size());
        for (const auto& instance:aScene.instances) {
            if (instance.meshIndex>=aScene.meshes.size()
                || instance.meshIndex>=aMeshDefinitionIndices.size()
                || aMeshDefinitionIndices[instance.meshIndex]
                    >=aDefinitions.size()) return {};
            const auto& definition=aDefinitions[
                aMeshDefinitionIndices[instance.meshIndex]];
            const auto& mesh=aScene.meshes[instance.meshIndex];
            if (!contactSources->emplace(instance.entityIdentifier,
                State::NativeContactSource{definition.label,
                    mesh.definitionIdentifier,mesh.geometryRevision,
                    definition.representation,instance.worldFromObject.values,
                    {definition.sourceOrigin.x,definition.sourceOrigin.y,definition.sourceOrigin.z},
                    aScene.metersPerUnit}).second) return {};
        }
        aNextState.lastFullNativeContactSources=std::move(contactSources);

#ifdef DEBUG
        if (gDebugBoundedCurveObservationArmed) {
            if (aDebugBoundedCurveObservation.definitionIdentifier.empty()) {
                return {};
            }
            const auto anObservedInstance = std::find_if(
                aScene.instances.begin(), aScene.instances.end(),
                [&](const InstanceSnapshot& theInstance) {
                    return theInstance.meshIndex < aScene.meshes.size()
                        && aScene.meshes[theInstance.meshIndex]
                            .definitionIdentifier
                            == aDebugBoundedCurveObservation
                                .definitionIdentifier;
                });
            if (anObservedInstance == aScene.instances.end()
                || anObservedInstance->meshIndex >= aScene.meshes.size()) {
                return {};
            }
            aDebugBoundedCurveObservation.entityIdentifier =
                anObservedInstance->entityIdentifier;
            aDebugBoundedCurveObservation.publicationSourceIdentifier =
                aScene.publicationSourceIdentifier;
            aDebugBoundedCurveObservation.modelRevision =
                aScene.revisions.model;
        }
#endif

        // Finish every potentially allocating operation before publishing
        // state. A failed allocation must not consume any revision.
        auto aCommittedState = std::make_unique<State>(std::move(aNextState));
        SnapshotPointer aSnapshot =
            std::make_shared<const SceneSnapshot>(std::move(aScene));
        myState.swap(aCommittedState);
#ifdef DEBUG
        if (gDebugBoundedCurveObservationArmed) {
            gDebugBoundedCurveObservation =
                std::move(aDebugBoundedCurveObservation);
            gDebugBoundedCurveObservationArmed = false;
        }
#endif
        return aSnapshot;
    } catch (const Standard_Failure&) {
        return {};
    } catch (...) {
        return {};
    }
}

} // namespace core3d::scene
