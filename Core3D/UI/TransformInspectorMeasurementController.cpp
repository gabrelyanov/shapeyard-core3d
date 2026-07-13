//
//  TransformInspectorMeasurementController.cpp
//  Core3D
//

#include "TransformInspectorMeasurementController.hpp"

#include "ObjectInteractor.hpp"
#include "ShapeInteractor.hpp"

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBndLib.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <Precision.hxx>
#include <Standard_ErrorHandler.hxx>
#include <Standard_Failure.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <TDataStd_Name.hxx>
#include <TDataStd_Real.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_EulerSequence.hxx>
#include <gp_Pnt.hxx>
#include <gp_Quaternion.hxx>
#include <gp_Trsf.hxx>
#include <gp_XYZ.hxx>

#include <dispatch/dispatch.h>
#include <os/signpost.h>
#include <pthread.h>
#include <TargetConditionals.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <limits>
#include <list>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

namespace core3d {

namespace {

constexpr double kLegacyMetersPerUnit = 0.001;
constexpr double kQuaternionHalfTurnZeroTolerance =
    64.0 * std::numeric_limits<double>::epsilon();
constexpr double kRadiansToDegrees =
    57.295779513082320876798154814105;

// These are the serialization-stable Core3DModelCapability bits. Keeping the
// representation policy here means the public bridge copies one authoritative
// value instead of re-reading OCAF or reconstructing feature policy.
constexpr std::uint64_t kBRepModelCapabilities = (1ull << 17) - 1ull;
constexpr std::uint64_t kTriangleMeshModelCapabilities =
    (1ull << 0)  // ObjectSelection
    | (1ull << 2)  // Translate
    | (1ull << 3)  // Rotate
    | (1ull << 6)  // Delete
    | (1ull << 7)  // Duplicate
    | (1ull << 12) // Material
    | (1ull << 13) // ExportOBJ
    | (1ull << 14) // ExportSTL
    | (1ull << 15); // ExportGLB

bool TryInspectorRepresentation(
    const OcctGeometryRepresentation theStoredRepresentation,
    TransformInspectorGeometryRepresentation& theRepresentation,
    std::uint64_t& theCapabilities) noexcept
{
    switch (theStoredRepresentation) {
        case OcctGeometryRepresentation::LegacyUnknown:
        case OcctGeometryRepresentation::BRep:
            theRepresentation =
                TransformInspectorGeometryRepresentation::BRep;
            theCapabilities = kBRepModelCapabilities;
            return true;
        case OcctGeometryRepresentation::TriangleMesh:
            theRepresentation =
                TransformInspectorGeometryRepresentation::TriangleMesh;
            theCapabilities = kTriangleMeshModelCapabilities;
            return true;
        case OcctGeometryRepresentation::Invalid:
            break;
    }
    theRepresentation =
        TransformInspectorGeometryRepresentation::Invalid;
    theCapabilities = 0;
    return false;
}

bool TryReadMetersPerUnit(
    const Handle(TDocStd_Document)& theDocument,
    double& theMetersPerUnit) noexcept
{
    theMetersPerUnit = kLegacyMetersPerUnit;
    if (theDocument.IsNull()) {
        return false;
    }
    try {
        const Standard_Boolean hasLengthUnit =
            XCAFDoc_DocumentTool::GetLengthUnit(
                theDocument, theMetersPerUnit);
        return (hasLengthUnit
                   && std::isfinite(theMetersPerUnit)
                   && theMetersPerUnit > 0.0)
            || (!hasLengthUnit
                && theMetersPerUnit == kLegacyMetersPerUnit);
    } catch (...) {
        return false;
    }
}

using Clock = std::chrono::steady_clock;

os_log_t TransformInspectorSignpostLog() noexcept
{
    static os_log_t aLog = os_log_create(
        "com.shapeyard.core3d",
        "TransformInspector");
    return aLog;
}

enum class TransformInspectorSignpost : std::uint8_t {
    Capture = 0,
    MeshSweep,
    BRepCopy,
    Accept,
};

class ScopedTransformInspectorSignpost final {
public:
    explicit ScopedTransformInspectorSignpost(
        const TransformInspectorSignpost theKind) noexcept
    : myKind(theKind),
      myIdentifier(os_signpost_id_generate(
          TransformInspectorSignpostLog()))
    {
        switch (myKind) {
            case TransformInspectorSignpost::Capture:
                os_signpost_interval_begin(
                    TransformInspectorSignpostLog(), myIdentifier,
                    "inspector.capture");
                break;
            case TransformInspectorSignpost::MeshSweep:
                os_signpost_interval_begin(
                    TransformInspectorSignpostLog(), myIdentifier,
                    "inspector.mesh_sweep");
                break;
            case TransformInspectorSignpost::BRepCopy:
                os_signpost_interval_begin(
                    TransformInspectorSignpostLog(), myIdentifier,
                    "inspector.brep_copy");
                break;
            case TransformInspectorSignpost::Accept:
                os_signpost_interval_begin(
                    TransformInspectorSignpostLog(), myIdentifier,
                    "inspector.accept");
                break;
        }
    }

    ~ScopedTransformInspectorSignpost() noexcept
    {
        switch (myKind) {
            case TransformInspectorSignpost::Capture:
                os_signpost_interval_end(
                    TransformInspectorSignpostLog(), myIdentifier,
                    "inspector.capture");
                break;
            case TransformInspectorSignpost::MeshSweep:
                os_signpost_interval_end(
                    TransformInspectorSignpostLog(), myIdentifier,
                    "inspector.mesh_sweep");
                break;
            case TransformInspectorSignpost::BRepCopy:
                os_signpost_interval_end(
                    TransformInspectorSignpostLog(), myIdentifier,
                    "inspector.brep_copy");
                break;
            case TransformInspectorSignpost::Accept:
                os_signpost_interval_end(
                    TransformInspectorSignpostLog(), myIdentifier,
                    "inspector.accept");
                break;
        }
    }

private:
    TransformInspectorSignpost myKind;
    os_signpost_id_t myIdentifier;
};

double MillisecondsSince(const Clock::time_point theStart) noexcept
{
    return std::chrono::duration<double, std::milli>(
               Clock::now() - theStart)
        .count();
}

struct TransformInspectorBoundsKey {
    Handle(TDocStd_Document) document;
    TopoDS_Shape shape;
};

struct TransformInspectorBoundsValue {
    std::array<double, 6> bounds{};
    TransformInspectorVector3 localDimensions;
};

enum class TransformInspectorNegativeBoundsReason : std::uint8_t {
    SweepAborted = 0,
    ExactBoundsFailed,
    CopyFailed,
};

bool BoundsKeysMatch(
    const TransformInspectorBoundsKey& theLeft,
    const TransformInspectorBoundsKey& theRight) noexcept
{
    try {
        return !theLeft.document.IsNull()
            && theLeft.document == theRight.document
            && !theLeft.shape.IsNull()
            && !theRight.shape.IsNull()
            // IsEqual is exactly TShape identity + Location + Orientation.
            && theLeft.shape.IsEqual(theRight.shape);
    } catch (...) {
        return false;
    }
}

bool TryBoundsValue(
    const Bnd_Box& theBox,
    TransformInspectorBoundsValue& theValue) noexcept
{
    try {
        OCC_CATCH_SIGNALS
        if (theBox.IsVoid() || theBox.IsOpen()) {
            return false;
        }
        theBox.Get(
            theValue.bounds[0],
            theValue.bounds[1],
            theValue.bounds[2],
            theValue.bounds[3],
            theValue.bounds[4],
            theValue.bounds[5]);
        for (const double aValue : theValue.bounds) {
            if (!std::isfinite(aValue)) {
                return false;
            }
        }
        const double aWidth =
            theValue.bounds[3] - theValue.bounds[0];
        const double aHeight =
            theValue.bounds[4] - theValue.bounds[1];
        const double aDepth =
            theValue.bounds[5] - theValue.bounds[2];
        if (!std::isfinite(aWidth) || !std::isfinite(aHeight)
            || !std::isfinite(aDepth) || aWidth < 0.0
            || aHeight < 0.0 || aDepth < 0.0) {
            return false;
        }
        theValue.localDimensions = {aWidth, aHeight, aDepth};
        return true;
    } catch (...) {
        return false;
    }
}

bool TryApplyBounds(
    const TransformInspectorBoundsValue& theBounds,
    TransformInspectorMeasurement& theMeasurement) noexcept
{
    const double aScale = std::abs(theMeasurement.uniformScale);
    const TransformInspectorVector3 aDimensions = {
        theBounds.localDimensions.x * aScale,
        theBounds.localDimensions.y * aScale,
        theBounds.localDimensions.z * aScale,
    };
    if (!std::isfinite(aScale) || !std::isfinite(aDimensions.x)
        || !std::isfinite(aDimensions.y)
        || !std::isfinite(aDimensions.z)) {
        return false;
    }
    theMeasurement.localBounds = theBounds.bounds;
    theMeasurement.localDimensions = theBounds.localDimensions;
    theMeasurement.dimensions = aDimensions;
    return true;
}

enum class TopologyAdmission : std::uint8_t {
    WithinLimit = 0,
    AboveLimit,
    Failed,
};

enum class TriangleMeshSweepOutcome : std::uint8_t {
    Success = 0,
    Failed,
    WatchdogAborted,
};

TopologyAdmission AdmitTriangleMeshBoundsSweep(
    const TopoDS_Shape& theShape,
    const Standard_Size theMaximumFaces,
    const Standard_Size theMaximumNodes,
    Standard_Size& theAdmittedNodeCount) noexcept
{
    theAdmittedNodeCount = 0;
    if (theShape.IsNull() || theMaximumFaces == 0
        || theMaximumNodes == 0) {
        return TopologyAdmission::Failed;
    }
    try {
        OCC_CATCH_SIGNALS
        Standard_Size aFaceCount = 0;
        for (TopExp_Explorer aFaceItem(theShape, TopAbs_FACE);
             aFaceItem.More(); aFaceItem.Next()) {
            if (++aFaceCount > theMaximumFaces) {
                return TopologyAdmission::AboveLimit;
            }
            const TopoDS_Face aFace = TopoDS::Face(aFaceItem.Current());
            TopLoc_Location aLocation;
            const Handle(Poly_Triangulation) aTriangulation =
                BRep_Tool::Triangulation(aFace, aLocation);
            if (aTriangulation.IsNull()
                || aTriangulation->HasDeferredData()
                || !aTriangulation->HasGeometry()
                || aTriangulation->NbNodes() <= 0
                || aTriangulation->NbTriangles() <= 0) {
                return TopologyAdmission::Failed;
            }
            const Standard_Size aFaceNodeCount =
                static_cast<Standard_Size>(
                    aTriangulation->NbNodes());
            if (aFaceNodeCount
                > theMaximumNodes - theAdmittedNodeCount) {
                return TopologyAdmission::AboveLimit;
            }
            theAdmittedNodeCount += aFaceNodeCount;
        }
        return aFaceCount > 0 && theAdmittedNodeCount > 0
            ? TopologyAdmission::WithinLimit
            : TopologyAdmission::Failed;
    } catch (...) {
        theAdmittedNodeCount = 0;
        return TopologyAdmission::Failed;
    }
}

TopologyAdmission CountBoundedTopology(
    const TopoDS_Shape& theShape,
    const Standard_Size theMaximumNodes,
    Standard_Size& theCount) noexcept
{
    theCount = 0;
    if (theShape.IsNull() || theMaximumNodes == 0) {
        return TopologyAdmission::Failed;
    }
    try {
        OCC_CATCH_SIGNALS
        std::vector<TopoDS_Shape> aStack;
        aStack.reserve(std::min<Standard_Size>(theMaximumNodes, 1'024));
        aStack.push_back(theShape);
        TopTools_IndexedMapOfShape aVisited;
        while (!aStack.empty()) {
            const TopoDS_Shape aCurrent = aStack.back();
            aStack.pop_back();
            if (aVisited.Contains(aCurrent)) {
                continue;
            }
            aVisited.Add(aCurrent);
            if (++theCount > theMaximumNodes) {
                return TopologyAdmission::AboveLimit;
            }
            for (TopoDS_Iterator aChild(
                     aCurrent, Standard_True, Standard_True);
                 aChild.More(); aChild.Next()) {
                if (!aVisited.Contains(aChild.Value())) {
                    if (aStack.size() >= theMaximumNodes) {
                        return TopologyAdmission::AboveLimit;
                    }
                    aStack.push_back(aChild.Value());
                }
            }
        }
        return theCount > 0
            ? TopologyAdmission::WithinLimit
            : TopologyAdmission::Failed;
    } catch (...) {
        theCount = 0;
        return TopologyAdmission::Failed;
    }
}

TriangleMeshSweepOutcome TrySweepTriangleMeshBounds(
    const TopoDS_Shape& theShape,
    const Standard_Size theAdmittedNodeCount,
    const Standard_Real theWatchdogDeadlineMilliseconds,
    const Standard_Size theWatchdogPollNodes,
    TransformInspectorBoundsValue& theValue) noexcept
{
    if (theShape.IsNull()) {
        return TriangleMeshSweepOutcome::Failed;
    }
    try {
        OCC_CATCH_SIGNALS
        const Clock::time_point aStart = Clock::now();
        Bnd_Box aBounds;
        Standard_Size aFaceCount = 0;
        Standard_Size aNodeCount = 0;
        for (TopExp_Explorer aFaceItem(theShape, TopAbs_FACE);
             aFaceItem.More(); aFaceItem.Next()) {
            ++aFaceCount;
            const TopoDS_Face aFace = TopoDS::Face(aFaceItem.Current());
            TopLoc_Location aLocation;
            const Handle(Poly_Triangulation) aTriangulation =
                BRep_Tool::Triangulation(aFace, aLocation);
            if (aTriangulation.IsNull()
                || aTriangulation->HasDeferredData()
                || !aTriangulation->HasGeometry()
                || aTriangulation->NbNodes() <= 0
                || aTriangulation->NbTriangles() <= 0) {
                return TriangleMeshSweepOutcome::Failed;
            }
            const bool hasIdentityLocation = aLocation.IsIdentity();
            const gp_Trsf& aLocationTransform =
                aLocation.Transformation();
            for (Standard_Integer aNodeIndex = 1;
                 aNodeIndex <= aTriangulation->NbNodes();
                 ++aNodeIndex) {
                gp_Pnt aPoint = aTriangulation->Node(aNodeIndex);
                if (!hasIdentityLocation) {
                    aPoint.Transform(aLocationTransform);
                }
                if (!std::isfinite(aPoint.X())
                    || !std::isfinite(aPoint.Y())
                    || !std::isfinite(aPoint.Z())) {
                    return TriangleMeshSweepOutcome::Failed;
                }
                aBounds.Update(aPoint.X(), aPoint.Y(), aPoint.Z());
                ++aNodeCount;
                if (theWatchdogPollNodes > 0
                    && aNodeCount % theWatchdogPollNodes == 0
                    && aNodeCount < theAdmittedNodeCount
                    && std::isfinite(theWatchdogDeadlineMilliseconds)
                    && MillisecondsSince(aStart)
                        >= theWatchdogDeadlineMilliseconds) {
                    return TriangleMeshSweepOutcome::WatchdogAborted;
                }
            }
        }
        return aFaceCount > 0 && aNodeCount > 0
            && TryBoundsValue(aBounds, theValue)
            ? TriangleMeshSweepOutcome::Success
            : TriangleMeshSweepOutcome::Failed;
    } catch (...) {
        return TriangleMeshSweepOutcome::Failed;
    }
}

bool TryReadReal(
    const TDF_Label& theDefinition,
    const Standard_Integer theTag,
    const double theDefault,
    double& theValue) noexcept
{
    theValue = theDefault;
    if (theDefinition.IsNull()) {
        return false;
    }
    try {
        const TDF_Label aChild =
            theDefinition.FindChild(theTag, Standard_False);
        if (!aChild.IsNull()) {
            Handle(TDataStd_Real) anAttribute;
            if (aChild.FindAttribute(TDataStd_Real::GetID(), anAttribute)
                && !anAttribute.IsNull()) {
                theValue = anAttribute->Get();
            }
        }
        return std::isfinite(theValue);
    } catch (...) {
        return false;
    }
}

double CanonicalDegrees(const double theDegrees) noexcept
{
    if (!std::isfinite(theDegrees)) {
        return theDegrees;
    }
    double aDegrees = std::remainder(theDegrees, 360.0);
    // std::remainder() includes both signed representations of the endpoint.
    // Keep -180 and map +180 to it so the public interval is exactly
    // [-180, 180), rather than choosing an endpoint based on quotient parity.
    if (aDegrees >= 180.0) {
        aDegrees -= 360.0;
    }
    return std::abs(aDegrees) < 1.0e-12 ? 0.0 : aDegrees;
}

struct PersistedTransformCapture {
    gp_Trsf transform;
    TransformInspectorVector3 position;
    TransformInspectorQuaternion quaternion;
    TransformInspectorVector3 extrinsicXYZDegrees;
    double uniformScale = 1.0;
};

bool TryCapturePersistedTransform(
    const TDF_Label& theDefinition,
    PersistedTransformCapture& theCapture) noexcept
{
    // Read and validate every scalar before constructing gp_Quaternion or
    // gp_Trsf. This prevents malformed OCAF attributes from reaching OCCT
    // constructors that require a nondegenerate quaternion/scale.
    double aValues[8] = {};
    const double aDefaults[8] = {
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0,
        1.0,
    };
    for (Standard_Integer anIndex = 0; anIndex < 8; ++anIndex) {
        if (!TryReadReal(
                theDefinition,
                anIndex + 1,
                aDefaults[anIndex],
                aValues[anIndex])) {
            return false;
        }
    }

    const double aMaxQuaternionComponent = std::max({
        std::abs(aValues[3]),
        std::abs(aValues[4]),
        std::abs(aValues[5]),
        std::abs(aValues[6]),
    });
    if (!std::isfinite(aMaxQuaternionComponent)
        || aMaxQuaternionComponent
            <= std::numeric_limits<double>::min()
        || std::abs(aValues[7]) <= Precision::Confusion()) {
        return false;
    }
    const double aQx = aValues[3] / aMaxQuaternionComponent;
    const double aQy = aValues[4] / aMaxQuaternionComponent;
    const double aQz = aValues[5] / aMaxQuaternionComponent;
    const double aQw = aValues[6] / aMaxQuaternionComponent;
    const double aScaledNorm = std::sqrt(
        aQx * aQx + aQy * aQy + aQz * aQz + aQw * aQw);
    if (!std::isfinite(aScaledNorm)
        || aScaledNorm <= std::numeric_limits<double>::epsilon()) {
        return false;
    }

    double aNormalizedX = aQx / aScaledNorm;
    double aNormalizedY = aQy / aScaledNorm;
    double aNormalizedZ = aQz / aScaledNorm;
    double aNormalizedW = aQw / aScaledNorm;
    // q and -q encode the same rotation. Choose one deterministic public
    // spelling. Values sufficiently close to a half turn first share the
    // exact w==0 spelling, then receive one lexicographic sign decision.
    if (std::abs(aNormalizedW)
        <= kQuaternionHalfTurnZeroTolerance) {
        aNormalizedW = 0.0;
    }
    if (aNormalizedW < 0.0
        || (aNormalizedW == 0.0
            && (aNormalizedX < 0.0
                || (aNormalizedX == 0.0 && aNormalizedY < 0.0)
                || (aNormalizedX == 0.0 && aNormalizedY == 0.0
                    && aNormalizedZ < 0.0)))) {
        aNormalizedX = -aNormalizedX;
        aNormalizedY = -aNormalizedY;
        aNormalizedZ = -aNormalizedZ;
        aNormalizedW = -aNormalizedW;
    }
    const double aCanonicalNorm = std::sqrt(
        aNormalizedX * aNormalizedX
        + aNormalizedY * aNormalizedY
        + aNormalizedZ * aNormalizedZ
        + aNormalizedW * aNormalizedW);
    if (!std::isfinite(aCanonicalNorm)
        || aCanonicalNorm
            <= std::numeric_limits<double>::epsilon()) {
        return false;
    }
    aNormalizedX /= aCanonicalNorm;
    aNormalizedY /= aCanonicalNorm;
    aNormalizedZ /= aCanonicalNorm;
    aNormalizedW /= aCanonicalNorm;
    if (aNormalizedW == 0.0) {
        // Negating the chosen half-turn tuple can create -0.0. Publish one
        // bit-stable zero representation without making another sign choice.
        aNormalizedW = 0.0;
    }

    try {
        OCC_CATCH_SIGNALS
        const gp_Quaternion aRotation(
            aNormalizedX,
            aNormalizedY,
            aNormalizedZ,
            aNormalizedW);
        double anEulerX = 0.0;
        double anEulerY = 0.0;
        double anEulerZ = 0.0;
        aRotation.GetEulerAngles(
            gp_Extrinsic_XYZ,
            anEulerX,
            anEulerY,
            anEulerZ);
        if (!std::isfinite(anEulerX) || !std::isfinite(anEulerY)
            || !std::isfinite(anEulerZ)) {
            return false;
        }

        gp_Trsf aTransform;
        aTransform.SetRotationPart(aRotation);
        aTransform.SetScaleFactor(aValues[7]);
        aTransform.SetTranslationPart(
            gp_XYZ(aValues[0], aValues[1], aValues[2]));
        for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
            for (Standard_Integer aColumn = 1; aColumn <= 4;
                 ++aColumn) {
                if (!std::isfinite(
                        aTransform.Value(aRow, aColumn))) {
                    return false;
                }
            }
        }

        theCapture.transform = aTransform;
        theCapture.position = {aValues[0], aValues[1], aValues[2]};
        theCapture.quaternion = {
            aNormalizedX,
            aNormalizedY,
            aNormalizedZ,
            aNormalizedW,
        };
        theCapture.extrinsicXYZDegrees = {
            CanonicalDegrees(anEulerX * kRadiansToDegrees),
            CanonicalDegrees(anEulerY * kRadiansToDegrees),
            CanonicalDegrees(anEulerZ * kRadiansToDegrees),
        };
        theCapture.uniformScale = aValues[7];
        return true;
    } catch (...) {
        return false;
    }
}

bool TransformsMatch(
    const gp_Trsf& theLeft,
    const gp_Trsf& theRight) noexcept
{
    try {
        for (Standard_Integer aRow = 1; aRow <= 3; ++aRow) {
            for (Standard_Integer aColumn = 1; aColumn <= 4;
                 ++aColumn) {
                const double aLeft = theLeft.Value(aRow, aColumn);
                const double aRight = theRight.Value(aRow, aColumn);
                const double aMagnitude = std::max({
                    1.0, std::abs(aLeft), std::abs(aRight)});
                if (!std::isfinite(aLeft) || !std::isfinite(aRight)
                    || std::abs(aLeft - aRight)
                        > Precision::Confusion() * aMagnitude) {
                    return false;
                }
            }
        }
        return true;
    } catch (...) {
        return false;
    }
}

std::string ReadDefinitionName(const TDF_Label& theDefinition) noexcept
{
    if (theDefinition.IsNull()) {
        return {};
    }
    try {
        Handle(TDataStd_Name) aName;
        if (!theDefinition.FindAttribute(TDataStd_Name::GetID(), aName)
            || aName.IsNull() || aName->Get().IsEmpty()) {
            return {};
        }
        const TCollection_ExtendedString& aValue = aName->Get();
        const Standard_Integer aLength = aValue.LengthOfCString();
        if (aLength <= 0) {
            return {};
        }
        std::vector<char> aBuffer(
            static_cast<std::size_t>(aLength) + 1, '\0');
        Standard_PCharacter aData = aBuffer.data();
        const Standard_Integer aWritten = aValue.ToUTF8CString(aData);
        return aWritten > 0
            ? std::string(aBuffer.data(), static_cast<std::size_t>(aWritten))
            : std::string();
    } catch (...) {
        return {};
    }
}

bool EnvironmentIsIdle(
    const Handle(OcctDocument)& theDocument,
    const std::shared_ptr<ObjectInteractor>& theObjectInteractor,
    const std::shared_ptr<ShapeInteractor>& theShapeInteractor) noexcept
{
    if (theDocument.IsNull() || theObjectInteractor == nullptr
        || theShapeInteractor == nullptr) {
        return false;
    }
    try {
        const Handle(TDocStd_Document) aDocument =
            theDocument->Document();
        return !aDocument.IsNull() && !aDocument->HasOpenCommand()
            && !theObjectInteractor->isManipulatorGestureActive()
            && !theObjectInteractor->hasActiveBoolean()
            && !theObjectInteractor->hasUnresolvedBoolean()
            && !theObjectInteractor->hasTrialMirrorObjects()
            && !theObjectInteractor->hasUnresolvedMirrorObjects()
            && !theShapeInteractor->hasActiveExtrusion()
            && !theShapeInteractor->hasActiveBevel();
    } catch (...) {
        return false;
    }
}

enum class BoundsWorkerOutcome : std::uint8_t {
    Success = 0,
    Cancelled,
    Failed,
};

struct TransformInspectorBoundsWorkerRequest {
    std::uint64_t generation = 0;
    TopoDS_Shape copiedShape;
    std::shared_ptr<std::atomic_bool> cancellation;
    bool forceFailure = false;
};

dispatch_queue_t TransformInspectorWorkerQueue()
{
    static dispatch_queue_t aQueue = []() {
        dispatch_queue_attr_t anAttribute =
            dispatch_queue_attr_make_with_qos_class(
                DISPATCH_QUEUE_SERIAL,
                QOS_CLASS_USER_INITIATED,
                0);
        return dispatch_queue_create(
            "com.shapeyard.core3d.transform-inspector-bounds",
            anAttribute);
    }();
    return aQueue;
}

} // namespace

struct TransformInspectorBoundsWorkerResult {
    std::uint64_t generation = 0;
    BoundsWorkerOutcome outcome = BoundsWorkerOutcome::Failed;
    TransformInspectorBoundsValue bounds;
    Standard_Boolean computeWasMainThread = Standard_False;
};

struct TransformInspectorBoundsWorkerEnqueueResult {
    Standard_Boolean accepted = Standard_False;
    std::optional<std::uint64_t> replacedPendingGeneration;
};

class TransformInspectorBoundsWorker final
    : public std::enable_shared_from_this<
          TransformInspectorBoundsWorker> {
public:
    explicit TransformInspectorBoundsWorker(
        std::weak_ptr<TransformInspectorMeasurementController> theOwner)
    : myOwner(std::move(theOwner)) {}

    ~TransformInspectorBoundsWorker() noexcept
    {
        shutdown();
    }

    TransformInspectorBoundsWorkerEnqueueResult enqueue(
        TransformInspectorBoundsWorkerRequest theRequest) noexcept
    {
        TransformInspectorBoundsWorkerEnqueueResult aResult;
        bool shouldSchedule = false;
        try {
            {
                std::lock_guard<std::mutex> aLock(myMutex);
                if (myStopped || theRequest.cancellation == nullptr
                    || theRequest.copiedShape.IsNull()) {
                    return aResult;
                }
                supersedeCancellation(myActiveCancellation);
                if (myPending.has_value()) {
                    aResult.replacedPendingGeneration =
                        myPending->generation;
                    supersedeCancellation(myPending->cancellation);
                    myPending.reset();
                    ++myPendingReplacementCount;
                }
                myPending = std::move(theRequest);
                ++mySubmittedCount;
                if (!myDrainScheduled) {
                    myDrainScheduled = true;
                    shouldSchedule = true;
                }
            }
            myCondition.notify_all();
            if (shouldSchedule) {
                auto* aContext =
                    new std::shared_ptr<TransformInspectorBoundsWorker>(
                        shared_from_this());
                dispatch_async_f(
                    TransformInspectorWorkerQueue(),
                    aContext,
                    &TransformInspectorBoundsWorker::DrainTrampoline);
            }
            aResult.accepted = Standard_True;
            return aResult;
        } catch (...) {
            (void)cancelAll();
            return aResult;
        }
    }

    std::optional<std::uint64_t> cancelAll() noexcept
    {
        std::optional<std::uint64_t> aCancelledPendingGeneration;
        {
            std::lock_guard<std::mutex> aLock(myMutex);
            supersedeCancellation(myActiveCancellation);
            if (myPending.has_value()) {
                aCancelledPendingGeneration = myPending->generation;
                supersedeCancellation(myPending->cancellation);
                myPending.reset();
            }
        }
        myCondition.notify_all();
        return aCancelledPendingGeneration;
    }

    void shutdown() noexcept
    {
        {
            std::lock_guard<std::mutex> aLock(myMutex);
            if (myStopped) {
                return;
            }
            myStopped = true;
            supersedeCancellation(myActiveCancellation);
            if (myPending.has_value()) {
                supersedeCancellation(myPending->cancellation);
                myPending.reset();
            }
#ifdef DEBUG
            myDebugBlocked = false;
#endif
        }
        myCondition.notify_all();
    }

#ifdef DEBUG
    struct DebugSnapshot {
        std::uint64_t submittedCount = 0;
        std::uint64_t startedCount = 0;
        std::uint64_t completedCount = 0;
        std::uint64_t cancelledOrSupersededCount = 0;
        std::uint64_t pendingReplacementCount = 0;
        Standard_Boolean active = Standard_False;
        Standard_Boolean pending = Standard_False;
        Standard_Boolean lastComputeWasMainThread = Standard_False;
        Standard_Boolean blocked = Standard_False;
    };

    DebugSnapshot debugSnapshot() const noexcept
    {
        std::lock_guard<std::mutex> aLock(myMutex);
        return {
            mySubmittedCount,
            myStartedCount,
            myCompletedCount,
            myCancelledOrSupersededCount,
            myPendingReplacementCount,
            myActiveCancellation != nullptr,
            myPending.has_value(),
            myLastComputeWasMainThread,
            myDebugBlocked,
        };
    }

    void debugSetBlocked(const Standard_Boolean theBlocked) noexcept
    {
        {
            std::lock_guard<std::mutex> aLock(myMutex);
            myDebugBlocked = theBlocked;
        }
        myCondition.notify_all();
    }
#endif

private:
    struct MainCompletion {
        std::weak_ptr<TransformInspectorMeasurementController> owner;
        TransformInspectorBoundsWorkerResult result;
    };

    static void DrainTrampoline(void* theContext)
    {
        std::unique_ptr<
            std::shared_ptr<TransformInspectorBoundsWorker>> aContext(
            static_cast<
                std::shared_ptr<TransformInspectorBoundsWorker>*>(
                theContext));
        (*aContext)->drain();
    }

    static void DeliverTrampoline(void* theContext)
    {
        std::unique_ptr<MainCompletion> aCompletion(
            static_cast<MainCompletion*>(theContext));
        if (const auto anOwner = aCompletion->owner.lock()) {
            anOwner->acceptWorkerResult(
                std::move(aCompletion->result));
        }
    }

    static TransformInspectorBoundsWorkerResult compute(
        const TransformInspectorBoundsWorkerRequest& theRequest) noexcept
    {
        TransformInspectorBoundsWorkerResult aResult;
        aResult.generation = theRequest.generation;
        aResult.computeWasMainThread = pthread_main_np() != 0;
        if (theRequest.cancellation == nullptr
            || theRequest.cancellation->load(std::memory_order_relaxed)) {
            aResult.outcome = BoundsWorkerOutcome::Cancelled;
            return aResult;
        }
        if (theRequest.forceFailure) {
            aResult.outcome = BoundsWorkerOutcome::Failed;
            return aResult;
        }
        try {
            OCC_CATCH_SIGNALS
            Bnd_Box aBounds;
            // AddOptimal has no cancellation seam. Deliberately do not read
            // the logical cancellation token after entering it: an exact stale
            // result is still safe and useful for the bounded local cache.
            BRepBndLib::AddOptimal(
                theRequest.copiedShape,
                aBounds,
                Standard_False,
                Standard_False);
            if (!TryBoundsValue(aBounds, aResult.bounds)) {
                aResult.outcome = BoundsWorkerOutcome::Failed;
                return aResult;
            }
            aResult.outcome = BoundsWorkerOutcome::Success;
            return aResult;
        } catch (...) {
            aResult.outcome = BoundsWorkerOutcome::Failed;
            return aResult;
        }
    }

    void drain() noexcept
    {
        for (;;) {
            TransformInspectorBoundsWorkerRequest aRequest;
            {
                std::unique_lock<std::mutex> aLock(myMutex);
                if (myStopped || !myPending.has_value()) {
                    myDrainScheduled = false;
                    myActiveCancellation.reset();
                    return;
                }
                aRequest = std::move(*myPending);
                myPending.reset();
                myActiveCancellation = aRequest.cancellation;
                ++myStartedCount;
#ifdef DEBUG
                myCondition.wait(aLock, [&]() {
                    return myStopped || !myDebugBlocked;
                });
#endif
            }

            TransformInspectorBoundsWorkerResult aResult =
                compute(aRequest);
            {
                std::lock_guard<std::mutex> aLock(myMutex);
                myActiveCancellation.reset();
                ++myCompletedCount;
                myLastComputeWasMainThread =
                    aResult.computeWasMainThread;
            }
            try {
                auto* aCompletion = new MainCompletion{
                    myOwner,
                    std::move(aResult),
                };
                dispatch_async_f(
                    dispatch_get_main_queue(),
                    aCompletion,
                    &TransformInspectorBoundsWorker::DeliverTrampoline);
            } catch (...) {
            }
        }
    }

    void supersedeCancellation(
        const std::shared_ptr<std::atomic_bool>& theCancellation) noexcept
    {
        if (theCancellation != nullptr
            && !theCancellation->exchange(
                true, std::memory_order_relaxed)) {
            ++myCancelledOrSupersededCount;
        }
    }

    std::weak_ptr<TransformInspectorMeasurementController> myOwner;
    mutable std::mutex myMutex;
    std::condition_variable myCondition;
    std::optional<TransformInspectorBoundsWorkerRequest> myPending;
    std::shared_ptr<std::atomic_bool> myActiveCancellation;
    bool myDrainScheduled = false;
    bool myStopped = false;
    std::uint64_t mySubmittedCount = 0;
    std::uint64_t myStartedCount = 0;
    std::uint64_t myCompletedCount = 0;
    std::uint64_t myCancelledOrSupersededCount = 0;
    std::uint64_t myPendingReplacementCount = 0;
    Standard_Boolean myLastComputeWasMainThread = Standard_False;
#ifdef DEBUG
    bool myDebugBlocked = false;
#endif
};

struct TransformInspectorMeasurementController::Impl {
    struct CacheEntry {
        TransformInspectorBoundsKey key;
        TransformInspectorBoundsValue value;
    };

    struct NegativeCacheEntry {
        TransformInspectorBoundsKey key;
        TransformInspectorNegativeBoundsReason reason =
            TransformInspectorNegativeBoundsReason::ExactBoundsFailed;
    };

    std::list<CacheEntry> cache;
    std::list<NegativeCacheEntry> negativeCache;
    //! Main-thread-only retained identity keys for the worker's active and
    //! replaceable pending generations. Live OCAF/TShape handles never cross
    //! onto the worker queue.
    std::list<std::pair<std::uint64_t, TransformInspectorBoundsKey>>
        outstandingKeys;
    Handle(TDocStd_Document) cacheDocument;
    std::optional<TransformInspectorBoundsKey> requestedKey;
    TransformInspectorMeasurement pendingMeasurement;
    std::shared_ptr<TransformInspectorBoundsWorker> worker;
    Standard_Real lastMeshSweepMilliseconds = 0.0;
    Standard_Real lastBRepCopyMilliseconds = 0.0;
    std::uint64_t meshSweepRunCount = 0;
    std::uint64_t watchdogFireCount = 0;
#ifdef DEBUG
    Standard_Boolean debugWorkerBlocked = Standard_False;
    Standard_Boolean debugForcedBoundsFailure = Standard_False;
    Standard_Size debugMaximumBRepTopologyNodes =
        TransformInspectorMeasurementController::
            kMaximumBRepTopologyNodes;
    Standard_Size debugMaximumTriangleMeshSweepNodes =
        TransformInspectorMeasurementController::
            kMaximumTriangleMeshSweepNodes;
    Standard_Real debugMeshSweepWatchdogDeadlineMilliseconds =
        std::numeric_limits<Standard_Real>::infinity();
    Standard_Size debugMeshSweepWatchdogPollNodes =
        TransformInspectorMeasurementController::
            kMeshSweepWatchdogPollNodes;
    std::uint64_t acceptedCount = 0;
    std::uint64_t staleCount = 0;
#endif

    void clearCacheForDocument(
        const Handle(TDocStd_Document)& theDocument) noexcept
    {
        if (cacheDocument != theDocument) {
            cache.clear();
            negativeCache.clear();
            cacheDocument = theDocument;
        }
    }

    bool lookup(
        const TransformInspectorBoundsKey& theKey,
        TransformInspectorBoundsValue& theValue) noexcept
    {
        try {
            for (auto anEntry = cache.begin(); anEntry != cache.end();
                 ++anEntry) {
                if (!BoundsKeysMatch(anEntry->key, theKey)) {
                    continue;
                }
                theValue = anEntry->value;
                if (anEntry != cache.begin()) {
                    cache.splice(cache.begin(), cache, anEntry);
                }
                return true;
            }
        } catch (...) {
        }
        return false;
    }

    void insert(
        TransformInspectorBoundsKey theKey,
        TransformInspectorBoundsValue theValue) noexcept
    {
        try {
            eraseNegative(theKey);
            for (auto anEntry = cache.begin(); anEntry != cache.end();
                 ++anEntry) {
                if (BoundsKeysMatch(anEntry->key, theKey)) {
                    anEntry->value = std::move(theValue);
                    if (anEntry != cache.begin()) {
                        cache.splice(cache.begin(), cache, anEntry);
                    }
                    return;
                }
            }
            cache.push_front({std::move(theKey), std::move(theValue)});
            while (cache.size()
                   > TransformInspectorMeasurementController::
                       kMaximumBoundsCacheEntries) {
                cache.pop_back();
            }
        } catch (...) {
        }
    }

    bool lookupNegative(
        const TransformInspectorBoundsKey& theKey,
        TransformInspectorNegativeBoundsReason& theReason) noexcept
    {
        try {
            for (auto anEntry = negativeCache.begin();
                 anEntry != negativeCache.end(); ++anEntry) {
                if (!BoundsKeysMatch(anEntry->key, theKey)) {
                    continue;
                }
                theReason = anEntry->reason;
                if (anEntry != negativeCache.begin()) {
                    negativeCache.splice(
                        negativeCache.begin(), negativeCache, anEntry);
                }
                return true;
            }
        } catch (...) {
        }
        return false;
    }

    void insertNegative(
        TransformInspectorBoundsKey theKey,
        const TransformInspectorNegativeBoundsReason theReason) noexcept
    {
        try {
            for (auto anEntry = cache.begin();
                 anEntry != cache.end(); ++anEntry) {
                if (BoundsKeysMatch(anEntry->key, theKey)) {
                    cache.erase(anEntry);
                    break;
                }
            }
            for (auto anEntry = negativeCache.begin();
                 anEntry != negativeCache.end(); ++anEntry) {
                if (!BoundsKeysMatch(anEntry->key, theKey)) {
                    continue;
                }
                anEntry->reason = theReason;
                if (anEntry != negativeCache.begin()) {
                    negativeCache.splice(
                        negativeCache.begin(), negativeCache, anEntry);
                }
                return;
            }
            negativeCache.push_front({std::move(theKey), theReason});
            while (negativeCache.size()
                   > TransformInspectorMeasurementController::
                       kMaximumNegativeBoundsCacheEntries) {
                negativeCache.pop_back();
            }
        } catch (...) {
        }
    }

    void eraseNegative(
        const TransformInspectorBoundsKey& theKey) noexcept
    {
        try {
            for (auto anEntry = negativeCache.begin();
                 anEntry != negativeCache.end(); ++anEntry) {
                if (BoundsKeysMatch(anEntry->key, theKey)) {
                    negativeCache.erase(anEntry);
                    return;
                }
            }
        } catch (...) {
        }
    }

    bool rememberOutstandingKey(
        const std::uint64_t theGeneration,
        TransformInspectorBoundsKey theKey) noexcept
    {
        try {
            forgetOutstandingKey(theGeneration);
            outstandingKeys.push_back(
                {theGeneration, std::move(theKey)});
            return true;
        } catch (...) {
            return false;
        }
    }

    void forgetOutstandingKey(
        const std::uint64_t theGeneration) noexcept
    {
        for (auto anItem = outstandingKeys.begin();
             anItem != outstandingKeys.end(); ++anItem) {
            if (anItem->first == theGeneration) {
                outstandingKeys.erase(anItem);
                return;
            }
        }
    }

    std::optional<TransformInspectorBoundsKey> takeOutstandingKey(
        const std::uint64_t theGeneration) noexcept
    {
        try {
            for (auto anItem = outstandingKeys.begin();
                 anItem != outstandingKeys.end(); ++anItem) {
                if (anItem->first != theGeneration) {
                    continue;
                }
                TransformInspectorBoundsKey aKey =
                    std::move(anItem->second);
                outstandingKeys.erase(anItem);
                return aKey;
            }
        } catch (...) {
        }
        return std::nullopt;
    }
};

TransformInspectorMeasurementController::
TransformInspectorMeasurementController(
    Handle(AIS_InteractiveContext) theContext,
    Handle(OcctDocument) theDocument)
: myContext(std::move(theContext)),
  myDoc(std::move(theDocument)),
  myImpl(std::make_unique<Impl>())
{
}

TransformInspectorMeasurementController::
~TransformInspectorMeasurementController() noexcept
{
    shutdown();
}

void TransformInspectorMeasurementController::
cancelPendingMeasurement() noexcept
{
    if (myGeneration != std::numeric_limits<std::uint64_t>::max()) {
        ++myGeneration;
    }
    myCompletion = {};
    if (myImpl != nullptr) {
        myImpl->requestedKey.reset();
        if (myImpl->worker != nullptr) {
            const std::optional<std::uint64_t>
                aCancelledPendingGeneration =
                    myImpl->worker->cancelAll();
            if (aCancelledPendingGeneration.has_value()) {
                myImpl->forgetOutstandingKey(
                    *aCancelledPendingGeneration);
            }
        }
    }
}

void TransformInspectorMeasurementController::shutdown() noexcept
{
    if (myStopped) {
        return;
    }
    myStopped = Standard_True;
    cancelPendingMeasurement();
    if (myImpl != nullptr) {
        if (myImpl->worker != nullptr) {
            myImpl->worker->shutdown();
            myImpl->worker.reset();
        }
        myImpl->cache.clear();
        myImpl->negativeCache.clear();
        myImpl->outstandingKeys.clear();
        myImpl->cacheDocument.Nullify();
    }
    myObjectInteractor.reset();
    myShapeInteractor.reset();
    myContext.Nullify();
    myDoc.Nullify();
}

TransformInspectorMeasurementPerformanceState
TransformInspectorMeasurementController::performanceState() const noexcept
{
    TransformInspectorMeasurementPerformanceState aState;
    if (myImpl == nullptr) {
        return aState;
    }
    aState.lastMeshSweepMilliseconds =
        myImpl->lastMeshSweepMilliseconds;
    aState.lastBRepCopyMilliseconds =
        myImpl->lastBRepCopyMilliseconds;
    aState.meshSweepRunCount = myImpl->meshSweepRunCount;
    aState.watchdogFireCount = myImpl->watchdogFireCount;
    aState.cacheEntryCount = myImpl->cache.size();
    aState.negativeCacheEntryCount = myImpl->negativeCache.size();
    return aState;
}

TransformInspectorMeasurement
TransformInspectorMeasurementController::capture(
    const std::shared_ptr<ObjectInteractor>& theObjectInteractor,
    const std::shared_ptr<ShapeInteractor>& theShapeInteractor,
    TransformInspectorMeasurementCompletion theCompletion) noexcept
{
    const ScopedTransformInspectorSignpost aCaptureSignpost(
        TransformInspectorSignpost::Capture);
    TransformInspectorMeasurement aMeasurement;
    if (myStopped || pthread_main_np() == 0 || myImpl == nullptr
        || myContext.IsNull() || myDoc.IsNull()) {
        aMeasurement.state =
            TransformInspectorMeasurementState::Invalid;
        return aMeasurement;
    }

    // Every capture is a new logical request. It suppresses any older
    // one-shot completion and retires replaceable pending work even when this
    // request ends synchronously. AddOptimal already running inside OCCT may
    // still finish and seed the same-document cache with its exact result.
    myCompletion = {};
    myImpl->requestedKey.reset();
    if (myImpl->worker != nullptr) {
        const std::optional<std::uint64_t>
            aCancelledPendingGeneration =
                myImpl->worker->cancelAll();
        if (aCancelledPendingGeneration.has_value()) {
            myImpl->forgetOutstandingKey(
                *aCancelledPendingGeneration);
        }
    }
    if (myGeneration == std::numeric_limits<std::uint64_t>::max()) {
        aMeasurement.state =
            TransformInspectorMeasurementState::Invalid;
        return aMeasurement;
    }
    ++myGeneration;
    aMeasurement.generation = myGeneration;
    myObjectInteractor = theObjectInteractor;
    myShapeInteractor = theShapeInteractor;

    try {
        OCC_CATCH_SIGNALS
        const Handle(TDocStd_Document) aDocument = myDoc->Document();
        if (aDocument.IsNull()) {
            return aMeasurement;
        }
        myImpl->clearCacheForDocument(aDocument);

        Handle(AIS_InteractiveObject) aSelected;
        Handle(SelectMgr_EntityOwner) aSelectedOwner;
        myContext->InitSelected();
        for (; myContext->MoreSelected(); myContext->NextSelected()) {
            if (aMeasurement.selectionCount
                == std::numeric_limits<std::size_t>::max()) {
                aMeasurement.state =
                    TransformInspectorMeasurementState::Invalid;
                return aMeasurement;
            }
            ++aMeasurement.selectionCount;
            if (aMeasurement.selectionCount == 1) {
                aSelected = myContext->SelectedInteractive();
                aSelectedOwner = myContext->SelectedOwner();
            }
        }
        if (theObjectInteractor == nullptr
            || theShapeInteractor == nullptr) {
            aMeasurement.state =
                TransformInspectorMeasurementState::Invalid;
            return aMeasurement;
        }
        if (!EnvironmentIsIdle(
                myDoc, theObjectInteractor, theShapeInteractor)) {
            aMeasurement.state =
                TransformInspectorMeasurementState::Busy;
            return aMeasurement;
        }
        if (aMeasurement.selectionCount == 0) {
            aMeasurement.state =
                TransformInspectorMeasurementState::NoSelection;
            return aMeasurement;
        }
        if (aMeasurement.selectionCount != 1) {
            aMeasurement.state =
                TransformInspectorMeasurementState::MultipleSelection;
            return aMeasurement;
        }
        if (theShapeInteractor->getSelectionMode()
                != ShapeSelectionMode::WholeShape
            || aSelected.IsNull() || aSelectedOwner.IsNull()
            || !aSelectedOwner->HasSelectable()
            || aSelectedOwner->Selectable() != aSelected
            || !myDoc->IsPresentationEditable(aSelected)) {
            aMeasurement.state =
                TransformInspectorMeasurementState::Unsupported;
            return aMeasurement;
        }

        const Handle(AIS_Shape) aPresentation =
            Handle(AIS_Shape)::DownCast(aSelected);
        const Handle(StdSelect_BRepOwner) aBRepOwner =
            Handle(StdSelect_BRepOwner)::DownCast(aSelectedOwner);
        const TDF_Label aDefinition = myDoc->ShapeLabel(aSelected);
        if (aPresentation.IsNull() || aBRepOwner.IsNull()
            || !aBRepOwner->HasShape()
            || aDefinition.IsNull()
            || !myDoc->IsEditableFreeSimpleDefinitionLabel(
                aDefinition)) {
            aMeasurement.state =
                TransformInspectorMeasurementState::Unsupported;
            return aMeasurement;
        }

        const TopoDS_Shape aStoredShape =
            XCAFDoc_ShapeTool::GetShape(aDefinition);
        if (aStoredShape.IsNull()
            || aPresentation->Shape().IsNull()
            || !aPresentation->Shape().IsEqual(aStoredShape)
            || !aBRepOwner->Shape().IsEqual(aStoredShape)) {
            // A presentation/owner that does not retain the authoritative
            // stored definition is stale, even when FindShape still resolves.
            aMeasurement.state =
                TransformInspectorMeasurementState::Invalid;
            return aMeasurement;
        }

        const OcctGeometryRepresentation aStoredRepresentation =
            myDoc->StoredGeometryRepresentationForLabel(aDefinition);
        if (!TryInspectorRepresentation(
                aStoredRepresentation,
                aMeasurement.representation,
                aMeasurement.modelCapabilities)) {
            aMeasurement.state =
                TransformInspectorMeasurementState::Invalid;
            return aMeasurement;
        }

        aMeasurement.entityIdentifier =
            myDoc->EntityIdentifierForLabel(aDefinition);
        aMeasurement.definitionIdentifier =
            myDoc->DefinitionIdentifierForLabel(aDefinition);
        if (aMeasurement.entityIdentifier.empty()
            || aMeasurement.definitionIdentifier.empty()) {
            aMeasurement.state =
                TransformInspectorMeasurementState::Invalid;
            return aMeasurement;
        }
        aMeasurement.name = ReadDefinitionName(aDefinition);

        PersistedTransformCapture aPersistedTransform;
        if (!TryCapturePersistedTransform(
                aDefinition, aPersistedTransform)) {
            aMeasurement.state =
                TransformInspectorMeasurementState::Invalid;
            return aMeasurement;
        }
        aMeasurement.position = aPersistedTransform.position;
        aMeasurement.quaternion = aPersistedTransform.quaternion;
        aMeasurement.extrinsicXYZDegrees =
            aPersistedTransform.extrinsicXYZDegrees;
        aMeasurement.uniformScale =
            aPersistedTransform.uniformScale;
        aMeasurement.presentationMatchesDocument = TransformsMatch(
            aPersistedTransform.transform,
            aPresentation->LocalTransformation());

        double aMetersPerUnit = kLegacyMetersPerUnit;
        if (!TryReadMetersPerUnit(aDocument, aMetersPerUnit)) {
            aMeasurement.state =
                TransformInspectorMeasurementState::Invalid;
            return aMeasurement;
        }
        aMeasurement.metersPerUnit = aMetersPerUnit;

        const TransformInspectorBoundsKey aKey{
            aDocument,
            aStoredShape,
        };
        TransformInspectorBoundsValue aBounds;
#ifdef DEBUG
        const bool forceBoundsFailure =
            myImpl->debugForcedBoundsFailure;
#else
        constexpr bool forceBoundsFailure = false;
#endif
        if (!forceBoundsFailure
            && myImpl->lookup(aKey, aBounds)) {
            if (!TryApplyBounds(aBounds, aMeasurement)) {
                aMeasurement.state =
                    TransformInspectorMeasurementState::Invalid;
                return aMeasurement;
            }
            aMeasurement.state =
                TransformInspectorMeasurementState::Ready;
            return aMeasurement;
        }
        TransformInspectorNegativeBoundsReason aNegativeReason =
            TransformInspectorNegativeBoundsReason::ExactBoundsFailed;
        if (!forceBoundsFailure
            && myImpl->lookupNegative(aKey, aNegativeReason)) {
            aMeasurement.state =
                aNegativeReason
                    == TransformInspectorNegativeBoundsReason::SweepAborted
                ? TransformInspectorMeasurementState::BoundsUnavailable
                : TransformInspectorMeasurementState::MeasurementFailed;
            return aMeasurement;
        }

        if (aMeasurement.representation
            == TransformInspectorGeometryRepresentation::TriangleMesh) {
            #ifdef DEBUG
            const Standard_Size aMaximumMeshSweepNodes =
                myImpl->debugMaximumTriangleMeshSweepNodes;
            const Standard_Real aMeshWatchdogDeadlineMilliseconds =
                myImpl->debugMeshSweepWatchdogDeadlineMilliseconds;
            const Standard_Size aMeshWatchdogPollNodes =
                myImpl->debugMeshSweepWatchdogPollNodes;
            #else
            constexpr Standard_Size aMaximumMeshSweepNodes =
                kMaximumTriangleMeshSweepNodes;
            #if TARGET_OS_SIMULATOR
            constexpr Standard_Real aMeshWatchdogDeadlineMilliseconds =
                std::numeric_limits<Standard_Real>::infinity();
            #else
            constexpr Standard_Real aMeshWatchdogDeadlineMilliseconds =
                kMeshSweepWatchdogDeadlineMilliseconds;
            #endif
            constexpr Standard_Size aMeshWatchdogPollNodes =
                kMeshSweepWatchdogPollNodes;
            #endif
            Standard_Size anAdmittedMeshNodeCount = 0;
            const TopologyAdmission aMeshAdmission =
                AdmitTriangleMeshBoundsSweep(
                    aStoredShape,
                    kMaximumTriangleMeshFaces,
                    aMaximumMeshSweepNodes,
                    anAdmittedMeshNodeCount);
            if (aMeshAdmission != TopologyAdmission::WithinLimit) {
                aMeasurement.state =
                    aMeshAdmission == TopologyAdmission::AboveLimit
                    ? TransformInspectorMeasurementState::BoundsUnavailable
                    : TransformInspectorMeasurementState::MeasurementFailed;
                return aMeasurement;
            }
            const Clock::time_point aStart = Clock::now();
            ++myImpl->meshSweepRunCount;
            TriangleMeshSweepOutcome aSweepOutcome =
                TriangleMeshSweepOutcome::Failed;
            if (!forceBoundsFailure) {
                const ScopedTransformInspectorSignpost aSweepSignpost(
                    TransformInspectorSignpost::MeshSweep);
                aSweepOutcome = TrySweepTriangleMeshBounds(
                    aStoredShape,
                    anAdmittedMeshNodeCount,
                    aMeshWatchdogDeadlineMilliseconds,
                    aMeshWatchdogPollNodes,
                    aBounds);
            }
            myImpl->lastMeshSweepMilliseconds =
                MillisecondsSince(aStart);
            if (aSweepOutcome != TriangleMeshSweepOutcome::Success) {
                if (aSweepOutcome
                    == TriangleMeshSweepOutcome::WatchdogAborted) {
                    ++myImpl->watchdogFireCount;
                    myImpl->insertNegative(
                        aKey,
                        TransformInspectorNegativeBoundsReason::SweepAborted);
                    aMeasurement.state =
                        TransformInspectorMeasurementState::BoundsUnavailable;
                } else {
                    myImpl->insertNegative(
                        aKey,
                        TransformInspectorNegativeBoundsReason::
                            ExactBoundsFailed);
                    aMeasurement.state =
                        TransformInspectorMeasurementState::MeasurementFailed;
                }
                return aMeasurement;
            }
            myImpl->insert(aKey, aBounds);
            if (!TryApplyBounds(aBounds, aMeasurement)) {
                aMeasurement.state =
                    TransformInspectorMeasurementState::Invalid;
                return aMeasurement;
            }
            aMeasurement.state =
                TransformInspectorMeasurementState::Ready;
            return aMeasurement;
        }

#ifdef DEBUG
        const Standard_Size aMaximumTopologyNodes =
            myImpl->debugMaximumBRepTopologyNodes;
#else
        constexpr Standard_Size aMaximumTopologyNodes =
            kMaximumBRepTopologyNodes;
#endif
        Standard_Size aTopologyNodeCount = 0;
        const TopologyAdmission aTopologyAdmission =
            CountBoundedTopology(
                aStoredShape,
                aMaximumTopologyNodes,
                aTopologyNodeCount);
        if (aTopologyAdmission != TopologyAdmission::WithinLimit) {
            aMeasurement.state =
                aTopologyAdmission == TopologyAdmission::AboveLimit
                ? TransformInspectorMeasurementState::BoundsUnavailable
                : TransformInspectorMeasurementState::MeasurementFailed;
            return aMeasurement;
        }

        const Clock::time_point aCopyStart = Clock::now();
        TopoDS_Shape aCopiedShape;
        try {
            OCC_CATCH_SIGNALS
            const ScopedTransformInspectorSignpost aCopySignpost(
                TransformInspectorSignpost::BRepCopy);
            BRepBuilderAPI_Copy aCopy(
                aStoredShape,
                Standard_True,
                Standard_False);
            aCopiedShape = aCopy.Shape();
        } catch (...) {
            aCopiedShape.Nullify();
        }
        myImpl->lastBRepCopyMilliseconds =
            MillisecondsSince(aCopyStart);
        if (aCopiedShape.IsNull()) {
            myImpl->insertNegative(
                aKey,
                TransformInspectorNegativeBoundsReason::CopyFailed);
            aMeasurement.state =
                TransformInspectorMeasurementState::MeasurementFailed;
            return aMeasurement;
        }

        if (myImpl->worker == nullptr) {
            myImpl->worker =
                std::make_shared<TransformInspectorBoundsWorker>(
                    weak_from_this());
#ifdef DEBUG
            myImpl->worker->debugSetBlocked(
                myImpl->debugWorkerBlocked);
#endif
        }
        TransformInspectorBoundsWorkerRequest aRequest;
        aRequest.generation = aMeasurement.generation;
        aRequest.copiedShape = std::move(aCopiedShape);
        aRequest.cancellation =
            std::make_shared<std::atomic_bool>(false);
        aRequest.forceFailure = forceBoundsFailure;
        myImpl->requestedKey = aKey;
        myImpl->pendingMeasurement = aMeasurement;
        myCompletion = std::move(theCompletion);
        if (!myImpl->rememberOutstandingKey(
                aMeasurement.generation, aKey)) {
            myImpl->requestedKey.reset();
            myCompletion = {};
            aMeasurement.state =
                TransformInspectorMeasurementState::MeasurementFailed;
            return aMeasurement;
        }
        const TransformInspectorBoundsWorkerEnqueueResult anEnqueue =
            myImpl->worker->enqueue(std::move(aRequest));
        if (!anEnqueue.accepted) {
            myImpl->forgetOutstandingKey(aMeasurement.generation);
            myImpl->requestedKey.reset();
            myCompletion = {};
            aMeasurement.state =
                TransformInspectorMeasurementState::MeasurementFailed;
            return aMeasurement;
        }
        if (anEnqueue.replacedPendingGeneration.has_value()) {
            myImpl->forgetOutstandingKey(
                *anEnqueue.replacedPendingGeneration);
        }
        aMeasurement.state =
            TransformInspectorMeasurementState::Measuring;
        myImpl->pendingMeasurement.state = aMeasurement.state;
        return aMeasurement;
    } catch (...) {
        myCompletion = {};
        myImpl->requestedKey.reset();
        aMeasurement.state =
            TransformInspectorMeasurementState::Invalid;
        return aMeasurement;
    }
}

void TransformInspectorMeasurementController::acceptWorkerResult(
    TransformInspectorBoundsWorkerResult theResult) noexcept
{
    const ScopedTransformInspectorSignpost anAcceptSignpost(
        TransformInspectorSignpost::Accept);
    if (myStopped || myImpl == nullptr || pthread_main_np() == 0) {
        return;
    }

    const std::optional<TransformInspectorBoundsKey> aResultKey =
        myImpl->takeOutstandingKey(theResult.generation);
    if (!aResultKey.has_value()) {
#ifdef DEBUG
        ++myImpl->staleCount;
#endif
        return;
    }

    Handle(TDocStd_Document) aCurrentDocument;
    try {
        aCurrentDocument = myDoc.IsNull()
            ? Handle(TDocStd_Document)()
            : myDoc->Document();
    } catch (...) {
        aCurrentDocument.Nullify();
    }
    // Document replacement is a hard cache boundary. Same-document results
    // remain useful after selection/request staleness, but an old document
    // must never be retained again after that boundary has moved.
    myImpl->clearCacheForDocument(aCurrentDocument);
    if (theResult.outcome == BoundsWorkerOutcome::Success
        && !aCurrentDocument.IsNull()
        && aCurrentDocument == aResultKey->document) {
        myImpl->insert(*aResultKey, theResult.bounds);
    } else if (theResult.outcome == BoundsWorkerOutcome::Failed
               && !aCurrentDocument.IsNull()
               && aCurrentDocument == aResultKey->document) {
        myImpl->insertNegative(
            *aResultKey,
            TransformInspectorNegativeBoundsReason::ExactBoundsFailed);
    }

    bool isCurrent = theResult.generation == myGeneration
        && myImpl->requestedKey.has_value()
        && BoundsKeysMatch(*myImpl->requestedKey, *aResultKey);
    const auto anObjectInteractor = myObjectInteractor.lock();
    const auto aShapeInteractor = myShapeInteractor.lock();
    if (isCurrent) {
        isCurrent = EnvironmentIsIdle(
            myDoc, anObjectInteractor, aShapeInteractor)
            && aShapeInteractor->getSelectionMode()
                == ShapeSelectionMode::WholeShape;
    }

    if (isCurrent) {
        try {
            OCC_CATCH_SIGNALS
            const Handle(TDocStd_Document) aDocument =
                myDoc.IsNull()
                ? Handle(TDocStd_Document)()
                : myDoc->Document();
            Handle(AIS_InteractiveObject) aSelected;
            Handle(SelectMgr_EntityOwner) aSelectedOwner;
            std::size_t aSelectionCount = 0;
            if (aDocument != aResultKey->document
                || myContext.IsNull()) {
                isCurrent = false;
            } else {
                for (myContext->InitSelected();
                     myContext->MoreSelected();
                     myContext->NextSelected()) {
                    ++aSelectionCount;
                    if (aSelectionCount == 1) {
                        aSelected = myContext->SelectedInteractive();
                        aSelectedOwner = myContext->SelectedOwner();
                    }
                }
                const TDF_Label aLabel =
                    aSelectionCount == 1 && !aSelected.IsNull()
                    ? myDoc->ShapeLabel(aSelected)
                    : TDF_Label();
                const TopoDS_Shape aStoredShape = aLabel.IsNull()
                    ? TopoDS_Shape()
                    : XCAFDoc_ShapeTool::GetShape(aLabel);
                const Handle(AIS_Shape) aPresentation =
                    Handle(AIS_Shape)::DownCast(aSelected);
                const Handle(StdSelect_BRepOwner) aBRepOwner =
                    Handle(StdSelect_BRepOwner)::DownCast(
                        aSelectedOwner);
                PersistedTransformCapture aCurrentTransform;
                const bool hasCurrentTransform =
                    TryCapturePersistedTransform(
                        aLabel, aCurrentTransform);
                const TransformInspectorMeasurement& aPending =
                    myImpl->pendingMeasurement;
                TransformInspectorGeometryRepresentation
                    aCurrentRepresentation =
                        TransformInspectorGeometryRepresentation::Invalid;
                std::uint64_t aCurrentCapabilities = 0;
                const bool hasCurrentRepresentation =
                    !aLabel.IsNull()
                    && TryInspectorRepresentation(
                        myDoc->StoredGeometryRepresentationForLabel(aLabel),
                        aCurrentRepresentation,
                        aCurrentCapabilities);
                double aCurrentMetersPerUnit = 0.0;
                const bool hasCurrentMetersPerUnit =
                    TryReadMetersPerUnit(
                        aDocument, aCurrentMetersPerUnit);
                const std::string aCurrentName =
                    ReadDefinitionName(aLabel);
                const auto close = [](const double theLeft,
                                      const double theRight) {
                    const double aMagnitude = std::max({
                        1.0, std::abs(theLeft), std::abs(theRight)});
                    return std::isfinite(theLeft)
                        && std::isfinite(theRight)
                        && std::abs(theLeft - theRight)
                            <= Precision::Confusion() * aMagnitude;
                };
                const bool transformIsCurrent = hasCurrentTransform
                    && close(aCurrentTransform.position.x,
                             aPending.position.x)
                    && close(aCurrentTransform.position.y,
                             aPending.position.y)
                    && close(aCurrentTransform.position.z,
                             aPending.position.z)
                    && close(aCurrentTransform.quaternion.x,
                             aPending.quaternion.x)
                    && close(aCurrentTransform.quaternion.y,
                             aPending.quaternion.y)
                    && close(aCurrentTransform.quaternion.z,
                             aPending.quaternion.z)
                    && close(aCurrentTransform.quaternion.w,
                             aPending.quaternion.w)
                    && close(aCurrentTransform.uniformScale,
                             aPending.uniformScale);
                isCurrent = aSelectionCount == 1
                    && !aStoredShape.IsNull()
                    && aStoredShape.IsEqual(aResultKey->shape)
                    && !aPresentation.IsNull()
                    && !aPresentation->Shape().IsNull()
                    && aPresentation->Shape().IsEqual(aStoredShape)
                    && !aBRepOwner.IsNull()
                    && aBRepOwner->HasShape()
                    && aBRepOwner->Shape().IsEqual(aStoredShape)
                    && aBRepOwner->HasSelectable()
                    && aBRepOwner->Selectable() == aSelected
                    && myDoc->IsPresentationEditable(aSelected)
                    && myDoc->IsEditableFreeSimpleDefinitionLabel(aLabel)
                    && myDoc->EntityIdentifierForLabel(aLabel)
                        == aPending.entityIdentifier
                    && myDoc->DefinitionIdentifierForLabel(aLabel)
                        == aPending.definitionIdentifier
                    && aCurrentName == aPending.name
                    && hasCurrentRepresentation
                    && aCurrentRepresentation
                        == aPending.representation
                    && aCurrentCapabilities
                        == aPending.modelCapabilities
                    && hasCurrentMetersPerUnit
                    && close(aCurrentMetersPerUnit,
                             aPending.metersPerUnit)
                    && transformIsCurrent;
                if (isCurrent) {
                    myImpl->pendingMeasurement
                        .presentationMatchesDocument = TransformsMatch(
                            aCurrentTransform.transform,
                            aPresentation->LocalTransformation());
                }
            }
        } catch (...) {
            isCurrent = false;
        }
    }

    if (!isCurrent) {
#ifdef DEBUG
        ++myImpl->staleCount;
#endif
        if (theResult.generation == myGeneration) {
            myImpl->requestedKey.reset();
            myCompletion = {};
        }
        return;
    }

    TransformInspectorMeasurement aMeasurement =
        myImpl->pendingMeasurement;
    if (theResult.outcome == BoundsWorkerOutcome::Success) {
        if (TryApplyBounds(theResult.bounds, aMeasurement)) {
            aMeasurement.state =
                TransformInspectorMeasurementState::Ready;
        } else {
            aMeasurement.state =
                TransformInspectorMeasurementState::MeasurementFailed;
        }
    } else {
        aMeasurement.state =
            TransformInspectorMeasurementState::MeasurementFailed;
    }
    myImpl->requestedKey.reset();
    TransformInspectorMeasurementCompletion aCompletion =
        std::move(myCompletion);
    myCompletion = {};
#ifdef DEBUG
    ++myImpl->acceptedCount;
#endif
    if (aCompletion) {
        try {
            aCompletion(std::move(aMeasurement));
        } catch (...) {
        }
    }
}

#ifdef DEBUG
TransformInspectorMeasurementDebugState
TransformInspectorMeasurementController::debugState() const noexcept
{
    TransformInspectorMeasurementDebugState aState;
    aState.generation = myGeneration;
    if (myImpl == nullptr) {
        return aState;
    }
    aState.acceptedCount = myImpl->acceptedCount;
    aState.staleCount = myImpl->staleCount;
    aState.workerBlocked = myImpl->debugWorkerBlocked;
    aState.forcedBoundsFailure =
        myImpl->debugForcedBoundsFailure;
    aState.lastMeshSweepMilliseconds =
        myImpl->lastMeshSweepMilliseconds;
    aState.lastBRepCopyMilliseconds =
        myImpl->lastBRepCopyMilliseconds;
    aState.maximumBRepTopologyNodes =
        myImpl->debugMaximumBRepTopologyNodes;
    aState.maximumTriangleMeshSweepNodes =
        myImpl->debugMaximumTriangleMeshSweepNodes;
    aState.meshSweepWatchdogDeadlineMilliseconds =
        myImpl->debugMeshSweepWatchdogDeadlineMilliseconds;
    aState.meshSweepWatchdogPollNodes =
        myImpl->debugMeshSweepWatchdogPollNodes;
    aState.meshSweepRunCount = myImpl->meshSweepRunCount;
    aState.watchdogFireCount = myImpl->watchdogFireCount;
    aState.cacheEntryCount = myImpl->cache.size();
    aState.negativeCacheEntryCount = myImpl->negativeCache.size();
    if (myImpl->worker != nullptr) {
        const TransformInspectorBoundsWorker::DebugSnapshot aWorker =
            myImpl->worker->debugSnapshot();
        aState.submittedCount = aWorker.submittedCount;
        aState.startedCount = aWorker.startedCount;
        aState.completedCount = aWorker.completedCount;
        aState.cancelledOrSupersededCount =
            aWorker.cancelledOrSupersededCount;
        aState.pendingReplacementCount =
            aWorker.pendingReplacementCount;
        aState.workerActive = aWorker.active;
        aState.workerPending = aWorker.pending;
        aState.lastComputeWasMainThread =
            aWorker.lastComputeWasMainThread;
        aState.workerBlocked = aWorker.blocked;
    }
    return aState;
}

void TransformInspectorMeasurementController::debugSetWorkerBlocked(
    const Standard_Boolean theBlocked) noexcept
{
    if (myImpl == nullptr) {
        return;
    }
    myImpl->debugWorkerBlocked = theBlocked;
    if (myImpl->worker != nullptr) {
        myImpl->worker->debugSetBlocked(theBlocked);
    }
}

void TransformInspectorMeasurementController::
debugSetForcedBoundsFailure(
    const Standard_Boolean theFailure) noexcept
{
    if (myImpl != nullptr) {
        myImpl->debugForcedBoundsFailure = theFailure;
        if (!theFailure) {
            myImpl->negativeCache.clear();
        }
    }
}

void TransformInspectorMeasurementController::
debugSetMaximumBRepTopologyNodes(
    const Standard_Size theLimit) noexcept
{
    if (myImpl != nullptr) {
        const Standard_Size aResolvedLimit =
            std::max<Standard_Size>(
                1,
                std::min(theLimit, kMaximumBRepTopologyNodes));
        if (myImpl->debugMaximumBRepTopologyNodes != aResolvedLimit) {
            // A DEBUG admission-policy change is a hard request boundary.
            // An AddOptimal call already inside OCCT may still return, but its
            // old key must not repopulate a cache governed by the new cap.
            cancelPendingMeasurement();
            myImpl->outstandingKeys.clear();
            myImpl->debugMaximumBRepTopologyNodes = aResolvedLimit;
            myImpl->cache.clear();
            myImpl->negativeCache.clear();
        }
    }
}


void TransformInspectorMeasurementController::
debugSetMaximumTriangleMeshSweepNodes(
    const Standard_Size theLimit) noexcept
{
    if (myImpl != nullptr) {
        const Standard_Size aResolvedLimit =
            std::max<Standard_Size>(
                1,
                std::min(theLimit, kMaximumTriangleMeshSweepNodes));
        if (myImpl->debugMaximumTriangleMeshSweepNodes
            != aResolvedLimit) {
            cancelPendingMeasurement();
            myImpl->outstandingKeys.clear();
            myImpl->debugMaximumTriangleMeshSweepNodes = aResolvedLimit;
            myImpl->cache.clear();
            myImpl->negativeCache.clear();
        }
    }
}

void TransformInspectorMeasurementController::
debugSetMeshSweepWatchdog(
    const Standard_Real theDeadlineMilliseconds,
    const Standard_Size thePollNodes) noexcept
{
    if (myImpl == nullptr) {
        return;
    }
    const Standard_Real aResolvedDeadline =
        std::isfinite(theDeadlineMilliseconds)
        ? std::max<Standard_Real>(0.0, theDeadlineMilliseconds)
        : std::numeric_limits<Standard_Real>::infinity();
    const Standard_Size aResolvedPollNodes =
        std::max<Standard_Size>(1, thePollNodes);
    if (myImpl->debugMeshSweepWatchdogDeadlineMilliseconds
            != aResolvedDeadline
        || myImpl->debugMeshSweepWatchdogPollNodes
            != aResolvedPollNodes) {
        cancelPendingMeasurement();
        myImpl->outstandingKeys.clear();
        myImpl->debugMeshSweepWatchdogDeadlineMilliseconds =
            aResolvedDeadline;
        myImpl->debugMeshSweepWatchdogPollNodes = aResolvedPollNodes;
        myImpl->cache.clear();
        myImpl->negativeCache.clear();
    }
}
#endif

} // namespace core3d
