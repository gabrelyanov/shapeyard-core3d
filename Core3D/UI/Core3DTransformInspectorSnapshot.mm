//
//  Core3DTransformInspectorSnapshot.mm
//  Core3D
//


#import "Core3DTransformInspectorSnapshot.h"
#import "Core3DTransformInspectorSnapshotFactory.hpp"

#include "TransformInspectorMeasurementController.hpp"

#include <gp_Quaternion.hxx>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

@interface Core3DTransformInspectorSnapshot ()

- (instancetype)initWithState:(Core3DTransformInspectorState)state
                 selectedCount:(NSUInteger)selectedCount
             requestGeneration:(uint64_t)requestGeneration
              entityIdentifier:(nullable NSString *)entityIdentifier
          definitionIdentifier:(nullable NSString *)definitionIdentifier
                   displayName:(nullable NSString *)displayName
                representation:(Core3DTransformInspectorRepresentation)representation
                      position:(simd_double3)position
                quaternionXYZW:(simd_double4)quaternionXYZW
      canonicalEulerXYZDegrees:(simd_double3)canonicalEulerXYZDegrees
                  uniformScale:(double)uniformScale
                 metersPerUnit:(double)metersPerUnit
            localBoundsMinimum:(simd_double3)localBoundsMinimum
            localBoundsMaximum:(simd_double3)localBoundsMaximum
                    dimensions:(simd_double3)dimensions
                 hasDimensions:(BOOL)hasDimensions
   presentationMatchesDocument:(BOOL)presentationMatchesDocument
             modelCapabilities:(Core3DModelCapability)modelCapabilities;

@end

@implementation Core3DTransformInspectorSnapshot

- (instancetype)initWithState:(Core3DTransformInspectorState)state
                 selectedCount:(NSUInteger)selectedCount
             requestGeneration:(uint64_t)requestGeneration
              entityIdentifier:(NSString *)entityIdentifier
          definitionIdentifier:(NSString *)definitionIdentifier
                   displayName:(NSString *)displayName
                representation:(Core3DTransformInspectorRepresentation)representation
                      position:(simd_double3)position
                quaternionXYZW:(simd_double4)quaternionXYZW
      canonicalEulerXYZDegrees:(simd_double3)canonicalEulerXYZDegrees
                  uniformScale:(double)uniformScale
                 metersPerUnit:(double)metersPerUnit
            localBoundsMinimum:(simd_double3)localBoundsMinimum
            localBoundsMaximum:(simd_double3)localBoundsMaximum
                    dimensions:(simd_double3)dimensions
                 hasDimensions:(BOOL)hasDimensions
   presentationMatchesDocument:(BOOL)presentationMatchesDocument
             modelCapabilities:(Core3DModelCapability)modelCapabilities {
    self = [super init];
    if (self) {
        _state = state;
        _selectedCount = selectedCount;
        _requestGeneration = requestGeneration;
        _entityIdentifier = [entityIdentifier copy];
        _definitionIdentifier = [definitionIdentifier copy];
        _displayName = [displayName copy];
        _representation = representation;
        _position = position;
        _quaternionXYZW = quaternionXYZW;
        _canonicalEulerXYZDegrees = canonicalEulerXYZDegrees;
        _uniformScale = uniformScale;
        _metersPerUnit = metersPerUnit;
        _localBoundsMinimum = localBoundsMinimum;
        _localBoundsMaximum = localBoundsMaximum;
        _dimensions = dimensions;
        _hasDimensions = hasDimensions;
        _presentationMatchesDocument = presentationMatchesDocument;
        _modelCapabilities = modelCapabilities;
    }
    return self;
}

@end


namespace {

constexpr std::uint64_t kBRepModelCapabilities =
    static_cast<std::uint64_t>(Core3DModelCapabilityObjectSelection)
    | static_cast<std::uint64_t>(Core3DModelCapabilitySubshapeSelection)
    | static_cast<std::uint64_t>(Core3DModelCapabilityTranslate)
    | static_cast<std::uint64_t>(Core3DModelCapabilityRotate)
    | static_cast<std::uint64_t>(Core3DModelCapabilityUniformScale)
    | static_cast<std::uint64_t>(Core3DModelCapabilityNonuniformScale)
    | static_cast<std::uint64_t>(Core3DModelCapabilityDelete)
    | static_cast<std::uint64_t>(Core3DModelCapabilityDuplicate)
    | static_cast<std::uint64_t>(Core3DModelCapabilityMirror)
    | static_cast<std::uint64_t>(Core3DModelCapabilityBoolean)
    | static_cast<std::uint64_t>(Core3DModelCapabilityChamfer)
    | static_cast<std::uint64_t>(Core3DModelCapabilityExtrusion)
    | static_cast<std::uint64_t>(Core3DModelCapabilityMaterial)
    | static_cast<std::uint64_t>(Core3DModelCapabilityExportOBJ)
    | static_cast<std::uint64_t>(Core3DModelCapabilityExportSTL)
    | static_cast<std::uint64_t>(Core3DModelCapabilityExportGLB)
    | static_cast<std::uint64_t>(Core3DModelCapabilityExportSTEP);
constexpr std::uint64_t kTriangleMeshModelCapabilities =
    static_cast<std::uint64_t>(Core3DModelCapabilityObjectSelection)
    | static_cast<std::uint64_t>(Core3DModelCapabilityTranslate)
    | static_cast<std::uint64_t>(Core3DModelCapabilityRotate)
    | static_cast<std::uint64_t>(Core3DModelCapabilityDelete)
    | static_cast<std::uint64_t>(Core3DModelCapabilityDuplicate)
    | static_cast<std::uint64_t>(Core3DModelCapabilityMaterial)
    | static_cast<std::uint64_t>(Core3DModelCapabilityExportOBJ)
    | static_cast<std::uint64_t>(Core3DModelCapabilityExportSTL)
    | static_cast<std::uint64_t>(Core3DModelCapabilityExportGLB);
constexpr double kQuaternionTolerance = 1.0e-8;
constexpr double kDimensionTolerance = 1.0e-9;
constexpr double kDegreesToRadians =
    0.017453292519943295769236907684886;
constexpr double kEulerRotationTolerance = 1.0e-10;
constexpr double kEulerGimbalRotationTolerance = 1.0e-8;
constexpr double kEulerGimbalDetectionToleranceDegrees = 1.0e-7;

bool Core3DIsFinite(const core3d::TransformInspectorVector3& value) noexcept {
    return std::isfinite(value.x)
        && std::isfinite(value.y)
        && std::isfinite(value.z);
}

simd_double3 Core3DMakeDouble3(
    const core3d::TransformInspectorVector3& value) noexcept {
    return (simd_double3){value.x, value.y, value.z};
}

bool Core3DNearlyEqual(const double first, const double second) noexcept {
    const double magnitude = std::max(
        {1.0, std::abs(first), std::abs(second)});
    return std::abs(first - second) <= kDimensionTolerance * magnitude;
}

NSString *_Nullable Core3DStringFromUTF8(
    const std::string& value,
    const bool permitEmpty) {
    if ((!permitEmpty && value.empty())
        || value.find('\0') != std::string::npos) {
        return nil;
    }
    if (value.empty()) {
        return @"";
    }
    return [[NSString alloc] initWithBytes:value.data()
                                    length:value.size()
                                  encoding:NSUTF8StringEncoding];
}

Core3DTransformInspectorSnapshot *Core3DMakeStateSnapshot(
    const Core3DTransformInspectorState state,
    const NSUInteger selectedCount,
    const std::uint64_t generation) {
    const simd_double3 zero3 = (simd_double3){0.0, 0.0, 0.0};
    return [[Core3DTransformInspectorSnapshot alloc]
        initWithState:state
        selectedCount:selectedCount
        requestGeneration:generation
        entityIdentifier:nil
        definitionIdentifier:nil
        displayName:nil
        representation:Core3DTransformInspectorRepresentationUnknown
        position:zero3
        quaternionXYZW:(simd_double4){0.0, 0.0, 0.0, 1.0}
        canonicalEulerXYZDegrees:zero3
        uniformScale:1.0
        metersPerUnit:0.0
        localBoundsMinimum:zero3
        localBoundsMaximum:zero3
        dimensions:zero3
        hasDimensions:NO
        presentationMatchesDocument:NO
        modelCapabilities:Core3DModelCapabilityNone];
}

Core3DTransformInspectorState Core3DPublicState(
    const core3d::TransformInspectorMeasurementState state) noexcept {
    using NativeState = core3d::TransformInspectorMeasurementState;
    switch (state) {
        case NativeState::NoSelection:
            return Core3DTransformInspectorStateNoSelection;
        case NativeState::MultipleSelection:
            return Core3DTransformInspectorStateMultipleSelection;
        case NativeState::Busy:
            return Core3DTransformInspectorStateBusy;
        case NativeState::Unsupported:
            return Core3DTransformInspectorStateUnsupportedSelection;
        case NativeState::Invalid:
            return Core3DTransformInspectorStateInvalid;
        case NativeState::Measuring:
            return Core3DTransformInspectorStateMeasuring;
        case NativeState::Ready:
            return Core3DTransformInspectorStateReady;
        case NativeState::BoundsUnavailable:
            return Core3DTransformInspectorStateMeasurementUnavailable;
        case NativeState::MeasurementFailed:
            return Core3DTransformInspectorStateMeasurementFailed;
    }
    return Core3DTransformInspectorStateInvalid;
}

bool Core3DHasMeaningfulSingleSelection(
    const Core3DTransformInspectorState state) noexcept {
    switch (state) {
        case Core3DTransformInspectorStateMeasuring:
        case Core3DTransformInspectorStateReady:
        case Core3DTransformInspectorStateMeasurementUnavailable:
        case Core3DTransformInspectorStateMeasurementFailed:
            return true;
        default:
            return false;
    }
}

bool Core3DSelectionCountIsValid(
    const Core3DTransformInspectorState state,
    const std::size_t count) noexcept {
    if (Core3DHasMeaningfulSingleSelection(state)) {
        return count == 1;
    }
    switch (state) {
        case Core3DTransformInspectorStateNoSelection:
            return count == 0;
        case Core3DTransformInspectorStateMultipleSelection:
            return count > 1;
        case Core3DTransformInspectorStateUnsupportedSelection:
            return count > 0;
        default:
            return true;
    }
}

Core3DTransformInspectorRepresentation Core3DPublicRepresentation(
    const core3d::TransformInspectorGeometryRepresentation representation)
    noexcept {
    using NativeRepresentation =
        core3d::TransformInspectorGeometryRepresentation;
    switch (representation) {
        case NativeRepresentation::BRep:
            return Core3DTransformInspectorRepresentationBRep;
        case NativeRepresentation::TriangleMesh:
            return Core3DTransformInspectorRepresentationTriangleMesh;
        case NativeRepresentation::Invalid:
            return Core3DTransformInspectorRepresentationUnknown;
    }
    return Core3DTransformInspectorRepresentationUnknown;
}

bool Core3DHasValidQuaternion(
    const core3d::TransformInspectorQuaternion& quaternion) noexcept {
    if (!std::isfinite(quaternion.x) || !std::isfinite(quaternion.y)
        || !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w)) {
        return false;
    }
    const double normSquared = quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w;
    return std::isfinite(normSquared)
        && std::abs(normSquared - 1.0) <= kQuaternionTolerance;
}

bool Core3DHasCanonicalEulerAngles(
    const core3d::TransformInspectorVector3& euler) noexcept {
    return Core3DIsFinite(euler)
        && euler.x >= -180.0 && euler.x < 180.0
        && euler.y >= -90.0 && euler.y <= 90.0
        && euler.z >= -180.0 && euler.z < 180.0;
}

bool Core3DQuaternionMatchesEuler(
    const core3d::TransformInspectorQuaternion& quaternion,
    const core3d::TransformInspectorVector3& euler) noexcept {
    try {
        const double quaternionNorm = std::sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w);
        if (!std::isfinite(quaternionNorm)
            || quaternionNorm
                <= std::numeric_limits<double>::epsilon()) {
            return false;
        }
        const double actual[4] = {
            quaternion.x / quaternionNorm,
            quaternion.y / quaternionNorm,
            quaternion.z / quaternionNorm,
            quaternion.w / quaternionNorm,
        };

        gp_Quaternion reconstructed;
        reconstructed.SetEulerAngles(
            gp_Extrinsic_XYZ,
            euler.x * kDegreesToRadians,
            euler.y * kDegreesToRadians,
            euler.z * kDegreesToRadians);
        const double reconstructedNorm = reconstructed.Norm();
        if (!std::isfinite(reconstructedNorm)
            || reconstructedNorm
                <= std::numeric_limits<double>::epsilon()) {
            return false;
        }
        const double expected[4] = {
            reconstructed.X() / reconstructedNorm,
            reconstructed.Y() / reconstructedNorm,
            reconstructed.Z() / reconstructedNorm,
            reconstructed.W() / reconstructedNorm,
        };
        double sameSignDistanceSquared = 0.0;
        double oppositeSignDistanceSquared = 0.0;
        for (std::size_t component = 0; component < 4; ++component) {
            const double sameSign = actual[component] - expected[component];
            const double oppositeSign = actual[component] + expected[component];
            sameSignDistanceSquared += sameSign * sameSign;
            oppositeSignDistanceSquared += oppositeSign * oppositeSign;
        }
        const bool isAtGimbalLock =
            std::abs(std::abs(euler.y) - 90.0)
                <= kEulerGimbalDetectionToleranceDegrees;
        const double tolerance = isAtGimbalLock
            ? kEulerGimbalRotationTolerance
            : kEulerRotationTolerance;
        return std::min(
                   sameSignDistanceSquared,
                   oppositeSignDistanceSquared)
            <= tolerance * tolerance;
    } catch (...) {
        return false;
    }
}

bool Core3DHasExpectedCapabilities(
    const Core3DTransformInspectorRepresentation representation,
    const std::uint64_t capabilities) noexcept {
    switch (representation) {
        case Core3DTransformInspectorRepresentationBRep:
            return capabilities == kBRepModelCapabilities;
        case Core3DTransformInspectorRepresentationTriangleMesh:
            return capabilities == kTriangleMeshModelCapabilities;
        case Core3DTransformInspectorRepresentationUnknown:
            return false;
    }
    return false;
}

bool Core3DHasValidDimensions(
    const core3d::TransformInspectorMeasurement& measurement) noexcept {
    if (!Core3DIsFinite(measurement.localDimensions)
        || !Core3DIsFinite(measurement.dimensions)) {
        return false;
    }
    for (const double coordinate : measurement.localBounds) {
        if (!std::isfinite(coordinate)) {
            return false;
        }
    }
    for (std::size_t axis = 0; axis < 3; ++axis) {
        const double minimum = measurement.localBounds[axis];
        const double maximum = measurement.localBounds[axis + 3];
        const double localDimension = axis == 0
            ? measurement.localDimensions.x
            : axis == 1
                ? measurement.localDimensions.y
                : measurement.localDimensions.z;
        const double dimension = axis == 0
            ? measurement.dimensions.x
            : axis == 1
                ? measurement.dimensions.y
                : measurement.dimensions.z;
        if (maximum < minimum || localDimension < 0.0 || dimension < 0.0
            || !Core3DNearlyEqual(localDimension, maximum - minimum)
            || !Core3DNearlyEqual(
                dimension,
                localDimension * std::abs(measurement.uniformScale))) {
            return false;
        }
    }
    return true;
}

} // namespace


Core3DTransformInspectorSnapshot *Core3DCreateTransformInspectorStateSnapshotDTO(
    const Core3DTransformInspectorState state) noexcept {
    try {
        switch (state) {
            case Core3DTransformInspectorStateUnavailable:
            case Core3DTransformInspectorStateNotSetup:
            case Core3DTransformInspectorStateInvalid:
                return Core3DMakeStateSnapshot(state, 0, 0);
            default:
                return Core3DMakeStateSnapshot(
                    Core3DTransformInspectorStateInvalid, 0, 0);
        }
    } catch (...) {
        return Core3DMakeStateSnapshot(
            Core3DTransformInspectorStateInvalid, 0, 0);
    }
}

Core3DTransformInspectorSnapshot *Core3DCreateTransformInspectorSnapshotDTO(
    const core3d::TransformInspectorMeasurement& measurement) noexcept {
    try {
        const Core3DTransformInspectorState state =
            Core3DPublicState(measurement.state);
        if (measurement.selectionCount
                > std::numeric_limits<NSUInteger>::max()
            || !Core3DSelectionCountIsValid(
                state, measurement.selectionCount)) {
            return Core3DMakeStateSnapshot(
                Core3DTransformInspectorStateInvalid, 0,
                measurement.generation);
        }
        const NSUInteger selectedCount =
            static_cast<NSUInteger>(measurement.selectionCount);
        if (!Core3DHasMeaningfulSingleSelection(state)) {
            return Core3DMakeStateSnapshot(
                state, selectedCount, measurement.generation);
        }

        NSString *entityIdentifier = Core3DStringFromUTF8(
            measurement.entityIdentifier, false);
        NSString *definitionIdentifier = Core3DStringFromUTF8(
            measurement.definitionIdentifier, false);
        NSString *displayName = Core3DStringFromUTF8(
            measurement.name, true);
        const Core3DTransformInspectorRepresentation representation =
            Core3DPublicRepresentation(measurement.representation);
        const bool hasExpectedCapabilities =
            measurement.modelCapabilities
                <= std::numeric_limits<NSUInteger>::max()
            && Core3DHasExpectedCapabilities(
                representation, measurement.modelCapabilities);
        if (entityIdentifier == nil || definitionIdentifier == nil
            || displayName == nil
            || representation
                == Core3DTransformInspectorRepresentationUnknown
            || (state == Core3DTransformInspectorStateMeasuring
                && representation
                    != Core3DTransformInspectorRepresentationBRep)
            || !Core3DIsFinite(measurement.position)
            || !Core3DHasCanonicalEulerAngles(
                measurement.extrinsicXYZDegrees)
            || !Core3DHasValidQuaternion(measurement.quaternion)
            || !Core3DQuaternionMatchesEuler(
                measurement.quaternion,
                measurement.extrinsicXYZDegrees)
            || !std::isfinite(measurement.uniformScale)
            || measurement.uniformScale == 0.0
            || !std::isfinite(measurement.metersPerUnit)
            || measurement.metersPerUnit <= 0.0
            || !hasExpectedCapabilities) {
            return Core3DMakeStateSnapshot(
                Core3DTransformInspectorStateInvalid, selectedCount,
                measurement.generation);
        }

        const BOOL hasDimensions =
            state == Core3DTransformInspectorStateReady;
        if (hasDimensions && !Core3DHasValidDimensions(measurement)) {
            return Core3DMakeStateSnapshot(
                Core3DTransformInspectorStateInvalid, selectedCount,
                measurement.generation);
        }
        const simd_double3 zero3 = (simd_double3){0.0, 0.0, 0.0};
        const simd_double3 boundsMinimum = hasDimensions
            ? (simd_double3){measurement.localBounds[0],
                             measurement.localBounds[1],
                             measurement.localBounds[2]}
            : zero3;
        const simd_double3 boundsMaximum = hasDimensions
            ? (simd_double3){measurement.localBounds[3],
                             measurement.localBounds[4],
                             measurement.localBounds[5]}
            : zero3;
        const simd_double3 dimensions = hasDimensions
            ? Core3DMakeDouble3(measurement.dimensions)
            : zero3;

        return [[Core3DTransformInspectorSnapshot alloc]
            initWithState:state
            selectedCount:selectedCount
            requestGeneration:measurement.generation
            entityIdentifier:entityIdentifier
            definitionIdentifier:definitionIdentifier
            displayName:displayName
            representation:representation
            position:Core3DMakeDouble3(measurement.position)
            quaternionXYZW:(simd_double4){measurement.quaternion.x,
                                          measurement.quaternion.y,
                                          measurement.quaternion.z,
                                          measurement.quaternion.w}
            canonicalEulerXYZDegrees:
                Core3DMakeDouble3(measurement.extrinsicXYZDegrees)
            uniformScale:measurement.uniformScale
            metersPerUnit:measurement.metersPerUnit
            localBoundsMinimum:boundsMinimum
            localBoundsMaximum:boundsMaximum
            dimensions:dimensions
            hasDimensions:hasDimensions
            presentationMatchesDocument:
                measurement.presentationMatchesDocument
            modelCapabilities:static_cast<Core3DModelCapability>(
                measurement.modelCapabilities)];
    } catch (...) {
        return Core3DMakeStateSnapshot(
            Core3DTransformInspectorStateInvalid, 0,
            measurement.generation);
    }
}
