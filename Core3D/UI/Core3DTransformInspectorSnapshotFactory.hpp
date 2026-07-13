//
//  Core3DTransformInspectorSnapshotFactory.hpp
//  Core3D
//
//  Internal Objective-C++ bridge. This header is intentionally not public.
//

#ifndef Core3DTransformInspectorSnapshotFactory_hpp
#define Core3DTransformInspectorSnapshotFactory_hpp

#if !defined(__OBJC__) || !defined(__cplusplus)
#error "Core3DTransformInspectorSnapshotFactory.hpp requires Objective-C++."
#endif

#import "Core3DTransformInspectorSnapshot.h"

namespace core3d {
struct TransformInspectorMeasurement;
}

NS_ASSUME_NONNULL_BEGIN

//! Convert one authoritative internal value into an immutable public DTO.
//! Invalid native values fail closed as a public Invalid snapshot.
Core3DTransformInspectorSnapshot *Core3DCreateTransformInspectorSnapshotDTO(
    const core3d::TransformInspectorMeasurement& measurement) noexcept;

//! Create a terminal public lifecycle state that has no selected-model data.
Core3DTransformInspectorSnapshot *Core3DCreateTransformInspectorStateSnapshotDTO(
    Core3DTransformInspectorState state) noexcept;

NS_ASSUME_NONNULL_END

#endif // Core3DTransformInspectorSnapshotFactory_hpp
