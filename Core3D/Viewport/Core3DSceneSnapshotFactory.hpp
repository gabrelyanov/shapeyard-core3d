//
//  Core3DSceneSnapshotFactory.hpp
//  Core3D
//
//  Internal Objective-C++ bridge. This header is intentionally not public.
//


#ifndef Core3DSceneSnapshotFactory_hpp
#define Core3DSceneSnapshotFactory_hpp

#if !defined(__OBJC__) || !defined(__cplusplus)
#error "Core3DSceneSnapshotFactory.hpp requires Objective-C++."
#endif

#import "Core3DSceneSnapshot.h"

namespace core3d {
namespace scene {
struct SceneSnapshot;
}
}

NS_ASSUME_NONNULL_BEGIN

//! Deep-copies a validated renderer-neutral C++ snapshot into the public
//! immutable DTO. Returns nil when a producer violates the public contract.
Core3DSceneSnapshot * _Nullable Core3DCreateSceneSnapshotDTO(
    const core3d::scene::SceneSnapshot& snapshot) noexcept;

NS_ASSUME_NONNULL_END

#endif // Core3DSceneSnapshotFactory_hpp
