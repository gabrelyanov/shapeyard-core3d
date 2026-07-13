//
//  Core3DGLBReader.hpp
//  Core3D
//
//  Private, descriptor-pinned GLB transfer into an isolated XDE document.
//

#ifndef Core3DGLBReader_hpp
#define Core3DGLBReader_hpp

#include "Core3DGLBPreflight.hpp"

#include <TDocStd_Document.hxx>

#include <atomic>
#include <cstdint>
#include <string>

class Message_ProgressRange;

namespace core3d::gltf {

enum class GLBReadStatus {
    Imported,
    Cancelled,
    Invalid,
    Unsupported,
    ResourceLimit,
    IOFailure,
    InternalFailure,
};

struct GLBReadResult {
    GLBReadStatus status = GLBReadStatus::Invalid;
    std::string message;
    std::uint64_t flattenedObjectCount = 0;
    std::uint64_t vertexCount = 0;
    std::uint64_t indexCount = 0;

    [[nodiscard]] bool IsSuccess() const noexcept {
        return status == GLBReadStatus::Imported;
    }
};

//! Transfers a GLB admitted by `PreflightPinnedGLB()` through the already-open
//! source descriptor. No source pathname is accepted or reopened. `descriptor`
//! and `cancelled` must remain valid for the duration of the call.
//!
//! The caller owns descriptor identity and full-digest checks before and after
//! this function. This function independently requires the exact pinned size,
//! reads with pread(2), and exposes only one unguessable in-process token to
//! OCCT's late-data loader.
[[nodiscard]] GLBReadResult ImportPinnedGLB(
    int descriptor,
    std::uint64_t expectedSize,
    const PreflightResult& preflight,
    const Handle(TDocStd_Document)& document,
    const std::atomic_bool *cancelled,
    const Message_ProgressRange& progress) noexcept;

} // namespace core3d::gltf

#endif // Core3DGLBReader_hpp
