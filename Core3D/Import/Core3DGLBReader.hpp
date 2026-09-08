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

#if DEBUG
// Bounded read-only trace for bundled parser ownership qualification.
struct GLBDebugStream {
    int type = 0, accessorID = -1;
    std::int64_t streamOffset = 0, streamLength = 0, accessorOffset = 0, count = 0;
    std::int32_t stride = 0;
    bool pinned = false;
};
struct GLBDebugPrimitive {
    std::string meshID;
    std::vector<GLBDebugStream> streams;
};
struct GLBDebugOccurrence {
    std::uint64_t primitive = 0;
    std::string label;
    std::vector<double> positions, normals, uvs;
    std::vector<int> indices;
};
struct GLBDebugTrace {
    std::vector<GLBDebugPrimitive> primitives;
    std::vector<GLBDebugOccurrence> occurrences;
};
#endif

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
    const Message_ProgressRange& progress
#if DEBUG
    , GLBDebugTrace* debugTrace = nullptr
#endif
    ) noexcept;

} // namespace core3d::gltf

#endif // Core3DGLBReader_hpp
