//
//  Core3DGLBPreflight.hpp
//  Core3D
//
//  Bounded, descriptor-based admission gate for the GLB 2.0 subset used by
//  native mobile import. This header is C++-only and intentionally private.
//


#ifndef Core3DGLBPreflight_hpp
#define Core3DGLBPreflight_hpp

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace core3d::gltf {

enum class PreflightStatus {
    Valid,
    Cancelled,
    Invalid,
    Unsupported,
    ResourceLimit,
    IOFailure,
};

struct ByteRange {
    std::uint64_t offset = 0;
    std::uint64_t length = 0;
};

struct EmbeddedImage {
    std::uint64_t imageIndex = 0;
    ByteRange bytes;
    std::string mimeType;
};

// Exact source records. Offsets identify bytes in the original pinned GLB;
// definition indices are used only to build a private parser view, never to
// infer native face ownership from parser traversal order.
struct GLBAccessorSource {
    std::uint64_t definition = 0;
    ByteRange view;
    std::uint64_t offset = 0, count = 0, stride = 0;
    std::uint64_t elementBytes = 0, componentType = 0;
    [[nodiscard]] bool IsPresent() const noexcept { return count != 0; }
};
struct GLBPrimitiveSource {
    std::uint64_t mesh = 0, primitive = 0;
    GLBAccessorSource position, normal, uv, indices, tangent;
};

struct PreflightResult {
    PreflightStatus status = PreflightStatus::Invalid;
    std::string message;

    ByteRange jsonChunk;
    bool hasBinaryChunk = false;
    ByteRange binaryChunk;

    // Estimates describe the fully flattened node occurrences admitted to the
    // document, rather than only the unique mesh definitions.
    std::uint64_t objectOccurrenceEstimate = 0;
    std::uint64_t vertexEstimate = 0;
    std::uint64_t indexEstimate = 0;
    std::vector<EmbeddedImage> embeddedImages;
    std::vector<GLBPrimitiveSource> primitiveSources;
    std::uint64_t suppliedFrameResidentEstimate = 0;

    [[nodiscard]] bool IsValid() const noexcept {
        return status == PreflightStatus::Valid;
    }
};

//! Validates a GLB through an already-open descriptor. The caller must keep
//! `descriptor` open for the duration of this call and supply the size observed
//! when that regular file was pinned. The implementation never changes the
//! descriptor offset and obtains all source bytes with pread(2).
//!
//! `cancelled` may be null. When non-null it must outlive this call.
[[nodiscard]] PreflightResult PreflightPinnedGLB(
    int descriptor,
    std::uint64_t expectedSize,
    const std::atomic_bool *cancelled) noexcept;

} // namespace core3d::gltf

#endif // Core3DGLBPreflight_hpp
