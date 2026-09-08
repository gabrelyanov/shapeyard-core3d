#pragma once
// Bounded private reader view; never owns,
// reopens or changes the original source descriptor or its seek position.
#include "Core3DGLBPreflight.hpp"
#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <unistd.h>
#include <utility>
#include <vector>

namespace core3d::gltf {
class GLBReaderSource final {
public:
    static constexpr std::uint64_t MaximumBytes = 32ULL * 1024ULL * 1024ULL;
    static constexpr std::uint64_t MaximumJSONBytes = 4ULL * 1024ULL * 1024ULL;

    // Ordinary imports retain their original descriptor-backed representation.
    GLBReaderSource(int descriptor, std::uint64_t originalSize)
        : myDescriptor(descriptor),
          myBinary{0, originalSize}, mySize(originalSize),
          myValid(descriptor >= 0 && originalSize >= 20 && originalSize <= MaximumBytes) {}

    // Adapted layout: generated GLB header/JSON/BIN header, original pinned BIN
    // bytes, then tightly packed unique POSITION ranges. Both vectors are moved
    // only after the builder has admitted their allocation and final sizes.
    GLBReaderSource(int descriptor, std::uint64_t originalSize, ByteRange binary,
                    std::vector<std::uint8_t> prefix, std::vector<std::uint8_t> tail)
        : myDescriptor(descriptor), myBinary(binary),
          myPrefix(std::move(prefix)), myTail(std::move(tail)) {
        if (descriptor < 0 || originalSize < 20 || originalSize > MaximumBytes
            || binary.offset > originalSize || binary.length > originalSize - binary.offset
            || binary.length == 0 || binary.length % 4
            || myPrefix.size() < 32 || myPrefix.size() > MaximumJSONBytes + 28
            || myPrefix.size() % 4 || myTail.size() % 4
            || myTail.size() > MaximumBytes || myPrefix.size() > MaximumBytes - myTail.size()
            || binary.length > MaximumBytes - myPrefix.size() - myTail.size()) return;
        mySize = myPrefix.size() + binary.length + myTail.size();
        const auto jsonLength = Read32(myPrefix.data() + 12);
        if (Read32(myPrefix.data()) != 0x46546c67 || Read32(myPrefix.data() + 4) != 2
            || Read32(myPrefix.data() + 8) != mySize
            || Read32(myPrefix.data() + 16) != 0x4e4f534a
            || jsonLength == 0 || jsonLength % 4 || jsonLength > MaximumJSONBytes
            || myPrefix.size() != std::uint64_t(jsonLength) + 28
            || Read32(myPrefix.data() + myPrefix.size() - 8) != binary.length + myTail.size()
            || Read32(myPrefix.data() + myPrefix.size() - 4) != 0x004e4942) {
            mySize = 0; return;
        }
        myValid = true;
    }

    bool IsValid() const noexcept { return myValid; }
    bool IsAdapted() const noexcept { return !myPrefix.empty(); }
    std::uint64_t Size() const noexcept { return myValid ? mySize : 0; }
    std::uint64_t BinaryOffset() const noexcept { return myPrefix.size(); }
    std::uint64_t TailOffset() const noexcept { return myPrefix.size() + myBinary.length; }

    bool MapOriginalBinaryRange(ByteRange original, ByteRange& mapped) const noexcept {
        mapped = {};
        if (!myValid || original.offset < myBinary.offset) return false;
        const auto relative = original.offset - myBinary.offset;
        if (relative > myBinary.length || original.length > myBinary.length - relative) return false;
        mapped = {myPrefix.size() + relative, original.length};
        return true;
    }

    // Exact bounded reads with cancellation between at most 64 KiB chunks.
    // On failure callers discard their entire destination; no partial data is
    // admitted. Cancellation is distinct from an I/O failure.
    bool Read(std::uint64_t offset, void* destination, std::size_t count,
              const std::atomic_bool* cancelled, bool& ioFailure) const noexcept {
        const auto stopped = [&]() { return cancelled && cancelled->load(std::memory_order_acquire); };
        if (!myValid || offset > mySize || count > mySize - offset || (!destination && count)) {
            ioFailure = true; return false;
        }
        auto* bytes = static_cast<std::uint8_t*>(destination);
        std::size_t completed = 0;
        while (completed < count) {
            if (stopped()) return false;
            const auto position = offset + completed;
            std::size_t request = std::min<std::size_t>(count - completed, 64U * 1024U);
            if (position < myPrefix.size()) {
                request = std::min<std::uint64_t>(request, myPrefix.size() - position);
                std::memcpy(bytes + completed, myPrefix.data() + position, request);
            } else if (position - myPrefix.size() < myBinary.length) {
                const auto relative = position - myPrefix.size();
                request = std::min<std::uint64_t>(request, myBinary.length - relative);
                ssize_t actual;
                do {
                    actual = ::pread(myDescriptor, bytes + completed, request,
                                     static_cast<off_t>(myBinary.offset + relative));
                } while (actual < 0 && errno == EINTR && !stopped());
                if (actual <= 0) {
                    if (!stopped()) ioFailure = true;
                    return false;
                }
                request = static_cast<std::size_t>(actual);
            } else {
                const auto relative = position - TailOffset();
                request = std::min<std::uint64_t>(request, myTail.size() - relative);
                std::memcpy(bytes + completed, myTail.data() + relative, request);
            }
            completed += request;
        }
        return !stopped();
    }

private:
    static std::uint32_t Read32(const std::uint8_t* p) noexcept {
        return std::uint32_t(p[0]) | (std::uint32_t(p[1]) << 8)
            | (std::uint32_t(p[2]) << 16) | (std::uint32_t(p[3]) << 24);
    }
    int myDescriptor;
    ByteRange myBinary;
    std::vector<std::uint8_t> myPrefix, myTail;
    std::uint64_t mySize = 0;
    bool myValid = false;
};
} // namespace core3d::gltf
