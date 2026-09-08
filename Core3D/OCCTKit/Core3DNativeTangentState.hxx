#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace core3d::render {
// Content identity survives CPU-array release. It contains no OCAF handle and
// cannot keep a closed document or a mutable attribute alive.
// Bounded read-only failure evidence; never used to admit geometry.
struct NativeTangentPreparationTrace {
    int stage = 0;
    int sourceElements = 0;
    int sourceAttributes = 0;
    int sourceCPUData = 0;
    int mismatchCorner = 0; // One-based; zero means no corner mismatch.
    int mismatchComponent = 0;
    std::uint32_t expectedBits = 0;
    std::uint32_t actualBits = 0;
};
struct NativeTangentArrayState {
    std::size_t bytes = 0;
    std::array<std::uint8_t, 32> basisIdentity = {};
};
} // namespace core3d::render
