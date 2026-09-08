#pragma once
#include <array>
#include <cstddef>
#include <cstdint>

namespace core3d::render {
// Content identity survives CPU-array release. It contains no OCAF handle and
// cannot keep a closed document or a mutable attribute alive.
struct NativeTangentArrayState {
    std::size_t bytes = 0;
    std::array<std::uint8_t, 32> basisIdentity = {};
};
} // namespace core3d::render
