#pragma once
// How many parts one Cut, Join or Overlap takes in all (CTO 23 Nov 03:55). The one place this
// number lives (Fable engine review 21 item 4); the app reads it through
// Core3DViewController.maximumBooleanParts for its sentence.
#include <cstddef>

namespace core3d {
inline constexpr std::size_t kMaximumBooleanParts = 32;
} // namespace core3d
