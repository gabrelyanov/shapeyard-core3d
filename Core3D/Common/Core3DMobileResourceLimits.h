#ifndef Core3DMobileResourceLimits_h
#define Core3DMobileResourceLimits_h

#include <cstddef>

namespace core3d::limits {

//! Maximum number of leaf occurrences that one mobile project may publish.
//! Keep native import, the OpenGL fallback, and immutable Metal snapshots on
//! this single admission ceiling so no renderer can amplify a shared assembly
//! beyond the memory envelope accepted at ingestion.
constexpr std::size_t kMaximumLeafPresentations = 2'048;

} // namespace core3d::limits

#endif // Core3DMobileResourceLimits_h
