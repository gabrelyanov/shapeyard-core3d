#ifndef Core3DMobileResourceLimits_h
#define Core3DMobileResourceLimits_h

#include <cstddef>

namespace core3d::limits {

//! Maximum number of leaf occurrences that one mobile project may publish.
//! Keep native import, the OpenGL fallback, and immutable Metal snapshots on
//! this single admission ceiling so no renderer can amplify a shared assembly
//! beyond the memory envelope accepted at ingestion.
constexpr std::size_t kMaximumLeafPresentations = 2'048;

//! Absolute value accepted for any authored/imported document-space
//! coordinate. Import validation and numeric transform authoring share this
//! ceiling so the editor cannot persist a value its own loader rejects.
constexpr double kMaximumModelCoordinateMagnitude = 1.0e6;

} // namespace core3d::limits

#endif // Core3DMobileResourceLimits_h
