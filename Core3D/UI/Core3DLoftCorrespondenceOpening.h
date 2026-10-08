#import <Foundation/Foundation.h>
#include <stddef.h>
#include <stdint.h>

NS_ASSUME_NONNULL_BEGIN

// LC0 is deliberately detached: no production authoring opening, document
// write, or carrier registration is exposed by this portion.
__attribute__((objc_subclassing_restricted))
@interface Core3DLoftCorrespondenceOpening : NSObject
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

#if DEBUG
// Read-only, versioned native evidence for LoftCorrespondenceGeometryTests.
// Each scenario constructs or challenges actual detached OCCT geometry and
// returns granular measurements in the caller-owned bounded buffers.
FOUNDATION_EXPORT uint64_t Core3DDebugLoftCorrespondenceGeometryProbe(
    int32_t scenario, double metersPerUnit,
    double *_Nullable values, size_t valueCapacity,
    uint64_t *_Nullable words, size_t wordCapacity);
#endif

NS_ASSUME_NONNULL_END
