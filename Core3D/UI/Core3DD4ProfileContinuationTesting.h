#pragma once

#include <stdint.h>

#ifdef __OBJC__
@class Core3DViewController, NSString;
#endif

#if DEBUG
#ifdef __cplusplus
extern "C" {
#endif
// DEBUG-only drivers/observers. They call production capture/prepare/stage
// and read actual native records. They never mint identities, recipes, shapes
// or a currentness result.
uint64_t Core3DDebugD4ProfileContinuationContractProbe(int32_t scenario,
                                                       double metersPerUnit);
void *Core3DDebugD4ProfileCreateFault(Core3DViewController *controller,
                                     NSString *host, NSString *source,
                                     int32_t fault);
void *Core3DDebugD4ProfileCreationStop(Core3DViewController *controller,
                                      NSString *host, NSString *source);
// NativePhysicalWorkingFrame seam observations. Scenarios 0...7 correspond
// one-for-one with AuthoredBooleanNativeTests' eight working-frame selectors.
void *Core3DDebugPhysicalWorkingFrameProbe(int32_t scenario);
void *Core3DDebugPhysicalWorkingFrameEmptyCentimetreDocumentSeed(void);
void Core3DDebugWorkingScaleClear(void);
void *Core3DDebugWorkingScaleTake(void);
#ifdef __cplusplus
}
#endif
#endif
