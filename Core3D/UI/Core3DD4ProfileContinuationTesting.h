#pragma once

#include <stdint.h>

#if DEBUG
#ifdef __cplusplus
extern "C" {
#endif
// DEBUG-only drivers/observers. They call production capture/prepare/stage
// and read actual native records. They never mint identities, recipes, shapes
// or a currentness result.
uint64_t Core3DDebugD4ProfileContinuationContractProbe(int32_t scenario,
                                                       double metersPerUnit);
#ifdef __cplusplus
}
#endif
#endif
