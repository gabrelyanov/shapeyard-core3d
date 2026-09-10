// Exact local callback identity, not native edit authority.
#import <Foundation/Foundation.h>
#import "Core3DQueuedAssetInput.h"
NS_ASSUME_NONNULL_BEGIN

@interface Core3DQueuedAssetRequest : NSObject
@property(nonatomic, strong, readonly) Core3DQueuedAssetInput *input;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

// Main-thread-only private controller slot. Retain one slot for controller lifetime.
// Never serialize/request identity from model-provided strings or entity names.
@interface Core3DQueuedAssetRequestSlot : NSObject
// Rejects a second load while either deferred or preparing work is current.
- (nullable Core3DQueuedAssetRequest *)acceptInput:(Core3DQueuedAssetInput *)input;
// Called with the same retained request when setup finishes. Exactly once.
// Success does not authorize cancellation, staging or replacement: acquire the
// native reservation before those side effects and keep it through cleanup.
- (BOOL)claimStartForRequest:(Core3DQueuedAssetRequest *)request;
- (BOOL)ownsRequest:(Core3DQueuedAssetRequest *)request;
// Only after this request's private work/cleanup and appropriate native owner
// settlement. A failed/stale callback must not clear loading UI or tool state.
// A retained replacement-recovery owner remains independent of this slot.
- (BOOL)finishRequest:(Core3DQueuedAssetRequest *)request;
@end
NS_ASSUME_NONNULL_END
