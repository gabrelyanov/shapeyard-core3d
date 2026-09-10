// Local request identity is not a wire token or native reservation.
#import "Core3DQueuedAssetRequest.h"
@interface Core3DQueuedAssetRequest ()
- (instancetype)initWithFrozenInput:(Core3DQueuedAssetInput *)input;
@end
@implementation Core3DQueuedAssetRequest
- (instancetype)initWithFrozenInput:(Core3DQueuedAssetInput *)input {
    self = [super init];
    if (self) _input = input;
    return self;
}
@end

@implementation Core3DQueuedAssetRequestSlot {
    Core3DQueuedAssetRequest *_current;
    BOOL _started;
}
- (Core3DQueuedAssetRequest *)acceptInput:(Core3DQueuedAssetInput *)input {
    if (![NSThread isMainThread] || input == nil || _current != nil) return nil;
    _current = [[Core3DQueuedAssetRequest alloc] initWithFrozenInput:input];
    _started = NO;
    return _current;
}
- (BOOL)ownsRequest:(Core3DQueuedAssetRequest *)request {
    // Strong capture of the exact object keeps pointer identity from being
    // reused while a delayed completion exists. Foreign slots cannot match.
    return [NSThread isMainThread] && request != nil && _current == request;
}
- (BOOL)claimStartForRequest:(Core3DQueuedAssetRequest *)request {
    if (![self ownsRequest:request] || _started) return NO;
    _started = YES;
    return YES;
}
- (BOOL)finishRequest:(Core3DQueuedAssetRequest *)request {
    if (![self ownsRequest:request]) return NO;
    _current = nil;
    _started = NO;
    return YES;
}
@end
