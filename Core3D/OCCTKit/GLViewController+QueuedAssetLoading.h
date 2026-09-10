#import "GLViewController.h"
#import "Core3DQueuedAssetInput.h"
@class Core3DQueuedAssetLoadOwner;
@interface GLViewController (QueuedAssetLoading)
- (Core3DAssetLoadResult)acceptQueuedAssetInput:(Core3DQueuedAssetInput *)input
    owner:(Core3DQueuedAssetLoadOwner **)owner
    progress:(void (^)(BOOL cleanupPending))progress
    completion:(void (^)(Core3DAssetLoadResult))completion;
- (void)abandonQueuedAssetOwner:(Core3DQueuedAssetLoadOwner *)owner;
- (void)retryQueuedAssetCleanup;
#ifdef DEBUG
- (void)debugPauseQueuedAssetAdoption:(BOOL)paused;
- (void)debugResumeQueuedAssetAdoption;
- (void)debugFailQueuedAssetCleanup:(NSUInteger)count;
- (NSDictionary<NSString *, id> *)debugQueuedAssetLoadState;
#endif
@end

@interface Core3DViewController (QueuedAssetLoadAdmission)
- (BOOL)core3d_canReserveQueuedInput:(Core3DQueuedAssetInput *)input
                  fromGLController:(GLViewController *)controller;
@end
