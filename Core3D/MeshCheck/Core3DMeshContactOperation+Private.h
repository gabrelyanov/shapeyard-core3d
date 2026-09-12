#import "Core3DMeshContactOperation.h"
#include "../OCCTKit/NativeContactSource.hpp"
#include <memory>

NS_ASSUME_NONNULL_BEGIN
// Both blocks run only on main. Only immutable numeric capture values enter
// the worker; controller implementations must capture their owner weakly.
typedef core3d::meshcheck::ContactSourceStatus (^Core3DMeshContactPrepare)(
    const std::atomic_bool& cancelled,
    const std::shared_ptr<void>& reservation,
    std::shared_ptr<const core3d::meshcheck::ContactSourceCapture>& output);
typedef core3d::meshcheck::ContactSourceStatus (^Core3DMeshContactValidate)(
    const core3d::meshcheck::ContactSourceCapture& original,
    const std::atomic_bool& cancelled);

@interface Core3DMeshContactOperation (Private)
#if DEBUG
//! One-shot main-thread test seams around the real capture/worker delivery.
//! They do not provide geometry, bypass validation, or block worker threads.
- (void)core3d_setAfterCaptureHook:(void (^ _Nullable)(void))afterCapture
              beforeDeliveryHook:(void (^ _Nullable)(void))beforeDelivery;
#endif
- (instancetype)initWithIdentity:(const core3d::meshcheck::ContactSourceIdentity&)identity
                      ownerToken:(NSObject *)ownerToken
                         prepare:(Core3DMeshContactPrepare)prepare
                        validate:(Core3DMeshContactValidate)validate
                      completion:(Core3DMeshContactCompletion)completion;
@end

@interface Core3DMeshContactReport (Private)
// This token alone is insufficient. The controller must also recognize the
// exact issued report object and validate its own retained original source.
@property(nonatomic, readonly, strong) NSObject *ownerToken;
//! Installed by the issuing owner before public delivery. May run on any
//! thread when the immutable report is released; owner work must go to main.
- (void)core3d_setReleaseHandler:(void (^)(void))handler;
@end
// Copies only bounded immutable coordinates into renderer-neutral display DTOs.
@interface Core3DMeshContactInspection (Private)
- (nullable instancetype)initWithSource:(const core3d::meshcheck::ContactSourceCapture&)source
    firstTriangle:(NSUInteger)firstTriangle secondTriangle:(NSUInteger)secondTriangle;
@end
NS_ASSUME_NONNULL_END
