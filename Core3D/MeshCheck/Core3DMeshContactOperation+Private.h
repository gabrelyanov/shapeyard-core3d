#import "Core3DMeshContactOperation.h"
#include "../OCCTKit/NativeContactSource.hpp"
#include <memory>

NS_ASSUME_NONNULL_BEGIN
// Both blocks run only on main. Only immutable numeric capture values enter
// the worker; controller implementations must capture their owner weakly.
typedef core3d::meshcheck::ContactSourceStatus (^Core3DMeshContactPrepare)(
    const std::atomic_bool& cancelled,
    std::shared_ptr<const core3d::meshcheck::ContactSourceCapture>& output);
typedef core3d::meshcheck::ContactSourceStatus (^Core3DMeshContactValidate)(
    const core3d::meshcheck::ContactSourceCapture& original,
    const std::atomic_bool& cancelled);

@interface Core3DMeshContactOperation (Private)
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
@end
NS_ASSUME_NONNULL_END
