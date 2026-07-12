#import "Core3DNativeImportOperation.h"

NS_ASSUME_NONNULL_BEGIN

@interface Core3DNativeImportOperation (Private)

- (nullable instancetype)initWithSTEPURL:(NSURL *)stepURL
                            temporaryRoot:(NSURL *)temporaryRoot;

@end

NS_ASSUME_NONNULL_END
