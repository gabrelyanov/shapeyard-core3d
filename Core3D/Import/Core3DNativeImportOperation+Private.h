#import "Core3DNativeImportOperation.h"

NS_ASSUME_NONNULL_BEGIN

@interface Core3DNativeImportOperation (Private)

- (nullable instancetype)initWithSourceURL:(NSURL *)sourceURL
                                    format:(Core3DNativeImportFormat)format
                             temporaryRoot:(NSURL *)temporaryRoot;

@end

NS_ASSUME_NONNULL_END
