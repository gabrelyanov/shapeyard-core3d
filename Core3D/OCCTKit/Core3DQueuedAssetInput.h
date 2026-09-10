// Private immutable load input, not edit authority.
#import <Foundation/Foundation.h>
NS_ASSUME_NONNULL_BEGIN

typedef NS_ENUM(NSUInteger, Core3DQueuedAssetInputKind) {
    Core3DQueuedAssetInputKindData,
    Core3DQueuedAssetInputKindVerifiedFile,
    Core3DQueuedAssetInputKindBundle,
};

@interface Core3DQueuedAssetInput : NSObject
@property(nonatomic, readonly) Core3DQueuedAssetInputKind kind;
@property(nonatomic, strong, readonly, nullable) NSData *data;
@property(nonatomic, strong, readonly, nullable) NSURL *fileURL;
@property(nonatomic, readonly) unsigned long long expectedByteCount;
@property(nonatomic, strong, readonly, nullable) NSString *expectedSHA256;
// Freeze inputs on the native owner/main thread, before cancellation or queue
// submission. No validation of native file contents, disk staging, reservation,
// tool cancellation or replacement is performed by these factories.
+ (nullable instancetype)freezeData:(NSData *)data;
+ (nullable instancetype)freezeVerifiedFile:(NSURL *)fileURL
                        expectedByteCount:(unsigned long long)byteCount
                           expectedSHA256:(NSString *)sha256;
+ (nullable instancetype)freezeBundleURL:(NSURL *)bundleURL;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end
NS_ASSUME_NONNULL_END
