// Physical input capture is independent of native edit authority.
#import "Core3DQueuedAssetInput.h"

namespace {
constexpr unsigned long long kMaximumNativeProjectBytes = 256ull * 1024ull * 1024ull;
constexpr NSUInteger kMaximumNativeProjectPathBytes = 4096;

NSURL *FreezeLocalURL(NSURL *url) {
    if (url == nil || !url.isFileURL) return nil;
    // A path is not file identity. StageVerifiedAssetFile must still open/check
    // exact bytes and hash with its existing no-follow/regular-file contract.
    NSString *path = url.path;
    if (path.length == 0 || !path.isAbsolutePath
        || [path lengthOfBytesUsingEncoding:NSUTF8StringEncoding] > kMaximumNativeProjectPathBytes)
        return nil;
    NSData *bytes = [path dataUsingEncoding:NSUTF8StringEncoding allowLossyConversion:NO];
    if (bytes == nil || bytes.length == 0) return nil;
    const unsigned char *characters = static_cast<const unsigned char *>(bytes.bytes);
    for (NSUInteger i = 0; i < bytes.length; ++i) if (characters[i] == 0) return nil;
    // Preserve Foundation URL resource/security-scope metadata. Rebuilding a
    // URL from its path could discard the caller's existing access context.
    // NSURL is immutable; this does not begin or extend scoped access.
    return [url copy];
}
NSString *FreezeSHA256(NSString *input) {
    if (input.length != 64) return nil;
    NSData *bytes = [input dataUsingEncoding:NSASCIIStringEncoding allowLossyConversion:NO];
    if (bytes == nil || bytes.length != 64) return nil;
    const unsigned char *characters = static_cast<const unsigned char *>(bytes.bytes);
    for (NSUInteger i = 0; i < 64; ++i) {
        unsigned char c = characters[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return nil;
    }
    return [[NSString alloc] initWithBytes:bytes.bytes length:bytes.length encoding:NSASCIIStringEncoding];
}
}

@interface Core3DQueuedAssetInput ()
- (instancetype)initWithKind:(Core3DQueuedAssetInputKind)kind
                       data:(NSData * _Nullable)data
                    fileURL:(NSURL * _Nullable)fileURL
          expectedByteCount:(unsigned long long)byteCount
             expectedSHA256:(NSString * _Nullable)sha256;
@end

@implementation Core3DQueuedAssetInput
- (instancetype)initWithKind:(Core3DQueuedAssetInputKind)kind
                       data:(NSData *)data fileURL:(NSURL *)fileURL
          expectedByteCount:(unsigned long long)byteCount expectedSHA256:(NSString *)sha256 {
    self = [super init];
    if (self) {
        _kind = kind; _data = data; _fileURL = fileURL;
        _expectedByteCount = byteCount; _expectedSHA256 = sha256;
    }
    return self;
}
+ (instancetype)freezeData:(NSData *)data {
    if (![NSThread isMainThread] || data == nil || data.length == 0
        || data.length > kMaximumNativeProjectBytes) return nil;
    // Owned physical copy, including NSMutableData inputs. The caller must not
    // mutate the object concurrently with this synchronous admission boundary.
    NSData *frozen = [[NSData alloc] initWithBytes:data.bytes length:data.length];
    if (frozen == nil) return nil;
    return [[self alloc] initWithKind:Core3DQueuedAssetInputKindData data:frozen
        fileURL:nil expectedByteCount:frozen.length expectedSHA256:nil];
}
+ (instancetype)freezeVerifiedFile:(NSURL *)fileURL expectedByteCount:(unsigned long long)byteCount
                           expectedSHA256:(NSString *)sha256 {
    if (![NSThread isMainThread] || byteCount == 0 || byteCount > kMaximumNativeProjectBytes) return nil;
    NSURL *frozenURL = FreezeLocalURL(fileURL); NSString *frozenHash = FreezeSHA256(sha256);
    if (frozenURL == nil || frozenHash == nil) return nil;
    return [[self alloc] initWithKind:Core3DQueuedAssetInputKindVerifiedFile data:nil
        fileURL:frozenURL expectedByteCount:byteCount expectedSHA256:frozenHash];
}
+ (instancetype)freezeBundleURL:(NSURL *)bundleURL {
    if (![NSThread isMainThread]) return nil;
    NSURL *frozenURL = FreezeLocalURL(bundleURL);
    if (frozenURL == nil) return nil;
    return [[self alloc] initWithKind:Core3DQueuedAssetInputKindBundle data:nil
        fileURL:frozenURL expectedByteCount:0 expectedSHA256:nil];
}
@end
