#import <Foundation/Foundation.h>
#import <TargetConditionals.h>
#include "NativeModelingTombstone.hxx"

namespace core3d::tombstone::detail {
int OpenNativeRoot() noexcept {
    try {
    @autoreleasepool {
        @try {
            if ([NSThread isMainThread]) return -1;
            NSFileManager *manager = NSFileManager.defaultManager;
            NSURL *support = [manager URLsForDirectory:NSApplicationSupportDirectory
                inDomains:NSUserDomainMask].firstObject;
            if (!support || !support.isFileURL) return -1;
            // Native owns this fixed path; no public API accepts a journal URL.
            BOOL directory = NO;
            if (![manager fileExistsAtPath:support.path isDirectory:&directory]) {
                if (![manager createDirectoryAtURL:support withIntermediateDirectories:YES
                    attributes:@{NSFilePosixPermissions:@0700} error:nil]) return -1;
            } else if (!directory) return -1;
            FD supportParent(::open(support.URLByDeletingLastPathComponent.path.fileSystemRepresentation,
                O_RDONLY|O_DIRECTORY|O_CLOEXEC|O_NOFOLLOW));
            if (supportParent.value<0 || !DirectorySync(supportParent.value)) return -1;
            FD root(OpenRootInParent(support.path.fileSystemRepresentation));
            if (root.value<0) return -1;
            NSURL *rootURL = [support URLByAppendingPathComponent:@"NativeModelingRequests" isDirectory:YES];
            NSError *error = nil;
            if (![rootURL setResourceValue:@YES forKey:NSURLIsExcludedFromBackupKey error:&error]) return -1;
#if TARGET_OS_IPHONE
            if (![manager setAttributes:@{NSFileProtectionKey:NSFileProtectionCompleteUntilFirstUserAuthentication}
                ofItemAtPath:rootURL.path error:&error]) return -1;
#endif
            if (!Owned(root.value,true,0700) || !DirectorySync(root.value)) return -1;
            return std::exchange(root.value,-1);
        } @catch (...) { return -1; }
    }
    } catch (...) { return -1; }
}
} // namespace core3d::tombstone::detail
