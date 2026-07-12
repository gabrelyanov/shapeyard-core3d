//
//  AssetBundleItem.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 23.04.2024.
//

#import "AssetBundleItem.h"

@interface AssetBundleItem ()
@property (nonatomic, strong, nullable) NSData *storedData;
@end

@implementation AssetBundleItem

- (instancetype)initWithData:(NSData * _Nonnull)data
                        type:(AssetBundleItemType)type {
    if (self = [super init]) {
        _storedData = data;
        _type = type;
        _url = NULL;
    }
    return self;
}

- (instancetype)initWithData:(NSData * _Nonnull)data
                        type:(AssetBundleItemType)type
                         url:(NSURL * _Nonnull)url {
    if (self = [super init]) {
        _storedData = data;
        _type = type;
        _url = url;
        _timestamp = [self timestampFromURL:url];
    }
    return self;
}

- (instancetype)initWithURL:(NSURL * _Nonnull)url
                       type:(AssetBundleItemType)type {
    if (self = [super init]) {
        _storedData = nil;
        _type = type;
        _url = url;
        _timestamp = [self timestampFromURL:url];
    }
    return self;
}

- (NSData *_Nullable)data {
    @synchronized (self) {
        if (_storedData == nil && _url != nil) {
            _storedData = [NSData dataWithContentsOfURL:_url
                                                options:kNilOptions
                                                  error:nil];
        }
        return _storedData;
    }
}

- (BOOL)hasLoadedData {
    @synchronized (self) {
        return _storedData != nil;
    }
}

- (NSNumber *_Nullable)timestampFromURL:(NSURL *)url {
    __auto_type filename = [[url lastPathComponent] stringByDeletingPathExtension];
    NSNumberFormatter *f = [[NSNumberFormatter alloc] init];
    f.numberStyle = NSNumberFormatterDecimalStyle;
    return [f numberFromString:filename];
}


@end
