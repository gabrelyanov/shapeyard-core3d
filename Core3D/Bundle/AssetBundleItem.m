//
//  AssetBundleItem.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 23.04.2024.
//

#import "AssetBundleItem.h"

@implementation AssetBundleItem

- (instancetype)initWithData:(NSData * _Nonnull)data
                        type:(AssetBundleItemType)type {
    if (self = [super init]) {
        _data = data;
        _type = type;
        _url = NULL;
    }
    return self;
}

- (instancetype)initWithData:(NSData * _Nonnull)data
                        type:(AssetBundleItemType)type
                         url:(NSURL * _Nonnull)url {
    if (self = [super init]) {
        _data = data;
        _type = type;
        _url = url;
        _timestamp = [self timestampFromURL:url];
    }
    return self;
}

- (NSNumber *_Nullable)timestampFromURL:(NSURL *)url {
    __auto_type filename = [[url lastPathComponent] stringByDeletingPathExtension];
    NSNumberFormatter *f = [[NSNumberFormatter alloc] init];
    f.numberStyle = NSNumberFormatterDecimalStyle;
    return [f numberFromString:filename];
}


@end
