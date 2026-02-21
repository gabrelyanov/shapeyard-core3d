//
//  AssetBundle.mm
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 23.04.2024.
//  Copyright © 2024 Magic Unicorn Inc. All rights reserved.
//

#import "AssetBundle.h"

@implementation AssetBundle {
    NSURL *_url;
    NSMutableArray<AssetBundleItem *> *_items;
}

+ (id<AssetBundleReaderInterface> _Nullable)makeReaderWithUrl:(NSURL *__nonnull)url {
    if (![NSFileManager.defaultManager fileExistsAtPath:url.path]) {
        return NULL;
    }
    __auto_type bundle = [[AssetBundle alloc] initWithUrl:url];
    @try {
        [bundle read];
    } @catch (NSException *exception) {
        NSLog(@"ERROR: %@", exception.reason);
        return NULL;
    }
    return bundle;
}

+ (id<AssetBundleWriterInterface> _Nonnull)makeWriterWithUrl:(NSURL *__nonnull)url {
    __auto_type bundle = [[AssetBundle alloc] initWithUrl:url];
    [bundle createBundleDirectoryIfNeeded];
    return bundle;
}

- (instancetype _Nullable)initWithUrl:(NSURL *__nonnull)url {
    if (self = [super init]) {
        _url = url;
        _items = [NSMutableArray<AssetBundleItem *> new];
    }
    return self;
}

- (NSArray<NSURL *> *_Nullable)contents {
    NSArray *contents = [NSFileManager.defaultManager contentsOfDirectoryAtPath:_url.path error:NULL];
    if (contents == NULL || contents.count == 0) {
        return NULL;
    }
    __auto_type result = [NSMutableArray<NSURL *> new];
    for (NSString *content in contents) {
        [result addObject:[_url URLByAppendingPathComponent:content]];
    }

    return result;
}

- (NSArray<NSURL *> *_Nullable)contentsWithExtension:(NSString *)extension {
    NSArray *contents = [NSFileManager.defaultManager contentsOfDirectoryAtPath:_url.path error:NULL];
    if (contents == NULL || contents.count == 0) {
        return NULL;
    }
    __auto_type result = [NSMutableArray<NSURL *> new];
    for (NSString *content in contents) {
        __auto_type contentUrl = [_url URLByAppendingPathComponent:content];
        if ([contentUrl.pathExtension isEqualToString:extension]) {
            [result addObject:contentUrl];
        }
    }
    return result;
}

- (void)read {
    __auto_type fm = NSFileManager.defaultManager;
    NSArray *contents = [fm contentsOfDirectoryAtPath:_url.path error:NULL];
    if (contents == NULL || contents.count == 0) {
        @throw [NSException exceptionWithName:@"ReadBundleException" 
                                       reason:[NSString stringWithFormat:
                                               @"The reading of the bundle occurred in error. %@", _url]
                                     userInfo:nil];
    }
    for (NSString *content in contents) {
        NSError *error = NULL;
        __auto_type contentUrl = [_url URLByAppendingPathComponent:content];
        __auto_type contentData = [NSData dataWithContentsOfURL:contentUrl options:kNilOptions error:&error];
        if (contentUrl == NULL || error) {
            @throw [NSException exceptionWithName:@"ReadBundleException"
                                           reason:[NSString stringWithFormat:
                                                   @"The reading of the bundle item [%@] occurred in error: %@", contentUrl, error.localizedDescription]
                                         userInfo:nil];
        }
        __auto_type type = [self getFileTypeByExtension:contentUrl.pathExtension];
        if (type == AssetBundleItemTypeUnknown) {
            NSLog(@"WARNING: Asset bundle type is unknown: %@", contentUrl);
            continue;
        }
        __auto_type item = [[AssetBundleItem alloc] initWithData:contentData
                                                            type:type
                                                             url:contentUrl];
        [self addItem: item];
    }
}

-(void)createBundleDirectoryIfNeeded {
    __auto_type fm = NSFileManager.defaultManager;
    if (![fm fileExistsAtPath:_url.path]) {
        NSError *error = NULL;
        if (![fm createDirectoryAtPath:_url.path
           withIntermediateDirectories:YES
                            attributes:NULL
                                 error:&error]) {
            
            @throw [NSException exceptionWithName:@"CreateBundleException"
                                           reason:[NSString stringWithFormat:
                                                   @"The creating of the bundle item [%@] occurred in error: %@", 
                                                   _url, error.localizedDescription]
                                         userInfo:nil];
        }
    }
}

- (void)writeItems:(NSArray<AssetBundleItem *> *_Nonnull)items {
    [_items addObjectsFromArray:items];
    __auto_type filename = [NSString stringWithFormat:@"%ld", (long)((NSInteger)([NSDate date].timeIntervalSince1970 * 1000))];
    for (AssetBundleItem *item in _items) {
        __auto_type fileExtension = [self getFileExtensionByType:item.type];
        [[self contentsWithExtension:fileExtension] enumerateObjectsUsingBlock:^(NSURL * _Nonnull obj, NSUInteger idx, BOOL * _Nonnull stop) {
            NSError *error = NULL;
            [NSFileManager.defaultManager removeItemAtURL:obj error:&error];
            if (error) {
                NSLog(@"ERROR: Removing %@ FAILED", obj);
            }
        }];
        NSError *error = NULL;
        NSURL *url = [[_url URLByAppendingPathComponent:filename] URLByAppendingPathExtension:fileExtension];
        [item.data writeToURL:url options:NSDataWritingAtomic error:&error];
        if (error) {
            @throw [NSException exceptionWithName:@"WriteBundleException"
                                           reason:[NSString stringWithFormat:
                                                   @"The writing of the bundle item [%@] occurred in error: %@", url, error.localizedDescription]
                                         userInfo:nil];
        }
    }
}

- (NSArray<AssetBundleItem *> * _Nullable)items { 
    return _items;
}

- (void)addItem:(AssetBundleItem *)item { 
    [_items addObject:item];
}

- (NSString *)getFileExtensionByType:(AssetBundleItemType)type {
    switch (type) {
        case AssetBundleItemTypeAsset:
            return @"asset";
        case AssetBundleItemTypeThumb:
            return @"thumb";
        default:
            NSAssert(false, @"Must be implemented!");
            break;
    }
    return NULL;
}

- (AssetBundleItemType)getFileTypeByExtension:(NSString *_Nonnull)extension {
    if ([extension isEqualToString:@"asset"]) {
        return AssetBundleItemTypeAsset;
    }

    if ([extension isEqualToString:@"thumb"]) {
        return AssetBundleItemTypeThumb;
    }

    return AssetBundleItemTypeUnknown;
}

@end
