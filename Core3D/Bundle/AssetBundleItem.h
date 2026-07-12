//
//  AssetBundleItem.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 23.04.2024.
//

#import <Foundation/Foundation.h>

typedef NS_ENUM(NSUInteger, AssetBundleItemType) {
    AssetBundleItemTypeUnknown = 0,
    AssetBundleItemTypeAsset = 1,
    AssetBundleItemTypeThumb = 2
};

NS_ASSUME_NONNULL_BEGIN

@interface AssetBundleItem : NSObject

- (instancetype)init NS_UNAVAILABLE;
- (instancetype)initWithData:(NSData * _Nonnull)data
                        type:(AssetBundleItemType)type;
- (instancetype)initWithData:(NSData * _Nonnull)data
                        type:(AssetBundleItemType)type
                         url:(NSURL * _Nonnull)url;
- (instancetype)initWithURL:(NSURL * _Nonnull)url
                       type:(AssetBundleItemType)type;

@property (nonatomic, assign, readonly) AssetBundleItemType type;
//! File-backed reader items load their bytes only when this property is read.
@property (nonatomic, strong, readonly, nullable) NSData *data;
@property (nonatomic, strong, readonly, nullable) NSURL *url;
@property (nonatomic, strong, readonly, nullable) NSNumber *timestamp;
@property (nonatomic, assign, readonly) BOOL hasLoadedData;

@end

NS_ASSUME_NONNULL_END
