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

@property (nonatomic, assign, readonly) AssetBundleItemType type;
@property (nonatomic, strong, readonly) NSData *data;
@property (nonatomic, strong, readonly, nullable) NSURL *url;
@property (nonatomic, strong, readonly, nullable) NSNumber *timestamp;

@end

NS_ASSUME_NONNULL_END
