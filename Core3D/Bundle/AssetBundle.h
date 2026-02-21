//
//  AssetBundle.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 23.04.2024.
//  Copyright © 2024 Magic Unicorn Inc. All rights reserved.
//

#import <Foundation/Foundation.h>

#import <Core3D/AssetBundleReaderInterface.h>
#import <Core3D/AssetBundleWriterInterface.h>

NS_ASSUME_NONNULL_BEGIN

@interface AssetBundle : NSObject<AssetBundleReaderInterface, AssetBundleWriterInterface>

+ (id<AssetBundleReaderInterface> _Nullable)makeReaderWithUrl:(NSURL *__nonnull)url;
+ (id<AssetBundleWriterInterface> _Nonnull)makeWriterWithUrl:(NSURL *__nonnull)url;

- (instancetype)init NS_UNAVAILABLE;


@end

NS_ASSUME_NONNULL_END
