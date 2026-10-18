//
//  Core3DViewController+ExportManager.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 24.04.2024.
//

#import <Core3D/Core3DViewController.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DSceneSnapshot;

@interface Core3DViewController (ExportManager)<ExportManagerProtocol>

- (BOOL)requestNativeStorageVersion:(NSInteger)version
                           expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(requestNativeStorageVersion(_:expected:));

- (nullable NSNumber *)nativeStorageVersionForExpectedScene:
    (Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(nativeStorageVersion(expected:));

+ (nullable NSNumber *)nativeStorageVersionOfAssetData:(NSData *)data
    NS_SWIFT_NAME(nativeStorageVersion(of:));

@end

NS_ASSUME_NONNULL_END
