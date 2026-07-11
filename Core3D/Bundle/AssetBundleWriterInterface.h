//
//  AssetBundleWriterInterface.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 23.04.2024.
//

#ifndef AssetBundleWriterInterface_h
#define AssetBundleWriterInterface_h

NS_ASSUME_NONNULL_BEGIN

@protocol AssetBundleWriterInterface <NSObject>

- (void)addItem:(AssetBundleItem *)item;
- (void)writeItems:(NSArray<AssetBundleItem *> *_Nonnull)items;

@end

NS_ASSUME_NONNULL_END

#endif /* AssetBundleWriterInterface_h */
