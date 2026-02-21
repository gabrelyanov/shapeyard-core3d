//
//  AssetBundleReaderInterface.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 23.04.2024.
//

#ifndef AssetBundleReaderInterface_h
#define AssetBundleReaderInterface_h

#import <Core3D/AssetBundleItem.h>

@protocol AssetBundleReaderInterface <NSObject>

- (NSArray<AssetBundleItem *> *_Nullable)items;

@end


#endif /* AssetBundleReaderInterface_h */
