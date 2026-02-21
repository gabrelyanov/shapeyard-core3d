//
//  ModelPart.h
//  Core3D
//
//  Created by Dmitry Sukhorukov on 24.09.2024.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

NS_ASSUME_NONNULL_BEGIN

@interface ModelPart : NSObject

@property (nonatomic, assign) NSUInteger identity;
@property (nonatomic, strong) UIImage* thumbImage;
@property (nonatomic, assign) BOOL aligned;

@end

NS_ASSUME_NONNULL_END
