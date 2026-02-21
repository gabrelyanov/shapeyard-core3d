//
//  NSBundle+Path.m
//  AvatarCore
//
//  Created by Max Reshetey on 21/01/2019.
//

#import "NSBundle+Path.h"

static const __auto_type kBundleIdentifier = @"org.shapeyard.Core3D";

@implementation NSBundle (Path)

+ (instancetype)c3dBundle {

//#if FRAMEWORK
    return [NSBundle bundleWithIdentifier:kBundleIdentifier];
//#else
//    return [NSBundle mainBundle];
//#endif

}

@end
