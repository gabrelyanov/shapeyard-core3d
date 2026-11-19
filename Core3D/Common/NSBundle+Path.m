//
//  NSBundle+Path.m
//  AvatarCore
//
//  Created by Max Reshetey on 21/01/2019.
//

#import "NSBundle+Path.h"

static const __auto_type kBundleIdentifier = @"org.shapeyard.Core3D";

/// Lives in the framework, so the framework can be found by class.
@interface C3DBundleToken : NSObject
@end

@implementation C3DBundleToken
@end

@implementation NSBundle (Path)

+ (instancetype)c3dBundle {
    // Device builds carry org.shapeyard.Core3D, simulator builds
    // io.magic-app.Core3D (project build settings). Looking up only the first
    // left every material preview blank in the simulator (Astra, toy car,
    // 15 Nov): fall back to the bundle that holds this framework's classes.
    NSBundle *bundle = [NSBundle bundleWithIdentifier:kBundleIdentifier];
    return bundle ?: [NSBundle bundleForClass:[C3DBundleToken class]];
}

@end
