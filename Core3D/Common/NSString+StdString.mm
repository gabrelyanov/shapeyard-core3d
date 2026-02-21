//
//  NSString+StdString.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 26.04.2024.
//

#import "NSString+StdString.h"

@implementation NSString(StdString)

- (std::string)stdString {
    return [NSString stdStringForString:self];
}

+ (std::string)stdStringForString:(NSString *)nsString {
    NSData *charData = [nsString dataUsingEncoding:NSUTF8StringEncoding];
    return std::string(reinterpret_cast<const char *>(charData.bytes),
                       charData.length);
}

+ (NSString *)stringForStdString:(const std::string&)stdString {
    // std::string may contain null termination character so we construct
    // using length.
    return [[NSString alloc] initWithBytes:stdString.data()
                                    length:stdString.length()
                                  encoding:NSUTF8StringEncoding];
}

@end
