//
//  NSString+StdString.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 26.04.2024.
//

#import <Foundation/Foundation.h>
#include <string>

NS_ASSUME_NONNULL_BEGIN

@interface NSString(StdString)

@property(nonatomic, readonly) std::string stdString;

+ (std::string)stdStringForString:(NSString *)nsString;
+ (NSString *)stringForStdString:(const std::string&)stdString;

@end

NS_ASSUME_NONNULL_END
