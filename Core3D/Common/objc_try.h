//
//  objc_try.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 24.04.2024.
//

#ifndef objc_try_h
#define objc_try_h

#import <Foundation/Foundation.h>

NS_INLINE NSException * _Nullable objc_try(void(^_Nonnull tryBlock)(void)) {
    @try {
        tryBlock();
    }
    @catch (NSException *exception) {
        return exception;
    }
    return nil;
}

#endif /* objc_try_h */
