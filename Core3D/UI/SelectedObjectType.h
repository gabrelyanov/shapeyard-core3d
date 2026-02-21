//
//  SelectedObjectType.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 15.05.2024.
//

#ifndef SelectedObjectType_h
#define SelectedObjectType_h

typedef NS_OPTIONS(NSUInteger, SelectedObjectType) {
    SelectedObjectTypeNone = 0,
    SelectedObjectTypePrimitive = 1 << 0,
    SelectedObjectTypeManipulator = 1 << 1
};

#endif /* SelectedObjectType_h */
