//
//  UIStateChanging.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 12.04.2024.
//

#ifndef UIStateChanging_h
#define UIStateChanging_h

typedef NS_OPTIONS(NSUInteger, UIStateChanging) {
    UIStateChangingNone = 0,
    UIStateChangingGizmo = 1 << 0,
    UIStateChangingSelection = 1 << 1,
    UIStateChangingAdd = 1 << 2,
    UIStateChangingDelete = 1 << 3,
    UIStateChangingDuplicate = 1 << 4,
    UIStateChangingHistory = 1 << 5,
    UIStateChangingApply = 1 << 6,
    UIStateChangingApplyMaterial = 1 << 7,
    UIStateChangingCoreInfoText = 1 << 8,
    UIStateChangingSnappingType = 1 << 9
};

#endif /* UIStateChanging_h */
