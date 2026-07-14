//
//  PrimitiveGizmoType.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 12.04.2024.
//

#ifndef PrimitiveGizmoType_h
#define PrimitiveGizmoType_h

typedef NS_ENUM(NSUInteger, PrimitiveGizmoType) {
    PrimitiveGizmoTypeNone = 0,
    PrimitiveGizmoTypeMoveRotate = 1,
    PrimitiveGizmoTypeScale = 2,
    PrimitiveGizmoTypeChamfer = 3,
    PrimitiveGizmoTypeSubtract = 4,
    PrimitiveGizmoTypeUnion = 5,
    PrimitiveGizmoTypeMirror = 6,
    PrimitiveGizmoTypeMaterial = 7,
    PrimitiveGizmoTypeExtrude = 8,
    PrimitiveGizmoTypeIntersect = 9,
    PrimitiveGizmoTypeLinearArray = 10,
};

#endif /* PrimitiveGizmoType_h */
