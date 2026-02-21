//
//  PrimitiveSelectionType.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 12.04.2024.
//

#ifndef PrimitiveSelectionType_h
#define PrimitiveSelectionType_h

typedef NS_ENUM(NSUInteger, PrimitiveSelectionType) {
    PrimitiveSelectionTypeNone = 0,
    PrimitiveSelectionTypeShape = 1,
    PrimitiveSelectionTypeFace = 2,
    PrimitiveSelectionTypeEdge = 3,
    PrimitiveSelectionTypeVertex = 4
};

#endif /* PrimitiveSelectionType_h */
