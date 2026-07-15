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

//! Synchronous authority result for a selection-mode request.
//! Any value other than Succeeded leaves the caller's authoritative mode
//! unchanged. The legacy void setter remains available for source and ABI
//! compatibility.
typedef NS_ENUM(NSInteger, Core3DSelectionTypeChangeResult) {
    Core3DSelectionTypeChangeResultSucceeded = 0,
    Core3DSelectionTypeChangeResultUnsupported,
    Core3DSelectionTypeChangeResultNotReady,
    Core3DSelectionTypeChangeResultWrongThread,
    Core3DSelectionTypeChangeResultBusy,
    Core3DSelectionTypeChangeResultPresentationFailure,
};

#endif /* PrimitiveSelectionType_h */
