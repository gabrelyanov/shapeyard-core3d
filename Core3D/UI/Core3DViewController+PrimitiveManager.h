//
//  Core3DViewController+PrimitiveManager.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 11.04.2024.
//

#import <Core3D/Core3DViewController.h>

NS_ASSUME_NONNULL_BEGIN

@interface Core3DViewController (PrimitiveManager)<PrimitiveManagerProtocol>

- (void)addPrimitive:(PrimitiveType)primitiveType;
- (void)addPrimitivesFromJSON:(NSString *)json;
- (void)deleteSelected;
- (void)duplicateSelected;
- (void)setChamfer:(CGFloat)value;
- (Boundaries)getChamferBoundaries;
- (Core3DModelingOperationResult)tryApplyChamfer
    NS_SWIFT_NAME(tryApplyChamfer());
- (Core3DModelingOperationResult)tryCancelChamfer
    NS_SWIFT_NAME(tryCancelChamfer());
- (void)applyChamfer;
- (BOOL)cancelChamfer;
- (void)setExtrusion:(CGFloat)value;
- (Boundaries)getExtrusionBoundaries;
- (Core3DModelingOperationResult)tryApplyExtrusion
    NS_SWIFT_NAME(tryApplyExtrusion());
- (Core3DModelingOperationResult)tryCancelExtrusion
    NS_SWIFT_NAME(tryCancelExtrusion());
- (BOOL)applyExtrusion;
- (BOOL)cancelExtrusion;
- (Core3DModelingOperationResult)tryApplySubtract
    NS_SWIFT_NAME(tryApplySubtract());
- (Core3DModelingOperationResult)tryCancelSubtract
    NS_SWIFT_NAME(tryCancelSubtract());
- (Core3DModelingOperationResult)tryApplyUnion
    NS_SWIFT_NAME(tryApplyUnion());
- (Core3DModelingOperationResult)tryCancelUnion
    NS_SWIFT_NAME(tryCancelUnion());
- (Core3DModelingOperationResult)tryApplyIntersect
    NS_SWIFT_NAME(tryApplyIntersect());
- (Core3DModelingOperationResult)tryCancelIntersect
    NS_SWIFT_NAME(tryCancelIntersect());
- (void)applyIntersect;
- (void)cancelIntersect;
- (Core3DModelingPreviewStatus)modelingPreviewStatusForGizmoType:
    (PrimitiveGizmoType)gizmoType
    NS_SWIFT_NAME(modelingPreviewStatus(for:));
- (void)undo;
- (void)redo;
- (void)completeOperationInteraction;
- (NSString *_Nullable)getCoreInfoText;
- (void)setOrthoProjection:(OrthoProjectionType)orthoType;
- (void)setSnappingTranslation:(double)value;
- (double)getSnappingTranslation;
- (void)setSnappingRotation:(double)value;
- (double)getSnappingRotation;
- (void)setSnappingScale:(double)value;
- (double)getSnappingScale;
#ifdef DEBUG
- (void)handleDebug_A;
- (void)handleDebug_B;
#endif

@end

NS_ASSUME_NONNULL_END
