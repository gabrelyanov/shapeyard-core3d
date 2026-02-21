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
- (void)deleteSelected;
- (void)duplicateSelected;
- (void)setChamfer:(CGFloat)value;
- (Boundaries)getChamferBoundaries;
- (void)applyChamfer;
- (void)cancelChamfer;
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
