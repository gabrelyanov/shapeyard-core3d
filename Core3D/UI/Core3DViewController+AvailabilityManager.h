//
//  Core3DViewController+AvailabilityManager.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 15.04.2024.
//

#import <Core3D/Core3DViewController.h>

NS_ASSUME_NONNULL_BEGIN

@interface Core3DViewController (AvailabilityManager)<AvailabilityManagerProtocol>

- (BOOL)canAdd;
- (BOOL)canDelete;
- (BOOL)canDuplicate;
- (BOOL)canUndo;
- (BOOL)canRedo;
- (NSArray<NSNumber *> *_Nonnull)availableGizmoTypes;
- (BOOL)canApplyMaterial;

@end

NS_ASSUME_NONNULL_END
