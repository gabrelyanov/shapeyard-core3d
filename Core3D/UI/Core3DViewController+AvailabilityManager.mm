//
//  Core3DViewController+AvailabilityManager.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 15.04.2024.
//

#import <Foundation/Foundation.h>
#import "GLViewController.h"
#import <Core3D/Core3DViewController+AvailabilityManager.h>

#include "GLViewController+Trick.h"

@implementation Core3DViewController (AvailabilityManager)

- (BOOL)canAdd {
    return self.can_add;
}

- (BOOL)canDelete {
    return self.can_delete;
}

- (BOOL)canDuplicate {
    return self.can_duplicate;
}

- (BOOL)canUndo {
    const std::shared_ptr<core3d::Core3DViewer> viewer = GLController.viewer;
    return viewer != nullptr
        && !viewer->getDocument().IsNull()
        && viewer->getDocument()->canUndo();
}
- (BOOL)canRedo {
    const std::shared_ptr<core3d::Core3DViewer> viewer = GLController.viewer;
    return viewer != nullptr
        && !viewer->getDocument().IsNull()
        && viewer->getDocument()->canRedo();
}

- (BOOL)canApply {
    return self.can_apply;
}

- (BOOL) canApplyMaterial {
    return self.can_apply_material;
}
- (NSArray<NSNumber *> *)availableGizmoTypes {
    return _availableGizmoTypes;
}

@end
