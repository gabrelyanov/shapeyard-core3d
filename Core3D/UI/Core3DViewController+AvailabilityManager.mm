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
    return self.can_undo;
}
- (BOOL)canRedo {
    return self.can_redo;
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
