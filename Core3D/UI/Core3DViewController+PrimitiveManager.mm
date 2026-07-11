//
//  Core3DViewController+PrimitiveManager.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 11.04.2024.
//

#import <Foundation/Foundation.h>
#import "GLViewController.h"
#import <Core3D/Core3DViewController+PrimitiveManager.h>

#include "GLViewController+Trick.h"
#include "Snapping.hpp"

@implementation Core3DViewController (PrimitiveManager)

- (void)addPrimitivesFromJSON:(NSString *)json {
    [self setGizmoType:PrimitiveGizmoTypeMoveRotate];
    [self setSelectionType:PrimitiveSelectionTypeShape];
    [GLController addPrimitivesFromJSON:json];
    self.can_undo = YES;
    self.can_delete = YES;
    self.can_duplicate = YES;
    self.can_apply_material = YES;
    _availableGizmoTypes = @[@(PrimitiveGizmoTypeMoveRotate),
                             @(PrimitiveGizmoTypeScale),
                             @(PrimitiveGizmoTypeChamfer),
                             @(PrimitiveGizmoTypeMirror),
                             @(PrimitiveGizmoTypeSubtract),
                             @(PrimitiveGizmoTypeUnion),
                             @(PrimitiveGizmoTypeMaterial)];
    [self sendNotifyUIState:UIStateChangingGizmo
                            | UIStateChangingSelection
                            | UIStateChangingDelete
                            | UIStateChangingDuplicate
                            | UIStateChangingHistory
                            | UIStateChangingApplyMaterial];
}

- (void)addPrimitive:(PrimitiveType)primitiveType {
    [self setGizmoType:PrimitiveGizmoTypeMoveRotate];
    [self setSelectionType:PrimitiveSelectionTypeShape];
    [GLController addPrimitive:primitiveType];
    self.can_undo = YES;
    self.can_delete = YES;
    self.can_duplicate = YES;
    self.can_apply_material = YES;
    _availableGizmoTypes = @[@(PrimitiveGizmoTypeMoveRotate),
                             @(PrimitiveGizmoTypeScale),
                             @(PrimitiveGizmoTypeChamfer),
                             @(PrimitiveGizmoTypeMirror),
                             @(PrimitiveGizmoTypeSubtract),
                             @(PrimitiveGizmoTypeUnion),
                             @(PrimitiveGizmoTypeMaterial)];
    //    [GLController selectLastObject];
    [self sendNotifyUIState:UIStateChangingGizmo
                            | UIStateChangingSelection
                            | UIStateChangingDelete
                            | UIStateChangingDuplicate
                            | UIStateChangingHistory
                            | UIStateChangingApplyMaterial];
}

- (void)deleteSelected {
    [GLController deleteSelected];
    self.can_undo = YES;
    self.can_delete = NO;
    self.can_duplicate = NO;
    self.can_apply_material = NO;
    _availableGizmoTypes = @[];
    [self setGizmoType:PrimitiveGizmoTypeMoveRotate];
    [self sendNotifyUIState:UIStateChangingGizmo
                             | UIStateChangingDelete
                             | UIStateChangingDuplicate
                             | UIStateChangingHistory
                             | UIStateChangingApplyMaterial];
}

- (void)selectAll {
	[GLController selectAll];
	[self sendNotifyUIState:UIStateChangingSelection];
}

- (void)duplicateSelected {
    [self setGizmoType:PrimitiveGizmoTypeMoveRotate];
    [GLController duplicateSelected];
    self.can_apply_material = YES;
    [self sendNotifyUIState:UIStateChangingApplyMaterial];
}

- (void)setChamfer:(CGFloat)value {
    if (_currentGizmoType != PrimitiveGizmoTypeChamfer) { return; }
    [GLController setChamfer:value*100];
}

- (Boundaries)getChamferBoundaries {
    return {.min = -15, .max = 15};
}

- (void)applyChamfer {
    [self completeOperationInteraction];
}

- (void)cancelChamfer {
    if (_currentGizmoType != PrimitiveGizmoTypeChamfer) {
        return;
    }
    [GLController cancelChamfer];
    [self completeOperationInteraction];
}

- (void)completeOperationInteraction {
    // remove selection
    _availableGizmoTypes = @[];
    [GLController deselectAll];
    [self setGizmoType:PrimitiveGizmoTypeNone];
    if (_currentSelectionType == PrimitiveSelectionTypeShape) {
        [self setGizmoType:PrimitiveGizmoTypeMoveRotate];
    }
    self.can_delete = NO;
    self.can_duplicate = NO;
    self.can_apply = NO;
    [self sendNotifyUIState:UIStateChangingGizmo
                             | UIStateChangingDuplicate
                             | UIStateChangingDelete
                             | UIStateChangingApply];
}

- (void)applyMirror {
    [GLController applyMirror];
    [self completeOperationInteraction];
}

- (void)cancelMirror {
    [GLController cancelMirror];
    [self completeOperationInteraction];
}

- (void)applySubtract {
	[GLController applySubtract];
    [self completeOperationInteraction];
}

- (void)cancelSubtract {
	[GLController cancelSubtract];
    [self completeOperationInteraction];
}

- (void)applyUnion {
	[GLController applyUnion];
    [self completeOperationInteraction];
}

- (void)cancelUnion {
    [GLController cancelUnion];
    [self completeOperationInteraction];
}

- (void)undo {
    [GLController undo];
}

- (void)redo {
    [GLController redo];
}

- (NSString *_Nullable)getCoreInfoText {
    return _coreInfoText;
}

- (void)setOrthoProjection:(OrthoProjectionType)orthoType {
    NSLog(@"SET ORTHO: %lu", static_cast<unsigned long>(orthoType));
    [GLController setOrthoProjection:orthoType];
}

- (void)setSnappingTranslation:(double)value {
    NSLog(@"SET SNAPPING TRANSLATION: %f", value);
    Snapping::Instance().setLinear(value);
    [self sendNotifyUIState:UIStateChangingSnappingType];
}

- (double)getSnappingTranslation {
    auto val = Snapping::Instance().getLinear();
    if(val.has_value()) {
        return *val;
    }
    return 0;
}

- (void)setSnappingRotation:(double)value {
    NSLog(@"SET SNAPPING ROTATION: %f", value);
    Snapping::Instance().setAngular(static_cast<double>(value));
    [self sendNotifyUIState:UIStateChangingSnappingType];
}

- (double)getSnappingRotation {
    auto val = Snapping::Instance().getAngular();
    if(val.has_value()) {
        return *val;
    }
    return 0;
}

- (void)setSnappingScale:(double)value {
    NSLog(@"SET SNAPPING SCALE: %f", static_cast<float>(value * 0.01));
    Snapping::Instance().setScaling(value * 0.01);
    [self sendNotifyUIState:UIStateChangingSnappingType];
}

- (double)getSnappingScale {
    auto val = Snapping::Instance().getScaling();
    if(val.has_value()) {
        return static_cast<double>(*val / 0.01);
    }
    return 0;
}

#ifdef DEBUG
- (void)handleDebug_A {
    NSLog(@"AAA");
}

- (void)handleDebug_B {
    NSLog(@"BBB");
}
#endif

@end
