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
    [self sendNotifyUIState:UIStateChangingApplyMaterial
                            | UIStateChangingHistory];
}

- (void)setChamfer:(CGFloat)value {
    if (_currentGizmoType != PrimitiveGizmoTypeChamfer) { return; }
    [GLController setChamfer:value*100];
}

- (Boundaries)getChamferBoundaries {
    return {.min = -15, .max = 15};
}

- (void)setExtrusion:(CGFloat)value {
    if (_currentGizmoType != PrimitiveGizmoTypeExtrude) {
        self.can_apply = NO;
        [self sendNotifyUIState:UIStateChangingApply];
        return;
    }
    (void)[GLController setExtrusion:value * 100.0];
    self.can_apply = [GLController canApplyExtrusion];
    [self viewDidChangeViewportPresentationState];
    [self sendNotifyUIState:UIStateChangingApply];
}

- (Boundaries)getExtrusionBoundaries {
    return {.min = -1.0, .max = 1.0};
}

- (BOOL)applyExtrusion {
    const BOOL applied = [GLController applyExtrusion];
    if (applied) {
        [self completeOperationInteraction];
    } else {
        self.can_apply = [GLController canApplyExtrusion];
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply];
    }
    return applied;
}

- (BOOL)cancelExtrusion {
    const BOOL cancelled = [GLController cancelExtrusion];
    if (cancelled) {
        [self completeOperationInteraction];
    } else {
        self.can_apply = [GLController canApplyExtrusion];
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply];
    }
    return cancelled;
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
                             | UIStateChangingApply
                             | UIStateChangingHistory];
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
	if ([GLController applySubtract]) {
        [self completeOperationInteraction];
    } else {
        if (![GLController hasActiveBoolean]) {
            [self completeOperationInteraction];
            return;
        }
        self.can_apply = [GLController canApplyBoolean];
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply];
    }
}

- (void)cancelSubtract {
	if ([GLController cancelSubtract]) {
        [self completeOperationInteraction];
    } else {
        self.can_apply = NO;
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply];
    }
}

- (void)applyUnion {
	if ([GLController applyUnion]) {
        [self completeOperationInteraction];
    } else {
        if (![GLController hasActiveBoolean]) {
            [self completeOperationInteraction];
            return;
        }
        self.can_apply = [GLController canApplyBoolean];
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply];
    }
}

- (void)cancelUnion {
    if ([GLController cancelUnion]) {
        [self completeOperationInteraction];
    } else {
        self.can_apply = NO;
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply];
    }
}

- (void)undo {
    const BOOL wasExtrusion =
        _currentGizmoType == PrimitiveGizmoTypeExtrude;
    [GLController undo];
    if (wasExtrusion) {
        _currentGizmoType = [GLController getGizmoType];
        if (_currentGizmoType != PrimitiveGizmoTypeExtrude) {
            [self completeOperationInteraction];
        } else {
            self.can_apply = [GLController canApplyExtrusion];
            [self viewDidChangeViewportPresentationState];
        }
        [self sendNotifyUIState:UIStateChangingGizmo
                                | UIStateChangingApply
                                | UIStateChangingHistory];
        return;
    }
    if (_currentGizmoType == PrimitiveGizmoTypeMirror
        || _currentGizmoType == PrimitiveGizmoTypeSubtract
        || _currentGizmoType == PrimitiveGizmoTypeUnion
        || _currentGizmoType == PrimitiveGizmoTypeExtrude) {
        self.can_apply = _currentGizmoType == PrimitiveGizmoTypeMirror
            ? [GLController hasTrialMirrorObjects]
            : _currentGizmoType == PrimitiveGizmoTypeExtrude
                ? [GLController canApplyExtrusion]
            : [GLController canApplyBoolean];
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply];
    }
}

- (void)redo {
    const BOOL wasExtrusion =
        _currentGizmoType == PrimitiveGizmoTypeExtrude;
    [GLController redo];
    if (wasExtrusion) {
        _currentGizmoType = [GLController getGizmoType];
        if (_currentGizmoType != PrimitiveGizmoTypeExtrude) {
            [self completeOperationInteraction];
        } else {
            self.can_apply = [GLController canApplyExtrusion];
            [self viewDidChangeViewportPresentationState];
        }
        [self sendNotifyUIState:UIStateChangingGizmo
                                | UIStateChangingApply
                                | UIStateChangingHistory];
        return;
    }
    if (_currentGizmoType == PrimitiveGizmoTypeMirror
        || _currentGizmoType == PrimitiveGizmoTypeSubtract
        || _currentGizmoType == PrimitiveGizmoTypeUnion
        || _currentGizmoType == PrimitiveGizmoTypeExtrude) {
        self.can_apply = _currentGizmoType == PrimitiveGizmoTypeMirror
            ? [GLController hasTrialMirrorObjects]
            : _currentGizmoType == PrimitiveGizmoTypeExtrude
                ? [GLController canApplyExtrusion]
            : [GLController canApplyBoolean];
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply];
    }
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
