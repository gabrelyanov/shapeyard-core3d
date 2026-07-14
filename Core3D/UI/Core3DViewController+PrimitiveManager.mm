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

namespace {

Core3DModelingPreviewStatus Core3DUnavailableModelingStatus(
    const PrimitiveGizmoType theOperation) noexcept {
    return {
        theOperation,
        Core3DModelingPreviewStateUnavailable,
        0,
        NO,
        NO,
    };
}

Core3DModelingPreviewState Core3DPreviewState(
    const core3d::BevelPreviewState theState) noexcept {
    switch (theState) {
        case core3d::BevelPreviewState::Selecting:
            return Core3DModelingPreviewStateSelecting;
        case core3d::BevelPreviewState::Computing:
        case core3d::BevelPreviewState::Committing:
            return Core3DModelingPreviewStateComputing;
        case core3d::BevelPreviewState::Ready:
            return Core3DModelingPreviewStateReady;
        case core3d::BevelPreviewState::Failed:
            return Core3DModelingPreviewStateFailed;
    }
    return Core3DModelingPreviewStateFailed;
}

Core3DModelingPreviewState Core3DPreviewState(
    const core3d::BooleanPreviewState theState) noexcept {
    switch (theState) {
        case core3d::BooleanPreviewState::Selecting:
            return Core3DModelingPreviewStateSelecting;
        case core3d::BooleanPreviewState::Computing:
        case core3d::BooleanPreviewState::Committing:
            return Core3DModelingPreviewStateComputing;
        case core3d::BooleanPreviewState::Ready:
            return Core3DModelingPreviewStateReady;
        case core3d::BooleanPreviewState::Failed:
            return Core3DModelingPreviewStateFailed;
    }
    return Core3DModelingPreviewStateFailed;
}

Core3DModelingPreviewState Core3DPreviewState(
    const core3d::ExtrusionPreviewState theState) noexcept {
    switch (theState) {
        case core3d::ExtrusionPreviewState::Unavailable:
            return Core3DModelingPreviewStateUnavailable;
        case core3d::ExtrusionPreviewState::Selecting:
            return Core3DModelingPreviewStateSelecting;
        case core3d::ExtrusionPreviewState::Ready:
            return Core3DModelingPreviewStateReady;
        case core3d::ExtrusionPreviewState::OutcomeUnknown:
            return Core3DModelingPreviewStateOutcomeUnknown;
        case core3d::ExtrusionPreviewState::Failed:
            return Core3DModelingPreviewStateFailed;
    }
    return Core3DModelingPreviewStateFailed;
}

Core3DModelingPreviewState Core3DPreviewState(
    const core3d::MirrorPreviewState theState) noexcept {
    switch (theState) {
        case core3d::MirrorPreviewState::Unavailable:
            return Core3DModelingPreviewStateUnavailable;
        case core3d::MirrorPreviewState::Selecting:
            return Core3DModelingPreviewStateSelecting;
        case core3d::MirrorPreviewState::Ready:
            return Core3DModelingPreviewStateReady;
        case core3d::MirrorPreviewState::Committing:
            return Core3DModelingPreviewStateComputing;
        case core3d::MirrorPreviewState::OutcomeUnknown:
            return Core3DModelingPreviewStateOutcomeUnknown;
        case core3d::MirrorPreviewState::Failed:
            return Core3DModelingPreviewStateFailed;
    }
    return Core3DModelingPreviewStateFailed;
}

Core3DModelingPreviewState Core3DPreviewState(
    const core3d::LinearArrayPreviewState theState) noexcept {
    switch (theState) {
        case core3d::LinearArrayPreviewState::Unavailable:
            return Core3DModelingPreviewStateUnavailable;
        case core3d::LinearArrayPreviewState::Selecting:
            return Core3DModelingPreviewStateSelecting;
        case core3d::LinearArrayPreviewState::Ready:
            return Core3DModelingPreviewStateReady;
        case core3d::LinearArrayPreviewState::Committing:
            return Core3DModelingPreviewStateComputing;
        case core3d::LinearArrayPreviewState::OutcomeUnknown:
            return Core3DModelingPreviewStateOutcomeUnknown;
        case core3d::LinearArrayPreviewState::Failed:
            return Core3DModelingPreviewStateFailed;
    }
    return Core3DModelingPreviewStateFailed;
}

Core3DModelingPreviewState Core3DPreviewState(
    const core3d::ShellPreviewState theState) noexcept {
    switch (theState) {
        case core3d::ShellPreviewState::Unavailable:
            return Core3DModelingPreviewStateUnavailable;
        case core3d::ShellPreviewState::Selecting:
            return Core3DModelingPreviewStateSelecting;
        case core3d::ShellPreviewState::Computing:
        case core3d::ShellPreviewState::Committing:
            return Core3DModelingPreviewStateComputing;
        case core3d::ShellPreviewState::Ready:
            return Core3DModelingPreviewStateReady;
        case core3d::ShellPreviewState::OutcomeUnknown:
            return Core3DModelingPreviewStateOutcomeUnknown;
        case core3d::ShellPreviewState::Failed:
            return Core3DModelingPreviewStateFailed;
    }
    return Core3DModelingPreviewStateFailed;
}

Core3DModelingPreviewStatus Core3DCurrentModelingStatus(
    GLViewController *theController,
    const PrimitiveGizmoType theOperation) {
    Core3DModelingPreviewStatus aStatus =
        Core3DUnavailableModelingStatus(theOperation);
    if (theController == nil || theController.viewer == nullptr) {
        return aStatus;
    }

    switch (theOperation) {
        case PrimitiveGizmoTypeChamfer: {
            const std::shared_ptr<core3d::ShapeInteractor> anInteractor =
                theController.viewer->getShapeInteractor();
            if (anInteractor == nullptr) {
                return aStatus;
            }
            aStatus.generation = anInteractor->bevelPreviewGeneration();
            aStatus.active = anInteractor->hasActiveBevel();
            if (!aStatus.active) {
                return aStatus;
            }
            aStatus.state = Core3DPreviewState(
                anInteractor->bevelPreviewState());
            aStatus.canApply =
                aStatus.state == Core3DModelingPreviewStateReady
                && anInteractor->canApplyBevel();
            return aStatus;
        }
        case PrimitiveGizmoTypeExtrude: {
            const std::shared_ptr<core3d::ShapeInteractor> anInteractor =
                theController.viewer->getShapeInteractor();
            if (anInteractor == nullptr) {
                return aStatus;
            }
            aStatus.active = anInteractor->hasActiveExtrusion();
            if (!aStatus.active) {
                return aStatus;
            }
            // Extrusion is synchronous, so generation zero is a complete
            // contract: no worker result can arrive after this observation.
            aStatus.state = Core3DPreviewState(
                anInteractor->extrusionPreviewState());
            aStatus.canApply =
                (aStatus.state == Core3DModelingPreviewStateReady
                    && anInteractor->canApplyExtrusion())
                || (aStatus.state
                        == Core3DModelingPreviewStateOutcomeUnknown
                    && anInteractor->canRetryExtrusionResolution());
            return aStatus;
        }
        case PrimitiveGizmoTypeMirror: {
            const std::shared_ptr<core3d::ObjectInteractor> anInteractor =
                theController.viewer->getObjectInteractor();
            if (anInteractor == nullptr) {
                return aStatus;
            }
            aStatus.generation = anInteractor->mirrorPreviewGeneration();
            aStatus.active = anInteractor->hasActiveMirror();
            if (!aStatus.active) {
                return aStatus;
            }
            aStatus.state = Core3DPreviewState(
                anInteractor->mirrorPreviewState());
            aStatus.canApply = anInteractor->canApplyMirror()
                && (aStatus.state == Core3DModelingPreviewStateReady
                    || aStatus.state
                        == Core3DModelingPreviewStateOutcomeUnknown);
            return aStatus;
        }
        case PrimitiveGizmoTypeLinearArray: {
            const std::shared_ptr<core3d::ObjectInteractor> anInteractor =
                theController.viewer->getObjectInteractor();
            if (anInteractor == nullptr) {
                return aStatus;
            }
            aStatus.generation =
                anInteractor->linearArrayPreviewGeneration();
            aStatus.active = anInteractor->hasActiveLinearArray();
            if (!aStatus.active) {
                return aStatus;
            }
            aStatus.state = Core3DPreviewState(
                anInteractor->linearArrayPreviewState());
            aStatus.canApply = anInteractor->canApplyLinearArray()
                && (aStatus.state == Core3DModelingPreviewStateReady
                    || aStatus.state
                        == Core3DModelingPreviewStateOutcomeUnknown);
            return aStatus;
        }
        case PrimitiveGizmoTypeShell: {
            const std::shared_ptr<core3d::ShapeInteractor> anInteractor =
                theController.viewer->getShapeInteractor();
            if (anInteractor == nullptr) {
                return aStatus;
            }
            aStatus.generation = anInteractor->shellPreviewGeneration();
            aStatus.active = anInteractor->hasActiveShell();
            if (!aStatus.active) {
                return aStatus;
            }
            aStatus.state = Core3DPreviewState(
                anInteractor->shellPreviewState());
            aStatus.canApply = anInteractor->canApplyShell()
                && (aStatus.state == Core3DModelingPreviewStateReady
                    || aStatus.state
                        == Core3DModelingPreviewStateOutcomeUnknown);
            return aStatus;
        }
        case PrimitiveGizmoTypeSubtract:
        case PrimitiveGizmoTypeUnion:
        case PrimitiveGizmoTypeIntersect: {
            const std::shared_ptr<core3d::ObjectInteractor> anInteractor =
                theController.viewer->getObjectInteractor();
            if (anInteractor == nullptr) {
                return aStatus;
            }
            core3d::BooleanAction anAction =
                core3d::BooleanAction::BooleanSubtract;
            if (theOperation == PrimitiveGizmoTypeUnion) {
                anAction = core3d::BooleanAction::BooleanUnion;
            } else if (theOperation == PrimitiveGizmoTypeIntersect) {
                anAction = core3d::BooleanAction::BooleanIntersect;
            }
            aStatus.generation =
                anInteractor->booleanPreviewGeneration();
            aStatus.active = anInteractor->hasActiveBoolean(anAction);
            if (!aStatus.active) {
                return aStatus;
            }
            aStatus.state = Core3DPreviewState(
                anInteractor->booleanPreviewState());
            aStatus.canApply =
                aStatus.state == Core3DModelingPreviewStateReady
                && anInteractor->canApplyBoolean();
            return aStatus;
        }
        default:
            return aStatus;
    }
}

} // namespace

@interface Core3DViewController (PrimitiveManagerOutcomePrivate)

- (Core3DModelingOperationResult)core3d_tryApplyOperation:
    (PrimitiveGizmoType)operation
    attempt:(BOOL (^)(void))attempt;
- (Core3DModelingOperationResult)core3d_tryCancelOperation:
    (PrimitiveGizmoType)operation
    attempt:(BOOL (^)(void))attempt;
- (void)core3d_retainOperationWithStatus:
    (Core3DModelingPreviewStatus)status;
- (void)core3d_reconcileInactiveOperation:
    (PrimitiveGizmoType)operation;

@end

@implementation Core3DViewController (PrimitiveManager)

- (void)addPrimitivesFromJSON:(NSString *)json {
    [self setGizmoType:PrimitiveGizmoTypeMoveRotate];
    if (_currentGizmoType != PrimitiveGizmoTypeMoveRotate
        || [GLController getGizmoType] != PrimitiveGizmoTypeMoveRotate) {
        return;
    }
    [self setSelectionType:PrimitiveSelectionTypeShape];
    if (_currentSelectionType != PrimitiveSelectionTypeShape
        || [GLController getSelectionType] != PrimitiveSelectionTypeShape) {
        return;
    }
    [GLController addPrimitivesFromJSON:json];
    self.can_undo = YES;
    self.can_delete = YES;
    self.can_duplicate = YES;
    self.can_apply_material = YES;
    _availableGizmoTypes = @[@(PrimitiveGizmoTypeMoveRotate),
                             @(PrimitiveGizmoTypeScale),
                             @(PrimitiveGizmoTypeChamfer),
                             @(PrimitiveGizmoTypeMirror),
                             @(PrimitiveGizmoTypeLinearArray),
                             @(PrimitiveGizmoTypeSubtract),
                             @(PrimitiveGizmoTypeUnion),
                             @(PrimitiveGizmoTypeIntersect),
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
    if (_currentGizmoType != PrimitiveGizmoTypeMoveRotate
        || [GLController getGizmoType] != PrimitiveGizmoTypeMoveRotate) {
        return;
    }
    [self setSelectionType:PrimitiveSelectionTypeShape];
    if (_currentSelectionType != PrimitiveSelectionTypeShape
        || [GLController getSelectionType] != PrimitiveSelectionTypeShape) {
        return;
    }
    [GLController addPrimitive:primitiveType];
    self.can_undo = YES;
    self.can_delete = YES;
    self.can_duplicate = YES;
    self.can_apply_material = YES;
    _availableGizmoTypes = @[@(PrimitiveGizmoTypeMoveRotate),
                             @(PrimitiveGizmoTypeScale),
                             @(PrimitiveGizmoTypeChamfer),
                             @(PrimitiveGizmoTypeMirror),
                             @(PrimitiveGizmoTypeLinearArray),
                             @(PrimitiveGizmoTypeSubtract),
                             @(PrimitiveGizmoTypeUnion),
                             @(PrimitiveGizmoTypeIntersect),
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
    [self setGizmoType:PrimitiveGizmoTypeMoveRotate];
    if (_currentGizmoType != PrimitiveGizmoTypeMoveRotate
        || [GLController getGizmoType] != PrimitiveGizmoTypeMoveRotate) {
        return;
    }
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
	if (_currentGizmoType == PrimitiveGizmoTypeChamfer
		|| _currentGizmoType == PrimitiveGizmoTypeExtrude
		|| _currentGizmoType == PrimitiveGizmoTypeShell
		|| _currentGizmoType == PrimitiveGizmoTypeMirror
		|| _currentGizmoType == PrimitiveGizmoTypeLinearArray
		|| _currentGizmoType == PrimitiveGizmoTypeSubtract
		|| _currentGizmoType == PrimitiveGizmoTypeUnion
		|| _currentGizmoType == PrimitiveGizmoTypeIntersect) {
		[self setGizmoType:PrimitiveGizmoTypeMoveRotate];
		if (_currentGizmoType != PrimitiveGizmoTypeMoveRotate
			|| [GLController getGizmoType]
				!= PrimitiveGizmoTypeMoveRotate) {
			return;
		}
	}
	[GLController selectAll];
	[self sendNotifyUIState:UIStateChangingSelection];
}

- (void)duplicateSelected {
    [self setGizmoType:PrimitiveGizmoTypeMoveRotate];
    if (_currentGizmoType != PrimitiveGizmoTypeMoveRotate
        || [GLController getGizmoType] != PrimitiveGizmoTypeMoveRotate) {
        return;
    }
    [GLController duplicateSelected];
    self.can_apply_material = YES;
    [self sendNotifyUIState:UIStateChangingApplyMaterial
                            | UIStateChangingHistory];
}

- (Core3DModelingPreviewStatus)modelingPreviewStatusForGizmoType:
    (PrimitiveGizmoType)gizmoType {
    if (![NSThread isMainThread]) {
        return Core3DUnavailableModelingStatus(gizmoType);
    }
    return Core3DCurrentModelingStatus(GLController, gizmoType);
}

- (void)core3d_retainOperationWithStatus:
    (Core3DModelingPreviewStatus)status {
    self.can_apply = status.active && status.canApply;
    [self viewDidChangeViewportPresentationState];
    [self sendNotifyUIState:UIStateChangingGizmo
                             | UIStateChangingApply];
}

- (void)core3d_reconcileInactiveOperation:
    (PrimitiveGizmoType)operation {
    if (_currentGizmoType != operation) {
        return;
    }

    // A stale control may outlive its Core operation by one UI refresh. Only
    // converge that matching control after proving that cleanup cannot abort
    // a different live modeling operation (for example, Subtract queried
    // while Union remains active).
    const PrimitiveGizmoType modelingOperations[] = {
        PrimitiveGizmoTypeChamfer,
        PrimitiveGizmoTypeExtrude,
        PrimitiveGizmoTypeShell,
        PrimitiveGizmoTypeMirror,
        PrimitiveGizmoTypeLinearArray,
        PrimitiveGizmoTypeSubtract,
        PrimitiveGizmoTypeUnion,
        PrimitiveGizmoTypeIntersect,
    };
    for (const PrimitiveGizmoType candidate : modelingOperations) {
        if (candidate != operation
            && [self modelingPreviewStatusForGizmoType:candidate].active) {
            return;
        }
    }
    [self completeOperationInteraction];
}

- (Core3DModelingOperationResult)core3d_tryApplyOperation:
    (PrimitiveGizmoType)operation
    attempt:(BOOL (^)(void))attempt {
    if (![NSThread isMainThread] || GLController == nil
        || GLController.viewer == nullptr) {
        return Core3DModelingOperationResultNoActiveOperation;
    }
    const Core3DModelingPreviewStatus before =
        [self modelingPreviewStatusForGizmoType:operation];
    if (!before.active) {
        [self core3d_reconcileInactiveOperation:operation];
        return Core3DModelingOperationResultNoActiveOperation;
    }
    const BOOL isApplicable =
        before.state == Core3DModelingPreviewStateReady
        || before.state == Core3DModelingPreviewStateOutcomeUnknown;
    if (!isApplicable || !before.canApply) {
        return Core3DModelingOperationResultNotReady;
    }

    const BOOL reportedCompletion = attempt != nil && attempt();
    const Core3DModelingPreviewStatus after =
        [self modelingPreviewStatusForGizmoType:operation];
    if (reportedCompletion && !after.active) {
        [self completeOperationInteraction];
        return Core3DModelingOperationResultSucceeded;
    }
    if (after.active) {
        [self core3d_retainOperationWithStatus:after];
        if (after.state == Core3DModelingPreviewStateOutcomeUnknown) {
            return Core3DModelingOperationResultOutcomeUnknown;
        }
        return Core3DModelingOperationResultRetryableFailure;
    }

    // The underlying operation did not report a completed commit and no
    // recovery state remains. Reconcile the public tool UI with that
    // fail-closed cleanup, but never report a false success.
    [self completeOperationInteraction];
    return Core3DModelingOperationResultFailed;
}

- (Core3DModelingOperationResult)core3d_tryCancelOperation:
    (PrimitiveGizmoType)operation
    attempt:(BOOL (^)(void))attempt {
    if (![NSThread isMainThread] || GLController == nil
        || GLController.viewer == nullptr) {
        return Core3DModelingOperationResultNoActiveOperation;
    }
    const Core3DModelingPreviewStatus before =
        [self modelingPreviewStatusForGizmoType:operation];
    if (!before.active) {
        [self core3d_reconcileInactiveOperation:operation];
        return Core3DModelingOperationResultNoActiveOperation;
    }

    const BOOL reportedCompletion = attempt != nil && attempt();
    const Core3DModelingPreviewStatus after =
        [self modelingPreviewStatusForGizmoType:operation];
    if (reportedCompletion && !after.active) {
        [self completeOperationInteraction];
        return Core3DModelingOperationResultSucceeded;
    }
    if (after.active) {
        [self core3d_retainOperationWithStatus:after];
        if (after.state == Core3DModelingPreviewStateOutcomeUnknown) {
            return Core3DModelingOperationResultOutcomeUnknown;
        }
        return Core3DModelingOperationResultRetryableFailure;
    }

    [self completeOperationInteraction];
    return Core3DModelingOperationResultFailed;
}

- (void)setChamfer:(CGFloat)value {
    if (_currentGizmoType != PrimitiveGizmoTypeChamfer) { return; }
    [GLController setChamfer:value*100];
    self.can_apply = [GLController canApplyChamfer];
    [self viewDidChangeViewportPresentationState];
    [self sendNotifyUIState:UIStateChangingApply];
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

- (Core3DModelingOperationResult)tryApplyExtrusion {
    return [self core3d_tryApplyOperation:PrimitiveGizmoTypeExtrude
        attempt:^BOOL {
            return [GLController applyExtrusion];
        }];
}

- (Core3DModelingOperationResult)tryCancelExtrusion {
    return [self core3d_tryCancelOperation:PrimitiveGizmoTypeExtrude
        attempt:^BOOL {
            return [GLController cancelExtrusion];
        }];
}

- (BOOL)applyExtrusion {
    return [self tryApplyExtrusion]
        == Core3DModelingOperationResultSucceeded;
}

- (BOOL)cancelExtrusion {
    return [self tryCancelExtrusion]
        == Core3DModelingOperationResultSucceeded;
}

- (Core3DShellParameters)getShellParameters {
    return [GLController getShellParameters];
}

- (BOOL)setShellThickness:(CGFloat)thickness {
    if (_currentGizmoType != PrimitiveGizmoTypeShell) {
        self.can_apply = NO;
        [self sendNotifyUIState:UIStateChangingApply];
        return NO;
    }
    const BOOL didSet = [GLController setShellThickness:thickness];
    self.can_apply = [GLController canApplyShell];
    [self viewDidChangeViewportPresentationState];
    [self sendNotifyUIState:UIStateChangingApply];
    return didSet;
}

- (Core3DModelingOperationResult)tryApplyShell {
    return [self core3d_tryApplyOperation:PrimitiveGizmoTypeShell
        attempt:^BOOL {
            return [GLController applyShell];
        }];
}

- (Core3DModelingOperationResult)tryCancelShell {
    return [self core3d_tryCancelOperation:PrimitiveGizmoTypeShell
        attempt:^BOOL {
            return [GLController cancelShell];
        }];
}

- (BOOL)applyShell {
    return [self tryApplyShell]
        == Core3DModelingOperationResultSucceeded;
}

- (BOOL)cancelShell {
    return [self tryCancelShell]
        == Core3DModelingOperationResultSucceeded;
}

- (Core3DModelingOperationResult)tryApplyChamfer {
    return [self core3d_tryApplyOperation:PrimitiveGizmoTypeChamfer
        attempt:^BOOL {
            return [GLController applyChamfer];
        }];
}

- (Core3DModelingOperationResult)tryCancelChamfer {
    return [self core3d_tryCancelOperation:PrimitiveGizmoTypeChamfer
        attempt:^BOOL {
            return [GLController cancelChamfer];
        }];
}

- (void)applyChamfer {
    (void)[self tryApplyChamfer];
}

- (BOOL)cancelChamfer {
    return [self tryCancelChamfer]
        == Core3DModelingOperationResultSucceeded;
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
	// A successful document redraw can retire the native manipulator before
	// this wrapper runs. setGizmoType above reconciles the public cache first;
	// now publish one authoritative empty-selection/material update without
	// weakening the normal cache/native invariant.
	[GLController refreshSelectionState];
    [self sendNotifyUIState:UIStateChangingGizmo
                             | UIStateChangingSelection
                             | UIStateChangingDuplicate
                             | UIStateChangingDelete
                             | UIStateChangingApply
                             | UIStateChangingApplyMaterial
                             | UIStateChangingHistory];
}

- (Core3DModelingOperationResult)tryApplyMirror {
    return [self core3d_tryApplyOperation:PrimitiveGizmoTypeMirror
        attempt:^BOOL {
            return [GLController applyMirror];
        }];
}

- (Core3DModelingOperationResult)tryCancelMirror {
    return [self core3d_tryCancelOperation:PrimitiveGizmoTypeMirror
        attempt:^BOOL {
            return [GLController cancelMirror];
        }];
}

- (void)applyMirror {
    (void)[self tryApplyMirror];
}

- (void)cancelMirror {
    (void)[self tryCancelMirror];
}

- (BOOL)beginMirrorPlanePicking {
	if (_currentGizmoType != PrimitiveGizmoTypeMirror) {
		return NO;
	}
	return [GLController beginMirrorPlanePicking];
}

- (BOOL)cancelMirrorPlanePicking {
	return [GLController cancelMirrorPlanePicking];
}

- (BOOL)isPickingMirrorPlane {
	return [GLController isPickingMirrorPlane];
}

- (BOOL)hasCustomMirrorPlane {
	return [GLController hasCustomMirrorPlane];
}

- (BOOL)setMirrorPlaneOffset:(CGFloat)offset {
	if (_currentGizmoType != PrimitiveGizmoTypeMirror) {
		return NO;
	}
	return [GLController setMirrorPlaneOffset:offset];
}

- (Boundaries)getMirrorPlaneOffsetBoundaries {
	return [GLController getMirrorPlaneOffsetBoundaries];
}

- (BOOL)resetMirrorPlane {
	if (_currentGizmoType != PrimitiveGizmoTypeMirror) {
		return NO;
	}
	return [GLController resetMirrorPlane];
}

- (Core3DLinearArrayParameters)getLinearArrayParameters {
    return [GLController getLinearArrayParameters];
}

- (BOOL)setLinearArrayAxis:(Core3DLinearArrayAxis)axis {
    if (_currentGizmoType != PrimitiveGizmoTypeLinearArray) {
        return NO;
    }
    const BOOL didSet = [GLController setLinearArrayAxis:axis];
    const Core3DModelingPreviewStatus status =
        [self modelingPreviewStatusForGizmoType:
            PrimitiveGizmoTypeLinearArray];
    self.can_apply = status.canApply;
    [self sendNotifyUIState:UIStateChangingApply];
    return didSet;
}

- (BOOL)setLinearArrayCount:(NSInteger)count {
    if (_currentGizmoType != PrimitiveGizmoTypeLinearArray) {
        return NO;
    }
    const BOOL didSet = [GLController setLinearArrayCount:count];
    const Core3DModelingPreviewStatus status =
        [self modelingPreviewStatusForGizmoType:
            PrimitiveGizmoTypeLinearArray];
    self.can_apply = status.canApply;
    [self sendNotifyUIState:UIStateChangingApply];
    return didSet;
}

- (BOOL)setLinearArraySpacing:(CGFloat)spacing {
    if (_currentGizmoType != PrimitiveGizmoTypeLinearArray) {
        return NO;
    }
    const BOOL didSet = [GLController setLinearArraySpacing:spacing];
    const Core3DModelingPreviewStatus status =
        [self modelingPreviewStatusForGizmoType:
            PrimitiveGizmoTypeLinearArray];
    self.can_apply = status.canApply;
    [self sendNotifyUIState:UIStateChangingApply];
    return didSet;
}

- (Core3DModelingOperationResult)tryApplyLinearArray {
    return [self core3d_tryApplyOperation:PrimitiveGizmoTypeLinearArray
        attempt:^BOOL {
            return [GLController applyLinearArray];
        }];
}

- (Core3DModelingOperationResult)tryCancelLinearArray {
    return [self core3d_tryCancelOperation:PrimitiveGizmoTypeLinearArray
        attempt:^BOOL {
            return [GLController cancelLinearArray];
        }];
}

- (Core3DModelingOperationResult)tryApplySubtract {
    return [self core3d_tryApplyOperation:PrimitiveGizmoTypeSubtract
        attempt:^BOOL {
            return [GLController applySubtract];
        }];
}

- (Core3DModelingOperationResult)tryCancelSubtract {
    return [self core3d_tryCancelOperation:PrimitiveGizmoTypeSubtract
        attempt:^BOOL {
            return [GLController cancelSubtract];
        }];
}

- (void)applySubtract {
    (void)[self tryApplySubtract];
}

- (void)cancelSubtract {
    (void)[self tryCancelSubtract];
}

- (Core3DModelingOperationResult)tryApplyUnion {
    return [self core3d_tryApplyOperation:PrimitiveGizmoTypeUnion
        attempt:^BOOL {
            return [GLController applyUnion];
        }];
}

- (Core3DModelingOperationResult)tryCancelUnion {
    return [self core3d_tryCancelOperation:PrimitiveGizmoTypeUnion
        attempt:^BOOL {
            return [GLController cancelUnion];
        }];
}

- (void)applyUnion {
    (void)[self tryApplyUnion];
}

- (void)cancelUnion {
    (void)[self tryCancelUnion];
}

- (Core3DModelingOperationResult)tryApplyIntersect {
    return [self core3d_tryApplyOperation:PrimitiveGizmoTypeIntersect
        attempt:^BOOL {
            return [GLController applyIntersect];
        }];
}

- (Core3DModelingOperationResult)tryCancelIntersect {
    return [self core3d_tryCancelOperation:PrimitiveGizmoTypeIntersect
        attempt:^BOOL {
            return [GLController cancelIntersect];
        }];
}

- (void)applyIntersect {
    (void)[self tryApplyIntersect];
}

- (void)cancelIntersect {
    (void)[self tryCancelIntersect];
}

- (void)undo {
    const BOOL wasExtrusion =
        _currentGizmoType == PrimitiveGizmoTypeExtrude;
    const BOOL wasShell =
        _currentGizmoType == PrimitiveGizmoTypeShell;
    const BOOL wasLinearArray =
        _currentGizmoType == PrimitiveGizmoTypeLinearArray;
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
    if (wasLinearArray) {
        const Core3DModelingPreviewStatus status =
            [self modelingPreviewStatusForGizmoType:
                PrimitiveGizmoTypeLinearArray];
        if (!status.active) {
            [self completeOperationInteraction];
        } else {
            self.can_apply = status.canApply;
            [self viewDidChangeViewportPresentationState];
        }
        [self sendNotifyUIState:UIStateChangingGizmo
                                | UIStateChangingApply
                                | UIStateChangingHistory];
        return;
    }
    if (wasShell) {
        const Core3DModelingPreviewStatus status =
            [self modelingPreviewStatusForGizmoType:
                PrimitiveGizmoTypeShell];
        if (!status.active) {
            [self completeOperationInteraction];
        } else {
            self.can_apply = status.canApply;
            [self viewDidChangeViewportPresentationOverlay];
        }
        [self sendNotifyUIState:UIStateChangingGizmo
                                | UIStateChangingApply
                                | UIStateChangingHistory];
        return;
    }
    if (_currentGizmoType == PrimitiveGizmoTypeMirror
        || _currentGizmoType == PrimitiveGizmoTypeLinearArray
        || _currentGizmoType == PrimitiveGizmoTypeShell
        || _currentGizmoType == PrimitiveGizmoTypeSubtract
        || _currentGizmoType == PrimitiveGizmoTypeUnion
        || _currentGizmoType == PrimitiveGizmoTypeIntersect
        || _currentGizmoType == PrimitiveGizmoTypeExtrude) {
        self.can_apply = _currentGizmoType == PrimitiveGizmoTypeMirror
            ? [self modelingPreviewStatusForGizmoType:
                PrimitiveGizmoTypeMirror].canApply
            : _currentGizmoType == PrimitiveGizmoTypeLinearArray
                ? [self modelingPreviewStatusForGizmoType:
                    PrimitiveGizmoTypeLinearArray].canApply
            : _currentGizmoType == PrimitiveGizmoTypeExtrude
                ? [GLController canApplyExtrusion]
            : _currentGizmoType == PrimitiveGizmoTypeShell
                ? [self modelingPreviewStatusForGizmoType:
                    PrimitiveGizmoTypeShell].canApply
            : [GLController canApplyBoolean];
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply
                                | UIStateChangingHistory];
        return;
    }
    [self sendNotifyUIState:UIStateChangingHistory];
}

- (void)redo {
    const BOOL wasExtrusion =
        _currentGizmoType == PrimitiveGizmoTypeExtrude;
    const BOOL wasShell =
        _currentGizmoType == PrimitiveGizmoTypeShell;
    const BOOL wasLinearArray =
        _currentGizmoType == PrimitiveGizmoTypeLinearArray;
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
    if (wasLinearArray) {
        const Core3DModelingPreviewStatus status =
            [self modelingPreviewStatusForGizmoType:
                PrimitiveGizmoTypeLinearArray];
        if (!status.active) {
            [self completeOperationInteraction];
        } else {
            self.can_apply = status.canApply;
            [self viewDidChangeViewportPresentationState];
        }
        [self sendNotifyUIState:UIStateChangingGizmo
                                | UIStateChangingApply
                                | UIStateChangingHistory];
        return;
    }
    if (wasShell) {
        const Core3DModelingPreviewStatus status =
            [self modelingPreviewStatusForGizmoType:
                PrimitiveGizmoTypeShell];
        if (!status.active) {
            [self completeOperationInteraction];
        } else {
            self.can_apply = status.canApply;
            [self viewDidChangeViewportPresentationOverlay];
        }
        [self sendNotifyUIState:UIStateChangingGizmo
                                | UIStateChangingApply
                                | UIStateChangingHistory];
        return;
    }
    if (_currentGizmoType == PrimitiveGizmoTypeMirror
        || _currentGizmoType == PrimitiveGizmoTypeLinearArray
        || _currentGizmoType == PrimitiveGizmoTypeShell
        || _currentGizmoType == PrimitiveGizmoTypeSubtract
        || _currentGizmoType == PrimitiveGizmoTypeUnion
        || _currentGizmoType == PrimitiveGizmoTypeIntersect
        || _currentGizmoType == PrimitiveGizmoTypeExtrude) {
        self.can_apply = _currentGizmoType == PrimitiveGizmoTypeMirror
            ? [self modelingPreviewStatusForGizmoType:
                PrimitiveGizmoTypeMirror].canApply
            : _currentGizmoType == PrimitiveGizmoTypeLinearArray
                ? [self modelingPreviewStatusForGizmoType:
                    PrimitiveGizmoTypeLinearArray].canApply
            : _currentGizmoType == PrimitiveGizmoTypeExtrude
                ? [GLController canApplyExtrusion]
            : _currentGizmoType == PrimitiveGizmoTypeShell
                ? [self modelingPreviewStatusForGizmoType:
                    PrimitiveGizmoTypeShell].canApply
            : [GLController canApplyBoolean];
        [self viewDidChangeViewportPresentationState];
        [self sendNotifyUIState:UIStateChangingApply
                                | UIStateChangingHistory];
        return;
    }
    [self sendNotifyUIState:UIStateChangingHistory];
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
