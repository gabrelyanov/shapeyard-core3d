//
//  Core3DViewController+GLViewControllerProtocol.mm
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 18.04.2024.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#import "GLViewController.h"

#include "GLViewController+Trick.h"
#include "Core3DViewController+GLViewControllerProtocol.h"
#include "Core3DViewController+PrimitiveManager.h"

#include <cmath>

namespace {

bool Core3DIsBooleanGizmo(const PrimitiveGizmoType type) noexcept {
    return type == PrimitiveGizmoTypeSubtract
        || type == PrimitiveGizmoTypeUnion
        || type == PrimitiveGizmoTypeIntersect;
}

bool Core3DPublicSelectionTypeMatchesMode(
    const PrimitiveSelectionType type,
    const core3d::ShapeSelectionMode mode) noexcept {
    switch (type) {
        case PrimitiveSelectionTypeShape:
            return mode == core3d::ShapeSelectionMode::WholeShape;
        case PrimitiveSelectionTypeFace:
            return mode == core3d::ShapeSelectionMode::Face;
        case PrimitiveSelectionTypeEdge:
            return mode == core3d::ShapeSelectionMode::Edge;
        case PrimitiveSelectionTypeVertex:
            return mode == core3d::ShapeSelectionMode::Vertex;
        case PrimitiveSelectionTypeNone:
        default:
            return false;
    }
}

Core3DPBRMaterial* Core3DSelectionPBRMaterial(
    const XCAFDoc_VisMaterialPBR& material,
    const BOOL supportsScalarEditing,
    const BOOL supportsBaseColorTextureEditing,
    const BOOL supportsEmissiveTextureEditing,
    const BOOL supportsMetallicRoughnessTextureEditing,
    const BOOL supportsOcclusionTextureEditing) {
    if (!material.IsDefined || !std::isfinite(material.Metallic)
        || !std::isfinite(material.Roughness)) {
        return nil;
    }
    Standard_Real red = 0.0;
    Standard_Real green = 0.0;
    Standard_Real blue = 0.0;
    material.BaseColor.GetRGB().Values(
        red, green, blue, Quantity_TOC_sRGB);
    UIColor* color = [UIColor colorWithRed:red
                                     green:green
                                      blue:blue
                                     alpha:material.BaseColor.Alpha()];
    return [[Core3DPBRMaterial alloc]
        initWithBaseColor:color
                 metallic:material.Metallic
                roughness:material.Roughness
    supportsScalarEditing:supportsScalarEditing
      hasBaseColorTexture:!material.BaseColorTexture.IsNull()
supportsBaseColorTextureEditing:supportsBaseColorTextureEditing
      hasEmissiveTexture:!material.EmissiveTexture.IsNull()
supportsEmissiveTextureEditing:supportsEmissiveTextureEditing
hasMetallicRoughnessTexture:!material.MetallicRoughnessTexture.IsNull()
supportsMetallicRoughnessTextureEditing:supportsMetallicRoughnessTextureEditing
hasOcclusionTexture:!material.OcclusionTexture.IsNull()
supportsOcclusionTextureEditing:supportsOcclusionTextureEditing];
}

} // namespace

@implementation Core3DViewController(GLViewControllerProtocol)

- (void)viewerDidChangeLinearArrayPresentationOverlay:(id)sender {
    (void)sender;
    if (_currentGizmoType != PrimitiveGizmoTypeLinearArray
        || [GLController getGizmoType] != PrimitiveGizmoTypeLinearArray) {
        return;
    }
    const Core3DModelingPreviewStatus status =
        [self modelingPreviewStatusForGizmoType:
            PrimitiveGizmoTypeLinearArray];
    if (!status.active) {
        return;
    }
    self.can_apply = status.canApply;
    [self viewDidChangeViewportPresentationOverlay];
}

- (void)viewerDidChangeRadialArrayPresentationOverlay:(id)sender {
    (void)sender;
    if (_currentGizmoType != PrimitiveGizmoTypeRadialArray
        || [GLController getGizmoType] != PrimitiveGizmoTypeRadialArray) {
        return;
    }
    const Core3DModelingPreviewStatus status =
        [self modelingPreviewStatusForGizmoType:
            PrimitiveGizmoTypeRadialArray];
    if (!status.active) {
        return;
    }
    self.can_apply = status.canApply;
    [self viewDidChangeViewportPresentationOverlay];
    [self sendNotifyUIState:UIStateChangingApply];
}

- (void)viewerDidChangeShellPresentationOverlay:(id)sender {
    (void)sender;
    if (_currentGizmoType != PrimitiveGizmoTypeShell
        || [GLController getGizmoType] != PrimitiveGizmoTypeShell) {
        return;
    }
    const Core3DModelingPreviewStatus status =
        [self modelingPreviewStatusForGizmoType:PrimitiveGizmoTypeShell];
    if (!status.active) {
        [self completeOperationInteraction];
        return;
    }
    self.can_apply = status.canApply;
    [self viewDidChangeViewportPresentationOverlay];
    [self sendNotifyUIState:UIStateChangingApply];
}

- (void)viewer:(id)sender didChangeSelections:(core3d::selection_t)selections {

    assert(_currentGizmoType == [GLController getGizmoType]);
    const BOOL duplicateRecoveryPending =
        [GLController hasUnresolvedEdit];
    self.can_apply_material = !duplicateRecoveryPending
        && (selections > 0);
    
    auto context = GLController.viewer->AisContext();
    auto document = GLController.viewer->getDocument();
    NSMutableArray* materials = [NSMutableArray array];
    NSMutableArray* colors = [NSMutableArray array];
    NSMutableArray* pbrMaterials = [NSMutableArray array];
    for (context->InitSelected(); context->MoreSelected(); context->NextSelected()) {
        auto selected = context->SelectedInteractive();
        Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(selected);
        
        if(shape.IsNull()) {
            continue;
        }
        
        const TDF_Label label = document->ShapeLabel(selected);
        XCAFDoc_VisMaterialPBR nativePBR;
        if (!label.IsNull()
            && document->TryEffectivePBRMaterialForLabel(
                label, nativePBR)) {
            Core3DPBRMaterial* pbr = Core3DSelectionPBRMaterial(
                nativePBR,
                document->SupportsScalarPBRMaterialEditingForLabel(label),
                document->SupportsBaseColorTextureEditingForLabel(label),
                document->SupportsEmissiveTextureEditingForLabel(label),
                document->SupportsMaterialTextureEditingForLabel(label, OcctMaterialTextureSlot::MetallicRoughness),
                document->SupportsMaterialTextureEditingForLabel(label, OcctMaterialTextureSlot::Occlusion));
            if (pbr != nil) {
                [pbrMaterials addObject:pbr];
            }
            continue;
        }

        auto name_of_material = GLController.viewer->getDocument()->MaterialNameForShape(shape);
        Graphic3d_MaterialAspect ma(name_of_material);
        Core3DMaterial* m = [[Core3DMaterial alloc] initWithIdentity:name_of_material
        nameKey:[NSString stringWithFormat:@"%s", ma.MaterialName()]
                                                        defaultColor:[self.materialController findColorWithName:ma.Color().Name()]];
        [materials addObject:m];
        
        Quantity_NameOfColor name;
        if (!label.IsNull()) {
            // OCAF is the source of truth for committed object styles. AIS can
            // transiently report no explicit color immediately after a load.
            name = document->ColorNameForLabel(label);
        } else {
            if (!shape->HasColor()) {
                continue;
            }
            Quantity_Color qc;
            shape->Color(qc);
            name = qc.Name();
        }
        Core3DColor* color = [self.materialController findColorWithName:name];
        if(color != nil) {
            [colors addObject:color];
        }

        const Graphic3d_PBRMaterial& legacyPBR = ma.PBRMaterial();
        XCAFDoc_VisMaterialPBR editablePBR;
        editablePBR.BaseColor = Quantity_ColorRGBA(
            Quantity_Color(name), legacyPBR.Alpha());
        editablePBR.EmissiveFactor = legacyPBR.Emission();
        editablePBR.Metallic = legacyPBR.Metallic();
        editablePBR.Roughness = legacyPBR.NormalizedRoughness();
        editablePBR.RefractionIndex = legacyPBR.IOR();
        Core3DPBRMaterial* pbr = Core3DSelectionPBRMaterial(
            editablePBR, YES, YES, YES, YES, YES);
        if (pbr != nil) {
            [pbrMaterials addObject:pbr];
        }
    }

    [self.materialController didChangeSelectionWithMaterials:[materials copy] colors:[colors copy]];
    [self.materialController didChangeSelectionWithPBRMaterials:[pbrMaterials copy]];
     
    switch (_currentGizmoType) {
        case PrimitiveGizmoTypeChamfer:
        case PrimitiveGizmoTypeSubtract:
        case PrimitiveGizmoTypeUnion:
        case PrimitiveGizmoTypeIntersect:
        case PrimitiveGizmoTypeExtrude:
        case PrimitiveGizmoTypeLinearArray:
        case PrimitiveGizmoTypeRadialArray:
        case PrimitiveGizmoTypeShell:
            self.can_delete = false;
            self.can_duplicate = false;
            break;

        default:
            self.can_delete = !duplicateRecoveryPending
                && (selections
                    & core3d::Core3DViewer::kSelectionTypeManipulator);
            self.can_duplicate = duplicateRecoveryPending
                || (selections
                    & core3d::Core3DViewer::kSelectionTypeManipulator);
            break;
    }

    if (selections & core3d::Core3DViewer::kSelectionTypeManipulator
                    || selections & core3d::Core3DViewer::kSelectionTypeObject) { // FIXME: it should be reviewed
        const std::shared_ptr<core3d::ShapeInteractor> shapeInteractor =
            GLController.viewer == nullptr
                ? nullptr
                : GLController.viewer->getShapeInteractor();
        const std::shared_ptr<core3d::ObjectInteractor> objectInteractor =
            GLController.viewer == nullptr
                ? nullptr
                : GLController.viewer->getObjectInteractor();
        const PrimitiveSelectionType strictSelectionType =
            [GLController getSelectionType];
        const BOOL preservesMirrorPlanePickingMode =
            _currentSelectionType == PrimitiveSelectionTypeShape
            && _currentGizmoType == PrimitiveGizmoTypeMirror
            && [GLController getGizmoType] == PrimitiveGizmoTypeMirror
            && [GLController isPickingMirrorPlane]
            && shapeInteractor != nullptr
            && shapeInteractor->getSelectionMode()
                == core3d::ShapeSelectionMode::WholeShape
            && objectInteractor != nullptr
            && objectInteractor->mirrorPlanePickingAuthorityMatches();
        const BOOL preservesRetainedOperationMode =
            strictSelectionType == PrimitiveSelectionTypeNone
            && _currentGizmoType == [GLController getGizmoType]
            && shapeInteractor != nullptr
            && Core3DPublicSelectionTypeMatchesMode(
                _currentSelectionType,
                shapeInteractor->getSelectionMode())
            && GLController.viewer != nullptr
            && GLController.viewer
                ->selectionModeAuthorityAllowsRetainedOperation();
        if (!preservesMirrorPlanePickingMode
            && !preservesRetainedOperationMode) {
            _currentSelectionType = strictSelectionType;
        }
        // Mirror plane picking temporarily adds Face selection alongside the
        // accepted Object mode and deactivates its manipulator detector. The
        // strict native getter therefore reports None until that private mode
        // ledger is restored. Keep the public rail on its accepted logical
        // mode while the picker or a validated typed operation owns those
        // temporary presentations. Extrude/Shell retain Face; object tools
        // retain Shape. The strict GL getter remains fail-closed; only the
        // logical rail survives until the ledger reconciles.
        switch (_currentSelectionType) {
            case PrimitiveSelectionTypeEdge:
                _availableGizmoTypes = @[@(PrimitiveGizmoTypeChamfer)];
                break;
            case PrimitiveSelectionTypeFace:
                _availableGizmoTypes = @[
                    @(PrimitiveGizmoTypeChamfer),
                    @(PrimitiveGizmoTypeExtrude),
                    @(PrimitiveGizmoTypeShell)
                ];
                break;
            case PrimitiveSelectionTypeShape:
                _availableGizmoTypes = @[@(PrimitiveGizmoTypeMoveRotate),
                                         @(PrimitiveGizmoTypeScale),
                                         @(PrimitiveGizmoTypeChamfer),
                                         @(PrimitiveGizmoTypeMirror),
                                         @(PrimitiveGizmoTypeLinearArray),
                                         @(PrimitiveGizmoTypeRadialArray),
                                         @(PrimitiveGizmoTypeSubtract),
                                         @(PrimitiveGizmoTypeUnion),
                                         @(PrimitiveGizmoTypeIntersect),
                                         @(PrimitiveGizmoTypeMaterial)];
                break;

            default:
                _availableGizmoTypes = @[];
                self.can_delete = NO;
                self.can_duplicate = NO;
                self.can_apply_material = NO;
                break;
        }

        switch (_currentGizmoType) {
            case PrimitiveGizmoTypeChamfer:
                self.can_apply = [GLController canApplyChamfer];
                break;
            case PrimitiveGizmoTypeSubtract:
            case PrimitiveGizmoTypeUnion:
            case PrimitiveGizmoTypeIntersect:
                self.can_apply = [GLController canApplyBoolean];
                break;
            case PrimitiveGizmoTypeExtrude:
                self.can_apply =
                    [self modelingPreviewStatusForGizmoType:
                        PrimitiveGizmoTypeExtrude].canApply;
                break;
			case PrimitiveGizmoTypeMirror:
				self.can_apply =
					[self modelingPreviewStatusForGizmoType:
						PrimitiveGizmoTypeMirror].canApply;
				break;
            case PrimitiveGizmoTypeLinearArray:
                self.can_apply =
                    [self modelingPreviewStatusForGizmoType:
                        PrimitiveGizmoTypeLinearArray].canApply;
                break;
            case PrimitiveGizmoTypeRadialArray:
                self.can_apply =
                    [self modelingPreviewStatusForGizmoType:
                        PrimitiveGizmoTypeRadialArray].canApply;
                break;
            case PrimitiveGizmoTypeShell:
                self.can_apply =
                    [self modelingPreviewStatusForGizmoType:
                        PrimitiveGizmoTypeShell].canApply;
                break;
            default:
                break;
        }

//        if ((_currentGizmoType == PrimitiveGizmoTypeChamfer) && ([GLController numberOfDetectedEdges] == 0)) {
//            _availableGizmoTypes = @[];
//        }
    } else {
        // tap to empty place
//        if (_currentGizmoType == PrimitiveGizmoTypeChamfer) {
//            if ([self respondsToSelector:@selector(cancelChamfer)]) { //FIXME: so, so ...
//                [self performSelector:@selector(cancelChamfer)];
//            }
//        }
//        if (_currentGizmoType == PrimitiveGizmoTypeSubtract) {
//            if ([self respondsToSelector:@selector(cancelSubtract)]) { //FIXME: so, so ...
//                [self performSelector:@selector(cancelSubtract)];
//            }
//        }
//        if (_currentGizmoType == PrimitiveGizmoTypeUnion) {
//            if ([self respondsToSelector:@selector(cancelUnion)]) { //FIXME: so, so ...
//                [self performSelector:@selector(cancelUnion)];
//            }
//        }

        switch (_currentGizmoType) {
            case PrimitiveGizmoTypeMaterial:
                [self completeOperationInteraction];
                break;
            case PrimitiveGizmoTypeSubtract:
            case PrimitiveGizmoTypeUnion:
            case PrimitiveGizmoTypeIntersect:
                // A ready Boolean preview owns and suppresses its source
                // presentations, so an empty AIS selection is expected here.
                // The retained operation remains the authority for Apply.
                self.can_apply = [GLController canApplyBoolean];
                break;
            case PrimitiveGizmoTypeExtrude:
                // The source/candidate pair is deliberately deactivated while
                // Ready and remains the sole retry authority while its commit
                // outcome is unknown. Refresh from typed state in both cases.
                self.can_apply =
                    [self modelingPreviewStatusForGizmoType:
                        PrimitiveGizmoTypeExtrude].canApply;
                break;
            case PrimitiveGizmoTypeChamfer:
                // A retained Bevel preview suppresses its source presentation,
                // so empty AIS selection does not mean the operation ended.
                self.can_apply = [GLController canApplyChamfer];
                break;
			case PrimitiveGizmoTypeMirror:
				// Plane picking intentionally preserves the source selection and
				// manipulator; refresh Apply from typed Mirror state after the tap.
				self.can_apply =
					[self modelingPreviewStatusForGizmoType:
						PrimitiveGizmoTypeMirror].canApply;
				break;
            case PrimitiveGizmoTypeLinearArray:
                // The source stays selected while the owned transient copies
                // are nonselectable. Typed state remains authoritative if a
                // lifecycle or recovery path briefly clears AIS selection.
                self.can_apply =
                    [self modelingPreviewStatusForGizmoType:
                        PrimitiveGizmoTypeLinearArray].canApply;
                break;
            case PrimitiveGizmoTypeRadialArray:
                // The source remains authoritative and selected while preview
                // instances are nonselectable; retained typed state wins over
                // a transiently empty AIS selection publication.
                self.can_apply =
                    [self modelingPreviewStatusForGizmoType:
                        PrimitiveGizmoTypeRadialArray].canApply;
                break;
            case PrimitiveGizmoTypeShell:
                // The captured opening face remains authoritative while the
                // source presentation is suppressed by a ready preview.
                self.can_apply =
                    [self modelingPreviewStatusForGizmoType:
                        PrimitiveGizmoTypeShell].canApply;
                break;

            default:
                _availableGizmoTypes = @[];
                break;
        }
    }

    if (duplicateRecoveryPending) {
        // The committed OCAF result may have no selectable AIS owner until the
        // next explicit Duplicate action repairs presentation. Keep only that
        // repair capability reachable; every other edit remains fail closed.
        _availableGizmoTypes = @[];
        self.can_delete = NO;
        self.can_duplicate = YES;
        self.can_apply = NO;
        self.can_apply_material = NO;
    }

    [self viewDidChangeViewportPresentationState];
    [self sendNotifyUIState:UIStateChangingGizmo
                             | UIStateChangingAdd
                             | UIStateChangingDelete
                             | UIStateChangingDuplicate
                             | UIStateChangingHistory
                             | UIStateChangingApply
                             | UIStateChangingApplyMaterial];
}

- (void)viewer:(id)sender
    willBeginPrimaryInteractionAtDrawablePoint:(CGPoint)point
                                  drawableSize:(CGSize)drawableSize {
    (void)sender;
    [self viewWillBeginPrimaryInteractionAtDrawablePoint:point
                                            drawableSize:drawableSize];
}

- (void)viewer:(id)sender
    didEndPrimaryInteractionCancelled:(BOOL)cancelled {
    (void)sender;
    [self viewDidEndPrimaryInteractionCancelled:cancelled];
}

- (void)viewer:(id)sender
    willSelectAtDrawablePoint:(CGPoint)point
                 drawableSize:(CGSize)drawableSize {
    (void)sender;
    [self viewWillSelectAtDrawablePoint:point drawableSize:drawableSize];
}

- (void)viewerDidFailToRetainBooleanMode:(id)sender {
    (void)sender;
    if (Core3DIsBooleanGizmo(_currentGizmoType)) {
        [self completeOperationInteraction];
    }
}

- (void)didSetupViewer:(id)sender { 
    [self viewDidSetup];
}

- (void)didInvalidateSceneSnapshot:(id)sender {
    [self viewDidInvalidateSceneSnapshot];
}

- (void)didChangeStatusString:(NSString *)status {
    self->_coreInfoText = status;
    [self sendNotifyUIState:UIStateChangingCoreInfoText];
}



@end
