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

Core3DPBRMaterial* Core3DSelectionPBRMaterial(
    const XCAFDoc_VisMaterialPBR& material,
    const BOOL supportsScalarEditing) {
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
    supportsScalarEditing:supportsScalarEditing];
}

} // namespace

@implementation Core3DViewController(GLViewControllerProtocol)

- (void)viewer:(id)sender didChangeSelections:(core3d::selection_t)selections {

    assert(_currentGizmoType == [GLController getGizmoType]);
    self.can_apply_material = (selections > 0);
    
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
                document->SupportsScalarPBRMaterialEditingForLabel(label));
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
            editablePBR, YES);
        if (pbr != nil) {
            [pbrMaterials addObject:pbr];
        }
    }

    [self.materialController didChangeSelectionWithMaterials:[materials copy] colors:[colors copy]];
    [self.materialController didChangeSelectionWithPBRMaterials:[pbrMaterials copy]];
     
    switch (_currentGizmoType) {
        case PrimitiveGizmoTypeSubtract:
        case PrimitiveGizmoTypeUnion:
            self.can_delete = false;
            self.can_duplicate = false;
            break;

        default:
            self.can_delete = selections & core3d::Core3DViewer::kSelectionTypeManipulator;
            self.can_duplicate = selections & core3d::Core3DViewer::kSelectionTypeManipulator;
            break;
    }

    if (selections & core3d::Core3DViewer::kSelectionTypeManipulator
                    || selections & core3d::Core3DViewer::kSelectionTypeObject) { // FIXME: it should be reviewed
        _currentSelectionType = [GLController getSelectionType];
        switch (_currentSelectionType) {
            case PrimitiveSelectionTypeEdge:
            case PrimitiveSelectionTypeFace:
                _availableGizmoTypes = @[@(PrimitiveGizmoTypeChamfer)];
                break;
            case PrimitiveSelectionTypeShape:
                _availableGizmoTypes = @[@(PrimitiveGizmoTypeMoveRotate),
                                         @(PrimitiveGizmoTypeScale),
                                         @(PrimitiveGizmoTypeChamfer),
                                         @(PrimitiveGizmoTypeMirror),
                                         @(PrimitiveGizmoTypeSubtract),
                                         @(PrimitiveGizmoTypeUnion),
                                         @(PrimitiveGizmoTypeMaterial)];
                break;

            default:
                break;
        }

        switch (_currentGizmoType) {
            case PrimitiveGizmoTypeChamfer:
                self.can_apply = true;
                break;
            case PrimitiveGizmoTypeSubtract:
            case PrimitiveGizmoTypeUnion:
                self.can_apply = [GLController canApplyBoolean];
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
                self.can_apply = false;
                break;
            case PrimitiveGizmoTypeChamfer:
                self.can_apply = false;
                break;

            default:
                _availableGizmoTypes = @[];
                break;
        }
    }

    [self viewDidChangeViewportPresentationState];
    [self sendNotifyUIState:UIStateChangingGizmo
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
