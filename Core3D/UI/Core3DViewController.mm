//
//  Core3DViewController.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 28.03.2024.
//

#import "GLViewController.h"
#import "Core3DViewController.h"
#import "Core3DViewController+PrimitiveManager.h"
#import "Core3DViewController+GLViewControllerProtocol.h"
#import <Core3D/AssetBundle.h>

#include "GLViewController+Trick.h"
#include "../Common/dispatch_cancelable_block.h"
#include "XCAFDoc_DocumentTool.hxx"

@interface Core3DViewController () {
    BOOL _isSetuped;
    NSURL *_shouldLoadBundleUrl;
    std::atomic_bool _isLoading;
}

@end

@implementation Core3DViewController {
    __weak dispatch_cancelable_block_t _uiStateChangingBlock;
    UIStateChanging _sendingState;

    __weak dispatch_cancelable_block_t _modifiedAssetBlock;
}

- (void)dealloc {
    _glController = nil;
    NSLog(@"~Core3DViewController");
}

- (instancetype)init {
    if (self = [super init]) {
        [self commontInit];
    }

    return self;
}

- (instancetype)initWithNibName:(NSString *)nibNameOrNil bundle:(NSBundle *)nibBundleOrNil {
    if (self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil]) {
        [self commontInit];
    }

    return self;
}

- (instancetype)initWithCoder:(NSCoder *)coder {
    if (self = [super initWithCoder:coder]) {
        [self commontInit];
    }
    return self;
}

- (void)commontInit {
    if (!_glController) {
        _materialController = [[Core3DMaterialController alloc] init];
        _materialController.pass = (id<Core3DMaterialControllerCalls>)self;
        
        _glController = [[GLViewController alloc] init];
        GLController.delegate = self;

        _currentStateChanging = UIStateChangingNone;

        _currentSelectionType = PrimitiveSelectionTypeShape;
        _currentGizmoType = PrimitiveGizmoTypeNone;

        _can_add = YES;
        _can_redo = YES;
        _can_undo = YES;
        _can_delete = NO;
        _can_duplicate = NO;

        _availableGizmoTypes = @[];
        
        [[NSNotificationCenter defaultCenter] addObserver:self
            selector:@selector(receiveOcctDocumentNotification:)
            name:@"OcctDocumentChanges"
            object:nil];
    }
}

-(void) updateSelectionWithMaterial:(Core3DMaterial*)material color:(Core3DColor*)color {
    auto context = GLController.viewer->AisContext();
    
    auto doc = GLController.viewer->getDocument();
    Handle(XCAFDoc_ShapeTool) shapeTool = XCAFDoc_DocumentTool::ShapeTool (doc->Document()->Main());
    TDF_Label label;

    if(doc->ChangeDocument()->HasOpenCommand()) {
        doc->ChangeDocument()->CommitCommand();
    }
    
    doc->ChangeDocument()->NewCommand();
    
    NSMutableArray* materials = [NSMutableArray array];
    NSMutableArray* colors = [NSMutableArray array];

    for (context->InitSelected(); context->MoreSelected(); context->NextSelected()) {
        auto selected = context->SelectedInteractive();
        
        Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(selected);
        if (shape.IsNull() || shape->Shape().IsNull())
            continue;

        if(material == nil && color == nil) {
            if(shapeTool->FindShape(shape->Shape(), label)) {
                shape->SetMaterial(Graphic3d_NameOfMaterial_ShinyPlastified);
                shape->SetColor(Quantity_NOC_GRAY80);
                doc->SaveObjectMaterial(label, Graphic3d_NameOfMaterial_ShinyPlastified);
                doc->SaveObjectColor(label, Quantity_NOC_GRAY80);
            }
        }
        
        if(material != nil) {
            if(shapeTool->FindShape(shape->Shape(), label)) {
                Graphic3d_MaterialAspect materialAspect((Graphic3d_NameOfMaterial)material.identity);
                shape->UnsetColor();
                shape->SetMaterial(materialAspect);
                doc->SaveObjectMaterial(label, (Graphic3d_NameOfMaterial)material.identity);
                if(color == nil) {
                    shape->SetColor(materialAspect.Color());
                    doc->SaveObjectColor(label, (Quantity_NameOfColor)materialAspect.Color().Name());
                }
            }
        }
        
        if(color != nil) {
            if(shapeTool->FindShape(shape->Shape(), label))  {
                shape->SetColor(Quantity_Color((Quantity_NameOfColor)color.identity));
                doc->SaveObjectColor(label, (Quantity_NameOfColor)color.identity);
            }
        }

        if(material != nil) {
            [materials addObject:material];
        }
        if(color != nil) {
            [colors addObject:color];
        }
    }
    
    [self.materialController didChangeSelectionWithMaterials:[materials copy] colors:[colors copy]];

    doc->ChangeDocument()->CommitCommand();
    context->UpdateCurrentViewer();
    
    [self sendNotifyUIState: UIStateChangingApplyMaterial];
}

-(void) receiveOcctDocumentNotification:(NSNotification *) notification {
    if ([[notification name] isEqualToString:@"OcctDocumentChanges"]) {
        [self viewDidAssetModify];
    }
}

- (void)viewWillAppear:(BOOL)animated {
    [super viewWillAppear:animated];
    [self sendNotifyUIState: UIStateChangingGizmo
                            | UIStateChangingSelection
                            | UIStateChangingHistory
                            | UIStateChangingDelete
                            | UIStateChangingDuplicate
                            | UIStateChangingApplyMaterial
                            | UIStateChangingSnappingType];
}

-(UIView*) glView {
    return GLController.view;
}

- (void)viewDidLoad {
    [super viewDidLoad];

    [self.view insertSubview:GLController.view atIndex:0];
    GLController.view.translatesAutoresizingMaskIntoConstraints = NO;
    [GLController.view.topAnchor constraintEqualToAnchor:self.view.topAnchor constant:0.0].active = YES;
    [GLController.view.bottomAnchor constraintEqualToAnchor:self.view.bottomAnchor constant:0.0].active = YES;
    [GLController.view.leadingAnchor constraintEqualToAnchor:self.view.leadingAnchor constant:0.0].active = YES;
    [GLController.view.trailingAnchor constraintEqualToAnchor:self.view.trailingAnchor constant:0.0].active = YES;
}

- (void)setSelectionType:(PrimitiveSelectionType)type {
    if (_currentSelectionType != type) {
        [GLController setSelectionType:type];
        _currentSelectionType = type;

        _availableGizmoTypes = @[];
        switch (_currentSelectionType) {
            case PrimitiveSelectionTypeShape:
                if ([GLController isSelected]) {
                    _availableGizmoTypes = @[@(PrimitiveGizmoTypeMoveRotate),
                                             @(PrimitiveGizmoTypeScale),
                                             @(PrimitiveGizmoTypeChamfer),
                                             @(PrimitiveGizmoTypeMirror),
                                             @(PrimitiveGizmoTypeSubtract),
                                             @(PrimitiveGizmoTypeUnion),
                                             @(PrimitiveGizmoTypeMaterial)];
                } else {
                    [self setGizmoType:PrimitiveGizmoTypeMoveRotate];
                }
                _can_delete = [GLController isSelected];
                _can_duplicate = [GLController isSelected];
                break;
            case PrimitiveSelectionTypeEdge:
            case PrimitiveSelectionTypeFace:
            {
                BOOL isEmptyOfDisplayedObjects = [GLController isEmptyOfDisplayedObjects];
                BOOL isSelected = [GLController isSelected];
                if (!isEmptyOfDisplayedObjects && isSelected) {
                    _availableGizmoTypes = @[@(PrimitiveGizmoTypeChamfer)];
                }
                [GLController deselectAll];
            }
                _can_delete = NO;
                _can_duplicate = NO;
                [self setGizmoType:PrimitiveGizmoTypeNone];
                // ^^ in this case setGizmoType may not be called, because gizmoType doesn't change
                //    when the selection type is changed [PrimitiveGizmoTypeChamfer -> PrimitiveGizmoTypeChamfer],
                //    so we notify ui state manually
                break;

            default:
                break;
        }

        [self sendNotifyUIState:UIStateChangingSelection
                                 | UIStateChangingGizmo
                                 | UIStateChangingDelete
                                 | UIStateChangingDuplicate];
    }
}

- (void)setGizmoType:(PrimitiveGizmoType)type {
    if (_currentGizmoType != type) {
        [GLController setGizmoType:type];
        _currentGizmoType = type;
        switch (_currentGizmoType) {
            case PrimitiveGizmoTypeChamfer:
                self.can_apply = [_glController isSelected];
                break;
            case PrimitiveGizmoTypeSubtract:
            case PrimitiveGizmoTypeUnion:
                self.can_apply = [_glController canApplyBoolean];
                break;
            case PrimitiveGizmoTypeMirror:
                self.can_apply = YES;
                break;
            default:
                break;
        }
        [self sendNotifyUIState:UIStateChangingGizmo | UIStateChangingApply];
    }
}

- (void)addTestPrimitives {
    [_glController addTestPrimitives];
}

- (void)viewWillUpdateUIState:(UIStateChanging)state {}

- (void)viewDidSetup {
    if (!_isSetuped) {
        _isSetuped = YES;
        if (_shouldLoadBundleUrl != NULL) {
            [self loadFromBundle:_shouldLoadBundleUrl];
        }
    }
}

- (void)viewDidLoadFromBundle {
    if (_delegate && [_delegate respondsToSelector:@selector(viewDidLoadFromBundle)]) {
        [_delegate viewDidLoadFromBundle];
    }
}

- (void)viewDidAssetModify {}

- (void)sendNotifyUIState:(UIStateChanging)state {
#ifdef DEBUG
    if (state == UIStateChangingCoreInfoText) {
        [self viewWillUpdateUIState:state];
    }
#endif
    if (_uiStateChangingBlock != nil) {
        cancel_block(_uiStateChangingBlock);
        _uiStateChangingBlock = nil;
    }
    _sendingState |= state;
    __weak typeof(self) weakSelf = self;
    _uiStateChangingBlock = dispatch_after_delay(0.1, ^{
        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf) return;
        if (strongSelf->_sendingState != UIStateChangingNone) {
            [strongSelf viewWillUpdateUIState:strongSelf->_sendingState];
            strongSelf->_sendingState = UIStateChangingNone;
        }
    });
}

- (void)sendNotifyAssetModified {
    if (_modifiedAssetBlock != nil) {
        cancel_block(_modifiedAssetBlock);
        _modifiedAssetBlock = nil;
    }
    __weak typeof(self) weakSelf = self;
    _modifiedAssetBlock = dispatch_after_delay(0.3, ^{
        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf) return;
        [strongSelf viewDidAssetModify];
    });
}

- (PrimitiveSelectionType) currentSelectionType {
    return _currentSelectionType;
}

- (PrimitiveGizmoType) currentGizmoType {
    return _currentGizmoType;
}

- (void) setPreviewMode {
    _isPreviewMode = YES;
    [GLController setPreviewMode];
}

- (void)assetData:(void(^)(NSData *_Nullable))completion {
    if (_isLoading || !_isSetuped) {
        completion(NULL);
        return;
    }
    [GLController assetData:^(NSData * _Nullable data) {
        completion(data);
    }];
}

- (NSData *)thumbData {
    return [GLController thumbData];
}

- (void)loadFromBundle:(NSURL *)bundleUrl {
    if(bundleUrl == nil) {
        return;
    }
    
    _isLoading = true;
    if (!_isSetuped) {
        _shouldLoadBundleUrl = bundleUrl;
        return;
    }
    NSData *assetData = NULL;
    __auto_type bundleReader = [AssetBundle makeReaderWithUrl:bundleUrl];
    for (AssetBundleItem *item in bundleReader.items) {
        if (item.type == AssetBundleItemTypeAsset) {
            NSError *error = NULL;
            assetData = [NSData dataWithContentsOfURL:item.url options:kNilOptions error:&error];
            if (error) {
                NSLog(@"ERROR: load from bundle: %@", error.localizedDescription);
            } else {
                if (!assetData) {
                    NSLog(@"ERROR: load from bundle: %@, NULL DATA", error.localizedDescription);
                }
            }
            break;
        }
    }

    if (assetData) {
        __weak typeof(self) weakSelf = self;
        [GLController setAssetData:assetData completion:^{
            __strong typeof(weakSelf) strongSelf = weakSelf;
            if (!strongSelf) return;
            [(GLViewController *)strongSelf.glController fitAll];
            if ([strongSelf currentGizmoType] == PrimitiveGizmoTypeNone) {
                [strongSelf setGizmoType:PrimitiveGizmoTypeMoveRotate];
            }
            strongSelf->_isLoading = false;
            [strongSelf viewDidLoadFromBundle];
        }];
    } else {
        _isLoading = false;
    }
}

- (void)saveSnapshot {
    [GLController saveSnapshot];
}

- (BOOL)isEmptyOfDisplayedObjects {
    return [GLController isEmptyOfDisplayedObjects];
}

@end
