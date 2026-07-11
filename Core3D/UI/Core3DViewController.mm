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
#import "../Viewport/Core3DSceneSnapshotFactory.hpp"

#include "GLViewController+Trick.h"
#include "../Common/dispatch_cancelable_block.h"
#include "XCAFDoc_DocumentTool.hxx"

#include <cmath>
#include <cstdint>
#include <limits>

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
    [[NSNotificationCenter defaultCenter] removeObserver:self];
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
	auto transaction = doc->ChangeDocument();
	if (transaction.IsNull() || transaction->HasOpenCommand()) {
		return;
	}

	struct PendingStyle {
		Handle(AIS_Shape) shape;
		TDF_Label label;
		Graphic3d_NameOfMaterial material;
		Quantity_NameOfColor color;
	};
	std::vector<PendingStyle> pendingStyles;
    NSMutableArray* materials = [NSMutableArray array];
    NSMutableArray* colors = [NSMutableArray array];

    for (context->InitSelected(); context->MoreSelected(); context->NextSelected()) {
        auto selected = context->SelectedInteractive();
        Handle(AIS_Shape) shape = Handle(AIS_Shape)::DownCast(selected);
        if (shape.IsNull() || shape->Shape().IsNull()) {
            continue;
        }
		const TDF_Label label = doc->ShapeLabel(selected);
		if (label.IsNull()) {
			return;
        }

		Graphic3d_NameOfMaterial materialName = doc->MaterialNameForLabel(label);
		Quantity_NameOfColor colorName = doc->ColorNameForLabel(label);
		if (material == nil && color == nil) {
			materialName = Graphic3d_NameOfMaterial_ShinyPlastified;
			colorName = Quantity_NOC_GRAY80;
		} else {
			if (material != nil) {
				materialName = (Graphic3d_NameOfMaterial)material.identity;
				if (color == nil) {
					Graphic3d_MaterialAspect materialAspect(materialName);
					colorName = (Quantity_NameOfColor)materialAspect.Color().Name();
				}
			}
			if (color != nil) {
				colorName = (Quantity_NameOfColor)color.identity;
			}
        }
		pendingStyles.push_back({shape, label, materialName, colorName});

        if(material != nil) {
            [materials addObject:material];
        }
        if(color != nil) {
            [colors addObject:color];
        }
    }
	if (pendingStyles.empty()) {
		return;
	}

	try {
		transaction->NewCommand();
		for (const PendingStyle& style : pendingStyles) {
			doc->SaveObjectMaterial(style.label, style.material);
			doc->SaveObjectColor(style.label, style.color);
		}
		if (!transaction->CommitCommand()) {
			if (transaction->HasOpenCommand()) { transaction->AbortCommand(); }
			return;
		}
	} catch (...) {
		if (transaction->HasOpenCommand()) { transaction->AbortCommand(); }
		return;
	}

	for (const PendingStyle& style : pendingStyles) {
		style.shape->UnsetColor();
		style.shape->SetMaterial(style.material);
		style.shape->SetColor(style.color);
	}
	[self.materialController didChangeSelectionWithMaterials:[materials copy] colors:[colors copy]];
	doc->NotifyChanges();
    context->UpdateCurrentViewer();
    [self sendNotifyUIState: UIStateChangingApplyMaterial];
}

-(void) receiveOcctDocumentNotification:(NSNotification *) notification {
    if (![[notification name] isEqualToString:@"OcctDocumentChanges"]
        || ![notification.object isKindOfClass:NSValue.class]) {
        return;
    }

    auto activeDocument = GLController.viewer->getDocument();
    if (activeDocument.IsNull()
        || [(NSValue *)notification.object pointerValue] != activeDocument.get()) {
        return;
    }

    [self viewDidAssetModify];
}

- (void)viewWillAppear:(BOOL)animated {
    [super viewWillAppear:animated];
    [GLController requestRender];
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

    [self addChildViewController:GLController];
    [self.view insertSubview:GLController.view atIndex:0];
    GLController.view.translatesAutoresizingMaskIntoConstraints = NO;
    [GLController.view.topAnchor constraintEqualToAnchor:self.view.topAnchor constant:0.0].active = YES;
    [GLController.view.bottomAnchor constraintEqualToAnchor:self.view.bottomAnchor constant:0.0].active = YES;
    [GLController.view.leadingAnchor constraintEqualToAnchor:self.view.leadingAnchor constant:0.0].active = YES;
    [GLController.view.trailingAnchor constraintEqualToAnchor:self.view.trailingAnchor constant:0.0].active = YES;
    [GLController didMoveToParentViewController:self];
}

- (NSUInteger)viewportRenderedFrameCount {
    return GLController.renderedFrameCount;
}

- (CGSize)viewportDrawableSize {
    return GLController.drawableSize;
}

- (BOOL)isViewportRenderLoopRunning {
    return GLController.isRenderLoopRunning;
}

- (Core3DViewportRenderingAPI)viewportRenderingAPI {
    return (Core3DViewportRenderingAPI)GLController.renderingAPIVersion;
}

- (Core3DSceneSnapshot *)captureSceneSnapshot {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) {
        return nil;
    }

    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer = GLController.viewer;
        if (viewer == nullptr) {
            return nil;
        }

        const CGSize drawableSize = GLController.drawableSize;
        if (!std::isfinite(drawableSize.width)
            || !std::isfinite(drawableSize.height)
            || drawableSize.width < 1.0
            || drawableSize.height < 1.0
            || drawableSize.width > std::numeric_limits<std::uint32_t>::max()
            || drawableSize.height > std::numeric_limits<std::uint32_t>::max()) {
            return nil;
        }

        const auto snapshot = viewer->captureSceneSnapshot(
            static_cast<std::uint32_t>(std::llround(drawableSize.width)),
            static_cast<std::uint32_t>(std::llround(drawableSize.height)));
        return snapshot == nullptr
            ? nil
            : Core3DCreateSceneSnapshotDTO(*snapshot);
    } catch (...) {
        return nil;
    }
}

- (Core3DSceneFrameSnapshot *)captureSceneFrameSnapshot {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) {
        return nil;
    }

    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer = GLController.viewer;
        if (viewer == nullptr) {
            return nil;
        }

        const CGSize drawableSize = GLController.drawableSize;
        if (!std::isfinite(drawableSize.width)
            || !std::isfinite(drawableSize.height)
            || drawableSize.width < 1.0
            || drawableSize.height < 1.0
            || drawableSize.width > std::numeric_limits<std::uint32_t>::max()
            || drawableSize.height > std::numeric_limits<std::uint32_t>::max()) {
            return nil;
        }

        const auto frame = viewer->captureSceneFrameSnapshot(
            static_cast<std::uint32_t>(std::llround(drawableSize.width)),
            static_cast<std::uint32_t>(std::llround(drawableSize.height)));
        return !frame.has_value()
            ? nil
            : Core3DCreateSceneFrameSnapshotDTO(*frame);
    } catch (...) {
        return nil;
    }
}

- (Core3DScenePresentationOverlaySnapshot *)captureScenePresentationOverlay {
    if (![NSThread isMainThread] || !_isSetuped || GLController == nil) {
        return nil;
    }

    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer = GLController.viewer;
        if (viewer == nullptr) {
            return nil;
        }
        const auto overlay = viewer->captureScenePresentationOverlay();
        return overlay == nullptr
            ? nil
            : Core3DCreateScenePresentationOverlaySnapshotDTO(*overlay);
    } catch (...) {
        return nil;
    }
}

- (void)viewDidInvalidateSceneSnapshot {
    // Renderer-neutral extension point. The OpenGL backend owns invalidation;
    // clients may coalesce immutable snapshot publication for another renderer.
}

- (void)viewDidChangeViewportPresentationState {
    // Renderer-neutral observation point. Core3D's editing state is already
    // authoritative, while the UI notification remains deliberately debounced.
}

- (void)viewWillSelectAtDrawablePoint:(CGPoint)point
                         drawableSize:(CGSize)drawableSize {
    (void)point;
    (void)drawableSize;
    // Renderer-neutral observation point. OCCT remains authoritative and the
    // tap continues through its normal selection/tool path after this returns.
}

- (void)viewWillBeginPrimaryInteractionAtDrawablePoint:(CGPoint)point
                                          drawableSize:(CGSize)drawableSize {
    (void)point;
    (void)drawableSize;
    // Renderer-neutral observation point used to retain a presented frame
    // before the authoritative OCCT interaction invalidates it.
}

- (void)viewDidEndPrimaryInteractionCancelled:(BOOL)cancelled {
    (void)cancelled;
    // Renderer-neutral observation point. OCCT has already resolved the
    // interaction and alternate renderers may now capture a fresh scene.
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

        // GLController emits render invalidations while this method is still
        // reconciling the public selection/gizmo state. Observe once more only
        // after that state is authoritative so alternate renderers cannot stay
        // gated by an intermediate mode until the debounced UI refresh.
        [self viewDidChangeViewportPresentationState];
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
        // GLController invalidates the native view before the public gizmo and
        // capability state above is final. Publish one final-state observation
        // so alternate renderers cannot remain promoted over an OCCT preview.
        [self viewDidChangeViewportPresentationState];
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

- (void)viewDidFailToLoadFromBundle:(Core3DAssetLoadResult)result {
    if (_delegate && [_delegate respondsToSelector:@selector(viewDidFailToLoadFromBundle:)]) {
        [_delegate viewDidFailToLoadFromBundle:result];
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
        [self viewDidFailToLoadFromBundle:Core3DAssetLoadResultInternalFailure];
        return;
    }
    
    _isLoading = true;
    if (!_isSetuped) {
        _shouldLoadBundleUrl = bundleUrl;
        return;
    }
    NSData *assetData = nil;
    BOOL foundAssetItem = NO;
    Core3DAssetLoadResult readFailure = Core3DAssetLoadResultInvalidData;
    __auto_type bundleReader = [AssetBundle makeReaderWithUrl:bundleUrl];
    for (AssetBundleItem *item in bundleReader.items) {
        if (item.type == AssetBundleItemTypeAsset) {
            foundAssetItem = YES;
            NSError *error = nil;
            assetData = [NSData dataWithContentsOfURL:item.url options:kNilOptions error:&error];
            if (error) {
                NSLog(@"ERROR: load from bundle: %@", error.localizedDescription);
                readFailure = Core3DAssetLoadResultTemporaryFileFailure;
            } else {
                if (!assetData) {
                    NSLog(@"ERROR: load from bundle: %@, NULL DATA", error.localizedDescription);
                    readFailure = Core3DAssetLoadResultTemporaryFileFailure;
                }
            }
            break;
        }
    }

    if (assetData) {
        __weak typeof(self) weakSelf = self;
        [GLController setAssetData:assetData completion:^(Core3DAssetLoadResult result) {
            __strong typeof(weakSelf) strongSelf = weakSelf;
            if (!strongSelf) return;
            strongSelf->_isLoading = false;
            if (result != Core3DAssetLoadResultSuccess) {
                [strongSelf viewDidFailToLoadFromBundle:result];
                return;
            }
            [(GLViewController *)strongSelf.glController fitAll];
            if ([strongSelf currentGizmoType] == PrimitiveGizmoTypeNone) {
                [strongSelf setGizmoType:PrimitiveGizmoTypeMoveRotate];
            }
            [strongSelf viewDidLoadFromBundle];
        }];
    } else {
        _isLoading = false;
        [self viewDidFailToLoadFromBundle:foundAssetItem
            ? readFailure
            : Core3DAssetLoadResultInvalidData];
    }
}

- (void)saveSnapshot {
    [GLController saveSnapshot];
}

- (BOOL)isEmptyOfDisplayedObjects {
    return [GLController isEmptyOfDisplayedObjects];
}

- (NSInteger)numberOfDisplayedShapes {
    return [GLController numberOfDisplayedShapes];
}

@end
