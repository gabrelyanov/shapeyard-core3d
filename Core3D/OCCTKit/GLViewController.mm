// Copyright (c) 2017 OPEN CASCADE SAS
//
// This file is part of the examples of the Open CASCADE Technology software library.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE

#import <Foundation/Foundation.h>

#import "GLViewController.h"
#import "GLView.h"

#import "NSString+StdString.h"

#include "ConstructorManipulator.hpp"

#include <gp_Quaternion.hxx>
#include <cstring>

using namespace core3d;

namespace {

constexpr char kCbfMagic[] = "BINFILE";

BOOL HasCbfMagic(NSData *data) {
    constexpr NSUInteger magicLength = sizeof(kCbfMagic) - 1;
    return data != nil
        && data.length >= magicLength
        && std::memcmp(data.bytes, kCbfMagic, magicLength) == 0;
}

Core3DAssetLoadResult AssetLoadResultFromImportResult(AssetImportResult result) {
    switch (result) {
        case AssetImportResult::Success:
            return Core3DAssetLoadResultSuccess;
        case AssetImportResult::InvalidData:
            return Core3DAssetLoadResultInvalidData;
        case AssetImportResult::TemporaryFileFailure:
            return Core3DAssetLoadResultTemporaryFileFailure;
        case AssetImportResult::Busy:
            return Core3DAssetLoadResultBusy;
        case AssetImportResult::UnsupportedVersion:
            return Core3DAssetLoadResultUnsupportedVersion;
        case AssetImportResult::InternalFailure:
            return Core3DAssetLoadResultInternalFailure;
    }
}

void CompleteAssetLoadOnMain(void (^completion)(Core3DAssetLoadResult),
                             Core3DAssetLoadResult result) {
    dispatch_async(dispatch_get_main_queue(), ^{
        completion(result);
    });
}

} // namespace

@interface GLViewController () <UIGestureRecognizerDelegate>
- (void)endActiveRenderingInteractions;
- (void)endRawPrimaryInteractionIfNeededCancelled:(BOOL)cancelled;
@end

@implementation GLViewController {
    dispatch_queue_t _assetDataQueue;
    BOOL _cancelTouches;
    BOOL _didSetupViewer;
    BOOL _rawTouchRendering;
    BOOL _rawTouchWasCancelled;
    NSMutableSet<UITouch *> *_rawViewportTouches;
    BOOL _pinchRendering;
    BOOL _panRendering;
    CGPoint _pinchPreviousTouch[2];
    UITapGestureRecognizer *_tapRecognizer;
    BOOL _suppressTapSelectionForManipulatorInteraction;
}

- (void)dealloc {
    [[NSNotificationCenter defaultCenter] removeObserver:self];
    if (_viewer != nullptr) {
        GLView *view = self.isViewLoaded ? [self viewportView] : nil;
        if (view != nil) {
            const std::shared_ptr<Core3DViewer> viewer = _viewer;
            const BOOL didRelease = [view performWithRenderingContext:^{
                viewer->release();
            }];
            if (!didRelease) {
                NSLog(@"Core3D rendering context unavailable during teardown; using safe fallback.");
                EAGLContext *previousContext = EAGLContext.currentContext;
                [EAGLContext setCurrentContext:nil];
                viewer->release();
                [EAGLContext setCurrentContext:previousContext];
            }
        } else {
            // No GL view means InitViewer never created graphics resources.
            _viewer->release();
        }
    }
    _viewer = nullptr;
    NSLog(@"~GLViewController");
}

// =======================================================================
// function : init
// purpose  :
// =======================================================================
- (id) init
{
    self = [super init];

    if (self) {
        _viewer = std::make_shared<Core3DViewer>();
        _assetDataQueue = dispatch_queue_create("com.shapeyard.sync", NULL);
		_rawViewportTouches = [NSMutableSet set];
		_isConstructorMode = false;
        [[NSNotificationCenter defaultCenter]
            addObserver:self
            selector:@selector(documentDidChange:)
            name:@"OcctDocumentChanges"
            object:nil];
        [[NSNotificationCenter defaultCenter]
            addObserver:self
            selector:@selector(applicationWillResignActive:)
            name:UIApplicationWillResignActiveNotification
            object:nil];
    }

    return self;
}

-(std::shared_ptr<core3d::Core3DViewer>) viewer {
    return _viewer;
}

- (GLView *)viewportView
{
    return [self.view isKindOfClass:GLView.class] ? (GLView *)self.view : nil;
}

- (CGFloat)drawableScale
{
    GLView *view = [self viewportView];
    if (view.bounds.size.width > 0.0 && view.drawableSize.width > 0.0) {
        return view.drawableSize.width / view.bounds.size.width;
    }
    return self.view.window.screen.scale ?: UIScreen.mainScreen.scale;
}

- (CGPoint)drawablePointForPoint:(CGPoint)point
{
    const CGFloat scale = [self drawableScale];
    return CGPointMake(lround(point.x * scale), lround(point.y * scale));
}

- (void)requestRender
{
    [[self viewportView] requestRender];
}

- (NSUInteger)renderedFrameCount
{
    return [self viewportView].renderedFrameCount;
}

- (CGSize)drawableSize
{
    return [self viewportView].drawableSize;
}

- (BOOL)isRenderLoopRunning
{
    return [self viewportView].isRenderLoopRunning;
}

- (NSInteger)renderingAPIVersion
{
    switch ([self viewportView].renderingAPI) {
        case kEAGLRenderingAPIOpenGLES3:
            return 3;
        case kEAGLRenderingAPIOpenGLES2:
            return 2;
        default:
            return 0;
    }
}

- (void)documentDidChange:(NSNotification *)notification
{
    if ([notification.object isKindOfClass:NSValue.class]
        && _viewer != nullptr
        && !_viewer->getDocument().IsNull()
        && [(NSValue *)notification.object pointerValue]
            != _viewer->getDocument().get()) {
        return;
    }
    [self requestRender];
}

- (void)applicationWillResignActive:(NSNotification *)notification
{
    (void)notification;
    [self endActiveRenderingInteractions];
}

- (void)viewWillDisappear:(BOOL)animated
{
    [super viewWillDisappear:animated];
    [self endActiveRenderingInteractions];
}

- (void)endActiveRenderingInteractions
{
    GLView *view = [self viewportView];
    const BOOL hadRawPrimaryInteraction = _rawTouchRendering;
    const BOOL hadActiveInteraction =
        _rawTouchRendering || _pinchRendering || _panRendering;
    if (hadActiveInteraction && _viewer != nullptr) {
        _viewer->CancelInteraction(0, 0);
    }
    if (_rawTouchRendering) {
        _rawTouchRendering = NO;
        [view endInteractiveRendering];
    }
    [_rawViewportTouches removeAllObjects];
    if (_pinchRendering) {
        _pinchRendering = NO;
        [view endInteractiveRendering];
    }
    if (_panRendering) {
        _panRendering = NO;
        [view endInteractiveRendering];
    }
    if (hadActiveInteraction) {
        [self requestRender];
    }
    if (hadRawPrimaryInteraction
        && _delegate
        && [_delegate respondsToSelector:
            @selector(viewer:didEndPrimaryInteractionCancelled:)]) {
        [_delegate viewer:self didEndPrimaryInteractionCancelled:YES];
    }
    _rawTouchWasCancelled = NO;
}
// =======================================================================
// function : Draw
// purpose  :
// =======================================================================
- (BOOL) Draw
{
    if (_didSetupViewer && _viewer != nullptr) {
        return _viewer->RenderFrame();
    }
    return NO;
}

// =======================================================================
// function : Setup
// purpose  :
// =======================================================================
- (void) Setup {
    if (_didSetupViewer) {
        _viewer->Resize();
        [self requestRender];
        return;
    }

    if (!_viewer->InitViewer(self.view)) {
        NSLog(@"Failed to init viewer");
        return;
    }

    _didSetupViewer = YES;
    _viewer->showGrid(!_isPreviewMode);
    if (_isPreviewMode) {
        _viewer->setPreviewMode();
    }
    //    [self importScrew:nullptr];
    if (_delegate && [_delegate respondsToSelector:@selector(didSetupViewer:)]) {
        __weak typeof(self) weakSelf = self;
        [_delegate didSetupViewer:weakSelf];
    }
    [self requestRender];
}

// =======================================================================
// function : loadView
// purpose  :
// =======================================================================
- (void) loadView
{
    GLView* aGLView = [[GLView alloc] init];
    aGLView->myController = self;
    self.view = aGLView;
}

- (void) setConstructorMode
{
	_isConstructorMode = true;
}

// =======================================================================
// function : touchesBegan
// purpose  :
// =======================================================================
- (void)touchesBegan:(NSSet<UITouch *> *)theTouches withEvent:(UIEvent *)theEvent
{
    [super touchesBegan:theTouches withEvent:theEvent];

    if (_isPreviewMode) {
        return;
    }

	const BOOL didBeginPrimaryInteraction = _rawViewportTouches.count == 0;
	[_rawViewportTouches unionSet:theTouches];
	if (didBeginPrimaryInteraction) {
		_cancelTouches = NO;
		_rawTouchWasCancelled = NO;
		_rawTouchRendering = YES;
		_suppressTapSelectionForManipulatorInteraction = NO;
		[[self viewportView] beginInteractiveRendering];
	}

    UITouch *aTouch = [theTouches anyObject];
    if (aTouch != NULL) {
        const CGPoint point = [self drawablePointForPoint:[aTouch locationInView:self.view]];
        if (didBeginPrimaryInteraction
            && _delegate
            && [_delegate respondsToSelector:
                @selector(viewer:willBeginPrimaryInteractionAtDrawablePoint:drawableSize:)]) {
            [_delegate viewer:self
                willBeginPrimaryInteractionAtDrawablePoint:point
                                             drawableSize:self.drawableSize];
        }
        _tapRecognizer.cancelsTouchesInView = YES;
        _viewer->StartRotation((int)point.x, (int)point.y);
        const std::shared_ptr<ObjectInteractor> anInteractor =
            _viewer->getObjectInteractor();
        _suppressTapSelectionForManipulatorInteraction =
            anInteractor != nullptr
            && anInteractor->isManipulatorInteractionActive();
        if (_suppressTapSelectionForManipulatorInteraction) {
            // The raw touch-up resolves both transforms and operation handles
            // such as mirror planes. Keep the recognizer from cancelling that
            // lifecycle; tapHandler still suppresses ordinary selection.
            _tapRecognizer.cancelsTouchesInView = NO;
        }
        [self requestRender];
    }
}

- (void)endRawPrimaryInteractionIfNeededCancelled:(BOOL)cancelled {
    if (!_rawTouchRendering || _rawViewportTouches.count != 0) {
        return;
    }
    _rawTouchRendering = NO;
    [[self viewportView] endInteractiveRendering];
    const BOOL resolvedAsCancelled = cancelled || _rawTouchWasCancelled;
    _rawTouchWasCancelled = NO;
    if (_delegate
        && [_delegate respondsToSelector:
            @selector(viewer:didEndPrimaryInteractionCancelled:)]) {
        [_delegate viewer:self
            didEndPrimaryInteractionCancelled:resolvedAsCancelled];
    }
}

// =======================================================================
// function : touchesMoved
// purpose  :
// =======================================================================
- (void)touchesMoved:(NSSet *)theTouches withEvent:(UIEvent *)theEvent
{
	if (_isPreviewMode) {
		return;
	}
    if(_cancelTouches) {
        [self touchesCancelled:theTouches withEvent:theEvent];
        return;
    }

    [super touchesMoved:theTouches withEvent:theEvent];
    
    UITouch *aTouch = [theTouches anyObject];
    if ((aTouch != NULL) && theEvent.allTouches.count == 1) {
        const CGPoint point = [self drawablePointForPoint:[aTouch locationInView:self.view]];
        _viewer->Rotation((int)point.x, (int)point.y);
        [self requestRender];

#ifdef DEBUG
        if (_delegate && [_delegate respondsToSelector:@selector(didChangeStatusString:)]) {
            [_delegate didChangeStatusString:[self statusString]];
        }
#endif
    }

    return;
}

-(void) cancellTouchEvents {
    _cancelTouches = YES;
}

-(void) touchesEnded:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event {
    [super touchesEnded:touches withEvent:event];

    UITouch *aTouch = [touches anyObject];
    if (!_isPreviewMode && aTouch != NULL) {
        const CGPoint point = [self drawablePointForPoint:[aTouch locationInView:self.view]];
        _viewer->FinishInteraction((int)point.x, (int)point.y);
        [self requestRender];

#ifdef DEBUG
        if (_delegate && [_delegate respondsToSelector:@selector(didChangeStatusString:)]) {
            [_delegate didChangeStatusString:[self statusString]];
        }
#endif
	}
	[_rawViewportTouches minusSet:touches];
	[self endRawPrimaryInteractionIfNeededCancelled:NO];

    return;
}

-(void) touchesCancelled:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event{
    [super touchesCancelled:touches withEvent:event];
	_rawTouchWasCancelled = YES;

    UITouch *aTouch = [touches anyObject];
    if (!_isPreviewMode && aTouch != NULL) {
        const CGPoint point = [self drawablePointForPoint:[aTouch locationInView:self.view]];
        _viewer->CancelInteraction((int)point.x, (int)point.y);
        [self requestRender];

#ifdef DEBUG
        if (_delegate && [_delegate respondsToSelector:@selector(didChangeStatusString:)]) {
            [_delegate didChangeStatusString:[self statusString]];
        }
#endif
	}
	[_rawViewportTouches minusSet:touches];
	[self endRawPrimaryInteractionIfNeededCancelled:YES];

    return;
}

// =======================================================================
// function : viewDidLoad
// purpose  :
// =======================================================================
-(void)viewDidLoad
{
    [super viewDidLoad];

    // add zoom recognizer
    UIPinchGestureRecognizer *aZoomRecognizer = [[UIPinchGestureRecognizer alloc]
                                                 initWithTarget:self
                                                 action:@selector(zoomHandler:)];
    aZoomRecognizer.delegate = self;

    [[self view] addGestureRecognizer:aZoomRecognizer];

    // add pan recognizer
    UIPanGestureRecognizer *aPanRecognizer = [[UIPanGestureRecognizer alloc]
                                              initWithTarget:self
                                              action:@selector(panHandler:)];

    aPanRecognizer.maximumNumberOfTouches = 2;
    aPanRecognizer.minimumNumberOfTouches = 2;
    aPanRecognizer.delegate = self;

    [[self view] addGestureRecognizer:aPanRecognizer];

    _tapRecognizer = [[UITapGestureRecognizer alloc]
                      initWithTarget:self
                      action:@selector(tapHandler:)];
    [[self view] addGestureRecognizer:_tapRecognizer];


    // add import buttons
    UIBarButtonItem *importScrewBtn = [[UIBarButtonItem alloc]
                                       initWithTitle:@"Sample 1"
                                       style:UIBarButtonItemStylePlain
                                       target:self
                                       action:@selector(importScrew:)];

    UIBarButtonItem *importLinkrodsBtn = [[UIBarButtonItem alloc]
                                          initWithTitle:@"Sample 2"
                                          style:UIBarButtonItemStylePlain
                                          target:self
                                          action:@selector(importLinkrods:)];


    UIBarButtonItem *addCubeBtn = [[UIBarButtonItem alloc]
                                   initWithTitle:@"Add cube"
                                   style:UIBarButtonItemStylePlain
                                   target:self
                                   action:@selector(addTestPrimitives:)];

    UIBarButtonItem *displayAboutDlgBtn = [[UIBarButtonItem alloc]
                                           initWithTitle:@"About"
                                           style:UIBarButtonItemStylePlain
                                           target:self
                                           action:@selector(displayAboutDlg:)];

    [self.navigationItem setLeftBarButtonItems:[NSArray arrayWithObjects:importScrewBtn, importLinkrodsBtn, addCubeBtn, nil]];
    [self.navigationItem setRightBarButtonItem: displayAboutDlgBtn];
}

- (BOOL)gestureRecognizer:(UIGestureRecognizer *)gestureRecognizer
        shouldRecognizeSimultaneouslyWithGestureRecognizer:(UIGestureRecognizer *)otherGestureRecognizer
{
    const BOOL isPinchPanPair =
        ([gestureRecognizer isKindOfClass:UIPinchGestureRecognizer.class]
         && [otherGestureRecognizer isKindOfClass:UIPanGestureRecognizer.class])
        || ([gestureRecognizer isKindOfClass:UIPanGestureRecognizer.class]
            && [otherGestureRecognizer isKindOfClass:UIPinchGestureRecognizer.class]);
    return isPinchPanPair;
}

// =======================================================================
// function : zoomHandler
// purpose  :
// =======================================================================
- (void)zoomHandler:(UIPinchGestureRecognizer *)pinchRecognizer
{
    const UIGestureRecognizerState state = pinchRecognizer.state;
    if (state == UIGestureRecognizerStateEnded
        || state == UIGestureRecognizerStateCancelled
        || state == UIGestureRecognizerStateFailed) {
        if (_pinchRendering) {
            _pinchRendering = NO;
            [[self viewportView] endInteractiveRendering];
        }
        return;
    }

    if (_isPreviewMode) {
        return;
    }

    if (pinchRecognizer.numberOfTouches > 1) {
        if (state == UIGestureRecognizerStateBegan) {
            if (!_pinchRendering) {
                _pinchRendering = YES;
                [[self viewportView] beginInteractiveRendering];
            }
            _pinchPreviousTouch[0] = [pinchRecognizer locationOfTouch:0 inView:self.view];
            _pinchPreviousTouch[1] = [pinchRecognizer locationOfTouch:1 inView:self.view];
        } else if (state == UIGestureRecognizerStateChanged) {
            CGPoint aLastTouch[2] = {
                [pinchRecognizer locationOfTouch:0 inView:self.view],
                [pinchRecognizer locationOfTouch:1 inView:self.view]
            };

            const CGFloat drawableScale = [self drawableScale];
            double aPinchCenterXStart =
                (_pinchPreviousTouch[0].x + _pinchPreviousTouch[1].x) * drawableScale / 2.0;
            double aPinchCenterYStart =
                (_pinchPreviousTouch[0].y + _pinchPreviousTouch[1].y) * drawableScale / 2.0;

            double aStartDist = Sqrt( ( _pinchPreviousTouch[0].x - _pinchPreviousTouch[1].x ) * ( _pinchPreviousTouch[0].x - _pinchPreviousTouch[1].x ) +
                                     ( _pinchPreviousTouch[0].y - _pinchPreviousTouch[1].y ) * ( _pinchPreviousTouch[0].y - _pinchPreviousTouch[1].y ) );
            double anEndDist = Sqrt( ( aLastTouch[0].x - aLastTouch[1].x ) * ( aLastTouch[0].x - aLastTouch[1].x ) +
                                    ( aLastTouch[0].y - aLastTouch[1].y ) * ( aLastTouch[0].y - aLastTouch[1].y ) );

            double aDeltaDist = (anEndDist - aStartDist) * drawableScale;

            _viewer->Zoom(aPinchCenterXStart, aPinchCenterYStart, aDeltaDist);
            [self requestRender];

            _pinchPreviousTouch[0] = aLastTouch[0];
            _pinchPreviousTouch[1] = aLastTouch[1];
        }
    }
}

// =======================================================================
// function : panHandler
// purpose  :
// =======================================================================
- (void)panHandler:(UIPanGestureRecognizer *)panRecognizer
{
    const UIGestureRecognizerState state = panRecognizer.state;
    if (state == UIGestureRecognizerStateEnded
        || state == UIGestureRecognizerStateCancelled
        || state == UIGestureRecognizerStateFailed) {
        if (_panRendering) {
            _panRendering = NO;
            [[self viewportView] endInteractiveRendering];
        }
        return;
    }

    if (_isPreviewMode) {
        return;
    }

    if (panRecognizer.numberOfTouches > 1) {
        if (state == UIGestureRecognizerStateBegan) {
            if (!_panRendering) {
                _panRendering = YES;
                [[self viewportView] beginInteractiveRendering];
            }
            [panRecognizer setTranslation:CGPointZero inView:self.view];

        } else if (state == UIGestureRecognizerStateChanged) {
            const CGFloat drawableScale = [self drawableScale];
            const CGPoint translation = [panRecognizer translationInView:self.view];
            [panRecognizer setTranslation:CGPointZero inView:self.view];
            const int deltaX = (int)lround(translation.x * drawableScale);
            const int deltaY = (int)lround(-translation.y * drawableScale);
            if (deltaX != 0 || deltaY != 0) {
                // Starting a fresh relative pan from the current camera for each
                // incremental delta keeps panning composable with simultaneous
                // pinch updates instead of repeatedly restoring an old camera.
                _viewer->Pan(deltaX, deltaY, Standard_True);
                [self requestRender];
            }
        }
    }
}

// =======================================================================
// function : tapHandler
// purpose  :
// =======================================================================
- (void)tapHandler:(UITapGestureRecognizer *)tapRecognizer
{
    if (_suppressTapSelectionForManipulatorInteraction) {
        _suppressTapSelectionForManipulatorInteraction = NO;
        return;
    }
    if (_isPreviewMode) {
        return;
    }

    const CGPoint aTapPoint =
        [self drawablePointForPoint:[tapRecognizer locationInView:self.view]];
	if (!_isConstructorMode) {
		if (_delegate
			&& [_delegate respondsToSelector:
				@selector(viewer:willSelectAtDrawablePoint:drawableSize:)]) {
			[_delegate viewer:self
				willSelectAtDrawablePoint:aTapPoint
				             drawableSize:self.drawableSize];
		}
		_viewer->Select((int)aTapPoint.x, (int)aTapPoint.y);
	}

	[self checkSelections];
	[self requestRender];
}

-(void) checkSelections {
	bool isSelected = _viewer->getObjectInteractor()->isSelected();
	bool isManipulatorAttached = _viewer->getObjectInteractor()->isManipulatorAttached();
	unsigned char selections = Core3DViewer::kSelectionTypeNone;
	if (isSelected) {
		selections |= Core3DViewer::kSelectionTypeObject;
	}
	if (isManipulatorAttached) {
		selections |= Core3DViewer::kSelectionTypeManipulator;
	}
	
	if (_delegate && [_delegate respondsToSelector:@selector(viewer:didChangeSelections:)]) {
		__weak typeof(self) weakSelf = self;
		[_delegate viewer:weakSelf didChangeSelections:selections];
	}
}

// =======================================================================
// function : importScrew
// purpose  :
// =======================================================================
- (void)importScrew:(UIBarButtonItem *)theSender
{
    NSString* aNsPath = [[NSBundle mainBundle] pathForResource:@"screw"
                                                        ofType:@"step"];
    std::string aPath = std::string([aNsPath UTF8String]);

    _viewer->ImportSTEP(aPath);
    _viewer->FitAll();
    [self requestRender];
}

// =======================================================================
// function : importLinkrods
// purpose  :
// =======================================================================
- (void)importLinkrods:(UIBarButtonItem *)theSender
{
    NSString* aNsPath = [[NSBundle mainBundle] pathForResource:@"linkrods"
                                                        ofType:@"step"];
    std::string aPath = std::string([aNsPath UTF8String]);

    _viewer->ImportSTEP(aPath);
    _viewer->FitAll();
    [self requestRender];
}

- (void)addCube:(UIBarButtonItem *)theSender {
    __auto_type stlFilename = _viewer->addTestPrimitives();
    _viewer->FitAll();
    [self requestRender];
    [self shareFile:stlFilename];
}

- (void)shareFile:(NSString *)filepath {
    __auto_type activityController = [[UIActivityViewController alloc] initWithActivityItems:@[[NSURL fileURLWithPath:filepath]]
                                                                       applicationActivities:nil];
    if ([UIDevice currentDevice].userInterfaceIdiom == UIUserInterfaceIdiomPad) {
        activityController.popoverPresentationController.sourceView = self.view;
        activityController.popoverPresentationController.permittedArrowDirections = UIPopoverArrowDirectionAny;
        activityController.popoverPresentationController.sourceRect = CGRectMake(self.view.frame.size.width/2.0,
                                                                                 self.view.frame.size.height/2.0, 1.0, 1.0);
    }
    [self presentViewController:activityController animated:YES completion:nil];
}

- (void)addTestPrimitives {
    _viewer->addTestPrimitives();
    _viewer->FitAll();
    [self requestRender];
}

- (void)addPrimitivesFromJSON:(NSString *)json {
    _viewer->addPrimitivesFromJSON(json);
    [self requestRender];
}

- (void)addPrimitive:(PrimitiveType)primitiveType {
    _viewer->addPrimitive(primitiveType);
    [self requestRender];
}

- (void)selectLastObject {
    _viewer->getObjectInteractor()->selectLastObject();
    [self requestRender];
}

- (void)deleteSelected {
    _viewer->getObjectInteractor()->deleteSelected();
    [self requestRender];
}

-(NSString*) statusString {
	auto interactor = _viewer->getObjectInteractor();
	auto transform = interactor->manipulatorTransform();
	if (transform.IsNull()) {
		return @"No active selection";
	}
	auto pos = interactor->manipulatorPosition();
	auto rot = transform->Trsf().GetRotation();
	Standard_Real rotX, rotY, rotZ;
	rot.GetEulerAngles(gp_YawPitchRoll, rotX, rotY, rotZ);
	NSString* status = [NSString stringWithFormat:@"Coord: x%.3f, y%.3f, z%.3f. Angle:x%.3f, y%.3f, z%.3f",
			pos.X(), pos.Y(), pos.Z(), rotX / M_PI * 180, rotY / M_PI * 180, rotZ / M_PI * 180];
	return status;
}

- (void)selectAll {
	_viewer->getObjectInteractor()->selectAll();
	[self checkSelections];
	[self requestRender];
}

- (void)duplicateSelected {
    _viewer->getObjectInteractor()->duplicateSelected();
    [self requestRender];
}

- (void)deselectAll {
    _viewer->deselectAll();
    [self requestRender];
}

- (BOOL)isSelected {
    return _viewer->getObjectInteractor()->isSelected();
}

- (void)fitAll {
    _viewer->FitAll();
    [self requestRender];
}

- (void)setPreviewMode {
    [self endActiveRenderingInteractions];
    _isPreviewMode = YES;
}

- (void)undo {
	if (!_viewer->getDocument()->canUndo()) { return; }
	PrimitiveGizmoType currentType = [self getGizmoType];
	if (currentType == PrimitiveGizmoTypeSubtract) {
		_viewer->getObjectInteractor()->cancelBoolean(BooleanAction::BooleanSubtract);
	} else if (currentType == PrimitiveGizmoTypeUnion) {
		_viewer->getObjectInteractor()->cancelBoolean(BooleanAction::BooleanUnion);
	} else if (currentType == PrimitiveGizmoTypeMirror) {
		_viewer->getObjectInteractor()->clearTrialMirrorObjects();
	}
	_viewer->getObjectInteractor()->detachManipulator(false);
	if (_viewer->getDocument()->undo()) {
		_viewer->redrawDocument();
		[self requestRender];
	}
}

- (void)redo {
	if (!_viewer->getDocument()->canRedo()) { return; }
	PrimitiveGizmoType currentType = [self getGizmoType];
	if (currentType == PrimitiveGizmoTypeSubtract) {
		_viewer->getObjectInteractor()->cancelBoolean(BooleanAction::BooleanSubtract);
	} else if (currentType == PrimitiveGizmoTypeUnion) {
		_viewer->getObjectInteractor()->cancelBoolean(BooleanAction::BooleanUnion);
	} else if (currentType == PrimitiveGizmoTypeMirror) {
		_viewer->getObjectInteractor()->clearTrialMirrorObjects();
	}
	_viewer->getObjectInteractor()->detachManipulator(false);
	if (_viewer->getDocument()->redo()) {
		_viewer->redrawDocument();
		[self requestRender];
	}
}

- (void)setSelectionType:(PrimitiveSelectionType)type {
	_viewer->getObjectInteractor()->cancelInteraction();
	PrimitiveGizmoType currentType = [self getGizmoType];
	if (currentType == PrimitiveGizmoTypeChamfer) {
		_viewer->getShapeInteractor()->resetWireframeTemplateShape();
	} else if (currentType == PrimitiveGizmoTypeSubtract) {
		_viewer->getObjectInteractor()->cancelBoolean(BooleanAction::BooleanSubtract);
	} else if (currentType == PrimitiveGizmoTypeUnion) {
		_viewer->getObjectInteractor()->cancelBoolean(BooleanAction::BooleanUnion);
	} else if (currentType == PrimitiveGizmoTypeMirror) {
		_viewer->getObjectInteractor()->clearTrialMirrorObjects();
	}

    ShapeSelectionMode selectionMode;
    switch (type) {
        case PrimitiveSelectionTypeShape:
            selectionMode = ShapeSelectionMode::WholeShape;
            break;
        case PrimitiveSelectionTypeEdge:
            selectionMode = ShapeSelectionMode::Edge;
            break;
        case PrimitiveSelectionTypeFace:
            selectionMode = ShapeSelectionMode::Face;
            break;
        case PrimitiveSelectionTypeVertex:
            selectionMode = ShapeSelectionMode::Vertex;
            break;
        default:
            assert(false);
            break;
    }
    _viewer->getShapeInteractor()->setSelectionMode(selectionMode);
    [self requestRender];
}

- (PrimitiveSelectionType)getSelectionType {
    PrimitiveSelectionType type = PrimitiveSelectionTypeNone;
    switch (_viewer->getShapeInteractor()->getSelectionMode()) {
        case ShapeSelectionMode::WholeShape:
            type = PrimitiveSelectionTypeShape;
            break;
        case ShapeSelectionMode::Edge:
            type = PrimitiveSelectionTypeEdge;
            break;
        case ShapeSelectionMode::Face:
            type = PrimitiveSelectionTypeFace;
            break;
        case ShapeSelectionMode::Vertex:
            type = PrimitiveSelectionTypeVertex;
            break;
        default:
            assert(false);
            break;
    }
    return type;
}

- (void)setGizmo:(Handle(Core3DManipulator))manipulator {
	_viewer->getObjectInteractor()->setManipulator(manipulator);
	[self requestRender];
}

- (void)setPrimitiveTransparent:(Handle(AIS_InteractiveObject))primitive transparent:(bool)set {
	_viewer->getObjectInteractor()->setObjectTransparent(primitive, set);
	[self requestRender];
}

- (void)setGizmoType:(PrimitiveGizmoType)type {
	const PrimitiveGizmoType previousType = [self getGizmoType];
	if (previousType == type) { return; }

	// Resolve the previous tool before changing manipulator mode or capturing
	// selection for the next tool. Its AIS previews may refer to document labels
	// that the resolution step replaces or removes.
	if (previousType == PrimitiveGizmoTypeChamfer) {
		_viewer->getShapeInteractor()->resetWireframeTemplateShape();
	} else if (previousType == PrimitiveGizmoTypeSubtract) {
		_viewer->getObjectInteractor()->cancelBoolean(BooleanAction::BooleanSubtract);
	} else if (previousType == PrimitiveGizmoTypeUnion) {
		_viewer->getObjectInteractor()->cancelBoolean(BooleanAction::BooleanUnion);
	} else if (previousType == PrimitiveGizmoTypeMirror) {
		_viewer->getObjectInteractor()->clearTrialMirrorObjects();
	}

    PrimitiveManipulatorType manipulatorType;
    switch (type) {
        case PrimitiveGizmoTypeMoveRotate:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate;
            break;
        case PrimitiveGizmoTypeScale:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeScale;
            break;
        case PrimitiveGizmoTypeChamfer:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer;
            break;
        case PrimitiveGizmoTypeNone:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
            break;
        case PrimitiveGizmoTypeSubtract:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract;
            break;
        case PrimitiveGizmoTypeUnion:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeUnion;
            break;
        case PrimitiveGizmoTypeMirror:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeMirror;
            break;
        case PrimitiveGizmoTypeMaterial:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeMaterial;
            break;
        default:
            assert(false);
            break;
    }
    _viewer->getObjectInteractor()->setManipulatorType(manipulatorType);
    if (type == PrimitiveGizmoTypeChamfer) {
        _viewer->getShapeInteractor()->saveSelectionEdges();
	} else if (type == PrimitiveGizmoTypeSubtract || type == PrimitiveGizmoTypeUnion) {
		Standard_Boolean forceActor = (type == PrimitiveGizmoTypeSubtract);
		_viewer->getObjectInteractor()->fillSelectedState(
			forceActor,
			type == PrimitiveGizmoTypeSubtract
				? BooleanAction::BooleanSubtract
				: BooleanAction::BooleanUnion);
    }
	[self requestRender];
}

- (PrimitiveGizmoType)getGizmoType {
    PrimitiveGizmoType type = PrimitiveGizmoTypeNone;
    switch (_viewer->getObjectInteractor()->getManipulatorType()) {
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate:
            type = PrimitiveGizmoTypeMoveRotate;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeScale:
            type = PrimitiveGizmoTypeScale;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer:
            type = PrimitiveGizmoTypeChamfer;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeNone:
            type = PrimitiveGizmoTypeNone;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract:
            type = PrimitiveGizmoTypeSubtract;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeUnion:
            type = PrimitiveGizmoTypeUnion;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMirror:
            type = PrimitiveGizmoTypeMirror;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMaterial:
            type = PrimitiveGizmoTypeMaterial;
            break;
        default:
            break;
    }
    return type;
}

- (void)setChamfer:(CGFloat)value {
    assert([self getGizmoType] == PrimitiveGizmoTypeChamfer);
    (void)_viewer->getShapeInteractor()->setChamferValueForSelection(value);
    [self requestRender];
    //if (!result)
    //	std::cout << "incorrect chamfer value" << std::endl;
}

- (void)cancelChamfer {
    _viewer->getShapeInteractor()->cancelChamfer();
    [self requestRender];
}

- (void) applyMirror {
    assert([self getGizmoType] == PrimitiveGizmoTypeMirror);
	_viewer->getObjectInteractor()->applyMirror();
	[self requestRender];
}

- (void) cancelMirror {
	_viewer->getObjectInteractor()->clearTrialMirrorObjects();
	[self requestRender];
}

- (void) applySubtract {
    _viewer->getObjectInteractor()->applyBoolean(BooleanAction::BooleanSubtract);
    [self requestRender];
}

- (void) cancelSubtract {
    _viewer->getObjectInteractor()->cancelBoolean(BooleanAction::BooleanSubtract);
    [self requestRender];
}

- (void) applyUnion {
    _viewer->getObjectInteractor()->applyBoolean(BooleanAction::BooleanUnion);
    [self requestRender];
}

- (void) cancelUnion {
    _viewer->getObjectInteractor()->cancelBoolean(BooleanAction::BooleanUnion);
    [self requestRender];
}

- (BOOL) canApplyBoolean {
    return _viewer->getObjectInteractor()->canApplyBoolean();
}

- (BOOL)isEmptyOfDisplayedObjects {
    return _viewer->getShapeInteractor()->isEmptyOfDisplayedObjects();
}

- (NSInteger)numberOfDisplayedShapes {
    return _viewer->getShapeInteractor()->getNumberOfDisplayedShapes();
}

- (NSInteger)numberOfDetectedEdges {
    return _viewer->getShapeInteractor()->getNumberOfDetectedEdges();
}

- (void)assetData:(void(^)(NSData *_Nullable))completion {
    __weak typeof(self) weakSelf = self;
    dispatch_async(_assetDataQueue, ^{
        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf) {
            dispatch_async(dispatch_get_main_queue(), ^{
                completion(nil);
            });
            return;
        }

        NSString *tmpFilename = [NSString stringWithFormat:@"%@.tmp", NSUUID.UUID.UUIDString];
        NSURL *tmpDirectory = [NSFileManager.defaultManager temporaryDirectory];
        NSURL *baseURL = [tmpDirectory URLByAppendingPathComponent:tmpFilename];
        NSString *expectedCbfPath = [baseURL.path stringByAppendingString:@".cbf"];
        const std::string fn = baseURL.path.UTF8String;

        __block std::string cbfFilePath;
        dispatch_sync(dispatch_get_main_queue(), ^{
            try {
                if (strongSelf->_viewer != nullptr) {
                    cbfFilePath = strongSelf->_viewer->getDocument()->save(fn);
                    if (!cbfFilePath.empty()
                        && strongSelf->_viewer->ValidateCbf(cbfFilePath)
                            != AssetImportResult::Success) {
                        cbfFilePath.clear();
                    }
                }
            } catch (...) {
                cbfFilePath.clear();
            }
        });

        NSString *dataPath = cbfFilePath.empty()
            ? nil
            : [NSString stringForStdString:cbfFilePath];
        NSError *error = nil;
        NSData *data = dataPath == nil
            ? nil
            : [NSData dataWithContentsOfFile:dataPath options:kNilOptions error:&error];

        NSFileManager *fileManager = NSFileManager.defaultManager;
        [fileManager removeItemAtURL:baseURL error:nil];
        [fileManager removeItemAtPath:expectedCbfPath error:nil];
        if (dataPath != nil && ![dataPath isEqualToString:expectedCbfPath]) {
            [fileManager removeItemAtPath:dataPath error:nil];
        }

        if (error != nil || !HasCbfMagic(data)) {
            data = nil;
        }

        dispatch_async(dispatch_get_main_queue(), ^{
            completion(data);
        });
    });
}

- (void)setAssetData:(NSData *)data completion:(void(^)(Core3DAssetLoadResult result))completion {
    __weak typeof(self) weakSelf = self;
    dispatch_async(_assetDataQueue, ^{
        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf) {
            CompleteAssetLoadOnMain(completion, Core3DAssetLoadResultInternalFailure);
            return;
        }
        if (!HasCbfMagic(data)) {
            CompleteAssetLoadOnMain(completion, Core3DAssetLoadResultInvalidData);
            return;
        }

        NSString *tmpFilename = [NSString stringWithFormat:@"%@.tmp.cbf", NSUUID.UUID.UUIDString];
        NSURL *tmpUrl = [[NSFileManager.defaultManager temporaryDirectory] URLByAppendingPathComponent:tmpFilename];
        NSError *error = nil;
        [data writeToURL:tmpUrl options:NSDataWritingAtomic error:&error];
        if (error != nil) {
            [NSFileManager.defaultManager removeItemAtURL:tmpUrl error:nil];
            CompleteAssetLoadOnMain(completion, Core3DAssetLoadResultTemporaryFileFailure);
            return;
        }

        const std::string fn = tmpUrl.path.UTF8String;
        __block Core3DAssetLoadResult result = Core3DAssetLoadResultInternalFailure;
        dispatch_sync(dispatch_get_main_queue(), ^{
            try {
                if (strongSelf->_viewer != nullptr) {
                    result = AssetLoadResultFromImportResult(strongSelf->_viewer->ImportCbf(fn));
                    if (result == Core3DAssetLoadResultSuccess) {
                        [strongSelf requestRender];
                    }
                }
            } catch (...) {
                result = Core3DAssetLoadResultInternalFailure;
            }
        });
        [NSFileManager.defaultManager removeItemAtURL:tmpUrl error:nil];
        CompleteAssetLoadOnMain(completion, result);
    });
}

- (NSData *)thumbData {
    NSString *tmpSnapthotFilename = [NSString stringWithFormat:@"%@.tmp.png", NSUUID.UUID.UUIDString];
    NSURL *tmpUrl = [[NSFileManager.defaultManager temporaryDirectory] URLByAppendingPathComponent:tmpSnapthotFilename];
    NSLog(@"Snapshot: %@", tmpUrl);
    if (![self saveSnapshot:tmpUrl]) {
        [NSFileManager.defaultManager removeItemAtURL:tmpUrl error:nil];
        return NULL;
    }
    NSError *error = nil;
    __auto_type data = [NSData dataWithContentsOfURL:tmpUrl options:kNilOptions error:&error];

    [NSFileManager.defaultManager removeItemAtURL:tmpUrl error:nil];

    if (error) {
        NSLog(@"ERROR: with thumb: %@", error.localizedDescription);
        return NULL;
    }

    return data;
}

- (BOOL)saveSnapshot:(NSURL *)tmpUrl {
    const auto fn = TCollection_AsciiString(tmpUrl.path.UTF8String);
    return _viewer->dumpOfDisplayedColoredObjects(500, 500, tmpUrl.path.UTF8String);
}

- (NSURL *_Nullable)exportWithType:(ExportType)exportType {
    NSString *pathExtension = NULL;
    switch (exportType) {
        case ExportTypeObj:
            pathExtension = @"obj";
            break;
        case ExportTypeStl:
            pathExtension = @"stl";
            break;
        case ExportTypeGltf:
            pathExtension = @"glb";
            break;
        case ExportTypeStep:
            pathExtension = @"step";
            break;
        default:
            assert(false);
            break;
    }

    NSString *exportFilename = [[NSString stringWithFormat:@"%u", (int)NSDate.now.timeIntervalSince1970] stringByAppendingPathExtension:pathExtension];
    NSURL *exportUrl = [[NSFileManager.defaultManager temporaryDirectory] URLByAppendingPathComponent:exportFilename];

    const auto exportPath = std::string(exportUrl.path.UTF8String);

    switch (exportType) {
        case ExportTypeObj:
            _viewer->getShapeInteractor()->exportToObj(exportPath);
            break;
        case ExportTypeStl:
            _viewer->getShapeInteractor()->exportToStl(exportPath);
            break;
        case ExportTypeGltf:
            _viewer->getShapeInteractor()->exportToGltf(exportPath);
            break;
        case ExportTypeStep:
            _viewer->getShapeInteractor()->exportToStep(exportPath);
            break;
        default:
            assert(false);
            break;
    }


    // Verify the file was actually created
    if (![NSFileManager.defaultManager fileExistsAtPath:exportUrl.path]) {
        NSLog(@"[Export] File was NOT created at: %@", exportUrl.path);
        return nil;
    }
    
    NSDictionary *attrs = [NSFileManager.defaultManager attributesOfItemAtPath:exportUrl.path error:nil];
    NSLog(@"[Export] File created: %@ (%llu bytes)", exportUrl.lastPathComponent, [attrs fileSize]);
    
    if ([attrs fileSize] == 0) {
        NSLog(@"[Export] File is empty, returning nil");
        return nil;
    }
    
    return exportUrl;
}

- (BOOL)isPreviewMode {
    return _isPreviewMode;
}

- (void)setOrthoProjection:(OrthoProjectionType)orthoType {
    _viewer->setOrthoProjection(orthoType);
    [self requestRender];
}

// =======================================================================
// function : displayAboutDlg
// purpose  :
// =======================================================================
- (void)displayAboutDlg:(UIBarButtonItem *)theSender
{
  UIAlertController* anAbout = [UIAlertController alertControllerWithTitle:@"About"
                                message:@"UIKit based application for tutorial to Open CASCADE Technology.\n\n"
                                      @"Copyright (c) 2017 OPEN CASCADE SAS"
                                preferredStyle:UIAlertControllerStyleAlert];
  
  UIAlertAction* aDefaultAction = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault
                                                         handler:^(UIAlertAction * action) {}];
  
  [anAbout addAction:aDefaultAction];
  [self presentViewController:anAbout animated:YES completion:nil];
}

@end
