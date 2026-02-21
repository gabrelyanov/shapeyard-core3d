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

using namespace core3d;

@implementation GLViewController {
    CGFloat _ns; // native screen scale
    dispatch_queue_t _assetDataQueue;
    
    BOOL _cancelTouches;
}

- (void)dealloc {
    _viewer->release();
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
        _ns = [[UIScreen mainScreen] scale];
        _assetDataQueue = dispatch_queue_create("com.shapeyard.sync", NULL);
		_isConstructorMode = false;
    }

    return self;
}

-(std::shared_ptr<core3d::Core3DViewer>) viewer {
    return _viewer;
}
// =======================================================================
// function : Draw
// purpose  :
// =======================================================================
- (void) Draw
{
}

// =======================================================================
// function : Setup
// purpose  :
// =======================================================================
- (void) Setup {
    if (!_viewer->InitViewer(self.view)) {
        NSLog(@"Failed to init viewer");
    }
    else {
        _viewer->showGrid(!_isPreviewMode);
        if (_isPreviewMode) {
            _viewer->setPreviewMode();
        }
        //    [self importScrew:nullptr];
        if (_delegate && [_delegate respondsToSelector:@selector(didSetupViewer:)]) {
            __weak typeof(self) weakSelf = self;
            [_delegate didSetupViewer:weakSelf];
        }
    }
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
- (void)touchesBegan:(NSSet *)theTouches withEvent:(UIEvent *)theEvent
{
    [super touchesBegan:theTouches withEvent:theEvent];

    _cancelTouches = NO;

    UITouch *aTouch = [theTouches anyObject];
    if (aTouch != NULL) {
        CGPoint aTouchPoint = [aTouch locationInView:self.view];
        _viewer->StartRotation((int)aTouchPoint.x * _ns, (int)aTouchPoint.y * _ns);
    }
}

// =======================================================================
// function : touchesMoved
// purpose  :
// =======================================================================
- (void)touchesMoved:(NSSet *)theTouches withEvent:(UIEvent *)theEvent
{
    if(_cancelTouches) {
        [self touchesCancelled:theTouches withEvent:theEvent];
        return;
    }
    
    [super touchesMoved:theTouches withEvent:theEvent];
    
    UITouch *aTouch = [theTouches anyObject];
    if ((aTouch != NULL) && theEvent.allTouches.count == 1) {
        CGPoint aTouchPoint = [aTouch locationInView:self.view];
        _viewer->Rotation((int)aTouchPoint.x * _ns, (int)aTouchPoint.y * _ns);

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
    if (aTouch != NULL) {
        CGPoint aTouchPoint = [aTouch locationInView:self.view];
        _viewer->FinishInteraction((int)aTouchPoint.x * _ns, (int)aTouchPoint.y * _ns);

#ifdef DEBUG
        if (_delegate && [_delegate respondsToSelector:@selector(didChangeStatusString:)]) {
            [_delegate didChangeStatusString:[self statusString]];
        }
#endif
    }

    return;
}

-(void) touchesCancelled:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event{
    [super touchesCancelled:touches withEvent:event];

    UITouch *aTouch = [touches anyObject];
    if (aTouch != NULL) {
        CGPoint aTouchPoint = [aTouch locationInView:self.view];
        _viewer->CancelInteraction((int)aTouchPoint.x * _ns, (int)aTouchPoint.y * _ns );

#ifdef DEBUG
        if (_delegate && [_delegate respondsToSelector:@selector(didChangeStatusString:)]) {
            [_delegate didChangeStatusString:[self statusString]];
        }
#endif
    }

    return;
}

// =======================================================================
// function : viewDidLoad
// purpose  :
// =======================================================================
-(void)viewDidLoad
{
    // add zoom recognizer
    UIPinchGestureRecognizer *aZoomRecognizer = [[UIPinchGestureRecognizer alloc]
                                                 initWithTarget:self
                                                 action:@selector(zoomHandler:)];

    [[self view] addGestureRecognizer:aZoomRecognizer];

    // add pan recognizer
    UIPanGestureRecognizer *aPanRecognizer = [[UIPanGestureRecognizer alloc]
                                              initWithTarget:self
                                              action:@selector(panHandler:)];

    aPanRecognizer.maximumNumberOfTouches = 2;
    aPanRecognizer.minimumNumberOfTouches = 2;

    [[self view] addGestureRecognizer:aPanRecognizer];

    UITapGestureRecognizer *aTapRecognizer = [[UITapGestureRecognizer alloc]
                                              initWithTarget:self
                                              action:@selector(tapHandler:)];

    [[self view] addGestureRecognizer:aTapRecognizer];


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

// =======================================================================
// function : zoomHandler
// purpose  :
// =======================================================================
- (void)zoomHandler:(UIPinchGestureRecognizer *)pinchRecognizer
{
    if (_isPreviewMode) {
        return;
    }

    if ([pinchRecognizer numberOfTouches] > 1)
    {
        UIGestureRecognizerState aState = [pinchRecognizer state];
        if (aState == UIGestureRecognizerStateBegan)
        {
            myFirstTouch[0] = [pinchRecognizer locationOfTouch:0 inView:self.view];
            myFirstTouch[1] = [pinchRecognizer locationOfTouch:1 inView:self.view];
        }
        else if (aState == UIGestureRecognizerStateChanged) {
            CGPoint aLastTouch[2] = {
                [pinchRecognizer locationOfTouch:0 inView:self.view],
                [pinchRecognizer locationOfTouch:1 inView:self.view]
            };

            double aPinchCenterXStart = ( myFirstTouch[0].x + myFirstTouch[1].x ) / 2.0;
            double aPinchCenterYStart = ( myFirstTouch[0].y + myFirstTouch[1].y ) / 2.0;

            double aStartDist = Sqrt( ( myFirstTouch[0].x - myFirstTouch[1].x ) * ( myFirstTouch[0].x - myFirstTouch[1].x ) +
                                     ( myFirstTouch[0].y - myFirstTouch[1].y ) * ( myFirstTouch[0].y - myFirstTouch[1].y ) );
            double anEndDist = Sqrt( ( aLastTouch[0].x - aLastTouch[1].x ) * ( aLastTouch[0].x - aLastTouch[1].x ) +
                                    ( aLastTouch[0].y - aLastTouch[1].y ) * ( aLastTouch[0].y - aLastTouch[1].y ) );

            double aDeltaDist = anEndDist - aStartDist;

            _viewer->Zoom(aPinchCenterXStart, aPinchCenterYStart, aDeltaDist);

            myFirstTouch[0] = aLastTouch[0];
            myFirstTouch[1] = aLastTouch[1];
        }
    }
}

// =======================================================================
// function : panHandler
// purpose  :
// =======================================================================
- (void)panHandler:(UIPanGestureRecognizer *)panRecognizer
{
    if (_isPreviewMode) {
        return;
    }

    if ([panRecognizer numberOfTouches] > 1)
    {
        UIGestureRecognizerState aState = [panRecognizer state];
        if (aState == UIGestureRecognizerStateBegan)
        {
            myFirstTouch[0] = [panRecognizer locationOfTouch:0 inView:self.view];
            myFirstTouch[1] = [panRecognizer locationOfTouch:1 inView:self.view];
            
            _viewer->Pan(0.0, 0.0, Standard_True);

        }
        else if (aState == UIGestureRecognizerStateChanged) {
            CGPoint aLastTouch[2] = {
                [panRecognizer locationOfTouch:0 inView:self.view],
                [panRecognizer locationOfTouch:1 inView:self.view]
            };

            double aPinchCenterXStart = ( myFirstTouch[0].x + myFirstTouch[1].x ) * _ns / 2.0;
            double aPinchCenterYStart = ( myFirstTouch[0].y + myFirstTouch[1].y ) * _ns / 2.0;

            double aPinchCenterXEnd = ( aLastTouch[0].x + aLastTouch[1].x ) * _ns / 2.0;
            double aPinchCenterYEnd = ( aLastTouch[0].y + aLastTouch[1].y ) * _ns / 2.0;

            double aPinchCenterXDev = aPinchCenterXEnd - aPinchCenterXStart;
            double aPinchCenterYDev = aPinchCenterYEnd - aPinchCenterYStart;

            _viewer->Pan((int)aPinchCenterXDev, (int)-aPinchCenterYDev, Standard_False);
        }
    }
}

// =======================================================================
// function : tapHandler
// purpose  :
// =======================================================================
- (void)tapHandler:(UITapGestureRecognizer *)tapRecognizer
{
    if (_isPreviewMode) {
        return;
    }

    CGPoint aTapPoint = [tapRecognizer locationInView:self.view];
	if (!_isConstructorMode)
		_viewer->Select(aTapPoint.x * _ns, aTapPoint.y * _ns);

	[self checkSelections];
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
}

- (void)addCube:(UIBarButtonItem *)theSender {
    __auto_type stlFilename = _viewer->addTestPrimitives();
    _viewer->FitAll();
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
}

- (void)addPrimitive:(PrimitiveType)primitiveType {
    _viewer->addPrimitive(primitiveType);
    _viewer->redraw();
}

- (void)selectLastObject {
    _viewer->getObjectInteractor()->selectLastObject();
}

- (void)deleteSelected {
    _viewer->getObjectInteractor()->deleteSelected();
}

-(NSString*) statusString {
	auto pos = _viewer->getObjectInteractor()->manipulatorPosition();
	auto rot = _viewer->getObjectInteractor()->manipulatorTransform()->Trsf().GetRotation();// .GetRotation();
	Standard_Real rotX, rotY, rotZ;
	rot.GetEulerAngles(gp_YawPitchRoll, rotX, rotY, rotZ);
	NSString* status = [NSString stringWithFormat:@"Coord: x%.3f, y%.3f, z%.3f. Angle:x%.3f, y%.3f, z%.3f",
			pos.X(), pos.Y(), pos.Z(), rotX / M_PI * 180, rotY / M_PI * 180, rotZ / M_PI * 180];
	NSLog(@"%@", status);
	return status;
}

- (void)selectAll {
	_viewer->getObjectInteractor()->selectAll();
	[self checkSelections];
}

- (void)duplicateSelected {
    _viewer->getObjectInteractor()->duplicateSelected();
}

- (void)deselectAll {
    _viewer->deselectAll();
}

- (BOOL)isSelected {
    return _viewer->getObjectInteractor()->isSelected();
}

- (void)fitAll {
    _viewer->FitAll();
}

- (void)setPreviewMode {
    _isPreviewMode = YES;
}

- (void)undo {
	if (_viewer->getDocument()->canUndo() && _viewer->getObjectInteractor() != nullptr)
		_viewer->getObjectInteractor()->detachManipulator(false);
    _viewer->getDocument()->undo();
    _viewer->redrawDocument();
}

- (void)redo {
	if (_viewer->getDocument()->canRedo() && _viewer->getObjectInteractor() != nullptr)
		_viewer->getObjectInteractor()->detachManipulator(false);
    _viewer->getDocument()->redo();
    _viewer->redrawDocument();
}

- (void)setSelectionType:(PrimitiveSelectionType)type {
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
}

- (void)setPrimitiveTransparent:(Handle(AIS_InteractiveObject))primitive transparent:(bool)set {
	_viewer->getObjectInteractor()->setObjectTransparent(primitive, set);
}

- (void)setGizmoType:(PrimitiveGizmoType)type {
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
    } else {
        if (type == PrimitiveGizmoTypeSubtract || type == PrimitiveGizmoTypeUnion) {
            Standard_Boolean forceActor = (type == PrimitiveGizmoTypeSubtract);
            _viewer->getObjectInteractor()->fillSelectedState(forceActor,
                                                                   (type == PrimitiveGizmoTypeSubtract)
                                                                   ? BooleanAction::BooleanSubtract
                                                                   : BooleanAction::BooleanUnion);
            _viewer->redraw();
        }
        _viewer->getShapeInteractor()->resetWireframeTemplateShape();
    }
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
    Standard_Boolean result = _viewer->getShapeInteractor()->setChamferValueForSelection(value);
    _viewer->redraw();
    //if (!result)
    //	std::cout << "incorrect chamfer value" << std::endl;
}

- (void) applyMirror {
    assert([self getGizmoType] == PrimitiveGizmoTypeMirror);
	_viewer->getObjectInteractor()->applyMirror();
}

- (void) cancelMirror {
	_viewer->getObjectInteractor()->clearTrialMirrorObjects();
}

- (void) applySubtract {
    _viewer->getObjectInteractor()->applyBoolean(BooleanAction::BooleanSubtract);
}

- (void) cancelSubtract {
    _viewer->getObjectInteractor()->cancelBoolean(BooleanAction::BooleanSubtract);
}

- (void) applyUnion {
    _viewer->getObjectInteractor()->applyBoolean(BooleanAction::BooleanUnion);
}

- (void) cancelUnion {
    _viewer->getObjectInteractor()->cancelBoolean(BooleanAction::BooleanUnion);
}

- (BOOL) canApplyBoolean {
    return _viewer->getObjectInteractor()->canApplyBoolean();
}

- (BOOL)isEmptyOfDisplayedObjects {
    return _viewer->getShapeInteractor()->isEmptyOfDisplayedObjects();
}

- (NSInteger)numberOfDetectedEdges {
    return _viewer->getShapeInteractor()->getNumberOfDetectedEdges();
}

- (void)assetData:(void(^)(NSData *_Nullable))completion {
    __weak typeof(self) weakSelf = self;
    dispatch_async(_assetDataQueue, ^{
        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf) return;
        NSString *tmpFilename = [NSString stringWithFormat:@"%u.tmp", (int)NSDate.now.timeIntervalSince1970];
        NSURL *tmpDirectory = [NSFileManager.defaultManager temporaryDirectory];
        const std::string fn = [tmpDirectory URLByAppendingPathComponent:tmpFilename].path.UTF8String;
        dispatch_sync(dispatch_get_main_queue(), ^{
            const std::string cbfFilePath = strongSelf->_viewer->getDocument()->save(fn);

            __auto_type dataPath = [NSString stringForStdString:cbfFilePath];

            NSError *error = NULL;
            NSData *data = [NSData dataWithContentsOfFile:dataPath options:kNilOptions error:&error];
//            NSLog(@">>> GET assetData: %ld", data.length);
            if (error) {
//                NSLog(@"ERROR: get asset data: %@", error.localizedDescription);
                completion(NULL);
                return;
            }
//            NSLog(@">>> GET assetData COMPLETION");
            completion(data);
        });
    });
}

- (void)setAssetData:(NSData *)data completion:(void(^)(void))completion {
    __weak typeof(self) weakSelf = self;
    dispatch_async(_assetDataQueue, ^{
        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf) return;
        NSString *tmpFilename = [NSString stringWithFormat:@"%u.tmp.cbf", (int)NSDate.now.timeIntervalSince1970];
        NSURL *tmpUrl = [[NSFileManager.defaultManager temporaryDirectory] URLByAppendingPathComponent:tmpFilename];
        NSError *error = NULL;
        //        NSLog(@">>> SET AssetData: %ld", data.length);
        [data writeToURL:tmpUrl options:NSDataWritingAtomic error:&error];
        const std::string fn = tmpUrl.path.UTF8String;
        //        NSLog(@">>> SET AssetData: %s", fn.c_str());
        dispatch_sync(dispatch_get_main_queue(), ^{
            strongSelf->_viewer->ImportCbf(fn);
            //            NSLog(@">>> SET AssetData: COMPLETION");
            completion();
        });
    });
}

- (NSData *)thumbData {
    NSString *tmpSnapthotFilename = [NSString stringWithFormat:@"%u.tmp.png", (int)NSDate.now.timeIntervalSince1970];
    NSURL *tmpUrl = [[NSFileManager.defaultManager temporaryDirectory] URLByAppendingPathComponent:tmpSnapthotFilename];
    const auto fn = TCollection_AsciiString(tmpUrl.path.UTF8String);
    NSLog(@"Snapshot: %@", tmpUrl);
    if (![self saveSnapshot:tmpUrl]) {
        return NULL;
    }
    NSError *error = NULL;
    __auto_type data = [NSData dataWithContentsOfURL:tmpUrl options:kNilOptions error:&error];

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
        default:
            assert(false);
            break;
    }


    return exportUrl;
}

- (BOOL)isPreviewMode {
    return _isPreviewMode;
}

- (void)setOrthoProjection:(OrthoProjectionType)orthoType {
    _viewer->setOrthoProjection(orthoType);
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
