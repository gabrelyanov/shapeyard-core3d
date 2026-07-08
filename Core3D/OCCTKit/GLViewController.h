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

#ifndef GLViewController_h
#define GLViewController_h

#import <UIKit/UIKit.h>
#import <Core3D/PrimitiveType.h>
#import <Core3D/PrimitiveSelectionType.h>
#import <Core3D/PrimitiveGizmoType.h>
#import <Core3D/ExportType.h>
#import <Core3D/ModelPart.h>

#import "Core3DViewer.h"
#import "GLViewControllerProtocol.h"

#include "OrthoProjectionType.h"

//! OpenGL view controller
@interface GLViewController : UIViewController {
    std::shared_ptr<core3d::Core3DViewer> _viewer;
    CGPoint myFirstTouch[2];
    BOOL _isPreviewMode;
	BOOL _isConstructorMode;
}

@property (nonatomic, weak, nullable) id<GLViewControllerProtocol> delegate;
@property (nonatomic, assign, readonly) BOOL isPreviewMode;

-(std::shared_ptr<core3d::Core3DViewer>) viewer;
-(void) Draw;
-(void) Setup;

- (void)addTestPrimitives;
- (void)addPrimitive:(PrimitiveType)primitiveType;
- (void)addPrimitivesFromJSON:(NSString *)json;

- (void)setSelectionType:(PrimitiveSelectionType)type;
- (PrimitiveSelectionType)getSelectionType;
- (void)setGizmoType:(PrimitiveGizmoType)type;
- (PrimitiveGizmoType)getGizmoType;

- (void)setGizmo:(Handle(Core3DManipulator))manipulator;
- (void)setPrimitiveTransparent:(Handle(AIS_InteractiveObject))primitive transparent:(bool)set;

- (void)selectLastObject;
- (void)selectAll;
- (void)deleteSelected;
- (void)duplicateSelected;
- (void)deselectAll;
- (BOOL)isSelected;

- (void)fitAll;
- (void)setPreviewMode;

- (BOOL)isEmptyOfDisplayedObjects;
- (NSInteger)numberOfDetectedEdges;

- (void)setChamfer:(CGFloat)value;
- (void) applyMirror;
- (void) cancelMirror;
- (void) applySubtract;
- (void) cancelSubtract;
- (void) applyUnion;
- (void) cancelUnion;
- (BOOL) canApplyBoolean;

- (void) setConstructorMode;

- (void)setOrthoProjection:(OrthoProjectionType)orthoType;

- (void)undo;
- (void)redo;

- (void)assetData:(void(^)(NSData *_Nullable))completion;
- (void)setAssetData:(NSData *_Nonnull)data completion:(void(^)(void))completion;
- (NSData *_Nullable)thumbData;
- (BOOL)saveSnapshot;
- (NSURL *_Nullable)exportWithType:(ExportType)exportType;
- (void) cancellTouchEvents;

-(NSString*) statusString;

@end

#endif // GLViewController_h
