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
#import "../UI/Core3DViewController.h"

NS_ASSUME_NONNULL_BEGIN

//! OpenGL view controller
@interface GLViewController : UIViewController {
    std::shared_ptr<core3d::Core3DViewer> _viewer;
    BOOL _isPreviewMode;
	BOOL _isConstructorMode;
}

@property (nonatomic, weak, nullable) id<GLViewControllerProtocol> delegate;
@property (nonatomic, assign, readonly) BOOL isPreviewMode;
@property (nonatomic, assign, readonly) NSUInteger renderedFrameCount;
@property (nonatomic, assign, readonly) CGSize drawableSize;
@property (nonatomic, assign, readonly, getter=isRenderLoopRunning) BOOL renderLoopRunning;
@property (nonatomic, assign, readonly) NSInteger renderingAPIVersion;

-(std::shared_ptr<core3d::Core3DViewer>) viewer;
-(BOOL) Draw;
-(void) Setup;
- (void)requestRender;

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
- (NSInteger)numberOfDisplayedShapes;
- (NSInteger)numberOfDetectedEdges;

- (void)setChamfer:(CGFloat)value;
- (BOOL)applyChamfer;
- (BOOL)cancelChamfer;
- (BOOL)canApplyChamfer;
- (BOOL)hasActiveBevel;
- (BOOL)setExtrusion:(CGFloat)value;
- (BOOL)applyExtrusion;
- (BOOL)cancelExtrusion;
- (BOOL)canApplyExtrusion;
- (BOOL) applyMirror;
- (BOOL) cancelMirror;
- (BOOL) applySubtract;
- (BOOL) cancelSubtract;
- (BOOL) applyUnion;
- (BOOL) cancelUnion;
- (BOOL) applyIntersect;
- (BOOL) cancelIntersect;
- (BOOL) canApplyBoolean;
- (BOOL) hasActiveBoolean;
- (BOOL) hasTrialMirrorObjects;
#ifdef DEBUG
- (void)debugRequestRender;
- (void)debugSetMaximumDisplayTraversalNodes:(NSUInteger)limit;
- (void)debugSetMaximumLeafPresentations:(NSUInteger)limit;
- (void)debugSetMaximumProjectTopologyValidationNodes:(NSUInteger)limit;
- (void)debugResetProjectTopologyValidationCounters;
- (NSUInteger)debugBoundedProjectTopologyValidationCount;
- (NSUInteger)debugGeometricBRepValidationCount;
- (NSInteger)debugSelectedShapeCount;
- (NSDictionary<NSString *, NSNumber *> *)debugMirrorState;
- (void)debugSetMirrorTransactionFailureCount:(NSUInteger)count;
- (void)debugSetMirrorAbortFailureCount:(NSUInteger)count;
- (void)debugSetMirrorEraseFailureCount:(NSUInteger)count;
- (void)debugSetMirrorCommitMode:(NSInteger)mode;
- (void)debugSetMirrorPostCommitInspectFailureCount:(NSUInteger)count;
- (void)debugSetMaximumMirrorTopologyNodes:(NSUInteger)limit;
- (BOOL)debugMutateFirstMirrorSourcePersistedTransform;
- (void)debugSetExtrusionCommitMode:(NSInteger)mode;
- (void)debugSetExtrusionAbortFailureCount:(NSUInteger)count;
- (void)debugSetExtrusionPostCommitInspectFailureCount:(NSUInteger)count;
- (NSDictionary<NSString *, NSNumber *> *)debugExtrusionState;
- (NSDictionary<NSString *, NSNumber *> *)debugBevelState;
- (void)debugSetBevelPreviewWorkerBlocked:(BOOL)blocked;
- (void)debugSetMaximumBevelCaptureTopologyNodes:(NSUInteger)limit;
- (void)debugSetMaximumBevelResultTopologyNodes:(NSUInteger)limit;
- (void)debugSetMaximumBevelResultSolids:(NSUInteger)limit;
- (void)debugSetBevelTransactionFailureCount:(NSUInteger)count;
- (void)debugSetBevelCancelDiscardFailureCount:(NSUInteger)count;
- (BOOL)debugMutateFirstBevelSourcePersistedTransform;
- (NSDictionary<NSString *, NSNumber *> *)debugBooleanPreviewState;
- (void)debugSetBooleanPreviewWorkerBlocked:(BOOL)blocked;
- (void)debugSetMaximumBooleanCaptureTopologyNodes:(NSUInteger)limit;
- (void)debugSetMaximumBooleanResultTopologyNodes:(NSUInteger)limit;
- (void)debugSetMaximumBooleanResultSolids:(NSUInteger)limit;
- (void)debugSetBooleanTransactionFailureCount:(NSUInteger)count;
- (void)debugSetBooleanAbortFailureCount:(NSUInteger)count;
- (NSArray<NSDictionary<NSString *, NSNumber *> *> *)
    debugDisplayedShapePresentationStates;
- (NSDictionary<NSString *, NSNumber *> *)
    debugFramebufferStatistics;
#endif

- (void) setConstructorMode;

- (void)setOrthoProjection:(OrthoProjectionType)orthoType;

- (void)undo;
- (void)redo;

- (void)assetData:(void(^)(NSData *_Nullable))completion;
- (void)setAssetData:(NSData *_Nonnull)data completion:(void(^)(Core3DAssetLoadResult result))completion;
- (void)setAssetFileURL:(NSURL *_Nonnull)assetFileURL
      expectedByteCount:(unsigned long long)expectedByteCount
          expectedSHA256:(NSString *_Nonnull)expectedSHA256
              completion:(void(^)(Core3DAssetLoadResult result))completion;
- (NSData *_Nullable)thumbData;
- (BOOL)saveSnapshot;
- (NSURL *_Nullable)exportWithType:(ExportType)exportType;
- (void) cancellTouchEvents;

-(NSString*) statusString;

@end

NS_ASSUME_NONNULL_END

#endif // GLViewController_h
