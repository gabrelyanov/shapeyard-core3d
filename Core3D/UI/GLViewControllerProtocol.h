//
//  GLViewControllerProtocol.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 18.04.2024.
//

#ifndef GLViewControllerProtocol_h
#define GLViewControllerProtocol_h

#import <UIKit/UIKit.h>

#include "Core3DViewer.h"

@protocol GLViewControllerProtocol <NSObject>

-(void)viewer:(id)sender didChangeSelections:(core3d::selection_t)selectionType;

@optional
-(void)viewer:(id)sender
    willBeginPrimaryInteractionAtDrawablePoint:(CGPoint)point
                                  drawableSize:(CGSize)drawableSize;
-(void)viewer:(id)sender
    didEndPrimaryInteractionCancelled:(BOOL)cancelled;
-(void)viewer:(id)sender
    willSelectAtDrawablePoint:(CGPoint)point
                 drawableSize:(CGSize)drawableSize;
//! Synchronize the public tool state when native Boolean cleanup cannot
//! establish a fresh action for a manipulator that would otherwise be inert.
-(void)viewerDidFailToRetainBooleanMode:(id)sender;
//! A Linear Array state change modified only its bounded transient overlay;
//! committed scene geometry and selection are unchanged.
-(void)viewerDidChangeLinearArrayPresentationOverlay:(id)sender;

@required
-(void)didSetupViewer:(id)sender;
-(void)didInvalidateSceneSnapshot:(id)sender;

#ifdef DEBUG
-(void)didChangeStatusString:(NSString *)status;
#endif

@end

#endif /* GLViewControllerProtocol_h */
