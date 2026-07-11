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

@required
-(void)didSetupViewer:(id)sender;
-(void)didInvalidateSceneSnapshot:(id)sender;

#ifdef DEBUG
-(void)didChangeStatusString:(NSString *)status;
#endif

@end

#endif /* GLViewControllerProtocol_h */
