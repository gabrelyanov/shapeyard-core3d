//
//  GLViewControllerProtocol.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 18.04.2024.
//

#ifndef GLViewControllerProtocol_h
#define GLViewControllerProtocol_h

@protocol GLViewControllerProtocol <NSObject>

-(void)viewer:(id)sender didChangeSelections:(core3d::selection_t)selectionType;
-(void)didSetupViewer:(id)sender;

#ifdef DEBUG
-(void)didChangeStatusString:(NSString *)status;
#endif

@end

#endif /* GLViewControllerProtocol_h */
