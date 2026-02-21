//
//  Core3DViewController+ExportManager.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 24.04.2024.
//

#import <Foundation/Foundation.h>
#import "GLViewController.h"
#import <Core3D/Core3DViewController+ExportManager.h>

#include "GLViewController+Trick.h"

@implementation Core3DViewController (ExportManager)

- (NSURL *)exportWithType:(ExportType)exportType {
    return [GLController exportWithType:exportType];
}

@end
