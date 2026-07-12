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

- (Core3DSceneSnapshot *)captureExportSceneSnapshot {
    if (![NSThread isMainThread] || GLController == nil) {
        return nil;
    }

    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        if (viewer == nullptr) {
            return nil;
        }

        const Handle(OcctDocument) document = viewer->getDocument();
        const Handle(TDocStd_Document) transaction = document.IsNull()
            ? Handle(TDocStd_Document)()
            : document->ChangeDocument();
        const auto objectInteractor = viewer->getObjectInteractor();
        if (transaction.IsNull() || transaction->HasOpenCommand()
            || objectInteractor == nullptr
            || objectInteractor->hasActiveBoolean()
            || objectInteractor->hasUnresolvedBoolean()
            || objectInteractor->hasUnresolvedMirrorObjects()) {
            return nil;
        }

        return [self captureSceneSnapshot];
    } catch (...) {
        return nil;
    }
}

@end
