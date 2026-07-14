//
//  Core3DViewController+ExportManager.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 24.04.2024.
//

#import <Foundation/Foundation.h>
#import "GLViewController.h"
#import "../Export/Core3DNativeExportOperation+Private.h"
#import <Core3D/Core3DViewController+AvailabilityManager.h>
#import <Core3D/Core3DViewController+ExportManager.h>

#include "GLViewController+Trick.h"
#include <Prs3d_Drawer.hxx>
#include <cmath>

namespace {

bool CanCaptureCommittedExport(
    const std::shared_ptr<core3d::Core3DViewer>& viewer) {
    if (viewer == nullptr) {
        return false;
    }
    const Handle(OcctDocument) document = viewer->getDocument();
    const Handle(TDocStd_Document) transaction = document.IsNull()
        ? Handle(TDocStd_Document)()
        : document->ChangeDocument();
    const auto objectInteractor = viewer->getObjectInteractor();
    const auto shapeInteractor = viewer->getShapeInteractor();
    return !transaction.IsNull()
        && !transaction->HasOpenCommand()
        && objectInteractor != nullptr
        && shapeInteractor != nullptr
        && !objectInteractor->hasActiveBoolean()
        && !objectInteractor->hasUnresolvedBoolean()
        && !objectInteractor->hasUnresolvedMirrorObjects()
        && !objectInteractor->hasActiveLinearArray()
        && !objectInteractor->hasUnresolvedLinearArray()
        && !objectInteractor->hasActiveRadialArray()
        && !objectInteractor->hasUnresolvedRadialArray()
        && !shapeInteractor->hasActiveExtrusion()
        && !shapeInteractor->hasActiveBevel()
        && !shapeInteractor->hasActiveShell()
        && !shapeInteractor->hasUnresolvedShell();
}

} // namespace

@implementation Core3DViewController (ExportManager)

- (NSURL *)exportWithType:(ExportType)exportType {
    if (![self canExportType:exportType]) {
        return nil;
    }
    return [GLController exportWithType:exportType];
}

- (Core3DNativeExportOperation *)prepareNativeExportOperationWithType:
    (ExportType)exportType {
    if (![NSThread isMainThread]
        || (exportType != ExportTypeObj
            && exportType != ExportTypeStl
            && exportType != ExportTypeStep)
        || GLController == nil) {
        return nil;
    }

    NSURL *inputRoot = nil;
    NSURL *cleanupRoot = nil;
    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        const Handle(OcctDocument) document = viewer == nullptr
            ? Handle(OcctDocument)()
            : viewer->getDocument();
        if (!CanCaptureCommittedExport(viewer)
            || document.IsNull()
            || (![self canExportType:exportType]
                && !document->IsGeometryDocumentEmpty())) {
            return nil;
        }

        Aspect_TypeOfDeflection deflectionType = Aspect_TOD_RELATIVE;
        Standard_Real deviationCoefficient = 0.001;
        Standard_Real deviationAngle = 20.0 * M_PI / 180.0;
        Standard_Real maximalChordialDeviation = 0.0001;
        if (exportType != ExportTypeStep) {
            const Handle(AIS_InteractiveContext)& context =
                viewer->AisContext();
            const Handle(Prs3d_Drawer) drawer = context.IsNull()
                ? Handle(Prs3d_Drawer)()
                : context->DefaultDrawer();
            if (drawer.IsNull()) {
                return nil;
            }
            deflectionType = drawer->TypeOfDeflection();
            deviationCoefficient = drawer->DeviationCoefficient();
            deviationAngle = drawer->DeviationAngle();
            maximalChordialDeviation =
                drawer->MaximalChordialDeviation();
            if ((deflectionType != Aspect_TOD_ABSOLUTE
                 && deflectionType != Aspect_TOD_RELATIVE)
                || !std::isfinite(deviationCoefficient)
                || deviationCoefficient <= 0.0
                || !std::isfinite(deviationAngle)
                || deviationAngle <= 0.0
                || !std::isfinite(maximalChordialDeviation)
                || maximalChordialDeviation <= 0.0) {
                NSLog(@"[NativeExport] Invalid mesh settings: type=%ld coefficient=%g angle=%g chord=%g",
                      static_cast<long>(deflectionType),
                      deviationCoefficient,
                      deviationAngle,
                      maximalChordialDeviation);
                return nil;
            }
        }

        NSFileManager *fileManager = NSFileManager.defaultManager;
        NSURL *temporaryDirectory = fileManager.temporaryDirectory;
        NSString *identifier = NSUUID.UUID.UUIDString;
        inputRoot = [temporaryDirectory URLByAppendingPathComponent:
            [@"Core3DNativeExportInput-" stringByAppendingString:identifier]
            isDirectory:YES];
        cleanupRoot = [temporaryDirectory URLByAppendingPathComponent:
            [@"Core3DNativeExportOutput-" stringByAppendingString:identifier]
            isDirectory:YES];
        NSURL *packageRoot = [cleanupRoot
            URLByAppendingPathComponent:@"contents"
            isDirectory:YES];
        NSError *directoryError = nil;
        if (![fileManager createDirectoryAtURL:inputRoot
                    withIntermediateDirectories:NO
                                     attributes:nil
                                          error:&directoryError]
            || ![fileManager createDirectoryAtURL:packageRoot
                       withIntermediateDirectories:YES
                                        attributes:nil
                                             error:&directoryError]) {
            NSLog(@"[NativeExport] Temporary directory creation failed: %@",
                  directoryError.localizedDescription);
            [fileManager removeItemAtURL:inputRoot error:nil];
            [fileManager removeItemAtURL:cleanupRoot error:nil];
            return nil;
        }

        NSURL *snapshotBase = [inputRoot
            URLByAppendingPathComponent:@"document.tmp"
            isDirectory:NO];
        const std::string snapshotPath = viewer->getDocument()->save(
            snapshotBase.path.UTF8String);
        if (snapshotPath.empty()) {
            NSLog(@"[NativeExport] Private document save failed");
            [fileManager removeItemAtURL:inputRoot error:nil];
            [fileManager removeItemAtURL:cleanupRoot error:nil];
            return nil;
        }
        NSString *snapshotPathString = [NSString
            stringWithUTF8String:snapshotPath.c_str()];
        if (snapshotPathString == nil) {
            NSLog(@"[NativeExport] Private document path was not UTF-8");
            [fileManager removeItemAtURL:inputRoot error:nil];
            [fileManager removeItemAtURL:cleanupRoot error:nil];
            return nil;
        }
        NSURL *snapshotURL = [NSURL fileURLWithPath:snapshotPathString];
        NSString *inputPrefix = [inputRoot.path stringByAppendingString:@"/"];
        NSDictionary<NSFileAttributeKey, id> *attributes = [fileManager
            attributesOfItemAtPath:snapshotURL.path
            error:nil];
        const unsigned long long snapshotByteCount =
            [attributes[NSFileSize] unsignedLongLongValue];
        if (![snapshotURL.path hasPrefix:inputPrefix]
            || ![attributes[NSFileType] isEqualToString:NSFileTypeRegular]
            || snapshotByteCount == 0
            || snapshotByteCount > 256ull * 1024ull * 1024ull) {
            NSLog(@"[NativeExport] Private document postflight failed: type=%@ bytes=%llu contained=%d",
                  attributes[NSFileType],
                  snapshotByteCount,
                  [snapshotURL.path hasPrefix:inputPrefix]);
            [fileManager removeItemAtURL:inputRoot error:nil];
            [fileManager removeItemAtURL:cleanupRoot error:nil];
            return nil;
        }

        Core3DNativeExportOperation *operation =
            [[Core3DNativeExportOperation alloc]
                initWithSnapshotURL:snapshotURL
                snapshotCleanupURL:inputRoot
                packageRootURL:packageRoot
                cleanupURL:cleanupRoot
                exportType:exportType
                deflectionType:static_cast<NSInteger>(deflectionType)
                deviationCoefficient:deviationCoefficient
                deviationAngle:deviationAngle
                maximalChordialDeviation:maximalChordialDeviation];
        if (operation == nil) {
            NSLog(@"[NativeExport] Operation construction failed");
            [fileManager removeItemAtURL:inputRoot error:nil];
            [fileManager removeItemAtURL:cleanupRoot error:nil];
        }
        return operation;
    } catch (...) {
        if (inputRoot != nil) {
            [NSFileManager.defaultManager removeItemAtURL:inputRoot error:nil];
        }
        if (cleanupRoot != nil) {
            [NSFileManager.defaultManager removeItemAtURL:cleanupRoot error:nil];
        }
        return nil;
    }
}

- (Core3DSceneSnapshot *)captureExportSceneSnapshot {
    if (![NSThread isMainThread] || GLController == nil) {
        return nil;
    }

    try {
        const std::shared_ptr<core3d::Core3DViewer> viewer =
            GLController.viewer;
        const Handle(OcctDocument) document = viewer == nullptr
            ? Handle(OcctDocument)()
            : viewer->getDocument();
        if (!CanCaptureCommittedExport(viewer)
            || document.IsNull()
            || (![self canExportType:ExportTypeGltf]
                && !document->IsGeometryDocumentEmpty())) {
            return nil;
        }

        return [self captureSceneSnapshot];
    } catch (...) {
        return nil;
    }
}

@end
