//
//  Core3DViewController+ExportManager.m
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 24.04.2024.
//

#import <Foundation/Foundation.h>
#import "GLViewController.h"
#import "../Export/Core3DNativeExportOperation+Private.h"
#import "../Viewport/Core3DSceneSnapshot.h"
#import <Core3D/Core3DViewController+AvailabilityManager.h>
#import <Core3D/Core3DViewController+ExportManager.h>

#include "GLViewController+Trick.h"
#include <Prs3d_Drawer.hxx>
#include <cmath>
#include <cstring>

namespace {

bool CanCaptureCommittedExport(
    const std::shared_ptr<core3d::Core3DViewer>& viewer) {
    if (viewer == nullptr || viewer->hasUnresolvedOrdinaryEdit()) {
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
        && !objectInteractor->hasUnresolvedDuplicate()
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

// Derive scope from one committed publication, before creating any private
// files. A missing or inconsistent selected item cannot widen the export.
NSArray<NSString *> *CaptureSelectedSTLIdentifiers(Core3DSceneSnapshot *snapshot) {
    constexpr NSUInteger maximumItems = 50'000;
    constexpr NSUInteger maximumIdentifierBytes = 128;
    if (snapshot == nil
        || snapshot.selectionMode != Core3DSceneElementKindObject
        || snapshot.selection.selectedElements.count == 0
        || snapshot.selection.selectedElements.count > maximumItems
        || snapshot.renderItems.count > maximumItems) {
        return nil;
    }
    NSMutableSet<NSString *> *selected = [NSMutableSet set];
    NSMutableArray<NSString *> *ordered = [NSMutableArray array];
    for (Core3DSceneElementIdentifier *element in snapshot.selection.selectedElements) {
        NSString *identifier = element.entityIdentifier;
        NSData *bytes = [identifier dataUsingEncoding:NSUTF8StringEncoding
                                allowLossyConversion:NO];
        if (element.kind != Core3DSceneElementKindObject
            || element.topologyIndex != 0
            || bytes.length == 0 || bytes.length > maximumIdentifierBytes
            || memchr(bytes.bytes, 0, bytes.length) != nullptr
            || [selected containsObject:identifier]) {
            return nil;
        }
        [selected addObject:identifier];
        [ordered addObject:identifier];
    }
    NSMutableSet<NSString *> *matched = [NSMutableSet set];
    for (Core3DSceneRenderItemSnapshot *item in snapshot.renderItems) {
        if (item.renderRole != Core3DSceneRenderRoleModel
            || item.coordinateSpace != Core3DSceneCoordinateSpaceWorld
            || item.renderStyle != Core3DSceneRenderStyleShaded
            || ![selected containsObject:item.entityIdentifier]) {
            continue;
        }
        if (!item.isVisible || !item.isSelected
            || [matched containsObject:item.entityIdentifier]) {
            return nil;
        }
        [matched addObject:item.entityIdentifier];
    }
    return [matched isEqualToSet:selected] ? [ordered copy] : nil;
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
    return [self prepareNativeExportOperationWithType:exportType
                                 selectedObjectsOnly:NO];
}

- (Core3DNativeExportOperation *)prepareNativeExportOperationWithType:
    (ExportType)exportType selectedObjectsOnly:(BOOL)selectedObjectsOnly {
    return [self prepareNativeExportOperationWithType:exportType
                                 selectedObjectsOnly:selectedObjectsOnly
                                         meshQuality:Core3DExportMeshQualityViewport];
}

- (Core3DNativeExportOperation *)prepareNativeExportOperationWithType:
    (ExportType)exportType selectedObjectsOnly:(BOOL)selectedObjectsOnly
    meshQuality:(Core3DExportMeshQuality)meshQuality {
    if (meshQuality < Core3DExportMeshQualityViewport
        || meshQuality > Core3DExportMeshQualityFine
        || (meshQuality != Core3DExportMeshQualityViewport
            && exportType != ExportTypeObj && exportType != ExportTypeStl)) {
        return nil;
    }
    if ((selectedObjectsOnly && exportType != ExportTypeStl)
        || ![NSThread isMainThread]
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

        NSArray<NSString *> *selectedIdentifiers = nil;
        if (selectedObjectsOnly) {
            selectedIdentifiers = CaptureSelectedSTLIdentifiers(
                [self captureExportSceneSnapshot]);
            if (selectedIdentifiers == nil) {
                return nil;
            }
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

        if (meshQuality != Core3DExportMeshQualityViewport) {
            deflectionType = Aspect_TOD_RELATIVE;
            // Dimensionless chord coefficients and angles in radians. These
            // belong to the export operation, never the live AIS drawer.
            deviationCoefficient = meshQuality == Core3DExportMeshQualityCoarse
                ? 0.01 : (meshQuality == Core3DExportMeshQualityFine ? 0.00025 : 0.001);
            deviationAngle = (meshQuality == Core3DExportMeshQualityCoarse
                ? 30.0 : (meshQuality == Core3DExportMeshQualityFine ? 10.0 : 20.0))
                * M_PI / 180.0;
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
                selectedEntityIdentifiers:selectedIdentifiers
                meshQuality:meshQuality
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
