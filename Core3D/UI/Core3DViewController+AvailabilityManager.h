//
//  Core3DViewController+AvailabilityManager.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 15.04.2024.
//

#import <Core3D/Core3DViewController.h>

NS_ASSUME_NONNULL_BEGIN

@interface Core3DViewController (AvailabilityManager)<AvailabilityManagerProtocol>

- (BOOL)canAdd;
- (BOOL)canDelete;
- (BOOL)canDuplicate;
- (BOOL)canUndo;
- (BOOL)canRedo;
- (NSArray<NSNumber *> *_Nonnull)availableGizmoTypes;
- (BOOL)canApplyMaterial;
//! Export support is document-wide rather than selection-based. In
//! particular, STEP is unavailable while any TriangleMesh definition remains.
- (BOOL)canExportType:(ExportType)exportType
    NS_SWIFT_NAME(canExport(type:));
- (NSArray<NSNumber *> *_Nonnull)availableExportTypes;

@end

NS_ASSUME_NONNULL_END
