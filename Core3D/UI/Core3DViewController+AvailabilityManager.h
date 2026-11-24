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
//! R9-C3 (CTO 22 Nov 18:47): Bevel is offered for the selected solid part but
//! cannot open, because its sharp edges are more than Bevel takes at once. The
//! column shows Bevel dimmed and a tap says "Too many edges at once. Pick edges
//! in Edge mode."
- (BOOL)bevelOverEdgeLimit NS_SWIFT_NAME(bevelOverEdgeLimit());
//! CTO 23 Nov 03:55: with bevelOverEdgeLimit, true when the part is past Bevel's
//! piece budget (too detailed), not past the edge limit: its own sentence.
- (BOOL)bevelTooDetailed NS_SWIFT_NAME(bevelTooDetailed());
//! Round 10 R10P-S1: with bevelOverEdgeLimit, true when every edge of the part is
//! already round (nothing sharp is left for a whole-part Bevel): its own sentence.
- (BOOL)bevelAllRound NS_SWIFT_NAME(bevelAllRound());
//! R10-F4 (store build, CTO 24 Nov 18:38): true when exactly one part is
//! picked as a whole object and it is a mesh (Mesh > Create mesh copy, or an
//! imported mesh). Not debug-only: the column keeps that part's other cells
//! dimmed with a reason in the App Store app too.
- (BOOL)singleSelectedPartIsMesh NS_SWIFT_NAME(singleSelectedPartIsMesh());
//! Export support is document-wide rather than selection-based. In
//! particular, STEP is unavailable while any TriangleMesh definition remains.
- (BOOL)canExportType:(ExportType)exportType
    NS_SWIFT_NAME(canExport(type:));
- (NSArray<NSNumber *> *_Nonnull)availableExportTypes;

@end

NS_ASSUME_NONNULL_END
