//
//  Core3DViewController.h
//  Core3D
//
//  Created by Dmitriy Zadorozhnyy on 28.03.2024.
//

#import <UIKit/UIKit.h>
#import <Core3D/PrimitiveType.h>
#import <Core3D/PrimitiveSelectionType.h>
#import <Core3D/PrimitiveGizmoType.h>
#import <Core3D/UIStateChanging.h>
#import <Core3D/ExportType.h>
#import <Core3D/OrthoProjectionType.h>
#import <Core3D/Core3DMaterialController.h>

NS_ASSUME_NONNULL_BEGIN

typedef NS_ENUM(NSInteger, Core3DAssetLoadResult) {
    Core3DAssetLoadResultSuccess = 0,
    Core3DAssetLoadResultInvalidData,
    Core3DAssetLoadResultTemporaryFileFailure,
    Core3DAssetLoadResultBusy,
    Core3DAssetLoadResultUnsupportedVersion,
    Core3DAssetLoadResultInternalFailure,
};

typedef struct {
    double min;
    double max;
} Boundaries;

@protocol PrimitiveManagerProtocol<NSObject>

- (void)addPrimitive:(PrimitiveType)primitiveType;
- (void)addPrimitivesFromJSON:(NSString *)json;
- (void)deleteSelected;
- (void)selectAll;
- (void)duplicateSelected;
- (void)setChamfer:(CGFloat)value;
- (Boundaries)getChamferBoundaries;
- (void)applyChamfer;
- (void)cancelChamfer;
- (void)applyMirror;
- (void)cancelMirror;
- (void)applySubtract;
- (void)cancelSubtract;
- (void)applyUnion;
- (void)cancelUnion;
- (void)undo;
- (void)redo;
- (NSString *_Nullable)getCoreInfoText;
- (void)setOrthoProjection:(OrthoProjectionType)orthoType;
- (void)setSnappingTranslation:(double)value;
- (double)getSnappingTranslation;
- (void)setSnappingRotation:(double)value;
- (double)getSnappingRotation;
- (void)setSnappingScale:(double)value;
- (double)getSnappingScale;

#ifdef DEBUG
- (void)handleDebug_A;
- (void)handleDebug_B;
#endif

@end

@protocol AvailabilityManagerProtocol<NSObject>

- (BOOL)canAdd;
- (BOOL)canDelete;
- (BOOL)canDuplicate;
- (BOOL)canUndo;
- (BOOL)canRedo;
- (BOOL)canApply;
- (BOOL)canApplyMaterial;

- (NSArray<NSNumber *> *_Nonnull)availableGizmoTypes;

@end

@protocol ExportManagerProtocol<NSObject>

- (NSURL *_Nullable)exportWithType:(ExportType)exportType;

@end


@protocol Core3DViewControllerProtocol<NSObject>

@optional
- (void)viewDidLoadFromBundle;
- (void)viewDidFailToLoadFromBundle:(Core3DAssetLoadResult)result
    NS_SWIFT_NAME(viewDidFailToLoadFromBundle(_:));

@end


@interface Core3DViewController : UIViewController {
    UIStateChanging _currentStateChanging;
    PrimitiveSelectionType _currentSelectionType;
    PrimitiveGizmoType _currentGizmoType;
    NSArray<NSNumber *> *_availableGizmoTypes;
    BOOL _isViewerSetup;
    BOOL _isPreviewMode;
    NSString *_coreInfoText;
}

@property (nonatomic, strong, readonly) Core3DMaterialController* materialController;

@property (nonatomic, strong, nonnull) id glController NS_REFINED_FOR_SWIFT;
@property (nonatomic, assign, readonly) PrimitiveSelectionType currentSelectionType;
@property (nonatomic, assign, readonly) PrimitiveGizmoType currentGizmoType;
@property (nonatomic, readonly) UIView* glView;

@property (nonatomic, assign) BOOL can_add NS_REFINED_FOR_SWIFT;
@property (nonatomic, assign) BOOL can_delete NS_REFINED_FOR_SWIFT;
@property (nonatomic, assign) BOOL can_duplicate NS_REFINED_FOR_SWIFT;
@property (nonatomic, assign) BOOL can_undo NS_REFINED_FOR_SWIFT;
@property (nonatomic, assign) BOOL can_redo NS_REFINED_FOR_SWIFT;
@property (nonatomic, assign) BOOL can_apply NS_REFINED_FOR_SWIFT;
@property (nonatomic, assign) BOOL can_apply_material NS_REFINED_FOR_SWIFT;

@property (nonatomic, assign, readonly) BOOL isEmptyOfDisplayedObjects;
@property (nonatomic, assign, readonly) NSInteger numberOfDisplayedShapes;

@property (nonatomic, weak, nullable) id<Core3DViewControllerProtocol> delegate;

- (void)setPreviewMode;

- (void)assetData:(void(^)(NSData *_Nullable))completion;
- (NSData *_Nullable)thumbData;

- (void)loadFromBundle:(NSURL *)bundleUrl;
- (void)saveSnapshot;

- (void)setSelectionType:(PrimitiveSelectionType)type;
- (void)setGizmoType:(PrimitiveGizmoType)type;

- (void)addTestPrimitives;

- (void)sendNotifyUIState:(UIStateChanging)state NS_REFINED_FOR_SWIFT;
- (void)viewWillUpdateUIState:(UIStateChanging)state;
- (void)viewDidSetup;
- (void)viewDidAssetModify;
- (void)viewDidLoadFromBundle;
- (void)viewDidFailToLoadFromBundle:(Core3DAssetLoadResult)result
    NS_SWIFT_NAME(viewDidFailToLoadFromBundle(_:));

@end

NS_ASSUME_NONNULL_END
