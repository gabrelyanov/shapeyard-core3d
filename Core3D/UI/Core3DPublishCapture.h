#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DSceneSnapshot;
@class Core3DViewController;

FOUNDATION_EXPORT NSErrorDomain const Core3DPublishCaptureErrorDomain;

typedef NS_ERROR_ENUM(Core3DPublishCaptureErrorDomain, Core3DPublishCaptureError) {
    Core3DPublishCaptureErrorInvalidThread = 1,
    Core3DPublishCaptureErrorBusy,
    Core3DPublishCaptureErrorSelection,
    Core3DPublishCaptureErrorUnsupportedOwner,
    Core3DPublishCaptureErrorMissingRecipe,
    Core3DPublishCaptureErrorAmbiguousRecipe,
    Core3DPublishCaptureErrorUnsupportedAppearance,
    Core3DPublishCaptureErrorNativeC1Wire,
    Core3DPublishCaptureErrorBounds,
    Core3DPublishCaptureErrorBudget,
    Core3DPublishCaptureErrorStale,
};

__attribute__((objc_subclassing_restricted))
@interface Core3DPublishOwnerCommitments : NSObject
@property(nonatomic, copy, readonly) NSString *ownerIdentifier;
@property(nonatomic, copy, readonly) NSData *recipeWitnessBytes;
@property(nonatomic, copy, readonly) NSData *geometryWitnessBytes;
@property(nonatomic, copy, readonly) NSData *placementWitnessBytes;
@property(nonatomic, copy, readonly) NSData *materialTextureWitnessBytes;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Opaque, immutable read-only opening for the initial publish source family:
//! editable free enclosures with zero or more retained cylindrical cuts.
//! Construction performs no export, OCAF command, history entry, or identity write.
__attribute__((objc_subclassing_restricted))
@interface Core3DPublishCapture : NSObject

@property(nonatomic, strong, readonly) Core3DSceneSnapshot *sceneSnapshot;
@property(nonatomic, copy, readonly) NSData *sourceWitnessBytes;
@property(nonatomic, copy, readonly) NSArray<NSData *> *operandWitnessBytes;
@property(nonatomic, copy, readonly) NSArray<Core3DPublishOwnerCommitments *> *ownerCommitments;
@property(nonatomic, copy, readonly) NSArray<NSString *> *selectedEntityIdentifiers;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *nativeBoundsMM;
@property(nonatomic, copy, readonly) NSString *documentIdentifier;
@property(nonatomic, copy, readonly) NSString *publicationSourceIdentifier;
@property(nonatomic, copy, readonly) NSString *frameConvention;
@property(nonatomic, assign, readonly) double metersPerUnit;
@property(nonatomic, assign, readonly) uint64_t documentGeneration;
@property(nonatomic, assign, readonly) uint64_t modelRevision;
@property(nonatomic, assign, readonly) uint64_t presentationRevision;

+ (nullable instancetype)captureController:(Core3DViewController *)controller
                                      error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(capture(controller:));

//! Re-observes every retained dependency and the exact selection/fence. A model
//! revision match alone is never sufficient.
- (BOOL)isCurrent:(NSError * _Nullable * _Nullable)error;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end

NS_ASSUME_NONNULL_END
