#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DSceneSnapshot;
@class Core3DViewController;
@class Core3DDecimationRequest;

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
    Core3DPublishCaptureErrorDuplicateArtifactEvidence,
};

//! The owner set observed by a capture. Selected never changes the editor's
//! selection; All Visible enumerates committed visible model owners directly.
typedef NS_ENUM(NSInteger, Core3DPublishCaptureScope) {
    Core3DPublishCaptureScopeSelected = 0,
    Core3DPublishCaptureScopeAllVisible = 1,
};

//! Interpretation of sourceFeatureIdentifier for an unambiguous single owner.
//! Both values are durable nonzero UUIDs; ordinary anchors do not manufacture a
//! retained feature and therefore have no retainedTerminalFeatureIdentifier.
typedef NS_ENUM(NSInteger, Core3DPublishSourceKind) {
    Core3DPublishSourceKindUnavailable = 0,
    Core3DPublishSourceKindRetainedFeature = 1,
    Core3DPublishSourceKindOrdinaryAnchor = 2,
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

//! Immutable F2 producer evidence: the completed-GLB observation joined to
//! the native face-evidence manifest from the same capture/export, plus the
//! independent measurement receipt. Bound to a capture exactly once by the
//! production publish session after measurement and currentness.
__attribute__((objc_subclassing_restricted))
@interface Core3DPublishArtifactEvidence : NSObject
@property(nonatomic, copy, readonly) NSString *artifactSHA256;            // 64-lower-hex, completed GLB
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *sourcePositionsMM; // 9 doubles per triangle, mm
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *sourceIndices;     // 3 per triangle, identity 0..n-1
@property(nonatomic, copy, readonly) NSArray<NSString *> *faceEvidenceSHA256InEmittedOrder; // 64-lower-hex each
//! Native-issued physical triangle identities, distinct from repeated analytic
//! face evidence. Nil on legacy/unqualified bindings; no public initializer
//! can manufacture these. Scalar identity alone does not authorize reduction.
@property(nonatomic, copy, readonly, nullable) NSArray<NSString *> *triangleIdentitySHA256InEmittedOrder;
@property(nonatomic, copy, readonly) NSString *measurementReceiptSHA256;  // 64-lower-hex
@property(nonatomic, assign, readonly) uint64_t addressedTriangleCount;
- (instancetype)initWithArtifactSHA256:(NSString *)artifactSHA256
                     sourcePositionsMM:(NSArray<NSNumber *> *)sourcePositionsMM
                         sourceIndices:(NSArray<NSNumber *> *)sourceIndices
      faceEvidenceSHA256InEmittedOrder:(NSArray<NSString *> *)faceEvidenceSHA256InEmittedOrder
             measurementReceiptSHA256:(NSString *)measurementReceiptSHA256
               addressedTriangleCount:(uint64_t)addressedTriangleCount NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Opaque, immutable read-only opening for current editable free owners.
//! Construction performs no export, OCAF command, history entry, or identity write.
__attribute__((objc_subclassing_restricted))
@interface Core3DPublishCapture : NSObject

@property(nonatomic, strong, readonly) Core3DSceneSnapshot *sceneSnapshot;
@property(nonatomic, copy, readonly) NSData *sourceWitnessBytes;
@property(nonatomic, copy, readonly) NSArray<NSData *> *operandWitnessBytes;
@property(nonatomic, copy, readonly) NSArray<Core3DPublishOwnerCommitments *> *ownerCommitments;
//! Captured owner identifiers (selected owners or all visible owners by scope).
@property(nonatomic, copy, readonly) NSArray<NSString *> *selectedEntityIdentifiers;
@property(nonatomic, assign, readonly) Core3DPublishCaptureScope scope;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *nativeBoundsMM;
@property(nonatomic, copy, readonly) NSString *documentIdentifier;
@property(nonatomic, copy, readonly) NSString *publicationSourceIdentifier;
@property(nonatomic, copy, readonly) NSString *frameConvention;
@property(nonatomic, assign, readonly) double metersPerUnit;
@property(nonatomic, assign, readonly) uint64_t documentGeneration;
@property(nonatomic, assign, readonly) uint64_t modelRevision;
@property(nonatomic, assign, readonly) uint64_t presentationRevision;

//! Durable source identities of the single admitted owner, exposed only when
//! exactly one owner commitment exists; otherwise nil.
@property(nonatomic, copy, readonly, nullable) NSString *sourceEntityIdentifier;
@property(nonatomic, copy, readonly, nullable) NSString *sourceDefinitionIdentifier;
@property(nonatomic, copy, readonly, nullable) NSString *sourceFeatureIdentifier;
@property(nonatomic, assign, readonly) Core3DPublishSourceKind sourceKind;
//! nil when the family has no terminal retained feature.
@property(nonatomic, copy, readonly, nullable) NSString *retainedTerminalFeatureIdentifier;
@property(nonatomic, strong, readonly, nullable) Core3DPublishArtifactEvidence *artifactEvidence;

//! Called exactly once by the production publish session after independent
//! measurement and currentness. A second or different binding refuses with a
//! typed error and leaves the first. Not for arbitrary callers.
- (BOOL)bindArtifactEvidence:(Core3DPublishArtifactEvidence *)evidence
                       error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(bindArtifactEvidence(_:));

//! Qualify one scalar owner after the completed GLB's physical channels have
//! been independently checked. Addresses are pairs (native primitive ordinal,
//! first native index), in emitted order; they are locators, never authority.
//! Native validates complete coverage, corners and analytic face membership,
//! then owns a copy of evidence with separate physical triangle identities.
- (BOOL)bindScalarArtifactEvidence:(Core3DPublishArtifactEvidence *)evidence
          sourceTriangleAddresses:(NSArray<NSNumber *> *)addresses
                            error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(bindScalarArtifactEvidence(_:sourceTriangleAddresses:));

//! Read-only construction from this capture's native-bound scalar channels.
//! Only settings are supplied by the caller; physical attributes, compact
//! material slots and unique triangle identities come from the native source.
//! This does not enable attributed execution or confer a persistence seal.
//! The retained flat-policy settings supply numeric inputs only here; a distinct
//! attributed settings/build contract is still required before worker admission.
- (nullable Core3DDecimationRequest *)makeScalarDecimationRequestWithSettingsCanonicalBytes:(NSData *)settings
    error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(makeScalarDecimationRequest(settingsCanonicalBytes:));

+ (nullable instancetype)captureController:(Core3DViewController *)controller
                                      error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(capture(controller:));

+ (nullable instancetype)captureController:(Core3DViewController *)controller
                                      scope:(Core3DPublishCaptureScope)scope
                                      error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(capture(controller:scope:));

//! Re-observes every retained dependency and the exact selection/fence. A model
//! revision match alone is never sufficient.
- (BOOL)isCurrent:(NSError * _Nullable * _Nullable)error;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end

NS_ASSUME_NONNULL_END
