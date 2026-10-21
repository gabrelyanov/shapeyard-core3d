#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DViewController;

FOUNDATION_EXPORT NSErrorDomain const Core3DAttachmentRigErrorDomain;

typedef NS_ERROR_ENUM(Core3DAttachmentRigErrorDomain, Core3DAttachmentRigError) {
    Core3DAttachmentRigErrorInvalidThread = 1,
    Core3DAttachmentRigErrorUnavailable,
    Core3DAttachmentRigErrorInvalidArgument,
    Core3DAttachmentRigErrorInvalidEncoding,
    Core3DAttachmentRigErrorInvalidMetadata,
    Core3DAttachmentRigErrorUnsupportedSurface,
    Core3DAttachmentRigErrorForeignOwner,
    Core3DAttachmentRigErrorStale,
    Core3DAttachmentRigErrorCancelled,
    Core3DAttachmentRigErrorReplayed,
    Core3DAttachmentRigErrorApplyUnavailable,
    Core3DAttachmentRigErrorCorruptStoredMetadata,
    Core3DAttachmentRigErrorRecoveryRequired,
};

typedef NS_ENUM(NSInteger, Core3DAttachmentRigApplyResult) {
    Core3DAttachmentRigApplyResultUnavailable = 0,
    Core3DAttachmentRigApplyResultRefused,
    Core3DAttachmentRigApplyResultCancelled,
    Core3DAttachmentRigApplyResultCommitted,
    Core3DAttachmentRigApplyResultCreated,
    Core3DAttachmentRigApplyResultRemoved,
    Core3DAttachmentRigApplyResultNoChange,
    Core3DAttachmentRigApplyResultOutcomeUnknown,
    Core3DAttachmentRigApplyResultRecoveryRequired,
};

//! Typed G0 owner identity. Each component is exactly 16 native UUID bytes.
__attribute__((objc_subclassing_restricted))
@interface Core3DAttachmentRigOwnerKey : NSObject
@property(nonatomic, copy, readonly) NSData *documentID;
@property(nonatomic, copy, readonly) NSData *entityID;
@property(nonatomic, copy, readonly) NSData *definitionID;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Immutable evidence captured from one selected retained native source owner.
__attribute__((objc_subclassing_restricted))
@interface Core3DAttachmentRigOwnerObservation : NSObject
@property(nonatomic, strong, readonly) Core3DAttachmentRigOwnerKey *ownerKey;
@property(nonatomic, copy, readonly) NSString *entityIdentifier;
@property(nonatomic, copy, readonly) NSData *sourceFeatureID;
@property(nonatomic, copy, readonly) NSData *recipeWitnessBytes;
@property(nonatomic, copy, readonly) NSData *operandWitnessBytes;
@property(nonatomic, copy, readonly) NSData *dependentWitnessBytes;
//! Column-major 3x3 vectorial part followed by translation in document units.
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *placementTransformDocument;
@property(nonatomic, assign, readonly) double positiveUniformScale;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! A validated attachment frame, with physical-mm local and composed world values.
__attribute__((objc_subclassing_restricted))
@interface Core3DAttachmentRigFrameObservation : NSObject
@property(nonatomic, copy, readonly) NSData *attachmentID;
@property(nonatomic, strong, readonly) Core3DAttachmentRigOwnerKey *ownerKey;
@property(nonatomic, copy, readonly) NSString *displayName;
@property(nonatomic, copy, readonly) NSString *role;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *localFrameMM;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *worldFrameMM;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! A validated joint bind frame. Root world frames are document frames; child
//! world frames are the parent-chain product.
__attribute__((objc_subclassing_restricted))
@interface Core3DAttachmentRigJointObservation : NSObject
@property(nonatomic, copy, readonly) NSData *jointID;
@property(nonatomic, copy, readonly, nullable) NSData *parentJointID;
@property(nonatomic, copy, readonly) NSString *displayName;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *bindFrameMM;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *worldFrameMM;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Detached validation result. It carries no document mutation capability.
__attribute__((objc_subclassing_restricted))
@interface Core3DAttachmentRigValidation : NSObject
@property(nonatomic, copy, readonly) NSData *canonicalMetadataBytes;
@property(nonatomic, copy, readonly) NSString *policyID;
@property(nonatomic, copy, readonly) NSString *policyVersion;
@property(nonatomic, copy, readonly) NSArray<NSString *> *requiredRoles;
@property(nonatomic, assign, readonly, getter=isExternallyVerified) BOOL externallyVerified;
@property(nonatomic, copy, readonly) NSArray<Core3DAttachmentRigFrameObservation *> *attachments;
@property(nonatomic, copy, readonly) NSArray<Core3DAttachmentRigJointObservation *> *joints;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Opaque S1 opening. Capture and proposal binding are read-only. The opening
//! retains its native document and nonserializable currentness fence.
__attribute__((objc_subclassing_restricted))
@interface Core3DAttachmentRigOpening : NSObject

@property(nonatomic, copy, readonly) NSArray<Core3DAttachmentRigOwnerObservation *> *owners;
@property(nonatomic, copy, readonly) NSArray<NSString *> *selectedEntityIdentifiers;
@property(nonatomic, copy, readonly) NSData *sourceWitnessBytes;
@property(nonatomic, copy, readonly, nullable) NSData *metadataProposalBytes;
@property(nonatomic, strong, readonly, nullable) Core3DAttachmentRigValidation *validation;
@property(nonatomic, copy, readonly) NSString *documentIdentifier;
@property(nonatomic, copy, readonly) NSString *contextIdentifier;
@property(nonatomic, assign, readonly) double metersPerUnit;
@property(nonatomic, assign, readonly) uint64_t documentGeneration;
@property(nonatomic, assign, readonly) uint64_t modelRevision;
@property(nonatomic, assign, readonly) uint64_t presentationRevision;
@property(nonatomic, assign, readonly) NSInteger historyDepth;

+ (nullable instancetype)captureController:(Core3DViewController *)controller
                                      error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(capture(controller:));

//! Consumes the unbound opening once on success and returns another immutable
//! opening that freezes the exact candidate bytes and typed validation result.
- (nullable Core3DAttachmentRigOpening *)openingByBindingMetadataProposal:(NSData *)bytes
                                                                    error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(binding(metadataProposal:));

//! Pure bounded ABR1 validation/canonicalization without document authority.
+ (nullable NSData *)canonicalABR1ByValidatingBytes:(NSData *)bytes
                                               error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(canonicalABR1(validating:));

//! Re-observes controller, document, context, selection, generation/revision,
//! owners, recipes, operands, dependents, placements, units and history.
- (BOOL)isCurrent:(NSError * _Nullable * _Nullable)error;
- (void)cancel;

//! Frozen S1 entrance; remains unavailable so S1 callers do not silently gain
//! durable mutation semantics.
- (Core3DAttachmentRigApplyResult)apply;

//! Additive S2 entrance. Applies the bound complete ABR1 value through the
//! one native owner, with exact persisted readback before the single close.
- (Core3DAttachmentRigApplyResult)commitStoredMetadata;

//! Exact bytes read from the authoritative native carrier. A missing carrier
//! returns nil without error; malformed/unsupported bytes return a typed error.
//! The nonnull_error convention is required for that contract: without it the
//! Swift bridge treats a nil success (absent carrier) as an unreported failure
//! and throws its generic nilError instead of returning nil.
- (nullable NSData *)storedMetadataBytesWithError:
    (NSError * _Nullable * _Nullable)error
    __attribute__((swift_error(nonnull_error)))
    NS_SWIFT_NAME(storedMetadataBytes());

//! Explicit retirement is distinct from an ordinary edit and never silently
//! follows an empty proposal. The receiver must be a current unbound opening.
- (Core3DAttachmentRigApplyResult)removeStoredMetadata;

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

@end

NS_ASSUME_NONNULL_END
