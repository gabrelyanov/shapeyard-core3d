#import <Core3D/Core3DSceneSnapshot.h>
#import <Core3D/Core3DViewController.h>

NS_ASSUME_NONNULL_BEGIN

typedef NS_ENUM(NSInteger, Core3DAuthoredParameterMutationKind) {
    Core3DAuthoredParameterMutationKindUnregistered = 0,
    Core3DAuthoredParameterMutationKindBooleanRecipeInput = 1,
    Core3DAuthoredParameterMutationKindBooleanAnalyticInput = 2,
    Core3DAuthoredParameterMutationKindBooleanOperation = 3,
    Core3DAuthoredParameterMutationKindBooleanPlacement = 4,
    Core3DAuthoredParameterMutationKindLoftStationValues = 5,
    Core3DAuthoredParameterMutationKindLoftFrame = 6,
    Core3DAuthoredParameterMutationKindProfileShellValues = 7,
};

typedef NS_ENUM(NSInteger, Core3DAuthoredParameterOpeningCurrentness) {
    Core3DAuthoredParameterOpeningCurrentnessCurrent = 0,
    Core3DAuthoredParameterOpeningCurrentnessStale,
    Core3DAuthoredParameterOpeningCurrentnessCancelled,
    Core3DAuthoredParameterOpeningCurrentnessRecovery,
};

typedef NS_ENUM(NSInteger, Core3DAuthoredParameterPreparationResult) {
    Core3DAuthoredParameterPreparationResultUnsupportedCapability = 0,
    Core3DAuthoredParameterPreparationResultMalformedMutation,
    Core3DAuthoredParameterPreparationResultForeignOpening,
    Core3DAuthoredParameterPreparationResultStaleOpening,
    Core3DAuthoredParameterPreparationResultCancelled,
    Core3DAuthoredParameterPreparationResultRecoveryRequired,
};

typedef NS_ENUM(NSInteger, Core3DAuthoredLoftField) {
    Core3DAuthoredLoftFieldStationZ = 0,
    Core3DAuthoredLoftFieldStationCenterX,
    Core3DAuthoredLoftFieldStationCenterY,
    Core3DAuthoredLoftFieldFrameTranslationX,
    Core3DAuthoredLoftFieldFrameTranslationY,
    Core3DAuthoredLoftFieldFrameTranslationZ,
    Core3DAuthoredLoftFieldFrameQuaternionX,
    Core3DAuthoredLoftFieldFrameQuaternionY,
    Core3DAuthoredLoftFieldFrameQuaternionZ,
    Core3DAuthoredLoftFieldFrameQuaternionW,
    Core3DAuthoredLoftFieldFrameSignedScale,
};

typedef NS_ENUM(NSInteger, Core3DAuthoredParameterApplyOutcome) {
    Core3DAuthoredParameterApplyOutcomeCommitted = 0,
    Core3DAuthoredParameterApplyOutcomeUnchanged,
    Core3DAuthoredParameterApplyOutcomeRefused,
    Core3DAuthoredParameterApplyOutcomeCancelled,
    Core3DAuthoredParameterApplyOutcomeOutcomeUnknown,
};

__attribute__((objc_subclassing_restricted))
@interface Core3DAuthoredParameterApplyResult : NSObject
@property(nonatomic,readonly) Core3DAuthoredParameterApplyOutcome outcome;
@property(nonatomic,copy,readonly) NSString *refusalCode;
@property(nonatomic,copy,readonly,nullable) NSNumber *measuredUndoDelta;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

typedef NS_ENUM(NSInteger, Core3DAuthoredParameterRecoveryOutcome) {
    Core3DAuthoredParameterRecoveryOutcomeCommitted = 0,
    Core3DAuthoredParameterRecoveryOutcomePriorRestored,
    Core3DAuthoredParameterRecoveryOutcomeRecoveryRequired,
    Core3DAuthoredParameterRecoveryOutcomeNotOwner,
};

typedef NS_ENUM(NSInteger, Core3DAuthoredParameterRecoveryPhase) {
    Core3DAuthoredParameterRecoveryPhaseNone = 0,
    Core3DAuthoredParameterRecoveryPhasePending,
    Core3DAuthoredParameterRecoveryPhaseCompanionSettled,
};

//! Typed receipt for the single outstanding document-owned authored-Boolean
//! recovery. Recovery-required and not-owner receipts never carry a delta.
__attribute__((objc_subclassing_restricted))
@interface Core3DAuthoredParameterRecoveryResult : NSObject
@property(nonatomic,readonly) Core3DAuthoredParameterRecoveryOutcome outcome;
@property(nonatomic,copy,readonly) NSString *refusalCode;
@property(nonatomic,copy,readonly,nullable) NSNumber *measuredUndoDelta;
@property(nonatomic,readonly) Core3DAuthoredParameterRecoveryPhase recoveryPhase;
#if DEBUG
@property(nonatomic,readonly) NSUInteger reconcileCalls;
@property(nonatomic,readonly) NSUInteger observationCalls;
@property(nonatomic,readonly) NSUInteger observationFaultHits;
@property(nonatomic,readonly) NSUInteger publicationCalls;
@property(nonatomic,readonly) NSUInteger recoveryCommandCalls;
#endif
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DAuthoredParameterOperation : NSObject
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
//! Stops only this operation. A settled operation returns NO and cannot affect
//! later authored work.
- (BOOL)cancel;
@end

//! Immutable native-issued source/suffix capture. Descriptive properties do
//! not carry mutation authority; the private native owner and document fence do.
__attribute__((objc_subclassing_restricted))
@interface Core3DAuthoredParameterOpening : NSObject
@property(nonatomic,copy,readonly) NSString *documentIdentifier;
@property(nonatomic,copy,readonly) NSString *entityIdentifier;
@property(nonatomic,copy,readonly) NSString *definitionIdentifier;
@property(nonatomic,copy,readonly) NSData *sourceCanonicalBytes;
@property(nonatomic,copy,readonly) NSData *suffixCanonicalBytes;
@property(nonatomic,readonly) double metersPerUnit;
@property(nonatomic,readonly) Core3DAuthoredParameterOpeningCurrentness currentness;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

typedef NS_ENUM(NSInteger, Core3DBooleanPairStatus) {
    Core3DBooleanPairStatusCaptured = 0,
    Core3DBooleanPairStatusPrepared,
    Core3DBooleanPairStatusRefused,
    Core3DBooleanPairStatusCancelled,
};

typedef NS_ENUM(NSInteger, Core3DBooleanPairRefusal) {
    Core3DBooleanPairRefusalNone = 0,
    Core3DBooleanPairRefusalWrongThread,
    Core3DBooleanPairRefusalForeignDocument,
    Core3DBooleanPairRefusalSameSource,
    Core3DBooleanPairRefusalAliasSource,
    Core3DBooleanPairRefusalUnsupportedSource,
    Core3DBooleanPairRefusalStaleSource,
    Core3DBooleanPairRefusalInvalidOperation,
    Core3DBooleanPairRefusalUnprovedCorrespondence,
    Core3DBooleanPairRefusalInvalidResult,
    Core3DBooleanPairRefusalBadEncoding,
    Core3DBooleanPairRefusalBudget,
    Core3DBooleanPairRefusalCancelled,
};

typedef NS_ENUM(NSInteger, Core3DBooleanOriginalsEditKind) {
    Core3DBooleanOriginalsEditKindOperation = 0,
    Core3DBooleanOriginalsEditKindDimensions,
    Core3DBooleanOriginalsEditKindPlacement,
    Core3DBooleanOriginalsEditKindScalarMaterial,
    Core3DBooleanOriginalsEditKindGroupMembership,
    Core3DBooleanOriginalsEditKindVisibility,
};

typedef NS_ENUM(NSInteger, Core3DBooleanOriginalRole) {
    Core3DBooleanOriginalRoleLeft = 1,
    Core3DBooleanOriginalRoleRight = 2,
};

//! Closed value edit. `values` is empty for operation/group/visibility, has
//! three dimensions, seven placement scalars (translation + quaternion), or
//! six scalar-material values (RGBA + metallic + roughness). No persisted
//! originals bytes, identities, labels or role map can be supplied.
__attribute__((objc_subclassing_restricted))
@interface Core3DBooleanOriginalsEdit : NSObject
- (nullable instancetype)initWithKind:(Core3DBooleanOriginalsEditKind)kind
    role:(Core3DBooleanOriginalRole)role values:(NSArray<NSNumber *> *)values
    operation:(Core3DPartBooleanOperation)operation flag:(BOOL)flag
    NS_SWIFT_NAME(init(kind:role:values:operation:flag:));
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Opaque native-issued paired authority. It exposes no labels, initializer,
//! command lease, apply operation, or serializable mutation authority.
__attribute__((objc_subclassing_restricted))
@interface Core3DBooleanPairOpening : NSObject
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DBooleanPairPrepared : NSObject
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DBooleanPairCaptureResult : NSObject
@property(nonatomic,readonly) Core3DBooleanPairStatus status;
@property(nonatomic,readonly) Core3DBooleanPairRefusal refusal;
@property(nonatomic,strong,readonly,nullable) Core3DBooleanPairOpening *opening;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DBooleanPairPrepareResult : NSObject
@property(nonatomic,readonly) Core3DBooleanPairStatus status;
@property(nonatomic,readonly) Core3DBooleanPairRefusal refusal;
@property(nonatomic,strong,readonly,nullable) Core3DBooleanPairPrepared *prepared;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Value-only O4 observation. All geometry values are read from the independent
//! ordinary-root replay and never from a caller candidate.
__attribute__((objc_subclassing_restricted))
@interface Core3DBooleanPairObservationResult : NSObject
@property(nonatomic,readonly) Core3DBooleanPairStatus status;
@property(nonatomic,readonly) Core3DBooleanPairRefusal refusal;
@property(nonatomic,copy,readonly) NSArray<NSString *> *entityIdentifiers;
@property(nonatomic,copy,readonly) NSArray<NSString *> *sourceFeatureIdentifiers;
@property(nonatomic,copy,readonly) NSArray<NSData *> *sourceRecipeBytes;
@property(nonatomic,copy,readonly) NSData *originalsCanonicalBytes;
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *sourceVolumes;
@property(nonatomic,readonly) double resultVolume;
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *resultBounds;
@property(nonatomic,readonly) NSUInteger resultClosedLoopCount;
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *resultMembership;
@property(nonatomic,readonly) double metersPerUnit;
@property(nonatomic,readonly) NSInteger undoDepth;
@property(nonatomic,readonly) NSInteger redoDepth;
@property(nonatomic,copy,readonly) NSData *retainedSuffixCanonicalBytes;
@property(nonatomic,copy,readonly) NSData *originalsDigest;
@property(nonatomic,copy,readonly) NSData *effectiveDigest;
@property(nonatomic,copy,readonly) NSData *identityDigest;
@property(nonatomic,copy,readonly) NSData *resultDigest;
@property(nonatomic,copy,readonly) NSArray<NSArray<NSNumber *> *> *effectiveDimensions;
@property(nonatomic,copy,readonly) NSArray<NSArray<NSNumber *> *> *effectivePlacements;
@property(nonatomic,copy,readonly) NSArray<NSArray<NSNumber *> *> *effectiveMaterials;
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *effectiveMaterialPresent;
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *effectiveGroupMembership;
@property(nonatomic,copy,readonly) NSArray<NSNumber *> *effectiveVisibility;
@property(nonatomic,readonly) BOOL resultVisible;
@property(nonatomic,readonly) Core3DPartBooleanOperation operation;
@property(nonatomic,readonly) NSUInteger graphNodeCount;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

typedef NS_ENUM(NSInteger, Core3DBooleanPairApplyStatus) {
    Core3DBooleanPairApplyStatusCommitted = 0,
    Core3DBooleanPairApplyStatusRefused,
    Core3DBooleanPairApplyStatusCancelled,
    Core3DBooleanPairApplyStatusOutcomeUnknown,
};

//! Immutable proof that one native pair command is known committed. It is
//! deliberately absent for refusal, cancellation and unknown settlement.
__attribute__((objc_subclassing_restricted))
@interface Core3DBooleanPairConsumptionReceipt : NSObject
@property(nonatomic,copy,readonly) NSString *targetEntityIdentifier;
@property(nonatomic,copy,readonly) NSString *toolEntityIdentifier;
@property(nonatomic,readonly) NSUInteger undoDelta;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

__attribute__((objc_subclassing_restricted))
@interface Core3DBooleanPairApplyResult : NSObject
@property(nonatomic,readonly) Core3DBooleanPairApplyStatus status;
@property(nonatomic,readonly) Core3DBooleanPairRefusal refusal;
@property(nonatomic,strong,readonly,nullable) Core3DBooleanPairConsumptionReceipt *receipt;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

#if DEBUG
//! N06 fixed creation-only fault sites. Values mirror the native enum and are
//! stable test contract; they are not the existing-result SAB fault sites.
typedef NS_ENUM(NSInteger, Core3DDebugBooleanPairCreationFaultSite) {
    Core3DDebugBooleanPairCreationFaultSiteAfterTargetShapeWrite = 0,
    Core3DDebugBooleanPairCreationFaultSiteAfterCompositeRecordWrite = 1,
    Core3DDebugBooleanPairCreationFaultSiteAfterSourceBindingsWrite = 2,
    Core3DDebugBooleanPairCreationFaultSiteAfterMetadataWrite = 3,
    Core3DDebugBooleanPairCreationFaultSiteAfterToolRemoval = 4,
    Core3DDebugBooleanPairCreationFaultSiteAfterRecordReadback = 5,
    Core3DDebugBooleanPairCreationFaultSiteAfterBytesReadback = 6,
    Core3DDebugBooleanPairCreationFaultSiteAfterShapeReadback = 7,
    Core3DDebugBooleanPairCreationFaultSiteAfterSemanticReadback = 8,
    Core3DDebugBooleanPairCreationFaultSiteAfterToolRemovalReadback = 9,
    Core3DDebugBooleanPairCreationFaultSitePublicationOwner = 10,
    Core3DDebugBooleanPairCreationFaultSiteHistorySaturation = 11,
};

__attribute__((objc_subclassing_restricted))
@interface Core3DDebugBooleanPairCreationFaultReceipt : NSObject
@property(nonatomic,readonly) Core3DDebugBooleanPairCreationFaultSite site;
@property(nonatomic,readonly) NSUInteger hitCount;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! SAB-N1 fixed post-write fault sites for the authored-Boolean stage. The
//! values mirror the native core3d::authored_parameter::debug site enum and
//! are the test contract: never renumber or reuse them.
typedef NS_ENUM(NSInteger, Core3DDebugAuthoredBooleanFaultSite) {
    Core3DDebugAuthoredBooleanFaultSiteAfterOwnerShapeWrite = 0,
    Core3DDebugAuthoredBooleanFaultSiteAfterPayloadWrite = 1,
    Core3DDebugAuthoredBooleanFaultSiteAfterPayloadBoundary = 2,
    Core3DDebugAuthoredBooleanFaultSiteAfterFirstSourceBinding = 3,
    Core3DDebugAuthoredBooleanFaultSiteAfterTreatmentAttribute = 4,
    Core3DDebugAuthoredBooleanFaultSiteAfterSecondSourceBinding = 5,
    Core3DDebugAuthoredBooleanFaultSiteGraphReader = 6,
    Core3DDebugAuthoredBooleanFaultSiteGraphValuePresence = 7,
    Core3DDebugAuthoredBooleanFaultSiteGraphBytes = 8,
    Core3DDebugAuthoredBooleanFaultSiteGraphSourceCount = 9,
    Core3DDebugAuthoredBooleanFaultSiteTreatmentReader = 10,
    Core3DDebugAuthoredBooleanFaultSiteTreatmentPresence = 11,
    Core3DDebugAuthoredBooleanFaultSiteTreatmentBytes = 12,
    Core3DDebugAuthoredBooleanFaultSiteTreatmentBase = 13,
    Core3DDebugAuthoredBooleanFaultSiteTreatmentCurrentBinding = 14,
    Core3DDebugAuthoredBooleanFaultSiteWholeR2Validation = 15,
    Core3DDebugAuthoredBooleanFaultSiteOperandEqualityFirst = 16,
    Core3DDebugAuthoredBooleanFaultSiteOperandEqualitySecond = 17,
    Core3DDebugAuthoredBooleanFaultSiteAfterSideCapture = 18,
    Core3DDebugAuthoredBooleanFaultSiteAfterCaptureBeforePublish = 19,
};

typedef NS_ENUM(NSInteger, Core3DDebugAuthoredBooleanFaultDelivery) {
    Core3DDebugAuthoredBooleanFaultDeliveryReturnFalse = 0,
    Core3DDebugAuthoredBooleanFaultDeliveryThrowStandardFailure = 1,
};

//! Immutable one-shot hit receipt of an armed authored-Boolean stage fault.
__attribute__((objc_subclassing_restricted))
@interface Core3DDebugAuthoredBooleanFaultReceipt : NSObject
@property(nonatomic,readonly) Core3DDebugAuthoredBooleanFaultSite site;
@property(nonatomic,readonly) Core3DDebugAuthoredBooleanFaultDelivery delivery;
@property(nonatomic,readonly) NSUInteger hitCount;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! One exact current TNaming (labelEntry, version) row read from the live
//! document; never a companion's saved before-value.
__attribute__((objc_subclassing_restricted))
@interface Core3DDebugTreatmentNamingVersionRow : NSObject
@property(nonatomic,copy,readonly) NSString *labelEntry;
@property(nonatomic,readonly) NSInteger version;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Immutable read-only observation of one host document for the SAB-N1 fault
//! proof: the exact naming-version vector plus history and companion counters.
__attribute__((objc_subclassing_restricted))
@interface Core3DDebugAuthoredBooleanObservation : NSObject
@property(nonatomic,copy,readonly) NSArray<Core3DDebugTreatmentNamingVersionRow *> *namingVersions;
@property(nonatomic,readonly) NSInteger availableUndos;
@property(nonatomic,readonly) NSInteger availableRedos;
@property(nonatomic,readonly) BOOL hasOpenCommand;
@property(nonatomic,readonly) BOOL hasPendingCompanion;
@property(nonatomic,readonly) BOOL pendingCompanionAfterCaptured;
@property(nonatomic,readonly) NSUInteger finalizedCompanionCount;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end
#endif

@interface Core3DViewController (AuthoredParameterOpening)
//! Capture both current ordinary Profile roots at once in target-left/tool-
//! right order. This and prepare are read-only; creator apply is intentionally
//! absent until N-COM.
- (Core3DBooleanPairCaptureResult *)openBooleanSourcesWithTargetEntityIdentifier:
    (NSString *)targetEntityIdentifier toolEntityIdentifier:(NSString *)toolEntityIdentifier
    operation:(Core3DPartBooleanOperation)operation expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(openBooleanSources(targetEntityIdentifier:toolEntityIdentifier:operation:expected:));
- (Core3DBooleanPairPrepareResult *)prepareBooleanSources:
    (Core3DBooleanPairOpening *)opening expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(prepareBooleanSources(_:expected:));
//! Externally driven Stop for detached preparation. Once stopped, this exact
//! opening can never prepare again and cannot affect a later opening.
- (BOOL)cancelBooleanSources:(Core3DBooleanPairOpening *)opening
    NS_SWIFT_NAME(cancelBooleanSources(_:));
- (Core3DBooleanPairObservationResult *)observeBooleanSources:
    (Core3DBooleanPairOpening *)opening
    NS_SWIFT_NAME(observeBooleanSources(_:));
//! Consume only a native PairPrepared authority. The bridge supplies current
//! viewport fencing but no labels, recipe bytes or replacement originals.
- (Core3DBooleanPairApplyResult *)applyBooleanSources:
    (Core3DBooleanPairPrepared *)prepared expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(applyBooleanSources(_:expected:));
//! Open, prepare and apply one retained-v4 effective edit. The two opaque
//! wrappers are native-issued and cannot be reconstructed from observations.
- (Core3DBooleanPairCaptureResult *)openBooleanOriginalsEditorForEntityIdentifier:
    (NSString *)entityIdentifier expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(openBooleanOriginalsEditor(entityIdentifier:expected:));
- (Core3DBooleanPairPrepareResult *)prepareBooleanOriginalsEdit:
    (Core3DBooleanPairOpening *)opening edit:(Core3DBooleanOriginalsEdit *)edit
    expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(prepareBooleanOriginalsEdit(_:edit:expected:));
- (Core3DBooleanPairObservationResult *)observeBooleanOriginals:
    (Core3DBooleanPairOpening *)opening
    NS_SWIFT_NAME(observeBooleanOriginals(_:));
//! A native read-only capture of the current original/effective record. This
//! neither selects the owner nor issues an opening/prepared edit authority.
- (Core3DBooleanPairObservationResult *)captureBooleanOriginalsObservationForEntityIdentifier:
    (NSString *)entityIdentifier expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(captureBooleanOriginalsObservation(entityIdentifier:expected:));
- (Core3DBooleanPairApplyResult *)applyBooleanOriginalsEdit:
    (Core3DBooleanPairPrepared *)prepared expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(applyBooleanOriginalsEdit(_:expected:));
#if DEBUG
//! Test-only metadata producer applied after an ordinary Profile is created.
//! This never installs a Boolean/composite fixture.
//! Attach only a suffix to an already production-created v4 pair for replay
//! evidence. It never creates or substitutes a Boolean base or its originals.
- (NSInteger)debugAppendBooleanOriginalsSuffix:(NSString *)entityIdentifier
    expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(debugAppendBooleanOriginalsSuffix(entityIdentifier:expected:));
- (BOOL)debugInstallBooleanSourceHiddenLayer:(NSString *)entityIdentifier
    NS_SWIFT_NAME(debugInstallBooleanSourceHiddenLayer(entityIdentifier:));
//! Test-only occurrence encoding producer applied to one exact ordinary
//! Profile before pair capture. Tags are one-based transform scalar tags.
- (BOOL)debugInstallBooleanSourceOccurrenceEncoding:(NSString *)entityIdentifier
    absentTags:(NSArray<NSNumber *> *)absentTags
    negativeZeroTags:(NSArray<NSNumber *> *)negativeZeroTags
    NS_SWIFT_NAME(debugInstallBooleanSourceOccurrenceEncoding(entityIdentifier:absentTags:negativeZeroTags:));
//! Read-only exact source metadata used to prove admission and label aliasing.
- (NSDictionary<NSString *, id> *_Nullable)debugBooleanSourceMetadata:
    (NSArray<NSString *> *)entityIdentifiers
    NS_SWIFT_NAME(debugBooleanSourceMetadata(entityIdentifiers:));
- (NSInteger)debugInstallBooleanSourceMaterialCensus:(NSString *)entityIdentifier
    peer:(NSString *)peer variant:(NSInteger)variant
    NS_SWIFT_NAME(debugInstallBooleanSourceMaterialCensus(entityIdentifier:peer:variant:));
- (NSDictionary<NSString *, id> *_Nullable)debugBooleanOriginalsMaterialMetadata:
    (NSString *)entityIdentifier
    NS_SWIFT_NAME(debugBooleanOriginalsMaterialMetadata(entityIdentifier:));
//! Arm one exact prepared creation authority. The receipt stays live across
//! Apply so its hit count proves that the requested real site was reached.
- (Core3DDebugBooleanPairCreationFaultReceipt *_Nullable)
    debugArmBooleanPairCreationFault:(Core3DBooleanPairPrepared *)prepared
    site:(Core3DDebugBooleanPairCreationFaultSite)site
    NS_SWIFT_NAME(debugArmBooleanPairCreationFault(_:site:));
#endif
//! Capture requires the exact selected object and caller-observed scene. It is
//! read-only and refuses every unsupported carrier rather than dropping data.
- (Core3DAuthoredParameterOpening *_Nullable)openAuthoredParameterForEntityIdentifier:
    (NSString *)entityIdentifier expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(openAuthoredParameter(entityIdentifier:expected:));
//! Read the complete analytic Boolean value carried by this exact opening.
//! Descriptive values do not carry authority; the opening remains required for
//! Apply and is revalidated against the owning host, scene and selection here.
- (Core3DPartBooleanValues *_Nullable)describeAuthoredBooleanParameterOpening:
    (Core3DAuthoredParameterOpening *)opening expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(describeAuthoredBooleanParameter(_:expected:));
//! Host-mediated prepare prevents a handle issued by another live document
//! from being used. N0 has no registered capability, so all well-formed kinds
//! return unsupportedCapability without opening a transaction.
- (Core3DAuthoredParameterPreparationResult)prepareAuthoredParameterOpening:
    (Core3DAuthoredParameterOpening *)opening
    mutationKind:(Core3DAuthoredParameterMutationKind)mutationKind
    NS_SWIFT_NAME(prepareAuthoredParameter(_:mutationKind:));
- (BOOL)cancelAuthoredParameterOpening:(Core3DAuthoredParameterOpening *)opening
    NS_SWIFT_NAME(cancelAuthoredParameter(_:));
//! Replace exactly one declared authored rectangular-loft value. Station fields
//! require the captured station ID; frame fields require stationIdentifier zero.
//! All other station values, IDs, correspondence, units and suffix identities
//! are verified natively before the existing counted source/suffix replay runs.
- (Core3DAuthoredParameterOperation *)applyAuthoredLoftParameterOpening:
    (Core3DAuthoredParameterOpening *)opening
    stationIdentifier:(uint32_t)stationIdentifier
    field:(Core3DAuthoredLoftField)field
    definition:(Core3DRectangularLoftDefinition *)definition
    expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DAuthoredParameterApplyResult *result))completion
    NS_SWIFT_NAME(applyAuthoredLoftParameter(_:stationIdentifier:field:definition:expected:completion:));
//! Replace exactly one analytic-prism dimension set, the Boolean operation,
//! or one operand placement. The complete requested values are descriptive;
//! the opening retains identities, metadata, suffix and mutation authority.
- (Core3DAuthoredParameterOperation *)applyAuthoredBooleanParameterOpening:
    (Core3DAuthoredParameterOpening *)opening
    mutationKind:(Core3DAuthoredParameterMutationKind)mutationKind
    inputIndex:(NSUInteger)inputIndex
    values:(Core3DPartBooleanValues *)values
    expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DAuthoredParameterApplyResult *result))completion
    NS_SWIFT_NAME(applyAuthoredBooleanParameter(_:mutationKind:inputIndex:values:expected:completion:));
//! Continue the one outstanding authored-Boolean recovery owned by this host's
//! current native document. This does not capture a scene or create an opening.
- (Core3DAuthoredParameterRecoveryResult *)recoverAuthoredBooleanParameter
    NS_SWIFT_NAME(recoverAuthoredBooleanParameter());
#if DEBUG
//! Arms the existing close-unproven hook through this host's exact current
//! authored opening. No transaction, outcome or recovery state is fabricated.
- (BOOL)debugReportNextAuthoredParameterCloseUnproven:
    (Core3DAuthoredParameterOpening *)opening
    NS_SWIFT_NAME(debugReportNextAuthoredParameterCloseUnproven(_:));
//! SAB-N1: arms the one-shot post-write stage fault through this host's exact
//! current authored opening. Foreign, stale, cancelled, recovery, repeated and
//! invalid-site requests return NO; no transaction, outcome or recovery state
//! is fabricated and the armed state is cleared on consumption, cancel or
//! disposal.
- (BOOL)debugArmAuthoredBooleanStageFault:(Core3DAuthoredParameterOpening *)opening
    site:(Core3DDebugAuthoredBooleanFaultSite)site
    delivery:(Core3DDebugAuthoredBooleanFaultDelivery)delivery
    NS_SWIFT_NAME(debugArmAuthoredBooleanStageFault(_:site:delivery:));
//! Read-only hit receipt for the fault armed on this exact opening; nil when
//! no fault was armed or the armed state was cleared by cancel or disposal.
- (Core3DDebugAuthoredBooleanFaultReceipt *_Nullable)debugAuthoredBooleanStageFaultReceipt:
    (Core3DAuthoredParameterOpening *)opening
    NS_SWIFT_NAME(debugAuthoredBooleanStageFaultReceipt(_:));
//! Read-only observation of this host's document for one entity: exact current
//! TNaming (labelEntry, version) rows plus undo/redo depths, open-command
//! state and the companion census. Not bound to any live opening, because an
//! abort may stale the issuer; nil when the observation itself fails.
- (Core3DDebugAuthoredBooleanObservation *_Nullable)debugObserveAuthoredBooleanStateForEntityIdentifier:
    (NSString *)entityIdentifier
    NS_SWIFT_NAME(debugObserveAuthoredBooleanState(entityIdentifier:));
#endif
@end

NS_ASSUME_NONNULL_END
