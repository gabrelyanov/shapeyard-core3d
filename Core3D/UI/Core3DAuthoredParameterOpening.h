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

#if DEBUG
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
