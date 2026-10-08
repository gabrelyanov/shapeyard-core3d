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

@interface Core3DViewController (AuthoredParameterOpening)
//! Capture requires the exact selected object and caller-observed scene. It is
//! read-only and refuses every unsupported carrier rather than dropping data.
- (Core3DAuthoredParameterOpening *_Nullable)openAuthoredParameterForEntityIdentifier:
    (NSString *)entityIdentifier expected:(Core3DSceneSnapshot *)expected
    NS_SWIFT_NAME(openAuthoredParameter(entityIdentifier:expected:));
//! Host-mediated prepare prevents a handle issued by another live document
//! from being used. N0 has no registered capability, so all well-formed kinds
//! return unsupportedCapability without opening a transaction.
- (Core3DAuthoredParameterPreparationResult)prepareAuthoredParameterOpening:
    (Core3DAuthoredParameterOpening *)opening
    mutationKind:(Core3DAuthoredParameterMutationKind)mutationKind
    NS_SWIFT_NAME(prepareAuthoredParameter(_:mutationKind:));
- (BOOL)cancelAuthoredParameterOpening:(Core3DAuthoredParameterOpening *)opening
    NS_SWIFT_NAME(cancelAuthoredParameter(_:));
@end

NS_ASSUME_NONNULL_END
