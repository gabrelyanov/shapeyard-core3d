// PC-D4-C production feature-pattern creator.
#import <Core3D/Core3DModelingTypes.h>
#import <Core3D/Core3DViewController.h>

NS_ASSUME_NONNULL_BEGIN

__attribute__((objc_subclassing_restricted))
@interface Core3DFeaturePatternCreationOpening : NSObject
//! Host identity, document units and native-discovered source/step choices.
@property(nonatomic,copy,readonly) NSDictionary<NSString *, id> *descriptor;
//! Truthful immutable projection retained by the one native preparation.
@property(nonatomic,copy,readonly) NSDictionary<NSString *, id> *review;
- (void)prepareCandidate:(NSDictionary<NSString *, id> *)candidate
    completion:(void (^)(Core3DBoundedCurvePreparationResult result,
                          NSString *detail))completion
    NS_SWIFT_NAME(prepare(candidate:completion:));
- (void)applyWithCompletion:
    (void (^)(Core3DProfileConstructionResult result, NSString *detail,
              NSString *_Nullable hostEntityIdentifier))completion
    NS_SWIFT_NAME(apply(completion:));
- (BOOL)cancel;
#if DEBUG
- (void)debugArmCreationFault:(NSInteger)fault
    NS_SWIFT_NAME(debugArmCreationFault(_:));
#endif
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DViewController (FeaturePatternCreationOpening)
//! Captures exactly one selected live Profile host. Source carriers and stable
//! Cylinder Difference steps are discovered from current same-document records.
- (Core3DFeaturePatternCreationOpening *_Nullable)beginFeaturePatternCreation
    NS_SWIFT_NAME(beginFeaturePatternCreation());
#if DEBUG
- (NSDictionary<NSString *, id> *_Nullable)debugFeaturePatternCreationEvidenceForEntityIdentifier:
    (NSString *)entityIdentifier
    NS_SWIFT_NAME(debugFeaturePatternCreationEvidence(entityIdentifier:));
#endif
@end

NS_ASSUME_NONNULL_END
