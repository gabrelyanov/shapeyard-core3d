//
//  Core3DPatternCreationOpening.h
//  Core3D
//
//  PC-D2-C production D2 retained-pattern creator. One new adapter header
//  carrying both the opaque opening type and the Core3DViewController
//  registration category, so the edit to Core3DViewController.h is one line.
//

#import <Core3D/Core3DModelingTypes.h>
#import <Core3D/Core3DViewController.h>

NS_ASSUME_NONNULL_BEGIN

typedef NS_ENUM(NSInteger, Core3DPatternEditParameterRefusal) {
    Core3DPatternEditParameterRefusalNone = 0,
    Core3DPatternEditParameterRefusalInvalidDescriptor,
    Core3DPatternEditParameterRefusalInvalidRowAxis,
    Core3DPatternEditParameterRefusalInvalidColumnAxis,
    Core3DPatternEditParameterRefusalDuplicateAxes,
    Core3DPatternEditParameterRefusalRowCountRequiresOne,
};

//! Read-only, typed diagnosis of the axis and non-grid row-count parameters.
//! `None` means only that these parameters passed their checks; the existing
//! apply method remains the sole full-admission and mutation route.
@interface Core3DPatternEditingOpening (PatternParameterPreflight)
- (Core3DPatternEditParameterRefusal)parameterRefusalForCandidate:
    (NSDictionary<NSString *, id> *)candidate
    NS_SWIFT_NAME(parameterRefusal(candidate:));
@end

//! One native D2 retained-pattern creation opening for ordinary authoring.
//! Capture freezes exactly one admissible selected source object from the live
//! document. `descriptor` projects the creation context (document identity,
//! units, source identity/family and the admission budgets). A candidate
//! carries explicit construction values only — kind, axes, counts, spacings,
//! pivot and sweep. Every durable identity (pattern owner/feature, member
//! entity/definition and recipe feature UUIDs) is minted natively through the
//! document's own issuance seams and is never accepted from the candidate.
//! prepare runs the full admission and budget projection without mutation;
//! apply commits the whole pattern as exactly one history command; cancel
//! leaves no record, no shapes and no history. Single use: a consumed,
//! cancelled or settled opening refuses every later call.
__attribute__((objc_subclassing_restricted))
@interface Core3DPatternCreationOpening : NSObject
@property(nonatomic,copy,readonly) NSDictionary<NSString *, id> *descriptor;
@property(nonatomic,readonly) NSInteger projectedInstances;
@property(nonatomic,readonly) NSInteger projectedDocumentBytes;
@property(nonatomic,readonly) NSInteger projectedMemoryBytes;
@property(nonatomic,readonly) NSInteger projectedTopologyNodes;
- (void)prepareCandidate:(NSDictionary<NSString *, id> *)candidate
    completion:(void (^)(Core3DBoundedCurvePreparationResult result,
                          NSString *detail))completion
    NS_SWIFT_NAME(prepare(candidate:completion:));
- (void)applyWithCompletion:
    (void (^)(Core3DProfileConstructionResult result, NSString *detail,
              NSString *_Nullable ownerEntityIdentifier))completion
    NS_SWIFT_NAME(apply(completion:));
- (BOOL)cancel;
#if DEBUG
//! DEBUG-only atomicity evidence: the next apply stages every member and then
//! fails the created member at `index` inside the command, forcing the real
//! abort path. No effect outside DEBUG or outside the Prepared state.
- (void)debugInjectStagingFailureAtMemberIndex:(NSInteger)index
    NS_SWIFT_NAME(debugInjectStagingFailure(memberIndex:));
#endif
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

@interface Core3DViewController (PatternCreationOpening)
//! Capture one fail-closed D2 creation opening from the exact current
//! whole-object selection. Main-thread only; nil never changes model/history.
//! The source must be one admissible free object that is not already retained
//! by an existing pattern or path array.
- (Core3DPatternCreationOpening *_Nullable)beginPatternCreation
    NS_SWIFT_NAME(beginPatternCreation());
#if DEBUG
//! Read-only D2 creation evidence. Reads every retained-pattern record and
//! selects strictly by the given owner/feature/member identity — never by
//! record count or ordering — then re-reads every actual member label and
//! recipe from the live document. Compiled out of Release.
- (NSDictionary<NSString *, id> *_Nullable)debugPatternCreationEvidenceForEntityIdentifier:(NSString *)entityIdentifier
    NS_SWIFT_NAME(debugPatternCreationEvidence(entityIdentifier:));
//! Complete identity-keyed readback for D2 axis edit evidence. Full canonical
//! record, shape and family-recipe encodings are retained for unit tests.
- (NSDictionary<NSString *, id> *_Nullable)debugPatternAxesEvidenceForEntityIdentifier:(NSString *)entityIdentifier
    NS_SWIFT_NAME(debugPatternAxesEvidence(entityIdentifier:));
//! Pure paired PrepareNative dumps for the inactive fields of the captured
//! pattern kind. No command is opened and every suppressed member is included.
- (NSDictionary<NSString *, id> *_Nullable)debugPatternAxesInertnessDumpForEntityIdentifier:(NSString *)entityIdentifier
    NS_SWIFT_NAME(debugPatternAxesInertnessDump(entityIdentifier:));
#endif
@end

NS_ASSUME_NONNULL_END
