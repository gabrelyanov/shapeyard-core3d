#import "Core3DEdgeTreatmentR2Types.h"
#include "../OCCTKit/RetainedEdgeTreatmentR2Snapshot.hxx"

@implementation Core3DRetainedBooleanRecipeLocatorR2 @end
@implementation Core3DRetainedBooleanAnalyticToolR2 @end
@implementation Core3DRetainedBooleanInputPlacementR2 @end
@implementation Core3DRetainedBooleanInputR2 @end
@implementation Core3DRetainedBooleanOperationR2 @end
@implementation Core3DRetainedBooleanMigrationStepMapR2 @end
@implementation Core3DRetainedBooleanMigrationAnchorMapR2 @end
@implementation Core3DRetainedBooleanSourceR2 @end
@implementation Core3DEdgeTreatmentSnapshotR2 @end
@implementation Core3DEdgeTreatmentCaptureR2 @end
@implementation Core3DRetainedBooleanMigrationCaptureR2 @end
@implementation Core3DFaceSelectorProofR2 @end
@implementation Core3DFaceSelectorQueryR2 @end
@implementation Core3DRetainedBooleanMigrationProofR2 @end
@implementation Core3DRetainedBooleanMigrationReviewR2 @end
@implementation Core3DRetainedBooleanEditR2 @end

@implementation Core3DRetainedBooleanLegacySelectorBindingR2
- (nullable instancetype)initWithOldStepID:(uint64_t)oldStepID intent:(Core3DFaceSelectorIntent *)intent {
    if (!oldStepID || !intent) return nil;
    if ((self = [super init])) { _oldStepID = oldStepID; _intent = intent; }
    return self;
}
@end
@implementation Core3DRetainedBooleanSelectorAppendR2
- (nullable instancetype)initWithIntent:(Core3DFaceSelectorIntent *)intent amountMM:(double)amountMM {
    if (!intent || !isfinite(amountMM) || amountMM <= 0 || amountMM > 20) return nil;
    if ((self = [super init])) { _intent = intent; _amountMM = amountMM; }
    return self;
}
@end
@implementation Core3DRetainedBooleanMigrationRequestR2
- (nullable instancetype)initWithSelectors:(NSArray<Core3DRetainedBooleanLegacySelectorBindingR2 *> *)selectors
    append:(Core3DRetainedBooleanSelectorAppendR2 *)append {
    if (selectors.count > 8) return nil;
    uint64_t previous = 0;
    for (Core3DRetainedBooleanLegacySelectorBindingR2 *binding in selectors) {
        if (binding.oldStepID <= previous) return nil;
        previous = binding.oldStepID;
    }
    if ((self = [super init])) { _version = 1; _selectors = [selectors copy]; _append = append; }
    return self;
}
@end

// Native-only subclasses in Core3DViewController.mm retain the corresponding
// r2 Snapshot/MigrationCapture/Review/SelectorTargetCapture.  No public DTO
// initializer can mint those proof-bearing values.
