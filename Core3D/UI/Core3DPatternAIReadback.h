// DEBUG-only actual-label retained-pattern observation.
#import <Core3D/Core3DViewController.h>

NS_ASSUME_NONNULL_BEGIN
#if DEBUG
@interface Core3DViewController (PatternAIReadback)
- (NSDictionary<NSString *, id> *_Nullable)debugPatternAIReadback:
    (NSString *)entityIdentifier
    NS_SWIFT_NAME(debugPatternAIReadback(entityIdentifier:));
@end
#endif
NS_ASSUME_NONNULL_END
