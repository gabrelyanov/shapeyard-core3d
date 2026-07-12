#import "Core3DNativeExportOperation.h"

NS_ASSUME_NONNULL_BEGIN

@interface Core3DNativeExportOperation (Private)

- (instancetype)initWithSnapshotURL:(NSURL *)snapshotURL
                 snapshotCleanupURL:(NSURL *)snapshotCleanupURL
                      packageRootURL:(NSURL *)packageRootURL
                          cleanupURL:(NSURL *)cleanupURL
                      deflectionType:(NSInteger)deflectionType
                deviationCoefficient:(double)deviationCoefficient
                       deviationAngle:(double)deviationAngle
            maximalChordialDeviation:(double)maximalChordialDeviation;

@end

NS_ASSUME_NONNULL_END
