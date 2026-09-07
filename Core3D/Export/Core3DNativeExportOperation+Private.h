#import "Core3DNativeExportOperation.h"
#import "ExportType.h"
#include "../Scene/OcctSceneSnapshotBuilder.hpp"

NS_ASSUME_NONNULL_BEGIN

@interface Core3DNativeExportOperation (Private)

- (instancetype)initWithSnapshotURL:(NSURL *)snapshotURL
                 snapshotCleanupURL:(NSURL *)snapshotCleanupURL
                      packageRootURL:(NSURL *)packageRootURL
                          cleanupURL:(NSURL *)cleanupURL
                          exportType:(ExportType)exportType
           selectedEntityIdentifiers:(NSArray<NSString *> *_Nullable)selectedEntityIdentifiers
                         meshQuality:(Core3DExportMeshQuality)meshQuality
                  objColorConvention:(Core3DOBJColorConvention)objColorConvention
                      deflectionType:(NSInteger)deflectionType
                deviationCoefficient:(double)deviationCoefficient
                       deviationAngle:(double)deviationAngle
            maximalChordialDeviation:(double)maximalChordialDeviation
                         sourceScene:(core3d::scene::OcctSceneSnapshotBuilder::SnapshotPointer)sourceScene
                 selectedObjectsOnly:(BOOL)selectedObjectsOnly;

@end

NS_ASSUME_NONNULL_END
