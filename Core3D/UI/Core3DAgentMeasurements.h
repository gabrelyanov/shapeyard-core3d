#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

typedef NS_ENUM(NSInteger, Core3DAgentMeasurementStatus) {
    Core3DAgentMeasurementStatusMeasured = 0,
    Core3DAgentMeasurementStatusUnavailable,
    Core3DAgentMeasurementStatusAmbiguous,
    Core3DAgentMeasurementStatusStale,
    Core3DAgentMeasurementStatusBudgetExceeded,
    Core3DAgentMeasurementStatusNotQualified,
};

/// Immutable copied-value source fence. It contains no native handle, edit
/// lease, selection token or history capability.
@interface Core3DAgentMeasurementSource : NSObject
@property(nonatomic, readonly, copy) NSString *targetAlias;
@property(nonatomic, readonly, copy) NSString *documentIdentifier;
@property(nonatomic, readonly, copy) NSString *publicationSourceIdentifier;
@property(nonatomic, readonly, copy) NSString *documentGeneration;
@property(nonatomic, readonly, copy) NSString *modelRevision;
@property(nonatomic, readonly, copy) NSString *presentationRevision;
@property(nonatomic, readonly) double metersPerUnit;
@property(nonatomic, readonly, copy) NSString *method;
@property(nonatomic, readonly, copy) NSString *sourceSHA256;
- (nullable instancetype)initWithTargetAlias:(NSString *)targetAlias
                          documentIdentifier:(NSString *)documentIdentifier
                 publicationSourceIdentifier:(NSString *)publicationSourceIdentifier
                          documentGeneration:(NSString *)documentGeneration
                                modelRevision:(NSString *)modelRevision
                         presentationRevision:(NSString *)presentationRevision
                              metersPerUnit:(double)metersPerUnit
                                       method:(NSString *)method
                                 sourceSHA256:(NSString *)sourceSHA256 NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;
@end

@interface Core3DAgentSolidMeasurement : NSObject
@property(nonatomic, readonly) Core3DAgentMeasurementStatus status;
@property(nonatomic, readonly, strong, nullable) Core3DAgentMeasurementSource *source;
@property(nonatomic, readonly, copy) NSArray<NSNumber *> *boundsMinMM;
@property(nonatomic, readonly, copy) NSArray<NSNumber *> *boundsMaxMM;
@property(nonatomic, readonly, copy) NSArray<NSNumber *> *localBoundsOptimalMinMM;
@property(nonatomic, readonly, copy) NSArray<NSNumber *> *localBoundsOptimalMaxMM;
@property(nonatomic, readonly) double volumeMM3;
@property(nonatomic, readonly) double volumeIntegrationRelativeError;
@property(nonatomic, readonly) BOOL valid;
@property(nonatomic, readonly) NSUInteger solids;
@property(nonatomic, readonly) NSUInteger shells;
@property(nonatomic, readonly) NSUInteger faces;
@property(nonatomic, readonly) NSUInteger edges;
@property(nonatomic, readonly) NSUInteger vertices;
@property(nonatomic, readonly) NSUInteger topologyNodes;
@end

@interface Core3DAgentLineSectionMeasurement : NSObject
@property(nonatomic, readonly) Core3DAgentMeasurementStatus status;
@property(nonatomic, readonly, strong, nullable) Core3DAgentMeasurementSource *source;
@property(nonatomic, readonly, copy) NSString *probeIdentifier;
@property(nonatomic, readonly, copy) NSArray<NSNumber *> *originMM;
@property(nonatomic, readonly, copy) NSArray<NSNumber *> *requestedDirection;
/// Ordered [enterMM, exitMM] pairs. An empty array is a successful measured
/// no-material result; unavailable is represented by status, never by [].
@property(nonatomic, readonly, copy) NSArray<NSArray<NSNumber *> *> *intervalsMM;
@end

@interface Core3DAgentMaterialMeasurement : NSObject
@property(nonatomic, readonly) Core3DAgentMeasurementStatus status;
@property(nonatomic, readonly, strong, nullable) Core3DAgentMeasurementSource *source;
@property(nonatomic, readonly, copy) NSArray<NSNumber *> *linearBaseColorRGBA;
@property(nonatomic, readonly) double metallic;
@property(nonatomic, readonly) double roughness;
@end

@interface Core3DAgentWorldPlacementMeasurement : NSObject
@property(nonatomic, readonly) Core3DAgentMeasurementStatus status;
@property(nonatomic, readonly, strong, nullable) Core3DAgentMeasurementSource *source;
@property(nonatomic, readonly, copy) NSArray<NSNumber *> *matrixColumnMajor;
@end

/// Complete passive state evidence payload. Each value is the canonical native
/// payload for one frozen commitment family, not a derived summary.
@interface Core3DAgentStateCapture : NSObject
@property(nonatomic, readonly, copy) NSString *documentIdentifier;
@property(nonatomic, readonly, copy) NSString *documentGeneration;
@property(nonatomic, readonly, copy) NSString *modelRevision;
@property(nonatomic, readonly, copy) NSString *presentationRevision;
@property(nonatomic, readonly) double metersPerUnit;
@property(nonatomic, readonly) NSInteger undoDepth;
@property(nonatomic, readonly) NSInteger redoDepth;
@property(nonatomic, readonly, copy) NSDictionary<NSString *, NSData *> *commitmentPayloads;
- (nullable instancetype)initWithDocumentIdentifier:(NSString *)documentIdentifier
                                 documentGeneration:(NSString *)documentGeneration
                                       modelRevision:(NSString *)modelRevision
                                presentationRevision:(NSString *)presentationRevision
                                     metersPerUnit:(double)metersPerUnit
                                           undoDepth:(NSInteger)undoDepth
                                           redoDepth:(NSInteger)redoDepth
                                  commitmentPayloads:(NSDictionary<NSString *, NSData *> *)commitmentPayloads NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;
@end

@interface Core3DAgentMeasurements : NSObject
/// D51.3/D52(c): remains NO until the source-bound complete native guard passes.
@property(class, nonatomic, readonly) BOOL observedUnchangedQualified;
@property(class, nonatomic, readonly, copy) NSArray<NSString *> *commitmentKeys;
@end

NS_ASSUME_NONNULL_END
