#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@class Core3DPublishCapture;

FOUNDATION_EXPORT NSErrorDomain const Core3DDecimationErrorDomain;

typedef NS_ERROR_ENUM(Core3DDecimationErrorDomain, Core3DDecimationError) {
    Core3DDecimationErrorTargetBelowClosedMeshMinimum = 1,
    Core3DDecimationErrorInvalidTopology,
    Core3DDecimationErrorNonFiniteInput,
    Core3DDecimationErrorResourceLimit,
    Core3DDecimationErrorCancelled,
    Core3DDecimationErrorUnreachableTriangleBudget,
    Core3DDecimationErrorStaleSource,
    Core3DDecimationErrorMalformedRequest,
};

//! Immutable POSITION-only input to the native geometry kernel. Values are
//! already in right-handed Z-up world millimetres, as independently observed
//! from a completed GLB. The source artifact and settings digests bind those
//! bytes; analytic contribution tokens arrive in exact emitted-triangle order.
__attribute__((objc_subclassing_restricted))
@interface Core3DDecimationRequest : NSObject
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *positionsMM;
@property(nonatomic, copy, readonly) NSArray<NSNumber *> *indices;
@property(nonatomic, copy, readonly) NSArray<NSString *> *sourceContributionTokens;
@property(nonatomic, copy, readonly) NSData *materialBytes;
@property(nonatomic, copy, readonly) NSString *sourceArtifactSHA256;
@property(nonatomic, copy, readonly) NSString *settingsSHA256;
@property(nonatomic, assign, readonly) NSUInteger targetTriangleCount;
@property(nonatomic, assign, readonly) double metersPerUnit;
- (instancetype)initWithPositionsMM:(NSArray<NSNumber *> *)positionsMM
                             indices:(NSArray<NSNumber *> *)indices
            sourceContributionTokens:(NSArray<NSString *> *)sourceContributionTokens
                       materialBytes:(NSData *)materialBytes
                sourceArtifactSHA256:(NSString *)sourceArtifactSHA256
                      settingsSHA256:(NSString *)settingsSHA256
                 targetTriangleCount:(NSUInteger)targetTriangleCount
                       metersPerUnit:(double)metersPerUnit NS_DESIGNATED_INITIALIZER;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Detached candidate bytes. Positions and indices are explicitly measured
//! again by the caller before admission. NORMAL is f2-normal-regen.v1 flat
//! per-corner data; materialBytes is an exact copy of the sole source domain.
__attribute__((objc_subclassing_restricted))
@interface Core3DDecimationResult : NSObject
@property(nonatomic, copy, readonly) NSData *positionFloat32Data;
@property(nonatomic, copy, readonly) NSData *normalFloat32Data;
@property(nonatomic, copy, readonly) NSData *indexUInt32Data;
@property(nonatomic, copy, readonly) NSData *materialBytes;
@property(nonatomic, copy, readonly) NSArray<NSArray<NSString *> *> *sourceContributionTokens;
@property(nonatomic, copy, readonly) NSString *normalPolicy;
@property(nonatomic, copy, readonly) NSString *appearanceLimitation;
@property(nonatomic, assign, readonly) NSUInteger triangleCount;
@property(nonatomic, assign, readonly) NSUInteger acceptedCollapseCount;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

//! Request-owned, read-only opening. When constructed with a publish capture,
//! the complete native source fence is checked before and after mesh work. A
//! nil capture is reserved for literal native kernel vectors and cannot confer
//! Current status on a document derivative.
__attribute__((objc_subclassing_restricted))
@interface Core3DDecimationOpening : NSObject
+ (nullable instancetype)openingWithCapture:(Core3DPublishCapture * _Nullable)capture
                                      error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(init(capture:));
- (nullable Core3DDecimationResult *)decimateRequest:(Core3DDecimationRequest *)request
                                                error:(NSError * _Nullable * _Nullable)error
    NS_SWIFT_NAME(decimate(_:));
//! Sticky for this opening and never affects another request.
- (BOOL)cancel;
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
@end

NS_ASSUME_NONNULL_END
