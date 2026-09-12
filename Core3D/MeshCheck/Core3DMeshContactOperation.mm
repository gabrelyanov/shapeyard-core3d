#import "Core3DMeshContactOperation+Private.h"
#include <mutex>

using namespace core3d::meshcheck;
namespace {
// A reservation follows the captured payload, including a completed report's
// owner record. Finishing a worker alone must not release the memory budget.
std::atomic_size_t reservedMeshContactCaptures{0};
struct MeshContactReservation {
    bool acquired=false;
    ~MeshContactReservation() {
        if(acquired) reservedMeshContactCaptures.fetch_sub(1,std::memory_order_acq_rel);
    }
};
struct MeshContactJob {
    ContactSourceIdentity identity;
    std::shared_ptr<const ContactSourceCapture> source;
    ContactReport result;
    Core3DMeshContactState state=Core3DMeshContactStateInternalFailure;
    std::atomic_bool cancelled{false};
    std::mutex completionMutex;
    bool finished=false;
    std::shared_ptr<void> reservation;
    std::chrono::steady_clock::time_point started=std::chrono::steady_clock::now();
};
Core3DMeshContactState PublicState(ContactSourceStatus status) {
    switch(status) {
        case ContactSourceStatus::Ready: return Core3DMeshContactStateComplete;
        case ContactSourceStatus::Unsupported: return Core3DMeshContactStateUnsupported;
        case ContactSourceStatus::InvalidGeometry: return Core3DMeshContactStateInvalidGeometry;
        case ContactSourceStatus::ResourceLimit: return Core3DMeshContactStateResourceLimit;
        case ContactSourceStatus::Cancelled: return Core3DMeshContactStateCancelled;
        case ContactSourceStatus::TimedOut: return Core3DMeshContactStateTimedOut;
        case ContactSourceStatus::StaleSource: return Core3DMeshContactStateStaleSource;
        case ContactSourceStatus::InternalFailure: return Core3DMeshContactStateInternalFailure;
    }
    return Core3DMeshContactStateInternalFailure;
}
Core3DMeshContactState PublicState(ContactStatus status) {
    switch(status) {
        case ContactStatus::Ready: return Core3DMeshContactStateComplete;
        case ContactStatus::Invalid: return Core3DMeshContactStateInvalidGeometry;
        case ContactStatus::TooLarge: return Core3DMeshContactStateResourceLimit;
        case ContactStatus::Cancelled: return Core3DMeshContactStateCancelled;
        case ContactStatus::TimedOut: return Core3DMeshContactStateTimedOut;
    }
    return Core3DMeshContactStateInternalFailure;
}
NSString *String(const std::string& value) {
    return [[NSString alloc] initWithBytes:value.data() length:value.size()
                                encoding:NSUTF8StringEncoding] ?: @"";
}
NSOperationQueue *MeshContactQueue() {
    static NSOperationQueue *queue;
    static dispatch_once_t once;
    dispatch_once(&once, ^{
        queue=[[NSOperationQueue alloc] init];
        queue.name=@"app.shapeyard.core3d.mesh-contact";
        queue.qualityOfService=NSQualityOfServiceUserInitiated;
        queue.maxConcurrentOperationCount=2;
    });
    return queue;
}
}

@interface Core3DMeshContactPair ()
- (instancetype)initWithFirst:(NSUInteger)first second:(NSUInteger)second;
@end
@implementation Core3DMeshContactPair
+ (BOOL)accessInstanceVariablesDirectly { return NO; }
- (instancetype)initWithFirst:(NSUInteger)first second:(NSUInteger)second {
    self=[super init];
    if(self) { _firstTriangle=first;_secondTriangle=second; }
    return self;
}
@end

@interface Core3DMeshContactReport ()
- (instancetype)initWithJob:(const MeshContactJob&)job ownerToken:(NSObject *)token;
@end
@implementation Core3DMeshContactReport {
    NSObject *_ownerToken;
    void (^_releaseHandler)(void);
}
+ (BOOL)accessInstanceVariablesDirectly { return NO; }
- (NSObject *)ownerToken { return _ownerToken; }
- (void)core3d_setReleaseHandler:(void (^)(void))handler {
    _releaseHandler=[handler copy];
}
- (void)dealloc {
    if(_releaseHandler) _releaseHandler();
}
- (instancetype)initWithJob:(const MeshContactJob&)job ownerToken:(NSObject *)token {
    self=[super init];
    if(self) {
        _state=job.state;_ownerToken=token;
        _entityIdentifier=String(job.identity.entityIdentifier);
        _definitionIdentifier=String(job.identity.definitionIdentifier);
        _publicationSourceIdentifier=String(job.identity.publicationSourceIdentifier);
        _documentGeneration=job.identity.documentGeneration;
        _modelRevision=job.identity.modelRevision;
        _geometryRevision=job.identity.geometryRevision;
        _elapsedSeconds=std::chrono::duration<double>(
            std::chrono::steady_clock::now()-job.started).count();
        _unexpectedPairs=@[];
        if(_state==Core3DMeshContactStateComplete) {
            _triangleCount=job.result.triangleCount;
            _candidatePairCount=job.result.candidatePairs;
            NSMutableArray<Core3DMeshContactPair *> *pairs=
                [[NSMutableArray alloc] initWithCapacity:job.result.unexpectedPairs.size()];
            for(const auto& pair:job.result.unexpectedPairs)
                [pairs addObject:[[Core3DMeshContactPair alloc]
                    initWithFirst:pair.first second:pair.second]];
            _unexpectedPairs=[pairs copy];
        }
    }
    return self;
}
@end

@interface Core3DMeshContactOperation ()
- (void)prepareOnMain;
- (void)deliverOnMain;
@end
@implementation Core3DMeshContactOperation {
    std::shared_ptr<MeshContactJob> _job;
    NSObject *_ownerToken;
    Core3DMeshContactPrepare _prepare;
    Core3DMeshContactValidate _validate;
    Core3DMeshContactCompletion _completion;
#if DEBUG
    void (^_afterCaptureHook)(void);
    void (^_beforeDeliveryHook)(void);
#endif
}
#if DEBUG
- (void)core3d_setAfterCaptureHook:(void (^)(void))afterCapture
              beforeDeliveryHook:(void (^)(void))beforeDelivery {
    NSAssert(NSThread.isMainThread,@"Contact test hooks require the owner thread");
    _afterCaptureHook=[afterCapture copy];_beforeDeliveryHook=[beforeDelivery copy];
}
#endif
- (instancetype)initWithIdentity:(const ContactSourceIdentity&)identity
                      ownerToken:(NSObject *)ownerToken
                         prepare:(Core3DMeshContactPrepare)prepare
                        validate:(Core3DMeshContactValidate)validate
                      completion:(Core3DMeshContactCompletion)completion {
    self=[super init];
    if(self) {
        _job=std::make_shared<MeshContactJob>();_job->identity=identity;
        _ownerToken=ownerToken;_prepare=[prepare copy];
        _validate=[validate copy];_completion=[completion copy];
        // Even a pre-cancelled/invalid request completes asynchronously. This
        // gives the caller ownership of the handle before any callback runs.
        dispatch_async(dispatch_get_main_queue(), ^{ [self prepareOnMain]; });
    }
    return self;
}
- (BOOL)isCancelled { return _job->cancelled.load(std::memory_order_acquire); }
- (void)cancel {
    std::lock_guard<std::mutex> lock(_job->completionMutex);
    if(!_job->finished) _job->cancelled.store(true,std::memory_order_release);
}
- (void)prepareOnMain {
    NSAssert(NSThread.isMainThread,@"Native contact capture requires its document owner");
    if(_job->cancelled.load(std::memory_order_acquire)) {
        _job->state=Core3DMeshContactStateCancelled;
        [self deliverOnMain];return;
    }
    try {
        auto reservation=std::make_shared<MeshContactReservation>();
        auto active=reservedMeshContactCaptures.load(std::memory_order_acquire);
        while(active<4 && !reservation->acquired) {
            reservation->acquired=reservedMeshContactCaptures.compare_exchange_weak(
                active,active+1,std::memory_order_acq_rel);
        }
        if(!reservation->acquired) {
            _job->state=Core3DMeshContactStateResourceLimit;
            [self deliverOnMain];return;
        }
        _job->reservation=std::move(reservation);
        const auto status=_prepare ? _prepare(_job->cancelled,_job->reservation,_job->source)
                                   : ContactSourceStatus::InternalFailure;
        _job->state=PublicState(status);
        if(status==ContactSourceStatus::Ready && !_job->source)
            _job->state=Core3DMeshContactStateInternalFailure;
    } catch(const std::bad_alloc&) { _job->state=Core3DMeshContactStateResourceLimit; }
      catch(...) { _job->state=Core3DMeshContactStateInternalFailure; }
    _prepare=nil;
    if(_job->state!=Core3DMeshContactStateComplete) {
        [self deliverOnMain];return;
    }
#if DEBUG
    void (^afterCapture)(void)=_afterCaptureHook;_afterCaptureHook=nil;
    if(afterCapture) afterCapture();
#endif
    [MeshContactQueue() addOperationWithBlock:^{
        // The captured source contains only numeric values and strings. This
        // worker never reads mutable OCAF/AIS or consults the selected object.
        self->_job->state=PublicState(AnalyzeTriangleContacts(
            self->_job->source->triangles,self->_job->result,self->_job->cancelled));
        dispatch_async(dispatch_get_main_queue(), ^{ [self deliverOnMain]; });
    }];
}
- (void)deliverOnMain {
    NSAssert(NSThread.isMainThread,@"Native contact completion requires main");
#if DEBUG
    void (^beforeDelivery)(void)=_beforeDeliveryHook;
    _beforeDeliveryHook=nil;_afterCaptureHook=nil;
    if(beforeDelivery) beforeDelivery();
#endif
    // Revalidate even a failed scan if a source was admitted: no outcome should
    // silently attach to a different or closed document during detached work.
    if(_job->source && !_job->cancelled.load(std::memory_order_acquire)) {
        try {
            const auto status=_validate ? _validate(*_job->source,_job->cancelled)
                                        : ContactSourceStatus::StaleSource;
            if(status!=ContactSourceStatus::Ready) _job->state=PublicState(status);
        } catch(...) { _job->state=Core3DMeshContactStateInternalFailure; }
    }
    {
        std::lock_guard<std::mutex> lock(_job->completionMutex);
        if(_job->finished) return;
        // Cancellation wins until this exact claim, including during owner
        // validation. A later cancel cannot change an already-delivered report.
        if(_job->cancelled.load(std::memory_order_acquire))
            _job->state=Core3DMeshContactStateCancelled;
        _job->finished=true;
    }
    if(_job->state!=Core3DMeshContactStateComplete) _job->result={};
    Core3DMeshContactReport *report=[[Core3DMeshContactReport alloc]
        initWithJob:*_job ownerToken:_ownerToken];
    Core3DMeshContactCompletion completion=_completion;
    _completion=nil;_prepare=nil;_validate=nil;
    // The controller may retain the original numeric capture in its bounded
    // registry for report actions. The completed operation releases its copy.
    _job->source.reset();_job->result={};
    _job->reservation.reset();
    if(completion) completion(report);
}
@end
