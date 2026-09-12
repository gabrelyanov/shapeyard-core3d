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

// Contact inspection owns copied renderer-neutral display values. No OCCT,
// document owner, diagnostic reservation, or editing capability escapes here.
#import "../Viewport/Core3DSceneSnapshotFactory.hpp"
#include "../Scene/SceneSnapshot.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
using namespace core3d::scene;
void ContactInclude(Bounds3d& bounds,const core3d::meshcheck::Point& point) {
    if(!bounds.valid) {
        bounds.minimum={point[0],point[1],point[2]};
        bounds.maximum=bounds.minimum;bounds.valid=true;return;
    }
    bounds.minimum={std::min(bounds.minimum.x,point[0]),std::min(bounds.minimum.y,point[1]),std::min(bounds.minimum.z,point[2])};
    bounds.maximum={std::max(bounds.maximum.x,point[0]),std::max(bounds.maximum.y,point[1]),std::max(bounds.maximum.z,point[2])};
}
Double3 ContactCenter(const Bounds3d& b) {
    return {b.minimum.x+(b.maximum.x-b.minimum.x)*.5,
        b.minimum.y+(b.maximum.y-b.minimum.y)*.5,
        b.minimum.z+(b.maximum.z-b.minimum.z)*.5};
}
double ContactRadius(const Bounds3d& b) {
    return std::hypot(b.maximum.x-b.minimum.x,
        b.maximum.y-b.minimum.y,b.maximum.z-b.minimum.z)*.5;
}
bool ContactCamera(const Bounds3d& full,const Bounds3d& pair,double yaw,
    double pitch,double zoom,bool focusPair,simd_uint2 viewport,CameraSnapshot& camera) {
    if(!std::isfinite(yaw)||!std::isfinite(pitch)||!std::isfinite(zoom)
        ||std::abs(yaw)>1e6||std::abs(pitch)>1.5||zoom<.25||zoom>16
        ||viewport.x==0||viewport.y==0||viewport.x>16384||viewport.y>16384) return false;
    const auto& bounds=focusPair?pair:full;
    camera.projection=Projection::Orthographic;
    camera.center=ContactCenter(bounds);
    const double radius=std::max(ContactRadius(bounds),1e-5);
    const double fullRadius=std::max(ContactRadius(full),1e-5);
    camera.aspect=double(viewport.x)/double(viewport.y);
    camera.viewportPixels={viewport.x,viewport.y};
    camera.orthographicHeight=2.4*radius/std::min(1.0,camera.aspect)/zoom;
    camera.verticalFovRadians=0.7853981633974483;
    // Keep the entire context inside the depth interval even while pair-fit
    // moves the camera center away from the full model center.
    const double distance=8*fullRadius+1;
    camera.eye={camera.center.x+distance*std::cos(pitch)*std::sin(yaw),
        camera.center.y+distance*std::sin(pitch),
        camera.center.z+distance*std::cos(pitch)*std::cos(yaw)};
    camera.up={0,1,0};camera.nearPlane=.001;camera.farPlane=distance+4*fullRadius+2;
    return true;
}
}

@implementation Core3DMeshContactInspection {
    std::string _inspectionPublication;
    core3d::scene::Bounds3d _inspectionBounds;
    core3d::scene::Bounds3d _inspectionPairBounds;
    uint64_t _inspectionFrameRevision;
}
- (instancetype)initWithSource:(const ContactSourceCapture&)source
    firstTriangle:(NSUInteger)firstTriangle secondTriangle:(NSUInteger)secondTriangle {
    if(!NSThread.isMainThread || source.triangles.empty() || source.triangles.size()>20000
        || firstTriangle>=source.triangles.size() || secondTriangle>=source.triangles.size()
        || firstTriangle==secondTriangle) return nil;
    self=[super init];if(!self)return nil;
    try {
        using namespace core3d::scene;
        std::vector<Triangle> world;world.reserve(source.triangles.size());
        Bounds3d worldBounds;
        for(const auto& triangle:source.triangles) {
            Triangle transformed;
            for(int corner=0;corner<3;++corner) {
                const auto& p=triangle[corner];core3d::meshcheck::Point local{};
                for(int row=0;row<3;++row) {
                    local[row]=source.facePlacement[row*4]*p[0]
                        +source.facePlacement[row*4+1]*p[1]
                        +source.facePlacement[row*4+2]*p[2]
                        +source.facePlacement[row*4+3]-source.sourceOrigin[row];
                }
                for(int row=0;row<3;++row) {
                    transformed[corner][row]=source.worldFromObject[row]*local[0]
                        +source.worldFromObject[4+row]*local[1]
                        +source.worldFromObject[8+row]*local[2]+source.worldFromObject[12+row];
                    if(!std::isfinite(transformed[corner][row]))return nil;
                }
                ContactInclude(worldBounds,transformed[corner]);
            }
            world.push_back(transformed);
        }
        const auto center=ContactCenter(worldBounds);
        const double radius=ContactRadius(worldBounds);
        if(!std::isfinite(radius)||radius<=0)return nil;
        const double scale=1/radius;
        if(!std::isfinite(scale)||scale<=0)return nil;
        // Normalize in Double before Float conversion. The detached diagnostic
        // geometry never feeds the exact intersection predicate or saved model.
        for(std::size_t i=0;i<world.size();++i)for(auto& p:world[i]) {
            p={(p[0]-center.x)*scale,(p[1]-center.y)*scale,(p[2]-center.z)*scale};
            for(double value:p)if(!std::isfinite(value))return nil;
            ContactInclude(_inspectionBounds,p);
            if(i==firstTriangle||i==secondTriangle)ContactInclude(_inspectionPairBounds,p);
        }
        SceneSnapshot scene;
        scene.publicationSourceIdentifier=NSUUID.UUID.UUIDString.UTF8String;
        scene.revisions={1,1,1,1,1};
        // One normalized display unit spans radius document units. Preserve
        // the exact native publication's physical scale despite recentering;
        // this detached read-only scene does not authorize model edits/export.
        if(!std::isfinite(source.metersPerUnit)||source.metersPerUnit<=0)return nil;
        scene.metersPerUnit=source.metersPerUnit*radius;
        if(!std::isfinite(scene.metersPerUnit)||scene.metersPerUnit<=0)return nil;
        if(!ContactCamera(_inspectionBounds,_inspectionPairBounds,.65,.4,1,false,
            (simd_uint2){1024,1024},scene.camera))return nil;
        const Float4 colors[3]={{.52f,.55f,.60f,1},{1,.12f,.16f,1},{.04f,.86f,1,1}};
        const char *names[3]={"Mesh context","First intersecting triangle","Second intersecting triangle"};
        for(int piece=0;piece<3;++piece) {
            MeshSnapshot mesh;mesh.definitionIdentifier="contact-piece-"+std::to_string(piece);
            mesh.geometryRevision=1;
            for(std::size_t i=0;i<world.size();++i) {
                const bool include=piece==0?(i!=firstTriangle&&i!=secondTriangle)
                    :i==(piece==1?firstTriangle:secondTriangle);
                if(!include)continue;
                // Assess the actual Float vertices the renderer will receive.
                // A native nondegenerate triangle can collapse after world
                // placement/normalization/Float conversion at extreme scale
                // ratios. Reject the inspection rather than invent a normal
                // for geometry that the display cannot faithfully represent.
                Triangle triangle;
                for(int corner=0;corner<3;++corner)for(int axis=0;axis<3;++axis)
                    triangle[corner][axis]=double(float(world[i][corner][axis]));
                const core3d::meshcheck::Point u={triangle[1][0]-triangle[0][0],triangle[1][1]-triangle[0][1],triangle[1][2]-triangle[0][2]};
                const core3d::meshcheck::Point v={triangle[2][0]-triangle[0][0],triangle[2][1]-triangle[0][1],triangle[2][2]-triangle[0][2]};
                core3d::meshcheck::Point n={u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]};
                const double length=std::hypot(n[0],n[1],n[2]);
                if(!(length>0)||!std::isfinite(length))return nil;
                for(auto& value:n)value/=length;
                for(const auto& p:triangle) {
                    Vertex vertex;
                    vertex.positionX=float(p[0]);vertex.positionY=float(p[1]);vertex.positionZ=float(p[2]);
                    vertex.normalX=float(n[0]);vertex.normalY=float(n[1]);vertex.normalZ=float(n[2]);
                    // Bounds describe the actual Float publication, including rounding.
                    ContactInclude(mesh.localBounds,{double(vertex.positionX),double(vertex.positionY),double(vertex.positionZ)});
                    mesh.indices.push_back(static_cast<std::uint32_t>(mesh.vertices.size()));
                    mesh.vertices.push_back(vertex);
                }
            }
            if(mesh.vertices.empty())continue;
            mesh.primitives.push_back({0,static_cast<std::uint32_t>(mesh.indices.size()),0,false});
            MaterialSnapshot material;material.identifier="contact-material-"+std::to_string(piece);
            material.baseColor=colors[piece];material.roughness=.85f;material.cullMode=CullMode::None;
            InstanceSnapshot instance;instance.entityIdentifier="contact-instance-"+std::to_string(piece);
            instance.meshIndex=static_cast<std::uint32_t>(scene.meshes.size());
            instance.name=names[piece];instance.selectable=false;instance.selected=false;
            instance.referenceAxis=ReferenceAxisSnapshot{};
            instance.primitiveBindings.push_back({static_cast<std::uint32_t>(scene.materials.size()),0,true});
            scene.materials.push_back(std::move(material));scene.meshes.push_back(std::move(mesh));
            scene.instances.push_back(std::move(instance));
        }
        Core3DSceneSnapshot *publicScene=Core3DCreateSceneSnapshotDTO(scene);
        if(publicScene==nil)return nil;
        PresentationOverlaySnapshot overlay;
        overlay.publicationSourceIdentifier=scene.publicationSourceIdentifier;
        overlay.baseSnapshotRevision=1;overlay.baseDocumentGeneration=1;
        overlay.baseModelRevision=1;overlay.basePresentationRevision=1;overlay.overlayRevision=1;
        Core3DScenePresentationOverlaySnapshot *publicOverlay=Core3DCreateScenePresentationOverlaySnapshotDTO(overlay);
        if(publicOverlay==nil)return nil;
        _scene=publicScene;_overlay=publicOverlay;_inspectionPublication=scene.publicationSourceIdentifier;
        _inspectionFrameRevision=1;_firstTriangle=firstTriangle;_secondTriangle=secondTriangle;
        return self;
    }catch(...){return nil;}
}
- (Core3DSceneFrameSnapshot *)frameWithYaw:(double)yaw pitch:(double)pitch
    zoom:(double)zoom focusPair:(BOOL)focusPair viewportSize:(simd_uint2)viewportSize {
    if(!NSThread.isMainThread||_inspectionFrameRevision==std::numeric_limits<uint64_t>::max())return nil;
    try {
        core3d::scene::FrameSnapshot frame;
        frame.publicationSourceIdentifier=_inspectionPublication;
        const auto revision=_inspectionFrameRevision+1;
        frame.revisions={revision,1,1,1,revision};
        if(!ContactCamera(_inspectionBounds,_inspectionPairBounds,yaw,pitch,zoom,focusPair,
            viewportSize,frame.camera))return nil;
        Core3DSceneFrameSnapshot *result=Core3DCreateSceneFrameSnapshotDTO(frame);
        if(result!=nil)_inspectionFrameRevision=revision;
        return result;
    }catch(...){return nil;}
}
@end
