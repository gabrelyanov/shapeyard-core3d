#import "Core3DMacDocumentSession.h"

#import "Core3DSceneSnapshotFactory.hpp"
#import "Core3DTransformInspectorSnapshotFactory.hpp"
#import "Core3DViewer.h"

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#if !defined(TARGET_OS_OSX) || !TARGET_OS_OSX
#error "Core3DMacDocumentSession is macOS-only."
#endif

namespace {

class ScopedCurrentOpenGLContext final {
public:
    ScopedCurrentOpenGLContext(NSOpenGLView *view, NSOpenGLContext *context)
        : previous_([NSOpenGLContext currentContext]), context_(context) {
        if (view == nil || context == nil || view.openGLContext != context
            || context.view != view) return;
        [context makeCurrentContext];
        valid_ = [NSOpenGLContext currentContext] == context;
    }

    ~ScopedCurrentOpenGLContext() {
        if (previous_ != nil) [previous_ makeCurrentContext];
        else [NSOpenGLContext clearCurrentContext];
    }

    bool valid() const noexcept { return valid_; }

private:
    NSOpenGLContext *__strong previous_ = nil;
    NSOpenGLContext *__strong context_ = nil;
    bool valid_ = false;
};

//! Teardown scope intentionally binds the originally retained context directly.
//! It does not consult the view's mutable context/window relationship.
class ScopedOwnedOpenGLContext final {
public:
    explicit ScopedOwnedOpenGLContext(NSOpenGLContext *context)
        : previous_([NSOpenGLContext currentContext]), context_(context) {
        if (context_ == nil) return;
        [context_ makeCurrentContext];
        valid_ = [NSOpenGLContext currentContext] == context_;
    }
    ~ScopedOwnedOpenGLContext() {
        if (previous_ != nil) [previous_ makeCurrentContext];
        else [NSOpenGLContext clearCurrentContext];
    }
    bool valid() const noexcept { return valid_; }
private:
    NSOpenGLContext *__strong previous_ = nil;
    NSOpenGLContext *__strong context_ = nil;
    bool valid_ = false;
};

bool ReleaseViewerWithOwnedContext(
    const std::shared_ptr<core3d::Core3DViewer>& viewer,
    NSOpenGLContext *context) noexcept {
    if (!viewer || context == nil) return false;
    try {
        ScopedOwnedOpenGLContext current(context);
        if (!current.valid()) return false;
        viewer->cancelTransformInspectorMeasurement();
        bool callbacksCleared = true;
        try {
            viewer->setBooleanPreviewStateChangedCallback({});
            viewer->setLinearArrayPreviewStateChangedCallback({});
            viewer->setRadialArrayPreviewStateChangedCallback({});
            viewer->setBevelPreviewStateChangedCallback({});
            viewer->setShellPreviewStateChangedCallback({});
            viewer->setInteractorRecreatedCallback({});
        } catch (...) {
            callbacksCleared = false;
        }
        viewer->release();
        return callbacksCleared;
    } catch (...) {
        // This is the last-resort deallocation path; no exception may cross it.
        return false;
    }
}

Core3DMacHistoryOutcome PublicHistoryOutcome(
    core3d::NativeHistoryOutcome outcome) noexcept {
    switch (outcome) {
        case core3d::NativeHistoryOutcome::PreviewCancelled:
            return Core3DMacHistoryOutcomePreviewCancelled;
        case core3d::NativeHistoryOutcome::PreviewCancellationFailed:
            return Core3DMacHistoryOutcomePreviewCancellationFailed;
        case core3d::NativeHistoryOutcome::NoHistory:
            return Core3DMacHistoryOutcomeNoHistory;
        case core3d::NativeHistoryOutcome::HistoryChanged:
            return Core3DMacHistoryOutcomeChanged;
        case core3d::NativeHistoryOutcome::HistoryFailed:
            return Core3DMacHistoryOutcomeFailed;
        case core3d::NativeHistoryOutcome::Unavailable:
            return Core3DMacHistoryOutcomeUnavailable;
    }
    return Core3DMacHistoryOutcomeUnavailable;
}

Core3DTransformInspectorPositionCommitResult PublicCommitResult(
    core3d::TransformInspectorPositionCommitResult result) noexcept {
    switch (result) {
        case core3d::TransformInspectorPositionCommitResult::Committed:
            return Core3DTransformInspectorPositionCommitResultCommitted;
        case core3d::TransformInspectorPositionCommitResult::Unchanged:
            return Core3DTransformInspectorPositionCommitResultUnchanged;
        case core3d::TransformInspectorPositionCommitResult::InvalidValue:
            return Core3DTransformInspectorPositionCommitResultInvalidValue;
        case core3d::TransformInspectorPositionCommitResult::Busy:
            return Core3DTransformInspectorPositionCommitResultBusy;
        case core3d::TransformInspectorPositionCommitResult::Stale:
            return Core3DTransformInspectorPositionCommitResultStale;
        case core3d::TransformInspectorPositionCommitResult::Unsupported:
            return Core3DTransformInspectorPositionCommitResultUnsupported;
        case core3d::TransformInspectorPositionCommitResult::Unavailable:
            return Core3DTransformInspectorPositionCommitResultUnavailable;
        case core3d::TransformInspectorPositionCommitResult::InternalFailure:
            return Core3DTransformInspectorPositionCommitResultInternalFailure;
    }
    return Core3DTransformInspectorPositionCommitResultInternalFailure;
}

bool NativeAxis(Core3DTransformInspectorAxis axis,
    core3d::TransformInspectorPositionAxis& result) noexcept {
    switch (axis) {
        case Core3DTransformInspectorAxisX:
            result = core3d::TransformInspectorPositionAxis::X; return true;
        case Core3DTransformInspectorAxisY:
            result = core3d::TransformInspectorPositionAxis::Y; return true;
        case Core3DTransformInspectorAxisZ:
            result = core3d::TransformInspectorPositionAxis::Z; return true;
    }
    return false;
}

std::size_t EditableModelCount(const core3d::scene::SceneSnapshot& scene) {
    std::size_t count = 0;
    for (const auto& instance : scene.instances) {
        if (instance.role == core3d::scene::RenderRole::Model
            && instance.visible && instance.selectable) ++count;
    }
    return count;
}

} // namespace

@interface Core3DMacScenePublication ()
- (instancetype)initWithScene:(Core3DSceneSnapshot *)scene
    overlay:(Core3DScenePresentationOverlaySnapshot *)overlay;
@end

@implementation Core3DMacScenePublication
- (instancetype)initWithScene:(Core3DSceneSnapshot *)scene
    overlay:(Core3DScenePresentationOverlaySnapshot *)overlay {
    if (scene == nil || overlay == nil
        || ![scene.publicationSourceIdentifier
            isEqualToString:overlay.publicationSourceIdentifier]
        || overlay.baseSnapshotRevision != scene.snapshotRevision
        || overlay.baseDocumentGeneration != scene.revisions.documentGeneration
        || overlay.baseModelRevision != scene.revisions.modelRevision
        || overlay.basePresentationRevision != scene.revisions.presentationRevision) {
        return nil;
    }
    self = [super init];
    if (self) { _scene = scene; _overlay = overlay; }
    return self;
}
@end

@interface Core3DMacHistoryResult ()
- (instancetype)initWithTransition:(const core3d::NativeHistoryTransition&)transition
    publication:(Core3DMacScenePublication * _Nullable)publication;
@end

@implementation Core3DMacHistoryResult
- (instancetype)initWithTransition:(const core3d::NativeHistoryTransition&)transition
    publication:(Core3DMacScenePublication * _Nullable)publication {
    self = [super init];
    if (self) {
        _outcome = PublicHistoryOutcome(transition.outcome);
        _documentRedrawn = transition.documentRedrawn;
        _booleanReconciliationRequired = transition.reconcileBooleanTool;
        _selectionRefreshRequested = transition.refreshSelection;
        _renderRequested = transition.requestRender;
        _primaryInteractionCancelled = transition.primaryInteractionCancelled;
        _publication = publication;
    }
    return self;
}
@end

@implementation Core3DMacDocumentSession {
    NSOpenGLView *_engineView;
    NSOpenGLContext *_engineContext;
    std::shared_ptr<core3d::Core3DViewer> _viewer;
    core3d::scene::OcctSceneSnapshotBuilder::SnapshotPointer _nativeScene;
    std::optional<core3d::TransformInspectorMeasurement> _measurement;
    Core3DTransformInspectorSnapshot *_measurementDTO;
    Core3DMacScenePublication *_publication;
    uint32_t _viewportWidth;
    uint32_t _viewportHeight;
    uint64_t _measurementRequest;
    uint64_t _actionGeneration;
    BOOL _closed;
    BOOL _nativePresentationDirty;
}

- (instancetype)initWithAttachedOpenGLView:(NSOpenGLView *)view {
    if (![NSThread isMainThread] || view == nil || view.window == nil
        || view.openGLContext == nil || view.openGLContext.view != view) return nil;
    self = [super init];
    if (!self) return nil;

    _engineView = view;
    _engineContext = view.openGLContext;
    try {
        _viewer = std::make_shared<core3d::Core3DViewer>();
        ScopedCurrentOpenGLContext current(_engineView, _engineContext);
        if (!current.valid()) {
            _viewer.reset();
            return nil;
        }
        if (!_viewer->InitViewer(_engineView)) {
            _viewer->release();
            _viewer.reset();
            return nil;
        }

        __weak Core3DMacDocumentSession *weakSelf = self;
        const auto notePresentation = [weakSelf]() noexcept {
            Core3DMacDocumentSession *session = weakSelf;
            if (session != nil) session->_nativePresentationDirty = YES;
        };
        _viewer->setBooleanPreviewStateChangedCallback(notePresentation);
        _viewer->setLinearArrayPreviewStateChangedCallback(notePresentation);
        _viewer->setRadialArrayPreviewStateChangedCallback(notePresentation);
        _viewer->setBevelPreviewStateChangedCallback(notePresentation);
        _viewer->setShellPreviewStateChangedCallback(notePresentation);
    } catch (...) {
        // Init may already own partial OCCT graphics state. Retire it under the
        // original context before the shared_ptr is allowed to destruct.
        ReleaseViewerWithOwnedContext(_viewer, _engineContext);
        _viewer.reset();
        return nil;
    }
    return self;
}

- (NSOpenGLView *)engineView { return _engineView; }
- (Core3DMacScenePublication *)publication {
    return _nativePresentationDirty ? nil : _publication;
}
- (BOOL)isClosed { return _closed; }

- (BOOL)core3d_canEnterAttachedContext {
    return [NSThread isMainThread] && !_closed && _viewer != nullptr
        && _engineView != nil && _engineView.window != nil
        && _engineContext != nil && _engineView.openGLContext == _engineContext
        && _engineContext.view == _engineView;
}

- (Core3DMacScenePublication *)core3d_capturePublicationLockedWidth:(uint32_t)width
    height:(uint32_t)height render:(BOOL)render {
    _nativeScene.reset();
    _publication = nil;
    _nativePresentationDirty = YES;
    if (width == 0 || height == 0 || _viewer == nullptr) return nil;
    try {
        if (render && !_viewer->RenderFrame()) return nil;
        const auto scene = _viewer->captureSceneSnapshot(width, height);
        if (!scene) return nil;
        const auto overlay = _viewer->captureScenePresentationOverlay();
        if (!overlay) return nil;
        Core3DSceneSnapshot *sceneDTO = Core3DCreateSceneSnapshotDTO(*scene);
        Core3DScenePresentationOverlaySnapshot *overlayDTO =
            Core3DCreateScenePresentationOverlaySnapshotDTO(*overlay);
        Core3DMacScenePublication *publication =
            [[Core3DMacScenePublication alloc] initWithScene:sceneDTO overlay:overlayDTO];
        if (publication == nil) return nil;
        _nativeScene = scene;
        _publication = publication;
        _viewportWidth = width;
        _viewportHeight = height;
        _nativePresentationDirty = NO;
        return publication;
    } catch (...) {
        return nil;
    }
}

- (void)core3d_invalidateAuthority {
    ++_measurementRequest;
    _measurement.reset();
    _measurementDTO = nil;
    _nativeScene.reset();
    _publication = nil;
}

- (Core3DMacSessionActionResult)refreshPublicationWithWidth:(uint32_t)width
    height:(uint32_t)height {
    if (![self core3d_canEnterAttachedContext])
        return [NSThread isMainThread] ? Core3DMacSessionActionResultContextUnavailable
                                      : Core3DMacSessionActionResultUnavailable;
    ++_actionGeneration;
    ScopedCurrentOpenGLContext current(_engineView, _engineContext);
    if (!current.valid()) return Core3DMacSessionActionResultContextUnavailable;
    return [self core3d_capturePublicationLockedWidth:width height:height render:YES] != nil
        ? Core3DMacSessionActionResultUnchanged
        : Core3DMacSessionActionResultUnavailable;
}

- (Core3DMacSessionActionResult)createCubeWithViewportWidth:(uint32_t)width
    height:(uint32_t)height {
    if (![self core3d_canEnterAttachedContext])
        return [NSThread isMainThread] ? Core3DMacSessionActionResultContextUnavailable
                                      : Core3DMacSessionActionResultUnavailable;
    ++_actionGeneration;
    ScopedCurrentOpenGLContext current(_engineView, _engineContext);
    if (!current.valid()) return Core3DMacSessionActionResultContextUnavailable;
    if (_viewer->hasUnresolvedEdit()) return Core3DMacSessionActionResultBusy;
    bool mutationAttempted = false;
    try {
        const auto before = _viewer->captureSceneSnapshot(width, height);
        if (!before) return Core3DMacSessionActionResultUnavailable;
        const std::size_t beforeCount = EditableModelCount(*before);
        const std::uint64_t beforeModel = before->revisions.model;

        mutationAttempted = true;
        _viewer->addPrimitive(PrimitiveTypeCube);
        [self core3d_invalidateAuthority];

        // Keep the accepted scene and its actual overlay adjacent: no native
        // render or host callback occurs between the two captures.
        const bool rendered = _viewer->RenderFrame();
        const auto after = _viewer->captureSceneSnapshot(width, height);
        if (!after) return Core3DMacSessionActionResultInternalFailure;
        const bool changed = after->revisions.model > beforeModel
            && EditableModelCount(*after) == beforeCount + 1;
        if (!changed) return _viewer->hasUnresolvedEdit()
            ? Core3DMacSessionActionResultBusy
            : Core3DMacSessionActionResultInternalFailure;

        const auto overlay = _viewer->captureScenePresentationOverlay();
        Core3DSceneSnapshot *sceneDTO = Core3DCreateSceneSnapshotDTO(*after);
        Core3DScenePresentationOverlaySnapshot *overlayDTO = overlay
            ? Core3DCreateScenePresentationOverlaySnapshotDTO(*overlay) : nil;
        Core3DMacScenePublication *publication =
            [[Core3DMacScenePublication alloc] initWithScene:sceneDTO overlay:overlayDTO];
        if (publication != nil && rendered) {
            _nativeScene = after; _publication = publication;
            _viewportWidth = width; _viewportHeight = height;
            _nativePresentationDirty = NO;
            return Core3DMacSessionActionResultChanged;
        }
        return Core3DMacSessionActionResultChangedPublicationUnavailable;
    } catch (...) {
        if (mutationAttempted) {
            [self core3d_invalidateAuthority];
            _nativePresentationDirty = YES;
        }
        return Core3DMacSessionActionResultInternalFailure;
    }
}

- (Core3DMacSessionActionResult)selectEntityIdentifier:(NSString *)identifier
    expectedPublication:(Core3DMacScenePublication *)expectedPublication
    viewportWidth:(uint32_t)width height:(uint32_t)height {
    if (![self core3d_canEnterAttachedContext])
        return [NSThread isMainThread] ? Core3DMacSessionActionResultContextUnavailable
                                      : Core3DMacSessionActionResultUnavailable;
    ++_actionGeneration;
    if (identifier.length == 0 || identifier.UTF8String == nullptr
        || expectedPublication == nil || expectedPublication != _publication
        || _nativePresentationDirty || !_nativeScene
        || width == 0 || height == 0) {
        return Core3DMacSessionActionResultStale;
    }

    ScopedCurrentOpenGLContext current(_engineView, _engineContext);
    if (!current.valid()) return Core3DMacSessionActionResultContextUnavailable;
    if (_viewer->hasUnresolvedEdit()) return Core3DMacSessionActionResultBusy;

    try {
    const std::string requested(identifier.UTF8String);
    const auto& scene = *_nativeScene;
    if (![expectedPublication.scene.publicationSourceIdentifier
            isEqualToString:[NSString stringWithUTF8String:
                scene.publicationSourceIdentifier.c_str()]]
        || expectedPublication.scene.revisions.documentGeneration
            != scene.revisions.documentGeneration
        || expectedPublication.scene.revisions.modelRevision != scene.revisions.model) {
        return Core3DMacSessionActionResultStale;
    }

    std::size_t matches = 0;
    for (const auto& instance : scene.instances) {
        if (instance.entityIdentifier == requested) {
            if (instance.role != core3d::scene::RenderRole::Model
                || !instance.visible || !instance.selectable) {
                return Core3DMacSessionActionResultUnsupported;
            }
            ++matches;
        }
    }
    if (matches != 1) return Core3DMacSessionActionResultStale;

    core3d::ObjectFrameIdentity identity;
    identity.entityIdentifier = requested;
    identity.publicationSourceIdentifier = scene.publicationSourceIdentifier;
    identity.documentGeneration = scene.revisions.documentGeneration;
    identity.modelRevision = scene.revisions.model;
    bool touched = false;
    const bool selected =
        _viewer->selectObjectFromBrowser(identity, width, height, touched);
    if (!selected) return _viewer->hasUnresolvedEdit()
        ? Core3DMacSessionActionResultBusy : Core3DMacSessionActionResultStale;

    ++_measurementRequest;
    _measurement.reset();
    _measurementDTO = nil;
    _nativeScene.reset();
    _publication = nil;
    Core3DMacScenePublication *updated =
        [self core3d_capturePublicationLockedWidth:width height:height render:YES];
    if (updated == nil) return touched
        ? Core3DMacSessionActionResultChangedPublicationUnavailable
        : Core3DMacSessionActionResultUnavailable;
    return touched ? Core3DMacSessionActionResultChanged
                   : Core3DMacSessionActionResultUnchanged;
    } catch (...) {
        return Core3DMacSessionActionResultInternalFailure;
    }
}

- (Core3DTransformInspectorSnapshot *)captureTransformMeasurementWithCompletion:
    (Core3DTransformInspectorCompletion)completion {
    if (![self core3d_canEnterAttachedContext]) {
        return Core3DCreateTransformInspectorStateSnapshotDTO(
            Core3DTransformInspectorStateUnavailable);
    }
    ++_actionGeneration;
    ScopedCurrentOpenGLContext current(_engineView, _engineContext);
    if (!current.valid()) {
        return Core3DCreateTransformInspectorStateSnapshotDTO(
            Core3DTransformInspectorStateUnavailable);
    }

    try {
    _viewer->cancelTransformInspectorMeasurement();
    const uint64_t request = ++_measurementRequest;
    _measurement.reset();
    _measurementDTO = nil;
    Core3DTransformInspectorCompletion completionCopy = [completion copy];
    __weak Core3DMacDocumentSession *weakSelf = self;
    core3d::TransformInspectorMeasurementCompletion nativeCompletion =
        [weakSelf, request, completionCopy](
            core3d::TransformInspectorMeasurement measurement) noexcept {
        try {
            auto retained = std::make_shared<core3d::TransformInspectorMeasurement>(
                std::move(measurement));
            dispatch_async(dispatch_get_main_queue(), ^{
                Core3DMacDocumentSession *session = weakSelf;
                if (session == nil || session->_closed
                    || session->_measurementRequest != request) return;
                try {
                    Core3DTransformInspectorSnapshot *snapshot =
                        Core3DCreateTransformInspectorSnapshotDTO(*retained);
                    session->_measurement = *retained;
                    session->_measurementDTO = snapshot;
                    if (completionCopy != nil) completionCopy(snapshot);
                } catch (...) {
                    ++session->_measurementRequest;
                    session->_measurement.reset();
                    session->_measurementDTO = nil;
                }
            });
        } catch (...) {
            dispatch_async(dispatch_get_main_queue(), ^{
                Core3DMacDocumentSession *session = weakSelf;
                if (session == nil || session->_closed
                    || session->_measurementRequest != request) return;
                ++session->_measurementRequest;
                session->_measurement.reset();
                session->_measurementDTO = nil;
                if (completionCopy != nil) {
                    @try {
                        try {
                            completionCopy(
                                Core3DCreateTransformInspectorStateSnapshotDTO(
                                    Core3DTransformInspectorStateUnavailable));
                        } catch (...) {
                        }
                    } @catch (__unused NSException *exception) {
                    }
                }
            });
        }
    };
    const auto measurement = _viewer->captureTransformInspectorMeasurement(
        std::move(nativeCompletion));
    Core3DTransformInspectorSnapshot *snapshot =
        Core3DCreateTransformInspectorSnapshotDTO(measurement);
    _measurement = measurement;
    _measurementDTO = snapshot;
    return snapshot;
    } catch (...) {
        ++_measurementRequest;
        _measurement.reset();
        _measurementDTO = nil;
        _viewer->cancelTransformInspectorMeasurement();
        return Core3DCreateTransformInspectorStateSnapshotDTO(
            Core3DTransformInspectorStateUnavailable);
    }
}

- (void)cancelTransformMeasurement {
    if (![NSThread isMainThread]) return;
    ++_actionGeneration;
    ++_measurementRequest;
    _measurement.reset();
    _measurementDTO = nil;
    // Even without context affinity, invalidate delivery of the outstanding
    // worker result. Native cancellation waits until context ownership returns.
    if (![self core3d_canEnterAttachedContext]) return;
    ScopedCurrentOpenGLContext current(_engineView, _engineContext);
    if (current.valid()) _viewer->cancelTransformInspectorMeasurement();
}

- (Core3DTransformInspectorPositionCommitResult)commitPositionValue:(double)value
    axis:(Core3DTransformInspectorAxis)axis
    expectedMeasurement:(Core3DTransformInspectorSnapshot *)expectedMeasurement {
    if (![NSThread isMainThread])
        return Core3DTransformInspectorPositionCommitResultUnavailable;
    ++_actionGeneration;
    // Public pointer identity is the UI-side lease. Reject it before touching
    // the retained native lease so an old text field cannot consume a newer one.
    if (expectedMeasurement == nil || expectedMeasurement != _measurementDTO)
        return Core3DTransformInspectorPositionCommitResultStale;
    if (!std::isfinite(value)
        || std::abs(value) > Core3DTransformInspectorMaximumPositionMagnitude) {
        return Core3DTransformInspectorPositionCommitResultInvalidValue;
    }
    if (![self core3d_canEnterAttachedContext] || !_measurement) {
        return Core3DTransformInspectorPositionCommitResultUnavailable;
    }
    core3d::TransformInspectorPositionAxis nativeAxis;
    if (!NativeAxis(axis, nativeAxis))
        return Core3DTransformInspectorPositionCommitResultInvalidValue;

    ScopedCurrentOpenGLContext current(_engineView, _engineContext);
    if (!current.valid())
        return Core3DTransformInspectorPositionCommitResultUnavailable;

    try {
    auto measurement = std::move(*_measurement);
    ++_measurementRequest;
    _measurement.reset();
    _measurementDTO = nil;
    if (!measurement.canEditPosition || !measurement.presentationMatchesDocument
        || measurement.positionEditGeneration == 0
        || measurement.documentEditGeneration == 0
        || measurement.geometryEditGeneration == 0
        || measurement.entityIdentifier.empty()
        || measurement.definitionIdentifier.empty()) {
        return Core3DTransformInspectorPositionCommitResultUnsupported;
    }

    core3d::TransformInspectorPositionCommitRequest request;
    request.kind = core3d::TransformInspectorEditKind::Position;
    request.positionEditGeneration = measurement.positionEditGeneration;
    request.documentEditGeneration = measurement.documentEditGeneration;
    request.geometryEditGeneration = measurement.geometryEditGeneration;
    request.entityIdentifier = measurement.entityIdentifier;
    request.definitionIdentifier = measurement.definitionIdentifier;
    request.expectedPosition = measurement.position;
    request.expectedMetersPerUnit = measurement.metersPerUnit;
    request.expectedRepresentation = measurement.representation;
    request.expectedModelCapabilities = measurement.modelCapabilities;
    request.axis = nativeAxis;
    request.value = value;

    const auto nativeResult = _viewer->commitTransformInspectorPosition(request);
    const auto result = PublicCommitResult(nativeResult);
    if (nativeResult == core3d::TransformInspectorPositionCommitResult::Committed) {
        _nativeScene.reset(); _publication = nil;
        if (_viewportWidth != 0 && _viewportHeight != 0) {
            (void)[self core3d_capturePublicationLockedWidth:_viewportWidth
                height:_viewportHeight render:YES];
        }
    } else if (nativeResult
        == core3d::TransformInspectorPositionCommitResult::InternalFailure) {
        _nativeScene.reset(); _publication = nil;
        _nativePresentationDirty = YES;
    }
    return result;
    } catch (...) {
        ++_measurementRequest;
        _measurement.reset();
        _measurementDTO = nil;
        return Core3DTransformInspectorPositionCommitResultInternalFailure;
    }
}

- (Core3DMacHistoryResult *)performHistory:(Core3DMacHistoryDirection)direction {
    if (![NSThread isMainThread]) {
        core3d::NativeHistoryTransition unavailable;
        return [[Core3DMacHistoryResult alloc] initWithTransition:unavailable publication:nil];
    }
    if ([NSThread isMainThread]) ++_actionGeneration;
    core3d::NativeHistoryTransition transition;
    Core3DMacScenePublication *publication = nil;
    if ([self core3d_canEnterAttachedContext]
        && (direction == Core3DMacHistoryDirectionUndo
            || direction == Core3DMacHistoryDirectionRedo)) {
        ScopedCurrentOpenGLContext current(_engineView, _engineContext);
        if (current.valid()) {
            try {
            _viewer->cancelTransformInspectorMeasurement();
            ++_measurementRequest; _measurement.reset(); _measurementDTO = nil;
            transition = _viewer->performHistory(
                direction == Core3DMacHistoryDirectionUndo
                    ? core3d::NativeHistoryDirection::Undo
                    : core3d::NativeHistoryDirection::Redo);
            if (transition.reconcileBooleanTool) {
                const auto objectInteractor = _viewer->getObjectInteractor();
                if (objectInteractor) objectInteractor->setManipulatorType(
                    core3d::PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
            }
            if (transition.refreshSelection || transition.requestRender) {
                _nativeScene.reset(); _publication = nil;
                publication = [self core3d_capturePublicationLockedWidth:_viewportWidth
                    height:_viewportHeight render:transition.requestRender];
            }
            } catch (...) {
                transition = {};
                transition.outcome = core3d::NativeHistoryOutcome::HistoryFailed;
                [self core3d_invalidateAuthority];
                _nativePresentationDirty = YES;
            }
        }
    }

    __attribute__((objc_precise_lifetime))
    Core3DMacDocumentSession *effectOwner = self;
    Core3DMacHistoryResult *result =
        [[Core3DMacHistoryResult alloc] initWithTransition:transition
            publication:publication];
    dispatch_block_t booleanHandler =
        [effectOwner.booleanReconciliationHandler copy];
    void (^selectionHandler)(Core3DMacScenePublication *) =
        [effectOwner.selectionRefreshHandler copy];
    void (^renderHandler)(Core3DMacScenePublication *) =
        [effectOwner.renderRequestHandler copy];
    dispatch_block_t primaryHandler =
        [effectOwner.primaryInteractionCancelledHandler copy];
    const uint64_t effectGeneration = effectOwner->_actionGeneration;

    // Native transition and context restoration are complete before this order.
    if (transition.reconcileBooleanTool && booleanHandler) booleanHandler();
    if (!effectOwner->_closed
        && effectOwner->_actionGeneration == effectGeneration
        && [effectOwner core3d_canEnterAttachedContext]
        && transition.refreshSelection && selectionHandler) {
        selectionHandler(publication);
    }
    if (!effectOwner->_closed
        && effectOwner->_actionGeneration == effectGeneration
        && [effectOwner core3d_canEnterAttachedContext]
        && transition.requestRender && renderHandler) {
        renderHandler(publication);
    }
    if (!effectOwner->_closed
        && effectOwner->_actionGeneration == effectGeneration
        && [effectOwner core3d_canEnterAttachedContext]
        && transition.primaryInteractionCancelled && primaryHandler) {
        primaryHandler();
    }
    return result;
}

- (Core3DMacOrdinaryEditRecoveryResult)reconcileOrdinaryEditWithViewportWidth:
    (uint32_t)width height:(uint32_t)height {
    if (![NSThread isMainThread])
        return Core3DMacOrdinaryEditRecoveryResultUnavailable;
    ++_actionGeneration;
    if (![self core3d_canEnterAttachedContext])
        return Core3DMacOrdinaryEditRecoveryResultContextUnavailable;
    ScopedCurrentOpenGLContext current(_engineView, _engineContext);
    if (!current.valid())
        return Core3DMacOrdinaryEditRecoveryResultContextUnavailable;
    if (!_viewer->hasUnresolvedOrdinaryEdit())
        return Core3DMacOrdinaryEditRecoveryResultNoUnresolvedEdit;

    _viewer->cancelTransformInspectorMeasurement();
    [self core3d_invalidateAuthority];
    _nativePresentationDirty = YES;
    const auto recovery = _viewer->reconcileOrdinaryEdit();
    switch (recovery) {
        case core3d::OrdinaryEditResult::NoChange:
        case core3d::OrdinaryEditResult::Committed: {
            Core3DMacScenePublication *fresh =
                [self core3d_capturePublicationLockedWidth:width
                    height:height render:YES];
            if (fresh == nil) {
                return recovery == core3d::OrdinaryEditResult::Committed
                    ? Core3DMacOrdinaryEditRecoveryResultCommittedPublicationUnavailable
                    : Core3DMacOrdinaryEditRecoveryResultNoChangePublicationUnavailable;
            }
            return recovery == core3d::OrdinaryEditResult::Committed
                ? Core3DMacOrdinaryEditRecoveryResultCommitted
                : Core3DMacOrdinaryEditRecoveryResultNoChange;
        }
        case core3d::OrdinaryEditResult::RetryableFailure:
            return Core3DMacOrdinaryEditRecoveryResultRetryableFailure;
        case core3d::OrdinaryEditResult::OutcomeUnknown:
            return Core3DMacOrdinaryEditRecoveryResultOutcomeUnknown;
        case core3d::OrdinaryEditResult::Busy:
            return Core3DMacOrdinaryEditRecoveryResultBusy;
        case core3d::OrdinaryEditResult::Invalid:
            return Core3DMacOrdinaryEditRecoveryResultInvalid;
    }
    return Core3DMacOrdinaryEditRecoveryResultUnavailable;
}

- (Core3DMacSessionActionResult)close {
    if (![NSThread isMainThread]) return Core3DMacSessionActionResultUnavailable;
    ++_actionGeneration;
    if (_closed) return Core3DMacSessionActionResultUnchanged;
    if (_viewer == nullptr) { _closed = YES; return Core3DMacSessionActionResultUnchanged; }

    ScopedOwnedOpenGLContext current(_engineContext);
    if (!current.valid()) return Core3DMacSessionActionResultContextUnavailable;
    if (_viewer->hasUnresolvedEdit())
        return Core3DMacSessionActionResultBusy;
    ++_measurementRequest;
    _measurement.reset();
    _measurementDTO = nil;
    _nativeScene.reset();
    _publication = nil;
    const bool cleanRelease =
        ReleaseViewerWithOwnedContext(_viewer, _engineContext);
    _viewer.reset();
    self.booleanReconciliationHandler = nil;
    self.selectionRefreshHandler = nil;
    self.renderRequestHandler = nil;
    self.primaryInteractionCancelledHandler = nil;
    _closed = YES;
    return cleanRelease ? Core3DMacSessionActionResultChanged
                        : Core3DMacSessionActionResultInternalFailure;
}

- (void)dealloc {
    if (!_closed && _viewer != nullptr) {
        auto viewer = std::move(_viewer);
        NSOpenGLContext *context = _engineContext;
        NSOpenGLView *view = _engineView;
        if ([NSThread isMainThread]) {
            (void)view;
            (void)ReleaseViewerWithOwnedContext(viewer, context);
        } else {
            dispatch_async(dispatch_get_main_queue(), ^{
                // Retain the original view/drawable alongside its context until
                // the viewer's GPU handles are retired on main.
                (void)view;
                (void)ReleaseViewerWithOwnedContext(viewer, context);
            });
        }
    }
}

@end
