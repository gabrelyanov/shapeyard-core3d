#import <Cocoa/Cocoa.h>
#import <OpenGL/gl3.h>
#import <Core3D/Core3DSharedModelingValues.h>
#import <Core3D/Core3DMacDocumentSession.h>

#include "Core3DViewer.h"

#include <OpenGl_Context.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Standard_Failure.hxx>

#include <cmath>
#include <algorithm>
#include <cstdio>
#include <exception>
#include <stdexcept>
#include <vector>

namespace
{
void require(bool theCondition, const char* theMessage)
{
  if (!theCondition)
  {
    throw std::runtime_error(theMessage);
  }
}

bool finite3(const core3d::scene::Double3& theValue)
{
  return std::isfinite(theValue.x)
      && std::isfinite(theValue.y)
      && std::isfinite(theValue.z);
}

double distanceSquared(const core3d::scene::Double3& a,
                       const core3d::scene::Double3& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

constexpr std::size_t kReadbackWidth = 640;
constexpr std::size_t kReadbackHeight = 480;
constexpr std::size_t kReadbackChannels = 4;
constexpr std::size_t kReadbackBytes =
  kReadbackWidth * kReadbackHeight * kReadbackChannels;
static_assert(kReadbackBytes == 1'228'800 && kReadbackBytes < 4 * 1024 * 1024,
              "probe readback must remain a small fixed allocation");

void requireNoGLError(const char* theMessage)
{
  require(glGetError() == GL_NO_ERROR, theMessage);
}

std::vector<std::uint8_t> readOwnedBackFramebuffer()
{
  std::vector<std::uint8_t> aPixels(kReadbackBytes, 0);
  GLint aPreviousReadFramebuffer = 0;
  GLint aPreviousPackAlignment = 0;
  GLint aPackBuffer = 0;
  GLint aPackRowLength = 0;
  GLint aPackSkipRows = 0;
  GLint aPackSkipPixels = 0;
  glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &aPreviousReadFramebuffer);
  glGetIntegerv(GL_PACK_ALIGNMENT, &aPreviousPackAlignment);
  glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &aPackBuffer);
  glGetIntegerv(GL_PACK_ROW_LENGTH, &aPackRowLength);
  glGetIntegerv(GL_PACK_SKIP_ROWS, &aPackSkipRows);
  glGetIntegerv(GL_PACK_SKIP_PIXELS, &aPackSkipPixels);
  requireNoGLError("OpenGL state query failed before framebuffer readback");
  require(aPackBuffer == 0 && aPackRowLength == 0
          && aPackSkipRows == 0 && aPackSkipPixels == 0,
          "pixel-pack state would exceed the bounded CPU readback layout");

  // Core3D sets buffersNoSwap and renders the NSOpenGL drawable's back buffer.
  // Read that same application-owned default framebuffer rather than creating
  // a second FBO or a separate validation renderer.
  glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
  GLint aPreviousDefaultReadBuffer = 0;
  glGetIntegerv(GL_READ_BUFFER, &aPreviousDefaultReadBuffer);
  glReadBuffer(GL_BACK);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadPixels(0, 0, static_cast<GLsizei>(kReadbackWidth),
               static_cast<GLsizei>(kReadbackHeight),
               GL_RGBA, GL_UNSIGNED_BYTE, aPixels.data());
  glFinish();
  requireNoGLError("owned back-buffer readback produced an OpenGL error");

  glPixelStorei(GL_PACK_ALIGNMENT, aPreviousPackAlignment);
  glReadBuffer(static_cast<GLenum>(aPreviousDefaultReadBuffer));
  glBindFramebuffer(GL_READ_FRAMEBUFFER,
                    static_cast<GLuint>(aPreviousReadFramebuffer));
  requireNoGLError("OpenGL state restoration failed after framebuffer readback");
  return aPixels;
}

std::size_t changedRGBPixelCount(const std::vector<std::uint8_t>& a,
                                 const std::vector<std::uint8_t>& b)
{
  require(a.size() == kReadbackBytes && b.size() == kReadbackBytes,
          "framebuffer readback size differs from its fixed bound");
  std::size_t aChangedCount = 0;
  for (std::size_t aPixel = 0;
       aPixel < kReadbackWidth * kReadbackHeight; ++aPixel)
  {
    const std::size_t anOffset = aPixel * kReadbackChannels;
    if (a[anOffset] != b[anOffset]
        || a[anOffset + 1] != b[anOffset + 1]
        || a[anOffset + 2] != b[anOffset + 2])
    {
      ++aChangedCount;
    }
  }
  return aChangedCount;
}

void requireValidCamera(const core3d::scene::CameraSnapshot& theCamera,
                        const char* theMessage)
{
  const double anUpLengthSquared =
      theCamera.up.x * theCamera.up.x
    + theCamera.up.y * theCamera.up.y
    + theCamera.up.z * theCamera.up.z;
  require(finite3(theCamera.eye) && finite3(theCamera.center)
          && finite3(theCamera.up), theMessage);
  require(distanceSquared(theCamera.eye, theCamera.center) > 1.0e-12,
          theMessage);
  require(anUpLengthSquared > 0.25, theMessage);
  require(theCamera.viewportPixels.x == 640
          && theCamera.viewportPixels.y == 480, theMessage);
  require(std::isfinite(theCamera.aspect)
          && std::abs(theCamera.aspect - (640.0 / 480.0)) < 1.0e-6,
          theMessage);
  require(std::isfinite(theCamera.nearPlane)
          && std::isfinite(theCamera.farPlane)
          && theCamera.nearPlane > 0.0
          && theCamera.farPlane > theCamera.nearPlane, theMessage);
  require(theCamera.projection == core3d::scene::Projection::Perspective
          && std::isfinite(theCamera.verticalFovRadians)
          && theCamera.verticalFovRadians > 0.0
          && theCamera.verticalFovRadians < 3.14159265358979323846,
          theMessage);
}

void requireRealCubeScene(const core3d::scene::SceneSnapshot& theScene)
{
  require(core3d::scene::IsValidSceneSnapshot(theScene),
          "shared scene validator rejected the cube snapshot");
  require(theScene.schemaVersion == core3d::scene::kSceneSnapshotSchemaVersion,
          "scene schema version differs");
  require(!theScene.publicationSourceIdentifier.empty(),
          "scene publication identity is empty");
  require(theScene.revisions.snapshot > 0
          && theScene.revisions.documentGeneration > 0
          && theScene.revisions.model > 0
          && theScene.revisions.camera > 0,
          "scene revision authority is incomplete");
  require(std::isfinite(theScene.metersPerUnit)
          && theScene.metersPerUnit > 0.0,
          "scene unit scale is invalid");
  require(!theScene.meshes.empty() && !theScene.instances.empty(),
          "real cube snapshot contains no mesh or instance");

  const core3d::scene::MeshSnapshot* aCubeMesh = nullptr;
  for (const core3d::scene::InstanceSnapshot& anInstance : theScene.instances)
  {
    require(anInstance.meshIndex < theScene.meshes.size(),
            "scene instance references a missing mesh");
    if (anInstance.role == core3d::scene::RenderRole::Model
        && anInstance.visible && anInstance.selectable)
    {
      aCubeMesh = &theScene.meshes[anInstance.meshIndex];
      break;
    }
  }
  require(aCubeMesh != nullptr,
          "snapshot has no visible selectable model instance");
  require(aCubeMesh->localBounds.valid
          && finite3(aCubeMesh->localBounds.minimum)
          && finite3(aCubeMesh->localBounds.maximum),
          "cube mesh bounds are invalid");
  require(aCubeMesh->topology.faceCount == 6
          && aCubeMesh->topology.edgeCount == 12
          && aCubeMesh->topology.vertexCount == 8,
          "model instance is not the real OCCT cube topology");
  require(!aCubeMesh->vertices.empty()
          && aCubeMesh->indices.size() >= 36
          && aCubeMesh->indices.size() % 3 == 0
          && !aCubeMesh->primitives.empty(),
          "cube mesh has no valid triangulated surface");
  for (const std::uint32_t anIndex : aCubeMesh->indices)
  {
    require(anIndex < aCubeMesh->vertices.size(),
            "cube mesh index exceeds its vertex buffer");
  }
  for (const core3d::scene::MeshPrimitive& aPrimitive : aCubeMesh->primitives)
  {
    require(aPrimitive.indexCount > 0
            && aPrimitive.indexCount % 3 == 0
            && aPrimitive.firstIndex <= aCubeMesh->indices.size()
            && aPrimitive.indexCount
                <= aCubeMesh->indices.size() - aPrimitive.firstIndex,
            "cube primitive range is invalid");
  }
  requireValidCamera(theScene.camera, "scene camera authority is invalid");
}

Core3DSceneRenderItemSnapshot *sessionModel(Core3DMacDocumentSession *session)
{
  for (Core3DSceneRenderItemSnapshot *item in session.publication.scene.renderItems)
    if (item.renderRole == Core3DSceneRenderRoleModel && item.visible && item.selectable)
      return item;
  return nil;
}

Core3DTransformInspectorSnapshot *readySessionMeasurement(Core3DMacDocumentSession *session)
{
  __block Core3DTransformInspectorSnapshot *completed = nil;
  Core3DTransformInspectorSnapshot *immediate =
    [session captureTransformMeasurementWithCompletion:^(Core3DTransformInspectorSnapshot *value) {
      completed = value;
    }];
  if (immediate.state != Core3DTransformInspectorStateMeasuring) completed = immediate;
  const auto deadline = [NSDate dateWithTimeIntervalSinceNow:5];
  while (completed == nil && deadline.timeIntervalSinceNow > 0)
    [[NSRunLoop mainRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.005]];
  require(completed != nil && completed.state == Core3DTransformInspectorStateReady
          && completed.canEditPosition, "Mac session did not deliver editable exact measurement");
  return completed;
}

void selectSessionModel(Core3DMacDocumentSession *session, NSString *entity,
                        uint32_t width, uint32_t height)
{
  const auto result = [session selectEntityIdentifier:entity
    expectedPublication:session.publication viewportWidth:width height:height];
  require(result == Core3DMacSessionActionResultChanged
          || result == Core3DMacSessionActionResultUnchanged,
          "Mac session could not select its displayed entity");
}

void requireMacSessionActions(NSOpenGLView *firstView, NSOpenGLView *secondView)
{
  NSOpenGLContext *sentinel = secondView.openGLContext;
  [sentinel makeCurrentContext];
  Core3DMacDocumentSession *first = [[Core3DMacDocumentSession alloc]
    initWithAttachedOpenGLView:firstView];
  Core3DMacDocumentSession *second = [[Core3DMacDocumentSession alloc]
    initWithAttachedOpenGLView:secondView];
  require(first != nil && second != nil, "Mac session initialization failed");
  require([NSOpenGLContext currentContext] == sentinel,
          "Mac session initialization changed the caller context");
  require([first createCubeWithViewportWidth:640 height:480] == Core3DMacSessionActionResultChanged,
          "first Mac session cube was not committed/published");
  require([second createCubeWithViewportWidth:64 height:64] == Core3DMacSessionActionResultChanged,
          "second Mac session cube was not committed/published");
  require([NSOpenGLContext currentContext] == sentinel,
          "interleaved creation did not restore caller context");
  NSString *firstEntity = [sessionModel(first).entityIdentifier copy];
  NSString *secondEntity = [sessionModel(second).entityIdentifier copy];
  require(firstEntity != nil && secondEntity != nil, "session cube identity missing");
  require([second selectEntityIdentifier:firstEntity expectedPublication:first.publication
    viewportWidth:64 height:64] == Core3DMacSessionActionResultStale,
    "Mac session admitted another document's publication");
  selectSessionModel(first, firstEntity, 640, 480);
  selectSessionModel(second, secondEntity, 64, 64);
  Core3DTransformInspectorSnapshot *old = readySessionMeasurement(first);
  Core3DTransformInspectorSnapshot *latest = readySessionMeasurement(first);
  Core3DTransformInspectorSnapshot *foreign = readySessionMeasurement(second);
  const double originalX = latest.position.x;
  const double movedX = originalX + 25;
  const auto secondRevision = second.publication.scene.revisions.modelRevision;
  require([first commitPositionValue:movedX axis:Core3DTransformInspectorAxisX expectedMeasurement:old]
            == Core3DTransformInspectorPositionCommitResultStale,
          "old Mac text-field measurement was accepted");
  require([first commitPositionValue:movedX axis:Core3DTransformInspectorAxisX expectedMeasurement:foreign]
            == Core3DTransformInspectorPositionCommitResultStale,
          "foreign measurement was accepted");
  require([first commitPositionValue:movedX axis:Core3DTransformInspectorAxisX expectedMeasurement:latest]
            == Core3DTransformInspectorPositionCommitResultCommitted,
          "foreign refusal consumed the valid native measurement");
  require(std::abs(readySessionMeasurement(first).position.x - movedX) < 1e-9,
          "Mac session numeric edit did not change the native position");
  require(second.publication.scene.revisions.modelRevision == secondRevision,
          "first session edit changed second document authority");
  __block NSInteger effectOrder = 0;
  __block BOOL wrongContext = NO;
  first.selectionRefreshHandler = ^(Core3DMacScenePublication *) {
    if ([NSOpenGLContext currentContext] != sentinel) wrongContext = YES;
    effectOrder = effectOrder * 10 + 1;
  };
  first.renderRequestHandler = ^(Core3DMacScenePublication *) {
    if ([NSOpenGLContext currentContext] != sentinel) wrongContext = YES;
    effectOrder = effectOrder * 10 + 2;
  };
  require([first performHistory:Core3DMacHistoryDirectionUndo].outcome == Core3DMacHistoryOutcomeChanged,
          "Mac session Undo did not change native history");
  require(effectOrder == 12 && !wrongContext, "Mac history effects/context were out of order");
  selectSessionModel(first, firstEntity, 640, 480);
  require(std::abs(readySessionMeasurement(first).position.x - originalX) < 1e-9,
          "Mac session Undo did not restore the original position");
  require([first performHistory:Core3DMacHistoryDirectionRedo].outcome == Core3DMacHistoryOutcomeChanged,
          "Mac session Redo did not restore the move");
  selectSessionModel(first, firstEntity, 640, 480);
  require(std::abs(readySessionMeasurement(first).position.x - movedX) < 1e-9,
          "Mac session Redo restored the wrong position");
  Core3DMacScenePublication *oldPublication = second.publication;
  require([second refreshPublicationWithWidth:0 height:64] != Core3DMacSessionActionResultUnchanged
          && second.publication == nil, "failed publication retained a stale available pair");
  require([second selectEntityIdentifier:secondEntity expectedPublication:oldPublication
    viewportWidth:64 height:64] == Core3DMacSessionActionResultStale,
    "failed refresh left an old selection lease usable");
  require([second refreshPublicationWithWidth:64 height:64] == Core3DMacSessionActionResultUnchanged,
          "valid publication could not recover after failed refresh");
  __block BOOL renderAfterClose = NO;
  __weak Core3DMacDocumentSession *weakFirst = first;
  first.selectionRefreshHandler = ^(Core3DMacScenePublication *) { [weakFirst close]; };
  first.renderRequestHandler = ^(Core3DMacScenePublication *) { renderAfterClose = YES; };
  (void)[first performHistory:Core3DMacHistoryDirectionUndo];
  require(first.closed && !renderAfterClose,
          "reentrant close did not suppress stale history effects");
  require([second close] == Core3DMacSessionActionResultChanged,
          "Mac session close did not release owned native state");
  require([NSOpenGLContext currentContext] == sentinel,
          "Mac session close changed the caller context");
  std::puts("PASS: Mac sessions create/select/edit/history, reject stale authority and isolate contexts");
}

int runProbe()
{
  // These public values must link to real implementations on Mac without the
  // UIKit controller. Exercise validated immutable editing through that header.
  NSArray<Class> *sharedValueClasses = @[
    [Core3DProfileCurveVertex class], [Core3DProfileCurveSegment class],
    [Core3DProfileCurveLoop class], [Core3DSweepPathSegment class],
    [Core3DSweepDefinition class], [Core3DRectangularLoftStation class],
    [Core3DRectangularLoftStationEdit class], [Core3DRectangularLoftDefinition class],
    [Core3DSavedCutSourceCoordinate class], [Core3DCylindricalCutDefinition class],
    [Core3DProfileDefinition class], [Core3DEnclosureDefinition class]
  ];
  require(sharedValueClasses.count == 12, "shared value implementations missing");
  Core3DEnclosureDefinition *enclosure = [[Core3DEnclosureDefinition alloc]
    initWithWidth:80 depth:60 height:30 wall:2 floor:2 cornerRadius:4
    plane:Core3DProfilePlaneXY metersPerUnit:0.001];
  Core3DEnclosureDefinition *wider = [enclosure
    definitionByUpdatingDimension:Core3DEnclosureDimensionWidth value:100];
  require(enclosure != nil && wider != nil && enclosure.width == 80
          && wider.width == 100 && wider.depth == enclosure.depth
          && wider.metersPerUnit == enclosure.metersPerUnit,
          "Mac shared recipe edit lost immutability, dimensions or units");
  require([enclosure definitionByUpdatingDimension:Core3DEnclosureDimensionWall value:100] == nil,
          "Mac shared recipe admitted an invalid wall");
  CGPoint center = CGPointMake(2, 3);
  NSValue *boxedCenter = [NSValue valueWithBytes:&center objCType:@encode(CGPoint)];
  Core3DProfileDefinition *profile = [[Core3DProfileDefinition alloc]
    initWithPoints:@[] circleCenter:boxedCenter outerRadius:10 innerRadius:2
    holeCenters:@[] holeRadii:@[] plane:Core3DProfilePlaneXY
    parameter:20 revolve:NO metersPerUnit:0.001];
  require(profile != nil && profile.circleCenter != nil,
          "Mac shared profile could not decode/encode Foundation CGPoint storage");
  CGPoint roundTrip{};
  [profile.circleCenter getValue:&roundTrip size:sizeof(roundTrip)];
  require(roundTrip.x == center.x && roundTrip.y == center.y
          && profile.outerRadius == 10 && profile.innerRadius == 2,
          "Mac profile round-trip changed its center or radii");


  require([NSThread isMainThread],
          "shared viewer probe must run on the AppKit main thread");
  [NSApplication sharedApplication];
  [NSApp setActivationPolicy:NSApplicationActivationPolicyProhibited];

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
  const NSOpenGLPixelFormatAttribute anAttributes[] = {
    NSOpenGLPFAOpenGLProfile, NSOpenGLProfileVersion3_2Core,
    NSOpenGLPFAColorSize, 24,
    NSOpenGLPFAAlphaSize, 8,
    NSOpenGLPFADepthSize, 24,
    NSOpenGLPFAStencilSize, 8,
    NSOpenGLPFADoubleBuffer,
    NSOpenGLPFAAccelerated,
    0
  };

  NSOpenGLPixelFormat* aPixelFormat = nil;
  NSOpenGLView* aPrimaryView = nil;
  NSOpenGLContext* aPrimaryContext = nil;
  NSWindow* aPrimaryWindow = nil;
  NSOpenGLView* anAlienView = nil;
  NSOpenGLContext* anAlienContext = nil;
  NSWindow* anAlienWindow = nil;
#pragma clang diagnostic pop

  core3d::Core3DViewer aViewer;
  bool aViewerReleased = false;
  bool didCleanup = false;
  const auto aCleanup = [&]() noexcept {
    if (didCleanup) return;
    didCleanup = true;
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    if (!aViewerReleased && aPrimaryContext != nil)
    {
      [aPrimaryContext makeCurrentContext];
      aViewer.release();
      aViewerReleased = true;
    }
    [NSOpenGLContext clearCurrentContext];
    [aPrimaryContext clearDrawable];
    [anAlienContext clearDrawable];
    [aPrimaryView setOpenGLContext:nil];
    [anAlienView setOpenGLContext:nil];
#pragma clang diagnostic pop
    [aPrimaryWindow setContentView:nil];
    [anAlienWindow setContentView:nil];
    [aPrimaryWindow orderOut:nil];
    [anAlienWindow orderOut:nil];
    [aPrimaryWindow close];
    [anAlienWindow close];
    aPrimaryContext = nil;
    anAlienContext = nil;
    aPrimaryView = nil;
    anAlienView = nil;
    aPixelFormat = nil;
    aPrimaryWindow = nil;
    anAlienWindow = nil;
  };

  try
  {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    aPixelFormat =
      [[NSOpenGLPixelFormat alloc] initWithAttributes:anAttributes];
    require(aPixelFormat != nil, "NSOpenGLPixelFormat creation failed");

    aPrimaryView =
      [[NSOpenGLView alloc] initWithFrame:NSMakeRect(0, 0, 640, 480)
                              pixelFormat:aPixelFormat];
    aPrimaryContext =
      [[NSOpenGLContext alloc] initWithFormat:aPixelFormat shareContext:nil];
    anAlienView =
      [[NSOpenGLView alloc] initWithFrame:NSMakeRect(0, 0, 64, 64)
                              pixelFormat:aPixelFormat];
    anAlienContext =
      [[NSOpenGLContext alloc] initWithFormat:aPixelFormat shareContext:nil];
    require(aPrimaryView != nil && aPrimaryContext != nil
            && anAlienView != nil && anAlienContext != nil,
            "owned OpenGL view/context creation failed");
    [aPrimaryView setOpenGLContext:aPrimaryContext];
    [anAlienView setOpenGLContext:anAlienContext];

    aPrimaryWindow =
      [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 640, 480)
                                 styleMask:NSWindowStyleMaskBorderless
                                   backing:NSBackingStoreBuffered
                                     defer:NO];
    anAlienWindow =
      [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 64, 64)
                                 styleMask:NSWindowStyleMaskBorderless
                                   backing:NSBackingStoreBuffered
                                     defer:NO];
    require(aPrimaryWindow != nil && anAlienWindow != nil,
            "hidden NSWindow creation failed");
    [aPrimaryWindow setReleasedWhenClosed:NO];
    [anAlienWindow setReleasedWhenClosed:NO];
    [aPrimaryWindow setContentView:aPrimaryView];
    [anAlienWindow setContentView:anAlienView];
    [aPrimaryContext setView:aPrimaryView];
    [anAlienContext setView:anAlienView];

    [NSOpenGLContext clearCurrentContext];
    require(!aViewer.InitViewer(aPrimaryView),
            "viewer accepted a view with no current native context");

    [anAlienContext makeCurrentContext];
    [anAlienContext update];
    require([NSOpenGLContext currentContext] == anAlienContext
            && [anAlienContext view] == anAlienView,
            "alien context setup failed");
    require(!aViewer.InitViewer(aPrimaryView),
            "viewer accepted a current context owned by another view");

    [aPrimaryContext makeCurrentContext];
    [aPrimaryContext update];
    require(!aViewer.InitViewer(nullptr),
            "viewer accepted a nil native view");
    require(aViewer.V3dViewer().IsNull()
            && aViewer.ActiveView().IsNull()
            && aViewer.AisContext().IsNull(),
            "rejected initialization left partial OCCT graphics state");

    require([NSOpenGLContext currentContext] == aPrimaryContext
            && [aPrimaryContext view] == aPrimaryView,
            "primary context is not current for its exact view");
    require(aViewer.InitViewer(aPrimaryView),
            "real shared Core3DViewer initialization failed");
#pragma clang diagnostic pop

    require(!aViewer.V3dViewer().IsNull()
            && !aViewer.ActiveView().IsNull()
            && !aViewer.AisContext().IsNull(),
            "successful initialization did not create native viewer state");
    {
      const Handle(OpenGl_GraphicDriver) aDriver =
        Handle(OpenGl_GraphicDriver)::DownCast(aViewer.V3dViewer()->Driver());
      require(!aDriver.IsNull() && !aDriver->GetSharedContext().IsNull(),
              "shared viewer has no real OCCT OpenGL context");
      require(aDriver->GetSharedContext()->RenderingContext() == aPrimaryContext,
              "OCCT did not retain the exact host NSOpenGLContext identity");
    } // Do not retain the driver across viewer/window teardown.

    requireNoGLError("OpenGL state was dirty before the empty render");
    require(aViewer.RenderFrame(),
            "real shared viewer did not render its empty baseline frame");
    requireNoGLError("empty shared-viewer render produced an OpenGL error");
    const std::vector<std::uint8_t> anEmptyFrame =
      readOwnedBackFramebuffer();

    aViewer.addPrimitive(PrimitiveTypeCube);
    require(aViewer.RenderFrame(),
            "real shared viewer did not render its committed cube frame");
    requireNoGLError("populated shared-viewer render produced an OpenGL error");
    const std::vector<std::uint8_t> aPopulatedFrame =
      readOwnedBackFramebuffer();
    const std::size_t aChangedPixelCount =
      changedRGBPixelCount(anEmptyFrame, aPopulatedFrame);
    require(aChangedPixelCount >= 256,
            "committed cube produced no meaningful visible framebuffer change");
    const auto aScene = aViewer.captureSceneSnapshot(640, 480);
    require(aScene != nullptr, "shared viewer did not publish a cube scene");
    requireRealCubeScene(*aScene);

    const auto aFrame = aViewer.captureSceneFrameSnapshot(640, 480);
    require(aFrame.has_value(),
            "shared viewer did not publish camera-only frame authority");
    require(aFrame->publicationSourceIdentifier
              == aScene->publicationSourceIdentifier
            && aFrame->revisions.documentGeneration
              == aScene->revisions.documentGeneration
            && aFrame->revisions.model == aScene->revisions.model
            && aFrame->revisions.camera >= aScene->revisions.camera,
            "camera frame is detached from the committed scene authority");
    requireValidCamera(aFrame->camera, "camera frame authority is invalid");

    const auto undo = aViewer.performHistory(core3d::NativeHistoryDirection::Undo);
    require(undo.outcome == core3d::NativeHistoryOutcome::HistoryChanged
            && undo.documentRedrawn && undo.refreshSelection && undo.requestRender
            && !undo.primaryInteractionCancelled && !undo.reconcileBooleanTool,
            "shared native Undo did not report its committed transition");
    const auto anUndoneScene = aViewer.captureSceneSnapshot(640, 480);
    require(anUndoneScene != nullptr
            && std::none_of(anUndoneScene->instances.begin(), anUndoneScene->instances.end(),
                [](const auto& instance) { return instance.role == core3d::scene::RenderRole::Model; }),
            "shared native Undo left the created cube in the scene");
    const auto noHistory = aViewer.performHistory(core3d::NativeHistoryDirection::Undo);
    require(noHistory.outcome == core3d::NativeHistoryOutcome::NoHistory
            && !noHistory.documentRedrawn && noHistory.refreshSelection
            && noHistory.requestRender, "empty native history reported a mutation");
    const auto redo = aViewer.performHistory(core3d::NativeHistoryDirection::Redo);
    require(redo.outcome == core3d::NativeHistoryOutcome::HistoryChanged
            && redo.documentRedrawn && redo.refreshSelection && redo.requestRender,
            "shared native Redo did not restore the committed creation");
    const auto aRedoneScene = aViewer.captureSceneSnapshot(640, 480);
    require(aRedoneScene != nullptr, "shared native Redo did not publish a scene");
    requireRealCubeScene(*aRedoneScene);
    require(aRedoneScene->instances.size() == aScene->instances.size()
            && aRedoneScene->instances.front().entityIdentifier
                == aScene->instances.front().entityIdentifier
            && aRedoneScene->revisions.model > aScene->revisions.model,
            "shared native history lost cube identity or model revision authority");

    // Real synchronous tool notifications replace both interactors during
    // cancellation. History must keep the executing owner alive, refuse the
    // stale transition, and preserve the committed cube for a subsequent Undo.
    const auto unionAction = core3d::BooleanAction::BooleanUnion;
    const auto unionTool = core3d::PrimitiveManipulatorType::PrimitiveGizmoTypeUnion;
    for (const bool useHistory : {true, false}) {
      aViewer.getObjectInteractor()->setManipulatorType(unionTool);
      require(aViewer.getObjectInteractor()->beginBoolean(unionAction),
              "could not begin Boolean callback replacement fixture");
      const std::weak_ptr<core3d::ObjectInteractor> oldOwner = aViewer.getObjectInteractor();
      bool notified = false, replaced = false, aliveInCallback = false;
      aViewer.setBooleanPreviewStateChangedCallback([&] {
        if (notified) return;
        notified = true;
        replaced = aViewer.redrawDocument();
        aliveInCallback = !oldOwner.expired();
      });
      if (useHistory) {
        const auto refused = aViewer.performHistory(core3d::NativeHistoryDirection::Undo);
        require(refused.outcome == core3d::NativeHistoryOutcome::Unavailable
                && !refused.documentRedrawn && !refused.refreshSelection
                && !refused.requestRender && !refused.reconcileBooleanTool,
                "history continued after a callback replaced its owners");
      } else {
        require(!aViewer.retireBooleanAction(unionAction),
                "Boolean retirement claimed success for a replaced owner");
      }
      aViewer.setBooleanPreviewStateChangedCallback({});
      require(notified && replaced && aliveInCallback && oldOwner.expired(),
              "cancel callback did not preserve then release its executing owner");
      const auto preserved = aViewer.captureSceneSnapshot(640, 480);
      require(preserved != nullptr, "callback replacement lost scene authority");
      requireRealCubeScene(*preserved);
    }
    {
      aViewer.getObjectInteractor()->setManipulatorType(unionTool);
      const std::weak_ptr<core3d::ObjectInteractor> oldOwner = aViewer.getObjectInteractor();
      bool notified = false, replaced = false, aliveInCallback = false;
      aViewer.setBooleanPreviewStateChangedCallback([&] {
        if (notified) return;
        notified = true;
        // Retire the just-created ledger before requesting a permitted redraw.
        aViewer.getObjectInteractor()->cancelBoolean(unionAction);
        replaced = aViewer.redrawDocument();
        aliveInCallback = !oldOwner.expired();
      });
      require(aViewer.restoreBooleanAction(unionAction)
                == core3d::NativeBooleanRetention::Unavailable,
              "Boolean restoration claimed retention after owner replacement");
      aViewer.setBooleanPreviewStateChangedCallback({});
      require(notified && replaced && aliveInCallback && oldOwner.expired(),
              "begin callback did not preserve then release its executing owner");
    }
    require(aViewer.performHistory(core3d::NativeHistoryDirection::Undo).outcome
              == core3d::NativeHistoryOutcome::HistoryChanged,
            "callback refusal consumed the committed Undo entry");
    require(aViewer.performHistory(core3d::NativeHistoryDirection::Redo).outcome
              == core3d::NativeHistoryOutcome::HistoryChanged,
            "callback refusal broke subsequent Redo");

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    [anAlienContext makeCurrentContext];
    require(!aViewer.RenderFrame(),
            "render accepted a different current NSOpenGLContext");
    require(!aViewer.InitViewer(aPrimaryView),
            "reinitialization accepted a context owned by another view");
    [aPrimaryContext makeCurrentContext];
    require(aViewer.RenderFrame(),
            "render did not recover after restoring the owned context");

    aViewer.release();
    aViewerReleased = true;
    require([NSOpenGLContext currentContext] == aPrimaryContext
            && [aPrimaryContext view] == aPrimaryView,
            "viewer teardown retired the application-owned current context");
#pragma clang diagnostic pop
    require(aViewer.V3dViewer().IsNull()
            && aViewer.ActiveView().IsNull()
            && aViewer.AisContext().IsNull(),
            "viewer teardown retained native graphics handles");
    aViewer.release(); // Repeated terminal teardown remains harmless.

    requireMacSessionActions(aPrimaryView, anAlienView);

    aCleanup();
    std::printf("PASS: shared Core3DViewer lifecycle, cube render/snapshot, history and callback ownership; changed pixels: %zu\n",
                aChangedPixelCount);
    return 0;
  }
  catch (...)
  {
    aCleanup();
    throw;
  }
}
} // namespace

int main()
{
  @autoreleasepool
  {
    try
    {
      return runProbe();
    }
    catch (const Standard_Failure& theFailure)
    {
      std::fprintf(stderr, "FAIL (OCCT): %s\n", theFailure.GetMessageString());
    }
    catch (const std::exception& theFailure)
    {
      std::fprintf(stderr, "FAIL: %s\n", theFailure.what());
    }
    return 1;
  }
}
