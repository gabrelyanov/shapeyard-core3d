#import <Cocoa/Cocoa.h>

#include <AIS_InteractiveContext.hxx>
#include <AIS_DisplayMode.hxx>
#include <AIS_Shape.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include "../../Core3D/OCCTKit/Core3DCocoa_Window.hxx"
#include <AIS_ManipulatorOwner.hxx>
#include <Image_PixMap.hxx>
#include <OpenGl_Context.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Standard_Failure.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>

#include <cstdio>
#include <exception>
#include <stdexcept>

namespace
{
void require(bool theCondition, const char* theMessage)
{
  if (!theCondition)
  {
    throw std::runtime_error(theMessage);
  }
}

int runProbe()
{
  require([NSThread isMainThread], "probe must own AppKit objects on the main thread");

  [NSApplication sharedApplication];
  [NSApp setActivationPolicy:NSApplicationActivationPolicyProhibited];

  // Exercise the app adapter's actual MRC retain/release boundary from ARC.
  // A duplicate derived-constructor retain leaves this weak reference alive.
  __weak NSView* aRetiredView = nil;
  @autoreleasepool
  {
    NSView* anOwnedView = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, 32, 24)];
    aRetiredView = anOwnedView;
    {
      Handle(Core3DCocoa_Window) aWrapper = new Core3DCocoa_Window(anOwnedView);
      Standard_Integer aWidth = 0, aHeight = 0;
      aWrapper->Size(aWidth, aHeight);
      require(aWidth == 32 && aHeight == 24, "app window adapter size differs");
      [anOwnedView setFrameSize:NSMakeSize(64, 48)];
      aWrapper->DoResize();
      aWrapper->Size(aWidth, aHeight);
      require(aWidth == 64 && aHeight == 48, "app window adapter resize differs");
    }
    anOwnedView = nil;
  }
  require(aRetiredView == nil, "app window adapter leaked its retained NSView");

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
  NSOpenGLView* aGLView = nil;
  NSOpenGLContext* aGLContext = nil;
  NSWindow* aWindow = nil;
#pragma clang diagnostic pop

  Handle(OpenGl_GraphicDriver) aDriver;
  Handle(V3d_Viewer) aViewer;
  Handle(V3d_View) aView;
  Handle(AIS_InteractiveContext) anAISContext;
  Handle(AIS_Shape) aBox;
  Handle(Core3DCocoa_Window) aCocoaWindow;

  bool didCleanup = false;
  bool cleanupPassed = true;
  const auto aCleanup = [&]() noexcept {
    if (didCleanup) return cleanupPassed;
    didCleanup = true;
    // Continue retiring owned resources if one OCCT removal reports a failure.
    const auto attempt = [&](const auto& operation) {
      try { operation(); }
      catch (...) { cleanupPassed = false; }
    };
    if (!anAISContext.IsNull())
    {
      attempt([&] { anAISContext->RemoveAll(Standard_False); });
      attempt([&] { anAISContext->ClearSelected(Standard_False); });
      attempt([&] { anAISContext->ClearDetected(Standard_False); });
    }
    if (!aView.IsNull()) attempt([&] { aView->Remove(); });
    aBox.Nullify();
    anAISContext.Nullify();
    aView.Nullify();
    if (!aViewer.IsNull()) attempt([&] { aViewer->Remove(); });
    aViewer.Nullify();
    aDriver.Nullify();
    aCocoaWindow.Nullify();

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    [NSOpenGLContext clearCurrentContext];
    [aGLContext clearDrawable];
    [aGLView setOpenGLContext:nil];
#pragma clang diagnostic pop
    [aWindow setContentView:nil];
    [aWindow orderOut:nil];
    [aWindow close];
    aGLContext = nil;
    aGLView = nil;
    aPixelFormat = nil;
    aWindow = nil;
    return cleanupPassed;
  };

  try
  {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    aPixelFormat =
      [[NSOpenGLPixelFormat alloc] initWithAttributes:anAttributes];
    require(aPixelFormat != nil, "NSOpenGLPixelFormat creation failed");

    aGLView =
      [[NSOpenGLView alloc] initWithFrame:NSMakeRect(0, 0, 640, 480)
                              pixelFormat:aPixelFormat];
    aGLContext =
      [[NSOpenGLContext alloc] initWithFormat:aPixelFormat shareContext:nil];
    require(aGLView != nil && aGLContext != nil, "owned OpenGL view/context creation failed");
    [aGLView setOpenGLContext:aGLContext];

    aWindow =
      [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 640, 480)
                                 styleMask:NSWindowStyleMaskBorderless
                                   backing:NSBackingStoreBuffered
                                     defer:NO];
    require(aWindow != nil, "hidden NSWindow creation failed");
    [aWindow setReleasedWhenClosed:NO];
    [aWindow setContentView:aGLView];
    [aGLContext setView:aGLView];
    [aGLContext makeCurrentContext];
    [aGLContext update];
    require(![aWindow isVisible], "probe window unexpectedly became visible");
    require([aGLContext view] == aGLView, "NSOpenGLContext did not attach to owned view");
    require([NSOpenGLContext currentContext] == aGLContext,
            "owned NSOpenGLContext is not current");
#pragma clang diagnostic pop
    Handle(Aspect_DisplayConnection) aDisplay = new Aspect_DisplayConnection();
    aDriver = new OpenGl_GraphicDriver(aDisplay, Standard_False);
    aViewer = new V3d_Viewer(aDriver);
    aViewer->SetDefaultLights();
    aViewer->SetLightOn();
    anAISContext = new AIS_InteractiveContext(aViewer);
    aView = aViewer->CreateView();
    aView->SetBackgroundColor(Quantity_NOC_BLACK);

    aCocoaWindow = new Core3DCocoa_Window(aGLView);
    aView->SetWindow(aCocoaWindow, aGLContext); // Aspect_RenderingContext is NSOpenGLContext* here.
    require(!aDriver->GetSharedContext().IsNull(), "OCCT did not create a render context");
    require(aDriver->GetSharedContext()->RenderingContext() == aGLContext,
            "OCCT did not retain the supplied NSOpenGLContext identity");

    const TopoDS_Shape aShape = BRepPrimAPI_MakeBox(20.0, 30.0, 40.0).Shape();
    BRepMesh_IncrementalMesh aMesher(aShape, 0.5, Standard_False, 0.25, Standard_True);
    require(aMesher.IsDone(), "box meshing failed");
    aBox = new AIS_Shape(aShape);
    static_assert(AIS_MM_ScalingUniform == 4 && AIS_MM_TranslationPlane == 5
                  && AIS_MM_MirroringPlaneNeg == 6 && AIS_MM_MirroringPlanePos == 7,
                  "desktop dependency must preserve the app's gizmo mode contract");
    for (AIS_ManipulatorMode aMode : {AIS_MM_Translation, AIS_MM_Rotation,
           AIS_MM_Scaling, AIS_MM_ScalingUniform, AIS_MM_TranslationPlane,
           AIS_MM_MirroringPlaneNeg, AIS_MM_MirroringPlanePos})
    {
      Handle(AIS_ManipulatorOwner) anOwner = new AIS_ManipulatorOwner(aBox, 0, aMode);
      require(anOwner->Mode() == aMode, "compiled OCCT owner changed app gizmo mode");
    }
    anAISContext->Display(aBox, Standard_False);
    anAISContext->SetDisplayMode(aBox, AIS_Shaded, Standard_False);
    aView->FitAll(0.05, Standard_False);
    aView->ZFitAll();
    aView->Redraw();

    Image_PixMap aRenderedImage;
    require(aView->ToPixMap(aRenderedImage, 160, 120, Graphic3d_BT_RGB),
            "OCCT framebuffer readback failed");
    Standard_Size aLitPixelCount = 0;
    for (Standard_Integer aY = 0; aY < 120; ++aY)
    {
      for (Standard_Integer anX = 0; anX < 160; ++anX)
      {
        const Quantity_Color aColor = aRenderedImage.PixelColor(anX, aY).GetRGB();
        if (aColor.Red() > 0.08 || aColor.Green() > 0.08 || aColor.Blue() > 0.08)
        {
          ++aLitPixelCount;
        }
      }
    }
    require(aLitPixelCount > 100, "rendered box produced no meaningful non-background pixels");

    Standard_Integer aPixelX = -1;
    Standard_Integer aPixelY = -1;
    aView->Convert(10.0, 15.0, 20.0, aPixelX, aPixelY);
    Standard_Integer aWidth = 0;
    Standard_Integer aHeight = 0;
    aCocoaWindow->Size(aWidth, aHeight);
    require(aPixelX >= 0 && aPixelX < aWidth && aPixelY >= 0 && aPixelY < aHeight,
            "projected box center is outside the native view");

    anAISContext->MoveTo(aPixelX, aPixelY, aView, Standard_False);
    require(anAISContext->HasDetected(), "projected box center was not detected");
    require(anAISContext->DetectedInteractive().get() == aBox.get(),
            "detected native AIS object identity differs from displayed box");
    anAISContext->SelectDetected();
    require(anAISContext->NbSelected() == 1 && anAISContext->IsSelected(aBox),
            "detected box did not become the sole selected object");
    require(anAISContext->FirstSelectedObject().get() == aBox.get(),
            "selected native AIS object identity differs from displayed box");

    anAISContext->Remove(aBox, Standard_False);
    require(!anAISContext->HasDetected(), "detection survived object removal");
    require(anAISContext->NbSelected() == 0 && !anAISContext->IsSelected(aBox),
            "selection survived object removal");
    aView->Redraw();

    require(aCleanup(), "OCCT graphics resource cleanup failed");
    std::printf("Rendered non-background pixels: %zu; projected center: %d,%d in %dx%d\n",
                aLitPixelCount, aPixelX, aPixelY, aWidth, aHeight);
    std::puts("PASS: native macOS OCCT display/picking, app window resize/retirement and gizmo mode transport");
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
    catch (...)
    {
      std::fputs("FAIL: unknown exception\n", stderr);
    }
    return 1;
  }
}
