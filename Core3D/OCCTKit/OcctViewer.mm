// Copyright (c) 2017 OPEN CASCADE SAS
//
// This file is part of the examples of the Open CASCADE Technology software library.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE

#include "OcctViewer.h"
#include <Aspect_Window.hxx>
#include <Graphic3d_Camera.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <cmath>
#include "OcctDocument.h"
#include "Core3DSTEPExchangeLock.h"
#include "Core3DNativeTangentBuffers.hxx"
#include "PatternAllLabelAuthority.hxx"
#include "TextReliefPersistence.hxx"
#include <AIS_ColoredShape.hxx>
#include <AIS_ListOfInteractive.hxx>
#include <Bnd_Box.hxx>
#include <Graphic3d_Camera.hxx>
#include <AIS_ConnectedInteractive.hxx>
#include <Graphic3d_ShaderVariable.hxx>
#include <OpenGl_ShaderProgram.hxx>
#include <OpenGl_Context.hxx>
#include <Prs3d_ShadingAspect.hxx>

namespace {
// A proxy uniform is consumed by OCCT after use, so republish it at every
// native draw boundary (including false). Never change the material, shader
// identity, selectable owner or presentation lifetime to switch lesson light.
void PrepareNativeLessonLighting(const Handle(AIS_InteractiveContext)& context,
                                const Handle(OpenGl_Context)& gl, bool enabled) {
    if (context.IsNull() || gl.IsNull()) return;
    const auto prepareProgram = [&](const Handle(Graphic3d_ShaderProgram)& program) {
        if (program.IsNull() || !program->GetId().StartsWith("shapeyard-data-maps-v1-")) return;
        const Standard_ShortReal value = enabled ? 1.0f : 0.0f;
        // A presentation may remain unrendered (occlusion/erasure). Update an
        // existing pending value instead of growing its queue each frame.
        for (const auto& variable : program->Variables()) {
            if (!variable.IsNull() && variable->Name() == "syLessonLighting") {
                auto* uniform = dynamic_cast<Graphic3d_UniformFloat*>(variable->Value());
                if (uniform) uniform->Value = value;
                return;
            }
        }
        program->PushVariable("syLessonLighting", value);
    };
    const auto prepare = [&](const Handle(Prs3d_Drawer)& drawer) {
        if (drawer.IsNull() || drawer->ShadingAspect().IsNull()) return;
        const auto aspect = drawer->ShadingAspect()->Aspect();
        if (aspect.IsNull() || aspect->ShaderProgram().IsNull()) return;
        const auto program = aspect->ShaderProgram();
        prepareProgram(program);
        // OCCT reuses a GPU program by ID after a new material creates a
        // new proxy. ApplyVariables reads the GPU program's retained first
        // proxy, not necessarily this drawer's current one. Publish to both,
        // without replacing/binding/releasing any native graphics resource.
        Handle(OpenGl_ShaderProgram) cached;
        if (gl->GetResource(program->GetId(), cached) && !cached.IsNull())
            prepareProgram(cached->Proxy());
    };
    AIS_ListOfInteractive objects;
    context->DisplayedObjects(AIS_KOI_Shape, -1, objects);
    for (AIS_ListIteratorOfListOfInteractive it(objects); it.More(); it.Next()) {
        Handle(AIS_InteractiveObject) object = it.Value();
        // Connected duplicates share the source presentation and shader proxy.
        // Follow that existing ownership, without changing/rebuilding it.
        for (int depth = 0; depth < 8; ++depth) {
            const auto connected = Handle(AIS_ConnectedInteractive)::DownCast(object);
            if (connected.IsNull() || !connected->HasConnection()) break;
            object = connected->ConnectedTo();
        }
        auto shape = Handle(AIS_Shape)::DownCast(object);
        if (shape.IsNull()) continue;
        prepare(shape->Attributes());
        const auto coloured = Handle(AIS_ColoredShape)::DownCast(shape);
        if (!coloured.IsNull()) {
            for (AIS_DataMapOfShapeDrawer::Iterator face(coloured->CustomAspectsMap()); face.More(); face.Next())
                prepare(face.Value());
        }
    }
}
} // namespace


#import <TargetConditionals.h>
#if defined(TARGET_OS_OSX) && TARGET_OS_OSX
  #import <AppKit/NSOpenGL.h>
#else
  #import <UIKit/UIKit.h>
#endif

#include <OpenGl_GraphicDriver.hxx>
#include <OpenGl_Context.hxx>
#include <Graphic3d_CLight.hxx>
#include <Standard_Failure.hxx>

#include <AIS_ConnectedInteractive.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <BRep_Builder.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include "Core3DCocoa_Window.hxx"
#include <Message.hxx>
#include <Message_Messenger.hxx>
#include <Prs3d_Drawer.hxx>
#include <StdPrs_ToolTriangulatedShape.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <TDF_Tool.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelMap.hxx>
#include <TDF_MapIteratorOfLabelMap.hxx>
#include <Transfer_TransientProcess.hxx>
#include <XSControl_TransferReader.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_LayerTool.hxx>
#include <XCAFDoc_VisMaterial.hxx>
#include <XCAFPrs_DocumentExplorer.hxx>

#include "BRepPrimAPI_MakeCylinder.hxx"
#include "BRepPrimAPI_MakeBox.hxx"
#include "BRepPrimAPI_MakeSphere.hxx"
#include "BRepAlgoAPI_Cut.hxx"
#include "BRepAlgoAPI_Fuse.hxx"
#include "STEPControl_Writer.hxx"
#include "BRepGProp.hxx"
#include "GProp_GProps.hxx"
#include <TopExp_Explorer.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS.hxx>
#include <algorithm>
#include <SelectMgr_AndOrFilter.hxx>
#include <SelectMgr_EntityOwner.hxx>
#include <StdSelect_ViewerSelector3d.hxx>
#include <StdSelect_BRepOwner.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <gp_Lin.hxx>
#include <Precision.hxx>
#include <vector>

#include <StlAPI_Writer.hxx>
#include <TDataStd_Name.hxx>
#include <TDataStd_Real.hxx>
#include <GP_Quaternion.hxx>

namespace {

constexpr Standard_Size kMaximumDisplayTraversalDepth = 128;

bool CaptureTextReliefConstructionOwners(
    const Handle(TDocStd_Document)& document,
    TDF_LabelMap& owners) noexcept {
    owners.Clear();
    try {
        std::vector<core3d::text_relief::persistence::Record> reliefs;
        std::vector<core3d::bounded_curve::Record> curves;
        if (!core3d::text_relief::persistence::ReadAll(document, reliefs)
            || !core3d::bounded_curve::ReadAll(document, curves)) return false;
        std::map<core3d::bounded_curve::UUID,
            const core3d::bounded_curve::Record*> byFeature;
        for (const auto& curve : curves) {
            if (!curve.value
                || !byFeature.emplace(
                    curve.value->persisted.value.feature, &curve).second)
                return false;
        }
        for (const auto& relief : reliefs) {
            if (!relief.payload) return false;
            for (const auto& reference : relief.payload->value.curves) {
                const auto found = byFeature.find(reference.feature);
                core3d::bounded_curve::Digest digest{};
                if (found == byFeature.end() || found->second == nullptr
                    || !found->second->value
                    || !core3d::bounded_curve::Hash(
                        found->second->value->definitionBytes,
                        core3d::bounded_curve::MaximumDefinitionBytes, digest)
                    || digest != reference.digest
                    || found->second->owner.IsNull()) return false;
                owners.Add(found->second->owner);
            }
        }
        return true;
    } catch (...) {
        owners.Clear();
        return false;
    }
}

#if defined(TARGET_OS_OSX) && TARGET_OS_OSX
bool HasCurrentNativeContext(const Handle(V3d_Viewer)& viewer) noexcept
{
    try {
        if (viewer.IsNull()) return false;
        const auto driver = Handle(OpenGl_GraphicDriver)::DownCast(viewer->Driver());
        if (driver.IsNull()) return false;
        const auto& context = driver->GetSharedContext();
        NSOpenGLContext* current = [NSOpenGLContext currentContext];
        return current != nil && !context.IsNull()
            && context->RenderingContext() == current;
    } catch (...) {
        return false;
    }
}
#endif

Standard_Boolean IsLabelAndLayersVisible(
    const TDF_Label& theLabel,
    const Handle(XCAFDoc_LayerTool)& theLayerTool)
{
    if (theLabel.IsNull())
    {
        return Standard_True;
    }
    if (!XCAFDoc_ColorTool::IsVisible(theLabel))
    {
        return Standard_False;
    }
    if (theLayerTool.IsNull())
    {
        return Standard_True;
    }

    TDF_LabelSequence aLayers;
    if (!theLayerTool->GetLayers(theLabel, aLayers))
    {
        return Standard_True;
    }
    for (TDF_LabelSequence::Iterator aLayer(aLayers);
         aLayer.More(); aLayer.Next())
    {
        if (!theLayerTool->IsVisible(aLayer.Value()))
        {
            return Standard_False;
        }
    }
    return Standard_True;
}

Standard_Boolean IsExplorerPathVisible(
    const XCAFPrs_DocumentExplorer& theExplorer,
    const Handle(XCAFDoc_LayerTool)& theLayerTool)
{
    const Standard_Integer aDepth = theExplorer.CurrentDepth();
    if (aDepth < 0)
    {
        return Standard_False;
    }
    for (Standard_Integer aPathIndex = 0;
         aPathIndex <= aDepth; ++aPathIndex)
    {
        const XCAFPrs_DocumentNode& aPathNode =
            theExplorer.Current(aPathIndex);
        if (!IsLabelAndLayersVisible(aPathNode.Label, theLayerTool)
            || !IsLabelAndLayersVisible(
                aPathNode.RefLabel, theLayerTool))
        {
            return Standard_False;
        }
    }
    return Standard_True;
}

class ImportedDocumentHistoryGuard
{
public:
    explicit ImportedDocumentHistoryGuard(
        const Handle(TDocStd_Document)& theDocument)
    : myDocument(theDocument)
    {
        if (!myDocument.IsNull()) {
            myDocument->ClearUndos();
            myDocument->SetUndoLimit(0);
        }
    }

    ~ImportedDocumentHistoryGuard()
    {
        if (myDocument.IsNull()) {
            return;
        }
        try {
            if (myDocument->HasOpenCommand()) {
                myDocument->AbortCommand();
            }
            myDocument->ClearUndos();
            myDocument->SetUndoLimit(40);
        } catch (...) {
        }
    }

private:
    Handle(TDocStd_Document) myDocument;
};

} // namespace

// =======================================================================
// function : OcctViewer
// purpose  :
// =======================================================================
OcctViewer::OcctViewer()
    : myDoc(myDocumentSession.Document())
{
}

// =======================================================================
// function : ~OcctViewer
// purpose  :
// =======================================================================
OcctViewer::~OcctViewer()
{
    release();
}

// =======================================================================
// function : release
// purpose  :
// =======================================================================
void OcctViewer::release() noexcept
{
    myPreparedTangentArrays.clear();
    myContext.Nullify();
    if (!myView.IsNull()) {
#if defined(TARGET_OS_OSX) && TARGET_OS_OSX
        if (HasCurrentNativeContext(myViewer)) {
#else
        if (EAGLContext.currentContext != nil) {
#endif
            try {
                myView->Remove();
            } catch (const Standard_Failure& failure) {
                NSLog(@"OCCT view teardown failed: %s", failure.GetMessageString());
            } catch (...) {
                NSLog(@"OCCT view teardown failed with an unknown error");
            }
        } else {
            // A controller-level context acquisition failure must not skip the
            // rest of teardown. OCCT handles are still released below; explicit
            // GPU removal is omitted because issuing GL calls without a current
            // context is undefined.
#if defined(TARGET_OS_OSX) && TARGET_OS_OSX
            NSLog(@"Releasing OCCT view without explicit GPU removal: native context is not current");
#else
            NSLog(@"Releasing OCCT view without explicit GPU removal: no EAGL context");
#endif
        }
    }
    myView.Nullify();
    myViewer.Nullify();
    myDocumentSession.Close();
}

// =======================================================================
// function : InitViewer
// purpose  :
// =======================================================================
bool OcctViewer::InitViewer (Core3DPlatformView* theWin)
{
#if defined(TARGET_OS_OSX) && TARGET_OS_OSX
    NSOpenGLContext* aRendCtx = [NSOpenGLContext currentContext];
    if (theWin == NULL || aRendCtx == NULL || [aRendCtx view] != theWin)
    {
        NSLog(@"Error: The native view's NSOpenGL context must be current!");
        return false;
    }
#else
    EAGLContext* aRendCtx = [EAGLContext currentContext];
    if (theWin == NULL || aRendCtx   == NULL)
    {
        NSLog(@"Error: No active EAGL context!");
        return false;
    }
#endif
    if (!myView.IsNull())
    {
#if defined(TARGET_OS_OSX) && TARGET_OS_OSX
        const auto window = Handle(Core3DCocoa_Window)::DownCast(myView->Window());
        if (window.IsNull() || window->HView() != theWin
            || !HasCurrentNativeContext(myViewer)) return false;
#endif
        Resize();
        return true;
    }
    
    Handle(Aspect_DisplayConnection) aDisplayConnection = new Aspect_DisplayConnection();
#if defined(TARGET_OS_OSX) && TARGET_OS_OSX
    // The AppKit host owns the current NSOpenGLContext. Prevent OCCT from
    // creating an implicit driver context and bind that host context below.
    Handle(OpenGl_GraphicDriver) aGraphicDriver =
        new OpenGl_GraphicDriver (aDisplayConnection, Standard_False);
#else
    Handle(OpenGl_GraphicDriver) aGraphicDriver = new OpenGl_GraphicDriver (aDisplayConnection);
#endif
    // GLView owns drawable presentation and CADisplayLink pacing. OCCT only
    // renders into the framebuffer that GLView binds for the current frame.
    aGraphicDriver->ChangeOptions().buffersNoSwap = Standard_True;
    aGraphicDriver->SetVerticalSync(false);
    
    // Create Viewer
    myViewer = new V3d_Viewer (aGraphicDriver);
    myViewer->SetDefaultLights();
    myViewer->SetLightOn();
    // A camera-aligned headlight flattens the equally angled faces of an
    // isometric model. Offset it so cavity floors, walls and rims remain
    // distinguishable, without changing the model's authored material.
    for (auto light = myViewer->ActiveLightIterator(); light.More(); light.Next()) {
        if (!light.Value().IsNull()
            && light.Value()->Type() == Graphic3d_TOLS_DIRECTIONAL
            && light.Value()->IsHeadlight()) {
            light.Value()->SetDirection(gp_Dir(0.35, -0.45, -1.0));
        }
    }
    
    // Create AIS context
    myContext = new Core3DContext (myViewer);
    myContext->SetDisplayMode ((int )AIS_DisplayMode::AIS_Shaded, false);
    
    myView = new Core3DView(myViewer, V3d_ORTHOGRAPHIC);
    myView->SetImmediateUpdate(Standard_False);
    myView->TriedronDisplay (Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.20, V3d_ZBUFFER);
    applyTriedronLift();

    myView->SetBgGradientColors(
        Quantity_Color(0.055, 0.065, 0.085, Quantity_TOC_sRGB),
        Quantity_Color(0.012, 0.016, 0.024, Quantity_TOC_sRGB),
        Aspect_GradientFillMethod_Vertical,
        Standard_False);
    Graphic3d_RenderingParams& renderingParams = myView->ChangeRenderingParams();
    renderingParams.NbMsaaSamples = 0;
    renderingParams.ToEnableDepthPrepass = Standard_False;
    // Start with basic unordered alpha blending. The capability probe after
    // SetWindow may enable weighted OIT only when OCCT reports the exact path
    // (including the MSAA variant) as supported.
    renderingParams.TransparencyMethod = Graphic3d_RTM_BLEND_UNORDERED;
    renderingParams.ShadingModel = Graphic3d_TypeOfShadingModel_Phong;
    
    Handle(Core3DCocoa_Window) aCocoaWindow = new Core3DCocoa_Window (theWin);
    myView->SetWindow (aCocoaWindow, aRendCtx);
    if (!aCocoaWindow->IsMapped())
    {
        aCocoaWindow->Map();
    }

    const Standard_Integer aMaxMsaa =
        aGraphicDriver->InquireLimit(Graphic3d_TypeOfLimit_MaxMsaa);
    renderingParams.NbMsaaSamples = aMaxMsaa >= 4 ? 4 : aMaxMsaa >= 2 ? 2 : 0;
    const bool hasBlendedOit =
        aGraphicDriver->InquireLimit(Graphic3d_TypeOfLimit_HasBlendedOit) != 0;
    const bool hasBlendedOitMsaa =
        aGraphicDriver->InquireLimit(Graphic3d_TypeOfLimit_HasBlendedOitMsaa) != 0;
    if ((renderingParams.NbMsaaSamples == 0 && hasBlendedOit)
        || (renderingParams.NbMsaaSamples > 0 && hasBlendedOitMsaa)) {
        renderingParams.TransparencyMethod = Graphic3d_RTM_BLEND_OIT;
    }
    
    myView->Camera()->SetProjectionType(Graphic3d_Camera::Projection_Perspective);
    Resize();
    return true;
}

void OcctViewer::setTriedronLift(int pixels)
{
    myTriedronLiftPixels = std::max(0, pixels);
    applyTriedronLift();
    if (!myView.IsNull()) {
        myView->Invalidate();
    }
}

void OcctViewer::applyTriedronLift()
{
    if (myView.IsNull()) return;
    const Handle(V3d_Trihedron)& trihedron = myView->Trihedron(false);
    if (trihedron.IsNull()) return;
    // SetPosition restores the default corner offset; the lift adds to it.
    trihedron->SetPosition(Aspect_TOTP_LEFT_LOWER);
    const Handle(Graphic3d_TransformPers)& persistence = trihedron->TransformPersistence();
    if (persistence.IsNull() || !persistence->IsTrihedronOr2d()) return;
    Graphic3d_Vec2i offset = persistence->Offset2d();
    offset.y() += myTriedronLiftPixels;
    persistence->SetOffset2d(offset);
    trihedron->Display(*myView);
}

void OcctViewer::Resize()
{
    if (myView.IsNull()) {
        return;
    }
    myView->MustBeResized();
    myView->Invalidate();
}

bool OcctViewer::RenderFrame()
{
    if (myView.IsNull()) {
        return false;
    }
#if defined(TARGET_OS_OSX) && TARGET_OS_OSX
    if (!HasCurrentNativeContext(myViewer)) return false;
#endif
    try {
        const auto driver = Handle(OpenGl_GraphicDriver)::DownCast(myViewer->Driver());
#if DEBUG
        core3d::render::NativeTangentPreparationTrace trace;
        trace.stage = 1;
        if (driver.IsNull() || !core3d::render::PrepareNativeTangentPresentations(
                myContext, driver->GetSharedContext(true), myPreparedTangentArrays, &trace)) {
            myDebugNativeFrameFailure = trace;
            if (myDebugNativeFrameFailures < UINT64_MAX) ++myDebugNativeFrameFailures;
            return false;
        }
#else
        if (driver.IsNull() || !core3d::render::PrepareNativeTangentPresentations(
                myContext, driver->GetSharedContext(true), myPreparedTangentArrays)) return false;
#endif
#if DEBUG
        if (myDebugFrameObserver && !myDebugFrameObserver(false)) return false;
#endif
        // Ordinary views that never enter the lesson have no new traversal.
        // Once used, keep publishing false after exit so newly redisplayed
        // proxies cannot inherit an old cached GPU program's enabled value.
        if (myView->hasUsedLessonLighting())
            PrepareNativeLessonLighting(myContext, driver->GetSharedContext(true), myView->lessonLightingEnabled());
        myView->RenderFrame();
#if DEBUG
        if (myDebugFrameObserver && !myDebugFrameObserver(true)) return false;
        if (myDebugNativeFrameSuccesses < UINT64_MAX) ++myDebugNativeFrameSuccesses;
#endif
        return true;
    } catch (const Standard_Failure&) {
#if DEBUG
        myDebugNativeFrameFailure = {}; myDebugNativeFrameFailure.stage = 90;
        if (myDebugNativeFrameFailures < UINT64_MAX) ++myDebugNativeFrameFailures;
#endif
        // GLView owns the bounded consecutive-failure policy and emits one
        // terminal pause diagnostic. Never log here per render attempt: a
        // damaged graphics context can otherwise grow CoreSimulator.log at
        // display-link frequency while recovery is being scheduled.
        return false;
    } catch (...) {
#if DEBUG
        myDebugNativeFrameFailure = {}; myDebugNativeFrameFailure.stage = 91;
        if (myDebugNativeFrameFailures < UINT64_MAX) ++myDebugNativeFrameFailures;
#endif
        return false;
    }
}

// =======================================================================
// function : FitAll
// purpose  :
// =======================================================================
void OcctViewer::FitAll()
{
    if (!myView.IsNull())
    {
        myView->FitAll();
        myView->ZFitAll();
    }
}

void OcctViewer::FitBox(const Bnd_Box& box)
{
    if (!myView.IsNull())
    {
        myView->FitAll(box);
        myView->ZFitAll();
    }
}

// =======================================================================
// function : StartRotation
// purpose  :
// =======================================================================
void OcctViewer::StartRotation(int theX, int theY)
{
    if (!myView.IsNull())
    {
        myView->StartRotation(theX, theY, 0);
    }
}

// =======================================================================
// function : Rotation
// purpose  :
// =======================================================================
void OcctViewer::Rotation(int theX, int theY)
{
    if (!myView.IsNull())
    {
        myView->Rotation(theX, theY);
    }
}

// =======================================================================
// function : Pan
// purpose  :
// =======================================================================
void OcctViewer::Pan(int theX, int theY, Standard_Boolean theToStart)
{
    if (!myView.IsNull())
    {
        myView->Pan(theX, theY, 1, theToStart);
    }
}

// =======================================================================
// function : Zoom
// purpose  :
// =======================================================================
namespace {
// Founder test of build 8545: a pinch out had no limit and the model shrank to a dot.
// A zoom out stops while the model's size is about a tenth of the view height.
constexpr Standard_Real kMinimumModelToViewRatio = 0.15;

Standard_Real ModelToViewRatio(const Handle(V3d_View)& theView,
                               const Handle(Core3DContext)& theContext,
                               const Handle(OcctDocument)& theDoc)
{
    if (theView.IsNull() || theContext.IsNull() || theDoc.IsNull()) return -1.0;
    AIS_ListOfInteractive anObjects;
    theContext->DisplayedObjects(anObjects);
    Bnd_Box aBox;
    for (AIS_ListOfInteractive::Iterator anIter(anObjects); anIter.More(); anIter.Next())
    {
        const Handle(AIS_InteractiveObject)& anObject = anIter.Value();
        if (anObject.IsNull() || theDoc->ShapeLabel(anObject).IsNull()) continue;
        Bnd_Box anObjectBox;
        anObject->BoundingBox(anObjectBox);
        if (!anObjectBox.IsVoid()) aBox.Add(anObjectBox);
    }
    if (aBox.IsVoid()) return -1.0;
    const Standard_Real aModelSize = Sqrt(aBox.SquareExtent());
    const Handle(Graphic3d_Camera)& aCamera = theView->Camera();
    if (aCamera.IsNull() || aModelSize <= 0.0) return -1.0;
    const gp_XYZ aDimensions = aCamera->ViewDimensions(aCamera->Distance());
    if (!(aDimensions.Y() > 0.0)) return -1.0;
    return aModelSize / aDimensions.Y();
}
}

void OcctViewer::Zoom(int theX, int theY, double theDelta)
{
    if (!myView.IsNull())
    {
        Handle(Graphic3d_Camera) aBefore;
        if (theDelta < 0.0)
        {
            aBefore = new Graphic3d_Camera();
            aBefore->Copy(myView->Camera());
        }
        ZoomWithoutLimit(theX, theY, theDelta);
        if (!aBefore.IsNull())
        {
            const Standard_Real aRatio = ModelToViewRatio(myView, myContext, myDoc);
            if (aRatio >= 0.0 && aRatio < kMinimumModelToViewRatio)
            {
                myView->Camera()->Copy(aBefore);
                myView->Invalidate();
            }
        }
    }
}

void OcctViewer::ZoomWithoutLimit(int theX, int theY, double theDelta)
{
    if (!myView.IsNull())
    {
        if (theX >=0 && theY >=0)
        {
            myView->StartZoomAtPoint(theX, theY);
            myView->ZoomAtPoint(0, 0, (int) theDelta, (int) theDelta);
        }
        else
        {
            double aCoeff = Abs(theDelta) / 100.0 + 1.0;
            aCoeff = theDelta > 0.0 ? aCoeff : 1.0 / aCoeff;
            myView->SetZoom(aCoeff, Standard_True);
        }
    }
}

bool OcctViewer::ZoomByFactor(double theX, double theY, double theFactor) noexcept
{
    if (!std::isfinite(theX) || !std::isfinite(theY)
        || !std::isfinite(theFactor) || theFactor <= 0.0
        || myView.IsNull() || myView->Camera().IsNull()
        || myView->Window().IsNull()) { return false; }
    Handle(Graphic3d_Camera) previous;
    const auto restore = [&]() {
        if (!previous.IsNull()) {
            try { myView->Camera()->Copy(previous); } catch (...) {}
        }
        return false;
    };
    try {
        Standard_Integer width = 0, height = 0;
        myView->Window()->Size(width, height);
        if (width <= 0 || height <= 0) { return false; }
        const auto camera = myView->Camera();
        const double scale = camera->Scale() / theFactor;
        if (!std::isfinite(scale) || scale <= 0.0) { return false; }
        if (theFactor == 1.0) { return true; }
        previous = new Graphic3d_Camera(camera);
        const auto center = previous->Center();
        const double nx = 2.0 * theX / width - 1.0;
        const double ny = 1.0 - 2.0 * theY / height;
        const auto anchor = previous->UnProject(gp_Pnt(nx, ny, previous->Project(center).Z()));

        // A pinch is a ratio of finger spacing, not mouse displacement.
        // Keep the target plane and direction while perspective scale changes
        // eye distance; orthographic scale changes only the view size.
        camera->SetScale(scale);
        if (!camera->IsOrthographic()) {
            camera->MoveEyeTo(center.Translated(gp_Vec(previous->Direction()) * -camera->Distance()));
        }
        const auto movedAnchor = camera->UnProject(gp_Pnt(nx, ny, camera->Project(center).Z()));
        gp_Trsf shift;
        shift.SetTranslation(gp_Vec(movedAnchor, anchor));
        camera->Transform(shift);
        myView->ZFitAll();
        const auto eye = camera->Eye(), target = camera->Center();
        for (const double value : {camera->Scale(), camera->Distance(),
                camera->ZNear(), camera->ZFar(), eye.X(), eye.Y(), eye.Z(),
                target.X(), target.Y(), target.Z()}) {
            if (!std::isfinite(value)) { return restore(); }
        }
        if (camera->Scale() <= 0.0 || camera->Distance() <= 0.0
            || camera->ZFar() <= camera->ZNear()
            || (!camera->IsOrthographic() && camera->ZNear() <= 0.0)) { return restore(); }
        // The zoom-out floor of Zoom() (722edb24, founder test of build 8545):
        // a pinch out stops while the model is about a tenth of the view, so
        // the model never shrinks to a dot. The pinch still counts as handled,
        // so spreading the fingers again zooms in at once.
        if (theFactor < 1.0) {
            const Standard_Real ratio = ModelToViewRatio(myView, myContext, myDoc);
            if (ratio >= 0.0 && ratio < kMinimumModelToViewRatio) {
                myView->Camera()->Copy(previous);
                myView->Invalidate();
                return true;
            }
        }
        myView->Invalidate();
        return true;
    } catch (...) { return restore(); }
}

namespace {

// A face hit whose surface normal is at a right angle to the pick ray: the
// face is seen exactly edge-on. Edges and points carry no normal (0 vector).
bool IsEdgeOnFaceHit(const SelectMgr_SortCriterion& theHit, const gp_Dir& theRay)
{
    const Standard_Real aLength = theHit.Normal.Modulus();
    if (!(aLength > 0.5)) { return false; }
    const Standard_Real aCos = (theHit.Normal.x() * theRay.X() + theHit.Normal.y() * theRay.Y()
                              + theHit.Normal.z() * theRay.Z()) / aLength;
    return std::abs(aCos) < 1.0e-4;
}

bool IsCameraFacingFaceHit(const SelectMgr_SortCriterion& theHit, const gp_Dir& theRay)
{
    return theHit.Normal.Modulus() > 0.5 && !IsEdgeOnFaceHit(theHit, theRay)
        && !Precision::IsInfinite(theHit.Point.X()) && !Precision::IsInfinite(theHit.Point.Y())
        && !Precision::IsInfinite(theHit.Point.Z());
}

Standard_Integer PickedRank(const Handle(StdSelect_ViewerSelector3d)& theSelector,
                            const Handle(SelectMgr_EntityOwner)& theOwner)
{
    const Standard_Integer aCount = std::min<Standard_Integer>(theSelector->NbPicked(), 256);
    for (Standard_Integer aRank = 1; aRank <= aCount; ++aRank) {
        if (theSelector->Picked(aRank) == theOwner) { return aRank; }
    }
    return 0;
}

enum class RayMeets { Unknown, No, Yes };

// Where the exact pick ray meets the owner's own shape, nearest first, as a
// position along the ray (in the part's frame, moved back to the world). No:
// the ray passes beside it (the part is only near the tap, seen edge-on).
// Unknown: no shape, or too large to test on a tap; OCCT's choice then stays.
RayMeets NearestRayHitAlong(const Handle(SelectMgr_EntityOwner)& theOwner,
                            const gp_Pnt& theOrigin, const gp_Dir& theRay, Standard_Real& theAlong)
{
    const Handle(StdSelect_BRepOwner) aBRep = Handle(StdSelect_BRepOwner)::DownCast(theOwner);
    if (aBRep.IsNull() || !aBRep->HasShape() || !aBRep->HasSelectable()) { return RayMeets::Unknown; }
    const TopoDS_Shape& aShape = aBRep->Shape();
    Standard_Integer aFaces = 0;
    for (TopExp_Explorer anExp(aShape, TopAbs_FACE); anExp.More(); anExp.Next()) {
        if (++aFaces > 4096) { return RayMeets::Unknown; }
    }
    if (aFaces == 0) { return RayMeets::Unknown; }
    const gp_Trsf aToWorld = aBRep->Selectable()->Transformation();
    const gp_Trsf aToPart = aToWorld.Inverted();
    IntCurvesFace_ShapeIntersector anIntersector;
    anIntersector.Load(aShape, 1.0e-7);
    anIntersector.Perform(gp_Lin(theOrigin.Transformed(aToPart), theRay.Transformed(aToPart)),
                          -1.0e7, 1.0e7);
    if (!anIntersector.IsDone()) { return RayMeets::Unknown; }
    bool aFound = false;
    for (Standard_Integer anIndex = 1; anIndex <= anIntersector.NbPnt(); ++anIndex) {
        const gp_Pnt aWorld = anIntersector.Pnt(anIndex).Transformed(aToWorld);
        const Standard_Real anAlong = gp_Vec(theOrigin, aWorld).Dot(gp_Vec(theRay));
        if (!aFound || anAlong < theAlong) { theAlong = anAlong; aFound = true; }
    }
    return aFound ? RayMeets::Yes : RayMeets::No;
}

} // namespace

// =======================================================================
// function : passEdgeOnFaceHit
// purpose  : OCCT ranks a triangle seen exactly edge-on (its plane holds the
//            view ray) at a false depth near 0, so a covered cutter's side
//            face won a tap on the body's top 12 mm above it (toy car, measure's
//            COVERED-CUTTER-TAP-TRACE.md items 6-8). When the first hit is such
//            a face, the exact ray decides: the part keeps the tap only where
//            the ray really meets it no farther than the next face that faces
//            the camera; otherwise that next face is the detected one.
// =======================================================================
void OcctViewer::passEdgeOnFaceHit()
{
    if (myContext.IsNull() || myView.IsNull() || !myContext->HasDetected()) { return; }
    try {
        OCC_CATCH_SIGNALS
        const Handle(StdSelect_ViewerSelector3d)& aSelector = myContext->MainSelector();
        if (aSelector.IsNull()) { return; }
        const Handle(SelectMgr_EntityOwner) aFirst = myContext->DetectedOwner();
        const Standard_Integer aFirstRank = PickedRank(aSelector, aFirst);
        if (aFirstRank < 1 || Handle(StdSelect_BRepOwner)::DownCast(aFirst).IsNull()) { return; }
        // The selector's own near/far ray: the pixel convention of the pick.
        const auto& aVolume = aSelector->GetManager();
        const gp_Pnt anOrigin = aVolume.GetNearPickedPnt(), aFar = aVolume.GetFarPickedPnt();
        const Standard_Real aLength = anOrigin.Distance(aFar);
        if (!std::isfinite(aLength) || aLength <= Precision::Confusion()) { return; }
        const gp_Dir aRay(gp_Vec(anOrigin, aFar));
        if (!IsEdgeOnFaceHit(aSelector->PickedData(aFirstRank), aRay)) { return; }

        // The next detected face that faces the camera, in OCCT's order (the
        // detected list already holds only what the filters let through).
        Handle(SelectMgr_EntityOwner) aNext;
        Standard_Integer aNextRank = 0;
        bool aPassedFirst = false;
        for (myContext->InitDetected(); myContext->MoreDetected(); myContext->NextDetected()) {
            const Handle(SelectMgr_EntityOwner) anOwner = myContext->DetectedCurrentOwner();
            if (anOwner == aFirst) { aPassedFirst = true; continue; }
            if (!aPassedFirst || Handle(StdSelect_BRepOwner)::DownCast(anOwner).IsNull()) { continue; }
            const Standard_Integer aRank = PickedRank(aSelector, anOwner);
            if (aRank < 1 || !IsCameraFacingFaceHit(aSelector->PickedData(aRank), aRay)) { continue; }
            aNext = anOwner; aNextRank = aRank;
            break;
        }
        if (aNext.IsNull()) { return; }
        const SelectMgr_SortCriterion& aNextHit = aSelector->PickedData(aNextRank);
        // Layers drawn on top win by OCCT's own rule, whatever the depth.
        if (aNextHit.ZLayerPosition != aSelector->PickedData(aFirstRank).ZLayerPosition) { return; }
        const Standard_Real aNextAlong = gp_Vec(anOrigin, aNextHit.Point).Dot(gp_Vec(aRay));
        Standard_Real aFirstAlong = 0.0;
        const RayMeets aMeets = NearestRayHitAlong(aFirst, anOrigin, aRay, aFirstAlong);
        if (aMeets == RayMeets::Unknown) { return; }
        if (aMeets == RayMeets::Yes
            && aFirstAlong <= aNextAlong + std::max(aNextHit.Tolerance, 1.0e-6)) {
            return; // the edge-on part really is in front (a part standing on another)
        }
        // HilightNextDetected makes its owner the one SelectDetected takes.
        const Standard_Integer aSteps = 2 * std::min<Standard_Integer>(aSelector->NbPicked(), 256) + 2;
        for (Standard_Integer aStep = 0; aStep < aSteps && myContext->DetectedOwner() != aNext; ++aStep) {
            if (myContext->HilightNextDetected(myView, Standard_False) == 0) { break; }
        }
        if (myContext->DetectedOwner() != aNext) {
            // Not reachable through the detected list: leave OCCT's choice.
            for (Standard_Integer aStep = 0; aStep < aSteps && myContext->DetectedOwner() != aFirst; ++aStep) {
                if (myContext->HilightNextDetected(myView, Standard_False) == 0) { break; }
            }
        }
        myView->RedrawImmediate();
    } catch (...) {
    }
}

// =======================================================================
// function : Select
// purpose  :
// =======================================================================
void OcctViewer::Select(int theX, int theY, AIS_SelectionScheme scheme)
{
    if (myContext.IsNull()) { return; }
    
    if (AIS_SOD_Nothing == myContext->MoveTo(theX, theY, myView, Standard_True)) {
        myContext->ClearSelected(Standard_True);
    }
    passEdgeOnFaceHit();
    myContext->SelectDetected(scheme);
}

bool OcctViewer::SelectBehind(int theX, int theY, const Handle(AIS_InteractiveObject)& theSkipped)
{
    if (myContext.IsNull() || myView.IsNull()) { return false; }
    try {
        OCC_CATCH_SIGNALS
        const Handle(StdSelect_ViewerSelector3d)& aSelector = myContext->MainSelector();
        if (aSelector.IsNull()) { return false; }
        // The hit list at the point, nearest first, exactly as MoveTo sorts it,
        // read without MoveTo: MoveTo would highlight the arrows and switch
        // their active mode on again.
        aSelector->Pick(theX, theY, myView);
        Handle(SelectMgr_EntityOwner) aBehind;
        const Standard_Integer aCount = std::min<Standard_Integer>(aSelector->NbPicked(), 256);
        for (Standard_Integer aRank = 1; aRank <= aCount; ++aRank) {
            const Handle(SelectMgr_EntityOwner) anOwner = aSelector->Picked(aRank);
            if (anOwner.IsNull() || !anOwner->HasSelectable()) { continue; }
            if (!theSkipped.IsNull() && anOwner->Selectable() == theSkipped) { continue; }
            // The selected part itself (the cutter behind its own arrows) is
            // not what the tap is for: go on to the part behind it. A part
            // counts as selected when any of its owners is.
            if (anOwner->IsSelected()) { continue; }
            const Handle(AIS_InteractiveObject) aPart =
                Handle(AIS_InteractiveObject)::DownCast(anOwner->Selectable());
            if (aPart.IsNull() || myContext->IsSelected(aPart)) { continue; }
            // A part can answer a pick through more than one owner; with one
            // of them selected the others are not, and taking one of those
            // would select the same part a second time (seen as a third
            // selection with two parts on screen, 15 Nov).
            bool aPartIsSelected = false;
            for (myContext->InitSelected(); myContext->MoreSelected() && !aPartIsSelected;
                 myContext->NextSelected()) {
                const Handle(SelectMgr_EntityOwner) aSelected = myContext->SelectedOwner();
                aPartIsSelected = !aSelected.IsNull() && aSelected->HasSelectable()
                    && aSelected->Selectable() == anOwner->Selectable();
            }
            if (aPartIsSelected) { continue; }
            // Only a part of the document: a normal tap never reaches the
            // helpers that can lie behind the nearest hit, this walk can.
            if (myDoc.IsNull() || myDoc->ShapeLabel(aPart).IsNull()
                || !myDoc->IsPresentationEditable(aPart)) { continue; }
            if (!myContext->GlobalFilter().IsNull()
                && !myContext->GlobalFilter()->IsOk(anOwner)) { continue; }
            aBehind = anOwner;
            break;
        }
        // The context's detected list indexes the selector's previous hit
        // list: reset it so nothing reads stale ranks after this query.
        myContext->ClearDetected(Standard_False);
        if (aBehind.IsNull()) { return false; }
        myContext->AddOrRemoveSelected(aBehind, Standard_True);
        return true;
    } catch (...) {
        return false;
    }
}

// =======================================================================
// function : ImportSTEP
// purpose  :
// =======================================================================
bool OcctViewer::ImportSTEP(const std::string &theFilename)
{
    // create a new document
    myDoc->InitDoc();
    ImportedDocumentHistoryGuard aHistoryGuard(myDoc->ChangeDocument());
    
    {
        std::lock_guard<std::mutex> aStepExchangeLock(
            Core3DSTEPExchangeMutex());
        STEPCAFControl_Reader aReader;
        Handle(XSControl_WorkSession) aSession = aReader.Reader().WS();

        try
        {
            if (aReader.ReadFile(theFilename.c_str()) != IFSelect_RetDone)
            {
                clearSession (aSession);
                return false;
            }

            if (!aReader.Transfer (myDoc->ChangeDocument()))
            {
                clearSession (aSession);
                return false;
            }

			// STEP creates XCAF labels without Core3D identity attributes. Migrate the
			// isolated document before it is exposed to normal editing/undo history.
			if (!myDoc->MigrateLegacyIdentifiers())
			{
				clearSession(aSession);
				return false;
			}
			if (!myDoc->MarkImportedBRepDefinitions())
			{
				clearSession(aSession);
				return false;
			}

			clearSession(aSession);
        }
        catch (const Standard_Failure& theFailure)
        {
            Message::SendFail (TCollection_AsciiString ("Exception raised during STEP import\n[")
                               + theFailure.GetMessageString() + "]\n" + theFilename.c_str());
            return false;
        }
    }
    
    Handle(XCAFDoc_ShapeTool) aShapeTool = XCAFDoc_DocumentTool::ShapeTool (myDoc->Document()->Main());
    TDF_LabelSequence aLabels;
    aShapeTool->GetFreeShapes (aLabels);
    
    // perform meshing explicitly
    TopoDS_Compound aCompound;
    BRep_Builder    aBuildTool;
    aBuildTool.MakeCompound (aCompound);
    for (Standard_Integer aLabIter = 1; aLabIter <= aLabels.Length(); ++aLabIter)
    {
        TopoDS_Shape     aShape;
        const TDF_Label& aLabel = aLabels.Value (aLabIter);
        if (XCAFDoc_ShapeTool::GetShape (aLabel, aShape))
        {
            aBuildTool.Add (aCompound, aShape);
        }
    }
    
    Handle(Prs3d_Drawer) aDrawer = myContext->DefaultDrawer();
    Standard_Real aDeflection = StdPrs_ToolTriangulatedShape::GetDeflection (aCompound, aDrawer);
    if (!BRepTools::Triangulation (aCompound, aDeflection))
    {
        BRepMesh_IncrementalMesh anAlgo;
        anAlgo.ChangeParameters().Deflection = aDeflection;
        anAlgo.ChangeParameters().Angle = aDrawer->DeviationAngle();
        anAlgo.ChangeParameters().InParallel = Standard_True;
        anAlgo.SetShape (aCompound);
        anAlgo.Perform();
    }
    
    // clear presentations
    clearContext();
    
    // create presentations
    XCAFPrs_Style aDefStyle;
    aDefStyle.SetColorSurf (Quantity_NOC_GRAY65);
    aDefStyle.SetColorCurv (Quantity_NOC_GRAY65);
    if (aLabels.Length() > 0
        && !displayWithChildren(myDoc->Document(), aLabels, aDefStyle))
    {
        clearContext();
        return false;
    }
    
    return true;
}

// =======================================================================
// function : displayWithChildren
// purpose  :
// =======================================================================
bool OcctViewer::displayWithChildren (
    const Handle(TDocStd_Document)& theDocument,
    const TDF_Label&                theLabel,
    const XCAFPrs_Style&            theDefaultStyle)
{
    TDF_LabelSequence aRoots;
    if (!theLabel.IsNull())
    {
        aRoots.Append(theLabel);
    }
    return displayWithChildren(theDocument, aRoots, theDefaultStyle);
}

// =======================================================================
// function : displayWithChildren
// purpose  : Traverse all unique roots under one aggregate admission budget.
// =======================================================================
bool OcctViewer::displayWithChildren (
    const Handle(TDocStd_Document)& theDocument,
    const TDF_LabelSequence&        theLabels,
    const XCAFPrs_Style&            theDefaultStyle)
{
    if (theDocument.IsNull() || theLabels.IsEmpty()
        || myContext.IsNull() || myDoc.IsNull()
        || myMaximumDisplayTraversalNodes == 0
        || myMaximumLeafPresentations == 0)
    {
        return false;
    }

    try
    {
        OCC_CATCH_SIGNALS

        TDF_LabelSequence aUniqueRoots;
        TDF_LabelMap aVisitedRoots;
        for (TDF_LabelSequence::Iterator aRoot(theLabels);
             aRoot.More(); aRoot.Next())
        {
            const TDF_Label& aLabel = aRoot.Value();
            if (aLabel.IsNull()
                || aLabel.Data() != theDocument->GetData())
            {
                return false;
            }
            if (aVisitedRoots.Add(aLabel))
            {
                aUniqueRoots.Append(aLabel);
            }
        }
        if (aUniqueRoots.IsEmpty())
        {
            return false;
        }

        core3d::pattern_owner::RetainedPresentationIndex aRetainedPresentation;
        if (!core3d::pattern_owner::CaptureRetainedPresentationIndex(
                theDocument, aRetainedPresentation))
        {
            return false;
        }
        TDF_LabelMap aReliefConstructionOwners;
        if (!CaptureTextReliefConstructionOwners(
                theDocument, aReliefConstructionOwners))
        {
            return false;
        }

        XCAFPrs_DocumentExplorer anExplorer;
        anExplorer.Init(theDocument,
                        aUniqueRoots,
                        XCAFPrs_DocumentExplorerFlags_None,
                        theDefaultStyle);
        const Handle(XCAFDoc_LayerTool) aLayerTool =
            XCAFDoc_DocumentTool::LayerTool(theDocument->Main());
        struct AdmittedLeaf
        {
            TDF_Label occurrence;
            TDF_Label definition;
            XCAFPrs_Style style;
            TopLoc_Location location;
            TopoDS_Shape shape;
            bool visible;
        };
        std::vector<AdmittedLeaf> anAdmittedLeaves;
        anAdmittedLeaves.reserve(
            std::min<Standard_Size>(myMaximumLeafPresentations, 256));
        Standard_Size aNodeCount = 0;
        Standard_Size aLeafCount = 0;

        // Admission is a complete first pass. In particular, do not allocate
        // CafShapePrs or touch AIS until the aggregate deduplicated-root walk
        // has proved both its node and leaf ceilings. A shared definition may
        // otherwise amplify into thousands of partially published objects
        // before a late occurrence crosses the limit.
        for (; anExplorer.More(); anExplorer.Next())
        {
            const Standard_Integer aDepth = anExplorer.CurrentDepth();
            if (aDepth < 0
                || static_cast<Standard_Size>(aDepth)
                    > kMaximumDisplayTraversalDepth
                || ++aNodeCount > myMaximumDisplayTraversalNodes)
            {
                return false;
            }

            const XCAFPrs_DocumentNode& aNode = anExplorer.Current();
            if (aNode.IsAssembly)
            {
                continue;
            }
            const TDF_Label aDefinition = aNode.RefLabel.IsNull()
                ? aNode.Label
                : aNode.RefLabel;
            if (aNode.Label.IsNull() || aDefinition.IsNull()
                || aNode.Label.Data() != theDocument->GetData()
                || aDefinition.Data() != theDocument->GetData()
                || !XCAFDoc_ShapeTool::IsShape(aDefinition)
                || XCAFDoc_ShapeTool::IsAssembly(aDefinition))
            {
                return false;
            }
            if (aReliefConstructionOwners.Contains(aDefinition))
            {
                continue;
            }
            if (++aLeafCount > myMaximumLeafPresentations)
            {
                return false;
            }
            const auto aRetainedState =
                core3d::pattern_owner::StateForEntityIdentifier(
                    aRetainedPresentation,
                    myDoc->EntityIdentifierForLabel(aDefinition));
            const bool isVisible = core3d::pattern_owner::EffectiveVisibility(
                aNode.Style.IsVisible(),
                IsExplorerPathVisible(anExplorer, aLayerTool), aRetainedState);

            const TopoDS_Shape aShape =
                XCAFDoc_ShapeTool::GetShape(aDefinition);
            if (aShape.IsNull())
            {
                return false;
            }

            anAdmittedLeaves.push_back({
                aNode.Label,
                aDefinition,
                aNode.Style,
                aNode.Location,
                aShape,
                isVisible,
            });
        }
        if (aLeafCount == 0)
        {
            return false;
        }

        // The full explorer is now admitted. Only this publication phase may
        // allocate or display one CafShapePrs per visible leaf occurrence.
        for (const AdmittedLeaf& aLeaf : anAdmittedLeaves)
        {
            if (!aLeaf.visible)
            {
                // A saved BRep may have no triangulation after reopening.
                // Prepare it through the normal admitted display-computation
                // path now, while loading, so read-only scene publication can
                // include hidden objects without invoking a mesher. A plain
                // temporary shape avoids XCAF visibility suppressing Compute.
                // No selection mode or viewer update is enabled; remove it
                // synchronously before any normal frame can be presented.
                Handle(AIS_Shape) aHiddenGeometry = new AIS_Shape(aLeaf.shape);
                try
                {
                    myContext->Display(aHiddenGeometry, AIS_Shaded, -1, Standard_False);
                    myContext->Remove(aHiddenGeometry, Standard_False);
                }
                catch (...)
                {
                    myContext->Remove(aHiddenGeometry, Standard_False);
                    throw;
                }
                continue;
            }
            // A definition may be instanced many times. Keep its BRep shared,
            // but never share the AIS object: selection and the OpenGL local
            // transform are occurrence state.
            const Graphic3d_MaterialAspect aDefaultMaterial(
                Graphic3d_NameOfMaterial_ShinyPlastified);
            Handle(CafShapePrs) aPresentation = new CafShapePrs(
                aLeaf.definition,
                aLeaf.occurrence,
                aLeaf.style,
                aDefaultMaterial);
            if (aPresentation.IsNull())
            {
                return false;
            }
            aPresentation->DispatchStyles(Standard_False);

            // Apply the explorer's effective whole-object appearance directly
            // as well as retaining CafShapePrs' per-subshape XCAF styles.
            const Handle(XCAFDoc_VisMaterial)& aVisualMaterial =
                aLeaf.style.Material();
            if (!aVisualMaterial.IsNull())
            {
                Graphic3d_MaterialAspect anAspect;
                aVisualMaterial->FillMaterialAspect(anAspect);
                aPresentation->SetMaterial(anAspect);
                aPresentation->SetColor(
                    aVisualMaterial->BaseColor().GetRGB());
            }
            if (aLeaf.style.IsSetColorSurf())
            {
                aPresentation->SetColor(aLeaf.style.GetColorSurf());
            }
            else if (aLeaf.style.IsSetColorCurv())
            {
                aPresentation->SetColor(aLeaf.style.GetColorCurv());
            }

            // Match the renderer-neutral snapshot path: the app-owned edit
            // transform stays on the definition and is composed with this
            // leaf's accumulated XCAF occurrence location.
            gp_Trsf aWorldTransform =
                myDoc->ObjectTransformForLabel(aLeaf.definition);
            aWorldTransform.Multiply(aLeaf.location.Transformation());

            // XCAFPrs_DocumentExplorer includes a definition shape's own
            // TopoDS location in the accumulated node location, while the AIS
            // geometry still carries that same location. Keep the BRep located
            // for sub-shape style identity, and cancel its location from the
            // AIS-local transform so it contributes exactly once at render.
            const TopLoc_Location& aShapeLocation = aLeaf.shape.Location();
            if (!aShapeLocation.IsIdentity())
            {
                aWorldTransform.Multiply(
                    aShapeLocation.Inverted().Transformation());
            }
            aPresentation->SetLocalTransformation(aWorldTransform);

            if (aPresentation->IsEditablePresentation())
            {
                // Preserve the established free-definition edit path.
                myDoc->LoadObjectMeterial(
                    aLeaf.definition, aPresentation);
                myContext->Display(aPresentation, Standard_False);
            }
            else
            {
                // The explorer style already merges imported definition
                // material and occurrence color in XCAF precedence order.
                // Re-applying the generic definition material here would
                // erase the occurrence color. Only explicit Shapeyard-owned
                // overrides may supersede that resolved style.
                myDoc->LoadObjectAuthoredMaterialOverrides(
                    aLeaf.definition, aPresentation);

                // Current edit persistence is definition-addressed. Component
                // occurrences remain visible but nonselectable so transform,
                // material, boolean, duplicate, and delete cannot accidentally
                // mutate their shared definition or double their placement.
                myContext->Display(
                    aPresentation,
                    AIS_Shaded,
                    -1,
                    Standard_False);
            }
        }
        return true;
    }
    catch (const Standard_Failure&)
    {
        return false;
    }
    catch (...)
    {
        return false;
    }
}

// =======================================================================
// function : clearSession
// purpose  :
// =======================================================================
void OcctViewer::clearSession (const Handle(XSControl_WorkSession)& theSession)
{
    if (theSession.IsNull())
    {
        return;
    }
    
    Handle(Transfer_TransientProcess) aMapReader = theSession->TransferReader()->TransientProcess();
    if (!aMapReader.IsNull())
    {
        aMapReader->Clear();
    }
    
    Handle(XSControl_TransferReader) aTransferReader = theSession->TransferReader();
    if (!aTransferReader.IsNull())
    {
        aTransferReader->Clear(1);
    }
}

// =======================================================================
// function : clearContext
// purpose  :
// =======================================================================
void OcctViewer::clearContext()
{
    if (!myContext.IsNull())
    {
        myContext->ClearSelected(Standard_False);
        myContext->RemoveAll(Standard_False);
    }
}
