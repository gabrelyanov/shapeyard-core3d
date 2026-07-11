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
#include "OcctDocument.h"

#include <OpenGl_GraphicDriver.hxx>
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
#include <Transfer_TransientProcess.hxx>
#include <XSControl_TransferReader.hxx>
#include <XCAFDoc_DocumentTool.hxx>

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

#include <StlAPI_Writer.hxx>
#include <TDataStd_Name.hxx>
#include <TDataStd_Real.hxx>
#include <GP_Quaternion.hxx>

namespace {

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
{
    myDoc = new OcctDocument();
    myDoc->InitDoc();
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
    if (!myDoc.IsNull() && !myDoc->Document().IsNull()) {
        try {
            const Handle(TDocStd_Document) document = myDoc->Document();
            const Handle(TDocStd_Application) application =
                Handle(TDocStd_Application)::DownCast(document->Application());
            if (!application.IsNull()) {
                application->Close(document);
            }
        } catch (const Standard_Failure& failure) {
            NSLog(@"OCCT document teardown failed: %s", failure.GetMessageString());
        } catch (...) {
            NSLog(@"OCCT document teardown failed with an unknown error");
        }
    }

    myContext.Nullify();
    if (!myView.IsNull()) {
        if (EAGLContext.currentContext != nil) {
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
            NSLog(@"Releasing OCCT view without explicit GPU removal: no EAGL context");
        }
    }
    myView.Nullify();
    myViewer.Nullify();
    myDoc.Nullify();
}

// =======================================================================
// function : InitViewer
// purpose  :
// =======================================================================
bool OcctViewer::InitViewer (UIView* theWin)
{
    EAGLContext* aRendCtx = [EAGLContext currentContext];
    if (theWin == NULL || aRendCtx   == NULL)
    {
        NSLog(@"Error: No active EAGL context!");
        return false;
    }
    if (!myView.IsNull())
    {
        Resize();
        return true;
    }
    
    Handle(Aspect_DisplayConnection) aDisplayConnection = new Aspect_DisplayConnection();
    Handle(OpenGl_GraphicDriver) aGraphicDriver = new OpenGl_GraphicDriver (aDisplayConnection);
    // GLView owns drawable presentation and CADisplayLink pacing. OCCT only
    // renders into the framebuffer that GLView binds for the current frame.
    aGraphicDriver->ChangeOptions().buffersNoSwap = Standard_True;
    aGraphicDriver->SetVerticalSync(false);
    
    // Create Viewer
    myViewer = new V3d_Viewer (aGraphicDriver);
    myViewer->SetDefaultLights();
    myViewer->SetLightOn();
    
    // Create AIS context
    myContext = new Core3DContext (myViewer);
    myContext->SetDisplayMode ((int )AIS_DisplayMode::AIS_Shaded, false);
    
    myView = new Core3DView(myViewer, V3d_ORTHOGRAPHIC);
    myView->SetImmediateUpdate(Standard_False);
    myView->TriedronDisplay (Aspect_TOTP_LEFT_LOWER, Quantity_NOC_WHITE, 0.20, V3d_ZBUFFER);

    myView->SetBgGradientColors(
        Quantity_Color(0.055, 0.065, 0.085, Quantity_TOC_sRGB),
        Quantity_Color(0.012, 0.016, 0.024, Quantity_TOC_sRGB),
        Aspect_GradientFillMethod_Vertical,
        Standard_False);
    Graphic3d_RenderingParams& renderingParams = myView->ChangeRenderingParams();
    renderingParams.NbMsaaSamples = 0;
    renderingParams.ToEnableDepthPrepass = Standard_True;
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
    try {
        myView->RenderFrame();
        return true;
    } catch (const Standard_Failure& failure) {
        std::cout << "Viewport render failure: "
                  << failure.GetMessageString() << std::endl;
        return false;
    } catch (...) {
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
void OcctViewer::Zoom(int theX, int theY, double theDelta)
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
    myContext->SelectDetected(scheme);
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
    
    STEPCAFControl_Reader aReader;
    Handle(XSControl_WorkSession) aSession = aReader.Reader().WS();
    
    try
    {
        if (!aReader.ReadFile (theFilename.c_str()))
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

		clearSession(aSession);
    }
    catch (const Standard_Failure& theFailure)
    {
        Message::SendFail (TCollection_AsciiString ("Exception raised during STEP import\n[")
                           + theFailure.GetMessageString() + "]\n" + theFilename.c_str());
        return false;
    }
    
    Handle(XCAFDoc_ShapeTool) aShapeTool = XCAFDoc_DocumentTool::ShapeTool (myDoc->Document()->Main());
    Handle(XCAFDoc_ColorTool) aColorTool = XCAFDoc_DocumentTool::ColorTool (myDoc->Document()->Main());
    
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
    MapOfPrsForShapes aMapOfShapes;
    XCAFPrs_Style aDefStyle;
    aDefStyle.SetColorSurf (Quantity_NOC_GRAY65);
    aDefStyle.SetColorCurv (Quantity_NOC_GRAY65);
    for (Standard_Integer aLabIter = 1; aLabIter <= aLabels.Length(); ++aLabIter)
    {
        const TDF_Label& aLabel = aLabels.Value (aLabIter);
        displayWithChildren (*aShapeTool, *aColorTool, aLabel, TopLoc_Location(), aDefStyle, "", aMapOfShapes);
    }
    
    return true;
}

// =======================================================================
// function : displayWithChildren
// purpose  :
// =======================================================================
void OcctViewer::displayWithChildren (XCAFDoc_ShapeTool&             theShapeTool,
                                      XCAFDoc_ColorTool&             theColorTool,
                                      const TDF_Label&               theLabel,
                                      const TopLoc_Location&         theParentTrsf,
                                      const XCAFPrs_Style&           theParentStyle,
                                      const TCollection_AsciiString& theParentId,
                                      MapOfPrsForShapes&             theMapOfShapes)
{
    TDF_Label aRefLabel = theLabel;
    if (theShapeTool.IsReference (theLabel))
    {
        theShapeTool.GetReferredShape (theLabel, aRefLabel);
    }
    
    TCollection_AsciiString anEntry;
    TDF_Tool::Entry (theLabel, anEntry);
    if (!theParentId.IsEmpty())
    {
        anEntry = theParentId + "\n" + anEntry;
    }
    anEntry += ".";
    
    if (!theShapeTool.IsAssembly (aRefLabel))
    {
        Handle(AIS_InteractiveObject) anAis;
        if (!theMapOfShapes.Find (aRefLabel, anAis))
        {
            anAis = new AIS_Shape(theShapeTool.GetShape(aRefLabel));
            //anAis = new CafShapePrs (aRefLabel, theParentStyle, Graphic3d_NameOfMaterial_ShinyPlastified);
            theMapOfShapes.Bind (aRefLabel, anAis);
        }

        myContext->ApplyDefaultMaterial(anAis);
        myDoc->LoadObjectTransform(aRefLabel, Handle(AIS_Shape)::DownCast(anAis));        
        myDoc->LoadObjectMeterial(aRefLabel, Handle(AIS_Shape)::DownCast(anAis));
        
        myContext->Display  (anAis, Standard_False);
        return;
    }
    
    XCAFPrs_Style aDefStyle = theParentStyle;
    Quantity_Color aColor;
    if (theColorTool.GetColor (aRefLabel, XCAFDoc_ColorGen, aColor))
    {
        aDefStyle.SetColorCurv (aColor);
        aDefStyle.SetColorSurf (aColor);
    }
    if (theColorTool.GetColor (aRefLabel, XCAFDoc_ColorSurf, aColor))
    {
        aDefStyle.SetColorSurf (aColor);
    }
    if (theColorTool.GetColor (aRefLabel, XCAFDoc_ColorCurv, aColor))
    {
        aDefStyle.SetColorCurv (aColor);
    }
    
    for (TDF_ChildIterator childIter (aRefLabel); childIter.More(); childIter.Next())
    {
        TDF_Label aLabel = childIter.Value();
        if (!aLabel.IsNull()
            && (aLabel.HasAttribute() || aLabel.HasChild()))
        {
            TopLoc_Location aTrsf = theParentTrsf * theShapeTool.GetLocation (aLabel);
            displayWithChildren (theShapeTool, theColorTool, aLabel, aTrsf, aDefStyle, anEntry, theMapOfShapes);
        }
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
