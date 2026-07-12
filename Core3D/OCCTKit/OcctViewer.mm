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
#include "Core3DSTEPExchangeLock.h"

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
#include <TDF_LabelMap.hxx>
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
#include <vector>

#include <StlAPI_Writer.hxx>
#include <TDataStd_Name.hxx>
#include <TDataStd_Real.hxx>
#include <GP_Quaternion.hxx>

namespace {

constexpr Standard_Size kMaximumDisplayTraversalDepth = 128;

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
        };
        std::vector<AdmittedLeaf> anAdmittedVisibleLeaves;
        anAdmittedVisibleLeaves.reserve(
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
            if (++aLeafCount > myMaximumLeafPresentations)
            {
                return false;
            }
            if (!aNode.Style.IsVisible()
                || !IsExplorerPathVisible(anExplorer, aLayerTool))
            {
                continue;
            }

            const TopoDS_Shape aShape =
                XCAFDoc_ShapeTool::GetShape(aDefinition);
            if (aShape.IsNull())
            {
                return false;
            }

            anAdmittedVisibleLeaves.push_back({
                aNode.Label,
                aDefinition,
                aNode.Style,
                aNode.Location,
                aShape,
            });
        }
        if (aLeafCount == 0)
        {
            return false;
        }

        // The full explorer is now admitted. Only this publication phase may
        // allocate or display one CafShapePrs per visible leaf occurrence.
        for (const AdmittedLeaf& aLeaf : anAdmittedVisibleLeaves)
        {
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
