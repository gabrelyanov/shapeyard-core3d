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

#ifndef OcctViewer_H
#define OcctViewer_H

#include "OcctDocument.h"
#include "CafShapePrs.h"
#include "../Common/Core3DMobileResourceLimits.h"

#include <AIS_InteractiveContext.hxx>
#include <V3d_Viewer.hxx>
#include <XSControl_WorkSession.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFPrs_Style.hxx>
#include <TDF_LabelSequence.hxx>

#import <UIKit/UIKit.h>

#include "Core3DView.hpp"
#include "Core3DContext.hpp"

//! OCCT 3D Viewer holder.
class OcctViewer
{
public:
    
    //! Empty constructor.
    Standard_EXPORT OcctViewer();
    
    //! Destructor.
    Standard_EXPORT virtual ~OcctViewer();
    
    //! Release the viewer.
    Standard_EXPORT void release() noexcept;
    
public:
    
    //! Return viewer instance.
    const Handle(V3d_Viewer)& V3dViewer() const { return myViewer; }
    
    //! Return active view.
    const Handle(V3d_View)& ActiveView() const { return myView; }
    
    //! Interactive context.
    const Handle(AIS_InteractiveContext)& AisContext() const { return myContext; }
    
    //! Invalidate active viewer.
    void Invalidate()
    {
        if (!myView.IsNull()) {
            myView->Redraw();
        }
    }
    
public:
    
    //! Perform OCCT Viewer (re)initialization.
    Standard_EXPORT bool InitViewer (UIView* theWin);

    //! Resize the existing drawable without recreating the OCCT viewer.
    Standard_EXPORT void Resize();

    //! Render one frame into the currently bound application-owned framebuffer.
    Standard_EXPORT bool RenderFrame();
    
    Standard_EXPORT void FitAll();
    Standard_EXPORT void FitBox(const Bnd_Box& box);
    Standard_EXPORT void StartRotation(int theX, int theY);
    Standard_EXPORT void Rotation(int theX, int theY);
    Standard_EXPORT void Pan(int theX, int theY, Standard_Boolean theToStart);
    Standard_EXPORT void Zoom(int theX, int theY, double theDelta);
    Standard_EXPORT void Select(int theX, int theY, AIS_SelectionScheme scheme = AIS_SelectionScheme::AIS_SelectionScheme_XOR);
    
    Standard_EXPORT bool ImportSTEP(const std::string &theFilename);
#ifdef DEBUG
    //! Test-only traversal ceiling used to exercise aggregate multi-root
    //! admission without constructing tens of thousands of AIS objects.
    void SetDebugMaximumDisplayTraversalNodes(const Standard_Size theLimit)
    {
        myMaximumDisplayTraversalNodes = theLimit > 0 ? theLimit : 1;
    }
    //! Test-only leaf-presentation ceiling used to prove admission completes
    //! before any AIS occurrence is allocated or displayed.
    void SetDebugMaximumLeafPresentations(const Standard_Size theLimit)
    {
        myMaximumLeafPresentations = theLimit > 0 ? theLimit : 1;
    }
#endif
protected:
    void clearSession(const Handle(XSControl_WorkSession)& theSession);
    
    //! Display every leaf occurrence below a root as an independent AIS
    //! presentation. XCAFPrs_DocumentExplorer supplies the resolved inherited
    //! style and the accumulated occurrence location for each leaf.
    bool displayWithChildren (const Handle(TDocStd_Document)& theDocument,
                              const TDF_Label&                theLabel,
                              const XCAFPrs_Style&            theDefaultStyle);
    //! Display a deduplicated root set with one traversal budget. Validation
    //! deduplicates document labels globally, so presentation admission must
    //! not reset its resource ceiling for every free root.
    bool displayWithChildren (const Handle(TDocStd_Document)& theDocument,
                              const TDF_LabelSequence&        theLabels,
                              const XCAFPrs_Style&            theDefaultStyle);
    void clearContext();
    
protected:
    
    Handle(V3d_Viewer)              myViewer;  //!< main viewer
    Handle(Core3DView)              myView;    //!< main view
    Handle(Core3DContext)           myContext; //!< interactive context containing displayed objects
    Handle(OcctDocument)            myDoc;
    Standard_Size                   myMaximumDisplayTraversalNodes = 32'768;
    Standard_Size                   myMaximumLeafPresentations =
        core3d::limits::kMaximumLeafPresentations;
};

#endif // OcctViewer_H
