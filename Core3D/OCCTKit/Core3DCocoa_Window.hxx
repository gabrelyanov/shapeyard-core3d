// Created on: 2012-11-12
// Created by: Kirill GAVRILOV
// Copyright (c) 2012-2014 OPEN CASCADE SAS
//
// This file is part of Open CASCADE Technology software library.
//
// This library is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License version 2.1 as published
// by the Free Software Foundation, with special exception defined in the file
// OCCT_LGPL_EXCEPTION.txt. Consult the file LICENSE_LGPL_21.txt included in OCCT
// distribution for complete text of the license and disclaimer of any warranty.
//
// Alternatively, this file may be used under the terms of Open CASCADE
// commercial license or contractual agreement.

#ifndef Core3DCocoa_Window_HeaderFile
#define Core3DCocoa_Window_HeaderFile

#if defined(__APPLE__)
  #import <TargetConditionals.h>
#endif

#if defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
  #ifdef __OBJC__
    @class UIView;
    @class UIWindow;
  #else
    struct UIView;
    struct UIWindow;
  #endif
#else
  #ifdef __OBJC__
    @class NSView;
    @class NSWindow;
  #else
    struct NSView;
    struct NSWindow;
  #endif
#endif

#include <Standard.hxx>
#include <Standard_Type.hxx>
#include <Cocoa_Window.hxx>

//! This class defines Cocoa window
class Core3DCocoa_Window : public Cocoa_Window
{
public:
    
    //! Creates a NSWindow and NSView defined by his position and size in pixels
    Standard_EXPORT Core3DCocoa_Window (const Standard_CString theTitle,
                                  const Standard_Integer thePxLeft,
                                  const Standard_Integer thePxTop,
                                  const Standard_Integer thePxWidth,
                                  const Standard_Integer thePxHeight);
    
#if defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
    //! Creates a wrapper over existing UIView handle
    Standard_EXPORT Core3DCocoa_Window (UIView* theViewUI);
#else
    //! Creates a wrapper over existing NSView handle
    Standard_EXPORT Core3DCocoa_Window (NSView* theViewNS);
#endif
    
    //! Destroys the Window and all resourses attached to it
    Standard_EXPORT ~Core3DCocoa_Window();
        
    //! Applies the resizing to the window <me>
    Standard_EXPORT virtual Aspect_TypeOfResize DoResize() Standard_OVERRIDE;
    
    //! Apply the mapping change to the window <me>
    Standard_EXPORT virtual Standard_Boolean DoMapping() const Standard_OVERRIDE;
    
    //! Returns The Window POSITION in PIXEL
    Standard_EXPORT virtual void Position (Standard_Integer& X1,
                                           Standard_Integer& Y1,
                                           Standard_Integer& X2,
                                           Standard_Integer& Y2) const Standard_OVERRIDE;
    
    //! Returns The Window SIZE in PIXEL
    Standard_EXPORT virtual void Size (Standard_Integer& theWidth,
                                       Standard_Integer& theHeight) const Standard_OVERRIDE;
    
    
protected:
    
    DEFINE_STANDARD_RTTIEXT(Core3DCocoa_Window, Cocoa_Window)
    
};

DEFINE_STANDARD_HANDLE(Core3DCocoa_Window, Cocoa_Window)

#endif // _Cocoa_Window_H__
