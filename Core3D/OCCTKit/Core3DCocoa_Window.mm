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

#import <TargetConditionals.h>

#if defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
#import <UIKit/UIKit.h>
#else
#import <Cocoa/Cocoa.h>
#endif

#include "Core3DCocoa_Window.hxx"

#include <Cocoa_LocalPool.hxx>

#include <Image_AlienPixMap.hxx>
#include <Aspect_WindowDefinitionError.hxx>

#include <cmath>

IMPLEMENT_STANDARD_RTTIEXT(Core3DCocoa_Window,Cocoa_Window)

#if defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
@protocol Core3DDrawableSizing <NSObject>
@property (nonatomic, readonly) CGSize drawableSize;
@end

@protocol Core3DDrawableInvalidating <NSObject>
- (void)requestRender;
@end

static CGFloat OcctDrawableScale(UIView *theView)
{
    const CGFloat aWindowScale = theView.window.screen.scale;
    if (aWindowScale > 0.0) {
        return aWindowScale;
    }
    if (theView.contentScaleFactor > 0.0) {
        return theView.contentScaleFactor;
    }
    return UIScreen.mainScreen.scale;
}

static Standard_Integer OcctPixelCoordinate(CGFloat theValue, CGFloat theScale)
{
    return static_cast<Standard_Integer>(std::lround(theValue * theScale));
}

static CGSize OcctDrawableSize(UIView *theView)
{
    if ([theView respondsToSelector:@selector(drawableSize)]) {
        const CGSize aDrawableSize = [(id<Core3DDrawableSizing>)theView drawableSize];
        if (aDrawableSize.width > 0.0 && aDrawableSize.height > 0.0) {
            return aDrawableSize;
        }
    }
    const CGFloat aScale = OcctDrawableScale(theView);
    const CGRect aBounds = theView.bounds;
    return CGSizeMake(
        OcctPixelCoordinate(aBounds.size.width, aScale),
        OcctPixelCoordinate(aBounds.size.height, aScale));
}
#else

#if !defined(MAC_OS_X_VERSION_10_12) || (MAC_OS_X_VERSION_MAX_ALLOWED < MAC_OS_X_VERSION_10_12)
// replacements for macOS versions before 10.12
#define NSWindowStyleMaskResizable NSResizableWindowMask
#define NSWindowStyleMaskClosable  NSClosableWindowMask
#define NSWindowStyleMaskTitled    NSTitledWindowMask
#endif

static Standard_Integer getScreenBottom()
{
    Cocoa_LocalPool aLocalPool;
    NSArray* aScreens = [NSScreen screens];
    if (aScreens == NULL || [aScreens count] == 0)
    {
        return 0;
    }
    
    NSScreen* aScreen = (NSScreen* )[aScreens objectAtIndex: 0];
    NSDictionary* aDict = [aScreen deviceDescription];
    NSNumber* aNumber = [aDict objectForKey: @"NSScreenNumber"];
    if (aNumber == NULL
        || [aNumber isKindOfClass: [NSNumber class]] == NO)
    {
        return 0;
    }
    
    CGDirectDisplayID aDispId = [aNumber unsignedIntValue];
    CGRect aRect = CGDisplayBounds(aDispId);
    return Standard_Integer(aRect.origin.y + aRect.size.height);
}
#endif

//! Extension for Cocoa_Window::InvalidateContent().
#if defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
@interface UIView (UIViewOcctAdditions)
- (void )invalidateContentOcct: (id )theSender;
@end
@implementation UIView (UIViewOcctAdditions)
- (void )invalidateContentOcct: (id )theSender
{
    (void )theSender;
    if ([self respondsToSelector:@selector(requestRender)]) {
        [(id<Core3DDrawableInvalidating>)self requestRender];
    } else {
        [self setNeedsDisplay];
    }
}
@end
#else
@interface NSView (NSViewOcctAdditions)
- (void )invalidateContentOcct: (id )theSender;
@end
@implementation NSView (NSViewOcctAdditions)
- (void )invalidateContentOcct: (id )theSender
{
    (void )theSender;
    [self setNeedsDisplay: YES];
}
@end
#endif

// =======================================================================
// function : Cocoa_Window
// purpose  :
// =======================================================================
Core3DCocoa_Window::Core3DCocoa_Window (const Standard_CString theTitle,
                                        const Standard_Integer thePxLeft,
                                        const Standard_Integer thePxTop,
                                        const Standard_Integer thePxWidth,
                                        const Standard_Integer thePxHeight)
: Cocoa_Window(theTitle, thePxLeft, thePxTop, thePxHeight, thePxWidth)
{
#if defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
    (void )theTitle;
#else
    if (thePxWidth <= 0 || thePxHeight <= 0)
    {
        throw Aspect_WindowDefinitionError("Coordinate(s) out of range");
    }
    else if (NSApp == NULL)
    {
        throw Aspect_WindowDefinitionError("Cocoa application should be instantiated before window");
        return;
    }
    
    // convert top-bottom coordinates to bottom-top (Cocoa)
    myYTop    = getScreenBottom() - myYBottom;
    myYBottom = myYTop + thePxHeight;
    
    Cocoa_LocalPool aLocalPool;
    NSUInteger aWinStyle = NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskResizable;
    NSRect aRectNs = NSMakeRect (float(myXLeft), float(myYTop), float(thePxWidth), float(thePxHeight));
    myHWindow = [[NSWindow alloc] initWithContentRect: aRectNs
                                            styleMask: aWinStyle
                                              backing: NSBackingStoreBuffered
                                                defer: NO];
    if (myHWindow == NULL)
    {
        throw Aspect_WindowDefinitionError("Unable to create window");
    }
    // for the moment, OpenGL renderer is expected to output sRGB colorspace
    [myHWindow setColorSpace: [NSColorSpace sRGBColorSpace]];
    myHView = [[myHWindow contentView] retain];
    
    NSString* aTitleNs = [[NSString alloc] initWithUTF8String: theTitle];
    [myHWindow setTitle: aTitleNs];
    [aTitleNs release];
    
    // do not destroy NSWindow on close - we didn't handle it!
    [myHWindow setReleasedWhenClosed: NO];
#endif
}

// =======================================================================
// function : Cocoa_Window
// purpose  :
// =======================================================================
#if defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
Core3DCocoa_Window::Core3DCocoa_Window (UIView* theViewNS)
: Cocoa_Window(theViewNS)
#else
Cocoa_Window::Cocoa_Window (NSView* theViewNS)
: Cocoa_Window(theViewNS)
#endif
{
#if defined(HAVE_OBJC_ARC)
    myHView = theViewNS;
#else
    myHView = [theViewNS retain];
#endif
    DoResize();
}

// =======================================================================
// function : ~Cocoa_Window
// purpose  :
// =======================================================================
Core3DCocoa_Window::~Core3DCocoa_Window()
{
#if !defined(HAVE_OBJC_ARC)
    Cocoa_LocalPool aLocalPool;
#endif
#if !(defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE)
    if (myHWindow != NULL)
    {
#if !defined(HAVE_OBJC_ARC)
        //[myHWindow close];
        [myHWindow release];
#endif
        myHWindow = NULL;
    }
#endif
    if (myHView != NULL)
    {
#if !defined(HAVE_OBJC_ARC)
        [myHView release];
#endif
        myHView = NULL;
    }
}

    
    // =======================================================================
    // function : DoResize
    // purpose  :
    // =======================================================================
    Aspect_TypeOfResize Core3DCocoa_Window::DoResize()
    {
        if (myHView == NULL)
        {
            return Aspect_TOR_UNKNOWN;
        }
        
#if defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
        const CGSize aDrawableSize = OcctDrawableSize(myHView);
        const Standard_Integer aXLeft = 0;
        const Standard_Integer aXRight = (Standard_Integer)aDrawableSize.width;
        const Standard_Integer aYTop = 0;
        const Standard_Integer aYBottom = (Standard_Integer)aDrawableSize.height;
#else
        NSRect aBounds = [myHView bounds];
        const Standard_Integer aXLeft = (Standard_Integer)aBounds.origin.x;
        const Standard_Integer aXRight =
            (Standard_Integer)(aBounds.origin.x + aBounds.size.width);
        const Standard_Integer aYTop = (Standard_Integer)aBounds.origin.y;
        const Standard_Integer aYBottom =
            (Standard_Integer)(aBounds.origin.y + aBounds.size.height);
#endif
        Standard_Integer aMask = 0;
        Aspect_TypeOfResize aMode = Aspect_TOR_UNKNOWN;
        
        if (Abs(aXLeft - myXLeft) > 2) aMask |= 1;
        if (Abs(aXRight - myXRight) > 2) aMask |= 2;
        if (Abs(aYTop - myYTop) > 2) aMask |= 4;
        if (Abs(aYBottom - myYBottom) > 2) aMask |= 8;
        switch (aMask)
        {
            case 0:  aMode = Aspect_TOR_NO_BORDER;               break;
            case 1:  aMode = Aspect_TOR_LEFT_BORDER;             break;
            case 2:  aMode = Aspect_TOR_RIGHT_BORDER;            break;
            case 4:  aMode = Aspect_TOR_TOP_BORDER;              break;
            case 5:  aMode = Aspect_TOR_LEFT_AND_TOP_BORDER;     break;
            case 6:  aMode = Aspect_TOR_TOP_AND_RIGHT_BORDER;    break;
            case 8:  aMode = Aspect_TOR_BOTTOM_BORDER;           break;
            case 9:  aMode = Aspect_TOR_BOTTOM_AND_LEFT_BORDER;  break;
            case 10: aMode = Aspect_TOR_RIGHT_AND_BOTTOM_BORDER; break;
            default: break;
        }
        
        myXLeft = aXLeft;
        myXRight = aXRight;
        myYTop = aYTop;
        myYBottom = aYBottom;
        return aMode;
    }
    
    // =======================================================================
    // function : DoMapping
    // purpose  :
    // =======================================================================
    Standard_Boolean Core3DCocoa_Window::DoMapping() const
    {
        return Standard_True;
    }
    
    // =======================================================================
    // function : Position
    // purpose  :
    // =======================================================================
    void Core3DCocoa_Window::Position (Standard_Integer& X1, Standard_Integer& Y1,
                                       Standard_Integer& X2, Standard_Integer& Y2) const
    {
#if defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
        const CGSize aDrawableSize = OcctDrawableSize(myHView);
        X1 = 0;
        Y1 = 0;
        X2 = (Standard_Integer)aDrawableSize.width;
        Y2 = (Standard_Integer)aDrawableSize.height;
#else
        NSWindow* aWindow = [myHView window];
        NSRect aWindowRect = [aWindow frame];
        X1 = (Standard_Integer) aWindowRect.origin.x;
        Y1 = getScreenBottom() - (Standard_Integer) aWindowRect.origin.y - (Standard_Integer) aWindowRect.size.height;
        X2 = X1 + (Standard_Integer) aWindowRect.size.width;
        Y2 = Y1 + (Standard_Integer) aWindowRect.size.height;
#endif
    }
    
    // =======================================================================
    // function : Size
    // purpose  :
    // =======================================================================
    void Core3DCocoa_Window::Size (Standard_Integer& theWidth,
                                   Standard_Integer& theHeight) const
    {
        if (myHView == NULL)
        {
            return;
        }
        
#if defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
        const CGSize aDrawableSize = OcctDrawableSize(myHView);
        theWidth = (Standard_Integer)aDrawableSize.width;
        theHeight = (Standard_Integer)aDrawableSize.height;
#else
        NSRect aBounds = [myHView bounds];
        theWidth = (Standard_Integer)aBounds.size.width;
        theHeight = (Standard_Integer)aBounds.size.height;
#endif
    }

    // =======================================================================
    // function : InvalidateContent
    // purpose  : Wake the application-owned frame scheduler.
    // =======================================================================
    void Core3DCocoa_Window::InvalidateContent (
        const Handle(Aspect_DisplayConnection)& theDisp)
    {
        (void )theDisp;
        if (myHView == NULL) {
            return;
        }
        if (NSThread.isMainThread) {
            [myHView invalidateContentOcct:nil];
        } else {
            [myHView performSelectorOnMainThread:@selector(invalidateContentOcct:)
                                      withObject:nil
                                   waitUntilDone:NO];
        }
    }
