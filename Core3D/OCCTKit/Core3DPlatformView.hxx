// Copyright (c) 2026 Shapeyard

#ifndef Core3DPlatformView_HeaderFile
#define Core3DPlatformView_HeaderFile

#import <TargetConditionals.h>

#if defined(TARGET_OS_OSX) && TARGET_OS_OSX
  #ifdef __OBJC__
    @class NSView;
  #else
    struct NSView;
  #endif
  using Core3DPlatformView = NSView;
#else
  #ifdef __OBJC__
    @class UIView;
  #else
    struct UIView;
  #endif
  using Core3DPlatformView = UIView;
#endif

#endif // Core3DPlatformView_HeaderFile
