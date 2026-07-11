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

#import <Foundation/Foundation.h>

#include <cstring>

#import "GLView.h"
#import "GLViewController.h"

@class GLView;

namespace {

class EAGLContextRestorer final {
public:
    explicit EAGLContextRestorer(EAGLContext *thePreviousContext)
    : myPreviousContext(thePreviousContext) {}

    ~EAGLContextRestorer() noexcept
    {
        if (EAGLContext.currentContext != myPreviousContext) {
            [EAGLContext setCurrentContext:myPreviousContext];
        }
    }

private:
    __strong EAGLContext *myPreviousContext;
};

} // namespace

@interface GLViewDisplayLinkProxy : NSObject
@property (nonatomic, weak) GLView *view;
- (void)displayLinkDidFire:(CADisplayLink *)displayLink;
@end

@interface GLView ()
- (void)displayLinkDidFire:(CADisplayLink *)displayLink;
- (BOOL)drawView;
- (void)scheduleDrawableCreationRetry;
@end

@implementation GLViewDisplayLinkProxy

- (void)displayLinkDidFire:(CADisplayLink *)displayLink
{
    [self.view displayLinkDidFire:displayLink];
}

@end

@implementation GLView {
    CADisplayLink *_displayLink;
    GLViewDisplayLinkProxy *_displayLinkProxy;
    BOOL _applicationActive;
    BOOL _hasDrawable;
    BOOL _isDrawing;
    BOOL _frameRequested;
    NSUInteger _interactiveRenderingDepth;
    NSUInteger _renderedFrameCount;
    NSUInteger _consecutiveRenderFailures;
    NSUInteger _consecutiveDrawableCreationFailures;
}

// =======================================================================
// function : layerClass
// purpose  :
// =======================================================================
+ (Class)layerClass
{
    return [CAEAGLLayer class];
}

// =======================================================================
// function : setupLayer
// purpose  :
// =======================================================================
- (void)setupLayer
{
    CAEAGLLayer* anEAGLLayer = (CAEAGLLayer*) self.layer;
    anEAGLLayer.opaque = YES;
    anEAGLLayer.drawableProperties = @{
        kEAGLDrawablePropertyRetainedBacking: @NO,
        kEAGLDrawablePropertyColorFormat: kEAGLColorFormatRGBA8,
    };
    const CGFloat scale = self.window.screen.scale ?: UIScreen.mainScreen.scale;
    self.contentScaleFactor = scale;
    anEAGLLayer.contentsScale = scale;
}

// =======================================================================
// function : setupContext
// purpose  :
// =======================================================================
- (void)setupContext
{
    // The bundled OCCT 7.8 OpenGLES driver is compiled against the ES2 API.
    // An ES3 EAGL context makes that driver query default-framebuffer enums on
    // our app-owned FBO, producing GL_INVALID_ENUM on iOS. The Metal renderer
    // will provide the modern GPU frontend while this compatibility viewport
    // remains on the API OCCT was built and validated for.
    myGLContext = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    if (!myGLContext)
    {
        NSLog(@"Failed to initialize an OpenGL ES context");
        return;
    }
    
    if (![EAGLContext setCurrentContext:myGLContext])
    {
        NSLog(@"Failed to set current OpenGL ES context");
    }
}

- (void)setupDisplayLink
{
    _displayLinkProxy = [[GLViewDisplayLinkProxy alloc] init];
    _displayLinkProxy.view = self;
    _displayLink = [CADisplayLink displayLinkWithTarget:_displayLinkProxy
                                               selector:@selector(displayLinkDidFire:)];
    _displayLink.paused = YES;
    [_displayLink addToRunLoop:NSRunLoop.mainRunLoop forMode:NSRunLoopCommonModes];
    [self updatePreferredFrameRate];
}

- (void)updatePreferredFrameRate
{
    const NSInteger maximumFramesPerSecond =
        MAX(30, self.window.screen.maximumFramesPerSecond ?: UIScreen.mainScreen.maximumFramesPerSecond);
    _displayLink.preferredFrameRateRange = CAFrameRateRangeMake(
        30.0f,
        (float)maximumFramesPerSecond,
        (float)maximumFramesPerSecond);
}

// =======================================================================
// function : createBuffers
// purpose  :
// =======================================================================
- (void) createBuffers
{
    NSAssert(NSThread.isMainThread, @"The viewport drawable is main-thread owned.");
    _hasDrawable = NO;
    if (!myGLContext || ![EAGLContext setCurrentContext:myGLContext]) {
        return;
    }

    const GLenum pendingError = glGetError();
    if (pendingError != GL_NO_ERROR) {
        NSLog(@"Cannot create viewport buffers with pending OpenGL error 0x%04x",
              pendingError);
        return;
    }

    if (myFrameBuffer == 0) {
        glGenFramebuffers(1, &myFrameBuffer);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, myFrameBuffer);
    if (myRenderBuffer == 0) {
        glGenRenderbuffers(1, &myRenderBuffer);
    }
    glBindRenderbuffer(GL_RENDERBUFFER, myRenderBuffer);
    
    if (![myGLContext renderbufferStorage:GL_RENDERBUFFER
                              fromDrawable:(CAEAGLLayer*)self.layer]) {
        myBackingWidth = 0;
        myBackingHeight = 0;
        return;
    }
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, myRenderBuffer);
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_WIDTH, &myBackingWidth);
    glGetRenderbufferParameteriv(GL_RENDERBUFFER, GL_RENDERBUFFER_HEIGHT, &myBackingHeight);
    if (glGetError() != GL_NO_ERROR || myBackingWidth <= 0 || myBackingHeight <= 0) {
        myBackingWidth = 0;
        myBackingHeight = 0;
        return;
    }
    
    if (myDepthBuffer == 0) {
        glGenRenderbuffers(1, &myDepthBuffer);
    }
    glBindRenderbuffer(GL_RENDERBUFFER, myDepthBuffer);

    const char *extensions = reinterpret_cast<const char *>(glGetString(GL_EXTENSIONS));
    const BOOL supportsPackedDepthStencil = extensions != nullptr
        && std::strstr(extensions, "GL_OES_packed_depth_stencil") != nullptr;
    BOOL hasCompleteDepthAttachment = NO;
    if (supportsPackedDepthStencil) {
        glRenderbufferStorage(GL_RENDERBUFFER,
                              GL_DEPTH24_STENCIL8_OES,
                              myBackingWidth,
                              myBackingHeight);
        if (glGetError() == GL_NO_ERROR) {
            glFramebufferRenderbuffer(GL_FRAMEBUFFER,
                                      GL_DEPTH_ATTACHMENT,
                                      GL_RENDERBUFFER,
                                      myDepthBuffer);
            glFramebufferRenderbuffer(GL_FRAMEBUFFER,
                                      GL_STENCIL_ATTACHMENT,
                                      GL_RENDERBUFFER,
                                      myDepthBuffer);
            const GLenum packedStatus = glCheckFramebufferStatus(GL_FRAMEBUFFER);
            hasCompleteDepthAttachment = glGetError() == GL_NO_ERROR
                && packedStatus == GL_FRAMEBUFFER_COMPLETE;
        }
    }
    if (!hasCompleteDepthAttachment) {
        // Packed depth/stencil can be advertised yet fail for a particular
        // drawable format. Detach it and retry with the universally supported
        // ES2 depth-only format before declaring the drawable unusable.
        glFramebufferRenderbuffer(GL_FRAMEBUFFER,
                                  GL_DEPTH_ATTACHMENT,
                                  GL_RENDERBUFFER,
                                  0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER,
                                  GL_STENCIL_ATTACHMENT,
                                  GL_RENDERBUFFER,
                                  0);
        if (glGetError() != GL_NO_ERROR) {
            return;
        }
        glBindRenderbuffer(GL_RENDERBUFFER, myDepthBuffer);
        glRenderbufferStorage(GL_RENDERBUFFER,
                              GL_DEPTH_COMPONENT16,
                              myBackingWidth,
                              myBackingHeight);
        if (glGetError() != GL_NO_ERROR) {
            return;
        }
        glFramebufferRenderbuffer(GL_FRAMEBUFFER,
                                  GL_DEPTH_ATTACHMENT,
                                  GL_RENDERBUFFER,
                                  myDepthBuffer);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER,
                                  GL_STENCIL_ATTACHMENT,
                                  GL_RENDERBUFFER,
                                  0);
        const GLenum depthOnlyStatus = glCheckFramebufferStatus(GL_FRAMEBUFFER);
        hasCompleteDepthAttachment = glGetError() == GL_NO_ERROR
            && depthOnlyStatus == GL_FRAMEBUFFER_COMPLETE;
    }

    glBindFramebuffer(GL_FRAMEBUFFER, myFrameBuffer);
    const GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    _hasDrawable = myBackingWidth > 0
        && myBackingHeight > 0
        && hasCompleteDepthAttachment
        && status == GL_FRAMEBUFFER_COMPLETE;
    if (!_hasDrawable) {
        NSLog(@"Failed to make complete framebuffer object %u", status);
        return;
    }
    glViewport(0, 0, myBackingWidth, myBackingHeight);
}

// =======================================================================
// function : destroyBuffers
// purpose  :
// =======================================================================
- (BOOL)performWithRenderingContext:(void (^)(void))work
{
    NSAssert(NSThread.isMainThread, @"The viewport context is main-thread owned.");
    if (!myGLContext || work == nil) {
        return NO;
    }
    EAGLContext *previousContext = EAGLContext.currentContext;
    if (![EAGLContext setCurrentContext:myGLContext]) {
        return NO;
    }
    EAGLContextRestorer restorePreviousContext(previousContext);
    work();
    return YES;
}

- (void) destroyBuffers
{
    [self performWithRenderingContext:^{
        glDeleteFramebuffers(1, &self->myFrameBuffer);
        glDeleteRenderbuffers(1, &self->myRenderBuffer);
        glDeleteRenderbuffers(1, &self->myDepthBuffer);
    }];
    myFrameBuffer = 0;
    myRenderBuffer = 0;
    myDepthBuffer = 0;
    myBackingWidth = 0;
    myBackingHeight = 0;
    _hasDrawable = NO;
}

// =======================================================================
// function : drawView
// purpose  :
// =======================================================================
- (BOOL) drawView
{
    NSAssert(NSThread.isMainThread, @"The viewport renderer is main-thread owned.");
    if (!_hasDrawable || _isDrawing || myController == nil) {
        return NO;
    }
    _isDrawing = YES;
    if (![EAGLContext setCurrentContext:myGLContext]) {
        _isDrawing = NO;
        return NO;
    }
    glBindFramebuffer(GL_FRAMEBUFFER, myFrameBuffer);
    glViewport(0, 0, myBackingWidth, myBackingHeight);
    if (![myController Draw]) {
        _isDrawing = NO;
        return NO;
    }
    glBindRenderbuffer(GL_RENDERBUFFER, myRenderBuffer);
    const BOOL didPresent = [myGLContext presentRenderbuffer:GL_RENDERBUFFER];
    if (didPresent) {
        ++_renderedFrameCount;
    }
    _isDrawing = NO;
    return didPresent;
}

// =======================================================================
// function : layoutSubviews
// purpose  :
// =======================================================================
- (void) layoutSubviews
{
    [super layoutSubviews];
    if (CGRectIsEmpty(self.bounds)
        || !myGLContext
        || !_applicationActive
        || self.window == nil) {
        return;
    }

    const CGFloat scale = self.window.screen.scale ?: UIScreen.mainScreen.scale;
    self.contentScaleFactor = scale;
    self.layer.contentsScale = scale;
    const int expectedWidth = (int)lround(CGRectGetWidth(self.bounds) * scale);
    const int expectedHeight = (int)lround(CGRectGetHeight(self.bounds) * scale);
    const BOOL drawableNeedsResize = !_hasDrawable
        || expectedWidth != myBackingWidth
        || expectedHeight != myBackingHeight;
    if (!drawableNeedsResize) {
        return;
    }

    // A size change is itself a frame request, even if the renderer was idle
    // before Auto Layout invalidated the drawable.
    _frameRequested = YES;
    [self createBuffers];
    if (!_hasDrawable) {
        [self scheduleDrawableCreationRetry];
        return;
    }
    _consecutiveDrawableCreationFailures = 0;
    [myController Setup];
    [self requestRender];
}

- (void)scheduleDrawableCreationRetry
{
    if (!_applicationActive || self.window == nil || !_frameRequested) {
        return;
    }
    ++_consecutiveDrawableCreationFailures;
    if (_consecutiveDrawableCreationFailures > 3) {
        _consecutiveDrawableCreationFailures = 0;
        _interactiveRenderingDepth = 0;
        _frameRequested = NO;
        NSLog(@"Viewport drawable creation paused after repeated failures.");
        [self updateDisplayLinkState];
        return;
    }

    __weak typeof(self) weakSelf = self;
    dispatch_async(dispatch_get_main_queue(), ^{
        GLView *strongSelf = weakSelf;
        if (strongSelf == nil
            || strongSelf->_hasDrawable
            || !strongSelf->_applicationActive
            || strongSelf.window == nil
            || !strongSelf->_frameRequested) {
            return;
        }
        [strongSelf setNeedsLayout];
    });
}

- (void)drawRect:(CGRect)rect
{
    (void)rect;
    [self requestRender];
}

// =======================================================================
// function : commonInit
// purpose  :
// =======================================================================
- (void)commonInit
{
    myController = nil;
    myBackingWidth = 0;
    myBackingHeight = 0;
    myFrameBuffer = 0;
    myRenderBuffer = 0;
    myDepthBuffer = 0;
    _applicationActive = UIApplication.sharedApplication.applicationState
        == UIApplicationStateActive;

    [self setupLayer];
    [self setupContext];
    [self setupDisplayLink];
    [[NSNotificationCenter defaultCenter]
        addObserver:self
        selector:@selector(applicationWillResignActive:)
        name:UIApplicationWillResignActiveNotification
        object:nil];
    [[NSNotificationCenter defaultCenter]
        addObserver:self
        selector:@selector(applicationDidBecomeActive:)
        name:UIApplicationDidBecomeActiveNotification
        object:nil];
}

- (instancetype)init
{
    return [self initWithFrame:CGRectZero];
}

- (instancetype)initWithFrame:(CGRect)frame
{
    self = [super initWithFrame:frame];
    if (self) {
        [self commonInit];
    }
    return self;
}

- (instancetype)initWithCoder:(NSCoder *)coder
{
    self = [super initWithCoder:coder];
    if (self) {
        [self commonInit];
    }
    return self;
}

- (void)dealloc
{
    [[NSNotificationCenter defaultCenter] removeObserver:self];
    [_displayLink invalidate];
    [self destroyBuffers];
    if ([EAGLContext currentContext] == myGLContext) {
        [EAGLContext setCurrentContext:nil];
    }
}

- (void)didMoveToWindow
{
    [super didMoveToWindow];
    [self updatePreferredFrameRate];
    if (self.window != nil) {
        [self setNeedsLayout];
        [self requestRender];
    } else {
        _interactiveRenderingDepth = 0;
        _consecutiveDrawableCreationFailures = 0;
        _hasDrawable = NO;
        [self updateDisplayLinkState];
    }
}

- (void)applicationWillResignActive:(NSNotification *)notification
{
    (void)notification;
    _applicationActive = NO;
    _interactiveRenderingDepth = 0;
    _frameRequested = NO;
    _consecutiveDrawableCreationFailures = 0;
    if (myGLContext && [EAGLContext setCurrentContext:myGLContext]) {
        glFinish();
    }
    _hasDrawable = NO;
    [self updateDisplayLinkState];
}

- (void)applicationDidBecomeActive:(NSNotification *)notification
{
    (void)notification;
    _applicationActive = YES;
    [self setNeedsLayout];
    [self requestRender];
}

- (BOOL)canRender
{
    return _applicationActive
        && self.window != nil
        && !self.hidden
        && self.alpha > 0.0
        && _hasDrawable;
}

- (void)updateDisplayLinkState
{
    const BOOL shouldRun = [self canRender]
        && (_frameRequested || _interactiveRenderingDepth > 0);
    _displayLink.paused = !shouldRun;
}

- (void)requestRender
{
    if (!NSThread.isMainThread) {
        __weak typeof(self) weakSelf = self;
        dispatch_async(dispatch_get_main_queue(), ^{
            [weakSelf requestRender];
        });
        return;
    }
    const BOOL beginsNewRequest = !_frameRequested;
    _frameRequested = YES;
    if (!_hasDrawable) {
        if (beginsNewRequest) {
            _consecutiveDrawableCreationFailures = 0;
        }
        [self setNeedsLayout];
    }
    [self updateDisplayLinkState];

    // This is the single normalized invalidation path for controller requests,
    // lifecycle recovery, and OCCT's Cocoa window callback. Keep clients on the
    // main thread even when the original render request came from a worker.
    GLViewController *controller = myController;
    id<GLViewControllerProtocol> delegate = controller.delegate;
    if (delegate
        && [delegate respondsToSelector:@selector(didInvalidateSceneSnapshot:)]) {
        [delegate didInvalidateSceneSnapshot:controller];
    }
}

- (void)beginInteractiveRendering
{
    if (!NSThread.isMainThread) {
        __weak typeof(self) weakSelf = self;
        dispatch_async(dispatch_get_main_queue(), ^{
            [weakSelf beginInteractiveRendering];
        });
        return;
    }
    ++_interactiveRenderingDepth;
    _frameRequested = YES;
    if (!_hasDrawable) {
        [self setNeedsLayout];
    }
    [self updateDisplayLinkState];
}

- (void)endInteractiveRendering
{
    if (!NSThread.isMainThread) {
        __weak typeof(self) weakSelf = self;
        dispatch_async(dispatch_get_main_queue(), ^{
            [weakSelf endInteractiveRendering];
        });
        return;
    }
    if (_interactiveRenderingDepth > 0) {
        --_interactiveRenderingDepth;
    }
    _frameRequested = YES;
    [self updateDisplayLinkState];
}

- (void)displayLinkDidFire:(CADisplayLink *)displayLink
{
    (void)displayLink;
    if (![self canRender]
        || (!_frameRequested && _interactiveRenderingDepth == 0)) {
        [self updateDisplayLinkState];
        return;
    }

    // Consume only the request that caused this frame. Any request raised
    // reentrantly by OCCT or client code while Draw is running remains set for
    // the next display-link tick.
    _frameRequested = NO;
    if ([self drawView]) {
        _consecutiveRenderFailures = 0;
    } else {
        ++_consecutiveRenderFailures;
        _frameRequested = _frameRequested || _consecutiveRenderFailures < 3;
        _hasDrawable = NO;
        if (_frameRequested) {
            [self setNeedsLayout];
        } else {
            _interactiveRenderingDepth = 0;
            NSLog(@"Viewport rendering paused after repeated frame failures.");
        }
    }
    [self updateDisplayLinkState];
}

- (NSUInteger)renderedFrameCount
{
    return _renderedFrameCount;
}

- (CGSize)drawableSize
{
    return CGSizeMake(myBackingWidth, myBackingHeight);
}

- (BOOL)isRenderLoopRunning
{
    return !_displayLink.paused;
}

- (EAGLRenderingAPI)renderingAPI
{
    return myGLContext.API;
}

@end
