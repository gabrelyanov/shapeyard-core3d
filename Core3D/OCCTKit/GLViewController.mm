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

#import "GLViewController.h"
#import "GLView.h"

#import "NSString+StdString.h"
#import <CommonCrypto/CommonDigest.h>

#include "ConstructorManipulator.hpp"
#include "CafShapePrs.h"
#include "OcctDocument.h"

#include <Graphic3d_TextureParams.hxx>
#include <Image_PixMap.hxx>
#include <Image_SupportedFormats.hxx>
#include <gp_Quaternion.hxx>
#include <TColStd_ListOfInteger.hxx>
#include <XCAFPrs_Texture.hxx>
#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace core3d;

namespace {

constexpr char kCbfMagic[] = "BINFILE";

BOOL HasCbfMagic(NSData *data) {
    constexpr NSUInteger magicLength = sizeof(kCbfMagic) - 1;
    return data != nil
        && data.length >= magicLength
        && std::memcmp(data.bytes, kCbfMagic, magicLength) == 0;
}

struct VerifiedFileIdentity {
    dev_t device = 0;
    ino_t inode = 0;
    off_t size = 0;
    mode_t mode = 0;
    nlink_t links = 0;
    timespec modification = {};
    timespec change = {};
};

VerifiedFileIdentity FileIdentity(const struct stat& value) {
    return {
        value.st_dev,
        value.st_ino,
        value.st_size,
        value.st_mode,
        value.st_nlink,
        value.st_mtimespec,
        value.st_ctimespec,
    };
}

bool operator==(const VerifiedFileIdentity& left,
                const VerifiedFileIdentity& right) {
    return left.device == right.device
        && left.inode == right.inode
        && left.size == right.size
        && left.mode == right.mode
        && left.links == right.links
        && left.modification.tv_sec == right.modification.tv_sec
        && left.modification.tv_nsec == right.modification.tv_nsec
        && left.change.tv_sec == right.change.tv_sec
        && left.change.tv_nsec == right.change.tv_nsec;
}

bool DecodeSHA256(NSString *value,
                  std::array<unsigned char, CC_SHA256_DIGEST_LENGTH>& result) {
    if (value == nil || value.length != CC_SHA256_DIGEST_LENGTH * 2) {
        return false;
    }
    const auto nibble = [](const unichar character, unsigned char& output) {
        if (character >= '0' && character <= '9') {
            output = static_cast<unsigned char>(character - '0');
            return true;
        }
        if (character >= 'a' && character <= 'f') {
            output = static_cast<unsigned char>(character - 'a' + 10);
            return true;
        }
        return false;
    };
    for (NSUInteger index = 0; index < result.size(); ++index) {
        unsigned char high = 0;
        unsigned char low = 0;
        if (!nibble([value characterAtIndex:index * 2], high)
            || !nibble([value characterAtIndex:index * 2 + 1], low)) {
            return false;
        }
        result[index] = static_cast<unsigned char>((high << 4) | low);
    }
    return true;
}

Core3DAssetLoadResult StageVerifiedAssetFile(
    NSURL *sourceURL,
    const unsigned long long expectedByteCount,
    NSString *expectedSHA256,
    NSURL **stagedURL) {
    static const unsigned long long kMaximumProjectDocumentBytes =
        256ull * 1024ull * 1024ull;
    if (stagedURL == nullptr) {
        return Core3DAssetLoadResultInternalFailure;
    }
    *stagedURL = nil;
    std::array<unsigned char, CC_SHA256_DIGEST_LENGTH> expectedDigest = {};
    const char *sourcePath = sourceURL.fileSystemRepresentation;
    if (!sourceURL.isFileURL || sourcePath == nullptr
        || expectedByteCount == 0
        || expectedByteCount > kMaximumProjectDocumentBytes
        || !DecodeSHA256(expectedSHA256, expectedDigest)) {
        return Core3DAssetLoadResultInvalidData;
    }

    struct stat pathStatus = {};
    if (::lstat(sourcePath, &pathStatus) != 0
        || (pathStatus.st_mode & S_IFMT) != S_IFREG
        || pathStatus.st_nlink != 1
        || pathStatus.st_size < 0
        || static_cast<unsigned long long>(pathStatus.st_size)
            != expectedByteCount) {
        return Core3DAssetLoadResultInvalidData;
    }
    const int source = ::open(sourcePath, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (source < 0) {
        return errno == ENOENT || errno == ENOTDIR || errno == ELOOP
            ? Core3DAssetLoadResultInvalidData
            : Core3DAssetLoadResultTemporaryFileFailure;
    }

    NSURL *temporaryURL = [NSFileManager.defaultManager.temporaryDirectory
        URLByAppendingPathComponent:[NSString stringWithFormat:
            @"%@.verified-project.cbf", NSUUID.UUID.UUIDString]];
    const char *temporaryPath = temporaryURL.fileSystemRepresentation;
    const int destination = temporaryPath == nullptr
        ? -1
        : ::open(temporaryPath,
                 O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
                 S_IRUSR | S_IWUSR);
    if (destination < 0) {
        ::close(source);
        return Core3DAssetLoadResultTemporaryFileFailure;
    }

    Core3DAssetLoadResult result = Core3DAssetLoadResultInvalidData;
    CC_SHA256_CTX hash = {};
    CC_SHA256_Init(&hash);
    std::array<unsigned char, 64 * 1024> buffer = {};
    unsigned long long copiedBytes = 0;
    struct stat openedStatus = {};
    if (::fstat(source, &openedStatus) == 0
        && FileIdentity(pathStatus) == FileIdentity(openedStatus)) {
        bool failed = false;
        while (!failed) {
            ssize_t count = ::read(source, buffer.data(), buffer.size());
            if (count < 0 && errno == EINTR) {
                continue;
            }
            if (count < 0) {
                result = Core3DAssetLoadResultTemporaryFileFailure;
                failed = true;
                break;
            }
            if (count == 0) {
                break;
            }
            if (static_cast<unsigned long long>(count)
                    > expectedByteCount - copiedBytes
                || CC_SHA256_Update(
                    &hash, buffer.data(), static_cast<CC_LONG>(count)) != 1) {
                failed = true;
                break;
            }
            copiedBytes += static_cast<unsigned long long>(count);
            ssize_t written = 0;
            while (written < count) {
                const ssize_t writeCount = ::write(
                    destination,
                    buffer.data() + written,
                    static_cast<size_t>(count - written));
                if (writeCount < 0 && errno == EINTR) {
                    continue;
                }
                if (writeCount <= 0) {
                    result = Core3DAssetLoadResultTemporaryFileFailure;
                    failed = true;
                    break;
                }
                written += writeCount;
            }
        }

        std::array<unsigned char, CC_SHA256_DIGEST_LENGTH> actualDigest = {};
        struct stat finalStatus = {};
        struct stat finalPathStatus = {};
        if (!failed
            && copiedBytes == expectedByteCount
            && CC_SHA256_Final(actualDigest.data(), &hash) == 1
            && actualDigest == expectedDigest
            && ::fstat(source, &finalStatus) == 0
            && FileIdentity(openedStatus) == FileIdentity(finalStatus)
            && ::lstat(sourcePath, &finalPathStatus) == 0
            && FileIdentity(openedStatus) == FileIdentity(finalPathStatus)) {
            result = Core3DAssetLoadResultSuccess;
        }
    }
    ::close(destination);
    ::close(source);
    if (result == Core3DAssetLoadResultSuccess) {
        *stagedURL = temporaryURL;
    } else {
        [NSFileManager.defaultManager removeItemAtURL:temporaryURL error:nil];
    }
    return result;
}

Core3DAssetLoadResult AssetLoadResultFromImportResult(AssetImportResult result) {
    switch (result) {
        case AssetImportResult::Success:
            return Core3DAssetLoadResultSuccess;
        case AssetImportResult::InvalidData:
            return Core3DAssetLoadResultInvalidData;
        case AssetImportResult::TemporaryFileFailure:
            return Core3DAssetLoadResultTemporaryFileFailure;
        case AssetImportResult::Busy:
            return Core3DAssetLoadResultBusy;
        case AssetImportResult::UnsupportedVersion:
            return Core3DAssetLoadResultUnsupportedVersion;
        case AssetImportResult::InternalFailure:
            return Core3DAssetLoadResultInternalFailure;
    }
}

void CompleteAssetLoadOnMain(void (^completion)(Core3DAssetLoadResult),
                             Core3DAssetLoadResult result) {
    dispatch_async(dispatch_get_main_queue(), ^{
        completion(result);
    });
}

} // namespace

@interface GLViewController () <UIGestureRecognizerDelegate>
- (void)endActiveRenderingInteractions;
- (void)endRawPrimaryInteractionIfNeededCancelled:(BOOL)cancelled;
- (void)checkSelections;
- (void)addCube:(UIBarButtonItem *)sender;
- (BOOL)restoreBooleanActionForRetainedGizmoType:(PrimitiveGizmoType)type;
- (BOOL)retireBooleanActionForGizmoType:(PrimitiveGizmoType)type;
@end

@implementation GLViewController {
    dispatch_queue_t _assetDataQueue;
    BOOL _cancelTouches;
    BOOL _didSetupViewer;
    BOOL _rawTouchRendering;
    BOOL _rawTouchWasCancelled;
    NSMutableSet<UITouch *> *_rawViewportTouches;
    BOOL _pinchRendering;
    BOOL _panRendering;
    CGPoint _pinchPreviousTouch[2];
    UITapGestureRecognizer *_tapRecognizer;
    BOOL _suppressTapSelectionForManipulatorInteraction;
}

- (void)dealloc {
    [[NSNotificationCenter defaultCenter] removeObserver:self];
    if (_viewer != nullptr) {
        GLView *view = self.isViewLoaded ? [self viewportView] : nil;
        if (view != nil) {
            const std::shared_ptr<Core3DViewer> viewer = _viewer;
            const BOOL didRelease = [view performWithRenderingContext:^{
                viewer->release();
            }];
            if (!didRelease) {
                NSLog(@"Core3D rendering context unavailable during teardown; using safe fallback.");
                EAGLContext *previousContext = EAGLContext.currentContext;
                [EAGLContext setCurrentContext:nil];
                viewer->release();
                [EAGLContext setCurrentContext:previousContext];
            }
        } else {
            // No GL view means InitViewer never created graphics resources.
            _viewer->release();
        }
    }
    _viewer = nullptr;
    NSLog(@"~GLViewController");
}

// =======================================================================
// function : init
// purpose  :
// =======================================================================
- (id) init
{
    self = [super init];

    if (self) {
        _viewer = std::make_shared<Core3DViewer>();
        _assetDataQueue = dispatch_queue_create("com.shapeyard.sync", NULL);
		_rawViewportTouches = [NSMutableSet set];
		_isConstructorMode = false;
        [[NSNotificationCenter defaultCenter]
            addObserver:self
            selector:@selector(documentDidChange:)
            name:@"OcctDocumentChanges"
            object:nil];
        [[NSNotificationCenter defaultCenter]
            addObserver:self
            selector:@selector(applicationWillResignActive:)
            name:UIApplicationWillResignActiveNotification
            object:nil];
    }

    return self;
}

-(std::shared_ptr<core3d::Core3DViewer>) viewer {
    return _viewer;
}

- (GLView *)viewportView
{
    return [self.view isKindOfClass:GLView.class] ? (GLView *)self.view : nil;
}

- (CGFloat)drawableScale
{
    GLView *view = [self viewportView];
    if (view.bounds.size.width > 0.0 && view.drawableSize.width > 0.0) {
        return view.drawableSize.width / view.bounds.size.width;
    }
    return self.view.window.screen.scale ?: UIScreen.mainScreen.scale;
}

- (CGPoint)drawablePointForPoint:(CGPoint)point
{
    const CGFloat scale = [self drawableScale];
    return CGPointMake(lround(point.x * scale), lround(point.y * scale));
}

- (void)requestRender
{
    [[self viewportView] requestRender];
}

#ifdef DEBUG
- (void)debugRequestRender
{
    [self requestRender];
}
#endif

- (NSUInteger)renderedFrameCount
{
    return [self viewportView].renderedFrameCount;
}

- (CGSize)drawableSize
{
    return [self viewportView].drawableSize;
}

- (BOOL)isRenderLoopRunning
{
    return [self viewportView].isRenderLoopRunning;
}

- (NSInteger)renderingAPIVersion
{
    switch ([self viewportView].renderingAPI) {
        case kEAGLRenderingAPIOpenGLES3:
            return 3;
        case kEAGLRenderingAPIOpenGLES2:
            return 2;
        default:
            return 0;
    }
}

- (void)documentDidChange:(NSNotification *)notification
{
    if ([notification.object isKindOfClass:NSValue.class]
        && _viewer != nullptr
        && !_viewer->getDocument().IsNull()
        && [(NSValue *)notification.object pointerValue]
            != _viewer->getDocument().get()) {
        return;
    }
    [self requestRender];
}

- (void)applicationWillResignActive:(NSNotification *)notification
{
    (void)notification;
    [self endActiveRenderingInteractions];
}

- (void)viewWillDisappear:(BOOL)animated
{
    [super viewWillDisappear:animated];
    [self endActiveRenderingInteractions];
}

- (void)endActiveRenderingInteractions
{
    GLView *view = [self viewportView];
    const BOOL hadRawPrimaryInteraction = _rawTouchRendering;
    const BOOL hadActiveInteraction =
        _rawTouchRendering || _pinchRendering || _panRendering;
    const BOOL hadUnresolvedMirrorObjects =
        _viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && _viewer->getObjectInteractor()->hasUnresolvedMirrorObjects();
    const BOOL hadBooleanOperation =
        _viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && (_viewer->getObjectInteractor()->hasActiveBoolean()
            || _viewer->getObjectInteractor()->hasUnresolvedBoolean());
    const PrimitiveGizmoType booleanGizmoType = hadBooleanOperation
        ? [self getGizmoType]
        : PrimitiveGizmoTypeNone;
    const BOOL hadExtrusion =
        _viewer != nullptr
        && _viewer->getShapeInteractor() != nullptr
        && _viewer->getShapeInteractor()->hasActiveExtrusion();
    if (hadActiveInteraction && _viewer != nullptr) {
        _viewer->CancelInteraction(0, 0);
    }
    if (hadUnresolvedMirrorObjects) {
        _viewer->getObjectInteractor()->clearTrialMirrorObjects();
    }
    if (hadBooleanOperation) {
        _viewer->getObjectInteractor()->cancelActiveBoolean();
        const BOOL didResolveBoolean =
            !_viewer->getObjectInteractor()->hasActiveBoolean()
            && !_viewer->getObjectInteractor()->hasUnresolvedBoolean();
        // Background/view disappearance retires transient geometry but keeps an
        // empty action provenance while the public tool mode remains Boolean, so
        // foreground taps can start a fresh operation without a mode toggle.
        if (!_isPreviewMode) {
            (void)[self restoreBooleanActionForRetainedGizmoType:
                booleanGizmoType];
        } else if (didResolveBoolean
                   && (booleanGizmoType == PrimitiveGizmoTypeSubtract
                   || booleanGizmoType == PrimitiveGizmoTypeUnion)) {
            if (_delegate != nil
                && [_delegate respondsToSelector:
                    @selector(viewerDidFailToRetainBooleanMode:)]) {
                [_delegate viewerDidFailToRetainBooleanMode:self];
            } else {
                _viewer->getObjectInteractor()->setManipulatorType(
                    PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
            }
        }
    }
    if (hadExtrusion) {
        _viewer->getShapeInteractor()->cancelExtrusion();
    }
    if (_rawTouchRendering) {
        _rawTouchRendering = NO;
        [view endInteractiveRendering];
    }
    [_rawViewportTouches removeAllObjects];
    if (_pinchRendering) {
        _pinchRendering = NO;
        [view endInteractiveRendering];
    }
    if (_panRendering) {
        _panRendering = NO;
        [view endInteractiveRendering];
    }
    if (hadActiveInteraction || hadUnresolvedMirrorObjects
        || hadBooleanOperation || hadExtrusion) {
        [self requestRender];
    }
    if (hadBooleanOperation || hadExtrusion) {
        // Selection notification is also the renderer-neutral presentation
        // invalidation and Apply-state refresh for lifecycle cancellation.
        [self checkSelections];
    }
    if ((hadRawPrimaryInteraction || hadUnresolvedMirrorObjects
         || hadBooleanOperation || hadExtrusion)
        && _delegate
        && [_delegate respondsToSelector:
            @selector(viewer:didEndPrimaryInteractionCancelled:)]) {
        [_delegate viewer:self didEndPrimaryInteractionCancelled:YES];
    }
    _rawTouchWasCancelled = NO;
}
// =======================================================================
// function : Draw
// purpose  :
// =======================================================================
- (BOOL) Draw
{
    if (_didSetupViewer && _viewer != nullptr) {
        return _viewer->RenderFrame();
    }
    return NO;
}

// =======================================================================
// function : Setup
// purpose  :
// =======================================================================
- (void) Setup {
    if (_didSetupViewer) {
        _viewer->Resize();
        [self requestRender];
        return;
    }

    if (!_viewer->InitViewer(self.view)) {
        NSLog(@"Failed to init viewer");
        return;
    }

    __weak typeof(self) weakSelf = self;
    _viewer->setBooleanPreviewStateChangedCallback([weakSelf]() {
        dispatch_async(dispatch_get_main_queue(), ^{
            __strong typeof(weakSelf) strongSelf = weakSelf;
            if (strongSelf == nil) {
                return;
            }
            [strongSelf checkSelections];
            [strongSelf requestRender];
        });
    });

    _didSetupViewer = YES;
    _viewer->showGrid(!_isPreviewMode);
    if (_isPreviewMode) {
        _viewer->setPreviewMode();
    }
    //    [self importScrew:nullptr];
    if (_delegate && [_delegate respondsToSelector:@selector(didSetupViewer:)]) {
        __weak typeof(self) weakSelf = self;
        [_delegate didSetupViewer:weakSelf];
    }
    [self requestRender];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    if (_viewer != nullptr
        && _viewer->getObjectInteractor() != nullptr
        && (_viewer->getObjectInteractor()->hasActiveBoolean()
            || _viewer->getObjectInteractor()->hasUnresolvedBoolean())) {
        const PrimitiveGizmoType currentType = [self getGizmoType];
        _viewer->getObjectInteractor()->cancelActiveBoolean();
        (void)[self restoreBooleanActionForRetainedGizmoType:currentType];
        [self checkSelections];
        [self requestRender];
    }
}

// =======================================================================
// function : loadView
// purpose  :
// =======================================================================
- (void) loadView
{
    GLView* aGLView = [[GLView alloc] init];
    aGLView->myController = self;
    self.view = aGLView;
}

- (void) setConstructorMode
{
	_isConstructorMode = true;
}

// =======================================================================
// function : touchesBegan
// purpose  :
// =======================================================================
- (void)touchesBegan:(NSSet<UITouch *> *)theTouches withEvent:(UIEvent *)theEvent
{
    [super touchesBegan:theTouches withEvent:theEvent];

    if (_isPreviewMode) {
        return;
    }

	const BOOL didBeginPrimaryInteraction = _rawViewportTouches.count == 0;
	[_rawViewportTouches unionSet:theTouches];
	if (didBeginPrimaryInteraction) {
		_cancelTouches = NO;
		_rawTouchWasCancelled = NO;
		_rawTouchRendering = YES;
		_suppressTapSelectionForManipulatorInteraction = NO;
		[[self viewportView] beginInteractiveRendering];
	}

    UITouch *aTouch = [theTouches anyObject];
    if (aTouch != NULL) {
        const CGPoint point = [self drawablePointForPoint:[aTouch locationInView:self.view]];
        if (didBeginPrimaryInteraction
            && _delegate
            && [_delegate respondsToSelector:
                @selector(viewer:willBeginPrimaryInteractionAtDrawablePoint:drawableSize:)]) {
            [_delegate viewer:self
                willBeginPrimaryInteractionAtDrawablePoint:point
                                             drawableSize:self.drawableSize];
        }
        _tapRecognizer.cancelsTouchesInView = YES;
        _viewer->StartRotation((int)point.x, (int)point.y);
        const std::shared_ptr<ObjectInteractor> anInteractor =
            _viewer->getObjectInteractor();
        _suppressTapSelectionForManipulatorInteraction =
            anInteractor != nullptr
            && anInteractor->isManipulatorInteractionActive();
        if (_suppressTapSelectionForManipulatorInteraction) {
            // The raw touch-up resolves both transforms and operation handles
            // such as mirror planes. Keep the recognizer from cancelling that
            // lifecycle; tapHandler still suppresses ordinary selection.
            _tapRecognizer.cancelsTouchesInView = NO;
        }
        [self requestRender];
    }
}

- (void)endRawPrimaryInteractionIfNeededCancelled:(BOOL)cancelled {
    if (!_rawTouchRendering || _rawViewportTouches.count != 0) {
        return;
    }
    _rawTouchRendering = NO;
    [[self viewportView] endInteractiveRendering];
    const BOOL resolvedAsCancelled = cancelled || _rawTouchWasCancelled;
    _rawTouchWasCancelled = NO;
    if (_delegate
        && [_delegate respondsToSelector:
            @selector(viewer:didEndPrimaryInteractionCancelled:)]) {
        [_delegate viewer:self
            didEndPrimaryInteractionCancelled:resolvedAsCancelled];
    }
}

// =======================================================================
// function : touchesMoved
// purpose  :
// =======================================================================
- (void)touchesMoved:(NSSet *)theTouches withEvent:(UIEvent *)theEvent
{
	if (_isPreviewMode) {
		return;
	}
    if(_cancelTouches) {
        [self touchesCancelled:theTouches withEvent:theEvent];
        return;
    }

    [super touchesMoved:theTouches withEvent:theEvent];
    
    UITouch *aTouch = [theTouches anyObject];
    if ((aTouch != NULL) && theEvent.allTouches.count == 1) {
        const CGPoint point = [self drawablePointForPoint:[aTouch locationInView:self.view]];
        _viewer->Rotation((int)point.x, (int)point.y);
        [self requestRender];

#ifdef DEBUG
        if (_delegate && [_delegate respondsToSelector:@selector(didChangeStatusString:)]) {
            [_delegate didChangeStatusString:[self statusString]];
        }
#endif
    }

    return;
}

-(void) cancellTouchEvents {
    _cancelTouches = YES;
}

-(void) touchesEnded:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event {
    [super touchesEnded:touches withEvent:event];

    UITouch *aTouch = [touches anyObject];
    if (!_isPreviewMode && aTouch != NULL) {
        const CGPoint point = [self drawablePointForPoint:[aTouch locationInView:self.view]];
        _viewer->FinishInteraction((int)point.x, (int)point.y);
        [self requestRender];

#ifdef DEBUG
        if (_delegate && [_delegate respondsToSelector:@selector(didChangeStatusString:)]) {
            [_delegate didChangeStatusString:[self statusString]];
        }
#endif
	}
	[_rawViewportTouches minusSet:touches];
	[self endRawPrimaryInteractionIfNeededCancelled:NO];

    return;
}

-(void) touchesCancelled:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event{
    [super touchesCancelled:touches withEvent:event];
	_rawTouchWasCancelled = YES;

    UITouch *aTouch = [touches anyObject];
    if (!_isPreviewMode && aTouch != NULL) {
        const CGPoint point = [self drawablePointForPoint:[aTouch locationInView:self.view]];
        _viewer->CancelInteraction((int)point.x, (int)point.y);
        [self requestRender];

#ifdef DEBUG
        if (_delegate && [_delegate respondsToSelector:@selector(didChangeStatusString:)]) {
            [_delegate didChangeStatusString:[self statusString]];
        }
#endif
	}
	[_rawViewportTouches minusSet:touches];
	[self endRawPrimaryInteractionIfNeededCancelled:YES];

    return;
}

// =======================================================================
// function : viewDidLoad
// purpose  :
// =======================================================================
-(void)viewDidLoad
{
    [super viewDidLoad];

    // add zoom recognizer
    UIPinchGestureRecognizer *aZoomRecognizer = [[UIPinchGestureRecognizer alloc]
                                                 initWithTarget:self
                                                 action:@selector(zoomHandler:)];
    aZoomRecognizer.delegate = self;

    [[self view] addGestureRecognizer:aZoomRecognizer];

    // add pan recognizer
    UIPanGestureRecognizer *aPanRecognizer = [[UIPanGestureRecognizer alloc]
                                              initWithTarget:self
                                              action:@selector(panHandler:)];

    aPanRecognizer.maximumNumberOfTouches = 2;
    aPanRecognizer.minimumNumberOfTouches = 2;
    aPanRecognizer.delegate = self;

    [[self view] addGestureRecognizer:aPanRecognizer];

    _tapRecognizer = [[UITapGestureRecognizer alloc]
                      initWithTarget:self
                      action:@selector(tapHandler:)];
    [[self view] addGestureRecognizer:_tapRecognizer];


    // add import buttons
    UIBarButtonItem *importScrewBtn = [[UIBarButtonItem alloc]
                                       initWithTitle:@"Sample 1"
                                       style:UIBarButtonItemStylePlain
                                       target:self
                                       action:@selector(importScrew:)];

    UIBarButtonItem *importLinkrodsBtn = [[UIBarButtonItem alloc]
                                          initWithTitle:@"Sample 2"
                                          style:UIBarButtonItemStylePlain
                                          target:self
                                          action:@selector(importLinkrods:)];


    UIBarButtonItem *addCubeBtn = [[UIBarButtonItem alloc]
                                   initWithTitle:@"Add cube"
                                   style:UIBarButtonItemStylePlain
                                   target:self
                                   action:@selector(addCube:)];

    UIBarButtonItem *displayAboutDlgBtn = [[UIBarButtonItem alloc]
                                           initWithTitle:@"About"
                                           style:UIBarButtonItemStylePlain
                                           target:self
                                           action:@selector(displayAboutDlg:)];

    [self.navigationItem setLeftBarButtonItems:[NSArray arrayWithObjects:importScrewBtn, importLinkrodsBtn, addCubeBtn, nil]];
    [self.navigationItem setRightBarButtonItem: displayAboutDlgBtn];
}

- (BOOL)gestureRecognizer:(UIGestureRecognizer *)gestureRecognizer
        shouldRecognizeSimultaneouslyWithGestureRecognizer:(UIGestureRecognizer *)otherGestureRecognizer
{
    const BOOL isPinchPanPair =
        ([gestureRecognizer isKindOfClass:UIPinchGestureRecognizer.class]
         && [otherGestureRecognizer isKindOfClass:UIPanGestureRecognizer.class])
        || ([gestureRecognizer isKindOfClass:UIPanGestureRecognizer.class]
            && [otherGestureRecognizer isKindOfClass:UIPinchGestureRecognizer.class]);
    return isPinchPanPair;
}

// =======================================================================
// function : zoomHandler
// purpose  :
// =======================================================================
- (void)zoomHandler:(UIPinchGestureRecognizer *)pinchRecognizer
{
    const UIGestureRecognizerState state = pinchRecognizer.state;
    if (state == UIGestureRecognizerStateEnded
        || state == UIGestureRecognizerStateCancelled
        || state == UIGestureRecognizerStateFailed) {
        if (_pinchRendering) {
            _pinchRendering = NO;
            [[self viewportView] endInteractiveRendering];
        }
        return;
    }

    if (_isPreviewMode) {
        return;
    }

    if (pinchRecognizer.numberOfTouches > 1) {
        if (state == UIGestureRecognizerStateBegan) {
            if (!_pinchRendering) {
                _pinchRendering = YES;
                [[self viewportView] beginInteractiveRendering];
            }
            _pinchPreviousTouch[0] = [pinchRecognizer locationOfTouch:0 inView:self.view];
            _pinchPreviousTouch[1] = [pinchRecognizer locationOfTouch:1 inView:self.view];
        } else if (state == UIGestureRecognizerStateChanged) {
            CGPoint aLastTouch[2] = {
                [pinchRecognizer locationOfTouch:0 inView:self.view],
                [pinchRecognizer locationOfTouch:1 inView:self.view]
            };

            const CGFloat drawableScale = [self drawableScale];
            double aPinchCenterXStart =
                (_pinchPreviousTouch[0].x + _pinchPreviousTouch[1].x) * drawableScale / 2.0;
            double aPinchCenterYStart =
                (_pinchPreviousTouch[0].y + _pinchPreviousTouch[1].y) * drawableScale / 2.0;

            double aStartDist = Sqrt( ( _pinchPreviousTouch[0].x - _pinchPreviousTouch[1].x ) * ( _pinchPreviousTouch[0].x - _pinchPreviousTouch[1].x ) +
                                     ( _pinchPreviousTouch[0].y - _pinchPreviousTouch[1].y ) * ( _pinchPreviousTouch[0].y - _pinchPreviousTouch[1].y ) );
            double anEndDist = Sqrt( ( aLastTouch[0].x - aLastTouch[1].x ) * ( aLastTouch[0].x - aLastTouch[1].x ) +
                                    ( aLastTouch[0].y - aLastTouch[1].y ) * ( aLastTouch[0].y - aLastTouch[1].y ) );

            double aDeltaDist = (anEndDist - aStartDist) * drawableScale;

            _viewer->Zoom(aPinchCenterXStart, aPinchCenterYStart, aDeltaDist);
            [self requestRender];

            _pinchPreviousTouch[0] = aLastTouch[0];
            _pinchPreviousTouch[1] = aLastTouch[1];
        }
    }
}

// =======================================================================
// function : panHandler
// purpose  :
// =======================================================================
- (void)panHandler:(UIPanGestureRecognizer *)panRecognizer
{
    const UIGestureRecognizerState state = panRecognizer.state;
    if (state == UIGestureRecognizerStateEnded
        || state == UIGestureRecognizerStateCancelled
        || state == UIGestureRecognizerStateFailed) {
        if (_panRendering) {
            _panRendering = NO;
            [[self viewportView] endInteractiveRendering];
        }
        return;
    }

    if (_isPreviewMode) {
        return;
    }

    if (panRecognizer.numberOfTouches > 1) {
        if (state == UIGestureRecognizerStateBegan) {
            if (!_panRendering) {
                _panRendering = YES;
                [[self viewportView] beginInteractiveRendering];
            }
            [panRecognizer setTranslation:CGPointZero inView:self.view];

        } else if (state == UIGestureRecognizerStateChanged) {
            const CGFloat drawableScale = [self drawableScale];
            const CGPoint translation = [panRecognizer translationInView:self.view];
            [panRecognizer setTranslation:CGPointZero inView:self.view];
            const int deltaX = (int)lround(translation.x * drawableScale);
            const int deltaY = (int)lround(-translation.y * drawableScale);
            if (deltaX != 0 || deltaY != 0) {
                // Starting a fresh relative pan from the current camera for each
                // incremental delta keeps panning composable with simultaneous
                // pinch updates instead of repeatedly restoring an old camera.
                _viewer->Pan(deltaX, deltaY, Standard_True);
                [self requestRender];
            }
        }
    }
}

// =======================================================================
// function : tapHandler
// purpose  :
// =======================================================================
- (void)tapHandler:(UITapGestureRecognizer *)tapRecognizer
{
    if (_suppressTapSelectionForManipulatorInteraction) {
        _suppressTapSelectionForManipulatorInteraction = NO;
        return;
    }
    if (_isPreviewMode) {
        return;
    }

    const CGPoint aTapPoint =
        [self drawablePointForPoint:[tapRecognizer locationInView:self.view]];
	if (!_isConstructorMode) {
		if (_delegate
			&& [_delegate respondsToSelector:
				@selector(viewer:willSelectAtDrawablePoint:drawableSize:)]) {
			[_delegate viewer:self
				willSelectAtDrawablePoint:aTapPoint
				             drawableSize:self.drawableSize];
		}
		_viewer->Select((int)aTapPoint.x, (int)aTapPoint.y);
	}

	[self checkSelections];
	[self requestRender];
}

-(void) checkSelections {
	bool isSelected = _viewer->getObjectInteractor()->isSelected();
	bool isManipulatorAttached = _viewer->getObjectInteractor()->isManipulatorAttached();
	unsigned char selections = Core3DViewer::kSelectionTypeNone;
	if (isSelected) {
		selections |= Core3DViewer::kSelectionTypeObject;
	}
	if (isManipulatorAttached) {
		selections |= Core3DViewer::kSelectionTypeManipulator;
	}
	
	if (_delegate && [_delegate respondsToSelector:@selector(viewer:didChangeSelections:)]) {
		__weak typeof(self) weakSelf = self;
		[_delegate viewer:weakSelf didChangeSelections:selections];
	}
}

// =======================================================================
// function : importScrew
// purpose  :
// =======================================================================
- (void)importScrew:(UIBarButtonItem *)theSender
{
    NSString* aNsPath = [[NSBundle mainBundle] pathForResource:@"screw"
                                                        ofType:@"step"];
    std::string aPath = std::string([aNsPath UTF8String]);

    _viewer->ImportSTEP(aPath);
    _viewer->FitAll();
    [self requestRender];
}

// =======================================================================
// function : importLinkrods
// purpose  :
// =======================================================================
- (void)importLinkrods:(UIBarButtonItem *)theSender
{
    NSString* aNsPath = [[NSBundle mainBundle] pathForResource:@"linkrods"
                                                        ofType:@"step"];
    std::string aPath = std::string([aNsPath UTF8String]);

    _viewer->ImportSTEP(aPath);
    _viewer->FitAll();
    [self requestRender];
}

- (void)addCube:(UIBarButtonItem *)theSender {
    __auto_type stlFilename = _viewer->addTestPrimitives();
    _viewer->FitAll();
    [self requestRender];
    [self shareFile:stlFilename];
}

- (void)shareFile:(NSString *)filepath {
    __auto_type activityController = [[UIActivityViewController alloc] initWithActivityItems:@[[NSURL fileURLWithPath:filepath]]
                                                                       applicationActivities:nil];
    if ([UIDevice currentDevice].userInterfaceIdiom == UIUserInterfaceIdiomPad) {
        activityController.popoverPresentationController.sourceView = self.view;
        activityController.popoverPresentationController.permittedArrowDirections = UIPopoverArrowDirectionAny;
        activityController.popoverPresentationController.sourceRect = CGRectMake(self.view.frame.size.width/2.0,
                                                                                 self.view.frame.size.height/2.0, 1.0, 1.0);
    }
    [self presentViewController:activityController animated:YES completion:nil];
}

- (void)addTestPrimitives {
    _viewer->addTestPrimitives();
    _viewer->FitAll();
    [self requestRender];
}

- (void)addPrimitivesFromJSON:(NSString *)json {
    _viewer->addPrimitivesFromJSON(json);
    [self requestRender];
}

- (void)addPrimitive:(PrimitiveType)primitiveType {
    _viewer->addPrimitive(primitiveType);
    [self requestRender];
}

- (void)selectLastObject {
    _viewer->getObjectInteractor()->selectLastObject();
    [self requestRender];
}

- (void)deleteSelected {
    _viewer->getObjectInteractor()->deleteSelected();
    [self requestRender];
}

-(NSString*) statusString {
	auto interactor = _viewer->getObjectInteractor();
	auto transform = interactor->manipulatorTransform();
	if (transform.IsNull()) {
		return @"No active selection";
	}
	auto pos = interactor->manipulatorPosition();
	auto rot = transform->Trsf().GetRotation();
	Standard_Real rotX, rotY, rotZ;
	rot.GetEulerAngles(gp_YawPitchRoll, rotX, rotY, rotZ);
	NSString* status = [NSString stringWithFormat:@"Coord: x%.3f, y%.3f, z%.3f. Angle:x%.3f, y%.3f, z%.3f",
			pos.X(), pos.Y(), pos.Z(), rotX / M_PI * 180, rotY / M_PI * 180, rotZ / M_PI * 180];
	return status;
}

- (void)selectAll {
	_viewer->getObjectInteractor()->selectAll();
	[self checkSelections];
	[self requestRender];
}

- (void)duplicateSelected {
    _viewer->getObjectInteractor()->duplicateSelected();
    [self requestRender];
}

- (void)deselectAll {
    if (_viewer != nullptr && _viewer->getShapeInteractor() != nullptr
        && !_viewer->getShapeInteractor()->cancelExtrusion()) {
        [self requestRender];
        return;
    }
    _viewer->deselectAll();
    [self requestRender];
}

- (BOOL)isSelected {
    return _viewer->getObjectInteractor()->isSelected();
}

- (void)fitAll {
    _viewer->FitAll();
    [self requestRender];
}

- (void)setPreviewMode {
    _isPreviewMode = YES;
    [self endActiveRenderingInteractions];
}

- (void)undo {
	PrimitiveGizmoType currentType = [self getGizmoType];
	const BOOL wasBoolean = currentType == PrimitiveGizmoTypeSubtract
		|| currentType == PrimitiveGizmoTypeUnion;
	const std::shared_ptr<ShapeInteractor> shapeInteractor =
		_viewer->getShapeInteractor();
	if (shapeInteractor != nullptr
		&& (currentType == PrimitiveGizmoTypeExtrude
			|| shapeInteractor->hasActiveExtrusion())) {
		if (shapeInteractor->cancelExtrusion()) {
			_viewer->getObjectInteractor()->setManipulatorType(
				PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
		}
		[self requestRender];
		return;
	}
	if (currentType == PrimitiveGizmoTypeSubtract) {
		if (![self retireBooleanActionForGizmoType:currentType]) {
			[self checkSelections];
			[self requestRender];
			return;
		}
	} else if (currentType == PrimitiveGizmoTypeUnion) {
		if (![self retireBooleanActionForGizmoType:currentType]) {
			[self checkSelections];
			[self requestRender];
			return;
		}
	} else if (currentType == PrimitiveGizmoTypeMirror) {
		_viewer->getObjectInteractor()->clearTrialMirrorObjects();
	}
	_viewer->getObjectInteractor()->detachManipulator(false);
	if (_viewer->getDocument()->canUndo()
		&& _viewer->getDocument()->undo()) {
		_viewer->redrawDocument();
	}
	if (wasBoolean) {
		(void)[self restoreBooleanActionForRetainedGizmoType:currentType];
	}
	[self checkSelections];
	[self requestRender];
}

- (void)redo {
	PrimitiveGizmoType currentType = [self getGizmoType];
	const BOOL wasBoolean = currentType == PrimitiveGizmoTypeSubtract
		|| currentType == PrimitiveGizmoTypeUnion;
	const std::shared_ptr<ShapeInteractor> shapeInteractor =
		_viewer->getShapeInteractor();
	if (shapeInteractor != nullptr
		&& (currentType == PrimitiveGizmoTypeExtrude
			|| shapeInteractor->hasActiveExtrusion())) {
		if (shapeInteractor->cancelExtrusion()) {
			_viewer->getObjectInteractor()->setManipulatorType(
				PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
		}
		[self requestRender];
		return;
	}
	if (currentType == PrimitiveGizmoTypeSubtract) {
		if (![self retireBooleanActionForGizmoType:currentType]) {
			[self checkSelections];
			[self requestRender];
			return;
		}
	} else if (currentType == PrimitiveGizmoTypeUnion) {
		if (![self retireBooleanActionForGizmoType:currentType]) {
			[self checkSelections];
			[self requestRender];
			return;
		}
	} else if (currentType == PrimitiveGizmoTypeMirror) {
		_viewer->getObjectInteractor()->clearTrialMirrorObjects();
	}
	_viewer->getObjectInteractor()->detachManipulator(false);
	if (_viewer->getDocument()->canRedo()
		&& _viewer->getDocument()->redo()) {
		_viewer->redrawDocument();
	}
	if (wasBoolean) {
		(void)[self restoreBooleanActionForRetainedGizmoType:currentType];
	}
	[self checkSelections];
	[self requestRender];
}

- (void)setSelectionType:(PrimitiveSelectionType)type {
	_viewer->getObjectInteractor()->cancelInteraction();
	PrimitiveGizmoType currentType = [self getGizmoType];
	if (currentType == PrimitiveGizmoTypeChamfer) {
		_viewer->getShapeInteractor()->resetWireframeTemplateShape();
	} else if (currentType == PrimitiveGizmoTypeSubtract) {
		if (![self retireBooleanActionForGizmoType:currentType]) {
			[self checkSelections];
			[self requestRender];
			return;
		}
	} else if (currentType == PrimitiveGizmoTypeUnion) {
		if (![self retireBooleanActionForGizmoType:currentType]) {
			[self checkSelections];
			[self requestRender];
			return;
		}
	} else if (currentType == PrimitiveGizmoTypeMirror) {
		_viewer->getObjectInteractor()->clearTrialMirrorObjects();
	} else if (currentType == PrimitiveGizmoTypeExtrude) {
		if (!_viewer->getShapeInteractor()->cancelExtrusion()) {
			[self requestRender];
			return;
		}
	}

    ShapeSelectionMode selectionMode;
    switch (type) {
        case PrimitiveSelectionTypeShape:
            selectionMode = ShapeSelectionMode::WholeShape;
            break;
        case PrimitiveSelectionTypeEdge:
            selectionMode = ShapeSelectionMode::Edge;
            break;
        case PrimitiveSelectionTypeFace:
            selectionMode = ShapeSelectionMode::Face;
            break;
        case PrimitiveSelectionTypeVertex:
            selectionMode = ShapeSelectionMode::Vertex;
            break;
        default:
            assert(false);
            break;
    }
    _viewer->getShapeInteractor()->setSelectionMode(selectionMode);
    (void)[self restoreBooleanActionForRetainedGizmoType:currentType];
    [self requestRender];
}

- (PrimitiveSelectionType)getSelectionType {
    PrimitiveSelectionType type = PrimitiveSelectionTypeNone;
    switch (_viewer->getShapeInteractor()->getSelectionMode()) {
        case ShapeSelectionMode::WholeShape:
            type = PrimitiveSelectionTypeShape;
            break;
        case ShapeSelectionMode::Edge:
            type = PrimitiveSelectionTypeEdge;
            break;
        case ShapeSelectionMode::Face:
            type = PrimitiveSelectionTypeFace;
            break;
        case ShapeSelectionMode::Vertex:
            type = PrimitiveSelectionTypeVertex;
            break;
        default:
            assert(false);
            break;
    }
    return type;
}

- (void)setGizmo:(Handle(Core3DManipulator))manipulator {
	_viewer->getObjectInteractor()->setManipulator(manipulator);
	[self requestRender];
}

- (void)setPrimitiveTransparent:(Handle(AIS_InteractiveObject))primitive transparent:(bool)set {
	_viewer->getObjectInteractor()->setObjectTransparent(primitive, set);
	[self requestRender];
}

- (void)setGizmoType:(PrimitiveGizmoType)type {
	const PrimitiveGizmoType previousType = [self getGizmoType];
	if (previousType == type) {
		if (type == PrimitiveGizmoTypeExtrude
			&& _viewer != nullptr
			&& _viewer->getShapeInteractor() != nullptr
			&& !_viewer->getShapeInteractor()->hasActiveExtrusion()
			&& !_viewer->getShapeInteractor()->beginExtrusionSelection()) {
			_viewer->getObjectInteractor()->setManipulatorType(
				PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
		}
		if (type == PrimitiveGizmoTypeSubtract
			|| type == PrimitiveGizmoTypeUnion) {
			(void)[self restoreBooleanActionForRetainedGizmoType:type];
		}
		[self requestRender];
		return;
	}

	// Resolve the previous tool before changing manipulator mode or capturing
	// selection for the next tool. Its AIS previews may refer to document labels
	// that the resolution step replaces or removes.
	if (previousType == PrimitiveGizmoTypeChamfer) {
		_viewer->getShapeInteractor()->resetWireframeTemplateShape();
	} else if (previousType == PrimitiveGizmoTypeSubtract) {
		if (![self retireBooleanActionForGizmoType:previousType]) {
			[self checkSelections];
			[self requestRender];
			return;
		}
	} else if (previousType == PrimitiveGizmoTypeUnion) {
		if (![self retireBooleanActionForGizmoType:previousType]) {
			[self checkSelections];
			[self requestRender];
			return;
		}
	} else if (previousType == PrimitiveGizmoTypeMirror) {
		_viewer->getObjectInteractor()->clearTrialMirrorObjects();
	} else if (previousType == PrimitiveGizmoTypeExtrude) {
		if (!_viewer->getShapeInteractor()->cancelExtrusion()) {
			[self requestRender];
			return;
		}
	}

    PrimitiveManipulatorType manipulatorType;
    switch (type) {
        case PrimitiveGizmoTypeMoveRotate:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate;
            break;
        case PrimitiveGizmoTypeScale:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeScale;
            break;
        case PrimitiveGizmoTypeChamfer:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer;
            break;
        case PrimitiveGizmoTypeNone:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeNone;
            break;
        case PrimitiveGizmoTypeSubtract:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract;
            break;
        case PrimitiveGizmoTypeUnion:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeUnion;
            break;
        case PrimitiveGizmoTypeMirror:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeMirror;
            break;
        case PrimitiveGizmoTypeMaterial:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeMaterial;
            break;
        case PrimitiveGizmoTypeExtrude:
            manipulatorType = PrimitiveManipulatorType::PrimitiveGizmoTypeExtrude;
            break;
        default:
            assert(false);
            break;
    }
    _viewer->getObjectInteractor()->setManipulatorType(manipulatorType);
    if (type == PrimitiveGizmoTypeChamfer) {
        _viewer->getShapeInteractor()->saveSelectionEdges();
	} else if (type == PrimitiveGizmoTypeSubtract || type == PrimitiveGizmoTypeUnion) {
			Standard_Boolean forceActor = (type == PrimitiveGizmoTypeSubtract);
			const BooleanAction action = type == PrimitiveGizmoTypeSubtract
				? BooleanAction::BooleanSubtract
				: BooleanAction::BooleanUnion;
			if (_viewer->getObjectInteractor()->beginBoolean(action)) {
				_viewer->getObjectInteractor()->fillSelectedState(
					forceActor,
					action);
			}
	} else if (type == PrimitiveGizmoTypeExtrude) {
		if (!_viewer->getShapeInteractor()->beginExtrusionSelection()) {
			_viewer->getObjectInteractor()->setManipulatorType(
				PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
		}
	}
	[self requestRender];
}

- (PrimitiveGizmoType)getGizmoType {
    PrimitiveGizmoType type = PrimitiveGizmoTypeNone;
    switch (_viewer->getObjectInteractor()->getManipulatorType()) {
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMoveRotate:
            type = PrimitiveGizmoTypeMoveRotate;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeScale:
            type = PrimitiveGizmoTypeScale;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeChamfer:
            type = PrimitiveGizmoTypeChamfer;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeNone:
            type = PrimitiveGizmoTypeNone;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeSubtract:
            type = PrimitiveGizmoTypeSubtract;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeUnion:
            type = PrimitiveGizmoTypeUnion;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMirror:
            type = PrimitiveGizmoTypeMirror;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeMaterial:
            type = PrimitiveGizmoTypeMaterial;
            break;
        case PrimitiveManipulatorType::PrimitiveGizmoTypeExtrude:
            type = PrimitiveGizmoTypeExtrude;
            break;
        default:
            break;
    }
    return type;
}

- (void)setChamfer:(CGFloat)value {
    assert([self getGizmoType] == PrimitiveGizmoTypeChamfer);
    (void)_viewer->getShapeInteractor()->setChamferValueForSelection(value);
    [self requestRender];
    //if (!result)
    //	std::cout << "incorrect chamfer value" << std::endl;
}

- (void)cancelChamfer {
    _viewer->getShapeInteractor()->cancelChamfer();
    [self requestRender];
}

- (BOOL)setExtrusion:(CGFloat)value {
    if ([self getGizmoType] != PrimitiveGizmoTypeExtrude
        || _viewer == nullptr
        || _viewer->getShapeInteractor() == nullptr) {
        return NO;
    }
    const BOOL result = _viewer->getShapeInteractor()
        ->setExtrusionValueForSelection(
            static_cast<Standard_Real>(value));
    [self requestRender];
    return result;
}

- (BOOL)applyExtrusion {
    if ([self getGizmoType] != PrimitiveGizmoTypeExtrude
        || _viewer == nullptr
        || _viewer->getShapeInteractor() == nullptr) {
        return NO;
    }
    const BOOL applied =
        _viewer->getShapeInteractor()->applyExtrusion();
    if (applied) {
        _viewer->redrawDocument();
    }
    [self requestRender];
    return applied;
}

- (BOOL)cancelExtrusion {
    const BOOL cancelled = _viewer == nullptr
        || _viewer->getShapeInteractor() == nullptr
        || _viewer->getShapeInteractor()->cancelExtrusion();
    [self requestRender];
    return cancelled;
}

- (BOOL)canApplyExtrusion {
    return _viewer != nullptr
        && _viewer->getShapeInteractor() != nullptr
        && (_viewer->getShapeInteractor()->canApplyExtrusion()
            || _viewer->getShapeInteractor()
                ->canRetryExtrusionResolution());
}

- (void) applyMirror {
    assert([self getGizmoType] == PrimitiveGizmoTypeMirror);
	_viewer->getObjectInteractor()->applyMirror();
	[self requestRender];
}

- (void) cancelMirror {
	_viewer->getObjectInteractor()->clearTrialMirrorObjects();
	[self requestRender];
}

- (BOOL) applySubtract {
	const BooleanApplyResult result =
		_viewer->getObjectInteractor()->applyBoolean(BooleanAction::BooleanSubtract);
	if (result == BooleanApplyResult::AppliedNeedsDocumentRedraw) {
		_viewer->redrawDocument();
	}
    [self requestRender];
    return result != BooleanApplyResult::NoChange;
}

- (BOOL) cancelSubtract {
    const std::shared_ptr<ObjectInteractor> anInteractor =
        _viewer->getObjectInteractor();
    anInteractor->cancelBoolean(BooleanAction::BooleanSubtract);
    [self requestRender];
    return !anInteractor->hasUnresolvedBoolean()
        && !anInteractor->hasActiveBoolean(
            BooleanAction::BooleanSubtract);
}

- (BOOL) applyUnion {
	const BooleanApplyResult result =
		_viewer->getObjectInteractor()->applyBoolean(BooleanAction::BooleanUnion);
	if (result == BooleanApplyResult::AppliedNeedsDocumentRedraw) {
		_viewer->redrawDocument();
    }
    [self requestRender];
    return result != BooleanApplyResult::NoChange;
}

- (BOOL) cancelUnion {
    const std::shared_ptr<ObjectInteractor> anInteractor =
        _viewer->getObjectInteractor();
    anInteractor->cancelBoolean(BooleanAction::BooleanUnion);
    [self requestRender];
    return !anInteractor->hasUnresolvedBoolean()
        && !anInteractor->hasActiveBoolean(BooleanAction::BooleanUnion);
}

- (BOOL) canApplyBoolean {
    return _viewer->getObjectInteractor()->canApplyBoolean();
}

- (BOOL)hasActiveBoolean {
    if (_viewer == nullptr
        || _viewer->getObjectInteractor() == nullptr) {
        return NO;
    }
    const PrimitiveGizmoType currentType = [self getGizmoType];
    if (currentType == PrimitiveGizmoTypeSubtract) {
        return _viewer->getObjectInteractor()->hasActiveBoolean(
            BooleanAction::BooleanSubtract);
    }
    if (currentType == PrimitiveGizmoTypeUnion) {
        return _viewer->getObjectInteractor()->hasActiveBoolean(
            BooleanAction::BooleanUnion);
    }
    return NO;
}

- (BOOL)restoreBooleanActionForRetainedGizmoType:(PrimitiveGizmoType)type {
    if (type != PrimitiveGizmoTypeSubtract
        && type != PrimitiveGizmoTypeUnion) {
        return YES;
    }
    if (_viewer == nullptr
        || _viewer->getObjectInteractor() == nullptr) {
        return NO;
    }
    const BooleanAction action = type == PrimitiveGizmoTypeSubtract
        ? BooleanAction::BooleanSubtract
        : BooleanAction::BooleanUnion;
    if (_viewer->getObjectInteractor()->hasActiveBoolean(action)) {
        return YES;
    }
    if (_viewer->getObjectInteractor()->beginBoolean(action)) {
        return YES;
    }
    if (_viewer->getObjectInteractor()->hasActiveBoolean(action)
        || _viewer->getObjectInteractor()->hasUnresolvedBoolean()) {
        // Cleanup is still retryable through the retained Boolean tool.
        return NO;
    }
    // A retained Boolean manipulator with no matching controller action is an
    // inert tool. Synchronize the public and native layers while exiting; do
    // not mutate the native manipulator behind the owner's cached UI state.
    if (_delegate != nil
        && [_delegate respondsToSelector:
            @selector(viewerDidFailToRetainBooleanMode:)]) {
        [_delegate viewerDidFailToRetainBooleanMode:self];
    } else {
        _viewer->getObjectInteractor()->setManipulatorType(
            PrimitiveManipulatorType::PrimitiveGizmoTypeNone);
    }
    return NO;
}

- (BOOL)retireBooleanActionForGizmoType:(PrimitiveGizmoType)type {
    if (type != PrimitiveGizmoTypeSubtract
        && type != PrimitiveGizmoTypeUnion) {
        return YES;
    }
    if (_viewer == nullptr
        || _viewer->getObjectInteractor() == nullptr) {
        return NO;
    }
    const BooleanAction action = type == PrimitiveGizmoTypeSubtract
        ? BooleanAction::BooleanSubtract
        : BooleanAction::BooleanUnion;
    const std::shared_ptr<ObjectInteractor> anInteractor =
        _viewer->getObjectInteractor();
    anInteractor->cancelBoolean(action);
    return !anInteractor->hasUnresolvedBoolean()
        && !anInteractor->hasActiveBoolean(action);
}

- (BOOL) hasTrialMirrorObjects {
	return _viewer != nullptr
		&& _viewer->getObjectInteractor() != nullptr
		&& _viewer->getObjectInteractor()->hasTrialMirrorObjects();
}

- (BOOL)isEmptyOfDisplayedObjects {
    return _viewer->getShapeInteractor()->isEmptyOfDisplayedObjects();
}

- (NSInteger)numberOfDisplayedShapes {
    return _viewer->getShapeInteractor()->getNumberOfDisplayedShapes();
}

#ifdef DEBUG
- (void)debugSetMaximumDisplayTraversalNodes:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->SetDebugMaximumDisplayTraversalNodes(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetMaximumLeafPresentations:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->SetDebugMaximumLeafPresentations(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetMaximumProjectTopologyValidationNodes:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->SetDebugMaximumProjectTopologyValidationNodes(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugResetProjectTopologyValidationCounters {
    if (_viewer != nullptr) {
        _viewer->DebugResetProjectTopologyValidationCounters();
    }
}

- (NSUInteger)debugBoundedProjectTopologyValidationCount {
    return _viewer == nullptr
        ? 0
        : static_cast<NSUInteger>(
            _viewer->DebugBoundedProjectTopologyValidationCount());
}

- (NSUInteger)debugGeometricBRepValidationCount {
    return _viewer == nullptr
        ? 0
        : static_cast<NSUInteger>(
            _viewer->DebugGeometricBRepValidationCount());
}

- (NSInteger)debugSelectedShapeCount {
    return _viewer == nullptr ? 0 : _viewer->selectedCount();
}

- (NSDictionary<NSString *, NSNumber *> *)debugBooleanPreviewState {
    if (_viewer == nullptr) {
        return @{};
    }
    const BooleanPreviewDebugState state =
        _viewer->DebugBooleanPreviewState();
    return @{
        @"state": @(static_cast<NSUInteger>(state.state)),
        @"generation": @(state.generation),
        @"submittedCount": @(state.submittedCount),
        @"startedCount": @(state.startedCount),
        @"completedCount": @(state.completedCount),
        @"cancelledCount": @(state.cancelledCount),
        @"pendingReplacementCount": @(state.pendingReplacementCount),
        @"acceptedCount": @(state.acceptedCount),
        @"staleSuppressionCount": @(state.staleSuppressionCount),
        @"activeOperation": @(state.activeOperation != Standard_False),
        @"workerActive": @(state.workerActive != Standard_False),
        @"workerPending": @(state.workerPending != Standard_False),
        @"canApply": @(state.canApply != Standard_False),
        @"documentCommandUnresolved": @(
            state.documentCommandUnresolved != Standard_False),
        @"documentCommandOpen": @(
            state.documentCommandOpen != Standard_False),
        @"lastComputeWasMainThread": @(
            state.lastComputeWasMainThread != Standard_False),
    };
}

- (void)debugSetBooleanPreviewWorkerBlocked:(BOOL)blocked {
    if (_viewer != nullptr) {
        _viewer->DebugSetBooleanPreviewWorkerBlocked(blocked);
    }
}

- (void)debugSetMaximumBooleanCaptureTopologyNodes:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->DebugSetMaximumBooleanCaptureTopologyNodes(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetMaximumBooleanResultTopologyNodes:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->DebugSetMaximumBooleanResultTopologyNodes(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetMaximumBooleanResultSolids:(NSUInteger)limit {
    if (_viewer != nullptr) {
        _viewer->DebugSetMaximumBooleanResultSolids(
            static_cast<Standard_Size>(limit));
    }
}

- (void)debugSetBooleanTransactionFailureCount:(NSUInteger)count {
    if (_viewer != nullptr) {
        _viewer->DebugSetBooleanTransactionFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetBooleanAbortFailureCount:(NSUInteger)count {
    if (_viewer != nullptr) {
        _viewer->DebugSetBooleanAbortFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetExtrusionCommitMode:(NSInteger)mode {
    if (_viewer != nullptr && _viewer->getShapeInteractor() != nullptr) {
        _viewer->getShapeInteractor()->debugSetExtrusionCommitMode(
            static_cast<Standard_Integer>(mode));
    }
}

- (void)debugSetExtrusionAbortFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getShapeInteractor() != nullptr) {
        _viewer->getShapeInteractor()->debugSetExtrusionAbortFailureCount(
            static_cast<Standard_Size>(count));
    }
}

- (void)debugSetExtrusionPostCommitInspectFailureCount:(NSUInteger)count {
    if (_viewer != nullptr && _viewer->getShapeInteractor() != nullptr) {
        _viewer->getShapeInteractor()
            ->debugSetExtrusionPostCommitInspectFailureCount(
                static_cast<Standard_Size>(count));
    }
}

- (NSDictionary<NSString *, NSNumber *> *)debugExtrusionState {
    if (_viewer == nullptr || _viewer->getShapeInteractor() == nullptr) {
        return @{};
    }
    const ExtrusionDebugState state =
        _viewer->getShapeInteractor()->debugExtrusionState();
    return @{
        @"selectionReady": @(state.selectionReady != Standard_False),
        @"previewActive": @(state.previewActive != Standard_False),
        @"canApply": @(state.canApply != Standard_False),
        @"resolutionRetryable": @(
            state.resolutionRetryable != Standard_False),
        @"commandOpen": @(state.commandOpen != Standard_False),
        @"lastApplySucceeded": @(
            state.lastApplySucceeded != Standard_False),
        @"distance": @(state.distance),
        @"sourceSubshapeCount": @(state.sourceSubshapeCount),
        @"profileEdgeCount": @(state.profileEdgeCount),
        @"candidateSubshapeCount": @(state.candidateSubshapeCount),
        @"candidateSolidCount": @(state.candidateSolidCount),
        @"lastFailureStage": @(state.lastFailureStage),
        @"lastFeatureStatus": @(state.lastFeatureStatus),
        @"lastCandidateShapeType": @(state.lastCandidateShapeType),
        @"rawDirectChildCount": @(state.rawDirectChildCount),
        @"rawDirectChildShapeType": @(state.rawDirectChildShapeType),
        @"historyHasModified": @(
            state.historyHasModified != Standard_False),
        @"historyHasGenerated": @(
            state.historyHasGenerated != Standard_False),
        @"historyFaceDeleted": @(
            state.historyFaceDeleted != Standard_False),
        @"candidateVolume": @(state.candidateVolume),
        @"candidateMinX": @(state.candidateMinX),
        @"candidateMinY": @(state.candidateMinY),
        @"candidateMinZ": @(state.candidateMinZ),
        @"candidateMaxX": @(state.candidateMaxX),
        @"candidateMaxY": @(state.candidateMaxY),
        @"candidateMaxZ": @(state.candidateMaxZ),
    };
}

- (NSDictionary<NSString *, NSNumber *> *)debugFramebufferStatistics {
    GLView *view = self.isViewLoaded ? [self viewportView] : nil;
    constexpr NSUInteger width = 96;
    constexpr NSUInteger height = 96;
    NSData *frame = _viewer == nullptr || view == nil
        ? nil
        : [view debugDrawAndReadCenteredRGBAWithWidth:width
                                               height:height];
    if (frame == nil || frame.length != width * height * 4) {
        return @{};
    }

    const Standard_Byte *pixels =
        static_cast<const Standard_Byte *>(frame.bytes);
    Standard_Size nearBlackCount = 0;
    Standard_Size chromaticCount = 0;
    Standard_Size redDominantCount = 0;
    Standard_Size greenDominantCount = 0;
    Standard_Size blueDominantCount = 0;
    Standard_Size brightNeutralCount = 0;
    Standard_Integer minimumRed = 255;
    Standard_Integer minimumGreen = 255;
    Standard_Integer minimumBlue = 255;
    Standard_Integer maximumRed = 0;
    Standard_Integer maximumGreen = 0;
    Standard_Integer maximumBlue = 0;
    for (NSUInteger pixelIndex = 0;
         pixelIndex < width * height;
         ++pixelIndex) {
        const Standard_Integer red = pixels[pixelIndex * 4];
        const Standard_Integer green = pixels[pixelIndex * 4 + 1];
        const Standard_Integer blue = pixels[pixelIndex * 4 + 2];
        minimumRed = std::min(minimumRed, red);
        minimumGreen = std::min(minimumGreen, green);
        minimumBlue = std::min(minimumBlue, blue);
        maximumRed = std::max(maximumRed, red);
        maximumGreen = std::max(maximumGreen, green);
        maximumBlue = std::max(maximumBlue, blue);
        const Standard_Integer maximum = std::max({red, green, blue});
        const Standard_Integer minimum = std::min({red, green, blue});
        if (maximum <= 12) {
            ++nearBlackCount;
        }
        if (maximum >= 32 && maximum - minimum >= 24) {
            ++chromaticCount;
        }
        if (red >= green + 24 && red >= blue + 24) {
            ++redDominantCount;
        }
        if (green >= red + 24 && green >= blue + 24) {
            ++greenDominantCount;
        }
        if (blue >= red + 24 && blue >= green + 24) {
            ++blueDominantCount;
        }
        if (minimum >= 96 && maximum - minimum < 24) {
            ++brightNeutralCount;
        }
    }
    const Standard_Size pixelCount = width * height;
    return @{
        @"captured": @YES,
        @"depthCaptured": @NO,
        @"width": @(width),
        @"height": @(height),
        @"pixelCount": @(pixelCount),
        @"nearBlackCount": @(nearBlackCount),
        @"chromaticCount": @(chromaticCount),
        @"redDominantCount": @(redDominantCount),
        @"greenDominantCount": @(greenDominantCount),
        @"blueDominantCount": @(blueDominantCount),
        @"brightNeutralCount": @(brightNeutralCount),
        @"modelPixelCount": @(pixelCount),
        @"modelNearBlackCount": @(nearBlackCount),
        @"modelChromaticCount": @(chromaticCount),
        @"modelRedDominantCount": @(redDominantCount),
        @"modelGreenDominantCount": @(greenDominantCount),
        @"modelBlueDominantCount": @(blueDominantCount),
        @"modelBrightNeutralCount": @(brightNeutralCount),
        @"minimumRed": @(minimumRed),
        @"minimumGreen": @(minimumGreen),
        @"minimumBlue": @(minimumBlue),
        @"maximumRed": @(maximumRed),
        @"maximumGreen": @(maximumGreen),
        @"maximumBlue": @(maximumBlue),
    };
}

- (NSArray<NSDictionary<NSString *, NSNumber *> *> *)
    debugDisplayedShapePresentationStates {
    if (_viewer == nullptr || _viewer->AisContext().IsNull()
        || _viewer->getDocument().IsNull()) {
        return @[];
    }
    AIS_ListOfInteractive displayed;
    _viewer->AisContext()->DisplayedObjects(AIS_KOI_Shape, -1, displayed);
    NSMutableArray<NSDictionary<NSString *, NSNumber *> *> *states =
        [NSMutableArray arrayWithCapacity:
            static_cast<NSUInteger>(displayed.Size())];
    for (AIS_ListIteratorOfListOfInteractive item(displayed);
         item.More(); item.Next()) {
        const Handle(AIS_Shape) shape =
            Handle(AIS_Shape)::DownCast(item.Value());
        if (shape.IsNull() || shape->Shape().IsNull()) {
            continue;
        }
        const gp_Trsf aLocalTransform = shape->LocalTransformation();
        const gp_XYZ translation = aLocalTransform.TranslationPart();
        const gp_Quaternion rotation = aLocalTransform.GetRotation();
        gp_Trsf aWorldTransform = aLocalTransform;
        aWorldTransform.Multiply(
            shape->Shape().Location().Transformation());
        const gp_XYZ aWorldTranslation =
            aWorldTransform.TranslationPart();
        Quantity_Color color(Quantity_NOC_BLACK);
        if (shape->HasColor()) {
            shape->Color(color);
        }
        Standard_Real red = 0.0;
        Standard_Real green = 0.0;
        Standard_Real blue = 0.0;
        color.Values(red, green, blue, Quantity_TOC_sRGB);
        Standard_Real metallic = -1.0;
        Standard_Real roughness = -1.0;
        Standard_Integer alphaMode = 999;
        Standard_Real alphaCutoff = -1.0;
        Standard_Integer faceCulling = 999;
        Standard_Boolean textureMapOn = Standard_False;
        Standard_Integer textureSetSize = 0;
        Standard_Boolean textureIsXCAFBridge = Standard_False;
        Standard_Integer textureUnit = -1;
        Standard_Size textureSourceByteCount = 0;
        Standard_Boolean textureDecodedImageAvailable = Standard_False;
        Standard_Integer textureDecodedWidth = 0;
        Standard_Integer textureDecodedHeight = 0;
        Standard_Integer textureDecodedFormat = -1;
        Standard_Boolean textureDecodedTopDown = Standard_False;
        std::array<Standard_Integer, 16> textureDecodedQuadrants;
        textureDecodedQuadrants.fill(-1);
        Standard_Boolean documentHasBaseColorTexture = Standard_False;
        Standard_Boolean textureSourceMatchesDocument = Standard_False;
        Handle(Image_Texture) documentBaseColorTexture;
        const TDF_Label presentationLabel =
            _viewer->getDocument()->ShapeLabel(shape);
        XCAFDoc_VisMaterialPBR effectivePresentationMaterial;
        if (!presentationLabel.IsNull()
            && _viewer->getDocument()->TryEffectivePBRMaterialForLabel(
                presentationLabel, effectivePresentationMaterial)
            && !effectivePresentationMaterial.BaseColorTexture.IsNull()) {
            documentHasBaseColorTexture = Standard_True;
            documentBaseColorTexture =
                effectivePresentationMaterial.BaseColorTexture;
        }
        const Handle(Prs3d_Drawer)& drawer = shape->Attributes();
        if (!drawer.IsNull() && !drawer->ShadingAspect().IsNull()) {
            const Graphic3d_PBRMaterial& pbrMaterial =
                drawer->ShadingAspect()->Material().PBRMaterial();
            metallic = pbrMaterial.Metallic();
            roughness = pbrMaterial.NormalizedRoughness();
            const Handle(Graphic3d_AspectFillArea3d)& fillAspect =
                drawer->ShadingAspect()->Aspect();
            if (!fillAspect.IsNull()) {
                alphaMode = static_cast<Standard_Integer>(
                    fillAspect->AlphaMode());
                alphaCutoff = fillAspect->AlphaCutoff();
                faceCulling = static_cast<Standard_Integer>(
                    fillAspect->FaceCulling());
                textureMapOn = fillAspect->ToMapTexture();
                const Handle(Graphic3d_TextureSet)& textureSet =
                    fillAspect->TextureSet();
                textureSetSize = textureSet.IsNull()
                    ? 0
                    : textureSet->Size();
                if (!textureSet.IsNull() && !textureSet->IsEmpty()) {
                    const Handle(XCAFPrs_Texture) texture =
                        Handle(XCAFPrs_Texture)::DownCast(
                            textureSet->First());
                    if (!texture.IsNull()) {
                        textureIsXCAFBridge = Standard_True;
                        if (!texture->GetParams().IsNull()) {
                            textureUnit = static_cast<Standard_Integer>(
                                texture->GetParams()->TextureUnit());
                        }
                        const Handle(Image_Texture)& source =
                            texture->GetImageSource();
                        if (!source.IsNull()
                            && !source->DataBuffer().IsNull()) {
                            textureSourceByteCount =
                                source->DataBuffer()->Size();
                            // Exercise the authoritative source decoder
                            // directly. TextureSet/source-handle state alone
                            // cannot prove that OpenGL received any texels.
                            const Handle(Image_PixMap) decoded =
                                source->ReadImage(
                                    Handle(Image_SupportedFormats)());
                            if (!decoded.IsNull()
                                && !decoded->IsEmpty()) {
                                textureDecodedImageAvailable =
                                    Standard_True;
                                textureDecodedWidth =
                                    static_cast<Standard_Integer>(
                                        decoded->SizeX());
                                textureDecodedHeight =
                                    static_cast<Standard_Integer>(
                                        decoded->SizeY());
                                textureDecodedFormat =
                                    static_cast<Standard_Integer>(
                                        decoded->Format());
                                textureDecodedTopDown =
                                    decoded->IsTopDown();
                                if (decoded->Format()
                                        == Image_Format_RGBA) {
                                    const Standard_Size aQuarterX =
                                        decoded->SizeX() / 4;
                                    const Standard_Size aQuarterY =
                                        decoded->SizeY() / 4;
                                    const Standard_Size threeQuarterX =
                                        decoded->SizeX() * 3 / 4;
                                    const Standard_Size threeQuarterY =
                                        decoded->SizeY() * 3 / 4;
                                    const std::array<
                                        std::array<Standard_Size, 2>,
                                        4> coordinates = {{
                                            {{aQuarterX, aQuarterY}},
                                            {{threeQuarterX, aQuarterY}},
                                            {{aQuarterX, threeQuarterY}},
                                            {{threeQuarterX,
                                              threeQuarterY}},
                                        }};
                                    for (Standard_Integer aSample = 0;
                                         aSample < 4;
                                         ++aSample) {
                                        const Standard_Byte* aPixel =
                                            decoded->RawValueXY(
                                                coordinates[aSample][0],
                                                coordinates[aSample][1]);
                                        for (Standard_Integer aChannel = 0;
                                             aChannel < 4;
                                             ++aChannel) {
                                            textureDecodedQuadrants[
                                                aSample * 4 + aChannel] =
                                                aPixel[aChannel];
                                        }
                                    }
                                }
                            }
                        }
                        if (!documentBaseColorTexture.IsNull()) {
                            textureSourceMatchesDocument =
                                Core3DBaseColorTexturesMatch(
                                    source,
                                    documentBaseColorTexture);
                        }
                    }
                }
            }
        }
        const Handle(CafShapePrs) cafShape =
            Handle(CafShapePrs)::DownCast(shape);
        Standard_Real defaultStyleRed = -1.0;
        Standard_Real defaultStyleGreen = -1.0;
        Standard_Real defaultStyleBlue = -1.0;
        Standard_Real defaultStyleMetallic = -1.0;
        Standard_Real defaultStyleRoughness = -1.0;
        Standard_Boolean defaultStyleHasMaterial = Standard_False;
        if (!cafShape.IsNull()) {
            XCAFPrs_Style defaultStyle;
            cafShape->DefaultStyle(defaultStyle);
            Quantity_Color defaultStyleColor(Quantity_NOC_BLACK);
            if (defaultStyle.IsSetColorSurf()) {
                defaultStyleColor = defaultStyle.GetColorSurf();
            } else if (defaultStyle.IsSetColorCurv()) {
                defaultStyleColor = defaultStyle.GetColorCurv();
            } else if (!defaultStyle.Material().IsNull()) {
                defaultStyleColor =
                    defaultStyle.Material()->BaseColor().GetRGB();
            }
            defaultStyleColor.Values(
                defaultStyleRed,
                defaultStyleGreen,
                defaultStyleBlue,
                Quantity_TOC_sRGB);
            if (!defaultStyle.Material().IsNull()) {
                defaultStyleHasMaterial = Standard_True;
                Graphic3d_MaterialAspect defaultStyleAspect;
                defaultStyle.Material()->FillMaterialAspect(
                    defaultStyleAspect);
                const Graphic3d_PBRMaterial& defaultStylePBR =
                    defaultStyleAspect.PBRMaterial();
                defaultStyleMetallic = defaultStylePBR.Metallic();
                defaultStyleRoughness =
                    defaultStylePBR.NormalizedRoughness();
            }
        }
        Handle(AIS_ColoredDrawer) rootCustomAspects;
        const Standard_Boolean hasRootCustomAspects =
            !cafShape.IsNull()
            && cafShape->FindCustomAspects(
                shape->Shape(), rootCustomAspects);
        Standard_Integer customMaterialOverrideCount = 0;
        Standard_Integer customColorOverrideCount = 0;
        Standard_Integer customTextureMapOnCount = 0;
        Standard_Integer customNonDefaultAlphaCount = 0;
        Standard_Integer customNonAutoFaceCullingCount = 0;
        if (!cafShape.IsNull()) {
            for (CafDataMapOfShapeColor::Iterator anOverride(
                     cafShape->ShapeColors());
                 anOverride.More(); anOverride.Next()) {
                const Handle(AIS_ColoredDrawer)& drawer =
                    anOverride.Value();
                if (!drawer.IsNull() && drawer->HasOwnMaterial()) {
                    ++customMaterialOverrideCount;
                }
                if (!drawer.IsNull() && drawer->HasOwnColor()) {
                    ++customColorOverrideCount;
                }
                if (!drawer.IsNull() && drawer->HasOwnShadingAspect()
                    && !drawer->ShadingAspect().IsNull()
                    && !drawer->ShadingAspect()->Aspect().IsNull()) {
                    const Handle(Graphic3d_AspectFillArea3d)& aspect =
                        drawer->ShadingAspect()->Aspect();
                    if (aspect->ToMapTexture()) {
                        ++customTextureMapOnCount;
                    }
                    if (aspect->AlphaMode()
                            != Graphic3d_AlphaMode_BlendAuto
                        || std::abs(aspect->AlphaCutoff() - 0.5f)
                            > 1.0e-6f) {
                        ++customNonDefaultAlphaCount;
                    }
                    if (aspect->FaceCulling()
                            != Graphic3d_TypeOfBackfacingModel_Auto) {
                        ++customNonAutoFaceCullingCount;
                    }
                }
            }
        }
        TColStd_ListOfInteger activeSelectionModes;
        _viewer->AisContext()->ActivatedModes(
            shape, activeSelectionModes);
        [states addObject:@{
            @"translationX": @(translation.X()),
            @"translationY": @(translation.Y()),
            @"translationZ": @(translation.Z()),
            @"rotationX": @(rotation.X()),
            @"rotationY": @(rotation.Y()),
            @"rotationZ": @(rotation.Z()),
            @"rotationW": @(rotation.W()),
            @"scaleFactor": @(aLocalTransform.ScaleFactor()),
            @"transform11": @(aLocalTransform.Value(1, 1)),
            @"transform12": @(aLocalTransform.Value(1, 2)),
            @"transform13": @(aLocalTransform.Value(1, 3)),
            @"transform14": @(aLocalTransform.Value(1, 4)),
            @"transform21": @(aLocalTransform.Value(2, 1)),
            @"transform22": @(aLocalTransform.Value(2, 2)),
            @"transform23": @(aLocalTransform.Value(2, 3)),
            @"transform24": @(aLocalTransform.Value(2, 4)),
            @"transform31": @(aLocalTransform.Value(3, 1)),
            @"transform32": @(aLocalTransform.Value(3, 2)),
            @"transform33": @(aLocalTransform.Value(3, 3)),
            @"transform34": @(aLocalTransform.Value(3, 4)),
            @"worldTranslationX": @(aWorldTranslation.X()),
            @"worldTranslationY": @(aWorldTranslation.Y()),
            @"worldTranslationZ": @(aWorldTranslation.Z()),
            @"hasColor": @(shape->HasColor()),
            @"red": @(red),
            @"green": @(green),
            @"blue": @(blue),
            @"metallic": @(metallic),
            @"roughness": @(roughness),
            @"alphaMode": @(alphaMode),
            @"alphaCutoff": @(alphaCutoff),
            @"faceCulling": @(faceCulling),
            @"isCafPresentation": @(!cafShape.IsNull()),
            @"textureMapOn": @(textureMapOn != Standard_False),
            @"textureSetSize": @(textureSetSize),
            @"textureIsXCAFBridge": @(
                textureIsXCAFBridge != Standard_False),
            @"textureUnit": @(textureUnit),
            @"textureSourceByteCount": @(textureSourceByteCount),
            @"textureDecodedImageAvailable": @(
                textureDecodedImageAvailable != Standard_False),
            @"textureDecodedWidth": @(textureDecodedWidth),
            @"textureDecodedHeight": @(textureDecodedHeight),
            @"textureDecodedFormat": @(textureDecodedFormat),
            @"textureDecodedIsRGBA": @(
                textureDecodedFormat
                    == static_cast<Standard_Integer>(
                        Image_Format_RGBA)),
            @"textureDecodedTopDown": @(
                textureDecodedTopDown != Standard_False),
            @"textureDecodedTopLeftRed": @(
                textureDecodedQuadrants[0]),
            @"textureDecodedTopLeftGreen": @(
                textureDecodedQuadrants[1]),
            @"textureDecodedTopLeftBlue": @(
                textureDecodedQuadrants[2]),
            @"textureDecodedTopLeftAlpha": @(
                textureDecodedQuadrants[3]),
            @"textureDecodedTopRightRed": @(
                textureDecodedQuadrants[4]),
            @"textureDecodedTopRightGreen": @(
                textureDecodedQuadrants[5]),
            @"textureDecodedTopRightBlue": @(
                textureDecodedQuadrants[6]),
            @"textureDecodedTopRightAlpha": @(
                textureDecodedQuadrants[7]),
            @"textureDecodedBottomLeftRed": @(
                textureDecodedQuadrants[8]),
            @"textureDecodedBottomLeftGreen": @(
                textureDecodedQuadrants[9]),
            @"textureDecodedBottomLeftBlue": @(
                textureDecodedQuadrants[10]),
            @"textureDecodedBottomLeftAlpha": @(
                textureDecodedQuadrants[11]),
            @"textureDecodedBottomRightRed": @(
                textureDecodedQuadrants[12]),
            @"textureDecodedBottomRightGreen": @(
                textureDecodedQuadrants[13]),
            @"textureDecodedBottomRightBlue": @(
                textureDecodedQuadrants[14]),
            @"textureDecodedBottomRightAlpha": @(
                textureDecodedQuadrants[15]),
            @"documentHasBaseColorTexture": @(
                documentHasBaseColorTexture != Standard_False),
            @"textureSourceMatchesDocument": @(
                textureSourceMatchesDocument != Standard_False),
            @"defaultStyleHasMaterial": @(defaultStyleHasMaterial),
            @"defaultStyleRed": @(defaultStyleRed),
            @"defaultStyleGreen": @(defaultStyleGreen),
            @"defaultStyleBlue": @(defaultStyleBlue),
            @"defaultStyleMetallic": @(defaultStyleMetallic),
            @"defaultStyleRoughness": @(defaultStyleRoughness),
            @"isAssemblyOccurrence": @(
                !cafShape.IsNull()
                && !cafShape->IsEditablePresentation()),
            @"isEditable": @(
                _viewer->getDocument()->IsPresentationEditable(shape)),
            @"hasRootCustomAspects": @(hasRootCustomAspects),
            @"customMaterialOverrideCount": @(
                customMaterialOverrideCount),
            @"customColorOverrideCount": @(
                customColorOverrideCount),
            @"customTextureMapOnCount": @(
                customTextureMapOnCount),
            @"customNonDefaultAlphaCount": @(
                customNonDefaultAlphaCount),
            @"customNonAutoFaceCullingCount": @(
                customNonAutoFaceCullingCount),
            @"activeSelectionModeCount": @(
                activeSelectionModes.Extent()),
        }];
    }
    return states;
}
#endif

- (NSInteger)numberOfDetectedEdges {
    return _viewer->getShapeInteractor()->getNumberOfDetectedEdges();
}

- (void)assetData:(void(^)(NSData *_Nullable))completion {
    __weak typeof(self) weakSelf = self;
    dispatch_async(_assetDataQueue, ^{
        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf) {
            dispatch_async(dispatch_get_main_queue(), ^{
                completion(nil);
            });
            return;
        }

        NSString *tmpFilename = [NSString stringWithFormat:@"%@.tmp", NSUUID.UUID.UUIDString];
        NSURL *tmpDirectory = [NSFileManager.defaultManager temporaryDirectory];
        NSURL *baseURL = [tmpDirectory URLByAppendingPathComponent:tmpFilename];
        NSString *expectedCbfPath = [baseURL.path stringByAppendingString:@".cbf"];
        NSString *expectedXbfPath = [baseURL.path stringByAppendingString:@".xbf"];
        const std::string fn = baseURL.path.UTF8String;

        __block std::string cbfFilePath;
        dispatch_sync(dispatch_get_main_queue(), ^{
            try {
                if (strongSelf->_viewer != nullptr) {
                    cbfFilePath = strongSelf->_viewer->getDocument()->save(fn);
                    if (!cbfFilePath.empty()) {
                        const AssetImportResult validation =
                            strongSelf->_viewer->ValidateCbf(cbfFilePath);
                        if (validation != AssetImportResult::Success) {
                            NSLog(@"CBF validation failed after save: %d at %@",
                                  static_cast<int>(validation),
                                  [NSString stringForStdString:cbfFilePath]);
                            cbfFilePath.clear();
                        }
                    }
                }
            } catch (...) {
                cbfFilePath.clear();
            }
        });

        NSString *dataPath = cbfFilePath.empty()
            ? nil
            : [NSString stringForStdString:cbfFilePath];
        NSError *error = nil;
        NSData *data = dataPath == nil
            ? nil
            : [NSData dataWithContentsOfFile:dataPath options:kNilOptions error:&error];

        NSFileManager *fileManager = NSFileManager.defaultManager;
        [fileManager removeItemAtURL:baseURL error:nil];
        [fileManager removeItemAtPath:expectedCbfPath error:nil];
        [fileManager removeItemAtPath:expectedXbfPath error:nil];
        if (dataPath != nil
            && ![dataPath isEqualToString:expectedCbfPath]
            && ![dataPath isEqualToString:expectedXbfPath]) {
            [fileManager removeItemAtPath:dataPath error:nil];
        }

        if (error != nil || !HasCbfMagic(data)) {
            data = nil;
        }

        dispatch_async(dispatch_get_main_queue(), ^{
            completion(data);
        });
    });
}

- (void)setAssetData:(NSData *)data completion:(void(^)(Core3DAssetLoadResult result))completion {
    __weak typeof(self) weakSelf = self;
    dispatch_async(_assetDataQueue, ^{
        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf) {
            CompleteAssetLoadOnMain(completion, Core3DAssetLoadResultInternalFailure);
            return;
        }
        static const NSUInteger kMaximumProjectDocumentBytes =
            256ull * 1024ull * 1024ull;
        if (data.length > kMaximumProjectDocumentBytes
            || !HasCbfMagic(data)) {
            CompleteAssetLoadOnMain(completion, Core3DAssetLoadResultInvalidData);
            return;
        }

        // OCCT reads FILE_FORMAT from the binary header before consulting the
        // path extension. Both BinOcaf and BinXCAF readers are registered, so
        // a neutral fixed suffix avoids duplicating its header parser here.
        NSString *tmpFilename = [NSString stringWithFormat:@"%@.tmp.cbf",
                                  NSUUID.UUID.UUIDString];
        NSURL *tmpUrl = [[NSFileManager.defaultManager temporaryDirectory] URLByAppendingPathComponent:tmpFilename];
        NSError *error = nil;
        [data writeToURL:tmpUrl options:NSDataWritingAtomic error:&error];
        if (error != nil) {
            [NSFileManager.defaultManager removeItemAtURL:tmpUrl error:nil];
            CompleteAssetLoadOnMain(completion, Core3DAssetLoadResultTemporaryFileFailure);
            return;
        }

        const std::string fn = tmpUrl.path.UTF8String;
        __block Core3DAssetLoadResult result = Core3DAssetLoadResultInternalFailure;
        dispatch_sync(dispatch_get_main_queue(), ^{
            try {
                if (strongSelf->_viewer != nullptr) {
                    [strongSelf endActiveRenderingInteractions];
                    result = AssetLoadResultFromImportResult(strongSelf->_viewer->ImportCbf(fn));
                    if (result == Core3DAssetLoadResultSuccess) {
                        [strongSelf requestRender];
                    }
                }
            } catch (...) {
                result = Core3DAssetLoadResultInternalFailure;
            }
        });
        [NSFileManager.defaultManager removeItemAtURL:tmpUrl error:nil];
        CompleteAssetLoadOnMain(completion, result);
    });
}

- (void)setAssetFileURL:(NSURL *)assetFileURL
      expectedByteCount:(unsigned long long)expectedByteCount
          expectedSHA256:(NSString *)expectedSHA256
              completion:(void(^)(Core3DAssetLoadResult result))completion {
    __weak typeof(self) weakSelf = self;
    dispatch_async(_assetDataQueue, ^{
        __strong typeof(weakSelf) strongSelf = weakSelf;
        if (!strongSelf) {
            CompleteAssetLoadOnMain(
                completion, Core3DAssetLoadResultInternalFailure);
            return;
        }
        NSURL *stagedURL = nil;
        const Core3DAssetLoadResult stagingResult = StageVerifiedAssetFile(
            assetFileURL,
            expectedByteCount,
            expectedSHA256,
            &stagedURL);
        if (stagingResult != Core3DAssetLoadResultSuccess
            || stagedURL == nil) {
            CompleteAssetLoadOnMain(completion, stagingResult);
            return;
        }

        const std::string filename = stagedURL.path.UTF8String;
        __block Core3DAssetLoadResult result =
            Core3DAssetLoadResultInternalFailure;
        dispatch_sync(dispatch_get_main_queue(), ^{
            try {
                if (strongSelf->_viewer != nullptr) {
                    [strongSelf endActiveRenderingInteractions];
                    result = AssetLoadResultFromImportResult(
                        strongSelf->_viewer->ImportCbf(filename));
                    if (result == Core3DAssetLoadResultSuccess) {
                        [strongSelf requestRender];
                    }
                }
            } catch (...) {
                result = Core3DAssetLoadResultInternalFailure;
            }
        });
        [NSFileManager.defaultManager removeItemAtURL:stagedURL error:nil];
        CompleteAssetLoadOnMain(completion, result);
    });
}

- (NSData *)thumbData {
    NSString *tmpSnapthotFilename = [NSString stringWithFormat:@"%@.tmp.png", NSUUID.UUID.UUIDString];
    NSURL *tmpUrl = [[NSFileManager.defaultManager temporaryDirectory] URLByAppendingPathComponent:tmpSnapthotFilename];
    NSLog(@"Snapshot: %@", tmpUrl);
    if (![self saveSnapshot:tmpUrl]) {
        [NSFileManager.defaultManager removeItemAtURL:tmpUrl error:nil];
        return NULL;
    }
    NSError *error = nil;
    __auto_type data = [NSData dataWithContentsOfURL:tmpUrl options:kNilOptions error:&error];

    [NSFileManager.defaultManager removeItemAtURL:tmpUrl error:nil];

    if (error) {
        NSLog(@"ERROR: with thumb: %@", error.localizedDescription);
        return NULL;
    }

    return data;
}

- (BOOL)saveSnapshot {
    NSData *data = [self thumbData];
    UIImage *image = data == nil ? nil : [UIImage imageWithData:data];
    if (image == nil) {
        return NO;
    }
    dispatch_async(dispatch_get_main_queue(), ^{
        UIImageWriteToSavedPhotosAlbum(image, nil, nil, nil);
    });
    return YES;
}

- (BOOL)saveSnapshot:(NSURL *)tmpUrl {
    const auto fn = TCollection_AsciiString(tmpUrl.path.UTF8String);
    return _viewer->dumpOfDisplayedColoredObjects(500, 500, tmpUrl.path.UTF8String);
}

- (NSURL *_Nullable)exportWithType:(ExportType)exportType {
    if (_viewer == nullptr) {
        return nil;
    }
    const Handle(OcctDocument) document = _viewer->getDocument();
    const Handle(TDocStd_Document) transaction = document.IsNull()
        ? Handle(TDocStd_Document)()
        : document->ChangeDocument();
    const auto objectInteractor = _viewer->getObjectInteractor();
    if (transaction.IsNull() || transaction->HasOpenCommand()
        || objectInteractor == nullptr
        || objectInteractor->hasActiveBoolean()
        || objectInteractor->hasUnresolvedBoolean()
        || objectInteractor->hasUnresolvedMirrorObjects()) {
        return nil;
    }
    NSString *pathExtension = NULL;
    switch (exportType) {
        case ExportTypeObj:
            pathExtension = @"obj";
            break;
        case ExportTypeStl:
            pathExtension = @"stl";
            break;
        case ExportTypeGltf:
            pathExtension = @"glb";
            break;
        case ExportTypeStep:
            pathExtension = @"step";
            break;
        default:
            assert(false);
            break;
    }

    NSString *exportFilename = [NSUUID.UUID.UUIDString
        stringByAppendingPathExtension:pathExtension];
    NSURL *exportUrl = [[NSFileManager.defaultManager temporaryDirectory] URLByAppendingPathComponent:exportFilename];
    void (^removeExportArtifacts)(void) = ^{
        [NSFileManager.defaultManager removeItemAtURL:exportUrl error:nil];
        if (exportType == ExportTypeObj) {
            NSURL *baseUrl = [exportUrl URLByDeletingPathExtension];
            [NSFileManager.defaultManager
                removeItemAtURL:[baseUrl URLByAppendingPathExtension:@"mtl"]
                         error:nil];
            [NSFileManager.defaultManager removeItemAtURL:baseUrl error:nil];
            NSURL *textureDirectory = [NSURL fileURLWithPath:
                [baseUrl.path stringByAppendingString:@"_textures"]];
            [NSFileManager.defaultManager
                removeItemAtURL:textureDirectory
                         error:nil];
        }
    };

    const auto exportPath = std::string(exportUrl.path.UTF8String);

    switch (exportType) {
        case ExportTypeObj:
            _viewer->getShapeInteractor()->exportToObj(exportPath);
            break;
        case ExportTypeStl:
            _viewer->getShapeInteractor()->exportToStl(exportPath);
            break;
        case ExportTypeGltf:
            _viewer->getShapeInteractor()->exportToGltf(exportPath);
            break;
        case ExportTypeStep:
            _viewer->getShapeInteractor()->exportToStep(exportPath);
            break;
        default:
            assert(false);
            break;
    }


    // Verify the file was actually created
    if (![NSFileManager.defaultManager fileExistsAtPath:exportUrl.path]) {
        NSLog(@"[Export] File was NOT created at: %@", exportUrl.path);
        removeExportArtifacts();
        return nil;
    }
    
    NSDictionary *attrs = [NSFileManager.defaultManager attributesOfItemAtPath:exportUrl.path error:nil];
    NSLog(@"[Export] File created: %@ (%llu bytes)", exportUrl.lastPathComponent, [attrs fileSize]);
    
    if ([attrs fileSize] == 0) {
        NSLog(@"[Export] File is empty, returning nil");
        removeExportArtifacts();
        return nil;
    }
    
    return exportUrl;
}

- (BOOL)isPreviewMode {
    return _isPreviewMode;
}

- (void)setOrthoProjection:(OrthoProjectionType)orthoType {
    _viewer->setOrthoProjection(orthoType);
    [self requestRender];
}

// =======================================================================
// function : displayAboutDlg
// purpose  :
// =======================================================================
- (void)displayAboutDlg:(UIBarButtonItem *)theSender
{
  UIAlertController* anAbout = [UIAlertController alertControllerWithTitle:@"About"
                                message:@"UIKit based application for tutorial to Open CASCADE Technology.\n\n"
                                      @"Copyright (c) 2017 OPEN CASCADE SAS"
                                preferredStyle:UIAlertControllerStyleAlert];
  
  UIAlertAction* aDefaultAction = [UIAlertAction actionWithTitle:@"OK" style:UIAlertActionStyleDefault
                                                         handler:^(UIAlertAction * action) {}];
  
  [anAbout addAction:aDefaultAction];
  [self presentViewController:anAbout animated:YES completion:nil];
}

@end
