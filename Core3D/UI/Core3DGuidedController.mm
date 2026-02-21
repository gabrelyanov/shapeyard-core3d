//
//  Core3DGuidedController.m
//  Core3D
//
//  Created by Dmitry Sukhorukov on 11.09.2024.
//

#import "Core3DGuidedController.h"
#import "NSBundle+Path.h"
#import "GLViewController.h"

#include <V3d_View.hxx>
#include <V3d_DirectionalLight.hxx>
#include <V3d_AmbientLight.hxx>
#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_DisplayMode.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <Aspect_NeutralWindow.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <Image_AlienPixMap.hxx>
#include <BRepBuilderAPI_Copy.hxx>

#include "GLViewController+Trick.h"
#include "ConstructorManipulator.hpp"
#include "GuidedObjectsMatcher.hpp"

#include <chrono>

@interface Core3DGuidedController ()

@property (nonatomic) Handle(ConstructorManipulator) manipulator;
@property (nonatomic) std::shared_ptr<GuidedObjectsMatcher> matcher;

@end

@implementation Core3DGuidedController {
    std::map<int, Handle(AIS_InteractiveObject)> _aisObjectsMap;
    Bnd_Box _modelBox;
}

- (void)viewDidLoad {
    [super viewDidLoad];
    // Do any additional setup after loading the view.
}

-(void) dealloc {
    
}

- (void)loadFromBundle:(NSURL *)bundleUrl {
    NSLog(@"loadFromBundle %@", bundleUrl);
    
    if(bundleUrl == nil) {
        return;
    }
    
    if(_parts != nil && _parts.count > 0) {
        return;
    }
    
    [super loadFromBundle:bundleUrl];
}

- (void)viewDidLoadFromBundle {
    [super viewDidLoadFromBundle];
    [self setupGizmo];
}

-(void) notifyProgress:(NSUInteger)progress total:(NSUInteger)total {
    if(_guidedControllerDelegate
       && [_guidedControllerDelegate respondsToSelector:@selector(didUpdatePrepareProgress:total:)]) {
        [self.guidedControllerDelegate didUpdatePrepareProgress:progress total:total];
    }
}

-(NSArray<ModelPart*>*) assemblyParts {
    NSMutableArray* parts = [NSMutableArray array];
    auto glc = (GLViewController*)self.glController;
    auto context = [glc viewer]->AisContext();
    AIS_ListOfInteractive objects;
    context->DisplayedObjects(AIS_KOI_Shape, -1, objects);

    AIS_ListIteratorOfListOfInteractive iobject(objects);

	auto t0 = std::chrono::high_resolution_clock::now();

	Handle(AIS_InteractiveContext) ctx;
	Handle(V3d_View) view;
	[self prepareOffscreenView:view withContext:ctx];
    int idx = 0;
    const auto total = objects.Size();
    
    Bnd_Box box;
    
    while (iobject.More()) {
        TopoDS_Shape myShape = Handle(AIS_Shape)::DownCast(iobject.Value())->Shape();

        [self notifyProgress:idx total:total];
        ModelPart*p = [ModelPart new];
        p.identity = (NSUInteger)++idx;
        p.aligned = NO;
		p.thumbImage = [self imageForShape:myShape withContext:ctx view:view];

        _aisObjectsMap[idx] = iobject.Value();
        
        Bnd_Box theBndBox;
        iobject.Value()->BoundingBox(theBndBox);
        box.Add(theBndBox);

        [parts addObject:p];
        iobject.Next();
    }
    
    _modelBox = box;

	std::chrono::duration<double> duration = std::chrono::high_resolution_clock::now() - t0;
	std::cout << "Thumb generation " << (double)duration.count() << " seconds" << std::endl;

    return [parts copy];
}

-(void) prepareOffscreenView:(Handle(V3d_View)&)view withContext:(Handle(AIS_InteractiveContext)&)context; {
	const Standard_Integer width = 256;
	const Standard_Integer height = 256;

	Handle(Aspect_DisplayConnection) displayConnection = new Aspect_DisplayConnection();
	Handle(OpenGl_GraphicDriver) graphicDriver = new OpenGl_GraphicDriver(displayConnection, Standard_False);
	graphicDriver->ChangeOptions().buffersNoSwap = true;
	graphicDriver->InitContext();

	Handle(V3d_Viewer) viewer = new V3d_Viewer(graphicDriver);
	viewer->SetDefaultTypeOfView(V3d_PERSPECTIVE);
	Handle(V3d_DirectionalLight) lightDir = new V3d_DirectionalLight(V3d_Zneg, Quantity_Color(Quantity_NOC_WHITE), Standard_True);
	Handle(V3d_AmbientLight)     lightAmb = new V3d_AmbientLight();
	lightDir->SetDirection(1.0, -2.0, -10.0);
	viewer->AddLight(lightDir);
	viewer->AddLight(lightAmb);
	viewer->SetLightOn(lightDir);
	viewer->SetLightOn(lightAmb);
	viewer->SetComputedMode(Standard_False);
	viewer->SetDefaultBackgroundColor(Quantity_Color(Quantity_NOC_BLACK));

	// prepare context
	context = new AIS_InteractiveContext(viewer);
	// prepare off-screen view
	view = viewer->CreateView();

	Graphic3d_RenderingParams& aRendParams = view->ChangeRenderingParams();
	aRendParams.ToEnableAlphaToCoverage = false; // multisampling
	aRendParams.RenderResolutionScale = 1.0f; // supersampling

	Handle(Aspect_NeutralWindow) wnd = new Aspect_NeutralWindow();
	EAGLContext* aRendCtx = [EAGLContext currentContext];

	wnd->SetSize(width, height);
	wnd->SetVirtual(true);
	view->SetWindow(wnd, aRendCtx);
	view->MustBeResized();
}

-(UIImage*) imageForShape:(TopoDS_Shape)shape withContext:(Handle(AIS_InteractiveContext))context view:(Handle(V3d_View))view {
	const Standard_Integer width = 256;
	const Standard_Integer height = 256;

    // prepare presentation filled by shape
    Handle(AIS_Shape) presentation = new AIS_Shape(shape);
    presentation->SetMaterial(Graphic3d_NameOfMaterial_ShinyPlastified);
    presentation->SetColor(Quantity_NOC_GRAY50);
    context->SetDisplayMode(presentation, AIS_Shaded, Standard_False);
	context->Display(presentation, Standard_False);
    view->FitAll();

    // prepare pixmap image
    Image_AlienPixMap img;
	bool hasDump = view->ToPixMap(img, width, height);
	context->Remove(presentation, Standard_False);

	if (hasDump) {

		CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
		CGBitmapInfo bitmapInfo = kCGImageByteOrderDefault;

		const auto sz = sizeof(unsigned char)*img.SizeBytes();

		CGDataProviderRef provider = CGDataProviderCreateWithData(nil, img.Data(), sz, nil);
		CGImageRef cgImageRef = CGImageCreate(width, height, 8, 24, img.SizeRowBytes(), colorSpace, bitmapInfo, provider, nil, false, (CGColorRenderingIntent)kCGRenderingIntentDefault);

		UIGraphicsBeginImageContext(CGSizeMake(width, height));
		[[UIImage imageWithCGImage:cgImageRef scale:1.0 orientation:UIImageOrientationDownMirrored] drawInRect:CGRectMake(0,0,width ,height)];
		UIImage* newImage = UIGraphicsGetImageFromCurrentImageContext();
		UIGraphicsEndImageContext();

		CFRelease(provider);
		CFRelease(colorSpace);
		CFRelease(cgImageRef);

		img.Clear();

		return newImage;

	} else {
		return nil;
	}
}

-(void) setupGizmo {
    _manipulator = new ConstructorManipulator();
    _manipulator->SetSize(200);
    _manipulator->SetZoomPersistence(Standard_True);
    _manipulator->SetModeActivationOnDetection(Standard_True);
    [GLController setGizmo:_manipulator];
    [GLController setConstructorMode];
    [self setGizmoType:PrimitiveGizmoTypeMoveRotate];

    [self setManipulatorAxisIndex:0];
}

-(void) setManipulatorAxisIndex:(Standard_Integer)index {
    if (_manipulator.IsNull()) {
        [self setupGizmo];
    }

    _manipulator->SetAxisIndex(index);
}

-(void) transparentAll {
    auto glc = (GLViewController*)self.glController;
    for(auto p : _aisObjectsMap) {
        [glc setPrimitiveTransparent:p.second transparent:true];
    }
}

-(void) prepareGuidedParts {
    _parts = [self assemblyParts];
    
    
    if(_guidedControllerDelegate
       && [_guidedControllerDelegate respondsToSelector:@selector(didPartsUpdated)]) {
        [_guidedControllerDelegate didPartsUpdated];
    }

    auto glc = (GLViewController*)self.glController;

    __weak typeof(self) weakSelf = self;
    [glc viewer]->setInteractiveCallback([weakSelf](int x, int y) {
        if(weakSelf == nil) {
            return;
        }
        if(weakSelf.matcher) {
            weakSelf.matcher->matchTransforms([weakSelf]() {
                if(weakSelf == nil) {
                    return;
                }

                __auto_type part = weakSelf.parts[weakSelf.matcher->tag() - 1];
                part.aligned = YES;
                
                auto glc = (GLViewController*)weakSelf.glController;
                [glc setPrimitiveTransparent:weakSelf.matcher->first() transparent:false];
                auto context = [glc viewer]->AisContext();

                weakSelf.manipulator->Detach();
                context->ClearSelected(Standard_False);
                context->Remove(weakSelf.matcher->second(), Standard_True);
                context->UpdateCurrentViewer();
                [glc cancellTouchEvents];

                if(weakSelf.guidedControllerDelegate != nil
                   && [weakSelf.guidedControllerDelegate respondsToSelector:@selector(didPartAligned:)]) {
                    [weakSelf.guidedControllerDelegate didPartAligned:part];
                }
                weakSelf.matcher = nullptr;

            });
        }
    });

    [self transparentAll];
    [glc viewer]->AisContext()->UpdateCurrentViewer();

}

- (void)guidedPlace:(ModelPart*)part {
    if(_aisObjectsMap.find((int)part.identity) == _aisObjectsMap.end()) {
        NSLog(@"Object not found in map");
        return;
    }
    
    auto glc = (GLViewController*)self.glController;
    auto context = [glc viewer]->AisContext();
    
    for (auto partInner : _aisObjectsMap) {
        if(_parts[partInner.first - 1].aligned) {
            continue;
        }
        context->Erase(partInner.second, Standard_False);
        if (partInner.first == (int)part.identity) {
            context->Display(partInner.second, AIS_Shaded, (Standard_Integer)-1, Standard_False);
            [glc setPrimitiveTransparent:partInner.second transparent:true];
        }
    }
    
    auto ais_original = _aisObjectsMap[(int)part.identity];
    NSLog(@"ais original found");
    
    Handle(AIS_Shape) anAis = Handle(AIS_Shape)::DownCast(ais_original);
    
    BRepBuilderAPI_Copy shapeCopy(anAis->Shape(), Standard_True, Standard_False);
    Handle(AIS_InteractiveObject) ais_copy = new AIS_Shape(shapeCopy);
    
    float rndTr = float(arc4random_uniform(150)) / 200.0f + 0.3f;
    rndTr *= ((arc4random_uniform(2) == 0) ? -1.0f : 1.0f);
    
    gp_Trsf transform = ais_original->LocalTransformation();
    gp_XYZ translation(0,0,0);
    
    int axisNum = arc4random_uniform(3);
    _manipulator->SetAxisIndex(axisNum);
    switch (axisNum) {
        case 0:
            translation.SetX(rndTr * abs(_modelBox.CornerMax().X() - _modelBox.CornerMin().X()));
            break;
        case 1:
            translation.SetY(rndTr * abs(_modelBox.CornerMax().Y() - _modelBox.CornerMin().Y()));
            break;
        case 2:
            translation.SetZ(rndTr * abs(_modelBox.CornerMax().Z() - _modelBox.CornerMin().Z()));
            break;
    }
    
    transform.SetTranslationPart(transform.TranslationPart() + translation);
    ais_copy->SetLocalTransformation(transform);
    
    [glc setPrimitiveTransparent:ais_copy transparent:false];
    context->Display(ais_copy, AIS_Shaded, (Standard_Integer)-1, Standard_False);
    
    _manipulator->Detach();
    //    context->SetSelected(ais_copy, Standard_True);

    //	Handle(Prs3d_Drawer) theDrawer = new Prs3d_Drawer();
    //	Quantity_Color theColor;
    //	ais_copy->Color(theColor);
    //	theDrawer->SetColor(theColor);
    //	ais_copy->SetHilightAttributes(theDrawer);
    //	ais_copy->SetDynamicHilightAttributes(theDrawer);

    _manipulator->Attach(ais_copy);

    Bnd_Box newBox(_modelBox);
    gp_Pnt cen(( newBox.CornerMax().X() + newBox.CornerMin().X() ) / 2.0,
               ( newBox.CornerMax().Y() + newBox.CornerMin().Y() ) / 2.0,
               ( newBox.CornerMax().Z() + newBox.CornerMin().Z() ) / 2.0);

    Bnd_Box bb;
    ais_copy->BoundingBox(bb);
    
    //min
    if(bb.CornerMin().X() < newBox.CornerMin().X()
       || bb.CornerMin().Y() < newBox.CornerMin().Y()
       || bb.CornerMin().Z() < newBox.CornerMin().Z()) {
        newBox.Add(bb.CornerMin());
        newBox.Add(bb.CornerMin().Mirrored(cen));
    }

    //max
    if(bb.CornerMax().X() > newBox.CornerMax().X()
       || bb.CornerMax().Y() > newBox.CornerMax().Y()
       || bb.CornerMax().Z() > newBox.CornerMax().Z()) {
        newBox.Add(bb.CornerMax());
        newBox.Add(bb.CornerMax().Mirrored(cen));
    }
    
    [glc viewer]->FitBox(newBox);
    
    _matcher = GuidedObjectsMatcher::Make(ais_original, ais_copy, axisNum, 0.1); // match with 10% tolerance along desired axis
    _matcher->setTag((int)part.identity);

}

/*
 #pragma mark - Navigation

 // In a storyboard-based application, you will often want to do a little preparation before navigation
 - (void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender {
 // Get the new view controller using [segue destinationViewController].
 // Pass the selected object to the new view controller.
 }
 */


#pragma mark GuidedControllerDataSource

-(NSUInteger) partsCount {
    return [_parts count];
}

-(ModelPart*) partForIndex:(NSInteger)index {
    if(_parts == nil || index > _parts.count - 1) {
        return nil;
    }
    return _parts[index];
}

- (void)revertAll {
    auto glc = (GLViewController*)self.glController;
    auto viewer = [glc viewer];
    for(ModelPart* p : _parts) {
        p.aligned = NO;
        [glc setPrimitiveTransparent:_aisObjectsMap[(int)p.identity] transparent:true];
    }
    
    viewer->AisContext()->ClearSelected(Standard_True);
    viewer->AisContext()->UpdateCurrentViewer();
    
    if(_guidedControllerDelegate && [_guidedControllerDelegate respondsToSelector:@selector(didPartsUpdated)]) {
        [_guidedControllerDelegate didPartsUpdated];
    }

}

- (void)didSelectPart:(ModelPart*_Nonnull)part {
    auto glc = (GLViewController*)self.glController;
    auto context = [glc viewer]->AisContext();
    
    for (auto partInner : _aisObjectsMap) {
        if(_parts[partInner.first - 1].aligned) {
            continue;
        }
        if (partInner.first != (int)part.identity) {
            context->Erase(partInner.second, Standard_False);
        } else {
            context->Display(partInner.second, AIS_Shaded, (Standard_Integer)-1, Standard_False);
            [glc setPrimitiveTransparent:partInner.second transparent:true];
        }
    }
    context->UpdateCurrentViewer();
}

@end
