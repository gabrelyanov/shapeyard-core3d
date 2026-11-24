//
//  Core3dView.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 04.06.2024.
//

#include "Core3DView.hpp"
#include <Aspect_DisplayConnection.hxx>
#include <GP_Quaternion.hxx>
#include <Bnd_Box.hxx>
#include <gp_Ax3.hxx>
#include <V3d_Viewer.hxx>
#include <Graphic3d_CView.hxx>
#include <Standard_Failure.hxx>
#include <algorithm>
#include <cmath>

Core3DView::Core3DView(const Handle(V3d_Viewer)& theViewer, const V3d_TypeOfView theType)
: V3d_View(theViewer, theType) {
    
}

void Core3DView::Redraw() const
{
    if (myIsRenderingFrame) {
        V3d_View::Redraw();
        return;
    }
    InvalidateAndRequestFrame();
}

void Core3DView::RedrawImmediate() const
{
    if (myIsRenderingFrame) {
        V3d_View::RedrawImmediate();
        return;
    }
    InvalidateAndRequestFrame();
}

void Core3DView::InvalidateAndRequestFrame() const
{
    Invalidate();
    if (!Window().IsNull()) {
        // Core3DCocoa_Window translates this renderer-neutral OCCT invalidation
        // into GLView's main-thread display-link request. This keeps every
        // OCCT redraw entry point inside the single scheduled draw boundary.
        Window()->InvalidateContent(Handle(Aspect_DisplayConnection)());
    }
}

#if DEBUG
static bool gDebugLegacyNearPlane = false;
void Core3DView::debugSetLegacyNearPlane(bool legacy) { gDebugLegacyNearPlane = legacy; }
static bool gDebugNearPlaneIgnoresFloor = false;
void Core3DView::debugSetNearPlaneIgnoresFloor(bool ignore) { gDebugNearPlaneIgnoresFloor = ignore; }
#endif

// Round 10 (founder items a and d; R10P-S7): OCCT's auto depth fit also measures the floor grid,
// which reaches behind the eye, so in perspective it clamps the near plane to about far / 10^4
// (0.07 mm with far 610 mm). Depth resolution then falls to roughly distance^2 / (near * 2^24):
// ~0.06 mm at 25 cm, so a part whose face lies a few hundredths of a millimetre in front of
// another (a fin on a body, the top of a grip under the body) loses whole triangles to the face
// behind it and their meeting line turns saw-toothed. For each drawn frame the near plane is lifted
// to half the distance of the nearest corner of the finite model's bounding box (the grid is an
// infinite structure and is left out), never nearer than OCCT's own value and never past the far
// plane, nor past half the camera's focus distance. When the eye is inside or beside that box
// (nearest corner not in front), OCCT's near stays.
// Only the drawing uses the lifted value: OCCT's own range is put back after the frame, so taps,
// picks and every other reader of the camera see exactly what they saw before.
bool Core3DView::FitDepthRangeForFrame(Standard_Real& theOcctNear, Standard_Real& theOcctFar) const
{
    const Handle(Graphic3d_Camera)& aCamera = Camera();
    if (aCamera.IsNull() || View().IsNull()) return false;
    ZFitAll(AutoZFitScaleFactor());
    theOcctNear = aCamera->ZNear();
    theOcctFar = aCamera->ZFar();
#if DEBUG
    if (gDebugLegacyNearPlane) return true;
#endif
    if (aCamera->IsOrthographic() || !(theOcctNear > 0.0) || !(theOcctFar > theOcctNear)) return true;
    const Bnd_Box aModel = View()->MinMaxValues(Standard_False);
    if (aModel.IsVoid() || aModel.IsWhole() || aModel.IsOpen()) return true;
    Standard_Real aBox[6];
    aModel.Get(aBox[0], aBox[1], aBox[2], aBox[3], aBox[4], aBox[5]);
    const gp_XYZ& aScale = aCamera->AxialScale();
    const gp_XYZ anEye = aCamera->Eye().XYZ();
    const gp_XYZ aForward = aCamera->Direction().XYZ();
    Standard_Real aNearest = RealLast();
    for (int aCorner = 0; aCorner < 8; ++aCorner) {
        const gp_XYZ aPoint(aBox[(aCorner & 1) ? 3 : 0] * aScale.X(),
                            aBox[(aCorner & 2) ? 4 : 1] * aScale.Y(),
                            aBox[(aCorner & 4) ? 5 : 2] * aScale.Z());
        aNearest = std::min(aNearest, (aPoint - anEye).Dot(aForward));
    }
    if (!(aNearest > 0.0) || !std::isfinite(aNearest)) return true;
    // Fable review 22 P2: the floor grid is not in the model's box, but the part of it the camera
    // sees must not be cut by the lifted near plane (a low camera sees the floor in front of the model
    // along the bottom of the screen). View depth is linear on the grid plane, so its nearest visible
    // point lies on one of the four corner rays of the view's frustum: each corner ray that meets the
    // plane in front of the eye counts with the view depth of that meeting point.
    bool aCountFloor = true;
#if DEBUG
    aCountFloor = !gDebugNearPlaneIgnoresFloor;
#endif
    if (aCountFloor && !Viewer().IsNull() && Viewer()->IsGridActive()) {
        const gp_Ax3& aPlane = Viewer()->PrivilegedPlane();
        const gp_XYZ aNormal = aPlane.Direction().XYZ();
        const gp_XYZ anOrigin = aPlane.Location().XYZ();
        const gp_XYZ anUp = aCamera->Up().XYZ();
        const gp_XYZ aRight = aForward.Crossed(anUp).Normalized();
        const gp_XYZ aTrueUp = aRight.Crossed(aForward).Normalized();
        const Standard_Real aHalfY = std::tan(0.5 * aCamera->FOVy() * M_PI / 180.0);
        const Standard_Real aHalfX = aHalfY * aCamera->Aspect();
        const Standard_Real anEyeHeight = aNormal.Dot(anOrigin - anEye);
        for (int aCorner = 0; aCorner < 4; ++aCorner) {
            // A ray with view depth 1 per unit: its parameter at the plane is the view depth there.
            const gp_XYZ aRay = aForward + aRight * ((aCorner & 1) ? aHalfX : -aHalfX)
                                         + aTrueUp * ((aCorner & 2) ? aHalfY : -aHalfY);
            const Standard_Real aSlope = aNormal.Dot(aRay);
            if (std::abs(aSlope) < 1.0e-12) continue;
            const Standard_Real aDepth = anEyeHeight / aSlope;
            if (aDepth > 0.0 && std::isfinite(aDepth)) aNearest = std::min(aNearest, aDepth);
        }
        if (std::abs(anEyeHeight) < 1.0e-9) return true;   // eye on the floor plane: keep OCCT's near
    }
    // Also stay in front of the camera's focus, where OCCT draws the corner axes.
    const Standard_Real aLifted = std::min(0.5 * aNearest, 0.5 * aCamera->Distance());
    if (aLifted > theOcctNear && aLifted < 0.5 * theOcctFar) {
        aCamera->SetZRange(aLifted, theOcctFar);
    }
    return true;
}

void Core3DView::RenderFrame() const
{
    struct RenderingGuard {
        Standard_Boolean& flag;
        explicit RenderingGuard(Standard_Boolean& theFlag) : flag(theFlag) {
            flag = Standard_True;
        }
        ~RenderingGuard() { flag = Standard_False; }
    } aGuard(myIsRenderingFrame);
    Standard_Real anOcctNear = 0.0, anOcctFar = 0.0;
    bool aFitted = false;
    if (AutoZFitMode()) {
        try {
            aFitted = FitDepthRangeForFrame(anOcctNear, anOcctFar);
        } catch (const Standard_Failure&) {
            aFitted = false;
        }
    }
    if (!aFitted) {
        V3d_View::Redraw();
        return;
    }
    // This frame's depth range is already fitted: OCCT's AutoZFit inside Redraw would undo the lift.
    Core3DView* aSelf = const_cast<Core3DView*>(this);
    const Standard_Real aScale = AutoZFitScaleFactor();
    struct AutoZFitRestore {
        Core3DView* view; Standard_Real scale; Standard_Real zNear; Standard_Real zFar;
        ~AutoZFitRestore() {
            view->SetAutoZFitMode(Standard_True, scale);
            const Handle(Graphic3d_Camera)& aCamera = view->Camera();
            if (!aCamera.IsNull() && zNear > 0.0 && zFar > zNear
                && (aCamera->ZNear() != zNear || aCamera->ZFar() != zFar)) {
                aCamera->SetZRange(zNear, zFar);
            }
        }
    } aRestore{aSelf, aScale, anOcctNear, anOcctFar};
    aSelf->SetAutoZFitMode(Standard_False, aScale);
    V3d_View::Redraw();
}

void Core3DView::StartRotation(const Standard_Integer X,
                               const Standard_Integer Y,
                               const Standard_Real zRotationThreshold)
{
    sx = X; sy = Y;
    Standard_Real x,y;
    Size(x,y);
    rx = Standard_Real(Convert(x));
    ry = Standard_Real(Convert(y));
    myRotateGravity = GravityPoint();
    OrientedRotate (0.0, 0.0, 0.0,
            myRotateGravity.X(), myRotateGravity.Y(), myRotateGravity.Z(),
            Standard_True);
    myZRotation = Standard_False;
    if( zRotationThreshold > 0. ) {
        Standard_Real dx = Abs(sx - rx/2.);
        Standard_Real dy = Abs(sy - ry/2.);
        //  if( dx > rx/3. || dy > ry/3. ) myZRotation = Standard_True;
        Standard_Real dd = zRotationThreshold * (rx + ry)/2.;
        if( dx > dd || dy > dd ) myZRotation = Standard_True;
    }
    
}

void Core3DView::Rotation (const Standard_Integer X, const Standard_Integer Y) {
    if( rx == 0. || ry == 0. ) {
        StartRotation(X,Y,0.);
        return;
    }
    Standard_Real dx=0.,dy=0.,dz=0.;
    if( myZRotation ) {
        dz = atan2(Standard_Real(X)-rx/2., ry/2.-Standard_Real(Y)) -
        atan2(sx-rx/2.,ry/2.-sy);
    } else {
        dx = (Standard_Real(X) - sx) * M_PI / rx;
        dy = (sy - Standard_Real(Y)) * M_PI / ry;
    }
    
    OrientedRotate (dx, dy, dz,
            myRotateGravity.X(), myRotateGravity.Y(), myRotateGravity.Z(),
            Standard_False);

}

#define DEUXPI (2. * M_PI)

void Core3DView::OrientedRotate(const Standard_Real ax, const Standard_Real ay, const Standard_Real az,
                        const Standard_Real X, const Standard_Real Y, const Standard_Real Z, const Standard_Boolean Start)
{
    
    Handle(Graphic3d_Camera) aCamera = Camera();
	
    // Orbit preserves the chosen projection and framing. Switching between
    // parallel and perspective views belongs to the explicit View controls.

    if (Start)
    {
        myPointStart.SetValues(ax, ay);
        myRotateGravity.SetCoord (X, Y, Z);
        
        gp_Trsf aTrsf;
        aTrsf.SetTransformation (gp_Ax3 (myRotateGravity, aCamera->OrthogonalizedUp(), aCamera->Direction()),
                                 gp_Ax3 (myRotateGravity, gp::DZ(), gp::DX()));
        const gp_Quaternion aRot = aTrsf.GetRotation();
        aRot.GetEulerAngles (gp_YawPitchRoll, myRotateStartYawPitchRoll[0], myRotateStartYawPitchRoll[1], myRotateStartYawPitchRoll[2]);

        aTrsf.Invert();
        myCamStartOpToEye    = gp_Vec(myRotateGravity, aCamera->Eye()).Transformed (aTrsf);
        myCamStartOpToCenter = gp_Vec(myRotateGravity, aCamera->Center()).Transformed (aTrsf);
        myOrbitStartCamera = new Graphic3d_Camera(aCamera);
        // Touch-down only captures the orbit origin. Reconstructing Euler
        // angles here clamps a pole view and discards roll before a tap can
        // select the object at its displayed coordinate.
        return;
    }
    if (ax == 0.0 && ay == 0.0 && az == 0.0) {
        // A zero-motion sample, including dragging back to the exact origin,
        // preserves the complete starting camera rather than its Euler approximation.
        if (!myOrbitStartCamera.IsNull()) {
            aCamera->Copy(myOrbitStartCamera);
            ImmediateUpdate();
        }
        return;
    }

    Graphic3d_Vec2i aWinXY;
    Window()->Size (aWinXY.x(), aWinXY.y());
    double aYawAngleDelta   = -ax;
    double aPitchAngleDelta = -ay;
    double aPitchAngleNew = 0.0, aRoll = 0.0;
    const double aYawAngleNew = myRotateStartYawPitchRoll[0] + aYawAngleDelta;

    if (!View()->IsActiveXR())
    {
      aPitchAngleNew = Max (Min (myRotateStartYawPitchRoll[1] + aPitchAngleDelta, M_PI * 0.5 - M_PI / 180.0), -M_PI * 0.5 + M_PI / 180.0);
      aRoll = 0.0;
    }
    
    gp_Quaternion aRot;
    aRot.SetEulerAngles (gp_YawPitchRoll, aYawAngleNew, aPitchAngleNew, aRoll);
    gp_Trsf aTrsfRot;
    aTrsfRot.SetRotation (aRot);

    const gp_Dir aNewUp = gp::DZ().Transformed (aTrsfRot);
    aCamera->SetUp (aNewUp);
    aCamera->SetEyeAndCenter (myRotateGravity.XYZ() + myCamStartOpToEye.Transformed (aTrsfRot).XYZ(),
                              myRotateGravity.XYZ() + myCamStartOpToCenter.Transformed (aTrsfRot).XYZ());

    aCamera->OrthogonalizeUp();
	
    ImmediateUpdate();
}
