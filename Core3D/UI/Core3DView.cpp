//
//  Core3dView.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 04.06.2024.
//

#include "Core3DView.hpp"
#include <GP_Quaternion.hxx>

Core3DView::Core3DView(const Handle(V3d_Viewer)& theViewer, const V3d_TypeOfView theType)
: V3d_View(theViewer, theType) {
    
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
	
    if (aCamera->ProjectionType() != Graphic3d_Camera::Projection_Perspective) {
		aCamera->SetProjectionType(Graphic3d_Camera::Projection_Perspective);
		aCamera->InvalidateProjection();
		aCamera->InvalidateOrientation();
		FitAll(0.2, Standard_False);
    }

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
