//
//  Core3dView.hpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 04.06.2024.
//

#ifndef Core3dView_hpp
#define Core3dView_hpp

#include <V3d_View.hxx>

DEFINE_STANDARD_HANDLE(Core3DView, Standard_Transient)

class Core3DView : public V3d_View {
public:
    Standard_EXPORT Core3DView(const Handle(V3d_Viewer)& theViewer,
                               const V3d_TypeOfView theType);

    //! Onscreen rendering is owned by GLView's CADisplayLink. OCCT may request
    //! redraws from many modeling APIs; these overrides convert those requests
    //! into invalidation until RenderFrame() opens the one legal draw boundary.
    Standard_EXPORT void Redraw() const Standard_OVERRIDE;
    Standard_EXPORT void RedrawImmediate() const Standard_OVERRIDE;
    Standard_EXPORT void RenderFrame() const;

    Standard_EXPORT void StartRotation(const Standard_Integer X,
                                       const Standard_Integer Y,
                                       const Standard_Real zRotationThreshold);
    
    Standard_EXPORT void Rotation (const Standard_Integer X,
                                   const Standard_Integer Y);
    
    Standard_EXPORT void OrientedRotate (const Standard_Real ax, const Standard_Real ay, const Standard_Real az,
                                const Standard_Real X, const Standard_Real Y, const Standard_Real Z, const Standard_Boolean Start);
    

private:
    void InvalidateAndRequestFrame() const;

    Graphic3d_Vec2 myPointStart;
    Standard_Boolean myZRotation = false;
    Standard_Integer sx = 0;
    Standard_Integer sy = 0;
    Standard_Real rx = 0.;
    Standard_Real ry = 0.;
    gp_Pnt myRotateGravity;
    Graphic3d_Vertex myGravityReferencePoint;
    Graphic3d_Vec3d myRotateStartYawPitchRoll;
    
    gp_Vec              myCamStartOpToCenter;
    gp_Vec              myCamStartOpToEye;
    mutable Standard_Boolean myIsRenderingFrame = Standard_False;
};
#endif /* Core3dView_hpp */
