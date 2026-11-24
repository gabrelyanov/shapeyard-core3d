//
//  Core3DContext.hpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 04.06.2024.
//

#ifndef Core3DContext_hpp
#define Core3DContext_hpp

#include <AIS_InteractiveContext.hxx>

class Core3DContext : public AIS_InteractiveContext {
    
public:
	static constexpr Quantity_NameOfColor kDefaultObjectColor = Quantity_NOC_GRAY;
	
    Standard_EXPORT Core3DContext(const Handle(V3d_Viewer)& MainViewer);
    Standard_EXPORT void Display (const Handle(AIS_InteractiveObject)& theIObj,
                                  const Standard_Boolean               theToUpdateViewer);
    
    Standard_EXPORT void Display (const Handle(AIS_InteractiveObject)& theIObj,
                                  const Standard_Integer               theDispMode,
                                  const Standard_Integer               theSelectionMode,
                                  const Standard_Boolean               theToUpdateViewer,
                                  const PrsMgr_DisplayStatus           theDispStatus = PrsMgr_DisplayStatus_None);
    void ApplyDefaultMaterial(const Handle(AIS_InteractiveObject)& obj);
    //! Keep selection wireframes on real edges, including resized spline faces.
    Standard_EXPORT static void DisableSurfaceIsolines(const Handle(Prs3d_Drawer)& drawer);
    using AIS_InteractiveContext::MoveTo;
    //! Refine cylindrical face candidates without replacing selection owners,
    //! then pass a face seen exactly edge-on on to the face the ray meets.
    Standard_EXPORT AIS_StatusOfDetection MoveTo(
        Standard_Integer x, Standard_Integer y,
        const Handle(V3d_View)& view, Standard_Boolean redraw);
private:
    AIS_StatusOfDetection moveToRefiningCylinders(
        Standard_Integer x, Standard_Integer y,
        const Handle(V3d_View)& view, Standard_Boolean redraw);
    //! After MoveTo: when the first hit is a face seen exactly edge-on (OCCT
    //! gives it a false depth near 0), the next face that faces the camera
    //! becomes the detected one unless the exact ray meets the first part no
    //! farther away. Changes only which detected owner is current.
    void passEdgeOnFaceHit(const Handle(V3d_View)& view, Standard_Boolean redraw);
public:
    //! The edge-on rule for a walk over the selector's raw hit list (after a
    //! Pick without MoveTo): true when the hit at rank is a face seen exactly
    //! edge-on that the exact ray does not meet before the next face that
    //! faces the camera, so the walk goes past it.
    Standard_EXPORT bool IsEdgeOnHitPassedBy(const Handle(V3d_View)& view, Standard_Integer rank);
#ifdef DEBUG
    //! Deterministic test-only preselection publication. Production hover
    //! continues to be owned exclusively by AIS_InteractiveContext::MoveTo().
    Standard_Boolean DebugSetDetectedOwner(
        const Handle(SelectMgr_EntityOwner)& owner) noexcept;
    //! Fable review 17, finding 4: time of the edge-on rule's exact ray on
    //! one part of about faceCount faces (at most 4096); -1 if it failed.
    Standard_EXPORT static double DebugEdgeOnRayMilliseconds(int faceCount) noexcept;
#endif
};
#endif /* Core3DContext_hpp */
