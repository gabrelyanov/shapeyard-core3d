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
    //! Refine cylindrical face candidates without replacing selection owners.
    Standard_EXPORT AIS_StatusOfDetection MoveTo(
        Standard_Integer x, Standard_Integer y,
        const Handle(V3d_View)& view, Standard_Boolean redraw);
#ifdef DEBUG
    //! Deterministic test-only preselection publication. Production hover
    //! continues to be owned exclusively by AIS_InteractiveContext::MoveTo().
    Standard_Boolean DebugSetDetectedOwner(
        const Handle(SelectMgr_EntityOwner)& owner) noexcept;
#endif
};
#endif /* Core3DContext_hpp */
