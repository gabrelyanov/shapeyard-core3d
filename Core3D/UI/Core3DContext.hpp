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
};
#endif /* Core3DContext_hpp */
