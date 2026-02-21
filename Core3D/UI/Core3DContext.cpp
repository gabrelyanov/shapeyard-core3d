//
//  Core3DContext.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 04.06.2024.
//

#include "Core3DContext.hpp"
#include "XCAFPrs_Style.hxx"
Core3DContext::Core3DContext(const Handle(V3d_Viewer)& MainViewer)
: AIS_InteractiveContext(MainViewer){
    
}

void Core3DContext::ApplyDefaultMaterial(const Handle(AIS_InteractiveObject)& obj) {
    obj->SetMaterial(Graphic3d_NameOfMaterial_ShinyPlastified);
    obj->SetColor(Quantity_NOC_GRAY80);
}

void Core3DContext::Display (const Handle(AIS_InteractiveObject)& theIObj,
                             const Standard_Boolean               theToUpdateViewer) {  
    AIS_InteractiveContext::Display(theIObj, theToUpdateViewer);
    
}

void Core3DContext::Display (const Handle(AIS_InteractiveObject)& theIObj,
                             const Standard_Integer               theDispMode,
                             const Standard_Integer               theSelectionMode,
                             const Standard_Boolean               theToUpdateViewer,
                             const PrsMgr_DisplayStatus           theDispStatus) {
    AIS_InteractiveContext::Display(theIObj, theDispMode, theSelectionMode, theToUpdateViewer, theDispStatus);
}

