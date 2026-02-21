//
//  CylinderDriver.hpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 24.07.2024.
//

#ifndef CylinderDriver_hpp
#define CylinderDriver_hpp

#include "BaseDriver.h"

#include <Standard_DefineHandle.hxx>
#include <TFunction_Logbook.hxx>

DEFINE_STANDARD_HANDLE(CylinderDriver, BaseDriver)

class CylinderDriver : public BaseDriver {
public:

    static const Standard_GUID& GetID();
    
    CylinderDriver();

    virtual Standard_Integer Execute(Handle(TFunction_Logbook)& log) const;

    DEFINE_STANDARD_RTTIEXT(CylinderDriver, BaseDriver)
};
#endif /* CylinderDriver_hpp */
