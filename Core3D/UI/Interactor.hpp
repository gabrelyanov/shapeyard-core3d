//
//  Interactor.hpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 19.04.2024.
//

#ifndef Interactor_hpp
#define Interactor_hpp

#include "Core3DContext.hpp"
#include "Core3DView.hpp"
#include "OcctDocument.h"

namespace core3d {
    class Interactor {
    public:
        Interactor() = delete;
        Interactor(Handle(Core3DContext), Handle(Core3DView), Handle(OcctDocument));
    
    protected:
        Handle(Core3DContext)  myContext; //!< interactive context containing displayed objects
        Handle(Core3DView)     myView;    //!< main view
        Handle(OcctDocument)   myDoc;    //!< main view
    };
}
#endif /* Interactor_hpp */
