//
//  Interactor.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 19.04.2024.
//

#include "Interactor.hpp"

namespace core3d {
    Interactor::Interactor(Handle(Core3DContext) context, Handle(Core3DView) view, Handle(OcctDocument) doc)
        : myContext(context)
        , myView(view)
        , myDoc(doc) {
        
    }
}
