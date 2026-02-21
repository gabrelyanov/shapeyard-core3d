//
//  GuidedObjectsMatcher.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 25.09.2024.
//

#include "GuidedObjectsMatcher.hpp"

GuidedObjectsMatcher::GuidedObjectsMatcher(Handle(AIS_InteractiveObject) first
                                           , Handle(AIS_InteractiveObject) second
                                           , const int axisIndex
                                           , const float tolerance)
: _first(first)
, _second(second)
, _transformTolerance(tolerance){

//    auto locA = _first->LocalTransformation().TranslationPart();
//    auto locB = _second->LocalTransformation().TranslationPart();

    Bnd_Box bboxA;
    _first->BoundingBox(bboxA);
    
    switch (axisIndex) {
        case 0:
            _axisSideLenght = bboxA.CornerMax().X() - bboxA.CornerMin().X();
            break;
        case 1:
            _axisSideLenght = bboxA.CornerMax().Y() - bboxA.CornerMin().Y();
            break;
        case 2:
            _axisSideLenght = bboxA.CornerMax().Z() - bboxA.CornerMin().Z();
            break;
        default:
            _axisSideLenght = bboxA.CornerMax().X() - bboxA.CornerMin().X();
            break;
    }

	if (_axisSideLenght < _minAxisSideLenght)
		_axisSideLenght = _minAxisSideLenght;
    
}

std::shared_ptr<GuidedObjectsMatcher> GuidedObjectsMatcher::Make(Handle(AIS_InteractiveObject) first
                                                                , Handle(AIS_InteractiveObject) second
                                                                , const int axisIndex
                                                                , const float tolerance) {
    return std::make_shared<GuidedObjectsMatcher>(first, second, axisIndex, tolerance);
}


void GuidedObjectsMatcher::matchTransforms(std::function<void()> matched) {
    Bnd_Box bboxA;
    _first->BoundingBox(bboxA);
    
    Bnd_Box bboxB;
    _second->BoundingBox(bboxB);

    auto dist = bboxA.CornerMax().Distance(bboxB.CornerMax());

    if(dist < _transformTolerance * _axisSideLenght) {
        matched();
    }
}
