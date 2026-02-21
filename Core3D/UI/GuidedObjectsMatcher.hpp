//
//  GuidedObjectsMatcher.hpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 25.09.2024.
//

#ifndef GuidedObjectsMatcher_hpp
#define GuidedObjectsMatcher_hpp

#include <stdio.h>
#include <AIS_InteractiveObject.hxx>
#include <functional>

class GuidedObjectsMatcher {
    float _transformTolerance = 0.3;
    float _axisSideLenght = 0.0f;
	float _minAxisSideLenght = 25.0f;
    Handle(AIS_InteractiveObject) _first;
    Handle(AIS_InteractiveObject) _second;
    int _tag = 0;
public:
    GuidedObjectsMatcher() = delete;
    GuidedObjectsMatcher(Handle(AIS_InteractiveObject) first
                         , Handle(AIS_InteractiveObject) second
                         , const int axisIndex
                         , const float tolerance);
    static std::shared_ptr<GuidedObjectsMatcher> Make(Handle(AIS_InteractiveObject) first
                                                     , Handle(AIS_InteractiveObject) second
                                                     , const int axisIndex
                                                     , const float tolerance);
    
    void matchTransforms(std::function<void()> matched);
    
    void setTag(const int theTag) {
        _tag = theTag;
    }
    
    const int tag() const {
        return _tag;
    }
    
    const Handle(AIS_InteractiveObject) first() const {
        return _first;
    }
    
     const Handle(AIS_InteractiveObject) second() const {
        return _second;
    }

};

#endif /* GuidedObjectsMatcher_hpp */
