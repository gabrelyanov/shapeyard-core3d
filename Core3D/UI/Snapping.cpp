//
//  Snapping.cpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 25.11.2024.
//


#include "Snapping.hpp"

struct SnappingHolder {
    double linear = 0;
    double angular = 0;
    double scaling = 0;
    gp_Pnt currentPosition{};
};

Snapping::Snapping()
: _impl(std::make_shared<SnappingHolder>()) {
    
}

void Snapping::set(const double linear, const double angular, const double scaling) {
    _impl->linear = linear;
    _impl->angular = angular;
    _impl->scaling = scaling;
}

std::optional<double> Snapping::getLinear() const {
    if(abs(_impl->linear) > std::numeric_limits<double>::epsilon()) {
        return _impl->linear;
    }
    return std::nullopt;
}

void Snapping::setLinear(const double val) {
    _impl->linear = val;
}

std::optional<double> Snapping::getAngular() const {
    if(abs(_impl->angular) > std::numeric_limits<double>::epsilon()) {
        return _impl->angular;
    }
    return std::nullopt;
}

void Snapping::setAngular(const double val) {
    _impl->angular = val;
}

std::optional<double> Snapping::getScaling() const {
    if(abs(_impl->scaling) > std::numeric_limits<double>::epsilon()) {
        return _impl->scaling;
    }
    return std::nullopt;
}

void Snapping::setScaling(const double val) {
    _impl->scaling = val;
}

void Snapping::setCurrentPosition(const gp_Pnt& val) {
    _impl->currentPosition = val;
}

gp_Pnt Snapping::getCurrentPosition() const {
    return _impl->currentPosition;
}


