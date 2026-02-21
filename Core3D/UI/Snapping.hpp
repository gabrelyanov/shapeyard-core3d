//
//  Snapping.hpp
//  Core3D
//
//  Created by Dmitry Sukhorukov on 25.11.2024.
//

#ifndef SNAPPING_HPP
#define SNAPPING_HPP

#include "SimpleSingleton.hpp"
#include <memory>
#include <optional>
#include "gp_Pnt.hxx"

class SnappingHolder;

class Snapping  :  public SimpleSingleton<Snapping> {
public:
    Snapping();
    
    ~Snapping() = default;
    
    void set(const double linear, const double angular, const double scaling);
    
    std::optional<double> getLinear() const;
    
    void setLinear(const double val);
    
    std::optional<double> getAngular() const;

    void setAngular(const double val);

    std::optional<double> getScaling() const;

    void setScaling(const double val);
    
    void setCurrentPosition(const gp_Pnt& val);
    
    gp_Pnt getCurrentPosition() const;
private:
    std::shared_ptr<SnappingHolder> _impl;
};

#endif //SNAPPING_HPP

