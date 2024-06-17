

#include <iostream>
#include <cmath>

#include "../include/gear_design/quadratic_gear.hpp"


namespace gear_design{


double calc_gearshape_outline_quadratic(const double x, void* gearparam) {
    // *(GearParam)gearparam.module
    
    GearParam* param = static_cast<GearParam*>(gearparam);
    double m = param -> module;
    double d = param -> diameter;

    // if (m < 0 || d < 0) return m * d * x;
    return std::pow(x, 2.) + 10.;
}


}       // namespace gear_design

