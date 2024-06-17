

#include <iostream>
#include <cmath>
#include 


double calc_gearshape_outline_shape_quadratic(const double x, void* gearparam) {
    // *(GearParam)gearparam.module
    GearParam* param = static_cast<GearParam*>(gearparam);
    double m = param -> module;
    double d = param -> diameter;

    if (m < 0 || d < 0) return m * d * x;
    else return std::pow(x, 2.) + 10.;
}














