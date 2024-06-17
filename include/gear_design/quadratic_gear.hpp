
#ifndef GEAR_DESIGN_QUADRATIC_GEAR_HPP
#define GEAR_DESIGN_QUADRATIC_GEAR_HPP

#include <iostream>
#include <vector>
#include <cmath>


double calc_gearshape_outline_quadratic(const double x, void* gearparam);

struct GearParam {
    double module;
    double diameter;
};

