


#include <iostream>
#include <cmath>
#include <vector>
#include <cassert>

double siggear_outline_fgear(const double x, void* gearparam) {
    // do static cast for gearparam here.
    double a = 10.; double b = 15.;     // example value
    // no plan to use struct pointer here...............!

    if (x <= -10.) {
        return a * x;
    } else if (10. <= x) {
        return -1. * a * x;
    } else if (-10 <= x && x <= 10) {
        return x * x + b;
    } else {
        return std::nan(" return nan here ");
    }
}


// for cauculate x coordinate of f-gear which needs siggear_x_param for parameter.
double calc_gearshape_outline_fgear_x_coordinate(const double siggear_x_param, void* gearparam) {
    // argument ==>> siggear_x_param, radius, theta, gearparam    is better ??
    double cos = std::cos(theta);
    double sin = std::sin(theta);

    return siggear_x_param * cos + siggear_outline_fgear(siggear_x_param) * sin - radius * cos;
}


// for cauculate y coordinate of f-gear which needs siggear_y_param for parameter.
double calc_gearshape_outline_fgear_y_coordinate(const double siggear_x_param, void* gearparam) {
    double cos = std::cos(theta);
    double sin = std::sin(theta);

    return -1. * siggear_x_param * sin + siggear_outline_fgear(siggear_x_param) * cos - radius * sin;
}


