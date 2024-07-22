

#include <iostream>
#include <cmath>
// #include <gear_design/quadratic_gear.hpp>
#include "../include/gear_design/quadratic_gear.hpp"
// #include <Eigen/Core>


namespace gear_design{


double calc_gearshape_outline_quadratic(const double& x, void* gearparam) {
    GearParamQuadratic* param = static_cast<GearParamQuadratic*>(gearparam);
    // double m = param -> module;
    // double r = param -> diameter;

    // if (m < 0 || d < 0) return m * d * x;
    return -1. * std::pow(x, 2.) + 10.;
}


double objfunc_calc_sig0y_quadratic() {
    

    return -1;
}


double calc_lower_bound_objfunc_calc_sig0y_quadratic() {


    return -1;
}


// double calc_sig0y_quadratic(const double& x, const double& radius, const double& theta, const double& quad_a, const double& quad_b) {
double calc_sig0y_quadratic(const double& x_input, void* chank) {
    GearParamQuadratic* params = static_cast<GearParamQuadratic*>(chank);
    double radius = params -> radius;
    double theta = params -> theta;
    double quad_a = params -> quad_a;
    double quad_b = params -> quad_b;
    double cos = std::cos(theta);
    double sin = std::sin(theta);
    // std::cout << "theta: " << theta << std::endl;

    double cons_D = x_input * cos + radius * cos;
    double A = quad_a * sin * sin;
    double B = -2. * quad_a * sin * cons_D - cos; 
    double C = quad_a * cons_D * cons_D + quad_b - radius * sin - x_input * sin;

    if (theta < M_PI / 2. | M_PI / 2. < theta) {
        // std::cout << "theta < M_PI / 2 < theta" << std::endl;
        double true_value = B * B - 4. * A * C;
        // std::cout << "A: " << A << std::endl;
        // std::cout << "true_value: " << true_value << std::endl;
        return (-1. * B + std::sqrt(B * B - 4. * A * C)) / (2. * A);        // return upper solution
    } else {    // theta == M_PI / 2
        // std::cout << "theta == M_PI / 2" << std::endl;
        return quad_a * x_input * x_input + quad_b + radius;
    }

    // return (-1. * B - std::sqrt(B * B - 4. * A * C)) / (2. * A);        // return lower solution
}


}       // namespace gear_design




