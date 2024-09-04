

#include <iostream>
#include <cmath>
// #include <gear_design/quadratic_gear.hpp>
#include <gear_design/f_gear.hpp>


int main(const int argc, const char* argv[]) {
    double radius = 5.;
    double theta = 30 / 180 * M_PI;
    double x_gear = 0.;
    double y_gear = gear_design::siggear_f_gearprofile(x_gear);
    double siggear_x = 1.;
    // double sigbase_y = gear_design::calc_sigbase_y_gear_from_xbase(siggear_x, radius, theta);
    double sigbase_y = y_gear;

    // double y = gear_design::calc_sig0y_quadratic(2., &chank);
    // double y = gear_design::calc_sig0y_quadratic(0., &chank);

    std::cout << sigbase_y << std::endl;


    return 0;
}

