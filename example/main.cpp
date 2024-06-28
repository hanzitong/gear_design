

#include <iostream>
#include <cmath>
#include <gear_design/quadratic_gear.hpp>


int main(const int argc, const char* argv[]) {
    gear_design::GearParamQuadratic chank = gear_design::GearParamQuadratic(10., 34. / 180. * M_PI, 4., 12.);

    double y = gear_design::calc_sig0y_quadratic(2., &chank);
    // double y = gear_design::calc_sig0y_quadratic(0., &chank);

    std::cout << y << std::endl;


    return 0;
}

