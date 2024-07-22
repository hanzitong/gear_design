

#include <iostream>
#include <cmath>
// #include <gear_design/quadratic_gear.hpp>
#include <gear_design/f_gear.hpp>


int main(const int argc, const char* argv[]) {
    // gear_design::GearParamQuadratic chank = gear_design::GearParamQuadratic(10., 34. / 180. * M_PI, 4., 12.);
    double x_test = 0.;
    double y_test = gear_design::siggear_gearprofile_fgear(x_test);
    double testes = gear_design::calc_sigbase_gearprofile_fgear_xy_vector();

    // double y = gear_design::calc_sig0y_quadratic(2., &chank);
    // double y = gear_design::calc_sig0y_quadratic(0., &chank);

    std::cout << y_test << std::endl;


    return 0;
}

