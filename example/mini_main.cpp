

#include <iostream>
#include <cmath>
#include <cassert>
// #include "../include/gear_design/coordinates.hpp"
// #include "../include/gear_design/objfunctions.hpp"
#include "gear_design/coordinates.hpp"
#include "gear_design/objfunctions.hpp"


int main(const int argc, const char* argv[]) {
    /* fixed condition */
    double sigbase_x = 0.;
    double radius = 50.;
    double phi = -30. / 180. * M_PI;

    double sigbase_y = \
        gear_design::calc_sigbase_yprofile_from_sigbase_x(sigbase_y, radius, phi, nullptr);


    std::cerr << sigbase_y << std::endl;


    return 0;
}

