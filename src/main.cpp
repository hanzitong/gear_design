
#include <iostream>
#include <cmath>
#include <vector>

#include "../include/gear_design/quadratic_gear.hpp"



int main(const int argc, const char* argv[]) {
    GearParam chank = {10., 20.};

    double y = calc_gearshape_outline_shape_quadratic(0, &chank);

    std::cout << y << std::endl;




    return 0;
}

