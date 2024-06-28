
#include <iostream>
#include <cmath>
#include <vector>

#include "../include/gear_design/quadratic_gear.hpp"
// #include <gear_design/quadratic_gear.hpp>
// #include <gear_design.hpp>


int main(const int argc, const char* argv[]) {
    gear_design::GearParam chank = {10., 20.};

    double y = gear_design::calc_gearshape_outline_quadratic(0, &chank);

    std::cout << y << std::endl;



    return 0;
}

