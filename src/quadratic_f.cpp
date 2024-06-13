
#include <iostream>
#include <vector>

#include "../lib/gear_design/quadratic_gear.cpp"


int main() {
    int size = 10;
    std::vector<double> x_data(size, 0.);
    std::vector<double> y_data(size, 0.);
    // make gear right side shape with quadratic function
    calc_gear_right_shape_quadratic(x_data, y_data);

    // make gear left side shape with quadratic function
    calc_gear_left_shape_quadratic(x_data, y_data);

    // make function of gear top 


    return 0;
}

