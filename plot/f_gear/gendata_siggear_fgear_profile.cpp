

#include <iostream>
#include <fstream>
#include <vector>
// #include <cassert>

// #include <gear_design/f_gear.hpp>
#include "../../include/gear_design/f_gear.hpp"


int main() {
    int N = 100;
    std::vector<double> x_input(N, 0.);
    std::vector<double> y_output(N, 0.);

    double x_range[2] = {-2., 2.};
    double x_diff = x_range[1] - x_range[0];
    for (int i = 0; i < N; ++i) {
        x_input[i] = x_range[0] + x_diff / N * i;
        y_output[i] = gear_design::siggear_gearprofile_fgear(x_input[i]);
    }

    std::ofstream dataFile;
    dataFile.open("data_fgear.dat");
    // put data into file
    for (int i = 0; i < N; ++i) {
        dataFile << x_input[i] << ' ' << y_output[i] << "\n";
    }
    dataFile.close();


    return 0;
}


