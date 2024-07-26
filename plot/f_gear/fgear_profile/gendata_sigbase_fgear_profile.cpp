


#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include "../../include/gear_design/f_gear.hpp"
#include "../../include/gear_design/sigbase_f_gear.hpp"



int main() {
    const int N = 100;
    const double radius = 10.;
    const double theta1 = 45. / 180. * M_PI;
    // double theta2 = 35. / 180. * M_PI;

    const double siggear_x_range[2] = {-2., 2.};
    const double siggear_x_diff = (siggear_x_range[1] - siggear_x_range[0]) / N;
    std::vector<double> siggear_x_parameter(N, 0.);
    for (int i = 0; i < N; ++i) {
        siggear_x_parameter[i] = siggear_x_range[0] + siggear_x_diff * i;
    }

    std::vector<double> sigbase_x_output(N, 0.);
    std::vector<double> sigbase_y_output(N, 0.);
    for (int i = 0; i < N; ++i) {
        sigbase_x_output[i] = \
            gear_design::calc_sigbase_gearprofile_fgear_x_coordinate(siggear_x_parameter[i], radius, theta1);
        sigbase_y_output[i] = \
            gear_design::calc_sigbase_gearprofile_fgear_y_coordinate(siggear_x_parameter[i], radius, theta1);
    }

    std::ofstream dataFile;
    dataFile.open("data_fgear.dat");
    // put data into file
    for (int i = 0; i < N; ++i) {
        dataFile << sigbase_x_output[i] << ' ' << sigbase_y_output[i] << "\n";
    }
    dataFile.close();


    return 0;
}

