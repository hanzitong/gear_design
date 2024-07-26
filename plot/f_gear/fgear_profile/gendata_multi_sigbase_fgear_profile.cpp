
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>

#include "../../include/gear_design/f_gear.hpp"


bool printer(const int N, const double radius, const double theta_deg) {
    /* condition setting */
    const double theta = theta_deg / 180. * M_PI;
    const double siggear_x_range[2] = {-2., 2.};
    const double siggear_x_diff = (siggear_x_range[1] - siggear_x_range[0]) / N;
    std::vector<double> siggear_x_parameter(N, 0.);
    for (int i = 0; i < N; ++i) {
        siggear_x_parameter[i] = siggear_x_range[0] + siggear_x_diff * i;
    }

    /* generate data */
    std::vector<double> sigbase_x_output(N, 0.);
    std::vector<double> sigbase_y_output(N, 0.);
    for (int i = 0; i < N; ++i) {
        sigbase_x_output[i] = \
            gear_design::calc_sigbase_f_gearprofile_x_coordinate(siggear_x_parameter[i], radius, theta);
        sigbase_y_output[i] = \
            gear_design::calc_sigbase_f_gearprofile_y_coordinate(siggear_x_parameter[i], radius, theta);
    }

    /* maninpulate file name */
    std::ofstream dataFile;
    std::stringstream ss;
    ss << "data/" << "data_fgear" << "_radius" << radius << "_theta" << theta_deg << ".dat";
    dataFile.open(ss.str());

    /* put data into file */
    for (int i = 0; i < N; ++i) {
        dataFile << sigbase_x_output[i] << ' ' << sigbase_y_output[i] << "\n";
    }
    dataFile.close();


    return true;
}


int main() {
    const int N = 100;
    const int profile_num = 10;
    const double radius = 4.;
    const double theta_deg = 80;
    const double deg_diff = 5;

    for (int i = 0; i <= profile_num; ++i) {
        printer(N, radius, theta_deg + deg_diff * (double)i);
    }


    return 0;
}
