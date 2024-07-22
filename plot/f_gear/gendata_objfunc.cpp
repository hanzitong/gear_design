


#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "../../include/gear_design/f_gear.hpp"
#include "../../include/gear_design/sigbase_f_gear.hpp"


int main() {
    /* prepare the plot range of x-axis */
    std::vector<double> dammy(10, 0.);
    double radius = 14.;
    double theta = 100. / 180 * M_PI;
    gear_design::GearParamFgear params(radius, theta, 0., 0.);
    std::vector<double> x_vec(1000, 0.);
    std::vector<double> y_vec(1000, 0.);
    double range_start = -50.;
    double range_delta = 0.5;
    for (int i = 0; i < x_vec.size(); ++i) {
        x_vec[i] = range_start + range_delta * i;
        y_vec[i] = \
            obj_gearprofile1_gearprofile2_intersection_nlopt(x_vec[i], dammy, (void*)&params);
    }

    /* put data to a record file */
    std::ofstream file;
    file.open("data/data_objfunc.dat");
    for (int i = 0; i < 100; ++i) {
        file << x_vec[i] << ' ' << y_vec[i] << "\n";
        // if (i % 10 == 0) file << "\n";
    }
    file.close();


    return 0;
}


