


#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "../../../include/gear_design/f_gear.hpp"
#include "../../../include/gear_design/sigbase_f_gear.hpp"


int main() {
    /* basic declaration */
    std::vector<double> x_data(10000, 0.);
    std::vector<double> y_data(10000, 0.);
    std::vector<double> dammy(10, 0.);

    /* prepare the plot range of x-axis */
    double radius = 14.;
    double theta = 100. / 180 * M_PI;
    double range_start = -10.;
    double range_delta = 0.3;   // maybe need to avoid x=0

    /* prepare parameters which objective function needs */
    gear_design::GearParamFgear params(radius, theta, 0., 0.);
    std::vector<double> x_ini = {0., 0.};

    /* set plot range x and calculate y (both are on sigbase) */
    for (int i = 0; i < x_data.size(); ++i) {
        x_ini[0] = range_start + range_delta * i;
        x_data[i] = x_ini[0];
        y_data[i] = \
            gear_design::obj_gearprofile1_gearprofile2_intersection_nlopt(x_ini, dammy, (void*)&params);
    }

    // for (int i = 0; i < y_data.size(); ++i) {
    //     y_data[i] = gear_design::obj_gearprofile1_gearprofile2_intersection_nlopt(x_data, dammy, (void*)&params);
    // }

    /* put x,y-data into a record file */
    std::ofstream file;
    file.open("data/data_objfunc.dat");
    file << "#x y\n";
    for (int i = 0; i < 100; ++i) {
        file << x_data[i] << ' ' << y_data[i] << "\n";
        // if (i % 10 == 0) file << "\n";
    }
    file.close();


    return 0;
}


