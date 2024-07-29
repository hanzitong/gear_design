

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

#include "../../../include/gear_design/f_gear.hpp"
#include "../../../include/gear_design/sigbase_f_gear.hpp"


int main() {
    std::cout << "here!!!!!!!!!!!!" << std::endl;
    /* basic declaration */
    int N = 1000;
    std::vector<double> y_base_record(N, 0.);
    std::vector<double> x_gear_record(N, 0.);
    std::cout << "here!!!!!!!!!!!!" << std::endl;
    std::vector<std::pair<double, double>> g_record(N, {0., 0.});
    // double g_record[2000][2000];
    // for (int i = 0; i < 2000; ++i) {
    //     for (int j = 0; j < 2000; ++j) {
    //         g_record[i][j] = 0.;
    //     }
    // }
    std::vector<double> dammy(10, 0.);

    std::cout << "here!!!!!!!!!!!!" << std::endl;

    /* prepare the plot range of x-axis */
    double radius = 0.2;        // [m]
    double theta = 100. / 180 * M_PI;
    double y_base_range_start = 10.;
    double x_gear_range_start = 10.;
    double range_delta = 0.5;   // maybe need to avoid x=0

    /* prepare parameters which objective function needs */
    gear_design::GearParamFgear params(radius, theta, 2.);
    std::vector<double> initial_val = {0., 0.};   // y_base, x_gear
    
    /* set variable range y_base & x_gear */
    for (int i = 0; i < N; ++i) y_base_record[i] = y_base_range_start + range_delta * i;   // y_base
    for (int j = 0; j < N; ++j) x_gear_record[j] = x_gear_range_start + range_delta * j;   // x_gear

    /* calculate y_base for each point */
    for (int i = 0; i < N; ++i) {
        initial_val[0] = y_base_record[i];
        for (int j = 0; j < N; ++j) {
            initial_val[1] = x_gear_record[j];
            // g_record[i][j] = \
            //     gear_design::obj_gearprofile1_gearprofile2_intersection_nlopt(initial_val, dammy, (void*)&params);
            g_record[i][j] = 1.;
        }
    }

    /* put x,y-record into a record file */
    std::ofstream file;
    file.open("data/data_objfunc.dat");
    file << "# y_base x_gear g(y_base, x_gear)\n";
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            file << y_base_record[i] << ' ' << x_gear_record[j] << ' ' << g_record[i][j] << "\n";
        }
    }
    file.close();


    return 0;
}


