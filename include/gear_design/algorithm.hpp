

#ifndef GEAR_DESIGN_ALGORITHM_HPP
#define GEAR_DESIGN_ALGORITHM_HPP

#include <vector>
#include <cassert>
#include <cmath>

#include "gear_design/coordinates.hpp"
#include "gear_design/objfunctions.hpp"



double x_axis(const double& x) 
{
    return 0.;
}


double trapezoidal_integral(
    int num_split, 
    double start_x, 
    double end_x, 
    double(*calc_upper_yprofile)(double), 
    double (*calc_lower_yprofile)(double)
)
{
    assert(start_x < end_x);
    const double dh = (end_x - start_x) / static_cast<double>(num_split);
    std::vector<double> range_x(num_split + 1, 0.);
    std::vector<double> upper_yprofile(num_split + 1, 0.);
    std::vector<double> lower_yprofile(num_split + 1, 0.);

    for (int i = 0; i < num_split + 1; ++i) {
        range_x[i] = start_x + static_cast<double>(i) * dh;
        upper_yprofile[i] = ;
        lower_yprofile[i] = ;
    }

    for (int i = 0; i < num_split + 1; ++i) {
        now_side = 0.5 * dh * (calc_upper_yprofile() - calc_lower_yprofile());
        next_side = 0.5 * dh * (calc_upper_y);
        assert(now_side >= 0.);
        assert(next_side >= 0.);
    }

    for (int j = 0; j < num_split; ++j) {

    }


    return ;
}


double skiving_integral(
    int num_split,
    double start_x,
    double end_x,
    std::vector<std::vector<double>>& upper_yprofile_cache, // vector of up_y, for each x_point
    std::vector<std::vector<double>>& lower_yprofile_cache, // vector of low_y, for each x_point
    (*void)params       // radius, phi
)
{

    return trapezoidal_integral();
}



# endif


