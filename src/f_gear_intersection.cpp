
#include <iostream>
#include <vector>
#include <nlopt.hpp>

#include "../include/gear_design/f_gear.hpp"
#include "../include/gear_design/sigbase_f_gear.hpp"


namespace gear_design{

/*==========< memo >============*
|  // defined at f_gear.hpp     |
|                               |
|  struct GearParamFgear {      |
|      double radius;  // mm    |      
|      double theta;   // rad   |
|      double quad_a;           |
|      double quad_b;           |
|                               |
|  };                           |
*==============================*/

// function name correct ???????????????
double obj_gearprofile1_gearprofile2_intersection_nlopt(
                                                        const std::vector<double>& input_vec,
                                                        std::vector<double>& grad, 
                                                        void* f_data) {
    gear_design::GearParamFgear* params = static_cast<gear_design::GearParamFgear*>(f_data);

    /* rename and cache */
    double radius = params -> radius;
    double theta = params -> theta;
    double cos = std::cos(theta);
    double sin = std::sin(theta);
    double x_base = params -> x_base;      // input 
    double y_base = input_vec[0];        // y_base is 1st-variable
    double x_gear = input_vec[1];        // y_base is 2nd-variable
    double y_gear = gear_design::siggear_f_gearprofile(x_gear);

    /* output: objective function */
    double res_1plus2 = -1. * x_base - y_base + x_gear * (sin + cos) - y_gear * (sin + cos) - radius * (sin - cos);

    return res_1plus2;
}


double calc_gearprofile1_gearprofile2_intersection(double x_base, double radius, double theta) {
    /*
        1st step: plot obj function shape
        2nd step: revize obj function shape
        3rd step:
    */

    // nlopt::opt obj_intersection(nlopt::LN_COBYLA, 2);
    // obj_intersection.set_min_objective(obj_gearprofile1_gearprofile2_intersection_nlopt, dammy, (void*)params);


    return -1;
}


}   // namespace gear_design


