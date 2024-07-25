
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
                                                        const std::vector<double>& x_base_input,
                                                        std::vector<double>& grad, 
                                                        void* f_data) {
    gear_design::GearParamFgear* params = static_cast<gear_design::GearParamFgear*>(f_data);
    double radius = params -> radius;
    double theta = params -> theta;

    double cos = std::cos(theta);
    double sin = std::sin(theta);
    // double y_base = gear_design::siggear_f_gearprofile(x_gear); // is y_base correct??????????
    double x_base = x_base_input[0];
    double y_base = gear_design::siggear_f_gearprofile(x_base);     // temporary change. not confident about whether its output is correct or not.
    double res = x_base * (cos + sin) - y_base * (sin + cos) - radius * sin + radius * cos - x_base - y_base;


    return res;
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


