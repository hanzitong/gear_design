 
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
// it seems NOT CORRECT (need to change in the future)
double obj_gearprofile1_gearprofile2_intersection_nlopt(
                                                        const std::vector<double>& input_vec,
                                                        std::vector<double>& grad, 
                                                        void* f_data) {
    /* cast parameters from input structs (nlopt manner) */
    gear_design::GearParamFgear* params = static_cast<gear_design::GearParamFgear*>(f_data);

    /* rename & cache */
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


// double calc_sigbase_y_base_objfunc_shape(
//                                          double& x_gear,    // variable to optmize 
//                                          double& x_base,    // input of sigbase_y_base
//                                          double& radius,    // parameter
//                                          double& theta) {   // parameter
double calc_sigbase_y_base_objfunc_shape(
                                         const std::vector<double>& opt_vec,
                                         std::vector<double>& grad,
                                         void* f_data)
{
    /* cast parameters from input structs (nlopt manner) */
    gear_design::GearParamFgear* params = static_cast<gear_design::GearParamFgear*>(f_data);

    /* rename & cache */
    double radius = params -> radius;
    double theta = params -> theta;
    double cos = std::cos(theta);
    double sin = std::sin(theta);
    double x_base = params -> x_base;
    double x_gear = opt_vec[0];
    double y_gear = siggear_f_gearprofile(x_gear);

    double res = x_base - x_gear * cos + y_gear * sin + radius * sin;


    return res;         // for using root finding algorithm
    // return res * res;   // for using optimization algorithm
}


}   // namespace gear_design


