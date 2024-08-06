 
#include <iostream>
#include <vector>
#include <nlopt.hpp>

#include "../include/gear_design/f_gear.hpp"


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

double obj_calc_sigbase_y_from_xgear_2variable(
                                               const std::vector<double>& input_vec,
                                               std::vector<double>& grad, 
                                               void* f_data)
{
    /*-----------------------------------------------------------*
    | 1 equation, 2 variables, it cannot determine solution..... |
    *-----------------------------------------------------------*/
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


double obj_calc_sigbase_y_from_xgear_1variable(
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
    double x_base = params -> x_base;   // input
    double x_gear = opt_vec[0];         // variable to optimize
    double y_gear = siggear_f_gearprofile(x_gear);

    /* objective function */
    double res = x_base - x_gear * cos + y_gear * sin + radius * sin;


    // return res;         // for using root finding algorithm
    return res * res;   // for using optimization algorithm
}


}   // namespace gear_design


