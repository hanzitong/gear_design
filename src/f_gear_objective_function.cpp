 
#include <iostream>
#include <vector>
#include <nlopt.hpp>

#include "../include/gear_design/f_gear.hpp"
#include "../include/gear_design/f_gear_objective_function.hpp"


namespace gear_design{

/*==========< memo >================================*
|  // defined at f_gear_objective_function.hpp      |
|                                                   |
|  struct GearParamFgear {                          |
|      double radius;  // mm                        |      
|      double theta;   // rad                       |
|      double x_base;                               |
|      double x_gear;                               |
|      double y_base;                               |
|      double y_gear;                               |
|                                                   |
|  };                                               |
*==================================================*/

double obj_calc_siggear_x_from_xbase(   // optimization step 1 in calc_sigbase_y_gear_from_xbase()
                                     const std::vector<double>& opt_vec,
                                     std::vector<double>& grad,
                                     void* f_data
                                    )
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

    /* objective function for calculating x_gear */
    double res = x_base - x_gear * cos + y_gear * sin + radius * sin;   // equaton 1 in paper


    // return res;         // for using root finding algorithm
    return res * res;   // for using optimization algorithm
}


// double obj_calc_sigbase_y_from_xgear(   // optimization step 2 in calc_sigbase_y_gear_from_xbase()
//                                      const std::vector<double>& opt_vec,
//                                      std::vector<double>& grad,
//                                      void* f_data
//                                     )
// {
//     /* cast parameters from input struct (It's nlopt manner) */
//     gear_design::GearParamFgear* params = static_cast<gear_design::GearParamFgear*>(f_data);

//     /* rename and cache */
//     double radius = params -> radius;
//     double theta = params -> theta;
//     double cos = std::cos(theta);
//     double sin = std::sin(theta);
//     double x_gear = params -> x_gear;
//     double y_gear = gear_design::siggear_f_gearprofile(x_gear);
//     double y_base = params -> y_base;

//     /* objective function (for calculating y_base)*/
//     double res = y_base - x_gear * sin + y_gear * cos + radius * cos;   // equation 2 in paper

//     return res * res;
// }




// double obj_calc_sigbase_y_from_xgear_2variable(
//                                                const std::vector<double>& input_vec,
//                                                std::vector<double>& grad, 
//                                                void* f_data)
// {
//     /*-----------------------------------------------------------*
//     | 1 equation, 2 variables, it cannot determine solution..... |
//     *-----------------------------------------------------------*/
//     /* cast parameters from input structs (nlopt manner) */
//     gear_design::GearParamFgear* params = static_cast<gear_design::GearParamFgear*>(f_data);

//     /* rename & cache */
//     double radius = params -> radius;
//     double theta = params -> theta;
//     double cos = std::cos(theta);
//     double sin = std::sin(theta);
//     double x_base = params -> x_base;      // input 
//     double y_base = input_vec[0];        // y_base is 1st-variable
//     double x_gear = input_vec[1];        // y_base is 2nd-variable
//     double y_gear = gear_design::siggear_f_gearprofile(x_gear);

//     /* output: objective function */
//     double res_1plus2 = -1. * x_base - y_base + x_gear * (sin + cos) - y_gear * (sin + cos) - radius * (sin - cos);

//     return res_1plus2;
// }




}   // namespace gear_design


